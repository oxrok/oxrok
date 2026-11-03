// Counterpart in sigrok:
//   libsigrok/src/hardware/ftdi-la/api.c,
//   Copyright (C) 2015 Sergey Alirzaev <zl29ah@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Gerhard Sittig, Paul,
//   Sergey Rzhevsky, Tom Matthews
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! ftdi-la: FTDI chips used as 8 channel logic analyzers, port A in
//! asynchronous bit-bang mode with every pin an input, sampled at a rate
//! the baud rate generator sets (3.6 kHz up to about 10 MHz). No
//! trigger.
//!
//! The chips, by USB id as `api.c` lists them: FT2232H (`0403:6010`),
//! the TIAO USB Multi-Protocol Adapter (an FT2232H at `0403:8a98`, its
//! channels named after its JTAG header), FT4232H (`0403:6011`), FT232R
//! (`0403:6001`, which a BM shares) and FT232H (`0403:6014`). Without
//! `conn=` the scan looks at the ids libftdi's `ftdi_usb_find_all`
//! looks at (`0403:6001`, `6010`, `6011`, `6014`, `6015`), so the TIAO
//! adapter needs `conn=`, and an FT230X (`6015`) is found but has no
//! entry. The vendor, model and serial number are the device's strings,
//! read as `ftdi_usb_get_strings` reads them: in that order, stopping at
//! the first one that is missing, a missing vendor "Generic" and a
//! missing model the chip's name.
//!
//! Opening is libftdi's open (reset, 9600 Bd), a purge of both buffers,
//! the bit mode reset and bit-bang mode; a sample rate is sent when it
//! is set, as a baud rate ([`protocol`]); the acquisition sets bit-bang
//! mode again and reads. The device sees the same requests as under
//! sigrok-cli, through [`crate::ftdi::libftdi`], checked with the
//! emulator (`tools/emu/ftdi-la.sh`), with one exception: libftdi
//! detaches the kernel's `ftdi_sio` from port A and never attaches it
//! again, while oxrok gives the port back when it closes the device.
//! `ftdi_sio` then binds port A again and probes it (reads its latency
//! timer and sets it to 16 ms), so after oxrok the serial port is
//! back, and the next open detaches it again. The emulator's cases do
//! not show it: sigrok-cli runs first in each, and oxrok finds the
//! port already free.
//!
//! Unlike libsigrok's driver, `conn` can be read (the C driver
//! advertises it but never sets the `sdi->conn` it reads, so it always
//! fails), the sample rate is kept in 64 bits (the C driver's
//! `uint32_t` turns 4295967296 Hz into 1 MHz), and a sample rate set
//! during an acquisition is refused (the C driver sends it between two
//! reads; here the acquisition thread owns the chip).

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    PacketSink, Result, Value, ValueList,
};
use crate::ftdi::BitMode;
use crate::ftdi::libftdi::Context;
use crate::usb::{Transport, UsbDeviceInfo};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

/// One chip the driver knows: `struct ftdi_chip_desc`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChipDesc {
    /// `idVendor`.
    pub vendor: u16,
    /// `idProduct`.
    pub product: u16,
    /// The sample rate divided by this is the baud rate to set.
    pub samplerate_div: u64,
    /// The channel names, port A's pins.
    pub channel_names: [&'static str; 8],
}

const ADBUS: [&str; 8] =
    ["ADBUS0", "ADBUS1", "ADBUS2", "ADBUS3", "ADBUS4", "ADBUS5", "ADBUS6", "ADBUS7"];

/// `chip_descs`, in the C order.
pub static CHIPS: [ChipDesc; 5] = [
    ChipDesc { vendor: 0x0403, product: 0x6010, samplerate_div: 20, channel_names: ADBUS },
    ChipDesc {
        vendor: 0x0403,
        product: 0x8a98,
        samplerate_div: 20,
        // The 20 pin JTAG header.
        channel_names: ["TCK", "TDI", "TDO", "TMS", "RST", "nTRST", "DBGRQ", "RTCK"],
    },
    ChipDesc { vendor: 0x0403, product: 0x6011, samplerate_div: 20, channel_names: ADBUS },
    ChipDesc {
        vendor: 0x0403,
        product: 0x6001,
        samplerate_div: 30,
        channel_names: ["TXD", "RXD", "RTS#", "CTS#", "DTR#", "DSR#", "DCD#", "RI#"],
    },
    ChipDesc { vendor: 0x0403, product: 0x6014, samplerate_div: 30, channel_names: ADBUS },
];

/// The ids `ftdi_usb_find_all(ftdic, &list, 0, 0)` lists.
const LIBFTDI_IDS: [u16; 5] = [0x6001, 0x6010, 0x6011, 0x6014, 0x6015];

/// `usb_str_maxlen`: the strings are cut to 31 characters.
const USB_STR_MAXLEN: usize = 32;

/// The chip of a USB id.
pub fn find_chip(vendor: u16, product: u16) -> Option<&'static ChipDesc> {
    CHIPS.iter().find(|c| c.vendor == vendor && c.product == product)
}

/// The model name `scan_device()` gives a chip whose product string is
/// missing.
pub fn default_model(product: u16) -> &'static str {
    match product {
        0x6001 => "FT232R",
        0x6010 => "FT2232H",
        0x6011 => "FT4232H",
        0x6014 => "FT232H",
        0x8a98 => "FT2232H-TUMPA",
        _ => "Unknown",
    }
}

/// Vendor, model and serial number from the strings
/// `ftdi_usb_get_strings` got before the first one it could not read:
/// none is "Generic" and the chip's name; no product the chip's name;
/// no serial number none. A string after a missing one is not read.
pub fn names(product: u16, strings: &[String]) -> (String, String, Option<String>) {
    let cut = |s: &String| s.chars().take(USB_STR_MAXLEN - 1).collect::<String>();
    let vendor = strings.first().map_or_else(|| "Generic".to_owned(), cut);
    let model = strings.get(1).map_or_else(|| default_model(product).to_owned(), cut);
    (vendor, model, strings.get(2).map(cut))
}

/// The driver.
pub struct FtdiLaDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::Conn, Access::GET),
];

impl Driver for FtdiLaDriver {
    fn id(&self) -> &'static str {
        "ftdi-la"
    }

    fn long_name(&self) -> &'static str {
        "FTDI LA"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::list_devices()? {
            // With conn= any device it names, else the ids libftdi
            // looks for; then the driver's own table.
            let listed = match &conn {
                Some(c) => info.matches_conn(c),
                None => info.vendor_id == 0x0403 && LIBFTDI_IDS.contains(&info.product_id),
            };
            if !listed {
                continue;
            }
            let Some(chip) = find_chip(info.vendor_id, info.product_id) else {
                log::trace!(
                    "Unsupported FTDI device 0x{:04x}:0x{:04x}.",
                    info.vendor_id,
                    info.product_id
                );
                continue;
            };
            let strings = match read_strings(&info) {
                Ok(s) => s,
                Err(e) => {
                    log::error!("Failed to get the FTDI strings: {e}");
                    continue;
                }
            };
            devices.push(Box::new(FtdiLaDevice::new(info, chip, &strings, None)));
        }
        Ok(devices)
    }
}

/// `ftdi_usb_get_strings`: opens the device and reads its manufacturer,
/// product and serial number strings, stopping at the first that fails
/// (index 0 fails without a transfer). The strings read are returned.
///
/// # Errors
///
/// When the device cannot be opened (libftdi's -4).
fn read_strings(info: &UsbDeviceInfo) -> Result<Vec<String>> {
    let mut t = crate::usb::open(info)?;
    let indexes = t.string_indexes().unwrap_or_default();
    let mut out = Vec::new();
    for (i, what) in indexes.into_iter().zip(["manufacturer", "product", "serial number"]) {
        match crate::usb::read_string_ascii(t.as_mut(), i) {
            Ok(s) => out.push(s),
            Err(e) => {
                log::debug!("The device lacks a {what} descriptor ({e}).");
                break;
            }
        }
    }
    Ok(out)
}

/// One FTDI chip.
pub struct FtdiLaDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    chip: &'static ChipDesc,
    channels: Vec<Channel>,
    samplerate: u64,
    limit_samples: u64,
    pending: Option<Box<dyn Transport>>,
    ctx: Arc<Mutex<Option<Context>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl FtdiLaDevice {
    fn new(
        usb: UsbDeviceInfo,
        chip: &'static ChipDesc,
        strings: &[String],
        transport: Option<Box<dyn Transport>>,
    ) -> Self {
        let (vendor, model, serial) = names(usb.product_id, strings);
        log::debug!("Found an FTDI device: {model}.");
        FtdiLaDevice {
            info: DeviceInfo {
                driver: "ftdi-la",
                vendor,
                model,
                version: String::new(),
                serial: serial.unwrap_or_default(),
                conn: Some(usb.conn()),
            },
            usb,
            chip,
            channels: chip
                .channel_names
                .iter()
                .enumerate()
                .map(|(i, n)| Channel::new(i, ChannelKind::Logic, *n))
                .collect(),
            samplerate: 0,
            limit_samples: 0,
            pending: transport,
            ctx: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording),
    /// its strings read from it first, as the scan does.
    ///
    /// # Errors
    ///
    /// When the transport's USB id is not one of [`CHIPS`].
    pub fn from_transport(mut transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        let chip = find_chip(usb.vendor_id, usb.product_id)
            .ok_or_else(|| Error::arg("not a chip ftdi-la knows"))?;
        let indexes = transport.string_indexes().unwrap_or_default();
        let mut strings = Vec::new();
        for i in indexes {
            match crate::usb::read_string_ascii(transport.as_mut(), i) {
                Ok(s) => strings.push(s),
                Err(_) => break,
            }
        }
        Ok(Self::new(usb, chip, &strings, Some(transport)))
    }

    /// `ftdi_la_set_samplerate()` on an open device.
    fn send_samplerate(ctx: &mut Context, chip: &ChipDesc, samplerate: u64) -> Result<()> {
        ctx.set_baudrate(protocol::baud_rate(samplerate, chip.samplerate_div)).map(|_| ()).map_err(
            |e| {
                log::error!("Failed to set baudrate ({samplerate}): {}.", e.message);
                Error::from(e)
            },
        )
    }
}

/// `dev_open()` after libftdi's open: purge, bit mode reset, bit-bang
/// with every pin an input.
fn set_up(t: Box<dyn Transport>) -> Result<Context> {
    let mut ctx = Context::open(t).map_err(|e| {
        log::error!("Failed to open device {e}");
        Error::from(e)
    })?;
    ctx.tcioflush().map_err(|e| {
        log::error!("Failed to purge FTDI RX/TX buffers {e}.");
        Error::from(e)
    })?;
    ctx.set_bitmode(0, BitMode::Reset as u8).map_err(|e| {
        log::error!("Failed to reset the FTDI chip bitmode {e}.");
        Error::from(e)
    })?;
    ctx.set_bitmode(0, BitMode::BitBang as u8).map_err(|e| {
        log::error!("Failed to put FTDI chip into bitbang mode {e}.");
        Error::from(e)
    })?;
    Ok(ctx)
}

impl Device for FtdiLaDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn enable_channel(&mut self, pos: usize, enabled: bool) -> Result<()> {
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        c.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        name.clone_into(&mut c.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::Samplerate => Value::U64(self.samplerate),
            Key::Conn => Value::Str(self.usb.conn()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => self.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::Samplerate => {
                let rate = value.as_u64().ok_or_else(bad)?;
                if rate < protocol::MIN_SAMPLERATE {
                    return Err(Error::arg("invalid samplerate"));
                }
                // Sent at once when open, else when opened; kept only
                // when the chip took it. During an acquisition the
                // thread holds the chip: refused, not kept unsent.
                let mut slot = self.ctx.lock().expect("ftdi-la lock");
                match slot.as_mut() {
                    Some(ctx) => Self::send_samplerate(ctx, self.chip, rate)?,
                    None if self.worker.is_some() => {
                        return Err(Error::NotSupported(
                            "changing the samplerate during an acquisition".into(),
                        ));
                    }
                    None => {}
                }
                drop(slot);
                self.samplerate = rate;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::Samplerate => Ok(ValueList::RangeStepU64 {
                min: protocol::MIN_SAMPLERATE,
                max: 10_000_000,
                step: 1,
            }),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn open(&mut self) -> Result<()> {
        if self.ctx.lock().expect("ftdi-la lock").is_some() {
            return Ok(());
        }
        let t = match self.pending.take() {
            Some(t) => t,
            None => crate::usb::open(&self.usb)?,
        };
        let mut ctx = set_up(t)?;
        if self.samplerate != 0 {
            Self::send_samplerate(&mut ctx, self.chip, self.samplerate)?;
        }
        *self.ctx.lock().expect("ftdi-la lock") = Some(ctx);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        drop(self.ctx.lock().expect("ftdi-la lock").take());
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let Some(mut ctx) = self.ctx.lock().expect("ftdi-la lock").take() else {
            return Err(Error::device("device is not open"));
        };
        // dev_acquisition_start() (api.c:461) ignores what this
        // returns and reads anyway.
        if let Err(e) = ctx.set_bitmode(0, BitMode::BitBang as u8) {
            log::debug!("Setting bit-bang mode again failed {e}; reading anyway.");
        }
        let limit = self.limit_samples;
        let slot = Arc::clone(&self.ctx);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("ftdi-la acquisition".into())
            .spawn(move || {
                protocol::run(&mut ctx, limit, &sink, &stop2);
                *slot.lock().expect("ftdi-la lock") = Some(ctx);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Packet;
    use crate::usb::record::Replay;
    use std::sync::mpsc::sync_channel;

    /// An FT2232H without strings through `dev_open`: libftdi's open
    /// (claim, reset, 9600 Bd), the purge, the bit mode reset,
    /// bit-bang.
    const OPENED: &str = "# oxrok usb fixture 1\n\
        device vid=0403 pid=6010 version=0700 bus=0 addr=0 strings=0,0,0\n\
        claim 0 -> ok\nmps 81 -> 512\n\
        ctl_out 40 00 0000 0001 - -> ok\nctl_out 40 03 04e2 0201 - -> ok\n\
        ctl_out 40 00 0001 0001 - -> ok\nctl_out 40 00 0002 0001 - -> ok\n\
        ctl_out 40 0b 0000 0001 - -> ok\nctl_out 40 0b 0100 0001 - -> ok\n";

    fn device(steps: &str) -> FtdiLaDevice {
        let text = format!("{OPENED}{steps}");
        FtdiLaDevice::from_transport(Box::new(Replay::from_text(&text).unwrap())).unwrap()
    }

    #[test]
    fn a_failed_bit_bang_at_the_start_still_reads() {
        // dev_acquisition_start() ignores ftdi_set_bitmode()'s result.
        let mut dev = device(
            "ctl_out 40 0b 0100 0001 - -> err device \"endpoint stalled\"\n\
             bulk_in 81 4096 -> 01600001020304050607080900\n",
        );
        dev.open().unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(10), None).unwrap();
        let (tx, rx) = sync_channel(16);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        assert!(matches!(packets.first(), Some(Packet::Header { .. })));
        assert!(
            matches!(&packets[1], Packet::Logic(l) if l.data == [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
            "{packets:?}"
        );
        assert!(matches!(packets.last(), Some(Packet::End)));
        dev.close().unwrap();
    }

    #[test]
    fn a_samplerate_during_an_acquisition_is_refused() {
        let mut dev = device(
            "ctl_out 40 0b 0100 0001 - -> ok\n\
             bulk_in 81 4096 -> 01600001020304050607080900\n\
             ctl_out 40 03 003c 0201 - -> ok\n",
        );
        dev.open().unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(10), None).unwrap();
        // No room in the channel: the thread waits on its header,
        // holding the chip.
        let (tx, rx) = sync_channel(0);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        let e = dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap_err();
        assert!(matches!(e, Error::NotSupported(_)), "{e:?}");
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(0), "not kept");
        assert_eq!(rx.iter().count(), 3, "header, samples, end");
        dev.stop().unwrap();
        // The chip is back: sent at once (1 MHz / 20, times 4).
        dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap();
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(1_000_000));
        dev.close().unwrap();
    }

    #[test]
    fn chips_and_names() {
        assert_eq!(find_chip(0x0403, 0x6001).unwrap().samplerate_div, 30);
        assert_eq!(find_chip(0x0403, 0x8a98).unwrap().channel_names[0], "TCK");
        assert!(find_chip(0x0403, 0x6015).is_none());
        let s = |v: &[&str]| v.iter().map(|x| (*x).to_owned()).collect::<Vec<_>>();
        assert_eq!(
            names(0x6010, &s(&["FTDI", "Dual RS232-HS"])),
            ("FTDI".into(), "Dual RS232-HS".into(), None)
        );
        assert_eq!(names(0x6010, &[]), ("Generic".into(), "FT2232H".into(), None));
        assert_eq!(names(0x6001, &s(&["FTDI"])), ("FTDI".into(), "FT232R".into(), None));
        assert_eq!(names(0x6015, &[]).1, "Unknown");
        let long = names(0x8a98, &s(&["TIAO", "TIAO USB Multi-Protocol Adapter X", "S"]));
        assert_eq!(long.1, "TIAO USB Multi-Protocol Adapter");
        assert_eq!(long.2.as_deref(), Some("S"));
    }
}
