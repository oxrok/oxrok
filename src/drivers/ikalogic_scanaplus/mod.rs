// Counterpart in sigrok:
//   libsigrok/src/hardware/ikalogic-scanaplus/api.c,
//   Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Daniel Elstner,
//   Gerhard Sittig, Martin Ling, Soeren Apel, Marc Schink
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! ikalogic-scanaplus: the IKALOGIC `ScanaPLUS`, a nine channel logic
//! analyzer sampling at a fixed 100 MHz: an FT232H (`0403:6014`, product
//! string `SCANAPLUS`) in synchronous FIFO mode in front of an FPGA that
//! streams run length compressed samples ([`protocol`]). No trigger, no
//! scan options; the sample and time limits.
//!
//! The scan is libftdi's `ftdi_usb_open_desc`: every `0403:6014` is
//! opened and its product string read (the language table, then the
//! string); the first whose string is `SCANAPLUS` is opened as libftdi
//! opens a device (claim port A, reset, 9600 Bd) and closed again. A
//! device whose string cannot be read ends the scan with libftdi's -8,
//! logged; none found (-3) is not logged. Opening does the same again,
//! then purges both buffers, resets the bit mode, sets sync FIFO mode
//! and a latency timer of 2 ms, asks for reads of 64 KiB (16 KiB
//! transfers, libftdi's limit on Linux) and reads the device id from
//! EEPROM words 16 and 17. A failed step closes the device; a close
//! that fails (the device is gone) is logged with libftdi's last error
//! string, as `ftdi_usb_close` leaves it. The acquisition writes the
//! FPGA's init and start, each pair or run of pairs a bulk transfer,
//! then reads. All of it as under sigrok-cli, checked with the emulator
//! (`tools/emu/ikalogic-scanaplus.sh`), with the differences of
//! [`protocol`] in the decoding and the limits. A limit set during an
//! acquisition counts from the next read, as in the C driver.
//!
//! As with `ftdi-la`, libftdi detaches the kernel's `ftdi_sio` from
//! port A and never attaches it again, while oxrok gives the port back
//! when it lets go of it: the scan's close and the device's close. On a
//! system where `ftdi_sio` binds the `ScanaPLUS`, it then probes port A
//! (reads the latency timer, sets it to 16 ms, and reads EEPROM words
//! `0x0d` and `0x0e`: `ctl_in 40 0a`, `ctl_out 40 09 0010`, `ctl_in 40
//! 90 0000 000d`, `ctl_in 40 90 0000 000e`) after the scan and after
//! the capture, and the next open detaches it again. The emulator's
//! cases do not show it: sigrok-cli runs first in each, and under
//! `--no-oracle` the harness unbinds the driver before oxrok runs.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    PacketSink, Result, Value, ValueList,
};
use crate::ftdi::BitMode;
use crate::ftdi::libftdi::{Context, FtdiError};
use crate::usb::Transport;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

/// `USB_VENDOR_ID`.
pub const VENDOR_ID: u16 = 0x0403;
/// `USB_DEVICE_ID`: the FT232H's own id.
pub const PRODUCT_ID: u16 = 0x6014;
/// `USB_IPRODUCT`: the product string libftdi looks for.
pub const PRODUCT: &str = "SCANAPLUS";
/// The channel names.
const CHANNELS: [&str; 9] = ["1", "2", "3", "4", "5", "6", "7", "8", "9"];

/// The driver.
pub struct ScanaPlusDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::LimitSamples, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
];

impl Driver for ScanaPlusDriver {
    fn id(&self) -> &'static str {
        "ikalogic-scanaplus"
    }

    fn long_name(&self) -> &'static str {
        "IKALOGIC ScanaPLUS"
    }

    fn scan_options(&self) -> &'static [Key] {
        &[]
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        if let Some((key, _)) = options.first() {
            return Err(Error::UnknownKey(key.id().to_owned()));
        }
        let ctx = match open_desc(None) {
            Ok(ctx) => ctx,
            // libftdi's "device not found" is not logged.
            Err(e) if e.code == -3 => return Ok(Vec::new()),
            Err(e) => {
                log::error!("Failed to open device ({}): {}", e.code, e.message);
                return Ok(Vec::new());
            }
        };
        close(Link { ctx, devid: [0; 3], last_error: ALL_FINE });
        Ok(vec![Box::new(ScanaPlusDevice::new(None))])
    }
}

/// libftdi's `ftdi_usb_open_desc(ftdic, 0x0403, 0x6014, "SCANAPLUS",
/// NULL)`: the first `0403:6014` whose product string is `SCANAPLUS`
/// (or `pending`, a transport already open), opened by
/// [`Context::open`].
///
/// # Errors
///
/// libftdi's: -12 when the devices cannot be listed, -4 when one cannot
/// be opened, -8 when its product string cannot be read (which ends the
/// search), -3 when none matches, and [`Context::open`]'s.
fn open_desc(pending: Option<Box<dyn Transport>>) -> std::result::Result<Context, FtdiError> {
    type Opener = Box<dyn FnOnce() -> Result<Box<dyn Transport>>>;
    let fail = |code, message| FtdiError { code, message };
    let mut candidates: Vec<Opener> = Vec::new();
    if let Some(t) = pending {
        candidates.push(Box::new(move || Ok(t)));
    } else {
        let devices = crate::usb::list_devices().map_err(|e| {
            log::debug!("listing USB devices: {e}");
            fail(-12, "libusb_get_device_list() failed")
        })?;
        for info in devices {
            if info.vendor_id == VENDOR_ID && info.product_id == PRODUCT_ID {
                candidates.push(Box::new(move || crate::usb::open(&info)));
            }
        }
    }
    for open in candidates {
        let mut t = open().map_err(|e| {
            log::debug!("opening the device: {e}");
            fail(-4, "usb_open() failed")
        })?;
        let index = t.string_indexes().map_or(0, |i| i[1]);
        let product = crate::usb::read_string_ascii(t.as_mut(), index).map_err(|e| {
            log::debug!("product string: {e}");
            fail(-8, "unable to fetch product description")
        })?;
        if product == PRODUCT {
            return Context::open(t);
        }
        log::trace!("Skipping a 0403:6014 with the product string {product:?}.");
    }
    Err(fail(-3, "device not found"))
}

/// The error string `ftdi_usb_open_dev` leaves when it succeeds; the
/// C driver logged "Failed to close FTDI device (-1): all fine." when
/// the device left right after the scan's open.
const ALL_FINE: &str = "all fine";

/// The open chip and what the driver keeps with it.
struct Link {
    ctx: Context,
    /// The device id read from the EEPROM.
    devid: [u8; 3],
    /// libftdi's error string: the message of the last call that
    /// failed, which `ftdi_usb_close` leaves as it is.
    last_error: &'static str,
}

impl Link {
    /// Keeps the error's message as libftdi keeps it.
    fn failed(&mut self, e: FtdiError) -> FtdiError {
        self.last_error = e.message;
        e
    }
}

/// `scanaplus_close()`: releases the chip, logging a failure with
/// libftdi's last error string.
fn close(link: Link) {
    let stale = link.last_error;
    if let Err(e) = link.ctx.close() {
        log::debug!("releasing port A: {e}");
        log::error!("Failed to close FTDI device (-1): {stale}.");
    }
}

/// `dev_open()` after the open: purge, bit mode reset, sync FIFO, the
/// latency timer, the read size, the device id. A failed step closes
/// the chip.
fn set_up(ctx: Context) -> Result<Link> {
    let mut link = Link { ctx, devid: [0; 3], last_error: ALL_FINE };
    match configure(&mut link) {
        Ok(()) => Ok(link),
        Err(e) => {
            close(link);
            Err(e)
        }
    }
}

fn configure(link: &mut Link) -> Result<()> {
    let r = link.ctx.tcioflush();
    r.map_err(|e| {
        log::error!("Failed to purge FTDI RX/TX buffers ({}): {}.", e.code, e.message);
        link.failed(e)
    })?;
    let r = link.ctx.set_bitmode(0xff, BitMode::Reset as u8);
    r.map_err(|e| {
        log::error!("Failed to reset the FTDI chip bitmode ({}): {}.", e.code, e.message);
        link.failed(e)
    })?;
    let r = link.ctx.set_bitmode(0xff, BitMode::SyncFifo as u8);
    r.map_err(|e| {
        log::error!("Failed to put FTDI chip into sync FIFO mode ({}): {}.", e.code, e.message);
        link.failed(e)
    })?;
    let r = link.ctx.set_latency_timer(2);
    r.map_err(|e| {
        log::error!("Failed to set FTDI latency timer ({}): {}.", e.code, e.message);
        link.failed(e)
    })?;
    link.ctx.set_read_chunksize(protocol::COMPRESSED_BUF_SIZE);
    // scanaplus_get_device_id()
    let mut words = [0u16; 2];
    for (i, addr) in [16u16, 17].into_iter().enumerate() {
        let r = link.ctx.read_eeprom_location(addr);
        words[i] = r.map_err(|e| {
            log::error!("Failed to read EEPROM index {addr} ({}): {}.", e.code, e.message);
            log::error!("Failed to get ScanaPLUS device ID: -1.");
            link.failed(e)
        })?;
    }
    link.devid = protocol::device_id(words[0], words[1]);
    let [a, b, c] = link.devid;
    log::debug!("Received ScanaPLUS device ID successfully: {a:02x} {b:02x} {c:02x}.");
    Ok(())
}

/// One `ScanaPLUS`.
pub struct ScanaPlusDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    limits: Arc<protocol::Limits>,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl ScanaPlusDevice {
    fn new(transport: Option<Box<dyn Transport>>) -> Self {
        ScanaPlusDevice {
            info: DeviceInfo {
                driver: "ikalogic-scanaplus",
                vendor: "IKALOGIC".to_owned(),
                model: "ScanaPLUS".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: CHANNELS
                .iter()
                .enumerate()
                .map(|(i, n)| Channel::new(i, ChannelKind::Logic, *n))
                .collect(),
            limits: Arc::default(),
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording),
    /// which [`Device::open`] opens as libftdi does: the product string
    /// first, then the chip.
    pub fn from_transport(transport: Box<dyn Transport>) -> Self {
        Self::new(Some(transport))
    }
}

impl Device for ScanaPlusDevice {
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
        match key {
            Key::Samplerate => Ok(Value::U64(protocol::SAMPLERATE)),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        // The acquisition reads the limits after every read.
        match key {
            Key::Samplerate => {
                if value.as_u64().ok_or_else(bad)? != protocol::SAMPLERATE {
                    log::error!("ScanaPLUS only supports samplerate = 100MHz.");
                    return Err(Error::arg("ScanaPLUS only supports samplerate = 100MHz"));
                }
            }
            Key::LimitMsec => {
                self.limits.msec.store(value.as_u64().ok_or_else(bad)?, Ordering::SeqCst);
            }
            Key::LimitSamples => {
                self.limits.samples.store(value.as_u64().ok_or_else(bad)?, Ordering::SeqCst);
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
            Key::Samplerate => Ok(ValueList::U64(vec![protocol::SAMPLERATE])),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("scanaplus lock").is_some() {
            return Ok(());
        }
        let ctx = open_desc(self.pending.take()).map_err(|e| {
            log::error!("Failed to open device ({}): {}", e.code, e.message);
            Error::from(e)
        })?;
        let link = set_up(ctx)?;
        *self.link.lock().expect("scanaplus lock") = Some(link);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(link) = self.link.lock().expect("scanaplus lock").take() {
            // As sigrok-cli, which ignores what sr_dev_close() returns.
            close(link);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let mut slot = self.link.lock().expect("scanaplus lock");
        let Some(link) = slot.as_mut() else {
            return Err(Error::device("device is not open"));
        };
        // scanaplus_init(), scanaplus_start_acquisition(): a failed
        // write ends the start, the chip stays open.
        let mut writes = protocol::init_writes(link.devid);
        writes.extend(protocol::start_writes(link.devid));
        if let Err(e) = protocol::write_all(&mut link.ctx, &writes) {
            return Err(link.failed(e).into());
        }
        let mut link = slot.take().expect("checked above");
        drop(slot);
        let limits = Arc::clone(&self.limits);
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("scanaplus acquisition".into())
            .spawn(move || {
                if let Err(e) = protocol::run(&mut link.ctx, &limits, &sink, &stop2) {
                    link.failed(e);
                }
                *slot.lock().expect("scanaplus lock") = Some(link);
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
    use std::fmt::Write as _;
    use std::sync::mpsc::sync_channel;

    /// The product string read, then `dev_open()` through to the id.
    const OPENED: &str = "# oxrok usb fixture 1\n\
        device vid=0403 pid=6014 version=0900 bus=0 addr=0 strings=1,2,0\n\
        ctl_in 00 06 0300 0000 4 -> 04030904\n\
        ctl_in 00 06 0302 0409 255 -> 14035300430041004e00410050004c0055005300\n\
        claim 0 -> ok\nmps 81 -> 512\n\
        ctl_out 40 00 0000 0001 - -> ok\nctl_out 40 03 04e2 0201 - -> ok\n\
        ctl_out 40 00 0001 0001 - -> ok\nctl_out 40 00 0002 0001 - -> ok\n\
        ctl_out 40 0b 00ff 0001 - -> ok\nctl_out 40 0b 40ff 0001 - -> ok\n\
        ctl_out 40 09 0002 0001 - -> ok\n\
        ctl_in 40 90 0000 0010 2 -> 1ca3\nctl_in 40 90 0000 0011 2 -> 5700\n";

    fn device(steps: &str) -> ScanaPlusDevice {
        let text = format!("{OPENED}{steps}");
        ScanaPlusDevice::from_transport(Box::new(Replay::from_text(&text).unwrap()))
    }

    fn writes() -> String {
        let id = [0x1c, 0x23, 0x57];
        let mut s = String::new();
        for w in protocol::init_writes(id).iter().chain(&protocol::start_writes(id)) {
            let _ = write!(s, "bulk_out 02 ");
            for b in w {
                let _ = write!(s, "{b:02x}");
            }
            let _ = writeln!(s, " -> {}", w.len());
        }
        s
    }

    /// A 16 KiB transfer of 32 packets, each 510 bytes of `pair`
    /// repeated after the status bytes.
    fn transfer(pair: [u8; 2]) -> String {
        let packet = format!("0160{}", format!("{:02x}{:02x}", pair[0], pair[1]).repeat(255));
        packet.repeat(32)
    }

    #[test]
    fn captures_after_the_dropped_64_kib() {
        // Five transfers fill the dropped read (5 * 16320 bytes), the
        // rest of the fifth starts the next one.
        let mut steps = writes();
        for _ in 0..5 {
            let _ = writeln!(steps, "bulk_in 81 16384 -> {}", transfer([0x02, 0x00]));
        }
        let _ = writeln!(steps, "release 0 -> ok");
        let mut dev = device(&steps);
        dev.open().unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(10), None).unwrap();
        let (tx, rx) = sync_channel(16);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        assert!(matches!(packets.first(), Some(Packet::Header { .. })));
        assert!(
            matches!(&packets[1], Packet::Logic(l) if l.data == [0; 20] && l.unit_size == 2),
            "{packets:?}"
        );
        assert!(matches!(packets.last(), Some(Packet::End)));
        dev.close().unwrap();
    }

    #[test]
    fn a_limit_set_during_a_capture_counts() {
        let mut steps = writes();
        for _ in 0..5 {
            let _ = writeln!(steps, "bulk_in 81 16384 -> {}", transfer([0x02, 0x00]));
        }
        let _ = writeln!(steps, "release 0 -> ok");
        let mut dev = device(&steps);
        dev.open().unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(10), None).unwrap();
        // No room in the channel: the thread waits on its header, before
        // its first read.
        let (tx, rx) = sync_channel(0);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        assert!(matches!(&packets[1], Packet::Logic(l) if l.data == [0; 8]), "{packets:?}");
        assert!(matches!(packets.last(), Some(Packet::End)));
        dev.close().unwrap();
    }

    #[test]
    fn a_failed_write_refuses_the_start() {
        let mut dev = device(
            "bulk_out 02 8841 -> 2\nbulk_out 02 89648a64 -> err disconnected\nrelease 0 -> ok\n",
        );
        dev.open().unwrap();
        let (tx, _rx) = sync_channel(16);
        assert!(dev.start(PacketSink::new(tx, 0)).is_err());
        assert!(dev.worker.is_none());
        dev.close().unwrap();
    }

    #[test]
    fn a_failed_open_closes_the_chip() {
        let text = OPENED.replace(
            "ctl_in 40 90 0000 0011 2 -> 5700\n",
            "ctl_in 40 90 0000 0011 2 -> err device \"endpoint stalled\"\nrelease 0 -> ok\n",
        );
        let mut dev = ScanaPlusDevice::from_transport(Box::new(Replay::from_text(&text).unwrap()));
        assert!(dev.open().is_err());
        assert!(dev.link.lock().unwrap().is_none());
    }

    #[test]
    fn only_100_mhz() {
        let mut dev = ScanaPlusDevice::new(None);
        assert!(dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).is_err());
        dev.set(&Key::Samplerate, &Value::U64(100_000_000), None).unwrap();
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        assert!(dev.get(&Key::LimitSamples, None).is_err());
        assert_eq!(dev.channels().len(), 9);
    }
}
