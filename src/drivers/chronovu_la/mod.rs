// Counterpart in sigrok:
//   libsigrok/src/hardware/chronovu-la/api.c,
//   Copyright (C) 2011-2015 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Gerhard Sittig,
//   Daniel Elstner, Martin Ling, Romain Tartière, Soeren Apel, Marek Antoniak
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! chronovu-la: the `ChronoVu` LA8 (8 channels, 100 MHz) and LA16 (16
//! channels, 200 MHz), an FTDI chip in FIFO mode in front of the
//! analyzer's logic and 8 MiB of sample memory. The device samples
//! until its trigger, fills the memory, then sends all of it; the host
//! finds the trigger in the data ([`protocol`]).
//!
//! The scan looks at every device with FTDI's vendor id (0x0403; the
//! boards carry 0403:8867 or the "old" 0403:6001) and reads its product
//! and serial number strings, as `api.c` does: a product string of
//! `ChronoVu LA8` or `ChronoVu LA16` is the device, whatever the
//! product id. Opening is libftdi's `ftdi_usb_open_desc` on those ids
//! and that product string (the string read again, then libftdi's open:
//! reset, 9600 Bd), a purge of both buffers, RTS/CTS flow control and
//! 100 ms. A capture writes the command and reads the memory through
//! [`crate::ftdi::libftdi`], so the device sees the requests it sees
//! under sigrok-cli; `tools/emu/chronovu-la.sh` checks that with the
//! emulator. Like ftdi-la, oxrok gives port A back to the kernel's
//! `ftdi_sio` when it closes the device (libftdi leaves it detached).
//!
//! Unlike libsigrok's driver, the sample and time limits are honoured,
//! the LA16 trigger is placed at the sample that met it and edge
//! triggers are reported, a failed write ends in one sequencer reset
//! rather than a recursion, and the wait for the trigger is sized from
//! the rate; a stop resets the sequencer and closes the chip; and the
//! device opened is the one the scan found, where `ftdi_usb_open_desc`
//! opens the first with those ids and that product string, so of two
//! boards libsigrok opens the first for both. [`protocol`] lists the
//! others with the C lines.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    PacketSink, Result, Trigger, TriggerFeatures, Value, ValueList,
};
use crate::drivers::saleae_logic16::split_conn;
use crate::ftdi::libftdi::{Context, FtdiError};
use crate::usb::{Transport, UsbDeviceInfo};
use protocol::{Capture, Ftdi, Profile};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

/// `SCAN_EXPECTED_VENDOR`.
const VENDOR: u16 = 0x0403;
/// The strings are read into 64 byte buffers: 63 characters at most.
const STR_MAX: usize = 63;

/// The driver.
pub struct ChronovuLaDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::LimitMsec, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: false, set: true, list: true }),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
];

/// libusb's name of an error, as the scan's warnings print it.
fn libusb_error_name(e: &Error) -> &'static str {
    match crate::ftdi::libftdi::libusb_code(e) {
        -2 => "LIBUSB_ERROR_INVALID_PARAM",
        -4 => "LIBUSB_ERROR_NO_DEVICE",
        -6 => "LIBUSB_ERROR_BUSY",
        -7 => "LIBUSB_ERROR_TIMEOUT",
        -9 => "LIBUSB_ERROR_PIPE",
        -12 => "LIBUSB_ERROR_NOT_SUPPORTED",
        _ => "LIBUSB_ERROR_IO",
    }
}

/// A string descriptor as `libusb_get_string_descriptor_ascii` reads
/// it into the scan's 64 byte buffer; index 0 is the empty string.
fn read_string(t: &mut dyn Transport, index: u8) -> Result<String> {
    if index == 0 {
        return Ok(String::new());
    }
    let s = crate::usb::read_string_ascii(t, index)?;
    Ok(s.chars().take(STR_MAX).collect())
}

/// The product and serial number strings of an opened device, or the
/// warning the scan logs.
fn product_and_serial(t: &mut dyn Transport) -> std::result::Result<(String, String), String> {
    let idx = t.string_indexes().unwrap_or_default();
    let product = read_string(t, idx[1]).map_err(|e| {
        format!("Failed to get product string descriptor: {}.", libusb_error_name(&e))
    })?;
    let serial = read_string(t, idx[2]).map_err(|e| {
        format!("Failed to get serial number string descriptor: {}.", libusb_error_name(&e))
    })?;
    Ok((product, serial))
}

impl Driver for ChronovuLaDriver {
    fn id(&self) -> &'static str {
        "chronovu-la"
    }

    fn long_name(&self) -> &'static str {
        "ChronoVu LA8/LA16"
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
        let conn = match conn.as_deref().map(split_conn).transpose() {
            Ok(c) => c,
            Err(msg) => {
                // sr_usb_find() finds nothing, and the scan no device.
                log::error!("{msg}");
                return Ok(Vec::new());
            }
        };
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::list_devices()? {
            if conn.is_some_and(|c| !c.matches(&info)) || info.vendor_id != VENDOR {
                continue;
            }
            let Ok(mut t) = crate::usb::open(&info) else { continue };
            let (product, serial) = match product_and_serial(t.as_mut()) {
                Ok(s) => s,
                Err(warning) => {
                    log::warn!("{warning}");
                    continue;
                }
            };
            drop(t);
            let Some(prof) = Profile::of_product(&product) else { continue };
            log::debug!(
                "Found {product} ({:04x}:{:04x}, {}.{}, {}).",
                info.vendor_id,
                info.product_id,
                info.bus,
                info.address,
                info.conn()
            );
            devices.push(Box::new(ChronovuLaDevice::new(info, prof, serial, None)));
        }
        Ok(devices)
    }
}

/// An LA8 or LA16.
pub struct ChronovuLaDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    prof: &'static Profile,
    channels: Vec<Channel>,
    samplerate: u64,
    limit_samples: u64,
    limit_msec: u64,
    poll_interval: Duration,
    trigger: Option<Trigger>,
    pending: Option<Box<dyn Transport>>,
    ftdi: Arc<Mutex<Option<Ftdi>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl ChronovuLaDevice {
    fn new(
        usb: UsbDeviceInfo,
        prof: &'static Profile,
        serial: String,
        transport: Option<Box<dyn Transport>>,
    ) -> Self {
        ChronovuLaDevice {
            info: DeviceInfo {
                driver: "chronovu-la",
                vendor: "ChronoVu".into(),
                model: prof.name.into(),
                version: String::new(),
                serial,
                conn: Some(usb.conn()),
            },
            usb,
            prof,
            channels: (0..prof.num_channels)
                .map(|i| Channel::new(i, ChannelKind::Logic, i.to_string()))
                .collect(),
            samplerate: prof.max_samplerate,
            limit_samples: 0,
            limit_msec: 0,
            poll_interval: Duration::from_micros(protocol::POLL_INTERVAL_US),
            trigger: None,
            pending: transport,
            ftdi: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording),
    /// its strings read from it first, as the scan does.
    ///
    /// # Errors
    ///
    /// When a string cannot be read or the product is neither model.
    pub fn from_transport(mut transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        let (product, serial) = product_and_serial(transport.as_mut()).map_err(Error::Device)?;
        let prof = Profile::of_product(&product)
            .ok_or_else(|| Error::arg("not a ChronoVu LA8 or LA16"))?;
        Ok(Self::new(usb, prof, serial, Some(transport)))
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Option<Ftdi>> {
        self.ftdi.lock().expect("chronovu-la lock")
    }
}

/// `ftdi_usb_open_desc` on the scanned device: its product string read
/// again and compared, then libftdi's open.
fn open_desc(mut t: Box<dyn Transport>, product: &str) -> std::result::Result<Context, FtdiError> {
    let idx = t.string_indexes().unwrap_or_default();
    match crate::usb::read_string_ascii(t.as_mut(), idx[1]) {
        Ok(s) if s == product => Context::open(t),
        Ok(_) => Err(FtdiError { code: -3, message: "device not found" }),
        Err(e) => {
            log::debug!("product string: {e}");
            Err(FtdiError { code: -8, message: "unable to fetch product description" })
        }
    }
}

/// `dev_open` after `ftdi_new`.
fn set_up(t: Box<dyn Transport>, prof: &Profile) -> Result<Ftdi> {
    let ctx = open_desc(t, prof.iproduct).map_err(|e| {
        log::error!("Failed to open FTDI device ({}): {}.", e.code, e.message);
        Error::from(e)
    })?;
    let mut ftdi = Ftdi::new(ctx);
    if let Err(e) = ftdi.ctx.tcioflush() {
        log::error!("Failed to purge FTDI buffers ({}): {}.", e.code, e.message);
        return Err(e.into());
    }
    if let Err(e) = ftdi.ctx.set_flow_ctrl(protocol::SIO_RTS_CTS_HS) {
        log::error!("Failed to enable FTDI flow control ({}): {}.", e.code, e.message);
        return Err(e.into());
    }
    std::thread::sleep(Duration::from_millis(100));
    Ok(ftdi)
}

impl Device for ChronovuLaDevice {
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
            Key::Conn => Value::Str(self.usb.conn()),
            Key::Samplerate => Value::U64(self.samplerate),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::Samplerate => {
                let rate = value.as_u64().ok_or_else(bad)?;
                log::trace!("Trying to set samplerate to {rate}Hz.");
                if self.prof.divcount(rate).is_none() {
                    log::error!("Invalid samplerate ({rate}Hz).");
                    log::debug!("Failed to set invalid samplerate ({rate}Hz).");
                    return Err(Error::device("invalid samplerate"));
                }
                self.samplerate = rate;
                log::debug!("Samplerate set to {rate}Hz.");
            }
            Key::LimitMsec => self.limit_msec = value.as_u64().ok_or_else(bad)?,
            Key::LimitSamples => self.limit_samples = value.as_u64().ok_or_else(bad)?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::Samplerate => ValueList::U64(self.prof.samplerates()),
            Key::LimitSamples => ValueList::MinMaxU64(0, self.prof.max_samples()),
            Key::TriggerMatch => ValueList::TriggerMatches(self.prof.trigger_matches.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.lock().is_some() {
            return Ok(());
        }
        log::debug!(
            "Opening {} device ({:04x}:{:04x}).",
            self.prof.name,
            self.usb.vendor_id,
            self.usb.product_id
        );
        let t = match self.pending.take() {
            Some(t) => t,
            None => crate::usb::open(&self.usb).inspect_err(|_| {
                log::error!("Failed to open FTDI device (-4): usb_open() failed.");
            })?,
        };
        let ftdi = set_up(t, self.prof)?;
        *self.lock() = Some(ftdi);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        // After a reset the chip is closed already (ftdic NULL in C).
        if let Some(ftdi) = self.lock().take() {
            ftdi.close();
        }
        Ok(())
    }

    fn trigger_features(&self) -> TriggerFeatures {
        TriggerFeatures { max_stages: 1, ..TriggerFeatures::default() }
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        self.trigger = trigger;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let Some(mut ftdi) = self.lock().take() else {
            log::error!("devc->ftdic was NULL.");
            return Err(Error::device("device is not open"));
        };
        let Some(divcount) = self.prof.divcount(self.samplerate) else {
            *self.lock() = Some(ftdi);
            log::error!("Invalid divcount/samplerate.");
            return Err(Error::device("invalid samplerate"));
        };
        let enabled: Vec<bool> = self.channels.iter().map(|c| c.enabled).collect();
        let trigger = match protocol::convert_trigger(self.prof, self.trigger.as_ref(), &enabled) {
            Ok(t) => t,
            Err(msg) => {
                *self.lock() = Some(ftdi);
                log::error!("{msg}");
                log::error!("Failed to configure trigger.");
                return Err(Error::NotSupported(msg.into()));
            }
        };
        log::debug!(
            "Trigger pattern/mask/edgemask = 0x{:04x} / 0x{:04x} / 0x{:04x}.",
            trigger.pattern,
            trigger.mask,
            trigger.edges
        );
        let command = protocol::command(self.prof.model, divcount, trigger);
        if ftdi.write(&command) != Some(command.len()) {
            protocol::close_usb_reset_sequencer(ftdi);
            log::error!("Acquisition failed to start.");
            return Err(Error::device("acquisition failed to start"));
        }
        let cap = Capture {
            prof: self.prof,
            divcount,
            trigger,
            limit: protocol::sample_limit(self.limit_samples, self.limit_msec, self.samplerate),
            poll_interval: self.poll_interval,
        };
        let slot = Arc::clone(&self.ftdi);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("chronovu-la acquisition".into())
            .spawn(move || {
                let back = protocol::run(ftdi, cap, &sink, &stop2);
                *slot.lock().expect("chronovu-la lock") = back;
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
    use crate::core::{Packet, TriggerMatch};
    use crate::usb::record::Replay;
    use std::fmt::Write as _;
    use std::sync::mpsc::sync_channel;

    /// An LA8 at 0403:8867 (a BM) through the scan's string reads and
    /// `dev_open`: the product string again, libftdi's open (claim,
    /// reset, 9600 Bd), the purge, flow control.
    const OPENED: &str = "# oxrok usb fixture 1\n\
        device vid=0403 pid=8867 version=0400 bus=0 addr=0 strings=1,2,3\n\
        ctl_in 00 06 0300 0000 4 -> 04030904\n\
        ctl_in 00 06 0302 0409 255 -> 1a034300680072006f006e006f005600750020004c0041003800\n\
        ctl_in 00 06 0300 0000 4 -> 04030904\n\
        ctl_in 00 06 0303 0409 255 -> 060353003100\n\
        ctl_in 00 06 0300 0000 4 -> 04030904\n\
        ctl_in 00 06 0302 0409 255 -> 1a034300680072006f006e006f005600750020004c0041003800\n\
        claim 0 -> ok\nmps 81 -> 64\n\
        ctl_out 40 00 0000 0001 - -> ok\nctl_out 40 03 4138 0000 - -> ok\n\
        ctl_out 40 00 0001 0001 - -> ok\nctl_out 40 00 0002 0001 - -> ok\n\
        ctl_out 40 02 0000 0101 - -> ok\n";

    fn device(steps: &str) -> ChronovuLaDevice {
        let text = format!("{OPENED}{steps}");
        ChronovuLaDevice::from_transport(Box::new(Replay::from_text(&text).unwrap())).unwrap()
    }

    /// The readout of a memory of zeros but byte `at` of the readout,
    /// `value`: 4096 byte transfers of 64 byte packets.
    fn readout(at: usize, value: u8) -> String {
        let mut s = String::new();
        let data_per = 62;
        let total = protocol::SDRAM_SIZE;
        let mut sent = 0;
        while sent < total {
            s.push_str("bulk_in 81 4096 -> ");
            for _ in 0..64 {
                if sent >= total {
                    break;
                }
                let n = data_per.min(total - sent);
                s.push_str("0160");
                if (sent..sent + n).contains(&at) {
                    let before = at - sent;
                    if before > 0 {
                        let _ = write!(s, "00{{{before}}}");
                    }
                    let _ = write!(s, "{value:02x}");
                    if n - before - 1 > 0 {
                        let _ = write!(s, "00{{{}}}", n - before - 1);
                    }
                } else {
                    let _ = write!(s, "00{{{n}}}");
                }
                sent += n;
            }
            s.push('\n');
        }
        s
    }

    fn capture(dev: &mut ChronovuLaDevice) -> Vec<Packet> {
        let (tx, rx) = sync_channel(4096);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        let packets = rx.iter().map(|(_, p)| p).collect();
        dev.stop().unwrap();
        packets
    }

    #[test]
    fn scans_strings_and_options() {
        let dev = device("");
        assert_eq!(dev.info().model, "LA8");
        assert_eq!(dev.info().serial, "S1");
        assert_eq!(dev.channels().len(), 8);
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        assert_eq!(dev.list(&Key::LimitSamples, None).unwrap(), ValueList::MinMaxU64(0, 8_388_608));
        assert_eq!(
            dev.list(&Key::TriggerMatch, None).unwrap(),
            ValueList::TriggerMatches(vec![TriggerMatch::Zero, TriggerMatch::One])
        );
    }

    #[test]
    fn a_capture_honours_the_limit_and_finds_the_trigger() {
        // 1 MHz (divcount 99), channel 7 high. Byte 1 of the readout is
        // sample 0 at divcount 99 (pairs swapped), so the trigger is
        // at sample 0.
        let steps = format!(
            "bulk_out 02 63ff8080 -> 4\nbulk_in 81 4096 -> 0160\nbulk_in 81 4096 -> 0160\n{}",
            readout(1, 0x80)
        );
        let mut dev = device(&steps);
        dev.open().unwrap();
        dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(10), None).unwrap();
        let mut t = Trigger::new();
        t.add(0, 7, TriggerMatch::One);
        dev.set_trigger(Some(t)).unwrap();
        let packets = capture(&mut dev);
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert!(matches!(packets[1], Packet::Trigger), "{:?}", &packets[1..]);
        assert!(
            matches!(&packets[2], Packet::Logic(l) if l.data == [0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        );
        assert!(matches!(packets[3], Packet::End));
        assert_eq!(packets.len(), 4);
        dev.close().unwrap();
    }

    #[test]
    fn a_short_block_resets_the_device_once() {
        // The FIFO runs dry in block 0, the reset's drain reads nothing,
        // the sequencer reset's write fails: no second reset.
        let steps = "bulk_out 02 00ff0000 -> 4\n\
            bulk_in 81 4096 -> 0160\n\
            bulk_in 81 4096 -> 01600000\n\
            bulk_in 81 4096 -> 0160\n\
            bulk_in 81 4096 -> 0160\n\
            bulk_out 02 0101010101010101 -> err disconnected\n\
            ctl_out 40 00 0001 0001 - -> err disconnected\n\
            ctl_out 40 00 0000 0001 - -> err disconnected\n\
            release 0 -> err disconnected\n";
        let mut dev = device(steps);
        dev.open().unwrap();
        let packets = capture(&mut dev);
        assert_eq!(packets.len(), 2, "header and end: {packets:?}");
        // Closed by the reset: nothing left to close.
        assert!(dev.lock().is_none());
        dev.close().unwrap();
    }

    #[test]
    fn the_trigger_wait_ends_after_its_reads() {
        let mut steps = String::from("bulk_out 02 00ff0101 -> 4\n");
        let polls = PROFILES_LA8_POLLS + 1;
        for _ in 0..polls {
            steps.push_str("bulk_in 81 4096 -> 0160\n");
        }
        steps.push_str(
            "bulk_in 81 4096 -> 0160\n\
             bulk_out 02 0101010101010101 -> 8\n\
             ctl_out 40 00 0001 0001 - -> ok\nctl_out 40 00 0002 0001 - -> ok\n\
             ctl_out 40 00 0000 0001 - -> ok\nrelease 0 -> ok\n",
        );
        let mut dev = device(&steps);
        dev.poll_interval = Duration::ZERO;
        dev.open().unwrap();
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::One);
        dev.set_trigger(Some(t)).unwrap();
        let packets = capture(&mut dev);
        assert_eq!(packets.len(), 2);
        dev.close().unwrap();
    }

    /// `deadline_polls` at 100 MHz on the LA8.
    const PROFILES_LA8_POLLS: usize = 631;

    #[test]
    fn edge_triggers_are_refused_on_the_la8() {
        let mut dev = device("");
        dev.open().unwrap();
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::Rising);
        dev.set_trigger(Some(t)).unwrap();
        let (tx, _rx) = sync_channel(4);
        assert!(dev.start(PacketSink::new(tx, 0)).is_err());
        // Still open.
        assert!(dev.lock().is_some());
    }

    #[test]
    fn a_stop_resets_the_sequencer_and_ends() {
        // Stopped before the first read: the sequencer reset, purge,
        // reset and close, no read and no "Trigger timed out".
        let steps = "bulk_out 02 00ff0000 -> 4\n\
            bulk_out 02 0101010101010101 -> 8\n\
            ctl_out 40 00 0001 0001 - -> ok\nctl_out 40 00 0002 0001 - -> ok\n\
            ctl_out 40 00 0000 0001 - -> ok\nrelease 0 -> ok\n";
        let mut dev = device(steps);
        dev.open().unwrap();
        let mut ftdi = dev.lock().take().unwrap();
        assert_eq!(ftdi.write(&[0, 0xff, 0, 0]), Some(4));
        let cap = Capture {
            prof: dev.prof,
            divcount: 0,
            trigger: protocol::TriggerMasks::default(),
            limit: 0,
            poll_interval: Duration::ZERO,
        };
        let (tx, rx) = sync_channel(4);
        let stop = AtomicBool::new(true);
        assert!(protocol::run(ftdi, cap, &PacketSink::new(tx, 0), &stop).is_none());
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        assert!(matches!(packets[..], [Packet::Header { .. }, Packet::End]), "{packets:?}");
    }

    #[test]
    fn a_stop_while_sending_ends_at_the_next_packet() {
        let steps = format!("bulk_out 02 00ff0000 -> 4\n{}", readout(1, 0));
        let mut dev = device(&steps);
        dev.open().unwrap();
        let mut ftdi = dev.lock().take().unwrap();
        assert_eq!(ftdi.write(&[0, 0xff, 0, 0]), Some(4));
        let cap = Capture {
            prof: dev.prof,
            divcount: 0,
            trigger: protocol::TriggerMasks::default(),
            limit: 0,
            poll_interval: Duration::ZERO,
        };
        let (tx, rx) = sync_channel(0);
        let stop = AtomicBool::new(false);
        let rest = std::thread::scope(|scope| {
            let stop = &stop;
            let worker =
                scope.spawn(move || protocol::run(ftdi, cap, &PacketSink::new(tx, 0), stop));
            assert!(matches!(rx.recv().unwrap().1, Packet::Header { .. }));
            assert!(matches!(rx.recv().unwrap().1, Packet::Logic(_)));
            stop.store(true, Ordering::SeqCst);
            let rest: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
            assert!(worker.join().unwrap().is_some(), "the chip stays open");
            rest
        });
        assert!(rest.len() <= 2, "at most the packet in hand and the end: {}", rest.len());
        assert!(matches!(rest.last(), Some(Packet::End)));
    }

    #[test]
    fn a_second_start_after_a_reset_is_refused() {
        let steps = "bulk_out 02 00ff0000 -> 4\n\
            bulk_in 81 4096 -> err device \"endpoint stalled\"\n\
            bulk_in 81 4096 -> err device \"endpoint stalled\"\n\
            bulk_out 02 0101010101010101 -> 8\n\
            ctl_out 40 00 0001 0001 - -> ok\nctl_out 40 00 0002 0001 - -> ok\n\
            ctl_out 40 00 0000 0001 - -> ok\nrelease 0 -> ok\n";
        let mut dev = device(steps);
        dev.open().unwrap();
        assert_eq!(capture(&mut dev).len(), 2, "header and end");
        let (tx, _rx) = sync_channel(4);
        let e = dev.start(PacketSink::new(tx, 0)).err().expect("the chip is closed");
        assert!(e.to_string().contains("not open"), "{e}");
        dev.stop().unwrap();
        dev.close().unwrap();
    }

    #[test]
    fn status_only_polls_are_paced() {
        // Three polls after the first read, 30 ms apart, then the failed
        // read's reset, whose own sleep is 100 ms.
        let steps = "bulk_out 02 00ff0000 -> 4\n\
            bulk_in 81 4096 -> 0160\nbulk_in 81 4096 -> 0160\nbulk_in 81 4096 -> 0160\n\
            bulk_in 81 4096 -> err device \"endpoint stalled\"\n\
            bulk_in 81 4096 -> err device \"endpoint stalled\"\n\
            bulk_out 02 0101010101010101 -> 8\n\
            ctl_out 40 00 0001 0001 - -> ok\nctl_out 40 00 0002 0001 - -> ok\n\
            ctl_out 40 00 0000 0001 - -> ok\nrelease 0 -> ok\n";
        let mut dev = device(steps);
        dev.poll_interval = Duration::from_millis(30);
        dev.open().unwrap();
        let started = std::time::Instant::now();
        assert_eq!(capture(&mut dev).len(), 2);
        assert!(started.elapsed() >= Duration::from_millis(190), "{:?}", started.elapsed());
    }

    #[test]
    fn invalid_samplerates_are_refused() {
        let mut dev = device("");
        assert!(dev.set(&Key::Samplerate, &Value::U64(3_000_000), None).is_err());
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        dev.set(&Key::Samplerate, &Value::U64(392_156), None).unwrap();
    }
}
