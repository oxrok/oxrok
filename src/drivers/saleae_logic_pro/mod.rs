// Counterpart in sigrok:
//   libsigrok/src/hardware/saleae-logic-pro/api.c,
//   libsigrok/src/hardware/saleae-logic-pro/protocol.c (the receive callback),
//   Copyright (C) 2017 Jan Luebbe <jluebbe@lasnet.de>,
//   with commits by Uwe Hermann, Gerhard Sittig, Romain Tartière
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! saleae-logic-pro: the Saleae Logic Pro 16 (`21a9:1006`), a Cypress FX3
//! in front of an FPGA, 16 digital channels at 1 to 50 MHz, no analog
//! input, no sample limit: a capture runs until it is stopped.
//!
//! The scan uploads the FX3 firmware to a board that does not show the
//! firmware's strings and looks again after 500 ms. Opening resets the
//! scrambler, loads the FPGA unless its scratch register says it is
//! loaded, and sets up the ADC; every capture authenticates against the
//! crypto chip ([`protocol`]). Scan, open and capture follow `api.c` and
//! `protocol.c` step by step, so that the device sees the same
//! transactions as under sigrok-cli (checked with the emulator,
//! `tools/emu/saleae-logic-pro.sh`).
//!
//! A capture ends on [`Device::stop`], when the device is gone, on a
//! failed transfer, or after 1 s without a transfer, the driver's
//! `BUF_TIMEOUT` (`api.c:391`).

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Value, ValueList,
};
use crate::usb::{BulkStream, Transport, UsbDeviceInfo, UsbId, libusb_error_name, split_conn};
use protocol::{Converter, Link};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

/// The board's id, before and after the firmware upload.
pub const ID: UsbId = UsbId { vendor: 0x21a9, product: 0x1006 };
const FX3_FIRMWARE: &str = "saleae-logicpro16-fx3.fw";
const CONFIGURATION: u8 = 1;
const INTERFACE: u8 = 0;
const ENDPOINT_IN: u8 = 0x82;
/// `BUF_SIZE`, `BUF_COUNT`.
const TRANSFER_SIZE: usize = 16 * 1024;
const TRANSFER_COUNT: usize = 512;
/// How long the stop flag can go unseen.
const POLL: Duration = Duration::from_millis(100);
/// `BUF_TIMEOUT`, in polls: the time without a transfer that ends a
/// capture.
const IDLE_POLLS: u32 = 10;
/// How long a board gets to renumerate (`scan()`).
const RENUM_WAIT: Duration = Duration::from_millis(500);

/// The sample rates.
pub const SAMPLERATES: [u64; 6] =
    [1_000_000, 2_000_000, 2_500_000, 10_000_000, 25_000_000, 50_000_000];

/// The driver.
pub struct LogicProDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
];

impl Driver for LogicProDriver {
    fn id(&self) -> &'static str {
        "saleae-logic-pro"
    }

    fn long_name(&self) -> &'static str {
        "Saleae Logic Pro"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    /// `scan()`, except that `conn=` selects: libsigrok's filter applies
    /// only when `sr_usb_find()` found something, so a conn that it
    /// refuses or that matches nothing selects every Logic Pro
    /// (`api.c:219-236`), and its upload goes to every board whatever the
    /// conn (`api.c:194-212`). Here a board the conn names is followed
    /// through its upload by its hub ports, and no other board is touched.
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
                log::error!("{msg}");
                return Ok(Vec::new());
            }
        };
        let mut found: Vec<UsbDeviceInfo> = crate::usb::find_devices(&[ID])?;
        found.retain(|d| conn.is_none_or(|c| c.matches(d)));
        let mut uploaded = false;
        for info in &found {
            if !has_firmware(info) {
                log::info!("Found a Logic Pro 16 device (no firmware loaded).");
                match upload(info) {
                    Ok(()) => uploaded = true,
                    Err(e) => {
                        log::debug!("Firmware upload: {e}.");
                        log::error!("Firmware upload failed, name {FX3_FIRMWARE}.");
                    }
                }
            }
        }
        if uploaded {
            // Give the device some time to come back and scan again.
            std::thread::sleep(RENUM_WAIT);
            let again = crate::usb::find_devices(&[ID])?;
            found = again.into_iter().filter(|d| found.iter().any(|s| same_board(d, s))).collect();
        }
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in found {
            log::debug!("Found a Logic Pro 16 device.");
            devices.push(Box::new(LogicProDevice::new(info, None)));
        }
        Ok(devices)
    }
}

/// Whether `d`, listed after an upload, is the board listed as `s`: by its
/// hub ports, which stay the same when it renumerates, and on a backend
/// that knows no ports by its bus.
fn same_board(d: &UsbDeviceInfo, s: &UsbDeviceInfo) -> bool {
    d.bus == s.bus && (d.ports == s.ports || d.ports.is_empty() || s.ports.is_empty())
}

/// `scan_firmware()`: the board opened and its manufacturer, then its
/// product string read. A board without them takes the upload.
fn has_firmware(info: &UsbDeviceInfo) -> bool {
    let Ok(mut t) = crate::usb::open(info) else { return false };
    let Ok([m, p, _]) = t.string_indexes() else { return false };
    crate::usb::read_string_ascii(t.as_mut(), m).is_ok_and(|s| s == "Saleae")
        && crate::usb::read_string_ascii(t.as_mut(), p).is_ok_and(|s| s == "Logic Pro")
}

fn upload(info: &UsbDeviceInfo) -> Result<()> {
    let mut t = crate::usb::open(info)?;
    protocol::upload_firmware(t.as_mut(), FX3_FIRMWARE)
}

/// `dev_open()`: the claim and `saleae_logic_pro_init()`.
/// `set_configuration` sends nothing when the configuration is active
/// (always on Linux), but macOS can leave a device unconfigured right
/// after its renumeration.
fn set_up(mut t: Box<dyn Transport>) -> Result<Link> {
    t.set_configuration(CONFIGURATION)?;
    t.claim_interface(INTERFACE).inspect_err(|e| {
        log::error!("Failed to claim interface: {}.", libusb_error_name(e));
    })?;
    let mut link = Link::new(t);
    link.init()?;
    Ok(link)
}

/// One Logic Pro 16.
pub struct LogicProDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    samplerate: u64,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl LogicProDevice {
    fn new(usb: UsbDeviceInfo, transport: Option<Box<dyn Transport>>) -> Self {
        LogicProDevice {
            info: DeviceInfo {
                driver: "saleae-logic-pro",
                vendor: "Saleae".into(),
                model: "Logic Pro 16".into(),
                version: String::new(),
                serial: String::new(),
                conn: Some(usb.conn()),
            },
            usb,
            channels: (0..16).map(|i| Channel::new(i, ChannelKind::Logic, i.to_string())).collect(),
            samplerate: SAMPLERATES[3],
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording), its
    /// firmware running.
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        if usb.id() != ID {
            return Err(Error::arg("not a Saleae Logic Pro"));
        }
        Ok(Self::new(usb, Some(transport)))
    }

    fn channel_mask(&self) -> u16 {
        self.channels.iter().filter(|c| c.enabled).fold(0, |m, c| m | 1 << c.index)
    }

    /// `dev_acquisition_start()` up to the header: the prepare and the
    /// transfers queued. A device that left during the prepare is what
    /// makes libusb's submit fail (`api.c:421-427`); nusb only reports it
    /// when a transfer ends, so the last request of the prepare says.
    fn begin(&self, link: &mut Link) -> Result<Box<dyn BulkStream>> {
        let mask = self.channel_mask();
        log::debug!("{} channels enabled (0x{mask:04x})", mask.count_ones());
        let Some((divider, pre_divider)) = protocol::rate_registers(self.samplerate) else {
            return Err(Error::Arg(format!("Unknown sample rate {}.", self.samplerate)));
        };
        let stream = match link.prepare(mask, divider, pre_divider) {
            Err(Error::Disconnected) => Err(Error::Disconnected),
            _ => link.transport().bulk_in_stream(ENDPOINT_IN, TRANSFER_SIZE, TRANSFER_COUNT),
        };
        stream.inspect_err(|e| {
            log::error!("Failed to submit transfer: {}.", libusb_error_name(e));
        })
    }
}

impl Device for LogicProDevice {
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
            Key::Conn => Ok(Value::Str(self.usb.conn())),
            Key::Samplerate => Ok(Value::U64(self.samplerate)),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// Any rate is taken, as in `config_set()`; the capture refuses one
    /// the board does not have. A rate goes out when a capture starts, so
    /// one set during a capture is refused rather than kept unsent.
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::Samplerate => {
                if self.worker.as_ref().is_some_and(|w| !w.handle.is_finished()) {
                    return Err(Error::Busy);
                }
                self.samplerate = value
                    .as_u64()
                    .ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())))?;
                Ok(())
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::Samplerate => Ok(ValueList::U64(SAMPLERATES.to_vec())),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let t = match self.pending.take() {
            Some(t) => t,
            None => crate::usb::open(&self.usb)?,
        };
        let link = set_up(t)?;
        *self.link.lock().expect("link lock") = Some(link);
        Ok(())
    }

    /// Ends a capture still running and joins its thread; the session is
    /// over by then, so a send cannot block it for ever.
    fn close(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        if let Some(mut link) = self.link.lock().expect("link lock").take() {
            let _ = link.transport().release_interface(INTERFACE);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(w) = self.worker.take() {
            if !w.handle.is_finished() {
                self.worker = Some(w);
                return Err(Error::Busy);
            }
            let _ = w.handle.join();
        }
        let link = self.link.lock().expect("link lock").take();
        let Some(mut link) = link else {
            return Err(Error::device("device is not open"));
        };
        match self.begin(&mut link) {
            Ok(stream) => {
                let slot = Arc::clone(&self.link);
                let stop = Arc::new(AtomicBool::new(false));
                let stop2 = Arc::clone(&stop);
                let converter = Converter::new(self.channel_mask());
                let handle = std::thread::Builder::new()
                    .name("saleae-logic-pro acquisition".into())
                    .spawn(move || {
                        run(stream, &mut link, converter, &sink, &stop2);
                        *slot.lock().expect("link lock") = Some(link);
                    })
                    .map_err(Error::Io)?;
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                // libsigrok's session stops a device whose start failed
                // as well (session.c:831-835), which sends the stop's
                // requests.
                link.stop();
                *self.link.lock().expect("link lock") = Some(link);
                Err(e)
            }
        }
    }

    /// Asks the capture to end; `close` joins its thread, which may still
    /// be sending packets.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// What a transfer leaves the capture to do.
#[derive(Debug, PartialEq, Eq)]
enum Next {
    Continue,
    End,
}

/// The acquisition thread: the header, the capture's start, the
/// transfers, the stop's requests with the stream still in place as in
/// `dev_acquisition_stop()` (the C driver leaves its transfers submitted;
/// here dropping the stream cancels them), and the end.
fn run(
    mut stream: Box<dyn BulkStream>,
    link: &mut Link,
    mut converter: Converter,
    sink: &PacketSink,
    stop: &AtomicBool,
) {
    sink.send(Packet::header());
    link.start();
    let mut idle = 0;
    while !stop.load(Ordering::SeqCst) {
        if on_transfer(stream.next(POLL), &mut converter, &mut idle, sink) == Next::End {
            break;
        }
    }
    link.stop();
    drop(stream);
    sink.send(Packet::End);
}

/// `saleae_logic_pro_receive_data()` and `dev_acquisition_handle()`: one
/// answer of the stream. The C driver drops a failed transfer without
/// submitting it again, and the idle timeout then ends the capture; here
/// the first failure does.
fn on_transfer(
    transfer: Result<Vec<u8>>,
    converter: &mut Converter,
    idle: &mut u32,
    sink: &PacketSink,
) -> Next {
    match transfer {
        Ok(data) => {
            *idle = 0;
            let samples = converter.convert(&data);
            if !samples.is_empty() && !sink.send(Packet::logic(2, samples)) {
                return Next::End;
            }
            Next::Continue
        }
        Err(Error::Timeout) => {
            *idle += 1;
            if *idle >= IDLE_POLLS { Next::End } else { Next::Continue }
        }
        Err(e) => {
            log::debug!("Transfer failed: {e}.");
            Next::End
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn device() -> LogicProDevice {
        LogicProDevice::new(UsbDeviceInfo::default(), None)
    }

    #[test]
    fn options_follow_the_c_driver() {
        let mut d = device();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(10_000_000));
        // Any rate is taken; the capture refuses it.
        d.set(&Key::Samplerate, &Value::U64(3_000_000), None).unwrap();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(3_000_000));
        assert_eq!(d.channel_mask(), 0xffff);
        d.enable_channel(3, false).unwrap();
        assert_eq!(d.channel_mask(), 0xfff7);
        assert!(d.set(&Key::LimitSamples, &Value::U64(1), None).is_err());
        let names: Vec<&str> = d.channels.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names[..3], ["0", "1", "2"]);
    }

    #[test]
    fn the_idle_timeout_failures_and_data() {
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let sink = PacketSink::new(tx, 0);
        let mut c = Converter::new(1);
        let mut idle = 0;
        for _ in 0..IDLE_POLLS - 1 {
            assert_eq!(on_transfer(Err(Error::Timeout), &mut c, &mut idle, &sink), Next::Continue);
        }
        // Data resets the time, and a short transfer gives only its own
        // samples (protocol.c:906 converts all 16 KiB).
        assert_eq!(on_transfer(Ok(vec![0; 4]), &mut c, &mut idle, &sink), Next::Continue);
        assert_eq!(idle, 0);
        let (_, p) = rx.try_recv().unwrap();
        assert_eq!(p, Packet::logic(2, vec![0; 64]));
        for _ in 0..IDLE_POLLS - 1 {
            assert_eq!(on_transfer(Err(Error::Timeout), &mut c, &mut idle, &sink), Next::Continue);
        }
        assert_eq!(on_transfer(Err(Error::Timeout), &mut c, &mut idle, &sink), Next::End);
        let mut idle = 0;
        let stalled = Err(Error::device("stalled"));
        assert_eq!(on_transfer(stalled, &mut c, &mut idle, &sink), Next::End);
        assert_eq!(on_transfer(Err(Error::Disconnected), &mut c, &mut idle, &sink), Next::End);
    }

    #[test]
    fn a_board_is_followed_by_its_ports() {
        let info = |ports: &[u8], address| UsbDeviceInfo {
            bus: 1,
            address,
            ports: ports.to_vec(),
            ..UsbDeviceInfo::default()
        };
        assert!(same_board(&info(&[1, 2], 9), &info(&[1, 2], 3)));
        assert!(!same_board(&info(&[1, 3], 9), &info(&[1, 2], 3)));
        assert!(same_board(&info(&[], 9), &info(&[], 3)));
    }
}
