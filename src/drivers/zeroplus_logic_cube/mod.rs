// Counterpart in sigrok:
//   libsigrok/src/hardware/zeroplus-logic-cube/api.c,
//   Copyright (C) 2010-2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Soeren Apel,
//   Gerhard Sittig, Martin Ling, Daniel Elstner, Jan Metzger,
//   Michał Janiszewski, René Pecher, Romain Tartière, Yasushi SHOJI, g-user
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! zeroplus-logic-cube: the ZEROPLUS Logic Cube LAP-C series, 16 or 32
//! channels with a sample memory of 32 K to 2 M samples per channel, at up
//! to 100 or 200 MHz. A vendor USB device with no firmware to upload: a
//! register file behind control requests and one bulk IN endpoint for the
//! memory ([`protocol`]).
//!
//! Scan, open and capture follow `api.c` step by step, so that the device
//! sees the same transactions as under sigrok-cli, checked with the
//! emulator (`tools/emu/zeroplus-logic-cube.sh`). The C's capture runs in
//! `dev_acquisition_start()` and cannot be stopped; here it runs on a
//! thread, a stop ends the wait or the readout and sends the
//! `analyzer_reset()` of `dev_acquisition_stop()`.
//!
//! Unlike libsigrok's driver, `conn=` selects a board, though it is not
//! listed (the C lists no scan option, api.c:187 and 450, and libsigrok
//! refuses any), and a rate that is not listed is refused rather than read
//! past the list (api.c:148).
//! [`protocol`] lists the others. Each device has its own registers, where
//! `analyzer.c` has one set for the process.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    PacketSink, Result, Trigger, TriggerFeatures, TriggerMatch, Value, ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, libusb_error_name, split_conn};
use protocol::{Link, Profile, Regs};
use std::fmt::Write as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

const USB_CONFIGURATION: u8 = 1;
const USB_INTERFACE: u8 = 0;
/// The serial number's buffer holds 63 characters.
const SERIAL_MAX: usize = 63;

const CHANNEL_NAMES: [&str; 32] = [
    "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7", "B0", "B1", "B2", "B3", "B4", "B5", "B6", "B7",
    "C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7",
];
const CLOCK_EDGES: [&str; 2] = ["rising", "falling"];
const TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];

/// The driver.
pub struct ZeroplusDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 7] = [
    OptionDesc::new(Key::LimitSamples, Access { get: false, set: true, list: true }),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::VoltageThreshold, Access::GET_SET_LIST),
    OptionDesc::new(Key::ExternalClock, Access::GET_SET),
    OptionDesc::new(Key::ClockEdge, Access::GET_SET_LIST),
];

/// The serial number, as the scan reads it into its 64 byte buffer.
fn read_serial(t: &mut dyn Transport) -> Result<String> {
    let index = t.string_indexes().unwrap_or_default()[2];
    if index == 0 {
        return Ok(String::new());
    }
    Ok(crate::usb::read_string_ascii(t, index)?.chars().take(SERIAL_MAX).collect())
}

impl Driver for ZeroplusDriver {
    fn id(&self) -> &'static str {
        "zeroplus-logic-cube"
    }

    fn long_name(&self) -> &'static str {
        "ZEROPLUS Logic Cube LAP-C series"
    }

    /// `conn=` is taken, not listed: the C lists no scan option (api.c:450)
    /// and `--show` follows it.
    fn accepted_scan_options(&self) -> &'static [Key] {
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
                log::error!("{msg}");
                return Ok(Vec::new());
            }
        };
        let mut devices = Vec::new();
        for info in crate::usb::list_devices()? {
            if info.vendor_id != protocol::VENDOR {
                continue;
            }
            let Some(prof) = Profile::of_pid(info.product_id) else { continue };
            if conn.is_some_and(|c| !c.matches(&info)) {
                continue;
            }
            let Ok(mut t) = crate::usb::open(&info) else { continue };
            let serial = match read_serial(t.as_mut()) {
                Ok(s) => s,
                Err(e) => {
                    log::warn!("Cannot get USB serial number: {}.", libusb_error_name(&e));
                    continue;
                }
            };
            drop(t);
            log::info!("Found ZEROPLUS {}.", prof.name);
            devices.push(ZeroplusDevice::new(info, prof, serial, None));
        }
        name_conn(&mut devices, conn.is_some());
        Ok(devices.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// The device lines name `conn=` when it was given, and when more boards
/// than one were found, which is when a user has to select. A lone board's
/// line is as sigrok-cli's, whose driver has no `conn`.
fn name_conn(devices: &mut [ZeroplusDevice], given: bool) {
    if given || devices.len() > 1 {
        for d in devices {
            d.info.conn = Some(d.usb.conn());
        }
    }
}

/// A board of the series.
pub struct ZeroplusDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    prof: &'static Profile,
    channels: Vec<Channel>,
    /// `cur_samplerate`, until set the 1 MHz the open chooses.
    samplerate: u64,
    limit_samples: u64,
    capture_ratio: u64,
    /// `cur_threshold`; the C takes a threshold of 0 V for none set
    /// (api.c:341).
    threshold: Option<f64>,
    ext_clock: bool,
    ext_falling: bool,
    trigger: Option<Trigger>,
    regs: Regs,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// `usb_get_port_path()`'s text: `BUS-PORT.PORT`.
fn port_path(usb: &UsbDeviceInfo) -> String {
    let mut s = format!("{}-", usb.bus);
    for (i, p) in usb.ports.iter().enumerate() {
        let _ = write!(s, "{}{p}", if i == 0 { "" } else { "." });
    }
    s
}

impl ZeroplusDevice {
    fn new(
        usb: UsbDeviceInfo,
        prof: &'static Profile,
        serial: String,
        transport: Option<Box<dyn Transport>>,
    ) -> Self {
        ZeroplusDevice {
            info: DeviceInfo {
                driver: "zeroplus-logic-cube",
                vendor: "ZEROPLUS".into(),
                model: prof.name.into(),
                version: String::new(),
                serial,
                conn: None,
            },
            usb,
            prof,
            channels: CHANNEL_NAMES[..prof.channels]
                .iter()
                .enumerate()
                .map(|(i, name)| Channel::new(i, ChannelKind::Logic, *name))
                .collect(),
            samplerate: 1_000_000,
            limit_samples: 0,
            capture_ratio: 0,
            threshold: None,
            ext_clock: false,
            ext_falling: false,
            trigger: None,
            regs: Regs::default(),
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording), its
    /// serial number read from it first, as the scan does.
    ///
    /// # Errors
    ///
    /// When the product is none of the boards, or the serial cannot be read.
    pub fn from_transport(mut transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        let prof = (usb.vendor_id == protocol::VENDOR)
            .then(|| Profile::of_pid(usb.product_id))
            .flatten()
            .ok_or_else(|| Error::arg("not a ZEROPLUS Logic Cube"))?;
        let serial = read_serial(transport.as_mut())?;
        Ok(Self::new(usb, prof, serial, Some(transport)))
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Option<Link>> {
        self.link.lock().expect("zeroplus-logic-cube lock")
    }

    /// True while a capture holds the link.
    fn capturing(&self) -> bool {
        self.worker.is_some() && self.lock().is_none()
    }

    /// `zp_set_samplerate()`, with the list really searched.
    fn set_samplerate(&mut self, rate: u64) -> Result<()> {
        if !protocol::SAMPLERATES.contains(&rate) || rate > self.prof.max_samplerate() {
            log::error!("Unsupported samplerate: {rate}Hz.");
            return Err(Error::arg("unsupported samplerate"));
        }
        log::info!("Setting samplerate to {rate}Hz.");
        (self.regs.freq, self.regs.scale) = protocol::split_rate(rate);
        self.samplerate = rate;
        Ok(())
    }

    /// `set_limit_samples()`.
    fn set_limit_samples(&mut self, samples: u64) -> Result<()> {
        // sr_config_set() refuses it before the driver sees it
        // (hwdriver.c, check_key()).
        if samples == 0 {
            log::error!("Cannot set 'limit_samples' to 0.");
            return Err(Error::arg("invalid value for limit_samples"));
        }
        let samples = samples.min(self.prof.max_samples());
        self.limit_samples = samples;
        self.regs.memory_size = protocol::memory_code(samples);
        log::info!(
            "Setting memory size to {}K.",
            protocol::memory_bytes(self.regs.memory_size) / 1024
        );
        Ok(())
    }

    /// `set_voltage_threshold()`.
    fn set_threshold(&mut self, volts: f64) {
        let volts = protocol::clamp_threshold(volts);
        self.threshold = Some(volts);
        self.regs.thresh = protocol::threshold_reg(volts);
        log::info!("Setting voltage threshold to {volts:.6}V.");
    }

    /// The end of `dev_open()` once the device is claimed.
    fn set_up(&mut self, mut link: Link) -> Link {
        // Set the default configuration after power on. A failed read is
        // not 0, so it configures nothing, as in C.
        if matches!(link.read_status(), Ok(0)) {
            link.configure(&self.regs);
        }
        link.reset();
        link.initialize();
        if self.threshold.is_none() {
            self.set_threshold(1.5);
        }
        link
    }
}

impl Device for ZeroplusDevice {
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
            Key::CaptureRatio => Value::U64(self.capture_ratio),
            Key::VoltageThreshold => {
                let v = self.threshold.unwrap_or(1.5);
                Value::RangeF64(v, v)
            }
            Key::ExternalClock => Value::Bool(self.ext_clock),
            Key::ClockEdge => Value::Str(CLOCK_EDGES[usize::from(self.ext_falling)].into()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        // Everything is pushed by the next capture's configure: a capture
        // running now would not take it, and the C could not be asked.
        if self.capturing() {
            return Err(Error::Busy);
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::Samplerate => self.set_samplerate(value.as_u64().ok_or_else(bad)?)?,
            Key::LimitSamples => self.set_limit_samples(value.as_u64().ok_or_else(bad)?)?,
            Key::CaptureRatio => {
                // sr_config_set() refuses it before the driver sees it
                // (hwdriver.c, check_key()).
                let r = value.as_u64().ok_or_else(bad)?;
                if r > 100 {
                    log::error!("Capture ratio must be 0..100.");
                    return Err(bad());
                }
                self.capture_ratio = r;
            }
            Key::VoltageThreshold => {
                let Value::RangeF64(low, high) = value else { return Err(bad()) };
                self.set_threshold((low + high) / 2.0);
            }
            Key::ExternalClock => {
                self.ext_clock = value.as_bool().ok_or_else(bad)?;
                self.regs.ext_clock = self.ext_clock;
            }
            Key::ClockEdge => {
                let v = value.as_str().ok_or_else(bad)?;
                let i = CLOCK_EDGES.iter().position(|e| *e == v).ok_or_else(bad)?;
                self.ext_falling = i == 1;
                self.regs.ext_falling = self.ext_falling;
            }
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
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::VoltageThreshold => {
                ValueList::RangeF64(crate::core::option::min_max_step_thresholds(-6.0, 6.0, 0.1))
            }
            Key::LimitSamples => ValueList::MinMaxU64(0, self.prof.max_samples()),
            Key::ClockEdge => ValueList::Str(CLOCK_EDGES.iter().map(|s| (*s).to_owned()).collect()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.lock().is_some() || self.worker.is_some() {
            return Ok(());
        }
        let mut t = match self.pending.take() {
            Some(t) => t,
            None => crate::usb::open(&self.usb)?,
        };
        t.force_configuration(USB_CONFIGURATION).inspect_err(|e| {
            log::error!(
                "Unable to set USB configuration {USB_CONFIGURATION}: {}.",
                libusb_error_name(e)
            );
        })?;
        t.claim_interface(USB_INTERFACE).inspect_err(|e| {
            log::error!("Unable to claim interface: {}.", libusb_error_name(e));
        })?;
        let link = self.set_up(Link::new(t));
        *self.lock() = Some(link);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(w) = self.worker.take() {
            let _ = w.handle.join();
        }
        let Some(link) = self.lock().take() else { return Ok(()) };
        log::info!(
            "Closing device on {}.{} (logical) / {} (physical) interface {USB_INTERFACE}.",
            self.usb.bus,
            self.usb.address,
            port_path(&self.usb)
        );
        let mut t = link.into_transport();
        // Their results are not looked at (api.c:358-360).
        let _ = t.release_interface(USB_INTERFACE);
        let _ = t.reset();
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
        if self.capturing() {
            return Err(Error::Busy);
        }
        // The last capture gave its link back and has its end to send.
        if let Some(w) = self.worker.take() {
            let _ = w.handle.join();
        }
        let enabled: Vec<bool> = self.channels.iter().map(|c| c.enabled).collect();
        let trigger =
            protocol::trigger_regs(self.trigger.as_ref(), &enabled).inspect_err(|_| {
                log::error!("Failed to configure triggers.");
            })?;
        let Some(link) = self.lock().take() else {
            return Err(Error::device("device is not open"));
        };
        self.regs.trigger_status = trigger.status;
        self.regs.trigger_edge = trigger.edge;
        let (bar, ramsize) = protocol::trigger_bar(
            self.limit_samples,
            self.regs.memory_size,
            trigger.any,
            self.capture_ratio,
        );
        self.regs.triggerbar = bar;
        self.regs.ramsize_trigger = ramsize;
        // %d and %x of the C's unsigned ints: a wrapped count prints -1.
        log::debug!("triggerbar_address = {}(0x{bar:x})", bar.cast_signed());
        log::debug!("ramsize_triggerbar_address = {}(0x{ramsize:x})", ramsize.cast_signed());
        let regs = self.regs.clone();
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("zeroplus-logic-cube acquisition".into())
            .spawn(move || {
                let mut give_back = |link: Link| {
                    *slot.lock().expect("zeroplus-logic-cube lock") = Some(link);
                };
                protocol::run(link, &regs, &sink, &stop2, &mut give_back);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the capture to end, without waiting: the thread may have to
    /// hand its packets to the session calling this. `close` joins it.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
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

    /// One `gl_reg_write()`.
    fn w(s: &mut String, reg: u8, value: u8) {
        let _ = writeln!(s, "ctl_out 41 0c 0083 0000 {reg:02x} -> ok");
        let _ = writeln!(s, "ctl_out 41 0c 0085 0000 {value:02x} -> ok");
    }

    /// One `gl_reg_read()`.
    fn r(s: &mut String, reg: u8, value: u8) {
        let _ = writeln!(s, "ctl_out 41 0c 0083 0000 {reg:02x} -> ok");
        let _ = writeln!(s, "ctl_in 41 0c 0084 0000 1 -> {value:02x}");
    }

    fn device_steps() -> String {
        let mut s = String::from(
            "# oxrok usb fixture 1\n\
             device vid=0c12 pid=700e version=0000 bus=1 addr=2 strings=0,0,0\n\
             config 1 -> ok\nclaim 0 -> ok\n",
        );
        // A warm board: no configure in the open.
        r(&mut s, 0x21, 0x02);
        w(&mut s, 0x20, 0xc0);
        w(&mut s, 0x20, 0xc1);
        w(&mut s, 0x20, 0x40);
        w(&mut s, 0x20, 0x42);
        w(&mut s, 0x20, 0x40);
        s
    }

    /// `analyzer_configure()` with `regs` and `analyzer_start()`.
    fn start_steps(s: &mut String, regs: &Regs) {
        for v in [0x41, 0x40, 0x42, 0x40] {
            w(s, 0x20, v);
        }
        let (sel, div, mul) = protocol::clock_regs(regs.freq, regs.scale).unwrap();
        w(s, 0x30, div);
        w(s, 0x31, mul);
        w(s, 0x32, 2);
        w(s, 0x34, sel);
        w(s, 0x35, regs.memory_size);
        w(s, 0x36, if regs.ext_clock { if regs.ext_falling { 0 } else { 2 } } else { 1 });
        for (i, v) in regs.trigger_status.iter().enumerate() {
            w(s, 0x40 + u8::try_from(i).unwrap(), *v);
        }
        w(s, 0x48, regs.trigger_edge);
        w(s, 0x50, 1);
        w(s, 0x51, 0);
        for reg in 0x55..0x59 {
            w(s, reg, regs.thresh);
        }
        for (reg, value) in [(0x60, regs.ramsize_trigger), (0x63, regs.triggerbar)] {
            let b = value.to_le_bytes();
            w(s, reg, b[0]);
            w(s, reg + 1, b[1]);
            w(s, reg + 2, b[2]);
        }
        w(s, 0x66, 1);
        for reg in 0x70..0x79 {
            w(s, reg, 0);
        }
        w(s, 0x7a, 0);
        w(s, 0x7b, 0);
        for (reg, v) in [(0x80, 0x12), (0x81, 0x34), (0x82, 0x56), (0x83, 0x78)] {
            w(s, reg, v);
        }
        w(s, 0x84, 1);
        w(s, 0x85, 0);
        for v in [0x40, 0x42, 0x40, 0x44] {
            w(s, 0x20, v);
        }
    }

    fn address(s: &mut String, reg0: u8, value: u32) {
        let b = value.to_le_bytes();
        r(s, reg0 + 2, b[2]);
        r(s, reg0 + 1, b[1]);
        r(s, reg0, b[0]);
    }

    fn read_start(s: &mut String) {
        w(s, 0x20, 0xf0);
        let _ = writeln!(s, "ctl_out 41 0c 0083 0000 a0 -> ok");
    }

    fn bulk(s: &mut String, data: &str) {
        let _ = writeln!(s, "ctl_out 41 04 0082 0000 0000000000080000 -> ok");
        let _ = writeln!(s, "bulk_in 81 2048 -> {data}");
    }

    fn device(steps: &str) -> ZeroplusDevice {
        let t = Replay::from_text(steps).unwrap();
        let mut d = ZeroplusDevice::from_transport(Box::new(t)).unwrap();
        d.open().unwrap();
        d
    }

    fn capture(d: &mut ZeroplusDevice) -> Vec<Packet> {
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        d.stop().unwrap();
        packets
    }

    /// The regs of the capture under test: a limit of 1000, no trigger.
    fn regs_1000() -> Regs {
        Regs { ramsize_trigger: 999, ..Regs::default() }
    }

    /// The addresses of a capture of 1000 at the 8 KiB memory: 1051
    /// samples to throw away, 996 to keep.
    fn addresses_1000(s: &mut String) {
        r(s, 0x21, 0);
        r(s, 0x21, 2);
        address(s, 0x9b, 0x3e6);
        address(s, 0x96, 0x3e7);
        address(s, 0x90, 0);
    }

    fn samples(packets: &[Packet]) -> usize {
        packets
            .iter()
            .map(|p| match p {
                Packet::Logic(l) => l.len(),
                _ => 0,
            })
            .sum()
    }

    fn stream_of(n: usize) -> String {
        format!("00{{{n}}}")
    }

    #[test]
    fn scan_results_and_options() {
        let d = device(&device_steps());
        assert_eq!(d.info().model, "LAP-C(16032)");
        assert_eq!(d.info().vendor, "ZEROPLUS");
        assert_eq!(d.channels().len(), 16);
        assert_eq!(d.channels()[15].name, "B7");
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(1_000_000));
        assert_eq!(d.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(1.5, 1.5));
        assert_eq!(d.list(&Key::LimitSamples, None).unwrap(), ValueList::MinMaxU64(0, 32768));
        assert!(d.get(&Key::LimitSamples, None).is_err());
        assert_eq!(d.get(&Key::ClockEdge, None).unwrap(), Value::Str("rising".into()));
    }

    #[test]
    fn the_threshold_list_accumulates_as_in_c() {
        let d = device(&device_steps());
        let ValueList::RangeF64(list) = d.list(&Key::VoltageThreshold, None).unwrap() else {
            panic!("a list of ranges");
        };
        assert_eq!(list.len(), 121);
        assert_eq!(list[0], (-6.0, -6.0));
        assert_eq!(list[60], (0.0, 0.0));
        // No entry is 1.5, so --show marks none current.
        assert!(!list.iter().any(|&(low, _)| (low - 1.5).abs() < 1e-15));
        assert!((list[75].0 - 1.5).abs() < 1e-9);
    }

    #[test]
    fn rates_not_listed_are_refused() {
        let mut d = device(&device_steps());
        for rate in [3_000_000, 46, 123_456_789, 150_000_000, 200_000_000, 0] {
            assert!(d.set(&Key::Samplerate, &Value::U64(rate), None).is_err(), "{rate}");
        }
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(1_000_000));
        d.set(&Key::Samplerate, &Value::U64(25_000_000), None).unwrap();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(25_000_000));
        assert_eq!(d.regs.freq, 25);
    }

    #[test]
    fn limits_and_thresholds_set_their_registers() {
        let mut d = device(&device_steps());
        assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
        d.set(&Key::LimitSamples, &Value::U64(40000), None).unwrap();
        assert_eq!((d.limit_samples, d.regs.memory_size), (32768, 2));
        d.set(&Key::VoltageThreshold, &Value::RangeF64(0.0, 3.3), None).unwrap();
        assert_eq!(d.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(1.65, 1.65));
        d.set(&Key::VoltageThreshold, &Value::RangeF64(9.0, 9.0), None).unwrap();
        assert_eq!(d.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(6.0, 6.0));
        assert_eq!(d.regs.thresh, 8);
        assert!(d.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
        assert!(d.set(&Key::ClockEdge, &Value::Str("up".into()), None).is_err());
        d.set(&Key::ClockEdge, &Value::Str("falling".into()), None).unwrap();
        assert!(d.regs.ext_falling);
        d.set(&Key::ExternalClock, &Value::Bool(true), None).unwrap();
        assert!(d.regs.ext_clock);
    }

    #[test]
    fn a_threshold_of_0_v_survives_the_open() {
        // api.c:341 takes it for none set: the open makes it 1.5 V.
        let t = Replay::from_text(&device_steps()).unwrap();
        let mut d = ZeroplusDevice::from_transport(Box::new(t)).unwrap();
        d.set(&Key::VoltageThreshold, &Value::RangeF64(0.0, 0.0), None).unwrap();
        d.open().unwrap();
        assert_eq!(d.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(0.0, 0.0));
        assert_eq!(d.regs.thresh, 63);
    }

    #[test]
    fn a_cold_board_is_configured_by_the_open() {
        let mut s = String::from(
            "# oxrok usb fixture 1\n\
             device vid=0c12 pid=700e version=0000 bus=1 addr=2 strings=0,0,0\n\
             config 1 -> ok\nclaim 0 -> ok\n",
        );
        r(&mut s, 0x21, 0);
        // The configure, up to the GO of a start, and the reset for it.
        let mut cfg = String::new();
        start_steps(&mut cfg, &Regs::default());
        let lines: Vec<&str> = cfg.lines().collect();
        for l in &lines[..lines.len() - 8] {
            s.push_str(l);
            s.push('\n');
        }
        w(&mut s, 0x20, 0xc0);
        w(&mut s, 0x20, 0xc1);
        w(&mut s, 0x20, 0x40);
        w(&mut s, 0x20, 0x42);
        w(&mut s, 0x20, 0x40);
        let d = device(&s);
        assert!(d.lock().is_some());
    }

    #[test]
    fn a_capture_discards_keeps_and_ends() {
        let mut s = device_steps();
        start_steps(&mut s, &regs_1000());
        addresses_1000(&mut s);
        read_start(&mut s);
        for _ in 0..4 {
            bulk(&mut s, &stream_of(2048));
        }
        w(&mut s, 0x20, 0xe0);
        w(&mut s, 0x20, 0xc0);
        s.push_str("release 0 -> ok\nreset -> ok\n");
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let packets = capture(&mut d);
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert!(matches!(packets.last(), Some(Packet::End)));
        assert_eq!(samples(&packets), 996);
        assert!(!packets.iter().any(|p| matches!(p, Packet::Trigger)));
        d.close().unwrap();
        assert!(d.lock().is_none());
    }

    #[test]
    fn a_short_read_ends_the_capture_with_what_came() {
        let mut s = device_steps();
        start_steps(&mut s, &regs_1000());
        addresses_1000(&mut s);
        read_start(&mut s);
        bulk(&mut s, &stream_of(2048));
        bulk(&mut s, &stream_of(2048));
        // 27 samples to throw away of 255 that came, and no read after it.
        bulk(&mut s, &stream_of(1020));
        w(&mut s, 0x20, 0xe0);
        w(&mut s, 0x20, 0xc0);
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(samples(&packets), 255 - 27);
        assert!(matches!(packets.last(), Some(Packet::End)));
    }

    #[test]
    fn a_failed_read_ends_the_capture_without_stale_samples() {
        let mut s = device_steps();
        start_steps(&mut s, &regs_1000());
        addresses_1000(&mut s);
        read_start(&mut s);
        bulk(&mut s, &stream_of(2048));
        bulk(&mut s, &stream_of(2048));
        bulk(&mut s, &stream_of(2048));
        bulk(&mut s, "err device \"endpoint stalled\"");
        w(&mut s, 0x20, 0xe0);
        w(&mut s, 0x20, 0xc0);
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(samples(&packets), 512 - 27);
        assert!(matches!(packets.last(), Some(Packet::End)));
    }

    #[test]
    fn a_failed_address_read_gives_no_samples() {
        let mut s = device_steps();
        start_steps(&mut s, &regs_1000());
        r(&mut s, 0x21, 0);
        r(&mut s, 0x21, 2);
        r(&mut s, 0x9d, 0);
        r(&mut s, 0x9c, 3);
        let _ = writeln!(s, "ctl_out 41 0c 0083 0000 9b -> ok");
        let _ = writeln!(s, "ctl_in 41 0c 0084 0000 1 -> err device \"endpoint stalled\"");
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let packets = capture(&mut d);
        assert!(matches!(packets[..], [Packet::Header { .. }, Packet::End]), "{packets:?}");
    }

    #[test]
    fn one_failed_poll_is_forgiven_and_too_many_give_up() {
        let mut s = device_steps();
        start_steps(&mut s, &regs_1000());
        for _ in 0..protocol::MAX_FAILED_POLLS {
            let _ = writeln!(s, "ctl_out 41 0c 0083 0000 21 -> ok");
            let _ = writeln!(s, "ctl_in 41 0c 0084 0000 1 -> err device \"endpoint stalled\"");
        }
        w(&mut s, 0x20, 0xc0);
        w(&mut s, 0x20, 0xc1);
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let packets = capture(&mut d);
        assert!(matches!(packets[..], [Packet::Header { .. }, Packet::End]), "{packets:?}");

        let mut s = device_steps();
        start_steps(&mut s, &regs_1000());
        let _ = writeln!(s, "ctl_out 41 0c 0083 0000 21 -> ok");
        let _ = writeln!(s, "ctl_in 41 0c 0084 0000 1 -> err device \"endpoint stalled\"");
        addresses_1000(&mut s);
        read_start(&mut s);
        for _ in 0..4 {
            bulk(&mut s, &stream_of(2048));
        }
        w(&mut s, 0x20, 0xe0);
        w(&mut s, 0x20, 0xc0);
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        assert_eq!(samples(&capture(&mut d)), 996);
    }

    #[test]
    fn an_unplug_while_polling_ends_at_once() {
        let mut s = device_steps();
        start_steps(&mut s, &regs_1000());
        let _ = writeln!(s, "ctl_out 41 0c 0083 0000 21 -> err disconnected");
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let packets = capture(&mut d);
        assert!(matches!(packets[..], [Packet::Header { .. }, Packet::End]), "{packets:?}");
    }

    #[test]
    fn a_stop_while_waiting_resets_the_board() {
        let s = {
            let mut s = device_steps();
            start_steps(&mut s, &regs_1000());
            w(&mut s, 0x20, 0xc0);
            w(&mut s, 0x20, 0xc1);
            s
        };
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let link = d.lock().take().unwrap();
        let (tx, rx) = sync_channel(16);
        // Set before the first poll: no status read at all.
        let stop = AtomicBool::new(true);
        let regs = regs_1000();
        let mut back = None;
        let mut give_back = |l: Link| back = Some(l);
        protocol::run(
            link,
            &regs,
            &PacketSink::new(tx, 0),
            &stop,
            &mut give_back,
        );
        assert!(back.is_some());
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        assert!(matches!(packets[..], [Packet::Header { .. }, Packet::End]), "{packets:?}");
    }

    #[test]
    fn a_stop_during_the_readout_ends_at_the_next_read() {
        let mut s = device_steps();
        start_steps(&mut s, &regs_1000());
        addresses_1000(&mut s);
        read_start(&mut s);
        for _ in 0..4 {
            bulk(&mut s, &stream_of(2048));
        }
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let link = d.lock().take().unwrap();
        // A queue of none holds the thread at each packet.
        let (tx, rx) = sync_channel(0);
        let stop = AtomicBool::new(false);
        let regs = regs_1000();
        let rest = std::thread::scope(|scope| {
            let stop = &stop;
            let regs = &regs;
            let worker = scope.spawn(move || {
                let mut give_back = |_: Link| {};
                protocol::run(
                    link,
                    regs,
                    &PacketSink::new(tx, 0),
                    stop,
                    &mut give_back,
                );
            });
            assert!(matches!(rx.recv().unwrap().1, Packet::Header { .. }));
            assert!(matches!(rx.recv().unwrap().1, Packet::Logic(_)));
            stop.store(true, Ordering::SeqCst);
            let rest: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
            worker.join().unwrap();
            rest
        });
        assert!(rest.len() <= 2, "at most the packet in hand and the end: {}", rest.len());
        assert!(matches!(rest.last(), Some(Packet::End)));
    }

    #[test]
    fn a_trigger_does_not_outlive_its_capture() {
        // analyzer.c:515-516 leaves the registers and analyzer.c:525 the
        // flag as they were when the next capture has no trigger.
        let empty = |s: &mut String| {
            r(s, 0x21, 2);
            r(s, 0x21, 2);
            address(s, 0x9b, 0);
            address(s, 0x96, 0);
            address(s, 0x90, 0);
        };
        let triggered = Regs {
            trigger_status: [1, 0, 0, 0, 0, 0, 0, 0],
            trigger_edge: 0x41,
            ramsize_trigger: 99,
            ..Regs::default()
        };
        let plain = Regs { ramsize_trigger: 99, ..Regs::default() };
        let mut s = device_steps();
        start_steps(&mut s, &triggered);
        empty(&mut s);
        start_steps(&mut s, &plain);
        empty(&mut s);
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(100), None).unwrap();
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::One);
        t.add(0, 1, TriggerMatch::Rising);
        d.set_trigger(Some(t)).unwrap();
        assert_eq!(capture(&mut d).len(), 2);
        assert_eq!(d.regs.trigger_edge, 0x41);
        d.set_trigger(None).unwrap();
        assert_eq!(capture(&mut d).len(), 2);
        assert_eq!(d.regs, plain);
    }

    #[test]
    fn a_set_during_a_capture_is_refused() {
        let mut s = device_steps();
        start_steps(&mut s, &regs_1000());
        addresses_1000(&mut s);
        read_start(&mut s);
        for _ in 0..4 {
            bulk(&mut s, &stream_of(2048));
        }
        w(&mut s, 0x20, 0xe0);
        w(&mut s, 0x20, 0xc0);
        let mut d = device(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let (tx, rx) = sync_channel(0);
        d.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(rx.recv().unwrap().1, Packet::Header { .. }));
        assert!(matches!(d.set(&Key::CaptureRatio, &Value::U64(1), None), Err(Error::Busy)));
        assert!(rx.iter().map(|(_, p)| p).any(|p| matches!(p, Packet::End)));
    }

    #[test]
    fn conn_is_named_when_given_or_when_there_is_a_choice() {
        let board = || {
            let t = Replay::from_text(&device_steps()).unwrap();
            ZeroplusDevice::from_transport(Box::new(t)).unwrap()
        };
        let mut one = [board()];
        name_conn(&mut one, false);
        assert_eq!(one[0].info().conn, None);
        name_conn(&mut one, true);
        assert_eq!(one[0].info().conn.as_deref(), Some("1.2"));
        let mut two = [board(), board()];
        name_conn(&mut two, false);
        assert_eq!(two[1].info().conn.as_deref(), Some("1.2"));
        let driver = ZeroplusDriver;
        assert!(driver.scan_options().is_empty());
        assert_eq!(driver.accepted_scan_options(), [Key::Conn]);
    }

    #[test]
    fn port_path_has_the_c_form() {
        let mut d = device(&device_steps());
        d.usb.ports = vec![1, 4];
        assert_eq!(port_path(&d.usb), "1-1.4");
    }
}
