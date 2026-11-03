// Counterpart in sigrok:
//   libsigrok/src/hardware/saleae-logic16/api.c,
//   libsigrok/src/hardware/saleae-logic16/protocol.c (the acquisition),
//   Copyright (C) 2013 Marcus Comstedt <marcus@mc.pp.se>,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Daniel Elstner,
//   Soeren Apel, Aurelien Jacobs, Martin Ling, Gerhard Sittig,
//   Romain Tartière, Tilman Sauerbeck, Andreas Piesk, Peter Zotov, Tim Hatch
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! saleae-logic16: the Saleae Logic16, an FX2 in front of an FPGA, 16
//! channels at 100 MHz / n or 160 MHz / n, at most 300 MHz over the
//! enabled channels.
//!
//! The FX2 comes up without firmware (`21a9:1001` without strings): the
//! scan uploads `saleae-logic16-fx2.fw` over [`crate::ezusb`] and the
//! device renumerates as "Saleae LLC" / "Logic S/16"; opening waits for
//! it. Opening also loads the FPGA's bitstream for the selected
//! `voltage_threshold` (see [`protocol`]), and a capture whose threshold
//! changed loads the other one. Triggers run on the host. Scan, open,
//! transfer sizes and the soft trigger follow `api.c` and `protocol.c`
//! step by step, so that the device sees the same transactions as under
//! sigrok-cli (checked with the emulator, `tools/emu/saleae-logic16.sh`).

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, SoftTriggerLogic, Trigger, TriggerMatch, Value, ValueList,
};
use crate::usb::{BulkStream, Transport, UsbDeviceInfo, UsbId};
use protocol::{Converter, Link, VoltageRange};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// The Logic16's id, before and after the firmware upload.
pub const ID: UsbId = UsbId { vendor: 0x21a9, product: 0x1001 };
const FX2_FIRMWARE: &str = "saleae-logic16-fx2.fw";
const CONFIGURATION: u8 = 1;
const INTERFACE: u8 = 0;
const ENDPOINT_IN: u8 = 0x82;
const RENUM_TIMEOUT: Duration = Duration::from_secs(3);
/// Consecutive empty or failed transfers after which the FX2 is taken
/// to have given up.
const MAX_EMPTY_TRANSFERS: usize = 64;

/// The sample rates `--show` lists; others of 100 MHz / n or 160 MHz /
/// n work as well, as in libsigrok.
pub const SAMPLERATES: [u64; 16] = [
    500_000,
    1_000_000,
    2_000_000,
    4_000_000,
    5_000_000,
    8_000_000,
    10_000_000,
    12_500_000,
    16_000_000,
    20_000_000,
    25_000_000,
    32_000_000,
    40_000_000,
    50_000_000,
    80_000_000,
    100_000_000,
];

/// The driver.
pub struct Logic16Driver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 7] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::VoltageThreshold, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
];
/// libsigrok's soft trigger: stages of level and edge matches only.
const TRIGGER_FEATURES: crate::core::TriggerFeatures = crate::core::TriggerFeatures {
    count: false,
    consecutive: false,
    duration: false,
    range: false,
    or: false,
    and: false,
    max_stages: 0,
};
static TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];

/// Whether `conn` is a spec `sr_usb_find()` understands: `VID.PID` in
/// hex or `BUS.ADDRESS`.
fn conn_parses(conn: &str) -> bool {
    conn.split_once('.').is_some_and(|(a, b)| {
        let hex = |s: &str| !s.is_empty() && s.len() <= 4 && u16::from_str_radix(s, 16).is_ok();
        (hex(a) && hex(b)) || (a.parse::<u8>().is_ok() && b.parse::<u8>().is_ok())
    })
}

impl Driver for Logic16Driver {
    fn id(&self) -> &'static str {
        "saleae-logic16"
    }

    fn long_name(&self) -> &'static str {
        "Saleae Logic16"
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
        if let Some(c) = &conn
            && !conn_parses(c)
        {
            // sr_usb_find() finds nothing, and the scan no device.
            log::error!("Could neither determine VID:PID nor bus.address numbers.");
            return Ok(Vec::new());
        }
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::find_devices(&[ID])? {
            if conn.as_deref().is_some_and(|c| !info.matches_conn(c)) {
                continue;
            }
            let mut dev = Logic16Device::new(info, None);
            if has_firmware(&dev.usb) {
                log::debug!("Found a Logic16 device.");
            } else {
                match upload(&dev.usb) {
                    Ok(()) => dev.fw_updated = Some(Instant::now()),
                    Err(e) => {
                        log::debug!("Firmware upload: {e}.");
                        log::error!("Firmware upload failed, name {FX2_FIRMWARE}.");
                    }
                }
                // The address after the renumeration is not known yet
                // (0xff in the C driver).
                dev.info.conn = None;
            }
            devices.push(Box::new(dev));
        }
        Ok(devices)
    }
}

/// `check_conf_profile()`: the device opened and its manufacturer, then
/// its product string read, "Saleae LLC" and "Logic S/16" when the
/// firmware runs. A device without strings (index 0) is read nothing.
fn has_firmware(info: &UsbDeviceInfo) -> bool {
    let Ok(mut t) = crate::usb::open(info) else { return false };
    let Ok([m, p, _]) = t.string_indexes() else { return false };
    crate::usb::read_string_ascii(t.as_mut(), m).is_ok_and(|s| s == "Saleae LLC")
        && crate::usb::read_string_ascii(t.as_mut(), p).is_ok_and(|s| s == "Logic S/16")
}

/// `ezusb_upload_firmware()`: open, `SET_CONFIGURATION` 1, the image
/// in chunks of 4096 bytes with the 8051 held in reset, release.
fn upload(info: &UsbDeviceInfo) -> Result<()> {
    let image = crate::core::firmware::read(
        FX2_FIRMWARE,
        "extract the Logic16 firmware with sigrok-util, see docs/DEVICES.md",
    )?;
    log::info!("Uploading firmware {FX2_FIRMWARE} to device on {}.", info.conn());
    let mut t = crate::usb::open(info)?;
    t.force_configuration(CONFIGURATION)?;
    crate::ezusb::upload_firmware(t.as_mut(), &image)
}

/// Settings kept between acquisitions.
#[derive(Clone, Debug)]
struct Settings {
    samplerate: u64,
    limit_samples: u64,
    capture_ratio: u64,
    voltage_range: VoltageRange,
}

/// One Logic16.
pub struct Logic16Device {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    settings: Settings,
    trigger: Option<Trigger>,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
    /// When the scan uploaded the firmware: the device renumerates and
    /// is looked for again on open.
    fw_updated: Option<Instant>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl Logic16Device {
    fn new(usb: UsbDeviceInfo, transport: Option<Box<dyn Transport>>) -> Self {
        Logic16Device {
            info: DeviceInfo {
                driver: "saleae-logic16",
                vendor: "Saleae".into(),
                model: "Logic16".into(),
                version: String::new(),
                serial: String::new(),
                conn: Some(usb.conn()),
            },
            usb,
            // Channels named "0" to "15", as channel_names[] of api.c.
            channels: (0..16).map(|i| Channel::new(i, ChannelKind::Logic, i.to_string())).collect(),
            settings: Settings {
                samplerate: SAMPLERATES[0],
                limit_samples: 0,
                capture_ratio: 0,
                voltage_range: VoltageRange::Low,
            },
            trigger: None,
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
            fw_updated: None,
        }
    }

    /// A device on an already open transport (a replayed recording),
    /// its firmware running.
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        if usb.id() != ID {
            return Err(Error::arg("not a Saleae Logic16"));
        }
        Ok(Self::new(usb, Some(transport)))
    }

    /// The mask of the enabled channels.
    fn channel_mask(&self) -> u16 {
        self.channels.iter().filter(|c| c.enabled).fold(0, |m, c| m | 1 << c.index)
    }

    /// `dev_acquisition_start()`: the rate checked, the FPGA set up, the
    /// transfers queued, then the start. A start that fails cancels the
    /// queued transfers as the stream drops.
    fn begin(&self, link: &mut Link) -> Result<(Box<dyn BulkStream>, Acquisition)> {
        let mask = self.channel_mask();
        let count = mask.count_ones();
        let rate = self.settings.samplerate;
        // logic16_setup_acquisition() refuses the rate before it sends
        // anything; the C code computes the transfer plan first and
        // divides by zero below 8000 bytes a second (api.c:544).
        let (clock_160, divisor) = protocol::clock_setting(rate, count)?;
        let (size, depth, timeout) = protocol::transfer_plan(rate, count)
            .ok_or_else(|| Error::Arg(format!("Unable to sample at {rate}Hz.")))?;
        // soft_trigger_logic_check() skips matches on disabled channels;
        // the samples are 16 bit whatever the channels.
        let limit = self.settings.limit_samples;
        let stl = self.trigger.as_ref().map(|t| {
            let stages = t
                .stages
                .iter()
                .map(|st| {
                    st.conditions
                        .iter()
                        .filter(|c| {
                            self.channels.iter().any(|ch| ch.index == c.channel && ch.enabled)
                        })
                        .map(|c| (c.channel, c.matches))
                        .collect()
                })
                .collect();
            let pre = self.settings.capture_ratio * limit / 100;
            SoftTriggerLogic::new(stages, 2, pre as usize)
        });
        link.setup_acquisition(clock_160, divisor, mask, self.settings.voltage_range)?;
        let stream = link.transport().bulk_in_stream(ENDPOINT_IN, size, depth)?;
        link.start_acquisition()?;
        log::debug!("Acquisition started: {depth} transfers of {size}.");
        Ok((stream, Acquisition { converter: Converter::new(mask), limit, timeout, stl }))
    }
}

/// Whether `d` is the device scanned as `usb`: by its hub ports, which
/// stay the same when it renumerates (`usb_get_port_path()`). A backend
/// that knows no ports falls back to the address, or after an upload to
/// a device on the same bus with the firmware's strings.
fn same_device(d: &UsbDeviceInfo, usb: &UsbDeviceInfo, after_upload: bool) -> bool {
    if d.bus != usb.bus {
        return false;
    }
    if !usb.ports.is_empty() && !d.ports.is_empty() {
        return d.ports == usb.ports;
    }
    if after_upload {
        d.manufacturer.as_deref() == Some("Saleae LLC")
    } else {
        d.address == usb.address
    }
}

/// `logic16_dev_open()`: the device at the scanned port, opened, its
/// interface claimed and the device set up ([`Link::init`]).
fn try_open(
    usb: &UsbDeviceInfo,
    after_upload: bool,
    range: VoltageRange,
) -> Result<(UsbDeviceInfo, Link)> {
    let found =
        crate::usb::find_devices(&[ID])?.into_iter().find(|d| same_device(d, usb, after_upload));
    let info = found.ok_or_else(|| Error::device("device not found"))?;
    let t = crate::usb::open(&info)?;
    Ok((info, set_up(t, range)?))
}

/// The claim and `logic16_init_device()`. `set_configuration(1)` comes
/// first; it sends nothing when the configuration is active (always on
/// Linux) but macOS can leave a device unconfigured right after its
/// renumeration.
fn set_up(mut t: Box<dyn Transport>, range: VoltageRange) -> Result<Link> {
    t.set_configuration(CONFIGURATION)?;
    t.claim_interface(INTERFACE).map_err(|e| match e {
        Error::Busy => {
            let msg = "Unable to claim USB interface. Another program or driver has already \
                       claimed it.";
            log::error!("{msg}");
            Error::device(msg)
        }
        e => e,
    })?;
    let mut link = Link::new(t);
    link.init(range).inspect_err(|e| {
        log::debug!("Init: {e}.");
        log::error!("Failed to init device.");
    })?;
    Ok(link)
}

impl Device for Logic16Device {
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
            // Unknown until the device came back after the upload.
            Key::Conn => match &self.info.conn {
                Some(c) => Value::Str(c.clone()),
                None => return Err(Error::device("the device has not renumerated yet")),
            },
            Key::Samplerate => Value::U64(self.settings.samplerate),
            Key::CaptureRatio => Value::U64(self.settings.capture_ratio),
            Key::VoltageThreshold => {
                let (low, high) = self.settings.voltage_range.threshold();
                Value::RangeF64(low, high)
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            // Any rate: the capture refuses those the clocks cannot
            // divide to, as in libsigrok.
            Key::Samplerate => self.settings.samplerate = value.as_u64().ok_or_else(bad)?,
            Key::LimitSamples => self.settings.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::CaptureRatio => {
                let r = value.as_u64().ok_or_else(bad)?;
                // libsigrok takes any value; above 100 its pre-trigger
                // count passes the sample limit.
                if r > 100 {
                    return Err(Error::arg("capture ratio is a percentage"));
                }
                self.settings.capture_ratio = r;
            }
            Key::VoltageThreshold => {
                let Value::RangeF64(low, high) = value else { return Err(bad()) };
                self.settings.voltage_range =
                    VoltageRange::from_threshold(*low, *high).ok_or_else(bad)?;
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
            Key::Samplerate => ValueList::U64(SAMPLERATES.to_vec()),
            Key::VoltageThreshold => {
                ValueList::RangeF64(VoltageRange::ALL.iter().map(|r| r.threshold()).collect())
            }
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let range = self.settings.voltage_range;
        let unable = |e: Error| {
            log::error!("Unable to open device.");
            e
        };
        let link = if let Some(t) = self.pending.take() {
            set_up(t, range).map_err(unable)?
        } else if let Some(uploaded) = self.fw_updated {
            // dev_open(): wait for the FX2 to renumerate, trying every
            // 100 ms until 3 s after the upload.
            log::info!("Waiting for device to reset.");
            std::thread::sleep(Duration::from_millis(300));
            let (info, link) = loop {
                match try_open(&self.usb, true, range) {
                    Ok(found) => break found,
                    Err(e) => {
                        std::thread::sleep(Duration::from_millis(100));
                        if uploaded.elapsed() >= RENUM_TIMEOUT {
                            log::debug!("Last attempt: {e}.");
                            log::error!("Device failed to renumerate.");
                            return Err(Error::device("the device failed to renumerate"));
                        }
                    }
                }
            };
            log::info!("Device came back after {} ms.", uploaded.elapsed().as_millis());
            self.fw_updated = None;
            self.info.conn = Some(info.conn());
            self.usb = info;
            link
        } else {
            log::info!("Firmware upload was not needed.");
            let (info, link) = try_open(&self.usb, false, range).map_err(unable)?;
            self.info.conn = Some(info.conn());
            self.usb = info;
            link
        };
        *self.link.lock().expect("link lock") = Some(link);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        drop(self.link.lock().expect("link lock").take());
        Ok(())
    }

    fn trigger_features(&self) -> crate::core::TriggerFeatures {
        TRIGGER_FEATURES
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&TRIGGER_FEATURES)?;
        }
        self.trigger = trigger;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let link = self.link.lock().expect("link lock").take();
        let Some(mut link) = link else {
            return Err(Error::device("device is not open"));
        };
        match self.begin(&mut link) {
            Ok((stream, acq)) => {
                let slot = Arc::clone(&self.link);
                let stop = Arc::new(AtomicBool::new(false));
                let stop2 = Arc::clone(&stop);
                let handle = std::thread::Builder::new()
                    .name("saleae-logic16 acquisition".into())
                    .spawn(move || {
                        acq.run(stream, &mut link, &sink, &stop2);
                        sink.send(Packet::End);
                        *slot.lock().expect("link lock") = Some(link);
                    })
                    .map_err(Error::Io)?;
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                // libsigrok's session stops a device whose start failed
                // as well (session.c:831-835), which sends the abort
                // commands (dev_acquisition_stop()); a setup cut short
                // leaves the FPGA idle so.
                if let Err(e) = link.abort_acquisition() {
                    log::debug!("Stopping after a failed start: {e}.");
                }
                *self.link.lock().expect("link lock") = Some(link);
                Err(e)
            }
        }
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        Ok(())
    }
}

/// One acquisition, on its own thread: `logic16_receive_transfer()`.
struct Acquisition {
    converter: Converter,
    limit: u64,
    timeout: Duration,
    stl: Option<SoftTriggerLogic>,
}

/// What a transfer leaves the acquisition to do.
#[derive(Debug, PartialEq, Eq)]
enum Next {
    Continue,
    End,
}

/// The counters of `struct dev_context` a capture keeps.
struct State {
    sent: u64,
    empty: usize,
    fired: bool,
}

impl Acquisition {
    /// Takes transfers until the sample limit, the FX2 giving up, the
    /// device gone or the stop flag; then the abort commands
    /// (`logic16_abort_acquisition()`), and the stream drops, which
    /// cancels the transfers still queued.
    fn run(
        mut self,
        mut stream: Box<dyn BulkStream>,
        link: &mut Link,
        sink: &PacketSink,
        stop: &AtomicBool,
    ) {
        sink.send(Packet::header());
        let mut state = State { sent: 0, empty: 0, fired: self.stl.is_none() };
        while !stop.load(Ordering::SeqCst) {
            let transfer = match stream.next(self.timeout) {
                Err(Error::Disconnected) => {
                    log::error!("Device has been disconnected.");
                    break;
                }
                other => other,
            };
            if self.transfer(transfer, &mut state, sink) == Next::End {
                break;
            }
        }
        if let Err(e) = link.abort_acquisition() {
            log::debug!("Stopping the acquisition: {e}.");
        }
        drop(stream);
    }

    /// One transfer, completed (`Ok`) or failed, as
    /// `logic16_receive_transfer()` takes it.
    fn transfer(&mut self, transfer: Result<Vec<u8>>, s: &mut State, sink: &PacketSink) -> Next {
        // A timeout counts as an empty transfer, any other failure as a
        // failed one.
        let (data, mut failed) = match transfer {
            Ok(d) => (d, false),
            Err(Error::Timeout) => (Vec::new(), false),
            Err(e) => {
                log::debug!("Transfer failed: {e}.");
                (Vec::new(), true)
            }
        };
        if data.len() % 2 != 0 {
            log::error!("Got an odd number of bytes from the device. This should not happen.");
            failed = true;
            s.empty = MAX_EMPTY_TRANSFERS;
        }
        if data.is_empty() || failed {
            s.empty += 1;
            // Past the limit the FX2 gave up.
            return if s.empty > MAX_EMPTY_TRANSFERS { Next::End } else { Next::Continue };
        }
        s.empty = 0;
        let mut samples = Vec::new();
        let n = self.converter.convert(&data, &mut samples) as u64;
        if n == 0 {
            return Next::Continue;
        }
        let limit = self.limit;
        let room = |sent: u64| if limit > 0 { limit.saturating_sub(sent) } else { u64::MAX };
        if s.fired {
            let n = n.min(room(s.sent));
            sink.send(Packet::logic(2, samples[..n as usize * 2].to_vec()));
            s.sent += n;
        } else if let Some(stl) = self.stl.as_mut()
            && let Some((offset, pre)) = stl.check(&samples)
        {
            for p in pre {
                s.sent += (p.len() / 2) as u64;
                sink.send(Packet::logic(2, p));
            }
            sink.send(Packet::Trigger);
            let k = (n - offset as u64).min(room(s.sent));
            let from = offset * 2;
            sink.send(Packet::logic(2, samples[from..from + k as usize * 2].to_vec()));
            s.sent += k;
            s.fired = true;
        }
        if limit > 0 && s.sent >= limit { Next::End } else { Next::Continue }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collect(rx: &std::sync::mpsc::Receiver<(usize, Packet)>) -> Vec<Packet> {
        rx.try_iter().map(|(_, p)| p).collect()
    }

    fn acquisition(limit: u64, stl: Option<SoftTriggerLogic>) -> Acquisition {
        Acquisition { converter: Converter::new(0b1), limit, timeout: Duration::ZERO, stl }
    }

    /// Words of channel 0 alone, each 16 samples, first in bit 15.
    fn words(words: &[u16]) -> Vec<u8> {
        words.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    /// The 16 bit samples of one word of channel 0.
    fn bits(w: u16) -> Vec<u8> {
        (0..16).flat_map(|i| [u8::from(w & (0x8000 >> i) != 0), 0]).collect()
    }

    #[test]
    fn the_limit_cuts_the_last_transfer() {
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let sink = PacketSink::new(tx, 0);
        let mut a = acquisition(20, None);
        let mut s = State { sent: 0, empty: 0, fired: true };
        assert_eq!(a.transfer(Ok(words(&[0xff00])), &mut s, &sink), Next::Continue);
        assert_eq!(a.transfer(Ok(words(&[0xf000])), &mut s, &sink), Next::End);
        assert_eq!(
            collect(&rx),
            [Packet::logic(2, bits(0xff00)), Packet::logic(2, bits(0xf000)[..8].to_vec())]
        );
    }

    #[test]
    fn empty_failed_and_odd_transfers() {
        let (tx, _rx) = std::sync::mpsc::sync_channel(64);
        let sink = PacketSink::new(tx, 0);
        let mut a = acquisition(0, None);
        let mut s = State { sent: 0, empty: 0, fired: true };
        for _ in 0..MAX_EMPTY_TRANSFERS {
            assert_eq!(a.transfer(Ok(Vec::new()), &mut s, &sink), Next::Continue);
        }
        assert_eq!(a.transfer(Err(Error::device("stall")), &mut s, &sink), Next::End);
        // Data resets the count; an odd length ends at once.
        let mut s = State { sent: 0, empty: 10, fired: true };
        assert_eq!(a.transfer(Ok(words(&[1])), &mut s, &sink), Next::Continue);
        assert_eq!(s.empty, 0);
        assert_eq!(a.transfer(Ok(vec![1, 2, 3]), &mut s, &sink), Next::End);
    }

    #[test]
    fn a_trigger_sends_the_pre_trigger_samples_first() {
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let sink = PacketSink::new(tx, 0);
        // Channel 0 rising, 2 samples before it, 6 in all.
        let stl = SoftTriggerLogic::new(vec![vec![(0, TriggerMatch::Rising)]], 2, 2);
        let mut a = acquisition(6, Some(stl));
        let mut s = State { sent: 0, empty: 0, fired: false };
        // The rise at sample 4 of the word.
        assert_eq!(a.transfer(Ok(words(&[0x0fff])), &mut s, &sink), Next::End);
        let all = bits(0x0fff);
        assert_eq!(
            collect(&rx),
            [
                Packet::logic(2, all[4..8].to_vec()),
                Packet::Trigger,
                Packet::logic(2, all[8..16].to_vec()),
            ]
        );
    }

    #[test]
    fn conn_specs() {
        assert!(conn_parses("21a9.1001"));
        assert!(conn_parses("3.7"));
        assert!(!conn_parses("nonsense"));
        assert!(!conn_parses("1.x"));
    }

    #[test]
    fn options_follow_the_c_driver() {
        let mut d = Logic16Device::new(UsbDeviceInfo::default(), None);
        assert_eq!(d.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(0.7, 1.4));
        d.set(&Key::VoltageThreshold, &Value::RangeF64(1.4, 3.6), None).unwrap();
        assert_eq!(d.settings.voltage_range, VoltageRange::High);
        assert!(d.set(&Key::VoltageThreshold, &Value::RangeF64(1.0, 2.0), None).is_err());
        // Any rate is taken; the capture refuses it.
        d.set(&Key::Samplerate, &Value::U64(3_000_000), None).unwrap();
        assert_eq!(d.channel_mask(), 0xffff);
        d.enable_channel(3, false).unwrap();
        assert_eq!(d.channel_mask(), 0xfff7);
        let names: Vec<&str> = d.channels.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names[..3], ["0", "1", "2"]);
    }
}
