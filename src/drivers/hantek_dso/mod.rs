// Counterpart in sigrok:
//   libsigrok/src/hardware/hantek-dso/api.c,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Philipp Marek,
//   Daniel Elstner, Gerhard Sittig, Martin Ling, Soeren Apel,
//   Aurelien Jacobs, Daniel Glöckner, Miroslav Sustek, Romain Tartière,
//   Valentin Ochs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! hantek-dso: the Hantek DSO-2090, 2150, 2250, 5200 and 5200A USB
//! oscilloscopes, an FX2 that takes a firmware and renumerates.
//!
//! The scan uploads `hantek-dso-<model>.fw` to a board that answers with
//! the cold boot ids; the open waits for it to come back at the same
//! ports with the firmware's. A capture is the C driver's state machine,
//! on a thread: the capture state polled about every millisecond (each
//! poll is a "begin" request, a command and an answer), three empty
//! states restart the capture with a forced trigger unless a trigger
//! source is set, and a ready frame is fetched. Scan, open, setup and
//! polling follow `api.c` and `protocol.c`, so that the device sees the
//! same transactions as under sigrok-cli (checked with the emulator,
//! `tools/emu/hantek-dso.sh`).
//!
//! A frame is read one packet at a time, as many as the C driver submits
//! at once. A transfer of one packet cannot end with part of the data, so
//! blocking reads give up nothing a queue would hold; each has the C
//! driver's 40 ms. Unlike the C driver's, an empty or timed out transfer
//! does not lose the frame: the bytes are still in the device, the read
//! is repeated.
//!
//! Where the C driver is wrong the port does otherwise: `num_hdiv` and
//! `num_vdiv` are answered (`config_get()` fills them in, then returns
//! `SR_ERR_NA`); a frame without pre-trigger samples sends no empty
//! analog packet, on which `output/analog.c` frees its buffer twice; and
//! no error is retried for ever. A disconnect ends the capture at once,
//! and so do [`MAX_FAILURES`] failures in a row of any one kind, which a
//! DSO-5200's 9 bit frames ("Not yet supported.") count as. A trigger
//! that never fires is no error, and is waited for until a stop, as a
//! scope does.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error,
    Key, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, UsbId, libusb_error_name, split_conn};
use protocol::{Frame, Levels, Link, Profile, Settings};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// The driver.
pub struct HantekDsoDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::Oscilloscope];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 12] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::Timebase, Access::GET_SET_LIST),
    OptionDesc::new(Key::NumHdiv, Access::GET),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::TriggerSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSlope, Access::GET_SET),
    OptionDesc::new(Key::Buffersize, Access::GET_SET_LIST),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::NumVdiv, Access::GET),
    OptionDesc::new(Key::TriggerLevel, Access::GET_SET),
];
static GROUP_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Vdiv, Access::GET_SET_LIST),
    OptionDesc::new(Key::Coupling, Access::GET_SET_LIST),
    OptionDesc::new(Key::Filter, Access::GET_SET_LIST),
];
/// `channel_names[]`.
const CHANNEL_NAMES: [&str; 2] = ["CH1", "CH2"];
/// `NUM_TIMEBASE`.
const NUM_HDIV: i32 = 10;
/// `NUM_VDIV`.
const NUM_VDIV: i32 = 8;
/// libsigrok's poll interval (`TICK`).
const TICK: Duration = Duration::from_millis(1);
/// What `dev_open()` waits for the FX2 to be gone from the bus.
const RENUM_GONE_DELAY: Duration = Duration::from_millis(300);
/// `MAX_RENUM_DELAY_MS`.
const RENUM_TIMEOUT: Duration = Duration::from_secs(3);
const RENUM_POLL_INTERVAL: Duration = Duration::from_millis(100);
/// Failures of one kind in a row after which a capture ends, the bound
/// fx2lafw has as `MAX_EMPTY_TRANSFERS`; the C driver retries for ever.
const MAX_FAILURES: usize = 64;

/// `usb_get_port_path()`: `BUS-PORT.PORT...`.
fn port_path(d: &UsbDeviceInfo) -> String {
    let ports: Vec<String> = d.ports.iter().map(u8::to_string).collect();
    format!("{}-{}", d.bus, ports.join("."))
}

/// Whether `d` is the device scanned as `usb`: by its hub ports, which
/// stay the same when it renumerates. A backend that knows no ports falls
/// back to the bus, and to the address too when it did not renumerate.
fn same_device(d: &UsbDeviceInfo, usb: &UsbDeviceInfo, renumerated: bool) -> bool {
    if d.bus != usb.bus {
        return false;
    }
    if !usb.ports.is_empty() && !d.ports.is_empty() {
        return d.ports == usb.ports;
    }
    renumerated || d.address == usb.address
}

/// `ezusb_upload_firmware()`.
fn upload(info: &UsbDeviceInfo, profile: &Profile) -> Result<()> {
    log::info!("uploading firmware to device on {}.{}", info.bus, info.address);
    let image = crate::core::firmware::read(profile.firmware, "see docs/DEVICES.md")?;
    let mut t = crate::usb::open(info).inspect_err(|e| {
        log::error!("failed to open device: {}.", libusb_error_name(e));
    })?;
    t.force_configuration(protocol::CONFIGURATION).inspect_err(|e| {
        log::error!("Unable to set configuration: {}", libusb_error_name(e));
    })?;
    crate::ezusb::upload_firmware_logged(t.as_mut(), profile.firmware, &image)
}

impl Driver for HantekDsoDriver {
    fn id(&self) -> &'static str {
        "hantek-dso"
    }

    fn long_name(&self) -> &'static str {
        "Hantek DSO"
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
        let ids: Vec<UsbId> = protocol::PROFILES.iter().flat_map(|p| [p.orig, p.fw]).collect();
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::find_devices(&ids)? {
            if conn.is_some_and(|c| !c.matches(&info)) {
                continue;
            }
            let Some(profile) =
                protocol::PROFILES.iter().find(|p| info.id() == p.orig || info.id() == p.fw)
            else {
                continue;
            };
            log::debug!("Found a {} {}.", profile.vendor, profile.model);
            let cold = info.id() == profile.orig;
            let mut dev = HantekDsoDevice::new(info, profile);
            if cold {
                match upload(&dev.usb, profile) {
                    Ok(()) => dev.fw_updated = Some(Instant::now()),
                    Err(e) => {
                        log::debug!("Firmware upload: {e}.");
                        log::error!("Firmware upload failed, name {}", profile.firmware);
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

/// One scope.
pub struct HantekDsoDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    profile: &'static Profile,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    settings: Settings,
    /// When the firmware was uploaded, until the device came back.
    fw_updated: Option<Instant>,
    /// The calibration of the last capture, which `devc` keeps in C.
    levels: Levels,
    /// A transport handed in (a replay).
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// Errors of a configuration request that does not apply, as libsigrok's
/// `SR_ERR_NA`.
fn not_applicable() -> Error {
    Error::not_supported("not applicable")
}

/// `dso_open()`: the device at the scanned ports with the firmware's
/// ids, opened and its endpoints checked.
fn dso_open(
    usb: &UsbDeviceInfo,
    profile: &Profile,
    renumerated: bool,
) -> Result<(UsbDeviceInfo, Box<dyn Transport>, usize)> {
    for d in crate::usb::find_devices(&[profile.fw])? {
        if !same_device(&d, usb, renumerated) {
            continue;
        }
        // Whatever comes of it, this was the device.
        let mut t = crate::usb::open(&d).inspect_err(|e| {
            log::error!("Failed to open device: {}.", libusb_error_name(e));
        })?;
        let mps = t.interfaces().map_or(0, |i| protocol::endpoint_packet_size(&i));
        if mps == 0 {
            log::error!("Wrong endpoint profile.");
            return Err(Error::device("wrong endpoint profile"));
        }
        log::info!(
            "Opened device on {} (logical) / {} (physical) interface {}.",
            d.conn(),
            port_path(&d),
            protocol::INTERFACE
        );
        return Ok((d, t, mps));
    }
    Err(Error::device("device not found"))
}

impl HantekDsoDevice {
    fn new(usb: UsbDeviceInfo, profile: &'static Profile) -> Self {
        let channels = CHANNEL_NAMES
            .iter()
            .enumerate()
            .map(|(i, name)| Channel::new(i, ChannelKind::Analog, *name))
            .collect();
        let groups = CHANNEL_NAMES
            .iter()
            .enumerate()
            .map(|(i, name)| ChannelGroup::new(*name, vec![i]))
            .collect();
        HantekDsoDevice {
            info: DeviceInfo {
                driver: "hantek-dso",
                vendor: profile.vendor.into(),
                model: profile.model.into(),
                version: String::new(),
                serial: String::new(),
                conn: Some(usb.conn()),
            },
            usb,
            profile,
            channels,
            groups,
            settings: Settings::default(),
            fw_updated: None,
            levels: [[[0; 2]; 9]; 2],
            pending: None,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording), its
    /// firmware running.
    ///
    /// # Errors
    ///
    /// When it is no scope of the driver's.
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        let Some(profile) = protocol::PROFILES.iter().find(|p| p.fw == usb.id()) else {
            return Err(Error::arg("not a Hantek DSO with its firmware running"));
        };
        let mut d = Self::new(usb, profile);
        d.pending = Some(transport);
        Ok(d)
    }

    /// The channel `name` stands for.
    fn group(&self, name: Option<&str>) -> Result<Option<usize>> {
        let Some(name) = name else { return Ok(None) };
        match self.groups.iter().position(|g| g.name.eq_ignore_ascii_case(name)) {
            Some(i) => Ok(Some(i)),
            None => Err(Error::ChannelGroup(name.to_owned())),
        }
    }

    /// Whether the acquisition owns the device.
    fn capturing(&self) -> bool {
        self.worker.is_some() && self.link.lock().expect("link lock").is_none()
    }

    /// A setting a capture cannot take: it would have to reach the
    /// device, or the thread that holds its own copy. Refused rather
    /// than stored unsent.
    fn refuse_while_capturing(&self) -> Result<()> {
        if self.capturing() { Err(Error::Busy) } else { Ok(()) }
    }

    /// `f` on the device when it is open; a closed one takes the setting
    /// at the next capture.
    fn when_open(&self, f: impl FnOnce(&mut Link, &Settings) -> Result<()>) -> Result<()> {
        match self.link.lock().expect("link lock").as_mut() {
            Some(link) => f(link, &self.settings),
            None => Ok(()),
        }
    }

    /// `dev_open()`, up to the claim.
    fn find(&mut self) -> Result<(UsbDeviceInfo, Box<dyn Transport>, usize)> {
        let Some(uploaded) = self.fw_updated else {
            return dso_open(&self.usb, self.profile, false);
        };
        log::info!("Waiting for device to reset.");
        std::thread::sleep(RENUM_GONE_DELAY);
        let mut waited = Duration::ZERO;
        let found = loop {
            if waited >= RENUM_TIMEOUT {
                break Err(Error::device("the device failed to renumerate"));
            }
            match dso_open(&self.usb, self.profile, true) {
                Ok(f) => break Ok(f),
                Err(e) => log::trace!("Not yet: {e}."),
            }
            std::thread::sleep(RENUM_POLL_INTERVAL);
            waited = uploaded.elapsed();
            log::trace!("Waited {} ms.", waited.as_millis());
        };
        log::info!("Device came back after {} ms.", waited.as_millis());
        found
    }
}

impl Device for HantekDsoDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &self.groups
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
        Ok(match self.group(group)? {
            None => DEV_OPTIONS.to_vec(),
            Some(_) => GROUP_OPTIONS.to_vec(),
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let s = &self.settings;
        Ok(match (self.group(group)?, key) {
            // config_get() fills these in and goes on to return SR_ERR_NA
            // (api.c:430-435, 482-483).
            (None, Key::NumHdiv) => Value::I32(NUM_HDIV),
            (None, Key::NumVdiv) => Value::I32(NUM_VDIV),
            (None, Key::TriggerLevel) => Value::F64(f64::from(s.voffset_trigger)),
            // Unknown until the device came back after the upload.
            (None, Key::Conn) => match &self.info.conn {
                Some(c) => Value::Str(c.clone()),
                None => return Err(Error::device("the device has not renumerated yet")),
            },
            (None, Key::Timebase) => {
                let (p, q) = protocol::TIMEBASES[s.timebase];
                Value::Rational(p, q)
            }
            (None, Key::Samplerate) => Value::U64(s.samplerate),
            (None, Key::Buffersize) => Value::U64(s.framesize),
            (None, Key::TriggerSource) => match s.trigger_source {
                Some(i) => Value::Str(protocol::TRIGGER_SOURCES[i].to_owned()),
                None => return Err(not_applicable()),
            },
            (None, Key::TriggerSlope) => {
                Value::Str(protocol::TRIGGER_SLOPES[usize::from(s.slope_negative)].to_owned())
            }
            (None, Key::CaptureRatio) => Value::U64(s.capture_ratio),
            (None, Key::LimitFrames) => Value::U64(s.limit_frames),
            (Some(c), Key::Filter) => Value::Bool(s.filter[c]),
            (Some(c), Key::Vdiv) => {
                let (p, q) = protocol::VDIVS[s.voltage[c]];
                Value::Rational(p, q)
            }
            (Some(c), Key::Coupling) => Value::Str(protocol::COUPLINGS[s.coupling[c]].to_owned()),
            (Some(_), _) => return Err(not_applicable()),
            (None, other) => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let str_index = |list: &[&str]| {
            let v = value.as_str().ok_or_else(bad)?;
            list.iter().position(|s| *s == v).ok_or_else(bad)
        };
        let tuple_index = |list: &[(u64, u64)]| {
            let Value::Rational(p, q) = value else { return Err(bad()) };
            // The parser reduces its fractions: 10 us comes as 1/100000.
            list.iter().position(|(a, b)| a * q == p * b).ok_or_else(bad)
        };
        match (self.group(group)?, key) {
            (None, Key::LimitFrames) => {
                self.refuse_while_capturing()?;
                self.settings.limit_frames = value.as_u64().ok_or_else(bad)?;
            }
            (None, Key::TriggerLevel) => {
                #[allow(clippy::cast_possible_truncation)]
                let level = value.as_f64().ok_or_else(bad)? as f32;
                if !(0.0..=1.0).contains(&level) {
                    log::error!("Trigger level must be in [0.0,1.0].");
                    return Err(bad());
                }
                self.refuse_while_capturing()?;
                self.settings.voffset_trigger = level;
                self.when_open(Link::set_voffsets)?;
            }
            (None, Key::TriggerSlope) => {
                self.settings.slope_negative = str_index(&protocol::TRIGGER_SLOPES)? == 1;
            }
            (None, Key::CaptureRatio) => {
                // sr_config_set() refuses it before the driver sees it
                // (hwdriver.c, check_key()).
                let r = value.as_u64().ok_or_else(bad)?;
                if r > 100 {
                    log::error!("Capture ratio must be 0..100.");
                    return Err(bad());
                }
                self.settings.capture_ratio = r;
            }
            (None, Key::Buffersize) => {
                let v = value.as_u64().ok_or_else(bad)?;
                if !self.profile.buffersizes.contains(&v) {
                    return Err(bad());
                }
                self.settings.framesize = v;
            }
            (None, Key::Timebase) => self.settings.timebase = tuple_index(&protocol::TIMEBASES)?,
            (None, Key::Samplerate) => {
                let v = value.as_u64().ok_or_else(bad)?;
                if !protocol::SAMPLERATES.contains(&v) {
                    return Err(bad());
                }
                self.refuse_while_capturing()?;
                self.settings.samplerate = v;
                self.when_open(Link::set_trigger_samplerate)?;
            }
            (None, Key::TriggerSource) => {
                self.settings.trigger_source = Some(str_index(&protocol::TRIGGER_SOURCES)?);
            }
            (Some(c), Key::Filter) => {
                self.settings.filter[c] = value.as_bool().ok_or_else(bad)?;
            }
            (Some(c), Key::Vdiv) => self.settings.voltage[c] = tuple_index(&protocol::VDIVS)?,
            (Some(c), Key::Coupling) => {
                self.settings.coupling[c] = str_index(&protocol::COUPLINGS)?;
            }
            (Some(_), _) => return Err(not_applicable()),
            (None, other) => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let strs = |list: &[&str]| ValueList::Str(list.iter().map(|s| (*s).to_owned()).collect());
        Ok(match (self.group(group)?, key) {
            (None, Key::Buffersize) => ValueList::U64(self.profile.buffersizes.to_vec()),
            (None, Key::Samplerate) => ValueList::U64(protocol::SAMPLERATES.to_vec()),
            (None, Key::Timebase) => ValueList::Rational(protocol::TIMEBASES.to_vec()),
            (None, Key::TriggerSource) => strs(&protocol::TRIGGER_SOURCES),
            (None, Key::TriggerSlope) => strs(&protocol::TRIGGER_SLOPES),
            (Some(_), Key::Coupling) => strs(&protocol::COUPLINGS),
            (Some(_), Key::Vdiv) => ValueList::Rational(protocol::VDIVS.to_vec()),
            (Some(_), _) => return Err(not_applicable()),
            (None, other) => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `dev_open()`: the device found, waiting for it after an upload,
    /// and the interface claimed.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let found = if let Some(mut t) = self.pending.take() {
            let usb = t.info().clone();
            t.interfaces().map(|i| (usb, t, protocol::endpoint_packet_size(&i)))
        } else {
            self.find()
        };
        let (usb, mut t, mps) = found.inspect_err(|_| log::error!("Unable to open device."))?;
        t.claim_interface(protocol::INTERFACE).inspect_err(|e| {
            log::error!("Unable to claim interface: {}.", libusb_error_name(e));
        })?;
        self.fw_updated = None;
        self.info.conn = Some(usb.conn());
        self.usb = usb;
        let link = Link::new(t, self.profile.fw.product, mps, self.levels);
        *self.link.lock().expect("link lock") = Some(link);
        Ok(())
    }

    /// `dev_close()`.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(w) = self.worker.take() {
            let _ = w.handle.join();
        }
        if let Some(link) = self.link.lock().expect("link lock").take() {
            log::info!(
                "Closing device on {} (logical) / {} (physical) interface {}.",
                self.usb.conn(),
                port_path(&self.usb),
                protocol::INTERFACE
            );
            self.levels = link.levels;
            link.close();
        }
        Ok(())
    }

    /// `dev_acquisition_start()`: the setup and the capture start, then
    /// the header from the thread.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        match self.worker.take() {
            Some(w) if w.handle.is_finished() => {
                let _ = w.handle.join();
            }
            Some(w) => {
                self.worker = Some(w);
                return Err(Error::Busy);
            }
            None => {}
        }
        let link = self.link.lock().expect("link lock").take();
        let Some(mut link) = link else {
            return Err(Error::device("device is not open"));
        };
        // configure_channels().
        for (i, c) in self.channels.iter().enumerate() {
            self.settings.ch_enabled[i] = c.enabled;
        }
        let settings = self.settings.clone();
        if let Err(e) = link.init(&settings).and_then(|()| link.capture_start()) {
            *self.link.lock().expect("link lock") = Some(link);
            return Err(e);
        }
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("hantek-dso acquisition".into())
            .spawn(move || {
                Acquisition::new(settings).run(&mut link, &sink, &stop2);
                // The link first: whoever sees the end may use it.
                *slot.lock().expect("link lock") = Some(link);
                sink.send(Packet::End);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the acquisition to end, without waiting: the thread must be
    /// able to hand its packets to the session calling this.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// `MAX_FAILURES` of one kind in a row.
#[derive(Default)]
struct Failures(usize);

impl Failures {
    /// One more failure: whether that is too many, or the device gone.
    fn failed(&mut self, e: &Error) -> bool {
        if matches!(e, Error::Disconnected) {
            log::error!("Lost communication to USB device.");
            return true;
        }
        self.0 += 1;
        if self.0 >= MAX_FAILURES {
            log::error!("{MAX_FAILURES} USB requests in a row failed, giving up.");
            return true;
        }
        false
    }
}

/// What a tick of the acquisition leads to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Next {
    /// `CAPTURE`: poll.
    Poll,
    /// `NEW_CAPTURE`: start the next frame's capture.
    NewCapture,
    /// `STOPPING`.
    End,
}

/// One acquisition, on its own thread: `handle_event()`.
struct Acquisition {
    settings: Settings,
    num_frames: u64,
    capture_empty_count: u32,
    /// Polls that failed, or answered what cannot go on.
    polls: Failures,
    /// Capture starts and channel data requests that failed.
    commands: Failures,
    /// A 9 bit frame's message was logged for this run of them.
    told_9bit: bool,
}

impl Acquisition {
    fn new(settings: Settings) -> Self {
        Acquisition {
            settings,
            num_frames: 0,
            capture_empty_count: 0,
            polls: Failures::default(),
            commands: Failures::default(),
            told_9bit: false,
        }
    }

    /// The header, then the state machine, until the frame limit, a stop
    /// or too many failures. [`Packet::End`] is the caller's.
    fn run(mut self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) {
        if !sink.send(Packet::header()) {
            return;
        }
        let mut next = Next::Poll;
        while next != Next::End && !stop.load(Ordering::SeqCst) {
            next = match next {
                Next::NewCapture => self.new_capture(link),
                _ => self.poll(link, sink, stop),
            };
            if next != Next::End {
                std::thread::sleep(TICK);
            }
        }
        log::debug!("Stopping acquisition.");
    }

    /// The capture started, the trigger enabled, and forced unless a
    /// source is set.
    fn restart(&self, link: &mut Link) -> Result<()> {
        link.capture_start()?;
        link.enable_trigger()?;
        if self.settings.trigger_source.is_none() {
            link.force_trigger()?;
        }
        log::debug!("Successfully requested next chunk.");
        Ok(())
    }

    /// `NEW_CAPTURE`: tried again at the next tick when it fails.
    fn new_capture(&mut self, link: &mut Link) -> Next {
        match self.restart(link) {
            Ok(()) => {
                self.commands.0 = 0;
                Next::Poll
            }
            Err(e) if self.commands.failed(&e) => Next::End,
            Err(_) => Next::NewCapture,
        }
    }

    /// `CAPTURE`: one poll of the capture state, and what it asks for.
    fn poll(&mut self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) -> Next {
        let (state, offset) = match link.capture_state() {
            Ok(s) => s,
            Err(e) if self.polls.failed(&e) => return Next::End,
            Err(_) => return Next::Poll,
        };
        log::debug!("Capturestate {state}.");
        log::debug!("Trigger offset 0x{offset:06x}.");
        match state {
            protocol::CAPTURE_READY_9BIT => {
                if !self.told_9bit {
                    log::error!("Not yet supported.");
                    self.told_9bit = true;
                }
                return self.not_going_on("the device's 9 bit samples");
            }
            protocol::CAPTURE_EMPTY
            | protocol::CAPTURE_FILLING
            | protocol::CAPTURE_READY_8BIT
            | protocol::CAPTURE_READY_2250
            | protocol::CAPTURE_TIMEOUT => {}
            other => {
                log::debug!("Unknown capture state: {other}.");
                return self.not_going_on("unknown capture states");
            }
        }
        self.polls.0 = 0;
        self.told_9bit = false;
        match state {
            protocol::CAPTURE_EMPTY => {
                self.capture_empty_count += 1;
                if self.capture_empty_count >= protocol::MAX_CAPTURE_EMPTY {
                    self.capture_empty_count = 0;
                    // A failure waits for the next three empty states.
                    return match self.restart(link) {
                        Ok(()) => {
                            self.commands.0 = 0;
                            Next::Poll
                        }
                        Err(e) if self.commands.failed(&e) => Next::End,
                        Err(_) => Next::Poll,
                    };
                }
                Next::Poll
            }
            protocol::CAPTURE_READY_8BIT | protocol::CAPTURE_READY_2250 => {
                match link.request_channel_data() {
                    Ok(()) => {
                        self.commands.0 = 0;
                        self.fetch(link, offset, sink, stop)
                    }
                    Err(e) if self.commands.failed(&e) => Next::End,
                    Err(_) => Next::Poll,
                }
            }
            // No data yet, or "we'll try again next time".
            _ => Next::Poll,
        }
    }

    /// An answer the capture cannot go on from, which may yet change.
    fn not_going_on(&mut self, what: &str) -> Next {
        self.polls.0 += 1;
        if self.polls.0 >= MAX_FAILURES {
            log::error!("{MAX_FAILURES} polls in a row answered {what}, giving up.");
            return Next::End;
        }
        Next::Poll
    }

    fn send_chunk(&self, sink: &PacketSink, buf: &[u8]) -> bool {
        protocol::analog_packets(&self.settings, buf)
            .into_iter()
            .all(|p| sink.send(Packet::Analog(p)))
    }

    /// `FETCH_DATA`: the frame's transfers, as many as it takes.
    fn fetch(
        &mut self,
        link: &mut Link,
        offset: u32,
        sink: &PacketSink,
        stop: &AtomicBool,
    ) -> Next {
        if !sink.send(Packet::FrameBegin) {
            return Next::End;
        }
        let mut frame = Frame::new(self.settings.framesize, offset);
        let mut failures = Failures::default();
        while !frame.complete() {
            if stop.load(Ordering::SeqCst) {
                sink.send(Packet::FrameEnd);
                return Next::End;
            }
            let lost = match link.read_channel_data() {
                Ok(data) if data.is_empty() => {
                    log::debug!("receive_transfer(): status LIBUSB_SUCCESS received 0 bytes.");
                    failures.failed(&Error::Timeout)
                }
                Ok(data) => {
                    failures.0 = 0;
                    if !self.send_chunk(sink, frame.push(&data)) {
                        return Next::End;
                    }
                    false
                }
                Err(e) => {
                    log::debug!("receive_transfer(): status {}.", libusb_error_name(&e));
                    failures.failed(&e)
                }
            };
            if lost {
                log::error!("The frame is incomplete, its pre-trigger samples are dropped.");
                sink.send(Packet::FrameEnd);
                return Next::End;
            }
        }
        log::debug!(
            "End of frame, sending {} pre-trigger buffered samples.",
            frame.buffered().len() / 2
        );
        if !self.send_chunk(sink, frame.buffered()) || !sink.send(Packet::FrameEnd) {
            return Next::End;
        }
        self.num_frames += 1;
        if self.settings.limit_frames != 0 && self.num_frames >= self.settings.limit_frames {
            Next::End
        } else {
            Next::NewCapture
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::analog::Unit;

    fn device(i: usize) -> HantekDsoDevice {
        HantekDsoDevice::new(UsbDeviceInfo::default(), &protocol::PROFILES[i])
    }

    #[test]
    fn channels_groups_and_defaults_follow_the_c_driver() {
        let d = device(0);
        assert_eq!(d.info.model, "DSO-2090");
        let names: Vec<&str> = d.channels.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, CHANNEL_NAMES);
        assert_eq!(d.groups[1], ChannelGroup::new("CH2", vec![1]));
        assert_eq!(d.get(&Key::Timebase, None).unwrap(), Value::Rational(100, 1_000_000));
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(10_000));
        assert_eq!(d.get(&Key::Buffersize, None).unwrap(), Value::U64(10_240));
        assert_eq!(d.get(&Key::CaptureRatio, None).unwrap(), Value::U64(100));
        assert_eq!(d.get(&Key::TriggerLevel, None).unwrap(), Value::F64(0.5));
        assert_eq!(d.get(&Key::TriggerSlope, None).unwrap(), Value::Str("r".into()));
        assert!(d.get(&Key::TriggerSource, None).is_err());
        assert_eq!(d.get(&Key::Vdiv, Some("CH1")).unwrap(), Value::Rational(500, 1000));
        assert_eq!(d.get(&Key::Coupling, Some("CH2")).unwrap(), Value::Str("DC".into()));
        assert_eq!(d.get(&Key::Filter, Some("CH2")).unwrap(), Value::Bool(false));
        assert!(d.get(&Key::Vdiv, Some("CH3")).is_err());
        assert!(d.get(&Key::Samplerate, Some("CH1")).is_err());
    }

    /// api.c:430-435, 482-483: the C driver fails on both.
    #[test]
    fn the_division_counts_are_answered() {
        let d = device(0);
        assert_eq!(d.get(&Key::NumHdiv, None).unwrap(), Value::I32(10));
        assert_eq!(d.get(&Key::NumVdiv, None).unwrap(), Value::I32(8));
    }

    #[test]
    fn settings_take_only_the_drivers_values() {
        let mut d = device(0);
        d.set(&Key::Timebase, &Value::Rational(1, 1000), None).unwrap();
        assert_eq!(d.settings.timebase, 6);
        assert!(d.set(&Key::Timebase, &Value::Rational(3, 1_000_000), None).is_err());
        assert!(d.set(&Key::Buffersize, &Value::U64(1000), None).is_err());
        d.set(&Key::Buffersize, &Value::U64(32_768), None).unwrap();
        assert!(device(2).set(&Key::Buffersize, &Value::U64(32_768), None).is_err());
        assert!(d.set(&Key::Samplerate, &Value::U64(3_000_000), None).is_err());
        d.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap();
        assert!(d.set(&Key::TriggerSlope, &Value::Str("x".into()), None).is_err());
        d.set(&Key::TriggerSlope, &Value::Str("f".into()), None).unwrap();
        assert!(d.settings.slope_negative);
        assert!(d.set(&Key::TriggerSource, &Value::Str("CH3".into()), None).is_err());
        d.set(&Key::TriggerSource, &Value::Str("CH2".into()), None).unwrap();
        assert_eq!(d.settings.trigger_source, Some(1));
        assert!(d.set(&Key::TriggerLevel, &Value::F64(1.5), None).is_err());
        assert!(d.set(&Key::TriggerLevel, &Value::F64(-0.5), None).is_err());
        d.set(&Key::TriggerLevel, &Value::F64(0.4), None).unwrap();
        // As a float, as the C driver keeps it.
        assert_eq!(d.get(&Key::TriggerLevel, None).unwrap(), Value::F64(f64::from(0.4_f32)));
        assert!(d.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
        assert!(d.set(&Key::Vdiv, &Value::Rational(3, 1), Some("CH1")).is_err());
        d.set(&Key::Vdiv, &Value::Rational(2, 1), Some("CH2")).unwrap();
        assert_eq!(d.settings.voltage, [5, 7]);
        assert!(d.set(&Key::Coupling, &Value::Str("XX".into()), Some("CH1")).is_err());
        d.set(&Key::Coupling, &Value::Str("GND".into()), Some("CH1")).unwrap();
        assert_eq!(d.settings.coupling, [2, 1]);
        d.set(&Key::Filter, &Value::Bool(true), Some("CH1")).unwrap();
        assert_eq!(d.settings.filter, [true, false]);
        assert!(d.set(&Key::Filter, &Value::Bool(true), None).is_err());
    }

    /// The acquisition owns the device: what would have to reach it, or
    /// its thread, is refused and not kept as if it had been sent.
    #[test]
    fn a_set_during_a_capture_is_refused_and_not_stored() {
        let mut d = device(0);
        d.worker = Some(Worker {
            stop: Arc::new(AtomicBool::new(false)),
            handle: std::thread::spawn(|| {}),
        });
        let before = d.settings.clone();
        for (k, v) in [
            (Key::Samplerate, Value::U64(1_000_000)),
            (Key::TriggerLevel, Value::F64(0.25)),
            (Key::LimitFrames, Value::U64(3)),
        ] {
            assert!(matches!(d.set(&k, &v, None), Err(Error::Busy)), "{k:?}");
        }
        assert_eq!(d.settings, before);
    }

    #[test]
    fn devices_are_found_again_by_their_ports() {
        let usb = UsbDeviceInfo { bus: 1, address: 5, ports: vec![1, 2], ..Default::default() };
        let moved = UsbDeviceInfo { address: 9, ..usb.clone() };
        assert!(same_device(&moved, &usb, false));
        let elsewhere = UsbDeviceInfo { ports: vec![1, 3], ..usb.clone() };
        assert!(!same_device(&elsewhere, &usb, true));
        let no_ports = UsbDeviceInfo { ports: Vec::new(), ..moved };
        assert!(!same_device(&no_ports, &usb, false));
        assert!(same_device(&no_ports, &usb, true));
        assert_eq!(port_path(&usb), "1-1.2");
    }

    const BEGIN: &str = "ctl_out 40 b3 0000 0000 0f03030368acfe000100 -> ok\n";

    /// A poll of the state's answer: the state and a zero offset.
    fn poll(state: u8) -> String {
        format!("{BEGIN}bulk_out 02 0600 -> 2\nbulk_in 86 512 -> {state:02x}000000\n")
    }

    /// An acquisition on a replayed link: the packets after the header.
    fn run(settings: Settings, steps: &str) -> Vec<Packet> {
        let text = format!("# oxrok usb fixture 1\ndevice vid=04b5 pid=2090\n{steps}");
        let r = crate::usb::record::Replay::from_text(&text).unwrap();
        let mut link = Link::new(Box::new(r.clone()), 0x2090, 4, [[[0; 2]; 9]; 2]);
        let (tx, rx) = std::sync::mpsc::sync_channel(1024);
        Acquisition::new(settings).run(&mut link, &PacketSink::new(tx, 0), &AtomicBool::new(false));
        r.finish().unwrap();
        rx.try_iter().map(|(_, p)| p).skip(1).collect()
    }

    /// A frame of two samples, two transfers of one sample would be: its
    /// request and data for `frame_settings()`.
    fn small() -> Settings {
        Settings { limit_frames: 1, framesize: 2, ..Settings::default() }
    }

    fn analog(p: &Packet) -> &crate::core::AnalogData {
        let Packet::Analog(a) = p else { panic!("{p:?}") };
        a
    }

    #[test]
    fn a_frame_sends_each_channel_and_ends_at_its_limit() {
        let steps = format!(
            "{}{BEGIN}bulk_out 02 0500 -> 2\nbulk_in 86 4 -> 00ff00ff\n",
            poll(protocol::CAPTURE_READY_8BIT)
        );
        let p = run(small(), &steps);
        assert_eq!(p.len(), 4);
        assert_eq!(p[0], Packet::FrameBegin);
        assert_eq!(analog(&p[1]).channels, [0]);
        assert_eq!(analog(&p[1]).unit, Unit::Volt);
        assert_eq!(analog(&p[1]).samples.len(), 2);
        assert_eq!(analog(&p[2]).channels, [1]);
        assert_eq!(p[3], Packet::FrameEnd);
    }

    /// api.c:719-721 leaves the transfer it got nothing from behind, and
    /// the frame unfinished for ever; the data is still in the device.
    #[test]
    fn an_empty_or_timed_out_transfer_is_read_again() {
        let steps = format!(
            "{}{BEGIN}bulk_out 02 0500 -> 2\nbulk_in 86 4 -> -\nbulk_in 86 4 -> err timeout\n\
             bulk_in 86 4 -> 00ff\nbulk_in 86 4 -> 00ff\n",
            poll(protocol::CAPTURE_READY_8BIT)
        );
        let p = run(small(), &steps);
        // Two transfers of a sample each, two channels each.
        assert_eq!(p.len(), 6);
        assert_eq!(p[5], Packet::FrameEnd);
    }

    #[test]
    fn a_disconnect_ends_a_frame_at_once() {
        let steps = format!(
            "{}{BEGIN}bulk_out 02 0500 -> 2\nbulk_in 86 4 -> err disconnected\n",
            poll(protocol::CAPTURE_READY_8BIT)
        );
        assert_eq!(run(small(), &steps), [Packet::FrameBegin, Packet::FrameEnd]);
    }

    #[test]
    fn a_frame_that_never_comes_is_given_up() {
        let mut steps =
            format!("{}{BEGIN}bulk_out 02 0500 -> 2\n", poll(protocol::CAPTURE_READY_8BIT));
        for _ in 0..MAX_FAILURES {
            steps.push_str("bulk_in 86 4 -> err timeout\n");
        }
        assert_eq!(run(small(), &steps), [Packet::FrameBegin, Packet::FrameEnd]);
    }

    #[test]
    fn failed_polls_are_bounded_and_a_good_one_resets_the_count() {
        let bad = format!("{BEGIN}bulk_out 02 0600 -> 2\nbulk_in 86 512 -> err timeout\n");
        let mut steps = bad.repeat(MAX_FAILURES - 1);
        steps.push_str(&poll(protocol::CAPTURE_FILLING));
        steps.push_str(&bad.repeat(MAX_FAILURES));
        assert_eq!(run(small(), &steps), []);
    }

    /// protocol.c:832, 847-854 reads what the answer did not fill.
    #[test]
    fn a_short_state_answer_is_a_failed_poll() {
        let bad = format!("{BEGIN}bulk_out 02 0600 -> 2\nbulk_in 86 512 -> 02\n");
        assert_eq!(run(small(), &bad.repeat(MAX_FAILURES)), []);
    }

    /// api.c:898-901: "Not yet supported." at every poll, for ever.
    #[test]
    fn nine_bit_frames_end_the_capture_after_a_while() {
        let steps = poll(protocol::CAPTURE_READY_9BIT).repeat(MAX_FAILURES);
        assert_eq!(run(small(), &steps), []);
    }

    #[test]
    fn unknown_states_end_the_capture_after_a_while() {
        let steps = poll(255).repeat(MAX_FAILURES);
        assert_eq!(run(small(), &steps), []);
    }

    /// Three empty states restart with a force when no source is set; the
    /// count goes on across the restart.
    #[test]
    fn empty_states_restart_the_capture() {
        let restart = format!(
            "{BEGIN}bulk_out 02 0300 -> 2\n{BEGIN}bulk_out 02 0400 -> 2\n\
             {BEGIN}bulk_out 02 0200 -> 2\n"
        );
        let empty = poll(protocol::CAPTURE_EMPTY);
        let steps = format!(
            "{e}{e}{e}{restart}{r}{BEGIN}bulk_out 02 0500 -> 2\nbulk_in 86 4 -> 00ff00ff\n",
            e = empty,
            r = poll(protocol::CAPTURE_READY_8BIT)
        );
        assert_eq!(run(small(), &steps).len(), 4);
    }

    /// With a source the restart does not force it.
    #[test]
    fn a_trigger_source_is_waited_for_without_a_force() {
        let restart = format!("{BEGIN}bulk_out 02 0300 -> 2\n{BEGIN}bulk_out 02 0400 -> 2\n");
        let empty = poll(protocol::CAPTURE_EMPTY);
        let steps = format!(
            "{e}{e}{e}{restart}{r}{BEGIN}bulk_out 02 0500 -> 2\nbulk_in 86 4 -> 00ff00ff\n",
            e = empty,
            r = poll(protocol::CAPTURE_READY_8BIT)
        );
        let s = Settings { trigger_source: Some(2), ..small() };
        assert_eq!(run(s, &steps).len(), 4);
    }

    #[test]
    fn a_failed_restart_is_bounded() {
        let empties = poll(protocol::CAPTURE_EMPTY).repeat(3);
        let failed = format!("{BEGIN}bulk_out 02 0300 -> err device \"endpoint stalled\"\n");
        let steps = format!("{empties}{failed}").repeat(MAX_FAILURES);
        assert_eq!(run(small(), &steps), []);
    }

    /// Two frames: the second capture is started after the first one's
    /// end, retried from its start, and forced.
    #[test]
    fn the_next_frame_starts_a_capture() {
        let data = format!(
            "{}{BEGIN}bulk_out 02 0500 -> 2\nbulk_in 86 4 -> 00ff00ff\n",
            poll(protocol::CAPTURE_READY_8BIT)
        );
        let steps = format!(
            "{data}{BEGIN}bulk_out 02 0300 -> 2\n\
             {BEGIN}bulk_out 02 0400 -> err device \"endpoint stalled\"\n\
             {BEGIN}bulk_out 02 0300 -> 2\n{BEGIN}bulk_out 02 0400 -> 2\n\
             {BEGIN}bulk_out 02 0200 -> 2\n{data}"
        );
        let s = Settings { limit_frames: 2, ..small() };
        assert_eq!(run(s, &steps).len(), 8);
    }

    #[test]
    fn the_stop_flag_ends_before_the_next_poll() {
        let text = "# oxrok usb fixture 1\ndevice vid=04b5 pid=2090\n";
        let r = crate::usb::record::Replay::from_text(text).unwrap();
        let mut link = Link::new(Box::new(r.clone()), 0x2090, 4, [[[0; 2]; 9]; 2]);
        let (tx, rx) = std::sync::mpsc::sync_channel(4);
        Acquisition::new(small()).run(&mut link, &PacketSink::new(tx, 0), &AtomicBool::new(true));
        r.finish().unwrap();
        assert_eq!(rx.try_iter().count(), 1);
    }
}
