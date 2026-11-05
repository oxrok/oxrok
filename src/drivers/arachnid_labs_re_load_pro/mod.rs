// Counterpart in sigrok:
//   libsigrok/src/hardware/arachnid-labs-re-load-pro/api.c,
//   libsigrok/src/hardware/arachnid-labs-re-load-pro/protocol.c,
//   Copyright (C) 2015-2016 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Frank Stettner, Gerhard Sittig,
//   Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Arachnid Labs Re:load Pro, an electronic load, on a USB serial port.
//!
//! `conn=` names the port, `serialcomm=` overrides 115200/8n1. A scan stops
//! monitoring, waits 50 ms and asks the version; firmware below 1.10, as a
//! float, is no device. A capture asks for a line every 200 ms and a sample
//! is a `read` line; `monitor 0` goes at the close, and at every close.
//!
//! Where this parts from the C:
//!
//! - An `err` line refuses a request, where C's check looks at the wrong bytes
//!   (`protocol.c:59`), and takes the error's text for the value.
//! - A value with no answer fails `--get`, where C reads 0, or dies on `read`.
//! - A setting with no answer fails, outside a capture, where C takes it for a
//!   success. The firmware answers each of them every time (`comms.c`), so
//!   `enabled=off` with no answer may have left the current flowing. In a capture
//!   a setting only writes, as in C, for the capture has the port.
//! - A line that comes to a request and is not its answer is passed over, as
//!   the device speaks by itself. What has arrived ahead of a request or a capture
//!   is read first, where C takes it for the answer, and only `overtemp` and
//!   `undervolt` are kept of it: as active flags, and in a capture as META too.
//! - A line in two pieces more than 250 ms apart is one line, in a capture and
//!   within a request's wait. A line that nothing ended by the end of that wait
//!   is no answer, where C takes `set 15` of `set 1500` for the value.
//! - A short `read` line is passed over, where C dies.
//! - A time limit counts when nothing comes, and so does a port gone, where C
//!   looks at neither (`protocol.c:425`), and a capture that only a sample
//!   limit can end gives up after [`NO_READING_BOUND`] with no reading.
//! - A NaN is refused, where C sends `set -2147483648`.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::serial::{Params, Port, serial_timeout};
use protocol::{Answer, Awaited, Line};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::ElectronicLoad];
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];
static GROUP_OPTIONS: [OptionDesc; 12] = [
    OptionDesc::new(Key::Enabled, SET_ONLY),
    OptionDesc::new(Key::Regulation, GET_LIST),
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::OverVoltageProtectionEnabled, Access::GET),
    OptionDesc::new(Key::OverCurrentProtectionEnabled, Access::GET),
    OptionDesc::new(Key::OverTemperatureProtection, Access::GET),
    OptionDesc::new(Key::OverTemperatureProtectionActive, Access::GET),
    OptionDesc::new(Key::UnderVoltageCondition, Access::GET),
    OptionDesc::new(Key::UnderVoltageConditionActive, Access::GET),
    OptionDesc::new(Key::UnderVoltageConditionThreshold, Access::GET_SET_LIST),
];
const SET_ONLY: Access = Access { get: false, set: true, list: false };
const GET_LIST: Access = Access { get: true, set: false, list: true };
/// The one channel group, holding both channels.
const GROUP: &str = "1";
/// `api.c`'s version check, as a float. It refuses no firmware that gets this
/// far: `version` came with v1.6, and 1.6 to 1.9 compare above this.
const MIN_VERSION: f64 = 1.10;
/// The wait after `monitor 0`: the firmware's queue of commands holds one.
const AFTER_MONITOR_STOP: Duration = Duration::from_millis(50);
/// The most `set` takes, in amperes.
const CURRENT_MAX: f32 = 6.0;
/// The most `uvlo` takes, in volts.
const THRESHOLD_MAX: f32 = 60.0;

/// How long a capture that only a sample limit can end goes on with no reading.
/// C has no bound. A live device sends a line 5 times a second.
pub const NO_READING_BOUND: Duration = Duration::from_secs(5);
/// How long a capture waits for a byte with the link locked.
const READ_SLICE: Duration = Duration::from_millis(20);
/// The pause with the link free after a read that gave nothing.
const EMPTY_READ_PAUSE: Duration = Duration::from_millis(2);
/// How long a `get` in a capture waits for its line, as in C.
const CAPTURE_REPLY_TIMEOUT: Duration = Duration::from_secs(5);
/// How long the End waits for room after a stop.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

const CHANNEL_V: usize = 0;
const CHANNEL_I: usize = 1;

/// The `arachnid-labs-re-load-pro` driver.
pub struct ReloadProDriver;

impl Driver for ReloadProDriver {
    fn id(&self) -> &'static str {
        "arachnid-labs-re-load-pro"
    }

    fn long_name(&self) -> &'static str {
        "Arachnid Labs Re:load Pro"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = protocol::SERIALCOMM.to_owned();
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => {
                    value
                        .as_str()
                        .ok_or_else(|| Error::arg("serialcomm"))?
                        .clone_into(&mut serialcomm);
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        // Every failure of the port here is no device, as `scan` returns NULL
        // for each.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (mut port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        let version = match probe(port.as_mut(), params) {
            Ok(version) => version,
            Err(e) => {
                log::debug!("Scan failed: {e}.");
                None
            }
        };
        // The port goes as the scan's `serial_close`; `dev_open` opens it again.
        drop(port);
        Ok(version
            .map(|v| Box::new(ReloadProDevice::new(v, conn, params)) as Box<dyn Device>)
            .into_iter()
            .collect())
    }
}

/// `serial_write_blocking` with `serial_timeout`, all or an error.
fn write_all(port: &mut dyn Port, params: Params, data: &[u8]) -> Result<()> {
    let timeout = serial_timeout(&params, u32::try_from(data.len()).unwrap_or(u32::MAX));
    let sent = port.write_timeout(data, timeout)?;
    if sent < data.len() {
        return Err(Error::device("the device did not take the whole command"));
    }
    Ok(())
}

/// Drops the input ahead of the version. C has no such step, so a port that fails
/// here has the write that follows to say so, with C's words.
fn drop_input(port: &mut dyn Port) {
    if let Err(e) = port.purge() {
        log::debug!("Could not drop the input: {e}.");
    }
}

/// `scan` after the open: the version, if this is a device to have. What may
/// still be on its way from monitoring left on goes first, or the version's
/// read meets it.
fn probe(port: &mut dyn Port, params: Params) -> Result<Option<String>> {
    if let Err(e) = write_all(port, params, protocol::MONITOR_STOP) {
        log::debug!("Unable to write while probing for hardware.");
        return Err(e);
    }
    std::thread::sleep(AFTER_MONITOR_STOP);
    drop_input(port);
    if let Err(e) = write_all(port, params, protocol::VERSION) {
        log::debug!("Unable to write while probing for hardware.");
        return Err(e);
    }
    let line = protocol::read_line(port, protocol::VERSION_TIMEOUT)?;
    let Some(version) = protocol::version_of(&line) else {
        log::debug!("Unable to probe version number.");
        return Ok(None);
    };
    let number = protocol::number(version);
    if number < MIN_VERSION {
        log::info!("Firmware >= 1.10 required (got {number:.2}).");
        return Ok(None);
    }
    Ok(Some(String::from_utf8_lossy(version).into_owned()))
}

/// The port and what the device said last. Taken by a request, or by a capture
/// for a slice of a read.
struct Link {
    port: Option<Box<dyn Port>>,
    params: Params,
    /// A capture reads the port, so a request only writes and waits.
    running: bool,
    voltage: f32,
    current: f32,
    current_limit: f32,
    threshold: f32,
    otp_active: bool,
    uvc_active: bool,
    /// What [`protocol::absorb`] has of a line still coming.
    started: Vec<u8>,
    /// Lines of each kind so far, which a `get` in a capture waits on.
    readings: u64,
    limits: u64,
    thresholds: u64,
}

impl Link {
    fn new(params: Params) -> Link {
        Link {
            port: None,
            params,
            running: false,
            voltage: 0.0,
            current: 0.0,
            current_limit: 0.0,
            threshold: 0.0,
            otp_active: false,
            uvc_active: false,
            started: Vec::new(),
            readings: 0,
            limits: 0,
            thresholds: 0,
        }
    }

    /// What a line the device sent by itself says. C looks only in a capture,
    /// and reads such a line as its answer.
    fn note(&mut self, line: &Line) {
        match line {
            Line::OverTemperature => {
                log::warn!("Overtemperature condition!");
                self.otp_active = true;
            }
            Line::UnderVoltage => {
                log::warn!("Undervoltage condition!");
                self.uvc_active = true;
            }
            _ => {}
        }
    }

    /// Reads what has arrived, for the flags it may hold, unless a capture has the
    /// port.
    fn absorb(&mut self) {
        if self.running {
            return;
        }
        let Some(port) = self.port.as_deref_mut() else { return };
        let mut noted = Vec::new();
        protocol::absorb(port, &mut self.started, &mut |line| noted.push(*line));
        for line in &noted {
            self.note(line);
        }
    }

    /// A request's write, and its answer unless a capture has the port.
    fn request(&mut self, command: &[u8], awaited: Awaited) -> Result<Answer> {
        self.absorb();
        let params = self.params;
        let running = self.running;
        let started = std::mem::take(&mut self.started);
        let port = self.port.as_deref_mut().ok_or_else(|| Error::device("device is not open"))?;
        if let Err(e) = write_all(port, params, command) {
            log::error!("Unable to send command.");
            return Err(e);
        }
        if running {
            return Ok(Answer::None);
        }
        let mut noted = Vec::new();
        let answer = protocol::await_answer(port, awaited, started, &mut |line| noted.push(*line));
        for line in &noted {
            self.note(line);
        }
        Ok(answer)
    }

    fn lines_seen(&self, awaited: Awaited) -> u64 {
        match awaited {
            Awaited::Reading => self.readings,
            Awaited::CurrentLimit => self.limits,
            Awaited::Threshold => self.thresholds,
            Awaited::Done => 0,
        }
    }
}

struct Shared {
    link: Mutex<Link>,
    /// A line came in a capture, or the capture ended.
    line: Condvar,
}

/// One load.
pub struct ReloadProDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    conn: String,
    params: Params,
    limit_samples: u64,
    limit_msec: u64,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    shared: Arc<Shared>,
    worker: Option<Worker>,
    /// [`CAPTURE_REPLY_TIMEOUT`], for tests to shorten.
    capture_reply: Duration,
    /// [`NO_READING_BOUND`], for tests to shorten.
    no_reading: Duration,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl ReloadProDevice {
    fn new(version: String, conn: String, params: Params) -> Self {
        ReloadProDevice {
            info: DeviceInfo {
                driver: "arachnid-labs-re-load-pro",
                vendor: "Arachnid Labs".to_owned(),
                model: "Re:load Pro".to_owned(),
                version,
                serial: String::new(),
                conn: None,
            },
            channels: vec![
                Channel::new(CHANNEL_V, ChannelKind::Analog, "V".to_owned()),
                Channel::new(CHANNEL_I, ChannelKind::Analog, "I".to_owned()),
            ],
            groups: vec![ChannelGroup::new(GROUP, vec![CHANNEL_V, CHANNEL_I])],
            conn,
            params,
            limit_samples: 0,
            limit_msec: 0,
            pending: None,
            shared: Arc::new(Shared { link: Mutex::new(Link::new(params)), line: Condvar::new() }),
            worker: None,
            capture_reply: CAPTURE_REPLY_TIMEOUT,
            no_reading: NO_READING_BOUND,
        }
    }

    /// A device on a port that stands in for the hardware, without a scan.
    /// [`Device::open`] takes it.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let params = Params::parse(protocol::SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device = Self::new("1.11".to_owned(), conn, params);
        device.pending = Some(port);
        device
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.shared.link.lock().expect("link lock")
    }

    /// A setting's request. No answer outside a capture is a failure, where C
    /// takes it for a success, and an `err` line a refusal, where C's check reads
    /// the wrong bytes (`protocol.c:59`).
    fn apply(&self, command: &[u8], awaited: Awaited, complaint: &str) -> Result<()> {
        let (answer, running) = {
            let mut link = self.link();
            (link.request(command, awaited), link.running)
        };
        match answer {
            Ok(Answer::Line(_)) => Ok(()),
            Ok(Answer::None) if running => Ok(()),
            Ok(Answer::None) => Err(no_answer(command)),
            Ok(Answer::Refused(text)) => {
                log::error!("Device replied with an error: '{text}'.");
                Err(Error::device(text))
            }
            Err(e) => {
                log::error!("{complaint}: {e}.");
                Err(e)
            }
        }
    }

    /// A question: its answer in the link, by a request or, in a capture, by
    /// the line the capture sees within [`CAPTURE_REPLY_TIMEOUT`].
    fn ask(&self, command: &[u8], awaited: Awaited, complaint: &str) -> Result<()> {
        let mut link = self.link();
        let running = link.running;
        let seen = link.lines_seen(awaited);
        let answer = link.request(command, awaited).inspect_err(|e| {
            log::error!("{complaint}: {e}.");
        })?;
        if running {
            let (link, _) = self
                .shared
                .line
                .wait_timeout_while(link, self.capture_reply, |l| {
                    l.running && l.lines_seen(awaited) == seen
                })
                .expect("link lock");
            return if link.lines_seen(awaited) == seen { Err(Error::Timeout) } else { Ok(()) };
        }
        match answer {
            Answer::Line(Line::Reading { voltage, current }) => {
                link.voltage = voltage;
                link.current = current;
                Ok(())
            }
            Answer::Line(Line::CurrentLimit(limit)) => {
                link.current_limit = limit;
                Ok(())
            }
            Answer::Line(Line::Threshold(threshold)) => {
                link.threshold = threshold;
                Ok(())
            }
            Answer::Refused(text) => {
                log::error!("Device replied with an error: '{text}'.");
                Err(Error::device(text))
            }
            Answer::Line(_) => Err(Error::device("a reading with too few numbers")),
            Answer::None => Err(no_answer(command)),
        }
    }

    fn voltage_current(&self) -> Result<(f32, f32)> {
        self.ask(b"read\n", Awaited::Reading, "Error sending voltage/current query")?;
        let link = self.link();
        Ok((link.voltage, link.current))
    }

    fn threshold(&self) -> Result<f32> {
        self.ask(b"uvlo\n", Awaited::Threshold, "Error sending under voltage threshold query")?;
        Ok(self.link().threshold)
    }

    fn group_get(&self, key: &Key) -> Result<Value> {
        Ok(match key {
            Key::Regulation => Value::from("CC"),
            Key::Voltage => Value::F64(f64::from(self.voltage_current()?.0)),
            Key::Current => Value::F64(f64::from(self.voltage_current()?.1)),
            Key::CurrentLimit => {
                self.ask(b"set\n", Awaited::CurrentLimit, "Error sending current limit query")?;
                Value::F64(f64::from(self.link().current_limit))
            }
            Key::OverVoltageProtectionEnabled
            | Key::OverCurrentProtectionEnabled
            | Key::OverTemperatureProtection => Value::Bool(true),
            Key::OverTemperatureProtectionActive => {
                let mut link = self.link();
                link.absorb();
                Value::Bool(link.otp_active)
            }
            Key::UnderVoltageCondition => Value::Bool(self.threshold()? != 0.0),
            Key::UnderVoltageConditionActive => {
                let mut link = self.link();
                link.absorb();
                Value::Bool(link.uvc_active)
            }
            Key::UnderVoltageConditionThreshold => Value::F64(f64::from(self.threshold()?)),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn group_set(&self, key: &Key, value: &Value) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let float = || value.as_f64().map(|v| v as f32).ok_or_else(bad);
        match key {
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(bad)?;
                let command: &[u8] = if on { b"on\n" } else { b"off\n" };
                self.apply(command, Awaited::Done, "Error sending on/off command")
            }
            Key::CurrentLimit => {
                let amps = float()?;
                if !(0.0..=CURRENT_MAX).contains(&amps) {
                    log::error!("The current limit must be 0-6 A (was {amps:.6} A).");
                    return Err(Error::Arg("the current limit must be 0-6 A".to_owned()));
                }
                let command = format!("set {}\n", protocol::to_milli(amps));
                self.apply(
                    command.as_bytes(),
                    Awaited::CurrentLimit,
                    "Error sending current limit command",
                )
            }
            Key::UnderVoltageConditionThreshold => {
                let volts = float()?;
                if !(0.0..=THRESHOLD_MAX).contains(&volts) {
                    log::error!("The under voltage threshold must be 0-60 V (was {volts:.6} V).");
                    return Err(Error::Arg(
                        "the under voltage threshold must be 0-60 V".to_owned(),
                    ));
                }
                let millivolts = protocol::to_milli(volts);
                log::trace!("Setting under voltage threshold to {volts:.6} V ({millivolts} mV).");
                self.apply(
                    format!("uvlo {millivolts}\n").as_bytes(),
                    Awaited::Threshold,
                    "Error sending under voltage threshold command",
                )
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `dev_acquisition_start`: `monitor 200`, then the thread, whose Header
    /// goes first.
    fn begin(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(worker) = &self.worker {
            if self.link().running {
                return Err(Error::Busy);
            }
            worker.stop.store(true, Ordering::SeqCst);
            let worker = self.worker.take().expect("just seen");
            let _ = worker.handle.join();
        }
        // What has arrived ahead of the capture: a line of the last capture's or a
        // request's that came late goes, but C's capture would report these.
        let mut opening = Vec::new();
        let started = {
            let mut link = self.link();
            let link = &mut *link;
            let params = link.params;
            let port =
                link.port.as_deref_mut().ok_or_else(|| Error::device("device is not open"))?;
            protocol::absorb(port, &mut link.started, &mut |line| {
                if matches!(line, Line::OverTemperature | Line::UnderVoltage) {
                    opening.push(*line);
                }
            });
            if let Err(e) = write_all(port, params, protocol::MONITOR_START) {
                log::error!("Unable to send 'monitor' command: {e}.");
                return Err(e);
            }
            link.running = true;
            std::mem::take(&mut link.started)
        };
        let capture = Capture {
            opening,
            started,
            shared: Arc::clone(&self.shared),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            no_reading: self.no_reading,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let spawned = std::thread::Builder::new()
            .name("arachnid-labs-re-load-pro acquisition".to_owned())
            .spawn(move || {
                capture.run(&sink, &stop2);
                capture.shared.link.lock().expect("link lock").running = false;
                capture.shared.line.notify_all();
                send_end(&sink, &stop2);
            });
        match spawned {
            Ok(handle) => {
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                self.link().running = false;
                Err(Error::Io(e))
            }
        }
    }
}

impl Device for ReloadProDevice {
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
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        ch.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        name.clone_into(&mut ch.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(GROUP) => Ok(GROUP_OPTIONS.to_vec()),
            Some(other) => Err(Error::ChannelGroup(other.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        match group {
            None => match key {
                Key::LimitSamples => Ok(Value::U64(self.limit_samples)),
                Key::LimitMsec => Ok(Value::U64(self.limit_msec)),
                other => Err(Error::UnknownKey(other.id().to_owned())),
            },
            Some(GROUP) => self.group_get(key),
            Some(other) => Err(Error::ChannelGroup(other.to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        match group {
            None => {
                let bad = || Error::Arg(format!("invalid value for {}", key.id()));
                match key {
                    // `core::config_set` refuses a 0 and says so; this is for
                    // a library caller that comes straight here.
                    Key::LimitSamples => {
                        self.limit_samples = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                    }
                    Key::LimitMsec => {
                        self.limit_msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                    }
                    other => return Err(Error::UnknownKey(other.id().to_owned())),
                }
                Ok(())
            }
            Some(GROUP) => self.group_set(key, value),
            Some(other) => Err(Error::ChannelGroup(other.to_owned())),
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        match group {
            None => Err(Error::UnknownKey(key.id().to_owned())),
            Some(GROUP) => match key {
                Key::Regulation => Ok(ValueList::Str(vec!["CC".to_owned()])),
                Key::CurrentLimit => {
                    Ok(ValueList::RangeStepF64 { min: 0.0, max: 6.0, step: 0.001 })
                }
                Key::UnderVoltageConditionThreshold => {
                    Ok(ValueList::RangeStepF64 { min: 0.0, max: 60.0, step: 0.001 })
                }
                other => Err(Error::UnknownKey(other.id().to_owned())),
            },
            Some(other) => Err(Error::ChannelGroup(other.to_owned())),
        }
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut link = self.shared.link.lock().expect("link lock");
        if link.port.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&self.conn, self.params)?,
        };
        link.port = Some(port);
        Ok(())
    }

    /// `dev_close`: `monitor 0`, however it goes, and the port.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let mut link = self.link();
        let params = link.params;
        if let Some(port) = link.port.as_deref_mut()
            && let Err(e) = write_all(port, params, protocol::MONITOR_STOP)
        {
            log::debug!("Unable to stop monitoring: {e}.");
        }
        link.port = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        self.begin(sink)
    }

    /// Asks the thread to end, and does not wait: it may be sending. Nothing
    /// goes to the device for this: monitoring goes on until the close.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// A device dropped with a capture on must not leave its thread reading.
impl Drop for ReloadProDevice {
    fn drop(&mut self) {
        if let Err(e) = self.stop() {
            log::debug!("Could not stop the device: {e}.");
        }
    }
}

/// One capture, on its own thread: `reloadpro_receive_data`.
struct Capture {
    /// `overtemp` and `undervolt` lines that came ahead of it.
    opening: Vec<Line>,
    /// What had come of a line ahead of it.
    started: Vec<u8>,
    shared: Arc<Shared>,
    limit_samples: u64,
    limit_msec: u64,
    no_reading: Duration,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return;
        }
        for line in &self.opening {
            if self.handle_line(*line, "", sink, stop).is_none() {
                return;
            }
        }
        let started = Instant::now();
        let limit = Duration::from_millis(self.limit_msec);
        let only_samples_can_end = self.limit_samples > 0 && self.limit_msec == 0;
        let mut samples = 0u64;
        let mut last_reading = started;
        let mut pending = self.started.clone();
        let mut chunk = [0u8; 64];
        loop {
            if !go_on() {
                return;
            }
            // C looks at a time limit only when a line has come
            // (`protocol.c:425`).
            if self.limit_msec > 0 && started.elapsed() >= limit {
                return;
            }
            let read = {
                let mut link = self.shared.link.lock().expect("link lock");
                match link.port.as_deref_mut() {
                    Some(port) => port.read(&mut chunk, READ_SLICE),
                    None => Err(Error::device("device is not open")),
                }
            };
            let n = match read {
                Ok(n) => n,
                Err(e) => {
                    log::error!("Serial port read error: {e}.");
                    return;
                }
            };
            for &byte in &chunk[..n] {
                let ended = matches!(byte, b'\r' | b'\n');
                if !ended {
                    pending.push(byte);
                }
                if (!ended && pending.len() < protocol::LINE_MAX) || pending.is_empty() {
                    continue;
                }
                let Some(sample) = self.handle(&pending, sink, stop) else { return };
                pending.clear();
                if sample {
                    samples += 1;
                    last_reading = Instant::now();
                    if self.limit_samples > 0 && samples >= self.limit_samples {
                        return;
                    }
                }
            }
            // After the read: a machine that stalled has lines to look at first.
            if only_samples_can_end && last_reading.elapsed() >= self.no_reading {
                log::error!("Acquisition failed: no reading for {:?}.", self.no_reading);
                return;
            }
            if n == 0 {
                std::thread::sleep(EMPTY_READ_PAUSE);
            }
        }
    }

    /// `handle_packet`: whether the line was a sample, or `None` when the
    /// packets have nowhere to go.
    fn handle(&self, raw: &[u8], sink: &PacketSink, stop: &AtomicBool) -> Option<bool> {
        let text = String::from_utf8_lossy(protocol::c_string(raw));
        self.handle_line(protocol::parse(raw), &text, sink, stop)
    }

    /// [`Capture::handle`], for a line already parsed.
    fn handle_line(
        &self,
        parsed: Line,
        text: &str,
        sink: &PacketSink,
        stop: &AtomicBool,
    ) -> Option<bool> {
        let go_on = || !stop.load(Ordering::SeqCst);
        let mut link = self.shared.link.lock().expect("link lock");
        let mut sample = false;
        let packets = match parsed {
            Line::OverTemperature => {
                link.note(&parsed);
                vec![Packet::meta(Key::OverTemperatureProtectionActive, true)]
            }
            Line::UnderVoltage => {
                link.note(&parsed);
                vec![Packet::meta(Key::UnderVoltageConditionActive, true)]
            }
            Line::Refused => {
                log::error!("Device replied with an error: '{text}'.");
                Vec::new()
            }
            Line::CurrentLimit(limit) => {
                link.current_limit = limit;
                link.limits += 1;
                vec![Packet::meta(Key::CurrentLimit, f64::from(limit))]
            }
            Line::Threshold(threshold) => {
                link.threshold = threshold;
                link.thresholds += 1;
                if threshold == 0.0 {
                    vec![Packet::meta(Key::UnderVoltageCondition, false)]
                } else {
                    vec![
                        Packet::meta(Key::UnderVoltageCondition, true),
                        Packet::meta(Key::UnderVoltageConditionThreshold, f64::from(threshold)),
                    ]
                }
            }
            Line::Reading { voltage, current } => {
                link.voltage = voltage;
                link.current = current;
                link.readings += 1;
                sample = true;
                vec![
                    Packet::FrameBegin,
                    analog(CHANNEL_V, voltage, Mq::Voltage, Unit::Volt),
                    analog(CHANNEL_I, current, Mq::Current, Unit::Ampere),
                    Packet::FrameEnd,
                ]
            }
            Line::ShortReading => {
                log::warn!("A reading with too few numbers: '{text}'.");
                Vec::new()
            }
            Line::Done | Line::Unknown => {
                log::debug!("Unknown packet: '{text}'.");
                Vec::new()
            }
        };
        drop(link);
        self.shared.line.notify_all();
        packets.into_iter().all(|p| sink.send_while(p, go_on)).then_some(sample)
    }
}

/// The error of a request that nothing answered.
fn no_answer(command: &[u8]) -> Error {
    Error::device(format!("no answer to '{}'", command.trim_ascii().escape_ascii()))
}

fn analog(channel: usize, value: f32, mq: Mq, unit: Unit) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![channel],
        samples: vec![value],
        mq,
        flags: MqFlags::DC,
        unit,
        digits: 3,
    })
}

/// The End, for which a client that goes on reading is waited for without a
/// bound, and after a stop only for [`END_AFTER_STOP`].
fn send_end(sink: &PacketSink, stop: &AtomicBool) {
    let stopped_at = std::cell::Cell::new(None);
    sink.send_while(Packet::End, || {
        if !stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = stopped_at.get().unwrap_or_else(Instant::now);
        stopped_at.set(Some(at));
        at.elapsed() < END_AFTER_STOP
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, RecvTimeoutError, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit
    /// that does not bite must fail, not hang the run.
    const CAPTURE_BOUND: Duration = Duration::from_secs(10);

    type Received = Receiver<(usize, Packet)>;

    /// What the fake saw and holds, shared with the test.
    #[derive(Default)]
    struct State {
        writes: Vec<(Instant, Vec<u8>)>,
        purges: usize,
        input: VecDeque<(Instant, u8)>,
        /// Sent at `monitor 200`: each with the pause before it.
        stream: Vec<(Duration, Vec<u8>)>,
        /// The answers to requests, in turn, instead of the usual ones.
        answers: VecDeque<Vec<u8>>,
        silent: bool,
        fail_writes_after: Option<usize>,
        short_writes_after: Option<usize>,
        fail_reads: bool,
    }

    /// A load that answers as the firmware does, and notes each write.
    struct Fake(Arc<Mutex<State>>);

    fn fake() -> (Fake, Arc<Mutex<State>>) {
        let state = Arc::new(Mutex::new(State::default()));
        (Fake(Arc::clone(&state)), state)
    }

    impl State {
        fn usual(data: &[u8]) -> Vec<u8> {
            match data {
                b"version\r\n" => b"version 1.11\r\n".to_vec(),
                b"read\n" => b"read 1499 12345 7 8\r\n".to_vec(),
                b"set\n" => b"set 1500\r\n".to_vec(),
                b"uvlo\n" => b"uvlo 3300\r\n".to_vec(),
                b"on\n" | b"off\n" => b"ok\r\n".to_vec(),
                // `set N` and `uvlo N` tell the value that now holds.
                setting if setting.starts_with(b"set ") || setting.starts_with(b"uvlo ") => {
                    let mut echo = setting.trim_ascii_end().to_vec();
                    echo.extend_from_slice(b"\r\n");
                    echo
                }
                _ => Vec::new(),
            }
        }

        /// A device answers in order: nothing overtakes what it owes.
        fn queue(&mut self, pause: Duration, bytes: &[u8]) {
            let now = Instant::now();
            let at = self.input.back().map_or(now, |(last, _)| (*last).max(now)) + pause;
            self.input.extend(bytes.iter().map(|&b| (at, b)));
        }
    }

    impl Port for Fake {
        fn name(&self) -> &'static str {
            "/dev/ttyFAKE"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            let deadline = Instant::now() + timeout;
            loop {
                let next = {
                    let mut s = self.0.lock().unwrap();
                    if s.fail_reads {
                        return Err(Error::device("Broken pipe"));
                    }
                    let now = Instant::now();
                    let there = s.input.iter().take_while(|(at, _)| *at <= now).count();
                    let n = there.min(buf.len());
                    if n > 0 {
                        for b in &mut buf[..n] {
                            *b = s.input.pop_front().expect("counted").1;
                        }
                        return Ok(n);
                    }
                    if now >= deadline {
                        return Ok(0);
                    }
                    s.input.front().map_or(deadline, |(at, _)| *at)
                };
                std::thread::sleep(
                    next.min(deadline)
                        .saturating_duration_since(Instant::now())
                        .min(Duration::from_millis(2)),
                );
            }
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            self.write_timeout(data, Duration::ZERO).map(|_| ())
        }
        fn write_timeout(&mut self, data: &[u8], _: Duration) -> Result<usize> {
            let mut s = self.0.lock().unwrap();
            if s.fail_writes_after.is_some_and(|n| s.writes.len() >= n) {
                return Err(Error::device("Broken pipe"));
            }
            if s.short_writes_after.is_some_and(|n| s.writes.len() >= n) {
                return Ok(0);
            }
            s.writes.push((Instant::now(), data.to_vec()));
            if data == protocol::MONITOR_START {
                for (pause, line) in std::mem::take(&mut s.stream) {
                    s.queue(pause, &line);
                }
                return Ok(data.len());
            }
            let answer = match s.answers.pop_front() {
                Some(answer) if data != protocol::MONITOR_STOP => answer,
                Some(answer) => {
                    s.answers.push_front(answer);
                    Vec::new()
                }
                None if s.silent => Vec::new(),
                None => State::usual(data),
            };
            s.queue(Duration::ZERO, &answer);
            Ok(data.len())
        }
        fn purge(&mut self) -> Result<()> {
            let mut s = self.0.lock().unwrap();
            s.purges += 1;
            let now = Instant::now();
            let n = s.input.iter().take_while(|(at, _)| *at <= now).count();
            s.input.drain(..n);
            Ok(())
        }
    }

    fn device(fake: Fake) -> ReloadProDevice {
        let mut d = ReloadProDevice::from_port(Box::new(fake));
        d.open().unwrap();
        d
    }

    fn params() -> Params {
        Params::parse(protocol::SERIALCOMM).unwrap()
    }

    fn writes(state: &Arc<Mutex<State>>) -> Vec<String> {
        let s = state.lock().unwrap();
        s.writes.iter().map(|(_, w)| String::from_utf8_lossy(w).into_owned()).collect()
    }

    fn stream(state: &Arc<Mutex<State>>, lines: &[(u64, &[u8])]) {
        state.lock().unwrap().stream =
            lines.iter().map(|(ms, l)| (Duration::from_millis(*ms), l.to_vec())).collect();
    }

    /// The packets up to and with the End, within the bound.
    fn until_end(rx: &Received) -> Vec<Packet> {
        let deadline = Instant::now() + CAPTURE_BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            assert!(!left.is_zero(), "no End after {CAPTURE_BOUND:?}");
            let (_, p) = rx.recv_timeout(left).expect("a packet in time");
            let done = matches!(p, Packet::End);
            out.push(p);
            if done {
                return out;
            }
        }
    }

    fn start(d: &mut ReloadProDevice) -> Received {
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        rx
    }

    fn capture(d: &mut ReloadProDevice) -> Vec<Packet> {
        let rx = start(d);
        until_end(&rx)
    }

    /// `(channel, value)` of each analog packet.
    fn readings(packets: &[Packet]) -> Vec<(usize, f32)> {
        packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some((a.channels[0], a.samples[0])),
                _ => None,
            })
            .collect()
    }

    #[allow(clippy::needless_pass_by_value)]
    fn set(d: &mut ReloadProDevice, key: Key, value: Value) -> Result<()> {
        d.set(&key, &value, Some(GROUP))
    }

    #[test]
    fn a_scan_stops_monitoring_and_waits_before_it_asks() {
        let (mut f, state) = fake();
        let version = probe(&mut f, params()).unwrap();
        assert_eq!(version.as_deref(), Some("1.11"));
        assert_eq!(writes(&state), ["monitor 0\r\n", "version\r\n"]);
        let s = state.lock().unwrap();
        let gap = s.writes[1].0 - s.writes[0].0;
        assert!(gap >= Duration::from_millis(50), "{gap:?}");
    }

    #[test]
    fn a_scan_drops_what_is_on_its_way_ahead_of_the_version() {
        let (mut f, state) = fake();
        state.lock().unwrap().queue(Duration::from_millis(10), b"read 1 2 3 4\r\n");
        assert_eq!(probe(&mut f, params()).unwrap().as_deref(), Some("1.11"));
    }

    fn scan_of(reply: &[u8]) -> Option<String> {
        let (mut f, state) = fake();
        state.lock().unwrap().answers.push_back(reply.to_vec());
        probe(&mut f, params()).unwrap()
    }

    #[test]
    fn the_version_check_is_the_cs() {
        assert_eq!(scan_of(b"version 1.10\r\n").as_deref(), Some("1.10"));
        assert_eq!(scan_of(b"version 1.9\r\n").as_deref(), Some("1.9"), "as a float");
        assert_eq!(scan_of(b"version 1.2\r\n").as_deref(), Some("1.2"), "as a float");
        assert_eq!(scan_of(b"version 1.1\r\n").as_deref(), Some("1.1"));
        assert_eq!(scan_of(b"version 1.09\r\n"), None);
        assert_eq!(scan_of(b"version 1.0\r\n"), None);
        assert_eq!(scan_of(b"version 1\r\n"), None);
        assert_eq!(scan_of(b"version abc\r\n"), None);
        assert_eq!(scan_of(b"version \r\n"), None);
        assert_eq!(scan_of(b"\r\nversion 1.11\r\n"), None, "no retry in a scan");
        assert_eq!(scan_of(b"ok\r\n"), None);
    }

    #[test]
    fn a_write_that_fails_or_falls_short_is_no_device() {
        for after in [0, 1] {
            let (mut f, state) = fake();
            state.lock().unwrap().fail_writes_after = Some(after);
            assert!(probe(&mut f, params()).is_err());
            assert_eq!(writes(&state).len(), after);
            let (mut f, state) = fake();
            state.lock().unwrap().short_writes_after = Some(after);
            assert!(probe(&mut f, params()).is_err());
            assert_eq!(writes(&state).len(), after);
        }
    }

    #[test]
    fn settings_go_out_as_the_firmware_takes_them() {
        let (f, state) = fake();
        let mut d = device(f);
        set(&mut d, Key::CurrentLimit, Value::F64(1.5)).unwrap();
        set(&mut d, Key::CurrentLimit, Value::F64(0.0)).unwrap();
        set(&mut d, Key::CurrentLimit, Value::F64(-0.0)).unwrap();
        set(&mut d, Key::CurrentLimit, Value::F64(6.0)).unwrap();
        set(&mut d, Key::CurrentLimit, Value::F64(2.0005)).unwrap();
        set(&mut d, Key::UnderVoltageConditionThreshold, Value::F64(3.3)).unwrap();
        set(&mut d, Key::UnderVoltageConditionThreshold, Value::F64(60.0)).unwrap();
        set(&mut d, Key::Enabled, Value::Bool(true)).unwrap();
        set(&mut d, Key::Enabled, Value::Bool(false)).unwrap();
        assert_eq!(
            writes(&state),
            [
                "set 1500\n",
                "set 0\n",
                "set 0\n",
                "set 6000\n",
                "set 2001\n",
                "uvlo 3300\n",
                "uvlo 60000\n",
                "on\n",
                "off\n"
            ]
        );
    }

    #[test]
    fn a_value_out_of_range_or_nan_sends_nothing() {
        let (f, state) = fake();
        let mut d = device(f);
        for v in [6.0005, 6.001, -1.0, f64::NAN, f64::INFINITY] {
            assert!(set(&mut d, Key::CurrentLimit, Value::F64(v)).is_err(), "{v}");
        }
        for v in [60.0005, 60.001, -1.0, f64::NAN] {
            let threshold = Key::UnderVoltageConditionThreshold;
            assert!(set(&mut d, threshold, Value::F64(v)).is_err(), "{v}");
        }
        assert_eq!(writes(&state), [] as [String; 0]);
    }

    #[test]
    fn a_refused_setting_fails_and_so_does_a_silent_one() {
        let (f, state) = fake();
        let mut d = device(f);
        state.lock().unwrap().answers.push_back(b"err Unknown command 'on'\r\n".to_vec());
        assert!(set(&mut d, Key::Enabled, Value::Bool(true)).is_err());
        state.lock().unwrap().silent = true;
        for (key, value, word) in [
            (Key::Enabled, Value::Bool(false), "off"),
            (Key::CurrentLimit, Value::F64(1.5), "set 1500"),
            (Key::UnderVoltageConditionThreshold, Value::F64(3.3), "uvlo 3300"),
        ] {
            let e = set(&mut d, key, value).unwrap_err().to_string();
            assert!(e.contains(&format!("no answer to '{word}'")), "{e}");
        }
    }

    #[test]
    fn a_setting_cut_short_fails() {
        let (f, state) = fake();
        let mut d = device(f);
        state.lock().unwrap().answers.push_back(b"set 15".to_vec());
        assert!(set(&mut d, Key::CurrentLimit, Value::F64(1.5)).is_err());
        state.lock().unwrap().answers.push_back(b"o".to_vec());
        assert!(set(&mut d, Key::Enabled, Value::Bool(false)).is_err());
    }

    #[test]
    fn a_get_cut_short_fails() {
        let (f, state) = fake();
        let d = device(f);
        state.lock().unwrap().answers.push_back(b"set 15".to_vec());
        assert!(d.get(&Key::CurrentLimit, Some(GROUP)).is_err());
    }

    #[test]
    fn a_setting_whose_read_fails_fails_at_once() {
        let (f, state) = fake();
        let mut d = device(f);
        state.lock().unwrap().fail_reads = true;
        let started = Instant::now();
        assert!(set(&mut d, Key::Enabled, Value::Bool(false)).is_err());
        assert!(started.elapsed() < protocol::REPLY_WAIT, "{:?}", started.elapsed());
    }

    #[test]
    fn a_setting_whose_write_fails_gives_the_ports_error() {
        let (f, state) = fake();
        let mut d = device(f);
        state.lock().unwrap().fail_writes_after = Some(0);
        let e = set(&mut d, Key::Enabled, Value::Bool(true)).unwrap_err().to_string();
        assert!(e.contains("Broken pipe"), "{e}");
    }

    /// Monitoring left on by a capture: lines ahead of the answer, by any count.
    #[test]
    fn a_backlog_ahead_of_the_echo_is_read_through() {
        let (f, state) = fake();
        let mut d = device(f);
        let mut reply = b"read 1 2 3 4\r\n".repeat(40);
        reply.extend_from_slice(b"set 1500\r\n");
        state.lock().unwrap().answers.push_back(reply.clone());
        assert_eq!(d.get(&Key::CurrentLimit, Some(GROUP)).unwrap(), Value::F64(1.5));
        state.lock().unwrap().answers.push_back(reply);
        set(&mut d, Key::CurrentLimit, Value::F64(1.5)).unwrap();
    }

    #[test]
    fn a_monitor_line_is_not_the_echo_of_a_setting() {
        let (f, state) = fake();
        let mut d = device(f);
        state.lock().unwrap().answers.push_back(b"read 1 2\r\n".to_vec());
        assert!(set(&mut d, Key::CurrentLimit, Value::F64(1.5)).is_err());
        state.lock().unwrap().answers.push_back(b"read 1 2\r\n".to_vec());
        assert!(set(&mut d, Key::Enabled, Value::Bool(true)).is_err());
    }

    #[test]
    fn values_are_the_replies_in_single_precision() {
        let (f, state) = fake();
        let d = device(f);
        let get = |key: Key| d.get(&key, Some(GROUP)).unwrap();
        assert_eq!(get(Key::Voltage), Value::F64(f64::from(12.345f32)));
        assert_eq!(get(Key::Current), Value::F64(f64::from(1.499f32)));
        assert_eq!(get(Key::CurrentLimit), Value::F64(1.5));
        assert_eq!(
            get(Key::UnderVoltageConditionThreshold),
            Value::F64(f64::from(3.3f32)),
            "3.2999999523162842"
        );
        assert_eq!(get(Key::UnderVoltageCondition), Value::Bool(true));
        assert_eq!(
            writes(&state),
            ["read\n", "read\n", "set\n", "uvlo\n", "uvlo\n"],
            "each value asks, and the rest of the table does not"
        );
        state.lock().unwrap().answers.push_back(b"uvlo 0\r\n".to_vec());
        assert_eq!(get(Key::UnderVoltageCondition), Value::Bool(false));
    }

    #[test]
    fn what_never_changes_sends_nothing() {
        let (f, state) = fake();
        let d = device(f);
        for key in [
            Key::OverVoltageProtectionEnabled,
            Key::OverCurrentProtectionEnabled,
            Key::OverTemperatureProtection,
        ] {
            assert_eq!(d.get(&key, Some(GROUP)).unwrap(), Value::Bool(true));
        }
        for key in [Key::OverTemperatureProtectionActive, Key::UnderVoltageConditionActive] {
            assert_eq!(d.get(&key, Some(GROUP)).unwrap(), Value::Bool(false));
        }
        assert_eq!(d.get(&Key::Regulation, Some(GROUP)).unwrap(), Value::from("CC"));
        assert_eq!(writes(&state), [] as [String; 0]);
    }

    #[test]
    fn a_value_with_no_answer_or_an_error_or_too_few_numbers_fails() {
        let (f, state) = fake();
        let d = device(f);
        for reply in [&b"err Unknown command 'set'\r\n"[..], b"nonsense\r\n"] {
            state.lock().unwrap().answers.push_back(reply.to_vec());
            assert!(d.get(&Key::CurrentLimit, Some(GROUP)).is_err());
        }
        state.lock().unwrap().answers.push_back(b"read 5\r\n".to_vec());
        assert!(d.get(&Key::Voltage, Some(GROUP)).is_err());
        state.lock().unwrap().silent = true;
        assert!(d.get(&Key::Voltage, Some(GROUP)).is_err());
        assert!(d.get(&Key::UnderVoltageCondition, Some(GROUP)).is_err());
    }

    #[test]
    fn a_line_of_the_devices_own_is_passed_over_and_noted() {
        let (f, state) = fake();
        let d = device(f);
        state.lock().unwrap().answers.push_back(b"overtemp\r\nundervolt\r\nset 77\r\n".to_vec());
        assert_eq!(
            d.get(&Key::CurrentLimit, Some(GROUP)).unwrap(),
            Value::F64(f64::from(0.077f32))
        );
        let active = |key: Key| d.get(&key, Some(GROUP)).unwrap();
        assert_eq!(active(Key::OverTemperatureProtectionActive), Value::Bool(true));
        assert_eq!(active(Key::UnderVoltageConditionActive), Value::Bool(true));
    }

    fn flags(d: &ReloadProDevice) -> (Value, Value) {
        let active = |key: Key| d.get(&key, Some(GROUP)).unwrap();
        (active(Key::OverTemperatureProtectionActive), active(Key::UnderVoltageConditionActive))
    }

    /// The two lines saying that the load turned itself off, queued ahead of each
    /// thing that reads the input.
    #[test]
    fn what_came_ahead_of_a_get_is_noted() {
        let (f, state) = fake();
        let d = device(f);
        state.lock().unwrap().queue(Duration::ZERO, b"overtemp\r\nundervolt\r\n");
        std::thread::sleep(Duration::from_millis(2));
        assert_eq!(d.get(&Key::CurrentLimit, Some(GROUP)).unwrap(), Value::F64(1.5));
        assert_eq!(flags(&d), (Value::Bool(true), Value::Bool(true)));
    }

    #[test]
    fn what_came_ahead_of_a_setting_is_noted() {
        let (f, state) = fake();
        let mut d = device(f);
        state.lock().unwrap().queue(Duration::ZERO, b"undervolt\r\n");
        std::thread::sleep(Duration::from_millis(2));
        set(&mut d, Key::Enabled, Value::Bool(false)).unwrap();
        assert_eq!(flags(&d), (Value::Bool(false), Value::Bool(true)));
    }

    #[test]
    fn the_flags_read_what_has_arrived_and_send_nothing() {
        let (f, state) = fake();
        let d = device(f);
        state.lock().unwrap().queue(Duration::ZERO, b"overtemp\r\n");
        std::thread::sleep(Duration::from_millis(2));
        assert_eq!(flags(&d), (Value::Bool(true), Value::Bool(false)));
        state.lock().unwrap().queue(Duration::ZERO, b"undervolt\r\n");
        std::thread::sleep(Duration::from_millis(2));
        assert_eq!(flags(&d), (Value::Bool(true), Value::Bool(true)));
        assert_eq!(writes(&state), [] as [String; 0]);
    }

    /// Each flag on its own, since either one's look finds both.
    #[test]
    fn each_flag_looks_for_itself() {
        let (f, state) = fake();
        let d = device(f);
        state.lock().unwrap().queue(Duration::ZERO, b"undervolt\r\n");
        std::thread::sleep(Duration::from_millis(2));
        let uvc = Key::UnderVoltageConditionActive;
        assert_eq!(d.get(&uvc, Some(GROUP)).unwrap(), Value::Bool(true));
        let (f, state) = fake();
        let d = device(f);
        state.lock().unwrap().queue(Duration::ZERO, b"overtemp\r\n");
        std::thread::sleep(Duration::from_millis(2));
        let otp = Key::OverTemperatureProtectionActive;
        assert_eq!(d.get(&otp, Some(GROUP)).unwrap(), Value::Bool(true));
    }

    #[test]
    fn a_line_split_across_two_requests_is_still_noted() {
        let (f, state) = fake();
        let d = device(f);
        state.lock().unwrap().queue(Duration::ZERO, b"overt");
        std::thread::sleep(Duration::from_millis(2));
        assert_eq!(flags(&d), (Value::Bool(false), Value::Bool(false)));
        state.lock().unwrap().answers.push_back(b"emp\r\nset 1500\r\n".to_vec());
        assert_eq!(d.get(&Key::CurrentLimit, Some(GROUP)).unwrap(), Value::F64(1.5));
        assert_eq!(flags(&d), (Value::Bool(true), Value::Bool(false)));
    }

    #[test]
    fn what_came_ahead_of_a_capture_is_reported_by_it() {
        let (f, state) = fake();
        stream(&state, &[(0, b"read 2000 2000\r\n")]);
        let mut d = device(f);
        d.limit_samples = 1;
        state.lock().unwrap().queue(Duration::ZERO, b"overtemp\r\nread 1 1\r\nundervolt\r\n");
        std::thread::sleep(Duration::from_millis(2));
        let packets = capture(&mut d);
        assert!(matches!(packets[0], Packet::Header { .. }), "{:?}", packets[0]);
        let Packet::Meta(first) = &packets[1] else { panic!("{:?}", packets[1]) };
        assert_eq!(first[0], (Key::OverTemperatureProtectionActive, Value::Bool(true)));
        let Packet::Meta(second) = &packets[2] else { panic!("{:?}", packets[2]) };
        assert_eq!(second[0], (Key::UnderVoltageConditionActive, Value::Bool(true)));
        assert_eq!(readings(&packets), [(0, 2.0), (1, 2.0)], "the stale reading goes");
        assert_eq!(flags(&d), (Value::Bool(true), Value::Bool(true)));
    }

    #[test]
    fn a_line_cut_by_a_capture_start_is_still_noted() {
        let (f, state) = fake();
        stream(&state, &[(0, b"emp\r\nread 2000 2000\r\n")]);
        let mut d = device(f);
        d.limit_samples = 1;
        state.lock().unwrap().queue(Duration::ZERO, b"overt");
        std::thread::sleep(Duration::from_millis(2));
        capture(&mut d);
        assert_eq!(flags(&d), (Value::Bool(true), Value::Bool(false)));
    }

    #[test]
    fn a_request_drops_what_came_before_it() {
        let (f, state) = fake();
        let d = device(f);
        state.lock().unwrap().queue(Duration::ZERO, b"set 1\r\n");
        std::thread::sleep(Duration::from_millis(2));
        assert_eq!(d.get(&Key::CurrentLimit, Some(GROUP)).unwrap(), Value::F64(1.5));
    }

    #[test]
    fn the_scopes_are_kept_apart() {
        let (f, _state) = fake();
        let mut d = device(f);
        assert!(d.get(&Key::Voltage, None).is_err());
        assert!(d.get(&Key::Voltage, Some("2")).is_err());
        assert!(d.get(&Key::LimitSamples, Some(GROUP)).is_err());
        assert!(d.set(&Key::Enabled, &Value::Bool(true), None).is_err());
        assert!(d.set(&Key::Enabled, &Value::Bool(true), Some("2")).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
        assert!(d.set(&Key::LimitMsec, &Value::U64(0), None).is_err());
        d.set(&Key::LimitSamples, &Value::U64(7), None).unwrap();
        d.set(&Key::LimitMsec, &Value::U64(8), None).unwrap();
        assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(7));
        assert_eq!(d.get(&Key::LimitMsec, None).unwrap(), Value::U64(8));
        assert!(d.list(&Key::CurrentLimit, None).is_err());
        assert!(d.list(&Key::CurrentLimit, Some("2")).is_err());
        assert!(d.options(Some("2")).is_err());
        assert_eq!(d.options(None).unwrap().len(), 3);
        assert_eq!(d.options(Some(GROUP)).unwrap().len(), 12);
    }

    #[test]
    fn a_capture_is_monitor_200_lines_and_at_the_close_monitor_0() {
        let (f, state) = fake();
        stream(
            &state,
            &[
                (0, b"read 1000 12000 0 0\r\n"),
                (0, b"read 2000 13000 0 0\r\n"),
                (0, b"read 9 9\r\n"),
            ],
        );
        let mut d = device(f);
        d.limit_samples = 2;
        let packets = capture(&mut d);
        assert!(matches!(packets[0], Packet::Header { .. }), "{:?}", packets[0]);
        assert!(matches!(packets[1], Packet::FrameBegin));
        assert!(matches!(packets[4], Packet::FrameEnd));
        assert!(matches!(packets[5], Packet::FrameBegin));
        assert_eq!(readings(&packets), [(0, 12.0), (1, 1.0), (0, 13.0), (1, 2.0)]);
        let Packet::Analog(a) = &packets[2] else { panic!("{:?}", packets[2]) };
        assert_eq!((a.mq, a.unit, a.digits), (Mq::Voltage, Unit::Volt, 3));
        assert_eq!(a.flags, MqFlags::DC);
        let Packet::Analog(a) = &packets[3] else { panic!("{:?}", packets[3]) };
        assert_eq!((a.mq, a.unit, a.digits), (Mq::Current, Unit::Ampere, 3));
        assert!(matches!(packets.last(), Some(Packet::End)));
        assert_eq!(writes(&state), ["monitor 200\r\n"], "a stop sends nothing");
        d.close().unwrap();
        assert_eq!(writes(&state), ["monitor 200\r\n", "monitor 0\r\n"]);
        d.close().unwrap();
        assert_eq!(writes(&state).len(), 2, "the port has gone");
    }

    #[test]
    fn every_close_stops_monitoring() {
        let (f, state) = fake();
        let mut d = device(f);
        d.close().unwrap();
        assert_eq!(writes(&state), ["monitor 0\r\n"]);
    }

    #[test]
    fn a_close_goes_on_when_its_write_fails() {
        let (f, state) = fake();
        let mut d = device(f);
        state.lock().unwrap().fail_writes_after = Some(0);
        d.close().unwrap();
    }

    #[test]
    fn a_start_whose_write_fails_starts_nothing() {
        for short in [false, true] {
            let (f, state) = fake();
            let mut d = device(f);
            if short {
                state.lock().unwrap().short_writes_after = Some(0);
            } else {
                state.lock().unwrap().fail_writes_after = Some(0);
            }
            let (tx, rx) = sync_channel(16);
            assert!(d.start(PacketSink::new(tx, 0)).is_err());
            assert!(rx.try_recv().is_err(), "no Header");
            assert!(!d.link().running);
        }
    }

    #[test]
    fn the_input_goes_before_a_capture() {
        let (f, state) = fake();
        stream(&state, &[(0, b"read 2000 2000\r\n")]);
        let mut d = device(f);
        d.limit_samples = 1;
        state.lock().unwrap().queue(Duration::ZERO, b"read 1 1\r\n");
        std::thread::sleep(Duration::from_millis(2));
        assert_eq!(readings(&capture(&mut d)), [(0, 2.0), (1, 2.0)]);
    }

    #[test]
    fn the_other_lines_are_metadata_or_a_log() {
        let (f, state) = fake();
        stream(
            &state,
            &[
                (0, b"overtemp\r\n"),
                (0, b"undervolt\r\n"),
                (0, b"set 1234\r\n"),
                (0, b"uvlo 3300\r\n"),
                (0, b"uvlo 0\r\n"),
                (0, b"err Unknown command 'x'\r\n"),
                (0, b"ok\r\nnonsense\r\n\r\n"),
                (0, b"read 5\r\n"),
                (0, b"read 1000 2000\r\n"),
            ],
        );
        let mut d = device(f);
        d.limit_samples = 1;
        let packets = capture(&mut d);
        let metas: Vec<(Key, Value)> = packets
            .iter()
            .filter_map(|p| match p {
                Packet::Meta(m) => Some(m[0].clone()),
                _ => None,
            })
            .collect();
        assert_eq!(
            metas,
            [
                (Key::OverTemperatureProtectionActive, Value::Bool(true)),
                (Key::UnderVoltageConditionActive, Value::Bool(true)),
                (Key::CurrentLimit, Value::F64(f64::from(1.234f32))),
                (Key::UnderVoltageCondition, Value::Bool(true)),
                (Key::UnderVoltageConditionThreshold, Value::F64(f64::from(3.3f32))),
                (Key::UnderVoltageCondition, Value::Bool(false)),
            ]
        );
        assert_eq!(readings(&packets), [(0, 2.0), (1, 1.0)], "only the whole reading counts");
        let active = |key: Key| d.get(&key, Some(GROUP)).unwrap();
        assert_eq!(active(Key::OverTemperatureProtectionActive), Value::Bool(true));
        assert_eq!(active(Key::UnderVoltageConditionActive), Value::Bool(true));
    }

    #[test]
    fn a_line_in_two_pieces_is_one_line() {
        let (f, state) = fake();
        stream(&state, &[(0, b"read 1000 "), (300, b"12000 0 0\r\n")]);
        let mut d = device(f);
        d.limit_samples = 1;
        assert_eq!(readings(&capture(&mut d)), [(0, 12.0), (1, 1.0)]);
    }

    #[test]
    fn a_line_too_long_is_cut_at_99_bytes() {
        let (f, state) = fake();
        let mut long = b"read 1000 12000 ".to_vec();
        long.resize(99, b'x');
        long.extend_from_slice(b"read 2000 13000\r\n");
        stream(&state, &[(0, &long)]);
        let mut d = device(f);
        d.limit_samples = 2;
        assert_eq!(readings(&capture(&mut d)), [(0, 12.0), (1, 1.0), (0, 13.0), (1, 2.0)]);
    }

    #[test]
    fn a_time_limit_ends_a_silent_capture_at_its_time() {
        let (f, _state) = fake();
        let mut d = device(f);
        d.limit_msec = 300;
        let started = Instant::now();
        let packets = capture(&mut d);
        let took = started.elapsed();
        assert_eq!(packets.len(), 2, "{packets:?}");
        assert!(took >= Duration::from_millis(300), "{took:?}");
        assert!(took < Duration::from_secs(3), "{took:?}");
    }

    #[test]
    fn a_silent_capture_with_a_sample_limit_gives_up_after_the_bound() {
        assert_eq!(NO_READING_BOUND, Duration::from_secs(5));
        let (f, _state) = fake();
        let mut d = device(f);
        d.limit_samples = 1;
        d.no_reading = Duration::from_millis(400);
        let started = Instant::now();
        let packets = capture(&mut d);
        let took = started.elapsed();
        assert_eq!(packets.len(), 2, "{packets:?}");
        assert!(took >= Duration::from_millis(400), "{took:?}");
        assert!(took < Duration::from_secs(4), "{took:?}");
    }

    #[test]
    fn the_bound_counts_from_the_last_reading_and_not_from_other_lines() {
        let (f, state) = fake();
        stream(
            &state,
            &[
                (200, b"read 1000 1000\r\n"),
                (200, b"read 1000 1000\r\n"),
                (200, b"read 1000 1000\r\n"),
                (600, b"nonsense\r\n"),
                (600, b"nonsense\r\n"),
                (600, b"nonsense\r\n"),
                // Within the bound of the last line, past it of the last reading.
                (600, b"read 1000 1000\r\n"),
            ],
        );
        let mut d = device(f);
        d.limit_samples = 100;
        d.no_reading = Duration::from_secs(1);
        let started = Instant::now();
        let packets = capture(&mut d);
        assert_eq!(readings(&packets).len(), 6, "three readings, and lines that are none");
        let took = started.elapsed();
        assert!(took >= Duration::from_millis(1600), "from the third reading: {took:?}");
    }

    #[test]
    fn a_time_limit_lifts_the_bound() {
        let (f, _state) = fake();
        let mut d = device(f);
        d.limit_samples = 1;
        d.limit_msec = 600;
        d.no_reading = Duration::from_millis(100);
        let started = Instant::now();
        let packets = capture(&mut d);
        let took = started.elapsed();
        assert_eq!(packets.len(), 2, "{packets:?}");
        assert!(took >= Duration::from_millis(600), "{took:?}");
    }

    #[test]
    fn no_limit_means_no_bound() {
        let (f, _state) = fake();
        let mut d = device(f);
        d.no_reading = Duration::from_millis(50);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::Header { .. }))));
        assert!(matches!(
            rx.recv_timeout(Duration::from_millis(400)),
            Err(RecvTimeoutError::Timeout)
        ));
        d.stop().unwrap();
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::End))));
    }

    #[test]
    fn a_stop_ends_a_capture_that_has_data_to_give() {
        let (f, state) = fake();
        let line: &[u8] = b"read 1000 1000\r\n";
        let lines: Vec<(u64, &[u8])> = (0..2000).map(|_| (5, line)).collect();
        stream(&state, &lines);
        let mut d = device(f);
        let rx = start(&mut d);
        let mut seen = 0;
        while seen < 2 {
            let (_, p) = rx.recv_timeout(CAPTURE_BOUND).expect("a packet in time");
            seen += usize::from(matches!(p, Packet::FrameEnd));
        }
        d.stop().unwrap();
        let rest = until_end(&rx);
        // 4 packets a line, 8000 in all: a stall may let some more through.
        assert!(rest.len() < 4000, "{}", rest.len());
    }

    #[test]
    fn a_port_gone_ends_a_capture() {
        let (f, state) = fake();
        let mut d = device(f);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::Header { .. }))));
        state.lock().unwrap().fail_reads = true;
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::End))));
    }

    #[test]
    fn a_get_in_a_capture_takes_the_line_the_capture_sees() {
        let (f, state) = fake();
        // The feeder's lines alone answer: a usual one too would be a stale line
        // for the next get.
        state.lock().unwrap().silent = true;
        let mut d = device(f);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::Header { .. }))));
        let shared = Arc::clone(&state);
        let feeder = std::thread::spawn(move || {
            // Each once its request has gone.
            for (done, reply) in [
                (2, &b"read 1499 12345\r\n"[..]),
                (3, b"read 2000 12345\r\n"),
                (4, b"set 1500\r\n"),
                (5, b"uvlo 3300\r\n"),
            ] {
                let deadline = Instant::now() + CAPTURE_BOUND;
                while shared.lock().unwrap().writes.len() < done && Instant::now() < deadline {
                    std::thread::sleep(Duration::from_millis(1));
                }
                shared.lock().unwrap().queue(Duration::from_millis(5), reply);
            }
        });
        let started = Instant::now();
        let get = |key: Key| d.get(&key, Some(GROUP)).unwrap();
        assert_eq!(get(Key::Voltage), Value::F64(f64::from(12.345f32)));
        assert_eq!(get(Key::Current), Value::F64(2.0));
        assert_eq!(get(Key::CurrentLimit), Value::F64(1.5));
        assert_eq!(get(Key::UnderVoltageConditionThreshold), Value::F64(f64::from(3.3f32)));
        // Each is woken by its line, not by the 5 s it may wait.
        assert!(started.elapsed() < Duration::from_secs(5), "{:?}", started.elapsed());
        feeder.join().unwrap();
        d.stop().unwrap();
        assert_eq!(writes(&state), ["monitor 200\r\n", "read\n", "read\n", "set\n", "uvlo\n"]);
        d.close().unwrap();
    }

    #[test]
    fn a_get_in_a_capture_gives_up_with_no_line_and_when_it_ends() {
        let (f, state) = fake();
        state.lock().unwrap().silent = true;
        let mut d = device(f);
        d.capture_reply = Duration::from_millis(200);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::Header { .. }))));
        let started = Instant::now();
        assert!(d.get(&Key::Voltage, Some(GROUP)).is_err());
        let took = started.elapsed();
        assert!(took >= Duration::from_millis(200), "{took:?}");
        assert!(took < Duration::from_secs(3), "{took:?}");
        assert_eq!(CAPTURE_REPLY_TIMEOUT, Duration::from_secs(5));

        let stop = Arc::clone(&d.worker.as_ref().unwrap().stop);
        d.capture_reply = CAPTURE_BOUND * 3;
        let stopper = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(200));
            stop.store(true, Ordering::SeqCst);
        });
        let started = Instant::now();
        assert!(d.get(&Key::Current, Some(GROUP)).is_err());
        assert!(started.elapsed() < CAPTURE_BOUND);
        stopper.join().unwrap();
    }

    /// No answer is no failure here, and the answer is the capture's to see: a
    /// setting that waited would hold the link and read it first.
    #[test]
    fn a_setting_in_a_capture_only_writes() {
        let (f, state) = fake();
        let mut d = device(f);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::Header { .. }))));
        set(&mut d, Key::CurrentLimit, Value::F64(1.5)).unwrap();
        let (_, meta) = rx.recv_timeout(CAPTURE_BOUND).expect("the capture sees the echo");
        let Packet::Meta(meta) = meta else { panic!("{meta:?}") };
        assert_eq!(meta[0], (Key::CurrentLimit, Value::F64(1.5)));
        state.lock().unwrap().silent = true;
        set(&mut d, Key::CurrentLimit, Value::F64(1.5)).unwrap();
        d.stop().unwrap();
        assert_eq!(writes(&state), ["monitor 200\r\n", "set 1500\n", "set 1500\n"]);
    }

    /// Each reading queued ahead of a request is the capture's. A request that read
    /// the input itself would take some of them, with the link in its hands.
    #[test]
    fn a_request_in_a_capture_leaves_the_input_to_it() {
        const ROUNDS: usize = 50;
        let (f, state) = fake();
        state.lock().unwrap().silent = true;
        let mut d = device(f);
        d.limit_samples = u64::try_from(ROUNDS).unwrap();
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::Header { .. }))));
        for _ in 0..ROUNDS {
            {
                // With the link held the capture is not in a read.
                let _link = d.link();
                state.lock().unwrap().queue(Duration::ZERO, b"read 1000 1000\r\n");
            }
            set(&mut d, Key::CurrentLimit, Value::F64(1.5)).unwrap();
        }
        assert_eq!(readings(&until_end(&rx)).len(), ROUNDS * 2);
    }

    #[test]
    fn a_second_start_while_running_is_busy_and_after_the_end_goes() {
        let (f, state) = fake();
        let mut d = device(f);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::Header { .. }))));
        let (tx, _unused) = sync_channel(16);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.stop().unwrap();
        until_end(&rx);
        stream(&state, &[(0, b"read 1000 1000\r\n")]);
        d.limit_samples = 1;
        assert_eq!(readings(&capture(&mut d)).len(), 2);
    }

    #[test]
    fn a_start_with_no_port_is_refused() {
        let (f, _state) = fake();
        let mut d = ReloadProDevice::from_port(Box::new(f));
        let (tx, _rx) = sync_channel(16);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
    }

    /// A queue no one reads must not hold the close.
    #[test]
    fn a_full_queue_does_not_hold_the_close() {
        let (f, state) = fake();
        let line: &[u8] = b"read 1000 1000\r\n";
        let lines: Vec<(u64, &[u8])> = (0..64).map(|_| (0, line)).collect();
        stream(&state, &lines);
        let mut d = device(f);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Until the thread has read what it cannot send.
        let deadline = Instant::now() + CAPTURE_BOUND;
        while !state.lock().unwrap().input.is_empty() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(2));
        }
        let (done_tx, done_rx) = sync_channel(1);
        let closer = std::thread::spawn(move || {
            d.close().unwrap();
            done_tx.send(()).unwrap();
        });
        assert!(done_rx.recv_timeout(CAPTURE_BOUND).is_ok(), "the close hangs");
        closer.join().unwrap();
        assert_eq!(writes(&state).last().map(String::as_str), Some("monitor 0\r\n"));
        drop(rx);
    }

    /// A sender with no room waits for [`END_AFTER_STOP`] after the stop, which a
    /// close must see out: its thread holds the last other reference.
    #[test]
    fn a_close_joins_the_capture() {
        let (f, state) = fake();
        stream(&state, &[(0, b"read 1000 1000\r\n")]);
        let mut d = device(f);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + CAPTURE_BOUND;
        while !state.lock().unwrap().input.is_empty() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(2));
        }
        d.close().unwrap();
        assert_eq!(Arc::strong_count(&d.shared), 1, "the thread goes on");
        drop(rx);
    }

    #[test]
    fn a_device_dropped_in_a_capture_stops_its_thread() {
        let (f, _state) = fake();
        let mut d = device(f);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::Header { .. }))));
        drop(d);
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::End))));
    }

    #[test]
    fn the_table_is_the_cs() {
        let (f, _state) = fake();
        let d = device(f);
        assert_eq!(
            d.list(&Key::CurrentLimit, Some(GROUP)).unwrap(),
            ValueList::RangeStepF64 { min: 0.0, max: 6.0, step: 0.001 }
        );
        assert_eq!(
            d.list(&Key::UnderVoltageConditionThreshold, Some(GROUP)).unwrap(),
            ValueList::RangeStepF64 { min: 0.0, max: 60.0, step: 0.001 }
        );
        assert_eq!(
            d.list(&Key::Regulation, Some(GROUP)).unwrap(),
            ValueList::Str(vec!["CC".to_owned()])
        );
        assert_eq!(d.info().conn, None);
        assert_eq!(d.channel_groups().len(), 1);
    }
}
