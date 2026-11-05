// Counterpart in sigrok:
//   libsigrok/src/hardware/motech-lps-30x/api.c,
//   libsigrok/src/hardware/motech-lps-30x/protocol.c,
//   Copyright (C) 2014 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com> (code from atten-pps3xxx),
//   with commits by Uwe Hermann, Lars-Peter Clausen, Martin Ling,
//   Gerhard Sittig, Aurelien Jacobs, Soeren Apel, Daniel Elstner,
//   Wolfram Sang, Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Motech LPS-301 power supply, one channel on a serial port at 2400 baud.
//!
//! C has tables for the LPS-301 to LPS-305 and registers one driver,
//! `motech-lps-301`, whose scan has the model fixed, so `MODEL` is only checked
//! for its `LPS-` (firmware 1.17 is said to give no number) and any LPS shows as
//! an LPS-301. So this port has that model alone. A scan asks `MODEL`, `VERSION`
//! (whose failure is no error, as firmware 1.17 is said to fail it while the
//! output is on) and `STATUS`, and closes the port; an open opens it again.
//!
//! `--get` and `--show` answer the status from the scan, what was set since, and
//! the readings of a capture, 0 until there is one: the device is never asked
//! for its set points. A setting is one line and its `OK`.
//!
//! A capture has one request out at a time: `VOUT1`, `IOUT1`, `STATUS`, the next
//! one in the same pass of the loop that read the last one's `OK`, the first 50
//! ms after the start. A `STATUS` answer makes the sample: two packets, volts and
//! amperes. When a limit ends the capture in a pass that read an `OK`, one more
//! request goes out, as it does in C, where the check of the state after a stop
//! does not look at the stop.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - A request expires after 250 ms and the next one follows once the line is
//!   free ([`Settle`]), as no answer says what it answers. A late answer is
//!   dropped; one later still shows as a status that is no number, which makes
//!   no sample and waits for the line again, and no round makes a sample until a
//!   good status has come after the expiry. C compares a
//!   monotonic time with the real time it stored (`protocol.c:199`, `:239`),
//!   so none ever expires and a reply that does not come holds it.
//! - A reading that is no number or does not come means no sample for that round,
//!   where C sends 0. A capture that only a sample limit can end stops after
//!   [`SILENCE_BOUND`] with none.
//! - A bare `OK` ends a request, where C waits for another, and a line too long
//!   is one bad line, where C stops reading for good.
//! - After a limit or a stop the state is left so a client can set and capture
//!   again: C never clears `acq_running` (`api.c:697`, read at `:552`), so it refuses every set
//!   after the first capture.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::serial::{Params, Port};
use protocol::{CAPTURE_LINE_MAX, CURRENT, Link, QUIET, REQ_TIMEOUT, SERIALCOMM, Settle, VOLTAGE};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::PowerSupply];
/// `channel_modes`. An LPS-301 lists the first and takes them all.
const CHANNEL_MODES: [&str; 3] = ["Independent", "Track1", "Track2"];

static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::ChannelConfig, Access::GET_SET_LIST),
];
static GROUP_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::VoltageTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::Enabled, Access::GET_SET),
];

/// The session source's timeout in `dev_acquisition_start`.
const POLL_INTERVAL: Duration = Duration::from_millis(50);
/// How long a capture that only a sample limit can end goes with no sample,
/// where C waits for ever. A request is 250 ms and a wait for the line at worst,
/// and a supply busy on its panel should not end a capture.
pub const SILENCE_BOUND: Duration = Duration::from_secs(2);
/// How often a wait looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);
/// The most one pass reads, far above what 2400 baud brings in a pass.
const PASS_BYTES: usize = 4096;
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// The `motech-lps-301` driver.
pub struct MotechLps301Driver;

impl Driver for MotechLps301Driver {
    fn id(&self) -> &'static str {
        "motech-lps-301"
    }

    fn long_name(&self) -> &'static str {
        "Motech LPS-301"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = SERIALCOMM.to_owned();
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
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        match probe(port, params) {
            Ok(Some((link, version))) => {
                Ok(vec![Box::new(MotechLpsDevice::new(link, conn, version))])
            }
            Ok(None) => Ok(Vec::new()),
            Err(e) => {
                log::debug!("Scan failed: {e}.");
                Ok(Vec::new())
            }
        }
    }
}

/// `do_scan` after the open. The port goes when this returns, as `serial_close`
/// on the way out, on each path, where C's early returns leave it open.
fn probe(port: Box<dyn Port>, params: Params) -> Result<Option<(Link, String)>> {
    let mut link = Link::new(Some(port), params);
    let model = link.query("MODEL")?;
    if !protocol::c_string(&model).starts_with(b"LPS-") {
        log::debug!("Unknown model code \"{}\"!", String::from_utf8_lossy(&model));
        return Ok(None);
    }
    let mut version = String::new();
    match link.query("VERSION") {
        Ok(reply) => {
            let reply = protocol::c_string(&reply);
            let Some(rest) = reply.strip_prefix(b"Ver-") else {
                log::debug!("Version string {} not recognized.", String::from_utf8_lossy(reply));
                return Ok(None);
            };
            String::from_utf8_lossy(rest).trim_end_matches(is_c_space).clone_into(&mut version);
        }
        Err(e) => {
            log::debug!("VERSION: {e}.");
            log::error!("Failed to query for hardware version: {}.", protocol::GENERIC_ERROR);
        }
    }
    link.query_status()?;
    link.port = None;
    Ok(Some((link, version)))
}

/// What `g_strstrip` takes off.
fn is_c_space(c: char) -> bool {
    u8::try_from(c).is_ok_and(protocol::is_glib_space)
}

/// One power supply.
pub struct MotechLpsDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    conn: String,
    limit_samples: u64,
    limit_msec: u64,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
    /// A capture is running: settings would mix with its requests.
    capturing: Arc<AtomicBool>,
    silence_bound: Duration,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl MotechLpsDevice {
    fn new(link: Link, conn: String, version: String) -> Self {
        MotechLpsDevice {
            info: DeviceInfo {
                driver: "motech-lps-301",
                vendor: "Motech".to_owned(),
                model: "LPS-301".to_owned(),
                version,
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "CH1".to_owned())],
            groups: vec![ChannelGroup::new("CG1", vec![0])],
            conn,
            limit_samples: 0,
            limit_msec: 0,
            pending: None,
            link: Arc::new(Mutex::new(link)),
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            silence_bound: SILENCE_BOUND,
        }
    }

    /// A device on a port that stands in for the hardware, without a scan's
    /// reading. [`Device::open`] takes it.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device = Self::new(Link::new(None, params), conn, String::new());
        device.pending = Some(port);
        device
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    fn check_group(&self, group: &str) -> Result<()> {
        if self.groups.iter().any(|g| g.name == group) {
            Ok(())
        } else {
            Err(Error::ChannelGroup(group.to_owned()))
        }
    }

    fn set_channel(&mut self, key: &Key, value: &Value) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        match key {
            Key::VoltageTarget => {
                let volts = in_range(key, value, VOLTAGE[1])?;
                link.command(&protocol::voltage_command(volts))?;
                link.voltage_target = f64::from(volts);
            }
            Key::CurrentLimit => {
                let amps = in_range(key, value, CURRENT[1])?;
                link.command(&protocol::current_command(amps))?;
                link.current_limit = f64::from(amps);
            }
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(|| Error::arg("enabled"))?;
                if on != link.output_enabled {
                    link.command(if on { "OUT1" } else { "OUT0" })?;
                    link.output_enabled = on;
                }
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }
}

/// A value from 0 to `max`, in single precision as sigrok-cli's `strtof` has it.
/// `api.c`'s check is `dval < 0 || dval > max`, so it takes a current under the
/// 5 mA its table lists, which is kept; and a NaN goes out as `VSET1   nan` to a
/// live output, and -0.0 as `-0.000`, which nothing says a supply parses: here
/// the one is refused and the other is 0.
fn in_range(key: &Key, value: &Value, max: f64) -> Result<f32> {
    let refused = || Error::Arg(format!("{} out of range", key.id()));
    let single = value.as_f64().ok_or_else(refused)? as f32;
    if single >= 0.0 && single <= max as f32 { Ok(single + 0.0) } else { Err(refused()) }
}

impl Device for MotechLpsDevice {
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
        Ok(match group {
            None => DEV_OPTIONS.to_vec(),
            Some(g) => {
                self.check_group(g)?;
                GROUP_OPTIONS.to_vec()
            }
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let Some(group) = group else {
            return Ok(match key {
                Key::LimitSamples => Value::U64(self.limit_samples),
                Key::LimitMsec => Value::U64(self.limit_msec),
                Key::ChannelConfig => Value::from(CHANNEL_MODES[0]),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            });
        };
        self.check_group(group)?;
        let link = self.link();
        Ok(match key {
            Key::Voltage => Value::F64(link.voltage),
            Key::VoltageTarget => Value::F64(link.voltage_target),
            Key::Current => Value::F64(link.current),
            Key::CurrentLimit => Value::F64(link.current_limit),
            Key::Enabled => Value::Bool(link.output_enabled),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        // "Would cause a mess with commands", as C's comment has it.
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::not_supported(format!("{} while capturing", key.id())));
        }
        if let Some(group) = group {
            self.check_group(group)?;
            return self.set_channel(key, value);
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            // `core::config_set` refuses a 0 and says so; this is for a library
            // caller that comes straight here.
            Key::LimitSamples => {
                self.limit_samples = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::LimitMsec => {
                self.limit_msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            // `api.c` takes every mode and does nothing with the others on this
            // model: there is no second channel to track.
            Key::ChannelConfig => {
                let mode = value.as_str().ok_or_else(bad)?;
                if !CHANNEL_MODES.contains(&mode) {
                    return Err(bad());
                }
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let Some(group) = group else {
            return match key {
                Key::ChannelConfig => Ok(ValueList::Str(vec![CHANNEL_MODES[0].to_owned()])),
                other => Err(Error::UnknownKey(other.id().to_owned())),
            };
        };
        self.check_group(group)?;
        let range = match key {
            Key::VoltageTarget => VOLTAGE,
            Key::CurrentLimit => CURRENT,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        };
        Ok(ValueList::RangeStepF64 { min: range[0], max: range[1], step: range[2] })
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        if link.port.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&self.conn, link.params)?,
        };
        link.port = Some(port);
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        self.link().port = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        if self.link().port.is_none() {
            return Err(Error::device("device is not open"));
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            enabled: self.channels[0].enabled,
            silence_bound: self.silence_bound,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new()
            .name("motech-lps-301 acquisition".to_owned())
            .spawn(move || {
                capture.run(&sink, &stop2);
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2);
            });
        match spawned {
            Ok(handle) => {
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                self.capturing.store(false, Ordering::SeqCst);
                Err(Error::Io(e))
            }
        }
    }

    /// Asks the thread to end, and does not wait: it may be sending. Nothing
    /// goes to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// `End`, waiting for room in the queue for as long as nobody asked to stop, and
/// after a stop only for [`END_AFTER_STOP`]: a close joins this thread.
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

/// `acq_req`: what was asked last.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Request {
    None,
    Volts,
    Amps,
    Status,
}

impl Request {
    fn next(self) -> Request {
        match self {
            Request::None | Request::Status => Request::Volts,
            Request::Volts => Request::Amps,
            Request::Amps => Request::Status,
        }
    }

    fn command(self) -> &'static str {
        match self {
            Request::None | Request::Volts => "VOUT1",
            Request::Amps => "IOUT1",
            Request::Status => "STATUS",
        }
    }
}

/// `acq_req_pending`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pending {
    None,
    Reply,
    Ok,
}

/// The reading loop's state: the rest of `dev_context`.
struct Round {
    request: Request,
    pending: Pending,
    sent_at: Instant,
    /// What has come of a line so far.
    line: Vec<u8>,
    /// The line has run past what it may hold, and is read up to its end.
    too_long: bool,
    /// Both readings of this round came as numbers.
    readings_good: bool,
    /// The samples this pass made: volts and amperes.
    ready: Vec<(f64, f64)>,
    /// Nothing goes out while this runs, and what comes is dropped.
    settle: Option<Settle>,
    step: Step,
}

/// Whether answers still go with their requests.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    /// As far as anything shows.
    Good,
    /// A request expired, and no status has been read since: answers may be one
    /// or two out of step.
    Suspect,
    /// They are: wait for the line again when this request has ended.
    Out,
}

impl Round {
    /// Starts over: what came of a line is part of what goes unread, or it would
    /// be read as the head of the next request's answer.
    fn wait_for_the_line(&mut self) {
        self.line.clear();
        self.too_long = false;
        self.step = Step::Suspect;
        self.settle = Some(Settle::new(QUIET, Instant::now()));
    }

    /// A byte from the line, for the wait if there is one.
    fn take(&mut self, byte: u8, known: &mut Link) {
        if let Some(settle) = &mut self.settle {
            if settle.byte(byte) {
                self.settle = None;
            }
            return;
        }
        self.feed(byte, known);
    }

    /// One byte, as `motech_lps_30x_receive_data`'s loop takes it.
    fn feed(&mut self, byte: u8, known: &mut Link) {
        if self.line.is_empty() && !self.too_long && protocol::is_glib_space(byte) {
            return;
        }
        if byte == b'\r' || byte == b'\n' {
            let line = std::mem::take(&mut self.line);
            let too_long = std::mem::take(&mut self.too_long);
            self.process_line(&line, too_long, known);
        } else if self.line.len() < CAPTURE_LINE_MAX {
            self.line.push(byte);
        } else {
            self.too_long = true;
        }
    }

    /// `process_line`, past its two errors: a bare `OK` ends the request, and
    /// a line ahead of `OK` does not, so the next request goes when the device
    /// takes one (its manual: a command before `OK` is rejected).
    fn process_line(&mut self, line: &[u8], too_long: bool, known: &mut Link) {
        let text = protocol::c_string(line);
        match self.pending {
            Pending::None => log::debug!("A line no one asked for: '{}'.", show(line)),
            Pending::Reply => {
                self.reply(if too_long { None } else { Some(text) }, line, known);
                self.pending = if text == b"OK" && !too_long { Pending::None } else { Pending::Ok };
            }
            Pending::Ok => {
                if text == b"OK" && !too_long {
                    self.pending = Pending::None;
                } else {
                    log::error!("Unexpected reply while waiting for OK: '{}'", show(line));
                }
            }
        }
    }

    fn reply(&mut self, text: Option<&[u8]>, line: &[u8], known: &mut Link) {
        if self.request == Request::Status {
            if let Some(status) = text.and_then(protocol::atoi) {
                known.output_enabled = protocol::output_of(status);
                self.step = Step::Good;
            } else {
                log::error!("Failed to convert '{}' to int", show(line));
                // Without an expiry it is the device's word, and as in C the
                // readings stand. After one it may be a reading, and they may
                // not: 576 V was the status.
                if self.step != Step::Good {
                    self.step = Step::Out;
                    return;
                }
            }
            if self.readings_good {
                self.ready.push((known.voltage, known.current));
            }
            return;
        }
        match text.and_then(protocol::atod_ascii) {
            Some(value) if self.request == Request::Volts => known.voltage = value,
            Some(value) => known.current = value,
            None => {
                log::error!("Failed to convert '{}' to double", show(line));
                self.readings_good = false;
            }
        }
    }
}

fn show(line: &[u8]) -> std::borrow::Cow<'_, str> {
    String::from_utf8_lossy(protocol::c_string(line))
}

/// One capture, on its own thread: `motech_lps_30x_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    limit_msec: u64,
    /// `-C`, which C ignores: its packets go out whatever it says.
    enabled: bool,
    silence_bound: Duration,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) {
        if !sink.send_while(Packet::header(), || !stop.load(Ordering::SeqCst)) {
            return;
        }
        if let Err(e) = self.passes(sink, stop) {
            log::error!("Acquisition failed: {e}.");
        }
    }

    /// Whether a device that stops answering would keep this capture for ever:
    /// nothing but samples can end it. `--time` and `--continuous` end by their
    /// own limit or a stop.
    fn bounded(&self) -> bool {
        self.limit_samples > 0 && self.limit_msec == 0
    }

    /// Reads what there is, as C's callback: true for a pass, false while nothing
    /// has come and none is due. A device that never stops talking cannot hold it.
    fn wait_for_pass(&self, round: &mut Round, due: Instant) -> Result<bool> {
        let mut link = self.link.lock().expect("link lock");
        let left = due.saturating_duration_since(Instant::now()).min(STOP_CHECK);
        let Some(first) = link.read_byte(left)? else {
            return Ok(Instant::now() >= due);
        };
        round.take(first, &mut link);
        for _ in 0..PASS_BYTES {
            let Some(byte) = link.read_byte(Duration::ZERO)? else { break };
            round.take(byte, &mut link);
        }
        Ok(true)
    }

    fn send_samples(&self, sink: &PacketSink, stop: &AtomicBool, round: &mut Round) -> Option<u64> {
        let go_on = || !stop.load(Ordering::SeqCst);
        let mut samples = 0;
        for (volts, amps) in round.ready.drain(..) {
            if self.enabled
                && !(sink.send_while(voltage_packet(volts), go_on)
                    && sink.send_while(current_packet(amps), go_on))
            {
                return None;
            }
            samples += 1;
        }
        Some(samples)
    }

    fn passes(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let started = Instant::now();
        // With nothing left open the first request goes at the first pass, as
        // in C. With an answer on its way from the last capture, not before it.
        let left_open = self.link.lock().expect("link lock").open_request.take();
        let settle = left_open
            .map_or_else(|| Settle::new(POLL_INTERVAL, started), |since| Settle::new(QUIET, since));
        let mut round = Round {
            request: Request::None,
            pending: Pending::None,
            sent_at: started,
            line: Vec::new(),
            too_long: false,
            readings_good: true,
            ready: Vec::new(),
            settle: Some(settle),
            step: Step::Good,
        };
        let ended = self.rounds(sink, stop, started, &mut round);
        if round.pending != Pending::None || round.settle.is_some() {
            self.link.lock().expect("link lock").open_request = Some(round.sent_at);
        }
        ended
    }

    fn rounds(
        &self,
        sink: &PacketSink,
        stop: &AtomicBool,
        started: Instant,
        round: &mut Round,
    ) -> Result<()> {
        let mut samples = 0u64;
        let mut last_sample = started;
        let mut due = started + POLL_INTERVAL;
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            if !self.wait_for_pass(round, due)? {
                // With the lock free: a get must not starve behind this loop.
                std::thread::sleep(Duration::from_millis(1));
                continue;
            }
            due = Instant::now() + POLL_INTERVAL;
            let Some(new) = self.send_samples(sink, stop, round) else { return Ok(()) };
            if new > 0 {
                samples += new;
                last_sample = Instant::now();
            }
            let time_up =
                self.limit_msec > 0 && started.elapsed() >= Duration::from_millis(self.limit_msec);
            let done = time_up || (self.limit_samples > 0 && samples >= self.limit_samples);
            if self.bounded() && !done && last_sample.elapsed() >= self.silence_bound {
                return Err(Error::device("no sample from the device"));
            }
            let mut link = self.link.lock().expect("link lock");
            if round.pending != Pending::None && round.sent_at.elapsed() > REQ_TIMEOUT {
                log::debug!("Request timeout: {:?}.", round.request);
                // Whatever had come of it, no sample rests on this round.
                round.readings_good = false;
                round.pending = Pending::None;
                round.wait_for_the_line();
            }
            if round.step == Step::Out && round.pending == Pending::None {
                round.wait_for_the_line();
            }
            if round.settle.as_ref().is_some_and(Settle::over) {
                round.settle = None;
            }
            if round.pending == Pending::None && round.settle.is_none() {
                round.request = round.request.next();
                if round.request == Request::Volts {
                    // A round begun out of step may come back in step within it:
                    // an answer dropped between two requests does that, and its
                    // good status would vouch for volts that were the last status.
                    round.readings_good = round.step == Step::Good;
                }
                let sent = link.send(round.request.command());
                round.sent_at = Instant::now();
                round.pending = Pending::Reply;
                // As C: a failed last request says nothing.
                if !done {
                    sent?;
                }
            }
            if done {
                return Ok(());
            }
        }
    }
}

fn voltage_packet(volts: f64) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![0],
        samples: vec![volts as f32],
        mq: Mq::Voltage,
        flags: MqFlags::DC,
        unit: Unit::Volt,
        digits: 3,
    })
}

fn current_packet(amps: f64) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![0],
        samples: vec![amps as f32],
        mq: Mq::Current,
        flags: MqFlags::empty(),
        unit: Unit::Ampere,
        digits: 4,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit
    /// that does not bite must fail, not hang the run.
    const CAPTURE_BOUND: Duration = Duration::from_secs(20);

    type Received = Receiver<(usize, Packet)>;
    type Seen = Arc<Mutex<Vec<(Instant, Vec<u8>)>>>;

    /// A supply that answers as the protocol says, noting each write and when.
    struct Fake {
        seen: Seen,
        /// Each byte and when it arrives.
        input: VecDeque<(Instant, u8)>,
        /// Commands answered, then silence.
        answers: Option<usize>,
        /// What is said instead of the usual, for each command in turn.
        replies: VecDeque<Vec<u8>>,
        fail_writes_after: Option<usize>,
        fail_reads: bool,
        /// A delay between a reading and its `OK`.
        ok_late: Duration,
        /// A delay between a status and its `OK`.
        status_ok_late: Duration,
        /// Every answer comes this much after its command.
        answer_after: Duration,
        /// Reads never run out of this.
        endless: Option<u8>,
    }

    impl Fake {
        fn new() -> (Fake, Seen) {
            let seen = Arc::new(Mutex::new(Vec::new()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                answers: None,
                replies: VecDeque::new(),
                fail_writes_after: None,
                fail_reads: false,
                ok_late: Duration::ZERO,
                status_ok_late: Duration::ZERO,
                answer_after: Duration::ZERO,
                endless: None,
            };
            (fake, seen)
        }

        fn answer(command: &[u8]) -> Vec<u8> {
            match command {
                b"MODEL" => b"LPS-301\r\n\r\nOK\r\n\r\n".to_vec(),
                b"VERSION" => b"Ver-1.17\r\n\r\nOK\r\n\r\n".to_vec(),
                b"STATUS" => b"576\r\n\r\nOK\r\n\r\n".to_vec(),
                b"VOUT1" => b"5.000\r\n\r\nOK\r\n\r\n".to_vec(),
                b"IOUT1" => b"0.2500\r\n\r\nOK\r\n\r\n".to_vec(),
                _ => b"OK\r\n\r\n".to_vec(),
            }
        }

        fn arrived(&self, now: Instant) -> usize {
            self.input.iter().take_while(|(at, _)| *at <= now).count()
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
            if self.fail_reads {
                return Err(Error::device("hung up"));
            }
            if let Some(byte) = self.endless {
                buf.fill(byte);
                return Ok(buf.len());
            }
            let now = Instant::now();
            let n = self.arrived(now).min(buf.len());
            if n == 0 {
                let until = now + timeout;
                let next = self.input.front().map_or(until, |(at, _)| *at);
                std::thread::sleep(next.min(until).saturating_duration_since(now));
                return Ok(0);
            }
            for b in &mut buf[..n] {
                *b = self.input.pop_front().expect("counted").1;
            }
            Ok(n)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let mut seen = self.seen.lock().unwrap();
            if self.fail_writes_after.is_some_and(|n| seen.len() >= n) {
                return Err(Error::device("Broken pipe"));
            }
            let now = Instant::now();
            seen.push((now, data.to_vec()));
            let command = data.strip_suffix(b"\r\n").expect("a line ends CR LF");
            if self.answers == Some(0) {
                return Ok(());
            }
            if let Some(n) = &mut self.answers {
                *n -= 1;
            }
            let reply = self.replies.pop_front().unwrap_or_else(|| Fake::answer(command));
            let at = self.input.back().map_or(now, |(last, _)| now.max(*last));
            let at = at + self.answer_after;
            let ok_late = if command == b"STATUS" { self.status_ok_late } else { self.ok_late };
            if !ok_late.is_zero() && reply.ends_with(b"OK\r\n\r\n") {
                let split = reply.len() - 6;
                self.input.extend(reply[..split].iter().map(|b| (at, *b)));
                let later = at + ok_late;
                self.input.extend(reply[split..].iter().map(|b| (later, *b)));
            } else {
                self.input.extend(reply.iter().map(|b| (at, *b)));
            }
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            let n = self.arrived(Instant::now());
            self.input.drain(..n);
            Ok(())
        }
    }

    fn device(fake: Fake) -> MotechLpsDevice {
        let mut d = MotechLpsDevice::from_port(Box::new(fake));
        d.open().unwrap();
        d
    }

    fn params() -> Params {
        Params::parse(SERIALCOMM).unwrap()
    }

    fn start(d: &mut MotechLpsDevice) -> Received {
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        rx
    }

    /// The values a capture sends up to its End, which must come within
    /// [`CAPTURE_BOUND`]: a stop or a limit that does not bite fails.
    fn readings(rx: &Received) -> Vec<f32> {
        let deadline = Instant::now() + CAPTURE_BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(left).expect("no End in time").1 {
                Packet::Analog(a) => out.push(a.samples[0]),
                Packet::End => return out,
                _ => {}
            }
        }
    }

    fn writes(seen: &Seen) -> Vec<String> {
        seen.lock()
            .unwrap()
            .iter()
            .map(|(_, w)| String::from_utf8_lossy(w).trim_end().to_owned())
            .collect()
    }

    /// Until `n` writes have happened, within a bound.
    fn wait_for_writes(seen: &Seen, n: usize) {
        let deadline = Instant::now() + CAPTURE_BOUND;
        while seen.lock().unwrap().len() < n {
            assert!(Instant::now() < deadline, "{} writes, not {n}", seen.lock().unwrap().len());
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    const NOTHING: [f32; 0] = [];

    #[test]
    fn a_scan_asks_three_things_sets_nothing_and_the_port_goes() {
        let (fake, seen) = Fake::new();
        let (link, version) = probe(Box::new(fake), params()).unwrap().expect("a device");
        assert_eq!(writes(&seen), ["MODEL", "VERSION", "STATUS"]);
        assert_eq!(version, "1.17");
        assert!(link.port.is_none());
        assert!(link.output_enabled, "576 has bit 6");
        assert_eq!(link.voltage, 0.0, "asked in a capture only");
    }

    #[test]
    fn a_scan_takes_any_lps_and_nothing_else() {
        for model in [&b"LPS-\r\nOK\r\n"[..], b"LPS-305\r\nOK\r\n", b"LPS-301 \r\nOK\r\n"] {
            let (mut fake, _) = Fake::new();
            fake.replies = VecDeque::from([model.to_vec()]);
            assert!(probe(Box::new(fake), params()).unwrap().is_some());
        }
        let (mut fake, seen) = Fake::new();
        fake.replies = VecDeque::from([b"HP-1\r\nOK\r\n".to_vec()]);
        assert!(probe(Box::new(fake), params()).unwrap().is_none());
        assert_eq!(writes(&seen), ["MODEL"], "and asks no more");
    }

    #[test]
    fn a_failed_version_is_no_error_and_a_strange_one_is_no_device() {
        let (mut fake, seen) = Fake::new();
        fake.replies = VecDeque::from([Fake::answer(b"MODEL"), b"ERROR\r\nOK\r\n".to_vec()]);
        let (_, version) = probe(Box::new(fake), params()).unwrap().expect("a device");
        assert_eq!(version, "");
        assert_eq!(writes(&seen).len(), 3);
        let (mut fake, seen) = Fake::new();
        fake.replies = VecDeque::from([Fake::answer(b"MODEL"), b"V1\r\nOK\r\n".to_vec()]);
        assert!(probe(Box::new(fake), params()).unwrap().is_none());
        assert_eq!(writes(&seen).len(), 2);
        // Only its end is stripped once the head has been checked, and VT is no
        // space to glib.
        let (mut fake, _) = Fake::new();
        fake.replies = VecDeque::from([Fake::answer(b"MODEL"), b"Ver-1.0 \t\r\nOK\r\n".to_vec()]);
        let (_, version) = probe(Box::new(fake), params()).unwrap().expect("a device");
        assert_eq!(version, "1.0");
        let (mut fake, _) = Fake::new();
        fake.replies = VecDeque::from([Fake::answer(b"MODEL"), b"Ver-1.0\x0b\r\nOK\r\n".to_vec()]);
        let (_, version) = probe(Box::new(fake), params()).unwrap().expect("a device");
        assert_eq!(version, "1.0\x0b");
    }

    #[test]
    fn a_silent_scan_ends_where_c_blocks_for_ever() {
        let (mut fake, seen) = Fake::new();
        fake.answers = Some(0);
        assert!(probe(Box::new(fake), params()).is_err());
        assert_eq!(writes(&seen), ["MODEL"]);
    }

    #[test]
    fn a_bad_status_is_no_device() {
        for status in [&b"abc\r\nOK\r\n"[..], b"ERROR\r\nOK\r\n", b"OK\r\n"] {
            let (mut fake, _) = Fake::new();
            fake.replies =
                VecDeque::from([Fake::answer(b"MODEL"), Fake::answer(b"VERSION"), status.to_vec()]);
            assert!(probe(Box::new(fake), params()).is_err(), "{}", status.escape_ascii());
        }
    }

    #[test]
    fn a_capture_asks_in_turn_and_stops_at_its_limit_with_one_more_request() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25, 5.0, 0.25]);
        assert_eq!(
            writes(&seen),
            ["VOUT1", "IOUT1", "STATUS", "VOUT1", "IOUT1", "STATUS", "VOUT1"]
        );
    }

    #[test]
    fn with_the_ok_not_there_yet_a_limit_sends_no_more() {
        // The pass that read the status ends the capture long before its OK.
        let (mut fake, seen) = Fake::new();
        fake.status_ok_late = Duration::from_secs(1);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25]);
        assert_eq!(writes(&seen), ["VOUT1", "IOUT1", "STATUS"]);
    }

    #[test]
    fn the_first_request_goes_50_ms_after_the_start() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        readings(&rx);
        let first = seen.lock().unwrap()[0].0 - began;
        assert!(first >= Duration::from_millis(50), "{first:?}");
    }

    #[test]
    fn a_time_limit_under_the_first_pass_still_sends_its_request() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), NOTHING);
        assert_eq!(writes(&seen), ["VOUT1"]);
    }

    #[test]
    fn a_reading_that_is_no_number_means_no_sample_where_c_sends_0() {
        let (mut fake, seen) = Fake::new();
        fake.replies = VecDeque::from([b"ERROR\r\nOK\r\n".to_vec()]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25]);
        assert_eq!(writes(&seen)[..6], ["VOUT1", "IOUT1", "STATUS", "VOUT1", "IOUT1", "STATUS"]);
    }

    #[test]
    fn a_bad_status_still_sends_the_readings_and_keeps_the_output() {
        let (mut fake, _) = Fake::new();
        fake.replies = VecDeque::from([
            Fake::answer(b"VOUT1"),
            Fake::answer(b"IOUT1"),
            b"abc\r\nOK\r\n".to_vec(),
        ]);
        let mut d = device(fake);
        d.link().output_enabled = true;
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25]);
        // C takes it for a status of 0, with the output off.
        assert_eq!(d.get(&Key::Enabled, Some("CG1")).unwrap(), Value::Bool(true));
    }

    #[test]
    fn a_bare_ok_ends_a_request_where_c_waits_for_another() {
        let (mut fake, seen) = Fake::new();
        fake.replies = VecDeque::from([b"OK\r\n".to_vec()]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25]);
        assert_eq!(writes(&seen)[0..2], ["VOUT1", "IOUT1"]);
        let seen = seen.lock().unwrap();
        let gap = seen[1].0 - seen[0].0;
        assert!(gap < Duration::from_millis(250), "it waited for a second OK: {gap:?}");
    }

    #[test]
    fn a_line_past_47_bytes_is_one_bad_line_and_47_is_a_reading() {
        let line = |n: usize| {
            let mut v = vec![b'0'; n - 1];
            v.push(b'1');
            v.extend_from_slice(b"\r\nOK\r\n");
            v
        };
        let (mut fake, _) = Fake::new();
        fake.replies = VecDeque::from([line(47)]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [1.0, 0.25]);

        let (mut fake, seen) = Fake::new();
        fake.replies = VecDeque::from([line(48)]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25]);
        assert_eq!(writes(&seen).len(), 7, "a whole round lost, and one more with the limit");
    }

    #[test]
    fn a_line_ahead_of_ok_leaves_the_next_request_until_the_ok() {
        // The device's manual: a command before OK is rejected. C sends it.
        let (mut fake, seen) = Fake::new();
        fake.ok_late = Duration::from_millis(60);
        fake.replies = VecDeque::from([b"1.000\r\njunk\r\nOK\r\n\r\n".to_vec()]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx)[0], 1.0);
        let seen = seen.lock().unwrap();
        assert!(seen[1].0 - seen[0].0 >= Duration::from_millis(55), "{:?}", seen[1].0 - seen[0].0);
    }

    #[test]
    fn a_request_expires_after_250_ms_and_the_next_waits_for_a_silent_line() {
        // C compares two clocks here and never expires one.
        let (mut fake, seen) = Fake::new();
        fake.replies = VecDeque::from([Vec::new()]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25], "that round has no sample");
        let seen = seen.lock().unwrap();
        let gap = seen[1].0 - seen[0].0;
        // 250 ms, then 400 ms of silence: the literals.
        assert!(gap >= Duration::from_millis(650), "{gap:?}");
        assert!(gap < Duration::from_secs(2), "{gap:?}");
        assert!(seen.len() >= 6, "a round lost");
    }

    #[test]
    fn a_silent_device_ends_a_sample_limited_capture_within_the_bound() {
        let (mut fake, seen) = Fake::new();
        fake.answers = Some(0);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), NOTHING);
        let took = began.elapsed();
        assert!(took >= Duration::from_secs(2), "{took:?}");
        assert!(took < Duration::from_secs(6), "{took:?}");
        assert!(writes(&seen).len() >= 2, "it kept asking");
    }

    #[test]
    fn a_time_limit_outlasts_the_bound_on_a_silent_device() {
        let (mut fake, _) = Fake::new();
        fake.answers = Some(0);
        let mut d = device(fake);
        d.set(&Key::LimitMsec, &Value::U64(2500), None).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), NOTHING);
        assert!(began.elapsed() >= Duration::from_millis(2500), "{:?}", began.elapsed());
        assert!(began.elapsed() < Duration::from_secs(10), "{:?}", began.elapsed());
    }

    #[test]
    fn with_no_limit_a_silent_device_is_waited_for_until_a_stop() {
        let (mut fake, _) = Fake::new();
        fake.answers = Some(0);
        let mut d = device(fake);
        let rx = start(&mut d);
        std::thread::sleep(Duration::from_millis(2500));
        assert!(d.capturing.load(Ordering::SeqCst), "the bound ended it");
        d.stop().unwrap();
        assert_eq!(readings(&rx), NOTHING);
    }

    #[test]
    fn a_stop_ends_a_capture_that_runs_and_asks_no_more() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        let rx = start(&mut d);
        wait_for_writes(&seen, 6);
        d.stop().unwrap();
        readings(&rx);
        let n = seen.lock().unwrap().len();
        std::thread::sleep(POLL_INTERVAL * 3);
        assert_eq!(seen.lock().unwrap().len(), n, "a request after the stop");
    }

    #[test]
    fn a_close_ends_a_capture_blocked_on_a_full_queue() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for_writes(&seen, 3);
        // Nobody reads: the header sits in the queue, and the volts wait for room.
        // On a thread of its own, so a close that hangs fails rather than hangs.
        let (done, closed) = sync_channel(1);
        std::thread::spawn(move || {
            d.close().unwrap();
            done.send(()).unwrap();
        });
        closed.recv_timeout(Duration::from_secs(5)).expect("the close hangs");
        drop(rx);
    }

    #[test]
    fn a_failed_write_or_read_ends_the_capture() {
        let (mut fake, _) = Fake::new();
        fake.fail_writes_after = Some(1);
        let mut d = device(fake);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), NOTHING);

        let (mut fake, _) = Fake::new();
        fake.fail_reads = true;
        let mut d = device(fake);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), NOTHING);
    }

    #[test]
    fn a_disabled_channel_sends_nothing_but_the_limit_still_counts() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.enable_channel(0, false).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), NOTHING);
        assert_eq!(writes(&seen).len(), 4);
    }

    #[test]
    fn the_readings_are_kept_and_packets_are_volts_then_amperes() {
        let (fake, _) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        let mut analog = Vec::new();
        loop {
            match rx.recv_timeout(CAPTURE_BOUND).expect("no End in time").1 {
                Packet::Analog(a) => analog.push(a),
                Packet::End => break,
                _ => {}
            }
        }
        assert_eq!((analog[0].mq, analog[0].unit, analog[0].digits), (Mq::Voltage, Unit::Volt, 3));
        assert_eq!(analog[0].flags, MqFlags::DC);
        assert_eq!(
            (analog[1].mq, analog[1].unit, analog[1].digits),
            (Mq::Current, Unit::Ampere, 4)
        );
        assert_eq!(analog[1].flags, MqFlags::empty());
        assert_eq!(d.get(&Key::Voltage, Some("CG1")).unwrap(), Value::F64(5.0));
        assert_eq!(d.get(&Key::Current, Some("CG1")).unwrap(), Value::F64(0.25));
    }

    #[test]
    fn a_setting_during_a_capture_is_refused_and_after_it_goes_out() {
        // C never clears acq_running, and refuses it after the capture too.
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        let rx = start(&mut d);
        wait_for_writes(&seen, 1);
        let n = seen.lock().unwrap().iter().filter(|(_, w)| w.starts_with(b"VSET")).count();
        assert!(d.set(&Key::VoltageTarget, &Value::F64(1.0), Some("CG1")).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::U64(1), None).is_err());
        assert_eq!(n, 0);
        d.stop().unwrap();
        readings(&rx);
        d.set(&Key::VoltageTarget, &Value::F64(1.0), Some("CG1")).unwrap();
        assert_eq!(writes(&seen).last().unwrap(), "VSET1 1.000");
    }

    /// A UART at 2400 baud: each answer's bytes 4.2 ms apart. It holds answers
    /// back as told, and notes a command that came before the last `OK` had gone,
    /// which the device's manual has rejected.
    struct Uart {
        seen: Seen,
        input: VecDeque<(Instant, u8)>,
        set_answer: Vec<u8>,
        /// Which `VOUT1`, from 1, is late, and by how much.
        late_volts: Option<(usize, Duration)>,
        /// Which `STATUS`, from 1, is late, and by how much.
        late_status: Option<(usize, Duration)>,
        volts: usize,
        status: usize,
        early: Arc<Mutex<usize>>,
        /// A device with output queued: what it answered meanwhile is all there
        /// as soon as the late answer has gone, not one byte time after another.
        burst: bool,
    }

    impl Port for Uart {
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
            let now = Instant::now();
            let n = self.input.iter().take_while(|(at, _)| *at <= now).count().min(buf.len());
            if n == 0 {
                std::thread::sleep(timeout.min(Duration::from_millis(1)));
                return Ok(0);
            }
            for b in &mut buf[..n] {
                *b = self.input.pop_front().expect("counted").1;
            }
            Ok(n)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let now = Instant::now();
            self.seen.lock().unwrap().push((now, data.to_vec()));
            let command = data.strip_suffix(b"\r\n").expect("a line ends CR LF");
            // The two line ends after OK may still be going; OK itself must not.
            if self.input.len() > 4 {
                *self.early.lock().unwrap() += 1;
            }
            let mut reply = Fake::answer(command);
            let mut late = Duration::ZERO;
            match command {
                b"VOUT1" => {
                    self.volts += 1;
                    if let Some((n, by)) = self.late_volts
                        && n == self.volts
                    {
                        late = by;
                    }
                }
                b"STATUS" => {
                    self.status += 1;
                    if let Some((n, by)) = self.late_status
                        && n == self.status
                    {
                        late = by;
                    }
                }
                c if c.starts_with(b"VSET") => reply.clone_from(&self.set_answer),
                _ => {}
            }
            let mut at = now + late;
            if !self.burst {
                at = self.input.back().map_or(now, |(last, _)| now.max(*last)) + late;
            }
            for b in reply {
                at += Duration::from_micros(4200);
                self.input.push_back((at, b));
            }
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            let now = Instant::now();
            let n = self.input.iter().take_while(|(at, _)| *at <= now).count();
            self.input.drain(..n);
            Ok(())
        }
    }

    struct UartDevice {
        device: MotechLpsDevice,
        seen: Seen,
        early: Arc<Mutex<usize>>,
    }

    fn uart(
        set_answer: &[u8],
        late_volts: Option<(usize, Duration)>,
        late_status: Option<(usize, Duration)>,
    ) -> UartDevice {
        let seen: Seen = Arc::new(Mutex::new(Vec::new()));
        let early = Arc::new(Mutex::new(0));
        let port = Uart {
            seen: Arc::clone(&seen),
            input: VecDeque::new(),
            set_answer: set_answer.to_vec(),
            late_volts,
            late_status,
            volts: 0,
            status: 0,
            early: Arc::clone(&early),
            burst: false,
        };
        let mut device = MotechLpsDevice::from_port(Box::new(port));
        device.open().unwrap();
        UartDevice { device, seen, early }
    }

    /// Whatever comes late, each sample is what the supply said it was.
    fn assert_all_true(got: &[f32], pairs: usize) {
        assert_eq!(got, [5.0, 0.25].repeat(pairs), "readings out of step");
    }

    #[test]
    fn a_second_capture_right_after_a_first_at_2400_is_not_out_of_step() {
        // The first's last request is still being answered.
        let mut u = uart(b"OK\r\n\r\n", None, None);
        u.device.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        for _ in 0..2 {
            let rx = start(&mut u.device);
            assert_all_true(&readings(&rx), 3);
        }
        assert_eq!(*u.early.lock().unwrap(), 0, "{:?}", writes(&u.seen));
    }

    #[test]
    fn a_second_capture_waits_out_a_late_answer_of_the_first() {
        // A stop with the second VOUT1 unanswered, its answer due 300 ms on: longer
        // than a first request's 50 ms, within the wait.
        let late = Some((2, Duration::from_millis(300)));
        let mut u = uart(b"OK\r\n\r\n", late, None);
        let rx = start(&mut u.device);
        wait_for_writes(&u.seen, 4);
        u.device.stop().unwrap();
        assert_all_true(&readings(&rx), 1);
        u.device.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let rx = start(&mut u.device);
        assert_all_true(&readings(&rx), 2);
        assert_eq!(*u.early.lock().unwrap(), 0, "{:?}", writes(&u.seen));
    }

    #[test]
    fn a_failed_setting_says_what_c_says() {
        let (mut fake, _) = Fake::new();
        fake.replies = VecDeque::from([b"ERROR\r\nOK\r\n".to_vec()]);
        let mut d = device(fake);
        let e = d.set(&Key::Enabled, &Value::Bool(true), Some("CG1")).unwrap_err();
        assert!(e.to_string().contains("generic/unspecified error"), "{e}");
    }

    #[test]
    fn a_reply_late_past_its_request_is_dropped_and_no_reading_shifts() {
        for late in [400, 500] {
            let late = Some((2, Duration::from_millis(late)));
            let mut u = uart(b"OK\r\n\r\n", late, None);
            u.device.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
            let rx = start(&mut u.device);
            assert_all_true(&readings(&rx), 4);
            assert_eq!(*u.early.lock().unwrap(), 0, "{:?}", writes(&u.seen));
        }
    }

    #[test]
    fn a_reply_later_than_the_wait_is_found_by_its_status_and_no_reading_shifts() {
        // 250 + 400 ms have gone by: it reads as the answer to IOUT1, the real one
        // as a status that is no number, and 576 V never goes out.
        let late = Some((2, Duration::from_millis(900)));
        let mut u = uart(b"OK\r\n\r\n", late, None);
        u.device.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        let rx = start(&mut u.device);
        assert_all_true(&readings(&rx), 4);
    }

    #[test]
    fn a_status_read_in_part_when_the_wait_starts_is_not_the_next_reading() {
        // Seen on a pty: the late answer, what was answered meanwhile in a burst,
        // and the start of the status's "576" all in one pass that ends on the
        // wait. Kept, that "576" was read as volts.
        let late = Some((2, Duration::from_millis(900)));
        let mut hit = false;
        for _ in 0..6 {
            let mut u = uart(b"OK\r\n\r\n", late, None);
            let seen = Arc::clone(&u.seen);
            let port = Uart {
                seen,
                input: VecDeque::new(),
                set_answer: Vec::new(),
                late_volts: late,
                late_status: None,
                volts: 0,
                status: 0,
                early: Arc::clone(&u.early),
                burst: true,
            };
            u.device.link().port = Some(Box::new(port));
            u.device.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
            let rx = start(&mut u.device);
            assert_all_true(&readings(&rx), 3);
            hit |= writes(&u.seen).len() > 9;
        }
        assert!(hit, "no round lost: the late answer did not bite");
    }

    #[test]
    fn a_status_later_than_the_wait_shifts_no_reading() {
        // The late 576 reads as volts, then 5.000 as amperes, then 0.2500 as a
        // status: this is the frame that gave 576 V.
        let late = Some((2, Duration::from_millis(900)));
        let mut u = uart(b"OK\r\n\r\n", None, late);
        u.device.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        let rx = start(&mut u.device);
        assert_all_true(&readings(&rx), 4);
    }

    #[test]
    fn a_status_that_is_no_number_keeps_its_sample_only_with_no_expiry() {
        let round = |status: &[u8]| {
            let (mut fake, _) = Fake::new();
            fake.replies =
                VecDeque::from([Fake::answer(b"VOUT1"), Fake::answer(b"IOUT1"), status.to_vec()]);
            fake
        };
        let mut d = device(round(b"abc\r\nOK\r\n"));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25]);

        // After an expiry: a first VOUT1 lost, then a good round but its status.
        let (mut fake, seen) = Fake::new();
        fake.replies = VecDeque::from([
            Vec::new(),
            Fake::answer(b"IOUT1"),
            b"abc\r\nOK\r\n".to_vec(),
            b"7.000\r\nOK\r\n".to_vec(),
            Fake::answer(b"IOUT1"),
            b"0.2500\r\nOK\r\n".to_vec(),
        ]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25], "not the 7 V of the suspect round");
        // And it waited for the line before the next round.
        let seen = seen.lock().unwrap();
        let gap = seen[6].0 - seen[5].0;
        assert!(gap >= Duration::from_millis(400), "{gap:?}");
    }

    #[test]
    fn a_round_begun_out_of_step_makes_no_sample_though_its_status_is_good() {
        // A status lost, its 576 read as volts, 5.000 as amperes, the answer that
        // should have followed dropped between two requests, and then a good
        // status: seen under load as 576 V.
        let (mut fake, _) = Fake::new();
        fake.replies = VecDeque::from([
            Fake::answer(b"VOUT1"),
            Fake::answer(b"IOUT1"),
            Fake::answer(b"STATUS"),
            Fake::answer(b"VOUT1"),
            Fake::answer(b"IOUT1"),
            Vec::new(),
            b"576\r\nOK\r\n".to_vec(),
            b"5.000\r\nOK\r\n".to_vec(),
            Fake::answer(b"STATUS"),
        ]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        assert_all_true(&readings(&rx), 2);
    }

    #[test]
    fn an_expiry_waiting_for_ok_costs_the_round_its_sample() {
        let (mut fake, _) = Fake::new();
        fake.replies = VecDeque::from([b"9.000\r\n".to_vec()]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25], "not the 9 V of a round with no OK");
    }

    #[test]
    fn a_good_status_clears_the_suspicion() {
        // After it a garbled status is the device's word again, as in C.
        let (mut fake, _) = Fake::new();
        fake.replies = VecDeque::from([
            Vec::new(),
            Fake::answer(b"IOUT1"),
            Fake::answer(b"STATUS"),
            b"7.000\r\nOK\r\n".to_vec(),
            Fake::answer(b"IOUT1"),
            b"abc\r\nOK\r\n".to_vec(),
        ]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [7.0, 0.25]);
    }

    #[test]
    fn a_setting_right_after_a_capture_gets_its_own_answer() {
        // The capture's last OK is on its way: C refuses the setting altogether.
        let mut u = uart(b"ERROR\r\n\r\nOK\r\n\r\n", None, None);
        u.device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut u.device);
        assert_all_true(&readings(&rx), 1);
        assert!(u.device.set(&Key::VoltageTarget, &Value::F64(1.0), Some("CG1")).is_err());
        assert_eq!(u.device.get(&Key::VoltageTarget, Some("CG1")).unwrap(), Value::F64(0.0));
        assert_eq!(*u.early.lock().unwrap(), 0, "{:?}", writes(&u.seen));
        // And it is one wait, not one for each.
        assert!(u.device.link().open_request.is_none());
        let mut u = uart(b"OK\r\n\r\n", None, None);
        u.device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut u.device);
        readings(&rx);
        u.device.set(&Key::VoltageTarget, &Value::F64(1.0), Some("CG1")).unwrap();
    }

    #[test]
    fn a_capture_after_a_stop_in_mid_answer_waits_too() {
        let mut u = uart(b"OK\r\n\r\n", None, None);
        let rx = start(&mut u.device);
        wait_for_writes(&u.seen, 1);
        u.device.stop().unwrap();
        readings(&rx);
        assert!(u.device.link().open_request.is_some());
        u.device.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let rx = start(&mut u.device);
        assert_all_true(&readings(&rx), 2);
        assert_eq!(*u.early.lock().unwrap(), 0, "{:?}", writes(&u.seen));
    }

    #[test]
    fn what_lay_in_the_input_before_a_setting_is_not_its_answer() {
        // It comes after the open's purge, and with no request left open.
        let (mut fake, _) = Fake::new();
        let at = Instant::now() + POLL_INTERVAL;
        fake.input.extend(b"x\r\nOK\r\n".iter().map(|b| (at, *b)));
        let mut d = device(fake);
        std::thread::sleep(POLL_INTERVAL * 2);
        d.set(&Key::VoltageTarget, &Value::F64(1.0), Some("CG1")).unwrap();
    }

    #[test]
    fn a_healthy_capture_goes_on_past_the_bound() {
        let (mut fake, _) = Fake::new();
        fake.answer_after = Duration::from_millis(60);
        let mut d = device(fake);
        d.silence_bound = Duration::from_millis(800);
        d.set(&Key::LimitSamples, &Value::U64(10), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        assert_all_true(&readings(&rx), 10);
        assert!(began.elapsed() > d.silence_bound, "{:?} proves nothing", began.elapsed());
    }

    #[test]
    fn a_status_in_a_capture_keeps_the_output_state() {
        let (mut fake, _) = Fake::new();
        fake.replies = VecDeque::from([
            Fake::answer(b"VOUT1"),
            Fake::answer(b"IOUT1"),
            b"512\r\nOK\r\n".to_vec(),
        ]);
        let mut d = device(fake);
        d.link().output_enabled = true;
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        readings(&rx);
        assert_eq!(d.get(&Key::Enabled, Some("CG1")).unwrap(), Value::Bool(false));
    }

    #[test]
    fn a_line_cut_by_an_expiry_does_not_start_the_next() {
        // With the "12" kept the next reading would be 125 V.
        let (mut fake, _) = Fake::new();
        fake.replies =
            VecDeque::from([Fake::answer(b"VOUT1"), Fake::answer(b"IOUT1"), b"12".to_vec()]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [5.0, 0.25]);
    }

    #[test]
    fn a_failed_last_request_says_nothing_and_an_earlier_one_ends_it() {
        let run = |fail_after| {
            let (mut fake, _) = Fake::new();
            fake.fail_writes_after = Some(fail_after);
            let d = device(fake);
            let capture = Capture {
                link: Arc::clone(&d.link),
                limit_samples: 1,
                limit_msec: 0,
                enabled: true,
                silence_bound: SILENCE_BOUND,
            };
            let (tx, _rx) = sync_channel(16);
            capture.passes(&PacketSink::new(tx, 0), &AtomicBool::new(false))
        };
        run(3).unwrap();
        assert!(run(1).is_err());
    }

    #[test]
    fn a_device_that_never_stops_talking_cannot_hold_off_a_stop() {
        let (mut fake, _) = Fake::new();
        fake.endless = Some(b'x');
        let mut d = device(fake);
        let rx = start(&mut d);
        std::thread::sleep(POLL_INTERVAL);
        d.stop().unwrap();
        assert_eq!(readings(&rx), NOTHING);
    }

    #[test]
    fn a_stop_ends_a_capture_blocked_on_its_header() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        let (tx, rx) = sync_channel(1);
        tx.send((0, Packet::End)).unwrap();
        d.start(PacketSink::new(tx, 0)).unwrap();
        std::thread::sleep(POLL_INTERVAL * 2);
        assert_eq!(writes(&seen).len(), 0, "no request ahead of its header");
        let (done, closed) = sync_channel(1);
        std::thread::spawn(move || {
            d.close().unwrap();
            done.send(()).unwrap();
        });
        closed.recv_timeout(Duration::from_secs(5)).expect("the close hangs");
        drop(rx);
    }

    #[test]
    fn a_second_capture_starts_afresh() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        for _ in 0..2 {
            let rx = start(&mut d);
            assert_eq!(readings(&rx), [5.0, 0.25]);
        }
        assert_eq!(writes(&seen)[4], "VOUT1", "the first request of the round again");
    }

    #[test]
    fn enabled_goes_out_only_when_it_changes() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::Enabled, &Value::Bool(false), Some("CG1")).unwrap();
        assert_eq!(writes(&seen).len(), 0);
        d.set(&Key::Enabled, &Value::Bool(true), Some("CG1")).unwrap();
        d.set(&Key::Enabled, &Value::Bool(true), Some("CG1")).unwrap();
        d.set(&Key::Enabled, &Value::Bool(false), Some("CG1")).unwrap();
        assert_eq!(writes(&seen), ["OUT1", "OUT0"]);
        assert_eq!(d.get(&Key::Enabled, Some("CG1")).unwrap(), Value::Bool(false));
    }

    #[test]
    fn a_refused_setting_leaves_what_was_known() {
        // C stores it before it sends: a failed OUT1 reads as on, and the next
        // enabled=on sends nothing.
        let (mut fake, seen) = Fake::new();
        fake.replies = VecDeque::from([
            b"ERROR\r\nOK\r\n".to_vec(),
            b"ERROR\r\nOK\r\n".to_vec(),
            b"x\r\nOK\r\n".to_vec(),
        ]);
        let mut d = device(fake);
        assert!(d.set(&Key::Enabled, &Value::Bool(true), Some("CG1")).is_err());
        assert_eq!(d.get(&Key::Enabled, Some("CG1")).unwrap(), Value::Bool(false));
        assert!(d.set(&Key::VoltageTarget, &Value::F64(1.0), Some("CG1")).is_err());
        assert_eq!(d.get(&Key::VoltageTarget, Some("CG1")).unwrap(), Value::F64(0.0));
        assert!(d.set(&Key::CurrentLimit, &Value::F64(1.0), Some("CG1")).is_err());
        assert_eq!(d.get(&Key::CurrentLimit, Some("CG1")).unwrap(), Value::F64(0.0));
        d.set(&Key::Enabled, &Value::Bool(true), Some("CG1")).unwrap();
        assert_eq!(writes(&seen).last().unwrap(), "OUT1");
    }

    #[test]
    fn settings_go_out_and_read_back_as_a_c_float() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::VoltageTarget, &Value::F64(3.3), Some("CG1")).unwrap();
        d.set(&Key::CurrentLimit, &Value::F64(0.1), Some("CG1")).unwrap();
        assert_eq!(writes(&seen), ["VSET1 3.300", "ISET1 0.1000"]);
        assert_eq!(d.get(&Key::VoltageTarget, Some("CG1")).unwrap(), Value::F64(f64::from(3.3f32)));
        assert_eq!(d.get(&Key::CurrentLimit, Some("CG1")).unwrap(), Value::F64(f64::from(0.1f32)));
    }

    #[test]
    fn the_range_is_c_less_a_nan_and_a_minus_zero() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        for (key, ok) in [
            (Key::VoltageTarget, 32.0),
            (Key::VoltageTarget, 0.0),
            (Key::CurrentLimit, 2.0),
            // Under the 5 mA listed, as C takes it.
            (Key::CurrentLimit, 0.001),
            (Key::VoltageTarget, -0.0),
            // Above in double precision, the maximum in single.
            (Key::VoltageTarget, 32.000_000_1),
            // No longer negative in single precision, and 0 for both.
            (Key::VoltageTarget, -1e-50),
            (Key::CurrentLimit, -1e-50),
        ] {
            d.set(&key, &Value::F64(ok), Some("CG1")).unwrap();
        }
        let sent = writes(&seen);
        assert_eq!(sent[4], "VSET1 0.000", "-0.0 goes out as 0");
        assert_eq!(sent[6], "VSET1 0.000", "and what a float makes of it");
        assert_eq!(sent[7], "ISET1 0.0000");
        assert_eq!(d.get(&Key::CurrentLimit, Some("CG1")).unwrap().as_f64().unwrap().to_bits(), 0);
        for (key, bad) in [
            (Key::VoltageTarget, 32.01),
            (Key::VoltageTarget, -0.5),
            (Key::VoltageTarget, f64::NAN),
            (Key::VoltageTarget, f64::INFINITY),
            (Key::CurrentLimit, 2.01),
            (Key::CurrentLimit, -0.5),
            (Key::CurrentLimit, f64::NAN),
        ] {
            assert!(d.set(&key, &Value::F64(bad), Some("CG1")).is_err(), "{bad}");
        }
        assert_eq!(writes(&seen).len(), sent.len(), "nothing sent for those");
    }

    #[test]
    fn channel_config_reads_and_lists_the_first_takes_all_and_sends_nothing() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        for mode in CHANNEL_MODES {
            d.set(&Key::ChannelConfig, &Value::from(mode), None).unwrap();
            assert_eq!(d.get(&Key::ChannelConfig, None).unwrap(), Value::from("Independent"));
        }
        assert!(d.set(&Key::ChannelConfig, &Value::from("Track3"), None).is_err());
        assert!(matches!(
            d.list(&Key::ChannelConfig, None).unwrap(),
            ValueList::Str(l) if l == ["Independent"]
        ));
        assert_eq!(writes(&seen).len(), 0);
    }

    #[test]
    fn the_options_are_the_cs_and_the_group_has_its_own() {
        let (fake, _) = Fake::new();
        let d = device(fake);
        let keys = |g| -> Vec<Key> { d.options(g).unwrap().into_iter().map(|o| o.key).collect() };
        assert_eq!(keys(None).len(), 4);
        assert_eq!(keys(Some("CG1")).len(), 5);
        assert!(d.options(Some("CG2")).is_err());
        assert!(d.get(&Key::Voltage, None).is_err());
        assert!(d.get(&Key::ChannelConfig, Some("CG1")).is_err());
        assert!(matches!(
            d.list(&Key::CurrentLimit, Some("CG1")).unwrap(),
            ValueList::RangeStepF64 { min, max, step }
                if (min - 0.005).abs() < 1e-9 && (max - 2.0).abs() < 1e-9 && (step - 0.001).abs() < 1e-9
        ));
        assert!(matches!(
            d.list(&Key::VoltageTarget, Some("CG1")).unwrap(),
            ValueList::RangeStepF64 { max, step, .. }
                if (max - 32.0).abs() < 1e-9 && (step - 0.01).abs() < 1e-9
        ));
    }
}
