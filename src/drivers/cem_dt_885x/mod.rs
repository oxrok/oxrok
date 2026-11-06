// Counterpart in sigrok:
//   libsigrok/src/hardware/cem-dt-885x/api.c,
//   libsigrok/src/hardware/cem-dt-885x/protocol.c,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Martin Ling,
//   Soeren Apel, Daniel Elstner, Aurelien Jacobs, Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The CEM DT-885x sound level meters, on a serial port at 9600 baud.
//!
//! A meter talks without being asked. A scan opens the port, looks for an `a5` and
//! closes it; an open opens it again and says nothing. A capture reads levels until
//! a limit or a stop; with `data_source=Memory` it asks for the stored log first,
//! once per byte until the log's start has come, and the log's end ends it, as a
//! client could not tell stored levels from live ones.
//!
//! A get of the meter's state waits for the token that says it, and a set waits and
//! then sends its toggle until that token says what was asked: no command has an
//! answer. C keeps what a capture saw, and what a wait saw only for the range and
//! hold: so what was never seen is waited for every time.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - A wait gets its bytes with a read that waits: see [`protocol::Link`]. On stock
//!   libsigrok every such get and set fails.
//! - Those waits and toggles have bounds, where C has none.
//! - A capture a meter's silence can hold for ever, one with a sample limit or a log,
//!   ends after [`SILENCE_BOUND`] with no byte.
//! - The limit is held exactly, also on a log's packets of 50, where C overshoots.
//! - What a set changed is remembered where C's next get or set would use the old
//!   state, and a capture of the log asks again each time.
//! - Settings and gets that would read the port are refused during a capture.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{CMD_TRANSFER_MEMORY, Event, Link, RANGES, SERIALCOMM};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::SoundLevelMeter];
const WEIGHT_FREQ: [&str; 2] = ["A", "C"];
const WEIGHT_TIME: [&str; 2] = ["F", "S"];
const DATA_SOURCES: [&str; 2] = ["Live", "Memory"];

static DEV_OPTIONS: [OptionDesc; 10] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::SplWeightFreq, Access::GET_SET_LIST),
    OptionDesc::new(Key::SplWeightTime, Access::GET_SET_LIST),
    OptionDesc::new(Key::SplMeasurementRange, Access::GET_SET_LIST),
    OptionDesc::new(Key::Datalog, Access::GET_SET),
    OptionDesc::new(Key::HoldMax, Access::GET_SET),
    OptionDesc::new(Key::HoldMin, Access::GET_SET),
    OptionDesc::new(Key::PowerOff, Access::GET_SET),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
];

/// How long a scan looks for an `a5`, where C has 25 ms from a comment's 23 between
/// tokens. A real one has one there; a loaded host has to get to it too.
const SCAN_WINDOW: Duration = Duration::from_millis(500);
/// One read of a scan.
const SCAN_READ: Duration = Duration::from_millis(5);
/// How long a capture that only samples or a log can end goes with no byte. A
/// meter's longest gap is 23 ms.
pub const SILENCE_BOUND: Duration = Duration::from_secs(3);
/// One read of a capture, with the link's lock held.
const CAPTURE_READ: Duration = Duration::from_millis(20);
/// The most one pass reads, a few hundred ms of a meter.
const PASS_BYTES: usize = 256;
/// How long one request for the log may wait for the line. C waits for ever and
/// ignores the result; nothing here must hold a stop for long.
const ASK_WAIT: Duration = Duration::from_millis(100);
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// The `cem-dt-885x` driver.
pub struct CemDt885xDriver;

impl Driver for CemDt885xDriver {
    fn id(&self) -> &'static str {
        "cem-dt-885x"
    }

    fn long_name(&self) -> &'static str {
        "CEM DT-885x"
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
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let params = Params::parse(SERIALCOMM)?;
        let mut port = match crate::serial::open(&conn, params) {
            Ok(port) => port,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        if !hears_a_meter(port.as_mut(), SCAN_WINDOW) {
            return Ok(Vec::new());
        }
        drop(port);
        Ok(vec![Box::new(CemDt885xDevice::new(Link::new(params), conn, true))])
    }
}

/// Whether an `a5` comes in `window`.
fn hears_a_meter(port: &mut dyn Port, window: Duration) -> bool {
    let started = Instant::now();
    let mut buf = [0; 64];
    while started.elapsed() < window {
        match port.read(&mut buf, SCAN_READ) {
            Ok(n) if buf[..n].contains(&0xa5) => return true,
            Ok(_) => {}
            Err(e) => {
                log::debug!("Scan: {e}.");
                return false;
            }
        }
    }
    false
}

/// One sound level meter.
pub struct CemDt885xDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    limit_samples: u64,
    from_memory: bool,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// Whether a power off closes and opens the port: not one a test holds.
    reopens: bool,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
    /// A capture is running: settings would take its bytes.
    capturing: Arc<AtomicBool>,
    silence_bound: Duration,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl CemDt885xDevice {
    fn new(link: Link, conn: String, reopens: bool) -> Self {
        CemDt885xDevice {
            info: DeviceInfo {
                driver: "cem-dt-885x",
                vendor: "CEM".to_owned(),
                model: "DT-885x".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "SPL".to_owned())],
            conn,
            limit_samples: 0,
            from_memory: false,
            pending: None,
            reopens,
            link: Arc::new(Mutex::new(link)),
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            silence_bound: SILENCE_BOUND,
        }
    }

    /// A device on a port that stands in for the hardware. [`Device::open`] takes it.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device = Self::new(Link::new(params), conn, false);
        device.pending = Some(port);
        device
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    fn refuse_while_capturing(&self, key: &Key) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::not_supported(format!("{} while capturing", key.id())));
        }
        Ok(())
    }

    /// `cem_dt_885x_power_off`, and C's close and open that follow it. C does not
    /// look at how that open went: a set that told the meter has done its work.
    fn power_off(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        link.power_off()?;
        if self.reopens {
            link.port = None;
            match crate::serial::open(&self.conn, link.params) {
                Ok(port) => link.port = Some(port),
                Err(e) => log::warn!("After the power off: {e}"),
            }
        }
        Ok(())
    }
}

fn named(key: &Key, value: &Value, names: &[&str; 2]) -> Result<bool> {
    let text = value.as_str().ok_or_else(|| Error::arg(key.id()))?;
    match names.iter().position(|n| *n == text) {
        Some(i) => Ok(i == 1),
        None => Err(Error::Arg(format!("invalid value for {}", key.id()))),
    }
}

impl Device for CemDt885xDevice {
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
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::LimitSamples => return Ok(Value::U64(self.limit_samples)),
            Key::PowerOff => return Ok(Value::Bool(false)),
            Key::DataSource => {
                return Ok(Value::from(DATA_SOURCES[usize::from(self.from_memory)]));
            }
            Key::Datalog
            | Key::SplWeightFreq
            | Key::SplWeightTime
            | Key::HoldMax
            | Key::HoldMin
            | Key::SplMeasurementRange => self.refuse_while_capturing(key)?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        let mut link = self.link();
        Ok(match key {
            Key::Datalog => Value::Bool(link.recording_get()?),
            Key::SplWeightFreq => Value::from(WEIGHT_FREQ[usize::from(link.weight_freq_get()?)]),
            Key::SplWeightTime => Value::from(WEIGHT_TIME[usize::from(link.weight_time_get()?)]),
            Key::HoldMax => Value::Bool(link.holdmode_get()? == MqFlags::MAX),
            Key::HoldMin => Value::Bool(link.holdmode_get()? == MqFlags::MIN),
            _ => {
                let (low, high) = link.meas_range_get()?;
                Value::RangeU64(low, high)
            }
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        self.refuse_while_capturing(key)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            // `core::config_set` refuses a 0 and says so; this is for a library
            // caller that comes straight here.
            Key::LimitSamples => {
                self.limit_samples = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                Ok(())
            }
            Key::Datalog => self.link().recording_set(value.as_bool().ok_or_else(bad)?),
            Key::SplWeightFreq => {
                let c_weight = named(key, value, &WEIGHT_FREQ)?;
                self.link().weight_freq_set(c_weight)
            }
            Key::SplWeightTime => {
                let slow = named(key, value, &WEIGHT_TIME)?;
                self.link().weight_time_set(slow)
            }
            Key::HoldMax | Key::HoldMin => {
                let flag = if *key == Key::HoldMax { MqFlags::MAX } else { MqFlags::MIN };
                let on = value.as_bool().ok_or_else(bad)?;
                self.link().holdmode_set(if on { flag } else { MqFlags::empty() })
            }
            Key::SplMeasurementRange => {
                let Value::RangeU64(low, high) = value else { return Err(bad()) };
                let range = RANGES.iter().find(|r| r.0 == *low && r.1 == *high).ok_or_else(bad)?;
                self.link().meas_range_set(range.2)
            }
            Key::PowerOff => {
                if value.as_bool().ok_or_else(bad)? {
                    self.power_off()?;
                }
                Ok(())
            }
            Key::DataSource => {
                self.from_memory = named(key, value, &DATA_SOURCES)?;
                Ok(())
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let strings =
            |list: &[&str]| ValueList::Str(list.iter().map(|s| (*s).to_owned()).collect());
        match key {
            Key::SplWeightFreq => Ok(strings(&WEIGHT_FREQ)),
            Key::SplWeightTime => Ok(strings(&WEIGHT_TIME)),
            Key::DataSource => Ok(strings(&DATA_SOURCES)),
            Key::SplMeasurementRange => {
                Ok(ValueList::RangeU64(RANGES.iter().map(|r| (r.0, r.1)).collect()))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
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
        // One whose End is stuck on a full queue gives it up here, within a bound.
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
        {
            let mut link = self.link();
            if link.port.is_none() {
                return Err(Error::device("device is not open"));
            }
            link.parser.start();
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            from_memory: self.from_memory,
            enabled: self.channels[0].enabled,
            silence_bound: self.silence_bound,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new().name("cem-dt-885x acquisition".to_owned()).spawn(
            move || {
                capture.run(&sink, &stop2);
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2);
            },
        );
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

    /// Asks the thread to end, and does not wait: it may be sending. Nothing goes
    /// to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Drop for CemDt885xDevice {
    fn drop(&mut self) {
        let _ = self.close();
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

/// One capture, on its own thread: `cem_dt_885x_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    from_memory: bool,
    /// `-C`, which C ignores: its packets go out whatever it says.
    enabled: bool,
    silence_bound: Duration,
}

/// What one pass of the read brought.
struct Pass {
    bytes: usize,
    events: Vec<Event>,
    /// A log's end or the limit came, and no byte after that one was looked at.
    done: bool,
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

    /// A read's worth, with the link's lock. Each byte goes as far as C's callback
    /// took it, so a request for the log follows each byte that was read before the
    /// log's start, and nothing follows the byte that ended the capture but that.
    fn pass(&self, samples: u64, wants_log: &mut bool) -> Result<Pass> {
        let mut link = self.link.lock().expect("link lock");
        let link = &mut *link;
        let port = link.port.as_mut().ok_or_else(|| Error::device("device is not open"))?;
        let mut buf = [0; PASS_BYTES];
        let bytes = port.read(&mut buf, CAPTURE_READ)?;
        let mut pass = Pass { bytes, events: Vec::new(), done: false };
        let mut count = samples;
        for byte in &buf[..bytes] {
            let seen = pass.events.len();
            link.parser.byte(*byte, true, Instant::now(), &mut pass.events);
            for event in &pass.events[seen..] {
                match event {
                    Event::Levels(levels, _) => count += levels.len() as u64,
                    Event::End => pass.done = true,
                    Event::Interval(_) => {}
                }
            }
            if self.limit_samples > 0 && count >= self.limit_samples {
                pass.done = true;
            }
            if *wants_log {
                if link.parser.in_log_header() {
                    *wants_log = false;
                } else if let Err(e) = port.write_timeout(&[CMD_TRANSFER_MEMORY], ASK_WAIT) {
                    // C drops the result: the next byte asks again.
                    log::debug!("Asking for the log: {e}.");
                }
            }
            if pass.done {
                break;
            }
        }
        Ok(pass)
    }

    /// Sends what a pass found, cut at the limit. The number of samples so far.
    fn send(
        &self,
        events: Vec<Event>,
        mut samples: u64,
        sink: &PacketSink,
        stop: &AtomicBool,
    ) -> Option<u64> {
        let go_on = || !stop.load(Ordering::SeqCst);
        for event in events {
            match event {
                Event::Interval(ms) => {
                    let meta = Packet::Meta(vec![(Key::SampleInterval, Value::U64(ms))]);
                    if !sink.send_while(meta, go_on) {
                        return None;
                    }
                }
                Event::Levels(mut levels, flags) => {
                    if self.limit_samples > 0 {
                        let left = self.limit_samples.saturating_sub(samples);
                        levels.truncate(usize::try_from(left).unwrap_or(usize::MAX));
                    }
                    samples += levels.len() as u64;
                    if self.enabled && !levels.is_empty() {
                        let packet = Packet::Analog(AnalogData {
                            channels: vec![0],
                            samples: levels,
                            mq: Mq::SoundPressureLevel,
                            flags,
                            unit: Unit::DecibelSpl,
                            digits: 1,
                        });
                        if !sink.send_while(packet, go_on) {
                            return None;
                        }
                    }
                }
                Event::End => {}
            }
        }
        Some(samples)
    }

    fn passes(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let mut samples = 0;
        let mut wants_log = self.from_memory;
        let mut last_byte = Instant::now();
        // A meter that goes silent would otherwise hold these for ever.
        let bounded = self.limit_samples > 0 || self.from_memory;
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            let pass = self.pass(samples, &mut wants_log)?;
            if pass.bytes == 0 {
                if bounded && last_byte.elapsed() >= self.silence_bound {
                    return Err(Error::device("no byte from the meter"));
                }
                // With the lock free: a get must not starve behind this loop.
                std::thread::sleep(Duration::from_millis(1));
                continue;
            }
            last_byte = Instant::now();
            let Some(sent) = self.send(pass.events, samples, sink, stop) else { return Ok(()) };
            samples = sent;
            if pass.done {
                return Ok(());
            }
        }
    }
}

#[cfg(test)]
mod tests;
