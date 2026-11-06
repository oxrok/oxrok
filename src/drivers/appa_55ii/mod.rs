// Counterpart in sigrok:
//   libsigrok/src/hardware/appa-55ii/api.c,
//   libsigrok/src/hardware/appa-55ii/protocol.c,
//   Copyright (C) 2013 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Martin Ling, Gerhard Sittig, Soeren Apel, Daniel Elstner, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The APPA 55II logging thermometer, and its clone the RS 55II, two probes on a
//! serial port at 9600 baud.
//!
//! It only sends. A scan looks for a frame among the first 50 bytes, for 500 ms,
//! and closes the port. A capture reads frames: live ones about three a second, or
//! a log a user starts on the device, by `data_source`. Nothing is sent to it, so
//! a capture has no requests to keep in step: each frame has a sync and a sum, and
//! a capture that starts in mid-frame finds the next.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - A time limit ends a capture on a silent device. C checks its limits only after
//!   a read, so it waits for ever (`protocol.c:282-283`, `:311`), and a limit can
//!   overrun by a period on a stream, where this one is on time.
//! - A port that goes away ends the capture with an error. C takes the hangup for
//!   no data, for ever (`protocol.c:282`).
//! - Log bytes with no record due are dropped. In C they pile up, and at 64 the
//!   loop in `appa_55ii_log_data` makes no progress, for ever (`protocol.c:208-216`).
//! - A frame with a wrong sum or bytes that start no frame, with records due, end a
//!   log's records with a warning. C joins the pieces that are left and reads every
//!   later record from the wrong bytes (`protocol.c:237-239`, `:245-247`). A piece
//!   gone without a trace reads as wrong here too, as nothing shows it before the
//!   end, where a log short of its count gets a warning. So does log data with no
//!   metadata ahead of it. All of these end at the log's End, as a good capture.
//! - A live capture under a sample limit alone ends with an error after
//!   [`SILENCE_BOUND`] with no byte at all, as the unit sends about three frames a
//!   second in menus and hold too, so silence means off or the cable out. This is a
//!   choice: C waits for ever.
//! - A metadata frame under 2 bytes keeps the count and warns. C reads the sum and
//!   the next byte as the count (`protocol.c:139`).
//! - A live frame too short for its probes is no sample. C reads what its buffer
//!   held and sends that (`protocol.c:79-81`).
//! - Nothing is sent after the End. C parses on after a log's end (`protocol.c:227`,
//!   `:298`), and calls its stop again if a limit came up in that read (`:311-312`).
//! - A capture starts clean, and a later one on an open port drops what came
//!   between. C keeps its buffers and its count of records, and reads what has
//!   waited in the tty as though it were new. A first capture does read that, as C
//!   does: a client that starts long after its open gets up to about 54 s of old
//!   frames as new, the tty's 4 KB, since a purge could take a log's start.
//!
//! As in C, a size over 32 ends a capture, as nothing can come of it and the buffer
//! fills (`protocol.c:306-309`), with a warning here. A log that is never sent does
//! not end one: a user may press the button at any time, so only a limit or a stop
//! does. Nor does silence under a time limit, with no limit, or in memory mode.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{BUF_SIZE, Event, Parser, packet_valid};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SERIALCOMM: &str = "9600/8n1";
const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Thermometer];
const DATA_SOURCES: [&str; 2] = ["Live", "Memory"];
const CHANNELS: usize = 2;

static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
];

/// `serial_stream_detect`'s arguments in `scan`: the buffer, the bytes one try
/// wants, and the time.
const DETECT_BYTES: usize = 50;
const DETECT_TRY: usize = 25;
const DETECT_TIMEOUT: Duration = Duration::from_millis(500);
/// How long a read waits: `serial_source_add`'s 50 ms in `dev_acquisition_start`.
const READ_WAIT: Duration = Duration::from_millis(50);
/// How long a stopped capture tries to hand over its End on a full queue: a close
/// joins this thread.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How long a live capture that only a sample limit can end goes with no byte.
pub const SILENCE_BOUND: Duration = Duration::from_secs(10);

/// The `appa-55ii` driver.
pub struct Appa55iiDriver;

impl Driver for Appa55iiDriver {
    fn id(&self) -> &'static str {
        "appa-55ii"
    }

    fn long_name(&self) -> &'static str {
        "APPA 55II"
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
        let (mut port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        log::info!("Probing serial port {conn}.");
        if !detect(port.as_mut(), &params, DETECT_TIMEOUT) {
            return Ok(Vec::new());
        }
        log::info!("Found device on port {conn}.");
        drop(port);
        Ok(vec![Box::new(Appa55iiDevice::new(conn, params, None))])
    }
}

/// `serial_stream_detect` with this driver's arguments: a byte at a time, and from
/// the 25th one try at the next place per byte. So 25 bytes ahead of a frame are
/// all it passes over. A port that fails finds nothing, as it reads as no bytes
/// until the time is up in C.
fn detect(port: &mut dyn Port, params: &Params, timeout: Duration) -> bool {
    let byte_wait = crate::serial::serial_timeout(params, 1);
    let start = Instant::now();
    let mut buf = Vec::with_capacity(DETECT_BYTES);
    let mut check = 0;
    let mut byte = [0u8; 1];
    while buf.len() < DETECT_BYTES {
        match port.read(&mut byte, byte_wait) {
            Ok(0) => {}
            Ok(_) => buf.push(byte[0]),
            Err(e) => {
                log::debug!("Reading failed: {e}.");
                return false;
            }
        }
        if buf.len() - check >= DETECT_TRY {
            if packet_valid(&buf[check..]) {
                return true;
            }
            check += 1;
        }
        if start.elapsed() >= timeout {
            break;
        }
    }
    log::info!("Didn't find a valid packet (read {} bytes).", buf.len());
    false
}

/// What a client may change while a capture runs, as C's callback reads its context
/// each time.
#[derive(Debug, Default)]
struct Settings {
    limit_samples: AtomicU64,
    limit_msec: AtomicU64,
    memory: AtomicBool,
    enabled: [AtomicBool; CHANNELS],
}

/// One thermometer.
pub struct Appa55iiDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    settings: Arc<Settings>,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// The port while no capture holds it. A capture puts it back before its End,
    /// so a client that saw the End can start again.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    worker: Option<Worker>,
    /// A capture has run since the open.
    captured: bool,
    silence_bound: Duration,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl Appa55iiDevice {
    fn new(conn: String, params: Params, port: Option<Box<dyn Port>>) -> Self {
        let settings = Settings::default();
        for enabled in &settings.enabled {
            enabled.store(true, Ordering::SeqCst);
        }
        Appa55iiDevice {
            info: DeviceInfo {
                driver: "appa-55ii",
                vendor: "APPA".to_owned(),
                model: "55II".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: vec![
                Channel::new(0, ChannelKind::Analog, "T1".to_owned()),
                Channel::new(1, ChannelKind::Analog, "T2".to_owned()),
            ],
            conn,
            params,
            settings: Arc::new(settings),
            pending: port,
            port: Arc::new(Mutex::new(None)),
            worker: None,
            captured: false,
            silence_bound: SILENCE_BOUND,
        }
    }

    /// A device on a port that stands in for the hardware, with no scan. [`Device::open`]
    /// takes it.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let name = port.name().to_owned();
        Self::new(name, params, Some(port))
    }

    fn end_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for Appa55iiDevice {
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
        self.settings.enabled[pos].store(enabled, Ordering::SeqCst);
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
        let s = &self.settings;
        Ok(match key {
            Key::LimitSamples => Value::U64(s.limit_samples.load(Ordering::SeqCst)),
            Key::LimitMsec => Value::U64(s.limit_msec.load(Ordering::SeqCst)),
            Key::DataSource => {
                Value::from(DATA_SOURCES[usize::from(s.memory.load(Ordering::SeqCst))])
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// Takes effect at once, in a capture too: nothing goes to the device.
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let s = &self.settings;
        match key {
            // `core::config_set` refuses a 0 and says so; this is for a library
            // caller that comes straight here.
            Key::LimitSamples => {
                let n = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                s.limit_samples.store(n, Ordering::SeqCst);
            }
            Key::LimitMsec => {
                let n = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                s.limit_msec.store(n, Ordering::SeqCst);
            }
            Key::DataSource => {
                let name = value.as_str().ok_or_else(bad)?;
                let idx = DATA_SOURCES.iter().position(|d| *d == name).ok_or_else(bad)?;
                s.memory.store(idx == 1, Ordering::SeqCst);
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
            Key::DataSource => {
                Ok(ValueList::Str(DATA_SOURCES.iter().map(|s| (*s).to_owned()).collect()))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `std_serial_dev_open`.
    fn open(&mut self) -> Result<()> {
        let mut slot = self.port.lock().expect("port lock");
        if slot.is_some() || self.worker.is_some() {
            return Ok(());
        }
        *slot = Some(match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&self.conn, self.params)?,
        });
        self.captured = false;
        Ok(())
    }

    /// `std_serial_dev_close`.
    fn close(&mut self) -> Result<()> {
        self.end_worker();
        self.port.lock().expect("port lock").take();
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        // A capture holds the port until just before its End.
        if self.worker.is_some() && self.port.lock().expect("port lock").is_none() {
            return Err(Error::Busy);
        }
        // Over, though its End may be stuck on a full queue: the stop lets it go.
        self.end_worker();
        let Some(mut port) = self.port.lock().expect("port lock").take() else {
            return Err(Error::device("device is not open"));
        };
        // What came since the last capture is old, and C would read it as new. Not
        // on a first capture, as C: right behind the open's flush nothing is old, and
        // a purge could take the start of a log. A client that starts much later
        // reads what has waited, up to about 54 s of frames, as new.
        if self.captured
            && let Err(e) = port.purge()
        {
            *self.port.lock().expect("port lock") = Some(port);
            return Err(e);
        }
        self.captured = true;
        let settings = Arc::clone(&self.settings);
        let slot = Arc::clone(&self.port);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let silence_bound = self.silence_bound;
        let handle = std::thread::Builder::new()
            .name("appa-55ii acquisition".to_owned())
            .spawn(move || {
                let capture = Capture { settings, sink: &sink, stop: &stop2, silence_bound };
                capture.run(port.as_mut());
                *slot.lock().expect("port lock") = Some(port);
                send_end(&sink, &stop2);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending. Nothing goes to
    /// the device, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Drop for Appa55iiDevice {
    fn drop(&mut self) {
        self.end_worker();
    }
}

/// `End`, waiting for room in the queue for as long as nobody asked to stop, and
/// after a stop only for [`END_AFTER_STOP`].
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

/// One capture, on its own thread: `appa_55ii_receive_data`.
struct Capture<'a> {
    settings: Arc<Settings>,
    sink: &'a PacketSink,
    stop: &'a AtomicBool,
    silence_bound: Duration,
}

impl Capture<'_> {
    fn stopped(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    fn send(&self, packet: Packet) -> bool {
        self.sink.send_while(packet, || !self.stopped())
    }

    fn run(&self, port: &mut dyn Port) {
        if !self.send(Packet::header()) {
            return;
        }
        let started = Instant::now();
        let mut last_byte = started;
        let mut parser = Parser::default();
        let mut samples = 0u64;
        let mut events = Vec::new();
        let mut chunk = [0u8; BUF_SIZE];
        loop {
            if self.stopped() || self.time_is_up(started) {
                return;
            }
            let room = parser.room();
            let n = match port.read(&mut chunk[..room], READ_WAIT) {
                Ok(n) => n,
                Err(e) => {
                    log::error!("Serial port read error: {e}.");
                    return;
                }
            };
            if n == 0 {
                if self.silence_is_final(last_byte) {
                    log::error!("No byte from the device for {:?}.", self.silence_bound);
                    return;
                }
                continue;
            }
            last_byte = Instant::now();
            let memory = self.settings.memory.load(Ordering::SeqCst);
            let full = parser.feed(&chunk[..n], memory, &mut events);
            for event in events.drain(..) {
                match event {
                    Event::Live { probes, flags } => {
                        if !self.send_live(&probes, flags) {
                            return;
                        }
                        samples += 1;
                    }
                    Event::Record { probes, time } => {
                        log::debug!("Timestamp: {:02}:{:02}:{:02}", time[0], time[1], time[2]);
                        if !self.send_record(probes) {
                            return;
                        }
                        samples += 1;
                    }
                    Event::LogEnd => return,
                }
            }
            if full {
                log::warn!("A frame's size is out of range: the capture ends.");
                return;
            }
            // As C: after all a read has brought, so a piece of a log that finishes
            // two records sends both.
            let limit = self.settings.limit_samples.load(Ordering::SeqCst);
            if limit > 0 && samples >= limit {
                return;
            }
        }
    }

    /// A capture in Live mode that only a sample limit can end. In memory mode the
    /// device may be waiting for its user.
    fn silence_is_final(&self, last_byte: Instant) -> bool {
        let s = &self.settings;
        s.limit_samples.load(Ordering::SeqCst) > 0
            && s.limit_msec.load(Ordering::SeqCst) == 0
            && !s.memory.load(Ordering::SeqCst)
            && last_byte.elapsed() >= self.silence_bound
    }

    fn time_is_up(&self, started: Instant) -> bool {
        let limit = self.settings.limit_msec.load(Ordering::SeqCst);
        limit > 0 && started.elapsed() >= Duration::from_millis(limit)
    }

    fn enabled(&self, channel: usize) -> bool {
        self.settings.enabled[channel].load(Ordering::SeqCst)
    }

    /// One packet per probe that is on: `appa_55ii_live_data`.
    fn send_live(&self, probes: &[(f32, i32); CHANNELS], flags: MqFlags) -> bool {
        for (channel, (value, digits)) in probes.iter().enumerate() {
            if self.enabled(channel)
                && !self.send(analog(vec![channel], vec![*value], flags, *digits))
            {
                return false;
            }
        }
        true
    }

    /// One packet for both probes: `appa_55ii_log_data_parse`.
    fn send_record(&self, probes: [f32; CHANNELS]) -> bool {
        let channels: Vec<usize> = (0..CHANNELS).filter(|c| self.enabled(*c)).collect();
        if channels.is_empty() {
            return true;
        }
        let samples = channels.iter().map(|c| probes[*c]).collect();
        self.send(analog(channels, samples, MqFlags::empty(), 1))
    }
}

fn analog(channels: Vec<usize>, samples: Vec<f32>, flags: MqFlags, digits: i32) -> Packet {
    Packet::Analog(AnalogData {
        channels,
        samples,
        mq: Mq::Temperature,
        flags,
        unit: Unit::Celsius,
        digits,
    })
}

#[cfg(test)]
mod tests;
