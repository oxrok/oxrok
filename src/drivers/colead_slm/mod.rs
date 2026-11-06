// Counterpart in sigrok:
//   libsigrok/src/hardware/colead-slm/api.c,
//   libsigrok/src/hardware/colead-slm/protocol.c,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Daniel Elstner,
//   Martin Ling, Soeren Apel, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Colead SL-5868P sound level meter, on a serial port.
//!
//! A scan needs `conn=` and touches nothing: any port is a meter until a capture
//! finds otherwise, as in C. `serialcomm=` overrides 2400/8n1, and is looked at
//! when the port opens. Nothing can be read or set on the meter. It has `limit_samples`
//! and `limit_time`, which can only be set.
//!
//! A capture waits for the meter to say a measurement is ready, asks for it, and
//! takes the ten bytes that follow ([`protocol`]). There is no request timer, so
//! requests can't pile up: one is out at a time, and a request goes only when a
//! read gives the one byte `0x10`. A record that doesn't come whole is filled by
//! what comes next, and is then refused by its own form, so a late reply can lose
//! readings but not shift them. Sizes of reads are as in C for that reason.
//!
//! Not copied from C, which looks at its limits only behind a level
//! (protocol.c:177-180) and takes a port that hangs up for a timeout (:199-201):
//!
//! - A port that fails ends the capture with an error.
//! - A time limit ends a capture behind the first level past it, as in C, or
//!   [`TIME_GRACE`] past it when none came, as a meter announces only twice a
//!   second. A capture that got no level then says so and still exits 0: the limit
//!   was met.
//! - When only a sample limit can end a capture, [`NO_LEVEL_BOUND`] with no level
//!   ends it with an error, which says whether any byte came. A meter that sends
//!   well-formed records saying "not valid" is the exception: it isn't silent, and
//!   C's comment has a mean run from 11 s to 24 h, so that is warned about once and
//!   waited for, as C waits. `--time` bounds a script.
//!
//! A record with no digits reads 0.0 dB, below the meter's range of 30 to 130 dB.
//! It goes out as in C, and is warned about once per capture.
//!
//! Nor C's `char` arithmetic, which depends on the platform; see
//! [`protocol::decode`]. Nor its state across captures: a capture here starts
//! with no record owed, where C goes on filling the one a stop cut.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{IDLE_READ, Machine, NoLevel, SEND, Silence, Step};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// api.c's `SERIALCOMM`.
const SERIALCOMM: &str = "2400/8n1";
const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::SoundLevelMeter];
/// Set only.
const LIMIT: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, LIMIT),
    OptionDesc::new(Key::LimitMsec, LIMIT),
];

/// How long a read of a capture waits, `serial_source_add`'s 150 ms.
const READ_TIMEOUT: Duration = Duration::from_millis(150);
/// How long a thread that was told to stop waits to put its `End` in a queue no
/// one reads. A close joins it.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How long no level may come when a sample limit is all that can end a capture.
/// The meter sends two a second. A 10-second mean is the longest thing the wiki
/// names that could hold a meter's first level back, though nothing says it does.
const NO_LEVEL_BOUND: Duration = Duration::from_secs(15);
/// How long past a time limit a capture with no level past it goes on: longer
/// than the 500 ms between a meter's announcements, so that C's level behind the
/// limit comes here too.
const TIME_GRACE: Duration = Duration::from_secs(1);

/// The driver.
pub struct ColeadSlmDriver;

impl Driver for ColeadSlmDriver {
    fn id(&self) -> &'static str {
        "colead-slm"
    }

    fn long_name(&self) -> &'static str {
        "Colead SLM"
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
        Ok(vec![Box::new(SlmDevice::new(conn, serialcomm, None))])
    }
}

/// The software limits, which a set changes under a running capture, as in C.
#[derive(Debug, Default)]
struct Limits {
    samples: AtomicU64,
    msec: AtomicU64,
}

/// One meter.
pub struct SlmDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    serialcomm: String,
    /// The line's settings, once the port opened.
    params: Option<Params>,
    limits: Arc<Limits>,
    /// [`NO_LEVEL_BOUND`], which a test shortens.
    no_level_bound: Duration,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// The port, unless a capture holds it. It comes back before `End`.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl SlmDevice {
    fn new(conn: String, serialcomm: String, port: Option<Box<dyn Port>>) -> Self {
        SlmDevice {
            info: DeviceInfo {
                driver: "colead-slm",
                vendor: "Colead".to_owned(),
                model: "SL-5868P".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "P1")],
            conn,
            serialcomm,
            params: None,
            limits: Arc::new(Limits::default()),
            no_level_bound: NO_LEVEL_BOUND,
            pending: port,
            port: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open port.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let name = port.name().to_owned();
        Self::new(name, SERIALCOMM.to_owned(), Some(port))
    }

    /// Tells a worker to stop and waits for it.
    fn join(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for SlmDevice {
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

    /// The C driver has no `config_get`.
    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let number =
            || value.as_u64().ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())));
        match key {
            Key::LimitSamples => self.limits.samples.store(number()?, Ordering::SeqCst),
            Key::LimitMsec => self.limits.msec.store(number()?, Ordering::SeqCst),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn open(&mut self) -> Result<()> {
        let mut slot = self.port.lock().expect("port lock");
        if slot.is_some() {
            return Ok(());
        }
        let params = Params::parse(&self.serialcomm)?;
        let port = match self.pending.take() {
            // Flushed as `serial_open` flushes a port it opens.
            Some(mut p) => {
                p.purge()?;
                p
            }
            None => crate::serial::open(&self.conn, params)?,
        };
        self.params = Some(params);
        *slot = Some(port);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.join();
        self.port.lock().expect("port lock").take();
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        let port = self.port.lock().expect("port lock").take();
        let Some(mut port) = port else {
            // No port and a worker that lives: it holds the port. The worker
            // gives it back before its End, so a client that saw an End is
            // never told this.
            if self.worker.as_ref().is_some_and(|w| !w.handle.is_finished()) {
                return Err(Error::Busy);
            }
            return Err(Error::device("device is not open"));
        };
        // A last capture's thread may wait to send its End to a queue nobody
        // reads: tell it, and it gives up within a bound.
        self.join();
        let params = self.params.expect("a port that is open has its settings");
        let capture = Capture {
            write_timeout: crate::serial::serial_timeout(&params, 1),
            limits: Arc::clone(&self.limits),
            no_level_bound: self.no_level_bound,
        };
        let slot = Arc::clone(&self.port);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("colead-slm acquisition".to_owned())
            .spawn(move || {
                let outcome = capture.run(port.as_mut(), &sink, &stop2);
                *slot.lock().expect("port lock") = Some(port);
                send_end(&sink, &stop2, outcome);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// `std_serial_dev_acquisition_stop` has nothing to send: the flag ends the
    /// capture. A close joins.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Drop for SlmDevice {
    fn drop(&mut self) {
        self.join();
    }
}

/// `End`, waiting for room for as long as nobody asked to stop, and after a stop
/// only for [`END_AFTER_STOP`]: a close joins this thread.
fn send_end(sink: &PacketSink, stop: &AtomicBool, outcome: Result<()>) {
    let stopped_at = std::cell::Cell::new(None);
    sink.end_while(outcome, || {
        if !stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = stopped_at.get().unwrap_or_else(Instant::now);
        stopped_at.set(Some(at));
        at.elapsed() < END_AFTER_STOP
    });
}

/// One capture, on its own thread.
struct Capture {
    write_timeout: Duration,
    limits: Arc<Limits>,
    no_level_bound: Duration,
}

impl Capture {
    /// Runs until a limit, a stop, a port that fails, a consumer that left, or
    /// no level for as long as a sample limit alone may wait.
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let send = |p: Packet| sink.send_while(p, || !stop.load(Ordering::SeqCst));
        if !send(Packet::header()) {
            return Ok(());
        }
        let started = Instant::now();
        let mut samples = 0u64;
        let mut machine = Machine::default();
        let mut no_level = NoLevel::new(self.no_level_bound, started);
        let mut only_samples = self.only_samples();
        let mut any_bytes = false;
        let mut said_no_digits = false;
        let mut chunk = [0u8; IDLE_READ];
        while !stop.load(Ordering::SeqCst) {
            // The read comes first, so a reply that is ready wins over any timer.
            let n = port.read(&mut chunk[..machine.want()], READ_TIMEOUT)?;
            let msec = self.limits.msec.load(Ordering::SeqCst);
            let over = |grace: Duration| {
                msec > 0 && started.elapsed() > Duration::from_millis(msec) + grace
            };
            if n == 0 {
                // A hung up tty may give this at once: never spin on it.
                std::thread::sleep(Duration::from_millis(1));
            } else {
                any_bytes = true;
                no_level.heard();
                match machine.got(&chunk[..n]) {
                    Step::Wait => {}
                    Step::Ask => self.ask(port, &mut machine)?,
                    Step::Record(record) => {
                        log::trace!("received packet:{}", spew(&record));
                        match protocol::decode(&record) {
                            Err(why) => {
                                log::debug!("{why}");
                                if protocol::says_not_valid(&record) {
                                    no_level.not_valid();
                                }
                            }
                            Ok(level) => {
                                if level.no_digits && !said_no_digits {
                                    said_no_digits = true;
                                    log::warn!("{NO_DIGITS}");
                                }
                                if !send(Packet::Analog(AnalogData {
                                    channels: vec![0],
                                    samples: vec![level.value],
                                    mq: Mq::SoundPressureLevel,
                                    flags: level.flags,
                                    unit: Unit::DecibelSpl,
                                    digits: 1,
                                })) {
                                    return Ok(());
                                }
                                // After the send: a slow consumer is no silence.
                                no_level.level(Instant::now());
                                samples += 1;
                                let limit = self.limits.samples.load(Ordering::SeqCst);
                                if (limit > 0 && samples >= limit) || over(Duration::ZERO) {
                                    return Ok(());
                                }
                            }
                        }
                    }
                }
            }
            // A record on its way may finish: it takes at most ten more reads with
            // bytes in them, so bytes that flow can't hold this off.
            if !(n > 0 && machine.waiting()) && over(TIME_GRACE) {
                if samples == 0 {
                    log::warn!("{}", time_up_with_no_level(any_bytes));
                }
                return Ok(());
            }
            // A client that sets the limit now gets the whole bound from now.
            let now_only_samples = self.only_samples();
            if now_only_samples && !only_samples {
                no_level.restart(Instant::now());
            }
            only_samples = now_only_samples;
            match no_level.look(Instant::now()) {
                Some(Silence::NotValid) if only_samples => {
                    log::warn!("{}", not_valid_warning(self.no_level_bound));
                }
                Some(Silence::NoBytes | Silence::NoLevel) if only_samples => {
                    let silence = no_level.silence();
                    return Err(no_level_error(silence, self.no_level_bound));
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Whether a sample limit is all that can end this capture.
    fn only_samples(&self) -> bool {
        self.limits.msec.load(Ordering::SeqCst) == 0
            && self.limits.samples.load(Ordering::SeqCst) > 0
    }

    /// The request. A write that finds no room is a request not sent, and the
    /// next announcement tries again, as in C. A port that fails is the end.
    fn ask(&self, port: &mut dyn Port, machine: &mut Machine) -> Result<()> {
        match port.write_timeout(&[SEND], self.write_timeout) {
            Ok(1) => machine.asked(),
            Ok(_) => log::error!("unable to send command"),
            Err(e) => {
                log::error!("unable to send command");
                return Err(e);
            }
        }
        Ok(())
    }
}

/// The record as `process_packet` logs it.
fn spew(record: &[u8]) -> String {
    use std::fmt::Write as _;
    record.iter().fold(String::new(), |mut out, b| {
        let _ = write!(out, " {b:02x}");
        out
    })
}

/// What a record with no digits gets, once per capture.
const NO_DIGITS: &str = "A record with no digits reads 0.0 dB, below the meter's range of 30 to \
                         130 dB: it is no measurement.";

/// What a time limit says when it ended a capture that got no level.
fn time_up_with_no_level(any_bytes: bool) -> &'static str {
    if any_bytes {
        "No usable level from the meter in the time limit, though bytes came."
    } else {
        "No bytes from the meter in the time limit: it is off, on another line setting, or its \
         cable is out."
    }
}

/// What a meter gets that has said "not valid" for a whole bound, once per capture.
fn not_valid_warning(bound: Duration) -> String {
    format!(
        "The meter has sent no valid level for {} s, and says its readings are not valid. Waiting \
         on, as a mean can take up to 24 h: --time bounds a capture.",
        bound.as_secs()
    )
}

/// A capture's end for lack of a level.
fn no_level_error(silence: Silence, bound: Duration) -> Error {
    let seconds = bound.as_secs();
    Error::device(match silence {
        Silence::NoBytes => format!(
            "no bytes from the meter for {seconds} s: it announces two measurements a second, \
             so it is off, on another line setting, or its cable is out"
        ),
        Silence::NoLevel | Silence::NotValid => {
            format!("no usable level from the meter for {seconds} s, though bytes came")
        }
    })
}

#[cfg(test)]
mod tests;
