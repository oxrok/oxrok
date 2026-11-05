// Counterpart in sigrok:
//   libsigrok/src/hardware/agilent-dmm/api.c,
//   libsigrok/src/hardware/agilent-dmm/protocol.c,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Aurelien Jacobs,
//   Martin Ling, Daniel Elstner, Soeren Apel, Gerd v. Egidy, Gerhard Sittig,
//   Wolfram Sang, Alex Hornung
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Agilent and Keysight U12xx handheld multimeters, behind an IR cable.
//!
//! `conn=` names the port, and the line is 9600/8n1 unless `serialcomm=` says.
//! A scan asks `*IDN?` once and closes the port.
//!
//! A capture runs C's jobs: `CONF?` and `STAT?` once a second, and `FETC?` per
//! enabled channel at the sample rate (1 to 20 Hz, 5 by default), one query at
//! a time, 300 ms each at most. A `FETC?` given up is asked again at once, as
//! its interval is over by then, so a meter that stops answering it gets no
//! more `CONF?` until it answers, as in C. A source other than `Live` reads the `U124xC`'s or
//! U128x's log, by index from 1, until the meter says `*E` or a limit ends it.
//!
//! `--samples N` counts samples, one per channel, so a meter of three channels
//! ends at 2 after P2.
//!
//! A meter that goes quiet, or a log entry that never reads, would hold a
//! capture only a sample limit can end for ever, as C does: here that ends after
//! [`SILENCE`]. A meter that answers and has no sample for the moment (the dial
//! on square wave, or a display with no quantity) is waited for, as the dial
//! may come back. A port that is gone ends the capture.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Trigger, Value, ValueList,
};
use crate::serial::{Params, Port, read_blocking};
use protocol::{
    DATA_SOURCES, IDENTITY_MAX, IDN, Identity, LINE_MAX, Meter, PROFILES, Profile, Reading,
    WRITE_TIMEOUT,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
/// `SERIALCOMM`.
const SERIALCOMM: &str = "9600/8n1";
/// `serial_readline`'s time in the scan.
const IDENTITY_TIME: Duration = Duration::from_millis(250);
/// `serial_source_add`'s 10 ms: how long the capture waits for a byte before it
/// looks at its limits and jobs.
const POLL: Duration = Duration::from_millis(10);
/// `samplerates`: lowest, highest, step.
const RATE_MIN: u64 = 1;
const RATE_MAX: u64 = 20;
const RATE_STEP: u64 = 1;
/// `cur_samplerate` after a scan.
const RATE_DEFAULT: u64 = 5;
/// How long a capture that only a sample limit can end goes on with no answer at
/// all, or on a log with no sample. Long enough for a U1177A's Bluetooth link
/// to come back, or an IR cable to be put back on its meter.
pub const SILENCE: Duration = Duration::from_secs(10);
/// How long a worker that was told to stop tries to hand over its `End`.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
static DEV_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
];

/// The `agilent-dmm` driver.
pub struct AgilentDmmDriver;

impl Driver for AgilentDmmDriver {
    fn id(&self) -> &'static str {
        "agilent-dmm"
    }

    fn long_name(&self) -> &'static str {
        "Agilent U12xx series DMMs"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => serialcomm = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let text = serialcomm.as_deref().unwrap_or(SERIALCOMM);
        let opened = Params::parse(text)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (mut port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        let identity = identify(port.as_mut());
        Ok(identity
            .map(|i| -> Box<dyn Device> { Box::new(AgilentDevice::new(i, conn, params)) })
            .into_iter()
            .collect())
    }
}

/// The scan's one question.
fn identify(port: &mut dyn Port) -> Option<Identity> {
    match port.write_timeout(IDN, WRITE_TIMEOUT) {
        Ok(n) if n == IDN.len() => {}
        Ok(_) => {
            log::error!("Unable to send identification string.");
            return None;
        }
        Err(e) => {
            log::debug!("{e}");
            log::error!("Unable to send identification string.");
            return None;
        }
    }
    let line = read_line(port);
    log::debug!("Received '{}'.", line.escape_ascii());
    protocol::parse_identity(&line)
}

/// `serial_readline`: up to a CR or LF, [`IDENTITY_MAX`] bytes or
/// [`IDENTITY_TIME`], and what came counts either way. A port that fails ends it.
fn read_line(port: &mut dyn Port) -> Vec<u8> {
    let start = Instant::now();
    let mut line = Vec::new();
    while line.len() < IDENTITY_MAX {
        let Some(left) = IDENTITY_TIME.checked_sub(start.elapsed()).filter(|d| !d.is_zero()) else {
            break;
        };
        let mut byte = [0];
        match read_blocking(port, &mut byte, left) {
            Ok(0) => {}
            Ok(_) => {
                if matches!(byte[0], b'\r' | b'\n') {
                    break;
                }
                line.push(byte[0]);
            }
            Err(e) => {
                log::debug!("{e}");
                break;
            }
        }
    }
    line
}

/// One meter.
pub struct AgilentDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    profile: &'static Profile,
    limit_samples: u64,
    limit_msec: u64,
    samplerate: u64,
    data_source: usize,
    silence: Duration,
    /// A port to use instead of opening `conn`, for tests and replays.
    pending: Option<Box<dyn Port>>,
    /// The port while it is open and no capture holds it: a capture gives
    /// it back before its `End`, so a client may start again on seeing that.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    capturing: Arc<AtomicBool>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl AgilentDevice {
    fn new(identity: Identity, conn: String, params: Params) -> Self {
        let profile = identity.profile;
        let names = ["P1", "P2", "P3"];
        AgilentDevice {
            info: DeviceInfo {
                driver: "agilent-dmm",
                vendor: identity.vendor.to_owned(),
                model: profile.name.to_owned(),
                version: identity.version,
                serial: String::new(),
                conn: None,
            },
            channels: names
                .iter()
                .take(profile.channels)
                .enumerate()
                .map(|(i, n)| Channel::new(i, ChannelKind::Analog, (*n).to_owned()))
                .collect(),
            conn,
            params,
            profile,
            limit_samples: 0,
            limit_msec: 0,
            samplerate: RATE_DEFAULT,
            data_source: 0,
            silence: SILENCE,
            pending: None,
            port: Arc::new(Mutex::new(None)),
            capturing: Arc::new(AtomicBool::new(false)),
            worker: None,
        }
    }

    /// A device of `model`, one of [`PROFILES`], on a port that stands in for
    /// the hardware, without a scan. [`Device::open`] takes it.
    ///
    /// # Panics
    ///
    /// On a model the table lacks.
    pub fn from_port(model: &str, port: Box<dyn Port>) -> Self {
        let profile = PROFILES.iter().find(|p| p.name == model).expect("a model of the table");
        let identity = Identity { vendor: "Agilent", profile, version: String::new() };
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device = Self::new(identity, conn, params);
        device.pending = Some(port);
        device
    }

    /// C's refusals at the start, which need no device.
    fn refusal(&self) -> Option<&'static str> {
        if self.data_source == 0 {
            None
        } else if !self.profile.has_log() {
            Some("Log data source is not implemented for this model.")
        } else if !self.channels[0].enabled {
            Some("Log data is only available for channel P1.")
        } else {
            None
        }
    }
}

fn no_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        None => Ok(()),
    }
}

impl Device for AgilentDevice {
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
        no_group(group)?;
        Ok(DEV_OPTIONS.to_vec())
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        no_group(group)?;
        Ok(match key {
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::LimitMsec => Value::U64(self.limit_msec),
            Key::Samplerate => Value::U64(self.samplerate),
            Key::DataSource => Value::Str(DATA_SOURCES[self.data_source].to_owned()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => self.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::LimitMsec => self.limit_msec = value.as_u64().ok_or_else(bad)?,
            Key::Samplerate => {
                let rate = value.as_u64().ok_or_else(bad)?;
                if !(RATE_MIN..=RATE_MAX).contains(&rate) {
                    return Err(bad());
                }
                self.samplerate = rate;
            }
            Key::DataSource => {
                let name = value.as_str().ok_or_else(bad)?;
                self.data_source = DATA_SOURCES.iter().position(|s| *s == name).ok_or_else(bad)?;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        match key {
            Key::Samplerate => {
                Ok(ValueList::RangeStepU64 { min: RATE_MIN, max: RATE_MAX, step: RATE_STEP })
            }
            Key::DataSource => {
                Ok(ValueList::Str(DATA_SOURCES.iter().map(|s| (*s).to_owned()).collect()))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut slot = self.port.lock().expect("port lock");
        if slot.is_some() {
            return Ok(());
        }
        *slot = Some(match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&self.conn, self.params)?,
        });
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        *self.port.lock().expect("port lock") = None;
        Ok(())
    }

    fn check_capture(&self, _trigger: Option<&Trigger>) -> Result<()> {
        match self.refusal() {
            Some(why) => Err(Error::NotSupported(why.to_owned())),
            None => Ok(()),
        }
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        self.check_capture(None)?;
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        if self.port.lock().expect("port lock").is_none() {
            return Err(Error::device("device is not open"));
        }
        let enabled: Vec<usize> =
            self.channels.iter().filter(|c| c.enabled).map(|c| c.index).collect();
        if enabled.is_empty() {
            return Err(Error::arg("No channel is enabled."));
        }
        self.capturing.store(true, Ordering::SeqCst);
        let capture = Capture {
            profile: self.profile,
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            samplerate: self.samplerate,
            data_source: self.data_source,
            silence: self.silence,
            enabled,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let slot = Arc::clone(&self.port);
        let capturing = Arc::clone(&self.capturing);
        // The thread takes the port from the slot, so a spawn that fails leaves
        // it there, for a later start and for close.
        let spawned = std::thread::Builder::new().name("agilent-dmm acquisition".to_owned()).spawn(
            move || {
                let port = slot.lock().expect("port lock").take();
                if let Some(mut port) = port {
                    capture.run(port.as_mut(), &sink, &stop2);
                    *slot.lock().expect("port lock") = Some(port);
                }
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2);
            },
        );
        let handle = match spawned {
            Ok(handle) => handle,
            Err(e) => {
                self.capturing.store(false, Ordering::SeqCst);
                return Err(Error::Io(e));
            }
        };
        // A worker that came before has given its port back and sent, or is
        // sending, its End: it holds nothing that this one needs.
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending. It sends
    /// nothing to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// `End`, waiting for room while nobody has said stop, and for a moment after.
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

/// One capture: `dev_acquisition_start` and `agdmm_receive_data`.
struct Capture {
    profile: &'static Profile,
    limit_samples: u64,
    limit_msec: u64,
    samplerate: u64,
    data_source: usize,
    silence: Duration,
    enabled: Vec<usize>,
}

impl Capture {
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) {
        let go_on = || !stop.load(Ordering::SeqCst);
        let started = Instant::now();
        let limit = Duration::from_millis(self.limit_msec);
        if !sink.send_while(Packet::header(), go_on) {
            return;
        }
        let mut meter =
            Meter::new(self.profile, self.enabled.clone(), self.data_source, self.samplerate);
        let only_samples_can_end = self.limit_samples > 0 && self.limit_msec == 0;
        let mut samples = 0u64;
        let mut lines = Lines::default();
        let mut progress = Instant::now();
        while go_on() {
            let mut byte = [0];
            let got = match read_blocking(port, &mut byte, POLL) {
                Ok(got) => got,
                Err(e) => {
                    log::error!("Read failed: {e}");
                    return;
                }
            };
            let line = (got > 0).then(|| lines.take(byte[0])).flatten();
            // On a log only a sample is progress, as an entry that never reads
            // keeps being asked for; live, any byte says the meter is there.
            if got > 0 && self.data_source == 0 {
                progress = Instant::now();
            }
            let mut wait_for_more = got > 0;
            if let Some(line) = line {
                wait_for_more = false;
                let received = meter.receive(&line);
                if let Some(reading) = received.reading {
                    samples += 1;
                    progress = Instant::now();
                    if !sink.send_while(analog(reading), go_on) {
                        return;
                    }
                }
                if received.stop {
                    return;
                }
            }
            if self.limit_samples > 0 && samples >= self.limit_samples {
                return;
            }
            if self.limit_msec > 0 && started.elapsed() >= limit {
                return;
            }
            if only_samples_can_end && progress.elapsed() >= self.silence {
                log::error!(
                    "Acquisition failed: no answer in {} s, and only a sample limit can end this.",
                    self.silence.as_secs()
                );
                return;
            }
            if wait_for_more {
                continue;
            }
            if let Some(command) = meter.dispatch(Instant::now())
                && !send(port, &command)
            {
                return;
            }
        }
    }
}

fn analog(r: Reading) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![r.channel],
        samples: vec![r.value],
        mq: r.mq,
        flags: r.flags,
        unit: r.unit,
        digits: r.digits,
    })
}

/// `agdmm_send`, and whether the capture can go on. With 1 ms for a write, as C
/// has, a meter not taking bytes costs a job its time, and the next try goes
/// out then. Only a port that is gone ends it, where C tries for ever.
fn send(port: &mut dyn Port, data: &[u8]) -> bool {
    match port.write_timeout(data, WRITE_TIMEOUT) {
        Ok(n) => {
            if n < data.len() {
                log::error!("Failed to send.");
            }
            true
        }
        Err(e) => {
            log::debug!("{e}");
            log::error!("Failed to send.");
            false
        }
    }
}

/// Cuts a capture's bytes into lines. C stops reading for good once its buffer
/// is full with no LF (`protocol.c:188`), and no limit but time ends the run;
/// here that line is dropped and the next one read.
#[derive(Default)]
struct Lines {
    line: Vec<u8>,
    overlong: bool,
}

impl Lines {
    /// A byte, and the line it ends.
    fn take(&mut self, byte: u8) -> Option<Vec<u8>> {
        if byte == b'\n' {
            let line = std::mem::take(&mut self.line);
            return (!std::mem::take(&mut self.overlong)).then_some(line);
        }
        if !self.overlong {
            self.line.push(byte);
            if self.line.len() == LINE_MAX {
                log::warn!("A line of over {} bytes dropped.", LINE_MAX - 1);
                self.line.clear();
                self.overlong = true;
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit
    /// that does not bite must fail, not hang the run.
    const BOUND: Duration = Duration::from_secs(10);

    type Received = Receiver<(usize, Packet)>;
    type Answer = fn(usize, &[u8]) -> Vec<u8>;

    #[derive(Default)]
    struct Seen {
        writes: Vec<(Instant, Vec<u8>)>,
        purges: usize,
    }

    /// A meter: `answer` gets the number of the write and its bytes.
    struct Fake {
        seen: Arc<Mutex<Seen>>,
        input: VecDeque<u8>,
        answer: Answer,
        fail_writes_from: Option<usize>,
        fail_reads: bool,
        /// Writes of these numbers find no room.
        blocked_writes: std::ops::Range<usize>,
    }

    impl Fake {
        fn new(answer: Answer) -> (Fake, Arc<Mutex<Seen>>) {
            let seen = Arc::new(Mutex::new(Seen::default()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                answer,
                fail_writes_from: None,
                fail_reads: false,
                blocked_writes: 0..0,
            };
            (fake, seen)
        }
    }

    impl Port for Fake {
        fn name(&self) -> &'static str {
            "fake"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            if self.fail_reads {
                return Err(Error::device("read failed"));
            }
            if let Some(b) = self.input.pop_front() {
                buf[0] = b;
                return Ok(1);
            }
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
            Ok(0)
        }
        fn write(&mut self, _: &[u8]) -> Result<()> {
            panic!("the driver's writes have C's 1 ms");
        }
        fn write_timeout(&mut self, data: &[u8], timeout: Duration) -> Result<usize> {
            assert_eq!(timeout, Duration::from_millis(1), "C's time");
            let mut seen = self.seen.lock().unwrap();
            let n = seen.writes.len();
            if self.fail_writes_from.is_some_and(|from| n >= from) {
                return Err(Error::device("write failed"));
            }
            seen.writes.push((Instant::now(), data.to_vec()));
            if self.blocked_writes.contains(&n) {
                return Ok(0);
            }
            self.input.extend((self.answer)(n, data));
            Ok(data.len())
        }
        fn purge(&mut self) -> Result<()> {
            self.seen.lock().unwrap().purges += 1;
            self.input.clear();
            Ok(())
        }
    }

    /// A U1232A that answers all it knows.
    fn answers(_: usize, data: &[u8]) -> Vec<u8> {
        match data {
            b"CONF?\n\r\n" => b"\"V,0,DC\"\r\n".to_vec(),
            b"STAT?\n\r\n" => b"\"000000000000000000000\"\r\n".to_vec(),
            b"FETC?\n\r\n" => b"+1.50000000E+00\r\n".to_vec(),
            other if other.starts_with(b"LOG:") => b"\"00123450000000\"\r\n".to_vec(),
            _ => Vec::new(),
        }
    }

    fn silent(_: usize, _: &[u8]) -> Vec<u8> {
        Vec::new()
    }

    /// Answers `FETC?` with a line that is no value.
    fn no_value(n: usize, data: &[u8]) -> Vec<u8> {
        if data.starts_with(b"FETC") { b"junk\r\n".to_vec() } else { answers(n, data) }
    }

    fn bad_log(_: usize, _: &[u8]) -> Vec<u8> {
        b"\"14123450000000\"\r\n".to_vec()
    }

    fn log_of_two(_: usize, data: &[u8]) -> Vec<u8> {
        match data {
            b"LOG:HAND 1\n\r\n" | b"LOG:HAND 2\n\r\n" => b"\"00123450000000\"\r\n".to_vec(),
            _ => b"*E\r\n".to_vec(),
        }
    }

    /// One entry that does not read, then good ones.
    fn bad_then_good(n: usize, _: &[u8]) -> Vec<u8> {
        if n == 0 { b"\"14123450000000\"\r\n".to_vec() } else { b"\"00123450000000\"\r\n".to_vec() }
    }

    fn device(model: &str, fake: Fake) -> AgilentDevice {
        let mut d = AgilentDevice::from_port(model, Box::new(fake));
        d.open().unwrap();
        d
    }

    fn start(d: &mut AgilentDevice) -> Received {
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        rx
    }

    /// Up to the `End`. A capture that keeps sending past its limit fails here
    /// at the bound rather than running on.
    fn packets_to_end(rx: &Received) -> Vec<Packet> {
        let deadline = Instant::now() + BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(left) {
                Ok((_, Packet::End)) => return out,
                Ok((_, p)) => out.push(p),
                Err(e) => panic!("no End after {BOUND:?}: {e}; {} packets", out.len()),
            }
        }
    }

    fn values(packets: &[Packet]) -> usize {
        packets.iter().filter(|p| matches!(p, Packet::Analog(_))).count()
    }

    fn writes(seen: &Arc<Mutex<Seen>>) -> Vec<Vec<u8>> {
        seen.lock().unwrap().writes.iter().map(|w| w.1.clone()).collect()
    }

    /// Waits for a condition, or fails.
    fn wait_until(what: &str, done: impl Fn() -> bool) {
        let deadline = Instant::now() + BOUND;
        while !done() {
            assert!(Instant::now() < deadline, "{what}");
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    #[test]
    fn identify_asks_once_and_reads_a_line() {
        let (mut fake, seen) =
            Fake::new(|_, _| b"Agilent Technologies,U1232A,x,V1\r\nrest".to_vec());
        let identity = identify(&mut fake).unwrap();
        assert_eq!(identity.profile.name, "U1232A");
        assert_eq!(writes(&seen), [b"*IDN?\r\n".to_vec()]);
    }

    #[test]
    fn identify_takes_what_came_when_time_is_up() {
        let (mut fake, _) = Fake::new(|_, _| b"Agilent Technologies,U1232A,x,V1".to_vec());
        let start = Instant::now();
        assert!(identify(&mut fake).is_some());
        let took = start.elapsed();
        assert!(took >= Duration::from_millis(250), "{took:?}");
    }

    #[test]
    fn identify_reads_no_more_than_the_buffer() {
        let (mut fake, _) = Fake::new(|_, _| {
            let mut a = b"Agilent Technologies,U1232A,x,".to_vec();
            a.resize(200, b'v');
            a
        });
        let identity = identify(&mut fake).unwrap();
        assert_eq!(identity.version.len(), 127 - 30);
        assert_eq!(fake.input.len(), 200 - 127, "the rest is not read");
    }

    #[test]
    fn a_write_that_goes_short_at_the_scan_is_no_device() {
        let (mut fake, _) = Fake::new(|_, _| b"Agilent Technologies,U1232A,x,V1\r\n".to_vec());
        fake.blocked_writes = 0..1;
        assert!(identify(&mut fake).is_none());
        let (mut fake, _) = Fake::new(silent);
        fake.fail_writes_from = Some(0);
        assert!(identify(&mut fake).is_none());
    }

    #[test]
    fn an_empty_answer_is_no_device() {
        for answer in
            [(|_, _| b"\r\n".to_vec()) as Answer, |_, _| b"\0\r\n".to_vec(), |_, _| Vec::new()]
        {
            let (mut fake, _) = Fake::new(answer);
            assert!(identify(&mut fake).is_none());
        }
    }

    #[test]
    fn a_sample_limit_ends_it_and_the_port_comes_back() {
        let (fake, seen) = Fake::new(answers);
        let mut d = device("U1232A", fake);
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        let packets = packets_to_end(&rx);
        assert_eq!(values(&packets), 2);
        assert_eq!(
            writes(&seen),
            [
                b"CONF?\n\r\n".to_vec(),
                b"STAT?\n\r\n".to_vec(),
                b"FETC?\n\r\n".to_vec(),
                b"FETC?\n\r\n".to_vec()
            ]
        );
        assert!(d.port.lock().unwrap().is_some(), "End came after the port went back");
        d.close().unwrap();
    }

    #[test]
    fn a_time_limit_ends_it() {
        let (fake, _) = Fake::new(silent);
        let mut d = device("U1232A", fake);
        d.set(&Key::LimitMsec, &Value::U64(50), None).unwrap();
        let begun = Instant::now();
        let rx = start(&mut d);
        packets_to_end(&rx);
        let took = begun.elapsed();
        assert!(took >= Duration::from_millis(50), "{took:?}");
        assert!(took < Duration::from_secs(5), "{took:?}");
        d.close().unwrap();
    }

    #[test]
    fn fetc_keeps_its_rate() {
        let (fake, seen) = Fake::new(answers);
        let mut d = device("U1232A", fake);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        d.set(&Key::Samplerate, &Value::U64(20), None).unwrap();
        let rx = start(&mut d);
        packets_to_end(&rx);
        let seen = seen.lock().unwrap();
        let fetcs: Vec<Instant> =
            seen.writes.iter().filter(|w| w.1.starts_with(b"FETC")).map(|w| w.0).collect();
        assert_eq!(fetcs.len(), 3);
        for pair in fetcs.windows(2) {
            let gap = pair[1] - pair[0];
            assert!(gap >= Duration::from_millis(50), "{gap:?}");
        }
    }

    #[test]
    fn a_silent_meter_has_each_job_given_up_in_its_time() {
        let (fake, seen) = Fake::new(silent);
        let mut d = device("U1232A", fake);
        d.set(&Key::LimitMsec, &Value::U64(700), None).unwrap();
        let rx = start(&mut d);
        packets_to_end(&rx);
        let seen = seen.lock().unwrap();
        // CONF? at 0, STAT? a job's time on, FETC? another on.
        assert_eq!(seen.writes[0].1, b"CONF?\n\r\n");
        assert_eq!(seen.writes[1].1, b"STAT?\n\r\n");
        assert_eq!(seen.writes[2].1, b"FETC?\n\r\n");
        for pair in seen.writes.windows(2) {
            let gap = pair[1].0 - pair[0].0;
            assert!(gap > Duration::from_millis(300), "{gap:?}");
        }
    }

    #[test]
    fn a_write_that_goes_short_costs_that_job_its_time_and_the_capture_goes_on() {
        let (mut fake, seen) = Fake::new(answers);
        fake.blocked_writes = 0..1;
        let mut d = device("U1232A", fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&packets_to_end(&rx)), 1);
        let seen = seen.lock().unwrap();
        let gap = seen.writes[1].0 - seen.writes[0].0;
        assert!(gap > Duration::from_millis(300), "{gap:?}");
    }

    #[test]
    fn a_port_gone_ends_the_capture() {
        let (mut fake, _) = Fake::new(answers);
        fake.fail_writes_from = Some(1);
        let mut d = device("U1232A", fake);
        let rx = start(&mut d);
        assert_eq!(values(&packets_to_end(&rx)), 0);
        d.close().unwrap();
        let (mut fake, _) = Fake::new(answers);
        fake.fail_reads = true;
        let mut d = device("U1232A", fake);
        let rx = start(&mut d);
        assert_eq!(values(&packets_to_end(&rx)), 0);
    }

    /// `silence` is short in these, as they wait for it.
    fn quick(model: &str, fake: Fake, silence: Duration) -> AgilentDevice {
        let mut d = device(model, fake);
        d.silence = silence;
        d
    }

    #[test]
    fn the_time_on_a_silent_meter_is_ten_seconds() {
        assert_eq!(SILENCE, Duration::from_secs(10));
    }

    #[test]
    fn silence_ends_a_capture_only_a_sample_limit_can_end() {
        let (fake, _) = Fake::new(silent);
        let mut d = quick("U1232A", fake, Duration::from_millis(150));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let begun = Instant::now();
        let rx = start(&mut d);
        assert_eq!(values(&packets_to_end(&rx)), 0);
        let took = begun.elapsed();
        assert!(took >= Duration::from_millis(150), "{took:?}");
    }

    #[test]
    fn silence_does_not_end_a_capture_a_time_limit_will() {
        let (fake, _) = Fake::new(silent);
        let mut d = quick("U1232A", fake, Duration::from_millis(50));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        d.set(&Key::LimitMsec, &Value::U64(300), None).unwrap();
        let begun = Instant::now();
        let rx = start(&mut d);
        packets_to_end(&rx);
        let took = begun.elapsed();
        assert!(took >= Duration::from_millis(300), "{took:?}");
    }

    #[test]
    fn silence_does_not_end_a_capture_with_no_limit() {
        let (fake, seen) = Fake::new(silent);
        let mut d = quick("U1232A", fake, Duration::from_millis(20));
        let rx = start(&mut d);
        // Past the bound twice over: two jobs have gone out at least.
        wait_until("a second job", || seen.lock().unwrap().writes.len() >= 2);
        assert!(rx.try_iter().all(|(_, p)| !matches!(p, Packet::End)));
        d.stop().unwrap();
        assert_eq!(values(&packets_to_end(&rx)), 0);
    }

    #[test]
    fn a_meter_that_answers_with_no_value_is_waited_for() {
        let (fake, seen) = Fake::new(no_value);
        // An answer to each FETC? that is no value, every 300 ms, within this
        // bound; the fourth comes after it, so only those can have held it off.
        let mut d = quick("U1232A", fake, Duration::from_millis(500));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        wait_until("a fourth FETC?", || {
            seen.lock().unwrap().writes.iter().filter(|w| w.1.starts_with(b"FETC")).count() >= 4
        });
        assert!(rx.try_iter().all(|(_, p)| !matches!(p, Packet::End)));
        d.stop().unwrap();
        packets_to_end(&rx);
    }

    #[test]
    fn a_log_entry_that_never_reads_ends_at_the_bound() {
        let (fake, seen) = Fake::new(bad_log);
        let mut d = quick("U1282A", fake, Duration::from_millis(700));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        d.set(&Key::DataSource, &Value::Str("Log-Hand".to_owned()), None).unwrap();
        let begun = Instant::now();
        let rx = start(&mut d);
        assert_eq!(values(&packets_to_end(&rx)), 0);
        assert!(begun.elapsed() >= Duration::from_millis(700));
        let writes = writes(&seen);
        assert!(writes.len() >= 2);
        assert!(writes.iter().all(|w| w == b"LOG:HAND 1\n\r\n"), "the same index");
    }

    #[test]
    fn a_log_entry_read_after_one_that_did_not_counts_as_progress() {
        let (fake, _) = Fake::new(bad_then_good);
        let mut d = quick("U1282A", fake, Duration::from_millis(600));
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        d.set(&Key::DataSource, &Value::Str("Log-Auto".to_owned()), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&packets_to_end(&rx)), 3);
    }

    #[test]
    fn the_log_ends_at_its_error_with_no_limit() {
        let (fake, seen) = Fake::new(log_of_two);
        let mut d = device("U1282A", fake);
        d.set(&Key::DataSource, &Value::Str("Log-Hand".to_owned()), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&packets_to_end(&rx)), 2);
        assert_eq!(writes(&seen).len(), 3);
    }

    #[test]
    fn the_refusals_of_a_log() {
        let (fake, seen) = Fake::new(answers);
        let mut d = device("U1232A", fake);
        d.set(&Key::DataSource, &Value::Str("Log-Hand".to_owned()), None).unwrap();
        assert!(d.check_capture(None).is_err());
        let (tx, _rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
        assert_eq!(writes(&seen), Vec::<Vec<u8>>::new());
        let (fake, _) = Fake::new(answers);
        let mut d = device("U1282A", fake);
        d.set(&Key::DataSource, &Value::Str("Log-Hand".to_owned()), None).unwrap();
        assert!(d.check_capture(None).is_ok());
        d.enable_channel(0, false).unwrap();
        assert!(d.check_capture(None).is_err());
        d.enable_channel(0, true).unwrap();
        d.enable_channel(2, false).unwrap();
        assert!(d.check_capture(None).is_ok(), "only P1 counts");
    }

    #[test]
    fn a_stop_ends_it_with_data_on_offer() {
        let (fake, seen) = Fake::new(answers);
        let mut d = device("U1232A", fake);
        d.set(&Key::Samplerate, &Value::U64(20), None).unwrap();
        let rx = start(&mut d);
        wait_until("some FETC?s", || {
            seen.lock().unwrap().writes.iter().filter(|w| w.1.starts_with(b"FETC")).count() >= 3
        });
        d.stop().unwrap();
        packets_to_end(&rx);
        let after = writes(&seen).len();
        std::thread::sleep(Duration::from_millis(150));
        assert_eq!(writes(&seen).len(), after, "no more writes");
        d.close().unwrap();
    }

    #[test]
    fn a_stop_holds_with_a_full_queue_and_a_close_joins() {
        let (fake, _) = Fake::new(answers);
        let mut d = device("U1282A", fake);
        d.set(&Key::DataSource, &Value::Str("Log-Hand".to_owned()), None).unwrap();
        // A queue of one, left unread: the worker is soon stuck on a send.
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_until("the queue to fill", || rx.try_recv().is_ok());
        std::thread::sleep(Duration::from_millis(20));
        // On a thread, so that a close the queue holds fails here, not hangs.
        let (done_tx, done_rx) = sync_channel(1);
        std::thread::spawn(move || {
            d.close().unwrap();
            let gone = d.port.lock().unwrap().is_none();
            done_tx.send(gone).unwrap();
        });
        assert!(done_rx.recv_timeout(BOUND).expect("close held by the queue"));
        drop(rx);
    }

    #[test]
    fn a_line_past_the_buffer_is_dropped_and_the_next_is_read() {
        let mut lines = Lines::default();
        for _ in 0..254 {
            assert_eq!(lines.take(b'x'), None);
        }
        assert_eq!(lines.take(b'\n').map(|l| l.len()), Some(254), "254 and its LF fit");
        for _ in 0..255 {
            assert_eq!(lines.take(b'x'), None);
        }
        assert_eq!(lines.line, Vec::<u8>::new());
        assert_eq!(lines.take(b'x'), None);
        assert_eq!(lines.take(b'\n'), None, "that line is dropped");
        assert_eq!(lines.take(b'a'), None);
        assert_eq!(lines.take(b'\n'), Some(b"a".to_vec()));
    }

    #[test]
    fn options_and_limits_as_c_has_them() {
        let (fake, _) = Fake::new(silent);
        let mut d = device("U1232A", fake);
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(5));
        assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::Str("Live".to_owned()));
        for bad in [0, 21] {
            assert!(d.set(&Key::Samplerate, &Value::U64(bad), None).is_err());
        }
        for good in [1, 20] {
            d.set(&Key::Samplerate, &Value::U64(good), None).unwrap();
            assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(good));
        }
        assert!(d.set(&Key::DataSource, &Value::Str("x".to_owned()), None).is_err());
        assert!(d.get(&Key::Conn, None).is_err());
        assert!(d.set(&Key::Conn, &Value::Str("x".to_owned()), None).is_err());
        assert!(d.list(&Key::Conn, None).is_err());
        assert!(d.get(&Key::Samplerate, Some("g")).is_err());
    }

    #[test]
    fn the_channels_are_the_models() {
        for (model, names) in
            [("U1232A", &["P1"][..]), ("U1242C", &["P1", "P2"]), ("U1282A", &["P1", "P2", "P3"])]
        {
            let (fake, _) = Fake::new(silent);
            let d = device(model, fake);
            let got: Vec<&str> = d.channels().iter().map(|c| c.name.as_str()).collect();
            assert_eq!(got, names);
        }
    }

    #[test]
    fn an_open_purges_a_given_port() {
        let (fake, seen) = Fake::new(silent);
        let _d = device("U1232A", fake);
        assert_eq!(seen.lock().unwrap().purges, 1);
    }
}
