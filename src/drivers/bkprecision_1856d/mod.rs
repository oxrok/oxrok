// Counterpart in sigrok:
//   libsigrok/src/hardware/bkprecision-1856d/api.c,
//   libsigrok/src/hardware/bkprecision-1856d/protocol.c,
//   Copyright (C) 2021 LUMERIIX,
//   Copyright (C) 2024 Daniel Anselmi <danselmi@gmx.ch>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The B&K Precision 1856D frequency counter on a serial port: the driver
//! `bkprecision-1856d`.
//!
//! `conn=` names the port, and a scan does not open it: anything named is a
//! device, as in C. Settings stay here until a capture sends them.
//!
//! A capture sends the input, the gate (and sleeps), and a request; each reading
//! sends the gate and a request again, so a capture is a round per reading. It
//! ends at `limit_samples` or a stop. A setting during a capture takes effect at
//! the next round, as in C.
//!
//! Where C does not do what it means:
//!
//! - After an expiry C's next 100 ms run from before the sleep after the gate:
//!   `fd_source_check` asks for the time (`session.c:108-109`), glib keeps it for
//!   that iteration, and `:134` adds to it after the callback. That leaves an answer
//!   60 ms after `G0`, 20 ms after `G1` and `G2`, which at 9600 baud has 15.6 ms of
//!   line in it, and none after `G3`, where `G3` and `D0` go again at once ("lib an
//!   expiry at gate 10s"). Here the 100 ms always run from the request, as they
//!   do in C after a reading. A choice: C still gets the reading.
//! - An answer out of step does not come back in step on a line that delivers
//!   bytes as they come. C flushes before the sleep after a wrong end of message
//!   (`protocol.c:245`): the tail arrives in that sleep, starts the next answer,
//!   and the next is wrong too. Here it flushes again after the sleep, and also
//!   ahead of the first request and of the one after a reading, which stops a late
//!   answer staying ahead of every later one, as it does in C. No flush follows an
//!   expiry: a device that takes over 100 ms would then never get a reading in.
//! - After an input changes, or at a restart, the next request waits until the last
//!   one has had its 100 ms, and then flushes. That drops an old input's answer
//!   from a device that answers in that time. One that answers later and queues
//!   requests still gives old readings: nothing tells them apart, and how many goes
//!   by its delay. Whether a real counter's first `D0` after `F` shows the old
//!   display is unknown.
//! - With only a sample limit to end the capture, no usable answer for
//!   [`silence_bound`] ends it, and failed writes after [`MAX_FAILED_WRITES`] end any
//!   capture: C goes on for ever. Both are failed captures, which exit 1. A device
//!   with nothing to show is not silent: it says so.
//!
//! The reading is a float here ([`AnalogData`]) where C's packet has the double. At
//! 3.7 GHz a float's step is 256 Hz, where the panel's is 100 Hz at a 10 s gate on the
//! internal time base (table 2), 10 Hz on an external one. The printed value is off
//! too, in both tools: `3700.0001MHz` prints `3.6999998 GHz`, from the output's
//! float arithmetic. A library client of C gets the double; here it does not.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{self, Params, Port};
use protocol::{COMMAND_SIZE, DATA_REQUEST, Frame, GATES, Input, MSG_SIZE, SERIALCOMM};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::FrequencyCounter];

/// `devopts`.
static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::GateTime, Access::GET_SET_LIST),
    OptionDesc::new(Key::LimitSamples, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
];

/// The session source's timeout in `dev_acquisition_start`: how long an answer
/// has, from the request or the last byte.
pub const ANSWER_TIMEOUT: Duration = Duration::from_millis(100);
/// Writes that fail in a row, with an error, that end a capture: the port has gone.
pub const MAX_FAILED_WRITES: u32 = 4;
/// The least time a device with no usable answer is waited for, when only a sample
/// limit could end the capture.
const SILENCE_BOUND_LEAST: Duration = Duration::from_secs(5);
/// How long the End waits for room in the queue once a stop has come. A client
/// that stops drains (`Session` does); one that does not must not hold up a close.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How often a wait looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(10);
/// The pause after a read that gave nothing early: a hung up tty gives one at once.
const EMPTY_READ_PAUSE: Duration = Duration::from_millis(1);

/// How long a capture with only a sample limit waits for a device that gives no
/// answer it can use: three gates, nothing being known of how a gate holds up an
/// answer, and at least `least`, which is [`SILENCE_BOUND_LEAST`] but in a test.
pub fn silence_bound(gate: usize, least: Duration) -> Duration {
    let (p, q) = GATES[gate].period;
    least.max(Duration::from_millis(3000 * p / q))
}

/// The `bkprecision-1856d` driver.
pub struct Bkprecision1856dDriver;

impl Driver for Bkprecision1856dDriver {
    fn id(&self) -> &'static str {
        "bkprecision-1856d"
    }

    fn long_name(&self) -> &'static str {
        "B&K Precision 1856D"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let conn = options.iter().rev().find_map(|(key, value)| match (key, value) {
            (Key::Conn, Value::Str(conn)) => Some(conn.clone()),
            _ => None,
        });
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        Ok(vec![Box::new(Bkprecision1856d::new(conn))])
    }
}

/// What a client may change while a capture runs, as C's `devc` is read at each step.
struct Settings {
    gate: AtomicUsize,
    /// 0 is A.
    input: AtomicUsize,
    limit_samples: AtomicU64,
    /// When the port last had a request, in this capture or the one before.
    last_request: Mutex<Option<Instant>>,
}

impl Settings {
    fn input(&self) -> Input {
        if self.input.load(Ordering::SeqCst) == 0 { Input::A } else { Input::C }
    }

    fn gate(&self) -> usize {
        self.gate.load(Ordering::SeqCst)
    }
}

/// One counter.
pub struct Bkprecision1856d {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    settings: Arc<Settings>,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// The open port. A capture has it from its start until just ahead of its End.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    worker: Option<Worker>,
    /// [`SILENCE_BOUND_LEAST`], unless a test wants a shorter one.
    least_silence: Duration,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl Bkprecision1856d {
    fn new(conn: String) -> Self {
        Bkprecision1856d {
            info: DeviceInfo {
                driver: "bkprecision-1856d",
                vendor: "BK Precision".to_owned(),
                model: "bk-1856d".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "P1".to_owned())],
            conn,
            params: Params::parse(SERIALCOMM).expect("the default parses"),
            settings: Arc::new(Settings {
                gate: AtomicUsize::new(0),
                input: AtomicUsize::new(1),
                limit_samples: AtomicU64::new(0),
                last_request: Mutex::new(None),
            }),
            pending: None,
            port: Arc::new(Mutex::new(None)),
            worker: None,
            least_silence: SILENCE_BOUND_LEAST,
        }
    }

    /// A device on a port that stands in for the hardware. [`Device::open`] takes it.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let mut device = Self::new(port.name().to_owned());
        device.pending = Some(port);
        device
    }

    fn join_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for Bkprecision1856d {
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

    fn options(&self, _group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(DEV_OPTIONS.to_vec())
    }

    /// `config_get`.
    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        match key {
            Key::GateTime => {
                let (p, q) = GATES[self.settings.gate()].period;
                Ok(Value::Rational(p, q))
            }
            Key::LimitSamples => Ok(Value::U64(self.settings.limit_samples.load(Ordering::SeqCst))),
            Key::DataSource => Ok(Value::from(self.settings.input().name())),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `config_set`: all of it stays here until the next round's commands.
    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::GateTime => {
                let Value::Rational(p, q) = value else {
                    return Err(bad());
                };
                let gate = protocol::gate_of(*p, *q)
                    .ok_or_else(|| Error::NotSupported(format!("{p}/{q} s is no gate time")))?;
                self.settings.gate.store(gate, Ordering::SeqCst);
                Ok(())
            }
            Key::LimitSamples => {
                // `core::config_set` refuses a 0 and says so; this is for a library
                // caller that comes straight here.
                let limit = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                self.settings.limit_samples.store(limit, Ordering::SeqCst);
                Ok(())
            }
            Key::DataSource => {
                let index = match value {
                    Value::Str(name) if name == "A" => 0,
                    Value::Str(name) if name == "C" => 1,
                    _ => return Err(bad()),
                };
                self.settings.input.store(index, Ordering::SeqCst);
                Ok(())
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `config_list`.
    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        match key {
            Key::GateTime => Ok(ValueList::Rational(GATES.iter().map(|g| g.period).collect())),
            Key::DataSource => {
                Ok(ValueList::Str(vec![Input::A.name().to_owned(), Input::C.name().to_owned()]))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `std_serial_dev_open`.
    fn open(&mut self) -> Result<()> {
        let mut slot = self.port.lock().expect("port lock");
        if slot.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => serial::open(&self.conn, self.params)?,
        };
        *slot = Some(port);
        *self.settings.last_request.lock().expect("settings lock") = None;
        Ok(())
    }

    /// `std_serial_dev_close`.
    fn close(&mut self) -> Result<()> {
        self.join_worker();
        *self.port.lock().expect("port lock") = None;
        Ok(())
    }

    /// `dev_acquisition_start`: the thread, whose Header goes first.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        // The port is back ahead of the End, so its being here says a capture is
        // over: a finished thread can say otherwise for a moment after the End.
        let port = self.port.lock().expect("port lock").take();
        if self.worker.is_some() {
            if port.is_none() {
                return Err(Error::Busy);
            }
            // Its End may be stuck on a full queue: the stop gives that up.
            self.join_worker();
        }
        let Some(port) = port else {
            return Err(Error::device("the device is not open"));
        };
        let stop = Arc::new(AtomicBool::new(false));
        let mut capture = Capture {
            port: Some(port),
            params: self.params,
            settings: Arc::clone(&self.settings),
            stop: Arc::clone(&stop),
            sink,
            selected: self.settings.input(),
            failed_writes: 0,
            least_silence: self.least_silence,
            samples: 0,
            last_answer: Instant::now(),
        };
        let slot = Arc::clone(&self.port);
        let handle = std::thread::Builder::new()
            .name("bkprecision-1856d acquisition".to_owned())
            .spawn(move || {
                let outcome = capture.run();
                *slot.lock().expect("port lock") = capture.port.take();
                capture.send_end(outcome);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// `std_serial_dev_acquisition_stop`. Asks the thread to end, and does not wait:
    /// it may be sending.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Drop for Bkprecision1856d {
    fn drop(&mut self) {
        self.join_worker();
    }
}

/// One capture, on its own thread: `bkprecision_1856d_receive_data` and what it calls.
struct Capture {
    /// Taken only when the capture is over.
    port: Option<Box<dyn Port>>,
    params: Params,
    settings: Arc<Settings>,
    stop: Arc<AtomicBool>,
    sink: PacketSink,
    /// `curr_sel_input`.
    selected: Input,
    failed_writes: u32,
    least_silence: Duration,
    samples: u64,
    /// The last answer that says the device is there and working.
    last_answer: Instant,
}

impl Capture {
    fn port(&mut self) -> &mut dyn Port {
        self.port.as_deref_mut().expect("the capture has the port")
    }

    fn stopped(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    fn send(&self, packet: Packet) -> bool {
        self.sink.send_while(packet, || !self.stopped())
    }

    /// The End, for which a client that goes on reading is waited for without a
    /// bound, and after a stop only for [`END_AFTER_STOP`].
    fn send_end(&self, outcome: Result<()>) {
        let stopped_at = std::cell::Cell::new(None);
        self.sink.end_while(outcome, || {
            if !self.stopped() {
                return true;
            }
            let at = stopped_at.get().unwrap_or_else(Instant::now);
            stopped_at.set(Some(at));
            at.elapsed() < END_AFTER_STOP
        });
    }

    /// `serial_write_blocking`, and C's message if nothing went. A write that times
    /// out is what a device that stops reading gets, so only an error counts.
    fn write(&mut self, command: [u8; 3], complaint: &str) {
        let time = serial::serial_timeout(&self.params, COMMAND_SIZE);
        match self.port().write_timeout(&command, time) {
            Ok(n) => {
                self.failed_writes = 0;
                if n < 1 {
                    log::error!("{complaint}");
                }
            }
            Err(e) => {
                self.failed_writes += 1;
                log::debug!("Write failed: {e}.");
                log::error!("{complaint}");
            }
        }
    }

    fn flush(&mut self) {
        if let Err(e) = self.port().purge() {
            log::debug!("Flush failed: {e}.");
        }
    }

    /// `bkprecision_1856d_send_input_sel`.
    fn send_input(&mut self) {
        let input = self.settings.input();
        log::trace!("selecting input {}", input.name());
        self.write(*input.command(), &format!("unable to send function {} command", input.name()));
        self.selected = input;
    }

    /// `bkprecision_1856d_send_gate_time`: false when a stop came in the sleep.
    fn send_gate(&mut self) -> bool {
        let gate = &GATES[self.settings.gate()];
        log::info!("{}", gate.info);
        self.write(*gate.command, "unable to send gate time command");
        self.wait(gate.sleep)
    }

    /// `bkprecision_1856d_request_data`.
    fn send_request(&mut self) {
        log::trace!("requesting data");
        *self.settings.last_request.lock().expect("settings lock") = Some(Instant::now());
        self.write(*DATA_REQUEST, "unable to send request data command");
    }

    fn input_changed(&self) -> bool {
        self.settings.input() != self.selected
    }

    /// Waits until the last request has had its [`ANSWER_TIMEOUT`], so a flush after
    /// this drops an answer of the old input from a device that takes less than that.
    /// False when a stop came first.
    fn wait_out_answer(&self) -> bool {
        let last = *self.settings.last_request.lock().expect("settings lock");
        last.is_none_or(|at| self.wait(ANSWER_TIMEOUT.saturating_sub(at.elapsed())))
    }

    /// Waits for `time`: false when a stop came first.
    fn wait(&self, time: Duration) -> bool {
        let due = Instant::now() + time;
        loop {
            if self.stopped() {
                return false;
            }
            let left = due.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return true;
            }
            std::thread::sleep(left.min(STOP_CHECK));
        }
    }

    fn analog(hz: f64, digits: i32) -> Packet {
        Packet::Analog(AnalogData {
            channels: vec![0],
            samples: vec![hz as f32],
            mq: Mq::Frequency,
            flags: MqFlags::empty(),
            unit: Unit::Hertz,
            digits,
        })
    }

    /// The capture: `Ok` for a limit met or a stop, or nobody listening any more, and
    /// the error for the two ways it gives up.
    fn run(&mut self) -> Result<()> {
        if !self.send(Packet::header()) {
            return Ok(());
        }
        self.flush();
        self.send_input();
        if !self.send_gate() {
            return Ok(());
        }
        // What the stop of an earlier capture left on the way, of whichever input.
        if !self.wait_out_answer() {
            return Ok(());
        }
        self.flush();
        self.send_request();

        let mut buffer = [0u8; MSG_SIZE];
        let mut level = 0;
        // The answer's 100 ms run from here, and from each byte.
        let mut waiting_since = Instant::now();
        self.last_answer = waiting_since;
        loop {
            if self.stopped() {
                return Ok(());
            }
            if self.failed_writes >= MAX_FAILED_WRITES {
                let what = format!("{} commands could not go out", self.failed_writes);
                log::error!("Acquisition failed: {what}.");
                return Err(Error::device(what));
            }
            // Whether time is up is asked ahead of a read and acted on after it, so an
            // answer that lies ready wins however late this looks: `fd_source_check`
            // tests `revents` first. A stall outside the wait must not cost a request.
            let left = ANSWER_TIMEOUT.saturating_sub(waiting_since.elapsed());
            let bound = silence_bound(self.settings.gate(), self.least_silence);
            let silent = self.settings.limit_samples.load(Ordering::SeqCst) > 0
                && self.last_answer.elapsed() >= bound;
            // A read of under a millisecond may wait for nothing or for ever.
            let wait = left.min(STOP_CHECK).max(Duration::from_millis(1));
            let got = match self.port().read(&mut buffer[level..], wait) {
                Ok(got) => got,
                Err(e) => {
                    log::debug!("Read failed: {e}.");
                    0
                }
            };
            if got > 0 {
                level += got;
                waiting_since = Instant::now();
                if level == MSG_SIZE {
                    level = 0;
                    if !self.answer(&buffer) {
                        return Ok(());
                    }
                    waiting_since = Instant::now();
                }
                continue;
            }
            if silent {
                log::error!("No usable answer from the device for {bound:?}.");
                return Err(Error::device(format!(
                    "no usable answer from the device for {bound:?}"
                )));
            }
            if left.is_zero() {
                // "in rare cases the bk1856 doesn't respond anymore": a part of an
                // answer stays, and its next bytes say it is out of step.
                if !self.send_gate() {
                    return Ok(());
                }
                self.send_request();
                waiting_since = Instant::now();
                continue;
            }
            std::thread::sleep(EMPTY_READ_PAUSE);
        }
    }

    /// What `bkprecision_1856d_parse_message` does after its checks: false when the
    /// capture is over.
    fn answer(&mut self, buffer: &[u8; MSG_SIZE]) -> bool {
        match protocol::parse(buffer) {
            Frame::NoCr => {
                log::error!("expected cr at end of message.");
                self.flush();
                let changed = self.input_changed();
                self.send_input();
                if !self.send_gate() || (changed && !self.wait_out_answer()) {
                    return false;
                }
                // The tail of what was out of step came in that sleep.
                self.flush();
                self.send_request();
            }
            Frame::Empty => {
                log::trace!("received an empty packet");
                self.last_answer = Instant::now();
                self.send_request();
            }
            Frame::NotFrequency => {
                log::error!("not a frequency returned");
                let changed = self.input_changed();
                self.send_input();
                if !self.send_gate() {
                    return false;
                }
                if changed {
                    if !self.wait_out_answer() {
                        return false;
                    }
                    self.flush();
                }
                self.send_request();
            }
            Frame::Reading { hz, digits } => {
                if !self.send(Self::analog(hz, digits)) {
                    return false;
                }
                // After the send: a client that is slow to take it is no silent device.
                self.last_answer = Instant::now();
                self.samples += 1;
                let limit = self.settings.limit_samples.load(Ordering::SeqCst);
                if limit > 0 && self.samples >= limit {
                    return false;
                }
                // `bkprecision_1856d_chk_select_input`.
                let changed = self.input_changed();
                if changed {
                    self.send_input();
                }
                if !self.send_gate() || (changed && !self.wait_out_answer()) {
                    return false;
                }
                // What answered an earlier request came while this one was due.
                self.flush();
                self.send_request();
            }
        }
        true
    }
}

#[cfg(test)]
mod tests;
