// Counterpart in sigrok:
//   libsigrok/src/hardware/center-3xx/api.c,
//   libsigrok/src/hardware/center-3xx/protocol.c,
//   Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Martin Ling,
//   Soeren Apel, Daniel Elstner, Gerhard Sittig, Wolfram Sang,
//   Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

// This file is generated with its header by tools/attribution.py.

//! Center 309 four channel thermometers, sold by Voltcraft as the K204, on a serial
//! port at 9600 baud. C registers two drivers from one directory, `center-309` and
//! `voltcraft-k204`, which differ only in the names they give a meter, and so does
//! this.
//!
//! A scan opens the port and closes it: nothing goes on the wire, so whatever opens
//! is a thermometer. An open opens it again.
//!
//! A capture sends `A` 500 ms after the start, and again 500 ms after the last bytes
//! once a whole packet has come. A packet holds all four channels, and counts as
//! one sample. A sample goes out once the line has been quiet for 25 ms, and the
//! limit is looked at after that, so a burst holding two packets sends both. `--get`
//! has nothing to answer: C has no `config_get`.
//!
//! Where it goes its own way, see `docs/DEVICES.md`. Jakob Unterwurzacher's pull
//! requests 300 and 302 against libsigrok, from a K204 on his desk, report the first
//! and the fifth, and are in no stock C:
//!
//! - C sends `A` once and waits for a whole packet, with no bound. So a reply lost,
//!   part of one, or one stray byte between two replies, ends its polling for good,
//!   and a `--samples` capture never ends. Here a request expires after 2 s, see
//!   [`Requests`].
//! - A capture with no sample says so at 10 s, whatever ends it. A `--samples`
//!   capture that has had no sample at all fails there, saying whether anything
//!   came; one that has had a sample goes on, and fails only after 10 minutes, as the
//!   rest of a long log is worth a warning. See [`Silence`].
//! - C's flag for that lives in a `static` (`protocol.c:218`), one for all devices
//!   and all captures of a process. So a capture that ends with `A` out leaves the
//!   next with no request. Here each capture knows what is out on its own link.
//! - What has come of a line is dropped ahead of a request, so bytes left by the
//!   last reply are not read as the head of the next. C keeps them.
//! - Only bytes 0 and 44 check a packet, so a stray `0x02` ahead of a reply whose
//!   byte 43 is 3 makes a window one byte out, with wrong temperatures. Where two
//!   valid windows overlap the last is taken, see [`protocol::Buffer`]. A burst that
//!   is no multiple of 45 bytes gets a warning: a byte put in the middle of a reply
//!   can still pass, and bit flips cannot be told at all, as there is no checksum.
//! - In MAX, MIN or REL mode the document moves the values to other bytes, and C
//!   always reads 7 to 14. So does this, and it warns once per capture.
//! - A reply a capture left on its way is no sample of the next one on an open
//!   device. C takes it as a reading.
//! - A port gone ends the capture, which C would go on calling back.
//! - `-C` is honoured; a sample still counts. C sends all four whatever it says.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port, serial_timeout};
use protocol::{Buffer, CHANNEL_NAMES, PACKET_SIZE, REQUEST, Reading, SERIALCOMM};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Thermometer];
const SET: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, SET),
    OptionDesc::new(Key::LimitMsec, SET),
];

/// The session source's timeout in `dev_acquisition_start`.
const PASS: Duration = Duration::from_millis(500);
/// How long a request stays out before the next.
const REQUEST_WAIT: Duration = Duration::from_secs(2);
/// The longest a slow meter is left to answer what it has before the next request.
const MAX_HOLD: Duration = Duration::from_secs(30);
/// The most requests counted as out: a meter that is off has lost the rest.
const MOST_OUT: u32 = 2;
/// How long a capture goes with no sample before it says so; and with only a sample
/// limit and no sample yet, before it fails.
const SILENCE_BOUND: Duration = Duration::from_secs(10);
/// How long a capture with only a sample limit goes on, once it has had a sample.
const OUTAGE_BOUND: Duration = Duration::from_secs(600);
/// The quiet that ends a burst: over the 16 ms an FTDI cable may hold its last bytes.
const SETTLE: Duration = Duration::from_millis(25);
/// How much longer than that a stream with no quiet can hold a sample up.
const MOST_UNSETTLED: u32 = 4;
/// How often a wait looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// A driver of this directory: C's table of `center_devs`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Brand {
    Center309,
    VoltcraftK204,
}

impl Brand {
    fn driver(self) -> &'static str {
        match self {
            Brand::Center309 => "center-309",
            Brand::VoltcraftK204 => "voltcraft-k204",
        }
    }

    fn long_name(self) -> &'static str {
        match self {
            Brand::Center309 => "Center 309",
            Brand::VoltcraftK204 => "Voltcraft K204",
        }
    }

    fn vendor(self) -> &'static str {
        match self {
            Brand::Center309 => "Center",
            Brand::VoltcraftK204 => "Voltcraft",
        }
    }

    fn model(self) -> &'static str {
        match self {
            Brand::Center309 => "309",
            Brand::VoltcraftK204 => "K204",
        }
    }
}

/// The `center-309` driver.
pub struct Center309Driver;
/// The `voltcraft-k204` driver.
pub struct VoltcraftK204Driver;

fn scan(brand: Brand, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
    let mut conn = None;
    let mut serialcomm = SERIALCOMM.to_owned();
    for (key, value) in options {
        match key {
            Key::Conn => conn = value.as_str().map(str::to_owned),
            Key::Serialcomm => {
                value.as_str().ok_or_else(|| Error::arg("serialcomm"))?.clone_into(&mut serialcomm);
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
    }
    let Some(conn) = conn else {
        return Ok(Vec::new());
    };
    let opened = Params::parse(&serialcomm)
        .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
    let params = match opened {
        Ok((port, params)) => {
            drop(port);
            params
        }
        Err(e) => {
            log::error!("{e}");
            return Ok(Vec::new());
        }
    };
    log::info!("Found device on port {conn}.");
    Ok(vec![Box::new(CenterDevice::new(brand, conn, params))])
}

/// The port, and what stays from one capture to the next, as in C's `dev_context`.
struct Link {
    port: Option<Box<dyn Port>>,
    params: Params,
    buffer: Buffer,
    /// Requests sent and not answered when a capture ended.
    out: u32,
}

/// One thermometer.
pub struct CenterDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    limit_samples: u64,
    limit_msec: u64,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
    capturing: Arc<AtomicBool>,
    pass: Duration,
    request_wait: Duration,
    silence_bound: Duration,
    outage_bound: Duration,
    settle: Duration,
    end_after_stop: Duration,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl CenterDevice {
    fn new(brand: Brand, conn: String, params: Params) -> Self {
        let channels = CHANNEL_NAMES
            .iter()
            .enumerate()
            .map(|(i, name)| Channel::new(i, ChannelKind::Analog, (*name).to_owned()))
            .collect();
        let link = Link { port: None, params, buffer: Buffer::default(), out: 0 };
        CenterDevice {
            info: DeviceInfo {
                driver: brand.driver(),
                vendor: brand.vendor().to_owned(),
                model: brand.model().to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels,
            conn,
            limit_samples: 0,
            limit_msec: 0,
            pending: None,
            link: Arc::new(Mutex::new(link)),
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            pass: PASS,
            request_wait: REQUEST_WAIT,
            silence_bound: SILENCE_BOUND,
            outage_bound: OUTAGE_BOUND,
            settle: SETTLE,
            end_after_stop: END_AFTER_STOP,
        }
    }

    /// A device on a port that stands in for the hardware, without a scan.
    /// [`Device::open`] takes it.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device = Self::new(Brand::Center309, conn, params);
        device.pending = Some(port);
        device
    }

    /// Sets the old worker's stop and joins it: an End stuck on a queue no one
    /// reads gives up within its bound.
    fn end_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for CenterDevice {
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

    /// C has no `config_get`.
    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        // `core::config_set` refuses a 0 and says so; this is for a library
        // caller that comes straight here.
        let positive = || {
            value
                .as_u64()
                .filter(|n| *n > 0)
                .ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())))
        };
        match key {
            Key::LimitSamples => self.limit_samples = positive()?,
            Key::LimitMsec => self.limit_msec = positive()?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    /// `std_serial_dev_open`: the port and nothing on the wire. The open flushes,
    /// so nothing of an earlier open is left to wait for.
    fn open(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        if link.port.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(port) => port,
            None => crate::serial::open(&self.conn, link.params)?,
        };
        link.port = Some(port);
        link.buffer.clear();
        link.out = 0;
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.end_worker();
        self.link.lock().expect("link lock").port = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.end_worker();
        if self.link.lock().expect("link lock").port.is_none() {
            return Err(Error::device("device is not open"));
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            enabled: std::array::from_fn(|i| self.channels[i].enabled),
            pass: self.pass,
            request_wait: self.request_wait,
            silence_bound: self.silence_bound,
            outage_bound: self.outage_bound,
            settle: self.settle,
        };
        let end_after_stop = self.end_after_stop;
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new()
            .name(format!("{} acquisition", self.info.driver))
            .spawn(move || {
                let outcome = capture.run(&sink, &stop2);
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2, outcome, end_after_stop);
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

    /// Asks the thread to end, and does not wait: it may be sending. Nothing goes
    /// to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// `End`, waiting for room in the queue for as long as nobody asked to stop, and
/// after a stop only for `after_stop`: a close joins this thread.
fn send_end(sink: &PacketSink, stop: &AtomicBool, outcome: Result<()>, after_stop: Duration) {
    let stopped_at = std::cell::Cell::new(None);
    sink.end_while(outcome, || {
        if !stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = stopped_at.get().unwrap_or_else(Instant::now);
        stopped_at.set(Some(at));
        at.elapsed() < after_stop
    });
}

/// What is to be done about requests, in a pass with no bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Turn {
    /// Nothing: a reply is due, or a slow meter still has replies to give.
    Wait,
    /// All were answered: ask.
    Ask,
    /// The last went unanswered for too long: ask anyway.
    Expired,
}

/// C's `request_new_packet`, with a count, since no reply says what it answers.
///
/// A packet leaves the next request to the next pass with no bytes, as in C. C has
/// no expiry, and here a request has one, of 2 s, for as long as nothing is heard: the
/// reply may be lost, or the meter off for a moment, and the first request it hears
/// should bring it back. The K204's owner reports that a unit ignores a repeated `A`,
/// so this costs nothing there.
///
/// A meter that queued every request and took longer than that to answer would have
/// them pile up. So a reply counted from the older of two requests out, or older than
/// the wait, starts a hold: for half as long again as the meter seems to take, from the
/// last packet, the next request waits. That is the gap since the last packet if no
/// request went between them, or the reply's age if one did; with more, its age, or the
/// gap at the second such reply in a row. Replies still queued then come with no more
/// going in, each starting it again. A hold running out with nothing more says one was lost, and takes
/// it off. A reply soon after a request may be one of those queued, so it ends a hold
/// only if the next request's reply also comes soon, one pass on. On a unit that
/// ignores repeats, a lost reply so costs a wait, one hold of about 3 s, and one pass.
/// Silence halves a hold, so a meter that goes off is soon asked every 2 s again. A
/// cable that flaps, a slow reply on each side of a gap, costs one hold of at most 30 s.
/// See the simulation in the tests, over latencies and both kinds of meter.
/// What the last reply said of the meter's pace.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LastReply {
    /// Nothing against it.
    InTime,
    /// It was a slow one.
    Slow,
    /// It came soon after its request, in a hold: the next request tries whether the
    /// hold can end.
    Unproven,
}

#[derive(Debug)]
struct Requests {
    out: u32,
    sent_at: Option<Instant>,
    /// The one before, while two are out.
    sent_before: Option<Instant>,
    first_wait: Duration,
    /// A packet came after the last request.
    heard: bool,
    /// How long after the last packet the next request waits, for a slow meter.
    hold: Duration,
    last_reply: LastReply,
    /// Requests since the last packet.
    asks: u32,
    last_packet: Option<Instant>,
    /// Packets still to come that answer requests of the capture before.
    to_drop: u32,
    /// A request of this capture expired, so more replies may come than C asks for.
    expired: bool,
}

impl Requests {
    /// As a capture finds them: what is left of the last one is dropped as it
    /// comes, for the time of one request from now, however old it is.
    fn new(left: u32, first_wait: Duration, now: Instant) -> Requests {
        Requests {
            out: left,
            sent_at: (left > 0).then_some(now),
            sent_before: None,
            first_wait,
            heard: false,
            hold: Duration::ZERO,
            last_reply: LastReply::InTime,
            asks: 0,
            last_packet: None,
            to_drop: left,
            expired: false,
        }
    }

    /// A packet came: whether it is to be dropped.
    fn packet(&mut self, now: Instant) -> bool {
        // In turn, if replies are: the oldest out.
        let asked = self.sent_before.take().or(self.sent_at);
        let age = asked.map_or(Duration::ZERO, |at| now.saturating_duration_since(at));
        self.out = self.out.saturating_sub(1);
        self.heard = true;
        let gap = self.last_packet.map_or(age, |at| now.saturating_duration_since(at));
        if age > self.first_wait {
            let takes = match self.asks {
                0 => gap,
                1 => age,
                _ if self.last_reply == LastReply::Slow => gap,
                _ => age.min(gap),
            };
            self.hold = (takes * 3 / 2).min(MAX_HOLD);
            self.last_reply = LastReply::Slow;
        } else if gap > self.first_wait && self.hold > Duration::ZERO {
            self.last_reply = LastReply::Unproven;
        } else {
            self.hold = if gap <= self.first_wait { Duration::ZERO } else { self.hold };
            self.last_reply = LastReply::InTime;
        }
        self.asks = 0;
        self.last_packet = Some(now);
        if self.to_drop > 0 {
            self.to_drop -= 1;
            return true;
        }
        false
    }

    fn turn(&self, now: Instant) -> Turn {
        let held = self.last_reply != LastReply::Unproven
            && self.last_packet.is_some_and(|at| now.saturating_duration_since(at) < self.hold);
        if held {
            return Turn::Wait;
        }
        if self.out == 0 {
            return Turn::Ask;
        }
        let wait = self.first_wait.max(self.hold);
        if self.sent_at.is_some_and(|at| now.saturating_duration_since(at) >= wait) {
            return Turn::Expired;
        }
        Turn::Wait
    }

    /// The request out has expired. What is left of the last capture stops being
    /// looked for, and a packet heard since says one was lost.
    fn expire(&mut self) {
        if self.to_drop > 0 {
            self.out = 0;
        } else if self.heard {
            self.out = self.out.saturating_sub(1);
        } else {
            self.hold /= 2;
        }
        self.to_drop = 0;
        self.expired = true;
    }

    fn asked(&mut self, now: Instant) {
        if self.last_reply == LastReply::Unproven {
            self.last_reply = LastReply::InTime;
        }
        self.asks = self.asks.saturating_add(1);
        self.sent_before = if self.out > 0 { self.sent_at } else { None };
        self.out = (self.out + 1).min(MOST_OUT);
        self.sent_at = Some(now);
        self.heard = false;
    }
}

/// What a capture has had since its last sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Silent {
    /// No bytes at all.
    NoBytes,
    /// Bytes, and no whole packet of them.
    NoPacket,
}

/// What [`Silence::check`] finds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Outage {
    /// Say so, and go on.
    Warn(Silent),
    /// End the capture, after this long with no sample.
    Fail(Silent, Duration),
}

/// How long a capture has had no sample. It runs from the last sample, not from the
/// last byte, so a stream of junk cannot keep a `--samples` capture going. The stamp
/// is taken after the send, so a client that keeps a send waiting is no silence of the
/// meter's.
///
/// At `bound` it warns, once per outage. Only a capture with a sample limit and no
/// time limit can go on for ever: with no sample yet it fails at `bound`, as nothing
/// says a meter is there, and after one only at `outage_bound`. A meter comes back
/// from a loose cable or from its auto power off, and the rest of a long log is
/// worth a warning.
#[derive(Debug)]
struct Silence {
    bound: Duration,
    outage_bound: Duration,
    since: Instant,
    bytes: bool,
    sampled: bool,
    warned: bool,
}

impl Silence {
    fn new(bound: Duration, outage_bound: Duration, now: Instant) -> Silence {
        Silence { bound, outage_bound, since: now, bytes: false, sampled: false, warned: false }
    }

    fn bytes(&mut self) {
        self.bytes = true;
    }

    /// A sample went out: how long it had been, if that was told.
    fn sample(&mut self, now: Instant) -> Option<Duration> {
        let told = self.warned.then(|| now.saturating_duration_since(self.since));
        self.since = now;
        self.bytes = false;
        self.sampled = true;
        self.warned = false;
        told
    }

    fn check(&mut self, only_samples: bool, now: Instant) -> Option<Outage> {
        let quiet = now.saturating_duration_since(self.since);
        if quiet < self.bound {
            return None;
        }
        let what = if self.bytes { Silent::NoPacket } else { Silent::NoBytes };
        if only_samples && !self.sampled {
            return Some(Outage::Fail(what, self.bound));
        }
        if only_samples && quiet >= self.outage_bound {
            return Some(Outage::Fail(what, self.outage_bound));
        }
        if self.warned {
            return None;
        }
        self.warned = true;
        Some(Outage::Warn(what))
    }
}

/// What the log says at the mark, going on.
fn outage_warning(what: Silent, after: Duration) -> String {
    let seconds = after.as_secs();
    match what {
        Silent::NoBytes => {
            format!("No answer from the thermometer for {seconds} s; still asking.")
        }
        Silent::NoPacket => format!(
            "No usable answer from the thermometer for {seconds} s: bytes came, but no whole \
             packet; still asking."
        ),
    }
}

/// What the log says of a meter that came back.
fn back_line(after: Duration) -> String {
    format!("The thermometer answers again, after {} s with no sample.", after.as_secs())
}

/// What ends a capture on a silence, for the log and for `capture failed`.
fn outage_failure(what: Silent, after: Duration) -> (String, String) {
    let seconds = after.as_secs();
    match what {
        Silent::NoBytes => (
            format!("No answer from the thermometer for {seconds} s."),
            format!("no answer from the thermometer for {seconds} s"),
        ),
        Silent::NoPacket => (
            format!(
                "No usable answer from the thermometer for {seconds} s: bytes came, but no \
                 whole packet."
            ),
            format!("no usable answer from the thermometer for {seconds} s"),
        ),
    }
}

/// What the log says of a burst that is no multiple of a packet.
fn burst_warning(bytes: usize) -> String {
    format!("A reply of {bytes} bytes, where 45 were due: this sample may be wrong.")
}

/// What the log says at the end, of more than one such burst.
fn bursts_line(count: u32) -> String {
    format!("{count} replies of this capture were of the wrong length.")
}

/// What the log says of a request that failed.
fn send_error_line(e: &Error) -> String {
    format!("Error sending 'A' command: {e}.")
}

/// What the log says of a read that failed.
fn read_error_line(e: &Error) -> String {
    format!("Serial port read error: {e}.")
}

/// `sr_sw_limits_check`'s time limit, in ms, 0 being none.
fn time_is_up(limit_msec: u64, started: Instant, now: Instant) -> bool {
    limit_msec > 0 && now.saturating_duration_since(started) > Duration::from_millis(limit_msec)
}

/// One capture, on its own thread: `receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    limit_msec: u64,
    /// `-C`, which C ignores: its packets go out whatever it says.
    enabled: [bool; protocol::CHANNELS],
    pass: Duration,
    request_wait: Duration,
    silence_bound: Duration,
    outage_bound: Duration,
    settle: Duration,
}

/// What a capture keeps count of, from one pass to the next.
#[derive(Debug, Default)]
struct Tally {
    samples: u64,
    /// Bytes since the last burst ended, or the last request.
    burst: usize,
    /// A packet came of them.
    packet: bool,
    /// Bursts holding a packet and no multiple of one.
    wrong: u32,
    told_mode: bool,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let started = Instant::now();
        if !sink.send_while(Packet::header(), || !stop.load(Ordering::SeqCst)) {
            return Ok(());
        }
        let left = std::mem::take(&mut self.link.lock().expect("link lock").out);
        let mut requests = Requests::new(left, self.request_wait, Instant::now());
        let mut tally = Tally::default();
        let outcome = self.passes(sink, stop, started, &mut requests, &mut tally);
        if tally.wrong > 1 {
            log::warn!("{}", bursts_line(tally.wrong));
        }
        self.link.lock().expect("link lock").out = requests.out;
        outcome
    }

    /// `serial_read_nonblocking`, with a short wait: what came.
    fn read(&self, wait: Duration) -> Result<usize> {
        let mut link = self.link.lock().expect("link lock");
        let Link { port, buffer, .. } = &mut *link;
        let Some(port) = port.as_mut() else {
            return Ok(0);
        };
        let got = port.read(buffer.room(), wait)?;
        buffer.filled(got);
        Ok(got)
    }

    /// The next whole packet that has come, `settled` once no more is coming. The lock
    /// goes before anything is sent.
    fn next_packet(&self, settled: bool) -> Option<[u8; PACKET_SIZE]> {
        self.link.lock().expect("link lock").buffer.next_packet(settled)
    }

    /// Whether what has come must see some quiet before the limits are looked at: a
    /// packet is held for it, or one came of this burst and its length is still open.
    fn settling(&self, tally: &Tally) -> bool {
        tally.burst > 0
            && (tally.packet
                || self.link.lock().expect("link lock").buffer.waiting() >= PACKET_SIZE)
    }

    /// `center_send`, and what the wait for its reply starts from. A timeout is no
    /// error, as `serial_write_blocking` gives 0. What has come of a line goes first:
    /// nothing has come for a pass, and the next reply must not be read after it.
    fn send_request(&self, requests: &mut Requests) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        if link.buffer.waiting() > 0 {
            log::debug!("Dropping {} bytes ahead of a request.", link.buffer.waiting());
            link.buffer.clear();
        }
        let timeout = serial_timeout(&link.params, 1);
        let Some(port) = link.port.as_mut() else {
            return Err(Error::device("device is not open"));
        };
        if let Err(e) = port.write_timeout(REQUEST, timeout) {
            log::error!("{}", send_error_line(&e));
            return Err(Error::device("unable to send a request"));
        }
        drop(link);
        requests.asked(Instant::now());
        Ok(())
    }

    /// Each packet that has come, as `handle_new_data`: false when the sink is gone.
    fn take_packets(
        &self,
        sink: &PacketSink,
        stop: &AtomicBool,
        settled: bool,
        requests: &mut Requests,
        silence: &mut Silence,
        tally: &mut Tally,
    ) -> bool {
        while let Some(bytes) = self.next_packet(settled) {
            if log::log_enabled!(log::Level::Trace) {
                log::trace!("Packet: {bytes:02x?}");
            }
            tally.packet = true;
            if requests.packet(Instant::now()) {
                log::debug!("Dropping a reply from the capture before.");
                continue;
            }
            // A read holding two packets sends both, past the limit, as C does. Not
            // where the second answers a request C would not have sent.
            if requests.expired && self.limit_samples > 0 && tally.samples >= self.limit_samples {
                log::debug!("Dropping a reply past the limit, after an expiry.");
                continue;
            }
            if !tally.told_mode
                && let Some(line) = protocol::mode_warning(bytes[1])
            {
                tally.told_mode = true;
                log::warn!("{line}");
            }
            if !self.send_reading(sink, stop, &protocol::parse(&bytes)) {
                return false;
            }
            tally.samples += 1;
            if let Some(after) = silence.sample(Instant::now()) {
                log::warn!("{}", back_line(after));
            }
        }
        true
    }

    /// The line has gone quiet, or a stream gives it no chance to: what is held goes
    /// out, and a burst holding a packet says if its length is wrong.
    fn end_burst(
        &self,
        sink: &PacketSink,
        stop: &AtomicBool,
        requests: &mut Requests,
        silence: &mut Silence,
        tally: &mut Tally,
    ) -> bool {
        if !self.take_packets(sink, stop, true, requests, silence, tally) {
            return false;
        }
        if tally.packet && !tally.burst.is_multiple_of(PACKET_SIZE) {
            tally.wrong += 1;
            if tally.wrong == 1 {
                log::warn!("{}", burst_warning(tally.burst));
            }
        }
        tally.burst = 0;
        tally.packet = false;
        true
    }

    fn send_reading(&self, sink: &PacketSink, stop: &AtomicBool, reading: &Reading) -> bool {
        let go_on = || !stop.load(Ordering::SeqCst);
        for (i, t) in reading.temperatures.iter().enumerate() {
            if !self.enabled[i] {
                continue;
            }
            let data = AnalogData {
                channels: vec![i],
                samples: vec![t.value],
                mq: Mq::Temperature,
                flags: MqFlags::empty(),
                unit: reading.unit,
                digits: t.digits,
            };
            if !sink.send_while(Packet::Analog(data), go_on) {
                return false;
            }
        }
        true
    }

    fn passes(
        &self,
        sink: &PacketSink,
        stop: &AtomicBool,
        started: Instant,
        requests: &mut Requests,
        tally: &mut Tally,
    ) -> Result<()> {
        let only_samples = self.limit_samples > 0 && self.limit_msec == 0;
        let mut silence = Silence::new(self.silence_bound, self.outage_bound, Instant::now());
        let mut due = Instant::now() + self.pass;
        let mut last_bytes = Instant::now();
        let mut unsettled_since = None;
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            // The read comes first, so a ready answer wins over the timer. A port
            // gone reads as an error: C's source calls back for ever with HUP set,
            // with no read and a failed request.
            let wait = due.saturating_duration_since(Instant::now()).min(STOP_CHECK);
            let got = self.read(wait).map_err(|e| {
                log::error!("{}", read_error_line(&e));
                Error::device("the port gone")
            })?;
            let now = Instant::now();
            if got > 0 {
                tally.burst += got;
                silence.bytes();
                last_bytes = now;
                if !self.take_packets(sink, stop, false, requests, &mut silence, tally) {
                    return Ok(());
                }
                due = Instant::now() + self.pass;
                if self.settling(tally) {
                    let since = *unsettled_since.get_or_insert(now);
                    if now.saturating_duration_since(since) < self.settle * MOST_UNSETTLED {
                        continue;
                    }
                    if !self.end_burst(sink, stop, requests, &mut silence, tally) {
                        return Ok(());
                    }
                    unsettled_since = None;
                }
            } else if self.settling(tally) {
                // Quiet is an empty read past the time: a thread that was held up has
                // looked at the line first.
                if now.saturating_duration_since(last_bytes) < self.settle {
                    continue;
                }
                if !self.end_burst(sink, stop, requests, &mut silence, tally) {
                    return Ok(());
                }
                unsettled_since = None;
                due = Instant::now() + self.pass;
            } else {
                if now < due {
                    continue;
                }
                let turn = requests.turn(now);
                if turn == Turn::Expired {
                    log::debug!("Request expired, with {} out.", requests.out);
                    requests.expire();
                }
                if turn != Turn::Wait {
                    tally.burst = 0;
                    self.send_request(requests)?;
                }
                due = Instant::now() + self.pass;
            }
            let now = Instant::now();
            if time_is_up(self.limit_msec, started, now)
                || (self.limit_samples > 0 && tally.samples >= self.limit_samples)
            {
                return Ok(());
            }
            match silence.check(only_samples, now) {
                None => {}
                Some(Outage::Warn(what)) => {
                    log::warn!("{}", outage_warning(what, self.silence_bound));
                }
                Some(Outage::Fail(what, after)) => {
                    let (line, why) = outage_failure(what, after);
                    log::error!("{line}");
                    return Err(Error::device(why));
                }
            }
        }
    }
}

impl Driver for Center309Driver {
    fn id(&self) -> &'static str {
        Brand::Center309.driver()
    }

    fn long_name(&self) -> &'static str {
        Brand::Center309.long_name()
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        scan(Brand::Center309, options)
    }
}

impl Driver for VoltcraftK204Driver {
    fn id(&self) -> &'static str {
        Brand::VoltcraftK204.driver()
    }

    fn long_name(&self) -> &'static str {
        Brand::VoltcraftK204.long_name()
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        scan(Brand::VoltcraftK204, options)
    }
}
