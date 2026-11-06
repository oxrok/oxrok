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
//! one sample. The limit is looked at once all of a read is out, so a read holding
//! two packets sends both. `--get` has nothing to answer: C has no `config_get`.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - C sends `A` once and waits for a whole packet, with no bound. So a reply lost,
//!   part of one, or one stray byte between two replies, ends its polling for good,
//!   and a `--samples` capture never ends. Here a request expires, see [`Requests`],
//!   and 10 s with no whole packet fails a `--samples` capture, saying whether
//!   anything came.
//! - C's flag for that lives in a `static` (`protocol.c:218`), one for all devices
//!   and all captures of a process. So a capture that ends with `A` out leaves the
//!   next with no request. Here each capture knows what is out on its own link.
//! - What has come of a line is dropped ahead of a request, so bytes left by the
//!   last reply are not read as the head of the next. C keeps them, and a stray
//!   `0x02` then makes a packet one byte out when the next reply's byte 43 is 3.
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
/// How long a request stays out before the next, at first.
const REQUEST_WAIT: Duration = Duration::from_secs(2);
/// The longest a request is left unanswered before the next.
const MAX_WAIT: Duration = Duration::from_secs(8);
/// How long a `--samples` capture goes with no whole packet.
const SILENCE_BOUND: Duration = Duration::from_secs(10);
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
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new()
            .name(format!("{} acquisition", self.info.driver))
            .spawn(move || {
                let outcome = capture.run(&sink, &stop2);
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2, outcome);
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
/// after a stop only for [`END_AFTER_STOP`]: a close joins this thread.
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

/// What is to be done about requests, in a pass with no bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Turn {
    /// Nothing: a reply is due.
    Wait,
    /// All were answered: ask.
    Ask,
    /// The last went unanswered for too long: ask anyway.
    Expired,
}

/// C's `request_new_packet`, with a count, since no reply says what it answers.
///
/// A packet leaves the next request to the next pass with no bytes, as in C. C has
/// no expiry, and here a request has one: the reply may be lost, and the meter may
/// have been off. It asks again, and the wait doubles up to [`MAX_WAIT`] for as long
/// as nothing is heard; a reply within the first wait, with none out, sets it back.
/// If a packet was heard since the last request when that expires, what is still
/// counted was lost. So at a steady latency the wait grows past it after one or two
/// expiries, and no more than two are out: see the simulation in the tests.
#[derive(Debug)]
struct Requests {
    out: u32,
    sent_at: Option<Instant>,
    first_wait: Duration,
    wait: Duration,
    /// A packet came after the last request.
    heard: bool,
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
            first_wait,
            wait: first_wait,
            heard: false,
            to_drop: left,
            expired: false,
        }
    }

    /// A packet came: whether it is to be dropped.
    fn packet(&mut self, now: Instant) -> bool {
        self.out = self.out.saturating_sub(1);
        self.heard = true;
        if self.out == 0
            && self.sent_at.is_some_and(|at| now.saturating_duration_since(at) <= self.first_wait)
        {
            self.wait = self.first_wait;
        }
        if self.to_drop > 0 {
            self.to_drop -= 1;
            return true;
        }
        false
    }

    fn turn(&self, now: Instant) -> Turn {
        if self.out == 0 {
            return Turn::Ask;
        }
        if self.sent_at.is_some_and(|at| now.saturating_duration_since(at) >= self.wait) {
            return Turn::Expired;
        }
        Turn::Wait
    }

    /// The request out has expired. What is left of the last capture stops being
    /// looked for, and a packet heard says the rest were lost.
    fn expire(&mut self) {
        if self.to_drop > 0 || self.heard {
            self.out = 0;
        } else {
            self.wait = (self.wait * 2).min(self.first_wait.max(MAX_WAIT));
        }
        self.to_drop = 0;
        self.expired = true;
    }

    fn asked(&mut self, now: Instant) {
        self.out = self.out.saturating_add(1);
        self.sent_at = Some(now);
        self.heard = false;
    }
}

/// What a capture with only a sample limit has had since its last sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Silent {
    /// No bytes at all.
    NoBytes,
    /// Bytes, and no whole packet of them.
    NoPacket,
}

/// The bound on a capture with no sample. It runs from the last sample, not from
/// the last byte, so a stream of junk cannot keep a `--samples` capture going. The
/// stamp is taken after the send, so a client that keeps a send waiting is no
/// silence of the meter's.
#[derive(Debug)]
struct Silence {
    bound: Duration,
    since: Instant,
    bytes: bool,
}

impl Silence {
    fn new(bound: Duration, now: Instant) -> Silence {
        Silence { bound, since: now, bytes: false }
    }

    fn bytes(&mut self) {
        self.bytes = true;
    }

    fn sample(&mut self, now: Instant) {
        self.since = now;
        self.bytes = false;
    }

    fn verdict(&self, now: Instant) -> Option<Silent> {
        if now.saturating_duration_since(self.since) < self.bound {
            return None;
        }
        Some(if self.bytes { Silent::NoPacket } else { Silent::NoBytes })
    }
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
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let started = Instant::now();
        if !sink.send_while(Packet::header(), || !stop.load(Ordering::SeqCst)) {
            return Ok(());
        }
        let left = std::mem::take(&mut self.link.lock().expect("link lock").out);
        let mut requests = Requests::new(left, self.request_wait, Instant::now());
        let outcome = self.passes(sink, stop, started, &mut requests);
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

    /// The next whole packet that has come. The lock goes before anything is sent.
    fn next_packet(&self) -> Option<[u8; PACKET_SIZE]> {
        self.link.lock().expect("link lock").buffer.next_packet()
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

    /// Each packet of a read, as `handle_new_data`: false when the sink is gone.
    fn take_packets(
        &self,
        sink: &PacketSink,
        stop: &AtomicBool,
        requests: &mut Requests,
        silence: &mut Silence,
        samples: &mut u64,
    ) -> bool {
        while let Some(bytes) = self.next_packet() {
            if log::log_enabled!(log::Level::Trace) {
                log::trace!("Packet: {bytes:02x?}");
            }
            if requests.packet(Instant::now()) {
                log::debug!("Dropping a reply from the capture before.");
                continue;
            }
            // A read holding two packets sends both, past the limit, as C does. Not
            // where the second answers a request C would not have sent.
            if requests.expired && self.limit_samples > 0 && *samples >= self.limit_samples {
                log::debug!("Dropping a reply past the limit, after an expiry.");
                continue;
            }
            if !self.send_reading(sink, stop, &protocol::parse(&bytes)) {
                return false;
            }
            *samples += 1;
            silence.sample(Instant::now());
        }
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
    ) -> Result<()> {
        let mut silence = Silence::new(self.silence_bound, Instant::now());
        let mut samples = 0u64;
        let mut due = Instant::now() + self.pass;
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
            if got > 0 {
                silence.bytes();
                if !self.take_packets(sink, stop, requests, &mut silence, &mut samples) {
                    return Ok(());
                }
            } else {
                let now = Instant::now();
                if now < due {
                    continue;
                }
                let turn = requests.turn(now);
                if turn == Turn::Expired {
                    log::debug!("Request expired, with {} out.", requests.out);
                    requests.expire();
                }
                if turn != Turn::Wait {
                    self.send_request(requests)?;
                }
            }
            due = Instant::now() + self.pass;
            let now = Instant::now();
            if time_is_up(self.limit_msec, started, now)
                || (self.limit_samples > 0 && samples >= self.limit_samples)
            {
                return Ok(());
            }
            if self.limit_samples > 0 && self.limit_msec == 0 {
                self.check_silence(&silence, now)?;
            }
        }
    }

    fn check_silence(&self, silence: &Silence, now: Instant) -> Result<()> {
        let seconds = self.silence_bound.as_secs();
        match silence.verdict(now) {
            None => Ok(()),
            Some(Silent::NoBytes) => {
                log::error!("No answer from the thermometer for {seconds} s.");
                Err(Error::device(format!("no answer from the thermometer for {seconds} s")))
            }
            Some(Silent::NoPacket) => {
                log::error!(
                    "No usable answer from the thermometer for {seconds} s: bytes came, but no \
                     whole packet."
                );
                Err(Error::device(format!("no usable answer from the thermometer for {seconds} s")))
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
