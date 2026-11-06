// Counterpart in sigrok:
//   libsigrok/src/hardware/tondaj-sl-814/api.c,
//   libsigrok/src/hardware/tondaj-sl-814/protocol.c,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Martin Ling,
//   Daniel Elstner, Gerhard Sittig, Soeren Apel, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Tondaj SL-814 sound level meter on a serial port, at 9600 8e1.
//!
//! A scan opens the port and asks nothing, so any port that opens is a meter. It
//! closes the port, where C leaves it open, and an open opens it again.
//!
//! A capture sends init at its first 500 ms and a request 500 ms after that reply,
//! then one request 500 ms after each reply, one out at a time. A reply of the wrong
//! form is said in C's words, and the command goes again at the same pace. `--get`
//! has nothing to answer: C has no `config_get`.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - Waits for a reply are bounded. C's init reply blocks with no timeout, and a
//!   request's reply is read for ever, ahead of the limit's check. A time limit ends
//!   a capture there, with a warning, and otherwise 5 s with nothing usable is a
//!   failed capture, in every mode, as nothing would ever ask again.
//! - What comes with no command out is read away ahead of the next command. In C a
//!   stray byte has every later reply cut in the same wrong place: no reading ever
//!   again, and a `--samples` capture never ends.
//! - Each capture starts at init. C keeps its state and part of a reply from one
//!   capture to the next, for a library client that runs two on one device.
//! - `-C` with the channel disabled sends no packets and still counts samples, where
//!   C ignores it and sends them.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port, read_blocking, serial_timeout};
use protocol::{Command, Exchange, GiveUp, Heard, Next, Reading, SERIALCOMM, Timing};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::SoundLevelMeter];
const SET: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, SET),
    OptionDesc::new(Key::LimitMsec, SET),
];

/// How often a read looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);
/// The most read away at one go, so that a flood gives way to the checks.
const READ_AWAY: usize = 64;
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// The `tondaj-sl-814` driver.
pub struct TondajSl814Driver;

/// One meter.
pub struct TondajDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limit_samples: u64,
    limit_msec: u64,
    /// The open port, while no capture has it.
    port: Option<Box<dyn Port>>,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    worker: Option<Worker>,
    capturing: Arc<AtomicBool>,
    timing: Timing,
}

struct Worker {
    stop: Arc<AtomicBool>,
    /// Ends by giving the port back.
    handle: JoinHandle<Box<dyn Port>>,
}

impl TondajDevice {
    fn new(conn: String, params: Params) -> Self {
        TondajDevice {
            info: DeviceInfo {
                driver: "tondaj-sl-814",
                vendor: "Tondaj".to_owned(),
                model: "SL-814".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "P1".to_owned())],
            conn,
            params,
            limit_samples: 0,
            limit_msec: 0,
            port: None,
            pending: None,
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            timing: Timing::default(),
        }
    }

    /// A device on a port that stands in for the hardware, without a scan.
    /// [`Device::open`] takes it.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let mut device = Self::new(port.name().to_owned(), params);
        device.pending = Some(port);
        device
    }

    /// Sets the old worker's stop and joins it, which gets the port back: an End
    /// stuck on a queue no one reads gives up within its bound.
    fn end_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            match worker.handle.join() {
                Ok(port) => self.port = Some(port),
                Err(_) => log::error!("The capture thread died with the port."),
            }
        }
    }
}

impl Device for TondajDevice {
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

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        if self.port.is_some() || self.worker.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(port) => port,
            None => crate::serial::open(&self.conn, self.params)?,
        };
        self.port = Some(port);
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.end_worker();
        self.port = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.end_worker();
        let Some(mut port) = self.port.take() else {
            return Err(Error::device("device is not open"));
        };
        let capture = Capture {
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            enabled: self.channels[0].enabled,
            write_timeout: serial_timeout(&self.params, 3),
            timing: self.timing,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        // Spawning only fails short of resources, and then the port goes with the
        // closure; a next open opens it again.
        let spawned = std::thread::Builder::new()
            .name(format!("{} acquisition", self.info.driver))
            .spawn(move || {
                let outcome = capture.run(port.as_mut(), &sink, &stop2);
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2, outcome);
                drop(sink);
                port
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

/// Whether a capture begun at `started` has run its limit, in ms, 0 being none.
fn time_is_up(limit_msec: u64, started: Instant, now: Instant) -> bool {
    limit_msec > 0 && now.saturating_duration_since(started) > Duration::from_millis(limit_msec)
}

/// What a time limit says when it ends a wait that no reply has ended.
fn time_limit_message(heard: bool) -> &'static str {
    if heard {
        "No usable answer from the meter before the time limit."
    } else {
        "No answer from the meter before the time limit."
    }
}

/// What C logs for a failed write.
fn send_error_message(command: Command, e: &Error) -> String {
    match command {
        Command::Init => format!("Error sending init command: {e}."),
        Command::Request => format!("Error sending request command: {e}."),
    }
}

/// What C logs for a failed read of a reply.
fn read_error_message(command: Command, e: &Error) -> String {
    match command {
        Command::Init => format!("Error reading init reply: {e}."),
        Command::Request => format!("Error reading packet: {e}."),
    }
}

/// `serial_write_blocking` with its timeout: one that runs out is no error, and the
/// wait for its reply finds out.
fn send(port: &mut dyn Port, command: Command, timeout: Duration) -> Result<()> {
    let bytes = command.bytes();
    log::trace!("Sending command: {bytes:02x?}.");
    if let Err(e) = port.write_timeout(bytes, timeout) {
        log::error!("{}", send_error_message(command, &e));
        return Err(e);
    }
    Ok(())
}

/// What is there, waiting up to `wait` for it. `read_blocking` returns with no read
/// at all when a stall took it past its time, so that nothing is ever concluded from
/// that, a last look comes after an empty wait: a bound or a command that follows
/// this has really looked at the line.
fn read_some(port: &mut dyn Port, buf: &mut [u8], wait: Duration) -> Result<usize> {
    match read_blocking(port, buf, wait)? {
        0 => read_blocking(port, buf, Duration::ZERO),
        got => Ok(got),
    }
}

fn packet(reading: Reading) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![0],
        samples: vec![reading.value],
        mq: Mq::SoundPressureLevel,
        flags: reading.flags,
        unit: Unit::DecibelSpl,
        digits: 1,
    })
}

/// One capture, on its own thread: `tondaj_sl_814_receive_data`.
struct Capture {
    limit_samples: u64,
    limit_msec: u64,
    /// `-C`, which C ignores: its packets go out whatever it says.
    enabled: bool,
    write_timeout: Duration,
    timing: Timing,
}

impl Capture {
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let started = Instant::now();
        let mut exchange = Exchange::new(self.timing, started);
        let mut samples = 0u64;
        let mut buf = [0u8; READ_AWAY];
        let give_up = |why: GiveUp| Err(Error::device(why.message(self.timing.no_answer)));
        loop {
            if !go_on() {
                return Ok(());
            }
            let wanted = exchange.want(buf.len());
            let wait = exchange.read_wait(STOP_CHECK, Instant::now());
            let got = match (read_some(port, &mut buf[..wanted], wait), exchange.owed()) {
                (Ok(got), _) => got,
                (Err(e), Some(command)) => {
                    log::error!("{}", read_error_message(command, &e));
                    return Err(e);
                }
                // C reads nothing with no command out, and its next write is what
                // finds a port gone, in its own words.
                (Err(e), None) => {
                    log::debug!("Read failed with nothing out: {e}.");
                    std::thread::sleep(wait);
                    0
                }
            };
            if got == 0 {
                let now = Instant::now();
                match exchange.empty_read(now) {
                    Next::Wait => {
                        // C checks no limit here: a silent meter never ended one.
                        if time_is_up(self.limit_msec, started, now) && exchange.overdue(now) {
                            log::warn!("{}", time_limit_message(exchange.heard()));
                            return Ok(());
                        }
                    }
                    Next::Send(command) => {
                        send(port, command, self.write_timeout)?;
                        let now = Instant::now();
                        exchange.sent(command, now);
                        if time_is_up(self.limit_msec, started, now) {
                            return Ok(());
                        }
                    }
                    Next::GiveUp(why) => return give_up(why),
                }
                continue;
            }
            let heard = exchange.take(&buf[..got], Instant::now());
            match heard {
                Heard::Part => continue,
                Heard::InitOk => log::trace!("Received init reply."),
                Heard::Reading(a, b) => {
                    log::trace!("Received packet: {a:02x} {b:02x}.");
                    let reading = protocol::parse_reading(a, b);
                    if self.enabled && !sink.send_while(packet(reading), go_on) {
                        return Ok(());
                    }
                    samples += 1;
                    exchange.delivered(Instant::now());
                    if self.limit_samples > 0 && samples >= self.limit_samples {
                        return Ok(());
                    }
                }
                Heard::BadInit(reply) => {
                    log::debug!("Init reply: {reply:02x?}.");
                    log::error!("Received incorrect init reply, retrying.");
                }
                Heard::BadReply(reply) => {
                    log::debug!("Reply: {reply:02x?}.");
                    log::error!("Received incorrect request reply, retrying.");
                }
                Heard::Dropped => log::debug!("Reading away {got} bytes nothing asked for."),
            }
            let now = Instant::now();
            // C has no limit check on a wrong reply, whose next command has one; a
            // flood may leave no next command.
            if !matches!(heard, Heard::BadInit(_) | Heard::BadReply(_))
                && time_is_up(self.limit_msec, started, now)
            {
                return Ok(());
            }
            if !matches!(heard, Heard::InitOk | Heard::Reading(..)) && exchange.junk_too_long(now) {
                return give_up(GiveUp::NoUsableAnswer);
            }
        }
    }
}

impl Driver for TondajSl814Driver {
    fn id(&self) -> &'static str {
        "tondaj-sl-814"
    }

    fn long_name(&self) -> &'static str {
        "Tondaj SL-814"
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
        Ok(vec![Box::new(TondajDevice::new(conn, params))])
    }
}
