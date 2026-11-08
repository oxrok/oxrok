// Counterpart in sigrok:
//   libsigrok/src/hardware/beaglelogic/api.c,
//   libsigrok/src/hardware/beaglelogic/beaglelogic_tcp.c,
//   libsigrok/src/hardware/beaglelogic/protocol.c,
//   Copyright (C) 2014-2017 Kumar Abhishek <abhishek@theembeddedkitchen.net>,
//   Copyright (C) 2017 Kumar Abhishek <abhishek@theembeddedkitchen.net>,
//   Copyright (C) 2013 Martin Ling <martin-sigrok@earth.li>,
//   Copyright (C) 2013 poljar (Damir Jelić) <poljarinho@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Aurelien Jacobs, Daniel Elstner, Soeren Apel, Gerhard Sittig,
//   Tilman Sauerbeck, Michał Janiszewski
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The `BeagleLogic` logic analyzer: the PRUs of a `BeagleBone` behind a kernel module,
//! reached here over the TCP server that goes with it (`conn=tcp-raw/HOST/5555`).
//!
//! C has a second way in, `/dev/beaglelogic` on the board itself: ioctls, a mapped
//! buffer and `poll`. That isn't ported, as nothing could test it, so a scan with no
//! `conn=` finds nothing and says so if the node is there. A board set up with the
//! project's `install.sh` runs the Go server as `beaglelogic-tcp.service` on port 5555,
//! so `conn=tcp-raw/127.0.0.1/5555` reaches the same module there; a board set up
//! without it has no server. TCP is slower than the mapped buffer: in a continuous
//! capture the kernel's overrun is only a warning in `dmesg`, and the Go server then
//! ends the stream in silence. See `docs/DEVICES.md` for what the native way in needs.
//!
//! A scan connects, asks `version` and closes. An open connects again, reads four
//! settings and asks for a continuous stream, which a limit turns to one-shot. A start
//! sets the sample width from the channels enabled, and the capture thread sends
//! `get`, reads until it has what it wants, and sends `close`.
//!
//! Where it goes its own way, each pinned by a case:
//!
//! - **Numbers.** An answer that is no number fails the open; C makes it 0, and a
//!   samplerate of 0 then divides by zero at the start.
//! - **Silence.** Every wait has a bound, where C's `recv` blocks for good. An answer
//!   gets [`protocol::NO_ANSWER`], and a stream a bound from its pace: see
//!   [`protocol::Bounds`] and [`protocol::Quiet`].
//! - **Refused settings.** The module refuses every setting while it finishes a
//!   capture, which can be a buffer's time after one ended. So a refused `sampleunit`,
//!   or `triggerflags` for a limit or at the open, is asked back: a capture goes on if
//!   the module holds what was wanted, with a warning, and fails saying why if not. C
//!   ignores a refused `sampleunit` and reads 8 bits as 16 with exit 0, and fails a
//!   limit on a refused `triggerflags`. A refused or failed set leaves the value as it
//!   was, where C keeps what was asked.
//! - **`--time` is a sample limit** here, as the CLI sets `limit_samples` from it for a
//!   driver with no `limit_msec`. So a stream that stays silent fails a `--time`
//!   capture at the bound, as it does `--samples`, where rule elsewhere would end it
//!   with a warning.
//! - **A rate the module rounds.** The module keeps a whole divisor of 100 MHz
//!   (`coreclockfreq` is 200 MHz, fixed at probe), so 60 MHz is 100 MHz. C reports what
//!   was asked, so every time in the output is wrong, with exit 0; here the warning
//!   says so and the value is the module's rule, worked out here rather than asked
//!   back, which keeps C's wire. A rate past 32 bits is refused, where C's cast sends
//!   something else.
//! - **`limit_samples` after a continuous capture** reads back what it was. C reads
//!   back `UINT64_MAX`, as it keeps a continuous capture's no-limit in the same field.
//! - **Triggers and limits:** see [`protocol::Acquire`].
//! - **A closed connection** ends the capture as failed. C sends End, then spins for
//!   good in its drain, and SIGPIPE kills it if it has a command to send.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, SoftTriggerLogic, Trigger, TriggerFeatures, TriggerMatch, Value,
    ValueList,
};
use crate::serial::Port;
use crate::serial::tcpraw::TcpRaw;
use protocol::{Acquire, Bounds, Dial, Done, Link, Quiet, TCP_BUFFER_SIZE, Times, Unit, Verdict};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::NumLogicChannels];
const CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static DEV_OPTIONS: [OptionDesc; 6] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::NumLogicChannels, Access::GET),
];
static TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];
const CHANNEL_NAMES: [&str; 14] = [
    "P8_45", "P8_46", "P8_43", "P8_44", "P8_41", "P8_42", "P8_39", "P8_40", "P8_27", "P8_29",
    "P8_28", "P8_30", "P8_21", "P8_20",
];
/// C's `/dev/beaglelogic`.
const DEV_NODE: &str = "/dev/beaglelogic";
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How long a buffer may take to fill before a start says so.
const SLOW_BUFFER: Duration = Duration::from_secs(10);
/// How long a connect may take, where C's has the system's.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// The `beaglelogic` driver.
pub struct BeagleLogicDriver;

/// What a device holds of its settings.
#[derive(Clone, Copy, Debug)]
struct Settings {
    samplerate: u32,
    unit: Unit,
    buffersize: u32,
    bufunitsize: u32,
    limit_samples: u64,
    /// `triggerflags`: a limit makes the stream one-shot.
    oneshot: bool,
    capture_ratio: u64,
}

/// One `BeagleLogic`.
pub struct BeagleLogicDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    settings: Settings,
    trigger: Option<Trigger>,
    /// The connection, while no capture has it. `None` closed, as is what it holds.
    link: Option<Link>,
    /// What makes a link, while there is none.
    dial: Option<Dial>,
    worker: Option<Worker>,
    capturing: Arc<AtomicBool>,
    /// Bounds on a capture's silence, instead of what the pace says.
    bounds: Option<Bounds>,
    /// The times of an exchange.
    times: Times,
}

struct Worker {
    stop: Arc<AtomicBool>,
    /// Ends by giving the link back.
    handle: JoinHandle<Link>,
}

fn channels(count: usize) -> Vec<Channel> {
    CHANNEL_NAMES
        .iter()
        .take(count)
        .enumerate()
        .map(|(i, name)| Channel::new(i, ChannelKind::Logic, (*name).to_owned()))
        .collect()
}

/// A connection: `beaglelogic_tcp_open`, with its messages. A port here is a number,
/// where C's `getaddrinfo` also takes a service's name.
fn dial_tcp(host: &str, port: &str) -> Result<Box<dyn Port>> {
    let failed = |text: String| {
        log::error!("{text}");
        Error::Io(std::io::Error::other(text))
    };
    let words = |e: &std::io::Error| {
        let text = e.to_string();
        text.split(" (os error").next().unwrap_or_default().to_owned()
    };
    let Ok(number) = port.parse::<u16>() else {
        return Err(failed(format!("Address lookup failed: {host}:{port}: not a port number")));
    };
    let addresses = (host, number)
        .to_socket_addrs()
        .map_err(|e| failed(format!("Address lookup failed: {host}:{port}: {}", words(&e))))?;
    let mut last = None;
    for address in addresses {
        match TcpStream::connect_timeout(&address, CONNECT_TIMEOUT) {
            Ok(stream) => {
                let name = format!("tcp-raw/{host}/{port}");
                return Ok(Box::new(TcpRaw::from_stream(name, stream)?));
            }
            Err(e) => last = Some(e),
        }
    }
    let why = last.map_or_else(|| "no address".to_owned(), |e| words(&e));
    Err(failed(format!("Failed to connect to {host}:{port}: {why}")))
}

impl BeagleLogicDevice {
    fn new(count: usize, dial: Dial) -> Self {
        BeagleLogicDevice {
            info: DeviceInfo {
                driver: "beaglelogic",
                vendor: String::new(),
                model: "BeagleLogic".to_owned(),
                version: "1.0".to_owned(),
                serial: String::new(),
                conn: None,
            },
            channels: channels(count),
            settings: Settings {
                samplerate: 0,
                unit: Unit::Sixteen,
                buffersize: 0,
                bufunitsize: 0,
                limit_samples: 10_000_000,
                oneshot: false,
                capture_ratio: 0,
            },
            trigger: None,
            link: None,
            dial: Some(dial),
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            bounds: None,
            times: Times::default(),
        }
    }

    /// Sets the old worker's stop and joins it, which gets the link back: an End
    /// stuck on a queue no one reads gives up within its bound.
    fn end_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            match worker.handle.join() {
                Ok(link) => self.link = Some(link),
                Err(_) => log::error!("The capture thread died with the connection."),
            }
        }
    }

    /// The link, with the last capture's thread out of the way. A set in a capture
    /// would have its answer lost in the stream, and the module refuses it anyway.
    fn idle_link(&mut self) -> Result<&mut Link> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.end_worker();
        self.link.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    fn set_samplerate(&mut self, value: u64) -> Result<()> {
        let Ok(asked) = u32::try_from(value) else {
            return Err(Error::Arg(format!(
                "The device takes a samplerate of at most 100 MHz, not {value} Hz"
            )));
        };
        self.idle_link()?.set("samplerate", asked)?;
        // A device that took it took it by this rule, in the module's source.
        let kept = protocol::module_rate(asked).unwrap_or(asked);
        if kept != asked {
            log::warn!("The device runs at a whole divisor of 100 MHz: {kept} Hz, not {asked} Hz.");
        }
        self.settings.samplerate = kept;
        Ok(())
    }

    fn set_limit_samples(&mut self, limit: u64) -> Result<()> {
        let Settings { unit, buffersize, .. } = self.settings;
        if protocol::past_buffer(limit, unit, buffersize) {
            log::warn!("Insufficient buffer space has been allocated.");
            log::warn!("{}", protocol::truncated_message(buffersize, unit));
        }
        match self.idle_link()?.set_checked("triggerflags", 0)? {
            None => {}
            Some(0) => log::warn!("{}.", protocol::kept_message("triggerflags", 0)),
            Some(held) => {
                return Err(Error::device(protocol::held_message("triggerflags", 0, held)));
            }
        }
        self.settings.limit_samples = limit;
        self.settings.oneshot = true;
        Ok(())
    }

    /// What `-t` makes of the trigger, on what a capture streams:
    /// `soft_trigger_logic_check` skips matches on disabled channels.
    fn soft_trigger(&self, unit: Unit, limit: Option<u64>) -> Option<SoftTriggerLogic> {
        let trigger = self.trigger.as_ref()?;
        let stages = trigger
            .stages
            .iter()
            .map(|stage| {
                stage
                    .conditions
                    .iter()
                    .filter(|c| self.channels.iter().any(|ch| ch.index == c.channel && ch.enabled))
                    .map(|c| (c.channel, c.matches))
                    .collect()
            })
            .collect();
        let pre = protocol::pre_trigger_samples(
            self.settings.capture_ratio,
            limit,
            unit,
            self.settings.buffersize,
        );
        Some(SoftTriggerLogic::new(stages, unit.bytes(), pre))
    }
}

impl Drop for BeagleLogicDevice {
    /// A server left streaming goes on reading its device for the next client's
    /// stream (`beaglelogicRun`): a capture that a dropped device ends gets its `close`.
    fn drop(&mut self) {
        self.end_worker();
    }
}

impl Device for BeagleLogicDevice {
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

    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        Ok(match key {
            Key::LimitSamples => Value::U64(self.settings.limit_samples),
            Key::Samplerate => Value::U64(u64::from(self.settings.samplerate)),
            Key::CaptureRatio => Value::U64(self.settings.capture_ratio),
            Key::NumLogicChannels => {
                Value::I32(i32::try_from(self.channels.len()).unwrap_or(i32::MAX))
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let number =
            || value.as_u64().ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())));
        match key {
            Key::Samplerate => self.set_samplerate(number()?),
            Key::LimitSamples => self.set_limit_samples(number()?),
            Key::CaptureRatio => {
                let ratio = number()?;
                // `core::config_set` says this first, for a front end.
                if ratio > 100 {
                    return Err(Error::arg("capture ratio is a percentage"));
                }
                if self.capturing.load(Ordering::SeqCst) {
                    return Err(Error::Busy);
                }
                self.settings.capture_ratio = ratio;
                Ok(())
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Ok(match key {
            Key::Samplerate => {
                ValueList::RangeStepU64 { min: 10, max: u64::from(protocol::TOP_RATE), step: 1 }
            }
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `dev_open`. C ignores how each step went, and goes on with 0 for what it
    /// could not read. A number it can't have fails here, and so does a connection
    /// that has gone or says nothing. C goes on past a refused `triggerflags` alone.
    fn open(&mut self) -> Result<()> {
        if self.link.is_some() || self.worker.is_some() {
            return Ok(());
        }
        let Some(dial) = self.dial.take() else {
            return Err(Error::device("device has no way to connect"));
        };
        let mut link = Link::new(dial);
        link.times = self.times;
        let read = |link: &mut Link, settings: &mut Settings| -> Result<()> {
            link.connect()?;
            settings.samplerate = link.ask_number("samplerate")?;
            if settings.samplerate == 0 {
                return Err(Error::device("The device says its samplerate is 0"));
            }
            let answer = link.ask("sampleunit")?;
            settings.unit = protocol::parse_unit(&answer).ok_or_else(|| {
                Error::device(format!("The answer to 'sampleunit' is not 0 or 1: {answer:?}"))
            })?;
            settings.buffersize = link.ask_number("memalloc")?;
            settings.bufunitsize = link.ask_number("bufunitsize")?;
            Ok(())
        };
        let mut settings = self.settings;
        let outcome = read(&mut link, &mut settings);
        if let Err(e) = outcome {
            link.close();
            self.link = None;
            self.dial = Some(link.into_dial());
            return Err(e);
        }
        settings.oneshot = match link.set_checked("triggerflags", 1) {
            Ok(None) => false,
            Ok(Some(1)) => {
                log::warn!("{}.", protocol::kept_message("triggerflags", 1));
                false
            }
            // A capture with no limit then ends with the module's buffer.
            Ok(Some(0)) => {
                log::warn!("{}.", protocol::oneshot_kept_message());
                true
            }
            Ok(Some(held)) => {
                log::warn!("The answer to 'triggerflags' is neither 0 nor 1: {held}.");
                false
            }
            Err(e) => {
                log::warn!("Asking for a continuous stream failed: {e}.");
                false
            }
        };
        self.settings = settings;
        self.link = Some(link);
        Ok(())
    }

    /// `dev_close`, with nothing on the wire for it.
    fn close(&mut self) -> Result<()> {
        self.end_worker();
        if let Some(mut link) = self.link.take() {
            link.close();
            self.dial = Some(link.into_dial());
        }
        Ok(())
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&TriggerFeatures::default())?;
        }
        self.trigger = trigger;
        Ok(())
    }

    /// `dev_acquisition_start`. C's sets its sample width and ignores the answer; a
    /// device on the other width then gives every sample wrong.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        let wide = self.channels.iter().any(|ch| ch.index >= 8 && ch.enabled);
        let unit = if wide { Unit::Sixteen } else { Unit::Eight };
        let link = self.idle_link()?;
        match link.set_checked("sampleunit", unit.number())? {
            None => {}
            Some(held) if held == unit.number() => {
                log::warn!("{}.", protocol::kept_message("sampleunit", held));
            }
            Some(held) => {
                return Err(Error::device(protocol::held_message(
                    "sampleunit",
                    unit.number(),
                    held,
                )));
            }
        }
        self.settings.unit = unit;
        let Settings { samplerate, buffersize, bufunitsize, limit_samples, oneshot, .. } =
            self.settings;
        let limit = oneshot.then_some(limit_samples);
        let buffer = protocol::buffer_time(bufunitsize, samplerate, unit);
        if buffer >= SLOW_BUFFER {
            log::warn!("{}", protocol::slow_message(buffer));
        }
        if buffersize == 0 {
            log::warn!("{}", protocol::NO_BUFFERS);
        }
        let capture = Capture {
            acquire: Acquire::new(
                unit,
                limit,
                oneshot.then_some(u64::from(buffersize)),
                self.soft_trigger(unit, limit),
            ),
            bounds: self.bounds.unwrap_or_else(|| Bounds::for_buffer(buffer)),
            limited: oneshot,
            asked: false,
        };
        let Some(mut link) = self.link.take() else {
            return Err(Error::device("device is not open"));
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new()
            .name(format!("{} acquisition", self.info.driver))
            .spawn(move || {
                let outcome = capture.run(&mut link, &sink, &stop2);
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2, outcome);
                drop(sink);
                link
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

    /// Asks the thread to end, and does not wait: it may be sending. The thread sends
    /// the `close`.
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

/// One capture, on its own thread.
struct Capture {
    acquire: Acquire,
    bounds: Bounds,
    limited: bool,
    /// Whether `get` went out, which a `close` needs.
    asked: bool,
}

impl Capture {
    /// The stream, and a `close` whatever became of it: C's order is End, `close` and
    /// a drain, and a thread that's waited on gives the link back first. After a
    /// connection's end there's no one to tell.
    fn run(mut self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let outcome = self.stream(link, sink, stop);
        // A `close` with no `get` kills `tcp-server-node`.
        if self.asked && link.is_connected() {
            let closing = link.send("close").and_then(|()| link.drain());
            if let Err(e) = closing {
                log::debug!("The close of a capture failed: {e}.");
            }
        }
        // What a slow module's last read still sends would pass for an answer.
        link.mark_stale();
        outcome
    }

    fn stream(&mut self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        // `beaglelogic_start`: what is on the line goes, then the stream starts.
        if link.drain()? {
            return Err(protocol::closed_error());
        }
        self.asked = true;
        link.send("get")?;
        let slice = link.times.slice;
        let mut quiet = Quiet::new(self.bounds, self.limited, slice.saturating_mul(2));
        let mut buf = vec![0u8; TCP_BUFFER_SIZE];
        while go_on() {
            // From ahead of the read: what a stall takes after it is no silence.
            let looked = Instant::now();
            let got = link.read_stream(&mut buf, slice)?;
            if got == 0 {
                match quiet.empty(looked.elapsed()) {
                    Verdict::Wait => {}
                    Verdict::Warn(text) => log::warn!("{text}."),
                    Verdict::Fail(text) => return Err(Error::device(text)),
                }
                continue;
            }
            if let Some(text) = quiet.heard() {
                log::warn!("{text}");
            }
            let (packets, done) = self.acquire.take(&buf[..got]);
            for packet in packets {
                if !sink.send_while(packet, go_on) {
                    return Ok(());
                }
            }
            match done {
                Done::No => {}
                Done::Limit => return Ok(()),
                Done::Buffer => {
                    if self.acquire.waiting() {
                        log::warn!("{}", protocol::unfired_message(self.acquire.offset()));
                    }
                    return Ok(());
                }
            }
        }
        Ok(())
    }
}

impl Driver for BeagleLogicDriver {
    fn id(&self) -> &'static str {
        "beaglelogic"
    }

    fn long_name(&self) -> &'static str {
        "BeagleLogic"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut count = 14;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::NumLogicChannels => {
                    let Value::I32(asked) = value else {
                        return Err(Error::arg("logic_channels"));
                    };
                    count = if *asked > 8 { 14 } else { 8 };
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            if let Some(text) = native_note(std::path::Path::new(DEV_NODE)) {
                log::warn!("{text}");
            }
            return Ok(Vec::new());
        };
        let (host, port) = match protocol::parse_conn(&conn) {
            Ok((host, port)) => (host.to_owned(), port.to_owned()),
            Err(text) => {
                log::error!("{text}");
                return Ok(Vec::new());
            }
        };
        let dial: Dial = Box::new(move || dial_tcp(&host, &port));
        Ok(scan_with(dial, count).map(|d| Box::new(d) as Box<dyn Device>).into_iter().collect())
    }
}

/// What a scan with no `conn=` says of a node it can't use.
fn native_note(node: &std::path::Path) -> Option<String> {
    node.exists().then(|| {
        format!(
            "{} is here, and only the TCP way in is ported: run the board's TCP server, and \
             conn=tcp-raw/127.0.0.1/5555.",
            node.display()
        )
    })
}

/// The scan's probe: a connection for `version` alone, closed after it however it went.
fn scan_with(dial: Dial, count: usize) -> Option<BeagleLogicDevice> {
    let mut link = Link::new(dial);
    if link.connect().is_err() {
        return None;
    }
    let answer = link.ask("version");
    link.close();
    match answer {
        Ok(version) if protocol::is_beaglelogic(&version) => {
            log::info!("BeagleLogic device found.");
            Some(BeagleLogicDevice::new(count, link.into_dial()))
        }
        Ok(version) => {
            log::debug!("Not a BeagleLogic: {version:?}.");
            None
        }
        Err(Error::Timeout) => None,
        Err(Error::Device(text)) => {
            log::error!("{text}.");
            None
        }
        Err(e) => {
            log::error!("{e}.");
            None
        }
    }
}
