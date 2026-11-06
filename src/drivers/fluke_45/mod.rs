// Counterpart in sigrok:
//   libsigrok/src/hardware/fluke-45/api.c,
//   libsigrok/src/hardware/fluke-45/protocol.c,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2017 John Chajecki <subs@qcontinuum.plus.com>,
//   with commits by Gerhard Sittig, JohnCh
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Fluke 45 dual display multimeter on its RS-232 port, with SCPI's words and a
//! prompt after each command.
//!
//! A scan asks `*IDN?` and asks once more after a reopen, and closes the port. A
//! capture goes round the two displays: `FUNC1?`, `AUTO?`, then `VAL1?` and `MOD?` for
//! a display in use on a channel enabled, and the same for 2. A round with a reading
//! is a sample, as in C, so two displays give two packets per sample.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - It waits for the prompt before the next command, as the manual says a host must.
//!   C sends on the answer's line end, and reads one line more at its stop
//!   (`api.c:209`); here a stop leaves what is out counted, for the next capture.
//! - It reads lines, in whatever pieces they came. libsigrok takes a read ending in LF
//!   as one answer, so with answer and prompt in one read, as a cable's latency timer
//!   makes likely, C has `VDC\r\n=>` for the function and no reading ever.
//! - A question given up on is counted, its late answer goes with it, and no later
//!   answer is shifted: [`protocol::Requests`]. The wait for an answer grows.
//! - `VAL1?` with no reading is no sample, where C crashes on the NULL
//!   (`protocol.c:265`). A reading that isn't one is refused, where C sends what the
//!   stack holds (`protocol.c:272`).
//! - A silence ends: [`protocol::Watch`]. C has no bound.
//! - `AACDC` says AC and DC, `REL` says relative, and `CONT` on the second display has
//!   its unit: see [`protocol`] for C's lines.
//! - Only a serial port: a `conn=` of USBTMC or TCP finds nothing, as the meter has
//!   neither, and with no `conn=` nothing is probed. C would ask any USBTMC device.

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::scpi::{self, Resource, Scpi};
use protocol::{Kind, LONGEST_LINE, Limits, Next, Reply, Requests, Step, Timing, Watch};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
const SET: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, SET),
    OptionDesc::new(Key::LimitMsec, SET),
];

/// How often a read looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);
/// The session source's timeout: what a round waits after the last.
const PACE: Duration = Duration::from_millis(10);
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// The `fluke-45` driver.
pub struct Fluke45Driver;

/// The port and what a line has said so far.
pub(crate) struct Link {
    scpi: Scpi,
    requests: Requests,
    /// What has come and no line end has ended.
    part: Vec<u8>,
    /// Whole lines not looked at yet, in order.
    lines: std::collections::VecDeque<Vec<u8>>,
    /// A line so long that nothing can make use of it is going past.
    overlong: bool,
}

impl Link {
    fn new(scpi: Scpi) -> Link {
        Link {
            scpi,
            requests: Requests::new(Instant::now()),
            part: Vec::new(),
            lines: std::collections::VecDeque::new(),
            overlong: false,
        }
    }

    /// `bytes` came. A line longer than any answer goes as a line of its own, so that
    /// a stream of them is still something heard.
    fn take(&mut self, bytes: &[u8]) {
        for b in bytes {
            if *b == b'\n' {
                let mut line = std::mem::take(&mut self.part);
                if line.last() == Some(&b'\r') {
                    line.pop();
                }
                if std::mem::take(&mut self.overlong) {
                    line.clear();
                    line.extend_from_slice(b"\xff");
                }
                self.lines.push_back(line);
            } else if self.part.len() < LONGEST_LINE {
                self.part.push(*b);
            } else if !self.overlong {
                self.overlong = true;
                self.part.clear();
                self.lines.push_back(b"\xff".to_vec());
            }
        }
    }

    /// Sends a question, and forgets a line cut short: its tail will pass for junk, and
    /// its head joined to the next answer would pass for an answer.
    fn ask(&mut self, command: &str, kind: Kind) -> Result<()> {
        self.part.clear();
        self.overlong = false;
        self.scpi.send(command)?;
        self.requests.sent(kind, Instant::now());
        Ok(())
    }

    /// A question and what it came to; `None` for a stop or a limit's end.
    fn query(
        &mut self,
        command: &str,
        kind: Kind,
        watch: &mut Option<&mut Watch>,
        go_on: &dyn Fn() -> bool,
    ) -> Result<Option<Reply>> {
        self.ask(command, kind)?;
        let mut buf = [0u8; 256];
        loop {
            if !go_on() {
                return Ok(None);
            }
            while let Some(line) = self.lines.pop_front() {
                let text = String::from_utf8_lossy(&line);
                let step = self.requests.line(&text, Instant::now());
                log::trace!("Line {text:?}: {step:?}.");
                if let Some(watch) = watch.as_deref_mut() {
                    watch.heard(&step);
                }
                if let Step::Done(reply) = step {
                    return Ok(Some(reply));
                }
            }
            // Stamped ahead of the read: what an empty read says holds from here, and
            // a stall after it would pass for silence with the answer already in.
            let looked = Instant::now();
            let got = self.scpi.read_some(&mut buf, STOP_CHECK)?;
            if got > 0 {
                self.take(&buf[..got]);
                if let Some(watch) = watch.as_deref_mut() {
                    Self::check(watch, Instant::now())?;
                }
                continue;
            }
            // A gone tty can give 0 at once.
            if looked.elapsed() < Duration::from_millis(1) {
                std::thread::sleep(Duration::from_millis(1));
            }
            if let Some(watch) = watch.as_deref_mut() {
                Self::check(watch, looked)?;
                if watch.time_is_up(looked)
                    && looked.saturating_duration_since(self.requests.heard_at())
                        >= protocol::FIRST_WAIT
                {
                    return Ok(Some(self.requests.give_up()));
                }
            }
            if let Some(reply) = self.requests.expired(looked) {
                return Ok(Some(reply));
            }
        }
    }

    fn check(watch: &mut Watch, now: Instant) -> Result<()> {
        match watch.check(now) {
            Next::Wait => Ok(()),
            Next::Warn => {
                log::warn!("{}", watch.silence_message());
                Ok(())
            }
            Next::GiveUp(why) => Err(Error::device(why.message())),
        }
    }
}

/// One meter.
pub struct Fluke45Device {
    info: DeviceInfo,
    channels: Vec<Channel>,
    resource: String,
    serialcomm: Option<String>,
    limit_samples: u64,
    limit_msec: u64,
    link: Option<Link>,
    /// A transport to use instead of opening `resource`, for tests.
    pending: Option<Scpi>,
    worker: Option<Worker>,
    capturing: Arc<AtomicBool>,
    timing: Timing,
}

struct Worker {
    stop: Arc<AtomicBool>,
    /// Ends by giving the link back.
    handle: JoinHandle<Link>,
}

/// `probe_device` on an open port: the id if it answers, once more after a reopen.
fn ask_id(scpi: Scpi) -> Result<(scpi::HwInfo, Scpi)> {
    let mut link = Link::new(scpi);
    let mut asked = 0;
    loop {
        asked += 1;
        let reply = link.query("*IDN?", Kind::Id, &mut None, &|| true);
        if let Ok(Some(Reply::Answer(text))) = reply {
            return Ok((scpi::parse_hw_id(&text)?, link.scpi));
        }
        if asked == 2 {
            return Err(Error::device("Couldn't get IDN response."));
        }
        log::info!("Couldn't get IDN response, retrying.");
        let _ = link.scpi.close();
        let _ = link.scpi.reopen();
        link.requests = Requests::new(Instant::now());
    }
}

fn probe(scpi: Scpi, serialcomm: Option<&str>) -> Option<Fluke45Device> {
    let (id, mut scpi) = match ask_id(scpi) {
        Ok(found) => found,
        Err(e) => {
            log::debug!("{e}");
            return None;
        }
    };
    let resource = scpi.resource().to_owned();
    let _ = scpi.close();
    if !id.manufacturer.eq_ignore_ascii_case("FLUKE") || id.model != "45" {
        log::debug!("Device {} {} is not a Fluke 45.", id.manufacturer, id.model);
        return None;
    }
    Some(Fluke45Device::new(&id, resource, serialcomm.map(str::to_owned)))
}

impl Fluke45Device {
    fn new(id: &scpi::HwInfo, resource: String, serialcomm: Option<String>) -> Self {
        Fluke45Device {
            info: DeviceInfo {
                driver: "fluke-45",
                vendor: "FLUKE".to_owned(),
                model: "45".to_owned(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            channels: (0..2)
                .map(|i| Channel::new(i, ChannelKind::Analog, format!("P{}", i + 1)))
                .collect(),
            resource,
            serialcomm,
            limit_samples: 0,
            limit_msec: 0,
            link: None,
            pending: None,
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            timing: Timing::default(),
        }
    }

    /// A device on a transport that stands in for the hardware, without a scan.
    /// [`Device::open`] takes it.
    pub fn from_scpi(scpi: Scpi) -> Self {
        let id = scpi::HwInfo {
            manufacturer: "FLUKE".to_owned(),
            model: "45".to_owned(),
            serial_number: String::new(),
            firmware_version: String::new(),
        };
        let mut device = Self::new(&id, scpi.resource().to_owned(), None);
        device.pending = Some(scpi);
        device
    }

    /// Sets the old worker's stop and joins it, which gets the link back: an End
    /// stuck on a queue no one reads gives up within its bound.
    fn end_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            match worker.handle.join() {
                Ok(link) => self.link = Some(link),
                Err(_) => log::error!("The capture thread died with the port."),
            }
        }
    }
}

impl Device for Fluke45Device {
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

    /// C's `config_get` answers the limits, which `devopts` doesn't offer to get, so
    /// no caller reaches it.
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

    /// `sr_scpi_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        if self.link.is_some() || self.worker.is_some() {
            return Ok(());
        }
        let scpi = match self.pending.take() {
            Some(scpi) => scpi,
            None => scpi::open_keeping_line(
                &Resource::parse(&self.resource)?,
                self.serialcomm.as_deref(),
            )?,
        };
        self.link = Some(Link::new(scpi));
        Ok(())
    }

    /// `sr_scpi_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.end_worker();
        self.link = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.end_worker();
        let Some(mut link) = self.link.take() else {
            return Err(Error::device("device is not open"));
        };
        let capture = Capture {
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            enabled: [self.channels[0].enabled, self.channels[1].enabled],
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

    /// Asks the thread to end, and does not wait: it may be sending.
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

/// One capture, on its own thread: `fl45_scpi_receive_data`.
struct Capture {
    limit_samples: u64,
    limit_msec: u64,
    enabled: [bool; 2],
    timing: Timing,
}

impl Capture {
    /// One display's questions. Whether it gave a sample; `None` for a stop.
    fn display(
        &self,
        i: usize,
        link: &mut Link,
        sink: &PacketSink,
        watch: &mut Watch,
        go_on: &dyn Fn() -> bool,
    ) -> Result<Option<bool>> {
        let n = i + 1;
        let mut watching = Some(&mut *watch);
        let Some(func) = link.query(&format!("FUNC{n}?"), Kind::Function, &mut watching, go_on)?
        else {
            return Ok(None);
        };
        // A failed FUNC? returns ahead of AUTO? in C (`protocol.c:57`); one the meter
        // refused or that says no function does not.
        if matches!(func, Reply::Silent) {
            return Ok(Some(false));
        }
        let label = match &func {
            Reply::Answer(word) => protocol::function(word),
            _ => None,
        };
        let Some(auto) = link.query("AUTO?", Kind::Flag, &mut watching, go_on)? else {
            return Ok(None);
        };
        let Some(mut label) = label.filter(|_| self.enabled[i]) else {
            return Ok(Some(false));
        };
        if matches!(&auto, Reply::Answer(a) if a == "1") {
            label.flags |= crate::core::MqFlags::AUTORANGE;
        }
        let Some(value) = link.query(&format!("VAL{n}?"), Kind::Reading, &mut watching, go_on)?
        else {
            return Ok(None);
        };
        let Some(reading) = (match &value {
            Reply::Answer(text) => protocol::reading(text),
            _ => None,
        }) else {
            log::debug!("No reading for channel {n}: {value:?}.");
            return Ok(Some(false));
        };
        let mut digits = reading.digits;
        let Some(mods) = link.query("MOD?", Kind::Modifiers, &mut watching, go_on)? else {
            return Ok(None);
        };
        if let Reply::Answer(text) = &mods
            && let Some(mods) = protocol::modifiers(text)
        {
            protocol::apply_modifiers(mods, &mut label, &mut digits);
        }
        let packet = Packet::Analog(AnalogData {
            channels: vec![i],
            samples: vec![reading.value],
            mq: label.mq,
            flags: label.flags,
            unit: label.unit,
            digits,
        });
        if !sink.send_while(packet, go_on) {
            return Ok(None);
        }
        if watch.delivered(Instant::now()) {
            log::warn!("The meter is giving readings again.");
        }
        Ok(Some(true))
    }

    fn run(&self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let limits = Limits {
            samples_only: self.limit_samples > 0 && self.limit_msec == 0,
            msec: self.limit_msec,
        };
        let mut watch = Watch::new(self.timing, limits, Instant::now());
        let mut samples = 0u64;
        loop {
            let mut sent = false;
            for i in 0..2 {
                match self.display(i, link, sink, &mut watch, &go_on)? {
                    Some(gave) => sent |= gave,
                    None => return Ok(()),
                }
            }
            if sent {
                samples += 1;
            }
            if self.limit_samples > 0 && samples >= self.limit_samples {
                return Ok(());
            }
            if watch.time_is_up(Instant::now()) {
                if !watch.sampled() {
                    log::warn!("{}", watch.time_limit_message());
                }
                return Ok(());
            }
            let until = Instant::now() + PACE;
            while Instant::now() < until {
                if !go_on() {
                    return Ok(());
                }
                std::thread::sleep(Duration::from_millis(2));
            }
        }
    }
}

impl Driver for Fluke45Driver {
    fn id(&self) -> &'static str {
        "fluke-45"
    }

    fn long_name(&self) -> &'static str {
        "Fluke 45"
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
        // What fails here is no device, as `sr_scpi_scan_resource` has it.
        let resource = match conn.as_deref().map(Resource::parse).transpose() {
            Ok(Some(resource @ Resource::Serial(_))) => resource,
            Ok(_) => return Ok(Vec::new()),
            Err(e) => {
                log::info!("{e}");
                return Ok(Vec::new());
            }
        };
        let comm = serialcomm.as_deref();
        let scpi = match scpi::open_keeping_line(&resource, comm) {
            Ok(scpi) => scpi,
            Err(e) => {
                log::warn!("Couldn't open SCPI device: {e}.");
                return Ok(Vec::new());
            }
        };
        Ok(probe(scpi, comm).into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}
