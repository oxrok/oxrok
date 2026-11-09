// Counterpart in sigrok:
//   libsigrok/src/hardware/hp-3478a/api.c,
//   libsigrok/src/hardware/hp-3478a/protocol.c,
//   Copyright (C) 2017-2021 Frank Stettner <frank-stettner@gmx.net>,
//   with commits by Gerhard Sittig, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The HP 3478A multimeter, on HP-IB: `conn=gpib/MINOR/PAD` ([`crate::scpi::gpib`]),
//! where libsigrok has `libgpib/NAME`. It speaks no SCPI and can't be identified, so
//! nothing is probed without a `conn=`, as in C. [`protocol`] has its codes.
//!
//! A scan asks `B`, the binary status, and closes; a meter whose error register is
//! set is no device, as in C. A setting sends its code (`F`, `R`, `N`) unless the last
//! status says it is already so, then asks `B` again; a get asks `B`. A capture sends
//! `T1` and asks `B`, then polls every 100 ms: a serial poll, and when its bit 0 says a
//! reading is ready, the reading and `B` again, for the sample's range and digits. Its
//! end sends `T1` and `D1`. A close sends the meter to local (`GTL`).
//!
//! Where it goes its own way:
//!
//! - A binary status shorter than four bytes, with no function, no digits or a range
//!   the function lacks, is no status, where C reads past the answer or keeps what it
//!   had (`protocol.c:416-419`, the unread returns of `parse_range_*`).
//! - A reading must have the meter's form (`+1.23456E+0`), where `sr_atod_ascii`
//!   takes any number; anything else fails the capture, as a number C can't read does.
//! - A meter that never has a reading ready fails a capture after 30 s, and after a
//!   reading gets a warning at 30 s and fails at 10 minutes; C polls for ever.
//! - A time limit ends a capture with no reading, where C checks it after a sample.
//! - `digits` takes `3.5`, `4.5` and `5.5` alone: C's loop matches `""` to `N-1` and
//!   reads past `digits_map` for anything else (`api.c:279-283`).
//! - A failed `T1` or status at the start fails the capture, where C goes on.
//! - A setting during a capture is refused, where C's would go out between a poll and
//!   its reading; so are a get and the range list, which ask the meter too.
//! - A message ends in a line feed with EOI on it, as `Scpi::send` adds one: `B\n`,
//!   `T1\n`. libsigrok's `sr_scpi_send` adds the line feed too, so the bytes are C's;
//!   the meter takes either end.
//! - Extended ohms (`F7`) take any range code: what the meter reports there is not
//!   documented, and C's `parse_range_ohm` is a guess too. The model reports 7.

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::gpib::IO_TIMEOUT;
use crate::scpi::{self, Scpi, ScpiTransport};
use protocol::{Limits, MQOPTS, Quiet, Silence, Silent, Status};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
static DEV_OPTIONS: [OptionDesc; 6] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::MeasuredQuantity, Access::GET_SET_LIST),
    OptionDesc::new(Key::Range, Access::GET_SET_LIST),
    OptionDesc::new(Key::Digits, Access::GET_SET_LIST),
];

/// The poll's period: `sr_scpi_source_add`'s 100 ms.
pub const POLL: Duration = Duration::from_millis(100);
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// The `hp-3478a` driver.
pub struct Hp3478aDriver;

/// `hp_3478a_get_status_bytes`: `B` and its answer, with C's lines for a set error
/// register. An error register set is an error, as in C.
fn get_status(scpi: &mut Scpi) -> Result<Status> {
    let bytes = scpi.get_data(Some("B"))?;
    let status = Status::parse(&bytes).map_err(Error::Device)?;
    let lines = status.error_lines();
    for line in &lines {
        log::error!("{line}");
    }
    if let Some(first) = lines.first() {
        return Err(Error::device(*first));
    }
    Ok(status)
}

/// `probe_device`: a meter whose status reads, closed again.
fn probe(mut scpi: Scpi) -> Option<Hp3478aDevice> {
    scpi.read_timeout = IO_TIMEOUT;
    let status = get_status(&mut scpi);
    let _ = scpi.close();
    match status {
        Ok(status) => Some(Hp3478aDevice::new(scpi, Some(status))),
        Err(e) => {
            log::debug!("No HP 3478A: {e}.");
            None
        }
    }
}

impl Driver for Hp3478aDriver {
    fn id(&self) -> &'static str {
        "hp-3478a"
    }

    fn long_name(&self) -> &'static str {
        "HP 3478A"
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
        // The meter can't be identified: only where it was said to be.
        let Some(conn) = conn else { return Ok(Vec::new()) };
        let found = match scpi::scan(Some(&conn), None, |s| Ok(probe(s))) {
            Ok(found) => found,
            Err(e) => {
                log::warn!("{e}");
                Vec::new()
            }
        };
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

struct Worker {
    stop: Arc<AtomicBool>,
    /// Gives the connection back to the link, with the last status, before it
    /// says the capture is over: a get after a capture that ended by itself finds
    /// it there.
    handle: JoinHandle<()>,
}

/// The connection and what the meter last said of itself.
struct Link {
    /// The connection while it is closed.
    closed: Option<Scpi>,
    /// The open connection, while no capture has it.
    scpi: Option<Scpi>,
    /// The last binary status.
    status: Option<Status>,
}

impl Link {
    fn open_scpi(&mut self) -> Result<&mut Scpi> {
        self.scpi.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `hp_3478a_get_status_bytes`, kept.
    fn refresh(&mut self) -> Result<Status> {
        let status = get_status(self.open_scpi()?)?;
        self.status = Some(status);
        Ok(status)
    }

    /// The last status, for a setting: C's `devc` fields.
    fn last(&mut self) -> Result<Status> {
        match self.status {
            Some(s) => Ok(s),
            None => self.refresh(),
        }
    }

    /// A code, then the status it leads to.
    fn command(&mut self, code: &str) -> Result<()> {
        self.open_scpi()?.send(code)?;
        self.refresh().map(|_| ())
    }
}

fn lock(link: &Mutex<Link>) -> MutexGuard<'_, Link> {
    link.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// One meter.
pub struct Hp3478aDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
    capturing: Arc<AtomicBool>,
    limits: Limits,
    silence: Silence,
}

impl Hp3478aDevice {
    fn new(closed: Scpi, status: Option<Status>) -> Self {
        Hp3478aDevice {
            info: DeviceInfo {
                driver: "hp-3478a",
                vendor: "Hewlett-Packard".to_owned(),
                model: "3478A".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "P1")],
            link: Arc::new(Mutex::new(Link { closed: Some(closed), scpi: None, status })),
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            limits: Limits::default(),
            silence: Silence::default(),
        }
    }

    /// A device on a transport that stands in for the connection, without a scan.
    /// [`Device::open`] takes it.
    pub fn from_transport(transport: Box<dyn ScpiTransport>) -> Self {
        let mut scpi = Scpi::new(transport);
        scpi.read_timeout = IO_TIMEOUT;
        Self::new(scpi, None)
    }

    /// The same, with other bounds on a silent meter, for a test.
    #[cfg(test)]
    fn with_silence(transport: Box<dyn ScpiTransport>, silence: Silence) -> Self {
        let mut d = Self::from_transport(transport);
        d.silence = silence;
        d
    }

    fn link(&self) -> MutexGuard<'_, Link> {
        lock(&self.link)
    }

    /// Sets the old worker's stop and joins it; it gives the connection back.
    fn end_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            if worker.handle.join().is_err() {
                log::error!("The capture thread died with the connection.");
            }
        }
    }

    /// The link for a setting or a get, with no capture running.
    fn idle(&mut self) -> Result<MutexGuard<'_, Link>> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.end_worker();
        Ok(self.link())
    }
}

impl Device for Hp3478aDevice {
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
        match key {
            Key::LimitSamples => return Ok(Value::U64(self.limits.samples)),
            Key::LimitMsec => return Ok(Value::U64(self.limits.msec)),
            Key::MeasuredQuantity | Key::Range | Key::Digits => {}
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let status = self.link().refresh()?;
        Ok(match key {
            Key::MeasuredQuantity => Value::Mq(status.mq, status.flag),
            Key::Range => Value::Str(status.range_name().to_owned()),
            _ => Value::Str(status.digits_name().to_owned()),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            // A capture took its limits at its start.
            Key::LimitSamples | Key::LimitMsec if self.capturing.load(Ordering::SeqCst) => {
                Err(Error::Busy)
            }
            Key::LimitSamples => {
                self.limits.samples = value.as_u64().ok_or_else(bad)?;
                Ok(())
            }
            Key::LimitMsec => {
                self.limits.msec = value.as_u64().ok_or_else(bad)?;
                Ok(())
            }
            Key::MeasuredQuantity => {
                let Value::Mq(mq, flags) = value else { return Err(bad()) };
                let code = protocol::mq_command(*mq, *flags)
                    .ok_or_else(|| Error::not_supported("the meter has no such function"))?;
                let mut link = self.idle()?;
                let last = link.last()?;
                if last.mq == *mq && last.flag == *flags {
                    return Ok(());
                }
                link.command(code)
            }
            Key::Range => {
                let name = value.as_str().ok_or_else(bad)?;
                let mut link = self.idle()?;
                let last = link.last()?;
                let exp = last.find_range(name).ok_or_else(|| {
                    Error::not_supported(format!("no range {name} on this function"))
                })?;
                if last.exp == exp {
                    return Ok(());
                }
                link.command(&protocol::range_command(exp))
            }
            Key::Digits => {
                let name = value.as_str().ok_or_else(bad)?;
                let (code, digits) = protocol::digits_command(name)
                    .ok_or_else(|| Error::not_supported(format!("no digits {name}")))?;
                let mut link = self.idle()?;
                if link.last()?.digits == digits {
                    return Ok(());
                }
                link.command(&code)
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        match key {
            Key::MeasuredQuantity => Ok(ValueList::Mq(MQOPTS.to_vec())),
            Key::Digits => Ok(ValueList::Str(protocol::DIGITS.map(str::to_owned).to_vec())),
            Key::Range => {
                if self.capturing.load(Ordering::SeqCst) {
                    return Err(Error::Busy);
                }
                let status = self.link().refresh()?;
                Ok(ValueList::Str(status.range_names().into_iter().map(str::to_owned).collect()))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `sr_scpi_open`.
    fn open(&mut self) -> Result<()> {
        if self.worker.is_some() {
            return Ok(());
        }
        let mut link = self.link();
        if link.scpi.is_some() {
            return Ok(());
        }
        let Some(mut scpi) = link.closed.take() else {
            return Err(Error::device("device has no connection"));
        };
        if let Err(e) = scpi.reopen() {
            link.closed = Some(scpi);
            return Err(e);
        }
        link.scpi = Some(scpi);
        Ok(())
    }

    /// `sr_scpi_close`: the meter goes to local.
    fn close(&mut self) -> Result<()> {
        self.end_worker();
        let mut link = self.link();
        if let Some(mut scpi) = link.scpi.take() {
            if let Err(e) = scpi.close() {
                log::debug!("close: {e}.");
            }
            link.closed = Some(scpi);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.end_worker();
        let Some(mut scpi) = self.link().scpi.take() else {
            return Err(Error::device("device is not open"));
        };
        let capture = Capture { limits: self.limits, silence: self.silence, status: None };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        let link = Arc::clone(&self.link);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new()
            .name(format!("{} acquisition", self.info.driver))
            .spawn(move || {
                let mut capture = capture;
                let outcome = capture.run(&mut scpi, &sink, &stop2);
                if outcome.is_ok() {
                    // dev_acquisition_stop: back to the internal trigger and the display.
                    for code in ["T1", "D1"] {
                        if let Err(e) = scpi.send(code) {
                            log::debug!("{code}: {e}.");
                        }
                    }
                }
                {
                    let mut link = lock(&link);
                    link.scpi = Some(scpi);
                    link.status = capture.status.or(link.status);
                }
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2, outcome);
                drop(sink);
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

    /// Asks the thread to end, and does not wait.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// `End`, waiting for room in the queue for as long as nobody asked to stop, and after
/// a stop only for [`END_AFTER_STOP`]: a close joins this thread.
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
    limits: Limits,
    silence: Silence,
    /// The last status read.
    status: Option<Status>,
}

impl Capture {
    /// Waits out a poll's period, or less on a stop.
    fn pause(stop: &AtomicBool) {
        let until = Instant::now() + POLL;
        while !stop.load(Ordering::SeqCst) {
            let left = until.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return;
            }
            std::thread::sleep(left.min(Duration::from_millis(10)));
        }
    }

    /// `hp_3478a_receive_data`, in a loop.
    fn run(&mut self, scpi: &mut Scpi, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        let began = Instant::now();
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        scpi.send("T1")?;
        self.status = Some(get_status(scpi)?);
        let mut quiet = Quiet::new(self.silence, Instant::now());
        let mut samples = 0u64;
        loop {
            Self::pause(stop);
            if !go_on() || self.limits.met(samples, began.elapsed()) {
                return Ok(());
            }
            // Stamped ahead of the poll: what an empty one says holds from here.
            let looked = Instant::now();
            let byte = scpi.serial_poll()?;
            if byte & protocol::SPOLL_READY == 0 {
                match quiet.nothing(looked) {
                    Silent::Wait => {}
                    Silent::Warn(secs) => log::warn!("{}", protocol::warn_text(secs)),
                    Silent::GiveUp(secs) => {
                        let text = quiet.give_up_text(secs);
                        log::error!("{text}");
                        return Err(Error::Device(text));
                    }
                }
                continue;
            }
            let raw = scpi.get_data(None)?;
            let text = String::from_utf8_lossy(&raw);
            let text = text.trim_end_matches('\n').trim_end_matches('\r');
            let Some(value) = protocol::parse_reading(text) else {
                let message = format!("The meter's reading is no number: '{text}'.");
                log::error!("{message}");
                return Err(Error::Device(message));
            };
            // Must come after the reading, which it resets (protocol.c).
            let status = get_status(scpi)?;
            self.status = Some(status);
            if quiet.reading(Instant::now()) {
                log::warn!("{}", protocol::BACK_TEXT);
            }
            let packet = Packet::Analog(AnalogData {
                channels: vec![0],
                samples: vec![value as f32],
                mq: status.mq,
                flags: status.flags,
                unit: status.unit,
                digits: status.places,
            });
            if !sink.send_while(packet, go_on) {
                return Ok(());
            }
            samples += 1;
            if self.limits.met(samples, began.elapsed()) {
                return Ok(());
            }
        }
    }
}
