// Counterpart in sigrok:
//   libsigrok/src/hardware/uni-t-ut32x/api.c,
//   libsigrok/src/hardware/uni-t-ut32x/protocol.c,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Gerhard Sittig,
//   Martin Ling, Soeren Apel, Daniel Elstner, Wolfram Sang, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The UNI-T `UT32x` thermometers: three channels (`T1`, `T2`, `T1-T2`), live records or
//! the stored ones, on a UT-D04 style HID cable or a serial port. `conn=` is required,
//! as the cable cannot be told from any other UT-D04, and a scan only opens the port
//! and closes it, as nothing can be asked.
//!
//! A capture sends `01` or `07` behind the header, passes what comes through C's
//! buffer ([`protocol::Receiver`]), one sample per record of the probe the meter
//! shows, and sends `02` when it ends. A record with no measurement counts for a
//! sample limit, which is how a limit on the memory passes over its free slots.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - C checks its limits only behind a record (`protocol.c:154-156`), so a device
//!   that says nothing never ends a capture, whatever the limit. Here a time limit
//!   allows [`TIME_GRACE`] past its end for that record, which is what C would stop
//!   on, and a sample limit alone stops after [`SILENCE_BOUND`] with no record.
//!   The second is a choice on an inference: no source says a meter never pauses, and
//!   a gap over 5 s, a menu say, ends a `--samples` capture with a warning and exit 0,
//!   where C waits.
//! - A failed read ends the capture. A gone tty polls readable and reads 0, which C
//!   takes as no data (`protocol.c:238-239`), so it spins on a CPU for ever.
//! - The first bad report on the cable ends the capture, as `HidPort` makes it a failed
//!   read, a choice. C goes on after a lone one with nothing queued, and with bytes
//!   queued it keeps `SR_ERR` in a `size_t` (`protocol.c:236-241`) and crashes.
//! - C loses every record for good after a marker split over its buffer's end
//!   (`protocol.c:231-234`), which one stray byte ahead of a record does; see
//!   [`protocol`].
//! - The port is given back and the `02` sent before `End`, where C has them behind
//!   it, so a client can start again as soon as it sees `End`. The wire is the same.
//! - A channel that is off gets no packet, and its records still count. C sends them
//!   whatever `-C` says (`protocol.c:119-143`). So `-C T2` with the meter on T1 gives
//!   no output and exit 0.
//! - `conn=1a86.e008`, a form C retired, is taken for `hid/ch9325`: it means the chip
//!   and not those ids, and of two cables the first the search finds.
//! - A limit set during a capture bites at the next record, as in C.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{CMD_GET_LIVE, CMD_GET_STORED, CMD_STOP, Parsed, Receiver};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Thermometer];
static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
];
const DATA_SOURCES: [&str; 2] = ["Live", "Memory"];
const CHANNEL_NAMES: [&str; 3] = ["T1", "T2", "T1-T2"];
const DEFAULT_SERIALCOMM: &str = "2400/8n1";

/// How long one read waits: a stop is seen this late.
const READ_TIMEOUT: Duration = Duration::from_millis(20);
/// How long past a time limit a capture waits for the record that ends it. The
/// slowest rate anyone gives is two records a second.
pub const TIME_GRACE: Duration = Duration::from_secs(2);
/// How long a capture that only a sample limit can end waits for a record.
pub const SILENCE_BOUND: Duration = Duration::from_secs(5);
/// How long `End` waits for room in the queue once asked to stop: a close joins.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// The driver.
pub struct Ut32xDriver;

impl Driver for Ut32xDriver {
    fn id(&self) -> &'static str {
        "uni-t-ut32x"
    }

    fn long_name(&self) -> &'static str {
        "UNI-T UT32x"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = DEFAULT_SERIALCOMM.to_owned();
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
        let conn = super::serial_dmm::hid_conn(&conn);
        // The device cannot be asked anything: an open that works must do, and one
        // that fails is no device.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| crate::serial::open(&conn, params).map(|port| (params, port)));
        match opened {
            Ok((params, port)) => {
                drop(port);
                Ok(vec![Box::new(Ut32xDevice::new(conn, params))])
            }
            Err(e) => {
                log::info!("Cannot open {conn}: {e}.");
                Ok(Vec::new())
            }
        }
    }
}

/// The limits, which a capture reads at each record.
#[derive(Debug, Default)]
struct Limits {
    samples: AtomicU64,
    msec: AtomicU64,
}

/// A capture's thread.
struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// One thermometer.
pub struct Ut32xDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limits: Arc<Limits>,
    memory: bool,
    /// The port, unless a capture has it.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    /// What a capture holds, which tests shorten.
    bounds: Bounds,
    worker: Option<Worker>,
    is_open: bool,
}

/// How long a capture waits with no record.
#[derive(Clone, Copy, Debug)]
struct Bounds {
    time_grace: Duration,
    silence: Duration,
}

impl Ut32xDevice {
    /// A closed device on `conn`.
    pub fn new(conn: String, params: Params) -> Ut32xDevice {
        Ut32xDevice {
            info: DeviceInfo {
                driver: "uni-t-ut32x",
                vendor: "UNI-T".to_owned(),
                model: "UT32x".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: CHANNEL_NAMES
                .iter()
                .enumerate()
                .map(|(i, n)| Channel::new(i, ChannelKind::Analog, *n))
                .collect(),
            conn,
            params,
            limits: Arc::default(),
            memory: false,
            port: Arc::new(Mutex::new(None)),
            bounds: Bounds { time_grace: TIME_GRACE, silence: SILENCE_BOUND },
            worker: None,
            is_open: false,
        }
    }

    /// The port slot, whatever a thread that panicked left in it.
    fn slot(&self) -> std::sync::MutexGuard<'_, Option<Box<dyn Port>>> {
        self.port.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

impl Device for Ut32xDevice {
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

    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        Ok(match key {
            Key::LimitSamples => Value::U64(self.limits.samples.load(Ordering::SeqCst)),
            Key::LimitMsec => Value::U64(self.limits.msec.load(Ordering::SeqCst)),
            Key::DataSource => Value::Str(DATA_SOURCES[usize::from(self.memory)].to_owned()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => {
                self.limits.samples.store(value.as_u64().ok_or_else(bad)?, Ordering::SeqCst);
            }
            Key::LimitMsec => {
                self.limits.msec.store(value.as_u64().ok_or_else(bad)?, Ordering::SeqCst);
            }
            Key::DataSource => {
                self.memory = match value.as_str().ok_or_else(bad)? {
                    "Live" => false,
                    "Memory" => true,
                    _ => return Err(bad()),
                };
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        match key {
            Key::DataSource => {
                Ok(ValueList::Str(DATA_SOURCES.iter().map(|s| (*s).to_owned()).collect()))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn open(&mut self) -> Result<()> {
        if self.is_open {
            return Ok(());
        }
        let port = crate::serial::open(&self.conn, self.params)?;
        *self.slot() = Some(port);
        self.is_open = true;
        Ok(())
    }

    /// `std_serial_dev_close`: the port, and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        *self.slot() = None;
        self.is_open = false;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if !self.is_open {
            return Err(Error::device("device is not open"));
        }
        let Some(port) = self.slot().take() else {
            return Err(Error::Busy);
        };
        // The port is back, so the last capture has done all but its End, which may
        // be waiting on a full queue that nobody reads.
        if let Some(old) = self.worker.take() {
            old.stop.store(true, Ordering::SeqCst);
            let _ = old.handle.join();
        }
        let capture = Capture {
            limits: Arc::clone(&self.limits),
            command: if self.memory { CMD_GET_STORED } else { CMD_GET_LIVE },
            enabled: self.channels.iter().map(|c| c.enabled).collect(),
            bounds: self.bounds,
        };
        let slot = Arc::clone(&self.port);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let spawned = std::thread::Builder::new().name("uni-t-ut32x acquisition".to_owned()).spawn(
            move || {
                let mut port = port;
                capture.run(port.as_mut(), &sink, &stop2);
                *slot.lock().unwrap_or_else(std::sync::PoisonError::into_inner) = Some(port);
                send_end(&sink, &stop2);
            },
        );
        match spawned {
            Ok(handle) => {
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => Err(Error::Io(e)),
        }
    }

    /// Asks the thread to end, and does not wait: it may be sending. It sends the
    /// `02`, as C's callback does.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// `End`, waiting for room in the queue for as long as nobody asked to stop, and after
/// a stop only for [`END_AFTER_STOP`].
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

/// What a capture needs.
struct Capture {
    limits: Arc<Limits>,
    command: u8,
    enabled: Vec<bool>,
    bounds: Bounds,
}

impl Capture {
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) {
        let going_on = || !stop.load(Ordering::SeqCst);
        if sink.send_while(Packet::header(), going_on) {
            // C takes no notice of a failed write, and neither does the meter's silence.
            if let Err(e) = port.write(&[self.command]) {
                log::warn!("Cannot send the start command: {e}.");
            }
            self.receive(port, sink, stop);
        }
        if let Err(e) = port.write(&[CMD_STOP]) {
            log::warn!("Cannot send the stop command: {e}.");
        }
    }

    fn receive(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) {
        let started = Instant::now();
        let mut last_record = started;
        let mut count = 0u64;
        let mut rx = Receiver::default();
        while !stop.load(Ordering::SeqCst) {
            let n = match port.read(rx.room(), READ_TIMEOUT) {
                Ok(n) => n,
                Err(e) => {
                    log::error!("Acquisition failed: {e}.");
                    return;
                }
            };
            let mut done = false;
            rx.received(n, |pkt| {
                let parsed = protocol::parse_packet(pkt);
                if parsed == Parsed::Invalid {
                    return;
                }
                last_record = Instant::now();
                if let Parsed::Reading { channel, value, unit, flags } = parsed
                    && self.enabled[channel]
                {
                    let data = AnalogData {
                        channels: vec![channel],
                        samples: vec![value],
                        mq: Mq::Temperature,
                        flags,
                        unit,
                        digits: 1,
                    };
                    let going_on = || !stop.load(Ordering::SeqCst);
                    if !sink.send_while(Packet::Analog(data), going_on) {
                        done = true;
                    }
                }
                count += 1;
                done |= self.limit_reached(count, started);
            });
            if done || self.gave_up_waiting(started, last_record) {
                return;
            }
        }
    }

    /// C's check, behind each record.
    fn limit_reached(&self, count: u64, started: Instant) -> bool {
        let samples = self.limits.samples.load(Ordering::SeqCst);
        let msec = self.limits.msec.load(Ordering::SeqCst);
        (samples > 0 && count >= samples)
            || (msec > 0 && started.elapsed() >= Duration::from_millis(msec))
    }

    /// Whether no record has come for longer than makes sense. Without a limit only a
    /// client's stop ends a capture, as it may want to wait.
    fn gave_up_waiting(&self, started: Instant, last_record: Instant) -> bool {
        let samples = self.limits.samples.load(Ordering::SeqCst);
        let msec = self.limits.msec.load(Ordering::SeqCst);
        let late = if msec > 0 {
            started.elapsed() >= Duration::from_millis(msec) + self.bounds.time_grace
        } else {
            samples > 0 && last_record.elapsed() >= self.bounds.silence
        };
        if late {
            log::warn!("No record from the device; ending the capture.");
        }
        late
    }
}

#[cfg(test)]
mod tests;
