// Counterpart in sigrok:
//   libsigrok/src/hardware/maynuo-m97/api.c,
//   libsigrok/src/hardware/maynuo-m97/protocol.c,
//   Copyright (C) 2015 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, François Revol,
//   Gerhard Sittig, Daniel Elstner, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Maynuo M97 and M98 series electronic loads, Modbus RTU slaves on a serial
//! line: the driver `maynuo-m97`.
//!
//! `conn=` names the port, `serialcomm=` overrides 9600/8n1 and `modbusaddr=`
//! the slave address 1. A scan reads the model and the version and closes the
//! port again. Of the many models a load can be, C's table has two it trusts,
//! so a scan finds those: see [`protocol::MODELS`].
//!
//! An open sets `PC1`, which locks the front panel, and a close clears it, and
//! so does a drop of the device, as the CLI's error paths drop it unclosed. What
//! does not get there leaves the panel locked: an unplug, a signal the CLI has
//! no handler for (`SIGTERM`, `SIGHUP`) and a crash. As in C, nothing looks at
//! how `PC1` went.
//!
//! The channels `V1` and `I1` are in the group `1`, which holds the settings
//! and the state; the limits are the device's. Nothing changes the load but what
//! `set` sends. `enabled` is three steps on the wire (the mode, on or off, the
//! mode again), and the CLI sends `enabled=on` last, once every setpoint was
//! taken.
//!
//! A capture reads `U` and `I` 10 ms after the last read ended, and each read
//! sends a frame. A setting during a capture goes out between two reads.
//!
//! Where C does not do what it means:
//!
//! - A capture asks. C sends no request in a capture and goes on reading the
//!   reply to it (`api.c:376-383`, `protocol.c:227`, which `modbus.c:437-438`
//!   says reads "the reply of a previously sent" one), so it has no samples:
//!   `--samples` never ends, and `--time` ends with nothing. The read here is
//!   what that reply would answer: four registers from `U`, the manual's two
//!   floats. No run of C or of a real load has shown it, see `docs/DEVICES.md`.
//! - A setting that is no number is refused, where C sends a NaN or an
//!   infinity. Any other value goes out, as in C: nothing says how a load takes
//!   one outside the list, and the model's limits in that list are a table's.
//! - A capture with only a sample limit ends after [`NO_REPLY_BOUND`] with no
//!   good read, as that limit may never come. Under `--time` or `--continuous` a
//!   device that went quiet is waited for and a capture goes on when it comes
//!   back. [`MAX_UNSENT_POLLS`] reads in a row whose request cannot go out end
//!   any capture: the port has gone.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::modbus::{self, Modbus, ScanOptions};
use crate::serial::{Params, Port};
use protocol::{Link, Model, Reading};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 3] = [Key::Conn, Key::Serialcomm, Key::Modbusaddr];
const CLASSES: [DeviceClass; 1] = [DeviceClass::ElectronicLoad];
const SERIALCOMM: &str = "9600/8n1";
const GROUP: &str = "1";

/// `devopts`.
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];

/// `devopts_cg`.
static GROUP_OPTIONS: [OptionDesc; 14] = [
    OptionDesc::new(Key::Enabled, Access::GET_SET),
    OptionDesc::new(Key::Regulation, Access::GET),
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::VoltageTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::OverVoltageProtectionEnabled, Access::GET),
    OptionDesc::new(Key::OverVoltageProtectionActive, Access::GET),
    OptionDesc::new(Key::OverVoltageProtectionThreshold, Access::GET_SET),
    OptionDesc::new(Key::OverCurrentProtectionEnabled, Access::GET),
    OptionDesc::new(Key::OverCurrentProtectionActive, Access::GET),
    OptionDesc::new(Key::OverCurrentProtectionThreshold, Access::GET_SET),
    OptionDesc::new(Key::OverTemperatureProtection, Access::GET),
    OptionDesc::new(Key::OverTemperatureProtectionActive, Access::GET),
];

/// The session source's timeout, `api.c`'s `dev_acquisition_start`: the time
/// from one read's end to the next.
const POLL_INTERVAL: Duration = Duration::from_millis(10);
/// How long a capture whose only limit is a sample count waits for a good read
/// before it ends: without one that limit never comes.
pub const NO_REPLY_BOUND: Duration = Duration::from_secs(5);
/// Reads in a row whose request cannot go out that end a capture.
pub const MAX_UNSENT_POLLS: u32 = 5;
/// How long the End waits for room in the queue once a stop has come. A client
/// that stops drains (`Session` does); one that does not must not hold up a close.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How often a wait between two reads looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(2);

const CHANNEL_V: usize = 0;
const CHANNEL_I: usize = 1;
const VOLTAGE_DIGITS: i32 = 3;
const CURRENT_DIGITS: i32 = 4;
/// The lists' steps, `api.c`'s `config_list`.
const VOLTAGE_STEP: f64 = 0.001;
const CURRENT_STEP: f64 = 0.0001;

/// The `maynuo-m97` driver.
pub struct MaynuoM97Driver;

impl Driver for MaynuoM97Driver {
    fn id(&self) -> &'static str {
        "maynuo-m97"
    }

    fn long_name(&self) -> &'static str {
        "maynuo M97/M98 series"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let options = ScanOptions::parse(options, SERIALCOMM)?;
        let Some(conn) = options.conn else {
            return Ok(Vec::new());
        };
        // Every failure of the port here is no device: `sr_modbus_scan_resource`.
        let params = match Params::parse(&options.serialcomm) {
            Ok(params) => params,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        let mut bus = match modbus::open(&conn, params, options.slave) {
            Ok(bus) => bus,
            Err(e) => {
                log::info!("Couldn't open Modbus device: {e}.");
                return Ok(Vec::new());
            }
        };
        match probe(&mut bus) {
            Ok(Some((model, version))) => {
                Ok(vec![Box::new(MaynuoDevice::new(model, version, conn, params, options.slave))])
            }
            Ok(None) => Ok(Vec::new()),
            Err(e) => {
                log::debug!("Probe failed: {e}.");
                Ok(Vec::new())
            }
        }
    }
}

/// `probe_device`. The port goes when the scan returns, as
/// `sr_modbus_scan_resource` closes it, and an open opens it again.
fn probe(bus: &mut Modbus) -> Result<Option<(&'static Model, u16)>> {
    let (id, version) = protocol::read_identity(bus)?;
    let Some(model) = protocol::lookup(id) else {
        log::error!("Unknown model: {id}.");
        return Ok(None);
    };
    Ok(Some((model, version)))
}

/// One load.
pub struct MaynuoDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    conn: String,
    params: Params,
    slave: u8,
    limit_samples: u64,
    limit_msec: u64,
    no_reply_bound: Duration,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl MaynuoDevice {
    fn new(model: &'static Model, version: u16, conn: String, params: Params, slave: u8) -> Self {
        MaynuoDevice {
            info: DeviceInfo {
                driver: "maynuo-m97",
                vendor: "Maynuo".to_owned(),
                model: model.name.to_owned(),
                version: protocol::version_text(version),
                serial: String::new(),
                conn: None,
            },
            channels: vec![
                Channel::new(CHANNEL_V, ChannelKind::Analog, "V1".to_owned()),
                Channel::new(CHANNEL_I, ChannelKind::Analog, "I1".to_owned()),
            ],
            groups: vec![ChannelGroup::new(GROUP, vec![CHANNEL_V, CHANNEL_I])],
            conn,
            params,
            slave,
            limit_samples: 0,
            limit_msec: 0,
            no_reply_bound: NO_REPLY_BOUND,
            pending: None,
            link: Arc::new(Mutex::new(Link::new(None, model))),
            worker: None,
        }
    }

    /// A device of `model` on a port that stands in for the hardware, without
    /// a scan's reading. [`Device::open`] takes it.
    pub fn from_port(model: &'static Model, port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device = Self::new(model, 0, conn, params, modbus::DEFAULT_SLAVE);
        device.pending = Some(port);
        device
    }

    fn link(&self) -> MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    fn float(&self, address: u16) -> Result<Value> {
        Ok(Value::F64(f64::from(self.link().get_float(address)?)))
    }

    fn bit(&self, address: u16) -> Result<Value> {
        Ok(Value::Bool(self.link().get_bit(address)?))
    }

    /// `maynuo_m97_set_float`'s value: what a float holds, and is a number.
    fn setpoint(key: &Key, value: &Value) -> Result<f32> {
        match value.as_f64().map(|v| v as f32) {
            Some(v) if v.is_finite() => Ok(v),
            _ => Err(Error::Arg(format!("invalid value for {}", key.id()))),
        }
    }
}

impl Device for MaynuoDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &self.groups
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
            Some(GROUP) => Ok(GROUP_OPTIONS.to_vec()),
            Some(other) => Err(Error::ChannelGroup(other.to_owned())),
        }
    }

    /// `config_get`, which does not look at the group.
    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        match key {
            Key::LimitSamples => Ok(Value::U64(self.limit_samples)),
            Key::LimitMsec => Ok(Value::U64(self.limit_msec)),
            Key::Enabled => self.bit(protocol::ISTATE),
            Key::Regulation => {
                let mut link = self.link();
                if link.get_bit(protocol::UNREG)? {
                    Ok(Value::from("UR"))
                } else {
                    Ok(Value::from(protocol::mode_name(link.get_mode()?)))
                }
            }
            Key::Voltage => self.float(protocol::U),
            Key::VoltageTarget => self.float(protocol::UFIX),
            Key::Current => self.float(protocol::I),
            Key::CurrentLimit => self.float(protocol::IFIX),
            Key::OverVoltageProtectionEnabled
            | Key::OverCurrentProtectionEnabled
            | Key::OverTemperatureProtection => Ok(Value::Bool(true)),
            Key::OverVoltageProtectionActive => self.bit(protocol::UOVER),
            Key::OverVoltageProtectionThreshold => self.float(protocol::UMAX),
            Key::OverCurrentProtectionActive => self.bit(protocol::IOVER),
            Key::OverCurrentProtectionThreshold => self.float(protocol::IMAX),
            Key::OverTemperatureProtectionActive => self.bit(protocol::HEAT),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `config_set`, which does not look at the group.
    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        // `core::config_set` refuses a 0 and says so; this is for a library
        // caller that comes straight here.
        match key {
            Key::LimitSamples => {
                self.limit_samples = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                Ok(())
            }
            Key::LimitMsec => {
                self.limit_msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                Ok(())
            }
            Key::Enabled => {
                let enable = value.as_bool().ok_or_else(bad)?;
                self.link().set_input(enable)
            }
            Key::VoltageTarget => {
                let v = Self::setpoint(key, value)?;
                self.link().set_float(protocol::UFIX, v)
            }
            Key::CurrentLimit => {
                let v = Self::setpoint(key, value)?;
                self.link().set_float(protocol::IFIX, v)
            }
            Key::OverVoltageProtectionThreshold => {
                let v = Self::setpoint(key, value)?;
                self.link().set_float(protocol::UMAX, v)
            }
            Key::OverCurrentProtectionThreshold => {
                let v = Self::setpoint(key, value)?;
                self.link().set_float(protocol::IMAX, v)
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `config_list`: the group's two ranges.
    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let model = self.link().model;
        match (key, group) {
            (Key::VoltageTarget, Some(_)) => Ok(ValueList::RangeStepF64 {
                min: 0.0,
                max: f64::from(model.max_voltage),
                step: VOLTAGE_STEP,
            }),
            (Key::CurrentLimit, Some(_)) => Ok(ValueList::RangeStepF64 {
                min: 0.0,
                max: f64::from(model.max_current),
                step: CURRENT_STEP,
            }),
            (other, _) => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `dev_open`: the port, and `PC1`, however that goes.
    fn open(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        if link.modbus.is_some() {
            return Ok(());
        }
        let bus = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                Modbus::new(port, self.slave)
            }
            None => modbus::open(&self.conn, self.params, self.slave)?,
        };
        link.modbus = Some(bus);
        if let Err(e) = link.set_bit(protocol::PC1, true) {
            log::debug!("Could not lock the panel: {e}.");
        }
        Ok(())
    }

    /// `dev_close`: `PC1`, however that goes, and the port.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let mut link = self.link();
        if link.modbus.is_some() {
            if let Err(e) = link.set_bit(protocol::PC1, false) {
                log::debug!("Could not unlock the panel: {e}.");
            }
            link.modbus = None;
        }
        Ok(())
    }

    /// `dev_acquisition_start`: the thread, whose Header goes first.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(worker) = &self.worker {
            if !worker.handle.is_finished() {
                return Err(Error::Busy);
            }
            let worker = self.worker.take().expect("just seen");
            let _ = worker.handle.join();
        }
        if self.link().modbus.is_none() {
            return Err(Error::device("the device is not open"));
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            interval: POLL_INTERVAL,
            no_reply_bound: self.no_reply_bound,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("maynuo-m97 acquisition".to_owned())
            .spawn(move || {
                capture.run(&sink, &stop2);
                send_end(&sink, &stop2);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// The CLI closes only on its way out that went well, and sigrok-cli on most:
/// a device dropped open must not leave a panel locked.
impl Drop for MaynuoDevice {
    fn drop(&mut self) {
        if let Err(e) = self.close() {
            log::debug!("Could not close the device: {e}.");
        }
    }
}

/// One capture, on its own thread: `maynuo_m97_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    limit_msec: u64,
    interval: Duration,
    no_reply_bound: Duration,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return;
        }
        let started = Instant::now();
        let limit = Duration::from_millis(self.limit_msec);
        let only_samples_can_end = self.limit_samples > 0 && self.limit_msec == 0;
        let mut samples = 0u64;
        let mut last_good = started;
        let mut unsent = 0;
        loop {
            if !wait(self.interval, stop) {
                return;
            }
            // The lock is held for the read alone, so that a setting gets in
            // between two, and the packets go out with no lock.
            let polled = {
                let mut link = self.link.lock().expect("link lock");
                let polled = link.poll(stop);
                unsent = if link.request_unsent() { unsent + 1 } else { 0 };
                polled
            };
            match polled {
                Ok(reading) => {
                    last_good = Instant::now();
                    if !frame_packets(reading).into_iter().all(|p| sink.send_while(p, go_on)) {
                        return;
                    }
                    samples += 1;
                }
                Err(e) => {
                    log::debug!("A read failed: {e}.");
                    if unsent >= MAX_UNSENT_POLLS {
                        log::error!("Acquisition failed: {unsent} requests could not go out.");
                        return;
                    }
                    if only_samples_can_end && last_good.elapsed() >= self.no_reply_bound {
                        log::error!("No reply from the device for {:?}.", self.no_reply_bound);
                        return;
                    }
                }
            }
            if self.limit_samples > 0 && samples >= self.limit_samples {
                return;
            }
            if self.limit_msec > 0 && started.elapsed() >= limit {
                return;
            }
        }
    }
}

/// The End, for which a client that goes on reading is waited for without a
/// bound, and after a stop only for [`END_AFTER_STOP`].
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

/// Waits for `time`: false when a stop came first.
fn wait(time: Duration, stop: &AtomicBool) -> bool {
    let due = Instant::now() + time;
    loop {
        if stop.load(Ordering::SeqCst) {
            return false;
        }
        let left = due.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return true;
        }
        std::thread::sleep(left.min(STOP_CHECK));
    }
}

fn analog(channel: usize, value: f32, mq: Mq, unit: Unit, digits: i32) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![channel],
        samples: vec![value],
        mq,
        flags: MqFlags::DC,
        unit,
        digits,
    })
}

/// A frame of one reading.
fn frame_packets(reading: Reading) -> [Packet; 4] {
    [
        Packet::FrameBegin,
        analog(CHANNEL_V, reading.voltage, Mq::Voltage, Unit::Volt, VOLTAGE_DIGITS),
        analog(CHANNEL_I, reading.current, Mq::Current, Unit::Ampere, CURRENT_DIGITS),
        Packet::FrameEnd,
    ]
}

#[cfg(test)]
mod tests;
