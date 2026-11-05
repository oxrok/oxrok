// Counterpart in sigrok:
//   libsigrok/src/hardware/rdtech-dps/api.c,
//   libsigrok/src/hardware/rdtech-dps/protocol.c,
//   Copyright (C) 2018 James Churchill <pelrun@gmail.com>,
//   Copyright (C) 2019 Frank Stettner <frank-stettner@gmx.net>,
//   Copyright (C) 2021 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Mikael Djurfeldt, Soeren Apel, Uwe Hermann, v1ne,
//   Volodymyr Ananiev
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `RDTech` (Riden) DPS, DPH and RD power supplies, Modbus RTU slaves on a
//! USB to UART bridge: the drivers `rdtech-dps` and `rdtech-rd`, one module as
//! in C.
//!
//! `conn=` names the port, `serialcomm=` overrides 9600/8n1 (DPS) or
//! 115200/8n1 (RD), and `modbusaddr=` the slave address 1. A scan reads the
//! model and the version and closes the port again.
//!
//! An open of a DPS locks its panel, and a close unlocks it, and so does a drop
//! of the device, as the CLI's error paths drop it unclosed where sigrok-cli
//! closes it. What does not get there leaves the panel locked: an unplug, a
//! signal the CLI has no handler for (`SIGTERM` and `SIGHUP`, as `ctrlc` is built
//! without `termination`; `SIGINT` before its handler is set) and a crash.
//! Nothing else changes the supply but what `set` sends, one register for each
//! setting.
//! Each `get` reads all of the state. In what order settings go out is the
//! caller's: the CLI sends `enabled=on` last, once every setpoint was taken.
//!
//! A capture reads the state 10 ms after the last read ended, and each read
//! sends a frame of V, I and P, then metadata for what has changed. A setting
//! during a capture goes out between two reads.
//!
//! Where C does not do what it means:
//!
//! - A model with one range has its scale, see [`protocol::Link::new`]; C has
//!   none, on every DPS and every RD but the RD6012P.
//! - A DPS's thresholds read in their scale, see [`protocol::decode_dps`].
//! - A setting rounds, and what the register does not hold, or the model's
//!   list does not, is refused, see [`protocol::register_value`].
//! - A capture ends after [`MAX_FAILED_POLLS`] reads in a row that fail. C goes
//!   on for ever, as its callback's negative result counts as true to the
//!   session and no limit is checked on that path (`protocol.c:740-741`).
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::modbus::{self, Modbus, ScanOptions};
use crate::serial::{Params, Port};
use protocol::{Family, Identity, Link, Model, Setting, State};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 3] = [Key::Conn, Key::Serialcomm, Key::Modbusaddr];
const CLASSES: [DeviceClass; 1] = [DeviceClass::PowerSupply];

/// `devopts_w_range`; `devopts` is all but the last.
static DEV_OPTIONS: [OptionDesc; 14] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::VoltageTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::Enabled, Access::GET_SET),
    OptionDesc::new(Key::Regulation, Access::GET),
    OptionDesc::new(Key::OverVoltageProtectionActive, Access::GET),
    OptionDesc::new(Key::OverVoltageProtectionThreshold, Access::GET_SET),
    OptionDesc::new(Key::OverCurrentProtectionActive, Access::GET),
    OptionDesc::new(Key::OverCurrentProtectionThreshold, Access::GET_SET),
    OptionDesc::new(Key::Range, Access::GET_SET_LIST),
];

/// The session source's timeout, `api.c`'s `dev_acquisition_start`: the time
/// from one read's end to the next.
const POLL_INTERVAL: Duration = Duration::from_millis(10);
/// Reads in a row that fail, each of three tries, that end a capture.
pub const MAX_FAILED_POLLS: u32 = 5;
/// How often a wait between two reads looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(2);

const CHANNEL_V: usize = 0;
const CHANNEL_I: usize = 1;
const CHANNEL_P: usize = 2;
const POWER_DIGITS: u32 = 2;

/// The `rdtech-dps` driver.
pub struct RdtechDpsDriver;
/// The `rdtech-rd` driver.
pub struct RdtechRdDriver;

fn scan(
    options: &[(Key, Value)],
    family: Family,
    serialcomm: &str,
) -> Result<Vec<Box<dyn Device>>> {
    let options = ScanOptions::parse(options, serialcomm)?;
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
    let bus = match modbus::open(&conn, params, options.slave) {
        Ok(bus) => bus,
        Err(e) => {
            log::info!("Couldn't open Modbus device: {e}.");
            return Ok(Vec::new());
        }
    };
    match probe(bus, family) {
        Ok(Some((link, identity))) => {
            Ok(vec![Box::new(RdtechDevice::new(link, identity, conn, params, options.slave))])
        }
        Ok(None) => Ok(Vec::new()),
        Err(e) => {
            log::debug!("Probe failed: {e}.");
            Ok(Vec::new())
        }
    }
}

/// `probe_device`: the model, the version, and the range of a model with more
/// than one. The port goes when this returns, as `sr_modbus_scan_resource`
/// closes it, and an open opens it again.
fn probe(mut bus: Modbus, family: Family) -> Result<Option<(Link, Identity)>> {
    let identity = protocol::read_identity(&mut bus, family)?;
    let Some(model) = protocol::lookup(family, identity.id) else {
        log::error!("Unknown model: {}{}.", family.prefix(), identity.id);
        return Ok(None);
    };
    let mut link = Link::new(Some(bus), model);
    link.update_range()?;
    link.modbus = None;
    Ok(Some((link, identity)))
}

impl Driver for RdtechDpsDriver {
    fn id(&self) -> &'static str {
        "rdtech-dps"
    }

    fn long_name(&self) -> &'static str {
        "RDTech DPS/DPH series power supply"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        scan(options, Family::Dps, "9600/8n1")
    }
}

impl Driver for RdtechRdDriver {
    fn id(&self) -> &'static str {
        "rdtech-rd"
    }

    fn long_name(&self) -> &'static str {
        "RDTech RD series power supply"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        scan(options, Family::Rd, "115200/8n1")
    }
}

/// One power supply.
pub struct RdtechDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    slave: u8,
    limit_samples: u64,
    limit_msec: u64,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl RdtechDevice {
    fn new(link: Link, identity: Identity, conn: String, params: Params, slave: u8) -> Self {
        let model = link.model;
        let driver = match model.family {
            Family::Dps => "rdtech-dps",
            Family::Rd => "rdtech-rd",
        };
        let serial = match (model.family, identity.serial) {
            (Family::Rd, n) if n != 0 => n.to_string(),
            _ => String::new(),
        };
        RdtechDevice {
            info: DeviceInfo {
                driver,
                vendor: "RDTech".to_owned(),
                model: model.name.to_owned(),
                version: identity.version_text(model.family),
                serial,
                conn: None,
            },
            channels: vec![
                Channel::new(CHANNEL_V, ChannelKind::Analog, "V".to_owned()),
                Channel::new(CHANNEL_I, ChannelKind::Analog, "I".to_owned()),
                Channel::new(CHANNEL_P, ChannelKind::Analog, "P".to_owned()),
            ],
            conn,
            params,
            slave,
            limit_samples: 0,
            limit_msec: 0,
            pending: None,
            link: Arc::new(Mutex::new(link)),
            worker: None,
        }
    }

    /// A device of `model` on a port that stands in for the hardware, without
    /// a scan's reading. [`Device::open`] takes it.
    pub fn from_port(model: &'static Model, port: Box<dyn Port>) -> Self {
        let params = Params::parse("9600/8n1").expect("the default parses");
        let conn = port.name().to_owned();
        let identity = Identity { id: model.id, version: 0, serial: 0 };
        let link = Link::new(None, model);
        let mut device = Self::new(link, identity, conn, params, modbus::DEFAULT_SLAVE);
        device.pending = Some(port);
        device
    }

    fn link(&self) -> MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    fn state(&self) -> Result<State> {
        self.link().read_state()
    }

    /// A value of `key` within what the model's list says, -0.0 being 0.
    fn in_list(key: &Key, value: &Value, max: f64) -> Result<f64> {
        match value.as_f64() {
            Some(v) if (0.0..=max).contains(&v) => Ok(v + 0.0),
            _ => Err(Error::Arg(format!("{} out of range", key.id()))),
        }
    }

    fn number(key: &Key, value: &Value) -> Result<f64> {
        value.as_f64().ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())))
    }
}

fn no_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        None => Ok(()),
    }
}

impl Device for RdtechDevice {
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
        let ranges = self.link().model.ranges.len();
        let count = if ranges > 1 { DEV_OPTIONS.len() } else { DEV_OPTIONS.len() - 1 };
        Ok(DEV_OPTIONS[..count].to_vec())
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        no_group(group)?;
        let number = |v: f32| Ok(Value::F64(f64::from(v)));
        match key {
            Key::LimitSamples => Ok(Value::U64(self.limit_samples)),
            Key::LimitMsec => Ok(Value::U64(self.limit_msec)),
            Key::Enabled => Ok(Value::Bool(self.state()?.output_enabled)),
            Key::Regulation => {
                Ok(Value::from(if self.state()?.regulation_cc { "CC" } else { "CV" }))
            }
            Key::Voltage => number(self.state()?.voltage),
            Key::VoltageTarget => number(self.state()?.voltage_target),
            Key::Current => number(self.state()?.current),
            Key::CurrentLimit => number(self.state()?.current_limit),
            Key::OverVoltageProtectionActive => Ok(Value::Bool(self.state()?.protect_ovp)),
            Key::OverVoltageProtectionThreshold => number(self.state()?.ovp_threshold),
            Key::OverCurrentProtectionActive => Ok(Value::Bool(self.state()?.protect_ocp)),
            Key::OverCurrentProtectionThreshold => number(self.state()?.ocp_threshold),
            Key::Range if self.link().model.ranges.len() > 1 => {
                let state = self.state()?;
                Ok(Value::from(self.link().model.ranges[state.range].name))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let range = *self.link().current_range();
        let setting = match key {
            // `core::config_set` refuses a 0 and says so; this is for a library
            // caller that comes straight here.
            Key::LimitSamples => {
                self.limit_samples = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                return Ok(());
            }
            Key::LimitMsec => {
                self.limit_msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                return Ok(());
            }
            Key::Enabled => Setting::Output(value.as_bool().ok_or_else(bad)?),
            Key::VoltageTarget => {
                Setting::VoltageTarget(Self::in_list(key, value, range.max_voltage)?)
            }
            Key::CurrentLimit => {
                Setting::CurrentLimit(Self::in_list(key, value, range.max_current)?)
            }
            Key::OverVoltageProtectionThreshold => Setting::OvpThreshold(Self::number(key, value)?),
            Key::OverCurrentProtectionThreshold => Setting::OcpThreshold(Self::number(key, value)?),
            Key::Range if self.link().model.ranges.len() > 1 => {
                let name = value.as_str().ok_or_else(bad)?;
                let ranges = self.link().model.ranges;
                let at = ranges.iter().position(|r| r.name == name);
                Setting::Range(at.ok_or_else(|| Error::not_supported("no such range"))?)
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        };
        self.link().set(setting)
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        let mut link = self.link();
        let model = link.model;
        match key {
            Key::VoltageTarget | Key::CurrentLimit => {
                // As `config_list`, which drops how this went.
                if let Err(e) = link.update_range() {
                    log::debug!("Could not read the range: {e}.");
                }
                let range = link.current_range();
                let (max, step) = if *key == Key::VoltageTarget {
                    (range.max_voltage, 1.0 / link.multipliers.voltage)
                } else {
                    (range.max_current, 1.0 / link.multipliers.current)
                };
                Ok(ValueList::RangeStepF64 { min: 0.0, max, step })
            }
            Key::Range if model.ranges.len() > 1 => {
                Ok(ValueList::Str(model.ranges.iter().map(|r| r.name.to_owned()).collect()))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `dev_open`: the port, and the panel's lock on a DPS. A port on which
    /// that fails goes. C leaves it there, with no close to follow; but the
    /// reply is what may have been lost, with the lock taken, so here it tries
    /// the unlock first.
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
        if let Err(e) = link.set(Setting::Lock(true)) {
            if let Err(unlock) = link.set(Setting::Lock(false)) {
                log::debug!("Could not unlock the panel: {unlock}.");
            }
            link.modbus = None;
            return Err(e);
        }
        Ok(())
    }

    /// `dev_close`: the panel's unlock, however it goes, and the port.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let mut link = self.link();
        if link.modbus.is_some() {
            if let Err(e) = link.set(Setting::Lock(false)) {
                log::debug!("Could not unlock the panel: {e}.");
            }
            link.modbus = None;
        }
        Ok(())
    }

    /// `dev_acquisition_start`: what the capture compares with, then the
    /// thread, whose Header goes first.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(worker) = &self.worker {
            if !worker.handle.is_finished() {
                return Err(Error::Busy);
            }
            let worker = self.worker.take().expect("just seen");
            let _ = worker.handle.join();
        }
        {
            let mut link = self.link();
            link.acquiring = true;
            if let Err(e) = link.seed() {
                link.acquiring = false;
                return Err(e);
            }
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            interval: POLL_INTERVAL,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let spawned = std::thread::Builder::new().name("rdtech-dps acquisition".to_owned()).spawn(
            move || {
                capture.run(&sink, &stop2);
                capture.link.lock().expect("link lock").acquiring = false;
                sink.send(Packet::End);
            },
        );
        match spawned {
            Ok(handle) => {
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                self.link().acquiring = false;
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

/// The CLI closes only on its way out that went well, and sigrok-cli on every
/// one: a device dropped open must not leave a panel locked.
impl Drop for RdtechDevice {
    fn drop(&mut self) {
        if let Err(e) = self.close() {
            log::debug!("Could not close the device: {e}.");
        }
    }
}

/// One capture, on its own thread: `rdtech_dps_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    limit_msec: u64,
    interval: Duration,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) {
        if !sink.send(Packet::header()) {
            return;
        }
        let started = Instant::now();
        let limit = Duration::from_millis(self.limit_msec);
        let mut samples = 0u64;
        let mut failed = 0;
        loop {
            if !wait(self.interval, stop) {
                return;
            }
            // The lock is held for the read alone, so that a setting gets in
            // between two, and the packets go out with no lock.
            let polled = {
                let mut link = self.link.lock().expect("link lock");
                link.read_state().map(|state| (state, poll_packets(&mut link, &state)))
            };
            match polled {
                Ok((_, packets)) => {
                    failed = 0;
                    if !packets.into_iter().all(|p| sink.send(p)) {
                        return;
                    }
                    samples += 1;
                }
                Err(e) => {
                    failed += 1;
                    log::debug!("A read failed: {e}.");
                    if failed >= MAX_FAILED_POLLS {
                        log::error!("Acquisition failed: {failed} reads in a row failed.");
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

fn analog(channel: usize, value: f32, mq: Mq, flags: MqFlags, unit: Unit, digits: u32) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![channel],
        samples: vec![value],
        mq,
        flags,
        unit,
        digits: i32::try_from(digits).expect("a few digits"),
    })
}

/// The frame of a state, and what has changed since the last in
/// `rdtech_dps_receive_data`'s order. The frame has the digits of the range
/// the values were read in, so a change of range shows only in the next frame.
fn poll_packets(link: &mut Link, state: &State) -> Vec<Packet> {
    let range = link.current_range();
    let mut packets = vec![
        Packet::FrameBegin,
        analog(
            CHANNEL_V,
            state.voltage,
            Mq::Voltage,
            MqFlags::DC,
            Unit::Volt,
            range.voltage_digits,
        ),
        analog(
            CHANNEL_I,
            state.current,
            Mq::Current,
            MqFlags::DC,
            Unit::Ampere,
            range.current_digits,
        ),
        analog(CHANNEL_P, state.power, Mq::Power, MqFlags::empty(), Unit::Watt, POWER_DIGITS),
        Packet::FrameEnd,
    ];
    let last = link.last;
    if last.protect_ovp != state.protect_ovp {
        packets.push(Packet::meta(Key::OverVoltageProtectionActive, state.protect_ovp));
    }
    if last.protect_ocp != state.protect_ocp {
        packets.push(Packet::meta(Key::OverCurrentProtectionActive, state.protect_ocp));
    }
    if last.regulation_cc != state.regulation_cc {
        packets.push(Packet::meta(Key::Regulation, if state.regulation_cc { "CC" } else { "CV" }));
    }
    if last.output_enabled != state.output_enabled {
        packets.push(Packet::meta(Key::Enabled, state.output_enabled));
    }
    if link.range != state.range {
        packets.push(Packet::meta(Key::Range, link.model.ranges[state.range].name));
        link.set_range(state.range);
    }
    link.last = *state;
    packets
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::modbus::{crc16, frame};
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit
    /// that does not bite must fail, not hang the run.
    const CAPTURE_BOUND: Duration = Duration::from_secs(15);

    /// What the fake saw.
    #[derive(Default)]
    struct Seen {
        writes: Vec<(Instant, Vec<u8>)>,
    }

    type Shared<T> = Arc<Mutex<T>>;

    /// A supply's registers, answering what it is asked.
    struct Fake {
        seen: Arc<Mutex<Seen>>,
        registers: Arc<Mutex<[u16; 256]>>,
        input: VecDeque<u8>,
        /// Answer no more frames than these.
        answers: Option<usize>,
        /// Each read of the poll block, the protect register's next value.
        protect_at: u16,
        protect: VecDeque<u16>,
    }

    impl Fake {
        fn new() -> (Fake, Shared<Seen>, Shared<[u16; 256]>) {
            let seen = Arc::new(Mutex::new(Seen::default()));
            let registers = Arc::new(Mutex::new([0; 256]));
            let fake = Fake {
                seen: Arc::clone(&seen),
                registers: Arc::clone(&registers),
                input: VecDeque::new(),
                answers: None,
                protect_at: 0,
                protect: VecDeque::new(),
            };
            (fake, seen, registers)
        }

        fn answer(&mut self, data: &[u8]) -> Vec<u8> {
            let (body, crc) = data.split_at(data.len() - 2);
            assert_eq!(crc16(body).to_le_bytes(), crc, "a frame's CRC");
            let address = usize::from(u16::from_be_bytes([body[2], body[3]]));
            let count = usize::from(u16::from_be_bytes([body[4], body[5]]));
            let mut registers = self.registers.lock().unwrap();
            match body[1] {
                3 => {
                    if count > 2
                        && let Some(value) = self.protect.pop_front()
                    {
                        registers[usize::from(self.protect_at)] = value;
                    }
                    let mut out = vec![3, u8::try_from(count * 2).unwrap()];
                    for r in &registers[address..address + count] {
                        out.extend_from_slice(&r.to_be_bytes());
                    }
                    frame(1, &out)
                }
                0x10 => {
                    registers[address] = u16::from_be_bytes([body[7], body[8]]);
                    frame(1, &body[1..6])
                }
                other => panic!("function {other}"),
            }
        }
    }

    impl Port for Fake {
        fn name(&self) -> &'static str {
            "/dev/ttyFAKE"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            if self.input.is_empty() {
                std::thread::sleep(timeout.min(Duration::from_millis(1)));
                return Ok(0);
            }
            let n = buf.len().min(self.input.len());
            for b in &mut buf[..n] {
                *b = self.input.pop_front().expect("counted");
            }
            Ok(n)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let number = {
                let mut seen = self.seen.lock().unwrap();
                seen.writes.push((Instant::now(), data.to_vec()));
                seen.writes.len()
            };
            if self.answers.is_none_or(|n| number <= n) {
                let answer = self.answer(data);
                self.input.extend(answer);
            }
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.input.clear();
            Ok(())
        }
    }

    fn model(family: Family, id: u16) -> &'static Model {
        protocol::lookup(family, id).unwrap()
    }

    fn write_frame(address: u16, value: u16) -> Vec<u8> {
        let [ah, al] = address.to_be_bytes();
        let [vh, vl] = value.to_be_bytes();
        frame(1, &[0x10, ah, al, 0, 1, 2, vh, vl])
    }

    fn writes(seen: &Arc<Mutex<Seen>>) -> Vec<Vec<u8>> {
        seen.lock().unwrap().writes.iter().map(|(_, w)| w.clone()).collect()
    }

    /// Only the frames that write, without the panel's lock.
    fn settings(seen: &Arc<Mutex<Seen>>) -> Vec<Vec<u8>> {
        let lock = [write_frame(6, 1), write_frame(6, 0)];
        writes(seen).into_iter().filter(|w| w[1] == 0x10 && !lock.contains(w)).collect()
    }

    fn dps(fake: Fake) -> RdtechDevice {
        let mut d = RdtechDevice::from_port(model(Family::Dps, 5005), Box::new(fake));
        d.open().unwrap();
        d
    }

    fn rd6012p(fake: Fake) -> RdtechDevice {
        let mut d = RdtechDevice::from_port(model(Family::Rd, 60125), Box::new(fake));
        d.open().unwrap();
        d
    }

    fn packets_until_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
        let deadline = Instant::now() + CAPTURE_BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            assert!(!left.is_zero(), "no End after {CAPTURE_BOUND:?}");
            let (_, p) = rx.recv_timeout(left).expect("a packet in time");
            let done = matches!(p, Packet::End);
            out.push(p);
            if done {
                return out;
            }
        }
    }

    fn capture(d: &mut RdtechDevice) -> Vec<Packet> {
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        packets_until_end(&rx)
    }

    fn set(d: &mut RdtechDevice, key: &Key, value: impl Into<Value>) -> Result<()> {
        d.set(key, &value.into(), None)
    }

    #[test]
    fn an_open_of_a_dps_locks_and_a_close_unlocks() {
        let (fake, seen, _) = Fake::new();
        let mut d = dps(fake);
        assert_eq!(writes(&seen), [write_frame(0x06, 1)]);
        d.close().unwrap();
        assert_eq!(writes(&seen), [write_frame(0x06, 1), write_frame(0x06, 0)]);
        d.close().unwrap();
        assert_eq!(writes(&seen).len(), 2, "a second close has no port to write to");
    }

    #[test]
    fn an_rd_has_no_lock_on_the_wire() {
        let (fake, seen, _) = Fake::new();
        let mut d = rd6012p(fake);
        d.close().unwrap();
        assert_eq!(writes(&seen), Vec::<Vec<u8>>::new());
    }

    #[test]
    fn a_lost_reply_to_the_lock_fails_the_open_after_an_unlock() {
        let (mut fake, seen, _) = Fake::new();
        fake.answers = Some(0);
        let mut d = RdtechDevice::from_port(model(Family::Dps, 5005), Box::new(fake));
        assert!(d.open().is_err());
        assert_eq!(writes(&seen), [write_frame(0x06, 1), write_frame(0x06, 0)]);
        d.close().unwrap();
        assert_eq!(writes(&seen).len(), 2, "the port has gone: nothing more");
    }

    #[test]
    fn a_device_dropped_open_unlocks_the_panel() {
        let (fake, seen, _) = Fake::new();
        let d = dps(fake);
        drop(d);
        assert_eq!(writes(&seen), [write_frame(0x06, 1), write_frame(0x06, 0)]);
    }

    #[test]
    fn a_device_closed_and_dropped_unlocks_once() {
        let (fake, seen, _) = Fake::new();
        let mut d = dps(fake);
        d.close().unwrap();
        drop(d);
        assert_eq!(writes(&seen).len(), 2);
    }

    #[test]
    fn a_device_never_opened_sends_nothing_when_dropped() {
        let (fake, seen, _) = Fake::new();
        drop(RdtechDevice::from_port(model(Family::Dps, 5005), Box::new(fake)));
        assert_eq!(writes(&seen).len(), 0);
    }

    #[test]
    fn a_failed_unlock_does_not_fail_the_close() {
        let (mut fake, seen, _) = Fake::new();
        fake.answers = Some(1);
        let mut d = dps(fake);
        d.close().unwrap();
        assert_eq!(writes(&seen).len(), 2);
    }

    #[test]
    fn a_setting_goes_to_its_register_in_its_scale() {
        let (fake, seen, _) = Fake::new();
        let mut d = dps(fake);
        set(&mut d, &Key::VoltageTarget, 3.3).unwrap();
        set(&mut d, &Key::CurrentLimit, 1.25).unwrap();
        set(&mut d, &Key::OverVoltageProtectionThreshold, 45.0).unwrap();
        set(&mut d, &Key::OverCurrentProtectionThreshold, 5.5).unwrap();
        set(&mut d, &Key::Enabled, true).unwrap();
        assert_eq!(
            settings(&seen),
            [
                write_frame(0x00, 330),
                write_frame(0x01, 1250),
                write_frame(0x52, 4500),
                write_frame(0x53, 5500),
                write_frame(0x09, 1),
            ],
            "each alone, in the order given, the output where it was asked"
        );
        let (fake, seen, _) = Fake::new();
        let mut d = rd6012p(fake);
        set(&mut d, &Key::VoltageTarget, 5.0).unwrap();
        set(&mut d, &Key::CurrentLimit, 1.0).unwrap();
        set(&mut d, &Key::OverVoltageProtectionThreshold, 6.0).unwrap();
        set(&mut d, &Key::OverCurrentProtectionThreshold, 2.0).unwrap();
        set(&mut d, &Key::Enabled, false).unwrap();
        assert_eq!(
            settings(&seen),
            [
                write_frame(8, 5000),
                write_frame(9, 10000),
                write_frame(82, 6000),
                write_frame(83, 20000),
                write_frame(18, 0),
            ]
        );
    }

    #[test]
    fn what_the_list_does_not_hold_is_refused_and_not_sent() {
        for (key, value) in [
            (Key::VoltageTarget, 50.01),
            (Key::VoltageTarget, -1.0),
            (Key::VoltageTarget, f64::NAN),
            (Key::CurrentLimit, 5.001),
            (Key::CurrentLimit, f64::NAN),
            (Key::OverVoltageProtectionThreshold, 655.36),
            (Key::OverVoltageProtectionThreshold, -1.0),
            (Key::OverCurrentProtectionThreshold, 65.536),
            (Key::OverCurrentProtectionThreshold, f64::NAN),
        ] {
            let (fake, seen, _) = Fake::new();
            let mut d = dps(fake);
            assert!(set(&mut d, &key, value).is_err(), "{key:?} {value}");
            assert_eq!(settings(&seen), Vec::<Vec<u8>>::new(), "{key:?} {value}");
        }
    }

    #[test]
    fn the_limit_of_a_list_goes_with_the_range() {
        let (fake, seen, registers) = Fake::new();
        registers.lock().unwrap()[20] = 1;
        let mut d = rd6012p(fake);
        assert!(set(&mut d, &Key::CurrentLimit, 12.0).is_err(), "not read yet: 6 A");
        d.list(&Key::CurrentLimit, None).unwrap();
        set(&mut d, &Key::CurrentLimit, 12.0).unwrap();
        assert_eq!(settings(&seen), [write_frame(9, 12000)]);
    }

    #[test]
    fn minus_zero_is_zero() {
        let (fake, seen, _) = Fake::new();
        let mut d = dps(fake);
        set(&mut d, &Key::VoltageTarget, -0.0).unwrap();
        assert_eq!(settings(&seen), [write_frame(0x00, 0)]);
    }

    #[test]
    fn a_range_is_written_and_changes_the_scale() {
        let (fake, seen, _) = Fake::new();
        let mut d = rd6012p(fake);
        set(&mut d, &Key::Range, "12A").unwrap();
        assert_eq!(settings(&seen), [write_frame(20, 1)]);
        set(&mut d, &Key::CurrentLimit, 1.0).unwrap();
        assert_eq!(settings(&seen)[1], write_frame(9, 1000), "in the new scale");
        assert!(matches!(set(&mut d, &Key::Range, "1A"), Err(Error::NotSupported(_))));
    }

    #[test]
    fn a_model_with_one_range_has_no_range_key() {
        let (fake, seen, _) = Fake::new();
        let mut d = dps(fake);
        let before = writes(&seen).len();
        assert!(d.options(None).unwrap().iter().all(|o| o.key != Key::Range));
        assert!(d.get(&Key::Range, None).is_err());
        assert!(d.list(&Key::Range, None).is_err());
        assert!(set(&mut d, &Key::Range, "5A").is_err());
        assert_eq!(writes(&seen).len(), before);
        let (fake, _, _) = Fake::new();
        let d = rd6012p(fake);
        assert_eq!(d.options(None).unwrap().last().unwrap().key, Key::Range);
    }

    #[test]
    fn a_get_reads_both_blocks_for_any_value() {
        let (fake, seen, registers) = Fake::new();
        {
            let mut r = registers.lock().unwrap();
            r[0] = 500;
            r[0x52] = 4000;
        }
        let d = dps(fake);
        assert_eq!(d.get(&Key::VoltageTarget, None).unwrap(), Value::F64(5.0));
        assert_eq!(d.get(&Key::OverVoltageProtectionThreshold, None).unwrap(), Value::F64(40.0));
        let w = writes(&seen);
        assert_eq!(w.len(), 5, "the lock, and two blocks twice: {w:02x?}");
        assert_eq!(w[1], frame(1, &[3, 0, 0, 0, 10]));
        assert_eq!(w[2], frame(1, &[3, 0, 0x52, 0, 2]));
    }

    #[test]
    fn a_get_gives_each_key() {
        let (fake, _, registers) = Fake::new();
        {
            let mut r = registers.lock().unwrap();
            r[8] = 12000;
            r[9] = 20000;
            r[10] = 11000;
            r[11] = 15000;
            r[16] = 2;
            r[17] = 1;
            r[18] = 1;
            r[20] = 1;
            r[82] = 62000;
            r[83] = 6200;
        }
        let d = rd6012p(fake);
        assert_eq!(d.get(&Key::VoltageTarget, None).unwrap(), Value::F64(12.0));
        assert_eq!(d.get(&Key::Voltage, None).unwrap(), Value::F64(11.0));
        assert_eq!(d.get(&Key::CurrentLimit, None).unwrap(), Value::F64(2.0));
        assert_eq!(d.get(&Key::Current, None).unwrap(), Value::F64(1.5));
        assert_eq!(d.get(&Key::Enabled, None).unwrap(), Value::Bool(true));
        assert_eq!(d.get(&Key::Regulation, None).unwrap(), Value::from("CC"));
        assert_eq!(d.get(&Key::OverVoltageProtectionActive, None).unwrap(), Value::Bool(false));
        assert_eq!(d.get(&Key::OverCurrentProtectionActive, None).unwrap(), Value::Bool(true));
        assert_eq!(d.get(&Key::OverVoltageProtectionThreshold, None).unwrap(), Value::F64(62.0));
        assert_eq!(d.get(&Key::Range, None).unwrap(), Value::from("12A"));
    }

    #[test]
    fn a_list_has_the_ranges_step() {
        let (fake, _, _) = Fake::new();
        let d = dps(fake);
        assert_eq!(
            d.list(&Key::VoltageTarget, None).unwrap(),
            ValueList::RangeStepF64 { min: 0.0, max: 50.0, step: 0.01 }
        );
        assert_eq!(
            d.list(&Key::CurrentLimit, None).unwrap(),
            ValueList::RangeStepF64 { min: 0.0, max: 5.0, step: 0.001 }
        );
    }

    #[test]
    fn a_capture_counts_its_frames_and_ends_at_the_limit() {
        let (fake, seen, registers) = Fake::new();
        registers.lock().unwrap()[2] = 1200;
        let mut d = dps(fake);
        set(&mut d, &Key::LimitSamples, 2_u64).unwrap();
        let packets = capture(&mut d);
        let frames = packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count();
        assert_eq!(frames, 2);
        assert!(matches!(packets.first(), Some(Packet::Header { .. })));
        assert!(matches!(packets.last(), Some(Packet::End)));
        let analog = packets.iter().find_map(|p| match p {
            Packet::Analog(a) => Some(a),
            _ => None,
        });
        assert_eq!(analog.unwrap().samples, [12.0]);
        // The lock, the seed's two reads, and two for each frame.
        assert_eq!(writes(&seen).len(), 1 + 2 + 4);
    }

    #[test]
    fn the_polls_keep_their_distance() {
        let (fake, seen, _) = Fake::new();
        let mut d = dps(fake);
        set(&mut d, &Key::LimitSamples, 3_u64).unwrap();
        capture(&mut d);
        let seen = seen.lock().unwrap();
        // The lock and the seed's two reads, then two reads per poll: from the
        // second block of one to the next poll's first. The fake answers at
        // once, so that is all the wait.
        let at: Vec<Instant> = seen.writes.iter().map(|(t, _)| *t).collect();
        for pair in at[3..].chunks(2).collect::<Vec<_>>().windows(2) {
            let gap = pair[1][0].duration_since(*pair[0].last().unwrap());
            assert!(gap >= Duration::from_millis(10), "{gap:?}");
        }
    }

    #[test]
    fn what_changes_comes_after_its_frame_in_c_order() {
        let (mut fake, _, _) = Fake::new();
        fake.protect_at = 7;
        // The seed's read takes the first; the first frame no change.
        fake.protect = [0, 0, 1, 2].into();
        let mut d = dps(fake);
        set(&mut d, &Key::LimitSamples, 3_u64).unwrap();
        let packets = capture(&mut d);
        let shown: Vec<String> = packets
            .iter()
            .filter_map(|p| match p {
                Packet::FrameBegin => Some("frame".to_owned()),
                Packet::Meta(m) => Some(format!("{:?}={:?}", m[0].0, m[0].1)),
                _ => None,
            })
            .collect();
        assert_eq!(shown.len(), 3 + 3, "{shown:?}");
        assert_eq!(shown[0], "frame");
        assert_eq!(shown[1], "frame");
        assert!(shown[2].contains("OverVoltage"), "{shown:?}");
        assert_eq!(shown[3], "frame");
        assert!(shown[4].contains("OverVoltage"), "{shown:?}");
        assert!(shown[5].contains("OverCurrent"), "{shown:?}");
    }

    #[test]
    fn a_change_of_range_shows_in_the_next_frame() {
        let (mut fake, _, registers) = Fake::new();
        // The range in register 20 and in its place in the block both.
        registers.lock().unwrap()[9] = 10000;
        fake.protect_at = 20;
        fake.protect = [0, 0, 1, 1].into();
        let mut d = rd6012p(fake);
        set(&mut d, &Key::LimitSamples, 3_u64).unwrap();
        let packets = capture(&mut d);
        let meta = packets.iter().find_map(|p| match p {
            Packet::Meta(m) => Some(m[0].clone()),
            _ => None,
        });
        assert_eq!(meta, Some((Key::Range, Value::from("12A"))));
        let digits: Vec<i32> = packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) if a.channels == [CHANNEL_I] => Some(a.digits),
                _ => None,
            })
            .collect();
        assert_eq!(digits, [4, 4, 3], "the frame that says so is still in the old scale");
    }

    #[test]
    fn a_failed_start_says_so_and_sets_nothing_going() {
        let (mut fake, seen, _) = Fake::new();
        fake.answers = Some(1);
        let mut d = dps(fake);
        let (tx, rx) = sync_channel(16);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
        assert!(rx.try_recv().is_err(), "no Header");
        assert!(!d.link().acquiring);
        assert_eq!(writes(&seen).len(), 1 + 3, "the lock, and one read's three tries");
    }

    #[test]
    fn reads_that_keep_failing_end_a_capture_with_no_limit() {
        let (mut fake, seen, _) = Fake::new();
        // The lock and the seed's two reads, then silence.
        fake.answers = Some(3);
        let mut d = dps(fake);
        let packets = capture(&mut d);
        assert!(!packets.iter().any(|p| matches!(p, Packet::FrameBegin)));
        assert!(matches!(packets.last(), Some(Packet::End)));
        let tries = 3 + 3 * usize::try_from(MAX_FAILED_POLLS).unwrap();
        assert_eq!(writes(&seen).len(), tries, "no more than the bound");
    }

    #[test]
    fn a_stop_ends_a_capture_and_the_link_is_free_for_a_setting() {
        let (fake, seen, _) = Fake::new();
        let mut d = dps(fake);
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Frames come until told otherwise.
        let deadline = Instant::now() + CAPTURE_BOUND;
        while !matches!(rx.recv_timeout(CAPTURE_BOUND).unwrap().1, Packet::FrameEnd) {
            assert!(Instant::now() < deadline);
        }
        set(&mut d, &Key::Enabled, true).unwrap();
        assert_eq!(settings(&seen), [write_frame(0x09, 1)], "between two reads");
        d.stop().unwrap();
        packets_until_end(&rx);
        assert!(!d.link().acquiring);
        d.close().unwrap();
        assert_eq!(writes(&seen).last().unwrap(), &write_frame(0x06, 0));
    }

    #[test]
    fn a_second_capture_runs_after_the_first() {
        let (fake, _, _) = Fake::new();
        let mut d = dps(fake);
        set(&mut d, &Key::LimitSamples, 1_u64).unwrap();
        capture(&mut d);
        let packets = capture(&mut d);
        assert_eq!(packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count(), 1);
    }

    #[test]
    fn a_time_limit_ends_a_capture_of_a_silent_supply_too() {
        let (mut fake, _, _) = Fake::new();
        fake.answers = Some(3);
        let mut d = dps(fake);
        set(&mut d, &Key::LimitMsec, 1_u64).unwrap();
        let started = Instant::now();
        capture(&mut d);
        assert!(started.elapsed() < Duration::from_secs(3), "one poll's three tries");
    }

    #[test]
    fn a_probe_reads_the_model_and_an_rd6012ps_range() {
        let (fake, seen, registers) = Fake::new();
        {
            let mut r = registers.lock().unwrap();
            r[0] = 60125;
            r[1] = 0;
            r[2] = 77;
            r[3] = 128;
            r[20] = 1;
        }
        let bus = Modbus::new(Box::new(fake), 1);
        let (link, identity) = probe(bus, Family::Rd).unwrap().unwrap();
        assert!(link.modbus.is_none(), "the port goes");
        assert_eq!(link.range, 1);
        assert_eq!((identity.id, identity.version, identity.serial), (60125, 128, 77));
        assert_eq!(
            writes(&seen),
            [frame(1, &[3, 0, 0, 0, 4]), frame(1, &[3, 0, 20, 0, 1])],
            "and nothing else"
        );
    }

    #[test]
    fn a_probe_of_one_range_asks_for_none() {
        let (fake, seen, registers) = Fake::new();
        {
            let mut r = registers.lock().unwrap();
            r[0x0b] = 5005;
            r[0x0c] = 116;
        }
        let bus = Modbus::new(Box::new(fake), 1);
        let (_, identity) = probe(bus, Family::Dps).unwrap().unwrap();
        assert_eq!(identity.version, 116);
        assert_eq!(writes(&seen).len(), 1);
    }

    #[test]
    fn an_unknown_model_is_no_device() {
        let (fake, _, _) = Fake::new();
        let bus = Modbus::new(Box::new(fake), 1);
        assert!(probe(bus, Family::Dps).unwrap().is_none());
    }

    #[test]
    fn the_driver_finds_nothing_without_conn() {
        assert!(RdtechDpsDriver.scan(&[]).unwrap().is_empty());
        assert!(RdtechRdDriver.scan(&[]).unwrap().is_empty());
    }

    #[test]
    fn the_identity_shows_as_c_prints_it() {
        let (fake, _, _) = Fake::new();
        let d = RdtechDevice::from_port(model(Family::Dps, 5205), Box::new(fake));
        assert_eq!(d.info().driver, "rdtech-dps");
        assert_eq!(d.info().model, "DPH5005");
        assert_eq!(d.info().vendor, "RDTech");
        let link = Link::new(None, model(Family::Rd, 60061));
        let identity = Identity { id: 60061, version: 128, serial: 42 };
        let params = Params::parse("115200/8n1").unwrap();
        let d = RdtechDevice::new(link, identity, "x".to_owned(), params, 1);
        assert_eq!(d.info().driver, "rdtech-rd");
        assert_eq!(d.info().version, "v1.28");
        assert_eq!(d.info().serial, "42");
    }
}
