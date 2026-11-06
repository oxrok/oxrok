// Counterpart in sigrok:
//   libsigrok/src/hardware/itech-it8500/api.c,
//   libsigrok/src/hardware/itech-it8500/protocol.c,
//   Copyright (C) 2020 Timo Kokkonen <tjko@iki.fi>,
//   with commits by Gerhard Sittig, Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! ITECH IT8500 series programmable DC electronic loads, and the BK Precision
//! 8500 series, which differ in looks alone, on their TTL serial port.
//!
//! `conn=` names the port. Without `serialcomm=` a scan tries 9600, 38400,
//! 19200 and 4800 baud, reopening the port for each, until a load answers a
//! broadcast. It then asks for the bar code, the limits and the state, sets
//! nothing, and closes the port. The line's speed sets the fastest sample rate.
//!
//! An open puts the load in remote mode and a close back in local mode, and a
//! failure of either is only noted. Everything a `--get` or `--show` reads, it
//! reads from the load at that moment, but for the limits and the sample rate.
//!
//! A capture polls the state once per period, the first after one period. A
//! sample is a frame with voltage, current and power, ahead of it a meta packet
//! for each of the output, the protections and the mode that has changed. A
//! poll that fails is no sample, and a capture goes on, as in C. Unlike C, a
//! time limit is checked after a failed poll as well, and a capture that only a
//! sample limit could end stops after [`NO_READING_BOUND`] with no reading.
//!
//! A status packet of success is no answer to a read, where C makes readings
//! of it: an inference, see [`protocol::parse_reply`]. The input queue is dropped
//! before each command. In C a byte too many makes every later answer fail: it
//! polls flat out with no sample, as the byte left unread fires `G_IO_IN`, and by
//! the count of bytes only 25 more such faults would bring it back. A value `int`
//! cannot hold is refused, where C's conversion is undefined. A negative one goes
//! out as in C: the range is the load's to check.
//!
//! An answer later than its 100 ms can pass for the next one's: see [`protocol`].
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::serial::{Params, Port};
use protocol::{Failed, Limits, Mode, Status, command};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::ElectronicLoad];
/// What a scan tries, the factory's first.
const SERIALCOMMS: [&str; 4] = ["9600/8n1", "38400/8n1", "19200/8n1", "4800/8n1"];
const GROUP: &str = "1";

static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
];
static GROUP_OPTIONS: [OptionDesc; 20] = [
    OptionDesc::new(Key::Enabled, Access::GET_SET),
    OptionDesc::new(Key::Regulation, Access::GET_SET_LIST),
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::VoltageTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::Power, Access::GET),
    OptionDesc::new(Key::PowerTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::ResistanceTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::OverVoltageProtectionEnabled, Access::GET),
    OptionDesc::new(Key::OverVoltageProtectionActive, Access::GET),
    OptionDesc::new(Key::OverVoltageProtectionThreshold, Access::GET_SET),
    OptionDesc::new(Key::OverCurrentProtectionEnabled, Access::GET),
    OptionDesc::new(Key::OverCurrentProtectionActive, Access::GET),
    OptionDesc::new(Key::OverCurrentProtectionThreshold, Access::GET_SET),
    OptionDesc::new(Key::UnderVoltageCondition, Access::GET),
    OptionDesc::new(Key::UnderVoltageConditionActive, Access::GET),
    OptionDesc::new(Key::UnderVoltageConditionThreshold, Access::GET_SET),
    OptionDesc::new(Key::OverTemperatureProtection, Access::GET),
    OptionDesc::new(Key::OverTemperatureProtectionActive, Access::GET),
];

/// How long a capture that only a sample limit could end goes without a
/// reading. C has no bound, so such a capture of a silent load never ends. A
/// time limit or a client's stop ends the rest.
pub const NO_READING_BOUND: Duration = Duration::from_secs(5);
/// How long a stopped capture waits to deliver its End to a queue that is full.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// The longest a wait goes without looking at the stop flag.
const LOOK: Duration = Duration::from_millis(5);

/// The `itech-it8500` driver.
pub struct ItechIt8500Driver;

impl Driver for ItechIt8500Driver {
    fn id(&self) -> &'static str {
        "itech-it8500"
    }

    fn long_name(&self) -> &'static str {
        "ITECH IT8500 series"
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
                Key::Serialcomm => {
                    serialcomm =
                        Some(value.as_str().ok_or_else(|| Error::arg("serialcomm"))?.to_owned());
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        log::info!("Probing serial port: {conn}");
        let list: Vec<&str> = match &serialcomm {
            Some(one) => vec![one.as_str()],
            None => SERIALCOMMS.to_vec(),
        };
        for text in list {
            // A port that does not open ends the scan, as in `scan`.
            let opened = Params::parse(text)
                .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
            let (port, params) = match opened {
                Ok(opened) => opened,
                Err(e) => {
                    log::error!("{e}");
                    return Ok(Vec::new());
                }
            };
            match Probe::new(port, params).run() {
                Probed::Found(device) => {
                    return Ok(vec![Box::new(ItechIt8500Device::new(*device, conn))]);
                }
                Probed::NoAnswer => {}
                Probed::Failed => return Ok(Vec::new()),
            }
        }
        Ok(Vec::new())
    }
}

/// What a scan learnt, the port closed.
struct Found {
    link: Link,
    info: protocol::ModelInfo,
}

enum Probed {
    Found(Box<Found>),
    /// No answer at this line's speed: the next one.
    NoAnswer,
    /// A load, or something like one, that would not do.
    Failed,
}

struct Probe {
    port: Box<dyn Port>,
    params: Params,
}

impl Probe {
    fn new(port: Box<dyn Port>, params: Params) -> Probe {
        Probe { port, params }
    }

    /// `scan` after the open. The port goes when this returns, as `scan`'s
    /// `serial_close`; `dev_open` opens it again.
    fn run(self) -> Probed {
        let Probe { port, params } = self;
        let mut link = Link::new(params, protocol::BROADCAST);
        link.port = Some(port);
        let Ok(reply) = link.exchange(command::GET_MODEL_INFO, &[]) else {
            return Probed::NoAnswer;
        };
        link.address = reply.address;
        let info = protocol::parse_model_info(&reply.data);
        log::info!("Model name: {} (v{})", info.model, info.version);
        log::info!("Address: {}", link.address);
        log::info!("Serial number: {}", info.serial);
        if let Ok(reply) = link.exchange(command::GET_BARCODE_INFO, &[]) {
            log::info!("Barcode: {}", protocol::parse_barcode(&reply.data));
        }
        let Ok(reply) = link.exchange(command::GET_LOAD_LIMITS, &[]) else {
            return Probed::Failed;
        };
        let limits = protocol::parse_limits(&reply.data);
        log::info!("Max current: {:.0} A", limits.max_current);
        log::info!("Max power: {:.0} W", limits.max_power);
        log::info!("Voltage range: {:.1} - {:.1} V", limits.min_voltage, limits.max_voltage);
        log::info!(
            "Resistance range: {:.2} - {:.2} Ohm",
            limits.min_resistance,
            limits.max_resistance
        );
        link.limits = limits;
        let Ok(status) = link.refresh() else {
            log::error!("Failed to get unit status: -1");
            return Probed::Failed;
        };
        log::info!("Mode: {}", status.mode().name());
        log::info!("State: {}", if status.load_on() { "ON" } else { "OFF" });
        log::info!("Default sample rate: {} Hz", protocol::DEFAULT_SAMPLERATE);
        log::info!("Maximum sample rate: {} Hz", protocol::max_samplerate(params.baud));
        link.port = None;
        Probed::Found(Box::new(Found { link, info }))
    }
}

/// The port and what has been read through it. A client's get or set and a
/// capture's poll take turns on it.
struct Link {
    port: Option<Box<dyn Port>>,
    params: Params,
    address: u8,
    limits: Limits,
    status: Status,
    /// `sr_sw_limits`, which a capture reads at each check, as C's does: a client
    /// can change them under it.
    limit_samples: u64,
    limit_msec: u64,
    /// A capture has been started and has not said it is over.
    capturing: bool,
    /// The last command timed out, and its answer may be on its way.
    late: bool,
}

impl Link {
    fn new(params: Params, address: u8) -> Link {
        Link {
            port: None,
            params,
            address,
            limits: Limits::default(),
            status: Status::default(),
            limit_samples: 0,
            limit_msec: 0,
            capturing: false,
            late: false,
        }
    }

    fn exchange(
        &mut self,
        command: u8,
        data: &[u8],
    ) -> std::result::Result<protocol::Reply, Failed> {
        let port =
            self.port.as_mut().ok_or_else(|| Failed::Port(Error::device("device is not open")))?;
        if std::mem::take(&mut self.late) {
            protocol::settle(port.as_mut())?;
        }
        let result = protocol::exchange(port.as_mut(), &self.params, self.address, command, data);
        self.late = matches!(result, Err(Failed::Late));
        result
    }

    /// `itech_it8500_get_status`, without its meta packets.
    fn refresh(&mut self) -> std::result::Result<Status, Failed> {
        let reply = self.exchange(command::GET_STATE, &[])?;
        self.status = Status::parse(&reply.data);
        Ok(self.status)
    }

    /// A number from a command that reads one.
    fn read_int(&mut self, command: u8) -> Result<i32> {
        Ok(protocol::parse_int(&self.exchange(command, &[])?.data))
    }

    /// The remote mode's change, whose failure is only a note.
    fn remote(&mut self, on: bool, failure: &str) {
        if self.exchange(command::SET_REMOTE_MODE, &[u8::from(on)]).is_err() {
            log::debug!("{failure}");
        }
    }
}

/// One load.
pub struct ItechIt8500Device {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    conn: String,
    samplerate: u64,
    /// [`NO_READING_BOUND`], but for tests.
    no_reading_bound: Duration,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl ItechIt8500Device {
    fn new(found: Found, conn: String) -> Self {
        let Found { link, info } = found;
        let names = ["V1", "I1", "P1"];
        let channels = names
            .iter()
            .enumerate()
            .map(|(i, name)| Channel::new(i, ChannelKind::Analog, (*name).to_owned()))
            .collect();
        ItechIt8500Device {
            info: DeviceInfo {
                driver: "itech-it8500",
                vendor: "ITECH".to_owned(),
                model: info.model,
                version: info.version,
                serial: info.serial,
                conn: None,
            },
            channels,
            groups: vec![ChannelGroup { name: GROUP.to_owned(), channels: vec![0, 1, 2] }],
            conn,
            samplerate: protocol::DEFAULT_SAMPLERATE,
            no_reading_bound: NO_READING_BOUND,
            pending: None,
            link: Arc::new(Mutex::new(link)),
            worker: None,
        }
    }

    /// A load on a port that stands in for the hardware, at address 0, of the
    /// limits given, with no scan. [`Device::open`] takes it.
    pub fn from_port(limits: Limits, port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMMS[0]).expect("the default parses");
        let conn = port.name().to_owned();
        let mut link = Link::new(params, 0);
        link.limits = limits;
        let info = protocol::ModelInfo {
            model: "8500".to_owned(),
            version: "1.23".to_owned(),
            serial: String::new(),
        };
        let mut device = Self::new(Found { link, info }, conn);
        device.pending = Some(port);
        device
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    /// A status read now, for a client.
    fn status(&self) -> Result<Status> {
        Ok(self.link().refresh()?)
    }

    fn read_scaled(&self, command: u8, unit: f64) -> Result<Value> {
        let n = self.link().read_int(command)?;
        Ok(Value::F64(f64::from(n) / unit))
    }

    /// A number in a setting's command: C's `ivalue = value * unit`, the value
    /// in `sigrok-cli`'s single precision.
    fn set_scaled(&self, key: &Key, command: u8, value: &Value, unit: f64) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let value = value.as_f64().ok_or_else(bad)?;
        let number = protocol::scaled(f64::from(value as f32), unit).ok_or_else(bad)?;
        self.link().exchange(command, &number.to_le_bytes())?;
        Ok(())
    }

    fn set_byte(&self, command: u8, value: u8) -> Result<()> {
        self.link().exchange(command, &[value])?;
        Ok(())
    }
}

impl Device for ItechIt8500Device {
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
            Some(g) if g.eq_ignore_ascii_case(GROUP) => Ok(GROUP_OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    /// As `config_get`, which does not look at the group.
    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        Ok(match key {
            Key::LimitSamples => Value::U64(self.link().limit_samples),
            Key::LimitMsec => Value::U64(self.link().limit_msec),
            Key::Samplerate => Value::U64(self.samplerate),
            Key::Enabled => Value::Bool(self.status()?.load_on()),
            Key::Regulation => Value::Str(self.status()?.mode().name().to_owned()),
            Key::Voltage => Value::F64(self.status()?.voltage),
            Key::VoltageTarget => self.read_scaled(command::GET_CV_VOLTAGE, 1000.0)?,
            Key::Current => Value::F64(self.status()?.current),
            Key::CurrentLimit => self.read_scaled(command::GET_CC_CURRENT, 10000.0)?,
            Key::Power => Value::F64(self.status()?.power),
            Key::PowerTarget => self.read_scaled(command::GET_CW_POWER, 1000.0)?,
            Key::ResistanceTarget => self.read_scaled(command::GET_CR_RESISTANCE, 1000.0)?,
            Key::OverVoltageProtectionEnabled
            | Key::OverCurrentProtectionEnabled
            | Key::OverTemperatureProtection => Value::Bool(true),
            Key::OverVoltageProtectionActive => {
                Value::Bool(self.status()?.demand & protocol::DS_OV != 0)
            }
            Key::OverVoltageProtectionThreshold => {
                self.read_scaled(command::GET_MAX_VOLTAGE, 1000.0)?
            }
            Key::OverCurrentProtectionActive => {
                Value::Bool(self.status()?.demand & protocol::DS_OC != 0)
            }
            Key::OverCurrentProtectionThreshold => {
                self.read_scaled(command::GET_MAX_CURRENT, 10000.0)?
            }
            Key::OverTemperatureProtectionActive => {
                Value::Bool(self.status()?.demand & protocol::DS_OT != 0)
            }
            // The hardware reports no under voltage, by `api.c`'s comment.
            Key::UnderVoltageCondition | Key::UnderVoltageConditionActive => Value::Bool(false),
            Key::UnderVoltageConditionThreshold => Value::F64(0.0),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            // `core::config_set` refuses a 0 and says so; this is for a library
            // caller that comes straight here.
            Key::LimitSamples => {
                self.link().limit_samples = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::LimitMsec => {
                self.link().limit_msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::Samplerate => {
                let rate = value.as_u64().ok_or_else(bad)?;
                let fastest = protocol::max_samplerate(self.link().params.baud);
                if !(1..=fastest).contains(&rate) {
                    return Err(Error::Arg("invalid samplerate".to_owned()));
                }
                self.samplerate = rate;
            }
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(bad)?;
                self.set_byte(command::LOAD_ON_OFF, u8::from(on))?;
            }
            Key::Regulation => {
                let mode = value.as_str().and_then(Mode::from_text).ok_or_else(bad)?;
                self.set_byte(command::SET_MODE, mode as u8)?;
            }
            Key::VoltageTarget => {
                self.set_scaled(key, command::SET_CV_VOLTAGE, value, 1000.0)?;
            }
            Key::CurrentLimit => {
                self.set_scaled(key, command::SET_CC_CURRENT, value, 10000.0)?;
            }
            Key::PowerTarget => self.set_scaled(key, command::SET_CW_POWER, value, 1000.0)?,
            Key::ResistanceTarget => {
                self.set_scaled(key, command::SET_CR_RESISTANCE, value, 1000.0)?;
            }
            Key::OverVoltageProtectionThreshold => {
                self.set_scaled(key, command::SET_MAX_VOLTAGE, value, 1000.0)?;
            }
            Key::OverCurrentProtectionThreshold => {
                self.set_scaled(key, command::SET_MAX_CURRENT, value, 10000.0)?;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    /// As `config_list`, whose rates are listed for a group that lacks the key
    /// and refused for the device that has it, so no client gets them.
    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if group.is_none() {
            return Err(Error::UnknownKey(key.id().to_owned()));
        }
        let limits = self.link().limits;
        Ok(match key {
            Key::Regulation => {
                ValueList::Str(Mode::ALL.iter().map(|m| m.name().to_owned()).collect())
            }
            Key::VoltageTarget => ValueList::RangeStepF64 {
                min: limits.min_voltage,
                max: limits.max_voltage,
                step: 0.01,
            },
            Key::CurrentLimit => {
                ValueList::RangeStepF64 { min: 0.0, max: limits.max_current, step: 0.001 }
            }
            Key::PowerTarget => {
                ValueList::RangeStepF64 { min: 0.0, max: limits.max_power, step: 0.01 }
            }
            Key::ResistanceTarget => ValueList::RangeStepF64 {
                min: limits.min_resistance,
                max: limits.max_resistance,
                step: 0.01,
            },
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// The port, and remote mode.
    fn open(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        if link.port.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&self.conn, link.params)?,
        };
        link.port = Some(port);
        link.remote(true, "Failed to set unit to remote mode");
        Ok(())
    }

    /// Local mode, and the port.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let mut link = self.link();
        link.capturing = false;
        if link.port.is_some() {
            link.remote(false, "Failed to set unit back to local mode: -1");
        }
        link.port = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        let reported = {
            let mut link = self.link();
            if link.capturing {
                return Err(Error::Busy);
            }
            if link.port.is_none() {
                return Err(Error::device("device is not open"));
            }
            link.capturing = true;
            link.status
        };
        // Over, by what it said ahead of its End, and maybe still sending that.
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            period: protocol::poll_period(self.samplerate),
            no_reading_bound: self.no_reading_bound,
            reported,
        };
        let link = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let spawned = std::thread::Builder::new()
            .name("itech-it8500 acquisition".to_owned())
            .spawn(move || {
                let outcome = capture.run(&sink, &stop2);
                // Ahead of the End, so that a client that sees it can start again.
                link.lock().expect("link lock").capturing = false;
                send_end(&sink, &stop2, outcome);
            });
        match spawned {
            Ok(handle) => {
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                self.link().capturing = false;
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

/// An error path of a client ends here too, and a load must not stay locked
/// in remote mode for it.
impl Drop for ItechIt8500Device {
    fn drop(&mut self) {
        if let Err(e) = self.close() {
            log::debug!("Could not close the device: {e}.");
        }
    }
}

/// One capture, on its own thread: `itech_it8500_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    period: Duration,
    no_reading_bound: Duration,
    /// The state the last meta packets go by.
    reported: Status,
}

impl Capture {
    fn run(mut self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let started = Instant::now();
        let mut samples = 0u64;
        let mut last_good = started;
        loop {
            if !wait(self.period, stop) {
                return Ok(());
            }
            // The lock is held for the poll alone: a client's setting gets in
            // between two, and a full queue holds up nobody.
            let (polled, limit_samples, limit_msec) = {
                let mut link = self.link.lock().expect("link lock");
                (link.refresh(), link.limit_samples, link.limit_msec)
            };
            match polled {
                Ok(status) => {
                    last_good = Instant::now();
                    let packets = sample_packets(&self.reported, &status);
                    self.reported = status;
                    if !packets.into_iter().all(|p| sink.send_while(p, go_on)) {
                        return Ok(());
                    }
                    samples += 1;
                }
                Err(Failed::Port(e)) => {
                    log::error!("Acquisition failed: {e}.");
                    return Err(e);
                }
                Err(Failed::Answer | Failed::Late) => {
                    let only_samples_can_end = limit_samples > 0 && limit_msec == 0;
                    if only_samples_can_end && last_good.elapsed() >= self.no_reading_bound {
                        log::error!("No reading from the load for {:?}.", self.no_reading_bound);
                        return Err(Error::device(format!(
                            "no reading from the load for {:?}",
                            self.no_reading_bound
                        )));
                    }
                }
            }
            if limit_samples > 0 && samples >= limit_samples {
                return Ok(());
            }
            if limit_msec > 0 && started.elapsed() >= Duration::from_millis(limit_msec) {
                return Ok(());
            }
        }
    }
}

/// `itech_it8500_status_change`: a meta packet for what changed.
fn meta_packets(old: &Status, new: &Status) -> Vec<Packet> {
    let mut out = Vec::new();
    if old.load_on() != new.load_on() {
        out.push(Packet::meta(Key::Enabled, Value::Bool(new.load_on())));
    }
    for (bit, key) in [
        (protocol::DS_OV, Key::OverVoltageProtectionActive),
        (protocol::DS_OC, Key::OverCurrentProtectionActive),
        (protocol::DS_OT, Key::OverTemperatureProtectionActive),
    ] {
        if old.demand & bit != new.demand & bit {
            out.push(Packet::meta(key, Value::Bool(new.demand & bit != 0)));
        }
    }
    if old.mode() != new.mode() {
        out.push(Packet::meta(Key::Regulation, Value::Str(new.mode().name().to_owned())));
    }
    out
}

/// A poll's packets: what changed, and a frame of the three readings.
fn sample_packets(old: &Status, new: &Status) -> Vec<Packet> {
    let mut out = meta_packets(old, new);
    out.push(Packet::FrameBegin);
    out.push(analog(0, new.voltage, Mq::Voltage, MqFlags::DC, Unit::Volt));
    out.push(analog(1, new.current, Mq::Current, MqFlags::DC, Unit::Ampere));
    out.push(analog(2, new.power, Mq::Power, MqFlags::empty(), Unit::Watt));
    out.push(Packet::FrameEnd);
    out
}

fn analog(channel: usize, value: f64, mq: Mq, flags: MqFlags, unit: Unit) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![channel],
        samples: vec![value as f32],
        mq,
        flags,
        unit,
        digits: 5,
    })
}

/// The End, for which a client that goes on reading is waited for without a
/// bound, and after a stop only for [`END_AFTER_STOP`].
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
        std::thread::sleep(left.min(LOOK));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits before it fails: a stop or a limit that does not
    /// bite must fail, not hang the run.
    const CAPTURE_BOUND: Duration = Duration::from_secs(20);

    const LIMITS: Limits = Limits {
        max_current: 30.0,
        max_voltage: 120.0,
        min_voltage: 0.1,
        max_power: 300.0,
        max_resistance: 7500.0,
        min_resistance: 0.05,
    };

    /// What the fake saw.
    #[derive(Default)]
    struct Seen {
        writes: crate::serial::WriteLog,
    }

    /// A load that answers each packet from its tables, and notes each write.
    struct Fake {
        seen: Arc<Mutex<Seen>>,
        input: VecDeque<u8>,
        /// Whole answers to `5f`, in turn; then the last of them over again.
        states: VecDeque<Vec<u8>>,
        /// Commands that get no answer.
        silent: Vec<u8>,
        /// Commands answered with this status.
        status: Vec<(u8, u8)>,
        /// The address in answers.
        address: u8,
        /// The answer to this command comes late: past a wait of 10 ms or more,
        /// so past any that a read for a whole answer asks, and it is there for
        /// the next short look at the line or, if none, ahead of the next answer.
        late: Option<u8>,
        withheld: Vec<u8>,
        /// A read that waits for less than this finds no answer yet: the answer
        /// comes this late.
        answer_after: Option<Duration>,
        /// All silent after this many writes.
        answers: Option<usize>,
        fail_writes_after: Option<usize>,
    }

    fn state(voltage: u32, operation: u8, demand: u16) -> Vec<u8> {
        let mut data = Vec::new();
        data.extend_from_slice(&voltage.to_le_bytes());
        data.extend_from_slice(&1000u32.to_le_bytes());
        data.extend_from_slice(&2000u32.to_le_bytes());
        data.push(operation);
        data.extend_from_slice(&demand.to_le_bytes());
        protocol::packet(0, command::GET_STATE, &data).to_vec()
    }

    impl Fake {
        fn new() -> (Fake, Arc<Mutex<Seen>>) {
            let seen = Arc::new(Mutex::new(Seen::default()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                states: VecDeque::from([state(12_000, 0, 64)]),
                silent: Vec::new(),
                status: Vec::new(),
                address: 0,
                late: None,
                withheld: Vec::new(),
                answer_after: None,
                answers: None,
                fail_writes_after: None,
            };
            (fake, seen)
        }

        fn answer(&mut self, command: u8) -> Vec<u8> {
            if let Some((_, code)) = self.status.iter().find(|(c, _)| *c == command) {
                return protocol::packet(self.address, command::RESPONSE, &[*code]).to_vec();
            }
            match command {
                command::GET_STATE => {
                    if self.states.len() > 1 {
                        self.states.pop_front().expect("counted")
                    } else {
                        self.states.front().cloned().unwrap_or_default()
                    }
                }
                command::GET_MODEL_INFO => {
                    let data = *b"85112\x23\x01ABCDEFGHIJ";
                    protocol::packet(self.address, command, &data).to_vec()
                }
                command::GET_LOAD_LIMITS => {
                    let mut data = Vec::new();
                    for n in [300_000u32, 120_000, 100, 300_000, 7_500_000] {
                        data.extend_from_slice(&n.to_le_bytes());
                    }
                    data.extend_from_slice(&50u16.to_le_bytes());
                    protocol::packet(self.address, command, &data).to_vec()
                }
                command::GET_BARCODE_INFO => {
                    protocol::packet(self.address, command, b"BAR").to_vec()
                }
                command::GET_CV_VOLTAGE => {
                    protocol::packet(self.address, command, &[0xff, 0xff, 0xff, 0xff]).to_vec()
                }
                command::GET_CC_CURRENT
                | command::GET_CW_POWER
                | command::GET_CR_RESISTANCE
                | command::GET_MAX_VOLTAGE
                | command::GET_MAX_CURRENT => {
                    protocol::packet(self.address, command, &5000u32.to_le_bytes()).to_vec()
                }
                _ => protocol::packet(self.address, command::RESPONSE, &[0x80]).to_vec(),
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
            if self.input.is_empty() && !self.withheld.is_empty() {
                if timeout <= Duration::from_millis(10) {
                    self.input.extend(self.withheld.drain(..));
                } else {
                    // All of the wait: a read for an answer asks no more.
                    std::thread::sleep(timeout);
                    return Ok(0);
                }
            }
            if self.answer_after.is_some_and(|after| timeout < after) {
                std::thread::sleep(timeout);
                return Ok(0);
            }
            if self.input.is_empty() {
                std::thread::sleep(timeout.min(Duration::from_millis(5)));
                return Ok(0);
            }
            let n = self.input.len().min(buf.len());
            for b in &mut buf[..n] {
                *b = self.input.pop_front().expect("counted");
            }
            Ok(n)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let writes = {
                let mut seen = self.seen.lock().unwrap();
                if self.fail_writes_after.is_some_and(|n| seen.writes.len() >= n) {
                    return Err(Error::device("Broken pipe"));
                }
                seen.writes.push((Instant::now(), data.to_vec()));
                seen.writes.len()
            };
            if self.answers.is_some_and(|n| writes > n) || self.silent.contains(&data[2]) {
                return Ok(());
            }
            // What was on its way gets here first.
            self.input.extend(self.withheld.drain(..));
            let answer = self.answer(data[2]);
            if self.late == Some(data[2]) {
                self.withheld = answer;
                return Ok(());
            }
            self.input.extend(answer);
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.input.clear();
            Ok(())
        }
    }

    fn writes(seen: &Arc<Mutex<Seen>>) -> Vec<Vec<u8>> {
        seen.lock().unwrap().writes.iter().map(|(_, w)| w.clone()).collect()
    }

    /// Each write as its address, command and first bytes of data.
    fn commands(seen: &Arc<Mutex<Seen>>) -> Vec<u8> {
        writes(seen).iter().map(|w| w[2]).collect()
    }

    fn device(fake: Fake) -> ItechIt8500Device {
        let mut d = ItechIt8500Device::from_port(LIMITS, Box::new(fake));
        d.open().unwrap();
        d
    }

    /// The packets up to and with the End, within the bound.
    fn until_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
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

    fn capture(d: &mut ItechIt8500Device) -> Vec<Packet> {
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        until_end(&rx)
    }

    fn voltages(packets: &[Packet]) -> Vec<f32> {
        packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) if a.channels == [0] => Some(a.samples[0]),
                _ => None,
            })
            .collect()
    }

    fn polls(seen: &Arc<Mutex<Seen>>) -> usize {
        bytecount(&commands(seen), command::GET_STATE)
    }

    fn bytecount(list: &[u8], wanted: u8) -> usize {
        list.iter().fold(0, |n, b| n + usize::from(*b == wanted))
    }

    /// Waits, with a bound, until the load has seen `n` polls.
    fn wait_for_polls(seen: &Arc<Mutex<Seen>>, n: usize) {
        let deadline = Instant::now() + CAPTURE_BOUND;
        while polls(seen) < n {
            assert!(Instant::now() < deadline, "no {n} polls in time");
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    fn probe(fake: Fake) -> Probed {
        let params = Params::parse("9600/8n1").unwrap();
        Probe::new(Box::new(fake), params).run()
    }

    #[test]
    fn a_scan_asks_four_things_and_sets_nothing() {
        let (mut fake, seen) = Fake::new();
        fake.address = 7;
        let Probed::Found(found) = probe(fake) else { panic!("a load") };
        let w = writes(&seen);
        assert_eq!(commands(&seen), [0x6a, 0x6b, 0x01, 0x5f]);
        assert_eq!(w[0][1], 0xff, "broadcast");
        assert_eq!(w[1][1], 7, "and then the address it said");
        assert_eq!(w[3][1], 7);
        assert_eq!(found.info.model, "85112");
        assert_eq!(found.info.version, "1.23");
        assert_eq!(found.info.serial, "ABCDEFGHIJ");
        assert_eq!(found.link.limits, LIMITS);
        assert!(found.link.port.is_none(), "closed, as the C's scan");
    }

    #[test]
    fn a_scan_takes_no_barcode_and_nothing_less() {
        let (mut fake, seen) = Fake::new();
        fake.silent = vec![0x6b];
        assert!(matches!(probe(fake), Probed::Found(_)));
        assert_eq!(commands(&seen), [0x6a, 0x6b, 0x01, 0x5f]);

        let (mut fake, seen) = Fake::new();
        fake.silent = vec![0x01];
        assert!(matches!(probe(fake), Probed::Failed));
        assert_eq!(commands(&seen), [0x6a, 0x6b, 0x01]);

        let (mut fake, seen) = Fake::new();
        fake.status = vec![(0x5f, 0xc0)];
        assert!(matches!(probe(fake), Probed::Failed));
        assert_eq!(commands(&seen), [0x6a, 0x6b, 0x01, 0x5f]);

        let (mut fake, seen) = Fake::new();
        fake.silent = vec![0x6a];
        assert!(matches!(probe(fake), Probed::NoAnswer));
        assert_eq!(commands(&seen), [0x6a]);
    }

    #[test]
    fn a_success_with_no_data_is_no_load() {
        let (mut fake, _) = Fake::new();
        fake.status = vec![(0x6a, 0x80)];
        assert!(matches!(probe(fake), Probed::NoAnswer));
    }

    #[test]
    fn open_and_close_change_the_remote_mode() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.close().unwrap();
        let w = writes(&seen);
        assert_eq!(w.len(), 2);
        assert_eq!(w[0], protocol::packet(0, 0x20, &[1]));
        assert_eq!(w[1], protocol::packet(0, 0x20, &[0]));
        d.close().unwrap();
        assert_eq!(writes(&seen).len(), 2, "a second close has no port");
    }

    #[test]
    fn a_remote_mode_refused_is_only_noted() {
        let (mut fake, seen) = Fake::new();
        fake.status = vec![(0x20, 0xc0)];
        let mut d = device(fake);
        assert!(d.get(&Key::Voltage, Some("1")).is_ok());
        assert!(d.close().is_ok());
        assert_eq!(commands(&seen), [0x20, 0x5f, 0x20]);
    }

    #[test]
    fn gets_ask_their_own_commands() {
        let (fake, seen) = Fake::new();
        let d = device(fake);
        let f = |k: Key| d.get(&k, Some("1")).unwrap().as_f64().unwrap();
        assert_eq!(f(Key::VoltageTarget), -0.001, "a signed number");
        assert_eq!(f(Key::CurrentLimit), 0.5);
        assert_eq!(f(Key::PowerTarget), 5.0);
        assert_eq!(f(Key::ResistanceTarget), 5.0);
        assert_eq!(f(Key::OverVoltageProtectionThreshold), 5.0);
        assert_eq!(f(Key::OverCurrentProtectionThreshold), 0.5);
        assert_eq!(commands(&seen), [0x20, 0x2d, 0x2b, 0x2f, 0x31, 0x23, 0x25]);
    }

    #[test]
    fn what_a_get_answers_itself_sends_nothing() {
        let (fake, seen) = Fake::new();
        let d = device(fake);
        for key in [
            Key::LimitSamples,
            Key::LimitMsec,
            Key::Samplerate,
            Key::OverVoltageProtectionEnabled,
            Key::OverCurrentProtectionEnabled,
            Key::OverTemperatureProtection,
            Key::UnderVoltageCondition,
            Key::UnderVoltageConditionActive,
            Key::UnderVoltageConditionThreshold,
        ] {
            d.get(&key, None).unwrap();
        }
        d.list(&Key::VoltageTarget, Some("1")).unwrap();
        assert_eq!(commands(&seen), [0x20]);
    }

    #[test]
    fn a_status_get_reads_the_state_each_time() {
        let (mut fake, seen) = Fake::new();
        fake.states = VecDeque::from([state(1000, 8, 1 << 7 | 2 | 4 | 16)]);
        let d = device(fake);
        assert!(matches!(d.get(&Key::Enabled, None), Ok(Value::Bool(true))));
        assert!(matches!(d.get(&Key::OverVoltageProtectionActive, None), Ok(Value::Bool(true))));
        assert!(matches!(d.get(&Key::OverCurrentProtectionActive, None), Ok(Value::Bool(true))));
        assert!(matches!(
            d.get(&Key::OverTemperatureProtectionActive, None),
            Ok(Value::Bool(true))
        ));
        assert_eq!(d.get(&Key::Voltage, None).unwrap().as_f64(), Some(1.0));
        assert_eq!(d.get(&Key::Current, None).unwrap().as_f64(), Some(0.1));
        assert_eq!(d.get(&Key::Power, None).unwrap().as_f64(), Some(2.0));
        assert!(matches!(d.get(&Key::Regulation, None), Ok(Value::Str(s)) if s == "CV"));
        assert_eq!(polls(&seen), 8);
    }

    #[test]
    fn settings_are_in_their_commands_and_units() {
        for (key, value, wire) in [
            (Key::VoltageTarget, 5.0, (0x2c, 5000)),
            (Key::CurrentLimit, 1.5, (0x2a, 15000)),
            (Key::PowerTarget, 20.0, (0x2e, 20000)),
            (Key::ResistanceTarget, 4.0, (0x30, 4000)),
            (Key::OverVoltageProtectionThreshold, 50.0, (0x22, 50000)),
            (Key::OverCurrentProtectionThreshold, 10.0, (0x24, 100_000)),
            (Key::VoltageTarget, 3.3, (0x2c, 3299)),
            (Key::CurrentLimit, -1.0, (0x2a, -10000)),
        ] {
            let (fake, seen) = Fake::new();
            let mut d = device(fake);
            d.set(&key, &Value::F64(value), Some("1")).unwrap();
            let want: i32 = wire.1;
            assert_eq!(
                writes(&seen)[1],
                protocol::packet(0, wire.0, &want.to_le_bytes()),
                "{key:?}"
            );
        }
    }

    #[test]
    fn byte_settings() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::Enabled, &Value::Bool(true), None).unwrap();
        d.set(&Key::Enabled, &Value::Bool(false), None).unwrap();
        d.set(&Key::Regulation, &Value::Str("CR".to_owned()), None).unwrap();
        d.set(&Key::Regulation, &Value::Str("CWX".to_owned()), None).unwrap();
        let w = writes(&seen);
        assert_eq!(w[1], protocol::packet(0, 0x21, &[1]));
        assert_eq!(w[2], protocol::packet(0, 0x21, &[0]));
        assert_eq!(w[3], protocol::packet(0, 0x28, &[3]));
        assert_eq!(w[4], protocol::packet(0, 0x28, &[2]));
    }

    #[test]
    fn what_c_leaves_undefined_or_refuses_sends_nothing() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 1e10] {
            assert!(d.set(&Key::VoltageTarget, &Value::F64(bad), None).is_err(), "{bad}");
        }
        assert!(d.set(&Key::Regulation, &Value::Str("XX".to_owned()), None).is_err());
        assert!(d.set(&Key::UnderVoltageConditionThreshold, &Value::F64(1.0), None).is_err());
        assert_eq!(commands(&seen), [0x20]);
    }

    #[test]
    fn a_refused_setting_is_an_error() {
        let (mut fake, _) = Fake::new();
        fake.status = vec![(0x2c, 0xa0)];
        let mut d = device(fake);
        assert!(d.set(&Key::VoltageTarget, &Value::F64(5.0), None).is_err());
    }

    #[test]
    fn the_samplerate_is_held_to_the_line() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        for ok in [1, 7, 15] {
            d.set(&Key::Samplerate, &Value::U64(ok), None).unwrap();
            assert_eq!(d.get(&Key::Samplerate, None).unwrap().as_u64(), Some(ok));
        }
        for bad in [0, 16, 60] {
            assert!(d.set(&Key::Samplerate, &Value::U64(bad), None).is_err());
            assert_eq!(d.get(&Key::Samplerate, None).unwrap().as_u64(), Some(15));
        }
        assert_eq!(commands(&seen), [0x20]);
    }

    #[test]
    fn the_options_are_scoped_as_c() {
        let (fake, _) = Fake::new();
        let d = device(fake);
        assert_eq!(d.options(None).unwrap().len(), 4);
        assert_eq!(d.options(Some("1")).unwrap().len(), 20);
        assert!(d.options(Some("2")).is_err());
        assert!(d.list(&Key::Samplerate, None).is_err(), "api.c:561-563");
        assert!(d.list(&Key::Regulation, None).is_err());
        assert!(
            matches!(d.list(&Key::Regulation, Some("1")), Ok(ValueList::Str(l)) if l.len() == 4)
        );
    }

    #[test]
    fn a_capture_ends_at_its_samples_with_three_readings_a_frame() {
        let (mut fake, seen) = Fake::new();
        fake.states = VecDeque::from([state(1000, 0, 64), state(2000, 0, 64)]);
        let mut d = device(fake);
        d.link().limit_samples = 2;
        let packets = capture(&mut d);
        assert_eq!(voltages(&packets), [1.0, 2.0]);
        assert_eq!(polls(&seen), 2);
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert!(matches!(packets[1], Packet::FrameBegin));
        let Packet::Analog(i) = &packets[3] else { panic!("current") };
        assert_eq!((i.channels[0], i.samples[0], i.unit), (1, 0.1, Unit::Ampere));
        assert_eq!(i.flags, MqFlags::DC);
        assert_eq!(i.digits, 5);
        let Packet::Analog(p) = &packets[4] else { panic!("power") };
        assert_eq!((p.channels[0], p.samples[0], p.unit), (2, 2.0, Unit::Watt));
        assert_eq!(p.flags, MqFlags::empty());
        assert!(matches!(packets[5], Packet::FrameEnd));
        assert!(matches!(packets.last(), Some(Packet::End)));
    }

    /// The first meta packets of a capture over these states, the first being
    /// what was known before it.
    fn metas(states: &[Vec<u8>]) -> Vec<(Key, String)> {
        let (mut fake, _) = Fake::new();
        fake.states = states.iter().cloned().collect();
        let mut d = device(fake);
        d.link().refresh().unwrap();
        d.link().limit_samples = states.len() as u64 - 1;
        d.samplerate = 15;
        let mut out = Vec::new();
        for p in capture(&mut d) {
            if let Packet::Meta(items) = p {
                out.extend(items.into_iter().map(|(k, v)| (k, format!("{v:?}"))));
            }
        }
        out
    }

    #[test]
    fn a_change_of_state_goes_out_once() {
        assert_eq!(metas(&[state(0, 0, 64), state(0, 0, 64)]), Vec::<(Key, String)>::new());
        let got = metas(&[state(0, 0, 64), state(0, 8, 64), state(0, 8, 64), state(0, 0, 64)]);
        assert_eq!(got.len(), 2);
        assert_eq!(got[0].0, Key::Enabled);
        assert!(got[0].1.contains("true"));
        assert_eq!(got[1].0, Key::Enabled);
        assert!(got[1].1.contains("false"));
    }

    #[test]
    fn each_flag_has_its_own_meta() {
        for (demand, key) in [
            (2, Key::OverVoltageProtectionActive),
            (4, Key::OverCurrentProtectionActive),
            (16, Key::OverTemperatureProtectionActive),
        ] {
            let got = metas(&[state(0, 0, 64), state(0, 0, 64 | demand)]);
            assert_eq!(got.len(), 1, "{key:?}");
            assert_eq!(got[0].0, key);
        }
        let got = metas(&[state(0, 0, 64), state(0, 0, 128)]);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].0, Key::Regulation);
        assert!(got[0].1.contains("CV"));
        // The bits with no meta of their own.
        assert_eq!(
            metas(&[state(0, 0, 64), state(0, 0x77, 64 | 1 | 8 | 32)]),
            Vec::<(Key, String)>::new()
        );
    }

    #[test]
    fn the_baseline_of_the_first_meta_is_the_last_known_state() {
        let (mut fake, _) = Fake::new();
        fake.states = VecDeque::from([state(0, 8, 64)]);
        let mut d = device(fake);
        d.get(&Key::Enabled, None).unwrap();
        d.link().limit_samples = 1;
        let packets = capture(&mut d);
        assert!(!packets.iter().any(|p| matches!(p, Packet::Meta(_))));
    }

    #[test]
    fn polls_keep_their_period_and_the_first_waits_for_one() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.link().limit_samples = 3;
        d.set(&Key::Samplerate, &Value::U64(5), None).unwrap();
        let started = Instant::now();
        capture(&mut d);
        let seen = seen.lock().unwrap();
        let times: Vec<Instant> =
            seen.writes.iter().filter(|(_, w)| w[2] == 0x5f).map(|(t, _)| *t).collect();
        assert_eq!(times.len(), 3);
        assert!(times[0] - started >= Duration::from_millis(200));
        let least = times.windows(2).map(|w| w[1] - w[0]).min().unwrap();
        assert!(least >= Duration::from_millis(200), "{least:?}");
    }

    #[test]
    fn a_failed_poll_is_no_sample_and_the_next_counts() {
        let bad = |mut p: Vec<u8>| {
            p[25] ^= 1;
            p
        };
        let (mut fake, seen) = Fake::new();
        fake.states = VecDeque::from([
            bad(state(9000, 0, 64)),
            state(1000, 0, 64),
            vec![0xaa, 0],
            protocol::packet(0, command::RESPONSE, &[0xa0]).to_vec(),
            protocol::packet(0, command::RESPONSE, &[0x80]).to_vec(),
            protocol::packet(0, command::GET_CC_CURRENT, &[]).to_vec(),
            state(2000, 0, 64),
        ]);
        let mut d = device(fake);
        d.link().limit_samples = 2;
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        assert_eq!(voltages(&capture(&mut d)), [1.0, 2.0]);
        assert_eq!(polls(&seen), 7);
    }

    #[test]
    fn a_long_answer_shifts_nothing() {
        let mut long = state(1000, 0, 64);
        long.push(0xff);
        let (mut fake, _) = Fake::new();
        fake.states = VecDeque::from([long, state(2000, 0, 64), state(3000, 0, 64)]);
        let mut d = device(fake);
        d.link().limit_samples = 3;
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        assert_eq!(voltages(&capture(&mut d)), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn silence_ends_a_capture_only_a_sample_limit_could_end() {
        let (mut fake, _) = Fake::new();
        fake.answers = Some(1);
        let mut d = device(fake);
        d.link().limit_samples = 2;
        d.no_reading_bound = Duration::from_millis(400);
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        let started = Instant::now();
        let packets = capture(&mut d);
        assert_eq!(voltages(&packets), Vec::<f32>::new());
        assert!(started.elapsed() >= Duration::from_millis(400));
        assert!(started.elapsed() < CAPTURE_BOUND);
        assert_eq!(NO_READING_BOUND, Duration::from_secs(5));
    }

    #[test]
    fn a_good_reading_starts_the_bound_over() {
        let (mut fake, _) = Fake::new();
        // Remote mode and three polls answered, then silence.
        fake.answers = Some(4);
        let mut d = device(fake);
        d.link().limit_samples = 5;
        d.no_reading_bound = Duration::from_millis(600);
        d.set(&Key::Samplerate, &Value::U64(5), None).unwrap();
        let started = Instant::now();
        let packets = capture(&mut d);
        assert_eq!(voltages(&packets).len(), 3);
        assert!(started.elapsed() >= Duration::from_millis(600 + 600));
    }

    #[test]
    fn a_time_limit_ends_a_silent_capture_ahead_of_the_bound() {
        let (mut fake, seen) = Fake::new();
        fake.answers = Some(1);
        let mut d = device(fake);
        d.link().limit_msec = 300;
        d.link().limit_samples = 2;
        d.no_reading_bound = Duration::from_secs(60);
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        let started = Instant::now();
        let packets = capture(&mut d);
        assert_eq!(voltages(&packets), Vec::<f32>::new());
        assert!(started.elapsed() >= Duration::from_millis(300));
        assert!(started.elapsed() < Duration::from_secs(30), "the bound did not end this");
        assert!(polls(&seen) >= 1);
    }

    #[test]
    fn a_time_limit_ends_a_capture_that_answers() {
        let (fake, _) = Fake::new();
        let mut d = device(fake);
        d.link().limit_msec = 250;
        d.set(&Key::Samplerate, &Value::U64(1), None).unwrap();
        let n = voltages(&capture(&mut d)).len();
        assert_eq!(n, 1, "the first poll comes after 1 s, and the limit is over by then");
    }

    #[test]
    fn with_no_limit_a_silent_capture_goes_on_until_a_stop() {
        let (mut fake, seen) = Fake::new();
        fake.answers = Some(1);
        let mut d = device(fake);
        d.no_reading_bound = Duration::from_millis(1);
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for_polls(&seen, 3);
        assert!(!d.worker.as_ref().unwrap().handle.is_finished());
        d.stop().unwrap();
        let packets = until_end(&rx);
        assert_eq!(voltages(&packets), Vec::<f32>::new());
    }

    #[test]
    fn a_stop_ends_a_capture_that_answers() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for_polls(&seen, 3);
        d.stop().unwrap();
        let packets = until_end(&rx);
        assert!(voltages(&packets).len() >= 2);
        let at_end = polls(&seen);
        std::thread::sleep(Duration::from_millis(100));
        assert_eq!(polls(&seen), at_end, "no poll after the End");
    }

    #[test]
    fn a_port_that_fails_ends_a_capture() {
        let (mut fake, seen) = Fake::new();
        fake.fail_writes_after = Some(3);
        let mut d = device(fake);
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(voltages(&packets).len(), 2);
        assert_eq!(polls(&seen), 2);
    }

    /// A close, on a queue no one reads, within the bound.
    fn close_returns(mut d: ItechIt8500Device) {
        let handle = std::thread::spawn(move || {
            d.close().unwrap();
            d
        });
        let deadline = Instant::now() + CAPTURE_BOUND;
        while !handle.is_finished() {
            assert!(Instant::now() < deadline, "the close hangs");
            std::thread::sleep(Duration::from_millis(5));
        }
        handle.join().unwrap();
    }

    #[test]
    fn a_close_ends_a_capture_blocked_on_its_header_and_its_end() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        // No one reads, and the queue is full before the header.
        let (tx, rx) = sync_channel(1);
        tx.send((0, Packet::FrameBegin)).unwrap();
        d.start(PacketSink::new(tx, 0)).unwrap();
        close_returns(d);
        drop(rx);
        assert_eq!(commands(&seen).last(), Some(&0x20), "the local mode went out");
    }

    #[test]
    fn a_close_ends_a_capture_blocked_on_its_samples() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        // The header fits and the first sample does not.
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for_polls(&seen, 1);
        close_returns(d);
        drop(rx);
        assert_eq!(commands(&seen).last(), Some(&0x20), "the local mode went out");
        assert_eq!(polls(&seen), 1, "blocked on the first sample until the stop");
    }

    #[test]
    fn an_answer_is_waited_for_100_ms() {
        for (after, in_time) in [(60, true), (150, false)] {
            let (mut fake, _) = Fake::new();
            fake.answer_after = Some(Duration::from_millis(after));
            let d = device(fake);
            assert_eq!(d.get(&Key::PowerTarget, None).is_ok(), in_time, "{after} ms");
        }
    }

    #[test]
    fn a_late_answer_does_not_pass_for_the_next() {
        let (mut fake, seen) = Fake::new();
        fake.late = Some(0x2a);
        fake.status = vec![(0x2c, 0xa0)];
        let mut d = device(fake);
        assert!(d.set(&Key::CurrentLimit, &Value::F64(1.0), None).is_err(), "no answer in time");
        assert!(
            d.set(&Key::VoltageTarget, &Value::F64(5.0), None).is_err(),
            "the first one's 80 is not the second one's"
        );
        assert_eq!(commands(&seen), [0x20, 0x2a, 0x2c]);
    }

    #[test]
    fn without_the_wait_a_late_answer_misleads_as_the_module_says() {
        let (mut fake, _) = Fake::new();
        fake.late = Some(0x2a);
        fake.status = vec![(0x2c, 0xa0)];
        let mut d = device(fake);
        assert!(d.set(&Key::CurrentLimit, &Value::F64(1.0), None).is_err());
        d.link().late = false;
        assert!(d.set(&Key::VoltageTarget, &Value::F64(5.0), None).is_ok());
    }

    #[test]
    fn only_a_timeout_waits_for_the_line() {
        let (mut fake, seen) = Fake::new();
        fake.status = vec![(0x2c, 0xa0)];
        let mut d = device(fake);
        assert!(d.set(&Key::VoltageTarget, &Value::F64(5.0), None).is_err());
        assert!(!d.link().late, "an answer came");
        let (mut fake2, _) = Fake::new();
        fake2.silent = vec![0x2c];
        let mut d2 = device(fake2);
        assert!(d2.set(&Key::VoltageTarget, &Value::F64(5.0), None).is_err());
        assert!(d2.link().late);
        drop(d2);
        assert_eq!(commands(&seen), [0x20, 0x2c]);
    }

    #[test]
    fn a_line_that_never_goes_quiet_ends_the_wait() {
        struct Noisy;
        impl Port for Noisy {
            fn name(&self) -> &'static str {
                "/dev/ttyNOISE"
            }
            fn set_params(&mut self, _: Params) -> Result<()> {
                Ok(())
            }
            fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
                Ok(())
            }
            fn read(&mut self, buf: &mut [u8], _: Duration) -> Result<usize> {
                std::thread::sleep(Duration::from_millis(1));
                buf[0] = 0x55;
                Ok(1)
            }
            fn write(&mut self, _: &[u8]) -> Result<()> {
                Ok(())
            }
            fn purge(&mut self) -> Result<()> {
                Ok(())
            }
        }
        let started = Instant::now();
        protocol::settle(&mut Noisy).map_err(Error::from).unwrap();
        assert!(started.elapsed() >= Duration::from_secs(1));
        assert!(started.elapsed() < CAPTURE_BOUND);
    }

    #[test]
    fn a_get_goes_through_while_a_capture_waits_on_a_full_queue() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        // The header fits and the first sample does not.
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for_polls(&seen, 1);
        let (answer, got) = sync_channel(1);
        std::thread::spawn(move || {
            // With a lock held over the send, this blocks for good.
            answer.send(d.get(&Key::Voltage, None).is_ok()).unwrap();
            d
        });
        assert_eq!(got.recv_timeout(CAPTURE_BOUND), Ok(true), "the get is held up");
        drop(rx);
    }

    #[test]
    fn a_time_limit_outlasts_a_shorter_bound() {
        let (mut fake, _) = Fake::new();
        fake.answers = Some(1);
        let mut d = device(fake);
        d.link().limit_samples = 2;
        d.link().limit_msec = 1200;
        d.no_reading_bound = Duration::from_millis(100);
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        let started = Instant::now();
        capture(&mut d);
        assert!(started.elapsed() >= Duration::from_millis(1200), "the bound ended it");
    }

    #[test]
    fn a_limit_set_during_a_capture_counts() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for_polls(&seen, 2);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        assert!(matches!(until_end(&rx).last(), Some(Packet::End)));
    }

    #[test]
    fn a_meta_comes_ahead_of_its_frame() {
        let old = Status { operation: 0, demand: 64, ..Status::default() };
        let new = Status { operation: 8, demand: 64, ..Status::default() };
        let packets = sample_packets(&old, &new);
        assert!(matches!(packets[0], Packet::Meta(_)));
        assert!(matches!(packets[1], Packet::FrameBegin));
        assert_eq!(packets.len(), 6);
    }

    #[test]
    fn a_start_is_no_busy_while_the_old_thread_waits_to_send_its_end() {
        let (fake, _) = Fake::new();
        let mut d = device(fake);
        d.link().limit_samples = 1;
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The frame's end stays in the queue, so the End cannot go.
        for _ in 0..5 {
            rx.recv_timeout(CAPTURE_BOUND).expect("a packet in time");
        }
        let deadline = Instant::now() + CAPTURE_BOUND;
        while d.link().capturing {
            assert!(Instant::now() < deadline, "the capture does not say it is over");
            std::thread::sleep(Duration::from_millis(1));
        }
        let (tx2, rx2) = sync_channel(4096);
        d.start(PacketSink::new(tx2, 0)).unwrap();
        assert_eq!(voltages(&until_end(&rx2)), [12.0]);
        // Told to stop and joined: its End was given up and its sink is gone.
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::FrameEnd))));
        assert_eq!(
            rx.recv_timeout(CAPTURE_BOUND).err(),
            Some(std::sync::mpsc::RecvTimeoutError::Disconnected)
        );
    }

    #[test]
    fn a_start_right_after_an_end_is_no_busy() {
        let (fake, _) = Fake::new();
        let mut d = device(fake);
        d.link().limit_samples = 1;
        // Faster than the line allows, for the many rounds.
        d.samplerate = 1000;
        for round in 0..300 {
            let (tx, rx) = sync_channel(64);
            d.start(PacketSink::new(tx, 0)).unwrap_or_else(|e| panic!("round {round}: {e}"));
            until_end(&rx);
        }
    }

    #[test]
    fn a_start_is_refused_with_the_port_closed() {
        let (fake, _) = Fake::new();
        let mut d = device(fake);
        d.close().unwrap();
        let (tx, _rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
    }

    #[test]
    fn a_second_start_while_running_is_busy() {
        let (fake, _) = Fake::new();
        let mut d = device(fake);
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx.clone(), 0)).unwrap();
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.stop().unwrap();
        until_end(&rx);
    }

    #[test]
    fn a_setting_gets_through_during_a_capture() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::Samplerate, &Value::U64(15), None).unwrap();
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for_polls(&seen, 1);
        d.set(&Key::Enabled, &Value::Bool(true), None).unwrap();
        d.stop().unwrap();
        until_end(&rx);
        assert!(commands(&seen).contains(&0x21));
    }
}
