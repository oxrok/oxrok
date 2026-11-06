// Counterpart in sigrok:
//   libsigrok/src/hardware/link-mso19/api.c,
//   libsigrok/src/hardware/link-mso19/protocol.c,
//   Copyright (C) 2011 Daniel Ribeiro <drwyrm@gmail.com>,
//   Copyright (C) 2012 Renato Caldas <rmsc@fe.up.pt>,
//   Copyright (C) 2013 Lior Elazary <lelazary@yahoo.com>,
//   Copyright (C) 2022 Paul Kasemir <paul.kasemir@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Martin Ling, Soeren Apel, Daniel Elstner, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Link Instruments MSO-19, a scope with 1 analog and 8 logic inputs and
//! 1024 samples of memory, on a USB serial port.
//!
//! A scan opens nothing: it goes by the port's USB ids, and takes the strings and
//! the calibration from the USB descriptors. An open resets the ADC, reads a
//! status and resets the sequencer. A capture writes every setting, arms, reads a
//! status, and polls one until the data is ready, every 10 ms or so; then it asks
//! for the dump and sends all 1024 samples, in two parts around the trigger when
//! the position is in the memory. A capture is one dump, and nothing else can end
//! it short of a stop.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - A status reading READY at the start is asked again, then dumped. C keeps
//!   it, so it never sends the dump request and hangs (`api.c:647`, `protocol.c:
//!   390`), which at a fast rate is any capture whose trigger comes at once.
//! - The trigger level is divided as a voltage, where C since 932a3452 divides
//!   whole volts as integers (`protocol.c:90-93`).
//! - A probe factor that a `uint16_t` does not hold is refused, where C wraps
//!   it (`api.c:530`) and at 65536 dies dividing by it (`protocol.c:93`). So is a
//!   level, an offset or a position that is no number.
//! - A device silent in a capture for [`SILENCE_BOUND`] ends it, where C waits
//!   for ever, as a trigger that never fires goes on being polled and is not
//!   silence. A stop leaves a reply on its way, so the next start or open waits
//!   for the line to go quiet ([`settle`]), for as long as a reply may take. A
//!   reply later than that can still land ahead of a dump on a device that also
//!   answers slower than a poll's read.
//! - C and this port read 19 digits of the serial number. A real unit has 11 more,
//!   which mso19fcgi uses at 200 MSa/s (`GetVbit` under 10 ns a sample), so 200 MHz
//!   readings here use the calibration of the other rates.
//! - A setting during a capture is refused as busy, where C takes it: a rate would
//!   go out among the polls, and what the dump is decoded with would change.
//! - The analog packets say they are from the channel DSO, as they do in C, whose
//!   channel has index 4, as logic channel 4 has: the index is C's.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Trigger, TriggerMatch,
    Unit, Value, ValueList,
};
use crate::serial::{self, Params, Port};
use protocol::{
    COUPLINGS, DSO_SLOPES, DUMP_BYTES, LA_SLOPES, LOGIC_THRESHOLDS, NUM_SAMPLES, SERIALCOMM,
    Settings, TRIGGER_SOURCES, TriggerSource,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 2] = [DeviceClass::Oscilloscope, DeviceClass::LogicAnalyzer];

static DEV_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::HorizTriggerpos, Access::GET_SET),
];
static DSO_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::Coupling, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSlope, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerLevel, Access::GET_SET),
    OptionDesc::new(Key::ProbeFactor, Access::GET_SET),
    OptionDesc::new(Key::Offset, Access::GET_SET),
];
static LA_OPTIONS: [OptionDesc; 2] = [
    OptionDesc::new(Key::TriggerSlope, Access::GET_SET_LIST),
    OptionDesc::new(Key::LogicThreshold, Access::GET_SET_LIST),
];

/// The channel group of the analog input.
const GROUP_DSO: &str = "DSO";
/// The channel group of the logic inputs.
const GROUP_LA: &str = "LA";
/// The analog channel's index. C takes what a loop left in its counter
/// (`api.c:302`), and it is logic channel 4's as well. Only its place in the
/// list tells them apart: DSO comes first, and a listing by index has it before 4.
const DSO_INDEX: usize = 4;

/// How long the reply of a status request at an open or start may take. C has
/// 10 ms, which fails on a loaded host where nothing is wrong: it costs only a
/// device that does not answer.
const STATUS_TIMEOUT: Duration = Duration::from_millis(100);
/// `serial_read_blocking`'s time in C's callback, which is what paces the polls:
/// it reads for 1024 bytes where a status is one, and so waits this long.
const POLL_READ: Duration = Duration::from_millis(10);
/// How long a capture goes with nothing from the device, where C waits for ever.
/// A dump takes 67 ms on the line and a status answers within ms, and anything
/// that arrives counts, so this is a device that has gone wrong.
pub const SILENCE_BOUND: Duration = Duration::from_secs(5);
/// How often a wait looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);
/// How long the line must have nothing on it after a stop to count as quiet: as
/// long as a status reply may take, or a reply a stop left out reads as the next
/// capture's status, one is outstanding from then on, and on a device slower than
/// [`POLL_READ`] it lands ahead of the dump and shifts it all.
const QUIET: Duration = STATUS_TIMEOUT;
/// How many reads of [`STOP_CHECK`] with nothing that has to be as well.
const QUIET_READS: u32 = 4;
/// How long [`settle`] goes on, for a device that never goes quiet. A dump takes
/// 67 ms on the line, and this leaves room for a slow host.
const SETTLE_BOUND: Duration = Duration::from_secs(2);
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// C's read in its callback, the most of a dump one pass takes.
const PASS_BYTES: usize = 1024;

/// The `link-mso19` driver.
pub struct LinkMso19Driver;

impl Driver for LinkMso19Driver {
    fn id(&self) -> &'static str {
        "link-mso19"
    }

    fn long_name(&self) -> &'static str {
        "Link Instruments MSO-19"
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
        let ports = serial::list_usb_ports();
        Ok(find_ports(ports, conn.as_deref())
            .into_iter()
            .filter_map(|port| device_of(&port).map(|d| Box::new(d) as Box<dyn Device>))
            .collect())
    }
}

/// The ports a scan looks at: all, or the one `conn` names, under a link or by
/// its own name.
fn find_ports(ports: Vec<serial::UsbPort>, conn: Option<&str>) -> Vec<serial::UsbPort> {
    let Some(conn) = conn else {
        return ports;
    };
    let real = std::fs::canonicalize(conn).ok();
    let callout = serial::callout_name(conn);
    ports
        .into_iter()
        .filter(|p| {
            p.name == conn
                || callout.as_deref() == Some(p.name.as_str())
                || real.as_deref().is_some_and(|r| r == std::path::Path::new(&p.name))
        })
        .collect()
}

/// `scan_handle_port`. C reads through the null a port with no serial number gives
/// it (`protocol.c:210`); here that is an invalid one.
fn device_of(port: &serial::UsbPort) -> Option<MsoDevice> {
    if port.vid != protocol::USB_VENDOR || port.pid != protocol::USB_PRODUCT {
        return None;
    }
    let number = port.serial.as_deref().unwrap_or("");
    let Some(calibration) = protocol::parse_serial(number) else {
        log::error!("Invalid serial: {number}.");
        return None;
    };
    let info = DeviceInfo {
        driver: "link-mso19",
        vendor: port.manufacturer.clone().unwrap_or_default(),
        model: port.product.clone().unwrap_or_default(),
        version: format!("r{}", calibration.hwrev),
        serial: number.to_owned(),
        conn: None,
    };
    let params = Params::parse(SERIALCOMM).expect("the driver's own");
    Some(MsoDevice::new(info, calibration, port.name.clone(), params))
}

/// What the device and its capture thread share.
struct Shared {
    /// The capture has it while it runs, and gives it back ahead of End.
    port: Mutex<Option<Box<dyn Port>>>,
    /// Something may still be on its way to the port.
    in_flight: AtomicBool,
}

/// One MSO-19.
pub struct MsoDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    conn: String,
    params: Params,
    settings: Settings,
    trigger: Option<Trigger>,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    shared: Arc<Shared>,
    worker: Option<Worker>,
    silence_bound: Duration,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl MsoDevice {
    fn new(
        info: DeviceInfo,
        calibration: protocol::Calibration,
        conn: String,
        params: Params,
    ) -> Self {
        let mut channels = vec![Channel::new(DSO_INDEX, ChannelKind::Analog, "DSO")];
        channels.extend((0..8).map(|i: usize| Channel::new(i, ChannelKind::Logic, i.to_string())));
        MsoDevice {
            info,
            channels,
            groups: vec![
                ChannelGroup::new(GROUP_DSO, vec![0]),
                ChannelGroup::new(GROUP_LA, (1..=8).collect()),
            ],
            conn,
            params,
            settings: Settings::new(calibration),
            trigger: None,
            pending: None,
            shared: Arc::new(Shared { port: Mutex::new(None), in_flight: AtomicBool::new(false) }),
            worker: None,
            silence_bound: SILENCE_BOUND,
        }
    }

    /// A device on a port that stands in for the hardware. [`Device::open`] takes
    /// it. `serial` is the USB serial number.
    pub fn from_port(port: Box<dyn Port>, serial: &str) -> Result<Self> {
        let calibration =
            protocol::parse_serial(serial).ok_or_else(|| Error::arg("Invalid serial"))?;
        let info = DeviceInfo {
            driver: "link-mso19",
            vendor: String::new(),
            model: String::new(),
            version: format!("r{}", calibration.hwrev),
            serial: serial.to_owned(),
            conn: None,
        };
        let conn = port.name().to_owned();
        let params = Params::parse(SERIALCOMM).expect("the driver's own");
        let mut device = Self::new(info, calibration, conn, params);
        device.pending = Some(port);
        Ok(device)
    }

    fn port(&self) -> MutexGuard<'_, Option<Box<dyn Port>>> {
        self.shared.port.lock().expect("port lock")
    }

    /// A capture holds the port and the settings that go with it.
    fn capturing(&self) -> bool {
        self.worker.is_some() && self.port().is_none()
    }

    fn group(group: &str) -> Result<&'static [OptionDesc]> {
        match group {
            GROUP_DSO => Ok(&DSO_OPTIONS),
            GROUP_LA => Ok(&LA_OPTIONS),
            other => Err(Error::ChannelGroup(other.to_owned())),
        }
    }

    /// `mso_configure_channels`: the session's trigger in the LA's registers, on
    /// whichever source: a match on a disabled channel is left out.
    fn trigger_registers(&self) -> (u8, u8) {
        let mut value = 0u8;
        let mut mask = 0xffu8;
        let conditions = self.trigger.iter().flat_map(|t| &t.stages).flat_map(|s| &s.conditions);
        for condition in conditions {
            let enabled = self
                .channels
                .iter()
                .any(|c| c.kind == ChannelKind::Logic && c.index == condition.channel && c.enabled);
            if !enabled {
                continue;
            }
            let bit = 1u8 << (condition.channel & 7);
            mask &= !bit;
            if condition.matches == TriggerMatch::One {
                value |= bit;
            }
        }
        (value, mask)
    }

    /// The sequence of `dev_acquisition_start`, through the first status.
    fn arm(&mut self, port: &mut dyn Port) -> Result<()> {
        let (value, mask) = self.trigger_registers();
        self.settings.la_trigger = value;
        self.settings.la_trigger_mask = mask;
        settle(port, &self.shared.in_flight, SETTLE_BOUND)?;
        port.purge()?;
        log::debug!("Requesting FSM reset.");
        protocol::send(port, &self.settings.reset_fsm_words())?;
        self.settings.apply_coupling();
        let rate = self.settings.cur_rate;
        let (words, slow) = protocol::rate_words(rate).ok_or_else(unsupported_rate)?;
        protocol::send(port, &words)?;
        self.settings.set_slow_mode(slow);
        log::debug!("Setting dac word for the offset.");
        protocol::send(port, &self.settings.dac_offset_words())?;
        log::debug!("Setting dac word for the logic threshold.");
        protocol::send(port, &self.settings.dac_threshold_words())?;
        protocol::send(port, &self.settings.trigger_words())?;
        log::debug!("Requesting trigger arm.");
        protocol::send(port, &self.settings.arm_words())?;
        // A reply that fails here may still come.
        self.shared.in_flight.store(true, Ordering::SeqCst);
        protocol::read_status(port, STATUS_TIMEOUT)?;
        // What this status says is not kept: only one the loop reads can start
        // the dump, as it sends the request.
        log::debug!("Requesting status.");
        protocol::send(port, &protocol::read_status_request())?;
        Ok(())
    }

    /// `dev_open` after the port's open, which has flushed it.
    fn init(&mut self, port: &mut dyn Port) -> Result<()> {
        settle(port, &self.shared.in_flight, SETTLE_BOUND)?;
        port.purge()?;
        log::debug!("Requesting ADC reset.");
        protocol::send(port, &self.settings.reset_adc_words())?;
        port.purge()?;
        self.shared.in_flight.store(true, Ordering::SeqCst);
        protocol::read_status(port, STATUS_TIMEOUT)?;
        self.shared.in_flight.store(false, Ordering::SeqCst);
        log::debug!("Requesting FSM reset.");
        protocol::send(port, &self.settings.reset_fsm_words())
    }

    fn set_group(&mut self, key: &Key, value: &Value, group: &str) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let name = || value.as_str().ok_or_else(bad);
        let dso = group == GROUP_DSO;
        match key {
            Key::TriggerSlope => {
                let slopes = if dso { &DSO_SLOPES } else { &LA_SLOPES };
                let index = index_of(slopes, name()?).ok_or_else(bad)?;
                if dso {
                    self.settings.dso_slope = index;
                } else {
                    self.settings.la_slope = index;
                }
            }
            Key::TriggerLevel if dso => self.settings.set_trigger_level(number(value, key)?),
            Key::Offset if dso => self.settings.set_offset(number(value, key)?),
            Key::Coupling if dso => {
                self.settings.coupling = index_of(&COUPLINGS, name()?).ok_or_else(bad)?;
            }
            Key::ProbeFactor if dso => {
                // C stores a uint16_t, so 65536 is the 0 it has just refused.
                let factor = value
                    .as_u64()
                    .filter(|f| *f > 0)
                    .and_then(|f| u16::try_from(f).ok())
                    .ok_or_else(bad)?;
                self.settings.set_probe_factor(factor);
            }
            Key::LogicThreshold if !dso => {
                self.settings.logic_threshold =
                    index_of(&LOGIC_THRESHOLDS, name()?).ok_or_else(bad)?;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }
}

fn unsupported_rate() -> Error {
    log::error!("Unsupported rate.");
    Error::device("generic/unspecified error")
}

/// A float setting. No number goes through C's comparisons, to a register of 0.
fn number(value: &Value, key: &Key) -> Result<f64> {
    value
        .as_f64()
        .filter(|v| !v.is_nan())
        .ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())))
}

fn index_of(names: &[&str], name: &str) -> Option<usize> {
    names.iter().position(|n| *n == name)
}

fn str_list(names: &[&str]) -> ValueList {
    ValueList::Str(names.iter().map(|n| (*n).to_owned()).collect())
}

/// When something is left on its way (a stop or a failure with a request out),
/// waits for the line to go quiet and drops what came. A purge alone drops only
/// what has arrived, and what comes later reads as the next status.
fn settle(port: &mut dyn Port, in_flight: &AtomicBool, bound: Duration) -> Result<()> {
    if !in_flight.swap(false, Ordering::SeqCst) {
        return Ok(());
    }
    log::debug!("Waiting for the line to go quiet.");
    let start = Instant::now();
    let mut quiet_since = Instant::now();
    let mut empty = 0;
    let mut buf = [0u8; PASS_BYTES];
    // Time alone is no quiet: a stalled host would find it without having looked,
    // so it takes reads with nothing as well.
    while (quiet_since.elapsed() < QUIET || empty < QUIET_READS) && start.elapsed() < bound {
        if port.read(&mut buf, STOP_CHECK)? > 0 {
            quiet_since = Instant::now();
            empty = 0;
        } else {
            empty += 1;
        }
    }
    Ok(())
}

/// A client that drops a device mid-capture leaves no thread polling, with the
/// port held, for as long as the trigger does not fire.
impl Drop for MsoDevice {
    fn drop(&mut self) {
        if let Err(e) = self.close() {
            log::debug!("Could not close the device: {e}.");
        }
    }
}

impl Device for MsoDevice {
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
        Ok(match group {
            None => DEV_OPTIONS.to_vec(),
            Some(g) => Self::group(g)?.to_vec(),
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let s = &self.settings;
        let unknown = || Error::UnknownKey(key.id().to_owned());
        let Some(group) = group else {
            return Ok(match key {
                Key::LimitSamples => Value::U64(NUM_SAMPLES as u64),
                Key::Samplerate => Value::U64(s.cur_rate),
                Key::TriggerSource => Value::from(match s.trigger_source {
                    TriggerSource::Dso => TRIGGER_SOURCES[0],
                    TriggerSource::La => TRIGGER_SOURCES[1],
                }),
                Key::HorizTriggerpos => Value::F64(s.horiz_triggerpos),
                _ => return Err(unknown()),
            });
        };
        Self::group(group)?;
        let dso = group == GROUP_DSO;
        Ok(match key {
            Key::TriggerSlope if dso => Value::from(DSO_SLOPES[s.dso_slope]),
            Key::TriggerSlope => Value::from(LA_SLOPES[s.la_slope]),
            Key::Coupling if dso => Value::from(COUPLINGS[s.coupling]),
            Key::ProbeFactor if dso => Value::U64(u64::from(s.probe_factor)),
            Key::TriggerLevel if dso => Value::F64(s.trigger_level),
            Key::Offset if dso => Value::F64(s.offset),
            Key::LogicThreshold if !dso => Value::from(LOGIC_THRESHOLDS[s.logic_threshold]),
            _ => return Err(unknown()),
        })
    }

    /// A rate goes out at once. What else is set goes out at the next start, so a
    /// capture running refuses it all, to leave what it decodes with alone.
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if self.capturing() {
            return Err(Error::Busy);
        }
        if let Some(group) = group {
            Self::group(group)?;
            return self.set_group(key, value, group);
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => {
                if value.as_u64() != Some(NUM_SAMPLES as u64) {
                    log::error!("Only {NUM_SAMPLES} samples are supported.");
                    return Err(bad());
                }
            }
            Key::Samplerate => {
                let rate = value.as_u64().ok_or_else(bad)?;
                let (words, slow) = protocol::rate_words(rate).ok_or_else(unsupported_rate)?;
                {
                    let mut port = self.shared.port.lock().expect("port lock");
                    let port = port.as_mut().ok_or_else(|| Error::device("device is not open"))?;
                    protocol::send(port.as_mut(), &words)?;
                }
                self.settings.set_slow_mode(slow);
                self.settings.cur_rate = rate;
            }
            Key::TriggerSource => {
                self.settings.trigger_source =
                    match value.as_str().and_then(|n| index_of(&TRIGGER_SOURCES, n)) {
                        Some(0) => TriggerSource::Dso,
                        Some(_) => TriggerSource::La,
                        None => return Err(bad()),
                    };
            }
            Key::HorizTriggerpos => {
                let position = number(value, key)?;
                if !(-10.0..=1.0).contains(&position) {
                    log::error!("Trigger position ({position:.6}) should be between -10.0 and 1.0");
                    return Err(bad());
                }
                self.settings.set_trigger_pos(position);
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let unknown = || Error::UnknownKey(key.id().to_owned());
        let Some(group) = group else {
            return Ok(match key {
                Key::Samplerate => {
                    ValueList::RangeStepU64 { min: 100, max: 200_000_000, step: 100 }
                }
                Key::TriggerSource => str_list(&TRIGGER_SOURCES),
                Key::TriggerMatch => {
                    ValueList::TriggerMatches(vec![TriggerMatch::Zero, TriggerMatch::One])
                }
                _ => return Err(unknown()),
            });
        };
        Self::group(group)?;
        let dso = group == GROUP_DSO;
        Ok(match key {
            Key::Coupling if dso => str_list(&COUPLINGS),
            Key::TriggerSlope if dso => str_list(&DSO_SLOPES),
            Key::TriggerSlope => str_list(&LA_SLOPES),
            Key::LogicThreshold if !dso => str_list(&LOGIC_THRESHOLDS),
            _ => return Err(unknown()),
        })
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        self.trigger = trigger;
        Ok(())
    }

    fn open(&mut self) -> Result<()> {
        if self.port().is_some() {
            return Ok(());
        }
        let mut port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => serial::open(&self.conn, self.params)?,
        };
        self.init(port.as_mut())?;
        *self.port() = Some(port);
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        *self.port() = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        // The thread gives the port back ahead of End, so a client that has seen
        // End finds it here, where `is_finished` may say no. What is left of the
        // thread is that End, which a stop bounds.
        if let Some(worker) = &self.worker {
            if self.port().is_none() {
                return Err(Error::Busy);
            }
            worker.stop.store(true, Ordering::SeqCst);
        }
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let Some(mut port) = self.port().take() else {
            return Err(Error::device("device is not open"));
        };
        if let Err(e) = self.arm(port.as_mut()) {
            *self.port() = Some(port);
            return Err(e);
        }
        let capture =
            Capture { settings: self.settings.clone(), silence_bound: self.silence_bound };
        let shared = Arc::clone(&self.shared);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let spawned = std::thread::Builder::new().name("link-mso19 acquisition".to_owned()).spawn(
            move || {
                let (done, outcome) = capture.run(port.as_mut(), &sink, &stop2);
                shared.in_flight.store(!done, Ordering::SeqCst);
                *shared.port.lock().expect("port lock") = Some(port);
                send_end(&sink, &stop2, outcome);
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

    /// Asks the thread to end, and does not wait: it may be sending. Nothing goes
    /// to the device, as in C.
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

/// How a wait for bytes ended.
enum Waited {
    /// This many came.
    Bytes(usize),
    /// Asked to.
    Stopped,
    /// [`Capture::silence_bound`] is up.
    Silent,
}

/// `mso_receive_data` on its own thread.
struct Capture {
    settings: Settings,
    silence_bound: Duration,
}

impl Capture {
    /// True when nothing is left on its way: the whole dump came. And how it went.
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) -> (bool, Result<()>) {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return (false, Ok(()));
        }
        let mut gave_up = None;
        match self.acquire(port, stop, &mut gave_up) {
            Ok(Some(dump)) => {
                self.send(&dump, sink, stop);
                log::info!("Requested number of samples reached.");
                (true, Ok(()))
            }
            // A reply may be on its way.
            Ok(None) => (false, gave_up.map_or(Ok(()), Err)),
            Err(e) => {
                log::error!("{e}");
                (false, Err(e))
            }
        }
    }

    /// Waits for a byte: what has come, up to `buf`'s size.
    fn wait(&self, port: &mut dyn Port, buf: &mut [u8], stop: &AtomicBool) -> Result<Waited> {
        let start = Instant::now();
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ok(Waited::Stopped);
            }
            let n = port.read(buf, STOP_CHECK)?;
            if n > 0 {
                return Ok(Waited::Bytes(n));
            }
            if start.elapsed() >= self.silence_bound {
                return Ok(Waited::Silent);
            }
            // A hung up tty answers at once, with nothing.
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// The polls and the dump. `None` when no dump is to come, with what went
    /// wrong logged and in `gave_up`, or on a stop.
    fn acquire(
        &self,
        port: &mut dyn Port,
        stop: &AtomicBool,
        gave_up: &mut Option<Error>,
    ) -> Result<Option<Vec<u8>>> {
        let mut buf = vec![0u8; PASS_BYTES];
        loop {
            match self.wait(port, &mut buf[..1], stop)? {
                Waited::Stopped => return Ok(None),
                Waited::Silent => {
                    log::error!("The device stopped answering.");
                    *gave_up = Some(Error::device("the device stopped answering"));
                    return Ok(None);
                }
                Waited::Bytes(_) => {}
            }
            // C's read is for all of a pass and takes what follows the status with
            // it, and the wait is its pace.
            serial::read_blocking(port, &mut buf[1..], POLL_READ)?;
            let status = buf[0];
            if !protocol::status_valid(status) {
                log::error!("Invalid status byte {status:02x}");
                *gave_up = Some(Error::device(format!("invalid status byte {status:02x}")));
                return Ok(None);
            }
            log::debug!("Status is: {status:#x}.");
            if protocol::status_ready(status) {
                break;
            }
            log::debug!("Requesting status.");
            protocol::send(port, &protocol::read_status_request())?;
        }
        log::debug!("Requesting buffer dump.");
        protocol::send(port, &protocol::read_buffer_request())?;
        let mut dump = vec![0u8; DUMP_BYTES];
        let mut got = 0;
        while got < DUMP_BYTES {
            let end = (got + PASS_BYTES).min(DUMP_BYTES);
            match self.wait(port, &mut dump[got..end], stop)? {
                Waited::Stopped => return Ok(None),
                // Not "stopped answering": on this line the tty takes XON and XOFF
                // out of what comes. mso19fcgi takes 3070 bytes as enough, so a unit
                // that sends a little short is possible, and loses this capture.
                Waited::Silent => {
                    log::error!("{got} of {DUMP_BYTES} bytes came.");
                    *gave_up = Some(Error::device(format!("{got} of {DUMP_BYTES} bytes came")));
                    return Ok(None);
                }
                Waited::Bytes(n) => got += n,
            }
        }
        Ok(Some(dump))
    }

    /// The samples, split at the trigger. A stop drops what is left.
    fn send(&self, dump: &[u8], sink: &PacketSink, stop: &AtomicBool) {
        let (logic, analog) = self.settings.decode(dump);
        let (pre, has_trigger) = self.settings.trigger_split();
        let go_on = || !stop.load(Ordering::SeqCst);
        let part = |range: std::ops::Range<usize>| {
            range.is_empty()
                || (sink.send_while(Packet::logic(1, logic[range.clone()].to_vec()), go_on)
                    && sink.send_while(
                        Packet::Analog(AnalogData {
                            channels: vec![DSO_INDEX],
                            samples: analog[range].to_vec(),
                            mq: Mq::Voltage,
                            flags: MqFlags::DC,
                            unit: Unit::Volt,
                            digits: 3,
                        }),
                        go_on,
                    ))
        };
        let _ = part(0..pre)
            && (!has_trigger || sink.send_while(Packet::Trigger, go_on))
            && part(pre..NUM_SAMPLES);
    }
}

#[cfg(test)]
mod tests;
