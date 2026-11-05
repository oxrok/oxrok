// Counterpart in sigrok:
//   libsigrok/src/hardware/rigol-ds/api.c,
//   libsigrok/src/hardware/rigol-ds/protocol.c,
//   libsigrok/src/hardware/rigol-ds/protocol.h,
//   Copyright (C) 2012 Martin Ling <martin-sigrok@earth.li>,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2013 Mathias Grimmberger <mgri@zaphod.sax.de>,
//   with commits by Uwe Hermann, Aurelien Jacobs, Daniel Elstner,
//   Gerhard Sittig, Soeren Apel, Frank Stettner, Guido Trentalancia,
//   Lars-Peter Clausen, Sylvain Munaut, Timo Kokkonen, Wolfram Sang,
//   Alexandru Gagniuc, Marc Schink
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! Rigol and Agilent oscilloscopes over SCPI: the `rigol-ds` driver.
//!
//! Five protocol generations share one driver, from the VS5000 and
//! DS1000 to the MSO5000; see [`models`]. A capture downloads one frame
//! per enabled channel from the screen (`Live`), the acquisition memory
//! (`Memory`) or a stored segment (`Segmented`), and streams it as
//! analog packets between `FrameBegin` and `FrameEnd`.
//!
//! `conn=` takes a SCPI resource (`usbtmc`, `usbtmc/BUS.ADDR`,
//! `tcp-raw/HOST/PORT`, `tcp-rigol/HOST/PORT`, a serial port); without
//! it every USBTMC device is asked `*IDN?`. `OXROK_SCPI_RECORD=FILE`
//! records the session for replay.

#![forbid(unsafe_code)]

pub mod models;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::scpi::Scpi;
use models::{Format, Model, Protocol};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Oscilloscope];
// sigrok's driver lists no conn option, so sigrok-cli prints none.
static DEV_OPTIONS: [OptionDesc; 9] = [
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET),
    OptionDesc::new(Key::Timebase, Access::GET_SET_LIST),
    OptionDesc::new(Key::NumHdiv, Access::GET),
    OptionDesc::new(Key::HorizTriggerpos, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::TriggerSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSlope, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerLevel, Access::GET_SET),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
];
static ANALOG_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::NumVdiv, Access::GET),
    OptionDesc::new(Key::Vdiv, Access::GET_SET_LIST),
    OptionDesc::new(Key::Coupling, Access::GET_SET_LIST),
    OptionDesc::new(Key::ProbeFactor, Access::GET_SET_LIST),
];

/// Bytes read from the instrument per call while downloading.
const READ_CHUNK: usize = 32 * 1024;
/// Samples per `:WAV:START`/`:WAV:STOP` window on the V4 protocol.
const BLOCK_SAMPLES: usize = 30_000;
/// Digital inputs on the models that have them.
const DIGITAL_CHANNELS: usize = 16;

/// Where a frame comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DataSource {
    Live,
    Memory,
    Segmented,
}

impl DataSource {
    fn name(self) -> &'static str {
        match self {
            DataSource::Live => "Live",
            DataSource::Memory => "Memory",
            DataSource::Segmented => "Segmented",
        }
    }
}

/// The driver.
pub struct RigolDsDriver;

impl Driver for RigolDsDriver {
    fn id(&self) -> &'static str {
        "rigol-ds"
    }

    fn long_name(&self) -> &'static str {
        "Rigol DS/MSO oscilloscope"
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
        let found = crate::scpi::scan(conn.as_deref(), serialcomm.as_deref(), |mut scpi| {
            let id = match scpi.hw_id() {
                Ok(id) => id,
                Err(e) => {
                    log::info!("Couldn't get IDN response, retrying: {e}");
                    scpi.hw_id()?
                }
            };
            let Some(model) = models::find(&id.manufacturer, &id.model) else {
                log::info!("{} {} is not a supported scope.", id.manufacturer, id.model);
                return Ok(None);
            };
            Ok(Some(RigolDevice::new(model, &id, scpi)))
        })?;
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// What the scope reported about itself, kept up to date by every
/// setting the driver makes.
#[derive(Clone, Debug)]
struct State {
    analog_on: Vec<bool>,
    digital_on: Vec<bool>,
    la_enabled: bool,
    timebase: f32,
    attenuation: Vec<f32>,
    vdiv: Vec<f32>,
    vert_offset: Vec<f32>,
    coupling: Vec<String>,
    trigger_source: String,
    horiz_triggerpos: f32,
    trigger_slope: String,
    trigger_level: f32,
    limit_frames: u64,
    data_source: DataSource,
    sample_rate: f64,
}

/// One scope.
pub struct RigolDevice {
    model: &'static Model,
    info: DeviceInfo,
    resource: String,
    format: Format,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    state: State,
    pending: Option<Scpi>,
    scpi: Arc<Mutex<Option<Scpi>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
    state: Arc<Mutex<State>>,
}

impl RigolDevice {
    /// A device of `model` on an open instrument.
    pub fn new(model: &'static Model, id: &crate::scpi::HwInfo, scpi: Scpi) -> RigolDevice {
        let mut channels = Vec::new();
        let mut groups = Vec::new();
        for i in 0..model.analog_channels {
            let name = format!("CH{}", i + 1);
            channels.push(Channel::new(i, ChannelKind::Analog, name.clone()));
            groups.push(ChannelGroup::new(name, vec![i]));
        }
        if model.has_digital {
            let first = channels.len();
            for i in 0..DIGITAL_CHANNELS {
                channels.push(Channel::new(i, ChannelKind::Logic, format!("D{i}")));
            }
            groups.push(ChannelGroup::new("LA", (first..first + DIGITAL_CHANNELS).collect()));
        }
        let mut format = model.series.format;
        // DS1000 firmware before 0.2.4 sends bare bytes. The version
        // string has five fields (00.02.06.00.01); the first three
        // count, as in libsigrok.
        if model.series.protocol == Protocol::V2 {
            let parts: Vec<Option<u32>> =
                id.firmware_version.split('.').take(3).map(|p| p.parse().ok()).collect();
            if let [Some(a), Some(b), Some(c)] = parts[..]
                && a * 100 + b * 10 + c < 24
            {
                format = Format::Raw;
            }
        }
        let n = model.analog_channels;
        RigolDevice {
            model,
            resource: scpi.resource().to_owned(),
            info: DeviceInfo {
                driver: "rigol-ds",
                vendor: model.series.vendor.name.to_owned(),
                model: model.name.to_owned(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            format,
            channels,
            groups,
            state: State {
                analog_on: vec![true; n],
                digital_on: vec![false; DIGITAL_CHANNELS],
                la_enabled: false,
                timebase: 1e-3,
                attenuation: vec![1.0; n],
                vdiv: vec![1.0; n],
                vert_offset: vec![0.0; n],
                coupling: vec!["DC".to_owned(); n],
                trigger_source: "CHAN1".to_owned(),
                horiz_triggerpos: 0.0,
                trigger_slope: "POS".to_owned(),
                trigger_level: 0.0,
                limit_frames: 0,
                data_source: DataSource::Live,
                sample_rate: 0.0,
            },
            pending: Some(scpi),
            scpi: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    fn protocol(&self) -> Protocol {
        self.model.series.protocol
    }

    /// Runs `f` on the open instrument.
    fn with_scpi<T>(&self, f: impl FnOnce(&mut Scpi) -> Result<T>) -> Result<T> {
        let mut guard = self.scpi.lock().expect("scpi lock");
        let scpi = guard.as_mut().ok_or(Error::Busy)?;
        f(scpi)
    }

    fn config_set(&self, command: &str) -> Result<()> {
        let protocol = self.protocol();
        self.with_scpi(|s| config_set(s, protocol, command))
    }

    /// The analog channel of a group name, `CH1` to `CH4`.
    fn analog_group(&self, group: Option<&str>) -> Result<Option<usize>> {
        let Some(g) = group else {
            return Ok(None);
        };
        if g == "LA" && self.model.has_digital {
            return Err(Error::NotSupported(format!("no such option on channel group {g}")));
        }
        let idx = self
            .groups
            .iter()
            .position(|cg| cg.name == g)
            .filter(|i| *i < self.model.analog_channels)
            .ok_or_else(|| Error::ChannelGroup(g.to_owned()))?;
        Ok(Some(idx))
    }

    /// Reads the scope's settings into the state, as libsigrok does at
    /// open time.
    fn read_config(&mut self) -> Result<()> {
        let model = self.model;
        let protocol = self.protocol();
        let mut st = self.state.clone();
        self.with_scpi(|s| {
            for i in 0..model.analog_channels {
                st.analog_on[i] = s.get_bool(&format!(":CHAN{}:DISP?", i + 1))?;
            }
            if model.has_digital {
                let la = if protocol >= Protocol::V3 { ":LA:STAT?" } else { ":LA:DISP?" };
                st.la_enabled = s.get_bool(la)?;
                for i in 0..DIGITAL_CHANNELS {
                    let cmd = match protocol {
                        Protocol::V5 => format!(":LA:DISP? D{i}"),
                        Protocol::V3 | Protocol::V4 => format!(":LA:DIG{i}:DISP?"),
                        _ => format!(":DIG{i}:TURN?"),
                    };
                    st.digital_on[i] = s.get_bool(&cmd)?;
                }
            }
            st.timebase = s.get_f32(":TIM:SCAL?")?;
            for i in 0..model.analog_channels {
                // The DSO1000B answers with an X after the factor.
                let text = s.get_string(&format!(":CHAN{}:PROB?", i + 1))?;
                let text = text.strip_suffix('X').unwrap_or(&text);
                st.attenuation[i] = text
                    .trim()
                    .parse()
                    .map_err(|_| Error::device(format!("bad probe factor '{text}'")))?;
            }
            read_vertical(s, model, &mut st)?;
            for i in 0..model.analog_channels {
                st.coupling[i] = s.get_string(&format!(":CHAN{}:COUP?", i + 1))?;
            }
            st.trigger_source = s.get_string(":TRIG:EDGE:SOUR?")?;
            st.horiz_triggerpos = s.get_f32(model.triggerpos_cmds.0)?;
            st.trigger_slope = s.get_string(":TRIG:EDGE:SLOP?")?;
            st.trigger_level = s.get_f32(":TRIG:EDGE:LEV?")?;
            Ok(())
        })?;
        for i in 0..model.analog_channels {
            self.channels[i].enabled = st.analog_on[i];
        }
        if model.has_digital {
            for i in 0..DIGITAL_CHANNELS {
                self.channels[model.analog_channels + i].enabled = st.digital_on[i];
            }
        }
        self.state = st;
        Ok(())
    }

    fn analog_frame_size(&self) -> usize {
        let enabled =
            self.channels.iter().filter(|c| c.kind == ChannelKind::Analog && c.enabled).count();
        if enabled == 0 {
            return 0;
        }
        match self.state.data_source {
            DataSource::Live => self.model.series.live_samples,
            _ => self.model.series.buffer_samples / enabled,
        }
    }

    fn sync_state(&mut self) {
        if let Some(w) = &self.worker {
            let shared = w.state.lock().expect("state lock");
            self.state.sample_rate = shared.sample_rate;
        }
    }
}

/// Sends a setting; the DS1000 needs a pause instead of `*OPC?`.
fn config_set(scpi: &mut Scpi, protocol: Protocol, command: &str) -> Result<()> {
    scpi.send(command)?;
    if protocol == Protocol::V2 {
        std::thread::sleep(Duration::from_millis(100));
        Ok(())
    } else {
        scpi.get_opc()
    }
}

fn read_vertical(scpi: &mut Scpi, model: &Model, st: &mut State) -> Result<()> {
    for i in 0..model.analog_channels {
        st.vdiv[i] = scpi.get_f32(&format!(":CHAN{}:SCAL?", i + 1))?;
    }
    for i in 0..model.analog_channels {
        st.vert_offset[i] = scpi.get_f32(&format!(":CHAN{}:OFFS?", i + 1))?;
    }
    Ok(())
}

/// True when two rationals are the same value, whatever their form:
/// the parser reduces `500us` to 1/2000 while the tables keep
/// 500/1000000.
fn same_ratio(a: (u64, u64), b: (u64, u64)) -> bool {
    u128::from(a.0) * u128::from(b.1) == u128::from(b.0) * u128::from(a.1)
}

/// The table entry nearest to `value`.
fn nearest(table: &[(u64, u64)], value: f32) -> Option<(u64, u64)> {
    table.iter().copied().min_by(|a, b| {
        let da = (a.0 as f32 / a.1 as f32 - value).abs();
        let db = (b.0 as f32 / b.1 as f32 - value).abs();
        da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
    })
}

fn source_to_scope(name: &str) -> &str {
    match name {
        "AC Line" => "ACL",
        "CH1" => "CHAN1",
        "CH2" => "CHAN2",
        "CH3" => "CHAN3",
        "CH4" => "CHAN4",
        other => other,
    }
}

fn source_from_scope(name: &str) -> &str {
    match name {
        "ACL" => "AC Line",
        "CHAN1" => "CH1",
        "CHAN2" => "CH2",
        "CHAN3" => "CH3",
        "CHAN4" => "CH4",
        other => other,
    }
}

/// Formats a float the way the C code's `%.Nf` did, for the scope.
fn fixed(value: f64, decimals: usize) -> String {
    format!("{value:.decimals$}")
}

impl Device for RigolDevice {
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
            Some("LA") if self.model.has_digital => Ok(Vec::new()),
            Some(g) => {
                self.analog_group(Some(g))?;
                Ok(ANALOG_OPTIONS.to_vec())
            }
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let st = &self.state;
        let ch = self.analog_group(group)?;
        let need_group = |v: Option<usize>| v.ok_or(Error::ChannelGroup(String::new()));
        Ok(match key {
            Key::NumHdiv => Value::I32(self.model.series.num_hdiv),
            Key::NumVdiv => Value::I32(self.model.vdivs().len() as i32),
            Key::DataSource => Value::Str(st.data_source.name().to_owned()),
            Key::LimitFrames => Value::U64(st.limit_frames),
            Key::Samplerate => {
                let rate = if st.sample_rate > 0.0 {
                    st.sample_rate
                } else {
                    f64::from(
                        self.analog_frame_size() as f32
                            / (st.timebase * self.model.series.num_hdiv as f32),
                    )
                };
                Value::U64(rate as u64)
            }
            Key::TriggerSource => Value::Str(source_from_scope(&st.trigger_source).to_owned()),
            Key::TriggerSlope => {
                if st.trigger_slope.starts_with("POS") {
                    Value::Str("r".to_owned())
                } else if st.trigger_slope.starts_with("NEG") {
                    Value::Str("f".to_owned())
                } else {
                    return Err(Error::not_supported("unknown trigger slope"));
                }
            }
            Key::TriggerLevel => Value::F64(f64::from(st.trigger_level)),
            Key::Timebase => {
                let (p, q) = nearest(self.model.timebases(), st.timebase)
                    .ok_or_else(|| Error::not_supported("timebase"))?;
                Value::Rational(p, q)
            }
            Key::Vdiv => {
                let i = need_group(ch)?;
                let (p, q) = nearest(models::VDIVS, st.vdiv[i])
                    .ok_or_else(|| Error::not_supported("vdiv"))?;
                Value::Rational(p, q)
            }
            Key::Coupling => Value::Str(st.coupling[need_group(ch)?].clone()),
            Key::ProbeFactor => Value::U64(st.attenuation[need_group(ch)?] as u64),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let ch = self.analog_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let need_group = |v: Option<usize>| v.ok_or(Error::ChannelGroup(String::new()));
        match key {
            Key::LimitFrames => self.state.limit_frames = value.as_u64().ok_or_else(bad)?,
            Key::TriggerSlope => {
                let s = value.as_str().ok_or_else(bad)?;
                if !models::TRIGGER_SLOPES.contains(&s) {
                    return Err(bad());
                }
                let slope = if s == "r" { "POS" } else { "NEG" };
                self.config_set(&format!(":TRIG:EDGE:SLOP {slope}"))?;
                slope.clone_into(&mut self.state.trigger_slope);
            }
            Key::HorizTriggerpos => {
                let pos = value.as_f64().ok_or_else(bad)?;
                if !(0.0..=1.0).contains(&pos) {
                    return Err(Error::Arg(format!("Invalid horiz. trigger position: {pos}.")));
                }
                // The position is a fraction of the frame; the scope
                // wants seconds from the screen centre.
                let seconds = -(pos - 0.5)
                    * f64::from(self.state.timebase)
                    * f64::from(self.model.series.num_hdiv);
                let cmd = self.model.triggerpos_cmds.1.replace("{}", &fixed(seconds, 6));
                self.config_set(&cmd)?;
                self.state.horiz_triggerpos = pos as f32;
            }
            Key::TriggerLevel => {
                let level = value.as_f64().ok_or_else(bad)?;
                self.config_set(&format!(":TRIG:EDGE:LEV {}", fixed(level, 3)))?;
                self.state.trigger_level = level as f32;
            }
            Key::Timebase => {
                let Value::Rational(p, q) = value else { return Err(bad()) };
                if !self.model.timebases().iter().any(|t| same_ratio(*t, (*p, *q))) {
                    return Err(bad());
                }
                let tb = *p as f64 / *q as f64;
                self.config_set(&format!(":TIM:SCAL {}", fixed(tb, 9)))?;
                self.state.timebase = tb as f32;
            }
            Key::TriggerSource => {
                let s = value.as_str().ok_or_else(bad)?;
                if !self.model.trigger_sources().contains(&s) {
                    return Err(bad());
                }
                let scope = source_to_scope(s).to_owned();
                self.config_set(&format!(":TRIG:EDGE:SOUR {scope}"))?;
                self.state.trigger_source = scope;
            }
            Key::Vdiv => {
                let i = need_group(ch)?;
                let Value::Rational(p, q) = value else { return Err(bad()) };
                if !models::VDIVS.iter().any(|v| same_ratio(*v, (*p, *q))) {
                    return Err(bad());
                }
                let v = *p as f64 / *q as f64;
                self.config_set(&format!(":CHAN{}:SCAL {}", i + 1, fixed(v, 3)))?;
                self.state.vdiv[i] = v as f32;
            }
            Key::Coupling => {
                let i = need_group(ch)?;
                let s = value.as_str().ok_or_else(bad)?;
                if !models::COUPLINGS.contains(&s) {
                    return Err(bad());
                }
                self.config_set(&format!(":CHAN{}:COUP {s}", i + 1))?;
                s.clone_into(&mut self.state.coupling[i]);
            }
            Key::ProbeFactor => {
                let i = need_group(ch)?;
                let f = value.as_u64().ok_or_else(bad)?;
                if !models::PROBE_FACTORS.contains(&f) {
                    return Err(bad());
                }
                self.config_set(&format!(":CHAN{}:PROB {f}", i + 1))?;
                self.state.attenuation[i] = f as f32;
                let model = self.model;
                let mut st = self.state.clone();
                self.with_scpi(|s| read_vertical(s, model, &mut st))?;
                self.state = st;
            }
            Key::DataSource => {
                let s = value.as_str().ok_or_else(bad)?;
                let protocol = self.protocol();
                self.state.data_source = match s {
                    "Live" => DataSource::Live,
                    "Memory" if protocol >= Protocol::V2 => DataSource::Memory,
                    "Segmented" if protocol >= Protocol::V3 => DataSource::Segmented,
                    _ => return Err(Error::Arg(format!("Unknown data source: '{s}'."))),
                };
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let ch = self.analog_group(group)?;
        let str_list =
            |items: &[&str]| ValueList::Str(items.iter().map(|s| (*s).to_owned()).collect());
        Ok(match key {
            Key::Coupling if ch.is_some() => str_list(models::COUPLINGS),
            Key::ProbeFactor if ch.is_some() => ValueList::U64(models::PROBE_FACTORS.to_vec()),
            Key::Vdiv if ch.is_some() => ValueList::Rational(self.model.vdivs().to_vec()),
            Key::Coupling | Key::ProbeFactor | Key::Vdiv => {
                return Err(Error::ChannelGroup(String::new()));
            }
            Key::Timebase => ValueList::Rational(self.model.timebases().to_vec()),
            Key::TriggerSource => str_list(self.model.trigger_sources()),
            Key::TriggerSlope => str_list(models::TRIGGER_SLOPES),
            Key::DataSource => str_list(self.model.data_sources()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.scpi.lock().expect("scpi lock").is_some() {
            return Ok(());
        }
        let scpi = if let Some(s) = self.pending.take() {
            s
        } else {
            crate::scpi::open(&crate::scpi::Resource::parse(&self.resource)?, None)?
        };
        *self.scpi.lock().expect("scpi lock") = Some(scpi);
        self.read_config()
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if self.protocol() == Protocol::V2 {
            // The DS1000 locks its front panel while remote.
            let _ = self.config_set(":KEY:LOCK DISABLE");
        }
        self.scpi.lock().expect("scpi lock").take();
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let scpi = self.scpi.lock().expect("scpi lock").take();
        let Some(scpi) = scpi else {
            return Err(Error::device("device is not open"));
        };
        let enabled: Vec<Channel> = self.channels.clone();
        if !enabled.iter().any(|c| c.enabled) {
            *self.scpi.lock().expect("scpi lock") = Some(scpi);
            return Err(Error::arg("No channels enabled."));
        }
        self.state.sample_rate = 0.0;
        let shared = Arc::new(Mutex::new(self.state.clone()));
        let acq = Acquisition {
            model: self.model,
            format: self.format,
            channels: enabled,
            state: Arc::clone(&shared),
        };
        let slot = Arc::clone(&self.scpi);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("rigol-ds acquisition".to_owned())
            .spawn(move || {
                let mut scpi = scpi;
                let outcome = acq.run(&mut scpi, &sink, &stop2);
                if let Err(e) = &outcome {
                    log::error!("Acquisition failed: {e}.");
                }
                sink.end(outcome);
                *slot.lock().expect("scpi lock") = Some(scpi);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle, state: shared });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        self.sync_state();
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
            let shared = w.state.lock().expect("state lock");
            // The acquisition may have switched channels on or off.
            self.state.analog_on.clone_from(&shared.analog_on);
            self.state.digital_on.clone_from(&shared.digital_on);
            self.state.la_enabled = shared.la_enabled;
            self.state.sample_rate = shared.sample_rate;
        }
        Ok(())
    }
}

/// One acquisition on its own thread.
struct Acquisition {
    model: &'static Model,
    format: Format,
    channels: Vec<Channel>,
    state: Arc<Mutex<State>>,
}

/// What a frame waits for after it is started.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Wait {
    Trigger,
    Stop,
}

/// Per channel vertical parameters for the byte to volt conversion.
#[derive(Clone, Copy, Default)]
struct Vertical {
    inc: f32,
    origin: f32,
    reference: i32,
    offset: f32,
}

impl Acquisition {
    fn protocol(&self) -> Protocol {
        self.model.series.protocol
    }

    fn set(&self, scpi: &mut Scpi, command: &str) -> Result<()> {
        config_set(scpi, self.protocol(), command)
    }

    fn enabled_list(&self) -> Vec<Channel> {
        let protocol = self.protocol();
        let mut list = Vec::new();
        let mut some_digital = false;
        for ch in &self.channels {
            if !ch.enabled {
                continue;
            }
            match ch.kind {
                ChannelKind::Analog => list.push(ch.clone()),
                // Before the V4 protocol all digital channels come in
                // one transfer, so one entry stands for all of them.
                ChannelKind::Logic => {
                    if protocol > Protocol::V3 || !some_digital {
                        list.push(ch.clone());
                    }
                    some_digital = true;
                }
            }
        }
        list
    }

    #[allow(clippy::too_many_lines)]
    fn run(&self, scpi: &mut Scpi, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let protocol = self.protocol();
        let series = self.model.series;
        let mut st = self.state.lock().expect("state lock").clone();
        let mut some_digital = false;
        for ch in &self.channels {
            match ch.kind {
                ChannelKind::Analog => {
                    if ch.enabled != st.analog_on[ch.index] {
                        self.set(
                            scpi,
                            &format!(
                                ":CHAN{}:DISP {}",
                                ch.index + 1,
                                if ch.enabled { "ON" } else { "OFF" }
                            ),
                        )?;
                        st.analog_on[ch.index] = ch.enabled;
                    }
                }
                ChannelKind::Logic => {
                    if ch.enabled {
                        some_digital = true;
                        if !st.la_enabled {
                            self.set(
                                scpi,
                                if protocol >= Protocol::V3 {
                                    ":LA:STAT ON"
                                } else {
                                    ":LA:DISP ON"
                                },
                            )?;
                            st.la_enabled = true;
                        }
                    }
                    if ch.enabled != st.digital_on[ch.index] {
                        let on = if ch.enabled { "ON" } else { "OFF" };
                        let cmd = match protocol {
                            Protocol::V5 => format!(":LA:DISP D{},{on}", ch.index),
                            Protocol::V3 | Protocol::V4 => format!(":LA:DIG{}:DISP {on}", ch.index),
                            _ => format!(":DIG{}:TURN {on}", ch.index),
                        };
                        self.set(scpi, &cmd)?;
                        st.digital_on[ch.index] = ch.enabled;
                    }
                }
            }
        }
        if st.la_enabled && !some_digital {
            self.set(scpi, if protocol >= Protocol::V3 { ":LA:STAT OFF" } else { ":LA:DISP OFF" })?;
            st.la_enabled = false;
        }
        let enabled = self.enabled_list();
        let analog_count = enabled.iter().filter(|c| c.kind == ChannelKind::Analog).count();
        let mut num_frames_segmented = 0u64;
        if st.data_source == DataSource::Segmented {
            match protocol {
                Protocol::V1 | Protocol::V2 => {
                    return Err(Error::not_supported(
                        "Data source 'Segmented' not supported on this model",
                    ));
                }
                Protocol::V3 | Protocol::V4 => {
                    let cmd = if protocol == Protocol::V4 {
                        "FUNC:WREP:FEND?"
                    } else {
                        "FUNC:WREP:FMAX?"
                    };
                    let frames = scpi.get_int(cmd)?;
                    if frames <= 0 {
                        return Err(Error::device("No segmented data available"));
                    }
                    num_frames_segmented = frames as u64;
                }
                Protocol::V5 => self.set(scpi, "REC:CURR 1")?,
            }
        }
        let mut analog_frame_size = match st.data_source {
            DataSource::Live => series.live_samples,
            _ if analog_count == 0 => 0,
            _ => series.buffer_samples / analog_count,
        };
        let mut digital_frame_size = match st.data_source {
            DataSource::Live => series.live_samples * 2,
            _ => series.buffer_samples * 2,
        };
        match protocol {
            Protocol::V2 => self.set(scpi, ":ACQ:MEMD LONG")?,
            Protocol::V3 => {
                // The DS2000 takes the memory depth only while running.
                self.set(scpi, ":RUN")?;
                self.set(scpi, &format!(":ACQ:MDEP {analog_frame_size}"))?;
                self.set(scpi, ":STOP")?;
            }
            _ => {}
        }
        if st.data_source == DataSource::Live {
            self.set(scpi, ":RUN")?;
        }
        sink.send(Packet::header());
        if st.data_source == DataSource::Live {
            st.sample_rate =
                f64::from(analog_frame_size as f32 / (st.timebase * series.num_hdiv as f32));
        } else {
            if protocol < Protocol::V3 {
                return Err(Error::not_supported("Cannot get samplerate (below V3)."));
            }
            let xinc = scpi.get_f64(":WAV:XINC?")?;
            if xinc <= 0.0 {
                return Err(Error::device("Invalid XINC value."));
            }
            st.sample_rate = 1.0 / xinc;
        }
        self.state.lock().expect("state lock").clone_from(&st);

        let mut vertical = vec![Vertical::default(); self.model.analog_channels];
        let mut num_frames = 0u64;
        let mut limit_frames = st.limit_frames;
        if num_frames_segmented != 0 && num_frames_segmented < limit_frames {
            limit_frames = num_frames_segmented;
        }
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            let first_frame = num_frames == 0;
            if limit_frames == 0 {
                log::debug!("Starting data capture for frameset {}", num_frames + 1);
            } else {
                log::debug!(
                    "Starting data capture for frameset {} of {limit_frames}",
                    num_frames + 1
                );
            }
            if !sink.send(Packet::FrameBegin) {
                return Ok(());
            }
            // Start the capture.
            let wait = match protocol {
                Protocol::V1 => Wait::Trigger,
                Protocol::V2 => {
                    if st.data_source == DataSource::Live {
                        self.set(scpi, ":WAV:POIN:MODE NORMAL")?;
                        Wait::Trigger
                    } else {
                        self.set(scpi, ":STOP")?;
                        self.set(scpi, ":WAV:POIN:MODE RAW")?;
                        let mode = scpi.get_string(":TRIG:MODE?")?;
                        self.set(scpi, &format!(":TRIG:{mode}:SWE SING"))?;
                        self.set(scpi, ":RUN")?;
                        Wait::Stop
                    }
                }
                Protocol::V3 | Protocol::V4 | Protocol::V5 => {
                    if first_frame {
                        self.set(scpi, ":WAV:FORM BYTE")?;
                    }
                    if st.data_source == DataSource::Live {
                        if first_frame {
                            self.set(scpi, ":WAV:MODE NORM")?;
                        }
                        analog_frame_size = series.live_samples;
                        digital_frame_size = series.live_samples;
                        Wait::Trigger
                    } else {
                        if protocol == Protocol::V3 {
                            if first_frame {
                                self.set(scpi, ":WAV:MODE RAW")?;
                            }
                        } else if first_frame {
                            // Channels 3 and 4 share memory with D0-7
                            // and D8-15.
                            let mut num_channels = 0;
                            for i in 0..self.model.analog_channels {
                                let pod_used = i >= 2
                                    && self.model.has_digital
                                    && (0..8).any(|j| st.digital_on[8 * (i - 2) + j]);
                                if st.analog_on[i] || pod_used {
                                    num_channels += 1;
                                }
                            }
                            let buffer_samples = series.buffer_samples;
                            if buffer_samples == 0 {
                                // The DS4000 memory depth is a menu
                                // setting.
                                let depth = scpi.get_int("ACQ:MDEP?")? as usize;
                                analog_frame_size = depth;
                                digital_frame_size = depth;
                            } else {
                                let size = match num_channels {
                                    0 | 1 => buffer_samples,
                                    2 => buffer_samples / 2,
                                    _ => buffer_samples / 4,
                                };
                                analog_frame_size = size;
                                digital_frame_size = size;
                            }
                        }
                        if st.data_source == DataSource::Segmented && protocol <= Protocol::V4 {
                            self.set(scpi, &format!("FUNC:WREP:FCUR {}", num_frames + 1))?;
                        }
                        Wait::Stop
                    }
                }
            };
            // Wait for the frame.
            match wait {
                Wait::Trigger => self.trigger_wait(scpi, &st, stop)?,
                Wait::Stop => {
                    event_wait(scpi, 'S', 'S', false, stop)?;
                    if protocol == Protocol::V3 {
                        self.check_stop(
                            scpi,
                            &enabled[0],
                            analog_frame_size,
                            digital_frame_size,
                            stop,
                        )?;
                    }
                }
            }
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            // Download every enabled channel.
            for ch in &enabled {
                self.channel_start(scpi, ch, first_frame, &st, &mut vertical)?;
                let expected = if ch.kind == ChannelKind::Analog {
                    analog_frame_size
                } else {
                    digital_frame_size
                };
                let mut received = 0usize;
                while received < expected {
                    if stop.load(Ordering::SeqCst) {
                        return Ok(());
                    }
                    if protocol >= Protocol::V4 && first_frame {
                        self.set(scpi, &format!(":WAV:START {}", received + 1))?;
                        self.set(
                            scpi,
                            &format!(
                                ":WAV:STOP {}",
                                (received + BLOCK_SAMPLES).min(analog_frame_size)
                            ),
                        )?;
                    }
                    if protocol >= Protocol::V3 {
                        self.set(scpi, ":WAV:BEG")?;
                        scpi.send(":WAV:DATA?")?;
                    }
                    scpi.read_begin()?;
                    let block_len = if self.format == Format::Ieee488 {
                        let len = read_block_header(scpi)?;
                        // Slow timebases make the DS2072 and DS1054Z
                        // answer short blocks now and then; the full
                        // one follows.
                        if st.data_source == DataSource::Live && len < expected {
                            log::debug!("Discarding a short block of {len} bytes.");
                            let mut junk = vec![0u8; len + 1];
                            read_exact(scpi, &mut junk)?;
                            continue;
                        }
                        len
                    } else {
                        expected
                    };
                    let mut block_read = 0;
                    while block_read < block_len {
                        let chunk_len = (block_len - block_read).min(READ_CHUNK);
                        let mut buf = vec![0u8; chunk_len];
                        let n = read_some(scpi, &mut buf)?;
                        buf.truncate(n);
                        block_read += n;
                        if ch.kind == ChannelKind::Analog {
                            let v = vertical[ch.index];
                            let samples: Vec<f32> = if protocol >= Protocol::V3 {
                                buf.iter()
                                    .map(|b| {
                                        (f32::from(*b) - v.reference as f32 - v.origin) * v.inc
                                    })
                                    .collect()
                            } else {
                                buf.iter()
                                    .map(|b| (128.0 - f32::from(*b)) * v.inc - v.offset)
                                    .collect()
                            };
                            let vlog = v.inc.log10();
                            let digits = -(vlog as i32) + i32::from(vlog < 0.0);
                            let data = AnalogData {
                                channels: vec![ch.index],
                                samples,
                                mq: Mq::Voltage,
                                flags: MqFlags::empty(),
                                unit: Unit::Volt,
                                digits,
                            };
                            if !sink.send(Packet::Analog(data)) {
                                return Ok(());
                            }
                        } else {
                            let unit_size = if protocol >= Protocol::V4 { 1 } else { 2 };
                            if !sink.send(Packet::logic(unit_size, buf)) {
                                return Ok(());
                            }
                        }
                    }
                    if protocol >= Protocol::V3 {
                        // The terminating linefeed.
                        let mut lf = [0u8; 1];
                        let _ = read_some(scpi, &mut lf);
                    }
                    received += block_len;
                    if self.format == Format::Ieee488
                        && st.data_source != DataSource::Live
                        && received < expected
                    {
                        self.block_wait(scpi, analog_frame_size, stop)?;
                    }
                }
                if protocol == Protocol::V3 && st.data_source != DataSource::Live {
                    // Needed before switching channels; it causes a
                    // query error on the scope, which is harmless.
                    let _ = self.set(scpi, ":WAV:END");
                }
            }
            if !sink.send(Packet::FrameEnd) {
                return Ok(());
            }
            num_frames += 1;
            if st.data_source == DataSource::Segmented && protocol == Protocol::V5 {
                // The MSO5000 has no frame count; step and read back.
                self.set(scpi, &format!("REC:CURR {}", num_frames + 1))?;
                num_frames_segmented = scpi.get_int("REC:CURR?")? as u64;
            }
            if num_frames == limit_frames
                || num_frames == num_frames_segmented
                || st.data_source == DataSource::Memory
            {
                return Ok(());
            }
        }
    }

    /// Waits for a trigger: a sleep of most of a sweep at fast
    /// timebases, else polling the trigger status.
    fn trigger_wait(&self, scpi: &mut Scpi, st: &State, stop: &AtomicBool) -> Result<()> {
        if st.timebase < 0.0499 {
            if st.timebase > 0.99e-6 {
                let sweep = f64::from(st.timebase) * f64::from(self.model.series.num_hdiv);
                std::thread::sleep(Duration::from_secs_f64(sweep * 0.85));
            }
            return Ok(());
        }
        event_wait(scpi, 'T', 'A', true, stop)
    }

    /// On the DS2000 a single shot may stop without triggering; the
    /// event status register says so, and the capture is retried.
    fn check_stop(
        &self,
        scpi: &mut Scpi,
        first: &Channel,
        analog_frame_size: usize,
        digital_frame_size: usize,
        stop: &AtomicBool,
    ) -> Result<()> {
        loop {
            if first.kind == ChannelKind::Logic {
                self.set(scpi, ":WAV:SOUR LA")?;
            } else {
                self.set(scpi, &format!(":WAV:SOUR CHAN{}", first.index + 1))?;
            }
            let points = if first.kind == ChannelKind::Logic {
                digital_frame_size
            } else {
                analog_frame_size
            };
            self.set(scpi, &format!(":WAV:POIN {points}"))?;
            let esr = scpi.get_int("*ESR?")?;
            if esr & 0x10 == 0 {
                return Ok(());
            }
            log::warn!("Single shot acquisition failed, retrying...");
            std::thread::sleep(Duration::from_millis(500));
            self.set(scpi, ":SING")?;
            event_wait(scpi, 'S', 'S', false, stop)?;
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
        }
    }

    /// Waits for the DS2000 to copy the next block out of its memory.
    fn block_wait(
        &self,
        scpi: &mut Scpi,
        analog_frame_size: usize,
        stop: &AtomicBool,
    ) -> Result<()> {
        if self.protocol() != Protocol::V3 {
            return Ok(());
        }
        let start = Instant::now();
        loop {
            if stop.load(Ordering::SeqCst) || start.elapsed() > Duration::from_secs(3) {
                return Ok(());
            }
            std::thread::sleep(Duration::from_millis(if analog_frame_size < 15_000 {
                100
            } else {
                1000
            }));
            // "READ,nnnn" while copying, "IDLE,nnnn" when done.
            let s = scpi.get_string(":WAV:STAT?")?;
            let len: usize = s.get(5..).and_then(|n| n.trim().parse().ok()).unwrap_or(0);
            if !s.starts_with('R') || len >= 1_000_000 {
                return Ok(());
            }
        }
    }

    /// Selects a channel on the scope and reads its vertical scaling.
    fn channel_start(
        &self,
        scpi: &mut Scpi,
        ch: &Channel,
        first_frame: bool,
        st: &State,
        vertical: &mut [Vertical],
    ) -> Result<()> {
        let protocol = self.protocol();
        match protocol {
            Protocol::V1 | Protocol::V2 => {
                if ch.kind == ChannelKind::Logic {
                    scpi.send(":WAV:DATA? DIG")?;
                } else {
                    scpi.send(&format!(":WAV:DATA? CHAN{}", ch.index + 1))?;
                }
            }
            Protocol::V3 => {
                if ch.kind == ChannelKind::Logic {
                    self.set(scpi, ":WAV:SOUR LA")?;
                } else {
                    self.set(scpi, &format!(":WAV:SOUR CHAN{}", ch.index + 1))?;
                }
                if st.data_source != DataSource::Live {
                    self.set(scpi, ":WAV:RES")?;
                    self.set(scpi, ":WAV:BEG")?;
                }
            }
            Protocol::V4 | Protocol::V5 => {
                if ch.kind == ChannelKind::Analog {
                    self.set(scpi, &format!(":WAV:SOUR CHAN{}", ch.index + 1))?;
                } else {
                    self.set(scpi, &format!(":WAV:SOUR D{}", ch.index))?;
                }
                if first_frame {
                    let mode = if st.data_source == DataSource::Live {
                        ":WAV:MODE NORM"
                    } else {
                        ":WAV:MODE RAW"
                    };
                    self.set(scpi, mode)?;
                }
                if st.data_source != DataSource::Live {
                    self.set(scpi, ":WAV:RES")?;
                }
            }
        }
        if ch.kind == ChannelKind::Analog {
            let v = &mut vertical[ch.index];
            if protocol >= Protocol::V3 {
                if first_frame {
                    v.inc = scpi.get_f32(":WAV:YINC?")?;
                    v.origin = scpi.get_f32(":WAV:YOR?")?;
                    v.reference = scpi.get_int(":WAV:YREF?")? as i32;
                }
            } else {
                v.inc = st.vdiv[ch.index] / 25.6;
                v.offset = st.vert_offset[ch.index];
            }
        }
        Ok(())
    }
}

/// Polls `:TRIG:STAT?` until it starts with `a` or `b`; with
/// `leave_first` it first waits for the status to be something else.
fn event_wait(
    scpi: &mut Scpi,
    a: char,
    b: char,
    leave_first: bool,
    stop: &AtomicBool,
) -> Result<()> {
    let status = |scpi: &mut Scpi| -> Result<char> {
        Ok(scpi.get_string(":TRIG:STAT?")?.chars().next().unwrap_or(' '))
    };
    if leave_first {
        while !stop.load(Ordering::SeqCst) {
            let c = status(scpi)?;
            if c != a && c != b {
                break;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    while !stop.load(Ordering::SeqCst) {
        let c = status(scpi)?;
        if c == a || c == b {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    Ok(())
}

/// Reads the `#NLLL...` header of a block response and returns the
/// data length.
fn read_block_header(scpi: &mut Scpi) -> Result<usize> {
    let mut head = [0u8; 2];
    read_exact(scpi, &mut head)?;
    if head[0] != b'#' || !head[1].is_ascii_digit() || head[1] == b'0' {
        return Err(Error::device(format!(
            "Received invalid data block header '{}{}'.",
            head[0] as char, head[1] as char
        )));
    }
    let digits = usize::from(head[1] - b'0');
    let mut len = vec![0u8; digits];
    read_exact(scpi, &mut len)?;
    std::str::from_utf8(&len)
        .ok()
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| Error::device("Received invalid data block length."))
}

fn read_some(scpi: &mut Scpi, buf: &mut [u8]) -> Result<usize> {
    let n = scpi.read_data(buf)?;
    if n == 0 {
        return Err(Error::Timeout);
    }
    Ok(n)
}

fn read_exact(scpi: &mut Scpi, buf: &mut [u8]) -> Result<()> {
    let mut got = 0;
    while got < buf.len() {
        got += read_some(scpi, &mut buf[got..])?;
    }
    Ok(())
}
