// Counterpart in sigrok:
//   libsigrok/src/hardware/siglent-sds/api.c,
//   libsigrok/src/hardware/siglent-sds/protocol.c,
//   libsigrok/src/hardware/siglent-sds/protocol.h,
//   Copyright (C) 2018 mhooijboer <marchelh@gmail.com>,
//   Copyright (C) 2016 mhooijboer <marchelh@gmail.com>,
//   Copyright (C) 2012 Martin Ling <martin-git@earth.li>,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2013 Mathias Grimmberger <mgri@zaphod.sax.de>,
//   with commits by Uwe Hermann, marchelh, Gerhard Sittig, fenugrec
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Siglent SDS oscilloscopes over SCPI: the `siglent-sds` driver.
//!
//! The settings and the order of the wire are C's. The capture isn't, where C's
//! doesn't run: stock C reads a waveform's first 363 bytes, then asks for what is left
//! of `samples - 363` and at the end for 0 bytes without end, so it has never
//! finished a waveform with samples in it (`docs/DEVICES.md`). Here a waveform is read
//! to its end, one request a channel, all samples. An E series on the screen takes a
//! single shot and `DAT2`, as libsigrok pull request 247 does on real ones.
//!
//! Page numbers are those of Siglent's "SDS1000-Series & SDS2000X & SDS2000X-E
//! Programming Guide", PG01-E02D.
//!
//! The scales and offsets that turn codes into volts are read at the open, and on an E
//! series again after each shot. On the others, as in C, a scale changed on the front
//! panel after the open scales the samples wrongly, and nothing says so.
//!
//! `conn=` takes a SCPI resource (`usbtmc`, `usbtmc/BUS.ADDR`, `tcp-raw/HOST/PORT`);
//! without it every USBTMC device is asked `*IDN?`.

#![forbid(unsafe_code)]

pub mod models;
pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::scpi::Scpi;
use models::{Model, Protocol};
use protocol::{BlockHead, Quiet, Silence, Waveform};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 2] = [DeviceClass::Oscilloscope, DeviceClass::LogicAnalyzer];
static DEV_OPTIONS: [OptionDesc; 11] = [
    OptionDesc::new(Key::Timebase, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSlope, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerLevel, Access::GET_SET),
    OptionDesc::new(Key::HorizTriggerpos, Access::GET_SET),
    OptionDesc::new(Key::NumHdiv, Access { get: true, set: false, list: true }),
    OptionDesc::new(Key::Samplerate, Access::GET),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::Averaging, Access::GET_SET),
    OptionDesc::new(Key::AvgSamples, Access::GET_SET_LIST),
];
static ANALOG_OPTIONS: [OptionDesc; 6] = [
    OptionDesc::new(Key::NumVdiv, Access::GET),
    OptionDesc::new(Key::Vdiv, Access::GET_SET_LIST),
    OptionDesc::new(Key::Timebase, Access::GET_SET_LIST),
    OptionDesc::new(Key::Coupling, Access::GET_SET_LIST),
    OptionDesc::new(Key::ProbeFactor, Access::GET_SET_LIST),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
];

/// Bytes read per call while downloading.
const READ_CHUNK: usize = 32 * 1024;
/// Digital inputs on the models that have them.
const DIGITAL_CHANNELS: usize = 16;
/// Enough of a first read to hold `ALL,#9` and its length.
const HEAD_READ: usize = 15;

/// How long things take: all of a capture's waits, so a test can shorten them.
#[derive(Clone, Copy, Debug)]
struct Timing {
    /// A scope with no sample yet may say nothing this long, or as a block's
    /// preparation needs. No manual gives a figure: see [`Silence`].
    first: Duration,
    /// With a sample, this long, with a warning at `first`.
    later: Duration,
    /// Between two looks at `:INR?`, C's 10 ms.
    poll: Duration,
    /// Saying a trigger hasn't come: not an error, and a slow timebase has seconds.
    no_trigger: Duration,
    /// C's sleeps are slept. A test without them runs in no time.
    sleeps: bool,
}

const TIMING: Timing = Timing {
    first: Duration::from_secs(30),
    later: Duration::from_secs(600),
    poll: Duration::from_millis(10),
    no_trigger: Duration::from_secs(60),
    sleeps: true,
};

/// The driver.
pub struct SiglentSdsDriver;

impl Driver for SiglentSdsDriver {
    fn id(&self) -> &'static str {
        "siglent-sds"
    }

    fn long_name(&self) -> &'static str {
        "Siglent SDS1000/SDS2000"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn usb_claims(&self) -> Vec<crate::core::UsbClaim> {
        vec![crate::scpi::usbtmc::CLAIM]
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
        let found =
            crate::scpi::scan(conn.as_deref(), serialcomm.as_deref(), |scpi| Ok(probe(scpi)))?;
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// `probe_device`: a model C knows, by the second field alone, told to answer with no
/// header and no unit.
fn probe(mut scpi: Scpi) -> Option<SiglentDevice> {
    let id = match scpi.hw_id() {
        Ok(id) => id,
        Err(e) => {
            log::info!("Couldn't get IDN response, retrying: {e}");
            // As C, whose results of these two go unread: the question says.
            let _ = scpi.close();
            let _ = scpi.reopen();
            match scpi.hw_id() {
                Ok(id) => id,
                Err(e) => {
                    log::info!("Couldn't get IDN response: {e}");
                    return None;
                }
            }
        }
    };
    let Some(model) = Model::find(&id.model) else {
        log::info!("{} {} is not a supported scope.", id.manufacturer, id.model);
        return None;
    };
    log::debug!("Setting Communication Headers to off.");
    if let Err(e) = scpi.send("CHDR OFF") {
        log::info!("Couldn't set the headers off: {e}");
        return None;
    }
    // The probe's connection ends with it; the device's open makes one.
    let _ = scpi.close();
    Some(SiglentDevice::new(model, &id, scpi))
}

/// Where a frame comes from. C reads `Screen` back for what it lists and takes as
/// `Display`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DataSource {
    Screen,
    History,
}

/// What the scope reported about itself, kept up to date by every setting made.
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
    /// As the scope spells it (`C1`) after an open, or as the list does (`CH1`) after a
    /// set, which is what a get answers, as in C.
    trigger_source: String,
    horiz_triggerpos: f32,
    trigger_slope: String,
    trigger_level: f32,
    limit_frames: u64,
    data_source: DataSource,
    averaging: bool,
    average_samples: u64,
    depth_analog: u64,
    depth_digital: u64,
    samplerate: f32,
}

/// One scope.
pub struct SiglentDevice {
    model: &'static Model,
    info: DeviceInfo,
    resource: String,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    state: State,
    timing: Timing,
    /// The instrument while the device is closed.
    closed: Option<Scpi>,
    scpi: Arc<Mutex<Option<Scpi>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
    state: Arc<Mutex<State>>,
}

/// A table's entry the same as `value` whatever its form: the parser reduces `500us`
/// to 1/2000 while the tables keep 500/1000000.
fn table_entry(table: &[(u64, u64)], value: (u64, u64)) -> Option<(u64, u64)> {
    table
        .iter()
        .copied()
        .find(|t| u128::from(t.0) * u128::from(value.1) == u128::from(value.0) * u128::from(t.1))
}

/// The first entry nearest to `value`, in C's floats.
fn nearest(table: &[(u64, u64)], value: f32) -> Option<(u64, u64)> {
    let mut best = None;
    let mut smallest = f32::INFINITY;
    for t in table {
        let diff = (value - t.0 as f32 / t.1 as f32).abs();
        if diff < smallest {
            smallest = diff;
            best = Some(*t);
        }
    }
    best
}

/// The scope's word for a trigger source of the list. C sends `LINE` or `C1` with
/// `TRSE`, and asks `C1:TRSL` after an open, but a slope after a set goes to `CH1:TRSL`
/// (`api.c:558`), which no scope is shown to have. Here it is always the scope's word.
fn scope_source(name: &str) -> &str {
    match name {
        "AC Line" => "LINE",
        "CH1" => "C1",
        "CH2" => "C2",
        "CH3" => "C3",
        "CH4" => "C4",
        "Ext" => "EX",
        "Ext /5" => "EX5",
        other => other,
    }
}

/// Reads `SANU?` and `:TDIV?`: `siglent_sds_get_dev_cfg_horizontal`.
fn read_horizontal(scpi: &mut Scpi, model: &Model, st: &mut State) -> Result<()> {
    match model.series.protocol {
        Protocol::Spo | Protocol::NonSpo => {
            let text = scpi.get_string("SANU? C1")?;
            let points = protocol::sample_points(&text)?;
            if points.is_none() {
                log::warn!("No sample points in '{text}': the rate is 0.");
            }
            st.depth_analog = points.unwrap_or(0);
        }
        Protocol::Eseries => {
            st.depth_analog = protocol::sample_points_e(&scpi.get_string("SANU? C1")?)?;
            if st.la_enabled {
                st.depth_digital = protocol::sample_points_e(&scpi.get_string("SANU? D0")?)?;
            }
        }
    }
    st.timebase = scpi.get_f32(":TDIV?")?;
    log::debug!("Current timebase: {}.", st.timebase);
    st.samplerate = protocol::sample_rate(st.depth_analog, st.timebase, model.series.num_hdiv);
    log::debug!("Current samplerate: {}.", st.samplerate);
    log::debug!("Current memory depth: {}.", st.depth_analog);
    Ok(())
}

fn read_vertical(scpi: &mut Scpi, model: &Model, st: &mut State) -> Result<()> {
    for (i, vdiv) in st.vdiv.iter_mut().enumerate().take(model.analog_channels) {
        *vdiv = scpi.get_f32(&format!("C{}:VDIV?", i + 1))?;
    }
    for (i, offset) in st.vert_offset.iter_mut().enumerate().take(model.analog_channels) {
        *offset = scpi.get_f32(&format!("C{}:OFST?", i + 1))?;
    }
    Ok(())
}

impl SiglentDevice {
    /// A device of `model` on an instrument, which [`Device::open`] opens again.
    pub fn new(model: &'static Model, id: &crate::scpi::HwInfo, scpi: Scpi) -> SiglentDevice {
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
        let n = model.analog_channels;
        SiglentDevice {
            model,
            resource: scpi.resource().to_owned(),
            info: DeviceInfo {
                driver: "siglent-sds",
                vendor: "Siglent".to_owned(),
                model: model.name.to_owned(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            channels,
            groups,
            state: State {
                analog_on: vec![false; n],
                digital_on: vec![false; DIGITAL_CHANNELS],
                la_enabled: false,
                timebase: 0.0,
                attenuation: vec![0.0; n],
                vdiv: vec![0.0; n],
                vert_offset: vec![0.0; n],
                coupling: vec![String::new(); n],
                trigger_source: String::new(),
                horiz_triggerpos: 0.0,
                trigger_slope: String::new(),
                trigger_level: 0.0,
                limit_frames: 1,
                data_source: DataSource::Screen,
                averaging: false,
                average_samples: 0,
                depth_analog: 0,
                depth_digital: 0,
                samplerate: 0.0,
            },
            timing: TIMING,
            closed: Some(scpi),
            scpi: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// Runs `f` on the open instrument.
    fn with_scpi<T>(&self, f: impl FnOnce(&mut Scpi) -> Result<T>) -> Result<T> {
        let mut guard = self.scpi.lock().expect("scpi lock");
        let scpi = guard.as_mut().ok_or(Error::Busy)?;
        f(scpi)
    }

    fn send(&self, command: &str) -> Result<()> {
        self.with_scpi(|s| s.send(command))
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

    /// Reads the scope's settings: `siglent_sds_get_dev_cfg`, in its order.
    fn read_config(&mut self) -> Result<()> {
        let model = self.model;
        let mut st = self.state.clone();
        self.with_scpi(|s| {
            for (i, on) in st.analog_on.iter_mut().enumerate().take(model.analog_channels) {
                *on = s.get_bool(&format!("C{}:TRA?", i + 1))?;
            }
            if model.has_digital {
                st.la_enabled = s.get_bool("DI:SW?")?;
                log::debug!("Logic analyzer status: {}", if st.la_enabled { "On" } else { "Off" });
                for i in 0..DIGITAL_CHANNELS {
                    st.digital_on[i] = st.la_enabled && s.get_bool(&format!("D{i}:TRA?"))?;
                }
            }
            st.timebase = s.get_f32(":TDIV?")?;
            for (i, factor) in st.attenuation.iter_mut().enumerate().take(model.analog_channels) {
                *factor = s.get_f32(&format!("C{}:ATTN?", i + 1))?;
            }
            read_vertical(s, model, &mut st)?;
            for (i, coupling) in st.coupling.iter_mut().enumerate().take(model.analog_channels) {
                *coupling = s.get_string(&format!("C{}:CPL?", i + 1))?;
            }
            let select = protocol::trigger_select(&s.get_string("TRSE?")?)?;
            st.horiz_triggerpos = select.position;
            st.trigger_slope = s.get_string(&format!("{}:TRSL?", select.source))?;
            if select.is_channel {
                st.trigger_level = s.get_f32(&format!("{}:TRLV?", select.source))?;
            }
            st.trigger_source = select.source;
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

    /// `C1:` and the rest of a setting on a channel, with the value checked.
    fn group_of(&self, group: Option<&str>) -> Result<usize> {
        self.analog_group(group)?.ok_or(Error::ChannelGroup(String::new()))
    }
}

impl Device for SiglentDevice {
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
        let need_group = || self.group_of(group);
        Ok(match key {
            Key::NumHdiv => Value::I32(self.model.series.num_hdiv),
            Key::NumVdiv => Value::I32(self.model.vdivs().len() as i32),
            Key::LimitFrames => Value::U64(st.limit_frames),
            Key::DataSource => Value::Str(
                match st.data_source {
                    DataSource::Screen => "Screen",
                    DataSource::History => "History",
                }
                .to_owned(),
            ),
            // The scope is asked each time, and a failure leaves what was known, as
            // C's dropped result does. During a capture the thread holds it.
            Key::Samplerate => {
                let mut fresh = st.clone();
                let asked = self.with_scpi(|s| read_horizontal(s, self.model, &mut fresh));
                if let Err(e) = &asked {
                    log::debug!("Couldn't read the horizontal settings: {e}");
                }
                let rate = match &self.worker {
                    Some(w) => w.state.lock().expect("state lock").samplerate,
                    None => {
                        if asked.is_ok() {
                            fresh.samplerate
                        } else {
                            st.samplerate
                        }
                    }
                };
                Value::U64(rate as u64)
            }
            Key::TriggerSource => Value::Str(st.trigger_source.clone()),
            Key::TriggerSlope => {
                if st.trigger_slope.starts_with("POS") {
                    Value::Str("r".to_owned())
                } else if st.trigger_slope.starts_with("NEG") {
                    Value::Str("f".to_owned())
                } else {
                    log::debug!("Unknown trigger slope: '{}'.", st.trigger_slope);
                    return Err(Error::not_supported("unknown trigger slope"));
                }
            }
            Key::TriggerLevel => Value::F64(f64::from(st.trigger_level)),
            Key::HorizTriggerpos => Value::F64(f64::from(st.horiz_triggerpos)),
            Key::Timebase => {
                let (p, q) = nearest(self.model.timebases(), st.timebase)
                    .ok_or_else(|| Error::not_supported("timebase"))?;
                Value::Rational(p, q)
            }
            Key::Vdiv => {
                let (p, q) = nearest(models::VDIVS, st.vdiv[need_group()?])
                    .ok_or_else(|| Error::not_supported("vdiv"))?;
                Value::Rational(p, q)
            }
            Key::Coupling => Value::Str(st.coupling[need_group()?].clone()),
            Key::ProbeFactor => Value::U64(st.attenuation[need_group()?] as u64),
            Key::Averaging => Value::Bool(st.averaging),
            Key::AvgSamples => Value::U64(st.average_samples),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// A setting has no answer, on this scope, so what goes wrong here is the send
    /// alone: the state changes after it. That the scope took it nothing says: the
    /// next open reads it, and a probe factor's read of the scales does.
    #[allow(clippy::too_many_lines)]
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitFrames => self.state.limit_frames = value.as_u64().ok_or_else(bad)?,
            Key::TriggerSlope => {
                let s = value.as_str().ok_or_else(bad)?;
                if !models::TRIGGER_SLOPES.contains(&s) {
                    return Err(bad());
                }
                let slope = if s == "r" { "POS" } else { "NEG" };
                let source = scope_source(&self.state.trigger_source).to_owned();
                self.send(&format!("{source}:TRSL {slope}"))?;
                slope.clone_into(&mut self.state.trigger_slope);
            }
            Key::HorizTriggerpos => {
                let pos = value.as_f64().ok_or_else(bad)?;
                if !(0.0..=1.0).contains(&pos) {
                    log::error!("Invalid horiz. trigger position: {pos}.");
                    return Err(Error::Arg(format!("Invalid horiz. trigger position: {pos}.")));
                }
                // The position is a fraction of the frame; the scope wants seconds. C
                // multiplies by `num_timebases` (`api.c:569`), where a frame would seem to
                // want the divisions. No source says which, and the guide has no
                // `:TIM:OFFS` at all (it has `TRDL`, p.185), so a scope likely ignores it:
                // C's number goes out.
                let seconds = -(pos - 0.5)
                    * f64::from(self.state.timebase)
                    * self.model.timebases().len() as f64;
                self.send(&format!(":TIM:OFFS {seconds:.6}"))?;
                self.state.horiz_triggerpos = pos as f32;
            }
            Key::TriggerLevel => {
                let level = value.as_f64().ok_or_else(bad)?;
                if !level.is_finite() {
                    return Err(bad());
                }
                self.send(&format!(":TRIG:EDGE:LEV {level:.3}"))?;
                self.state.trigger_level = level as f32;
            }
            Key::Timebase => {
                let Value::Rational(p, q) = value else { return Err(bad()) };
                let (p, q) = table_entry(self.model.timebases(), (*p, *q)).ok_or_else(bad)?;
                let unit = match q {
                    1 => "S",
                    1000 => "MS",
                    1_000_000 => "US",
                    _ => "NS",
                };
                self.send(&format!("TDIV {p}{unit}"))?;
                self.state.timebase = p as f32 / q as f32;
            }
            Key::TriggerSource => {
                let s = value.as_str().ok_or_else(bad)?;
                // All of them, as C's check: the list is shorter on a model with no
                // digital inputs.
                if !models::TRIGGER_SOURCES.contains(&s) {
                    return Err(bad());
                }
                self.send(&format!("TRSE EDGE,SR,{},OFF", scope_source(s)))?;
                s.clone_into(&mut self.state.trigger_source);
            }
            Key::Vdiv => {
                let i = self.group_of(group)?;
                let Value::Rational(p, q) = value else { return Err(bad()) };
                let (p, q) = table_entry(models::VDIVS, (*p, *q)).ok_or_else(bad)?;
                let unit = match q {
                    1 => "V",
                    1000 => "MV",
                    _ => "UV",
                };
                self.send(&format!("C{}:VDIV {p}{unit}", i + 1))?;
                self.state.vdiv[i] = p as f32 / q as f32;
            }
            Key::Coupling => {
                let i = self.group_of(group)?;
                let s = value.as_str().ok_or_else(bad)?;
                if !models::COUPLINGS.contains(&s) {
                    return Err(bad());
                }
                let word = s.get(..3).unwrap_or(s);
                self.send(&format!("C{}:CPL {word}", i + 1))?;
                s.clone_into(&mut self.state.coupling[i]);
            }
            Key::ProbeFactor => {
                let i = self.group_of(group)?;
                let f = value.as_u64().ok_or_else(bad)?;
                if !models::PROBE_FACTORS.contains(&f) {
                    return Err(bad());
                }
                self.send(&format!("C{}:ATTN {f}", i + 1))?;
                self.state.attenuation[i] = f as f32;
                let model = self.model;
                let mut st = self.state.clone();
                // As C, a failure here leaves the set done: the scales stay as known.
                match self.with_scpi(|s| read_vertical(s, model, &mut st)) {
                    Ok(()) => self.state = st,
                    Err(e) => log::warn!("Couldn't read the scales back: {e}"),
                }
            }
            Key::DataSource => {
                let s = value.as_str().ok_or_else(bad)?;
                self.state.data_source = match s {
                    "Display" => DataSource::Screen,
                    "History" => DataSource::History,
                    _ => {
                        log::error!("Unknown data source: '{s}'.");
                        return Err(Error::Arg(format!("Unknown data source: '{s}'.")));
                    }
                };
            }
            Key::Averaging => self.state.averaging = value.as_bool().ok_or_else(bad)?,
            Key::AvgSamples => self.state.average_samples = value.as_u64().ok_or_else(bad)?,
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
            Key::AvgSamples => ValueList::U64(models::AVERAGES.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.scpi.lock().expect("scpi lock").is_some() {
            return Ok(());
        }
        let mut scpi = if let Some(s) = self.closed.take() {
            s
        } else {
            crate::scpi::open(&crate::scpi::Resource::parse(&self.resource)?, None)?
        };
        if let Err(e) = scpi.reopen() {
            log::error!("Failed to open SCPI device: {e}.");
            self.closed = Some(scpi);
            return Err(e);
        }
        *self.scpi.lock().expect("scpi lock") = Some(scpi);
        self.read_config().inspect_err(|e| log::error!("Failed to get device config: {e}."))
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(mut scpi) = self.scpi.lock().expect("scpi lock").take() {
            let _ = scpi.close();
            self.closed = Some(scpi);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        if !self.channels.iter().any(|c| c.enabled) {
            return Err(Error::arg("No channels enabled."));
        }
        let scpi = self.scpi.lock().expect("scpi lock").take();
        let Some(scpi) = scpi else {
            return Err(Error::device("device is not open"));
        };
        let shared = Arc::new(Mutex::new(self.state.clone()));
        let stop = Arc::new(AtomicBool::new(false));
        let capture = Capture {
            model: self.model,
            channels: self.channels.clone(),
            st: self.state.clone(),
            shared: Arc::clone(&shared),
            stop: Arc::clone(&stop),
            timing: self.timing,
        };
        let slot = Arc::clone(&self.scpi);
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("siglent-sds acquisition".to_owned())
            .spawn(move || {
                let mut scpi = scpi;
                let mut capture = capture;
                let outcome = capture.run(&mut scpi, &sink);
                if let Err(e) = &outcome {
                    log::error!("Acquisition failed: {e}.");
                }
                *slot.lock().expect("scpi lock") = Some(scpi);
                sink.end_while(outcome, || !stop2.load(Ordering::SeqCst));
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle, state: shared });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
            self.state = w.state.lock().expect("state lock").clone();
        }
        Ok(())
    }
}

impl Drop for SiglentDevice {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

/// A capture ended short of its limit by a stop or a gone client: no error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Flow {
    Go,
    Stopped,
}

/// What a read that found nothing comes to.
fn quiet(silence: &mut Silence, began: Instant, what: &str) -> Result<()> {
    match silence.nothing(began) {
        Quiet::Wait => Ok(()),
        Quiet::Warn => {
            log::warn!("No answer to {what} in {} s; still waiting.", silence.bound().as_secs());
            Ok(())
        }
        Quiet::GiveUp => Err(Error::device(format!("No answer to {what}: the scope is silent."))),
    }
}

/// What ends a waveform: two line feeds, if they come. An answer that hasn't them
/// loses a read's timeout here; on USBTMC the end of the message says so at once.
fn line_feeds(scpi: &mut Scpi) {
    let mut lf = [0u8; 2];
    if let Ok(n) = scpi.read_data(&mut lf)
        && n == 1
    {
        let mut one = [0u8; 1];
        let _ = scpi.read_data(&mut one);
    }
}

/// One capture on its own thread.
struct Capture {
    model: &'static Model,
    channels: Vec<Channel>,
    st: State,
    shared: Arc<Mutex<State>>,
    stop: Arc<AtomicBool>,
    timing: Timing,
}

impl Capture {
    fn stopped(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    fn protocol(&self) -> Protocol {
        self.model.series.protocol
    }

    /// As long as `wait`, or until a stop.
    fn pause(&self, wait: Duration) {
        if !self.timing.sleeps {
            return;
        }
        let began = Instant::now();
        while !self.stopped() {
            let Some(left) = wait.checked_sub(began.elapsed()) else { return };
            if left.is_zero() {
                return;
            }
            std::thread::sleep(left.min(Duration::from_millis(20)));
        }
    }

    /// What C waits ahead of reading a block: `protocol.c:519-537`. An SPO model has
    /// 2.5 µs a point, an E series a tenth of a sweep.
    fn preparation(&self) -> Duration {
        let micros = match self.protocol() {
            Protocol::Spo | Protocol::NonSpo => self.st.depth_analog as f64 * 2.5,
            Protocol::Eseries => {
                f64::from(self.st.timebase) * f64::from(self.model.series.num_hdiv) * 100_000.0
            }
        };
        if micros.is_finite() && micros > 0.0 {
            Duration::from_secs_f64(micros / 1e6)
        } else {
            Duration::ZERO
        }
    }

    /// What C sleeps once triggered: `protocol.c:84-93`. Its comment says 85% of a
    /// sweep and the sum is a thousandth of one: as the sum.
    fn after_trigger(&self) -> Duration {
        let tb = self.st.timebase;
        if tb < 0.51 && tb > 0.99e-6 {
            Duration::from_secs_f64(
                f64::from(tb) * f64::from(self.model.series.num_hdiv) * 1000.0 / 1e6,
            )
        } else {
            Duration::ZERO
        }
    }

    /// The start of `dev_acquisition_start`: what's on and off, the setup and its order.
    fn setup(&mut self, scpi: &mut Scpi) -> Result<()> {
        let some_digital = self.channels.iter().any(|c| c.kind == ChannelKind::Logic && c.enabled);
        for ch in self.channels.clone() {
            match ch.kind {
                ChannelKind::Analog => {
                    if ch.enabled != self.st.analog_on[ch.index] {
                        let on = if ch.enabled { "ON" } else { "OFF" };
                        scpi.send(&format!("C{}:TRA {on}", ch.index + 1))?;
                        self.st.analog_on[ch.index] = ch.enabled;
                    }
                }
                ChannelKind::Logic if some_digital => {
                    if ch.enabled && !self.st.la_enabled {
                        // C sends `DI:SW?`, here, reads no answer and takes it as on
                        // (`api.c:833`): a query where its comment says turn it on, and
                        // an answer left to be read as the next one's on a line with no
                        // tags. The 2018 commit that made it replaced `DGST ON`.
                        scpi.send("DI:SW ON")?;
                        self.st.la_enabled = true;
                    }
                    let on = if ch.enabled { "ON" } else { "OFF" };
                    scpi.send(&format!("D{}:TRA {on}", ch.index))?;
                    self.st.digital_on[ch.index] = ch.enabled;
                }
                ChannelKind::Logic => {}
            }
        }
        if self.st.la_enabled && !some_digital {
            // C sends `DGST OFF` (`api.c:851`), which the guide gives for the SDS1000X
            // and SDS2000X and not for the X-E series (p.74), the only ones in C's table
            // with an analyzer: `DI:SW` is theirs (p.77). C also keeps the flag true
            // after it, and so says it again at each capture.
            scpi.send("DI:SW OFF")?;
            self.st.la_enabled = false;
        }
        match self.protocol() {
            Protocol::Spo => {
                scpi.send("WFSU SP,0,TYPE,1")?;
                if self.st.averaging {
                    let n = self.st.average_samples;
                    scpi.send(&format!("ACQW AVERAGE,{n}"))?;
                } else {
                    scpi.send("ACQW SAMPLING")?;
                }
            }
            Protocol::NonSpo => {
                scpi.send("WFSU SP,0,TYPE,1")?;
                scpi.send("ACQW SAMPLING")?;
            }
            Protocol::Eseries => {}
        }
        // C asks ahead of those and again at the header, as sigrok-cli's get. Here that
        // get is `oxrok`'s ahead of the start, which with this one makes C's wire. A
        // failure leaves the depth as known, as C drops this result.
        let mut st = self.st.clone();
        match read_horizontal(scpi, self.model, &mut st) {
            Ok(()) => self.st = st,
            Err(e) => log::warn!("Couldn't read the horizontal settings: {e}"),
        }
        self.shared.lock().expect("state lock").clone_from(&self.st);
        Ok(())
    }

    fn run(&mut self, scpi: &mut Scpi, sink: &PacketSink) -> Result<()> {
        self.setup(scpi)?;
        let stop = Arc::clone(&self.stop);
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let analog: Vec<Channel> = self
            .channels
            .iter()
            .filter(|c| c.kind == ChannelKind::Analog && c.enabled)
            .cloned()
            .collect();
        let digital: Vec<usize> = self
            .channels
            .iter()
            .filter(|c| c.kind == ChannelKind::Logic && c.enabled)
            .map(|c| c.index)
            .collect();
        let now = Instant::now();
        let mut silence =
            Silence::new(now, self.timing.first, self.timing.later, scpi.read_timeout);
        let mut limit = self.st.limit_frames;
        let mut frames = 0u64;
        loop {
            if self.stopped() {
                return Ok(());
            }
            log::debug!("Starting data capture for frameset {} of {limit}", frames + 1);
            if self.capture_start(scpi, frames, &mut limit, &mut silence)? == Flow::Stopped {
                return Ok(());
            }
            if !sink.send_while(Packet::FrameBegin, go_on) {
                return Ok(());
            }
            silence.heard(Instant::now());
            for ch in &analog {
                if self.waveform(scpi, ch, sink, &mut silence)? == Flow::Stopped {
                    return Ok(());
                }
            }
            if !digital.is_empty()
                && self.digital(scpi, &digital, sink, &mut silence)? == Flow::Stopped
            {
                return Ok(());
            }
            if !sink.send_while(Packet::FrameEnd, go_on) {
                return Ok(());
            }
            silence.heard(Instant::now());
            frames += 1;
            if frames == limit {
                return Ok(());
            }
        }
    }

    /// The answer to a question, through a silence the rule allows. It is asked once:
    /// `:INR?` clears what it reads, so a late answer is the one that matters, and on a
    /// line with no tags a second question would leave a second answer owed.
    fn ask(
        &self,
        scpi: &mut Scpi,
        question: &str,
        silence: &mut Silence,
    ) -> Result<Option<String>> {
        scpi.send(question)?;
        silence.heard(Instant::now());
        loop {
            if self.stopped() {
                return Ok(None);
            }
            let began = Instant::now();
            match scpi.get_data(None) {
                Ok(data) => {
                    silence.heard(Instant::now());
                    return Ok(Some(String::from_utf8_lossy(&data).trim_end().to_owned()));
                }
                Err(Error::Timeout) => quiet(silence, began, question)?,
                Err(e) => return Err(e),
            }
        }
    }

    /// `siglent_sds_capture_start`, and the wait for a trigger the callback does next.
    ///
    /// An E series on the screen goes as libsigrok pull request 247 does, the one code
    /// shown to finish captures on real ones: `TRMD SINGLE`, then `TRMD?` until `STOP`,
    /// then the scales and the depth read again, as a front panel may have moved them.
    /// C's `ARM` stays for the others. That a second frame arms again on either is
    /// unproven: C never finished one, and pull request 247 takes one frame.
    fn capture_start(
        &mut self,
        scpi: &mut Scpi,
        frames: u64,
        limit: &mut u64,
        silence: &mut Silence,
    ) -> Result<Flow> {
        match (self.protocol(), self.st.data_source) {
            (Protocol::NonSpo, _) => self.trigger_wait(scpi, silence),
            (Protocol::Eseries, DataSource::Screen) => self.single(scpi, silence),
            (_, DataSource::Screen) => {
                scpi.send("ARM")?;
                let Some(text) = self.ask(scpi, ":INR?", silence)? else {
                    return Ok(Flow::Stopped);
                };
                if protocol::acquired(protocol::parse_inr(&text)?) {
                    log::debug!("Device triggered.");
                    Ok(Flow::Go)
                } else {
                    self.trigger_wait(scpi, silence)
                }
            }
            (_, DataSource::History) => {
                Capture::history_start(scpi, frames, limit)?;
                Ok(Flow::Go)
            }
        }
    }

    /// One single shot on an E series, as pull request 247 has it.
    fn single(&mut self, scpi: &mut Scpi, silence: &mut Silence) -> Result<Flow> {
        scpi.send("TRMD SINGLE")?;
        let began = Instant::now();
        let mut said = false;
        loop {
            let Some(mode) = self.ask(scpi, "TRMD?", silence)? else {
                return Ok(Flow::Stopped);
            };
            if mode.trim() == "STOP" {
                break;
            }
            if !said && began.elapsed() >= self.timing.no_trigger {
                said = true;
                log::warn!("No trigger in {} s; still waiting.", began.elapsed().as_secs());
            }
            if self.timing.sleeps {
                std::thread::sleep(self.timing.poll);
            }
        }
        log::debug!("Device stopped.");
        // As pull request 247, a failure here leaves what was known.
        let mut st = self.st.clone();
        let read = read_vertical(scpi, self.model, &mut st)
            .and_then(|()| read_horizontal(scpi, self.model, &mut st));
        match read {
            Ok(()) => self.st = st,
            Err(e) => log::warn!("Couldn't read the settings again after the stop: {e}"),
        }
        self.shared.lock().expect("state lock").clone_from(&self.st);
        Ok(if self.stopped() { Flow::Stopped } else { Flow::Go })
    }

    /// `FPAR?`, its count, and `FRAM`. C reads it with no read begun, so on USBTMC it
    /// never gets past this (`protocol.c:192`).
    fn history_start(scpi: &mut Scpi, frames: u64, limit: &mut u64) -> Result<()> {
        let answer = scpi.get_data(Some("FPAR?"))?;
        let bytes = answer.get(40..44).ok_or_else(|| {
            Error::device(format!("FPAR? answered {} bytes, too few to count.", answer.len()))
        })?;
        let count = u64::from(u32::from_le_bytes(bytes.try_into().expect("four bytes")));
        if *limit > count {
            log::error!("Frame limit higher than frames in buffer of device!");
        } else if *limit == 0 {
            *limit = count;
        }
        log::debug!("Starting data capture for history frameset {} of {limit}", frames + 1);
        scpi.send(&format!("FRAM {}", frames + 1))
    }

    /// `:INR?` until bit 0 says a signal was acquired; see [`protocol::acquired`]. C takes
    /// any value but 0, so 8192 alone, a trigger ready and no waveform, for one. A
    /// trigger that doesn't come is no error, whatever the timebase, and a silence is:
    /// the scope always has this to say.
    fn trigger_wait(&self, scpi: &mut Scpi, silence: &mut Silence) -> Result<Flow> {
        let began = Instant::now();
        let mut said = false;
        loop {
            let Some(text) = self.ask(scpi, ":INR?", silence)? else {
                return Ok(Flow::Stopped);
            };
            if protocol::acquired(protocol::parse_inr(&text)?) {
                log::debug!("Device triggered.");
                self.pause(self.after_trigger());
                return Ok(if self.stopped() { Flow::Stopped } else { Flow::Go });
            }
            if !said && began.elapsed() >= self.timing.no_trigger {
                said = true;
                log::warn!("No trigger in {} s; still waiting.", began.elapsed().as_secs());
            }
            if self.timing.sleeps {
                std::thread::sleep(self.timing.poll);
            }
        }
    }

    /// The bound on a block's silence: what the scope takes to get it ready, three times
    /// over, and never less than the plain one.
    fn block_bound(&self) -> Duration {
        self.timing.first.max(self.preparation() * 3)
    }

    /// Bytes of an answer, through the silence the rule allows; `None` at a stop.
    fn some(
        &self,
        scpi: &mut Scpi,
        buf: &mut [u8],
        silence: &mut Silence,
        what: &str,
    ) -> Result<Option<usize>> {
        loop {
            if self.stopped() {
                return Ok(None);
            }
            let began = Instant::now();
            match scpi.read_data(buf) {
                Ok(0) | Err(Error::Timeout) => quiet(silence, began, what)?,
                Ok(n) => {
                    silence.heard(Instant::now());
                    return Ok(Some(n));
                }
                Err(e) if scpi.read_complete() => {
                    return Err(Error::device(format!("{what} was cut short ({e}).")));
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Starts reading an answer, once. On USBTMC this is a request and its first transfer,
    /// and C's next callback asks again after a timeout (`protocol.c:526`): a scope
    /// preparing a long record would then owe an answer for each, all but the last with
    /// the wrong tag. Here there is no second request: how long one is waited for is the
    /// transport's, `src/scpi/usbtmc.rs`. On TCP a begin asks nothing and can't time
    /// out; the reads that follow wait under the silence rule.
    fn begin(&self, scpi: &mut Scpi, what: &str) -> Result<Flow> {
        if self.stopped() {
            return Ok(Flow::Stopped);
        }
        match scpi.read_begin() {
            Ok(()) => Ok(Flow::Go),
            Err(Error::Timeout) => {
                Err(Error::device(format!("No answer to {what}: the scope is silent.")))
            }
            Err(e) => Err(e),
        }
    }

    /// The head of a block: `Some` with what it says and what was read past it.
    fn head(
        &self,
        scpi: &mut Scpi,
        silence: &mut Silence,
        what: &str,
    ) -> Result<Option<(usize, Vec<u8>)>> {
        let mut bytes = Vec::with_capacity(HEAD_READ);
        let mut buf = [0u8; HEAD_READ];
        let mut want = HEAD_READ;
        loop {
            if let BlockHead::Whole { taken, length } = protocol::block_head(&bytes)? {
                return Ok(Some((length, bytes.split_off(taken))));
            }
            let Some(n) = self.some(scpi, &mut buf[..want], silence, what)? else {
                return Ok(None);
            };
            bytes.extend_from_slice(&buf[..n]);
            want = 1;
        }
    }

    /// `n` bytes into a buffer that holds what came already.
    fn fill(
        &self,
        scpi: &mut Scpi,
        bytes: &mut Vec<u8>,
        n: usize,
        silence: &mut Silence,
        what: &str,
    ) -> Result<Flow> {
        let mut buf = [0u8; 256];
        while bytes.len() < n {
            let want = (n - bytes.len()).min(buf.len());
            let Some(got) = self.some(scpi, &mut buf[..want], silence, what)? else {
                return Ok(Flow::Stopped);
            };
            bytes.extend_from_slice(&buf[..got]);
        }
        Ok(Flow::Go)
    }

    /// One analog channel's waveform: asked once, read to its end, in packets.
    fn waveform(
        &self,
        scpi: &mut Scpi,
        ch: &Channel,
        sink: &PacketSink,
        silence: &mut Silence,
    ) -> Result<Flow> {
        log::debug!("Start reading data from channel {}.", ch.name);
        // An E series in a single shot takes the waveform alone, `DAT2`, whose length
        // is its samples (p.264-265), as pull request 247 does; C's `ALL` has a LeCroy
        // descriptor ahead of them.
        let single =
            self.protocol() == Protocol::Eseries && self.st.data_source == DataSource::Screen;
        let what = format!("C{}:WF? {}", ch.index + 1, if single { "DAT2" } else { "ALL" });
        scpi.send(&what)?;
        silence.heard(Instant::now());
        silence.set_first(self.block_bound());
        self.pause(self.preparation());
        if self.begin(scpi, &what)? == Flow::Stopped {
            return Ok(Flow::Stopped);
        }
        // The head's first read takes no more than the head and a part of what follows,
        // so with [`protocol::DESC_HEAD`] bytes of a descriptor no sample has been read.
        let Some((length, mut descriptor)) = self.head(scpi, silence, &what)? else {
            return Ok(Flow::Stopped);
        };
        let mut early = Vec::new();
        let shape = if single {
            std::mem::swap(&mut early, &mut descriptor);
            Waveform { descriptor: 0, samples: length, disagrees: false }
        } else {
            if self.fill(scpi, &mut descriptor, protocol::DESC_HEAD, silence, &what)?
                == Flow::Stopped
            {
                return Ok(Flow::Stopped);
            }
            Waveform::new(length, &descriptor)?
        };
        if shape.disagrees {
            log::warn!(
                "A waveform's lengths disagree: {length} bytes, with a descriptor of {}. \
                 Taking {} samples.",
                shape.descriptor,
                shape.samples
            );
        }
        // What the descriptor has beyond the two fields, which nothing here reads.
        let mut skip = shape.descriptor.saturating_sub(descriptor.len());
        let mut buf = vec![0u8; READ_CHUNK];
        while skip > 0 {
            let want = skip.min(buf.len());
            let Some(n) = self.some(scpi, &mut buf[..want], silence, &what)? else {
                return Ok(Flow::Stopped);
            };
            skip -= n;
        }
        let vdiv = self.st.vdiv[ch.index];
        let offset = self.st.vert_offset[ch.index];
        let digits = protocol::digits(vdiv);
        let send = |bytes: &[u8]| {
            let samples: Vec<f32> =
                bytes.iter().map(|b| protocol::volts(*b, vdiv, offset)).collect();
            sink.send_while(
                Packet::Analog(AnalogData {
                    channels: vec![ch.index],
                    samples,
                    mq: Mq::Voltage,
                    flags: MqFlags::empty(),
                    unit: Unit::Volt,
                    digits,
                }),
                || !self.stopped(),
            )
        };
        let mut left = shape.samples;
        // On `DAT2` the head's first read may hold samples.
        early.truncate(left);
        if !early.is_empty() {
            if !send(&early) {
                return Ok(Flow::Stopped);
            }
            left -= early.len();
            silence.sampled(Instant::now());
        }
        while left > 0 {
            let want = left.min(buf.len());
            let got = match self.some(scpi, &mut buf[..want], silence, &what) {
                Ok(Some(n)) => n,
                Ok(None) => return Ok(Flow::Stopped),
                Err(e) => {
                    log::error!(
                        "Read error, aborting capture: {left} of {} samples missing.",
                        shape.samples
                    );
                    return Err(e);
                }
            };
            if !send(&buf[..got]) {
                return Ok(Flow::Stopped);
            }
            left -= got;
            silence.sampled(Instant::now());
        }
        log::debug!("Transfer has been completed.");
        line_feeds(scpi);
        Ok(Flow::Go)
    }

    /// All enabled logic channels at once, as `siglent_sds_get_digital`: one packet
    /// of 16 bit samples. Unlike C's, it comes back for the next frame.
    ///
    /// `#9` gives points, a bit each, low bit first: 700 points come as 88 bytes (p.268).
    /// C reads the bytes `SANU? D0` counts, of what holds an eighth of them.
    fn digital(
        &self,
        scpi: &mut Scpi,
        channels: &[usize],
        sink: &PacketSink,
        silence: &mut Silence,
    ) -> Result<Flow> {
        let mut samples = Vec::new();
        let mut buf = vec![0u8; READ_CHUNK];
        for index in channels {
            let what = format!("D{index}:WF? DAT2");
            scpi.send(&what)?;
            silence.heard(Instant::now());
            silence.set_first(self.block_bound());
            self.pause(self.preparation());
            if self.begin(scpi, &what)? == Flow::Stopped {
                return Ok(Flow::Stopped);
            }
            let Some((points, mut data)) = self.head(scpi, silence, &what)? else {
                return Ok(Flow::Stopped);
            };
            let length = points.div_ceil(8);
            let mut left = length.saturating_sub(data.len());
            data.truncate(length);
            while left > 0 {
                let want = left.min(buf.len());
                let Some(n) = self.some(scpi, &mut buf[..want], silence, &what)? else {
                    return Ok(Flow::Stopped);
                };
                data.extend_from_slice(&buf[..n]);
                left -= n;
            }
            silence.sampled(Instant::now());
            line_feeds(scpi);
            let count = points;
            if self.st.depth_digital != 0 && self.st.depth_digital != count as u64 {
                log::debug!("{what} gave {count} points; SANU? D0 said {}.", self.st.depth_digital);
            }
            if samples.len() < count {
                samples.resize(count, 0u16);
            }
            protocol::merge_digital(&mut samples, &data, *index);
        }
        let bytes = samples.iter().flat_map(|s| s.to_le_bytes()).collect();
        if sink.send_while(Packet::logic(2, bytes), || !self.stopped()) {
            silence.heard(Instant::now());
            Ok(Flow::Go)
        } else {
            Ok(Flow::Stopped)
        }
    }
}

#[cfg(test)]
mod tests;
