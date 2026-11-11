// Counterpart in sigrok:
//   libsigrok/src/hardware/hameg-hmo/api.c,
//   libsigrok/src/hardware/hameg-hmo/protocol.c,
//   libsigrok/src/hardware/hameg-hmo/protocol.h,
//   Copyright (C) 2013 poljar (Damir Jelić) <poljarinho@gmail.com>,
//   Copyright (C) 2018 Guido Trentalancia <guido@trentalancia.com>,
//   with commits by Uwe Hermann, Gerhard Sittig, Lars-Peter Clausen,
//   Bert Vermeulen, Soeren Apel, Martin Ling, Stefan Brüns, Aurelien Jacobs,
//   Daniel Elstner, Valentin Ochs, Mathias Katzer, Johannes Römer, xandinator
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Hameg and Rohde & Schwarz oscilloscopes over SCPI: the `hameg-hmo` driver.
//!
//! HMO, RTC, RTB, RTM, RTA and RTH scopes with up to 4 analog channels and 2 pods of 8
//! logic channels, on USBTMC, TCP (`tcp-raw/HOST/PORT`) or a serial port; see [`models`]
//! and [`protocol`]. The channels that capture are those on at the open, unless a client
//! says otherwise, and the difference goes to the scope at the start.
//!
//! A setting that fails after its send may have been taken. The error says so, and the
//! next look at the state reads it from the scope again. A stop that leaves an answer on
//! its way connects again before the next use, as a new connection can't receive an old
//! answer.

#![forbid(unsafe_code)]

pub mod models;
pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::scpi::Scpi;
use models::{DIGITAL_CHANNELS_PER_POD, Dialect, MAN, Model, NUM_YDIVS, TRIGGER_SLOPES, USER2};
use protocol::{BlockRead, Bounds, LogicStore, State, Verdict, Wait};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const DRIVER_CLASSES: [DeviceClass; 2] = [DeviceClass::Oscilloscope, DeviceClass::LogicAnalyzer];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Oscilloscope];
const SET_ONLY: Access = Access { get: false, set: true, list: false };
const GET_SET: Access = Access::GET_SET;
static DEV_OPTIONS: [OptionDesc; 11] = [
    OptionDesc::new(Key::LimitSamples, SET_ONLY),
    OptionDesc::new(Key::LimitFrames, GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET),
    OptionDesc::new(Key::Timebase, Access::GET_SET_LIST),
    OptionDesc::new(Key::NumHdiv, Access::GET),
    OptionDesc::new(Key::HorizTriggerpos, GET_SET),
    OptionDesc::new(Key::TriggerSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSlope, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerPattern, GET_SET),
    OptionDesc::new(Key::HighResolution, GET_SET),
    OptionDesc::new(Key::PeakDetection, GET_SET),
];
static ANALOG_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::NumVdiv, Access::GET),
    OptionDesc::new(Key::Vdiv, Access::GET_SET_LIST),
    OptionDesc::new(Key::Coupling, Access::GET_SET_LIST),
];
static DIGITAL_OPTIONS: [OptionDesc; 2] = [
    OptionDesc::new(Key::LogicThreshold, Access::GET_SET_LIST),
    OptionDesc::new(Key::LogicThresholdCustom, GET_SET),
];

/// A pause after bytes that made no block, as C's 50 ms between its tries.
const JUNK_PAUSE: Duration = Duration::from_millis(50);

/// The driver.
pub struct HamegHmoDriver;

impl Driver for HamegHmoDriver {
    fn id(&self) -> &'static str {
        "hameg-hmo"
    }

    fn long_name(&self) -> &'static str {
        "Hameg HMO"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &DRIVER_CLASSES
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
        let found = crate::scpi::scan(conn.as_deref(), serialcomm.as_deref(), |mut scpi| {
            let id = match scpi.hw_id() {
                Ok(id) => id,
                Err(e) => {
                    log::info!("Couldn't get IDN response: {e}");
                    return Ok(None);
                }
            };
            if !protocol::known_manufacturer(&id.manufacturer) {
                return Ok(None);
            }
            let Some(model) = models::find(&id.model) else {
                log::debug!("Unsupported device.");
                return Ok(None);
            };
            // The probe's connection ends with it; the device's open makes one.
            let _ = scpi.close();
            Ok(Some(HamegDevice::new(model, &id, scpi)))
        })?;
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// What a channel group is to the driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Group {
    None,
    Analog(usize),
    Digital(usize),
}

/// One scope.
pub struct HamegDevice {
    model: &'static Model,
    info: DeviceInfo,
    resource: String,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    state: Mutex<State>,
    limit_samples: Arc<AtomicU64>,
    limit_frames: Arc<AtomicU64>,
    /// The instrument while the device is closed.
    closed: Option<Scpi>,
    scpi: Arc<Mutex<Option<Scpi>>>,
    /// A setting failed after its send: the state may not be the scope's.
    doubt: AtomicBool,
    /// A capture stopped with an answer on its way.
    owed: Arc<AtomicBool>,
    capturing: Arc<AtomicBool>,
    worker: Option<Worker>,
    /// A test's short bounds.
    bounds: Option<Bounds>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl HamegDevice {
    /// A device of `model` on an instrument, which [`Device::open`] opens again.
    pub fn new(model: &'static Model, id: &crate::scpi::HwInfo, scpi: Scpi) -> HamegDevice {
        let mut channels = Vec::new();
        let mut groups = Vec::new();
        for i in 0..model.analog {
            let name = format!("CH{}", i + 1);
            channels.push(Channel::new(i, ChannelKind::Analog, name.clone()));
            groups.push(ChannelGroup::new(name, vec![i]));
        }
        let first = channels.len();
        for i in 0..model.digital {
            channels.push(Channel::new(i, ChannelKind::Logic, format!("D{i}")));
        }
        for pod in 0..model.pods() {
            let start = first + pod * DIGITAL_CHANNELS_PER_POD;
            let members = (start..start + DIGITAL_CHANNELS_PER_POD).collect();
            groups.push(ChannelGroup::new(format!("POD{}", pod + 1), members));
        }
        HamegDevice {
            model,
            resource: scpi.resource().to_owned(),
            info: DeviceInfo {
                driver: "hameg-hmo",
                vendor: id.manufacturer.clone(),
                model: id.model.clone(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            channels,
            groups,
            state: Mutex::new(State::new(model)),
            limit_samples: Arc::new(AtomicU64::new(0)),
            limit_frames: Arc::new(AtomicU64::new(0)),
            closed: Some(scpi),
            scpi: Arc::new(Mutex::new(None)),
            doubt: AtomicBool::new(false),
            owed: Arc::new(AtomicBool::new(false)),
            capturing: Arc::new(AtomicBool::new(false)),
            worker: None,
            bounds: None,
        }
    }

    /// Runs `f` on the open instrument. Refused during a capture, whose thread holds it.
    fn with_scpi<T>(&self, f: impl FnOnce(&mut Scpi) -> Result<T>) -> Result<T> {
        let mut guard = self.scpi.lock().expect("scpi lock");
        let scpi = guard.as_mut().ok_or(Error::Busy)?;
        f(scpi)
    }

    /// What is owed from before, on an open idle instrument.
    fn make_fresh(&self) -> Result<()> {
        if self.owed.load(Ordering::SeqCst) {
            self.with_scpi(|s| {
                let _ = s.close();
                s.reopen()
            })?;
            self.owed.store(false, Ordering::SeqCst);
        }
        if self.doubt.load(Ordering::SeqCst) {
            let mut state = self.state.lock().expect("state lock").clone();
            self.with_scpi(|s| protocol::read_state(s, self.model, &mut state))?;
            *self.state.lock().expect("state lock") = state;
            self.doubt.store(false, Ordering::SeqCst);
        }
        Ok(())
    }

    fn group(&self, group: Option<&str>) -> Result<Group> {
        let Some(g) = group else { return Ok(Group::None) };
        let Some(i) = self.groups.iter().position(|cg| cg.name == g) else {
            log::error!("Invalid channel group specified.");
            return Err(Error::ChannelGroup(g.to_owned()));
        };
        Ok(if i < self.model.analog {
            Group::Analog(i)
        } else {
            Group::Digital(i - self.model.analog)
        })
    }

    /// Sends a setting and waits for `*OPC?`. A failure after the send leaves the state
    /// in doubt, and says so.
    fn setting(&self, command: &str) -> Result<()> {
        self.with_scpi(|s| {
            s.send(command)?;
            s.get_opc().inspect_err(|_| {
                self.doubt.store(true, Ordering::SeqCst);
                log::warn!(
                    "'{command}' wasn't confirmed, and the scope may have taken it. \
                     Its state is read again before the next use."
                );
            })
        })
    }

    fn state(&self) -> std::sync::MutexGuard<'_, State> {
        self.state.lock().expect("state lock")
    }

    /// The commands of `logic_threshold_custom`.
    ///
    /// C's loop finds `USER2` or `MAN` in the model's table. On an RTH, `USER2` with the
    /// second dialect, it passes `(idx, 2, float_str)` to `:DIG%d:THR %s`, so the `%s`
    /// reads the 2 as a pointer and sigrok-cli dies (`api.c:500-502`). Here the
    /// value goes where that command has it.
    fn custom_threshold(&self, number: usize, value: &str) -> Option<(String, String)> {
        let dialect = self.model.dialect;
        let name = self.custom_threshold_name()?;
        let user = usize::from(name == USER2) * 2;
        Some((dialect.set_user_threshold(number, user, value), dialect.set_threshold(number, name)))
    }

    fn custom_threshold_name(&self) -> Option<&'static str> {
        self.model.thresholds.iter().copied().find(|t| *t == USER2 || *t == MAN)
    }

    /// One capture's setup, on the caller's thread as C's start is: `hmo_setup_channels`.
    fn setup_channels(&self, scpi: &mut Scpi) -> Result<()> {
        let mut state = self.state();
        let mut pod_enabled = vec![false; self.model.pods()];
        let mut changed = false;
        for ch in &self.channels {
            match ch.kind {
                ChannelKind::Analog => {
                    if ch.enabled == state.analog[ch.index].on {
                        continue;
                    }
                    scpi.send(&format!(":CHAN{}:STAT {}", ch.index + 1, u8::from(ch.enabled)))?;
                    state.analog[ch.index].on = ch.enabled;
                    changed = true;
                }
                ChannelKind::Logic => {
                    if ch.enabled {
                        pod_enabled[ch.index / DIGITAL_CHANNELS_PER_POD] = true;
                    }
                    if ch.enabled == state.digital[ch.index] {
                        continue;
                    }
                    scpi.send(&format!(":LOG{}:STAT {}", ch.index, u8::from(ch.enabled)))?;
                    state.digital[ch.index] = ch.enabled;
                    changed = true;
                }
            }
        }
        for (i, on) in pod_enabled.iter().enumerate() {
            if state.pods[i].on == *on {
                continue;
            }
            scpi.send(&self.model.dialect.set_pod_state(i + 1, *on))?;
            state.pods[i].on = *on;
            changed = true;
        }
        if changed {
            state.sample_rate = protocol::read_sample_rate(scpi)?;
        }
        Ok(())
    }

    /// The channels a capture asks for, in order: each analog one, and a pod's first.
    fn request_list(&self) -> Vec<Channel> {
        let mut added = vec![false; self.model.pods()];
        let mut list = Vec::new();
        for ch in self.channels.iter().filter(|c| c.enabled) {
            match ch.kind {
                ChannelKind::Analog => list.push(ch.clone()),
                ChannelKind::Logic => {
                    let pod = ch.index / DIGITAL_CHANNELS_PER_POD;
                    if !added[pod] {
                        added[pod] = true;
                        list.push(ch.clone());
                    }
                }
            }
        }
        list
    }

    fn end_worker(&mut self) {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
    }
}

impl Device for HamegDevice {
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
        Ok(match self.group(group)? {
            Group::None => DEV_OPTIONS.to_vec(),
            Group::Analog(_) => ANALOG_OPTIONS.to_vec(),
            Group::Digital(_) => DIGITAL_OPTIONS.to_vec(),
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let group = self.group(group)?;
        // A capture holds the instrument, and no setting can have failed during one.
        if !self.capturing.load(Ordering::SeqCst) && self.scpi.lock().expect("scpi lock").is_some()
        {
            self.make_fresh()?;
        }
        let model = self.model;
        let state = self.state();
        let analog = || match group {
            Group::None => Err(Error::ChannelGroup(String::new())),
            Group::Analog(i) => Ok(i),
            Group::Digital(_) => Err(Error::not_supported("not on a pod")),
        };
        let pod = || match group {
            Group::None => Err(Error::ChannelGroup(String::new())),
            Group::Digital(i) => Ok(i),
            Group::Analog(_) => Err(Error::not_supported("not on an analog channel")),
        };
        let rational = |(p, q): (u64, u64)| Value::Rational(p, q);
        Ok(match key {
            Key::NumHdiv => Value::I32(state.xdivs as i32),
            Key::Timebase => rational(model.timebases[state.timebase]),
            Key::NumVdiv => {
                analog()?;
                Value::I32(NUM_YDIVS)
            }
            Key::Vdiv => rational(model.vdivs[state.analog[analog()?].vdiv]),
            Key::TriggerSource => {
                Value::Str(model.trigger_sources[state.trigger_source].to_owned())
            }
            Key::TriggerSlope => Value::Str(TRIGGER_SLOPES[state.trigger_slope].to_owned()),
            Key::TriggerPattern => Value::Str(state.trigger_pattern.clone()),
            Key::HighResolution => Value::Bool(state.high_resolution),
            Key::PeakDetection => Value::Bool(state.peak_detection),
            Key::HorizTriggerpos => Value::F64(f64::from(state.horiz_triggerpos)),
            Key::Coupling => {
                Value::Str(model.coupling[state.analog[analog()?].coupling].to_owned())
            }
            Key::Samplerate => Value::U64(state.sample_rate),
            Key::LogicThreshold => {
                Value::Str(model.thresholds[state.pods[pod()?].threshold].to_owned())
            }
            Key::LogicThresholdCustom => {
                let i = pod()?;
                // In custom mode only.
                let current = model.thresholds[state.pods[i].threshold];
                if self.custom_threshold_name() != Some(current) {
                    return Err(Error::not_supported("the pod has no custom threshold on"));
                }
                Value::F64(f64::from(state.pods[i].user_threshold))
            }
            Key::LimitFrames => Value::U64(self.limit_frames.load(Ordering::SeqCst)),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    #[allow(clippy::too_many_lines)]
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let group = self.group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let model = self.model;
        let in_str = |table: &[&str]| {
            let s = value.as_str().ok_or_else(bad)?;
            table.iter().position(|t| *t == s).ok_or_else(bad)
        };
        let in_table = |table: &[(u64, u64)]| {
            let Value::Rational(p, q) = value else { return Err(bad()) };
            table.iter().position(|e| protocol::same_ratio(*e, (*p, *q))).ok_or_else(bad)
        };
        let need_group = || Error::ChannelGroup(String::new());
        match key {
            Key::LimitSamples => {
                self.limit_samples.store(value.as_u64().ok_or_else(bad)?, Ordering::SeqCst);
                return Ok(());
            }
            Key::LimitFrames => {
                self.limit_frames.store(value.as_u64().ok_or_else(bad)?, Ordering::SeqCst);
                return Ok(());
            }
            _ => {}
        }
        self.make_fresh()?;
        match key {
            Key::Vdiv => {
                if group == Group::None {
                    return Err(need_group());
                }
                let idx = in_table(model.vdivs)?;
                let Group::Analog(j) = group else { return Err(bad()) };
                let text = protocol::table_value(model.vdivs[idx]);
                self.setting(&format!(":CHAN{}:SCAL {text}", j + 1))?;
                self.state().analog[j].vdiv = idx;
            }
            Key::Timebase => {
                let idx = in_table(model.timebases)?;
                let text = protocol::table_value(model.timebases[idx]);
                self.setting(&format!(":TIM:SCAL {text}"))?;
                self.state().timebase = idx;
                let rate = self.with_scpi(protocol::read_sample_rate)?;
                self.state().sample_rate = rate;
            }
            Key::HorizTriggerpos => {
                let place = value.as_f64().ok_or_else(bad)?;
                // NaN too, which C's two comparisons let through to the scope.
                if !(0.0..=1.0).contains(&place) {
                    return Err(bad());
                }
                let (timebase, xdivs) = {
                    let state = self.state();
                    (model.timebases[state.timebase], state.xdivs)
                };
                let seconds =
                    protocol::trigger_seconds(place, timebase.0 as f64 / timebase.1 as f64, xdivs);
                self.setting(&format!(":TIM:POS {}", protocol::format_e(seconds)))?;
                self.state().horiz_triggerpos = place as f32;
            }
            Key::TriggerSource => {
                let idx = in_str(model.trigger_sources)?;
                self.setting(&format!(":TRIG:A:SOUR {}", model.trigger_sources[idx]))?;
                self.state().trigger_source = idx;
            }
            Key::TriggerSlope => {
                let idx = in_str(TRIGGER_SLOPES)?;
                self.setting(&format!(
                    ":TRIG:A:TYPE EDGE;:TRIG:A:EDGE:SLOP {}",
                    TRIGGER_SLOPES[idx]
                ))?;
                self.state().trigger_slope = idx;
            }
            Key::TriggerPattern => {
                let pattern = value.as_str().ok_or_else(bad)?;
                if pattern.is_empty() || pattern.len() > model.analog + model.digital {
                    return Err(bad());
                }
                // In full: at 20 characters C's 128 byte buffer loses the closing quote.
                self.setting(&format!(
                    ":TRIG:A:TYPE LOGIC;:TRIG:A:PATT:FUNC AND;:TRIG:A:PATT:COND \"TRUE\";\
                     :TRIG:A:PATT:MODE OFF;:TRIG:A:PATT:SOUR \"{pattern}\""
                ))?;
                self.state().trigger_pattern = protocol::keep_pattern(pattern);
            }
            Key::HighResolution | Key::PeakDetection => {
                let on = value.as_bool().ok_or_else(bad)?;
                let hires = *key == Key::HighResolution;
                let (this, other) =
                    if hires { (":ACQ:HRES", ":ACQ:PEAK") } else { (":ACQ:PEAK", ":ACQ:HRES") };
                self.setting(&format!("{this} {}", if on { "AUTO" } else { "OFF" }))?;
                // Told at once, unlike C: the scope has this one if the next fails.
                let mark = |state: &mut State, which_hires: bool, to: bool| {
                    if which_hires {
                        state.high_resolution = to;
                    } else {
                        state.peak_detection = to;
                    }
                };
                mark(&mut self.state(), hires, on);
                if on {
                    self.setting(&format!("{other} OFF"))?;
                    mark(&mut self.state(), !hires, false);
                }
            }
            Key::Coupling => {
                if group == Group::None {
                    return Err(need_group());
                }
                let idx = in_str(model.coupling)?;
                let Group::Analog(j) = group else { return Err(bad()) };
                self.setting(&format!(":CHAN{}:COUP {}", j + 1, model.coupling[idx]))?;
                self.state().analog[j].coupling = idx;
            }
            Key::LogicThreshold => {
                let j = match group {
                    Group::None => return Err(need_group()),
                    Group::Analog(_) => return Err(Error::not_supported("not on a channel")),
                    Group::Digital(j) => j,
                };
                let idx = in_str(model.thresholds)?;
                let number = model.threshold_number(j);
                self.setting(&model.dialect.set_threshold(number, model.thresholds[idx]))?;
                self.state().pods[j].threshold = idx;
            }
            Key::LogicThresholdCustom => {
                let j = match group {
                    Group::None => return Err(need_group()),
                    Group::Analog(_) => return Err(Error::not_supported("not on a channel")),
                    Group::Digital(j) => j,
                };
                let level = value.as_f64().ok_or_else(bad)?;
                // NaN too, as for the trigger's place.
                if !(-2.0..=8.0).contains(&level) {
                    return Err(bad());
                }
                let number = model.threshold_number(j);
                let text = protocol::format_e(level);
                let (first, second) = self.custom_threshold(number, &text).ok_or_else(bad)?;
                self.setting(&first)?;
                self.setting(&second)?;
                // C leaves `threshold` as it was, so a get in this session says it isn't
                // in custom mode, with the scope in it.
                let name = self.custom_threshold_name();
                let mut state = self.state();
                state.pods[j].user_threshold = level as f32;
                if let Some(idx) = model.thresholds.iter().position(|t| Some(*t) == name) {
                    state.pods[j].threshold = idx;
                }
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let group = self.group(group)?;
        let strs = |items: &[&str]| ValueList::Str(items.iter().map(|s| (*s).to_owned()).collect());
        let need = || Err(Error::ChannelGroup(String::new()));
        Ok(match key {
            Key::Coupling | Key::Vdiv | Key::LogicThreshold if group == Group::None => {
                return need();
            }
            Key::Coupling => strs(self.model.coupling),
            Key::TriggerSource => strs(self.model.trigger_sources),
            Key::TriggerSlope => strs(TRIGGER_SLOPES),
            Key::Timebase => ValueList::Rational(self.model.timebases.to_vec()),
            Key::Vdiv => ValueList::Rational(self.model.vdivs.to_vec()),
            Key::LogicThreshold => strs(self.model.thresholds),
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
            self.closed = Some(scpi);
            return Err(e);
        }
        self.doubt.store(false, Ordering::SeqCst);
        self.owed.store(false, Ordering::SeqCst);
        let mut state = State::new(self.model);
        let read = protocol::read_state(&mut scpi, self.model, &mut state);
        *self.scpi.lock().expect("scpi lock") = Some(scpi);
        read?;
        let (analog, digital): (Vec<bool>, Vec<bool>) =
            (state.analog.iter().map(|a| a.on).collect(), state.digital.clone());
        for ch in &mut self.channels {
            ch.enabled = match ch.kind {
                ChannelKind::Analog => analog[ch.index],
                ChannelKind::Logic => digital[ch.index],
            };
        }
        *self.state.lock().expect("state lock") = state;
        Ok(())
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
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        // A worker whose End is stuck on a full queue gives it up.
        self.end_worker();
        self.make_fresh().map_err(|e| match e {
            Error::Busy => Error::device("device is not open"),
            e => e,
        })?;
        let list = self.request_list();
        if list.is_empty() {
            return Err(Error::arg("No channels enabled."));
        }
        let mut pods = vec![false; self.model.pods()];
        let mut analog = vec![false; self.model.analog];
        let mut pod_count = 0;
        for ch in &list {
            match ch.kind {
                ChannelKind::Analog => analog[ch.index] = true,
                ChannelKind::Logic => {
                    let pod = ch.index / DIGITAL_CHANNELS_PER_POD;
                    pods[pod] = true;
                    pod_count = pod_count.max(pod + 1);
                }
            }
        }
        if protocol::channels_conflict(&analog, &pods) {
            log::error!("Invalid channel configuration specified!");
            return Err(Error::not_supported("an analog channel and a pod share their inputs"));
        }
        let mut scpi = self
            .scpi
            .lock()
            .expect("scpi lock")
            .take()
            .ok_or_else(|| Error::device("device is not open"))?;
        if let Err(e) = self.setup_channels(&mut scpi) {
            log::error!("Failed to setup channel configuration!");
            *self.scpi.lock().expect("scpi lock") = Some(scpi);
            return Err(e);
        }
        let (sweep, units) = {
            let state = self.state();
            (state.sweep(self.model), state.analog.iter().map(|a| a.probe_unit).collect())
        };
        let stop = Arc::new(AtomicBool::new(false));
        let capture = Capture {
            dialect: self.model.dialect,
            list,
            pod_count,
            units,
            bounds: self.bounds.unwrap_or_else(|| Bounds::for_sweep(sweep)),
            limit_samples: Arc::clone(&self.limit_samples),
            limit_frames: Arc::clone(&self.limit_frames),
            stop: Arc::clone(&stop),
            owed: Arc::clone(&self.owed),
        };
        let slot = Arc::clone(&self.scpi);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let stop_end = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("hameg-hmo acquisition".to_owned())
            .spawn(move || {
                let outcome = capture.run(&mut scpi, &sink);
                if let Err(e) = &outcome {
                    log::error!("Acquisition failed: {e}");
                }
                // Back before the End, so that a client can start when it sees it.
                *slot.lock().expect("scpi lock") = Some(scpi);
                capturing.store(false, Ordering::SeqCst);
                sink.end_while(outcome, || !stop_end.load(Ordering::SeqCst));
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        self.end_worker();
        Ok(())
    }
}

/// One acquisition on its own thread: `hmo_receive_data`, as a loop.
struct Capture {
    dialect: Dialect,
    list: Vec<Channel>,
    pod_count: usize,
    units: Vec<char>,
    bounds: Bounds,
    limit_samples: Arc<AtomicU64>,
    limit_frames: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    owed: Arc<AtomicBool>,
}

impl Capture {
    /// `hmo_request_data`. Always `LSBF`, which is what `protocol::floats` reads.
    fn request(&self, ch: &Channel) -> String {
        match ch.kind {
            ChannelKind::Analog => {
                format!(":FORM:BORD LSBF;:FORM REAL,32;:CHAN{}:DATA?", ch.index + 1)
            }
            ChannelKind::Logic => self.dialect.dig_data(ch.index / DIGITAL_CHANNELS_PER_POD + 1),
        }
    }

    fn go_on(&self) -> bool {
        !self.stop.load(Ordering::SeqCst)
    }

    /// The block asked for, with C's retries and a bound on them. `None` on a stop.
    fn block(&self, scpi: &mut Scpi, wait: &mut Wait, ch: &Channel) -> Result<Option<Vec<u8>>> {
        let go_on = || self.go_on();
        loop {
            // Ahead of the read: an answer that lies ready wins over any stall.
            let before = Instant::now();
            let junk = match protocol::read_block(scpi, &self.bounds, &go_on)? {
                BlockRead::Stopped => return Ok(None),
                BlockRead::Whole(data) => {
                    if data.is_empty() {
                        log::warn!("The scope sent an empty block for {}.", ch.name);
                    }
                    Capture::heard(wait);
                    return Ok(Some(data));
                }
                BlockRead::Short { data, announced } => {
                    log::warn!(
                        "The scope sent {} of the {announced} bytes it announced for {}.",
                        data.len(),
                        ch.name
                    );
                    Capture::heard(wait);
                    return Ok(Some(data));
                }
                BlockRead::Silent => false,
                BlockRead::Junk => true,
            };
            match wait.look(before, junk) {
                Verdict::Wait => {}
                Verdict::Warn(text) => log::warn!("{text}"),
                Verdict::Fail(text) => return Err(Error::device(text)),
            }
            if junk {
                std::thread::sleep(JUNK_PAUSE);
            }
        }
    }

    fn heard(wait: &mut Wait) {
        if wait.block() {
            log::warn!("The scope is answering again.");
        }
    }

    fn analog_packet(&self, ch: &Channel, data: &[u8]) -> Packet {
        let mut samples = protocol::floats(data);
        let limit = self.limit_samples.load(Ordering::SeqCst);
        if limit > 0 && samples.len() as u64 > limit {
            samples.truncate(limit as usize);
        }
        let (mq, unit) = if self.units[ch.index] == 'V' {
            (Mq::Voltage, Unit::Volt)
        } else {
            (Mq::Current, Unit::Ampere)
        };
        // C's "TODO: Use proper 'digits' value".
        Packet::Analog(AnalogData {
            channels: vec![ch.index],
            samples,
            mq,
            flags: MqFlags::empty(),
            unit,
            digits: 2,
        })
    }

    fn run(&self, scpi: &mut Scpi, sink: &PacketSink) -> Result<()> {
        let go_on = || self.go_on();
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let mut wait = Wait::new(self.bounds, Instant::now());
        let mut frames = 0u64;
        loop {
            let mut store = LogicStore::default();
            let mut num_samples = 0u64;
            for (i, ch) in self.list.iter().enumerate() {
                self.owed.store(true, Ordering::SeqCst);
                scpi.send(&self.request(ch))?;
                if i == 0 && !sink.send_while(Packet::FrameBegin, go_on) {
                    return Ok(());
                }
                wait.asked(Instant::now());
                let Some(data) = self.block(scpi, &mut wait, ch)? else {
                    return Ok(());
                };
                self.owed.store(false, Ordering::SeqCst);
                let limit = self.limit_samples.load(Ordering::SeqCst);
                match ch.kind {
                    ChannelKind::Analog => {
                        num_samples = (data.len() / 4) as u64;
                        if data.len() >= 4 && !sink.send_while(self.analog_packet(ch, &data), go_on)
                        {
                            return Ok(());
                        }
                    }
                    ChannelKind::Logic => {
                        num_samples = (data.len() / self.pod_count) as u64;
                        if self.pod_count == 1 {
                            let most = if limit > 0 {
                                usize::try_from(limit).unwrap_or(usize::MAX).min(data.len())
                            } else {
                                data.len()
                            };
                            if most > 0
                                && !sink.send_while(Packet::logic(1, data[..most].to_vec()), go_on)
                            {
                                return Ok(());
                            }
                        } else {
                            let group = ch.index / DIGITAL_CHANNELS_PER_POD;
                            store.queue(group, self.pod_count, &data);
                        }
                    }
                }
            }
            let limit = self.limit_samples.load(Ordering::SeqCst);
            if let Some(bytes) = store.take(self.pod_count, limit)
                && !bytes.is_empty()
                && !sink.send_while(Packet::logic(self.pod_count, bytes), go_on)
            {
                return Ok(());
            }
            if !sink.send_while(Packet::FrameEnd, go_on) {
                return Ok(());
            }
            frames += 1;
            let frame_limit = self.limit_frames.load(Ordering::SeqCst);
            if protocol::capture_over(frames, frame_limit, num_samples, limit) {
                return Ok(());
            }
        }
    }
}
