// Counterpart in sigrok:
//   libsigrok/src/hardware/yokogawa-dlm/api.c,
//   libsigrok/src/hardware/yokogawa-dlm/protocol.c,
//   libsigrok/src/hardware/yokogawa-dlm/protocol_wrappers.c,
//   Copyright (C) 2014 abraxa (Soeren Apel) <soeren@apelpie.net>,
//   with commits by Uwe Hermann, Soeren Apel, Lars-Peter Clausen,
//   Bert Vermeulen, Martin Ling, Gerhard Sittig, Aurelien Jacobs,
//   Daniel Elstner, Stefan Brüns
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Yokogawa DL and DLM oscilloscopes over SCPI: the `yokogawa-dlm` driver.
//!
//! `conn=` takes a SCPI resource (`usbtmc`, `usbtmc/BUS.ADDR`, `tcp-raw/HOST/PORT`, a
//! serial port); without it every USBTMC device is asked `*IDN?`. C has VXI too, which
//! is how a DLM's Ethernet speaks, and oxrok has no VXI, so on a real DLM2000 only USBTMC
//! is known to reach it. See [`protocol`] for the commands.
//!
//! A scan asks `*IDN?`, turns response headers off and closes. An open reads every
//! setting. A capture reads what the scope holds, once, channel by channel, and ends:
//! one frame, whatever `--frames` asks, as in C, with a warning here. Setting a channel
//! on or off goes to the scope at once, and may turn its logic pod on or off with it.
//!
//! Where this leaves `api.c`, `protocol.c` and `protocol_wrappers.c`:
//!
//! - A capture sends a header. C's start has none (`api.c:522-559`), so sigrok-cli skips
//!   every packet (`session.c:286`), and a stock capture prints nothing and exits 0.
//! - An empty block no longer crashes: `protocol.c:1074` frees the buffer and keeps the
//!   pointer, and the next read writes to it. Empty blocks go on through `:STOP` for 10
//!   s at most, a frame begins once, and the requests aren't flat out.
//! - A failed `:WAVEFORM:RANGE?` or `:WAVEFORM:OFFSET?` fails the open. C's `result &=`
//!   turns any error into `SR_OK`, as `SR_OK` is 0 (`protocol_wrappers.c:197`, `:209`),
//!   and goes on with a range of 0: every sample is the offset, with exit 0.
//! - An answer is no number when the manual's forms aren't there: see
//!   [`protocol::number`], [`protocol::length`], [`protocol::scaled_index`] and
//!   [`protocol::block_head`], each with what C does instead. An out of range length
//!   wraps in C, and a bad one writes what it never read to the state.
//! - A block whose header says less than the record is cut short, as is one with fewer
//!   bytes than its header says. C doesn't read the count, so a line end can be its last
//!   sample.
//! - The models with no pods are found. `g_malloc0(0)` is `NULL`, and `dlm_device_init`
//!   takes that for no memory (`protocol.c:782`), so stock C finds no DLM2022, DLM2032,
//!   DLM2052 or DL9040 to DL9240L. What it would do with them no run shows.
//! - A block's bytes on `tcp-raw` are read up to what its header says, and to 5 s of
//!   silence for what is missing. C takes each short read as the end, so an answer the
//!   stack cuts is "Truncated". A scope that goes silent ends a capture
//!   ([`protocol::Bounds`]) where C waits for ever, on either carrier.
//! - A request that can't be sent fails the capture. C drops the result for the first
//!   (`api.c:553`) and for an empty block's retry, and then waits for ever with nothing
//!   pending. A gone USB device likewise, where C tries `read_begin` for ever.
//! - A setting changes the state only once sent and acknowledged, where C sets it first,
//!   and says that it may have taken effect if `*OPC?` didn't answer.
//! - `nan` is no trigger position, where it passes two comparisons in C
//!   (`api.c:325`) and goes out as `NAN`.
//! - A capture stops between two reads; a set during one is refused with `Busy`. What
//!   sigrok-cli's `strtof` narrows stays a double here.
//! - Logic channels have the bit of the byte they read as their index, where C numbers
//!   them from 32 (`DLM_DIG_CHAN_INDEX_OFFS`), so a `bits` or `hex` output there reads
//!   four samples ahead, and past the end of the buffer on the last ones. The 16 and 32
//!   channel models still get a byte per sample, as in C.
//!
//! Channel 4 with logic is refused at a capture's start, as C's `dlm_check_channels`
//! does (`api.c:458-484`, `:520-521`), on every model. Its comment says CH4 and logic
//! share their inputs on a DLM2000, and the sigrok wiki's table has those as "4 analog
//! / 3 analog+8 digital". For the DL9000 C has a TODO and no source says either way.
//!
//! Setting a channel on or off with `-C` changes the scope's own display, and its logic
//! pod, at once, as C does, and it stays so after the capture.
//!
//! Kept as in C, unproven against a scope: a trigger position past the middle sends a
//! negative delay, which the manual's range doesn't have; `:LOGIC:PODA:BITn` and
//! `:LOGIC:MODE` for the DL9000's pods; channel 4 refused with logic on the DL9000.

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::scpi::Scpi;
use protocol::{Bounds, Config, Empties, Head, Header, Quiet, Silence};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 2] = [DeviceClass::LogicAnalyzer, DeviceClass::Oscilloscope];
static DEV_OPTIONS: [OptionDesc; 7] = [
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET),
    OptionDesc::new(Key::Timebase, Access::GET_SET_LIST),
    OptionDesc::new(Key::NumHdiv, Access::GET),
    OptionDesc::new(Key::HorizTriggerpos, Access::GET_SET),
    OptionDesc::new(Key::TriggerSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSlope, Access::GET_SET_LIST),
];
static ANALOG_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::NumVdiv, Access::GET),
    OptionDesc::new(Key::Vdiv, Access::GET_SET_LIST),
    OptionDesc::new(Key::Coupling, Access::GET_SET_LIST),
];

/// `RECEIVE_BUFFER_SIZE`: on `tcp-raw` a read's size says where an answer ends.
const RECEIVE_BUFFER_SIZE: usize = 4096;
/// Samples per packet: what a frame's conversion holds at once.
const PACKET_SAMPLES: usize = 1 << 20;
/// What an answer may bring past all it is known to need, or with no header at all,
/// before it counts as no answer. Bytes that flow must not hold a capture for ever.
const SURPLUS: usize = 1 << 20;
/// A failed `read_begin` that took no time: as often as `dlm_data_receive`'s source.
const BEGIN_PAUSE: Duration = Duration::from_millis(5);

const STOP_CMD: &str = ":STOP";

/// The driver.
pub struct YokogawaDlmDriver;

fn probe(mut scpi: Scpi) -> Option<YokogawaDevice> {
    let id = match scpi.hw_id() {
        Ok(id) => id,
        Err(e) => {
            log::info!("Couldn't get IDN response: {e}");
            return None;
        }
    };
    if id.manufacturer != "YOKOGAWA" {
        return None;
    }
    let Some((config, name)) = protocol::model_get(&id.model) else {
        log::error!("Found unsupported DLM device with model identifier {}.", id.model);
        return None;
    };
    // Disable non-standard response behavior.
    if let Err(e) = scpi.send(":COMMUNICATE:HEADER OFF") {
        log::debug!("Couldn't turn response headers off: {e}.");
        return None;
    }
    // The probe's connection ends with it; the device's open makes one. A close that
    // fails loses no device, as in C.
    let _ = scpi.close();
    Some(YokogawaDevice::new(config, name, &id, scpi))
}

impl Driver for YokogawaDlmDriver {
    fn id(&self) -> &'static str {
        "yokogawa-dlm"
    }

    fn long_name(&self) -> &'static str {
        "Yokogawa DL/DLM"
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
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let found = crate::scpi::scan(conn.as_deref(), None, |scpi| Ok(probe(scpi)))?;
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// What an open reads of an analog channel.
#[derive(Clone, Copy, Debug, Default)]
struct AnalogState {
    on: bool,
    vdiv: usize,
    position: f32,
    range: f32,
    offset: f32,
    coupling: usize,
}

/// `struct scope_state`.
#[derive(Clone, Debug, Default)]
struct State {
    analog: Vec<AnalogState>,
    digital: Vec<bool>,
    pods: Vec<bool>,
    timebase: usize,
    horiz_triggerpos: f32,
    source: usize,
    slope: usize,
    sample_rate: u64,
    samples_per_frame: u32,
}

/// Which kind of group an option is asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Group {
    None,
    Analog(usize),
    Digital,
}

/// One scope.
pub struct YokogawaDevice {
    config: &'static Config,
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    state: State,
    frame_limit: u64,
    /// The instrument while the device is closed.
    closed: Option<Scpi>,
    scpi: Arc<Mutex<Option<Scpi>>>,
    worker: Option<Worker>,
    bounds: Bounds,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// A number from the instrument: see [`protocol::number`].
fn get_number(scpi: &mut Scpi, command: &str) -> Result<f32> {
    let text = scpi.get_string(command)?;
    protocol::number(&text)
        .ok_or_else(|| Error::device(format!("{command}: not a number: '{text}'")))
}

/// `dlm_sample_rate_query`.
fn sample_rate_query(scpi: &mut Scpi) -> Result<u64> {
    scpi.send(":WAVEFORM:TRACE 1")?;
    scpi.send(":WAVEFORM:RECORD 0")?;
    let value = get_number(scpi, ":WAVEFORM:SRATE?")?;
    protocol::rate(value)
        .ok_or_else(|| Error::device(format!(":WAVEFORM:SRATE?: not a rate: {value}")))
}

/// `dlm_scope_state_query`.
fn scope_state_query(scpi: &mut Scpi, config: &Config, state: &mut State) -> Result<()> {
    let tables = |text: &str, what: &str, table: &[(u64, u64)]| {
        protocol::scaled_index(text, table)
            .ok_or_else(|| Error::device(format!("{what}: no such value: '{text}'")))
    };
    for (i, slot) in state.analog.iter_mut().enumerate().take(config.analog) {
        let n = i + 1;
        let mut ch = AnalogState { on: scpi.get_bool(&format!(":CHANNEL{n}:DISPLAY?"))?, ..*slot };
        let text = scpi.get_string(&format!(":CHANNEL{n}:VDIV?"))?;
        ch.vdiv = tables(&text, "vdiv", &protocol::VDIVS)?;
        ch.position = get_number(scpi, &format!(":CHANNEL{n}:POSITION?"))?;
        scpi.send(&format!(":WAVEFORM:TRACE {n}"))?;
        ch.range = get_number(scpi, ":WAVEFORM:RANGE?")?;
        scpi.send(&format!(":WAVEFORM:TRACE {n}"))?;
        ch.offset = get_number(scpi, ":WAVEFORM:OFFSET?")?;
        let text = scpi.get_string(&format!(":CHANNEL{n}:COUPLING?"))?;
        ch.coupling = protocol::COUPLINGS
            .iter()
            .position(|c| *c == text)
            .ok_or_else(|| Error::device(format!("coupling: no such value: '{text}'")))?;
        *slot = ch;
    }
    if config.digital == 0 {
        log::warn!("Tried obtaining digital channel states on a model without digital inputs.");
    } else {
        for (i, on) in state.digital.iter_mut().enumerate().take(config.digital) {
            *on = scpi.get_bool(&format!(":LOGIC:PODA:BIT{}:DISPLAY?", i + 1))?;
        }
        for pod in &mut state.pods {
            // The pod is ignored in C: the DLM2000 has pod A alone.
            *pod = scpi.get_bool(":LOGIC:MODE?")?;
        }
    }
    let text = scpi.get_string(":TIMEBASE:TDIV?")?;
    state.timebase = tables(&text, "timebase", &protocol::TIMEBASES)?;
    let delay = get_number(scpi, ":TRIGGER:DELAY:TIME?")?;
    state.horiz_triggerpos =
        protocol::position_for_delay(delay, protocol::TIMEBASES[state.timebase], config.xdivs);
    let text = scpi.get_string(":TRIGGER:ATRIGGER:SIMPLE:SOURCE?")?;
    state.source = config
        .sources
        .iter()
        .position(|s| *s == text)
        .ok_or_else(|| Error::device(format!("trigger source: no such value: '{text}'")))?;
    let text = scpi.get_string(":TRIGGER:ATRIGGER:SIMPLE:SLOPE?")?;
    state.slope = match text.as_str() {
        "RISE" => 0,
        "FALL" => 1,
        _ => return Err(Error::device(format!("trigger slope: no such value: '{text}'"))),
    };
    let text = scpi.get_string(":WAVEFORM:LENGTH?").inspect_err(|_| {
        log::error!("Failed to query acquisition length.");
    })?;
    state.samples_per_frame = protocol::length(&text).ok_or_else(|| {
        log::error!("Failed to query acquisition length.");
        Error::device(format!(":WAVEFORM:LENGTH?: not a length: '{text}'"))
    })?;
    // Its result is dropped in C too: the rate stays what it was.
    match sample_rate_query(scpi) {
        Ok(rate) => state.sample_rate = rate,
        Err(e) => log::debug!("No sample rate: {e}."),
    }
    log::info!(
        "Current samplerate: {} Hz, samples per acquisition (i.e. frame): {}, \
         trigger: {} (source), {} (slope) {:.2} (offset)",
        state.sample_rate,
        state.samples_per_frame,
        config.sources[state.source],
        protocol::SLOPES[state.slope],
        state.horiz_triggerpos,
    );
    Ok(())
}

impl YokogawaDevice {
    /// A device of `config` on an instrument, which [`Device::open`] opens again.
    pub fn new(
        config: &'static Config,
        name: &str,
        id: &crate::scpi::HwInfo,
        scpi: Scpi,
    ) -> YokogawaDevice {
        let mut channels = Vec::new();
        let mut groups = Vec::new();
        for i in 0..config.analog {
            let name = (i + 1).to_string();
            channels.push(Channel::new(i, ChannelKind::Analog, name.clone()));
            groups.push(ChannelGroup::new(name, vec![i]));
        }
        for pod in 0..config.pods {
            let in_pod = (0..config.digital).filter(|i| i / 8 == pod).map(|i| config.analog + i);
            groups.push(ChannelGroup::new(format!("POD{pod}"), in_pod.collect()));
        }
        for (i, name) in config.digital_names.iter().enumerate().take(config.digital) {
            channels.push(Channel::new(i, ChannelKind::Logic, *name));
        }
        YokogawaDevice {
            config,
            info: DeviceInfo {
                driver: "yokogawa-dlm",
                vendor: "Yokogawa".to_owned(),
                model: name.to_owned(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            channels,
            groups,
            state: State {
                analog: vec![AnalogState::default(); config.analog],
                digital: vec![false; config.digital],
                pods: vec![false; config.pods],
                ..State::default()
            },
            frame_limit: 0,
            closed: Some(scpi),
            scpi: Arc::new(Mutex::new(None)),
            worker: None,
            bounds: Bounds::default(),
        }
    }

    /// Runs `f` on the open instrument.
    fn with_scpi<T>(&self, f: impl FnOnce(&mut Scpi) -> Result<T>) -> Result<T> {
        let mut guard = self.scpi.lock().expect("scpi lock");
        let scpi = guard.as_mut().ok_or(Error::Busy)?;
        f(scpi)
    }

    /// A setting and the `*OPC?`s that follow it. Nothing is known of the state past a
    /// failed one: a setting with no acknowledgement may have taken effect.
    fn setting(&self, command: &str, opcs: u32) -> Result<()> {
        self.with_scpi(|scpi| {
            scpi.send(command)?;
            for _ in 0..opcs {
                scpi.get_opc().inspect_err(|e| {
                    log::warn!(
                        "No acknowledgement of {command}, which may have taken effect: {e}."
                    );
                })?;
            }
            Ok(())
        })
    }

    fn group(&self, group: Option<&str>) -> Result<Group> {
        let Some(name) = group else { return Ok(Group::None) };
        let at = self
            .groups
            .iter()
            .position(|g| g.name == name)
            .ok_or_else(|| Error::ChannelGroup(name.to_owned()))?;
        Ok(if at < self.config.analog { Group::Analog(at) } else { Group::Digital })
    }

    /// `dlm_channel_state_set`, the state in step with what the scope took.
    fn channel_state_set(&mut self, pos: usize, on: bool) -> Result<()> {
        let first_logic = self.config.analog;
        let mut wanted_pods = vec![false; self.config.pods];
        let mut result = Ok(());
        let word = if on { "ON" } else { "OFF" };
        for at in 0..self.channels.len() {
            let target = at == pos;
            if at < first_logic {
                if target {
                    let cmd = format!(":CHANNEL{}:DISPLAY {word}", at + 1);
                    match self.with_scpi(|s| s.send(&cmd)) {
                        Ok(()) => self.state.analog[at].on = on,
                        Err(e) => result = Err(e),
                    }
                }
                continue;
            }
            let i = at - first_logic;
            if target {
                let cmd = format!(":LOGIC:PODA:BIT{}:DISPLAY {word}", i + 1);
                match self.with_scpi(|s| s.send(&cmd)) {
                    Ok(()) => {
                        self.state.digital[i] = on;
                        wanted_pods[i / 8] |= on;
                    }
                    Err(e) => result = Err(e),
                }
            } else {
                wanted_pods[i / 8] |= self.channels[at].enabled;
            }
        }
        for (i, wanted) in wanted_pods.iter().enumerate() {
            if self.state.pods[i] == *wanted {
                continue;
            }
            let cmd = if *wanted { ":LOGIC:MODE ON" } else { ":LOGIC:MODE OFF" };
            match self.with_scpi(|s| s.send(cmd)) {
                Ok(()) => self.state.pods[i] = *wanted,
                Err(e) => {
                    result = Err(e);
                    break;
                }
            }
        }
        result
    }

    /// The channels a capture reads: each analog one, and one for all logic ones.
    fn capture_channels(&self) -> Vec<Channel> {
        let mut digital_added = false;
        let mut list = Vec::new();
        for ch in self.channels.iter().filter(|c| c.enabled) {
            if ch.kind != ChannelKind::Logic || !digital_added {
                list.push(ch.clone());
                digital_added |= ch.kind == ChannelKind::Logic;
            }
        }
        list
    }

    fn capture_refusal(&self) -> Result<Vec<Channel>> {
        let list = self.capture_channels();
        if list.is_empty() {
            return Err(Error::arg("No channels enabled."));
        }
        let chan4 = list.iter().any(|c| c.kind == ChannelKind::Analog && c.index == 3);
        let logic = list.iter().any(|c| c.kind == ChannelKind::Logic);
        if protocol::channels_conflict(chan4, logic) {
            log::error!("Invalid channel configuration specified!");
            return Err(Error::not_supported("Invalid channel configuration specified!"));
        }
        Ok(list)
    }

    /// Ends a capture that is over or running, and takes back the instrument.
    fn end_worker(&mut self) {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
    }
}

impl Device for YokogawaDevice {
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

    /// `sr_dev_channel_enable` and `config_channel_set`: it goes to the scope when the
    /// state changes. With the device closed it is only noted, and an open reads over it.
    fn enable_channel(&mut self, pos: usize, enabled: bool) -> Result<()> {
        let was = self.channels.get(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?.enabled;
        if self.closed.is_some() {
            self.channels[pos].enabled = enabled;
            return Ok(());
        }
        if was == enabled {
            return Ok(());
        }
        self.channels[pos].enabled = enabled;
        let result = self.channel_state_set(pos, enabled);
        if result.is_err() {
            self.channels[pos].enabled = was;
        }
        result
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
            Group::Digital => Vec::new(),
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let st = &self.state;
        let group = self.group(group)?;
        let analog = || match group {
            Group::None => Err(Error::ChannelGroup(String::new())),
            Group::Analog(i) => Ok(i),
            Group::Digital => Err(Error::not_supported("no such option on a pod")),
        };
        Ok(match key {
            Key::NumHdiv => Value::I32(self.config.xdivs),
            Key::Timebase => {
                let (p, q) = protocol::TIMEBASES[st.timebase];
                Value::Rational(p, q)
            }
            Key::NumVdiv => {
                analog()?;
                Value::I32(self.config.ydivs)
            }
            Key::Vdiv => {
                let (p, q) = protocol::VDIVS[st.analog[analog()?].vdiv];
                Value::Rational(p, q)
            }
            Key::TriggerSource => Value::Str(self.config.sources[st.source].to_owned()),
            Key::TriggerSlope => Value::Str(protocol::SLOPES[st.slope].to_owned()),
            Key::HorizTriggerpos => Value::F64(f64::from(st.horiz_triggerpos)),
            Key::Coupling => {
                Value::Str(protocol::COUPLINGS[st.analog[analog()?].coupling].to_owned())
            }
            Key::Samplerate => Value::U64(st.sample_rate),
            other => return Err(Error::not_supported(format!("{} can't be read", other.id()))),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let group = self.group(group)?;
        let analog = |g: Group| match g {
            Group::None => Err(Error::ChannelGroup(String::new())),
            Group::Analog(i) => Ok(i),
            Group::Digital => Err(Error::arg("no such option on a pod")),
        };
        let table_entry = |table: &[(u64, u64)]| {
            let Value::Rational(p, q) = value else { return Err(bad()) };
            table
                .iter()
                .position(|e| u128::from(e.0) * u128::from(*q) == u128::from(*p) * u128::from(e.1))
                .ok_or_else(bad)
        };
        match key {
            Key::LimitFrames => {
                self.frame_limit = value.as_u64().ok_or_else(bad)?;
                self.with_scpi(Scpi::get_opc)?;
            }
            Key::TriggerSource => {
                let text = value.as_str().ok_or_else(bad)?;
                let idx = self.config.sources.iter().position(|s| *s == text).ok_or_else(bad)?;
                self.setting(&format!(":TRIGGER:ATRIGGER:SIMPLE:SOURCE {text}"), 1)?;
                self.state.source = idx;
            }
            Key::Vdiv => {
                let i = analog(group)?;
                let idx = table_entry(&protocol::VDIVS)?;
                let text = protocol::table_text(protocol::VDIVS[idx]);
                self.setting(&format!(":CHANNEL{}:VDIV {text}", i + 1), 2)?;
                self.state.analog[i].vdiv = idx;
            }
            Key::Timebase => {
                let idx = table_entry(&protocol::TIMEBASES)?;
                let text = protocol::table_text(protocol::TIMEBASES[idx]);
                self.setting(&format!(":TIMEBASE:TDIV {text}"), 1)?;
                self.state.timebase = idx;
                self.state.sample_rate = self.with_scpi(sample_rate_query)?;
            }
            Key::HorizTriggerpos => {
                let position = value.as_f64().ok_or_else(bad)?;
                if !(0.0..=1.0).contains(&position) {
                    return Err(Error::Arg(format!("Invalid horiz. trigger position: {position}")));
                }
                let delay = protocol::delay_for_position(
                    position,
                    protocol::TIMEBASES[self.state.timebase],
                    self.config.xdivs,
                );
                self.setting(&format!(":TRIGGER:DELAY:TIME {}", protocol::format_e(delay)), 1)?;
                self.state.horiz_triggerpos = position as f32;
            }
            Key::TriggerSlope => {
                let text = value.as_str().ok_or_else(bad)?;
                let idx = protocol::SLOPES.iter().position(|s| *s == text).ok_or_else(bad)?;
                let word = if idx == 0 { "RISE" } else { "FALL" };
                self.setting(&format!(":TRIGGER:ATRIGGER:SIMPLE:SLOPE {word}"), 1)?;
                self.state.slope = idx;
            }
            Key::Coupling => {
                let i = analog(group)?;
                let text = value.as_str().ok_or_else(bad)?;
                let idx = protocol::COUPLINGS.iter().position(|c| *c == text).ok_or_else(bad)?;
                self.setting(&format!(":CHANNEL{}:COUPLING {text}", i + 1), 2)?;
                self.state.analog[i].coupling = idx;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let group = self.group(group)?;
        let strs = |items: &[&str]| ValueList::Str(items.iter().map(|s| (*s).to_owned()).collect());
        Ok(match (key, group) {
            (Key::Timebase, Group::None) => ValueList::Rational(protocol::TIMEBASES.to_vec()),
            (Key::TriggerSource, Group::None) => strs(self.config.sources),
            (Key::TriggerSlope, Group::None) => strs(&protocol::SLOPES),
            (Key::Coupling | Key::Vdiv, Group::None) => {
                return Err(Error::ChannelGroup(String::new()));
            }
            (Key::Coupling, _) => strs(&protocol::COUPLINGS),
            (Key::Vdiv, _) => ValueList::Rational(protocol::VDIVS.to_vec()),
            (other, _) => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.scpi.lock().expect("scpi lock").is_some() {
            return Ok(());
        }
        let Some(mut scpi) = self.closed.take() else { return Err(Error::Busy) };
        if let Err(e) = scpi.reopen() {
            self.closed = Some(scpi);
            return Err(e);
        }
        *self.scpi.lock().expect("scpi lock") = Some(scpi);
        let config = self.config;
        let mut state = self.state.clone();
        self.with_scpi(|s| scope_state_query(s, config, &mut state))?;
        for i in 0..config.analog {
            self.channels[i].enabled = state.analog[i].on;
        }
        for i in 0..config.digital {
            self.channels[config.analog + i].enabled = state.digital[i];
        }
        self.state = state;
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.end_worker();
        if let Some(mut scpi) = self.scpi.lock().expect("scpi lock").take() {
            let _ = scpi.close();
            self.closed = Some(scpi);
        }
        Ok(())
    }

    /// C numbers logic from 32, which comes after every analog channel.
    fn channels_listed_as_they_are(&self) -> bool {
        true
    }

    fn check_capture(&self, _trigger: Option<&crate::core::Trigger>) -> Result<()> {
        self.capture_refusal().map(|_| ())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        // A capture over, or stuck on a full queue no one reads.
        self.end_worker();
        let enabled = self.capture_refusal()?;
        let Some(scpi) = self.scpi.lock().expect("scpi lock").take() else {
            return Err(Error::device("device is not open"));
        };
        if self.frame_limit > 1 {
            log::warn!("Only what the scope holds is read: 1 frame of {}.", self.frame_limit);
        }
        // The timebase as the open read it or a setting made it: asking the scope
        // again here would be a query C doesn't send.
        let (p, q) = protocol::TIMEBASES[self.state.timebase];
        let (bounds, warning) = self.bounds.for_timebase(p as f64 / q as f64);
        if let Some(text) = warning {
            log::warn!("{text}");
        }
        let stop = Arc::new(AtomicBool::new(false));
        let acq = Acquisition {
            channels: enabled,
            state: self.state.clone(),
            usbtmc: scpi.resource().starts_with("usbtmc"),
            bounds,
            stop: Arc::clone(&stop),
        };
        let slot = Arc::clone(&self.scpi);
        let handle = std::thread::Builder::new()
            .name("yokogawa-dlm acquisition".to_owned())
            .spawn(move || {
                let mut scpi = scpi;
                let outcome = acq.run(&mut scpi, &sink);
                // Back before the end, so a client can start again on seeing it.
                *slot.lock().expect("scpi lock") = Some(scpi);
                sink.end_while(outcome, || !acq.stop.load(Ordering::SeqCst));
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

/// One acquisition on its own thread.
struct Acquisition {
    channels: Vec<Channel>,
    /// A block comes whole, to the end of its message, so one with no line end lost a
    /// byte.
    usbtmc: bool,
    state: State,
    bounds: Bounds,
    stop: Arc<AtomicBool>,
}

/// An answer to `:WAVEFORM:SEND?`.
#[derive(Debug, Default, PartialEq, Eq)]
struct Answer {
    /// `None` when what began the answer was no header.
    header: Option<Header>,
    /// The bytes after the header, up to what a frame needs.
    kept: Vec<u8>,
    /// How many came after the header, all told.
    after: usize,
}

/// Fails a capture with C's words.
fn fail<T>(text: &str) -> Result<T> {
    log::error!("{text}");
    Err(Error::device(text))
}

impl Acquisition {
    fn stopped(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    /// What a quiet read comes to; `Err` ends the capture.
    fn quiet(silence: &mut Silence, asked: Instant) -> Result<()> {
        match silence.quiet(asked) {
            Quiet::Wait => Ok(()),
            Quiet::Warn(text) => {
                log::warn!("{text}");
                Ok(())
            }
            Quiet::GiveUp(text) => fail(&text),
        }
    }

    /// `dlm_channel_data_request`.
    fn request(scpi: &mut Scpi, ch: &Channel) -> Result<()> {
        let trace = match ch.kind {
            ChannelKind::Analog => (ch.index + 1).to_string(),
            ChannelKind::Logic => "LOGIC".to_owned(),
        };
        let lines = [
            ":WAVEFORM:FORMAT BYTE".to_owned(),
            ":WAVEFORM:RECORD 0".to_owned(),
            ":WAVEFORM:START 0".to_owned(),
            ":WAVEFORM:END 124999999".to_owned(),
            format!(":WAVEFORM:TRACE {trace}"),
            ":WAVEFORM:SEND? 1".to_owned(),
        ];
        for line in &lines {
            if let Err(e) = scpi.send(line) {
                log::error!("Failed to request acquisition data.");
                return Err(e);
            }
        }
        Ok(())
    }

    /// One answer, as `dlm_data_receive` reads it, up to the end of the message, and on
    /// `tcp-raw` on to what the header says. `None` on a stop. `tail` says the last
    /// answer ended short of its line end, which then begins this one.
    fn read_answer(
        &self,
        scpi: &mut Scpi,
        silence: &mut Silence,
        tail: bool,
    ) -> Result<Option<Answer>> {
        let need = self.state.samples_per_frame as usize;
        // A retried read_begin on USBTMC is a new request, as in C, where the source's
        // callback tries again.
        loop {
            if self.stopped() {
                return Ok(None);
            }
            let asked = Instant::now();
            match scpi.read_begin() {
                Ok(()) => break,
                Err(e @ (Error::Disconnected | Error::Io(_))) => return Err(e),
                Err(e) => {
                    log::debug!("No start of an answer: {e}.");
                    Self::quiet(silence, asked)?;
                    std::thread::sleep(BEGIN_PAUSE);
                }
            }
        }
        let mut buffer = [0u8; RECEIVE_BUFFER_SIZE];
        let mut head: Vec<u8> = Vec::new();
        let mut answer = Answer::default();
        let mut started = !tail;
        let mut decided = false;
        let mut total = 0usize;
        loop {
            if self.stopped() {
                return Ok(None);
            }
            let asked = Instant::now();
            let n = match scpi.read_data(&mut buffer) {
                Ok(n) => n,
                Err(Error::Timeout) => 0,
                // A message over and short of what it said: the end of it.
                Err(_) if total > 0 && scpi.read_complete() => return Ok(Some(answer)),
                Err(e) => {
                    log::error!("Error while reading data: {e}");
                    return Err(e);
                }
            };
            if n == 0 {
                if total > 0 && scpi.read_complete() {
                    // `tcp-raw` has it complete on any silence: only the header says.
                    let enough = answer.header.is_some_and(|h| answer.after >= h.bytes);
                    if enough || (decided && answer.header.is_none()) || silence.gap_over(asked) {
                        return Ok(Some(answer));
                    }
                    continue;
                }
                Self::quiet(silence, asked)?;
                continue;
            }
            if let Some(text) = silence.heard(Instant::now()) {
                log::warn!("{text}");
            }
            let mut bytes = &buffer[..n];
            if !started {
                started = true;
                if bytes[0] == b'\n' {
                    bytes = &bytes[1..];
                }
            }
            total += bytes.len();
            if decided {
                Self::keep(&mut answer, bytes, need);
            } else {
                head.extend_from_slice(bytes);
                match protocol::block_head(&head) {
                    Head::Short => {}
                    Head::Bad => decided = true,
                    Head::Good(header) => {
                        decided = true;
                        answer.header = Some(header);
                        Self::keep(&mut answer, &head[header.len..], need);
                    }
                }
            }
            // Whole, or no block a wait would mend. A short one reads on: on USBTMC that
            // finds the message's end, on `tcp-raw` the rest or silence.
            let enough = answer.header.is_some_and(|h| answer.after >= h.bytes);
            if scpi.read_complete() && (enough || (decided && answer.header.is_none())) {
                return Ok(Some(answer));
            }
            let known = answer.header.map_or(0, |h| h.bytes).max(need);
            if total > known + SURPLUS {
                return fail("The scope sends more than a waveform holds.");
            }
        }
    }

    fn keep(answer: &mut Answer, bytes: &[u8], need: usize) {
        answer.after += bytes.len();
        let room = need.saturating_sub(answer.kept.len());
        answer.kept.extend_from_slice(&bytes[..bytes.len().min(room)]);
    }

    /// `dlm_analog_samples_send` and `dlm_digital_samples_send`: the first
    /// `samples_per_frame` bytes. Ok(false) when nobody takes them any more.
    fn send_samples(&self, sink: &PacketSink, ch: &Channel, bytes: &[u8]) -> bool {
        let go_on = || !self.stopped();
        match ch.kind {
            ChannelKind::Analog => {
                let st = self.state.analog[ch.index];
                bytes.chunks(PACKET_SAMPLES).all(|chunk| {
                    let samples = chunk.iter().map(|b| protocol::volts(*b, st.range, st.offset));
                    let data = AnalogData {
                        channels: vec![ch.index],
                        samples: samples.collect(),
                        mq: Mq::Voltage,
                        flags: MqFlags::empty(),
                        unit: Unit::Volt,
                        // TODO in C too: a proper one for this device and its modes.
                        digits: 2,
                    };
                    sink.send_while(Packet::Analog(data), go_on)
                })
            }
            ChannelKind::Logic => bytes
                .chunks(PACKET_SAMPLES)
                .all(|chunk| sink.send_while(Packet::logic(1, chunk.to_vec()), go_on)),
        }
    }

    fn run(&self, scpi: &mut Scpi, sink: &PacketSink) -> Result<()> {
        let go_on = || !self.stopped();
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let need = self.state.samples_per_frame as usize;
        let mut silence = Silence::new(self.bounds, Instant::now());
        let mut empties = Empties::new(&self.bounds);
        let mut frame_begun = false;
        let mut tail = false;
        for ch in &self.channels {
            Self::request(scpi, ch)?;
            silence.asked(Instant::now());
            let answer = loop {
                let Some(answer) = self.read_answer(scpi, &mut silence, tail)? else {
                    return Ok(());
                };
                tail = false;
                if !frame_begun {
                    frame_begun = true;
                    if !sink.send_while(Packet::FrameBegin, go_on) {
                        return Ok(());
                    }
                }
                let Some(header) = answer.header else {
                    return fail("Encountered malformed block data header.");
                };
                if header.bytes != 0 {
                    break answer;
                }
                match empties.empty(Instant::now()) {
                    Quiet::Wait => {}
                    Quiet::Warn(text) => log::warn!("{text}"),
                    Quiet::GiveUp(text) => return fail(&text),
                }
                // Its result doesn't matter: the request after it says.
                let _ = scpi.send(STOP_CMD);
                std::thread::sleep(self.bounds.empty_pause);
                Self::request(scpi, ch)?;
                silence.asked(Instant::now());
            };
            let header = answer.header.unwrap_or(Header { len: 0, bytes: 0 });
            // What the header doesn't count isn't a sample, however many bytes came.
            if header.bytes < need || answer.after < header.bytes {
                return fail("Truncated waveform data packet received.");
            }
            // The manual ends every answer with one (<RMT>, p. 4-5). On USBTMC a block comes
            // whole, so a byte is gone and the last sample is the line end. On `tcp-raw`
            // the line end may only be late, and then it begins the next answer: that a
            // byte is lost there is only a warning, as nothing tells the two apart.
            // tcp-raw ends a block at its length and keeps a line end that came for
            // the next answer: one that came is taken here.
            let line_end = !self.usbtmc && scpi.take_line_end() == Some(true);
            if answer.after == header.bytes && !line_end {
                if self.usbtmc {
                    return fail("A block with no line end: a byte of it is lost.");
                }
                tail = true;
                log::warn!("A block with no line end: a byte of it may be lost.");
            }
            if !self.send_samples(sink, ch, &answer.kept) {
                return Ok(());
            }
            silence.sample_given();
        }
        sink.send_while(Packet::FrameEnd, go_on);
        Ok(())
    }
}
