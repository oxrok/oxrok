// Counterpart in sigrok:
//   libsigrok/src/hardware/demo/api.c,
//   libsigrok/src/hardware/demo/protocol.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The demo driver: a pattern generator that needs no hardware.
//!
//! It exists so the CLI, the formats and the decoders can be tested
//! anywhere, and so a new front end has something to talk to. Its
//! options and defaults follow sigrok's demo driver, so the same command
//! lines work: `-d demo:logic_channels=4`, `-c pattern=walking-one`,
//! `-g A0 -c pattern=sine`.

#![forbid(unsafe_code)]

pub mod patterns;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, PreTriggerBuffer, Result,
    SoftTrigger, Trigger, TriggerMatch, Value, ValueList,
};
use patterns::{ANALOG_TABLE_LEN, AnalogPattern, LogicGen, LogicPattern, Rng, analog_table};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const DEFAULT_LOGIC_CHANNELS: usize = 8;
const DEFAULT_ANALOG_CHANNELS: usize = 5;
const DEFAULT_SAMPLERATE: u64 = 200_000;
const DEFAULT_AMPLITUDE: f64 = 10.0;
const DEFAULT_CAPTURE_RATIO: u64 = 20;
const SAMPLES_PER_FRAME: u64 = 1000;
const LOGIC_CHUNK_BYTES: usize = 4096;
const TICK: Duration = Duration::from_millis(10);
const ANALOG_DIGITS: i32 = 4;

/// The demo driver.
pub struct DemoDriver;

static SCAN_OPTIONS: [Key; 3] = [Key::NumLogicChannels, Key::NumAnalogChannels, Key::LimitFrames];
static CLASSES: [DeviceClass; 3] =
    [DeviceClass::Demo, DeviceClass::LogicAnalyzer, DeviceClass::Oscilloscope];

impl Driver for DemoDriver {
    fn id(&self) -> &'static str {
        "demo"
    }

    fn long_name(&self) -> &'static str {
        "Demo driver and pattern generator"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut logic = DEFAULT_LOGIC_CHANNELS;
        let mut analog = DEFAULT_ANALOG_CHANNELS;
        let mut frames = 0;
        for (key, value) in options {
            match key {
                Key::NumLogicChannels => {
                    logic = usize::try_from(value.as_i32().unwrap_or(-1))
                        .map_err(|_| Error::arg("logic_channels must be 0 or more"))?;
                }
                Key::NumAnalogChannels => {
                    analog = usize::try_from(value.as_i32().unwrap_or(-1))
                        .map_err(|_| Error::arg("analog_channels must be 0 or more"))?;
                }
                Key::LimitFrames => frames = value.as_u64().unwrap_or(0),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        if logic > 64 {
            return Err(Error::arg("at most 64 logic channels"));
        }
        Ok(vec![Box::new(DemoDevice::new(logic, analog, frames))])
    }
}

/// Settings of one analog channel.
#[derive(Clone, Debug)]
struct AnalogGen {
    channel_index: usize,
    pattern: AnalogPattern,
    amplitude: f64,
    offset: f64,
    mq: Mq,
    flags: MqFlags,
}

/// Everything the acquisition thread needs, copied at start.
#[derive(Clone, Debug)]
struct Settings {
    samplerate: u64,
    limit_samples: u64,
    limit_msec: u64,
    limit_frames: u64,
    averaging: bool,
    avg_samples: u64,
    capture_ratio: u64,
    logic_pattern: LogicPattern,
    analog: Vec<AnalogGen>,
}

/// The demo device.
pub struct DemoDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    num_logic: usize,
    settings: Settings,
    trigger: Option<Trigger>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl DemoDevice {
    fn new(num_logic: usize, num_analog: usize, limit_frames: u64) -> Self {
        let mut channels = Vec::new();
        let mut groups = Vec::new();
        if num_logic > 0 {
            for i in 0..num_logic {
                channels.push(Channel::logic(i));
            }
            groups.push(ChannelGroup::new("Logic", (0..num_logic).collect()));
        }
        let mut analog = Vec::new();
        if num_analog > 0 {
            for i in 0..num_analog {
                channels.push(Channel::new(num_logic + i, ChannelKind::Analog, format!("A{i}")));
                analog.push(AnalogGen {
                    channel_index: num_logic + i,
                    pattern: AnalogPattern::ALL[i % AnalogPattern::ALL.len()],
                    amplitude: DEFAULT_AMPLITUDE,
                    offset: 0.0,
                    mq: Mq::Voltage,
                    flags: MqFlags::DC,
                });
            }
            groups.push(ChannelGroup::new("Analog", (num_logic..num_logic + num_analog).collect()));
            for i in 0..num_analog {
                groups.push(ChannelGroup::new(format!("A{i}"), vec![num_logic + i]));
            }
        }
        DemoDevice {
            info: DeviceInfo { driver: "demo", model: "Demo device".into(), ..Default::default() },
            channels,
            groups,
            num_logic,
            settings: Settings {
                samplerate: DEFAULT_SAMPLERATE,
                limit_samples: 0,
                limit_msec: 0,
                limit_frames,
                averaging: false,
                avg_samples: 0,
                capture_ratio: DEFAULT_CAPTURE_RATIO,
                logic_pattern: LogicPattern::Oxrok,
                analog,
            },
            trigger: None,
            worker: None,
        }
    }

    /// The analog generators of `group`: all of them for the device or
    /// the `Analog` group, one for a per-channel group.
    fn analog_of(&self, group: Option<&str>) -> Result<Vec<usize>> {
        let positions = self.group_positions(group)?;
        let gens: Vec<usize> = positions
            .iter()
            .filter(|p| self.channels[**p].kind == ChannelKind::Analog)
            .map(|p| self.channels[*p].index - self.num_logic)
            .collect();
        if gens.is_empty() {
            return Err(Error::arg("this option needs an analog channel group"));
        }
        Ok(gens)
    }

    fn group_kind(&self, group: &str) -> Result<ChannelKind> {
        let positions = self.group_positions(Some(group))?;
        positions.first().map(|p| self.channels[*p].kind).ok_or_else(|| Error::arg("empty group"))
    }

    fn is_all_analog_group(group: &str) -> bool {
        group.eq_ignore_ascii_case("Analog")
    }
}

static DEV_OPTIONS: [OptionDesc; 9] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::Averaging, Access::GET_SET),
    OptionDesc::new(Key::AvgSamples, Access::GET_SET),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
];
static LOGIC_OPTIONS: [OptionDesc; 1] = [OptionDesc::new(Key::PatternMode, Access::GET_SET_LIST)];
static ANALOG_GROUP_OPTIONS: [OptionDesc; 2] = [
    OptionDesc::new(Key::Amplitude, Access::GET_SET),
    OptionDesc::new(Key::Offset, Access::GET_SET),
];
static ANALOG_CHANNEL_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::MeasuredQuantity, Access::GET_SET),
    OptionDesc::new(Key::PatternMode, Access::GET_SET_LIST),
    OptionDesc::new(Key::Amplitude, Access::GET_SET),
    OptionDesc::new(Key::Offset, Access::GET_SET),
];

impl Device for DemoDevice {
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
        let ch = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        ch.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        name.clone_into(&mut ch.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(match group {
            None => DEV_OPTIONS.to_vec(),
            Some(g) => match self.group_kind(g)? {
                ChannelKind::Logic => LOGIC_OPTIONS.to_vec(),
                ChannelKind::Analog if Self::is_all_analog_group(g) => {
                    ANALOG_GROUP_OPTIONS.to_vec()
                }
                ChannelKind::Analog => ANALOG_CHANNEL_OPTIONS.to_vec(),
            },
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let s = &self.settings;
        Ok(match (key, group) {
            (Key::Samplerate, None) => Value::U64(s.samplerate),
            (Key::LimitSamples, None) => Value::U64(s.limit_samples),
            (Key::LimitMsec, None) => Value::U64(s.limit_msec),
            (Key::LimitFrames, None) => Value::U64(s.limit_frames),
            (Key::Averaging, None) => Value::Bool(s.averaging),
            (Key::AvgSamples, None) => Value::U64(s.avg_samples),
            (Key::CaptureRatio, None) => Value::U64(s.capture_ratio),
            (Key::PatternMode, Some(g)) => match self.group_kind(g)? {
                ChannelKind::Logic => Value::Str(s.logic_pattern.id().to_owned()),
                ChannelKind::Analog => {
                    let gens = self.analog_of(group)?;
                    Value::Str(s.analog[gens[0]].pattern.id().to_owned())
                }
            },
            (Key::Amplitude, Some(_)) => Value::F64(s.analog[self.analog_of(group)?[0]].amplitude),
            (Key::Offset, Some(_)) => Value::F64(s.analog[self.analog_of(group)?[0]].offset),
            (Key::MeasuredQuantity, Some(_)) => {
                let g = &s.analog[self.analog_of(group)?[0]];
                Value::Mq(g.mq, g.flags)
            }
            (k, _) => return Err(Error::UnknownKey(k.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value '{value}' for '{key}'"));
        match (key, group) {
            (Key::Samplerate, None) => {
                let rate = value.as_u64().ok_or_else(bad)?;
                if rate == 0 || rate > 1_000_000_000 {
                    return Err(bad());
                }
                self.settings.samplerate = rate;
            }
            (Key::LimitSamples, None) => {
                self.settings.limit_samples = value.as_u64().ok_or_else(bad)?;
                self.settings.limit_msec = 0;
            }
            (Key::LimitMsec, None) => {
                self.settings.limit_msec = value.as_u64().ok_or_else(bad)?;
                self.settings.limit_samples = 0;
            }
            (Key::LimitFrames, None) => {
                self.settings.limit_frames = value.as_u64().ok_or_else(bad)?;
            }
            (Key::Averaging, None) => self.settings.averaging = value.as_bool().ok_or_else(bad)?,
            (Key::AvgSamples, None) => {
                self.settings.avg_samples = value.as_u64().ok_or_else(bad)?;
            }
            (Key::CaptureRatio, None) => {
                let r = value.as_u64().ok_or_else(bad)?;
                if r > 100 {
                    return Err(bad());
                }
                self.settings.capture_ratio = r;
            }
            (Key::PatternMode, Some(g)) => {
                let name = value.as_str().ok_or_else(bad)?;
                match self.group_kind(g)? {
                    ChannelKind::Logic => {
                        self.settings.logic_pattern =
                            LogicPattern::from_id(name).ok_or_else(bad)?;
                    }
                    ChannelKind::Analog => {
                        if Self::is_all_analog_group(g) {
                            return Err(Error::not_supported("pattern on the Analog group"));
                        }
                        let p = AnalogPattern::from_id(name).ok_or_else(bad)?;
                        for i in self.analog_of(group)? {
                            self.settings.analog[i].pattern = p;
                        }
                    }
                }
            }
            (Key::Amplitude, Some(_)) => {
                let a = value.as_f64().ok_or_else(bad)?;
                for i in self.analog_of(group)? {
                    self.settings.analog[i].amplitude = a;
                }
            }
            (Key::Offset, Some(_)) => {
                let o = value.as_f64().ok_or_else(bad)?;
                for i in self.analog_of(group)? {
                    self.settings.analog[i].offset = o;
                }
            }
            (Key::MeasuredQuantity, Some(_)) => {
                let Value::Mq(mq, flags) = value else { return Err(bad()) };
                for i in self.analog_of(group)? {
                    self.settings.analog[i].mq = *mq;
                    self.settings.analog[i].flags = *flags;
                }
            }
            (k, _) => return Err(Error::UnknownKey(k.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        Ok(match (key, group) {
            (Key::Samplerate, None) => {
                ValueList::RangeStepU64 { min: 1, max: 1_000_000_000, step: 1 }
            }
            (Key::TriggerMatch, None) => ValueList::TriggerMatches(vec![
                TriggerMatch::Zero,
                TriggerMatch::One,
                TriggerMatch::Rising,
                TriggerMatch::Falling,
                TriggerMatch::Edge,
            ]),
            (Key::PatternMode, Some(g)) => match self.group_kind(g)? {
                ChannelKind::Logic => {
                    ValueList::Str(LogicPattern::ALL.iter().map(|p| p.id().to_owned()).collect())
                }
                ChannelKind::Analog if Self::is_all_analog_group(g) => {
                    return Err(Error::not_supported("pattern on the Analog group"));
                }
                ChannelKind::Analog => {
                    ValueList::Str(AnalogPattern::ALL.iter().map(|p| p.id().to_owned()).collect())
                }
            },
            (k, _) => return Err(Error::UnknownKey(k.id().to_owned())),
        })
    }

    fn trigger_features(&self) -> crate::core::TriggerFeatures {
        SoftTrigger::FEATURES
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            SoftTrigger::check(t)?;
        }
        self.trigger = trigger;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let settings = self.settings.clone();
        let trigger = self.trigger.clone();
        let logic_enabled: Vec<usize> = self
            .channels
            .iter()
            .filter(|c| c.kind == ChannelKind::Logic && c.enabled)
            .map(|c| c.index)
            .collect();
        let analog_enabled: Vec<usize> = self
            .channels
            .iter()
            .filter(|c| c.kind == ChannelKind::Analog && c.enabled)
            .map(|c| c.index - self.num_logic)
            .collect();
        let num_logic = self.num_logic;
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("demo acquisition".into())
            .spawn(move || {
                let mut acq = Acquisition::new(settings, num_logic, &logic_enabled, analog_enabled);
                acq.set_trigger(trigger.as_ref());
                acq.run(&sink, &stop2);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        Ok(())
    }
}

/// One running acquisition, on its own thread.
struct Acquisition {
    settings: Settings,
    logic: Option<LogicGen>,
    logic_mask: Vec<u8>,
    analog_enabled: Vec<usize>,
    tables: Vec<Vec<f32>>,
    rng: Rng,
    /// Averaging state per analog generator: sum and count.
    avg: Vec<(f64, u64)>,
    /// The software trigger, until it fires.
    soft: Option<SoftTrigger>,
    /// Samples kept before the trigger, up to the capture ratio.
    pre: PreTriggerBuffer,
}

impl Acquisition {
    fn new(
        settings: Settings,
        num_logic: usize,
        logic_enabled: &[usize],
        analog_enabled: Vec<usize>,
    ) -> Self {
        let seed = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0x5eed, |d| d.as_nanos() as u64);
        let logic = (num_logic > 0 && !logic_enabled.is_empty())
            .then(|| LogicGen::new(settings.logic_pattern, num_logic, seed));
        let unit_size = crate::core::channel::unit_size(num_logic);
        let mut logic_mask = vec![0u8; unit_size];
        for i in logic_enabled {
            logic_mask[i / 8] |= 1 << (i % 8);
        }
        let tables = AnalogPattern::ALL.iter().map(|p| analog_table(*p)).collect();
        let avg = vec![(0.0, 0); settings.analog.len()];
        Acquisition {
            soft: None,
            pre: PreTriggerBuffer::new(1, 0),
            settings,
            logic,
            logic_mask,
            analog_enabled,
            tables,
            rng: Rng::new(seed.rotate_left(17)),
            avg,
        }
    }

    /// Arms the software trigger on the logic channels. Samples before
    /// the trigger are dropped, except the capture ratio's share of the
    /// sample limit.
    fn set_trigger(&mut self, trigger: Option<&Trigger>) {
        let Some(g) = &self.logic else { return };
        let unit = g.unit_size();
        self.soft = trigger.and_then(|t| SoftTrigger::new(t, unit));
        self.pre = PreTriggerBuffer::new(
            unit,
            pre_trigger_samples(self.settings.limit_samples, self.settings.capture_ratio, unit),
        );
    }

    fn run(&mut self, sink: &PacketSink, stop: &AtomicBool) {
        let s = self.settings.clone();
        if !sink.send(Packet::header()) {
            return;
        }
        let frames = s.limit_frames > 0;
        if frames {
            sink.send(Packet::FrameBegin);
        }
        let start = Instant::now();
        let mut spent_us: u64 = 0;
        let mut sent: u64 = 0;
        let mut frame_sent: u64 = 0;
        let mut frames_left = s.limit_frames;
        let limit_us = s.limit_msec * 1000;
        let mut in_frame = frames;
        loop {
            if stop.load(Ordering::SeqCst) {
                break;
            }
            let elapsed_us = start.elapsed().as_micros() as u64;
            let todo_us = if limit_us > 0 && limit_us < elapsed_us {
                limit_us.saturating_sub(spent_us)
            } else {
                elapsed_us.saturating_sub(spent_us)
            };
            let mut todo =
                (u128::from(todo_us) * u128::from(s.samplerate)).div_ceil(1_000_000) as u64;
            if s.limit_samples > 0 {
                todo = todo.min(s.limit_samples.saturating_sub(sent));
            }
            if frames {
                todo = todo.min(SAMPLES_PER_FRAME - frame_sent);
            }
            if todo == 0 {
                if (s.limit_samples > 0 && sent >= s.limit_samples)
                    || (limit_us > 0 && spent_us >= limit_us)
                {
                    break;
                }
                std::thread::sleep(TICK);
                continue;
            }
            let covered_us = todo * 1_000_000 / s.samplerate;
            let Some(emitted) = self.send_logic(sink, todo) else { return };
            if !self.send_analog(sink, sent, emitted) {
                return;
            }
            sent += emitted;
            frame_sent += emitted;
            spent_us += covered_us;
            if frames && frame_sent >= SAMPLES_PER_FRAME {
                sink.send(Packet::FrameEnd);
                in_frame = false;
                frame_sent = 0;
                frames_left -= 1;
                if frames_left == 0 {
                    break;
                }
            }
            if (s.limit_samples > 0 && sent >= s.limit_samples)
                || (limit_us > 0 && spent_us >= limit_us)
            {
                break;
            }
            if frames && frame_sent == 0 {
                sink.send(Packet::FrameBegin);
                in_frame = true;
            }
            std::thread::sleep(TICK);
        }
        self.flush_averages(sink);
        if in_frame {
            sink.send(Packet::FrameEnd);
        }
        sink.send(Packet::End);
    }

    /// Generates `count` logic samples and sends those past the
    /// trigger. Returns the number sent, or `None` when the consumer is
    /// gone.
    fn send_logic(&mut self, sink: &PacketSink, count: u64) -> Option<u64> {
        let Some(g) = &mut self.logic else { return Some(count) };
        let unit = g.unit_size();
        let per_chunk = (LOGIC_CHUNK_BYTES / unit).max(1) as u64;
        let mut left = count;
        let mut emitted = 0u64;
        while left > 0 {
            let n = left.min(per_chunk) as usize;
            let mut data = Vec::with_capacity(n * unit);
            g.generate(n, &mut data);
            for sample in data.chunks_exact_mut(unit) {
                for (b, m) in sample.iter_mut().zip(&self.logic_mask) {
                    *b &= m;
                }
            }
            left -= n as u64;
            if let Some(soft) = self.soft.as_mut() {
                match soft.feed(&data) {
                    None => {
                        self.pre.push(&data);
                        continue;
                    }
                    Some(i) => {
                        let mut before = self.pre.take();
                        before.extend_from_slice(&data[..i * unit]);
                        if !before.is_empty() {
                            emitted += (before.len() / unit) as u64;
                            if !sink.send(Packet::logic(unit, before)) {
                                return None;
                            }
                        }
                        sink.send(Packet::Trigger);
                        data.drain(..i * unit);
                        self.soft = None;
                    }
                }
            }
            emitted += (data.len() / unit) as u64;
            if !sink.send(Packet::logic(unit, data)) {
                return None;
            }
        }
        Some(emitted)
    }

    fn send_analog(&mut self, sink: &PacketSink, from: u64, count: u64) -> bool {
        let Acquisition { settings, tables, rng, avg, analog_enabled, .. } = self;
        for &gi in analog_enabled.iter() {
            let agen = &settings.analog[gi];
            let table =
                &tables[AnalogPattern::ALL.iter().position(|p| *p == agen.pattern).unwrap_or(0)];
            let mut pos = from;
            let mut left = count;
            while left > 0 {
                let at = if table.is_empty() { 0 } else { (pos % table.len() as u64) as usize };
                let n = if table.is_empty() {
                    left.min(ANALOG_TABLE_LEN as u64) as usize
                } else {
                    left.min((table.len() - at) as u64) as usize
                };
                let samples: Vec<f32> = (0..n)
                    .map(|i| {
                        let unit = if table.is_empty() {
                            rng.next_f32() * 2.0 - 1.0
                        } else {
                            table[at + i]
                        };
                        (f64::from(unit) * agen.amplitude + agen.offset) as f32
                    })
                    .collect();
                if settings.averaging {
                    let (sum, cnt) = &mut avg[gi];
                    let mut out = Vec::new();
                    for v in samples {
                        *sum += f64::from(v);
                        *cnt += 1;
                        if settings.avg_samples > 0 && *cnt >= settings.avg_samples {
                            out.push((*sum / *cnt as f64) as f32);
                            *sum = 0.0;
                            *cnt = 0;
                        }
                    }
                    if !out.is_empty() && !sink.send(Packet::Analog(analog_packet(agen, out))) {
                        return false;
                    }
                } else if !sink.send(Packet::Analog(analog_packet(agen, samples))) {
                    return false;
                }
                pos += n as u64;
                left -= n as u64;
            }
        }
        true
    }

    fn flush_averages(&mut self, sink: &PacketSink) {
        if !self.settings.averaging || self.settings.avg_samples != 0 {
            return;
        }
        for gi in self.analog_enabled.clone() {
            let (sum, cnt) = self.avg[gi];
            if cnt > 0 {
                let agen = self.settings.analog[gi].clone();
                sink.send(Packet::Analog(analog_packet(&agen, vec![(sum / cnt as f64) as f32])));
            }
        }
    }
}

fn analog_packet(agen: &AnalogGen, samples: Vec<f32>) -> AnalogData {
    AnalogData {
        channels: vec![agen.channel_index],
        samples,
        mq: agen.mq,
        flags: agen.flags,
        unit: agen.mq.default_unit(),
        digits: ANALOG_DIGITS,
    }
}

/// The capture ratio's share of the sample limit, whose bytes must fit.
fn pre_trigger_samples(limit: u64, ratio: u64, unit: usize) -> usize {
    let share = u128::from(limit) * u128::from(ratio) / 100;
    let most = usize::MAX / unit.max(1);
    usize::try_from(share).map_or(most, |share| share.min(most))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_huge_limit_keeps_its_pre_trigger_share_in_range() {
        const HUGE: u64 = 18_446_744_073_709_549_568;
        assert_eq!(pre_trigger_samples(HUGE, 20, 1), 3_689_348_814_741_909_913);
        assert_eq!(pre_trigger_samples(HUGE, 100, 1), usize::try_from(HUGE).unwrap());
        assert_eq!(pre_trigger_samples(HUGE, 100, 2), usize::MAX / 2);
        assert_eq!(pre_trigger_samples(1000, 25, 1), 250);
    }
    use std::sync::mpsc::sync_channel;

    fn capture(args: &[(Key, Value)], configure: impl FnOnce(&mut dyn Device)) -> Vec<Packet> {
        let mut dev = DemoDriver.scan(args).unwrap().remove(0);
        configure(dev.as_mut());
        let (tx, rx) = sync_channel(64);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        let mut packets = Vec::new();
        for (_, p) in rx {
            let end = matches!(p, Packet::End);
            packets.push(p);
            if end {
                break;
            }
        }
        dev.stop().unwrap();
        packets
    }

    fn logic_bytes(packets: &[Packet]) -> Vec<u8> {
        packets
            .iter()
            .filter_map(|p| match p {
                Packet::Logic(l) => Some(l.data.clone()),
                _ => None,
            })
            .flatten()
            .collect()
    }

    #[test]
    fn scan_builds_channels_and_groups() {
        let dev = DemoDriver
            .scan(&[
                (Key::NumLogicChannels, Value::I32(3)),
                (Key::NumAnalogChannels, Value::I32(1)),
            ])
            .unwrap()
            .remove(0);
        let names: Vec<&str> = dev.channels().iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["D0", "D1", "D2", "A0"]);
        let groups: Vec<&str> = dev.channel_groups().iter().map(|g| g.name.as_str()).collect();
        assert_eq!(groups, ["Logic", "Analog", "A0"]);
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(200_000));
        assert_eq!(dev.get(&Key::PatternMode, Some("Logic")).unwrap(), Value::Str("oxrok".into()));
        assert_eq!(dev.get(&Key::PatternMode, Some("A0")).unwrap(), Value::Str("square".into()));
    }

    #[test]
    fn limited_capture_produces_exact_sample_count() {
        let packets = capture(
            &[(Key::NumLogicChannels, Value::I32(4)), (Key::NumAnalogChannels, Value::I32(0))],
            |d| {
                d.set(&Key::LimitSamples, &Value::U64(20), None).unwrap();
                d.set(&Key::PatternMode, &Value::Str("walking-one".into()), Some("Logic")).unwrap();
            },
        );
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert!(matches!(packets.last(), Some(Packet::End)));
        let data = logic_bytes(&packets);
        assert_eq!(data.len(), 20);
        assert_eq!(&data[..6], &[0, 1, 2, 4, 8, 0]);
    }

    #[test]
    fn disabled_channels_read_low() {
        let packets = capture(
            &[(Key::NumLogicChannels, Value::I32(8)), (Key::NumAnalogChannels, Value::I32(0))],
            |d| {
                d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
                d.set(&Key::PatternMode, &Value::Str("all-high".into()), Some("Logic")).unwrap();
                d.enable_channel(7, false).unwrap();
            },
        );
        assert_eq!(logic_bytes(&packets), vec![0x7f; 4]);
    }

    #[test]
    fn analog_follows_the_tables() {
        let packets = capture(
            &[(Key::NumLogicChannels, Value::I32(0)), (Key::NumAnalogChannels, Value::I32(2))],
            |d| {
                d.set(&Key::LimitSamples, &Value::U64(6), None).unwrap();
                d.set(&Key::Amplitude, &Value::F64(2.0), Some("A1")).unwrap();
                d.set(&Key::Offset, &Value::F64(1.0), Some("A1")).unwrap();
            },
        );
        let analog: Vec<&AnalogData> = packets
            .iter()
            .filter_map(|p| if let Packet::Analog(a) = p { Some(a) } else { None })
            .collect();
        let a0: Vec<f32> =
            analog.iter().filter(|a| a.channels == [0]).flat_map(|a| a.samples.clone()).collect();
        let a1: Vec<f32> =
            analog.iter().filter(|a| a.channels == [1]).flat_map(|a| a.samples.clone()).collect();
        assert_eq!(a0, [-10.0, -10.0, -10.0, -10.0, -10.0, 10.0]);
        assert_eq!(a1.len(), 6);
        assert!((a1[0] - 1.0).abs() < 1e-6, "{a1:?}");
        assert!((a1[5] - 3.0).abs() < 1e-6, "{a1:?}");
    }

    #[test]
    fn frames_wrap_the_data() {
        let packets = capture(
            &[
                (Key::NumLogicChannels, Value::I32(1)),
                (Key::NumAnalogChannels, Value::I32(0)),
                (Key::LimitFrames, Value::U64(2)),
            ],
            |_| {},
        );
        let kinds: Vec<&str> = packets
            .iter()
            .filter_map(|p| match p {
                Packet::FrameBegin => Some("begin"),
                Packet::FrameEnd => Some("end"),
                Packet::End => Some("END"),
                _ => None,
            })
            .collect();
        assert_eq!(kinds, ["begin", "end", "begin", "end", "END"]);
        assert_eq!(logic_bytes(&packets).len(), 2000);
    }
}
