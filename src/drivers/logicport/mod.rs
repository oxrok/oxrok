// Counterpart in sigrok:
//   libsigrok/src/hardware/intronix-logicport/api.c.
// This file is a port by the same author, GPL-3.0-or-later like it.

//! Intronix LogicPort LA1034 driver: 34 channels (D0-D31, CLK1, CLK2),
//! 2048 sample entries with run length compression, 1 kHz to 500 MHz
//! timing mode and state mode on an external clock, two hardware trigger
//! levels with patterns and edges, and an adjustable input threshold.
//!
//! A port of the author's libsigrok driver; the protocol is summarised
//! in the `protocol` module. The FPGA bitstreams come from the vendor's
//! `LogicPort.ccf`, see [`firmware`].

#![forbid(unsafe_code)]
#![allow(clippy::doc_markdown)]

pub mod firmware;
pub mod protocol;
pub mod readout;

use crate::core::key::CustomKey;
use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Trigger, TriggerCombine, TriggerDuration, TriggerFeatures,
    TriggerMatch, Value, ValueKind, ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, UsbId};
use protocol::{
    COMBINE_A, COMBINE_A_AND_B, COMBINE_A_OR_B, COMBINE_A_THEN_B, COMBINE_NONE, COUNTER_MAX, Link,
    Program, Qualifier, RATES, Rate, STATUS_DONE, STATUS_PREFILLED, STATUS_TRIGGERED, SamplePoint,
    TriggerLevel, find_rate,
};
use readout::{SampleOut, Sink, UNIT_SIZE};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// The USB id of the LogicPort.
pub const USB_ID: UsbId = UsbId { vendor: 0x0403, product: 0xdc48 };
/// Channels: D0-D31, CLK1, CLK2.
pub const NUM_CHANNELS: usize = 34;
const ALL_CHANNELS: u64 = (1 << NUM_CHANNELS) - 1;
const THRESHOLD_MIN: f64 = -6.0;
const THRESHOLD_MAX: f64 = 6.0;
const THRESHOLD_STEP: f64 = 0.05;
/// Compression is only available up to this sample rate.
const RLE_MAX_RATE: u64 = 200_000_000;
/// Upper bound for RLE expansion when no sample limit was set.
const RLE_DEFAULT_LIMIT: u64 = 16 * 1024 * 1024;
const POLL: Duration = Duration::from_millis(10);
const CLOCK_SOURCES: [&str; 2] = ["CLK1", "CLK2"];
const CLOCK_EDGES: [&str; 2] = ["rising", "falling"];
const QUALIFIERS: [&str; 3] = ["off", "active-high", "active-low"];
const SAMPLE_POINTS: [&str; 3] = ["on", "before", "after"];

/// The device specific options.
static KEY_CLOCK_QUALIFIER: CustomKey = CustomKey {
    id: "clock_qualifier",
    name: "Clock qualifier",
    description: "In state mode, sample only while the qualifier input is high or low.",
    kind: ValueKind::Str,
};
static KEY_SAMPLE_POINT: CustomKey = CustomKey {
    id: "sample_point",
    name: "Sample point",
    description: "In state mode, sample on the clock edge, 2.5 ns before it or 2.5 ns after it.",
    kind: ValueKind::Str,
};
static KEY_INVERT: CustomKey = CustomKey {
    id: "invert",
    name: "Inverted inputs",
    description: "Inputs read inverted, as channel names or a hex mask; none for no inversion.",
    kind: ValueKind::Str,
};
static KEY_FREQUENCY_CHANNEL: CustomKey = CustomKey {
    id: "frequency_channel",
    name: "Frequency counter input",
    description: "The input the frequency counter measures.",
    kind: ValueKind::Str,
};
static KEY_FREQUENCY: CustomKey = CustomKey {
    id: "frequency",
    name: "Frequency",
    description: "The frequency on the frequency counter input in Hz, measured when read.",
    kind: ValueKind::U64,
};

/// What the hardware trigger does beyond level and edge stages.
const FEATURES: TriggerFeatures = TriggerFeatures {
    count: true,
    consecutive: true,
    duration: true,
    range: true,
    or: true,
    and: true,
    max_stages: 2,
};
static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 16] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::VoltageThreshold, Access::GET_SET_LIST),
    OptionDesc::new(Key::Rle, Access::GET_SET),
    OptionDesc::new(Key::ExternalClock, Access::GET_SET),
    OptionDesc::new(Key::ExternalClockSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::ClockEdge, Access::GET_SET_LIST),
    OptionDesc::new(Key::Custom(&KEY_CLOCK_QUALIFIER), Access::GET_SET_LIST),
    OptionDesc::new(Key::Custom(&KEY_SAMPLE_POINT), Access::GET_SET_LIST),
    OptionDesc::new(Key::Custom(&KEY_INVERT), Access::GET_SET),
    OptionDesc::new(Key::Custom(&KEY_FREQUENCY_CHANNEL), Access::GET_SET_LIST),
    OptionDesc::new(Key::Custom(&KEY_FREQUENCY), Access::GET),
];
static TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];

/// The driver.
pub struct LogicPortDriver;

impl Driver for LogicPortDriver {
    fn id(&self) -> &'static str {
        "intronix-logicport"
    }

    fn long_name(&self) -> &'static str {
        "Intronix LogicPort"
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
        let found = crate::usb::find_devices(&[USB_ID])?;
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in found {
            if conn.as_deref().is_some_and(|c| !info.matches_conn(c)) {
                continue;
            }
            if info.product.as_deref().is_some_and(|p| p != "LogicPort Logic Analyzer") {
                log::debug!(
                    "Unexpected product string '{}'.",
                    info.product.as_deref().unwrap_or("")
                );
            }
            devices.push(Box::new(LogicPortDevice::new(info, None)));
        }
        Ok(devices)
    }
}

/// Settings kept between acquisitions.
#[derive(Clone, Debug)]
struct Settings {
    rate: Rate,
    limit_samples: u64,
    limit_frames: u64,
    capture_ratio: u64,
    threshold: f64,
    rle: bool,
    external_clock: bool,
    clock_source: usize,
    clock_edge: usize,
    qualifier: Qualifier,
    sample_point: SamplePoint,
    invert_mask: u64,
    /// The frequency counter input, a channel index.
    freq_channel: usize,
}

/// One LogicPort.
pub struct LogicPortDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    settings: Settings,
    trigger: Option<Trigger>,
    /// A transport to use instead of opening the USB device, for
    /// replayed sessions.
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl LogicPortDevice {
    fn new(usb: UsbDeviceInfo, transport: Option<Box<dyn Transport>>) -> Self {
        let mut channels: Vec<Channel> = (0..32).map(Channel::logic).collect();
        channels.push(Channel::new(32, ChannelKind::Logic, "CLK1"));
        channels.push(Channel::new(33, ChannelKind::Logic, "CLK2"));
        LogicPortDevice {
            info: DeviceInfo {
                driver: "intronix-logicport",
                vendor: "Intronix".into(),
                model: "LogicPort".into(),
                version: String::new(),
                serial: usb.serial.clone().unwrap_or_default(),
                conn: Some(usb.conn()),
            },
            usb,
            channels,
            settings: Settings {
                rate: *find_rate(100_000_000).expect("100 MHz is in the table"),
                limit_samples: 0,
                limit_frames: 0,
                capture_ratio: 50,
                threshold: 1.5,
                rle: false,
                external_clock: false,
                clock_source: 0,
                clock_edge: 0,
                qualifier: Qualifier::Off,
                sample_point: SamplePoint::On,
                invert_mask: 0,
                freq_channel: 32,
            },
            trigger: None,
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport, e.g. a replayed
    /// recording. `open` uses it instead of opening the USB device.
    pub fn from_transport(transport: Box<dyn Transport>) -> Self {
        let usb = transport.info().clone();
        Self::new(usb, Some(transport))
    }

    /// Bit k set for each enabled channel k; all channels if none is.
    fn enabled_mask(&self) -> u64 {
        let mask = self.channels.iter().filter(|c| c.enabled).fold(0u64, |m, c| m | (1 << c.index));
        if mask == 0 { ALL_CHANNELS } else { mask }
    }

    /// Maps trigger stages onto the two hardware trigger levels. Level
    /// matches form a pattern, edge matches an edge mask. With both
    /// present in a stage the level fires on an edge while the pattern
    /// is true.
    fn convert_trigger(&self) -> Result<([TriggerLevel; 2], u8)> {
        let mut levels = [TriggerLevel::default(); 2];
        let Some(trigger) = &self.trigger else {
            return Ok((levels, COMBINE_NONE));
        };
        if trigger.stages.is_empty() {
            return Ok((levels, COMBINE_NONE));
        }
        trigger.check(&FEATURES)?;
        let ddr = !self.settings.external_clock && self.settings.rate.ddr;
        for (stage, level) in trigger.stages.iter().zip(levels.iter_mut()) {
            for cond in &stage.conditions {
                let Some(ch) = self.channels.get(cond.channel) else { continue };
                if !ch.enabled {
                    continue;
                }
                let bit = 1u64 << ch.index;
                match cond.matches {
                    TriggerMatch::Zero => level.pattern_mask |= bit,
                    TriggerMatch::One => {
                        level.pattern_mask |= bit;
                        level.pattern_value |= bit;
                    }
                    TriggerMatch::Rising => level.rising_mask |= bit,
                    TriggerMatch::Falling => level.falling_mask |= bit,
                    TriggerMatch::Edge => {
                        level.rising_mask |= bit;
                        level.falling_mask |= bit;
                    }
                    other => {
                        return Err(Error::Arg(format!(
                            "Unsupported trigger match '{}'.",
                            other.code()
                        )));
                    }
                }
            }
            if let Some(r) = &stage.range {
                // The hardware weighs channel k as 2^k, so the group must
                // be ascending with its lowest channel least significant.
                if !r.channels.is_sorted() {
                    return Err(Error::arg(
                        "Trigger value range channels must be in ascending order.",
                    ));
                }
                for (k, &index) in r.channels.iter().enumerate() {
                    let ch = self.channels.get(index).filter(|c| c.enabled).ok_or_else(|| {
                        Error::Arg(format!("Trigger value range channel {index} is not enabled."))
                    })?;
                    let bit = 1u64 << ch.index;
                    level.value_mask |= bit;
                    if r.low >> k & 1 != 0 {
                        level.value_low |= bit;
                    }
                    if r.high >> k & 1 != 0 {
                        level.value_high |= bit;
                    }
                }
                level.value_inside = r.inside;
            }
            let count = stage.count.map(|c| (c.count, c.consecutive));
            if let Some((n, _)) = count
                && u64::from(n) > COUNTER_MAX + 1
            {
                return Err(Error::Arg(format!(
                    "Trigger count {n} is above the hardware limit of {}.",
                    COUNTER_MAX + 1
                )));
            }
            let duration = match stage.duration {
                Some(TriggerDuration::Longer(n)) => {
                    // The counter ticks at 250 MHz in DDR mode, every
                    // second sample.
                    let ticks = if ddr { n.div_ceil(2) } else { n };
                    if ticks > COUNTER_MAX {
                        return Err(Error::Arg(format!(
                            "Trigger duration {n} is above the hardware limit of {} samples.",
                            if ddr { COUNTER_MAX * 2 } else { COUNTER_MAX }
                        )));
                    }
                    Some(ticks as u32)
                }
                None => None,
            };
            level.finish(count, duration);
        }
        let combine = match (trigger.stages.len(), trigger.combine) {
            (1, _) => COMBINE_A,
            (_, TriggerCombine::Sequence) => COMBINE_A_THEN_B,
            (_, TriggerCombine::Or) => COMBINE_A_OR_B,
            (_, TriggerCombine::And) => COMBINE_A_AND_B,
        };
        Ok((levels, combine))
    }

    /// Parses `invert=` values: `none`, a hex mask or channel names.
    fn parse_invert(&self, text: &str) -> Result<u64> {
        let text = text.trim();
        if text.eq_ignore_ascii_case("none") || text.is_empty() {
            return Ok(0);
        }
        if let Some(hex) = text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
            return u64::from_str_radix(hex, 16)
                .ok()
                .filter(|m| m & !ALL_CHANNELS == 0)
                .ok_or_else(|| Error::Arg(format!("Invalid invert mask '{text}'.")));
        }
        let mut mask = 0;
        for name in text.split(',') {
            let ch = self
                .channels
                .iter()
                .find(|c| c.name == name.trim())
                .ok_or_else(|| Error::Arg(format!("Invalid channel '{name}'.")))?;
            mask |= 1 << ch.index;
        }
        Ok(mask)
    }

    /// The inverted inputs as channel names, or `none`.
    fn format_invert(&self) -> String {
        let names: Vec<&str> = self
            .channels
            .iter()
            .filter(|c| self.settings.invert_mask >> c.index & 1 != 0)
            .map(|c| c.name.as_str())
            .collect();
        if names.is_empty() { "none".to_owned() } else { names.join(",") }
    }

    /// Measures the frequency on the selected input. The FPGA must hold
    /// an image and the threshold must be set for the inputs to count.
    fn measure_frequency(&self) -> Result<u64> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let mut guard = self.link.lock().expect("link lock");
        let link = guard.as_mut().ok_or_else(|| Error::device("device is not open"))?;
        let program = self.program()?;
        link.configure_fpga(program.image())?;
        link.set_threshold(self.settings.threshold)?;
        link.measure_frequency(self.settings.freq_channel)
    }

    fn program(&self) -> Result<Program> {
        let (trigger, trigger_combine) = self.convert_trigger()?;
        let s = &self.settings;
        Ok(Program {
            rate: s.rate,
            external_clock: s.external_clock,
            clock_source: s.clock_source,
            clock_edge: s.clock_edge,
            qualifier: s.qualifier,
            sample_point: s.sample_point,
            invert_mask: s.invert_mask,
            rle: s.rle,
            compression_mask: self.enabled_mask(),
            capture_ratio: s.capture_ratio,
            threshold: s.threshold,
            trigger,
            trigger_combine,
        })
    }
}

impl Device for LogicPortDevice {
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
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        c.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        name.clone_into(&mut c.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let s = &self.settings;
        Ok(match key {
            Key::Conn => Value::Str(self.usb.conn()),
            Key::Samplerate => Value::U64(s.rate.samplerate),
            Key::LimitSamples => Value::U64(s.limit_samples),
            Key::LimitFrames => Value::U64(s.limit_frames),
            Key::CaptureRatio => Value::U64(s.capture_ratio),
            Key::VoltageThreshold => Value::RangeF64(s.threshold, s.threshold),
            Key::Rle => Value::Bool(s.rle),
            Key::ExternalClock => Value::Bool(s.external_clock),
            Key::ExternalClockSource => Value::Str(CLOCK_SOURCES[s.clock_source].into()),
            Key::ClockEdge => Value::Str(CLOCK_EDGES[s.clock_edge].into()),
            Key::Custom(c) if c.id == KEY_CLOCK_QUALIFIER.id => {
                Value::Str(QUALIFIERS[s.qualifier as usize].into())
            }
            Key::Custom(c) if c.id == KEY_SAMPLE_POINT.id => {
                Value::Str(SAMPLE_POINTS[s.sample_point as usize].into())
            }
            Key::Custom(c) if c.id == KEY_INVERT.id => Value::Str(self.format_invert()),
            Key::Custom(c) if c.id == KEY_FREQUENCY_CHANNEL.id => {
                Value::Str(self.channels[s.freq_channel].name.clone())
            }
            Key::Custom(c) if c.id == KEY_FREQUENCY.id => Value::U64(self.measure_frequency()?),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let s = &mut self.settings;
        match key {
            Key::Samplerate => {
                let rate = value
                    .as_u64()
                    .and_then(find_rate)
                    .ok_or_else(|| Error::Arg(format!("unsupported sample rate {value:?}")))?;
                s.rate = *rate;
            }
            Key::LimitSamples => s.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::LimitFrames => s.limit_frames = value.as_u64().ok_or_else(bad)?,
            Key::CaptureRatio => {
                let r = value.as_u64().ok_or_else(bad)?;
                if r > 100 {
                    return Err(Error::arg("capture ratio is a percentage"));
                }
                s.capture_ratio = r;
            }
            Key::VoltageThreshold => {
                let t = match value {
                    Value::RangeF64(lo, hi) => (lo + hi) / 2.0,
                    other => other.as_f64().ok_or_else(bad)?,
                };
                if !(THRESHOLD_MIN..=THRESHOLD_MAX).contains(&t) {
                    return Err(Error::arg("threshold is -6.0 to 6.0 V"));
                }
                s.threshold = t;
            }
            Key::Rle => s.rle = value.as_bool().ok_or_else(bad)?,
            Key::ExternalClock => s.external_clock = value.as_bool().ok_or_else(bad)?,
            Key::ExternalClockSource => {
                let v = value.as_str().ok_or_else(bad)?;
                s.clock_source =
                    CLOCK_SOURCES.iter().position(|c| c.eq_ignore_ascii_case(v)).ok_or_else(bad)?;
            }
            Key::ClockEdge => {
                let v = value.as_str().ok_or_else(bad)?;
                s.clock_edge =
                    CLOCK_EDGES.iter().position(|c| c.eq_ignore_ascii_case(v)).ok_or_else(bad)?;
            }
            Key::Custom(c) if c.id == KEY_CLOCK_QUALIFIER.id => {
                let v = value.as_str().ok_or_else(bad)?;
                s.qualifier = match QUALIFIERS.iter().position(|c| c.eq_ignore_ascii_case(v)) {
                    Some(0) => Qualifier::Off,
                    Some(1) => Qualifier::ActiveHigh,
                    Some(2) => Qualifier::ActiveLow,
                    _ => return Err(bad()),
                };
            }
            Key::Custom(c) if c.id == KEY_SAMPLE_POINT.id => {
                let v = value.as_str().ok_or_else(bad)?;
                s.sample_point = match SAMPLE_POINTS.iter().position(|c| c.eq_ignore_ascii_case(v))
                {
                    Some(0) => SamplePoint::On,
                    Some(1) => SamplePoint::Before,
                    Some(2) => SamplePoint::After,
                    _ => return Err(bad()),
                };
            }
            Key::Custom(c) if c.id == KEY_INVERT.id => {
                let mask = self.parse_invert(value.as_str().ok_or_else(bad)?)?;
                self.settings.invert_mask = mask;
            }
            Key::Custom(c) if c.id == KEY_FREQUENCY_CHANNEL.id => {
                let v = value.as_str().ok_or_else(bad)?;
                let index = self
                    .channels
                    .iter()
                    .position(|c| c.name.eq_ignore_ascii_case(v))
                    .ok_or_else(|| Error::Arg(format!("Invalid channel '{v}'.")))?;
                self.settings.freq_channel = index;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::Samplerate => ValueList::U64(RATES.iter().map(|r| r.samplerate).collect()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::VoltageThreshold => ValueList::RangeStepF64 {
                min: THRESHOLD_MIN,
                max: THRESHOLD_MAX,
                step: THRESHOLD_STEP,
            },
            Key::ExternalClockSource => {
                ValueList::Str(CLOCK_SOURCES.iter().map(|s| (*s).to_owned()).collect())
            }
            Key::ClockEdge => ValueList::Str(CLOCK_EDGES.iter().map(|s| (*s).to_owned()).collect()),
            Key::Custom(c) if c.id == KEY_CLOCK_QUALIFIER.id => {
                ValueList::Str(QUALIFIERS.iter().map(|s| (*s).to_owned()).collect())
            }
            Key::Custom(c) if c.id == KEY_SAMPLE_POINT.id => {
                ValueList::Str(SAMPLE_POINTS.iter().map(|s| (*s).to_owned()).collect())
            }
            Key::Custom(c) if c.id == KEY_FREQUENCY_CHANNEL.id => {
                ValueList::Str(self.channels.iter().map(|c| c.name.clone()).collect())
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let transport = match self.pending.take() {
            Some(t) => t,
            None => crate::usb::open(&self.usb)?,
        };
        let mut link = Link::open(transport)?;
        let _ = link.read_calibration();
        // An unconfigured FPGA (or one left halfway through
        // reconfiguration) doesn't drain the FIFO, so retrying won't
        // help. That's not an error, it gets configured when the
        // acquisition starts.
        if link.resync(false, 1).is_ok() {
            let _ = link.probe_image();
        }
        *self.link.lock().expect("link lock") = Some(link);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(link) = self.link.lock().expect("link lock").take() {
            // The transport is dropped here, which also ends a recording.
            drop(link.into_transport());
        }
        Ok(())
    }

    fn trigger_features(&self) -> TriggerFeatures {
        FEATURES
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&FEATURES)?;
        }
        self.trigger = trigger;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let s = &self.settings;
        if s.rle && !s.external_clock && s.rate.samplerate > RLE_MAX_RATE {
            return Err(Error::Arg(format!(
                "RLE is only available up to {} MHz.",
                RLE_MAX_RATE / 1_000_000
            )));
        }
        let program = self.program()?;
        let acq = Acquisition {
            program,
            limit_samples: s.limit_samples,
            limit_frames: s.limit_frames,
            triggered: self.trigger.as_ref().is_some_and(|t| !t.stages.is_empty()),
        };
        let link = self.link.lock().expect("link lock").take();
        let Some(link) = link else {
            return Err(Error::device("device is not open"));
        };
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("logicport acquisition".into())
            .spawn(move || {
                let mut link = link;
                let outcome = acq.run(&mut link, &sink, &stop2);
                if let Err(e) = &outcome {
                    log::error!("Acquisition failed: {e}.");
                }
                sink.end(outcome);
                *slot.lock().expect("link lock") = Some(link);
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

/// One acquisition, on its own thread.
struct Acquisition {
    program: Program,
    limit_samples: u64,
    limit_frames: u64,
    triggered: bool,
}

/// Feeds samples to the packet sink in chunks.
struct Feed<'a> {
    sink: &'a PacketSink,
    buf: Vec<u8>,
    alive: bool,
}

const FEED_CHUNK: usize = 4096 * UNIT_SIZE;

impl Feed<'_> {
    fn flush(&mut self) {
        if !self.buf.is_empty() {
            let data = std::mem::take(&mut self.buf);
            if !self.sink.send(Packet::logic(UNIT_SIZE, data)) {
                self.alive = false;
            }
        }
    }
}

impl SampleOut for Feed<'_> {
    fn samples(&mut self, sample: u64, count: u64) {
        let bytes = &sample.to_le_bytes()[..UNIT_SIZE];
        let mut left = count;
        while left > 0 && self.alive {
            let room = (FEED_CHUNK - self.buf.len()) / UNIT_SIZE;
            let n = left.min(room as u64) as usize;
            for _ in 0..n {
                self.buf.extend_from_slice(bytes);
            }
            left -= n as u64;
            if self.buf.len() >= FEED_CHUNK {
                self.flush();
            }
        }
    }

    fn trigger(&mut self) {
        self.flush();
        self.sink.send(Packet::Trigger);
    }
}

impl Acquisition {
    /// Loads the FPGA image the settings need, then writes the settings.
    fn prepare(&self, link: &mut Link) -> Result<()> {
        link.configure_fpga(self.program.image())?;
        link.program(&self.program)
    }

    fn run(&self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let mut r = self.prepare(link);
        if let Err(Error::Device(_) | Error::Timeout) = &r {
            // The link may be out of step, resync and try once more.
            log::debug!("Retrying acquisition setup after resync.");
            if link.resync(true, 1).is_ok() {
                let _ = link.probe_image();
            }
            r = self.prepare(link);
        }
        r?;
        let mut frames_done = 0u64;
        loop {
            let mut fill = FillLimits::new(&self.program, self.limit_samples);
            link.start()?;
            log::debug!("Acquisition armed.");
            // The header goes out once the device is armed, so a test
            // bench can start its stimulus when it sees it.
            if frames_done == 0 {
                sink.send(Packet::header());
            }
            let status = loop {
                if stop.load(Ordering::SeqCst) {
                    let _ = link.halt();
                    return Ok(());
                }
                std::thread::sleep(POLL);
                let status = link.status()?;
                if status & STATUS_DONE != 0 {
                    break status;
                }
                fill.check(link, status)?;
            };
            log::debug!("Acquisition done, status 0x{status:02x}.");
            if self.limit_frames > 0 {
                sink.send(Packet::FrameBegin);
            }
            self.read_samples(link, sink, status)?;
            if self.limit_frames > 0 {
                sink.send(Packet::FrameEnd);
            }
            frames_done += 1;
            if frames_done >= self.limit_frames.max(1) {
                return Ok(());
            }
        }
    }

    fn read_samples(&self, link: &mut Link, sink: &PacketSink, status: u8) -> Result<()> {
        let mut limit = self.limit_samples;
        if self.program.rle && limit == 0 {
            limit = RLE_DEFAULT_LIMIT;
        }
        let mut feed = Feed { sink, buf: Vec::with_capacity(FEED_CHUNK), alive: true };
        let sent;
        {
            let mut s = Sink::new(&mut feed, limit, self.program.capture_ratio);
            let extra = self.program.trigger_delay_extra();
            if self.program.ddr() {
                link.read_ddr(status)?.decode(&mut s, self.triggered, extra);
            } else {
                link.read_sdr(status)?.decode(&mut s, self.triggered, extra);
            }
            sent = s.sent;
        }
        feed.flush();
        log::debug!("Sent {sent} samples.");
        Ok(())
    }
}

/// With compression, memory fills with transitions rather than samples,
/// so an idle input never completes a capture. The pre-fill and
/// post-fill phases are bounded in time instead, derived from the sample
/// limit.
struct FillLimits {
    prefill: Option<Duration>,
    postfill: Duration,
    start: Instant,
    trigger_seen: Option<Instant>,
    prefill_ended: bool,
    postfill_ended: bool,
}

impl FillLimits {
    fn new(p: &Program, limit_samples: u64) -> Self {
        let mut prefill = None;
        let mut postfill = Duration::ZERO;
        if p.rle && !p.external_clock && limit_samples > 0 {
            let pre = limit_samples * p.capture_ratio / 100;
            let post = limit_samples - pre;
            let us = |n: u64| Duration::from_micros((n * 1_000_000 / p.rate.samplerate).max(1));
            prefill = Some(us(pre));
            postfill = us(post);
        }
        FillLimits {
            prefill,
            postfill,
            start: Instant::now(),
            trigger_seen: None,
            prefill_ended: false,
            postfill_ended: false,
        }
    }

    fn check(&mut self, link: &mut Link, status: u8) -> Result<()> {
        let Some(prefill) = self.prefill else { return Ok(()) };
        let now = Instant::now();
        if status & STATUS_PREFILLED == 0 && !self.prefill_ended && now - self.start >= prefill {
            log::debug!("Pre-fill time elapsed.");
            link.end_prefill()?;
            self.prefill_ended = true;
        }
        if status & STATUS_TRIGGERED != 0 && self.trigger_seen.is_none() {
            self.trigger_seen = Some(now);
        }
        if let Some(seen) = self.trigger_seen
            && !self.postfill_ended
            && now - seen >= self.postfill
        {
            log::debug!("Post-fill time elapsed.");
            link.end_postfill()?;
            self.postfill_ended = true;
        }
        Ok(())
    }
}
