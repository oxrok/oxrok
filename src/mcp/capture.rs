// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Captures kept in memory, and what is computed over them: an agent
//! asks for transitions, statistics and measurements, not for samples.

use crate::core::{AnalogData, Channel, ChannelKind, DeviceInfo, Packet};
use serde_json::{Value, json};
use std::collections::VecDeque;
use std::time::SystemTime;

/// A block of analog samples as the device sent it, with where it
/// starts in each of its channels' sample sequences.
#[derive(Clone, Debug, PartialEq)]
pub struct AnalogBlock {
    /// The index of the block's first sample in its channels' sequences.
    pub at: u64,
    /// The samples.
    pub data: AnalogData,
}

/// A capture: what one acquisition or one file gave.
#[derive(Clone, Debug)]
pub struct Capture {
    /// The id in the store.
    pub id: u64,
    /// Where it came from: a device's driver and model, or a file.
    pub source: String,
    /// The device, when it came from one.
    pub device: Option<DeviceInfo>,
    /// Every channel of the device or file, enabled or not; the logic
    /// data has bit `index` of each enabled logic channel.
    pub channels: Vec<Channel>,
    /// Samples per second, when known.
    pub samplerate: Option<u64>,
    /// Bytes per logic sample.
    pub unit_size: usize,
    /// The logic samples, packed.
    pub logic: Vec<u8>,
    /// The analog blocks, in order.
    pub analog: Vec<AnalogBlock>,
    /// The logic sample the trigger fired at.
    pub trigger_at: Option<u64>,
    /// Frames seen.
    pub frames: u64,
    /// Why the acquisition failed, when it did; what came before is good.
    pub failed: Option<String>,
    /// Whether the acquisition is still going: the data grows.
    pub running: bool,
    /// Why the acquisition ended before its limit, when it did:
    /// `capture_stop`, `timeout`, `cancelled`, `memory budget`, ...
    pub stopped: Option<String>,
    /// The captures evicted from the store to make room for this one.
    pub evicted: Vec<u64>,
    /// When it was made.
    pub created: SystemTime,
}

impl Capture {
    /// An empty capture over `channels`.
    pub fn new(source: impl Into<String>, channels: Vec<Channel>, samplerate: Option<u64>) -> Self {
        Self {
            id: 0,
            source: source.into(),
            device: None,
            channels,
            samplerate,
            unit_size: 0,
            logic: Vec::new(),
            analog: Vec::new(),
            trigger_at: None,
            frames: 0,
            failed: None,
            running: false,
            stopped: None,
            evicted: Vec::new(),
            created: SystemTime::now(),
        }
    }

    /// Takes one packet of the feed.
    pub fn push(&mut self, packet: &Packet) {
        match packet {
            Packet::Meta(items) => {
                for (k, v) in items {
                    if *k == crate::core::Key::Samplerate
                        && let Some(rate) = v.as_u64()
                    {
                        self.samplerate = Some(rate);
                    }
                }
            }
            Packet::Trigger => self.trigger_at = Some(self.logic_samples()),
            Packet::Logic(l) => {
                if self.unit_size == 0 {
                    self.unit_size = l.unit_size;
                }
                if l.unit_size == self.unit_size {
                    self.logic.extend_from_slice(&l.data);
                }
            }
            Packet::Analog(a) => {
                let at = a.channels.first().map_or(0, |c| self.analog_samples(*c));
                self.analog.push(AnalogBlock { at, data: a.clone() });
            }
            Packet::FrameEnd => self.frames += 1,
            Packet::Failed(why) => self.failed = Some(why.clone()),
            _ => {}
        }
    }

    /// How many logic samples.
    pub fn logic_samples(&self) -> u64 {
        self.logic.len().checked_div(self.unit_size).unwrap_or(0) as u64
    }

    /// How many analog samples channel `index` has.
    pub fn analog_samples(&self, index: usize) -> u64 {
        self.analog
            .iter()
            .filter(|b| b.data.channels.contains(&index))
            .map(|b| b.data.len() as u64)
            .sum()
    }

    /// The bytes this capture holds, for the budget.
    pub fn bytes(&self) -> usize {
        self.logic.len() + self.analog.iter().map(|b| b.data.samples.len() * 4).sum::<usize>()
    }

    /// Seconds for a sample index, when the rate is known.
    pub fn time(&self, sample: u64) -> Option<f64> {
        self.samplerate.filter(|r| *r > 0).map(|r| sample as f64 / r as f64)
    }

    /// The sample index for a time, when the rate is known.
    pub fn sample_at(&self, seconds: f64) -> Option<u64> {
        self.samplerate.filter(|r| *r > 0).map(|r| (seconds * r as f64).round().max(0.0) as u64)
    }

    /// The channel called `name`, or at `index` when `name` is a number
    /// no channel is called.
    pub fn channel(&self, name: &str) -> Option<&Channel> {
        self.channels.iter().find(|c| c.name == name).or_else(|| {
            let index: usize = name.parse().ok()?;
            self.channels.iter().find(|c| c.index == index)
        })
    }

    /// The enabled logic channels.
    pub fn logic_channels(&self) -> impl Iterator<Item = &Channel> {
        self.channels.iter().filter(|c| c.enabled && c.kind == ChannelKind::Logic)
    }

    /// The analog channels with samples.
    pub fn analog_channels(&self) -> impl Iterator<Item = &Channel> {
        self.channels
            .iter()
            .filter(|c| c.kind == ChannelKind::Analog && self.analog_samples(c.index) > 0)
    }

    /// The level of logic bit `index` at `sample`.
    pub fn level(&self, index: usize, sample: u64) -> bool {
        let byte = sample as usize * self.unit_size + index / 8;
        self.logic.get(byte).is_some_and(|b| b & (1 << (index % 8)) != 0)
    }

    /// The levels of logic bit `index` over `from..to`.
    pub fn levels(&self, index: usize, from: u64, to: u64) -> impl Iterator<Item = bool> + '_ {
        (from..to.min(self.logic_samples())).map(move |s| self.level(index, s))
    }

    /// The samples of analog channel `index` over `from..to` of its
    /// sequence.
    pub fn analog_values(&self, index: usize, from: u64, to: u64) -> Vec<f32> {
        let mut out = Vec::new();
        for block in &self.analog {
            let Some(col) = block.data.channels.iter().position(|c| *c == index) else { continue };
            let n = block.data.channels.len();
            let len = block.data.len() as u64;
            let (b0, b1) = (from.max(block.at), to.min(block.at + len));
            if b0 >= b1 {
                continue;
            }
            for i in b0 - block.at..b1 - block.at {
                out.push(block.data.samples[i as usize * n + col]);
            }
        }
        out
    }

    /// What the analog channel measures, from its first block.
    pub fn analog_kind(&self, index: usize) -> Option<(&'static str, &'static str)> {
        self.analog
            .iter()
            .find(|b| b.data.channels.contains(&index))
            .map(|b| (b.data.mq.id(), b.data.unit.id()))
    }

    /// Sample positions where logic bit `index` changes in `from..to`:
    /// `(sample, new level)`, after the level at `from`.
    pub fn transitions(&self, index: usize, from: u64, to: u64) -> Vec<(u64, bool)> {
        let mut out = Vec::new();
        let mut prev = None;
        for (i, level) in self.levels(index, from, to).enumerate() {
            if let Some(p) = prev
                && p != level
            {
                out.push((from + i as u64, level));
            }
            prev = Some(level);
        }
        out
    }

    /// The per-channel figures of `capture_summary`.
    pub fn summary(&self) -> Value {
        let logic: Vec<Value> = self
            .logic_channels()
            .map(|c| {
                let stats = logic_stats(self, c.index, 0, self.logic_samples());
                let mut v = json!({"name": c.name, "index": c.index});
                merge(&mut v, &stats);
                v
            })
            .collect();
        let analog: Vec<Value> = self
            .analog_channels()
            .map(|c| {
                let values = self.analog_values(c.index, 0, u64::MAX);
                let (mq, unit) = self.analog_kind(c.index).unwrap_or(("", ""));
                let mut v = json!({"name": c.name, "index": c.index, "mq": mq, "unit": unit});
                merge(&mut v, &analog_stats(&values, self.samplerate));
                v
            })
            .collect();
        json!({
            "capture_id": self.id,
            "source": self.source,
            "samplerate": self.samplerate,
            "samples": self.logic_samples(),
            "duration": self.time(self.logic_samples()),
            "unit_size": self.unit_size,
            "trigger": self.trigger_at.map(|s| json!({"sample": s, "t": self.time(s)})),
            "frames": self.frames,
            "failed": self.failed,
            "running": self.running,
            "stopped": self.stopped,
            "evicted": self.evicted,
            "bytes": self.bytes(),
            "logic": logic,
            "analog": analog,
        })
    }

    /// The one-line entry of `list_captures`.
    pub fn entry(&self) -> Value {
        json!({
            "capture_id": self.id,
            "source": self.source,
            "channels": self.channels.iter().filter(|c| c.enabled).map(|c| c.name.as_str()).collect::<Vec<_>>(),
            "samplerate": self.samplerate,
            "samples": self.logic_samples(),
            "analog_samples": self.analog_channels().map(|c| self.analog_samples(c.index)).max(),
            "bytes": self.bytes(),
            "failed": self.failed,
            "running": self.running,
            "stopped": self.stopped,
            "evicted": self.evicted,
        })
    }
}

fn merge(into: &mut Value, from: &Value) {
    if let (Some(a), Some(b)) = (into.as_object_mut(), from.as_object()) {
        for (k, v) in b {
            a.insert(k.clone(), v.clone());
        }
    }
}

/// Transitions, duty, levels and the shortest pulse of a logic channel.
pub fn logic_stats(capture: &Capture, index: usize, from: u64, to: u64) -> Value {
    let to = to.min(capture.logic_samples());
    let mut n = 0u64;
    let mut high = 0u64;
    let mut first = None;
    let mut last = None;
    let mut prev: Option<bool> = None;
    let mut run_start = from;
    let mut min_pulse: Option<u64> = None;
    let mut edges = 0u64;
    for (i, level) in capture.levels(index, from, to).enumerate() {
        let s = from + i as u64;
        n += 1;
        if level {
            high += 1;
        }
        first.get_or_insert(level);
        last = Some(level);
        if let Some(p) = prev
            && p != level
        {
            edges += 1;
            let run = s - run_start;
            min_pulse = Some(min_pulse.map_or(run, |m| m.min(run)));
            run_start = s;
        }
        prev = Some(level);
    }
    json!({
        "samples": n,
        "transitions": edges,
        "duty": if n == 0 { Value::Null } else { json!(high as f64 / n as f64) },
        "first": first,
        "last": last,
        "min_pulse_samples": min_pulse,
        "min_pulse": min_pulse.and_then(|p| capture.time(p)),
    })
}

/// Min, max, mean and RMS of analog samples.
pub fn analog_stats(values: &[f32], samplerate: Option<u64>) -> Value {
    if values.is_empty() {
        return json!({"samples": 0});
    }
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    let mut sum = 0.0;
    let mut sq = 0.0;
    for &v in values {
        let v = f64::from(v);
        min = min.min(v);
        max = max.max(v);
        sum += v;
        sq += v * v;
    }
    let n = values.len() as f64;
    json!({
        "samples": values.len(),
        "min": min,
        "max": max,
        "mean": sum / n,
        "rms": (sq / n).sqrt(),
        "duration": samplerate.filter(|r| *r > 0).map(|r| n / r as f64),
    })
}

/// What `measure` reports for a logic channel over a range.
pub fn measure_logic(capture: &Capture, index: usize, from: u64, to: u64) -> Value {
    let to = to.min(capture.logic_samples());
    let edges = capture.transitions(index, from, to);
    let rising = edges.iter().filter(|(_, l)| *l).count();
    let falling = edges.len() - rising;
    // Pulse widths: the runs between consecutive edges.
    let mut highs = Vec::new();
    let mut lows = Vec::new();
    for w in edges.windows(2) {
        let width = w[1].0 - w[0].0;
        if w[0].1 { highs.push(width) } else { lows.push(width) }
    }
    // Periods: between consecutive rising edges.
    let risings: Vec<u64> = edges.iter().filter(|(_, l)| *l).map(|(s, _)| *s).collect();
    let periods: Vec<u64> = risings.windows(2).map(|w| w[1] - w[0]).collect();
    let rate = capture.samplerate.filter(|r| *r > 0).map(|r| r as f64);
    let secs = |samples: f64| rate.map(|r| samples / r);
    let stats = |widths: &[u64]| -> Value {
        if widths.is_empty() {
            return Value::Null;
        }
        let min = *widths.iter().min().unwrap_or(&0);
        let max = *widths.iter().max().unwrap_or(&0);
        let mean = widths.iter().sum::<u64>() as f64 / widths.len() as f64;
        json!({
            "count": widths.len(),
            "min_samples": min, "max_samples": max, "mean_samples": mean,
            "min": secs(min as f64), "max": secs(max as f64), "mean": secs(mean),
        })
    };
    let mean_period = if periods.is_empty() {
        None
    } else {
        Some(periods.iter().sum::<u64>() as f64 / periods.len() as f64)
    };
    // Duty: high time over complete periods (rising edge to rising edge);
    // the fraction of samples high when there is no complete period.
    let duty = {
        let mut high = 0u64;
        let mut span = 0u64;
        // Edges alternate, so after a rising edge at `k` the falling one
        // is `k + 1` and the next rising one `k + 2`: one pass.
        for (k, &(rise, level)) in edges.iter().enumerate() {
            if !level {
                continue;
            }
            if let (Some(&(fall, _)), Some(&(next, _))) = (edges.get(k + 1), edges.get(k + 2)) {
                high += fall - rise;
                span += next - rise;
            }
        }
        if span > 0 {
            Some(high as f64 / span as f64)
        } else if to > from {
            let n = capture.levels(index, from, to).filter(|l| *l).count();
            Some(n as f64 / (to - from) as f64)
        } else {
            None
        }
    };
    json!({
        "from": from,
        "to": to,
        "samples": to.saturating_sub(from),
        "transitions": edges.len(),
        "rising": rising,
        "falling": falling,
        "period_samples": mean_period,
        "period": mean_period.and_then(secs),
        "frequency": mean_period.and_then(|p| rate.map(|r| r / p)),
        "duty": duty,
        "high": stats(&highs),
        "low": stats(&lows),
        "level_at_from": if from < to { Some(capture.level(index, from)) } else { None },
    })
}

/// What `measure` reports for an analog channel over a range.
pub fn measure_analog(capture: &Capture, index: usize, from: u64, to: u64) -> Value {
    let values = capture.analog_values(index, from, to);
    let mut v = analog_stats(&values, capture.samplerate);
    if let Some(mean) = v.get("mean").and_then(Value::as_f64) {
        // Frequency by crossings of the mean, rising.
        let mut crossings = Vec::new();
        for (i, w) in values.windows(2).enumerate() {
            if f64::from(w[0]) < mean && f64::from(w[1]) >= mean {
                crossings.push(i);
            }
        }
        let freq = if crossings.len() >= 2 {
            let span = (crossings[crossings.len() - 1] - crossings[0]) as f64;
            let cycles = (crossings.len() - 1) as f64;
            capture.samplerate.filter(|r| *r > 0).map(|r| r as f64 * cycles / span)
        } else {
            None
        };
        let vpp = v["max"].as_f64().unwrap_or(0.0) - v["min"].as_f64().unwrap_or(0.0);
        merge(
            &mut v,
            &json!({
                "vpp": vpp,
                "frequency": freq,
                "cycles": crossings.len().saturating_sub(1),
            }),
        );
    }
    let (mq, unit) = capture.analog_kind(index).unwrap_or(("", ""));
    merge(
        &mut v,
        &json!({"from": from, "to": to.min(from + values.len() as u64), "mq": mq, "unit": unit}),
    );
    v
}

/// Min, max and mean per bucket of an analog channel over `from..to`.
pub fn analog_buckets(
    capture: &Capture,
    index: usize,
    from: u64,
    to: u64,
    buckets: usize,
) -> Vec<Value> {
    let values = capture.analog_values(index, from, to);
    if values.is_empty() || buckets == 0 {
        return Vec::new();
    }
    let per = values.len().div_ceil(buckets);
    values
        .chunks(per)
        .enumerate()
        .map(|(i, chunk)| {
            let s = analog_stats(chunk, None);
            let start = from + (i * per) as u64;
            json!({
                "sample": start,
                "t": capture.time(start),
                "samples": chunk.len(),
                "min": s["min"],
                "max": s["max"],
                "mean": s["mean"],
            })
        })
        .collect()
}

/// What `find` looks for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Find {
    /// Any edge.
    Edge,
    /// A rising edge.
    Rising,
    /// A falling edge.
    Falling,
    /// A pulse (high or low run) at least this many samples long, found
    /// at its start.
    PulseAtLeast(u64),
}

/// The first match of `what` on logic bit `index` at or after `from`:
/// `(sample, level after it)`.
pub fn find(capture: &Capture, index: usize, from: u64, what: Find) -> Option<(u64, bool)> {
    let total = capture.logic_samples();
    if from >= total {
        return None;
    }
    let mut prev = capture.level(index, from);
    let mut run_start = from;
    for s in from + 1..total {
        let level = capture.level(index, s);
        if level == prev {
            continue;
        }
        match what {
            Find::Edge => return Some((s, level)),
            Find::Rising if level => return Some((s, level)),
            Find::Falling if !level => return Some((s, level)),
            Find::PulseAtLeast(width) if s - run_start >= width && run_start > from => {
                return Some((run_start, prev));
            }
            _ => {}
        }
        prev = level;
        run_start = s;
    }
    if let Find::PulseAtLeast(width) = what
        && total - run_start >= width
        && run_start > from
    {
        return Some((run_start, prev));
    }
    None
}

/// The captures a host keeps, under a byte budget: when a new one does
/// not fit, the oldest go. A running capture is never evicted; it is
/// stopped instead when it grows past the budget alone.
pub struct CaptureStore {
    captures: VecDeque<Capture>,
    next_id: u64,
    budget: usize,
}

impl CaptureStore {
    /// An empty store of `budget` bytes.
    pub fn new(budget: usize) -> Self {
        Self { captures: VecDeque::new(), next_id: 1, budget }
    }

    /// Bytes held.
    pub fn used(&self) -> usize {
        self.captures.iter().map(Capture::bytes).sum()
    }

    /// The budget.
    pub fn budget(&self) -> usize {
        self.budget
    }

    /// Adds a capture, giving it its id; returns the id and the ids
    /// evicted to make room. A capture larger than the whole budget is
    /// kept alone.
    pub fn add(&mut self, mut capture: Capture) -> (u64, Vec<u64>) {
        let id = self.next_id;
        self.next_id += 1;
        capture.id = id;
        let evicted = self.make_room(capture.bytes(), id);
        capture.evicted.clone_from(&evicted);
        self.captures.push_back(capture);
        (id, evicted)
    }

    /// Evicts the oldest finished captures, other than `keep`, until
    /// `extra` more bytes fit; returns the ids evicted. What is left may
    /// still not fit when only running captures remain.
    pub fn make_room(&mut self, extra: usize, keep: u64) -> Vec<u64> {
        let mut evicted = Vec::new();
        while self.used() + extra > self.budget {
            let Some(pos) = self.captures.iter().position(|c| !c.running && c.id != keep) else {
                break;
            };
            if let Some(old) = self.captures.remove(pos) {
                evicted.push(old.id);
            }
        }
        evicted
    }

    /// Whether the store holds more than its budget.
    pub fn over_budget(&self) -> bool {
        self.used() > self.budget
    }

    /// The capture with `id`.
    pub fn get(&self, id: u64) -> Option<&Capture> {
        self.captures.iter().find(|c| c.id == id)
    }

    /// The capture with `id`, to feed.
    pub fn get_mut(&mut self, id: u64) -> Option<&mut Capture> {
        self.captures.iter_mut().find(|c| c.id == id)
    }

    /// Removes the capture with `id`.
    pub fn remove(&mut self, id: u64) -> Option<Capture> {
        let pos = self.captures.iter().position(|c| c.id == id)?;
        self.captures.remove(pos)
    }

    /// Every capture, oldest first.
    pub fn iter(&self) -> impl Iterator<Item = &Capture> {
        self.captures.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Mq, MqFlags, Unit};

    /// A capture of one logic channel from a bit string, at 1 kHz.
    fn bits(pattern: &str) -> Capture {
        let mut c =
            Capture::new("test", vec![Channel::new(0, ChannelKind::Logic, "D0")], Some(1000));
        let data: Vec<u8> = pattern.bytes().map(|b| u8::from(b == b'1')).collect();
        c.push(&Packet::logic(1, data));
        c
    }

    #[test]
    fn transitions_and_stats() {
        let c = bits("0011110000111100");
        assert_eq!(c.logic_samples(), 16);
        assert_eq!(c.transitions(0, 0, 16), [(2, true), (6, false), (10, true), (14, false)]);
        let s = logic_stats(&c, 0, 0, 16);
        assert_eq!(s["transitions"], 4);
        assert_eq!(s["duty"], 0.5);
        assert_eq!(s["first"], false);
        assert_eq!(s["min_pulse_samples"], 2);
        assert_eq!(s["min_pulse"], 0.002);
        let m = measure_logic(&c, 0, 0, 16);
        assert_eq!(m["rising"], 2);
        assert_eq!(m["falling"], 2);
        assert_eq!(m["period_samples"], 8.0);
        assert_eq!(m["frequency"], 125.0);
        assert_eq!(m["duty"], 0.5);
        assert_eq!(m["high"]["count"], 2);
        assert_eq!(m["high"]["mean"], 0.004);
        // A sub-range.
        assert_eq!(c.transitions(0, 5, 11), [(6, false), (10, true)]);
        assert_eq!(measure_logic(&c, 0, 5, 11)["transitions"], 2);
    }

    #[test]
    fn finds_edges_and_pulses() {
        let c = bits("0011110000111100");
        assert_eq!(find(&c, 0, 0, Find::Edge), Some((2, true)));
        assert_eq!(find(&c, 0, 2, Find::Edge), Some((6, false)));
        assert_eq!(find(&c, 0, 0, Find::Falling), Some((6, false)));
        assert_eq!(find(&c, 0, 7, Find::Rising), Some((10, true)));
        assert_eq!(find(&c, 0, 14, Find::Edge), None);
        assert_eq!(find(&c, 0, 99, Find::Edge), None);
        // The first run of 4 after the start: the high at 2.
        assert_eq!(find(&c, 0, 0, Find::PulseAtLeast(4)), Some((2, true)));
        assert_eq!(find(&c, 0, 0, Find::PulseAtLeast(5)), None);
        // A final run counts.
        let c = bits("0101000000");
        assert_eq!(find(&c, 0, 0, Find::PulseAtLeast(5)), Some((4, false)));
    }

    #[test]
    fn analog_blocks_and_measures() {
        let mut c = Capture::new(
            "test",
            vec![
                Channel::new(0, ChannelKind::Analog, "A0"),
                Channel::new(1, ChannelKind::Analog, "A1"),
            ],
            Some(8),
        );
        let block = |samples: Vec<f32>| {
            Packet::Analog(AnalogData {
                channels: vec![0, 1],
                samples,
                mq: Mq::Voltage,
                flags: MqFlags::default(),
                unit: Unit::Volt,
                digits: 3,
            })
        };
        // A0: a square wave -1, 1; A1: a ramp.
        c.push(&block(vec![-1.0, 0.0, -1.0, 1.0, 1.0, 2.0, 1.0, 3.0]));
        c.push(&block(vec![-1.0, 4.0, -1.0, 5.0, 1.0, 6.0, 1.0, 7.0]));
        assert_eq!(c.analog_samples(0), 8);
        assert_eq!(c.analog[1].at, 4);
        assert_eq!(c.analog_values(1, 2, 6), [2.0, 3.0, 4.0, 5.0]);
        let m = measure_analog(&c, 0, 0, 8);
        assert_eq!(m["min"], -1.0);
        assert_eq!(m["max"], 1.0);
        assert_eq!(m["vpp"], 2.0);
        assert_eq!(m["mean"], 0.0);
        assert_eq!(m["rms"], 1.0);
        assert_eq!(m["frequency"], 2.0);
        assert_eq!(m["unit"], "V");
        let b = analog_buckets(&c, 1, 0, 8, 2);
        assert_eq!(b.len(), 2);
        assert_eq!(b[1]["min"], 4.0);
        assert_eq!(b[1]["max"], 7.0);
        assert_eq!(b[1]["t"], 0.5);
        assert_eq!(c.bytes(), 64);
        let s = c.summary();
        assert_eq!(s["analog"][0]["name"], "A0");
        assert_eq!(s["analog"][1]["max"], 7.0);
    }

    #[test]
    fn the_store_evicts_the_oldest() {
        let mut store = CaptureStore::new(20);
        let (a, ev) = store.add(bits("00000000"));
        assert_eq!((a, ev), (1, vec![]));
        let (b, ev) = store.add(bits("00000000"));
        assert_eq!((b, ev), (2, vec![]));
        let (c, ev) = store.add(bits("0000000000"));
        assert_eq!((c, ev), (3, vec![1]));
        assert!(store.get(1).is_none());
        assert_eq!(store.iter().map(|c| c.id).collect::<Vec<_>>(), [2, 3]);
        assert!(store.remove(2).is_some());
        assert_eq!(store.used(), 10);
        // Larger than the budget: kept alone.
        let (d, ev) = store.add(bits(&"1".repeat(30)));
        assert_eq!((d, ev), (4, vec![3]));
        assert_eq!(store.used(), 30);
    }

    #[test]
    fn the_trigger_marks_a_sample() {
        let mut c = Capture::new("t", vec![Channel::new(0, ChannelKind::Logic, "D0")], None);
        c.push(&Packet::logic(1, vec![0, 0, 0]));
        c.push(&Packet::Trigger);
        c.push(&Packet::logic(1, vec![1, 1]));
        c.push(&Packet::Meta(vec![(crate::core::Key::Samplerate, 500u64.into())]));
        assert_eq!(c.trigger_at, Some(3));
        assert_eq!(c.samplerate, Some(500));
        assert_eq!(c.time(5), Some(0.01));
        assert_eq!(c.sample_at(0.01), Some(5));
        assert_eq!(c.channel("D0").map(|c| c.index), Some(0));
        assert_eq!(c.channel("0").map(|c| c.index), Some(0));
        assert!(c.channel("D7").is_none());
    }
}
