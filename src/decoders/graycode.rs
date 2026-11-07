// Counterpart in sigrok:
//   libsigrokdecode/decoders/graycode/pd.py,
//   Copyright (C) 2017 Christoph Rackwitz <christoph.rackwitz@rwth-aachen.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Gray code and rotary encoder behaviour and the
//   decoder's documented classes and texts.

//! Gray code and rotary encoder: phase, increment, count, turns, rate.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};
use std::collections::VecDeque;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_PHASE: usize = 0;
const ANN_INCREMENT: usize = 1;
const ANN_COUNT: usize = 2;
const ANN_TURNS: usize = 3;
const ANN_INTERVAL: usize = 4;
const ANN_AVERAGE: usize = 5;
const ANN_RATE: usize = 6;

// pd.py: MAX_CHANNELS = 8 # 10 channels causes some weird problems...
const MAX_CHANNELS: usize = 8;

static OPTIONS: [OptionDesc; 2] = [
    OptionDesc { id: "edges", desc: "Edges per rotation", default: OptionValue::i(0), values: &[] },
    OptionDesc {
        id: "avg_period",
        desc: "Averaging period",
        default: OptionValue::i(10),
        values: &[],
    },
];

static OPTIONAL_CHANNELS: [ChannelDesc; MAX_CHANNELS] = [
    ChannelDesc { id: "d0", name: "D0", desc: "Data line 0" },
    ChannelDesc { id: "d1", name: "D1", desc: "Data line 1" },
    ChannelDesc { id: "d2", name: "D2", desc: "Data line 2" },
    ChannelDesc { id: "d3", name: "D3", desc: "Data line 3" },
    ChannelDesc { id: "d4", name: "D4", desc: "Data line 4" },
    ChannelDesc { id: "d5", name: "D5", desc: "Data line 5" },
    ChannelDesc { id: "d6", name: "D6", desc: "Data line 6" },
    ChannelDesc { id: "d7", name: "D7", desc: "Data line 7" },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "graycode",
    name: "Gray code",
    longname: "Gray code and rotary encoder",
    desc: "Accumulate rotary encoder increments, provide statistics.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Encoding"],
    channels: &[],
    optional_channels: &OPTIONAL_CHANNELS,
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "phase", desc: "Phase" },
        AnnClass { id: "increment", desc: "Increment" },
        AnnClass { id: "count", desc: "Count" },
        AnnClass { id: "turns", desc: "Turns" },
        AnnClass { id: "interval", desc: "Interval" },
        AnnClass { id: "average", desc: "Average" },
        AnnClass { id: "rpm", desc: "Rate" },
    ],
    annotation_rows: &[
        AnnRow { id: "phase", desc: "Phase", classes: &[0] },
        AnnRow { id: "increment", desc: "Increment", classes: &[1] },
        AnnRow { id: "count", desc: "Count", classes: &[2] },
        AnnRow { id: "turns", desc: "Turns", classes: &[3] },
        AnnRow { id: "interval", desc: "Interval", classes: &[4] },
        AnnRow { id: "average", desc: "Average", classes: &[5] },
        AnnRow { id: "rpm", desc: "Rate", classes: &[6] },
    ],
    binary: &[],
    doc: "Gray code and rotary encoder protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(GrayCode::new())
}

/// pd.py `gray_decode()`: the inverse of the Gray-to-binary step.
fn gray_decode(gray: u32) -> u32 {
    let mut temp = gray;
    temp ^= temp >> 8;
    temp ^= temp >> 4;
    temp ^= temp >> 2;
    temp ^= temp >> 1;
    temp
}

/// pd.py `bitpack()`: element 0 is the least significant bit.
fn bitpack(bits: &[bool]) -> u32 {
    bits.iter().enumerate().fold(0, |acc, (i, &b)| acc | (u32::from(b) << i))
}

/// Python's `math.floor` of `p / 3 * 3`, kept in floating point so the
/// prefix lookup sees the same `-9.0` etc. keys.
fn prefix_exponent(p: f64) -> f64 {
    (p / 3.0).floor() * 3.0
}

/// pd.py `prefix_fmt()`: three significant figures, engineering prefix.
///
/// The Python truncates to three significant figures with `math.floor`
/// before formatting, so the result keeps trailing zeros up to
/// `decimals = 2 - int(p)`.
fn prefix_fmt(value: f64, emin: Option<i32>) -> String {
    let sgn = if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    };
    let abs = value.abs();
    let p = if abs == 0.0 { 0.0 } else { abs.log10() };
    // int(3 - p) truncates toward zero.
    let n = (3.0 - p) as i32;
    let scale = 10f64.powi(n);
    // sgn * math.floor(value * 10**int(3 - p)) * 10**-int(3 - p)
    let mut value = sgn * (abs * scale).floor() * 10f64.powi(-n);
    let mut e = prefix_exponent(p);
    if let Some(emin) = emin
        && e < f64::from(emin)
    {
        e = f64::from(emin);
    }
    value *= 10f64.powi(-(e as i32));
    let p = p - e;
    let decimals = (2 - (p as i32)).max(0) as usize;
    let prefix = match e as i32 {
        -9 => "n",
        -6 => "µ",
        -3 => "m",
        3 => "k",
        6 => "M",
        9 => "G",
        _ => "",
    };
    format!("{value:.decimals$} {prefix}")
}

/// Python's `sum()` over floats: since 3.12 CPython uses Neumaier
/// compensated summation, and the rounding decides the printed
/// prefix at a boundary.
fn py_sum(values: impl Iterator<Item = f64>) -> f64 {
    let mut sum = 0.0f64;
    let mut c = 0.0f64;
    for x in values {
        let t = sum + x;
        if sum.abs() >= x.abs() {
            c += (sum - t) + x;
        } else {
            c += (x - t) + sum;
        }
        sum = t;
    }
    if c == 0.0 { sum } else { sum + c }
}

/// pd.py `Value`, whose callbacks fire only when the value changes.
#[derive(Default)]
struct Value {
    timestamp: Option<u64>,
    value: Option<i64>,
}

impl Value {
    fn new(value: i64, timestamp: u64) -> Value {
        Value { timestamp: Some(timestamp), value: Some(value) }
    }
}

struct GrayCode {
    samplerate: u64,
    last_n: VecDeque<(i64, f64)>,
    phase: Value,
    increment: Value,
    count: Value,
    turns: Value,
    encoder_steps: i64,
    edges: i64,
    avg_period: i64,
}

impl GrayCode {
    fn new() -> GrayCode {
        GrayCode {
            samplerate: 0,
            last_n: VecDeque::new(),
            phase: Value::default(),
            increment: Value::default(),
            count: Value::default(),
            turns: Value::default(),
            encoder_steps: 1,
            edges: 0,
            avg_period: 10,
        }
    }

    /// pd.py `Value.set()` for the phase; the callback annotates the
    /// value that just ended.
    fn set_phase(&mut self, ctx: &mut dyn Context, t: u64, newval: i64) {
        if self.phase.value != Some(newval) {
            if let (Some(told), Some(vold)) = (self.phase.timestamp, self.phase.value) {
                ctx.put_ann(told, t, ANN_PHASE, vec![vold.to_string()]);
            }
            self.phase.value = Some(newval);
            self.phase.timestamp = Some(t);
        }
    }

    fn set_increment(&mut self, ctx: &mut dyn Context, t: u64, newval: i64) {
        if self.increment.value != Some(newval) {
            if let (Some(told), Some(vold)) = (self.increment.timestamp, self.increment.value) {
                let message = if vold == 0 {
                    "0".to_owned()
                } else if vold.abs() == self.encoder_steps / 2 {
                    // pd.py prints the plus-minus sign for half a step.
                    "±π".to_owned()
                } else {
                    format!("{vold:+}")
                };
                ctx.put_ann(told, t, ANN_INCREMENT, vec![message]);
            }
            self.increment.value = Some(newval);
            self.increment.timestamp = Some(t);
        }
    }

    fn set_count(&mut self, ctx: &mut dyn Context, t: u64, newval: i64) {
        if self.count.value != Some(newval) {
            if let (Some(told), Some(vold)) = (self.count.timestamp, self.count.value) {
                ctx.put_ann(told, t, ANN_COUNT, vec![vold.to_string()]);
            }
            self.count.value = Some(newval);
            self.count.timestamp = Some(t);
        }
    }

    fn set_turns(&mut self, ctx: &mut dyn Context, t: u64, newval: i64) {
        if self.turns.value != Some(newval) {
            if let (Some(told), Some(vold)) = (self.turns.timestamp, self.turns.value) {
                ctx.put_ann(told, t, ANN_TURNS, vec![format!("{vold:+}")]);
            }
            self.turns.value = Some(newval);
            self.turns.timestamp = Some(t);
        }
    }
}

impl Decoder for GrayCode {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.edges = config.int("edges", 0);
        self.avg_period = config.int("avg_period", 10);
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let num_channels = (0..MAX_CHANNELS).filter(|&i| ctx.has_channel(i)).count();
        // pd.py raises ChannelMapError('Assigned channels need to be
        // contiguous') unless the assigned channels are a prefix.
        for i in 0..MAX_CHANNELS {
            if ctx.has_channel(i) != (i < num_channels) {
                return Err(Error::arg("Assigned channels need to be contiguous"));
            }
        }

        self.encoder_steps = 1i64 << num_channels;

        let startbits = ctx.wait(&[])?;
        let starttime = ctx.samplenum();
        let bits: Vec<bool> = (0..num_channels).map(|i| startbits.get(i)).collect();

        self.turns = Value::new(0, starttime);
        self.count = Value::new(0, starttime);
        let phase = i64::from(gray_decode(bitpack(&bits)));
        self.phase = Value::new(phase, starttime);

        let groups: Vec<Vec<Cond>> =
            (0..num_channels).map(|i| vec![Cond::pin(i, CondKind::Edge)]).collect();
        let conds: Vec<&[Cond]> = groups.iter().map(Vec::as_slice).collect();

        let mut curtime = starttime;
        loop {
            let prevtime = curtime;
            let pins = ctx.wait(&conds)?;
            curtime = ctx.samplenum();

            let oldcount = self.count.value.unwrap_or(0);
            let oldphase = self.phase.value.unwrap_or(0);

            let bits: Vec<bool> = (0..num_channels).map(|i| pins.get(i)).collect();
            let newphase = i64::from(gray_decode(bitpack(&bits)));
            self.set_phase(ctx, curtime, newphase);

            // pd.py: (newphase - oldphase + (steps // 2 - 1)) % steps
            //        - (steps // 2 - 1), with Python's non-negative %.
            let half = self.encoder_steps / 2;
            let phasedelta_raw =
                (newphase - oldphase + (half - 1)).rem_euclid(self.encoder_steps) - (half - 1);
            let mut phasedelta = phasedelta_raw;
            self.set_increment(ctx, curtime, phasedelta);
            if phasedelta.abs() == half {
                phasedelta = 0;
            }

            self.set_count(ctx, curtime, oldcount + phasedelta);

            if self.edges != 0 {
                let turns = self.count.value.unwrap_or(0) / self.edges;
                self.set_turns(ctx, curtime, turns);
            }

            if self.samplerate != 0 {
                let samplerate = self.samplerate as f64;
                let period = (curtime - prevtime) as f64 / samplerate;
                let freq = (phasedelta_raw.abs() as f64) / period;

                // pd.py: '{}s, {}Hz' with the prefix's own unit letter.
                ctx.put_ann(
                    prevtime,
                    curtime,
                    ANN_INTERVAL,
                    vec![format!("{}s, {}Hz", prefix_fmt(period, None), prefix_fmt(freq, None))],
                );

                if self.avg_period != 0 {
                    self.last_n.push_back((phasedelta_raw.abs(), period));
                    if self.last_n.len() > self.avg_period as usize {
                        self.last_n.pop_front();
                    }

                    let sum_v = py_sum(self.last_n.iter().map(|&(_, v)| v));
                    let sum_u: i64 = self.last_n.iter().map(|&(u, _)| u).sum();
                    let avg_period = sum_v / if sum_u != 0 { sum_u as f64 } else { 1.0 };
                    ctx.put_ann(
                        prevtime,
                        curtime,
                        ANN_AVERAGE,
                        vec![format!(
                            "{}s, {}Hz",
                            prefix_fmt(avg_period, None),
                            prefix_fmt(1.0 / avg_period, None)
                        )],
                    );
                }

                if self.edges != 0 {
                    // pd.py passes emin=0 so the prefix is never below 1.
                    let rpm = 60.0 * freq / self.edges as f64;
                    ctx.put_ann(
                        prevtime,
                        curtime,
                        ANN_RATE,
                        vec![format!("{}rpm", prefix_fmt(rpm, Some(0)))],
                    );
                }
            }
        }
    }
}
