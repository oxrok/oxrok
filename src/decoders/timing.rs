// Counterpart in sigrok:
//   libsigrokdecode/decoders/timing/pd.py,
//   Copyright (C) 2014 Torsten Duwe <duwe@suse.de>
//   Copyright (C) 2014 Sebastien Bourdelin
//   <sebastien.bourdelin@savoirfairelinux.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the edge timing measurement and the decoder's
//   documented classes and texts.

//! Timing: reports the time between edges, a moving average, and the
//! delta from the previous interval.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};
use std::collections::VecDeque;

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_TIME: usize = 0;
const ANN_AVERAGE: usize = 1;
const ANN_DELTA: usize = 2;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "timing",
    name: "Timing",
    longname: "Timing calculation with frequency and averaging",
    desc: "Calculate time between edges.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Clock/timing", "Util"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[],
    options: &[
        OptionDesc {
            id: "avg_period",
            desc: "Averaging period",
            default: OptionValue::i(100),
            values: &[],
        },
        OptionDesc {
            id: "edge",
            desc: "Edges to check",
            default: OptionValue::s("any"),
            values: &[OptionValue::s("any"), OptionValue::s("rising"), OptionValue::s("falling")],
        },
        OptionDesc {
            id: "delta",
            desc: "Show delta from last",
            default: OptionValue::s("no"),
            values: &[OptionValue::s("yes"), OptionValue::s("no")],
        },
    ],
    annotations: &[
        AnnClass { id: "time", desc: "Time" },
        AnnClass { id: "average", desc: "Average" },
        AnnClass { id: "delta", desc: "Delta" },
    ],
    annotation_rows: &[
        AnnRow { id: "time", desc: "Time", classes: &[0] },
        AnnRow { id: "average", desc: "Average", classes: &[1] },
        AnnRow { id: "delta", desc: "Delta", classes: &[2] },
    ],
    binary: &[],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Timing::default())
}

/// Python's `sum()` of a series of floats: since CPython 3.12 it adds
/// them with Neumaier's compensation, so the moving averages match the
/// reference decoder in the last digit.
fn py_sum<'a>(values: impl Iterator<Item = &'a f64>) -> f64 {
    let mut f = 0.0f64;
    let mut c = 0.0f64;
    for &item in values {
        let t = f + item;
        if f.abs() >= item.abs() {
            c += (f - t) + item;
        } else {
            c += (item - t) + f;
        }
        f = t;
    }
    if c != 0.0 && c.is_finite() {
        f += c;
    }
    f
}

/// `normalize_time(t)` from the Python decoder.
fn normalize_time(t: f64) -> String {
    if t.abs() >= 1.0 {
        format!("{t:.3} s  ({:.3} Hz)", 1.0 / t)
    } else if t.abs() >= 0.001 {
        if 1.0 / t / 1000.0 < 1.0 {
            format!("{:.3} ms ({:.3} Hz)", t * 1000.0, 1.0 / t)
        } else {
            format!("{:.3} ms ({:.3} kHz)", t * 1000.0, (1.0 / t) / 1000.0)
        }
    } else if t.abs() >= 0.000_001 {
        if 1.0 / t / 1000.0 / 1000.0 < 1.0 {
            format!("{:.3} μs ({:.3} kHz)", t * 1000.0 * 1000.0, (1.0 / t) / 1000.0)
        } else {
            format!("{:.3} μs ({:.3} MHz)", t * 1000.0 * 1000.0, (1.0 / t) / 1000.0 / 1000.0)
        }
    } else if t.abs() >= 0.000_000_001 {
        // The Python decoder's condition here is the truthiness of
        // `1/t/1000/1000/1000`, which is true for every finite non-zero
        // t, so the ns/GHz branch is effectively unreachable.
        let per_second = 1.0 / t / 1000.0 / 1000.0 / 1000.0;
        if per_second == 0.0 {
            format!(
                "{:.3} ns ({:.3} GHz)",
                t * 1000.0 * 1000.0 * 1000.0,
                (1.0 / t) / 1000.0 / 1000.0 / 1000.0
            )
        } else {
            format!(
                "{:.3} ns ({:.3} MHz)",
                t * 1000.0 * 1000.0 * 1000.0,
                (1.0 / t) / 1000.0 / 1000.0
            )
        }
    } else {
        format!("{t:.6}")
    }
}

/// The timing decoder.
#[derive(Default)]
pub struct Timing {
    samplerate: u64,
    last_samplenum: Option<u64>,
    last_n: VecDeque<f64>,
    last_t: Option<f64>,
    edge: Option<CondKind>,
    avg_period: i64,
    delta: bool,
}

impl Decoder for Timing {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.last_samplenum = None;
        self.last_n.clear();
        self.last_t = None;
        self.edge = Some(match config.str("edge", "any") {
            "rising" => CondKind::Rising,
            "falling" => CondKind::Falling,
            _ => CondKind::Edge,
        });
        self.avg_period = config.int("avg_period", 100);
        self.delta = config.str("delta", "no") == "yes";
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }
        let kind = self.edge.unwrap_or(CondKind::Edge);
        loop {
            let _pins = ctx.wait(&[&[Cond::pin(DATA, kind)]])?;
            let now = ctx.samplenum();

            // "if not self.last_samplenum:" -- Python's truthiness makes
            // a first edge at sample 0 take this branch again next time.
            if self.last_samplenum.is_none_or(|n| n == 0) {
                self.last_samplenum = Some(now);
                continue;
            }
            let samples = now - self.last_samplenum.unwrap_or(0);
            let t = samples as f64 / self.samplerate as f64;

            // "if t > 0: self.last_n.append(t)"
            if t > 0.0 {
                self.last_n.push_back(t);
            }
            if self.last_n.len() as i64 > self.avg_period {
                self.last_n.pop_front();
            }

            ctx.put_ann(self.last_samplenum.unwrap_or(0), now, ANN_TIME, vec![normalize_time(t)]);
            if self.avg_period > 0 {
                let average = py_sum(self.last_n.iter()) / self.last_n.len() as f64;
                ctx.put_ann(
                    self.last_samplenum.unwrap_or(0),
                    now,
                    ANN_AVERAGE,
                    vec![normalize_time(average)],
                );
            }
            // "if self.last_t and self.options['delta'] == 'yes':"
            if self.last_t.is_some_and(|last| last != 0.0) && self.delta {
                ctx.put_ann(
                    self.last_samplenum.unwrap_or(0),
                    now,
                    ANN_DELTA,
                    vec![normalize_time(t - self.last_t.unwrap_or(0.0))],
                );
            }

            self.last_t = Some(t);
            self.last_samplenum = Some(now);
        }
    }
}
