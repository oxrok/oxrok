// Counterpart in sigrok:
//   libsigrokdecode/decoders/jitter/pd.py,
//   Copyright (C) 2014 Sebastien Bourdelin
//   <sebastien.bourdelin@savoirfairelinux.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the clock/signal state machine and the decoder's
//   documented classes and texts.

//! Jitter: measures the delay between a clock edge and the resulting
//! signal edge, and counts transitions in the wrong state.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue,
};

const CLK: usize = 0;
const SIG: usize = 1;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_JITTER: usize = 0;
const ANN_CLK_MISSED: usize = 1;
const ANN_SIG_MISSED: usize = 2;

const BIN_ASCII_FLOAT: usize = 0;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "jitter",
    name: "Jitter",
    longname: "Timing jitter calculation",
    desc: "Retrieves the timing jitter between two digital signals.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Clock/timing", "Util"],
    channels: &[
        ChannelDesc { id: "clk", name: "Clock", desc: "Clock reference channel" },
        ChannelDesc {
            id: "sig",
            name: "Resulting signal",
            desc: "Resulting signal controlled by the clock",
        },
    ],
    optional_channels: &[],
    options: &[
        OptionDesc {
            id: "clk_polarity",
            desc: "Clock edge polarity",
            default: OptionValue::s("rising"),
            values: &[OptionValue::s("rising"), OptionValue::s("falling"), OptionValue::s("both")],
        },
        OptionDesc {
            id: "sig_polarity",
            desc: "Resulting signal edge polarity",
            default: OptionValue::s("rising"),
            values: &[OptionValue::s("rising"), OptionValue::s("falling"), OptionValue::s("both")],
        },
    ],
    annotations: &[
        AnnClass { id: "jitter", desc: "Jitter value" },
        AnnClass { id: "clk_missed", desc: "Clock missed" },
        AnnClass { id: "sig_missed", desc: "Signal missed" },
    ],
    annotation_rows: &[
        AnnRow { id: "jitter", desc: "Jitter values", classes: &[0] },
        AnnRow { id: "clk_missed", desc: "Clock missed", classes: &[1] },
        AnnRow { id: "sig_missed", desc: "Signal missed", classes: &[2] },
    ],
    binary: &[BinClass {
        id: "ascii-float",
        desc: "Jitter values as newline-separated ASCII floats",
    }],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Jitter::default())
}

/// The helper dictionary `edge_detector` of the Python decoder.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum Polarity {
    #[default]
    Rising,
    Falling,
    Both,
}

impl Polarity {
    fn parse(name: &str) -> Polarity {
        match name {
            "falling" => Polarity::Falling,
            "both" => Polarity::Both,
            _ => Polarity::Rising,
        }
    }

    // 'rising': lambda x, y: bool(not x and y)
    // 'falling': lambda x, y: bool(x and not y)
    // 'both': lambda x, y: bool(x ^ y)
    fn edge(self, old: bool, new: bool) -> bool {
        match self {
            Polarity::Rising => !old && new,
            Polarity::Falling => old && !new,
            Polarity::Both => old != new,
        }
    }
}

/// The state machine's two states.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum State {
    #[default]
    Clk,
    Sig,
}

/// Python's `str()` of a float, which formats the binary output. Rust's
/// `{:?}` prints the same shortest round-trip digits but writes the
/// exponent as `e-5`; Python writes `e-05` and keeps the sign.
fn py_float_str(v: f64) -> String {
    if v.is_nan() {
        return if v.is_sign_negative() { "-nan" } else { "nan" }.to_owned();
    }
    if v.is_infinite() {
        return if v < 0.0 { "-inf" } else { "inf" }.to_owned();
    }
    let s = format!("{v:?}");
    let Some((mantissa, exp)) = s.split_once('e') else { return s };
    let (sign, digits) = match exp.strip_prefix('-') {
        Some(d) => ('-', d),
        None => ('+', exp.strip_prefix('+').unwrap_or(exp)),
    };
    format!("{mantissa}e{sign}{digits:0>2}")
}

/// The jitter decoder.
#[derive(Default)]
pub struct Jitter {
    samplerate: u64,
    state: State,
    oldclk: bool,
    oldsig: bool,
    clk_start: Option<u64>,
    sig_start: Option<u64>,
    clk_missed: i64,
    sig_missed: i64,
    clk_polarity: Polarity,
    sig_polarity: Polarity,
}

impl Jitter {
    /// "Helper function for jitter time annotations."
    fn putx(&self, ctx: &mut dyn Context, delta: f64) {
        // Adjust granularity.
        let delta_s = if delta == 0.0 || delta >= 1.0 {
            format!("{delta:.1}s")
        } else if delta <= 1e-12 {
            format!("{:.1}fs", delta * 1e15)
        } else if delta <= 1e-9 {
            format!("{:.1}ps", delta * 1e12)
        } else if delta <= 1e-6 {
            format!("{:.1}ns", delta * 1e9)
        } else if delta <= 1e-3 {
            format!("{:.1}μs", delta * 1e6)
        } else {
            format!("{:.1}ms", delta * 1e3)
        };
        ctx.put_ann(
            self.clk_start.unwrap_or(0),
            self.sig_start.unwrap_or(0),
            ANN_JITTER,
            vec![delta_s],
        );
    }

    /// "Helper function for ASCII float jitter values (one value per
    /// line)."
    fn putb(&self, ctx: &mut dyn Context, delta: f64) {
        // "Format the delta to an ASCII float value terminated by a
        // newline."
        let mut x = py_float_str(delta).into_bytes();
        x.push(b'\n');
        ctx.put_binary(
            self.clk_start.unwrap_or(0),
            self.sig_start.unwrap_or(0),
            BIN_ASCII_FLOAT,
            x,
        );
    }

    /// The Python decoder's `handle_clk()`; true when this sample is
    /// done.
    fn handle_clk(&mut self, ctx: &mut dyn Context, clk: bool, sig: bool) -> bool {
        let s = ctx.samplenum();
        if self.clk_start == Some(s) {
            // "Clock transition already treated."
            return true;
        }

        if self.clk_polarity.edge(self.oldclk, clk) {
            // "Clock edge found."
            self.clk_start = Some(s);
            self.state = State::Sig;
            return false;
        }

        // "If any transition in the resulting signal occurs while we are
        // waiting for a clock, we increase the missed signal counter."
        if self.sig_start.is_some_and(|v| v != s) && self.sig_polarity.edge(self.oldsig, sig) {
            self.sig_missed += 1;
            ctx.put_meta(s, s, "Signal missed", Item::Int(self.sig_missed));
            ctx.put_ann(s, s, ANN_SIG_MISSED, vec!["Missed signal".into(), "MS".into()]);
        }
        true
    }

    /// The Python decoder's `handle_sig()`; true when this sample is
    /// done.
    fn handle_sig(&mut self, ctx: &mut dyn Context, clk: bool, sig: bool) -> bool {
        let s = ctx.samplenum();
        if self.sig_start == Some(s) {
            // "Signal transition already treated."
            return true;
        }

        if self.sig_polarity.edge(self.oldsig, sig) {
            // "Signal edge found."
            self.sig_start = Some(s);
            self.state = State::Clk;
            // "Calculate and report the timing jitter."
            let delta = (s - self.clk_start.unwrap_or(0)) as f64 / self.samplerate as f64;
            self.putx(ctx, delta);
            self.putb(ctx, delta);
            return false;
        }

        // "If any transition in the clock signal occurs while we are
        // waiting for a resulting signal, we increase the missed clock
        // counter."
        if self.clk_start != Some(s) && self.clk_polarity.edge(self.oldclk, clk) {
            self.clk_missed += 1;
            ctx.put_meta(s, s, "Clock missed", Item::Int(self.clk_missed));
            ctx.put_ann(s, s, ANN_CLK_MISSED, vec!["Missed clock".into(), "MC".into()]);
        }
        true
    }
}

impl Decoder for Jitter {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        // As the Python decoder's reset(), then start().
        self.state = State::Clk;
        self.oldclk = false;
        self.oldsig = false;
        self.clk_start = None;
        self.sig_start = None;
        self.clk_missed = 0;
        self.sig_missed = 0;
        self.clk_polarity = Polarity::parse(config.str("clk_polarity", "rising"));
        self.sig_polarity = Polarity::parse(config.str("sig_polarity", "rising"));
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }
        loop {
            // "Wait for a transition on CLK and/or SIG."
            let pins =
                ctx.wait(&[&[Cond::pin(CLK, CondKind::Edge)], &[Cond::pin(SIG, CondKind::Edge)]])?;
            let clk = pins.get(CLK);
            let sig = pins.get(SIG);

            // "State machine: For each sample we can move 2 steps
            // forward in the state machine."
            loop {
                if self.state == State::Clk && self.handle_clk(ctx, clk, sig) {
                    break;
                }
                if self.state == State::Sig && self.handle_sig(ctx, clk, sig) {
                    break;
                }
            }

            // "Save current CLK/SIG values for the next round."
            self.oldclk = clk;
            self.oldsig = sig;
        }
    }
}
