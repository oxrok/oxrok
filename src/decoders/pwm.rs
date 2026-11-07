// Counterpart in sigrok:
//   libsigrokdecode/decoders/pwm/pd.py,
//   Copyright (C) 2014 Torsten Duwe <duwe@suse.de>
//   Copyright (C) 2014 Sebastien Bourdelin
//   <sebastien.bourdelin@savoirfairelinux.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the duty cycle measurement and the decoder's
//   documented classes and texts.

//! PWM: reports the duty cycle and period of a pulse-width modulated
//! signal.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue,
};

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_DUTY: usize = 0;
const ANN_PERIOD: usize = 1;

const BIN_RAW: usize = 0;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "pwm",
    name: "PWM",
    longname: "Pulse-width modulation",
    desc: "Analog level encoded in duty cycle percentage.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Encoding"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[],
    options: &[OptionDesc {
        id: "polarity",
        desc: "Polarity",
        default: OptionValue::s("active-high"),
        values: &[OptionValue::s("active-low"), OptionValue::s("active-high")],
    }],
    annotations: &[
        AnnClass { id: "duty-cycle", desc: "Duty cycle" },
        AnnClass { id: "period", desc: "Period" },
    ],
    annotation_rows: &[
        AnnRow { id: "duty-cycle", desc: "Duty cycle", classes: &[0] },
        AnnRow { id: "period", desc: "Period", classes: &[1] },
    ],
    binary: &[BinClass { id: "raw", desc: "RAW file" }],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Pwm::default())
}

/// The PWM decoder.
#[derive(Default)]
pub struct Pwm {
    samplerate: u64,
    active_low: bool,
}

impl Pwm {
    /// "Adjust granularity."
    fn period_text(period_t: f64) -> String {
        if period_t == 0.0 || period_t >= 1.0 {
            format!("{period_t:.1} s")
        } else if period_t <= 1e-12 {
            format!("{:.1} fs", period_t * 1e15)
        } else if period_t <= 1e-9 {
            format!("{:.1} ps", period_t * 1e12)
        } else if period_t <= 1e-6 {
            format!("{:.1} ns", period_t * 1e9)
        } else if period_t <= 1e-3 {
            format!("{:.1} μs", period_t * 1e6)
        } else {
            format!("{:.1} ms", period_t * 1e3)
        }
    }
}

impl Decoder for Pwm {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.active_low = config.str("polarity", "active-high") == "active-low";
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }

        let mut num_cycles: i64 = 0;
        let mut average = 0.0f64;

        // "Wait for an 'active' edge (depends on config). This starts
        // the first full period of the inspected signal waveform."
        let start_edge = if self.active_low { CondKind::Falling } else { CondKind::Rising };
        ctx.wait(&[&[Cond::pin(DATA, start_edge)]])?;
        let first_samplenum = ctx.samplenum();

        // "Keep getting samples for the period's middle and terminal
        // edges. At the same time that last sample starts the next
        // period."
        loop {
            let start_samplenum = ctx.samplenum();
            ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?;
            let end_samplenum = ctx.samplenum();
            ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?;
            let es_block = ctx.samplenum();
            let ss_block = start_samplenum;

            // "Calculate the period, the duty cycle, and its ratio."
            let period = es_block - start_samplenum;
            let duty = end_samplenum - start_samplenum;
            let ratio = duty as f64 / period as f64;

            // "Report the duty cycle in percent."
            let percent = ratio * 100.0;
            ctx.put_ann(ss_block, es_block, ANN_DUTY, vec![format!("{percent:.6}%")]);

            // "Report the duty cycle in the binary output."
            // bytes([int(ratio * 256)])
            ctx.put_binary(ss_block, es_block, BIN_RAW, vec![(ratio * 256.0) as u8]);

            // "Report the period in units of time."
            let period_t = period as f64 / self.samplerate as f64;
            ctx.put_ann(ss_block, es_block, ANN_PERIOD, vec![Self::period_text(period_t)]);

            // "Update and report the new duty cycle average."
            num_cycles += 1;
            average += percent;
            ctx.put_meta(
                first_samplenum,
                es_block,
                "Average",
                Item::Float(average / num_cycles as f64),
            );
        }
    }
}
