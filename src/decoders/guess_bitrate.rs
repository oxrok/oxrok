// Counterpart in sigrok:
//   libsigrokdecode/decoders/guess_bitrate/pd.py,
//   Copyright (C) 2013-2016 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the shortest edge distance heuristic and the
//   decoder's documented classes and texts.

//! Guess bitrate: reports the bitrate implied by the shortest edge
//! distance.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef, DecoderInfo,
};

const DATA: usize = 0;

const ANN_BITRATE: usize = 0;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "guess_bitrate",
    name: "Guess bitrate",
    longname: "Guess bitrate/baudrate",
    desc: "Guess the bitrate/baudrate of a UART (or other) protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Clock/timing", "Util"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[],
    options: &[],
    annotations: &[AnnClass { id: "bitrate", desc: "Bitrate / baudrate" }],
    annotation_rows: &[],
    binary: &[],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(GuessBitrate::default())
}

/// The guess bitrate decoder.
#[derive(Default)]
pub struct GuessBitrate {
    samplerate: u64,
}

impl Decoder for GuessBitrate {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        // The Python decoder's reset() only clears ss_edge; the sample
        // rate arrives through metadata().
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }

        // "Get the first edge on the data line."
        ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?;
        let mut ss_edge = ctx.samplenum();

        // "Get any subsequent edge on the data line. Get the smallest
        // distance between any two transitions, assuming it corresponds
        // to one bit time of the respective bitrate of the input stream.
        // This heuristics keeps getting better for longer captures."
        let mut bitwidth: Option<u64> = None;
        loop {
            ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?;
            let s = ctx.samplenum();
            let b = s - ss_edge;
            if bitwidth.is_none_or(|w| b < w) {
                bitwidth = Some(b);
                // "bitrate = int(float(self.samplerate) / float(b))"
                let bitrate = (self.samplerate as f64 / b as f64) as i64;
                ctx.put_ann(ss_edge, s, ANN_BITRATE, vec![bitrate.to_string()]);
            }
            ss_edge = s;
        }
    }
}
