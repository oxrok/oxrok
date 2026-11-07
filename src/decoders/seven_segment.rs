// Counterpart in sigrok:
//   libsigrokdecode/decoders/seven_segment/pd.py,
//   Copyright (C) 2019 Benedikt Otto <benedikt_o@web.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the segment encoding and the decoder's documented
//   classes and texts.

//! 7-segment display: decodes the seven segment lines into hex digits.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_DIGIT: usize = 0;

/// Segment patterns in the order (a, b, c, d, e, f, g).
const DIGITS: [([bool; 7], char); 17] = [
    ([false, false, false, false, false, false, false], ' '),
    ([true, true, true, true, true, true, false], '0'),
    ([false, true, true, false, false, false, false], '1'),
    ([true, true, false, true, true, false, true], '2'),
    ([true, true, true, true, false, false, true], '3'),
    ([false, true, true, false, false, true, true], '4'),
    ([true, false, true, true, false, true, true], '5'),
    ([true, false, true, true, true, true, true], '6'),
    ([true, true, true, false, false, false, false], '7'),
    ([true, true, true, true, true, true, true], '8'),
    ([true, true, true, true, false, true, true], '9'),
    ([true, true, true, false, true, true, true], 'A'),
    ([false, false, true, true, true, true, true], 'B'),
    ([true, false, false, true, true, true, false], 'C'),
    ([false, true, true, true, true, false, true], 'D'),
    ([true, false, false, true, true, true, true], 'E'),
    ([true, false, false, false, true, true, true], 'F'),
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "seven_segment",
    name: "7-segment",
    longname: "7-segment display",
    desc: "7-segment display protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Display"],
    channels: &[
        ChannelDesc { id: "a", name: "A", desc: "Segment A" },
        ChannelDesc { id: "b", name: "B", desc: "Segment B" },
        ChannelDesc { id: "c", name: "C", desc: "Segment C" },
        ChannelDesc { id: "d", name: "D", desc: "Segment D" },
        ChannelDesc { id: "e", name: "E", desc: "Segment E" },
        ChannelDesc { id: "f", name: "F", desc: "Segment F" },
        ChannelDesc { id: "g", name: "G", desc: "Segment G" },
    ],
    optional_channels: &[ChannelDesc { id: "dp", name: "DP", desc: "Decimal point" }],
    options: &[OptionDesc {
        id: "polarity",
        desc: "Expected polarity",
        default: OptionValue::s("common-cathode"),
        values: &[OptionValue::s("common-cathode"), OptionValue::s("common-anode")],
    }],
    annotations: &[AnnClass { id: "decoded-digit", desc: "Decoded digit" }],
    annotation_rows: &[AnnRow { id: "decoded-digits", desc: "Decoded digits", classes: &[0] }],
    binary: &[],
    doc: "This decoder decodes the output of a 7-segment display.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(SevenSegment::default())
}

/// The 7-segment decoder.
#[derive(Default)]
pub struct SevenSegment {
    common_anode: bool,
}

impl SevenSegment {
    /// The character a segment pattern shows, if it is a known one.
    ///
    /// The Python decoder maps the tuple of seven pin levels through a
    /// dict; patterns that are not listed produce no annotation.
    fn digit(pins: &[bool]) -> Option<char> {
        DIGITS.iter().find(|(pattern, _)| pattern[..] == *pins).map(|(_, c)| *c)
    }
}

impl Decoder for SevenSegment {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.common_anode = config.str("polarity", "common-cathode") == "common-anode";
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // The Python decoder waits once without conditions, which in the
        // host examines the first sample in place.
        let mut oldpins = ctx.wait(&[])?.0;

        // "Check if at least the 7 signals are present." An unassigned
        // channel reads as 0xff, which is not a level.
        if oldpins[..7].iter().any(|&p| p > 1) {
            return Err(Error::arg("7 or 8 pins have to be present."));
        }

        let mut lastpos = ctx.samplenum();
        let have_dp = ctx.has_channel(7);

        // "Wait for any change" on each segment, and the decimal point
        // when it is connected.
        let mut groups: Vec<Vec<Cond>> =
            (0..7).map(|i| vec![Cond::pin(i, CondKind::Edge)]).collect();
        if have_dp {
            groups.push(vec![Cond::pin(7, CondKind::Edge)]);
        }
        let conds: Vec<&[Cond]> = groups.iter().map(Vec::as_slice).collect();

        loop {
            let pins = ctx.wait(&conds)?.0;

            // "Invert all data lines if a common anode display is used."
            // Only the previous pin levels are inverted, and the Python
            // decoder trims them to seven entries when there is no DP.
            let mut levels = Vec::with_capacity(8);
            let count = if have_dp { oldpins.len() } else { 7 };
            for &state in &oldpins[..count] {
                levels.push(if self.common_anode { state == 0 } else { state == 1 });
            }

            if let Some(digit) = Self::digit(&levels[..7]) {
                // "Check if decimal point is present and active." The
                // decimal point is inverted with the segments above.
                let mut text = digit.to_string();
                if have_dp && levels[7] {
                    text.push('.');
                }
                ctx.put_ann(lastpos, ctx.samplenum(), ANN_DIGIT, vec![text]);
            }

            lastpos = ctx.samplenum();
            oldpins = pins;
        }
    }
}
