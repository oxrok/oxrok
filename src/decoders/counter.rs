// Counterpart in sigrok:
//   libsigrokdecode/decoders/counter/pd.py,
//   Copyright (C) 2018 Stefan Brüns <stefan.bruens@rwth-aachen.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the edge counting behaviour and the decoder's
//   documented classes and texts.

//! Counter: counts edges on a data line and groups them into words.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const DATA: usize = 0;
const RESET: usize = 1;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_EDGE: usize = 0;
const ANN_WORD: usize = 1;
const ANN_RESET: usize = 2;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "counter",
    name: "Counter",
    longname: "Edge counter",
    desc: "Count the number of edges in a signal.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Util"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[ChannelDesc { id: "reset", name: "Reset", desc: "Reset line" }],
    options: &[
        OptionDesc {
            id: "data_edge",
            desc: "Edges to count (data)",
            default: OptionValue::s("any"),
            values: &[OptionValue::s("any"), OptionValue::s("rising"), OptionValue::s("falling")],
        },
        OptionDesc {
            id: "divider",
            desc: "Count divider (word width)",
            default: OptionValue::i(0),
            values: &[],
        },
        OptionDesc {
            id: "reset_edge",
            desc: "Edge which clears counters (reset)",
            default: OptionValue::s("falling"),
            values: &[OptionValue::s("rising"), OptionValue::s("falling")],
        },
        OptionDesc {
            id: "edge_off",
            desc: "Edge counter value after start/reset",
            default: OptionValue::i(0),
            values: &[],
        },
        OptionDesc {
            id: "word_off",
            desc: "Word counter value after start/reset",
            default: OptionValue::i(0),
            values: &[],
        },
        OptionDesc {
            id: "dead_cycles",
            desc: "Ignore this many edges after reset",
            default: OptionValue::i(0),
            values: &[],
        },
        OptionDesc {
            id: "start_with_reset",
            desc: "Assume decode starts with reset",
            default: OptionValue::s("no"),
            values: &[OptionValue::s("no"), OptionValue::s("yes")],
        },
    ],
    annotations: &[
        AnnClass { id: "edge_count", desc: "Edge count" },
        AnnClass { id: "word_count", desc: "Word count" },
        AnnClass { id: "word_reset", desc: "Word reset" },
    ],
    annotation_rows: &[
        AnnRow { id: "edge_counts", desc: "Edges", classes: &[0] },
        AnnRow { id: "word_counts", desc: "Words", classes: &[1] },
        AnnRow { id: "word_resets", desc: "Word resets", classes: &[2] },
    ],
    binary: &[],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Counter::default())
}

/// The edge an option counts.
fn edge_kind(name: &str) -> CondKind {
    match name {
        "rising" => CondKind::Rising,
        "falling" => CondKind::Falling,
        // 'any'
        _ => CondKind::Edge,
    }
}

/// The counter decoder.
#[derive(Default)]
pub struct Counter {
    data_edge: Option<CondKind>,
    reset_edge: Option<CondKind>,
    divider: i64,
    edge_off: i64,
    word_off: i64,
    dead_cycles: i64,
    start_with_reset: bool,
}

impl Decoder for Counter {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        // opt_edge_map = {'rising': 'r', 'falling': 'f', 'any': 'e'}
        self.data_edge = Some(edge_kind(config.str("data_edge", "any")));
        self.reset_edge = Some(edge_kind(config.str("reset_edge", "falling")));
        let divider = config.int("divider", 0);
        // "if divider < 0: divider = 0"
        self.divider = divider.max(0);
        self.edge_off = config.int("edge_off", 0);
        self.word_off = config.int("word_off", 0);
        self.dead_cycles = config.int("dead_cycles", 0);
        self.start_with_reset = config.str("start_with_reset", "no") == "yes";
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let cond_data = [Cond::pin(DATA, self.data_edge.unwrap_or(CondKind::Edge))];
        let have_reset = ctx.has_channel(RESET);
        let cond_reset = [Cond::pin(RESET, self.reset_edge.unwrap_or(CondKind::Falling))];
        let condition: Vec<&[Cond]> =
            if have_reset { vec![&cond_data, &cond_reset] } else { vec![&cond_data] };

        let mut edge_count = self.edge_off;
        let mut edge_start: Option<u64> = None;
        let mut word_count = self.word_off;
        let mut word_start: Option<u64> = None;

        // "if self.options['start_with_reset'] == 'yes':"
        let mut dead_count = if self.start_with_reset { self.dead_cycles } else { 0 };

        loop {
            ctx.wait(&condition)?;
            let now = ctx.samplenum();

            if have_reset && ctx.matched().get(1).copied().unwrap_or(false) {
                edge_count = self.edge_off;
                edge_start = Some(now);
                word_count = self.word_off;
                word_start = Some(now);
                ctx.put_ann(
                    now,
                    now,
                    ANN_RESET,
                    vec!["Word reset".into(), "Reset".into(), "Rst".into(), "R".into()],
                );
                dead_count = self.dead_cycles;
                continue;
            }

            if dead_count != 0 {
                dead_count -= 1;
                edge_start = Some(now);
                word_start = Some(now);
                continue;
            }

            // "In the absence of a RESET condition before the first data
            // edge ... assume that the cycle which corresponds to the
            // counter value started at sample number 0 ... (see bug
            // #1210)."
            if edge_start.is_none() {
                edge_start = Some(0);
            }
            if word_start.is_none() {
                word_start = Some(0);
            }

            edge_count += 1;
            ctx.put_ann(edge_start.unwrap_or(0), now, ANN_EDGE, vec![edge_count.to_string()]);
            edge_start = Some(now);

            let word_edge_count = edge_count - self.edge_off;
            // "if divider and (word_edge_count % divider) == 0:"
            if self.divider != 0 && word_edge_count % self.divider == 0 {
                word_count += 1;
                ctx.put_ann(word_start.unwrap_or(0), now, ANN_WORD, vec![word_count.to_string()]);
                word_start = Some(now);
            }
        }
    }
}
