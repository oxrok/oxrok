// Counterpart in sigrok:
//   libsigrokdecode/decoders/parallel/pd.py,
//   Copyright (C) 2013-2016 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the parallel bus protocol and the decoder's
//   documented classes and texts.
#![allow(clippy::redundant_else)]

//! Parallel: items and words sampled on a clock edge or any data edge.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue, Pins,
};

/// The optional clock channel; the data lines follow it.
const CLK: usize = 0;

static OPTIONS: [OptionDesc; 3] = [
    OptionDesc {
        id: "clock_edge",
        desc: "Clock edge to sample on",
        default: OptionValue::s("rising"),
        values: &[OptionValue::s("rising"), OptionValue::s("falling")],
    },
    OptionDesc {
        id: "wordsize",
        desc: "Data wordsize (# bus cycles)",
        default: OptionValue::i(0),
        values: &[],
    },
    OptionDesc {
        id: "endianness",
        desc: "Data endianness",
        default: OptionValue::s("little"),
        values: &[OptionValue::s("little"), OptionValue::s("big")],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "parallel",
    name: "Parallel",
    longname: "Parallel sync bus",
    desc: "Generic parallel synchronous bus.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["parallel"],
    tags: &["Util"],
    channels: &[],
    optional_channels: &[
        ChannelDesc { id: "clk", name: "CLK", desc: "Clock line" },
        ChannelDesc { id: "d0", name: "D0", desc: "Data line 0" },
        ChannelDesc { id: "d1", name: "D1", desc: "Data line 1" },
        ChannelDesc { id: "d2", name: "D2", desc: "Data line 2" },
        ChannelDesc { id: "d3", name: "D3", desc: "Data line 3" },
        ChannelDesc { id: "d4", name: "D4", desc: "Data line 4" },
        ChannelDesc { id: "d5", name: "D5", desc: "Data line 5" },
        ChannelDesc { id: "d6", name: "D6", desc: "Data line 6" },
        ChannelDesc { id: "d7", name: "D7", desc: "Data line 7" },
    ],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "items", desc: "Items" },
        AnnClass { id: "words", desc: "Words" },
    ],
    annotation_rows: &[
        AnnRow { id: "items", desc: "Items", classes: &[0] },
        AnnRow { id: "words", desc: "Words", classes: &[1] },
    ],
    binary: &[],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Parallel::default())
}

/// A non-negative arbitrary-precision value as little-endian bits,
/// which is how the Python decoder's `sum(items[i] << (i * pins))`
/// grows beyond 64 bits.
type Word = Vec<bool>;

/// Renders little-endian `bits` as lowercase hex, zero-padded to at
/// least `width` digits, as Python's `'{{:0{}x}}'.format(...)` does.
fn hex_from_bits(bits: &[bool], width: usize) -> String {
    let ndigits = bits.len().div_ceil(4);
    let mut s = String::with_capacity(ndigits.max(width));
    let mut started = false;
    for d in (0..ndigits).rev() {
        let mut v = 0u8;
        for b in 0..4 {
            if bits.get(d * 4 + b).copied().unwrap_or(false) {
                v |= 1 << b;
            }
        }
        let c = char::from_digit(u32::from(v), 16).expect("hex digit");
        if v != 0 {
            started = true;
        }
        if started {
            s.push(c);
        }
    }
    if !started {
        s.push('0');
    }
    while s.len() < width {
        s.insert(0, '0');
    }
    s
}

/// The value as Python's `py_to_item` keeps it: an `int` while it fits,
/// else big-endian bytes without leading zeros.
fn item_from_bits(bits: &[bool]) -> Item {
    let len = bits.iter().rposition(|&b| b).map_or(0, |i| i + 1);
    if len <= 63 {
        let mut v = 0u64;
        for (i, &b) in bits[..len].iter().enumerate() {
            if b {
                v |= 1 << i;
            }
        }
        Item::Int(v as i64)
    } else {
        let n = len.div_ceil(8);
        let mut bytes = vec![0u8; n];
        for (i, &b) in bits[..len].iter().enumerate() {
            if b {
                bytes[n - 1 - i / 8] |= 1 << (i % 8);
            }
        }
        Item::BigUint(bytes)
    }
}

/// The parallel decoder.
pub struct Parallel {
    /// Items collected for the current word.
    items: Vec<i64>,
    saved_item: Option<i64>,
    ss_item: u64,
    es_item: u64,
    saved_word: Option<Word>,
    ss_word: u64,
    es_word: u64,
    first: bool,
    clock_falling: bool,
    wordsize: i64,
    big_endian: bool,
    /// Width of a data item, i.e. the highest connected channel index.
    item_bits: usize,
    /// Hex digits an item is padded to.
    item_digits: usize,
    /// Hex digits a word is padded to.
    word_digits: usize,
}

impl Default for Parallel {
    fn default() -> Self {
        Parallel {
            items: Vec::new(),
            saved_item: None,
            ss_item: 0,
            es_item: 0,
            saved_word: None,
            ss_word: 0,
            es_word: 0,
            first: true,
            clock_falling: false,
            wordsize: 0,
            big_endian: false,
            item_bits: 0,
            item_digits: 0,
            word_digits: 0,
        }
    }
}

impl Parallel {
    /// `bitpack(bits[1:idx_strip])` over the decoder channels.
    fn sample_item(ctx: &dyn Context, pins: &Pins, max_connected: usize) -> i64 {
        let mut item = 0i64;
        for idx in 1..=max_connected {
            if ctx.has_channel(idx) {
                item |= i64::from(pins.get(idx)) << (idx - 1);
            }
        }
        item
    }

    /// The Python decoder's `handle_bits(item, used_pins)`.
    fn handle_bits(&mut self, ctx: &mut dyn Context, item: i64) {
        // "If a word was previously accumulated, then emit its annotation
        // now after its end samplenumber became available."
        if let Some(word) = self.saved_word.take()
            && self.wordsize > 0
        {
            let s = ctx.samplenum();
            self.es_word = s;
            let text = hex_from_bits(&word, self.word_digits);
            ctx.put_ann(self.ss_word, s, 1, vec![text]);
            // The code emits only the type and value, not the
            // documented bitsize and item count.
            let value = item_from_bits(&word);
            ctx.put_python(self.ss_word, s, Item::List(vec![Item::Str("WORD".into()), value]));
        }

        // "Defer annotations for individual items until the next sample
        // is taken, and the previous sample's end samplenumber has
        // become available."
        let s = ctx.samplenum();
        if self.first {
            // "Save the start sample and item for later (no output yet)."
            self.ss_item = s;
            self.first = false;
            self.saved_item = Some(item);
        } else {
            // "Output the saved item (from the last CLK edge to the
            // current)."
            self.es_item = s;
            let saved = self.saved_item.unwrap_or(0);
            ctx.put_python(
                self.ss_item,
                self.es_item,
                Item::List(vec![Item::Str("ITEM".into()), Item::Int(saved)]),
            );
            let text = format!("{saved:0width$x}", width = self.item_digits);
            ctx.put_ann(self.ss_item, self.es_item, 0, vec![text]);
            self.ss_item = s;
            self.saved_item = Some(item);
        }

        // "Get as many items as the configured wordsize specifies."
        if self.items.is_empty() {
            self.ss_word = s;
        }
        self.items.push(item);
        let ws = self.wordsize;
        if (self.items.len() as i64) < ws {
            return;
        }

        // "Collect words and prepare annotation details, but defer
        // emission until the end samplenumber becomes available."
        let n = usize::try_from(ws).unwrap_or(0);
        let used = self.item_bits;
        let mut vals = self.items.clone();
        if self.big_endian {
            vals.reverse();
        }
        let mut bits = vec![false; n.saturating_mul(used)];
        for (i, &v) in vals.iter().take(n).enumerate() {
            for b in 0..used {
                if (v >> b) & 1 == 1 {
                    bits[i * used + b] = true;
                }
            }
        }
        self.saved_word = Some(bits);
        self.items.clear();
    }
}

impl Decoder for Parallel {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Parallel {
            clock_falling: config.str("clock_edge", "rising") == "falling",
            wordsize: config.int("wordsize", 0),
            big_endian: config.str("endianness", "little") == "big",
            ..Parallel::default()
        };
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // "Determine which (optional) channels have input data. Insist in
        // a non-empty input data set. Cope with sparse connection maps."
        let max_possible = INFO.optional_channels.len();
        let connected: Vec<usize> = (0..max_possible).filter(|&i| ctx.has_channel(i)).collect();
        if connected.is_empty() {
            return Err(Error::arg("At least one channel has to be supplied."));
        }
        let max_connected = *connected.last().expect("connected channels");

        // "Pre-determine which input data to strip off, the width of
        // individual items and multiplexed words, as well as format
        // strings here."
        self.item_bits = max_connected;
        self.item_digits = max_connected.div_ceil(4);
        let word_bits = (max_connected as i64).saturating_mul(self.wordsize.max(0));
        self.word_digits = usize::try_from((word_bits + 3) / 4).unwrap_or(0);

        // "Determine .wait() conditions, depending on the presence of a
        // clock signal. Either inspect samples on the configured edge of
        // the clock, or inspect samples upon ANY edge of ANY of the pins
        // which provide input data."
        if ctx.has_channel(CLK) {
            let kind = if self.clock_falling { CondKind::Falling } else { CondKind::Rising };
            let group = [Cond::pin(CLK, kind)];
            loop {
                let pins = ctx.wait(&[&group])?;
                let item = Self::sample_item(ctx, &pins, max_connected);
                self.handle_bits(ctx, item);
            }
        } else {
            let groups: Vec<[Cond; 1]> =
                connected.iter().map(|&idx| [Cond::pin(idx, CondKind::Edge)]).collect();
            let refs: Vec<&[Cond]> = groups.iter().map(<[Cond; 1]>::as_slice).collect();
            loop {
                let pins = ctx.wait(&refs)?;
                let item = Self::sample_item(ctx, &pins, max_connected);
                self.handle_bits(ctx, item);
            }
        }
    }
}
