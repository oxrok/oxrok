// Counterpart in sigrok:
//   libsigrokdecode/decoders/wiegand/pd.py,
//   Copyright (C) 2016 Sean Burford <sburford@google.com>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Wiegand interface behaviour and the decoder's
//   documented classes and texts.

//! Wiegand interface: D0/D1 bit stream state and accumulated bits.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Config, Context, Decoder, DecoderDef, DecoderInfo, OptionDesc,
    OptionValue,
};

const D0: usize = 0;
const D1: usize = 1;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BITS: usize = 0;
const ANN_STATE: usize = 1;

static OPTIONS: [OptionDesc; 2] = [
    OptionDesc {
        id: "active",
        desc: "Data lines active level",
        default: OptionValue::s("low"),
        values: &[OptionValue::s("low"), OptionValue::s("high")],
    },
    OptionDesc {
        id: "bitwidth_ms",
        desc: "Single bit width in milliseconds",
        default: OptionValue::i(4),
        values: &[
            OptionValue::i(1),
            OptionValue::i(2),
            OptionValue::i(4),
            OptionValue::i(8),
            OptionValue::i(16),
            OptionValue::i(32),
        ],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "wiegand",
    name: "Wiegand",
    longname: "Wiegand interface",
    desc: "Wiegand interface for electronic entry systems.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Embedded/industrial", "RFID"],
    channels: &[
        ChannelDesc { id: "d0", name: "D0", desc: "Data 0 line" },
        ChannelDesc { id: "d1", name: "D1", desc: "Data 1 line" },
    ],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[AnnClass { id: "bits", desc: "Bits" }, AnnClass { id: "state", desc: "State" }],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Binary value", classes: &[0] },
        AnnRow { id: "state", desc: "Stream state", classes: &[1] },
    ],
    binary: &[],
    doc: "The Wiegand interface is a de facto wiring standard commonly used to connect\n\
          a card swipe mechanism to the rest of an electronic entry system.\n\
          \n\
          Details:\n\
          https://en.wikipedia.org/wiki/Wiegand_interface",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Wiegand::new())
}

/// The stream state, `None` before the first change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Data,
    Invalid,
}

/// The Wiegand decoder.
pub struct Wiegand {
    samplerate: Option<u64>,
    samples_per_bit: u64,
    d0_prev: Option<bool>,
    d1_prev: Option<bool>,
    state: Option<State>,
    ss_state: u64,
    ss_bit: u64,
    es_bit: Option<u64>,
    bit: Option<bool>,
    bits: Vec<bool>,
    active: bool,
    inactive: bool,
    /// The `bitwidth_ms` option, in milliseconds.
    bitwidth: u64,
}

impl Wiegand {
    fn new() -> Self {
        Wiegand {
            samplerate: None,
            // pd.py: self._samples_per_bit = 10
            samples_per_bit: 10,
            d0_prev: None,
            d1_prev: None,
            state: None,
            ss_state: 0,
            ss_bit: 0,
            es_bit: None,
            bit: None,
            bits: Vec::new(),
            active: false,
            inactive: true,
            bitwidth: 4,
        }
    }

    fn update_state(&mut self, ctx: &mut dyn Context, state: Option<State>, bit: Option<bool>) {
        let now = ctx.samplenum();
        // if self._bit is not None:
        //     self._bits.append(self._bit)
        //     self.put(self.ss_bit, self.samplenum, self.out_ann, [0, [str(self._bit)]])
        if let Some(bit) = self.bit {
            self.bits.push(bit);
            ctx.put_ann(self.ss_bit, now, ANN_BITS, vec![if bit { "1" } else { "0" }.to_owned()]);
        }
        self.bit = bit;
        self.ss_bit = now;
        // if bit is not None: self.es_bit = self.samplenum + self._samples_per_bit
        // else: self.es_bit = None
        self.es_bit = if bit.is_some() { Some(now + self.samples_per_bit) } else { None };

        if state != self.state {
            let ann = match self.state {
                Some(State::Data) => {
                    let accum_bits: String =
                        self.bits.iter().map(|&b| if b { '1' } else { '0' }).collect();
                    Some(vec![
                        format!("{} bits {}", self.bits.len(), accum_bits),
                        format!("{} bits", self.bits.len()),
                    ])
                }
                Some(State::Invalid) => Some(vec!["invalid".to_owned()]),
                _ => None,
            };
            if let Some(texts) = ann {
                ctx.put_ann(self.ss_state, now, ANN_STATE, texts);
            }
            self.ss_state = now;
            self.state = state;
            self.bits.clear();
        }
    }
}

impl Decoder for Wiegand {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        // self._active = 1 if self.options['active'] == 'high' else 0
        self.active = config.str("active", "low") == "high";
        self.inactive = !self.active;
        self.bitwidth = config.int("bitwidth_ms", 4).max(0) as u64;
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = Some(samplerate);
        if samplerate != 0 {
            // ms_per_sample = 1000 * (1.0 / self.samplerate)
            // ms_per_bit = float(self.options['bitwidth_ms'])
            // self._samples_per_bit = int(max(1, int(ms_per_bit / ms_per_sample)))
            let ms_per_sample = 1000.0 * (1.0 / samplerate as f64);
            let ms_per_bit = self.bitwidth as f64;
            let samples = (ms_per_bit / ms_per_sample) as i64;
            self.samples_per_bit = samples.max(1) as u64;
        }
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate.is_none() {
            return Err(Error::device("Cannot decode without samplerate."));
        }
        loop {
            // (d0, d1) = self.wait()
            let pins = ctx.wait(&[])?;
            let d0 = pins.get(D0);
            let d1 = pins.get(D1);

            if Some(d0) == self.d0_prev && Some(d1) == self.d1_prev {
                if let Some(es_bit) = self.es_bit
                    && ctx.samplenum() >= es_bit
                {
                    if d0 == self.inactive && d1 == self.inactive {
                        self.update_state(ctx, Some(State::Idle), None);
                    } else {
                        self.update_state(ctx, Some(State::Invalid), None);
                    }
                }
                continue;
            }

            match self.state {
                None | Some(State::Idle | State::Data) => {
                    if d0 == self.active && d1 == self.inactive {
                        self.update_state(ctx, Some(State::Data), Some(false));
                    } else if d0 == self.inactive && d1 == self.active {
                        self.update_state(ctx, Some(State::Data), Some(true));
                    } else if d0 == self.active && d1 == self.active {
                        self.update_state(ctx, Some(State::Invalid), None);
                    }
                }
                Some(State::Invalid) => {
                    // Wait until we see an idle state before leaving invalid.
                    // This prevents inverted lines from being misread.
                    if d0 == self.inactive && d1 == self.inactive {
                        self.update_state(ctx, Some(State::Idle), None);
                    }
                }
            }

            self.d0_prev = Some(d0);
            self.d1_prev = Some(d1);
        }
    }
}
