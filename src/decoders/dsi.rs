// Counterpart in sigrok:
//   libsigrokdecode/decoders/dsi/pd.py,
//   Copyright (C) 2015 Jeremy Swanson <jeremy@rakocontrols.com>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the biphase coding and the decoder's documented
//   classes and texts.

//! DSI: biphase/manchester lighting control.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Config, Context, Decoder, DecoderDef, DecoderInfo, OptionDesc,
    OptionValue,
};

const DSI: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_STARTBIT: usize = 1;
const ANN_LEVEL: usize = 2;
const ANN_RAW: usize = 3;

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "polarity",
    desc: "Polarity",
    default: OptionValue::s("active-high"),
    values: &[OptionValue::s("active-low"), OptionValue::s("active-high")],
}];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "dsi",
    name: "DSI",
    longname: "Digital Serial Interface",
    desc: "Digital Serial Interface (DSI) lighting protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Embedded/industrial", "Lighting"],
    channels: &[ChannelDesc { id: "dsi", name: "DSI", desc: "DSI data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "startbit", desc: "Start bit" },
        AnnClass { id: "level", desc: "Dimmer level" },
        AnnClass { id: "raw", desc: "Raw data" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "raw", desc: "Raw data", classes: &[3] },
        AnnRow { id: "fields", desc: "Fields", classes: &[1, 2] },
    ],
    binary: &[],
    doc: "DSI is a biphase/manchester based lighting control protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Dsi::new())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Phase0,
    Phase1,
}

/// The DSI decoder.
#[allow(clippy::struct_field_names)]
pub struct Dsi {
    samplerate: u64,
    halfbit: i64,
    edges: Vec<i64>,
    bits: Vec<(i64, u8)>,
    ss_es_bits: Vec<(i64, i64)>,
    state: State,
    phase0: u8,
    old_dsi: u8,
    dsi: u8,
    active_high: bool,
}

impl Dsi {
    fn new() -> Self {
        Dsi {
            samplerate: 0,
            halfbit: 0,
            edges: Vec::new(),
            bits: Vec::new(),
            ss_es_bits: Vec::new(),
            state: State::Idle,
            phase0: 0,
            old_dsi: 0,
            dsi: 0,
            active_high: true,
        }
    }

    fn reset_decoder_state(&mut self) {
        self.edges.clear();
        self.bits.clear();
        self.ss_es_bits.clear();
        self.state = State::Idle;
    }

    fn putb(
        &self,
        ctx: &mut dyn Context,
        bit1: usize,
        bit2: usize,
        class: usize,
        texts: Vec<String>,
    ) {
        let ss = self.ss_es_bits[bit1].0;
        let es = self.ss_es_bits[bit2].1;
        // pd.py's `put` clamps a negative start sample to zero.
        ctx.put_ann(ss.max(0) as u64, es.max(0) as u64, class, texts);
    }

    fn handle_bits(&mut self, ctx: &mut dyn Context, length: usize) {
        let mut f: u32 = 0;
        let mut g: f64 = 0.0;
        // Individual raw bits.
        for i in 0..length {
            let ss = if i == 0 { self.bits[0].0.max(0) } else { self.ss_es_bits[i - 1].1 };
            let es = self.bits[i].0 + self.halfbit * 2;
            self.ss_es_bits.push((ss, es));
            let texts = vec![self.bits[i].1.to_string()];
            self.putb(ctx, i, i, ANN_BIT, texts);
        }
        // Bits[0:0]: Startbit.
        let s = vec![
            format!("Startbit: {}", self.bits[0].1),
            format!("ST: {}", self.bits[0].1),
            "ST".into(),
            "S".into(),
            "S".into(),
        ];
        self.putb(ctx, 0, 0, ANN_STARTBIT, s.clone());
        self.putb(ctx, 0, 0, ANN_RAW, s);
        // Bits[1:8].
        for i in 0..8 {
            f |= u32::from(self.bits[1 + i].1) << (7 - i);
            g = f64::from(f) / 2.55;
        }
        if length == 9 {
            // BACKWARD Frame.
            let s = vec![
                format!("Data: {f:02X}"),
                format!("Dat: {f:02X}"),
                format!("Dat: {f:02X}"),
                format!("D: {f:02X}"),
                "D".into(),
            ];
            self.putb(ctx, 1, 8, ANN_RAW, s);
            let g = g as i64;
            let s = vec![
                format!("Level: {g}%"),
                format!("Lev: {g}%"),
                format!("Lev: {g}%"),
                format!("L: {g}"),
                "D".into(),
            ];
            self.putb(ctx, 1, 8, ANN_LEVEL, s);
        }
    }
}

impl Decoder for Dsi {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.active_high = config.str("polarity", "active-high") == "active-high";
        self.old_dsi = u8::from(config.str("polarity", "active-high") == "active-low");
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        // pd.py: `int((samplerate * 0.0016667) / 2.0)`.
        self.halfbit = (samplerate as f64 * 0.001_666_7 / 2.0) as i64;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::device("Cannot decode without samplerate."));
        }
        loop {
            // pd.py waits for any change of the line, one sample at a time.
            let pins = ctx.wait(&[])?;
            self.dsi = u8::from(pins.get(DSI));
            if self.active_high {
                self.dsi ^= 1;
            }

            if self.state == State::Idle {
                if self.old_dsi == self.dsi {
                    continue;
                }
                let sn = ctx.samplenum() as i64;
                // Add in the first half of the start bit.
                self.edges.push(sn - self.halfbit);
                self.edges.push(sn);
                // Start bit is 0->1.
                self.phase0 = self.dsi ^ 1;
                self.state = State::Phase1;
                self.old_dsi = self.dsi;
                continue;
            }

            let sn = ctx.samplenum() as i64;
            if self.old_dsi != self.dsi {
                self.edges.push(sn);
            } else if sn == *self.edges.last().expect("edges") + (self.halfbit as f64 * 1.5) as i64
            {
                self.edges.push(sn - (self.halfbit as f64 * 0.5) as i64);
            } else {
                continue;
            }

            let bit = self.old_dsi;
            if self.state == State::Phase0 {
                self.phase0 = bit;
                self.state = State::Phase1;
            } else if self.state == State::Phase1 {
                if bit == 1 && self.phase0 == 1 {
                    // Stop bit.
                    if self.bits.len() == 17 || self.bits.len() == 9 {
                        // Forward or Backward.
                        let len = self.bits.len();
                        self.handle_bits(ctx, len);
                    }
                    self.reset_decoder_state(); // Reset upon errors.
                    continue;
                }
                let ss = self.edges[self.edges.len() - 3];
                self.bits.push((ss, bit));
                self.state = State::Phase0;
            }

            self.old_dsi = self.dsi;
        }
    }
}
