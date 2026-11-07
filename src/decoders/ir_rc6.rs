// Counterpart in sigrok:
//   libsigrokdecode/decoders/ir_rc6/pd.py,
//   Copyright (C) 2019 Benedikt Otto <benedikt_o@web.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the RC-6 biphase coding and the decoder's
//   documented classes and texts.

//! IR RC-6: sync and start bits, mode field, address and data, in the
//! standard and mode 6 frame layouts.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const IR: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_SYNC: usize = 1;
const ANN_STARTBIT: usize = 2;
const ANN_FIELD: usize = 3;
const ANN_TOGGLE: usize = 4;
const ANN_ADDRESS: usize = 5;
const ANN_COMMAND: usize = 6;

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "polarity",
    desc: "Polarity",
    default: OptionValue::s("auto"),
    values: &[OptionValue::s("auto"), OptionValue::s("active-low"), OptionValue::s("active-high")],
}];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ir_rc6",
    name: "IR RC-6",
    longname: "IR RC-6",
    desc: "RC-6 infrared remote control protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["IR"],
    channels: &[ChannelDesc { id: "ir", name: "IR", desc: "IR data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "sync", desc: "Sync" },
        AnnClass { id: "startbit", desc: "Startbit" },
        AnnClass { id: "field", desc: "Field" },
        AnnClass { id: "togglebit", desc: "Togglebit" },
        AnnClass { id: "address", desc: "Address" },
        AnnClass { id: "command", desc: "Command" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "fields", desc: "Fields", classes: &[1, 2, 3, 4, 5, 6] },
    ],
    binary: &[],
    doc: "RC-6 is a biphase/manchester based infrared remote control protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(IrRc6::default())
}

/// A decoded bit: start and end sample, width and value.
type Bit = (i64, i64, i64, i64);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Sync,
    Data,
}

/// The IR RC-6 decoder.
pub struct IrRc6 {
    samplerate: u64,
    halfbit: i64,
    edges: Vec<i64>,
    deltas: Vec<i64>,
    bits: Vec<Bit>,
    state: State,
    mode: i64,
    polarity: String,
    // "self.invert = self.ir == 0"
    invert: bool,
}

impl Default for IrRc6 {
    fn default() -> Self {
        IrRc6 {
            samplerate: 0,
            halfbit: 0,
            edges: Vec::new(),
            deltas: Vec::new(),
            bits: Vec::new(),
            state: State::Idle,
            mode: 0,
            polarity: "auto".to_owned(),
            invert: false,
        }
    }
}

impl IrRc6 {
    fn putb(&self, ctx: &mut dyn Context, bit: Bit, class: usize, texts: Vec<String>) {
        ctx.put_ann(bit.0.max(0) as u64, bit.1.max(0) as u64, class, texts);
    }

    fn putbits(
        &self,
        ctx: &mut dyn Context,
        bit1: Bit,
        bit2: Bit,
        class: usize,
        texts: Vec<String>,
    ) {
        ctx.put_ann(bit1.0.max(0) as u64, bit2.1.max(0) as u64, class, texts);
    }

    fn handle_bit(&mut self, ctx: &mut dyn Context) {
        if self.bits.len() != 6 {
            return;
        }
        if self.bits[0].2 == 8 && self.bits[0].3 == 1 {
            self.putb(ctx, self.bits[0], ANN_SYNC, vec!["Synchronisation".into(), "Sync".into()]);
        } else {
            return;
        }
        if self.bits[1].3 == 1 {
            self.putb(ctx, self.bits[1], ANN_STARTBIT, vec!["Startbit".into(), "Start".into()]);
        } else {
            return;
        }
        self.mode = (0..3).fold(0, |m, i| m | (self.bits[2 + i].3 << (2 - i)));
        self.putbits(
            ctx,
            self.bits[2],
            self.bits[4],
            ANN_FIELD,
            vec![format!("Field: {}", self.mode)],
        );
        self.putb(ctx, self.bits[5], ANN_TOGGLE, vec![format!("Toggle: {}", self.bits[5].3)]);
    }

    fn handle_package(&mut self, ctx: &mut dyn Context) {
        // Sync and start bits have to be 1.
        if self.bits[0].3 == 0 || self.bits[1].3 == 0 {
            return;
        }
        if self.bits.len() <= 6 {
            return;
        }

        if self.mode == 0 && self.bits.len() == 22 {
            // Mode 0 standard.
            let value = (0..8).fold(0i64, |v, i| v | (self.bits[6 + i].3 << (7 - i)));
            self.putbits(
                ctx,
                self.bits[6],
                self.bits[13],
                ANN_ADDRESS,
                vec![format!("Address: {value:02X}")],
            );

            let value = (0..8).fold(0i64, |v, i| v | (self.bits[14 + i].3 << (7 - i)));
            self.putbits(
                ctx,
                self.bits[14],
                self.bits[21],
                ANN_COMMAND,
                vec![format!("Data: {value:02X}")],
            );

            self.bits.clear();
        }

        if self.mode == 6 && self.bits.len() >= 15 {
            if self.bits[6].3 == 0 {
                // Short addr, Mode 6A.
                let value = (0..8).fold(0i64, |v, i| v | (self.bits[6 + i].3 << (7 - i)));
                self.putbits(
                    ctx,
                    self.bits[6],
                    self.bits[13],
                    ANN_ADDRESS,
                    vec![format!("Address: {value:02X}")],
                );

                let num_data_bits = self.bits.len() as i64 - 14;
                let value = (0..num_data_bits).fold(0i64, |v, i| {
                    v | (self.bits[14 + i as usize].3 << (num_data_bits - 1 - i))
                });
                self.putbits(
                    ctx,
                    self.bits[14],
                    self.bits[self.bits.len() - 1],
                    ANN_COMMAND,
                    vec![format!("Data: {value:X}")],
                );

                self.bits.clear();
            } else if self.bits.len() >= 23 {
                // Long addr, Mode 6B.
                let value = (0..16).fold(0i64, |v, i| v | (self.bits[6 + i].3 << (15 - i)));
                self.putbits(
                    ctx,
                    self.bits[6],
                    self.bits[21],
                    ANN_ADDRESS,
                    vec![format!("Address: {value:02X}")],
                );

                let num_data_bits = self.bits.len() as i64 - 22;
                let value = (0..num_data_bits).fold(0i64, |v, i| {
                    v | (self.bits[22 + i as usize].3 << (num_data_bits - 1 - i))
                });
                self.putbits(
                    ctx,
                    self.bits[22],
                    self.bits[self.bits.len() - 1],
                    ANN_COMMAND,
                    vec![format!("Data: {value:X}")],
                );

                self.bits.clear();
            }
        }
    }
}

impl Decoder for IrRc6 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.polarity = config.str("polarity", "auto").to_owned();
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        // One bit: 0.889ms (one half low, one half high).
        self.halfbit = ((samplerate as f64 * 0.000_889) / 2.0) as i64;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }
        let mut value = 0i64;
        let mut num_edges: i64 = -1;
        self.invert = false;

        loop {
            let mut conditions: Vec<Vec<Cond>> = vec![vec![Cond::Pin(IR, CondKind::Edge)]];
            if self.state == State::Data {
                conditions.push(vec![Cond::Skip((self.halfbit * 6).max(0) as u64)]);
            }
            let refs: Vec<&[Cond]> = conditions.iter().map(Vec::as_slice).collect();
            let pins = ctx.wait(&refs)?;
            let ir = pins.get(IR);
            let samplenum = ctx.samplenum() as i64;

            if conditions.len() == 2 && ctx.matched()[1] {
                self.state = State::Idle;
            }

            self.edges.push(samplenum);
            if self.edges.len() < 2 {
                continue;
            }

            let delta = (self.edges[self.edges.len() - 1] - self.edges[self.edges.len() - 2])
                as f64
                / self.halfbit as f64;
            let delta = (delta + 0.5) as i64;
            self.deltas.push(delta);

            if self.deltas.len() < 2 {
                continue;
            }

            let dlen = self.deltas.len();
            if self.deltas[dlen - 2..] == [6, 2] {
                self.state = State::Sync;
                num_edges = 0;
                self.bits.clear();

                if self.polarity == "auto" {
                    value = 1;
                } else {
                    value = if self.polarity == "active-high" {
                        i64::from(ir)
                    } else {
                        1 - i64::from(ir)
                    };
                }

                self.bits.push((
                    self.edges[self.edges.len() - 3],
                    self.edges[self.edges.len() - 1],
                    8,
                    value,
                ));
                self.invert = !ir;
                self.putb(ctx, self.bits[self.bits.len() - 1], ANN_BIT, vec![value.to_string()]);
            }

            if num_edges % 2 == 0 {
                // Only count every second edge.
                let dlen = self.deltas.len();
                let d2 = self.deltas[dlen - 2];
                let d1 = self.deltas[dlen - 1];
                if [1, 2, 3].contains(&d2) && [1, 2, 3, 6].contains(&d1) {
                    self.state = State::Data;
                    if d2 != d1 {
                        // Insert border between 2 bits.
                        let elen = self.edges.len();
                        let border = self.edges[elen - 2] + d2 * self.halfbit;
                        self.edges.insert(elen - 1, border);
                        let total = self.deltas[dlen - 1];
                        self.deltas[dlen - 1] = self.deltas[dlen - 2];
                        self.deltas.push(total - self.deltas[dlen - 1]);

                        let elen = self.edges.len();
                        self.bits.push((
                            self.edges[elen - 4],
                            self.edges[elen - 2],
                            self.deltas[dlen - 2] * 2,
                            value,
                        ));

                        num_edges += 1;
                    } else {
                        let elen = self.edges.len();
                        self.bits.push((
                            self.edges[elen - 3],
                            self.edges[elen - 1],
                            self.deltas[self.deltas.len() - 1] * 2,
                            value,
                        ));
                    }

                    self.putb(
                        ctx,
                        self.bits[self.bits.len() - 1],
                        ANN_BIT,
                        vec![value.to_string()],
                    );
                }
            }

            if !self.bits.is_empty() {
                self.handle_bit(ctx);
                if self.state == State::Idle {
                    self.handle_package(ctx);
                }
            }

            value = if self.polarity == "auto" {
                if self.invert { i64::from(ir) } else { 1 - i64::from(ir) }
            } else if self.polarity == "active-low" {
                i64::from(ir)
            } else {
                1 - i64::from(ir)
            };

            num_edges += 1;
        }
    }
}
