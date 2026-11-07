// Counterpart in sigrok:
//   libsigrokdecode/decoders/rc_encode/pd.py,
//   Copyright (C) 2018 Steve R <steversig@virginmedia.com>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the PT2262/HX2262/SC5262 pulse coding and the
//   decoder's documented classes and texts.

//! PT2262/HX2262/SC5262 remote control encoder: 12 tri-state bits and
//! a sync bit.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};
use std::fmt::Write as _;

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT_0: usize = 0;
const ANN_BIT_1: usize = 1;
const ANN_BIT_F: usize = 2;
const ANN_BIT_U: usize = 3;
const ANN_BIT_SYNC: usize = 4;
const ANN_PIN: usize = 5;
const ANN_CODE_WORD_ADDR: usize = 6;
const ANN_CODE_WORD_DATA: usize = 7;

// pd.py: bitvals = ('0', '1', 'f', 'U'); the index of the value in that
// tuple is the annotation class.

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "remote",
    desc: "Remote",
    default: OptionValue::s("none"),
    values: &[OptionValue::s("none"), OptionValue::s("maplin_l95ar")],
}];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "rc_encode",
    name: "RC encode",
    longname: "Remote control encoder",
    desc: "PT2262/HX2262/SC5262 remote control encoder protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["IC", "IR"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit-0", desc: "Bit 0" },
        AnnClass { id: "bit-1", desc: "Bit 1" },
        AnnClass { id: "bit-f", desc: "Bit f" },
        AnnClass { id: "bit-U", desc: "Bit U" },
        AnnClass { id: "bit-sync", desc: "Bit sync" },
        AnnClass { id: "pin", desc: "Pin" },
        AnnClass { id: "code-word-addr", desc: "Code word address" },
        AnnClass { id: "code-word-data", desc: "Code word data" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0, 1, 2, 3, 4] },
        AnnRow { id: "pins", desc: "Pins", classes: &[5] },
        AnnRow { id: "code-words", desc: "Code words", classes: &[6, 7] },
    ],
    binary: &[],
    doc: "This PD decodes the remote control protocol which is frequently used\n\
          within key fobs and power socket remotes.\n\
          \n\
          They contain encoding chips like the PT2262 which converts the button\n\
          pressed and address settings into a series of pulses which is then\n\
          transmitted over whatever frequency and modulation that the designer\n\
          chooses. These devices operate at a number of frequencies including 433MHz.\n\
          \n\
          This PD should also decode the HX2262 and SC5262 which are equivalents.\n\
          \n\
          The decoder also contains some additional decoding for a Maplin L95AR\n\
          remote control and will turn the received signal into which button was\n\
          pressed and what the address code DIP switches are set to.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(RcEncode::new())
}

/// pd.py `decode_bit()`: four pulse widths of one tri-state bit.
///
/// The datasheet says a long pulse is three times a short one; pd.py
/// accepts 2x to 5x and checks the equal pulses within 0.5x to 1.5x.
fn decode_bit(edges: &[i64; 4]) -> &'static str {
    const LMIN: f64 = 2.0;
    const LMAX: f64 = 5.0;
    const EQMIN: f64 = 0.5;
    const EQMAX: f64 = 1.5;
    let e = edges.map(|v| v as f64);
    if e[1] >= e[0] * LMIN
        && e[1] <= e[0] * LMAX
        && e[2] >= e[0] * EQMIN
        && e[2] <= e[0] * EQMAX
        && e[3] >= e[0] * LMIN
        && e[3] <= e[0] * LMAX
    {
        "0"
    } else if e[0] >= e[1] * LMIN
        && e[0] <= e[1] * LMAX
        && e[0] >= e[2] * EQMIN
        && e[0] <= e[2] * EQMAX
        && e[0] >= e[3] * LMIN
        && e[0] <= e[3] * LMAX
    {
        "1"
    } else if e[1] >= e[0] * LMIN
        && e[1] <= e[0] * LMAX
        && e[2] >= e[0] * LMIN
        && e[2] <= e[0] * LMAX
        && e[3] >= e[0] * EQMIN
        && e[3] <= e[0] * EQMAX
    {
        "f"
    } else {
        "U"
    }
}

/// pd.py `pinlabels()`: the pin name of an A/D pin of the PT2262.
fn pinlabels(bit_count: i64) -> String {
    if bit_count <= 6 {
        format!("A{}", bit_count - 1)
    } else {
        format!("A{}/D{}", bit_count - 1, 12 - bit_count)
    }
}

/// One stored bit: its value, start and end sample.
struct StoredBit {
    value: &'static str,
    ss: u64,
    es: u64,
}

/// pd.py `decode_model()` for the Maplin L95AR four-socket remote.
///
/// Returns the address text and its span, then the button text and its
/// span, as pd.py indexes its list.
struct ModelDecode {
    address: String,
    address_ss: u64,
    address_es: u64,
    button: String,
    button_ss: u64,
    button_es: u64,
}

fn decode_model(model: &str, bits: &[StoredBit]) -> Option<ModelDecode> {
    if model != "maplin_l95ar" {
        return None;
    }
    // Address pins A0 to A5.
    let mut address = "Addr".to_owned();
    for (i, bit) in bits.iter().take(6).enumerate() {
        let state = if bit.value == "0" { "on" } else { "off" };
        let _ = write!(address, " {}:{}", i + 1, state);
    }
    // Button pins A6/D5 to A11/D0.
    let mut button = "Button".to_owned();
    if bits[6].value == "0" && bits[11].value == "0" {
        button.push_str(" A ON/OFF");
    } else if bits[7].value == "0" && bits[11].value == "0" {
        button.push_str(" B ON/OFF");
    } else if bits[9].value == "0" && bits[11].value == "0" {
        button.push_str(" C ON/OFF");
    } else if bits[8].value == "0" && bits[11].value == "0" {
        button.push_str(" D ON/OFF");
    } else {
        button.push_str(" Unknown");
    }
    Some(ModelDecode {
        address,
        address_ss: bits[0].ss,
        address_es: bits[5].es,
        button,
        button_ss: bits[6].ss,
        button_es: bits[11].es,
    })
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Decoding,
    DecodeTimeout,
}

struct RcEncode {
    model: String,
    samplenumber_last: Option<u64>,
    pulses: Vec<i64>,
    bits: Vec<StoredBit>,
    bit_count: i64,
    ss: u64,
    es: u64,
    state: State,
}

impl RcEncode {
    fn new() -> RcEncode {
        RcEncode {
            model: "none".to_owned(),
            samplenumber_last: None,
            pulses: Vec::new(),
            bits: Vec::new(),
            bit_count: 0,
            ss: 0,
            es: 0,
            state: State::Idle,
        }
    }

    fn reset(&mut self) {
        self.samplenumber_last = None;
        self.pulses.clear();
        self.bits.clear();
        self.bit_count = 0;
        self.ss = 0;
        self.es = 0;
        self.state = State::Idle;
    }

    fn putx(&mut self, ctx: &mut dyn Context, class: usize, text: String) {
        ctx.put_ann(self.ss, self.es, class, vec![text]);
    }
}

impl Decoder for RcEncode {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        config.str("remote", "none").clone_into(&mut self.model);
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?;
            self.state = State::Decoding;

            // pd.py: if not self.samplenumber_last: ... Python treats 0
            // as unset as well as None.
            if matches!(self.samplenumber_last, None | Some(0)) {
                self.samplenumber_last = Some(ctx.samplenum());
                self.ss = ctx.samplenum();
                continue;
            }

            if self.bit_count < 12 {
                // Decode A0 to A11.
                self.bit_count += 1;
                for i in 0..4 {
                    // Get four pulses for each bit.
                    if i > 0 {
                        ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?;
                    }
                    let samples = (ctx.samplenum() - self.samplenumber_last.unwrap_or(0)) as i64;
                    self.pulses.push(samples);
                    self.samplenumber_last = Some(ctx.samplenum());
                }
                self.es = ctx.samplenum();
                let mut edges = [0i64; 4];
                edges.copy_from_slice(&self.pulses[..4]);
                let value = decode_bit(&edges);
                self.bits.push(StoredBit { value, ss: self.ss, es: self.es });
                let idx = match value {
                    "0" => ANN_BIT_0,
                    "1" => ANN_BIT_1,
                    "f" => ANN_BIT_F,
                    _ => ANN_BIT_U,
                };
                self.putx(ctx, idx, value.to_owned());
                self.putx(ctx, ANN_PIN, pinlabels(self.bit_count));
                self.pulses.clear();
                self.ss = ctx.samplenum();
            } else {
                if self.model != "none"
                    && let Some(labels) = decode_model(&self.model, &self.bits)
                {
                    ctx.put_ann(
                        labels.address_ss,
                        labels.address_es,
                        ANN_CODE_WORD_ADDR,
                        vec![labels.address],
                    );
                    ctx.put_ann(
                        labels.button_ss,
                        labels.button_es,
                        ANN_CODE_WORD_DATA,
                        vec![labels.button],
                    );
                }
                let samples = (ctx.samplenum() - self.samplenumber_last.unwrap_or(0)) as i64;
                // Wait for the end of the sync bit.
                ctx.wait(&[&[Cond::Skip((8 * samples) as u64)]])?;
                self.es = ctx.samplenum();
                self.putx(ctx, ANN_BIT_SYNC, "Sync".to_owned());
                self.reset();
                self.state = State::DecodeTimeout;
            }
            if self.state != State::DecodeTimeout {
                self.samplenumber_last = Some(ctx.samplenum());
            }
        }
    }
}
