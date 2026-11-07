// Counterpart in sigrok:
//   libsigrokdecode/decoders/miller/pd.py,
//   Copyright (C) 2017 Christoph Rackwitz <christoph.rackwitz@rwth-aachen.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Miller encoding rules and the decoder's
//   documented classes and texts.

//! Miller encoding: edge timed bits, LSB first, four bits to a group.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_BITSTRING: usize = 1;
const BIN_RAW: usize = 0;

static OPTIONS: [OptionDesc; 2] = [
    OptionDesc { id: "baudrate", desc: "Baud rate", default: OptionValue::i(106_000), values: &[] },
    OptionDesc {
        id: "edge",
        desc: "Edge",
        default: OptionValue::s("falling"),
        values: &[OptionValue::s("rising"), OptionValue::s("falling"), OptionValue::s("either")],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "miller",
    name: "Miller",
    longname: "Miller encoding",
    desc: "Miller encoding protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Encoding"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data signal" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "bitstring", desc: "Bitstring" },
    ],
    annotation_rows: &[
        AnnRow { id: "bit", desc: "Bit", classes: &[0] },
        AnnRow { id: "bitstring", desc: "Bitstring", classes: &[1] },
    ],
    binary: &[BinClass { id: "raw", desc: "Raw binary" }],
    doc: "The Miller protocol decoder supports (modified) Miller encoded data.\n\
          \n\
          E.g. used in NFC communication at 106 kbaud.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Miller::new())
}

/// Python's `round()`, which rounds exact halves to even.
#[allow(clippy::float_cmp)]
fn py_round(x: f64) -> f64 {
    if !x.is_finite() {
        return x;
    }
    let t = x.trunc();
    if (x - t).abs() == 0.5 { if t % 2.0 == 0.0 { t } else { t + x.signum() } } else { x.round() }
}

/// pd.py `roundto()`: round to a multiple of `k`.
fn roundto(x: f64, k: f64) -> f64 {
    py_round(x / k) * k
}

/// One entry of the bit stream `decode_bits()` yields.
#[derive(Clone, Copy)]
enum Bit {
    /// A decoded value, with the span of the unit it covers.
    Value(u8, f64, f64),
    /// An invalid timing, with the span to mark as an error.
    Error(f64, f64),
    /// End of the run: flush the bits accumulated so far.
    Flush,
}

/// The bit string `decode_run()` assembles, LSB first.
#[derive(Default)]
struct Run {
    numbits: usize,
    bitvalue: Vec<u8>,
    bitstring: String,
    stringstart: Option<u64>,
    stringend: Option<u64>,
}

impl Run {
    fn push(&mut self, bit: Bit, ctx: &mut dyn Context) -> bool {
        match bit {
            Bit::Value(value, ss, es) => {
                let (ss, es) = (ss as u64, es as u64);
                ctx.put_ann(ss, es, ANN_BIT, vec![value.to_string()]);
                if self.stringstart.is_none() {
                    self.stringstart = Some(ss);
                }
                self.stringend = Some(es);
                if value != 0 {
                    let byte = self.numbits / 8;
                    if self.bitvalue.len() <= byte {
                        self.bitvalue.resize(byte + 1, 0);
                    }
                    self.bitvalue[byte] |= 1 << (self.numbits % 8);
                }
                self.numbits += 1;
                self.bitstring.push(if value != 0 { '1' } else { '0' });
                if self.numbits.is_multiple_of(4) {
                    self.bitstring.push(' ');
                }
                false
            }
            Bit::Error(ss, es) => {
                ctx.put_ann(ss as u64, es as u64, ANN_BITSTRING, vec!["ERROR".to_owned()]);
                self.numbits = 0;
                true
            }
            Bit::Flush => true,
        }
    }

    fn finish(self, ctx: &mut dyn Context) {
        if self.numbits == 0 {
            return;
        }
        let (Some(ss), Some(es)) = (self.stringstart, self.stringend) else { return };
        ctx.put_ann(ss, es, ANN_BITSTRING, vec![self.bitstring]);
        // numbytes = numbits // 8 + (numbits % 8 > 0)
        let numbytes = self.numbits / 8 + usize::from(!self.numbits.is_multiple_of(8));
        let mut bytes = self.bitvalue;
        bytes.resize(numbytes, 0);
        ctx.put_binary(ss, es, BIN_RAW, bytes);
    }
}

struct Miller {
    samplerate: u64,
    baudrate: i64,
    edge: CondKind,
}

impl Miller {
    fn new() -> Miller {
        Miller { samplerate: 0, baudrate: 106_000, edge: CondKind::Falling }
    }

    /// One run of `decode_bits()` and the `decode_run()` consumer. The
    /// generator is recreated for each run, as pd.py calls
    /// `self.decode_bits()` afresh in every `decode_run()`.
    #[allow(clippy::float_cmp)]
    fn decode_run(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let timeunit = self.samplerate as f64 / self.baudrate as f64;
        let edgetype = self.edge;

        // self.wait({0: edgetype}) # first symbol, beginning of unit
        ctx.wait(&[&[Cond::pin(DATA, edgetype)]])?;
        let mut prevedge = ctx.samplenum();

        let mut run = Run::default();

        // start of message: '0'
        let mut prevbit = 0u8;
        if run.push(Bit::Value(0, prevedge as f64, prevedge as f64 + timeunit), ctx) {
            run.finish(ctx);
            return Ok(());
        }
        let mut expectedstart = ctx.samplenum() as f64 + timeunit;

        // End of message: '0' followed by one idle symbol.
        let skip = (3.0 * timeunit) as u64;
        loop {
            ctx.wait(&[&[Cond::pin(DATA, edgetype)], &[Cond::Skip(skip)]])?;
            let sampledelta = ctx.samplenum() - prevedge;
            prevedge = ctx.samplenum();
            let timedelta = roundto(sampledelta as f64 / timeunit, 0.5);
            let now = ctx.samplenum() as f64;

            if prevbit == 0 {
                // space -> ???
                if timedelta == 1.0 {
                    // 1.0 units -> space
                    if run.push(Bit::Value(0, now, now + timeunit), ctx) {
                        break;
                    }
                    prevbit = 0;
                    expectedstart = now + timeunit;
                } else if timedelta == 1.5 {
                    // 1.5 units -> mark
                    if run.push(Bit::Value(1, expectedstart, now + 0.5 * timeunit), ctx) {
                        break;
                    }
                    prevbit = 1;
                    expectedstart = now + 0.5 * timeunit;
                } else if timedelta >= 2.0 {
                    // idle symbol (end of message)
                    if run.push(Bit::Flush, ctx) {
                        break;
                    }
                } else {
                    // assert timedelta >= 2.0
                    run.push(Bit::Error(now - sampledelta as f64, now), ctx);
                    break;
                }
            } else {
                // mark -> ???
                if timedelta <= 0.5 {
                    run.push(Bit::Error(now - sampledelta as f64, now), ctx);
                    break;
                }
                if timedelta == 1.0 {
                    // 1.0 units -> mark again (1.5 from start)
                    if run.push(Bit::Value(1, expectedstart, now + 0.5 * timeunit), ctx) {
                        break;
                    }
                    prevbit = 1;
                    expectedstart = now + 0.5 * timeunit;
                } else if timedelta == 1.5 {
                    // 1.5 units -> space (no pulse) and space (pulse)
                    if run.push(Bit::Value(0, expectedstart, now), ctx) {
                        break;
                    }
                    if run.push(Bit::Value(0, now, now + timeunit), ctx) {
                        break;
                    }
                    prevbit = 0;
                    expectedstart = now + timeunit;
                } else if timedelta == 2.0 {
                    // 2.0 units -> space (no pulse) and mark (pulse)
                    if run.push(Bit::Value(0, expectedstart, expectedstart + timeunit), ctx) {
                        break;
                    }
                    if run.push(Bit::Value(1, now - 0.5 * timeunit, now + 0.5 * timeunit), ctx) {
                        break;
                    }
                    prevbit = 1;
                    expectedstart = now + 0.5 * timeunit;
                } else {
                    // longer -> space and end of message
                    if run.push(Bit::Value(0, expectedstart, expectedstart + timeunit), ctx) {
                        break;
                    }
                    if run.push(Bit::Flush, ctx) {
                        break;
                    }
                }
            }
        }

        run.finish(ctx);
        Ok(())
    }
}

impl Decoder for Miller {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.baudrate = config.int("baudrate", 106_000);
        // pd.py: edgetype = self.options['edge'][0]
        self.edge = match config.str("edge", "falling").as_bytes().first() {
            Some(b'r') => CondKind::Rising,
            Some(b'e') => CondKind::Edge,
            _ => CondKind::Falling,
        };
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            self.decode_run(ctx)?;
        }
    }
}
