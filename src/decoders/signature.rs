// Counterpart in sigrok:
//   libsigrokdecode/decoders/signature/pd.py,
//   Copyright (C) 2019 Shirow Miura <shirowmiura@gmail.com>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the signature analysis rules and the decoder's
//   documented classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - A START edge that closes the gate is only acted on inside
//     `if start != prev_start and not gate_is_open`; closing it again
//     emits nothing, so a level change while the gate is open is
//     ignored.
//   - START and STOP are alternative branches (`elif`): when both
//     change at one clock edge, only START is looked at.
//   - The signature is 16 bits, shifted right with the incoming parity
//     bit in bit 15; the message is the four nibbles low nibble first.
//   - Bit annotations use the data value itself as the annotation
//     class (0 or 1). The START annotation uses class 2.

//! Signature: HP 5004A compatible signature analysis of a clocked line.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

// Channel indices, in declaration order.
const START: usize = 0;
const STOP: usize = 1;
const CLOCK: usize = 2;
const DATA: usize = 3;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
// Bit annotations use the data value itself as the class.
const ANN_START: usize = 2;
const ANN_STOP: usize = 3;
const ANN_SIG: usize = 4;

static OPTIONS: [OptionDesc; 4] = [
    OptionDesc {
        id: "start_edge",
        desc: "START edge polarity",
        default: OptionValue::s("rising"),
        values: &[OptionValue::s("rising"), OptionValue::s("falling")],
    },
    OptionDesc {
        id: "stop_edge",
        desc: "STOP edge polarity",
        default: OptionValue::s("rising"),
        values: &[OptionValue::s("rising"), OptionValue::s("falling")],
    },
    OptionDesc {
        id: "clk_edge",
        desc: "CLOCK edge polarity",
        default: OptionValue::s("falling"),
        values: &[OptionValue::s("rising"), OptionValue::s("falling")],
    },
    OptionDesc {
        id: "annbits",
        desc: "Enable bit level annotations",
        default: OptionValue::s("no"),
        values: &[OptionValue::s("yes"), OptionValue::s("no")],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "signature",
    name: "Signature",
    longname: "Signature analysis",
    desc: "Annotate signature of logic patterns.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Debug/trace", "Util", "Encoding"],
    channels: &[
        ChannelDesc { id: "start", name: "START", desc: "START channel" },
        ChannelDesc { id: "stop", name: "STOP", desc: "STOP channel" },
        ChannelDesc { id: "clk", name: "CLOCK", desc: "CLOCK channel" },
        ChannelDesc { id: "data", name: "DATA", desc: "DATA channel" },
    ],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit0", desc: "Bit0" },
        AnnClass { id: "bit1", desc: "Bit1" },
        AnnClass { id: "start", desc: "START" },
        AnnClass { id: "stop", desc: "STOP" },
        AnnClass { id: "sig", desc: "Sig" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0, 1, 2, 3] },
        AnnRow { id: "sig", desc: "Sig", classes: &[4] },
    ],
    binary: &[],
    doc: "Signature analysis function for troubleshooting logic circuits.\n\
          This generates the same signature as Hewlett-Packard 5004A.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Signature::new())
}

/// The symbol of a four bit signature nibble, as `symbol_map` in pd.py.
fn symbol(nibble: u32) -> char {
    match nibble {
        0b0000 => '0',
        0b1000 => '1',
        0b0100 => '2',
        0b1100 => '3',
        0b0010 => '4',
        0b1010 => '5',
        0b0110 => '6',
        0b1110 => '7',
        0b0001 => '8',
        0b1001 => '9',
        0b0101 => 'A',
        0b1101 => 'C',
        0b0011 => 'F',
        0b1011 => 'H',
        0b0111 => 'P',
        0b1111 => 'U',
        _ => '?',
    }
}

struct Signature {
    start_rising: bool,
    stop_rising: bool,
    clock_edge: CondKind,
    annbits: bool,
}

impl Signature {
    fn new() -> Signature {
        Signature {
            start_rising: true,
            stop_rising: true,
            clock_edge: CondKind::Falling,
            annbits: false,
        }
    }

    /// pd.py `putsig()`: the four nibbles of the signature, low first.
    fn putsig(ctx: &mut dyn Context, ss: u64, es: u64, signature: u32) {
        let s: String = [
            signature & 0x0f,
            (signature >> 4) & 0x0f,
            (signature >> 8) & 0x0f,
            (signature >> 12) & 0x0f,
        ]
        .iter()
        .map(|n| symbol(*n))
        .collect();
        ctx.put_ann(ss, es, ANN_SIG, vec![s]);
    }
}

impl Decoder for Signature {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.start_rising = config.str("start_edge", "rising") == "rising";
        self.stop_rising = config.str("stop_edge", "rising") == "rising";
        self.clock_edge = if config.str("clk_edge", "falling") == "rising" {
            CondKind::Rising
        } else {
            CondKind::Falling
        };
        self.annbits = config.str("annbits", "no") == "yes";
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let start_rising = self.start_rising;
        let stop_rising = self.stop_rising;
        let annbits = self.annbits;
        let mut gate_is_open = false;
        let mut sample_start: Option<u64> = None;
        let mut started = false;
        let mut last_samplenum = 0u64;
        let mut prev_start = u8::from(!start_rising);
        let mut prev_stop = u8::from(!stop_rising);
        let mut shiftreg: u32 = 0;
        loop {
            let pins = ctx.wait(&[&[Cond::pin(CLOCK, self.clock_edge)]])?;
            let start = pins.0[START];
            let stop = pins.0[STOP];
            let data = pins.0[DATA];
            let samplenum = ctx.samplenum();

            if start != prev_start && !gate_is_open {
                // pd.py: gate_is_open = (start == 1) if rising else (start == 0)
                let opens = if start_rising { start == 1 } else { start == 0 };
                gate_is_open = opens;
                if gate_is_open {
                    // Start sampling.
                    sample_start = Some(samplenum);
                    started = true;
                }
            } else if stop != prev_stop && gate_is_open {
                let closes = if stop_rising { stop == 1 } else { stop == 0 };
                gate_is_open = !closes;
                if !gate_is_open {
                    // Stop sampling.
                    if annbits {
                        ctx.put_ann(
                            last_samplenum,
                            samplenum,
                            ANN_STOP,
                            vec!["STOP".into(), "STP".into(), "P".into()],
                        );
                    }
                    let ss = sample_start.unwrap_or(samplenum);
                    Self::putsig(ctx, ss, samplenum, shiftreg);
                    shiftreg = 0;
                    sample_start = None;
                }
            }
            if gate_is_open {
                if annbits {
                    if started {
                        let s = format!("<{data}>");
                        ctx.put_ann(
                            last_samplenum,
                            samplenum,
                            ANN_START,
                            vec![format!("START{s}"), format!("STR{s}"), format!("S{s}")],
                        );
                        started = false;
                    } else {
                        ctx.put_ann(
                            last_samplenum,
                            samplenum,
                            usize::from(data),
                            vec![data.to_string()],
                        );
                    }
                }
                // pd.py: bin(shiftreg & 0b0000_0010_1001_0001).count('1')
                let ones = (shiftreg & 0b0000_0010_1001_0001).count_ones();
                let incoming = (ones + u32::from(data)) & 1;
                shiftreg = (incoming << 15) | (shiftreg >> 1);
            }
            prev_start = start;
            prev_stop = stop;
            last_samplenum = samplenum;
        }
    }
}
