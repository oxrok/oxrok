// Counterpart in sigrok:
//   libsigrokdecode/decoders/mdio/pd.py,
//   Copyright (C) 2016 Elias Oenal <sigrok@eliasoenal.com>.
// This file is a port of the sigrok counterpart, BSD licensed like it,
//   written from the MII management bus and the decoder's documented classes
//   and texts.
//
// Subtle points of the Python original that this port keeps:
//   - The decoder is clocked on rising MDC edges and reads MDIO at that edge;
//     `mdiobits` is newest first, so `mdiobits[1]` is the bit whose cycle ends
//     now and `mdiobits[32]` is the 32nd previous bit's start.
//   - A preamble shorter than 32 ones is still labelled `PRE #n` but gets a
//     `SHORT PREAMBLE` warning; a longer one is split into `IDLE #n` and
//     `PRE #32`. Only past 10000 extra ones is `mdiobits` trimmed:
//     `del self.mdiobits[33:-1]`.
//   - The last data bit's end sample is a guess: the first quartile (index 12)
//     of the 48 cycle lengths before it, `quartile_cycle_length()`.
//   - Clause 45's address is kept across frames in `clause45_addr` and is
//     incremented after a read-increment (opcode 2); `reset()` keeps it but
//     `reset_decoder_state()` does not touch it.

//! MDIO: the MII management bus between a MAC and a PHY.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Config, Context, Decoder, DecoderDef, DecoderInfo, Item,
    OptionDesc, OptionValue,
};
use std::fmt::Write as _;

const MDC: usize = 0;
const MDIO: usize = 1;

// Annotation classes, as the libsigrokdecode decoder numbers them.
const ANN_BIT_VAL: usize = 0;
const ANN_BIT_NUM: usize = 1;
const ANN_FRAME: usize = 2;
const ANN_FRAME_IDLE: usize = 3;
const ANN_FRAME_ERROR: usize = 4;
const ANN_DECODE: usize = 5;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "mdio",
    name: "MDIO",
    longname: "Management Data Input/Output",
    desc: "MII management bus between MAC and PHY.",
    license: "bsd",
    inputs: &["logic"],
    outputs: &["mdio"],
    tags: &["Networking"],
    channels: &[
        ChannelDesc { id: "mdc", name: "MDC", desc: "Clock" },
        ChannelDesc { id: "mdio", name: "MDIO", desc: "Data" },
    ],
    optional_channels: &[],
    options: &[OptionDesc {
        id: "show_debug_bits",
        desc: "Show debug bits",
        default: OptionValue::s("no"),
        values: &[OptionValue::s("yes"), OptionValue::s("no")],
    }],
    annotations: &[
        AnnClass { id: "bit-val", desc: "Bit value" },
        AnnClass { id: "bit-num", desc: "Bit number" },
        AnnClass { id: "frame", desc: "Frame" },
        AnnClass { id: "frame-idle", desc: "Bus idle state" },
        AnnClass { id: "frame-error", desc: "Frame error" },
        AnnClass { id: "decode", desc: "Decode" },
    ],
    annotation_rows: &[
        AnnRow { id: "bit-val", desc: "Bit value", classes: &[0] },
        AnnRow { id: "bit-num", desc: "Bit number", classes: &[1] },
        AnnRow { id: "frame", desc: "Frame", classes: &[2, 3] },
        AnnRow { id: "frame-error", desc: "Frame error", classes: &[4] },
        AnnRow { id: "decode", desc: "Decode", classes: &[5] },
    ],
    binary: &[],
    doc: "MDIO (Management Data Input/Output) is the two-wire bus a MAC \
uses to read and write a PHY's registers, clocked by MDC. The decoder \
handles Clause 22 and Clause 45 frames: it checks the preamble, decodes \
the start, opcode, port/PHY address, device/register address and data \
fields, flags turnaround and opcode errors, and passes the decoded \
frame to stacked decoders.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Mdio::default())
}

fn texts(t: &[&str]) -> Vec<String> {
    t.iter().map(|s| (*s).to_owned()).collect()
}

/// One bit cycle: value, start sample, end sample once the next edge came.
#[derive(Clone, Copy, Debug)]
struct Bit {
    value: u8,
    ss: u64,
    es: Option<u64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Pre,
    St,
    Op,
    Prtad,
    Devad,
    Ta,
    Data,
}

/// The MDIO decoder.
pub struct Mdio {
    show_debug_bits: bool,
    /// Kept across frames (`reset()`).
    illegal_bus: u8,
    /// Clause 45 is context sensitive; kept across frames.
    clause45_addr: i64,
    /// `reset_decoder_state()`.
    mdiobits: Vec<Bit>,
    bitcount: i64,
    opcode: i64,
    clause45: u8,
    ss_frame: i64,
    ss_frame_field: i64,
    preamble_len: usize,
    ta_invalid: Option<String>,
    ta_seen: bool,
    op_invalid: String,
    portad: i64,
    portad_bits: i64,
    devad: i64,
    devad_bits: i64,
    data: i64,
    data_bits: i64,
    state: State,
    ss_illegal: i64,
}

impl Default for Mdio {
    fn default() -> Self {
        Mdio {
            show_debug_bits: false,
            illegal_bus: 0,
            clause45_addr: -1,
            mdiobits: Vec::new(),
            bitcount: -1,
            opcode: -1,
            clause45: 0,
            ss_frame: -1,
            ss_frame_field: -1,
            preamble_len: 0,
            ta_invalid: None,
            ta_seen: false,
            op_invalid: String::new(),
            portad: -1,
            portad_bits: 5,
            devad: -1,
            devad_bits: 5,
            data: -1,
            data_bits: 16,
            state: State::Pre,
            ss_illegal: -1,
        }
    }
}

impl Mdio {
    fn putbit(&self, ctx: &mut dyn Context, mdio: u8, ss: u64, es: u64) {
        ctx.put_ann(ss, es, ANN_BIT_VAL, vec![mdio.to_string()]);
        if self.show_debug_bits {
            let n = self.bitcount - 1;
            ctx.put_ann(ss, es, ANN_BIT_NUM, vec![n.to_string(), (n % 10).to_string()]);
        }
    }

    fn putff(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        let ss = self.ss_frame_field.max(0) as u64;
        ctx.put_ann(ss, ctx.samplenum(), class, texts);
    }

    fn putdata(&mut self, ctx: &mut dyn Context) {
        let es = self.mdiobits[0].es.expect("bit end");
        let data = self.data;
        ctx.put_ann(
            self.ss_frame_field.max(0) as u64,
            es,
            ANN_FRAME,
            vec![format!("DATA: {data:04X}"), "DATA".to_owned(), "D".to_owned()],
        );

        if self.clause45 != 0 && self.opcode == 0 {
            self.clause45_addr = data;
        }

        // Decode data.
        if self.opcode > 0 || self.clause45 == 0 {
            let mut decoded_min = String::new();
            if self.clause45 != 0 && self.clause45_addr != -1 {
                let _ = write!(decoded_min, "ADDR: {:04X} ", self.clause45_addr);
            } else if self.clause45 != 0 {
                decoded_min.push_str("ADDR: UKWN ");
            }

            let is_read =
                (self.clause45 != 0 && self.opcode > 1) || (self.clause45 == 0 && self.opcode != 0);
            if is_read {
                let _ = write!(decoded_min, "READ:  {data:04X}");
            } else {
                let _ = write!(decoded_min, "WRITE: {data:04X}");
            }

            let mut decoded_ext = String::new();
            let addr_name = if self.clause45 != 0 { "PRTAD" } else { "PHYAD" };
            let _ = write!(decoded_ext, " {addr_name}: {:02}", self.portad);
            let dev_name = if self.clause45 != 0 { "DEVAD" } else { "REGAD" };
            let _ = write!(decoded_ext, " {dev_name}: {:02}", self.devad);
            if self.ta_invalid.is_some() || !self.op_invalid.is_empty() {
                decoded_ext.push_str(" ERROR");
            }

            let ss = self.ss_frame.max(0) as u64;
            ctx.put_ann(
                ss,
                es,
                ANN_DECODE,
                vec![format!("{decoded_min}{decoded_ext}"), decoded_min],
            );

            // The Python decoder passes a list holding one tuple, so the
            // item is nested one level: `[[bool, int, bool, int, int, int]]`.
            ctx.put_python(
                ss,
                es,
                Item::List(vec![Item::List(vec![
                    Item::Bool(self.clause45 != 0),
                    Item::Int(self.clause45_addr),
                    Item::Bool(is_read),
                    Item::Int(self.portad),
                    Item::Int(self.devad),
                    Item::Int(data),
                ])]),
            );
        }

        // Post read increment address.
        if self.clause45 != 0 && self.opcode == 2 && self.clause45_addr != -1 {
            self.clause45_addr += 1;
        }
    }

    fn reset_decoder_state(&mut self) {
        self.mdiobits.clear();
        self.bitcount = -1;
        self.opcode = -1;
        self.clause45 = 0;
        self.ss_frame = -1;
        self.ss_frame_field = -1;
        self.preamble_len = 0;
        self.ta_invalid = None;
        self.ta_seen = false;
        self.op_invalid.clear();
        self.portad = -1;
        self.portad_bits = 5;
        self.devad = -1;
        self.devad_bits = 5;
        self.data = -1;
        self.data_bits = 16;
        self.state = State::Pre;
    }

    fn state_pre(&mut self, ctx: &mut dyn Context, mdio: u8) {
        if self.illegal_bus != 0 {
            if mdio == 0 {
                // Stay in illegal bus state.
                return;
            }
            // Leave and continue parsing.
            self.illegal_bus = 0;
            ctx.put_ann(
                self.ss_illegal.max(0) as u64,
                ctx.samplenum(),
                ANN_FRAME_ERROR,
                texts(&["ILLEGAL BUS STATE", "ILL"]),
            );
            self.ss_frame = ctx.samplenum() as i64;
        }

        if self.ss_frame == -1 {
            self.ss_frame = ctx.samplenum() as i64;
        }

        if mdio == 1 {
            self.preamble_len += 1;
        }

        // Valid MDIO can't clock more than 16 succeeding ones without being in
        // either IDLE or PRE.
        if self.preamble_len > 16 {
            if self.preamble_len >= 10000 + 32 {
                let end = self.mdiobits[32].ss;
                let n = self.preamble_len - 32;
                ctx.put_ann(
                    self.ss_frame.max(0) as u64,
                    end,
                    ANN_FRAME_IDLE,
                    vec![format!("IDLE #{n}"), "IDLE".to_owned(), "I".to_owned()],
                );
                self.ss_frame = end as i64;
                self.preamble_len = 32;
                // This is getting out of hand, free some memory:
                // del self.mdiobits[33:-1].
                let len = self.mdiobits.len();
                if len > 34 {
                    self.mdiobits.drain(33..len - 1);
                }
            }
            if mdio == 0 {
                match self.preamble_len.cmp(&32) {
                    std::cmp::Ordering::Less => {
                        let ss = self.mdiobits[self.preamble_len].ss;
                        self.ss_frame = ss as i64;
                        ctx.put_ann(
                            ss,
                            ctx.samplenum(),
                            ANN_FRAME_ERROR,
                            texts(&["SHORT PREAMBLE", "SHRT PRE"]),
                        );
                    }
                    std::cmp::Ordering::Greater => {
                        let ss = self.mdiobits[32].ss;
                        self.ss_frame = ss as i64;
                        let start = self.mdiobits[self.preamble_len].ss;
                        let n = self.preamble_len - 32;
                        ctx.put_ann(
                            start,
                            ss,
                            ANN_FRAME_IDLE,
                            vec![format!("IDLE #{n}"), "IDLE".to_owned(), "I".to_owned()],
                        );
                        self.preamble_len = 32;
                    }
                    std::cmp::Ordering::Equal => {
                        self.ss_frame = self.mdiobits[32].ss as i64;
                    }
                }
                let ss = self.ss_frame.max(0) as u64;
                let n = self.preamble_len;
                ctx.put_ann(
                    ss,
                    ctx.samplenum(),
                    ANN_FRAME,
                    vec![format!("PRE #{n}"), "PRE".to_owned(), "P".to_owned()],
                );
                self.ss_frame_field = ctx.samplenum() as i64;
                self.state = State::St;
            }
        } else if mdio == 0 {
            self.ss_illegal = self.ss_frame;
            self.illegal_bus = 1;
        }
    }

    fn state_st(&mut self, mdio: u8) {
        if mdio == 0 {
            self.clause45 = 1;
        }
        self.state = State::Op;
    }

    fn state_op(&mut self, ctx: &mut dyn Context, mdio: u8) {
        if self.opcode == -1 {
            let st = if self.clause45 != 0 {
                texts(&["ST (Clause 45)", "ST 45", "ST", "S"])
            } else {
                texts(&["ST (Clause 22)", "ST 22", "ST", "S"])
            };
            self.putff(ctx, ANN_FRAME, st);
            self.ss_frame_field = ctx.samplenum() as i64;

            if mdio != 0 {
                self.opcode = 2;
            } else {
                self.opcode = 0;
            }
        } else if self.clause45 != 0 {
            self.state = State::Prtad;
            self.opcode += i64::from(mdio);
        } else {
            if i64::from(mdio) == self.opcode {
                "invalid for Clause 22".clone_into(&mut self.op_invalid);
            }
            self.state = State::Prtad;
        }
    }

    fn state_prtad(&mut self, ctx: &mut dyn Context, mdio: u8) {
        if self.portad == -1 {
            self.portad = 0;
            let op = if self.clause45 != 0 {
                match self.opcode {
                    0 => texts(&["OP: ADDR", "OP: A"]),
                    1 => texts(&["OP: WRITE", "OP: W"]),
                    2 => texts(&["OP: READINC", "OP: RI"]),
                    3 => texts(&["OP: READ", "OP: R"]),
                    _ => Vec::new(),
                }
            } else if self.opcode != 0 {
                texts(&["OP: READ", "OP: R"])
            } else {
                texts(&["OP: WRITE", "OP: W"])
            };
            let mut op = op;
            op.extend(texts(&["OP", "O"]));
            self.putff(ctx, ANN_FRAME, op);
            if !self.op_invalid.is_empty() {
                let msg = format!("OP {}", self.op_invalid);
                self.putff(ctx, ANN_FRAME_ERROR, vec![msg, "OP".to_owned(), "O".to_owned()]);
            }
            self.ss_frame_field = ctx.samplenum() as i64;
        }
        self.portad_bits -= 1;
        self.portad |= i64::from(mdio) << self.portad_bits;
        if self.portad_bits == 0 {
            self.state = State::Devad;
        }
    }

    fn state_devad(&mut self, ctx: &mut dyn Context, mdio: u8) {
        if self.devad == -1 {
            self.devad = 0;
            let prtad = if self.clause45 != 0 {
                vec![format!("PRTAD: {:02}", self.portad), "PRT".to_owned(), "P".to_owned()]
            } else {
                vec![format!("PHYAD: {:02}", self.portad), "PHY".to_owned(), "P".to_owned()]
            };
            self.putff(ctx, ANN_FRAME, prtad);
            self.ss_frame_field = ctx.samplenum() as i64;
        }
        self.devad_bits -= 1;
        self.devad |= i64::from(mdio) << self.devad_bits;
        if self.devad_bits == 0 {
            self.state = State::Ta;
        }
    }

    fn state_ta(&mut self, ctx: &mut dyn Context, mdio: u8) {
        if self.ta_seen {
            if mdio != 0 {
                self.ta_invalid = Some(if self.ta_invalid.is_some() {
                    " invalid (bit1 and bit2)".to_owned()
                } else {
                    " invalid (bit2)".to_owned()
                });
            }
            self.state = State::Data;
        } else {
            self.ta_seen = true;
            let regad = if self.clause45 != 0 {
                vec![format!("DEVAD: {:02}", self.devad), "DEV".to_owned(), "D".to_owned()]
            } else {
                vec![format!("REGAD: {:02}", self.devad), "REG".to_owned(), "R".to_owned()]
            };
            self.putff(ctx, ANN_FRAME, regad);
            self.ss_frame_field = ctx.samplenum() as i64;
            if mdio != 1
                && ((self.clause45 != 0 && self.opcode < 2)
                    || (self.clause45 == 0 && self.opcode == 0))
            {
                self.ta_invalid = Some(" invalid (bit1)".to_owned());
            }
        }
    }

    fn state_data(&mut self, ctx: &mut dyn Context, mdio: u8) {
        if self.data == -1 {
            self.data = 0;
            self.putff(ctx, ANN_FRAME, texts(&["TA", "T"]));
            if let Some(inv) = self.ta_invalid.clone() {
                self.putff(
                    ctx,
                    ANN_FRAME_ERROR,
                    vec![format!("TA{inv}"), "TA".to_owned(), "T".to_owned()],
                );
            }
            self.ss_frame_field = ctx.samplenum() as i64;
        }
        self.data_bits -= 1;
        self.data |= i64::from(mdio) << self.data_bits;
        if self.data_bits == 0 {
            // Output final bit.
            let q = self.quartile_cycle_length();
            let last = &mut self.mdiobits[0];
            last.es = Some(last.ss + q);
            let (value, ss, es) = (last.value, last.ss, last.es.expect("bit end"));
            self.bitcount += 1;
            self.putbit(ctx, value, ss, es);
            self.putdata(ctx);
            self.reset_decoder_state();
        }
    }

    /// The first quartile point of the frame's cycle lengths: a conservative
    /// guess for the end of the last cycle.
    fn quartile_cycle_length(&self) -> u64 {
        // 48 is the minimum number of samples we have to have at the end of a
        // frame. The last sample only has a leading clock edge and is ignored.
        let mut bitlen: Vec<u64> =
            (1..=48).map(|i| self.mdiobits[i].es.expect("bit end") - self.mdiobits[i].ss).collect();
        bitlen.sort_unstable();
        bitlen[12]
    }

    fn process_state(&mut self, ctx: &mut dyn Context, mdio: u8) {
        match self.state {
            State::Pre => self.state_pre(ctx, mdio),
            State::St => self.state_st(mdio),
            State::Op => self.state_op(ctx, mdio),
            State::Prtad => self.state_prtad(ctx, mdio),
            State::Devad => self.state_devad(ctx, mdio),
            State::Ta => self.state_ta(ctx, mdio),
            State::Data => self.state_data(ctx, mdio),
        }
    }

    fn handle_bit(&mut self, ctx: &mut dyn Context, mdio: u8) {
        self.bitcount += 1;
        self.mdiobits.insert(0, Bit { value: mdio, ss: ctx.samplenum(), es: None });

        if self.bitcount > 0 {
            let s = ctx.samplenum();
            let prev = &mut self.mdiobits[1];
            prev.es = Some(s); // Note end of last cycle.
            let (value, ss) = (prev.value, prev.ss);
            // Output the last bit we processed.
            self.putbit(ctx, value, ss, s);
        }

        self.process_state(ctx, mdio);
    }
}

impl Decoder for Mdio {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Mdio {
            show_debug_bits: config.str("show_debug_bits", "no") == "yes",
            ..Mdio::default()
        };
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            // Process pin state upon rising MDC edge.
            let pins =
                ctx.wait(&[&[crate::decode::Cond::Pin(MDC, crate::decode::CondKind::Rising)]])?;
            let mdio = u8::from(pins.get(MDIO));
            self.handle_bit(ctx, mdio);
        }
    }
}
