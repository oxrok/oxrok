// Counterpart in sigrok:
//   libsigrokdecode/decoders/em4305/pd.py,
//   Copyright (C) 2016 Benjamin Larsson <benjamin@southpole.se>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the field-gap timings, the 50/57-bit word layout and
//   the decoder's documented classes and texts.

//! EM4205/EM4305: 100-150kHz RFID, gap coded write frames.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT_VALUE: usize = 0;
const ANN_FFS: usize = 1;
const ANN_WRITE_MODE_EXIT: usize = 3;
const ANN_BIT: usize = 4;
const ANN_OPCODE: usize = 5;
const ANN_ADDRESS: usize = 9;
const ANN_BITRATE: usize = 10;

static OPTIONS: [OptionDesc; 7] = [
    OptionDesc {
        id: "coilfreq",
        desc: "Coil frequency",
        default: OptionValue::i(125_000),
        values: &[],
    },
    OptionDesc {
        id: "first_field_stop",
        desc: "First field stop min",
        default: OptionValue::i(40),
        values: &[],
    },
    OptionDesc { id: "w_gap", desc: "Write gap min", default: OptionValue::i(12), values: &[] },
    OptionDesc { id: "w_one_max", desc: "Write one max", default: OptionValue::i(32), values: &[] },
    OptionDesc {
        id: "w_zero_on_min",
        desc: "Write zero on min",
        default: OptionValue::i(15),
        values: &[],
    },
    OptionDesc {
        id: "w_zero_off_max",
        desc: "Write zero off max",
        default: OptionValue::i(27),
        values: &[],
    },
    OptionDesc {
        id: "em4100_decode",
        desc: "EM4100 decode",
        default: OptionValue::s("on"),
        values: &[OptionValue::s("on"), OptionValue::s("off")],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "em4305",
    name: "EM4305",
    longname: "RFID EM4205/EM4305",
    desc: "EM4205/EM4305 100-150kHz RFID protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["IC", "RFID"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit_value", desc: "Bit value" },
        AnnClass { id: "first_field_stop", desc: "First field stop" },
        AnnClass { id: "write_gap", desc: "Write gap" },
        AnnClass { id: "write_mode_exit", desc: "Write mode exit" },
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "opcode", desc: "Opcode" },
        AnnClass { id: "lock", desc: "Lock" },
        AnnClass { id: "data", desc: "Data" },
        AnnClass { id: "password", desc: "Password" },
        AnnClass { id: "address", desc: "Address" },
        AnnClass { id: "bitrate", desc: "Bitrate" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "structure", desc: "Structure", classes: &[1, 2, 3, 4] },
        AnnRow { id: "fields", desc: "Fields", classes: &[5, 6, 7, 8, 9] },
        AnnRow { id: "decode", desc: "Decode", classes: &[10] },
    ],
    binary: &[],
    doc: "EM4305 is a 100-150kHz RFID protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Em4305::new())
}

/// The gap detection state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    FfsSearch,
    FfsDetected,
}

/// The EM4305 decoder.
pub struct Em4305 {
    samplerate: u64,
    last_samplenum: u64,
    state: State,
    /// `[bit, ss, es]` per received bit, 70 rows as in pd.py.
    bits_pos: [[u64; 3]; 70],
    bit_nr: usize,
    em4100_decode1_partial: u32,
    em4100_decode: bool,

    field_clock: f64,
    wzmax: f64,
    wzmin: f64,
    womax: f64,
    ffs: f64,
    writegap: f64,
    nogap: f64,

    coilfreq: i64,
    first_field_stop: i64,
    w_gap: i64,
    w_one_max: i64,
    w_zero_on_min: i64,
    w_zero_off_max: i64,

    // Decode loop state.
    oldsamplenum: u64,
    old_gap_end: u64,
    gap_detected: bool,

    br_string: [&'static str; 8],
    encoder: [&'static str; 4],
    delayed_on: [&'static str; 4],
    cmds: [&'static str; 8],
}

impl Em4305 {
    fn new() -> Self {
        Em4305 {
            samplerate: 0,
            last_samplenum: 0,
            state: State::FfsSearch,
            bits_pos: [[0; 3]; 70],
            bit_nr: 0,
            em4100_decode1_partial: 0,
            em4100_decode: true,
            field_clock: 0.0,
            wzmax: 0.0,
            wzmin: 0.0,
            womax: 0.0,
            ffs: 0.0,
            writegap: 0.0,
            nogap: 0.0,
            coilfreq: 125_000,
            first_field_stop: 40,
            w_gap: 12,
            w_one_max: 32,
            w_zero_on_min: 15,
            w_zero_off_max: 27,
            oldsamplenum: 0,
            old_gap_end: 0,
            gap_detected: false,
            br_string: ["RF/8", "RF/16", "Unused", "RF/32", "RF/40", "Unused", "Unused", "RF/64"],
            encoder: ["not used", "Manchester", "Bi-phase", "not used"],
            delayed_on: ["No delay", "Delayed on - BP/8", "Delayed on - BP/4", "No delay"],
            cmds: [
                "Invalid",
                "Login",
                "Write word",
                "Invalid",
                "Read word",
                "Disable",
                "Protect",
                "Invalid",
            ],
        }
    }

    /// `self.bits_pos[idx][0]`, the bit value.
    fn bit(&self, idx: usize) -> u32 {
        self.bits_pos[idx][0] as u32
    }

    /// `[self.bits_pos[idx][1], self.bits_pos[idx][2]]`.
    fn span(&self, idx: usize) -> (u64, u64) {
        (self.bits_pos[idx][1], self.bits_pos[idx][2])
    }

    fn get_8_bits(&self, idx: usize) -> u32 {
        let mut retval = 0;
        for i in 0..8 {
            retval <<= 1;
            retval |= self.bit(i + idx);
        }
        retval
    }

    fn get_3_bits(&self, idx: usize) -> u32 {
        (self.bit(idx) << 2) | (self.bit(idx + 1) << 1) | self.bit(idx + 2)
    }

    fn get_4_bits(&self, idx: usize) -> u32 {
        self.bit(idx)
            | (self.bit(idx + 1) << 1)
            | (self.bit(idx + 2) << 2)
            | (self.bit(idx + 3) << 3)
    }

    fn get_32_bits(&self, idx: usize) -> u64 {
        (u64::from(self.get_8_bits(idx + 27)) << 24)
            | (u64::from(self.get_8_bits(idx + 18)) << 16)
            | (u64::from(self.get_8_bits(idx + 9)) << 8)
            | u64::from(self.get_8_bits(idx))
    }

    fn add_bits_pos(&mut self, bit: u64, ss_bit: u64, es_bit: u64) {
        if self.bit_nr < 70 {
            self.bits_pos[self.bit_nr][0] = bit;
            self.bits_pos[self.bit_nr][1] = ss_bit;
            self.bits_pos[self.bit_nr][2] = es_bit;
            self.bit_nr += 1;
        }
    }

    fn decode_config(&mut self, ctx: &mut dyn Context, idx: usize) {
        let bitrate = self.get_3_bits(idx + 2) as usize;
        let (ss, es) = (self.span(idx).0, self.span(idx + 5).1);
        ctx.put_ann(
            ss,
            es,
            ANN_BITRATE,
            vec![
                format!("Data rate: {}", self.br_string[bitrate]),
                self.br_string[bitrate].to_owned(),
            ],
        );

        let encoding = self.bit(idx + 6) | (self.bit(idx + 7) << 1);
        let encoding = encoding as usize;
        let (ss, es) = (self.span(idx + 6).0, self.span(idx + 10).1);
        ctx.put_ann(
            ss,
            es,
            ANN_BITRATE,
            vec![format!("Encoder: {}", self.encoder[encoding]), self.encoder[encoding].to_owned()],
        );

        let (ss, es) = (self.span(idx + 11).0, self.span(idx + 12).1);
        ctx.put_ann(ss, es, ANN_BITRATE, vec!["Zero bits".into(), "ZB".into()]);

        let delay_on = (self.bit(idx + 13) | (self.bit(idx + 14) << 1)) as usize;
        let (ss, es) = (self.span(idx + 13).0, self.span(idx + 14).1);
        ctx.put_ann(
            ss,
            es,
            ANN_BITRATE,
            vec![
                format!("Delayed on: {}", self.delayed_on[delay_on]),
                self.delayed_on[delay_on].to_owned(),
            ],
        );

        let lwr = (self.bit(idx + 15) << 3)
            | (self.bit(idx + 16) << 2)
            | (self.bit(idx + 18) << 1)
            | self.bit(idx + 19);
        let (ss, es) = (self.span(idx + 15).0, self.span(idx + 19).1);
        ctx.put_ann(
            ss,
            es,
            ANN_BITRATE,
            vec![format!("Last default read word: {lwr}"), format!("LWR: {lwr}"), format!("{lwr}")],
        );

        let (ss, es) = self.span(idx + 20);
        ctx.put_ann(
            ss,
            es,
            ANN_BITRATE,
            vec![format!("Read login: {}", self.bit(idx + 20)), format!("{}", self.bit(idx + 20))],
        );

        let (ss, es) = self.span(idx + 21);
        ctx.put_ann(ss, es, ANN_BITRATE, vec!["Zero bits".into(), "ZB".into()]);

        let (ss, es) = self.span(idx + 22);
        ctx.put_ann(
            ss,
            es,
            ANN_BITRATE,
            vec![format!("Write login: {}", self.bit(idx + 22)), format!("{}", self.bit(idx + 22))],
        );

        let (ss, es) = (self.span(idx + 23).0, self.span(idx + 24).1);
        ctx.put_ann(ss, es, ANN_BITRATE, vec!["Zero bits".into(), "ZB".into()]);

        let (ss, es) = self.span(idx + 25);
        ctx.put_ann(
            ss,
            es,
            ANN_BITRATE,
            vec![format!("Disable: {}", self.bit(idx + 25)), format!("{}", self.bit(idx + 25))],
        );

        let (ss, es) = self.span(idx + 27);
        ctx.put_ann(
            ss,
            es,
            ANN_BITRATE,
            vec![
                format!("Reader talk first: {}", self.bit(idx + 27)),
                format!("RTF: {}", self.bit(idx + 27)),
            ],
        );

        let (ss, es) = self.span(idx + 28);
        ctx.put_ann(ss, es, ANN_BITRATE, vec!["Zero bits".into(), "ZB".into()]);

        let (ss, es) = self.span(idx + 29);
        ctx.put_ann(
            ss,
            es,
            ANN_BITRATE,
            vec![format!("Pigeon mode: {}", self.bit(idx + 29)), format!("{}", self.bit(idx + 29))],
        );

        let (ss, es) = (self.span(idx + 30).0, self.span(idx + 34).1);
        ctx.put_ann(ss, es, ANN_BITRATE, vec!["Reserved".into(), "Res".into(), "R".into()]);
    }

    fn put4bits(&mut self, ctx: &mut dyn Context, idx: usize) {
        let bits = (self.bit(idx) << 3)
            | (self.bit(idx + 1) << 2)
            | (self.bit(idx + 2) << 1)
            | self.bit(idx + 3);
        let (ss, es) = (self.span(idx).0, self.span(idx + 3).1);
        ctx.put_ann(ss, es, ANN_BITRATE, vec![format!("{bits:X}")]);
    }

    fn em4100_decode1(&mut self, ctx: &mut dyn Context, idx: usize) {
        let (ss, es) = (self.span(idx).0, self.span(idx + 9).1);
        ctx.put_ann(
            ss,
            es,
            ANN_BITRATE,
            vec!["EM4100 header".into(), "EM header".into(), "Header".into(), "H".into()],
        );

        self.put4bits(ctx, idx + 10);

        let bits = (self.bit(idx + 15) << 3)
            | (self.bit(idx + 16) << 2)
            | (self.bit(idx + 18) << 1)
            | self.bit(idx + 19);
        let (ss, es) = (self.span(idx + 15).0, self.span(idx + 19).1);
        ctx.put_ann(ss, es, ANN_BITRATE, vec![format!("{bits:X}")]);

        self.put4bits(ctx, idx + 21);
        self.put4bits(ctx, idx + 27);

        self.em4100_decode1_partial =
            (self.bit(idx + 32) << 3) | (self.bit(idx + 33) << 2) | (self.bit(idx + 34) << 1);
        let (ss, es) = (self.span(idx + 32).0, self.span(idx + 34).1);
        ctx.put_ann(ss, es, ANN_BITRATE, vec!["Partial nibble".into()]);
    }

    fn em4100_decode2(&mut self, ctx: &mut dyn Context, idx: usize) {
        if self.em4100_decode1_partial != 0 {
            let bits = self.em4100_decode1_partial + self.bit(idx);
            let (ss, es) = self.span(idx);
            ctx.put_ann(ss, es, ANN_BITRATE, vec![format!("{bits:X}")]);
            self.em4100_decode1_partial = 0;
        } else {
            let (ss, es) = self.span(idx);
            ctx.put_ann(ss, es, ANN_BITRATE, vec!["Partial nibble".into()]);
        }

        self.put4bits(ctx, idx + 2);

        let bits = (self.bit(idx + 7) << 3)
            | (self.bit(idx + 9) << 2)
            | (self.bit(idx + 10) << 1)
            | self.bit(idx + 11);
        let (ss, es) = (self.span(idx + 7).0, self.span(idx + 11).1);
        ctx.put_ann(ss, es, ANN_BITRATE, vec![format!("{bits:X}")]);

        self.put4bits(ctx, idx + 13);
        self.put4bits(ctx, idx + 19);

        let bits = (self.bit(idx + 24) << 3)
            | (self.bit(idx + 25) << 2)
            | (self.bit(idx + 27) << 1)
            | self.bit(idx + 28);
        let (ss, es) = (self.span(idx + 24).0, self.span(idx + 28).1);
        ctx.put_ann(ss, es, ANN_BITRATE, vec![format!("{bits:X}")]);

        let (ss, es) = (self.span(idx + 30).0, self.span(idx + 34).1);
        ctx.put_ann(ss, es, ANN_BITRATE, vec!["EM4100 trailer".into()]);
    }

    fn print_row_parity(&mut self, ctx: &mut dyn Context, idx: usize, length: usize) {
        let mut parity = 0;
        for i in 0..length {
            parity += self.bit(idx + i);
        }
        parity &= 1;
        let (ss, es) = self.span(idx + length);
        if parity == self.bit(idx + length) {
            ctx.put_ann(
                ss,
                es,
                ANN_OPCODE,
                vec!["Row parity OK".into(), "Parity OK".into(), "OK".into()],
            );
        } else {
            ctx.put_ann(
                ss,
                es,
                ANN_OPCODE,
                vec!["Row parity failed".into(), "Parity failed".into(), "Fail".into()],
            );
        }
    }

    fn print_col_parity(&mut self, ctx: &mut dyn Context, idx: usize) {
        let data_1 = self.get_8_bits(idx);
        let data_2 = self.get_8_bits(idx + 9);
        let data_3 = self.get_8_bits(idx + 9 + 9);
        let data_4 = self.get_8_bits(idx + 9 + 9 + 9);
        let col_par = self.get_8_bits(idx + 9 + 9 + 9 + 9);
        let col_par_calc = data_1 ^ data_2 ^ data_3 ^ data_4;

        let (ss, es) = (self.span(idx + 9 + 9 + 9 + 9).0, self.span(idx + 9 + 9 + 9 + 9 + 7).1);
        if col_par == col_par_calc {
            ctx.put_ann(
                ss,
                es,
                ANN_OPCODE,
                vec!["Column parity OK".into(), "Parity OK".into(), "OK".into()],
            );
        } else {
            ctx.put_ann(
                ss,
                es,
                ANN_OPCODE,
                vec!["Column parity failed".into(), "Parity failed".into(), "Fail".into()],
            );
        }
    }

    fn print_8bit_data(&mut self, ctx: &mut dyn Context, idx: usize) {
        let data = self.get_8_bits(idx);
        let (ss, es) = (self.span(idx).0, self.span(idx + 7).1);
        ctx.put_ann(ss, es, ANN_ADDRESS, vec![format!("Data: {data:X}"), format!("{data:X}")]);
    }

    fn put_fields(&mut self, ctx: &mut dyn Context) {
        if self.bit_nr == 50 {
            let (ss, es) = self.span(0);
            ctx.put_ann(ss, es, ANN_BIT, vec!["Logic zero".into()]);
            let (ss, es) = (self.span(1).0, self.span(4).1);
            ctx.put_ann(ss, es, ANN_BIT, vec!["Command".into(), "Cmd".into(), "C".into()]);
            let (ss, es) = (self.span(5).0, self.span(49).1);
            ctx.put_ann(
                ss,
                es,
                ANN_BIT,
                vec!["Password".into(), "Passwd".into(), "Pass".into(), "P".into()],
            );

            // Get command.
            let cmd = self.get_3_bits(1) as usize;
            let (ss, es) = (self.span(1).0, self.span(3).1);
            ctx.put_ann(ss, es, ANN_OPCODE, vec![self.cmds[cmd].to_owned()]);
            self.print_row_parity(ctx, 1, 3);

            // Print data.
            self.print_8bit_data(ctx, 5);
            self.print_row_parity(ctx, 5, 8);
            self.print_8bit_data(ctx, 14);
            self.print_row_parity(ctx, 14, 8);
            self.print_8bit_data(ctx, 23);
            self.print_row_parity(ctx, 23, 8);
            self.print_8bit_data(ctx, 32);
            self.print_row_parity(ctx, 32, 8);
            self.print_col_parity(ctx, 5);
            let (ss, es) = self.span(49);
            if self.bit(49) == 0 {
                ctx.put_ann(
                    ss,
                    es,
                    ANN_OPCODE,
                    vec!["Stop bit".into(), "Stop".into(), "SB".into()],
                );
            } else {
                ctx.put_ann(ss, es, ANN_OPCODE, vec!["Stop bit error".into(), "Error".into()]);
            }

            if cmd == 1 {
                let password = self.get_32_bits(5);
                let (ss, es) = (self.span(12).0, self.span(46).1);
                ctx.put_ann(ss, es, ANN_BITRATE, vec![format!("Login password: {password:X}")]);
            }
        }

        if self.bit_nr == 57 {
            let (ss, es) = self.span(0);
            ctx.put_ann(ss, es, ANN_BIT, vec!["Logic zero".into(), "LZ".into()]);
            let (ss, es) = (self.span(1).0, self.span(4).1);
            ctx.put_ann(ss, es, ANN_BIT, vec!["Command".into(), "Cmd".into(), "C".into()]);
            let (ss, es) = (self.span(5).0, self.span(11).1);
            ctx.put_ann(ss, es, ANN_BIT, vec!["Address".into(), "Addr".into(), "A".into()]);
            let (ss, es) = (self.span(12).0, self.span(56).1);
            ctx.put_ann(ss, es, ANN_BIT, vec!["Data".into(), "Da".into(), "D".into()]);

            // Get command.
            let cmd = self.get_3_bits(1) as usize;
            let (ss, es) = (self.span(1).0, self.span(3).1);
            ctx.put_ann(ss, es, ANN_OPCODE, vec![self.cmds[cmd].to_owned()]);
            self.print_row_parity(ctx, 1, 3);

            // Get address.
            let addr = self.get_4_bits(5);
            let (ss, es) = (self.span(5).0, self.span(8).1);
            ctx.put_ann(ss, es, ANN_ADDRESS, vec![format!("Addr: {addr}"), format!("{addr}")]);
            let (ss, es) = (self.span(9).0, self.span(10).1);
            ctx.put_ann(ss, es, ANN_OPCODE, vec!["Zero bits".into(), "ZB".into()]);
            self.print_row_parity(ctx, 5, 6);
            // Print data.
            self.print_8bit_data(ctx, 12);
            self.print_row_parity(ctx, 12, 8);
            self.print_8bit_data(ctx, 21);
            self.print_row_parity(ctx, 21, 8);
            self.print_8bit_data(ctx, 30);
            self.print_row_parity(ctx, 30, 8);
            self.print_8bit_data(ctx, 39);
            self.print_row_parity(ctx, 39, 8);
            self.print_col_parity(ctx, 12);
            let (ss, es) = self.span(56);
            if self.bit(56) == 0 {
                ctx.put_ann(
                    ss,
                    es,
                    ANN_OPCODE,
                    vec!["Stop bit".into(), "Stop".into(), "SB".into()],
                );
            } else {
                ctx.put_ann(ss, es, ANN_OPCODE, vec!["Stop bit error".into(), "Error".into()]);
            }

            if addr == 4 {
                self.decode_config(ctx, 12);
            }

            if addr == 2 {
                let password = self.get_32_bits(12);
                let (ss, es) = (self.span(12).0, self.span(46).1);
                ctx.put_ann(ss, es, ANN_BITRATE, vec![format!("Write password: {password:X}")]);
            }

            // If we are programming EM4100 data we can decode it halfway.
            if addr == 5 && self.em4100_decode {
                self.em4100_decode1(ctx, 12);
            }
            if addr == 6 && self.em4100_decode {
                self.em4100_decode2(ctx, 12);
            }
        }

        self.bit_nr = 0;
    }
}

impl Decoder for Em4305 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.coilfreq = config.int("coilfreq", 125_000);
        self.first_field_stop = config.int("first_field_stop", 40);
        self.w_gap = config.int("w_gap", 12);
        self.w_one_max = config.int("w_one_max", 32);
        self.w_zero_on_min = config.int("w_zero_on_min", 15);
        self.w_zero_off_max = config.int("w_zero_off_max", 27);
        self.em4100_decode = config.str("em4100_decode", "on") == "on";
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        self.field_clock = samplerate as f64 / self.coilfreq as f64;
        self.wzmax = self.w_zero_off_max as f64 * self.field_clock;
        self.wzmin = self.w_zero_on_min as f64 * self.field_clock;
        self.womax = self.w_one_max as f64 * self.field_clock;
        self.ffs = self.first_field_stop as f64 * self.field_clock;
        self.writegap = self.w_gap as f64 * self.field_clock;
        self.nogap = 300.0 * self.field_clock;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::device("Cannot decode without samplerate."));
        }

        // Initialize internal state.
        self.last_samplenum = ctx.samplenum();
        self.oldsamplenum = 0;
        self.old_gap_end = 0;
        self.gap_detected = false;
        self.bit_nr = 0;

        let cond = [Cond::pin(DATA, CondKind::Edge)];
        loop {
            // Ignore identical samples, only process edges.
            ctx.wait(&[&cond])?;
            let now = ctx.samplenum();

            let pl = now - self.oldsamplenum;

            if self.state == State::FfsDetected {
                if pl as f64 > self.writegap {
                    self.gap_detected = true;
                }
                if (self.last_samplenum as f64 - self.old_gap_end as f64) > self.nogap {
                    self.gap_detected = false;
                    self.state = State::FfsSearch;
                    ctx.put_ann(
                        self.old_gap_end,
                        self.last_samplenum,
                        ANN_WRITE_MODE_EXIT,
                        vec!["Write mode exit".into()],
                    );
                    self.put_fields(ctx);
                }
            }

            if self.state == State::FfsSearch && pl as f64 > self.ffs {
                self.gap_detected = true;
                ctx.put_ann(
                    self.last_samplenum,
                    now,
                    ANN_FFS,
                    vec!["First field stop".into(), "Field stop".into(), "FFS".into()],
                );
                self.state = State::FfsDetected;
            }

            if self.gap_detected {
                self.gap_detected = false;
                let gap = self.last_samplenum as f64 - self.old_gap_end as f64;
                if gap > self.wzmin && gap < self.wzmax {
                    ctx.put_ann(self.old_gap_end, now, ANN_BIT_VALUE, vec!["0".into()]);
                    self.add_bits_pos(0, self.old_gap_end, now);
                }
                if gap > self.womax && gap < self.nogap {
                    // One or more 1 bits.
                    let one_bits = (gap / self.womax) as u64;
                    for ox in 0..one_bits {
                        let bs = (self.old_gap_end as f64 + ox as f64 * self.womax) as u64;
                        let be =
                            (self.old_gap_end as f64 + ox as f64 * self.womax + self.womax) as u64;
                        ctx.put_ann(bs, be, ANN_BIT_VALUE, vec!["1".into()]);
                        self.add_bits_pos(1, bs, be);
                    }
                    let between = now as f64 - self.last_samplenum as f64;
                    if between > self.wzmin && between < self.wzmax {
                        let bs = (self.old_gap_end as f64 + one_bits as f64 * self.womax) as u64;
                        ctx.put_ann(bs, now, ANN_BIT_VALUE, vec!["0".into()]);
                        self.add_bits_pos(0, bs, now);
                    }
                }

                self.old_gap_end = now;
            }

            self.oldsamplenum = now;
            self.last_samplenum = now;
        }
    }
}
