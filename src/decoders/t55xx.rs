// Counterpart in sigrok:
//   libsigrokdecode/decoders/t55xx/pd.py,
//   Copyright (C) 2015 Benjamin Larsson <benjamin@southpole.se>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the pulse interval coding and the decoder's
//   documented classes and texts.

//! T55xx: 100-150kHz RFID downlink/write protocol.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT_VALUE: usize = 0;
const ANN_START_GAP: usize = 1;
const ANN_WRITE_GAP: usize = 2;
const ANN_WRITE_MODE_EXIT: usize = 3;
const ANN_BIT: usize = 4;
const ANN_OPCODE: usize = 5;
const ANN_LOCK: usize = 6;
const ANN_DATA: usize = 7;
const ANN_PASSWORD: usize = 8;
const ANN_ADDRESS: usize = 9;
const ANN_BITRATE: usize = 10;

static OPTIONS: [OptionDesc; 8] = [
    OptionDesc {
        id: "coilfreq",
        desc: "Coil frequency",
        default: OptionValue::i(125_000),
        values: &[],
    },
    OptionDesc { id: "start_gap", desc: "Start gap min", default: OptionValue::i(20), values: &[] },
    OptionDesc { id: "w_gap", desc: "Write gap min", default: OptionValue::i(20), values: &[] },
    OptionDesc { id: "w_one_min", desc: "Write one min", default: OptionValue::i(48), values: &[] },
    OptionDesc { id: "w_one_max", desc: "Write one max", default: OptionValue::i(63), values: &[] },
    OptionDesc {
        id: "w_zero_min",
        desc: "Write zero min",
        default: OptionValue::i(16),
        values: &[],
    },
    OptionDesc {
        id: "w_zero_max",
        desc: "Write zero max",
        default: OptionValue::i(31),
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
    id: "t55xx",
    name: "T55xx",
    longname: "RFID T55xx",
    desc: "T55xx 100-150kHz RFID protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["IC", "RFID"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit_value", desc: "Bit value" },
        AnnClass { id: "start_gap", desc: "Start gap" },
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
    doc: "T55xx is a 100-150kHz RFID protocol according to the Atmel e555x\n\
          downlink/write protocol (pulse interval coding).",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(T55xx::new())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    StartGap,
    WriteGap,
}

const BR_STRING: [&str; 8] =
    ["RF/8", "RF/16", "RF/32", "RF/40", "RF/50", "RF/64", "RF/100", "RF/128"];
const MOD_STR1: [&str; 4] = ["Direct", "Manchester", "Biphase", "Reserved"];
const MOD_STR2: [&str; 8] = ["Direct", "PSK1", "PSK2", "PSK3", "FSK1", "FSK2", "FSK1a", "FSK2a"];
const PSKCF_STR: [&str; 4] = ["RF/2", "RF/4", "RF/8", "Reserved"];

/// The T55xx decoder.
pub struct T55xx {
    samplerate: u64,
    field_clock: f64,
    wzmax: f64,
    wzmin: f64,
    womax: f64,
    womin: f64,
    startgap: f64,
    writegap: f64,
    nogap: f64,
    state: State,
    bits_pos: [[i64; 3]; 70],
    bit_nr: usize,
    old_gap_start: i64,
    old_gap_end: i64,
    gap_detected: bool,
    last_samplenum: i64,
    oldsamplenum: i64,
    oldpl: i64,
    oldpp: u32,
    last_bit_pos: i64,
    last_edge: i64,
    coilfreq: i64,
    start_gap: i64,
    w_gap: i64,
    w_one_min: i64,
    w_one_max: i64,
    w_zero_min: i64,
    w_zero_max: i64,
    em4100_decode: bool,
    em4100_decode1_partial: i64,
}

impl T55xx {
    fn new() -> Self {
        T55xx {
            samplerate: 0,
            field_clock: 0.0,
            wzmax: 0.0,
            wzmin: 0.0,
            womax: 0.0,
            womin: 0.0,
            startgap: 0.0,
            writegap: 0.0,
            nogap: 0.0,
            state: State::StartGap,
            bits_pos: [[0; 3]; 70],
            bit_nr: 0,
            old_gap_start: 0,
            old_gap_end: 0,
            gap_detected: false,
            last_samplenum: 0,
            oldsamplenum: 0,
            oldpl: 0,
            oldpp: 0,
            last_bit_pos: 0,
            last_edge: 0,
            coilfreq: 125_000,
            start_gap: 20,
            w_gap: 20,
            w_one_min: 48,
            w_one_max: 63,
            w_zero_min: 16,
            w_zero_max: 31,
            em4100_decode: true,
            em4100_decode1_partial: 0,
        }
    }

    fn decode_config(&mut self, ctx: &mut dyn Context, idx: usize) {
        let safer_key = (self.bits_pos[idx][0] << 3)
            | (self.bits_pos[idx + 1][0] << 2)
            | (self.bits_pos[idx + 2][0] << 1)
            | self.bits_pos[idx + 3][0];
        ctx.put_ann(
            self.bits_pos[idx][1].max(0) as u64,
            self.bits_pos[idx + 3][2].max(0) as u64,
            ANN_BITRATE,
            vec![format!("Safer Key: {safer_key:X}"), format!("{safer_key:X}")],
        );
        let bitrate = (self.bits_pos[idx + 11][0] << 2)
            | (self.bits_pos[idx + 12][0] << 1)
            | self.bits_pos[idx + 13][0];
        let s = BR_STRING[bitrate as usize];
        ctx.put_ann(
            self.bits_pos[idx + 11][1].max(0) as u64,
            self.bits_pos[idx + 13][2].max(0) as u64,
            ANN_BITRATE,
            vec![format!("Data Bit Rate: {s}"), s.to_owned()],
        );
        let modulation1 = (self.bits_pos[idx + 15][0] << 1) | self.bits_pos[idx + 16][0];
        let modulation2 = (self.bits_pos[idx + 17][0] << 2)
            | (self.bits_pos[idx + 18][0] << 1)
            | self.bits_pos[idx + 19][0];
        let mod_string = if modulation1 == 0 {
            MOD_STR2[modulation2 as usize]
        } else {
            MOD_STR1[modulation1 as usize]
        };
        ctx.put_ann(
            self.bits_pos[idx + 15][1].max(0) as u64,
            self.bits_pos[idx + 19][2].max(0) as u64,
            ANN_BITRATE,
            vec![format!("Modulation: {mod_string}"), mod_string.to_owned()],
        );
        let psk_cf = (self.bits_pos[idx + 20][0] << 1) | self.bits_pos[idx + 21][0];
        let s = PSKCF_STR[psk_cf as usize];
        ctx.put_ann(
            self.bits_pos[idx + 20][1].max(0) as u64,
            self.bits_pos[idx + 21][2].max(0) as u64,
            ANN_BITRATE,
            vec![format!("PSK-CF: {s}"), s.to_owned()],
        );
        let v = self.bits_pos[idx + 22][0];
        ctx.put_ann(
            self.bits_pos[idx + 22][1].max(0) as u64,
            self.bits_pos[idx + 22][2].max(0) as u64,
            ANN_BITRATE,
            vec![format!("AOR: {v}"), format!("{v}")],
        );
        let maxblock = (self.bits_pos[idx + 24][0] << 2)
            | (self.bits_pos[idx + 25][0] << 1)
            | self.bits_pos[idx + 26][0];
        ctx.put_ann(
            self.bits_pos[idx + 24][1].max(0) as u64,
            self.bits_pos[idx + 26][2].max(0) as u64,
            ANN_BITRATE,
            vec![format!("Max-Block: {maxblock}"), format!("{maxblock}")],
        );
        let v = self.bits_pos[idx + 27][0];
        ctx.put_ann(
            self.bits_pos[idx + 27][1].max(0) as u64,
            self.bits_pos[idx + 27][2].max(0) as u64,
            ANN_BITRATE,
            vec![format!("PWD: {v}"), format!("{v}")],
        );
        let v = self.bits_pos[idx + 28][0];
        ctx.put_ann(
            self.bits_pos[idx + 28][1].max(0) as u64,
            self.bits_pos[idx + 28][2].max(0) as u64,
            ANN_BITRATE,
            vec![format!("ST-sequence terminator: {v}"), format!("{v}")],
        );
        let v = self.bits_pos[idx + 31][0];
        ctx.put_ann(
            self.bits_pos[idx + 31][1].max(0) as u64,
            self.bits_pos[idx + 31][2].max(0) as u64,
            ANN_BITRATE,
            vec![format!("POR delay: {v}"), format!("{v}")],
        );
    }

    fn put4bits(&mut self, ctx: &mut dyn Context, idx: usize) {
        let bits = (self.bits_pos[idx][0] << 3)
            | (self.bits_pos[idx + 1][0] << 2)
            | (self.bits_pos[idx + 2][0] << 1)
            | self.bits_pos[idx + 3][0];
        ctx.put_ann(
            self.bits_pos[idx][1].max(0) as u64,
            self.bits_pos[idx + 3][2].max(0) as u64,
            ANN_BITRATE,
            vec![format!("{bits:X}")],
        );
    }

    fn em4100_decode1(&mut self, ctx: &mut dyn Context, idx: usize) {
        ctx.put_ann(
            self.bits_pos[idx][1].max(0) as u64,
            self.bits_pos[idx + 8][2].max(0) as u64,
            ANN_BITRATE,
            vec!["EM4100 header".into(), "EM header".into(), "Header".into(), "H".into()],
        );
        self.put4bits(ctx, idx + 9);
        self.put4bits(ctx, idx + 14);
        self.put4bits(ctx, idx + 19);
        self.put4bits(ctx, idx + 24);
        self.em4100_decode1_partial = (self.bits_pos[idx + 29][0] << 3)
            | (self.bits_pos[idx + 30][0] << 2)
            | (self.bits_pos[idx + 31][0] << 1);
        ctx.put_ann(
            self.bits_pos[idx + 29][1].max(0) as u64,
            self.bits_pos[idx + 31][2].max(0) as u64,
            ANN_BITRATE,
            vec!["Partial nibble".into()],
        );
    }

    fn em4100_decode2(&mut self, ctx: &mut dyn Context, idx: usize) {
        if self.em4100_decode1_partial != 0 {
            let bits = self.em4100_decode1_partial + self.bits_pos[idx][0];
            ctx.put_ann(
                self.bits_pos[idx][1].max(0) as u64,
                self.bits_pos[idx][2].max(0) as u64,
                ANN_BITRATE,
                vec![format!("{bits:X}")],
            );
            self.em4100_decode1_partial = 0;
        } else {
            ctx.put_ann(
                self.bits_pos[idx][1].max(0) as u64,
                self.bits_pos[idx][2].max(0) as u64,
                ANN_BITRATE,
                vec!["Partial nibble".into()],
            );
        }
        self.put4bits(ctx, idx + 2);
        self.put4bits(ctx, idx + 7);
        self.put4bits(ctx, idx + 12);
        self.put4bits(ctx, idx + 17);
        self.put4bits(ctx, idx + 22);
        ctx.put_ann(
            self.bits_pos[idx + 27][1].max(0) as u64,
            self.bits_pos[idx + 31][2].max(0) as u64,
            ANN_BITRATE,
            vec!["EM4100 trailer".into()],
        );
    }

    fn get_32_bits(&self, idx: usize) -> u64 {
        let mut retval = 0u64;
        for i in 0..32 {
            retval <<= 1;
            retval |= self.bits_pos[i + idx][0] as u64;
        }
        retval
    }

    fn get_3_bits(&self, idx: usize) -> u64 {
        ((self.bits_pos[idx][0] << 2)
            | (self.bits_pos[idx + 1][0] << 1)
            | self.bits_pos[idx + 2][0]) as u64
    }

    #[allow(clippy::too_many_lines)]
    fn put_fields(&mut self, ctx: &mut dyn Context) {
        if self.bit_nr == 70 {
            let op0 = self.bits_pos[0][0];
            let op1 = self.bits_pos[1][0];
            ctx.put_ann(
                self.bits_pos[0][1].max(0) as u64,
                self.bits_pos[1][2].max(0) as u64,
                ANN_OPCODE,
                vec![format!("Opcode: {op0}{op1}"), format!("{op0}{op1}")],
            );
            let password = self.get_32_bits(2);
            ctx.put_ann(
                self.bits_pos[2][1].max(0) as u64,
                self.bits_pos[33][2].max(0) as u64,
                ANN_PASSWORD,
                vec![format!("Password: {password:X}"), format!("{password:X}")],
            );
            let lock = self.bits_pos[34][0] as u64;
            ctx.put_ann(
                self.bits_pos[34][1].max(0) as u64,
                self.bits_pos[34][2].max(0) as u64,
                ANN_LOCK,
                vec![format!("Lock: {lock:X}"), format!("{lock:X}")],
            );
            let data = self.get_32_bits(35);
            ctx.put_ann(
                self.bits_pos[35][1].max(0) as u64,
                self.bits_pos[66][2].max(0) as u64,
                ANN_DATA,
                vec![format!("Data: {data:X}"), format!("{data:X}")],
            );
            let addr = self.get_3_bits(67);
            ctx.put_ann(
                self.bits_pos[67][1].max(0) as u64,
                self.bits_pos[69][2].max(0) as u64,
                ANN_ADDRESS,
                vec![format!("Addr: {addr:X}"), format!("{addr:X}")],
            );
            if addr == 0 {
                self.decode_config(ctx, 35);
            }
            if addr == 7 {
                ctx.put_ann(
                    self.bits_pos[35][1].max(0) as u64,
                    self.bits_pos[66][2].max(0) as u64,
                    ANN_BITRATE,
                    vec![format!("Password: {data:X}"), format!("{data:X}")],
                );
            }
            // If we are programming EM4100 data we can decode it halfway.
            if addr == 1 && self.em4100_decode {
                self.em4100_decode1(ctx, 35);
            }
            if addr == 2 && self.em4100_decode {
                self.em4100_decode2(ctx, 35);
            }
        }

        if self.bit_nr == 38 {
            let op0 = self.bits_pos[0][0];
            let op1 = self.bits_pos[1][0];
            ctx.put_ann(
                self.bits_pos[0][1].max(0) as u64,
                self.bits_pos[1][2].max(0) as u64,
                ANN_OPCODE,
                vec![format!("Opcode: {op0}{op1}"), format!("{op0}{op1}")],
            );
            let lock = self.bits_pos[2][0] as u64;
            ctx.put_ann(
                self.bits_pos[2][1].max(0) as u64,
                self.bits_pos[2][2].max(0) as u64,
                ANN_LOCK,
                vec![format!("Lock: {lock:X}"), format!("{lock:X}")],
            );
            let data = self.get_32_bits(3);
            ctx.put_ann(
                self.bits_pos[3][1].max(0) as u64,
                self.bits_pos[34][2].max(0) as u64,
                ANN_DATA,
                vec![format!("Data: {data:X}"), format!("{data:X}")],
            );
            let addr = self.get_3_bits(35);
            ctx.put_ann(
                self.bits_pos[35][1].max(0) as u64,
                self.bits_pos[37][2].max(0) as u64,
                ANN_ADDRESS,
                vec![format!("Addr: {addr:X}"), format!("{addr:X}")],
            );
            if addr == 0 {
                self.decode_config(ctx, 3);
            }
            if addr == 7 {
                ctx.put_ann(
                    self.bits_pos[3][1].max(0) as u64,
                    self.bits_pos[34][2].max(0) as u64,
                    ANN_BITRATE,
                    vec![format!("Password: {data:X}"), format!("{data:X}")],
                );
            }
            // If we are programming EM4100 data we can decode it halfway.
            if addr == 1 && self.em4100_decode {
                self.em4100_decode1(ctx, 3);
            }
            if addr == 2 && self.em4100_decode {
                self.em4100_decode2(ctx, 3);
            }
        }

        if self.bit_nr == 2 {
            let op0 = self.bits_pos[0][0];
            let op1 = self.bits_pos[1][0];
            ctx.put_ann(
                self.bits_pos[0][1].max(0) as u64,
                self.bits_pos[1][2].max(0) as u64,
                ANN_OPCODE,
                vec![format!("Opcode: {op0}{op1}"), format!("{op0}{op1}")],
            );
        }
        self.bit_nr = 0;
    }

    fn add_bits_pos(&mut self, bit: i64, bit_start: i64, bit_end: i64) {
        if self.bit_nr < 70 {
            self.bits_pos[self.bit_nr][0] = bit;
            self.bits_pos[self.bit_nr][1] = bit_start;
            self.bits_pos[self.bit_nr][2] = bit_end;
            self.bit_nr += 1;
        }
    }
}

impl Decoder for T55xx {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.coilfreq = config.int("coilfreq", 125_000);
        self.start_gap = config.int("start_gap", 20);
        self.w_gap = config.int("w_gap", 20);
        self.w_one_min = config.int("w_one_min", 48);
        self.w_one_max = config.int("w_one_max", 63);
        self.w_zero_min = config.int("w_zero_min", 16);
        self.w_zero_max = config.int("w_zero_max", 31);
        self.em4100_decode = config.str("em4100_decode", "on") == "on";
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        // pd.py divides as floats and multiplies the integer options.
        self.field_clock = samplerate as f64 / self.coilfreq as f64;
        self.wzmax = self.w_zero_max as f64 * self.field_clock;
        self.wzmin = self.w_zero_min as f64 * self.field_clock;
        self.womax = self.w_one_max as f64 * self.field_clock;
        self.womin = self.w_one_min as f64 * self.field_clock;
        self.startgap = self.start_gap as f64 * self.field_clock;
        self.writegap = self.w_gap as f64 * self.field_clock;
        self.nogap = 64.0 * self.field_clock;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::device("Cannot decode without samplerate."));
        }

        self.last_samplenum = 0;
        self.oldsamplenum = 0;
        self.oldpl = 0;
        self.oldpp = 0;
        self.last_bit_pos = 0;
        self.old_gap_start = 0;
        self.old_gap_end = 0;
        self.gap_detected = false;
        self.bit_nr = 0;
        self.last_edge = 0;

        loop {
            let pins = ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?;
            let sn = ctx.samplenum() as i64;
            let pl = sn - self.oldsamplenum;
            let pp = u32::from(pins.get(DATA));

            if self.state == State::WriteGap {
                if pl as f64 > self.writegap {
                    self.gap_detected = true;
                    ctx.put_ann(
                        self.last_samplenum.max(0) as u64,
                        sn.max(0) as u64,
                        ANN_WRITE_GAP,
                        vec!["Write gap".into()],
                    );
                }
                if (self.last_samplenum - self.old_gap_end) as f64 > self.nogap {
                    self.gap_detected = false;
                    self.state = State::StartGap;
                    ctx.put_ann(
                        self.old_gap_end.max(0) as u64,
                        self.last_samplenum.max(0) as u64,
                        ANN_WRITE_MODE_EXIT,
                        vec!["Write mode exit".into()],
                    );
                    self.put_fields(ctx);
                }
            }

            if self.state == State::StartGap && pl as f64 > self.startgap {
                self.gap_detected = true;
                ctx.put_ann(
                    self.last_samplenum.max(0) as u64,
                    sn.max(0) as u64,
                    ANN_START_GAP,
                    vec!["Start gap".into()],
                );
                self.state = State::WriteGap;
            }

            if self.gap_detected {
                self.gap_detected = false;
                if (self.last_samplenum - self.old_gap_end) as f64 > self.wzmin
                    && ((self.last_samplenum - self.old_gap_end) as f64) < self.wzmax
                {
                    ctx.put_ann(
                        self.old_gap_end.max(0) as u64,
                        self.last_samplenum.max(0) as u64,
                        ANN_BIT_VALUE,
                        vec!["0".into()],
                    );
                    ctx.put_ann(
                        self.old_gap_end.max(0) as u64,
                        self.last_samplenum.max(0) as u64,
                        ANN_BIT,
                        vec!["Bit".into()],
                    );
                    self.add_bits_pos(0, self.old_gap_end, self.last_samplenum);
                }
                if (self.last_samplenum - self.old_gap_end) as f64 > self.womin
                    && ((self.last_samplenum - self.old_gap_end) as f64) < self.womax
                {
                    ctx.put_ann(
                        self.old_gap_end.max(0) as u64,
                        self.last_samplenum.max(0) as u64,
                        ANN_BIT_VALUE,
                        vec!["1".into()],
                    );
                    ctx.put_ann(
                        self.old_gap_end.max(0) as u64,
                        self.last_samplenum.max(0) as u64,
                        ANN_BIT,
                        vec!["Bit".into()],
                    );
                    self.add_bits_pos(1, self.old_gap_end, self.last_samplenum);
                }

                self.old_gap_start = self.last_samplenum;
                self.old_gap_end = sn;
            }

            self.oldpl = pl;
            self.oldpp = pp;
            self.oldsamplenum = sn;
            self.last_samplenum = sn;
        }
    }
}
