// Counterpart in sigrok:
//   libsigrokdecode/decoders/flexray/pd.py,
//   Copyright (C) 2019 Stephan Thiele <stephan.thiele@mailbox.org>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the FlexRay 3.0.1 frame layout and the decoder's
//   documented classes and texts.

//! `FlexRay`: automotive field bus frames.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const CH: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_TSS: usize = 1;
const ANN_FSS: usize = 2;
const ANN_RESERVED: usize = 3;
const ANN_PPI: usize = 4;
const ANN_NULL: usize = 5;
const ANN_SYNC: usize = 6;
const ANN_STARTUP: usize = 7;
const ANN_ID: usize = 8;
const ANN_LENGTH: usize = 9;
const ANN_HCRC: usize = 10;
const ANN_CYCLE: usize = 11;
const ANN_DATA_BYTE: usize = 12;
const ANN_FRAME_CRC: usize = 13;
const ANN_CID_DELIM: usize = 14;
const ANN_BSS: usize = 15;
const ANN_BIT: usize = 17;
const ANN_CID: usize = 18;
const ANN_DTS: usize = 19;
const ANN_CAS: usize = 20;

// FlexRay specification 3.0.1 Chapter A.1 constants, as pd.py's Const.
const C_CHANNEL_IDLE_DELIMITER: i64 = 11;
const C_CRC_INIT_A: u64 = 0x00FE_DCBA;
const C_CRC_INIT_B: u64 = 0x00AB_CDEF;
const C_CRC_POLYNOMIAL: u64 = 0x5D_6DCB;
const C_CRC_SIZE: u32 = 24;
const C_HCRC_INIT: u64 = 0x01A;
const C_HCRC_POLYNOMIAL: u64 = 0x385;
const C_HCRC_SIZE: u32 = 11;

static OPTIONS: [OptionDesc; 2] = [
    OptionDesc {
        id: "channel_type",
        desc: "Channel type",
        default: OptionValue::s("A"),
        values: &[OptionValue::s("A"), OptionValue::s("B")],
    },
    OptionDesc {
        id: "bitrate",
        desc: "Bitrate (bit/s)",
        default: OptionValue::i(10_000_000),
        values: &[OptionValue::i(10_000_000), OptionValue::i(5_000_000), OptionValue::i(2_500_000)],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "flexray",
    name: "FlexRay",
    longname: "FlexRay",
    desc: "Automotive network communications protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Automotive"],
    channels: &[ChannelDesc { id: "channel", name: "Channel", desc: "FlexRay bus channel" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "data", desc: "FlexRay payload data" },
        AnnClass { id: "tss", desc: "Transmission start sequence" },
        AnnClass { id: "fss", desc: "Frame start sequence" },
        AnnClass { id: "reserved-bit", desc: "Reserved bit" },
        AnnClass { id: "ppi", desc: "Payload preamble indicator" },
        AnnClass { id: "null-frame", desc: "Nullframe indicator" },
        AnnClass { id: "sync-frame", desc: "Full identifier" },
        AnnClass { id: "startup-frame", desc: "Startup frame indicator" },
        AnnClass { id: "id", desc: "Frame ID" },
        AnnClass { id: "length", desc: "Data length" },
        AnnClass { id: "header-crc", desc: "Header CRC" },
        AnnClass { id: "cycle", desc: "Cycle code" },
        AnnClass { id: "data-byte", desc: "Data byte" },
        AnnClass { id: "frame-crc", desc: "Frame CRC" },
        AnnClass { id: "cid-delimiter", desc: "Channel idle delimiter" },
        AnnClass { id: "bss", desc: "Byte start sequence" },
        AnnClass { id: "warnings", desc: "Human-readable warnings" },
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "cid", desc: "Channel idle delimiter" },
        AnnClass { id: "dts", desc: "Dynamic trailing sequence" },
        AnnClass { id: "cas", desc: "Collision avoidance symbol" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[15, 17] },
        AnnRow {
            id: "fields",
            desc: "Fields",
            classes: &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 18, 19, 20],
        },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[16] },
    ],
    binary: &[],
    doc: "FlexRay is a fast, deterministic and fault-tolerant fieldbus system\n\
          which is used in cars in high security related areas like X-by-Wire.\n\
          \n\
          It is the result of the FlexRay consortium which consisted of BMW,\n\
          Daimler, Motorola (today Freescale) and Philips, with the goal of\n\
          working out a common standard automotive bus system.\n\
          \n\
          This decoder assumes that at least one channel of a logic level RX line\n\
          of a transceiver is sampled (e.g. NXP TJA1080).",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(FlexRay::new())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    GetBits,
}

/// The `FlexRay` decoder.
pub struct FlexRay {
    samplerate: u64,
    bitrate: f64,
    channel_b: bool,
    bit_width: f64,
    sample_point: f64,
    sample_point_percent: f64,
    state: State,
    tss_start: Option<i64>,
    tss_end: Option<i64>,
    rawbits: Vec<u8>,
    bits: Vec<u8>,
    curbit: i64,
    last_databit: i64,
    last_xmit_bit: i64,
    ss_block: Option<i64>,
    ss_databytebits: Vec<i64>,
    end_of_frame: bool,
    dynamic_frame: bool,
    ss_bit0: Option<i64>,
    ss_bit1: Option<i64>,
    ss_bit2: Option<i64>,
    payload_length: i64,
    dom_edge_snum: i64,
    dom_edge_bcount: i64,
}

impl FlexRay {
    fn new() -> Self {
        let mut d = FlexRay {
            samplerate: 0,
            bitrate: 10_000_000.0,
            channel_b: false,
            bit_width: 0.0,
            sample_point: 0.0,
            sample_point_percent: 50.0,
            state: State::Idle,
            tss_start: None,
            tss_end: None,
            rawbits: Vec::new(),
            bits: Vec::new(),
            curbit: 0,
            last_databit: 999,
            last_xmit_bit: 999,
            ss_block: None,
            ss_databytebits: Vec::new(),
            end_of_frame: false,
            dynamic_frame: false,
            ss_bit0: None,
            ss_bit1: None,
            ss_bit2: None,
            payload_length: 0,
            dom_edge_snum: 0,
            dom_edge_bcount: 0,
        };
        d.reset_variables();
        d
    }

    fn reset_variables(&mut self) {
        self.sample_point_percent = 50.0;
        self.state = State::Idle;
        self.tss_start = None;
        self.tss_end = None;
        self.rawbits.clear();
        self.bits.clear();
        self.curbit = 0;
        self.last_databit = 999;
        self.last_xmit_bit = 999;
        self.ss_block = None;
        self.ss_databytebits.clear();
        self.end_of_frame = false;
        self.dynamic_frame = false;
        self.ss_bit0 = None;
        self.ss_bit1 = None;
        self.ss_bit2 = None;
    }

    /// pd.py's `putg`: widen each annotation by half a bit on either side.
    fn putg(&self, ctx: &mut dyn Context, ss: i64, es: i64, class: usize, texts: Vec<String>) {
        let left = self.sample_point as i64;
        let right = (self.bit_width - self.sample_point) as i64;
        ctx.put_ann((ss - left).max(0) as u64, (es + right).max(0) as u64, class, texts);
    }

    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        let sn = ctx.samplenum() as i64;
        self.putg(ctx, sn, sn, class, texts);
    }

    fn putb(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        let ss = self.ss_block.expect("ss_block set");
        let es = ctx.samplenum() as i64;
        self.putg(ctx, ss, es, class, texts);
    }

    /// pd.py's generic `crc` for MSB-first bit strings.
    fn crc(bits: &[u8], polynom: u64, crc_len_bits: u32, iv: u64) -> u64 {
        let mask = (1u64 << crc_len_bits) - 1;
        let mut reg = iv;
        for &b in bits {
            let bit = ((reg >> (crc_len_bits - 1)) & 1) ^ u64::from(b);
            reg = (reg << 1) & mask;
            if bit != 0 {
                reg ^= polynom;
            }
        }
        reg & mask
    }

    fn bits_to_int(bits: &[u8]) -> i64 {
        let mut v: i64 = 0;
        for &b in bits {
            v = (v << 1) | i64::from(b);
        }
        v
    }

    fn dom_edge_seen(&mut self, ctx: &dyn Context) {
        self.dom_edge_snum = ctx.samplenum() as i64;
        self.dom_edge_bcount = self.curbit;
    }

    fn get_sample_point(&self, bitnum: i64) -> i64 {
        let mut sn = self.dom_edge_snum as f64;
        sn += self.bit_width * (bitnum - self.dom_edge_bcount) as f64;
        sn += self.sample_point;
        sn as i64
    }

    fn is_bss_sequence(&self) -> bool {
        // pd.py: `if self.end_of_frame: return False`; then two modulo
        // tests on the raw bit count (Python modulo is non-negative).
        if self.end_of_frame {
            return false;
        }
        let n = self.rawbits.len() as i64;
        if (n - 2).rem_euclid(10) == 0 {
            return true;
        }
        if (n - 3).rem_euclid(10) == 0 {
            return true;
        }
        false
    }

    #[allow(clippy::too_many_lines)]
    fn handle_bit(&mut self, ctx: &mut dyn Context, fr_rx: u8) {
        self.rawbits.push(fr_rx);
        self.bits.push(fr_rx);

        // Index of the current FlexRay frame bit.
        let bitnum = self.bits.len() as i64 - 1;

        // A byte start sequence is not part of the frame bits.
        if self.is_bss_sequence() {
            self.bits.pop();
            if bitnum > 1 {
                self.putx(ctx, ANN_BSS, vec![fr_rx.to_string()]);
            } else if self.rawbits.len() == 2 {
                self.ss_bit1 = Some(ctx.samplenum() as i64);
            } else if self.rawbits.len() == 3 {
                self.ss_bit2 = Some(ctx.samplenum() as i64);
            }
            self.curbit += 1; // bitnum is not affected.
            return;
        }
        if bitnum > 1 {
            self.putx(ctx, ANN_BIT, vec![fr_rx.to_string()]);
        }

        if bitnum == 0 {
            // Bit 0: Frame start sequence (FSS) bit.
            self.ss_bit0 = Some(ctx.samplenum() as i64);
        } else if bitnum == 1 {
            // Bit 1: Start of header.
            if self.rawbits.get(..3) == Some(&[1u8, 1, 0][..]) {
                let tss_start = self.tss_start.expect("tss_start");
                let tss_end = self.tss_end.expect("tss_end");
                ctx.put_ann(
                    tss_start.max(0) as u64,
                    tss_end.max(0) as u64,
                    ANN_TSS,
                    vec!["Transmission start sequence".into(), "TSS".into()],
                );
                let b0 = self.ss_bit0.expect("ss_bit0");
                self.putg(ctx, b0, b0, ANN_BIT, vec![self.rawbits[0].to_string()]);
                self.putg(ctx, b0, b0, ANN_FSS, vec!["FSS".into(), "Frame start sequence".into()]);
                let b1 = self.ss_bit1.expect("ss_bit1");
                self.putg(ctx, b1, b1, ANN_BSS, vec![self.rawbits[1].to_string()]);
                let b2 = self.ss_bit2.expect("ss_bit2");
                self.putg(ctx, b2, b2, ANN_BSS, vec![self.rawbits[2].to_string()]);
                self.putx(ctx, ANN_BIT, vec![fr_rx.to_string()]);
                self.putx(
                    ctx,
                    ANN_RESERVED,
                    vec![format!("Reserved bit: {fr_rx}"), format!("RB: {fr_rx}"), "RB".into()],
                );
            } else {
                let tss_start = self.tss_start.expect("tss_start");
                let tss_end = self.tss_end.expect("tss_end");
                ctx.put_ann(
                    tss_start.max(0) as u64,
                    tss_end.max(0) as u64,
                    ANN_CAS,
                    vec!["Collision avoidance symbol".into(), "CAS".into()],
                );
                self.reset_variables();
            }
        } else if bitnum == 2 {
            // Bit 2: Payload preamble indicator.
            self.putx(
                ctx,
                ANN_PPI,
                vec![format!("Payload preamble indicator: {fr_rx}"), format!("PPI: {fr_rx}")],
            );
        } else if bitnum == 3 {
            // Bit 3: Null frame indicator (inversed).
            let data_type = if fr_rx != 0 { "data frame" } else { "null frame" };
            self.putx(
                ctx,
                ANN_NULL,
                vec![
                    format!("Null frame indicator: {data_type}"),
                    format!("NF: {fr_rx}"),
                    "NF".into(),
                ],
            );
        } else if bitnum == 4 {
            // Bit 4: Sync frame indicator.
            self.putx(
                ctx,
                ANN_SYNC,
                vec![
                    format!("Sync frame indicator: {fr_rx}"),
                    format!("Sync: {fr_rx}"),
                    "Sync".into(),
                ],
            );
        } else if bitnum == 5 {
            // Bit 5: Startup frame indicator.
            self.putx(
                ctx,
                ANN_STARTUP,
                vec![
                    format!("Startup frame indicator: {fr_rx}"),
                    format!("Startup: {fr_rx}"),
                    "Startup".into(),
                ],
            );
        } else if bitnum == 6 {
            // Remember start of ID.
            self.ss_block = Some(ctx.samplenum() as i64);
        } else if bitnum == 16 {
            // Bits 6-16: Frame identifier (ID[10..0]).
            let id = Self::bits_to_int(&self.bits[6..]);
            self.putb(
                ctx,
                ANN_ID,
                vec![format!("Frame ID: {id}"), format!("ID: {id}"), format!("{id}")],
            );
        } else if bitnum == 17 {
            // Remember start of payload length.
            self.ss_block = Some(ctx.samplenum() as i64);
        } else if bitnum == 23 {
            // Bits 17-23: Payload length (Length[7..0]).
            self.payload_length = Self::bits_to_int(&self.bits[17..]);
            let n = self.payload_length;
            self.putb(
                ctx,
                ANN_LENGTH,
                vec![format!("Payload length: {n}"), format!("Length: {n}"), format!("{n}")],
            );
        } else if bitnum == 24 {
            // Remember start of header CRC.
            self.ss_block = Some(ctx.samplenum() as i64);
        } else if bitnum == 34 {
            // Bits 24-34: Header CRC (11-bit) (HCRC[11..0]).
            let expected_crc =
                Self::crc(&self.bits[4..24], C_HCRC_POLYNOMIAL, C_HCRC_SIZE, C_HCRC_INIT);
            let header_crc = Self::bits_to_int(&self.bits[24..]) as u64;
            let crc_ann = if header_crc == expected_crc { "OK" } else { "bad" };
            self.putb(
                ctx,
                ANN_HCRC,
                vec![
                    format!("Header CRC: 0x{header_crc:X} ({crc_ann})"),
                    format!("0x{header_crc:X} ({crc_ann})"),
                    format!("0x{header_crc:X}"),
                ],
            );
        } else if bitnum == 35 {
            // Remember start of cycle code.
            self.ss_block = Some(ctx.samplenum() as i64);
        } else if bitnum == 40 {
            // Bits 35-40: Cycle code (Cyc[6..0]).
            let cycle = Self::bits_to_int(&self.bits[35..]);
            self.putb(
                ctx,
                ANN_CYCLE,
                vec![format!("Cycle: {cycle}"), format!("Cyc: {cycle}"), format!("{cycle}")],
            );
            self.last_databit = 41 + 2 * self.payload_length * 8;
        } else if (41..self.last_databit).contains(&bitnum) {
            // Remember all databyte bits, except the very last one.
            self.ss_databytebits.push(ctx.samplenum() as i64);
        } else if bitnum == self.last_databit {
            // Bits 41-X: Data field; bits within a byte are MSB-first.
            self.ss_databytebits.push(ctx.samplenum() as i64);
            for i in 0..(2 * self.payload_length) {
                let x = (40 + (8 * i) + 1) as usize;
                let b = Self::bits_to_int(&self.bits[x..x + 8]);
                let ss = self.ss_databytebits[(i * 8) as usize];
                let es = self.ss_databytebits[(((i + 1) * 8) - 1) as usize];
                self.putg(
                    ctx,
                    ss,
                    es,
                    ANN_DATA_BYTE,
                    vec![
                        format!("Data byte {i}: 0x{b:02x}"),
                        format!("DB{i}: 0x{b:02x}"),
                        format!("{b:02X}"),
                    ],
                );
            }
            self.ss_databytebits.clear();
            // Remember start of trailer CRC.
            self.ss_block = Some(ctx.samplenum() as i64);
        } else if bitnum == self.last_databit + 23 {
            // Trailer CRC (24-bit) (CRC[11..0]).
            let end = (self.bits.len() as i64 - 24) as usize;
            let expected_crc = Self::crc(
                &self.bits[1..end],
                C_CRC_POLYNOMIAL,
                C_CRC_SIZE,
                if self.channel_b { C_CRC_INIT_B } else { C_CRC_INIT_A },
            );
            let frame_crc = Self::bits_to_int(&self.bits[self.last_databit as usize..]) as u64;
            let crc_ann = if frame_crc == expected_crc { "OK" } else { "bad" };
            self.putb(
                ctx,
                ANN_FRAME_CRC,
                vec![
                    format!("Frame CRC: 0x{frame_crc:X} ({crc_ann})"),
                    format!("0x{frame_crc:X} ({crc_ann})"),
                    format!("0x{frame_crc:X}"),
                ],
            );
            self.end_of_frame = true;
        } else if bitnum == self.last_databit + 24 {
            // Remember start of frame end sequence.
            self.ss_block = Some(ctx.samplenum() as i64);
        } else if bitnum == self.last_databit + 25 {
            // Frame end sequence, must be 1 followed by 0.
            self.putb(ctx, ANN_CID_DELIM, vec!["Frame end sequence".into(), "FES".into()]);
        } else if bitnum == self.last_databit + 26 {
            // Check for DTS.
            if fr_rx == 0 {
                self.dynamic_frame = true;
            } else {
                self.last_xmit_bit = bitnum;
            }
            self.ss_block = Some(ctx.samplenum() as i64);
        } else if bitnum == self.last_xmit_bit {
            // Remember start of channel idle delimiter.
            self.ss_block = Some(ctx.samplenum() as i64);
        } else if bitnum == self.last_xmit_bit + C_CHANNEL_IDLE_DELIMITER - 1 {
            // Channel idle limiter (CID[11..0]).
            self.putb(ctx, ANN_CID, vec!["Channel idle delimiter".into(), "CID".into()]);
            self.reset_variables();
        } else if bitnum > self.last_databit + 27 {
            // DTS if dynamic frame.
            if self.dynamic_frame && fr_rx != 0 && self.last_xmit_bit == 999 {
                self.putb(ctx, ANN_DTS, vec!["Dynamic trailing sequence".into(), "DTS".into()]);
                self.last_xmit_bit = bitnum + 1;
                self.ss_block = Some(ctx.samplenum() as i64);
            }
        }

        self.curbit += 1;
    }
}

impl Decoder for FlexRay {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.channel_b = config.str("channel_type", "A") == "B";
        self.bitrate = config.int("bitrate", 10_000_000) as f64;
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        // pd.py: `self.bit_width = samplerate / float(bitrate)` and
        // `self.sample_point = (bit_width / 100.0) * sample_point_percent`.
        self.bit_width = samplerate as f64 / self.bitrate;
        self.sample_point = (self.bit_width / 100.0) * self.sample_point_percent;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::device("Cannot decode without samplerate."));
        }

        loop {
            match self.state {
                State::Idle => {
                    // Wait for a dominant state (logic 0) on the bus.
                    ctx.wait(&[&[Cond::pin(CH, CondKind::Low)]])?;
                    self.tss_start = Some(ctx.samplenum() as i64);
                    ctx.wait(&[&[Cond::pin(CH, CondKind::High)]])?;
                    self.tss_end = Some(ctx.samplenum() as i64);
                    self.dom_edge_seen(ctx);
                    self.state = State::GetBits;
                }
                State::GetBits => {
                    // Wait until the correct bit/sampling position.
                    let pos = self.get_sample_point(self.curbit);
                    let skip = (pos - ctx.samplenum() as i64).max(0) as u64;
                    let pins =
                        ctx.wait(&[&[Cond::Skip(skip)], &[Cond::pin(CH, CondKind::Falling)]])?;
                    let fr_rx = u8::from(pins.get(CH));
                    let matched = ctx.matched().to_vec();
                    if matched.get(1).copied().unwrap_or(false) {
                        self.dom_edge_seen(ctx);
                    }
                    if matched.first().copied().unwrap_or(false) {
                        self.handle_bit(ctx, fr_rx);
                    }
                }
            }
        }
    }
}
