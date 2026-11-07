// Counterpart in sigrok:
//   libsigrokdecode/decoders/maple_bus/pd.py,
//   Copyright (C) 2017 Marcus Comstedt <marcus@mc.pp.se>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Maple bus protocol and the decoder's documented
//   classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - The start pattern counts falling SDCKB edges between rising SDCKA
//     edges: 4 is a plain start, 6 a start with CRC, 8 the SDCKB
//     occupancy pattern, 14 or more a RESET. Anything else, and any
//     start with SDCKB high, is a frame error.
//   - Bits are clocked with both lines: a falling SDCKA edge takes
//     SDCKB, a falling SDCKB edge takes SDCKA, and the sequence must
//     alternate. Four B clocks end a byte.
//   - `frame_error()` emits annotation class 7, "Source AP" in the
//     class table, not the class 12 "Frame error" entry. This is the
//     Python decoder's behaviour and is kept.
//   - `if self.pending_bit_pos:` treats sample 0 as absent.
//   - The byte before the checksum, and the first four bytes of a
//     packet, are size/source/dest/command; a mismatch of the final
//     byte with the XOR checksum is a checksum error, and an end
//     pattern whose length is not expected_length + 1 is a size error.

//! Maple bus: the SEGA Dreamcast peripheral protocol.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

// Channel indices, in declaration order.
const SDCKA: usize = 0;
const SDCKB: usize = 1;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_START: usize = 0;
const ANN_END: usize = 1;
const ANN_START_WITH_CRC: usize = 2;
const ANN_OCCUPANCY: usize = 3;
const ANN_RESET: usize = 4;
const ANN_BIT: usize = 5;
// The byte classes follow the six binary classes: size starts at 6.
const ANN_BYTE_BASE: usize = 6;
const ANN_FRAME_ERROR: usize = 7;
const ANN_CHECKSUM_ERROR: usize = 13;
const ANN_SIZE_ERROR: usize = 14;

/// The annotation texts of each byte class, as pd.py's `ann` table.
static ANN_NAMES: [&[&str]; 6] =
    [&["Size", "L"], &["SrcAP", "S"], &["DstAP", "D"], &["Cmd", "C"], &["Data"], &["Cksum", "K"]];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "maple_bus",
    name: "Maple bus",
    longname: "SEGA Maple bus",
    desc: "Maple bus peripheral protocol for SEGA Dreamcast.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Retro computing"],
    channels: &[
        ChannelDesc { id: "sdcka", name: "SDCKA", desc: "Data/clock line A" },
        ChannelDesc { id: "sdckb", name: "SDCKB", desc: "Data/clock line B" },
    ],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "start", desc: "Start pattern" },
        AnnClass { id: "end", desc: "End pattern" },
        AnnClass { id: "start-with-crc", desc: "Start pattern with CRC" },
        AnnClass { id: "occupancy", desc: "SDCKB occupancy pattern" },
        AnnClass { id: "reset", desc: "RESET pattern" },
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "size", desc: "Data size" },
        AnnClass { id: "source", desc: "Source AP" },
        AnnClass { id: "dest", desc: "Destination AP" },
        AnnClass { id: "command", desc: "Command" },
        AnnClass { id: "data", desc: "Data" },
        AnnClass { id: "checksum", desc: "Checksum" },
        AnnClass { id: "frame-error", desc: "Frame error" },
        AnnClass { id: "checksum-error", desc: "Checksum error" },
        AnnClass { id: "size-error", desc: "Size error" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0, 1, 2, 3, 4, 5] },
        AnnRow { id: "fields", desc: "Fields", classes: &[6, 7, 8, 9, 10, 11] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[12, 13, 14] },
    ],
    binary: &[
        BinClass { id: "size", desc: "Data size" },
        BinClass { id: "source", desc: "Source AP" },
        BinClass { id: "dest", desc: "Destination AP" },
        BinClass { id: "command", desc: "Command code" },
        BinClass { id: "data", desc: "Data" },
        BinClass { id: "checksum", desc: "Checksum" },
    ],
    doc: "Maple bus is serial communication protocol used by peripherals for the\n\
          SEGA Dreamcast game console.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(MapleBus::new())
}

struct MapleBus {
    ss: u64,
    es: u64,
    pending_bit_pos: Option<u64>,
    pending_bit: u8,
    data: u32,
    length: usize,
    expected_length: usize,
    checksum: u8,
}

impl MapleBus {
    fn new() -> MapleBus {
        MapleBus {
            ss: 0,
            es: 0,
            pending_bit_pos: None,
            pending_bit: 0,
            data: 0,
            length: 0,
            expected_length: 4,
            checksum: 0,
        }
    }

    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: &[&str]) {
        ctx.put_ann(self.ss, self.es, class, texts.iter().map(|t| (*t).to_owned()).collect());
    }

    fn byte_annotation(bintype: usize, d: u32) -> (usize, Vec<String>) {
        let mut texts: Vec<String> =
            ANN_NAMES[bintype].iter().map(|name| format!("{name}: {d:02X}")).collect();
        texts.push(format!("{d:02X}"));
        (ANN_BYTE_BASE + bintype, texts)
    }

    fn got_start(&self, ctx: &mut dyn Context) {
        self.putx(ctx, ANN_START, &["Start pattern", "Start", "S"]);
    }

    fn got_end(&self, ctx: &mut dyn Context) {
        self.putx(ctx, ANN_END, &["End pattern", "End", "E"]);
        if self.length != self.expected_length + 1 {
            self.putx(ctx, ANN_SIZE_ERROR, &["Size error", "L error", "LE"]);
        }
    }

    fn got_start_with_crc(&self, ctx: &mut dyn Context) {
        self.putx(ctx, ANN_START_WITH_CRC, &["Start pattern with CRC", "Start CRC", "SC"]);
    }

    fn got_occupancy(&self, ctx: &mut dyn Context) {
        self.putx(ctx, ANN_OCCUPANCY, &["SDCKB occupancy pattern", "Occupancy", "O"]);
    }

    fn got_reset(&self, ctx: &mut dyn Context) {
        self.putx(ctx, ANN_RESET, &["RESET pattern", "RESET", "R"]);
    }

    /// pd.py `output_pending_bit()`: `if self.pending_bit_pos:`.
    fn output_pending_bit(&self, ctx: &mut dyn Context) {
        if let Some(pos) = self.pending_bit_pos
            && pos != 0
        {
            ctx.put_ann(
                pos,
                pos,
                ANN_BIT,
                vec![format!("Bit: {}", self.pending_bit), self.pending_bit.to_string()],
            );
        }
    }

    fn got_bit(&mut self, n: u8, ctx: &mut dyn Context) {
        self.output_pending_bit(ctx);
        self.data = self.data * 2 + u32::from(n);
        self.pending_bit = n;
        self.pending_bit_pos = Some(ctx.samplenum());
    }

    fn got_byte(&mut self, ctx: &mut dyn Context) {
        self.output_pending_bit(ctx);
        let mut bintype = 4usize;
        if self.length < 4 {
            if self.length == 0 {
                self.expected_length = 4 * (self.data as usize + 1);
            }
            bintype = self.length;
        } else if self.length == self.expected_length {
            bintype = 5;
            if self.data != u32::from(self.checksum) {
                self.putx(ctx, ANN_CHECKSUM_ERROR, &["Cksum error", "K error", "KE"]);
            }
        }
        self.length += 1;
        self.checksum ^= self.data as u8;
        let (class, texts) = Self::byte_annotation(bintype, self.data);
        ctx.put_ann(self.ss, self.es, class, texts);
        ctx.put_binary(self.ss, self.es, bintype, vec![self.data as u8]);
        self.pending_bit_pos = None;
    }

    fn frame_error(&self, ctx: &mut dyn Context) {
        // pd.py uses class 7 here ("Source AP"), not class 12.
        self.putx(ctx, ANN_FRAME_ERROR, &["Frame error", "F error", "FE"]);
    }

    /// pd.py `handle_start()`: true when a start pattern was found.
    fn handle_start(&mut self, ctx: &mut dyn Context) -> Result<bool> {
        ctx.wait(&[&[Cond::pin(SDCKA, CondKind::Low), Cond::pin(SDCKB, CondKind::High)]])?;
        self.ss = ctx.samplenum();
        let mut count = 0usize;
        loop {
            let pins = ctx.wait(&[
                &[Cond::pin(SDCKB, CondKind::Falling)],
                &[Cond::pin(SDCKA, CondKind::Rising)],
            ])?;
            let matched = ctx.matched().to_vec();
            if matched[0] {
                count += 1;
            }
            if matched[1] {
                self.es = ctx.samplenum();
                let sdckb = pins.0[SDCKB] == 1;
                if sdckb {
                    if count == 4 {
                        self.got_start(ctx);
                        return Ok(true);
                    } else if count == 6 {
                        self.got_start_with_crc(ctx);
                        return Ok(true);
                    } else if count == 8 {
                        self.got_occupancy(ctx);
                        return Ok(false);
                    } else if count >= 14 {
                        self.got_reset(ctx);
                        return Ok(false);
                    }
                }
                self.frame_error(ctx);
                return Ok(false);
            }
        }
    }

    /// pd.py `handle_byte_or_stop()`: true when a byte was received.
    #[allow(clippy::similar_names)]
    fn handle_byte_or_stop(&mut self, ctx: &mut dyn Context) -> Result<bool> {
        self.ss = ctx.samplenum();
        self.pending_bit_pos = None;
        let mut initial = true;
        let mut counta = 0usize;
        let mut countb = 0usize;
        self.data = 0;
        while countb < 4 {
            let pins = ctx.wait(&[
                &[Cond::pin(SDCKA, CondKind::Falling)],
                &[Cond::pin(SDCKB, CondKind::Falling)],
            ])?;
            let matched = ctx.matched().to_vec();
            self.es = ctx.samplenum();
            if matched[0] {
                if counta == countb {
                    self.got_bit(pins.0[SDCKB], ctx);
                    counta += 1;
                } else if counta == 1 && countb == 0 && self.data == 0 && pins.0[SDCKB] == 0 {
                    let hi = [Cond::pin(SDCKA, CondKind::High), Cond::pin(SDCKB, CondKind::High)];
                    let a_fall = [Cond::pin(SDCKA, CondKind::Falling)];
                    let b_fall = [Cond::pin(SDCKB, CondKind::Falling)];
                    ctx.wait(&[&hi, &a_fall, &b_fall])?;
                    let matched = ctx.matched().to_vec();
                    self.es = ctx.samplenum();
                    if matched[0] {
                        self.got_end(ctx);
                    } else {
                        self.frame_error(ctx);
                    }
                    return Ok(false);
                } else {
                    self.frame_error(ctx);
                    return Ok(false);
                }
            } else if matched[1] {
                if counta == countb + 1 {
                    self.got_bit(pins.0[SDCKA], ctx);
                    countb += 1;
                } else if counta == 0 && countb == 0 && pins.0[SDCKA] == 1 && initial {
                    self.ss = ctx.samplenum();
                    initial = false;
                } else {
                    self.frame_error(ctx);
                    return Ok(false);
                }
            }
        }
        ctx.wait(&[&[Cond::pin(SDCKA, CondKind::High)]])?;
        self.es = ctx.samplenum();
        self.got_byte(ctx);
        Ok(true)
    }
}

impl Decoder for MapleBus {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.ss = 0;
        self.es = 0;
        self.pending_bit_pos = None;
        self.pending_bit = 0;
        self.data = 0;
        self.length = 0;
        self.expected_length = 4;
        self.checksum = 0;
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            while !self.handle_start(ctx)? {}
            self.length = 0;
            self.expected_length = 4;
            self.checksum = 0;
            while self.handle_byte_or_stop(ctx)? {}
        }
    }
}
