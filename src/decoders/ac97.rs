// Counterpart in sigrok:
//   libsigrokdecode/decoders/ac97/pd.py,
//   Copyright (C) 2017 Gerhard Sittig <gerhard.sittig@gmx.net>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the AC'97 frame layout and the decoder's documented
//   classes and texts.

//! AC'97: 256-bit frames of a 16-bit TAG slot and twelve 20-bit slots.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

// The Python decoder's Pins.
const SYNC: usize = 0;
const BIT_CLK: usize = 1;
const SDATA_OUT: usize = 2;
const SDATA_IN: usize = 3;
const RESET: usize = 4;

// Ann.
const BITS_OUT: usize = 0;
const BITS_IN: usize = 1;
const SLOT_OUT_RAW: usize = 2;
const SLOT_OUT_TAG: usize = 3;
const SLOT_IN_RAW: usize = 16;
const SLOT_IN_TAG: usize = 17;
const WARN: usize = 30;
const ERROR: usize = 31;

// Binary classes.
const BIN_FRAME_OUT: usize = 0;
const BIN_FRAME_IN: usize = 1;
const BIN_SLOT_RAW_OUT: usize = 2;
const BIN_SLOT_RAW_IN: usize = 3;

/// `[0, 16] + [16 + 20 * i for i in range(1, 13)]`.
const SLOT_LENS: [usize; 14] = [0, 16, 36, 56, 76, 96, 116, 136, 156, 176, 196, 216, 236, 256];
const TOTAL_BITS: usize = SLOT_LENS[13];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ac97",
    name: "AC '97",
    longname: "Audio Codec '97",
    desc: "Audio and modem control for PC systems.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Audio", "PC"],
    channels: &[
        ChannelDesc { id: "sync", name: "SYNC", desc: "Frame synchronization" },
        ChannelDesc { id: "clk", name: "BIT_CLK", desc: "Data bits clock" },
    ],
    optional_channels: &[
        ChannelDesc { id: "out", name: "SDATA_OUT", desc: "Data output" },
        ChannelDesc { id: "in", name: "SDATA_IN", desc: "Data input" },
        ChannelDesc { id: "rst", name: "RESET#", desc: "Reset line" },
    ],
    options: &[],
    annotations: &[
        AnnClass { id: "bit-out", desc: "Output bits" },
        AnnClass { id: "bit-in", desc: "Input bits" },
        AnnClass { id: "slot-out-raw", desc: "Output raw value" },
        AnnClass { id: "slot-out-tag", desc: "Output TAG" },
        AnnClass { id: "slot-out-cmd-addr", desc: "Output command address" },
        AnnClass { id: "slot-out-cmd-data", desc: "Output command data" },
        AnnClass { id: "slot-out-03", desc: "Output slot 3" },
        AnnClass { id: "slot-out-04", desc: "Output slot 4" },
        AnnClass { id: "slot-out-05", desc: "Output slot 5" },
        AnnClass { id: "slot-out-06", desc: "Output slot 6" },
        AnnClass { id: "slot-out-07", desc: "Output slot 7" },
        AnnClass { id: "slot-out-08", desc: "Output slot 8" },
        AnnClass { id: "slot-out-09", desc: "Output slot 9" },
        AnnClass { id: "slot-out-10", desc: "Output slot 10" },
        AnnClass { id: "slot-out-11", desc: "Output slot 11" },
        AnnClass { id: "slot-out-io-ctrl", desc: "Output I/O control" },
        AnnClass { id: "slot-in-raw", desc: "Input raw value" },
        AnnClass { id: "slot-in-tag", desc: "Input TAG" },
        AnnClass { id: "slot-in-sts-addr", desc: "Input status address" },
        AnnClass { id: "slot-in-sts-data", desc: "Input status data" },
        AnnClass { id: "slot-in-03", desc: "Input slot 3" },
        AnnClass { id: "slot-in-04", desc: "Input slot 4" },
        AnnClass { id: "slot-in-05", desc: "Input slot 5" },
        AnnClass { id: "slot-in-06", desc: "Input slot 6" },
        AnnClass { id: "slot-in-07", desc: "Input slot 7" },
        AnnClass { id: "slot-in-08", desc: "Input slot 8" },
        AnnClass { id: "slot-in-09", desc: "Input slot 9" },
        AnnClass { id: "slot-in-10", desc: "Input slot 10" },
        AnnClass { id: "slot-in-11", desc: "Input slot 11" },
        AnnClass { id: "slot-in-io-sts", desc: "Input I/O status" },
        AnnClass { id: "warning", desc: "Warning" },
        AnnClass { id: "error", desc: "Error" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits-out", desc: "Output bits", classes: &[BITS_OUT] },
        AnnRow { id: "slots-out-raw", desc: "Output numbers", classes: &[SLOT_OUT_RAW] },
        AnnRow {
            id: "slots-out",
            desc: "Output slots",
            classes: &[
                SLOT_OUT_TAG,
                SLOT_OUT_TAG + 1,
                SLOT_OUT_TAG + 2,
                SLOT_OUT_TAG + 3,
                SLOT_OUT_TAG + 4,
                SLOT_OUT_TAG + 5,
                SLOT_OUT_TAG + 6,
                SLOT_OUT_TAG + 7,
                SLOT_OUT_TAG + 8,
                SLOT_OUT_TAG + 9,
                SLOT_OUT_TAG + 10,
                SLOT_OUT_TAG + 11,
                SLOT_OUT_TAG + 12,
            ],
        },
        AnnRow { id: "bits-in", desc: "Input bits", classes: &[BITS_IN] },
        AnnRow { id: "slots-in-raw", desc: "Input numbers", classes: &[SLOT_IN_RAW] },
        AnnRow {
            id: "slots-in",
            desc: "Input slots",
            classes: &[
                SLOT_IN_TAG,
                SLOT_IN_TAG + 1,
                SLOT_IN_TAG + 2,
                SLOT_IN_TAG + 3,
                SLOT_IN_TAG + 4,
                SLOT_IN_TAG + 5,
                SLOT_IN_TAG + 6,
                SLOT_IN_TAG + 7,
                SLOT_IN_TAG + 8,
                SLOT_IN_TAG + 9,
                SLOT_IN_TAG + 10,
                SLOT_IN_TAG + 11,
                SLOT_IN_TAG + 12,
            ],
        },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[WARN] },
        AnnRow { id: "errors", desc: "Errors", classes: &[ERROR] },
    ],
    binary: &[
        BinClass { id: "frame-out", desc: "Frame bits, output data" },
        BinClass { id: "frame-in", desc: "Frame bits, input data" },
        BinClass { id: "slot-raw-out", desc: "Raw slot bits, output data" },
        BinClass { id: "slot-raw-in", desc: "Raw slot bits, input data" },
    ],
    doc: "AC'97 (Audio Codec '97) is a protocol for audio and modem I/O \
functionality in mainstream PC systems.\n\n\
AC'97 communicates full duplex data (SDATA_IN, SDATA_OUT), where bits \
are clocked by the BIT_CLK and frames are signalled by the SYNC signals. \
A low active RESET# line completes the set of signals.\n\n\
Frames repeat at a nominal frequency of 48kHz, and consist of 256 bits \
each. One 16bit slot contains management information, twelve 20bit slots \
follow which carry data for three management and nine audio/modem channels. \
Optionally two slots of one frame can get combined for higher resolution \
on fewer channels, or double data rate.\n\n\
Details:\n\
http://download.intel.com/support/motherboards/desktop/sb/ac97_r23.pdf",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Ac97::default())
}

fn texts(t: &[&str]) -> Vec<String> {
    t.iter().map(|s| (*s).to_owned()).collect()
}

/// The Python decoder's `bits_to_int()`: an MSB-first bit sequence.
fn bits_to_int(bits: &[u8]) -> u64 {
    let mut value = 0u64;
    for (i, &bit) in bits.iter().enumerate() {
        if bit != 0 {
            value |= 1u64 << (bits.len() - 1 - i);
        }
    }
    value
}

/// The Python decoder's `bits_to_bin_ann()`: eight bits to a byte, MSB
/// first, with a short final byte when the count is not a multiple of 8.
fn bits_to_bin_ann(bits: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut rest = bits;
    let mut count = bits.len() as i64;
    while count > 0 {
        count -= 8;
        let take = rest.len().min(8);
        let (head, tail) = rest.split_at(take);
        out.push(bits_to_int(head) as u8);
        rest = tail;
    }
    out
}

/// The Python decoder's `int_to_nibble_text()`.
fn int_to_nibble_text(value: u64, bitcount: usize) -> String {
    let digits = bitcount.div_ceil(4);
    format!("{value:0digits$x}")
}

/// The Python decoder's `get_bit_field()`.
fn get_bit_field(data: u64, size: usize, off: usize, count: usize) -> u64 {
    let shift = size - off - count;
    let mut data = data >> shift;
    let mask = (1u64 << count) - 1;
    data &= mask;
    data
}

/// The AC'97 decoder.
#[derive(Default)]
pub struct Ac97 {
    frame_ss_list: Option<Vec<u64>>,
    frame_bits_out: Vec<u8>,
    frame_bits_in: Vec<u8>,
    frame_slot_data_out: Vec<u64>,
    frame_slot_data_in: Vec<u64>,
    /// Indexed by direction: 0 = output, 1 = input.
    have_slots: [Option<Vec<bool>>; 2],
}

impl Ac97 {
    fn dir(is_out: bool) -> usize {
        usize::from(!is_out)
    }

    #[allow(clippy::unused_self)]
    fn putx(&self, ctx: &mut dyn Context, ss: u64, es: u64, class: usize, data: Vec<String>) {
        ctx.put_ann(ss, es, class, data);
    }
    /// The Python decoder's `putf()`: a bit range of the current frame.
    fn putf(
        &self,
        ctx: &mut dyn Context,
        frombit: usize,
        bitcount: usize,
        class: usize,
        data: Vec<String>,
    ) {
        if let Some(list) = &self.frame_ss_list {
            let ss = list[frombit];
            let es = list[frombit + bitcount];
            self.putx(ctx, ss, es, class, data);
        }
    }

    /// The Python decoder's `putb()`.
    fn putb(
        &self,
        ctx: &mut dyn Context,
        frombit: usize,
        bitcount: usize,
        class: usize,
        data: Vec<u8>,
    ) {
        if let Some(list) = &self.frame_ss_list {
            let ss = list[frombit];
            let es = list[frombit + bitcount];
            ctx.put_binary(ss, es, class, data);
        }
    }

    /// The Python decoder's `flush_frame_bits()`.
    fn flush_frame_bits(&self, ctx: &mut dyn Context) {
        let count = self.frame_bits_out.len();
        let data = bits_to_bin_ann(&self.frame_bits_out);
        self.putb(ctx, 0, count, BIN_FRAME_OUT, data);

        let count = self.frame_bits_in.len();
        let data = bits_to_bin_ann(&self.frame_bits_in);
        self.putb(ctx, 0, count, BIN_FRAME_IN, data);
    }

    /// The Python decoder's `start_frame()`.
    fn start_frame(&mut self, ctx: &mut dyn Context, ss: u64) {
        // "Flush bits if we had a frame before the frame which is
        // starting here."
        if self.frame_ss_list.as_ref().is_some_and(|list| !list.is_empty()) {
            self.flush_frame_bits(ctx);
        }
        self.frame_ss_list = Some(vec![ss]);
        self.frame_bits_out = Vec::new();
        self.frame_bits_in = Vec::new();
        self.frame_slot_data_out = Vec::new();
        self.frame_slot_data_in = Vec::new();
        self.have_slots = [None, None];
    }

    /// The Python decoder's `handle_slot_dummy()`.
    fn handle_slot_dummy(
        &self,
        ctx: &mut dyn Context,
        slotidx: usize,
        bitidx: usize,
        bitcount: usize,
        is_out: bool,
        data: u64,
    ) {
        // "Only process data of slots 1-12 when slot 0 says valid."
        let dir = Self::dir(is_out);
        if !self.have_slots[dir].as_ref().is_some_and(|slots| slots[slotidx]) {
            return;
        }

        let text = int_to_nibble_text(data, bitcount);
        let anncls = if is_out { SLOT_OUT_TAG } else { SLOT_IN_TAG };
        self.putf(ctx, bitidx, bitcount, anncls + slotidx, vec![text]);

        // "For now the decoder just exports the upper 16 bits of each
        // audio channel that happens to be valid."
        let anncls = if is_out { BIN_SLOT_RAW_OUT } else { BIN_SLOT_RAW_IN };
        let data_bin = ((data >> 4) & 0xffff) as u16;
        self.putb(ctx, bitidx, bitcount, anncls, data_bin.to_be_bytes().to_vec());
    }

    /// The Python decoder's `handle_slot_00()`.
    fn handle_slot_00(
        &mut self,
        ctx: &mut dyn Context,
        slotidx: usize,
        _bitidx: usize,
        bitcount: usize,
        is_out: bool,
        data: u64,
    ) {
        let slotpos = SLOT_LENS[slotidx];
        let mut fieldoff = 0;
        let anncls = if is_out { SLOT_OUT_TAG } else { SLOT_IN_TAG };

        let fieldlen = 1;
        let ready = get_bit_field(data, bitcount, fieldoff, fieldlen);
        let text = if ready != 0 {
            texts(&["READY: 1", "READY", "RDY", "R"])
        } else {
            texts(&["ready: 0", "rdy", "-"])
        };
        self.putf(ctx, slotpos + fieldoff, fieldlen, anncls, text);
        fieldoff += fieldlen;

        let fieldlen = 12;
        let valid = get_bit_field(data, bitcount, fieldoff, fieldlen);
        let text = vec![format!("VALID: {valid:3x}"), format!("{valid:3x}")];
        self.putf(ctx, slotpos + fieldoff, fieldlen, anncls, text);
        let mut have_slots = vec![true];
        have_slots.resize(13, false);
        for idx in 0..12 {
            have_slots[idx + 1] = valid & (1 << (11 - idx)) != 0;
        }
        self.have_slots[Self::dir(is_out)] = Some(have_slots);
        fieldoff += fieldlen;

        let fieldlen = 1;
        let rsv = get_bit_field(data, bitcount, fieldoff, fieldlen);
        if rsv != 0 {
            let text = texts(&["reserved bit error", "rsv error", "rsv"]);
            self.putf(ctx, slotpos + fieldoff, fieldlen, ERROR, text);
        }
        fieldoff += fieldlen;

        // "Will input slot 0 have a Codec ID, or 3 reserved bits?"
        let fieldlen = 2;
        let codec = get_bit_field(data, bitcount, fieldoff, fieldlen);
        let text = vec![format!("CODEC: {codec:1x}"), format!("{codec:1x}")];
        self.putf(ctx, slotpos + fieldoff, fieldlen, anncls, text);
    }

    /// The Python decoder's `handle_slot_01()`.
    fn handle_slot_01(
        &self,
        ctx: &mut dyn Context,
        slotidx: usize,
        _bitidx: usize,
        bitcount: usize,
        is_out: bool,
        data: u64,
    ) {
        let slotpos = SLOT_LENS[slotidx];
        let dir = Self::dir(is_out);
        if !self.have_slots[dir].as_ref().is_some_and(|slots| slots[slotidx]) {
            return;
        }
        let mut fieldoff = 0;
        let anncls = if is_out { SLOT_OUT_TAG } else { SLOT_IN_TAG } + slotidx;

        let fieldlen = 1;
        if is_out {
            let is_read = get_bit_field(data, bitcount, fieldoff, fieldlen);
            let text = if is_read != 0 {
                texts(&["READ", "RD", "R"])
            } else {
                texts(&["WRITE", "WR", "W"])
            };
            self.putf(ctx, slotpos + fieldoff, fieldlen, anncls, text);
        } else {
            let rsv = get_bit_field(data, bitcount, fieldoff, fieldlen);
            if rsv != 0 {
                let text = texts(&["reserved bit error", "rsv error", "rsv"]);
                self.putf(ctx, slotpos + fieldoff, fieldlen, ERROR, text);
            }
        }
        fieldoff += fieldlen;

        let fieldlen = 7;
        let regaddr = get_bit_field(data, bitcount, fieldoff, fieldlen);
        let text = vec![format!("ADDR: {regaddr:2x}"), format!("{regaddr:2x}")];
        self.putf(ctx, slotpos + fieldoff, fieldlen, anncls, text);
        if regaddr & 0x01 != 0 {
            let text = texts(&["odd register address", "odd reg addr", "odd addr", "odd"]);
            self.putf(ctx, slotpos + fieldoff, fieldlen, ERROR, text);
        }
        fieldoff += fieldlen;

        let fieldlen = 10;
        let reqdata = get_bit_field(data, bitcount, fieldoff, fieldlen);
        if is_out && reqdata != 0 {
            let text = texts(&["reserved bit error", "rsv error", "rsv"]);
            self.putf(ctx, slotpos + fieldoff, fieldlen, ERROR, text);
        }
        if !is_out {
            let text = vec![format!("REQ: {reqdata:3x}"), format!("{reqdata:3x}")];
            self.putf(ctx, slotpos + fieldoff, fieldlen, anncls, text);
        }
        fieldoff += fieldlen;

        let fieldlen = 2;
        let rsv = get_bit_field(data, bitcount, fieldoff, fieldlen);
        if rsv != 0 {
            let text = texts(&["reserved bit error", "rsv error", "rsv"]);
            self.putf(ctx, slotpos + fieldoff, fieldlen, ERROR, text);
        }
    }

    /// The Python decoder's `handle_slot_02()`.
    fn handle_slot_02(
        &self,
        ctx: &mut dyn Context,
        slotidx: usize,
        _bitidx: usize,
        bitcount: usize,
        is_out: bool,
        data: u64,
    ) {
        let slotpos = SLOT_LENS[slotidx];
        let dir = Self::dir(is_out);
        if !self.have_slots[dir].as_ref().is_some_and(|slots| slots[slotidx]) {
            return;
        }
        let mut fieldoff = 0;
        let anncls = if is_out { SLOT_OUT_TAG } else { SLOT_IN_TAG } + slotidx;

        let fieldlen = 16;
        let rwdata = get_bit_field(data, bitcount, fieldoff, fieldlen);
        let text = vec![format!("DATA: {rwdata:4x}"), format!("{rwdata:4x}")];
        self.putf(ctx, slotpos + fieldoff, fieldlen, anncls, text);
        fieldoff += fieldlen;

        let fieldlen = 4;
        let rsv = get_bit_field(data, bitcount, fieldoff, fieldlen);
        if rsv != 0 {
            let text = texts(&["reserved bits error", "rsv error", "rsv"]);
            self.putf(ctx, slotpos + fieldoff, fieldlen, ERROR, text);
        }
    }

    /// The Python decoder's `handle_slot()`.
    fn handle_slot(
        &mut self,
        ctx: &mut dyn Context,
        slotidx: usize,
        data_out: Option<u64>,
        data_in: Option<u64>,
    ) {
        let bitidx = SLOT_LENS[slotidx];
        let bitcount = SLOT_LENS[slotidx + 1] - bitidx;
        if let Some(data) = data_out {
            match slotidx {
                0 => self.handle_slot_00(ctx, slotidx, bitidx, bitcount, true, data),
                1 => self.handle_slot_01(ctx, slotidx, bitidx, bitcount, true, data),
                2 => self.handle_slot_02(ctx, slotidx, bitidx, bitcount, true, data),
                _ => self.handle_slot_dummy(ctx, slotidx, bitidx, bitcount, true, data),
            }
        }
        if let Some(data) = data_in {
            match slotidx {
                0 => self.handle_slot_00(ctx, slotidx, bitidx, bitcount, false, data),
                1 => self.handle_slot_01(ctx, slotidx, bitidx, bitcount, false, data),
                2 => self.handle_slot_02(ctx, slotidx, bitidx, bitcount, false, data),
                _ => self.handle_slot_dummy(ctx, slotidx, bitidx, bitcount, false, data),
            }
        }
    }

    /// The Python decoder's `handle_bits()`.
    #[allow(clippy::similar_names)]
    fn handle_bits(
        &mut self,
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        bit_out: Option<u8>,
        bit_in: Option<u8>,
    ) {
        // "Emit the bits' annotations. Only interpret the data when we
        // are in a frame (have seen the start of the frame, and don't
        // exceed the expected number of bits in a frame)."
        if let Some(bit) = bit_out {
            self.putx(ctx, ss, es, BITS_OUT, vec![format!("{bit}")]);
        }
        if let Some(bit) = bit_in {
            self.putx(ctx, ss, es, BITS_IN, vec![format!("{bit}")]);
        }
        if self.frame_ss_list.is_none() {
            return;
        }
        if let Some(list) = &mut self.frame_ss_list {
            list.push(es);
        }
        let have_len = self.frame_ss_list.as_ref().map_or(0, Vec::len) - 1;
        if have_len > TOTAL_BITS {
            return;
        }

        // "Accumulate the bits within the frame, until one slot of the
        // frame has become available."
        let mut slot_idx = 0;
        if let Some(bit) = bit_out {
            self.frame_bits_out.push(bit);
            slot_idx = self.frame_slot_data_out.len();
        }
        if let Some(bit) = bit_in {
            self.frame_bits_in.push(bit);
            slot_idx = self.frame_slot_data_in.len();
        }
        let want_len = SLOT_LENS[slot_idx + 1];
        if have_len != want_len {
            return;
        }
        let prev_len = SLOT_LENS[slot_idx];

        // "Convert bits to integer values."
        let mut slot_data_out = None;
        if bit_out.is_some() {
            let slot_bits = &self.frame_bits_out[prev_len..];
            let slot_data = bits_to_int(slot_bits);
            self.frame_slot_data_out.push(slot_data);
            slot_data_out = Some(slot_data);
        }
        let mut slot_data_in = None;
        if bit_in.is_some() {
            let slot_bits = &self.frame_bits_in[prev_len..];
            let slot_data = bits_to_int(slot_bits);
            self.frame_slot_data_in.push(slot_data);
            slot_data_in = Some(slot_data);
        }

        // "Emit simple annotations for the integer values."
        let slot_len = have_len - prev_len;
        let slot_ss = self.frame_ss_list.as_ref().expect("frame")[prev_len];
        let slot_es = self.frame_ss_list.as_ref().expect("frame")[have_len];
        if let Some(data) = slot_data_out {
            let slot_text = int_to_nibble_text(data, slot_len);
            self.putx(ctx, slot_ss, slot_es, SLOT_OUT_RAW, vec![slot_text]);
        }
        if let Some(data) = slot_data_in {
            let slot_text = int_to_nibble_text(data, slot_len);
            self.putx(ctx, slot_ss, slot_es, SLOT_IN_RAW, vec![slot_text]);
        }

        self.handle_slot(ctx, slot_idx, slot_data_out, slot_data_in);
    }
}

impl Decoder for Ac97 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Ac97::default();
        Ok(())
    }

    #[allow(clippy::too_many_lines, clippy::similar_names)]
    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let have_sdo = ctx.has_channel(SDATA_OUT);
        let have_sdi = ctx.has_channel(SDATA_IN);
        if !have_sdo && !have_sdi {
            return Err(Error::arg("Either SDATA_OUT or SDATA_IN (or both) are required."));
        }
        let _have_reset = ctx.has_channel(RESET);

        // "Data is sampled at falling CLK edges. Annotations need to span
        // the period between rising edges. SYNC rises one cycle _before_
        // the start of a frame. Grab the earliest SYNC sample we can get
        // and advance to the start of a bit time. Then keep getting the
        // samples and the end of all subsequent bit times."
        let mut prev_sync: [Option<u8>; 3] = [None, None, None];
        let mut pins = ctx.wait(&[&[Cond::pin(BIT_CLK, CondKind::Edge)]])?;
        if !pins.get(BIT_CLK) {
            prev_sync[2] = Some(u8::from(pins.get(SYNC)));
            pins = ctx.wait(&[&[Cond::pin(BIT_CLK, CondKind::Rising)]])?;
        }
        let _ = pins;
        let mut bit_ss = ctx.samplenum();
        loop {
            let pins = ctx.wait(&[&[Cond::pin(BIT_CLK, CondKind::Falling)]])?;
            prev_sync.rotate_left(1);
            prev_sync[2] = Some(u8::from(pins.get(SYNC)));
            ctx.wait(&[&[Cond::pin(BIT_CLK, CondKind::Rising)]])?;
            if prev_sync[0] == Some(0) && prev_sync[1] == Some(1) {
                self.start_frame(ctx, bit_ss);
            }
            let out = have_sdo.then(|| u8::from(pins.get(SDATA_OUT)));
            let input = have_sdi.then(|| u8::from(pins.get(SDATA_IN)));
            let now = ctx.samplenum();
            self.handle_bits(ctx, bit_ss, now, out, input);
            bit_ss = now;
        }
    }
}
