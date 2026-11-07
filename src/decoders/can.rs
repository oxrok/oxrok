// Counterpart in sigrok:
//   libsigrokdecode/decoders/can/pd.py,
//   Copyright (C) 2012-2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2019 Stephan Thiele <stephan.thiele@mailbox.org>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the CAN framing and the decoder's documented classes and
//   texts.
//
// Subtle points of the Python original that this port keeps:
//   - Bit timing is a "poor man's clock synchronization": a dominant edge
//     resets `dom_edge_snum`/`dom_edge_bcount`, and the next sample point is
//     `dom_edge_snum + bit_width * (curbit - dom_edge_bcount) + sample_point`.
//   - `wait([{'skip': pos - samplenum}, {0: 'f'}])`: the skip count is clamped
//     to zero by the Python host, and a falling edge in the same sample is
//     handled before `handle_bit` (so a matched edge moves the reference first).
//   - Stuff bits are detected on the last six *raw* bits and popped from
//     `bits`; the `bits` list therefore keeps only frame bits.
//   - After the DLC only `last_databit + 17` bits are still checked for
//     stuffing (`if len(self.bits) > self.last_databit + 17: return False`).
//   - An FD frame switches to the fast bitrate on a recessive BRS bit before
//     the stuff-bit check, and the CRC delimiter switches back to nominal.

//! CAN: classic and CAN FD frames, standard and extended identifiers.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const CAN_RX: usize = 0;

// Annotation classes, as the libsigrokdecode decoder numbers them.
const ANN_DATA: usize = 0;
const ANN_SOF: usize = 1;
const ANN_EOF: usize = 2;
const ANN_ID: usize = 3;
const ANN_EXT_ID: usize = 4;
const ANN_FULL_ID: usize = 5;
const ANN_IDE: usize = 6;
const ANN_RESERVED: usize = 7;
const ANN_RTR: usize = 8;
const ANN_SRR: usize = 9;
const ANN_DLC: usize = 10;
const ANN_CRC_SEQUENCE: usize = 11;
const ANN_CRC_DELIMITER: usize = 12;
const ANN_ACK_SLOT: usize = 13;
const ANN_ACK_DELIMITER: usize = 14;
const ANN_STUFF_BIT: usize = 15;
const ANN_WARNINGS: usize = 16;
const ANN_BIT: usize = 17;

static OPTIONS: [OptionDesc; 3] = [
    OptionDesc {
        id: "nominal_bitrate",
        desc: "Nominal bitrate (bits/s)",
        default: OptionValue::i(1_000_000),
        values: &[],
    },
    OptionDesc {
        id: "fast_bitrate",
        desc: "Fast bitrate (bits/s)",
        default: OptionValue::i(2_000_000),
        values: &[],
    },
    OptionDesc {
        id: "sample_point",
        desc: "Sample point (%)",
        default: OptionValue::f(70.0),
        values: &[],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "can",
    name: "CAN",
    longname: "Controller Area Network",
    desc: "Field bus protocol for distributed realtime control.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Automotive"],
    channels: &[ChannelDesc { id: "can_rx", name: "CAN RX", desc: "CAN bus line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "data", desc: "CAN payload data" },
        AnnClass { id: "sof", desc: "Start of frame" },
        AnnClass { id: "eof", desc: "End of frame" },
        AnnClass { id: "id", desc: "Identifier" },
        AnnClass { id: "ext-id", desc: "Extended identifier" },
        AnnClass { id: "full-id", desc: "Full identifier" },
        AnnClass { id: "ide", desc: "Identifier extension bit" },
        AnnClass { id: "reserved-bit", desc: "Reserved bit 0 and 1" },
        AnnClass { id: "rtr", desc: "Remote transmission request" },
        AnnClass { id: "srr", desc: "Substitute remote request" },
        AnnClass { id: "dlc", desc: "Data length count" },
        AnnClass { id: "crc-sequence", desc: "CRC sequence" },
        AnnClass { id: "crc-delimiter", desc: "CRC delimiter" },
        AnnClass { id: "ack-slot", desc: "ACK slot" },
        AnnClass { id: "ack-delimiter", desc: "ACK delimiter" },
        AnnClass { id: "stuff-bit", desc: "Stuff bit" },
        AnnClass { id: "warnings", desc: "Human-readable warnings" },
        AnnClass { id: "bit", desc: "Bit" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[15, 17] },
        AnnRow {
            id: "fields",
            desc: "Fields",
            classes: &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14],
        },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[16] },
    ],
    binary: &[],
    doc: "CAN (Controller Area Network) is a differential field bus for \
distributed realtime control. The decoder reads one bus line (CAN RX), \
recognizes standard and extended frames, classic and CAN FD, and \
annotates the identifier, control and data fields, the CRC and \
acknowledge slots and every bit. Bit stuffing is undone for the frame \
fields, and an FD frame's BRS bit switches to the fast bitrate.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Can::default())
}

/// The 4-bit DLC to payload length table.
fn dlc2len(dlc: usize) -> usize {
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64][dlc]
}

fn texts(t: &[&str]) -> Vec<String> {
    t.iter().map(|s| (*s).to_owned()).collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    GetBits,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FrameType {
    Standard,
    Extended,
}

/// The CAN decoder.
pub struct Can {
    /// Options.
    nominal_bitrate: f64,
    fast_bitrate: f64,
    sample_point_pct: f64,
    /// From the metadata.
    samplerate: u64,
    bit_width: f64,
    sample_point: f64,
    /// Frame decoding state (`reset_variables()`).
    state: State,
    frame_type: Option<FrameType>,
    dlc: usize,
    /// All bits, including stuff bits.
    rawbits: Vec<u8>,
    /// Only actual CAN frame bits, no stuff bits.
    bits: Vec<u8>,
    /// Current raw bit of the frame, which counts stuff bits.
    curbit: u32,
    /// Positive value that bitnum + x will never match while idle.
    last_databit: u32,
    ss_block: u64,
    ss_bit12: u64,
    ss_bit32: u64,
    ss_databytebits: Vec<u64>,
    fd: bool,
    rtr: u8,
    dlc_start: u32,
    crc_len: u32,
    /// Clock synchronization reference.
    dom_edge_snum: u64,
    dom_edge_bcount: u32,
}

impl Default for Can {
    fn default() -> Self {
        Can {
            nominal_bitrate: 1_000_000.0,
            fast_bitrate: 2_000_000.0,
            sample_point_pct: 70.0,
            samplerate: 0,
            bit_width: 0.0,
            sample_point: 0.0,
            state: State::Idle,
            frame_type: None,
            dlc: 0,
            rawbits: Vec::new(),
            bits: Vec::new(),
            curbit: 0,
            last_databit: 999,
            ss_block: 0,
            ss_bit12: 0,
            ss_bit32: 0,
            ss_databytebits: Vec::new(),
            fd: false,
            rtr: 0,
            dlc_start: 0,
            crc_len: 0,
            dom_edge_snum: 0,
            dom_edge_bcount: 0,
        }
    }
}

impl Can {
    fn set_bit_rate(&mut self, bitrate: f64) {
        self.bit_width = self.samplerate as f64 / bitrate;
        self.sample_point = (self.bit_width / 100.0) * self.sample_point_pct;
    }

    fn set_nominal_bitrate(&mut self) {
        let rate = self.nominal_bitrate;
        self.set_bit_rate(rate);
    }

    fn set_fast_bitrate(&mut self) {
        let rate = self.fast_bitrate;
        self.set_bit_rate(rate);
    }

    fn reset_variables(&mut self) {
        self.state = State::Idle;
        self.frame_type = None;
        self.dlc = 0;
        self.rawbits.clear();
        self.bits.clear();
        self.curbit = 0;
        self.last_databit = 999;
        self.ss_block = 0;
        self.ss_bit12 = 0;
        self.ss_bit32 = 0;
        self.ss_databytebits.clear();
        self.fd = false;
        self.rtr = 0;
    }

    /// Generic helper for CAN bit annotations.
    fn putg(&self, ctx: &mut dyn Context, ss: u64, es: u64, class: usize, texts: Vec<String>) {
        let left = self.sample_point as u64;
        let right = (self.bit_width - self.sample_point) as u64;
        ctx.put_ann(ss.saturating_sub(left), es + right, class, texts);
    }

    /// Single-CAN-bit annotation using the current samplenum.
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        let s = ctx.samplenum();
        self.putg(ctx, s, s, class, texts);
    }

    /// Single-CAN-bit annotation using the samplenum of CAN bit 12.
    fn put12(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        let s = self.ss_bit12;
        self.putg(ctx, s, s, class, texts);
    }

    /// Single-CAN-bit annotation using the samplenum of CAN bit 32.
    fn put32(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        let s = self.ss_bit32;
        self.putg(ctx, s, s, class, texts);
    }

    /// Multi-CAN-bit annotation from `ss_block` to the current samplenum.
    fn putb(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        let e = ctx.samplenum();
        let s = self.ss_block;
        self.putg(ctx, s, e, class, texts);
    }

    /// Poor man's clock synchronization: remember the sample and raw bit of
    /// a dominant edge.
    fn dom_edge_seen(&mut self, samplenum: u64) {
        self.dom_edge_snum = samplenum;
        self.dom_edge_bcount = self.curbit;
    }

    /// The sample position of the next desired bit.
    fn get_sample_point(&self, bitnum: u32) -> u64 {
        let mut samplenum = self.dom_edge_snum as f64;
        samplenum += self.bit_width * (f64::from(bitnum) - f64::from(self.dom_edge_bcount));
        samplenum += self.sample_point;
        samplenum as u64
    }

    fn is_stuff_bit(&mut self) -> bool {
        // CAN uses NRZ encoding and bit stuffing. After 5 identical bits, a
        // stuff bit of opposite value is added. But not in the CRC delimiter,
        // ACK, and end of frame fields.
        if self.bits.len() > self.last_databit as usize + 17 {
            return false;
        }
        if self.rawbits.len() < 6 {
            return false;
        }
        let last = &self.rawbits[self.rawbits.len() - 6..];
        if !(last == &[0, 0, 0, 0, 0, 1][..] || last == &[1, 1, 1, 1, 1, 0][..]) {
            return false;
        }
        // Stuff bit. Keep it in rawbits, but drop it from bits.
        self.bits.pop();
        true
    }

    fn is_valid_crc(crc_bits: &[u8]) -> bool {
        let _ = crc_bits;
        true // TODO: as in the Python decoder.
    }

    /// Handle CRC, ACK and EOF. Returns true if the frame ended.
    fn decode_frame_end(&mut self, ctx: &mut dyn Context, can_rx: u8, bitnum: u32) -> bool {
        // Remember start of CRC sequence (see below).
        if bitnum == self.last_databit + 1 {
            self.ss_block = ctx.samplenum();
            if self.fd {
                if dlc2len(self.dlc) < 16 {
                    self.crc_len = 27; // 17 + SBC + stuff bits
                } else {
                    self.crc_len = 32; // 21 + SBC + stuff bits
                }
            } else {
                self.crc_len = 15;
            }
        // CRC sequence (15 bits, 17 bits or 21 bits)
        } else if bitnum == self.last_databit + self.crc_len {
            let crc_type = if self.fd {
                if dlc2len(self.dlc) < 16 { "CRC-17" } else { "CRC-21" }
            } else {
                "CRC-15"
            };

            let x = self.last_databit as usize + 1;
            // Python takes `bits[x:x + crc_len + 1]`, which is every bit from
            // the CRC start to the current one because the slice is clamped.
            let crc_bits = &self.bits[x..];
            let crc = crc_bits.iter().fold(0u32, |v, &b| (v << 1) | u32::from(b));
            self.putb(
                ctx,
                ANN_CRC_SEQUENCE,
                vec![
                    format!("{crc_type} sequence: 0x{crc:04x}"),
                    format!("{crc_type}: 0x{crc:04x}"),
                    crc_type.to_owned(),
                ],
            );
            if !Self::is_valid_crc(crc_bits) {
                self.putb(ctx, ANN_WARNINGS, texts(&["CRC is invalid"]));
            }
        // CRC delimiter bit (recessive)
        } else if bitnum == self.last_databit + self.crc_len + 1 {
            self.putx(
                ctx,
                ANN_CRC_DELIMITER,
                vec![
                    format!("CRC delimiter: {can_rx}"),
                    format!("CRC d: {can_rx}"),
                    "CRC d".to_owned(),
                ],
            );
            if can_rx != 1 {
                self.putx(ctx, ANN_WARNINGS, texts(&["CRC delimiter must be a recessive bit"]));
            }

            if self.fd {
                self.set_nominal_bitrate();
            }
        // ACK slot bit (dominant: ACK, recessive: NACK)
        } else if bitnum == self.last_databit + self.crc_len + 2 {
            let ack = if can_rx == 0 { "ACK" } else { "NACK" };
            self.putx(
                ctx,
                ANN_ACK_SLOT,
                vec![format!("ACK slot: {ack}"), format!("ACK s: {ack}"), "ACK s".to_owned()],
            );
        // ACK delimiter bit (recessive)
        } else if bitnum == self.last_databit + self.crc_len + 3 {
            self.putx(
                ctx,
                ANN_ACK_DELIMITER,
                vec![
                    format!("ACK delimiter: {can_rx}"),
                    format!("ACK d: {can_rx}"),
                    "ACK d".to_owned(),
                ],
            );
            if can_rx != 1 {
                self.putx(ctx, ANN_WARNINGS, texts(&["ACK delimiter must be a recessive bit"]));
            }
        // Remember start of EOF (see below).
        } else if bitnum == self.last_databit + self.crc_len + 4 {
            self.ss_block = ctx.samplenum();
        // End of frame (EOF), 7 recessive bits
        } else if bitnum == self.last_databit + self.crc_len + 10 {
            self.putb(ctx, ANN_EOF, texts(&["End of frame", "EOF", "E"]));
            let n = self.rawbits.len();
            let ok = n >= 7 && self.rawbits[n - 7..] == [1, 1, 1, 1, 1, 1, 1][..];
            if !ok {
                self.putb(
                    ctx,
                    ANN_WARNINGS,
                    texts(&["End of frame (EOF) must be 7 recessive bits"]),
                );
            }
            self.reset_variables();
            return true;
        }

        false
    }

    /// Decode a standard frame bit. Returns true if the frame ended.
    fn decode_standard_frame(&mut self, ctx: &mut dyn Context, can_rx: u8, bitnum: u32) -> bool {
        // Bit 14: FDF (Flexible data format). Has to be sent dominant when FD
        // frame, has to be sent recessive when classic CAN frame.
        if bitnum == 14 {
            self.fd = can_rx != 0;
            if self.fd {
                self.putx(
                    ctx,
                    ANN_RESERVED,
                    vec![
                        format!("Flexible data format: {can_rx}"),
                        format!("FDF: {can_rx}"),
                        "FDF".to_owned(),
                    ],
                );
            } else {
                self.putx(
                    ctx,
                    ANN_RESERVED,
                    vec![
                        format!("Reserved bit 0: {can_rx}"),
                        format!("RB0: {can_rx}"),
                        "RB0".to_owned(),
                    ],
                );
            }

            if self.fd {
                // Bit 12: Substitute remote request (SRR) bit. The Python
                // decoder tags this with class 8 (rtr), not 9.
                self.put12(ctx, ANN_RTR, texts(&["Substitute remote request", "SRR"]));
                self.dlc_start = 18;
            } else {
                // Bit 12: Remote transmission request (RTR) bit. Data frame:
                // dominant, remote frame: recessive. Remote frames do not
                // contain a data field.
                let rtr = if self.bits[12] == 1 { "remote" } else { "data" };
                self.put12(
                    ctx,
                    ANN_RTR,
                    vec![
                        format!("Remote transmission request: {rtr} frame"),
                        format!("RTR: {rtr} frame"),
                        "RTR".to_owned(),
                    ],
                );
                self.dlc_start = 15;
            }
        }

        if bitnum == 15 && self.fd {
            self.putx(
                ctx,
                ANN_RESERVED,
                vec![format!("Reserved: {can_rx}"), format!("R0: {can_rx}"), "R0".to_owned()],
            );
        }

        if bitnum == 16 && self.fd {
            self.putx(
                ctx,
                ANN_RESERVED,
                vec![
                    format!("Bit rate switch: {can_rx}"),
                    format!("BRS: {can_rx}"),
                    "BRS".to_owned(),
                ],
            );
        }

        if bitnum == 17 && self.fd {
            self.putx(
                ctx,
                ANN_RESERVED,
                vec![
                    format!("Error state indicator: {can_rx}"),
                    format!("ESI: {can_rx}"),
                    "ESI".to_owned(),
                ],
            );
        // Remember start of DLC (see below).
        } else if bitnum == self.dlc_start {
            self.ss_block = ctx.samplenum();
        // Bits 15-18: Data length code (DLC), in number of bytes (0-8).
        } else if bitnum == self.dlc_start + 3 {
            let start = self.dlc_start as usize;
            let dlc =
                self.bits[start..start + 4].iter().fold(0usize, |v, &b| (v << 1) | b as usize);
            self.dlc = dlc;
            self.putb(
                ctx,
                ANN_DLC,
                vec![format!("Data length code: {dlc}"), format!("DLC: {dlc}"), "DLC".to_owned()],
            );
            self.last_databit = self.dlc_start + 3 + (dlc2len(dlc) as u32 * 8);
            if dlc > 8 && !self.fd {
                self.putb(ctx, ANN_WARNINGS, texts(&["Data length code (DLC) > 8 is not allowed"]));
            }
        // Remember all databyte bits, except the very last one.
        } else if bitnum >= self.dlc_start + 4 && bitnum < self.last_databit {
            self.ss_databytebits.push(ctx.samplenum());
        // Bits 19-X: Data field. The bits within a data byte are transferred
        // MSB-first.
        } else if bitnum == self.last_databit {
            self.ss_databytebits.push(ctx.samplenum()); // Last databyte bit.
            self.put_data_bytes(ctx);
        } else if bitnum > self.last_databit {
            return self.decode_frame_end(ctx, can_rx, bitnum);
        }

        false
    }

    /// Decode an extended frame bit. Returns true if the frame ended.
    #[allow(clippy::too_many_lines)]
    fn decode_extended_frame(&mut self, ctx: &mut dyn Context, can_rx: u8, bitnum: u32) -> bool {
        // Remember start of EID (see below).
        if bitnum == 14 {
            self.ss_block = ctx.samplenum();
            self.fd = false;
            self.dlc_start = 35;
        // Bits 14-31: Extended identifier (EID[17..0])
        } else if bitnum == 31 {
            let eid = self.bits[14..].iter().fold(0u32, |v, &b| (v << 1) | u32::from(b));
            let s = format!("{eid} (0x{eid:x})");
            self.putb(
                ctx,
                ANN_EXT_ID,
                vec![
                    format!("Extended Identifier: {s}"),
                    format!("Extended ID: {s}"),
                    "Extended ID".to_owned(),
                    "EID".to_owned(),
                ],
            );

            let id = self.bits[1..12].iter().fold(0u32, |v, &b| (v << 1) | u32::from(b));
            let fullid = (id << 18) | eid;
            let s = format!("{fullid} (0x{fullid:x})");
            self.putb(
                ctx,
                ANN_FULL_ID,
                vec![
                    format!("Full Identifier: {s}"),
                    format!("Full ID: {s}"),
                    "Full ID".to_owned(),
                    "FID".to_owned(),
                ],
            );

            // Bit 12: Substitute remote request (SRR) bit
            let b12 = self.bits[12];
            self.put12(
                ctx,
                ANN_SRR,
                vec![
                    format!("Substitute remote request: {b12}"),
                    format!("SRR: {b12}"),
                    "SRR".to_owned(),
                ],
            );
        }

        // Bit 32: Remote transmission request (RTR) bit. Data frame: dominant,
        // remote frame: recessive. Remote frames do not contain a data field.
        if bitnum == 32 {
            self.ss_bit32 = ctx.samplenum();
            self.rtr = can_rx;

            if !self.fd {
                let rtr = if can_rx == 1 { "remote" } else { "data" };
                self.putx(
                    ctx,
                    ANN_RTR,
                    vec![
                        format!("Remote transmission request: {rtr} frame"),
                        format!("RTR: {rtr} frame"),
                        "RTR".to_owned(),
                    ],
                );
            }
        // Bit 33: RB1 (reserved bit)
        } else if bitnum == 33 {
            self.fd = can_rx != 0;
            if self.fd {
                self.dlc_start = 37;
                self.putx(
                    ctx,
                    ANN_RESERVED,
                    vec![
                        format!("Flexible data format: {can_rx}"),
                        format!("FDF: {can_rx}"),
                        "FDF".to_owned(),
                    ],
                );
                let rtr = self.rtr;
                self.put32(
                    ctx,
                    ANN_RESERVED,
                    vec![format!("Reserved bit 1: {rtr}"), format!("RB1: {rtr}"), "RB1".to_owned()],
                );
            } else {
                self.putx(
                    ctx,
                    ANN_RESERVED,
                    vec![
                        format!("Reserved bit 1: {can_rx}"),
                        format!("RB1: {can_rx}"),
                        "RB1".to_owned(),
                    ],
                );
            }
        // Bit 34: RB0 (reserved bit)
        } else if bitnum == 34 {
            self.putx(
                ctx,
                ANN_RESERVED,
                vec![
                    format!("Reserved bit 0: {can_rx}"),
                    format!("RB0: {can_rx}"),
                    "RB0".to_owned(),
                ],
            );
        } else if bitnum == 35 && self.fd {
            self.putx(
                ctx,
                ANN_RESERVED,
                vec![
                    format!("Bit rate switch: {can_rx}"),
                    format!("BRS: {can_rx}"),
                    "BRS".to_owned(),
                ],
            );
        } else if bitnum == 36 && self.fd {
            self.putx(
                ctx,
                ANN_RESERVED,
                vec![
                    format!("Error state indicator: {can_rx}"),
                    format!("ESI: {can_rx}"),
                    "ESI".to_owned(),
                ],
            );
        // Remember start of DLC (see below).
        } else if bitnum == self.dlc_start {
            self.ss_block = ctx.samplenum();
        // Bits 35-38: Data length code (DLC), in number of bytes (0-8).
        } else if bitnum == self.dlc_start + 3 {
            let start = self.dlc_start as usize;
            let dlc =
                self.bits[start..start + 4].iter().fold(0usize, |v, &b| (v << 1) | b as usize);
            self.dlc = dlc;
            self.putb(
                ctx,
                ANN_DLC,
                vec![format!("Data length code: {dlc}"), format!("DLC: {dlc}"), "DLC".to_owned()],
            );
            self.last_databit = self.dlc_start + 3 + (dlc2len(dlc) as u32 * 8);
        // Remember all databyte bits, except the very last one.
        } else if bitnum >= self.dlc_start + 4 && bitnum < self.last_databit {
            self.ss_databytebits.push(ctx.samplenum());
        // Bits 39-X: Data field. The bits within a data byte are transferred
        // MSB-first.
        } else if bitnum == self.last_databit {
            self.ss_databytebits.push(ctx.samplenum()); // Last databyte bit.
            self.put_data_bytes(ctx);
        } else if bitnum > self.last_databit {
            return self.decode_frame_end(ctx, can_rx, bitnum);
        }

        false
    }

    /// Emit the data bytes of a complete data field.
    fn put_data_bytes(&mut self, ctx: &mut dyn Context) {
        let start = self.dlc_start as usize;
        for i in 0..dlc2len(self.dlc) {
            let x = start + 4 + (8 * i);
            let b = self.bits[x..x + 8].iter().fold(0u32, |v, &bit| (v << 1) | u32::from(bit));
            let ss = self.ss_databytebits[i * 8];
            let es = self.ss_databytebits[(i + 1) * 8 - 1];
            self.putg(
                ctx,
                ss,
                es,
                ANN_DATA,
                vec![
                    format!("Data byte {i}: 0x{b:02x}"),
                    format!("DB {i}: 0x{b:02x}"),
                    "DB".to_owned(),
                ],
            );
        }
        self.ss_databytebits.clear();
    }

    fn handle_bit(&mut self, ctx: &mut dyn Context, can_rx: u8) {
        self.rawbits.push(can_rx);
        self.bits.push(can_rx);

        // Get the index of the current CAN frame bit (without stuff bits).
        let bitnum = (self.bits.len() - 1) as u32;

        if self.fd && can_rx != 0 {
            let special = match self.frame_type {
                Some(FrameType::Standard) => bitnum == 16,
                Some(FrameType::Extended) => bitnum == 35,
                None => false,
            };
            if special {
                self.dom_edge_seen(ctx.samplenum());
                self.set_fast_bitrate();
            }
        }

        // If this is a stuff bit, remove it from bits and ignore it.
        if self.is_stuff_bit() {
            self.putx(ctx, ANN_STUFF_BIT, vec![can_rx.to_string()]);
            self.curbit += 1; // Increase curbit (bitnum is not affected).
            return;
        }
        self.putx(ctx, ANN_BIT, vec![can_rx.to_string()]);

        // Bit 0: Start of frame (SOF) bit
        if bitnum == 0 {
            self.putx(ctx, ANN_SOF, texts(&["Start of frame", "SOF", "S"]));
            if can_rx != 0 {
                self.putx(
                    ctx,
                    ANN_WARNINGS,
                    texts(&["Start of frame (SOF) must be a dominant bit"]),
                );
            }
        // Remember start of ID (see below).
        } else if bitnum == 1 {
            self.ss_block = ctx.samplenum();
        // Bits 1-11: Identifier (ID[10..0]). The bits ID[10..4] must NOT be
        // all recessive.
        } else if bitnum == 11 {
            let id = self.bits[1..].iter().fold(0u32, |v, &b| (v << 1) | u32::from(b));
            let s = format!("{id} (0x{id:x})");
            self.putb(
                ctx,
                ANN_ID,
                vec![format!("Identifier: {s}"), format!("ID: {s}"), "ID".to_owned()],
            );
            if (id & 0x7f0) == 0x7f0 {
                self.putb(
                    ctx,
                    ANN_WARNINGS,
                    texts(&["Identifier bits 10..4 must not be all recessive"]),
                );
            }
        // RTR or SRR bit, depending on frame type (gets handled later).
        } else if bitnum == 12 {
            self.ss_bit12 = ctx.samplenum();
        // Bit 13: Identifier extension (IDE) bit. Standard frame: dominant,
        // extended frame: recessive.
        } else if bitnum == 13 {
            let ide = if can_rx == 0 { "standard" } else { "extended" };
            self.frame_type =
                Some(if can_rx == 0 { FrameType::Standard } else { FrameType::Extended });
            self.putx(
                ctx,
                ANN_IDE,
                vec![
                    format!("Identifier extension bit: {ide} frame"),
                    format!("IDE: {ide} frame"),
                    "IDE".to_owned(),
                ],
            );
        // Bits 14-X: Frame-type dependent.
        } else if bitnum >= 14 {
            let done = match self.frame_type {
                Some(FrameType::Standard) => self.decode_standard_frame(ctx, can_rx, bitnum),
                _ => self.decode_extended_frame(ctx, can_rx, bitnum),
            };
            // The handlers return true if a frame ended (EOF).
            if done {
                return;
            }
        }

        // After a frame there are 3 intermission bits (recessive). After these
        // bits, the bus is considered free.
        self.curbit += 1;
    }
}

impl Decoder for Can {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.nominal_bitrate = config.float("nominal_bitrate", 1_000_000.0);
        self.fast_bitrate = config.float("fast_bitrate", 2_000_000.0);
        self.sample_point_pct = config.float("sample_point", 70.0);
        self.samplerate = 0;
        self.bit_width = 0.0;
        self.sample_point = 0.0;
        self.reset_variables();
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        // As in the Python decoder's metadata(): the bit width and sample
        // point are also recomputed here, from the nominal bitrate.
        self.samplerate = samplerate;
        let rate = self.nominal_bitrate;
        self.bit_width = self.samplerate as f64 / rate;
        self.sample_point = (self.bit_width / 100.0) * self.sample_point_pct;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }

        loop {
            match self.state {
                State::Idle => {
                    // Wait for a dominant state (logic 0) on the bus.
                    ctx.wait(&[&[Cond::Pin(CAN_RX, CondKind::Low)]])?;
                    self.dom_edge_seen(ctx.samplenum());
                    self.state = State::GetBits;
                }
                State::GetBits => {
                    // Wait until we're in the correct bit/sampling position.
                    let pos = self.get_sample_point(self.curbit);
                    let now = ctx.samplenum();
                    // The Python host clamps a negative skip to zero.
                    let skip = pos.saturating_sub(now);
                    let groups: [&[Cond]; 2] =
                        [&[Cond::Skip(skip)], &[Cond::Pin(CAN_RX, CondKind::Falling)]];
                    let pins = ctx.wait(&groups)?;
                    let can_rx = u8::from(pins.get(CAN_RX));
                    let matched = ctx.matched().to_vec();
                    if matched.get(1).copied().unwrap_or(false) {
                        self.dom_edge_seen(ctx.samplenum());
                    }
                    if matched.first().copied().unwrap_or(false) {
                        self.handle_bit(ctx, can_rx);
                    }
                }
            }
        }
    }
}
