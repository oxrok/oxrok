// Counterpart in sigrok:
//   libsigrokdecode/decoders/usb_power_delivery/pd.py,
//   Copyright (C) 2015 Google, Inc,
//   Copyright (C) 2018 davidanger <davidanger@163.com>,
//   Copyright (C) 2018 Peter Hazenberg <sigrok@haas-en-berg.nl>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the USB Power Delivery BMC/4b5b framing and the
//   decoder's documented classes and texts.

//! USB Power Delivery: BMC decoding, 4b5b symbols, headers, PDOs, CRC.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue,
};
use std::collections::HashMap;
use std::fmt::Write as _;

const CC1: usize = 0;
const CC2: usize = 1;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_PREAMBLE: usize = 1;
const ANN_SOP: usize = 2;
const ANN_HEADER: usize = 3;
const ANN_DATA: usize = 4;
const ANN_CRC: usize = 5;
const ANN_EOP: usize = 6;
const ANN_SYM: usize = 7;
const ANN_WARN: usize = 8;
const ANN_SRC: usize = 9;
const ANN_SNK: usize = 10;
const ANN_PAYLOAD: usize = 11;
const ANN_TEXT: usize = 12;

/// BMC with a 600kHz datarate: one UI in microseconds.
const UI_US: f64 = 1_000_000.0 / 600_000.0;
/// Threshold to discriminate half-1 from 0 in Binary Mark Coding.
#[allow(clippy::manual_midpoint)] // Keep the Python decoder's expression.
const THRESHOLD_US: f64 = (UI_US + 2.0 * UI_US) / 2.0;

const CTRL_TYPES: [&str; 16] = [
    "reserved",
    "GOOD CRC",
    "GOTO MIN",
    "ACCEPT",
    "REJECT",
    "PING",
    "PS RDY",
    "GET SOURCE CAP",
    "GET SINK CAP",
    "DR SWAP",
    "PR SWAP",
    "VCONN SWAP",
    "WAIT",
    "SOFT RESET",
    "reserved",
    "reserved",
];

fn data_type(t: u32) -> &'static str {
    match t {
        1 => "SOURCE CAP",
        2 => "REQUEST",
        3 => "BIST",
        4 => "SINK CAP",
        5 => "Battery_Status",
        6 => "Alert",
        7 => "Get_Country_Info",
        15 => "VDM",
        _ => "DAT???",
    }
}

/// 4b5b encoding of the symbols.
const DEC4B5B: [u8; 32] = [
    0x10, // Error      00000
    0x10, // Error      00001
    0x10, // Error      00010
    0x10, // Error      00011
    0x10, // Error      00100
    0x10, // Error      00101
    0x13, // Sync-3     00110
    0x14, // RST-1      00111
    0x10, // Error      01000
    0x01, // 1 = 0001   01001
    0x04, // 4 = 0100   01010
    0x05, // 5 = 0101   01011
    0x10, // Error      01100
    0x16, // EOP        01101
    0x06, // 6 = 0110   01110
    0x07, // 7 = 0111   01111
    0x10, // Error      10000
    0x12, // Sync-2     10001
    0x08, // 8 = 1000   10010
    0x09, // 9 = 1001   10011
    0x02, // 2 = 0010   10100
    0x03, // 3 = 0011   10101
    0x0A, // A = 1010   10110
    0x0B, // B = 1011   10111
    0x11, // Sync-1     11000
    0x15, // RST-2      11001
    0x0C, // C = 1100   11010
    0x0D, // D = 1101   11011
    0x0E, // E = 1110   11100
    0x0F, // F = 1111   11101
    0x00, // 0 = 0000   11110
    0x10, // Error      11111
];
const SYNC1: u8 = 0x11;
const SYNC2: u8 = 0x12;
const SYNC3: u8 = 0x13;
const RST1: u8 = 0x14;
const RST2: u8 = 0x15;
const EOP: u8 = 0x16;

const SOP_SEQUENCES: [[u8; 4]; 7] = [
    [SYNC1, SYNC1, SYNC1, SYNC2],
    [SYNC1, SYNC1, SYNC3, SYNC3],
    [SYNC1, SYNC3, SYNC1, SYNC3],
    [SYNC1, RST2, RST2, SYNC3],
    [SYNC1, RST2, SYNC3, SYNC2],
    [RST1, SYNC1, RST1, SYNC3],
    [RST1, RST1, RST1, RST2],
];

const SOP_NAMES: [&str; 7] =
    ["SOP", "SOP'", "SOP\"", "SOP' Debug", "SOP\" Debug", "Cable Reset", "Hard Reset"];

const SYM_NAME: [&[&str; 2]; 23] = [
    &["0x0", "0"],
    &["0x1", "1"],
    &["0x2", "2"],
    &["0x3", "3"],
    &["0x4", "4"],
    &["0x5", "5"],
    &["0x6", "6"],
    &["0x7", "7"],
    &["0x8", "8"],
    &["0x9", "9"],
    &["0xA", "A"],
    &["0xB", "B"],
    &["0xC", "C"],
    &["0xD", "D"],
    &["0xE", "E"],
    &["0xF", "F"],
    &["ERROR", "X"],
    &["SYNC-1", "S1"],
    &["SYNC-2", "S2"],
    &["SYNC-3", "S3"],
    &["RST-1", "R1"],
    &["RST-2", "R2"],
    &["EOP", "#"],
];

/// `RDO_FLAGS`, in the decoder's descending mask order.
const RDO_FLAGS: [(u32, &str); 5] = [
    (1 << 27, "give_back"),
    (1 << 26, "cap_mismatch"),
    (1 << 25, "comm_cap"),
    (1 << 24, "no_suspend"),
    (1 << 23, "unchunked"),
];

fn bist_mode(mode: u32) -> Option<&'static str> {
    Some(match mode {
        0 => "Receiver",
        1 => "Transmit",
        2 => "Counters",
        3 => "Carrier 0",
        4 => "Carrier 1",
        5 => "Carrier 2",
        6 => "Carrier 3",
        7 => "Eye",
        _ => return None,
    })
}

fn vdm_cmd(cmd: u32) -> Option<&'static str> {
    Some(match cmd {
        1 => "Disc Ident",
        2 => "Disc SVID",
        3 => "Disc Mode",
        4 => "Enter Mode",
        5 => "Exit Mode",
        6 => "Attention",
        // 16..31: SVID Specific Commands
        // DisplayPort Commands
        16 => "DP Status",
        17 => "DP Configure",
        _ => return None,
    })
}

const VDM_ACK: [&str; 4] = ["REQ", "ACK", "NAK", "BSY"];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "usb_power_delivery",
    name: "USB PD",
    longname: "USB Power Delivery",
    desc: "USB Power Delivery protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["usb_pd"],
    tags: &["PC"],
    channels: &[ChannelDesc { id: "cc1", name: "CC1", desc: "Configuration Channel 1" }],
    optional_channels: &[ChannelDesc { id: "cc2", name: "CC2", desc: "Configuration Channel 2" }],
    options: &[OptionDesc {
        id: "fulltext",
        desc: "Full text decoding of packets",
        default: OptionValue::s("no"),
        values: &[OptionValue::s("yes"), OptionValue::s("no")],
    }],
    annotations: &[
        AnnClass { id: "type", desc: "Packet Type" },
        AnnClass { id: "preamble", desc: "Preamble" },
        AnnClass { id: "sop", desc: "Start of Packet" },
        AnnClass { id: "header", desc: "Header" },
        AnnClass { id: "data", desc: "Data" },
        AnnClass { id: "crc", desc: "Checksum" },
        AnnClass { id: "eop", desc: "End Of Packet" },
        AnnClass { id: "sym", desc: "4b5b symbols" },
        AnnClass { id: "warnings", desc: "Warnings" },
        AnnClass { id: "src", desc: "Source Message" },
        AnnClass { id: "snk", desc: "Sink Message" },
        AnnClass { id: "payload", desc: "Payload" },
        AnnClass { id: "text", desc: "Plain text" },
    ],
    annotation_rows: &[
        AnnRow { id: "4b5b", desc: "Symbols", classes: &[7] },
        AnnRow { id: "phase", desc: "Parts", classes: &[1, 2, 3, 4, 5, 6] },
        AnnRow { id: "payload", desc: "Payload", classes: &[11] },
        AnnRow { id: "type", desc: "Type", classes: &[0, 9, 10] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[8] },
        AnnRow { id: "text", desc: "Full text", classes: &[12] },
    ],
    binary: &[BinClass { id: "raw-data", desc: "RAW binary data" }],
    doc: "USB Power Delivery - baseband protocol decoder / checker.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(UsbPowerDelivery::default())
}

/// Format a float as Python's `%g` does, with its default precision of 6.
fn py_g(x: f64) -> String {
    if x == 0.0 {
        return if x.is_sign_negative() { "-0".into() } else { "0".into() };
    }
    let sci = format!("{x:.5e}");
    let (mant, exp) = sci.split_once('e').expect("exponential formatting");
    let exp: i32 = exp.parse().expect("an exponent");
    if !(-4..6).contains(&exp) {
        let mut mant = mant.to_owned();
        if mant.contains('.') {
            while mant.ends_with('0') {
                mant.pop();
            }
            if mant.ends_with('.') {
                mant.pop();
            }
        }
        let sign = if exp < 0 { '-' } else { '+' };
        return format!("{mant}e{sign}{:02}", exp.abs());
    }
    let decimals = (5 - exp).max(0) as usize;
    let mut text = format!("{x:.decimals$}");
    if text.contains('.') {
        while text.ends_with('0') {
            text.pop();
        }
        if text.ends_with('.') {
            text.pop();
        }
    }
    text
}

/// `zlib.crc32` of the little-endian header and payload words.
fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    crc ^ 0xFFFF_FFFF
}

/// The USB Power Delivery decoder.
#[derive(Default)]
pub struct UsbPowerDelivery {
    samplerate: u64,
    maxbit: u64,
    threshold: u64,
    fulltext: bool,
    idx: i64,
    packet_seq: u64,
    previous: u64,
    startsample: Option<u64>,
    bits: Vec<u8>,
    edges: Vec<u64>,
    half_one: bool,
    start_one: u64,
    stored_pdos: HashMap<usize, String>,
    cap_mark: [u32; 8],
    head: u32,
    data: Vec<u32>,
    text: String,
}

impl UsbPowerDelivery {
    fn us2samples(&self, us: f64) -> u64 {
        (us * self.samplerate as f64 / 1_000_000.0) as u64
    }

    /// `self.edges[i]`, with Python's negative indexing.
    fn edge(&self, i: i64) -> u64 {
        let n = self.edges.len() as i64;
        let index = if i < 0 { i + n } else { i };
        self.edges[index as usize]
    }

    /// `self.putx(s0, s1, data)` of the Python decoder.
    fn putx(&self, ctx: &mut dyn Context, s0: i64, s1: i64, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.edge(s0), self.edge(s1), class, texts);
    }

    fn putwarn(&self, ctx: &mut dyn Context, long: String, short: &str) {
        self.putx(ctx, 0, -1, ANN_WARN, vec![long, short.to_owned()]);
    }

    fn sym_at(&self, i: usize) -> u8 {
        let v = self.bits[i]
            | (self.bits[i + 1] << 1)
            | (self.bits[i + 2] << 2)
            | (self.bits[i + 3] << 3)
            | (self.bits[i + 4] << 4);
        DEC4B5B[v as usize]
    }

    fn rec_sym(&self, ctx: &mut dyn Context, i: usize, sym: u8) {
        self.putx(
            ctx,
            i as i64,
            i as i64 + 5,
            ANN_SYM,
            SYM_NAME[sym as usize].iter().map(|t| (*t).to_owned()).collect(),
        );
    }

    fn get_sym(&self, ctx: &mut dyn Context, i: usize) -> u8 {
        let sym = self.sym_at(i);
        self.rec_sym(ctx, i, sym);
        sym
    }

    fn get_short(&mut self, ctx: &mut dyn Context) -> u32 {
        let i = self.idx as usize;
        // Check it's not a truncated packet.
        if self.bits.len().saturating_sub(i) <= 20 {
            self.putwarn(ctx, "Truncated".into(), "!");
            return 0x0BAD;
        }
        let k = [
            self.get_sym(ctx, i),
            self.get_sym(ctx, i + 5),
            self.get_sym(ctx, i + 10),
            self.get_sym(ctx, i + 15),
        ];
        let val = u32::from(k[0])
            | (u32::from(k[1]) << 4)
            | (u32::from(k[2]) << 8)
            | (u32::from(k[3]) << 12);
        self.idx += 20;
        val
    }

    fn get_word(&mut self, ctx: &mut dyn Context) -> u32 {
        let lo = self.get_short(ctx);
        let hi = self.get_short(ctx);
        lo | (hi << 16)
    }

    fn find_corrupted_sop(k: [u8; 4]) -> Option<&'static str> {
        // Start of packet are valid even if they have only 3 correct
        // symbols out of 4.
        for (seq, name) in SOP_SEQUENCES.iter().zip(SOP_NAMES) {
            if (0..4).filter(|&i| k[i] == seq[i]).count() >= 3 {
                return Some(name);
            }
        }
        None
    }

    fn scan_eop(&mut self, ctx: &mut dyn Context) -> i64 {
        for i in 0..self.bits.len().saturating_sub(19) {
            let k = [self.sym_at(i), self.sym_at(i + 5), self.sym_at(i + 10), self.sym_at(i + 15)];
            let mut sym = SOP_SEQUENCES
                .iter()
                .zip(SOP_NAMES)
                .find(|(seq, _)| **seq == k)
                .map(|(_, name)| name);
            if sym.is_none() {
                sym = Self::find_corrupted_sop(k);
            }
            // We have an interesting symbol sequence.
            if let Some(sym) = sym {
                // Annotate the preamble.
                self.putx(ctx, 0, i as i64, ANN_PREAMBLE, vec!["Preamble".into(), "...".into()]);
                // Annotate each symbol.
                for (j, s) in k.iter().enumerate() {
                    self.rec_sym(ctx, i + j * 5, *s);
                }
                if sym == "Hard Reset" {
                    self.text += "HRST";
                    return -1; // Hard reset
                }
                if sym == "Cable Reset" {
                    self.text += "CRST";
                    return -1; // Cable reset
                }
                self.putx(ctx, i as i64, i as i64 + 20, ANN_SOP, vec![sym.to_owned(), "S".into()]);
                return i as i64 + 20;
            }
        }
        self.putx(
            ctx,
            0,
            self.bits.len() as i64,
            ANN_PREAMBLE,
            vec!["Junk???".into(), "XXX".into()],
        );
        self.text += "Junk???";
        self.putwarn(ctx, "No start of packet found".into(), "XXX");
        -1 // No Start Of Packet
    }

    fn get_request(&self, rdo: u32) -> String {
        let pos = ((rdo >> 28) & 7) as usize;

        let t_settings = match self.cap_mark[pos] {
            3 => {
                let op_v = f64::from((rdo >> 9) & 0x7ff) * 0.02;
                let op_a = f64::from(rdo & 0x3f) * 0.05;
                format!("{}V {}A", py_g(op_v), py_g(op_a))
            }
            2 => {
                let op_w = f64::from((rdo >> 10) & 0x3ff) * 0.25;
                format!("{}W (operating)", py_g(op_w))
            }
            _ => {
                let op_a = f64::from((rdo >> 10) & 0x3ff) * 0.01;
                let max_a = f64::from(rdo & 0x3ff) * 0.01;
                format!("{}A (operating) / {}A (max)", py_g(op_a), py_g(max_a))
            }
        };

        let mut t_flags = String::new();
        for (mask, name) in RDO_FLAGS {
            if rdo & mask != 0 {
                let _ = write!(t_flags, " [{name}]");
            }
        }

        let t_pdo = match self.stored_pdos.get(&pos) {
            Some(pdo) => format!("#{pos}: {pdo}"),
            None => format!("#{pos}"),
        };

        format!("(PDO {t_pdo}) {t_settings}{t_flags}")
    }

    #[allow(clippy::verbose_bit_mask)] // Keep the Python decoder's masks.
    fn get_source_sink_cap(&mut self, pdo: u32, idx: usize, source: bool) -> String {
        let t1 = (pdo >> 30) & 3;
        self.cap_mark[idx] = t1;

        let flags: Vec<(u32, &str)> = match t1 {
            0 => {
                if source {
                    vec![
                        (1 << 29, "dual_role_power"),
                        (1 << 28, "suspend"),
                        (1 << 27, "unconstrained"),
                        (1 << 26, "comm_cap"),
                        (1 << 25, "dual_role_data"),
                        (1 << 24, "unchunked"),
                    ]
                } else {
                    // Sink
                    vec![
                        (1 << 29, "dual_role_power"),
                        (1 << 28, "high_capability"),
                        (1 << 27, "unconstrained"),
                        (1 << 26, "comm_cap"),
                        (1 << 25, "dual_role_data"),
                        // Sorted by descending mask, as the Python decoder
                        // iterates `sorted(flags.keys(), reverse=True)`.
                        (0b11 << 23, "fr_swap 3.0 A"),
                        (0b10 << 23, "fr_swap 1.5 A"),
                        (0b01 << 23, "fr_swap default power"),
                    ]
                }
            }
            1 | 2 => Vec::new(), // No flags defined for Battery/Variable PDO in PD 3.0 spec
            _ => {
                if (pdo >> 28) & 3 == 0 {
                    vec![(1 << 29, "power_limited")]
                } else {
                    Vec::new()
                }
            }
        };

        let (t_name, p) = match t1 {
            0 => {
                let mv = f64::from((pdo >> 10) & 0x3ff) * 0.05;
                let ma = f64::from(pdo & 0x3ff) * 0.01;
                let p = format!("{}V {}A ({}W)", py_g(mv), py_g(ma), py_g(mv * ma));
                self.stored_pdos.insert(idx, format!("Fixed {}V", py_g(mv)));
                ("Fixed", p)
            }
            1 => {
                let minv = f64::from((pdo >> 10) & 0x3ff) * 0.05;
                let maxv = f64::from((pdo >> 20) & 0x3ff) * 0.05;
                let mw = f64::from(pdo & 0x3ff) * 0.25;
                let p = format!("{}/{}V {}W", py_g(minv), py_g(maxv), py_g(mw));
                self.stored_pdos.insert(idx, format!("Battery {}/{}V", py_g(minv), py_g(maxv)));
                ("Battery", p)
            }
            2 => {
                let minv = f64::from((pdo >> 10) & 0x3ff) * 0.05;
                let maxv = f64::from((pdo >> 20) & 0x3ff) * 0.05;
                let ma = f64::from(pdo & 0x3ff) * 0.01;
                let p = format!("{}/{}V {}A", py_g(minv), py_g(maxv), py_g(ma));
                self.stored_pdos.insert(idx, format!("Variable {}/{}V", py_g(minv), py_g(maxv)));
                ("Variable", p)
            }
            _ => {
                let t2 = (pdo >> 28) & 3;
                if t2 == 0 {
                    let minv = f64::from((pdo >> 8) & 0xff) * 0.1;
                    let maxv = f64::from((pdo >> 17) & 0xff) * 0.1;
                    let ma = f64::from(pdo & 0xff) * 0.05;
                    let mut p = format!("{}/{}V {}A", py_g(minv), py_g(maxv), py_g(ma));
                    if (pdo >> 27) & 1 != 0 {
                        p += " [limited]";
                    }
                    self.stored_pdos
                        .insert(idx, format!("Programmable|PPS {}/{}V", py_g(minv), py_g(maxv)));
                    ("Programmable|PPS", p)
                } else {
                    let t_name = format!("Reserved APDO: {}", bin(t2));
                    let p = format!("[raw: {}]", bin(pdo));
                    self.stored_pdos.insert(idx, format!("{t_name} {p}"));
                    return format!("[{t_name}] {p}");
                }
            }
        };

        let mut t_flags = String::new();
        for &(mask, name) in &flags {
            if pdo & mask != 0 {
                let _ = write!(t_flags, " [{name}]");
            }
        }
        format!("[{t_name}] {p}{t_flags}")
    }

    fn get_vdm(idx: usize, data: u32) -> String {
        if idx == 0 {
            // VDM header
            let vid = data >> 16;
            if data & (1 << 15) != 0 {
                // Structured VDM
                let cmd = data & 0x1f;
                let ack = (data >> 6) & 3;
                let pos = (data >> 8) & 7;
                let mut txt = format!("{} ", VDM_ACK[ack as usize]);
                txt += vdm_cmd(cmd).unwrap_or("cmd?");
                if pos != 0 {
                    let _ = write!(txt, " pos {pos}");
                } else {
                    txt.push(' ');
                }
                let _ = write!(txt, " SVID:{vid:04x}");
                txt
            } else {
                // Unstructured VDM
                format!("unstruct [{:04x}] SVID:{vid:04x}", data & 0x7fff)
            }
        } else {
            // VDM payload
            format!("VDO:{data:08x}")
        }
    }

    fn get_bist(idx: usize, data: u32) -> String {
        let mode = data >> 28;
        let counter = data & 0xffff;
        let mode_name = if mode == 2 {
            format!("Counter[= {counter}]")
        } else {
            bist_mode(mode).unwrap_or("INVALID").to_owned()
        };
        // TODO: Check all 0 bits are 0 / emit warnings.
        if idx == 0 { format!("mode {mode_name}") } else { "invalid BRO".into() }
    }

    fn putpayload(&mut self, ctx: &mut dyn Context, s0: i64, s1: i64, idx: usize) {
        let t = self.head_type();
        let mut txt = format!("[{}] ", idx + 1);
        if t == 2 {
            txt += &self.get_request(self.data[idx]);
        } else if t == 1 || t == 4 {
            txt += &self.get_source_sink_cap(self.data[idx], idx + 1, t == 1);
        } else if t == 15 {
            txt += &Self::get_vdm(idx, self.data[idx]);
        } else if t == 3 {
            txt += &Self::get_bist(idx, self.data[idx]);
        }
        self.putx(ctx, s0, s1, ANN_PAYLOAD, vec![txt.clone(), txt.clone()]);
        self.text += " - ";
        self.text += &txt;
    }

    fn puthead(&mut self, ctx: &mut dyn Context) {
        let power = self.head_power_role();
        let ann_type = if power { ANN_SRC } else { ANN_SNK };
        let mut role = if power { "SRC" } else { "SNK" }.to_owned();
        if self.head_data_role() != power {
            role += if self.head_data_role() { "/DFP" } else { "/UFP" };
        }
        let t = self.head_type();
        let shortm = if self.head_count() == 0 { CTRL_TYPES[t as usize] } else { data_type(t) };

        let longm = format!("(r{}) {}[{}]: {}", self.head_rev(), role, self.head_id(), shortm);
        self.putx(ctx, 0, -1, ann_type, vec![longm.clone(), shortm.to_owned()]);
        self.text += &longm;
    }

    fn head_id(&self) -> u32 {
        (self.head >> 9) & 7
    }

    fn head_power_role(&self) -> bool {
        (self.head >> 8) & 1 != 0
    }

    fn head_data_role(&self) -> bool {
        (self.head >> 5) & 1 != 0
    }

    fn head_rev(&self) -> u32 {
        ((self.head >> 6) & 3) + 1
    }

    fn head_type(&self) -> u32 {
        self.head & 0xF
    }

    fn head_count(&self) -> u32 {
        (self.head >> 12) & 7
    }

    fn compute_crc32(&self) -> u32 {
        let mut data = Vec::new();
        data.extend_from_slice(&((self.head & 0xffff) as u16).to_le_bytes());
        for d in &self.data {
            data.extend_from_slice(&d.to_le_bytes());
        }
        crc32(&data)
    }

    fn decode_packet(&mut self, ctx: &mut dyn Context) {
        self.data = Vec::new();
        self.idx = 0;
        self.text = String::new();

        if self.edges.len() < 50 {
            return; // Not a real PD packet
        }

        self.packet_seq += 1;
        let tstamp = self.startsample.unwrap_or(0) as f64 / self.samplerate as f64;
        let _ = write!(self.text, "#{:<4} ({:8.6}ms): ", self.packet_seq, tstamp * 1000.0);

        self.idx = self.scan_eop(ctx);
        if self.idx < 0 {
            // Full text trace of the issue.
            self.putx(ctx, 0, self.idx, ANN_TEXT, vec![self.text.clone(), "...".into()]);
            return; // No real packet: ABORT.
        }

        // Packet header
        self.head = self.get_short(ctx);
        self.putx(
            ctx,
            self.idx - 20,
            self.idx,
            ANN_HEADER,
            vec![format!("H:{:04x}", self.head), "HD".into()],
        );
        self.puthead(ctx);

        // Decode data payload
        for i in 0..self.head_count() as usize {
            let word = self.get_word(ctx);
            self.data.push(word);
            self.putx(
                ctx,
                self.idx - 40,
                self.idx,
                ANN_DATA,
                vec![format!("[{i}]{:08x}", self.data[i]), format!("D{i}")],
            );
            self.putpayload(ctx, self.idx - 40, self.idx, i);
        }

        // CRC check
        let crc = self.get_word(ctx);
        let computed_crc = self.compute_crc32();
        if crc != computed_crc {
            self.putwarn(ctx, format!("Bad CRC {crc:08x} != {computed_crc:08x}"), "CRC!");
        }
        self.putx(
            ctx,
            self.idx - 40,
            self.idx,
            ANN_CRC,
            vec![format!("CRC:{crc:08x}"), "CRC".into()],
        );

        // End of Packet
        if self.bits.len() >= self.idx as usize + 5 && self.get_sym(ctx, self.idx as usize) == EOP {
            self.putx(ctx, self.idx, self.idx + 5, ANN_EOP, vec!["EOP".into(), "E".into()]);
            self.idx += 5;
        } else {
            self.putwarn(ctx, "No EOP".into(), "EOP!");
        }
        // Full text trace
        if self.fulltext {
            self.putx(ctx, 0, self.idx, ANN_TEXT, vec![self.text.clone(), "...".into()]);
        }

        // Meta data for bitrate
        let first_edge = self.edge(0);
        let last_edge = self.edge(-1);
        let bitrate =
            self.samplerate as f64 * self.bits.len() as f64 / (last_edge - first_edge) as f64;
        // The Python decoder passes the edges in this (reversed) order.
        ctx.put_meta(last_edge, first_edge, "Bitrate", Item::Int(bitrate as i64));
        // Raw binary data (BMC decoded)
        ctx.put_binary(last_edge, first_edge, 0, self.bits.clone());
    }

    #[allow(clippy::too_many_lines)]
    fn decode_loop(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }
        let skip = (self.samplerate as f64 / 1e3) as u64;
        loop {
            let _pins = ctx.wait(&[
                &[Cond::pin(CC1, CondKind::Edge)],
                &[Cond::pin(CC2, CondKind::Edge)],
                &[Cond::Skip(skip)],
            ])?;

            // First sample of the packet, just record the start date.
            if self.startsample.unwrap_or(0) == 0 {
                self.startsample = Some(ctx.samplenum());
                self.previous = ctx.samplenum();
                continue;
            }

            let diff = ctx.samplenum() - self.previous;

            // Large idle: use it as the end of packet.
            if diff > self.maxbit {
                // The last edge of the packet.
                self.edges.push(self.previous);
                // Export the packet.
                self.decode_packet(ctx);
                // Reset for next packet.
                self.startsample = Some(ctx.samplenum());
                self.bits.clear();
                self.edges.clear();
                self.half_one = false;
                self.start_one = 0;
            } else {
                // Add the bit to the packet.
                let is_zero = diff > self.threshold;
                if is_zero && !self.half_one {
                    self.bits.push(0);
                    self.edges.push(self.previous);
                } else if !is_zero && self.half_one {
                    self.bits.push(1);
                    self.edges.push(self.start_one);
                    self.half_one = false;
                } else if !is_zero && !self.half_one {
                    self.half_one = true;
                    self.start_one = self.previous;
                } else {
                    // Invalid BMC sequence
                    // TODO: Try to recover.
                    self.bits.push(0);
                    self.edges.push(self.previous);
                    self.half_one = false;
                }
            }
            self.previous = ctx.samplenum();
        }
    }
}

/// Python's `bin()` for a non-negative integer.
fn bin(v: u32) -> String {
    format!("0b{v:b}")
}

impl Decoder for UsbPowerDelivery {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.fulltext = config.str("fulltext", "no") == "yes";
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        // 0 is 2 UI, space larger than 1.5x 0 is definitely wrong.
        self.maxbit = self.us2samples(3.0 * UI_US);
        // Duration threshold between half 1 and 0.
        self.threshold = self.us2samples(THRESHOLD_US);
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.decode_loop(ctx)
    }
}
