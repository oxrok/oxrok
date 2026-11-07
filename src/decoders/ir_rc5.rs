// Counterpart in sigrok:
//   libsigrokdecode/decoders/ir_rc5/pd.py,
//   Copyright (C) 2014 Uwe Hermann <uwe@hermann-uwe.de>,
//   and decoders/ir_rc5/lists.py, Copyright (C) 2014 Uwe Hermann,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the RC-5 biphase coding and the decoder's
//   documented classes and texts.
#![allow(clippy::bool_to_int_with_if, clippy::many_single_char_names, clippy::needless_late_init)]

//! IR RC-5: 14 Manchester-coded bits, start bits, toggle, address and
//! command.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const IR: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_STARTBIT1: usize = 1;
const ANN_STARTBIT2: usize = 2;
const ANN_TOGGLE0: usize = 3;
const ANN_TOGGLE1: usize = 4;
const ANN_ADDRESS: usize = 5;
const ANN_COMMAND: usize = 6;

static OPTIONS: [OptionDesc; 2] = [
    OptionDesc {
        id: "polarity",
        desc: "Polarity",
        default: OptionValue::s("active-low"),
        values: &[OptionValue::s("active-low"), OptionValue::s("active-high")],
    },
    OptionDesc {
        id: "protocol",
        desc: "Protocol type",
        default: OptionValue::s("standard"),
        values: &[OptionValue::s("standard"), OptionValue::s("extended")],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ir_rc5",
    name: "IR RC-5",
    longname: "IR RC-5",
    desc: "RC-5 infrared remote control protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["IR"],
    channels: &[ChannelDesc { id: "ir", name: "IR", desc: "IR data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "startbit1", desc: "Startbit 1" },
        AnnClass { id: "startbit2", desc: "Startbit 2" },
        AnnClass { id: "togglebit-0", desc: "Toggle bit 0" },
        AnnClass { id: "togglebit-1", desc: "Toggle bit 1" },
        AnnClass { id: "address", desc: "Address" },
        AnnClass { id: "command", desc: "Command" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "fields", desc: "Fields", classes: &[1, 2, 3, 4, 5, 6] },
    ],
    binary: &[],
    doc: "RC-5 is a biphase/manchester based infrared remote control protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(IrRc5::default())
}

// Systems/addresses (0..31). Items that are not listed are
// reserved/unknown.
const SYSTEM: [(i64, &str, &str); 26] = [
    (0, "TV receiver 1", "TV1"),
    (1, "TV receiver 2", "TV2"),
    (2, "Teletext", "Txt"),
    (3, "Extension to TV1 and TV2", "Ext TV1/TV2"),
    (4, "LaserVision player", "LV"),
    (5, "Video cassette recorder 1", "VCR1"),
    (6, "Video cassette recorder 2", "VCR2"),
    (7, "Experimental", "Exp"),
    (8, "Satellite TV receiver 1", "Sat1"),
    (9, "Extension to VCR1 and VCR2", "Ext VCR1/VCR2"),
    (10, "Satellite TV receiver 2", "Sat2"),
    (12, "Compact disc video player", "CD-Video"),
    (13, "Camcorder", "Cam"),
    (14, "Photo on compact disc player", "CD-Photo"),
    (16, "Audio preamplifier 1", "Preamp1"),
    (17, "Radio tuner", "Tuner"),
    (18, "Analog cassette recoder 1", "Rec1"),
    (19, "Audio preamplifier 2", "Preamp2"),
    (20, "Compact disc player", "CD"),
    (21, "Audio stack or record player", "Combi"),
    (22, "Audio satellite", "Sat"),
    (23, "Analog cassette recoder 2", "Rec2"),
    (26, "Compact disc recorder", "CD-R"),
    (29, "Lighting 1", "Light1"),
    (30, "Lighting 2", "Light2"),
    (31, "Telephone", "Phone"),
];

// Commands (0..63 for RC-5, and 0..127 for Extended RC-5). Items that
// are not listed are reserved/unknown.
const TV_COMMANDS: [(i64, &str, &str); 24] = [
    (0, "0", "0"),
    (1, "1", "1"),
    (2, "2", "2"),
    (3, "3", "3"),
    (4, "4", "4"),
    (5, "5", "5"),
    (6, "6", "6"),
    (7, "7", "7"),
    (8, "8", "8"),
    (9, "9", "9"),
    (10, "-/--", "-/--"),
    (11, "Channel/program", "Ch/P"),
    (12, "Standby", "StBy"),
    (13, "Mute", "M"),
    (14, "Personal preferences", "PP"),
    (15, "Display", "Disp"),
    (16, "Volume up", "Vol+"),
    (17, "Volume down", "Vol-"),
    (18, "Brightness up", "Br+"),
    (19, "Brightness down", "Br-"),
    (20, "Saturation up", "S+"),
    (21, "Saturation down", "S-"),
    (32, "Program up", "P+"),
    (33, "Program down", "P-"),
];

const VCR_COMMANDS: [(i64, &str, &str); 19] = [
    (0, "0", "0"),
    (1, "1", "1"),
    (2, "2", "2"),
    (3, "3", "3"),
    (4, "4", "4"),
    (5, "5", "5"),
    (6, "6", "6"),
    (7, "7", "7"),
    (8, "8", "8"),
    (9, "9", "9"),
    (10, "-/--", "-/--"),
    (12, "Standby", "StBy"),
    (32, "Program up", "P+"),
    (33, "Program down", "P-"),
    (50, "Fast rewind", "FRW"),
    (52, "Fast forward", "FFW"),
    (53, "Play", "Pl"),
    (54, "Stop", "St"),
    (55, "Recording", "Rec"),
];

fn system(a: i64) -> (&'static str, &'static str) {
    SYSTEM.iter().find(|(n, _, _)| *n == a).map_or(("Unknown", "Unk"), |&(_, a, b)| (a, b))
}

fn command(vcr: bool, c: i64) -> (&'static str, &'static str) {
    let table: &[(i64, &str, &str)] = if vcr { &VCR_COMMANDS } else { &TV_COMMANDS };
    table.iter().find(|(n, _, _)| *n == c).map_or(("Unknown", "Unk"), |&(_, a, b)| (a, b))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Mid1,
    Mid0,
    Start1,
    Start0,
}

/// The IR RC-5 decoder.
pub struct IrRc5 {
    samplerate: u64,
    halfbit: i64,
    edges: Vec<i64>,
    bits: Vec<(i64, i64)>,
    ss_es_bits: Vec<(i64, i64)>,
    state: State,
    active_low: bool,
    extended: bool,
    next_edge_low: bool,
}

impl Default for IrRc5 {
    fn default() -> Self {
        IrRc5 {
            samplerate: 0,
            halfbit: 0,
            edges: Vec::new(),
            bits: Vec::new(),
            ss_es_bits: Vec::new(),
            state: State::Idle,
            active_low: true,
            extended: false,
            next_edge_low: true,
        }
    }
}

impl IrRc5 {
    fn putb(
        &self,
        ctx: &mut dyn Context,
        bit1: usize,
        bit2: usize,
        class: usize,
        texts: Vec<String>,
    ) {
        let ss = self.ss_es_bits[bit1].0;
        let es = self.ss_es_bits[bit2].1;
        ctx.put_ann(ss.max(0) as u64, es.max(0) as u64, class, texts);
    }

    fn handle_bits(&mut self, ctx: &mut dyn Context) {
        // Individual raw bits. The first span starts half a bit before
        // the first edge; each next one at the previous span's end.
        for i in 0..14 {
            let ss = if i == 0 {
                (self.bits[0].0 - self.halfbit).max(0)
            } else {
                self.ss_es_bits[i - 1].1
            };
            let es = self.bits[i].0 + self.halfbit;
            self.ss_es_bits.push((ss, es));
            self.putb(ctx, i, i, ANN_BIT, vec![self.bits[i].1.to_string()]);
        }
        let b = self.bits.clone();
        // Bits[0:0]: Startbit 1
        self.putb(
            ctx,
            0,
            0,
            ANN_STARTBIT1,
            vec![
                format!("Startbit1: {}", b[0].1),
                format!("SB1: {}", b[0].1),
                "SB1".into(),
                "S1".into(),
                "S".into(),
            ],
        );
        // Bits[1:1]: Startbit 2, or the inverted command bit 6 of the
        // extended protocol.
        let (idx, s) = if self.extended {
            (
                ANN_COMMAND,
                vec![
                    format!("CMD[6]#: {}", b[1].1),
                    format!("C6#: {}", b[1].1),
                    "C6#".into(),
                    "C#".into(),
                    "C".into(),
                ],
            )
        } else {
            (
                ANN_STARTBIT2,
                vec![
                    format!("Startbit2: {}", b[1].1),
                    format!("SB2: {}", b[1].1),
                    "SB2".into(),
                    "S2".into(),
                    "S".into(),
                ],
            )
        };
        self.putb(ctx, 1, 1, idx, s);
        // Bits[2:2]: Toggle bit
        let s = vec![
            format!("Togglebit: {}", b[2].1),
            format!("Toggle: {}", b[2].1),
            format!("TB: {}", b[2].1),
            "TB".into(),
            "T".into(),
        ];
        self.putb(ctx, 2, 2, if b[2].1 == 0 { ANN_TOGGLE0 } else { ANN_TOGGLE1 }, s);
        // Bits[3:7]: Address (MSB-first)
        let mut a = 0i64;
        for i in 0..5 {
            a |= b[3 + i].1 << (4 - i);
        }
        let x = system(a);
        let s = vec![
            format!("Address: {a} ({})", x.0),
            format!("Addr: {a} ({})", x.1),
            format!("Addr: {a}"),
            format!("A: {a}"),
            "A".into(),
        ];
        self.putb(ctx, 3, 7, ANN_ADDRESS, s);
        // Bits[8:13]: Command (MSB-first)
        let mut c = 0i64;
        for i in 0..6 {
            c |= b[8 + i].1 << (5 - i);
        }
        if self.extended {
            let inverted_bit6 = if b[1].1 == 0 { 1 } else { 0 };
            c |= inverted_bit6 << 6;
        }
        let cmd_type = if x.1 == "VCR1" || x.1 == "VCR2" { "VCR" } else { "TV" };
        let x = command(cmd_type == "VCR", c);
        let s = vec![
            format!("Command: {c} ({})", x.0),
            format!("Cmd: {c} ({})", x.1),
            format!("Cmd: {c}"),
            format!("C: {c}"),
            "C".into(),
        ];
        self.putb(ctx, 8, 13, ANN_COMMAND, s);
    }

    /// Categorize according to distance from the last edge (short/long).
    fn classify(&self, samplenum: i64) -> u8 {
        // 0 = long, 1 = short, 2 = error.
        let distance = samplenum - self.edges.last().copied().unwrap_or(0);
        let s = self.halfbit;
        let l = self.halfbit * 2;
        let margin = self.halfbit / 2;
        if distance >= l - margin && distance <= l + margin {
            return 0;
        }
        if distance >= s - margin && distance <= s + margin {
            return 1;
        }
        2
    }

    fn reset_decoder_state(&mut self) {
        self.edges.clear();
        self.bits.clear();
        self.ss_es_bits.clear();
        self.state = State::Idle;
    }
}

impl Decoder for IrRc5 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.active_low = config.str("polarity", "active-low") == "active-low";
        self.extended = config.str("protocol", "standard") == "extended";
        self.next_edge_low = self.active_low;
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        // One bit: 1.78ms (one half low, one half high).
        self.halfbit = ((samplerate as f64 * 0.001_78) / 2.0) as i64;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }
        loop {
            let pins = ctx.wait(&[&[Cond::Pin(
                IR,
                if self.next_edge_low { CondKind::Low } else { CondKind::High },
            )]])?;
            let ir = pins.get(IR);
            let samplenum = ctx.samplenum() as i64;

            if self.state == State::Idle {
                let bit = 1;
                self.edges.push(samplenum);
                self.bits.push((samplenum, bit));
                self.state = State::Mid1;
                // "self.next_edge = 'l' if self.ir else 'h'"
                self.next_edge_low = ir;
                continue;
            }

            let edge = self.classify(samplenum);
            if edge == 2 {
                self.reset_decoder_state(); // Reset state machine upon errors.
                continue;
            }
            let short = edge == 1;
            let bit: Option<i64>;
            match self.state {
                State::Mid1 => {
                    self.state = if short { State::Start1 } else { State::Mid0 };
                    bit = if short { None } else { Some(0) };
                }
                State::Mid0 => {
                    self.state = if short { State::Start0 } else { State::Mid1 };
                    bit = if short { None } else { Some(1) };
                }
                State::Start1 => {
                    if short {
                        self.state = State::Mid1;
                    }
                    bit = if short { Some(1) } else { None };
                }
                State::Start0 => {
                    if short {
                        self.state = State::Mid0;
                    }
                    bit = if short { Some(0) } else { None };
                }
                State::Idle => unreachable!(),
            }

            self.edges.push(samplenum);
            if let Some(bit) = bit {
                self.bits.push((samplenum, bit));
            }

            if self.bits.len() == 14 {
                self.handle_bits(ctx);
                self.reset_decoder_state();
            }

            self.next_edge_low = ir;
        }
    }
}
