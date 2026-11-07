// Counterpart in sigrok:
//   libsigrokdecode/decoders/dali/pd.py,
//   Copyright (C) 2015 Jeremy Swanson <jeremy@rakocontrols.com>,
//   and decoders/dali/lists.py, same copyright,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the DALI biphase coding and the decoder's
//   documented classes and texts.
#![allow(
    clippy::bool_to_int_with_if,
    clippy::many_single_char_names,
    clippy::redundant_else,
    clippy::struct_field_names
)]

//! DALI: forward and backward Manchester frames with addresses,
//! commands and replies.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Config, Context, Decoder, DecoderDef, DecoderInfo, OptionDesc,
    OptionValue,
};

const DALI: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_STARTBIT: usize = 1;
const ANN_YBIT: usize = 3;
const ANN_ADDRESS: usize = 4;
const ANN_COMMAND: usize = 5;
const ANN_REPLY: usize = 6;
const ANN_RAW: usize = 7;

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "polarity",
    desc: "Polarity",
    default: OptionValue::s("active-low"),
    values: &[OptionValue::s("active-low"), OptionValue::s("active-high")],
}];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "dali",
    name: "DALI",
    longname: "Digital Addressable Lighting Interface",
    desc: "Digital Addressable Lighting Interface (DALI) protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Embedded/industrial", "Lighting"],
    channels: &[ChannelDesc { id: "dali", name: "DALI", desc: "DALI data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "startbit", desc: "Startbit" },
        AnnClass { id: "sbit", desc: "Select bit" },
        AnnClass { id: "ybit", desc: "Individual or group" },
        AnnClass { id: "address", desc: "Address" },
        AnnClass { id: "command", desc: "Command" },
        AnnClass { id: "reply", desc: "Reply data" },
        AnnClass { id: "raw", desc: "Raw data" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "raw", desc: "Raw data", classes: &[7] },
        AnnRow { id: "fields", desc: "Fields", classes: &[1, 2, 3, 4, 5, 6] },
    ],
    binary: &[],
    doc: "DALI is a biphase/manchester based lighting control protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Dali::default())
}

// DALI extended commands.
const EXTENDED_COMMANDS: [(i64, &str, &str); 17] = [
    (0xA1, "Terminate special processes", "Terminate"),
    (0xA3, "DTR = DATA", "DTR"),
    (0xA5, "INITIALISE", "INIT"),
    (0xA7, "RANDOMISE", "RAND"),
    (0xA9, "COMPARE", "COMP"),
    (0xAB, "WITHDRAW", "WDRAW"),
    (0xB1, "SET SEARCH H", "SAH"),
    (0xB3, "SET SEARCH M", "SAM"),
    (0xB5, "SET SEARCH L", "SAL"),
    (0xB7, "Program Short Address", "ProgSA"),
    (0xB9, "Verify Short Address", "VfySA"),
    (0xBB, "Query Short Address", "QryShort"),
    (0xBD, "Physical Selection", "PysSel"),
    (0xC1, "Enable Device Type X", "EnTyp"),
    (0xC3, "DTR1 = DATA", "DTR1"),
    (0xC5, "DTR2 = DATA", "DTR2"),
    (0xC7, "Write Memory Location", "WRI"),
];

// List of commands.
const DALI_COMMANDS: [(i64, &str, &str); 46] = [
    (0x00, "Immediate Off", "IOFF"),
    (0x01, "Up 200ms", "Up"),
    (0x02, "Down 200ms", "Down"),
    (0x03, "Step Up", "Step+"),
    (0x04, "Step Down", "Step-"),
    (0x05, "Recall Maximum Level", "Recall Max"),
    (0x06, "Recall Minimum Level", "Recall Min"),
    (0x07, "Step down and off", "Down Off"),
    (0x08, "Step ON and UP", "On Up"),
    (0x20, "Reset", "Rst"),
    (0x21, "Store Dim Level in DTR", "Level -> DTR"),
    (0x2A, "Store DTR as Max Level", "DTR->Max"),
    (0x2B, "Store DTR as Min Level", "DTR->Min"),
    (0x2C, "Store DTR as Fail Level", "DTR->Fail"),
    (0x2D, "Store DTR as Power On Level", "DTR->Poweron"),
    (0x2E, "Store DTR as Fade Time", "DTR->Fade"),
    (0x2F, "Store DTR as Fade Rate", "DTR->Rate"),
    (0x80, "Store DTR as Short Address", "DTR->Add"),
    (0x81, "Enable Memory Write", "WEn"),
    (0x90, "Query Status", "Status"),
    (0x91, "Query Ballast", "Ballast"),
    (0x92, "Query Lamp Failure", "LmpFail"),
    (0x93, "Query Power On", "Power On"),
    (0x94, "Query Limit Error", "Limit Err"),
    (0x95, "Query Reset", "Reset State"),
    (0x96, "Query Missing Short Address", "NoSrt"),
    (0x97, "Query Version", "Ver"),
    (0x98, "Query DTR", "GetDTR"),
    (0x99, "Query Device Type", "Type"),
    (0x9A, "Query Physical Minimum", "PhysMin"),
    (0x9B, "Query Power Fail", "PowerFailed"),
    (0x9C, "Query DTR1", "GetDTR1"),
    (0x9D, "Query DTR2", "GetDTR2"),
    (0xA0, "Query Level", "GetLevel"),
    (0xA1, "Query Max Level", "GetMax"),
    (0xA2, "Query Min Level", "GetMin"),
    (0xA3, "Query Power On", "GetPwrOn"),
    (0xA4, "Query Fail Level", "GetFail"),
    (0xA5, "Query Fade Rate", "GetRate"),
    (0xA6, "Query Power Fail", "PwrFail"),
    (0xC0, "Query Groups 0-7", "GetGrpsL"),
    (0xC1, "Query Groups 7-15", "GetGrpsH"),
    (0xC2, "Query BRNH", "BRNH"),
    (0xC3, "Query BRNM", "BRNM"),
    (0xC4, "Query BRNL", "BRNL"),
    (0xC5, "Query Memory", "GetMem"),
];

// DALI device type 8.
const DALI_DEVICE_TYPE8: [(i64, &str, &str); 5] = [
    (0xE0, "Set Temp X-Y Coordinate", "Set X-Y"),
    (0xE2, "Activate Colour Set point", "Activate SetPoint"),
    (0xE7, "Set Colour Temperature Tc", "DTRs->ColTemp"),
    (0xF9, "Query Features", "QryFeats"),
    (0xFA, "Query Current Setpoint Colour", "GetSetPoint"),
];

fn lookup3(
    table: &'static [(i64, &'static str, &'static str)],
    key: i64,
) -> (&'static str, &'static str) {
    table.iter().find(|&&(k, _, _)| k == key).map_or(("Unknown", "Unk"), |&(_, a, b)| (a, b))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Phase0,
    Phase1,
}

/// The DALI decoder.
pub struct Dali {
    samplerate: u64,
    halfbit: i64,
    edges: Vec<i64>,
    bits: Vec<(i64, i64)>,
    ss_es_bits: Vec<(i64, i64)>,
    state: State,
    dev_type: Option<i64>,
    old_dali: i64,
    active_high: bool,
    phase0: i64,
}

impl Default for Dali {
    fn default() -> Self {
        Dali {
            samplerate: 0,
            halfbit: 0,
            edges: Vec::new(),
            bits: Vec::new(),
            ss_es_bits: Vec::new(),
            state: State::Idle,
            dev_type: None,
            old_dali: 1,
            active_high: false,
            phase0: 0,
        }
    }
}

impl Dali {
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

    #[allow(clippy::too_many_lines)]
    fn handle_bits(&mut self, ctx: &mut dyn Context, length: usize) {
        let b = self.bits.clone();
        // Individual raw bits.
        for i in 0..length {
            let ss = if i == 0 { self.bits[0].0.max(0) } else { self.ss_es_bits[i - 1].1 };
            let es = self.bits[i].0 + self.halfbit * 2;
            self.ss_es_bits.push((ss, es));
            self.putb(ctx, i, i, ANN_BIT, vec![self.bits[i].1.to_string()]);
        }
        // Bits[0:0]: Startbit.
        let s = vec![
            format!("Startbit: {}", b[0].1),
            format!("ST: {}", b[0].1),
            "ST".into(),
            "S".into(),
            "S".into(),
        ];
        self.putb(ctx, 0, 0, ANN_STARTBIT, s.clone());
        self.putb(ctx, 0, 0, ANN_RAW, s);
        // Bits[1:8]
        let mut f = 0i64;
        for i in 0..8 {
            f |= b[1 + i].1 << (7 - i);
        }
        if length == 9 {
            // BACKWARD Frame.
            let s = vec![
                format!("Reply: {f:02X}"),
                format!("Rply: {f:02X}"),
                format!("Rep: {f:02X}"),
                format!("R: {f:02X}"),
                "R".into(),
            ];
            self.putb(ctx, 1, 8, ANN_RAW, s);
            let s = vec![
                format!("Reply: {f}"),
                format!("Rply: {f}"),
                format!("Rep: {f}"),
                format!("R: {f}"),
                "R".into(),
            ];
            self.putb(ctx, 1, 8, ANN_REPLY, s);
            return;
        }

        // FORWARD FRAME
        // Bits[9:16]: Command/data (MSB-first)
        let mut c = 0i64;
        for i in 0..8 {
            c |= b[9 + i].1 << (7 - i);
        }
        // Raw output.
        let s = vec![
            format!("Raw data: {f:02X}"),
            format!("Raw: {f:02X}"),
            format!("Raw: {f:02X}"),
            format!("R: {f:02X}"),
            "R".into(),
        ];
        self.putb(ctx, 1, 8, ANN_RAW, s);
        let s = vec![
            format!("Raw data: {c:02X}"),
            format!("Raw: {c:02X}"),
            format!("Raw: {c:02X}"),
            format!("R: {c:02X}"),
            "R".into(),
        ];
        self.putb(ctx, 9, 16, ANN_RAW, s);

        // Bits[8:8]: Select bit.
        let s = if b[8].1 == 1 {
            vec!["Command".into(), "Comd".into(), "COM".into(), "CO".into(), "C".into()]
        } else {
            vec!["Arc Power Level".into(), "Arc Pwr".into(), "ARC".into(), "AC".into(), "A".into()]
        };
        self.putb(ctx, 8, 8, ANN_STARTBIT, s);

        if f >= 254 {
            // BROADCAST
            let s = vec!["BROADCAST".into(), "Brdcast".into(), "BC".into(), "B".into(), "B".into()];
            self.putb(ctx, 1, 7, ANN_COMMAND, s);
        } else if f >= 160 {
            // Extended command 0b10100000.
            if f == 0xC1 {
                // DALI_ENABLE_DEVICE_TYPE_X
                self.dev_type = Some(-1);
            }
            let x = lookup3(&EXTENDED_COMMANDS, f);
            let s = vec![
                format!("Extended Command: {f:02X} ({})", x.0),
                format!("XC: {f:02X} ({})", x.1),
                format!("XC: {f:02X}"),
                format!("X: {f:02X}"),
                "X".into(),
            ];
            self.putb(ctx, 1, 8, ANN_COMMAND, s);
        } else if f >= 128 {
            // Group.
            // Bits[1:1]: Ybit.
            let s = vec![
                format!("YBit: {}", b[1].1),
                format!("YB: {}", b[1].1),
                "YB".into(),
                "Y".into(),
                "Y".into(),
            ];
            self.putb(ctx, 1, 1, ANN_YBIT, s);
            let g = (f & 127) >> 1;
            let s = vec![
                format!("Group address: {g}"),
                format!("Group: {g}"),
                format!("GP: {g}"),
                format!("G: {g}"),
                "G".into(),
            ];
            self.putb(ctx, 2, 7, ANN_ADDRESS, s);
        } else {
            // Short address.
            // Bits[1:1]: Ybit.
            let s = vec![
                format!("YBit: {}", b[1].1),
                format!("YB: {}", b[1].1),
                "YB".into(),
                "Y".into(),
                "Y".into(),
            ];
            self.putb(ctx, 1, 1, ANN_YBIT, s);
            let a = f >> 1;
            let s = vec![
                format!("Short address: {a}"),
                format!("Addr: {a}"),
                format!("Addr: {a}"),
                format!("A: {a}"),
                "A".into(),
            ];
            self.putb(ctx, 2, 7, ANN_ADDRESS, s);
        }

        // Bits[9:16]: Command/data (MSB-first)
        let s = if (160..254).contains(&f) {
            if self.dev_type == Some(-1) {
                self.dev_type = Some(c);
                vec![
                    format!("Type: {c}"),
                    format!("Typ: {c}"),
                    format!("Typ: {c}"),
                    format!("T: {c}"),
                    "D".into(),
                ]
            } else {
                self.dev_type = None;
                vec![
                    format!("Data: {c}"),
                    format!("Dat: {c}"),
                    format!("Dat: {c}"),
                    format!("D: {c}"),
                    "D".into(),
                ]
            }
        } else if b[8].1 == 1 {
            let un = c & 0xF0;
            let ln = c & 0x0F;
            let x: Vec<String> = if un == 0x10 {
                // Set scene command.
                vec![format!("Recall Scene {ln}"), format!("SC {ln}")]
            } else if un == 0x40 {
                vec![format!("Store DTR as Scene {ln}"), format!("SC {ln} = DTR")]
            } else if un == 0x50 {
                vec![format!("Delete Scene {ln}"), format!("DEL SC {ln}")]
            } else if un == 0x60 {
                vec![format!("Add to Group {ln}"), format!("Grp {ln} Add")]
            } else if un == 0x70 {
                vec![format!("Remove from Group {ln}"), format!("Grp {ln} Del")]
            } else if un == 0xB0 {
                vec![format!("Query Scene {ln} Level"), format!("Sc {ln} Level")]
            } else if c >= 224 {
                // Application specific commands.
                if self.dev_type == Some(8) {
                    let x = lookup3(&DALI_DEVICE_TYPE8, c);
                    vec![x.0.to_owned(), x.1.to_owned()]
                } else {
                    vec![format!("Application Specific Command {c}"), format!("App Cmd {c}")]
                }
            } else {
                let x = lookup3(&DALI_COMMANDS, c);
                vec![x.0.to_owned(), x.1.to_owned()]
            };
            vec![
                format!("Command: {c} ({})", x[0]),
                format!("Com: {c} ({})", x[1]),
                format!("Com: {c}"),
                format!("C: {c}"),
                "C".into(),
            ]
        } else {
            vec![
                format!("Arc Power Level: {c}"),
                format!("Level: {c}"),
                format!("Lev: {c}"),
                format!("L: {c}"),
                "L".into(),
            ]
        };
        self.putb(ctx, 9, 16, ANN_COMMAND, s);
    }

    fn reset_decoder_state(&mut self) {
        self.edges.clear();
        self.bits.clear();
        self.ss_es_bits.clear();
        self.state = State::Idle;
    }
}

impl Decoder for Dali {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.active_high = config.str("polarity", "active-low") == "active-high";
        self.old_dali = if self.active_high { 0 } else { 1 };
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        // One bit: 833.33us (one half low, one half high).
        // This is how may samples are in 1TE.
        self.halfbit = ((samplerate as f64 * 0.000_833_3) / 2.0) as i64;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }
        loop {
            // TODO: Come up with more appropriate self.wait() conditions.
            let pins = ctx.wait(&[])?;
            let mut dali = i64::from(pins.get(DALI));
            if self.active_high {
                dali ^= 1; // Invert.
            }
            let samplenum = ctx.samplenum() as i64;

            // State machine.
            if self.state == State::Idle {
                // Wait for any edge (rising or falling).
                if self.old_dali == dali {
                    continue;
                }
                self.edges.push(samplenum);
                self.state = State::Phase0;
                self.old_dali = dali;
                continue;
            }

            if self.old_dali != dali {
                self.edges.push(samplenum);
            } else if samplenum == self.edges[self.edges.len() - 1] + (self.halfbit * 3 / 2) {
                self.edges.push(samplenum - self.halfbit / 2);
            } else {
                continue;
            }

            let bit = self.old_dali;
            if self.state == State::Phase0 {
                self.phase0 = bit;
                self.state = State::Phase1;
            } else if self.state == State::Phase1 {
                if bit == 1 && self.phase0 == 1 {
                    // Stop bit.
                    if self.bits.len() == 17 || self.bits.len() == 9 {
                        // Forward or Backward.
                        let len = self.bits.len();
                        self.handle_bits(ctx, len);
                    }
                    self.reset_decoder_state(); // Reset upon errors.
                    continue;
                } else {
                    self.bits.push((self.edges[self.edges.len() - 3], bit));
                    self.state = State::Phase0;
                }
            }

            self.old_dali = dali;
        }
    }
}
