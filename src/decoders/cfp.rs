// Counterpart in sigrok:
//   libsigrokdecode/decoders/cfp/pd.py,
//   Copyright (C) 2018 Elias Oenal <sigrok@eliasoenal.com>.
// This file is a port of the sigrok counterpart, BSD licensed like it,
//   written from the CFP MSA register map and the decoder's documented
//   classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - Only reads are decoded; a write is skipped silently.
//   - The register address is the Clause 45 address the mdio decoder
//     sends, not the register field of the frame, and the ranges are
//     inclusive of both ends (0x8000..=0x807F and so on).
//   - Every annotation uses the ss/es of the whole mdio frame, not the
//     register field, because `decode()` stores its arguments in
//     `self.ss`/`self.es` and `putx()` uses those.
//   - The module identifier is only decoded for 0x8000, and unknown
//     values are printed as `Reserved`.
#![allow(clippy::default_constructed_unit_structs, clippy::too_many_lines, clippy::unused_self)]

//! CFP: 100 Gigabit C form-factor pluggable transceiver registers.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

const ANN_REGISTER: usize = 0;
const ANN_DECODE: usize = 1;

/// `MODULE_ID` in pd.py.
fn module_id(reg: i64) -> &'static str {
    match reg {
        0x00 => "Unknown or unspecified",
        0x01 => "GBIC",
        0x02 => "Module/connector soldered to motherboard",
        0x03 => "SFP",
        0x04 => "300 pin XSBI",
        0x05 => "XENPAK",
        0x06 => "XFP",
        0x07 => "XFF",
        0x08 => "XFP-E",
        0x09 => "XPAK",
        0x0a => "X2",
        0x0B => "DWDM-SFP",
        0x0C => "QSFP",
        0x0D => "QSFP+",
        0x0E => "CFP",
        0x0F => "CXP (TBD)",
        0x11 => "CFP2",
        0x12 => "CFP4",
        // The Python dict has no 0x10 entry: `MODULE_ID.get(reg, ...)`.
        _ => "Reserved",
    }
}

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "cfp",
    name: "CFP",
    longname: "100 Gigabit C form-factor pluggable",
    desc: "100 Gigabit C form-factor pluggable (CFP) protocol.",
    license: "BSD",
    inputs: &["mdio"],
    outputs: &[],
    tags: &["Networking"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "register", desc: "Register" },
        AnnClass { id: "decode", desc: "Decode" },
    ],
    annotation_rows: &[
        AnnRow { id: "registers", desc: "Registers", classes: &[0] },
        AnnRow { id: "decodes", desc: "Decodes", classes: &[1] },
    ],
    binary: &[],
    // The package's __init__.py docstring, which the Python host reads.
    doc: "This decoder stacks on top of the 'mdio' PD and decodes the CFP 100G\n\
          pluggable transceiver protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Cfp::default())
}

/// The CFP decoder.
#[derive(Default)]
pub struct Cfp;

/// An item as a Python bool, also accepting an int the way Python
/// `not is_read` treats 0/1.
fn as_bool(item: &Item) -> Result<bool> {
    match item {
        Item::Bool(b) => Ok(*b),
        Item::Int(v) => Ok(*v != 0),
        _ => Err(Error::device("cfp: expected a boolean")),
    }
}

fn as_int(item: &Item) -> Result<i64> {
    match item {
        Item::Int(v) => Ok(*v),
        _ => Err(Error::device("cfp: expected an integer")),
    }
}

impl Cfp {
    fn putx(&self, ctx: &mut dyn Context, ss: u64, es: u64, class: usize, texts: Vec<String>) {
        ctx.put_ann(ss, es, class, texts);
    }

    fn handle_entry(&self, ctx: &mut dyn Context, ss: u64, es: u64, entry: &Item) -> Result<()> {
        let Item::List(parts) = entry else {
            return Err(Error::device("cfp: mdio entry is not a list"));
        };
        if parts.len() != 6 {
            return Err(Error::device("cfp: mdio entry is not a 6 element tuple"));
        }
        let _clause45 = as_bool(&parts[0])?;
        let clause45_addr = as_int(&parts[1])?;
        let is_read = as_bool(&parts[2])?;
        let _portad = as_int(&parts[3])?;
        let _devad = as_int(&parts[4])?;
        let reg = as_int(&parts[5])?;

        if !is_read {
            return Ok(());
        }

        if (0x8000..=0x807F).contains(&clause45_addr) {
            self.putx(
                ctx,
                ss,
                es,
                ANN_REGISTER,
                vec!["CFP NVR 1: Basic ID register".to_owned(), "NVR1".to_owned()],
            );
            if clause45_addr == 0x8000 {
                self.putx(
                    ctx,
                    ss,
                    es,
                    ANN_DECODE,
                    vec![format!("Module identifier: {}", module_id(reg))],
                );
            }
        } else if (0x8080..=0x80FF).contains(&clause45_addr) {
            self.putx(
                ctx,
                ss,
                es,
                ANN_REGISTER,
                vec!["CFP NVR 2: Extended ID register".to_owned(), "NVR2".to_owned()],
            );
        } else if (0x8100..=0x817F).contains(&clause45_addr) {
            self.putx(
                ctx,
                ss,
                es,
                ANN_REGISTER,
                vec!["CFP NVR 3: Network lane specific register".to_owned(), "NVR3".to_owned()],
            );
        } else if (0x8180..=0x81FF).contains(&clause45_addr) {
            self.putx(ctx, ss, es, ANN_REGISTER, vec!["CFP NVR 4".to_owned(), "NVR4".to_owned()]);
        } else if (0x8400..=0x847F).contains(&clause45_addr) {
            self.putx(
                ctx,
                ss,
                es,
                ANN_REGISTER,
                vec!["Vendor NVR 1: Vendor data register".to_owned(), "V-NVR1".to_owned()],
            );
        } else if (0x8480..=0x84FF).contains(&clause45_addr) {
            self.putx(
                ctx,
                ss,
                es,
                ANN_REGISTER,
                vec!["Vendor NVR 2: Vendor data register".to_owned(), "V-NVR2".to_owned()],
            );
        } else if (0x8800..=0x887F).contains(&clause45_addr) {
            self.putx(
                ctx,
                ss,
                es,
                ANN_REGISTER,
                vec!["User NVR 1: User data register".to_owned(), "U-NVR1".to_owned()],
            );
        } else if (0x8880..=0x88FF).contains(&clause45_addr) {
            self.putx(
                ctx,
                ss,
                es,
                ANN_REGISTER,
                vec!["User NVR 2: User data register".to_owned(), "U-NVR2".to_owned()],
            );
        } else if (0xA000..=0xA07F).contains(&clause45_addr) {
            self.putx(
                ctx,
                ss,
                es,
                ANN_REGISTER,
                vec![
                    "CFP Module VR 1: CFP Module level control and DDM register".to_owned(),
                    "Mod-VR1".to_owned(),
                ],
            );
        } else if (0xA080..=0xA0FF).contains(&clause45_addr) {
            self.putx(
                ctx,
                ss,
                es,
                ANN_REGISTER,
                vec![
                    "MLG VR 1: MLG Management Interface register".to_owned(),
                    "MLG-VR1".to_owned(),
                ],
            );
        }
        Ok(())
    }
}

impl Decoder for Cfp {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("cfp takes an mdio decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        // `decode()` unpacks a list of `(clause45, addr, is_read, ...)`
        // tuples; a single frame carries one of them in the mdio output.
        let Item::List(entries) = item else {
            return Err(Error::device("cfp: expected a list of mdio entries"));
        };
        for entry in entries {
            self.handle_entry(ctx, ss, es, entry)?;
        }
        Ok(())
    }
}
