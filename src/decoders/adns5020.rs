// Counterpart in sigrok:
//   libsigrokdecode/decoders/adns5020/pd.py,
//   Copyright (C) 2015 Karl Palsson <karlp@tweak.net.au>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the ADNS-5020 register map and the decoder's
//   documented classes and texts.

//! ADNS-5020: Avago bidirectional optical mouse sensor.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

const ANN_READ: usize = 0;
const ANN_WRITE: usize = 1;
const ANN_WARN: usize = 2;

/// pd.py's `regs` dict.
const REGS: &[(u64, &str)] = &[
    (0, "Product_ID"),
    (1, "Revision_ID"),
    (2, "Motion"),
    (3, "Delta_X"),
    (4, "Delta_Y"),
    (5, "SQUAL"),
    (6, "Shutter_Upper"),
    (7, "Shutter_Lower"),
    (8, "Maximum_Pixel"),
    (9, "Pixel_Sum"),
    (0xa, "Minimum_Pixel"),
    (0xb, "Pixel_Grab"),
    (0xd, "Mouse_Control"),
    (0x3a, "Chip_Reset"),
    (0x3f, "Inv_Rev_ID"),
    (0x63, "Motion_Burst"),
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "adns5020",
    name: "ADNS-5020",
    longname: "Avago ADNS-5020",
    desc: "Bidirectional optical mouse sensor protocol.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["IC", "PC", "Sensor"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "read", desc: "Register read commands" },
        AnnClass { id: "write", desc: "Register write commands" },
        AnnClass { id: "warning", desc: "Warnings" },
    ],
    annotation_rows: &[
        AnnRow { id: "read", desc: "Read", classes: &[ANN_READ] },
        AnnRow { id: "write", desc: "Write", classes: &[ANN_WRITE] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[ANN_WARN] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes ADNS-5020 optical mouse\nsensor commands and data.\n\nUse MOSI for the SDIO shared line.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Adns5020::default())
}

/// A non-negative Python integer from an SPI DATA item.
fn value(item: &Item) -> Result<u64> {
    match item {
        Item::Int(v) if *v >= 0 => Ok(*v as u64),
        Item::BigUint(bytes) => {
            let mut v = 0u64;
            for &b in bytes {
                v = v << 8 | u64::from(b);
            }
            Ok(v)
        }
        _ => Err(Error::arg(format!("adns5020: not an SPI word: {item}"))),
    }
}

/// The ADNS-5020 decoder.
#[derive(Default)]
pub struct Adns5020 {
    ss_cmd: u64,
    es_cmd: u64,
    mosi_bytes: Vec<u64>,
}

impl Adns5020 {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_cmd, self.es_cmd, class, texts);
    }
}

/// Splits an item from the SPI decoder into `(ptype, data1, data2)`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("adns5020: item is not a list"));
    };
    let [ptype, data1, data2] = parts.as_slice() else {
        return Err(Error::arg("adns5020: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("adns5020: packet type is not a string"));
    };
    Ok((ptype.as_str(), data1, data2))
}

impl Decoder for Adns5020 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let _ = config;
        *self = Adns5020::default();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let _ = ctx;
        Err(Error::not_supported("adns5020 is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, data1, data2) = split_item(item)?;

        if ptype == "CS-CHANGE" {
            // If we transition high mid-stream, toss out our data and
            // restart.
            if matches!(data1, Item::Int(0)) && matches!(data2, Item::Int(1)) {
                if self.mosi_bytes.len() != 0 && self.mosi_bytes.len() != 2 {
                    ctx.put_ann(self.ss_cmd, es, ANN_WARN, vec!["Misplaced CS#!".to_owned()]);
                    self.mosi_bytes.clear();
                }
            }
            return Ok(());
        }

        // Don't care about anything else.
        if ptype != "DATA" {
            return Ok(());
        }

        let mosi = value(data1)?;
        if self.mosi_bytes.is_empty() {
            self.ss_cmd = ss;
        }
        self.mosi_bytes.push(mosi);

        // Writes/reads are mostly two transfers (burst mode is different).
        if self.mosi_bytes.len() != 2 {
            return Ok(());
        }

        self.es_cmd = es;
        let cmd = self.mosi_bytes[0];
        let arg = self.mosi_bytes[1];
        let write = cmd & 0x80 != 0;
        let reg_addr = cmd & 0x7f;
        let mut reg_desc = REGS
            .iter()
            .find(|(addr, _)| *addr == reg_addr)
            .map_or_else(|| format!("Reserved {reg_addr:#x}"), |(_, name)| (*name).to_owned());
        if reg_addr > 0x63 {
            reg_desc = "Unknown".to_owned();
        }
        if write {
            self.putx(ctx, ANN_WRITE, vec![format!("{reg_desc}: {arg:#x}")]);
        } else {
            self.putx(ctx, ANN_READ, vec![format!("{reg_desc}: {arg}")]);
        }

        self.mosi_bytes.clear();
        Ok(())
    }
}
