// Counterpart in sigrok:
//   libsigrokdecode/decoders/max7219/pd.py,
//   Copyright (C) 2015 Paul Evans <leonerd@leonerd.org.uk>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the MAX7219 register set and the decoder's
//   documented classes and texts.

//! MAX7219/MAX7221 8-digit LED display driver writes on an SPI bus.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_REG: usize = 0;
const ANN_DIGIT: usize = 1;
const ANN_WARNING: usize = 2;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "max7219",
    name: "MAX7219",
    longname: "Maxim MAX7219/MAX7221",
    desc: "Maxim MAX72xx series 8-digit LED display driver.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["Display"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "register", desc: "Registers written to the device" },
        AnnClass { id: "digit", desc: "Digits displayed on the device" },
        AnnClass { id: "warnings", desc: "Human-readable warnings" },
    ],
    annotation_rows: &[
        AnnRow { id: "commands", desc: "Commands", classes: &[0, 1] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[2] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes the Maxim MAX7219 and\n\
          MAX7221 LED matrix driver protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Max7219::default())
}

/// The value text of a register write, as pd.py's `registers` lambdas
/// format it.
fn register_value(reg: i64, value: i64) -> String {
    match reg {
        // 0x00: ['No-op', lambda _: ''],
        0x00 => String::new(),
        // 0x09: ['Decode', lambda v: '0b{:08b}'.format(v)],
        0x09 => format!("0b{value:08b}"),
        // 0x0A: ['Intensity', _decode_intensity],
        0x0A => match value & 0x0f {
            0 => "min".to_owned(),
            15 => "max".to_owned(),
            intensity => intensity.to_string(),
        },
        // 0x0B: ['Scan limit', lambda v: 1 + v],
        0x0B => (1 + value).to_string(),
        // 0x0C: ['Shutdown', lambda v: 'off' if v else 'on'],
        0x0C => if value != 0 { "off" } else { "on" }.to_owned(),
        // 0x0F: ['Display test', lambda v: 'on' if v else 'off'],
        0x0F => if value != 0 { "on" } else { "off" }.to_owned(),
        _ => unreachable!(),
    }
}

/// The register name, or `None` when pd.py's `registers` has no entry.
fn register_name(reg: i64) -> Option<&'static str> {
    Some(match reg {
        0x00 => "No-op",
        0x09 => "Decode",
        0x0A => "Intensity",
        0x0B => "Scan limit",
        0x0C => "Shutdown",
        0x0F => "Display test",
        _ => return None,
    })
}

/// The MAX7219 decoder.
#[derive(Default)]
pub struct Max7219 {
    cs_asserted: bool,
    pos: u64,
    addr: i64,
    addr_start: u64,
    cs_start: u64,
}

impl Max7219 {
    fn putreg(ctx: &mut dyn Context, ss: u64, es: u64, reg: &str, value: &str) {
        ctx.put_ann(ss, es, ANN_REG, vec![format!("{reg}: {value}")]);
    }

    fn putdigit(ctx: &mut dyn Context, ss: u64, es: u64, digit: i64, value: i64) {
        ctx.put_ann(ss, es, ANN_DIGIT, vec![format!("Digit {digit}: {value:02X}")]);
    }

    fn putwarn(ctx: &mut dyn Context, ss: u64, es: u64, message: &str) {
        ctx.put_ann(ss, es, ANN_WARNING, vec![message.to_owned()]);
    }
}

/// `ptype, mosi, _ = data`; the third element is unused here.
fn split_item(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("max7219: item is not a list"));
    };
    let [ptype, mosi, _] = parts.as_slice() else {
        return Err(Error::arg("max7219: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("max7219: packet type is not a string"));
    };
    Ok((ptype.as_str(), mosi))
}

fn int(item: &Item) -> Result<i64> {
    match item {
        Item::Int(v) => Ok(*v),
        _ => Err(Error::arg(format!("max7219: not a word: {item}"))),
    }
}

impl Decoder for Max7219 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        // pd.py start(): self.pos = 0; self.cs_start = 0. reset() does
        // nothing, so the initial cs_asserted is unset until the first
        // CS-CHANGE.
        *self = Max7219::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("max7219 is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, mosi) = split_item(item)?;

        if ptype == "DATA" {
            if !self.cs_asserted {
                return Ok(());
            }

            let mosi = int(mosi)?;
            if self.pos == 0 {
                self.addr = mosi;
                self.addr_start = ss;
            } else if self.pos == 1 {
                if (1..=8).contains(&self.addr) {
                    Self::putdigit(ctx, self.addr_start, es, self.addr, mosi);
                } else if let Some(name) = register_name(self.addr) {
                    let value = register_value(self.addr, mosi);
                    Self::putreg(ctx, self.addr_start, es, name, &value);
                } else {
                    Self::putwarn(
                        ctx,
                        self.addr_start,
                        es,
                        &format!("Unknown register {:02X}", self.addr),
                    );
                }
            }

            self.pos += 1;
        } else if ptype == "CS-CHANGE" {
            // `self.cs_asserted = mosi`, where mosi is the old CS value
            // for this packet: data[1].
            let Item::List(parts) = item else { unreachable!() };
            let old = &parts[1];
            self.cs_asserted = !matches!(old, Item::None) && int(old)? != 0;
            if self.cs_asserted {
                self.pos = 0;
                self.cs_start = ss;
            } else if self.pos == 1 {
                // Don't warn if pos=0 so that CS# glitches don't appear
                // as spurious warnings.
                Self::putwarn(ctx, self.cs_start, es, "Short write");
            } else if self.pos > 2 {
                Self::putwarn(ctx, self.cs_start, es, "Overlong write");
            }
        }

        Ok(())
    }
}
