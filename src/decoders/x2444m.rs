// Counterpart in sigrok:
//   libsigrokdecode/decoders/x2444m/pd.py,
//   Copyright (C) 2018 Stefan Petersen <spe@ciellt.se>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the X2444M/P command set and the decoder's
//   documented classes and texts.

//! Xicor X2444M/P nonvolatile static RAM protocol on an SPI bus.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "x2444m",
    name: "X2444M/P",
    longname: "Xicor X2444M/P",
    desc: "Xicor X2444M/P nonvolatile static RAM protocol.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["IC", "Memory"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "wrds", desc: "Write disable" },
        AnnClass { id: "sto", desc: "Store RAM data in EEPROM" },
        AnnClass { id: "sleep", desc: "Enter sleep mode" },
        AnnClass { id: "write", desc: "Write data into RAM" },
        AnnClass { id: "wren", desc: "Write enable" },
        AnnClass { id: "rcl", desc: "Recall EEPROM data into RAM" },
        AnnClass { id: "read", desc: "Data read from RAM" },
        // pd.py lists the READ class twice, with indices 6 and 7.
        AnnClass { id: "read", desc: "Data read from RAM" },
    ],
    annotation_rows: &[],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes the Xicor X2444M/P\n\
          nonvolatile static RAM protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(X2444m::default())
}

/// pd.py's `registers`: name, annotation class, and whether the address
/// field is formatted as `0x%x` (READ/WRITE) or is the empty string.
struct Register {
    name: &'static str,
    class: usize,
    has_address: bool,
}

fn register(addr: i64) -> Option<Register> {
    Some(match addr & 0x87 {
        0x80 => Register { name: "WRDS", class: 0, has_address: false },
        0x81 => Register { name: "STO", class: 1, has_address: false },
        0x82 => Register { name: "SLEEP", class: 2, has_address: false },
        0x83 => Register { name: "WRITE", class: 3, has_address: true },
        0x84 => Register { name: "WREN", class: 4, has_address: false },
        0x85 => Register { name: "RCL", class: 5, has_address: false },
        0x86 => Register { name: "READ", class: 6, has_address: true },
        0x87 => Register { name: "READ", class: 7, has_address: true },
        _ => return None,
    })
}

/// The X2444M/P decoder.
#[derive(Default)]
pub struct X2444m {
    cs_start: u64,
    cs_asserted: bool,
    cmd_digit: u64,
    addr: i64,
    addr_start: u64,
    read_value: i64,
    write_value: i64,
}

impl X2444m {
    #[allow(clippy::too_many_arguments)]
    fn putreadwrite(
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        reg: &str,
        idx: usize,
        addr: &str,
        value: i64,
    ) {
        let initial = reg.chars().next().unwrap_or(' ');
        ctx.put_ann(
            ss,
            es,
            idx,
            vec![
                format!("{reg}: {addr} => 0x{value:04x}"),
                format!("{initial}: {addr} => 0x{value:04x}"),
                initial.to_string(),
            ],
        );
    }

    fn putcmd(ctx: &mut dyn Context, ss: u64, es: u64, reg: &str, idx: usize) {
        let initial = reg.chars().next().unwrap_or(' ');
        ctx.put_ann(ss, es, idx, vec![reg.to_owned(), initial.to_string()]);
    }
}

/// `ptype, mosi, miso = data`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("x2444m: item is not a list"));
    };
    let [ptype, mosi, miso] = parts.as_slice() else {
        return Err(Error::arg("x2444m: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("x2444m: packet type is not a string"));
    };
    Ok((ptype.as_str(), mosi, miso))
}

fn int(item: &Item) -> Result<i64> {
    match item {
        Item::Int(v) => Ok(*v),
        _ => Err(Error::arg(format!("x2444m: not a word: {item}"))),
    }
}

impl Decoder for X2444m {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = X2444m::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("x2444m is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, mosi, miso) = split_item(item)?;

        if ptype == "DATA" {
            if !self.cs_asserted {
                return Ok(());
            }

            if self.cmd_digit == 0 {
                self.addr = int(mosi)?;
                self.addr_start = ss;
            } else {
                self.read_value = (self.read_value << 8) + int(miso)?;
                self.write_value = (self.write_value << 8) + int(mosi)?;
            }
            self.cmd_digit += 1;
        } else if ptype == "CS-CHANGE" {
            // `self.cs_asserted = (miso == 1)`: data[2], the new value.
            self.cs_asserted = !matches!(miso, Item::None) && int(miso)? == 1;
            // When not asserted, CS has just changed from asserted to
            // deasserted.
            if !self.cs_asserted {
                let entry =
                    register(self.addr).ok_or_else(|| Error::arg("x2444m: unknown command"))?;
                if self.cmd_digit == 1 {
                    // Only one digit, simple command. Else read/write.
                    Self::putcmd(ctx, self.addr_start, es, entry.name, entry.class);
                } else if self.cmd_digit > 1 {
                    let value = if entry.name == "READ" {
                        self.read_value
                    } else if entry.name == "WRITE" {
                        self.write_value
                    } else {
                        0
                    };
                    let address = if entry.has_address {
                        format!("0x{:x}", (self.addr >> 3) & 0x0f)
                    } else {
                        String::new()
                    };
                    Self::putreadwrite(
                        ctx,
                        self.addr_start,
                        es,
                        entry.name,
                        entry.class,
                        &address,
                        value,
                    );
                }
            }

            if self.cs_asserted {
                self.cs_start = ss;
                self.cmd_digit = 0;
                self.read_value = 0;
                self.write_value = 0;
            }
        }
        Ok(())
    }
}
