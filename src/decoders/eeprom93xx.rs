// Counterpart in sigrok:
//   libsigrokdecode/decoders/eeprom93xx/pd.py,
//   Copyright (C) 2017 Kevin Redon <kingkevin@cuvoodoo.info>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the M93Cx6 datasheet and the decoder's documented
//   instructions, classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - The start bit the microwire decoder sends is part of `data`; the
//     opcode is taken from `data[0].si` and `data[1].si` only.
//   - A read consumes the address, then as many whole words as the
//     packet holds; a trailing partial word becomes a "Not enough word
//     bits" warning. A write reports a missing word at the start of the
//     bit that is there, ending on that bit's *start* sample.
//   - A blank field (a bit carrying 'E' errors) contributes an empty
//     string in the hex dump, which becomes a space.
//   - The address binary is a single byte, so an address wider than 8
//     bits fails the way `bytes([a])` does in Python.
#![allow(
    clippy::assigning_clones,
    clippy::doc_markdown,
    clippy::too_many_lines,
    clippy::unused_self
)]

//! 93xx Microwire EEPROM: READ, WRITE, ERASE, WEN, WDS, ERAL and WRAL.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item,
    OptionDesc, OptionValue,
};

const ANN_SI_DATA: usize = 0;
const ANN_SO_DATA: usize = 1;
const ANN_WARNING: usize = 2;
const BIN_ADDRESS: usize = 0;
const BIN_DATA: usize = 1;

static OPTIONS: [OptionDesc; 3] = [
    OptionDesc { id: "addresssize", desc: "Address size", default: OptionValue::i(8), values: &[] },
    OptionDesc { id: "wordsize", desc: "Word size", default: OptionValue::i(16), values: &[] },
    OptionDesc {
        id: "format",
        desc: "Data format",
        default: OptionValue::s("hex"),
        values: &[OptionValue::s("ascii"), OptionValue::s("hex")],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "eeprom93xx",
    name: "93xx EEPROM",
    longname: "93xx Microwire EEPROM",
    desc: "93xx series Microwire EEPROM protocol.",
    license: "gplv2+",
    inputs: &["microwire"],
    outputs: &[],
    tags: &["IC", "Memory"],
    channels: &[],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "si-data", desc: "SI data" },
        AnnClass { id: "so-data", desc: "SO data" },
        AnnClass { id: "warning", desc: "Warning" },
    ],
    annotation_rows: &[
        AnnRow { id: "data", desc: "Data", classes: &[0, 1] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[2] },
    ],
    binary: &[BinClass { id: "address", desc: "Address" }, BinClass { id: "data", desc: "Data" }],
    // The package's __init__.py docstring, which the Python host reads.
    doc: "This decoder stacks on top of the 'microwire' PD and decodes the 93xx EEPROM\n\
          specific instructions.\n\
          \n\
          The implemented instructions come from the STMicroelectronics M93Cx6 EEPROM\n\
          datasheet. They are compatible with the Atmel AT93Cxx EEPROM with slightly\n\
          different names.\n\
          \n\
          Warning: Other EEPROMs using Microwire might have different operation codes\n\
          and instructions.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Eeprom93xx::default())
}

/// One microwire bit: `[ss, es, si, so]`.
#[derive(Clone, Copy)]
struct Bit {
    ss: u64,
    es: u64,
    si: i64,
    so: i64,
}

impl Bit {
    fn parse(item: &Item) -> Result<Bit> {
        let Item::List(parts) = item else {
            return Err(Error::device("eeprom93xx: microwire bit is not a list"));
        };
        if parts.len() != 4 {
            return Err(Error::device("eeprom93xx: microwire bit is not [ss, es, si, so]"));
        }
        let (Item::Int(ss), Item::Int(es), Item::Int(si), Item::Int(so)) =
            (&parts[0], &parts[1], &parts[2], &parts[3])
        else {
            return Err(Error::device("eeprom93xx: microwire bit fields are not integers"));
        };
        Ok(Bit { ss: *ss as u64, es: *es as u64, si: *si, so: *so })
    }
}

/// The eeprom93xx decoder.
pub struct Eeprom93xx {
    addresssize: usize,
    wordsize: usize,
    format: String,
}

impl Default for Eeprom93xx {
    fn default() -> Self {
        Eeprom93xx { addresssize: 8, wordsize: 16, format: "hex".to_owned() }
    }
}

impl Eeprom93xx {
    /// pd.py `put_address()`: MSb first, one byte of binary output.
    fn put_address(&self, ctx: &mut dyn Context, data: &[Bit]) -> Result<()> {
        let mut a: i64 = 0;
        let len = data.len();
        for (b, bit) in data.iter().enumerate() {
            a += bit.si << (len - b - 1);
        }
        ctx.put_ann(
            data[0].ss,
            data[len - 1].es,
            ANN_SI_DATA,
            vec![format!("Address: 0x{a:04x}"), format!("Addr: 0x{a:04x}"), format!("0x{a:04x}")],
        );
        // `bytes([a])` requires 0 <= a <= 255.
        if !(0..=255).contains(&a) {
            return Err(Error::device("eeprom93xx: address does not fit in a byte"));
        }
        ctx.put_binary(data[0].ss, data[len - 1].es, BIN_ADDRESS, vec![a as u8]);
        Ok(())
    }

    /// pd.py `put_word()`: MSb first, `si` for an SI word and `so` for
    /// an SO word.
    fn put_word(&self, ctx: &mut dyn Context, si: bool, data: &[Bit]) -> Result<()> {
        let mut word: i64 = 0;
        let len = data.len();
        for (b, bit) in data.iter().enumerate() {
            let d = if si { bit.si } else { bit.so };
            word += d << (len - b - 1);
        }
        let idx = if si { ANN_SI_DATA } else { ANN_SO_DATA };

        if self.format == "ascii" {
            let mut word_str = String::new();
            let mut s = 0;
            while s < len {
                let c = 0xff & (word >> s);
                if (32..=126).contains(&c) {
                    word_str = format!("{}{word_str}", char::from(c as u8));
                } else {
                    word_str = format!("[{c:02X}]{word_str}");
                }
                s += 8;
            }
            ctx.put_ann(
                data[0].ss,
                data[len - 1].es,
                idx,
                vec![format!("Data: {word_str}"), word_str],
            );
        } else {
            ctx.put_ann(
                data[0].ss,
                data[len - 1].es,
                idx,
                vec![format!("Data: 0x{word:04x}"), format!("0x{word:04x}")],
            );
            // `bytes([...])` requires both bytes to be 0..=255; the
            // Python original masks the high byte with 0xff00.
            let hi = (word & 0xff00) >> 8;
            let lo = word & 0xff;
            if !(0..=255).contains(&hi) || !(0..=255).contains(&lo) {
                return Err(Error::device("eeprom93xx: word does not fit in two bytes"));
            }
            ctx.put_binary(data[0].ss, data[len - 1].es, BIN_DATA, vec![hi as u8, lo as u8]);
        }
        Ok(())
    }

    /// pd.py `decode()`.
    fn handle(&self, ctx: &mut dyn Context, ss: u64, es: u64, data: &[Bit]) -> Result<()> {
        if data.len() < 2 + self.addresssize {
            ctx.put_ann(ss, es, ANN_WARNING, vec!["Not enough packet bits".to_owned()]);
            return Ok(());
        }

        let opcode = (data[0].si << 1) + data[1].si;
        let addr_start = 2;
        let addr_end = 2 + self.addresssize;

        match opcode {
            // READ instruction.
            2 => {
                ctx.put_ann(
                    data[0].ss,
                    data[1].es,
                    ANN_SI_DATA,
                    vec!["Read word".to_owned(), "READ".to_owned()],
                );
                self.put_address(ctx, &data[addr_start..addr_end])?;

                let mut word_start = addr_end;
                while data.len() > word_start {
                    if data.len() - word_start < self.wordsize {
                        ctx.put_ann(
                            data[word_start].ss,
                            data[data.len() - 1].es,
                            ANN_WARNING,
                            vec!["Not enough word bits".to_owned()],
                        );
                        break;
                    }
                    self.put_word(ctx, false, &data[word_start..word_start + self.wordsize])?;
                    word_start += self.wordsize;
                }
            }
            // WRITE instruction.
            1 => {
                ctx.put_ann(
                    data[0].ss,
                    data[1].es,
                    ANN_SI_DATA,
                    vec!["Write word".to_owned(), "WRITE".to_owned()],
                );
                self.put_address(ctx, &data[addr_start..addr_end])?;
                if data.len() < addr_end + self.wordsize {
                    ctx.put_ann(
                        data[addr_end].ss,
                        data[data.len() - 1].ss,
                        ANN_WARNING,
                        vec!["Not enough word bits".to_owned()],
                    );
                } else {
                    self.put_word(ctx, true, &data[addr_end..addr_end + self.wordsize])?;
                }
            }
            // ERASE instruction.
            3 => {
                ctx.put_ann(
                    data[0].ss,
                    data[1].es,
                    ANN_SI_DATA,
                    vec!["Erase word".to_owned(), "ERASE".to_owned()],
                );
                self.put_address(ctx, &data[addr_start..addr_end])?;
            }
            // WEN, WDS, ERAL and WRAL instructions.
            0 => {
                let end = 2 + self.addresssize - 1;
                if data[2].si == 1 && data[3].si == 1 {
                    ctx.put_ann(
                        data[0].ss,
                        data[end].es,
                        ANN_SI_DATA,
                        vec!["Write enable".to_owned(), "WEN".to_owned()],
                    );
                } else if data[2].si == 0 && data[3].si == 0 {
                    ctx.put_ann(
                        data[0].ss,
                        data[end].es,
                        ANN_SI_DATA,
                        vec!["Write disable".to_owned(), "WDS".to_owned()],
                    );
                } else if data[2].si == 1 && data[3].si == 0 {
                    ctx.put_ann(
                        data[0].ss,
                        data[end].es,
                        ANN_SI_DATA,
                        vec![
                            "Erase all memory".to_owned(),
                            "Erase all".to_owned(),
                            "ERAL".to_owned(),
                        ],
                    );
                } else if data[2].si == 0 && data[3].si == 1 {
                    ctx.put_ann(
                        data[0].ss,
                        data[end].es,
                        ANN_SI_DATA,
                        vec![
                            "Write all memory".to_owned(),
                            "Write all".to_owned(),
                            "WRAL".to_owned(),
                        ],
                    );
                    if data.len() < addr_end + self.wordsize {
                        ctx.put_ann(
                            data[addr_end].ss,
                            data[data.len() - 1].ss,
                            ANN_WARNING,
                            vec!["Not enough word bits".to_owned()],
                        );
                    } else {
                        self.put_word(ctx, true, &data[addr_end..addr_end + self.wordsize])?;
                    }
                }
            }
            _ => {}
        }
        Ok(())
    }
}

impl Decoder for Eeprom93xx {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.addresssize = config.int("addresssize", 8).max(0) as usize;
        self.wordsize = config.int("wordsize", 16).max(0) as usize;
        self.format = config.str("format", "hex").to_owned();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("eeprom93xx takes a microwire decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let Item::List(items) = item else {
            return Err(Error::device("eeprom93xx: expected a list of microwire bits"));
        };
        let mut data = Vec::with_capacity(items.len());
        for entry in items {
            data.push(Bit::parse(entry)?);
        }
        self.handle(ctx, ss, es, &data)
    }
}
