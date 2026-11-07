// Counterpart in sigrok:
//   libsigrokdecode/decoders/ds2408/pd.py,
//   Copyright (C) 2019 Mariusz Bialonczyk <manio@skyboo.net>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the DS2408 datasheet and the decoder's documented
//   commands and texts.

//! Maxim DS2408: 1-Wire 8-channel addressable switch.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// `command`: function commands and their names.
static COMMAND: [(u8, &str); 6] = [
    (0xf0, "Read PIO Registers"),
    (0xf5, "Channel Access Read"),
    (0x5a, "Channel Access Write"),
    (0xcc, "Write Conditional Search Register"),
    (0xc3, "Reset Activity Latches"),
    (0x3c, "Disable Test Mode"),
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ds2408",
    name: "DS2408",
    longname: "Maxim DS2408",
    desc: "1-Wire 8-channel addressable switch.",
    license: "gplv2+",
    inputs: &["onewire_network"],
    outputs: &[],
    tags: &["Embedded/industrial", "IC"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[AnnClass { id: "text", desc: "Human-readable text" }],
    annotation_rows: &[],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'onewire_network' PD and decodes the\n\
Maxim DS2408 1-Wire 8-channel addressable switch protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Ds2408::default())
}

/// A Python integer as an unsigned 64 bit value.
fn item_u64(item: &Item) -> Result<u64> {
    match item {
        Item::Int(v) if *v >= 0 => Ok(*v as u64),
        Item::BigUint(bytes) if bytes.len() <= 8 => {
            Ok(bytes.iter().fold(0u64, |acc, &b| (acc << 8) | u64::from(b)))
        }
        _ => Err(Error::device("ds2408: expected a non-negative integer")),
    }
}

/// The DS2408 decoder.
#[derive(Default)]
pub struct Ds2408 {
    ss: u64,
    es: u64,
    /// Bytes for function command.
    bytes: Vec<u8>,
}

impl Ds2408 {
    fn putx(&self, ctx: &mut dyn Context, text: String) {
        ctx.put_ann(self.ss, self.es, 0, vec![text]);
    }

    fn handle_data(&mut self, ctx: &mut dyn Context, val: u8, ss: u64, es: u64) {
        self.bytes.push(val);
        if self.bytes.len() == 1 {
            self.ss = ss;
            self.es = es;
            if let Some((_, name)) = COMMAND.iter().find(|(code, _)| *code == val) {
                self.putx(ctx, format!("{name} (0x{val:02x})"));
            } else {
                self.putx(ctx, format!("Unrecognized command: 0x{val:02x}"));
            }
        } else if self.bytes[0] == 0xf0 {
            // Read PIO Registers
            if self.bytes.len() == 2 {
                self.ss = ss;
            } else if self.bytes.len() == 3 {
                self.es = es;
                let address = (u32::from(self.bytes[2]) << 8) + u32::from(self.bytes[1]);
                self.putx(ctx, format!("Target address: 0x{address:04x}"));
            } else if self.bytes.len() > 3 {
                self.ss = ss;
                self.es = es;
                self.putx(ctx, format!("Data: 0x{:02x}", self.bytes[self.bytes.len() - 1]));
            }
        } else if self.bytes[0] == 0xf5 {
            // Channel Access Read
            if self.bytes.len() == 2 {
                self.ss = ss;
            } else if self.bytes.len() > 2 {
                self.ss = ss;
                self.es = es;
                self.putx(ctx, format!("PIO sample: 0x{:02x}", self.bytes[self.bytes.len() - 1]));
            }
        } else if self.bytes[0] == 0x5a {
            // Channel Access Write
            if self.bytes.len() == 2 {
                self.ss = ss;
            } else if self.bytes.len() == 3 {
                self.es = es;
                let last = self.bytes[self.bytes.len() - 1];
                let previous = self.bytes[self.bytes.len() - 2];
                if last == (previous ^ 0xff) {
                    self.putx(
                        ctx,
                        format!("Data: 0x{previous:02x} (bit-inversion correct: 0x{last:02x})"),
                    );
                } else {
                    self.putx(
                        ctx,
                        format!(
                            "Data error: second byte (0x{last:02x}) is not bit-inverse of first (0x{previous:02x})"
                        ),
                    );
                }
            } else if self.bytes.len() > 3 {
                self.ss = ss;
                self.es = es;
                let last = self.bytes[self.bytes.len() - 1];
                if last == 0xaa {
                    self.putx(ctx, "Success".into());
                } else if last == 0xff {
                    self.putx(ctx, "Fail New State".into());
                }
            }
        } else if self.bytes[0] == 0xcc {
            // Write Conditional Search Register
            if self.bytes.len() == 2 {
                self.ss = ss;
            } else if self.bytes.len() == 3 {
                self.es = es;
                let address = (u32::from(self.bytes[2]) << 8) + u32::from(self.bytes[1]);
                self.putx(ctx, format!("Target address: 0x{address:04x}"));
            } else if self.bytes.len() > 3 {
                self.ss = ss;
                self.es = es;
                self.putx(ctx, format!("Data: 0x{:02x}", self.bytes[self.bytes.len() - 1]));
            }
        } else if self.bytes[0] == 0xc3 {
            // Reset Activity Latches
            if self.bytes.len() == 2 {
                self.ss = ss;
            } else if self.bytes.len() > 2 {
                self.ss = ss;
                self.es = es;
                let last = self.bytes[self.bytes.len() - 1];
                if last == 0xaa {
                    self.putx(ctx, "Success".into());
                } else {
                    self.putx(ctx, "Invalid byte".into());
                }
            }
        }
    }
}

impl Decoder for Ds2408 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.bytes.clear();
        self.ss = 0;
        self.es = 0;
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("ds2408 is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let Item::List(parts) = item else {
            return Err(Error::device("ds2408: expected a onewire_network packet"));
        };
        if parts.len() != 2 {
            return Err(Error::device("ds2408: expected a two element packet"));
        }
        let Item::Str(code) = &parts[0] else {
            return Err(Error::device("ds2408: packet type is not a string"));
        };
        match code.as_str() {
            "RESET/PRESENCE" => {
                let present = match &parts[1] {
                    Item::Bool(b) => *b,
                    Item::Int(v) => *v != 0,
                    _ => return Err(Error::device("ds2408: malformed reset/presence")),
                };
                self.ss = ss;
                self.es = es;
                self.putx(
                    ctx,
                    format!("Reset/presence: {}", if present { "true" } else { "false" }),
                );
                self.bytes.clear();
            }
            "ROM" => {
                let val = item_u64(&parts[1])?;
                self.ss = ss;
                self.es = es;
                let family_code = val & 0xff;
                self.putx(ctx, format!("ROM: 0x{val:016x} (family code 0x{family_code:02x})"));
                self.bytes.clear();
            }
            "DATA" => {
                let val = match &parts[1] {
                    Item::Int(v) => *v as u8,
                    _ => return Err(Error::device("ds2408: data byte is not an integer")),
                };
                self.handle_data(ctx, val, ss, es);
            }
            _ => {}
        }
        Ok(())
    }
}
