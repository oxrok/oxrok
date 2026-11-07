// Counterpart in sigrok:
//   libsigrokdecode/decoders/ds28ea00/pd.py,
//   Copyright (C) 2012 Iztok Jeras <iztok.jeras@gmail.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the DS28EA00 datasheet and the decoder's documented
//   commands and texts.

//! Maxim DS28EA00: 1-Wire digital thermometer with Sequence Detect and PIO.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// `command`: function commands and their names.
static COMMAND: [(u8, &str); 9] = [
    // Scratchpad
    (0x4e, "Write scratchpad"),
    (0xbe, "Read scratchpad"),
    (0x48, "Copy scratchpad"),
    // Thermometer
    (0x44, "Convert temperature"),
    (0xb4, "Read power mode"),
    (0xb8, "Recall EEPROM"),
    (0xf5, "PIO access read"),
    (0xa5, "PIO access write"),
    (0x99, "Chain"),
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ds28ea00",
    name: "DS28EA00",
    longname: "Maxim DS28EA00 1-Wire digital thermometer",
    desc: "1-Wire digital thermometer with Sequence Detect and PIO.",
    license: "gplv2+",
    inputs: &["onewire_network"],
    outputs: &[],
    tags: &["IC", "Sensor"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[AnnClass { id: "text", desc: "Human-readable text" }],
    annotation_rows: &[],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'onewire_network' PD and decodes the\n\
Maxim DS28EA00 1-Wire digital thermometer protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Ds28ea00::default())
}

/// A Python integer as an unsigned 64 bit value.
fn item_u64(item: &Item) -> Result<u64> {
    match item {
        Item::Int(v) if *v >= 0 => Ok(*v as u64),
        Item::BigUint(bytes) if bytes.len() <= 8 => {
            Ok(bytes.iter().fold(0u64, |acc, &b| (acc << 8) | u64::from(b)))
        }
        _ => Err(Error::device("ds28ea00: expected a non-negative integer")),
    }
}

/// The DS28EA00 decoder.
#[derive(Default)]
pub struct Ds28ea00 {
    ss: u64,
    es: u64,
    state: String,
    rom: u64,
}

impl Ds28ea00 {
    fn putx(&self, ctx: &mut dyn Context, text: String) {
        ctx.put_ann(self.ss, self.es, 0, vec![text]);
    }

    fn handle_data(&mut self, ctx: &mut dyn Context, val: u8) {
        if self.state == "COMMAND" {
            let Some((_, name)) = COMMAND.iter().find(|(code, _)| *code == val) else {
                self.putx(ctx, format!("Unrecognized command: 0x{val:02x}"));
                return;
            };
            self.putx(ctx, format!("Function command: 0x{val:02x} '{name}'"));
            self.state = name.to_uppercase();
        } else if self.state == "READ SCRATCHPAD" {
            self.putx(ctx, format!("Scratchpad data: 0x{val:02x}"));
        } else if self.state == "CONVERT TEMPERATURE" {
            self.putx(ctx, format!("Temperature conversion status: 0x{val:02x}"));
        } else if COMMAND.iter().any(|(_, name)| name.to_uppercase() == self.state) {
            self.putx(ctx, format!("TODO '{}': 0x{val:02x}", self.state));
        }
    }
}

impl Decoder for Ds28ea00 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.state = "ROM".into();
        self.rom = 0;
        self.ss = 0;
        self.es = 0;
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("ds28ea00 is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let Item::List(parts) = item else {
            return Err(Error::device("ds28ea00: expected a onewire_network packet"));
        };
        if parts.len() != 2 {
            return Err(Error::device("ds28ea00: expected a two element packet"));
        }
        let Item::Str(code) = &parts[0] else {
            return Err(Error::device("ds28ea00: packet type is not a string"));
        };
        self.ss = ss;
        self.es = es;

        // State machine.
        match code.as_str() {
            "RESET/PRESENCE" => {
                let present = match &parts[1] {
                    Item::Bool(b) => *b,
                    Item::Int(v) => *v != 0,
                    _ => return Err(Error::device("ds28ea00: malformed reset/presence")),
                };
                self.putx(
                    ctx,
                    format!("Reset/presence: {}", if present { "true" } else { "false" }),
                );
                self.state = "ROM".into();
            }
            "ROM" => {
                let val = item_u64(&parts[1])?;
                self.rom = val;
                self.putx(ctx, format!("ROM: 0x{val:016x}"));
                self.state = "COMMAND".into();
            }
            "DATA" => {
                let val = match &parts[1] {
                    Item::Int(v) => *v as u8,
                    _ => return Err(Error::device("ds28ea00: data byte is not an integer")),
                };
                self.handle_data(ctx, val);
            }
            _ => {}
        }
        Ok(())
    }
}
