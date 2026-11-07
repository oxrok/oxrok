// Counterpart in sigrok:
//   libsigrokdecode/decoders/pca9571/pd.py,
//   Copyright (C) 2019 Mickael Bosch <mickael.bosch@linux.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented
//   classes and texts.

//! NXP PCA9571: 8 bit I2C output expander.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    GetSlaveAddr,
    ReadData,
    WriteData,
}

/// The pca9571 decoder.
pub struct Pca9571 {
    state: State,
    /// `last_write` init 0xFF: the chip port default state is high.
    last_write: i64,
    ss: u64,
    es: u64,
}

impl Default for Pca9571 {
    fn default() -> Self {
        Pca9571 { state: State::Idle, last_write: 0xFF, ss: 0, es: 0 }
    }
}

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "pca9571",
    name: "PCA9571",
    longname: "NXP PCA9571",
    desc: "NXP PCA9571 8-bit I²C output expander.",
    license: "gplv2+",
    inputs: &["i2c"],
    outputs: &[],
    tags: &["Embedded/industrial", "IC"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "register", desc: "Register type" },
        AnnClass { id: "value", desc: "Register value" },
        AnnClass { id: "warning", desc: "Warning messages" },
    ],
    annotation_rows: &[
        AnnRow { id: "regs", desc: "Registers", classes: &[0, 1] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[2] },
    ],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module. The first line has a trailing
    // space in the upstream source and therefore in the docstring.
    doc: "This decoder stacks on top of the 'i2c' PD and decodes the NXP Semiconductors \n\
PCA9571 8-bit I²C output expander protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Pca9571::default())
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("pca9571: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("pca9571: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("pca9571: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

impl Pca9571 {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, class, texts);
    }

    fn handle_io(&mut self, ctx: &mut dyn Context, b: i64) {
        if self.state == State::ReadData {
            let operation = ["Outputs read", "R"];
            if b != self.last_write {
                self.putx(
                    ctx,
                    2,
                    vec![format!(
                        "Warning: read value and last write value ({:02X}) are different",
                        self.last_write
                    )],
                );
            }
            self.putx(
                ctx,
                1,
                vec![
                    format!("{}: {:02X}", operation[0], b),
                    format!("{}: {:02X}", operation[1], b),
                ],
            );
        } else {
            let operation = ["Outputs set", "W"];
            self.last_write = b;
            self.putx(
                ctx,
                1,
                vec![
                    format!("{}: {:02X}", operation[0], b),
                    format!("{}: {:02X}", operation[1], b),
                ],
            );
        }
    }

    fn check_correct_chip(&self, ctx: &mut dyn Context, addr: i64) -> bool {
        if addr != 0x25 {
            self.putx(
                ctx,
                2,
                vec![format!("Warning: I²C slave 0x{addr:02X} not a PCA9571 compatible chip.")],
            );
            return false;
        }
        true
    }

    fn handle(&mut self, ctx: &mut dyn Context, cmd: &str, databyte: Option<i64>) {
        // START/STOP and ACK/NACK packets carry no data byte; Python's
        // `databyte` is None for those and only data/address branches
        // read it.
        let b = databyte.unwrap_or(0);
        if cmd == "ACK" || cmd == "BITS" {
            // Discard 'ACK' and 'BITS'.
        } else if cmd == "START" || cmd == "START REPEAT" {
            // Start a communication.
            self.state = State::GetSlaveAddr;
        } else if cmd == "NACK" || cmd == "STOP" {
            // Reset the state machine.
            self.state = State::Idle;
        } else if cmd == "ADDRESS READ" || cmd == "ADDRESS WRITE" {
            if self.state == State::GetSlaveAddr && self.check_correct_chip(ctx, b) {
                self.state = if cmd == "ADDRESS READ" { State::ReadData } else { State::WriteData };
            } else {
                self.state = State::Idle;
            }
        } else if cmd == "DATA READ" || cmd == "DATA WRITE" {
            if self.state == State::ReadData || self.state == State::WriteData {
                self.handle_io(ctx, b);
            } else {
                self.state = State::Idle;
            }
        }
    }
}

impl Decoder for Pca9571 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("pca9571 takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;

        // Store the start/end samples of this I2C packet.
        self.ss = ss;
        self.es = es;

        let databyte = match data {
            Item::Int(v) => Some(*v),
            _ => None,
        };
        self.handle(ctx, cmd, databyte);
        Ok(())
    }
}
