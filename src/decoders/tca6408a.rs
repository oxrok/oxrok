// Counterpart in sigrok:
//   libsigrokdecode/decoders/tca6408a/pd.py,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2013 Matt Ranostay <mranostay@gmail.com>,
//   Copyright (C) 2014 alberink <alberink@stampfini.org>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented
//   classes and texts.

//! Texas Instruments TCA6408A: 8 bit I2C I/O expander.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    GetSlaveAddr,
    GetRegAddr,
    WriteIoRegs,
    ReadIoRegs,
    ReadIoRegs2,
}

/// The tca6408a decoder.
pub struct Tca6408a {
    state: State,
    /// `self.chip`, -1 until an address was seen. Only assigned upstream,
    /// never read.
    chip: Option<i64>,
    ss: u64,
    es: u64,
    reg: i64,
}

impl Default for Tca6408a {
    fn default() -> Self {
        Tca6408a { state: State::Idle, chip: None, ss: 0, es: 0, reg: 0 }
    }
}

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "tca6408a",
    name: "TI TCA6408A",
    longname: "Texas Instruments TCA6408A",
    desc: "Texas Instruments TCA6408A 8-bit I²C I/O expander.",
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
        AnnClass { id: "warnings", desc: "Warning messages" },
    ],
    annotation_rows: &[
        AnnRow { id: "regs", desc: "Registers", classes: &[0, 1] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[2] },
    ],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'i2c' PD and decodes the Texas Instruments\n\
TCA6408A 8-bit I²C I/O expander protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Tca6408a::default())
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("tca6408a: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("tca6408a: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("tca6408a: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

impl Tca6408a {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, class, texts);
    }

    fn handle_reg_0x00(&self, ctx: &mut dyn Context, b: i64) {
        self.putx(ctx, 1, vec![format!("State of inputs: {b:02X}")]);
    }

    fn handle_reg_0x01(&self, ctx: &mut dyn Context, b: i64) {
        self.putx(ctx, 1, vec![format!("Outputs set: {b:02X}")]);
    }

    fn handle_reg_0x02(&self, ctx: &mut dyn Context, b: i64) {
        self.putx(ctx, 1, vec![format!("Polarity inverted: {b:02X}")]);
    }

    fn handle_reg_0x03(&self, ctx: &mut dyn Context, b: i64) {
        self.putx(ctx, 1, vec![format!("Configuration: {b:02X}")]);
    }

    fn handle_write_reg(&self, ctx: &mut dyn Context, b: i64) {
        match b {
            0 => self.putx(ctx, 0, vec!["Input port".to_owned(), "In".to_owned(), "I".to_owned()]),
            1 => {
                self.putx(ctx, 0, vec!["Output port".to_owned(), "Out".to_owned(), "O".to_owned()]);
            }
            2 => self.putx(
                ctx,
                0,
                vec!["Polarity inversion register".to_owned(), "Pol".to_owned(), "P".to_owned()],
            ),
            3 => self.putx(
                ctx,
                0,
                vec!["Configuration register".to_owned(), "Conf".to_owned(), "C".to_owned()],
            ),
            _ => {}
        }
    }

    /// Upstream's `check_correct_chip` does not return a value; it sets
    /// the state to IDLE on a foreign address, and the caller then
    /// returns before overwriting it.
    fn check_correct_chip(&mut self, ctx: &mut dyn Context, addr: i64) {
        if addr != 0x20 && addr != 0x21 {
            self.putx(
                ctx,
                2,
                vec![format!("Warning: I²C slave 0x{addr:02X} not a TCA6408A compatible chip.")],
            );
            self.state = State::Idle;
        }
    }

    fn handle_reg(&self, ctx: &mut dyn Context, reg: i64, b: i64) {
        match reg {
            0x00 => self.handle_reg_0x00(ctx, b),
            0x01 => self.handle_reg_0x01(ctx, b),
            0x02 => self.handle_reg_0x02(ctx, b),
            0x03 => self.handle_reg_0x03(ctx, b),
            _ => {}
        }
    }

    fn handle(&mut self, ctx: &mut dyn Context, cmd: &str, databyte: Option<i64>) {
        // START/STOP and ACK/NACK packets carry no data byte; Python's
        // `databyte` is None for those and only data/address branches
        // read it.
        let b = databyte.unwrap_or(0);
        match self.state {
            State::Idle => {
                // Wait for an I2C START condition.
                if cmd != "START" {
                    return;
                }
                self.state = State::GetSlaveAddr;
            }
            State::GetSlaveAddr => {
                self.chip = databyte;
                self.state = State::GetRegAddr;
            }
            State::GetRegAddr => {
                // Wait for a data write (master selects the slave register).
                if cmd == "ADDRESS READ" || cmd == "ADDRESS WRITE" {
                    self.check_correct_chip(ctx, b);
                }
                if cmd != "DATA WRITE" {
                    return;
                }
                self.reg = b;
                self.handle_write_reg(ctx, self.reg);
                self.state = State::WriteIoRegs;
            }
            State::WriteIoRegs => {
                // If we see a Repeated Start here, the master wants to read.
                if cmd == "START REPEAT" {
                    self.state = State::ReadIoRegs;
                    return;
                }
                // Otherwise: Get data bytes until a STOP condition occurs.
                if cmd == "DATA WRITE" {
                    self.handle_reg(ctx, self.reg, b);
                } else if cmd == "STOP" {
                    self.state = State::Idle;
                    self.chip = None;
                }
            }
            State::ReadIoRegs => {
                // Wait for an address read operation.
                if cmd == "ADDRESS READ" {
                    self.state = State::ReadIoRegs2;
                    self.chip = databyte;
                }
            }
            State::ReadIoRegs2 => {
                if cmd == "DATA READ" {
                    self.handle_reg(ctx, self.reg, b);
                } else if cmd == "STOP" {
                    self.state = State::Idle;
                }
            }
        }
    }
}

impl Decoder for Tca6408a {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("tca6408a takes an i2c decoder as input"))
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
