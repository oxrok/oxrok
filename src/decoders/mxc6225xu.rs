// Counterpart in sigrok:
//   libsigrokdecode/decoders/mxc6225xu/pd.py,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented classes and texts.
#![allow(clippy::format_push_string, clippy::identity_op)]

//! MEMSIC MXC6225XU digital thermal orientation sensor over I2C.
//!
//! The chip's seven bit slave address is 0x2a; the decoder deliberately
//! does not filter on it (upstream has a TODO) and decodes the first
//! register write sequence it sees. Registers 0x00 and 0x01 are the two
//! acceleration outputs, 0x02 the status register and 0x03 the write-only
//! detection configuration register. Like `getattr()` upstream, any
//! other register number is an error.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "mxc6225xu",
    name: "MXC6225XU",
    longname: "MEMSIC MXC6225XU",
    desc: "Digital Thermal Orientation Sensor (DTOS) protocol.",
    license: "gplv2+",
    inputs: &["i2c"],
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
    doc: "This decoder stacks on top of the 'i2c' PD and decodes the MEMSIC MXC6225XU\n\
digital thermal orientation sensor (DTOS) protocol.\n\
\n\
The chip's I²C interface supports standard mode and fast mode (max. 400kHz).\n\
Its I²C slave address is 0x2a.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Mxc6225xu::default())
}

/// The state machine's states, as upstream's `self.state`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    GetSlaveAddr,
    GetRegAddr,
    WriteRegs,
    ReadRegs,
    ReadRegs2,
}

/// The MXC6225XU decoder.
pub struct Mxc6225xu {
    state: State,
    reg: i64,
    ss: u64,
    es: u64,
}

impl Default for Mxc6225xu {
    fn default() -> Self {
        Mxc6225xu { state: State::Idle, reg: 0, ss: 0, es: 0 }
    }
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("mxc6225xu: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("mxc6225xu: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("mxc6225xu: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

/// `status['sh']`, keyed by the raw two bit value.
fn status_sh(v: i64) -> &'static str {
    match v {
        0b00 => "none",
        0b01 => "shake left",
        0b10 => "shake right",
        _ => "undefined",
    }
}

/// `status['ori']` / `status['or']`, the same text for both fields.
fn status_ori(v: i64) -> &'static str {
    match v {
        0b00 => "vertical in upright orientation",
        0b01 => "rotated 90 degrees clockwise",
        0b10 => "vertical in inverted orientation",
        _ => "rotated 90 degrees counterclockwise",
    }
}

/// `status['shth']`.
fn status_shth(v: i64) -> &'static str {
    match v {
        0b00 => "0.5g",
        0b01 => "1.0g",
        0b10 => "1.5g",
        _ => "2.0g",
    }
}

/// `status['shc']`.
fn status_shc(v: i64) -> &'static str {
    match v {
        0b00 => "16",
        0b01 => "32",
        0b10 => "64",
        _ => "128",
    }
}

impl Mxc6225xu {
    fn putx(&self, ctx: &mut dyn Context, text: String) {
        ctx.put_ann(self.ss, self.es, 0, vec![text]);
    }

    fn handle_reg(&mut self, ctx: &mut dyn Context, b: i64) -> Result<()> {
        match self.reg {
            0x00 => {
                // XOUT: 8-bit x-axis acceleration output. Upstream prints the
                // byte unsigned even though the comment calls it two's
                // complement.
                self.putx(ctx, format!("XOUT: {b}"));
            }
            0x01 => {
                self.putx(ctx, format!("YOUT: {b}"));
            }
            0x02 => {
                let int_val = (b >> 7) & 1;
                let s = if int_val == 0 { "unchanged and no" } else { "changed or" };
                let mut ann = format!("INT = {int_val}: Orientation {s} shake event occurred\n");
                let sh = (((b >> 6) & 1) << 1) | ((b >> 5) & 1);
                ann.push_str(&format!("SH[1:0] = {}: Shake event: {}\n", bin(sh), status_sh(sh)));
                let tilt = (b >> 4) & 1;
                let s = if tilt == 0 { "" } else { "not " };
                ann.push_str(&format!("TILT = {tilt}: Orientation measurement is {s}valid\n"));
                let ori = (((b >> 3) & 1) << 1) | ((b >> 2) & 1);
                ann.push_str(&format!("ORI[1:0] = {}: {}\n", bin(ori), status_ori(ori)));
                let or_val = (((b >> 1) & 1) << 1) | ((b >> 0) & 1);
                ann.push_str(&format!("OR[1:0] = {}: {}\n", bin(or_val), status_ori(or_val)));
                self.putx(ctx, ann);
            }
            0x03 => {
                // DETECTION: write-only. Upstream loses the PD line by
                // assigning `ann` a second time instead of appending, so
                // only SHM, SHTH, SHC and ORC are in the text.
                let shm = (b >> 6) & 1;
                let mut ann = format!("SHM = {shm}: Set shake mode to {shm}\n");
                let shth = (((b >> 5) & 1) << 1) | ((b >> 4) & 1);
                ann.push_str(&format!(
                    "SHTH[1:0] = {}: Set shake threshold to {}\n",
                    bin(shth),
                    status_shth(shth)
                ));
                let shc = (((b >> 3) & 1) << 1) | ((b >> 2) & 1);
                ann.push_str(&format!(
                    "SHC[1:0] = {}: Set shake count to {} readings\n",
                    bin(shc),
                    status_shc(shc)
                ));
                let orc = (((b >> 1) & 1) << 1) | ((b >> 0) & 1);
                ann.push_str(&format!(
                    "ORC[1:0] = {}: Set orientation count to {} readings\n",
                    bin(orc),
                    status_shc(orc)
                ));
                self.putx(ctx, ann);
            }
            _ => {
                return Err(Error::device(format!(
                    "mxc6225xu: no handler for register 0x{:02x}",
                    self.reg
                )));
            }
        }
        self.reg += 1;
        Ok(())
    }
}

/// Python's `bin(n)[2:]` for the small non-negative values used here.
fn bin(v: i64) -> String {
    format!("{v:b}")
}

impl Decoder for Mxc6225xu {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Mxc6225xu::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("mxc6225xu takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;

        // Store the start/end samples of this I²C packet. Upstream does
        // this for `BITS` too; the state machine ignores it.
        self.ss = ss;
        self.es = es;

        match self.state {
            State::Idle => {
                if cmd != "START" {
                    return Ok(());
                }
                self.state = State::GetSlaveAddr;
            }
            State::GetSlaveAddr => {
                if cmd != "ADDRESS WRITE" {
                    return Ok(());
                }
                self.state = State::GetRegAddr;
            }
            State::GetRegAddr => {
                if cmd != "DATA WRITE" {
                    return Ok(());
                }
                let Item::Int(databyte) = data else {
                    return Err(Error::device("mxc6225xu: DATA WRITE has no data byte"));
                };
                self.reg = *databyte;
                self.state = State::WriteRegs;
            }
            State::WriteRegs => {
                if cmd == "START REPEAT" {
                    self.state = State::ReadRegs;
                } else if cmd == "DATA WRITE" {
                    let Item::Int(databyte) = data else {
                        return Err(Error::device("mxc6225xu: DATA WRITE has no data byte"));
                    };
                    self.handle_reg(ctx, *databyte)?;
                } else if cmd == "STOP" {
                    self.state = State::Idle;
                }
            }
            State::ReadRegs => {
                if cmd == "ADDRESS READ" {
                    self.state = State::ReadRegs2;
                }
            }
            State::ReadRegs2 => {
                if cmd == "DATA READ" {
                    let Item::Int(databyte) = data else {
                        return Err(Error::device("mxc6225xu: DATA READ has no data byte"));
                    };
                    self.handle_reg(ctx, *databyte)?;
                } else if cmd == "STOP" {
                    self.state = State::Idle;
                }
            }
        }
        Ok(())
    }
}
