// Counterpart in sigrok:
//   libsigrokdecode/decoders/nunchuk/pd.py,
//   Copyright (C) 2010-2014 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented classes and texts.

//! Nintendo Wii Nunchuk controller over I2C.
//!
//! A read transaction returns six bytes: stick X/Y, accelerometer X/Y/Z
//! bits 9:2 and a final byte with the two button bits and the
//! accelerometer bits 1:0. A write transaction is the two byte
//! initialisation sequence (0x40 0x00). The summary and init annotations
//! span the whole transaction; the per-register ones span a byte or the
//! relevant bits inside the last byte.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "nunchuk",
    name: "Nunchuk",
    longname: "Nintendo Wii Nunchuk",
    desc: "Nintendo Wii Nunchuk controller protocol.",
    license: "gplv2+",
    inputs: &["i2c"],
    outputs: &[],
    tags: &["Sensor"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "reg-0x00", desc: "Register 0x00" },
        AnnClass { id: "reg-0x01", desc: "Register 0x01" },
        AnnClass { id: "reg-0x02", desc: "Register 0x02" },
        AnnClass { id: "reg-0x03", desc: "Register 0x03" },
        AnnClass { id: "reg-0x04", desc: "Register 0x04" },
        AnnClass { id: "reg-0x05", desc: "Register 0x05" },
        AnnClass { id: "bit-bz", desc: "BZ bit" },
        AnnClass { id: "bit-bc", desc: "BC bit" },
        AnnClass { id: "bit-ax", desc: "AX bits" },
        AnnClass { id: "bit-ay", desc: "AY bits" },
        AnnClass { id: "bit-az", desc: "AZ bits" },
        AnnClass { id: "nunchuk-write", desc: "Nunchuk write" },
        AnnClass { id: "cmd-init", desc: "Init command" },
        AnnClass { id: "summary", desc: "Summary" },
        AnnClass { id: "warnings", desc: "Warnings" },
    ],
    annotation_rows: &[
        AnnRow {
            id: "regs",
            desc: "Registers",
            classes: &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
        },
        AnnRow { id: "summary", desc: "Summary", classes: &[13] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[14] },
    ],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'i2c' PD and decodes the Nintendo Wii\n\
Nunchuk controller protocol.\n\
\n\
Details:\n\
http://wiibrew.org/wiki/Wiimote/Extension_Controllers/Nunchuck\n\
http://todbot.com/blog/2008/02/18/wiichuck-wii-nunchuck-adapter-available/\n\
https://www.sparkfun.com/products/9281",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Nunchuk::default())
}

/// One I2C bit: value and sample range, index 0 the LSB.
#[derive(Clone, Copy)]
struct Bit {
    #[allow(dead_code)]
    value: i64,
    ss: u64,
    es: u64,
}

/// The state machine's states, as upstream's `self.state`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    GetSlaveAddr,
    ReadRegs,
    WriteRegs,
}

/// The Nunchuk decoder.
pub struct Nunchuk {
    state: State,
    sx: i64,
    sy: i64,
    ax: i64,
    ay: i64,
    az: i64,
    bz: i64,
    bc: i64,
    reg: i64,
    ss: u64,
    es: u64,
    ss_block: u64,
    es_block: u64,
    bits: Vec<Bit>,
    init_seq: Vec<i64>,
}

impl Default for Nunchuk {
    fn default() -> Self {
        Nunchuk {
            state: State::Idle,
            sx: -1,
            sy: -1,
            ax: -1,
            ay: -1,
            az: -1,
            bz: -1,
            bc: -1,
            reg: 0x00,
            ss: 0,
            es: 0,
            ss_block: 0,
            es_block: 0,
            bits: Vec::new(),
            init_seq: Vec::new(),
        }
    }
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("nunchuk: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("nunchuk: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("nunchuk: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

/// The BITS packet payload is a list of `[bit, ss, es]`, index 0 the LSB.
fn parse_bits(data: &Item) -> Result<Vec<Bit>> {
    let Item::List(items) = data else {
        return Err(Error::device("nunchuk: BITS payload is not a list"));
    };
    let mut bits = Vec::with_capacity(items.len());
    for item in items {
        let Item::List(parts) = item else {
            return Err(Error::device("nunchuk: BITS entry is not a list"));
        };
        if parts.len() != 3 {
            return Err(Error::device("nunchuk: BITS entry is not [bit, ss, es]"));
        }
        let (Item::Int(value), Item::Int(ss), Item::Int(es)) = (&parts[0], &parts[1], &parts[2])
        else {
            return Err(Error::device("nunchuk: BITS entry is not [bit, ss, es]"));
        };
        bits.push(Bit { value: *value, ss: *ss as u64, es: *es as u64 });
    }
    Ok(bits)
}

impl Nunchuk {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, class, texts);
    }

    fn putb(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_block, self.es_block, class, texts);
    }

    /// `putd(bit1, bit2, data)`: from the start of bit `bit1` to the end
    /// of bit `bit2` in the last received byte.
    fn putd(
        &self,
        ctx: &mut dyn Context,
        bit1: usize,
        bit2: usize,
        class: usize,
        texts: Vec<String>,
    ) -> Result<()> {
        if bit1 >= self.bits.len() || bit2 >= self.bits.len() {
            return Err(Error::device("nunchuk: register 5 without its bits"));
        }
        ctx.put_ann(self.bits[bit1].ss, self.bits[bit2].es, class, texts);
        Ok(())
    }

    fn handle_reg(&mut self, ctx: &mut dyn Context, databyte: i64) -> Result<()> {
        match self.reg {
            0x00 => {
                self.ss_block = self.ss;
                self.sx = databyte;
                let s = format!("0x{:02X}", self.sx);
                self.putx(
                    ctx,
                    0,
                    vec![format!("Analog stick X position: {s}"), format!("SX: {s}")],
                );
            }
            0x01 => {
                self.sy = databyte;
                let s = format!("0x{:02X}", self.sy);
                self.putx(
                    ctx,
                    1,
                    vec![format!("Analog stick Y position: {s}"), format!("SY: {s}")],
                );
            }
            0x02 => {
                self.ax = databyte << 2;
                let s = format!("0x{:03X}", self.ax);
                self.putx(
                    ctx,
                    2,
                    vec![format!("Accelerometer X value bits[9:2]: {s}"), format!("AX[9:2]: {s}")],
                );
            }
            0x03 => {
                self.ay = databyte << 2;
                let s = format!("0x{:03X}", self.ay);
                self.putx(
                    ctx,
                    3,
                    vec![format!("Accelerometer Y value bits[9:2]: {s}"), format!("AY[9:2]: {s}")],
                );
            }
            0x04 => {
                self.az = databyte << 2;
                let s = format!("0x{:03X}", self.az);
                self.putx(
                    ctx,
                    4,
                    vec![format!("Accelerometer Z value bits[9:2]: {s}"), format!("AZ[9:2]: {s}")],
                );
            }
            0x05 => {
                self.es_block = self.es;
                self.bz = databyte & 1;
                self.bc = (databyte >> 1) & 1;
                let ax_rest = (databyte >> 2) & 0x3;
                let ay_rest = (databyte >> 4) & 0x3;
                let az_rest = (databyte >> 6) & 0x3;
                self.ax |= ax_rest;
                self.ay |= ay_rest;
                self.az |= az_rest;

                let s = if self.bz == 0 { "" } else { "not " };
                self.putd(
                    ctx,
                    0,
                    0,
                    6,
                    vec![format!("Z: {s}pressed"), format!("BZ: {}", self.bz)],
                )?;

                let s = if self.bc == 0 { "" } else { "not " };
                self.putd(
                    ctx,
                    1,
                    1,
                    7,
                    vec![format!("C: {s}pressed"), format!("BC: {}", self.bc)],
                )?;

                self.putd(
                    ctx,
                    3,
                    2,
                    8,
                    vec![
                        format!("Accelerometer X value bits[1:0]: 0x{ax_rest:X}"),
                        format!("AX[1:0]: 0x{ax_rest:X}"),
                    ],
                )?;
                self.putd(
                    ctx,
                    5,
                    4,
                    9,
                    vec![
                        format!("Accelerometer Y value bits[1:0]: 0x{ay_rest:X}"),
                        format!("AY[1:0]: 0x{ay_rest:X}"),
                    ],
                )?;
                self.putd(
                    ctx,
                    7,
                    6,
                    10,
                    vec![
                        format!("Accelerometer Z value bits[1:0]: 0x{az_rest:X}"),
                        format!("AZ[1:0]: 0x{az_rest:X}"),
                    ],
                )?;

                // Upstream resets the register counter here, and then its
                // caller increments it to 1.
                self.reg = 0x00;
            }
            _ => {
                return Err(Error::device(format!(
                    "nunchuk: no handler for register 0x{:02x}",
                    self.reg
                )));
            }
        }
        self.reg += 1;
        Ok(())
    }

    fn handle_reg_write(&mut self, ctx: &mut dyn Context, databyte: i64) {
        self.putx(ctx, 11, vec![format!("Nunchuk write: 0x{databyte:02X}")]);
        if self.init_seq.len() < 2 {
            self.init_seq.push(databyte);
        }
    }

    fn output_full_block_if_possible(&self, ctx: &mut dyn Context) {
        // For now, only output summary annotations if all values are available.
        if [self.sx, self.sy, self.ax, self.ay, self.az, self.bz, self.bc].contains(&-1) {
            return;
        }
        let bz = if self.bz == 0 { "pressed" } else { "not pressed" };
        let bc = if self.bc == 0 { "pressed" } else { "not pressed" };
        let s = format!(
            "Analog stick: {}/{}, accelerometer: {}/{}/{}, Z: {}, C: {}",
            self.sx, self.sy, self.ax, self.ay, self.az, bz, bc
        );
        self.putb(ctx, 13, vec![s]);
    }

    fn output_init_seq(&self, ctx: &mut dyn Context) {
        if self.init_seq.len() != 2 {
            self.putb(
                ctx,
                14,
                vec![format!("Init sequence was {} bytes long (2 expected)", self.init_seq.len())],
            );
            return;
        }

        if self.init_seq != [0x40, 0x00] {
            self.putb(ctx, 14, vec!["Unknown init sequence (expected: 0x40 0x00)".to_owned()]);
            return;
        }

        // TODO: Detect Nunchuk clones (they have different init sequences).
        self.putb(
            ctx,
            12,
            vec![
                "Initialize Nunchuk".to_owned(),
                "Init Nunchuk".to_owned(),
                "Init".to_owned(),
                "I".to_owned(),
            ],
        );
    }
}

impl Decoder for Nunchuk {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Nunchuk::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("nunchuk takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;

        // Collect the 'BITS' packet, then return. The next packet is
        // guaranteed to belong to these bits we just stored.
        if cmd == "BITS" {
            self.bits = parse_bits(data)?;
            return Ok(());
        }

        self.ss = ss;
        self.es = es;

        match self.state {
            State::Idle => {
                if cmd != "START" {
                    return Ok(());
                }
                self.state = State::GetSlaveAddr;
                self.ss_block = ss;
            }
            State::GetSlaveAddr => {
                if cmd == "ADDRESS READ" {
                    self.state = State::ReadRegs;
                } else if cmd == "ADDRESS WRITE" {
                    self.state = State::WriteRegs;
                }
            }
            State::ReadRegs => {
                if cmd == "DATA READ" {
                    let Item::Int(databyte) = data else {
                        return Err(Error::device("nunchuk: DATA READ has no data byte"));
                    };
                    self.handle_reg(ctx, *databyte)?;
                } else if cmd == "STOP" {
                    self.es_block = es;
                    self.output_full_block_if_possible(ctx);
                    self.sx = -1;
                    self.sy = -1;
                    self.ax = -1;
                    self.ay = -1;
                    self.az = -1;
                    self.bz = -1;
                    self.bc = -1;
                    self.state = State::Idle;
                }
            }
            State::WriteRegs => {
                if cmd == "DATA WRITE" {
                    let Item::Int(databyte) = data else {
                        return Err(Error::device("nunchuk: DATA WRITE has no data byte"));
                    };
                    self.handle_reg_write(ctx, *databyte);
                } else if cmd == "STOP" {
                    self.es_block = es;
                    self.output_init_seq(ctx);
                    self.init_seq.clear();
                    self.state = State::Idle;
                }
            }
        }
        Ok(())
    }
}
