// Counterpart in sigrok:
//   libsigrokdecode/decoders/nrf24l01/pd.py,
//   Copyright (C) 2014 Jens Steinhauser <jens.steinhauser@gmail.com>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the nRF24L01(+) command set and the decoder's
//   documented classes and texts.

//! Nordic Semiconductor nRF24L01(+) 2.4 GHz transceiver commands and
//! payloads on an SPI bus.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_CMD: usize = 0;
const ANN_TX: usize = 1;
const ANN_REG: usize = 2;
const ANN_RX: usize = 3;
const ANN_WARN: usize = 4;

/// pd.py's `regs`: address → (name, size).
fn reg(addr: i64, xn297: bool) -> Option<(&'static str, usize)> {
    Some(match addr {
        0x00 => ("CONFIG", 1),
        0x01 => ("EN_AA", 1),
        0x02 => ("EN_RXADDR", 1),
        0x03 => ("SETUP_AW", 1),
        0x04 => ("SETUP_RETR", 1),
        0x05 => ("RF_CH", 1),
        0x06 => ("RF_SETUP", 1),
        0x07 => ("STATUS", 1),
        0x08 => ("OBSERVE_TX", 1),
        0x09 => ("RPD", 1),
        0x0a => ("RX_ADDR_P0", 5),
        0x0b => ("RX_ADDR_P1", 5),
        0x0c => ("RX_ADDR_P2", 1),
        0x0d => ("RX_ADDR_P3", 1),
        0x0e => ("RX_ADDR_P4", 1),
        0x0f => ("RX_ADDR_P5", 1),
        0x10 => ("TX_ADDR", 5),
        0x11 => ("RX_PW_P0", 1),
        0x12 => ("RX_PW_P1", 1),
        0x13 => ("RX_PW_P2", 1),
        0x14 => ("RX_PW_P3", 1),
        0x15 => ("RX_PW_P4", 1),
        0x16 => ("RX_PW_P5", 1),
        0x17 => ("FIFO_STATUS", 1),
        0x1c => ("DYNPD", 1),
        0x1d => ("FEATURE", 1),
        // pd.py's start() updates the global `regs` with xn297_regs, so
        // those addresses are known whenever the chip option is xn297.
        0x19 if xn297 => ("DEMOD_CAL", 5),
        0x1e if xn297 => ("RF_CAL", 7),
        0x1f if xn297 => ("BB_CAL", 5),
        _ => return None,
    })
}

/// pd.py's register name lookup for `R_REGISTER`.
fn reg_name(addr: i64, xn297: bool) -> Option<&'static str> {
    reg(addr, xn297).map(|(name, _)| name)
}

/// A parsed command: `(name, data, min, max)`.
struct Command {
    name: &'static str,
    dat: Option<i64>,
    min: usize,
    max: usize,
}

/// pd.py's `parse_command`; `None` means an unknown command byte.
fn parse_command(b: i64, xn297: bool) -> Option<Command> {
    let masked = b & 0xe0;
    if masked == 0x00 || masked == 0x20 {
        let name = if masked == 0 { "R_REGISTER" } else { "W_REGISTER" };
        let d = b & 0x1f;
        let m = reg(d, xn297).map_or(1, |(_, size)| size);
        return Some(Command { name, dat: Some(d), min: 1, max: m });
    }
    let (name, dat, min, max) = match b {
        // nRF24L01 only.
        0x50 => ("ACTIVATE", None, 1, 1),
        0x61 => ("R_RX_PAYLOAD", None, 1, 32),
        0x60 => ("R_RX_PL_WID", None, 1, 1),
        0xa0 => ("W_TX_PAYLOAD", None, 1, 32),
        0xb0 => ("W_TX_PAYLOAD_NOACK", None, 1, 32),
        0xe1 => ("FLUSH_TX", None, 0, 0),
        0xe2 => ("FLUSH_RX", None, 0, 0),
        0xe3 => ("REUSE_TX_PL", None, 0, 0),
        0xff => ("NOP", None, 0, 0),
        _ if (b & 0xf8) == 0xa8 => ("W_ACK_PAYLOAD", Some(b & 0x07), 1, 32),
        _ => return None,
    };
    Some(Command { name, dat, min, max })
}

/// Python's `str.isprintable()` for the bytes a register or payload can
/// hold: only 0x20-0x7e, 0xa1-0xac and 0xae-0xff print themselves.
fn printable(byte: i64) -> bool {
    matches!(byte, 0x20..=0x7e | 0xa1..=0xac | 0xae..=0xff)
}

fn escape(byte: i64, always_hex: bool) -> String {
    if always_hex {
        return format!("{byte:02X}");
    }
    if printable(byte) {
        char::from_u32(u32::try_from(byte).unwrap_or(0)).unwrap_or('\u{fffd}').to_string()
    } else {
        format!("\\x{byte:02X}")
    }
}

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "nrf24l01",
    name: "nRF24L01(+)",
    longname: "Nordic Semiconductor nRF24L01(+)",
    desc: "2.4GHz RF transceiver chip.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["IC", "Wireless/RF"],
    channels: &[],
    optional_channels: &[],
    options: &[OptionDesc {
        id: "chip",
        desc: "Chip type",
        default: OptionValue::s("nrf24l01"),
        values: &[OptionValue::s("nrf24l01"), OptionValue::s("xn297")],
    }],
    annotations: &[
        // Sent from the host to the chip.
        AnnClass { id: "cmd", desc: "Commands sent to the device" },
        AnnClass { id: "tx-data", desc: "Payload sent to the device" },
        // Returned by the chip.
        AnnClass { id: "register", desc: "Registers read from the device" },
        AnnClass { id: "rx-data", desc: "Payload read from the device" },
        AnnClass { id: "warning", desc: "Warnings" },
    ],
    annotation_rows: &[
        AnnRow { id: "commands", desc: "Commands", classes: &[0, 1] },
        AnnRow { id: "responses", desc: "Responses", classes: &[2, 3] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[4] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes the protocol spoken\n\
          by the Nordic Semiconductor nRF24L01 and nRF24L01+ 2.4GHz transceiver chips.\n\
          \n\
          Details:\n\
          http://www.nordicsemi.com/eng/Products/2.4GHz-RF/nRF24L01\n\
          http://www.nordicsemi.com/eng/Products/2.4GHz-RF/nRF24L01P",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Nrf24l01::default())
}

/// The nRF24L01(+) decoder.
#[allow(clippy::struct_excessive_bools)] // as upstream's four flags
pub struct Nrf24l01 {
    xn297: bool,
    requirements_met: bool,
    cs_was_released: bool,
    first: bool,
    cmd: Option<&'static str>,
    dat: i64,
    min: usize,
    max: usize,
    mb: Vec<(i64, i64)>,
    mb_s: i64,
    mb_e: i64,
}

impl Default for Nrf24l01 {
    fn default() -> Self {
        let mut this = Nrf24l01 {
            xn297: false,
            requirements_met: true,
            cs_was_released: false,
            first: true,
            cmd: None,
            dat: 0,
            min: 0,
            max: 0,
            mb: Vec::new(),
            mb_s: -1,
            mb_e: -1,
        };
        this.next();
        this
    }
}

impl Nrf24l01 {
    /// pd.py's `next()`: resets the decoder after a complete command was
    /// decoded.
    fn next(&mut self) {
        // 'True' for the first byte after CS went low.
        self.first = true;

        // The current command, and the minimum and maximum number of
        // data bytes to follow.
        self.cmd = None;
        self.min = 0;
        self.max = 0;

        // Used to collect the bytes after the command byte (and the
        // start/end sample number).
        self.mb.clear();
        self.mb_s = -1;
        self.mb_e = -1;
    }

    fn warn(ctx: &mut dyn Context, ss: u64, es: u64, msg: &str) {
        ctx.put_ann(ss, es, ANN_WARN, vec![msg.to_owned()]);
    }

    fn putp(ctx: &mut dyn Context, ss: u64, es: u64, ann: usize, msg: String) {
        ctx.put_ann(ss, es, ann, vec![msg]);
    }

    fn mosi_bytes(&self) -> Vec<i64> {
        self.mb.iter().map(|b| b.0).collect()
    }

    fn miso_bytes(&self) -> Vec<i64> {
        self.mb.iter().map(|b| b.1).collect()
    }

    fn format_command(&self) -> String {
        match self.cmd {
            Some("R_REGISTER") => {
                let name = reg_name(self.dat, self.xn297).unwrap_or("unknown register");
                format!("Cmd R_REGISTER \"{name}\"")
            }
            Some(cmd) => format!("Cmd {cmd}"),
            None => String::new(),
        }
    }

    /// pd.py's `decode_command`: decodes the command byte and prepares
    /// the decoding of the following data bytes.
    fn decode_command(&mut self, ctx: &mut dyn Context, ss: u64, es: u64, b: i64) {
        let Some(c) = parse_command(b, self.xn297) else {
            Self::warn(ctx, ss, es, "unknown command");
            return;
        };

        self.cmd = Some(c.name);
        self.dat = c.dat.unwrap_or(0);
        self.min = c.min;
        self.max = c.max;

        if c.name == "W_REGISTER" || c.name == "ACTIVATE" {
            // Don't output anything now, the command is merged with the
            // data bytes following it.
            self.mb_s = ss as i64;
        } else {
            let text = self.format_command();
            Self::putp(ctx, ss, es, ANN_CMD, text);
        }
    }

    /// pd.py's `decode_mb_data`: the data bytes are joined in the order
    /// given; only the register path reverses them beforehand.
    #[allow(clippy::too_many_arguments)]
    fn decode_mb_data(
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        ann: usize,
        data: &[i64],
        label: &str,
        always_hex: bool,
    ) {
        let data: String = data.iter().map(|&b| escape(b, always_hex)).collect();
        let text = format!("{label} = \"{data}\"");
        Self::putp(ctx, ss, es, ann, text);
    }

    /// pd.py's `decode_register`. `regid` is either a register address or
    /// a register name.
    fn decode_register(
        &self,
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        ann: usize,
        regid: RegId,
        data: &[i64],
    ) {
        let name = match regid {
            RegId::Addr(addr) => {
                let Some(name) = reg_name(addr, self.xn297) else {
                    Self::warn(ctx, ss, es, "unknown register");
                    return;
                };
                name
            }
            RegId::Name(name) => name,
        };

        // Multi byte register come LSByte first.
        let reversed: Vec<i64> = data.iter().rev().copied().collect();
        let label = if self.cmd == Some("W_REGISTER") && ann == ANN_CMD {
            // The 'W_REGISTER' command is merged with the following byte(s).
            format!("{}: {name}", self.format_command())
        } else {
            format!("Reg {name}")
        };

        Self::decode_mb_data(ctx, ss, es, ann, &reversed, &label, true);
    }

    /// pd.py's `finish_command`: decodes the remaining data bytes.
    fn finish_command(&mut self, ctx: &mut dyn Context, ss: u64, es: u64) {
        match self.cmd {
            Some("R_REGISTER") => {
                let data = self.miso_bytes();
                self.decode_register(ctx, ss, es, ANN_REG, RegId::Addr(self.dat), &data);
            }
            Some("W_REGISTER") => {
                let data = self.mosi_bytes();
                self.decode_register(ctx, ss, es, ANN_CMD, RegId::Addr(self.dat), &data);
            }
            Some("R_RX_PAYLOAD") => {
                let data = self.miso_bytes();
                Self::decode_mb_data(ctx, ss, es, ANN_RX, &data, "RX payload", false);
            }
            Some("W_TX_PAYLOAD" | "W_TX_PAYLOAD_NOACK") => {
                let data = self.mosi_bytes();
                Self::decode_mb_data(ctx, ss, es, ANN_TX, &data, "TX payload", false);
            }
            Some("W_ACK_PAYLOAD") => {
                let lbl = format!("ACK payload for pipe {}", self.dat);
                let data = self.mosi_bytes();
                Self::decode_mb_data(ctx, ss, es, ANN_TX, &data, &lbl, false);
            }
            Some("R_RX_PL_WID") => {
                let msg = format!("Payload width = {}", self.mb[0].1);
                Self::putp(ctx, ss, es, ANN_REG, msg);
            }
            Some("ACTIVATE") => {
                let text = self.format_command();
                Self::putp(ctx, ss, es, ANN_CMD, text);
                let data = self.mosi_bytes();
                if data[0] != 0x73 {
                    Self::warn(ctx, ss, es, "wrong data for \"ACTIVATE\" command");
                }
            }
            _ => {}
        }
    }
}

/// A register identifier: an address or a direct name (`STATUS`).
#[derive(Clone, Copy)]
enum RegId {
    Addr(i64),
    Name(&'static str),
}

/// `ptype, data1, data2 = data`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("nrf24l01: item is not a list"));
    };
    let [ptype, data1, data2] = parts.as_slice() else {
        return Err(Error::arg("nrf24l01: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("nrf24l01: packet type is not a string"));
    };
    Ok((ptype.as_str(), data1, data2))
}

fn int(item: &Item) -> Result<i64> {
    match item {
        Item::Int(v) => Ok(*v),
        _ => Err(Error::arg(format!("nrf24l01: not a word: {item}"))),
    }
}

impl Decoder for Nrf24l01 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self =
            Nrf24l01 { xn297: config.str("chip", "nrf24l01") == "xn297", ..Nrf24l01::default() };
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("nrf24l01 is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        if !self.requirements_met {
            return Ok(());
        }

        let (ptype, data1, data2) = split_item(item)?;

        if ptype == "CS-CHANGE" {
            if matches!(data1, Item::None) {
                if matches!(data2, Item::None) {
                    self.requirements_met = false;
                    return Err(Error::arg("CS# pin required."));
                } else if int(data2)? == 1 {
                    self.cs_was_released = true;
                }
            }

            let old_is_zero = !matches!(data1, Item::None) && int(data1)? == 0;
            let new_is_one = !matches!(data2, Item::None) && int(data2)? == 1;
            if old_is_zero && new_is_one {
                // Rising edge, the complete command is transmitted,
                // process the bytes that were send after the command byte.
                if self.cmd.is_some() {
                    // Check if we got the minimum number of data bytes
                    // after the command byte.
                    if self.mb.len() < self.min {
                        Self::warn(ctx, ss, ss, "missing data bytes");
                    } else if !self.mb.is_empty() {
                        let (s, e) = (self.mb_s as u64, self.mb_e as u64);
                        self.finish_command(ctx, s, e);
                    }
                }

                self.next();
                self.cs_was_released = true;
            }
        } else if ptype == "DATA" && self.cs_was_released {
            if matches!(data1, Item::None) || matches!(data2, Item::None) {
                self.requirements_met = false;
                return Err(Error::arg("Both MISO and MOSI pins required."));
            }
            let mosi = int(data1)?;
            let miso = int(data2)?;

            if self.first {
                self.first = false;
                // First MOSI byte is always the command.
                self.decode_command(ctx, ss, es, mosi);
                // First MISO byte is always the status register.
                self.decode_register(ctx, ss, es, ANN_REG, RegId::Name("STATUS"), &[miso]);
            } else if self.cmd.is_none() || self.mb.len() >= self.max {
                Self::warn(ctx, ss, es, "excess byte");
            } else {
                // Collect the bytes after the command byte.
                if self.mb_s == -1 {
                    self.mb_s = ss as i64;
                }
                self.mb_e = es as i64;
                self.mb.push((mosi, miso));
            }
        }
        Ok(())
    }
}
