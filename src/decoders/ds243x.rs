// Counterpart in sigrok:
//   libsigrokdecode/decoders/ds243x/pd.py,
//   Copyright (C) 2017 Kevin Redon <kingkevin@cuvoodoo.info>,
//   Copyright (C) 2017 Soeren Apel <soeren@apelpie.net>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the DS243x datasheets and the decoder's documented
//   commands and texts.
#![allow(
    clippy::assigning_clones,
    clippy::doc_markdown,
    clippy::match_same_arms,
    clippy::too_many_lines,
    clippy::type_complexity
)]

//! Maxim DS2432/3: 1-Wire EEPROM commands, scratchpad, secrets and MACs.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, BinClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// `commands_2432`.
static COMMANDS_2432: [(u8, &str); 7] = [
    (0x0f, "Write scratchpad"),
    (0xaa, "Read scratchpad"),
    (0x55, "Copy scratchpad"),
    (0xf0, "Read memory"),
    (0x5a, "Load first secret"),
    (0x33, "Compute next secret"),
    (0xa5, "Read authenticated page"),
];

/// `commands_2433`.
static COMMANDS_2433: [(u8, &str); 4] = [
    (0x0f, "Write scratchpad"),
    (0xaa, "Read scratchpad"),
    (0x55, "Copy scratchpad"),
    (0xf0, "Read memory"),
];

/// `family_codes`: family code, device name and its command set.
static FAMILY_CODES: [(u8, &str, &[(u8, &str)]); 2] =
    [(0x33, "DS2432", &COMMANDS_2432), (0x23, "DS2433", &COMMANDS_2433)];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ds243x",
    name: "DS243x",
    longname: "Maxim DS2432/3",
    desc: "Maxim DS243x series 1-Wire EEPROM protocol.",
    license: "gplv2+",
    inputs: &["onewire_network"],
    outputs: &[],
    tags: &["IC", "Memory"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[AnnClass { id: "text", desc: "Human-readable text" }],
    annotation_rows: &[],
    binary: &[BinClass { id: "mem_read", desc: "Data read from memory" }],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'onewire_network' PD and decodes the\n\
Maxim DS243x (1-Wire EEPROM) protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Ds243x::default())
}

/// `crc16`: Maxim's CRC-16, initial value 0x0000, reverse polynom
/// 0xa001, xor-out 0xffff.
fn crc16(bytes: &[u8]) -> u16 {
    let mut crc: u16 = 0x0000;
    for &byte in bytes {
        let mut byte = u16::from(byte);
        for _ in 0..8 {
            if (byte ^ crc) & 1 != 0 {
                crc = (crc >> 1) ^ 0xa001;
            } else {
                crc >>= 1;
            }
            byte >>= 1;
        }
    }
    crc ^ 0xffff
}

/// `','.join(format(n, '#04x') for n in bytes)`.
fn hex_join(bytes: &[u8]) -> String {
    bytes.iter().map(|n| format!("{n:#04x}")).collect::<Vec<_>>().join(",")
}

/// The DS243x decoder.
#[derive(Default)]
pub struct Ds243x {
    ss: u64,
    es: u64,
    /// Bytes for function command.
    bytes: Vec<u8>,
    family_code: Option<u8>,
    family: String,
    commands: &'static [(u8, &'static str)],
}

impl Ds243x {
    fn putx(&self, ctx: &mut dyn Context, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, 0, texts);
    }

    fn handle_rom(&mut self, ctx: &mut dyn Context, val: u64, ss: u64, es: u64) {
        self.ss = ss;
        self.es = es;
        let family_code = (val & 0xff) as u8;
        self.family_code = Some(family_code);

        let s = if let Some((_, name, commands)) =
            FAMILY_CODES.iter().find(|(code, _, _)| *code == family_code)
        {
            self.family = (*name).to_owned();
            self.commands = commands;
            format!("is 0x{family_code:02x}, {name} detected")
        } else {
            format!("0x{family_code:02x} unknown")
        };

        self.putx(
            ctx,
            vec![
                format!("ROM: 0x{val:016x} (family code {s})"),
                format!("ROM: 0x{val:016x} ({})", self.family),
            ],
        );
        self.bytes.clear();
    }

    fn handle_data(&mut self, ctx: &mut dyn Context, val: u8, ss: u64, es: u64) {
        self.bytes.push(val);
        let n = self.bytes.len();
        if n == 1 {
            self.ss = ss;
            self.es = es;
            if let Some((_, name)) = self.commands.iter().find(|(code, _)| *code == val) {
                self.putx(ctx, vec![format!("Function command: {name} (0x{val:02x})")]);
            } else {
                self.putx(ctx, vec![format!("Unrecognized command: 0x{val:02x}")]);
            }
            return;
        }

        match self.bytes[0] {
            // Write scratchpad
            0x0f => match n {
                2 => self.ss = ss,
                3 => {
                    self.es = es;
                    let address = (u32::from(self.bytes[2]) << 8) + u32::from(self.bytes[1]);
                    self.putx(ctx, vec![format!("Target address: 0x{address:04x}")]);
                }
                4 => self.ss = ss,
                11 => {
                    self.es = es;
                    self.putx(ctx, vec![format!("Data: {}", hex_join(&self.bytes[3..11]))]);
                }
                12 => self.ss = ss,
                13 => {
                    self.es = es;
                    let expected = u16::from(self.bytes[11]) + (u16::from(self.bytes[12]) << 8);
                    let ok = crc16(&self.bytes[0..11]) == expected;
                    self.putx(ctx, vec![format!("CRC: {}", if ok { "ok" } else { "error" })]);
                }
                _ => {}
            },
            // Read scratchpad
            0xaa => match n {
                2 => self.ss = ss,
                3 => {
                    self.es = es;
                    let address = (u32::from(self.bytes[2]) << 8) + u32::from(self.bytes[1]);
                    self.putx(ctx, vec![format!("Target address: 0x{address:04x}")]);
                }
                4 => {
                    self.ss = ss;
                    self.es = es;
                    self.putx(ctx, vec![format!("Data status (E/S): 0x{:02x}", self.bytes[3])]);
                }
                5 => self.ss = ss,
                12 => {
                    self.es = es;
                    self.putx(ctx, vec![format!("Data: {}", hex_join(&self.bytes[4..12]))]);
                }
                13 => self.ss = ss,
                14 => {
                    self.es = es;
                    let expected = u16::from(self.bytes[12]) + (u16::from(self.bytes[13]) << 8);
                    let ok = crc16(&self.bytes[0..12]) == expected;
                    self.putx(ctx, vec![format!("CRC: {}", if ok { "ok" } else { "error" })]);
                }
                _ => {}
            },
            // Load first secret
            0x5a => {
                if n == 2 {
                    self.ss = ss;
                } else if n == 4 {
                    self.es = es;
                    self.putx(
                        ctx,
                        vec![format!(
                            "Authorization pattern (TA1, TA2, E/S): {}",
                            hex_join(&self.bytes[1..4])
                        )],
                    );
                } else if n > 4 {
                    self.ss = ss;
                    self.es = es;
                    let last = self.bytes[n - 1];
                    if last == 0xaa || last == 0x55 {
                        self.putx(ctx, vec!["End of operation".into()]);
                    }
                }
            }
            // Compute next secret
            0x33 => {
                if n == 2 {
                    self.ss = ss;
                } else if n == 3 {
                    self.es = es;
                    let address = (u32::from(self.bytes[2]) << 8) + u32::from(self.bytes[1]);
                    self.putx(ctx, vec![format!("Target address: 0x{address:04x}")]);
                } else if n > 3 {
                    self.ss = ss;
                    self.es = es;
                    let last = self.bytes[n - 1];
                    if last == 0xaa || last == 0x55 {
                        self.putx(ctx, vec!["End of operation".into()]);
                    }
                }
            }
            // Copy scratchpad
            0x55 => {
                if n == 2 {
                    self.ss = ss;
                } else if n == 4 {
                    self.es = es;
                    self.putx(
                        ctx,
                        vec![format!(
                            "Authorization pattern (TA1, TA2, E/S): {}",
                            hex_join(&self.bytes[1..4])
                        )],
                    );
                } else if n == 5 {
                    self.ss = ss;
                } else if n == 24 {
                    self.es = es;
                    let mac = hex_join(&self.bytes[4..24]);
                    self.putx(
                        ctx,
                        vec![format!("Message authentication code: {mac}"), format!("MAC: {mac}")],
                    );
                } else if n > 24 {
                    self.ss = ss;
                    self.es = es;
                    let last = self.bytes[n - 1];
                    if last == 0xaa || last == 0x55 {
                        self.putx(ctx, vec!["Operation succeeded".into()]);
                    } else if last == 0 {
                        self.putx(ctx, vec!["Operation failed".into()]);
                    }
                }
            }
            // Read authenticated page
            0xa5 => match n {
                2 => self.ss = ss,
                3 => {
                    self.es = es;
                    let address = (u32::from(self.bytes[2]) << 8) + u32::from(self.bytes[1]);
                    self.putx(ctx, vec![format!("Target address: 0x{address:04x}")]);
                }
                4 => self.ss = ss,
                35 => {
                    self.es = es;
                    self.putx(ctx, vec![format!("Data: {}", hex_join(&self.bytes[3..35]))]);
                }
                36 => {
                    self.ss = ss;
                    self.es = es;
                    let ok = self.bytes[35] == 0xff;
                    self.putx(ctx, vec![format!("Padding: {}", if ok { "ok" } else { "error" })]);
                }
                37 => self.ss = ss,
                38 => {
                    self.es = es;
                    let expected = u16::from(self.bytes[36]) + (u16::from(self.bytes[37]) << 8);
                    let ok = crc16(&self.bytes[0..36]) == expected;
                    self.putx(ctx, vec![format!("CRC: {}", if ok { "ok" } else { "error" })]);
                }
                39 => self.ss = ss,
                58 => {
                    self.es = es;
                    let mac = hex_join(&self.bytes[38..58]);
                    self.putx(
                        ctx,
                        vec![format!("Message authentication code: {mac}"), format!("MAC: {mac}")],
                    );
                }
                59 => self.ss = ss,
                60 => {
                    self.es = es;
                    let expected = u16::from(self.bytes[58]) + (u16::from(self.bytes[59]) << 8);
                    let ok = crc16(&self.bytes[38..58]) == expected;
                    self.putx(ctx, vec![format!("MAC CRC: {}", if ok { "ok" } else { "error" })]);
                }
                _ => {
                    if n > 60 {
                        self.ss = ss;
                        self.es = es;
                        let last = self.bytes[n - 1];
                        if last == 0xaa || last == 0x55 {
                            self.putx(ctx, vec!["Operation completed".into()]);
                        }
                    }
                }
            },
            // Read memory
            0xf0 => {
                if n == 2 {
                    self.ss = ss;
                } else if n == 3 {
                    self.es = es;
                    let address = (u32::from(self.bytes[2]) << 8) + u32::from(self.bytes[1]);
                    self.putx(ctx, vec![format!("Target address: 0x{address:04x}")]);
                } else if n > 3 {
                    self.ss = ss;
                    self.es = es;
                    let last = self.bytes[n - 1];
                    self.putx(ctx, vec![format!("Data: 0x{last:02x}")]);
                    ctx.put_binary(ss, es, 0, vec![last]);
                }
            }
            _ => {}
        }
    }
}

/// A Python integer as an unsigned 64 bit value.
fn item_u64(item: &Item) -> Result<u64> {
    match item {
        Item::Int(v) if *v >= 0 => Ok(*v as u64),
        Item::BigUint(bytes) if bytes.len() <= 8 => {
            Ok(bytes.iter().fold(0u64, |acc, &b| (acc << 8) | u64::from(b)))
        }
        _ => Err(Error::device("ds243x: expected a non-negative integer")),
    }
}

impl Decoder for Ds243x {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.bytes.clear();
        self.family_code = None;
        self.family.clear();
        self.commands = &COMMANDS_2432;
        self.ss = 0;
        self.es = 0;
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("ds243x is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let Item::List(parts) = item else {
            return Err(Error::device("ds243x: expected a onewire_network packet"));
        };
        if parts.len() != 2 {
            return Err(Error::device("ds243x: expected a two element packet"));
        }
        let Item::Str(code) = &parts[0] else {
            return Err(Error::device("ds243x: packet type is not a string"));
        };
        match code.as_str() {
            "RESET/PRESENCE" => {
                let present = match &parts[1] {
                    Item::Bool(b) => *b,
                    Item::Int(v) => *v != 0,
                    _ => return Err(Error::device("ds243x: malformed reset/presence")),
                };
                self.ss = ss;
                self.es = es;
                self.putx(
                    ctx,
                    vec![format!("Reset/presence: {}", if present { "true" } else { "false" })],
                );
                self.bytes.clear();
            }
            "ROM" => {
                let val = item_u64(&parts[1])?;
                self.handle_rom(ctx, val, ss, es);
            }
            "DATA" => {
                let val = match &parts[1] {
                    Item::Int(v) => *v as u8,
                    _ => return Err(Error::device("ds243x: data byte is not an integer")),
                };
                self.handle_data(ctx, val, ss, es);
            }
            _ => {}
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::crc16;

    #[test]
    fn crc16_matches_maxim() {
        // The DS2432 test capture's scratchpad is validated at runtime;
        // these values come from the reference polynomial.
        assert_eq!(crc16(&[]), 0xffff);
        assert_eq!(crc16(&[0x00]), 0xffff);
        assert_eq!(crc16(&[0xff]), 0xbfbf);
    }
}
