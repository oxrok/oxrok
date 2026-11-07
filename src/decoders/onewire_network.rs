// Counterpart in sigrok:
//   libsigrokdecode/decoders/onewire_network/pd.py,
//   Copyright (C) 2012 Iztok Jeras <iztok.jeras@gmail.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the 1-Wire network layer and the decoder's
//   documented ROM commands and texts.

//! 1-Wire network layer: ROM commands, 64 bit device addresses and data.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// `command`: ROM command, its name and the next state.
static COMMAND: [(u64, &str, &str); 10] = [
    (0x33, "Read ROM", "GET ROM"),
    (0x0f, "Conditional read ROM", "GET ROM"),
    (0xcc, "Skip ROM", "TRANSPORT"),
    (0x55, "Match ROM", "GET ROM"),
    (0xf0, "Search ROM", "SEARCH ROM"),
    (0xec, "Conditional search ROM", "SEARCH ROM"),
    (0x3c, "Overdrive skip ROM", "TRANSPORT"),
    (0x69, "Overdrive match ROM", "GET ROM"),
    (0xa5, "Resume", "TRANSPORT"),
    (0x96, "DS2408: Disable Test Mode", "GET ROM"),
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "onewire_network",
    name: "1-Wire network layer",
    longname: "1-Wire serial communication bus (network layer)",
    desc: "Bidirectional, half-duplex, asynchronous serial bus.",
    license: "gplv2+",
    inputs: &["onewire_link"],
    outputs: &["onewire_network"],
    tags: &["Embedded/industrial"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[AnnClass { id: "text", desc: "Human-readable text" }],
    annotation_rows: &[],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'onewire_link' PD and decodes the\n\
1-Wire protocol (network layer).\n\
\n\
The 1-Wire protocol enables bidirectional communication over a single wire\n\
(and ground) between a single master and one or multiple slaves. The protocol\n\
is layered:\n\
\n\
 - Link layer (reset, presence detection, reading/writing bits)\n\
 - Network layer (skip/search/match device ROM addresses)\n\
 - Transport layer (transport data between 1-Wire master and device)\n\
\n\
Network layer:\n\
\n\
The following link layer annotations are shown:\n\
\n\
 - RESET/PRESENCE True/False\n\
   The event is marked from the signal negative edge to the end of the reset\n\
   high period. It is also reported if there are any devices attached to the\n\
   bus.\n\
\n\
The following network layer annotations are shown:\n\
\n\
 - ROM command <val> <name>\n\
   The requested ROM command is displayed as an 8bit hex value and by name.\n\
 - ROM <val>\n\
   The 64bit value of the addressed device is displayed:\n\
   Family code (1 byte) + serial number (6 bytes) + CRC (1 byte)\n\
 - Data <val>\n\
   Data intended for the transport layer is displayed as an 8bit hex value.\n\
\n\
TODO:\n\
 - Add CRC checks, to see if there were communication errors on the wire.\n\
 - Add reporting original/complement address values from the search algorithm.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(OnewireNetwork::default())
}

/// A Python integer as an item: `Int` when it fits, else `BigUint` in
/// big-endian bytes without leading zeros, as the Python host does.
fn uint_item(v: u64) -> Item {
    if let Ok(small) = i64::try_from(v) {
        Item::Int(small)
    } else {
        let bytes = v.to_be_bytes();
        let first = bytes.iter().position(|&b| b != 0).unwrap_or(bytes.len());
        Item::BigUint(bytes[first..].to_vec())
    }
}

/// The mask `(1 << length) - 1`, which Python computes without an
/// overflow for a 64 bit length.
fn bit_mask(length: usize) -> u64 {
    if length >= 64 { u64::MAX } else { (1u64 << length) - 1 }
}

/// The 1-Wire network layer decoder.
#[derive(Default)]
pub struct OnewireNetwork {
    ss: u64,
    es: u64,
    ss_block: u64,
    es_block: u64,
    state: &'static str,
    bit_cnt: usize,
    search: u8,
    data_p: u64,
    data_n: u64,
    data: u64,
    rom: u64,
}

impl OnewireNetwork {
    fn putx(&self, ctx: &mut dyn Context, texts: Vec<String>) {
        ctx.put_ann(self.ss_block, self.es_block, 0, texts);
    }

    fn puty(&self, ctx: &mut dyn Context, item: Item) {
        ctx.put_python(self.ss_block, self.es_block, item);
    }

    /// `onewire_collect`: collect `length` bits LSB first.
    fn collect(&mut self, length: usize, val: u64, ss: u64, es: u64) -> bool {
        if self.bit_cnt == 0 {
            self.ss_block = ss;
        }
        self.data = (self.data & !(1u64 << self.bit_cnt)) | (val << self.bit_cnt);
        self.bit_cnt += 1;
        // In case the full length of the sequence is received, return 1.
        if self.bit_cnt == length {
            self.es_block = es;
            self.data &= bit_mask(length);
            self.bit_cnt = 0;
            true
        } else {
            false
        }
    }

    /// `onewire_search`: collect the original, complement and selected
    /// address bits of the 1-Wire search algorithm.
    fn search_rom(&mut self, length: usize, val: u64, ss: u64, es: u64) -> bool {
        // Storing the sample this sequence begins with.
        if self.bit_cnt == 0 && self.search == b'P' {
            self.ss_block = ss;
        }

        match self.search {
            b'P' => {
                // Master receives an original address bit.
                self.data_p = (self.data_p & !(1u64 << self.bit_cnt)) | (val << self.bit_cnt);
                self.search = b'N';
            }
            b'N' => {
                // Master receives a complemented address bit.
                self.data_n = (self.data_n & !(1u64 << self.bit_cnt)) | (val << self.bit_cnt);
                self.search = b'D';
            }
            b'D' => {
                // Master transmits an address bit.
                self.data = (self.data & !(1u64 << self.bit_cnt)) | (val << self.bit_cnt);
                self.search = b'P';
                self.bit_cnt += 1;
            }
            _ => {}
        }

        // In case the full length of the sequence is received, return 1.
        if self.bit_cnt == length {
            self.es_block = es;
            self.data_p &= bit_mask(length);
            self.data_n &= bit_mask(length);
            self.data &= bit_mask(length);
            self.search = b'P';
            self.bit_cnt = 0;
            true
        } else {
            false
        }
    }

    fn handle_item(
        &mut self,
        ctx: &mut dyn Context,
        code: &str,
        val: &Item,
        ss: u64,
        es: u64,
    ) -> Result<()> {
        // State machine.
        if code == "RESET/PRESENCE" {
            let present = match val {
                Item::Bool(b) => *b,
                Item::Int(v) => *v != 0,
                _ => return Err(Error::device("onewire_network: malformed reset/presence")),
            };
            self.search = b'P';
            self.bit_cnt = 0;
            ctx.put_ann(
                ss,
                es,
                0,
                vec![format!("Reset/presence: {}", if present { "true" } else { "false" })],
            );
            ctx.put_python(
                ss,
                es,
                Item::List(vec![Item::Str("RESET/PRESENCE".into()), Item::Bool(present)]),
            );
            self.state = "COMMAND";
            return Ok(());
        }

        // For now we're only interested in 'RESET/PRESENCE' and 'BIT' packets.
        if code != "BIT" {
            return Ok(());
        }

        let bit = match val {
            Item::Int(v) => u64::from(*v != 0),
            Item::Bool(b) => u64::from(*b),
            _ => return Err(Error::device("onewire_network: malformed bit")),
        };

        match self.state {
            "COMMAND" => {
                // Receiving and decoding a ROM command.
                if !self.collect(8, bit, ss, es) {
                    return Ok(());
                }
                if let Some((_, name, next)) =
                    COMMAND.iter().find(|(code, _, _)| *code == self.data)
                {
                    self.putx(ctx, vec![format!("ROM command: 0x{:02x} '{}'", self.data, name)]);
                    self.state = next;
                } else {
                    self.putx(
                        ctx,
                        vec![format!("ROM command: 0x{:02x} 'unrecognized'", self.data)],
                    );
                    self.state = "COMMAND ERROR";
                }
            }
            "GET ROM" => {
                // A 64 bit device address is selected.
                // Family code (1 byte) + serial number (6 bytes) + CRC (1 byte)
                if !self.collect(64, bit, ss, es) {
                    return Ok(());
                }
                self.rom = self.data & 0xffff_ffff_ffff_ffff;
                self.putx(ctx, vec![format!("ROM: 0x{:016x}", self.rom)]);
                self.puty(ctx, Item::List(vec![Item::Str("ROM".into()), uint_item(self.rom)]));
                self.state = "TRANSPORT";
            }
            "SEARCH ROM" => {
                // A 64 bit device address is searched for.
                // Family code (1 byte) + serial number (6 bytes) + CRC (1 byte)
                if !self.search_rom(64, bit, ss, es) {
                    return Ok(());
                }
                self.rom = self.data & 0xffff_ffff_ffff_ffff;
                self.putx(ctx, vec![format!("ROM: 0x{:016x}", self.rom)]);
                self.puty(ctx, Item::List(vec![Item::Str("ROM".into()), uint_item(self.rom)]));
                self.state = "TRANSPORT";
            }
            "TRANSPORT" => {
                // The transport layer is handled in byte sized units.
                if !self.collect(8, bit, ss, es) {
                    return Ok(());
                }
                self.putx(ctx, vec![format!("Data: 0x{:02x}", self.data)]);
                self.puty(
                    ctx,
                    Item::List(vec![Item::Str("DATA".into()), Item::Int(self.data as i64)]),
                );
            }
            "COMMAND ERROR" => {
                // Since the command is not recognized, print raw data.
                if !self.collect(8, bit, ss, es) {
                    return Ok(());
                }
                self.putx(ctx, vec![format!("ROM error data: 0x{:02x}", self.data)]);
            }
            _ => {}
        }
        Ok(())
    }
}

impl Decoder for OnewireNetwork {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.ss_block = 0;
        self.es_block = 0;
        self.state = "COMMAND";
        self.bit_cnt = 0;
        self.search = b'P';
        self.data_p = 0;
        self.data_n = 0;
        self.data = 0;
        self.rom = 0;
        self.ss = 0;
        self.es = 0;
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("onewire_network is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let Item::List(parts) = item else {
            return Err(Error::device("onewire_network: expected a onewire_link packet"));
        };
        if parts.len() != 2 {
            return Err(Error::device("onewire_network: expected a two element packet"));
        }
        let Item::Str(code) = &parts[0] else {
            return Err(Error::device("onewire_network: packet type is not a string"));
        };
        self.ss = ss;
        self.es = es;
        self.handle_item(ctx, code, &parts[1], ss, es)
    }
}

#[cfg(test)]
mod tests {
    use super::{bit_mask, uint_item};
    use crate::decode::Item;

    #[test]
    fn masks_match_python() {
        assert_eq!(bit_mask(1), 1);
        assert_eq!(bit_mask(8), 0xff);
        assert_eq!(bit_mask(64), u64::MAX);
    }

    #[test]
    fn rom_values_pick_the_matching_item() {
        assert!(matches!(uint_item(0x28_00_00_00_00_00_00_00), Item::Int(_)));
        assert!(
            matches!(uint_item(0xf0_00_00_00_00_00_00_00), Item::BigUint(ref b) if b == &[0xf0, 0, 0, 0, 0, 0, 0, 0])
        );
    }
}
