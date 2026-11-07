// Counterpart in sigrok:
//   libsigrokdecode/decoders/eeprom24xx/pd.py and lists.py,
//   Copyright (C) 2014 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented classes and texts.

//! 24xx series I²C EEPROM: control words, word addresses and data.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item,
    OptionDesc, OptionValue,
};

/// Chip properties used by the decoder. Upstream's `chips` table has
/// more fields (vendor, model, size, `page_wraparound`, `max_speed`), but
/// the decoder itself only reads `addr_bytes` and `page_size`.
struct Chip {
    id: &'static str,
    /// Number of EEPROM address bytes used.
    addr_bytes: usize,
    /// Page size in bytes.
    page_size: i64,
}

/// In the order of `chips.keys()` in lists.py, which fixes the option's
/// value list.
static CHIPS: [Chip; 16] = [
    Chip { id: "generic", addr_bytes: 1, page_size: 8 },
    Chip { id: "microchip_24aa65", addr_bytes: 2, page_size: 64 },
    Chip { id: "microchip_24lc65", addr_bytes: 2, page_size: 64 },
    Chip { id: "microchip_24c65", addr_bytes: 2, page_size: 64 },
    Chip { id: "microchip_24aa64", addr_bytes: 2, page_size: 32 },
    Chip { id: "microchip_24lc64", addr_bytes: 2, page_size: 32 },
    Chip { id: "microchip_24aa02uid", addr_bytes: 1, page_size: 8 },
    Chip { id: "microchip_24aa025uid", addr_bytes: 1, page_size: 16 },
    Chip { id: "microchip_24aa025uid_sot23", addr_bytes: 1, page_size: 16 },
    Chip { id: "onsemi_cat24c256", addr_bytes: 2, page_size: 64 },
    Chip { id: "onsemi_cat24m01", addr_bytes: 2, page_size: 256 },
    Chip { id: "siemens_slx_24c01", addr_bytes: 1, page_size: 8 },
    Chip { id: "siemens_slx_24c02", addr_bytes: 1, page_size: 8 },
    Chip { id: "st_m24c01", addr_bytes: 1, page_size: 16 },
    Chip { id: "st_m24c02", addr_bytes: 1, page_size: 16 },
    Chip { id: "xicor_x24c02", addr_bytes: 1, page_size: 4 },
];

static CHIP_VALUES: [OptionValue; 16] = [
    OptionValue::s("generic"),
    OptionValue::s("microchip_24aa65"),
    OptionValue::s("microchip_24lc65"),
    OptionValue::s("microchip_24c65"),
    OptionValue::s("microchip_24aa64"),
    OptionValue::s("microchip_24lc64"),
    OptionValue::s("microchip_24aa02uid"),
    OptionValue::s("microchip_24aa025uid"),
    OptionValue::s("microchip_24aa025uid_sot23"),
    OptionValue::s("onsemi_cat24c256"),
    OptionValue::s("onsemi_cat24m01"),
    OptionValue::s("siemens_slx_24c01"),
    OptionValue::s("siemens_slx_24c02"),
    OptionValue::s("st_m24c01"),
    OptionValue::s("st_m24c02"),
    OptionValue::s("xicor_x24c02"),
];

fn chip(id: &str) -> &'static Chip {
    CHIPS.iter().find(|c| c.id == id).unwrap_or(&CHIPS[0])
}

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "eeprom24xx",
    name: "24xx EEPROM",
    longname: "24xx I²C EEPROM",
    desc: "24xx series I²C EEPROM protocol.",
    license: "gplv2+",
    inputs: &["i2c"],
    outputs: &[],
    tags: &["IC", "Memory"],
    channels: &[],
    optional_channels: &[],
    options: &[
        OptionDesc {
            id: "chip",
            desc: "Chip",
            default: OptionValue::s("generic"),
            values: &CHIP_VALUES,
        },
        OptionDesc {
            id: "addr_counter",
            desc: "Initial address counter value",
            default: OptionValue::i(0),
            values: &[],
        },
    ],
    annotations: &[
        // Warnings
        AnnClass { id: "warnings", desc: "Warnings" },
        // Bits/bytes
        AnnClass { id: "control-code", desc: "Control code" },
        AnnClass { id: "address-pin", desc: "Address pin (A0/A1/A2)" },
        AnnClass { id: "rw-bit", desc: "Read/write bit" },
        AnnClass { id: "word-addr-byte", desc: "Word address byte" },
        AnnClass { id: "data-byte", desc: "Data byte" },
        // Fields
        AnnClass { id: "control-word", desc: "Control word" },
        AnnClass { id: "word-addr", desc: "Word address" },
        AnnClass { id: "data", desc: "Data" },
        // Operations
        AnnClass { id: "byte-write", desc: "Byte write" },
        AnnClass { id: "page-write", desc: "Page write" },
        AnnClass { id: "cur-addr-read", desc: "Current address read" },
        AnnClass { id: "random-read", desc: "Random read" },
        AnnClass { id: "seq-random-read", desc: "Sequential random read" },
        AnnClass { id: "seq-cur-addr-read", desc: "Sequential current address read" },
        AnnClass { id: "ack-polling", desc: "Acknowledge polling" },
        AnnClass { id: "set-bank-addr", desc: "Set bank address" }, // SBA. Only 34AA04.
        AnnClass { id: "read-bank-addr", desc: "Read bank address" }, // RBA. Only 34AA04.
        AnnClass { id: "set-wp", desc: "Set write protection" },    // SWP
        AnnClass { id: "clear-all-wp", desc: "Clear all write protection" }, // CWP
        AnnClass { id: "read-wp", desc: "Read write protection status" }, // RPS
    ],
    annotation_rows: &[
        AnnRow { id: "bits-bytes", desc: "Bits/bytes", classes: &[1, 2, 3, 4, 5] },
        AnnRow { id: "fields", desc: "Fields", classes: &[6, 7, 8] },
        AnnRow {
            id: "ops",
            desc: "Operations",
            classes: &[9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20],
        },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[0] },
    ],
    binary: &[BinClass { id: "binary", desc: "Binary" }],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'i2c' PD and decodes the\n\
industry standard 24xx series serial EEPROM protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Eeprom24xx::default())
}

/// One I2C bit: value and sample range.
#[derive(Clone, Copy)]
struct Bit {
    value: i64,
    ss: u64,
    es: u64,
}

/// A packet the decoder keeps: `[ss, es, cmd, databyte, bits]`. The
/// command is kept for fidelity with upstream, which stores it but only
/// reads the sample range, data byte and bits.
#[derive(Clone)]
struct Packet {
    ss: u64,
    es: u64,
    #[allow(dead_code)]
    cmd: String,
    databyte: i64,
    bits: Vec<Bit>,
}

/// The state machine's states, named as upstream's `self.state`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    WaitForStart,
    GetControlWord,
    RGetAckNackAfterControlWord,
    RGetWordAddrOrByte,
    RGetAckNackAfterWordAddrOrByte,
    RGetRestart,
    RReadByte,
    RGetAckNackAfterByteWasRead,
    WGetAckNackAfterControlWord,
    WGetWordAddr,
    WGetAckAfterWordAddr,
    WDetermineReadOrWrite,
    WWriteByte,
    WGetAckNackAfterByteWasWritten,
    R2GetControlWord,
    R2GetAckAfterAddrRead,
    R2ReadByte,
    R2GetAckNackAfterByteWasRead,
    GetStopAfterLastByte,
}

/// The eeprom24xx decoder.
#[allow(clippy::struct_excessive_bools)] // as upstream's five flags
pub struct Eeprom24xx {
    chip: &'static Chip,
    addr_counter: i64,
    state: State,
    packets: Vec<Packet>,
    bytebuf: Vec<i64>,
    is_cur_addr_read: bool,
    is_random_access_read: bool,
    is_seq_random_read: bool,
    is_byte_write: bool,
    is_page_write: bool,
    ss: u64,
    es: u64,
    ss_block: u64,
    es_block: u64,
    cmd: String,
    databyte: Option<i64>,
    bits: Option<Vec<Bit>>,
}

impl Default for Eeprom24xx {
    fn default() -> Self {
        Eeprom24xx {
            chip: &CHIPS[0],
            addr_counter: 0,
            state: State::WaitForStart,
            packets: Vec::new(),
            bytebuf: Vec::new(),
            is_cur_addr_read: false,
            is_random_access_read: false,
            is_seq_random_read: false,
            is_byte_write: false,
            is_page_write: false,
            ss: 0,
            es: 0,
            ss_block: 0,
            es_block: 0,
            cmd: String::new(),
            databyte: None,
            bits: None,
        }
    }
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("eeprom24xx: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("eeprom24xx: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("eeprom24xx: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

/// The BITS packet payload is a list of `[bit, ss, es]`, index 0 the LSB.
fn parse_bits(data: &Item) -> Result<Vec<Bit>> {
    let Item::List(items) = data else {
        return Err(Error::device("eeprom24xx: BITS payload is not a list"));
    };
    let mut bits = Vec::with_capacity(items.len());
    for item in items {
        let Item::List(parts) = item else {
            return Err(Error::device("eeprom24xx: BITS entry is not a list"));
        };
        if parts.len() != 3 {
            return Err(Error::device("eeprom24xx: BITS entry is not [bit, ss, es]"));
        }
        let (Item::Int(value), Item::Int(ss), Item::Int(es)) = (&parts[0], &parts[1], &parts[2])
        else {
            return Err(Error::device("eeprom24xx: BITS entry is not [bit, ss, es]"));
        };
        bits.push(Bit { value: *value, ss: *ss as u64, es: *es as u64 });
    }
    Ok(bits)
}

impl Eeprom24xx {
    fn reset_variables(&mut self) {
        self.state = State::WaitForStart;
        self.packets.clear();
        self.bytebuf.clear();
        self.is_cur_addr_read = false;
        self.is_random_access_read = false;
        self.is_seq_random_read = false;
        self.is_byte_write = false;
        self.is_page_write = false;
    }

    fn packet_append(&mut self) {
        self.packets.push(Packet {
            ss: self.ss,
            es: self.es,
            cmd: self.cmd.clone(),
            databyte: self.databyte.unwrap_or(0),
            bits: self.bits.clone().unwrap_or_default(),
        });
        if self.cmd == "DATA READ" || self.cmd == "DATA WRITE" {
            self.bytebuf.push(self.databyte.unwrap_or(0));
        }
    }

    fn hexbytes(&self, idx: usize) -> String {
        self.bytebuf[idx..].iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
    }

    fn addr_and_len(&self) -> String {
        let a = if self.chip.addr_bytes == 1 {
            format!("{:02X}", self.bytebuf[0])
        } else {
            format!("{:02X}{:02X}", self.bytebuf[0], self.bytebuf[1])
        };
        let num_data_bytes = self.bytebuf.len() as i64 - self.chip.addr_bytes as i64;
        let mut d = format!("{num_data_bytes} bytes");
        if num_data_bytes <= 1 {
            d.pop();
        }
        format!("addr={a}, {d}")
    }

    fn putb(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_block, self.es_block, class, texts);
    }

    fn putbin(&self, ctx: &mut dyn Context, data: Vec<u8>) {
        ctx.put_binary(self.ss_block, self.es_block, 0, data);
    }

    fn put_control_word(&self, ctx: &mut dyn Context, bits: &[Bit]) {
        let s: String = (4..8).rev().map(|i| bits[i].value.to_string()).collect();
        ctx.put_ann(
            bits[7].ss,
            bits[4].es,
            1,
            vec![
                format!("Control code bits: {s}"),
                format!("Control code: {s}"),
                format!("Ctrl code: {s}"),
                "Ctrl code".to_owned(),
                "Ctrl".to_owned(),
                "C".to_owned(),
            ],
        );
        for i in (0..self.chip_addr_pins()).rev() {
            ctx.put_ann(
                bits[i + 1].ss,
                bits[i + 1].es,
                2,
                vec![
                    format!("Address bit {i}: {}", bits[i + 1].value),
                    format!("Addr bit {i}"),
                    format!("A{i}"),
                    "A".to_owned(),
                ],
            );
        }
        let (s1, s2) = if bits[0].value == 1 { ("read", "R") } else { ("write", "W") };
        ctx.put_ann(
            bits[0].ss,
            bits[0].es,
            3,
            vec![format!("R/W bit: {s1}"), "R/W".to_owned(), "RW".to_owned(), s2.to_owned()],
        );
        ctx.put_ann(
            bits[7].ss,
            bits[0].es,
            6,
            vec!["Control word".to_owned(), "Control".to_owned(), "CW".to_owned(), "C".to_owned()],
        );
    }

    /// Number of A0/A1/A2 pins of the selected chip.
    fn chip_addr_pins(&self) -> usize {
        match self.chip.id {
            "microchip_24aa02uid" | "siemens_slx_24c01" | "siemens_slx_24c02" => 0,
            "microchip_24aa025uid_sot23" | "onsemi_cat24m01" => 2,
            _ => 3,
        }
    }

    fn put_word_addr(&mut self, ctx: &mut dyn Context, packets: &[Packet]) {
        if self.chip.addr_bytes == 1 {
            let a = packets[1].databyte;
            ctx.put_ann(
                packets[1].ss,
                packets[1].es,
                4,
                vec![
                    format!("Word address byte: {a:02X}"),
                    format!("Word addr byte: {a:02X}"),
                    format!("Addr: {a:02X}"),
                    format!("A: {a:02X}"),
                    format!("{a:02X}"),
                ],
            );
            ctx.put_ann(
                packets[1].ss,
                packets[1].es,
                7,
                vec![
                    "Word address".to_owned(),
                    "Word addr".to_owned(),
                    "Addr".to_owned(),
                    "A".to_owned(),
                ],
            );
            self.addr_counter = a;
        } else {
            let a = packets[1].databyte;
            ctx.put_ann(
                packets[1].ss,
                packets[1].es,
                4,
                vec![
                    format!("Word address high byte: {a:02X}"),
                    format!("Word addr high byte: {a:02X}"),
                    format!("Addr high: {a:02X}"),
                    format!("AH: {a:02X}"),
                    format!("{a:02X}"),
                ],
            );
            let a = packets[2].databyte;
            ctx.put_ann(
                packets[2].ss,
                packets[2].es,
                4,
                vec![
                    format!("Word address low byte: {a:02X}"),
                    format!("Word addr low byte: {a:02X}"),
                    format!("Addr low: {a:02X}"),
                    format!("AL: {a:02X}"),
                    format!("{a:02X}"),
                ],
            );
            ctx.put_ann(
                packets[1].ss,
                packets[2].es,
                7,
                vec![
                    "Word address".to_owned(),
                    "Word addr".to_owned(),
                    "Addr".to_owned(),
                    "A".to_owned(),
                ],
            );
            self.addr_counter = (packets[1].databyte << 8) | packets[2].databyte;
        }
    }

    fn put_data_byte(&self, ctx: &mut dyn Context, p: &Packet) {
        let s = if self.chip.addr_bytes == 1 {
            format!("{:02X}", self.addr_counter)
        } else {
            format!("{:04X}", self.addr_counter)
        };
        let v = p.databyte;
        ctx.put_ann(
            p.ss,
            p.es,
            5,
            vec![
                format!("Data byte {s}: {v:02X}"),
                format!("Data byte: {v:02X}"),
                format!("Byte: {v:02X}"),
                format!("DB: {v:02X}"),
                format!("{v:02X}"),
            ],
        );
    }

    fn put_data_bytes(
        &mut self,
        ctx: &mut dyn Context,
        packets: &[Packet],
        idx: usize,
        cls: usize,
        s: &str,
    ) {
        for p in &packets[idx..] {
            self.put_data_byte(ctx, p);
            self.addr_counter += 1;
        }
        ctx.put_ann(
            packets[idx].ss,
            packets[packets.len() - 1].es,
            8,
            vec!["Data".to_owned(), "D".to_owned()],
        );
        // ''.join(['%s' % c[0] for c in s.split()]).upper()
        let a: String = s
            .split_whitespace()
            .filter_map(|w| w.chars().next())
            .collect::<String>()
            .to_uppercase();
        let addr_and_len = self.addr_and_len();
        let hex = self.hexbytes(self.chip.addr_bytes);
        self.putb(
            ctx,
            cls,
            vec![
                format!("{s} ({addr_and_len}): {hex}"),
                format!("{s} ({addr_and_len})"),
                s.to_owned(),
                a,
                s.chars().next().unwrap_or(' ').to_string(),
            ],
        );
        let data: Vec<u8> = self.bytebuf[self.chip.addr_bytes..].iter().map(|&b| b as u8).collect();
        self.putbin(ctx, data);
    }

    fn decide_on_seq_or_rnd_read(&mut self) {
        if self.bytebuf.len() < 2 {
            self.reset_variables();
            return;
        }
        if self.bytebuf.len() == 2 {
            self.is_random_access_read = true;
        } else {
            self.is_seq_random_read = true;
        }
    }

    fn put_operation(&mut self, ctx: &mut dyn Context) {
        let packets = self.packets.clone();
        let idx = 1 + self.chip.addr_bytes;
        if self.is_byte_write {
            // Byte write: word address, one data byte.
            self.put_word_addr(ctx, &packets);
            self.put_data_bytes(ctx, &packets, idx, 9, "Byte write");
        } else if self.is_page_write {
            // Page write: word address, two or more data bytes.
            self.put_word_addr(ctx, &packets);
            let intitial_addr = self.addr_counter;
            self.put_data_bytes(ctx, &packets, idx, 10, "Page write");
            let num_bytes_to_write = (packets.len() - idx) as i64;
            if num_bytes_to_write > self.chip.page_size {
                self.putb(
                    ctx,
                    0,
                    vec![format!(
                        "Warning: Wrote {num_bytes_to_write} bytes but page size is only {} bytes!",
                        self.chip.page_size
                    )],
                );
            }
            let page1 = intitial_addr / self.chip.page_size;
            let page2 = (self.addr_counter - 1) / self.chip.page_size;
            if page1 != page2 {
                self.putb(
                    ctx,
                    0,
                    vec![format!(
                        "Warning: Page write crossed page boundary from page {page1} to {page2}!"
                    )],
                );
            }
        } else if self.is_cur_addr_read {
            // Current address read: no word address, one data byte.
            self.put_data_byte(ctx, &packets[1]);
            ctx.put_ann(
                packets[1].ss,
                packets[packets.len() - 1].es,
                8,
                vec!["Data".to_owned(), "D".to_owned()],
            );
            self.putb(
                ctx,
                11,
                vec![
                    format!("Current address read: {:02X}", self.bytebuf[0]),
                    "Current address read".to_owned(),
                    "Cur addr read".to_owned(),
                    "CAR".to_owned(),
                    "C".to_owned(),
                ],
            );
            self.putbin(ctx, vec![self.bytebuf[0] as u8]);
            self.addr_counter += 1;
        } else if self.is_random_access_read {
            // Random access read: word address, one data byte.
            self.put_control_word(ctx, &packets[idx].bits);
            self.put_word_addr(ctx, &packets);
            self.put_data_bytes(ctx, &packets, idx + 1, 12, "Random access read");
        } else if self.is_seq_random_read {
            // Sequential random read: word address, two or more data bytes.
            self.put_control_word(ctx, &packets[idx].bits);
            self.put_word_addr(ctx, &packets);
            self.put_data_bytes(ctx, &packets, idx + 1, 13, "Sequential random read");
        }
    }

    fn handle_wait_for_start(&mut self) {
        // Wait for an I²C START condition.
        if self.cmd != "START" && self.cmd != "START REPEAT" {
            return;
        }
        self.ss_block = self.ss;
        self.state = State::GetControlWord;
    }

    fn handle_get_control_word(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // The packet after START must be an ADDRESS READ or ADDRESS WRITE.
        if self.cmd != "ADDRESS READ" && self.cmd != "ADDRESS WRITE" {
            self.reset_variables();
            return Ok(());
        }
        self.packet_append();
        let bits = self
            .bits
            .clone()
            .ok_or_else(|| Error::device("eeprom24xx: control word without a BITS packet"))?;
        self.put_control_word(ctx, &bits);
        self.state = if self.cmd.starts_with("ADDRESS R") {
            State::RGetAckNackAfterControlWord
        } else {
            State::WGetAckNackAfterControlWord
        };
        Ok(())
    }

    fn handle_r_get_ack_nack_after_control_word(&mut self, ctx: &mut dyn Context) {
        if self.cmd == "ACK" {
            self.state = State::RGetWordAddrOrByte;
        } else if self.cmd == "NACK" {
            self.es_block = self.es;
            self.putb(ctx, 0, vec!["Warning: No reply from slave!".to_owned()]);
            self.reset_variables();
        } else {
            self.reset_variables();
        }
    }

    fn handle_r_get_word_addr_or_byte(&mut self, ctx: &mut dyn Context) {
        if self.cmd == "STOP" {
            self.es_block = self.es;
            self.putb(ctx, 0, vec!["Warning: Slave replied, but master aborted!".to_owned()]);
            self.reset_variables();
            return;
        } else if self.cmd != "DATA READ" {
            self.reset_variables();
            return;
        }
        self.packet_append();
        self.state = State::RGetAckNackAfterWordAddrOrByte;
    }

    fn handle_r_get_ack_nack_after_word_addr_or_byte(&mut self) {
        if self.cmd == "ACK" {
            self.state = State::RGetRestart;
        } else if self.cmd == "NACK" {
            self.is_cur_addr_read = true;
            self.state = State::GetStopAfterLastByte;
        } else {
            self.reset_variables();
        }
    }

    fn handle_r_get_restart(&mut self) {
        if self.cmd == "RESTART" {
            self.state = State::RReadByte;
        } else {
            self.reset_variables();
        }
    }

    fn handle_r_read_byte(&mut self) {
        if self.cmd == "DATA READ" {
            self.packet_append();
            self.state = State::RGetAckNackAfterByteWasRead;
        } else {
            self.reset_variables();
        }
    }

    fn handle_r_get_ack_nack_after_byte_was_read(&mut self) {
        if self.cmd == "ACK" {
            self.state = State::RReadByte;
        } else if self.cmd == "NACK" {
            // It's either a RANDOM READ or a SEQUENTIAL READ.
            self.state = State::GetStopAfterLastByte;
        } else {
            self.reset_variables();
        }
    }

    fn handle_w_get_ack_nack_after_control_word(&mut self, ctx: &mut dyn Context) {
        if self.cmd == "ACK" {
            self.state = State::WGetWordAddr;
        } else if self.cmd == "NACK" {
            self.es_block = self.es;
            self.putb(ctx, 0, vec!["Warning: No reply from slave!".to_owned()]);
            self.reset_variables();
        } else {
            self.reset_variables();
        }
    }

    fn handle_w_get_word_addr(&mut self, ctx: &mut dyn Context) {
        if self.cmd == "STOP" {
            self.es_block = self.es;
            self.putb(ctx, 0, vec!["Warning: Slave replied, but master aborted!".to_owned()]);
            self.reset_variables();
            return;
        } else if self.cmd != "DATA WRITE" {
            self.reset_variables();
            return;
        }
        self.packet_append();
        self.state = State::WGetAckAfterWordAddr;
    }

    fn handle_w_get_ack_after_word_addr(&mut self) {
        if self.cmd == "ACK" {
            self.state = State::WDetermineReadOrWrite;
        } else {
            self.reset_variables();
        }
    }

    fn handle_w_determine_eeprom_read_or_write(&mut self) {
        if self.cmd == "START REPEAT" {
            // It's either a RANDOM ACCESS READ or SEQUENTIAL RANDOM READ.
            self.state = State::R2GetControlWord;
        } else if self.cmd == "DATA WRITE" {
            self.packet_append();
            self.state = State::WGetAckNackAfterByteWasWritten;
        } else {
            self.reset_variables();
        }
    }

    fn handle_w_write_byte(&mut self, ctx: &mut dyn Context) {
        if self.cmd == "DATA WRITE" {
            self.packet_append();
            self.state = State::WGetAckNackAfterByteWasWritten;
        } else if self.cmd == "STOP" {
            if self.bytebuf.len() < 2 {
                self.reset_variables();
                return;
            }
            self.es_block = self.es;
            if self.bytebuf.len() == 2 {
                self.is_byte_write = true;
            } else {
                self.is_page_write = true;
            }
            self.put_operation(ctx);
            self.reset_variables();
        } else if self.cmd == "START REPEAT" {
            // It's either a RANDOM ACCESS READ or SEQUENTIAL RANDOM READ.
            self.state = State::R2GetControlWord;
        } else {
            self.reset_variables();
        }
    }

    fn handle_w_get_ack_nack_after_byte_was_written(&mut self) {
        if self.cmd == "ACK" {
            self.state = State::WWriteByte;
        } else {
            self.reset_variables();
        }
    }

    fn handle_r2_get_control_word(&mut self) {
        if self.cmd == "ADDRESS READ" {
            self.packet_append();
            self.state = State::R2GetAckAfterAddrRead;
        } else {
            self.reset_variables();
        }
    }

    fn handle_r2_get_ack_after_addr_read(&mut self) {
        if self.cmd == "ACK" {
            self.state = State::R2ReadByte;
        } else {
            self.reset_variables();
        }
    }

    fn handle_r2_read_byte(&mut self, ctx: &mut dyn Context) {
        if self.cmd == "DATA READ" {
            self.packet_append();
            self.state = State::R2GetAckNackAfterByteWasRead;
        } else if self.cmd == "STOP" {
            self.decide_on_seq_or_rnd_read();
            self.es_block = self.es;
            self.putb(ctx, 0, vec!["Warning: STOP expected after a NACK (not ACK)".to_owned()]);
            self.put_operation(ctx);
            self.reset_variables();
        } else {
            self.reset_variables();
        }
    }

    fn handle_r2_get_ack_nack_after_byte_was_read(&mut self) {
        if self.cmd == "ACK" {
            self.state = State::R2ReadByte;
        } else if self.cmd == "NACK" {
            self.decide_on_seq_or_rnd_read();
            self.state = State::GetStopAfterLastByte;
        } else {
            self.reset_variables();
        }
    }

    fn handle_get_stop_after_last_byte(&mut self, ctx: &mut dyn Context) {
        if self.cmd == "STOP" {
            self.es_block = self.es;
            self.put_operation(ctx);
            self.reset_variables();
        } else if self.cmd == "START REPEAT" {
            self.es_block = self.es;
            self.putb(ctx, 0, vec!["Warning: STOP expected (not RESTART)".to_owned()]);
            self.put_operation(ctx);
            self.reset_variables();
            self.ss_block = self.ss;
            self.state = State::GetControlWord;
        } else {
            self.reset_variables();
        }
    }

    fn handle(&mut self, ctx: &mut dyn Context) -> Result<()> {
        match self.state {
            State::WaitForStart => self.handle_wait_for_start(),
            State::GetControlWord => self.handle_get_control_word(ctx)?,
            State::RGetAckNackAfterControlWord => {
                self.handle_r_get_ack_nack_after_control_word(ctx);
            }
            State::RGetWordAddrOrByte => self.handle_r_get_word_addr_or_byte(ctx),
            State::RGetAckNackAfterWordAddrOrByte => {
                self.handle_r_get_ack_nack_after_word_addr_or_byte();
            }
            State::RGetRestart => self.handle_r_get_restart(),
            State::RReadByte => self.handle_r_read_byte(),
            State::RGetAckNackAfterByteWasRead => self.handle_r_get_ack_nack_after_byte_was_read(),
            State::WGetAckNackAfterControlWord => {
                self.handle_w_get_ack_nack_after_control_word(ctx);
            }
            State::WGetWordAddr => self.handle_w_get_word_addr(ctx),
            State::WGetAckAfterWordAddr => self.handle_w_get_ack_after_word_addr(),
            State::WDetermineReadOrWrite => self.handle_w_determine_eeprom_read_or_write(),
            State::WWriteByte => self.handle_w_write_byte(ctx),
            State::WGetAckNackAfterByteWasWritten => {
                self.handle_w_get_ack_nack_after_byte_was_written();
            }
            State::R2GetControlWord => self.handle_r2_get_control_word(),
            State::R2GetAckAfterAddrRead => self.handle_r2_get_ack_after_addr_read(),
            State::R2ReadByte => self.handle_r2_read_byte(ctx),
            State::R2GetAckNackAfterByteWasRead => {
                self.handle_r2_get_ack_nack_after_byte_was_read();
            }
            State::GetStopAfterLastByte => self.handle_get_stop_after_last_byte(ctx),
        }
        Ok(())
    }
}

impl Decoder for Eeprom24xx {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.chip = chip(config.str("chip", "generic"));
        self.addr_counter = config.int("addr_counter", 0);
        self.reset_variables();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("eeprom24xx takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;
        cmd.clone_into(&mut self.cmd);
        self.databyte = match data {
            Item::Int(v) => Some(*v),
            _ => None,
        };

        // Collect the 'BITS' packet, then return. The next packet is
        // guaranteed to belong to these bits we just stored.
        if self.cmd == "BITS" {
            self.bits = Some(parse_bits(data)?);
            return Ok(());
        }

        // Store the start/end samples of this I²C packet.
        self.ss = ss;
        self.es = es;

        self.handle(ctx)
    }
}
