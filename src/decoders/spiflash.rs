// Counterpart in sigrok:
//   libsigrokdecode/decoders/spiflash/pd.py and lists.py,
//   Copyright (C) 2011-2016 Uwe Hermann <uwe@hermann-uwe.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the xx25 series SPI flash protocol and the
//   decoder's documented classes and texts.

//! SPI flash/EEPROM: the xx25 series command set on an SPI bus.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};

/// One command of lists.py's ordered `cmds` dict.
struct Cmd {
    id: u64,
    short: &'static str,
    desc: &'static str,
}

/// `cmds` from lists.py, in order; the annotation class indices are
/// this table's positions, followed by BIT, FIELD and WARN.
const CMDS: &[Cmd] = &[
    Cmd { id: 0x01, short: "WRSR", desc: "Write status register" },
    Cmd { id: 0x02, short: "PP", desc: "Page program" },
    Cmd { id: 0x03, short: "READ", desc: "Read data" },
    Cmd { id: 0x04, short: "WRDI", desc: "Write disable" },
    Cmd { id: 0x05, short: "RDSR", desc: "Read status register" },
    Cmd { id: 0x06, short: "WREN", desc: "Write enable" },
    Cmd { id: 0x0b, short: "FAST/READ", desc: "Fast read data" },
    Cmd { id: 0x20, short: "SE", desc: "Sector erase" },
    Cmd { id: 0x2b, short: "RDSCUR", desc: "Read security register" },
    Cmd { id: 0x2f, short: "WRSCUR", desc: "Write security register" },
    Cmd { id: 0x35, short: "RDSR2", desc: "Read status register 2" },
    Cmd { id: 0x60, short: "CE", desc: "Chip erase" },
    Cmd { id: 0x70, short: "ESRY", desc: "Enable SO to output RY/BY#" },
    Cmd { id: 0x80, short: "DSRY", desc: "Disable SO to output RY/BY#" },
    Cmd {
        id: 0x82,
        short: "WRITE1",
        desc: "Main memory page program through buffer 1 with built-in erase",
    },
    Cmd {
        id: 0x85,
        short: "WRITE2",
        desc: "Main memory page program through buffer 2 with built-in erase",
    },
    Cmd { id: 0x90, short: "REMS", desc: "Read electronic manufacturer & device ID" },
    Cmd { id: 0x9f, short: "RDID", desc: "Read identification" },
    Cmd { id: 0xab, short: "RDP/RES", desc: "Release from deep powerdown / Read electronic ID" },
    Cmd { id: 0xad, short: "CP", desc: "Continuously program mode" },
    Cmd { id: 0xb1, short: "ENSO", desc: "Enter secured OTP" },
    Cmd { id: 0xb9, short: "DP", desc: "Deep power down" },
    Cmd { id: 0xbb, short: "2READ", desc: "2x I/O read" },
    Cmd { id: 0xc1, short: "EXSO", desc: "Exit secured OTP" },
    Cmd { id: 0xc7, short: "CE2", desc: "Chip erase" },
    Cmd { id: 0xd7, short: "STATUS", desc: "Status register read" },
    Cmd { id: 0xd8, short: "BE", desc: "Block erase" },
    Cmd { id: 0xef, short: "REMS2", desc: "Read ID for 2x I/O mode" },
];

// Annotation class indices. The command classes are the CMDS positions;
// pd.py's Ann enum has BIT, FIELD, WARN after them.
const ANN_WRSR: usize = 0;
const ANN_PP: usize = 1;
const ANN_READ: usize = 2;
const ANN_WRDI: usize = 3;
const ANN_RDSR: usize = 4;
const ANN_WREN: usize = 5;
const ANN_FAST_READ: usize = 6;
const ANN_SE: usize = 7;
const ANN_RDSR2: usize = 10;
const ANN_CE: usize = 11;
const ANN_WRITE1: usize = 14;
const ANN_WRITE2: usize = 15;
const ANN_REMS: usize = 16;
const ANN_RDID: usize = 17;
const ANN_RDP_RES: usize = 18;
const ANN_READ2X: usize = 22;
const ANN_CE2: usize = 24;
const ANN_STATUS: usize = 25;
const ANN_BIT: usize = 28;
const ANN_FIELD: usize = 29;
const ANN_WARN: usize = 30;

/// `chips` from lists.py. Only the vendor strings are observable here.
struct Chip {
    id: &'static str,
    vendor: &'static str,
}

const CHIPS: &[Chip] = &[
    Chip { id: "adesto_at45db161e", vendor: "Adesto" },
    Chip { id: "atmel_at25128", vendor: "Atmel" },
    Chip { id: "atmel_at25256", vendor: "Atmel" },
    Chip { id: "fidelix_fm25q32", vendor: "FIDELIX" },
    Chip { id: "macronix_mx25l1605d", vendor: "Macronix" },
    Chip { id: "macronix_mx25l3205d", vendor: "Macronix" },
    Chip { id: "macronix_mx25l6405d", vendor: "Macronix" },
    Chip { id: "winbond_w25q80dv", vendor: "Winbond" },
];

/// `device_name` from lists.py, keyed by the chip id's vendor prefix.
/// There is no `atmel` entry; pd.py raises `KeyError` for those chips.
const DEVICE_NAMES: &[(&str, &[(u64, &str)])] = &[
    ("adesto", &[(0x00, "AT45Dxxx family, standard series")]),
    ("fidelix", &[(0x15, "FM25Q32")]),
    ("macronix", &[(0x14, "MX25L1605D"), (0x15, "MX25L3205D"), (0x16, "MX25L6405D")]),
    ("winbond", &[(0x13, "W25Q80DV")]),
];

static OPTIONS: [OptionDesc; 2] = [
    OptionDesc {
        id: "chip",
        desc: "Chip",
        // tuple(chips.keys())[0]
        default: OptionValue::s("adesto_at45db161e"),
        values: &[
            OptionValue::s("adesto_at45db161e"),
            OptionValue::s("atmel_at25128"),
            OptionValue::s("atmel_at25256"),
            OptionValue::s("fidelix_fm25q32"),
            OptionValue::s("macronix_mx25l1605d"),
            OptionValue::s("macronix_mx25l3205d"),
            OptionValue::s("macronix_mx25l6405d"),
            OptionValue::s("winbond_w25q80dv"),
        ],
    },
    OptionDesc {
        id: "format",
        desc: "Data format",
        default: OptionValue::s("hex"),
        values: &[OptionValue::s("hex"), OptionValue::s("ascii")],
    },
];

/// The decoder description. `cmd_annotation_classes()` builds the ids
/// from the lowercased command short names.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "spiflash",
    name: "SPI flash/EEPROM",
    longname: "SPI flash/EEPROM chips",
    desc: "xx25 series SPI (NOR) flash/EEPROM chip protocol.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["IC", "Memory"],
    channels: &[],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "wrsr", desc: "Write status register" },
        AnnClass { id: "pp", desc: "Page program" },
        AnnClass { id: "read", desc: "Read data" },
        AnnClass { id: "wrdi", desc: "Write disable" },
        AnnClass { id: "rdsr", desc: "Read status register" },
        AnnClass { id: "wren", desc: "Write enable" },
        AnnClass { id: "fast/read", desc: "Fast read data" },
        AnnClass { id: "se", desc: "Sector erase" },
        AnnClass { id: "rdscur", desc: "Read security register" },
        AnnClass { id: "wrscur", desc: "Write security register" },
        AnnClass { id: "rdsr2", desc: "Read status register 2" },
        AnnClass { id: "ce", desc: "Chip erase" },
        AnnClass { id: "esry", desc: "Enable SO to output RY/BY#" },
        AnnClass { id: "dsry", desc: "Disable SO to output RY/BY#" },
        AnnClass {
            id: "write1",
            desc: "Main memory page program through buffer 1 with built-in erase",
        },
        AnnClass {
            id: "write2",
            desc: "Main memory page program through buffer 2 with built-in erase",
        },
        AnnClass { id: "rems", desc: "Read electronic manufacturer & device ID" },
        AnnClass { id: "rdid", desc: "Read identification" },
        AnnClass { id: "rdp/res", desc: "Release from deep powerdown / Read electronic ID" },
        AnnClass { id: "cp", desc: "Continuously program mode" },
        AnnClass { id: "enso", desc: "Enter secured OTP" },
        AnnClass { id: "dp", desc: "Deep power down" },
        AnnClass { id: "2read", desc: "2x I/O read" },
        AnnClass { id: "exso", desc: "Exit secured OTP" },
        AnnClass { id: "ce2", desc: "Chip erase" },
        AnnClass { id: "status", desc: "Status register read" },
        AnnClass { id: "be", desc: "Block erase" },
        AnnClass { id: "rems2", desc: "Read ID for 2x I/O mode" },
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "field", desc: "Field" },
        AnnClass { id: "warning", desc: "Warning" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[28] },
        AnnRow { id: "fields", desc: "Fields", classes: &[29] },
        AnnRow {
            id: "commands",
            desc: "Commands",
            classes: &[
                0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
                23, 24, 25, 26, 27,
            ],
        },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[30] },
    ],
    binary: &[],
    // pd.py has no module docstring, so the Python decoder's doc is "".
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Spiflash::default())
}

use std::fmt::Write as _;

/// A non-negative Python integer from an SPI word, of any width.
#[derive(Clone, Default, PartialEq, Eq, Debug)]
struct Int(Vec<u8>);

impl Int {
    fn from_u64(v: u64) -> Self {
        let mut bytes = v.to_le_bytes().to_vec();
        while bytes.last() == Some(&0) {
            bytes.pop();
        }
        Int(bytes)
    }

    fn from_be_bytes(bytes: &[u8]) -> Self {
        let mut out: Vec<u8> = bytes.iter().rev().copied().collect();
        while out.last() == Some(&0) {
            out.pop();
        }
        Int(out)
    }

    fn from_item(item: &Item) -> Result<Self> {
        match item {
            Item::Int(v) if *v >= 0 => Ok(Int::from_u64(*v as u64)),
            Item::BigUint(bytes) => Ok(Int::from_be_bytes(bytes)),
            _ => Err(Error::arg(format!("spiflash: not an SPI word: {item}"))),
        }
    }

    fn is_zero(&self) -> bool {
        self.0.is_empty()
    }

    fn bit(&self, bit: u32) -> bool {
        let byte = self.0.get((bit / 8) as usize).copied().unwrap_or(0);
        byte & (1 << (bit % 8)) != 0
    }

    fn eq_u64(&self, v: u64) -> bool {
        *self == Int::from_u64(v)
    }

    fn to_u64(&self) -> Option<u64> {
        if self.0.len() > 8 {
            return None;
        }
        let mut v = [0u8; 8];
        v[..self.0.len()].copy_from_slice(&self.0);
        Some(u64::from_le_bytes(v))
    }

    /// Python's `'%0Nx' % value`, lowercase, at least `width` digits.
    fn hex(&self, width: usize) -> String {
        let mut s = String::new();
        for (i, b) in self.0.iter().rev().enumerate() {
            if i == 0 {
                let _ = write!(s, "{b:x}");
            } else {
                let _ = write!(s, "{b:02x}");
            }
        }
        if s.is_empty() {
            s.push('0');
        }
        while s.len() < width {
            s.insert(0, '0');
        }
        s
    }

    /// Python's `'%d' % value`.
    fn decimal(&self) -> String {
        let mut n = self.0.clone();
        if n.is_empty() {
            return "0".to_owned();
        }
        let mut digits = Vec::new();
        while !n.is_empty() {
            let mut rem = 0u32;
            for b in n.iter_mut().rev() {
                let cur = (rem << 8) | u32::from(*b);
                *b = (cur / 10) as u8;
                rem = cur % 10;
            }
            digits.push(b'0' + rem as u8);
            while n.last() == Some(&0) {
                n.pop();
            }
        }
        digits.reverse();
        String::from_utf8(digits).unwrap_or_default()
    }

    /// `value % m`, without leaving the byte representation.
    fn mod_u64(&self, m: u64) -> u64 {
        let mut rem = 0u128;
        for b in self.0.iter().rev() {
            rem = (rem * 256 + u128::from(*b)) % u128::from(m);
        }
        rem as u64
    }

    /// `self |= other << (8 * shift_bytes)`, as pd.py accumulates the
    /// 24 bit address.
    fn or_shift(&mut self, other: &Int, shift_bytes: usize) {
        if other.0.is_empty() {
            return;
        }
        let need = shift_bytes + other.0.len();
        if self.0.len() < need {
            self.0.resize(need, 0);
        }
        for (i, b) in other.0.iter().enumerate() {
            self.0[shift_bytes + i] |= *b;
        }
    }

    /// Python's `chr(value)`.
    fn to_char(&self) -> Result<char> {
        let v = self
            .to_u64()
            .filter(|&v| v <= 0x0010_ffff && !(0xd800..=0xdfff).contains(&v))
            .ok_or_else(|| Error::arg("spiflash: data byte is not a valid character"))?;
        char::from_u32(v as u32)
            .ok_or_else(|| Error::arg("spiflash: data byte is not a valid character"))
    }
}

/// An error raised inside a command handler. pd.py wraps the handler
/// call in `except KeyError`, so a `KeyError` becomes an "Unknown command"
/// annotation while any other exception ends the decoder.
enum Halt {
    Key,
    Err(Error),
}

impl From<Error> for Halt {
    fn from(e: Error) -> Self {
        Halt::Err(e)
    }
}

impl Halt {
    fn into_error(self) -> Error {
        match self {
            Halt::Key => Error::arg("spiflash: command state is not a known command"),
            Halt::Err(e) => e,
        }
    }
}

type HResult<T = ()> = std::result::Result<T, Halt>;

/// pd.py's `on_end_transaction` closure.
#[derive(Clone, Copy)]
enum EndAction {
    DataBlock { label: &'static str, idx: usize },
    Status,
}

/// The SPI flash decoder.
#[derive(Default)]
pub struct Spiflash {
    ss: u64,
    es: u64,
    ss_cmd: u64,
    // pd.py leaves es_cmd unset until a handler that needs it runs; a
    // putc() before that is an AttributeError there and an error here.
    es_cmd: Option<u64>,
    ss_field: u64,
    es_field: u64,
    state: Option<Int>,
    cmdstate: i64,
    addr: Int,
    data: Vec<Int>,
    writestate: u8,
    device_id: Option<Int>,
    chip_vendor: &'static str,
    vendor_key: &'static str,
    format_ascii: bool,
    on_end: Option<EndAction>,
    /// Set by REMS and used for its two ID bytes.
    manufacturer_id_first: bool,
    ids: Vec<Int>,
}

impl Spiflash {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        // pd.py: self.put(self.ss, self.es, self.out_ann, data)
        ctx.put_ann(self.ss, self.es, class, texts);
    }

    fn putf(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_field, self.es_field, class, texts);
    }

    fn putc(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) -> Result<()> {
        let es =
            self.es_cmd.ok_or_else(|| Error::arg("spiflash: command end sample is not set"))?;
        ctx.put_ann(self.ss_cmd, es, class, texts);
        Ok(())
    }

    fn cmd(&self) -> HResult<&'static Cmd> {
        let state = self.state.as_ref().ok_or(Halt::Key)?;
        CMDS.iter().find(|c| state.eq_u64(c.id)).ok_or(Halt::Key)
    }

    /// pd.py: `device()`, which raises `KeyError` for the `atmel` chips
    /// because `device_name` has no `atmel` entry.
    fn device(&self) -> HResult<&'static str> {
        let names =
            DEVICE_NAMES.iter().find(|(vendor, _)| *vendor == self.vendor_key).ok_or(Halt::Key)?;
        let Some(id) = &self.device_id else {
            return Ok("Unknown");
        };
        Ok(names.1.iter().find(|(key, _)| id.eq_u64(*key)).map_or("Unknown", |(_, n)| *n))
    }

    fn vendor_device(&self) -> HResult<String> {
        Ok(format!("{} {}", self.chip_vendor, self.device()?))
    }

    fn cmd_ann_list(&self) -> HResult<Vec<String>> {
        let cmd = self.cmd()?;
        Ok(vec![
            format!("Command: {} ({})", cmd.desc, cmd.short),
            format!("Command: {}", cmd.desc),
            format!("Cmd: {}", cmd.desc),
            format!("Cmd: {}", cmd.short),
            cmd.short.to_owned(),
        ])
    }

    fn cmd_vendor_dev_list(&self) -> HResult<Vec<String>> {
        let cmd = self.cmd()?;
        let d = format!("Device = {}", self.vendor_device()?);
        Ok(vec![
            format!("{} ({}): {}", cmd.desc, cmd.short, d),
            format!("{}: {}", cmd.desc, d),
            format!("{}: {}", cmd.short, d),
            d,
            self.vendor_device()?,
        ])
    }

    /// pd.py: `end_current_transaction`.
    fn end_current_transaction(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // pd.py clears the callback only after it returns.
        if let Some(action) = self.on_end {
            match action {
                EndAction::DataBlock { label, idx } => {
                    self.output_data_block(ctx, label, idx).map_err(Halt::into_error)?;
                }
                EndAction::Status => {
                    let text = self.cmd().map_err(Halt::into_error)?.desc.to_owned();
                    self.putc(ctx, ANN_STATUS, vec![text])?;
                }
            }
            self.on_end = None;
        }
        self.state = None;
        self.cmdstate = 1;
        self.addr = Int::default();
        self.data.clear();
        Ok(())
    }

    /// pd.py: `emit_cmd_byte`.
    fn emit_cmd_byte(&mut self, ctx: &mut dyn Context) -> HResult {
        self.ss_cmd = self.ss;
        let texts = self.cmd_ann_list()?;
        self.putx(ctx, ANN_FIELD, texts);
        self.addr = Int::default();
        Ok(())
    }

    /// pd.py: `emit_addr_bytes`.
    fn emit_addr_bytes(&mut self, ctx: &mut dyn Context, mosi: &Int) {
        // pd.py shifts by `(4 - self.cmdstate) * 8` bits, i.e. whole bytes.
        self.addr.or_shift(mosi, (4 - self.cmdstate) as usize);
        let b = ((3 - (self.cmdstate - 2)) * 8) - 1;
        let h = mosi.hex(2);
        self.putx(
            ctx,
            ANN_BIT,
            vec![
                format!("Address bits {b}..{}: 0x{h}", b - 7),
                format!("Addr bits {b}..{}: 0x{h}", b - 7),
                format!("Addr bits {b}..{}", b - 7),
                format!("A{b}..A{}", b - 7),
            ],
        );
        if self.cmdstate == 2 {
            self.ss_field = self.ss;
        }
        if self.cmdstate == 4 {
            self.es_field = self.es;
            let a = self.addr.hex(6);
            self.putf(
                ctx,
                ANN_FIELD,
                vec![format!("Address: 0x{a}"), format!("Addr: 0x{a}"), format!("0x{a}")],
            );
        }
    }

    /// pd.py: `handle_wren`.
    fn handle_wren(&mut self, ctx: &mut dyn Context) -> HResult {
        let texts = self.cmd_ann_list()?;
        self.putx(ctx, ANN_WREN, texts);
        self.writestate = 1;
        Ok(())
    }

    /// pd.py: `handle_wrdi`.
    fn handle_wrdi(&mut self, ctx: &mut dyn Context) -> HResult {
        let texts = self.cmd_ann_list()?;
        self.putx(ctx, ANN_WRDI, texts);
        self.writestate = 0;
        Ok(())
    }

    /// pd.py: `handle_rdid`.
    fn handle_rdid(&mut self, ctx: &mut dyn Context, miso: &Int) -> HResult {
        match self.cmdstate {
            1 => self.emit_cmd_byte(ctx)?,
            2 => self.putx(ctx, ANN_FIELD, vec![format!("Manufacturer ID: 0x{}", miso.hex(2))]),
            3 => self.putx(ctx, ANN_FIELD, vec![format!("Memory type: 0x{}", miso.hex(2))]),
            4 => {
                self.device_id = Some(miso.clone());
                self.putx(ctx, ANN_FIELD, vec![format!("Device ID: 0x{}", miso.hex(2))]);
            }
            _ => {}
        }
        if self.cmdstate == 4 {
            self.es_cmd = Some(self.es);
            let texts = self.cmd_vendor_dev_list()?;
            self.putc(ctx, ANN_RDID, texts)?;
            self.state = None;
        } else {
            self.cmdstate += 1;
        }
        Ok(())
    }

    /// pd.py: `handle_rdsr`.
    fn handle_rdsr(&mut self, ctx: &mut dyn Context, miso: &Int) -> HResult {
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
        } else if self.cmdstate >= 2 {
            self.es_cmd = Some(self.es);
            self.putx(ctx, ANN_BIT, vec![decode_status_reg(miso)]);
            self.putx(ctx, ANN_FIELD, vec!["Status register".to_owned()]);
            let texts = self.cmd_ann_list()?;
            self.putc(ctx, ANN_RDSR, texts)?;
            self.writestate = u8::from(miso.bit(1));
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// pd.py: `handle_rdsr2`.
    fn handle_rdsr2(&mut self, ctx: &mut dyn Context, miso: &Int) -> HResult {
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
        } else if self.cmdstate >= 2 {
            self.es_cmd = Some(self.es);
            self.putx(ctx, ANN_BIT, vec![decode_status_reg(miso)]);
            self.putx(ctx, ANN_FIELD, vec!["Status register 2".to_owned()]);
            let texts = self.cmd_ann_list()?;
            self.putc(ctx, ANN_RDSR2, texts)?;
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// pd.py: `handle_wrsr`.
    fn handle_wrsr(&mut self, ctx: &mut dyn Context, mosi: &Int, miso: &Int) -> HResult {
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
        } else if self.cmdstate == 2 {
            self.putx(ctx, ANN_BIT, vec![decode_status_reg(mosi)]);
            self.putx(ctx, ANN_FIELD, vec!["Status register 1".to_owned()]);
            self.writestate = u8::from(miso.bit(1));
        } else if self.cmdstate == 3 {
            self.putx(ctx, ANN_BIT, vec![decode_status_reg(mosi)]);
            self.putx(ctx, ANN_FIELD, vec!["Status register 2".to_owned()]);
            self.es_cmd = Some(self.es);
            let texts = self.cmd_ann_list()?;
            self.putc(ctx, ANN_WRSR, texts)?;
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// pd.py: `handle_read`.
    fn handle_read(&mut self, ctx: &mut dyn Context, mosi: &Int, miso: &Int) -> HResult {
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
        } else if (2..=4).contains(&self.cmdstate) {
            self.emit_addr_bytes(ctx, mosi);
        } else if self.cmdstate >= 5 {
            self.es_field = self.es;
            if self.cmdstate == 5 {
                self.ss_field = self.ss;
                self.on_end = Some(EndAction::DataBlock { label: "Data", idx: ANN_READ });
            }
            self.data.push(miso.clone());
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// pd.py: `handle_write_common`.
    fn handle_write_common(&mut self, ctx: &mut dyn Context, mosi: &Int, idx: usize) -> HResult {
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
            if self.writestate == 0 {
                // pd.py uses putc() here; es_cmd is only set by the
                // handlers that run later in a transaction.
                self.putc(ctx, ANN_WARN, vec!["Warning: WREN might be missing".to_owned()])?;
            }
        } else if (2..=4).contains(&self.cmdstate) {
            self.emit_addr_bytes(ctx, mosi);
        } else if self.cmdstate >= 5 {
            self.es_field = self.es;
            if self.cmdstate == 5 {
                self.ss_field = self.ss;
                self.on_end = Some(EndAction::DataBlock { label: "Data", idx });
            }
            self.data.push(mosi.clone());
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// pd.py: `handle_fast_read`.
    fn handle_fast_read(&mut self, ctx: &mut dyn Context, mosi: &Int, miso: &Int) -> HResult {
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
        } else if (2..=4).contains(&self.cmdstate) {
            self.emit_addr_bytes(ctx, mosi);
        } else if self.cmdstate == 5 {
            self.putx(ctx, ANN_BIT, vec![format!("Dummy byte: 0x{}", mosi.hex(2))]);
        } else if self.cmdstate >= 6 {
            self.es_field = self.es;
            if self.cmdstate == 6 {
                self.ss_field = self.ss;
                self.on_end = Some(EndAction::DataBlock { label: "Data", idx: ANN_FAST_READ });
            }
            self.data.push(miso.clone());
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// pd.py: `handle_2read`.
    fn handle_2read(&mut self, ctx: &mut dyn Context, mosi: &Int, miso: &Int) -> HResult {
        let dual = if self.cmdstate == 1 { None } else { Some(decode_dual_bytes(mosi, miso)?) };
        match self.cmdstate {
            1 => self.emit_cmd_byte(ctx)?,
            2 => {
                let (b1, b2) = dual.unwrap();
                self.emit_addr_bytes(ctx, &Int::from_u64(u64::from(b1)));
                self.cmdstate = 3;
                self.emit_addr_bytes(ctx, &Int::from_u64(u64::from(b2)));
            }
            4 => {
                let (b1, b2) = dual.unwrap();
                self.emit_addr_bytes(ctx, &Int::from_u64(u64::from(b1)));
                self.cmdstate = 5;
                self.putx(
                    ctx,
                    ANN_BIT,
                    vec![format!("Dummy byte: 0x{}", Int::from_u64(u64::from(b2)).hex(2))],
                );
            }
            _ if self.cmdstate >= 6 => {
                let (b1, b2) = dual.unwrap();
                self.es_field = self.es;
                if self.cmdstate == 6 {
                    self.ss_field = self.ss;
                    self.on_end = Some(EndAction::DataBlock { label: "Data", idx: ANN_READ2X });
                }
                self.data.push(Int::from_u64(u64::from(b1)));
                self.data.push(Int::from_u64(u64::from(b2)));
            }
            _ => {}
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// pd.py: `handle_status`.
    fn handle_status(&mut self, ctx: &mut dyn Context, miso: &Int) -> HResult {
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
            self.on_end = Some(EndAction::Status);
        } else {
            self.es_cmd = Some(self.es);
            self.es_field = self.es;
            if self.cmdstate == 2 {
                self.ss_field = self.ss;
            }
            self.putx(
                ctx,
                ANN_BIT,
                vec![format!(
                    "Status register byte {}: 0x{}",
                    (self.cmdstate % 2) + 1,
                    miso.hex(2)
                )],
            );
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// pd.py: `handle_se`.
    fn handle_se(&mut self, ctx: &mut dyn Context, mosi: &Int) -> HResult {
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
            if self.writestate == 0 {
                self.putx(ctx, ANN_WARN, vec!["Warning: WREN might be missing".to_owned()]);
            }
        } else if (2..=4).contains(&self.cmdstate) {
            self.emit_addr_bytes(ctx, mosi);
        }

        if self.cmdstate == 4 {
            self.es_cmd = Some(self.es);
            let d = format!("Erase sector {} (0x{})", self.addr.decimal(), self.addr.hex(6));
            self.putc(ctx, ANN_SE, vec![d])?;
            // Sector addresses must be 4K-aligned (same for all 3 chips).
            if self.addr.mod_u64(4096) != 0 {
                self.putc(ctx, ANN_WARN, vec!["Warning: Invalid sector address!".to_owned()])?;
            }
            self.state = None;
        } else {
            self.cmdstate += 1;
        }
        Ok(())
    }

    /// pd.py: `handle_ce` and `handle_ce2`.
    fn handle_ce(&mut self, ctx: &mut dyn Context, idx: usize) -> HResult {
        let texts = self.cmd_ann_list()?;
        self.putx(ctx, idx, texts);
        if self.writestate == 0 {
            self.putx(ctx, ANN_WARN, vec!["Warning: WREN might be missing".to_owned()]);
        }
        Ok(())
    }

    /// pd.py: `handle_rdp_res`.
    fn handle_rdp_res(&mut self, ctx: &mut dyn Context, mosi: &Int, miso: &Int) -> HResult {
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
        } else if (2..=4).contains(&self.cmdstate) {
            self.putx(ctx, ANN_FIELD, vec![format!("Dummy byte: {}", mosi.hex(2))]);
        } else if self.cmdstate == 5 {
            self.es_cmd = Some(self.es);
            self.device_id = Some(miso.clone());
            self.putx(ctx, ANN_FIELD, vec![format!("Device ID: {}", self.device()?)]);
            let texts = self.cmd_vendor_dev_list()?;
            self.putc(ctx, ANN_RDP_RES, texts)?;
            self.state = None;
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// pd.py: `handle_rems`.
    fn handle_rems(&mut self, ctx: &mut dyn Context, mosi: &Int, miso: &Int) -> HResult {
        match self.cmdstate {
            1 => self.emit_cmd_byte(ctx)?,
            2 | 3 => {
                self.putx(ctx, ANN_FIELD, vec![format!("Dummy byte: 0x{}", mosi.hex(2))]);
            }
            4 => {
                self.manufacturer_id_first = mosi.is_zero();
                let d = if self.manufacturer_id_first { "manufacturer" } else { "device" };
                self.putx(ctx, ANN_FIELD, vec![format!("Master wants {d} ID first")]);
            }
            5 => {
                self.ids = vec![miso.clone()];
                let d = if self.manufacturer_id_first { "Manufacturer" } else { "Device" };
                self.putx(ctx, ANN_FIELD, vec![format!("{d} ID: 0x{}", miso.hex(2))]);
            }
            6 => {
                self.ids.push(miso.clone());
                let d = if self.manufacturer_id_first { "Device" } else { "Manufacturer" };
                self.putx(ctx, ANN_FIELD, vec![format!("{d} ID: 0x{}", miso.hex(2))]);
            }
            _ => {}
        }
        if self.cmdstate == 6 {
            let id_ =
                if self.manufacturer_id_first { self.ids[1].clone() } else { self.ids[0].clone() };
            self.device_id = Some(id_);
            self.es_cmd = Some(self.es);
            let texts = self.cmd_vendor_dev_list()?;
            self.putc(ctx, ANN_REMS, texts)?;
            self.state = None;
        } else {
            self.cmdstate += 1;
        }
        Ok(())
    }

    /// pd.py: `handle_pp`.
    fn handle_pp(&mut self, ctx: &mut dyn Context, mosi: &Int, miso: &Int) -> HResult {
        let _ = miso;
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
        } else if (2..=4).contains(&self.cmdstate) {
            self.emit_addr_bytes(ctx, mosi);
        } else if self.cmdstate >= 5 {
            self.es_field = self.es;
            if self.cmdstate == 5 {
                self.ss_field = self.ss;
                self.on_end = Some(EndAction::DataBlock { label: "Data", idx: ANN_PP });
            }
            self.data.push(mosi.clone());
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// pd.py: `output_data_block`.
    fn output_data_block(&mut self, ctx: &mut dyn Context, label: &str, idx: usize) -> HResult {
        self.es_cmd = Some(self.es);
        let s = if self.format_ascii {
            let mut s = String::new();
            for b in &self.data {
                s.push(b.to_char()?);
            }
            s
        } else {
            self.data.iter().map(|b| b.hex(2)).collect::<Vec<_>>().join(" ")
        };
        self.putf(ctx, ANN_FIELD, vec![format!("{label} ({} bytes)", self.data.len())]);
        let cmd = self.cmd()?;
        let text =
            format!("{} (addr 0x{}, {} bytes): {}", cmd.desc, self.addr.hex(6), self.data.len(), s);
        self.putc(ctx, idx, vec![text])?;
        Ok(())
    }

    /// Dispatches one DATA byte to the handler `cmds` selects. An
    /// unknown command returns `Halt::Key`, as dict lookup does.
    fn dispatch(&mut self, ctx: &mut dyn Context, mosi: &Int, miso: &Int) -> HResult {
        let Some(state) = self.state.as_ref().and_then(Int::to_u64) else {
            return Err(Halt::Key);
        };
        match state {
            0x01 => self.handle_wrsr(ctx, mosi, miso),
            0x02 => self.handle_pp(ctx, mosi, miso),
            0x03 => self.handle_read(ctx, mosi, miso),
            0x04 => self.handle_wrdi(ctx),
            0x05 => self.handle_rdsr(ctx, miso),
            0x06 => self.handle_wren(ctx),
            0x0b => self.handle_fast_read(ctx, mosi, miso),
            0x20 => self.handle_se(ctx, mosi),
            // TODO in pd.py: no annotations at all.
            0x2b | 0x2f | 0x70 | 0x80 | 0xad | 0xb1 | 0xb9 | 0xc1 | 0xd8 | 0xef => Ok(()),
            0x35 => self.handle_rdsr2(ctx, miso),
            0x60 => self.handle_ce(ctx, ANN_CE),
            0x82 => self.handle_write_common(ctx, mosi, ANN_WRITE1),
            0x85 => self.handle_write_common(ctx, mosi, ANN_WRITE2),
            0x90 => self.handle_rems(ctx, mosi, miso),
            0x9f => self.handle_rdid(ctx, miso),
            0xab => self.handle_rdp_res(ctx, mosi, miso),
            0xbb => self.handle_2read(ctx, mosi, miso),
            0xc7 => self.handle_ce(ctx, ANN_CE2),
            0xd7 => self.handle_status(ctx, miso),
            _ => Err(Halt::Key),
        }
    }
}

/// pd.py: `decode_status_reg`.
fn decode_status_reg(data: &Int) -> String {
    // Bits[0:0]: WIP (write in progress)
    let s = if data.bit(0) { "W" } else { "No w" };
    let mut ret = format!("{s}rite operation in progress.\n");

    // Bits[1:1]: WEL (write enable latch)
    let s = if data.bit(1) { "" } else { "not " };
    let _ = writeln!(ret, "Internal write enable latch is {s}set.");

    // Bits[5:2]: Block protect bits
    let bp = (u64::from(data.bit(2))
        | (u64::from(data.bit(3)) << 1)
        | (u64::from(data.bit(4)) << 2)
        | (u64::from(data.bit(5)) << 3)) as u8;
    let _ = writeln!(ret, "Block protection bits (BP3-BP0): 0x{bp:x}.");

    // Bits[6:6]: Continuously program mode (CP mode)
    let s = if data.bit(6) { "" } else { "not " };
    let _ = writeln!(ret, "Device is {s}in continuously program mode (CP mode).");

    // Bits[7:7]: SRWD (status register write disable)
    let s = if data.bit(7) { "not " } else { "" };
    let _ = writeln!(ret, "Status register writes are {s}allowed.");

    ret
}

/// pd.py: `decode_dual_bytes`.
fn decode_dual_bytes(sio0: &Int, sio1: &Int) -> HResult<(u8, u8)> {
    fn combine_byte(even: u64, odd: u64) -> u8 {
        let mut result = 0u8;
        for bit in 0..4 {
            if even & (1 << bit) != 0 {
                result |= 1 << (bit * 2);
            }
            if odd & (1 << bit) != 0 {
                result |= 1 << ((bit * 2) + 1);
            }
        }
        result
    }
    let a = sio0.to_u64().ok_or_else(|| Error::arg("spiflash: dual I/O word too wide"))?;
    let b = sio1.to_u64().ok_or_else(|| Error::arg("spiflash: dual I/O word too wide"))?;
    Ok((combine_byte(a >> 4, b >> 4), combine_byte(a, b)))
}

/// Splits an item from the SPI decoder into `(ptype, mosi, miso)`, as
/// pd.py's `ptype, mosi, miso = data` does. A non-string packet type
/// compares unequal to every name in pd.py and is ignored.
fn split_item(item: &Item) -> Result<(Option<&str>, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("spiflash: item is not a list"));
    };
    let [ptype, mosi, miso] = parts.as_slice() else {
        return Err(Error::arg("spiflash: item is not a three element packet"));
    };
    Ok((
        match ptype {
            Item::Str(s) => Some(s.as_str()),
            _ => None,
        },
        mosi,
        miso,
    ))
}

impl Decoder for Spiflash {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let chip_id = config.str("chip", "adesto_at45db161e");
        let chip = CHIPS
            .iter()
            .find(|c| c.id == chip_id)
            .ok_or_else(|| Error::arg(format!("spiflash: unknown chip '{chip_id}'")))?;
        // pd.py: self.vendor = self.options['chip'].split('_')[0]
        let vendor_key = chip.id.split('_').next().unwrap_or_default();
        *self = Spiflash {
            chip_vendor: chip.vendor,
            vendor_key,
            format_ascii: config.str("format", "hex") == "ascii",
            ..Spiflash::default()
        };
        self.reset();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let _ = ctx;
        Err(Error::not_supported("spiflash is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, mosi, miso) = split_item(item)?;

        self.ss = ss;
        self.es = es;

        if ptype == Some("CS-CHANGE") {
            self.end_current_transaction(ctx)?;
        }

        if ptype != Some("DATA") {
            return Ok(());
        }

        // If we encountered a known chip command, enter the resp. state.
        let mosi = Int::from_item(mosi)?;
        let miso = Int::from_item(miso)?;
        if self.state.is_none() {
            self.state = Some(mosi.clone());
            self.cmdstate = 1;
        }

        // Handle commands. pd.py's try/except KeyError also catches the
        // KeyError from device() for the atmel chips.
        match self.dispatch(ctx, &mosi, &miso) {
            Ok(()) => Ok(()),
            Err(Halt::Key) => {
                let text = format!("Unknown command: 0x{}", mosi.hex(2));
                self.putx(ctx, ANN_BIT, vec![text]);
                self.state = None;
                Ok(())
            }
            Err(Halt::Err(e)) => Err(e),
        }
    }
}

impl Spiflash {
    /// pd.py's `reset()`: `end_current_transaction()` plus the write
    /// latch state.
    fn reset(&mut self) {
        self.device_id = None;
        self.on_end = None;
        self.state = None;
        self.cmdstate = 1;
        self.addr = Int::default();
        self.data.clear();
        self.writestate = 0;
        self.manufacturer_id_first = false;
        self.ids.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::{Int, decode_status_reg};

    #[test]
    fn status_register_text() {
        assert_eq!(
            decode_status_reg(&Int::from_u64(0x00)),
            "No write operation in progress.\n\
             Internal write enable latch is not set.\n\
             Block protection bits (BP3-BP0): 0x0.\n\
             Device is not in continuously program mode (CP mode).\n\
             Status register writes are allowed.\n"
        );
        assert_eq!(
            decode_status_reg(&Int::from_u64(0xff)),
            "Write operation in progress.\n\
             Internal write enable latch is set.\n\
             Block protection bits (BP3-BP0): 0xf.\n\
             Device is in continuously program mode (CP mode).\n\
             Status register writes are not allowed.\n"
        );
    }

    #[test]
    fn hex_is_pythons_percent_02x() {
        assert_eq!(Int::from_u64(0).hex(2), "00");
        assert_eq!(Int::from_u64(0x1f).hex(2), "1f");
        assert_eq!(Int::from_u64(0x001f_2600).hex(6), "1f2600");
    }
}
