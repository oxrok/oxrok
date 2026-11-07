// Counterpart in sigrok:
//   libsigrokdecode/decoders/enc28j60/pd.py and lists.py,
//   Copyright (C) 2019 Jiahao Li <reg@ljh.me>,
//   MIT.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the ENC28J60 SPI command set and register banks and
//   the decoder's documented classes and texts.
#![allow(clippy::identity_op)]

//! ENC28J60: Microchip 10Base-T Ethernet controller.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

const OPCODE_MASK: u64 = 0b1110_0000;
const REG_ADDR_MASK: u64 = 0b0001_1111;

// pd.py's annotation class indices.
const ANN_RCR: usize = 0;
const ANN_RBM: usize = 1;
const ANN_WCR: usize = 2;
const ANN_WBM: usize = 3;
const ANN_BFS: usize = 4;
const ANN_BFC: usize = 5;
const ANN_SRC: usize = 6;
const ANN_DATA: usize = 7;
const ANN_REG_ADDR: usize = 8;
const ANN_WARNING: usize = 9;

const REG_ADDR_ECON1: u64 = 0x1F;
const BIT_ECON1_BSEL0: u64 = 0b0000_0001;
const BIT_ECON1_BSEL1: u64 = 0b0000_0010;

// pd.py's REGS table. Note the em dash entries and that pd.py compares
// a register name with an ASCII '-' for the invalid-register warning,
// so em dash entries do not warn.
const BANK0: [&str; 32] = [
    "ERDPTL", "ERDPTH", "EWRPTL", "EWRPTH", "ETXSTL", "ETXSTH", "ETXNDL", "ETXNDH", "ERXSTL",
    "ERXSTH", "ERXNDL", "ERXNDH", "ERXRDPTL", "ERXRDPTH", "ERXWRPTL", "ERXWRPTH", "EDMASTL",
    "EDMASTH", "EDMANDL", "EDMANDH", "EDMADSTL", "EDMADSTH", "EDMACSL", "EDMACSH", "—", "—",
    "Reserved", "EIE", "EIR", "ESTAT", "ECON2", "ECON1",
];

const BANK1: [&str; 32] = [
    "EHT0", "EHT1", "EHT2", "EHT3", "EHT4", "EHT5", "EHT6", "EHT7", "EPMM0", "EPMM1", "EPMM2",
    "EPMM3", "EPMM4", "EPMM5", "EPMM6", "EPMM7", "EPMCSL", "EPMCSH", "—", "—", "EPMOL", "EPMOH",
    "Reserved", "Reserved", "ERXFCON", "EPKTCNT", "Reserved", "EIE", "EIR", "ESTAT", "ECON2",
    "ECON1",
];

const BANK2: [&str; 32] = [
    "MACON1", "Reserved", "MACON3", "MACON4", "MABBIPG", "—", "MAIPGL", "MAIPGH", "MACLCON1",
    "MACLCON2", "MAMXFLL", "MAMXFLH", "Reserved", "Reserved", "Reserved", "—", "Reserved",
    "Reserved", "MICMD", "—", "MIREGADR", "Reserved", "MIWRL", "MIWRH", "MIRDL", "MIRDH",
    "Reserved", "EIE", "EIR", "ESTAT", "ECON2", "ECON1",
];

const BANK3: [&str; 32] = [
    "MAADR5", "MAADR6", "MAADR3", "MAADR4", "MAADR1", "MAADR2", "EBSTSD", "EBSTCON", "EBSTCSL",
    "EBSTCSH", "MISTAT", "—", "—", "—", "—", "—", "—", "—", "EREVID", "—", "—", "ECOCON",
    "Reserved", "EFLOCON", "EPAUSL", "EPAUSH", "Reserved", "EIE", "EIR", "ESTAT", "ECON2", "ECON1",
];

const REGS: [&[&str; 32]; 4] = [&BANK0, &BANK1, &BANK2, &BANK3];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "enc28j60",
    name: "ENC28J60",
    longname: "Microchip ENC28J60",
    desc: "Microchip ENC28J60 10Base-T Ethernet controller protocol.",
    license: "mit",
    inputs: &["spi"],
    outputs: &[],
    tags: &["Embedded/industrial", "Networking"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "rcr", desc: "Read Control Register" },
        AnnClass { id: "rbm", desc: "Read Buffer Memory" },
        AnnClass { id: "wcr", desc: "Write Control Register" },
        AnnClass { id: "wbm", desc: "Write Buffer Memory" },
        AnnClass { id: "bfs", desc: "Bit Field Set" },
        AnnClass { id: "bfc", desc: "Bit Field Clear" },
        AnnClass { id: "src", desc: "System Reset Command" },
        AnnClass { id: "data", desc: "Data" },
        AnnClass { id: "reg-addr", desc: "Register Address" },
        AnnClass { id: "warning", desc: "Warning" },
    ],
    annotation_rows: &[
        AnnRow { id: "fields", desc: "Fields", classes: &[ANN_DATA, ANN_REG_ADDR] },
        AnnRow {
            id: "commands",
            desc: "Commands",
            classes: &[ANN_RCR, ANN_RBM, ANN_WCR, ANN_WBM, ANN_BFS, ANN_BFC, ANN_SRC],
        },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[ANN_WARNING] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes the protocol spoken\nby the Microchip ENC28J60 Ethernet chip.\n\nDetails:\nhttp://ww1.microchip.com/downloads/en/DeviceDoc/39662e.pdf",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Enc28j60::default())
}

/// A non-negative Python integer from an SPI DATA item.
fn value(item: &Item) -> Result<u64> {
    match item {
        Item::Int(v) if *v >= 0 => Ok(*v as u64),
        Item::BigUint(bytes) => {
            let mut v = 0u64;
            for &b in bytes {
                v = v << 8 | u64::from(b);
            }
            Ok(v)
        }
        _ => Err(Error::arg(format!("enc28j60: not an SPI word: {item}"))),
    }
}

/// The ENC28J60 decoder.
#[derive(Default)]
pub struct Enc28j60 {
    mosi: Vec<u64>,
    miso: Vec<u64>,
    ranges: Vec<(u64, u64)>,
    cmd_ss: u64,
    cmd_es: u64,
    range_ss: u64,
    range_es: u64,
    active: bool,
    bsel0: Option<u64>,
    bsel1: Option<u64>,
}

impl Enc28j60 {
    fn reset(&mut self) {
        *self = Enc28j60::default();
    }

    fn putc(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.cmd_ss, self.cmd_es, class, texts);
    }

    fn putr(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.range_ss, self.range_es, class, texts);
    }

    fn put_command_warning(&self, ctx: &mut dyn Context, reason: &str) {
        self.putc(ctx, ANN_WARNING, vec![format!("Warning: {reason}"), "Warning".to_owned()]);
    }

    fn get_register_name(&self, reg_addr: u64) -> Option<&'static str> {
        match (self.bsel0, self.bsel1) {
            // We don't know the bank we're in yet.
            (Some(bsel0), Some(bsel1)) => {
                let bank = ((bsel1 << 1) + bsel0) as usize;
                Some(REGS[bank][reg_addr as usize])
            }
            _ => None,
        }
    }

    fn put_register_header(&mut self, ctx: &mut dyn Context) {
        let reg_addr = self.mosi[0] & REG_ADDR_MASK;
        let reg_name = self.get_register_name(reg_addr);

        self.range_ss = self.cmd_ss;
        self.range_es = self.ranges[1].0;

        match reg_name {
            None => {
                // We don't know the bank we're in yet.
                self.putr(
                    ctx,
                    ANN_REG_ADDR,
                    vec![format!("Reg Bank ? Addr 0x{reg_addr:02X}"), format!("?:{reg_addr:02X}")],
                );
                self.putr(
                    ctx,
                    ANN_WARNING,
                    vec!["Warning: Register bank not known yet.".to_owned(), "Warning".to_owned()],
                );
            }
            Some(reg_name) => {
                self.putr(ctx, ANN_REG_ADDR, vec![format!("Reg {reg_name}"), reg_name.to_owned()]);
                // pd.py tests for an ASCII '-', while its table uses an
                // em dash for those entries, so only 'Reserved' matches.
                if reg_name == "-" || reg_name == "Reserved" {
                    self.putr(
                        ctx,
                        ANN_WARNING,
                        vec![
                            "Warning: Invalid register accessed.".to_owned(),
                            "Warning".to_owned(),
                        ],
                    );
                }
            }
        }
    }

    fn put_data_byte(&mut self, ctx: &mut dyn Context, data: u64, byte_index: usize, binary: bool) {
        self.range_ss = self.ranges[byte_index].0;
        if byte_index == self.mosi.len() - 1 {
            self.range_es = self.cmd_es;
        } else {
            self.range_es = self.ranges[byte_index + 1].0;
        }

        if binary {
            self.putr(ctx, ANN_DATA, vec![format!("Data 0b{data:08b}"), format!("{data:08b}")]);
        } else {
            self.putr(ctx, ANN_DATA, vec![format!("Data 0x{data:02X}"), format!("{data:02X}")]);
        }
    }

    fn process_rcr(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, ANN_RCR, vec!["Read Control Register".to_owned(), "RCR".to_owned()]);

        if self.mosi.len() != 2 && self.mosi.len() != 3 {
            self.put_command_warning(ctx, "Invalid command length.");
            return;
        }

        self.put_register_header(ctx);

        let reg_name = self.get_register_name(self.mosi[0] & REG_ADDR_MASK);
        if let Some(reg_name) = reg_name {
            if reg_name.starts_with('M') && self.mosi.len() != 3 {
                self.put_command_warning(
                    ctx,
                    "Attempting to read a MAC/MII register without using the dummy byte.",
                );
                return;
            }
            if !reg_name.starts_with('M') && self.mosi.len() != 2 {
                self.put_command_warning(
                    ctx,
                    "Attempting to read a non-MAC/MII register using the dummy byte.",
                );
                return;
            }
        }

        if self.mosi.len() == 2 {
            self.put_data_byte(ctx, self.miso[1], 1, false);
        } else {
            self.range_ss = self.ranges[1].0;
            self.range_es = self.ranges[2].0;
            self.putr(ctx, ANN_DATA, vec!["Dummy Byte".to_owned(), "Dummy".to_owned()]);
            self.put_data_byte(ctx, self.miso[2], 2, false);
        }
    }

    fn process_rbm(&mut self, ctx: &mut dyn Context) {
        if self.mosi[0] != 0b0011_1010 {
            self.put_command_warning(ctx, "Invalid header byte.");
            return;
        }

        self.putc(
            ctx,
            ANN_RBM,
            vec![format!("Read Buffer Memory: Length {}", self.mosi.len() - 1), "RBM".to_owned()],
        );

        for i in 1..self.miso.len() {
            self.put_data_byte(ctx, self.miso[i], i, false);
        }
    }

    fn process_wcr(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, ANN_WCR, vec!["Write Control Register".to_owned(), "WCR".to_owned()]);

        if self.mosi.len() != 2 {
            self.put_command_warning(ctx, "Invalid command length.");
            return;
        }

        self.put_register_header(ctx);
        self.put_data_byte(ctx, self.mosi[1], 1, false);

        if self.mosi[0] & REG_ADDR_MASK == REG_ADDR_ECON1 {
            self.bsel0 = Some((self.mosi[1] & BIT_ECON1_BSEL0) >> 0);
            self.bsel1 = Some((self.mosi[1] & BIT_ECON1_BSEL1) >> 1);
        }
    }

    fn process_wbm(&mut self, ctx: &mut dyn Context) {
        if self.mosi[0] != 0b0111_1010 {
            self.put_command_warning(ctx, "Invalid header byte.");
            return;
        }

        self.putc(
            ctx,
            ANN_WBM,
            vec![format!("Write Buffer Memory: Length {}", self.mosi.len() - 1), "WBM".to_owned()],
        );

        for i in 1..self.mosi.len() {
            self.put_data_byte(ctx, self.mosi[i], i, false);
        }
    }

    fn process_bfc(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, ANN_BFC, vec!["Bit Field Clear".to_owned(), "BFC".to_owned()]);

        if self.mosi.len() != 2 {
            self.put_command_warning(ctx, "Invalid command length.");
            return;
        }

        self.put_register_header(ctx);
        self.put_data_byte(ctx, self.mosi[1], 1, true);

        if self.mosi[0] & REG_ADDR_MASK == REG_ADDR_ECON1 {
            if self.mosi[1] & BIT_ECON1_BSEL0 != 0 {
                self.bsel0 = Some(0);
            }
            if self.mosi[1] & BIT_ECON1_BSEL1 != 0 {
                self.bsel1 = Some(0);
            }
        }
    }

    fn process_bfs(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, ANN_BFS, vec!["Bit Field Set".to_owned(), "BFS".to_owned()]);

        if self.mosi.len() != 2 {
            self.put_command_warning(ctx, "Invalid command length.");
            return;
        }

        self.put_register_header(ctx);
        self.put_data_byte(ctx, self.mosi[1], 1, true);

        if self.mosi[0] & REG_ADDR_MASK == REG_ADDR_ECON1 {
            if self.mosi[1] & BIT_ECON1_BSEL0 != 0 {
                self.bsel0 = Some(1);
            }
            if self.mosi[1] & BIT_ECON1_BSEL1 != 0 {
                self.bsel1 = Some(1);
            }
        }
    }

    fn process_src(&mut self, ctx: &mut dyn Context) {
        self.putc(ctx, ANN_SRC, vec!["System Reset Command".to_owned(), "SRC".to_owned()]);

        if self.mosi.len() != 1 {
            self.put_command_warning(ctx, "Invalid command length.");
            return;
        }

        self.bsel0 = Some(0);
        self.bsel1 = Some(0);
    }

    fn process_command(&mut self, ctx: &mut dyn Context) {
        if self.mosi.is_empty() {
            self.active = false;
            return;
        }

        let header = self.mosi[0];
        let opcode = header & OPCODE_MASK;

        match opcode {
            0b0000_0000 => self.process_rcr(ctx),
            0b0010_0000 => self.process_rbm(ctx),
            0b0100_0000 => self.process_wcr(ctx),
            0b0110_0000 => self.process_wbm(ctx),
            0b1000_0000 => self.process_bfs(ctx),
            0b1010_0000 => self.process_bfc(ctx),
            0b1110_0000 => self.process_src(ctx),
            _ => self.put_command_warning(ctx, "Unknown opcode."),
        }

        self.active = false;
    }
}

/// Splits an item from the SPI decoder into `(ptype, data1, data2)`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("enc28j60: item is not a list"));
    };
    let [ptype, data1, data2] = parts.as_slice() else {
        return Err(Error::arg("enc28j60: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("enc28j60: packet type is not a string"));
    };
    Ok((ptype.as_str(), data1, data2))
}

impl Decoder for Enc28j60 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let _ = config;
        self.reset();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let _ = ctx;
        Err(Error::not_supported("enc28j60 is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, data1, data2) = split_item(item)?;

        if ptype == "CS-CHANGE" {
            let new_cs = data2;

            if matches!(new_cs, Item::Int(0)) {
                self.active = true;
                self.cmd_ss = ss;
                self.mosi.clear();
                self.miso.clear();
                self.ranges.clear();
            } else if matches!(new_cs, Item::Int(1)) && self.active {
                self.cmd_es = es;
                self.process_command(ctx);
            }
        } else if ptype == "DATA" {
            let mosi = value(data1)?;
            let miso = value(data2)?;

            self.mosi.push(mosi);
            self.miso.push(miso);
            self.ranges.push((ss, es));
        }
        Ok(())
    }
}
