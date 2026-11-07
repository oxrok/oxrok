// Counterpart in sigrok:
//   libsigrokdecode/decoders/mrf24j40/pd.py and lists.py,
//   Copyright (C) 2015 Karl Palsson <karlp@tweak.net.au>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the MRF24J40 register map and the decoder's
//   documented classes and texts.

//! Microchip MRF24J40 IEEE 802.15.4 2.4 GHz transceiver commands and
//! frames on an SPI bus.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_SREAD: usize = 0;
const ANN_SWRITE: usize = 1;
const ANN_LREAD: usize = 2;
const ANN_LWRITE: usize = 3;
const ANN_WARNING: usize = 4;
const ANN_TX_FRAME: usize = 5;
const ANN_RX_FRAME: usize = 6;

// The two frame directions, as pd.py's RX, TX = range(2).
const RX: usize = 0;
const TX: usize = 1;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "mrf24j40",
    name: "MRF24J40",
    longname: "Microchip MRF24J40",
    desc: "IEEE 802.15.4 2.4 GHz RF tranceiver chip.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["IC", "Wireless/RF"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "sread", desc: "Short register read commands" },
        AnnClass { id: "swrite", desc: "Short register write commands" },
        AnnClass { id: "lread", desc: "Long register read commands" },
        AnnClass { id: "lwrite", desc: "Long register write commands" },
        AnnClass { id: "warning", desc: "Warnings" },
        AnnClass { id: "tx-frame", desc: "TX frame" },
        AnnClass { id: "rx-frame", desc: "RX frame" },
        AnnClass { id: "tx-retry-1", desc: "1x TX retry" },
        AnnClass { id: "tx-retry-2", desc: "2x TX retry" },
        AnnClass { id: "tx-retry-3", desc: "3x TX retry" },
        AnnClass { id: "tx-fail", desc: "TX fail (too many retries)" },
        AnnClass { id: "ccafail", desc: "CCAFAIL (channel busy)" },
    ],
    annotation_rows: &[
        AnnRow { id: "read", desc: "Read", classes: &[0, 2] },
        AnnRow { id: "write", desc: "Write", classes: &[1, 3] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[4] },
        AnnRow { id: "tx-frames", desc: "TX frames", classes: &[5] },
        AnnRow { id: "rx-frames", desc: "RX frames", classes: &[6] },
        AnnRow { id: "tx-retries-1", desc: "1x TX retries", classes: &[7] },
        AnnRow { id: "tx-retries-2", desc: "2x TX retries", classes: &[8] },
        AnnRow { id: "tx-retries-3", desc: "3x TX retries", classes: &[9] },
        AnnRow { id: "tx-fails", desc: "TX fails", classes: &[10] },
        AnnRow { id: "ccafails", desc: "CCAFAILs", classes: &[11] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes Microchip MRF24J40\n\
          IEEE 802.15.4 2.4 GHz RF tranceiver commands and data.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Mrf24j40::default())
}

/// pd.py's `sregs`, a short register name by address; unknown addresses
/// are `illegal`.
#[allow(clippy::match_same_arms)]
fn sreg(reg: i64) -> &'static str {
    match reg {
        0x00 => "RXMCR",
        0x01 => "PANIDL",
        0x02 => "PANIDH",
        0x03 => "SADRL",
        0x04 => "SADRH",
        0x05 => "EADR0",
        0x06 => "EADR1",
        0x07 => "EADR2",
        0x08 => "EADR3",
        0x09 => "EADR4",
        0x0a => "EADR5",
        0x0b => "EADR6",
        0x0c => "EADR7",
        0x0d => "RXFLUSH",
        0x0e | 0x0f => "Reserved",
        0x10 => "ORDER",
        0x11 => "TXMCR",
        0x12 => "ACKTMOUT",
        0x13 => "ESLOTG1",
        0x14 => "SYMTICKL",
        0x15 => "SYMTICKH",
        0x16 => "PACON0",
        0x17 => "PACON1",
        0x18 => "PACON2",
        0x19 => "Reserved",
        0x1a => "TXBCON0",
        0x1b => "TXNCON",
        0x1c => "TXG1CON",
        0x1d => "TXG2CON",
        0x1e => "ESLOTG23",
        0x1f => "ESLOTG45",
        0x20 => "ESLOTG67",
        0x21 => "TXPEND",
        0x22 => "WAKECON",
        0x23 => "FRMOFFSET",
        0x24 => "TXSTAT",
        0x25 => "TXBCON1",
        0x26 => "GATECLK",
        0x27 => "TXTIME",
        0x28 => "HSYMTIMRL",
        0x29 => "HSYMTIMRH",
        0x2a => "SOFTRST",
        0x2b => "Reserved",
        0x2c => "SECCON0",
        0x2d => "SECCON1",
        0x2e => "TXSTBL",
        0x2f => "Reserved",
        0x30 => "RXSR",
        0x31 => "INTSTAT",
        0x32 => "INTCON",
        0x33 => "GPIO",
        0x34 => "TRISGPIO",
        0x35 => "SLPACK",
        0x36 => "RFCTL",
        0x37 => "SECCR2",
        0x38 => "BBREG0",
        0x39 => "BBREG1",
        0x3a => "BBREG2",
        0x3b => "BBREG3",
        0x3c => "BBREG4",
        0x3d => "Reserved",
        0x3e => "BBREG6",
        0x3f => "CCAEDTH",
        _ => "illegal",
    }
}

/// pd.py's `lregs`, a long register name by address; unknown addresses
/// are `illegal`.
#[allow(clippy::match_same_arms)]
fn lreg(reg: i64) -> &'static str {
    match reg {
        0x200 => "RFCON0",
        0x201 => "RFCON1",
        0x202 => "RFCON2",
        0x203 => "RFCON3",
        0x204 => "Reserved",
        0x205 => "RFCON5",
        0x206 => "RFCON6",
        0x207 => "RFCON7",
        0x208 => "RFCON8",
        0x209 => "SLPCAL0",
        0x20A => "SLPCAL1",
        0x20B => "SLPCAL2",
        0x20C..=0x20E => "Reserved",
        0x20F => "RFSTATE",
        0x210 => "RSSI",
        0x211 => "SLPCON0",
        0x212..=0x21F => "Reserved",
        0x220 => "SLPCON1",
        0x221 => "Reserved",
        0x222 => "WAKETIMEL",
        0x223 => "WAKETIMEH",
        0x224 => "REMCNTL",
        0x225 => "REMCNTH",
        0x226 => "MAINCNT0",
        0x227 => "MAINCNT1",
        0x228 => "MAINCNT2",
        0x229 => "MAINCNT3",
        0x22A..=0x22E => "Reserved",
        0x22F => "TESTMODE",
        0x230 => "ASSOEADR0",
        0x231 => "ASSOEADR1",
        0x232 => "ASSOEADR2",
        0x233 => "ASSOEADR3",
        0x234 => "ASSOEADR4",
        0x235 => "ASSOEADR5",
        0x236 => "ASSOEADR6",
        0x237 => "ASSOEADR7",
        0x238 => "ASSOSADR0",
        0x239 => "ASSOSADR1",
        0x23A | 0x23B => "Reserved",
        0x23C..=0x23F => "Unimplemented",
        0x240 => "UPNONCE0",
        0x241 => "UPNONCE1",
        0x242 => "UPNONCE2",
        0x243 => "UPNONCE3",
        0x244 => "UPNONCE4",
        0x245 => "UPNONCE5",
        0x246 => "UPNONCE6",
        0x247 => "UPNONCE7",
        0x248 => "UPNONCE8",
        0x249 => "UPNONCE9",
        0x24A => "UPNONCE10",
        0x24B => "UPNONCE11",
        0x24C => "UPNONCE12",
        _ => "illegal",
    }
}

/// The MRF24J40 decoder.
#[derive(Default)]
pub struct Mrf24j40 {
    ss_cmd: u64,
    es_cmd: u64,
    ss_frame: [u64; 2],
    es_frame: [u64; 2],
    mosi_bytes: Vec<i64>,
    miso_bytes: Vec<i64>,
    /// pd.py's `framecache`, indexed by RX/TX.
    framecache: [Vec<i64>; 2],
}

impl Mrf24j40 {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_cmd, self.es_cmd, class, texts);
    }

    fn putw(ctx: &mut dyn Context, ss: u64, es: u64, msg: &str) {
        ctx.put_ann(ss, es, ANN_WARNING, vec![msg.to_owned()]);
    }

    fn reset_data(&mut self) {
        self.mosi_bytes.clear();
        self.miso_bytes.clear();
    }

    fn handle_short(&mut self, ctx: &mut dyn Context) {
        let write = self.mosi_bytes[0] & 0x1 != 0;
        let reg = (self.mosi_bytes[0] >> 1) & 0x3f;
        let reg_desc = sreg(reg);
        for rxtx in [RX, TX] {
            if self.framecache[rxtx].is_empty() {
                continue;
            }
            let bit0 = self.mosi_bytes[1] & (1 << 0);
            if rxtx == TX && !(reg_desc == "TXNCON" && bit0 == 1) {
                continue;
            }
            if rxtx == RX && !(reg_desc == "RXFLUSH" && bit0 == 1) {
                continue;
            }
            let idx = if rxtx == TX { ANN_TX_FRAME } else { ANN_RX_FRAME };
            let xmitdir = if rxtx == TX { "TX" } else { "RX" };
            let frame = self.framecache[rxtx]
                .iter()
                .map(|b| format!("{b:02X}"))
                .collect::<Vec<_>>()
                .join(" ");
            ctx.put_ann(
                self.ss_frame[rxtx],
                self.es_frame[rxtx],
                idx,
                vec![format!("{xmitdir} frame: {frame}")],
            );
            self.framecache[rxtx].clear();
        }
        if write {
            self.putx(ctx, ANN_SWRITE, vec![format!("{reg_desc}: {:#x}", self.mosi_bytes[1])]);
        } else {
            self.putx(ctx, ANN_SREAD, vec![format!("{reg_desc}: {:#x}", self.miso_bytes[1])]);
            let numretries = (self.miso_bytes[1] & 0xc0) >> 6;
            if reg_desc == "TXSTAT" && numretries > 0 {
                let txfail = i64::from((self.miso_bytes[1] & (1 << 0)) != 0);
                let idx = (6 + numretries + txfail) as usize;
                if txfail != 0 {
                    self.putx(ctx, idx, vec!["TX fail (>= 4 retries)".into(), "TX fail".into()]);
                } else {
                    self.putx(ctx, idx, vec![format!("TX retries: {numretries}")]);
                }
            }
            if reg_desc == "TXSTAT" && (self.miso_bytes[1] & (1 << 5)) != 0 {
                self.putx(ctx, 11, vec!["CCAFAIL (channel busy)".into(), "CCAFAIL".into()]);
            }
        }
    }

    fn handle_long(&mut self, ctx: &mut dyn Context) {
        let dword = (self.mosi_bytes[0] << 8) | self.mosi_bytes[1];
        let write = (dword & (0x1 << 4)) != 0;
        let reg = (dword >> 5) & 0x3ff;
        // pd.py applies these in order, so the later range wins.
        let mut reg_desc = format!("TX:{reg:#x}");
        if reg >= 0x80 {
            reg_desc = format!("TX beacon:{reg:#x}");
        }
        if reg >= 0x100 {
            reg_desc = format!("TX GTS1:{reg:#x}");
        }
        if reg >= 0x180 {
            reg_desc = format!("TX GTS2:{reg:#x}");
        }
        if reg >= 0x200 {
            lreg(reg).clone_into(&mut reg_desc);
        }
        if reg >= 0x280 {
            reg_desc = format!("Security keys:{reg:#x}");
        }
        if reg >= 0x2c0 {
            reg_desc = format!("Reserved:{reg:#x}");
        }
        if reg >= 0x300 {
            reg_desc = format!("RX:{reg:#x}");
        }

        if write {
            self.putx(ctx, ANN_LWRITE, vec![format!("{reg_desc}: {:#x}", self.mosi_bytes[2])]);
        } else {
            self.putx(ctx, ANN_LREAD, vec![format!("{reg_desc}: {:#x}", self.miso_bytes[2])]);
        }

        for rxtx in [RX, TX] {
            if rxtx == RX && !reg_desc.starts_with("RX:") {
                continue;
            }
            if rxtx == TX && !reg_desc.starts_with("TX:") {
                continue;
            }
            if self.framecache[rxtx].is_empty() {
                self.ss_frame[rxtx] = self.ss_cmd;
            }
            self.es_frame[rxtx] = self.es_cmd;
            let value = if rxtx == TX { self.mosi_bytes[2] } else { self.miso_bytes[2] };
            self.framecache[rxtx].push(value);
        }
    }
}

/// `ptype, mosi, miso = data`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("mrf24j40: item is not a list"));
    };
    let [ptype, mosi, miso] = parts.as_slice() else {
        return Err(Error::arg("mrf24j40: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("mrf24j40: packet type is not a string"));
    };
    Ok((ptype.as_str(), mosi, miso))
}

fn int(item: &Item) -> Result<i64> {
    match item {
        Item::Int(v) => Ok(*v),
        _ => Err(Error::arg(format!("mrf24j40: not a word: {item}"))),
    }
}

impl Decoder for Mrf24j40 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Mrf24j40::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("mrf24j40 is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, mosi, miso) = split_item(item)?;

        if ptype == "CS-CHANGE" {
            // If we transition high mid-stream, toss out our data and
            // restart.
            let cs_old = mosi;
            let cs_new = miso;
            let old_is_zero = !matches!(cs_old, Item::None) && int(cs_old)? == 0;
            let new_is_one = !matches!(cs_new, Item::None) && int(cs_new)? == 1;
            if old_is_zero && new_is_one && !matches!(self.mosi_bytes.len(), 0 | 2 | 3) {
                Self::putw(ctx, self.ss_cmd, es, "Misplaced CS!");
                self.reset_data();
            }
            return Ok(());
        }

        // Don't care about anything else.
        if ptype != "DATA" {
            return Ok(());
        }

        if self.mosi_bytes.is_empty() {
            self.ss_cmd = ss;
        }
        self.mosi_bytes.push(int(mosi)?);
        self.miso_bytes.push(int(miso)?);

        // Everything is either 2 bytes or 3 bytes.
        if self.mosi_bytes.len() < 2 {
            return Ok(());
        }

        if self.mosi_bytes[0] & 0x80 != 0 {
            if self.mosi_bytes.len() == 3 {
                self.es_cmd = es;
                self.handle_long(ctx);
                self.reset_data();
            }
        } else {
            self.es_cmd = es;
            self.handle_short(ctx);
            self.reset_data();
        }
        Ok(())
    }
}
