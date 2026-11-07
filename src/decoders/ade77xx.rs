// Counterpart in sigrok:
//   libsigrokdecode/decoders/ade77xx/pd.py and lists.py,
//   Copyright (C) 2017 Karl Palsson <karlp@etactica.com>,
//   MIT.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the ADE7758 register map (datasheet rev E) and the
//   decoder's documented classes and texts.

//! ADE77xx: Analog Devices poly phase energy metering ICs.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

// pd.py's annotation class indices.
const ANN_READ: usize = 0;
const ANN_WRITE: usize = 1;
const ANN_WARN: usize = 2;

/// One entry of lists.py's `regs` OrderedDict. pd.py only observes the
/// name (`rblob[0]`) and the bit width (`rblob[3]`).
struct Reg {
    addr: u64,
    name: &'static str,
    width: u64,
}

/// `regs` from lists.py, in order. Note that there is no register 0:
/// a command byte with that address is an unknown register.
const REGS: &[Reg] = &[
    Reg { addr: 0x1, name: "AWATTHR", width: 16 },
    Reg { addr: 0x2, name: "BWATTHR", width: 16 },
    Reg { addr: 0x3, name: "CWATTHR", width: 16 },
    Reg { addr: 0x4, name: "AVARHR", width: 16 },
    Reg { addr: 0x5, name: "BVARHR", width: 16 },
    Reg { addr: 0x6, name: "CVARHR", width: 16 },
    Reg { addr: 0x7, name: "AVAHR", width: 16 },
    Reg { addr: 0x8, name: "BVAHR", width: 16 },
    Reg { addr: 0x9, name: "CVAHR", width: 16 },
    Reg { addr: 0xa, name: "AIRMS", width: 24 },
    Reg { addr: 0xb, name: "BIRMS", width: 24 },
    Reg { addr: 0xc, name: "CIRMS", width: 24 },
    Reg { addr: 0xd, name: "AVRMS", width: 24 },
    Reg { addr: 0xe, name: "BVRMS", width: 24 },
    Reg { addr: 0xf, name: "CVRMS", width: 24 },
    Reg { addr: 0x10, name: "FREQ", width: 12 },
    Reg { addr: 0x11, name: "TEMP", width: 8 },
    Reg { addr: 0x12, name: "WFORM", width: 24 },
    Reg { addr: 0x13, name: "OPMODE", width: 8 },
    Reg { addr: 0x14, name: "MMODE", width: 8 },
    Reg { addr: 0x15, name: "WAVMODE", width: 8 },
    Reg { addr: 0x16, name: "COMPMODE", width: 8 },
    Reg { addr: 0x17, name: "LCYCMODE", width: 8 },
    Reg { addr: 0x18, name: "Mask", width: 24 },
    Reg { addr: 0x19, name: "Status", width: 24 },
    Reg { addr: 0x1a, name: "RSTATUS", width: 24 },
    Reg { addr: 0x1b, name: "ZXTOUT", width: 16 },
    Reg { addr: 0x1c, name: "LINECYC", width: 16 },
    Reg { addr: 0x1d, name: "SAGCYC", width: 8 },
    Reg { addr: 0x1e, name: "SAGLVL", width: 8 },
    Reg { addr: 0x1f, name: "VPINTLVL", width: 8 },
    Reg { addr: 0x20, name: "IPINTLVL", width: 8 },
    Reg { addr: 0x21, name: "VPEAK", width: 8 },
    Reg { addr: 0x22, name: "IPEAK", width: 8 },
    Reg { addr: 0x23, name: "Gain", width: 8 },
    Reg { addr: 0x24, name: "AVRMSGAIN", width: 12 },
    Reg { addr: 0x25, name: "BVRMSGAIN", width: 12 },
    Reg { addr: 0x26, name: "CVRMSGAIN", width: 12 },
    Reg { addr: 0x27, name: "AIGAIN", width: 12 },
    Reg { addr: 0x28, name: "BIGAIN", width: 12 },
    Reg { addr: 0x29, name: "CIGAIN", width: 12 },
    Reg { addr: 0x2a, name: "AWG", width: 12 },
    Reg { addr: 0x2b, name: "BWG", width: 12 },
    Reg { addr: 0x2c, name: "CWG", width: 12 },
    Reg { addr: 0x2d, name: "AVARG", width: 12 },
    Reg { addr: 0x2e, name: "BVARG", width: 12 },
    Reg { addr: 0x2f, name: "CVARG", width: 12 },
    Reg { addr: 0x30, name: "AVAG", width: 12 },
    Reg { addr: 0x31, name: "BVAG", width: 12 },
    Reg { addr: 0x32, name: "CVAG", width: 12 },
    Reg { addr: 0x33, name: "AVRMSOS", width: 12 },
    Reg { addr: 0x34, name: "BVRMSOS", width: 12 },
    Reg { addr: 0x35, name: "CVRMSOS", width: 12 },
    Reg { addr: 0x36, name: "AIRMSOS", width: 12 },
    Reg { addr: 0x37, name: "BIRMSOS", width: 12 },
    Reg { addr: 0x38, name: "CIRMSOS", width: 12 },
    Reg { addr: 0x39, name: "AWATTOS", width: 12 },
    Reg { addr: 0x3a, name: "BWATTOS", width: 12 },
    Reg { addr: 0x3b, name: "CWATTOS", width: 12 },
    Reg { addr: 0x3c, name: "AVAROS", width: 12 },
    Reg { addr: 0x3d, name: "BVAROS", width: 12 },
    Reg { addr: 0x3e, name: "CVAROS", width: 12 },
    Reg { addr: 0x3f, name: "APHCAL", width: 7 },
    Reg { addr: 0x40, name: "BPHCAL", width: 7 },
    Reg { addr: 0x41, name: "CPHCAL", width: 7 },
    Reg { addr: 0x42, name: "WDIV", width: 8 },
    Reg { addr: 0x43, name: "VARDIV", width: 8 },
    Reg { addr: 0x44, name: "VADIV", width: 8 },
    Reg { addr: 0x45, name: "APCFNUM", width: 16 },
    Reg { addr: 0x46, name: "APCFDEN", width: 12 },
    Reg { addr: 0x47, name: "VARCFNUM", width: 16 },
    Reg { addr: 0x48, name: "VARCFDEN", width: 12 },
    Reg { addr: 0x7e, name: "CHKSUM", width: 8 },
    Reg { addr: 0x7f, name: "Version", width: 8 },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ade77xx",
    name: "ADE77xx",
    longname: "Analog Devices ADE77xx",
    desc: "Poly phase multifunction energy metering IC protocol.",
    license: "mit",
    inputs: &["spi"],
    outputs: &[],
    tags: &["Analog/digital", "IC", "Sensor"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "read", desc: "Register read commands" },
        AnnClass { id: "write", desc: "Register write commands" },
        AnnClass { id: "warning", desc: "Warnings" },
    ],
    annotation_rows: &[
        AnnRow { id: "read", desc: "Read", classes: &[ANN_READ] },
        AnnRow { id: "write", desc: "Write", classes: &[ANN_WRITE] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[ANN_WARN] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes Analog Devices\nADE77xx command/responses.\n\nThe ADE77xx is a \"Poly Phase Multifunction Energy Metering IC with Per Phase\nInformation\".\n\nThis PD has been tested with an ADE7758 so far, support for other devices\nfrom the ADE77xx series can be added in the future.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Ade77xx::default())
}

/// A non-negative Python integer, as the SPI decoder sends them in its
/// `DATA` items.
#[derive(Clone, Copy, Default)]
struct Int(u64);

impl Int {
    fn from_item(item: &Item) -> Result<Self> {
        match item {
            Item::Int(v) if *v >= 0 => Ok(Int(*v as u64)),
            Item::BigUint(bytes) => {
                if bytes.iter().take(bytes.len().saturating_sub(8)).any(|&b| b != 0) {
                    return Err(Error::arg("ade77xx: SPI word too wide"));
                }
                let mut v = 0u64;
                for &b in bytes {
                    v = v << 8 | u64::from(b);
                }
                Ok(Int(v))
            }
            _ => Err(Error::arg(format!("ade77xx: not an SPI word: {item}"))),
        }
    }
}

fn reg(addr: u64) -> Option<&'static Reg> {
    REGS.iter().find(|r| r.addr == addr)
}

/// The ADE77xx decoder.
#[derive(Default)]
pub struct Ade77xx {
    ss_cmd: u64,
    es_cmd: u64,
    expected: u64,
    mosi_bytes: Vec<u64>,
    miso_bytes: Vec<u64>,
    cmd: Option<u64>,
}

impl Ade77xx {
    fn reset_data(&mut self) {
        self.expected = 0;
        self.mosi_bytes.clear();
        self.miso_bytes.clear();
    }

    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_cmd, self.es_cmd, class, texts);
    }

    fn put_warn(&self, ctx: &mut dyn Context, ss: u64, es: u64, msg: &str) {
        ctx.put_ann(ss, es, ANN_WARN, vec![msg.to_owned()]);
    }

    /// pd.py's CS-CHANGE branch.
    fn cs_change(&mut self, es: u64, old: &Item, new: &Item, ctx: &mut dyn Context) -> Result<()> {
        let old_zero = matches!(old, Item::Int(0));
        let new_one = matches!(new, Item::Int(1));
        if !(old_zero && new_one) {
            return Ok(());
        }
        if !self.mosi_bytes.is_empty() && (self.mosi_bytes.len() as u64 - 1) < self.expected {
            self.es_cmd = es;
            let cmd = self
                .cmd
                .ok_or_else(|| Error::arg("ade77xx: short transfer before any command byte"))?;
            let write = cmd & 0x80 != 0;
            let reg_addr = cmd & 0x7f;
            let rblob = reg(reg_addr)
                .ok_or_else(|| Error::arg("ade77xx: short transfer of an unknown register"))?;
            let idx = if write { ANN_WRITE } else { ANN_READ };
            self.putx(ctx, idx, vec![format!("{}: SHORT", rblob.name)]);
            self.put_warn(ctx, self.ss_cmd, es, "Short transfer!");
        }
        self.reset_data();
        Ok(())
    }
}

/// Splits an item from the SPI decoder into `(ptype, data1, data2)`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("ade77xx: item is not a list"));
    };
    let [ptype, data1, data2] = parts.as_slice() else {
        return Err(Error::arg("ade77xx: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("ade77xx: packet type is not a string"));
    };
    Ok((ptype.as_str(), data1, data2))
}

impl Decoder for Ade77xx {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let _ = config;
        *self = Ade77xx::default();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let _ = ctx;
        Err(Error::not_supported("ade77xx is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, data1, data2) = split_item(item)?;

        if ptype == "CS-CHANGE" {
            // Bear in mind, that CS is optional according to the
            // datasheet. If we transition high mid-stream, toss out our
            // data and restart.
            return self.cs_change(es, data1, data2, ctx);
        }

        // Don't care about anything else.
        if ptype != "DATA" {
            return Ok(());
        }

        let mosi = Int::from_item(data1)?;
        let miso = Int::from_item(data2)?;

        if self.mosi_bytes.is_empty() {
            self.ss_cmd = ss;
        }
        self.mosi_bytes.push(mosi.0);
        self.miso_bytes.push(miso.0);

        // A transfer is 2-4 bytes, (command + 1..3 byte reg).
        if self.mosi_bytes.len() < 2 {
            return Ok(());
        }

        let cmd = self.mosi_bytes[0];
        self.cmd = Some(cmd);
        let write = cmd & 0x80 != 0;
        let reg_addr = cmd & 0x7f;
        let Some(rblob) = reg(reg_addr) else {
            // If you don't have CS, this will _destroy_ comms!
            self.put_warn(ctx, self.ss_cmd, es, "Unknown register!");
            return Ok(());
        };

        // math.ceil(rblob[3] / 8)
        self.expected = rblob.width.div_ceil(8);
        if self.mosi_bytes.len() as u64 - 1 != self.expected {
            return Ok(());
        }

        let val = |bytes: &[u64]| -> u64 {
            match self.expected {
                3 => bytes[1] << 16 | bytes[2] << 8 | bytes[3],
                2 => bytes[1] << 8 | bytes[2],
                _ => bytes[1],
            }
        };
        let valo = val(&self.mosi_bytes);
        let vali = val(&self.miso_bytes);
        self.es_cmd = es;

        if write {
            self.putx(ctx, ANN_WRITE, vec![format!("{}: {:#x}", rblob.name, valo)]);
        } else {
            self.putx(ctx, ANN_READ, vec![format!("{}: {:#x}", rblob.name, vali)]);
        }

        self.reset_data();
        Ok(())
    }
}
