// Counterpart in sigrok:
//   libsigrokdecode/decoders/avr_isp/pd.py and parts.py,
//   Copyright (C) 2012-2014 Uwe Hermann <uwe@hermann-uwe.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the AVR ISP protocol and the decoder's documented
//   classes and texts.

//! AVR ISP: Atmel AVR in-system programming commands on an SPI bus.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

const VENDOR_CODE_ATMEL: u64 = 0x1e;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_PROGRAMMING_ENABLE: usize = 0;
const ANN_RBS0: usize = 1;
const ANN_RBS1: usize = 2;
const ANN_RBS2: usize = 3;
const ANN_CHIP_ERASE: usize = 4;
const ANN_RFB: usize = 5;
const ANN_RHFB: usize = 6;
const ANN_REFB: usize = 7;
const ANN_WARN: usize = 8;
const ANN_DEV: usize = 9;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "avr_isp",
    name: "AVR ISP",
    longname: "AVR In-System Programming",
    desc: "Atmel AVR In-System Programming (ISP) protocol.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["Debug/trace"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "pe", desc: "Programming enable" },
        AnnClass { id: "rsb0", desc: "Read signature byte 0" },
        AnnClass { id: "rsb1", desc: "Read signature byte 1" },
        AnnClass { id: "rsb2", desc: "Read signature byte 2" },
        AnnClass { id: "ce", desc: "Chip erase" },
        AnnClass { id: "rfb", desc: "Read fuse bits" },
        AnnClass { id: "rhfb", desc: "Read high fuse bits" },
        AnnClass { id: "refb", desc: "Read extended fuse bits" },
        AnnClass { id: "warnings", desc: "Warnings" },
        AnnClass { id: "dev", desc: "Device" },
    ],
    annotation_rows: &[
        // pd.py: ('bits', 'Bits', ()) -- no annotation classes.
        AnnRow { id: "bits", desc: "Bits", classes: &[] },
        AnnRow { id: "commands", desc: "Commands", classes: &[0, 1, 2, 3, 4, 5, 6, 7] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[8] },
        AnnRow { id: "dev", desc: "Device", classes: &[9] },
    ],
    binary: &[],
    // pd.py has no module docstring, so the Python decoder's doc is "".
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(AvrIsp::default())
}

use std::fmt::Write as _;

/// A non-negative Python integer, as the SPI decoder sends them in its
/// `DATA` items. Python has no width limit; a word may be wider than
/// 64 bits, so this keeps a little-endian magnitude.
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

    /// The value of an SPI `DATA` item. pd.py works with plain Python
    /// ints, which the SPI decoder produces through its word value.
    fn from_item(item: &Item) -> Result<Self> {
        match item {
            Item::Int(v) if *v >= 0 => Ok(Int::from_u64(*v as u64)),
            Item::BigUint(bytes) => Ok(Int::from_be_bytes(bytes)),
            _ => Err(Error::arg(format!("avr_isp: not an SPI word: {item}"))),
        }
    }

    fn bit(&self, bit: u32) -> bool {
        let byte = self.0.get((bit / 8) as usize).copied().unwrap_or(0);
        byte & (1 << (bit % 8)) != 0
    }

    fn eq_u64(&self, v: u64) -> bool {
        *self == Int::from_u64(v)
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
}

/// A decoded byte pair: what pd.py keeps in `self.mosi_bytes` etc.
type Bytes = Vec<Int>;

fn warn_unexpected() -> String {
    "Warning: Unexpected bytes in reply!".to_owned()
}

/// `vendor_code` from parts.py.
fn vendor_name(code: &Int) -> Option<&'static str> {
    if code.eq_u64(0x1e) {
        Some("Atmel")
    } else if code.eq_u64(0x00) {
        Some("Device locked")
    } else {
        None
    }
}

/// `part` from parts.py, keyed by (part family + flash size, part number).
fn part_name(family: &Int, number: &Int) -> Option<&'static str> {
    const PARTS: &[(u64, u64, &str)] = &[
        (0x90, 0x01, "AT90S1200"),
        (0x91, 0x01, "AT90S2313"),
        (0x92, 0x01, "AT90S4414"),
        (0x92, 0x05, "ATmega48"),
        (0x93, 0x01, "AT90S8515"),
        (0x93, 0x0a, "ATmega88"),
        (0x94, 0x06, "ATmega168"),
        (0xff, 0xff, "Device code erased, or target missing"),
        (0x01, 0x02, "Device locked"),
    ];
    PARTS.iter().find(|(f, n, _)| family.eq_u64(*f) && number.eq_u64(*n)).map(|(_, _, name)| *name)
}

/// The AVR ISP decoder.
#[derive(Default)]
pub struct AvrIsp {
    mosi_bytes: Bytes,
    miso_bytes: Bytes,
    ss_cmd: u64,
    es_cmd: u64,
    xx: Int,
    yy: Int,
    // pd.py stores ret[0] in `zz` but never uses it.
    mm: Int,
    ss_device: Option<u64>,
    vendor_code: Int,
    part_fam_flash_size: Int,
    part_number: Int,
}

impl AvrIsp {
    fn putx(&self, ctx: &mut dyn Context, class: usize, text: String) {
        ctx.put_ann(self.ss_cmd, self.es_cmd, class, vec![text]);
    }

    /// pd.py: `handle_cmd_programming_enable`.
    fn handle_programming_enable(&mut self, ctx: &mut dyn Context, cmd: &[Int], ret: &[Int]) {
        self.putx(ctx, ANN_PROGRAMMING_ENABLE, "Programming enable".to_owned());
        // Note: The chip doesn't send any ACK for 'Programming enable'.
        if !(ret[1].eq_u64(0xac) && ret[2].eq_u64(0x53) && ret[3] == cmd[2]) {
            self.putx(ctx, ANN_WARN, warn_unexpected());
        }
    }

    /// pd.py: `handle_cmd_read_signature_byte_0x00`.
    fn handle_signature_byte_0(
        &mut self,
        ctx: &mut dyn Context,
        cmd: &[Int],
        ret: &[Int],
    ) -> Result<()> {
        // Signature byte 0x00: vendor code.
        self.vendor_code = ret[3].clone();
        // pd.py does `v = vendor_code[self.vendor_code]`, which raises
        // KeyError for any other value and ends the decoder.
        let v = vendor_name(&self.vendor_code).ok_or_else(|| {
            Error::arg(format!("avr_isp: unknown vendor code {}", self.vendor_code.hex(2)))
        })?;
        self.putx(ctx, ANN_RBS0, format!("Vendor code: 0x{} ({})", ret[3].hex(2), v));

        // Store for later.
        self.xx = cmd[1].clone(); // Same as ret[2].
        self.yy = cmd[3].clone();

        // Sanity check on reply.
        if !(ret[1].eq_u64(0x30) && ret[2] == cmd[1]) {
            self.putx(ctx, ANN_WARN, warn_unexpected());
        }

        // Sanity check for the vendor code.
        if !self.vendor_code.eq_u64(VENDOR_CODE_ATMEL) {
            self.putx(ctx, ANN_WARN, "Warning: Vendor code was not 0x1e (Atmel)!".to_owned());
        }
        Ok(())
    }

    /// pd.py: `handle_cmd_read_signature_byte_0x01`.
    fn handle_signature_byte_1(&mut self, ctx: &mut dyn Context, cmd: &[Int], ret: &[Int]) {
        // Signature byte 0x01: part family and memory size.
        self.part_fam_flash_size = ret[3].clone();
        self.putx(ctx, ANN_RBS1, format!("Part family / memory size: 0x{}", ret[3].hex(2)));

        // Store for later.
        self.mm = cmd[3].clone();
        self.ss_device = Some(self.ss_cmd);

        // Sanity check on reply.
        if !(ret[1].eq_u64(0x30) && ret[2] == cmd[1] && ret[0] == self.yy) {
            self.putx(ctx, ANN_WARN, warn_unexpected());
        }
    }

    /// pd.py: `handle_cmd_read_signature_byte_0x02`.
    fn handle_signature_byte_2(
        &mut self,
        ctx: &mut dyn Context,
        _cmd: &[Int],
        ret: &[Int],
    ) -> Result<()> {
        // Signature byte 0x02: part number.
        self.part_number = ret[3].clone();
        self.putx(ctx, ANN_RBS2, format!("Part number: 0x{}", ret[3].hex(2)));

        let p = part_name(&self.part_fam_flash_size, &self.part_number)
            .ok_or_else(|| Error::arg("avr_isp: unknown part family / number combination"))?;
        let ss_device =
            self.ss_device.ok_or_else(|| Error::arg("avr_isp: part number without part family"))?;
        ctx.put_ann(ss_device, self.es_cmd, ANN_DEV, vec![format!("Device: Atmel {p}")]);

        // Sanity check on reply.
        if !(ret[1].eq_u64(0x30) && ret[2] == self.xx && ret[0] == self.mm) {
            self.putx(ctx, ANN_WARN, warn_unexpected());
        }

        self.xx = Int::default();
        self.yy = Int::default();
        self.mm = Int::default();
        Ok(())
    }

    /// pd.py: `handle_cmd_chip_erase`.
    fn handle_chip_erase(&mut self, ctx: &mut dyn Context, cmd: &[Int], ret: &[Int]) {
        self.putx(ctx, ANN_CHIP_ERASE, "Chip erase".to_owned());

        // Sanity check on reply.
        let bit = ret[2].bit(7);
        if !(ret[1].eq_u64(0xac) && bit && ret[3] == cmd[2]) {
            self.putx(ctx, ANN_WARN, warn_unexpected());
        }
    }

    fn put_reply_byte(&mut self, ctx: &mut dyn Context, class: usize, prefix: &str, ret: &[Int]) {
        self.putx(ctx, class, format!("{prefix}: 0x{}", ret[3].hex(2)));
    }

    /// pd.py: `handle_command`.
    fn handle_command(&mut self, ctx: &mut dyn Context, cmd: &[Int], ret: &[Int]) -> Result<()> {
        if cmd[0].eq_u64(0xac) && cmd[1].eq_u64(0x53) {
            self.handle_programming_enable(ctx, cmd, ret);
        } else if cmd[0].eq_u64(0xac) && cmd[1].bit(7) {
            self.handle_chip_erase(ctx, cmd, ret);
        } else if cmd[0].eq_u64(0x50) && cmd[1].eq_u64(0x00) && cmd[2].eq_u64(0x00) {
            self.put_reply_byte(ctx, ANN_RFB, "Read fuse bits", ret);
        } else if cmd[0].eq_u64(0x58) && cmd[1].eq_u64(0x08) && cmd[2].eq_u64(0x00) {
            self.put_reply_byte(ctx, ANN_RHFB, "Read fuse high bits", ret);
        } else if cmd[0].eq_u64(0x50) && cmd[1].eq_u64(0x08) && cmd[2].eq_u64(0x00) {
            self.put_reply_byte(ctx, ANN_REFB, "Read extended fuse bits", ret);
        } else if cmd[0].eq_u64(0x30) && cmd[2].eq_u64(0x00) {
            self.handle_signature_byte_0(ctx, cmd, ret)?;
        } else if cmd[0].eq_u64(0x30) && cmd[2].eq_u64(0x01) {
            self.handle_signature_byte_1(ctx, cmd, ret);
        } else if cmd[0].eq_u64(0x30) && cmd[2].eq_u64(0x02) {
            self.handle_signature_byte_2(ctx, cmd, ret)?;
        } else {
            let c = cmd.iter().map(|v| v.hex(2)).collect::<Vec<_>>().join(" ");
            let r = ret.iter().map(|v| v.hex(2)).collect::<Vec<_>>().join(" ");
            self.putx(ctx, ANN_PROGRAMMING_ENABLE, format!("Unknown command: {c} (reply: {r})!"));
        }
        Ok(())
    }
}

/// Splits an item from the SPI decoder into `(ptype, mosi, miso)`, as
/// pd.py's `ptype, mosi, miso = data` does. A non-string packet type
/// compares unequal to every name in pd.py and is ignored.
fn split_item(item: &Item) -> Result<(Option<&str>, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("avr_isp: item is not a list"));
    };
    let [ptype, mosi, miso] = parts.as_slice() else {
        return Err(Error::arg("avr_isp: item is not a three element packet"));
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

impl Decoder for AvrIsp {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let _ = config;
        *self = AvrIsp::default();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let _ = ctx;
        Err(Error::not_supported("avr_isp is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, mosi, miso) = split_item(item)?;

        // For now, only use DATA and BITS packets.
        if ptype != Some("DATA") && ptype != Some("BITS") {
            return Ok(());
        }

        // pd.py stores the bit values for a BITS packet and returns; it
        // never uses them again in this decoder.
        if ptype == Some("BITS") {
            return Ok(());
        }

        self.ss_cmd = if self.mosi_bytes.is_empty() { ss } else { self.ss_cmd };

        // Append new bytes.
        self.mosi_bytes.push(Int::from_item(mosi)?);
        self.miso_bytes.push(Int::from_item(miso)?);

        // All commands consist of 4 bytes.
        if self.mosi_bytes.len() < 4 {
            return Ok(());
        }

        self.es_cmd = es;

        let result = self.handle_command(ctx, &self.mosi_bytes.clone(), &self.miso_bytes.clone());
        // pd.py clears the byte lists only after handle_command returns;
        // a KeyError there leaves them in place, as this does.
        result?;
        self.mosi_bytes.clear();
        self.miso_bytes.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::Int;

    #[test]
    fn hex_is_pythons_percent_02x() {
        assert_eq!(Int::from_u64(0).hex(2), "00");
        assert_eq!(Int::from_u64(0xa).hex(2), "0a");
        assert_eq!(Int::from_u64(0x1e).hex(2), "1e");
        assert_eq!(Int::from_u64(0x1234).hex(2), "1234");
        assert_eq!(Int::from_u64(0x1234).hex(6), "001234");
    }
}
