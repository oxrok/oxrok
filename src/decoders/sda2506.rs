// Counterpart in sigrok:
//   libsigrokdecode/decoders/sda2506/pd.py,
//   Copyright (C) 2018 Max Weller,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the serial EEPROM protocol and the decoder's
//   documented classes and texts.
#![allow(clippy::struct_field_names)]

//! Siemens SDA 2506-5: decodes the command and data bytes of the
//! 1-Kbit serial EEPROM's clocked interface.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

const CLK: usize = 0;
const DATA: usize = 1;
const CE: usize = 2;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_CMDBIT: usize = 0;
const ANN_DATABIT: usize = 1;
const ANN_CMD: usize = 2;
const ANN_DATA: usize = 3;
#[allow(dead_code)]
const ANN_WARNING: usize = 4;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "sda2506",
    name: "SDA2506",
    longname: "Siemens SDA 2506-5",
    desc: "Serial nonvolatile 1-Kbit EEPROM.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["IC", "Memory"],
    channels: &[
        ChannelDesc { id: "clk", name: "CLK", desc: "Clock" },
        ChannelDesc { id: "d", name: "DATA", desc: "Data" },
        ChannelDesc { id: "ce", name: "CE#", desc: "Chip-enable" },
    ],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "cmdbit", desc: "Command bit" },
        AnnClass { id: "databit", desc: "Data bit" },
        AnnClass { id: "cmd", desc: "Command" },
        AnnClass { id: "data", desc: "Data byte" },
        AnnClass { id: "warnings", desc: "Human-readable warnings" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0, 1] },
        AnnRow { id: "data", desc: "Data", classes: &[3] },
        AnnRow { id: "commands", desc: "Commands", classes: &[2] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[4] },
    ],
    binary: &[],
    doc: "Decoder for Siemens EEPROM SDA 2506-5.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Sda2506::default())
}

/// One bit of the command shift register: its level, and the sample
/// range the decoder saw it in.
#[derive(Clone, Copy)]
struct Bit {
    bit: bool,
    ss: u64,
    es: u64,
}

/// The SDA 2506 decoder.
#[derive(Default)]
pub struct Sda2506 {
    samplerate: u64,
    cmdbits: Vec<Bit>,
    databits: Vec<bool>,
    datastart: u64,
}

impl Sda2506 {
    /// The Python `reset()`: forget the shifted-in bits.
    fn reset(&mut self) {
        self.cmdbits.clear();
        self.databits.clear();
    }

    /// `decode_bits` from the Python decoder: the value of `width` bits
    /// starting at `offset` in the (newest-first) command bits, with the
    /// sample range the decoder reports for the field.
    ///
    /// `self.cmdbits[offset + width - 1][1]` is the start of the last
    /// bit and `self.cmdbits[offset][2]` the end of the first, so the
    /// range runs backwards, exactly as the Python decoder builds it.
    fn decode_bits(&self, offset: usize, width: usize) -> Option<(u32, u64, u64)> {
        if offset + width > self.cmdbits.len() || width == 0 {
            return None;
        }
        let mut out = 0u32;
        for i in 0..width {
            out = (out << 1) | u32::from(self.cmdbits[offset + i].bit);
        }
        Some((out, self.cmdbits[offset + width - 1].ss, self.cmdbits[offset].es))
    }

    /// `decode_field` from the Python decoder: an annotation with the
    /// field name and value.
    fn decode_field(&self, ctx: &mut dyn Context, name: &str, offset: usize, width: usize) -> bool {
        let Some((val, ss, es)) = self.decode_bits(offset, width) else {
            return false;
        };
        ctx.put_ann(ss, es, ANN_DATA, vec![format!("{name}: {val:02X}")]);
        true
    }

    /// `putdata` from the Python decoder: the eight collected data bits
    /// as a byte, oldest bit first because the list is prepended to.
    fn putdata(&self, ctx: &mut dyn Context, ss: u64, es: u64) {
        let mut value = 0u32;
        for bit in &self.databits[..8] {
            value = (value << 1) | u32::from(*bit);
        }
        ctx.put_ann(ss, es, ANN_DATA, vec![format!("{value:02X}")]);
    }
}

impl Decoder for Sda2506 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.reset();
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            // "Wait for CLK edge or CE edge."
            let pins =
                ctx.wait(&[&[Cond::pin(CLK, CondKind::Edge)], &[Cond::pin(CE, CondKind::Edge)]])?;
            let matched = ctx.matched().to_vec();
            let clk = pins.get(CLK);
            let d = pins.get(DATA);
            let ce = pins.get(CE);

            if matched[0] && ce && clk {
                // "Rising clk edge and command mode."
                let bitstart = ctx.samplenum();
                ctx.wait(&[&[Cond::pin(CLK, CondKind::Falling)]])?;
                let now = ctx.samplenum();
                self.cmdbits.insert(0, Bit { bit: d, ss: bitstart, es: now });
                self.cmdbits.truncate(24);
                ctx.put_ann(bitstart, now, ANN_CMDBIT, vec![u8::from(d).to_string()]);
            } else if matched[0] && !ce && !clk {
                // "Falling clk edge and data mode." "Wait 25 us for data
                // ready." The Python skip count is
                // int(2.5 * (1e6 / self.samplerate)).
                if self.samplerate == 0 {
                    return Err(Error::arg("float division by zero"));
                }
                let bitstart = ctx.samplenum();
                let skip = (2.5 * (1e6 / self.samplerate as f64)) as u64;
                let pins = ctx.wait(&[
                    &[Cond::Skip(skip)],
                    &[Cond::pin(CLK, CondKind::Rising)],
                    &[Cond::pin(CE, CondKind::Edge)],
                ])?;
                let d = pins.get(DATA);
                if matches!(ctx.matched(), [true, false, false]) {
                    // Upstream calls wait() without assigning its result,
                    // so the data bit stays the one seen at the skip.
                    ctx.wait(&[
                        &[Cond::pin(CLK, CondKind::Rising)],
                        &[Cond::pin(CE, CondKind::Edge)],
                    ])?;
                }
                if self.databits.is_empty() {
                    self.datastart = bitstart;
                }
                self.databits.insert(0, d);
                ctx.put_ann(bitstart, ctx.samplenum(), ANN_DATABIT, vec![u8::from(d).to_string()]);
                if self.databits.len() == 8 {
                    let now = ctx.samplenum();
                    self.putdata(ctx, self.datastart, now);
                    self.databits.clear();
                }
            } else if matched[1] && !ce {
                // "Chip enable edge." The Python decoder wraps the whole
                // field decoding in `except Exception: self.reset()`, so
                // incomplete command bits just drop the command.
                if !self.handle_ce_edge(ctx, d) {
                    self.reset();
                }
            }
        }
    }
}

impl Sda2506 {
    /// The chip-enable falling edge handler. `false` means the Python
    /// decoder's `except Exception` path, which resets its bit lists.
    fn handle_ce_edge(&mut self, ctx: &mut dyn Context, d: bool) -> bool {
        // `decode_field('addr', 1, 7)` and `decode_field('CB', 0, 1)`.
        if !self.decode_field(ctx, "addr", 1, 7) {
            return false;
        }
        if !self.decode_field(ctx, "CB", 0, 1) {
            return false;
        }
        if !self.cmdbits.is_empty() && !self.cmdbits[0].bit {
            // Beginning read command.
            if !self.decode_field(ctx, "read", 1, 7) {
                return false;
            }
            if self.cmdbits.len() < 8 {
                return false;
            }
            let ss = self.cmdbits[7].ss;
            let es = ctx.samplenum();
            ctx.put_ann(ss, es, ANN_CMD, vec!["read".to_owned()]);
        } else if !d {
            // Beginning write command.
            if !self.decode_field(ctx, "data", 8, 8) {
                return false;
            }
            let Some((addr, _, _)) = self.decode_bits(1, 7) else { return false };
            let Some((data, _, _)) = self.decode_bits(8, 8) else { return false };
            let cmdstart = ctx.samplenum();
            if ctx.wait(&[&[Cond::pin(CE, CondKind::Rising)]]).is_err() {
                return false;
            }
            let es = ctx.samplenum();
            ctx.put_ann(cmdstart, es, ANN_CMD, vec![format!("Write to {addr:02X}: {data:02X}")]);
        } else {
            // Beginning erase command.
            let Some((val, _, _)) = self.decode_bits(1, 7) else { return false };
            let cmdstart = ctx.samplenum();
            if ctx.wait(&[&[Cond::pin(CE, CondKind::Rising)]]).is_err() {
                return false;
            }
            let es = ctx.samplenum();
            ctx.put_ann(cmdstart, es, ANN_CMD, vec![format!("Erase: {val:02X}")]);
        }
        self.databits.clear();
        true
    }
}
