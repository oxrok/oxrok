// Counterpart in sigrok:
//   libsigrokdecode/decoders/st7735/pd.py,
//   Copyright (C) 2018 Aleksander Alekseev <afiskon@gmail.com>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the ST7735 command set and the decoder's
//   documented classes and texts.
#![allow(clippy::similar_names, clippy::unused_self)]

//! Sitronix ST7735 TFT controller: decodes the 8-bit command and data
//! bytes of its 4-wire serial interface.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

const CS: usize = 0;
const CLK: usize = 1;
const MOSI: usize = 2;
const DC: usize = 3;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BITS: usize = 0;
const ANN_CMD: usize = 1;
const ANN_DATA: usize = 2;
const ANN_DESC: usize = 3;

/// The longest data run a command description keeps.
const MAX_DATA_LEN: usize = 128;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "st7735",
    name: "ST7735",
    longname: "Sitronix ST7735",
    desc: "Sitronix ST7735 TFT controller protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Display", "IC"],
    channels: &[
        ChannelDesc { id: "cs", name: "CS#", desc: "Chip-select" },
        ChannelDesc { id: "clk", name: "CLK", desc: "Clock" },
        ChannelDesc { id: "mosi", name: "MOSI", desc: "Master out, slave in" },
        ChannelDesc { id: "dc", name: "DC", desc: "Data or command" },
    ],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "command", desc: "Command" },
        AnnClass { id: "data", desc: "Data" },
        AnnClass { id: "description", desc: "Description" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "fields", desc: "Fields", classes: &[1, 2] },
        AnnRow { id: "description", desc: "Description", classes: &[3] },
    ],
    binary: &[],
    doc: "This decoder decodes the Sitronix ST7735 TFT controller protocol.\n\nDetails:\n\
          http://www.displayfuture.com/Display/datasheet/controller/ST7735.pdf",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(St7735::default())
}

/// A command name and what it does, as `META` in the Python decoder.
fn meta(cmd: u8) -> Option<(&'static str, &'static str)> {
    Some(match cmd {
        0x00 => ("NOP", "No operation"),
        0x01 => ("SWRESET", "Software reset"),
        0x04 => ("RDDID", "Read display ID"),
        0x09 => ("RDDST", "Read display status"),
        0x10 => ("SLPIN", "Sleep in & booster off"),
        0x11 => ("SLPOUT", "Sleep out & booster on"),
        0x12 => ("PTLON", "Partial mode on"),
        0x13 => ("NORON", "Partial off (normal)"),
        0x20 => ("INVOFF", "Display inversion off"),
        0x21 => ("INVON", "Display inversion on"),
        0x28 => ("DISPOFF", "Display off"),
        0x29 => ("DISPON", "Display on"),
        0x2A => ("CASET", "Column address set"),
        0x2B => ("RASET", "Row address set"),
        0x2C => ("RAMWR", "Memory write"),
        0x2E => ("RAMRD", "Memory read"),
        0x30 => ("PTLAR", "Partial start/end address set"),
        0x36 => ("MADCTL", "Memory data address control"),
        0x3A => ("COLMOD", "Interface pixel format"),
        0xB1 => ("FRMCTR1", "Frame rate control (in normal mode / full colors)"),
        0xB2 => ("FRMCTR2", "Frame rate control (in idle mode / 8-colors)"),
        0xB3 => ("FRMCTR3", "Frame rate control (in partial mode / full colors) "),
        0xB4 => ("INVCTR", "Display inversion control"),
        0xB6 => ("DISSET5", "Display function set 5"),
        0xC0 => ("PWCTR1", "Power control 1"),
        0xC1 => ("PWCTR2", "Power control 2"),
        0xC2 => ("PWCTR3", "Power control 3"),
        0xC3 => ("PWCTR4", "Power control 4"),
        0xC4 => ("PWCTR5", "Power control 5"),
        0xC5 => ("VMCTR1", "VCOM control 1"),
        0xDA => ("RDID1", "Read ID1"),
        0xDB => ("RDID2", "Read ID2"),
        0xDC => ("RDID3", "Read ID3"),
        0xDD => ("RDID4", "Read ID4"),
        0xFC => ("PWCTR6", "Power control 6"),
        0xE0 => ("GMCTRP1", "Gamma '+'polarity correction characteristics setting"),
        0xE1 => ("GMCTRN1", "Gamma '-'polarity correction characteristics setting"),
        _ => return None,
    })
}

/// The ST7735 decoder.
#[derive(Default)]
pub struct St7735 {
    accum_byte: u32,
    accum_bits_num: usize,
    bit_ss: i64,
    byte_ss: i64,
    current_bit: i32,
}

impl St7735 {
    fn reset(&mut self) {
        self.accum_byte = 0;
        self.accum_bits_num = 0;
        self.bit_ss = -1;
        self.byte_ss = -1;
        self.current_bit = -1;
    }

    /// Emits the description of the command whose bytes just ended.
    ///
    /// The Python decoder indexes `META[cmd]` directly; an unknown
    /// command raises `KeyError` and ends the decode when the *next*
    /// command byte arrives. The native decoder keeps that behaviour so
    /// both stop at the same sample.
    fn put_desc(
        &self,
        ctx: &mut dyn Context,
        ss: i64,
        es: i64,
        cmd: i32,
        _data: &[u8],
    ) -> Result<()> {
        if cmd == -1 {
            return Ok(());
        }
        let cmd = cmd as u8;
        let Some((name, desc)) = meta(cmd) else {
            return Err(Error::arg(format!("unknown ST7735 command {cmd:02X}")));
        };
        ctx.put_ann(ss as u64, es as u64, ANN_DESC, vec![format!("{name}: {desc}")]);
        Ok(())
    }
}

impl Decoder for St7735 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.reset();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let mut current_cmd: i32 = -1;
        let mut current_data: Vec<u8> = Vec::new();
        let mut desc_ss: i64 = -1;
        let mut desc_es: i64 = -1;
        self.reset();

        loop {
            // "Check data on both CLK edges."
            let pins = ctx.wait(&[&[Cond::pin(CLK, CondKind::Edge)]])?;
            let cs = pins.get(CS);
            let clk = pins.get(CLK);
            let mosi = pins.get(MOSI);
            let dc = pins.get(DC);

            if cs {
                // "Wait for CS = low, ignore the rest."
                self.reset();
                continue;
            }

            if clk {
                // Read one bit.
                self.bit_ss = ctx.samplenum() as i64;
                if self.accum_bits_num == 0 {
                    self.byte_ss = ctx.samplenum() as i64;
                }
                self.current_bit = i32::from(mosi);
            }

            if !clk && self.current_bit >= 0 {
                // Process one bit.
                ctx.put_ann(
                    self.bit_ss as u64,
                    ctx.samplenum(),
                    ANN_BITS,
                    vec![self.current_bit.to_string()],
                );
                // MSB-first.
                self.accum_byte = (self.accum_byte << 1) | self.current_bit as u32;
                self.accum_bits_num += 1;
                if self.accum_bits_num == 8 {
                    // Process one byte. "DC = low for commands."
                    let ann = if dc { ANN_DATA } else { ANN_CMD };
                    ctx.put_ann(
                        self.byte_ss as u64,
                        ctx.samplenum(),
                        ann,
                        vec![format!("{:02X}", self.accum_byte)],
                    );
                    if ann == ANN_CMD {
                        self.put_desc(ctx, desc_ss, desc_es, current_cmd, &current_data)?;
                        desc_ss = self.byte_ss;
                        desc_es = ctx.samplenum() as i64;
                        current_cmd = self.accum_byte as i32;
                        current_data.clear();
                    } else {
                        if current_data.len() < MAX_DATA_LEN {
                            current_data.push(self.accum_byte as u8);
                        }
                        desc_es = ctx.samplenum() as i64;
                    }

                    self.accum_bits_num = 0;
                    self.accum_byte = 0;
                    self.byte_ss = -1;
                }
                self.current_bit = -1;
                self.bit_ss = -1;
            }
        }
    }
}
