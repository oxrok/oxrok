// Counterpart in sigrok:
//   libsigrokdecode/decoders/rgb_led_spi/pd.py,
//   Copyright (C) 2014 Matt Ranostay <mranostay@gmail.com>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the RGB LED protocol and the decoder's documented
//   classes and texts.

//! Generic RGB LED strings whose RGB values are clocked over SPI.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_RGB: usize = 0;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "rgb_led_spi",
    name: "RGB LED (SPI)",
    longname: "RGB LED string decoder (SPI)",
    desc: "RGB LED string protocol (RGB values clocked over SPI).",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["Display"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[AnnClass { id: "rgb", desc: "RGB values" }],
    annotation_rows: &[],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes generic RGB LED string\n\
          values that are clocked over SPI in RGB values.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(RgbLedSpi::default())
}

/// The RGB LED (SPI) decoder.
#[derive(Default)]
pub struct RgbLedSpi {
    ss_cmd: u64,
    es_cmd: u64,
    mosi_bytes: Vec<i64>,
}

impl RgbLedSpi {
    fn putx(&self, ctx: &mut dyn Context, text: String) {
        ctx.put_ann(self.ss_cmd, self.es_cmd, ANN_RGB, vec![text]);
    }
}

/// `ptype, mosi, miso = data`.
fn split_item(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("rgb_led_spi: item is not a list"));
    };
    let [ptype, mosi, _] = parts.as_slice() else {
        return Err(Error::arg("rgb_led_spi: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("rgb_led_spi: packet type is not a string"));
    };
    Ok((ptype.as_str(), mosi))
}

impl Decoder for RgbLedSpi {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = RgbLedSpi::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("rgb_led_spi is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, mosi) = split_item(item)?;

        // Only care about data packets.
        if ptype != "DATA" {
            return Ok(());
        }

        if self.mosi_bytes.is_empty() {
            self.ss_cmd = ss;
        }
        let mosi = match mosi {
            Item::Int(v) => *v,
            _ => return Err(Error::arg(format!("rgb_led_spi: not a word: {mosi}"))),
        };
        self.mosi_bytes.push(mosi);

        // RGB value == 3 bytes.
        if self.mosi_bytes.len() != 3 {
            return Ok(());
        }

        let [red, green, blue] = self.mosi_bytes[..] else { unreachable!() };
        let rgb_value = (red << 16) | (green << 8) | blue;

        self.es_cmd = es;
        self.putx(ctx, format!("#{rgb_value:06x}"));
        self.mosi_bytes.clear();
        Ok(())
    }
}
