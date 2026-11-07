// Counterpart in sigrok:
//   libsigrokdecode/decoders/ssi32/pd.py,
//   Copyright (C) 2016 Robert Bosch Car Multimedia GmbH,
//   Authors: Oleksij Rempel, GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Bosch SSI32 protocol and the decoder's
//   documented classes and texts.

//! Bosch Synchronous Serial Interface (32bit) protocol on an SPI bus.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};
use std::fmt::Write as _;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_CTRL_TX: usize = 0;
const ANN_ACK_TX: usize = 1;
const ANN_CTRL_RX: usize = 2;
const ANN_ACK_RX: usize = 3;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ssi32",
    name: "SSI32",
    longname: "Synchronous Serial Interface (32bit)",
    desc: "Synchronous Serial Interface (32bit) protocol.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["Embedded/industrial"],
    channels: &[],
    optional_channels: &[],
    options: &[OptionDesc {
        id: "msgsize",
        desc: "Message size",
        default: OptionValue::i(64),
        values: &[],
    }],
    annotations: &[
        AnnClass { id: "ctrl-tx", desc: "CTRL TX" },
        AnnClass { id: "ack-tx", desc: "ACK TX" },
        AnnClass { id: "ctrl-rx", desc: "CTRL RX" },
        AnnClass { id: "ack-rx", desc: "ACK RX" },
    ],
    annotation_rows: &[
        AnnRow { id: "tx", desc: "TX", classes: &[0, 1] },
        AnnRow { id: "rx", desc: "RX", classes: &[2, 3] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes the Bosch\n\
          SSI32 protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Ssi32::default())
}

/// The SSI32 decoder.
#[derive(Default)]
pub struct Ssi32 {
    msgsize: usize,
    ss_cmd: u64,
    es_cmd: u64,
    mosi_bytes: Vec<i64>,
    miso_bytes: Vec<i64>,
    es_array: Vec<u64>,
}

impl Ssi32 {
    fn putx(&self, ctx: &mut dyn Context, class: usize, text: String) {
        ctx.put_ann(self.ss_cmd, self.es_cmd, class, vec![text]);
    }

    fn reset_data(&mut self) {
        self.mosi_bytes.clear();
        self.miso_bytes.clear();
        self.es_array.clear();
    }

    fn handle_ack(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // Only first byte should have ACK data, other 3 bytes are reserved.
        let es =
            *self.es_array.first().ok_or_else(|| Error::arg("ssi32: ACK without an end sample"))?;
        self.es_cmd = es;
        self.putx(ctx, ANN_ACK_TX, format!("> ACK:0x{:02x}", self.mosi_bytes[0]));
        self.putx(ctx, ANN_ACK_RX, format!("< ACK:0x{:02x}", self.miso_bytes[0]));
        Ok(())
    }

    fn handle_ctrl(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let mut mosi = String::new();
        let mut miso = String::new();
        let tx_size = usize::try_from(self.mosi_bytes[2])
            .map_err(|_| Error::arg("ssi32: negative TX size"))?;
        let rx_size = usize::try_from(self.miso_bytes[2])
            .map_err(|_| Error::arg("ssi32: negative RX size"))?;

        if tx_size > 0 {
            mosi.push_str(", DATA:0x");
            // Python's slice clamps at the end of the list.
            let end = (tx_size + 4).min(self.mosi_bytes.len());
            for &b in &self.mosi_bytes[4.min(end)..end] {
                let _ = write!(mosi, "{b:02x}");
            }
        }
        if rx_size > 0 {
            miso.push_str(", DATA:0x");
            let end = (rx_size + 4).min(self.miso_bytes.len());
            for &b in &self.miso_bytes[4.min(end)..end] {
                let _ = write!(miso, "{b:02x}");
            }
        }

        self.es_cmd = *self
            .es_array
            .get(tx_size + 3)
            .ok_or_else(|| Error::arg("ssi32: TX message shorter than its size"))?;
        self.putx(
            ctx,
            ANN_CTRL_TX,
            format!(
                "> CTRL:0x{:02x}, LUN:0x{:02x}, SIZE:0x{:02x}, CRC:0x{:02x}{}",
                self.mosi_bytes[0],
                self.mosi_bytes[1],
                self.mosi_bytes[2],
                self.mosi_bytes[3],
                mosi
            ),
        );

        self.es_cmd = *self
            .es_array
            .get(rx_size + 3)
            .ok_or_else(|| Error::arg("ssi32: RX message shorter than its size"))?;
        self.putx(
            ctx,
            ANN_CTRL_RX,
            format!(
                "< CTRL:0x{:02x}, LUN:0x{:02x}, SIZE:0x{:02x}, CRC:0x{:02x}{}",
                self.miso_bytes[0],
                self.miso_bytes[1],
                self.miso_bytes[2],
                self.miso_bytes[3],
                miso
            ),
        );
        Ok(())
    }
}

/// `ptype, mosi, miso = data`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("ssi32: item is not a list"));
    };
    let [ptype, mosi, miso] = parts.as_slice() else {
        return Err(Error::arg("ssi32: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("ssi32: packet type is not a string"));
    };
    Ok((ptype.as_str(), mosi, miso))
}

fn int(item: &Item) -> Result<i64> {
    match item {
        Item::Int(v) => Ok(*v),
        _ => Err(Error::arg(format!("ssi32: not a word: {item}"))),
    }
}

impl Decoder for Ssi32 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Ssi32 {
            msgsize: usize::try_from(config.int("msgsize", 64).max(1)).unwrap_or(usize::MAX),
            ..Ssi32::default()
        };
        self.reset_data();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("ssi32 is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, mosi, miso) = split_item(item)?;

        if ptype == "CS-CHANGE" {
            self.reset_data();
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
        self.es_array.push(es);

        if self.mosi_bytes[0] & 0x80 != 0 {
            if self.mosi_bytes.len() < 4 {
                return Ok(());
            }

            self.handle_ack(ctx)?;
            self.reset_data();
        } else {
            if self.mosi_bytes.len() < self.msgsize {
                return Ok(());
            }

            self.handle_ctrl(ctx)?;
            self.reset_data();
        }
        Ok(())
    }
}
