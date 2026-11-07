// Counterpart in sigrok:
//   libsigrokdecode/decoders/i2cdemux/pd.py,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented classes and texts.

//! I2C demux: sorts I2C transactions into per-slave streams.

use crate::core::{Error, Result};
use crate::decode::{Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "i2cdemux",
    name: "I²C demux",
    longname: "I²C demultiplexer",
    desc: "Demux I²C packets into per-slave-address streams.",
    license: "gplv2+",
    inputs: &["i2c"],
    // TODO: Only known at run-time upstream, which registers one
    // OUTPUT_PYTHON per slave address.
    outputs: &[],
    tags: &["Util"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[],
    annotation_rows: &[],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This is a generic I²C demultiplexing protocol decoder.\n\
\n\
It takes an I²C stream as input and outputs multiple I²C streams, each\n\
stream containing only I²C packets for one specific I²C slave.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(I2cDemux::default())
}

/// A cached I2C packet, exactly as it came in from the decoder below.
#[derive(Clone)]
struct Packet {
    ss: u64,
    es: u64,
    item: Item,
}

/// The i2cdemux decoder.
///
/// Upstream registers one Python output stream per slave address, so a
/// decoder stacked on it sees one stream. The runtime here has a single
/// stacked output; the packets are emitted in the same order on it.
pub struct I2cDemux {
    packets: Vec<Packet>,
    /// Known slave addresses, in first-seen order.
    slaves: Vec<i64>,
    /// Current output stream, -1 before an address was seen.
    stream: i64,
    /// Number of created output streams.
    streamcount: usize,
}

impl Default for I2cDemux {
    fn default() -> Self {
        I2cDemux { packets: Vec::new(), slaves: Vec::new(), stream: -1, streamcount: 0 }
    }
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("i2cdemux: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("i2cdemux: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("i2cdemux: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

impl I2cDemux {
    fn handle(&mut self, ctx: &mut dyn Context, ss: u64, es: u64, item: &Item) -> Result<()> {
        let (cmd, data) = split_packet(item)?;

        // Add the I²C packet to our local cache.
        self.packets.push(Packet { ss, es, item: item.clone() });

        if cmd == "ADDRESS READ" || cmd == "ADDRESS WRITE" {
            let databyte = match data {
                Item::Int(v) => *v,
                _ => return Err(Error::device("i2cdemux: address packet has no data byte")),
            };
            if let Some(index) = self.slaves.iter().position(|&s| s == databyte) {
                self.stream = index as i64;
                return Ok(());
            }

            // We've never seen this slave, add a new stream.
            self.slaves.push(databyte);
            self.stream = self.streamcount as i64;
            self.streamcount += 1;
        } else if cmd == "STOP" {
            if self.stream == -1 {
                return Err(Error::device("Invalid stream!")); // FIXME?
            }

            // Send the whole chunk of I²C packets to the correct stream.
            for p in &self.packets {
                ctx.put_python(p.ss, p.es, p.item.clone());
            }

            self.packets.clear();
            self.stream = -1;
        }
        Ok(())
    }
}

impl Decoder for I2cDemux {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = I2cDemux::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("i2cdemux takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        self.handle(ctx, ss, es, item)
    }
}
