// Counterpart in sigrok:
//   libsigrokdecode/decoders/i2cfilter/pd.py,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented classes and texts.

//! I2C filter: forwards only the transactions of one slave/direction.

use crate::core::{Error, Result};
use crate::decode::{
    Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc, OptionValue,
};

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "i2cfilter",
    name: "I²C filter",
    longname: "I²C filter",
    desc: "Filter out addresses/directions in an I²C stream.",
    license: "gplv3+",
    inputs: &["i2c"],
    outputs: &["i2c"],
    tags: &["Util"],
    channels: &[],
    optional_channels: &[],
    options: &[
        OptionDesc {
            id: "address",
            desc: "Slave address to filter (decimal)",
            default: OptionValue::i(0),
            values: &[],
        },
        OptionDesc {
            id: "direction",
            desc: "Direction to filter",
            default: OptionValue::s("both"),
            values: &[OptionValue::s("read"), OptionValue::s("write"), OptionValue::s("both")],
        },
    ],
    annotations: &[],
    annotation_rows: &[],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This is a generic I²C filtering protocol decoder.\n\
\n\
It takes input from the I²C protocol decoder and removes all traffic\n\
except that from/to the specified slave address and/or direction.\n\
\n\
It then outputs the filtered data again as OUTPUT_PROTO of type/format 'i2c'\n\
(up the protocol decoder stack). No annotations are output.\n\
\n\
The I²C slave address to filter out should be passed in as an option\n\
'address', as an integer. A specific read or write operation can be selected\n\
with the 'direction' option, which should be 'read', 'write', or 'both'.\n\
\n\
Both of these are optional; if no options are specified the entire payload\n\
of the I²C session will be output.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(I2cFilter::default())
}

/// A cached I2C packet, exactly as it came in from the decoder below.
#[derive(Clone)]
struct Packet {
    ss: u64,
    es: u64,
    item: Item,
}

/// The i2cfilter decoder.
pub struct I2cFilter {
    /// -1 until an address packet was seen, as in the Python decoder.
    curslave: i64,
    /// `None` until an address packet was seen, then "read" or "write".
    curdirection: Option<&'static str>,
    packets: Vec<Packet>,
    address: i64,
    direction: String,
}

impl Default for I2cFilter {
    fn default() -> Self {
        I2cFilter {
            curslave: -1,
            curdirection: None,
            packets: Vec::new(),
            address: 0,
            direction: "both".to_owned(),
        }
    }
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("i2cfilter: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("i2cfilter: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("i2cfilter: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

impl I2cFilter {
    /// The decoding state is reset as upstream's `reset()` does, keeping
    /// the configured options.
    fn reset(&mut self) {
        self.curslave = -1;
        self.curdirection = None;
        self.packets.clear();
    }

    fn handle(&mut self, ctx: &mut dyn Context, ss: u64, es: u64, item: &Item) -> Result<()> {
        let (cmd, data) = split_packet(item)?;

        // Add the I²C packet to our local cache.
        self.packets.push(Packet { ss, es, item: item.clone() });

        if cmd == "ADDRESS READ" || cmd == "ADDRESS WRITE" {
            let databyte = match data {
                Item::Int(v) => *v,
                _ => return Err(Error::device("i2cfilter: address packet has no data byte")),
            };
            self.curslave = databyte;
            // cmd[8:].lower() is "read" for ADDRESS READ and "write" for
            // ADDRESS WRITE.
            self.curdirection = Some(if cmd == "ADDRESS READ" { "read" } else { "write" });
        } else if cmd == "STOP" || cmd == "START REPEAT" {
            // If this chunk was not for the correct slave, drop it.
            if self.address == 0 {
            } else if self.curslave != self.address {
                self.packets.clear();
                return Ok(());
            }

            // If this chunk was not in the right direction, drop it.
            if self.direction == "both" {
            } else if Some(self.direction.as_str()) != self.curdirection {
                self.packets.clear();
                return Ok(());
            }

            // Otherwise, send out the whole chunk of I²C packets. This
            // includes the STOP or START REPEAT packet just cached.
            for p in &self.packets {
                ctx.put_python(p.ss, p.es, p.item.clone());
            }

            self.packets.clear();
        }
        Ok(())
    }
}

impl Decoder for I2cFilter {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let address = config.int("address", 0);
        if !(0..=127).contains(&address) {
            return Err(Error::arg("Invalid slave (must be 0..127)."));
        }
        self.address = address;
        config.str("direction", "both").clone_into(&mut self.direction);
        self.reset();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("i2cfilter takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        self.handle(ctx, ss, es, item)
    }
}
