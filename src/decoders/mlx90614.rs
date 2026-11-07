// Counterpart in sigrok:
//   libsigrokdecode/decoders/mlx90614/pd.py,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented classes and texts.

//! Melexis MLX90614 infrared thermometer over I2C.
//!
//! Upstream calls this a "quick hack implementation" and it is: after a
//! repeated start it waits for three `DATA WRITE` bytes even though the
//! datasheet read uses two read bytes and a PEC byte, which is exactly
//! what the captured bus traffic contains. The first two bytes form the
//! little endian Kelvin value (`0.02 K` per count); the third byte only
//! triggers the output.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "mlx90614",
    name: "MLX90614",
    longname: "Melexis MLX90614",
    desc: "Melexis MLX90614 infrared thermometer protocol.",
    license: "gplv2+",
    inputs: &["i2c"],
    outputs: &[],
    tags: &["IC", "Sensor"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "celsius", desc: "Temperature in degrees Celsius" },
        AnnClass { id: "kelvin", desc: "Temperature in Kelvin" },
    ],
    annotation_rows: &[],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'i2c' PD and decodes the Melexis MLX90614\n\
infrared thermometer protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Mlx90614::default())
}

/// The state machine's states, as upstream's `self.state`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    IgnoreStartRepeat,
    IgnoreAddressWrite,
    GetTemperature,
}

/// The MLX90614 decoder.
pub struct Mlx90614 {
    state: State,
    /// The two temperature bytes, first and second `DATA WRITE`.
    data: Vec<i64>,
    ss: u64,
    es: u64,
}

impl Default for Mlx90614 {
    fn default() -> Self {
        Mlx90614 { state: State::IgnoreStartRepeat, data: Vec::new(), ss: 0, es: 0 }
    }
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("mlx90614: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("mlx90614: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("mlx90614: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

impl Mlx90614 {
    fn handle(
        &mut self,
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        cmd: &str,
        data: &Item,
    ) -> Result<()> {
        match self.state {
            State::IgnoreStartRepeat => {
                if cmd != "START REPEAT" {
                    return Ok(());
                }
                self.state = State::IgnoreAddressWrite;
            }
            State::IgnoreAddressWrite => {
                if cmd != "ADDRESS WRITE" {
                    return Ok(());
                }
                self.state = State::GetTemperature;
            }
            State::GetTemperature => {
                if cmd != "DATA WRITE" {
                    return Ok(());
                }
                let Item::Int(databyte) = data else {
                    return Err(Error::device("mlx90614: DATA WRITE has no data byte"));
                };
                if self.data.is_empty() {
                    self.data.push(*databyte);
                    self.ss = ss;
                } else if self.data.len() == 1 {
                    self.data.push(*databyte);
                    self.es = es;
                } else {
                    // kelvin = (self.data[0] | (self.data[1] << 8)) * 0.02
                    let kelvin = ((self.data[0] | (self.data[1] << 8)) as f64) * 0.02;
                    let celsius = kelvin - 273.15;
                    // `%3.2f`: the width of 3 never pads a two-decimal value.
                    ctx.put_ann(self.ss, self.es, 0, vec![format!("Temperature: {celsius:.2} °C")]);
                    ctx.put_ann(self.ss, self.es, 1, vec![format!("Temperature: {kelvin:.2} K")]);
                    self.state = State::IgnoreStartRepeat;
                    self.data.clear();
                }
            }
        }
        Ok(())
    }
}

impl Decoder for Mlx90614 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Mlx90614::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("mlx90614 takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;
        self.handle(ctx, ss, es, cmd, data)
    }
}
