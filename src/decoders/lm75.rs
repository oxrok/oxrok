// Counterpart in sigrok:
//   libsigrokdecode/decoders/lm75/pd.py,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented
//   classes and texts.

//! National LM75 (and compatibles): temperature sensor.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc, OptionValue,
};

/// `resolution`: CONFIG[6:5] to resolution in bits. Only used by the
/// dead `sensor != 'lm75'` branch below.
fn resolution(bits: i64) -> i64 {
    match bits & 0x03 {
        0x00 => 9,
        0x01 => 10,
        0x02 => 11,
        _ => 12,
    }
}

/// `ft`: CONFIG[4:3] to fault tolerance setting.
fn fault_tolerance(bits: i64) -> i64 {
    match bits & 0x03 {
        0x00 => 1,
        0x01 => 2,
        0x02 => 4,
        _ => 6,
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    GetSlaveAddr,
    ReadRegs,
    WriteRegs,
}

/// The lm75 decoder.
pub struct Lm75 {
    state: State,
    /// Currently selected register. Upstream initialises it to 0x00 in
    /// `reset()` and never changes it, so only the temperature handler
    /// (`handle_reg_0x00`) is reachable.
    reg: i64,
    databytes: Vec<i64>,
    /// `self.options['sensor'] == 'lm75'`.
    is_lm75_sensor: bool,
    ss: u64,
    es: u64,
    ss_block: u64,
    es_block: u64,
}

impl Default for Lm75 {
    fn default() -> Self {
        Lm75 {
            state: State::Idle,
            reg: 0x00,
            databytes: Vec::new(),
            is_lm75_sensor: true,
            ss: 0,
            es: 0,
            ss_block: 0,
            es_block: 0,
        }
    }
}

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "lm75",
    name: "LM75",
    longname: "National LM75",
    desc: "National LM75 (and compatibles) temperature sensor.",
    license: "gplv2+",
    inputs: &["i2c"],
    outputs: &[],
    tags: &["Sensor"],
    channels: &[],
    optional_channels: &[],
    options: &[
        OptionDesc {
            id: "sensor",
            desc: "Sensor type",
            default: OptionValue::s("lm75"),
            values: &[OptionValue::s("lm75")],
        },
        OptionDesc {
            id: "resolution",
            desc: "Resolution (bits)",
            default: OptionValue::i(9),
            values: &[
                OptionValue::i(9),
                OptionValue::i(10),
                OptionValue::i(11),
                OptionValue::i(12),
            ],
        },
    ],
    annotations: &[
        AnnClass { id: "celsius", desc: "Temperature in degrees Celsius" },
        AnnClass { id: "kelvin", desc: "Temperature in Kelvin" },
        AnnClass { id: "text-verbose", desc: "Human-readable text (verbose)" },
        AnnClass { id: "text", desc: "Human-readable text" },
        AnnClass { id: "warnings", desc: "Human-readable warnings" },
    ],
    // lm75 defines no `annotation_rows` and no `binary` classes.
    annotation_rows: &[],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'i2c' PD and decodes the National LM75\n\
(and compatibles) temperature sensor protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Lm75::default())
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("lm75: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("lm75: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("lm75: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

impl Lm75 {
    /// Annotations which span exactly one I2C packet.
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, class, texts);
    }

    /// Annotations which span a block of I2C packets.
    fn putb(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_block, self.es_block, class, texts);
    }

    fn warn_upon_invalid_slave(&self, ctx: &mut dyn Context, addr: i64) {
        // LM75 and compatible devices have a 7-bit I2C slave address where
        // the 4 MSBs are fixed to 1001, and the 3 LSBs are configurable.
        // Thus, valid slave addresses (1001xxx) range from 0x48 to 0x4f.
        if !(0x48..=0x4f).contains(&addr) {
            self.putx(
                ctx,
                4,
                vec![format!("Warning: I²C slave 0x{addr:02x} not an LM75 compatible sensor.")],
            );
        }
    }

    fn output_temperature(&self, ctx: &mut dyn Context, s: &str, rw: &str) {
        // TODO upstream: Support for resolutions other than 9 bit.
        let before = self.databytes[0];
        let after = (self.databytes[1] >> 7) * 5;
        let celsius: f64 = format!("{before}.{after}").parse().unwrap();
        let kelvin = celsius + 273.15;
        self.putb(ctx, 0, vec![format!("{s}: {celsius:.1} °C")]);
        self.putb(ctx, 1, vec![format!("{s}: {kelvin:.1} K")]);

        // Warn about the temperature register (0x00) being read-only.
        if s == "Temperature" && rw == "WRITE" {
            let s = "Warning: The temperature register is read-only!";
            self.putb(ctx, 4, vec![s.to_owned()]);
        }
    }

    fn handle_temperature_reg(&mut self, ctx: &mut dyn Context, b: i64, s: &str, rw: &str) {
        // Common helper for the temperature/T_HYST/T_OS registers.
        if self.databytes.is_empty() {
            self.ss_block = self.ss;
            self.databytes.push(b);
            return;
        }
        self.databytes.push(b);
        self.es_block = self.es;
        self.output_temperature(ctx, s, rw);
        self.databytes.clear();
    }

    fn handle_reg_0x00(&mut self, ctx: &mut dyn Context, b: i64, rw: &str) {
        // Temperature register (16bits, read-only).
        self.handle_temperature_reg(ctx, b, "Temperature", rw);
    }

    #[allow(clippy::format_push_string)]
    fn handle_reg_0x01(&self, ctx: &mut dyn Context, b: i64, _rw: &str) {
        // Configuration register (8 bits, read/write).
        // TODO upstream: Bit-exact annotation ranges.

        let sd = b & (1 << 0);
        let tmp = if sd == 0 { "normal operation" } else { "shutdown mode" };
        let mut s = format!("SD = {sd}: {tmp}\n");
        let mut s2 = format!("SD = {tmp}, ");

        let cmp_int = b & (1 << 1);
        let tmp = if cmp_int == 0 { "comparator" } else { "interrupt" };
        s.push_str(&format!("CMP/INT = {cmp_int}: {tmp} mode\n"));
        s2.push_str(&format!("CMP/INT = {tmp}, "));

        let pol = b & (1 << 2);
        let tmp = if pol == 0 { "low" } else { "high" };
        s.push_str(&format!("POL = {pol}: OS polarity is active-{tmp}\n"));
        s2.push_str(&format!("POL = active-{tmp}, "));

        let bits = (b & ((1 << 4) | (1 << 3))) >> 3;
        s.push_str(&format!("Fault tolerance setting: {} bit(s)\n", fault_tolerance(bits)));
        s2.push_str(&format!("FT = {}", fault_tolerance(bits)));

        // Not supported by LM75, but by various compatible devices.
        if !self.is_lm75_sensor {
            // TODO upstream.
            let bits = (b & ((1 << 6) | (1 << 5))) >> 5;
            s.push_str(&format!("Resolution: {} bits\n", resolution(bits)));
            s2.push_str(&format!(", resolution = {}", resolution(bits)));
        }

        self.putx(ctx, 2, vec![s]);
        self.putx(ctx, 3, vec![s2]);
    }

    fn handle_reg_0x02(&mut self, ctx: &mut dyn Context, b: i64, rw: &str) {
        // T_HYST register (16 bits, read/write).
        self.handle_temperature_reg(ctx, b, "T_HYST trip temperature", rw);
    }

    fn handle_reg_0x03(&mut self, ctx: &mut dyn Context, b: i64, rw: &str) {
        // T_OS register (16 bits, read/write).
        self.handle_temperature_reg(ctx, b, "T_OS trip temperature", rw);
    }

    fn handle(&mut self, ctx: &mut dyn Context, cmd: &str, databyte: Option<i64>) {
        // START/STOP and ACK/NACK packets carry no data byte; Python's
        // `databyte` is None for those and only data/address branches
        // read it.
        let b = databyte.unwrap_or(0);
        match self.state {
            State::Idle => {
                // Wait for an I2C START condition.
                if cmd != "START" {
                    return;
                }
                self.state = State::GetSlaveAddr;
            }
            State::GetSlaveAddr => {
                // Wait for an address read/write operation.
                if cmd == "ADDRESS READ" || cmd == "ADDRESS WRITE" {
                    self.warn_upon_invalid_slave(ctx, b);
                    self.state =
                        if cmd == "ADDRESS READ" { State::ReadRegs } else { State::WriteRegs };
                }
            }
            State::ReadRegs | State::WriteRegs => {
                if cmd == "DATA READ" || cmd == "DATA WRITE" {
                    let rw = if cmd == "DATA READ" { "READ" } else { "WRITE" };
                    match self.reg {
                        0x00 => self.handle_reg_0x00(ctx, b, rw),
                        0x01 => self.handle_reg_0x01(ctx, b, rw),
                        0x02 => self.handle_reg_0x02(ctx, b, rw),
                        0x03 => self.handle_reg_0x03(ctx, b, rw),
                        _ => {}
                    }
                } else if cmd == "STOP" {
                    // TODO upstream: Any output?
                    self.state = State::Idle;
                }
            }
        }
    }
}

impl Decoder for Lm75 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.is_lm75_sensor = config.str("sensor", "lm75") == "lm75";
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("lm75 takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;

        // Store the start/end samples of this I2C packet.
        self.ss = ss;
        self.es = es;

        let databyte = match data {
            Item::Int(v) => Some(*v),
            _ => None,
        };
        self.handle(ctx, cmd, databyte);
        Ok(())
    }
}
