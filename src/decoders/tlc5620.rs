// Counterpart in sigrok:
//   libsigrokdecode/decoders/tlc5620/pd.py,
//   Copyright (C) 2012-2015 Uwe Hermann <uwe@hermann-uwe.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the TLC5620 serial interface and the decoder's
//   documented classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - DATA is shifted in on the falling CLK edge, MSB first. The
//     decoder keeps at most the last 11 bits; fewer than 11 on LOAD is
//     a "Command too short" error.
//   - The 11 bits are 2 DAC select bits, 1 gain bit and 8 value bits.
//     The value annotation runs to a guessed end, one clock width past
//     the last bit: `self.es_value = self.bits[10][1] + self.clock_width`.
//   - A low LDAC sets the voltage immediately (class 7), otherwise the
//     value is stored in a register (class 6). With the optional LDAC
//     channel unassigned `pins[3]` is None in Python, which is not 0,
//     so the register path is taken.
//   - The LDAC falling edge updates all four DACs at once and prints
//     the register values, or `?` for a DAC that was never written.
//   - The reference voltages are read per DAC from the vref_a..vref_d
//     options; the voltage is `vref * (value / 256) * gain` with the
//     gain being 1 or 2, printed with two decimals and a `V` suffix.

//! TI TLC5620: an 8-bit quad DAC on a serial interface.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

// Channel indices, in declaration order.
const CLK: usize = 0;
const DATA: usize = 1;
const LOAD: usize = 2;
const LDAC: usize = 3;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_DAC_SELECT: usize = 0;
const ANN_GAIN: usize = 1;
const ANN_VALUE: usize = 2;
const ANN_DATA_LATCH: usize = 3;
const ANN_LDAC_FALL: usize = 4;
const ANN_BIT: usize = 5;
const ANN_REG_WRITE: usize = 6;
const ANN_VOLTAGE_UPDATE: usize = 7;
const ANN_VOLTAGE_UPDATE_ALL: usize = 8;
const ANN_INVALID_CMD: usize = 9;

static OPTIONS: [OptionDesc; 4] = [
    OptionDesc {
        id: "vref_a",
        desc: "Reference voltage DACA (V)",
        default: OptionValue::f(3.3),
        values: &[],
    },
    OptionDesc {
        id: "vref_b",
        desc: "Reference voltage DACB (V)",
        default: OptionValue::f(3.3),
        values: &[],
    },
    OptionDesc {
        id: "vref_c",
        desc: "Reference voltage DACC (V)",
        default: OptionValue::f(3.3),
        values: &[],
    },
    OptionDesc {
        id: "vref_d",
        desc: "Reference voltage DACD (V)",
        default: OptionValue::f(3.3),
        values: &[],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "tlc5620",
    name: "TI TLC5620",
    longname: "Texas Instruments TLC5620",
    desc: "Texas Instruments TLC5620 8-bit quad DAC.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["IC", "Analog/digital"],
    channels: &[
        ChannelDesc { id: "clk", name: "CLK", desc: "Serial interface clock" },
        ChannelDesc { id: "data", name: "DATA", desc: "Serial interface data" },
    ],
    optional_channels: &[
        ChannelDesc { id: "load", name: "LOAD", desc: "Serial interface load control" },
        ChannelDesc { id: "ldac", name: "LDAC", desc: "Load DAC" },
    ],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "dac-select", desc: "DAC select" },
        AnnClass { id: "gain", desc: "Gain" },
        AnnClass { id: "value", desc: "DAC value" },
        AnnClass { id: "data-latch", desc: "Data latch point" },
        AnnClass { id: "ldac-fall", desc: "LDAC falling edge" },
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "reg-write", desc: "Register write" },
        AnnClass { id: "voltage-update", desc: "Voltage update" },
        AnnClass { id: "voltage-update-all", desc: "Voltage update (all DACs)" },
        AnnClass { id: "invalid-cmd", desc: "Invalid command" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[5] },
        AnnRow { id: "fields", desc: "Fields", classes: &[0, 1, 2] },
        AnnRow { id: "registers", desc: "Registers", classes: &[6, 7] },
        AnnRow { id: "voltage-updates", desc: "Voltage updates", classes: &[8] },
        AnnRow { id: "events", desc: "Events", classes: &[3, 4] },
        AnnRow { id: "errors", desc: "Errors", classes: &[9] },
    ],
    binary: &[],
    doc: "The Texas Instruments TLC5620 is an 8-bit quad DAC.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Tlc5620::new())
}

/// pd.py `dacs`: the two select bits as a DAC name.
fn dac_name(select: u32) -> &'static str {
    match select {
        0 => "DACA",
        1 => "DACB",
        2 => "DACC",
        _ => "DACD",
    }
}

struct Tlc5620 {
    /// `(bit, samplenum)` of the shifted in bits.
    bits: Vec<(u8, u64)>,
    ss_dac_first: Option<u64>,
    ss_dac: u64,
    es_dac: u64,
    ss_gain: u64,
    es_gain: u64,
    ss_value: u64,
    es_value: u64,
    dac_select: Option<&'static str>,
    gain: Option<u32>,
    dac_value: Option<u32>,
    /// The last value of each DAC, and its gain, as pd.py keeps them.
    dacval: [Option<u32>; 4],
    gains: [Option<u32>; 4],
    vrefs: [f64; 4],
    /// The level of the optional LDAC pin, or None when unassigned.
    ldac: Option<bool>,
}

impl Tlc5620 {
    fn new() -> Tlc5620 {
        Tlc5620 {
            bits: Vec::new(),
            ss_dac_first: None,
            ss_dac: 0,
            es_dac: 0,
            ss_gain: 0,
            es_gain: 0,
            ss_value: 0,
            es_value: 0,
            dac_select: None,
            gain: None,
            dac_value: None,
            dacval: [None; 4],
            gains: [None; 4],
            vrefs: [3.3; 4],
            ldac: None,
        }
    }

    fn dac_index(name: &str) -> usize {
        match name.as_bytes().last() {
            Some(b'B') => 1,
            Some(b'C') => 2,
            Some(b'D') => 3,
            _ => 0,
        }
    }

    /// pd.py `handle_11bits()`: the last 11 bits are a DAC command.
    fn handle_11bits(&mut self, ctx: &mut dyn Context) -> bool {
        // Only look at the last 11 bits, the rest is ignored by the TLC5620.
        if self.bits.len() > 11 {
            let start = self.bits.len() - 11;
            self.bits.drain(..start);
        }

        // If there are less than 11 bits, something is probably wrong.
        if self.bits.len() < 11 {
            let samplenum = ctx.samplenum();
            let mut ss = samplenum;
            let mut es = samplenum;
            if self.bits.len() >= 2 {
                ss = self.bits[0].1;
                es = self.bits[self.bits.len() - 1].1 + (self.bits[1].1 - self.bits[0].1);
            }
            ctx.put_ann(ss, es, ANN_INVALID_CMD, vec!["Command too short".into()]);
            self.bits.clear();
            return false;
        }

        self.ss_dac = self.bits[0].1;
        self.es_dac = self.bits[2].1;
        self.ss_gain = self.bits[2].1;
        self.es_gain = self.bits[3].1;
        self.ss_value = self.bits[3].1;
        let clock_width = self.es_gain - self.ss_gain;
        self.es_value = self.bits[10].1 + clock_width;

        if self.ss_dac_first.is_none() {
            self.ss_dac_first = Some(self.ss_dac);
        }

        let select = (u32::from(self.bits[0].0) << 1) | u32::from(self.bits[1].0);
        let s = dac_name(select);
        self.dac_select = Some(s);
        ctx.put_ann(
            self.ss_dac,
            self.es_dac,
            ANN_DAC_SELECT,
            vec![
                format!("DAC select: {s}"),
                format!("DAC sel: {s}"),
                format!("DAC: {s}"),
                format!("D: {s}"),
                s.to_owned(),
                s[3..].to_owned(),
            ],
        );

        let g = 1 + u32::from(self.bits[2].0);
        self.gain = Some(g);
        ctx.put_ann(
            self.ss_gain,
            self.es_gain,
            ANN_GAIN,
            vec![format!("Gain: x{g}"), format!("G: x{g}"), format!("x{g}")],
        );

        let v = self.bits[3..].iter().fold(0u32, |v, &(bit, _)| (v << 1) | u32::from(bit));
        self.dac_value = Some(v);
        ctx.put_ann(
            self.ss_value,
            self.es_value,
            ANN_VALUE,
            vec![
                format!("DAC value: {v}"),
                format!("Value: {v}"),
                format!("Val: {v}"),
                format!("V: {v}"),
                v.to_string(),
            ],
        );

        // Emit an annotation for each bit.
        for i in 1..11 {
            ctx.put_ann(
                self.bits[i - 1].1,
                self.bits[i].1,
                ANN_BIT,
                vec![self.bits[i - 1].0.to_string()],
            );
        }
        ctx.put_ann(
            self.bits[10].1,
            self.bits[10].1 + clock_width,
            ANN_BIT,
            vec![self.bits[10].0.to_string()],
        );

        self.bits.clear();

        true
    }

    fn handle_falling_edge_load(&mut self, ctx: &mut dyn Context) {
        if !self.handle_11bits(ctx) {
            return;
        }
        let s = self.dac_select.expect("DAC select");
        let v = self.dac_value.expect("DAC value");
        let g = self.gain.expect("gain");
        ctx.put_ann(
            ctx.samplenum(),
            ctx.samplenum(),
            ANN_DATA_LATCH,
            vec!["Falling edge on LOAD".into(), "LOAD fall".into(), "F".into()],
        );
        let idx = Self::dac_index(s);
        let vref = self.vrefs[idx];
        let volt = format!("{:.2}V", vref * (f64::from(v) / 256.0) * f64::from(g));
        if self.ldac == Some(false) {
            // If LDAC is low, the voltage is set immediately.
            ctx.put_ann(
                self.ss_dac,
                self.es_value,
                ANN_VOLTAGE_UPDATE,
                vec![format!("Setting {s} voltage to {volt}"), format!("{s}={volt}")],
            );
        } else {
            // If LDAC is high, the voltage is not set immediately, but
            // rather stored in a register.
            ctx.put_ann(
                self.ss_dac,
                self.es_value,
                ANN_REG_WRITE,
                vec![format!("Setting {s} register value to {volt}"), format!("{s}={volt}")],
            );
        }
        // Save the last value the respective DAC was set to.
        self.dacval[idx] = Some(v);
        self.gains[idx] = Some(g);
    }

    fn handle_falling_edge_ldac(&mut self, ctx: &mut dyn Context) {
        ctx.put_ann(
            ctx.samplenum(),
            ctx.samplenum(),
            ANN_LDAC_FALL,
            vec!["Falling edge on LDAC".into(), "LDAC fall".into(), "LDAC".into(), "L".into()],
        );

        // Don't emit any annotations if we didn't see any register writes.
        let Some(ss_dac_first) = self.ss_dac_first else { return };

        // Calculate voltages based on Vref and the per-DAC gain.
        let vals: [String; 4] = std::array::from_fn(|i| match (self.dacval[i], self.gains[i]) {
            (Some(val), Some(gain)) => {
                let v = self.vrefs[i] * (f64::from(val) / 256.0) * f64::from(gain);
                format!("{v:.2}V")
            }
            _ => "?".to_owned(),
        });
        let s = format!("DACA={} DACB={} DACC={} DACD={}", vals[0], vals[1], vals[2], vals[3]);
        ctx.put_ann(
            ss_dac_first,
            ctx.samplenum(),
            ANN_VOLTAGE_UPDATE_ALL,
            vec![format!("Updating voltages: {s}"), s.clone(), s.replace("DAC", "")],
        );
        self.ss_dac_first = None;
    }
}

impl Decoder for Tlc5620 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.bits.clear();
        self.ss_dac_first = None;
        self.ss_dac = 0;
        self.es_dac = 0;
        self.ss_gain = 0;
        self.es_gain = 0;
        self.ss_value = 0;
        self.es_value = 0;
        self.dac_select = None;
        self.gain = None;
        self.dac_value = None;
        self.dacval = [None; 4];
        self.gains = [None; 4];
        self.vrefs = [
            config.float("vref_a", 3.3),
            config.float("vref_b", 3.3),
            config.float("vref_c", 3.3),
            config.float("vref_d", 3.3),
        ];
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            let clk_fall = [Cond::pin(CLK, CondKind::Falling)];
            let load_fall = [Cond::pin(LOAD, CondKind::Falling)];
            let ldac_fall = [Cond::pin(LDAC, CondKind::Falling)];
            let pins = ctx.wait(&[&clk_fall, &load_fall, &ldac_fall])?;
            let matched = ctx.matched().to_vec();
            let ldac = if ctx.has_channel(LDAC) { Some(pins.0[LDAC] == 1) } else { None };
            self.ldac = ldac;

            if matched[0] {
                self.bits.push((pins.0[DATA], ctx.samplenum()));
            }
            if matched[1] {
                self.handle_falling_edge_load(ctx);
            }
            if matched[2] {
                self.handle_falling_edge_ldac(ctx);
            }
        }
    }
}
