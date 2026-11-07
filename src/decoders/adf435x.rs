// Counterpart in sigrok:
//   libsigrokdecode/decoders/adf435x/pd.py,
//   Copyright (C) 2017 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   GPL-3.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the ADF4350/ADF4351 register map and the decoder's
//   documented classes and texts.

//! ADF435x: Analog Devices ADF4350/ADF4351 wideband synthesizer.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

// pd.py's ANN_REG; it is the only annotation class.
const ANN_REG: usize = 0;

/// One row of pd.py's `regs` dict: name, offset, width, parser.
struct Field {
    name: &'static str,
    offset: u32,
    width: u32,
    parser: Parser,
}

/// The per-field value formatting of pd.py, including its exact
/// strings and Python `%` conversions.
#[derive(Clone, Copy)]
enum Parser {
    None,
    NotAllowedBelow32,
    DisabledEnabled,
    Prescalar,
    PhaseAdjust,
    PdPolarity,
    Ldp,
    Ldf,
    ChargePumpCurrent,
    Muxout,
    LowNoiseLowSpur,
    ClockDividerMode,
    Abp,
    BandSelectClockMode,
    OutputPower,
    AuxOutputSelect,
    VcoPowerDown,
    RfDividerSelect,
    FeedbackSelect,
    LdPinMode,
}

const REG0: &[Field] = &[
    Field { name: "FRAC", offset: 3, width: 12, parser: Parser::None },
    Field { name: "INT", offset: 15, width: 16, parser: Parser::NotAllowedBelow32 },
];

const REG1: &[Field] = &[
    Field { name: "MOD", offset: 3, width: 12, parser: Parser::None },
    Field { name: "Phase", offset: 15, width: 12, parser: Parser::None },
    Field { name: "Prescalar", offset: 27, width: 1, parser: Parser::Prescalar },
    Field { name: "Phase Adjust", offset: 28, width: 1, parser: Parser::PhaseAdjust },
];

const REG2: &[Field] = &[
    Field { name: "Counter Reset", offset: 3, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "Charge Pump Three-State", offset: 4, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "Power-Down", offset: 5, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "PD Polarity", offset: 6, width: 1, parser: Parser::PdPolarity },
    Field { name: "LDP", offset: 7, width: 1, parser: Parser::Ldp },
    Field { name: "LDF", offset: 8, width: 1, parser: Parser::Ldf },
    Field {
        name: "Charge Pump Current Setting",
        offset: 9,
        width: 4,
        parser: Parser::ChargePumpCurrent,
    },
    Field { name: "Double Buffer", offset: 13, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "R Counter", offset: 14, width: 10, parser: Parser::None },
    Field { name: "RDIV2", offset: 24, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "Reference Doubler", offset: 25, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "MUXOUT", offset: 26, width: 3, parser: Parser::Muxout },
    Field {
        name: "Low Noise and Low Spur Modes",
        offset: 29,
        width: 2,
        parser: Parser::LowNoiseLowSpur,
    },
];

const REG3: &[Field] = &[
    Field { name: "Clock Divider", offset: 3, width: 12, parser: Parser::None },
    Field { name: "Clock Divider Mode", offset: 15, width: 2, parser: Parser::ClockDividerMode },
    Field { name: "CSR Enable", offset: 18, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "Charge Cancellation", offset: 21, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "ABP", offset: 22, width: 1, parser: Parser::Abp },
    Field {
        name: "Band Select Clock Mode",
        offset: 23,
        width: 1,
        parser: Parser::BandSelectClockMode,
    },
];

const REG4: &[Field] = &[
    Field { name: "Output Power", offset: 3, width: 2, parser: Parser::OutputPower },
    Field { name: "Output Enable", offset: 5, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "AUX Output Power", offset: 6, width: 2, parser: Parser::OutputPower },
    Field { name: "AUX Output Select", offset: 8, width: 1, parser: Parser::AuxOutputSelect },
    Field { name: "AUX Output Enable", offset: 9, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "MTLD", offset: 10, width: 1, parser: Parser::DisabledEnabled },
    Field { name: "VCO Power-Down", offset: 11, width: 1, parser: Parser::VcoPowerDown },
    Field { name: "Band Select Clock Divider", offset: 12, width: 8, parser: Parser::None },
    Field { name: "RF Divider Select", offset: 20, width: 3, parser: Parser::RfDividerSelect },
    Field { name: "Feedback Select", offset: 23, width: 1, parser: Parser::FeedbackSelect },
];

const REG5: &[Field] =
    &[Field { name: "LD Pin Mode", offset: 22, width: 2, parser: Parser::LdPinMode }];

/// pd.py's `regs` dict; `reg_value < len(regs)` gates the field output.
const REGS: [&[Field]; 6] = [REG0, REG1, REG2, REG3, REG4, REG5];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "adf435x",
    name: "ADF435x",
    longname: "Analog Devices ADF4350/1",
    desc: "Wideband synthesizer with integrated VCO.",
    license: "gplv3+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["Clock/timing", "IC", "Wireless/RF"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[AnnClass { id: "register", desc: "Register written to the device" }],
    annotation_rows: &[AnnRow { id: "registers", desc: "Register writes", classes: &[ANN_REG] }],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes the protocol spoken\nby the Analog Devices ADF4350 and ADF4351 RF synthesizer chips.\n\nDetails:\nhttp://www.analog.com/media/en/technical-documentation/data-sheets/ADF4350.pdf\nhttp://www.analog.com/media/en/technical-documentation/data-sheets/ADF4351.pdf",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Adf435x::default())
}

/// One bit of a BITS packet: `[value, ss, es]`.
#[derive(Clone, Copy)]
struct Bit {
    value: bool,
    ss: u64,
    es: u64,
}

/// The list of MOSI bits, as Python's `data1 + self.bits`.
fn parse_bits(data: &Item) -> Result<Vec<Bit>> {
    let Item::List(items) = data else {
        return Err(Error::arg("adf435x: BITS payload is not a list"));
    };
    let mut bits = Vec::with_capacity(items.len());
    for item in items {
        let Item::List(parts) = item else {
            return Err(Error::arg("adf435x: BITS entry is not a list"));
        };
        let [Item::Int(value), Item::Int(ss), Item::Int(es)] = parts.as_slice() else {
            return Err(Error::arg("adf435x: BITS entry is not [bit, ss, es]"));
        };
        bits.push(Bit { value: *value != 0, ss: *ss as u64, es: *es as u64 });
    }
    Ok(bits)
}

/// The ADF435x decoder.
#[derive(Default)]
pub struct Adf435x {
    /// pd.py keeps the bits most recent first.
    bits: Vec<Bit>,
}

impl Adf435x {
    /// pd.py: `decode_bits`, including the start and end sample choice.
    fn decode_bits(&self, offset: usize, width: usize) -> (u64, (u64, u64)) {
        let mut value = 0u64;
        for i in 0..width {
            if self.bits[offset + i].value {
                value |= 1 << i;
            }
        }
        (value, (self.bits[offset + width - 1].ss, self.bits[offset].es))
    }

    fn format(&self, parser: Parser, val: u64) -> String {
        let v = val as usize;
        match parser {
            Parser::None => val.to_string(),
            Parser::NotAllowedBelow32 => {
                if val < 32 {
                    "Not Allowed".to_owned()
                } else {
                    val.to_string()
                }
            }
            Parser::DisabledEnabled => ["Disabled", "Enabled"][v].to_owned(),
            Parser::Prescalar => ["4/5", "8/9"][v].to_owned(),
            Parser::PhaseAdjust => ["Off", "On"][v].to_owned(),
            Parser::PdPolarity => ["Negative", "Positive"][v].to_owned(),
            Parser::Ldp => ["10ns", "6ns"][v].to_owned(),
            Parser::Ldf => ["FRAC-N", "INT-N"][v].to_owned(),
            Parser::ChargePumpCurrent => {
                const MA: [&str; 16] = [
                    "0.31", "0.63", "0.94", "1.25", "1.56", "1.88", "2.19", "2.50", "2.81", "3.13",
                    "3.44", "3.75", "4.06", "4.38", "4.69", "5.00",
                ];
                format!("{}mA @ 5.1kΩ", MA[v])
            }
            Parser::Muxout => [
                "Three-State Output",
                "DVdd",
                "DGND",
                "R Counter Output",
                "N Divider Output",
                "Analog Lock Detect",
                "Digital Lock Detect",
                "Reserved",
            ][v]
                .to_owned(),
            Parser::LowNoiseLowSpur => {
                ["Low Noise Mode", "Reserved", "Reserved", "Low Spur Mode"][v].to_owned()
            }
            Parser::ClockDividerMode => {
                ["Clock Divider Off", "Fast Lock Enable", "Resync Enable", "Reserved"][v].to_owned()
            }
            Parser::Abp => ["6ns (FRAC-N)", "3ns (INT-N)"][v].to_owned(),
            Parser::BandSelectClockMode => ["Low", "High"][v].to_owned(),
            Parser::OutputPower => {
                // pd.py: '%+ddBm' % [-4, -1, 2, 5][v]
                let dbm = [-4i64, -1, 2, 5][v];
                format!("{dbm:+}dBm")
            }
            Parser::AuxOutputSelect => ["Divided Output", "Fundamental"][v].to_owned(),
            Parser::VcoPowerDown => {
                format!("VCO Powered {}", if val == 1 { "Down" } else { "Up" })
            }
            Parser::RfDividerSelect => format!("÷{}", 2u64.pow(val as u32)),
            Parser::FeedbackSelect => ["Divided", "Fundamental"][v].to_owned(),
            Parser::LdPinMode => ["Low", "Digital Lock Detect", "Low", "High"][v].to_owned(),
        }
    }

    /// pd.py: `decode_field`.
    fn decode_field(&self, ctx: &mut dyn Context, field: &Field) {
        let (val, (ss, es)) = self.decode_bits(field.offset as usize, field.width as usize);
        let text = format!("{}: {}", field.name, self.format(field.parser, val));
        ctx.put_ann(ss, es, ANN_REG, vec![text]);
    }

    /// pd.py's CS-CHANGE branch: the previous transaction is complete.
    fn end_transaction(&mut self, ctx: &mut dyn Context) {
        if self.bits.len() == 32 {
            let (reg_value, reg_pos) = self.decode_bits(0, 3);
            ctx.put_ann(
                reg_pos.0,
                reg_pos.1,
                ANN_REG,
                vec![
                    format!("Register: {reg_value}"),
                    format!("Reg: {reg_value}"),
                    format!("[{reg_value}]"),
                ],
            );
            if (reg_value as usize) < REGS.len() {
                for field in REGS[reg_value as usize] {
                    self.decode_field(ctx, field);
                }
            }
        }
        self.bits.clear();
    }
}

/// Splits an item from the SPI decoder into `(ptype, data1, data2)`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("adf435x: item is not a list"));
    };
    let [ptype, data1, data2] = parts.as_slice() else {
        return Err(Error::arg("adf435x: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("adf435x: packet type is not a string"));
    };
    Ok((ptype.as_str(), data1, data2))
}

impl Decoder for Adf435x {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let _ = config;
        *self = Adf435x::default();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let _ = ctx;
        Err(Error::not_supported("adf435x is a stacked decoder on 'spi'"))
    }

    fn decode_item(
        &mut self,
        _ss: u64,
        _es: u64,
        item: &Item,
        ctx: &mut dyn Context,
    ) -> Result<()> {
        let (ptype, data1, _data2) = split_item(item)?;

        if ptype == "CS-CHANGE" {
            // pd.py checks the *old* CS# value: 1 means the pin was
            // deasserted and is now asserted, so the bits of the
            // previous transfer are complete.
            if matches!(data1, Item::Int(1)) {
                self.end_transaction(ctx);
            }
        }

        if ptype == "BITS" {
            let bits = parse_bits(data1)?;
            let mut all = bits;
            all.extend_from_slice(&self.bits);
            self.bits = all;
        }
        Ok(())
    }
}
