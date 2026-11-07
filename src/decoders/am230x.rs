// Counterpart in sigrok:
//   libsigrokdecode/decoders/am230x/pd.py,
//   Copyright (C) 2014 Johannes Roemer <jroemer@physik.uni-wuerzburg.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the single-wire timing table and the decoder's
//   documented classes and texts.

//! AM230x/DHTxx/RHTxx: single-wire humidity and temperature sensors.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const SDA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_START: usize = 0;
const ANN_RESPONSE: usize = 1;
const ANN_BIT: usize = 2;
const ANN_END: usize = 3;
const ANN_BYTE: usize = 4;
const ANN_HUMIDITY: usize = 5;
const ANN_TEMPERATURE: usize = 6;
const ANN_CHECKSUM: usize = 7;

// Timing values in microseconds, as the module-level `timing` dict of
// pd.py spells them out.
const TIMING: [(&str, f64, f64); 7] = [
    ("START LOW", 750.0, 25_000.0),
    ("START HIGH", 10.0, 10_000.0),
    ("RESPONSE LOW", 50.0, 90.0),
    ("RESPONSE HIGH", 50.0, 90.0),
    ("BIT LOW", 45.0, 90.0),
    ("BIT 0 HIGH", 20.0, 35.0),
    ("BIT 1 HIGH", 65.0, 80.0),
];

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "device",
    desc: "Device type",
    default: OptionValue::s("am230x"),
    values: &[OptionValue::s("am230x/rht"), OptionValue::s("dht11")],
}];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "am230x",
    name: "AM230x",
    longname: "Aosong AM230x/DHTxx/RHTxx",
    desc: "Aosong AM230x/DHTxx/RHTxx humidity/temperature sensor.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["IC", "Sensor"],
    channels: &[ChannelDesc { id: "sda", name: "SDA", desc: "Single wire serial data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "start", desc: "Start" },
        AnnClass { id: "response", desc: "Response" },
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "end", desc: "End" },
        AnnClass { id: "byte", desc: "Byte" },
        AnnClass { id: "humidity", desc: "Relative humidity in percent" },
        AnnClass { id: "temperature", desc: "Temperature in degrees Celsius" },
        AnnClass { id: "checksum", desc: "Checksum" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0, 1, 2, 3] },
        AnnRow { id: "bytes", desc: "Bytes", classes: &[4] },
        AnnRow { id: "results", desc: "Results", classes: &[5, 6, 7] },
    ],
    binary: &[],
    doc: "This decoder handles the proprietary single wire communication protocol used\n\
          by the Aosong AM230x/DHTxx/RHTxx series of digital humidity and temperature\n\
          sensors.\n\
          \n\
          Sample rate:\n\
          A sample rate of at least 200kHz is recommended to properly detect all the\n\
          elements of the protocol.\n\
          \n\
          Options:\n\
          The AM230x and DHTxx/RHTxx digital humidity and temperature sensors use the\n\
          same single-wire protocol with different encoding of the measured values.\n\
          Therefore the option 'device' must be used to properly decode the\n\
          communication of the respective sensor.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Am230x::new())
}

#[allow(clippy::enum_variant_names)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    WaitForStartLow,
    WaitForStartHigh,
    WaitForResponseLow,
    WaitForResponseHigh,
    WaitForFirstBit,
    WaitForBitHigh,
    WaitForBitLow,
    WaitForEnd,
}

/// The `AM230x` decoder.
pub struct Am230x {
    samplerate: u64,
    /// Sample counts of the timing table, as `self.cnt[name]`.
    cnt: [(f64, f64); 7],
    state: State,
    fall: u64,
    rise: u64,
    bits: Vec<u8>,
    bytepos: Vec<u64>,
    dht11: bool,
}

impl Am230x {
    fn new() -> Self {
        Am230x {
            samplerate: 0,
            cnt: [(0.0, 0.0); 7],
            state: State::WaitForStartLow,
            fall: 0,
            rise: 0,
            bits: Vec::new(),
            bytepos: Vec::new(),
            dht11: false,
        }
    }

    fn reset_variables(&mut self) {
        self.state = State::WaitForStartLow;
        self.fall = 0;
        self.rise = 0;
        self.bits.clear();
        self.bytepos.clear();
    }

    fn putfs(&mut self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.fall, ctx.samplenum(), class, texts);
    }

    fn putb(&mut self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        let ss = *self.bytepos.last().expect("bytepos not empty");
        ctx.put_ann(ss, ctx.samplenum(), class, texts);
    }

    fn putv(&mut self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        let ss = self.bytepos[self.bytepos.len() - 2];
        ctx.put_ann(ss, ctx.samplenum(), class, texts);
    }

    fn is_valid(&self, ctx: &dyn Context, name: &str) -> bool {
        let (min, max) = Self::limits(&self.cnt, name);
        let dt = if name.ends_with("LOW") {
            ctx.samplenum() - self.fall
        } else if name.ends_with("HIGH") {
            ctx.samplenum() - self.rise
        } else {
            0
        };
        let dt = dt as f64;
        // pd.py: `if dt >= self.cnt[name]['min'] and dt <= self.cnt[name]['max']`.
        dt >= min && dt <= max
    }

    fn limits(cnt: &[(f64, f64); 7], name: &str) -> (f64, f64) {
        let i = TIMING.iter().position(|t| t.0 == name).expect("timing name");
        cnt[i]
    }

    fn bits2num(bits: &[u8]) -> u64 {
        // pd.py: `number += bitlist[-1 - i] * 2**i`, most significant first.
        let mut number = 0u64;
        for i in 0..bits.len() {
            number += u64::from(bits[bits.len() - 1 - i]) * 2u64.pow(i as u32);
        }
        number
    }

    fn calculate_humidity(&self, bits: &[u8]) -> f64 {
        if self.dht11 {
            Self::bits2num(&bits[..8]) as f64
        } else {
            Self::bits2num(bits) as f64 / 10.0
        }
    }

    fn calculate_temperature(&self, bits: &[u8]) -> f64 {
        if self.dht11 {
            Self::bits2num(&bits[..8]) as f64
        } else {
            let mut t = Self::bits2num(&bits[1..]) as f64 / 10.0;
            if bits[0] == 1 {
                t = -t;
            }
            t
        }
    }

    fn calculate_checksum(bits: &[u8]) -> u64 {
        let mut checksum = 0u64;
        let mut i = 8;
        while i <= bits.len() {
            checksum += Self::bits2num(&bits[i - 8..i]);
            i += 8;
        }
        checksum % 256
    }

    fn handle_byte(&mut self, ctx: &mut dyn Context, bit: u8) {
        self.bits.push(bit);
        self.putfs(ctx, ANN_BIT, vec![format!("Bit: {bit}"), bit.to_string()]);
        self.fall = ctx.samplenum();
        self.state = State::WaitForBitHigh;
        if self.bits.len().is_multiple_of(8) {
            let byte = Self::bits2num(&self.bits[self.bits.len() - 8..]);
            self.putb(ctx, ANN_BYTE, vec![format!("Byte: {byte:#04x}"), format!("{byte:#04x}")]);
            if self.bits.len() == 16 {
                let h = self.calculate_humidity(&self.bits[self.bits.len() - 16..]);
                self.putv(
                    ctx,
                    ANN_HUMIDITY,
                    vec![format!("Humidity: {h:.1} %"), format!("RH = {h:.1} %")],
                );
            } else if self.bits.len() == 32 {
                let t = self.calculate_temperature(&self.bits[self.bits.len() - 16..]);
                self.putv(
                    ctx,
                    ANN_TEMPERATURE,
                    vec![format!("Temperature: {t:.1} °C"), format!("T = {t:.1} °C")],
                );
            } else if self.bits.len() == 40 {
                let parity = Self::bits2num(&self.bits[self.bits.len() - 8..]);
                if parity == Self::calculate_checksum(&self.bits[..32]) {
                    self.putb(ctx, ANN_CHECKSUM, vec!["Checksum: OK".into(), "OK".into()]);
                } else {
                    self.putb(ctx, ANN_CHECKSUM, vec!["Checksum: not OK".into(), "NOK".into()]);
                }
                self.state = State::WaitForEnd;
            }
            self.bytepos.push(ctx.samplenum());
        }
    }
}

impl Decoder for Am230x {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.dht11 = config.str("device", "am230x") == "dht11";
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        // pd.py: `timing[e][t] * self.samplerate / 1000000`.
        for (i, (_, min, max)) in TIMING.iter().enumerate() {
            self.cnt[i] =
                (*min * samplerate as f64 / 1_000_000.0, *max * samplerate as f64 / 1_000_000.0);
        }
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::device("Cannot decode without samplerate."));
        }
        loop {
            match self.state {
                State::WaitForStartLow => {
                    ctx.wait(&[&[Cond::pin(SDA, CondKind::Falling)]])?;
                    self.fall = ctx.samplenum();
                    self.state = State::WaitForStartHigh;
                }
                State::WaitForStartHigh => {
                    ctx.wait(&[&[Cond::pin(SDA, CondKind::Rising)]])?;
                    if self.is_valid(ctx, "START LOW") {
                        self.rise = ctx.samplenum();
                        self.state = State::WaitForResponseLow;
                    } else {
                        self.reset_variables();
                    }
                }
                State::WaitForResponseLow => {
                    ctx.wait(&[&[Cond::pin(SDA, CondKind::Falling)]])?;
                    if self.is_valid(ctx, "START HIGH") {
                        self.putfs(ctx, ANN_START, vec!["Start".into(), "S".into()]);
                        self.fall = ctx.samplenum();
                        self.state = State::WaitForResponseHigh;
                    } else {
                        self.reset_variables();
                    }
                }
                State::WaitForResponseHigh => {
                    ctx.wait(&[&[Cond::pin(SDA, CondKind::Rising)]])?;
                    if self.is_valid(ctx, "RESPONSE LOW") {
                        self.rise = ctx.samplenum();
                        self.state = State::WaitForFirstBit;
                    } else {
                        self.reset_variables();
                    }
                }
                State::WaitForFirstBit => {
                    ctx.wait(&[&[Cond::pin(SDA, CondKind::Falling)]])?;
                    if self.is_valid(ctx, "RESPONSE HIGH") {
                        self.putfs(ctx, ANN_RESPONSE, vec!["Response".into(), "R".into()]);
                        self.fall = ctx.samplenum();
                        self.bytepos.push(ctx.samplenum());
                        self.state = State::WaitForBitHigh;
                    } else {
                        self.reset_variables();
                    }
                }
                State::WaitForBitHigh => {
                    ctx.wait(&[&[Cond::pin(SDA, CondKind::Rising)]])?;
                    if self.is_valid(ctx, "BIT LOW") {
                        self.rise = ctx.samplenum();
                        self.state = State::WaitForBitLow;
                    } else {
                        self.reset_variables();
                    }
                }
                State::WaitForBitLow => {
                    ctx.wait(&[&[Cond::pin(SDA, CondKind::Falling)]])?;
                    let bit = if self.is_valid(ctx, "BIT 0 HIGH") {
                        0
                    } else if self.is_valid(ctx, "BIT 1 HIGH") {
                        1
                    } else {
                        self.reset_variables();
                        continue;
                    };
                    self.handle_byte(ctx, bit);
                }
                State::WaitForEnd => {
                    ctx.wait(&[&[Cond::pin(SDA, CondKind::Rising)]])?;
                    self.putfs(ctx, ANN_END, vec!["End".into(), "E".into()]);
                    self.reset_variables();
                }
            }
        }
    }
}
