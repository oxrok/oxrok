// Counterpart in sigrok:
//   libsigrokdecode/decoders/rgb_led_ws281x/pd.py,
//   Copyright (C) 2016 Vladimir Ermakov <vooon341@gmail.com>,
//   GPL-3.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the WS281x pulse widths and the decoder's
//   documented classes and texts.
#![allow(clippy::doc_markdown)]

//! RGB LED string (WS281x): decodes the reset-separated 24-bit GRB
//! words of a WS2812/WS2811 data line.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Config, Context, Decoder, DecoderDef, DecoderInfo,
};

const DIN: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_RESET: usize = 1;
const ANN_RGB: usize = 2;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "rgb_led_ws281x",
    name: "RGB LED (WS281x)",
    longname: "RGB LED string decoder (WS281x)",
    desc: "RGB LED string protocol (WS281x).",
    license: "gplv3+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Display", "IC"],
    channels: &[ChannelDesc { id: "din", name: "DIN", desc: "DIN data line" }],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "reset", desc: "RESET" },
        AnnClass { id: "rgb", desc: "RGB" },
    ],
    annotation_rows: &[
        AnnRow { id: "bit", desc: "Bits", classes: &[0, 1] },
        AnnRow { id: "rgb", desc: "RGB", classes: &[2] },
    ],
    binary: &[],
    doc: "WS281x RGB LED protocol decoder.\n\nDetails:\n\
          https://cpldcpu.wordpress.com/2014/01/14/light_ws2812-library-v2-0-part-i-understanding-the-ws2812/",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(RgbLedWs281x::default())
}

/// The WS281x decoder.
#[derive(Default)]
pub struct RgbLedWs281x {
    samplerate: u64,
    oldpin: Option<bool>,
    ss_packet: Option<u64>,
    ss: Option<u64>,
    es: Option<u64>,
    bits: Vec<bool>,
    inreset: bool,
}

impl RgbLedWs281x {
    /// Emits the RGB value once 24 bits have been collected.
    fn handle_bits(&mut self, ctx: &mut dyn Context, samplenum: u64) {
        if self.bits.len() != 24 {
            return;
        }
        let mut grb: u32 = 0;
        for &bit in &self.bits {
            grb = (grb << 1) | u32::from(bit);
        }
        let rgb = (grb & 0x00ff_0000) >> 8 | (grb & 0x0000_ff00) << 8 | (grb & 0x0000_00ff);
        if let Some(ss) = self.ss_packet {
            ctx.put_ann(ss, samplenum, ANN_RGB, vec![format!("#{rgb:06x}")]);
        }
        self.bits.clear();
        self.ss_packet = None;
    }
}

impl Decoder for RgbLedWs281x {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }

        loop {
            let pin = ctx.wait(&[])?.get(DIN);

            let Some(oldpin) = self.oldpin else {
                self.oldpin = Some(pin);
                continue;
            };

            // "Check RESET condition (manufacturer recommends 50 usec
            // minimal, but real minimum is ~10 usec)." The decoder uses
            // the 50 usec figure.
            if !self.inreset
                && !pin
                && self.es.is_some()
                && self.ss.is_some()
                && (ctx.samplenum() - self.es.unwrap()) as f64 / self.samplerate as f64 > 50e-6
            {
                let es = self.es.unwrap();
                let ss = self.ss.unwrap();

                // "Decode last bit value." The falling edge is noted after
                // this check, so on a slow line `es` is still the edge
                // before `ss`: a negative time, and a 0 bit, in Python.
                let th = (es as f64 - ss as f64) / self.samplerate as f64;
                let bit = th >= 625e-9;
                self.bits.push(bit);
                self.handle_bits(ctx, es);

                ctx.put_ann(ss, es, ANN_BIT, vec![u8::from(bit).to_string()]);
                ctx.put_ann(
                    es,
                    ctx.samplenum(),
                    ANN_RESET,
                    vec!["RESET".to_owned(), "RST".to_owned(), "R".to_owned()],
                );

                self.inreset = true;
                self.bits.clear();
                self.ss_packet = None;
                self.ss = None;
            }

            if !oldpin && pin {
                // Rising edge.
                if self.ss.is_some_and(|ss| ss != 0) && self.es.is_some_and(|es| es != 0) {
                    let ss = self.ss.unwrap();
                    let es = self.es.unwrap();
                    let period = ctx.samplenum() - ss;
                    let duty = es - ss;
                    // "Ideal duty for T0H: 33%, T1H: 66%."
                    let bit = (duty as f64 / period as f64) > 0.5;

                    ctx.put_ann(ss, ctx.samplenum(), ANN_BIT, vec![u8::from(bit).to_string()]);

                    self.bits.push(bit);
                    let now = ctx.samplenum();
                    self.handle_bits(ctx, now);
                }

                if self.ss_packet.is_none() {
                    self.ss_packet = Some(ctx.samplenum());
                }

                self.ss = Some(ctx.samplenum());
            } else if oldpin && !pin {
                // Falling edge.
                self.inreset = false;
                self.es = Some(ctx.samplenum());
            }

            self.oldpin = Some(pin);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decode::{Output, Stack, parse_spec};

    /// A line slower than the 50 us of a RESET: at the falling edge that
    /// ends a pulse, the last falling edge known is the one before the
    /// pulse began. What sigrok-cli prints for these samples at 20 kHz:
    /// `3-2 0`, `2-4 RESET`, `7-4 0`, `4-8 RESET`.
    #[test]
    fn a_reset_at_the_falling_edge_of_a_slow_pulse() {
        let config = parse_spec(&INFO, &[], &["D0".to_owned()]).unwrap();
        let mut stack = Stack::new();
        stack.add("ws-1".to_owned(), boxed(), &config, 1, 20_000).unwrap();
        let rx = stack.outputs().unwrap();
        stack.feed(&[1, 1, 0, 1, 0, 0, 0, 1, 0]);
        stack.end();
        let seen: Vec<(u64, u64, String)> = rx
            .into_iter()
            .filter_map(|o| match o {
                Output::Annotation { ss, es, texts, .. } => Some((ss, es, texts[0].clone())),
                _ => None,
            })
            .collect();
        stack.join();
        let want = [(3, 2, "0"), (2, 4, "RESET"), (7, 4, "0"), (4, 8, "RESET")];
        let want: Vec<(u64, u64, String)> =
            want.iter().map(|&(ss, es, text)| (ss, es, text.to_owned())).collect();
        assert_eq!(seen, want);
    }
}
