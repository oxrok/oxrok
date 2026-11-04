// Counterpart in sigrok:
//   libsigrok/src/output/analog.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `analog`: one line per analog sample with its unit, e.g.
//! `A0: 1000.0 mV DC`.

use super::{Output, OutputContext, OutputModule, OutputOption};
use crate::core::analog::unit_string;
use crate::core::{Key, Packet, Result, Value};
use std::fmt::Write as _;

pub(super) static MODULE: OutputModule = OutputModule {
    id: "analog",
    name: "Analog",
    description: "ASCII analog data values and units",
    options: &[
        OutputOption {
            id: "digits",
            name: "Digits",
            description: "Digits to show (all or spec)",
            default: "all",
        },
        OutputOption {
            id: "meta",
            name: "Meta",
            description: "Print META lines for metadata packets",
            default: "true",
        },
    ],
    extensions: &[],
    analog: true,
    create: |ctx, options| {
        let meta = super::option_bool(options, "meta", true)?;
        Ok(Box::new(Analog::new(ctx, meta)))
    },
};

struct Analog {
    names: Vec<(usize, String)>,
    meta: bool,
    /// Whether a `META samplerate` line prints; see [`fallback`].
    samplerate: bool,
}

/// The `analog` output a front end sends what the selected output cannot
/// show. As `-O analog` but for `META samplerate` lines, which sigrok-cli
/// prints here too and oxrok does not (docs/CLI.md).
pub fn fallback(ctx: &OutputContext) -> Box<dyn Output> {
    let mut analog = Analog::new(ctx, true);
    analog.samplerate = false;
    Box::new(analog)
}

/// A `META` line's value as analog.c:111-124 prints it: a boolean as a
/// number, a float as `%f`, nothing for the types it has no branch for.
fn meta_text(value: &Value) -> String {
    match value {
        Value::Bool(v) => u8::from(*v).to_string(),
        Value::F64(v) => format!("{v:.6}"),
        Value::U64(v) => v.to_string(),
        Value::Str(v) => v.clone(),
        _ => String::new(),
    }
}

impl Analog {
    fn new(ctx: &OutputContext, meta: bool) -> Self {
        let names = ctx.channels.iter().map(|c| (c.index, c.name.clone())).collect();
        Analog { names, meta, samplerate: true }
    }

    fn name(&self, index: usize) -> String {
        self.names
            .iter()
            .find(|(i, _)| *i == index)
            .map_or_else(|| format!("?{index}"), |(_, n)| n.clone())
    }
}

/// Scales `value` to an SI prefix and returns the prefix. `digits` is
/// adjusted so the same significant digits remain.
///
/// The choice follows sigrok: values below 1 in magnitude move down one
/// prefix (1 V prints as 1000.0 mV), and the prefix never goes below what
/// the digits can show.
pub fn si_prefix(value: &mut f32, digits: &mut i32) -> &'static str {
    const PREFIXES: [&str; 10] = ["f", "p", "n", "\u{b5}", "m", "", "k", "M", "G", "T"];
    const NEG: i32 = 5;
    const POS: i32 = 4;
    if value.is_nan() {
        return "";
    }
    let logval = value.abs().log10();
    let p = logval / 3.0 - if logval < 1.0 { 1.0 } else { 0.0 };
    // The float to int conversion of an infinite (an overload reading)
    // or out of range value is undefined in C. On x86-64, where the
    // goldens are made, cvttss2si gives INT_MIN, which is what this
    // does; on aarch64 C saturates to INT_MAX and prints another
    // prefix. Rust's `as` saturates too, and INT_MAX would overflow
    // `3 * prefix` below.
    let mut prefix = if p.is_finite() && p.abs() < 2e9 { p as i32 } else { i32::MIN };
    if prefix < -NEG {
        prefix = -NEG;
    }
    if 3 * prefix < -*digits {
        prefix = (-*digits + 2 * i32::from(*digits < 0)) / 3;
    }
    if prefix > POS {
        prefix = POS;
    }
    *value /= 10f32.powi(3 * prefix);
    *digits += 3 * prefix;
    PREFIXES[(prefix + NEG) as usize]
}

/// Formats one analog value the way the `analog` output does: number,
/// space, prefix and unit text.
pub fn format_value(
    mut value: f32,
    mut digits: i32,
    unit: crate::core::Unit,
    flags: crate::core::MqFlags,
) -> String {
    let prefix = if unit.si_prefix_friendly() { si_prefix(&mut value, &mut digits) } else { "" };
    let decimals = digits.max(0) as usize;
    let unit = unit_string(unit, flags);
    // printf's spelling; Rust's is "NaN", without a sign.
    if value.is_nan() {
        let sign = if value.is_sign_negative() { "-" } else { "" };
        return format!("{sign}nan {prefix}{unit}");
    }
    format!("{value:.decimals$} {prefix}{unit}")
}

impl Output for Analog {
    fn receive(&mut self, packet: &Packet, out: &mut Vec<u8>) -> Result<()> {
        match packet {
            Packet::FrameBegin => out.extend_from_slice(b"FRAME-BEGIN\n"),
            Packet::FrameEnd => out.extend_from_slice(b"FRAME-END\n"),
            _ => {}
        }
        if let Packet::Meta(items) = packet {
            if !self.meta {
                return Ok(());
            }
            for (key, value) in items {
                if *key != Key::Samplerate || self.samplerate {
                    let text = meta_text(value);
                    out.extend_from_slice(format!("META {key}: {text}\n").as_bytes());
                }
            }
        }
        if let Packet::Analog(a) = packet {
            let n = a.channels.len();
            let mut text = String::new();
            for i in 0..a.len() {
                for (c, index) in a.channels.iter().enumerate() {
                    let value = a.samples[i * n + c];
                    let _ = writeln!(
                        text,
                        "{}: {}",
                        self.name(*index),
                        format_value(value, a.digits, a.unit, a.flags)
                    );
                }
            }
            out.extend_from_slice(text.as_bytes());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{MqFlags, Unit};

    #[test]
    fn the_fallback_shows_all_metadata_but_the_samplerate() {
        let meta = Packet::Meta(vec![
            (Key::Samplerate, Value::U64(1000)),
            (Key::Enabled, Value::Bool(true)),
            (Key::OverCurrentProtectionEnabled, Value::Bool(false)),
            (Key::Regulation, Value::Str("CC".into())),
            (Key::VoltageTarget, Value::F64(5.0)),
        ]);
        let text = |o: &mut dyn Output| {
            let mut out = Vec::new();
            o.receive(&meta, &mut out).unwrap();
            String::from_utf8(out).unwrap()
        };
        let ctx = OutputContext::default();
        let rest = "META enabled: 1\nMETA ocp_enabled: 0\nMETA regulation: CC\n\
                    META voltage_target: 5.000000\n";
        assert_eq!(text(fallback(&ctx).as_mut()), rest);
        assert_eq!(text(&mut Analog::new(&ctx, true)), format!("META samplerate: 1000\n{rest}"));
        assert_eq!(text(&mut Analog::new(&ctx, false)), "");
        // Samplerate alone gives the fallback nothing to print.
        let mut out = Vec::new();
        let rate = Packet::Meta(vec![(Key::Samplerate, Value::U64(1000))]);
        fallback(&ctx).receive(&rate, &mut out).unwrap();
        assert_eq!(out, b"");
    }

    #[test]
    fn formats_like_sigrok() {
        let f = |v: f32| format_value(v, 4, Unit::Volt, MqFlags::DC);
        assert_eq!(f(0.0), "0.0 mV DC");
        assert_eq!(f(1.0), "1000.0 mV DC");
        assert_eq!(f(2.0), "2.0000 V DC");
        assert_eq!(f(3.0902), "3.0902 V DC");
        assert_eq!(f(10.0), "10.0000 V DC");
        assert_eq!(f(-10.0), "-10.0000 V DC");
        assert_eq!(f(-1.0), "-1000.0 mV DC");
        assert_eq!(f(0.5), "500.0 mV DC");
        // Significant digits are kept, so the decimals grow with the prefix.
        assert_eq!(f(1500.0), "1.5000000 kV DC");
    }

    #[test]
    fn infinity_takes_the_prefix_of_the_digits() {
        // A UT61E overload on the 220 MOhm range: sigrok prints "inf MΩ".
        assert_eq!(format_value(f32::INFINITY, -4, Unit::Ohm, MqFlags::empty()), "inf M\u{2126}");
        assert_eq!(format_value(f32::NEG_INFINITY, 2, Unit::Volt, MqFlags::empty()), "-inf V");
    }

    #[test]
    fn nan_reads_as_printf_has_it() {
        assert_eq!(format_value(f32::NAN, 2, Unit::Celsius, MqFlags::empty()), "nan \u{b0}C");
        assert_eq!(format_value(-f32::NAN, 2, Unit::Celsius, MqFlags::empty()), "-nan \u{b0}C");
    }

    #[test]
    fn percent_has_no_prefix() {
        assert_eq!(format_value(0.5, 2, Unit::Percentage, MqFlags::empty()), "0.50 %");
    }
}
