// Counterpart in sigrok:
//   libsigrok/src/strutil.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Parsing and formatting helpers shared by the CLI and the formats.
//!
//! The formats match sigrok's, so files and command lines written for
//! sigrok keep working: `200k` is 200000, a sample rate prints as
//! `200 kHz`, a time is `1s` or `100ms`.

use crate::core::error::{Error, Result};

const UNITS: [u64; 7] = [
    1,
    1_000,
    1_000_000,
    1_000_000_000,
    1_000_000_000_000,
    1_000_000_000_000_000,
    1_000_000_000_000_000_000,
];
const PREFIXES: [&str; 7] = ["", "k", "M", "G", "T", "P", "E"];

/// Formats `x` with an SI prefix and `unit`, trimming trailing zeros:
/// `200 kHz`, `1.5 MHz`, `1 Hz`, `7 ` for an empty unit.
pub fn si_string_u64(x: u64, unit: &str) -> String {
    let mut i = 0;
    while i + 1 < UNITS.len() && x / UNITS[i] >= 1000 {
        i += 1;
    }
    let quot = x / UNITS[i];
    let mut fract = String::new();
    if i > 0 {
        let rem = x % UNITS[i];
        if rem != 0 {
            let digits = i * 3;
            fract = format!(".{rem:0digits$}");
            while fract.ends_with('0') {
                fract.pop();
            }
            if fract.ends_with('.') {
                fract.pop();
            }
        }
    }
    format!("{quot}{fract} {}{unit}", PREFIXES[i])
}

/// A sample rate as `200 kHz`.
pub fn samplerate_string(rate: u64) -> String {
    si_string_u64(rate, "Hz")
}

/// Parses a number with an optional fraction and SI suffix: `100`,
/// `1.5k`, `200 k`, `24m`, `1g`, optionally followed by `Hz`.
///
/// Lower and upper case suffixes mean the same thing (`m` is mega, not
/// milli), as in sigrok-cli.
pub fn parse_sizestring(text: &str) -> Result<u64> {
    let bad = || Error::Arg(format!("invalid number '{text}'"));
    let s = text.trim();
    let end = s.find(|c: char| !c.is_ascii_digit()).unwrap_or(s.len());
    let (int_part, mut rest) = s.split_at(end);
    if int_part.is_empty() {
        return Err(bad());
    }
    let mut value: u64 = int_part.parse().map_err(|_| bad())?;
    let mut frac = 0.0f64;
    if let Some(after) = rest.strip_prefix('.') {
        let end = after.find(|c: char| !c.is_ascii_digit()).unwrap_or(after.len());
        let digits = &after[..end];
        if !digits.is_empty() {
            frac = format!("0.{digits}").parse().map_err(|_| bad())?;
        }
        rest = &after[end..];
    }
    rest = rest.trim_start();
    let mut multiplier = 1u64;
    if let Some(c) = rest.chars().next() {
        let m = match c.to_ascii_lowercase() {
            'k' => Some(UNITS[1]),
            'm' => Some(UNITS[2]),
            'g' => Some(UNITS[3]),
            't' => Some(UNITS[4]),
            'p' => Some(UNITS[5]),
            'e' => Some(UNITS[6]),
            _ => None,
        };
        if let Some(m) = m {
            multiplier = m;
            rest = &rest[c.len_utf8()..];
        }
    }
    let rest = rest.trim();
    if !rest.is_empty() && !rest.eq_ignore_ascii_case("hz") {
        return Err(bad());
    }
    value = value.checked_mul(multiplier).ok_or_else(bad)?;
    if frac != 0.0 {
        if multiplier == 1 {
            return Err(bad());
        }
        value = value.checked_add((frac * multiplier as f64).round() as u64).ok_or_else(bad)?;
    }
    Ok(through_double(value))
}

/// C's `*size += frac_part` (strutil.c:1248-1250) takes the size through a
/// double even with no fraction: what has more than 53 bits rounds, and what
/// rounds to 2^64 is 0 on x86-64, where the cast is out of range. So
/// sigrok-cli reads `--samples 18446744073709551615` as 0 and refuses it.
fn through_double(value: u64) -> u64 {
    let rounded = value as f64;
    if rounded >= 18_446_744_073_709_551_616.0 { 0 } else { rounded as u64 }
}

/// Parses a duration as milliseconds: `100`, `100ms`, `2s`. Returns
/// `None` for anything else.
pub fn parse_timestring(text: &str) -> Option<u64> {
    let s = text.trim();
    let end = s.find(|c: char| !c.is_ascii_digit()).unwrap_or(s.len());
    let value: u64 = s[..end].parse().ok()?;
    match s[end..].trim() {
        "" | "ms" => Some(value),
        "s" => value.checked_mul(1000),
        _ => None,
    }
}

/// Parses a boolean that must be clearly on or off: `true`, `yes`, `on`,
/// `1` or `false`, `no`, `off`, `0`, in any case. `sr_parse_boolstring()`
/// takes an empty string as true, matches prefixes (`1x`, `onward`) and
/// takes all else as false; a device must not be told `off` for a typo.
pub fn parse_bool(text: &str) -> Option<bool> {
    const ON: [&str; 4] = ["true", "yes", "on", "1"];
    const OFF: [&str; 4] = ["false", "no", "off", "0"];
    if ON.iter().any(|w| text.eq_ignore_ascii_case(w)) {
        Some(true)
    } else if OFF.iter().any(|w| text.eq_ignore_ascii_case(w)) {
        Some(false)
    } else {
        None
    }
}

/// Parses a period like `10ms`, `2.5 us`, `1 s` into a rational number of
/// seconds (p, q). The units are libsigrok's: `s` down to `fs`.
pub fn parse_period(text: &str) -> Option<(u64, u64)> {
    parse_rational(
        text,
        &[
            ("s", 1),
            ("ms", 1_000),
            ("us", 1_000_000),
            ("ns", 1_000_000_000),
            ("ps", 1_000_000_000_000),
            ("fs", 1_000_000_000_000_000),
        ],
    )
}

/// Parses a voltage like `500mV`, `2 V` into a rational number of volts.
pub fn parse_voltage(text: &str) -> Option<(u64, u64)> {
    parse_rational(text, &[("v", 1), ("mv", 1_000)])
}

fn parse_rational(text: &str, units: &[(&str, u64)]) -> Option<(u64, u64)> {
    let s = text.trim();
    let end = s.find(|c: char| !c.is_ascii_digit() && c != '.').unwrap_or(s.len());
    let number = &s[..end];
    // A unit alone is no value: libsigrok needs digits before the unit.
    if !number.bytes().any(|c| c.is_ascii_digit()) {
        return None;
    }
    let unit = s[end..].trim().to_ascii_lowercase();
    let (_, divisor) = units.iter().find(|(u, _)| *u == unit)?;
    let (int, frac) = number.split_once('.').unwrap_or((number, ""));
    let int: u64 = if int.is_empty() { 0 } else { int.parse().ok()? };
    // value = (int + frac / scale) / divisor
    let (mut p, mut q) = (int, *divisor);
    if !frac.is_empty() {
        let scale = 10u64.checked_pow(frac.len() as u32)?;
        p = p.checked_mul(scale)?.checked_add(frac.parse::<u64>().ok()?)?;
        q = q.checked_mul(scale)?;
    }
    // Reduce p/q so that 10 ms is 1/100, not 10/1000.
    let g = gcd(p, q);
    Some(if g > 1 { (p / g, q / g) } else { (p, q) })
}

fn gcd(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        (a, b) = (b, a % b);
    }
    a
}

/// A period `p/q` seconds as text with the unit that fits: `1 us`,
/// `2.5 ms`, `1 s`.
pub fn period_string(p: u64, q: u64) -> String {
    let seconds = p as f64 / q.max(1) as f64;
    let freq = 1.0 / seconds;
    let (factor, unit) = if freq > 1e9 {
        (1e12, "ps")
    } else if freq > 1e6 {
        (1e9, "ns")
    } else if freq > 1e3 {
        (1e6, "us")
    } else if freq > 1.0 {
        (1e3, "ms")
    } else {
        (1.0, "s")
    };
    let v = seconds * factor;
    if (v - v.trunc()).abs() < f64::from(f32::MIN_POSITIVE) {
        format!("{v:.0} {unit}")
    } else {
        format!("{v:.3} {unit}")
    }
}

/// A voltage `p/q` volts as sigrok prints it: `500 mV`, `2 V`, else a
/// float with the unit.
pub fn voltage_string(p: u64, q: u64) -> String {
    match q {
        1000 => format!("{p} mV"),
        1 => format!("{p} V"),
        _ => format!("{} V", fmt_g(p as f64 / q as f64, 6)),
    }
}

/// Formats `value` like C's `%.<precision>g`: `precision` significant
/// digits, trailing zeros removed, exponent form when the exponent is
/// below -4 or at least `precision`.
pub fn fmt_g(value: f64, precision: usize) -> String {
    let p = precision.max(1);
    if value == 0.0 {
        return if value.is_sign_negative() { "-0".to_owned() } else { "0".to_owned() };
    }
    if !value.is_finite() {
        return if value.is_nan() {
            if value.is_sign_negative() { "-nan" } else { "nan" }.to_owned()
        } else if value < 0.0 {
            "-inf".to_owned()
        } else {
            "inf".to_owned()
        };
    }
    // The exponent after rounding to p significant digits.
    let e_text = format!("{value:.*e}", p - 1);
    let (mantissa, exp) = e_text.split_once('e').unwrap_or((&e_text, "0"));
    let exp: i32 = exp.parse().unwrap_or(0);
    if exp < -4 || exp >= p as i32 {
        let m = trim_zeros(mantissa);
        let sign = if exp < 0 { '-' } else { '+' };
        format!("{m}e{sign}{:02}", exp.abs())
    } else {
        let decimals = (p as i32 - 1 - exp).max(0) as usize;
        trim_zeros(&format!("{value:.decimals$}"))
    }
}

fn trim_zeros(s: &str) -> String {
    if !s.contains('.') {
        return s.to_owned();
    }
    let t = s.trim_end_matches('0');
    t.trim_end_matches('.').to_owned()
}

/// How many bytes `f(c)` holds for at the start of `text`.
fn span(text: &[u8], f: impl Fn(u8) -> bool) -> usize {
    text.iter().take_while(|c| f(**c)).count()
}

/// A hexadecimal float after its `0x`, as far as it goes: `None` with no digit.
fn hex_float(text: &[u8]) -> Option<f64> {
    let whole = span(text, |c| c.is_ascii_hexdigit());
    let fraction = if text.get(whole) == Some(&b'.') {
        span(&text[whole + 1..], |c| c.is_ascii_hexdigit())
    } else {
        0
    };
    if whole + fraction == 0 {
        return None;
    }
    let digit = |c: &u8| f64::from(char::from(*c).to_digit(16).expect("a hex digit"));
    let mut value = text[..whole].iter().fold(0.0, |v, c| v * 16.0 + digit(c));
    let mut scale = 1.0 / 16.0;
    for c in text[whole..].iter().skip(1).take(fraction) {
        value += digit(c) * scale;
        scale /= 16.0;
    }
    let used = if fraction > 0 { whole + 1 + fraction } else { whole };
    let rest = &text[used..];
    if matches!(rest.first(), Some(b'p' | b'P')) {
        let sign = usize::from(matches!(rest.get(1), Some(b'+' | b'-')));
        let digits = span(&rest[1 + sign..], |c| c.is_ascii_digit());
        if digits > 0 {
            let end = 1 + sign + digits;
            let exp = std::str::from_utf8(&rest[1..end]).ok()?.parse::<i32>().unwrap_or(i32::MAX);
            value *= 2f64.powi(exp);
        }
    }
    Some(value)
}

/// `g_ascii_strtod`: the number `text` starts with, or `None` where C's would
/// convert nothing and give 0.
pub fn g_ascii_strtod(text: &[u8]) -> Option<f64> {
    let spaces = span(text, |c| matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r'));
    let text = &text[spaces..];
    let negative = text.first() == Some(&b'-');
    let text = text.strip_prefix(b"-").or_else(|| text.strip_prefix(b"+")).unwrap_or(text);
    let sign = if negative { -1.0 } else { 1.0 };
    let starts =
        |word: &[u8]| text.len() >= word.len() && text[..word.len()].eq_ignore_ascii_case(word);
    if starts(b"inf") {
        return Some(sign * f64::INFINITY);
    }
    if starts(b"nan") {
        return Some(f64::NAN);
    }
    if starts(b"0x")
        && let Some(value) = hex_float(&text[2..])
    {
        return Some(sign * value);
    }
    let whole = span(text, |c| c.is_ascii_digit());
    let fraction = if text.get(whole) == Some(&b'.') {
        span(&text[whole + 1..], |c| c.is_ascii_digit())
    } else {
        0
    };
    if whole + fraction == 0 {
        return None;
    }
    let mut used = if text.get(whole) == Some(&b'.') { whole + 1 + fraction } else { whole };
    if matches!(text.get(used), Some(b'e' | b'E')) {
        let exp_sign = usize::from(matches!(text.get(used + 1), Some(b'+' | b'-')));
        let digits = span(&text[(used + 1 + exp_sign).min(text.len())..], |c| c.is_ascii_digit());
        if digits > 0 {
            used += 1 + exp_sign + digits;
        }
    }
    let value: f64 = std::str::from_utf8(&text[..used]).ok()?.parse().ok()?;
    Some(sign * value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn g_ascii_strtod_goes_as_far_as_it_can() {
        let cases: &[(&[u8], Option<f64>)] = &[
            (b"1234", Some(1234.0)),
            (b" 12 ", Some(12.0)),
            (b"1.2 ", Some(1.2)),
            (b"-100", Some(-100.0)),
            (b"+1", Some(1.0)),
            (b"1e01", Some(10.0)),
            (b"1e", Some(1.0)),
            (b"1e+", Some(1.0)),
            (b".5", Some(0.5)),
            (b"5.", Some(5.0)),
            (b"12x4", Some(12.0)),
            (b"0x10", Some(16.0)),
            (b"0x", Some(0.0)),
            (b"0x.8", Some(0.5)),
            (b"0x1p2", Some(4.0)),
            (b"inf", Some(f64::INFINITY)),
            (b"-INF", Some(f64::NEG_INFINITY)),
            (b"", None),
            (b"xxxx", None),
            (b".", None),
            (b"-", None),
            (b"e5", None),
        ];
        for (text, want) in cases {
            assert_eq!(g_ascii_strtod(text), *want, "{}", text.escape_ascii());
        }
        assert!(g_ascii_strtod(b"nanx").unwrap().is_nan());
    }

    #[test]
    fn formats_sample_rates_like_sigrok() {
        assert_eq!(samplerate_string(200_000), "200 kHz");
        assert_eq!(samplerate_string(1_000_000_000), "1 GHz");
        assert_eq!(samplerate_string(1), "1 Hz");
        assert_eq!(samplerate_string(1_500_000), "1.5 MHz");
        assert_eq!(samplerate_string(24_000_000), "24 MHz");
        assert_eq!(samplerate_string(921_600), "921.6 kHz");
    }

    #[test]
    fn parses_sizes_like_sigrok() {
        assert_eq!(parse_sizestring("100").unwrap(), 100);
        assert_eq!(parse_sizestring("200k").unwrap(), 200_000);
        assert_eq!(parse_sizestring("1.5k").unwrap(), 1_500);
        assert_eq!(parse_sizestring("24m").unwrap(), 24_000_000);
        assert_eq!(parse_sizestring("1 MHz").unwrap(), 1_000_000);
        assert_eq!(parse_sizestring("1g").unwrap(), 1_000_000_000);
        assert!(parse_sizestring("").is_err());
        assert!(parse_sizestring("12x").is_err());
        assert!(parse_sizestring("1.5").is_err());
    }

    /// As sigrok-cli reads them: `-c limit_samples=N --get limit_samples`.
    #[test]
    fn sizes_go_through_a_double_as_in_c() {
        assert_eq!(parse_sizestring("9007199254740992").unwrap(), 9_007_199_254_740_992);
        assert_eq!(parse_sizestring("9007199254740993").unwrap(), 9_007_199_254_740_992);
        assert_eq!(parse_sizestring("18446744073709549568").unwrap(), 18_446_744_073_709_549_568);
        assert_eq!(parse_sizestring("18446744073709551615").unwrap(), 0);
        assert_eq!(parse_sizestring("18446744073709551000").unwrap(), 0);
    }

    #[test]
    fn parses_times() {
        assert_eq!(parse_timestring("100"), Some(100));
        assert_eq!(parse_timestring("100ms"), Some(100));
        assert_eq!(parse_timestring("2 s"), Some(2000));
        assert_eq!(parse_timestring("2h"), None);
    }

    #[test]
    fn parses_only_clear_booleans() {
        for on in ["true", "yes", "on", "1", "TRUE", "Yes", "oN"] {
            assert_eq!(parse_bool(on), Some(true), "{on}");
        }
        for off in ["false", "no", "off", "0", "FALSE", "No", "oFf"] {
            assert_eq!(parse_bool(off), Some(false), "{off}");
        }
        for neither in ["", " ", " on", "on ", "1x", "enable", "onward", "2", "00", "yes!", "t"] {
            assert_eq!(parse_bool(neither), None, "{neither:?}");
        }
    }

    #[test]
    fn formats_periods() {
        assert_eq!(period_string(1, 1_000_000), "1 us");
        assert_eq!(period_string(1, 1_000_000_000), "1 ns");
        assert_eq!(period_string(1, 400), "2.500 ms");
        assert_eq!(period_string(2, 1), "2 s");
    }

    #[test]
    fn formats_like_printf_g() {
        assert_eq!(fmt_g(-10.0, 6), "-10");
        assert_eq!(fmt_g(3.090_17, 6), "3.09017");
        assert_eq!(fmt_g(f64::from(3.090_17f32), 6), "3.09017");
        assert_eq!(fmt_g(f64::NAN, 6), "nan");
        assert_eq!(fmt_g(-f64::NAN, 6), "-nan");
        assert_eq!(fmt_g(0.0, 6), "0");
        assert_eq!(fmt_g(1.5e-5, 6), "1.5e-05");
        assert_eq!(fmt_g(1_234_567.0, 6), "1.23457e+06");
        assert_eq!(fmt_g(100_000.0, 6), "100000");
        assert_eq!(fmt_g(0.0001, 6), "0.0001");
        assert_eq!(fmt_g(-2.25, 16), "-2.25");
        assert_eq!(fmt_g(f64::from(0.1f32), 16), "0.1000000014901161");
    }

    #[test]
    fn parses_rationals() {
        assert_eq!(parse_period("10ms"), Some((1, 100)));
        assert_eq!(parse_period("2.5 us"), Some((1, 400_000)));
        assert_eq!(parse_voltage("500mV"), Some((1, 2)));
        assert_eq!(parse_voltage("2 V"), Some((2, 1)));
        assert_eq!(parse_voltage("2 A"), None);
    }
}

/// Parses a `probe_names` scan option: a comma separated list of
/// channel names, where an item may be an alias for a group of signals
/// (`uart`, `i2c`, `spi`, `jtag`, `swd`). The result is padded with
/// `defaults` unless the spec starts with `-`, and never longer than
/// `defaults`. An empty spec returns the defaults.
pub fn parse_probe_names(spec: &str, defaults: &[&str]) -> Vec<String> {
    let spec = spec.trim();
    let (spec, pad) = match spec.strip_prefix('-') {
        Some(rest) => (rest, false),
        None => (spec, true),
    };
    let mut names: Vec<String> = Vec::new();
    for item in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        let alias: &[&str] = match item.to_ascii_lowercase().as_str() {
            "uart" => &["rx", "tx"],
            "i2c" => &["scl", "sda"],
            "spi" => &["cs", "clk", "mosi", "miso"],
            "jtag" => &["tdi", "tdo", "tck", "tms"],
            "swd" => &["swclk", "swdio"],
            _ => &[],
        };
        if alias.is_empty() {
            names.push(item.to_owned());
        } else {
            names.extend(alias.iter().map(|s| (*s).to_owned()));
        }
    }
    if names.is_empty() {
        return defaults.iter().map(|s| (*s).to_owned()).collect();
    }
    names.truncate(defaults.len());
    if pad {
        for d in &defaults[names.len()..] {
            names.push((*d).to_owned());
        }
    }
    names
}

#[cfg(test)]
mod probe_names_tests {
    use super::parse_probe_names;

    #[test]
    fn probe_names() {
        let d = ["D0", "D1", "D2", "D3"];
        assert_eq!(parse_probe_names("", &d), ["D0", "D1", "D2", "D3"]);
        assert_eq!(parse_probe_names("rx,tx", &d), ["rx", "tx", "D2", "D3"]);
        assert_eq!(parse_probe_names("-i2c", &d), ["scl", "sda"]);
        assert_eq!(parse_probe_names("spi,a,b,c", &d), ["cs", "clk", "mosi", "miso"]);
    }
}
