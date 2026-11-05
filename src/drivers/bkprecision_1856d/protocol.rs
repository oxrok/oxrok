// Counterpart in sigrok:
//   libsigrok/src/hardware/bkprecision-1856d/api.c,
//   libsigrok/src/hardware/bkprecision-1856d/protocol.c,
//   libsigrok/src/hardware/bkprecision-1856d/protocol.h,
//   Copyright (C) 2021 LUMERIIX,
//   Copyright (C) 2024 Daniel Anselmi <danselmi@gmx.ch>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The 1856D's wire: commands of a letter, a parameter and CR, and an answer of
//! fifteen bytes.
//!
//! From the manual's section 3-7 (the C driver names the same PDF): 9600 baud,
//! 8n1. `G0` to `G3` set the gate, `F0` and `F2` the input, and `D0` asks for a
//! reading. The answer is 10 bytes of number "include dp", 4 of unit and CR. `H`
//! (hold) and `R` (remote) exist, and no path here sends them.
//!
//! The rest is what C does with it, as no recording was found:
//!
//! - The 11th byte is `M` or `k`, or anything else for no factor, and the next
//!   three must be `Hz `. The panel has only M and K, so 3.5 GHz reads in MHz. The
//!   manual writes `K (Kilo)` and `KHz`, and C tests only `k`: a counter that sends
//!   `KHz ` reads 1000 times low, here as in C, until a recording says which it is.
//! - Spaces and at most one `0` where the number and the unit go is the "empty"
//!   message: nothing to report yet, so ask again, with no new gate.
//! - The number goes through `strtod` and the digits count from where it ends:
//!   [`Frame`] keeps what that makes of any bytes, a NUL ending the text.
//! - After `G` the driver sleeps, per gate, for times nothing else explains.

use std::time::Duration;

/// `SERIALCOMM`.
pub const SERIALCOMM: &str = "9600/8n1/dtr=1/rts=0";
/// `BKPRECISION1856D_MSG_SIZE`.
pub const MSG_SIZE: usize = 15;
/// `BKPRECISION1856D_MSG_NUMBER_SIZE`.
const NUMBER_SIZE: usize = 10;
/// A command's length, `LENGHT_OF_CMD`.
pub const COMMAND_SIZE: u32 = 3;
/// `DATA_REQ`.
pub const DATA_REQUEST: &[u8; 3] = b"D0\r";

/// An entry of `timebases` and `gate_time_config_commands`.
#[derive(Debug)]
pub struct Gate {
    /// In seconds, as C's table has it: `--get` prints it so.
    pub period: (u64, u64),
    /// The command.
    pub command: &'static [u8; 3],
    /// What the info log says.
    pub info: &'static str,
    /// The sleep after the command.
    pub sleep: Duration,
}

/// The gates in C's order.
pub static GATES: [Gate; 4] = [
    Gate {
        period: (10, 1000),
        command: b"G0\r",
        info: "sending gate time 0 (10ms)",
        sleep: Duration::from_millis(40),
    },
    Gate {
        period: (100, 1000),
        command: b"G1\r",
        info: "sending gate time 1 (100ms)",
        sleep: Duration::from_millis(80),
    },
    Gate {
        period: (1, 1),
        command: b"G2\r",
        info: "sending gate time 2 (1s)",
        sleep: Duration::from_millis(80),
    },
    Gate {
        period: (10, 1),
        command: b"G3\r",
        info: "sending gate time 3 (10s)",
        sleep: Duration::from_millis(800),
    },
];

/// The gate of a period. The core reduces a fraction, so this compares values:
/// `1000ms` is `1s`, where C's table compare refuses it.
pub fn gate_of(p: u64, q: u64) -> Option<usize> {
    GATES.iter().position(|gate| {
        u128::from(p) * u128::from(gate.period.1) == u128::from(gate.period.0) * u128::from(q)
    })
}

/// The inputs in `data_sources`' order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Input {
    /// `F0`.
    A,
    /// `F2`.
    C,
}

impl Input {
    /// The input's name in `data_source`.
    pub fn name(self) -> &'static str {
        match self {
            Input::A => "A",
            Input::C => "C",
        }
    }

    /// `FUNCTION_A` or `FUNCTION_C`.
    pub fn command(self) -> &'static [u8; 3] {
        match self {
            Input::A => b"F0\r",
            Input::C => b"F2\r",
        }
    }
}

/// What a whole answer is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Frame {
    /// The last byte is no CR.
    NoCr,
    /// `check_for_zero_message`.
    Empty,
    /// Its unit is no `Hz `.
    NotFrequency,
    /// A reading in Hz, and the digits it says it has.
    Reading {
        /// The value with its factor.
        hz: f64,
        /// The digits, less the factor's.
        digits: i32,
    },
}

/// `bkprecision_1856d_parse_message`'s checks and conversion, in its order.
pub fn parse(frame: &[u8; MSG_SIZE]) -> Frame {
    if frame[MSG_SIZE - 1] != b'\r' {
        return Frame::NoCr;
    }
    let body = &frame[..MSG_SIZE - 1];
    if is_empty_message(body) {
        return Frame::Empty;
    }
    // C ends the buffer at the CR, and the first NUL ends it for strtod and strchr.
    let text = body.split(|c| *c == 0).next().unwrap_or(&[]);
    let (mut hz, end) = strtod(text);
    if &body[NUMBER_SIZE + 1..] != b"Hz " {
        return Frame::NotFrequency;
    }
    let end = end as i32;
    let mut digits = match text.iter().position(|c| *c == b'.') {
        Some(point) => end - (point as i32 + 1),
        None => end,
    };
    match body[NUMBER_SIZE] {
        b'M' => {
            hz *= 1e6;
            digits -= 6;
        }
        b'k' => {
            hz *= 1e3;
            digits -= 3;
        }
        _ => {}
    }
    Frame::Reading { hz, digits }
}

/// Spaces and at most one `0`, whatever else a reading would need.
fn is_empty_message(body: &[u8]) -> bool {
    let mut zero_found = false;
    for byte in body {
        match byte {
            b' ' => {}
            b'0' if !zero_found => zero_found = true,
            _ => return false,
        }
    }
    true
}

fn span(text: &[u8], accept: impl Fn(u8) -> bool) -> usize {
    text.iter().take_while(|c| accept(**c)).count()
}

fn starts_without_case(text: &[u8], word: &[u8]) -> bool {
    text.len() >= word.len() && text[..word.len()].eq_ignore_ascii_case(word)
}

/// C's `strtod`, in the C locale: the number `text` starts with and how far it
/// goes, 0 and nothing used where it converts nothing. Too large is infinite and
/// too small 0, as `strtod` returns them. A NUL must not be in `text`.
pub fn strtod(text: &[u8]) -> (f64, usize) {
    let spaces = span(text, |c| matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r'));
    let rest = &text[spaces..];
    let negative = rest.first() == Some(&b'-');
    let signs = usize::from(matches!(rest.first(), Some(b'-' | b'+')));
    let rest = &rest[signs..];
    let (value, used) = if starts_without_case(rest, b"infinity") {
        (f64::INFINITY, 8)
    } else if starts_without_case(rest, b"inf") {
        (f64::INFINITY, 3)
    } else if starts_without_case(rest, b"nan") {
        (f64::NAN, 3 + nan_tail(&rest[3..]))
    } else if let Some(hex) = hex_float(rest) {
        hex
    } else if let Some(decimal) = decimal(rest) {
        decimal
    } else {
        return (0.0, 0);
    };
    (if negative { -value } else { value }, spaces + signs + used)
}

/// `(` letters, digits or `_` and `)`, which glibc takes after a `nan`.
fn nan_tail(text: &[u8]) -> usize {
    if text.first() != Some(&b'(') {
        return 0;
    }
    let inside = span(&text[1..], |c| c.is_ascii_alphanumeric() || c == b'_');
    if text.get(1 + inside) == Some(&b')') { inside + 2 } else { 0 }
}

/// How many bytes an exponent with this letter takes, if there is one with digits.
fn exponent_size(text: &[u8], letters: [u8; 2]) -> usize {
    if !text.first().is_some_and(|c| letters.contains(c)) {
        return 0;
    }
    let sign = usize::from(matches!(text.get(1), Some(b'+' | b'-')));
    let digits = span(&text[1 + sign..], |c| c.is_ascii_digit());
    if digits == 0 { 0 } else { 1 + sign + digits }
}

fn decimal(text: &[u8]) -> Option<(f64, usize)> {
    let whole = span(text, |c| c.is_ascii_digit());
    let point = text.get(whole) == Some(&b'.');
    let fraction = if point { span(&text[whole + 1..], |c| c.is_ascii_digit()) } else { 0 };
    if whole + fraction == 0 {
        return None;
    }
    let mantissa = whole + usize::from(point) + fraction;
    let used = mantissa + exponent_size(&text[mantissa..], *b"eE");
    let value = std::str::from_utf8(&text[..used]).ok()?.parse().ok()?;
    Some((value, used))
}

/// `0x`, digits with a point somewhere, and a binary exponent. A 14 byte field holds
/// at most 12 hex digits, so the mantissa is exact in a double.
fn hex_float(text: &[u8]) -> Option<(f64, usize)> {
    if !starts_without_case(text, b"0x") {
        return None;
    }
    let digits = &text[2..];
    let whole = span(digits, |c| c.is_ascii_hexdigit());
    let point = digits.get(whole) == Some(&b'.');
    let fraction = if point { span(&digits[whole + 1..], |c| c.is_ascii_hexdigit()) } else { 0 };
    if whole + fraction == 0 {
        return None;
    }
    let hex = |range: &[u8], start: f64| {
        range.iter().fold(start, |value, c| {
            value * 16.0 + f64::from(char::from(*c).to_digit(16).unwrap_or(0))
        })
    };
    let mut value = hex(&digits[..whole], 0.0);
    if point {
        value =
            hex(&digits[whole + 1..whole + 1 + fraction], value) * 16f64.powi(-(fraction as i32));
    }
    let mantissa = 2 + whole + usize::from(point) + fraction;
    let size = exponent_size(&text[mantissa..], *b"pP");
    if size > 0 {
        let exponent = std::str::from_utf8(&text[mantissa + 1..mantissa + size]).ok()?;
        value *= 2f64.powi(exponent.parse().unwrap_or(i32::MAX));
    }
    Some((value, mantissa + size))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(text: &[u8]) -> [u8; MSG_SIZE] {
        text.try_into().expect("15 bytes")
    }

    fn reading(text: &[u8]) -> (f64, i32) {
        match parse(&frame(text)) {
            Frame::Reading { hz, digits } => (hz, digits),
            other => panic!("{other:?} for {}", text.escape_ascii()),
        }
    }

    #[test]
    fn the_table_of_gates_is_cs() {
        let sleeps: Vec<u128> = GATES.iter().map(|g| g.sleep.as_millis()).collect();
        assert_eq!(sleeps, [40, 80, 80, 800]);
        let commands: Vec<&[u8; 3]> = GATES.iter().map(|g| g.command).collect();
        assert_eq!(commands, [b"G0\r", b"G1\r", b"G2\r", b"G3\r"]);
        assert_eq!(DATA_REQUEST, b"D0\r");
        assert_eq!(Input::A.command(), b"F0\r");
        assert_eq!(Input::C.command(), b"F2\r");
    }

    #[test]
    fn a_gate_is_found_by_its_value() {
        assert_eq!(gate_of(10, 1000), Some(0));
        assert_eq!(gate_of(1, 100), Some(0));
        assert_eq!(gate_of(100, 1000), Some(1));
        assert_eq!(gate_of(1, 10), Some(1));
        assert_eq!(gate_of(1, 1), Some(2));
        assert_eq!(gate_of(1000, 1000), Some(2));
        assert_eq!(gate_of(10, 1), Some(3));
        assert_eq!(gate_of(2, 1), None);
        assert_eq!(gate_of(0, 1), None);
        assert_eq!(gate_of(u64::MAX, u64::MAX), Some(2));
    }

    #[test]
    fn the_prefixes_scale_the_value_and_the_digits() {
        assert_eq!(reading(b"   1.00000MHz \r"), (1e6, -1));
        assert_eq!(reading(b"   1.00000kHz \r"), (1e3, 2));
        assert_eq!(reading(b"   1.00000 Hz \r"), (1.0, 5));
        assert_eq!(reading(b"   1.00000GHz \r"), (1.0, 5));
        // As C: no factor, and the reading 1000 times low if a counter sends this.
        assert_eq!(reading(b"   1.00000KHz \r"), (1.0, 5));
    }

    #[test]
    fn the_digits_count_from_the_number_and_the_point() {
        // No point: the whole of the text up to the end of the number.
        assert_eq!(reading(b"  12345678 Hz \r"), (12_345_678.0, 10));
        assert_eq!(reading(b"      1234MHz \r"), (1234e6, 4));
        // A point past the end of the number counts back.
        assert_eq!(reading(b"12 3.4     Hz \r"), (12.0, -3));
        // No number: nothing used, so all of it counts back to the start.
        assert_eq!(reading(b"abc.def    Hz \r"), (0.0, -4));
        assert_eq!(reading(b"abcdefgh   Hz \r"), (0.0, 0));
        assert_eq!(reading(b"         . Hz \r"), (0.0, -10));
        assert_eq!(reading(b"   1.5e+03 Hz \r"), (1500.0, 5));
        assert_eq!(reading(b"0.00000000 Hz \r"), (0.0, 8));
    }

    #[test]
    fn a_nul_ends_the_text_for_the_number_and_the_point() {
        assert_eq!(reading(b"\0  1.5     Hz \r"), (0.0, 0));
        assert_eq!(reading(b"1.5\0.25    Hz \r"), (1.5, 1));
        assert_eq!(reading(b"12\0.25     Hz \r"), (12.0, 2));
    }

    #[test]
    fn what_is_empty_and_what_is_not() {
        for empty in [
            &b"              \r"[..],
            b"0             \r",
            b"             0\r",
            b"         0    \r",
        ] {
            assert_eq!(parse(&frame(empty)), Frame::Empty, "{}", empty.escape_ascii());
        }
        for other in [&b"     0  0     \r"[..], b"     \0        \r", b"  a           \r"] {
            assert_eq!(parse(&frame(other)), Frame::NotFrequency, "{}", other.escape_ascii());
        }
        // A zero with its unit is a reading of 0, and so are zeros with no point.
        assert_eq!(reading(b"0000000000 Hz \r"), (0.0, 10));
    }

    #[test]
    fn the_checks_go_in_cs_order() {
        assert_eq!(parse(&frame(b"              \n")), Frame::NoCr);
        assert_eq!(parse(&frame(b"   1.00000 Hz \n")), Frame::NoCr);
        for unit in [&b"us  "[..], b" Hz_", b"Hz  ", b"    ", b" HZ ", b" Hz\0"] {
            let mut text = b"   1.00000".to_vec();
            text.extend_from_slice(unit);
            text.push(b'\r');
            assert_eq!(parse(&frame(&text)), Frame::NotFrequency, "{}", text.escape_ascii());
        }
    }

    #[test]
    fn strtod_takes_what_c_takes() {
        for (text, value, used) in [
            (&b""[..], 0.0, 0),
            (b"   ", 0.0, 0),
            (b"-", 0.0, 0),
            (b".", 0.0, 0),
            (b"x", 0.0, 0),
            (b" \t\n\x0b\x0c\r1", 1.0, 7),
            (b"+1.5", 1.5, 4),
            (b"-1.5x", -1.5, 4),
            (b"1.", 1.0, 2),
            (b".5", 0.5, 2),
            (b"1e", 1.0, 1),
            (b"1e+", 1.0, 1),
            (b"1e2", 100.0, 3),
            (b"1E-2", 0.01, 4),
            (b"1e400", f64::INFINITY, 5),
            (b"1e-999", 0.0, 6),
            (b"inf", f64::INFINITY, 3),
            (b"INFx", f64::INFINITY, 3),
            (b"infinit", f64::INFINITY, 3),
            (b"Infinity", f64::INFINITY, 8),
            (b"-inf", f64::NEG_INFINITY, 4),
            (b"0x10", 16.0, 4),
            (b"0X1p4", 16.0, 5),
            (b"0x1.8p1", 3.0, 7),
            (b"0x1.8", 1.5, 5),
            (b"0x.8", 0.5, 4),
            (b"0x1p", 1.0, 3),
            (b"0x1p-1", 0.5, 6),
            (b"-0x2", -2.0, 4),
            // No digit after the x: the 0 alone.
            (b"0x", 0.0, 1),
            (b"0xg", 0.0, 1),
        ] {
            assert_eq!(strtod(text), (value, used), "{}", text.escape_ascii());
        }
        for (text, used, negative) in [
            (&b"nan"[..], 3, false),
            (b"NaN ", 3, false),
            (b"-nan", 4, true),
            (b"nan(1)", 6, false),
            (b"nan(a_1)x", 8, false),
            (b"nan(", 3, false),
            (b"nan(1", 3, false),
            (b"nan(-)", 3, false),
        ] {
            let (value, end) = strtod(text);
            assert!(value.is_nan(), "{}", text.escape_ascii());
            assert_eq!(value.is_sign_negative(), negative, "{}", text.escape_ascii());
            assert_eq!(end, used, "{}", text.escape_ascii());
        }
        assert!(strtod(b"-0").0.is_sign_negative());
    }

    #[test]
    fn any_fifteen_bytes_parse_without_a_panic() {
        let mut state = 0x1234_5678u32;
        for _ in 0..20_000 {
            let mut bytes = [0u8; MSG_SIZE];
            for byte in &mut bytes {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                *byte = b" .0123456789+-eExXpPnaif()\0\rHzkM"[(state >> 24) as usize % 32];
            }
            let _ = parse(&bytes);
        }
    }
}
