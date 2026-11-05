// Counterpart in sigrok:
//   libsigrok/src/lcr/vc4080.c,
//   Copyright (C) 2019 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Voltcraft 4080 and its rebrands, as `src/lcr/vc4080.c` has them: 39 ASCII
//! characters answer an `N`, at 1200/7e1 by the meter's documents and read at
//! 8n1 here with the parity bit in bit 7, as the cable's FT232R gave nothing at
//! 7e1. So every byte loses bit 7 before it is looked at, which also works on a
//! line really at 7e1.
//!
//! ```text
//!  0      L, C, R         5-9   main value, '8' first: switching ranges,
//!  1      Q, D, R, _            '9' first: over the range
//!  2      A 1 kHz, B 120 Hz    10   main range, 0-6
//!  3      P, S            11-14 secondary value
//!  4      A auto, M       15    secondary range, 1-5
//! 16      a sequence counter, which nothing reads
//! 17-21   D and its range, 22-26 Q and its range: no channel takes them
//! 27-36   S setup, F fuse, H hold, R/M/I/A/X, R/S relative, L limits,
//!         T/S tolerance, B backlight, A adapter, B low battery; _ for none
//! 37 38   CR LF
//! ```
//!
//! Not copied from C: `d_range`, `q_range`, `d_invalid` and `q_invalid`
//! (vc4080.c:393-404), for channels `VC4080_WITH_DQ_CHANS` leaves out; line 404
//! subtracts `'0'` from `d_range` twice and never from `q_range`, which nothing
//! can see.

use super::{Reading, Settings, pow10};
use crate::core::{Mq, MqFlags, Unit};
use std::time::Duration;

/// A packet's size.
pub const PACKET_SIZE: usize = 39;

/// What asks for a packet.
pub const REQUEST: &[u8] = b"N";

/// `req_timeout_ms` in the driver's table.
pub const REQUEST_INTERVAL: Duration = Duration::from_millis(500);

/// `frequencies[]`.
pub const FREQUENCIES: [f64; 2] = [120.0, 1000.0];

/// `circuit_models[]`.
pub const CIRCUIT_MODELS: [&str; 3] = ["PARALLEL", "SERIES", "NONE"];

/// The frame: CR LF at its end, whatever else is there.
pub fn valid(pkt: &[u8]) -> bool {
    pkt[37] & 0x7f == b'\r' && pkt[38] & 0x7f == b'\n'
}

fn strip_parity(pkt: &[u8]) -> [u8; PACKET_SIZE] {
    let mut out = [0; PACKET_SIZE];
    for (o, b) in out.iter_mut().zip(pkt) {
        *o = b & 0x7f;
    }
    out
}

fn frequency(code: u8) -> u64 {
    match code {
        b'A' => 1000,
        b'B' => 120,
        _ => 0,
    }
}

/// What the function and the model letter make: only L and C have a model.
fn circuit_model(function: u8, code: u8) -> &'static str {
    match (function, code) {
        (b'L' | b'C', b'P') => "PARALLEL",
        (b'L' | b'C', b'S') => "SERIES",
        _ => "NONE",
    }
}

/// The meter's settings.
pub fn settings(pkt: &[u8]) -> Settings {
    let p = strip_parity(pkt);
    Settings { output_freq: frequency(p[2]), circuit_model: circuit_model(p[0], p[3]) }
}

/// `strtod` on all of `text`, as `g_ascii_strtod` in the C locale and
/// `sr_atof_ascii`'s check have it: `None` where text is left over or the value
/// is out of a double's range. No conversion on an empty text gives 0.
fn strtod_whole(text: &[u8]) -> Option<f64> {
    if text.is_empty() {
        return Some(0.0);
    }
    let start = text.iter().position(|b| !matches!(b, b' ' | 9..=13))?;
    let text = &text[start..];
    let (negative, rest) = match text[0] {
        b'-' => (true, &text[1..]),
        b'+' => (false, &text[1..]),
        _ => (false, text),
    };
    let lower = rest.to_ascii_lowercase();
    let value = if lower == b"inf" || lower == b"infinity" {
        f64::INFINITY
    } else if lower == b"nan"
        || (lower.starts_with(b"nan(")
            && lower.ends_with(b")")
            && lower[4..lower.len() - 1].iter().all(|b| b.is_ascii_alphanumeric() || *b == b'_'))
    {
        f64::NAN
    } else if let Some(hex) = lower.strip_prefix(b"0x") {
        parse_hex(hex)?
    } else {
        if !lower.first().is_some_and(|b| b.is_ascii_digit() || *b == b'.') {
            return None;
        }
        let value: f64 = std::str::from_utf8(&lower).ok()?.parse().ok()?;
        // ERANGE: strtod's overflow is an error there.
        if value.is_infinite() {
            return None;
        }
        value
    };
    Some(if negative { -value } else { value })
}

/// A hexadecimal float's digits, `[hex][.hex][p[+-]dec]`, which five characters
/// can hold little of. No rounding matters at that size.
fn parse_hex(text: &[u8]) -> Option<f64> {
    let (mantissa, exponent) = match text.iter().position(|b| *b == b'p') {
        Some(at) => (&text[..at], Some(&text[at + 1..])),
        None => (text, None),
    };
    let (whole, fraction) = match mantissa.iter().position(|b| *b == b'.') {
        Some(at) => (&mantissa[..at], &mantissa[at + 1..]),
        None => (mantissa, &mantissa[..0]),
    };
    if whole.is_empty() && fraction.is_empty() {
        return None;
    }
    let digit = |b: &u8| char::from(*b).to_digit(16);
    let mut value = 0.0f64;
    for b in whole {
        value = value * 16.0 + f64::from(digit(b)?);
    }
    let mut scale = 1.0 / 16.0;
    for b in fraction {
        value += f64::from(digit(b)?) * scale;
        scale /= 16.0;
    }
    if let Some(exponent) = exponent {
        let exponent: i32 = std::str::from_utf8(exponent).ok()?.parse().ok()?;
        value *= 2f64.powi(exponent);
        if value.is_infinite() {
            return None;
        }
    }
    Some(value)
}

/// `parse_number`: the field as a C string, to a float, 0 if it is none.
fn parse_number(field: &[u8]) -> f32 {
    let end = field.iter().position(|b| *b == 0).unwrap_or(field.len());
    strtod_whole(&field[..end]).map_or(0.0, |v| v as f32)
}

/// A range's digit, or a value no table has: `'9' - '0'` in C.
fn range_of(code: u8) -> u8 {
    if (b'0'..=b'6').contains(&code) { code - b'0' } else { 9 }
}

/// `get_main_scale_rs`: the power of ten of the main display, and the Rs it
/// says, for its function, frequency and range.
fn main_scale_rs(range: u8, function: u8, freq: u64) -> Option<(i32, i32)> {
    const DIG_R: [i32; 7] = [-3, -2, -1, 0, 1, 2, 3];
    const DIG_L_1K: [i32; 7] = [-7, -6, -5, -4, -3, -2, -1];
    const DIG_L_120: [i32; 7] = [-6, -5, -4, -3, -2, -1, 0];
    const DIG_C_1K: [i32; 7] = [-13, -12, -11, -10, -9, -8, -7];
    const DIG_C_120: [i32; 7] = [-12, -11, -10, -9, -8, -7, -6];
    const RS_R_L: [i32; 7] = [100, 100, 100, 1000, 10_000, 100_000, 100_000];
    const RS_C: [i32; 7] = [100_000, 100_000, 10_000, 1000, 100, 100, 100];
    if range > 6 {
        return None;
    }
    let (digits, rs) = match (function, freq) {
        (b'R', _) => (&DIG_R, &RS_R_L),
        (b'L', 1000) => (&DIG_L_1K, &RS_R_L),
        (b'L', 120) => (&DIG_L_120, &RS_R_L),
        (b'C', 1000) => (&DIG_C_1K, &RS_C),
        (b'C', 120) => (&DIG_C_120, &RS_C),
        _ => return None,
    };
    Some((digits[usize::from(range)], rs[usize::from(range)]))
}

/// `get_sec_scale`: the power of ten of the secondary display, for its letter,
/// the range and the Rs of the main display.
fn sec_scale(range: u8, letter: u8, rs: i32) -> Option<i32> {
    const DIG_D_Q: [i32; 6] = [0, -1, -2, -3, -4, 0];
    const DIG_R_100: [i32; 6] = [0, -2, -1, 0, 1, 0];
    const DIG_R_1K_10K: [i32; 6] = [0, -2, -1, 0, 1, 2];
    const DIG_R_100K: [i32; 6] = [0, 0, -1, 0, 1, 2];
    if !(1..=5).contains(&range) {
        return None;
    }
    let table = match (letter, rs) {
        (b'D' | b'Q', _) if range <= 4 => &DIG_D_Q,
        (b'R', 100) if range <= 4 => &DIG_R_100,
        (b'R', 1000 | 10_000) => &DIG_R_1K_10K,
        (b'R', 100_000) if range >= 2 => &DIG_R_100K,
        _ => return None,
    };
    Some(table[usize::from(range)])
}

/// What `channel` reads, the main display on 0 and the secondary on 1. Nothing
/// while the range is switching, in a mode the driver does not take (setup,
/// max - min, relative setup, limits, tolerance), on a letter it does not know,
/// or when the display has no scale. A meter on low battery says so on each
/// value: it may be wrong.
pub fn reading(pkt: &[u8], channel: usize) -> Option<Reading> {
    let p = strip_parity(pkt);
    if channel >= 2 || !matches!(p[0], b'L' | b'R' | b'C') {
        return None;
    }
    if !matches!(p[1], b'D' | b'Q' | b'R' | b'_') || p[5] == b'8' || p[27] != b'_' {
        return None;
    }
    let mut flags = MqFlags::empty();
    match p[30] {
        b'R' | b'_' => {}
        b'M' => flags |= MqFlags::MAX,
        b'I' => flags |= MqFlags::MIN,
        b'A' => flags |= MqFlags::AVG,
        _ => return None,
    }
    match p[31] {
        b'R' => flags |= MqFlags::RELATIVE,
        b'_' => {}
        _ => return None,
    }
    if p[32] != b'_' || p[33] != b'_' {
        return None;
    }
    if p[4] == b'A' {
        flags |= MqFlags::AUTORANGE;
    }
    if p[29] == b'H' {
        flags |= MqFlags::HOLD;
    }
    if p[35] == b'A' {
        flags |= MqFlags::FOUR_WIRE;
    }
    let main = main_scale_rs(range_of(p[10]), p[0], frequency(p[2]));
    let parallel = circuit_model(p[0], p[3]) == "PARALLEL";
    let (mq, unit, text, over, exponent) = if channel == 0 {
        let (mq, unit) = match p[0] {
            b'L' if parallel => (Mq::ParallelInductance, Unit::Henry),
            b'L' => (Mq::SeriesInductance, Unit::Henry),
            b'C' if parallel => (Mq::ParallelCapacitance, Unit::Farad),
            b'C' => (Mq::SeriesCapacitance, Unit::Farad),
            _ => (Mq::SeriesResistance, Unit::Ohm),
        };
        (mq, unit, &p[5..10], p[5] == b'9', main?.0)
    } else {
        let rs = main.map_or(0, |m| m.1);
        let exponent = sec_scale(range_of(p[15]), p[1], rs)?;
        let (mq, unit) = match p[1] {
            b'D' => (Mq::DissipationFactor, Unit::Unitless),
            b'Q' => (Mq::QualityFactor, Unit::Unitless),
            _ => (Mq::Resistance, Unit::Ohm),
        };
        // A '9' first is no overflow here: real values start with one.
        (mq, unit, &p[11..15], false, exponent)
    };
    if p[36] == b'B' {
        log::warn!("Low battery.");
    }
    let (value, digits) =
        if over { (f32::INFINITY, 0) } else { (parse_number(text) * pow10(exponent), -exponent) };
    Some(Reading { value, mq, flags, unit, digits })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The first 27 characters and the 10 flags, with the frame.
    fn packet(head: &str, flags: &str) -> Vec<u8> {
        let p = format!("{head}{flags}\r\n").into_bytes();
        assert_eq!(p.len(), PACKET_SIZE);
        p
    }

    const HEAD: &str = "LQAPA1234530134210711414062";
    const NO_FLAGS: &str = "__________";

    fn with_flags(flags: &str) -> Vec<u8> {
        packet(HEAD, flags)
    }

    #[test]
    fn the_frame_is_cr_lf_with_or_without_parity_bits() {
        let p = with_flags(NO_FLAGS);
        assert!(valid(&p));
        let mut q = p.clone();
        q[37] |= 0x80;
        q[38] |= 0x80;
        assert!(valid(&q));
        for (at, byte) in [(37, b'\n'), (38, b'\r')] {
            let mut q = p.clone();
            q[at] = byte;
            assert!(!valid(&q), "{at}");
        }
    }

    #[test]
    fn parity_bits_mean_nothing() {
        let p = with_flags(NO_FLAGS);
        let q: Vec<u8> = p.iter().map(|b| b | 0x80).collect();
        assert_eq!(settings(&q), settings(&p));
        assert_eq!(reading(&q, 0), reading(&p, 0));
        assert_eq!(reading(&q, 1), reading(&p, 1));
    }

    #[test]
    fn the_documented_examples() {
        // vc4080.c:97-101: C, Q, 120, ser, man; 09891 @2000uF -> C = 989.1uF.
        let p = packet("CQBSM0989150013417583100134", NO_FLAGS);
        let r = reading(&p, 0).unwrap();
        assert_eq!(r.mq, Mq::SeriesCapacitance);
        assert_eq!(r.unit, Unit::Farad);
        assert_eq!(r.value, 9891.0 * pow10(-7));
        assert_eq!(r.digits, 7);
        assert!(r.flags.is_empty(), "manual");
        // Fields 0013 and range 4 by the layout the code has, where the
        // comment reads 00134 -> Q = 13.4.
        let r = reading(&p, 1).unwrap();
        assert_eq!(r.mq, Mq::QualityFactor);
        assert_eq!(r.value, 13.0 * pow10(-4));
        assert_eq!(settings(&p), Settings { output_freq: 120, circuit_model: "SERIES" });
    }

    #[test]
    fn main_ranges_scale_by_function_and_frequency() {
        let scale = |f, code, r| {
            let head = format!("{f}Q{code}SA12345{r}0134210711414062");
            reading(&packet(&head, NO_FLAGS), 0).map(|x| x.digits)
        };
        assert_eq!(scale('R', 'A', 0), Some(3));
        assert_eq!(scale('R', 'B', 6), Some(-3), "R has one table");
        assert_eq!(scale('R', 'Z', 3), Some(0), "whatever the frequency");
        assert_eq!(scale('L', 'A', 0), Some(7));
        assert_eq!(scale('L', 'A', 6), Some(1));
        assert_eq!(scale('L', 'B', 0), Some(6));
        assert_eq!(scale('L', 'B', 6), Some(0));
        assert_eq!(scale('C', 'A', 0), Some(13));
        assert_eq!(scale('C', 'A', 6), Some(7));
        assert_eq!(scale('C', 'B', 0), Some(12));
        assert_eq!(scale('C', 'B', 6), Some(6));
        assert_eq!(scale('L', 'Z', 3), None, "L needs its frequency");
        assert_eq!(scale('C', 'Z', 3), None, "C too");
        assert_eq!(scale('L', 'A', 7), None);
        assert_eq!(scale('L', 'A', 9), None);
    }

    #[test]
    fn secondary_ranges_scale_by_letter_and_rs() {
        let scale = |f, l, main, r| {
            let head = format!("{f}{l}ASA12345{main}0134{r}10711414062");
            reading(&packet(&head, NO_FLAGS), 1).map(|x| x.digits)
        };
        // D, Q at any Rs, ranges 1 to 4.
        assert_eq!(scale('L', 'D', 3, 1), Some(1));
        assert_eq!(scale('L', 'Q', 3, 4), Some(4));
        assert_eq!(scale('L', 'Q', 3, 0), None);
        assert_eq!(scale('L', 'Q', 3, 5), None);
        // R: Rs 100.
        assert_eq!(scale('L', 'R', 0, 1), Some(2));
        assert_eq!(scale('L', 'R', 0, 4), Some(-1));
        assert_eq!(scale('L', 'R', 0, 5), None);
        // R: Rs 1k and 10k.
        assert_eq!(scale('L', 'R', 3, 1), Some(2));
        assert_eq!(scale('L', 'R', 3, 5), Some(-2));
        assert_eq!(scale('C', 'R', 2, 5), Some(-2));
        // R: Rs 100k.
        assert_eq!(scale('L', 'R', 5, 1), None);
        assert_eq!(scale('L', 'R', 5, 2), Some(1));
        assert_eq!(scale('L', 'R', 5, 5), Some(-2));
        assert_eq!(scale('C', 'R', 0, 2), Some(1), "C has its Rs the other way round");
        // No letter, or a main display with no Rs.
        assert_eq!(scale('R', '_', 3, 2), None);
        assert_eq!(scale('L', 'R', 9, 2), None);
    }

    #[test]
    fn a_secondary_does_not_need_a_good_main_display() {
        let head = "LQZPA1234530134210711414062";
        assert_eq!(reading(&packet(head, NO_FLAGS), 0), None);
        assert_eq!(reading(&packet(head, NO_FLAGS), 1).unwrap().mq, Mq::QualityFactor);
    }

    #[test]
    fn quantities_and_models() {
        let mq = |head: &str, ch| reading(&packet(head, NO_FLAGS), ch).unwrap().mq;
        assert_eq!(mq("LQAPA1234530134210711414062", 0), Mq::ParallelInductance);
        assert_eq!(mq("LQASA1234530134210711414062", 0), Mq::SeriesInductance);
        assert_eq!(mq("LQAXA1234530134210711414062", 0), Mq::SeriesInductance);
        assert_eq!(mq("CQAPA1234530134210711414062", 0), Mq::ParallelCapacitance);
        assert_eq!(mq("CQASA1234530134210711414062", 0), Mq::SeriesCapacitance);
        assert_eq!(mq("RQAPA1234530134210711414062", 0), Mq::SeriesResistance);
        assert_eq!(mq("LDAPA1234530134210711414062", 1), Mq::DissipationFactor);
        assert_eq!(mq("LRAPA1234530134210711414062", 1), Mq::Resistance);
        let model = |head: &str| settings(&packet(head, NO_FLAGS)).circuit_model;
        assert_eq!(model("LQAPA1234530134210711414062"), "PARALLEL");
        assert_eq!(model("CQASA1234530134210711414062"), "SERIES");
        assert_eq!(model("LQAXA1234530134210711414062"), "NONE");
        assert_eq!(model("RQAPA1234530134210711414062"), "NONE");
        assert_eq!(model("XQAPA1234530134210711414062"), "NONE");
        let freq = |head: &str| settings(&packet(head, NO_FLAGS)).output_freq;
        assert_eq!(freq("LQAPA1234530134210711414062"), 1000);
        assert_eq!(freq("LQBPA1234530134210711414062"), 120);
        assert_eq!(freq("LQZPA1234530134210711414062"), 0);
    }

    #[test]
    fn letters_that_stop_a_packet() {
        for head in [
            "XQAPA1234530134210711414062",
            "LXAPA1234530134210711414062",
            "LQAPA8234530134210711414062",
        ] {
            let p = packet(head, NO_FLAGS);
            assert_eq!(reading(&p, 0), None, "{head}");
            assert_eq!(reading(&p, 1), None, "{head}");
        }
        for flags in [
            "S_________",
            "X_________",
            "___X______",
            "___Q______",
            "____S_____",
            "____X_____",
            "_____L____",
            "______T___",
            "______S___",
        ] {
            let p = with_flags(flags);
            assert_eq!(reading(&p, 0), None, "{flags}");
            assert_eq!(reading(&p, 1), None, "{flags}");
        }
        assert_eq!(reading(&with_flags(NO_FLAGS), 2), None);
    }

    #[test]
    fn flags_each_say_theirs() {
        let on = |flags: &str| reading(&with_flags(flags), 0).unwrap().flags;
        assert_eq!(on(NO_FLAGS), MqFlags::AUTORANGE);
        assert!(on("__H_______").contains(MqFlags::HOLD));
        assert!(on("___M______").contains(MqFlags::MAX));
        assert!(on("___I______").contains(MqFlags::MIN));
        assert!(on("___A______").contains(MqFlags::AVG));
        assert_eq!(on("___R______"), MqFlags::AUTORANGE, "the live reading");
        assert!(on("____R_____").contains(MqFlags::RELATIVE));
        assert!(on("________A_").contains(MqFlags::FOUR_WIRE));
        assert_eq!(on("_F_____B__"), MqFlags::AUTORANGE, "fuse and backlight");
    }

    #[test]
    fn over_the_range_on_the_main_display_only() {
        let r = reading(&packet("LQAPA9000030134210711414062", NO_FLAGS), 0).unwrap();
        assert_eq!(r.value, f32::INFINITY);
        assert_eq!(r.digits, 0);
        let r = reading(&packet("LQAPA1234539134210711414062", NO_FLAGS), 1).unwrap();
        assert_eq!(r.value, 9134.0 * pow10(-2));
    }

    #[test]
    fn numbers_as_c_reads_them() {
        let n = |text: &[u8]| parse_number(text);
        assert_eq!(n(b"12345"), 12345.0);
        assert_eq!(n(b"12a34"), 0.0);
        assert_eq!(n(b" 1.50"), 1.5);
        assert_eq!(n(b"\t\n\x0b\x0c\r1"), 1.0, "all of isspace");
        assert_eq!(n(b"\x011"), 0.0, "not a space");
        assert_eq!(n(b"1.5e1"), 15.0);
        assert_eq!(n(b"1e+1"), 10.0);
        assert_eq!(n(b"1e"), 0.0);
        assert_eq!(n(b"+1234"), 1234.0);
        assert_eq!(n(b"-123"), -123.0);
        assert_eq!(n(b"+-12"), 0.0);
        assert_eq!(n(b"--12"), 0.0);
        assert_eq!(n(b"."), 0.0);
        assert_eq!(n(b".5"), 0.5);
        assert_eq!(n(b"5."), 5.0);
        assert_eq!(n(b"e5"), 0.0);
        assert_eq!(n(b"0x1f0"), 496.0);
        assert_eq!(n(b"0X1F"), 31.0);
        assert_eq!(n(b"0x"), 0.0);
        assert_eq!(n(b"0x.8"), 0.5);
        assert_eq!(n(b"0x1p3"), 8.0);
        assert_eq!(n(b"0x1pz"), 0.0);
        assert_eq!(n(b"0xg"), 0.0);
        assert_eq!(n(b"12\x0045"), 12.0, "a C string");
        assert_eq!(n(b"\0\0\0\0"), 0.0);
        assert_eq!(n(b"1e999"), 0.0, "ERANGE");
        assert_eq!(n(b"9e99"), f32::INFINITY, "a double that no float holds");
        assert_eq!(n(b"1e-99"), 0.0, "and the other way");
        assert_eq!(n(b"inf  "), 0.0, "text left over");
        assert_eq!(n(b"infin"), 0.0);
        assert_eq!(n(b"inf\0"), f32::INFINITY);
        assert_eq!(n(b"INF"), f32::INFINITY);
        assert_eq!(n(b"-inf"), f32::NEG_INFINITY);
        assert!(n(b"nan\0").is_nan());
        assert!(n(b"NaN").is_nan());
        assert!(n(b"nan()").is_nan());
        assert_eq!(n(b"nan(!"), 0.0);
        assert_eq!(n(b"   "), 0.0);
    }

    #[test]
    fn scales_are_float_products() {
        let r = reading(&with_flags(NO_FLAGS), 0).unwrap();
        assert_eq!(r.value, 12345.0 * pow10(-4));
        assert_eq!(r.digits, 4);
        let r = reading(&with_flags(NO_FLAGS), 1).unwrap();
        assert_eq!(r.value, 134.0 * pow10(-2));
        assert_eq!(r.digits, 2);
    }
}
