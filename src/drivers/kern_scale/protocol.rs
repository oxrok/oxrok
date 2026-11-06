// Counterpart in sigrok:
//   libsigrok/src/scale/kern.c,
//   Copyright (C) 2015 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Aurelien Jacobs, Daniel Elstner, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The packets of a KERN scale, as text: 14 or 15 bytes, as its menu says.
//!
//! ```text
//! 14: P1 D1 .. D7     U1 U2 S1 S2 CR LF
//! 15: P1 D1 .. D7 D8  U1 U2 S1 S2 CR LF
//! ```
//!
//! P1 is a sign or a space, the D's a number, U1 U2 the unit, S1 a tolerance's
//! result, which nothing reads, and S2 a state: `S` stable, `U` not, `E` an error,
//! with the digits worthless. KERN's manual `EW_EG-(N)-EWB-BA-e-1226`, chapter 8,
//! has the layout; sigrok-dumps' `uart/kern_ew_6200-2nm`, captured from a real
//! unit, has `+0026.90 G U`, `-0000.04 G S`, `+002014.8CT S` and over its range
//! `+0000.00 G E`.
//!
//! The manual has leading zeros suppressed with spaces (8.4.3) and a space as a
//! sign (8.4.2). The captures are all zero filled, but a number behind spaces is
//! still that number: C's `strtod` takes no space behind a sign (`scale/kern.c:63`),
//! so `+  127.20` is 0 there, and 127.2 here.
//!
//! A digit field holds digits, one point and spaces, by the same chapter. Anything
//! else is no reading, where C's `strtod` makes 0 of `+-0026.90` and `+abcdefgh`, and
//! hex, infinities, NaNs and exponents of what looks like those. Only `E` makes a
//! reading of digits that say nothing, as the captures have `+0000.00 G E` over the
//! range.
//!
//! A limit of the format: there is no checksum. A point lost from `+00127.20 G S`
//! leaves `+0012720 G S`, a good 14-byte packet that reads 12720.

use crate::core::util::g_ascii_strtod;
use crate::core::{MqFlags, Unit};

/// The longer packet: the size of a scan's window (`api.c:194`).
pub const PACKET_SIZE: usize = 15;
/// The shorter packet.
const SHORT_PACKET_SIZE: usize = 14;

/// A packet's end, if it has one, as `get_buflen`.
fn packet_len(buf: &[u8]) -> Option<usize> {
    if buf.get(12..14) == Some(b"\r\n") {
        Some(SHORT_PACKET_SIZE)
    } else if buf.get(13..15) == Some(b"\r\n") {
        Some(PACKET_SIZE)
    } else {
        None
    }
}

/// `sr_kern_packet_valid`: the length of the packet at the head of `buf`.
///
/// `buf` has at least 14 bytes, of which the 15th is read only when the first 14
/// do not end a line.
pub fn valid(buf: &[u8]) -> Option<usize> {
    let len = packet_len(buf)?;
    let s1 = buf[len - 4];
    let s2 = buf[len - 3];
    let good = matches!(buf[0], b'+' | b'-' | b' ')
        && matches!(s1, b'L' | b'G' | b'H' | b' ')
        && matches!(s2, b'U' | b'S' | b'E' | b' ');
    good.then_some(len)
}

/// What the head of a capture's buffer holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Head {
    /// A packet of this length.
    Packet(usize),
    /// A byte that starts none.
    Skip,
    /// Too little to say.
    Wait,
}

/// Looks at the head of a capture's buffer.
///
/// C looks at 15 bytes at a time and steps 15 over any packet (`protocol.c:89-92`),
/// so it needs a byte after a 14-byte packet and takes it. That byte is the next
/// packet's sign, so C reads every second packet of a 14-byte stream. Here a packet
/// takes its own length.
pub fn head(buf: &[u8]) -> Head {
    if buf.len() < SHORT_PACKET_SIZE {
        return Head::Wait;
    }
    if let Some(len) = valid(buf) {
        return Head::Packet(len);
    }
    // 14 bytes that do not end a line may still start 15 that do.
    if buf.len() < PACKET_SIZE && packet_len(buf).is_none() {
        return Head::Wait;
    }
    Head::Skip
}

/// A packet's meaning.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// The number, or infinity when the scale says its digits mean nothing.
    pub value: f32,
    /// Digits behind the point.
    pub digits: i32,
    /// The unit: none when the packet names no unit, where C leaves it at 0.
    pub unit: Unit,
    /// `UNSTABLE` or nothing.
    pub flags: MqFlags,
}

/// A digit field, with its sign's byte gone, as a number: digits and at most one
/// point, with spaces around them.
fn number(field: &[u8]) -> Option<f64> {
    let start = field.iter().position(|&b| b != b' ')?;
    let end = field.iter().rposition(|&b| b != b' ')? + 1;
    let body = &field[start..end];
    let is_point = |&b: &u8| b == b'.';
    let fits = body.iter().all(|&b| b.is_ascii_digit() || b == b'.')
        && body.iter().position(is_point) == body.iter().rposition(is_point);
    if fits { g_ascii_strtod(body) } else { None }
}

/// `sr_kern_parse`, on a packet [`valid`] took: `None` when it holds no reading.
pub fn parse(packet: &[u8]) -> Option<Reading> {
    let len = packet.len();
    let digits_end = len - 6;
    let s2 = packet[len - 3];
    let (value, digits) = if s2 == b'E' {
        log::trace!("Over/under limit.");
        (f32::INFINITY, 0)
    } else {
        let magnitude = number(&packet[1..digits_end])?;
        let value = if packet[0] == b'-' { -magnitude } else { magnitude };
        let text = &packet[..digits_end];
        let digits = text.iter().position(|&b| b == b'.').map_or(0, |dot| digits_end - dot - 1);
        // In range: a position in 9 bytes.
        (value as f32, i32::try_from(digits).unwrap_or(0))
    };
    Some(Reading {
        value,
        digits,
        unit: unit([packet[digits_end], packet[digits_end + 1]]),
        flags: if s2 == b'U' { MqFlags::UNSTABLE } else { MqFlags::default() },
    })
}

fn unit(bytes: [u8; 2]) -> Unit {
    match &bytes {
        b" G" => Unit::Gram,
        b"CT" => Unit::Carat,
        b"OZ" => Unit::Ounce,
        b"LB" => Unit::Pound,
        b"OT" => Unit::TroyOunce,
        b"DW" => Unit::Pennyweight,
        b"GR" => Unit::Grain,
        b"TL" => Unit::Tael,
        b"MO" => Unit::Momme,
        b"to" => Unit::Tola,
        b" %" => Unit::Percentage,
        b"PC" => Unit::Piece,
        _ => Unit::Unitless,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(packet: &[u8]) -> (f32, i32, Unit, bool) {
        assert_eq!(valid(packet), Some(packet.len()), "{}", packet.escape_ascii());
        let r = parse(packet).expect("a reading");
        (r.value, r.digits, r.unit, r.flags == MqFlags::UNSTABLE)
    }

    #[test]
    fn packets_as_a_real_unit_sent_them() {
        // sigrok-dumps' uart/kern_ew_6200-2nm, each file's first packet.
        assert_eq!(read(b"+0000.00 G S\r\n"), (0.0, 2, Unit::Gram, false));
        assert_eq!(read(b"+000000 PC S\r\n"), (0.0, 0, Unit::Piece, false));
        assert_eq!(read(b"+000000  % U\r\n"), (0.0, 0, Unit::Percentage, true));
        assert_eq!(read(b"+00127.20 G S\r\n"), (127.2, 2, Unit::Gram, false));
        assert_eq!(read(b"+002014.8CT S\r\n"), (2014.8, 1, Unit::Carat, false));
        assert_eq!(read(b"+0026.90 G U\r\n"), (26.9, 2, Unit::Gram, true));
        assert_eq!(read(b"+00635.8CT S\r\n"), (635.8, 1, Unit::Carat, false));
        assert_eq!(read(b"+000636.0CT S\r\n"), (636.0, 1, Unit::Carat, false));
        assert_eq!(read(b"-0000.04 G S\r\n"), (-0.04, 2, Unit::Gram, false));
        assert_eq!(read(b"+0000.00 G E\r\n"), (f32::INFINITY, 0, Unit::Gram, false));
    }

    #[test]
    fn every_unit_of_the_manual() {
        for (bytes, want) in [
            (b" G", Unit::Gram),
            (b"CT", Unit::Carat),
            (b"OZ", Unit::Ounce),
            (b"LB", Unit::Pound),
            (b"OT", Unit::TroyOunce),
            (b"DW", Unit::Pennyweight),
            (b"GR", Unit::Grain),
            (b"TL", Unit::Tael),
            (b"MO", Unit::Momme),
            (b"to", Unit::Tola),
            (b" %", Unit::Percentage),
            (b"PC", Unit::Piece),
        ] {
            assert_eq!(unit(*bytes), want);
        }
        assert_eq!(unit(*b" g"), Unit::Unitless);
        assert_eq!(unit(*b"G "), Unit::Unitless);
    }

    /// Every packet in sigrok-dumps' `uart/kern_ew_6200-2nm`, in all 13 files.
    #[test]
    fn no_packet_of_the_captures_is_refused() {
        for packet in [
            &b"+0000.00 G E\r\n"[..],
            b"+0000.00 G S\r\n",
            b"+0000.07 G U\r\n",
            b"+0000.09 G U\r\n",
            b"+0000.14 G U\r\n",
            b"+0000.75 G U\r\n",
            b"+000000  % U\r\n",
            b"+000000 PC S\r\n",
            b"+000636.0CT S\r\n",
            b"+0009.34 G U\r\n",
            b"+00127.20 G S\r\n",
            b"+002014.8CT S\r\n",
            b"+0026.90 G S\r\n",
            b"+0026.90 G U\r\n",
            b"+0026.98 G U\r\n",
            b"+00402.95 G U\r\n",
            b"+00635.8CT S\r\n",
            b"+0083.90 G U\r\n",
            b"+0127.20 G S\r\n",
            b"+0456.51 G U\r\n",
            b"+0514.69 G U\r\n",
            b"+1097.44 G U\r\n",
            b"+1600.97 G U\r\n",
            b"+1718.30 G U\r\n",
            b"+1733.57 G U\r\n",
            b"+1743.04 G U\r\n",
            b"+1756.20 G U\r\n",
            b"+1761.01 G U\r\n",
            b"+1792.70 G U\r\n",
            b"+1798.32 G U\r\n",
            b"+1800.76 G U\r\n",
            b"+1831.50 G U\r\n",
            b"+1839.02 G U\r\n",
            b"+1848.64 G U\r\n",
            b"+2272.63 G U\r\n",
            b"+3551.75 G U\r\n",
            b"+4318.15 G U\r\n",
            b"+5616.66 G U\r\n",
            b"-0000.04 G S\r\n",
        ] {
            assert_eq!(valid(packet), Some(packet.len()), "{}", packet.escape_ascii());
            assert!(parse(packet).is_some(), "{}", packet.escape_ascii());
        }
    }

    #[test]
    fn a_number_behind_spaces_is_that_number() {
        // The manual's suppressed zeros. C's strtod gives 0 for the first two.
        assert_eq!(read(b"+  127.20 G S\r\n"), (127.2, 2, Unit::Gram, false));
        assert_eq!(read(b"-  127.20 G S\r\n"), (-127.2, 2, Unit::Gram, false));
        assert_eq!(read(b"   127.20 G S\r\n").0, 127.2);
        assert_eq!(read(b"-   12.7 G S\r\n").0, -12.7);
        assert_eq!(read(b"+12.3     G S\r\n").0, 12.3);
        assert_eq!(read(b"-1234567. G S\r\n").0, -1_234_567.0);
    }

    #[test]
    fn a_field_that_is_no_number_is_no_reading() {
        for packet in [
            &b"+-0026.90 G S\r\n"[..],
            b"++0026.90 G S\r\n",
            b"+abcdefgh G S\r\n",
            b"+1.5e1    G S\r\n",
            b"+0x1f     G S\r\n",
            b"+inf      G S\r\n",
            b"-inf      G S\r\n",
            b"+nan      G S\r\n",
            b"+12\x005.7   G S\r\n",
            b"+12.34.56 G S\r\n",
            b"+12 34.56 G S\r\n",
            b"+         G S\r\n",
            b"+.        G S\r\n",
            b"+abc.def G S\r\n",
        ] {
            assert_eq!(valid(packet), Some(packet.len()), "{}", packet.escape_ascii());
            assert_eq!(parse(packet), None, "{}", packet.escape_ascii());
        }
    }

    #[test]
    fn an_error_is_a_reading_whatever_its_digits() {
        assert_eq!(parse(b"+abcdefgh G E\r\n").map(|r| r.value), Some(f32::INFINITY));
        assert_eq!(parse(b"+         G E\r\n").map(|r| r.value), Some(f32::INFINITY));
    }

    #[test]
    fn a_lost_point_makes_a_good_packet_with_no_checksum_to_say_otherwise() {
        assert_eq!(read(b"+0012720 G S\r\n").0, 12720.0);
    }

    #[test]
    fn digits_count_to_the_end_of_the_field() {
        assert_eq!(read(b"+12.3     G S\r\n").1, 5);
        assert_eq!(read(b"+.1234567 G S\r\n").1, 7);
        assert_eq!(read(b"+1234567. G S\r\n").1, 0);
        assert_eq!(read(b"+.123456 G S\r\n").1, 6);
    }

    #[test]
    fn an_error_is_infinite_with_no_digits_whatever_its_sign() {
        assert_eq!(read(b"-00127.20 G E\r\n"), (f32::INFINITY, 0, Unit::Gram, false));
        assert_eq!(read(b"+00127.20CT E\r\n"), (f32::INFINITY, 0, Unit::Carat, false));
    }

    #[test]
    fn the_check() {
        for good in [
            &b"+00127.20 G S\r\n"[..],
            b"-00127.20 G U\r\n",
            b" 00127.20 G E\r\n",
            b"+00127.20 G  \r\n",
            b"+00127.20 GLS\r\n",
            b"+00127.20 GGS\r\n",
            b"+00127.20 GHS\r\n",
            b"+0127.20 G S\r\n",
            b"+??????????  \r\n",
        ] {
            assert_eq!(valid(good), Some(good.len()), "{}", good.escape_ascii());
        }
        for bad in [
            &b"*00127.20 G S\r\n"[..],
            b"+00127.20 GXS\r\n",
            b"+00127.20 G X\r\n",
            b"+00127.20 G S\n\r",
            b"+00127.20 G S\r\r",
            b"+00127.20 G S\n\n",
            b"*0127.20 G S\r\nx",
            b"+0127.20 GXS\r\nx",
            b"+0127.20 G X\r\nx",
            b"+0127.20 G S\n\rx",
        ] {
            assert_eq!(valid(bad), None, "{}", bad.escape_ascii());
        }
    }

    #[test]
    fn a_short_packet_needs_no_15th_byte_and_a_long_one_does() {
        assert_eq!(head(b"+0026.90 G S\r\n"), Head::Packet(14));
        assert_eq!(head(b"+0026.90 G S\r\n+"), Head::Packet(14));
        assert_eq!(head(b"+00127.20 G S\r"), Head::Wait, "the LF is on its way");
        assert_eq!(head(b"+00127.20 G S\r\n"), Head::Packet(15));
        assert_eq!(head(b"+0026.90 G"), Head::Wait);
    }

    #[test]
    fn what_starts_no_packet_is_stepped_over() {
        // A line's end in the short place with a bad head can't start a long one.
        assert_eq!(head(b"*0026.90 G S\r\n"), Head::Skip);
        assert_eq!(head(b"*00127.20 G S\r\n"), Head::Skip);
        assert_eq!(head(b"0123456789abcdef"), Head::Skip);
    }

    #[test]
    fn a_stream_of_14_byte_packets_gives_each() {
        // C reads every second one, as it steps 15.
        let mut buf = Vec::new();
        for i in 0..6 {
            buf.extend_from_slice(format!("+0000.0{i} G S\r\n").as_bytes());
        }
        let mut at = 0;
        let mut got = Vec::new();
        loop {
            match head(&buf[at..]) {
                Head::Packet(n) => {
                    got.push(parse(&buf[at..at + n]).expect("a reading").value);
                    at += n;
                }
                Head::Skip => at += 1,
                Head::Wait => break,
            }
        }
        assert_eq!(got, [0.0, 0.01, 0.02, 0.03, 0.04, 0.05]);
        assert_eq!(at, buf.len());
    }
}
