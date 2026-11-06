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
//! The manual says leading zeros are suppressed with spaces, and the captures
//! are all zero filled. `strtod` takes no space behind a sign, so `+  127.20` is
//! 0 in C and here (`scale/kern.c:63`). `   127.20` reads as 127.2.

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

/// `sr_kern_parse`, on a packet [`valid`] took.
pub fn parse(packet: &[u8]) -> Reading {
    let len = packet.len();
    let digits_end = len - 6;
    let s2 = packet[len - 3];
    let (value, digits) = if s2 == b'E' {
        log::trace!("Over/under limit.");
        (f32::INFINITY, 0)
    } else {
        // As a C string: a NUL ends it.
        let text = &packet[..digits_end];
        let text = &text[..text.iter().position(|&b| b == 0).unwrap_or(text.len())];
        let value = g_ascii_strtod(text).unwrap_or(0.0);
        let digits = text.iter().position(|&b| b == b'.').map_or(0, |dot| digits_end - dot - 1);
        // In range: a position in 9 bytes.
        (value as f32, i32::try_from(digits).unwrap_or(0))
    };
    Reading {
        value,
        digits,
        unit: unit([packet[digits_end], packet[digits_end + 1]]),
        flags: if s2 == b'U' { MqFlags::UNSTABLE } else { MqFlags::default() },
    }
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
        let r = parse(packet);
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

    #[test]
    fn what_strtod_makes_of_the_digits() {
        // No space behind a sign, as the manual's suppressed zeros would have.
        assert_eq!(read(b"+  127.20 G S\r\n").0, 0.0);
        assert_eq!(read(b"-  127.20 G S\r\n").0, 0.0);
        assert_eq!(read(b"   127.20 G S\r\n").0, 127.2);
        assert_eq!(read(b"+1.5e1    G S\r\n").0, 15.0);
        assert_eq!(read(b"+0x1f     G S\r\n").0, 31.0);
        assert_eq!(read(b"-inf      G S\r\n").0, f32::NEG_INFINITY);
        assert!(read(b"+nan      G S\r\n").0.is_nan());
        assert_eq!(read(b"+1e99     G S\r\n").0, f32::INFINITY);
        assert_eq!(read(b"+abcdefgh G S\r\n").0, 0.0);
    }

    #[test]
    fn a_nul_ends_the_number_and_the_search_for_a_point() {
        assert_eq!(read(b"+12\x005.7   G S\r\n"), (12.0, 0, Unit::Gram, false));
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
                    got.push(parse(&buf[at..at + n]).value);
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
