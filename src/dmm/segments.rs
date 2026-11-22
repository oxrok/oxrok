// Counterpart in sigrok:
//   libsigrok/src/dmm/rs9lcd.c,
//   libsigrok/src/dmm/bm25x.c,
//   libsigrok/src/dmm/qm1578.c,
//   Copyright (C) 2012 Alexandru Gagniuc <mr.nuke.me@gmail.com>,
//   Copyright (C) 2014 Janne Huttunen <jahuttun@gmail.com>,
//   Copyright (C) 2021 Mike McCauley <mikem@airspayce.com>,
//   with commits by Uwe Hermann, Bert Vermeulen, Daniel Elstner,
//   Gerhard Sittig, Martin Ling, Aurelien Jacobs, Frank Stettner
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! Binary packets with seven segment digits: the RadioShack 22-812,
//! the Brymen BM25x and the Digitech QM1578.

use crate::core::{Mq, MqFlags, Unit};
use crate::dmm::{Reading, count, scaled};

/// RS 22-812: 9 bytes.
pub const RS9LCD_SIZE: usize = 9;
/// BM25x: 15 bytes.
pub const BM25X_SIZE: usize = 15;
/// QM1578: 15 bytes.
pub const QM1578_SIZE: usize = 15;

const RS_MODE_INVALID: u8 = 26;

fn rs_digit(raw: u8) -> Option<u32> {
    Some(match raw & !0x08 {
        0x00 | 0xd7 => 0,
        0x50 => 1,
        0xb5 => 2,
        0xf1 => 3,
        0x72 => 4,
        0xe3 => 5,
        0xe7 => 6,
        0x51 => 7,
        0xf7 => 8,
        0xf3 => 9,
        _ => return None,
    })
}

/// True for a RadioShack 22-812 packet: a known mode, the checksum
/// (sum of the bytes plus 57) and one multiplier and one unit at most.
pub fn rs9lcd_valid(buf: &[u8]) -> bool {
    if buf.len() < RS9LCD_SIZE || buf[0] >= RS_MODE_INVALID {
        return false;
    }
    let sum = buf[..8].iter().fold(57u8, |s, b| s.wrapping_add(*b));
    if sum != buf[8] {
        return false;
    }
    let (ind1, ind2) = (buf[1], buf[2]);
    count(&[
        ind1 & 0x20 != 0,
        ind1 & 0x10 != 0,
        ind1 & 0x01 != 0,
        ind2 & 0x80 != 0,
        ind2 & 0x40 != 0,
    ]) <= 1
        && count(&[
            ind1 & 0x80 != 0,
            ind1 & 0x40 != 0,
            ind1 & 0x08 != 0,
            ind1 & 0x04 != 0,
            ind1 & 0x02 != 0,
            ind2 & 0x20 != 0,
            ind2 & 0x10 != 0,
            ind2 & 0x08 != 0,
            ind2 & 0x04 != 0,
        ]) <= 1
}

/// The number on the display; `skip_last` leaves out the last digit
/// (the temperature unit letter).
fn rs_value(buf: &[u8], skip_last: bool) -> (f32, i32) {
    let mut value = 0.0f32;
    let mut exponent = 0;
    let mut point = false;
    let end = usize::from(skip_last);
    let mut nan = false;
    for i in (end..4).rev() {
        let raw = buf[3 + (3 - i)];
        let Some(d) = rs_digit(raw) else {
            nan = true;
            break;
        };
        if i < 3 && raw & 0x08 != 0 {
            point = true;
        }
        if point {
            exponent -= 1;
        }
        value = value * 10.0 + d as f32;
    }
    if nan {
        value = f32::NAN;
    }
    if buf[7] & 0x08 != 0 {
        value = -value;
    }
    let (ind1, ind2) = (buf[1], buf[2]);
    if ind2 & 0x40 != 0 {
        exponent -= 9;
    } else if ind2 & 0x80 != 0 {
        exponent -= 6;
    } else if ind1 & 0x01 != 0 {
        exponent -= 3;
    } else if ind1 & 0x20 != 0 {
        exponent += 3;
    } else if ind1 & 0x10 != 0 {
        exponent += 6;
    }
    (value, exponent)
}

/// Parses a RadioShack 22-812 packet.
pub fn rs9lcd_parse(buf: &[u8]) -> Option<Reading> {
    if !rs9lcd_valid(buf) {
        return None;
    }
    let (mut value, mut exponent) = rs_value(buf, false);
    let mut r = Reading::new(0.0, Mq::Gain, Unit::Unitless, 0);
    let digit2 = buf[5] & !0x08;
    match buf[0] {
        0 => {
            (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
            r.flags |= MqFlags::DC;
        }
        1 => {
            (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
            r.flags |= MqFlags::AC;
        }
        2..=4 => {
            (r.mq, r.unit) = (Mq::Current, Unit::Ampere);
            r.flags |= MqFlags::DC;
        }
        5..=7 => {
            (r.mq, r.unit) = (Mq::Current, Unit::Ampere);
            r.flags |= MqFlags::AC;
        }
        8 => (r.mq, r.unit) = (Mq::Resistance, Unit::Ohm),
        9 => (r.mq, r.unit) = (Mq::Capacitance, Unit::Farad),
        20 => {
            (r.mq, r.unit) = (Mq::Continuity, Unit::Boolean);
            value = if digit2 == 0x66 { 1.0 } else { 0.0 };
        }
        19 => {
            (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
            r.flags |= MqFlags::DIODE | MqFlags::DC;
        }
        10..=12 => (r.mq, r.unit) = (Mq::Frequency, Unit::Hertz),
        22 => {
            r.mq = Mq::Voltage;
            if value.is_nan() {
                r.unit = Unit::Boolean;
                value = if digit2 == 0x76 { 1.0 } else { 0.0 };
            } else {
                r.unit = Unit::Volt;
            }
        }
        21 => (r.mq, r.unit) = (Mq::Gain, Unit::Unitless),
        13..=15 => (r.mq, r.unit) = (Mq::DutyCycle, Unit::Percentage),
        16..=18 => (r.mq, r.unit) = (Mq::PulseWidth, Unit::Second),
        25 => {
            r.mq = Mq::Temperature;
            (value, exponent) = rs_value(buf, true);
            r.unit = if buf[3] & !0x08 == 0x87 { Unit::Celsius } else { Unit::Fahrenheit };
        }
        23 => {
            (r.mq, r.unit) = (Mq::Power, Unit::DecibelMw);
            r.flags |= MqFlags::AC;
        }
        _ => {}
    }
    if buf[7] & 0x10 != 0 {
        r.flags |= MqFlags::HOLD;
    }
    if buf[3] & 0x08 != 0 {
        r.flags |= MqFlags::MAX;
    }
    if buf[2] & 0x01 != 0 {
        r.flags |= MqFlags::MIN;
    }
    if buf[7] & 0x01 != 0 {
        r.flags |= MqFlags::AUTORANGE;
    }
    (r.value, r.digits) = scaled(value, exponent);
    Some(r)
}

/// True for a Brymen BM25x packet: a leading 2 then sync nibbles.
pub fn bm25x_valid(buf: &[u8]) -> bool {
    buf.len() >= BM25X_SIZE && buf[0] == 2 && (1..BM25X_SIZE).all(|i| usize::from(buf[i] >> 4) == i)
}

/// A BM25x digit as a number or a letter (`b' '`, `b'-'`, `b'L'`,
/// `b'C'`, `b'F'`, ...).
fn bm_digit(num: usize, buf: &[u8]) -> Option<u8> {
    let v = (buf[3 + 2 * num] & 0x0e) | (buf[4 + 2 * num] << 4);
    Some(match v {
        0xbe => 0,
        0xa0 => 1,
        0xda => 2,
        0xf8 => 3,
        0xe4 => 4,
        0x7c => 5,
        0x7e => 6,
        0xa8 => 7,
        0xfe => 8,
        0xfc => 9,
        0x00 => b' ',
        0x40 => b'-',
        0x16 => b'L',
        0x1e => b'C',
        0x4e => b'F',
        0x5e => b'E',
        0x62 => b'n',
        0x42 => b'r',
        _ => return None,
    })
}

/// Parses a Brymen BM25x packet.
pub fn bm25x_parse(buf: &[u8]) -> Option<Reading> {
    if !bm25x_valid(buf) {
        return None;
    }
    let mut r = Reading::new(0.0, Mq::Gain, Unit::Unitless, 0);
    if buf[1] & 8 != 0 {
        r.flags |= MqFlags::AUTORANGE;
    }
    if buf[1] & 4 != 0 {
        r.flags |= MqFlags::DC;
    }
    if buf[1] & 2 != 0 {
        r.flags |= MqFlags::AC;
    }
    if buf[1] & 1 != 0 {
        r.flags |= MqFlags::RELATIVE;
    }
    if buf[11] & 8 != 0 {
        r.flags |= MqFlags::HOLD;
    }
    if buf[13] & 8 != 0 {
        r.flags |= MqFlags::MAX;
    }
    if buf[14] & 8 != 0 {
        r.flags |= MqFlags::MIN;
    }
    if buf[14] & 4 != 0 {
        (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
        if !r.flags.contains(MqFlags::DC) && !r.flags.contains(MqFlags::AC) {
            r.flags |= MqFlags::DIODE | MqFlags::DC;
        }
    }
    if buf[14] & 2 != 0 {
        (r.mq, r.unit) = (Mq::Current, Unit::Ampere);
    }
    if buf[12] & 4 != 0 {
        (r.mq, r.unit) = (Mq::Resistance, Unit::Ohm);
    }
    if buf[13] & 4 != 0 {
        (r.mq, r.unit) = (Mq::Capacitance, Unit::Farad);
    }
    if buf[12] & 2 != 0 {
        (r.mq, r.unit) = (Mq::Frequency, Unit::Hertz);
    }
    let last = bm_digit(3, buf);
    if last == Some(b'C') {
        (r.mq, r.unit) = (Mq::Temperature, Unit::Celsius);
    }
    if last == Some(b'F') {
        (r.mq, r.unit) = (Mq::Temperature, Unit::Fahrenheit);
    }
    // The value: up to four digits, fewer when the last is a unit
    // letter; the decimal point position gives the exponent.
    let mut value = 0.0f32;
    let mut ndigits = 0;
    let mut special = false;
    for i in 0..4 {
        let d = bm_digit(i, buf);
        if i == 3 && matches!(d, Some(b'C' | b'F')) {
            break;
        }
        match d {
            Some(d) if d <= 9 => value = value * 10.0 + f32::from(d),
            _ => {
                special = true;
                break;
            }
        }
        ndigits += 1;
    }
    let mut exponent = 0;
    if special {
        value = if bm_digit(1, buf) == Some(0) && bm_digit(2, buf) == Some(b'L') {
            f32::INFINITY
        } else {
            f32::NAN
        };
    } else {
        let mut point = 0;
        for i in 1..4 {
            if buf[11 - 2 * i] & 1 != 0 {
                point = if point == 0 { i } else { 0 };
            }
        }
        let pos = if point == 0 { 0 } else { point as i32 + ndigits - 4 };
        if (0..=3).contains(&pos) {
            exponent = -pos;
        }
    }
    if buf[11] & 2 != 0 {
        exponent += 6;
    } else if buf[11] & 1 != 0 {
        exponent += 3;
    } else if buf[13] & 1 != 0 {
        exponent -= 3;
    } else if buf[13] & 2 != 0 {
        exponent -= 6;
    } else if buf[12] & 1 != 0 {
        exponent -= 9;
    }
    let (v, digits) = scaled(value, exponent);
    r.value = if buf[3] & 1 != 0 { -v } else { v };
    r.digits = digits;
    Some(r)
}

/// True for a QM1578 packet: the record separator at the end.
pub fn qm1578_valid(buf: &[u8]) -> bool {
    buf.len() >= QM1578_SIZE && buf[14] == 0x0d
}

/// Parses a QM1578 packet.
pub fn qm1578_parse(buf: &[u8]) -> Option<Reading> {
    if !qm1578_valid(buf) {
        return None;
    }
    let mut r = Reading::new(0.0, Mq::Gain, Unit::Unitless, 0);
    let s = buf[13];
    if s & 0x10 != 0 {
        r.flags |= MqFlags::AUTORANGE;
    }
    if s & 0x40 != 0 {
        r.flags |= MqFlags::DC;
    }
    if s & 0x80 != 0 {
        r.flags |= MqFlags::AC;
    }
    if s & 0x20 != 0 {
        r.flags |= MqFlags::RELATIVE;
    }
    if buf[12] & 0x40 != 0 {
        r.flags |= MqFlags::HOLD;
    }
    if s & 0x0c == 0x0c {
        r.flags |= MqFlags::MAX | MqFlags::AVG;
    }
    if s & 0x0c == 0x08 {
        r.flags |= MqFlags::MIN;
    }
    match buf[4] {
        0x01 => {
            (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
            r.flags |= MqFlags::AC | MqFlags::RMS;
        }
        0x02 => {
            (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
            r.flags |= MqFlags::DC;
        }
        0x04 => (r.mq, r.unit) = (Mq::Resistance, Unit::Ohm),
        0x05 => (r.mq, r.unit) = (Mq::Capacitance, Unit::Farad),
        0x06 => {
            r.mq = Mq::Temperature;
            r.unit = if buf[10] == 0x08 { Unit::Celsius } else { Unit::Fahrenheit };
        }
        0x07..=0x09 => {
            (r.mq, r.unit) = (Mq::Current, Unit::Ampere);
            r.flags |= MqFlags::DC;
        }
        0x0c..=0x0e => {
            (r.mq, r.unit) = (Mq::Current, Unit::Ampere);
            r.flags |= MqFlags::AC | MqFlags::RMS;
        }
        0x0f => {
            (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
            r.flags |= MqFlags::DIODE;
        }
        0x10 if buf[10] == 0x04 => (r.mq, r.unit) = (Mq::Frequency, Unit::Hertz),
        0x10 => (r.mq, r.unit) = (Mq::DutyCycle, Unit::Percentage),
        0x20 => (r.mq, r.unit) = (Mq::Continuity, Unit::Ohm),
        _ => {}
    }
    let value = if buf[8] == 0x0b {
        f32::INFINITY
    } else {
        let mut v = 0.0f32;
        for i in 0..4 {
            let d = buf[8 - i];
            if d <= 9 {
                v = v * 10.0 + f32::from(d);
            }
        }
        v
    };
    let prefix = match buf[11] {
        0x00 => 0,
        0x01 => 3,
        0x02 => 6,
        0x03 => -9,
        0x04 => -6,
        0x05 | 0x06 => -3,
        _ => -1,
    };
    let (v, digits) = scaled(value, prefix - i32::from(buf[9]));
    r.value = if buf[12] & 0x80 != 0 { -v } else { v };
    r.digits = digits;
    Some(r)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rs_packet(mode: u8, i1: u8, i2: u8, digits: [u8; 4], info: u8) -> [u8; 9] {
        let mut p = [mode, i1, i2, digits[0], digits[1], digits[2], digits[3], info, 0];
        p[8] = p[..8].iter().fold(57u8, |s, b| s.wrapping_add(*b));
        p
    }

    #[test]
    fn rs9lcd_packets() {
        // DC V, 1.234: digit4 = 1, digit3 = 2 with point, digit2 = 3,
        // digit1 = 4.
        let p = rs_packet(0, 0x02, 0, [0x50, 0xb5 | 0x08, 0xf1, 0x72], 0x01);
        assert!(rs9lcd_valid(&p));
        let r = rs9lcd_parse(&p).unwrap();
        assert_eq!((r.mq, r.unit, r.digits), (Mq::Voltage, Unit::Volt, 3));
        assert!((r.value - 1.234).abs() < 1e-5);
        assert!(r.flags.contains(MqFlags::DC | MqFlags::AUTORANGE));
        // Bad checksum.
        let mut bad = p;
        bad[8] ^= 1;
        assert!(!rs9lcd_valid(&bad));
        // kOhm, negative, 12.34 -> -12340 Ohm.
        let p = rs_packet(8, 0x40 | 0x20, 0, [0x50, 0xb5 | 0x08, 0xf1, 0x72], 0x08);
        let r = rs9lcd_parse(&p).unwrap();
        assert_eq!((r.mq, r.digits), (Mq::Resistance, 0));
        assert!(
            (r.value + 1234.0 * 10.0 / 10.0).abs() < 1e-2
                || (r.value + 12340.0 / 10.0).abs() < 1e-2
        );
        // Temperature in Celsius: digits "25C" with the last as C.
        let p = rs_packet(25, 0, 0, [0x87, 0xb5, 0xe3, 0], 0);
        let r = rs9lcd_parse(&p).unwrap();
        assert_eq!((r.mq, r.unit), (Mq::Temperature, Unit::Celsius));
    }

    #[test]
    fn bm25x_packet() {
        // Build a packet: byte 0 = 2, bytes 1..15 with sync nibbles.
        // Digits 1 2 3 4 (codes a0 da f8 e4) at nibble pairs 3/4, 5/6,
        // 7/8, 9/10; point after the first digit (byte 9 bit 0 is for
        // i=1 -> byte 11-2 = 9); volt (byte 14 bit 2), DC (byte 1 bit 2).
        let codes = [0xa0u8, 0xda, 0xf8, 0xe4];
        let mut p = vec![2u8];
        for i in 1..15 {
            p.push((i as u8) << 4);
        }
        for (n, c) in codes.iter().enumerate() {
            p[3 + 2 * n] |= c & 0x0e;
            p[4 + 2 * n] |= c >> 4;
        }
        p[5] |= 1;
        p[14] |= 4;
        p[1] |= 4;
        assert!(bm25x_valid(&p));
        let r = bm25x_parse(&p).unwrap();
        assert_eq!((r.mq, r.unit, r.digits), (Mq::Voltage, Unit::Volt, 3));
        assert!((r.value - 1.234).abs() < 1e-5, "{}", r.value);
        assert!(r.flags.contains(MqFlags::DC));
    }

    #[test]
    fn qm1578_packet() {
        // DC V, digits 1 2 3 4 (buf[8] is the most significant), 3
        // decimals, no prefix, DC and auto flags.
        let mut p = [0u8; 15];
        p[4] = 0x02;
        p[8] = 1;
        p[7] = 2;
        p[6] = 3;
        p[5] = 4;
        p[9] = 3;
        p[13] = 0x50;
        p[14] = 0x0d;
        assert!(qm1578_valid(&p));
        let r = qm1578_parse(&p).unwrap();
        assert_eq!((r.mq, r.digits), (Mq::Voltage, 3));
        assert!((r.value - 1.234).abs() < 1e-5);
        assert!(r.flags.contains(MqFlags::DC | MqFlags::AUTORANGE));
        p[8] = 0x0b;
        assert!(qm1578_parse(&p).unwrap().value.is_infinite());
    }
}

// UT372 tachometer: libsigrok/src/dmm/ut372.c, Copyright (C) 2014
// Martin Ling <martin-sigrok@earth.li>.

/// UT372: 27 bytes, hex digit pairs.
pub const UT372_SIZE: usize = 27;

const UT372_DIGITS: [u8; 10] = [0x7b, 0x60, 0x5e, 0x7c, 0x65, 0x3d, 0x3f, 0x70, 0x7f, 0x7d];

/// Two ASCII hex digits, with letters past `9` taken as `A` to `F`.
fn ut372_pair(b: &[u8]) -> u8 {
    let nibble = |c: u8| {
        // Wrapping, as the C `char` does: a byte near 0xff is no digit
        // either way, and counts as 0.
        let c = if c > 0x39 { c.wrapping_add(7) } else { c };
        u8::from_str_radix(std::str::from_utf8(&[c]).unwrap_or("0"), 16).unwrap_or(0)
    };
    nibble(b[0]) << 4 | nibble(b[1])
}

/// True for a UT372 packet: CR LF at the end and the RPM or count
/// mode lit (neither means the setup menu).
pub fn ut372_valid(buf: &[u8]) -> bool {
    buf.len() >= UT372_SIZE
        && buf[25] == b'\r'
        && buf[26] == b'\n'
        && ut372_pair(&buf[23..25]) & 0x03 != 0
}

/// The reading of a UT372 packet: revolutions per minute or a count.
pub fn ut372_parse(buf: &[u8]) -> Option<Reading> {
    if !ut372_valid(buf) {
        return None;
    }
    let flags1 = ut372_pair(&buf[21..23]);
    let flags2 = ut372_pair(&buf[23..25]);
    let mut r = if flags2 & 0x01 != 0 {
        Reading::new(0.0, Mq::Frequency, Unit::RevolutionsPerMinute, 0)
    } else {
        Reading::new(0.0, Mq::Count, Unit::Unitless, 0)
    };
    if flags1 & 0x04 != 0 {
        r.flags |= MqFlags::HOLD;
    }
    if flags2 & 0x20 != 0 {
        r.flags |= MqFlags::MIN;
    }
    if flags2 & 0x10 != 0 {
        r.flags |= MqFlags::MAX;
    }
    if flags2 & 0x40 != 0 {
        r.flags |= MqFlags::AVG;
    }
    let mut value = 0u32;
    let mut exponent = 0i32;
    for i in 0..5 {
        let seg = ut372_pair(&buf[1 + 2 * i..3 + 2 * i]);
        if let Some(d) = UT372_DIGITS.iter().position(|s| *s == seg & 0x7f) {
            value += d as u32 * 10u32.pow(i as u32);
        }
        if seg & 0x80 != 0 {
            exponent = -(i as i32);
        }
    }
    (r.value, r.digits) = scaled(value as f32, exponent);
    Some(r)
}

#[cfg(test)]
mod ut372_tests {
    use super::*;

    #[test]
    fn ut372_packet() {
        // Digits 1234.5: pairs run from the least significant digit,
        // the point sits on the tens pair (0x80), RPM mode with max.
        // Hex digits above 9 come as the characters after '9'.
        let pair = |b: u8| [b'0' + (b >> 4), b'0' + (b & 0x0f)];
        let mut p = Vec::new();
        p.push(b'0');
        for seg in [0x3d_u8, 0xe5, 0x7c, 0x5e, 0x60] {
            p.extend(pair(seg));
        }
        p.extend(b"0000000000");
        p.extend(b"00");
        p.extend(b"11");
        p.extend(b"\r\n");
        assert_eq!(p.len(), UT372_SIZE);
        assert!(ut372_valid(&p));
        let r = ut372_parse(&p).unwrap();
        assert_eq!((r.mq, r.unit, r.digits), (Mq::Frequency, Unit::RevolutionsPerMinute, 1));
        assert!((r.value - 1234.5).abs() < 1e-3);
        assert!(r.flags.contains(MqFlags::MAX));
        // Setup menu: neither mode bit.
        p[23] = b'0';
        p[24] = b'0';
        assert!(!ut372_valid(&p));
    }

    /// Bytes that are no hex digit, up to 0xff, count as 0 and do not
    /// overflow the step from `9` to `A`.
    #[test]
    fn ut372_bytes_that_are_no_digits() {
        assert_eq!(ut372_pair(&[0xff, 0xf9]), 0);
        assert_eq!(ut372_pair(b":?"), 0xaf);
        let mut p = [0xfe_u8; UT372_SIZE];
        p[23..].copy_from_slice(b"11\r\n");
        assert!(ut372_valid(&p));
        let _ = ut372_parse(&p);
    }
}
