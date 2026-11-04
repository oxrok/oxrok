// Counterpart in sigrok:
//   libsigrok/src/dmm/fs9721.c,
//   libsigrok/src/dmm/fs9922.c,
//   libsigrok/src/dmm/dtm0660.c,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2012 Alexandru Gagniuc <mr.nuke.me@gmail.com>,
//   Copyright (C) 2015 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Bert Vermeulen, Gerhard Sittig, Daniel Elstner,
//   Martin Ling, Aurelien Jacobs, Frank Stettner
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! The Fortune Semiconductor FS9721 and FS9922 and the Dream Tech
//! DTM0660 chipsets: packets that mirror the LCD, nibble by nibble,
//! with a sync nibble per byte.

use crate::core::{Mq, MqFlags, Unit};
use crate::dmm::{Prefix, Reading, count, scaled};

/// What the FS9721's four user defined LCD symbols mean on a given
/// meter.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Fs9721Symbols {
    /// Unknown, ignored.
    #[default]
    None,
    /// Symbol 00 is temperature in Celsius.
    Temp00C,
    /// Symbol 01 is temperature in Celsius.
    Temp01C,
    /// Symbol 10 is temperature in Celsius.
    Temp10C,
    /// Symbol 01 is Fahrenheit, symbol 10 Celsius.
    Temp01F10C,
    /// Symbol 00 is MAX, 01 Celsius, 11 MIN.
    MaxCMin,
}

/// FS9721: 14 bytes.
pub const FS9721_SIZE: usize = 14;
/// FS9922: 14 bytes.
pub const FS9922_SIZE: usize = 14;
/// DTM0660: 15 bytes.
pub const DTM0660_SIZE: usize = 15;

/// Bytes `1..=8` hold four digits as nibble pairs; `mask` strips the
/// bit that is not part of a digit.
fn nibble_digits(buf: &[u8], mask: u8) -> [u8; 4] {
    let mut d = [0u8; 4];
    for (i, digit) in d.iter_mut().enumerate() {
        *digit = ((buf[1 + i * 2] & 0x0f) << 4 | (buf[2 + i * 2] & 0x0f)) & !mask;
    }
    d
}

fn sync_nibbles_valid(buf: &[u8], size: usize) -> bool {
    buf.len() >= size
        && buf.iter().take(size).enumerate().all(|(i, b)| usize::from(b >> 4) == i + 1)
}

#[derive(Clone, Copy, Debug, Default)]
struct LcdFlags {
    prefix: Prefix,
    ac: bool,
    dc: bool,
    auto: bool,
    rs232: bool,
    diode: bool,
    percent: bool,
    beep: bool,
    farad: bool,
    ohm: bool,
    rel: bool,
    hold: bool,
    ampere: bool,
    volt: bool,
    hz: bool,
    symbols: [bool; 4],
    degc: bool,
    degf: bool,
    min: bool,
    max: bool,
}

impl LcdFlags {
    fn valid(&self) -> bool {
        self.prefix.is_single()
            && count(&[self.hz, self.ohm, self.farad, self.ampere, self.volt, self.percent]) <= 1
            && !(self.ac && self.dc)
            && self.rs232
    }

    fn apply(&self, value: f32, exponent: i32) -> Reading {
        let (v, digits) = scaled(value, exponent + self.prefix.exponent());
        let mut r = Reading::new(v, Mq::Gain, Unit::Unitless, digits);
        if self.volt {
            (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
        }
        if self.ampere {
            (r.mq, r.unit) = (Mq::Current, Unit::Ampere);
        }
        if self.ohm {
            (r.mq, r.unit) = (Mq::Resistance, Unit::Ohm);
        }
        if self.hz {
            (r.mq, r.unit) = (Mq::Frequency, Unit::Hertz);
        }
        if self.farad {
            (r.mq, r.unit) = (Mq::Capacitance, Unit::Farad);
        }
        if self.beep {
            (r.mq, r.unit) = (Mq::Continuity, Unit::Boolean);
            r.value = if r.value.is_infinite() { 0.0 } else { 1.0 };
        }
        if self.diode {
            (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
        }
        if self.percent {
            (r.mq, r.unit) = (Mq::DutyCycle, Unit::Percentage);
        }
        if self.degc {
            (r.mq, r.unit) = (Mq::Temperature, Unit::Celsius);
        }
        if self.degf {
            (r.mq, r.unit) = (Mq::Temperature, Unit::Fahrenheit);
        }
        if self.ac {
            r.flags |= MqFlags::AC;
        }
        if self.dc {
            r.flags |= MqFlags::DC;
        }
        if self.auto {
            r.flags |= MqFlags::AUTORANGE;
        }
        if self.diode {
            r.flags |= MqFlags::DIODE | MqFlags::DC;
        }
        if self.hold {
            r.flags |= MqFlags::HOLD;
        }
        if self.rel {
            r.flags |= MqFlags::RELATIVE;
        }
        if self.min {
            r.flags |= MqFlags::MIN;
        }
        if self.max {
            r.flags |= MqFlags::MAX;
        }
        r
    }
}

fn bit(b: u8, n: u8) -> bool {
    b & (1 << n) != 0
}

/// FS9721 flags: the symbol bits sit in the low nibble, high bit first.
fn fs9721_flags(buf: &[u8]) -> LcdFlags {
    LcdFlags {
        ac: bit(buf[0], 3),
        dc: bit(buf[0], 2),
        auto: bit(buf[0], 1),
        rs232: bit(buf[0], 0),
        prefix: Prefix {
            micro: bit(buf[9], 3),
            nano: bit(buf[9], 2),
            kilo: bit(buf[9], 1),
            milli: bit(buf[10], 3),
            mega: bit(buf[10], 1),
            pico: false,
        },
        diode: bit(buf[9], 0),
        percent: bit(buf[10], 2),
        beep: bit(buf[10], 0),
        farad: bit(buf[11], 3),
        ohm: bit(buf[11], 2),
        rel: bit(buf[11], 1),
        hold: bit(buf[11], 0),
        ampere: bit(buf[12], 3),
        volt: bit(buf[12], 2),
        hz: bit(buf[12], 1),
        // Symbols 00, 01, 10, 11.
        symbols: [bit(buf[13], 0), bit(buf[13], 1), bit(buf[13], 2), bit(buf[13], 3)],
        ..Default::default()
    }
}

/// DTM0660 flags: the same layout with the nibble bits reversed, and a
/// fifteenth byte.
fn dtm0660_flags(buf: &[u8]) -> LcdFlags {
    LcdFlags {
        ac: bit(buf[0], 0),
        dc: bit(buf[0], 1),
        auto: bit(buf[0], 2),
        rs232: bit(buf[0], 3),
        prefix: Prefix {
            micro: bit(buf[9], 0),
            nano: bit(buf[9], 1),
            kilo: bit(buf[9], 2),
            milli: bit(buf[10], 0),
            mega: bit(buf[10], 2),
            pico: false,
        },
        diode: bit(buf[9], 3),
        percent: bit(buf[10], 1),
        beep: bit(buf[10], 3),
        farad: bit(buf[11], 0),
        ohm: bit(buf[11], 1),
        rel: bit(buf[11], 2),
        hold: bit(buf[11], 3),
        ampere: bit(buf[12], 0),
        volt: bit(buf[12], 1),
        hz: bit(buf[12], 2),
        degf: bit(buf[13], 0),
        degc: bit(buf[13], 1),
        symbols: [bit(buf[13], 2), bit(buf[13], 3), false, false],
        min: bit(buf[14], 1),
        max: bit(buf[14], 3),
    }
}

fn fs9721_digit(b: u8) -> Option<u32> {
    Some(match b {
        0x7d => 0,
        0x05 => 1,
        0x5b => 2,
        0x1f => 3,
        0x27 => 4,
        0x3e => 5,
        0x7e => 6,
        0x15 => 7,
        0x7f => 8,
        0x3f => 9,
        _ => return None,
    })
}

fn dtm0660_digit(b: u8) -> Option<u32> {
    Some(match b {
        0xeb => 0,
        0x0a => 1,
        0xad => 2,
        0x8f => 3,
        0x4e => 4,
        0xc7 => 5,
        0xe7 => 6,
        0x8a => 7,
        0xef => 8,
        0xcf => 9,
        _ => return None,
    })
}

/// The value and the decimal exponent of a nibble packet. `point_bit`
/// is the bit of bytes 3, 5 and 7 that marks the decimal point.
fn nibble_value(
    buf: &[u8],
    digit_mask: u8,
    digit: fn(u8) -> Option<u32>,
    over: [u8; 4],
    sign_bit: u8,
    point_bit: u8,
) -> Option<(f32, i32)> {
    let digits = nibble_digits(buf, digit_mask);
    if digits == over {
        return Some((f32::INFINITY, 0));
    }
    let mut value = 0u32;
    for d in digits {
        value = value * 10 + digit(d)?;
    }
    let exponent = if bit(buf[3], point_bit) {
        -3
    } else if bit(buf[5], point_bit) {
        -2
    } else if bit(buf[7], point_bit) {
        -1
    } else {
        0
    };
    let sign = if bit(buf[1], sign_bit) { -1.0 } else { 1.0 };
    Some((value as f32 * sign, exponent))
}

/// True for an FS9721 packet.
pub fn fs9721_valid(buf: &[u8]) -> bool {
    sync_nibbles_valid(buf, FS9721_SIZE) && fs9721_flags(buf).valid()
}

/// Parses an FS9721 packet, with the meter's meaning of the user
/// symbols.
pub fn fs9721_parse(buf: &[u8], symbols: Fs9721Symbols) -> Option<Reading> {
    if buf.len() < FS9721_SIZE {
        return None;
    }
    let (value, exponent) = nibble_value(buf, 0x80, fs9721_digit, [0x00, 0x7d, 0x68, 0x00], 3, 3)?;
    let flags = fs9721_flags(buf);
    let mut r = flags.apply(value, exponent);
    let [s00, s01, s10, s11] = flags.symbols;
    let temp = |r: &mut Reading, unit| (r.mq, r.unit) = (Mq::Temperature, unit);
    match symbols {
        Fs9721Symbols::None => {}
        Fs9721Symbols::Temp00C if s00 => temp(&mut r, Unit::Celsius),
        Fs9721Symbols::Temp01C if s01 => temp(&mut r, Unit::Celsius),
        Fs9721Symbols::Temp10C if s10 => temp(&mut r, Unit::Celsius),
        Fs9721Symbols::Temp01F10C => {
            if s01 {
                temp(&mut r, Unit::Fahrenheit);
            }
            if s10 {
                temp(&mut r, Unit::Celsius);
            }
        }
        Fs9721Symbols::MaxCMin => {
            if s00 {
                r.flags |= MqFlags::MAX;
            }
            if s01 {
                temp(&mut r, Unit::Celsius);
            }
            if s11 {
                r.flags |= MqFlags::MIN;
            }
        }
        _ => {}
    }
    Some(r)
}

/// True for a DTM0660 packet.
pub fn dtm0660_valid(buf: &[u8]) -> bool {
    sync_nibbles_valid(buf, DTM0660_SIZE) && dtm0660_flags(buf).valid()
}

/// Parses a DTM0660 packet.
pub fn dtm0660_parse(buf: &[u8]) -> Option<Reading> {
    if buf.len() < DTM0660_SIZE {
        return None;
    }
    let (value, exponent) = nibble_value(buf, 0x10, dtm0660_digit, [0x00, 0xeb, 0x61, 0x00], 0, 0)?;
    Some(dtm0660_flags(buf).apply(value, exponent))
}

/// FS9922 flags, from bytes 7 to 10 of its ASCII-plus-bits packet.
#[derive(Clone, Copy, Debug, Default)]
struct Fs9922Flags {
    auto: bool,
    dc: bool,
    ac: bool,
    rel: bool,
    hold: bool,
    z1: bool,
    max: bool,
    min: bool,
    prefix: Prefix,
    beep: bool,
    diode: bool,
    percent: bool,
    volt: bool,
    ampere: bool,
    ohm: bool,
    hfe: bool,
    hertz: bool,
    farad: bool,
    celsius: bool,
    fahrenheit: bool,
}

fn fs9922_flags(buf: &[u8]) -> Fs9922Flags {
    Fs9922Flags {
        auto: bit(buf[7], 5),
        dc: bit(buf[7], 4),
        ac: bit(buf[7], 3),
        rel: bit(buf[7], 2),
        hold: bit(buf[7], 1),
        z1: bit(buf[8], 7),
        max: bit(buf[8], 5),
        min: bit(buf[8], 4),
        prefix: Prefix {
            nano: bit(buf[8], 1),
            micro: bit(buf[9], 7),
            milli: bit(buf[9], 6),
            kilo: bit(buf[9], 5),
            mega: bit(buf[9], 4),
            pico: false,
        },
        beep: bit(buf[9], 3),
        diode: bit(buf[9], 2),
        percent: bit(buf[9], 1),
        volt: bit(buf[10], 7),
        ampere: bit(buf[10], 6),
        ohm: bit(buf[10], 5),
        hfe: bit(buf[10], 4),
        hertz: bit(buf[10], 3),
        farad: bit(buf[10], 2),
        celsius: bit(buf[10], 1),
        fahrenheit: bit(buf[10], 0),
    }
}

impl Fs9922Flags {
    fn valid(&self) -> bool {
        self.prefix.is_single()
            && count(&[
                self.percent,
                self.volt,
                self.ampere,
                self.ohm,
                self.hfe,
                self.hertz,
                self.farad,
                self.celsius,
                self.fahrenheit,
            ]) <= 1
            && !(self.ac && self.dc)
            && !(self.celsius && self.fahrenheit)
    }
}

/// True for an FS9922 packet.
pub fn fs9922_valid(buf: &[u8]) -> bool {
    buf.len() >= FS9922_SIZE
        && (buf[0] == b'+' || buf[0] == b'-')
        && buf[12] == b'\r'
        && buf[13] == b'\n'
        && fs9922_flags(buf).valid()
}

/// Parses an FS9922 packet. With `z1_diode` the first user symbol
/// means diode mode (Voltcraft VC-830).
pub fn fs9922_parse(buf: &[u8], z1_diode: bool) -> Option<Reading> {
    if buf.len() < FS9922_SIZE {
        return None;
    }
    let sign = match buf[0] {
        b'+' => 1.0,
        b'-' => -1.0,
        _ => return None,
    };
    // An overload ("0.L") leaves parse_value before the sign and the
    // point are looked at: plus infinity, with no decimals of its own.
    let (value, exponent) = if &buf[1..5] == b"?0:?" {
        (f32::INFINITY, 0)
    } else {
        let mut v = 0u32;
        for &c in &buf[1..5] {
            v = v * 10 + u32::from(c.checked_sub(b'0').filter(|d| *d < 10)?);
        }
        let exponent = match buf[6] {
            b'0' => 0,
            b'1' => -3,
            b'2' => -2,
            b'4' => -1,
            _ => return None,
        };
        (v as f32 * sign, exponent)
    };
    let f = fs9922_flags(buf);
    let (v, digits) = scaled(value, exponent + f.prefix.exponent());
    let mut r = Reading::new(v, Mq::Gain, Unit::Unitless, digits);
    if f.volt || f.diode {
        (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
    }
    if f.ampere {
        (r.mq, r.unit) = (Mq::Current, Unit::Ampere);
    }
    if f.ohm {
        (r.mq, r.unit) = (Mq::Resistance, Unit::Ohm);
    }
    if f.hfe {
        (r.mq, r.unit) = (Mq::Gain, Unit::Unitless);
    }
    if f.hertz {
        (r.mq, r.unit) = (Mq::Frequency, Unit::Hertz);
    }
    if f.farad {
        (r.mq, r.unit) = (Mq::Capacitance, Unit::Farad);
    }
    if f.celsius {
        (r.mq, r.unit) = (Mq::Temperature, Unit::Celsius);
    }
    if f.fahrenheit {
        (r.mq, r.unit) = (Mq::Temperature, Unit::Fahrenheit);
    }
    if f.beep {
        (r.mq, r.unit) = (Mq::Continuity, Unit::Boolean);
        r.value = if r.value.is_infinite() { 0.0 } else { 1.0 };
    }
    if f.percent {
        (r.mq, r.unit) = (Mq::DutyCycle, Unit::Percentage);
    }
    if f.ac {
        r.flags |= MqFlags::AC;
    }
    if f.dc {
        r.flags |= MqFlags::DC;
    }
    if f.auto {
        r.flags |= MqFlags::AUTORANGE;
    }
    if f.diode || (z1_diode && f.z1) {
        (r.mq, r.unit) = (Mq::Voltage, Unit::Volt);
        r.flags |= MqFlags::DIODE | MqFlags::DC;
    }
    if f.hold {
        r.flags |= MqFlags::HOLD;
    }
    if f.max {
        r.flags |= MqFlags::MAX;
    }
    if f.min {
        r.flags |= MqFlags::MIN;
    }
    if f.rel {
        r.flags |= MqFlags::RELATIVE;
    }
    Some(r)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dmm::hexbytes;

    /// An FS9721 packet for the digits `d` (LCD codes), with flag
    /// nibbles for bytes 0 and 9 to 13.
    fn fs9721(d: [u8; 4], points: [bool; 3], b0: u8, tail: [u8; 5]) -> Vec<u8> {
        let mut p = vec![0x10 | b0];
        for (i, code) in d.iter().enumerate() {
            let hi = code >> 4;
            let lo = code & 0x0f;
            let mut hi_byte = hi;
            if i > 0 && points[i - 1] {
                hi_byte |= 0x08;
            }
            p.push(((2 + i * 2) as u8) << 4 | hi_byte);
            p.push(((3 + i * 2) as u8) << 4 | lo);
        }
        for (i, t) in tail.iter().enumerate() {
            p.push(((10 + i) as u8) << 4 | t);
        }
        p
    }

    #[test]
    fn fs9721_dc_volts() {
        // 1.234 V DC, auto, RS232: digits 1 2 3 4, point after the first.
        let p =
            fs9721([0x05, 0x5b, 0x1f, 0x27], [true, false, false], 0b0111, [0, 0, 0, 0b0100, 0]);
        assert!(fs9721_valid(&p));
        let r = fs9721_parse(&p, Fs9721Symbols::None).unwrap();
        assert_eq!((r.mq, r.unit, r.digits), (Mq::Voltage, Unit::Volt, 3));
        assert!((r.value - 1.234).abs() < 1e-5);
        assert!(r.flags.contains(MqFlags::DC | MqFlags::AUTORANGE));
        // Without the RS232 flag the packet is not valid.
        let p =
            fs9721([0x05, 0x5b, 0x1f, 0x27], [true, false, false], 0b0110, [0, 0, 0, 0b0100, 0]);
        assert!(!fs9721_valid(&p));
        // Over limit in kOhm, with the first user symbol meaning MAX.
        let p =
            fs9721([0x00, 0x7d, 0x68, 0x00], [false; 3], 0b0001, [0b0010, 0, 0b0100, 0, 0b0001]);
        let r = fs9721_parse(&p, Fs9721Symbols::MaxCMin).unwrap();
        assert!(r.value.is_infinite());
        assert_eq!((r.mq, r.digits), (Mq::Resistance, -3));
        assert!(r.flags.contains(MqFlags::MAX));
        // Symbol 10 as temperature.
        let p =
            fs9721([0x5b, 0x3e, 0x7d, 0x7d], [false, true, false], 0b0001, [0, 0, 0, 0, 0b0100]);
        let r = fs9721_parse(&p, Fs9721Symbols::Temp10C).unwrap();
        assert_eq!((r.mq, r.unit), (Mq::Temperature, Unit::Celsius));
        assert!((r.value - 25.0).abs() < 1e-5);
    }

    #[test]
    fn fs9922_packets() {
        // "+1234 1" with the volt and DC bits: 1.234 V.
        let mut p = b"+1234 1".to_vec();
        p.extend([0b0011_0000, 0, 0, 0b1000_0000, 0, b'\r', b'\n']);
        assert!(fs9922_valid(&p));
        let r = fs9922_parse(&p, false).unwrap();
        assert_eq!((r.mq, r.unit, r.digits), (Mq::Voltage, Unit::Volt, 3));
        assert!((r.value - 1.234).abs() < 1e-5);
        assert!(r.flags.contains(MqFlags::DC | MqFlags::AUTORANGE));
        // Over limit, milli, ampere, AC: parse_value leaves before the
        // sign and the point, so plus infinity with the prefix's digits
        // only, whatever byte 6 says (tools/emu/uni-t-dmm.sh, "overload
        // fs9922", against sigrok-cli).
        for point in *b"019" {
            let mut p = b"-?0:? 0".to_vec();
            p[6] = point;
            p.extend([0b0000_1000, 0, 0b0100_0000, 0b0100_0000, 0, b'\r', b'\n']);
            let r = fs9922_parse(&p, false).unwrap();
            assert!(r.value.is_infinite() && r.value > 0.0);
            assert_eq!((r.mq, r.flags, r.digits), (Mq::Current, MqFlags::AC, 3));
        }
        let mut p = b"-?0:? 0".to_vec();
        p.extend([0b0000_1000, 0, 0b0100_0000, 0b0100_0000, 0, b'\r', b'\n']);
        // Both AC and DC is not a packet.
        p[7] = 0b0001_1000;
        assert!(!fs9922_valid(&p));
        // z1 as diode on the VC-830.
        let mut p = b"+0567 2".to_vec();
        p.extend([0, 0b1000_0000, 0, 0b1000_0000, 0, b'\r', b'\n']);
        let r = fs9922_parse(&p, true).unwrap();
        assert!(r.flags.contains(MqFlags::DIODE));
        assert!((r.value - 5.67).abs() < 1e-5);
    }

    #[test]
    fn dtm0660_packet() {
        // 2.5 Celsius: digits 0 0 2 5 (LCD codes eb eb ad c7), point
        // after the third digit, RS232 and DC, degc.
        let d = [0xebu8, 0xeb, 0xad, 0xc7];
        let mut p = vec![0x10 | 0b1010];
        for (i, code) in d.iter().enumerate() {
            let mut hi = code >> 4;
            if i == 3 {
                hi |= 0x01;
            }
            p.push(((2 + i * 2) as u8) << 4 | hi);
            p.push(((3 + i * 2) as u8) << 4 | (code & 0x0f));
        }
        p.extend([0xa0, 0xb0, 0xc0, 0xd0, 0xe0 | 0b0010, 0xf0]);
        assert_eq!(p.len(), DTM0660_SIZE);
        assert!(dtm0660_valid(&p), "{p:02x?}");
        let r = dtm0660_parse(&p).unwrap();
        assert_eq!((r.mq, r.unit, r.digits), (Mq::Temperature, Unit::Celsius, 1));
        assert!((r.value - 2.5).abs() < 1e-5);
        let _ = hexbytes("00");
    }
}
