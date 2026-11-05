// Counterpart in sigrok:
//   libsigrok/src/hardware/norma-dmm/protocol.c,
//   libsigrok/src/hardware/norma-dmm/protocol.h,
//   Copyright (C) 2013 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Uwe Hermann, Martin Ling, Bert Vermeulen, Daniel Elstner,
//   Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The `DM9x0`'s two questions and what its answers mean.
//!
//! `IDN?` gets `1834 065 V1.06,IF V1.02`: digit 7 of the string is the type, 1 to
//! 5 for the DM910 to DM950 (B1024 to B1028), and what follows the type's place
//! is the version. `STATUS?` gets 20 hex digits. C's comments say what they are;
//! the sigrok wiki's page has real lines, none of which contradict them:
//!
//! | digits | what |
//! |--------|------|
//! | 0, 1   | keyboard, central switch: not used |
//! | 2      | value type: V, A (2 A), Ω, F, °C, Hz, A (10 A), scale factor, %, dB |
//! | 3      | range: scale the value by a power of 10, for the first six types |
//! | 4      | 4 invalid, 2 duplicate of a value sent before, 1 overload |
//! | 5      | sign (4) and first digit (3) |
//! | 6-9    | the other digits |
//! | 10     | mode: gives the unit and AC, DC and RMS |
//! | 11     | device status: 1 normal, 2 input loop, 3 TRANS/SENS, 4 error |
//! | 12, 13 | a byte: 0x20 beep, 0x04 REL, 0x01 % |
//! | 14, 15 | a byte: 0x80 manual range, 0x40 low battery, 0x08 HOLD, 2 MAX, 1 MIN |
//! | 16, 17 | a byte: 0x10 low battery with readings off spec, 0x01 dB |
//! | 18, 19 | a byte: 0x40 remote mode, among others not used |
//!
//! The wiki's `03030024913100400040` has its last byte at 0x40, from a meter under
//! remote control, which is what says a pair is a byte with its high digit first.

use crate::core::{Mq, MqFlags, Unit};

/// `SERIALCOMM`.
pub const SERIALCOMM: &str = "4800/8n1/dtr=1/rts=0/flow=1";
/// `IDN?`.
pub const IDN: &[u8] = b"IDN?\r\n";
/// `STATUS?`.
pub const STATUS: &[u8] = b"STATUS?\r\n";
/// `BUF_MAX`, the room `serial_readline` has in a scan.
pub const ID_BUF: usize = 50;
/// `NMADMM_BUFSIZE`.
pub const BUFSIZE: usize = 256;
/// `LINE_LENGTH`.
pub const LINE_LENGTH: usize = 20;

/// Which of the two names a meter goes by.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Brand {
    /// `norma-dmm`.
    Norma,
    /// `siemens-b102x`.
    Siemens,
}

impl Brand {
    /// `get_brandstr`.
    pub fn name(self) -> &'static str {
        match self {
            Brand::Norma => "Norma",
            Brand::Siemens => "Siemens",
        }
    }

    /// `get_typestr`.
    pub fn model(self, meter_type: u32) -> &'static str {
        const NAMES: [[&str; 2]; 5] = [
            ["DM910", "B1024"],
            ["DM920", "B1025"],
            ["DM930", "B1026"],
            ["DM940", "B1027"],
            ["DM950", "B1028"],
        ];
        let row = usize::try_from(meter_type).ok().and_then(|t| t.checked_sub(1));
        match row.and_then(|r| NAMES.get(r)) {
            Some(names) => names[usize::from(self == Brand::Siemens)],
            None => "Unknown type!",
        }
    }
}

/// A hex digit's value, 0 for anything else. C's `xgittoint` gives `a` to `f` as 0
/// to 5 (`protocol.c:67-74`), which its own table of modes cannot live with: 0xa
/// and 0xb never come (`:208-217`).
pub fn hex_digit(c: u8) -> u32 {
    char::from(c).to_digit(16).unwrap_or(0)
}

/// Two digits as a byte. C shifts the first by 8 (`protocol.c:282`, `:300`, `:316`),
/// so none of its masks above 0x08 ever holds.
fn pair(line: &[u8], at: usize) -> u32 {
    (hex_digit(line[at]) << 4) | hex_digit(line[at + 1])
}

/// A reply as C reads it: a string, up to its first NUL.
pub fn c_string(reply: &[u8]) -> &[u8] {
    let n = reply.iter().position(|c| *c == 0).unwrap_or(reply.len());
    &reply[..n]
}

/// The regular expression `^1834 [^,]*,IF V*`, whose `V*` takes nothing. `GLib`
/// matches UTF-8, and fails on what is not.
fn matches_id(id: &str) -> bool {
    id.strip_prefix("1834 ")
        .and_then(|rest| rest.split_once(','))
        .is_some_and(|(_, after)| after.starts_with("IF "))
}

/// The type and the version from an answer to `IDN?`, if it is one.
pub fn parse_id(line: &[u8]) -> Option<(u32, String)> {
    let id = std::str::from_utf8(c_string(line)).ok()?;
    if !matches_id(id) {
        return None;
    }
    // The shortest match is 9 bytes long, so both places exist; bytes for the
    // version, which may cut a character the regular expression passed.
    let bytes = id.as_bytes();
    Some((hex_digit(bytes[7]), String::from_utf8_lossy(&bytes[9..]).into_owned()))
}

/// A reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// The value, in single precision as C's.
    pub value: f32,
    /// What was measured.
    pub mq: Mq,
    /// How.
    pub flags: MqFlags,
    /// Its unit.
    pub unit: Unit,
}

/// A line for a message.
pub fn show(line: &[u8]) -> String {
    // libsigrok's logger takes the line breaks out of a message (log.c:228).
    String::from_utf8_lossy(c_string(line)).replace(['\r', '\n'], "")
}

/// `(mq, scale)` for a value type, before its range. C's float arithmetic: these
/// are computed in double and stored in a float.
fn value_type(vt: u32) -> Option<Mq> {
    Some(match vt {
        0 => Mq::Voltage,
        1 | 6 => Mq::Current,
        2 => Mq::Resistance,
        3 => Mq::Capacitance,
        4 => Mq::Temperature,
        5 => Mq::Frequency,
        7..=9 => Mq::Gain,
        _ => return None,
    })
}

/// The exponent a range adds for a value type; other types have a fixed format.
fn range_offset(vt: u32) -> Option<i32> {
    Some(match vt {
        0 => -5,
        1 => -7,
        2 | 5 => -2,
        3 => -12,
        4 => -1,
        _ => return None,
    })
}

/// A mode's unit, what it adds to the flags, and what it makes of the quantity.
fn mode(mode: u32, meter_type: u32) -> Option<(Unit, MqFlags, Option<Mq>)> {
    let rms = || if meter_type >= 3 { MqFlags::RMS } else { MqFlags::empty() };
    let trms = MqFlags::AC | MqFlags::DC | MqFlags::RMS;
    Some(match mode {
        0 => (Unit::Hertz, MqFlags::empty(), None),
        1 => (Unit::Volt, trms, None),
        2 => (Unit::Volt, MqFlags::AC | rms(), None),
        3 => (Unit::Volt, MqFlags::DC, None),
        4 => (Unit::Ohm, MqFlags::empty(), None),
        5 => (Unit::Boolean, MqFlags::empty(), Some(Mq::Continuity)),
        6 => (Unit::Celsius, MqFlags::empty(), None),
        7 => (Unit::Farad, MqFlags::empty(), None),
        8 => (Unit::Ampere, MqFlags::DC, None),
        9 => (Unit::Ampere, MqFlags::AC | rms(), None),
        0xa => (Unit::Ampere, trms, None),
        0xb => (Unit::Volt, MqFlags::DIODE | MqFlags::DC, None),
        _ => return None,
    })
}

/// The line's own checks, as C's and one more: what is left after its line end
/// must not be longer than the 20 digits. C cuts it to them, so a line that lost
/// its end and ran into the next one passes as a reading neither of them held.
fn well_formed(line: &[u8]) -> bool {
    let text = c_string(line);
    let text = text.strip_suffix(b"\n").unwrap_or(text);
    let text = text.strip_suffix(b"\r").unwrap_or(text);
    if text.len() > LINE_LENGTH {
        log::error!("line: Invalid status '{}', must be 20 hex digits.", show(line));
        return false;
    }
    let digits = &c_string(line)[..c_string(line).len().min(LINE_LENGTH)];
    if digits.len() != LINE_LENGTH {
        log::error!("line: Invalid status '{}', must be 20 hex digits.", show(digits));
        return false;
    }
    if let Some(pos) = digits.iter().position(|c| !c.is_ascii_hexdigit()) {
        log::error!("line: Expected hex digit in '{}' at pos {pos}!", show(digits));
        return false;
    }
    true
}

/// `nma_process_line`: a line with its CR LF, to the reading it holds. There is no
/// reading in a line that is wrong, in a device error, or in a duplicate.
///
/// A value type or a mode C has no entry for gives its error and no reading,
/// as a value of no known quantity is none. No C run has either: its digit
/// conversion stops at 9.
pub fn process_line(line: &[u8], meter_type: u32) -> Option<Reading> {
    log::trace!("Received line '{}'.", show(line));
    if !well_formed(line) {
        return None;
    }
    let line = &line[..LINE_LENGTH];
    let digit = |at: usize| hex_digit(line[at]);

    let vt = digit(2);
    let mut scale = 1.0f32;
    let mq = value_type(vt);
    if mq.is_none() {
        log::error!("Unknown value type: 0x{vt:02x}.");
    }
    if vt == 8 || vt == 9 {
        scale = (f64::from(scale) / 100.0) as f32;
    }
    let range = digit(3);
    if let Some(offset) = range_offset(vt) {
        let exponent = i32::try_from(range).unwrap_or(0) + offset;
        scale = (f64::from(scale) * 10f64.powi(exponent)) as f32;
    }
    let flags5 = digit(5);
    let mut value = (flags5 & 0x03) as f32;
    if flags5 & 0x04 != 0 {
        scale = -scale;
    }
    for at in 6..10 {
        value = value * 10.0 + digit(at) as f32;
    }
    value *= scale;

    let mmode = digit(10);
    let meaning = mode(mmode, meter_type);
    if meaning.is_none() {
        log::error!("Unknown mmode: 0x{mmode:02x}.");
    }
    let devstat = digit(11);
    match devstat {
        1..=3 => {}
        4 => {
            log::error!("Device error. Fuse?");
            return None;
        }
        other => log::error!("Unknown device status: 0x{other:02x}"),
    }
    let (Some(mq), Some((mut unit, mut flags, new_mq))) = (mq, meaning) else {
        return None;
    };
    let mq = new_mq.unwrap_or(mq);

    let symbols = pair(line, 12);
    if mq == Mq::Continuity {
        value = if symbols & 0x20 != 0 { 1.0 } else { 0.0 };
    }
    if symbols & 0x04 != 0 {
        flags |= MqFlags::RELATIVE;
    }
    if symbols & 0x01 != 0 {
        unit = Unit::Percentage;
    }

    let state = pair(line, 14);
    if state & 0x80 == 0 {
        flags |= MqFlags::AUTORANGE;
    }
    if state & 0x08 != 0 {
        flags |= MqFlags::HOLD;
    }
    if state & 0x02 != 0 {
        flags |= MqFlags::MAX;
    }
    if state & 0x01 != 0 {
        flags |= MqFlags::MIN;
    }

    let extras = pair(line, 16);
    if extras & 0x10 != 0 {
        log::warn!("Low battery, measurement quality degraded!");
    }
    if extras & 0x01 != 0 {
        unit = if unit == Unit::Volt { Unit::DecibelVolt } else { Unit::Unitless };
    }

    let marks = digit(4);
    if marks & 0x04 != 0 {
        value = f32::NAN;
    } else if marks & 0x01 != 0 {
        value = f32::INFINITY;
    }
    if marks & 0x02 != 0 {
        log::trace!("Duplicate value, dismissing!");
        return None;
    }
    log::trace!("range={range}/scale={scale}/value={value}");
    Some(Reading { value, mq, flags, unit })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reading(line: &str) -> Reading {
        process_line(line.as_bytes(), 5).expect("a reading")
    }

    #[test]
    fn hex_digits_have_their_values() {
        for (c, want) in [(b'0', 0), (b'9', 9), (b'a', 10), (b'f', 15), (b'A', 10), (b'F', 15)] {
            assert_eq!(hex_digit(c), want);
        }
        assert_eq!(hex_digit(b'x'), 0);
        assert_eq!(hex_digit(b' '), 0);
    }

    #[test]
    fn the_models_are_the_table() {
        assert_eq!(Brand::Norma.model(1), "DM910");
        assert_eq!(Brand::Norma.model(5), "DM950");
        assert_eq!(Brand::Siemens.model(1), "B1024");
        assert_eq!(Brand::Siemens.model(5), "B1028");
        for t in [0, 6, 15] {
            assert_eq!(Brand::Norma.model(t), "Unknown type!");
            assert_eq!(Brand::Siemens.model(t), "Unknown type!");
        }
    }

    #[test]
    fn an_id_is_as_the_regular_expression_has_it() {
        assert_eq!(parse_id(b"1834 065 V1.06,IF V1.02"), Some((5, "V1.06,IF V1.02".to_owned())));
        assert_eq!(parse_id(b"1834 063 V1.04,IF V"), Some((3, "V1.04,IF V".to_owned())));
        assert_eq!(parse_id(b"1834 ,IF "), Some((15, String::new())));
        assert_eq!(parse_id(b"1834 06x V1,IF "), Some((0, "V1,IF ".to_owned())));
        for no in [
            &b"1835 065 V1.06,IF V1.02"[..],
            b" 1834 065 V1.06,IF V1.02",
            b"1834 065 V1.06",
            b"1834 065 V1.06,XF",
            b"1834 065 V1.06,IF",
            b"1834 065,V1.06,IF V1.02",
            b"1834",
            b"",
            b"1834 \xff,IF ",
        ] {
            assert_eq!(parse_id(no), None, "{}", no.escape_ascii());
        }
        // A NUL ends the string.
        assert_eq!(parse_id(b"1834 06\x001,IF "), None);
        assert_eq!(parse_id(b"1834 065,IF \0more"), Some((5, "IF ".to_owned())));
    }

    #[test]
    fn the_wikis_line_is_24_91_volts_dc() {
        let r = reading("03030024913100400040\r\n");
        assert_eq!(r.value, 2491.0f32 * 0.01f32);
        assert_eq!(r.mq, Mq::Voltage);
        assert_eq!(r.unit, Unit::Volt);
        assert_eq!(r.flags, MqFlags::DC | MqFlags::AUTORANGE);
    }

    #[test]
    fn each_value_type_has_its_quantity_and_scale() {
        for (vt, mq, want) in [
            ('0', Mq::Voltage, 12345.0f32 * 1e-5f32),
            ('1', Mq::Current, 12345.0 * 1e-7),
            ('2', Mq::Resistance, 12345.0 * 1e-2),
            ('3', Mq::Capacitance, 12345.0 * 1e-12),
            ('4', Mq::Temperature, 12345.0 * 1e-1),
            ('5', Mq::Frequency, 12345.0 * 1e-2),
            ('6', Mq::Current, 12345.0),
            ('7', Mq::Gain, 12345.0),
            ('8', Mq::Gain, 12345.0 * 1e-2),
            ('9', Mq::Gain, 12345.0 * 1e-2),
        ] {
            let r = reading(&format!("00{vt}00123453100000000"));
            assert_eq!(r.mq, mq, "{vt}");
            assert_eq!(r.value, want, "{vt}");
        }
    }

    #[test]
    fn a_range_is_a_power_of_ten_and_a_letter_is_its_value() {
        assert_eq!(reading("00050012343100000000").value, 1234.0);
        assert_eq!(reading("00070000013100000000").value, 100.0);
        assert_eq!(reading("000a0000013100000000").value, 100_000.0);
        assert_eq!(reading("000f0000013100000000").value, 10_000_000_000.0);
        // No range of their own.
        assert_eq!(reading("00690000013100000000").value, 1.0);
        assert_eq!(reading("00790000013100000000").value, 1.0);
    }

    #[test]
    fn sign_and_digits() {
        assert_eq!(reading("00000000013100000000").value, 1.0f32 * 1e-5f32);
        assert_eq!(reading("00000400013100000000").value, -1e-5f32);
        assert!(reading("00000400003100000000").value.is_sign_negative());
        assert_eq!(reading("00200000003100000000").value, 0.0);
        // The first digit is two bits: 9 has the 1 of them.
        assert_eq!(reading("00200399993100000000").value, 39999.0 * 0.01);
        assert_eq!(reading("00200999993100000000").value, 19999.0 * 0.01);
        // A letter among the digits has its value too.
        assert_eq!(reading("002000a0003100000000").value, 10000.0 * 0.01);
    }

    #[test]
    fn each_mode_has_its_unit_and_flags() {
        let auto = MqFlags::AUTORANGE;
        let trms = MqFlags::AC | MqFlags::DC | MqFlags::RMS;
        for (m, unit, flags) in [
            ('0', Unit::Hertz, MqFlags::empty()),
            ('1', Unit::Volt, trms),
            ('2', Unit::Volt, MqFlags::AC | MqFlags::RMS),
            ('3', Unit::Volt, MqFlags::DC),
            ('4', Unit::Ohm, MqFlags::empty()),
            ('5', Unit::Boolean, MqFlags::empty()),
            ('6', Unit::Celsius, MqFlags::empty()),
            ('7', Unit::Farad, MqFlags::empty()),
            ('8', Unit::Ampere, MqFlags::DC),
            ('9', Unit::Ampere, MqFlags::AC | MqFlags::RMS),
            ('a', Unit::Ampere, trms),
            ('A', Unit::Ampere, trms),
            ('b', Unit::Volt, MqFlags::DIODE | MqFlags::DC),
        ] {
            let r = reading(&format!("0000000001{m}100000000"));
            assert_eq!(r.unit, unit, "{m}");
            assert_eq!(r.flags, flags | auto, "{m}");
        }
    }

    #[test]
    fn no_rms_under_type_3() {
        for (t, rms) in
            [(1, false), (2, false), (3, true), (4, true), (5, true), (0, false), (6, true)]
        {
            for m in ['2', '9'] {
                let line = format!("0000000001{m}100000000");
                let r = process_line(line.as_bytes(), t).unwrap();
                assert_eq!(r.flags.contains(MqFlags::RMS), rms, "{t} {m}");
                assert!(r.flags.contains(MqFlags::AC));
            }
        }
    }

    #[test]
    fn continuity_is_its_flag_alone() {
        let r = reading("00000000015100000000");
        assert_eq!((r.mq, r.unit, r.value), (Mq::Continuity, Unit::Boolean, 0.0));
        assert_eq!(reading("00000000015120000000").value, 1.0);
        // Another digit of the pair is not the beep.
        assert_eq!(reading("00000000015102000000").value, 0.0);
        assert_eq!(reading("00000000015110000000").value, 0.0);
    }

    #[test]
    fn the_flags_of_each_pair() {
        let flags = |l: &str| reading(l).flags;
        let base = MqFlags::DC | MqFlags::AUTORANGE;
        assert_eq!(flags("00000000013104000000"), base | MqFlags::RELATIVE);
        assert_eq!(reading("00000000013101000000").unit, Unit::Percentage);
        assert_eq!(flags("00000000013100080000"), base | MqFlags::HOLD);
        assert_eq!(flags("00000000013100020000"), base | MqFlags::MAX);
        assert_eq!(flags("00000000013100010000"), base | MqFlags::MIN);
        assert_eq!(flags("00000000013100800000"), MqFlags::DC);
        // What means nothing to a sample.
        assert_eq!(flags("00000000013182440077"), base);
    }

    #[test]
    fn db_makes_volts_dbv_and_anything_else_unitless() {
        assert_eq!(reading("00000000013100000100").unit, Unit::DecibelVolt);
        assert_eq!(reading("00000000014100000100").unit, Unit::Unitless);
        assert_eq!(reading("00000000013100001000").unit, Unit::Volt);
    }

    #[test]
    fn invalid_overload_duplicate() {
        assert!(reading("00004000013100000000").value.is_nan());
        assert_eq!(reading("00001000013100000000").value, f32::INFINITY);
        assert!(reading("00005000013100000000").value.is_nan());
        // A sign leaves an overload positive, as C sets the value after.
        assert_eq!(reading("00001400013100000000").value, f32::INFINITY);
        assert_eq!(process_line(b"00002000013100000000", 5), None);
        assert_eq!(process_line(b"00006000013100000000", 5), None);
    }

    #[test]
    fn a_device_error_is_no_reading_and_an_unknown_status_is_one() {
        assert_eq!(process_line(b"00000000013400000000", 5), None);
        for s in ['1', '2', '3', '0', '5', '9', 'e', 'f'] {
            let line = format!("00000000013{s}00000000");
            assert!(process_line(line.as_bytes(), 5).is_some(), "{s}");
        }
    }

    #[test]
    fn what_c_has_no_entry_for_is_no_reading() {
        for vt in ['a', 'f'] {
            assert_eq!(process_line(format!("00{vt}00000013100000000").as_bytes(), 5), None);
        }
        for m in ['c', 'f'] {
            assert_eq!(process_line(format!("0000000001{m}100000000").as_bytes(), 5), None);
        }
    }

    #[test]
    fn a_line_must_be_20_hex_digits() {
        for bad in [
            &b""[..],
            b"\r\n",
            b"abc\r\n",
            b"0000000001310000000\r\n",
            b"000000000131000000\r\n",
            b"0000000001310000000x\r\n",
            b" 0000000013100000000\r\n",
            b"00000000013100000000 \r\n",
            b"000000000131000000000\r\n",
            b"0303002403030024913100400040\r\n",
            b"0000000001310000\x00000\r\n",
        ] {
            assert_eq!(process_line(bad, 5), None, "{}", bad.escape_ascii());
        }
        for good in
            [&b"00000000013100000000"[..], b"00000000013100000000\n", b"00000000013100000000\r\n"]
        {
            assert!(process_line(good, 5).is_some(), "{}", good.escape_ascii());
        }
    }

    #[test]
    fn a_full_flag_sweep_has_no_panic() {
        for d in "0123456789abcdefABCDEF".bytes() {
            for at in 0..LINE_LENGTH {
                let mut line = *b"00000000013100000000";
                line[at] = d;
                let _ = process_line(&line, 5);
            }
        }
    }
}
