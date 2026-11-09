// Counterpart in sigrok:
//   libsigrok/src/hardware/hp-3478a/protocol.c,
//   libsigrok/src/hardware/hp-3478a/protocol.h,
//   Copyright (C) 2017-2021 Frank Stettner <frank-stettner@gmx.net>,
//   with commits by Uwe Hermann, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The HP 3478A's codes and answers, with no I/O.
//!
//! The meter takes one or two character codes (HP's quick reference card):
//! `F1`..`F7` the function (DC volts, AC volts, 2-wire ohms, 4-wire ohms, DC amps, AC
//! amps, extended ohms), `R-2`..`R7` the range as a power of ten of its full scale
//! over 3 (and `RA` autorange), `N3`..`N5` the digits, `T1`..`T5` the trigger (`T1`
//! internal), `D1` the normal display. `B` makes the next talk five bytes of binary
//! status: the function, range and digits; the status (internal trigger, autorange,
//! autozero, 50 Hz, front terminals, CAL RAM, external trigger); the SRQ mask; the
//! error register (CAL RAM checksum, RAM, ROM, A/D slope, A/D self test, A/D link);
//! the DAC's setting. A reading is 13 characters with EOI: `+1.23456E+0` and CR LF,
//! an overload `+9.99999E+9`. The serial poll byte's bit 0 says a reading is ready
//! (manual, 3-105), bit 6 that service is requested.

use crate::core::{Mq, MqFlags, Unit};

/// libsigrok's dummy exponent for autorange.
pub const AUTO: i8 = -99;

/// Serial poll: a reading is ready (`SRQ_BUS_AVAIL`).
pub const SPOLL_READY: u8 = 0x01;

/// A reading at or over this is an overload (`protocol.c`, `9.998e+9`).
pub const OVERLOAD: f64 = 9.998e9;

/// The quantities `--list` offers, `mqopts` in `api.c`.
pub const MQOPTS: [(Mq, MqFlags); 6] = [
    (Mq::Voltage, MqFlags::DC),
    (Mq::Voltage, MqFlags::AC),
    (Mq::Current, MqFlags::DC),
    (Mq::Current, MqFlags::AC),
    (Mq::Resistance, MqFlags::empty()),
    (Mq::Resistance, MqFlags::FOUR_WIRE),
];

/// One of `rangeopts` in `api.c`: a quantity, its flag, the range's exponent and
/// its name.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RangeOpt {
    /// The quantity.
    pub mq: Mq,
    /// The one flag that tells the function.
    pub flag: MqFlags,
    /// Power of ten of full scale over 3, or [`AUTO`].
    pub exp: i8,
    /// What `range` says.
    pub name: &'static str,
}

const fn r(mq: Mq, flag: MqFlags, exp: i8, name: &'static str) -> RangeOpt {
    RangeOpt { mq, flag, exp, name }
}

const V: Mq = Mq::Voltage;
const A: Mq = Mq::Current;
const O: Mq = Mq::Resistance;
const DC: MqFlags = MqFlags::DC;
const AC: MqFlags = MqFlags::AC;
const NONE: MqFlags = MqFlags::empty();
const W4: MqFlags = MqFlags::FOUR_WIRE;

/// `rangeopts`, in C's order.
pub const RANGES: [RangeOpt; 33] = [
    r(V, DC, AUTO, "Auto"),
    r(V, DC, -2, "30mV"),
    r(V, DC, -1, "300mV"),
    r(V, DC, 0, "3V"),
    r(V, DC, 1, "30V"),
    r(V, DC, 2, "300V"),
    r(V, AC, AUTO, "Auto"),
    r(V, AC, -1, "300mV"),
    r(V, AC, 0, "3V"),
    r(V, AC, 1, "30V"),
    r(V, AC, 2, "300V"),
    r(A, DC, AUTO, "Auto"),
    r(A, DC, -1, "300mA"),
    r(A, DC, 0, "3A"),
    r(A, AC, AUTO, "Auto"),
    r(A, AC, -1, "300mA"),
    r(A, AC, 0, "3A"),
    r(O, NONE, AUTO, "Auto"),
    r(O, NONE, 1, "30R"),
    r(O, NONE, 2, "300R"),
    r(O, NONE, 3, "3kR"),
    r(O, NONE, 4, "30kR"),
    r(O, NONE, 5, "300kR"),
    r(O, NONE, 6, "3MR"),
    r(O, NONE, 7, "30MR"),
    r(O, W4, AUTO, "Auto"),
    r(O, W4, 1, "30R"),
    r(O, W4, 2, "300R"),
    r(O, W4, 3, "3kR"),
    r(O, W4, 4, "30kR"),
    r(O, W4, 5, "300kR"),
    r(O, W4, 6, "3MR"),
    r(O, W4, 7, "30MR"),
];

/// The digits `--list` offers.
pub const DIGITS: [&str; 3] = ["3.5", "4.5", "5.5"];

/// What the binary status says, as `parse_function_byte` and `parse_status_byte`
/// keep it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    /// The quantity.
    pub mq: Mq,
    /// The function's flag: DC, AC, 4-wire or none (`measurement_mq_flag`).
    pub flag: MqFlags,
    /// What a sample carries (`acquisition_mq_flags`): RMS with AC, autorange.
    pub flags: MqFlags,
    /// The unit.
    pub unit: Unit,
    /// The range's exponent, [`AUTO`] under autorange.
    pub exp: i8,
    /// All digits, the half one counted: 4, 5 or 6.
    pub digits: u8,
    /// Decimal places of a sample (`sr_digits`).
    pub places: i32,
    /// The error register.
    pub errors: u8,
    /// The front terminals are selected.
    pub front: bool,
}

/// The error register's bits and libsigrok's words for them, in C's order.
pub const ERRORS: [(u8, &str); 6] = [
    (1 << 5, "Failure in the A/D link"),
    (1 << 4, "A/D has failed its internal Self Test"),
    (1 << 3, "There has been an A/D slope error"),
    (1 << 2, "The ROM Self Test has failed"),
    (1 << 1, "The RAM Self Test has failed"),
    (
        1,
        "Self Test: Any of the CAL RAM locations have bad checksums, or a range with a bad \
         checksum is selected",
    ),
];

impl Status {
    /// The binary status's first four bytes. C reads them from whatever came (four
    /// bytes past a shorter answer, `protocol.c:441-444`), and keeps what it had where
    /// a field holds a value the card lists as none (`parse_range_*` return
    /// `SR_ERR_DATA`, unread) or with no function at all, the flags cleared. Here each
    /// of those is no status.
    pub fn parse(bytes: &[u8]) -> Result<Status, String> {
        let hex = || bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ");
        let &[function, status, _mask, errors, ..] = bytes else {
            return Err(format!("The binary status is {} bytes ({}), not 5.", bytes.len(), hex()));
        };
        let digits = match function & 0b11 {
            0b01 => 6,
            0b10 => 5,
            0b11 => 4,
            _ => return Err(format!("The binary status gives no digits ({}).", hex())),
        };
        let code = (function >> 2) & 0b111;
        let (mq, flag, mut flags, unit, lowest, highest) = match function >> 5 {
            1 => (V, DC, DC, Unit::Volt, -2, 2),
            2 => (V, AC, AC | MqFlags::RMS, Unit::Volt, -1, 2),
            3 | 7 => (O, NONE, NONE, Unit::Ohm, 1, 7),
            4 => (O, W4, W4, Unit::Ohm, 1, 7),
            5 => (A, DC, DC, Unit::Ampere, -1, 0),
            6 => (A, AC, AC | MqFlags::RMS, Unit::Ampere, -1, 0),
            _ => return Err(format!("The binary status gives no function ({}).", hex())),
        };
        let range = lowest + i8::try_from(code).unwrap_or(0) - 1;
        if code == 0 || range > highest {
            return Err(format!("The binary status gives no such range ({}).", hex()));
        }
        // sr_digits: decimal places of the range's full scale at these digits. 3 V at
        // 6 digits is 3.00000; each decade up takes one, the ohms start at 30.
        let places = i32::from(digits) - 1 - i32::from(range);
        let mut exp = range;
        if status & 0b10 != 0 {
            flags |= MqFlags::AUTORANGE;
            exp = AUTO;
        }
        Ok(Status {
            mq,
            flag,
            flags,
            unit,
            exp,
            digits,
            places,
            errors,
            front: status & 0b1_0000 != 0,
        })
    }

    /// What `parse_error_byte` says of the error register, in its order.
    pub fn error_lines(&self) -> Vec<&'static str> {
        ERRORS.iter().filter(|(bit, _)| self.errors & bit != 0).map(|(_, text)| *text).collect()
    }

    /// `range`'s answer: the name of the range selected, `Auto` under autorange and
    /// where none matches, as C has it.
    pub fn range_name(&self) -> &'static str {
        RANGES
            .iter()
            .find(|o| o.mq == self.mq && o.flag == self.flag && o.exp == self.exp)
            .map_or("Auto", |o| o.name)
    }

    /// `digits`' answer, `digits_map`.
    pub fn digits_name(&self) -> &'static str {
        match self.digits {
            4 => "3.5",
            5 => "4.5",
            _ => "5.5",
        }
    }

    /// The ranges of the function selected.
    pub fn range_names(&self) -> Vec<&'static str> {
        RANGES.iter().filter(|o| o.mq == self.mq && o.flag == self.flag).map(|o| o.name).collect()
    }

    /// The range named `name` of the function selected.
    pub fn find_range(&self, name: &str) -> Option<i8> {
        RANGES
            .iter()
            .find(|o| o.mq == self.mq && o.flag == self.flag && o.name == name)
            .map(|o| o.exp)
    }
}

/// `set_mq_volt`, `set_mq_amp` and `set_mq_ohm`: the code for a quantity, `None` for
/// one the meter has no function for. With both AC and DC asked, AC wins, as in C.
pub fn mq_command(mq: Mq, flags: MqFlags) -> Option<&'static str> {
    let (ac, dc) = (flags.contains(MqFlags::AC), flags.contains(MqFlags::DC));
    match mq {
        Mq::Voltage if ac => Some("F2"),
        Mq::Voltage if dc => Some("F1"),
        Mq::Current if ac => Some("F6"),
        Mq::Current if dc => Some("F5"),
        Mq::Resistance if flags.contains(MqFlags::FOUR_WIRE) => Some("F4"),
        Mq::Resistance => Some("F3"),
        _ => None,
    }
}

/// `hp_3478a_set_range`'s code.
pub fn range_command(exp: i8) -> String {
    if exp == AUTO { "RA".to_owned() } else { format!("R{exp}") }
}

/// `hp_3478a_set_digits`'s code for `3.5`, `4.5` or `5.5`, with the digit count it
/// selects (the half one counted); `None` for anything else. C's loop also matches
/// `""` to an `N-1` and reads past `digits_map` for a word it lacks.
pub fn digits_command(name: &str) -> Option<(String, u8)> {
    let digits = match name {
        "3.5" => 4,
        "4.5" => 5,
        "5.5" => 6,
        _ => return None,
    };
    Some((format!("N{}", digits - 1), digits))
}

/// A reading, as the manual has it: a sign, six digits around a point with one to
/// three before it, `E`, a sign and one digit. An overload is infinity, as in C. C's
/// `sr_atod_ascii` takes any number and more (hexadecimal, `inf`, `nan`); here what
/// isn't the meter's form is no reading.
pub fn parse_reading(text: &str) -> Option<f64> {
    let b = text.as_bytes();
    let point = b.iter().position(|c| *c == b'.')?;
    let ok = b.len() == 11
        && matches!(b[0], b'+' | b'-')
        && (2..=4).contains(&point)
        && b[1..point].iter().chain(&b[point + 1..8]).all(u8::is_ascii_digit)
        && b[8] == b'E'
        && matches!(b[9], b'+' | b'-')
        && b[10].is_ascii_digit();
    if !ok {
        return None;
    }
    let value: f64 = text.parse().ok()?;
    Some(if value >= OVERLOAD { f64::INFINITY } else { value })
}

/// What a capture says of a meter that has said nothing for a while, decided on the
/// times it is given.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Silent {
    /// Go on.
    Wait,
    /// Go on, and say this once.
    Warn(u64),
    /// Fail.
    GiveUp(u64),
}

/// The bound on a meter with no reading ready: libsigrok's capture polls for ever.
/// The 3478A under `T1` makes readings without pause, the slowest a 5½ digit one
/// with autozero, under a second by the manual's chapter 3; no figure for its rate
/// was found, so the bound leaves room: 30 s with no reading at all fails a capture,
/// and after a reading 30 s gets a warning and 10 minutes the failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Silence {
    /// Before the first reading.
    pub first: std::time::Duration,
    /// After one, the warning.
    pub warn: std::time::Duration,
    /// After one, the failure.
    pub give_up: std::time::Duration,
}

impl Default for Silence {
    fn default() -> Silence {
        Silence {
            first: std::time::Duration::from_secs(30),
            warn: std::time::Duration::from_secs(30),
            give_up: std::time::Duration::from_secs(600),
        }
    }
}

/// The silence of one capture.
#[derive(Clone, Debug)]
pub struct Quiet {
    bounds: Silence,
    since: std::time::Instant,
    readings: u64,
    warned: bool,
}

impl Quiet {
    /// A capture starting at `now`.
    pub fn new(bounds: Silence, now: std::time::Instant) -> Quiet {
        Quiet { bounds, since: now, readings: 0, warned: false }
    }

    /// A reading came at `now`: whether to say the meter is back.
    pub fn reading(&mut self, now: std::time::Instant) -> bool {
        self.since = now;
        self.readings += 1;
        std::mem::take(&mut self.warned)
    }

    /// A poll at `looked` found nothing ready. `Warn` and `GiveUp` carry the
    /// seconds of silence.
    pub fn nothing(&mut self, looked: std::time::Instant) -> Silent {
        let quiet = looked.saturating_duration_since(self.since);
        let secs = quiet.as_secs();
        if self.readings == 0 {
            return if quiet >= self.bounds.first { Silent::GiveUp(secs) } else { Silent::Wait };
        }
        if quiet >= self.bounds.give_up {
            return Silent::GiveUp(secs);
        }
        if quiet >= self.bounds.warn && !self.warned {
            self.warned = true;
            return Silent::Warn(secs);
        }
        Silent::Wait
    }

    /// The words of a give-up.
    pub fn give_up_text(&self, secs: u64) -> String {
        if self.readings == 0 {
            format!("The meter had no reading ready in {secs} s.")
        } else {
            format!("The meter has had no reading ready for {secs} s.")
        }
    }
}

/// The words of a warning.
pub fn warn_text(secs: u64) -> String {
    format!("The meter has had no reading ready for {secs} s; still waiting.")
}

/// The words when it is back.
pub const BACK_TEXT: &str = "The meter has a reading ready again.";

/// A capture's limits, `sr_sw_limits`: samples, and milliseconds from the start.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Limits {
    /// Samples, 0 for none.
    pub samples: u64,
    /// Milliseconds, 0 for none.
    pub msec: u64,
}

impl Limits {
    /// Whether `samples` taken by `elapsed` meet a limit.
    pub fn met(&self, samples: u64, elapsed: std::time::Duration) -> bool {
        (self.samples > 0 && samples >= self.samples)
            || (self.msec > 0 && elapsed.as_millis() >= u128::from(self.msec))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn statuses_read_as_c_reads_them() {
        // DCV 3 V autorange 5½: C's sr_digits 5 (6 - 1), range Auto.
        let s = Status::parse(&[0b0010_1101, 0b0001_0111, 0, 0, 33]).unwrap();
        assert_eq!(
            (s.mq, s.flag, s.unit, s.exp, s.digits, s.places),
            (V, DC, Unit::Volt, AUTO, 6, 5)
        );
        assert_eq!(s.flags, DC | MqFlags::AUTORANGE);
        assert_eq!((s.range_name(), s.digits_name(), s.front), ("Auto", "5.5", true));
        // Each function and range, as parse_range_* sets sr_digits at 5½ digits.
        for (function, code, mq, flag, exp, places) in [
            (1, 1, V, DC, -2, 7),
            (1, 2, V, DC, -1, 6),
            (1, 5, V, DC, 2, 3),
            (2, 1, V, AC, -1, 6),
            (2, 4, V, AC, 2, 3),
            (3, 1, O, NONE, 1, 4),
            (3, 7, O, NONE, 7, -2),
            (4, 3, O, W4, 3, 2),
            (5, 1, A, DC, -1, 6),
            (5, 2, A, DC, 0, 5),
            (6, 2, A, AC, 0, 5),
            (7, 7, O, NONE, 7, -2),
        ] {
            let s = Status::parse(&[function << 5 | code << 2 | 1, 0, 0, 0, 0]).unwrap();
            assert_eq!(
                (s.mq, s.flag, s.exp, s.places),
                (mq, flag, exp, places),
                "F{function} {code}"
            );
        }
        // 3½ digits on 30 V: 2 places fewer.
        let s = Status::parse(&[0b0011_0011, 0, 0, 0, 0]).unwrap();
        assert_eq!((s.digits, s.places, s.range_name(), s.digits_name()), (4, 2, "30V", "3.5"));
        let ac = Status::parse(&[0b0100_1010, 0, 0, 0, 0]).unwrap();
        assert_eq!(ac.flags, AC | MqFlags::RMS);
        assert_eq!(ac.range_names(), ["Auto", "300mV", "3V", "30V", "300V"]);
        assert_eq!(ac.find_range("3V"), Some(0));
        assert_eq!(ac.find_range("30mV"), None);
    }

    #[test]
    fn statuses_that_say_nothing_are_refused() {
        for (bytes, says) in [
            (&[0x2d, 0x17, 0][..], "3 bytes (2d 17 00), not 5"),
            (&[0x2c, 0, 0, 0, 0], "no digits"),
            (&[0x0d, 0, 0, 0, 0], "no function"),
            (&[0x21, 0, 0, 0, 0], "no such range"),
            (&[0x39, 0, 0, 0, 0], "no such range"),
            (&[0xb9, 0, 0, 0, 0], "no such range"),
        ] {
            let e = Status::parse(bytes).unwrap_err();
            assert!(e.contains(says), "{e}");
        }
        // Four bytes are enough for what C reads.
        assert!(Status::parse(&[0x2d, 0x17, 0, 0]).is_ok());
    }

    #[test]
    fn errors_are_named_in_cs_order() {
        let s = Status::parse(&[0x2d, 0, 0, 0x21, 0]).unwrap();
        assert_eq!(s.error_lines(), [ERRORS[0].1, ERRORS[5].1]);
        assert_eq!(Status::parse(&[0x2d, 0, 0, 0, 0]).unwrap().error_lines().len(), 0);
    }

    #[test]
    fn commands_are_cs() {
        assert_eq!(mq_command(V, DC), Some("F1"));
        assert_eq!(mq_command(V, AC | DC), Some("F2"), "AC wins");
        assert_eq!(mq_command(V, NONE), None);
        assert_eq!(mq_command(A, AC), Some("F6"));
        assert_eq!(mq_command(A, DC), Some("F5"));
        assert_eq!(mq_command(O, W4), Some("F4"));
        assert_eq!(mq_command(O, AC), Some("F3"));
        assert_eq!(mq_command(Mq::Frequency, NONE), None);
        assert_eq!(range_command(AUTO), "RA");
        assert_eq!(range_command(-2), "R-2");
        assert_eq!(range_command(7), "R7");
        assert_eq!(digits_command("4.5"), Some(("N4".to_owned(), 5)));
        assert_eq!(digits_command(""), None);
        assert_eq!(digits_command("6.5"), None);
    }

    #[test]
    fn readings_take_the_meters_form_alone() {
        assert_eq!(parse_reading("+0.12345E+0"), Some(0.12345));
        assert_eq!(parse_reading("-035.801E-3"), Some(-0.035_801));
        assert_eq!(parse_reading("+12.3456E+6"), Some(12.3456e6));
        assert_eq!(parse_reading("+9.99999E+9"), Some(f64::INFINITY));
        for bad in [
            "",
            "+1.2X456E+0",
            "1.23456E+00",
            "+1.23456E+00",
            "+1234.56E+0",
            "+.123456E+0",
            "+1.23456e+0",
            "+1.23456E 0",
            "0x1p3",
            "inf",
            "nan",
            " +1.2345E+0",
            "+1.23456E+",
        ] {
            assert_eq!(parse_reading(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn silence_fails_before_a_reading_and_warns_after() {
        let t0 = Instant::now();
        let b = Silence::default();
        let mut q = Quiet::new(b, t0);
        assert_eq!(q.nothing(t0 + Duration::from_secs(29)), Silent::Wait);
        assert_eq!(q.nothing(t0 + Duration::from_secs(30)), Silent::GiveUp(30));
        assert_eq!(q.give_up_text(30), "The meter had no reading ready in 30 s.");
        let mut q = Quiet::new(b, t0);
        assert!(!q.reading(t0 + Duration::from_secs(1)));
        assert_eq!(q.nothing(t0 + Duration::from_secs(31)), Silent::Warn(30));
        assert_eq!(q.nothing(t0 + Duration::from_secs(60)), Silent::Wait, "once");
        assert_eq!(q.nothing(t0 + Duration::from_secs(601)), Silent::GiveUp(600));
        assert_eq!(q.give_up_text(600), "The meter has had no reading ready for 600 s.");
        assert!(q.reading(t0 + Duration::from_secs(602)), "back");
        assert!(!q.reading(t0 + Duration::from_secs(603)));
        assert_eq!(warn_text(30), "The meter has had no reading ready for 30 s; still waiting.");
    }

    #[test]
    fn limits_are_samples_or_time() {
        let l = Limits { samples: 2, msec: 0 };
        assert!(!l.met(1, Duration::from_secs(99)));
        assert!(l.met(2, Duration::ZERO));
        let t = Limits { samples: 0, msec: 500 };
        assert!(!t.met(9, Duration::from_millis(499)));
        assert!(t.met(0, Duration::from_millis(500)));
        assert!(!Limits::default().met(u64::MAX, Duration::MAX));
    }
}
