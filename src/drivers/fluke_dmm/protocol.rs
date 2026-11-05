// Counterpart in sigrok:
//   libsigrok/src/hardware/fluke-dmm/api.c,
//   libsigrok/src/hardware/fluke-dmm/protocol.c,
//   libsigrok/src/hardware/fluke-dmm/protocol.h,
//   libsigrok/src/hardware/fluke-dmm/fluke-18x.c,
//   libsigrok/src/hardware/fluke-dmm/fluke-190.c,
//   libsigrok/src/hardware/fluke-dmm/fluke-28x.c,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2019-2020, 2024 Andreas Sandberg <andreas@sandberg.uk>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Martin Ling,
//   Daniel Elstner, Soeren Apel, Gerhard Sittig, Mathieu Pilato
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The meters' protocol. A command ends in CR, and each answer starts with
//! a one digit acknowledge and CR: `0` is followed by the data and CR,
//! anything else by nothing.
//!
//! `ID` answers `FLUKE MODEL,VERSION,SERIAL`, or on a `ScopeMeter`
//! `FLUKE MODEL;VERSION;DATE;LANGUAGES`. `QM` answers, by family:
//!
//! - 87, 89, 187, 189: `QM,VALUE UNIT`, the display as text, or
//!   `QM,Out of range. UNIT`;
//! - 287, 289: `VALUE,UNIT,STATE,ATTRIBUTE`;
//! - 199B: sets of `NO,VALID,SOURCE,UNIT,TYPE,PRES,RESOL`, what the display
//!   shows. `QM NO` then answers the value.
//!
//! Everything here but [`read_line`] is pure, and takes the bytes a meter
//! sent: nothing says they are text.

use crate::core::{Mq, MqFlags, Result, Unit};
use crate::serial::{Port, read_blocking};
use std::time::{Duration, Instant};

/// `FLUKEDMM_BUFSIZE`, less the terminating NUL: what a capture's line may
/// hold, its CR included.
pub const LINE_MAX: usize = 255;
/// What the scan's `buf` holds: `serial_readline` keeps room for the NUL.
pub const SCAN_LINE_MAX: usize = 127;

/// How a meter says what it measured.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    /// `fluke-18x.c`.
    Display,
    /// `fluke-190.c`.
    Scope,
    /// `fluke-28x.c`.
    Fields,
}

/// A row of `supported_flukedmm`.
#[derive(Debug, PartialEq, Eq)]
pub struct Profile {
    /// What the ID has after `FLUKE `.
    pub name: &'static str,
    /// Which parser its answers go to.
    pub family: Family,
    /// Between two `QM`, once the last has been answered.
    pub poll_period: Duration,
    /// Between two `QM` when the last has not.
    pub timeout: Duration,
}

const fn profile(name: &'static str, family: Family, period: u64, timeout: u64) -> Profile {
    Profile {
        name,
        family,
        poll_period: Duration::from_millis(period),
        timeout: Duration::from_millis(timeout),
    }
}

/// `supported_flukedmm`.
pub static PROFILES: [Profile; 7] = [
    profile("87", Family::Display, 100, 1000),
    profile("89", Family::Display, 100, 1000),
    profile("187", Family::Display, 100, 1000),
    profile("189", Family::Display, 100, 1000),
    profile("199B", Family::Scope, 1000, 3500),
    profile("287", Family::Fields, 100, 1000),
    profile("289", Family::Fields, 100, 1000),
];

/// The bytes up to the first NUL, as C's string functions see them.
pub fn c_string(bytes: &[u8]) -> &[u8] {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    &bytes[..end]
}

fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}

fn contains(haystack: &[u8], needle: &[u8]) -> bool {
    find(haystack, needle).is_some()
}

fn shown(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

/// C's `isspace` in the C locale.
fn is_space(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

/// `strtol(text, &end, 10)`: the value and how much it used. `None` for
/// `ERANGE`.
fn strtol(text: &[u8]) -> Option<(i64, usize)> {
    let mut at = text.iter().take_while(|&&b| is_space(b)).count();
    let negative = text.get(at) == Some(&b'-');
    if matches!(text.get(at), Some(b'-' | b'+')) {
        at += 1;
    }
    let digits = text[at..].iter().take_while(|b| b.is_ascii_digit()).count();
    if digits == 0 {
        return Some((0, 0));
    }
    let mut value = 0i64;
    for &b in &text[at..at + digits] {
        let digit = i64::from(b - b'0');
        let next = value.checked_mul(10)?;
        value = if negative { next.checked_sub(digit)? } else { next.checked_add(digit)? };
    }
    Some((value, at + digits))
}

/// `strtol` for a number the C keeps in an `int`, its `ERANGE` result
/// (`LONG_MAX`, `LONG_MIN`) included.
fn c_int(text: &[u8]) -> i32 {
    let saturated = || {
        let negative = text.iter().find(|&&b| !is_space(b)) == Some(&b'-');
        if negative { i64::MIN } else { i64::MAX }
    };
    let long = strtol(text).map_or_else(saturated, |(value, _)| value);
    long as i32
}

/// `sr_count_digits`, the decimals a text has. It takes a sign, digits, a
/// point, digits and an exponent, so neither `inf`, `nan` nor hexadecimal.
fn count_digits(text: &[u8]) -> Option<i32> {
    let mut at = text.iter().take_while(|&&b| is_space(b)).count();
    if matches!(text.get(at), Some(b'-' | b'+')) {
        at += 1;
    }
    let whole = text[at..].iter().take_while(|b| b.is_ascii_digit()).count();
    at += whole;
    let mut decimals = 0;
    if text.get(at) == Some(&b'.') {
        at += 1;
        decimals = text[at..].iter().take_while(|b| b.is_ascii_digit()).count();
        at += decimals;
    }
    let mut exponent = 0;
    if matches!(text.get(at), Some(b'e' | b'E')) {
        at += 1;
        let (long, used) = strtol(&text[at..])?;
        exponent = long as i32;
        at += used;
    }
    if at != text.len() || (whole == 0 && decimals == 0) {
        return None;
    }
    i32::try_from(decimals).ok().map(|d| d.wrapping_sub(exponent))
}

/// `sr_atof_ascii_digits`: a whole text as a number and its decimals, or
/// `None`. A value too large or too small for a double is `strtod`'s
/// `ERANGE`, and one too large for a float is its infinity, as in C.
pub fn parse_float(text: &[u8]) -> Option<(f32, i32)> {
    let digits = count_digits(text)?;
    let start = text.iter().take_while(|&&b| is_space(b)).count();
    let number = std::str::from_utf8(&text[start..]).ok()?;
    let value: f64 = number.parse().ok()?;
    let mantissa = number.split(['e', 'E']).next().unwrap_or("");
    let nonzero = mantissa.bytes().any(|b| (b'1'..=b'9').contains(&b));
    if value.is_infinite() || (nonzero && value.abs() < f64::MIN_POSITIVE) {
        return None;
    }
    Some((value as f32, digits))
}

/// `serial_readline`: up to CR or LF, which goes, up to `max` bytes, or what
/// has come by the time. A read that fails is one that found nothing, as
/// there.
pub fn read_line(port: &mut dyn Port, max: usize, timeout: Duration) -> Vec<u8> {
    let start = Instant::now();
    let mut line = Vec::new();
    let mut left = timeout;
    while line.len() < max {
        let mut byte = [0];
        let got = read_blocking(port, &mut byte, left).unwrap_or(0);
        if got > 0 {
            if matches!(byte[0], b'\r' | b'\n') {
                break;
            }
            line.push(byte[0]);
        }
        let elapsed = start.elapsed();
        if elapsed >= timeout {
            break;
        }
        left = timeout.saturating_sub(elapsed);
        if got == 0 {
            std::thread::sleep(Duration::from_millis(2).min(left));
        }
    }
    if !line.is_empty() {
        log::debug!("Received {}: '{}'.", line.len(), shown(c_string(&line)));
    }
    line
}

/// What an ID says.
#[derive(Debug, PartialEq, Eq)]
pub struct Identity {
    /// The row of the table.
    pub profile: &'static Profile,
    /// The version, without leading spaces.
    pub version: String,
}

/// An ID line, from `fluke_scan`: `None` for what it does not take. Which
/// separator goes by where the first comma is, as a `ScopeMeter`'s languages
/// may hold commas.
pub fn parse_identity(line: &[u8]) -> Option<Identity> {
    let text = c_string(line);
    let comma = text.iter().position(|&b| b == b',').unwrap_or(text.len());
    let separator = if comma < 15 { b',' } else { b';' };
    let mut tokens = text.splitn(3, |&b| b == separator);
    let first = tokens.next()?;
    let version = tokens.next()?;
    tokens.next()?;
    if !first.starts_with(b"FLUKE") {
        return None;
    }
    // Any byte may stand between FLUKE and the model, and C reads past the
    // end of "FLUKE" alone, which matches nothing here.
    let name = first.get(6..)?;
    let profile = PROFILES.iter().find(|p| p.name.as_bytes() == name)?;
    let spaces = version.iter().take_while(|&&b| b == b' ').count();
    Some(Identity { profile, version: shown(&version[spaces..]) })
}

/// One measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// In the unit, scaled.
    pub value: f32,
    /// What was measured.
    pub mq: Mq,
    /// How.
    pub flags: MqFlags,
    /// Its unit.
    pub unit: Unit,
    /// `sr_analog_init` takes these in an `int8_t`.
    pub digits: i32,
}

fn reading(value: f32, digits: i32, (mq, unit, flags): (Mq, Unit, MqFlags)) -> Reading {
    Reading { value, mq, flags, unit, digits: i32::from(digits as i8) }
}

/// What a line of a capture leads to.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Handled {
    /// A measurement to send, and count.
    pub reading: Option<Reading>,
    /// The meter has answered, so the next `QM` waits only for the poll
    /// period.
    pub answered: bool,
    /// What to send at once.
    pub ask: Option<String>,
}

/// The state of a capture's answers: `handle_line` and `dev_context`.
#[derive(Debug)]
pub struct Decoder {
    family: Family,
    /// What the last usable set of a `ScopeMeter` says the values are.
    meaning: Option<(Mq, Unit, MqFlags)>,
    /// Which reading to ask for: 0, 11 or 19.
    meas_type: i32,
}

impl Decoder {
    /// For a meter of this family, at the start of a capture. C keeps the
    /// `ScopeMeter`'s state from one capture to the next, where a new
    /// capture's first `QM` sets it anyway unless the display is off; here
    /// that state goes with the capture.
    pub fn new(family: Family) -> Decoder {
        Decoder { family, meaning: None, meas_type: 0 }
    }

    /// A whole line, without its CR.
    pub fn line(&mut self, line: &[u8]) -> Handled {
        if line.len() == 1 {
            if line[0] != b'0' {
                log::debug!("Got CMD_ACK '{}'.", line[0].escape_ascii());
                return Handled { answered: true, ..Handled::default() };
            }
            return Handled::default();
        }
        let text = c_string(line);
        if text.is_empty() {
            return Handled::default();
        }
        let tokens: Vec<&[u8]> = text.split(|&b| b == b',').collect();
        let mut handled = Handled { answered: true, ..Handled::default() };
        match self.family {
            Family::Display => handled.reading = display_reading(&tokens),
            Family::Fields => handled.reading = fields_reading(&tokens),
            Family::Scope => {
                handled.reading = self.scope_line(&tokens);
                if self.meas_type != 0 {
                    handled.ask = Some(format!("QM {}\r", self.meas_type));
                }
            }
        }
        handled
    }

    /// `fluke_handle_qm_190`.
    fn scope_line(&mut self, tokens: &[&[u8]]) -> Option<Reading> {
        if tokens.len() < 7 {
            return self.scope_value(tokens[0]);
        }
        self.meas_type = 0;
        for start in (0..tokens.len()).step_by(7) {
            self.scope_set(&tokens[start..]);
        }
        None
    }

    /// `handle_qm_19x_meta`, on what is left of the line from a set on.
    fn scope_set(&mut self, tokens: &[&[u8]]) {
        if tokens.len() < 7 || tokens[1] != b"1" || tokens[2] != b"3" {
            return;
        }
        let meas_type = c_int(tokens[0]);
        if meas_type != 11 && meas_type != 19 {
            return;
        }
        // Relative takes precedence over absolute.
        if meas_type == 11 && self.meas_type == 19 {
            return;
        }
        let meas_unit = c_int(tokens[3]);
        if meas_unit == 0 {
            // The device is turned off.
            return;
        }
        let meas_char = c_int(tokens[4]);
        let ac_dc = |char: i32| match char {
            1 => MqFlags::DC,
            2 => MqFlags::AC,
            3 => MqFlags::DC | MqFlags::AC,
            _ => MqFlags::empty(),
        };
        self.meaning = match (meas_unit, meas_char) {
            (1, 15) => Some((Mq::Voltage, Unit::Volt, MqFlags::DIODE | MqFlags::DC)),
            (1, c) => Some((Mq::Voltage, Unit::Volt, ac_dc(c))),
            (2, c) => Some((Mq::Current, Unit::Ampere, ac_dc(c))),
            (3, 1) => Some((Mq::Resistance, Unit::Ohm, MqFlags::empty())),
            (3, 16) => Some((Mq::Continuity, Unit::Boolean, MqFlags::empty())),
            (3, _) => None,
            (12, _) => Some((Mq::Temperature, Unit::Celsius, MqFlags::empty())),
            (13, _) => Some((Mq::Temperature, Unit::Fahrenheit, MqFlags::empty())),
            (other, _) => {
                log::debug!("unknown unit: {other}");
                None
            }
        };
        if self.meaning.is_some() {
            self.meas_type = meas_type;
        }
    }

    /// `handle_qm_19x_data`. A value of exactly 0 is refused, as there.
    fn scope_value(&self, token: &[u8]) -> Option<Reading> {
        let (mut value, digits) = if token == b"9.9E+37" {
            (f32::NAN, 2)
        } else {
            match parse_float(token) {
                Some((value, digits)) if value != 0.0 => (value, digits),
                _ => {
                    log::error!("Invalid float '{}'.", shown(token));
                    return None;
                }
            }
        };
        let meaning = self.meaning?;
        match meaning.0 {
            Mq::Resistance if value.is_nan() => value = f32::INFINITY,
            Mq::Continuity => value = if value.is_nan() { 0.0 } else { 1.0 },
            _ => {}
        }
        Some(reading(value, digits, meaning))
    }
}

/// `fluke_handle_qm_18x`. A value with no space after it has no unit, so no
/// sample, where C reads on past its string.
fn display_reading(tokens: &[&[u8]]) -> Option<Reading> {
    if tokens[0] != b"QM" {
        return None;
    }
    let display = *tokens.get(1)?;
    let out_of_range = find(display, b"Out of range");
    let (mut value, digits, tail): (f32, i32, &[u8]) = if let Some(at) = out_of_range {
        let from = &display[at..];
        let dot = from.iter().position(|&b| b == b'.').unwrap_or(from.len());
        (f32::NAN, 0, &from[dot..])
    } else {
        let end = display.iter().position(|&b| b == b' ').unwrap_or(display.len());
        let number = &display[..end];
        let Some((value, digits)) = parse_float(number) else {
            // Happens all the time, when switching modes.
            log::debug!("Invalid float: '{}'", shown(number));
            return None;
        };
        (value, digits, display.get(end + 1..).unwrap_or(b""))
    };
    let oor = out_of_range.is_some();
    let spaces = tail.iter().take_while(|&&b| b == b' ').count();
    let e = &tail[spaces..];
    let prefix = e.first().copied();
    let scaled = |table: &[(u8, i32)]| -> i32 {
        if oor {
            return 0;
        }
        table.iter().find(|(p, _)| Some(*p) == prefix).map_or(0, |(_, x)| *x)
    };
    let ac_dc = |from: &[u8]| {
        let mut flags = MqFlags::empty();
        if contains(from, b"AC") {
            flags |= MqFlags::AC | MqFlags::RMS;
        }
        if contains(from, b"DC") {
            flags |= MqFlags::DC;
        }
        flags
    };
    let any = |a: &[u8], b: &[u8]| find(e, a).or_else(|| find(e, b));
    let none = MqFlags::empty();
    let (meaning, exponent) = if let Some(u) = any(b"V DC", b"V AC") {
        ((Mq::Voltage, Unit::Volt, ac_dc(&e[u..])), scaled(&[(b'm', -3)]))
    } else if let Some(u) = any(b"dBV", b"dBm") {
        let unit = if e[u + 2] == b'm' { Unit::DecibelMw } else { Unit::DecibelVolt };
        ((Mq::Voltage, unit, MqFlags::AC | MqFlags::RMS), 0)
    } else if contains(e, b"Ohms") {
        if oor {
            value = f32::INFINITY;
        }
        ((Mq::Resistance, Unit::Ohm, none), scaled(&[(b'k', 3), (b'M', 6)]))
    } else if e == b"nS" {
        ((Mq::Conductance, Unit::Siemens, none), -9)
    } else if contains(e, b"Farads") {
        ((Mq::Capacitance, Unit::Farad, none), scaled(&[(b'm', -3), (b'u', -6), (b'n', -9)]))
    } else if let Some(u) = any(b"Deg C", b"Deg F") {
        let unit = if e[u + 4] == b'C' { Unit::Celsius } else { Unit::Fahrenheit };
        ((Mq::Temperature, unit, none), 0)
    } else if let Some(u) = any(b"A AC", b"A DC") {
        ((Mq::Current, Unit::Ampere, ac_dc(&e[u..])), scaled(&[(b'm', -3), (b'u', -6)]))
    } else if contains(e, b"Hz") {
        let exponent = if prefix == Some(b'k') { 3 } else { 0 };
        ((Mq::Frequency, Unit::Hertz, none), exponent)
    } else if e == b"%" {
        ((Mq::DutyCycle, Unit::Percentage, none), 0)
    } else if contains(e, b"ms") {
        ((Mq::PulseWidth, Unit::Second, none), -3)
    } else {
        return None;
    };
    let scale = match exponent {
        -9 => 1e-9,
        -6 => 1e-6,
        -3 => 1e-3,
        3 => 1e3,
        6 => 1e6,
        _ => 1.0,
    };
    let scaled_value = (f64::from(value) * scale) as f32;
    Some(reading(scaled_value, digits.wrapping_sub(exponent), meaning))
}

/// `unit_map`. `A` is a current here, where C's table says `SR_MQ_VOLTAGE`
/// (`fluke-28x.c:103`) beside the unit ampere.
fn fields_unit(name: &[u8]) -> Option<(Mq, Unit, MqFlags)> {
    let none = MqFlags::empty();
    let rms = MqFlags::AC | MqFlags::RMS;
    Some(match name {
        b"VDC" => (Mq::Voltage, Unit::Volt, MqFlags::DC),
        b"VAC" => (Mq::Voltage, Unit::Volt, rms),
        b"ADC" => (Mq::Current, Unit::Ampere, MqFlags::DC),
        b"AAC" => (Mq::Current, Unit::Ampere, rms),
        b"VAC_PLUS_DC" => (Mq::Voltage, Unit::Volt, rms | MqFlags::DC),
        b"AAC_PLUS_DC" => (Mq::Current, Unit::Ampere, rms | MqFlags::DC),
        b"V" => (Mq::Voltage, Unit::Volt, none),
        b"A" => (Mq::Current, Unit::Ampere, none),
        b"OHM" => (Mq::Resistance, Unit::Ohm, none),
        b"SIE" => (Mq::Conductance, Unit::Siemens, none),
        b"Hz" => (Mq::Frequency, Unit::Hertz, none),
        b"S" => (Mq::PulseWidth, Unit::Second, none),
        b"F" => (Mq::Capacitance, Unit::Farad, none),
        b"CEL" => (Mq::Temperature, Unit::Celsius, none),
        b"FAR" => (Mq::Temperature, Unit::Fahrenheit, none),
        b"PCT" => (Mq::DutyCycle, Unit::Percentage, none),
        b"dBm" => (Mq::Power, Unit::DecibelMw, rms),
        b"dBV" => (Mq::Voltage, Unit::DecibelVolt, rms),
        _ => return None,
    })
}

/// `state_map`.
const STATES: [&[u8]; 7] =
    [b"INVALID", b"NORMAL", b"BLANK", b"DISCHARGE", b"OL", b"OL_MINUS", b"OPEN_TC"];
/// `attribute_map`.
const ATTRIBUTES: [&[u8]; 9] = [
    b"NONE",
    b"OPEN_CIRCUIT",
    b"SHORT_CIRCUIT",
    b"GLITCH_CIRCUIT",
    b"GOOD_DIODE",
    b"LO_OHMS",
    b"NEGATIVE_EDGE",
    b"POSITIVE_EDGE",
    b"HIGH_CURRENT",
];

/// `fluke_handle_qm_28x`. A line of two tokens has no state and so no sample,
/// where C reads a pointer from past the `NULL` that ends its array
/// (`fluke-28x.c:196`) and dies in about half of the runs.
fn fields_reading(tokens: &[&[u8]]) -> Option<Reading> {
    let Some((mut value, digits)) = parse_float(tokens[0]) else {
        log::error!("Invalid float '{}'.", shown(tokens[0]));
        return None;
    };
    let Some(unit_name) = tokens.get(1) else {
        log::error!("Invalid unit '(null)'.");
        return None;
    };
    let Some(mut meaning) = fields_unit(unit_name) else {
        log::warn!("Unknown unit '{}'", shown(unit_name));
        log::error!("Invalid unit '{}'.", shown(unit_name));
        return None;
    };
    let state = tokens.get(2).copied();
    let attribute = tokens.get(3).copied();
    if let Some(s) = state.filter(|s| !STATES.contains(s)) {
        log::warn!("Unknown measurement state '{}'", shown(s));
    }
    if let Some(a) = attribute.filter(|a| !ATTRIBUTES.contains(a)) {
        log::warn!("Unknown measurement attribute '{}'", shown(a));
    }
    let mut measured = true;
    if meaning.0 == Mq::Resistance {
        match attribute {
            Some(b"NONE") => {}
            Some(b"OPEN_CIRCUIT") => {
                meaning.0 = Mq::Continuity;
                meaning.1 = Unit::Boolean;
                value = 0.0;
            }
            Some(b"SHORT_CIRCUIT") => {
                meaning.0 = Mq::Continuity;
                meaning.1 = Unit::Boolean;
                value = 1.0;
            }
            _ => measured = false,
        }
    }
    match state {
        Some(b"NORMAL") => {}
        Some(b"OL") => value = f32::INFINITY,
        Some(b"OL_MINUS") => value = f32::NEG_INFINITY,
        Some(b"OPEN_TC") => value = f32::NAN,
        _ => measured = false,
    }
    measured.then(|| reading(value, digits, meaning))
}

/// The answer to `ID`, as `fluke_scan` asks for it, on a port just purged:
/// what the first answer of the three that may come says, or `None`.
///
/// # Errors
///
/// When the port's write fails, which is one of the tries gone.
pub fn ask_identity(port: &mut dyn Port) -> Result<Option<Identity>> {
    port.write(b"ID\r")?;
    let ack = read_line(port, SCAN_LINE_MAX, Duration::from_millis(150));
    if ack != b"0" {
        return Ok(None);
    }
    let line = read_line(port, SCAN_LINE_MAX, Duration::from_millis(850));
    if line.len() < 10 {
        return Ok(None);
    }
    Ok(parse_identity(&line))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[test]
    fn floats_as_sr_atof_ascii_digits() {
        for (text, value, digits) in [
            (&b"1.5"[..], 1.5f32, 1),
            (b"+12.345", 12.345, 3),
            (b"-1", -1.0, 0),
            (b".5", 0.5, 1),
            (b"5.", 5.0, 0),
            (b"1E2", 100.0, -2),
            (b"1.5E-1", 0.15, 2),
            (b"1.0327E0", 1.0327, 4),
            (b" \t1.5", 1.5, 1),
            (b"1234.5678E-3", 1.234_567_8, 7),
            (b"+9.99999999E+37", 1e38, -29),
        ] {
            let (v, d) = parse_float(text).unwrap();
            assert_eq!((v, d), (value, digits), "{}", text.escape_ascii());
        }
        assert!(parse_float(b"-0.0").unwrap().0.is_sign_negative());
    }

    #[test]
    fn what_is_no_float() {
        for text in [
            &b""[..],
            b" ",
            b".",
            b"E5",
            b"x",
            b"1.2.3",
            b"1 ",
            b"1E",
            b"1E 5",
            b"1Ex",
            b"0x10",
            b"inf",
            b"-inf",
            b"nan",
            b"infinity",
            b"1E999",
            b"1E-400",
            b"1E-310",
            b"0E99999999999999999999",
            b"\xff",
        ] {
            assert_eq!(parse_float(text), None, "{}", text.escape_ascii());
        }
    }

    #[test]
    fn a_float_too_large_for_single_precision_is_its_infinity() {
        assert_eq!(parse_float(b"1E39"), Some((f32::INFINITY, -39)));
    }

    #[test]
    fn an_exponent_wraps_as_in_an_int() {
        // 3000000000 in an int is -1294967296.
        assert_eq!(parse_float(b"0E3000000000"), Some((0.0, 1_294_967_296)));
        let r = fields_reading(&[b"0E3000000000", b"VDC", b"NORMAL", b"NONE"]).unwrap();
        assert_eq!(r.digits, 0, "the low byte");
    }

    #[test]
    fn strtol_as_in_c() {
        assert_eq!(c_int(b"11"), 11);
        assert_eq!(c_int(b" 11abc"), 11);
        assert_eq!(c_int(b"-3"), -3);
        assert_eq!(c_int(b"x"), 0);
        assert_eq!(c_int(b""), 0);
        assert_eq!(c_int(b"99999999999999999999"), -1, "LONG_MAX in an int");
        assert_eq!(c_int(b"-99999999999999999999"), 0, "LONG_MIN in an int");
    }

    fn identity(line: &[u8]) -> Option<(&'static str, String)> {
        parse_identity(line).map(|i| (i.profile.name, i.version))
    }

    #[test]
    fn identities() {
        let v = |n: &'static str, s: &str| Some((n, s.to_owned()));
        assert_eq!(identity(b"FLUKE 287,V1.16,95680258"), v("287", "V1.16"));
        assert_eq!(identity(b"FLUKE 89,  V0.39,1"), v("89", "V0.39"));
        assert_eq!(identity(b"FLUKE 287,,"), v("287", ""));
        assert_eq!(identity(b"FLUKE 287,V1,2,3"), v("287", "V1"));
        assert_eq!(identity(b"FLUKE 287;V1.16;1"), v("287", "V1.16"));
        assert_eq!(identity(b"FLUKEX287,V1,2"), v("287", "V1"));
        assert_eq!(identity(b"FLUKE 199B;V11.40;D;A,B,C"), v("199B", "V11.40"));
        assert_eq!(identity(b"FLUKE 199B;V11.40,20;EN"), v("199B", "V11.40,20"));
        assert_eq!(identity(b"FLUKE 287,V1.16,\x001"), v("287", "V1.16"));
        assert_eq!(identity(b"FLUKE 199B;V1.4,0;E"), v("199B", "V1.4,0"), "a comma at 15");
    }

    #[test]
    fn what_is_no_identity() {
        for line in [
            &b"FLUKE 287,V1.16"[..],
            b"FLUKE 287",
            b"",
            b"fluke 287,V1,2",
            b"ACME 287,V1,2",
            b"FLUKE,V1,2",
            b"FLUKE 999,V1,2",
            b"FLUKE 2870,V1,2",
            b"FLUKE 28,V1,2",
            b"FLUKE 199B;V1,2,3;E",
            b"FLUKE 287,V1\x00,2",
            b"FLUKE 287;V1;2",
            b"FLUKE 199B;V1.,0;E",
        ] {
            assert_eq!(identity(line), None, "{}", line.escape_ascii());
        }
    }

    fn display(text: &[u8]) -> Option<Reading> {
        let mut d = Decoder::new(Family::Display);
        let mut line = b"QM,".to_vec();
        line.extend_from_slice(text);
        d.line(&line).reading
    }

    #[test]
    fn the_display_units() {
        let rms = MqFlags::AC | MqFlags::RMS;
        let none = MqFlags::empty();
        for (text, value, mq, unit, flags, digits) in [
            (&b"+1.234 V DC"[..], 1.234f32, Mq::Voltage, Unit::Volt, MqFlags::DC, 3),
            (b"+1.234 mV AC", 0.001_234, Mq::Voltage, Unit::Volt, rms, 6),
            (b"+1.5 V AC+DC", 1.5, Mq::Voltage, Unit::Volt, rms | MqFlags::DC, 1),
            (b"+1.5 dBV", 1.5, Mq::Voltage, Unit::DecibelVolt, rms, 1),
            (b"+1.5 dBm", 1.5, Mq::Voltage, Unit::DecibelMw, rms, 1),
            (b"+1.5 kOhms", 1500.0, Mq::Resistance, Unit::Ohm, none, -2),
            (b"+1.5 MOhms", 1_500_000.0, Mq::Resistance, Unit::Ohm, none, -5),
            (b"+1.5 Ohms", 1.5, Mq::Resistance, Unit::Ohm, none, 1),
            (b"+1.5 nS", 1.5e-9, Mq::Conductance, Unit::Siemens, none, 10),
            (b"+1.5 Farads", 1.5, Mq::Capacitance, Unit::Farad, none, 1),
            (b"+1.5 mFarads", 1.5e-3, Mq::Capacitance, Unit::Farad, none, 4),
            (b"+1.5 uFarads", 1.5e-6, Mq::Capacitance, Unit::Farad, none, 7),
            (b"+1.5 nFarads", 1.5e-9, Mq::Capacitance, Unit::Farad, none, 10),
            (b"+1.5 Deg C", 1.5, Mq::Temperature, Unit::Celsius, none, 1),
            (b"+1.5 Deg F", 1.5, Mq::Temperature, Unit::Fahrenheit, none, 1),
            (b"+1.5 A DC", 1.5, Mq::Current, Unit::Ampere, MqFlags::DC, 1),
            (b"+1.5 mA AC", 1.5e-3, Mq::Current, Unit::Ampere, rms, 4),
            (b"+1.5 uA AC+DC", 1.5e-6, Mq::Current, Unit::Ampere, rms | MqFlags::DC, 7),
            (b"+1.5 Hz", 1.5, Mq::Frequency, Unit::Hertz, none, 1),
            (b"+1.5 kHz", 1500.0, Mq::Frequency, Unit::Hertz, none, -2),
            (b"+50.0 %", 50.0, Mq::DutyCycle, Unit::Percentage, none, 1),
            (b"+1.5 ms", 1.5e-3, Mq::PulseWidth, Unit::Second, none, 4),
        ] {
            let r = display(text).unwrap_or_else(|| panic!("{}", text.escape_ascii()));
            let label = text.escape_ascii();
            assert_eq!((r.mq, r.unit, r.flags), (mq, unit, flags), "{label}");
            assert_eq!((r.value, r.digits), (value, digits), "{label}");
        }
    }

    #[test]
    fn out_of_range() {
        let r = display(b"Out of range. V DC").unwrap();
        assert!(r.value.is_nan());
        assert_eq!((r.mq, r.digits), (Mq::Voltage, 0));
        let r = display(b"Out of range. kOhms").unwrap();
        assert_eq!((r.value, r.digits), (f32::INFINITY, 0));
        let r = display(b"Out of range. mV DC").unwrap();
        assert!(r.value.is_nan());
        assert_eq!(r.digits, 0, "no prefix counts");
        let r = display(b"Out of range. ms").unwrap();
        assert_eq!(r.digits, 3, "this exponent has no such check");
        for none in [&b"Out of range. nS"[..], b"Out of range. %", b"Out of range V DC"] {
            assert_eq!(display(none), None, "{}", none.escape_ascii());
        }
    }

    #[test]
    fn what_gives_a_display_no_reading() {
        for text in [
            &b"+1.5"[..],
            b"+1.5 VDC",
            b"+1.5 parsecs",
            b"+1.5 mS",
            b"x V DC",
            b"Out of Range mVDC",
            b" +1.5 V DC",
        ] {
            assert_eq!(display(text), None, "{}", text.escape_ascii());
        }
        let mut d = Decoder::new(Family::Display);
        assert_eq!(d.line(b"XX,+1.5 V DC").reading, None);
        assert_eq!(d.line(b"QM").reading, None);
        assert!(d.line(b"QM").answered, "an answer all the same");
    }

    #[test]
    fn an_ack_is_an_answer_only_when_refused() {
        let mut d = Decoder::new(Family::Fields);
        assert_eq!(d.line(b"0"), Handled::default());
        assert!(d.line(b"5").answered);
        assert!(d.line(b"\0").answered, "a NUL is no 0");
        assert_eq!(d.line(b""), Handled::default());
        assert_eq!(d.line(b"\0abc"), Handled::default(), "an empty string");
    }

    #[test]
    fn the_fields() {
        let mut d = Decoder::new(Family::Fields);
        let r = d.line(b"1.0327E0,ADC,NORMAL,NONE").reading.unwrap();
        assert_eq!((r.value, r.mq, r.unit), (1.0327, Mq::Current, Unit::Ampere));
        assert_eq!((r.flags, r.digits), (MqFlags::DC, 4));
        let r = d.line(b"1,A,NORMAL,NONE").reading.unwrap();
        assert_eq!(r.mq, Mq::Current, "fluke-28x.c:103 says voltage");
        let r = d.line(b"1,VDC,OL,NONE").reading.unwrap();
        assert_eq!(r.value, f32::INFINITY);
        let r = d.line(b"1,VDC,OL_MINUS,NONE").reading.unwrap();
        assert_eq!(r.value, f32::NEG_INFINITY);
        assert!(d.line(b"1,CEL,OPEN_TC,NONE").reading.unwrap().value.is_nan());
        let r = d.line(b"7,OHM,NORMAL,OPEN_CIRCUIT").reading.unwrap();
        assert_eq!((r.value, r.mq, r.unit), (0.0, Mq::Continuity, Unit::Boolean));
        let r = d.line(b"7,OHM,NORMAL,SHORT_CIRCUIT").reading.unwrap();
        assert_eq!(r.value, 1.0);
        let r = d.line(b"7,OHM,OL,OPEN_CIRCUIT").reading.unwrap();
        assert_eq!(r.value, f32::INFINITY, "the state comes last");
        let r = d.line(b"7,VDC,NORMAL").reading.unwrap_or_else(|| {
            panic!("no attribute is only a resistance's business");
        });
        assert_eq!(r.value, 7.0);
        for line in [
            &b"1,VDC,BLANK,NONE"[..],
            b"1,VDC,DISCHARGE,NONE",
            b"1,VDC,INVALID,NONE",
            b"1,VDC,WHAT,NONE",
            b"1,OHM,NORMAL,GOOD_DIODE",
            b"1,OHM,NORMAL,WHAT",
            b"1,OHM,NORMAL",
            b"1,VDC",
            b"1.5",
            b"x,VDC,NORMAL,NONE",
            b"1,VOLTS,NORMAL,NONE",
        ] {
            let h = d.line(line);
            assert_eq!(h.reading, None, "{}", line.escape_ascii());
            assert!(h.answered, "{}", line.escape_ascii());
        }
    }

    fn scope(sets: &[u8]) -> Decoder {
        let mut d = Decoder::new(Family::Scope);
        d.line(sets);
        d
    }

    #[test]
    fn a_scope_asks_for_what_it_shows() {
        let mut d = Decoder::new(Family::Scope);
        assert_eq!(d.line(b"11,1,3,1,1,0,1").ask.as_deref(), Some("QM 11\r"));
        assert_eq!(d.line(b"19,1,3,1,1,0,1").ask.as_deref(), Some("QM 19\r"));
        let h = d.line(b"+1.5E+00");
        assert_eq!(h.ask.as_deref(), Some("QM 19\r"), "a value asks for the next");
        assert_eq!(h.reading.unwrap().value, 1.5);
        assert_eq!(d.line(b"0"), Handled::default(), "an ack asks for nothing");
    }

    #[test]
    fn a_scope_keeps_its_flags() {
        for (char, flags) in [
            (1, MqFlags::DC),
            (2, MqFlags::AC),
            (3, MqFlags::AC | MqFlags::DC),
            (15, MqFlags::DIODE | MqFlags::DC),
            (7, MqFlags::empty()),
        ] {
            let mut d = scope(format!("11,1,3,1,{char},0,1").as_bytes());
            let r = d.line(b"1.5E0").reading.unwrap();
            assert_eq!((r.mq, r.flags), (Mq::Voltage, flags), "{char}");
        }
        for (char, flags) in [
            (1, MqFlags::DC),
            (2, MqFlags::AC),
            (3, MqFlags::AC | MqFlags::DC),
            (7, MqFlags::empty()),
        ] {
            let mut d = scope(format!("11,1,3,2,{char},0,1").as_bytes());
            let r = d.line(b"1.5E0").reading.unwrap();
            assert_eq!((r.mq, r.unit, r.flags), (Mq::Current, Unit::Ampere, flags), "{char}");
        }
    }

    #[test]
    fn scope_sets_that_ask_for_nothing() {
        for set in [
            &b"11,0,3,1,1,0,1"[..],
            b"11,1,1,1,1,0,1",
            b"21,1,3,1,1,0,1",
            b"11,1,3,0,1,0,1",
            b"11,1,3,4,1,0,1",
            b"11,1,3,3,7,0,1",
        ] {
            let mut d = Decoder::new(Family::Scope);
            let h = d.line(set);
            assert_eq!(h.ask, None, "{}", set.escape_ascii());
            assert!(h.answered);
            assert_eq!(d.line(b"1.5E0").reading, None, "no meaning yet");
        }
    }

    #[test]
    fn relative_takes_precedence() {
        let mut d = Decoder::new(Family::Scope);
        let h = d.line(b"19,1,3,1,1,1,1,11,1,3,3,1,0,1");
        assert_eq!(h.ask.as_deref(), Some("QM 19\r"));
        assert_eq!(d.line(b"1.5E0").reading.unwrap().mq, Mq::Voltage, "the first set's");
        let mut d = Decoder::new(Family::Scope);
        let h = d.line(b"11,1,3,3,1,0,1,19,1,3,1,1,1,1");
        assert_eq!(h.ask.as_deref(), Some("QM 19\r"));
        assert_eq!(d.line(b"1.5E0").reading.unwrap().mq, Mq::Voltage, "the second set's");
    }

    #[test]
    fn a_later_set_it_cannot_read_leaves_no_meaning() {
        let mut d = Decoder::new(Family::Scope);
        let h = d.line(b"11,1,3,1,1,0,1,19,1,3,4,1,1,1");
        assert_eq!(h.ask.as_deref(), Some("QM 11\r"), "the first set's");
        assert_eq!(d.line(b"1.5E0").reading, None);
    }

    #[test]
    fn a_set_not_valid_keeps_the_last_meaning() {
        let mut d = scope(b"11,1,3,3,1,0,1");
        assert_eq!(d.line(b"11,0,3,1,1,0,1").ask, None);
        assert_eq!(d.line(b"1.5E0").reading.unwrap().mq, Mq::Resistance);
    }

    #[test]
    fn scope_values() {
        let mut d = scope(b"11,1,3,1,1,0,1");
        let r = d.line(b"9.9E+37").reading.unwrap();
        assert!(r.value.is_nan());
        assert_eq!(r.digits, 2);
        assert_eq!(d.line(b"xy").reading, None);
        assert_eq!(d.line(b"0.000E+00").reading, None, "0 is refused, as in C");
        assert_eq!(d.line(b"-0.0").reading, None);
        assert_eq!(d.line(b"xy").ask.as_deref(), Some("QM 11\r"), "and it goes on");
        assert_eq!(d.line(b"x").ask, None, "one byte is an acknowledge");
        let r = d.line(b"+1.5E+00,+2.5E+00,3").reading.unwrap();
        assert_eq!(r.value, 1.5);
        let mut d = scope(b"11,1,3,3,1,0,1");
        assert_eq!(d.line(b"9.9E+37").reading.unwrap().value, f32::INFINITY);
        let mut d = scope(b"11,1,3,3,16,0,1");
        let r = d.line(b"9.9E+37").reading.unwrap();
        assert_eq!((r.value, r.mq, r.digits), (0.0, Mq::Continuity, 2));
        assert_eq!(d.line(b"+1.5E+01").reading.unwrap().value, 1.0);
    }

    /// Bytes to read, one by one, with nothing after them.
    struct Bytes(VecDeque<u8>, usize);

    impl Port for Bytes {
        fn name(&self) -> &'static str {
            "bytes"
        }
        fn set_params(&mut self, _: crate::serial::Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], _: Duration) -> Result<usize> {
            self.1 += 1;
            match self.0.pop_front() {
                Some(b) => {
                    buf[0] = b;
                    Ok(1)
                }
                None => Ok(0),
            }
        }
        fn write(&mut self, _: &[u8]) -> Result<()> {
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            Ok(())
        }
    }

    fn bytes(data: &[u8]) -> Bytes {
        Bytes(data.iter().copied().collect(), 0)
    }

    const SHORT: Duration = Duration::from_millis(20);

    #[test]
    fn a_line_ends_at_cr_or_lf_or_its_time_or_its_room() {
        let mut p = bytes(b"ab\rcd\nef");
        assert_eq!(read_line(&mut p, 127, SHORT), b"ab");
        assert_eq!(read_line(&mut p, 127, SHORT), b"cd");
        assert_eq!(read_line(&mut p, 127, SHORT), b"ef");
        assert_eq!(read_line(&mut p, 127, SHORT), b"");
        let mut p = bytes(b"abcdef\r");
        assert_eq!(read_line(&mut p, 4, SHORT), b"abcd");
        assert_eq!(read_line(&mut p, 4, SHORT), b"ef", "the rest stays");
    }

    #[test]
    fn identity_takes_an_ack_and_a_line() {
        let mut p = bytes(b"0\rFLUKE 287,V1.16,1\r");
        assert_eq!(ask_identity(&mut p).unwrap().unwrap().profile.name, "287");
        for bad in [&b"1\r"[..], b"00\rFLUKE 287,V1.16,1\r", b"0\rFLUKE 87\r", b""] {
            let mut p = bytes(bad);
            assert_eq!(ask_identity(&mut p).unwrap(), None, "{}", bad.escape_ascii());
        }
    }
}
