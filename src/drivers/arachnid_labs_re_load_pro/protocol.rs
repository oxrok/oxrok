// Counterpart in sigrok:
//   libsigrok/src/hardware/arachnid-labs-re-load-pro/api.c,
//   libsigrok/src/hardware/arachnid-labs-re-load-pro/protocol.c,
//   libsigrok/src/hardware/arachnid-labs-re-load-pro/protocol.h,
//   Copyright (C) 2015-2016 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Frank Stettner, Gerhard Sittig,
//   Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Re:load Pro's protocol: lines of text at 115200/8n1.
//!
//! The firmware's source is public (`arachnidlabs/reload-pro`, `comms.c`). A
//! request ends in CR or LF, and an answer in CR LF, so that reading up to the
//! CR leaves an LF for the next read:
//!
//! | Sent | Answer |
//! | --- | --- |
//! | `version` | `version 1.11` |
//! | `monitor 200` | none; `read ...` every 200 ms from then on |
//! | `monitor 0` | none |
//! | `read` | `read MA MV UAH UWH` (older firmware: `read MA MV`) |
//! | `set` | `set MA` |
//! | `set MA` | `set MA`, after `err ...` when out of range |
//! | `uvlo` | `uvlo MV`, with 0 for off |
//! | `uvlo MV` | `uvlo MV`, after `err ...` when out of range |
//! | `on`, `off` | `ok` |
//!
//! The device speaks by itself too: `overtemp` and `undervolt` when it turns
//! off, and `set MA` when the knob is turned. So a line that comes to a request
//! need not be its answer. `err Unknown command 'x'` comes for a word the
//! firmware does not know: `uvlo` came with v1.9.

use crate::core::Result;
use crate::serial::{Port, read_blocking};
use std::time::{Duration, Instant};

/// The serial parameters unless `serialcomm=` says otherwise.
pub const SERIALCOMM: &str = "115200/8n1";
/// Stops monitoring.
pub const MONITOR_STOP: &[u8] = b"monitor 0\r\n";
/// Starts monitoring, at the fastest the C driver asks.
pub const MONITOR_START: &[u8] = b"monitor 200\r\n";
/// Asks for the version.
pub const VERSION: &[u8] = b"version\r\n";

/// The longest line: the C's buffers of 100, with room for its NUL.
pub const LINE_MAX: usize = 99;
/// How long a scan waits for the version line.
pub const VERSION_TIMEOUT: Duration = Duration::from_secs(3);
/// How long a request waits for a line.
pub const REPLY_TIMEOUT: Duration = Duration::from_millis(500);
/// Silent reads that make a request unanswered, as the C's retry has it.
const SILENT_READS: u32 = 2;
/// Lines passed over before a request counts as unanswered: the device's own
/// and empty ones, each CR LF being two. It speaks far less often than this in
/// the time of a reply.
const OTHER_LINES: u32 = 16;

/// `serial_readline`: up to CR or LF, which goes, up to [`LINE_MAX`] bytes, or
/// what has come by the time. An empty result is an empty line or nothing.
///
/// # Errors
///
/// What the port's read gives, which in C is a read that found nothing.
pub fn read_line(port: &mut dyn Port, timeout: Duration) -> Result<Vec<u8>> {
    Ok(read_line_ended(port, timeout)?.0)
}

/// [`read_line`], and whether something ended the line before the time did.
fn read_line_ended(port: &mut dyn Port, timeout: Duration) -> Result<(Vec<u8>, bool)> {
    let deadline = Instant::now() + timeout;
    let mut line = Vec::new();
    while line.len() < LINE_MAX {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return Ok((line, false));
        }
        let mut byte = [0];
        if read_blocking(port, &mut byte, left)? == 0 {
            return Ok((line, false));
        }
        if matches!(byte[0], b'\r' | b'\n') {
            return Ok((line, true));
        }
        line.push(byte[0]);
    }
    Ok((line, true))
}

/// What a C string of these bytes holds.
pub fn c_string(line: &[u8]) -> &[u8] {
    line.split(|&b| b == 0).next().unwrap_or(line)
}

/// The version a `version` line says, if it is one. `buf + 8`, for at least a
/// character after the word.
pub fn version_of(line: &[u8]) -> Option<&[u8]> {
    if line.len() < 9 {
        return None;
    }
    line.strip_prefix(b"version ").map(c_string)
}

/// `g_ascii_strtod`, with 0 where it converts nothing. Firmware >= 1.10 is what
/// C asks, as a float: 1.9 is above it, and so 1.2.
pub fn number(text: &[u8]) -> f64 {
    crate::drivers::manson_hcs_3xxx::protocol::strtod(c_string(text)).unwrap_or(0.0)
}

/// What the firmware's whole numbers of milli are in floats, as C keeps them.
fn milli(text: &[u8]) -> f32 {
    (number(text) / 1000.0) as f32
}

/// `round(value * 1000)`: the product in single precision, as C has it.
pub fn to_milli(value: f32) -> i32 {
    f64::from(value * 1000.0).round() as i32
}

/// A line from the device.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Line {
    /// `overtemp`: it has turned off.
    OverTemperature,
    /// `undervolt`: it has turned off.
    UnderVoltage,
    /// `err ...`.
    Refused,
    /// `set MA`, in amperes.
    CurrentLimit(f32),
    /// `uvlo MV`, in volts.
    Threshold(f32),
    /// `read MA MV ...`, in volts and amperes.
    Reading {
        /// Volts.
        voltage: f32,
        /// Amperes.
        current: f32,
    },
    /// `read` with too few numbers, on which C dies.
    ShortReading,
    /// `ok`.
    Done,
    /// Anything else.
    Unknown,
}

/// What a line is, by how it starts, as `handle_packet`.
pub fn parse(line: &[u8]) -> Line {
    let line = c_string(line);
    if line.starts_with(b"overtemp") {
        return Line::OverTemperature;
    }
    if line.starts_with(b"undervolt") {
        return Line::UnderVoltage;
    }
    if line.starts_with(b"err ") {
        return Line::Refused;
    }
    if let Some(value) = line.strip_prefix(b"set ") {
        return Line::CurrentLimit(milli(value));
    }
    if let Some(value) = line.strip_prefix(b"uvlo ") {
        return Line::Threshold(milli(value));
    }
    if line == b"ok" {
        return Line::Done;
    }
    if !line.starts_with(b"read ") {
        return Line::Unknown;
    }
    let mut tokens = line.splitn(3, |&b| b == b' ').skip(1);
    match (tokens.next(), tokens.next()) {
        (Some(current), Some(voltage)) => {
            Line::Reading { voltage: milli(voltage), current: milli(current) }
        }
        _ => Line::ShortReading,
    }
}

/// What a request is waiting for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Awaited {
    /// `read`'s line.
    Reading,
    /// `set`'s line.
    CurrentLimit,
    /// `uvlo`'s line.
    Threshold,
    /// `ok`.
    Done,
}

impl Awaited {
    /// Whether this line is its answer.
    fn answered_by(self, line: &Line) -> bool {
        matches!(
            (self, line),
            (Awaited::Reading, Line::Reading { .. } | Line::ShortReading)
                | (Awaited::CurrentLimit, Line::CurrentLimit(_))
                | (Awaited::Threshold, Line::Threshold(_))
                | (Awaited::Done, Line::Done)
        )
    }
}

/// The answer to a request.
#[derive(Debug, PartialEq)]
pub enum Answer {
    /// It came.
    Line(Line),
    /// `err ...`, with the text.
    Refused(String),
    /// Nothing came that answers it.
    None,
}

/// The answer to a request, up to [`OTHER_LINES`] of what else comes going to
/// `note`: the lines the device sent by itself.
///
/// A read that fails finds nothing, as in `serial_readline`. A setting's next
/// write says what became of the port.
pub fn await_answer(port: &mut dyn Port, awaited: Awaited, note: &mut dyn FnMut(&Line)) -> Answer {
    let mut silent = 0;
    let mut others = 0;
    while silent < SILENT_READS && others < OTHER_LINES {
        let (line, ended) = read_line_ended(port, REPLY_TIMEOUT).unwrap_or_else(|e| {
            log::debug!("A read failed: {e}.");
            (Vec::new(), false)
        });
        if line.is_empty() {
            if ended {
                others += 1;
            } else {
                silent += 1;
            }
            continue;
        }
        let parsed = parse(&line);
        if awaited.answered_by(&parsed) {
            return Answer::Line(parsed);
        }
        if parsed == Line::Refused {
            return Answer::Refused(String::from_utf8_lossy(c_string(&line)).into_owned());
        }
        log::debug!("Passed over: '{}'.", String::from_utf8_lossy(&line));
        note(&parsed);
        others += 1;
    }
    Answer::None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Error;
    use crate::serial::Params;
    use std::collections::VecDeque;

    struct Bytes {
        input: VecDeque<u8>,
        fail: bool,
    }

    fn bytes(input: &[u8]) -> Bytes {
        Bytes { input: input.iter().copied().collect(), fail: false }
    }

    impl Port for Bytes {
        fn name(&self) -> &'static str {
            "fake"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], _: Duration) -> Result<usize> {
            if self.fail {
                return Err(Error::device("gone"));
            }
            let n = self.input.len().min(buf.len());
            for b in &mut buf[..n] {
                *b = self.input.pop_front().expect("counted");
            }
            Ok(n)
        }
        fn write(&mut self, _: &[u8]) -> Result<()> {
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            Ok(())
        }
    }

    const SHORT: Duration = Duration::from_millis(20);

    #[test]
    fn a_line_ends_at_cr_or_lf_and_leaves_the_rest() {
        let mut p = bytes(b"one\r\ntwo\nthree");
        assert_eq!(read_line(&mut p, SHORT).unwrap(), b"one");
        assert_eq!(read_line(&mut p, SHORT).unwrap(), b"", "the LF");
        assert_eq!(read_line(&mut p, SHORT).unwrap(), b"two");
        assert_eq!(read_line(&mut p, SHORT).unwrap(), b"three", "what came by the time");
        assert_eq!(read_line(&mut p, SHORT).unwrap(), b"");
    }

    #[test]
    fn a_line_stops_at_99_bytes() {
        let mut p = bytes(&[b'x'; 120]);
        assert_eq!(read_line(&mut p, SHORT).unwrap().len(), 99);
        assert_eq!(read_line(&mut p, SHORT).unwrap().len(), 21);
    }

    #[test]
    fn a_port_that_fails_is_an_error() {
        let mut p = bytes(b"");
        p.fail = true;
        assert!(read_line(&mut p, SHORT).is_err());
    }

    #[test]
    fn a_version_needs_something_after_the_word() {
        assert_eq!(version_of(b"version 1.11"), Some(&b"1.11"[..]));
        assert_eq!(version_of(b"version 1"), Some(&b"1"[..]));
        assert_eq!(version_of(b"version "), None);
        assert_eq!(version_of(b"version"), None);
        assert_eq!(version_of(b"ok"), None);
        assert_eq!(version_of(b"Version 1.11"), None);
        assert_eq!(version_of(b"version 1.1\x001"), Some(&b"1.1"[..]));
    }

    #[test]
    fn numbers_are_what_strtod_makes_of_them() {
        assert_eq!(number(b"1.11"), 1.11);
        assert_eq!(number(b"1.11x"), 1.11);
        assert_eq!(number(b"abc"), 0.0);
        assert_eq!(number(b""), 0.0);
        assert_eq!(number(b"12000 3 4"), 12000.0);
        assert_eq!(number(b"1\x002"), 1.0);
    }

    #[test]
    fn the_product_is_in_single_precision() {
        assert_eq!(to_milli(1.5), 1500);
        assert_eq!(to_milli(0.0), 0);
        assert_eq!(to_milli(-0.0), 0);
        assert_eq!(to_milli(6.0), 6000);
        // 2.0005f is a shade below; the float product is 2000.5, the double's
        // 2000.49996.
        assert_eq!(to_milli(2.0005), 2001);
        assert_eq!((f64::from(2.0005f32) * 1000.0).round() as i32, 2000);
    }

    #[test]
    fn the_lines() {
        assert_eq!(parse(b"overtemp"), Line::OverTemperature);
        assert_eq!(parse(b"overtemperature"), Line::OverTemperature);
        assert_eq!(parse(b"undervolt"), Line::UnderVoltage);
        assert_eq!(parse(b"err Unknown command 'x'"), Line::Refused);
        assert_eq!(parse(b"err"), Line::Unknown);
        assert_eq!(parse(b"set 1500"), Line::CurrentLimit(1.5));
        assert_eq!(parse(b"set "), Line::CurrentLimit(0.0));
        assert_eq!(parse(b"set"), Line::Unknown);
        assert_eq!(parse(b"uvlo 3300"), Line::Threshold(3.3));
        assert_eq!(parse(b"ok"), Line::Done);
        assert_eq!(parse(b"okay"), Line::Unknown);
        assert_eq!(parse(b"read"), Line::Unknown);
        assert_eq!(parse(b"read "), Line::ShortReading);
        assert_eq!(parse(b"read 5"), Line::ShortReading);
        assert_eq!(parse(b"read 1500 12345"), Line::Reading { voltage: 12.345, current: 1.5 });
        assert_eq!(
            parse(b"read 1500 12345 77 88"),
            Line::Reading { voltage: 12.345, current: 1.5 },
            "the firmware's four numbers"
        );
        assert_eq!(parse(b"read x y"), Line::Reading { voltage: 0.0, current: 0.0 });
        assert_eq!(parse(b"read 1\x00 2"), Line::ShortReading, "a C string ends there");
        assert_eq!(parse(b""), Line::Unknown);
    }

    fn answer(input: &[u8], awaited: Awaited) -> (Answer, Vec<Line>) {
        let mut p = bytes(input);
        let mut noted = Vec::new();
        let got = await_answer(&mut p, awaited, &mut |line| noted.push(*line));
        (got, noted)
    }

    #[test]
    fn an_answer_gets_past_what_the_device_says_by_itself() {
        assert_eq!(answer(b"ok\r\n", Awaited::Done).0, Answer::Line(Line::Done));
        assert_eq!(
            answer(b"\nset 100\r\n", Awaited::CurrentLimit).0,
            Answer::Line(Line::CurrentLimit(0.1)),
            "the LF of the last answer"
        );
        let (got, noted) =
            answer(b"overtemp\r\nundervolt\r\nset 5\r\nuvlo 100\r\n", Awaited::Threshold);
        assert_eq!(got, Answer::Line(Line::Threshold(0.1)));
        assert_eq!(noted, [Line::OverTemperature, Line::UnderVoltage, Line::CurrentLimit(0.005)]);
    }

    #[test]
    fn a_refusal_says_so_and_silence_is_no_answer() {
        assert_eq!(
            answer(b"err Unknown command 'uvlo'\r\nuvlo 0\r\n", Awaited::Threshold).0,
            Answer::Refused("err Unknown command 'uvlo'".to_owned())
        );
        assert_eq!(answer(b"", Awaited::Done).0, Answer::None);
        assert_eq!(answer(b"nonsense\r\n", Awaited::Done).0, Answer::None);
    }

    #[test]
    fn what_is_passed_over_is_bounded() {
        let mut input = b"overtemp\n".repeat(16);
        input.extend_from_slice(b"ok\r\n");
        let (got, noted) = answer(&input, Awaited::Done);
        assert_eq!(got, Answer::None);
        assert_eq!(noted.len(), 16);
        let mut input = b"\n".repeat(16);
        input.extend_from_slice(b"ok\r\n");
        assert_eq!(answer(&input, Awaited::Done).0, Answer::None);
        let mut input = b"\n".repeat(15);
        input.extend_from_slice(b"ok\r\n");
        assert_eq!(answer(&input, Awaited::Done).0, Answer::Line(Line::Done));
    }

    #[test]
    fn a_read_that_fails_finds_nothing() {
        let mut p = bytes(b"ok\r\n");
        p.fail = true;
        assert_eq!(await_answer(&mut p, Awaited::Done, &mut |_| ()), Answer::None);
    }

    #[test]
    fn a_silent_device_takes_two_reads() {
        let started = Instant::now();
        assert_eq!(answer(b"", Awaited::Done).0, Answer::None);
        let took = started.elapsed();
        assert!(took >= Duration::from_millis(1000), "{took:?}");
    }
}
