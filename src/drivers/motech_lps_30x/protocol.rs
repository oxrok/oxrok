// Counterpart in sigrok:
//   libsigrok/src/hardware/motech-lps-30x/api.c,
//   libsigrok/src/hardware/motech-lps-30x/protocol.c,
//   libsigrok/src/hardware/motech-lps-30x/protocol.h,
//   Copyright (C) 2014 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com> (code from atten-pps3xxx),
//   with commits by Uwe Hermann, Lars-Peter Clausen, Martin Ling,
//   Gerhard Sittig, Aurelien Jacobs, Soeren Apel, Daniel Elstner,
//   Wolfram Sang, Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the Motech LPS-300 series, as far as an LPS-301 goes.
//!
//! ASCII at 2400 baud, 8n1, one command to a line ended CR LF, and each answer
//! closed by a line saying `OK`. The device may send up to four line ends in a
//! row, so `serial_readline`'s reading of one line per CR or LF needs up to five
//! reads to get to one line (`api.c`'s comment, and its two loops of five).
//!
//! - `MODEL` gives `LPS-...`, `VERSION` `Ver-...`, `STATUS` a decimal number
//!   whose bit 6 is the output (the series' manual, note 7);
//! - `VOUT1` and `IOUT1` give a number;
//! - `VSET1 %05.3f`, `ISET1 %05.4f` and `OUT<b>` answer `OK` alone.
//!
//! An answer of `ERROR` and `OK` is a failure. What C does with a read that
//! comes to no line is another matter: it has 250 ms for all of an answer, and
//! the read after those are up has 0, which `sp_blocking_read` takes as no
//! timeout. `serial_readline` then waits for one byte and breaks, so C hangs for
//! as long as the device stays silent, and a late reply fails after five tries.
//! Here a read ends when those 250 ms do, and the answer has failed.
//!
//! No line says which request it answers, and the device's manual has a command
//! before `OK` rejected. So after a request was left unanswered nothing goes out
//! until [`Settle`] says the line is free.

use crate::core::{Error, Result};
use crate::serial::{Params, Port, read_blocking, serial_timeout};
use std::time::{Duration, Instant};

/// `SERIALCOMM`.
pub const SERIALCOMM: &str = "2400/8n1/dtr=1/rts=1/flow=0";
/// `LINELEN_MAX`.
const LINELEN_MAX: usize = 50;
/// The longest line the capture's buffer takes, by its read loop's condition:
/// C stops reading for good at the 48th byte with no line end.
pub const CAPTURE_LINE_MAX: usize = LINELEN_MAX - 3;
/// `REQ_TIMEOUT_MS`.
pub const REQ_TIMEOUT: Duration = Duration::from_millis(250);
/// The tries of each of `lps_read_reply`'s loops.
const LINE_TRIES: usize = 5;
/// How long the line must stay silent before a request follows one that went
/// unanswered, unless its `OK` comes first. Much more than a late answer's bytes
/// take at 2400 baud, so what is late by up to this much is read and dropped.
pub const QUIET: Duration = Duration::from_millis(400);
/// The most a line that never falls silent can hold the next request.
const SETTLE_MAX: Duration = Duration::from_secs(1);
/// `sr_strerror(SR_ERR)`, all that C's messages have to say of a failed answer.
pub const GENERIC_ERROR: &str = "generic/unspecified error";

/// Min, max and step, as `api.c`'s table has them for the LPS-301.
pub type Range = [f64; 3];
/// The voltage target's range.
pub const VOLTAGE: Range = [0.0, 32.0, 0.01];
/// The current limit's range.
pub const CURRENT: Range = [0.005, 2.0, 0.001];

/// A reply as C reads it: a string, up to its first NUL.
pub fn c_string(reply: &[u8]) -> &[u8] {
    let n = reply.iter().position(|c| *c == 0).unwrap_or(reply.len());
    &reply[..n]
}

/// `g_ascii_isspace`, which VT is not.
pub fn is_glib_space(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | 0x0c | b'\r')
}

/// C's `isspace` in the C locale, as `strtol` and `strtod` skip it.
fn is_c_space(c: u8) -> bool {
    is_glib_space(c) || c == 0x0b
}

fn skip_spaces(text: &[u8]) -> &[u8] {
    let n = text.iter().take_while(|c| is_c_space(**c)).count();
    &text[n..]
}

fn digits(text: &[u8]) -> usize {
    text.iter().take_while(|c| c.is_ascii_digit()).count()
}

/// `sr_atoi`: `strtol` in base 10, then only white space, and within an `int`.
/// A text with no number and nothing else is 0, as `strtol` leaves the end at
/// its start.
pub fn atoi(text: &[u8]) -> Option<i32> {
    let text = c_string(text);
    let body = skip_spaces(text);
    let (negative, unsigned) = match body.first() {
        Some(b'-') => (true, &body[1..]),
        Some(b'+') => (false, &body[1..]),
        _ => (false, body),
    };
    let n = digits(unsigned);
    if n == 0 {
        return skip_spaces(text).is_empty().then_some(0);
    }
    if !skip_spaces(&unsigned[n..]).is_empty() {
        return None;
    }
    let value: i64 = std::str::from_utf8(&unsigned[..n]).ok()?.parse().ok()?;
    i32::try_from(if negative { -value } else { value }).ok()
}

/// After `0x`, all of `H[.H][pD]`, or `None`; also where a double cannot hold it
/// (`ERANGE`).
fn hex_double(text: &[u8]) -> Option<f64> {
    let mut value = 0f64;
    let mut scale = 0i32;
    let mut seen = false;
    let mut dot = false;
    let mut at = 0;
    while let Some(&c) = text.get(at) {
        if c == b'.' && !dot {
            dot = true;
        } else if let Some(d) = char::from(c).to_digit(16) {
            value = value * 16.0 + f64::from(d);
            seen = true;
            if dot {
                scale = scale.saturating_sub(4);
            }
        } else {
            break;
        }
        at += 1;
    }
    if !seen {
        return None;
    }
    if at < text.len() {
        if !matches!(text[at], b'p' | b'P') {
            return None;
        }
        let rest = &text[at + 1..];
        let (negative, rest) = match rest.first() {
            Some(b'-') => (true, &rest[1..]),
            Some(b'+') => (false, &rest[1..]),
            _ => (false, rest),
        };
        if rest.is_empty() || digits(rest) != rest.len() {
            return None;
        }
        let exponent: i32 = std::str::from_utf8(rest).ok()?.parse().unwrap_or(i32::MAX);
        scale = scale.saturating_add(if negative { -exponent } else { exponent });
    }
    if value == 0.0 {
        return Some(0.0);
    }
    // In two steps, as `powi` alone gives up before the product would.
    let result = value * 2f64.powi(scale / 2) * 2f64.powi(scale - scale / 2);
    result.is_normal().then_some(result)
}

/// `sr_atod_ascii`: `g_ascii_strtod` on all of the text, and neither too large
/// nor too small for a double (`ERANGE`). An empty text is 0, as for [`atoi`].
pub fn atod_ascii(text: &[u8]) -> Option<f64> {
    let text = c_string(text);
    let body = skip_spaces(text);
    if body.is_empty() {
        return text.is_empty().then_some(0.0);
    }
    let (negative, unsigned) = match body[0] {
        b'-' => (true, &body[1..]),
        b'+' => (false, &body[1..]),
        _ => (false, body),
    };
    let signed = |v: f64| if negative { -v } else { v };
    let lower = unsigned.to_ascii_lowercase();
    if lower == b"inf" || lower == b"infinity" {
        return Some(signed(f64::INFINITY));
    }
    if lower == b"nan" {
        return Some(f64::NAN);
    }
    if let Some(inside) = lower.strip_prefix(b"nan(").and_then(|r| r.strip_suffix(b")")) {
        return inside.iter().all(|c| c.is_ascii_alphanumeric() || *c == b'_').then_some(f64::NAN);
    }
    if lower.len() > 2 && lower.starts_with(b"0x") {
        return hex_double(&unsigned[2..]).map(signed);
    }
    let whole = digits(unsigned);
    let mut end = whole;
    let mut seen = whole;
    if unsigned.get(end) == Some(&b'.') {
        let part = digits(&unsigned[end + 1..]);
        end += 1 + part;
        seen += part;
    }
    if seen == 0 {
        return None;
    }
    let mantissa_is_zero = unsigned[..end].iter().all(|c| matches!(c, b'0' | b'.'));
    if matches!(unsigned.get(end), Some(b'e' | b'E')) {
        let rest = &unsigned[end + 1..];
        let sign = usize::from(matches!(rest.first(), Some(b'-' | b'+')));
        let n = digits(&rest[sign..]);
        if n == 0 {
            return None;
        }
        end += 1 + sign + n;
    }
    if end != unsigned.len() {
        return None;
    }
    let value: f64 = std::str::from_utf8(unsigned).ok()?.parse().ok()?;
    if value.is_infinite() || (!mantissa_is_zero && !value.is_normal()) {
        return None;
    }
    Some(signed(value))
}

/// `VSET1 %05.3f`, of what a C float keeps: the value comes through
/// sigrok-cli's `strtof` there.
pub fn voltage_command(volts: f32) -> String {
    format!("VSET1 {:05.3}", f64::from(volts))
}

/// `ISET1 %05.4f`, as [`voltage_command`].
pub fn current_command(amps: f32) -> String {
    format!("ISET1 {:05.4}", f64::from(amps))
}

/// `lps_process_status`'s one use for an LPS-301: the output is on.
pub fn output_of(status: i32) -> bool {
    status & 0x40 != 0
}

/// The wait for the line to be free: over at a line saying `OK`, or when nothing
/// has come for the time it was given, or at [`SETTLE_MAX`].
#[derive(Debug)]
pub struct Settle {
    quiet: Duration,
    last: Instant,
    began: Instant,
    /// The start of a line, as much as tells `OK` from others.
    line: Vec<u8>,
}

impl Settle {
    /// A wait of `quiet` counted from `since`.
    pub fn new(quiet: Duration, since: Instant) -> Settle {
        Settle { quiet, last: since, began: Instant::now(), line: Vec::new() }
    }

    /// A byte from the line, which is dropped: true when it ended a line `OK`.
    pub fn byte(&mut self, byte: u8) -> bool {
        self.last = Instant::now();
        if byte == b'\r' || byte == b'\n' {
            let ok = self.line == b"OK";
            self.line.clear();
            return ok;
        }
        if self.line.len() <= 2 {
            self.line.push(byte);
        }
        false
    }

    /// The line has been silent for long enough, or there is no more waiting.
    pub fn over(&self) -> bool {
        self.last.elapsed() >= self.quiet || self.began.elapsed() >= SETTLE_MAX
    }
}

/// What the driver knows of the device: part of `dev_context`.
pub struct Link {
    /// The port, while open.
    pub port: Option<Box<dyn Port>>,
    /// Its parameters.
    pub params: Params,
    /// As `STATUS` or the last setting has it.
    pub output_enabled: bool,
    /// Volts last read.
    pub voltage: f64,
    /// Amperes last read.
    pub current: f64,
    /// The voltage set, which the device is never asked.
    pub voltage_target: f64,
    /// The current limit set, which the device is never asked.
    pub current_limit: f64,
    /// A capture ended with a request not fully answered, sent then.
    pub open_request: Option<Instant>,
}

impl Link {
    /// A link on `port`, with nothing read yet.
    pub fn new(port: Option<Box<dyn Port>>, params: Params) -> Link {
        Link {
            port,
            params,
            output_enabled: false,
            voltage: 0.0,
            current: 0.0,
            voltage_target: 0.0,
            current_limit: 0.0,
            open_request: None,
        }
    }

    fn port(&mut self) -> Result<&mut Box<dyn Port>> {
        self.port.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `lps_send_va`: the line with its CR LF. A write that times out is no
    /// error, as `serial_write_blocking` says 0 there; what follows it fails.
    pub fn send(&mut self, command: &str) -> Result<()> {
        log::debug!("Sending '{command}'.");
        let line = format!("{command}\r\n");
        let timeout = serial_timeout(&self.params, u32::try_from(line.len()).unwrap_or(u32::MAX));
        self.port()?.write_timeout(line.as_bytes(), timeout)?;
        Ok(())
    }

    /// One byte within `timeout`, or `None`.
    pub fn read_byte(&mut self, timeout: Duration) -> Result<Option<u8>> {
        let mut byte = [0];
        let n = read_blocking(self.port()?.as_mut(), &mut byte, timeout)?;
        Ok((n == 1).then_some(byte[0]))
    }

    /// Discards what has come and was not read.
    pub fn purge(&mut self) -> Result<()> {
        self.port()?.purge()
    }

    /// `serial_readline` with `LINELEN_MAX`'s room: a line without its one CR
    /// or LF, or what fits. Time running out is an error, where C goes on to a
    /// read of no timeout, as this module's documentation says.
    fn read_line(&mut self, deadline: Instant) -> Result<Vec<u8>> {
        let mut line = Vec::new();
        while line.len() < LINELEN_MAX - 1 {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(Error::Timeout);
            }
            let Some(byte) = self.read_byte(left)? else { continue };
            if byte == b'\r' || byte == b'\n' {
                break;
            }
            line.push(byte);
        }
        Ok(line)
    }

    /// `lps_read_reply`: the data line, `None` for an answer of `OK` alone, or
    /// an error, `ERROR` among those.
    fn read_reply(&mut self) -> Result<Option<Vec<u8>>> {
        let deadline = Instant::now() + REQ_TIMEOUT;
        let mut reply = None;
        for _ in 0..LINE_TRIES {
            let line = self.read_line(deadline)?;
            if c_string(&line) == b"OK" {
                return Ok(None);
            }
            if !line.is_empty() {
                reply = Some(line);
                break;
            }
        }
        for _ in 0..LINE_TRIES {
            let line = self.read_line(deadline)?;
            if c_string(&line) == b"OK" {
                if reply.as_deref().is_some_and(|r| c_string(r) == b"ERROR") {
                    return Err(Error::device("the device answered ERROR"));
                }
                return Ok(reply);
            }
        }
        Err(Error::device("no OK after the answer"))
    }

    /// Waits out an answer a capture left on its way: a purge only drops what has
    /// come, and its `OK` would pass for the answer to what goes next.
    fn settle(&mut self, since: Instant) -> Result<()> {
        let mut settle = Settle::new(QUIET, since);
        while !settle.over() {
            if let Some(byte) = self.read_byte(Duration::from_millis(5))?
                && settle.byte(byte)
            {
                break;
            }
        }
        Ok(())
    }

    /// `lps_cmd_ok`: a command whose answer is `OK` alone. What a capture's last
    /// request left is no answer to this: C never gets here, as it refuses every
    /// setting after a capture. A failure is C's word for it, as it has no other;
    /// the debug log says which.
    pub fn command(&mut self, command: &str) -> Result<()> {
        if let Some(since) = self.open_request.take() {
            self.settle(since)?;
        }
        self.purge()?;
        self.send(command)?;
        match self.read_reply() {
            Ok(None) => Ok(()),
            Ok(Some(reply)) => {
                log::debug!(
                    "Data in the answer to {command}: '{}'.",
                    String::from_utf8_lossy(&reply)
                );
                Err(Error::device(GENERIC_ERROR))
            }
            Err(Error::Device(why)) => {
                log::debug!("{command}: {why}.");
                Err(Error::device(GENERIC_ERROR))
            }
            Err(other) => Err(other),
        }
    }

    /// `lps_cmd_reply`: a command and its data line.
    pub fn query(&mut self, command: &str) -> Result<Vec<u8>> {
        self.send(command)?;
        self.read_reply()?
            .ok_or_else(|| Error::device(format!("no data in the answer to {command}")))
    }

    /// `lps_query_status`, so far as an LPS-301 needs it.
    pub fn query_status(&mut self) -> Result<()> {
        let reply = self.query("STATUS").inspect_err(|e| {
            log::debug!("STATUS: {e}.");
            log::error!("lps_query_status: Failed to read status: {GENERIC_ERROR}.");
        })?;
        let status = atoi(&reply).ok_or_else(|| Error::device("a status that is no number"))?;
        self.output_enabled = output_of(status);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// A port whose answers are all there at once, and that keeps what it was told.
    struct Script {
        input: VecDeque<u8>,
        written: std::sync::Arc<std::sync::Mutex<Vec<u8>>>,
    }

    impl Port for Script {
        fn name(&self) -> &'static str {
            "/dev/ttyFAKE"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            let n = self.input.len().min(buf.len());
            if n == 0 {
                std::thread::sleep(timeout.min(Duration::from_millis(5)));
            }
            for b in &mut buf[..n] {
                *b = self.input.pop_front().expect("counted");
            }
            Ok(n)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            self.written.lock().unwrap().extend_from_slice(data);
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            Ok(())
        }
    }

    fn link(input: &[u8]) -> Link {
        let port =
            Script { input: input.iter().copied().collect(), written: std::sync::Arc::default() };
        Link::new(Some(Box::new(port)), Params::parse(SERIALCOMM).unwrap())
    }

    #[test]
    fn serialcomm_parses_and_is_2400() {
        let p = Params::parse(SERIALCOMM).unwrap();
        assert_eq!(p.baud, 2400);
        assert_eq!((p.rts, p.dtr), (Some(true), Some(true)));
    }

    #[test]
    fn up_to_five_reads_get_to_a_line_and_to_its_ok() {
        // Four line ends ahead of each.
        let mut l = link(b"\r\n\r\nLPS-301\r\n\r\nOK\r\n");
        assert_eq!(l.query("MODEL").unwrap(), b"LPS-301");
        // Five empty reads use up the first loop, and then OK with no data.
        let mut l = link(b"\r\n\r\n\rLPS-301\r\nOK\r\n");
        assert!(l.query("MODEL").is_err());
        // Five lines of no OK.
        let mut l = link(b"x\ra\rb\rc\rd\re\rOK\r");
        assert!(l.query("MODEL").is_err());
        // Four is fine, and they are dropped.
        let mut l = link(b"x\ra\rb\rc\rd\rOK\r");
        assert_eq!(l.query("MODEL").unwrap(), b"x");
    }

    #[test]
    fn an_answer_is_ok_alone_or_data_and_error_is_a_failure() {
        assert!(link(b"OK\r\n").command("OUT1").is_ok());
        assert!(link(b"x\r\nOK\r\n").command("OUT1").is_err(), "data");
        assert!(link(b"ERROR\r\nOK\r\n").command("OUT1").is_err());
        assert!(link(b"ERROR\r\nOK\r\n").query("MODEL").is_err());
        assert!(link(b"OK\r\n").query("MODEL").is_err(), "no data");
        assert_eq!(link(b"ERRORS\r\nOK\r\n").query("MODEL").unwrap(), b"ERRORS");
    }

    #[test]
    fn a_silent_device_fails_in_the_reply_time_where_c_blocks_for_ever() {
        for input in [&b""[..], b"LPS-301", b"LPS-301\r\n", b"\r\n"] {
            let began = Instant::now();
            assert!(link(input).query("MODEL").is_err(), "{}", input.escape_ascii());
            let took = began.elapsed();
            assert!(took >= Duration::from_millis(250), "{took:?}");
            assert!(took < Duration::from_millis(1500), "{took:?}");
        }
    }

    #[test]
    fn a_long_line_is_cut_at_49_bytes_and_the_rest_read_as_more_lines() {
        let long = [b'x'; 60];
        let mut input = long.to_vec();
        input.extend_from_slice(b"\r\nOK\r\n");
        assert_eq!(link(&input).query("VERSION").unwrap(), [b'x'; 49]);
    }

    #[test]
    fn a_command_ends_cr_lf() {
        let written = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let port = Script { input: VecDeque::new(), written: std::sync::Arc::clone(&written) };
        let mut l = Link::new(Some(Box::new(port)), Params::parse(SERIALCOMM).unwrap());
        l.send("MODEL").unwrap();
        assert_eq!(*written.lock().unwrap(), b"MODEL\r\n");
    }

    #[test]
    fn commands_format_as_the_c_in_single_precision() {
        assert_eq!(voltage_command(5.0), "VSET1 5.000");
        assert_eq!(voltage_command(32.0), "VSET1 32.000");
        assert_eq!(voltage_command(0.0), "VSET1 0.000");
        assert_eq!(voltage_command(1.2345), "VSET1 1.235", "1.2345f is above, 1.2345 below");
        assert_eq!(voltage_command(0.1235), "VSET1 0.123", "0.1235f is below, 0.1235 above");
        assert_eq!(current_command(0.25), "ISET1 0.2500");
        assert_eq!(current_command(2.0), "ISET1 2.0000");
        assert_eq!(current_command(0.001), "ISET1 0.0010");
    }

    /// Each pair is what `sr_atoi` answers: `strtol`, base 10, then white space.
    #[test]
    fn the_status_reads_as_sr_atoi() {
        for (text, want) in [
            (&b"64"[..], Some(64)),
            (b"+64", Some(64)),
            (b"-1", Some(-1)),
            (b" 64", Some(64)),
            (b"64 ", Some(64)),
            (b"64\t\r", Some(64)),
            (b"\x0b64\x0b", Some(64)),
            (b"007", Some(7)),
            (b"", Some(0)),
            (b"  ", Some(0)),
            (b"2147483647", Some(i32::MAX)),
            (b"-2147483648", Some(i32::MIN)),
            (b"2147483648", None),
            (b"99999999999999999999999", None),
            (b"-", None),
            (b"+", None),
            (b"x", None),
            (b"64x", None),
            (b"0x40", None),
            (b"6 4", None),
            (b"+-1", None),
        ] {
            assert_eq!(atoi(text), want, "{}", text.escape_ascii());
        }
        assert_eq!(atoi(b"64\0x"), Some(64), "a string ends at its NUL");
    }

    #[test]
    fn vt_is_space_to_strtol_and_not_to_glib() {
        assert!(!is_glib_space(0x0b));
        assert!(is_c_space(0x0b));
        for c in [b' ', b'\t', b'\n', 0x0c, b'\r'] {
            assert!(is_glib_space(c) && is_c_space(c), "{c}");
        }
        assert!(!is_glib_space(b'x') && !is_c_space(b'x'));
    }

    #[test]
    fn a_settle_ends_at_ok_or_silence_and_has_a_limit() {
        let quiet = Duration::from_millis(100);
        let mut s = Settle::new(quiet, Instant::now());
        assert!(!s.over());
        for b in *b"5.000\r\n" {
            assert!(!s.byte(b), "{b}");
        }
        assert!(!s.byte(b'O') && !s.byte(b'K'));
        assert!(s.byte(b'\r'));
        for b in *b"OKAY\r" {
            assert!(!s.byte(b), "{b}");
        }
        // Long ago: over at once.
        std::thread::sleep(Duration::from_millis(1));
        let s = Settle::new(Duration::ZERO, Instant::now());
        assert!(s.over());
        // A byte starts the silence again, up to the limit.
        let mut s = Settle::new(Duration::from_secs(60), Instant::now());
        s.byte(b'x');
        assert!(!s.over());
        s.began -= SETTLE_MAX;
        assert!(s.over());
    }

    /// The output's bit, and no other.
    #[test]
    fn only_bit_6_is_the_output() {
        assert!(output_of(64));
        assert!(output_of(-1));
        assert!(!output_of(191));
        assert!(!output_of(0));
        assert!(output_of(576));
    }

    /// Each pair is what `sr_atod_ascii` answered, run on this table with glib.
    #[test]
    fn readings_read_as_sr_atod_ascii() {
        for (text, want) in [
            (&b"12.345"[..], 12.345),
            (b"1", 1.0),
            (b"1.", 1.0),
            (b".5", 0.5),
            (b"+.5", 0.5),
            (b"-2", -2.0),
            (b"1.5e1", 15.0),
            (b"1E2", 100.0),
            (b"1e-1", 0.1),
            (b"0x10", 16.0),
            (b"0x1p-1", 0.5),
            (b"0X.8", 0.5),
            (b"-0x10", -16.0),
            (b"0x0p9999", 0.0),
            (b"0x1p1000", 2f64.powi(1000)),
            (b"0x1000p-1020", 2f64.powi(-1008)),
            (b"\x0b1", 1.0),
            (b" 1", 1.0),
            (b"0e999", 0.0),
            (b"0.0000", 0.0),
            (b"", 0.0),
        ] {
            assert_eq!(atod_ascii(text), Some(want), "{}", text.escape_ascii());
        }
        for text in [&b"inf"[..], b"INF", b"infinity", b"+Infinity"] {
            assert_eq!(atod_ascii(text), Some(f64::INFINITY), "{}", text.escape_ascii());
        }
        assert_eq!(atod_ascii(b"-inf"), Some(f64::NEG_INFINITY));
        for text in [&b"nan"[..], b"NAN", b"-nan", b"nan(abc_1)", b"nan()"] {
            assert!(atod_ascii(text).unwrap().is_nan(), "{}", text.escape_ascii());
        }
        for no in [
            &b"x"[..],
            b"1 ",
            b"1V",
            b"1.5V",
            b"1,5",
            b"1.2.3",
            b"1e",
            b"1e+",
            b"e5",
            b".",
            b"-",
            b"+",
            b"0x",
            b"0xg",
            b"0x1p",
            b"OK",
            b"ERROR",
            b"infi",
            b"inf x",
            b"nan(",
            b"nan(a b)",
            b"1e999",
            b"-1e999",
            b"1e-999",
            b"1e-310",
            b"  ",
            b"0x1p9999",
            b"-0x1p9999",
            b"0x1p-9999",
            b"0x1p-1060",
            b"0xfp99999999999",
        ] {
            assert_eq!(atod_ascii(no), None, "{}", no.escape_ascii());
        }
    }
}
