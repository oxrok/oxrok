// Counterpart in sigrok:
//   libsigrok/src/hardware/gwinstek-psp/api.c,
//   libsigrok/src/hardware/gwinstek-psp/protocol.c,
//   libsigrok/src/hardware/gwinstek-psp/protocol.h,
//   Copyright (C) 2023 ettom <36895504+ettom@users.noreply.github.com>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the GW Instek PSP-603, PSP-405 and PSP-2010.
//!
//! ASCII at 2400/8n1. A command ends in CR LF here, as the C driver sends it,
//! though the manual asks only for CR. `SU n`, `SV xx.xx`, `SI x.xx`, `KOE` and
//! `KOD` have no answer, and the device needs 50 ms after each command.
//!
//! `L` answers 37 characters, `Vvv.vvAa.aaaWwww.wUuuIi.iiPpppFffffff`, then CR
//! LF or, in the mode the C driver calls URPSP1, CR CR LF. The numbers are read
//! as `sscanf` reads them, so the widths are not checked, and the flags by their
//! place in the frame: byte 31 the output, byte 32 over temperature. While the
//! panel sets a limit its letter is in lower case, and `u` or `i` fails the read.
//! The answer takes about 245 ms on a real unit, and within that time of the
//! last `L` the driver answers what it has, so with a sample every 50 ms after a
//! read each sample there is new. A device that answers at once, as the model
//! does, gives one new sample in about five.
//!
//! A stop can come with an answer on its way. It is not left for the next read
//! to take for its own: see [`Link::poll`].

use crate::core::{Error, Result};
use crate::serial::{Params, Port, read_blocking};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// Min, max and step, as `api.c`'s tables.
pub type Range = [f64; 3];

/// `GWINSTEK_PSP_PROCESSING_TIME_MS`: the least time between two commands.
pub const PROCESSING_TIME: Duration = Duration::from_millis(50);
/// `GWINSTEK_PSP_STATUS_POLL_TIME_MS`: within this of the last `L`, no other.
pub const STATUS_POLL_TIME: Duration = Duration::from_millis(245);
/// `RESPONSE_ALL_QUERY_LEN`.
pub const RESPONSE_LEN: usize = 37;
/// How long a whole answer may take, and how long a silence ends a drain.
const READ_TIME: Duration = Duration::from_millis(1000);
/// `gwinstek_psp_check_terminator`'s wait for the line end's last byte.
const TERMINATOR_WAIT: Duration = Duration::from_millis(20);
/// How long a drain goes on with a device that keeps talking, where C waits for
/// ever. No device shows what is right; it only has to outlast a whole answer.
pub const DRAIN_BOUND: Duration = Duration::from_secs(3);
/// How often a read looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(50);
/// More than a line end can leave. What is beyond it says nothing new.
const MOST_LEFT: usize = 4096;

/// One line of `api.c`'s `models[]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Model {
    /// The name `force_detect` gives, and shown.
    pub name: &'static str,
    /// The voltage target's range.
    pub voltage: Range,
    /// The current limit's range.
    pub current: Range,
}

/// `api.c`'s `models[]`.
pub const MODELS: [Model; 3] = [
    Model { name: "PSP-603", voltage: [0.0, 60.0, 0.01], current: [0.0, 3.5, 0.01] },
    Model { name: "PSP-405", voltage: [0.0, 40.0, 0.01], current: [0.0, 5.0, 0.01] },
    Model { name: "PSP-2010", voltage: [0.0, 20.0, 0.01], current: [0.0, 10.0, 0.01] },
];

/// `model_lookup`: the model whose name the text starts with, in any case.
pub fn model_lookup(id: &str) -> Option<&'static Model> {
    let found = MODELS.iter().find(|m| {
        id.as_bytes().get(..m.name.len()).is_some_and(|h| h.eq_ignore_ascii_case(m.name.as_bytes()))
    });
    if found.is_none() && !id.is_empty() {
        log::debug!("Could not find a matching model for: [{id}].");
    }
    found
}

/// C's `isspace` in the C locale.
fn is_space(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

fn digits(text: &[u8]) -> usize {
    text.iter().take_while(|c| c.is_ascii_digit()).count()
}

/// After `0x`: the value of `H[.H][pD]` and how much of it there is.
fn hex_float(text: &[u8]) -> Option<(f64, usize)> {
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
    if matches!(text.get(at), Some(b'p' | b'P')) {
        let rest = &text[at + 1..];
        let (negative, sign) = match rest.first() {
            Some(b'-') => (true, 1),
            Some(b'+') => (false, 1),
            _ => (false, 0),
        };
        let n = digits(&rest[sign..]);
        if n == 0 {
            return None;
        }
        let exponent: i32 =
            std::str::from_utf8(&rest[sign..sign + n]).ok()?.parse().unwrap_or(i32::MAX);
        scale = scale.saturating_add(if negative { -exponent } else { exponent });
        at += 1 + sign + n;
    }
    // Zero whatever the exponent, where a huge one is 0 x inf, a NaN.
    let value = if value == 0.0 { 0.0 } else { value * 2f64.powi(scale) };
    Some((value, at))
}

/// `sscanf`'s `%f`, as glibc reads it: the value and the bytes used, or `None`
/// where the conversion fails. `1e` is a failure, where it could have been 1, and
/// a hexadecimal number takes the `A` after it for a digit and a `P` for its
/// exponent.
fn scan_float(text: &[u8]) -> Option<(f32, usize)> {
    let spaces = text.iter().take_while(|c| is_space(**c)).count();
    let signed = &text[spaces..];
    let (negative, sign) = match signed.first() {
        Some(b'-') => (true, 1),
        Some(b'+') => (false, 1),
        _ => (false, 0),
    };
    let body = &signed[sign..];
    let done = |value: f32, used: usize| {
        Some((if negative { -value } else { value }, spaces + sign + used))
    };
    let lower: Vec<u8> = body.iter().take(3).map(u8::to_ascii_lowercase).collect();
    if lower == b"inf" {
        // `inf`, or all of `infinity`: glibc refuses what stops in between.
        let rest: Vec<u8> = body[3..].iter().take(5).map(u8::to_ascii_lowercase).collect();
        if rest.first() != Some(&b'i') {
            return done(f32::INFINITY, 3);
        }
        if rest != b"inity" {
            return None;
        }
        return done(f32::INFINITY, 8);
    }
    if lower == b"nan" {
        // nan(...) must be closed, and may hold letters, digits and _.
        if body.get(3) != Some(&b'(') {
            return done(f32::NAN, 3);
        }
        let inside = &body[4..];
        let n = inside.iter().take_while(|c| c.is_ascii_alphanumeric() || **c == b'_').count();
        if inside.get(n) != Some(&b')') {
            return None;
        }
        return done(f32::NAN, 5 + n);
    }
    if body.len() >= 2 && body[0] == b'0' && body[1].eq_ignore_ascii_case(&b'x') {
        let (value, used) = hex_float(&body[2..])?;
        return done(value as f32, 2 + used);
    }
    let whole = digits(body);
    let mut end = whole;
    let mut seen = whole;
    if body.get(end) == Some(&b'.') {
        let part = digits(&body[end + 1..]);
        end += 1 + part;
        seen += part;
    }
    if seen == 0 {
        return None;
    }
    if matches!(body.get(end), Some(b'e' | b'E')) {
        let rest = &body[end + 1..];
        let exp_sign = usize::from(matches!(rest.first(), Some(b'-' | b'+')));
        let n = digits(&rest[exp_sign..]);
        if n == 0 {
            return None;
        }
        end += 1 + exp_sign + n;
    }
    let value: f32 = std::str::from_utf8(&body[..end]).ok()?.parse().ok()?;
    done(value, end)
}

/// `sscanf`'s `%d`: glibc reads a long and stores its low bits, so a number too
/// wide comes out as whatever they are.
fn scan_int(text: &[u8]) -> Option<(i32, usize)> {
    let spaces = text.iter().take_while(|c| is_space(**c)).count();
    let signed = &text[spaces..];
    let sign = usize::from(matches!(signed.first(), Some(b'-' | b'+')));
    let n = digits(&signed[sign..]);
    if n == 0 {
        return None;
    }
    let shown = std::str::from_utf8(&signed[..sign + n]).ok()?;
    let long = shown.parse::<i64>().unwrap_or(if signed[0] == b'-' { i64::MIN } else { i64::MAX });
    Some((long as i32, spaces + sign + n))
}

/// What one `L` says.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Status {
    /// Volts, whether or not the output is on.
    pub voltage: f32,
    /// Amperes.
    pub current: f32,
    /// Watts.
    pub power: f32,
    /// The voltage limit, in whole volts.
    pub voltage_limit: i32,
    /// The current limit in amperes.
    pub current_limit: f32,
    /// The relay.
    pub output_enabled: bool,
    /// Over temperature.
    pub otp_active: bool,
}

/// `sscanf(reply, "V%fA%fW%fU%dI%f", ...)` and the two flags, on an answer of
/// the whole length. All or nothing, where C has stored the fields ahead of the
/// one that failed.
pub fn parse_status(reply: &[u8]) -> Option<Status> {
    let end = reply.iter().position(|c| *c == 0).unwrap_or(reply.len());
    let mut text = &reply[..end];
    let float = |text: &mut &[u8], letter: u8| -> Option<f32> {
        let rest = text.strip_prefix(&[letter])?;
        let (value, used) = scan_float(rest)?;
        *text = &rest[used..];
        Some(value)
    };
    let voltage = float(&mut text, b'V')?;
    let current = float(&mut text, b'A')?;
    let power = float(&mut text, b'W')?;
    text = text.strip_prefix(b"U")?;
    let (voltage_limit, used) = scan_int(text)?;
    text = &text[used..];
    let current_limit = float(&mut text, b'I')?;
    Some(Status {
        voltage,
        current,
        power,
        voltage_limit,
        current_limit,
        output_enabled: reply.get(31) == Some(&b'1'),
        otp_active: reply.get(32) == Some(&b'1'),
    })
}

/// `SV %05.2f`, of what a C float keeps: the value comes through sigrok-cli's
/// `strtof` there.
pub fn voltage_command(volts: f64) -> String {
    format!("SV {:05.2}\r\n", f64::from(volts as f32))
}

/// `SI %04.2f`, as [`voltage_command`].
pub fn current_command(amps: f64) -> String {
    format!("SI {:04.2}\r\n", f64::from(amps as f32))
}

/// `SU %d` of the next whole volt, for a target in the model's range.
pub fn limit_for(volts: f64) -> i32 {
    f64::from(volts as f32).ceil() as i32
}

/// What a poll came to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Poll {
    /// Under 245 ms since the last `L`: nothing was asked.
    Cached,
    /// An answer was read.
    Fresh,
    /// A whole answer that does not read: the device is there, as when its panel
    /// is setting a limit and its letter is in lower case.
    Unreadable,
    /// Nothing, or only part of an answer.
    Silent,
    /// A stop came during the wait.
    Stopped,
}

/// The port and what the driver knows of the device: `dev_context`.
pub struct Link {
    /// The port, while open.
    pub port: Option<Box<dyn Port>>,
    /// Its parameters.
    pub params: Params,
    /// The model `force_detect` named.
    pub model: &'static Model,
    /// The last `L` that read whole. `output_enabled` here is what the device
    /// said, which the capture's meta compares.
    pub status: Status,
    /// The output as a `--get` answers it: what the device said, or was told.
    pub output_enabled: bool,
    /// The last reading, 0 with the output off.
    pub voltage_or_0: f32,
    /// The device has no way to say this: its voltage with the output off.
    pub voltage_target: f32,
    /// When the device last said a voltage target.
    pub voltage_target_updated: Option<Instant>,
    /// The last target set or reported as a meta.
    pub set_voltage_target: f32,
    /// When that was.
    pub set_voltage_target_updated: Option<Instant>,
    /// 2 or 3: CR LF or CR CR LF.
    terminator_len: usize,
    next_request: Option<Instant>,
    last_query: Option<Instant>,
    /// How much of the last `L`'s answer a stop left unread, and when that `L` went.
    unread: Option<(usize, Instant)>,
}

impl Link {
    /// A link on `port`, with nothing read yet.
    pub fn new(port: Option<Box<dyn Port>>, params: Params, model: &'static Model) -> Link {
        Link {
            port,
            params,
            model,
            status: Status::default(),
            output_enabled: false,
            voltage_or_0: 0.0,
            voltage_target: 0.0,
            voltage_target_updated: None,
            set_voltage_target: 0.0,
            set_voltage_target_updated: None,
            terminator_len: 2,
            next_request: None,
            last_query: None,
            unread: None,
        }
    }

    fn port(&mut self) -> Result<&mut Box<dyn Port>> {
        self.port.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// A capture starts with no wait, as `dev_acquisition_start` has it.
    pub fn forget_next_request(&mut self) {
        self.next_request = None;
    }

    /// `gwinstek_psp_send_cmd`: waits out the time the last command needs.
    pub fn send(&mut self, command: &str) -> Result<()> {
        if let Some(due) = self.next_request {
            std::thread::sleep(due.saturating_duration_since(Instant::now()));
        }
        log::debug!("Sending '{}'.", command.escape_default());
        let sent = self.port()?.write(command.as_bytes()).inspect_err(|e| {
            log::error!("Error sending command: {e}.");
        });
        self.next_request = Some(Instant::now() + PROCESSING_TIME);
        sent
    }

    /// `serial_read_blocking`, in pieces short enough to see a stop.
    fn read(&mut self, buf: &mut [u8], time: Duration, stop: &AtomicBool) -> Result<usize> {
        let deadline = Instant::now() + time;
        let port = self.port()?;
        let mut got = 0;
        while got < buf.len() && !stop.load(Ordering::SeqCst) {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            got += read_blocking(port.as_mut(), &mut buf[got..], left.min(STOP_CHECK))?;
        }
        Ok(got)
    }

    /// After a failed read: until a second's silence, which also finds the start
    /// of the next answer. C has no bound; a stop or [`DRAIN_BOUND`] ends it here.
    fn drain(&mut self, stop: &AtomicBool) -> Result<()> {
        let started = Instant::now();
        let mut byte = [0u8];
        while self.read(&mut byte, READ_TIME, stop)? > 0 {
            if started.elapsed() >= DRAIN_BOUND {
                log::warn!("The device keeps talking after {DRAIN_BOUND:?}.");
                break;
            }
        }
        Ok(())
    }

    /// Waits out what a stop left of an answer, up to the time that answer had from
    /// its `L`, and drops it: else each later read takes the one before its own.
    /// False if a stop came again, which leaves what is still to come noted.
    fn settle_unread(&mut self, stop: &AtomicBool) -> Result<bool> {
        let Some((left, asked)) = self.unread else { return Ok(true) };
        let time = (asked + READ_TIME).saturating_duration_since(Instant::now());
        let mut rest = [0u8; 50];
        let got = self.read(&mut rest[..left], time, stop)?;
        if got < left && stop.load(Ordering::SeqCst) {
            self.unread = Some((left - got, asked));
            return Ok(false);
        }
        self.unread = None;
        self.port()?.purge()?;
        Ok(true)
    }

    /// `gwinstek_psp_get_all_values`, ahead of it what an earlier stop left.
    ///
    /// # Errors
    ///
    /// What the port gives, where a write fails or a read does.
    pub fn poll(&mut self, stop: &AtomicBool) -> Result<Poll> {
        if !self.settle_unread(stop)? {
            return Ok(Poll::Stopped);
        }
        let now = Instant::now();
        if let Some(delta) = self.last_query.map(|last| now.duration_since(last))
            && delta <= STATUS_POLL_TIME
        {
            log::trace!("Last status query was only {delta:?} ago, returning");
            return Ok(Poll::Cached);
        }
        let sent = self.send("L\r\n");
        self.last_query = Some(now);
        sent?;
        let wanted = RESPONSE_LEN + self.terminator_len;
        let mut reply = [0u8; 50];
        let got = self.read(&mut reply[..wanted], READ_TIME, stop).inspect_err(|e| {
            log::error!("Error {e} reading from device.");
        })?;
        if got < wanted && stop.load(Ordering::SeqCst) {
            self.unread = Some((wanted - got, now));
            return Ok(Poll::Stopped);
        }
        log::debug!("Received: '{}'", reply[..got].escape_ascii());
        let status = (got == wanted).then(|| parse_status(&reply[..got])).flatten();
        let Some(status) = status else {
            log::error!("Parsing status payload failed");
            self.drain(stop)?;
            if stop.load(Ordering::SeqCst) {
                self.unread = Some((0, now));
                return Ok(Poll::Stopped);
            }
            return Ok(if got == wanted { Poll::Unreadable } else { Poll::Silent });
        };
        self.status = status;
        self.output_enabled = status.output_enabled;
        if status.output_enabled {
            self.voltage_or_0 = status.voltage;
        } else {
            self.voltage_target = status.voltage;
            self.voltage_target_updated = Some(Instant::now());
            self.voltage_or_0 = 0.0;
        }
        log::trace!(
            "Status: voltage_or_0={:.2}, voltage_target={:.2}, current={:.3}, power={:.1}, \
             voltage_limit={}, current_limit={:.2}",
            self.voltage_or_0,
            self.voltage_target,
            status.current,
            status.power,
            status.voltage_limit,
            status.current_limit,
        );
        Ok(Poll::Fresh)
    }

    /// `gwinstek_psp_check_terminator`: whether the line end is CR LF or CR CR LF,
    /// from what is left after a read made for the short one.
    ///
    /// # Errors
    ///
    /// More than one byte left, or what the port gives.
    pub fn check_terminator(&mut self) -> Result<()> {
        std::thread::sleep(TERMINATOR_WAIT);
        let port = self.port()?;
        let mut left = 0;
        let mut buf = [0u8; 64];
        // What `serial_has_receive_data` counts, read away: `Port` has no count.
        while left < MOST_LEFT {
            let n = port.read(&mut buf, Duration::from_millis(1))?;
            if n == 0 {
                break;
            }
            left += n;
        }
        log::debug!("{left} bytes left in buffer");
        match left {
            0 => log::debug!("Device is in URPSP2 mode, terminator is CRLF"),
            1 => {
                self.terminator_len = 3;
                log::debug!("Device is in URPSP1 mode, terminator is CRCRLF");
            }
            _ => {
                log::error!("Don't know how to deal with {left} bytes left");
                return Err(Error::device("too much after the status"));
            }
        }
        self.port()?.purge()
    }

    /// `gwinstek_psp_get_initial_voltage_target`: the voltage is the target if the
    /// output is off or the supply is in CV, which it takes to be so when the
    /// current is not at its limit. Single precision, as C subtracts floats and
    /// compares the result with a double's 0.01.
    pub fn take_initial_voltage_target(&mut self) {
        let s = self.status;
        if !s.output_enabled || f64::from((s.current - s.current_limit).abs()) >= 0.01 {
            self.voltage_target = s.voltage;
            log::debug!("Set initial voltage target to {:.2}", self.voltage_target);
        } else {
            log::warn!("Could not determine actual voltage target, falling back to 0");
            self.voltage_target = 0.0;
        }
    }

    /// The voltage target's commands: `SU` if the limit differs, then `SV`. The
    /// limit is kept as sent, where C keeps the last `L`'s, so a second target
    /// straight after the first compares with what the device now has.
    ///
    /// # Errors
    ///
    /// What the port gives.
    pub fn set_voltage(&mut self, volts: f64) -> Result<()> {
        let limit = limit_for(volts);
        if self.status.voltage_limit == limit {
            log::debug!("Correct limit ({limit}V) already set");
        } else {
            self.send(&format!("SU {limit}\r\n"))?;
            self.status.voltage_limit = limit;
        }
        self.send(&voltage_command(volts))?;
        self.set_voltage_target = volts as f32;
        self.set_voltage_target_updated = Some(Instant::now());
        // What a get answers until an `L` with the output off says otherwise: with
        // it on, no `L` tells the target, so this stands. The capture's comparison
        // waits a second past this, so it does not see the difference.
        self.voltage_target = volts as f32;
        Ok(())
    }

    /// A new current limit, kept for a get until the next `L`.
    ///
    /// # Errors
    ///
    /// What the port gives.
    pub fn set_current(&mut self, amps: f64) -> Result<()> {
        self.send(&current_command(amps))?;
        self.status.current_limit = amps as f32;
        Ok(())
    }

    /// `KOE` or `KOD`, sent even if it says what the device already said, kept
    /// for a get. The meta of a capture still waits for the device to say it.
    ///
    /// # Errors
    ///
    /// What the port gives.
    pub fn set_output(&mut self, on: bool) -> Result<()> {
        self.send(if on { "KOE\r\n" } else { "KOD\r\n" })?;
        self.output_enabled = on;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(body: &[u8]) -> Vec<u8> {
        let mut reply = body.to_vec();
        reply.extend_from_slice(b"\r\n");
        reply
    }

    #[test]
    fn the_manuals_example_reads_as_it_says() {
        let s = parse_status(&frame(b"V20.00A2.500W050.0U40I5.00P200F101000")).unwrap();
        assert_eq!(s.voltage, 20.0);
        assert_eq!(s.current, 2.5);
        assert_eq!(s.power, 50.0);
        assert_eq!(s.voltage_limit, 40);
        assert_eq!(s.current_limit, 5.0);
        assert!(s.output_enabled);
        assert!(!s.otp_active);
    }

    #[test]
    fn the_flags_are_read_by_their_place() {
        for (flags, output, otp) in [
            (b"000000", false, false),
            (b"100000", true, false),
            (b"010000", false, true),
            (b"110000", true, true),
            (b"201111", false, false),
        ] {
            let mut body = b"V20.00A2.500W050.0U40I5.00P200F".to_vec();
            body.extend_from_slice(flags);
            let s = parse_status(&frame(&body)).unwrap();
            assert_eq!((s.output_enabled, s.otp_active), (output, otp), "{flags:?}");
        }
    }

    #[test]
    fn a_limit_the_panel_is_setting_fails_or_not_as_the_format_says() {
        assert!(parse_status(&frame(b"V20.00A2.500W050.0u40I5.00P200F101000")).is_none());
        assert!(parse_status(&frame(b"V20.00A2.500W050.0U40i5.00P200F101000")).is_none());
        assert!(parse_status(&frame(b"V20.00A2.500W050.0U40I5.00p200F101000")).is_some());
    }

    #[test]
    fn numbers_read_as_sscanf_reads_them() {
        let s = parse_status(&frame(b"V1.5e1A.5W-1.0U+7I 2 P200F00010000000")).unwrap();
        assert_eq!(s.voltage, 15.0);
        assert_eq!(s.current, 0.5);
        assert_eq!(s.power, -1.0);
        assert_eq!(s.voltage_limit, 7);
        assert_eq!(s.current_limit, 2.0);
        assert!(parse_status(&frame(b"VnanA0W0U1I1")).unwrap().voltage.is_nan());
        assert_eq!(parse_status(&frame(b"V-infA0W0U1I1")).unwrap().voltage, f32::NEG_INFINITY);
        assert_eq!(parse_status(&frame(b"VinfinityA0W0U1I1")).unwrap().voltage, f32::INFINITY);
        assert!(parse_status(&frame(b"VinfinA0W0U1I1")).is_none());
        assert!(parse_status(&frame(b"Vnan(x_1)A0W0U1I1")).unwrap().voltage.is_nan());
        assert!(parse_status(&frame(b"Vnan(xA0W0U1I1")).is_none());
        assert!(parse_status(&frame(b"V1eA0W0U1I1")).is_none());
        assert!(parse_status(&frame(b"V1e+A0W0U1I1")).is_none());
        assert!(parse_status(&frame(b"V.A0W0U1I1")).is_none());
        assert!(parse_status(&frame(b"xV1A0W0U1I1")).is_none());
        assert!(parse_status(&frame(b"V1A0W0U+I1")).is_none());
        assert!(parse_status(&frame(b"V1 A0W0U1I1")).is_none(), "no space before a literal");
    }

    #[test]
    fn hexadecimal_takes_what_follows_it_for_itself() {
        let s = parse_status(&frame(b"V12.00A0W0U60I0x10P200F000000")).unwrap();
        assert_eq!(s.current_limit, f32::INFINITY);
        let s = parse_status(&frame(b"V12.00A0W0U60I0x10.8Z")).unwrap();
        assert_eq!(s.current_limit, 16.5);
        let s = parse_status(&frame(b"V12.00A0W0U60I0x1p-1Z")).unwrap();
        assert_eq!(s.current_limit, 0.5);
        assert!(parse_status(&frame(b"V0x10A0.000W000.0U60I1.00")).is_none());
        assert!(parse_status(&frame(b"V0xZA0W0U60I1")).is_none());
        // Zero, as glibc has it, whatever the exponent.
        assert_eq!(parse_status(&frame(b"V0x0p9999Z")), None);
        let s = parse_status(&frame(b"V1A0W0U1I0x0p9999Z")).unwrap();
        assert_eq!(s.current_limit, 0.0);
        let s = parse_status(&frame(b"V1A0W0U1I0x0.0p-9999Z")).unwrap();
        assert_eq!(s.current_limit, 0.0);
        assert!(parse_status(&frame(b"V0x1pA0W0U60I1")).is_none());
    }

    #[test]
    fn a_limit_too_wide_is_its_low_bits() {
        let s = parse_status(&frame(b"V12.00A0W0U99999999999I1.00")).unwrap();
        assert_eq!(s.voltage_limit, 99_999_999_999i64 as i32);
        assert_eq!(s.voltage_limit, 1_215_752_191);
        let s = parse_status(&frame(b"V12.00A0W0U999999999999999999999I1.00")).unwrap();
        assert_eq!(s.voltage_limit, -1, "LONG_MAX's low bits");
        let s = parse_status(&frame(b"V12.00A0W0U-999999999999999999999I1.00")).unwrap();
        assert_eq!(s.voltage_limit, 0, "LONG_MIN's low bits");
    }

    #[test]
    fn a_nul_ends_the_text_and_not_the_flags() {
        let mut body = b"V12.00A0.000W000.0U60I1.00".to_vec();
        body.extend_from_slice(&[0; 4]);
        body.extend_from_slice(b"F100100");
        let s = parse_status(&frame(&body)).unwrap();
        assert!(s.output_enabled);
        assert!(parse_status(b"V12.00A0.000W000.0U60\0I1.00").is_none());
    }

    #[test]
    fn a_short_reply_has_no_flags() {
        let s = parse_status(b"V1A2W3U4I5").unwrap();
        assert!(!s.output_enabled);
        assert!(!s.otp_active);
    }

    #[test]
    fn models_are_found_by_their_start_in_any_case() {
        assert_eq!(model_lookup("PSP-603").unwrap().name, "PSP-603");
        assert_eq!(model_lookup("psp-405").unwrap().name, "PSP-405");
        assert_eq!(model_lookup("Psp-2010xyz").unwrap().name, "PSP-2010");
        for none in ["", "PSP", "PSP-60", "PSP-999", "é-603"] {
            assert_eq!(model_lookup(none), None, "{none}");
        }
    }

    #[test]
    fn the_tables_say_what_api_c_says() {
        assert_eq!(MODELS[0].voltage, [0.0, 60.0, 0.01]);
        assert_eq!(MODELS[0].current, [0.0, 3.5, 0.01]);
        assert_eq!(MODELS[1].voltage, [0.0, 40.0, 0.01]);
        assert_eq!(MODELS[1].current, [0.0, 5.0, 0.01]);
        assert_eq!(MODELS[2].voltage, [0.0, 20.0, 0.01]);
        assert_eq!(MODELS[2].current, [0.0, 10.0, 0.01]);
    }

    #[test]
    fn commands_have_cs_widths_and_rounding() {
        assert_eq!(voltage_command(5.0), "SV 05.00\r\n");
        assert_eq!(voltage_command(60.0), "SV 60.00\r\n");
        assert_eq!(voltage_command(0.0), "SV 00.00\r\n");
        assert_eq!(voltage_command(3.3), "SV 03.30\r\n");
        assert_eq!(current_command(1.5), "SI 1.50\r\n");
        assert_eq!(current_command(10.0), "SI 10.00\r\n");
        assert_eq!(current_command(0.0), "SI 0.00\r\n");
        // What glibc gives on these ties, from the goldens.
        assert_eq!(voltage_command(0.125), "SV 00.12\r\n");
        assert_eq!(voltage_command(0.375), "SV 00.38\r\n");
        assert_eq!(current_command(0.125), "SI 0.12\r\n");
    }

    #[test]
    fn the_limit_is_the_next_whole_volt_in_single_precision() {
        assert_eq!(limit_for(5.0), 5);
        assert_eq!(limit_for(5.1), 6);
        assert_eq!(limit_for(3.3), 4);
        assert_eq!(limit_for(0.0), 0);
        assert_eq!(limit_for(0.001), 1);
        assert_eq!(limit_for(60.0), 60);
        // 4.00000001 is 4 in a float, and a double's ceil would say 5.
        assert_eq!(limit_for(4.000_000_01), 4);
    }

    #[test]
    fn the_timings_are_the_headers() {
        assert_eq!(PROCESSING_TIME, Duration::from_millis(50));
        assert_eq!(STATUS_POLL_TIME, Duration::from_millis(245));
        assert_eq!(RESPONSE_LEN, 37);
        assert_eq!(READ_TIME, Duration::from_millis(1000));
        assert_eq!(TERMINATOR_WAIT, Duration::from_millis(20));
        assert_eq!(DRAIN_BOUND, Duration::from_secs(3));
    }
}
