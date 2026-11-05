// Counterpart in sigrok:
//   libsigrok/src/hardware/gwinstek-gpd/api.c,
//   libsigrok/src/hardware/gwinstek-gpd/protocol.c,
//   libsigrok/src/hardware/gwinstek-gpd/protocol.h,
//   Copyright (C) 2018 Bastian Schmitz <bastian.schmitz@udo.edu>,
//   with commits by Richard Allen, Timo Kokkonen, Uwe Hermann,
//   Andreas Sandberg, Gerhard Sittig, Wolfram Sang, Michał Janiszewski
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the GW Instek GPD-2303S and GPD-3303S.
//!
//! ASCII lines: a command ends in LF, an answer in CR, CR LF (newer firmware)
//! or LF. A setting has no answer. `*IDN?` gives `GW INSTEK,<model>,SN:<n>,V<v>`,
//! `STATUS?` eight digits, of which the sixth is the output, or on old firmware
//! `C C T T B X O X` and two lines more. `ISETn?`, `VSETn?`, `IOUTn?` and
//! `VOUTn?` give a number that `sscanf`'s `%f` reads up to its unit letter.
//!
//! A reply must come whole within 250 ms. Where C keeps the buffer's old bytes
//! after a failed read and parses those, here no reply is no reply.

use crate::core::{Error, Result};
use crate::serial::{Params, Port, read_blocking};
use std::time::{Duration, Instant};

/// Min, max and step, as `api.c`'s tables.
pub type Range = [f64; 3];

/// The channels of every model the driver has.
pub const CHANNELS: usize = 2;
/// `gpd_receive_reply`'s time for a whole line.
const REPLY_TIME: Duration = Duration::from_millis(250);
/// `scan`'s buffer.
pub const SCAN_BUFFER: usize = 100;
/// `gpd_receive_data`'s buffer.
pub const CAPTURE_BUFFER: usize = 50;

/// One line of `api.c`'s `models[]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Model {
    /// The name in the ID, and shown.
    pub name: &'static str,
    /// The voltage target's range.
    pub voltage: Range,
    /// The current limit's range.
    pub current: Range,
}

/// `api.c`'s `models[]`.
pub const MODELS: [Model; 2] = [
    Model { name: "GPD-2303S", voltage: [0.0, 30.0, 0.001], current: [0.0, 3.0, 0.001] },
    Model { name: "GPD-3303S", voltage: [0.0, 32.0, 0.001], current: [0.0, 3.2, 0.001] },
];

/// `scan`'s check of the reply's start.
pub fn is_identification(reply: &[u8]) -> bool {
    c_string(reply).starts_with(b"GW INSTEK")
}

/// A reply as C reads it: a string, up to its first NUL.
pub fn c_string(reply: &[u8]) -> &[u8] {
    let n = reply.iter().position(|c| *c == 0).unwrap_or(reply.len());
    &reply[..n]
}

/// The `(.+)` the regex `GW INSTEK,(.+),SN:(.+),(V.+)` catches first: it is
/// greedy, so it runs up to the last `,SN:` the rest still matches after.
fn model_name(text: &[u8]) -> Option<&[u8]> {
    const HEAD: &[u8] = b"GW INSTEK,";
    const SERIAL: &[u8] = b",SN:";
    let matches_rest = |rest: &[u8]| {
        // (.+),(V.+): one byte at least on each side.
        rest.windows(2).enumerate().any(|(j, w)| j >= 1 && w == b",V" && rest.len() > j + 2)
    };
    let mut from = 0;
    while let Some(at) = text[from..].windows(HEAD.len()).position(|w| w == HEAD) {
        let body = &text[from + at + HEAD.len()..];
        let found = body
            .windows(SERIAL.len())
            .enumerate()
            .rev()
            .find(|(i, w)| *i >= 1 && *w == SERIAL && matches_rest(&body[i + SERIAL.len()..]));
        if let Some((i, _)) = found {
            return Some(&body[..i]);
        }
        from += at + 1;
    }
    None
}

/// The model an `*IDN?` reply names.
pub fn lookup(reply: &[u8]) -> Option<&'static Model> {
    let name = model_name(c_string(reply))?;
    MODELS.iter().find(|m| m.name.as_bytes() == name)
}

/// C's `isspace` in the C locale.
fn is_space(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

fn skip_spaces(text: &[u8]) -> &[u8] {
    let n = text.iter().take_while(|c| is_space(**c)).count();
    &text[n..]
}

fn digits(text: &[u8]) -> usize {
    text.iter().take_while(|c| c.is_ascii_digit()).count()
}

/// After `0x`: the value of `H[.H][pD]`, or `None` as `sscanf` fails.
fn hex_float(text: &[u8]) -> Option<f64> {
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
        let (negative, rest) = match rest.first() {
            Some(b'-') => (true, &rest[1..]),
            Some(b'+') => (false, &rest[1..]),
            _ => (false, rest),
        };
        let n = digits(rest);
        if n == 0 {
            return None;
        }
        let exponent: i32 = std::str::from_utf8(&rest[..n]).ok()?.parse().unwrap_or(i32::MAX);
        scale = scale.saturating_add(if negative { -exponent } else { exponent });
    }
    Some(value * 2f64.powi(scale))
}

/// `sscanf(reply, "%f", ...)`: the longest number the text starts with, unit
/// letters and all behind it ignored. glibc's reading: `1e` is no number,
/// where it could have been 1.
pub fn scan_f32(reply: &[u8]) -> Option<f32> {
    let text = skip_spaces(c_string(reply));
    let (negative, body) = match text.first() {
        Some(b'-') => (true, &text[1..]),
        Some(b'+') => (false, &text[1..]),
        _ => (false, text),
    };
    let signed = |v: f32| if negative { -v } else { v };
    let lower: Vec<u8> = body.iter().take(3).map(u8::to_ascii_lowercase).collect();
    if lower == b"inf" {
        return Some(signed(f32::INFINITY));
    }
    if lower == b"nan" {
        // nan(...) must be closed, and may hold letters, digits and _.
        if body.get(3) == Some(&b'(') {
            let inside = &body[4..];
            let n = inside.iter().take_while(|c| c.is_ascii_alphanumeric() || **c == b'_').count();
            if inside.get(n) != Some(&b')') {
                return None;
            }
        }
        return Some(signed(f32::NAN));
    }
    if body.len() >= 2 && body[0] == b'0' && body[1].eq_ignore_ascii_case(&b'x') {
        return hex_float(&body[2..]).map(|v| signed(v as f32));
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
        let sign = usize::from(matches!(rest.first(), Some(b'-' | b'+')));
        let n = digits(&rest[sign..]);
        if n == 0 {
            return None;
        }
        end += 1 + sign + n;
    }
    let value: f32 = std::str::from_utf8(&body[..end]).ok()?.parse().ok()?;
    Some(signed(value))
}

/// The status's two forms: `(output, old)`, or `None` for neither.
///
/// `%1u%1u...`: eight digits, spaces allowed before each. The old one is
/// `%1u %1u %1u %1u %1u X %1u X`, of which the last `X` does not decide.
pub fn parse_status(reply: &[u8]) -> Option<(bool, bool)> {
    let mut text = c_string(reply);
    let digit = |text: &mut &[u8]| {
        let rest = skip_spaces(text);
        let d = *rest.first()?;
        d.is_ascii_digit().then(|| {
            *text = &rest[1..];
            d - b'0'
        })
    };
    let mut new_form = text;
    let new: Vec<u8> = (0..8).map_while(|_| digit(&mut new_form)).collect();
    if new.len() == 8 {
        return Some((new[5] != 0, false));
    }
    if (0..5).map_while(|_| digit(&mut text)).count() != 5 {
        return None;
    }
    let rest = skip_spaces(text);
    let mut rest = rest.strip_prefix(b"X")?;
    let output = digit(&mut rest)?;
    Some((output != 0, true))
}

/// `VSETn:%05.3lf`, of what a C float keeps: the value comes through
/// sigrok-cli's `strtof` there. `n` counts from 0 here.
pub fn voltage_command(channel: usize, volts: f64) -> String {
    format!("VSET{}:{:05.3}\n", channel + 1, f64::from(volts as f32))
}

/// `ISETn:%05.3lf`, as [`voltage_command`].
pub fn current_command(channel: usize, amps: f64) -> String {
    format!("ISET{}:{:05.3}\n", channel + 1, f64::from(amps as f32))
}

/// What one channel has last said or been told: `per_channel_config`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ChannelState {
    /// Volts last read.
    pub voltage: f32,
    /// Amperes last read.
    pub current: f32,
    /// The voltage target.
    pub voltage_target: f32,
    /// The current limit.
    pub current_limit: f32,
}

/// The port and what the driver knows of the device: `dev_context`.
pub struct Link {
    /// The port, while open.
    pub port: Option<Box<dyn Port>>,
    /// Its parameters.
    pub params: Params,
    /// The model.
    pub model: &'static Model,
    /// As `STATUS?` or the last setting has it.
    pub output_enabled: bool,
    /// Per channel.
    pub channels: [ChannelState; CHANNELS],
}

impl Link {
    /// A link on `port`, with nothing read yet.
    pub fn new(port: Option<Box<dyn Port>>, params: Params, model: &'static Model) -> Link {
        Link {
            port,
            params,
            model,
            output_enabled: false,
            channels: [ChannelState::default(); CHANNELS],
        }
    }

    fn port(&mut self) -> Result<&mut Box<dyn Port>> {
        self.port.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `gpd_send_cmd`.
    pub fn send(&mut self, command: &str) -> Result<()> {
        log::debug!("Sending '{}'.", command.escape_default());
        self.port()?.write(command.as_bytes()).inspect_err(|e| {
            log::error!("Error sending command: {e}.");
        })
    }

    /// One byte within `timeout`, or `None`.
    pub fn read_byte(&mut self, timeout: Duration) -> Result<Option<u8>> {
        let mut byte = [0];
        let n = read_blocking(self.port()?.as_mut(), &mut byte, timeout)?;
        Ok((n == 1).then_some(byte[0]))
    }

    /// `gpd_receive_reply`: a line without its end, line ends before it
    /// skipped, or `None` when it did not come whole in time or within what
    /// the buffer of `size` takes. C's own read runs two bytes over the buffer
    /// there (`protocol.c:62`, `:91`); here it stops at the same byte. `first` is
    /// a byte the caller has read already. C restarts no clock for a line end
    /// it skips, nor checks one: here the 250 ms hold over those too, so an
    /// endless run of them ends.
    pub fn read_line(&mut self, size: usize, first: Option<u8>) -> Result<Option<Vec<u8>>> {
        let started = Instant::now();
        let mut line = Vec::new();
        let mut next = first;
        loop {
            let byte = if let Some(byte) = next.take() {
                byte
            } else {
                let left = REPLY_TIME.saturating_sub(started.elapsed());
                if left.is_zero() {
                    return Ok(None);
                }
                let Some(byte) = self.read_byte(left)? else { return Ok(None) };
                byte
            };
            if byte == b'\r' || byte == b'\n' {
                if line.is_empty() {
                    continue;
                }
                log::debug!("Received line '{}'.", String::from_utf8_lossy(&line));
                return Ok(Some(line));
            }
            line.push(byte);
            if line.len() > size {
                return Ok(None);
            }
        }
    }

    /// A query and its reply, an empty one when none came.
    pub fn ask(&mut self, command: &str, size: usize) -> Result<Vec<u8>> {
        self.send(command)?;
        Ok(self.read_line(size, None)?.unwrap_or_default())
    }

    /// `scan` after the ID: the status, then every channel's values. An answer
    /// that is no number means no device, each as C's message says.
    pub fn read_everything(&mut self) -> Result<bool> {
        self.port()?.purge()?;
        let reply = self.ask("STATUS?\n", SCAN_BUFFER)?;
        let Some((output, old)) = parse_status(&reply) else {
            log::error!("Invalid reply to STATUS: '{}'.", String::from_utf8_lossy(&reply));
            return Ok(false);
        };
        self.output_enabled = output;
        if old {
            // The other two lines of the old form, whatever they bring.
            let _ = self.read_line(SCAN_BUFFER, None)?;
            let _ = self.read_line(SCAN_BUFFER, None)?;
        }
        for ch in 0..CHANNELS {
            let n = ch + 1;
            let ok = self
                .scan_value(&format!("ISET{n}?\n"), "ISETn?", ch, |s, v| s.current_limit = v)?
                && self.scan_value(&format!("VSET{n}?\n"), "VSETn?", ch, |s, v| {
                    s.voltage_target = v;
                })?
                && self.scan_value(&format!("IOUT{n}?\n"), "IOUTn?", ch, |s, v| s.current = v)?
                && self.scan_value(&format!("VOUT{n}?\n"), "VOUTn?", ch, |s, v| s.voltage = v)?;
            if !ok {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn scan_value(
        &mut self,
        command: &str,
        name: &str,
        channel: usize,
        store: fn(&mut ChannelState, f32),
    ) -> Result<bool> {
        let reply = self.ask(command, SCAN_BUFFER)?;
        let Some(value) = scan_f32(&reply) else {
            log::error!("Invalid reply to {name}: '{}'.", String::from_utf8_lossy(&reply));
            return Ok(false);
        };
        store(&mut self.channels[channel], value);
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_id_takes_the_regex_as_far_as_the_c() {
        let name = |t: &[u8]| lookup(t).map(|m| m.name);
        assert_eq!(name(b"GW INSTEK,GPD-2303S,SN:ER915277,V2.10"), Some("GPD-2303S"));
        assert_eq!(name(b"GW INSTEK,GPD-3303S,SN:1,V1"), Some("GPD-3303S"));
        assert_eq!(name(b"GW INSTEK,GPD-3303S,SN:1,V1 extra"), Some("GPD-3303S"));
        assert_eq!(name(b"GW INSTEK,GPD-3303S,SN:a,b,V1"), Some("GPD-3303S"));
        assert_eq!(name(b"GW INSTEK junk GW INSTEK,GPD-3303S,SN:1,V1"), Some("GPD-3303S"));
        for no in [
            &b"GW INSTEK,GPD-4303S,SN:1,V1"[..],
            b"GW INSTEK,GPD-2303S",
            b"GW INSTEK,GPD-3303S,SN:1,V",
            b"GW INSTEK,GPD-3303S,SN:,V1",
            b"GW INSTEK,,SN:1,V1",
            b"GW INSTEK,GPD-3303S,x,SN:1,SN:2,V1",
            b"gw instek,GPD-2303S,SN:1,V1",
            b"",
        ] {
            assert_eq!(name(no), None, "{}", no.escape_ascii());
        }
        assert_eq!(model_name(b"GW INSTEK,A,SN:1,SN:2,V1"), Some(&b"A,SN:1"[..]));
        // The last ,SN: has nothing to match after it, so the one before counts.
        assert_eq!(model_name(b"GW INSTEK,A,SN:1,V1,SN:2"), Some(&b"A"[..]));
    }

    /// Each pair is what glibc's `sscanf("%f")` answered, run on this table.
    #[test]
    fn numbers_read_as_glibcs_sscanf() {
        for (text, want) in [
            (&b"2.000A\n"[..], 2.0),
            (b"12.000V\n", 12.0),
            (b"1", 1.0),
            (b"1.", 1.0),
            (b".5", 0.5),
            (b"+.5", 0.5),
            (b"-2V", -2.0),
            (b"1e5", 100_000.0),
            (b"1E2A", 100.0),
            (b"1e2.5", 100.0),
            (b"1.2.3", 1.2),
            (b"0x10", 16.0),
            (b"0x1p3", 8.0),
            (b"0x.8", 0.5),
            (b"0X1P1", 2.0),
            (b" \t\n 1.5", 1.5),
            (b"\n.000A\n", 0.0),
            (b"1e-999", 0.0),
            (b"1,5", 1.0),
            (b"1 5", 1.0),
            (b"1e5e5", 100_000.0),
            (b"0e0", 0.0),
            (b"00.000V\r\n", 0.0),
        ] {
            assert_eq!(scan_f32(text), Some(want), "{}", text.escape_ascii());
        }
        for text in [&b"-inf"[..], b"infinity", b"infx"] {
            assert!(scan_f32(text).unwrap().is_infinite(), "{}", text.escape_ascii());
        }
        assert_eq!(scan_f32(b"1e999"), Some(f32::INFINITY));
        assert_eq!(
            scan_f32(b"0123456789012345678901234567890123456789012345678901"),
            Some(f32::INFINITY)
        );
        assert!(scan_f32(b"nan").unwrap().is_nan());
        assert!(scan_f32(b"NAN(abc)").unwrap().is_nan());
        for no in [
            &b"."[..],
            b"-",
            b"+",
            b"1e",
            b"1e+",
            b"0x",
            b"0x1p",
            b"0xg",
            b"nan(",
            b"in",
            b"n",
            b"x",
            b"",
            b"\n",
            b"+-1",
            b"e5",
        ] {
            assert_eq!(scan_f32(no), None, "{}", no.escape_ascii());
        }
        assert_eq!(scan_f32(b"1\0.5"), Some(1.0), "a string ends at its NUL");
    }

    /// Run on glibc too: `new` is 8 conversions, `old` 6.
    #[test]
    fn the_status_reads_as_glibcs_sscanf() {
        for (text, want) in [
            (&b"11011010"[..], Some((false, false))),
            (b"11011110\n", Some((true, false))),
            (b"11011210", Some((true, false))),
            (b" 1 1 0 1 1 0 1 0", Some((false, false))),
            (b"110110101234", Some((false, false))),
            (b"1 1 0 1 1 X 0 X", Some((false, true))),
            (b"1 1 0 1 1 X 1 X", Some((true, true))),
            (b"1 1 0 1 1X0", Some((false, true))),
            (b"11011X0", Some((false, true))),
            (b"1 1 0 1 1 X 2 X", Some((true, true))),
            (b"1 1 0 1 1\nX\n1", Some((true, true))),
            (b"1 1 0 1 1 X X", None),
            (b"1 1 0 1 1", None),
            (b"+1011010", None),
            (b"1101", None),
            (b"abc", None),
            (b"", None),
            (b"GW INSTEK,GPD-2303S,SN:1,V1", None),
        ] {
            assert_eq!(parse_status(text), want, "{}", text.escape_ascii());
        }
    }

    #[test]
    fn commands_format_as_the_c_in_single_precision() {
        assert_eq!(voltage_command(0, 5.0), "VSET1:5.000\n");
        assert_eq!(voltage_command(1, 30.0), "VSET2:30.000\n");
        assert_eq!(voltage_command(0, 0.0), "VSET1:0.000\n");
        assert_eq!(voltage_command(0, 1.2345), "VSET1:1.235\n", "1.2345f is above, 1.2345 below");
        assert_eq!(voltage_command(0, 0.1235), "VSET1:0.123\n", "0.1235f is below, 0.1235 above");
        assert_eq!(current_command(1, 3.2), "ISET2:3.200\n");
        assert_eq!(current_command(0, 0.25), "ISET1:0.250\n");
    }
}
