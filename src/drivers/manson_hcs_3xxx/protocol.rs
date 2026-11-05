// Counterpart in sigrok:
//   libsigrok/src/hardware/manson-hcs-3xxx/api.c,
//   libsigrok/src/hardware/manson-hcs-3xxx/protocol.c,
//   libsigrok/src/hardware/manson-hcs-3xxx/protocol.h,
//   Copyright (C) 2014 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2014 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Aurelien Jacobs,
//   Gerhard Sittig, Martin Ling, Michael Klengel, Soeren Apel, turboaffe,
//   Daniel Elstner, Jan Luebbe, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the Manson HCS-3xxx family, and its table of models.
//!
//! ASCII at 9600/8n1 behind a CP2102. A command ends in `\r`, and so does each
//! line of a reply, whose last line is `OK`:
//!
//! - `GMOD`: the model code, as `3202` or `HCS-3202`.
//! - `GETD`: `VVVVCCCCM`, hundredths of a volt, hundredths of an ampere, and
//!   0 for constant voltage or 1 for constant current.
//! - `GMAX`: `VVVCCC`, the limits, in the units of `VOLT` and `CURR`.
//! - `VOLTnnn`, in tenths of a volt; `CURRnnn`, in tenths or hundredths of an
//!   ampere by model; `SOUT0` is on and `SOUT1` off. Each answers `OK`.
//!
//! All of it is as in `protocol.c`, but for a reply that does not come, which
//! C waits for for ever, and fields that make no number, which C takes for 0.

use crate::core::{Error, Result};
use crate::serial::{Port, read_blocking};
use std::time::{Duration, Instant};

/// The last line of every reply.
pub const OK: &[u8] = b"OK\r";
/// What `hcs_read_reply` takes at most: its loop runs to the buffer's size and
/// one more.
pub const REPLY_MAX: usize = 51;
/// How long a whole reply to a blocking request may take. C has no bound; this
/// is four times what it allows a reading in a capture, and no source gives a
/// real one's latency.
pub const REPLY_TIMEOUT: Duration = Duration::from_secs(2);
/// A `GETD` of a capture may go unanswered this long, then it is asked again.
pub const REQ_TIMEOUT: Duration = Duration::from_millis(500);
/// What a capture collects of a reply: `dev_context`'s `buf`, whose last byte
/// is for the NUL.
pub const CAPTURE_MAX: usize = 49;
/// The lowest voltage target, the same on every model.
pub const VOLTAGE_MIN: f64 = 1.0;
/// The voltage's step, the same on every model.
pub const VOLTAGE_STEP: f64 = 0.1;
/// The lowest current limit, the same on every model.
pub const CURRENT_MIN: f64 = 0.0;

/// One model. `api.c`'s table has each twice, with and without `HCS-`, and
/// maxima that nothing reads: `GMAX` has the say.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Model {
    /// The name shown.
    pub name: &'static str,
    /// The model code.
    pub code: &'static str,
    /// The current limit's step, and so `CURR`'s unit.
    pub current_step: f64,
}

const fn model(name: &'static str, code: &'static str, current_step: f64) -> Model {
    Model { name, code, current_step }
}

/// The models of `api.c`.
pub const MODELS: [Model; 16] = [
    model("HCS-3100", "3100", 0.10),
    model("HCS-3102", "3102", 0.01),
    model("HCS-3104", "3104", 0.01),
    model("HCS-3150", "3150", 0.10),
    model("HCS-3200", "3200", 0.10),
    model("HCS-3202", "3202", 0.10),
    model("HCS-3204", "3204", 0.01),
    model("HCS-3300-USB", "3300", 0.10),
    model("HCS-3302-USB", "3302", 0.10),
    model("HCS-3304-USB", "3304", 0.10),
    model("HCS-3400-USB", "3400", 0.10),
    model("HCS-3402-USB", "3402", 0.10),
    model("HCS-3404-USB", "3404", 0.10),
    model("HCS-3600-USB", "3600", 0.10),
    model("HCS-3602-USB", "3602", 0.10),
    model("HCS-3604-USB", "3604", 0.10),
];

/// The model whose ID this is, with or without its `HCS-`.
pub fn lookup(id: &[u8]) -> Option<&'static Model> {
    let code = id.strip_prefix(b"HCS-").unwrap_or(id);
    MODELS.iter().find(|m| m.code.as_bytes() == code)
}

/// A reply as C reads it: a string, up to its first NUL.
pub fn c_string(reply: &[u8]) -> &[u8] {
    let n = reply.iter().position(|c| *c == 0).unwrap_or(reply.len());
    &reply[..n]
}

/// The reply's first line: what `g_strsplit` leaves in `tokens[0]`.
pub fn first_line(reply: &[u8]) -> &[u8] {
    let text = c_string(reply);
    let n = text.iter().position(|c| *c == b'\r').unwrap_or(text.len());
    &text[..n]
}

/// How many bytes `f(c)` holds for at the start of `text`.
fn span(text: &[u8], f: impl Fn(u8) -> bool) -> usize {
    text.iter().take_while(|c| f(**c)).count()
}

/// A hexadecimal float after its `0x`, as far as it goes: `None` with no digit.
fn hex_float(text: &[u8]) -> Option<f64> {
    let whole = span(text, |c| c.is_ascii_hexdigit());
    let fraction = if text.get(whole) == Some(&b'.') {
        span(&text[whole + 1..], |c| c.is_ascii_hexdigit())
    } else {
        0
    };
    if whole + fraction == 0 {
        return None;
    }
    let digit = |c: &u8| f64::from(char::from(*c).to_digit(16).expect("a hex digit"));
    let mut value = text[..whole].iter().fold(0.0, |v, c| v * 16.0 + digit(c));
    let mut scale = 1.0 / 16.0;
    for c in text[whole..].iter().skip(1).take(fraction) {
        value += digit(c) * scale;
        scale /= 16.0;
    }
    let used = if fraction > 0 { whole + 1 + fraction } else { whole };
    let rest = &text[used..];
    if matches!(rest.first(), Some(b'p' | b'P')) {
        let sign = usize::from(matches!(rest.get(1), Some(b'+' | b'-')));
        let digits = span(&rest[1 + sign..], |c| c.is_ascii_digit());
        if digits > 0 {
            let end = 1 + sign + digits;
            let exp = std::str::from_utf8(&rest[1..end]).ok()?.parse::<i32>().unwrap_or(i32::MAX);
            value *= 2f64.powi(exp);
        }
    }
    Some(value)
}

/// `g_ascii_strtod`: the number `text` starts with, or `None` where C's would
/// convert nothing and give 0.
pub fn strtod(text: &[u8]) -> Option<f64> {
    let spaces = span(text, |c| matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r'));
    let text = &text[spaces..];
    let negative = text.first() == Some(&b'-');
    let text = text.strip_prefix(b"-").or_else(|| text.strip_prefix(b"+")).unwrap_or(text);
    let sign = if negative { -1.0 } else { 1.0 };
    let starts =
        |word: &[u8]| text.len() >= word.len() && text[..word.len()].eq_ignore_ascii_case(word);
    if starts(b"inf") {
        return Some(sign * f64::INFINITY);
    }
    if starts(b"nan") {
        return Some(f64::NAN);
    }
    if starts(b"0x")
        && let Some(value) = hex_float(&text[2..])
    {
        return Some(sign * value);
    }
    let whole = span(text, |c| c.is_ascii_digit());
    let fraction = if text.get(whole) == Some(&b'.') {
        span(&text[whole + 1..], |c| c.is_ascii_digit())
    } else {
        0
    };
    if whole + fraction == 0 {
        return None;
    }
    let mut used = if text.get(whole) == Some(&b'.') { whole + 1 + fraction } else { whole };
    if matches!(text.get(used), Some(b'e' | b'E')) {
        let exp_sign = usize::from(matches!(text.get(used + 1), Some(b'+' | b'-')));
        let digits = span(&text[(used + 1 + exp_sign).min(text.len())..], |c| c.is_ascii_digit());
        if digits > 0 {
            used += 1 + exp_sign + digits;
        }
    }
    let value: f64 = std::str::from_utf8(&text[..used]).ok()?.parse().ok()?;
    Some(sign * value)
}

/// `GETD`'s first line as volts and amperes, or `None` where it is not eight
/// digits and a mode. C takes what `strtod` makes of each field, 0 for no number
/// and whatever lies past a short line, so a fragment of a reply that joins
/// the next makes up a reading. Eight digits is the form the wiki gives; no
/// firmware is known to send another, and a recording could say otherwise. The
/// mode byte is not read, as no key shows it.
pub fn parse_getd(line: &[u8]) -> Option<(f32, f32)> {
    if !is_reading(line) {
        return None;
    }
    let field = |from: usize| {
        let digits = &line[from..from + 4];
        digits.iter().fold(0.0, |value, digit| value * 10.0 + f64::from(digit - b'0'))
    };
    Some(((field(0) / 100.0) as f32, (field(4) / 100.0) as f32))
}

/// `GMAX`'s first line as the limits, volts and amperes.
///
/// A field that makes no number, or a line too short to have it, is a limit of
/// 0: no voltage can then be set, as the lowest is 1 V, and a current of 0
/// still can. C gets there by strtod's 0, but for what lies past a short line.
/// A limit that is not finite is 0 too, where C takes `inf` and passes
/// `voltage_target=inf` on as `VOLTinf`. Any finite value stands as in C, so a
/// garbled `GMAX` above the rating is not caught: nothing here knows the rating.
pub fn parse_gmax(line: &[u8], model: &Model) -> (f32, f32) {
    let limit = |value: f64| Some(value as f32).filter(|v| v.is_finite()).unwrap_or(0.0);
    let volts = strtod(&line[..line.len().min(3)]).unwrap_or(0.0) * VOLTAGE_STEP;
    let amps = line.get(3..).and_then(strtod).unwrap_or(0.0) * model.current_step;
    (limit(volts), limit(amps))
}

/// Whether a line is `GETD`'s, in the form the wiki gives: eight digits and a
/// mode.
pub fn is_reading(line: &[u8]) -> bool {
    matches!(line.len(), 8 | 9) && line[..8].iter().all(u8::is_ascii_digit)
}

/// `VOLT%03.0f\r`.
pub fn voltage_command(volts: f64) -> String {
    format!("VOLT{:03.0}\r", volts / VOLTAGE_STEP)
}

/// `CURR%03.0f\r`.
pub fn current_command(amps: f64, model: &Model) -> String {
    format!("CURR{:03.0}\r", amps / model.current_step)
}

/// `SOUT%1d\r`: 0 is on.
pub fn output_command(on: bool) -> String {
    format!("SOUT{}\r", u8::from(!on))
}

/// What a capture has of a reply: `dev_context`'s `buf`, without the running
/// off its end that C does (`protocol.c:184`). C looks for the end as a string
/// (`protocol.c:192`), so after a NUL no reply ends and the buffer runs over;
/// here a NUL costs the reply it is in.
#[derive(Debug, Default)]
pub struct ReplyBuffer {
    bytes: Vec<u8>,
    overrun: bool,
}

impl ReplyBuffer {
    /// Takes a byte. A reply that ended gives its bytes, or none if it was too
    /// long to keep.
    pub fn push(&mut self, byte: u8) -> Option<Vec<u8>> {
        if self.bytes.len() < CAPTURE_MAX {
            self.bytes.push(byte);
        } else {
            log::warn!("A reply of over {CAPTURE_MAX} bytes with no OK, dropped.");
            self.overrun = true;
            self.bytes.clear();
            self.bytes.push(byte);
        }
        if !self.bytes.ends_with(OK) {
            return None;
        }
        let bytes = std::mem::take(&mut self.bytes);
        Some(if std::mem::take(&mut self.overrun) { Vec::new() } else { bytes })
    }
}

/// The port and what the driver knows of the device: `dev_context`.
pub struct Link {
    /// The port, while open.
    pub port: Option<Box<dyn Port>>,
    /// The model.
    pub model: &'static Model,
    /// Volts last read.
    pub voltage: f32,
    /// Amperes last read.
    pub current: f32,
    /// The target last set: 0 until then, as nothing asks the device.
    pub voltage_target: f32,
    /// The limit last set: 0 until then.
    pub current_limit: f32,
    /// What `GMAX` said of the voltage.
    pub voltage_max: f32,
    /// What `GMAX` said of the current.
    pub current_max: f32,
    /// Whether the output is on: the last reading's voltage was not 0, or a
    /// setting said so.
    pub output_enabled: bool,
    /// A capture's `GETD` has not had its reply yet.
    pub reply_pending: bool,
    /// What has come of that reply.
    pub buffer: ReplyBuffer,
    /// A setting dropped a capture's reading, which is no silence of the
    /// device's: the capture takes this as its `no reading` clock's restart.
    pub reading_dropped: bool,
}

impl Link {
    /// A link on `port`, with nothing read yet.
    pub fn new(port: Option<Box<dyn Port>>, model: &'static Model) -> Link {
        Link {
            port,
            model,
            voltage: 0.0,
            current: 0.0,
            voltage_target: 0.0,
            current_limit: 0.0,
            voltage_max: 0.0,
            current_max: 0.0,
            output_enabled: false,
            reply_pending: false,
            buffer: ReplyBuffer::default(),
            reading_dropped: false,
        }
    }

    fn port(&mut self) -> Result<&mut Box<dyn Port>> {
        self.port.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `hcs_send_cmd`.
    pub fn send(&mut self, command: &str) -> Result<()> {
        log::debug!("Sending '{}'.", command.escape_default());
        self.port()?.write(command.as_bytes()).inspect_err(|e| {
            log::error!("Error sending command: {e}.");
        })
    }

    /// One byte within `timeout`, if it comes.
    pub fn read_byte(&mut self, timeout: Duration) -> Result<Option<u8>> {
        let mut byte = [0];
        let n = read_blocking(self.port()?.as_mut(), &mut byte, timeout)?;
        Ok((n == 1).then_some(byte[0]))
    }

    /// `hcs_read_reply`: `lines` lines, the last of them ending in `OK`.
    ///
    /// # Errors
    ///
    /// When they do not come within [`REPLY_TIMEOUT`], where C waits for ever,
    /// or do not end so.
    pub fn read_reply(&mut self, lines: usize) -> Result<Vec<u8>> {
        self.read_reply_by(lines, Instant::now() + REPLY_TIMEOUT)
    }

    /// A setting's answer. A capture's reply that came after its 500 ms is not
    /// that answer, and would fail a setting the device took: it is passed
    /// over, its line and its `OK`.
    ///
    /// # Errors
    ///
    /// As [`Link::read_reply`].
    pub fn read_ack(&mut self) -> Result<()> {
        let deadline = Instant::now() + REPLY_TIMEOUT;
        loop {
            match self.read_lines(1, deadline)? {
                line if is_reading(first_line(&line)) => {
                    log::debug!("A late reading in the way: '{}'.", line.escape_ascii());
                    self.read_lines(1, deadline)?;
                }
                line if c_string(&line).ends_with(OK) => return Ok(()),
                _ => return Err(Error::device("the device did not answer OK")),
            }
        }
    }

    fn read_reply_by(&mut self, lines: usize, deadline: Instant) -> Result<Vec<u8>> {
        let reply = self.read_lines(lines, deadline)?;
        let seen = reply.iter().filter(|c| **c == b'\r').count();
        if seen == lines && c_string(&reply).ends_with(OK) {
            Ok(reply)
        } else {
            Err(Error::device("the device did not answer OK"))
        }
    }

    /// `lines` lines, or as much as [`REPLY_MAX`] takes.
    fn read_lines(&mut self, lines: usize, deadline: Instant) -> Result<Vec<u8>> {
        let mut reply = Vec::with_capacity(REPLY_MAX);
        let mut seen = 0;
        while seen < lines && reply.len() < REPLY_MAX {
            let left = deadline.saturating_duration_since(Instant::now());
            let Some(byte) = self.read_byte(left)? else {
                return Err(Error::Timeout);
            };
            seen += usize::from(byte == b'\r');
            reply.push(byte);
        }
        Ok(reply)
    }

    /// A question whose answer is one line and `OK`.
    pub fn ask(&mut self, command: &str) -> Result<Vec<u8>> {
        self.send(command)?;
        self.read_reply(2)
    }

    /// `hcs_parse_volt_curr_mode` on a reply, whether it made a reading.
    pub fn take_getd(&mut self, reply: &[u8]) -> bool {
        log::debug!("Received '{}'.", reply.escape_ascii());
        let Some((voltage, current)) = parse_getd(first_line(reply)) else {
            log::warn!("A reading not in its form, dropped: '{}'.", reply.escape_ascii());
            return false;
        };
        self.voltage = voltage;
        self.current = current;
        // Works because the voltage cannot be set to 0.0 directly, says C.
        self.output_enabled = voltage != 0.0;
        true
    }

    /// A setting and its `OK`.
    pub fn apply(&mut self, command: &str) -> Result<()> {
        self.settle();
        self.send(command)?;
        self.read_ack()
    }

    /// What goes ahead of a setting. A capture's reply on its way would be
    /// taken for the setting's, which C does (only a library can ask during
    /// one): that reading is dropped first. So is what has come and is no one's,
    /// as the late `OK` of a setting that timed out, which C would take for the
    /// next setting's. One still on its way can't be told apart.
    pub fn settle(&mut self) {
        if self.reply_pending {
            self.finish_pending();
        }
        if let Some(Err(e)) = self.port.as_mut().map(|port| port.purge()) {
            log::debug!("{e}");
        }
    }

    /// Waits out a capture's reply, and forgets it.
    fn finish_pending(&mut self) {
        let deadline = Instant::now() + REQ_TIMEOUT;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match self.read_byte(left) {
                Ok(Some(byte)) => {
                    if self.buffer.push(byte).is_some() {
                        break;
                    }
                }
                _ => break,
            }
        }
        self.buffer = ReplyBuffer::default();
        self.reply_pending = false;
        self.reading_dropped = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::Params;
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    /// A port whose next read gives these bytes, and that notes nothing.
    fn link_with(input: &[u8], ack: &[u8]) -> (Link, Arc<Mutex<Vec<Vec<u8>>>>) {
        let written = Arc::new(Mutex::new(Vec::new()));
        let port = Script {
            input: input.iter().copied().collect(),
            ack: ack.to_vec(),
            written: Arc::clone(&written),
        };
        (Link::new(Some(Box::new(port)), lookup(b"3202").unwrap()), written)
    }

    struct Script {
        input: VecDeque<u8>,
        ack: Vec<u8>,
        written: Arc<Mutex<Vec<Vec<u8>>>>,
    }

    impl Port for Script {
        fn name(&self) -> &'static str {
            "/dev/ttySCRIPT"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], _: Duration) -> Result<usize> {
            let Some(byte) = self.input.pop_front() else { return Ok(0) };
            buf[0] = byte;
            Ok(1)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            self.written.lock().unwrap().push(data.to_vec());
            self.input.extend(self.ack.iter());
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.input.clear();
            Ok(())
        }
    }

    #[test]
    fn a_late_ack_is_not_the_next_settings_answer() {
        let (mut link, written) = link_with(b"OK\r", b"ERR\r");
        assert!(link.apply("SOUT1\r").is_err(), "the device said ERR to this one");
        assert_eq!(written.lock().unwrap().len(), 1);
    }

    #[test]
    fn a_late_reading_is_passed_over_for_the_settings_answer() {
        let (mut link, _) = link_with(b"", b"120001200\rOK\rOK\r");
        link.apply("SOUT1\r").expect("the setting's OK comes after the reading's");
        let (mut link, _) = link_with(b"", b"120001200\rOK\rERR\r");
        assert!(link.apply("SOUT1\r").is_err(), "and its answer still counts");
        let (mut link, _) = link_with(b"", b"12\rOK\r");
        assert!(link.apply("SOUT1\r").is_err(), "no reading, so no answer");
    }

    #[test]
    fn a_reading_has_its_form() {
        for good in [&b"12340100"[..], b"123401000", b"123401001"] {
            assert!(is_reading(good), "{}", good.escape_ascii());
        }
        for bad in [&b""[..], b"OK", b"1234010", b"1234010000", b"1234x1000"] {
            assert!(!is_reading(bad), "{}", bad.escape_ascii());
        }
    }

    #[test]
    fn every_model_has_two_ids() {
        for m in &MODELS {
            assert_eq!(lookup(m.code.as_bytes()), Some(m));
            assert_eq!(lookup(format!("HCS-{}", m.code).as_bytes()), Some(m));
        }
        for bad in [&b""[..], b"HCS-", b"hcs-3202", b"3202 ", b"HCS-HCS-3202", b"3999"] {
            assert_eq!(lookup(bad), None, "{}", bad.escape_ascii());
        }
    }

    #[test]
    fn strtod_goes_as_far_as_it_can() {
        let cases: &[(&[u8], Option<f64>)] = &[
            (b"1234", Some(1234.0)),
            (b" 12 ", Some(12.0)),
            (b"1.2 ", Some(1.2)),
            (b"-100", Some(-100.0)),
            (b"+1", Some(1.0)),
            (b"1e01", Some(10.0)),
            (b"1e", Some(1.0)),
            (b"1e+", Some(1.0)),
            (b".5", Some(0.5)),
            (b"5.", Some(5.0)),
            (b"12x4", Some(12.0)),
            (b"0x10", Some(16.0)),
            (b"0x", Some(0.0)),
            (b"0x.8", Some(0.5)),
            (b"0x1p2", Some(4.0)),
            (b"inf", Some(f64::INFINITY)),
            (b"-INF", Some(f64::NEG_INFINITY)),
            (b"", None),
            (b"xxxx", None),
            (b".", None),
            (b"-", None),
            (b"e5", None),
        ];
        for (text, want) in cases {
            assert_eq!(strtod(text), *want, "{}", text.escape_ascii());
        }
        assert!(strtod(b"nanx").unwrap().is_nan());
    }

    #[test]
    fn a_reading_has_two_fields() {
        assert_eq!(parse_getd(b"123401000"), Some((12.34, 1.0)));
        assert_eq!(parse_getd(b"12340100"), Some((12.34, 1.0)), "with no mode");
        assert_eq!(parse_getd(b"1234"), None, "no current");
        assert_eq!(parse_getd(b"12"), None);
        assert_eq!(parse_getd(b""), None);
        assert_eq!(parse_getd(b"xxxx0100"), None);
        assert_eq!(parse_getd(b"1234yyyy"), None);
        assert_eq!(parse_getd(b"1e0102e1"), None, "strtod's, not the device's");
        assert_eq!(parse_getd(b" 12 100"), None);
        assert_eq!(parse_getd(b"infxinfx0"), None);
        assert_eq!(parse_getd(b"12123401000"), None, "a fragment and a reading");
    }

    #[test]
    fn gmax_scales_by_the_models_units() {
        let tenths = lookup(b"3202").unwrap();
        let hundredths = lookup(b"3104").unwrap();
        assert_eq!(parse_gmax(b"360100", tenths), (36.0, 10.0));
        assert_eq!(parse_gmax(b"600250", hundredths), (60.0, 2.5));
        assert_eq!(parse_gmax(b"362102", tenths), (36.2, 10.2));
        assert_eq!(parse_gmax(b"xxxyyy", tenths), (0.0, 0.0));
        assert_eq!(parse_gmax(b"360", tenths), (36.0, 0.0));
        assert_eq!(parse_gmax(b"36", tenths), (3.6, 0.0), "too short to have a current");
        assert_eq!(parse_gmax(b"", tenths), (0.0, 0.0));
        assert_eq!(parse_gmax(b"infinf", tenths), (0.0, 0.0));
        assert_eq!(parse_gmax(b"nannan", tenths), (0.0, 0.0));
        assert_eq!(parse_gmax(b"3601e99", tenths), (36.0, 0.0), "too much for a float");
        assert_eq!(parse_gmax(b"999999", tenths), (99.9, 99.9), "finite: as in C");
    }

    #[test]
    fn commands_are_as_printf_makes_them() {
        let tenths = lookup(b"3202").unwrap();
        let hundredths = lookup(b"3104").unwrap();
        assert_eq!(voltage_command(5.0), "VOLT050\r");
        assert_eq!(voltage_command(1.0), "VOLT010\r");
        assert_eq!(voltage_command(36.0), "VOLT360\r");
        // 3.25 / 0.1 is 32.5 exactly, and %.0f rounds that to even.
        assert_eq!(voltage_command(3.25), "VOLT032\r");
        assert_eq!(voltage_command(f64::from(3.33f32)), "VOLT033\r");
        assert_eq!(current_command(2.5, tenths), "CURR025\r");
        assert_eq!(current_command(0.0, tenths), "CURR000\r");
        assert_eq!(current_command(2.5, hundredths), "CURR250\r");
        assert_eq!(current_command(f64::from(1.23f32), hundredths), "CURR123\r");
        assert_eq!(output_command(true), "SOUT0\r");
        assert_eq!(output_command(false), "SOUT1\r");
    }

    #[test]
    fn a_reply_ends_at_ok() {
        let mut b = ReplyBuffer::default();
        for c in b"12\rOK" {
            assert_eq!(b.push(*c), None);
        }
        assert_eq!(b.push(b'\r').as_deref(), Some(&b"12\rOK\r"[..]));
        for c in b"x\0OK" {
            assert_eq!(b.push(*c), None);
        }
        let reply = b.push(b'\r').expect("a NUL does not hide the end");
        assert_eq!(first_line(&reply), b"x");
        assert_eq!(parse_getd(first_line(&reply)), None);
    }

    #[test]
    fn a_reply_too_long_is_dropped_whole_at_its_ok() {
        let mut b = ReplyBuffer::default();
        for _ in 0..200 {
            assert_eq!(b.push(b'x'), None);
            assert!(b.bytes.len() <= CAPTURE_MAX);
        }
        for c in b"123401000\rOK" {
            assert_eq!(b.push(*c), None);
        }
        assert_eq!(b.push(b'\r').as_deref(), Some(&b""[..]), "its end, not a reading");
        for c in b"123401000\rOK" {
            assert_eq!(b.push(*c), None);
        }
        assert_eq!(b.push(b'\r').as_deref(), Some(&b"123401000\rOK\r"[..]), "the next is whole");
    }
}
