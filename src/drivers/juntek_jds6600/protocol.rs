// Counterpart in sigrok:
//   libsigrok/src/hardware/juntek-jds6600/api.c,
//   libsigrok/src/hardware/juntek-jds6600/protocol.c,
//   libsigrok/src/hardware/juntek-jds6600/protocol.h,
//   Copyright (C) 2023 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The JDS6600's serial protocol.
//!
//! A request is a line, `:rNN=0.` CR LF to read register NN or `:wNN=VALUE.`
//! CR LF to write it. A read's reply is `:rNN=VALUE.` CR LF, a write's `:ok`
//! CR LF. Register numbers, scales and offsets are in the functions here,
//! and were found by experiment on the real device (see `protocol.c`'s notes
//! in libsigrok). The arbitrary waveforms' `a` and `b` are not ported, as
//! they are compiled out there too.
//!
//! Where this leaves `protocol.c`:
//!
//! - A silent device ends a wait here, after [`REPLY_BOUND`]. There every
//!   wait but the first one has no end (`timeout_ms` 0, `protocol.c:442-450`).
//! - A frequency of fewer than two words is an error here. There the next
//!   parser gets NULL and the program dies (`protocol.c:602-603`, `608-609`).
//! - The first wait is [`IDENTIFY_TIMEOUT`], for a host that is busy or a USB
//!   path that is slow, where C has 10 ms, measured at the device's UART.
//! - The input queue is dropped before each request, which costs nothing on
//!   the wire, so one late reply does not shift all that follow.
//! - Hexadecimal floats and `nan(...)` in a reply are refused. `strtof` takes
//!   them, and no device was seen to send them.

use crate::core::{Error, Result};
use crate::serial::{self, Port};
use std::time::{Duration, Instant};

/// `TIMEOUT_READ_CHUNK`: one read's wait, after which a reply so far is looked at.
const READ_CHUNK: Duration = Duration::from_millis(2);
/// How long a scan waits for the first reply. C has 10 ms.
pub const IDENTIFY_TIMEOUT: Duration = Duration::from_millis(100);
/// How long any other request waits for its reply, in silence: as long as C's
/// longest (`TIMEOUT_WAVEFORM`), which is for 2048 samples, where C waits for
/// ever here.
pub const REPLY_BOUND: Duration = Duration::from_millis(1200);
/// What one reply may hold: glib's `GString` rounds `MAX_RSP_LENGTH`'s 10248
/// up to 16384 bytes, and `serial_recv_textline` keeps one for the NUL.
const REPLY_BUFFER: usize = 16383;

/// The number of generator channels.
pub const CHANNELS: usize = 2;
const REG_TYPE: usize = 0;
const REG_SERIAL: usize = 1;
const REG_ENABLE: usize = 20;
const REG_WAVEFORM: usize = 21;
const REG_FREQUENCY: usize = 23;
const REG_AMPLITUDE: usize = 25;
const REG_OFFSET: usize = 27;
const REG_DUTY: usize = 29;
const REG_PHASE: usize = 31;

/// The built in waveforms, whose codes are their positions.
const BUILTIN_WAVES: [&str; 17] = [
    "sine",
    "square",
    "pulse",
    "triangle",
    "partial-sine",
    "cmos",
    "dc",
    "half-wave",
    "full-wave",
    "pos-ladder",
    "neg-ladder",
    "noise",
    "exp-rise",
    "exp-decay",
    "multi-tone",
    "sinc",
    "lorenz",
];
/// The arbitrary waveforms, `arb-01` on, code 101 on.
const ARBITRARY_WAVES: usize = 60;
const ARBITRARY_BASE: usize = 100;

/// All waveform names, in the order `--show` lists them.
pub fn wave_names() -> Vec<String> {
    let arbitrary = (1..=ARBITRARY_WAVES).map(|n| format!("arb-{n:02}"));
    BUILTIN_WAVES.iter().map(|s| (*s).to_owned()).chain(arbitrary).collect()
}

/// The firmware's code of the waveform at `index` of [`wave_names`].
pub fn wave_code(index: usize) -> usize {
    if index < BUILTIN_WAVES.len() {
        index
    } else {
        index - BUILTIN_WAVES.len() + ARBITRARY_BASE + 1
    }
}

/// Where `code` stands in [`wave_names`].
pub fn wave_index(code: u64) -> Option<usize> {
    let code = usize::try_from(code).ok()?;
    if code < BUILTIN_WAVES.len() {
        return Some(code);
    }
    let arbitrary = code.checked_sub(ARBITRARY_BASE + 1)?;
    (arbitrary < ARBITRARY_WAVES).then_some(BUILTIN_WAVES.len() + arbitrary)
}

fn data(why: &str) -> Error {
    Error::Device(why.to_owned())
}

/// `isspace` in the C locale, which has `\v` where `u8::is_ascii_whitespace`
/// does not.
fn is_space(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

fn skip_spaces(mut s: &[u8]) -> &[u8] {
    while let [first, rest @ ..] = s
        && is_space(*first)
    {
        s = rest;
    }
    s
}

/// `sr_atoul_base` at base 10: the number and what follows it past white
/// space. As `strtoul` has it, no digits are 0 with nothing taken, a minus
/// sign wraps, and only a number too big fails.
pub fn c_strtoul(text: &[u8]) -> Result<(u64, &[u8])> {
    let start = skip_spaces(text);
    let (negative, digits) = match start {
        [b'-', rest @ ..] => (true, rest),
        [b'+', rest @ ..] => (false, rest),
        _ => (false, start),
    };
    let count = digits.iter().take_while(|c| c.is_ascii_digit()).count();
    if count == 0 {
        return Ok((0, skip_spaces(start)));
    }
    let mut number = 0u64;
    for c in &digits[..count] {
        number = number
            .checked_mul(10)
            .and_then(|n| n.checked_add(u64::from(c - b'0')))
            .ok_or_else(|| data("a number too big"))?;
    }
    let number = if negative { number.wrapping_neg() } else { number };
    Ok((number, skip_spaces(&digits[count..])))
}

/// `sr_atod`: the whole text as a float, in single precision as it reads it
/// with `strtof`. Nothing at all is 0, as `strtof` takes nothing there and no
/// byte is left over; a number out of a float's range fails.
pub fn c_atod(text: &[u8]) -> Result<f64> {
    let bad = || data("no number");
    let mut s = skip_spaces(text);
    while let [rest @ .., last] = s
        && is_space(*last)
    {
        s = rest;
    }
    if s.is_empty() {
        return Ok(0.0);
    }
    let shown = std::str::from_utf8(s).map_err(|_| bad())?;
    let value: f32 = shown.parse().map_err(|_| bad())?;
    let digits = |t: &str| t.split(['e', 'E']).next().is_some_and(has_non_zero_digit);
    let named = shown.bytes().any(|c| c.is_ascii_alphabetic() && !matches!(c, b'e' | b'E'));
    if !named && (value.is_infinite() || (!value.is_normal() && digits(shown))) {
        return Err(data("a number out of range"));
    }
    Ok(f64::from(value))
}

fn has_non_zero_digit(text: &str) -> bool {
    text.bytes().any(|c| (b'1'..=b'9').contains(&c))
}

/// `replace_separators` and `sr_text_next_word` in one: the words of a value
/// list, where commas and a period at the very end are white space.
fn words(rhs: &[u8]) -> Vec<&[u8]> {
    let body = rhs.strip_suffix(b".").unwrap_or(rhs);
    body.split(|c| is_space(*c) || *c == b',').filter(|w| !w.is_empty()).collect()
}

/// `parse_freq_text`: centihertz and a scale code.
pub fn parse_frequency(rhs: &[u8]) -> Result<f64> {
    /// `scales_freq`.
    const SCALES: [f64; 5] = [1.0, 1.0, 1.0, 1e-3, 1e-6];
    let words = words(rhs);
    let mantissa = c_atod(words.first().ok_or_else(|| data("no frequency"))?)?;
    let (code, _) = c_strtoul(words.get(1).ok_or_else(|| data("no frequency scale"))?)?;
    let scale = usize::try_from(code)
        .ok()
        .and_then(|i| SCALES.get(i))
        .ok_or_else(|| data("no such frequency scale"))?;
    Ok(mantissa / 100.0 / scale)
}

/// `parse_volt_text`: millivolts.
pub fn parse_volts(rhs: &[u8]) -> Result<f64> {
    Ok(c_atod(rhs)? / 1000.0)
}

/// `parse_bias_text`: tens of millivolts, 0 V at 10 V, capped at 9.99 V.
pub fn parse_offset(rhs: &[u8]) -> Result<f64> {
    let volts = c_atod(rhs)? / 100.0 - 10.0;
    // Two comparisons, as a reply's NaN goes through them.
    let volts = if volts >= 9.99 { 9.99 } else { volts };
    Ok(if volts <= -9.99 { -9.99 } else { volts })
}

/// `parse_duty_text`: tenths of a percent, as 0 to 1.
pub fn parse_duty(rhs: &[u8]) -> Result<f64> {
    Ok(c_atod(rhs)? / 1000.0)
}

/// `parse_phase_text`: tenths of a degree.
pub fn parse_phase(rhs: &[u8]) -> Result<f64> {
    Ok(c_atod(rhs)? / 10.0)
}

/// `%.0f`, but for a negative zero, where C sends `-0`: nothing says the
/// firmware reads that.
fn whole(value: f64) -> String {
    let text = format!("{value:.0}");
    if text == "-0" { "0".to_owned() } else { text }
}

/// `write_freq_text` with `jds6600_set_frequency`'s limits. Always scale 0.
pub fn frequency_text(hertz: f64, top: u64) -> String {
    let hertz = if hertz < 0.01 { 0.01 } else { hertz };
    let hertz = if hertz > top as f64 { top as f64 } else { hertz };
    format!("{},0", whole(hertz * 100.0))
}

/// `write_volt_text`, kept to 0 to 20 V.
pub fn volts_text(volts: f64) -> String {
    let volts = if volts > 20.0 { 20.0 } else { volts };
    let volts = if volts < 0.0 { 0.0 } else { volts };
    whole(volts * 1000.0)
}

/// `write_bias_text`, kept to 9.99 V either way.
pub fn offset_text(volts: f64) -> String {
    let volts = if volts > 9.99 { 9.99 } else { volts };
    let volts = if volts < -9.99 { -9.99 } else { volts };
    whole((volts + 10.0) * 100.0)
}

/// `write_duty_text`, kept to 0 to 1.
pub fn duty_text(duty: f64) -> String {
    let duty = if duty < 0.0 { 0.0 } else { duty };
    let duty = if duty > 1.0 { 1.0 } else { duty };
    whole(duty * 1000.0)
}

/// `write_phase_text`, modulo a turn with the sign kept.
pub fn phase_text(degrees: f64) -> String {
    whole(degrees % 360.0 * 10.0)
}

/// `serial_recv_textline` once the line is here, for a write's reply: `:ok`,
/// with or without the period.
fn check_ok(line: &[u8]) -> Result<()> {
    if frame(line)? == b"ok" { Ok(()) } else { Err(data("not 'ok'")) }
}

/// White space off the end and the period, but not off the start, as that
/// function drops what `sr_text_trim_spaces` returns. Then the colon.
fn frame(line: &[u8]) -> Result<&[u8]> {
    let mut s = line;
    while let [rest @ .., last] = s
        && is_space(*last)
    {
        s = rest;
    }
    let s = s.strip_suffix(b".").unwrap_or(s);
    s.strip_prefix(b":").ok_or_else(|| {
        log::debug!("serial read, colon missing");
        data("no colon")
    })
}

/// The same for a read's reply: `insn`, the register unless that is 0, whose
/// number goes unchecked in C, then `=`. What follows it is the value.
fn check_read(line: &[u8], insn: u8, register: usize) -> Result<&[u8]> {
    let s = frame(line)?;
    let [got, rest @ ..] = s else {
        log::debug!("serial read, unexpected insn");
        return Err(data("no instruction"));
    };
    if *got != insn {
        log::debug!("serial read, unexpected insn");
        return Err(data("another instruction"));
    }
    let (index, rest) = c_strtoul(rest)?;
    if register != 0 && index != register as u64 {
        log::debug!("serial read, unexpected index {index}");
        return Err(data("another register"));
    }
    rest.strip_prefix(b"=").ok_or_else(|| {
        log::debug!("serial read, equals sign missing");
        data("no equals sign")
    })
}

/// Holds on to a reply that did not fit, for the end of a scan. Anything else
/// (a device gone silent) ends the scan now.
fn keep(result: Result<()>, failed: &mut Option<Error>) -> Result<()> {
    match result {
        Err(e @ Error::Device(_)) => {
            log::debug!("Scan: {e}.");
            *failed = Some(e);
            Ok(())
        }
        other => other,
    }
}

/// The port and what must be known between requests.
pub struct Link {
    /// `None` while closed.
    pub port: Option<Box<dyn Port>>,
    /// What the device last said, or was last told, of each output.
    pub enabled: [bool; CHANNELS],
    /// The model's top frequency, in Hz.
    pub top_frequency: u64,
    /// The serial number the scan read.
    pub serial: String,
}

impl Link {
    /// A link on `port`, knowing nothing yet.
    pub fn new(port: Option<Box<dyn Port>>) -> Link {
        Link { port, enabled: [false; CHANNELS], top_frequency: 0, serial: String::new() }
    }

    fn port(&mut self) -> Result<&mut Box<dyn Port>> {
        self.port.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `serial_send_textline`.
    fn send(&mut self, text: &str) -> Result<()> {
        log::trace!("serial TX text: --> {text}.");
        let port = self.port()?;
        port.purge()?;
        port.write(format!("{text}.\r\n").as_bytes())
    }

    /// `serial_recv_textline`'s reading: a line, without what is on it after
    /// the first NUL. What comes behind the line feed in the same read is
    /// gone. Only a read that gave nothing looks at the clock.
    fn receive(&mut self, bound: Duration) -> Result<Vec<u8>> {
        let port = self.port()?;
        let deadline = Instant::now() + bound;
        let mut buffer = vec![0u8; REPLY_BUFFER];
        let mut have = 0;
        while have < REPLY_BUFFER {
            let got = serial::read_blocking(port.as_mut(), &mut buffer[have..], READ_CHUNK)?;
            let chunk = &buffer[have..have + got];
            // `strchr` stops at a NUL, so a line feed behind one does not count.
            let text = chunk.split(|c| *c == 0).next().unwrap_or(chunk);
            if let Some(at) = text.iter().position(|c| *c == b'\n') {
                let end = have + at;
                let line = buffer[..end].split(|c| *c == 0).next().unwrap_or(&[]).to_vec();
                log::trace!("serial RX text: <-- {}", line.escape_ascii());
                return Ok(line);
            }
            have += got;
            if got == 0 && Instant::now() >= deadline {
                log::debug!("serial read, unterminated response, discarded");
                return Err(Error::Timeout);
            }
        }
        log::debug!("serial read, unterminated response, discarded");
        Err(data("a reply with no end of line"))
    }

    /// `quick_send_read_then_recv`: register `register`'s value.
    fn read_register(&mut self, register: usize, bound: Duration) -> Result<Vec<u8>> {
        self.send(&format!(":r{register:02}=0"))?;
        let line = self.receive(bound)?;
        check_read(&line, b'r', register).map(<[u8]>::to_vec)
    }

    fn read(&mut self, register: usize) -> Result<Vec<u8>> {
        self.read_register(register, REPLY_BOUND)
    }

    /// `quick_send_write_then_recv_ok`.
    fn write(&mut self, register: usize, value: &str) -> Result<()> {
        self.send(&format!(":w{register:02}={value}"))?;
        let line = self.receive(REPLY_BOUND)?;
        check_ok(&line)
    }

    /// `jds6600_identify`: the type, which is the model's top frequency in
    /// MHz, in the width C keeps it, and the serial number.
    fn identify(&mut self) -> Result<u32> {
        let reply = self.read_register(REG_TYPE, IDENTIFY_TIMEOUT)?;
        let (kind, _) = c_strtoul(&reply)?;
        log::debug!("identify, device type '{}'", reply.escape_ascii());
        let reply = self.read(REG_SERIAL)?;
        self.serial = String::from_utf8_lossy(&reply).into_owned();
        log::debug!("identify, serial number '{}'", self.serial);
        Ok(kind as u32)
    }

    /// A scan's part once the port is open: `jds6600_identify`, then
    /// `jds6600_setup_devc`'s reading of all the state. As there, a reply
    /// that does not fit fails the scan only at the end, and the requests
    /// that follow still go, up to the end of the first channel in which one
    /// failed, then the phase. A device that went silent ends it at once.
    ///
    /// # Errors
    ///
    /// When anything failed: there is no such device.
    pub fn probe(&mut self) -> Result<()> {
        let kind = self.identify()?;
        if kind == 0 {
            return Err(data("device type 0"));
        }
        self.top_frequency = u64::from(kind) * 1_000_000;
        let mut failed = None;
        keep(self.get_enabled(), &mut failed)?;
        for channel in 0..CHANNELS {
            keep(self.get_waveform(channel).map(drop), &mut failed)?;
            keep(self.get_frequency(channel).map(drop), &mut failed)?;
            keep(self.get_amplitude(channel).map(drop), &mut failed)?;
            keep(self.get_offset(channel).map(drop), &mut failed)?;
            keep(self.get_duty(channel).map(drop), &mut failed)?;
            if failed.is_some() {
                break;
            }
        }
        keep(self.get_phase().map(drop), &mut failed)?;
        failed.map_or(Ok(()), Err)
    }

    /// `jds6600_get_chans_enable`. A flag is any number, cut to C's `int`.
    /// The first of two is kept when the second fails.
    pub fn get_enabled(&mut self) -> Result<()> {
        let reply = self.read(REG_ENABLE)?;
        log::debug!("get enabled, response text: {}", reply.escape_ascii());
        let words = words(&reply);
        for channel in 0..CHANNELS {
            let word = words.get(channel).ok_or_else(|| data("an enable flag missing"))?;
            let (on, rest) = c_strtoul(word)?;
            if !rest.is_empty() {
                return Err(data("an enable flag that is no number"));
            }
            self.enabled[channel] = on as u32 != 0;
        }
        Ok(())
    }

    /// `jds6600_set_chans_enable`, with `enabled` as it would be. It is kept
    /// when the device took it, where C keeps it either way.
    pub fn set_enabled(&mut self, enabled: [bool; CHANNELS]) -> Result<()> {
        let flag = |on: bool| if on { '1' } else { '0' };
        let text = format!("{},{}", flag(enabled[0]), flag(enabled[1]));
        log::debug!("set enabled, request text: {text}");
        self.write(REG_ENABLE, &text)?;
        self.enabled = enabled;
        Ok(())
    }

    /// `jds6600_get_waveform`: where the code stands in [`wave_names`]. What
    /// is no number reads as code 0, and text behind a number is passed over,
    /// as there.
    pub fn get_waveform(&mut self, channel: usize) -> Result<usize> {
        let reply = self.read(REG_WAVEFORM + channel)?;
        log::debug!("get waveform, response text: {}", reply.escape_ascii());
        let (code, _) = c_strtoul(&reply)?;
        wave_index(code).ok_or_else(|| data("no such waveform"))
    }

    /// `jds6600_set_waveform`.
    pub fn set_waveform(&mut self, channel: usize, index: usize) -> Result<()> {
        self.write(REG_WAVEFORM + channel, &wave_code(index).to_string())
    }

    /// `jds6600_get_frequency`.
    pub fn get_frequency(&mut self, channel: usize) -> Result<f64> {
        let reply = self.read(REG_FREQUENCY + channel)?;
        log::debug!("get frequency, response text: {}", reply.escape_ascii());
        parse_frequency(&reply)
    }

    /// `jds6600_set_frequency`.
    pub fn set_frequency(&mut self, channel: usize, hertz: f64) -> Result<()> {
        let text = frequency_text(hertz, self.top_frequency);
        self.write(REG_FREQUENCY + channel, &text)
    }

    /// `jds6600_get_amplitude`.
    pub fn get_amplitude(&mut self, channel: usize) -> Result<f64> {
        let reply = self.read(REG_AMPLITUDE + channel)?;
        log::debug!("get amplitude, response text: {}", reply.escape_ascii());
        parse_volts(&reply)
    }

    /// `jds6600_set_amplitude`.
    pub fn set_amplitude(&mut self, channel: usize, volts: f64) -> Result<()> {
        self.write(REG_AMPLITUDE + channel, &volts_text(volts))
    }

    /// `jds6600_get_offset`.
    pub fn get_offset(&mut self, channel: usize) -> Result<f64> {
        let reply = self.read(REG_OFFSET + channel)?;
        log::debug!("get offset, response text: {}", reply.escape_ascii());
        parse_offset(&reply)
    }

    /// `jds6600_set_offset`.
    pub fn set_offset(&mut self, channel: usize, volts: f64) -> Result<()> {
        self.write(REG_OFFSET + channel, &offset_text(volts))
    }

    /// `jds6600_get_dutycycle`.
    pub fn get_duty(&mut self, channel: usize) -> Result<f64> {
        let reply = self.read(REG_DUTY + channel)?;
        log::debug!("get duty cycle, response text: {}", reply.escape_ascii());
        parse_duty(&reply)
    }

    /// `jds6600_set_dutycycle`.
    pub fn set_duty(&mut self, channel: usize, duty: f64) -> Result<()> {
        self.write(REG_DUTY + channel, &duty_text(duty))
    }

    /// `jds6600_get_phase_chans`.
    pub fn get_phase(&mut self) -> Result<f64> {
        let reply = self.read(REG_PHASE)?;
        log::debug!("get phase, response text: {}", reply.escape_ascii());
        parse_phase(&reply)
    }

    /// `jds6600_set_phase_chans`.
    pub fn set_phase(&mut self, degrees: f64) -> Result<()> {
        self.write(REG_PHASE, &phase_text(degrees))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    /// A device behind a port: each write takes the next reply, and `None`
    /// is none. A reply stands in the queue until it is read or purged.
    #[derive(Default)]
    pub(crate) struct Script {
        pub(crate) replies: VecDeque<Option<Vec<u8>>>,
        pub(crate) writes: Vec<String>,
        pub(crate) purges: usize,
        input: Vec<u8>,
        /// A reply to come only this long after its request.
        pub(crate) late: Option<(Vec<u8>, Instant)>,
    }

    #[derive(Clone, Default)]
    pub(crate) struct Fake(pub(crate) Arc<Mutex<Script>>);

    impl Fake {
        pub(crate) fn with(replies: &[&str]) -> Fake {
            let fake = Fake::default();
            let script = replies.iter().map(|r| Some(r.as_bytes().to_vec())).collect();
            fake.0.lock().unwrap().replies = script;
            fake
        }

        pub(crate) fn writes(&self) -> Vec<String> {
            self.0.lock().unwrap().writes.clone()
        }

        pub(crate) fn link(&self) -> Link {
            Link::new(Some(Box::new(self.clone())))
        }
    }

    impl Port for Fake {
        fn name(&self) -> &'static str {
            "fake"
        }

        fn set_params(&mut self, _params: serial::Params) -> Result<()> {
            Ok(())
        }

        fn set_handshake(&mut self, _rts: Option<bool>, _dtr: Option<bool>) -> Result<()> {
            Ok(())
        }

        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            {
                let mut s = self.0.lock().unwrap();
                if let Some((_, when)) = &s.late
                    && Instant::now() >= *when
                {
                    let (bytes, _) = s.late.take().unwrap();
                    s.input.extend(bytes);
                }
                if !s.input.is_empty() {
                    let n = s.input.len().min(buf.len());
                    buf[..n].copy_from_slice(&s.input[..n]);
                    s.input.drain(..n);
                    return Ok(n);
                }
            }
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
            Ok(0)
        }

        fn write(&mut self, data: &[u8]) -> Result<()> {
            let mut s = self.0.lock().unwrap();
            s.writes.push(String::from_utf8_lossy(data).into_owned());
            if let Some(Some(reply)) = s.replies.pop_front() {
                s.input.extend(reply);
            }
            Ok(())
        }

        fn purge(&mut self) -> Result<()> {
            let mut s = self.0.lock().unwrap();
            s.purges += 1;
            s.input.clear();
            Ok(())
        }
    }

    /// The replies of a whole scan of a 60 MHz device.
    pub(crate) const SCAN: [&str; 14] = [
        ":r00=60.\r\n",
        ":r01=1234.\r\n",
        ":r20=1,0.\r\n",
        ":r21=103.\r\n",
        ":r23=100000,0.\r\n",
        ":r25=5000.\r\n",
        ":r27=1000.\r\n",
        ":r29=500.\r\n",
        ":r22=0.\r\n",
        ":r24=100000,0.\r\n",
        ":r26=5000.\r\n",
        ":r28=1000.\r\n",
        ":r30=500.\r\n",
        ":r31=0.\r\n",
    ];

    /// The registers of a scan in order, up to `last`, then those it goes to.
    fn requests(registers: &[usize]) -> Vec<String> {
        registers.iter().map(|r| format!(":r{r:02}=0.\r\n")).collect()
    }

    /// Fails the test rather than hanging it, should a bound go.
    fn bounded<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(f());
        });
        rx.recv_timeout(Duration::from_secs(60)).expect("no end in sight")
    }

    #[test]
    fn a_scan_asks_for_all_in_the_c_order_and_sets_nothing() {
        let fake = Fake::with(&SCAN);
        let mut link = fake.link();
        link.probe().unwrap();
        assert_eq!(
            fake.writes(),
            requests(&[0, 1, 20, 21, 23, 25, 27, 29, 22, 24, 26, 28, 30, 31])
        );
        assert_eq!(link.top_frequency, 60_000_000);
        assert_eq!(link.serial, "1234");
        assert_eq!(link.enabled, [true, false]);
    }

    #[test]
    fn a_reply_that_does_not_fit_fails_after_the_channel_and_the_phase() {
        let mut replies = SCAN;
        replies[4] = ":r23=x,0.\r\n";
        replies[8] = ":r31=0.\r\n";
        let fake = Fake::with(&replies);
        assert!(matches!(fake.link().probe(), Err(Error::Device(_))));
        assert_eq!(fake.writes(), requests(&[0, 1, 20, 21, 23, 25, 27, 29, 31]));
    }

    #[test]
    fn an_enable_that_does_not_fit_still_goes_through_the_first_channel() {
        let mut replies = SCAN;
        replies[2] = ":r20=1,x.\r\n";
        replies[8] = ":r31=0.\r\n";
        let fake = Fake::with(&replies);
        assert!(fake.link().probe().is_err());
        assert_eq!(fake.writes(), requests(&[0, 1, 20, 21, 23, 25, 27, 29, 31]));
    }

    #[test]
    fn silence_in_a_scan_ends_it_where_it_is() {
        let fake = Fake::with(&SCAN[..3]);
        fake.0.lock().unwrap().replies.push_back(None);
        let f = fake.clone();
        let result = bounded(move || f.link().probe());
        assert!(matches!(result, Err(Error::Timeout)));
        assert_eq!(fake.writes(), requests(&[0, 1, 20, 21]));
    }

    #[test]
    fn type_0_is_no_device_and_asks_for_no_state() {
        for (reply, name) in [
            (":r00=0.\r\n", "0"),
            (":r00=abc.\r\n", "no digits"),
            (":r00=.\r\n", "none"),
            (":r00=4294967296.\r\n", "33 bits"),
        ] {
            let fake = Fake::with(&[reply, ":r01=1.\r\n"]);
            assert!(fake.link().probe().is_err(), "{name}");
            assert_eq!(fake.writes(), requests(&[0, 1]), "{name}");
        }
    }

    #[test]
    fn the_type_is_kept_in_32_bits_and_a_negative_one_wraps() {
        let mut replies = SCAN;
        replies[0] = ":r00=4294967295.\r\n";
        let mut link = Fake::with(&replies).link();
        link.probe().unwrap();
        assert_eq!(link.top_frequency, 4_294_967_295_000_000);
        replies[0] = ":r00=-1.\r\n";
        let mut link = Fake::with(&replies).link();
        link.probe().unwrap();
        assert_eq!(link.top_frequency, 4_294_967_295_000_000);
    }

    #[test]
    fn the_times_are_as_said() {
        assert_eq!(READ_CHUNK, Duration::from_millis(2));
        assert_eq!(IDENTIFY_TIMEOUT, Duration::from_millis(100));
        assert_eq!(REPLY_BOUND, Duration::from_millis(1200));
        assert_eq!(REPLY_BUFFER, 16383);
    }

    #[test]
    fn a_silent_device_ends_the_first_wait_after_its_time() {
        let fake = Fake::default();
        let mut link = fake.link();
        let started = Instant::now();
        let result = bounded(move || link.read_register(0, IDENTIFY_TIMEOUT));
        assert!(matches!(result, Err(Error::Timeout)));
        assert!(started.elapsed() >= Duration::from_millis(100));
    }

    #[test]
    fn a_silent_device_ends_any_other_wait_after_its_time() {
        for what in 0..2 {
            let fake = Fake::default();
            let mut link = fake.link();
            let started = Instant::now();
            let result =
                bounded(
                    move || {
                        if what == 0 { link.get_phase().map(drop) } else { link.set_phase(1.0) }
                    },
                );
            assert!(matches!(result, Err(Error::Timeout)));
            assert!(started.elapsed() >= Duration::from_millis(1200));
        }
    }

    #[test]
    fn a_reply_slower_than_c_would_wait_is_still_taken_in_a_scan() {
        let fake = Fake::default();
        fake.0.lock().unwrap().late =
            Some((b":r00=60.\r\n".to_vec(), Instant::now() + Duration::from_millis(30)));
        let reply = fake.link().read_register(0, IDENTIFY_TIMEOUT).unwrap();
        assert_eq!(reply, b"60.".strip_suffix(b".").unwrap());
    }

    #[test]
    fn an_endless_line_ends_at_the_buffer_with_no_clock_involved() {
        let fake = Fake::default();
        let long = vec![b'x'; 2 * REPLY_BUFFER];
        fake.0.lock().unwrap().replies.push_back(Some(long));
        let mut link = fake.link();
        let result = bounded(move || {
            let r = link.read_register(1, Duration::from_secs(600));
            (r, link)
        });
        let Err(Error::Device(why)) = result.0 else { panic!("{:?}", result.0) };
        assert!(why.contains("no end of line"), "{why}");
        assert_eq!(fake.0.lock().unwrap().input.len(), REPLY_BUFFER, "read to the byte");
    }

    #[test]
    fn a_late_reply_is_dropped_before_the_next_request() {
        let fake = Fake::with(&[":r31=900.\r\n"]);
        fake.0.lock().unwrap().input.extend(b":r25=1.\r\n");
        assert_eq!(fake.link().get_phase().unwrap(), 90.0);
        assert_eq!(fake.0.lock().unwrap().purges, 1);
    }

    #[test]
    fn what_follows_the_line_in_one_read_is_gone() {
        let fake = Fake::with(&[":r31=900.\r\n:r25=1.\r\n", ":r25=5.\r\n"]);
        let mut link = fake.link();
        assert_eq!(link.get_phase().unwrap(), 90.0);
        assert_eq!(fake.0.lock().unwrap().input.len(), 0);
        assert_eq!(link.get_amplitude(0).unwrap(), 0.005);
    }

    #[test]
    fn a_line_in_pieces_is_put_together() {
        let fake = Fake::default();
        let mut link = fake.link();
        fake.0.lock().unwrap().late =
            Some((b"60.\r\n".to_vec(), Instant::now() + Duration::from_millis(20)));
        fake.0.lock().unwrap().replies.push_back(Some(b":r00=".to_vec()));
        assert_eq!(link.read_register(0, IDENTIFY_TIMEOUT).unwrap(), b"60");
    }

    #[test]
    fn a_nul_cuts_a_line_and_hides_a_line_feed_behind_it_in_that_read() {
        let fake = Fake::with(&[":r31=9\0x\r\n"]);
        fake.0.lock().unwrap().late =
            Some((b"\n".to_vec(), Instant::now() + Duration::from_millis(20)));
        assert_eq!(fake.link().get_phase().unwrap(), 0.9);
        assert!(fake.0.lock().unwrap().late.is_none(), "it waited for the second line feed");
    }

    #[test]
    fn a_reply_is_checked_as_there() {
        for bad in [
            &b"r00=60."[..],
            b"",
            b" :r01=60.",
            b":",
            b":w01=60.",
            b":b01=60.",
            b":r02=60.",
            b":r01",
            b":r01 60",
            b":r99999999999999999999999=1.",
        ] {
            assert!(check_read(bad, b'r', 1).is_err(), "{}", bad.escape_ascii());
        }
        for (good, value) in [
            (&b":r01=60."[..], &b"60"[..]),
            (b":r01=60", b"60"),
            (b":r01=60. \r", b"60"),
            (b":r1=60.", b"60"),
            (b":r 01 =60.", b"60"),
            (b":r01=.", b""),
        ] {
            assert_eq!(check_read(good, b'r', 1).unwrap(), value, "{}", good.escape_ascii());
        }
        // The type's register is 0, which C takes for no check at all.
        assert_eq!(check_read(b":r07=60.", b'r', 0).unwrap(), b"60");
        assert_eq!(check_read(b":r=60.", b'r', 0).unwrap(), b"60");
        assert!(check_read(b":r60.", b'r', 1).is_err());
    }

    #[test]
    fn a_writes_reply_is_ok_alone() {
        for good in [&b":ok"[..], b":ok.", b":ok\r", b":ok. "] {
            assert!(check_ok(good).is_ok(), "{}", good.escape_ascii());
        }
        for bad in [&b"ok"[..], b":err", b":ok,", b"", b" :ok", b":OK", b":ok.."] {
            assert!(check_ok(bad).is_err(), "{}", bad.escape_ascii());
        }
    }

    #[test]
    fn strtoul_as_c_has_it() {
        assert_eq!(c_strtoul(b"60").unwrap(), (60, &b""[..]));
        assert_eq!(c_strtoul(b"  60 x").unwrap(), (60, &b"x"[..]));
        assert_eq!(c_strtoul(b"abc").unwrap(), (0, &b"abc"[..]));
        assert_eq!(c_strtoul(b" -").unwrap(), (0, &b"-"[..]));
        assert_eq!(c_strtoul(b"").unwrap(), (0, &b""[..]));
        assert_eq!(c_strtoul(b"+7").unwrap(), (7, &b""[..]));
        assert_eq!(c_strtoul(b"-1").unwrap(), (u64::MAX, &b""[..]));
        assert_eq!(c_strtoul(b"18446744073709551615").unwrap().0, u64::MAX);
        assert!(c_strtoul(b"18446744073709551616").is_err());
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn atod_reads_in_single_precision_as_c_has_it() {
        assert_eq!(c_atod(b"").unwrap(), 0.0);
        assert_eq!(c_atod(b"  ").unwrap(), 0.0);
        assert_eq!(c_atod(b" 12.5 ").unwrap(), 12.5);
        assert_eq!(c_atod(b"0.1").unwrap(), f64::from(0.1f32));
        assert_eq!(c_atod(b"1234567891").unwrap(), 1_234_567_936.0);
        assert_eq!(c_atod(b"-500").unwrap(), -500.0);
        assert!(c_atod(b"inf").unwrap().is_infinite());
        assert!(c_atod(b"nan").unwrap().is_nan());
        assert_eq!(c_atod(b"0e99").unwrap(), 0.0);
        for bad in [&b"x"[..], b"1x", b"1,0", b"1e99", b"-1e99", b"1e-99", b"-", b".", b"0x10"] {
            assert!(c_atod(bad).is_err(), "{}", bad.escape_ascii());
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn replies_scale_as_there() {
        assert_eq!(parse_frequency(b"100000,0").unwrap(), 1000.0);
        assert_eq!(parse_frequency(b"100,1").unwrap(), 1.0);
        assert_eq!(parse_frequency(b"100,2").unwrap(), 1.0);
        assert_eq!(parse_frequency(b"100,3").unwrap(), 1000.0);
        assert_eq!(parse_frequency(b"100,4").unwrap(), 1.0 / 1e-6);
        assert_eq!(parse_frequency(b"100,0.").unwrap(), 1.0);
        assert_eq!(parse_frequency(b"100,x").unwrap(), 1.0, "no digits are code 0");
        assert_eq!(parse_frequency(b"100,0,9").unwrap(), 1.0, "more words go unread");
        for bad in [&b""[..], b"100", b"100,5", b"x,0", b"100,99999999999999999999"] {
            assert!(parse_frequency(bad).is_err(), "{}", bad.escape_ascii());
        }
        assert_eq!(parse_volts(b"5000").unwrap(), 5.0);
        assert_eq!(parse_volts(b"").unwrap(), 0.0);
        assert!(parse_volts(b"5000,1").is_err());
        assert_eq!(parse_offset(b"1000").unwrap(), 0.0);
        assert_eq!(parse_offset(b"1023").unwrap(), 10.23 - 10.0);
        assert_eq!(parse_offset(b"2000").unwrap(), 9.99);
        assert_eq!(parse_offset(b"0").unwrap(), -9.99);
        assert!(parse_offset(b"nan").unwrap().is_nan());
        assert_eq!(parse_duty(b"500").unwrap(), 0.5);
        assert_eq!(parse_phase(b"900").unwrap(), 90.0);
        assert_eq!(parse_phase(b"-900").unwrap(), -90.0);
    }

    #[test]
    fn requests_have_the_c_text() {
        assert_eq!(frequency_text(1000.0, 60_000_000), "100000,0");
        assert_eq!(frequency_text(0.0, 60_000_000), "1,0");
        assert_eq!(frequency_text(-5.0, 60_000_000), "1,0");
        assert_eq!(frequency_text(f64::INFINITY, 60_000_000), "6000000000,0");
        assert_eq!(frequency_text(70e6, 15_000_000), "1500000000,0");
        assert_eq!(frequency_text(0.125, 60_000_000), "12,0", "half to even");
        assert_eq!(volts_text(5.0), "5000");
        assert_eq!(volts_text(21.0), "20000");
        assert_eq!(volts_text(-1.0), "0");
        assert_eq!(offset_text(0.0), "1000");
        assert_eq!(offset_text(10.0), "1999");
        assert_eq!(offset_text(-10.0), "1");
        assert_eq!(duty_text(0.5), "500");
        assert_eq!(duty_text(2.0), "1000");
        assert_eq!(duty_text(-1.0), "0");
        assert_eq!(phase_text(90.0), "900");
        assert_eq!(phase_text(450.5), "905");
        assert_eq!(phase_text(-90.0), "-900");
        assert_eq!(phase_text(360.0), "0");
    }

    #[test]
    fn a_negative_zero_goes_out_as_zero() {
        assert_eq!(volts_text(-0.0), "0");
        assert_eq!(duty_text(-0.0), "0");
        assert_eq!(phase_text(-0.0), "0");
        assert_eq!(phase_text(-360.0), "0");
        assert_eq!(phase_text(-0.01), "0");
    }

    #[test]
    fn a_half_rounds_to_even_as_printf() {
        assert_eq!(whole(0.5), "0");
        assert_eq!(whole(1.5), "2");
        assert_eq!(whole(2.5), "2");
    }

    #[test]
    fn waveforms_map_both_ways() {
        let names = wave_names();
        assert_eq!(names.len(), 77);
        assert_eq!(names[0], "sine");
        assert_eq!(names[16], "lorenz");
        assert_eq!(names[17], "arb-01");
        assert_eq!(names[76], "arb-60");
        assert_eq!(wave_code(16), 16);
        assert_eq!(wave_code(17), 101);
        assert_eq!(wave_code(76), 160);
        for index in 0..names.len() {
            assert_eq!(wave_index(wave_code(index) as u64), Some(index));
        }
        for code in [17, 100, 161, u64::MAX] {
            assert_eq!(wave_index(code), None, "{code}");
        }
    }

    #[test]
    fn enables_follow_the_c_int() {
        let mut link = Fake::with(&[":r20=2,4294967296.\r\n"]).link();
        link.get_enabled().unwrap();
        assert_eq!(link.enabled, [true, false]);
        let mut link = Fake::with(&[":r20=1.\r\n"]).link();
        assert!(link.get_enabled().is_err());
        assert_eq!(link.enabled, [true, false], "the first is kept");
    }

    #[test]
    fn a_write_the_device_did_not_take_leaves_the_enables() {
        let fake = Fake::with(&[":err\r\n", ":ok\r\n"]);
        let mut link = fake.link();
        assert!(link.set_enabled([true, true]).is_err());
        assert_eq!(link.enabled, [false, false]);
        link.set_enabled([true, false]).unwrap();
        assert_eq!(link.enabled, [true, false]);
        assert_eq!(fake.writes(), [":w20=1,1.\r\n", ":w20=1,0.\r\n"]);
    }

    #[test]
    fn a_closed_link_says_so() {
        assert!(Link::new(None).get_phase().is_err());
    }
}
