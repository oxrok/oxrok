// Counterpart in sigrok:
//   libsigrok/src/scpi/scpi.c,
//   libsigrok/src/scpi.h,
//   Copyright (C) 2013 poljar (Damir Jelic) <poljarinho@gmail.com>,
//   Copyright (C) 2013 Martin Ling <martin-sigrok@earth.li>,
//   with commits by Bert Vermeulen, Uwe Hermann, Aurelien Jacobs,
//   Gerhard Sittig, Soeren Apel, Daniel Elstner, Frank Stettner,
//   Alexandru Gagniuc, Marc Schink, Lars-Peter Clausen, Guido Trentalancia,
//   Stefan Mandl, Sylvain Munaut, Timo Kokkonen, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! SCPI over USBTMC, TCP and serial ports.
//!
//! A driver holds a [`Scpi`] and asks it questions: [`Scpi::get_string`],
//! [`Scpi::get_f32`], [`Scpi::get_block`] and so on. Below that sits a
//! [`ScpiTransport`], one per wire protocol, chosen from the `conn=`
//! resource string as sigrok spells it:
//!
//! - `usbtmc` or `usbtmc/BUS.ADDR`: a USB test and measurement class
//!   device, the common case for bench instruments;
//! - `tcp-raw/HOST/PORT`: SCPI over a plain TCP socket;
//! - `tcp-rigol/HOST/PORT`: Rigol's variant with a length prefix on
//!   every response chunk;
//! - anything else: a serial port, with `serialcomm=` for the line
//!   parameters.
//!
//! With `OXROK_SCPI_RECORD=FILE` set, every command and response goes
//! to a text fixture that [`record::Replay`] plays back in tests.

#![forbid(unsafe_code)]

pub mod record;
pub mod serial;
pub mod tcp;
pub mod usbtmc;

use crate::core::{Error, Result};
use std::time::{Duration, Instant};

/// A wire protocol that carries SCPI messages.
pub trait ScpiTransport: Send {
    /// The resource string this transport was opened with.
    fn resource(&self) -> &str;
    /// Sends one complete message, newline included.
    fn send(&mut self, data: &[u8]) -> Result<()>;
    /// Starts reading a response.
    fn read_begin(&mut self) -> Result<()>;
    /// Reads part of the response, waiting at most `timeout` for data.
    /// Returns 0 when nothing arrived in time.
    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize>;
    /// True once the whole response has been read.
    fn read_complete(&self) -> bool;
    /// Closes the connection, keeping what [`ScpiTransport::reopen`] needs, as
    /// `sr_scpi_close` leaves an instance to open again. A transport with no
    /// connection of its own, such as a replay, has nothing to do.
    fn close(&mut self) -> Result<()> {
        Ok(())
    }
    /// Opens again what [`ScpiTransport::close`] closed.
    fn reopen(&mut self) -> Result<()> {
        Ok(())
    }
    /// Drops what has arrived and nobody asked for, where the transport has a
    /// queue of that. Nothing on the wire.
    fn purge(&mut self) -> Result<()> {
        Ok(())
    }
    /// Whether an answer says which question it answers, as USBTMC's `bTag` does.
    /// On a line that doesn't, asking again after a timeout would owe one answer
    /// more, and each later reading would be the answer to the one before.
    fn tags_answers(&self) -> bool {
        false
    }
    /// Reads what arrives within `timeout` and drops it, leaving no step in a
    /// recording, for a transport where a read is only a read and sends nothing.
    /// How much came, or `None` where a transport has no such read: USBTMC asks
    /// the device for each message, so a look at the line is a request.
    fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
        let _ = timeout;
        Ok(None)
    }
    /// A device clear: USBTMC's `INITIATE_CLEAR`. False where there's none.
    fn clear(&mut self) -> Result<bool> {
        Ok(false)
    }
    /// After a read's timeout, whether the next read waits for the answer to the
    /// request already made (on), or makes a new request (off, as libsigrok does,
    /// and the default). Only USBTMC has requests.
    fn wait_out_requests(&mut self, on: bool) {
        let _ = on;
    }
}

/// The identification an instrument gives for `*IDN?`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct HwInfo {
    /// Manufacturer, as written by the instrument.
    pub manufacturer: String,
    /// Model name.
    pub model: String,
    /// Serial number, `Unknown` when the answer has three fields.
    pub serial_number: String,
    /// Firmware version.
    pub firmware_version: String,
}

/// An instrument on a SCPI transport.
pub struct Scpi {
    transport: Box<dyn ScpiTransport>,
    /// How long one response may take to start, and how long a pause
    /// inside one may last.
    pub read_timeout: Duration,
    /// How long `*OPC?` may go unanswered before the instrument counts as gone: a
    /// choice, see [`Scpi::get_opc`].
    pub opc_silence: Duration,
    /// A read gave up, so an answer may be on its way to no question.
    stale: bool,
    /// What the last channel selection named: `actual_channel_name`.
    channel: Option<String>,
}

/// How many answers `*OPC?` gets.
const OPC_TRIES: u32 = 100;
/// How long `*OPC?` may go unanswered: a choice, see [`Scpi::get_opc`].
const OPC_SILENCE: Duration = Duration::from_secs(30);
/// Of a read's timeout, how long a stale line may be drained.
const SETTLE_TURNS: u32 = 5;

/// Bytes read per call while collecting a response.
const CHUNK: usize = 4096;

impl Scpi {
    /// Wraps an open transport.
    pub fn new(transport: Box<dyn ScpiTransport>) -> Scpi {
        Scpi {
            transport,
            read_timeout: Duration::from_secs(1),
            opc_silence: OPC_SILENCE,
            stale: false,
            channel: None,
        }
    }

    /// The resource string.
    pub fn resource(&self) -> &str {
        self.transport.resource()
    }

    /// Drops what has arrived unasked; see [`ScpiTransport::purge`].
    pub fn purge(&mut self) -> Result<()> {
        self.transport.purge()
    }

    /// Closes the connection; see [`ScpiTransport::close`]. libsigrok closes an
    /// instrument after each probe, and a driver's open opens it again, which is
    /// on the wire: USBTMC asks for the capabilities again, TCP connects again.
    pub fn close(&mut self) -> Result<()> {
        self.transport.close()
    }

    /// Opens again what [`Scpi::close`] closed.
    pub fn reopen(&mut self) -> Result<()> {
        self.transport.reopen()
    }

    /// Reads and drops what arrives; see [`ScpiTransport::discard`].
    pub fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
        self.transport.discard(timeout)
    }

    /// Sends `command`, adding the newline when it lacks one.
    pub fn send(&mut self, command: &str) -> Result<()> {
        log::debug!("SCPI send: {}", command.trim_end());
        if self.stale {
            self.settle();
        }
        let mut data = command.as_bytes().to_vec();
        if data.last() != Some(&b'\n') {
            data.push(b'\n');
        }
        self.transport.send(&data)
    }

    /// A read gave up, so what the line still owes must not pass for the answer to
    /// what comes next: it drains until the line has been quiet for a read's timeout,
    /// for [`SETTLE_TURNS`] of those at most. An answer later still is beyond telling
    /// on a line with no tags, and shifts what follows.
    fn settle(&mut self) {
        self.stale = false;
        let quiet = self.read_timeout;
        let began = Instant::now();
        loop {
            match self.transport.discard(quiet) {
                Ok(Some(0)) | Err(_) => return,
                Ok(None) => {
                    let _ = self.transport.purge();
                    return;
                }
                Ok(Some(_)) => {
                    if began.elapsed() >= quiet * SETTLE_TURNS {
                        log::warn!("The instrument is still sending what nobody asked for.");
                        return;
                    }
                }
            }
        }
    }

    /// Starts reading a response; see [`Scpi::read_data`].
    pub fn read_begin(&mut self) -> Result<()> {
        self.transport.read_begin()
    }

    /// Reads part of a response started with [`Scpi::read_begin`], for
    /// responses too large to collect at once. Returns 0 on a timeout.
    pub fn read_data(&mut self, buf: &mut [u8]) -> Result<usize> {
        let timeout = self.read_timeout;
        let read = self.transport.read_data(buf, timeout);
        // The driver may give up on what is still to come, as a get does.
        if !matches!(read, Ok(n) if n > 0) {
            self.stale = true;
        }
        read
    }

    /// True once the response started with [`Scpi::read_begin`] is
    /// complete.
    pub fn read_complete(&self) -> bool {
        self.transport.read_complete()
    }

    /// The question a read gave up on has its answer after all, read by asking again
    /// with [`Scpi::get_data`] and no command: nothing is owed, so the next
    /// [`Scpi::send`] needn't drain the line.
    pub fn answered(&mut self) {
        self.stale = false;
    }

    /// One look for the answer to a question already sent, for a driver that keeps
    /// its own count of what is out and finds an answer's end itself: what came in a
    /// read's timeout, `None` for nothing, and what came of an answer cut off by the
    /// timeout too, which the next look goes on from. Nothing counts as stale.
    pub fn poll_answer(&mut self) -> Result<Option<Vec<u8>>> {
        self.answered();
        match self.transport.read_begin() {
            Ok(()) => {}
            Err(Error::Timeout) => return Ok(None),
            Err(e) => return Err(e),
        }
        let mut out = Vec::new();
        let mut deadline = Instant::now() + self.read_timeout;
        let mut chunk = [0u8; CHUNK];
        while !self.transport.read_complete() {
            let left = deadline.saturating_duration_since(Instant::now());
            let n = match self.transport.read_data(&mut chunk, left.max(Duration::from_millis(1))) {
                Ok(n) => n,
                Err(Error::Timeout) => 0,
                Err(e) => return Err(e),
            };
            if n > 0 {
                out.extend_from_slice(&chunk[..n]);
                deadline = Instant::now() + self.read_timeout;
            } else if Instant::now() >= deadline {
                break;
            }
        }
        Ok((!out.is_empty()).then_some(out))
    }

    /// Whether answers say which request they answer, as USBTMC's do.
    pub fn tags_answers(&self) -> bool {
        self.transport.tags_answers()
    }

    /// A device clear, where the transport has one: what the instrument holds of
    /// input and output goes, answers to questions asked before it among them. False
    /// where there's no such thing, as on a socket or a serial port.
    pub fn clear(&mut self) -> Result<bool> {
        self.stale = false;
        self.transport.clear()
    }

    /// Asks the transport to wait out a request already made rather than make
    /// another after a timeout, where the two differ (USBTMC): see
    /// [`ScpiTransport::wait_out_requests`].
    pub fn wait_out_requests(&mut self, on: bool) {
        self.transport.wait_out_requests(on);
    }

    /// A question went out and nobody will read its answer: the next [`Scpi::send`]
    /// waits for the line to go quiet first, as after a timeout.
    pub fn forget_question(&mut self) {
        self.stale = true;
    }

    /// Sends `command` if given and collects the whole response.
    pub fn get_data(&mut self, command: Option<&str>) -> Result<Vec<u8>> {
        if let Some(c) = command {
            self.send(c)?;
        }
        self.transport.read_begin()?;
        let mut out = Vec::new();
        let mut deadline = Instant::now() + self.read_timeout;
        let mut chunk = [0u8; CHUNK];
        while !self.transport.read_complete() {
            let left = deadline.saturating_duration_since(Instant::now());
            let n = match self.transport.read_data(&mut chunk, left.max(Duration::from_millis(1))) {
                Ok(n) => n,
                Err(Error::Timeout) => 0,
                Err(e) => {
                    self.stale = true;
                    return Err(e);
                }
            };
            if n > 0 {
                out.extend_from_slice(&chunk[..n]);
                deadline = Instant::now() + self.read_timeout;
            } else if Instant::now() >= deadline {
                log::debug!("Timed out waiting for the SCPI response.");
                self.stale = true;
                return Err(Error::Timeout);
            }
        }
        Ok(out)
    }

    /// The response to `command` as text, without the line ending.
    pub fn get_string(&mut self, command: &str) -> Result<String> {
        let mut data = self.get_data(Some(command))?;
        if data.last() == Some(&b'\n') {
            data.pop();
        }
        if data.last() == Some(&b'\r') {
            data.pop();
        }
        let text = String::from_utf8_lossy(&data).into_owned();
        log::debug!("SCPI response: {}", text.chars().take(70).collect::<String>());
        Ok(text)
    }

    /// The response to `command` as a boolean: `1`/`0`, `ON`/`OFF`,
    /// `TRUE`/`FALSE`, `YES`/`NO` in any case.
    pub fn get_bool(&mut self, command: &str) -> Result<bool> {
        let s = self.get_string(command)?;
        parse_bool(&s).ok_or_else(|| Error::device(format!("SCPI: not a boolean: '{s}'")))
    }

    /// The response to `command` as an integer; a decimal with a
    /// fraction is rejected.
    pub fn get_int(&mut self, command: &str) -> Result<i64> {
        let s = self.get_string(command)?;
        parse_int(&s).ok_or_else(|| Error::device(format!("SCPI: not an integer: '{s}'")))
    }

    /// The response to `command` as a float.
    pub fn get_f32(&mut self, command: &str) -> Result<f32> {
        self.get_f64(command).map(|v| v as f32)
    }

    /// The response to `command` as a double.
    pub fn get_f64(&mut self, command: &str) -> Result<f64> {
        let s = self.get_string(command)?;
        s.trim().parse().map_err(|_| Error::device(format!("SCPI: not a number: '{s}'")))
    }

    /// Waits for `*OPC?` to answer true: [`Scpi::get_opc_while`] with no stop.
    pub fn get_opc(&mut self) -> Result<()> {
        self.get_opc_while(|| true)
    }

    /// Waits for `*OPC?` to answer true, up to a hundred answers 10 ms apart, as
    /// `sr_scpi_get_opc`.
    ///
    /// Silence is where this differs from C, by choice. C asks a hundred times
    /// through timeouts, so about 101 s of a dead instrument; here [`Scpi::opc_silence`]
    /// of silence, 30 s, ends it. And where answers have no tags it asks once and
    /// keeps reading, for a late answer to that question and no other (over TCP C's
    /// read blocks, which comes to the same). An error that is no timeout, such as a
    /// closed connection, ends it at once, where C goes on asking. Ok when `go_on` says
    /// no, for a capture's stop: its caller looks at the same flag.
    pub fn get_opc_while(&mut self, go_on: impl Fn() -> bool) -> Result<()> {
        let mut answers = 0;
        let mut heard = Instant::now();
        let mut asked = false;
        while go_on() {
            if !asked {
                self.send("*OPC?")?;
            }
            match self.get_data(None) {
                Ok(data) => {
                    // The one question out has its answer.
                    self.stale = false;
                    let text = String::from_utf8_lossy(&data);
                    if parse_bool(text.trim_end_matches(['\r', '\n'])) == Some(true) {
                        return Ok(());
                    }
                    answers += 1;
                    if answers >= OPC_TRIES {
                        return Err(Error::device("SCPI: *OPC? never answered 1"));
                    }
                    heard = Instant::now();
                    asked = false;
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(Error::Timeout) => {
                    if heard.elapsed() >= self.opc_silence {
                        return Err(Error::device(format!(
                            "SCPI: no answer to *OPC? in {} s",
                            self.opc_silence.as_secs_f32()
                        )));
                    }
                    asked = !self.transport.tags_answers();
                }
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// The response to `command`, comma separated floats. Err on one that is no
    /// number, where `sr_scpi_get_floatv` returns an error with those before it.
    pub fn get_floatv(&mut self, command: &str) -> Result<Vec<f32>> {
        let s = self.get_string(command)?;
        s.split(',')
            .map(|t| {
                t.trim()
                    .parse()
                    .map_err(|_| Error::device(format!("SCPI: not a list of numbers: '{s}'")))
            })
            .collect()
    }

    /// Selects a channel if it isn't the one selected, as `sr_scpi_cmd` does: the
    /// name is remembered before the send, and a close forgets nothing.
    fn select_channel<T: PartialEq>(
        &mut self,
        table: &[(T, &str)],
        channel: Option<(T, &str)>,
    ) -> Result<()> {
        let Some((command, name)) = channel else { return Ok(()) };
        let Some(template) = cmd_get(table, &command) else { return Ok(()) };
        if self.channel.as_deref() == Some(name) {
            return Ok(());
        }
        self.channel = Some(name.to_owned());
        self.send(&render(template, &[CmdArg::Str(name)]))
    }

    /// `sr_scpi_cmd`: `command` from `table` with `args`, after selecting `channel`
    /// (its command and its name). A command the table lacks is fine, and sends
    /// nothing, not even the selection.
    pub fn cmd<T: PartialEq>(
        &mut self,
        table: &[(T, &str)],
        channel: Option<(T, &str)>,
        command: &T,
        args: &[CmdArg<'_>],
    ) -> Result<()> {
        let Some(template) = cmd_get(table, command) else { return Ok(()) };
        self.select_channel(table, channel)?;
        self.send(&render(template, args))
    }

    /// `sr_scpi_cmd_resp`: as [`Scpi::cmd`], and the answer as text, for
    /// [`parse_bool`] or a number. A command the table lacks is
    /// [`Error::NotSupported`].
    pub fn cmd_resp<T: PartialEq>(
        &mut self,
        table: &[(T, &str)],
        channel: Option<(T, &str)>,
        command: &T,
        args: &[CmdArg<'_>],
    ) -> Result<String> {
        let Some(template) = cmd_get(table, command) else {
            return Err(Error::not_supported("the device has no such command"));
        };
        self.select_channel(table, channel)?;
        self.get_string(&render(template, args))
    }

    /// The response to `command` as an IEEE 488.2 definite length
    /// block: `#`, one digit with the length of the length, the length,
    /// then that many bytes.
    pub fn get_block(&mut self, command: Option<&str>) -> Result<Vec<u8>> {
        if let Some(c) = command {
            self.send(c)?;
        }
        self.transport.read_begin()?;
        let mut buf: Vec<u8> = Vec::new();
        let mut chunk = [0u8; CHUNK];
        let mut deadline = Instant::now() + self.read_timeout;
        let mut need = 2usize;
        let mut data_len = None;
        let mut header_len = 0usize;
        loop {
            if buf.len() >= need {
                if data_len.is_none() {
                    if buf[0] != b'#' {
                        return Err(Error::device("SCPI: block response does not start with #"));
                    }
                    let digits = usize::from(buf[1].checked_sub(b'0').unwrap_or(99));
                    if digits == 0 || digits > 9 {
                        return Err(Error::not_supported("SCPI: indefinite length block"));
                    }
                    if buf.len() < 2 + digits {
                        need = 2 + digits;
                        continue;
                    }
                    let len: usize = std::str::from_utf8(&buf[2..2 + digits])
                        .ok()
                        .and_then(|s| s.parse().ok())
                        .ok_or_else(|| Error::device("SCPI: bad block length"))?;
                    header_len = 2 + digits;
                    data_len = Some(len);
                    need = header_len + len;
                }
                if buf.len() >= need {
                    buf.drain(..header_len);
                    buf.truncate(need - header_len);
                    return Ok(buf);
                }
            }
            if self.transport.read_complete() && data_len.is_some() {
                // The instrument sent less than it announced.
                self.stale = true;
                buf.drain(..header_len);
                return Ok(buf);
            }
            let left = deadline.saturating_duration_since(Instant::now());
            let n = match self.transport.read_data(&mut chunk, left.max(Duration::from_millis(1))) {
                Ok(n) => n,
                Err(Error::Timeout) => 0,
                Err(e) => {
                    self.stale = true;
                    return Err(e);
                }
            };
            if n > 0 {
                buf.extend_from_slice(&chunk[..n]);
                deadline = Instant::now() + self.read_timeout;
            } else if Instant::now() >= deadline {
                // Cut short, or nothing: the rest may yet come.
                self.stale = true;
                if data_len.is_some() {
                    buf.drain(..header_len);
                    return Ok(buf);
                }
                return Err(Error::Timeout);
            }
        }
    }

    /// `*IDN?`, split into its fields.
    pub fn hw_id(&mut self) -> Result<HwInfo> {
        let response = self.get_string("*IDN?")?;
        let fields: Vec<&str> = response.split(',').collect();
        if fields.len() < 3 {
            return Err(Error::device(format!("SCPI: short *IDN? response '{response}'")));
        }
        let manufacturer = fields[0].find("IDN ").map_or(fields[0], |i| &fields[0][i + 4..]);
        let (serial, firmware) = if fields.len() < 4 {
            log::warn!("Short IDN response, assume missing serial number.");
            ("Unknown", fields[2])
        } else {
            (fields[2], fields[3])
        };
        Ok(HwInfo {
            manufacturer: manufacturer.trim().to_owned(),
            model: fields[1].trim().to_owned(),
            serial_number: serial.trim().to_owned(),
            firmware_version: firmware.trim().to_owned(),
        })
    }
}

/// As `parse_strict_bool`: `1` and `0` whole, the words by how they start, so
/// that `T'D` is true and `NONE` false.
pub fn parse_bool(s: &str) -> Option<bool> {
    let s = s.trim().to_ascii_lowercase();
    if s == "1" || s.starts_with(['y', 't']) || s.starts_with("on") {
        Some(true)
    } else if s == "0" || s.starts_with(['n', 'f']) || s.starts_with("off") {
        Some(false)
    } else {
        None
    }
}

fn parse_int(s: &str) -> Option<i64> {
    let s = s.trim();
    if let Ok(v) = s.parse::<i64>() {
        return Some(v);
    }
    // Instruments answer integers as floats at times: 1.2E+03.
    let f: f64 = s.parse().ok()?;
    if f.fract() == 0.0 && f.abs() < 9.0e15 { Some(f as i64) } else { None }
}

/// A value for a command table's template.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CmdArg<'a> {
    /// For `%s`.
    Str(&'a str),
    /// For `%f` and `%.Nf`.
    Float(f64),
    /// For `%d`, `%i` and `%u`.
    Int(i64),
}

/// The template of `command` in a driver's table: `sr_scpi_cmd_get`.
pub fn cmd_get<'t, T: PartialEq>(table: &'t [(T, &str)], command: &T) -> Option<&'t str> {
    table.iter().find(|(c, _)| c == command).map(|(_, text)| *text)
}

/// A command table's template with its values, for the conversions those tables
/// use: `%s`, `%d`, `%i`, `%u`, `%f`, `%.Nf`, `%%`. One with no value, or of another
/// kind, stays as it stands, so that it shows in a `.tx`.
pub fn render(template: &str, args: &[CmdArg<'_>]) -> String {
    use std::fmt::Write as _;
    let mut out = String::with_capacity(template.len() + 16);
    let mut args = args.iter();
    let mut rest = template;
    while let Some(at) = rest.find('%') {
        out.push_str(&rest[..at]);
        rest = &rest[at + 1..];
        let end = rest.find(|c: char| !(c.is_ascii_digit() || c == '.')).unwrap_or(rest.len());
        let (spec, tail) = rest.split_at(end);
        let Some(kind) = tail.chars().next() else {
            let _ = write!(out, "%{spec}");
            rest = tail;
            break;
        };
        rest = &tail[kind.len_utf8()..];
        if kind == '%' {
            out.push('%');
            continue;
        }
        let places = spec.strip_prefix('.').map_or(Some(6), |p| p.parse().ok());
        match (kind, args.next(), places) {
            ('s', Some(CmdArg::Str(text)), _) => out.push_str(text),
            ('d' | 'i' | 'u', Some(CmdArg::Int(v)), _) => {
                let _ = write!(out, "{v}");
            }
            ('f', Some(CmdArg::Float(v)), Some(places)) => {
                let _ = write!(out, "{v:.places$}");
            }
            _ => {
                let _ = write!(out, "%{spec}{kind}");
            }
        }
    }
    out.push_str(rest);
    out
}

/// Takes the quotes off a SCPI string, single or double, and undoes the doubling of
/// them inside: `sr_scpi_unquote_string`. What isn't quoted comes back as it is.
pub fn unquote_string(s: &str) -> String {
    let bytes = s.as_bytes();
    if bytes.len() < 2 || !matches!(bytes[0], b'"' | b'\'') || bytes[0] != bytes[bytes.len() - 1] {
        return s.to_owned();
    }
    let quote = &s[..1];
    s[1..s.len() - 1].replace(&quote.repeat(2), quote)
}

/// The short vendor name sigrok uses for a manufacturer string.
pub fn vendor_alias(raw: &str) -> &str {
    const ALIASES: [(&str, &str); 8] = [
        ("Agilent Technologies", "Agilent"),
        ("CHROMA", "Chroma"),
        ("Chroma ATE", "Chroma"),
        ("HEWLETT-PACKARD", "HP"),
        ("Keysight Technologies", "Keysight"),
        ("PHILIPS", "Philips"),
        ("RIGOL TECHNOLOGIES", "Rigol"),
        ("Siglent Technologies", "Siglent"),
    ];
    ALIASES.iter().find(|(long, _)| long.eq_ignore_ascii_case(raw)).map_or(raw, |(_, short)| short)
}

/// Where an instrument is, from a `conn=` string.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Resource {
    /// A USBTMC device; `None` means any.
    Usbtmc(Option<String>),
    /// SCPI over a TCP socket.
    TcpRaw(String, u16),
    /// Rigol's length prefixed TCP protocol.
    TcpRigol(String, u16),
    /// A serial port path.
    Serial(String),
}

impl Resource {
    /// Parses a `conn=` string.
    pub fn parse(conn: &str) -> Result<Resource> {
        let bad = || Error::Arg(format!("Invalid SCPI resource '{conn}'."));
        if conn == "usbtmc" {
            return Ok(Resource::Usbtmc(None));
        }
        if let Some(rest) = conn.strip_prefix("usbtmc/") {
            return Ok(Resource::Usbtmc(Some(rest.to_owned())));
        }
        for (prefix, rigol) in [("tcp-raw/", false), ("tcp-rigol/", true)] {
            if let Some(rest) = conn.strip_prefix(prefix) {
                let (host, port) = rest.split_once('/').ok_or_else(bad)?;
                let port: u16 = port.parse().map_err(|_| bad())?;
                return Ok(if rigol {
                    Resource::TcpRigol(host.to_owned(), port)
                } else {
                    Resource::TcpRaw(host.to_owned(), port)
                });
            }
        }
        if conn.starts_with("vxi/") || conn.starts_with("visa/") || conn.starts_with("libgpib/") {
            return Err(Error::not_supported(format!("SCPI transport of '{conn}' not available")));
        }
        if conn.is_empty() {
            return Err(bad());
        }
        Ok(Resource::Serial(conn.to_owned()))
    }

    /// The resource as a `conn=` string.
    pub fn to_conn(&self) -> String {
        match self {
            Resource::Usbtmc(None) => "usbtmc".to_owned(),
            Resource::Usbtmc(Some(a)) => format!("usbtmc/{a}"),
            Resource::TcpRaw(h, p) => format!("tcp-raw/{h}/{p}"),
            Resource::TcpRigol(h, p) => format!("tcp-rigol/{h}/{p}"),
            Resource::Serial(p) => p.clone(),
        }
    }
}

/// Opens the transport for `resource`; `serialcomm` is the serial
/// line parameters, with a default of `115200/8n1`. Recording applies
/// when `OXROK_SCPI_RECORD` is set.
pub fn open(resource: &Resource, serialcomm: Option<&str>) -> Result<Scpi> {
    let transport: Box<dyn ScpiTransport> = match resource {
        Resource::Usbtmc(None) => {
            let devices = usbtmc::list()?;
            let first = devices.first().ok_or_else(|| Error::device("No USBTMC device found."))?;
            Box::new(usbtmc::Usbtmc::open(first)?)
        }
        Resource::Usbtmc(Some(addr)) => {
            let devices = usbtmc::list()?;
            let dev = devices
                .iter()
                .find(|d| d.matches_conn(addr))
                .ok_or_else(|| Error::device(format!("No USBTMC device at '{addr}'.")))?;
            Box::new(usbtmc::Usbtmc::open(dev)?)
        }
        Resource::TcpRaw(host, port) => Box::new(tcp::Tcp::connect(host, *port, false)?),
        Resource::TcpRigol(host, port) => Box::new(tcp::Tcp::connect(host, *port, true)?),
        Resource::Serial(path) => {
            let params = crate::serial::Params::parse(serialcomm.unwrap_or("115200/8n1"))?;
            Box::new(serial::Serial::open(path, params)?)
        }
    };
    Ok(Scpi::new(recording(transport)))
}

/// `transport`, recorded when `OXROK_SCPI_RECORD` is set.
fn recording(transport: Box<dyn ScpiTransport>) -> Box<dyn ScpiTransport> {
    match std::env::var("OXROK_SCPI_RECORD") {
        Ok(file) if !file.is_empty() => {
            log::info!("Recording the SCPI session to {file}.");
            Box::new(record::Recorder::to_file(transport, file))
        }
        _ => transport,
    }
}

/// [`open`], but with no `serialcomm` a serial port keeps the line it has, as
/// `scpi_serial_open` does when it has none, where [`open`] has a default. It does
/// not look in `scpi_serial.c`'s table of USB ids: see
/// [`crate::serial::open_keeping_line`], also for where it has no line to keep.
pub fn open_keeping_line(resource: &Resource, serialcomm: Option<&str>) -> Result<Scpi> {
    match (resource, serialcomm) {
        (Resource::Serial(path), None) => {
            Ok(Scpi::new(recording(Box::new(serial::Serial::open_keeping_line(path)?))))
        }
        _ => open(resource, serialcomm),
    }
}

/// Finds instruments the way libsigrok's `sr_scpi_scan` does: with no
/// `conn`, every USBTMC device is probed; with `conn`, that resource
/// is. `probe` gets an open instrument and returns a device when it
/// recognises it.
pub fn scan<T>(
    conn: Option<&str>,
    serialcomm: Option<&str>,
    mut probe: impl FnMut(Scpi) -> Result<Option<T>>,
) -> Result<Vec<T>> {
    let mut found = Vec::new();
    let resource = conn.map(Resource::parse).transpose()?;
    match resource {
        None | Some(Resource::Usbtmc(None)) => {
            for dev in usbtmc::list()? {
                let res = Resource::Usbtmc(Some(dev.conn()));
                match open_keeping_line(&res, serialcomm) {
                    Ok(scpi) => match probe(scpi) {
                        Ok(Some(d)) => found.push(d),
                        Ok(None) => {}
                        Err(e) => log::info!("Probe of {} failed: {e}", res.to_conn()),
                    },
                    Err(e) => log::warn!("Cannot open {}: {e}.", res.to_conn()),
                }
            }
        }
        Some(res) => match open_keeping_line(&res, serialcomm) {
            Ok(scpi) => match probe(scpi) {
                Ok(Some(d)) => found.push(d),
                Ok(None) => {}
                Err(e) => log::info!("Probe of {} failed: {e}", res.to_conn()),
            },
            Err(e) => log::warn!("Couldn't open SCPI device {}: {e}.", res.to_conn()),
        },
    }
    Ok(found)
}

#[cfg(test)]
mod tests {
    use super::*;
    use record::Replay;
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    #[test]
    fn a_poll_leaves_nothing_stale_and_a_forgotten_question_settles() {
        let (mut scpi, line) =
            paced(|q| if q.ends_with('?') { vec![(T + 1, b"1\n".to_vec())] } else { Vec::new() });
        scpi.send("Q?").unwrap();
        assert_eq!(scpi.poll_answer().unwrap(), None);
        scpi.send("SET 1").unwrap();
        assert_eq!(line.lock().unwrap().discards, 0, "the answer stays for the question");
        assert_eq!(scpi.poll_answer().unwrap().as_deref(), Some(&b"1\n"[..]));
        scpi.send("Q?").unwrap();
        scpi.forget_question();
        scpi.send("SET 2").unwrap();
        assert!(line.lock().unwrap().discards > 0, "the line goes quiet first");
    }

    /// One read's timeout on the [`Paced`] line's own clock, in its ticks.
    const T: u64 = 5;

    /// What a [`Paced`] line answers to a question: pieces, each with how long after
    /// the question or the piece before it comes.
    type Script = fn(&str) -> Vec<(u64, Vec<u8>)>;

    /// An instrument on a line with no tags, at its own pace, on a clock of its own:
    /// a read that finds nothing moves the clock by its timeout, so a loaded machine
    /// slows the fake and the host alike. Answers wait in line behind each other.
    struct Paced {
        script: Script,
        tags: bool,
        closed: bool,
        line: Arc<Mutex<PacedLine>>,
        done: bool,
    }

    #[derive(Default)]
    struct PacedLine {
        sent: Vec<String>,
        now: u64,
        /// The discards that were asked for.
        discards: u32,
        /// When each piece is due, what it is, and whether it ends an answer.
        coming: VecDeque<(u64, Vec<u8>, bool)>,
    }

    impl Paced {
        /// The next piece, within `timeout`, else `None` with the time gone.
        fn next(&self, timeout: Duration) -> Option<(Vec<u8>, bool)> {
            let wait = (timeout.as_millis() as u64).max(1);
            let mut line = self.line.lock().unwrap();
            if let Some(due) = line.coming.front().map(|c| c.0)
                && due <= line.now + wait
            {
                line.now = line.now.max(due);
                let (_, bytes, last) = line.coming.pop_front().unwrap();
                return Some((bytes, last));
            }
            line.now += wait;
            drop(line);
            std::thread::sleep(timeout);
            None
        }
    }

    impl ScpiTransport for Paced {
        fn resource(&self) -> &'static str {
            "paced"
        }
        fn send(&mut self, data: &[u8]) -> Result<()> {
            let text = String::from_utf8_lossy(data).trim_end().to_owned();
            let mut line = self.line.lock().unwrap();
            let mut at = line.now;
            let pieces = (self.script)(&text);
            let n = pieces.len();
            for (i, (delay, bytes)) in pieces.into_iter().enumerate() {
                at += delay;
                line.coming.push_back((at, bytes, i + 1 == n));
            }
            line.sent.push(text);
            Ok(())
        }
        fn read_begin(&mut self) -> Result<()> {
            self.done = false;
            Ok(())
        }
        fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            if self.closed {
                return Err(Error::device("the line is gone"));
            }
            let Some((bytes, last)) = self.next(timeout) else { return Ok(0) };
            buf[..bytes.len()].copy_from_slice(&bytes);
            self.done = last;
            Ok(bytes.len())
        }
        fn read_complete(&self) -> bool {
            self.done
        }
        fn tags_answers(&self) -> bool {
            self.tags
        }
        fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
            if self.tags {
                return Ok(None);
            }
            self.line.lock().unwrap().discards += 1;
            let got = self.next(timeout).map_or(0, |(bytes, _)| bytes.len());
            if got > 0 {
                // Well past what a line that really ended would take, so that the
                // bound on a drain is reached in real time.
                std::thread::sleep(Duration::from_millis(1));
            }
            Ok(Some(got))
        }
    }

    /// `f`, failing at a bound where it would hang: a mutant must fail.
    fn within<R: Send + 'static>(f: impl FnOnce() -> R + Send + 'static) -> R {
        let (done, result) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let _ = done.send(f());
        });
        result.recv_timeout(Duration::from_secs(60)).expect("it should have ended by now")
    }

    fn paced(script: Script) -> (Scpi, Arc<Mutex<PacedLine>>) {
        paced_as(script, false, false)
    }

    fn paced_as(script: Script, tags: bool, closed: bool) -> (Scpi, Arc<Mutex<PacedLine>>) {
        let line = Arc::new(Mutex::new(PacedLine::default()));
        let fake = Paced { script, tags, closed, line: Arc::clone(&line), done: false };
        let mut scpi = Scpi::new(Box::new(fake));
        scpi.read_timeout = Duration::from_millis(T);
        (scpi, line)
    }

    /// Each question gets itself back at once, but for `LATE?`, 1.6 reads late.
    fn late_once(q: &str) -> Vec<(u64, Vec<u8>)> {
        let delay = if q == "LATE?" { 8 } else { 0 };
        vec![(delay, format!("{q}\n").into_bytes())]
    }

    #[test]
    fn a_late_answer_does_not_shift_what_follows() {
        let (mut scpi, line) = paced(late_once);
        assert!(matches!(scpi.get_string("LATE?"), Err(Error::Timeout)));
        // It comes while the next send waits for the line to go quiet.
        assert_eq!(scpi.get_string("B?").unwrap(), "B?");
        assert_eq!(scpi.get_string("C?").unwrap(), "C?");
        assert_eq!(line.lock().unwrap().sent, ["LATE?", "B?", "C?"]);
    }

    #[test]
    fn a_late_answer_in_pieces_goes_whole() {
        fn pieces(q: &str) -> Vec<(u64, Vec<u8>)> {
            if q == "LATE?" {
                vec![(8, b"LA".to_vec()), (3, b"TE?\n".to_vec())]
            } else {
                vec![(0, format!("{q}\n").into_bytes())]
            }
        }
        let (mut scpi, _) = paced(pieces);
        assert!(matches!(scpi.get_string("LATE?"), Err(Error::Timeout)));
        assert_eq!(scpi.get_string("B?").unwrap(), "B?");
    }

    #[test]
    fn a_line_quiet_from_the_start_only_costs_the_wait() {
        fn never(q: &str) -> Vec<(u64, Vec<u8>)> {
            if q == "NEVER?" { Vec::new() } else { late_once(q) }
        }
        let (mut scpi, _) = paced(never);
        assert!(matches!(scpi.get_string("NEVER?"), Err(Error::Timeout)));
        assert_eq!(scpi.get_string("B?").unwrap(), "B?");
    }

    #[test]
    fn a_draining_line_is_left_at_the_bound() {
        // An instrument that doesn't stop: the send goes out all the same.
        fn stream(q: &str) -> Vec<(u64, Vec<u8>)> {
            if q == "RUN?" {
                let mut pieces = vec![(8, b"x".to_vec())];
                pieces.extend((0..2000).map(|_| (1, b"x".to_vec())));
                pieces
            } else {
                Vec::new()
            }
        }
        let (mut scpi, line) = paced(stream);
        assert!(matches!(scpi.get_string("RUN?"), Err(Error::Timeout)));
        within(move || {
            scpi.send("B?").unwrap();
            let line = line.lock().unwrap();
            assert_eq!(line.sent, ["RUN?", "B?"]);
            assert!(!line.coming.is_empty(), "the bound came first: {} discards", line.discards);
        });
    }

    #[test]
    fn opc_reads_on_for_its_late_answer_and_owes_nothing() {
        fn late_opc(q: &str) -> Vec<(u64, Vec<u8>)> {
            let (delay, text) = if q == "*OPC?" { (8, "1".to_owned()) } else { (0, q.to_owned()) };
            vec![(delay, format!("{text}\n").into_bytes())]
        }
        let (mut scpi, line) = paced(late_opc);
        scpi.get_opc().unwrap();
        assert_eq!(scpi.get_string("A?").unwrap(), "A?", "not the 1 of a second *OPC?");
        let line = line.lock().unwrap();
        assert_eq!(line.sent, ["*OPC?", "A?"]);
        assert_eq!(line.discards, 0, "the line owes nothing, so nothing to wait out");
    }

    #[test]
    fn opc_takes_a_long_silence_and_gives_up_on_a_longer_one() {
        fn slow(_: &str) -> Vec<(u64, Vec<u8>)> {
            vec![(14 * T, b"1\n".to_vec())]
        }
        let (mut scpi, line) = paced(slow);
        scpi.opc_silence = Duration::from_secs(60);
        within(move || scpi.get_opc().unwrap());
        assert_eq!(line.lock().unwrap().sent.len(), 1);

        let (mut scpi, line) = paced(|_| Vec::new());
        scpi.opc_silence = Duration::from_millis(20);
        let e = within(move || scpi.get_opc().unwrap_err().to_string());
        assert!(e.contains("no answer to *OPC? in 0.02 s"), "{e}");
        assert_eq!(line.lock().unwrap().sent.len(), 1, "one question");
    }

    #[test]
    fn opc_asks_again_where_answers_have_tags() {
        fn never(_: &str) -> Vec<(u64, Vec<u8>)> {
            Vec::new()
        }
        let (mut scpi, line) = paced_as(never, true, false);
        // Several reads' worth, however slowly a loaded machine sleeps through one.
        scpi.opc_silence = Duration::from_millis(500);
        assert!(within(move || scpi.get_opc().is_err()));
        assert!(line.lock().unwrap().sent.len() >= 2, "as C asks, with bTag to tell them apart");
    }

    #[test]
    fn opc_counts_a_hundred_zeros() {
        fn zero(_: &str) -> Vec<(u64, Vec<u8>)> {
            vec![(0, b"0\n".to_vec())]
        }
        let (mut scpi, line) = paced(zero);
        let e = scpi.get_opc().unwrap_err().to_string();
        assert!(e.contains("*OPC? never answered 1"), "{e}");
        assert_eq!(line.lock().unwrap().sent.len(), 100, "C's count");
    }

    #[test]
    fn opc_says_when_the_line_is_gone() {
        let (mut scpi, line) = paced_as(late_once, false, true);
        let e = within(move || scpi.get_opc().unwrap_err().to_string());
        assert!(e.contains("the line is gone"), "{e}");
        assert_eq!(line.lock().unwrap().sent.len(), 1, "at once");
    }

    #[test]
    fn opc_stops_when_it_is_told_to() {
        fn never(_: &str) -> Vec<(u64, Vec<u8>)> {
            Vec::new()
        }
        let (mut scpi, line) = paced(never);
        within(move || {
            scpi.get_opc_while(|| false).unwrap();
            let calls = std::cell::Cell::new(0);
            scpi.get_opc_while(|| {
                calls.set(calls.get() + 1);
                calls.get() < 3
            })
            .unwrap();
        });
        assert_eq!(line.lock().unwrap().sent.len(), 1, "none for the first, one for the second");
    }

    #[test]
    fn a_block_cut_short_does_not_shift_what_follows() {
        fn cut(q: &str) -> Vec<(u64, Vec<u8>)> {
            if q == "D?" {
                vec![(0, b"#15ab".to_vec()), (8, b"cde".to_vec())]
            } else {
                vec![(0, format!("{q}\n").into_bytes())]
            }
        }
        let (mut scpi, _) = paced(cut);
        assert_eq!(scpi.get_block(Some("D?")).unwrap(), b"ab");
        assert_eq!(scpi.get_string("B?").unwrap(), "B?", "the rest of the block is gone");
    }

    #[test]
    fn a_driver_that_reads_for_itself_leaves_the_line_to_settle() {
        fn late(q: &str) -> Vec<(u64, Vec<u8>)> {
            if q == "D?" {
                vec![(8, b"late".to_vec())]
            } else {
                vec![(0, format!("{q}\n").into_bytes())]
            }
        }
        let (mut scpi, _) = paced(late);
        scpi.send("D?").unwrap();
        scpi.read_begin().unwrap();
        let mut buf = [0u8; 8];
        assert_eq!(scpi.read_data(&mut buf).unwrap(), 0);
        assert_eq!(scpi.get_string("B?").unwrap(), "B?");
    }

    #[test]
    fn a_block_read_that_fails_leaves_the_line_to_settle() {
        fn late(q: &str) -> Vec<(u64, Vec<u8>)> {
            if q == "D?" {
                vec![(8, b"#13abc".to_vec())]
            } else {
                vec![(0, format!("{q}\n").into_bytes())]
            }
        }
        let (mut scpi, _) = paced(late);
        assert!(matches!(scpi.get_block(Some("D?")), Err(Error::Timeout)));
        assert_eq!(scpi.get_string("B?").unwrap(), "B?");
    }

    /// The commands of a driver's table, as `sr_scpi_cmd` takes them.
    #[derive(Clone, Copy, PartialEq)]
    enum Cmd {
        Select,
        Set,
        Get,
        Missing,
    }

    const TABLE: [(Cmd, &str); 3] =
        [(Cmd::Select, "INST %s"), (Cmd::Set, ":VOLT %.3f"), (Cmd::Get, ":VOLT?")];

    #[test]
    fn commands_select_their_channel_once_and_a_missing_one_is_fine() {
        fn queries(q: &str) -> Vec<(u64, Vec<u8>)> {
            if q.ends_with('?') { late_once(q) } else { Vec::new() }
        }
        let (mut scpi, line) = paced(queries);
        let ch = |n| Some((Cmd::Select, n));
        scpi.cmd(&TABLE, ch("1"), &Cmd::Set, &[CmdArg::Float(1.5)]).unwrap();
        scpi.cmd(&TABLE, ch("1"), &Cmd::Set, &[CmdArg::Float(2.0)]).unwrap();
        assert_eq!(scpi.cmd_resp(&TABLE, ch("2"), &Cmd::Get, &[]).unwrap(), ":VOLT?");
        scpi.cmd(&TABLE, ch("3"), &Cmd::Missing, &[]).unwrap();
        assert!(matches!(
            scpi.cmd_resp(&TABLE, ch("3"), &Cmd::Missing, &[]),
            Err(Error::NotSupported(_))
        ));
        scpi.cmd(&TABLE, None, &Cmd::Set, &[CmdArg::Float(3.0)]).unwrap();
        assert_eq!(
            line.lock().unwrap().sent,
            ["INST 1", ":VOLT 1.500", ":VOLT 2.000", "INST 2", ":VOLT?", ":VOLT 3.000"]
        );
    }

    #[test]
    fn templates_render_as_printf_does() {
        let (s, f, i) = (CmdArg::Str("CH1"), CmdArg::Float(1.25), CmdArg::Int(-3));
        assert_eq!(render(":A %s", &[s]), ":A CH1");
        assert_eq!(render(":A %.6f", &[f]), ":A 1.250000");
        assert_eq!(render(":A %f", &[f]), ":A 1.250000");
        assert_eq!(render(":A %.0fV", &[CmdArg::Float(2.0)]), ":A 2V");
        assert_eq!(render(":A %d,%i", &[i, CmdArg::Int(4)]), ":A -3,4");
        assert_eq!(render("%s:%s", &[s, CmdArg::Str("x")]), "CH1:x");
        assert_eq!(render("100%% %u", &[CmdArg::Int(7)]), "100% 7");
        assert_eq!(render(":A %s", &[]), ":A %s", "shows in a .tx");
        assert_eq!(render(":A %d", &[s]), ":A %d");
        assert_eq!(render(":A %", &[]), ":A %");
    }

    #[test]
    fn strings_unquote_as_c_does() {
        for (text, want) in [
            ("\"VOLT:DC\"", "VOLT:DC"),
            ("'a'", "a"),
            ("\"a\"\"b\"", "a\"b"),
            ("'it''s'", "it's"),
            ("\"", "\""),
            ("", ""),
            ("plain", "plain"),
            ("\"mismatch'", "\"mismatch'"),
            ("\"\"", ""),
        ] {
            assert_eq!(unquote_string(text), want, "{text}");
        }
    }

    #[test]
    fn lists_of_floats_read_whole_or_not_at_all() {
        fn list(q: &str) -> Vec<(u64, Vec<u8>)> {
            let text: &[u8] = if q == "A?" { b"1.5, 2,-3e1\n" } else { b"1,x\n" };
            vec![(0, text.to_vec())]
        }
        let (mut scpi, _) = paced(list);
        assert_eq!(scpi.get_floatv("A?").unwrap(), [1.5, 2.0, -30.0]);
        assert!(scpi.get_floatv("B?").is_err());
    }

    #[test]
    fn booleans_read_as_c_reads_them() {
        for (text, want) in [
            ("1", Some(true)),
            ("0", Some(false)),
            ("ON", Some(true)),
            ("off", Some(false)),
            ("T'D", Some(true)),
            ("YES", Some(true)),
            ("NO", Some(false)),
            ("FALSE", Some(false)),
            ("2", None),
            ("", None),
            ("10", None),
        ] {
            assert_eq!(parse_bool(text), want, "{text}");
        }
    }

    #[test]
    fn a_resource_that_does_not_open_is_no_device() {
        // Nothing listens on the port of a socket just closed.
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        drop(listener);
        let conn = format!("tcp-raw/127.0.0.1/{port}");
        let found: Vec<()> = scan(Some(&conn), None, |_| Ok(Some(()))).unwrap();
        assert_eq!(found, Vec::<()>::new());
        assert!(scan::<()>(Some(""), None, |_| Ok(Some(()))).is_err(), "a bad resource says so");
    }

    fn scpi(fixture: &str) -> Scpi {
        Scpi::new(Box::new(Replay::from_text(fixture).unwrap()))
    }

    #[test]
    fn resources_parse() {
        assert_eq!(Resource::parse("usbtmc").unwrap(), Resource::Usbtmc(None));
        assert_eq!(Resource::parse("usbtmc/1.5").unwrap(), Resource::Usbtmc(Some("1.5".into())));
        assert_eq!(
            Resource::parse("tcp-raw/192.168.1.2/5555").unwrap(),
            Resource::TcpRaw("192.168.1.2".into(), 5555)
        );
        assert_eq!(
            Resource::parse("tcp-rigol/scope/5555").unwrap().to_conn(),
            "tcp-rigol/scope/5555"
        );
        assert_eq!(
            Resource::parse("/dev/ttyUSB0").unwrap(),
            Resource::Serial("/dev/ttyUSB0".into())
        );
        assert!(Resource::parse("tcp-raw/host").is_err());
        assert!(Resource::parse("vxi/host").is_err());
    }

    #[test]
    fn idn_and_values() {
        let mut s = scpi(
            "# oxrok scpi fixture 1\nresource usbtmc/1.5\nsend \"*IDN?\\n\"\nresponse \"Rigol Technologies,DS1102E,DS1EB104702974,00.04.02\\n\"\nsend \":CHAN1:DISP?\\n\"\nresponse \"1\\n\"\nsend \":TIM:SCAL?\\n\"\nresponse \"1.000000e-03\\n\"\nsend \":ACQ:MDEP?\\n\"\nresponse \"1.2E+04\\n\"\n",
        );
        let id = s.hw_id().unwrap();
        assert_eq!(id.manufacturer, "Rigol Technologies");
        assert_eq!(id.model, "DS1102E");
        assert_eq!(id.firmware_version, "00.04.02");
        assert!(s.get_bool(":CHAN1:DISP?").unwrap());
        assert!((s.get_f32(":TIM:SCAL?").unwrap() - 1e-3).abs() < 1e-9);
        assert_eq!(s.get_int(":ACQ:MDEP?").unwrap(), 12000);
        assert_eq!(vendor_alias("RIGOL TECHNOLOGIES"), "Rigol");
        assert_eq!(vendor_alias("Rigol Technologies"), "Rigol");
        assert_eq!(vendor_alias("Other"), "Other");
    }

    /// A long reply of what is no text, as a wrong line rate gives: the debug
    /// log's cut must fall between characters.
    #[test]
    fn a_long_reply_that_is_no_text_can_be_logged() {
        struct Everything;
        impl log::Log for Everything {
            fn enabled(&self, _: &log::Metadata<'_>) -> bool {
                true
            }
            fn log(&self, record: &log::Record<'_>) {
                let _ = record.args().to_string();
            }
            fn flush(&self) {}
        }
        static LOGGER: Everything = Everything;
        // Fails only when another test set one first, which is as good.
        let _ = log::set_logger(&LOGGER);
        log::set_max_level(log::LevelFilter::Debug);
        // Each lossy byte becomes three, so byte 70 falls inside a character.
        let bytes = "ff".repeat(100);
        let mut s = scpi(&format!(
            "# oxrok scpi fixture 1\nresource usbtmc/1.5\nsend \"X?\\n\"\nresponse hex {bytes}0a\n"
        ));
        assert_eq!(s.get_string("X?").unwrap().chars().count(), 100);
    }

    #[test]
    fn blocks() {
        let mut s = scpi(
            "# oxrok scpi fixture 1\nresource usbtmc/1.5\nsend \":WAV:DATA?\\n\"\nresponse hex 2332303505010203040a\n",
        );
        assert_eq!(s.get_block(Some(":WAV:DATA?")).unwrap(), vec![5, 1, 2, 3, 4]);
        let mut s = scpi(
            "# oxrok scpi fixture 1\nresource usbtmc/1.5\nsend \":X?\\n\"\nresponse \"abc\\n\"\n",
        );
        assert!(s.get_block(Some(":X?")).is_err());
    }
}
