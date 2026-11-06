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
    /// Reads what arrives within `timeout` and drops it, leaving no step in a
    /// recording, for a transport where a read is only a read and sends nothing.
    /// How much came, or `None` where a transport has no such read: USBTMC asks
    /// the device for each message, so a look at the line is a request.
    fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
        let _ = timeout;
        Ok(None)
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
    /// A read timed out, so an answer may be on its way to no question.
    stale: bool,
}

/// How many times `*OPC?` is asked.
const OPC_TRIES: u32 = 100;
/// How many `*OPC?` in a row may time out before the instrument counts as gone.
const OPC_SILENCES: u32 = 3;

/// Bytes read per call while collecting a response.
const CHUNK: usize = 4096;

impl Scpi {
    /// Wraps an open transport.
    pub fn new(transport: Box<dyn ScpiTransport>) -> Scpi {
        Scpi { transport, read_timeout: Duration::from_secs(1), stale: false }
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
        // A late answer must not pass for the answer to this. The purge takes what
        // has come: one later still is beyond telling on a line with no tags.
        if std::mem::take(&mut self.stale) {
            self.transport.purge()?;
        }
        let mut data = command.as_bytes().to_vec();
        if data.last() != Some(&b'\n') {
            data.push(b'\n');
        }
        self.transport.send(&data)
    }

    /// Starts reading a response; see [`Scpi::read_data`].
    pub fn read_begin(&mut self) -> Result<()> {
        self.transport.read_begin()
    }

    /// Reads part of a response started with [`Scpi::read_begin`], for
    /// responses too large to collect at once. Returns 0 on a timeout.
    pub fn read_data(&mut self, buf: &mut [u8]) -> Result<usize> {
        let timeout = self.read_timeout;
        self.transport.read_data(buf, timeout)
    }

    /// True once the response started with [`Scpi::read_begin`] is
    /// complete.
    pub fn read_complete(&self) -> bool {
        self.transport.read_complete()
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

    /// Waits for `*OPC?` to answer true: up to a hundred tries 10 ms apart, as
    /// `sr_scpi_get_opc`. libsigrok goes on through a timeout too, which makes
    /// 100 s of a dead instrument, on each setting and with no way to stop it, so
    /// [`OPC_SILENCES`] of those in a row end it here.
    pub fn get_opc(&mut self) -> Result<()> {
        let mut silences = 0;
        for _ in 0..OPC_TRIES {
            match self.get_bool("*OPC?") {
                Ok(true) => return Ok(()),
                Err(Error::Timeout) => {
                    silences += 1;
                    if silences >= OPC_SILENCES {
                        return Err(Error::device("SCPI: no answer to *OPC?"));
                    }
                }
                _ => silences = 0,
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        Err(Error::device("SCPI: *OPC? never answered 1"))
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
                buf.drain(..header_len);
                return Ok(buf);
            }
            let left = deadline.saturating_duration_since(Instant::now());
            let n = match self.transport.read_data(&mut chunk, left.max(Duration::from_millis(1))) {
                Ok(n) => n,
                Err(Error::Timeout) => 0,
                Err(e) => return Err(e),
            };
            if n > 0 {
                buf.extend_from_slice(&chunk[..n]);
                deadline = Instant::now() + self.read_timeout;
            } else if Instant::now() >= deadline {
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
fn parse_bool(s: &str) -> Option<bool> {
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
                match open(&res, serialcomm) {
                    Ok(scpi) => match probe(scpi) {
                        Ok(Some(d)) => found.push(d),
                        Ok(None) => {}
                        Err(e) => log::info!("Probe of {} failed: {e}", res.to_conn()),
                    },
                    Err(e) => log::info!("Cannot open {}: {e}", res.to_conn()),
                }
            }
        }
        Some(res) => match open(&res, serialcomm) {
            Ok(scpi) => match probe(scpi) {
                Ok(Some(d)) => found.push(d),
                Ok(None) => {}
                Err(e) => log::info!("Probe of {} failed: {e}", res.to_conn()),
            },
            Err(e) => log::info!("Couldn't open SCPI device {}: {e}", res.to_conn()),
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

    /// What a [`Fake`] did, and answers to give: `None` is a timeout.
    #[derive(Default)]
    struct Line {
        answers: VecDeque<Option<&'static str>>,
        sent: Vec<String>,
        purges: u32,
        /// What arrived unasked, which a purge drops.
        arrived: Option<&'static str>,
    }

    struct Fake {
        line: Arc<Mutex<Line>>,
        done: bool,
    }

    impl ScpiTransport for Fake {
        fn resource(&self) -> &'static str {
            "fake"
        }
        fn send(&mut self, data: &[u8]) -> Result<()> {
            self.line.lock().unwrap().sent.push(String::from_utf8_lossy(data).into_owned());
            Ok(())
        }
        fn read_begin(&mut self) -> Result<()> {
            self.done = false;
            Ok(())
        }
        fn read_data(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<usize> {
            let mut line = self.line.lock().unwrap();
            let next = line.arrived.take().map(Some).or_else(|| line.answers.pop_front());
            if let Some(Some(text)) = next {
                buf[..text.len()].copy_from_slice(text.as_bytes());
                self.done = true;
                Ok(text.len())
            } else {
                // Past the 1 ms deadline, so one None is one timeout.
                std::thread::sleep(Duration::from_millis(3));
                Err(Error::Timeout)
            }
        }
        fn read_complete(&self) -> bool {
            self.done
        }
        fn purge(&mut self) -> Result<()> {
            let mut line = self.line.lock().unwrap();
            line.purges += 1;
            line.arrived = None;
            Ok(())
        }
    }

    fn faked(answers: &[Option<&'static str>]) -> (Scpi, Arc<Mutex<Line>>) {
        let line = Arc::new(Mutex::new(Line {
            answers: answers.iter().copied().collect(),
            ..Line::default()
        }));
        let fake = Fake { line: Arc::clone(&line), done: false };
        let mut scpi = Scpi::new(Box::new(fake));
        scpi.read_timeout = Duration::from_millis(1);
        (scpi, line)
    }

    #[test]
    fn opc_waits_a_hundred_tries_and_gives_up_on_silence() {
        let mut busy = vec![Some("0"); 99];
        busy.push(Some("1"));
        let (mut scpi, line) = faked(&busy);
        scpi.get_opc().unwrap();
        assert_eq!(line.lock().unwrap().sent.len(), 100, "C's count");

        let (mut scpi, _) = faked(&[Some("0"); 100]);
        let e = scpi.get_opc().unwrap_err().to_string();
        assert!(e.contains("*OPC? never answered 1"), "{e}");

        let (mut scpi, line) = faked(&[None, None, None, Some("1")]);
        let e = scpi.get_opc().unwrap_err().to_string();
        assert!(e.contains("no answer to *OPC?"), "{e}");
        assert_eq!(line.lock().unwrap().sent.len(), 3, "three silences end it");

        // Silences not in a row don't.
        let (mut scpi, _) = faked(&[None, None, Some("0"), None, None, Some("1")]);
        scpi.get_opc().unwrap();
    }

    #[test]
    fn a_late_answer_is_not_taken_for_the_next() {
        let (mut scpi, line) = faked(&[None, Some("right")]);
        assert!(scpi.get_string("A?").is_err());
        assert_eq!(line.lock().unwrap().purges, 0, "the timeout purges nothing yet");
        line.lock().unwrap().arrived = Some("late");
        assert_eq!(scpi.get_string("B?").unwrap(), "right");
        assert_eq!(line.lock().unwrap().purges, 1);
        // Once: the line is as good as new.
        line.lock().unwrap().answers.push_back(Some("next"));
        assert_eq!(scpi.get_string("C?").unwrap(), "next");
        assert_eq!(line.lock().unwrap().purges, 1);
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
