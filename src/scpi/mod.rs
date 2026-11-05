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
    /// Drops what has arrived and nobody asked for, where the transport has a
    /// queue of that. Nothing on the wire.
    fn purge(&mut self) -> Result<()> {
        Ok(())
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
}

/// Bytes read per call while collecting a response.
const CHUNK: usize = 4096;

impl Scpi {
    /// Wraps an open transport.
    pub fn new(transport: Box<dyn ScpiTransport>) -> Scpi {
        Scpi { transport, read_timeout: Duration::from_secs(1) }
    }

    /// The resource string.
    pub fn resource(&self) -> &str {
        self.transport.resource()
    }

    /// Drops what has arrived unasked; see [`ScpiTransport::purge`].
    pub fn purge(&mut self) -> Result<()> {
        self.transport.purge()
    }

    /// Sends `command`, adding the newline when it lacks one.
    pub fn send(&mut self, command: &str) -> Result<()> {
        log::debug!("SCPI send: {}", command.trim_end());
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
                Err(e) => return Err(e),
            };
            if n > 0 {
                out.extend_from_slice(&chunk[..n]);
                deadline = Instant::now() + self.read_timeout;
            } else if Instant::now() >= deadline {
                log::debug!("Timed out waiting for the SCPI response.");
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

    /// Waits for `*OPC?` to answer true, up to ten tries 10 ms apart.
    pub fn get_opc(&mut self) -> Result<()> {
        for _ in 0..10 {
            if self.get_bool("*OPC?").unwrap_or(false) {
                return Ok(());
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

fn parse_bool(s: &str) -> Option<bool> {
    match s.trim().to_ascii_lowercase().as_str() {
        "1" | "on" | "true" | "yes" => Some(true),
        "0" | "off" | "false" | "no" => Some(false),
        _ => None,
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
/// `scpi_serial_open` does when it has none, where [`open`] has a default.
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
        Some(res) => {
            let scpi = open(&res, serialcomm)?;
            if let Some(d) = probe(scpi)? {
                found.push(d);
            }
        }
    }
    Ok(found)
}

#[cfg(test)]
mod tests {
    use super::*;
    use record::Replay;

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
