// Counterpart in sigrok:
//   libsigrok/src/serial.c,
//   libsigrok/src/serial_libsp.c,
//   Copyright (C) 2010-2012 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2010-2012 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2012 Alexandru Gagniuc <mr.nuke.me@gmail.com>,
//   Copyright (C) 2014 Uffe Jakobsen <uffe@uffe.org>,
//   Copyright (C) 2017-2019 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Martin Ling, Daniel Elstner, Aurelien Jacobs,
//   Soeren Apel, Frank Stettner, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! Serial port transport: a small trait over a port, the sigrok
//! `serialcomm` parameter syntax, and record and replay of a session.
//!
//! Drivers talk to a [`Port`]. [`open`] gives a real one and, with
//! `OXROK_SERIAL_RECORD=FILE` set, records everything to a fixture that
//! [`record::Replay`] plays back in tests without the hardware.
//!
//! A fixture is a text file, one call per line:
//!
//! ```text
//! # oxrok serial fixture 1
//! port /dev/ttyUSB0 2400/8n1/rts=0/dtr=1
//! write 44
//! read 32 -> 2b30313334203420...
//! read 32 -> -
//! ```
//!
//! `read N -> DATA` is a read of up to `N` bytes that returned `DATA`,
//! or `-` when it returned nothing before its timeout. A `params` line
//! records a later change of the parameters, `handshake RTS DTR` a
//! change of the lines (`-` for unchanged).

#![forbid(unsafe_code)]

pub mod hid;
pub mod record;

use crate::core::{Error, Result};
use std::time::Duration;

/// Parity of a serial frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Parity {
    /// No parity bit.
    #[default]
    None,
    /// Even parity.
    Even,
    /// Odd parity.
    Odd,
}

/// Flow control.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Flow {
    /// None.
    #[default]
    None,
    /// RTS/CTS hardware flow control.
    RtsCts,
    /// XON/XOFF software flow control.
    XonXoff,
}

/// Port parameters, in the sigrok `serialcomm` syntax:
/// `9600/8n1`, `600/7o2/dtr=1/rts=0`, `460800/8n1/flow=2`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Params {
    /// Bits per second.
    pub baud: u32,
    /// Data bits, 5 to 8.
    pub data_bits: u8,
    /// Parity.
    pub parity: Parity,
    /// Stop bits, 1 or 2.
    pub stop_bits: u8,
    /// Flow control.
    pub flow: Flow,
    /// RTS line level to set, or leave alone.
    pub rts: Option<bool>,
    /// DTR line level to set, or leave alone.
    pub dtr: Option<bool>,
}

impl Params {
    /// Parses the `serialcomm` syntax. The speed is required; the frame
    /// defaults to `8n1`, the lines to unchanged.
    pub fn parse(text: &str) -> Result<Params> {
        let invalid = || Error::Arg(format!("Invalid serial parameters '{text}'."));
        let mut fields = text.split('/');
        let baud: u32 =
            fields.next().and_then(|s| s.parse().ok()).filter(|b| *b > 0).ok_or_else(invalid)?;
        let mut p = Params {
            baud,
            data_bits: 8,
            parity: Parity::None,
            stop_bits: 1,
            flow: Flow::None,
            rts: None,
            dtr: None,
        };
        let rest: Vec<&str> = fields.collect();
        let mut options = rest.as_slice();
        if let Some(frame) = rest.first()
            && frame.len() == 3
            && !frame.contains('=')
        {
            let b = frame.as_bytes();
            p.data_bits = match b[0] {
                b'5'..=b'8' => b[0] - b'0',
                _ => return Err(invalid()),
            };
            p.parity = match b[1] {
                b'n' => Parity::None,
                b'e' => Parity::Even,
                b'o' => Parity::Odd,
                _ => return Err(invalid()),
            };
            p.stop_bits = match b[2] {
                b'1' | b'2' => b[2] - b'0',
                _ => return Err(invalid()),
            };
            options = &rest[1..];
        }
        for opt in options {
            let (k, v) = opt.split_once('=').ok_or_else(invalid)?;
            let level = match v {
                "0" => false,
                "1" => true,
                "2" if k == "flow" => true,
                _ => return Err(invalid()),
            };
            match k {
                "rts" => p.rts = Some(level),
                "dtr" => p.dtr = Some(level),
                "flow" => {
                    p.flow = match v {
                        "0" => Flow::None,
                        "1" => Flow::RtsCts,
                        _ => Flow::XonXoff,
                    }
                }
                _ => return Err(invalid()),
            }
        }
        Ok(p)
    }

    /// The parameters in the `serialcomm` syntax.
    pub fn to_text(self) -> String {
        let mut s = format!(
            "{}/{}{}{}",
            self.baud,
            self.data_bits,
            match self.parity {
                Parity::None => 'n',
                Parity::Even => 'e',
                Parity::Odd => 'o',
            },
            self.stop_bits
        );
        match self.flow {
            Flow::None => {}
            Flow::RtsCts => s.push_str("/flow=1"),
            Flow::XonXoff => s.push_str("/flow=2"),
        }
        if let Some(r) = self.rts {
            s.push_str(if r { "/rts=1" } else { "/rts=0" });
        }
        if let Some(d) = self.dtr {
            s.push_str(if d { "/dtr=1" } else { "/dtr=0" });
        }
        s
    }
}

/// An open serial port.
pub trait Port: Send {
    /// The path or name it was opened with.
    fn name(&self) -> &str;
    /// Applies parameters, including the RTS and DTR lines when given.
    fn set_params(&mut self, params: Params) -> Result<()>;
    /// Sets the RTS and DTR lines; `None` leaves a line as it is.
    fn set_handshake(&mut self, rts: Option<bool>, dtr: Option<bool>) -> Result<()>;
    /// Reads up to `buf.len()` bytes, waiting at most `timeout` for the
    /// first one. Returns 0 when nothing arrived in time.
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize>;
    /// Writes all of `data`.
    fn write(&mut self, data: &[u8]) -> Result<()>;
    /// Discards pending input.
    fn purge(&mut self) -> Result<()>;
}

impl dyn Port {
    /// Reads exactly `buf.len()` bytes or fails with a timeout.
    pub fn read_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<()> {
        let mut got = 0;
        while got < buf.len() {
            let n = self.read(&mut buf[got..], timeout)?;
            if n == 0 {
                return Err(Error::Timeout);
            }
            got += n;
        }
        Ok(())
    }
}

/// A port on the operating system.
pub struct Native {
    name: String,
    port: Box<dyn serialport::SerialPort>,
}

fn map_err(e: &serialport::Error) -> Error {
    match e.kind() {
        serialport::ErrorKind::InvalidInput => Error::Arg(e.to_string()),
        _ => Error::device(format!("serial port: {e}")),
    }
}

fn data_bits(p: Params) -> serialport::DataBits {
    use serialport::DataBits;
    match p.data_bits {
        5 => DataBits::Five,
        6 => DataBits::Six,
        7 => DataBits::Seven,
        _ => DataBits::Eight,
    }
}

fn parity(p: Params) -> serialport::Parity {
    match p.parity {
        Parity::None => serialport::Parity::None,
        Parity::Even => serialport::Parity::Even,
        Parity::Odd => serialport::Parity::Odd,
    }
}

fn stop_bits(p: Params) -> serialport::StopBits {
    if p.stop_bits == 2 { serialport::StopBits::Two } else { serialport::StopBits::One }
}

fn flow_control(p: Params) -> serialport::FlowControl {
    match p.flow {
        Flow::None => serialport::FlowControl::None,
        Flow::RtsCts => serialport::FlowControl::Hardware,
        Flow::XonXoff => serialport::FlowControl::Software,
    }
}

/// Opens the tty `name` and clears `HUPCL`, as `sp_open` does: a close
/// leaves DTR and RTS as they are, where the kernel would lower them, and
/// some adapters reset their board on that. The crate has no call for
/// it, but termios belongs to the tty, so this goes through a descriptor
/// of its own. It comes first since the crate's open takes `TIOCEXCL`,
/// which keeps a later open out; the caller keeps it until that open is
/// done, so the tty stays open in between. Whatever fails here is the
/// crate's open to report.
#[cfg(unix)]
fn clear_hupcl(name: &str) -> Option<std::fs::File> {
    use rustix::fs::OFlags;
    use rustix::termios::{ControlModes, OptionalActions, tcgetattr, tcsetattr};
    use std::os::unix::fs::OpenOptionsExt as _;
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags((OFlags::NOCTTY | OFlags::NONBLOCK).bits().cast_signed())
        .open(name)
        .ok()?;
    let mut termios = tcgetattr(&file).ok()?;
    termios.control_modes.remove(ControlModes::HUPCL);
    tcsetattr(&file, OptionalActions::Now, &termios).ok()?;
    Some(file)
}

impl Native {
    /// Opens `name` with `params`, as `serial_open` does: raw, without
    /// `HUPCL`, the frame and speed set in one step (one
    /// `SET_LINE_CODING` on a CDC-ACM port; the crate's `set_*` calls
    /// after an open would each send theirs), then RTS and DTR when given,
    /// then both buffers flushed. The crate's open takes `TIOCEXCL` and an
    /// exclusive `flock`, as `sp_open` does. Unlike `sp_open` it goes
    /// through `cfmakeraw`, which sets 8 bits without parity for a moment
    /// on a port left in another frame; only the wire shows that.
    pub fn open(name: &str, params: Params) -> Result<Native> {
        #[cfg(unix)]
        let _hupcl = clear_hupcl(name);
        let port = serialport::new(name, params.baud)
            .data_bits(data_bits(params))
            .parity(parity(params))
            .stop_bits(stop_bits(params))
            .flow_control(flow_control(params))
            .preserve_dtr_on_open()
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| map_err(&e))?;
        let mut native = Native { name: name.to_owned(), port };
        native.set_handshake(params.rts, params.dtr)?;
        native.port.clear(serialport::ClearBuffer::All).map_err(|e| map_err(&e))?;
        Ok(native)
    }
}

impl Port for Native {
    fn name(&self) -> &str {
        &self.name
    }

    fn set_params(&mut self, p: Params) -> Result<()> {
        let port = &mut self.port;
        port.set_baud_rate(p.baud).map_err(|e| map_err(&e))?;
        port.set_data_bits(data_bits(p)).map_err(|e| map_err(&e))?;
        port.set_parity(parity(p)).map_err(|e| map_err(&e))?;
        port.set_stop_bits(stop_bits(p)).map_err(|e| map_err(&e))?;
        port.set_flow_control(flow_control(p)).map_err(|e| map_err(&e))?;
        self.set_handshake(p.rts, p.dtr)
    }

    fn set_handshake(&mut self, rts: Option<bool>, dtr: Option<bool>) -> Result<()> {
        if let Some(r) = rts {
            self.port.write_request_to_send(r).map_err(|e| map_err(&e))?;
        }
        if let Some(d) = dtr {
            self.port.write_data_terminal_ready(d).map_err(|e| map_err(&e))?;
        }
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        self.port.set_timeout(timeout).map_err(|e| map_err(&e))?;
        match std::io::Read::read(&mut self.port, buf) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(0),
            Err(e) => Err(Error::Io(e)),
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        // No drain, as `serial_write_blocking` has none: `tcdrain` waits
        // for the device to take the bytes, and a device that stopped
        // reading then holds the driver past any limit.
        std::io::Write::write_all(&mut self.port, data).map_err(Error::Io)
    }

    fn purge(&mut self) -> Result<()> {
        self.port.clear(serialport::ClearBuffer::Input).map_err(|e| map_err(&e))
    }
}

/// Opens the port `name` with `params`: a device path, or a HID cable
/// as `hid[/CHIP][/sn=SERIAL]` (see [`hid`]). With
/// `OXROK_SERIAL_RECORD=FILE` in the environment the session is
/// recorded to that file.
pub fn open(name: &str, params: Params) -> Result<Box<dyn Port>> {
    let native: Box<dyn Port> = if hid::is_hid_name(name) {
        hid::open(name, params)?
    } else {
        Box::new(Native::open(name, params)?)
    };
    if let Ok(file) = std::env::var("OXROK_SERIAL_RECORD")
        && !file.is_empty()
    {
        log::info!("Recording the serial session to {file}.");
        return Ok(Box::new(record::Recorder::to_file(native, params, file)));
    }
    Ok(native)
}

/// The serial ports on this machine.
pub fn list_ports() -> Vec<String> {
    serialport::available_ports()
        .map(|ports| ports.into_iter().map(|p| p.port_name).collect())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_serialcomm() {
        let p = Params::parse("2400/7o1/rts=0/dtr=1").unwrap();
        assert_eq!((p.baud, p.data_bits, p.parity, p.stop_bits), (2400, 7, Parity::Odd, 1));
        assert_eq!((p.rts, p.dtr, p.flow), (Some(false), Some(true), Flow::None));
        assert_eq!(p.to_text(), "2400/7o1/rts=0/dtr=1");
        let p = Params::parse("9600").unwrap();
        assert_eq!((p.data_bits, p.parity, p.stop_bits), (8, Parity::None, 1));
        let p = Params::parse("460800/8n1/flow=2").unwrap();
        assert_eq!(p.flow, Flow::XonXoff);
        assert_eq!(Params::parse("1200/7n2").unwrap().to_text(), "1200/7n2");
        for bad in ["", "abc", "9600/9n1", "9600/8x1", "9600/8n1/rts=2", "9600/8n1/foo=1"] {
            assert!(Params::parse(bad).is_err(), "{bad}");
        }
    }
}
