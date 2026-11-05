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
    /// `serial_write_blocking` with its timeout: writes what room `timeout`
    /// gives and says how much went, none being no error, so a device that
    /// stopped reading looks as it does in C. A port that fails is an error. A
    /// port with no room to wait for (a replay, a HID cable) writes it all.
    fn write_timeout(&mut self, data: &[u8], timeout: Duration) -> Result<usize> {
        let _ = timeout;
        self.write(data)?;
        Ok(data.len())
    }
    /// Discards pending input.
    fn purge(&mut self) -> Result<()>;
    /// Waits until what was written has left, `serial_drain`, for at most
    /// `timeout`: then [`Error::Timeout`]. A port with nothing to wait for
    /// (a replay, a HID cable whose reports are gone once written) returns
    /// at once. A recording has no step for it.
    fn drain(&mut self, timeout: Duration) -> Result<()> {
        let _ = timeout;
        Ok(())
    }
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

/// `serial_timeout`: 10 ms for the system, and the time `bytes` take on the
/// line, in whole milliseconds.
pub fn serial_timeout(params: &Params, bytes: u32) -> Duration {
    let parity = u32::from(params.parity != Parity::None);
    let bits = 1 + u32::from(params.data_bits) + parity + u32::from(params.stop_bits);
    let line = 1000.0 / f64::from(params.baud) * f64::from(bits) * f64::from(bytes);
    Duration::from_millis(10 + line as u64)
}

/// `serial_read_blocking` with a bound: fills `buf` until `timeout` has
/// passed and says how much came, none being no error. Where
/// [`Port::read`] returns what it has at once, a reply in several pieces
/// comes back whole, and where it waits for less than a millisecond it
/// waits for one: `Duration::as_millis` truncates, and serialport's
/// `COMMTIMEOUTS` constant of 0 on Windows is outside the combination
/// Microsoft documents for a read that waits for its first byte. A 0 from
/// a read is not the end either: a hung up tty gives one at once, so this
/// pauses and tries again until the time is up, which also means it cannot
/// spin. A `timeout` of 0 tries once, unlike C's, where 0 waits for ever.
///
/// # Errors
///
/// What the port's read gives.
pub fn read_blocking(port: &mut dyn Port, buf: &mut [u8], timeout: Duration) -> Result<usize> {
    /// The pause after a read that gave nothing before its time.
    const EMPTY_READ_PAUSE: Duration = Duration::from_millis(1);
    let deadline = std::time::Instant::now() + timeout;
    let mut got = 0;
    loop {
        if got == buf.len() {
            return Ok(got);
        }
        let left = deadline.saturating_duration_since(std::time::Instant::now());
        if left.is_zero() && !timeout.is_zero() {
            return Ok(got);
        }
        let n = port.read(&mut buf[got..], whole_millis(left))?;
        got += n;
        if timeout.is_zero() {
            return Ok(got);
        }
        if n == 0 {
            std::thread::sleep(EMPTY_READ_PAUSE.min(left));
        }
    }
}

/// `left` rounded up to a whole millisecond, and at least one.
fn whole_millis(left: Duration) -> Duration {
    let ms = left.as_nanos().div_ceil(1_000_000).max(1);
    Duration::from_millis(u64::try_from(ms).unwrap_or(u64::MAX))
}

/// A port on the operating system.
pub struct Native {
    name: String,
    port: Box<dyn serialport::SerialPort>,
    /// See [`termios_handle`].
    #[cfg(unix)]
    termios: Option<std::fs::File>,
}

/// How long a write may wait for room before it fails. It is per chunk the
/// kernel takes, so a long write at a slow rate goes through, and a device
/// that stopped reading does not hold a driver for ever, as `sp_blocking_write`
/// without a timeout does.
const WRITE_TIMEOUT: Duration = Duration::from_secs(5);

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

/// A descriptor of the tty for what the crate has no call for. It comes
/// before the crate's open, whose `TIOCEXCL` keeps any later open out.
/// The crate's `TTYPort` gives only a raw descriptor, and making a
/// borrowed one of that is `unsafe`, which this module forbids; so the tty
/// is opened twice, and on a CDC-ACM port each open sends its
/// `SET_CONTROL_LINE_STATE`. `None` when it fails: then the crate's open
/// reports why.
#[cfg(unix)]
fn termios_handle(name: &str) -> Option<std::fs::File> {
    use rustix::fs::OFlags;
    use std::os::unix::fs::OpenOptionsExt as _;
    std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags((OFlags::NOCTTY | OFlags::NONBLOCK).bits().cast_signed())
        .open(name)
        .ok()
}

/// What `sp_open` and `sp_set_config` leave and the crate does not:
/// - `HUPCL` off, so a close leaves DTR and RTS as they are, where the
///   kernel would lower them;
/// - `INPCK` off, which the crate's `set_parity` sets for odd and even.
///   libsigrok has always sent the opposite parity (`serial.c:652-655`)
///   with no check, so a table entry whose parity is wrong works there; a
///   check would turn its bytes into errors or NULs here.
#[cfg(unix)]
fn as_libserialport(file: &std::fs::File) -> Result<()> {
    use rustix::termios::{ControlModes, InputModes, OptionalActions, tcgetattr, tcsetattr};
    let io = |e: rustix::io::Errno| Error::Io(e.into());
    let mut termios = tcgetattr(file).map_err(io)?;
    termios.control_modes.remove(ControlModes::HUPCL);
    termios.input_modes.remove(InputModes::INPCK);
    tcsetattr(file, OptionalActions::Now, &termios).map_err(io)
}

impl Native {
    /// Opens `name` with `params`, as `serial_open` does: raw, without
    /// `HUPCL` or `INPCK`, the frame and speed set in one step (one
    /// `SET_LINE_CODING` on a CDC-ACM port; the crate's `set_*` calls
    /// after an open would each send theirs), then RTS and DTR when given,
    /// then both buffers flushed. The crate's open takes `TIOCEXCL` and an
    /// exclusive `flock`, as `sp_open` does. Unlike `sp_open` it goes
    /// through `cfmakeraw`, which sets 8 bits without parity for a moment
    /// on a port left in another frame; only the wire shows that.
    pub fn open(name: &str, params: Params) -> Result<Native> {
        #[cfg(unix)]
        let termios = termios_handle(name);
        let port = serialport::new(name, params.baud)
            .data_bits(data_bits(params))
            .parity(parity(params))
            .stop_bits(stop_bits(params))
            .flow_control(flow_control(params))
            .preserve_dtr_on_open()
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| map_err(&e))?;
        let mut native = Native {
            name: name.to_owned(),
            port,
            #[cfg(unix)]
            termios,
        };
        native.settle_termios()?;
        native.set_handshake(params.rts, params.dtr)?;
        native.port.clear(serialport::ClearBuffer::All).map_err(|e| map_err(&e))?;
        Ok(native)
    }

    #[cfg(unix)]
    fn settle_termios(&self) -> Result<()> {
        self.termios.as_ref().map_or(Ok(()), as_libserialport)
    }

    /// Windows has neither flag.
    #[cfg(not(unix))]
    #[allow(clippy::unused_self, clippy::unnecessary_wraps)]
    fn settle_termios(&self) -> Result<()> {
        Ok(())
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
        self.settle_termios()?;
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
        // Its own timeout: the crate polls with whatever the last read set.
        // No drain, as `serial_write_blocking` has none; see `drain`.
        self.port.set_timeout(WRITE_TIMEOUT).map_err(|e| map_err(&e))?;
        std::io::Write::write_all(&mut self.port, data).map_err(Error::Io)
    }

    /// The crate's write polls for room with the port's timeout and gives
    /// `TimedOut` when there is none, `BrokenPipe` on a hung up tty.
    fn write_timeout(&mut self, data: &[u8], timeout: Duration) -> Result<usize> {
        let deadline = std::time::Instant::now() + timeout;
        let mut done = 0;
        while done < data.len() {
            let left = deadline.saturating_duration_since(std::time::Instant::now());
            self.port.set_timeout(whole_millis(left)).map_err(|e| map_err(&e))?;
            match std::io::Write::write(&mut self.port, &data[done..]) {
                Ok(n) => done += n,
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => return Ok(done),
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
                Err(e) => return Err(Error::Io(e)),
            }
            if left.is_zero() {
                break;
            }
        }
        Ok(done)
    }

    /// `tcdrain` has no bound (a device that stopped reading held oxrok
    /// past SIGTERM with it), so this polls the output queue instead. A
    /// queue that is empty says the kernel's buffers are, not a chip's
    /// FIFO behind them, which `tcdrain` covers only on some drivers.
    fn drain(&mut self, timeout: Duration) -> Result<()> {
        let start = std::time::Instant::now();
        while self.port.bytes_to_write().map_err(|e| map_err(&e))? > 0 {
            if start.elapsed() >= timeout {
                return Err(Error::Timeout);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        Ok(())
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

    /// Plays `script`, each read taking one entry: how many bytes it gives.
    /// It remembers the time each read was given.
    struct Script {
        reads: Vec<usize>,
        next: usize,
        asked: Vec<Duration>,
        slow_zero: bool,
    }

    impl Script {
        fn new(script: &[usize], slow_zero: bool) -> Script {
            Script { reads: script.to_vec(), next: 0, asked: Vec::new(), slow_zero }
        }
    }

    impl Port for Script {
        fn name(&self) -> &'static str {
            "script"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            self.asked.push(timeout);
            let n = self.reads.get(self.next).copied().unwrap_or(0).min(buf.len());
            self.next += 1;
            if n == 0 && self.slow_zero {
                std::thread::sleep(timeout);
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

    #[test]
    fn serial_timeout_is_the_lines_time_and_10_ms() {
        let p = Params::parse("9600/8n1").unwrap();
        for (bytes, ms) in [(1, 11), (3, 13), (5, 15), (48, 60)] {
            assert_eq!(serial_timeout(&p, bytes), Duration::from_millis(ms));
        }
        let p = Params::parse("1200/7e2").unwrap();
        assert_eq!(serial_timeout(&p, 1), Duration::from_millis(19));
    }

    #[test]
    fn a_blocking_read_puts_the_pieces_together() {
        let mut port = Script::new(&[2, 1, 2], true);
        let mut buf = [0; 5];
        let n = read_blocking(&mut port, &mut buf, Duration::from_secs(5)).unwrap();
        assert_eq!(n, 5);
        assert_eq!(port.next, 3, "no read once it is full");
    }

    #[test]
    fn a_blocking_read_gives_what_came_by_its_time() {
        let mut port = Script::new(&[2], true);
        let mut buf = [0; 5];
        let begun = std::time::Instant::now();
        let n = read_blocking(&mut port, &mut buf, Duration::from_millis(30)).unwrap();
        assert_eq!(n, 2);
        assert!(begun.elapsed() >= Duration::from_millis(30), "{:?}", begun.elapsed());
    }

    #[test]
    fn a_read_that_gives_0_at_once_neither_ends_it_nor_spins() {
        // A hung up tty, then the bytes.
        let mut port = Script::new(&[0, 0, 3], false);
        let mut buf = [0; 3];
        assert_eq!(read_blocking(&mut port, &mut buf, Duration::from_secs(5)).unwrap(), 3);
        // Nothing at all: it ends at the time, with a pause per try.
        let mut port = Script::new(&[], false);
        let begun = std::time::Instant::now();
        assert_eq!(read_blocking(&mut port, &mut buf, Duration::from_millis(50)).unwrap(), 0);
        assert!(begun.elapsed() >= Duration::from_millis(50), "{:?}", begun.elapsed());
        assert!(port.asked.len() <= 51, "{} reads in 50 ms", port.asked.len());
    }

    #[test]
    fn no_read_asks_for_less_than_a_millisecond() {
        assert_eq!(whole_millis(Duration::ZERO), Duration::from_millis(1));
        assert_eq!(whole_millis(Duration::from_micros(1)), Duration::from_millis(1));
        assert_eq!(whole_millis(Duration::from_micros(1001)), Duration::from_millis(2));
        assert_eq!(whole_millis(Duration::from_millis(15)), Duration::from_millis(15));
        let mut port = Script::new(&[], false);
        let mut buf = [0; 1];
        read_blocking(&mut port, &mut buf, Duration::from_micros(300)).unwrap();
        assert!(port.asked.iter().all(|t| *t >= Duration::from_millis(1)), "{:?}", port.asked);
    }

    #[test]
    fn a_blocking_read_of_no_time_tries_once() {
        let mut port = Script::new(&[1], false);
        let mut buf = [0; 3];
        assert_eq!(read_blocking(&mut port, &mut buf, Duration::ZERO).unwrap(), 1);
        assert_eq!(port.asked.len(), 1);
    }

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

    /// A pty stands in for the tty: its master reads the slave's termios.
    /// Its driver forces 8 bits without parity in `c_cflag`, so only these
    /// flags can be seen here; `INPCK` is the crate's for odd and even
    /// whatever the driver does with the rest.
    #[cfg(target_os = "linux")]
    #[test]
    fn a_native_port_checks_no_parity_and_keeps_the_lines_at_close() {
        use rustix::pty::{OpenptFlags, grantpt, openpt, ptsname, unlockpt};
        use rustix::termios::{ControlModes, InputModes, tcgetattr};
        let master = openpt(OpenptFlags::RDWR | OpenptFlags::NOCTTY).unwrap();
        grantpt(&master).unwrap();
        unlockpt(&master).unwrap();
        let slave = ptsname(&master, Vec::new()).unwrap();
        let slave = slave.to_str().unwrap();
        for text in ["1200/7o1", "2400/7e2", "9600/8n1"] {
            let mut port = Native::open(slave, Params::parse(text).unwrap()).unwrap();
            let t = tcgetattr(&master).unwrap();
            assert!(!t.input_modes.contains(InputModes::INPCK), "{text}");
            assert!(!t.control_modes.contains(ControlModes::HUPCL), "{text}");
            // set_params goes through the crate's set_parity again.
            port.set_params(Params::parse("1200/7o1").unwrap()).unwrap();
            let t = tcgetattr(&master).unwrap();
            assert!(!t.input_modes.contains(InputModes::INPCK), "{text} set_params");
        }
    }
}
