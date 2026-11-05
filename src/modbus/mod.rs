// Counterpart in sigrok:
//   libsigrok/src/modbus/modbus.c,
//   libsigrok/src/modbus/modbus_serial_rtu.c,
//   libsigrok/src/crc.c,
//   Copyright (C) 2015 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, Daniel Elstner, Gerhard Sittig,
//   Wolfram Sang, v1ne, James Churchill, Andreas Sandberg
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Modbus RTU over a serial port, for the drivers of instruments that speak
//! it (`rdtech-dps`, and `maynuo-m97` to come).
//!
//! A [`Modbus`] holds a [`Port`] and the slave's address, and gives what
//! `modbus.c` and `modbus_serial_rtu.c` give: a request and a reply as PDUs,
//! and the functions the drivers use on top of them. Registers are host order
//! here; `modbus.c` hands the bytes over as they came and leaves that to its
//! callers.
//!
//! A frame is the address, the PDU and a CRC-16 low byte first. The timeouts
//! are `modbus_serial_rtu.c`'s: 500 ms for the first byte of a reply, then
//! 100 ms for the function code and for the CRC, and 1 s from the last byte
//! that came for the rest. A reply from another slave is a failed read, with
//! the rest of its bytes left for whoever reads next.
//!
//! Two things differ from C, neither in a frame sent to a working device:
//!
//! - A frame goes out in one write, where C writes the address, the PDU and
//!   the CRC separately. RTU ends a frame at a silence of 3.5 characters, so
//!   this is the one that cannot fall apart on a loaded host.
//! - A request first drops what is in the input queue. In C, what is left of
//!   a bad reply (too long, or another slave's) stays and fails the replies
//!   after it, one byte per try, while each try adds a new reply.
//!
//! Scan options and the open are here as well: a Modbus driver takes `conn=`,
//! `serialcomm=` and `modbusaddr=` alike.

use crate::core::{Error, Key, Result, Value};
use crate::serial::{self, Params, Port, read_blocking};
use std::time::{Duration, Instant};

/// A Modbus RTU address, when the scan options name none.
pub const DEFAULT_SLAVE: u8 = 1;

const READ_COILS: u8 = 0x01;
const READ_HOLDING_REGISTERS: u8 = 0x03;
const WRITE_COIL: u8 = 0x05;
const WRITE_MULTIPLE_REGISTERS: u8 = 0x10;
/// Set in the function code of an exception reply.
const EXCEPTION: u8 = 0x80;

const MAX_READ_COILS: usize = 2000;
const MAX_READ_REGISTERS: usize = 125;
const MAX_WRITE_REGISTERS: usize = 123;

const FIRST_BYTE: Duration = Duration::from_millis(500);
const FUNCTION_CODE: Duration = Duration::from_millis(100);
const CRC_BYTES: Duration = Duration::from_millis(100);
const READ_TIMEOUT: Duration = Duration::from_secs(1);

/// `sr_crc16` over `data`, going on from `crc`.
fn crc16_update(crc: u16, data: &[u8]) -> u16 {
    data.iter().fold(crc, |crc, byte| {
        (0..8).fold(
            crc ^ u16::from(*byte),
            |c, _| {
                if c & 1 == 1 { (c >> 1) ^ 0xa001 } else { c >> 1 }
            },
        )
    })
}

/// CRC-16/MODBUS of `data`.
pub fn crc16(data: &[u8]) -> u16 {
    crc16_update(0xffff, data)
}

/// The frame of a PDU for `slave`.
pub fn frame(slave: u8, pdu: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(pdu.len() + 3);
    out.push(slave);
    out.extend_from_slice(pdu);
    out.extend_from_slice(&crc16(&out).to_le_bytes());
    out
}

/// What the C's `SR_ERR_DATA` says.
fn invalid_data() -> Error {
    Error::device("data is invalid")
}

/// What the C's `SR_ERR` says, when nothing logged why.
fn failed(why: &str) -> Error {
    Error::device(format!("no Modbus reply: {why}"))
}

/// `sr_modbus_error_check`: logs an exception reply, and says if it was one.
fn is_exception(reply: &[u8]) -> bool {
    if reply[0] & EXCEPTION == 0 {
        return false;
    }
    let function = match reply[0] & !EXCEPTION {
        READ_COILS => "MODBUS_READ_COILS",
        READ_HOLDING_REGISTERS => "READ_HOLDING_REGISTERS",
        WRITE_COIL => "WRITE_COIL",
        WRITE_MULTIPLE_REGISTERS => "WRITE_MULTIPLE_REGISTERS",
        _ => "UNKNOWN",
    };
    let error = match reply[1] {
        0x01 => "ILLEGAL FUNCTION".to_owned(),
        0x02 => "ILLEGAL DATA ADDRESS".to_owned(),
        0x03 => "ILLEGAL DATA VALUE".to_owned(),
        0x04 => "SLAVE DEVICE FAILURE".to_owned(),
        0x05 => "ACKNOWLEDGE".to_owned(),
        0x06 => "SLAVE DEVICE BUSY".to_owned(),
        0x08 => "MEMORY PARITY ERROR".to_owned(),
        0x0a => "GATEWAY PATH UNAVAILABLE".to_owned(),
        0x0b => "GATEWAY TARGET DEVICE FAILED TO RESPOND".to_owned(),
        other => format!("{other:#X}").replace("0X", "0x"),
    };
    log::error!("{error} error executing {function} function.");
    true
}

/// A slave on a serial port.
pub struct Modbus {
    port: Box<dyn Port>,
    slave: u8,
    unsent: bool,
}

impl Modbus {
    /// A slave at address `slave`, on a port that is open.
    pub fn new(port: Box<dyn Port>, slave: u8) -> Modbus {
        Modbus { port, slave, unsent: false }
    }

    /// The name of the port.
    pub fn port_name(&self) -> &str {
        self.port.name()
    }

    /// Sends a PDU: `sr_modbus_request`.
    ///
    /// # Errors
    ///
    /// An empty PDU, and what the port gives.
    pub fn request(&mut self, pdu: &[u8]) -> Result<()> {
        if pdu.is_empty() {
            return Err(Error::arg("an empty Modbus request"));
        }
        let sent = self.port.purge().and_then(|()| self.port.write(&frame(self.slave, pdu)));
        self.unsent = sent.is_err();
        sent
    }

    /// Whether the last request could not go out, where a failure to read
    /// says nothing of the kind: the port has gone, not the slave's reply.
    pub fn request_unsent(&self) -> bool {
        self.unsent
    }

    /// Reads a reply PDU of at most `reply`'s length, its CRC checked:
    /// `sr_modbus_reply`. An exception is two bytes long, whatever `reply`'s
    /// length, and fills only those.
    ///
    /// # Errors
    ///
    /// A buffer under two bytes, no reply or a late one, a reply from another
    /// slave, a bad CRC, and what the port gives.
    pub fn reply(&mut self, reply: &mut [u8]) -> Result<()> {
        if reply.len() < 2 {
            return Err(Error::arg("a Modbus reply buffer under two bytes"));
        }
        let mut crc = self.read_begin(&mut reply[0])?;
        let wanted = if reply[0] & EXCEPTION == 0 { reply.len() } else { 2 };
        let mut filled = 1;
        let mut last = Instant::now();
        while filled < wanted {
            let left = READ_TIMEOUT.saturating_sub(last.elapsed());
            let n = match self.port.read(&mut reply[filled..wanted], left) {
                Ok(n) => n,
                Err(e) => {
                    log::error!("Incompletely read Modbus response.");
                    return Err(e);
                }
            };
            if n > 0 {
                crc = crc16_update(crc, &reply[filled..filled + n]);
                filled += n;
                last = Instant::now();
            } else if last.elapsed() >= READ_TIMEOUT {
                log::error!("Timed out waiting for Modbus response.");
                return Err(failed("timed out"));
            }
        }
        self.read_end(crc)
    }

    /// The address and the function code: the CRC so far.
    fn read_begin(&mut self, function: &mut u8) -> Result<u16> {
        let mut slave = [0];
        if read_blocking(self.port.as_mut(), &mut slave, FIRST_BYTE)? != 1 {
            return Err(failed("nothing came"));
        }
        if slave[0] != self.slave {
            return Err(failed("from another slave"));
        }
        let code = std::slice::from_mut(function);
        if read_blocking(self.port.as_mut(), code, FUNCTION_CODE)? != 1 {
            return Err(failed("no function code"));
        }
        Ok(crc16_update(crc16(&slave), code))
    }

    fn read_end(&mut self, crc: u16) -> Result<()> {
        let mut got = [0; 2];
        if read_blocking(self.port.as_mut(), &mut got, CRC_BYTES)? != 2 {
            return Err(failed("no CRC"));
        }
        let got = u16::from_le_bytes(got);
        if got != crc {
            log::error!("CRC error (0x{got:04X} vs 0x{crc:04X}).");
            return Err(invalid_data());
        }
        Ok(())
    }

    /// Sends a request and reads its reply: `sr_modbus_request_reply`.
    ///
    /// # Errors
    ///
    /// As [`Modbus::request`] and [`Modbus::reply`].
    pub fn request_reply(&mut self, pdu: &[u8], reply: &mut [u8]) -> Result<()> {
        self.request(pdu)?;
        self.reply(reply)
    }

    /// Reads `count` coils from `address` into `coils`, one bit each, as many
    /// bytes as that takes: `sr_modbus_read_coils`.
    ///
    /// # Errors
    ///
    /// A count outside 1 to 2000 or one `coils` cannot hold, an exception or
    /// a reply that is not to this request, and what [`Modbus::request_reply`]
    /// gives.
    pub fn read_coils(&mut self, address: u16, count: usize, coils: &mut [u8]) -> Result<()> {
        if !(1..=MAX_READ_COILS).contains(&count) {
            return Err(Error::arg("a count of coils outside 1 to 2000"));
        }
        let bytes = count.div_ceil(8);
        if coils.len() < bytes {
            return Err(Error::arg("a buffer too small for the coils"));
        }
        let mut reply = vec![0; 2 + bytes];
        self.request_reply(&read_request(READ_COILS, address, count), &mut reply)?;
        if is_exception(&reply) {
            return Err(invalid_data());
        }
        if reply[0] != READ_COILS || usize::from(reply[1]) != bytes {
            return Err(invalid_data());
        }
        coils[..bytes].copy_from_slice(&reply[2..]);
        Ok(())
    }

    /// Asks for `count` holding registers from `address`, leaving the reply
    /// for [`Modbus::read_holding_reply`]: `sr_modbus_read_holding_registers`
    /// with no buffer.
    ///
    /// # Errors
    ///
    /// A count outside 1 to 125, and what [`Modbus::request`] gives.
    pub fn send_read_holding_registers(&mut self, address: u16, count: usize) -> Result<()> {
        check_read_registers(count)?;
        self.request(&read_request(READ_HOLDING_REGISTERS, address, count))
    }

    /// Reads the reply to a request for as many registers as `registers`
    /// holds: `sr_modbus_read_holding_registers` with an address of -1.
    ///
    /// # Errors
    ///
    /// A count outside 1 to 125, an exception or a reply that is not to such
    /// a request, and what [`Modbus::reply`] gives.
    pub fn read_holding_reply(&mut self, registers: &mut [u16]) -> Result<()> {
        check_read_registers(registers.len())?;
        let bytes = registers.len() * 2;
        let mut reply = vec![0; 2 + bytes];
        self.reply(&mut reply)?;
        if is_exception(&reply) {
            return Err(invalid_data());
        }
        // Up to 250, which fits.
        if reply[0] != READ_HOLDING_REGISTERS || usize::from(reply[1]) != bytes {
            return Err(invalid_data());
        }
        let (pairs, _) = reply[2..].as_chunks::<2>();
        for (register, pair) in registers.iter_mut().zip(pairs) {
            *register = u16::from_be_bytes(*pair);
        }
        Ok(())
    }

    /// Reads as many holding registers as `registers` holds, from `address`:
    /// `sr_modbus_read_holding_registers`.
    ///
    /// # Errors
    ///
    /// As [`Modbus::send_read_holding_registers`] and
    /// [`Modbus::read_holding_reply`].
    pub fn read_holding_registers(&mut self, address: u16, registers: &mut [u16]) -> Result<()> {
        self.send_read_holding_registers(address, registers.len())?;
        self.read_holding_reply(registers)
    }

    /// Writes a coil: `sr_modbus_write_coil`.
    ///
    /// # Errors
    ///
    /// An exception or an answer that is not the request's echo, and what
    /// [`Modbus::request_reply`] gives.
    pub fn write_coil(&mut self, address: u16, value: bool) -> Result<()> {
        let [ah, al] = address.to_be_bytes();
        let [vh, vl] = (if value { 0xff00u16 } else { 0 }).to_be_bytes();
        let request = [WRITE_COIL, ah, al, vh, vl];
        let mut reply = [0; 5];
        self.request_reply(&request, &mut reply)?;
        if is_exception(&reply) || reply != request {
            return Err(invalid_data());
        }
        Ok(())
    }

    /// Writes `registers` from `address`: `sr_modbus_write_multiple_registers`.
    ///
    /// # Errors
    ///
    /// A count outside 1 to 123, an exception or an answer that does not
    /// repeat the address and the count, and what [`Modbus::request_reply`]
    /// gives.
    pub fn write_multiple_registers(&mut self, address: u16, registers: &[u16]) -> Result<()> {
        if !(1..=MAX_WRITE_REGISTERS).contains(&registers.len()) {
            return Err(Error::arg("a count of registers outside 1 to 123"));
        }
        let mut request = read_request(WRITE_MULTIPLE_REGISTERS, address, registers.len()).to_vec();
        // Up to 246, which fits.
        request.push((registers.len() * 2) as u8);
        for register in registers {
            request.extend_from_slice(&register.to_be_bytes());
        }
        let mut reply = [0; 5];
        self.request_reply(&request, &mut reply)?;
        if is_exception(&reply) || reply[..] != request[..5] {
            return Err(invalid_data());
        }
        Ok(())
    }
}

fn check_read_registers(count: usize) -> Result<()> {
    if (1..=MAX_READ_REGISTERS).contains(&count) {
        Ok(())
    } else {
        Err(Error::arg("a count of registers outside 1 to 125"))
    }
}

/// A function code, an address and a count, each of 16 bits big endian.
fn read_request(function: u8, address: u16, count: usize) -> [u8; 5] {
    let [ah, al] = address.to_be_bytes();
    // Callers hold it to 2000.
    let [ch, cl] = (count as u16).to_be_bytes();
    [function, ah, al, ch, cl]
}

/// What a Modbus driver's scan is given: `sr_modbus_scan`'s options.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ScanOptions {
    /// `conn=`, without which there is no device: no Modbus transport scans.
    pub conn: Option<String>,
    /// `serialcomm=`, or the driver's default.
    pub serialcomm: String,
    /// `modbusaddr=`, or 1.
    pub slave: u8,
}

impl ScanOptions {
    /// The options of a scan, with the driver's default `serialcomm`. C keeps
    /// the low byte of `modbusaddr` and so talks to some other slave; here
    /// what no slave address holds is refused.
    ///
    /// # Errors
    ///
    /// A value of the wrong type, a `modbusaddr` over 255, or another key.
    pub fn parse(options: &[(Key, Value)], serialcomm: &str) -> Result<ScanOptions> {
        let mut parsed =
            ScanOptions { conn: None, serialcomm: serialcomm.to_owned(), slave: DEFAULT_SLAVE };
        for (key, value) in options {
            match key {
                Key::Conn => parsed.conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => {
                    value
                        .as_str()
                        .ok_or_else(|| Error::arg("serialcomm"))?
                        .clone_into(&mut parsed.serialcomm);
                }
                Key::Modbusaddr => {
                    parsed.slave = value
                        .as_u64()
                        .and_then(|n| u8::try_from(n).ok())
                        .ok_or_else(|| Error::arg("modbusaddr must be 0 to 255"))?;
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        Ok(parsed)
    }

    /// `serialcomm` as parameters.
    ///
    /// # Errors
    ///
    /// When it does not parse.
    pub fn params(&self) -> Result<Params> {
        Params::parse(&self.serialcomm)
    }
}

/// Opens the slave at `conn`: `sr_modbus_open`, which is `serial_open`.
///
/// # Errors
///
/// What the port gives.
pub fn open(conn: &str, params: Params, slave: u8) -> Result<Modbus> {
    Ok(Modbus::new(serial::open(conn, params)?, slave))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    /// A slave's side, as a port.
    #[derive(Default)]
    struct Wire {
        written: Vec<Vec<u8>>,
        input: VecDeque<u8>,
        purges: usize,
        /// Taken for each write: what comes in reply.
        answers: VecDeque<Vec<u8>>,
    }

    struct Fake(Arc<Mutex<Wire>>);

    impl Port for Fake {
        fn name(&self) -> &'static str {
            "fake"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            let mut wire = self.0.lock().unwrap();
            if wire.input.is_empty() {
                drop(wire);
                // A little of the wait, not all: tests of a silence stay short.
                std::thread::sleep(timeout.min(Duration::from_millis(2)));
                return Ok(0);
            }
            let n = buf.len().min(wire.input.len());
            for b in &mut buf[..n] {
                *b = wire.input.pop_front().expect("counted");
            }
            Ok(n)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let mut wire = self.0.lock().unwrap();
            wire.written.push(data.to_vec());
            if let Some(answer) = wire.answers.pop_front() {
                wire.input.extend(answer);
            }
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            let mut wire = self.0.lock().unwrap();
            wire.purges += 1;
            wire.input.clear();
            Ok(())
        }
    }

    fn on(answers: Vec<Vec<u8>>) -> (Modbus, Arc<Mutex<Wire>>) {
        let wire = Arc::new(Mutex::new(Wire { answers: answers.into(), ..Wire::default() }));
        (Modbus::new(Box::new(Fake(Arc::clone(&wire))), 1), wire)
    }

    #[test]
    fn the_crc_is_modbus_low_byte_first() {
        assert_eq!(crc16(&[1, 3, 0, 0, 0, 1]), 0x0a84);
        assert_eq!(frame(1, &[3, 0, 0, 0, 1]), [1, 3, 0, 0, 0, 1, 0x84, 0x0a]);
    }

    #[test]
    fn registers_are_big_endian_on_the_wire() {
        let (mut m, wire) = on(vec![frame(1, &[3, 4, 0x12, 0x34, 0xab, 0xcd])]);
        let mut r = [0; 2];
        m.read_holding_registers(0x0b, &mut r).unwrap();
        assert_eq!(r, [0x1234, 0xabcd]);
        assert_eq!(wire.lock().unwrap().written, [frame(1, &[3, 0, 0x0b, 0, 2])]);
    }

    #[test]
    fn a_read_may_be_asked_and_answered_apart() {
        let (mut m, wire) = on(vec![frame(1, &[3, 2, 0, 7])]);
        m.send_read_holding_registers(5, 1).unwrap();
        assert_eq!(wire.lock().unwrap().written.len(), 1);
        let mut r = [0; 1];
        m.read_holding_reply(&mut r).unwrap();
        assert_eq!(r, [7]);
        assert_eq!(wire.lock().unwrap().written.len(), 1, "the reply alone sends nothing");
    }

    #[test]
    fn a_write_of_registers_has_its_frame_and_its_echo() {
        let (mut m, wire) = on(vec![frame(1, &[0x10, 0, 9, 0, 1])]);
        m.write_multiple_registers(9, &[0x0102]).unwrap();
        assert_eq!(wire.lock().unwrap().written, [frame(1, &[0x10, 0, 9, 0, 1, 2, 1, 2])]);
    }

    #[test]
    fn a_write_with_a_wrong_echo_fails() {
        let (mut m, _) = on(vec![frame(1, &[0x10, 0, 8, 0, 1])]);
        assert!(m.write_multiple_registers(9, &[1]).is_err());
    }

    #[test]
    fn coils_are_read_and_written() {
        let (mut m, wire) = on(vec![frame(1, &[1, 2, 0xa5, 0x01]), frame(1, &[5, 0, 3, 0xff, 0])]);
        let mut c = [0; 2];
        m.read_coils(3, 9, &mut c).unwrap();
        assert_eq!(c, [0xa5, 0x01]);
        m.write_coil(3, true).unwrap();
        let written = wire.lock().unwrap().written.clone();
        assert_eq!(written, [frame(1, &[1, 0, 3, 0, 9]), frame(1, &[5, 0, 3, 0xff, 0])]);
        let (mut m, wire) = on(vec![frame(1, &[5, 0, 3, 0, 0])]);
        m.write_coil(3, false).unwrap();
        assert_eq!(wire.lock().unwrap().written, [frame(1, &[5, 0, 3, 0, 0])]);
    }

    #[test]
    fn a_coil_that_is_not_echoed_fails() {
        let (mut m, _) = on(vec![frame(1, &[5, 0, 3, 0, 0])]);
        assert!(m.write_coil(3, true).is_err());
    }

    #[test]
    fn an_exception_is_two_bytes_and_a_failure() {
        for (pdu, name) in [(0x83u8, "reading"), (0x90, "writing")] {
            let (mut m, wire) = on(vec![frame(1, &[pdu, 2])]);
            let got = if pdu == 0x83 {
                m.read_holding_registers(0, &mut [0; 4])
            } else {
                m.write_multiple_registers(0, &[1])
            };
            assert!(got.is_err(), "{name}");
            assert!(wire.lock().unwrap().input.is_empty(), "{name}: all of it was read");
        }
        let (mut m, _) = on(vec![frame(1, &[0x81, 2])]);
        assert!(m.read_coils(0, 8, &mut [0; 1]).is_err());
        let (mut m, _) = on(vec![frame(1, &[0x85, 2])]);
        assert!(m.write_coil(0, true).is_err());
    }

    #[test]
    fn every_exception_is_named() {
        assert!(is_exception(&[0x83, 0x42]));
        assert!(!is_exception(&[0x03, 0x02]));
    }

    #[test]
    fn a_bad_crc_fails() {
        let mut f = frame(1, &[3, 2, 0, 7]);
        *f.last_mut().unwrap() ^= 1;
        let (mut m, _) = on(vec![f]);
        assert!(m.read_holding_registers(0, &mut [0; 1]).is_err());
    }

    #[test]
    fn the_crc_covers_the_address() {
        let mut f = frame(2, &[3, 2, 0, 7]);
        f[0] = 1;
        let (mut m, _) = on(vec![f]);
        assert!(m.read_holding_registers(0, &mut [0; 1]).is_err());
    }

    #[test]
    fn a_reply_from_another_slave_fails() {
        let (mut m, wire) = on(vec![frame(2, &[3, 2, 0, 7])]);
        assert!(m.read_holding_registers(0, &mut [0; 1]).is_err());
        assert_eq!(wire.lock().unwrap().input.len(), 6, "one byte was read, as in C");
    }

    #[test]
    fn a_reply_with_the_wrong_function_or_count_fails() {
        let (mut m, _) = on(vec![frame(1, &[4, 2, 0, 7])]);
        assert!(m.read_holding_registers(0, &mut [0; 1]).is_err());
        let (mut m, _) = on(vec![frame(1, &[3, 4, 0, 7])]);
        assert!(m.read_holding_registers(0, &mut [0; 1]).is_err());
        let (mut m, _) = on(vec![frame(1, &[2, 1, 0])]);
        assert!(m.read_coils(0, 1, &mut [0; 1]).is_err());
    }

    #[test]
    fn a_request_clears_what_a_bad_reply_left() {
        let mut too_long = frame(1, &[3, 2, 0, 7]);
        too_long.extend_from_slice(&[0x55, 0xaa]);
        let (mut m, wire) = on(vec![too_long, frame(1, &[3, 2, 0, 8])]);
        let mut r = [0; 1];
        m.read_holding_registers(0, &mut r).unwrap();
        assert_eq!(r, [7]);
        m.read_holding_registers(0, &mut r).unwrap();
        assert_eq!(r, [8], "the two bytes left over are gone");
        assert_eq!(wire.lock().unwrap().purges, 2);
    }

    #[test]
    fn a_reply_alone_clears_nothing() {
        let (mut m, wire) = on(vec![]);
        wire.lock().unwrap().input.extend(frame(1, &[3, 2, 0, 9]));
        let mut r = [0; 1];
        m.read_holding_reply(&mut r).unwrap();
        assert_eq!(r, [9]);
        assert_eq!(wire.lock().unwrap().purges, 0);
    }

    #[test]
    fn silence_fails_after_its_time() {
        let (mut m, _) = on(vec![]);
        let started = Instant::now();
        assert!(m.read_holding_registers(0, &mut [0; 1]).is_err());
        let took = started.elapsed();
        assert!(took >= FIRST_BYTE, "{took:?}");
        assert!(took < READ_TIMEOUT, "{took:?}");
    }

    #[test]
    fn a_reply_cut_short_fails_a_second_after_its_last_byte() {
        let mut f = frame(1, &[3, 4, 0, 7, 0, 8]);
        f.truncate(4);
        let (mut m, _) = on(vec![f]);
        let started = Instant::now();
        assert!(m.read_holding_registers(0, &mut [0; 2]).is_err());
        let took = started.elapsed();
        assert!(took >= READ_TIMEOUT, "{took:?}");
    }

    #[test]
    fn a_request_that_cannot_go_out_says_so() {
        struct NoWrite;
        impl Port for NoWrite {
            fn name(&self) -> &'static str {
                "no write"
            }
            fn set_params(&mut self, _: Params) -> Result<()> {
                Ok(())
            }
            fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
                Ok(())
            }
            fn read(&mut self, _: &mut [u8], _: Duration) -> Result<usize> {
                Ok(0)
            }
            fn write(&mut self, _: &[u8]) -> Result<()> {
                Err(Error::device("Broken pipe"))
            }
            fn purge(&mut self) -> Result<()> {
                Ok(())
            }
        }
        let mut m = Modbus::new(Box::new(NoWrite), 1);
        assert!(!m.request_unsent());
        assert!(m.request(&[3]).is_err());
        assert!(m.request_unsent());
        let (mut m, _) = on(vec![]);
        assert!(m.read_holding_registers(0, &mut [0; 1]).is_err());
        assert!(!m.request_unsent(), "a silence is not that");
    }

    #[test]
    fn counts_are_held_to_what_modbus_has() {
        let (mut m, wire) = on(vec![]);
        assert!(matches!(m.read_holding_registers(0, &mut []), Err(Error::Arg(_))));
        assert!(matches!(m.read_holding_registers(0, &mut [0; 126]), Err(Error::Arg(_))));
        assert!(matches!(m.write_multiple_registers(0, &[]), Err(Error::Arg(_))));
        assert!(matches!(m.write_multiple_registers(0, &[0; 124]), Err(Error::Arg(_))));
        assert!(matches!(m.read_coils(0, 0, &mut [0; 1]), Err(Error::Arg(_))));
        assert!(matches!(m.read_coils(0, 2001, &mut [0; 300]), Err(Error::Arg(_))));
        assert!(matches!(m.read_coils(0, 9, &mut [0; 1]), Err(Error::Arg(_))));
        assert!(matches!(m.request(&[]), Err(Error::Arg(_))));
        assert!(matches!(m.reply(&mut [0; 1]), Err(Error::Arg(_))));
        assert_eq!(wire.lock().unwrap().written.len(), 0);
    }

    #[test]
    fn the_scan_options_have_their_defaults() {
        let o = ScanOptions::parse(&[], "9600/8n1").unwrap();
        assert_eq!(
            o,
            ScanOptions { conn: None, serialcomm: "9600/8n1".to_owned(), slave: DEFAULT_SLAVE }
        );
        let given = [
            (Key::Conn, Value::from("/dev/x")),
            (Key::Serialcomm, Value::from("19200/8n1")),
            (Key::Modbusaddr, Value::U64(7)),
        ];
        let o = ScanOptions::parse(&given, "9600/8n1").unwrap();
        assert_eq!(o.conn.as_deref(), Some("/dev/x"));
        assert_eq!(o.serialcomm, "19200/8n1");
        assert_eq!(o.slave, 7);
    }

    #[test]
    fn a_modbusaddr_no_slave_has_is_refused() {
        let given = [(Key::Modbusaddr, Value::U64(257))];
        assert!(matches!(ScanOptions::parse(&given, "9600/8n1"), Err(Error::Arg(_))));
    }
}
