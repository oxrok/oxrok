// Counterpart in sigrok:
//   libftdi1 (ftdi.c), by Intra2net AG and the libftdi developers.
// This file is written for oxrok from FTDI's application notes; libftdi was
//   the reference for which requests a driver needs.

//! The FTDI vendor protocol for FT232/FT245 class chips, on top of
//! [`crate::usb::Transport`].
//!
//! Written from FTDI's public application notes (`AN_120` for the baud
//! rate encoding, the D2XX programmer's guide for the request numbers
//! and the bit-bang modes). It does what oxrok's drivers need: reset,
//! purge, latency timer, baud rate, bit-bang mode, pin readback, modem
//! status, EEPROM reads, and FIFO reads that strip the two status bytes
//! of every USB packet. EEPROM writes are not implemented on purpose.

use crate::core::{Error, Result};
use crate::usb::{ControlRequest, Transport};
use std::time::Duration;

pub mod libftdi;

/// Vendor request numbers.
mod req {
    pub const RESET: u8 = 0x00;
    pub const MODEM_CTRL: u8 = 0x01;
    pub const SET_FLOW_CTRL: u8 = 0x02;
    pub const SET_BAUD_RATE: u8 = 0x03;
    pub const SET_DATA: u8 = 0x04;
    pub const POLL_MODEM_STATUS: u8 = 0x05;
    pub const SET_LATENCY_TIMER: u8 = 0x09;
    pub const GET_LATENCY_TIMER: u8 = 0x0a;
    pub const SET_BITMODE: u8 = 0x0b;
    pub const READ_PINS: u8 = 0x0c;
    pub const READ_EEPROM: u8 = 0x90;
}

/// FTDI's vendor id.
pub const VENDOR_ID: u16 = 0x0403;

/// The chip family, from `bcdDevice`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChipType {
    /// FT232AM / FT245AM.
    Am,
    /// FT232BM / FT245BM / FT232BL / FT245BL.
    Bm,
    /// FT2232C / FT2232D.
    Ft2232c,
    /// FT232R / FT245R.
    R,
    /// FT2232H.
    Ft2232h,
    /// FT4232H.
    Ft4232h,
    /// FT232H.
    Ft232h,
    /// FT230X / FT231X / FT234X.
    Ft230x,
    /// Not recognised; treated like a BM for the baud rate.
    Unknown(u16),
}

impl ChipType {
    /// Classifies a `bcdDevice` value.
    pub fn from_device_version(v: u16) -> Self {
        match v {
            0x0200 => ChipType::Am,
            0x0400 => ChipType::Bm,
            0x0500 => ChipType::Ft2232c,
            0x0600 => ChipType::R,
            0x0700 => ChipType::Ft2232h,
            0x0800 => ChipType::Ft4232h,
            0x0900 => ChipType::Ft232h,
            0x1000 => ChipType::Ft230x,
            other => ChipType::Unknown(other),
        }
    }

    /// The name as FTDI writes it.
    pub fn name(self) -> String {
        match self {
            ChipType::Am => "FT232AM/FT245AM".into(),
            ChipType::Bm => "FT232BM/FT245BM".into(),
            ChipType::Ft2232c => "FT2232C".into(),
            ChipType::R => "FT232R/FT245R".into(),
            ChipType::Ft2232h => "FT2232H".into(),
            ChipType::Ft4232h => "FT4232H".into(),
            ChipType::Ft232h => "FT232H".into(),
            ChipType::Ft230x => "FT230X".into(),
            ChipType::Unknown(v) => format!("unknown (bcdDevice {v:04x})"),
        }
    }

    /// True for the chips with more than one port.
    pub fn multi_port(self) -> bool {
        matches!(self, ChipType::Ft2232c | ChipType::Ft2232h | ChipType::Ft4232h)
    }

    /// True for the high speed chips with the 120 MHz clock.
    pub fn high_speed(self) -> bool {
        matches!(self, ChipType::Ft2232h | ChipType::Ft4232h | ChipType::Ft232h)
    }

    /// EEPROM size in 16 bit words.
    pub fn eeprom_words(self) -> usize {
        match self {
            ChipType::Ft2232h | ChipType::Ft4232h | ChipType::Ft232h => 128,
            _ => 64,
        }
    }
}

/// Which port of a multi port chip.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    /// The first (or only) port.
    A,
    /// The second port.
    B,
    /// The third port of an FT4232H.
    C,
    /// The fourth port of an FT4232H.
    D,
}

impl Port {
    fn interface(self) -> u8 {
        match self {
            Port::A => 0,
            Port::B => 1,
            Port::C => 2,
            Port::D => 3,
        }
    }

    /// `wIndex` of the vendor requests: 0 on single port chips, else
    /// the port number from 1.
    fn index(self, chip: ChipType) -> u16 {
        if chip.multi_port() { u16::from(self.interface()) + 1 } else { 0 }
    }

    fn in_endpoint(self) -> u8 {
        0x81 + 2 * self.interface()
    }

    fn out_endpoint(self) -> u8 {
        0x02 + 2 * self.interface()
    }
}

/// The bit modes of `SET_BITMODE`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BitMode {
    /// Back to the UART or FIFO function.
    Reset = 0x00,
    /// Asynchronous bit-bang.
    BitBang = 0x01,
    /// MPSSE (FT2232 and later).
    Mpsse = 0x02,
    /// Synchronous bit-bang.
    SyncBitBang = 0x04,
    /// MCU host bus emulation.
    Mcu = 0x08,
    /// Fast opto-isolated serial.
    Opto = 0x10,
    /// CBUS bit-bang.
    Cbus = 0x20,
    /// Synchronous FIFO (FT2232H, FT232H).
    SyncFifo = 0x40,
}

/// Computes the `SET_BAUD_RATE` value and index for `baud` on `chip`.
///
/// Returns the fields and the baud rate the divisor really gives. The
/// base clock is 3 MHz; the H chips switch to 12 MHz above 1.2 `MBaud`.
/// Divisors 1 and 1.5 have their own codes (0 and 1); from 2 up the
/// divisor is 14 integer bits and a 3 bit code for the eighths, of
/// which the AM chips only have 0, 1/8, 1/4 and 1/2.
pub fn baud_divisor(chip: ChipType, baud: u32) -> Result<(u16, u16, u32)> {
    const FRAC_CODE: [u32; 8] = [0, 3, 2, 4, 1, 5, 6, 7];
    if baud == 0 {
        return Err(Error::arg("baud rate 0"));
    }
    let high = chip.high_speed() && baud > 1_200_000;
    let clock: u64 = if high { 12_000_000 } else { 3_000_000 };
    let baud64 = u64::from(baud);
    let (encoded, actual) = if baud64 * 5 >= clock * 4 {
        (0u32, clock)
    } else if baud64 * 7 >= clock * 4 {
        (1, clock * 2 / 3)
    } else {
        let mut d8 = ((clock * 8 + baud64 / 2) / baud64).min(0x3ffff) as u32;
        if chip == ChipType::Am {
            d8 = match d8 & 7 {
                0 | 1 => d8,
                2 | 3 => (d8 & !7) | 2,
                _ => (d8 & !7) | 4,
            };
        }
        let int = d8 >> 3;
        let frac = d8 & 7;
        (int | (FRAC_CODE[frac as usize] << 14), clock * 8 / u64::from(d8))
    };
    let mut index = ((encoded >> 16) & 1) as u16;
    if high {
        // Bit 9 of the index selects the 120 MHz clock.
        index |= 0x0200;
    }
    Ok(((encoded & 0xffff) as u16, index, actual as u32))
}

/// An FTDI port.
pub struct Ftdi {
    t: Box<dyn Transport>,
    chip: ChipType,
    port: Port,
    max_packet: usize,
    leftover: Vec<u8>,
    read_chunk: usize,
    /// Timeout of one control request.
    pub control_timeout: Duration,
    /// Timeout of one bulk read.
    pub read_timeout: Duration,
    /// Timeout of one bulk write.
    pub write_timeout: Duration,
}

impl Ftdi {
    /// Claims `port` on `transport`. The chip type comes from the
    /// device descriptor.
    pub fn open(mut transport: Box<dyn Transport>, port: Port) -> Result<Self> {
        let chip = ChipType::from_device_version(transport.info().device_version);
        transport.claim_interface(port.interface())?;
        let max_packet = transport.max_packet_size(port.in_endpoint())?.max(3);
        Ok(Ftdi {
            t: transport,
            chip,
            port,
            max_packet,
            leftover: Vec::new(),
            read_chunk: 4096,
            control_timeout: Duration::from_secs(1),
            read_timeout: Duration::from_secs(1),
            write_timeout: Duration::from_secs(1),
        })
    }

    /// Gives the transport back, e.g. to collect a recording.
    pub fn into_transport(self) -> Box<dyn Transport> {
        self.t
    }

    /// The transport.
    pub fn transport(&mut self) -> &mut dyn Transport {
        self.t.as_mut()
    }

    /// The chip family.
    pub fn chip(&self) -> ChipType {
        self.chip
    }

    /// `wMaxPacketSize` of the IN endpoint.
    pub fn max_packet_size(&self) -> usize {
        self.max_packet
    }

    fn index(&self) -> u16 {
        self.port.index(self.chip)
    }

    fn out(&mut self, request: u8, value: u16, index: u16) -> Result<()> {
        self.t.control_out(ControlRequest::vendor(request, value, index), &[], self.control_timeout)
    }

    fn inp(&mut self, request: u8, value: u16, index: u16, len: usize) -> Result<Vec<u8>> {
        let d = self.t.control_in(
            ControlRequest::vendor(request, value, index),
            len,
            self.control_timeout,
        )?;
        if d.len() < len {
            return Err(Error::device(format!("short answer to request 0x{request:02x}")));
        }
        Ok(d)
    }

    /// Resets the port (`SIO_RESET`) and drops buffered reads.
    pub fn reset(&mut self) -> Result<()> {
        self.leftover.clear();
        let index = self.index();
        self.out(req::RESET, 0, index)
    }

    /// Drops the data the chip holds for the host (its "TX" buffer in
    /// FTDI's naming) and anything buffered here.
    pub fn purge_read(&mut self) -> Result<()> {
        self.leftover.clear();
        let index = self.index();
        self.out(req::RESET, 2, index)
    }

    /// Drops the data the chip holds from the host (its "RX" buffer).
    pub fn purge_write(&mut self) -> Result<()> {
        let index = self.index();
        self.out(req::RESET, 1, index)
    }

    /// Sets the latency timer, 1 to 255 ms: how long the chip waits
    /// before sending a partial packet.
    pub fn set_latency_timer(&mut self, ms: u8) -> Result<()> {
        if ms == 0 {
            return Err(Error::arg("latency timer must be 1 to 255"));
        }
        let index = self.index();
        self.out(req::SET_LATENCY_TIMER, u16::from(ms), index)
    }

    /// Reads the latency timer back.
    pub fn latency_timer(&mut self) -> Result<u8> {
        let index = self.index();
        Ok(self.inp(req::GET_LATENCY_TIMER, 0, index, 1)?[0])
    }

    /// Sets the baud rate, which also clocks the bit-bang modes.
    /// Returns the rate the divisor really gives.
    pub fn set_baudrate(&mut self, baud: u32) -> Result<u32> {
        let (value, mut index, actual) = baud_divisor(self.chip, baud)?;
        if self.chip.multi_port() {
            index = (index & 0xff) | (self.index() << 8);
        }
        self.out(req::SET_BAUD_RATE, value, index)?;
        Ok(actual)
    }

    /// Line settings: data bits, stop bits code (0 = 1, 1 = 1.5, 2 = 2)
    /// and parity code (0 none, 1 odd, 2 even, 3 mark, 4 space).
    pub fn set_line(&mut self, bits: u8, stop: u8, parity: u8) -> Result<()> {
        let index = self.index();
        let value = u16::from(bits) | (u16::from(parity) << 8) | (u16::from(stop) << 11);
        self.out(req::SET_DATA, value, index)
    }

    /// Hardware flow control off.
    pub fn set_flow_control_none(&mut self) -> Result<()> {
        let index = self.index();
        self.out(req::SET_FLOW_CTRL, 0, index)
    }

    /// DTR and RTS.
    pub fn set_dtr_rts(&mut self, dtr: bool, rts: bool) -> Result<()> {
        let index = self.index();
        let value = 0x0100 | u16::from(dtr) | 0x0200 | (u16::from(rts) << 1);
        self.out(req::MODEM_CTRL, value, index)
    }

    /// The modem status word (CTS, DSR, RI, DCD and the line status).
    pub fn modem_status(&mut self) -> Result<u16> {
        let index = self.index();
        let d = self.inp(req::POLL_MODEM_STATUS, 0, index, 2)?;
        Ok(u16::from_le_bytes([d[0], d[1]]))
    }

    /// Enters a bit mode with `mask` selecting the output pins.
    pub fn set_bitmode(&mut self, mask: u8, mode: BitMode) -> Result<()> {
        let index = self.index();
        self.out(req::SET_BITMODE, (u16::from(mode as u8) << 8) | u16::from(mask), index)
    }

    /// The level of the eight data pins right now.
    pub fn read_pins(&mut self) -> Result<u8> {
        let index = self.index();
        Ok(self.inp(req::READ_PINS, 0, index, 1)?[0])
    }

    /// Reads `words` 16 bit words of the EEPROM, little endian. The
    /// EEPROM is never written by this crate.
    pub fn read_eeprom(&mut self, words: usize) -> Result<Vec<u8>> {
        let mut out = Vec::with_capacity(words * 2);
        for addr in 0..words {
            let d = self.inp(req::READ_EEPROM, 0, addr as u16, 2)?;
            out.extend_from_slice(&d[..2]);
        }
        Ok(out)
    }

    /// Writes `data` to the port, in chunks the chip's buffer takes.
    pub fn write(&mut self, data: &[u8]) -> Result<()> {
        let ep = self.port.out_endpoint();
        for chunk in data.chunks(4096) {
            let n = self.t.bulk_out(ep, chunk, self.write_timeout)?;
            if n != chunk.len() {
                return Err(Error::device(format!("short write ({n} of {} bytes)", chunk.len())));
            }
        }
        Ok(())
    }

    /// Reads what the chip has for the host into `buf`, after
    /// stripping the two status bytes of every USB packet. Returns the
    /// byte count, 0 when nothing arrived within the read timeout.
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        if self.leftover.is_empty() {
            let ep = self.port.in_endpoint();
            let want = self.read_chunk.max(self.max_packet);
            let raw = match self.t.bulk_in(ep, want, self.read_timeout) {
                Ok(raw) => raw,
                Err(Error::Timeout) => return Ok(0),
                Err(e) => return Err(e),
            };
            self.leftover = strip_status(&raw, self.max_packet);
        }
        let n = buf.len().min(self.leftover.len());
        buf[..n].copy_from_slice(&self.leftover[..n]);
        self.leftover.drain(..n);
        Ok(n)
    }

    /// Reads exactly `buf.len()` bytes or fails with a timeout after
    /// `timeout` without progress.
    pub fn read_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<()> {
        let mut got = 0;
        let mut last = std::time::Instant::now();
        while got < buf.len() {
            let n = self.read(&mut buf[got..])?;
            got += n;
            if n > 0 {
                last = std::time::Instant::now();
            } else if last.elapsed() > timeout {
                log::debug!("read timeout ({got} of {} bytes)", buf.len());
                return Err(Error::Timeout);
            }
        }
        Ok(())
    }
}

/// Removes the two status bytes that start every USB packet of
/// `max_packet` bytes.
fn strip_status(raw: &[u8], max_packet: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(raw.len());
    for packet in raw.chunks(max_packet) {
        if packet.len() > 2 {
            out.extend_from_slice(&packet[2..]);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chip_types() {
        assert_eq!(ChipType::from_device_version(0x0400), ChipType::Bm);
        assert_eq!(ChipType::from_device_version(0x0600), ChipType::R);
        assert!(ChipType::Ft2232h.high_speed());
        assert!(ChipType::Ft2232c.multi_port());
        assert_eq!(ChipType::Bm.eeprom_words(), 64);
        assert_eq!(Port::B.index(ChipType::Ft2232h), 2);
        assert_eq!(Port::A.index(ChipType::Bm), 0);
        assert_eq!(Port::B.in_endpoint(), 0x83);
        assert_eq!(Port::B.out_endpoint(), 0x04);
    }

    #[test]
    fn baud_divisors_match_the_application_note() {
        // (chip, baud, value, index, actual)
        let cases = [
            (ChipType::Bm, 460_800, 0x4006, 0, 461_538),
            (ChipType::Bm, 115_200, 0x001a, 0, 115_384),
            (ChipType::Bm, 9_600, 0x4138, 0, 9_600),
            (ChipType::Bm, 1_000_000, 0x0003, 0, 1_000_000),
            (ChipType::Bm, 2_000_000, 0x0001, 0, 2_000_000),
            (ChipType::Bm, 3_000_000, 0x0000, 0, 3_000_000),
            (ChipType::Bm, 300, 0x2710, 0, 300),
            (ChipType::Am, 460_800, 0x4006, 0, 461_538),
            (ChipType::Ft2232h, 12_000_000, 0x0000, 0x0200, 12_000_000),
            (ChipType::Ft2232h, 8_000_000, 0x0001, 0x0200, 8_000_000),
            (ChipType::Ft2232h, 3_000_000, 0x0004, 0x0200, 3_000_000),
            (ChipType::Ft2232h, 1_000_000, 0x0003, 0, 1_000_000),
            (ChipType::Ft2232h, 115_200, 0x001a, 0, 115_384),
        ];
        for (chip, baud, value, index, actual) in cases {
            let got = baud_divisor(chip, baud).unwrap();
            assert_eq!(got, (value, index, actual), "{chip:?} {baud}");
        }
        assert!(baud_divisor(ChipType::Bm, 0).is_err());
    }

    #[test]
    fn status_bytes_are_stripped_per_packet() {
        let raw = [0x31, 0x60, 1, 2, 0x31, 0x60, 3];
        assert_eq!(strip_status(&raw, 4), [1, 2, 3]);
        assert_eq!(strip_status(&[0x31, 0x60], 64), Vec::<u8>::new());
    }
}
