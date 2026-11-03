// Counterpart in sigrok:
//   libftdi1 (ftdi.c), by Intra2net AG and the libftdi developers.
// This file is written for oxrok to send what libftdi 1.5 sends;
//   ftdi_to_clkbits, ftdi_to_clkbits_AM and ftdi_convert_baudrate are ported
//   (LGPL-2.1-only, taken under its section 3 as GPL-3.0-or-later).

//! An FTDI port (A, or B with [`Interface`]) driven the way libftdi 1.5
//! drives it, for ports of
//! libsigrok drivers that use libftdi (`ftdi-la`, and the other FTDI
//! drivers after it): the same requests in the same order, with the
//! same values, so the device sees under oxrok what it sees under
//! sigrok-cli, and the same error codes and messages.
//!
//! What differs from [`super::Ftdi`], which follows FTDI's application
//! notes:
//!
//! - every vendor request carries libftdi's port index, 1 for port A,
//!   also on single port chips (where the chip ignores it);
//! - the baud rate divisor is libftdi's (`ftdi_to_clkbits`, and
//!   `ftdi_to_clkbits_AM` on the AM chips, which lack the eighths 3, 5,
//!   6 and 7): the H chips use the 120 MHz clock from 733 Bd up, a rate
//!   more than 5 % from what the divisor gives is refused, and in a bit
//!   mode the rate is multiplied by 4 first, as `ftdi_set_baudrate`
//!   does;
//! - `SET_BAUD_RATE`'s `wIndex` is the divisor's bits 16 and up on the
//!   chips before the H ones, and on the H chips those bits in the high
//!   byte and the port index in the low one;
//! - a read ([`Context::read_data`]) is `ftdi_read_data`: bulk transfers
//!   of 4096 bytes until the buffer is full or a transfer brings no
//!   data, the rest of the last transfer kept for the next read, and a
//!   failed transfer failing the read with what it had gathered.
//!
//! Opening ([`Context::open`]) is `ftdi_usb_open_dev`: claim the port,
//! reset it, take the chip type from `bcdDevice`, the packet size from
//! the endpoint, then 9600 Bd. Closing is dropping it: libftdi sends
//! nothing either.
//!
//! One difference the device sees: libftdi detaches the kernel's
//! `ftdi_sio` from port A and never attaches it again, while oxrok's
//! USB layer gives the port back to the kernel when it lets go of it.
//! `ftdi_sio` then binds port A again and probes it (it reads the
//! latency timer, `GET_LATENCY_TIMER`, and sets it to 16 ms), requests
//! a run under sigrok-cli never causes; and the next open, by either
//! tool, detaches it again. The user gets the serial port back.

use super::{ChipType, req, strip_status};
use crate::core::{Error, Result};
use crate::usb::{ControlRequest, Transport};
use std::fmt;
use std::time::Duration;

/// `ftdi_init`'s `readbuffer_chunksize`.
const READ_CHUNK: usize = 4096;
/// `ftdi_init`'s `writebuffer_chunksize`.
const WRITE_CHUNK: usize = 4096;
/// The largest `readbuffer_chunksize` `ftdi_read_data_set_chunksize`
/// takes on Linux (`MAX_BULK_BUFFER_LENGTH`).
const MAX_READ_CHUNK: usize = 16384;

/// libftdi's `enum ftdi_interface`: the port of a multi-port chip that
/// `ftdi_set_interface` selects before the open.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Interface {
    /// Port A, libftdi's default.
    #[default]
    A,
    /// Port B, of the two and four port chips.
    B,
}

impl Interface {
    /// `ftdi->index`: the port in `wIndex`.
    fn index(self) -> u16 {
        match self {
            Interface::A => 1,
            Interface::B => 2,
        }
    }

    /// `ftdi->interface`: the USB interface number.
    fn number(self) -> u8 {
        match self {
            Interface::A => 0,
            Interface::B => 1,
        }
    }

    /// The bulk IN endpoint (libftdi's `out_ep`).
    fn in_ep(self) -> u8 {
        match self {
            Interface::A => 0x81,
            Interface::B => 0x83,
        }
    }

    /// The bulk OUT endpoint (libftdi's `in_ep`).
    fn out_ep(self) -> u8 {
        match self {
            Interface::A => 0x02,
            Interface::B => 0x04,
        }
    }
}
/// `ftdi_init`'s `usb_read_timeout` and `usb_write_timeout`.
const TIMEOUT: Duration = Duration::from_millis(5000);

/// A libftdi error: its return code and its error string
/// (`ftdi_get_error_string`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FtdiError {
    /// The negative code the libftdi call returns.
    pub code: i32,
    /// The message libftdi keeps for `ftdi_get_error_string`.
    pub message: &'static str,
}

impl FtdiError {
    fn new(code: i32, message: &'static str) -> Self {
        FtdiError { code, message }
    }
}

impl fmt::Display for FtdiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}): {}", self.code, self.message)
    }
}

impl From<FtdiError> for Error {
    fn from(e: FtdiError) -> Error {
        Error::Device(format!("libftdi error {e}"))
    }
}

/// The libusb error code an oxrok transport error stands for, as
/// libftdi passes `libusb_bulk_transfer`'s return on.
pub fn libusb_code(e: &Error) -> i32 {
    match e {
        Error::Timeout => -7,
        Error::Disconnected => -4,
        Error::Busy => -6,
        Error::Device(m) if m.contains("stall") => -9,
        Error::NotSupported(_) => -12,
        Error::Arg(_) => -2,
        _ => -1,
    }
}

/// libftdi's chip type: `bcdDevice`, except that a BM without a serial
/// number reports 0x0200, which libftdi takes for a BM when
/// `iSerialNumber` is 0 and for an AM otherwise.
pub fn chip_type(bcd_device: u16, serial_index: u8) -> ChipType {
    if bcd_device == 0x0200 && serial_index == 0 {
        ChipType::Bm
    } else {
        ChipType::from_device_version(bcd_device)
    }
}

/// `ftdi_to_clkbits`: the divisor code (17 bits) for `baud` from clock
/// `clk` divided by `div`, and the rate it gives.
fn clkbits(baud: u64, clk: u64, div: u64) -> (u32, u64) {
    const FRAC_CODE: [u32; 8] = [0, 3, 2, 4, 1, 5, 6, 7];
    if baud >= clk / div {
        (0, clk / div)
    } else if baud >= clk / (div + div / 2) {
        (1, clk / (div + div / 2))
    } else if baud >= clk / (2 * div) {
        (2, clk / (2 * div))
    } else {
        // Three fractional bits and one to round with.
        let divisor = clk * 16 / div / baud;
        let mut best = if divisor & 1 == 1 { divisor / 2 + 1 } else { divisor / 2 };
        if best > 0x20000 {
            best = 0x1ffff;
        }
        let b = clk * 16 / div / best;
        let b = if b & 1 == 1 { b / 2 + 1 } else { b / 2 };
        (((best >> 3) as u32) | (FRAC_CODE[(best & 7) as usize] << 14), b)
    }
}

/// `ftdi_to_clkbits_AM`: the divisor code (17 bits) for `baud` on an
/// AM chip, and the rate it gives, rounded to the nearest. The AM's
/// divisor of its 3 MHz base takes only the eighths 0, 1/8, 1/4 and
/// 1/2, no value from 1.125 to 1.875, and at most 0x3fff. libftdi
/// tries the divisor 24 MHz / `baud` gives, rounded down to an eighth
/// the AM has, and the one above it rounded up to one, and keeps the
/// closer.
fn clkbits_am(baud: u64) -> (u32, u64) {
    const FRAC_CODE: [u32; 8] = [0, 3, 2, 4, 1, 5, 6, 7];
    const ADJUST_UP: [u64; 8] = [0, 0, 0, 1, 0, 3, 2, 1];
    const ADJUST_DN: [u64; 8] = [0, 0, 0, 1, 0, 1, 2, 3];
    let mut divisor = 24_000_000 / baud;
    divisor -= ADJUST_DN[(divisor & 7) as usize];
    let (mut best_divisor, mut best_baud, mut best_diff) = (0, 0, 0);
    for i in 0..2 {
        let mut try_divisor = divisor + i;
        if try_divisor <= 8 {
            try_divisor = 8;
        } else if divisor < 16 {
            // libftdi tests divisor here, not try_divisor.
            try_divisor = 16;
        } else {
            try_divisor += ADJUST_UP[(try_divisor & 7) as usize];
            if try_divisor > 0x1fff8 {
                try_divisor = 0x1fff8;
            }
        }
        let estimate = (24_000_000 + try_divisor / 2) / try_divisor;
        let diff = estimate.abs_diff(baud);
        if i == 0 || diff < best_diff {
            (best_divisor, best_baud, best_diff) = (try_divisor, estimate, diff);
            if diff == 0 {
                break;
            }
        }
    }
    let mut encoded = ((best_divisor >> 3) as u32) | (FRAC_CODE[(best_divisor & 7) as usize] << 14);
    // 3 MBd and 2 MBd have codes of their own.
    if encoded == 1 {
        encoded = 0;
    } else if encoded == 0x4001 {
        encoded = 1;
    }
    (encoded, best_baud)
}

/// `ftdi_convert_baudrate` and the tolerance check of
/// `ftdi_set_baudrate`, for a rate already multiplied by 4 in a bit
/// mode: `wValue`, the divisor bits of `wIndex` (bits 16 and up of the
/// code, before the port goes in) and the rate the divisor gives.
///
/// # Errors
///
/// libftdi's: a rate of 0, or a rate more than 5 % from what the
/// divisor gives.
pub fn divisor(chip: ChipType, baud: u64) -> std::result::Result<(u16, u16, u64), FtdiError> {
    const H_CLK: u64 = 120_000_000;
    const C_CLK: u64 = 48_000_000;
    if baud == 0 {
        return Err(FtdiError::new(-1, "Silly baudrate <= 0."));
    }
    let (encoded, best) = match chip {
        ChipType::Ft2232h | ChipType::Ft4232h | ChipType::Ft232h => {
            if baud * 10 > H_CLK / 0x3fff {
                let (e, b) = clkbits(baud, H_CLK, 10);
                (e | 0x20000, b)
            } else {
                clkbits(baud, C_CLK, 16)
            }
        }
        ChipType::Am => clkbits_am(baud),
        _ => clkbits(baud, C_CLK, 16),
    };
    if best == 0 {
        return Err(FtdiError::new(-1, "Silly baudrate <= 0."));
    }
    let off = if best < baud { best * 21 < baud * 20 } else { baud * 21 < best * 20 };
    if best * 2 < baud || off {
        return Err(FtdiError::new(
            -1,
            "Unsupported baudrate. Note: bitbang baudrates are automatically multiplied by 4",
        ));
    }
    Ok(((encoded & 0xffff) as u16, (encoded >> 16) as u16, best))
}

/// A port of an FTDI chip (port A unless opened with
/// [`Context::open_interface`]), opened and driven as libftdi does.
pub struct Context {
    t: Box<dyn Transport>,
    chip: ChipType,
    max_packet: usize,
    bitbang: bool,
    /// What the last transfer brought beyond what the last read took.
    buffer: Vec<u8>,
    pos: usize,
    port: Interface,
    /// `readbuffer_chunksize`.
    chunk: usize,
    /// `ftdi->error_str`: the message of the last error, which
    /// `ftdi_usb_close` leaves as it is when it fails.
    error: &'static str,
    /// A transfer found the device gone.
    gone: bool,
}

impl Context {
    /// `ftdi_usb_open_dev` on an opened device: claims port A (the
    /// kernel driver detached when it holds it), resets the port (`-6`
    /// on failure), takes the chip type from `bcdDevice` and the packet
    /// size from the bulk IN endpoint, sets 9600 Bd (`-7`).
    ///
    /// # Errors
    ///
    /// libftdi's code and message for the step that failed; the
    /// transport is dropped then, as libftdi closes the device.
    pub fn open(t: Box<dyn Transport>) -> std::result::Result<Self, FtdiError> {
        Self::open_interface(t, Interface::A)
    }

    /// `ftdi_set_interface` then `ftdi_usb_open_dev`: [`Context::open`]
    /// on port `port`.
    ///
    /// # Errors
    ///
    /// As [`Context::open`].
    pub fn open_interface(
        mut t: Box<dyn Transport>,
        port: Interface,
    ) -> std::result::Result<Self, FtdiError> {
        let info = t.info().clone();
        // libftdi asks libusb, which has the descriptor cached.
        let serial_index = t.string_indexes().map_or(u8::from(info.serial.is_some()), |i| i[2]);
        if let Err(e) = t.claim_interface(port.number()) {
            log::debug!("claiming port {port:?}: {e}");
            return Err(FtdiError::new(
                -5,
                "unable to claim usb device. Make sure the default FTDI driver is not in use",
            ));
        }
        let chip = chip_type(info.device_version, serial_index);
        let default_packet = if chip.high_speed() { 512 } else { 64 };
        let max_packet = t.max_packet_size(port.in_ep()).unwrap_or(default_packet).max(3);
        let mut ctx = Context {
            t,
            chip,
            max_packet,
            bitbang: false,
            buffer: Vec::new(),
            pos: 0,
            port,
            chunk: READ_CHUNK,
            error: "",
            gone: false,
        };
        if ctx.reset().is_err() {
            return Err(ctx.fail(-6, "ftdi_usb_reset failed"));
        }
        if ctx.set_baudrate(9600).is_err() {
            return Err(ctx.fail(-7, "set baudrate failed"));
        }
        Ok(ctx)
    }

    /// The chip type libftdi took from the descriptor.
    pub fn chip(&self) -> ChipType {
        self.chip
    }

    /// `wMaxPacketSize` of the bulk IN endpoint.
    pub fn max_packet_size(&self) -> usize {
        self.max_packet
    }

    /// Gives the transport back.
    pub fn into_transport(self) -> Box<dyn Transport> {
        self.t
    }

    fn out(&mut self, request: u8, value: u16, index: u16) -> Result<()> {
        let r = self.t.control_out(ControlRequest::vendor(request, value, index), &[], TIMEOUT);
        self.gone |= matches!(r, Err(Error::Disconnected));
        r
    }

    /// `ftdi_error_return`: the error, its message kept for
    /// [`Context::close`].
    fn fail(&mut self, code: i32, message: &'static str) -> FtdiError {
        self.error = message;
        FtdiError::new(code, message)
    }

    fn drop_buffer(&mut self) {
        self.buffer.clear();
        self.pos = 0;
    }

    /// `ftdi_usb_reset`.
    ///
    /// # Errors
    ///
    /// `-1`, "FTDI reset failed".
    pub fn reset(&mut self) -> std::result::Result<(), FtdiError> {
        if let Err(e) = self.out(req::RESET, 0, self.port.index()) {
            log::debug!("reset: {e}");
            return Err(self.fail(-1, "FTDI reset failed"));
        }
        self.drop_buffer();
        Ok(())
    }

    /// `ftdi_tcioflush`: the chip's buffer from the host (`-1`), then
    /// its buffer to the host and what is buffered here (`-2`).
    ///
    /// # Errors
    ///
    /// The code above with the failing purge's message.
    pub fn tcioflush(&mut self) -> std::result::Result<(), FtdiError> {
        if let Err(e) = self.out(req::RESET, 1, self.port.index()) {
            log::debug!("purge from host: {e}");
            return Err(self.fail(-1, "FTDI purge of TX buffer failed"));
        }
        if let Err(e) = self.out(req::RESET, 2, self.port.index()) {
            log::debug!("purge to host: {e}");
            return Err(self.fail(-2, "FTDI purge of RX buffer failed"));
        }
        self.drop_buffer();
        Ok(())
    }

    /// `ftdi_set_bitmode`: `mode` is the `SET_BITMODE` mode byte
    /// ([`super::BitMode`] as `u8`), `mask` the outputs.
    ///
    /// # Errors
    ///
    /// `-1`, libftdi's message.
    pub fn set_bitmode(&mut self, mask: u8, mode: u8) -> std::result::Result<(), FtdiError> {
        let value = (u16::from(mode) << 8) | u16::from(mask);
        if let Err(e) = self.out(req::SET_BITMODE, value, self.port.index()) {
            log::debug!("bit mode: {e}");
            return Err(self
                .fail(-1, "unable to configure bitbang mode. Perhaps not a BM/2232C type chip?"));
        }
        self.bitbang = mode != 0;
        Ok(())
    }

    /// `ftdi_set_baudrate`: times 4 in a bit mode, libftdi's divisor,
    /// refused beyond 5 %. Returns the rate the divisor gives.
    ///
    /// # Errors
    ///
    /// [`divisor`]'s, and `-2` when the request fails.
    pub fn set_baudrate(&mut self, baud: u64) -> std::result::Result<u64, FtdiError> {
        let baud = if self.bitbang { baud * 4 } else { baud };
        let (value, bits, best) =
            divisor(self.chip, baud).map_err(|e| self.fail(e.code, e.message))?;
        let index = if self.chip.high_speed() { (bits << 8) | self.port.index() } else { bits };
        if let Err(e) = self.out(req::SET_BAUD_RATE, value, index) {
            log::debug!("baud rate: {e}");
            return Err(self.fail(-2, "Setting new baudrate failed"));
        }
        Ok(best)
    }

    /// `ftdi_setflowctrl`: `flowctrl` is the high byte of `wIndex`
    /// (`SIO_RTS_CTS_HS` is `0x0100`), the port index the low one.
    ///
    /// # Errors
    ///
    /// `-1`, "set flow control failed".
    pub fn set_flow_ctrl(&mut self, flowctrl: u16) -> std::result::Result<(), FtdiError> {
        self.out(req::SET_FLOW_CTRL, 0, flowctrl | INDEX_A).map_err(|e| {
            log::debug!("flow control: {e}");
            FtdiError::new(-1, "set flow control failed")
        })
    }

    /// `ftdi_read_data`: fills `buf` from what the last transfer left,
    /// then from bulk transfers of 4096 bytes (or what
    /// [`Context::read_data_set_chunksize`] set), their status bytes
    /// stripped, until it is full or a transfer brings nothing. What
    /// does not fit stays for the next read. Returns the bytes read.
    ///
    /// # Errors
    ///
    /// A failed transfer, with libusb's code and "usb bulk read
    /// failed"; the bytes gathered by this call are lost, as in
    /// libftdi.
    pub fn read_data(&mut self, buf: &mut [u8]) -> std::result::Result<usize, FtdiError> {
        let left = self.buffer.len() - self.pos;
        if buf.len() <= left {
            buf.copy_from_slice(&self.buffer[self.pos..self.pos + buf.len()]);
            self.pos += buf.len();
            return Ok(buf.len());
        }
        buf[..left].copy_from_slice(&self.buffer[self.pos..]);
        let mut offset = left;
        self.drop_buffer();
        while offset < buf.len() {
            let raw = match self.t.bulk_in(self.port.in_ep(), self.chunk, TIMEOUT) {
                Ok(raw) => raw,
                Err(e) => {
                    log::debug!("bulk read: {e}");
                    self.gone |= matches!(e, Error::Disconnected);
                    return Err(self.fail(libusb_code(&e), "usb bulk read failed"));
                }
            };
            if raw.len() <= 2 {
                // Nothing but the status bytes: no more data.
                return Ok(offset);
            }
            let data = strip_status(&raw, self.max_packet);
            let room = buf.len() - offset;
            if data.len() <= room {
                buf[offset..offset + data.len()].copy_from_slice(&data);
                offset += data.len();
            } else {
                buf[offset..].copy_from_slice(&data[..room]);
                offset += room;
                self.buffer = data;
                self.pos = room;
            }
        }
        Ok(offset)
    }

    /// `ftdi_read_data_set_chunksize`: the length of the bulk IN
    /// transfers of [`Context::read_data`], at most 16 KiB as libftdi
    /// has it on Linux (on every platform here, so that the device sees
    /// one sequence). What the last transfer left is dropped. Sends
    /// nothing.
    pub fn read_data_set_chunksize(&mut self, size: usize) {
        self.drop_buffer();
        self.chunk = size.clamp(1, MAX_READ_CHUNK);
    }

    /// `ftdi_write_data`: bulk transfers of at most 4096 bytes to the
    /// port's OUT endpoint. Returns the bytes written.
    ///
    /// # Errors
    ///
    /// `-1`, "usb bulk write failed", when a transfer fails, also after
    /// a partial one, or takes nothing.
    pub fn write_data(&mut self, data: &[u8]) -> std::result::Result<usize, FtdiError> {
        let mut offset = 0;
        while offset < data.len() {
            let end = (offset + WRITE_CHUNK).min(data.len());
            match self.t.bulk_out(self.port.out_ep(), &data[offset..end], TIMEOUT) {
                // libftdi would send it again for ever; a guard ends it.
                Ok(0) => return Err(self.fail(-1, "usb bulk write failed")),
                // A short take: the rest goes in the next transfer.
                Ok(n) => offset += n,
                Err(e) => {
                    log::debug!("bulk write: {e}");
                    self.gone |= matches!(e, Error::Disconnected);
                    return Err(self.fail(-1, "usb bulk write failed"));
                }
            }
        }
        Ok(offset)
    }

    /// `ftdi_set_latency_timer`, in ms (1 to 255).
    ///
    /// # Errors
    ///
    /// `-1` for 0, `-2` when the request fails.
    pub fn set_latency_timer(&mut self, latency: u8) -> std::result::Result<(), FtdiError> {
        if latency < 1 {
            return Err(self.fail(-1, "latency out of range. Only valid for 1-255"));
        }
        if let Err(e) = self.out(req::SET_LATENCY_TIMER, u16::from(latency), self.port.index()) {
            log::debug!("latency timer: {e}");
            return Err(self.fail(-2, "unable to set latency timer"));
        }
        Ok(())
    }

    /// `ftdi_usb_close`: releases the port's interface and lets go of
    /// the device. Nothing is sent.
    ///
    /// # Errors
    ///
    /// `-1` when the release fails, with the message of the last
    /// error, which libftdi leaves as it was. On a device a transfer
    /// found gone it always fails, as `libusb_release_interface` does
    /// (`LIBUSB_ERROR_NO_DEVICE`), whatever the USB layer's own release
    /// says: nusb's reports success there.
    pub fn close(mut self) -> std::result::Result<(), FtdiError> {
        let released = self.t.release_interface(self.port.number());
        if let Err(e) = &released {
            log::debug!("releasing port {:?}: {e}", self.port);
        }
        if released.is_err() || self.gone {
            return Err(FtdiError::new(-1, self.error));
        }
        Ok(())
    }
}

/// The product string check `ftdi_usb_open_desc` makes on each device
/// with the ids asked for, before opening it: `iProduct` read as
/// `libusb_get_string_descriptor_ascii` reads it, then compared.
///
/// # Errors
///
/// `-8` when the string cannot be read, `-3` ("device not found") when
/// it is another one; libftdi then tries the next device.
pub fn check_product(t: &mut dyn Transport, product: &str) -> std::result::Result<(), FtdiError> {
    let index = t.string_indexes().map_or(0, |i| i[1]);
    match crate::usb::read_string_ascii(t, index) {
        Ok(s) if s == product => Ok(()),
        Ok(s) => {
            log::debug!("The product string '{s}' is not '{product}'.");
            Err(FtdiError::new(-3, "device not found"))
        }
        Err(e) => {
            log::debug!("product string: {e}");
            Err(FtdiError::new(-8, "unable to fetch product description"))
        }
    }
}

/// What a sync FIFO driver adds (`ikalogic-scanaplus`): EEPROM words.
impl Context {
    /// `ftdi_read_eeprom_location`: EEPROM word `addr` (`READ_EEPROM`,
    /// `wIndex` the word's address, no port index).
    ///
    /// # Errors
    ///
    /// `-1`, "reading eeprom failed", when the request fails or brings
    /// other than 2 bytes.
    pub fn read_eeprom_location(&mut self, addr: u16) -> std::result::Result<u16, FtdiError> {
        let d = self.t.control_in(ControlRequest::vendor(req::READ_EEPROM, 0, addr), 2, TIMEOUT);
        let d = match d {
            Ok(d) if d.len() == 2 => d,
            r => {
                log::debug!("EEPROM word {addr}: {r:?}");
                return Err(self.fail(-1, "reading eeprom failed"));
            }
        };
        Ok(u16::from_le_bytes([d[0], d[1]]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;

    #[test]
    fn divisors_as_libftdi_computes_them() {
        // (chip, baud, value, bits, best): the emulator's FTDI shell
        // holds the same cases from libftdi's own results.
        let cases = [
            (ChipType::R, 9_600, 0x4138, 0, 9_600),
            (ChipType::Bm, 115_200, 0x001a, 0, 115_385),
            (ChipType::R, 3_000_000, 0, 0, 3_000_000),
            (ChipType::R, 2_000_000, 1, 0, 2_000_000),
            (ChipType::R, 480, 0x186a, 0, 480),
            (ChipType::Ft2232h, 2_000_000, 6, 2, 2_000_000),
            (ChipType::Ft2232h, 720, 0x5046, 1, 720),
            (ChipType::Ft2232h, 12_000_000, 0, 2, 12_000_000),
            (ChipType::Ft2232h, 9_600, 0x04e2, 2, 9_600),
            (ChipType::Ft232h, 200_000, 0x003c, 2, 200_000),
        ];
        for (chip, baud, value, bits, best) in cases {
            assert_eq!(divisor(chip, baud), Ok((value, bits, best)), "{chip:?} {baud}");
        }
        // 100 MHz on an FT2232H: 5 MBd, 20 MBd in bit-bang mode, beyond
        // the 12 MBd the clock gives.
        assert_eq!(divisor(ChipType::Ft2232h, 20_000_000).unwrap_err().code, -1);
        assert!(divisor(ChipType::R, 0).is_err());
    }

    #[test]
    fn am_divisors_as_libftdi_computes_them() {
        // (baud, value, bits, best), from ftdi_to_clkbits_AM: 9600 Bd
        // is what libftdi's open sends an AM (ctl 40 03 4138 0000).
        let cases = [
            (9_600, 0x4138, 0, 9_600),
            (115_200, 0x001a, 0, 115_385),
            (3_000_000, 0, 0, 3_000_000),
            // 24 MHz / 1.6 MBd is 15: no divisor from 9 to 15, so 16.
            (1_600_000, 2, 0, 1_500_000),
            // 24 MHz / 57600 is 416.67 (in eighths of 3 MHz): 416
            // (52) gives 57692, 417 (52 1/8) 57554, the closer.
            (57_600, 0xc034, 0, 57_554),
            // 24 MHz / 38400 = 625 = 78 1/8: the eighth an AM has.
            (38_400, 0xc04e, 0, 38_400),
            // 24 MHz / 13000 = 1846 = 230 6/8, which the AM lacks: down
            // to 230 4/8 (13015 Bd), or 1847 up to 231 (12987 Bd),
            // the closer.
            (13_000, 0x00e7, 0, 12_987),
            // 300 Bd: 80000 = 10000.0.
            (300, 0x2710, 0, 300),
            // Below 183.1 Bd the divisor stops at 0x1fff8: 0x3fff.
            (150, 0x3fff, 0, 183),
        ];
        for (baud, value, bits, best) in cases {
            let got = clkbits_am(baud);
            assert_eq!(
                ((got.0 & 0xffff) as u16, (got.0 >> 16) as u16, got.1),
                (value, bits, best),
                "{baud}"
            );
            // No eighths code beyond the AM's 0 to 3.
            assert!(got.0 >> 14 <= 3, "{baud}: {:#x}", got.0);
        }
        assert_eq!(divisor(ChipType::Am, 9_600), Ok((0x4138, 0, 9_600)));
        // 150 Bd gives 183 Bd, beyond 5 %: refused as libftdi does.
        assert_eq!(divisor(ChipType::Am, 150).unwrap_err().code, -1);
    }

    #[test]
    fn chip_types_as_libftdi_guesses_them() {
        assert_eq!(chip_type(0x0200, 0), ChipType::Bm);
        assert_eq!(chip_type(0x0200, 3), ChipType::Am);
        assert_eq!(chip_type(0x0700, 0), ChipType::Ft2232h);
    }

    fn replay(steps: &str) -> Box<dyn Transport> {
        let text = format!(
            "# oxrok usb fixture 1\ndevice vid=0403 pid=6010 version=0700 bus=0 addr=0 \
             strings=1,2,0\nclaim 0 -> ok\nmps 81 -> 512\n{steps}"
        );
        Box::new(Replay::from_text(&text).unwrap())
    }

    const OPEN: &str = "ctl_out 40 00 0000 0001 - -> ok\nctl_out 40 03 04e2 0201 - -> ok\n";

    /// Two transfers of 4096 bytes in 512 byte packets: 4080 bytes each.
    fn packets(start: u8, n: usize) -> String {
        use std::fmt::Write as _;
        let mut s = String::new();
        let mut v = start;
        for _ in 0..n {
            s.push_str("0160");
            for _ in 0..510 {
                let _ = write!(s, "{v:02x}");
                v = v.wrapping_add(1);
            }
        }
        s
    }

    #[test]
    fn reads_as_ftdi_read_data() {
        let steps = format!(
            "{OPEN}ctl_out 40 0b 0100 0001 - -> ok\nctl_out 40 03 003c 0201 - -> ok\n\
             bulk_in 81 4096 -> 0160\n\
             bulk_in 81 4096 -> {}\nbulk_in 81 4096 -> {}\nbulk_in 81 4096 -> err device \"endpoint stalled\"\n",
            packets(0, 8),
            packets(0xf0, 8)
        );
        let mut ctx = Context::open(replay(&steps)).unwrap();
        assert_eq!(ctx.max_packet_size(), 512);
        ctx.set_bitmode(0, 1).unwrap();
        // 1 MHz / 20, times 4.
        assert_eq!(ctx.set_baudrate(50_000), Ok(200_000));
        let mut buf = vec![0u8; 5000];
        // A status-only transfer: nothing.
        assert_eq!(ctx.read_data(&mut buf), Ok(0));
        // 4080 + 920 of the next 4080; 3160 stay.
        assert_eq!(ctx.read_data(&mut buf), Ok(5000));
        assert_eq!(&buf[..3], &[0, 1, 2]);
        assert_eq!(buf[4080], 0xf0);
        let mut small = [0u8; 10];
        assert_eq!(ctx.read_data(&mut small), Ok(10));
        assert_eq!(small[0], 0xf0u8.wrapping_add(920u32 as u8));
        // The rest, then a stall: the read fails and loses it.
        let e = ctx.read_data(&mut buf).unwrap_err();
        assert_eq!(e, FtdiError::new(-9, "usb bulk read failed"));
    }

    /// Port B through `ftdi_usb_open_dev`.
    fn port_b(steps: &str) -> Context {
        let text = format!(
            "# oxrok usb fixture 1\ndevice vid=0403 pid=6010 version=0700 bus=0 addr=0 \
             strings=1,2,0\nclaim 1 -> ok\nmps 83 -> 512\n\
             ctl_out 40 00 0000 0002 - -> ok\nctl_out 40 03 04e2 0202 - -> ok\n{steps}"
        );
        Context::open_interface(Box::new(Replay::from_text(&text).unwrap()), Interface::B).unwrap()
    }

    #[test]
    fn a_short_write_goes_on_from_what_was_taken() {
        let mut ctx = port_b(
            "bulk_out 04 01020304 -> 3\nbulk_out 04 04 -> 1\n\
             bulk_out 04 05 -> 0\nbulk_out 04 06 -> err device \"endpoint stalled\"\n",
        );
        assert_eq!(ctx.write_data(&[1, 2, 3, 4]), Ok(4));
        assert_eq!(ctx.write_data(&[5]).unwrap_err().code, -1);
        assert_eq!(ctx.write_data(&[6]).unwrap_err().message, "usb bulk write failed");
    }

    #[test]
    fn port_b_requests_and_endpoints() {
        let mut ctx = port_b(
            "ctl_out 40 00 0001 0002 - -> ok\nctl_out 40 00 0002 0002 - -> ok\n\
             ctl_out 40 0b 00ff 0002 - -> ok\nctl_out 40 09 0010 0002 - -> ok\n\
             bulk_out 04 c000000000 -> 5\n\
             bulk_in 83 16384 -> 016031414c53\nrelease 1 -> ok\n",
        );
        ctx.tcioflush().unwrap();
        ctx.set_bitmode(0xff, 0).unwrap();
        ctx.set_latency_timer(16).unwrap();
        assert_eq!(ctx.set_latency_timer(0).unwrap_err().code, -1);
        // Asked for 64 KiB: 16 KiB, as libftdi on Linux.
        ctx.read_data_set_chunksize(64 * 1024);
        assert_eq!(ctx.write_data(&[0xc0, 0, 0, 0, 0]), Ok(5));
        let mut id = [0u8; 4];
        assert_eq!(ctx.read_data(&mut id), Ok(4));
        assert_eq!(&id, b"1ALS");
        ctx.close().unwrap();
    }

    #[test]
    fn a_failed_close_keeps_the_last_message() {
        // A device gone: the release fails whatever the layer below
        // says, with the message of the write before it.
        let mut ctx = port_b("bulk_out 04 00 -> err disconnected\nrelease 1 -> ok\n");
        assert_eq!(ctx.write_data(&[0]).unwrap_err().message, "usb bulk write failed");
        assert_eq!(ctx.close(), Err(FtdiError::new(-1, "usb bulk write failed")));
        // A release that fails on a device that seems there.
        let ctx = port_b("release 1 -> err device \"gone\"\n");
        assert_eq!(ctx.close().unwrap_err().code, -1);
    }

    #[test]
    fn the_product_string_decides() {
        const STRINGS: &str = "ctl_in 00 06 0300 0000 4 -> 04030904\n\
            ctl_in 00 06 0302 0409 255 -> 0a034100420043004400\n";
        let strings = |steps: &str| -> Box<dyn Transport> {
            let text = format!(
                "# oxrok usb fixture 1\ndevice vid=0403 pid=6010 version=0700 bus=0 addr=0 \
                 strings=1,2,0\n{steps}"
            );
            Box::new(Replay::from_text(&text).unwrap())
        };
        let mut t = strings(STRINGS);
        assert_eq!(check_product(t.as_mut(), "ABCD"), Ok(()));
        let mut t = strings(STRINGS);
        assert_eq!(check_product(t.as_mut(), "Other").unwrap_err().code, -3);
        let mut t = strings("ctl_in 00 06 0300 0000 4 -> err device \"endpoint stalled\"\n");
        assert_eq!(check_product(t.as_mut(), "ABCD").unwrap_err().code, -8);
    }

    #[test]
    fn writes_as_ftdi_write_data() {
        // 5000 bytes: 4096, then 904.
        let (a, b) = ("55".repeat(4096), "55".repeat(904));
        let steps = format!(
            "{OPEN}bulk_out 02 {a} -> 4096\nbulk_out 02 {b} -> 904\n\
             bulk_out 02 01 -> err device \"endpoint stalled\"\n"
        );
        let mut ctx = Context::open(replay(&steps)).unwrap();
        assert_eq!(ctx.write_data(&[0x55; 5000]), Ok(5000));
        assert_eq!(ctx.write_data(&[1]), Err(FtdiError::new(-1, "usb bulk write failed")));
    }

    #[test]
    fn a_short_write_goes_on_and_an_empty_one_fails() {
        let steps =
            format!("{OPEN}bulk_out 02 010203 -> 2\nbulk_out 02 03 -> 1\nbulk_out 02 04 -> 0\n");
        let mut ctx = Context::open(replay(&steps)).unwrap();
        assert_eq!(ctx.write_data(&[1, 2, 3]), Ok(3));
        assert_eq!(ctx.write_data(&[4]), Err(FtdiError::new(-1, "usb bulk write failed")));
    }

    #[test]
    fn open_fails_as_libftdi() {
        let steps = "ctl_out 40 00 0000 0001 - -> err device \"endpoint stalled\"\n";
        assert_eq!(Context::open(replay(steps)).err().unwrap().code, -6);
        let steps = format!("{OPEN}ctl_out 40 00 0001 0001 - -> err device \"endpoint stalled\"\n");
        let mut ctx = Context::open(replay(&steps)).unwrap();
        assert_eq!(ctx.tcioflush().unwrap_err().message, "FTDI purge of TX buffer failed");
    }

    #[test]
    fn sync_fifo_calls_as_libftdi() {
        let steps = format!(
            "{OPEN}ctl_out 40 09 0002 0001 - -> ok\n\
             ctl_in 40 90 0000 0010 2 -> 1ca3\n\
             ctl_in 40 90 0000 0011 2 -> 57\n\
             ctl_in 40 90 0000 0012 2 -> err device \"endpoint stalled\"\n\
             ctl_out 40 09 0010 0001 - -> err device \"endpoint stalled\"\n\
             bulk_out 02 8841 -> 2\n\
             bulk_out 02 8840 -> err disconnected\n\
             bulk_in 81 16384 -> 0160{}\n\
             release 0 -> ok\n",
            "ab".repeat(510)
        );
        let mut ctx = Context::open(replay(&steps)).unwrap();
        ctx.set_latency_timer(2).unwrap();
        assert_eq!(ctx.read_eeprom_location(16), Ok(0xa31c));
        // A short answer fails as a failed request does.
        let e = FtdiError::new(-1, "reading eeprom failed");
        assert_eq!(ctx.read_eeprom_location(17), Err(e.clone()));
        assert_eq!(ctx.read_eeprom_location(18), Err(e));
        assert_eq!(ctx.set_latency_timer(0).unwrap_err().code, -1, "nothing sent");
        assert_eq!(ctx.set_latency_timer(16).unwrap_err().code, -2);
        assert_eq!(ctx.write_data(&[0x88, 0x41]), Ok(2));
        assert_eq!(ctx.write_data(&[0x88, 0x40]).unwrap_err().message, "usb bulk write failed");
        // 64 KiB asked for, 16 KiB transfers on Linux.
        ctx.read_data_set_chunksize(64 * 1024);
        let mut buf = vec![0u8; 100];
        assert_eq!(ctx.read_data(&mut buf), Ok(100));
        ctx.close().unwrap();
    }

    #[test]
    fn flow_control_as_libftdi() {
        let steps = format!(
            "{OPEN}ctl_out 40 02 0000 0101 - -> ok\n\
             ctl_out 40 02 0000 0101 - -> err device \"endpoint stalled\"\n"
        );
        let mut ctx = Context::open(replay(&steps)).unwrap();
        ctx.set_flow_ctrl(0x0100).unwrap();
        assert_eq!(
            ctx.set_flow_ctrl(0x0100).unwrap_err(),
            FtdiError::new(-1, "set flow control failed")
        );
    }
}
