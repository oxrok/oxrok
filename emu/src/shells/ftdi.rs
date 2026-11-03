// Counterpart in sigrok:
//   libftdi1 (ftdi.c), by Intra2net AG and the libftdi developers.
// This file is written for oxrok from FTDI's application notes; libftdi was
//   the reference for which requests a driver sends.

//! The FTDI shell: what every FT232/FT245/FT2232/FT4232/FT232H/FT-X
//! device shares, so that a model only says what its pins and FIFOs
//! carry.
//!
//! - The descriptors of each chip as it leaves the factory: `bcdDevice`
//!   (which libftdi and the kernel's `ftdi_sio` take the chip type
//!   from), the speed, one vendor class interface per port with a bulk
//!   IN endpoint `0x81 + 2n` and a bulk OUT endpoint `0x02 + 2n`, 64
//!   byte packets at full speed and 512 at high speed. [`Chip`] lists
//!   them; [`FtdiConfig`] adds the ids and strings a board's EEPROM
//!   gives it.
//! - The vendor requests, device recipient, each on one port: reset
//!   and the two purges, modem control (DTR, RTS), flow control, baud
//!   rate (the `AN_120` divisor encoding, decoded and checked), data
//!   characteristics, event and error characters, poll modem status,
//!   set and get latency timer, set bit mode (each mode checked against
//!   the chip and port that have it), read pins, and EEPROM reads (an
//!   image built from the configuration, with its checksum). A write or
//!   erase of the EEPROM is applied but logged as a violation: no
//!   driver under test has a reason to send one.
//! - The two status bytes (modem status, line status) that start every
//!   packet of a bulk IN transfer, `wMaxPacketSize` apart, and the
//!   status-only packet the chip sends when it has nothing (after its
//!   latency timer on a real chip; at once here, or held when the
//!   model says so).
//!
//! The port a request addresses is the low byte of `wIndex`: 1 for
//! port A, 2 for B, and so on (libftdi's `INTERFACE_A` and up, as the
//! kernel also sends on multi port chips). A single port chip takes 0
//! (what `ftdi_sio` sends) or 1 (what libftdi sends). `SET_BAUD_RATE`
//! is the exception: on single port chips its `wIndex` is bit 16 of
//! the divisor, on the others the high byte holds divisor bit 16 and,
//! on the H chips, the 120 MHz clock switch in bit 9.
//!
//! A model implements [`FtdiPorts`]: notice of what the host changed
//! ([`Event`]), the bytes the host writes, the bytes the chip has for
//! the host, and the pin levels. The shell keeps each port's settings
//! in a [`PortState`] the model reads.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{BULK, Descriptors, RECIPIENT_DEVICE, Reply, Setup, Speed, VENDOR, Violations};

/// FTDI's vendor id.
pub const FTDI_VENDOR: u16 = 0x0403;

/// The vendor requests.
pub mod request {
    /// Reset the port (`wValue` 0), or purge one of its buffers (1, 2).
    pub const RESET: u8 = 0x00;
    /// DTR and RTS: the levels in the low byte, which to change in the high.
    pub const MODEM_CTRL: u8 = 0x01;
    /// Flow control: the mode in the high byte of `wIndex`.
    pub const SET_FLOW_CTRL: u8 = 0x02;
    /// The baud rate divisor.
    pub const SET_BAUD_RATE: u8 = 0x03;
    /// Data bits, parity, stop bits, break.
    pub const SET_DATA: u8 = 0x04;
    /// The two status bytes.
    pub const POLL_MODEM_STATUS: u8 = 0x05;
    /// The event character.
    pub const SET_EVENT_CHAR: u8 = 0x06;
    /// The error character.
    pub const SET_ERROR_CHAR: u8 = 0x07;
    /// The latency timer, 1 to 255 ms.
    pub const SET_LATENCY_TIMER: u8 = 0x09;
    /// The latency timer, one byte.
    pub const GET_LATENCY_TIMER: u8 = 0x0a;
    /// The bit mode in the high byte of `wValue`, the output mask in the low.
    pub const SET_BITMODE: u8 = 0x0b;
    /// The levels of the eight data pins, one byte.
    pub const READ_PINS: u8 = 0x0c;
    /// One EEPROM word, the address in `wIndex`.
    pub const READ_EEPROM: u8 = 0x90;
    /// One EEPROM word written.
    pub const WRITE_EEPROM: u8 = 0x91;
    /// The whole EEPROM erased.
    pub const ERASE_EEPROM: u8 = 0x92;
}

/// The `RESET` request's `wValue`s.
pub mod reset {
    /// Resets the port: buffers emptied, settings kept.
    pub const SIO: u16 = 0;
    /// Drops the data the host wrote and the chip has not sent out yet
    /// (libftdi's `SIO_TCOFLUSH`, the old `SIO_RESET_PURGE_RX`).
    pub const FLUSH_FROM_HOST: u16 = 1;
    /// Drops the data the chip holds for the host (`SIO_TCIFLUSH`, the
    /// old `SIO_RESET_PURGE_TX`).
    pub const FLUSH_TO_HOST: u16 = 2;
}

/// The bit modes of `SET_BITMODE`, one bit each.
pub mod bitmode {
    /// Back to the UART or FIFO function.
    pub const RESET: u8 = 0x00;
    /// Asynchronous bit-bang (BM and later).
    pub const BITBANG: u8 = 0x01;
    /// MPSSE (FT2232 and the H chips).
    pub const MPSSE: u8 = 0x02;
    /// Synchronous bit-bang (R, FT2232 and later).
    pub const SYNCBB: u8 = 0x04;
    /// MCU host bus emulation (FT2232, FT2232H port A, FT232H).
    pub const MCU: u8 = 0x08;
    /// Fast opto-isolated serial (FT2232, FT2232H, FT232H).
    pub const OPTO: u8 = 0x10;
    /// Bit-bang on the CBUS pins (R, FT232H, FT-X).
    pub const CBUS: u8 = 0x20;
    /// Synchronous FIFO (FT2232H port A, FT232H).
    pub const SYNCFF: u8 = 0x40;
    /// FT1284 (FT232H).
    pub const FT1284: u8 = 0x80;
}

/// The chip families, by `bcdDevice`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chip {
    /// FT8U232AM / FT8U245AM.
    Am,
    /// FT232BM / FT245BM (and the BL, BQ).
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
}

impl Chip {
    /// `bcdDevice` of the chip, as libftdi and `ftdi_sio` recognise it.
    pub fn bcd_device(self) -> u16 {
        match self {
            Chip::Am => 0x0200,
            Chip::Bm => 0x0400,
            Chip::Ft2232c => 0x0500,
            Chip::R => 0x0600,
            Chip::Ft2232h => 0x0700,
            Chip::Ft4232h => 0x0800,
            Chip::Ft232h => 0x0900,
            Chip::Ft230x => 0x1000,
        }
    }

    /// The product id the chip has without an EEPROM saying otherwise.
    pub fn default_product(self) -> u16 {
        match self {
            Chip::Am | Chip::Bm | Chip::R => 0x6001,
            Chip::Ft2232c | Chip::Ft2232h => 0x6010,
            Chip::Ft4232h => 0x6011,
            Chip::Ft232h => 0x6014,
            Chip::Ft230x => 0x6015,
        }
    }

    /// The product string FTDI programs.
    pub fn default_description(self) -> &'static str {
        match self {
            Chip::Am | Chip::Bm => "USB <-> Serial",
            Chip::Ft2232c => "Dual RS232",
            Chip::R => "FT232R USB UART",
            Chip::Ft2232h => "Dual RS232-HS",
            Chip::Ft4232h => "Quad RS232-HS",
            Chip::Ft232h => "Single RS232-HS",
            Chip::Ft230x => "FT230X Basic UART",
        }
    }

    /// The number of ports (interfaces).
    pub fn ports(self) -> u8 {
        match self {
            Chip::Ft2232c | Chip::Ft2232h => 2,
            Chip::Ft4232h => 4,
            _ => 1,
        }
    }

    /// The high speed chips, with the 120 MHz clock and 512 byte packets.
    pub fn high_speed(self) -> bool {
        matches!(self, Chip::Ft2232h | Chip::Ft4232h | Chip::Ft232h)
    }

    /// `wMaxPacketSize` of the bulk endpoints.
    pub fn packet_size(self) -> u16 {
        if self.high_speed() { 512 } else { 64 }
    }

    /// `bMaxPacketSize0`.
    pub fn max_packet0(self) -> u8 {
        if self.high_speed() { 64 } else { 8 }
    }

    /// The EEPROM size in 16 bit words: a 93C46 on the older chips,
    /// the internal one of the R, a 93C56 on the H chips, the MTP
    /// memory of the X chips.
    pub fn eeprom_words(self) -> u16 {
        match self {
            Chip::Am | Chip::Bm | Chip::Ft2232c | Chip::R => 64,
            Chip::Ft2232h | Chip::Ft4232h | Chip::Ft232h | Chip::Ft230x => 128,
        }
    }

    /// The bit modes port `port` (0 based) of the chip has, as a mask
    /// of [`bitmode`] bits.
    pub fn modes(self, port: u8) -> u8 {
        use bitmode::{BITBANG, CBUS, FT1284, MCU, MPSSE, OPTO, SYNCBB, SYNCFF};
        match (self, port) {
            (Chip::Am, _) => 0,
            (Chip::Bm, _) => BITBANG,
            (Chip::Ft2232c, 0) => BITBANG | MPSSE | SYNCBB | MCU | OPTO,
            (Chip::Ft2232c, _) => BITBANG | SYNCBB | OPTO,
            (Chip::R | Chip::Ft230x, _) => BITBANG | SYNCBB | CBUS,
            (Chip::Ft2232h, 0) => BITBANG | MPSSE | SYNCBB | MCU | OPTO | SYNCFF,
            (Chip::Ft2232h, _) => BITBANG | MPSSE | SYNCBB | OPTO,
            (Chip::Ft4232h, 0 | 1) => BITBANG | MPSSE | SYNCBB,
            (Chip::Ft4232h, _) => BITBANG | SYNCBB,
            (Chip::Ft232h, _) => BITBANG | MPSSE | SYNCBB | MCU | OPTO | CBUS | SYNCFF | FT1284,
        }
    }

    /// The name as FTDI writes it.
    pub fn name(self) -> &'static str {
        match self {
            Chip::Am => "FT8U232AM",
            Chip::Bm => "FT232BM",
            Chip::Ft2232c => "FT2232C",
            Chip::R => "FT232R",
            Chip::Ft2232h => "FT2232H",
            Chip::Ft4232h => "FT4232H",
            Chip::Ft232h => "FT232H",
            Chip::Ft230x => "FT230X",
        }
    }
}

/// What a board makes of its chip: ids and strings (from its EEPROM),
/// power.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FtdiConfig {
    /// The chip.
    pub chip: Chip,
    /// `idVendor`.
    pub vendor: u16,
    /// `idProduct`.
    pub product: u16,
    /// The manufacturer string, absent when `None` (`iManufacturer` 0).
    pub manufacturer: Option<String>,
    /// The product string.
    pub description: Option<String>,
    /// The serial number string.
    pub serial: Option<String>,
    /// Configuration `bmAttributes`: 0x80 bus powered, 0xa0 with remote
    /// wakeup, 0xc0 self powered.
    pub attributes: u8,
    /// `bMaxPower` in mA.
    pub max_power: u16,
}

impl FtdiConfig {
    /// The chip as FTDI ships it: its own ids and strings, no serial
    /// number, bus powered at 100 mA.
    pub fn new(chip: Chip) -> FtdiConfig {
        FtdiConfig {
            chip,
            vendor: FTDI_VENDOR,
            product: chip.default_product(),
            manufacturer: Some("FTDI".to_owned()),
            description: Some(chip.default_description().to_owned()),
            serial: None,
            attributes: 0x80,
            max_power: 100,
        }
    }
}

/// A decoded `SET_BAUD_RATE`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Divisor {
    /// The 17 bit encoded divisor: 14 integer bits, a 3 bit code for
    /// the eighths in bits 14 to 16.
    pub encoded: u32,
    /// The divisor in eighths (8 is divide by 1).
    pub eighths: u32,
    /// The clock it divides, in Hz: 3 MHz (48 MHz / 16), or 12 MHz
    /// (120 MHz / 10) on the H chips with the switch set.
    pub clock: u32,
}

impl Divisor {
    /// The baud rate it gives, in Bd.
    pub fn baud(self) -> u32 {
        (u64::from(self.clock) * 8 / u64::from(self.eighths)) as u32
    }
}

/// `AN_120`: the 3 bit code of each eighth (0, 1/8, 1/4, 3/8, 1/2, ...).
const FRAC_CODE: [u32; 8] = [0, 3, 2, 4, 1, 5, 6, 7];
/// The inverse: the eighths each code stands for.
const CODE_EIGHTHS: [u32; 8] = [0, 4, 2, 1, 3, 5, 6, 7];

/// Decodes `SET_BAUD_RATE`'s `wValue` and the divisor bits of its
/// `wIndex` (bit 0 divisor bit 16, bit 1 the H chips' 120 MHz switch;
/// already shifted down to bit 0 on the multi port and H chips).
///
/// # Errors
///
/// A combination `AN_120` rules out (sub-integer divisors below 2 other
/// than 1 and 1.5; eighths the AM chips lack; the clock switch on a
/// chip without it; other index bits), as a message.
pub fn decode_divisor(chip: Chip, value: u16, bits: u16) -> Result<Divisor, String> {
    if bits & !3 != 0 {
        return Err(format!("SET_BAUD_RATE divisor bits {bits:#06x} beyond bits 16 and 17"));
    }
    let clk10 = bits & 2 != 0;
    if clk10 && !chip.high_speed() {
        return Err(format!("SET_BAUD_RATE with the 120 MHz switch on an {}", chip.name()));
    }
    let encoded = u32::from(value) | (u32::from(bits & 1) << 16);
    let int = encoded & 0x3fff;
    let code = encoded >> 14;
    if chip == Chip::Am && code > 3 {
        return Err(format!("SET_BAUD_RATE eighths code {code} on an AM chip, which has 0 to 3"));
    }
    let eighths = match (int, code) {
        (0, 0) => 8,
        (1, 0) => 12,
        (0 | 1, _) => {
            return Err(format!(
                "SET_BAUD_RATE divisor {int} with eighths code {code}: AN_120 allows no \
                 fraction below 2 other than 1.5"
            ));
        }
        _ => int * 8 + CODE_EIGHTHS[code as usize],
    };
    Ok(Divisor { encoded, eighths, clock: if clk10 { 12_000_000 } else { 3_000_000 } })
}

/// libftdi's divisor for `baud` (`ftdi_convert_baudrate`, the bit-bang
/// factor of 4 already applied by the caller): `wValue`, the divisor
/// bits of `wIndex` as [`decode_divisor`] takes them, and the rate it
/// gives. libftdi refuses rates more than 5 % from what they give.
pub fn libftdi_divisor(chip: Chip, baud: u32) -> (u16, u16, u32) {
    fn clkbits(baud: u64, clk: u64, div: u64) -> (u32, u32) {
        if baud >= clk / div {
            (0, (clk / div) as u32)
        } else if baud >= clk / (div + div / 2) {
            (1, (clk / (div + div / 2)) as u32)
        } else if baud >= clk / (2 * div) {
            (2, (clk / (2 * div)) as u32)
        } else {
            let divisor = clk * 16 / div / baud;
            let mut best = if divisor & 1 == 1 { divisor / 2 + 1 } else { divisor / 2 };
            if best > 0x20000 {
                best = 0x1ffff;
            }
            let b = clk * 16 / div / best;
            let b = if b & 1 == 1 { b / 2 + 1 } else { b / 2 };
            (((best >> 3) as u32) | (FRAC_CODE[(best & 7) as usize] << 14), b as u32)
        }
    }
    let baud = u64::from(baud.max(1));
    let (encoded, best) = if chip.high_speed() && baud * 10 > 120_000_000 / 0x3fff {
        let (e, b) = clkbits(baud, 120_000_000, 10);
        (e | 0x20000, b)
    } else {
        clkbits(baud, 48_000_000, 16)
    };
    ((encoded & 0xffff) as u16, (encoded >> 16) as u16, best)
}

/// The settings of one port, as the host left them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PortState {
    /// The bit mode, one of [`bitmode`].
    pub bitmode: u8,
    /// The output mask of the bit mode: 1 bits are outputs.
    pub mask: u8,
    /// The last baud rate divisor set, `None` until one is.
    pub divisor: Option<Divisor>,
    /// `SET_DATA`'s `wValue`: data bits, parity << 8, stop bits << 11,
    /// break << 14. 8N1 at power on.
    pub line: u16,
    /// The flow control mode (`SET_FLOW_CTRL`'s high `wIndex` byte):
    /// 0, 1 RTS/CTS, 2 DTR/DSR, 4 XON/XOFF.
    pub flow: u8,
    /// XON in the low byte, XOFF in the high, for XON/XOFF flow control.
    pub xon_xoff: u16,
    /// The DTR output.
    pub dtr: bool,
    /// The RTS output.
    pub rts: bool,
    /// The latency timer in ms, 16 at power on.
    pub latency: u8,
    /// `SET_EVENT_CHAR`'s `wValue`: the character, bit 8 enables it.
    pub event_char: u16,
    /// `SET_ERROR_CHAR`'s `wValue`.
    pub error_char: u16,
    /// The modem status byte: bit 0 always set, CTS bit 4, DSR 5, RI 6,
    /// DCD 7. A model sets the line bits through [`Ftdi::port_mut`].
    pub modem_status: u8,
    /// The line status byte: data ready, overrun, parity, framing,
    /// break, then THRE and TEMT (bits 5 and 6, set when idle), FIFO
    /// error.
    pub line_status: u8,
}

impl Default for PortState {
    fn default() -> PortState {
        PortState {
            bitmode: bitmode::RESET,
            mask: 0,
            divisor: None,
            line: 8,
            flow: 0,
            xon_xoff: 0,
            dtr: false,
            rts: false,
            latency: 16,
            event_char: 0,
            error_char: 0,
            modem_status: 0x01,
            line_status: 0x60,
        }
    }
}

impl PortState {
    /// The two bytes at the start of every IN packet.
    pub fn status(&self) -> [u8; 2] {
        [self.modem_status, self.line_status]
    }

    /// The baud rate set, `None` before a `SET_BAUD_RATE`.
    pub fn baud(&self) -> Option<u32> {
        self.divisor.map(Divisor::baud)
    }
}

/// What a request changed, for [`FtdiPorts::changed`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// `RESET` with [`reset::SIO`].
    Reset,
    /// [`reset::FLUSH_FROM_HOST`].
    FlushFromHost,
    /// [`reset::FLUSH_TO_HOST`].
    FlushToHost,
    /// `SET_BITMODE`.
    Bitmode,
    /// `SET_BAUD_RATE`.
    Baud,
    /// `SET_DATA`.
    Line,
    /// `SET_FLOW_CTRL`.
    Flow,
    /// `MODEM_CTRL`.
    ModemCtrl,
    /// `SET_LATENCY_TIMER`.
    Latency,
    /// `SET_EVENT_CHAR` or `SET_ERROR_CHAR`.
    Chars,
}

/// The answer to a bulk IN transfer, from [`FtdiPorts::read`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Fill {
    /// These bytes (no more than asked); empty is [`Fill::Idle`].
    Data(Vec<u8>),
    /// Nothing: a packet of the two status bytes alone, which a real
    /// chip sends when its latency timer runs out.
    Idle,
    /// Nothing yet: the transfer is held (NAK) and offered again.
    Wait,
}

/// What a model makes of the ports.
pub trait FtdiPorts {
    /// A request changed port `port` (0 based); `state` is after it.
    fn changed(&mut self, port: usize, event: Event, state: &PortState, v: &mut Violations) {
        let _ = (port, event, state, v);
    }

    /// The host wrote `data` to the port's OUT endpoint.
    fn write(&mut self, port: usize, data: &[u8], state: &PortState, v: &mut Violations) -> Reply;

    /// Up to `max` bytes the chip has for the host on the port.
    fn read(&mut self, port: usize, max: usize, state: &PortState, v: &mut Violations) -> Fill;

    /// `READ_PINS`: the levels of the port's eight data pins.
    fn pins(&mut self, port: usize, state: &PortState, v: &mut Violations) -> u8;
}

/// An FTDI chip on a board.
#[derive(Clone, Debug)]
pub struct Ftdi {
    config: FtdiConfig,
    descriptors: Descriptors,
    ports: Vec<PortState>,
    eeprom: Vec<u16>,
}

impl Ftdi {
    /// The chip of `config`, at power on, its EEPROM built from the
    /// configuration.
    pub fn new(config: FtdiConfig) -> Ftdi {
        let descriptors = build_descriptors(&config);
        let eeprom = build_eeprom(&config);
        let ports = vec![PortState::default(); usize::from(config.chip.ports())];
        Ftdi { config, descriptors, ports, eeprom }
    }

    /// The configuration.
    pub fn config(&self) -> &FtdiConfig {
        &self.config
    }

    /// The chip.
    pub fn chip(&self) -> Chip {
        self.config.chip
    }

    /// The descriptors.
    pub fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    /// Port `i` (0 based).
    pub fn port(&self, i: usize) -> &PortState {
        &self.ports[i]
    }

    /// Port `i`, to set its status bytes.
    pub fn port_mut(&mut self, i: usize) -> &mut PortState {
        &mut self.ports[i]
    }

    /// The EEPROM words.
    pub fn eeprom(&self) -> &[u16] {
        &self.eeprom
    }

    /// Replaces the EEPROM image; its length must be the chip's.
    pub fn set_eeprom(&mut self, words: Vec<u16>) {
        assert_eq!(words.len(), usize::from(self.chip().eeprom_words()));
        self.eeprom = words;
    }

    /// A bus reset: every port back to its power on state.
    pub fn reset(&mut self) {
        for p in &mut self.ports {
            let (m, l) = (p.modem_status, p.line_status);
            *p = PortState { modem_status: m, line_status: l, ..PortState::default() };
        }
    }

    /// Whether `SET_BAUD_RATE`'s `wIndex` is all divisor bits, with no
    /// port in it: the single port chips other than the FT232H, as
    /// libftdi sends it (the kernel sends the FT-X chips' like the
    /// FT232H's, which libftdi does not).
    fn baud_index_is_divisor(&self) -> bool {
        self.chip().ports() == 1 && !self.chip().high_speed()
    }

    /// The port of a request's `wIndex` low byte, 0 based.
    fn port_of(&self, low: u8, v: &mut Violations) -> Option<usize> {
        let n = self.chip().ports();
        let port = match low {
            0 | 1 if n == 1 => Some(0),
            1.. if low <= n => Some(usize::from(low - 1)),
            _ => None,
        };
        if port.is_none() {
            let allowed = if n == 1 { "0 or 1".to_owned() } else { format!("1 to {n}") };
            v.report(format!(
                "FTDI request for port index {low}; the {} takes {allowed}",
                self.chip().name()
            ));
        }
        port
    }

    /// A control transfer: the vendor requests. Anything else is
    /// reported and stalled.
    // One dispatch over the request set, kept in one place so that each
    // request's checks read next to its effect.
    #[allow(clippy::too_many_lines)]
    pub fn control(
        &mut self,
        ports: &mut dyn FtdiPorts,
        setup: Setup,
        data: &[u8],
        v: &mut Violations,
    ) -> Reply {
        use request as r;
        if setup.kind() != VENDOR || setup.recipient() != RECIPIENT_DEVICE {
            v.report(format!(
                "request {:#04x} {:#04x} value {:#06x} index {:#06x}: an FTDI chip answers \
                 vendor requests to the device only",
                setup.request_type, setup.request, setup.value, setup.index
            ));
            return Reply::Stall;
        }
        let (inward, length) = match setup.request {
            r::POLL_MODEM_STATUS | r::READ_EEPROM => (true, 2),
            r::GET_LATENCY_TIMER | r::READ_PINS => (true, 1),
            r::RESET
            | r::MODEM_CTRL
            | r::SET_FLOW_CTRL
            | r::SET_BAUD_RATE
            | r::SET_DATA
            | r::SET_EVENT_CHAR
            | r::SET_ERROR_CHAR
            | r::SET_LATENCY_TIMER
            | r::SET_BITMODE
            | r::WRITE_EEPROM
            | r::ERASE_EEPROM => (false, 0),
            other => {
                v.report(format!(
                    "vendor request {other:#04x} (value {:#06x} index {:#06x}): not one an \
                     FTDI chip has",
                    setup.value, setup.index
                ));
                return Reply::Stall;
            }
        };
        if setup.is_in() != inward {
            v.report(format!(
                "vendor request {:#04x} as {}; it is {}",
                setup.request,
                if setup.is_in() { "IN" } else { "OUT" },
                if inward { "IN" } else { "OUT" }
            ));
            return Reply::Stall;
        }
        if setup.length != length {
            v.report(format!(
                "vendor request {:#04x} with wLength {}; it has {length}",
                setup.request, setup.length
            ));
        }
        let _ = data;
        match setup.request {
            r::READ_EEPROM => return self.read_eeprom(setup, v),
            r::WRITE_EEPROM | r::ERASE_EEPROM => return self.write_eeprom(setup, v),
            _ => {}
        }
        let low = (setup.index & 0xff) as u8;
        let hi = setup.index >> 8;
        // The single port chips before the H take SET_BAUD_RATE's whole
        // wIndex as the divisor's high bits; the others its high byte,
        // the port in the low one as usual.
        let mut whole = setup.request == r::SET_BAUD_RATE && self.baud_index_is_divisor();
        // The kernel's ftdi_sio sends the FT-X chips' like the FT232H's,
        // divisor bits in the high byte, port 0 in the low one: a wIndex
        // libftdi's form (bits 16 and 17 alone) cannot have.
        if whole && self.chip() == Chip::Ft230x && low == 0 && hi != 0 {
            whole = false;
        }
        let port = if whole { Some(0) } else { self.port_of(low, v) };
        let Some(port) = port else { return Reply::Stall };
        let baud_bits = if whole { setup.index } else { hi };
        if setup.request != r::SET_BAUD_RATE && setup.request != r::SET_FLOW_CTRL && hi != 0 {
            v.report(format!(
                "vendor request {:#04x} with wIndex {:#06x}: the high byte is 0",
                setup.request, setup.index
            ));
        }
        let chip = self.chip();
        let state = &mut self.ports[port];
        let event = match setup.request {
            r::RESET => match setup.value {
                reset::SIO => Event::Reset,
                reset::FLUSH_FROM_HOST => Event::FlushFromHost,
                reset::FLUSH_TO_HOST => Event::FlushToHost,
                other => {
                    v.report(format!("RESET with wValue {other}; it takes 0, 1 or 2"));
                    return Reply::Stall;
                }
            },
            r::MODEM_CTRL => {
                let (levels, mask) = ((setup.value & 0xff) as u8, (setup.value >> 8) as u8);
                if levels & !3 != 0 || mask & !3 != 0 {
                    v.report(format!(
                        "MODEM_CTRL {:#06x}: only DTR (bit 0) and RTS (bit 1) exist",
                        setup.value
                    ));
                }
                if levels & !mask & 3 != 0 {
                    v.report(format!(
                        "MODEM_CTRL {:#06x}: a level set without its mask bit, which the chip \
                         ignores",
                        setup.value
                    ));
                }
                if mask & 1 != 0 {
                    state.dtr = levels & 1 != 0;
                }
                if mask & 2 != 0 {
                    state.rts = levels & 2 != 0;
                }
                Event::ModemCtrl
            }
            r::SET_FLOW_CTRL => {
                let mode = hi as u8;
                if !matches!(mode, 0 | 1 | 2 | 4) {
                    v.report(format!(
                        "SET_FLOW_CTRL mode {mode:#04x}: one of 0, RTS/CTS 1, DTR/DSR 2, \
                         XON/XOFF 4"
                    ));
                }
                if mode != 4 && setup.value != 0 {
                    v.report(format!(
                        "SET_FLOW_CTRL wValue {:#06x} (XON/XOFF characters) without XON/XOFF",
                        setup.value
                    ));
                }
                state.flow = mode;
                state.xon_xoff = setup.value;
                Event::Flow
            }
            r::SET_BAUD_RATE => match decode_divisor(chip, setup.value, baud_bits) {
                Ok(d) => {
                    state.divisor = Some(d);
                    Event::Baud
                }
                Err(e) => {
                    v.report(e);
                    return Reply::Stall;
                }
            },
            r::SET_DATA => {
                let bits = setup.value & 0xff;
                let parity = (setup.value >> 8) & 7;
                let stop = (setup.value >> 11) & 7;
                if !matches!(bits, 7 | 8) || parity > 4 || stop > 2 || setup.value & 0x8000 != 0 {
                    v.report(format!(
                        "SET_DATA {:#06x}: 7 or 8 data bits, parity 0 to 4, stop bits 0 to 2, \
                         break in bit 14",
                        setup.value
                    ));
                }
                state.line = setup.value;
                Event::Line
            }
            r::SET_EVENT_CHAR | r::SET_ERROR_CHAR => {
                if setup.value & 0xfe00 != 0 {
                    v.report(format!(
                        "vendor request {:#04x} {:#06x}: a character and the enable bit 8",
                        setup.request, setup.value
                    ));
                }
                if setup.request == r::SET_EVENT_CHAR {
                    state.event_char = setup.value;
                } else {
                    state.error_char = setup.value;
                }
                Event::Chars
            }
            r::SET_LATENCY_TIMER => {
                if setup.value == 0 || setup.value > 255 {
                    v.report(format!("SET_LATENCY_TIMER {}: 1 to 255 ms", setup.value));
                    return Reply::Stall;
                }
                state.latency = setup.value as u8;
                Event::Latency
            }
            r::GET_LATENCY_TIMER => {
                if setup.value != 0 {
                    v.report(format!("GET_LATENCY_TIMER with wValue {:#06x}", setup.value));
                }
                return Reply::Data(vec![state.latency]);
            }
            r::POLL_MODEM_STATUS => {
                if setup.value != 0 {
                    v.report(format!("POLL_MODEM_STATUS with wValue {:#06x}", setup.value));
                }
                return Reply::Data(state.status().to_vec());
            }
            r::READ_PINS => {
                if setup.value != 0 {
                    v.report(format!("READ_PINS with wValue {:#06x}", setup.value));
                }
                let pins = ports.pins(port, state, v);
                return Reply::Data(vec![pins]);
            }
            r::SET_BITMODE => {
                let (mask, mode) = ((setup.value & 0xff) as u8, (setup.value >> 8) as u8);
                if mode != bitmode::RESET {
                    if mode.count_ones() != 1 {
                        v.report(format!("SET_BITMODE mode {mode:#04x}: one mode bit at a time"));
                        return Reply::Stall;
                    }
                    if chip.modes(port as u8) & mode == 0 {
                        v.report(format!(
                            "SET_BITMODE mode {mode:#04x} on port {}: the {} has no such mode \
                             there",
                            port_letter(port),
                            chip.name()
                        ));
                        return Reply::Stall;
                    }
                }
                state.bitmode = mode;
                state.mask = mask;
                Event::Bitmode
            }
            _ => unreachable!("request {:#04x} classified above", setup.request),
        };
        ports.changed(port, event, state, v);
        Reply::ack()
    }

    fn read_eeprom(&self, setup: Setup, v: &mut Violations) -> Reply {
        let size = self.chip().eeprom_words();
        if setup.value != 0 {
            v.report(format!("READ_EEPROM with wValue {:#06x}", setup.value));
        }
        // libftdi's ftdi_read_eeprom reads 128 words whatever the chip
        // (FTDI_MAX_EEPROM_SIZE) and guesses the size from the halves,
        // so the words of a 64 word EEPROM are read twice; beyond 128
        // nothing reads.
        if setup.index >= size.max(128) {
            v.report(format!(
                "READ_EEPROM of word {:#06x}; the {} has {size} and libftdi reads 128",
                setup.index,
                self.chip().name()
            ));
        }
        // The serial EEPROM sees only its address bits.
        let w = self.eeprom[usize::from(setup.index % size)];
        Reply::Data(w.to_le_bytes().to_vec())
    }

    fn write_eeprom(&mut self, setup: Setup, v: &mut Violations) -> Reply {
        let size = self.chip().eeprom_words();
        if setup.request == request::ERASE_EEPROM {
            v.report("ERASE_EEPROM: no driver under test has a reason to erase the EEPROM");
            self.eeprom.fill(0xffff);
        } else {
            v.report(format!(
                "WRITE_EEPROM of word {:#06x}: no driver under test has a reason to write the \
                 EEPROM",
                setup.index
            ));
            if setup.index < size {
                self.eeprom[usize::from(setup.index)] = setup.value;
            }
        }
        Reply::ack()
    }

    /// The port of a bulk endpoint number: IN `1 + 2n`, OUT `2 + 2n`.
    fn port_of_endpoint(&self, ep: u8, is_in: bool) -> Option<usize> {
        let n = usize::from(self.chip().ports());
        let ep = usize::from(ep);
        let port = if is_in { ep.checked_sub(1)? } else { ep.checked_sub(2)? };
        (port % 2 == 0 && port / 2 < n).then_some(port / 2)
    }

    /// A bulk OUT transfer to endpoint number `ep`.
    pub fn out(
        &mut self,
        ports: &mut dyn FtdiPorts,
        ep: u8,
        data: &[u8],
        v: &mut Violations,
    ) -> Reply {
        let Some(port) = self.port_of_endpoint(ep, false) else {
            v.report(format!("OUT transfer to endpoint {ep}, which no port has"));
            return Reply::Stall;
        };
        ports.write(port, data, &self.ports[port], v)
    }

    /// A bulk IN transfer from endpoint number `ep`, up to `max`
    /// bytes: packets of `wMaxPacketSize` that each start with the two
    /// status bytes, the last one short.
    pub fn in_transfer(
        &mut self,
        ports: &mut dyn FtdiPorts,
        ep: u8,
        max: usize,
        v: &mut Violations,
    ) -> Reply {
        let Some(port) = self.port_of_endpoint(ep, true) else {
            v.report(format!("IN transfer from endpoint {ep}, which no port has"));
            return Reply::Stall;
        };
        let packet = usize::from(self.chip().packet_size());
        if max < 2 {
            v.report(format!(
                "IN transfer of {max} bytes; every packet starts with two status bytes"
            ));
            return Reply::Stall;
        }
        let payload = packet - 2;
        let rest = max % packet;
        let budget = max / packet * payload + rest.saturating_sub(2);
        let state = &self.ports[port];
        let status = state.status();
        let data = match ports.read(port, budget, state, v) {
            Fill::Wait => return Reply::Nak,
            Fill::Idle => Vec::new(),
            Fill::Data(d) => d,
        };
        if data.len() > budget {
            v.report(format!(
                "model bug: {} bytes for an IN transfer that holds {budget}",
                data.len()
            ));
        }
        let data = &data[..data.len().min(budget)];
        if rest != 0 && data.len() > max / packet * payload {
            v.report(format!(
                "IN transfer of {max} bytes, not a multiple of wMaxPacketSize {packet}: a real \
                 chip's full packet would overflow it"
            ));
        }
        let mut out = Vec::with_capacity(data.len() + 2 * (data.len() / payload + 1));
        for chunk in data.chunks(payload) {
            out.extend_from_slice(&status);
            out.extend_from_slice(chunk);
        }
        // Full packets alone do not end a transfer: a status-only
        // packet does, when there is room for one.
        if data.len() % payload == 0 && out.len() + 2 <= max {
            out.extend_from_slice(&status);
        }
        Reply::Data(out)
    }
}

/// `A`, `B`, ... for port `i`.
pub fn port_letter(i: usize) -> char {
    char::from(b'A' + (i as u8))
}

fn build_descriptors(c: &FtdiConfig) -> Descriptors {
    let mut strings = Vec::new();
    let mut index = |s: &Option<String>| -> u8 {
        s.as_ref().map_or(0, |s| {
            strings.push(s.clone());
            strings.len() as u8
        })
    };
    let i_manufacturer = index(&c.manufacturer);
    let i_product = index(&c.description);
    let i_serial = index(&c.serial);
    let packet = c.chip.packet_size();
    let mut body = Vec::new();
    for p in 0..c.chip.ports() {
        body.extend_from_slice(&descriptor::interface(p, 0, 2, [0xff, 0xff, 0xff], i_product));
        body.extend_from_slice(&descriptor::endpoint(0x81 + 2 * p, BULK, packet, 0));
        body.extend_from_slice(&descriptor::endpoint(0x02 + 2 * p, BULK, packet, 0));
    }
    let config = descriptor::configuration(1, 0, c.attributes, c.max_power, &body);
    let device = DeviceDescriptor {
        usb: 0x0200,
        class: [0, 0, 0],
        max_packet0: c.chip.max_packet0(),
        vendor: c.vendor,
        product: c.product,
        device: c.chip.bcd_device(),
        strings: [i_manufacturer, i_product, i_serial],
        configurations: 1,
    };
    Descriptors {
        speed: if c.chip.high_speed() { Speed::High } else { Speed::Full },
        device: device.bytes(),
        configs: vec![config],
        strings,
    }
}

/// The EEPROM image of a configuration, in the layout FTDI's tools and
/// libftdi write: ids at words 1 to 3, power at word 4, the string
/// pointers (byte offset, byte length) at bytes 0x0e to 0x13, the
/// strings as USB string descriptors from where libftdi puts them
/// (0x94 on the AM and BM, 0x98 on the R, which a 93C46 sees as 0x14
/// and 0x18; 0x9a on the FT2232H and FT4232H, 0xa0 on the FT232H and
/// the X chips), the CBUS functions of the R and
/// the FT232H at their defaults (no pin a GPIO, so `ftdi_sio` registers
/// no GPIO chip), and the checksum in the last word.
pub fn build_eeprom(c: &FtdiConfig) -> Vec<u16> {
    let words = usize::from(c.chip.eeprom_words());
    let mut b = vec![0u8; words * 2];
    b[2..4].copy_from_slice(&c.vendor.to_le_bytes());
    b[4..6].copy_from_slice(&c.product.to_le_bytes());
    b[6..8].copy_from_slice(&c.chip.bcd_device().to_le_bytes());
    b[8] = c.attributes;
    b[9] = (c.max_power / 2) as u8;
    // Use the serial number string.
    b[0x0a] = if c.serial.is_some() { 0x08 } else { 0 };
    if matches!(c.chip, Chip::Am | Chip::Bm | Chip::Ft2232c) {
        b[0x0c..0x0e].copy_from_slice(&0x0200u16.to_le_bytes());
    }
    match c.chip {
        // CBUS0 TXLED#, CBUS1 RXLED#, CBUS2 TXDEN, CBUS3 PWREN#, CBUS4 SLEEP#.
        Chip::R => b[0x14..0x17].copy_from_slice(&[0x23, 0x10, 0x05]),
        // ACBUS0 TRISTATE#, 1 RXLED#, 2 TXLED#, 3 TXRXLED#, 4 SLEEP#,
        // 5..9 TRISTATE (none a GPIO).
        Chip::Ft232h => b[0x18..0x1d].copy_from_slice(&[0x10, 0x32, 0x04, 0x00, 0x00]),
        _ => {}
    }
    // Where libftdi puts the strings; a 93C46 sees the low 7 bits.
    let pointer: u8 = match c.chip {
        Chip::Am | Chip::Bm => 0x94,
        Chip::Ft2232c => 0x96,
        Chip::R => 0x98,
        Chip::Ft2232h | Chip::Ft4232h => 0x9a,
        Chip::Ft232h | Chip::Ft230x => 0xa0,
    };
    let mut at = usize::from(pointer) % b.len();
    for (slot, s) in [&c.manufacturer, &c.description, &c.serial].into_iter().enumerate() {
        let Some(s) = s else { continue };
        let d = descriptor::string(s);
        // The checksum word stays free.
        if at + d.len() > b.len() - 2 {
            break;
        }
        b[0x0e + 2 * slot] = pointer + (at - usize::from(pointer) % b.len()) as u8;
        b[0x0f + 2 * slot] = d.len() as u8;
        b[at..at + d.len()].copy_from_slice(&d);
        at += d.len();
    }
    let mut w: Vec<u16> = b.chunks(2).map(|p| u16::from_le_bytes([p[0], p[1]])).collect();
    w[words - 1] = eeprom_checksum(&w[..words - 1]);
    w
}

/// FTDI's EEPROM checksum: from `0xaaaa`, each word taken in by
/// exclusive or and the sum rotated left by one.
pub fn eeprom_checksum(words: &[u16]) -> u16 {
    words.iter().fold(0xaaaa_u16, |sum, &w| (sum ^ w).rotate_left(1))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A port that counts the bytes it hands out and records events.
    #[derive(Default)]
    struct Counter {
        next: u8,
        events: Vec<(usize, Event)>,
        written: Vec<u8>,
        have: Option<usize>,
    }

    impl FtdiPorts for Counter {
        fn changed(&mut self, port: usize, event: Event, _: &PortState, _: &mut Violations) {
            self.events.push((port, event));
        }
        fn write(&mut self, _: usize, data: &[u8], _: &PortState, _: &mut Violations) -> Reply {
            self.written.extend_from_slice(data);
            Reply::ack()
        }
        fn read(&mut self, _: usize, max: usize, _: &PortState, _: &mut Violations) -> Fill {
            let n = self.have.map_or(max, |h| h.min(max));
            if n == 0 {
                return Fill::Idle;
            }
            Fill::Data(
                (0..n)
                    .map(|_| {
                        self.next = self.next.wrapping_add(1);
                        self.next
                    })
                    .collect(),
            )
        }
        fn pins(&mut self, _: usize, _: &PortState, _: &mut Violations) -> u8 {
            0x5a
        }
    }

    fn out(request: u8, value: u16, index: u16) -> Setup {
        Setup { request_type: 0x40, request, value, index, length: 0 }
    }

    fn inp(request: u8, value: u16, index: u16, length: u16) -> Setup {
        Setup { request_type: 0xc0, request, value, index, length }
    }

    #[test]
    fn divisors_as_libftdi_encodes_them() {
        // (chip, baud, value, bits, best): libftdi's own results.
        let cases = [
            (Chip::R, 9_600, 0x4138, 0, 9_600),
            (Chip::Bm, 115_200, 0x001a, 0, 115_385),
            (Chip::R, 3_000_000, 0, 0, 3_000_000),
            (Chip::R, 2_000_000, 1, 0, 2_000_000),
            (Chip::R, 480, 0x186a, 0, 480),
            (Chip::Ft2232h, 2_000_000, 6, 2, 2_000_000),
            (Chip::Ft2232h, 720, 0x5046, 1, 720),
            (Chip::Ft2232h, 12_000_000, 0, 2, 12_000_000),
        ];
        for (chip, baud, value, bits, best) in cases {
            assert_eq!(libftdi_divisor(chip, baud), (value, bits, best), "{chip:?} {baud}");
            let d = decode_divisor(chip, value, bits).unwrap();
            assert!(d.baud().abs_diff(best) <= 1, "{chip:?} {baud}: {} vs {best}", d.baud());
        }
    }

    #[test]
    fn divisors_an_120_rules_out() {
        assert!(decode_divisor(Chip::R, 0x4001, 0).is_err(), "1.5 + 1/2");
        assert!(decode_divisor(Chip::R, 0x8000, 0).is_err(), "0 + 1/4");
        assert!(decode_divisor(Chip::R, 6, 2).is_err(), "120 MHz switch on an R");
        assert!(decode_divisor(Chip::R, 6, 4).is_err(), "unknown index bit");
        assert!(decode_divisor(Chip::Am, 0x0003, 1).is_err(), "AM has no code 4 and up");
        assert_eq!(decode_divisor(Chip::R, 1, 0).unwrap().baud(), 2_000_000);
        assert_eq!(decode_divisor(Chip::R, 0, 0).unwrap().baud(), 3_000_000);
    }

    #[test]
    fn requests_as_libftdi_sends_them() {
        let mut chip = Ftdi::new(FtdiConfig::new(Chip::Ft2232h));
        let mut ports = Counter::default();
        let mut v = Violations::default();
        let c = |chip: &mut Ftdi, p: &mut Counter, s: Setup, v: &mut Violations| {
            chip.control(p, s, &[], v)
        };
        // ftdi_usb_open: reset, 9600 Bd; then purges, bit-bang.
        assert_eq!(c(&mut chip, &mut ports, out(0, 0, 1), &mut v), Reply::ack());
        let (value, bits, _) = libftdi_divisor(Chip::Ft2232h, 9600);
        let s = out(3, value, (bits << 8) | 1);
        assert_eq!(c(&mut chip, &mut ports, s, &mut v), Reply::ack());
        assert_eq!(chip.port(0).baud(), Some(9600));
        c(&mut chip, &mut ports, out(0, 1, 1), &mut v);
        c(&mut chip, &mut ports, out(0, 2, 1), &mut v);
        c(&mut chip, &mut ports, out(0x0b, 0x0000, 1), &mut v);
        c(&mut chip, &mut ports, out(0x0b, 0x01f0, 2), &mut v);
        assert_eq!(chip.port(1).bitmode, bitmode::BITBANG);
        assert_eq!(chip.port(1).mask, 0xf0);
        c(&mut chip, &mut ports, out(9, 2, 1), &mut v);
        assert_eq!(c(&mut chip, &mut ports, inp(0x0a, 0, 1, 1), &mut v), Reply::Data(vec![2]));
        assert_eq!(c(&mut chip, &mut ports, inp(0x0a, 0, 2, 1), &mut v), Reply::Data(vec![16]));
        assert_eq!(
            c(&mut chip, &mut ports, inp(5, 0, 1, 2), &mut v),
            Reply::Data(vec![0x01, 0x60])
        );
        assert_eq!(c(&mut chip, &mut ports, inp(0x0c, 0, 1, 1), &mut v), Reply::Data(vec![0x5a]));
        c(&mut chip, &mut ports, out(1, 0x0303, 1), &mut v);
        assert!(chip.port(0).dtr && chip.port(0).rts);
        c(&mut chip, &mut ports, out(1, 0x0200, 1), &mut v);
        assert!(chip.port(0).dtr && !chip.port(0).rts);
        c(&mut chip, &mut ports, out(2, 0, 0x0101), &mut v);
        c(&mut chip, &mut ports, out(2, 0x1311, 0x0401), &mut v);
        c(&mut chip, &mut ports, out(4, 0x0008 | (2 << 8) | (2 << 11), 1), &mut v);
        c(&mut chip, &mut ports, out(6, 0x017e, 1), &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(
            ports.events,
            [
                (0, Event::Reset),
                (0, Event::Baud),
                (0, Event::FlushFromHost),
                (0, Event::FlushToHost),
                (0, Event::Bitmode),
                (1, Event::Bitmode),
                (0, Event::Latency),
                (0, Event::ModemCtrl),
                (0, Event::ModemCtrl),
                (0, Event::Flow),
                (0, Event::Flow),
                (0, Event::Line),
                (0, Event::Chars),
            ]
        );
        // The EEPROM: ids, and a checksum that checks.
        let e = chip.eeprom().to_vec();
        assert_eq!(e.len(), 128);
        assert_eq!(&e[1..4], &[0x0403, 0x6010, 0x0700]);
        assert_eq!(eeprom_checksum(&e[..127]), e[127]);
        for (i, w) in e.iter().enumerate().take(4) {
            let s = inp(0x90, 0, i as u16, 2);
            assert_eq!(c(&mut chip, &mut ports, s, &mut v), Reply::Data(w.to_le_bytes().to_vec()));
        }
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_what_the_chip_does_not_take() {
        let mut chip = Ftdi::new(FtdiConfig::new(Chip::R));
        let mut ports = Counter::default();
        let mut v = Violations::default();
        let mut one = |s: Setup, expect: Reply, why: &str| {
            assert_eq!(chip.control(&mut ports, s, &[], &mut v), expect, "{why}");
            assert_eq!(v.drain().len(), 1, "{why}");
        };
        one(out(0x0b, 0x0200, 1), Reply::Stall, "MPSSE on an R");
        one(out(0x0b, 0x0300, 1), Reply::Stall, "two mode bits");
        one(out(0x0b, 0x0100, 2), Reply::Stall, "port B on a one port chip");
        one(out(0, 3, 1), Reply::Stall, "RESET 3");
        one(out(9, 0, 1), Reply::Stall, "latency 0");
        one(out(3, 0x4001, 0), Reply::Stall, "divisor 1.5 + 1/2");
        one(out(3, 6, 2), Reply::Stall, "120 MHz switch on an R");
        one(out(4, 0x0009, 1), Reply::ack(), "nine data bits");
        one(out(2, 0, 0x0301), Reply::ack(), "flow mode 3");
        one(out(1, 0x0001, 1), Reply::ack(), "DTR without its mask bit");
        one(out(0x0b, 0x0100, 0x0101), Reply::ack(), "a high wIndex byte");
        one(inp(0x0a, 0, 1, 2), Reply::Data(vec![16]), "latency read of 2");
        one(out(0x0a, 0, 1), Reply::Stall, "GET_LATENCY_TIMER as OUT");
        one(out(0x42, 0, 1), Reply::Stall, "unknown request");
        let class = Setup { request_type: 0x21, request: 0x20, value: 0, index: 0, length: 0 };
        one(class, Reply::Stall, "a class request");
        one(inp(0x90, 0, 128, 2), Reply::Data(vec![0, 0]), "EEPROM word 128");
        one(out(0x91, 0x1234, 3), Reply::ack(), "EEPROM write");
        assert_eq!(chip.eeprom()[3], 0x1234);
        // libftdi reads 128 words of a 64 word EEPROM: they wrap.
        let w3 = 0x1234u16.to_le_bytes().to_vec();
        assert_eq!(chip.control(&mut ports, inp(0x90, 0, 67, 2), &[], &mut v), Reply::Data(w3));
        assert!(v.is_empty(), "{v:?}");
        // Port B of a two port chip is index 2; 0 and 3 are not ports.
        let mut chip = Ftdi::new(FtdiConfig::new(Chip::Ft2232h));
        assert_eq!(chip.control(&mut ports, out(0, 0, 2), &[], &mut v), Reply::ack());
        assert!(v.is_empty());
        assert_eq!(chip.control(&mut ports, out(0, 0, 0), &[], &mut v), Reply::Stall);
        assert_eq!(chip.control(&mut ports, out(0, 0, 3), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 2);
        // Synchronous FIFO is port A's only.
        assert_eq!(chip.control(&mut ports, out(0x0b, 0x4000, 2), &[], &mut v), Reply::Stall);
        assert_eq!(chip.control(&mut ports, out(0x0b, 0x4000, 1), &[], &mut v), Reply::ack());
        assert_eq!(v.drain().len(), 1);
    }

    #[test]
    fn baud_index_forms() {
        let mut ports = Counter::default();
        let mut v = Violations::default();
        let near =
            |f: &Ftdi, port: usize, best: u32| f.port(port).baud().unwrap().abs_diff(best) <= 1;
        // A divisor of 2 3/8 has eighths code 4, so bit 16 set: libftdi
        // puts it alone in wIndex on an R or an X.
        let (value, bits, best) = libftdi_divisor(Chip::R, 1_263_158);
        assert_eq!((value, bits), (2, 1));
        for chip in [Chip::R, Chip::Ft230x] {
            let mut f = Ftdi::new(FtdiConfig::new(chip));
            assert_eq!(f.control(&mut ports, out(3, value, bits), &[], &mut v), Reply::ack());
            assert!(near(&f, 0, best), "{chip:?}");
        }
        // The kernel's form for the X: bits in the high byte, port 0.
        let mut f = Ftdi::new(FtdiConfig::new(Chip::Ft230x));
        assert_eq!(f.control(&mut ports, out(3, value, bits << 8), &[], &mut v), Reply::ack());
        assert!(near(&f, 0, best));
        // The multi port chips: bits in the high byte, the port in the low.
        let mut f = Ftdi::new(FtdiConfig::new(Chip::Ft4232h));
        let (value, bits, best) = libftdi_divisor(Chip::Ft4232h, 5_052_632);
        assert_eq!(bits, 3, "the 120 MHz switch and bit 16");
        assert_eq!(
            f.control(&mut ports, out(3, value, (bits << 8) | 4), &[], &mut v),
            Reply::ack()
        );
        assert!(near(&f, 3, best));
        assert!(v.is_empty(), "{v:?}");
        // The R does not take the high byte form.
        let mut f = Ftdi::new(FtdiConfig::new(Chip::R));
        assert_eq!(f.control(&mut ports, out(3, 2, 0x0100), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1);
    }

    #[test]
    fn in_packets_carry_status_bytes() {
        let mut chip = Ftdi::new(FtdiConfig::new(Chip::R));
        let mut ports = Counter::default();
        let mut v = Violations::default();
        // 128 bytes: two full packets of 62 data bytes.
        let Reply::Data(d) = chip.in_transfer(&mut ports, 1, 128, &mut v) else { panic!() };
        assert_eq!(d.len(), 128);
        assert_eq!(&d[..3], &[0x01, 0x60, 1]);
        assert_eq!(&d[64..67], &[0x01, 0x60, 63]);
        // Nothing to send: the status alone.
        ports.have = Some(0);
        assert_eq!(chip.in_transfer(&mut ports, 1, 4096, &mut v), Reply::Data(vec![0x01, 0x60]));
        // Exactly one packet's worth: a status-only packet ends it.
        ports.have = Some(62);
        let Reply::Data(d) = chip.in_transfer(&mut ports, 1, 4096, &mut v) else { panic!() };
        assert_eq!(d.len(), 66);
        assert_eq!(&d[64..], &[0x01, 0x60]);
        // Three bytes: one short packet.
        ports.have = Some(3);
        let Reply::Data(d) = chip.in_transfer(&mut ports, 1, 4096, &mut v) else { panic!() };
        assert_eq!(d.len(), 5);
        assert!(v.is_empty(), "{v:?}");
        // A transfer a full packet would overflow.
        ports.have = None;
        let Reply::Data(d) = chip.in_transfer(&mut ports, 1, 100, &mut v) else { panic!() };
        assert_eq!(d.len(), 100);
        assert_eq!(v.drain().len(), 1);
        assert_eq!(chip.in_transfer(&mut ports, 3, 64, &mut v), Reply::Stall);
        assert_eq!(chip.out(&mut ports, 2, b"xy", &mut v), Reply::ack());
        assert_eq!(ports.written, b"xy");
        assert_eq!(v.drain().len(), 1);
    }

    #[test]
    fn descriptors_of_each_chip() {
        for chip in [Chip::Bm, Chip::R, Chip::Ft2232h, Chip::Ft4232h, Chip::Ft232h, Chip::Ft230x] {
            let f = Ftdi::new(FtdiConfig::new(chip));
            let d = f.descriptors();
            assert_eq!(d.bcd_device(), chip.bcd_device());
            assert_eq!(d.vendor_id(), FTDI_VENDOR);
            let eps = Descriptors::endpoints(&d.configs[0]);
            assert_eq!(eps.len(), 2 * usize::from(chip.ports()));
            assert!(eps.iter().all(|e| e.max_packet == chip.packet_size()));
            assert_eq!(eps.last().unwrap().address, 2 * chip.ports());
            assert_eq!(d.speed == Speed::High, chip.high_speed());
            let e = f.eeprom();
            assert_eq!(eeprom_checksum(&e[..e.len() - 1]), e[e.len() - 1]);
        }
    }
}
