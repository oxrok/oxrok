// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The Silicon Labs CP2110 HID-to-UART bridge (`10c4:ea80`) with a
//! UNI-T UT61E (Cyrustek ES51922, 14 byte packets at 19200 baud, 7o1)
//! on its UART, as libsigrok's `serial-dmm` driver reads it through
//! `conn=hid/cp2110`.
//!
//! The chip, after Silicon Labs' AN434 (CP2110/4 Interface
//! Specification, rev 0.4):
//!
//! - UART data travels on the interrupt pipes in reports 0x01 to 0x3f,
//!   the report id being the number of data bytes behind it (1 to 63).
//!   The device sends exactly that many; libsigrok reads into a 64 byte
//!   buffer and takes the count from byte 0. Get Receive Data (5.2)
//!   also lets the host fetch them by `GET_REPORT` of an input report;
//!   the model answers it from the same FIFO.
//! - Configuration is feature reports on the control pipe, the report
//!   id in byte 0 of the payload: 0x41 Get/Set UART Enable (1 byte,
//!   0 or 1), 0x42 Get UART Status (TX FIFO and RX FIFO fill, two bytes
//!   each, error status, break status), 0x43 Set Purge FIFOs (1 byte:
//!   1 TX, 2 RX, 3 both), 0x46 Get Version Information (part number
//!   0x0a, device version), 0x50 Get/Set UART Config (baud rate as a
//!   32 bit big endian number, parity, flow control 0 or 1, data bits,
//!   stop bits 0 short or 1 long).
//! - A Get Report the chip does not know stalls (AN434 2.1.2).
//!
//! The UART receives whenever it is enabled, as a real one does: the
//! meter sends a packet every `period` polls of the interrupt IN
//! endpoint (default 25, about half a second, a UT61E's rate), counted
//! rather than timed (see `shells::hidcable::Stream`) so that both
//! tools see the same packets. A packet reaches the FIFO in two parts,
//! 9 bytes and then, 3 polls later, the other 5, as the bytes of a
//! packet trickle in over the wire: a read can end inside a packet,
//! which the driver keeps for the next one. The FIFO goes to the host
//! in reports of what it holds, up to 63 bytes. Every Set UART Enable
//! of 1 (each open, in `cp2110_set_params`) starts the period over, so
//! a host that opens and closes again at once (`--show` after its
//! scan) sees no packet, and a purge empties the FIFO.
//!
//! Option `burst=N`: at most N packets after each Set UART Enable (each
//! open), then silence until the next (default: no limit). It makes a
//! scan that finds nothing deterministic: such a scan listens for 3 s
//! of wall time, in which an unlimited stream sends a varying number of
//! packets, while a burst is sent well inside them. `burst=0` is a
//! meter that never sends.
//!
//! The meter plays eight readings (DC volts, negative DC volts with
//! hold, AC volts, ohms, DC milliamps, capacitance, frequency, an
//! overload) and then ten packets of noise (`?` bytes, no CR LF), which
//! the driver has to search through, and again; the last digit counts
//! the cycles. A model reset starts over at the first.
//!
//! What the line settings do to the bytes: at another baud rate than
//! 19200 every byte is received as a garbled one (framing errors; the
//! model maps each byte to another, one for one, rather than modelling
//! bit timing); with 8 data bits the meter's parity bit lands in bit 7;
//! with 5 or 6 data bits a byte is cut to its low bits (a real UART
//! would also misframe the next start bit; the model does not). A
//! parity setting that the meter's odd parity bit does not satisfy
//! sets the parity error bit of Get UART Status; the bytes still
//! arrive. More than the FIFO's 480 bytes sets the overrun bit and
//! drops the rest.
//!
//! The pedantic checks: feature report ids, lengths (report id
//! included) and values as above, the report id in byte 0 of a
//! `SET_REPORT` agreeing with `wValue`, a baud rate within 300 to
//! 1000000 (libsigrok's `CP2110_BAUDRATE_MAX`; AN434 rev 0.4 says
//! 500000, later parts 1000000), data bits 0 to 3, parity 0 to 4, data
//! reports only once the UART is enabled, and with a byte count between
//! 1 and 63 that fits the transfer. Reports the model does not
//! implement (GPIO, lock bytes, line break, the PROM customisation)
//! stall and are flagged: neither tool has a reason to send them.
//!
//! Habits of the C code that the chip's documentation does not back,
//! documented here rather than flagged so that a sigrok-cli run is
//! clean:
//!
//! - `cp2110_flush` purges with `hid_write`, an output report 0x43 on
//!   the interrupt pipe, where AN434 (Table 7) has Set Purge FIFOs as a
//!   feature report on the control pipe (as pyserial's CP2110 backend
//!   sends it). Whether a real chip honours 0x43 on its interrupt OUT
//!   pipe is not documented; the model takes both as the purge.
//! - The codes: AN434 rev 0.4 has parity 1 even, 2 odd and data bits
//!   0x05 to 0x08; pyserial's backend sends parity 1 odd, 2 even and
//!   data bits 0 to 3. libsigrok mixes the two, rev 0.4's parity with
//!   data bits 0 to 3. The model takes data bits 0 to 3 and reads
//!   parity as pyserial does (unverified on a chip), so libsigrok's 7o1
//!   (code 2) is checked as even parity: only the error bit shows it,
//!   which no host reads.
//!
//! What the model showed about libsigrok (0bc24877):
//!
//! - It finds a CP2110 only by path, `conn=hid/cp2110/raw=/dev/hidrawN`:
//!   its search (`conn=hid`, `hid/cp2110`, `hid/cp2110/sn=...`,
//!   `ser_hid_chip_search`) converts the searched path twice and hands
//!   hidapi the port name to open, whatever the hidapi backend.
//! - `cp2110_set_params` reads `serialcomm`'s flow (1 RTS/CTS, 2
//!   XON/XOFF) with libserialport's numbering (1 XON/XOFF, 2 RTS/CTS):
//!   it refuses `flow=1` and turns RTS/CTS on for `flow=2`.
//!
//! The device is full speed with a 64 byte endpoint 0, interrupt IN
//! 0x81 and interrupt OUT 0x01 of 64 bytes polled every millisecond,
//! bus powered at 100 mA, with Silicon Labs' strings.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::hid::{self, HidInterface, HidReports};
use crate::shells::hidcable::Stream;
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "cp2110";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Silicon Labs CP2110 HID-to-UART bridge (10c4:ea80) with a UNI-T UT61E stream; \
     period=N polls per packet (5 to 1000, default 25), serial=TEXT (default 0001A5F8), \
     burst=N (packets per open)";

/// A cable from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut period = 25;
    let mut burst = None;
    let mut serial = DEFAULT_SERIAL.to_owned();
    for (k, v) in options {
        match k.as_str() {
            "period" => period = crate::shells::hidcable::parse_number(k, v, 5..=1000)?,
            "burst" => burst = crate::shells::hidcable::parse_burst(v)?,
            "serial" => {
                if v.is_empty() || v.len() > 30 {
                    return Err(Error::Invalid(format!("serial={v}: 1 to 30 characters")));
                }
                serial.clone_from(v);
            }
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let mut cable = Cp2110::new(period, &serial);
    cable.chip = Chip::new(period, burst);
    Ok(Box::new(cable))
}

/// The USB vendor id.
pub const VENDOR_ID: u16 = 0x10c4;
/// The USB product id.
pub const PRODUCT_ID: u16 = 0xea80;
/// The serial number string unless `serial=` says otherwise.
pub const DEFAULT_SERIAL: &str = "0001A5F8";

/// Get/Set UART Enable.
pub const UART_ENABLE: u8 = 0x41;
/// Get UART Status.
pub const UART_STATUS: u8 = 0x42;
/// Set Purge FIFOs.
pub const PURGE_FIFOS: u8 = 0x43;
/// Get Version Information.
pub const VERSION: u8 = 0x46;
/// Get/Set UART Config.
pub const UART_CONFIG: u8 = 0x50;
/// The largest UART data report, 63 bytes behind the id.
pub const MAX_DATA: usize = 63;
/// The part number Get Version Information reports.
pub const PART_NUMBER: u8 = 0x0a;
/// The device version it reports.
pub const DEVICE_VERSION: u8 = 0x01;
/// The RX FIFO's size.
pub const FIFO_BYTES: usize = 480;

/// Purge type bits.
const PURGE_TX: u8 = 0x01;
const PURGE_RX: u8 = 0x02;

/// Parity codes as pyserial has them.
const PARITY_ODD: u8 = 1;
/// Error status: a parity error since the last read.
const ERROR_PARITY: u8 = 0x01;
/// Error status: the RX FIFO overran.
const ERROR_OVERRUN: u8 = 0x02;

/// The meter's line: 19200 baud, 7 data bits, odd parity, 1 stop bit.
pub const METER_BAUD: u32 = 19200;
/// The meter's packet length.
pub const PACKET_BYTES: usize = 14;
/// The bytes of a packet that reach the FIFO first.
const FIRST_PART: usize = 9;
/// Polls after which the rest of a packet follows.
const REST_POLLS: u32 = 3;
/// Readings, then noise packets, per cycle of the meter.
const READINGS: u64 = 8;
const NOISE: u64 = 10;

/// Every feature report the model implements: id, payload length after
/// the id, and whether it can be got and set.
const FEATURES: [(u8, usize, bool, bool); 5] = [
    (UART_ENABLE, 1, true, true),
    (UART_STATUS, 6, true, false),
    (PURGE_FIFOS, 1, false, true),
    (VERSION, 2, true, false),
    (UART_CONFIG, 8, true, true),
];

/// The report descriptor: a vendor page application collection with
/// input and output reports 1 to 63 of as many bytes as their id, and
/// the feature reports of [`FEATURES`].
pub fn report_descriptor() -> Vec<u8> {
    let mut d = vec![
        0x06, 0x00, 0xff, // usage page, vendor defined
        0x09, 0x01, // usage 1
        0xa1, 0x01, // collection, application
        0x15, 0x00, // logical minimum 0
        0x26, 0xff, 0x00, // logical maximum 255
        0x75, 0x08, // report size 8
    ];
    for id in 1..=MAX_DATA as u8 {
        d.extend_from_slice(&[0x85, id, 0x95, id, 0x09, 0x01, 0x81, 0x02, 0x09, 0x01, 0x91, 0x02]);
    }
    for (id, len, _, _) in FEATURES {
        d.extend_from_slice(&[0x85, id, 0x95, len as u8, 0x09, 0x01, 0xb1, 0x02]);
    }
    d.push(0xc0);
    d
}

/// One UT61E packet: range, five digits, function, status, four option
/// bytes, CR LF; each of the first twelve bytes is `0x30` plus a
/// nibble or an ASCII digit.
fn packet(range: u8, digits: [u8; 5], function: u8, status: u8, options: [u8; 4]) -> [u8; 14] {
    let mut p = [0u8; PACKET_BYTES];
    p[0] = b'0' + range;
    p[1..6].copy_from_slice(&digits);
    p[6] = function;
    p[7] = 0x30 | status;
    for (i, o) in options.iter().enumerate() {
        p[8 + i] = 0x30 | o;
    }
    p[12] = b'\r';
    p[13] = b'\n';
    p
}

// Option 3 bits: DC, AC, auto range.
const DC: u8 = 1 << 3;
const AC: u8 = 1 << 2;
const AUTO: u8 = 1 << 1;
// Status bits: sign, overload; option 4: hold.
const SIGN: u8 = 1 << 2;
const OL: u8 = 1 << 0;
const HOLD: u8 = 1 << 1;

/// Packet `n` of the meter's stream: eight readings in turn, the last
/// digit counting the cycles, then ten packets of noise.
pub fn meter_packet(n: u64) -> [u8; PACKET_BYTES] {
    let cycle_len = READINGS + NOISE;
    let cycle = b'0' + (n / cycle_len % 10) as u8;
    let d = |s: &[u8; 4]| [s[0], s[1], s[2], s[3], cycle];
    match n % cycle_len {
        // 1.2345 V DC, auto range.
        0 => packet(0, d(b"1234"), 0x3b, 0, [0, 0, DC | AUTO, 0]),
        // -4.7110 V DC, held.
        1 => packet(1, d(b"0471"), 0x3b, SIGN, [0, 0, DC | AUTO, HOLD]),
        // 230.00 V AC.
        2 => packet(2, d(b"2300"), 0x3b, 0, [0, 0, AC | AUTO, 0]),
        // 100.00 kOhm.
        3 => packet(3, d(b"1000"), 0x33, 0, [0, 0, AUTO, 0]),
        // 15.000 mA DC.
        4 => packet(0, d(b"1500"), 0x3f, 0, [0, 0, DC | AUTO, 0]),
        // 4.7000 uF.
        5 => packet(3, d(b"0470"), 0x36, 0, [0, 0, AUTO, 0]),
        // 5000.0 Hz.
        6 => packet(2, d(b"0500"), 0x32, 0, [0, 0, AUTO, 0]),
        // Overload on the resistance range.
        7 => packet(6, d(b"0000"), 0x33, OL, [0, 0, AUTO, 0]),
        // Noise.
        _ => [b'?'; PACKET_BYTES],
    }
}

/// The UART settings, as Get/Set UART Config carries them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Bits per second.
    pub baud: u32,
    /// Parity code, 0 to 4.
    pub parity: u8,
    /// Flow control, 0 none or 1 RTS/CTS.
    pub flow: u8,
    /// Data bits code, 0 to 3 for 5 to 8 bits.
    pub data_bits: u8,
    /// Stop bits, 0 short or 1 long.
    pub stop_bits: u8,
}

impl Config {
    /// The power-on settings (AN434 1.3): 115200 baud, 8N1.
    pub const DEFAULT: Config =
        Config { baud: 115_200, parity: 0, flow: 0, data_bits: 3, stop_bits: 0 };

    fn bytes(self) -> [u8; 8] {
        let b = self.baud.to_be_bytes();
        [b[0], b[1], b[2], b[3], self.parity, self.flow, self.data_bits, self.stop_bits]
    }
}

/// The chip behind the reports, and the meter on its UART.
#[derive(Clone, Debug)]
pub struct Chip {
    /// UART enabled.
    pub enabled: bool,
    /// The UART settings.
    pub config: Config,
    /// Error status bits since the last Get UART Status.
    pub errors: u8,
    /// The RX FIFO.
    pub fifo: VecDeque<u8>,
    /// The meter's next packet.
    pub next_packet: u64,
    /// When the meter's packets are due.
    stream: Stream,
    /// The part of the last packet still on the wire, and the polls
    /// until it is in.
    rest: Vec<u8>,
    rest_in: u32,
    period: u32,
    /// Packets per Set UART Enable; `None` for no limit.
    burst: Option<u32>,
}

impl Chip {
    fn new(period: u32, burst: Option<u32>) -> Chip {
        Chip {
            enabled: false,
            config: Config::DEFAULT,
            errors: 0,
            fifo: VecDeque::new(),
            next_packet: 0,
            stream: Stream::new(period, false).with_burst(burst),
            rest: Vec::new(),
            rest_in: 0,
            period,
            burst,
        }
    }

    /// One offer of the interrupt IN transfer: the meter's bytes that
    /// are due go into the FIFO.
    fn tick(&mut self) {
        if !self.enabled {
            return;
        }
        if self.rest_in > 0 {
            self.rest_in -= 1;
            if self.rest_in == 0 {
                let rest = std::mem::take(&mut self.rest);
                self.receive(&rest);
            }
        }
        if self.stream.poll() {
            let p = meter_packet(self.next_packet);
            self.next_packet += 1;
            self.receive(&p[..FIRST_PART]);
            self.rest = p[FIRST_PART..].to_vec();
            self.rest_in = REST_POLLS;
        }
    }

    /// Set UART Enable: on starts the meter's period over, off stops
    /// the reception.
    fn enable(&mut self, on: bool) {
        self.enabled = on;
        self.rest.clear();
        self.rest_in = 0;
        if on {
            self.stream.restart();
        } else {
            self.stream = Stream::new(self.period, false).with_burst(self.burst);
        }
    }

    /// Set Purge FIFOs, by feature report or (libsigrok's way) by output
    /// report.
    fn purge(&mut self, kind: u8, v: &mut Violations) {
        if kind & !(PURGE_TX | PURGE_RX) != 0 || kind == 0 {
            v.report(format!("purge type {kind:#04x}; AN434 has 1 (TX), 2 (RX) or 3 (both)"));
            return;
        }
        if kind & PURGE_RX != 0 {
            self.fifo.clear();
        }
    }

    /// The meter's 7o1 bytes as this UART configuration receives them.
    fn receive(&mut self, bytes: &[u8]) {
        let bits = self.config.data_bits + 5;
        let garbled = self.config.baud != METER_BAUD;
        let framed = bytes.iter().map(|&b| {
            let b = if garbled {
                // A framing error, one for one.
                b.wrapping_mul(0x9d) ^ 0x5a
            } else if bits == 8 {
                // The parity bit, odd, lands in bit 7.
                b | (u8::from(b.count_ones() % 2 == 0) << 7)
            } else {
                b
            };
            if bits < 8 { b & ((1u8 << bits) - 1) } else { b }
        });
        // Parity is checked on the bit after the data bits: with 7 data
        // bits that is the meter's odd parity bit; with 8 it is the stop
        // bit, which is 1.
        let parity_ok = match (bits, self.config.parity) {
            (_, 0) => true,
            (7, p) => p == PARITY_ODD,
            _ => false,
        };
        if garbled || !parity_ok {
            self.errors |= ERROR_PARITY;
        }
        for b in framed {
            if self.fifo.len() < FIFO_BYTES {
                self.fifo.push_back(b);
            } else {
                self.errors |= ERROR_OVERRUN;
            }
        }
    }

    /// Up to `max` bytes of the FIFO as a data report: count, then
    /// the bytes.
    fn data_report(&mut self, max: usize) -> Vec<u8> {
        let n = self.fifo.len().min(max).min(MAX_DATA);
        let mut r = Vec::with_capacity(n + 1);
        r.push(n as u8);
        r.extend(self.fifo.drain(..n));
        r
    }

    /// An output report on the interrupt pipe (or, flagged, by
    /// `SET_REPORT`): UART data, or libsigrok's purge.
    fn output(&mut self, data: &[u8], v: &mut Violations) {
        let Some(&id) = data.first() else {
            v.report("empty output report");
            return;
        };
        if id == PURGE_FIFOS {
            // libsigrok's cp2110_flush: hid_write of { 0x43, 0x03 }.
            if data.len() != 2 {
                v.report(format!("purge output report of {} bytes, needs 2", data.len()));
            }
            self.purge(data.get(1).copied().unwrap_or(0), v);
            return;
        }
        let count = usize::from(id);
        if !(1..=MAX_DATA).contains(&count) {
            v.report(format!("output report id {id:#04x}: data reports are 0x01 to 0x3f"));
            return;
        }
        if data.len() < count + 1 {
            v.report(format!(
                "data report of {count} bytes in a transfer of {}, too short",
                data.len()
            ));
        }
        if !self.enabled {
            v.report("UART data before the UART is enabled");
        }
        v.report(format!(
            "{count} bytes sent to the meter, whose cable has no receive line (the UT61E only talks)"
        ));
    }
}

impl HidReports for Chip {
    fn get_report(&mut self, kind: u8, id: u8, length: usize, v: &mut Violations) -> Reply {
        if kind == hid::INPUT {
            // Get Receive Data (AN434 5.2).
            if !(1..=MAX_DATA as u8).contains(&id) {
                v.report(format!("GET_REPORT of input report {id:#04x}: data reports are 0x01 to 0x3f"));
                return Reply::Stall;
            }
            let mut r = self.data_report(usize::from(id));
            r.truncate(length);
            return Reply::Data(r);
        }
        if kind != hid::FEATURE {
            v.report(format!("GET_REPORT of kind {kind} id {id:#04x}"));
            return Reply::Stall;
        }
        let Some(&(_, len, _, _)) = FEATURES.iter().find(|f| f.0 == id && f.2) else {
            v.report(format!("GET_REPORT of feature {id:#04x}, which the model cannot be got"));
            return Reply::Stall;
        };
        if length != len + 1 {
            v.report(format!(
                "GET_REPORT of feature {id:#04x} with {length} bytes; it has {}",
                len + 1
            ));
        }
        let mut r = vec![id];
        match id {
            UART_ENABLE => r.push(u8::from(self.enabled)),
            UART_STATUS => {
                // TX drains at once: the meter takes nothing.
                r.extend_from_slice(&0u16.to_be_bytes());
                r.extend_from_slice(&(self.fifo.len() as u16).to_be_bytes());
                r.push(self.errors);
                r.push(0);
                self.errors = 0;
            }
            VERSION => r.extend_from_slice(&[PART_NUMBER, DEVICE_VERSION]),
            _ => r.extend_from_slice(&self.config.bytes()),
        }
        r.truncate(length);
        Reply::Data(r)
    }

    fn set_report(&mut self, kind: u8, id: u8, data: &[u8], v: &mut Violations) -> Reply {
        if data.first() != Some(&id) {
            v.report(format!(
                "SET_REPORT {id:#04x} whose payload starts with {:?}, not the report id",
                data.first()
            ));
        }
        if kind == hid::OUTPUT {
            v.report("output report by SET_REPORT; the CP2110 takes them on its interrupt OUT pipe");
            self.output(data, v);
            return Reply::ack();
        }
        if kind != hid::FEATURE {
            v.report(format!("SET_REPORT of kind {kind}"));
            return Reply::Stall;
        }
        let Some(&(_, len, _, _)) = FEATURES.iter().find(|f| f.0 == id && f.3) else {
            v.report(format!("SET_REPORT of feature {id:#04x}, which the model cannot be set"));
            return Reply::Stall;
        };
        if data.len() != len + 1 {
            v.report(format!(
                "SET_REPORT of feature {id:#04x} with {} bytes; it has {}",
                data.len(),
                len + 1
            ));
            return Reply::Stall;
        }
        let p = &data[1..];
        match id {
            UART_ENABLE => match p[0] {
                0 => self.enable(false),
                1 => self.enable(true),
                other => v.report(format!("UART enable {other}; 0 or 1")),
            },
            PURGE_FIFOS => self.purge(p[0], v),
            _ => {
                let c = Config {
                    baud: u32::from_be_bytes([p[0], p[1], p[2], p[3]]),
                    parity: p[4],
                    flow: p[5],
                    data_bits: p[6],
                    stop_bits: p[7],
                };
                let mut legal = true;
                let mut bad = |what: String| {
                    v.report(what);
                    legal = false;
                };
                if !(300..=1_000_000).contains(&c.baud) {
                    bad(format!("baud rate {} (bytes {:02x?}): 300 to 1000000", c.baud, &p[..4]));
                }
                if c.parity > 4 {
                    bad(format!("parity {}: 0 to 4", c.parity));
                }
                if c.flow > 1 {
                    bad(format!("flow control {}: 0 or 1", c.flow));
                }
                if c.data_bits > 3 {
                    bad(format!("data bits code {}: 0 to 3 (5 to 8 bits)", c.data_bits));
                }
                if c.stop_bits > 1 {
                    bad(format!("stop bits {}: 0 or 1", c.stop_bits));
                }
                if legal {
                    self.config = c;
                }
            }
        }
        Reply::ack()
    }
}

/// The emulated cable.
pub struct Cp2110 {
    descriptors: Descriptors,
    hid: HidInterface,
    chip: Chip,
}

impl Cp2110 {
    /// A cable whose meter sends a packet every `period` polls, with
    /// this USB serial number.
    pub fn new(period: u32, serial: &str) -> Cp2110 {
        let hid = HidInterface::new(0, report_descriptor());
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 2, [3, 0, 0], 0));
        body.extend_from_slice(&hid.descriptor());
        body.extend_from_slice(&descriptor::endpoint(0x81, 3, 64, 1));
        body.extend_from_slice(&descriptor::endpoint(0x01, 3, 64, 1));
        let config = descriptor::configuration(1, 0, 0x80, 100, &body);
        let device = DeviceDescriptor {
            usb: 0x0200,
            class: [0, 0, 0],
            max_packet0: 64,
            vendor: VENDOR_ID,
            product: PRODUCT_ID,
            device: 0x0100,
            strings: [1, 2, 3],
            configurations: 1,
        };
        Cp2110 {
            descriptors: Descriptors {
                speed: Speed::Full,
                device: device.bytes(),
                configs: vec![config],
                strings: vec![
                    "Silicon Laboratories".to_owned(),
                    "CP2110 HID USB-to-UART Bridge".to_owned(),
                    serial.to_owned(),
                ],
            },
            hid,
            chip: Chip::new(period, None),
        }
    }

    /// The chip state.
    pub fn chip(&self) -> &Chip {
        &self.chip
    }
}

impl DeviceModel for Cp2110 {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if let Some(reply) = self.hid.control(&mut self.chip, setup, data, v) {
            return reply;
        }
        v.report(format!(
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the CP2110 answers",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, _ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.output(data, v);
        Reply::ack()
    }

    fn in_transfer(&mut self, _ep: u8, max: usize, v: &mut Violations) -> Reply {
        self.chip.tick();
        if self.chip.fifo.is_empty() {
            return Reply::Nak;
        }
        if max < 2 {
            v.report(format!("interrupt IN of {max} bytes; a data report needs 2 or more"));
            return Reply::Stall;
        }
        Reply::Data(self.chip.data_report(max - 1))
    }

    fn reset(&mut self) {
        // A bus reset restarts the chip with its power-on settings, and
        // the meter with its first packet.
        self.chip = Chip::new(self.chip.period, self.chip.burst);
        self.hid.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// No report.
    const NONE: Vec<Vec<u8>> = Vec::new();

    fn set_feature(dev: &mut Cp2110, data: &[u8], v: &mut Violations) -> Reply {
        let s = Setup {
            request_type: 0x21,
            request: 9,
            value: 0x0300 | u16::from(data[0]),
            index: 0,
            length: data.len() as u16,
        };
        dev.control(s, data, v)
    }

    fn get_report(dev: &mut Cp2110, kind: u8, id: u8, length: u16, v: &mut Violations) -> Reply {
        let s = Setup {
            request_type: 0xa1,
            request: 1,
            value: u16::from(kind) << 8 | u16::from(id),
            index: 0,
            length,
        };
        dev.control(s, &[], v)
    }

    /// What `cp2110_set_params` and `cp2110_flush` send for 19200/7o1.
    fn open_like_libsigrok(dev: &mut Cp2110, v: &mut Violations) {
        assert_eq!(set_feature(dev, &[0x41, 1], v), Reply::ack());
        assert_eq!(set_feature(dev, &[0x50, 0, 0, 0x4b, 0, 2, 0, 2, 0], v), Reply::ack());
        assert_eq!(dev.out(1, &[0x43, 0x03], v), Reply::ack());
    }

    /// The reports of `polls` offers of the interrupt IN transfer.
    fn poll(dev: &mut Cp2110, polls: u32, v: &mut Violations) -> Vec<Vec<u8>> {
        let mut reports = Vec::new();
        for _ in 0..polls {
            if let Reply::Data(r) = dev.in_transfer(1, 64, v) {
                reports.push(r);
            }
        }
        reports
    }

    fn bytes(reports: &[Vec<u8>]) -> Vec<u8> {
        reports.iter().flat_map(|r| r[1..].to_vec()).collect()
    }

    #[test]
    fn streams_a_packet_per_period_in_two_parts() {
        let mut dev = Cp2110::new(5, DEFAULT_SERIAL);
        let mut v = Violations::default();
        // Nothing before the UART is up, however long.
        assert_eq!(poll(&mut dev, 20, &mut v), NONE);
        open_like_libsigrok(&mut dev, &mut v);
        // Poll 5: the first 9 bytes; poll 8: the other 5.
        assert_eq!(poll(&mut dev, 4, &mut v), NONE);
        let first = poll(&mut dev, 1, &mut v);
        assert_eq!(first, [[&[9u8][..], &meter_packet(0)[..9]].concat()]);
        assert_eq!(poll(&mut dev, 2, &mut v), NONE);
        let rest = poll(&mut dev, 1, &mut v);
        assert_eq!(rest, [[&[5u8][..], &meter_packet(0)[9..]].concat()]);
        // And on: packet 1 at poll 10.
        assert_eq!(bytes(&poll(&mut dev, 5, &mut v)), meter_packet(1));
        assert!(v.is_empty(), "{v:?}");
        // libsigrok's parity code 2 is even under pyserial's reading.
        assert_eq!(dev.chip().errors, ERROR_PARITY);
        let s = get_report(&mut dev, 3, 0x42, 7, &mut v);
        assert_eq!(s, Reply::Data(vec![0x42, 0, 0, 0, 0, 1, 0]));
        assert_eq!(dev.chip().errors, 0);
        assert_eq!(get_report(&mut dev, 3, 0x46, 3, &mut v), Reply::Data(vec![0x46, 0x0a, 0x01]));
        assert_eq!(
            get_report(&mut dev, 3, 0x50, 9, &mut v),
            Reply::Data(vec![0x50, 0, 0, 0x4b, 0, 2, 0, 2, 0])
        );
        assert_eq!(get_report(&mut dev, 3, 0x41, 2, &mut v), Reply::Data(vec![0x41, 1]));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_purge_drops_what_the_fifo_holds_and_enable_restarts_the_period() {
        let mut dev = Cp2110::new(5, DEFAULT_SERIAL);
        let mut v = Violations::default();
        open_like_libsigrok(&mut dev, &mut v);
        // Packet 0's first part is in the FIFO, unread (no IN offer
        // takes it: Get Receive Data peeks at the fill instead).
        for _ in 0..5 {
            dev.chip.tick();
        }
        assert_eq!(dev.chip().fifo.len(), 9);
        set_feature(&mut dev, &[0x43, 0x02], &mut v);
        assert!(dev.chip().fifo.is_empty(), "the feature report purge");
        // The rest still comes: it was on the wire.
        assert_eq!(bytes(&poll(&mut dev, 3, &mut v)), meter_packet(0)[9..]);
        // A new open starts the period over: packet 1 five polls later.
        open_like_libsigrok(&mut dev, &mut v);
        assert_eq!(poll(&mut dev, 4, &mut v), NONE);
        assert_eq!(bytes(&poll(&mut dev, 4, &mut v)), meter_packet(1));
        // Disabled, the UART receives nothing.
        set_feature(&mut dev, &[0x41, 0], &mut v);
        assert_eq!(poll(&mut dev, 20, &mut v), NONE);
        // A reset starts the meter over.
        dev.reset();
        open_like_libsigrok(&mut dev, &mut v);
        assert_eq!(bytes(&poll(&mut dev, 8, &mut v)), meter_packet(0));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn get_receive_data_reads_the_fifo() {
        let mut dev = Cp2110::new(5, DEFAULT_SERIAL);
        let mut v = Violations::default();
        open_like_libsigrok(&mut dev, &mut v);
        for _ in 0..5 {
            dev.chip.tick();
        }
        let r = get_report(&mut dev, 1, 4, 64, &mut v);
        assert_eq!(r, Reply::Data([&[4u8][..], &meter_packet(0)[..4]].concat()));
        let r = get_report(&mut dev, 1, 63, 64, &mut v);
        assert_eq!(r, Reply::Data([&[5u8][..], &meter_packet(0)[4..9]].concat()));
        assert_eq!(get_report(&mut dev, 1, 63, 64, &mut v), Reply::Data(vec![0]), "empty");
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(get_report(&mut dev, 1, 0x40, 64, &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "input report 0x40");
    }

    #[test]
    fn readings_and_noise() {
        for n in 0..36 {
            let p = meter_packet(n);
            if n % 18 < 8 {
                assert_eq!(&p[12..], b"\r\n");
                assert!(p[..12].iter().all(|b| (0x30..=0x3f).contains(b)), "{p:?}");
                assert!(p[1..6].iter().all(u8::is_ascii_digit));
            } else {
                assert!(!p.contains(&b'\n'), "noise is no packet");
            }
        }
        assert_eq!(meter_packet(18)[5], b'1', "the cycle count");
        assert_eq!(meter_packet(0), meter_packet(180), "ten cycles wrap");
    }

    #[test]
    fn the_line_settings_shape_the_bytes() {
        let mut v = Violations::default();
        let mut dev = Cp2110::new(5, DEFAULT_SERIAL);
        set_feature(&mut dev, &[0x41, 1], &mut v);
        // Another baud rate: garbled bytes, flagged in the status.
        set_feature(&mut dev, &[0x50, 0, 0, 0x25, 0x80, 0, 0, 2, 0], &mut v);
        let got = bytes(&poll(&mut dev, 8, &mut v));
        assert_eq!(got.len(), 14);
        assert_ne!(got, meter_packet(0));
        assert!(got.iter().all(|b| *b < 0x80), "7 data bits");
        assert_eq!(dev.chip().errors, ERROR_PARITY);
        // 8 data bits: the odd parity bit becomes bit 7.
        set_feature(&mut dev, &[0x50, 0, 0, 0x4b, 0, 0, 0, 3, 0], &mut v);
        set_feature(&mut dev, &[0x41, 1], &mut v);
        let got = bytes(&poll(&mut dev, 8, &mut v));
        for (b, p) in got.iter().zip(meter_packet(1)) {
            assert_eq!(b & 0x7f, p);
            assert_eq!(b.count_ones() % 2, 1, "odd parity over 8 bits");
        }
        // 5 data bits: the low five.
        set_feature(&mut dev, &[0x50, 0, 0, 0x4b, 0, 0, 0, 0, 0], &mut v);
        set_feature(&mut dev, &[0x41, 1], &mut v);
        let got = bytes(&poll(&mut dev, 8, &mut v));
        assert!(got.iter().zip(meter_packet(2)).all(|(b, p)| *b == p & 0x1f));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn the_fifo_overruns() {
        let mut dev = Cp2110::new(5, DEFAULT_SERIAL);
        let mut v = Violations::default();
        open_like_libsigrok(&mut dev, &mut v);
        // 40 packets of 14 bytes, never read: 560 > 480.
        for _ in 0..40 * 5 + 3 {
            dev.chip.tick();
        }
        assert_eq!(dev.chip().fifo.len(), FIFO_BYTES);
        assert_eq!(dev.chip().errors & ERROR_OVERRUN, ERROR_OVERRUN);
    }

    #[test]
    fn flags_what_the_chip_does_not_take() {
        let mut dev = Cp2110::new(5, DEFAULT_SERIAL);
        let mut v = Violations::default();
        // Baud rate out of range; parity, flow, data bits, stop bits.
        set_feature(&mut dev, &[0x50, 0, 0, 0, 200, 0, 0, 3, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "baud 200");
        assert_eq!(dev.chip().config, Config::DEFAULT, "not applied");
        set_feature(&mut dev, &[0x50, 0, 0, 0x4b, 0, 5, 2, 4, 2], &mut v);
        assert_eq!(v.drain().len(), 4, "parity, flow, data bits, stop bits");
        // Wrong lengths, ids, values.
        assert_eq!(set_feature(&mut dev, &[0x41, 1, 0], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "enable of 3 bytes");
        set_feature(&mut dev, &[0x41, 2], &mut v);
        assert_eq!(v.drain().len(), 1, "enable 2");
        set_feature(&mut dev, &[0x43, 4], &mut v);
        assert_eq!(v.drain().len(), 1, "purge type 4");
        assert_eq!(set_feature(&mut dev, &[0x42, 0, 0, 0, 0, 0, 0], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "status is get only");
        assert_eq!(set_feature(&mut dev, &[0x45, 0, 0, 0, 0], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "GPIO not modelled");
        let s = Setup { request_type: 0x21, request: 9, value: 0x0341, index: 0, length: 2 };
        dev.control(s, &[0x50, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "payload id differs from wValue");
        assert_eq!(get_report(&mut dev, 3, 0x43, 2, &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "purge is set only");
        get_report(&mut dev, 3, 0x50, 64, &mut v);
        assert_eq!(v.drain().len(), 1, "config of 64 bytes");
        // Data before enable, to a meter that takes none; bad ids.
        let mut data = vec![3, b'a', b'b', b'c'];
        data.resize(64, 0);
        dev.out(1, &data, &mut v);
        assert_eq!(v.drain().len(), 2, "data before enable, and to the meter");
        dev.out(1, &[0x40, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "report id 0x40 on the interrupt pipe");
        dev.out(1, &[9, 1, 2], &mut v);
        assert_eq!(v.drain().len(), 3, "9 bytes announced, 2 sent");
        dev.out(1, &[0x43, 3, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "purge output report of 3 bytes");
        let output = Setup { request_type: 0x21, request: 9, value: 0x0243, index: 0, length: 2 };
        dev.control(output, &[0x43, 3], &mut v);
        assert_eq!(v.drain().len(), 1, "output report over the control pipe");
        // An interrupt IN too short for a data report.
        open_like_libsigrok(&mut dev, &mut v);
        for _ in 0..5 {
            dev.chip.tick();
        }
        assert_eq!(dev.in_transfer(1, 1, &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "a one byte transfer");
    }

    #[test]
    fn options_are_checked() {
        assert!(build(&[("period".into(), "4".into())]).is_err());
        assert!(build(&[("period".into(), "50".into())]).is_ok());
        assert!(build(&[("serial".into(), String::new())]).is_err());
        assert!(build(&[("colour".into(), "red".into())]).is_err());
        let m = build(&[("serial".into(), "XYZ".into())]).unwrap();
        assert_eq!(m.descriptors().strings[2], "XYZ");
        assert_eq!(report_descriptor().len(), 14 + 63 * 12 + 5 * 8 + 1);
    }
}
