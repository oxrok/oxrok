// Counterpart in sigrok:
//   libsigrok/src/serial_hid_cp2110.c (the reports the chip is sent),
//   libsigrok/src/serial_hid.c (hidapi calls behind them),
//   libsigrok/src/dmm/es519xx.c (the UT61E packet layout),
//   Copyright (C) 2017 Carl-Fredrik Sundström <audio.cf@gmail.com>,
//   Copyright (C) 2017-2019 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2013 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Daniel Elstner, Bert Vermeulen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! The Silicon Labs CP2110 HID-to-UART bridge (`10c4:ea80`) with a
//! UNI-T UT61E (Cyrustek ES51922, 14 byte packets at 19200 baud, 7o1)
//! on its UART, as libsigrok's `serial-dmm` driver reads it through
//! `conn=hid/cp2110`.
//!
//! The chip, after Silicon Labs' AN434 (CP2110/4 Interface
//! Specification):
//!
//! - UART data travels on the interrupt pipes in reports 0x01 to 0x3f,
//!   the report id being the number of data bytes behind it (1 to 63).
//!   The device here sends exactly that many; libsigrok reads into a 64
//!   byte buffer and takes the count from byte 0.
//! - Configuration is feature reports on the control pipe, the report
//!   id in byte 0 of the payload: 0x41 Get/Set UART Enable (1 byte,
//!   0 or 1), 0x42 Get UART Status (TX FIFO and RX FIFO fill, two bytes
//!   each, error status, break status), 0x43 Set Purge FIFOs (1 byte:
//!   1 TX, 2 RX, 3 both), 0x46 Get Version Information (part number
//!   0x0a, device version), 0x50 Get/Set UART Config (baud rate as a
//!   32 bit big endian number, parity, flow control 0 or 1, data bits
//!   0 to 3 for 5 to 8, stop bits 0 short or 1 long).
//! - A Get Report the chip does not know stalls (AN434 2.1.2).
//!
//! The pedantic checks: feature report ids, lengths (report id
//! included) and values as above, the report id in byte 0 of a
//! `SET_REPORT` agreeing with `wValue`, a baud rate within 300 to
//! 1000000 (libsigrok's `CP2110_BAUDRATE_MAX`; AN434 rev 0.4 says
//! 500000, later parts and revisions 1000000), data reports only once
//! the UART is enabled, and with a byte count between 1 and 63 that
//! fits the transfer. Reports the model does not implement (GPIO, lock
//! bytes, line break, the PROM customisation) stall and are flagged:
//! neither tool has a reason to send them.
//!
//! Two habits of the C code that the chip's documentation does not
//! back, documented here rather than flagged so that a sigrok-cli run
//! is clean:
//!
//! - `cp2110_flush` purges with `hid_write`, an output report 0x43 on
//!   the interrupt pipe, where AN434 has Set Purge FIFOs as a feature
//!   report on the control pipe (as pyserial's CP2110 backend sends
//!   it). What a real chip makes of a report id above 0x3f on its
//!   interrupt OUT pipe is not documented; the model takes it as the
//!   purge it is meant to be, and takes the feature report as well.
//! - libsigrok's parity codes are 1 for even and 2 for odd, as in
//!   AN434 rev 0.4. Revision 0.5 "corrected the values for Data Bits
//!   and Parity" to 1 odd, 2 even, which is what pyserial, pycp2110 and
//!   the `cp211x_uart` crate send; libsigrok has the corrected data bits
//!   (0 to 3) but the old parity. A UT61E's 7o1 frames under a chip
//!   set to code 2 are then checked for even parity: the bytes still
//!   arrive, the chip only raises the parity error flag in its status.
//!   The model does that (`Get UART Status` error bit 0x01), so the
//!   bytes the host sees do not depend on which reading is right, and
//!   no host reads the status. Any code 0 to 4 is legal.
//!
//! The meter's stream is cut to what the host can see deterministically.
//! A real UT61E sends a packet about every half second whatever the
//! host does; the kernel polls the interrupt IN endpoint as long as the
//! `hidraw` node is open, so a model that answered every poll would log
//! as many transfers as the host had time for. Instead the RX FIFO
//! fills once per purge of it (a purge is the last thing libsigrok's
//! `serial_open` does): with the UART enabled and set to the meter's
//! 19200 baud, the next `packets` packets of the meter (default 4)
//! become receivable `delay_ms` (default 200) after the purge, in three
//! reports of 5, 8 and the rest of the bytes, so that the first packet
//! only completes in the last report and every report has been fetched
//! before a host that stops at its first or its Nth packet closes the
//! node. Then the FIFO stays empty until the next purge. The delay is
//! the meter's (a UT61E sends twice a second) and keeps a host that
//! opens the cable without reading (`--show` opens the device after
//! its scan) from racing the kernel's polling: it closes before the
//! burst is due, so no report goes. A configuration at another baud rate receives
//! nothing (framing errors); 8 data bits deliver the parity bit as bit
//! 7, 5 or 6 data bits cut the bytes short, as a UART does.
//!
//! What the model showed about the hosts:
//!
//! - libsigrok finds a CP2110 only by its node,
//!   `conn=hid/cp2110/raw=/dev/hidrawN`. Its search (`conn=hid`,
//!   `hid/cp2110`, `hid/cp2110/sn=...`) puts the full port name
//!   (`hid/cp2110/raw=/dev/hidrawN`) where the hidapi path belongs and
//!   asks hidapi-hidraw to open that, which fails ("Failed to open HID
//!   device"); the scenario names the node.
//! - libsigrok's nonblocking HID read keeps taking reports while they
//!   come within 10 ms, so the three reports of a burst are one read,
//!   and serial-dmm checks the sample limit only after every packet of
//!   a read is out: a burst of four packets gives four samples whatever
//!   `--samples` (up to four) asks. oxrok now does both.
//! - Every open is enable, config, purge: serial-dmm closes the cable
//!   after its scan and opens it again to acquire, so a scan and an
//!   acquisition see two bursts (packets 0 to 3, then 4 to 7).
//!
//! The packets cycle through eight readings (DC volts, negative DC
//! volts with hold, AC volts, ohms, DC milliamps, capacitance,
//! frequency, an overload), the last digit counting the cycles, and
//! start over on `reset`, so both tools see the same numbers.
//!
//! The device is full speed with a 64 byte endpoint 0, interrupt IN
//! 0x81 and interrupt OUT 0x01 of 64 bytes polled every millisecond,
//! bus powered at 100 mA, with Silicon Labs' strings.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::hid::{self, HidInterface, HidReports};
use crate::{Error, Result};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

/// The model name.
pub const NAME: &str = "cp2110";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Silicon Labs CP2110 HID-to-UART bridge (10c4:ea80) with a UNI-T UT61E stream; \
     packets=N per purge (1 to 4, default 4), delay_ms=N after it (default 200), \
     serial=TEXT (default 0001A5F8)";

/// A cable from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut packets = 4;
    let mut delay_ms = 200;
    let mut serial = DEFAULT_SERIAL.to_owned();
    for (k, v) in options {
        match k.as_str() {
            "packets" => {
                packets = v
                    .parse()
                    .ok()
                    .filter(|n| (1..=4).contains(n))
                    .ok_or_else(|| Error(format!("packets={v}: 1 to 4")))?;
            }
            "delay_ms" => {
                delay_ms = v
                    .parse()
                    .ok()
                    .filter(|n| *n <= 10_000)
                    .ok_or_else(|| Error(format!("delay_ms={v}: 0 to 10000")))?;
            }
            "serial" => {
                if v.is_empty() || v.len() > 30 {
                    return Err(Error(format!("serial={v}: 1 to 30 characters")));
                }
                serial.clone_from(v);
            }
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    let mut cable = Cp2110::new(packets, &serial);
    cable.chip.delay = Duration::from_millis(delay_ms);
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

/// Purge type bits.
const PURGE_TX: u8 = 0x01;
const PURGE_RX: u8 = 0x02;

/// Parity codes as AN434 rev 0.5 has them.
const PARITY_ODD: u8 = 1;
/// Error status: a parity error since the last read.
const ERROR_PARITY: u8 = 0x01;

/// The meter's line: 19200 baud, 7 data bits, odd parity, 1 stop bit.
pub const METER_BAUD: u32 = 19200;
/// The meter's packet length.
pub const PACKET_BYTES: usize = 14;
/// How the bytes of a burst are cut into reports, the rest going into
/// the last one.
const SPLIT: [usize; 2] = [5, 8];

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
/// digit counting the cycles.
pub fn meter_packet(n: u64) -> [u8; PACKET_BYTES] {
    let cycle = b'0' + (n / 8 % 10) as u8;
    let d = |s: &[u8; 4]| [s[0], s[1], s[2], s[3], cycle];
    match n % 8 {
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
        _ => packet(6, d(b"0000"), 0x33, OL, [0, 0, AUTO, 0]),
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
    /// Input reports ready for the interrupt IN pipe, each with its
    /// count byte.
    pub rx: VecDeque<Vec<u8>>,
    /// Packets per burst.
    pub packets: usize,
    /// The meter's next packet.
    pub next_packet: u64,
    /// Bursts delivered.
    pub bursts: u32,
    /// How long after a purge the burst is receivable.
    pub delay: Duration,
    /// When the reports in `rx` are receivable.
    pub ready_at: Option<Instant>,
}

impl Chip {
    fn new(packets: usize) -> Chip {
        Chip {
            enabled: false,
            config: Config::DEFAULT,
            errors: 0,
            rx: VecDeque::new(),
            packets,
            next_packet: 0,
            bursts: 0,
            delay: Duration::from_millis(200),
            ready_at: None,
        }
    }

    /// The bytes in the RX FIFO.
    fn rx_fill(&self) -> usize {
        self.rx.iter().map(|r| r.len() - 1).sum()
    }

    /// Set Purge FIFOs, by feature report or (libsigrok's way) by output
    /// report. Purging RX with the UART enabled at the meter's baud rate
    /// lets the next burst in.
    fn purge(&mut self, kind: u8, v: &mut Violations) {
        if kind & !(PURGE_TX | PURGE_RX) != 0 || kind == 0 {
            v.report(format!("purge type {kind:#04x}; AN434 has 1 (TX), 2 (RX) or 3 (both)"));
            return;
        }
        if kind & PURGE_RX == 0 {
            return;
        }
        self.rx.clear();
        self.ready_at = None;
        if !self.enabled || self.config.baud != METER_BAUD {
            return;
        }
        let mut bytes = Vec::with_capacity(self.packets * PACKET_BYTES);
        for _ in 0..self.packets {
            bytes.extend_from_slice(&meter_packet(self.next_packet));
            self.next_packet += 1;
        }
        self.receive(&bytes);
        self.ready_at = Some(Instant::now() + self.delay);
        self.bursts += 1;
    }

    /// The meter's 7o1 bytes as this UART configuration receives them,
    /// cut into the burst's reports.
    fn receive(&mut self, bytes: &[u8]) {
        let bits = self.config.data_bits + 5;
        let framed: Vec<u8> = bytes
            .iter()
            .map(|&b| match bits {
                // The parity bit, odd, lands in bit 7.
                8 => b | (u8::from(b.count_ones() % 2 == 0) << 7),
                7 => b,
                n => b & ((1u8 << n) - 1),
            })
            .collect();
        // Parity is checked on the bit after the data bits: with 7 data
        // bits that is the meter's odd parity bit; with 8 it is the stop
        // bit, which is 1.
        let parity_ok = match (bits, self.config.parity) {
            (7, p) => p == 0 || p == PARITY_ODD,
            (8, 0) => true,
            _ => false,
        };
        if !parity_ok {
            self.errors |= ERROR_PARITY;
        }
        let mut rest = framed.as_slice();
        for cut in SPLIT {
            if rest.len() <= cut {
                break;
            }
            let (head, tail) = rest.split_at(cut);
            self.rx.push_back(data_report(head));
            rest = tail;
        }
        if !rest.is_empty() {
            self.rx.push_back(data_report(rest));
        }
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

/// A UART data report: count byte and data.
fn data_report(bytes: &[u8]) -> Vec<u8> {
    let mut r = Vec::with_capacity(bytes.len() + 1);
    r.push(bytes.len() as u8);
    r.extend_from_slice(bytes);
    r
}

impl HidReports for Chip {
    fn get_report(&mut self, kind: u8, id: u8, length: usize, v: &mut Violations) -> Reply {
        if kind != hid::FEATURE {
            v.report(format!(
                "GET_REPORT of kind {kind} id {id:#04x}; UART data comes on the interrupt pipe"
            ));
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
                r.extend_from_slice(&(self.rx_fill() as u16).to_be_bytes());
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
                0 => self.enabled = false,
                1 => self.enabled = true,
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
    /// A cable sending `packets` meter packets per purge, with this USB
    /// serial number.
    pub fn new(packets: usize, serial: &str) -> Cp2110 {
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
            chip: Chip::new(packets),
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
        if self.chip.ready_at.is_some_and(|t| Instant::now() < t) {
            return Reply::Nak;
        }
        match self.chip.rx.front() {
            Some(r) if r.len() > max => {
                v.report(format!("interrupt IN of {max} bytes, the report has {}", r.len()));
                Reply::Stall
            }
            Some(_) => Reply::Data(self.chip.rx.pop_front().expect("report")),
            None => Reply::Nak,
        }
    }

    fn reset(&mut self) {
        // A bus reset restarts the chip with its power-on settings.
        let (packets, delay) = (self.chip.packets, self.chip.delay);
        self.chip = Chip::new(packets);
        self.chip.delay = delay;
        self.hid.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A cable whose bursts are receivable at once.
    fn cable(packets: usize) -> Cp2110 {
        let mut dev = Cp2110::new(packets, DEFAULT_SERIAL);
        dev.chip.delay = Duration::ZERO;
        dev
    }

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

    fn get_feature(dev: &mut Cp2110, id: u8, length: u16, v: &mut Violations) -> Reply {
        let s = Setup {
            request_type: 0xa1,
            request: 1,
            value: 0x0300 | u16::from(id),
            index: 0,
            length,
        };
        dev.control(s, &[], v)
    }

    /// What `cp2110_set_params` and `cp2110_flush` send for 19200/7o1.
    fn open_like_libsigrok(dev: &mut Cp2110, v: &mut Violations) {
        assert_eq!(set_feature(dev, &[0x41, 1], v), Reply::ack());
        assert_eq!(set_feature(dev, &[0x50, 0, 0, 0x4b, 0, 2, 0, 2, 0], v), Reply::ack());
        let mut purge = vec![0x43, 0x03];
        purge.resize(2, 0);
        assert_eq!(dev.out(2, &purge, v), Reply::ack());
    }

    fn collect(dev: &mut Cp2110, v: &mut Violations) -> Vec<Vec<u8>> {
        let mut reports = Vec::new();
        while let Reply::Data(r) = dev.in_transfer(1, 64, v) {
            reports.push(r);
        }
        reports
    }

    #[test]
    fn streams_a_burst_per_purge_as_libsigrok_opens_it() {
        let mut dev = cable(4);
        let mut v = Violations::default();
        // Nothing before the UART is up.
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak);
        open_like_libsigrok(&mut dev, &mut v);
        let reports = collect(&mut dev, &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(reports.iter().map(Vec::len).collect::<Vec<_>>(), [6, 9, 44]);
        let bytes: Vec<u8> = reports.iter().flat_map(|r| r[1..].to_vec()).collect();
        for (i, r) in reports.iter().enumerate() {
            assert_eq!(usize::from(r[0]), r.len() - 1, "report {i} id is its count");
        }
        assert_eq!(&bytes[..14], &meter_packet(0));
        assert_eq!(&bytes[42..], &meter_packet(3));
        // libsigrok's parity code 2 is even under AN434 rev 0.5.
        assert_eq!(dev.chip().errors, ERROR_PARITY);
        // Read the status: RX FIFO empty, the parity error, cleared after.
        let s = get_feature(&mut dev, 0x42, 7, &mut v);
        assert_eq!(s, Reply::Data(vec![0x42, 0, 0, 0, 0, 1, 0]));
        assert_eq!(dev.chip().errors, 0);
        // The FIFO stays empty until the next open.
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak);
        open_like_libsigrok(&mut dev, &mut v);
        let bytes: Vec<u8> = collect(&mut dev, &mut v).iter().flat_map(|r| r[1..].to_vec()).collect();
        assert_eq!(&bytes[..14], &meter_packet(4));
        assert_eq!(get_feature(&mut dev, 0x46, 3, &mut v), Reply::Data(vec![0x46, 0x0a, 0x01]));
        assert_eq!(
            get_feature(&mut dev, 0x50, 9, &mut v),
            Reply::Data(vec![0x50, 0, 0, 0x4b, 0, 2, 0, 2, 0])
        );
        assert_eq!(get_feature(&mut dev, 0x41, 2, &mut v), Reply::Data(vec![0x41, 1]));
        assert!(v.is_empty(), "{v:?}");
        // The feature report purge of AN434 does the same.
        assert_eq!(set_feature(&mut dev, &[0x43, 0x02], &mut v), Reply::ack());
        assert_eq!(collect(&mut dev, &mut v).len(), 3);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn the_burst_waits_for_the_delay() {
        let mut dev = Cp2110::new(1, DEFAULT_SERIAL);
        dev.chip.delay = Duration::from_millis(50);
        let mut v = Violations::default();
        open_like_libsigrok(&mut dev, &mut v);
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak, "not yet");
        std::thread::sleep(Duration::from_millis(60));
        assert_eq!(collect(&mut dev, &mut v).len(), 3, "14 bytes: 5, 8 and 1");
        // A reset keeps the option.
        dev.reset();
        assert_eq!(dev.chip.delay, Duration::from_millis(50));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn readings_are_valid_ut61e_packets() {
        for n in 0..16 {
            let p = meter_packet(n);
            assert_eq!(&p[12..], b"\r\n");
            assert!(p[..12].iter().all(|b| (0x30..=0x3f).contains(b)), "{p:?}");
            assert!(p[1..6].iter().all(u8::is_ascii_digit));
        }
        assert_eq!(meter_packet(8)[5], b'1', "the cycle count");
        assert_eq!(meter_packet(0), meter_packet(80), "ten cycles wrap");
    }

    #[test]
    fn the_line_settings_shape_the_bytes() {
        let mut v = Violations::default();
        // Another baud rate: framing errors, nothing received.
        let mut dev = cable(1);
        set_feature(&mut dev, &[0x41, 1], &mut v);
        set_feature(&mut dev, &[0x50, 0, 0, 0x25, 0x80, 0, 0, 3, 0], &mut v);
        set_feature(&mut dev, &[0x43, 3], &mut v);
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak);
        // 8 data bits: the odd parity bit becomes bit 7.
        set_feature(&mut dev, &[0x50, 0, 0, 0x4b, 0, 0, 0, 3, 0], &mut v);
        set_feature(&mut dev, &[0x43, 3], &mut v);
        let bytes: Vec<u8> = collect(&mut dev, &mut v).iter().flat_map(|r| r[1..].to_vec()).collect();
        let plain = meter_packet(0);
        for (b, p) in bytes.iter().zip(plain) {
            assert_eq!(b & 0x7f, p);
            assert_eq!(b.count_ones() % 2, 1, "odd parity over 8 bits");
        }
        assert_eq!(dev.chip().errors, 0);
        // A purge with the UART disabled lets nothing in.
        set_feature(&mut dev, &[0x41, 0], &mut v);
        set_feature(&mut dev, &[0x43, 3], &mut v);
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_what_the_chip_does_not_take() {
        let mut dev = cable(4);
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
        assert_eq!(get_feature(&mut dev, 0x43, 2, &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "purge is set only");
        get_feature(&mut dev, 0x50, 64, &mut v);
        assert_eq!(v.drain().len(), 1, "config of 64 bytes");
        let input = Setup { request_type: 0xa1, request: 1, value: 0x0105, index: 0, length: 6 };
        assert_eq!(dev.control(input, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "input report by GET_REPORT");
        // Data before enable, to a meter that takes none; bad ids.
        let mut data = vec![3, b'a', b'b', b'c'];
        data.resize(64, 0);
        dev.out(2, &data, &mut v);
        assert_eq!(v.drain().len(), 2, "data before enable, and to the meter");
        dev.out(2, &[0x40, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "report id 0x40 on the interrupt pipe");
        dev.out(2, &[9, 1, 2], &mut v);
        assert_eq!(v.drain().len(), 3, "9 bytes announced, 2 sent");
        dev.out(2, &[0x43, 3, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "purge output report of 3 bytes");
        let output = Setup { request_type: 0x21, request: 9, value: 0x0243, index: 0, length: 2 };
        dev.control(output, &[0x43, 3], &mut v);
        assert_eq!(v.drain().len(), 1, "output report over the control pipe");
    }

    #[test]
    fn options_are_checked() {
        assert!(build(&[("packets".into(), "5".into())]).is_err());
        assert!(build(&[("packets".into(), "2".into())]).is_ok());
        assert!(build(&[("serial".into(), String::new())]).is_err());
        assert!(build(&[("colour".into(), "red".into())]).is_err());
        assert!(build(&[("delay_ms".into(), "10001".into())]).is_err());
        assert!(build(&[("delay_ms".into(), "0".into())]).is_ok());
        let m = build(&[("serial".into(), "XYZ".into())]).unwrap();
        assert_eq!(m.descriptors().strings[2], "XYZ");
        assert_eq!(report_descriptor().len(), 14 + 63 * 12 + 5 * 8 + 1);
    }
}
