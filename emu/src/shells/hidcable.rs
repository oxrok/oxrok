// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! What the serial over HID cable models (`bu86x`, `ch9325`, `victor`)
//! share: the device the Pico emulator presents (`hil/pico-hidcable`),
//! the meter packets they play, and the poll counted stream of the two
//! cables that send without being asked.

use crate::options::parse_hex_list;
pub use crate::options::parse_number;
use std::collections::VecDeque;

use crate::Result;
use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, Reply, Speed, Violations};
use crate::shells::hid::HidInterface;

/// The serial number the emulator's cables report.
pub const SERIAL: &str = "HIDCABLE1";

/// The interrupt IN endpoint, number 1.
pub const EP_IN: u8 = 1;

/// The Pico emulator's vendor page report descriptor
/// (`CABLE_REPORT(IN, OUT)`): input and output reports of `input` and
/// `output` bytes, a feature report of 8, no report ids.
pub fn report_descriptor(input: u8, output: u8) -> Vec<u8> {
    vec![
        0x06, 0x00, 0xff, // usage page, vendor defined
        0x09, 0x01, // usage 1
        0xa1, 0x01, // collection, application
        0x09, 0x02, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, input, 0x81,
        0x02, // input
        0x09, 0x03, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, output, 0x91,
        0x02, // output
        0x09, 0x04, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x08, 0xb1,
        0x02, // feature, 8 bytes
        0xc0, // end collection
    ]
}

/// The device as the Pico's `TinyUSB` stack presents it: full speed,
/// a 64 byte endpoint 0, one HID interface with a 64 byte interrupt IN
/// endpoint polled every 2 ms and no OUT endpoint (output reports
/// arrive as `SET_REPORT`), serial number [`SERIAL`]. The Pico also
/// has a CDC interface for its command port, which no driver uses and
/// the model leaves out.
pub fn descriptors(
    vendor: u16,
    product: u16,
    manufacturer: &str,
    name: &str,
    hid: &HidInterface,
) -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 1, [3, 0, 0], 0));
    body.extend_from_slice(&hid.descriptor());
    body.extend_from_slice(&descriptor::endpoint(0x80 | EP_IN, 3, 64, 2));
    let config = descriptor::configuration(1, 0, 0xa0, 100, &body);
    let device = DeviceDescriptor {
        usb: 0x0200,
        class: [0, 0, 0],
        max_packet0: 64,
        vendor,
        product,
        device: 0x0100,
        strings: [1, 2, 3],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::Full,
        device: device.bytes(),
        configs: vec![config],
        strings: vec![manufacturer.to_owned(), name.to_owned(), SERIAL.to_owned()],
    }
}

/// Parses `packet=HEX[,HEX...]`: see [`parse_hex_list`].
///
/// # Errors
///
/// On a malformed hex string or a packet length out of range.
pub fn parse_packets(value: &str, min: usize, max: usize) -> Result<Vec<Vec<u8>>> {
    parse_hex_list("packet", value, min, max)
}

/// Parses `burst=N`, packets per start (0 to 1000).
///
/// # Errors
///
/// When it is not a number in the range.
pub fn parse_burst(value: &str) -> Result<Option<u32>> {
    parse_number("burst", value, 0..=1000).map(Some)
}

/// The meter's packets, played in turn and from the first again after
/// the last; a fresh model starts at the first.
#[derive(Clone, Debug)]
pub struct Packets {
    list: Vec<Vec<u8>>,
    next: usize,
}

impl Packets {
    /// The packets, at least one.
    pub fn new(list: Vec<Vec<u8>>) -> Packets {
        assert!(!list.is_empty(), "a meter plays at least one packet");
        Packets { list, next: 0 }
    }

    /// The next packet.
    pub fn take(&mut self) -> Vec<u8> {
        let p = self.list[self.next].clone();
        self.next = (self.next + 1) % self.list.len();
        p
    }
}

/// The input reports a cable has to send, in order.
#[derive(Clone, Debug, Default)]
pub struct Reports(VecDeque<Vec<u8>>);

impl Reports {
    /// Queues reports.
    pub fn push(&mut self, reports: impl IntoIterator<Item = Vec<u8>>) {
        self.0.extend(reports);
    }

    /// Reports still queued.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// None queued.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// The answer to an interrupt IN transfer of up to `max` bytes: the
    /// next report, or a NAK while there is none. A transfer shorter
    /// than the report would cut it, which hidraw never asks for.
    pub fn answer(&mut self, max: usize, v: &mut Violations) -> Reply {
        let Some(r) = self.0.pop_front() else {
            return Reply::Nak;
        };
        if max < r.len() {
            v.report(format!("interrupt IN of {max} bytes; the report has {}", r.len()));
        }
        Reply::Data(r)
    }
}

/// A cable that sends without being asked, paced by the host's polls
/// rather than by wall time, so that both tools of a case see the same
/// number of packets.
///
/// The Pico sends a packet every `PERIOD` ms. Here a packet is due
/// after `period` offers of the interrupt IN transfer the kernel keeps
/// pending while the hidraw node is open: the server offers a held
/// transfer when it arrives and then once per 20 ms tick, so a period
/// of N is about N times 20 ms, but counted, and the count stands
/// still while the node is closed (no transfer is pending then). A
/// tool that closes the node within a period of the last packet it
/// needed has thus caused exactly the packets it read, and an open
/// does not see a packet within the 50 ms after it, in which the
/// kernel's usbhid drops what arrives (`HID_RESUME_RUNNING`), unless
/// the node was opened and closed again several times within one
/// period.
///
/// A burst limit makes the count independent of how long the host
/// listens: at most that many packets per start (an open, for the
/// cables that learn of one), then silence until the next. A scan that
/// finds nothing listens for 3 s of wall time, in which an unlimited
/// stream sends a number of packets that varies from run to run; a
/// limited one sends its burst well inside them, and nothing after.
/// A burst of 0 is a meter that never sends.
#[derive(Clone, Debug)]
pub struct Stream {
    period: u32,
    polls: u32,
    running: bool,
    burst: Option<u32>,
    sent: u32,
}

impl Stream {
    /// A stream of a packet per `period` polls; it runs from the start
    /// when `running`.
    pub fn new(period: u32, running: bool) -> Stream {
        Stream { period, polls: 0, running, burst: None, sent: 0 }
    }

    /// At most `burst` packets per start; `None` (the default) sends
    /// for as long as the host polls.
    #[must_use]
    pub fn with_burst(mut self, burst: Option<u32>) -> Stream {
        self.burst = burst;
        self
    }

    /// Starts it, or starts the period (and the burst) over.
    pub fn restart(&mut self) {
        self.running = true;
        self.polls = 0;
        self.sent = 0;
    }

    /// One offer of the interrupt IN transfer with no report queued:
    /// true when a packet is due.
    pub fn poll(&mut self) -> bool {
        if !self.running || self.burst.is_some_and(|b| self.sent >= b) {
            return false;
        }
        self.polls += 1;
        if self.polls >= self.period {
            self.polls = 0;
            self.sent += 1;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_packets() {
        assert_eq!(parse_packets("0a0B", 1, 4).unwrap(), vec![vec![0x0a, 0x0b]]);
        assert_eq!(parse_packets("01,0203", 1, 4).unwrap(), vec![vec![1], vec![2, 3]]);
        assert!(parse_packets("012", 1, 4).is_err());
        assert!(parse_packets("zz", 1, 4).is_err());
        assert!(parse_packets("", 1, 4).is_err());
        assert!(parse_packets("0102030405", 1, 4).is_err());
        assert!(parse_number("period", "0", 1..=9).is_err());
        assert_eq!(parse_number("period", "9", 1..=9).unwrap(), 9);
    }

    #[test]
    fn packets_cycle_and_the_stream_counts_polls() {
        let mut p = Packets::new(vec![vec![1], vec![2]]);
        assert_eq!([p.take(), p.take(), p.take()], [vec![1], vec![2], vec![1]]);
        let mut s = Stream::new(3, false);
        assert!(!s.poll() && !s.poll() && !s.poll() && !s.poll());
        s.restart();
        assert_eq!([s.poll(), s.poll(), s.poll(), s.poll()], [false, false, true, false]);
        let mut burst = Stream::new(1, true).with_burst(Some(2));
        assert_eq!(
            [burst.poll(), burst.poll(), burst.poll()],
            [true, true, false],
            "two, then silence"
        );
        burst.restart();
        assert_eq!(
            [burst.poll(), burst.poll(), burst.poll()],
            [true, true, false],
            "two per start"
        );
        let mut silent = Stream::new(1, true).with_burst(Some(0));
        assert!(!silent.poll() && !silent.poll());
        let mut r = Reports::default();
        let mut v = Violations::default();
        assert_eq!(r.answer(64, &mut v), Reply::Nak);
        r.push([vec![1, 2], vec![3]]);
        assert_eq!(r.len(), 2);
        assert_eq!(r.answer(1, &mut v), Reply::Data(vec![1, 2]));
        assert_eq!(v.drain().len(), 1, "a transfer shorter than the report");
        assert_eq!(r.answer(64, &mut v), Reply::Data(vec![3]));
        assert!(r.is_empty() && v.is_empty());
    }
}
