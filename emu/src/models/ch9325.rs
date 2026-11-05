// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The WCH CH9325 UART to HID bridge of the UNI-T UT-D04 cable and
//! others, as the Pico emulator plays it (`hil/pico-hidcable`,
//! `MODE ch9325`): a HID device `1a86:e008`, vendor `WCH`, with 8 byte
//! input and output reports and no report ids, behind which a meter
//! sends its packets unasked, here a UNI-T UT61E (`ES519xx`, 14 bytes at
//! 19200/7o1).
//!
//! An input report carries up to 7 UART bytes behind a count byte
//! `0xf0 | count`, so a 14 byte packet is two reports. The host sets
//! the line parameters with a feature report, which `ch9325_set_params`
//! lays out as report id 0 (stripped by hidraw) and then
//!
//! | byte | meaning |
//! |---|---|
//! | 0, 1 | bit rate, little endian |
//! | 2, 3 | unknown, 0 |
//! | 4 | data bits minus 5 (0 to 3) |
//!
//! five bytes, sent as `SET_REPORT` of feature report 0. The C driver
//! does so on every open (`serial_open` applies the `serialcomm`), so
//! the model starts the stream with the first one and starts its period
//! over with each. A packet is due every `period` polls of the
//! interrupt IN endpoint, not every `PERIOD` ms as on the Pico (see
//! `shells::hidcable::Stream`), so both tools see the same packets.
//!
//! The count is what makes the comparison deterministic. Both tools
//! open the node for the scan, read the first packet, close, and open
//! it again for the device (sigrok's scan closes its port; oxrok's
//! serial-dmm does the same for a HID cable). Each open's feature
//! report starts the period over, so a tool sees no packet beyond the
//! ones it reads unless it keeps the node open for a whole period
//! after the line setting or after its last packet. sigrok's open
//! holds the node about 100 ms before it sets the line (it opens the
//! path once to check it, then again), in which the held transfer is
//! offered some 10 times; the default period of 50 polls, about a
//! second, leaves that window and the close after the last sample far
//! behind. A period of 10 let sigrok see an extra packet there.
//!
//! Options: `packet=HEX[,HEX...]`, the meter's packets in turn (1 to
//! 64 bytes each; default the `diffcli.sh --hw` UT61E packet);
//! `period=N`, polls per packet (2 to 1000, default 50, about a second);
//! `baud=N` and `bits=N`, the meter's line (default 19200 and 7), which
//! the host's feature report must set; `burst=N`, at most N packets
//! after each line setting (each open), then silence until the next
//! (default: no limit). `burst=0` is a meter that never sends: the scan
//! gives up after its 3 s, and, the count being per open, a scan that
//! finds nothing in a burst sends the same reports whatever the time.
//!
//! The pedantic checks: the feature report is the five bytes above
//! with the meter's rate and data bits; an output report (the UART
//! transmit direction, which `ch9325_write_bytes` frames as 8 bytes
//! with the count, 1 to 7, in the report id position) has that form;
//! the host does not read the feature report back nor read an input
//! report by `GET_REPORT`.

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Violations};
use crate::shells::hidcable as cable;
use crate::shells::hid::{self, HidInterface, HidReports};
use crate::{Error, Result};
use crate::shells::hidcable::{Packets, Reports, Stream};

/// The model name.
pub const NAME: &str = "ch9325";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "WCH CH9325 cable of a UNI-T UT61E (HID, 1a86:e008), the Pico emulator's \
                         role; packet=HEX[,HEX...], period=N (polls per packet, default 50), \
                         baud=N (19200), bits=N (7), burst=N (packets per open)";

/// The USB vendor id.
pub const VENDOR_ID: u16 = 0x1a86;
/// The USB product id.
pub const PRODUCT_ID: u16 = 0xe008;
/// The report length, both ways.
pub const REPORT_BYTES: usize = 8;
/// UART bytes per report.
pub const CHUNK: usize = 7;
/// The line parameter feature report, report id stripped.
pub const FEATURE_BYTES: usize = 5;
/// A UT61E packet as the `--hw` case plays it.
pub const DEFAULT_PACKET: &str = "3131323334353b3030303a300d0a";

/// A cable from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut packets = cable::parse_packets(DEFAULT_PACKET, 1, 64)?;
    let mut period = 50;
    let mut baud = 19200;
    let mut bits = 7;
    let mut burst = None;
    for (k, v) in options {
        match k.as_str() {
            "packet" => packets = cable::parse_packets(v, 1, 64)?,
            "period" => period = cable::parse_number(k, v, 2..=1000)?,
            "baud" => baud = cable::parse_number(k, v, 1..=0xffff)?,
            "bits" => bits = cable::parse_number(k, v, 5..=8)?,
            "burst" => burst = cable::parse_burst(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let mut cable = Ch9325::new(packets, period, baud as u16, bits as u8);
    cable.bridge.stream = Stream::new(period, false).with_burst(burst);
    Ok(Box::new(cable))
}

/// The chip and the meter behind it.
#[derive(Clone, Debug)]
pub struct Bridge {
    packets: Packets,
    reports: Reports,
    stream: Stream,
    baud: u16,
    bits: u8,
    feature: Vec<u8>,
}

impl Bridge {
    /// The reports of the next packet, 7 bytes each behind the count.
    fn send_packet(&mut self) {
        let p = self.packets.take();
        self.reports.push(p.chunks(CHUNK).map(|c| {
            let mut r = vec![0xf0 | c.len() as u8];
            r.extend_from_slice(c);
            r.resize(REPORT_BYTES, 0);
            r
        }));
    }

    fn line(&mut self, data: &[u8], v: &mut Violations) {
        if data.len() != FEATURE_BYTES {
            v.report(format!(
                "line parameter report of {} bytes ({}); ch9325_set_params sends {FEATURE_BYTES}",
                data.len(),
                crate::txlog::hex(data)
            ));
        }
        if data.len() < FEATURE_BYTES {
            return;
        }
        let baud = u16::from_le_bytes([data[0], data[1]]);
        if data[2] != 0 || data[3] != 0 {
            v.report(format!("line parameter bytes 2 and 3 are {:02x} {:02x}, not 0", data[2], data[3]));
        }
        if data[4] > 3 {
            v.report(format!("data bits code {}; 0 to 3 are 5 to 8 bits", data[4]));
        }
        if baud != self.baud || data[4] + 5 != self.bits {
            v.report(format!(
                "line set to {baud} baud, {} bits; the meter sends at {} baud, {} bits",
                data[4] + 5,
                self.baud,
                self.bits
            ));
        }
        data.clone_into(&mut self.feature);
        self.stream.restart();
    }
}

impl HidReports for Bridge {
    fn get_report(&mut self, kind: u8, id: u8, length: usize, v: &mut Violations) -> Reply {
        if kind != hid::FEATURE {
            v.report(format!("GET_REPORT of kind {kind}; the chip's input comes on endpoint 1"));
            return Reply::Stall;
        }
        v.report(format!("GET_REPORT of feature report {id}; the driver never reads it back"));
        Reply::Data(self.feature[..length.min(self.feature.len())].to_vec())
    }

    fn set_report(&mut self, kind: u8, id: u8, data: &[u8], v: &mut Violations) -> Reply {
        match kind {
            hid::FEATURE => {
                if id != 0 {
                    v.report(format!("feature report id {id}; the line parameters are report 0"));
                }
                self.line(data, v);
                Reply::ack()
            }
            hid::OUTPUT => {
                // UART transmit: the count in the report id position,
                // then the count again and the bytes, 8 in all. The
                // meter does not listen.
                let count = usize::from(id);
                if !(1..=CHUNK).contains(&count)
                    || data.len() != REPORT_BYTES
                    || usize::from(data[0]) != count
                {
                    v.report(format!(
                        "output report id {id} with {}; ch9325_write_bytes sends the count \
                         (1 to 7) and 7 bytes",
                        crate::txlog::hex(data)
                    ));
                }
                Reply::ack()
            }
            other => {
                v.report(format!("SET_REPORT of kind {other}"));
                Reply::Stall
            }
        }
    }
}

/// The emulated cable.
pub struct Ch9325 {
    descriptors: Descriptors,
    hid: HidInterface,
    bridge: Bridge,
}

impl Ch9325 {
    /// A cable whose meter sends `packets` in turn, one per `period`
    /// polls once the host has set `baud` and `bits`.
    pub fn new(packets: Vec<Vec<u8>>, period: u32, baud: u16, bits: u8) -> Ch9325 {
        let hid = HidInterface::new(0, cable::report_descriptor(8, 8));
        Ch9325 {
            descriptors: cable::descriptors(
                VENDOR_ID,
                PRODUCT_ID,
                "WCH",
                "CH9325 cable emulator",
                &hid,
            ),
            hid,
            bridge: Bridge {
                packets: Packets::new(packets),
                reports: Reports::default(),
                stream: Stream::new(period, false),
                baud,
                bits,
                feature: Vec::new(),
            },
        }
    }
}

impl DeviceModel for Ch9325 {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if let Some(reply) = self.hid.control(&mut self.bridge, setup, data, v) {
            return reply;
        }
        v.report(format!(
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the chip answers",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!("OUT transfer to endpoint {ep}; the chip has none"));
        Reply::Stall
    }

    fn in_transfer(&mut self, _ep: u8, max: usize, v: &mut Violations) -> Reply {
        if self.bridge.reports.is_empty() && self.bridge.stream.poll() {
            self.bridge.send_packet();
        }
        self.bridge.reports.answer(max, v)
    }

    fn reset(&mut self) {
        self.hid.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feature(data: &[u8]) -> (Setup, Vec<u8>) {
        let s = Setup {
            request_type: 0x21,
            request: 9,
            value: 0x0300,
            index: 0,
            length: data.len() as u16,
        };
        (s, data.to_vec())
    }

    fn polls_until_data(dev: &mut Ch9325, v: &mut Violations) -> (usize, Vec<u8>) {
        for n in 1..100 {
            if let Reply::Data(d) = dev.in_transfer(1, 64, v) {
                return (n, d);
            }
        }
        panic!("no report");
    }

    #[test]
    fn streams_once_the_line_is_set() {
        let mut dev = Ch9325::new(cable::parse_packets(DEFAULT_PACKET, 1, 64).unwrap(), 4, 19200, 7);
        let mut v = Violations::default();
        for _ in 0..20 {
            assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak, "silent before the line is set");
        }
        // 19200/7o1, as ch9325_set_params lays it out.
        let (s, d) = feature(&[0x00, 0x4b, 0, 0, 2]);
        assert_eq!(dev.control(s, &d, &mut v), Reply::ack());
        let (n, first) = polls_until_data(&mut dev, &mut v);
        assert_eq!(n, 4);
        assert_eq!(first, [0xf7, 0x31, 0x31, 0x32, 0x33, 0x34, 0x35, 0x3b]);
        assert_eq!(
            dev.in_transfer(1, 64, &mut v),
            Reply::Data(vec![0xf7, 0x30, 0x30, 0x30, 0x3a, 0x30, 0x0d, 0x0a])
        );
        assert_eq!(polls_until_data(&mut dev, &mut v).0, 4);
        dev.in_transfer(1, 64, &mut v);
        // A new feature report starts the period over.
        dev.in_transfer(1, 64, &mut v);
        dev.control(s, &d, &mut v);
        assert_eq!(polls_until_data(&mut dev, &mut v).0, 4);
        // Writes are framed with the count; the meter ignores them.
        let out = Setup { request_type: 0x21, request: 9, value: 0x0203, index: 0, length: 8 };
        assert_eq!(dev.control(out, &[3, b'a', b'b', b'c', 0, 0, 0, 0], &mut v), Reply::ack());
        assert!(v.is_empty(), "{v:?}");
        // A short packet is one report.
        let mut short = Ch9325::new(vec![vec![0x41]], 2, 19200, 7);
        short.control(s, &d, &mut v);
        assert_eq!(polls_until_data(&mut short, &mut v).1, [0xf1, 0x41, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn flags_line_parameters_outside_the_layout() {
        let mut dev = Ch9325::new(vec![vec![1]], 4, 19200, 7);
        let mut v = Violations::default();
        let (s, d) = feature(&[0x00, 0x4b, 0, 0, 2, 0]);
        dev.control(s, &d, &mut v);
        assert_eq!(v.drain().len(), 1, "six bytes");
        let (s, d) = feature(&[0x60, 0x09, 0, 0, 2]);
        dev.control(s, &d, &mut v);
        assert_eq!(v.drain().len(), 1, "2400 baud");
        let (s, d) = feature(&[0x00, 0x4b, 0, 0, 3]);
        dev.control(s, &d, &mut v);
        assert_eq!(v.drain().len(), 1, "8 bits");
        let (s, d) = feature(&[0x00, 0x4b, 1, 0, 4]);
        dev.control(s, &d, &mut v);
        assert_eq!(v.drain().len(), 3, "byte 2, bits code, and the line");
        let (s, d) = feature(&[0x00, 0x4b]);
        dev.control(s, &d, &mut v);
        assert_eq!(v.drain().len(), 1, "short");
        let id = Setup { request_type: 0x21, request: 9, value: 0x0301, index: 0, length: 5 };
        dev.control(id, &[0x00, 0x4b, 0, 0, 2], &mut v);
        assert_eq!(v.drain().len(), 1, "report id 1");
        let out = Setup { request_type: 0x21, request: 9, value: 0x0200, index: 0, length: 3 };
        dev.control(out, &[0x00, 0x86, 0x66], &mut v);
        assert_eq!(v.drain().len(), 1, "a BU-86X request is no UART write");
        let get = Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 8 };
        assert_eq!(dev.control(get, &[], &mut v), Reply::Data(vec![0x00, 0x4b, 0, 0, 2]));
        assert_eq!(v.drain().len(), 1, "feature read back");
        let input = Setup { request_type: 0xa1, request: 1, value: 0x0100, index: 0, length: 8 };
        assert_eq!(dev.control(input, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1);
        assert!(build(&[("period".into(), "1".into())]).is_err());
        assert!(build(&[("bits".into(), "9".into())]).is_err());
        assert!(build(&[("baud".into(), "2400".into()), ("bits".into(), "8".into())]).is_ok());
    }
}
