// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The UNI-T UT-D04 cable as libsigrok's `uni-t-dmm` driver reaches it:
//! a WCH CH9325 UART to HID bridge, `1a86:e008`, driven by raw USB and
//! not through the kernel's HID layer, with a meter behind it sending
//! its packets unasked.
//!
//! Sources. The descriptors, byte for byte, come from the `lsusb -v` of
//! the sigrok wiki's "Device cables/Info" page (bcdDevice 11.00; the
//! 37 byte report descriptor is rebuilt from its item list and its
//! length agrees). What the chip does comes from the wiki's "WCH CH9325"
//! page: a report at least every 12 ms, `f0` and zeros when the UART
//! brought nothing, 2400, 4800, 9600 and 19200 baud with anything else
//! falling back to 2400, byte 5 of the setting the data bits, and the
//! setting being required before the chip works. The driver's side is
//! `hardware/uni-t-dmm/protocol.c`, the framing `serial_hid_ch9325.c`.
//! Not known: the bus speed. `lsusb` does not say, and bMaxPacketSize0
//! 8, wMaxPacketSize 8 and bcdUSB 1.00 fit low speed too; this is full
//! speed on the word of a sibling's datasheet (WCH's CH9326, V1.6: "12Mbps
//! full-speed"), an assumption no golden depends on. That datasheet
//! differs in what it offers (other rates, a 9600 default), so it says
//! nothing else about this chip. Nor is known what a real chip answers
//! before the setting, beyond "must be sent". No CH9325 datasheet or
//! firmware was found. No cable was on the bench.
//!
//! Unlike `ch9325` (the Pico emulator's stand-in, endpoint 1, 64 bytes,
//! paced by NAKs so that a hidraw reader sees whole packets) this one
//! is the real chip's shape: endpoint `0x82` of 8 bytes, and every IN
//! transfer is answered, because `protocol.c` makes one synchronous
//! transfer per poll and takes a 1 s timeout as the end of the capture.
//! So the model only serves the raw driver. Through hidraw its idle
//! reports would flood usbhid at the server's pace; the HID route stays
//! with `ch9325`.
//!
//! The line setting is the point of the driver. `hid_chip_init` claims
//! interface 0 and sends `SET_REPORT` of feature report 0 with
//! `baud_lo baud_hi 00 00 03`: the 3 asks for 8 data bits whatever the
//! meter's frame, so with a 7o1 meter (`ES519xx`, `UT71x`) the parity bit
//! arrives in bit 7 of each byte, and the driver masks it off. `parity`
//! makes the model set that bit. A chip at the wrong baud rate would
//! see framing errors; what it would hand over is unknown, so the model
//! then stays idle (`f0`), an artifact a wrong driver id runs into.
//!
//! Pacing, in IN transfers, never in time: `idle=N` idle reports before
//! each packet, then the packet's bytes in reports of `chunk=A[,B...]`
//! bytes in turn (1 to 7). The default chunk follows the rate: 12 ms
//! holds 2 whole bytes at 2400 baud, and 7 or more at 9600 and 19200.
//!
//! Options: `packet=HEX[,HEX...]`, the meter's packets in turn, 1 to 64
//! bytes each (default a UT61E's); `baud=N`, the meter's rate (19200);
//! `parity=odd|none` (odd); `chunk=`; `idle=N` (3); `burst=N`, packets
//! after each line setting, then idle for good (no limit). Faults, all
//! off by default: `stall-report=1` STALLs the line setting;
//! `stall-in=N` STALLs the Nth IN transfer and `short-in=N` answers it
//! with 4 bytes (one-shot, which is an emulation artifact: a real STALL
//! halts the endpoint until cleared); `hold=N` answers N IN transfers
//! and then NAKs them all, a chip that has hung.
//!
//! Checks: the setting is five bytes on feature report 0 with bytes 2
//! to 4 as `hid_chip_init` writes them; an IN transfer takes the whole
//! 8 bytes; nothing reads a report by `GET_REPORT`, and the driver
//! writes nothing to the UART (the meters only talk).
//!
//! Not offered: a count nibble above 7 (`buf[0] & 0x0f` with 8 to 15,
//! which `protocol.c` would follow past its 8 byte buffer): the chip's
//! reports hold 7 at most, so no real device sends one.

use std::collections::VecDeque;

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::hid::{self, HidInterface, HidReports};
use crate::shells::hidcable as cable;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "uni-t-dmm";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "UNI-T UT-D04 cable (WCH CH9325, 1a86:e008) on raw USB, as libsigrok's \
                         uni-t-dmm drives it; packet=HEX[,HEX...], baud=N (19200), \
                         parity=odd|none, chunk=A[,B...], idle=N (3), burst=N, stall-report=1, \
                         stall-in=N, short-in=N, hold=N";

/// The USB vendor id.
pub const VENDOR_ID: u16 = 0x1a86;
/// The USB product id.
pub const PRODUCT_ID: u16 = 0xe008;
/// The interrupt IN endpoint.
pub const EP_IN: u8 = 2;
/// The interrupt OUT endpoint.
pub const EP_OUT: u8 = 2;
/// A report, both ways.
pub const REPORT_BYTES: usize = 8;
/// UART bytes per report.
pub const CHUNK: usize = 7;
/// The line setting, as `hid_chip_init` sends it.
pub const FEATURE_BYTES: usize = 5;
/// A UT61E packet (`tools/diffcli.sh --hw`'s), 1.2345 V.
pub const DEFAULT_PACKET: &str = "3131323334353b3030303a300d0a";

/// The report descriptor of the wiki's `lsusb -v`, 37 bytes.
const REPORT_DESCRIPTOR: [u8; 37] = [
    0x06, 0xa0, 0xff, // usage page 0xffa0
    0x09, 0x01, // usage 1
    0xa1, 0x01, // collection, application
    0x09, 0x01, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x08, 0x81, 0x02, // input, 8
    0x09, 0x02, 0x75, 0x08, 0x95, 0x08, 0x91, 0x02, // output, 8
    0x09, 0x03, 0x75, 0x08, 0x95, 0x05, 0xb1, 0x02, // feature, 5
    0xc0, // end collection
];

/// A cable from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut cfg = Config::default();
    for (k, v) in options {
        let nth = |k: &str, v: &str| cable::parse_number(k, v, 1..=100_000).map(Some);
        match k.as_str() {
            "packet" => cfg.packets = cable::parse_packets(v, 1, 64)?,
            "baud" => cfg.baud = cable::parse_number(k, v, 1..=0xffff)? as u16,
            "parity" => {
                cfg.odd = match v.as_str() {
                    "odd" => true,
                    "none" => false,
                    _ => return Err(Error::Invalid(format!("parity={v}: odd or none"))),
                };
            }
            "chunk" => cfg.chunks = Some(parse_chunks(v)?),
            "idle" => cfg.idle = cable::parse_number(k, v, 0..=1000)?,
            "burst" => cfg.burst = cable::parse_burst(v)?,
            "stall-report" => cfg.stall_report = cable::parse_number(k, v, 0..=1)? == 1,
            "stall-in" => cfg.stall_in = nth(k, v)?,
            "short-in" => cfg.short_in = nth(k, v)?,
            "hold" => cfg.hold = Some(cable::parse_number(k, v, 0..=100_000)?),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(Box::new(UniTDmm::new(cfg)))
}

fn parse_chunks(value: &str) -> Result<Vec<usize>> {
    value
        .split(',')
        .map(|c| cable::parse_number("chunk", c, 1..=CHUNK as u32).map(|n| n as usize))
        .collect()
}

/// The meter behind the cable, the pacing and the faults.
#[derive(Clone, Debug)]
pub struct Config {
    packets: Vec<Vec<u8>>,
    baud: u16,
    odd: bool,
    chunks: Option<Vec<usize>>,
    idle: u32,
    burst: Option<u32>,
    stall_report: bool,
    stall_in: Option<u32>,
    short_in: Option<u32>,
    hold: Option<u32>,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            packets: cable::parse_packets(DEFAULT_PACKET, 1, 64).expect("the default packet"),
            baud: 19200,
            odd: true,
            chunks: None,
            idle: 3,
            burst: None,
            stall_report: false,
            stall_in: None,
            short_in: None,
            hold: None,
        }
    }
}

/// The chip's rate for a requested one: the wiki's list, 2400 otherwise.
fn chip_rate(requested: u16) -> u16 {
    if matches!(requested, 2400 | 4800 | 9600 | 19200) { requested } else { 2400 }
}

/// Whole bytes of the meter's UART in the 12 ms of one report.
fn default_chunk(baud: u16) -> usize {
    (usize::from(baud) * 12 / 10_000).clamp(1, CHUNK)
}

/// A byte as the chip reads it at 8 data bits from a 7o1 meter: the odd
/// parity bit is bit 7.
fn with_odd_parity(b: u8) -> u8 {
    let b = b & 0x7f;
    if b.count_ones().is_multiple_of(2) { b | 0x80 } else { b }
}

/// The chip and what the feature report set.
#[derive(Clone, Debug)]
struct Bridge {
    cfg: Config,
    /// The line matches the meter's, so its bytes get through.
    listening: bool,
    next_packet: usize,
    sent: u32,
    idle_left: u32,
    uart: VecDeque<u8>,
    next_chunk: usize,
    /// IN transfers answered so far, faults included.
    ins: u32,
}

impl Bridge {
    fn line(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        if self.cfg.stall_report {
            return Reply::Stall;
        }
        if data.len() != FEATURE_BYTES {
            v.report(format!(
                "line setting of {} bytes ({}); hid_chip_init sends {FEATURE_BYTES}",
                data.len(),
                crate::txlog::hex(data)
            ));
        }
        if data.len() < 2 {
            return Reply::ack();
        }
        if data.len() >= FEATURE_BYTES && (data[2], data[3], data[4]) != (0, 0, 3) {
            v.report(format!(
                "line setting bytes 2 to 4 are {:02x} {:02x} {:02x}; hid_chip_init sends 00 00 03",
                data[2], data[3], data[4]
            ));
        }
        let baud = chip_rate(u16::from_le_bytes([data[0], data[1]]));
        self.listening = baud == self.cfg.baud;
        self.sent = 0;
        self.idle_left = self.cfg.idle;
        self.uart.clear();
        Reply::ack()
    }

    fn next_report(&mut self) -> Vec<u8> {
        if self.uart.is_empty() && self.listening {
            if self.idle_left > 0 {
                self.idle_left -= 1;
            } else if self.cfg.burst.is_none_or(|b| self.sent < b) {
                let p = &self.cfg.packets[self.next_packet];
                let odd = self.cfg.odd;
                self.uart.extend(p.iter().map(|&b| if odd { with_odd_parity(b) } else { b }));
                self.next_packet = (self.next_packet + 1) % self.cfg.packets.len();
                self.sent += 1;
                self.idle_left = self.cfg.idle;
            }
        }
        let size = match &self.cfg.chunks {
            Some(list) => {
                let s = list[self.next_chunk % list.len()];
                if !self.uart.is_empty() {
                    self.next_chunk += 1;
                }
                s
            }
            None => default_chunk(self.cfg.baud),
        };
        let n = size.min(self.uart.len());
        let mut r = vec![0xf0 | n as u8];
        r.extend(self.uart.drain(..n));
        r.resize(REPORT_BYTES, 0);
        r
    }
}

impl HidReports for Bridge {
    fn get_report(&mut self, kind: u8, id: u8, _length: usize, v: &mut Violations) -> Reply {
        v.report(format!("GET_REPORT of kind {kind}, id {id}; the driver reads endpoint 2"));
        Reply::Stall
    }

    fn set_report(&mut self, kind: u8, id: u8, data: &[u8], v: &mut Violations) -> Reply {
        if kind != hid::FEATURE {
            v.report(format!("SET_REPORT of kind {kind}; the driver sends the feature report"));
            return Reply::Stall;
        }
        if id != 0 {
            v.report(format!("feature report id {id}; the line setting is report 0"));
        }
        self.line(data, v)
    }
}

/// The emulated cable.
pub struct UniTDmm {
    descriptors: Descriptors,
    hid: HidInterface,
    bridge: Bridge,
}

impl UniTDmm {
    /// A cable with this meter behind it.
    pub fn new(cfg: Config) -> UniTDmm {
        let hid = HidInterface::new(0, REPORT_DESCRIPTOR.to_vec()).with_version(0x0100);
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 2, [3, 0, 0], 0));
        body.extend_from_slice(&hid.descriptor());
        body.extend_from_slice(&descriptor::endpoint(0x80 | EP_IN, 3, 8, 5));
        body.extend_from_slice(&descriptor::endpoint(EP_OUT, 3, 8, 5));
        let device = DeviceDescriptor {
            usb: 0x0100,
            class: [0, 0, 0],
            max_packet0: 8,
            vendor: VENDOR_ID,
            product: PRODUCT_ID,
            device: 0x1100,
            strings: [1, 2, 0],
            configurations: 1,
        };
        UniTDmm {
            descriptors: Descriptors {
                speed: Speed::Full,
                device: device.bytes(),
                configs: vec![descriptor::configuration(1, 4, 0x80, 100, &body)],
                // lsusb shows the manufacturer with a stray 0x11, the
                // product, and an empty iConfiguration, string 4.
                strings: vec![
                    "WCH.CN \u{11}".to_owned(),
                    "USB to Serial".to_owned(),
                    String::new(),
                    String::new(),
                ],
            },
            hid,
            bridge: Bridge {
                cfg,
                listening: false,
                next_packet: 0,
                sent: 0,
                idle_left: 0,
                uart: VecDeque::new(),
                next_chunk: 0,
                ins: 0,
            },
        }
    }
}

impl DeviceModel for UniTDmm {
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

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!(
            "OUT transfer to endpoint {ep} ({}); the driver writes nothing to the UART",
            crate::txlog::hex(data)
        ));
        Reply::ack()
    }

    fn in_transfer(&mut self, _ep: u8, max: usize, v: &mut Violations) -> Reply {
        let b = &mut self.bridge;
        if b.cfg.hold.is_some_and(|h| b.ins >= h) {
            return Reply::Nak;
        }
        if max < REPORT_BYTES {
            v.report(format!("interrupt IN of {max} bytes; a report has {REPORT_BYTES}"));
        }
        b.ins += 1;
        if b.cfg.stall_in == Some(b.ins) {
            return Reply::Stall;
        }
        let mut r = b.next_report();
        if b.cfg.short_in == Some(b.ins) {
            r.truncate(4);
        }
        Reply::Data(r)
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

    fn dev(options: &[(&str, &str)]) -> Box<dyn DeviceModel> {
        let o: Vec<_> = options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        build(&o).unwrap()
    }

    fn data(r: Reply) -> Vec<u8> {
        match r {
            Reply::Data(d) => d,
            other => panic!("{other:?}"),
        }
    }

    const LINE_19200: [u8; 5] = [0x00, 0x4b, 0, 0, 3];
    const LINE_2400: [u8; 5] = [0x60, 0x09, 0, 0, 3];

    #[test]
    fn descriptors_are_the_wikis() {
        let d = dev(&[]);
        let c = &d.descriptors().configs[0];
        assert_eq!(c.len(), 41, "wTotalLength");
        assert_eq!(REPORT_DESCRIPTOR.len(), 37, "wDescriptorLength");
        assert_eq!(c[2], 41);
        assert_eq!(&d.descriptors().device[2..4], &[0x00, 0x01], "bcdUSB 1.00");
        assert_eq!(&d.descriptors().device[12..14], &[0x00, 0x11], "bcdDevice 11.00");
        assert!(c.windows(7).any(|w| w == [7, 5, 0x82, 3, 8, 0, 5]));
        assert!(c.windows(7).any(|w| w == [7, 5, 0x02, 3, 8, 0, 5]));
    }

    #[test]
    fn idle_until_set_then_packets_with_parity() {
        let mut d = dev(&[("idle", "1"), ("packet", "3103")]);
        let mut v = Violations::default();
        assert_eq!(data(d.in_transfer(2, 8, &mut v))[0], 0xf0, "idle before the setting");
        let (s, b) = feature(&LINE_19200);
        assert_eq!(d.control(s, &b, &mut v), Reply::ack());
        assert_eq!(data(d.in_transfer(2, 8, &mut v))[0], 0xf0, "idle=1");
        // 0x31 has three bits set, so no parity bit; 0x03 has two.
        assert_eq!(data(d.in_transfer(2, 8, &mut v)), [0xf2, 0x31, 0x83, 0, 0, 0, 0, 0]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn chunks_follow_the_option_and_the_rate() {
        let mut d = dev(&[
            ("idle", "0"),
            ("baud", "2400"),
            ("parity", "none"),
            ("packet", "0102030405"),
            ("chunk", "1,2"),
        ]);
        let mut v = Violations::default();
        let (s, b) = feature(&LINE_2400);
        d.control(s, &b, &mut v);
        assert_eq!(data(d.in_transfer(2, 8, &mut v))[..2], [0xf1, 1]);
        assert_eq!(data(d.in_transfer(2, 8, &mut v))[..3], [0xf2, 2, 3]);
        assert_eq!(data(d.in_transfer(2, 8, &mut v))[..2], [0xf1, 4]);
        assert_eq!(data(d.in_transfer(2, 8, &mut v))[..2], [0xf1, 5], "the last, cut short");
        assert_eq!(default_chunk(2400), 2);
        assert_eq!(default_chunk(9600), 7);
        assert_eq!(default_chunk(19200), 7);
    }

    #[test]
    fn the_wrong_rate_hears_nothing_and_a_bad_rate_is_2400() {
        let mut d = dev(&[("idle", "0"), ("baud", "2400"), ("parity", "none")]);
        let mut v = Violations::default();
        let (s, b) = feature(&LINE_19200);
        d.control(s, &b, &mut v);
        assert_eq!(data(d.in_transfer(2, 8, &mut v))[0], 0xf0);
        // 19230 is no rate of the chip's: Note 1 of api.c.
        let (s, b) = feature(&[0x1e, 0x4b, 0, 0, 3]);
        d.control(s, &b, &mut v);
        assert_ne!(data(d.in_transfer(2, 8, &mut v))[0], 0xf0, "2400 after all");
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn burst_stops_the_meter_until_the_next_setting() {
        let mut d = dev(&[("idle", "0"), ("burst", "1"), ("packet", "31")]);
        let mut v = Violations::default();
        let (s, b) = feature(&LINE_19200);
        d.control(s, &b, &mut v);
        assert_eq!(data(d.in_transfer(2, 8, &mut v))[0], 0xf1);
        assert_eq!(data(d.in_transfer(2, 8, &mut v))[0], 0xf0);
        d.control(s, &b, &mut v);
        assert_eq!(data(d.in_transfer(2, 8, &mut v))[0], 0xf1);
    }

    #[test]
    fn faults() {
        let mut v = Violations::default();
        let (s, b) = feature(&LINE_19200);
        let mut d = dev(&[("stall-report", "1")]);
        assert_eq!(d.control(s, &b, &mut v), Reply::Stall);
        let mut d = dev(&[("stall-in", "2")]);
        d.in_transfer(2, 8, &mut v);
        assert_eq!(d.in_transfer(2, 8, &mut v), Reply::Stall);
        assert_eq!(data(d.in_transfer(2, 8, &mut v)).len(), 8, "one-shot");
        let mut d = dev(&[("short-in", "1")]);
        assert_eq!(data(d.in_transfer(2, 8, &mut v)).len(), 4);
        let mut d = dev(&[("hold", "1")]);
        assert_eq!(data(d.in_transfer(2, 8, &mut v)).len(), 8);
        assert_eq!(d.in_transfer(2, 8, &mut v), Reply::Nak);
        assert_eq!(d.in_transfer(2, 8, &mut v), Reply::Nak);
        assert!(v.is_empty(), "{v:?}");
        for bad in [("parity", "even"), ("chunk", "8"), ("chunk", "0"), ("nope", "1")] {
            assert!(build(&[(bad.0.to_owned(), bad.1.to_owned())]).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn pedantic_checks_fire() {
        let mut d = dev(&[]);
        let mut v = Violations::default();
        let (s, b) = feature(&[0x00, 0x4b, 0, 0, 3, 0]);
        d.control(s, &b, &mut v);
        assert_eq!(v.drain().len(), 1, "six bytes");
        let (s, b) = feature(&[0x00]);
        d.control(s, &b, &mut v);
        assert_eq!(v.drain().len(), 1, "one byte");
        let (s, b) = feature(&[0x00, 0x4b, 0, 0, 2]);
        d.control(s, &b, &mut v);
        assert_eq!(v.drain().len(), 1, "the serial route's data bits code");
        let id = Setup { request_type: 0x21, request: 9, value: 0x0301, index: 0, length: 5 };
        d.control(id, &LINE_19200, &mut v);
        assert_eq!(v.drain().len(), 1, "report id 1");
        let out = Setup { request_type: 0x21, request: 9, value: 0x0200, index: 0, length: 5 };
        assert_eq!(d.control(out, &LINE_19200, &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "an output report");
        let get = Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 5 };
        assert_eq!(d.control(get, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "a read back");
        let vendor = Setup { request_type: 0x40, request: 1, value: 0, index: 0, length: 0 };
        assert_eq!(d.control(vendor, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "a vendor request");
        d.out(2, &[1, 2], &mut v);
        assert_eq!(v.drain().len(), 1, "a UART write");
        d.in_transfer(2, 4, &mut v);
        assert_eq!(v.drain().len(), 1, "a short IN");
    }
}
