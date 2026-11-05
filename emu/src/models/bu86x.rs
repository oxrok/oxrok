// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The Brymen BU-86X infrared cable of the `BM86x` meters, as the Pico
//! emulator plays it (`hil/pico-hidcable`, `MODE bu86x`): a HID device
//! `0820:0001`, vendor `Brymen`, with 8 byte input and output reports
//! and no report ids, behind which the meter answers requests.
//!
//! The cable has fixed line parameters and no feature reports. The
//! host asks for a reading with the output report `00 86 66`
//! (`sr_brymen_bm86x_packet_request` writes `00 00 86 66`, report id 0
//! first, which hidraw strips; with no OUT endpoint the kernel sends it
//! as `SET_REPORT`), and the meter answers its 24 byte packet in three
//! input reports of 8 bytes. Nothing is sent unasked, so the number of
//! packets in a run is the number of requests, whatever the timing.
//!
//! Option `packet=HEX[,HEX...]`: the packets the meter answers, in
//! turn, 1 to 64 bytes each, sent in 8 byte reports padded with zeros
//! as the Pico does; the default is the `diffcli.sh --hw` packet, a
//! BM867s showing 1.2345 V AC with 50.0 Hz on the second display.
//! Option `meter=bm52x`: the meter behind the cable is a `BM52x`
//! (`brymen-bm52x`), asked with `00 52 66` (libsigrok's
//! `sr_brymen_bm52x_packet_request` writes `00 00 52 66`), whose
//! packets carry 0x52 in bytes 16 to 19; the default packet is then
//! 12.34 V DC on the main display and 50.00 Hz on the second. Unlike
//! the `BM86x` request, the `BM52x` one checks that its write went out,
//! which is what a `stall=` case shows. The default is `bm86x`.
//! Option `drop=N`: the meter leaves every Nth request unanswered, as
//! one the infrared link lost (0, the default, answers all), so that the
//! driver's re-request after 500 ms without an answer
//! (`req_packet`'s timeout) has a case. A request is counted from the
//! start of the model, which the harness builds anew for every case.
//! Option `lose=N[,N...]`: the requests of these numbers (1 for the
//! first) go unanswered, so that two in a row can be lost. Option
//! `stall=N[,N...]`: the cable STALLs the `SET_REPORT` of these
//! requests, as a cable whose infrared side is busy might: the host's
//! write fails (`hid_write` returns -1) and nothing is answered.
//!
//! The pedantic checks: an output report is exactly the meter's
//! request, `00 86 66` or `00 52 66` (the Pico answers any `00 xx 66`
//! of 3 bytes or more, the C driver sends only its meter's), a request comes only once the previous answer has
//! been read, and the host neither writes nor reads a feature report,
//! nor reads an input report by `GET_REPORT`, none of which the cable
//! has a use for.

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Violations};
use crate::shells::hidcable as cable;
use crate::shells::hid::{self, HidInterface, HidReports};
use crate::{Error, Result};
use crate::shells::hidcable::{Packets, Reports};

/// The model name.
pub const NAME: &str = "bu86x";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Brymen BU-86X cable of a BM86x meter (HID, 0820:0001), the Pico emulator's \
                         role; packet=HEX[,HEX...] (the meter's 24 byte answers, in turn), meter=bm86x|bm52x, drop=N \
                         (every Nth request unanswered), lose=N[,N...] (these requests \
                         unanswered), stall=N[,N...] (these requests STALLed)";

/// The USB vendor id.
pub const VENDOR_ID: u16 = 0x0820;
/// The USB product id.
pub const PRODUCT_ID: u16 = 0x0001;
/// The report length, both ways.
pub const REPORT_BYTES: usize = 8;
/// The request the host writes, report id stripped.
pub const REQUEST: [u8; 3] = [0x00, 0x86, 0x66];
/// A `BM52x`'s request.
pub const BM52X_REQUEST: [u8; 3] = [0x00, 0x52, 0x66];
/// A `BM52x` packet: 12.34 V DC, auto range, with 50.00 Hz on the second
/// display.
pub const BM52X_PACKET: &str = "002000a0dbe9e4006dbfafaf000120005252525210000000";
/// A BM867s packet: 1.2345 V AC with 50.0 Hz on the second display.
pub const DEFAULT_PACKET: &str = "000001a0dbf8e47c01007cbebf0004008686868600000000";

/// A cable from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a malformed packet.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut packets = None;
    let mut request = REQUEST;
    let mut drop = 0;
    let mut lose = Vec::new();
    let mut stall = Vec::new();
    for (k, v) in options {
        match k.as_str() {
            "packet" => packets = Some(cable::parse_packets(v, 1, 64)?),
            "meter" => {
                request = match v.as_str() {
                    "bm86x" => REQUEST,
                    "bm52x" => BM52X_REQUEST,
                    _ => return Err(Error::Invalid(format!("meter={v}: bm86x or bm52x"))),
                }
            }
            "drop" => drop = cable::parse_number(k, v, 0..=1000)?,
            "lose" => lose = parse_requests(k, v)?,
            "stall" => stall = parse_requests(k, v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let default = if request == REQUEST { DEFAULT_PACKET } else { BM52X_PACKET };
    let packets = match packets {
        Some(p) => p,
        None => cable::parse_packets(default, 1, 64)?,
    };
    let mut cable = Bu86x::new(packets);
    cable.meter.request = request;
    cable.meter.drop = drop;
    cable.meter.lose = lose;
    cable.meter.stall = stall;
    Ok(Box::new(cable))
}

/// Parses `N[,N...]`, request numbers from 1.
fn parse_requests(key: &str, value: &str) -> Result<Vec<u32>> {
    value.split(',').map(|n| cable::parse_number(key, n, 1..=1000)).collect()
}

/// The meter behind the cable.
#[derive(Clone, Debug)]
pub struct Meter {
    packets: Packets,
    reports: Reports,
    feature: Vec<u8>,
    /// The request the meter answers.
    request: [u8; 3],
    /// Every `drop`th request goes unanswered; 0 answers all.
    drop: u32,
    /// Requests (numbered from 1) that go unanswered.
    lose: Vec<u32>,
    /// Requests whose `SET_REPORT` the cable STALLs.
    stall: Vec<u32>,
    requests: u32,
}

impl Meter {
    /// The packet split into reports the way the Pico sends it.
    fn answer(&mut self) {
        let p = self.packets.take();
        self.reports.push(p.chunks(REPORT_BYTES).map(|c| {
            let mut r = c.to_vec();
            r.resize(REPORT_BYTES, 0);
            r
        }));
    }

    /// An output report: the reply to its `SET_REPORT`.
    fn request(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        if data != self.request {
            v.report(format!(
                "output report {}; the meter's request is {}",
                crate::txlog::hex(data),
                crate::txlog::hex(&self.request)
            ));
        }
        if data.len() >= 3 && data[2] == 0x66 {
            if !self.reports.is_empty() {
                v.report(format!(
                    "a request while {} reports of the previous answer are unread",
                    self.reports.len()
                ));
            }
            self.requests += 1;
            if self.stall.contains(&self.requests) {
                return Reply::Stall;
            }
            let dropped = self.drop != 0 && self.requests.is_multiple_of(self.drop);
            if !dropped && !self.lose.contains(&self.requests) {
                self.answer();
            }
        }
        Reply::ack()
    }
}

impl HidReports for Meter {
    fn get_report(&mut self, kind: u8, id: u8, length: usize, v: &mut Violations) -> Reply {
        if kind != hid::FEATURE {
            v.report(format!("GET_REPORT of kind {kind}; the cable's input comes on endpoint 1"));
            // The firmware answers no data: a STALL.
            return Reply::Stall;
        }
        v.report(format!("GET_REPORT of feature report {id}; the cable has no settings"));
        Reply::Data(self.feature[..length.min(self.feature.len())].to_vec())
    }

    fn set_report(&mut self, kind: u8, id: u8, data: &[u8], v: &mut Violations) -> Reply {
        if id != 0 {
            v.report(format!("SET_REPORT with report id {id}; the cable has none"));
        }
        match kind {
            hid::OUTPUT => self.request(data, v),
            hid::FEATURE => {
                v.report("feature report to the BU-86X, whose line parameters are fixed");
                data.clone_into(&mut self.feature);
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
pub struct Bu86x {
    descriptors: Descriptors,
    hid: HidInterface,
    meter: Meter,
}

impl Bu86x {
    /// A cable whose meter answers `packets` in turn.
    pub fn new(packets: Vec<Vec<u8>>) -> Bu86x {
        let hid = HidInterface::new(0, cable::report_descriptor(8, 8));
        Bu86x {
            descriptors: cable::descriptors(
                VENDOR_ID,
                PRODUCT_ID,
                "Brymen",
                "BU-86X emulator",
                &hid,
            ),
            hid,
            meter: Meter {
                packets: Packets::new(packets),
                reports: Reports::default(),
                feature: Vec::new(),
                request: REQUEST,
                drop: 0,
                lose: Vec::new(),
                stall: Vec::new(),
                requests: 0,
            },
        }
    }
}

impl DeviceModel for Bu86x {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if let Some(reply) = self.hid.control(&mut self.meter, setup, data, v) {
            return reply;
        }
        v.report(format!(
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the cable answers",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!("OUT transfer to endpoint {ep}; the cable has none"));
        Reply::Stall
    }

    fn in_transfer(&mut self, _ep: u8, max: usize, v: &mut Violations) -> Reply {
        self.meter.reports.answer(max, v)
    }

    fn reset(&mut self) {
        self.hid.reset();
        self.meter.reports = Reports::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn output(data: &[u8]) -> (Setup, Vec<u8>) {
        let s = Setup {
            request_type: 0x21,
            request: 9,
            value: 0x0200,
            index: 0,
            length: data.len() as u16,
        };
        (s, data.to_vec())
    }

    fn cable() -> Bu86x {
        Bu86x::new(cable::parse_packets(&format!("{DEFAULT_PACKET},0102"), 1, 64).unwrap())
    }

    #[test]
    fn answers_a_request_in_three_reports() {
        let mut dev = cable();
        let mut v = Violations::default();
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak, "nothing unasked");
        let (s, d) = output(&REQUEST);
        assert_eq!(dev.control(s, &d, &mut v), Reply::ack());
        let mut got = Vec::new();
        for _ in 0..3 {
            match dev.in_transfer(1, 64, &mut v) {
                Reply::Data(r) => {
                    assert_eq!(r.len(), 8);
                    got.extend(r);
                }
                other => panic!("{other:?}"),
            }
        }
        assert_eq!(got, cable::parse_packets(DEFAULT_PACKET, 1, 64).unwrap()[0]);
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak);
        // The second packet, padded to a report.
        dev.control(s, &d, &mut v);
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Data(vec![1, 2, 0, 0, 0, 0, 0, 0]));
        assert!(v.is_empty(), "{v:?}");
        // And the first again.
        dev.control(s, &d, &mut v);
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Data(vec![0, 0, 1, 0xa0, 0xdb, 0xf8, 0xe4, 0x7c]));
        let rd = Setup { request_type: 0x81, request: 6, value: 0x2200, index: 0, length: 255 };
        assert_eq!(dev.control(rd, &[], &mut v), Reply::Data(cable::report_descriptor(8, 8)));
    }

    #[test]
    fn flags_what_the_cable_does_not_take() {
        let mut dev = cable();
        let mut v = Violations::default();
        let (s, d) = output(&[0x00, 0x52, 0x66]);
        dev.control(s, &d, &mut v);
        assert_eq!(v.drain().len(), 1, "another request byte");
        assert!(matches!(dev.in_transfer(1, 64, &mut v), Reply::Data(_)), "the Pico answers it");
        let (s, d) = output(&REQUEST);
        dev.control(s, &d, &mut v);
        assert_eq!(v.drain().len(), 1, "a request before the answer was read");
        let (s, d) = output(&[0x00, 0x86, 0x66, 0, 0, 0, 0, 0]);
        dev.control(s, &d, &mut v);
        assert_eq!(v.drain().len(), 2, "a padded request, still unread reports");
        let (s, d) = output(&[0x00, 0x86]);
        dev.control(s, &d, &mut v);
        assert_eq!(v.drain().len(), 1, "a short request");
        let feature = Setup { request_type: 0x21, request: 9, value: 0x0300, index: 0, length: 2 };
        assert_eq!(dev.control(feature, &[1, 2], &mut v), Reply::ack());
        assert_eq!(v.drain().len(), 1, "a feature report");
        let get = Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 8 };
        assert_eq!(dev.control(get, &[], &mut v), Reply::Data(vec![1, 2]));
        assert_eq!(v.drain().len(), 1, "a feature read");
        let input = Setup { request_type: 0xa1, request: 1, value: 0x0100, index: 0, length: 8 };
        assert_eq!(dev.control(input, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "an input report by GET_REPORT");
        assert_eq!(dev.out(1, &[0], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1);
        assert!(build(&[("packet".into(), "0".into())]).is_err());
        assert!(build(&[("period".into(), "1".into())]).is_err());
        assert!(build(&[("drop".into(), "x".into())]).is_err());
    }

    #[test]
    fn drops_every_nth_request() {
        let mut dev = cable();
        dev.meter.drop = 2;
        let mut v = Violations::default();
        let (s, d) = output(&REQUEST);
        dev.control(s, &d, &mut v);
        for _ in 0..3 {
            assert!(matches!(dev.in_transfer(1, 64, &mut v), Reply::Data(_)));
        }
        dev.control(s, &d, &mut v);
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak, "the second is lost");
        dev.control(s, &d, &mut v);
        assert!(matches!(dev.in_transfer(1, 64, &mut v), Reply::Data(_)), "the re-request is not");
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn loses_and_stalls_the_requests_named() {
        let mut dev = cable();
        dev.meter.lose = vec![2, 3];
        dev.meter.stall = vec![4];
        let mut v = Violations::default();
        let (s, d) = output(&REQUEST);
        assert_eq!(dev.control(s, &d, &mut v), Reply::ack());
        for _ in 0..3 {
            assert!(matches!(dev.in_transfer(1, 64, &mut v), Reply::Data(_)));
        }
        for n in [2, 3] {
            assert_eq!(dev.control(s, &d, &mut v), Reply::ack());
            assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak, "request {n} is lost");
        }
        assert_eq!(dev.control(s, &d, &mut v), Reply::Stall, "request 4 stalls");
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak);
        assert_eq!(dev.control(s, &d, &mut v), Reply::ack());
        assert!(matches!(dev.in_transfer(1, 64, &mut v), Reply::Data(_)), "request 5 is answered");
        assert!(v.is_empty(), "{v:?}");
        assert!(build(&[("lose".into(), "0".into())]).is_err());
        assert!(build(&[("stall".into(), "1,x".into())]).is_err());
        assert!(build(&[("stall".into(), "1,2".into()), ("lose".into(), "3".into())]).is_ok());
    }

    #[test]
    fn a_bm52x_behind_the_cable() {
        assert!(build(&[("meter".into(), "bm82x".into())]).is_err());
        let mut dev = Bu86x::new(cable::parse_packets(BM52X_PACKET, 1, 64).unwrap());
        dev.meter.request = BM52X_REQUEST;
        let mut v = Violations::default();
        let (s, d) = output(&BM52X_REQUEST);
        assert_eq!(dev.control(s, &d, &mut v), Reply::ack());
        let mut got = Vec::new();
        for _ in 0..3 {
            if let Reply::Data(r) = dev.in_transfer(1, 64, &mut v) {
                got.extend(r);
            }
        }
        assert_eq!(got[16..20], [0x52; 4]);
        assert!(v.is_empty(), "{v:?}");
        let (s, d) = output(&REQUEST);
        dev.control(s, &d, &mut v);
        assert_eq!(v.drain().len(), 1, "a BM86x request to a BM52x");
    }
}
