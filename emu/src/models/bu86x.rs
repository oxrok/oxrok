// Counterpart in sigrok:
//   libsigrok/src/serial_hid_bu86x.c,
//   libsigrok/src/dmm/bm86x.c (the packet request),
//   Copyright (C) 2019 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   Copyright (C) 2014 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, François Revol
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

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
//!
//! The pedantic checks: an output report is exactly `00 86 66` (the
//! Pico answers any `00 xx 66` of 3 bytes or more, the C driver sends
//! only this one), a request comes only once the previous answer has
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
                         role; packet=HEX[,HEX...] (the meter's 24 byte answers, in turn)";

/// The USB vendor id.
pub const VENDOR_ID: u16 = 0x0820;
/// The USB product id.
pub const PRODUCT_ID: u16 = 0x0001;
/// The report length, both ways.
pub const REPORT_BYTES: usize = 8;
/// The request the host writes, report id stripped.
pub const REQUEST: [u8; 3] = [0x00, 0x86, 0x66];
/// A BM867s packet: 1.2345 V AC with 50.0 Hz on the second display.
pub const DEFAULT_PACKET: &str = "000001a0dbf8e47c01007cbebf0004008686868600000000";

/// A cable from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a malformed packet.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut packets = cable::parse_packets(DEFAULT_PACKET, 1, 64)?;
    for (k, v) in options {
        match k.as_str() {
            "packet" => packets = cable::parse_packets(v, 1, 64)?,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(Box::new(Bu86x::new(packets)))
}

/// The meter behind the cable.
#[derive(Clone, Debug)]
pub struct Meter {
    packets: Packets,
    reports: Reports,
    feature: Vec<u8>,
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

    fn request(&mut self, data: &[u8], v: &mut Violations) {
        if data != REQUEST {
            v.report(format!(
                "output report {}; the BM86x request is 00 86 66",
                crate::txlog::hex(data)
            ));
        }
        if data.len() >= 3 && data[2] == 0x66 {
            if !self.reports.is_empty() {
                v.report(format!(
                    "a request while {} reports of the previous answer are unread",
                    self.reports.len()
                ));
            }
            self.answer();
        }
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
            hid::OUTPUT => {
                self.request(data, v);
                Reply::ack()
            }
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
    }
}
