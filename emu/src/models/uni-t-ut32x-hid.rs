// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A UNI-T `UT32x` thermometer on its own cable, the instrument of `uni-t-ut32x`
//! behind a WCH CH9325 UART to HID bridge, as a tool reaches it through hidraw.
//!
//! Sources. The descriptors: the `lsusb -v` of the sigrok wiki's "UNI-T UT325/Info"
//! page, `1a86:e008`, bcdDevice 12.00, bcdUSB 1.00, HID 1.00, interrupt endpoints
//! 0x82 and 0x02 of 8 bytes with bInterval 5. That page shows no string texts and
//! parses 9 bytes of the 37 of its report descriptor, so those two are taken from
//! the "Device cables/Info" lsusb of the same chip, bcdDevice 11.00 (see
//! `uni-t-dmm`). The speed is the assumption `uni-t-dmm` states. The chip's habits:
//! the wiki's "WCH CH9325" page. An input report is `f0 | count` and up to 7 UART
//! bytes, at least every 12 ms and `f0` alone when the UART brought nothing, which
//! at 2400 baud leaves 2 bytes to a report. Rates are 2400, 4800, 9600 and 19200,
//! anything else falling back to 2400. The setting must come first. The output
//! report's form is what libsigrok's `ch9325_write_bytes` sends, its count without
//! the `f0`. What a real chip sends beyond its count is not known. No cable was on
//! the bench.
//!
//! The setting is `SET_REPORT` of feature report 0, five bytes: the rate, little
//! endian, two unknown bytes, and data bits minus 5.
//!
//! Emulation artifacts:
//!
//! - It NAKs where the chip sends an idle `f0` every 12 ms. Through hidraw those
//!   would come at the server's pace, and the number a tool reads would depend on
//!   the time. `idle=N` puts N of them before each record instead.
//! - The instrument's period counts offers of the held interrupt transfer, about
//!   one per 20 ms and more when a report has just gone.
//! - With the setting at another rate than the instrument's, or none yet, nothing
//!   passes either way: what a real chip hands over then is not known.
//!
//! Options, besides the instrument's (see `uni-t-ut32x`): `baud=N` and `bits=N`, the
//! instrument's line (2400, 8); `chunk=N`, UART bytes per report, 1 to 7 (2);
//! `idle=N`, idle reports before each record (0). Faults, all off by default:
//! `stall-report=1` STALLs the setting, as a chip that has hung, and `stall-out=N`
//! STALLs the Nth OUT transfer, one-shot, which is an emulation artifact;
//! `bad-report=HH` puts a report with this first byte behind the first of each record.
//! No chip is known to send that, only `ch9325_read_bytes` checks for it, and it is
//! here for what the check's error does.
//!
//! The pedantic checks: the setting is the five bytes above, with bytes 2 and 3
//! zero and a known data bits code; an output report is 8 bytes, its count 1 to 7;
//! nothing reads a report by `GET_REPORT`. The bytes behind the count are not
//! checked: `ch9325_write_bytes` leaves them uninitialised.

use super::uni_t_ut32x::Thermometer;
use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::hid::{self, HidInterface, HidReports};
use crate::shells::hidcable::{self as cable, Reports};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "uni-t-ut32x-hid";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A UNI-T UT32x thermometer on its CH9325 cable (HID, 1a86:e008); live=, \
                         stored=, slots=, period=, burst= as uni-t-ut32x, baud=N (2400), bits=N \
                         (8), chunk=N (2), idle=N, stall-report=1, stall-out=N, bad-report=HH";

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
/// UART bytes in a report, at most.
pub const CHUNK: usize = 7;
/// The line setting, report id stripped.
pub const FEATURE_BYTES: usize = 5;

/// The 37 bytes of the chip's report descriptor.
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
    let mut chip = Chip::default();
    for (k, v) in options {
        if chip.meter.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "bad-report" => {
                let list = cable::parse_packets(v, 1, 1)?;
                chip.bad_report = Some(list[0][0]);
            }
            "baud" => chip.baud = cable::parse_number(k, v, 1..=0xffff)? as u16,
            "bits" => chip.bits = cable::parse_number(k, v, 5..=8)? as u8,
            "chunk" => chip.chunk = cable::parse_number(k, v, 1..=CHUNK as u32)? as usize,
            "idle" => chip.idle = cable::parse_number(k, v, 0..=1000)?,
            "stall-report" => chip.stall_report = cable::parse_number(k, v, 0..=1)? == 1,
            "stall-out" => chip.stall_out = Some(cable::parse_number(k, v, 1..=100_000)?),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(Box::new(Cable::new(chip)))
}

/// The rate a setting leaves the chip at.
fn rate_of(asked: u16) -> u16 {
    match asked {
        2400 | 4800 | 9600 | 19200 => asked,
        _ => 2400,
    }
}

/// The chip and the instrument behind it.
#[derive(Clone, Debug)]
struct Chip {
    meter: Thermometer,
    baud: u16,
    bits: u8,
    chunk: usize,
    idle: u32,
    stall_report: bool,
    stall_out: Option<u32>,
    bad_report: Option<u8>,
    /// The line as the host set it, if it did.
    line: Option<(u16, u8)>,
    reports: Reports,
    outs: u32,
}

impl Default for Chip {
    fn default() -> Chip {
        Chip {
            meter: Thermometer::default(),
            baud: 2400,
            bits: 8,
            chunk: 2,
            idle: 0,
            stall_report: false,
            stall_out: None,
            bad_report: None,
            line: None,
            reports: Reports::default(),
            outs: 0,
        }
    }
}

impl Chip {
    fn line_matches(&self) -> bool {
        self.line == Some((self.baud, self.bits))
    }

    /// A record as the reports that carry it, behind its idle ones.
    fn reports_of(&self, record: &[u8]) -> Vec<Vec<u8>> {
        let idle = (0..self.idle).map(|_| vec![0xf0, 0, 0, 0, 0, 0, 0, 0]);
        let data = record.chunks(self.chunk).map(|c| {
            let mut r = vec![0xf0 | c.len() as u8];
            r.extend_from_slice(c);
            r.resize(REPORT_BYTES, 0);
            r
        });
        let mut reports: Vec<Vec<u8>> = idle.chain(data).collect();
        if let Some(header) = self.bad_report {
            let mut bad = vec![header];
            bad.resize(REPORT_BYTES, 0);
            let at = reports.len().min(self.idle as usize + 1);
            reports.insert(at, bad);
        }
        reports
    }

    fn line(&mut self, data: &[u8], v: &mut Violations) {
        if data.len() != FEATURE_BYTES {
            v.report(format!(
                "line setting of {} bytes ({}); ch9325_set_params sends {FEATURE_BYTES}",
                data.len(),
                crate::txlog::hex(data)
            ));
        }
        if data.len() < FEATURE_BYTES {
            return;
        }
        if data[2] != 0 || data[3] != 0 {
            v.report(format!("line bytes 2 and 3 are {:02x} {:02x}, not 0", data[2], data[3]));
        }
        if data[4] > 3 {
            v.report(format!("data bits code {}; 0 to 3 are 5 to 8 bits", data[4]));
        }
        self.line = Some((rate_of(u16::from_le_bytes([data[0], data[1]])), data[4] + 5));
    }

    /// An output report, with the count first.
    fn output(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        self.outs += 1;
        if self.stall_out == Some(self.outs) {
            return Reply::Stall;
        }
        let count = data.first().map_or(0, |c| usize::from(*c));
        if data.len() != REPORT_BYTES || !(1..=CHUNK).contains(&count) {
            v.report(format!(
                "output report {}; ch9325_write_bytes sends the count, 1 to 7, in 8 bytes",
                crate::txlog::hex(data)
            ));
            return Reply::ack();
        }
        if self.line_matches() {
            self.meter.commands(&data[1..=count], |m| v.report(m));
        }
        Reply::ack()
    }
}

impl HidReports for Chip {
    fn get_report(&mut self, kind: u8, id: u8, _length: usize, v: &mut Violations) -> Reply {
        v.report(format!("GET_REPORT of kind {kind}, report {id}; no path of the driver asks"));
        Reply::Stall
    }

    fn set_report(&mut self, kind: u8, id: u8, data: &[u8], v: &mut Violations) -> Reply {
        match kind {
            hid::FEATURE => {
                if self.stall_report {
                    return Reply::Stall;
                }
                if id != 0 {
                    v.report(format!("feature report id {id}; the line setting is report 0"));
                }
                self.line(data, v);
                Reply::ack()
            }
            // A host without the OUT endpoint's path: the same bytes.
            hid::OUTPUT => self.output(data, v),
            other => {
                v.report(format!("SET_REPORT of kind {other}"));
                Reply::Stall
            }
        }
    }
}

/// The emulated cable.
pub struct Cable {
    descriptors: Descriptors,
    hid: HidInterface,
    chip: Chip,
    mark: bool,
}

impl Cable {
    fn new(chip: Chip) -> Cable {
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
            device: 0x1200,
            strings: [1, 2, 0],
            configurations: 1,
        };
        Cable {
            descriptors: Descriptors {
                speed: Speed::Full,
                device: device.bytes(),
                configs: vec![descriptor::configuration(1, 4, 0x80, 100, &body)],
                strings: vec![
                    "WCH.CN \u{11}".to_owned(),
                    "USB to Serial".to_owned(),
                    String::new(),
                    String::new(),
                ],
            },
            hid,
            chip,
            mark: false,
        }
    }
}

impl DeviceModel for Cable {
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
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the chip answers",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != EP_OUT {
            v.report(format!("OUT transfer to endpoint {ep}; the chip has {EP_OUT}"));
            return Reply::Stall;
        }
        self.chip.output(data, v)
    }

    fn in_transfer(&mut self, _ep: u8, max: usize, v: &mut Violations) -> Reply {
        let chip = &mut self.chip;
        if chip.reports.is_empty()
            && let Some(record) = chip.meter.offer()
        {
            let reports = chip.reports_of(&record);
            chip.reports.push(reports);
        }
        if chip.reports.is_empty() && chip.meter.take_mark() {
            self.mark = true;
        }
        chip.reports.answer(max, v)
    }

    fn reset(&mut self) {
        self.hid.reset();
        let chip = &mut self.chip;
        chip.meter.power_on();
        chip.line = None;
        chip.reports = Reports::default();
        chip.outs = 0;
        self.mark = false;
    }

    fn mark(&mut self) -> bool {
        std::mem::take(&mut self.mark)
    }
}

#[cfg(test)]
mod tests {
    use super::super::uni_t_ut32x::{CMD_CLEAR, CMD_LIVE, CMD_STOP, RECORD_BYTES};
    use super::*;

    fn cable(o: &[(&str, &str)]) -> Cable {
        let mut chip = Chip::default();
        for (k, value) in o {
            match *k {
                "chunk" => chip.chunk = value.parse().unwrap(),
                "idle" => chip.idle = value.parse().unwrap(),
                "stall-report" => chip.stall_report = *value == "1",
                "stall-out" => chip.stall_out = Some(value.parse().unwrap()),
                "bad-report" => chip.bad_report = Some(value.parse().unwrap()),
                "baud" => chip.baud = value.parse().unwrap(),
                _ => assert!(chip.meter.option(k, value).unwrap(), "{k}"),
            }
        }
        Cable::new(chip)
    }

    fn feature(data: &[u8]) -> Setup {
        Setup {
            request_type: 0x21,
            request: 9,
            value: 0x0300,
            index: 0,
            length: data.len() as u16,
        }
    }

    const LINE: [u8; 5] = [0x60, 0x09, 0, 0, 3];

    fn set_line(dev: &mut Cable, v: &mut Violations, line: &[u8]) -> Reply {
        dev.control(feature(line), line, v)
    }

    fn write(dev: &mut Cable, v: &mut Violations, cmd: u8) -> Reply {
        dev.out(EP_OUT, &[1, cmd, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5], v)
    }

    fn reports(dev: &mut Cable, v: &mut Violations, offers: usize) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        for _ in 0..offers {
            if let Reply::Data(d) = dev.in_transfer(EP_IN, 8, v) {
                out.push(d);
            }
        }
        out
    }

    #[test]
    fn the_descriptors_are_the_wikis() {
        let d = cable(&[]);
        let c = &d.descriptors().configs[0];
        assert_eq!(c[2], 41, "wTotalLength");
        assert_eq!(&d.descriptors().device[2..4], &[0x00, 0x01], "bcdUSB 1.00");
        assert_eq!(&d.descriptors().device[12..14], &[0x00, 0x12], "bcdDevice 12.00");
        assert_eq!(d.descriptors().device[7], 8, "bMaxPacketSize0");
        assert!(c.windows(7).any(|w| w == [7, 5, 0x82, 3, 8, 0, 5]));
        assert!(c.windows(7).any(|w| w == [7, 5, 0x02, 3, 8, 0, 5]));
    }

    #[test]
    fn a_record_comes_in_reports_once_it_has_been_told() {
        let mut d = cable(&[("period", "1"), ("live", "41424344454647")]);
        let mut v = Violations::default();
        assert_eq!(set_line(&mut d, &mut v, &LINE), Reply::ack());
        assert!(reports(&mut d, &mut v, 5).is_empty(), "nothing unasked");
        assert_eq!(write(&mut d, &mut v, CMD_LIVE), Reply::ack());
        assert_eq!(
            reports(&mut d, &mut v, 4)[..4],
            [
                vec![0xf2, 0x41, 0x42, 0, 0, 0, 0, 0],
                vec![0xf2, 0x43, 0x44, 0, 0, 0, 0, 0],
                vec![0xf2, 0x45, 0x46, 0, 0, 0, 0, 0],
                vec![0xf1, 0x47, 0, 0, 0, 0, 0, 0],
            ]
        );
        write(&mut d, &mut v, CMD_STOP);
        assert_eq!(reports(&mut d, &mut v, 5), [] as [std::vec::Vec<u8>; 0]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_default_record_in_two_byte_reports() {
        let mut d = cable(&[("period", "1")]);
        let mut v = Violations::default();
        set_line(&mut d, &mut v, &LINE);
        write(&mut d, &mut v, CMD_LIVE);
        let got = reports(&mut d, &mut v, RECORD_BYTES.div_ceil(2));
        let bytes: Vec<u8> = got.iter().flat_map(|r| r[1..=usize::from(r[0] & 15)].to_vec()).collect();
        assert_eq!(bytes.len(), RECORD_BYTES);
        assert!(bytes.ends_with(b"\r\n"));
    }

    #[test]
    fn idle_reports_go_before_each_record() {
        let mut d = cable(&[("period", "1"), ("live", "41"), ("idle", "2")]);
        let mut v = Violations::default();
        set_line(&mut d, &mut v, &LINE);
        write(&mut d, &mut v, CMD_LIVE);
        let got = reports(&mut d, &mut v, 3);
        assert_eq!(got[0][0], 0xf0);
        assert_eq!(got[1][0], 0xf0);
        assert_eq!(got[2], [0xf1, 0x41, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn with_no_line_or_another_nothing_passes_and_an_unknown_rate_is_2400() {
        let mut v = Violations::default();
        let mut d = cable(&[("period", "1")]);
        write(&mut d, &mut v, CMD_LIVE);
        assert!(reports(&mut d, &mut v, 3).is_empty(), "no line yet");
        set_line(&mut d, &mut v, &[0x80, 0x25, 0, 0, 3]);
        write(&mut d, &mut v, CMD_LIVE);
        assert!(reports(&mut d, &mut v, 3).is_empty(), "9600");
        set_line(&mut d, &mut v, &[0x60, 0x09, 0, 0, 2]);
        write(&mut d, &mut v, CMD_LIVE);
        assert!(reports(&mut d, &mut v, 3).is_empty(), "7 bits");
        set_line(&mut d, &mut v, &[0xb0, 0x04, 0, 0, 3]);
        write(&mut d, &mut v, CMD_LIVE);
        assert!(!reports(&mut d, &mut v, 3).is_empty(), "1200 falls back to 2400");
        let mut fast = cable(&[("period", "1"), ("baud", "9600")]);
        set_line(&mut fast, &mut v, &[0x80, 0x25, 0, 0, 3]);
        write(&mut fast, &mut v, CMD_LIVE);
        assert!(!reports(&mut fast, &mut v, 3).is_empty(), "an instrument at 9600");
        assert!(v.is_empty(), "the user's serialcomm is no fault of the host: {v:?}");
    }

    #[test]
    fn the_checks_fire() {
        let mut d = cable(&[]);
        let mut v = Violations::default();
        set_line(&mut d, &mut v, &[0x60, 0x09, 0, 0, 3, 0]);
        assert_eq!(v.drain().len(), 1, "six bytes");
        set_line(&mut d, &mut v, &[0x60, 0x09]);
        assert_eq!(v.drain().len(), 1, "two bytes");
        set_line(&mut d, &mut v, &[0x60, 0x09, 1, 0, 3]);
        assert_eq!(v.drain().len(), 1, "byte 2");
        set_line(&mut d, &mut v, &[0x60, 0x09, 0, 0, 4]);
        assert_eq!(v.drain().len(), 1, "bits code");
        let id = Setup { value: 0x0301, ..feature(&LINE) };
        d.control(id, &LINE, &mut v);
        assert_eq!(v.drain().len(), 1, "report id 1");
        let other = Setup { value: 0x0100, ..feature(&LINE) };
        assert_eq!(d.control(other, &LINE, &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "input report by SET_REPORT");
        let get = Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 5 };
        assert_eq!(d.control(get, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "GET_REPORT");
        d.out(EP_OUT, &[1, CMD_LIVE], &mut v);
        assert_eq!(v.drain().len(), 1, "short");
        d.out(EP_OUT, &[0, 0, 0, 0, 0, 0, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "count 0");
        d.out(EP_OUT, &[8, 0, 0, 0, 0, 0, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "count 8");
        assert_eq!(d.out(1, &[1, 1, 0, 0, 0, 0, 0, 0], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "endpoint 1");
        set_line(&mut d, &mut v, &LINE);
        write(&mut d, &mut v, CMD_CLEAR);
        assert_eq!(v.drain().len(), 1, "the instrument's own");
        let out = Setup { request_type: 0x21, request: 9, value: 0x0201, index: 0, length: 8 };
        assert_eq!(d.control(out, &[1, CMD_STOP, 0, 0, 0, 0, 0, 0], &mut v), Reply::ack());
        assert!(v.is_empty(), "by SET_REPORT: {v:?}");
    }

    #[test]
    fn the_faults_do_what_they_say() {
        let mut v = Violations::default();
        let mut d = cable(&[("stall-report", "1")]);
        assert_eq!(set_line(&mut d, &mut v, &LINE), Reply::Stall);
        let mut d = cable(&[("stall-out", "2"), ("period", "1")]);
        set_line(&mut d, &mut v, &LINE);
        assert_eq!(write(&mut d, &mut v, CMD_LIVE), Reply::ack());
        assert_eq!(write(&mut d, &mut v, CMD_STOP), Reply::Stall);
        assert!(!reports(&mut d, &mut v, 2).is_empty(), "a stalled stop does not stop it");
        assert_eq!(write(&mut d, &mut v, CMD_STOP), Reply::ack(), "one-shot");
        let mut d = cable(&[("bad-report", "255"), ("period", "1"), ("live", "414243")]);
        set_line(&mut d, &mut v, &LINE);
        write(&mut d, &mut v, CMD_LIVE);
        let got = reports(&mut d, &mut v, 3);
        assert_eq!(got[0], [0xf2, 0x41, 0x42, 0, 0, 0, 0, 0]);
        assert_eq!(got[1], [0xff, 0, 0, 0, 0, 0, 0, 0], "behind the record's first");
        assert_eq!(got[2], [0xf1, 0x43, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn the_event_point_comes_when_the_burst_has_gone() {
        let mut d = cable(&[("period", "2"), ("live", "41"), ("burst", "1")]);
        let mut v = Violations::default();
        set_line(&mut d, &mut v, &LINE);
        write(&mut d, &mut v, CMD_LIVE);
        assert_eq!(reports(&mut d, &mut v, 2).len(), 1);
        assert!(!d.mark());
        reports(&mut d, &mut v, 2);
        assert!(d.mark());
        assert!(!d.mark(), "once");
    }

    #[test]
    fn a_reset_is_power_on() {
        let mut d = cable(&[("period", "1")]);
        let mut v = Violations::default();
        set_line(&mut d, &mut v, &LINE);
        write(&mut d, &mut v, CMD_LIVE);
        assert_ne!(reports(&mut d, &mut v, 1), [] as [std::vec::Vec<u8>; 0]);
        d.reset();
        assert_eq!(reports(&mut d, &mut v, 3), [] as [std::vec::Vec<u8>; 0]);
        write(&mut d, &mut v, CMD_LIVE);
        assert!(reports(&mut d, &mut v, 3).is_empty(), "the line is gone");
    }

    #[test]
    fn options_are_checked() {
        let o = |k: &str, v: &str| vec![(k.to_owned(), v.to_owned())];
        assert!(build(&o("nothing", "1")).is_err());
        assert!(build(&o("chunk", "8")).is_err());
        assert!(build(&o("chunk", "0")).is_err());
        assert!(build(&o("bits", "9")).is_err());
        assert!(build(&o("stall-out", "0")).is_err());
        assert!(build(&o("stall-report", "2")).is_err());
        assert!(build(&o("bad-report", "0000")).is_err());
        assert!(build(&o("bad-report", "00")).is_ok());
        assert!(build(&o("period", "3")).is_ok());
    }
}
