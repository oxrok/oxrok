// Counterpart in sigrok:
//   libsigrok/src/serial_hid_victor.c,
//   Copyright (C) 2019 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! The cable of the Victor 70C and 86C meters, as the Pico emulator
//! plays it (`hil/pico-hidcable`, `MODE victor`): a HID device
//! `1244:d237`, vendor `Victor`, with 14 byte input and output reports
//! and no report ids, which forwards the meter's FS9922 packets
//! (14 bytes) one per input report, scrambled:
//!
//! - the bits of every byte reversed,
//! - the bytes shuffled, wire byte `i` coming from packet byte
//!   `13 - shuffle[i]`,
//! - the magic `jodenxunickxia` added byte by byte.
//!
//! `victor_unobfuscate` undoes it. The cable is read only and has no
//! line parameters (`victor_write_bytes` writes nothing, `set_params`
//! is the dummy), so the host sends nothing at all: the meter streams
//! from the start, a packet every `period` polls of the interrupt IN
//! endpoint rather than every `PERIOD` ms as on the Pico (see
//! `shells::hidcable::Stream`), so both tools see the same packets.
//!
//! Options: `packet=HEX[,HEX...]`, the meter's packets in turn, 14
//! bytes each (default the `diffcli.sh --hw` packet, `+1234 10` with
//! the DC and volt flags); `period=N`, polls per packet (4 to 1000,
//! default 50, about a second). Nothing restarts the period here, so
//! it must outlast the windows in which one tool holds the node open
//! and the other does not: the 50 ms after an open in which the kernel
//! drops input, and the about 100 ms in which sigrok's open checks the
//! path before it opens the node for good, where the held transfer is
//! offered some 10 times.
//!
//! Option `burst=N`: the meter sends N packets in all (from the
//! model's start, which nothing restarts here) and then falls silent;
//! `burst=0` is a meter that never sends, whose scan gives up after
//! its 3 s. The default sends for as long as the host polls.
//!
//! The pedantic checks: the host sends no report of any kind and reads
//! none by `GET_REPORT`.

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Violations};
use crate::shells::hidcable as cable;
use crate::shells::hid::{HidInterface, HidReports};
use crate::{Error, Result};
use crate::shells::hidcable::{Packets, Reports, Stream};

/// The model name.
pub const NAME: &str = "victor";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Victor 70C/86C meter cable (HID, 1244:d237, scrambled FS9922), the Pico \
                         emulator's role; packet=HEX[,HEX...] (14 bytes each), period=N (polls \
                         per packet, default 50), burst=N (packets in all, then silence)";

/// The USB vendor id.
pub const VENDOR_ID: u16 = 0x1244;
/// The USB product id.
pub const PRODUCT_ID: u16 = 0xd237;
/// The report length, which is the packet length.
pub const REPORT_BYTES: usize = 14;
/// An FS9922 packet as the `--hw` case plays it.
pub const DEFAULT_PACKET: &str = "2b31323334203130000080000d0a";

const OBFUSCATION: [u8; REPORT_BYTES] = *b"jodenxunickxia";
const SHUFFLE: [usize; REPORT_BYTES] = [6, 13, 5, 11, 2, 7, 9, 8, 3, 10, 12, 0, 4, 1];

/// A cable from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut packets = cable::parse_packets(DEFAULT_PACKET, REPORT_BYTES, REPORT_BYTES)?;
    let mut period = 50;
    let mut burst = None;
    for (k, v) in options {
        match k.as_str() {
            "packet" => packets = cable::parse_packets(v, REPORT_BYTES, REPORT_BYTES)?,
            "period" => period = cable::parse_number(k, v, 4..=1000)?,
            "burst" => burst = cable::parse_burst(v)?,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    let mut cable = Victor::new(packets, period);
    cable.meter.stream = Stream::new(period, true).with_burst(burst);
    Ok(Box::new(cable))
}

/// A packet as the cable puts it on the wire.
pub fn scramble(plain: &[u8]) -> Vec<u8> {
    (0..REPORT_BYTES)
        .map(|i| plain[REPORT_BYTES - 1 - SHUFFLE[i]].reverse_bits().wrapping_add(OBFUSCATION[i]))
        .collect()
}

/// The meter behind the cable.
#[derive(Clone, Debug)]
pub struct Meter {
    packets: Packets,
    reports: Reports,
    stream: Stream,
}

impl HidReports for Meter {
    fn get_report(&mut self, kind: u8, id: u8, _length: usize, v: &mut Violations) -> Reply {
        v.report(format!("GET_REPORT of kind {kind} id {id}; the cable is read on endpoint 1"));
        Reply::Stall
    }

    fn set_report(&mut self, kind: u8, id: u8, data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!(
            "SET_REPORT of kind {kind} id {id} ({}); the cable is read only",
            crate::txlog::hex(data)
        ));
        Reply::ack()
    }
}

/// The emulated cable.
pub struct Victor {
    descriptors: Descriptors,
    hid: HidInterface,
    meter: Meter,
}

impl Victor {
    /// A cable whose meter sends `packets` in turn, one per `period`
    /// polls.
    pub fn new(packets: Vec<Vec<u8>>, period: u32) -> Victor {
        let hid = HidInterface::new(0, cable::report_descriptor(14, 14));
        Victor {
            descriptors: cable::descriptors(
                VENDOR_ID,
                PRODUCT_ID,
                "Victor",
                "Victor cable emulator",
                &hid,
            ),
            hid,
            meter: Meter {
                packets: Packets::new(packets),
                reports: Reports::default(),
                stream: Stream::new(period, true),
            },
        }
    }
}

impl DeviceModel for Victor {
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
        let m = &mut self.meter;
        if m.reports.is_empty() && m.stream.poll() {
            let p = m.packets.take();
            m.reports.push([scramble(&p)]);
        }
        m.reports.answer(max, v)
    }

    fn reset(&mut self) {
        self.hid.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `victor_unobfuscate`, for the round trip.
    fn unscramble(wire: &[u8]) -> Vec<u8> {
        let mut out = vec![0u8; REPORT_BYTES];
        for (i, b) in wire.iter().enumerate() {
            out[REPORT_BYTES - 1 - SHUFFLE[i]] = b.wrapping_sub(OBFUSCATION[i]).reverse_bits();
        }
        out
    }

    #[test]
    fn streams_scrambled_packets() {
        let plain = cable::parse_packets(DEFAULT_PACKET, 14, 14).unwrap();
        let mut dev = Victor::new(plain.clone(), 4);
        let mut v = Violations::default();
        for round in 0..2 {
            for _ in 0..3 {
                assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak, "round {round}");
            }
            match dev.in_transfer(1, 64, &mut v) {
                Reply::Data(r) => assert_eq!(unscramble(&r), plain[0]),
                other => panic!("{other:?}"),
            }
        }
        assert!(v.is_empty());
        // The Pico's scrambling of the default packet, byte for byte.
        assert_eq!(scramble(&plain[0])[..2], [0x0c + 0x6a, 0x43]);
        let rd = Setup { request_type: 0x81, request: 6, value: 0x2200, index: 0, length: 255 };
        assert_eq!(dev.control(rd, &[], &mut v), Reply::Data(cable::report_descriptor(14, 14)));
    }

    #[test]
    fn flags_any_report_from_the_host() {
        let mut dev = Victor::new(vec![vec![0; 14]], 4);
        let mut v = Violations::default();
        let out = Setup { request_type: 0x21, request: 9, value: 0x0200, index: 0, length: 2 };
        assert_eq!(dev.control(out, &[1, 2], &mut v), Reply::ack());
        assert_eq!(v.drain().len(), 1);
        let feature = Setup { request_type: 0x21, request: 9, value: 0x0300, index: 0, length: 2 };
        dev.control(feature, &[1, 2], &mut v);
        assert_eq!(v.drain().len(), 1);
        let get = Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 8 };
        assert_eq!(dev.control(get, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1);
        assert!(build(&[("packet".into(), "00".into())]).is_err(), "14 bytes");
        assert!(build(&[("period".into(), "3".into())]).is_err());
    }
}
