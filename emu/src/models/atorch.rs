// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An Atorch meter or load on the CDC-ACM shell: a report every so often,
//! unasked, framed by a magic and a type, with nothing to ask for.
//!
//! The device, and how sure this is:
//!
//! - The link: the units the driver was tested on (a J7-C USB meter and a
//!   DL24MP load, says the driver's commit message) speak Bluetooth; so do
//!   those of `esphome-atorch-dl24`. So this USB function, its ids and its
//!   strings are an emulation artifact, a stand-in for an RFCOMM tty, and the
//!   line settings do nothing, as there.
//! - A message: `ff 55`, a type byte, then a body fixed by the type: 36 bytes
//!   in all for a report (`01`), 8 for a reply (`02`), 10 for a command (`11`).
//!   That the last byte of each is the sum of those from the type on, xor
//!   `44`, comes from `esphome-atorch-dl24` (`atorch_dl24.cpp`, `crc`) and
//!   holds for every recording below. Its README says the J7-C, DL24M and
//!   DT24PW need that check turned off, so a host must not rely on it.
//! - A report's fourth byte is the unit's type: `01` AC, `02` DC, `03` USB.
//!   The fields are the driver's, big endian. Default reports are recordings
//!   quoted in `atorch_dl24.cpp`, which come from `NiceLabs/atorch-console`'s
//!   spec files: real bytes, not made up.
//! - The rate: its README's log has one report a second. Here `period=` counts
//!   offers of a read, about 20 ms each, so a report per half a second.
//! - What it takes: commands exist on a real unit (`11`, with a reply `02`), as
//!   those two sources show. No driver path this model serves sends one, so
//!   anything written is flagged and has no effect.
//!
//! No firmware source was found; the sources are those two programs and the
//! driver. Nothing here was checked on a unit.
//!
//! Emulation artifacts, besides the link: a stream starts over at each open,
//! which a real unit does not know of (it goes by [`LineEvent::opened`]), so a
//! tool that closes within a period of the report it needed has caused just
//! that. And since a tool's `.tx` would otherwise hold what time let through,
//! `scan=N` and `burst=N` end the stream after a count: with them the bytes of a
//! case do not depend on the clock. What's left is a margin: a report that lands
//! inside an open is lost to its flush, and offers come fast at an open, with
//! the kernel's 16 reads to fill. A `period` of 5 lost them, so the cases use 25.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `meter=dc|usb|ac`, the recordings to play;
//! - `packet=HEX[,HEX...]`, what to play in turn instead, each sent whole, so
//!   one may hold several messages or any bytes (1 to 1024 bytes);
//! - `noise=HEX`, bytes sent before each packet;
//! - `period=N`, offers per packet (2 to 100000, default 25);
//! - `scan=N`, packets up to the first close after the first packet's gone (by
//!   default as `burst=`);
//! - `burst=N`, packets per later open, then silence, with the event point
//!   marked a period after the last. With neither, it sends for ever.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, LineEvent, Port, Serial};
use crate::shells::hidcable::{self as cable, Packets};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "atorch";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "An Atorch meter or load, which reports unasked; meter=dc|usb|ac, \
                         packet=HEX[,HEX...], noise=HEX, period=N, scan=N, burst=N, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// Recordings of a DL24 (`packet-meter-dc.spec.ts`).
const DC: [&str; 2] = [
    "ff55010200011a00003c0004d40000002000006400000000002600590d363c00000000f0",
    "ff55010200011a0000000004d40000002000006400000000002a00590e343c000000003f",
];
/// Recordings of a USB meter (`packet-meter-usb.spec.ts`).
const USB: [&str; 4] = [
    "ff5501030001f3000000000638000003110007000a000000122e333c000000000000004e",
    "ff5501030001fb000000003cc70000554e00070007000000472f243c00000000000000ce",
    "ff5501030001cd00007f003cc80000554e0009000a00000047300d3c000000000000008f",
    "ff5501030001fb000001006c3f00006c4400070006001a00471c1a3c0000000000000078",
];
/// Recordings of an AC meter (`packet-meter-ac.spec.ts`), a type the driver
/// does not know.
const AC: [&str; 3] = [
    "ff55010100090400000e0000040000000000006401f40085002f00000a093c0000000039",
    "ff5501010008eb000000000000000001fe00006401f40000002f003125143c0000000066",
    "ff5501010008ff0000270000210000000000006401f401740031000038083c0000000088",
];

/// A meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut recordings: &[&str] = &DC;
    let mut packets = None;
    let mut noise = Vec::new();
    let mut period = 25;
    let mut scan = None;
    let mut burst = None;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "meter" => {
                recordings = match v.as_str() {
                    "dc" => &DC,
                    "usb" => &USB,
                    "ac" => &AC,
                    _ => return Err(Error::Invalid(format!("meter={v}: dc, usb or ac"))),
                };
            }
            "packet" => packets = Some(cable::parse_packets(v, 1, 1024)?),
            "noise" => noise = cable::parse_packets(v, 1, 1024)?.remove(0),
            "period" => period = cable::parse_number(k, v, 2..=100_000)?,
            "scan" => scan = Some(cable::parse_number(k, v, 0..=1000)?),
            "burst" => burst = cable::parse_burst(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let list = match packets {
        Some(list) => list,
        None => recordings
            .iter()
            .map(|hex| Ok(cable::parse_packets(hex, 36, 36)?.remove(0)))
            .collect::<Result<_>>()?,
    };
    let meter = Meter {
        packets: Packets::new(list.clone()),
        list,
        noise,
        period,
        scan: scan.or(burst),
        burst,
        offers: 0,
        left: scan.or(burst),
        played: false,
        later: false,
        quiet: 0,
    };
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Atorch link");
    Ok(Cdc::new(NAME, &id, faults, meter))
}

struct Meter {
    list: Vec<Vec<u8>>,
    packets: Packets,
    noise: Vec<u8>,
    period: u32,
    /// Packets up to the first close after the first has gone.
    scan: Option<u32>,
    /// Packets per later open.
    burst: Option<u32>,
    /// Offers since the last packet, or the open.
    offers: u32,
    /// What this open may still send.
    left: Option<u32>,
    /// A packet has gone since power on, so the next open is a later one.
    played: bool,
    /// This open is a later one.
    later: bool,
    /// Offers since the last packet of a stream that ended.
    quiet: u32,
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        // A real unit takes commands; nothing the driver does sends one.
        port.violation(format!("{} bytes written to a meter the driver only reads", data.len()));
    }

    fn changed(&mut self, event: LineEvent, _port: &mut Port<'_>) {
        if event.opened() {
            self.offers = 0;
            self.quiet = 0;
            self.later = self.played;
            self.left = if self.later { self.burst } else { self.scan };
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if self.left == Some(0) {
            // A period after the last packet, so that the tool has read it
            // before an event lands. The scan's open has no mark unless it has
            // no packet at all: a tool still scanning a period on is no event's.
            if (self.later || self.scan == Some(0)) && self.quiet < self.period {
                self.quiet += 1;
                if self.quiet == self.period {
                    port.mark();
                }
            }
            return;
        }
        self.offers += 1;
        if self.offers < self.period {
            return;
        }
        self.offers = 0;
        port.send(&self.noise);
        port.send(&self.packets.take());
        self.played = true;
        if let Some(left) = &mut self.left {
            *left -= 1;
        }
    }

    fn reset(&mut self) {
        self.packets = Packets::new(self.list.clone());
        self.offers = 0;
        self.quiet = 0;
        self.left = self.scan;
        self.played = false;
        self.later = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::Line;
    use crate::shells::cdc::host::Host;

    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::default(), true, true)
    }

    fn sum_ok(message: &[u8]) -> bool {
        let sum = message[2..message.len() - 1].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        sum ^ 0x44 == message[message.len() - 1]
    }

    /// `period` offers: the last holds what they gave.
    fn offers(h: &mut Host<Meter>, period: u32) -> Reply {
        for _ in 1..period {
            assert_eq!(h.offer(), Reply::Nak, "before its time");
        }
        h.offer()
    }

    #[test]
    fn the_recordings_have_the_form_and_the_sum_they_should() {
        for (list, kind) in [(&DC[..], 2), (&USB[..], 3), (&AC[..], 1)] {
            for hex in list {
                let m = cable::parse_packets(hex, 36, 36).unwrap().remove(0);
                assert_eq!(m[..3], [0xff, 0x55, 0x01]);
                assert_eq!(m[3], kind);
                assert!(sum_ok(&m), "{hex}");
            }
        }
    }

    #[test]
    fn the_stream_plays_in_turn_on_its_period() {
        let mut h = meter(&[("period", "3"), ("meter", "usb")]);
        for hex in [USB[0], USB[1], USB[2], USB[3], USB[0]] {
            let want = cable::parse_packets(hex, 36, 36).unwrap().remove(0);
            assert_eq!(offers(&mut h, 3), Reply::Data(want));
        }
        assert!(h.v.is_empty());
    }

    #[test]
    fn each_open_starts_the_period_over_and_a_packet_is_sent_whole() {
        let mut h = meter(&[("period", "3"), ("packet", "0102,03"), ("noise", "aa")]);
        h.offer();
        h.offer();
        h.set_lines(true, true);
        assert_eq!(offers(&mut h, 3), Reply::Data(vec![0xaa, 1, 2]));
        assert_eq!(offers(&mut h, 3), Reply::Data(vec![0xaa, 3]));
    }

    #[test]
    fn a_scan_and_a_burst_end_the_stream_and_only_a_later_open_marks() {
        let mut h = meter(&[("period", "2"), ("scan", "1"), ("burst", "2")]);
        // An open before any packet is still the scan's: oxrok opens twice.
        h.set_lines(true, true);
        assert!(matches!(offers(&mut h, 2), Reply::Data(_)));
        for _ in 0..10 {
            assert_eq!(h.offer(), Reply::Nak, "the scan's is over");
        }
        assert!(!h.cdc.mark());
        h.set_lines(true, true);
        assert!(matches!(offers(&mut h, 2), Reply::Data(_)));
        assert!(matches!(offers(&mut h, 2), Reply::Data(_)));
        assert_eq!(h.offer(), Reply::Nak);
        assert!(!h.cdc.mark(), "a period later");
        assert_eq!(h.offer(), Reply::Nak);
        assert!(h.cdc.mark());
        for _ in 0..10 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert!(!h.cdc.mark(), "once");
    }

    #[test]
    fn a_meter_that_never_sends_marks_a_period_on() {
        let mut h = meter(&[("period", "2"), ("scan", "0")]);
        assert_eq!(h.offer(), Reply::Nak);
        assert!(!h.cdc.mark());
        assert_eq!(h.offer(), Reply::Nak);
        assert!(h.cdc.mark());
    }

    #[test]
    fn the_burst_alone_holds_for_the_first_open_too() {
        let mut h = meter(&[("period", "2"), ("burst", "1")]);
        assert!(matches!(offers(&mut h, 2), Reply::Data(_)));
        for _ in 0..4 {
            assert_eq!(h.offer(), Reply::Nak);
        }
    }

    #[test]
    fn power_on_starts_the_list_and_the_scan_over() {
        let mut h = meter(&[("period", "2"), ("scan", "1"), ("burst", "0")]);
        let first = offers(&mut h, 2);
        assert!(matches!(first, Reply::Data(_)));
        h.cdc.model.reset();
        h.set_lines(true, true);
        assert_eq!(offers(&mut h, 2), first);
    }

    #[test]
    fn anything_written_is_flagged_and_changes_nothing() {
        let mut h = meter(&[("period", "2")]);
        h.write(&[0xff, 0x55, 0x11, 3, 1, 0, 0, 0, 0, 0x51]);
        assert_eq!(h.v.drain().len(), 1);
        assert_eq!(h.read(), b"", "no reply");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("meter", "x"),
            ("packet", "0"),
            ("period", "1"),
            ("scan", "1001"),
            ("burst", "x"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("stall", "line"), ("noise", "00ff"), ("meter", "ac")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
