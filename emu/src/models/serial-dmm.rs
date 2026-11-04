// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A serial multimeter on the CDC-ACM shell, the shell's pilot: one meter
//! that must be asked for each packet and two that send on their own.
//!
//! - `meter=metex` (the default), for `metex-m3640d`: at 1200/7n2, one
//!   byte `D` asks for a 14 byte packet (`dmm/metex14.c`). The packets
//!   are ASCII: `DC  1.234 V  \r`.
//! - `meter=es51922`, for `uni-t-ut61e-ser`, at 19200/7o1, and
//!   `meter=fs9721`, for `uni-t-ut60a-ser`, at 2400/8n1: the meter
//!   streams 14 byte packets, a host that writes to it writes to nothing.
//!   The default packets are those `tools/diffcli.sh --hw` plays through
//!   the ESP32's `EMIT`.
//!
//! No such meter has a USB interface of its own: they sit behind a UART
//! cable (a UT-D02 and the like), usually an FTDI or a Prolific chip that
//! makes a `ttyUSBn`. This model gives them a CDC-ACM cable, which shows
//! the same line settings and handshake lines to the tools. On a misset
//! line nothing gets through, in either direction
//! ([`Cdc::behind_uart`]), which is what a scan with the wrong
//! `serialcomm=` finds on the real cable: no packet it can use.
//!
//! The asking meter counts requests, so a run's bytes are the driver's
//! own. A streaming one sends a packet per `period` offers of a read
//! (about 20 ms each), counted from each open ([`LineEvent::opened`]), so a tool
//! that closes within a period of the last packet it needed has caused
//! exactly those; see [`crate::shells::hidcable::Stream`].
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `meter=metex|es51922|fs9721`;
//! - `packet=HEX[,HEX...]`, the packets played in turn, 14 bytes each;
//! - `noise=HEX`, bytes the meter sends before each packet;
//! - `flood=N`, 256 bytes of noise before each packet after the first N,
//!   past what a scan takes (128) and up to the acquisition's buffer;
//! - `period=N`, offers per packet of a streaming meter (4 to 1000,
//!   default 25);
//! - `burst=N`, packets per open of a streaming meter, then silence, and
//!   the event point marked a period later; 0 never sends;
//! - `answers=N`, requests the asking meter answers per power on; it
//!   stays silent to the next and marks the event point.
//!
//! The pedantic checks: the asking meter takes `D` and nothing else, with
//! DTR up and RTS down as the drivers' `serialcomm` has them (the cable
//! takes its power there); a streaming meter takes nothing. None of this
//! has been checked on such a meter or a data sheet: the `rts=0/dtr=1`
//! comes from `api.c`'s table.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::{self as cable, Packets, Stream};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "serial-dmm";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A serial multimeter on a CDC-ACM cable, the shell's pilot; \
                         meter=metex|es51922|fs9721, packet=HEX[,HEX...], noise=HEX, period=N, \
                         burst=N, answers=N, flood=N, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

const PACKET_BYTES: usize = 14;

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
    let mut kind = Kind::Metex;
    let mut packets = None;
    let mut noise = Vec::new();
    let mut period = 25;
    let mut burst = None;
    let mut answers = None;
    let mut flood = None;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "meter" => {
                kind = match v.as_str() {
                    "metex" => Kind::Metex,
                    "es51922" => Kind::Es51922,
                    "fs9721" => Kind::Fs9721,
                    _ => return Err(Error(format!("meter={v}: metex, es51922 or fs9721"))),
                };
            }
            "packet" => packets = Some(cable::parse_packets(v, PACKET_BYTES, PACKET_BYTES)?),
            "noise" => noise = cable::parse_packets(v, 1, 256)?.remove(0),
            "period" => period = cable::parse_number(k, v, 4..=1000)?,
            "burst" => burst = cable::parse_burst(v)?,
            "answers" => answers = Some(cable::parse_number(k, v, 0..=1000)?),
            "flood" => flood = Some(cable::parse_number(k, v, 0..=1000)?),
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    let packets = match packets {
        Some(p) => p,
        None => vec![kind.default_packet().to_vec()],
    };
    let meter = Meter {
        kind,
        packets: Packets::new(packets),
        noise,
        stream: Stream::new(period, false).with_burst(burst),
        burst_left: burst,
        period,
        quiet: 0,
        burst,
        answers,
        answered: 0,
        flood,
        played: 0,
    };
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Serial DMM cable");
    let line = Line::parse(kind.line()).expect("the table's lines parse");
    Ok(Cdc::new(NAME, &id, faults, meter).behind_uart(line))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Metex,
    Es51922,
    Fs9721,
}

impl Kind {
    /// The line `api.c`'s table sets for the driver this meter serves.
    fn line(self) -> &'static str {
        match self {
            Kind::Metex => "1200/7n2",
            Kind::Es51922 => "19200/7o1",
            Kind::Fs9721 => "2400/8n1",
        }
    }

    fn default_packet(self) -> &'static [u8; PACKET_BYTES] {
        match self {
            Kind::Metex => b"DC  1.234 V  \r",
            // tools/diffcli.sh --hw: "serial dmm ut61e".
            Kind::Es51922 => b"\x31\x31\x32\x33\x34\x35\x3b\x30\x30\x30\x3a\x30\x0d\x0a",
            // tools/diffcli.sh --hw: "serial dmm fs9721".
            Kind::Fs9721 => b"\x17\x20\x35\x4d\x5b\x61\x7f\x82\x97\xa0\xb0\xc0\xd4\xe0",
        }
    }
}

struct Meter {
    kind: Kind,
    packets: Packets,
    noise: Vec<u8>,
    stream: Stream,
    burst: Option<u32>,
    burst_left: Option<u32>,
    period: u32,
    quiet: u32,
    answers: Option<u32>,
    answered: u32,
    flood: Option<u32>,
    played: u32,
}

impl Meter {
    fn send_packet(&mut self, port: &mut Port<'_>) {
        if self.flood.is_some_and(|n| self.played >= n) {
            port.send(&[0x3f; 256]);
        }
        self.played = self.played.saturating_add(1);
        port.send(&self.noise);
        port.send(&self.packets.take());
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        if self.kind != Kind::Metex {
            port.violation(format!("{} bytes written to a meter that only sends", data.len()));
            return;
        }
        for &byte in data {
            if byte != b'D' {
                port.violation(format!("{byte:#04x} written; the meter is asked with 'D'"));
                continue;
            }
            if !port.dtr() || port.rts() {
                port.violation(format!(
                    "asked with DTR {} and RTS {}; serialcomm has dtr=1, rts=0",
                    u8::from(port.dtr()),
                    u8::from(port.rts())
                ));
            }
            if self.answers.is_some_and(|n| self.answered >= n) {
                port.mark_event_point();
                continue;
            }
            self.answered += 1;
            self.send_packet(port);
        }
    }

    fn changed(&mut self, event: LineEvent, _port: &mut Port<'_>) {
        if event.opened() && self.kind != Kind::Metex {
            self.stream.restart();
            self.burst_left = self.burst;
            self.quiet = 0;
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if self.kind == Kind::Metex {
            return;
        }
        if self.stream.poll() {
            self.send_packet(port);
            if let Some(left) = &mut self.burst_left {
                *left = left.saturating_sub(1);
            }
        } else if self.burst_left == Some(0) {
            // A period after the last packet, so that the tool has read it
            // before an event lands: bytes just delivered are lost to an
            // unplug's hangup, or not, as the pace has it.
            self.quiet += 1;
            if self.quiet >= self.period {
                port.mark_event_point();
                self.burst_left = None;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    const PACKET: &[u8] = b"DC  1.234 V  \r";

    /// The meter of `options` with a tool's port open on its line.
    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let cdc = function(&o).unwrap();
        let line = Line::parse(cdc.model.kind.line()).unwrap();
        Host::open(cdc, line, true, false)
    }

    #[test]
    fn the_metex_answers_each_request() {
        let mut h = meter(&[]);
        h.write(b"D");
        assert_eq!(h.read(), PACKET);
        h.write(b"DD");
        assert_eq!(h.read().len(), 2 * PACKET.len(), "per byte, however cut");
        assert_eq!(h.read(), b"", "nothing unasked");
        assert!(h.v.is_empty(), "{:?}", h.v);
    }

    #[test]
    fn the_metex_flags_what_it_is_not_asked() {
        let mut h = meter(&[]);
        h.write(b"X");
        assert_eq!(h.v.drain().len(), 1);
        assert_eq!(h.read(), b"");
        h.set_lines(false, false);
        h.write(b"D");
        assert_eq!(h.v.drain().len(), 1, "DTR down");
        h.set_lines(true, true);
        h.write(b"D");
        assert_eq!(h.v.drain().len(), 1, "RTS up");
    }

    #[test]
    fn a_misset_line_gets_no_answer_and_no_violation() {
        let mut h = meter(&[]);
        h.set_line(Line::parse("9600/7n2").unwrap());
        h.write(b"D");
        assert_eq!(h.read(), b"");
        assert!(h.v.is_empty());
    }

    #[test]
    fn a_flood_comes_after_the_first_packets() {
        let mut h = meter(&[("flood", "1")]);
        h.write(b"D");
        assert_eq!(h.read(), PACKET);
        h.write(b"D");
        assert_eq!(h.read().len(), 256 + PACKET.len());
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = meter(&[("answers", "1")]);
        h.write(b"D");
        assert_eq!(h.read(), PACKET);
        assert!(!h.cdc.take_event_point());
        h.write(b"D");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.take_event_point());
    }

    #[test]
    fn a_stream_counts_offers_from_each_open() {
        let mut h = meter(&[("meter", "es51922"), ("period", "4")]);
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert_eq!(h.offer(), Reply::Data(Kind::Es51922.default_packet().to_vec()));
        for _ in 0..2 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        // An open, whatever the lines were: the count starts over.
        h.set_lines(true, true);
        h.set_lines(true, false);
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert!(matches!(h.offer(), Reply::Data(_)));
        assert!(h.v.is_empty());
    }

    #[test]
    fn a_stream_takes_nothing_from_the_host() {
        let mut h = meter(&[("meter", "fs9721")]);
        h.write(b"D");
        assert_eq!(h.v.drain().len(), 1);
    }

    #[test]
    fn a_burst_ends_in_silence_and_the_point() {
        let mut h = meter(&[("meter", "fs9721"), ("period", "4"), ("burst", "1")]);
        for _ in 0..3 {
            h.offer();
        }
        assert!(matches!(h.offer(), Reply::Data(_)));
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
            assert!(!h.cdc.take_event_point(), "a period later");
        }
        assert_eq!(h.offer(), Reply::Nak);
        assert!(h.cdc.take_event_point());
        for _ in 0..10 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert!(!h.cdc.take_event_point(), "once");
    }

    #[test]
    fn options_parse() {
        for bad in [("meter", "x"), ("packet", "00"), ("period", "3"), ("nothing", "1")] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("stall", "line"), ("noise", "00ff"), ("answers", "0")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
