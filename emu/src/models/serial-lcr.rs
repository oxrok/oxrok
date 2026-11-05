// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! LCR meters on a serial cable, the two packet formats sigrok's `serial-lcr`
//! knows, on the CDC-ACM shell.
//!
//! - `meter=es51919` (the default), for `deree-de5000`, `mastech-ms5308`,
//!   `peaktech-2170` and `uni-t-ut612`: a Cyrustek ES51919 chipset at
//!   9600/8n1 streams binary packets of 17 bytes, and a host that writes to
//!   it writes to nothing. Header `00 0d`, footer `0d 0a`.
//! - `meter=vc4080`, for `peaktech-2165` and `voltcraft-4080`: the meter
//!   answers a one byte `N` with 39 ASCII characters ending in CR LF.
//!
//! Sources, and how sure they are:
//! - `src/lcr/es51919.c` and `src/lcr/vc4080.c` in libsigrok for both layouts,
//!   which are reverse engineered by their authors. The sample lines in the
//!   latter are from a `PeakTech` 2165.
//! - sigrok's wiki (Voltcraft 4080) and the C's comment: the meter's line is
//!   1200/7e1, its cable an FT232R with which a host at 7e1 got nothing, and
//!   8n1 gets each character with its parity bit in bit 7. Hence
//!   `parity=` here. The protocol documents of Conrad and `PeakTech`, which the
//!   C names, were not reachable, so nothing here comes from them.
//! - `github.com/4x1md/de5000_lcr_py` agrees on 9600/8n1 and 17 bytes, but
//!   says it took them from sigrok.
//! - No firmware source, no recording, no unit on a desk. The default
//!   packets are built from the layouts.
//!
//! Emulation artifacts:
//! - The cables are FT232R and the like; here they are CDC-ACM, which shows
//!   the same line settings and handshake lines. On a misset line nothing
//!   passes ([`Cdc::behind_uart`]), where a real one would give garbage.
//! - A packet goes out whole, however slow the line; the tests of the port
//!   hold a byte-paced fake for that. `chunk=` cuts them.
//! - The streaming meter's rate is a count of read offers, [`Stream`], from
//!   each open; nobody has said what a real ES51919 meter's is.
//! - `burst=` and `scan-burst=` are silences no meter is known to have: they
//!   make a capture's packets independent of how long a tool listens.
//! - The vc4080's sequence counter at byte 16 stands still, unless `packet=`
//!   moves it: no driver reads it.
//! - Of the meter's other commands (`A`, `B`, `[...]` setup, see the wiki) the
//!   model has none, and flags them: no C path sends them. A real meter has
//!   them, so a tool that means them should not read this as a refusal.
//!
//! Options, besides the shell's ([`Faults`]):
//! - `meter=es51919|vc4080`;
//! - `packet=HEX[,HEX...]`, the packets played in turn, of the meter's size;
//! - `noise=HEX`, bytes sent before each packet;
//! - `parity=0|1`, the vc4080's parity bits, on by default;
//! - `uart=LINE|native`, the line behind the cable, instead of the meter's;
//! - `period=N`, offers per packet of a streaming meter (4 to 1000, default
//!   25);
//! - `burst=N`, packets per open of a streaming meter, then silence and a mark
//!   a period later; 0 never sends;
//! - `scan-burst=N`, the same for the opens up to the Nth packet since power
//!   on, which are a scan's, with no mark. Counted in packets, as a tool's
//!   open can raise the lines more than once (oxrok's does);
//! - `answers=N`, requests the vc4080 answers per power on; it stays silent
//!   to the next and marks the event point.
//!
//! The pedantic checks: the vc4080 takes `N` alone, and the ES51919 nothing.
//! `rts=1/dtr=1` in the ES51919's `serialcomm` is not checked: no source says
//! what the meter does without them.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::{self as cable, Packets, Stream};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "serial-lcr";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "LCR meters on a CDC-ACM cable; meter=es51919|vc4080, \
                         packet=HEX[,HEX...], noise=HEX, parity=0|1, uart=LINE|native, period=N, \
                         burst=N, scan-burst=N, answers=N, and the shell's stall=, chunk=, \
                         in-limit=, in-stall=, out-hold=";

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
    let mut kind = Kind::Es51919;
    let mut packets = None;
    let mut noise = Vec::new();
    let mut parity = true;
    let mut uart = None;
    let mut period = 25;
    let mut burst = None;
    let mut scan_burst = None;
    let mut answers = None;
    // meter= decides the size of packet=, wherever it stands.
    for (k, v) in options {
        if k == "meter" {
            kind = match v.as_str() {
                "es51919" => Kind::Es51919,
                "vc4080" => Kind::Vc4080,
                _ => return Err(Error::Invalid(format!("meter={v}: es51919 or vc4080"))),
            };
        }
    }
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "meter" => {}
            "packet" => packets = Some(cable::parse_packets(v, kind.size(), kind.size())?),
            "noise" => noise = cable::parse_packets(v, 1, 256)?.remove(0),
            "parity" => parity = cable::parse_number(k, v, 0..=1)? == 1,
            "uart" => {
                uart = Some(if v == "native" { None } else { Some(Line::parse(v)?) });
            }
            "period" => period = cable::parse_number(k, v, 4..=1000)?,
            "burst" => burst = cable::parse_burst(v)?,
            "scan-burst" => scan_burst = cable::parse_burst(v)?,
            "answers" => answers = Some(cable::parse_number(k, v, 0..=1000)?),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let packets = packets.unwrap_or_else(|| vec![kind.default_packet().to_vec()]);
    let meter = Meter {
        kind,
        all: packets.clone(),
        packets: Packets::new(packets),
        noise,
        parity,
        stream: Stream::new(period, false),
        period,
        burst,
        scan_burst,
        left: None,
        marks: false,
        quiet: 0,
        sent: 0,
        answers,
        answered: 0,
    };
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Serial LCR cable");
    let cdc = Cdc::new(NAME, &id, faults, meter);
    Ok(match uart {
        Some(None) => cdc,
        Some(Some(line)) => cdc.behind_uart(line),
        None => cdc.behind_uart(Line::parse(kind.line()).expect("the table's lines parse")),
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Es51919,
    Vc4080,
}

impl Kind {
    /// The line the C's table sets for these meters.
    fn line(self) -> &'static str {
        match self {
            Kind::Es51919 => "9600/8n1",
            Kind::Vc4080 => "1200/8n1",
        }
    }

    fn size(self) -> usize {
        match self {
            Kind::Es51919 => 17,
            Kind::Vc4080 => 39,
        }
    }

    fn default_packet(self) -> &'static [u8] {
        match self {
            // LCR mode, series, 1 kHz; 12.345 nF; D 0.123.
            Kind::Es51919 => &[
                0x00, 0x0d, 0x20, 0x50, 0x00, 0x02, 0x30, 0x39, 0x53, 0x00, 0x01, 0x00, 0x7b, 0x03,
                0x00, 0x0d, 0x0a,
            ],
            // L, Q, 1 kHz, parallel, auto: 12345 in range 3, Q 0134 in range 2.
            Kind::Vc4080 => b"LQAPA1234530134210711414062__________\r\n",
        }
    }
}

/// Sets `byte`'s bit 7 for even parity over the other seven.
fn with_even_parity(byte: u8) -> u8 {
    let low = byte & 0x7f;
    if low.count_ones() % 2 == 1 { low | 0x80 } else { low }
}

struct Meter {
    kind: Kind,
    all: Vec<Vec<u8>>,
    packets: Packets,
    noise: Vec<u8>,
    parity: bool,
    stream: Stream,
    period: u32,
    burst: Option<u32>,
    scan_burst: Option<u32>,
    left: Option<u32>,
    marks: bool,
    quiet: u32,
    sent: u32,
    answers: Option<u32>,
    answered: u32,
}

impl Meter {
    fn send_packet(&mut self, port: &mut Port<'_>) {
        port.send(&self.noise);
        let mut packet = self.packets.take();
        if self.kind == Kind::Vc4080 && self.parity {
            for b in &mut packet {
                *b = with_even_parity(*b);
            }
        }
        port.send(&packet);
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        if self.kind != Kind::Vc4080 {
            port.violation(format!("{} bytes written to a meter that only sends", data.len()));
            return;
        }
        for &byte in data {
            if byte != b'N' {
                port.violation(format!("{byte:#04x} written; a packet is asked for with 'N'"));
                continue;
            }
            if self.answers.is_some_and(|n| self.answered >= n) {
                port.mark();
                continue;
            }
            self.answered += 1;
            self.send_packet(port);
        }
    }

    fn changed(&mut self, event: LineEvent, _port: &mut Port<'_>) {
        if event.opened() && self.kind == Kind::Es51919 {
            let first = self.scan_burst.is_some_and(|n| self.sent < n);
            self.left = if first { self.scan_burst } else { self.burst };
            self.marks = !first;
            self.stream = Stream::new(self.period, true).with_burst(self.left);
            self.quiet = 0;
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if self.kind != Kind::Es51919 {
            return;
        }
        if self.stream.poll() {
            self.sent = self.sent.saturating_add(1);
            self.send_packet(port);
            if let Some(left) = &mut self.left {
                *left = left.saturating_sub(1);
            }
        } else if self.left == Some(0) && self.marks {
            // A period after the last packet, so that the tool has read it
            // before an event lands: bytes just delivered are lost to an
            // unplug's hangup, or not, as the pace has it.
            self.quiet += 1;
            if self.quiet >= self.period {
                port.mark();
                self.left = None;
                self.marks = false;
            }
        }
    }

    fn reset(&mut self) {
        self.packets = Packets::new(self.all.clone());
        self.stream = Stream::new(self.period, false);
        self.left = None;
        self.marks = false;
        self.quiet = 0;
        self.sent = 0;
        self.answered = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    /// The meter of `options` with a tool's port open on its line.
    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let cdc = function(&o).unwrap();
        let line = Line::parse(cdc.model.kind.line()).unwrap();
        Host::open(cdc, line, true, true)
    }

    /// Offers `n` reads: how many got a packet.
    fn packets(h: &mut Host<Meter>, n: u32) -> u32 {
        (0..n).map(|_| u32::from(matches!(h.offer(), Reply::Data(_)))).sum()
    }

    fn stripped(data: &[u8]) -> Vec<u8> {
        data.iter().map(|b| b & 0x7f).collect()
    }

    #[test]
    fn the_default_packets_have_the_layouts_frame() {
        let es = Kind::Es51919.default_packet();
        assert_eq!(es.len(), 17);
        assert_eq!(&es[..2], [0x00, 0x0d]);
        assert_eq!(&es[15..], [0x0d, 0x0a]);
        let vc = Kind::Vc4080.default_packet();
        assert_eq!(vc.len(), 39);
        assert_eq!(&vc[37..], b"\r\n");
    }

    #[test]
    fn the_vc4080_answers_each_n_with_parity_bits() {
        let mut h = meter(&[("meter", "vc4080")]);
        h.write(b"N");
        let got = h.read();
        assert_eq!(stripped(&got), Kind::Vc4080.default_packet());
        // 'L' has three ones, so its parity bit is set; 'Q' too; CR has three.
        assert_eq!(got[0], b'L' | 0x80);
        assert_eq!(got[37], b'\r' | 0x80);
        // LF has two: no bit.
        assert_eq!(got[38], b'\n');
        h.write(b"NN");
        assert_eq!(h.read().len(), 78, "per byte, however cut");
        assert_eq!(h.read(), b"", "nothing unasked");
        assert!(h.v.is_empty(), "{:?}", h.v);
    }

    #[test]
    fn the_n_itself_has_even_parity() {
        assert_eq!(with_even_parity(b'N'), b'N');
    }

    #[test]
    fn parity_can_be_off() {
        let mut h = meter(&[("meter", "vc4080"), ("parity", "0")]);
        h.write(b"N");
        assert_eq!(h.read(), Kind::Vc4080.default_packet());
    }

    #[test]
    fn the_vc4080_flags_what_it_is_not_asked() {
        let mut h = meter(&[("meter", "vc4080")]);
        h.write(b"A");
        assert_eq!(h.v.drain().len(), 1);
        assert_eq!(h.read(), b"");
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = meter(&[("meter", "vc4080"), ("answers", "1")]);
        h.write(b"N");
        assert_eq!(h.read().len(), 39);
        assert!(!h.cdc.mark());
        h.write(b"N");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_misset_line_gets_no_answer_and_no_violation() {
        let mut h = meter(&[("meter", "vc4080")]);
        h.set_line(Line::parse("1200/7e1").unwrap());
        h.write(b"N");
        assert_eq!(h.read(), b"");
        assert!(h.v.is_empty());
    }

    #[test]
    fn uart_moves_the_line_or_takes_it_away() {
        let mut h = meter(&[("meter", "vc4080"), ("uart", "1200/7e1")]);
        h.set_line(Line::parse("1200/7e1").unwrap());
        h.write(b"N");
        assert_eq!(h.read().len(), 39);
        let mut h = meter(&[("meter", "vc4080"), ("uart", "native")]);
        h.set_line(Line::parse("115200/8n1").unwrap());
        h.write(b"N");
        assert_eq!(h.read().len(), 39);
    }

    #[test]
    fn noise_comes_before_each_packet() {
        let mut h = meter(&[("meter", "vc4080"), ("noise", "3f3f")]);
        h.write(b"N");
        let got = h.read();
        assert_eq!(got.len(), 41);
        assert_eq!(&got[..2], [0x3f, 0x3f]);
    }

    #[test]
    fn the_es51919_streams_and_starts_over_at_each_open() {
        let mut h = meter(&[("period", "4")]);
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert_eq!(h.offer(), Reply::Data(Kind::Es51919.default_packet().to_vec()));
        for _ in 0..2 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        h.set_lines(true, true);
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert!(matches!(h.offer(), Reply::Data(_)));
        assert!(h.v.is_empty());
    }

    #[test]
    fn the_es51919_takes_nothing_from_the_host() {
        let mut h = meter(&[]);
        h.write(b"N");
        assert_eq!(h.v.drain().len(), 1);
    }

    #[test]
    fn a_burst_ends_in_silence_and_the_point() {
        let mut h = meter(&[("period", "4"), ("burst", "1")]);
        for _ in 0..3 {
            h.offer();
        }
        assert!(matches!(h.offer(), Reply::Data(_)));
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
            assert!(!h.cdc.mark(), "a period later");
        }
        assert_eq!(h.offer(), Reply::Nak);
        assert!(h.cdc.mark());
        for _ in 0..10 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert!(!h.cdc.mark(), "once");
    }

    #[test]
    fn a_scan_burst_is_for_the_first_open_and_marks_nothing() {
        let mut h = meter(&[("period", "4"), ("scan-burst", "1"), ("burst", "2")]);
        // Host::open raises the lines twice, and that is still the scan.
        assert_eq!(packets(&mut h, 30), 1);
        assert!(!h.cdc.mark());
        // The next open, however often it raises them: two, and the point.
        h.set_lines(true, true);
        h.set_lines(true, true);
        assert_eq!(packets(&mut h, 30), 2);
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_burst_of_nothing_never_sends() {
        let mut h = meter(&[("period", "4"), ("burst", "0")]);
        for _ in 0..30 {
            assert_eq!(h.offer(), Reply::Nak);
        }
    }

    #[test]
    fn packets_are_played_in_turn() {
        let p1 = "000d".to_owned() + &"00".repeat(13) + "0d0a";
        let p2 = "000d".to_owned() + &"11".repeat(13) + "0d0a";
        let list = format!("{p1},{p2}");
        let mut h = meter(&[("period", "4"), ("packet", list.as_str())]);
        let mut got = Vec::new();
        for _ in 0..12 {
            if let Reply::Data(d) = h.offer() {
                got.push(d[2]);
            }
        }
        assert_eq!(got, [0x00, 0x11, 0x00]);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let p1 = "000d".to_owned() + &"00".repeat(13) + "0d0a";
        let p2 = "000d".to_owned() + &"11".repeat(13) + "0d0a";
        let list = format!("{p1},{p2}");
        let mut h = meter(&[("period", "4"), ("packet", list.as_str()), ("scan-burst", "1")]);
        h.cdc.model.sent = 5;
        h.cdc.model.answered = 5;
        h.cdc.model.packets.take();
        h.cdc.model.reset();
        assert_eq!(h.cdc.model.sent, 0);
        assert_eq!(h.cdc.model.answered, 0);
        assert_eq!(h.cdc.model.packets.take()[2], 0x00);
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("meter", "x"),
            ("packet", "00"),
            ("period", "3"),
            ("nothing", "1"),
            ("parity", "2"),
            ("uart", "fast"),
            ("answers", "1001"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("stall", "line"), ("noise", "00ff"), ("answers", "0")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
        // meter= decides the size of packet=, wherever it stands.
        let vc = "41".repeat(39);
        let o = vec![("packet".to_owned(), vc), ("meter".to_owned(), "vc4080".to_owned())];
        assert!(build(&o).is_ok());
    }
}
