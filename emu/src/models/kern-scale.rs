// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A KERN EW/EG-N scale on its RS-232 port, on the CDC-ACM shell, for sigrok's
//! `kern-ew-6200-2nm`.
//!
//! Sources, and how sure they are:
//! - KERN's manual `EW_EG-(N)-EWB-BA-e-1226`, chapter 8 (kern-sohn.com answered a
//!   plain fetch with 403; read through `web.archive.org`, from the link on
//!   sigrok's wiki): the two formats, P1 D1..D7 (D8) U1 U2 S1 S2 CR LF, the
//!   units, S1 `L G H`, S2 `S U E` or a space, the four character commands, O1
//!   as continuous output, and the answers `A00` and `E01`.
//! - sigrok-dumps' `uart/kern_ew_6200-2nm`, captures of a real unit, decoded by
//!   hand: `+0026.90 G U`, `-0000.04 G S`, `+00127.20 G S`, `+002014.8CT S`,
//!   `+000000 PC S`, `+000000  % U`, and over its range `+0000.00 G E`. Its
//!   values are zero filled, where the manual speaks of leading zeros
//!   suppressed by spaces; both can be played with `packet=`. A stable packet
//!   comes about every 500 ms, and an unstable or error one about every 128 ms,
//!   which at 1200/8n2 (11 bits a byte) is what 14 bytes take on the line: those
//!   follow each other with no gap. The longest silence is 2018 ms, at a tare. The
//!   captures hold no host's commands, so what mode that was isn't known, and that
//!   `O1` answers `A00` before the next packet is the manual's and the C's comment,
//!   not an observation.
//! - No firmware source, no unit on a desk.
//!
//! Emulation artifacts:
//! - The real cable is RS-232, so a USB adapter of some kind; here CDC-ACM. On a
//!   misset line nothing passes ([`Cdc::behind_uart`]), where a real one gives
//!   garbage. The rate, data bits and whether there is parity decide, not the
//!   stop bits: a host at 1200/8n1 hears a 8n2 scale.
//! - The default is a silent scale until `O1`, and each `O1` starts the stream
//!   over. A real one's menu may have it sending already.
//! - A packet goes out whole, however slow the line. The port's unit tests have
//!   a byte-paced fake.
//! - The rate is a count of read offers ([`Stream`]) from each `O1`.
//! - `burst=`, `scan-burst=` and `group=` make a capture's packets independent
//!   of how long a tool listens. No scale is known to fall silent, or to send a
//!   group at once, which stands for a host that read late.
//! - `late=N` holds the answer to `O1`, and the stream, for `N` read offers. No
//!   unit is known to take so long.
//!
//! Options, besides the shell's ([`Faults`]):
//! - `packet=HEX[,HEX...]`, the packets played in turn, a scan's first;
//! - `noise=HEX`, bytes sent before each packet, `pad=HEX` after each;
//! - `uart=LINE|native`, the line behind the cable, instead of 1200/8n2;
//! - `ack=a00|e01|none`, the answer to `O1`. The manual names only the answer:
//!   that no stream starts after `E01` is an assumption here, as `late=` is. A
//!   stream already running goes on;
//! - `ack-from=N`, the `O1` from which `ack=` holds, counted from 1; before it the
//!   answer is `A00`;
//! - `period=N`, offers per packet (4 to 1000, default 25);
//! - `burst=N`, shots per `O1`, then silence and a mark a period later; 0 never
//!   sends;
//! - `scan-burst=N`, the same for the first `O1` since power on, a scan's, one
//!   packet a shot and no mark;
//! - `group=N`, packets per shot after the first `O1`, sent at once;
//! - `late=N`, as above.
//!
//! The default packet is the 15-byte form, though the captured unit mostly had the
//! 14-byte one: C needs a 15th byte to look at a 14-byte packet, so a 14-byte
//! default would leave the last packet of every burst unread by it, and most cases
//! would compare nothing. `packet=` and `pad=` have cases for the other form.
//!
//! The pedantic checks: what the host sends is whole commands of four
//! characters, and only `O1`. A real scale has `O0` to `OB` and `T `, which no C
//! path sends: a tool that means them should not read this as a refusal.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::{self as cable, Packets, Stream};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "kern-scale";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "KERN EW/EG-N scale on a CDC-ACM cable; packet=HEX[,HEX...], noise=HEX, \
                         pad=HEX, uart=LINE|native, ack=a00|e01|none, ack-from=N, period=N, burst=N, \
                         scan-burst=N, group=N, late=N, and the shell's stall=, chunk=, \
                         in-limit=, in-stall=, out-hold=";

/// From a capture of a real unit: 127.2 g, stable, in the 15-byte format.
const DEFAULT_PACKET: &[u8] = b"+00127.20 G S\r\n";
/// The line the C's table and the captures have.
const LINE: &str = "1200/8n2";
/// The one command a C path sends.
const O1: &[u8] = b"O1\r\n";
/// The size of every command.
const COMMAND: usize = 4;

/// A scale from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Scale>> {
    let mut faults = Faults::default();
    let mut packets = vec![DEFAULT_PACKET.to_vec()];
    let mut noise = Vec::new();
    let mut pad = Vec::new();
    let mut uart = Some(Line::parse(LINE)?);
    let mut ack = Ack::A00;
    let mut ack_from = 1;
    let mut period = 25;
    let mut burst = None;
    let mut scan_burst = None;
    let mut group = 1;
    let mut late = 0;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "packet" => packets = cable::parse_packets(v, 1, 256)?,
            "noise" => noise = cable::parse_packets(v, 1, 256)?.remove(0),
            "pad" => pad = cable::parse_packets(v, 1, 256)?.remove(0),
            "uart" => uart = if v == "native" { None } else { Some(Line::parse(v)?) },
            "ack" => {
                ack = match v.as_str() {
                    "a00" => Ack::A00,
                    "e01" => Ack::E01,
                    "none" => Ack::None,
                    _ => return Err(Error::Invalid(format!("ack={v}: a00, e01 or none"))),
                };
            }
            "ack-from" => ack_from = cable::parse_number(k, v, 1..=1000)?,
            "period" => period = cable::parse_number(k, v, 4..=1000)?,
            "burst" => burst = cable::parse_burst(v)?,
            "scan-burst" => scan_burst = cable::parse_burst(v)?,
            "group" => group = cable::parse_number(k, v, 1..=100)?,
            "late" => late = cable::parse_number(k, v, 0..=100_000)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let scale = Scale {
        all: packets.clone(),
        packets: Packets::new(packets),
        noise,
        pad,
        ack,
        ack_from,
        period,
        burst,
        scan_burst,
        group,
        late,
        input: Vec::new(),
        commands: 0,
        waiting: None,
        stream: Stream::new(period, false),
        shot_size: 1,
        left: None,
        marks: false,
        quiet: 0,
    };
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "KERN scale cable");
    let cdc = Cdc::new(NAME, &id, faults, scale);
    Ok(match uart {
        Some(line) => cdc.behind_uart(line),
        None => cdc,
    })
}

/// The answer to a command, chapter 8.6 of the manual.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Ack {
    A00,
    E01,
    None,
}

struct Scale {
    all: Vec<Vec<u8>>,
    packets: Packets,
    noise: Vec<u8>,
    pad: Vec<u8>,
    ack: Ack,
    ack_from: u32,
    period: u32,
    burst: Option<u32>,
    scan_burst: Option<u32>,
    group: u32,
    late: u32,
    /// What came of a command so far, under [`COMMAND`] bytes.
    input: Vec<u8>,
    /// `O1`s since power on.
    commands: u32,
    /// Offers left before an `O1` takes effect.
    waiting: Option<u32>,
    stream: Stream,
    /// Packets per shot of the stream now running.
    shot_size: u32,
    /// Shots left of a burst.
    left: Option<u32>,
    marks: bool,
    quiet: u32,
}

impl Scale {
    /// `O1` takes effect: the answer, and the stream unless it is refused.
    fn start(&mut self, port: &mut Port<'_>) {
        let ack = if self.commands < self.ack_from { Ack::A00 } else { self.ack };
        match ack {
            Ack::A00 => port.send(b"A00\r\n"),
            Ack::E01 => {
                port.send(b"E01\r\n");
                return;
            }
            Ack::None => {}
        }
        let first = self.commands == 1;
        self.left = if first { self.scan_burst } else { self.burst };
        self.shot_size = if first { 1 } else { self.group };
        self.marks = !first && self.left.is_some();
        self.quiet = 0;
        self.stream = Stream::new(self.period, true).with_burst(self.left);
    }

    fn send_packet(&mut self, port: &mut Port<'_>) {
        port.send(&self.noise);
        let packet = self.packets.take();
        port.send(&packet);
        port.send(&self.pad);
    }
}

impl Serial for Scale {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        self.input.extend_from_slice(data);
        while self.input.len() >= COMMAND {
            let command: Vec<u8> = self.input.drain(..COMMAND).collect();
            if command != O1 {
                port.violation(format!(
                    "{} written; the C sends O1 CR LF alone",
                    command.escape_ascii()
                ));
                self.input.clear();
                continue;
            }
            self.commands = self.commands.saturating_add(1);
            if self.late == 0 {
                self.start(port);
            } else {
                self.waiting = Some(self.late);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, _port: &mut Port<'_>) {
        if event.opened() {
            self.input.clear();
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if let Some(left) = self.waiting {
            if left > 1 {
                self.waiting = Some(left - 1);
            } else {
                self.waiting = None;
                self.start(port);
            }
            return;
        }
        if self.stream.poll() {
            for _ in 0..self.shot_size {
                self.send_packet(port);
            }
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
        self.input.clear();
        self.commands = 0;
        self.waiting = None;
        self.stream = Stream::new(self.period, false);
        self.shot_size = 1;
        self.left = None;
        self.marks = false;
        self.quiet = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    /// The scale of `options` with a tool's port open on its line.
    fn scale(options: &[(&str, &str)]) -> Host<Scale> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse(LINE).unwrap(), true, true)
    }

    /// Offers `n` reads and gives what came of them, in one piece.
    fn offers(h: &mut Host<Scale>, n: u32) -> Vec<u8> {
        let mut got = Vec::new();
        for _ in 0..n {
            if let Reply::Data(d) = h.offer() {
                got.extend_from_slice(&d);
            }
        }
        got
    }

    #[test]
    fn it_is_silent_until_o1_then_answers_and_streams() {
        let mut h = scale(&[("period", "4")]);
        assert_eq!(offers(&mut h, 20), b"");
        h.write(b"O1\r\n");
        assert_eq!(h.offer(), Reply::Data(b"A00\r\n".to_vec()));
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert_eq!(h.offer(), Reply::Data(DEFAULT_PACKET.to_vec()));
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert_eq!(h.offer(), Reply::Data(DEFAULT_PACKET.to_vec()));
        assert!(h.v.is_empty(), "{:?}", h.v);
    }

    #[test]
    fn a_command_can_come_in_pieces() {
        let mut h = scale(&[("period", "4")]);
        h.write(b"O1");
        assert_eq!(h.read(), b"");
        h.write(b"\r\n");
        assert_eq!(h.read(), b"A00\r\n");
        assert!(h.v.is_empty(), "{:?}", h.v);
    }

    #[test]
    fn another_command_is_flagged() {
        for bad in [&b"O0\r\n"[..], b"T \r\n", b"O1\n\r", b"O1\r\r"] {
            let mut h = scale(&[("period", "4")]);
            h.write(bad);
            assert_eq!(h.v.drain().len(), 1, "{}", bad.escape_ascii());
            assert_eq!(offers(&mut h, 10), b"", "{}", bad.escape_ascii());
        }
    }

    #[test]
    fn half_a_command_does_not_outlast_the_tool() {
        let mut h = scale(&[("period", "4")]);
        h.write(b"O1");
        h.set_lines(true, true);
        h.write(b"O1\r\n");
        assert_eq!(h.read(), b"A00\r\n");
        assert!(h.v.is_empty(), "{:?}", h.v);
    }

    #[test]
    fn e01_starts_nothing_and_no_answer_still_streams() {
        let mut h = scale(&[("period", "4"), ("ack", "e01")]);
        h.write(b"O1\r\n");
        assert_eq!(h.read(), b"E01\r\n");
        assert_eq!(offers(&mut h, 20), b"");
        let mut h = scale(&[("period", "4"), ("ack", "none")]);
        h.write(b"O1\r\n");
        assert_eq!(offers(&mut h, 4), DEFAULT_PACKET);
    }

    #[test]
    fn ack_from_puts_off_the_answer() {
        let mut h =
            scale(&[("period", "4"), ("ack", "e01"), ("ack-from", "2"), ("scan-burst", "1")]);
        h.write(b"O1\r\n");
        assert_eq!(h.offer(), Reply::Data(b"A00\r\n".to_vec()));
        assert_eq!(offers(&mut h, 8), DEFAULT_PACKET);
        h.write(b"O1\r\n");
        assert_eq!(h.read(), b"E01\r\n");
        assert_eq!(offers(&mut h, 20), b"");
    }

    #[test]
    fn a_misset_line_hears_nothing() {
        let mut h = scale(&[("period", "4")]);
        h.set_line(Line::parse("9600/8n1").unwrap());
        h.write(b"O1\r\n");
        assert_eq!(offers(&mut h, 20), b"");
        assert!(h.v.is_empty());
    }

    #[test]
    fn uart_moves_the_line_or_takes_it_away() {
        let mut h = scale(&[("period", "4"), ("uart", "9600/8o2")]);
        h.set_line(Line::parse("9600/8o2").unwrap());
        h.write(b"O1\r\n");
        assert_eq!(h.read(), b"A00\r\n");
        let mut h = scale(&[("period", "4"), ("uart", "native")]);
        h.set_line(Line::parse("115200/8n1").unwrap());
        h.write(b"O1\r\n");
        assert_eq!(h.read(), b"A00\r\n");
    }

    #[test]
    fn noise_and_pad_go_around_each_packet() {
        let mut h = scale(&[("period", "4"), ("noise", "3f"), ("pad", "21")]);
        h.write(b"O1\r\n");
        h.offer();
        let got = offers(&mut h, 4);
        assert_eq!(got[0], 0x3f);
        assert_eq!(&got[1..16], DEFAULT_PACKET);
        assert_eq!(got[16], 0x21);
    }

    #[test]
    fn a_scan_burst_is_for_the_first_o1_and_a_burst_marks_the_end() {
        let mut h = scale(&[("period", "4"), ("scan-burst", "1"), ("burst", "2"), ("group", "3")]);
        h.write(b"O1\r\n");
        h.offer();
        assert_eq!(offers(&mut h, 30).len(), 15, "one packet, whatever the group");
        assert!(!h.cdc.mark());
        h.write(b"O1\r\n");
        h.offer();
        assert_eq!(offers(&mut h, 4).len(), 45, "a group at once");
        assert_eq!(offers(&mut h, 4).len(), 45);
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
            assert!(!h.cdc.mark(), "a period later");
        }
        assert_eq!(h.offer(), Reply::Nak);
        assert!(h.cdc.mark());
        assert_eq!(offers(&mut h, 30), b"");
        assert!(!h.cdc.mark(), "once");
    }

    #[test]
    fn a_burst_of_nothing_never_sends() {
        let mut h = scale(&[("period", "4"), ("scan-burst", "0")]);
        h.write(b"O1\r\n");
        h.offer();
        assert_eq!(offers(&mut h, 30), b"");
    }

    #[test]
    fn late_holds_the_answer_and_the_stream() {
        let mut h = scale(&[("period", "4"), ("late", "3")]);
        h.write(b"O1\r\n");
        assert_eq!(offers(&mut h, 2), b"");
        assert_eq!(offers(&mut h, 1), b"A00\r\n");
        assert_eq!(offers(&mut h, 4), DEFAULT_PACKET);
    }

    #[test]
    fn packets_are_played_in_turn_and_a_reset_starts_over() {
        let mut h = scale(&[("period", "4"), ("packet", "2b,2d")]);
        h.write(b"O1\r\n");
        h.offer();
        assert_eq!(offers(&mut h, 12), b"+-+");
        h.cdc.model.reset();
        assert_eq!(h.cdc.model.commands, 0);
        assert_eq!(offers(&mut h, 12), b"", "silent again");
        h.write(b"O1\r\n");
        h.offer();
        assert_eq!(offers(&mut h, 4), b"+");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("packet", "zz"),
            ("noise", ""),
            ("pad", "1"),
            ("uart", "fast"),
            ("ack", "x"),
            ("period", "3"),
            ("group", "0"),
            ("ack-from", "0"),
            ("late", "100001"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("stall", "line"), ("uart", "native"), ("ack", "none")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
