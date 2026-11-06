// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A MASTECH MS6514 dual-channel thermometer on a CDC-ACM cable.
//!
//! Sources, none of them a firmware or a recording of this unit:
//!
//! - sigrok.org's wiki pages "MASTECH MS6514" and ".../Info": the CP2102, its
//!   strings, 9600/8n1, about 2 frames a second once PC-Link is on, the one
//!   command `a1` ("start sending stored data"), and the 18 byte layout below;
//! - the C driver, for what a host expects of it;
//! - `tc2100` (a Python tool for a sibling unit) and `MS6514GUI`, which read the
//!   same frames: their published frame is the `sibling` case of the scenario.
//!
//! ```text
//!  0  1   65 14
//!  2      bit 0: 1 stored value, 0 live
//!  3  4   index of a stored value, big endian, oldest 0
//!  5  6   main display, big endian, without a sign
//!  7  8   aux display, likewise
//!  9      bits 5-4 mode (3 READ), bits 2-0 thermocouple
//! 10      bit 6 hold, bit 5 record, bits 1-0 unit (1 C, 2 F, 3 K)
//! 11      main: bit 7 negative, bit 6 overload, bit 3 tenths, bits 1-0 function
//! 12      aux: the same bits, and bits 1-0 none, MAX, MIN, AVG
//! 13-15   hours, minutes, seconds
//! 16 17   0d 0a
//! ```
//!
//! No source says how fast stored frames come, what a unit with an empty memory
//! sends, or what follows the last stored frame. Here they come one per offer,
//! and the live stream goes on after them, which is what the driver's end of a
//! dump needs. A real CP2102 can't be shown: `cp210x` binds its ids whatever the
//! class, so the cable keeps its strings but not 10c4:ea60.
//!
//! Emulation artifacts:
//!
//! - The stream is counted in offers of a read (about 20 ms each) from each open,
//!   and it starts over from the first frame there, so a tool that closes within
//!   a period has caused exactly what it read. The real unit streams whoever
//!   listens, at about 500 ms. The default period is shorter, so that a case
//!   costs little, and the first frame after an open is later than the rest, as a
//!   margin ahead of the flush a tool does just after its open.
//! - The first open after power on, a scan's, gets one send and then nothing: a
//!   tool's close takes longer than a period here, so what else came before it
//!   would be the clock's in a golden, and a frame sent in that close can be lost
//!   to the next open's flush, which hangs a capture that counts on its burst.
//! - `bunch=N` stands for frames that queued up while a host wasn't reading.
//! - `still=1` makes the unit silent, from the first open after a live frame went
//!   out, until it is asked, so no live frame can cross the request, which would
//!   end the driver's dump before it starts. The real unit has no such state, and
//!   that crossing is a real race. `still=2` stays silent after
//!   the dump as well. An open is a line state of DTR and RTS up, which a tool
//!   may send more than once, so frames tell a scan's open from the next.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `frame=HEX[,HEX...]`, the live frames, 1 to 64 bytes each, so noise and cut
//!   frames go here too;
//! - `stored=HEX[,HEX...]|none`, what `a1` plays;
//! - `period=N`, offers per send of live frames (4 to 1000, default 5);
//! - `first=N`, offers from an open before the first period starts (0 to 1000,
//!   default 10);
//! - `burst=N`, live frames per open, then silence, with the event point marked
//!   a period later; 0 never sends;
//! - `bunch=N`, frames in one send (1 to 8);
//! - `still=0|1|2`.
//!
//! The pedantic checks: the unit takes `a1` and nothing else, and not while a
//! dump is running, as no path of the driver sends that.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::{self as cable, Stream};
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "mastech-ms6514";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A MASTECH MS6514 thermometer on a CDC-ACM cable; frame=HEX[,HEX...], \
                         stored=HEX[,HEX...]|none, period=N, first=N, burst=N, bunch=N, still=0|1|2, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// The one command: start sending stored data.
const CMD_STORED: u8 = 0xa1;
/// The longest frame an option may hold; the real one has 18 bytes.
const MAX_FRAME: usize = 64;

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
    let mut frames = vec![live_frame(235, 241), live_frame(236, 242), live_frame(237, 243)];
    let mut stored = vec![stored_frame(0, 200, 210), stored_frame(1, 201, 211), stored_frame(2, 202, 212)];
    let mut period = 5;
    let mut first = 10;
    let mut burst = None;
    let mut bunch = 1;
    let mut still = 0;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "frame" => frames = cable::parse_packets(v, 1, MAX_FRAME)?,
            "stored" if v == "none" => stored = Vec::new(),
            "stored" => stored = cable::parse_packets(v, 1, MAX_FRAME)?,
            "period" => period = cable::parse_number(k, v, 4..=1000)?,
            "first" => first = cable::parse_number(k, v, 0..=1000)?,
            "burst" => burst = cable::parse_burst(v)?,
            "bunch" => bunch = cable::parse_number(k, v, 1..=8)?,
            "still" => still = cable::parse_number(k, v, 0..=2)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let meter = Meter {
        live: frames,
        next_live: 0,
        stored,
        dump: VecDeque::new(),
        stream: Stream::new(period, false),
        period,
        first,
        wait: first,
        burst,
        burst_left: burst,
        quiet: 0,
        bunch,
        still,
        sent_live: false,
        probing: true,
        holding: false,
    };
    // The wiki's lsusb has these strings; 10c4:ea60 would bind cp210x.
    let id = Identity::new(0x1209, 0x0001, "Silicon Labs", "CP2102 USB to UART Bridge Controller");
    let line = Line::parse("9600/8n1").expect("a line parses");
    Ok(Cdc::new(NAME, &id, faults, meter).behind_uart(line))
}

/// A frame in tenths of a degree C, thermocouple K, T1 on main and T2 on aux.
fn frame(source: u8, index: u16, mode: u8, main: u16, aux: u16) -> Vec<u8> {
    let [ih, il] = index.to_be_bytes();
    let [mh, ml] = main.to_be_bytes();
    let [ah, al] = aux.to_be_bytes();
    vec![
        0x65, 0x14, source, ih, il, mh, ml, ah, al, mode | 1, 1, 8, 8, 12, 34, 56, 0x0d, 0x0a,
    ]
}

fn live_frame(main: u16, aux: u16) -> Vec<u8> {
    frame(0, 0, 0, main, aux)
}

fn stored_frame(index: u16, main: u16, aux: u16) -> Vec<u8> {
    frame(1, index, 0x30, main, aux)
}

struct Meter {
    live: Vec<Vec<u8>>,
    next_live: usize,
    stored: Vec<Vec<u8>>,
    /// What is left of the stored frames a request asked for.
    dump: VecDeque<Vec<u8>>,
    stream: Stream,
    period: u32,
    first: u32,
    /// Offers left before the stream's first period.
    wait: u32,
    burst: Option<u32>,
    burst_left: Option<u32>,
    quiet: u32,
    bunch: u32,
    still: u32,
    /// A live frame went out since power on.
    sent_live: bool,
    /// In the first open: it gets one send.
    probing: bool,
    /// `still=` is keeping the live stream back.
    holding: bool,
}

impl Meter {
    /// The next live frame, if the burst has one.
    fn take_live(&mut self) -> Option<Vec<u8>> {
        if self.holding {
            return None;
        }
        if let Some(left) = &mut self.burst_left {
            if *left == 0 {
                return None;
            }
            *left -= 1;
        }
        self.sent_live = true;
        let f = self.live[self.next_live].clone();
        self.next_live = (self.next_live + 1) % self.live.len();
        Some(f)
    }

    /// Up to `bunch` frames in one send: stored ones first, and live ones after
    /// them in the same send, when the bunch goes on past the dump's end.
    fn send(&mut self, port: &mut Port<'_>) {
        let mut out = Vec::new();
        for _ in 0..self.bunch {
            let f = match self.dump.pop_front() {
                Some(f) => Some(f),
                None => self.take_live(),
            };
            let Some(f) = f else { break };
            out.extend_from_slice(&f);
        }
        port.send(&out);
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte != CMD_STORED {
                port.violation(format!("{byte:#04x} written; the one command is 0xa1"));
            } else if !self.dump.is_empty() {
                port.violation("0xa1 written during a dump".to_owned());
            } else {
                self.dump = self.stored.iter().cloned().collect();
                self.holding = self.holding && self.still == 2;
                self.wait = 0;
                self.stream.restart();
            }
        }
    }

    fn changed(&mut self, event: LineEvent, _port: &mut Port<'_>) {
        if event.opened() {
            self.probing = self.probing && !self.sent_live;
            self.holding = self.still > 0 && self.sent_live;
            self.dump.clear();
            self.next_live = 0;
            self.wait = self.first;
            self.stream.restart();
            self.burst_left = self.burst;
            self.quiet = 0;
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if !self.dump.is_empty() {
            self.send(port);
            return;
        }
        if self.holding || (self.probing && self.sent_live) {
            return;
        }
        if self.wait > 0 {
            self.wait -= 1;
            return;
        }
        if self.burst_left == Some(0) {
            // A period after the last frame, so that the tool has read it
            // before an event lands.
            if self.quiet < self.period {
                self.quiet += 1;
                if self.quiet == self.period {
                    port.mark();
                }
            }
            return;
        }
        if self.stream.poll() {
            self.send(port);
        }
    }

    fn reset(&mut self) {
        self.sent_live = false;
        self.probing = true;
        self.holding = false;
        self.dump.clear();
        self.next_live = 0;
        self.wait = self.first;
        self.burst_left = self.burst;
        self.quiet = 0;
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
        let mut h = Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true);
        // The tests below count from the first period.
        h.cdc.model.first = 0;
        h.cdc.model.wait = 0;
        h.cdc.model.probing = false;
        h
    }

    /// Offers until something comes, within a bound.
    fn next(h: &mut Host<Meter>) -> Vec<u8> {
        for _ in 0..1100 {
            if let Reply::Data(d) = h.offer() {
                return d;
            }
        }
        panic!("nothing came");
    }

    #[test]
    fn frames_have_the_wikis_layout() {
        let f = stored_frame(258, 235, 241);
        assert_eq!(f.len(), 18);
        assert_eq!(&f[..2], [0x65, 0x14]);
        assert_eq!(f[2] & 1, 1);
        assert_eq!(&f[3..5], [1, 2]);
        assert_eq!(&f[5..7], [0, 235]);
        assert_eq!(&f[7..9], [0, 241]);
        assert_eq!(f[9] >> 4 & 3, 3, "READ");
        assert_eq!(f[10] & 3, 1, "C");
        assert_eq!(f[11] & 8, 8, "tenths");
        assert_eq!(&f[16..], [0x0d, 0x0a]);
        assert_eq!(live_frame(1, 2)[2] & 1, 0);
    }

    #[test]
    fn the_first_frame_waits_for_first() {
        let o = vec![
            ("first".to_owned(), "2".to_owned()),
            ("period".to_owned(), "4".to_owned()),
        ];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true);
        h.cdc.model.probing = false;
        for _ in 0..5 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert_eq!(h.offer(), Reply::Data(live_frame(235, 241)));
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert_eq!(h.offer(), Reply::Data(live_frame(236, 242)));
    }

    #[test]
    fn a_scans_open_gets_one_send() {
        let o = vec![("period".to_owned(), "4".to_owned()), ("first".to_owned(), "0".to_owned())];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true);
        assert_eq!(next(&mut h), live_frame(235, 241));
        for _ in 0..20 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        h.set_lines(true, true);
        assert_eq!(next(&mut h), live_frame(235, 241));
        assert_eq!(next(&mut h), live_frame(236, 242));
    }

    #[test]
    fn it_streams_in_periods_from_each_open() {
        let mut h = meter(&[("period", "4")]);
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert_eq!(h.offer(), Reply::Data(live_frame(235, 241)));
        assert_eq!(next(&mut h), live_frame(236, 242));
        h.set_lines(true, true);
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert_eq!(h.offer(), Reply::Data(live_frame(235, 241)), "an open starts over");
        assert!(h.v.is_empty());
    }

    #[test]
    fn a1_plays_the_stored_frames_then_the_live_stream() {
        let mut h = meter(&[("period", "4")]);
        h.write(&[0xa1]);
        assert_eq!(h.offer(), Reply::Data(stored_frame(0, 200, 210)));
        assert_eq!(h.offer(), Reply::Data(stored_frame(1, 201, 211)));
        assert_eq!(h.offer(), Reply::Data(stored_frame(2, 202, 212)));
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert_eq!(h.offer(), Reply::Data(live_frame(235, 241)));
        assert!(h.v.is_empty());
    }

    #[test]
    fn an_empty_memory_goes_straight_on_live() {
        let mut h = meter(&[("period", "4"), ("stored", "none")]);
        h.write(&[0xa1]);
        assert_eq!(next(&mut h), live_frame(235, 241));
        assert!(h.v.is_empty());
    }

    #[test]
    fn it_flags_what_it_is_not_asked() {
        let mut h = meter(&[]);
        h.write(&[0x00]);
        assert_eq!(h.v.drain().len(), 1, "another byte");
        h.write(&[0xa1]);
        assert!(h.v.is_empty());
        h.write(&[0xa1]);
        assert_eq!(h.v.drain().len(), 1, "during a dump");
        for _ in 0..3 {
            h.offer();
        }
        h.write(&[0xa1]);
        assert!(h.v.is_empty(), "after one");
    }

    #[test]
    fn a_misset_line_gets_nothing_through() {
        let mut h = meter(&[("period", "4")]);
        h.set_line(Line::parse("1200/8n1").unwrap());
        h.write(&[0xa1]);
        for _ in 0..20 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert!(h.v.is_empty());
    }

    #[test]
    fn a_bunch_runs_from_the_dump_into_the_stream() {
        let mut h = meter(&[("period", "4"), ("bunch", "5")]);
        h.write(&[0xa1]);
        let mut want = Vec::new();
        for i in 0..3 {
            want.extend(stored_frame(i, 200 + i, 210 + i));
        }
        want.extend(live_frame(235, 241));
        want.extend(live_frame(236, 242));
        assert_eq!(next(&mut h), want);
    }

    #[test]
    fn a_burst_ends_in_silence_and_the_point() {
        let mut h = meter(&[("period", "4"), ("burst", "1")]);
        assert_eq!(next(&mut h), live_frame(235, 241));
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
    fn still_holds_the_stream_from_the_second_open_until_asked() {
        let mut h = meter(&[("period", "4"), ("still", "1")]);
        assert_eq!(next(&mut h), live_frame(235, 241), "a scan's open streams");
        h.set_lines(true, true);
        for _ in 0..20 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        h.write(&[0xa1]);
        assert_eq!(h.offer(), Reply::Data(stored_frame(0, 200, 210)));
    }

    #[test]
    fn still_2_stays_silent_after_the_dump() {
        let mut h = meter(&[("period", "4"), ("still", "2"), ("stored", "none")]);
        assert_eq!(next(&mut h), live_frame(235, 241));
        h.set_lines(true, true);
        h.write(&[0xa1]);
        for _ in 0..20 {
            assert_eq!(h.offer(), Reply::Nak);
        }
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("frame", "zz"),
            ("stored", ""),
            ("period", "3"),
            ("bunch", "9"),
            ("bunch", "0"),
            ("still", "3"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("stored", "none"), ("frame", "00"), ("burst", "0")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
