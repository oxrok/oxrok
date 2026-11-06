// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A French electricity meter's customer output (télé-information client),
//! "historic" form, behind a serial cable, on the CDC-ACM shell.
//!
//! The meter only sends, for as long as it has power: a host cannot ask it
//! anything, and nothing on the line tells it a host is there. A frame is STX,
//! then groups of LF, label, space, data, space, control, CR, then ETX. Control
//! is the sum of label, space and data, its low six bits, plus 0x20. An
//! interruption cuts a frame with EOT, and the meter starts a new frame when it
//! comes back. The line is 1200 bd, 7 data bits, even parity, one stop bit.
//!
//! Sources, and how sure they are:
//! - Enedis-NOI-CPT_02E, version 6, of 15/02/2017, from `enedis.fr`: 1.5.1 for
//!   the characters, 1.5.2 for the frame and the control's "mode 1", 2.4's
//!   table 3 (the single-phase multi-rate meter, CBEMM) for the groups, their
//!   order and widths, the `BASE`, `HC..`, `EJP.` and `BBRx` options, and the
//!   16.7 to 33.4 ms between frames. That is the meter's maker's specification
//!   and is taken as is.
//! - `github.com/hallard/LibTeleinfo`: the same control in its `calcChecksum`,
//!   written against real meters.
//! - sigrok's C driver, for which labels a host looks at.
//! - No recording of a real line, no unit on a desk. Each default frame is built
//!   from table 3, with invented values.
//!
//! Emulation artifacts:
//! - The cable is CDC-ACM rather than an FTDI or a demodulator board on a UART.
//!   On a misset line nothing passes ([`Cdc::behind_uart`]), where a real one
//!   would give garbage. Parity bits are not on the wire here: what arrives has
//!   bit 7 clear, as from a UART at 7e1.
//! - A frame goes out whole, however slow the line, where a real one takes over
//!   a second at 120 bytes a second. The tests of the port hold a byte-paced
//!   fake for that.
//! - A frame's rate is a count of read offers, [`Stream`], from each open. So
//!   the time between frames is the host's, and has nothing of a real meter's
//!   16.7 to 33.4 ms.
//! - `burst=` and `scan-burst=` are silences no meter is known to have, save for
//!   an interruption, whose length no source gives: they make a capture's frames
//!   independent of how long a tool listens.
//! - The default frames' values are made up: each frame's index counts one up, so
//!   frames tell apart.
//!
//! Options, besides the shell's ([`Faults`]):
//! - `optarif=TEXT`, the option a default frame says, of 4 characters or any
//!   other: `BASE` (the default), `HC..`, `EJP.`, `BBR(`. A `BBR` start gets
//!   Tempo's six indexes, `HC..` two, `EJP.` two, anything else `BASE`'s one;
//! - `frame=GROUP[+GROUP...][,...]`, frames played in turn, and from the first
//!   after the last. A group is `LABEL:DATA`, whose control is the right one, or
//!   `LABEL:DATA:XX` with a control in hex, or `=HEX`, these bytes. An empty
//!   `DATA` is an empty field;
//! - `packet=HEX[,HEX...]`, bytes played in turn in place of frames, for what a
//!   group builder would refuse to spell;
//! - `noise=HEX`, bytes sent before each frame;
//! - `uart=LINE|native`, the line behind the cable;
//! - `period=N`, offers per frame (4 to 1000, default 50);
//! - `burst=N`, frames per open, then silence and a mark a period later; 0 never
//!   sends;
//! - `scan-burst=N`, the same for the opens up to the Nth frame since power on,
//!   which are a scan's, with no mark. In frames, as a tool's open can raise the
//!   lines more than once;
//! - `late=N`, offers with no frame after an open that is no scan's, so a tool's time
//!   limit can run out before the first frame by a margin no stall takes away;
//! - `eot=N`, every Nth frame stops half way through with EOT, as an
//!   interruption does.
//!
//! The pedantic check: the meter takes nothing, so a byte written is flagged.
//! A real one has no input at all.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::{self as cable, Packets, Stream};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "teleinfo";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "EDF Teleinfo on a CDC-ACM cable; optarif=TEXT, frame=GROUP+GROUP,..., \
                         packet=HEX[,HEX...], noise=HEX, uart=LINE|native, period=N, burst=N, \
                         scan-burst=N, late=N, eot=N, and the shell's stall=, chunk=, in-limit=, in-stall=, \
                         out-hold=";

const STX: u8 = 0x02;
const ETX: u8 = 0x03;
const EOT: u8 = 0x04;
const LF: u8 = 0x0a;
const CR: u8 = 0x0d;

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
    let mut optarif = "BASE".to_owned();
    let mut played = None;
    let mut noise = Vec::new();
    let mut uart = Some(Line::parse("1200/7e1")?);
    let mut period = 50;
    let mut burst = None;
    let mut scan_burst = None;
    let mut eot = None;
    let mut late = 0;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "optarif" => v.clone_into(&mut optarif),
            "frame" => played = Some(parse_frames(v)?),
            "packet" => played = Some(cable::parse_packets(v, 1, 1024)?),
            "noise" => noise = cable::parse_packets(v, 1, 1024)?.remove(0),
            "uart" => uart = if v == "native" { None } else { Some(Line::parse(v)?) },
            "period" => period = cable::parse_number(k, v, 4..=1000)?,
            "burst" => burst = cable::parse_burst(v)?,
            "scan-burst" => scan_burst = cable::parse_burst(v)?,
            "late" => late = cable::parse_number(k, v, 0..=100_000)?,
            "eot" => eot = Some(cable::parse_number(k, v, 1..=1000)?),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let meter = Meter {
        all: played.clone(),
        played: played.map(Packets::new),
        optarif,
        made: 0,
        noise,
        stream: Stream::new(period, false),
        period,
        burst,
        scan_burst,
        left: None,
        marks: false,
        quiet: 0,
        sent: 0,
        eot,
        late,
        wait: 0,
    };
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Teleinfo cable");
    let cdc = Cdc::new(NAME, &id, faults, meter);
    Ok(match uart {
        Some(line) => cdc.behind_uart(line),
        None => cdc,
    })
}

/// A group's control, mode 1: label, space and data.
fn control(label: &[u8], data: &[u8]) -> u8 {
    let sum = label.iter().chain(b" ").chain(data).fold(0u32, |sum, b| sum + u32::from(*b));
    (sum & 0x3f) as u8 + 0x20
}

/// A group, on the wire.
fn group(label: &str, data: &str, wrong: Option<u8>) -> Vec<u8> {
    let (label, data) = (label.as_bytes(), data.as_bytes());
    let mut out = vec![LF];
    out.extend_from_slice(label);
    out.push(b' ');
    out.extend_from_slice(data);
    out.push(b' ');
    out.push(wrong.unwrap_or_else(|| control(label, data)));
    out.push(CR);
    out
}

/// A frame around its groups.
fn frame<'a>(groups: impl IntoIterator<Item = &'a [u8]>) -> Vec<u8> {
    let mut out = vec![STX];
    for g in groups {
        out.extend_from_slice(g);
    }
    out.push(ETX);
    out
}

/// `frame=`: see the module.
fn parse_frames(value: &str) -> Result<Vec<Vec<u8>>> {
    let bad = |what: &str| Error::Invalid(format!("frame={value}: {what}"));
    value
        .split(',')
        .map(|text| {
            let groups = text
                .split('+')
                .map(|g| {
                    if let Some(hex) = g.strip_prefix('=') {
                        let mut raw = cable::parse_packets(hex, 1, 1024)?;
                        return Ok(raw.remove(0));
                    }
                    let mut fields = g.split(':');
                    let label = fields.next().unwrap_or_default();
                    let data = fields.next().ok_or_else(|| bad("a group is LABEL:DATA"))?;
                    let wrong = fields
                        .next()
                        .map(|x| u8::from_str_radix(x, 16).map_err(|_| bad("a control is hex")))
                        .transpose()?;
                    if label.is_empty() || fields.next().is_some() {
                        return Err(bad("a group is LABEL:DATA[:XX]"));
                    }
                    Ok(group(label, data, wrong))
                })
                .collect::<Result<Vec<_>>>()?;
            Ok(frame(groups.iter().map(Vec::as_slice)))
        })
        .collect()
}

/// A default frame of an option, the `n`th since power on: table 3's order.
fn default_frame(optarif: &str, n: u32) -> Vec<u8> {
    let index = |base: u32| format!("{:09}", base + n);
    let indexes: Vec<(&str, String)> = match optarif {
        "HC.." => vec![("HCHC", index(1_000)), ("HCHP", index(2_000))],
        "EJP." => vec![("EJPHN", index(3_000)), ("EJPHPM", index(4_000))],
        o if o.starts_with("BBR") => vec![
            ("BBRHCJB", index(5_000)),
            ("BBRHPJB", index(6_000)),
            ("BBRHCJW", index(7_000)),
            ("BBRHPJW", index(8_000)),
            ("BBRHCJR", index(9_000)),
            ("BBRHPJR", index(10_000)),
        ],
        _ => vec![("BASE", index(12_000))],
    };
    let mut groups = vec![
        group("ADCO", "031428097115", None),
        group("OPTARIF", optarif, None),
        group("ISOUSC", "30", None),
    ];
    groups.extend(indexes.iter().map(|(label, data)| group(label, data, None)));
    let ptec = if optarif == "BASE" { "TH.." } else { "HP.." };
    groups.push(group("PTEC", ptec, None));
    if optarif.starts_with("BBR") {
        groups.push(group("DEMAIN", "----", None));
    }
    groups.extend([
        group("IINST", "002", None),
        group("IMAX", "030", None),
        group("PAPP", "00460", None),
        group("HHPHC", "A", None),
        group("MOTDETAT", "000000", None),
    ]);
    frame(groups.iter().map(Vec::as_slice))
}

struct Meter {
    all: Option<Vec<Vec<u8>>>,
    played: Option<Packets>,
    optarif: String,
    made: u32,
    noise: Vec<u8>,
    stream: Stream,
    period: u32,
    burst: Option<u32>,
    scan_burst: Option<u32>,
    left: Option<u32>,
    marks: bool,
    quiet: u32,
    sent: u32,
    eot: Option<u32>,
    late: u32,
    wait: u32,
}

impl Meter {
    fn send_frame(&mut self, port: &mut Port<'_>) {
        port.send(&self.noise);
        let mut bytes = match &mut self.played {
            Some(played) => played.take(),
            None => default_frame(&self.optarif, self.made),
        };
        self.made = self.made.saturating_add(1);
        if self.eot.is_some_and(|n| self.made.is_multiple_of(n)) {
            bytes.truncate(bytes.len() / 2);
            bytes.push(EOT);
        }
        port.send(&bytes);
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        port.violation(format!("{} bytes written to a meter that only sends", data.len()));
    }

    fn changed(&mut self, event: LineEvent, _port: &mut Port<'_>) {
        if event.opened() {
            let first = self.scan_burst.is_some_and(|n| self.sent < n);
            self.left = if first { self.scan_burst.map(|n| n - self.sent) } else { self.burst };
            self.marks = !first;
            self.wait = if first { 0 } else { self.late };
            self.stream = Stream::new(self.period, true).with_burst(self.left);
            self.quiet = 0;
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if self.wait > 0 {
            self.wait -= 1;
        } else if self.stream.poll() {
            self.sent = self.sent.saturating_add(1);
            self.send_frame(port);
            if let Some(left) = &mut self.left {
                *left = left.saturating_sub(1);
            }
        } else if self.left == Some(0) && self.marks {
            // A period after the last frame, so that the tool has read it before
            // an event lands: bytes just delivered are lost to an unplug's
            // hangup, or not, as the pace has it.
            self.quiet += 1;
            if self.quiet >= self.period {
                port.mark();
                self.left = None;
                self.marks = false;
            }
        }
    }

    fn reset(&mut self) {
        self.played = self.all.clone().map(Packets::new);
        self.made = 0;
        self.stream = Stream::new(self.period, false);
        self.left = None;
        self.marks = false;
        self.quiet = 0;
        self.sent = 0;
        self.wait = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{DeviceModel, Reply};
    use crate::shells::cdc::host::Host;

    /// The meter of `options` with a tool's port open on the line.
    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let cdc = function(&o).unwrap();
        Host::open(cdc, Line::parse("1200/7e1").unwrap(), true, true)
    }

    /// Makes `n` offers with nothing queued, and gives what came of them. A frame
    /// over 128 bytes takes further offers, which count for nothing.
    fn offers(h: &mut Host<Meter>, n: u32) -> Vec<u8> {
        let mut got = Vec::new();
        let mut ticks = 0;
        let mut more = false;
        while ticks < n || more {
            if let Reply::Data(d) = h.offer() {
                ticks += u32::from(!more);
                more = d.len() == 128;
                got.extend(d);
            } else {
                ticks += 1;
                more = false;
            }
        }
        got
    }

    #[test]
    fn the_control_is_the_specification_s_mode_1() {
        // By hand: "IINST 002", 0x49+0x49+0x4e+0x53+0x54 = 0x187, +0x20 = 0x1a7,
        // +0x30+0x30+0x32 = 0x239, low six bits 0x39, +0x20 = 0x59.
        assert_eq!(control(b"IINST", b"002"), 0x59);
        // "PTEC HP..": a sum whose low six bits are 0, so a control of a space,
        // which the specification's range allows.
        assert_eq!(control(b"PTEC", b"HP.."), b' ');
    }

    #[test]
    fn a_group_and_a_frame_are_as_on_the_wire() {
        assert_eq!(group("HHPHC", "A", None), b"\nHHPHC A ,\r");
        assert_eq!(group("HHPHC", "A", Some(0x21)), b"\nHHPHC A !\r");
        assert_eq!(frame([b"xy".as_slice()]), b"\x02xy\x03");
    }

    #[test]
    fn a_frame_option_builds_groups() {
        assert_eq!(parse_frames("=0a0b").unwrap()[0], [STX, 0x0a, 0x0b, ETX]);
        let frames = parse_frames("HHPHC:A+HHPHC:A:21,X:").unwrap();
        assert_eq!(frames[0], b"\x02\nHHPHC A ,\r\nHHPHC A !\r\x03");
        assert_eq!(frames[1], b"\x02\nX  X\r\x03");
        for bad in ["", "X", ":1", "X:1:zz", "X:1:21:1", "=zz", "="] {
            assert!(parse_frames(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn each_option_has_its_groups() {
        let labels = |optarif: &str| {
            let f = default_frame(optarif, 0);
            String::from_utf8(f).unwrap().split('\n').skip(1).map(|g| g.split(' ').next().unwrap().to_owned()).collect::<Vec<_>>().join(" ")
        };
        assert_eq!(labels("BASE"), "ADCO OPTARIF ISOUSC BASE PTEC IINST IMAX PAPP HHPHC MOTDETAT");
        assert_eq!(labels("HC.."), "ADCO OPTARIF ISOUSC HCHC HCHP PTEC IINST IMAX PAPP HHPHC MOTDETAT");
        assert_eq!(labels("EJP."), "ADCO OPTARIF ISOUSC EJPHN EJPHPM PTEC IINST IMAX PAPP HHPHC MOTDETAT");
        assert_eq!(
            labels("BBR("),
            "ADCO OPTARIF ISOUSC BBRHCJB BBRHPJB BBRHCJW BBRHPJW BBRHCJR BBRHPJR PTEC DEMAIN IINST \
             IMAX PAPP HHPHC MOTDETAT"
        );
    }

    #[test]
    fn a_stream_sends_a_frame_per_period_and_its_index_moves() {
        let mut h = meter(&[("period", "4")]);
        let one = offers(&mut h, 4);
        assert_eq!(one, default_frame("BASE", 0));
        let two = offers(&mut h, 4);
        assert_eq!(two, default_frame("BASE", 1));
        assert_ne!(one, two);
        assert!(h.v.is_empty(), "{:?}", h.v);
    }

    #[test]
    fn a_burst_stops_and_marks_a_period_later() {
        let mut h = meter(&[("period", "4"), ("burst", "1")]);
        assert_ne!(offers(&mut h, 4), [0u8; 0]);
        assert!(offers(&mut h, 8).is_empty(), "silence");
        assert!(h.cdc.mark(), "the event point");
    }

    #[test]
    fn a_scan_burst_is_for_a_scan_and_a_burst_follows() {
        let mut h = meter(&[("period", "4"), ("scan-burst", "2"), ("burst", "0")]);
        assert_eq!(offers(&mut h, 16).len(), 2 * default_frame("BASE", 0).len());
        assert!(!h.cdc.mark(), "no mark on a scan");
        h.set_lines(true, true);
        assert!(offers(&mut h, 16).is_empty(), "burst=0 after it");
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_second_open_within_a_scan_burst_does_not_start_it_over() {
        let mut h = meter(&[("period", "4"), ("scan-burst", "2"), ("burst", "0")]);
        assert_eq!(offers(&mut h, 4).len(), default_frame("BASE", 0).len());
        h.set_lines(true, true);
        assert_eq!(offers(&mut h, 16).len(), default_frame("BASE", 1).len(), "one left");
    }

    #[test]
    fn late_holds_the_first_frame_of_an_open_that_is_no_scans() {
        let o = [("period", "4"), ("scan-burst", "1"), ("late", "8"), ("frame", "A:1")];
        let mut h = meter(&o);
        assert_ne!(offers(&mut h, 4), [0u8; 0], "a scan's is not held");
        h.set_lines(true, true);
        assert_eq!(offers(&mut h, 8), [0u8; 0], "held");
        assert_eq!(offers(&mut h, 3), [0u8; 0], "and a period");
        assert_ne!(offers(&mut h, 1), [0u8; 0]);
    }

    #[test]
    fn played_frames_come_in_turn_and_start_over_at_a_reset() {
        let mut h = meter(&[("period", "4"), ("frame", "A:1,B:2")]);
        let a = parse_frames("A:1").unwrap().remove(0);
        let b = parse_frames("B:2").unwrap().remove(0);
        assert_eq!(offers(&mut h, 4), a);
        assert_eq!(offers(&mut h, 4), b);
        assert_eq!(offers(&mut h, 4), a);
    }

    #[test]
    fn packet_noise_and_eot() {
        let mut h = meter(&[("period", "4"), ("packet", "0a0b"), ("noise", "3f")]);
        assert_eq!(offers(&mut h, 4), [0x3f, 0x0a, 0x0b]);
        let mut h = meter(&[("period", "4"), ("frame", "A:1"), ("eot", "2")]);
        let whole = parse_frames("A:1").unwrap().remove(0);
        assert_eq!(offers(&mut h, 4), whole);
        let cut = offers(&mut h, 4);
        assert_eq!(cut.len(), whole.len() / 2 + 1);
        assert_eq!(cut.last(), Some(&EOT));
        assert_eq!(offers(&mut h, 4), whole, "a new frame after it");
    }

    #[test]
    fn nothing_passes_on_a_wrong_line() {
        let o = vec![("period".to_owned(), "4".to_owned())];
        let cdc = function(&o).unwrap();
        let mut h = Host::open(cdc, Line::parse("1200/8n1").unwrap(), true, true);
        assert_eq!(offers(&mut h, 8), [0u8; 0]);
        h.set_line(Line::parse("1200/7e1").unwrap());
        assert_ne!(offers(&mut h, 8), [0u8; 0]);
    }

    #[test]
    fn a_byte_written_is_flagged() {
        let mut h = meter(&[]);
        h.write(b"?");
        let said = h.v.drain();
        assert_eq!(said.len(), 1, "{said:?}");
        assert!(said[0].contains("only sends"), "{said:?}");
    }

    #[test]
    fn options_are_checked() {
        for (k, v) in [("period", "3"), ("nope", "1"), ("frame", "X"), ("packet", "zz"), ("eot", "0")] {
            assert!(function(&[(k.to_owned(), v.to_owned())]).is_err(), "{k}={v}");
        }
        let native = function(&[("uart".to_owned(), "native".to_owned())]);
        assert!(native.is_ok());
    }
}
