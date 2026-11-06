// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The MIC 98581 temperature logger and MIC 98583 temperature and humidity
//! logger on the CDC-ACM shell.
//!
//! Told to, a logger sends a reading now and then as ASCII, with no ask per
//! reading: `v ttt\r` on a 98581, `v ttt hhh\r` on a 98583, each field three
//! digits.
//!
//! Sources, and how sure they are:
//! - sigrok's wiki, MIC 98583: a PL2303 behind a line of 38400/8n2, and `I\r`
//!   answered with `i A 98583 V1.0\r`. MIC 98581: an MSP430F4152 and a
//!   PL2303HX, "almost identical" to the 98583 in its protocol. So the 98581's
//!   answer to `I` here is an inference from that; no driver asks.
//! - libsigrok's driver for the frames, and for the one command it sends,
//!   `S 1 M 2 32 3\r`, which it calls setting real-time mode. Nothing says what
//!   its numbers mean, how often the logger then sends, whether it answers
//!   the command, or whether it stays in that mode across a host's close.
//! - The manual (photo `File:Mic_98581_manual_specs.jpg` on the wiki) and an
//!   archived 2013 vendor page: -40.0 to 85.0 degrees, RH 0.1 to 99.9 %, a sample
//!   interval of 1 s to 6 h set by the vendor's program, and a Fahrenheit range. A
//!   display photo shows `25.7 °C`. So the third digit is taken for tenths, by
//!   inference: those ranges fit 001 to 999 and 850, and no frame was ever recorded.
//!   How a logger writes a temperature below 0 or sends Fahrenheit is not known,
//!   and this model has neither.
//! - No firmware source, no recording, no unit on a desk.
//!
//! Emulation artifacts:
//! - A real unit has a PL2303, which the `pl2303` driver would bind by id: here
//!   it is CDC-ACM with neutral ids, and shows the same line settings. On a
//!   misset line nothing passes ([`Cdc::behind_uart`]), where a real one would
//!   give garbage. Two stop bits against one cannot be told apart that way.
//! - The rate is a count of read offers, as nobody has said what the real one
//!   is: `period=` between readings, and `late=` from the command to the first.
//! - The stream goes on across a close and stands still while it is closed.
//! - The default readings have 0 as their third digit, so that they say nothing
//!   of how a tool treats tenths: `packet=` is for that.
//! - A packet goes out whole, however slow the line; the tests of the port hold
//!   a byte-paced fake for that.
//! - Of a real logger's other commands (stored logs, its settings) the model has
//!   none and flags them: no driver path sends them. A real unit has them, so a
//!   tool that means them should not read this as a refusal.
//!
//! Options, besides the shell's ([`Faults`]):
//! - `device=98581|98583`, the default being the 98583;
//! - `packet=HEX[,HEX...]`, the bytes of each reading, played in turn, and
//!   anything at all: they need not be a frame, or one frame;
//! - `noise=HEX`, bytes sent before each reading;
//! - `ack=HEX`, bytes the real-time command is answered with (none by default);
//! - `uart=LINE|native`, the line behind the cable, instead of 38400/8n2;
//! - `period=N`, offers between readings (4 to 1000, default 50);
//! - `late=N`, offers from the command to the first reading (1 to 1000, default
//!   5), so that a command takes effect late;
//! - `burst=N`, readings per power on, then silence and a mark a period later; 0
//!   never sends;
//! - `running=1`, in real-time mode from power on, as if an earlier program had
//!   left it so. `late=` counts from the last open until the first reading, so
//!   that a scan's short open uses none of a burst; that is an artifact.
//!
//! The pedantic checks: a command is `I` or `S 1 M 2 32 3` and ends in CR within
//! [`MAX_COMMAND`] bytes. What is on the line at a close is not checked, as a
//! logger does not see a host go.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::{self as cable, Packets};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "mic-985xx";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "MIC 98581 and 98583 loggers on a CDC-ACM cable; device=98581|98583, \
                         packet=HEX[,HEX...], noise=HEX, ack=HEX, uart=LINE|native, period=N, \
                         late=N, burst=N, running=1, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

/// The most a command may hold, its CR aside; longer is no command.
const MAX_COMMAND: usize = 32;
/// The command that starts the readings, as a driver sends it, without its CR.
const REALTIME: &[u8] = b"S 1 M 2 32 3";

/// A logger from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Logger>> {
    let mut faults = Faults::default();
    let mut kind = Kind::M98583;
    let mut packets = None;
    let mut noise = Vec::new();
    let mut ack = Vec::new();
    let mut uart = None;
    let mut period = 50;
    let mut late = 5;
    let mut burst = None;
    let mut running = false;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "device" => {
                kind = match v.as_str() {
                    "98581" => Kind::M98581,
                    "98583" => Kind::M98583,
                    _ => return Err(Error::Invalid(format!("device={v}: 98581 or 98583"))),
                };
            }
            "packet" => packets = Some(cable::parse_packets(v, 1, 128)?),
            "noise" => noise = cable::parse_packets(v, 1, 128)?.remove(0),
            "ack" => ack = cable::parse_packets(v, 1, 128)?.remove(0),
            "uart" => {
                uart = Some(if v == "native" { None } else { Some(Line::parse(v)?) });
            }
            "period" => period = cable::parse_number(k, v, 4..=1000)?,
            "late" => late = cable::parse_number(k, v, 1..=1000)?,
            "burst" => burst = cable::parse_burst(v)?,
            "running" => running = cable::parse_number(k, v, 0..=1)? == 1,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let all = packets.unwrap_or_else(|| vec![kind.default_packet().to_vec()]);
    let logger = Logger {
        kind,
        packets: Packets::new(all.clone()),
        all,
        noise,
        ack,
        period,
        late,
        burst,
        running,
        realtime: running,
        sent: false,
        due: late,
        left: burst,
        quiet: 0,
        command: Vec::new(),
    };
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "MIC 985xx logger");
    let cdc = Cdc::new(NAME, &id, faults, logger);
    Ok(match uart {
        Some(None) => cdc,
        Some(Some(line)) => cdc.behind_uart(line),
        None => cdc.behind_uart(Line::parse("38400/8n2").expect("the table's line parses")),
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    M98581,
    M98583,
}

impl Kind {
    fn default_packet(self) -> &'static [u8] {
        match self {
            Kind::M98581 => b"v 230\r",
            Kind::M98583 => b"v 230 610\r",
        }
    }

    fn identity(self) -> &'static [u8] {
        match self {
            Kind::M98581 => b"i A 98581 V1.0\r",
            Kind::M98583 => b"i A 98583 V1.0\r",
        }
    }
}

struct Logger {
    kind: Kind,
    all: Vec<Vec<u8>>,
    packets: Packets,
    noise: Vec<u8>,
    ack: Vec<u8>,
    period: u32,
    late: u32,
    burst: Option<u32>,
    running: bool,
    /// Sending readings.
    realtime: bool,
    /// One has gone out since power on.
    sent: bool,
    /// Offers until the next reading.
    due: u32,
    /// Readings left of a burst.
    left: Option<u32>,
    /// Offers since a burst's last reading.
    quiet: u32,
    /// What came of a command so far.
    command: Vec<u8>,
}

impl Logger {
    fn command(&mut self, port: &mut Port<'_>) {
        if self.command == b"I" {
            port.send(self.kind.identity());
        } else if self.command == REALTIME {
            port.send(&self.ack);
            // A logger already at it has nothing to start.
            if !self.realtime {
                self.realtime = true;
                self.due = self.late;
            }
        } else {
            port.violation(format!(
                "command {:?}: a logger's driver sends only {:?}",
                String::from_utf8_lossy(&self.command),
                String::from_utf8_lossy(REALTIME)
            ));
        }
        self.command.clear();
    }
}

impl Serial for Logger {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte == b'\r' {
                self.command(port);
            } else if self.command.len() >= MAX_COMMAND {
                port.violation(format!("no CR in {MAX_COMMAND} bytes of a command"));
                self.command.clear();
            } else {
                self.command.push(byte);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, _port: &mut Port<'_>) {
        if event.opened() && self.running && !self.sent {
            self.due = self.late;
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if !self.realtime {
            return;
        }
        if self.left == Some(0) {
            // A period after the last reading, so that the tool has read it
            // before an event lands: bytes just delivered are lost to an
            // unplug's hangup, or not, as the pace has it.
            self.quiet = self.quiet.saturating_add(1);
            if self.quiet == self.period {
                port.mark();
            }
            return;
        }
        self.due -= 1;
        if self.due == 0 {
            self.due = self.period;
            self.sent = true;
            port.send(&self.noise);
            port.send(&self.packets.take());
            if let Some(left) = &mut self.left {
                *left -= 1;
            }
        }
    }

    fn reset(&mut self) {
        self.packets = Packets::new(self.all.clone());
        self.realtime = self.running;
        self.sent = false;
        self.due = self.late;
        self.left = self.burst;
        self.quiet = 0;
        self.command.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    /// The logger of `options` with a tool's port open on its line.
    fn logger(options: &[(&str, &str)]) -> Host<Logger> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let cdc = function(&o).unwrap();
        Host::open(cdc, Line::parse("38400/8n2").unwrap(), true, true)
    }

    /// Offers `n` reads: what came, read by read.
    fn offers(h: &mut Host<Logger>, n: u32) -> Vec<Vec<u8>> {
        (0..n)
            .filter_map(|_| match h.offer() {
                Reply::Data(d) => Some(d),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn says_nothing_until_it_is_told() {
        let mut h = logger(&[]);
        assert_eq!(offers(&mut h, 500), Vec::<Vec<u8>>::new());
        assert!(h.v.is_empty());
    }

    #[test]
    fn the_command_starts_readings_of_the_device() {
        for (device, frame) in [("98581", &b"v 230\r"[..]), ("98583", &b"v 230 610\r"[..])] {
            let mut h = logger(&[("device", device), ("late", "2"), ("period", "4")]);
            h.write(b"S 1 M 2 32 3\r");
            assert_eq!(h.offer(), Reply::Nak);
            assert_eq!(h.offer(), Reply::Data(frame.to_vec()));
            for _ in 0..3 {
                assert_eq!(h.offer(), Reply::Nak);
            }
            assert_eq!(h.offer(), Reply::Data(frame.to_vec()));
            assert!(h.v.is_empty());
        }
    }

    #[test]
    fn a_default_reading_has_no_tenths() {
        for k in [Kind::M98581, Kind::M98583] {
            assert_eq!(k.default_packet()[4], b'0');
        }
        assert_eq!(Kind::M98583.default_packet()[8], b'0');
    }

    #[test]
    fn a_command_cut_in_two_counts_as_one() {
        let mut h = logger(&[("late", "1")]);
        h.write(b"S 1 M ");
        assert_eq!(offers(&mut h, 3), Vec::<Vec<u8>>::new());
        h.write(b"2 32 3\r");
        assert_eq!(offers(&mut h, 1).len(), 1);
        assert!(h.v.is_empty());
    }

    #[test]
    fn identify_answers_and_starts_nothing() {
        let mut h = logger(&[]);
        h.write(b"I\r");
        assert_eq!(h.read(), b"i A 98583 V1.0\r");
        assert_eq!(offers(&mut h, 100), Vec::<Vec<u8>>::new());
        let mut h = logger(&[("device", "98581")]);
        h.write(b"I\r");
        assert_eq!(h.read(), b"i A 98581 V1.0\r");
        assert!(h.v.is_empty());
    }

    #[test]
    fn other_commands_are_flagged() {
        let mut h = logger(&[]);
        h.write(b"S 1 M 2 32 4\r");
        assert_eq!(h.v.drain().len(), 1);
        assert_eq!(offers(&mut h, 100), Vec::<Vec<u8>>::new());
        h.write(b"\r");
        assert_eq!(h.v.drain().len(), 1, "an empty one too");
    }

    #[test]
    fn a_long_command_is_flagged_and_bounded() {
        let mut h = logger(&[]);
        h.write(&[b'x'; 33]);
        assert_eq!(h.v.drain().len(), 1);
        assert!(h.cdc.model.command.len() <= MAX_COMMAND);
    }

    #[test]
    fn ack_answers_each_command_and_noise_goes_before_each_reading() {
        let mut h = logger(&[("ack", "3f3f"), ("noise", "21"), ("late", "4"), ("period", "4")]);
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 1), vec![vec![0x3f, 0x3f]]);
        assert_eq!(offers(&mut h, 4), vec![b"!v 230 610\r".to_vec()]);
    }

    #[test]
    fn packets_are_played_in_turn_and_may_be_anything() {
        let mut h = logger(&[("packet", "76,2031"), ("late", "1"), ("period", "4")]);
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 9), vec![vec![0x76], vec![0x20, 0x31], vec![0x76]]);
    }

    #[test]
    fn a_burst_ends_in_silence_and_the_point() {
        let mut h = logger(&[("burst", "1"), ("late", "1"), ("period", "4")]);
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 1).len(), 1);
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
            assert!(!h.cdc.mark(), "a period later");
        }
        assert_eq!(h.offer(), Reply::Nak);
        assert!(h.cdc.mark());
        assert_eq!(offers(&mut h, 20), Vec::<Vec<u8>>::new());
        assert!(!h.cdc.mark(), "once");
    }

    #[test]
    fn a_burst_of_nothing_never_sends_and_still_marks() {
        let mut h = logger(&[("burst", "0"), ("period", "4")]);
        assert_eq!(offers(&mut h, 30), Vec::<Vec<u8>>::new());
        assert!(!h.cdc.mark(), "not before it is told");
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 30), Vec::<Vec<u8>>::new());
        assert!(h.cdc.mark());
    }

    #[test]
    fn late_holds_the_first_reading() {
        let mut h = logger(&[("late", "7")]);
        h.write(b"S 1 M 2 32 3\r");
        for _ in 0..6 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert!(matches!(h.offer(), Reply::Data(_)));
    }

    #[test]
    fn running_needs_no_command_and_a_command_does_not_restart_it() {
        let mut h = logger(&[("running", "1"), ("late", "2"), ("period", "4")]);
        assert_eq!(h.offer(), Reply::Nak);
        assert!(matches!(h.offer(), Reply::Data(_)));
        assert_eq!(h.offer(), Reply::Nak);
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(h.offer(), Reply::Nak);
        assert_eq!(h.offer(), Reply::Nak);
        assert!(matches!(h.offer(), Reply::Data(_)), "a period from the last");
        assert!(h.v.is_empty());
    }

    #[test]
    fn running_counts_late_from_the_last_open_until_a_reading_went() {
        let mut h = logger(&[("running", "1"), ("late", "2"), ("period", "4")]);
        assert_eq!(h.offer(), Reply::Nak);
        h.set_lines(true, true);
        assert_eq!(h.offer(), Reply::Nak);
        assert!(matches!(h.offer(), Reply::Data(_)));
        h.set_lines(true, true);
        for _ in 0..3 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert!(matches!(h.offer(), Reply::Data(_)), "a period from the last");
    }

    #[test]
    fn a_close_does_not_stop_it() {
        let mut h = logger(&[("late", "1"), ("period", "4")]);
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 1).len(), 1);
        h.set_lines(false, false);
        h.set_lines(true, true);
        assert_eq!(offers(&mut h, 4).len(), 1);
    }

    #[test]
    fn a_misset_line_hears_nothing_and_says_nothing() {
        let mut h = logger(&[("late", "1")]);
        h.set_line(Line::parse("9600/8n2").unwrap());
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 100), Vec::<Vec<u8>>::new());
        h.set_line(Line::parse("38400/8n2").unwrap());
        assert_eq!(offers(&mut h, 100), Vec::<Vec<u8>>::new(), "the command was lost");
        assert!(h.v.is_empty());
    }

    #[test]
    fn one_stop_bit_passes_as_nothing_tells_them_apart() {
        let mut h = logger(&[("late", "1")]);
        h.set_line(Line::parse("38400/8n1").unwrap());
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 1).len(), 1);
    }

    #[test]
    fn uart_moves_the_line_or_takes_it_away() {
        let mut h = logger(&[("uart", "9600/8n1"), ("late", "1")]);
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 10), Vec::<Vec<u8>>::new());
        h.set_line(Line::parse("9600/8n1").unwrap());
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 1).len(), 1);
        let mut h = logger(&[("uart", "native"), ("late", "1")]);
        h.set_line(Line::parse("115200/8n1").unwrap());
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 1).len(), 1);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = logger(&[("burst", "1"), ("late", "1"), ("period", "4")]);
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 10).len(), 1);
        assert!(h.cdc.mark());
        h.write(b"S 1");
        h.cdc.model.reset();
        assert!(!h.cdc.model.realtime);
        assert_eq!(h.cdc.model.command, b"");
        assert_eq!(h.cdc.model.left, Some(1));
        h.write(b"S 1 M 2 32 3\r");
        assert_eq!(offers(&mut h, 10).len(), 1);
        assert!(h.cdc.mark());
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("device", "98582"),
            ("packet", "0"),
            ("period", "3"),
            ("late", "0"),
            ("running", "2"),
            ("uart", "fast"),
            ("burst", "1001"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("stall", "line"), ("noise", "00ff"), ("ack", "00")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
