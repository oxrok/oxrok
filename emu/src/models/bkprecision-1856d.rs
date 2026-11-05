// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A B&K Precision 1856D frequency counter on the CDC-ACM shell: two letters
//! and a CR to it, fifteen bytes back.
//!
//! The device, and how sure this is:
//!
//! - The line and the commands: the 1856D manual, section 3-7 (pages 17 and 18).
//!   9600 baud, 8n1. A command is a letter, a parameter and CR: `H` hold, `G` gate
//!   (0 to 3 for 0.01, 0.1, 1 and 10 s), `D` data request ("don't care" for its
//!   parameter), `F` function, `R` remote. The function table lists `FA NC FC
//!   PERIOD TOTAL RPM NC NC` under 0 to 7, so `F0` is input A and `F2` input C.
//! - An answer: the same page, 10 bytes of data "include dp", 4 of unit, and CR.
//! - That the unit reads `MHz `, `kHz ` and ` Hz `: what libsigrok's driver expects
//!   of the fourth byte and those after it, and the display of page 11, which has
//!   only M and K. No recording was found. ` Hz ` itself is a guess that fits it.
//! - The number's width, right justified: a guess. Its digits follow the manual's
//!   tables 1 and 2 with the internal time base: 5, 6, 7 and 8 for the four gates.
//! - The empty message, for a counter with nothing to show: only what that driver
//!   tests for, spaces and at most one `0`, and a comment there. Its layout here is
//!   a guess.
//! - When a reading comes: the manual does not say. The driver waits 100 ms for
//!   one, whatever the gate, and says the device rarely stops answering. So it
//!   answers at once, with a reading whatever the gate, unless `settle=` asks for
//!   empty messages first.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id, where the real one is a
//!   9 pin RS-232 port: only the byte stream and the line settings are the device's.
//! - It has no input: `a=` and `c=` say what each one reads, all the time.
//! - It answers at once, so no real time passes in a gate. `late=` holds an answer
//!   back. A request meanwhile is lost, which nothing says of the device.
//! - `replies=` plays frames no device was seen sending, for what a parser does.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `a=HZ`, `c=HZ`: what each input reads, 0 being the empty message;
//! - `replies=HEX,HEX`: what each request gets instead, in turn and then again,
//!   with an empty one being no answer;
//! - `settle=N`: the empty message for the first N requests after a `G`;
//! - `answers=N`: requests answered per power on, then silence and the event point
//!   at the next offer of a read;
//! - `late=N:T`: the Nth request answered after T offers of a read.
//!
//! The pedantic checks: only what the driver sends, `G0` to `G3`, `D0`, `F0` and
//! `F2`, each ended by CR, and nothing left of a command when a tool opens. `H`, `R`
//! and the other functions are real commands that no path sends: they are flagged
//! and do nothing.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "bkprecision-1856d";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A B&K Precision 1856D frequency counter on CDC-ACM; a=, c=, replies=, \
                         settle=, answers=, late=, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

/// More than the longest command; what is longer is flagged.
const LONGEST: usize = 8;
/// More than a frame needs, for what `replies=` may send.
const LONGEST_REPLY: usize = 64;
/// The most a frequency may be, above the manual's 3.7 GHz.
const HZ_MAX: u32 = 4_000_000_000;
/// Digits on the display per gate: tables 1 and 2, the internal time base.
const DIGITS: [u32; 4] = [5, 6, 7, 8];
/// The empty message.
const EMPTY: &[u8; 15] = b"         0    \r";

/// A counter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Counter>> {
    let mut faults = Faults::default();
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "a" => config.hz_a = parse_number(k, v, 0..=HZ_MAX)?,
            "c" => config.hz_c = parse_number(k, v, 0..=HZ_MAX)?,
            "replies" => config.replies = Some(Packets::new(parse_hex_list(k, v, 0, LONGEST_REPLY)?)),
            "settle" => config.settle = parse_number(k, v, 0..=100_000)?,
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "late" => {
                let (nth, offers) = v
                    .split_once(':')
                    .ok_or_else(|| Error::Invalid(format!("late={v}: as in 2:40")))?;
                config.late = Some((
                    parse_number(k, nth, 1..=100_000)?,
                    parse_number(k, offers, 1..=100_000)?,
                ));
            }
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "1856D frequency counter");
    Ok(Cdc::new(NAME, &id, faults, Counter::new(config)).behind_uart(Line::parse("9600/8n1")?))
}

#[derive(Clone, Debug)]
struct Config {
    hz_a: u32,
    hz_c: u32,
    replies: Option<Packets>,
    settle: u32,
    answers: Option<u32>,
    /// Which request, from 1, and for how many offers of a read.
    late: Option<(u32, u32)>,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            hz_a: 10_000_000,
            hz_c: 1_000_000_000,
            replies: None,
            settle: 0,
            answers: None,
            late: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Input {
    A,
    C,
}

struct Counter {
    config: Config,
    replies: Option<Packets>,
    input: Input,
    gate: usize,
    settling: u32,
    requests: u32,
    answered: u32,
    pending: Vec<u8>,
    held: Option<(u32, Vec<u8>)>,
    mark_due: bool,
}

impl Counter {
    fn new(config: Config) -> Counter {
        Counter {
            replies: config.replies.clone(),
            config,
            input: Input::A,
            gate: 0,
            settling: 0,
            requests: 0,
            answered: 0,
            pending: Vec::new(),
            held: None,
            mark_due: false,
        }
    }

    fn frame(&mut self) -> Vec<u8> {
        if let Some(replies) = &mut self.replies {
            return replies.take();
        }
        if self.settling > 0 {
            self.settling -= 1;
            return EMPTY.to_vec();
        }
        let hz = match self.input {
            Input::A => self.config.hz_a,
            Input::C => self.config.hz_c,
        };
        reading(hz, DIGITS[self.gate])
    }

    fn request(&mut self, port: &mut Port<'_>) {
        if self.held.is_some() {
            return;
        }
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            self.mark_due = true;
            return;
        }
        self.answered += 1;
        self.requests += 1;
        let frame = self.frame();
        if let Some((nth, offers)) = self.config.late
            && nth == self.requests
        {
            self.held = Some((offers, frame));
            return;
        }
        port.send(&frame);
    }

    fn command(&mut self, command: &[u8], port: &mut Port<'_>) {
        match command {
            b"G0" | b"G1" | b"G2" | b"G3" => {
                self.gate = usize::from(command[1] - b'0');
                self.settling = self.config.settle;
            }
            b"D0" => self.request(port),
            b"F0" => self.input = Input::A,
            b"F2" => self.input = Input::C,
            other => {
                port.violation(format!("'{}': nothing the driver sends", show(other)));
            }
        }
    }
}

/// A reading of `hz` with `digits` on the display. The tables end at 3.7 GHz in
/// MHz, so at least one decimal is left, and the point is always there.
fn reading(hz: u32, digits: u32) -> Vec<u8> {
    if hz == 0 {
        return EMPTY.to_vec();
    }
    let (divisor, places, prefix) = match hz {
        1_000_000.. => (1_000_000, 6, 'M'),
        1000.. => (1000, 3, 'k'),
        _ => (1, 0, ' '),
    };
    let whole = hz / divisor;
    let decimals = digits.saturating_sub(whole.ilog10() + 1).max(1) as usize;
    let part = if places == 0 { String::new() } else { format!("{:0places$}", hz % divisor) };
    let number = format!("{whole}.{part:0<decimals$.decimals$}");
    format!("{number:>10}{prefix}Hz \r").into_bytes()
}

fn show(bytes: &[u8]) -> String {
    bytes.escape_ascii().to_string()
}

impl Serial for Counter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte == b'\r' {
                let command = std::mem::take(&mut self.pending);
                self.command(&command, port);
            } else if self.pending.len() >= LONGEST {
                port.violation(format!("'{}': a command too long", show(&self.pending)));
                self.pending.clear();
            } else {
                self.pending.push(byte);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("'{}': a command left unfinished", show(&self.pending)));
            self.pending.clear();
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if std::mem::take(&mut self.mark_due) {
            port.mark();
        }
        if let Some((offers, frame)) = &mut self.held {
            *offers -= 1;
            if *offers == 0 {
                port.send(frame);
                self.held = None;
            }
        }
    }

    fn reset(&mut self) {
        *self = Counter::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn counter(options: &[(&str, &str)]) -> Host<Counter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, false)
    }

    fn violations(h: &mut Host<Counter>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn a_frame_is_ten_bytes_four_of_unit_and_cr() {
        for (hz, digits, want) in [
            (10_000_000, 5, " 10.000MHz \r"),
            (10_000_000, 8, " 10.000000MHz \r"),
            (1_000_000_000, 5, "1000.0MHz \r"),
            (3_500_000_000, 8, "3500.0000MHz \r"),
            (12_345_678, 6, "12.3456MHz \r"),
            (1000, 7, "1.000000kHz \r"),
            (999, 5, "999.00 Hz \r"),
            (1, 8, "1.0000000 Hz \r"),
            (1234, 6, "1.23400kHz \r"),
        ] {
            let frame = String::from_utf8(reading(hz, digits)).unwrap();
            assert_eq!(frame.len(), 15, "{frame}");
            assert_eq!(frame.trim_start(), want.trim_start());
        }
        assert_eq!(reading(0, 5), EMPTY);
    }

    #[test]
    fn the_input_and_the_gate_choose_the_reading() {
        let mut h = counter(&[("a", "1000"), ("c", "2000000000")]);
        h.write(b"F0\rG0\rD0\r");
        assert_eq!(h.read(), b"    1.0000kHz \r");
        h.write(b"G3\rD0\r");
        assert_eq!(h.read(), b" 1.0000000kHz \r");
        h.write(b"F2\rD0\r");
        assert_eq!(h.read(), b" 2000.0000MHz \r");
        h.write(b"G1\rD0\r");
        assert_eq!(h.read(), b"   2000.00MHz \r");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_the_driver_does_not_send_is_flagged_and_does_nothing() {
        for bad in [
            &b"H0"[..],
            b"H1",
            b"H2",
            b"R0",
            b"R1",
            b"F1",
            b"F3",
            b"F7",
            b"G4",
            b"D1",
            b"D",
            b"G",
            b"",
            b"d0",
            b"xyz",
        ] {
            let mut h = counter(&[]);
            h.write(bad);
            h.write(b"\r");
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            assert_eq!(h.read(), b"", "{}", show(bad));
        }
    }

    #[test]
    fn the_commands_it_takes_are_not_flagged_in_any_cut() {
        let mut h = counter(&[]);
        for byte in b"F0\rG0\rG1\rG2\rG3\rF2\r" {
            h.write(&[*byte]);
        }
        assert_eq!(violations(&mut h), 0);
        assert_eq!(h.cdc.model.input, Input::C);
        assert_eq!(h.cdc.model.gate, 3);
    }

    #[test]
    fn replies_play_in_turn_with_an_empty_one_for_none() {
        let mut h = counter(&[("replies", "41,,4243")]);
        for want in [&b"A"[..], b"", b"BC", b"A"] {
            h.write(b"D0\r");
            assert_eq!(h.read(), want);
        }
    }

    #[test]
    fn a_gate_settles_through_empty_messages() {
        let mut h = counter(&[("settle", "2")]);
        h.write(b"G0\rD0\r");
        assert_eq!(h.read(), EMPTY);
        h.write(b"D0\r");
        assert_eq!(h.read(), EMPTY);
        h.write(b"D0\r");
        assert_eq!(h.read(), b"    10.000MHz \r");
        h.write(b"G0\rD0\r");
        assert_eq!(h.read(), EMPTY);
    }

    #[test]
    fn answers_runs_out_and_marks_the_point_at_an_offer() {
        let mut h = counter(&[("answers", "1")]);
        h.write(b"D0\r");
        assert_ne!(h.read(), b"");
        assert!(!h.cdc.mark());
        h.write(b"D0\r");
        assert!(!h.cdc.mark(), "not inside the write");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_late_answer_waits_for_its_offers_and_a_request_meanwhile_is_lost() {
        let mut h = counter(&[("late", "2:3"), ("a", "1000")]);
        h.write(b"D0\r");
        assert_ne!(h.read(), b"");
        h.write(b"D0\r");
        assert_eq!(h.read(), b"");
        h.write(b"D0\r");
        assert_eq!(h.read(), b"");
        let mut got = h.read();
        got.extend(h.read());
        assert_eq!(got, b"    1.0000kHz \r");
        assert_eq!(h.read(), b"");
        h.write(b"D0\r");
        assert_eq!(h.read(), b"    1.0000kHz \r");
        assert!(build(&[("late".to_owned(), "2".to_owned())]).is_err());
    }

    #[test]
    fn behind_its_uart_a_wrong_rate_hears_nothing() {
        let o = vec![];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("2400/8n1").unwrap(), true, true);
        h.write(b"D0\r");
        assert_eq!(h.read(), b"");
        h.set_line(Line::parse("9600/8n1").unwrap());
        h.write(b"D0\r");
        assert_ne!(h.read(), b"");
    }

    #[test]
    fn a_long_command_is_flagged_and_what_is_kept_stays_short() {
        let mut h = counter(&[]);
        h.write(&[b'x'; 4096]);
        assert!(h.cdc.model.pending.len() <= LONGEST);
        assert!(violations(&mut h) > 0);
    }

    #[test]
    fn a_command_left_unfinished_shows_at_the_next_open() {
        let mut h = counter(&[]);
        h.write(b"D");
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        h.write(b"D0\r");
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = counter(&[("answers", "1"), ("replies", "41,42")]);
        h.write(b"F2\rG3\rD0\r");
        assert_eq!(h.read(), b"A");
        h.cdc.model.reset();
        assert_eq!(h.cdc.model.input, Input::A);
        assert_eq!(h.cdc.model.gate, 0);
        h.write(b"D0\r");
        assert_eq!(h.read(), b"A");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("a", "x"),
            ("c", "4000000001"),
            ("settle", "-1"),
            ("late", "0:1"),
            ("late", "1:0"),
            ("replies", "4"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("a", "0"), ("replies", "41,"), ("late", "1:1")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
