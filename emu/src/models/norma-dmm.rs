// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Norma `DM9x0` bench multimeter (Siemens B102x) with its RS-232 interface
//! board, on the CDC-ACM shell. It answers two questions: `IDN?` and `STATUS?`.
//!
//! The device, and how sure this is:
//!
//! - The sigrok wiki's page "Norma DM950", from a person with the meter, gives
//!   DTR on and RTS off with RTS/CTS flow control, about 50 commands, and these
//!   replies: `1834 065 V1.06,IF V1.02` from a DM950, `1834 063 V1.04,IF V1.02`
//!   from a B1026, and `03030024913100400040` to `STATUS?`, 20 hex characters.
//!   It writes each reply ending in CR, and each command ending in LF.
//! - The rate, 4800 8n1, the commands' CR LF and the replies' CR LF are what
//!   libsigrok's driver uses or says in its comments, written with a DM950 and a
//!   B1026 in hand. That conflict with the wiki on the line end stays open, so
//!   `eol=` has each reading. No manual or recording was found. The wiki's link
//!   to set.nl no longer resolves; the Wayback Machine has the product page, with
//!   specifications and "NM 950 with interface, A 1834 06512", which is what the
//!   `1834 065` of `IDN?` is, and no protocol.
//! - What each digit of a status means: only that driver's comments. So this
//!   model plays a status as a string and gives its digits no meaning.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id, where the real one is
//!   a 5 pin printer cable and an adaptor: only the byte stream and the line
//!   settings are the device's. CTS cannot be shown.
//! - It answers at once, unless `late=` holds an answer back. The real interface
//!   takes up to 10 s to charge a capacitor when it lives off the RS-232 lines
//!   (the driver's comment); `idn=~;~;...` plays that.
//! - The meter would mark a value it sent before as a duplicate, at its own
//!   conversion rate. Here each `STATUS?` gets the next entry of the list, and a
//!   list has to spell a duplicate out.
//! - The answers past the wiki's are stand-ins for a meter in other functions and
//!   for a line gone wrong.
//!
//! An answer in `idn=` and `status=` is `TEXT`, sent with the line end, or
//! `TEXT~`, those bytes alone, so `~` alone is no answer. An option's value holds
//! no space, so `_` stands for one; and an IDN string holds a comma, so `;` is
//! what separates answers. They play in turn and then again.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `idn=`, `status=`, the lists;
//! - `eol=crlf|cr|lf`, the line end of what it sends;
//! - `uart=LINE`, its line settings;
//! - `answers=N`, commands answered per power on, then silence and the event
//!   point;
//! - `late=N:T`, the Nth `STATUS?` answered after T offers of a read.
//!
//! The pedantic checks: only `IDN?` and `STATUS?`, each ended by CR LF; nothing
//! left of a command when a tool opens. The meter's other commands are real, and
//! no path of the driver sends them, so they are flagged and get no answer, which
//! is itself a guess.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "norma-dmm";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Norma DM9x0 / Siemens B102x multimeter on CDC-ACM; idn=, status= \
                         (answers in turn), eol=, uart=, answers=, late=, and the shell's \
                         stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// The wiki's DM950.
const IDN: &str = "1834_065_V1.06,IF_V1.02";
/// The wiki's `STATUS?` example: 24.91 V DC, by the driver's reading.
const STATUS: &str = "03030024913100400040";
/// More than the longest command; what is longer is flagged.
const LONGEST: usize = 32;

/// A meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn answers_list(value: &str) -> Packets {
    Packets::new(value.split(';').map(|t| t.replace('_', " ").into_bytes()).collect())
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("4800/8n1")?;
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "idn" => config.idn = answers_list(v),
            "status" => config.status = answers_list(v),
            "uart" => uart = Line::parse(v)?,
            "eol" => {
                config.eol = match v.as_str() {
                    "cr" => b"\r",
                    "lf" => b"\n",
                    "crlf" => b"\r\n",
                    _ => return Err(Error::Invalid(format!("eol={v}: cr, lf or crlf"))),
                };
            }
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
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Norma DM9x0 multimeter");
    Ok(Cdc::new(NAME, &id, faults, Meter::new(config)).behind_uart(uart))
}

#[derive(Clone, Debug)]
struct Config {
    idn: Packets,
    status: Packets,
    eol: &'static [u8],
    answers: Option<u32>,
    /// Which `STATUS?`, from 1, and for how many offers of a read.
    late: Option<(u32, u32)>,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            idn: answers_list(IDN),
            status: answers_list(STATUS),
            eol: b"\r\n",
            answers: None,
            late: None,
        }
    }
}

struct Meter {
    config: Config,
    idn: Packets,
    status: Packets,
    /// What has come of a line so far.
    pending: Vec<u8>,
    /// A CR came, so the LF is due.
    after_cr: bool,
    answered: u32,
    status_asked: u32,
    /// An answer held back: the offers it still waits, and its bytes.
    held: Option<(u32, Vec<u8>)>,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter {
            idn: config.idn.clone(),
            status: config.status.clone(),
            pending: Vec::new(),
            after_cr: false,
            config,
            answered: 0,
            status_asked: 0,
            held: None,
        }
    }

    fn bytes_of(&self, answer: &[u8]) -> Vec<u8> {
        if let Some(raw) = answer.strip_suffix(b"~") {
            return raw.to_vec();
        }
        let mut out = answer.to_vec();
        out.extend_from_slice(self.config.eol);
        out
    }

    fn line(&mut self, line: &[u8], port: &mut Port<'_>) {
        if line != b"IDN?" && line != b"STATUS?" {
            port.violation(format!("'{}': nothing the driver sends", show(line)));
            return;
        }
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return;
        }
        self.answered += 1;
        if line == b"IDN?" {
            let answer = self.idn.take();
            port.send(&self.bytes_of(&answer));
            return;
        }
        self.status_asked += 1;
        let answer = self.status.take();
        let bytes = self.bytes_of(&answer);
        if let Some((nth, offers)) = self.config.late
            && nth == self.status_asked
        {
            self.held = Some((offers, bytes));
            return;
        }
        port.send(&bytes);
    }
}

fn show(bytes: &[u8]) -> String {
    bytes.escape_ascii().to_string()
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            let after_cr = std::mem::take(&mut self.after_cr);
            match byte {
                b'\n' if after_cr => {
                    let line = std::mem::take(&mut self.pending);
                    self.line(&line, port);
                }
                b'\n' => {
                    port.violation(format!("'{}': LF with no CR", show(&self.pending)));
                    self.pending.clear();
                }
                _ if after_cr => {
                    port.violation(format!("'{}': CR with no LF", show(&self.pending)));
                    self.pending.clear();
                    self.pending.push(byte);
                }
                b'\r' => self.after_cr = true,
                _ if self.pending.len() >= LONGEST => {
                    port.violation(format!("'{}': a line too long", show(&self.pending)));
                    self.pending.clear();
                }
                _ => self.pending.push(byte),
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && (!self.pending.is_empty() || self.after_cr) {
            port.violation(format!("'{}': a command left unfinished", show(&self.pending)));
            self.pending.clear();
            self.after_cr = false;
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if let Some((offers, bytes)) = &mut self.held {
            *offers -= 1;
            if *offers == 0 {
                port.send(bytes);
                self.held = None;
            }
        }
    }

    fn reset(&mut self) {
        *self = Meter::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let line = Line::parse("4800/8n1").unwrap();
        Host::open(function(&o).unwrap(), line, true, false)
    }

    fn violations(h: &mut Host<Meter>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn the_two_questions_have_the_wikis_answers() {
        let mut h = meter(&[]);
        h.write(b"IDN?\r\n");
        assert_eq!(h.read(), b"1834 065 V1.06,IF V1.02\r\n");
        h.write(b"STATUS?\r\n");
        assert_eq!(h.read(), b"03030024913100400040\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn answers_play_in_turn_with_their_forms() {
        let mut h = meter(&[("status", "a;b~;~;c_d"), ("idn", "x,y;z")]);
        for want in [&b"a\r\n"[..], b"b", b"", b"c d\r\n", b"a\r\n"] {
            h.write(b"STATUS?\r\n");
            assert_eq!(h.read(), want);
        }
        for want in [&b"x,y\r\n"[..], b"z\r\n", b"x,y\r\n"] {
            h.write(b"IDN?\r\n");
            assert_eq!(h.read(), want);
        }
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn line_ends_are_an_option() {
        for (eol, want) in [("cr", &b"a\r"[..]), ("lf", b"a\n"), ("crlf", b"a\r\n")] {
            let mut h = meter(&[("eol", eol), ("status", "a")]);
            h.write(b"STATUS?\r\n");
            assert_eq!(h.read(), want);
        }
    }

    #[test]
    fn what_the_driver_never_sends_is_flagged_and_gets_no_answer() {
        for bad in [&b"DATA?"[..], b"idn?", b"", b"STATUS", b"IDN?x"] {
            let mut h = meter(&[]);
            h.write(bad);
            h.write(b"\r\n");
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            assert_eq!(h.read(), b"");
        }
    }

    #[test]
    fn line_ends_other_than_cr_lf_are_flagged() {
        let mut h = meter(&[]);
        h.write(b"IDN?\n");
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.read(), b"");
        let mut h = meter(&[]);
        h.write(b"IDN?\rSTATUS?\r\n");
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.read(), b"03030024913100400040\r\n");
    }

    #[test]
    fn a_long_line_is_flagged_and_what_is_kept_stays_short() {
        let mut h = meter(&[]);
        h.write(&[b'x'; 4096]);
        assert!(h.cdc.model.pending.len() <= LONGEST);
        assert!(violations(&mut h) > 0);
    }

    #[test]
    fn a_command_left_unfinished_shows_at_the_next_open() {
        let mut h = meter(&[]);
        h.write(b"IDN");
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        h.write(b"IDN?\r");
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = meter(&[("answers", "1")]);
        h.write(b"IDN?\r\n");
        assert_ne!(h.read(), b"");
        assert!(!h.cdc.mark());
        h.write(b"STATUS?\r\n");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_late_answer_waits_for_its_offers() {
        let mut h = meter(&[("late", "2:5"), ("status", "a;b;c")]);
        h.write(b"STATUS?\r\n");
        assert_eq!(h.read(), b"a\r\n");
        h.write(b"STATUS?\r\n");
        for _ in 0..4 {
            assert_eq!(h.read(), b"");
        }
        let mut got = h.read();
        got.extend(h.read());
        assert_eq!(got, b"b\r\n");
        h.write(b"STATUS?\r\n");
        assert_eq!(h.read(), b"c\r\n");
        assert!(build(&[("late".to_owned(), "2".to_owned())]).is_err());
    }

    #[test]
    fn behind_its_uart_a_wrong_rate_hears_nothing() {
        let o = vec![];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, false);
        h.write(b"IDN?\r\n");
        assert_eq!(h.read(), b"");
        h.set_line(Line::parse("4800/8n1").unwrap());
        h.write(b"IDN?\r\n");
        assert_ne!(h.read(), b"");
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = meter(&[("status", "a;b"), ("answers", "1")]);
        h.write(b"STATUS?\r\n");
        assert_eq!(h.read(), b"a\r\n");
        h.cdc.model.reset();
        h.write(b"STATUS?\r\n");
        assert_eq!(h.read(), b"a\r\n");
    }

    #[test]
    fn options_parse() {
        for bad in [("eol", "x"), ("uart", "4800"), ("nothing", "1"), ("answers", "x")] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("status", "x"), ("uart", "9600/8n1"), ("late", "1:1")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
