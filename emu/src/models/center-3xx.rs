// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Center 309 four channel thermometer, sold by Voltcraft as the K204, on the
//! CDC-ACM shell. It knows one command: `A`, answered with 45 bytes.
//!
//! The device, and how sure this is:
//!
//! - Conrad's "Schnittstelle Voltcraft K204 Datalogger" (document 100518-da-01-en,
//!   read from the Wayback Machine's 2017 copy; the sigrok wiki's page on the K204
//!   links it) gives 9600 8n1, the command, and this answer from a unit, which is
//!   the default here: `02 80 80 01 02 02 02 00 EF 7F FF 7F FF 7F FF`, 24 bytes of
//!   00, then `0E 00 00 00 0E 03`, read as T1 23.9 C and T2 to T4 over limit.
//! - By that document, counting from 0: byte 1 holds REC (bit 0), the mode (bits 1
//!   and 2), T1-T2, REL, HOLD, low battery and Celsius (bits 3 to 7); byte 2 memory
//!   full (bit 0) and auto power off (bit 7); bytes 7 to 14 the four values, big
//!   endian; 15 to 22, 23 to 30 and 31 to 38 the same in REL, MIN and MAX; 39 to 42
//!   the over limit bits per channel, normal, REL, MAX and MIN in that order; and
//!   43 the resolution per channel, bit clear for tenths. Bytes 3 to 6 "can't use".
//! - It says nothing of how long an answer takes, of what else a unit answers, or
//!   of what a unit does with another command. So a list of answers here stands in
//!   for a meter in other modes, and all but the default are stand-ins.
//! - No recording of a unit was found, and no firmware. The sigrok wiki has no page
//!   on the Center 309 itself, and calls the K204 a rebadged 309.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id, where the real one is an
//!   RS-232 cable: only the byte stream and the line settings are the device's.
//! - It answers at once, in the same server round, unless `late=` or `split=` says
//!   otherwise, and takes no time over the 45 bytes: 47 ms at 9600 on a real line.
//! - Each `A` gets the next entry of the list, where a unit measures by itself.
//! - `split=` and an entry of the wrong length are lines gone wrong, which a unit
//!   can produce only through a bad cable. No answer is a unit that is off.
//!
//! An answer in `frames=` is hex, any length, those bytes alone, or `~` for no
//! answer. They are separated by `;`, and play in turn and then again.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `frames=`, the list;
//! - `uart=LINE`, its line settings;
//! - `answers=N`, commands answered per power on, then silence and the event point;
//! - `late=N:T`, the Nth `A` answered after T offers of a read;
//! - `split=N:K:T`, the Nth answer's first K bytes at once, the rest T offers later;
//! - `mark=N:T`, the event point T offers after the Nth `A`, a quiet moment.
//!
//! What is held back keeps its turn: a unit's answers do not overtake each other.
//!
//! The pedantic checks: only `A`. The vendor's program may send others, and no path
//! of the driver does, so they are flagged and get no answer, which is itself a
//! guess.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, Port, Serial};
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "center-3xx";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Center 309 / Voltcraft K204 thermometer on CDC-ACM; frames= (answers \
                         in turn), uart=, answers=, late=, split=, mark=, and the shell's \
                         stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// The answer in Conrad's document.
const EXAMPLE: &str = concat!(
    "02808001020202",
    "00ef7fff7fff7fff",
    "000000000000000000000000000000000000000000000000",
    "0e0000000e03",
);

/// A thermometer from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn hex(text: &str) -> Result<Vec<u8>> {
    if !text.is_ascii() || text.len() % 2 != 0 {
        return Err(Error::Invalid(format!("frames: '{text}' is not whole hex bytes")));
    }
    (0..text.len() / 2)
        .map(|i| {
            u8::from_str_radix(&text[2 * i..2 * i + 2], 16)
                .map_err(|_| Error::Invalid(format!("frames: '{text}' is not hex")))
        })
        .collect()
}

fn frames_list(value: &str) -> Result<Vec<Vec<u8>>> {
    value.split(';').map(|t| if t == "~" { Ok(Vec::new()) } else { hex(t) }).collect()
}

/// The numbers of an option written `A:B`, or `A:B:C`.
fn parts(key: &str, value: &str, count: usize) -> Result<Vec<u32>> {
    let parts: Vec<&str> = value.split(':').collect();
    if parts.len() != count {
        return Err(Error::Invalid(format!("{key}={value}: {count} numbers, with colons")));
    }
    parts.iter().map(|p| parse_number(key, p, 0..=100_000)).collect()
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("9600/8n1")?;
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "frames" => config.frames = frames_list(v)?,
            "uart" => uart = Line::parse(v)?,
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "late" => {
                let p = parts(k, v, 2)?;
                config.late = Some((p[0], p[1]));
            }
            "split" => {
                let p = parts(k, v, 3)?;
                config.split = Some((p[0], p[1] as usize, p[2]));
            }
            "mark" => {
                let p = parts(k, v, 2)?;
                config.mark = Some((p[0], p[1]));
            }
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Center 309 thermometer");
    Ok(Cdc::new(NAME, &id, faults, Meter::new(config)).behind_uart(uart))
}

#[derive(Clone, Debug)]
struct Config {
    frames: Vec<Vec<u8>>,
    answers: Option<u32>,
    /// Which `A`, from 1, and for how many offers of a read.
    late: Option<(u32, u32)>,
    /// Which `A`, from 1, how many bytes go at once, and the offers to the rest.
    split: Option<(u32, usize, u32)>,
    /// Which `A`, from 1, and the offers from it to the event point.
    mark: Option<(u32, u32)>,
}

impl Default for Config {
    fn default() -> Config {
        let frames = frames_list(EXAMPLE).expect("the example is hex");
        Config { frames, answers: None, late: None, split: None, mark: None }
    }
}

struct Meter {
    config: Config,
    asked: u32,
    /// What is on its way: the offers it still waits, once it is first, and bytes.
    held: VecDeque<(u32, Vec<u8>)>,
    /// The offers left to the event point.
    to_mark: Option<u32>,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter { config, asked: 0, held: VecDeque::new(), to_mark: None }
    }

    /// Sends now, unless something is ahead of it.
    fn queue(&mut self, offers: u32, bytes: Vec<u8>, port: &mut Port<'_>) {
        if offers == 0 && self.held.is_empty() {
            port.send(&bytes);
        } else {
            self.held.push_back((offers, bytes));
        }
    }

    fn command(&mut self, port: &mut Port<'_>) {
        if self.config.answers.is_some_and(|n| self.asked >= n) {
            port.mark();
            return;
        }
        self.asked += 1;
        if let Some((_, offers)) = self.config.mark.filter(|(n, _)| *n == self.asked) {
            self.to_mark = Some(offers);
        }
        let at = (self.asked as usize - 1) % self.config.frames.len();
        let frame = self.config.frames[at].clone();
        if frame.is_empty() {
            return;
        }
        let late = self.config.late.filter(|(n, _)| *n == self.asked).map_or(0, |(_, t)| t);
        if let Some((_, first, offers)) = self.config.split.filter(|(n, ..)| *n == self.asked) {
            let first = first.min(frame.len());
            self.queue(late, frame[..first].to_vec(), port);
            self.queue(offers, frame[first..].to_vec(), port);
        } else {
            self.queue(late, frame, port);
        }
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte == b'A' {
                self.command(port);
            } else {
                port.violation(format!("0x{byte:02x}: nothing the driver sends"));
            }
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if let Some(offers) = &mut self.to_mark {
            *offers = offers.saturating_sub(1);
            if *offers == 0 {
                self.to_mark = None;
                port.mark();
            }
        }
        let Some((offers, _)) = self.held.front_mut() else {
            return;
        };
        *offers = offers.saturating_sub(1);
        if *offers == 0
            && let Some((_, bytes)) = self.held.pop_front()
        {
            port.send(&bytes);
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
        Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Meter>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn the_default_is_the_documents_answer() {
        let mut h = meter(&[]);
        h.write(b"A");
        let got = h.read();
        assert_eq!(got.len(), 45);
        let head = [2, 0x80, 0x80, 1, 2, 2, 2, 0, 0xef, 0x7f, 0xff, 0x7f, 0xff, 0x7f, 0xff];
        assert_eq!(&got[..15], head);
        assert!(got[15..39].iter().all(|&b| b == 0));
        assert_eq!(&got[39..], [0x0e, 0, 0, 0, 0x0e, 3]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn answers_play_in_turn_and_again() {
        let mut h = meter(&[("frames", "aa;~;bbcc")]);
        for want in [&b"\xaa"[..], b"", b"\xbb\xcc", b"\xaa"] {
            h.write(b"A");
            assert_eq!(h.read(), want);
        }
    }

    #[test]
    fn two_commands_in_one_write_get_two_answers() {
        let mut h = meter(&[("frames", "aa;bb")]);
        h.write(b"AA");
        assert_eq!(h.read(), b"\xaa\xbb");
    }

    #[test]
    fn what_the_driver_never_sends_is_flagged_and_gets_no_answer() {
        for bad in [&b"a"[..], b"K", b"\r", b"\0"] {
            let mut h = meter(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{bad:?}");
            assert_eq!(h.read(), b"");
        }
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = meter(&[("answers", "1")]);
        h.write(b"A");
        assert_ne!(h.read(), b"");
        assert!(!h.cdc.mark());
        h.write(b"A");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_late_answer_waits_for_its_offers_and_keeps_its_turn() {
        let mut h = meter(&[("late", "2:5"), ("frames", "aa;bb;cc")]);
        h.write(b"A");
        assert_eq!(h.read(), b"\xaa");
        h.write(b"A");
        h.write(b"A");
        for _ in 0..4 {
            assert_eq!(h.read(), b"");
        }
        let mut got = Vec::new();
        for _ in 0..3 {
            got.extend(h.read());
        }
        assert_eq!(got, b"\xbb\xcc");
    }

    #[test]
    fn a_split_answer_sends_its_tail_later() {
        let mut h = meter(&[("split", "1:1:4"), ("frames", "aabbcc")]);
        h.write(b"A");
        assert_eq!(h.read(), b"\xaa");
        for _ in 0..2 {
            assert_eq!(h.read(), b"");
        }
        let mut got = h.read();
        got.extend(h.read());
        assert_eq!(got, b"\xbb\xcc");
    }

    #[test]
    fn a_mark_comes_its_offers_after_its_command() {
        let mut h = meter(&[("mark", "2:2")]);
        h.write(b"A");
        h.read();
        h.read();
        h.read();
        assert!(!h.cdc.mark());
        h.write(b"A");
        h.read();
        assert!(!h.cdc.mark());
        h.read();
        h.read();
        assert!(h.cdc.mark());
    }

    #[test]
    fn behind_its_uart_a_wrong_rate_hears_nothing() {
        let o = vec![];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("4800/8n1").unwrap(), true, true);
        h.write(b"A");
        assert_eq!(h.read(), b"");
        h.set_line(Line::parse("9600/8n1").unwrap());
        h.write(b"A");
        assert_ne!(h.read(), b"");
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = meter(&[("frames", "aa;bb"), ("answers", "1")]);
        h.write(b"A");
        assert_eq!(h.read(), b"\xaa");
        h.cdc.model.reset();
        h.write(b"A");
        assert_eq!(h.read(), b"\xaa");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("frames", "abc"),
            ("frames", "xy"),
            ("frames", "é"),
            ("uart", "9600"),
            ("nothing", "1"),
            ("answers", "x"),
            ("late", "2"),
            ("split", "1:2"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("frames", "~"), ("uart", "4800/8n1"), ("late", "1:1")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
