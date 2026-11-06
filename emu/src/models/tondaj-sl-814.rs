// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Tondaj SL-814 sound level meter on the CDC-ACM shell. It speaks only when
//! spoken to, in three byte commands.
//!
//! The device, and how sure this is:
//!
//! - The sigrok wiki's page "Tondaj SL-814", written with the meter in hand (an
//!   `ATmega8L` behind a PL2303 cable), gives 9600 8e1 and these commands:
//!   `30 ZZ 0D` asks for a measurement and has `AA BB ZZ+1 0D` for an answer, ZZ
//!   being any value; `10 KK 0D` is a key press and has `KK+1 0D`, with 20, 30, 40
//!   and 50 the keys named; and `10 04 0D`, called init, has `05 0D`, does nothing
//!   anyone saw, and may be left out.
//! - Its example output and its Python script fix the reading: AA bit 7 is C, bit
//!   3 slow, bits 5-4 a level, bits 2-0 and BB eleven bits of tenths of a dB, so
//!   `09 AF` is 43.1 dB. The page says "BCD", and its examples are plain binary.
//!   The default list here is those examples.
//! - Bit 6 of AA has no known meaning. Nothing was found on what a reply does when
//!   commands come fast, nor on any firmware.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id, as the kernel's `pl2303`
//!   would take the real `067b:2303`: only the byte stream and the line settings
//!   are the device's.
//! - It answers at once, unless `late=` or `cut=` hold an answer back, counted in
//!   offers of a read. How long a real one takes was not found.
//! - The replies past the wiki's are stand-ins for a line gone wrong.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `data=HEX,...`, the readings in turn, each two bytes: AA BB;
//! - `reply=HEX,...`, whole replies to the first measurements, each up to 16 bytes
//!   and an empty one no reply, once, and then `data=` again: a list that came round
//!   would bring a second stray byte, and that can put a tool back in step;
//! - `init=HEX,...`, whole replies to the first inits, the same way;
//! - `uart=LINE`, its line settings;
//! - `answers=N`, commands answered per power on, then silence, and the event point
//!   `mark=K` offers (5) after the first command not answered;
//! - `quiet-mark=N:K`, the event point K offers after the Nth command is answered,
//!   with nothing out;
//! - `late=N:T`, the Nth measurement answered after T offers;
//! - `cut=K:T`, each reply's first K bytes at once and the rest T offers later.
//!
//! The pedantic checks: a command that is not one of the two forms is flagged and
//! gets no answer. The ones the wiki names and no C path sends are flagged and
//! answered, as the meter would: a key press, a measurement before init since power
//! on, and a `ZZ` other than 0, unless it is a new one that asks again after a
//! measurement that got no whole answer at once. Part of a command at a tool's open is flagged. More
//! than [`MOST_HELD`] replies held is flagged, and the rest get none.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "tondaj-sl-814";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Tondaj SL-814 sound level meter on CDC-ACM; data=, reply=, init= \
                         (in turn), uart=, answers=, mark=, quiet-mark=, late=, cut=, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// The wiki's example output, one per weighting and level it shows.
const DATA: &str = "09af,89cb,8a6c,8299,9285,a393,b3f2";
/// What a key press and a measurement start with, and what ends every command.
const KEY: u8 = 0x10;
const MEASURE: u8 = 0x30;
const END: u8 = 0x0d;
/// The key the driver sends as init.
const INIT_KEY: u8 = 0x04;
/// Replies held at one time. A driver has one out.
const MOST_HELD: usize = 16;
/// How many offers on from a silence the event point comes, by default.
const MARK_AFTER: u32 = 5;

/// A meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn pair(key: &str, value: &str) -> Result<(u32, u32)> {
    let (a, b) =
        value.split_once(':').ok_or_else(|| Error::Invalid(format!("{key}={value}: as in 2:40")))?;
    Ok((parse_number(key, a, 0..=100_000)?, parse_number(key, b, 1..=100_000)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("9600/8e1")?;
    let mut config = Config {
        data: Packets::new(parse_hex_list("data", DATA, 2, 2)?),
        reply: VecDeque::new(),
        init: VecDeque::new(),
        answers: None,
        mark: MARK_AFTER,
        quiet_mark: None,
        late: None,
        cut: None,
    };
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "data" => config.data = Packets::new(parse_hex_list(k, v, 2, 2)?),
            "reply" => config.reply = parse_hex_list(k, v, 0, 16)?.into(),
            "init" => config.init = parse_hex_list(k, v, 0, 16)?.into(),
            "uart" => uart = Line::parse(v)?,
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "mark" => config.mark = parse_number(k, v, 1..=100_000)?,
            "quiet-mark" => config.quiet_mark = Some(pair(k, v)?),
            "late" => config.late = Some(pair(k, v)?),
            "cut" => {
                let (bytes, offers) = pair(k, v)?;
                config.cut = Some((bytes as usize, offers));
            }
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Tondaj SL-814");
    Ok(Cdc::new(NAME, &id, faults, Meter::new(config)).behind_uart(uart))
}

#[derive(Clone, Debug)]
struct Config {
    data: Packets,
    reply: VecDeque<Vec<u8>>,
    init: VecDeque<Vec<u8>>,
    answers: Option<u32>,
    mark: u32,
    /// Which command answered, from 1, and how many offers after it.
    quiet_mark: Option<(u32, u32)>,
    /// Which measurement, from 1, and for how many offers.
    late: Option<(u32, u32)>,
    /// How many bytes go at once, and how many offers before the rest.
    cut: Option<(usize, u32)>,
}

struct Meter {
    config: Config,
    data: Packets,
    reply: VecDeque<Vec<u8>>,
    init: VecDeque<Vec<u8>>,
    /// What has come of a command so far.
    command: Vec<u8>,
    answered: u32,
    measured: u32,
    /// An init came, or a key press that is one, since power on.
    initialised: bool,
    /// A measurement has had no whole answer at once, so a tool may ask again.
    went_unanswered: bool,
    /// The last sequence number of a measurement asked again.
    last_again: u8,
    /// Replies on their way, in order: the offers each still waits, and its bytes.
    held: VecDeque<(u32, Vec<u8>)>,
    /// Offers until the event point, once it has been set off.
    mark_in: Option<u32>,
    marked: bool,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter {
            data: config.data.clone(),
            reply: config.reply.clone(),
            init: config.init.clone(),
            config,
            command: Vec::new(),
            answered: 0,
            measured: 0,
            initialised: false,
            went_unanswered: false,
            last_again: 0,
            held: VecDeque::new(),
            mark_in: None,
            marked: false,
        }
    }

    fn command(&mut self, [first, value, end]: [u8; 3], port: &mut Port<'_>) {
        if end != END || (first != KEY && first != MEASURE) {
            port.violation(format!("{first:02x} {value:02x} {end:02x}: no command of the meter's"));
            return;
        }
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            self.went_unanswered |= first == MEASURE;
            if !self.marked && self.mark_in.is_none() {
                self.mark_in = Some(self.config.mark);
            }
            return;
        }
        self.answered += 1;
        if self.config.quiet_mark.is_some_and(|(n, _)| n == self.answered) {
            self.mark_in = self.config.quiet_mark.map(|(_, offers)| offers);
        }
        let (reply, late) = if first == KEY {
            (self.key(value, port), None)
        } else {
            let reply = self.measure(value, port);
            let late = self.config.late.filter(|(nth, _)| *nth == self.measured).map(|(_, t)| t);
            self.went_unanswered |= late.is_some() || self.config.cut.is_some() || reply.len() < 4;
            (reply, late)
        };
        self.answer(&reply, late, port);
    }

    fn key(&mut self, key: u8, port: &mut Port<'_>) -> Vec<u8> {
        if key != INIT_KEY {
            port.violation(format!("key {key:02x}: no C path presses it"));
        }
        self.initialised = true;
        self.init.pop_front().unwrap_or_else(|| vec![key.wrapping_add(1), END])
    }

    fn measure(&mut self, zz: u8, port: &mut Port<'_>) -> Vec<u8> {
        if zz != 0 && !(self.went_unanswered && (zz > self.last_again || self.last_again == 255)) {
            port.violation(format!("sequence {zz:02x}: not one asking again"));
        }
        self.last_again = if zz == 0 { self.last_again } else { zz };
        if !self.initialised {
            port.violation("a measurement before init".to_owned());
        }
        self.measured += 1;
        self.reply.pop_front().unwrap_or_else(|| {
            let reading = self.data.take();
            vec![reading[0], reading[1], zz.wrapping_add(1), END]
        })
    }

    /// Sends a reply, or holds it as the options say. What is out stays ahead of it.
    fn answer(&mut self, reply: &[u8], late: Option<u32>, port: &mut Port<'_>) {
        let (now, later) = match self.config.cut {
            Some((bytes, _)) if bytes < reply.len() => reply.split_at(bytes),
            _ => (reply, &[][..]),
        };
        let mut pieces = Vec::new();
        if !now.is_empty() {
            pieces.push((late.unwrap_or(0), now.to_vec()));
        }
        if let Some((_, offers)) = self.config.cut
            && !later.is_empty()
        {
            pieces.push((offers, later.to_vec()));
        }
        for (offers, piece) in pieces {
            if offers == 0 && self.held.is_empty() {
                port.send(&piece);
            } else if self.held.len() < MOST_HELD {
                self.held.push_back((offers, piece));
            } else {
                port.violation("too many replies held".to_owned());
            }
        }
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            self.command.push(byte);
            if let [first, value, end] = self.command[..] {
                self.command.clear();
                self.command([first, value, end], port);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.command.is_empty() {
            port.violation(format!("{:02x?}: a command left unfinished", self.command));
            self.command.clear();
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if let Some((offers, _)) = self.held.front_mut() {
            *offers = offers.saturating_sub(1);
            if *offers == 0
                && let Some((_, bytes)) = self.held.pop_front()
            {
                port.send(&bytes);
            }
        }
        if let Some(offers) = &mut self.mark_in {
            *offers -= 1;
            if *offers == 0 {
                self.mark_in = None;
                self.marked = true;
                port.mark();
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

    const INIT: &[u8] = &[0x10, 0x04, 0x0d];
    const ASK: &[u8] = &[0x30, 0x00, 0x0d];

    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("9600/8e1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Meter>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn init_then_the_wikis_readings_with_the_sequence_plus_one() {
        let mut h = meter(&[]);
        h.write(INIT);
        assert_eq!(h.read(), [0x05, 0x0d]);
        for want in [[0x09u8, 0xaf], [0x89, 0xcb], [0x8a, 0x6c]] {
            h.write(ASK);
            assert_eq!(h.read(), [want[0], want[1], 0x01, 0x0d]);
        }
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn how_a_command_is_cut_means_nothing() {
        let mut h = meter(&[("data", "1234")]);
        for &byte in INIT {
            h.write(&[byte]);
        }
        assert_eq!(h.read(), [0x05, 0x0d]);
        h.write(&[0x30]);
        assert_eq!(h.read(), b"");
        h.write(&[0x00, 0x0d]);
        assert_eq!(h.read(), [0x12, 0x34, 0x01, 0x0d]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_is_no_command_is_flagged_and_has_no_answer() {
        for bad in [&[0x31u8, 0, 0x0d][..], &[0x30, 0, 0x0a], &[0x0d, 0x0d, 0x0d]] {
            let mut h = meter(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{bad:02x?}");
            assert_eq!(h.read(), b"");
        }
    }

    #[test]
    fn what_no_c_path_sends_is_flagged_and_answered_as_the_wiki_says() {
        let mut h = meter(&[]);
        h.write(&[0x10, 0x40, 0x0d]);
        assert_eq!(violations(&mut h), 1, "a key");
        assert_eq!(h.read(), [0x41, 0x0d]);
        h.write(&[0x30, 0xff, 0x0d]);
        assert_eq!(violations(&mut h), 1, "a sequence");
        assert_eq!(h.read(), [0x09, 0xaf, 0x00, 0x0d]);
    }

    #[test]
    fn a_new_sequence_may_ask_again_after_a_silence() {
        let mut h = meter(&[("reply", "1234010d,"), ("data", "7788")]);
        h.write(INIT);
        let _ = h.read();
        h.write(ASK);
        assert_eq!(h.read(), [0x12, 0x34, 0x01, 0x0d]);
        h.write(&[0x30, 1, 0x0d]);
        assert_eq!(violations(&mut h), 1, "nothing went unanswered yet");
        let _ = h.read();
        h.cdc.model.reset();
        h.write(INIT);
        let _ = h.read();
        h.write(ASK);
        assert_eq!(h.read(), [0x12, 0x34, 0x01, 0x0d]);
        h.write(ASK);
        assert_eq!(h.read(), b"", "lost");
        for zz in [1, 2] {
            h.write(&[0x30, zz, 0x0d]);
            assert_eq!(h.read(), [0x77, 0x88, zz + 1, 0x0d]);
            assert_eq!(violations(&mut h), 0, "{zz}");
        }
        h.write(&[0x30, 2, 0x0d]);
        assert_eq!(violations(&mut h), 1, "the same one again");
    }

    #[test]
    fn a_measurement_before_init_is_flagged_and_answered() {
        let mut h = meter(&[]);
        h.write(ASK);
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.read(), [0x09, 0xaf, 0x01, 0x0d]);
    }

    #[test]
    fn a_command_left_unfinished_shows_at_the_next_open() {
        let mut h = meter(&[]);
        h.write(&[0x10, 0x04]);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        h.write(INIT);
        assert_eq!(h.read(), [0x05, 0x0d]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn whole_replies_play_once_and_an_empty_one_is_none() {
        let mut h = meter(&[("reply", "aa,,1234560d"), ("init", ",060d"), ("data", "7788")]);
        h.write(INIT);
        assert_eq!(h.read(), b"");
        h.write(INIT);
        assert_eq!(h.read(), [0x06, 0x0d]);
        h.write(INIT);
        assert_eq!(h.read(), [0x05, 0x0d]);
        for want in [&[0xaa][..], &[], &[0x12, 0x34, 0x56, 0x0d], &[0x77, 0x88, 1, 0x0d]] {
            h.write(ASK);
            assert_eq!(h.read(), want);
        }
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn answers_runs_out_and_the_mark_comes_later() {
        let mut h = meter(&[("answers", "1"), ("mark", "3")]);
        h.write(INIT);
        assert_ne!(h.read(), b"");
        h.write(ASK);
        for _ in 0..2 {
            assert_eq!(h.read(), b"");
            assert!(!h.cdc.mark());
        }
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_quiet_mark_counts_from_the_answer() {
        let mut h = meter(&[("quiet-mark", "2:3")]);
        h.write(INIT);
        assert_ne!(h.read(), b"");
        h.write(ASK);
        assert_ne!(h.read(), b"");
        let mut offers = 0;
        while !h.cdc.mark() {
            let _ = h.read();
            offers += 1;
            assert!(offers < 10);
        }
        assert!(offers >= 2, "{offers}");
    }

    #[test]
    fn a_late_answer_waits_for_its_offers() {
        let mut h = meter(&[("late", "2:5"), ("data", "1111,2222,3333")]);
        h.write(INIT);
        assert_eq!(h.read(), [0x05, 0x0d]);
        h.write(ASK);
        assert_eq!(h.read(), [0x11, 0x11, 1, 0x0d]);
        h.write(ASK);
        for _ in 0..4 {
            assert_eq!(h.read(), b"");
        }
        let mut got = h.read();
        got.extend(h.read());
        assert_eq!(got, [0x22, 0x22, 1, 0x0d]);
        h.write(ASK);
        assert_eq!(h.read(), [0x33, 0x33, 1, 0x0d]);
    }

    #[test]
    fn a_cut_reply_comes_in_two_pieces() {
        let mut h = meter(&[("cut", "2:3"), ("data", "1234")]);
        h.write(INIT);
        let mut got = h.read();
        assert_eq!(got, [0x05, 0x0d], "as long as the cut: nothing held");
        h.write(ASK);
        got = h.read();
        assert_eq!(got, [0x12, 0x34]);
        assert_eq!(h.read(), b"");
        let mut rest = h.read();
        rest.extend(h.read());
        assert_eq!(rest, [0x01, 0x0d]);
    }

    #[test]
    fn replies_held_have_a_bound() {
        let mut h = meter(&[("cut", "0:1000"), ("data", "1234")]);
        // The first reply's two bytes are one piece, held for its offers.
        for _ in 0..=MOST_HELD {
            h.write(INIT);
        }
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.cdc.model.held.len(), MOST_HELD);
    }

    #[test]
    fn behind_its_uart_a_wrong_line_hears_nothing() {
        let o = vec![];
        let line = Line::parse("9600/8n1").unwrap();
        let mut h = Host::open(function(&o).unwrap(), line, true, true);
        h.write(INIT);
        assert_eq!(h.read(), b"");
        h.set_line(Line::parse("9600/8e1").unwrap());
        h.write(INIT);
        assert_eq!(h.read(), [0x05, 0x0d]);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = meter(&[("data", "1111,2222"), ("answers", "2")]);
        h.write(INIT);
        h.write(ASK);
        assert_eq!(h.read().len(), 6);
        h.cdc.model.reset();
        h.write(INIT);
        assert_eq!(h.read(), [0x05, 0x0d]);
        h.write(ASK);
        assert_eq!(h.read(), [0x11, 0x11, 1, 0x0d]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("data", "12"),
            ("data", "123456"),
            ("reply", "zz"),
            ("uart", "9600"),
            ("late", "1"),
            ("cut", "x:1"),
            ("nothing", "1"),
            ("answers", "x"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("reply", ""), ("uart", "2400/8e1"), ("late", "1:1")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
