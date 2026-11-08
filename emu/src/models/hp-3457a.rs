// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An HP 3457A multimeter, on a socket alone: a real one has HP-IB, which oxrok does
//! not, so this stands for one behind a GPIB to Ethernet adapter that libsigrok
//! reaches as `tcp-raw/HOST/PORT`. It is no SCPI instrument: no `*IDN?`, and commands
//! with no question mark that answer (`?` itself, `TRIG SGL`, `RMATH HIRES`).
//!
//! The device, and how sure this is:
//!
//! - **From libsigrok's driver and nothing better.** The commands, `ID?` being
//!   `HP3457A`, the card ids, and that `REV?` is two numbers. No manual could be
//!   read here (the copies found refuse a plain fetch) and no recording exists.
//! - **sigrok's wiki** says only that a channel switched by hand gives a "Trig too
//!   fast" error, which is why a trigger with a measurement under way is flagged.
//! - **Guesses, which no test should lean on:** the form of a number
//!   (`+1.234567E+00`), the line end (CR LF), that a preset selects 1 PLC (`plc=`
//!   changes it), that a preset selects the front terminals (`left-rear=1` is the other
//!   reading), that a preset selects DCV (on which a port's `Voltage` label rests when
//!   no quantity is set), that a `NPLC` goes up to the next of the six times, the `REV?`
//!   answer, what a `SLIST` with a leading comma does (taken as a list).
//! - **`real-rate=1`** paces a measurement by arithmetic: twice the PLCs for the
//!   auto zero, at 50 Hz, so 4 s at 100 PLC. Autorange and a card's relays would add
//!   to it, and no data sheet could be read to compare.
//!
//! Emulation artifacts: a reading is a table's value and a counter, by function,
//! terminals and channel, so that a wrong setting shows in the output. Without
//! `real-rate=1` a measurement takes no time. A connection is the adapter's: the
//! meter's settings outlive it, and what it was to say goes with it.
//!
//! Options, besides the SCPI shell's ([`scpi::Config::option`]):
//!
//! - `id=TEXT`, `rev=TEXT`, `opt=TEXT`: what `ID?`, `REV?` and `OPT?` answer;
//!   `card=N` is `opt=` as a number: 0 rear terminals, 44491, 44492 (the default is
//!   none, which answers `-1`, as no document says what that is);
//! - `readings=A,B,...`, `hires=A,...`, `ranges=A,...`, `plcs=A,...`, `chans=A,...`:
//!   the texts the measurement, `RMATH HIRES`, `RANGE?`, `NPLC?` and `CHAN?` answer, in
//!   turn, each list played once and then the table's. `_` is a space;
//! - `plc=N`: what a preset selects;
//! - `real-rate=1`: above;
//! - `left-rear=1`: the terminals are at the rear from an earlier run, as the harness
//!   replaces the model before each tool, and a preset leaves them;
//! - `split=AT:TICKS:FROM`: from answer number FROM on, each comes in two pieces, TICKS
//!   apart, cut AT bytes in, or that many from its end when AT is negative. It stands
//!   for what a bridge might do, and no bridge has been seen to;
//! - `chan-skew=N`: `CHAN?` answers N more than the channel it is on.
//!
//! The pedantic checks: what the meter has no command for, an argument no command
//! takes, and a trigger with a measurement under way. `NPLC NAN` and a scan list with
//! a channel the card lacks are errors and no violation, as C sends both.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use crate::model::{DeviceModel, Violations};
use crate::options::parse_number;
use crate::shells::scpi::{self, Answer, Engine, Instrument, Message, Settings};
use crate::shells::tcp::{NoUsb, Port, Stream};
use crate::shells::usbtmc::Messages;
use crate::{Error, Result};

/// The model's name.
pub const NAME: &str = "hp-3457a";
/// One line for `oxrok-emu list`.
pub const ABOUT: &str = "HP 3457A multimeter on TCP alone; id=, rev=, opt=, card=, readings=, \
                         hires=, ranges=, plcs=, chans=, plc=, real-rate=, left-rear=, split=, chan-skew=, and the SCPI \
                         shell's mute-at=, silent-at=, late-at=, mark-at=, eol=";

/// The integration times, in power line cycles.
const PLCS: [f64; 6] = [0.0005, 0.005, 0.1, 1.0, 10.0, 100.0];
/// What the reading rate is reckoned at: a 50 Hz line.
const LINE_HZ: f64 = 50.0;
/// A tick on TCP, in seconds.
const TICK_S: f64 = 0.020;
/// What is asked for by the last entry of a card's table, and what any card has.
const CARDS: [(i64, u32); 3] = [(0, 1), (44491, 14), (44492, 10)];

/// What a measurement is of.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Function {
    command: &'static str,
    /// The first reading.
    reading: f64,
    /// What `RANGE?` says.
    range: f64,
}

#[allow(clippy::unreadable_literal)]
const FUNCTIONS: [Function; 9] = [
    Function { command: "DCV", reading: 1.234567, range: 30.0 },
    Function { command: "ACV", reading: 2.345678, range: 30.0 },
    Function { command: "ACDCV", reading: 3.456789, range: 30.0 },
    Function { command: "DCI", reading: 0.012345, range: 0.03 },
    Function { command: "ACI", reading: 0.023456, range: 0.03 },
    Function { command: "ACDCI", reading: 0.034567, range: 0.03 },
    Function { command: "OHM", reading: 1000.123, range: 3000.0 },
    Function { command: "OHMF", reading: 999.876, range: 3000.0 },
    Function { command: "FREQ", reading: 50.0123, range: 300.0 },
];

/// `%+.6E`, as this model has a number's form.
fn sci(value: f64) -> String {
    let text = format!("{value:+.6E}");
    let (mantissa, exponent) = text.split_once('E').unwrap_or((&text, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    format!("{mantissa}E{exponent:+03}")
}

/// A list of texts played once.
#[derive(Clone, Debug, Default)]
struct Played(VecDeque<String>);

impl Played {
    fn parse(value: &str) -> Played {
        Played(value.split(',').map(|t| t.replace('_', " ")).collect())
    }

    fn next(&mut self) -> Option<String> {
        self.0.pop_front()
    }
}

/// The meter.
#[derive(Debug)]
// Each flag is a switch on the meter, not a state of one machine.
#[allow(clippy::struct_excessive_bools)]
struct Meter {
    id: String,
    rev: String,
    opt: String,
    preset_plc: f64,
    keeps_terminals: bool,
    plc: f64,
    function: &'static Function,
    rear: bool,
    /// The scan list, with its place, while scan advance is on.
    list: Vec<u32>,
    place: usize,
    advance: bool,
    chan_skew: i64,
    readings: Played,
    hires: Played,
    ranges: Played,
    plcs: Played,
    chans: Played,
    /// Measurements so far.
    count: u32,
    /// What a measurement takes, in ticks; the socket's side reads it.
    pace: Arc<AtomicU32>,
    real_rate: bool,
    /// How many channels the card has.
    channels: u32,
}

impl Meter {
    fn set_plc(&mut self, plc: f64) {
        self.plc = plc;
        let seconds = 2.0 * plc / LINE_HZ;
        let ticks = if self.real_rate { (seconds / TICK_S).ceil() as u32 } else { 0 };
        self.pace.store(ticks, Ordering::SeqCst);
    }

    fn preset(&mut self) {
        self.set_plc(self.preset_plc);
        self.function = &FUNCTIONS[0];
        self.rear = self.rear && self.keeps_terminals;
        self.list.clear();
        self.place = 0;
        self.advance = false;
    }

    fn channel(&self) -> Option<u32> {
        self.list.get(self.place).copied()
    }

    /// A measurement: its reading, and scan advance moves on as it is sent.
    fn measure(&mut self) -> Answer {
        let channel = self.channel().map_or(0.0, f64::from);
        let terminals = if self.rear { 100.0 } else { 0.0 };
        let table = self.function.reading + terminals + channel * 10.0 + f64::from(self.count);
        self.count += 1;
        if self.advance && !self.list.is_empty() {
            self.place = (self.place + 1) % self.list.len();
        }
        Answer::Text(self.readings.next().unwrap_or_else(|| sci(table)))
    }

    fn select(&mut self, plc_text: &str) -> Answer {
        let Ok(asked) = plc_text.trim().parse::<f64>() else {
            return Answer::Error(-1, "number expected".to_owned());
        };
        // A NaN is the C's to send, and no time to select.
        if asked.is_nan() {
            return Answer::Error(-1, "number expected".to_owned());
        }
        let selected = PLCS.iter().copied().find(|p| *p >= asked).unwrap_or(100.0);
        self.set_plc(selected);
        Answer::Done
    }
}

impl Instrument for Meter {
    fn settings(&self) -> Settings {
        Settings::default()
    }

    fn command(&mut self, m: &Message<'_>, _s: &mut Settings, v: &mut Violations) -> Answer {
        let word = |v: &mut Violations, words: &[&str]| {
            if !words.contains(&m.args) {
                v.report(format!("{}: no such argument", m.text));
            }
        };
        match m.header.as_str() {
            "END" => {
                word(v, &["ALWAYS"]);
                Answer::Done
            }
            "ID?" => Answer::Text(self.id.clone()),
            "REV?" => Answer::Text(self.rev.clone()),
            "OPT?" => Answer::Text(self.opt.clone()),
            "PRESET" => {
                self.preset();
                Answer::Done
            }
            "INBUF" => {
                word(v, &["ON"]);
                Answer::Done
            }
            "TRIG" => {
                word(v, &["SGL", "HOLD", "AUTO"]);
                if m.args == "SGL" { self.measure() } else { Answer::Done }
            }
            "?" => self.measure(),
            "RMATH" => {
                word(v, &["HIRES"]);
                let tail = 1e-7 * f64::from(self.count);
                Answer::Text(self.hires.next().unwrap_or_else(|| sci(tail)))
            }
            "RANGE?" => {
                Answer::Text(self.ranges.next().unwrap_or_else(|| sci(self.function.range)))
            }
            "CHAN?" => {
                let on = i64::from(self.channel().unwrap_or(0)) + self.chan_skew;
                Answer::Text(self.chans.next().unwrap_or_else(|| sci(on as f64)))
            }
            "NPLC?" => Answer::Text(self.plcs.next().unwrap_or_else(|| sci(self.plc))),
            "NPLC" => self.select(m.args),
            "TERM" => {
                word(v, &["FRONT", "REAR"]);
                self.rear = m.args == "REAR";
                Answer::Done
            }
            "SLIST" => {
                let list: Option<Vec<u32>> = m
                    .args
                    .split(',')
                    .filter(|t| !t.is_empty())
                    .map(|t| t.trim().parse().ok().filter(|c| *c < self.channels))
                    .collect();
                match list {
                    Some(list) => {
                        self.list = list;
                        self.place = 0;
                        Answer::Done
                    }
                    None => Answer::Error(-2, "no such channel".to_owned()),
                }
            }
            "SADV" => {
                word(v, &["AUTO", "HOLD"]);
                self.advance = m.args == "AUTO" && !self.list.is_empty();
                Answer::Done
            }
            other => match FUNCTIONS.iter().find(|f| f.command == other) {
                Some(function) => {
                    self.function = function;
                    Answer::Done
                }
                None => Answer::Unknown,
            },
        }
    }
}

/// The meter behind its adapter. An answer goes when its measurement is done, and
/// what follows waits behind it.
struct OnSocket {
    engine: Engine<Meter>,
    pace: Arc<AtomicU32>,
    line: Vec<u8>,
    /// What is to go, when its ticks are out, and whether it is a measurement's.
    held: VecDeque<(u32, Vec<u8>, bool)>,
    split: Option<Split>,
    answers: u32,
}

/// `split=`.
#[derive(Clone, Copy, Debug)]
struct Split {
    at: i32,
    ticks: u32,
    from: u32,
}

impl OnSocket {
    /// Queues an answer, in two pieces if `split=` says so.
    fn hold(&mut self, ticks: u32, bytes: Vec<u8>, measuring: bool) {
        self.answers += 1;
        let cut = self.split.filter(|s| self.answers >= s.from).and_then(|s| {
            let at = if s.at < 0 { bytes.len() as i32 + s.at } else { s.at };
            usize::try_from(at).ok().filter(|at| *at > 0 && *at < bytes.len()).map(|at| (at, s.ticks))
        });
        match cut {
            Some((at, gap)) => {
                self.held.push_back((ticks, bytes[..at].to_vec(), measuring));
                self.held.push_back((gap, bytes[at..].to_vec(), false));
            }
            None => self.held.push_back((ticks, bytes, measuring)),
        }
    }
}

impl OnSocket {
    fn out(&mut self, port: &mut Port) {
        while self.held.front().is_some_and(|(ticks, ..)| *ticks == 0) {
            if let Some((_, bytes, _)) = self.held.pop_front() {
                port.send(&bytes);
            }
        }
        if self.engine.mark() {
            port.mark();
        }
    }

    fn line_in(&mut self, line: &[u8], port: &mut Port) {
        let text = String::from_utf8_lossy(line);
        let text = text.trim();
        let trigger = text == "?" || text.eq_ignore_ascii_case("TRIG SGL");
        if trigger && self.held.iter().any(|(_, _, measuring)| *measuring) {
            port.violation(format!("{text}: a trigger with a measurement under way"));
        }
        let mut with_end = line.to_vec();
        with_end.push(b'\n');
        self.engine.message(&with_end, port.violations());
        let ticks = if trigger { self.pace.load(Ordering::SeqCst) } else { 0 };
        while let Some(bytes) = self.engine.answer() {
            self.hold(ticks, bytes, trigger && ticks > 0);
        }
    }
}

impl Stream for OnSocket {
    fn received(&mut self, data: &[u8], port: &mut Port) {
        for b in data {
            if *b == b'\n' {
                let line = std::mem::take(&mut self.line);
                self.line_in(&line, port);
            } else if self.line.len() < 1 << 16 {
                self.line.push(*b);
            }
        }
        self.out(port);
    }

    fn tick(&mut self, port: &mut Port) {
        self.engine.tick();
        while let Some(bytes) = self.engine.answer() {
            self.hold(0, bytes, false);
        }
        if let Some((ticks, ..)) = self.held.front_mut() {
            *ticks = ticks.saturating_sub(1);
        }
        self.out(port);
    }

    fn closed(&mut self) {
        self.line.clear();
        self.held.clear();
        self.engine.clear();
    }
}

fn parse_split(value: &str) -> Result<Split> {
    let bad = || Error::Invalid(format!("split={value}: as in 5:10:4 or -1:10:4"));
    let mut fields = value.split(':');
    let mut field = || fields.next().ok_or_else(bad);
    let at = field()?.parse().map_err(|_| bad())?;
    let ticks = field()?.parse().map_err(|_| bad())?;
    let from = field()?.parse().map_err(|_| bad())?;
    Ok(Split { at, ticks, from })
}

fn parts(
    options: &[(String, String)],
) -> Result<(Meter, scpi::Config, Arc<AtomicU32>, Option<Split>)> {
    let mut config = scpi::Config::default().with_eol(b"\r\n");
    let mut split = None;
    let pace = Arc::new(AtomicU32::new(0));
    let mut meter = Meter {
        id: "HP3457A".to_owned(),
        rev: "2.0,8.0".to_owned(),
        opt: "-1".to_owned(),
        preset_plc: 1.0,
        keeps_terminals: false,
        plc: 1.0,
        function: &FUNCTIONS[0],
        rear: false,
        list: Vec::new(),
        place: 0,
        advance: false,
        chan_skew: 0,
        readings: Played::default(),
        hires: Played::default(),
        ranges: Played::default(),
        plcs: Played::default(),
        chans: Played::default(),
        count: 0,
        pace: Arc::clone(&pace),
        real_rate: false,
        channels: 0,
    };
    for (k, v) in options {
        if config.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "id" => v.clone_into(&mut meter.id),
            "rev" => meter.rev = v.replace('_', " "),
            "opt" => meter.opt = v.replace('_', " "),
            "card" => {
                let card = i64::from(parse_number(k, v, 0..=99_999)?);
                meter.opt = sci(card as f64);
                meter.channels = CARDS.iter().find(|c| c.0 == card).map_or(0, |c| c.1);
            }
            "readings" => meter.readings = Played::parse(v),
            "hires" => meter.hires = Played::parse(v),
            "ranges" => meter.ranges = Played::parse(v),
            "plcs" => meter.plcs = Played::parse(v),
            "chans" => meter.chans = Played::parse(v),
            "plc" => {
                meter.preset_plc = v
                    .parse()
                    .ok()
                    .filter(|p| PLCS.contains(p))
                    .ok_or_else(|| Error::Invalid(format!("plc={v}: one of {PLCS:?}")))?;
            }
            "left-rear" => {
                meter.rear = parse_number(k, v, 0..=1)? == 1;
                meter.keeps_terminals = meter.rear;
            }
            "split" => split = Some(parse_split(v)?),
            "real-rate" => meter.real_rate = parse_number(k, v, 0..=1)? == 1,
            "chan-skew" => meter.chan_skew = i64::from(parse_number(k, v, 0..=99)?),
            _ => return Err(Error::Invalid(format!("{NAME} has no option {k}"))),
        }
    }
    let plc = meter.preset_plc;
    meter.set_plc(plc);
    Ok((meter, config, pace, split))
}

/// `oxrok-emu serve` needs a device; `EMU_TCP=only` attaches none.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    parts(options)?;
    Ok(Box::new(NoUsb::new(NAME)))
}

/// The meter on a socket.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    let (meter, config, pace, split) = parts(options)?;
    Ok(Box::new(OnSocket {
        engine: Engine::new(meter, config),
        pace,
        line: Vec::new(),
        held: VecDeque::new(),
        split,
        answers: 0,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn opts(list: &[(&str, &str)]) -> Vec<(String, String)> {
        list.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    struct Rig {
        stream: Box<dyn Stream>,
    }

    impl Rig {
        fn new(list: &[(&str, &str)]) -> Rig {
            Rig { stream: build_tcp(&opts(list)).unwrap() }
        }

        /// What the line gets for `text`, and what was flagged.
        fn say(&mut self, text: &str) -> (String, Vec<String>) {
            let mut port = Port::default();
            let line = format!("{text}\n");
            self.stream.received(line.as_bytes(), &mut port);
            Self::heard(port)
        }

        fn heard(mut port: Port) -> (String, Vec<String>) {
            let sent = port.sent().concat();
            let flagged = port.violations().drain();
            (String::from_utf8_lossy(&sent).into_owned(), flagged)
        }

        fn ask(&mut self, text: &str) -> String {
            let (heard, flagged) = self.say(text);
            assert_eq!(flagged, Vec::<String>::new(), "{text}");
            heard
        }

        fn ticks(&mut self, n: u32) -> String {
            let mut port = Port::default();
            for _ in 0..n {
                self.stream.tick(&mut port);
            }
            Self::heard(port).0
        }
    }

    #[test]
    fn it_says_who_it_is_with_no_star_idn() {
        let mut r = Rig::new(&[]);
        assert_eq!(r.ask("END ALWAYS"), "");
        assert_eq!(r.ask("ID?"), "HP3457A\r\n");
        assert_eq!(r.ask("REV?"), "2.0,8.0\r\n");
        assert_eq!(r.ask("OPT?"), "-1\r\n");
        let (_, flagged) = r.say("*IDN?");
        assert_eq!(flagged.len(), 1, "{flagged:?}");
    }

    #[test]
    fn commands_with_no_question_mark_answer() {
        let mut r = Rig::new(&[("hires", "+5.000000E-08"), ("readings", "+1.000000E+00")]);
        assert_eq!(r.ask("TRIG SGL"), "+1.000000E+00\r\n");
        assert_eq!(r.ask("RMATH HIRES"), "+5.000000E-08\r\n");
        assert_eq!(r.ask("RANGE?"), "+3.000000E+01\r\n");
        assert_eq!(r.ask("?"), "+1.234567E+00\r\n".replace("1.234567", "2.234567"));
    }

    #[test]
    fn a_function_and_terminals_show_in_the_reading() {
        let mut r = Rig::new(&[]);
        assert_eq!(r.ask("TRIG SGL"), "+1.234567E+00\r\n");
        for f in &FUNCTIONS {
            r.ask(f.command);
            assert_eq!(r.ask("RANGE?"), format!("{}\r\n", sci(f.range)));
        }
        r.ask("DCV");
        r.ask("TERM REAR");
        assert_eq!(r.ask("?"), "+1.022346E+02\r\n");
        r.ask("PRESET");
        assert_eq!(r.ask("?"), "+3.234567E+00\r\n");
    }

    #[test]
    fn a_time_goes_up_to_the_next_there_is() {
        let mut r = Rig::new(&[]);
        assert_eq!(r.ask("NPLC?"), "+1.000000E+00\r\n");
        for (asked, got) in [
            ("1E+01", "+1.000000E+01"),
            ("5E+00", "+1.000000E+01"),
            ("2E-01", "+1.000000E+00"),
            ("1E+02", "+1.000000E+02"),
            ("1E-06", "+5.000000E-04"),
        ] {
            r.ask(&format!("NPLC {asked}"));
            assert_eq!(r.ask("NPLC?"), format!("{got}\r\n"));
        }
        // A NaN is an error of the meter's, with what it had kept.
        assert_eq!(r.ask("NPLC NAN"), "");
        assert_eq!(r.ask("NPLC?"), "+5.000000E-04\r\n");
        r.ask("NPLC 1E+02");
        r.ask("PRESET");
        assert_eq!(r.ask("NPLC?"), "+1.000000E+00\r\n");
    }

    #[test]
    fn scan_advance_moves_as_a_reading_goes() {
        let mut r = Rig::new(&[("card", "44491")]);
        assert_eq!(r.ask("OPT?"), "+4.449100E+04\r\n");
        r.ask("TERM REAR");
        r.ask("SLIST ,3,7");
        r.ask("SADV AUTO");
        assert_eq!(r.ask("CHAN?"), "+3.000000E+00\r\n");
        assert_eq!(r.ask("?"), "+1.312346E+02\r\n");
        assert_eq!(r.ask("CHAN?"), "+7.000000E+00\r\n");
        r.ask("?");
        assert_eq!(r.ask("CHAN?"), "+3.000000E+00\r\n");
        r.ask("SADV HOLD");
        r.ask("?");
        assert_eq!(r.ask("CHAN?"), "+3.000000E+00\r\n");
    }

    #[test]
    fn a_channel_a_card_lacks_is_an_error_and_keeps_the_list() {
        let mut r = Rig::new(&[("card", "0")]);
        assert_eq!(r.ask("SLIST ,4294967295"), "");
        assert_eq!(r.ask("CHAN?"), "+0.000000E+00\r\n");
        let mut r = Rig::new(&[("card", "44492")]);
        r.ask("SLIST ,9");
        assert_eq!(r.ask("CHAN?"), "+9.000000E+00\r\n");
        r.ask("SLIST ,10");
        assert_eq!(r.ask("CHAN?"), "+9.000000E+00\r\n");
    }

    #[test]
    fn chan_skew_is_on_the_answer() {
        let mut r = Rig::new(&[("card", "44491"), ("chan-skew", "2")]);
        r.ask("SLIST ,1");
        assert_eq!(r.ask("CHAN?"), "+3.000000E+00\r\n");
        let mut r = Rig::new(&[("chans", "x")]);
        assert_eq!(r.ask("CHAN?"), "x\r\n");
        assert_eq!(r.ask("CHAN?"), "+0.000000E+00\r\n");
    }

    #[test]
    fn lists_play_once_and_a_space_is_written_so() {
        let mut r = Rig::new(&[("readings", "_1.5,,x"), ("ranges", "7"), ("plcs", "bad")]);
        assert_eq!(r.ask("?"), " 1.5\r\n");
        assert_eq!(r.ask("?"), "\r\n");
        assert_eq!(r.ask("?"), "x\r\n");
        assert_eq!(r.ask("?"), "+4.234567E+00\r\n");
        assert_eq!(r.ask("RANGE?"), "7\r\n");
        assert_eq!(r.ask("RANGE?"), "+3.000000E+01\r\n");
        assert_eq!(r.ask("NPLC?"), "bad\r\n");
        assert_eq!(r.ask("NPLC?"), "+1.000000E+00\r\n");
    }

    #[test]
    fn real_rate_holds_a_measurement_and_what_follows() {
        let mut r = Rig::new(&[("real-rate", "1"), ("plc", "10")]);
        assert_eq!(r.ask("TRIG SGL"), "");
        // 2 * 10 PLC at 50 Hz is 400 ms, or 20 ticks.
        assert_eq!(r.ticks(19), "");
        assert_eq!(r.ticks(1), "+1.234567E+00\r\n");
        assert_eq!(r.ask("RANGE?"), "+3.000000E+01\r\n");
        // A question behind a measurement waits for it.
        assert_eq!(r.ask("?"), "");
        assert_eq!(r.ask("RANGE?"), "");
        assert_eq!(r.ticks(20), "+2.234567E+00\r\n+3.000000E+01\r\n");
    }

    #[test]
    fn a_trigger_with_a_measurement_under_way_is_flagged() {
        let mut r = Rig::new(&[("real-rate", "1")]);
        assert_eq!(r.ask("TRIG SGL"), "");
        let (_, flagged) = r.say("?");
        assert_eq!(flagged, ["?: a trigger with a measurement under way"]);
    }

    #[test]
    fn a_close_drops_what_it_was_to_say_and_keeps_the_settings() {
        let mut r = Rig::new(&[("real-rate", "1")]);
        r.ask("NPLC 1E+01");
        assert_eq!(r.ask("TRIG SGL"), "");
        r.stream.closed();
        assert_eq!(r.ticks(40), "");
        assert_eq!(r.ask("NPLC?"), "+1.000000E+01\r\n");
    }

    #[test]
    fn terminals_left_at_the_rear_outlive_a_preset() {
        let mut r = Rig::new(&[("left-rear", "1")]);
        r.ask("PRESET");
        assert_eq!(r.ask("?"), "+1.012346E+02\r\n");
        r.ask("TERM FRONT");
        r.ask("PRESET");
        assert_eq!(r.ask("?"), "+2.234567E+00\r\n");
    }

    #[test]
    fn an_answer_goes_in_two_pieces_from_its_turn() {
        let mut r = Rig::new(&[("split", "5:10:2")]);
        assert_eq!(r.ask("ID?"), "HP3457A\r\n");
        assert_eq!(r.ask("?"), "+1.23");
        assert_eq!(r.ticks(9), "");
        assert_eq!(r.ticks(1), "4567E+00\r\n");
        let mut r = Rig::new(&[("split", "-1:3:1")]);
        assert_eq!(r.ask("ID?"), "HP3457A\r");
        assert_eq!(r.ask("RANGE?"), "");
        assert_eq!(r.ticks(3), "\n+3.000000E+01\r");
        assert_eq!(r.ticks(3), "\n");
        assert!(parse_split("5:10").is_err());
        assert!(parse_split("x:1:1").is_err());
    }

    #[test]
    fn a_trigger_behind_a_piece_is_no_trigger_too_soon() {
        let mut r = Rig::new(&[("split", "5:10:1")]);
        assert_eq!(r.ask("?"), "+1.23");
        let (heard, flagged) = r.say("?");
        assert_eq!((heard.as_str(), flagged.len()), ("", 0));
    }

    #[test]
    fn what_has_no_command_or_argument_is_flagged() {
        let mut r = Rig::new(&[]);
        for text in
            ["END SOMETIMES", "INBUF OFF", "TRIG EXT", "RMATH MEAN", "TERM SIDE", "SADV ON", "XYZ"]
        {
            let (_, flagged) = r.say(text);
            assert_eq!(flagged.len(), 1, "{text}: {flagged:?}");
        }
    }

    #[test]
    fn options_are_checked() {
        for bad in [("plc", "2"), ("real-rate", "2"), ("chan-skew", "x"), ("nope", "1")] {
            assert!(parts(&opts(&[bad])).is_err(), "{bad:?}");
        }
    }
}
