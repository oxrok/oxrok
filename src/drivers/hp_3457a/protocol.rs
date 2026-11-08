// Counterpart in sigrok:
//   libsigrok/src/hardware/hp-3457a/protocol.c,
//   libsigrok/src/hardware/hp-3457a/protocol.h,
//   Copyright (C) 2016 Alexandru Gagniuc <mr.nuke.me@gmail.com>,
//   with commits by Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The HP 3457A's rules, with no I/O: numbers, commands, digits, the steps of a
//! measurement, and how long a meter may say nothing.
//!
//! A measurement is one trigger (`TRIG SGL`, then `?`) and its reading; `RMATH HIRES`,
//! whose answer adds to the reading, at 10 PLC and over; and `RANGE?`, which says how
//! many of the digits count. Each ninth sample on a scan of several channels `CHAN?`
//! checks that the scan advance is where it should be.
//!
//! Nothing in an answer says what it answers, so one answer goes with one question:
//! a question goes only when the one before has its answer, and a wait goes on as
//! long as the bound allows. A trigger with a measurement under way would earn a
//! "Trig too fast" error, by sigrok's wiki, and then two answers for one read: the
//! second would be taken for the range.

use crate::core::{Mq, MqFlags, Unit};
use std::time::{Duration, Instant};

/// Consecutive answers with no sample that end a capture: a quicker failure than
/// silence, and it takes a meter that answers and makes no sense.
pub const UNUSABLE_LIMIT: u32 = 16;
/// The most kept of an answer with no line end, on a line that ends them: past this it
/// is no answer of this meter's, and counts as one that makes nothing.
pub const LONGEST_ANSWER: usize = 256;

/// A number as the meter may send one, as `sr_atod_ascii` takes it, leaving out what
/// no meter means by a number: nothing at all (0 in C), hexadecimal, `inf`, `nan`, and
/// what underflows. Leading white space passes and trailing white space does not, as
/// in C. No manual says more of the form, so a digit gone wrong can't be seen here.
pub fn parse_number(text: &str) -> Option<f64> {
    let text = text.trim_start_matches([' ', '\t', '\n', '\r', '\x0b', '\x0c']);
    // Of what the parser takes, only `inf`, `infinity` and `nan` have no digit.
    if !text.bytes().any(|b| b.is_ascii_digit()) {
        return None;
    }
    let value: f64 = text.parse().ok()?;
    let mantissa = text.split(['e', 'E']).next().unwrap_or(text);
    let underflow = value == 0.0 && mantissa.bytes().any(|b| (b'1'..=b'9').contains(&b));
    (value.is_finite() && !underflow).then_some(value)
}

/// C's `(int)` of a double on x86-64: `cvttsd2si` gives `INT_MIN` for what doesn't
/// fit, and for a NaN.
fn c_int(value: f64) -> i32 {
    if value.is_nan() || value <= f64::from(i32::MIN) - 1.0 || value >= f64::from(i32::MAX) + 1.0 {
        i32::MIN
    } else {
        value as i32
    }
}

/// `get_revision`: two numbers make `MAJOR.MINOR`; anything else is `0.0`.
pub fn revision(text: &str) -> String {
    let numbers: Option<Vec<f64>> = text.split(',').map(parse_number).collect();
    match numbers.as_deref() {
        Some([major, minor, ..]) => format!("{}.{}", c_int(*major), c_int(*minor)),
        _ => "0.0".to_owned(),
    }
}

/// `(unsigned int)` of a float that is in range for a `long`, as C on x86-64 has it.
fn c_unsigned(value: f64) -> u32 {
    value as i64 as u32
}

/// A rear option.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Card {
    /// The number `OPT?` gives.
    pub id: u32,
    /// What it is.
    pub name: &'static str,
    /// Its channel group.
    pub group: &'static str,
    /// How many channels.
    pub channels: u32,
}

/// The rear terminals, which any meter without a card has.
pub const REAR_TERMINALS: u32 = 0;

/// `rear_card_parameters`.
pub const CARDS: [Card; 3] = [
    Card { id: REAR_TERMINALS, name: "Rear terminals", group: "rear", channels: 1 },
    Card { id: 44491, name: "44491A Armature Relay Multiplexer", group: "44491a", channels: 14 },
    Card { id: 44492, name: "44492A Reed Relay Multiplexer", group: "44492a", channels: 10 },
];

/// The card `OPT?` names. An answer that is no number names none, where C reads an
/// empty one, and `inf` and `nan`, as card 0.
pub fn find_card(text: &str) -> Option<&'static Card> {
    let id = c_unsigned(parse_number(text)?);
    CARDS.iter().find(|c| c.id == id)
}

/// `sr_mq_to_cmd_map` and its helpers: the command for a quantity, and its unit.
pub fn mq_command(mq: Mq, flags: MqFlags) -> Option<(String, Unit)> {
    let ac_dc = |mode: &str, unit| {
        if flags.iter().any(|f| f != MqFlags::AC && f != MqFlags::DC) {
            return None;
        }
        let ac = if flags.contains(MqFlags::AC) { "AC" } else { "" };
        // It must say DC when it doesn't say AC.
        let dc =
            if flags.contains(MqFlags::DC) || !flags.contains(MqFlags::AC) { "DC" } else { "" };
        Some((format!("{ac}{dc}{mode}"), unit))
    };
    match mq {
        Mq::Voltage => ac_dc("V", Unit::Volt),
        Mq::Current => ac_dc("I", Unit::Ampere),
        Mq::Resistance => {
            if flags.iter().any(|f| f != MqFlags::FOUR_WIRE) {
                return None;
            }
            let four = if flags.contains(MqFlags::FOUR_WIRE) { "F" } else { "" };
            Some((format!("OHM{four}"), Unit::Ohm))
        }
        Mq::Frequency => Some(("FREQ".to_owned(), Unit::Hertz)),
        _ => None,
    }
}

/// `hp_3457a_set_nplc`'s check and command: `NPLC %.0E`, or `None` for what it
/// refuses. The value is a float there, so 1E-6 itself falls under its floor. A NaN
/// passes both of C's comparisons and goes out as `NPLC NAN`; not here.
pub fn plc_command(plc: f32) -> Option<String> {
    let wide = f64::from(plc);
    if wide.is_nan() || wide < 1E-6 || wide > 100.0 {
        return None;
    }
    let text = format!("{wide:.0e}");
    let (mantissa, exponent) = text.split_once('e')?;
    let exponent: i32 = exponent.parse().ok()?;
    Some(format!("NPLC {mantissa}E{exponent:+03}"))
}

/// `is_highres_enabled`: the register holds something from 10 PLC.
pub fn high_resolution(plc: Option<f32>) -> bool {
    plc.is_some_and(|p| f64::from(p) >= 10.0)
}

/// `calculate_num_zero_digits`, with C's conversions on x86-64. A reading below zero
/// has a ratio below zero, whose logarithm is NaN or minus infinity, and `(int)` of
/// those is `INT_MIN`, whose low byte is 0: such a reading keeps all its digits.
fn zero_digits(measurement: f64, range: f64) -> i32 {
    let min_full_res = 10f64.powf(f64::from(c_int(range.log10())));
    if measurement > min_full_res || measurement == 0.0 {
        return 0;
    }
    c_int((min_full_res / measurement).ceil().log10().ceil())
}

/// The digits of a sample. C's `int` goes to `int8_t` in `sr_analog_init`.
pub fn digits(high_resolution: bool, measurement: f64, range: f64) -> i32 {
    let most: i32 = if high_resolution { 8 } else { 7 };
    i32::from(most.wrapping_sub(zero_digits(measurement, range)) as i8)
}

/// A sample, for a channel of the list a capture got.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// The place in that list.
    pub place: usize,
    /// The value; C sends a float for now, as its output modules take no double.
    pub value: f32,
    /// Its digits.
    pub digits: i32,
}

/// What comes after an answer.
#[derive(Clone, Debug, PartialEq)]
pub enum Then {
    /// This goes out, and its answer is waited for.
    Ask(&'static str),
    /// The limit is met.
    Done,
    /// The capture ends in failure: lines to log, the first being its message.
    Fail(Vec<String>),
}

/// Which answer is awaited.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Awaited {
    Reading,
    Hires,
    Range,
    Channel,
}

/// `hp_3457a_receive_data`, an answer at a time.
#[derive(Debug)]
pub struct Steps {
    awaited: Awaited,
    high_resolution: bool,
    /// What `CHAN?` should say of each channel in turn: its number on the card. The
    /// rear terminals are one channel, which never gets asked.
    list: Vec<i32>,
    place: usize,
    limit: u64,
    samples: u64,
    base: f64,
    hires: f64,
    unusable: u32,
}

impl Steps {
    /// After the first trigger.
    pub fn new(high_resolution: bool, list: Vec<i32>, limit: u64) -> Steps {
        Steps {
            awaited: Awaited::Reading,
            high_resolution,
            list,
            place: 0,
            limit,
            samples: 0,
            base: 0.0,
            hires: 0.0,
            unusable: 0,
        }
    }

    /// `retrigger_measurement` for an answer that makes nothing.
    fn unusable(&mut self, text: &str) -> Then {
        self.unusable += 1;
        if self.unusable >= UNUSABLE_LIMIT {
            let shown: String = text.chars().take(40).collect();
            return Then::Fail(vec![format!(
                "No usable answer from the meter in {UNUSABLE_LIMIT} tries; the last was '{shown}'."
            )]);
        }
        self.awaited = Awaited::Reading;
        Then::Ask("?")
    }

    /// Bytes came with nothing asked, so something is out of step: this measurement
    /// starts again, and it counts as an answer that makes nothing.
    pub fn stray(&mut self, bytes: usize) -> Then {
        self.unusable(&format!("{bytes} bytes unasked"))
    }

    /// What an answer comes to.
    pub fn answer(&mut self, text: &str) -> (Option<Sample>, Then) {
        let number = parse_number(text);
        match self.awaited {
            Awaited::Reading => match number {
                None => (None, self.unusable(text)),
                Some(value) => {
                    self.base = value;
                    if self.high_resolution {
                        self.awaited = Awaited::Hires;
                        (None, Then::Ask("RMATH HIRES"))
                    } else {
                        self.awaited = Awaited::Range;
                        (None, Then::Ask("RANGE?"))
                    }
                }
            },
            Awaited::Hires => match number {
                None => (None, self.unusable(text)),
                Some(value) => {
                    self.hires = value;
                    self.awaited = Awaited::Range;
                    (None, Then::Ask("RANGE?"))
                }
            },
            Awaited::Range => match number {
                None => (None, self.unusable(text)),
                Some(range) => self.sample(range),
            },
            Awaited::Channel => match number {
                None => {
                    (None, Then::Fail(vec!["Cannot check channel synchronization.".to_owned()]))
                }
                Some(channel) => (None, self.channel_said(channel)),
            },
        }
    }

    fn sample(&mut self, range: f64) -> (Option<Sample>, Then) {
        let mut value = self.base;
        if self.high_resolution {
            value += self.hires;
        }
        let sample = Sample {
            place: self.place,
            value: value as f32,
            digits: digits(self.high_resolution, value, range),
        };
        self.samples += 1;
        self.unusable = 0;
        if self.limit != 0 && self.samples >= self.limit {
            return (Some(sample), Then::Done);
        }
        self.place = (self.place + 1) % self.list.len().max(1);
        if self.samples % 10 == 9 && self.list.len() > 1 {
            self.awaited = Awaited::Channel;
            return (Some(sample), Then::Ask("CHAN?"));
        }
        self.awaited = Awaited::Reading;
        (Some(sample), Then::Ask("?"))
    }

    /// Exactly equal, as in C: `CHAN?` says a whole number or something is wrong.
    #[allow(clippy::float_cmp)]
    fn channel_said(&mut self, said: f64) -> Then {
        let expected = self.list.get(self.place).copied().unwrap_or(-1);
        if f64::from(expected) != said {
            return Then::Fail(vec![
                "Current channel and scan advance out of sync.".to_owned(),
                format!(
                    "Expected channel {}, but device says {}",
                    expected as u32,
                    c_unsigned(said)
                ),
            ]);
        }
        self.awaited = Awaited::Reading;
        Then::Ask("?")
    }
}

/// How long a meter may say nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bounds {
    /// With no sample yet, under a sample limit: a failure. The slowest pace known is
    /// reckoned, not read: 100 PLC with auto zero at 50 Hz is 4 s, and no manual could
    /// be read for what autorange or a card's relays add.
    pub first: Duration,
    /// The warning, in any capture.
    pub warn: Duration,
    /// With a sample, under a sample limit: a failure.
    pub give_up: Duration,
    /// How long a stopped capture's answer is waited for ahead of what comes next.
    pub owed: Duration,
    /// How long a question at a scan or an open is waited for. A close leaves the meter
    /// on `TRIG AUTO` at the time it had, and whether a measurement holds up a question
    /// isn't known: this is a few of the slowest measurements reckoned above, which C's
    /// blocking read on TCP also sits out.
    pub query: Duration,
    /// How long `NPLC?` is waited for after a setting, with the meter on `TRIG HOLD`.
    pub setting: Duration,
    /// How long the rest of an answer is waited for on a line shown to end no answers.
    pub tail: Duration,
}

impl Default for Bounds {
    fn default() -> Self {
        Bounds {
            first: Duration::from_secs(60),
            warn: Duration::from_secs(60),
            give_up: Duration::from_secs(600),
            owed: Duration::from_secs(10),
            query: Duration::from_secs(30),
            setting: Duration::from_secs(5),
            tail: Duration::from_millis(50),
        }
    }
}

/// The most of a silence excused as time the thread did not run.
const CREDIT_CAP: Duration = Duration::from_secs(30);

/// What a silence comes to.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Silent {
    /// Nothing yet.
    Wait,
    /// Say this, once, and go on.
    Warn(String),
    /// The capture ends in failure with this.
    GiveUp(String),
}

/// How long since the meter was last asked, over `Instant`s the caller takes.
#[derive(Debug)]
pub struct Silence {
    bounds: Bounds,
    /// Only a limit on samples ends this capture, so a silence has to.
    limited: bool,
    had_sample: bool,
    since: Instant,
    last_look: Instant,
    credit: Duration,
    warned: bool,
}

impl Silence {
    /// As a capture starts.
    pub fn new(bounds: Bounds, limited: bool, now: Instant) -> Silence {
        Silence {
            bounds,
            limited,
            had_sample: false,
            since: now,
            last_look: now,
            credit: Duration::ZERO,
            warned: false,
        }
    }

    /// A question went out; `now` is taken after its send, and after a sample's send
    /// to a client that may be slow, neither being a silence of the meter's.
    pub fn asked(&mut self, now: Instant) {
        self.since = now;
        self.last_look = now;
        self.credit = Duration::ZERO;
    }

    /// An answer came. What to say if the silence had been told.
    pub fn heard(&mut self, now: Instant, sample: bool) -> Option<String> {
        self.had_sample |= sample;
        let was = std::mem::take(&mut self.warned);
        was.then(|| {
            let after = now.saturating_duration_since(self.since).as_secs();
            format!("The meter answers again, after {after} s.")
        })
    }

    /// A read that started at `looked` and waited up to `wait` found nothing. `looked`
    /// comes ahead of the read: what an empty read says holds from there. A gap since
    /// the last look beyond a read's wait is the thread not running, up to a limit.
    pub fn empty_read(&mut self, looked: Instant, wait: Duration) -> Silent {
        let gap = looked.saturating_duration_since(self.last_look);
        if let Some(lost) = gap.checked_sub(wait * 2) {
            self.credit = (self.credit + lost).min(CREDIT_CAP);
        }
        self.last_look = looked;
        let quiet = looked.saturating_duration_since(self.since).saturating_sub(self.credit);
        if self.limited {
            let bound = if self.had_sample { self.bounds.give_up } else { self.bounds.first };
            if quiet >= bound {
                return Silent::GiveUp(format!(
                    "No answer from the meter in {} s.",
                    bound.as_secs()
                ));
            }
        }
        if quiet >= self.bounds.warn && !self.warned {
            self.warned = true;
            return Silent::Warn(format!(
                "The meter has said nothing for {} s; still waiting.",
                self.bounds.warn.as_secs()
            ));
        }
        Silent::Wait
    }
}
