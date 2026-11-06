// Counterpart in sigrok:
//   libsigrok/src/hardware/fluke-45/protocol.c,
//   libsigrok/src/hardware/fluke-45/protocol.h,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2017 John Chajecki <subs@qcontinuum.plus.com>,
//   with commits by JohnCh, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! What a Fluke 45's lines mean and what a capture does about them, with no port and
//! no clock of its own.
//!
//! The users manual (PN 855981, chapter 5): an answer ends in CR LF, and `=>` says
//! the command ran and the interface is ready. A host must "wait for the meter to
//! return a prompt before sending another command", or the second string is a command
//! error and is discarded. C doesn't wait. `!>` and `?>` are C's, for what could not
//! be done, and an Execution Error is the manual's word for `FUNC2?` with the second
//! display off.
//!
//! So a query here is an exchange: an answer, then the prompt that closes it, read
//! line by line whatever pieces they came in. Nothing but the prompts says which
//! question a line answers: see [`Requests`].
//!
//! A line has no checksum, and parity isn't looked at. The form of an answer is all
//! there is to check: a flipped bit that leaves a digit a digit gives a wrong reading.

use crate::core::{Mq, MqFlags, Unit};
use std::time::{Duration, Instant};

/// How long an answer may take to start, as `sr_scpi_get_string` in C, while the
/// meter has been answering. A prompt gets as long.
pub const FIRST_WAIT: Duration = Duration::from_secs(1);
/// How long the wait for an answer gets, doubling while nothing is heard.
pub const LONGEST_WAIT: Duration = Duration::from_secs(30);
/// How long a prompt is waited for, once two in a row did not come: a meter that
/// sends none is not waited for on each question.
pub const SHORT_PROMPT_WAIT: Duration = Duration::from_millis(50);
/// A meter that goes unheard this long is told. On a sample limit alone and with no
/// sample yet, it ends the capture. Slow is 2.5 readings a second by the manual, and
/// a blank display waits for the next measurement: this has room for a range to
/// settle, and a meter on an external trigger has `--time` or `--continuous`.
pub const NO_ANSWER: Duration = Duration::from_secs(30);
/// Bytes that make nothing, with no sample yet: a quicker failure than silence.
pub const NO_USABLE: Duration = Duration::from_secs(10);
/// What else ends a capture on a sample limit alone: a meter that had a sample or
/// that answers well and gives no reading.
pub const GIVE_UP: Duration = Duration::from_secs(600);
/// The most questions counted as out. A dead meter is asked again and again, and
/// the one that comes back owes little of it.
pub const MOST_OWED: u32 = 3;
/// How soon after its question a line is too soon to be another's.
pub const QUICK_LINE: Duration = Duration::from_millis(100);
/// With no answer taken for this long, what is out is written off: under [`NO_ANSWER`],
/// so that a question lost at the start doesn't fail a capture.
pub const LOST_AFTER: Duration = Duration::from_secs(20);
/// The longest line kept: `FLUKEDMM_BUFSIZE`.
pub const LONGEST_LINE: usize = 256;

/// What a kind of question may be answered with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// `*IDN?`.
    Id,
    /// `FUNC1?`.
    Function,
    /// `AUTO?`.
    Flag,
    /// `VAL1?`.
    Reading,
    /// `MOD?`.
    Modifiers,
}

impl Kind {
    /// Whether `line` can be the answer to this. A late answer to another question is
    /// no answer to this: of the questions that follow each other in a round, no two
    /// but `FUNC1?` and `FUNC2?` have one form.
    pub fn fits(self, line: &str) -> bool {
        match self {
            Kind::Id => line.split(',').count() >= 3,
            Kind::Function => function(line).is_some(),
            Kind::Flag => line == "0" || line == "1",
            Kind::Reading => reading(line).is_some(),
            Kind::Modifiers => modifiers(line).is_some(),
        }
    }
}

/// What a function's reading is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Label {
    /// The quantity.
    pub mq: Mq,
    /// The unit.
    pub unit: Unit,
    /// The flags.
    pub flags: MqFlags,
}

/// `fl45_get_status`'s table. `AACDC` says AC and DC, as `VACDC` does, where C has AC
/// alone (`protocol.c:68-71`). `CONT` has its unit on its own channel, where C sets
/// the first channel's (`protocol.c:78`).
pub fn function(word: &str) -> Option<Label> {
    let (mq, unit, flags) = match word {
        "AAC" => (Mq::Current, Unit::Ampere, MqFlags::AC),
        "AACDC" => (Mq::Current, Unit::Ampere, MqFlags::AC | MqFlags::DC),
        "ADC" => (Mq::Current, Unit::Ampere, MqFlags::DC),
        "CONT" => (Mq::Continuity, Unit::Boolean, MqFlags::empty()),
        "DIODE" => (Mq::Voltage, Unit::Volt, MqFlags::DIODE),
        "FREQ" => (Mq::Frequency, Unit::Hertz, MqFlags::empty()),
        "OHMS" => (Mq::Resistance, Unit::Ohm, MqFlags::empty()),
        "VAC" => (Mq::Voltage, Unit::Volt, MqFlags::AC),
        "VACDC" => (Mq::Voltage, Unit::Volt, MqFlags::AC | MqFlags::DC),
        "VDC" => (Mq::Voltage, Unit::Volt, MqFlags::DC),
        _ => return None,
    };
    Some(Label { mq, unit, flags })
}

/// A value and how many of its decimals mean anything.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// The value, infinite for an overload.
    pub value: f32,
    /// `get_reading_dd`.
    pub digits: i32,
}

/// What C's overload is, compared whole (`protocol.c:265`).
const OVERLOAD: &str = "+1E+9";

/// `VAL1?`'s answer. The manual's form is `+1.2345E+0`: a sign, digits with a point,
/// and an exponent. Refused is what isn't that, where `sr_atof_ascii` takes hex,
/// `inf` and `nan`, and where it fails C sends what the stack holds
/// (`protocol.c:272`). The sign is not asked for, as only examples show it, and a
/// point isn't, as C's overload has none. The exponent is: it's what tells a reading
/// from `AUTO?`'s or `MOD?`'s late answer.
pub fn reading(line: &str) -> Option<Reading> {
    if line == OVERLOAD {
        return Some(Reading { value: f32::INFINITY, digits: 0 });
    }
    let body = line.strip_prefix(['+', '-']).unwrap_or(line);
    let (mantissa, exponent) = body.split_once('E')?;
    let (whole, fraction) = mantissa.split_once('.').unwrap_or((mantissa, ""));
    let digits = |s: &str| s.bytes().all(|b| b.is_ascii_digit());
    let sign_less = exponent.strip_prefix(['+', '-']).unwrap_or(exponent);
    if whole.is_empty() || !digits(whole) || !digits(fraction) {
        return None;
    }
    if sign_less.is_empty() || !digits(sign_less) {
        return None;
    }
    #[expect(clippy::cast_possible_truncation, reason = "C's float, from strtod's double")]
    let value = line.parse::<f64>().ok()? as f32;
    value.is_finite().then(|| Reading { value, digits: reading_digits(line) })
}

/// `get_reading_dd`, as it is. A number with no point counts from the start of the
/// text, sign and all, so `+5E-1` has 2 digits. Two characters of an exponent are
/// read, so `E+10` is 1: the meter's reach ends at 9.
fn reading_digits(line: &str) -> i32 {
    let bytes = line.as_bytes();
    let (mut point, mut e) = (0, 0);
    for (i, b) in bytes.iter().enumerate() {
        if *b == b'.' {
            point = i;
        }
        if *b == b'E' {
            e = i;
            break;
        }
    }
    let exponent = bytes.get(e + 1..).map_or(&b""[..], |rest| &rest[..rest.len().min(2)]);
    let exponent = std::str::from_utf8(exponent).ok().and_then(|s| s.parse::<i32>().ok());
    let span = i32::try_from(e).unwrap_or(0) - i32::try_from(point).unwrap_or(0);
    span - 1 - exponent.unwrap_or(0)
}

/// `MOD?`'s answer, the manual's sum: 127 at the most. Refused is what isn't one,
/// where `atoi` makes 0 of it, and no flags.
pub fn modifiers(line: &str) -> Option<u32> {
    let all_digits = !line.is_empty() && line.bytes().all(|b| b.is_ascii_digit());
    all_digits.then(|| line.parse().ok()).flatten().filter(|m| *m <= 127)
}

/// `fl45_get_modifiers`, on a reading's label and digits. REL says relative, where C
/// says hold (`protocol.c:164`); COMP, 64, says nothing, as in C. dB's quantity is C's
/// (`protocol.c:148`), a power factor: wrong, and no output shows it.
pub fn apply_modifiers(mods: u32, label: &mut Label, digits: &mut i32) {
    if mods & 0x01 != 0 {
        label.flags |= MqFlags::MIN;
    }
    if mods & 0x02 != 0 {
        label.flags |= MqFlags::MAX;
    }
    if mods & 0x04 != 0 {
        label.flags |= MqFlags::HOLD;
    }
    if mods & 0x08 != 0 {
        label.mq = Mq::PowerFactor;
        label.unit = Unit::DecibelMw;
        label.flags = MqFlags::empty();
        *digits = 2;
    }
    if mods & 0x10 != 0 {
        label.mq = Mq::Power;
        label.unit = Unit::DecibelSpl;
        label.flags = MqFlags::empty();
        *digits = 2;
    }
    if mods & 0x20 != 0 {
        label.flags |= MqFlags::RELATIVE;
    }
}

/// What a question came to.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Reply {
    /// This answer, of the kind asked.
    Answer(String),
    /// `!>` or `?>`: the meter says it can't, and is not silent.
    Refused,
    /// Nothing for the whole wait.
    Silent,
    /// Lines that made nothing, and then nothing for the whole wait.
    Unusable,
}

/// What a line came to, while a question is out.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Step {
    /// The question is closed.
    Done(Reply),
    /// An answer of the kind asked, and its prompt is to come.
    Good,
    /// A prompt ahead of an answer, or an older question's line. Says nothing of
    /// whether the meter makes sense, and the wait goes on.
    Passed,
    /// Not of the kind asked, and not an older question's.
    Junk,
}

/// One question at a time, closed by its prompt, as a value over `Instant`.
///
/// What is out is counted, a prompt closing the oldest. While more than one is out, a
/// line is an older question's and is dropped, so a late answer to a question given
/// up on doesn't pass for the next one's, as in C, where it shifts every later
/// answer, for good. A line of the wrong kind is dropped too, with the wait going on,
/// and that brings a line one question behind back in step, as the right answer
/// comes behind it.
///
/// The wait for an answer starts at C's second and doubles at each that runs out, to
/// [`LONGEST_WAIT`], and each line starts it again. It's back at the first only after
/// two answers in a row within it: one quick answer may be an older question's.
///
/// A lost question would have every later answer taken for an older one's. Two lines
/// in a row dropped for that within [`QUICK_LINE`] of their own question say so: what
/// is out is written off, and the second is taken. One proves nothing, as a question
/// can go just ahead of a late answer. Failing that, with no answer taken for
/// [`LOST_AFTER`], what is out is written off at the next question, and [`MOST_OWED`]
/// keeps a meter that was dead a while from owing more than that. A rule that wrote
/// off after one quiet wait took a simulated meter's answers at 2.5 s for the next
/// question's, for good.
///
/// A line of the wrong kind whose prompt comes has had its say: no question is left
/// out for it, and its end doesn't lengthen the wait.
///
/// A prompt nobody sees closes nothing, and an answer is good without one: after
/// [`FIRST_WAIT`], and [`SHORT_PROMPT_WAIT`] once two in a row went unseen.
#[derive(Debug)]
pub struct Requests {
    owed: u32,
    wait: Duration,
    quick: u32,
    missed_prompts: u32,
    kind: Kind,
    answer: Option<String>,
    junk: bool,
    junk_closed: bool,
    quick_older: u32,
    sent: Instant,
    heard: Instant,
    taken: Instant,
}

impl Requests {
    /// With nothing out.
    pub fn new(now: Instant) -> Requests {
        Requests {
            owed: 0,
            wait: FIRST_WAIT,
            quick: 0,
            missed_prompts: 0,
            kind: Kind::Function,
            answer: None,
            junk: false,
            junk_closed: false,
            quick_older: 0,
            sent: now,
            heard: now,
            taken: now,
        }
    }

    /// A question of this kind has gone, its write having returned.
    pub fn sent(&mut self, kind: Kind, now: Instant) {
        if now.saturating_duration_since(self.taken) >= LOST_AFTER {
            self.owed = 0;
            self.taken = now;
        }
        self.owed = (self.owed + 1).min(MOST_OWED);
        self.kind = kind;
        self.answer = None;
        self.junk = false;
        self.junk_closed = false;
        self.sent = now;
        self.heard = now;
    }

    /// How many are out.
    pub fn owed(&self) -> u32 {
        self.owed
    }

    /// When a line last came, or the question went.
    pub fn heard_at(&self) -> Instant {
        self.heard
    }

    /// A closing prompt came: it closes the oldest out, and whether that left none.
    fn closes(&mut self) -> bool {
        if self.owed > 1 {
            self.owed -= 1;
            false
        } else {
            self.owed = 0;
            true
        }
    }

    /// A whole line, its line end gone, came.
    pub fn line(&mut self, line: &str, now: Instant) -> Step {
        self.heard = now;
        match line {
            "=>" => {
                self.missed_prompts = 0;
                if self.owed > 1 {
                    self.closes();
                    return Step::Passed;
                }
                // C's rule: a prompt ahead of the answer is ignored, for the tail of
                // what was before, here that of a scan or a stop.
                if let Some(answer) = self.answer.take() {
                    self.closes();
                    self.settle_wait(now);
                    Step::Done(Reply::Answer(answer))
                } else {
                    self.junk_closed = self.junk;
                    Step::Passed
                }
            }
            "!>" | "?>" => {
                self.missed_prompts = 0;
                if self.owed > 1 && self.second_quick_older(now) {
                    self.owed = 1;
                }
                if !self.closes() {
                    return Step::Passed;
                }
                self.settle_wait(now);
                match self.answer.take() {
                    Some(answer) => Step::Done(Reply::Answer(answer)),
                    None => Step::Done(Reply::Refused),
                }
            }
            _ if self.owed > 1 => {
                if !self.second_quick_older(now) || !self.kind.fits(line) {
                    return Step::Passed;
                }
                self.owed = 1;
                self.answer = Some(line.to_owned());
                Step::Good
            }
            _ if self.answer.is_none() && self.kind.fits(line) => {
                self.answer = Some(line.to_owned());
                Step::Good
            }
            _ => {
                self.junk = true;
                Step::Junk
            }
        }
    }

    /// A line that would be an older question's came; whether it's the second in a
    /// row too soon after its own question to be one.
    fn second_quick_older(&mut self, now: Instant) -> bool {
        if now.saturating_duration_since(self.sent) > QUICK_LINE {
            self.quick_older = 0;
            return false;
        }
        self.quick_older += 1;
        if self.quick_older < 2 {
            return false;
        }
        self.quick_older = 0;
        true
    }

    fn settle_wait(&mut self, now: Instant) {
        self.taken = now;
        self.quick_older = 0;
        if now.saturating_duration_since(self.sent) <= FIRST_WAIT {
            self.quick += 1;
            if self.quick >= 2 {
                self.wait = FIRST_WAIT;
            }
        } else {
            self.quick = 0;
        }
    }

    /// How long the line may stay quiet now.
    pub fn quiet_allowed(&self) -> Duration {
        if self.answer.is_none() {
            self.wait
        } else if self.missed_prompts >= 2 {
            SHORT_PROMPT_WAIT
        } else {
            FIRST_WAIT
        }
    }

    /// A read that started at `looked` found nothing: the question's end, if its time
    /// is up.
    pub fn expired(&mut self, looked: Instant) -> Option<Reply> {
        if looked.saturating_duration_since(self.heard) < self.quiet_allowed() {
            return None;
        }
        Some(self.give_up())
    }

    /// The question ends here, whatever its wait says.
    pub fn give_up(&mut self) -> Reply {
        if let Some(answer) = self.answer.take() {
            // Its prompt went unseen, and comes to nothing later: a prompt with no
            // answer ahead of it is ignored.
            self.missed_prompts += 1;
            self.owed = 0;
            self.taken = self.heard;
            return Reply::Answer(answer);
        }
        if self.junk_closed {
            self.owed = 0;
            return Reply::Unusable;
        }
        self.wait = (self.wait * 2).min(LONGEST_WAIT);
        self.quick = 0;
        if self.junk { Reply::Unusable } else { Reply::Silent }
    }
}

/// What can end a capture, as far as a silence goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Limits {
    /// Only a sample limit could end it: a meter that stops must be given up on.
    pub samples_only: bool,
    /// The time limit in ms, 0 being none.
    pub msec: u64,
}

/// What has been heard since the last sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Heard {
    Nothing,
    Junk,
    Answers,
}

/// What to do about how long nothing came.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Next {
    /// Go on.
    Wait,
    /// Say once that nothing usable has come, and go on.
    Warn,
    /// The capture failed.
    GiveUp(GiveUp),
}

/// Why a capture gave up, each in its own words.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GiveUp {
    /// Not a line.
    NoAnswer(Duration),
    /// Lines, and none of use.
    NoUsable(Duration),
    /// Well-formed answers, and no reading.
    NoReading(Duration),
}

impl GiveUp {
    /// What a capture that gave up says.
    pub fn message(self) -> String {
        match self {
            GiveUp::NoAnswer(after) => {
                format!("no answer from the meter for {} s", after.as_secs_f32())
            }
            GiveUp::NoUsable(after) => {
                format!("no usable answer from the meter for {} s", after.as_secs_f32())
            }
            GiveUp::NoReading(after) => {
                format!("no reading from the meter for {} s", after.as_secs_f32())
            }
        }
    }
}

/// The times of a [`Watch`], which a test may want shorter.
#[derive(Clone, Copy, Debug)]
pub struct Timing {
    /// [`NO_ANSWER`].
    pub no_answer: Duration,
    /// [`NO_USABLE`].
    pub no_usable: Duration,
    /// [`GIVE_UP`].
    pub give_up: Duration,
}

impl Default for Timing {
    fn default() -> Timing {
        Timing { no_answer: NO_ANSWER, no_usable: NO_USABLE, give_up: GIVE_UP }
    }
}

/// How long a capture goes with no sample, and what it does then. C has no bound: a
/// silent meter is asked for ever and a `--samples` capture never ends, unless the
/// meter answered `FUNC1?` first, and then C crashes (see `docs/DEVICES.md`).
#[derive(Debug)]
pub struct Watch {
    timing: Timing,
    limits: Limits,
    started: Instant,
    usable: Instant,
    heard: Heard,
    sampled: bool,
    warned: bool,
}

impl Watch {
    /// As a capture starts.
    pub fn new(timing: Timing, limits: Limits, now: Instant) -> Watch {
        Watch {
            timing,
            limits,
            started: now,
            usable: now,
            heard: Heard::Nothing,
            sampled: false,
            warned: false,
        }
    }

    /// A line came, of use or not.
    pub fn heard(&mut self, step: &Step) {
        let now = match step {
            Step::Passed => return,
            Step::Junk => Heard::Junk,
            Step::Good | Step::Done(_) => Heard::Answers,
        };
        self.heard = self.heard.max(now);
    }

    /// A reading was handed on, which may have taken a slow client some time: none of
    /// that counts against the meter. Whether it had been told otherwise.
    pub fn delivered(&mut self, now: Instant) -> bool {
        self.usable = now;
        self.heard = Heard::Nothing;
        self.sampled = true;
        std::mem::take(&mut self.warned)
    }

    /// What to do, with nothing usable from `self.usable` to `now`.
    pub fn check(&mut self, now: Instant) -> Next {
        let silence = now.saturating_duration_since(self.usable);
        if self.limits.samples_only {
            let bound = match (self.sampled, self.heard) {
                (false, Heard::Nothing) => GiveUp::NoAnswer(self.timing.no_answer),
                (false, Heard::Junk) => GiveUp::NoUsable(self.timing.no_usable),
                (_, Heard::Answers) => GiveUp::NoReading(self.timing.give_up),
                (true, Heard::Nothing) => GiveUp::NoAnswer(self.timing.give_up),
                (true, Heard::Junk) => GiveUp::NoUsable(self.timing.give_up),
            };
            let (GiveUp::NoAnswer(after) | GiveUp::NoUsable(after) | GiveUp::NoReading(after)) =
                bound;
            if silence >= after {
                return Next::GiveUp(bound);
            }
        }
        if silence >= self.timing.no_answer && !self.warned {
            self.warned = true;
            return Next::Warn;
        }
        Next::Wait
    }

    /// Whether the capture has run its time limit, and the wait for this question has
    /// been as long as C's whole read, so a reply C takes is not lost for it.
    pub fn time_is_up(&self, now: Instant) -> bool {
        self.limits.msec > 0
            && now.saturating_duration_since(self.started)
                >= Duration::from_millis(self.limits.msec)
    }

    /// What a capture says at a long silence that it goes on from.
    pub fn silence_message(&self) -> String {
        let secs = self.timing.no_answer.as_secs_f32();
        match self.heard {
            Heard::Nothing => format!("No answer from the meter for {secs} s; still asking."),
            Heard::Junk => format!("No usable answer from the meter for {secs} s; still asking."),
            Heard::Answers => format!(
                "The meter answers and gave no reading for {secs} s, as it may on an external \
                 trigger; still asking."
            ),
        }
    }

    /// What a time limit says of a capture with nothing to show.
    pub fn time_limit_message(&self) -> &'static str {
        match self.heard {
            Heard::Nothing => "No answer from the meter before the time limit.",
            Heard::Junk => "No usable answer from the meter before the time limit.",
            Heard::Answers => "No reading from the meter before the time limit.",
        }
    }

    /// Whether any sample came.
    pub fn sampled(&self) -> bool {
        self.sampled
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[test]
    fn the_table_as_c_has_it_but_for_two() {
        let f = |w| function(w).unwrap();
        assert_eq!(f("VDC").flags, MqFlags::DC);
        assert_eq!(f("VAC").flags, MqFlags::AC);
        assert_eq!(f("VACDC").flags, MqFlags::AC | MqFlags::DC);
        assert_eq!(f("AAC").unit, Unit::Ampere);
        assert_eq!(f("AAC").flags, MqFlags::AC);
        assert_eq!(f("ADC").flags, MqFlags::DC);
        assert_eq!(f("AACDC").flags, MqFlags::AC | MqFlags::DC);
        assert_eq!(f("CONT").mq, Mq::Continuity);
        assert_eq!(f("CONT").unit, Unit::Boolean);
        assert_eq!(f("DIODE").flags, MqFlags::DIODE);
        assert_eq!(f("FREQ").unit, Unit::Hertz);
        assert_eq!(f("OHMS").mq, Mq::Resistance);
        for no in ["", "vdc", "VDC ", "VOLTS", "VDC\r\n=>"] {
            assert_eq!(function(no), None, "{no:?}");
        }
    }

    #[test]
    fn a_reading_and_its_digits_as_c_counts_them() {
        for (text, value, digits) in [
            ("+1.2345E+0", 1.2345, 4),
            ("+6.7890E+3", 6789.0, 1),
            ("-2.5000E-3", -0.0025, 7),
            ("+3.0000E+2", 300.0, 2),
            // No point: the count runs from the text's start, the sign in it.
            ("+5E-1", 0.5, 2),
            ("5E-1", 0.5, 1),
            ("-1E+9", -1e9, -8),
            // Two characters of an exponent.
            ("+1.5E+10", 1.5e10, 0),
        ] {
            assert_eq!(reading(text), Some(Reading { value, digits }), "{text}");
        }
        assert_eq!(reading("+1E+9"), Some(Reading { value: f32::INFINITY, digits: 0 }));
    }

    #[test]
    fn what_is_no_reading() {
        for no in [
            "",
            "1",
            "0",
            "=>",
            "+1.2345",
            "1.5",
            "E+0",
            "+E+0",
            "+.5E+0",
            "+1.2E",
            "+1.2E+",
            "+1.2E+0x",
            "0x10E+0",
            "inf",
            "nan",
            "+inf",
            "+1.2.3E+0",
            "+1,2E+0",
            " +1.2E+0",
            "+1.2E+0 ",
            "+1.2e+0",
            "+1.2E+99",
            "VDC",
        ] {
            assert_eq!(reading(no), None, "{no:?}");
        }
    }

    /// What the form can't catch, a number for the report: the manual's frame and the
    /// overload, each single bit flipped and each byte lost.
    #[test]
    fn what_flips_and_losses_do_to_a_frame() {
        let mut flips = (0, 0, 0);
        let count = |frame: &str, flips: &mut (u32, u32, u32), bad: Vec<u8>| {
            let good = reading(frame).unwrap();
            let Ok(text) = String::from_utf8(bad) else {
                flips.0 += 1;
                return;
            };
            match reading(&text) {
                None => flips.0 += 1,
                Some(r) if r == good => flips.1 += 1,
                Some(_) => flips.2 += 1,
            }
        };
        for frame in ["+1.2345E+0", "+1E+9"] {
            let bytes = frame.as_bytes();
            for at in 0..bytes.len() {
                for bit in 0..8 {
                    let mut bad = bytes.to_vec();
                    bad[at] ^= 1 << bit;
                    count(frame, &mut flips, bad);
                }
            }
        }
        // Refused, the same, and a wrong value with nothing to tell: a digit that a
        // flip leaves a digit, and no more.
        assert_eq!(flips, (94, 0, 26));
        let mut losses = (0, 0, 0);
        for frame in ["+1.2345E+0", "+1E+9"] {
            let bytes = frame.as_bytes();
            for at in 0..bytes.len() {
                let mut bad = bytes.to_vec();
                bad.remove(at);
                count(frame, &mut losses, bad);
            }
        }
        assert_eq!(losses, (6, 2, 7));
    }

    #[test]
    fn modifiers_are_a_sum_and_nothing_else() {
        for (text, value) in [("0", 0), ("40", 40), ("127", 127), ("007", 7)] {
            assert_eq!(modifiers(text), Some(value));
        }
        for no in ["", "128", "-1", "+1", "1.0", "1 ", "abc", "=>", "4294967296"] {
            assert_eq!(modifiers(no), None, "{no:?}");
        }
    }

    #[test]
    fn what_each_modifier_does() {
        let base = function("VDC").unwrap();
        let on = |mods| {
            let mut label = base;
            let mut digits = 4;
            apply_modifiers(mods, &mut label, &mut digits);
            (label, digits)
        };
        assert_eq!(on(0), (base, 4));
        assert_eq!(on(64), (base, 4));
        assert_eq!(on(1).0.flags, MqFlags::DC | MqFlags::MIN);
        assert_eq!(on(2).0.flags, MqFlags::DC | MqFlags::MAX);
        assert_eq!(on(4).0.flags, MqFlags::DC | MqFlags::HOLD);
        assert_eq!(on(32).0.flags, MqFlags::DC | MqFlags::RELATIVE);
        let (db, digits) = on(8 | 1);
        assert_eq!(
            (db.mq, db.unit, db.flags, digits),
            (Mq::PowerFactor, Unit::DecibelMw, MqFlags::empty(), 2)
        );
        let (dbp, digits) = on(16);
        assert_eq!((dbp.mq, dbp.unit, digits), (Mq::Power, Unit::DecibelSpl, 2));
        assert_eq!(on(8 | 32).0.flags, MqFlags::RELATIVE);
    }

    fn at(start: Instant, ms: u64) -> Instant {
        start + Duration::from_millis(ms)
    }

    #[test]
    fn an_answer_is_done_at_its_prompt_and_a_prompt_ahead_is_ignored() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        r.sent(Kind::Flag, t);
        assert_eq!(r.line("=>", t), Step::Passed, "a tail, as C has it");
        assert_eq!(r.line("1", t), Step::Good);
        assert_eq!(r.line("=>", t), Step::Done(Reply::Answer("1".to_owned())));
        assert_eq!(r.owed(), 0);
    }

    #[test]
    fn an_error_prompt_closes_with_no_answer() {
        let t = Instant::now();
        for p in ["!>", "?>"] {
            let mut r = Requests::new(t);
            r.sent(Kind::Function, t);
            assert_eq!(r.line(p, t), Step::Done(Reply::Refused));
            assert_eq!(r.owed(), 0);
        }
    }

    #[test]
    fn a_late_answer_goes_with_its_own_question_and_the_next_gets_its_own() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        r.sent(Kind::Flag, t);
        assert_eq!(r.expired(at(t, 999)), None);
        assert_eq!(r.expired(at(t, 1000)), Some(Reply::Silent));
        r.sent(Kind::Modifiers, at(t, 1000));
        assert_eq!(r.owed(), 2);
        assert_eq!(
            r.line("1", at(t, 1100)),
            Step::Passed,
            "a late AUTO?, which would pass as MOD?"
        );
        assert_eq!(r.line("=>", at(t, 1101)), Step::Passed);
        assert_eq!(r.owed(), 1);
        assert_eq!(r.line("40", at(t, 1200)), Step::Good);
        assert_eq!(r.line("=>", at(t, 1201)), Step::Done(Reply::Answer("40".to_owned())));
    }

    #[test]
    fn one_behind_comes_back_in_step_by_the_form() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        r.sent(Kind::Reading, t);
        assert_eq!(r.line("VDC", t), Step::Junk);
        assert_eq!(r.line("=>", t), Step::Passed);
        assert_eq!(r.line("+1.5E+0", t), Step::Good);
        assert_eq!(r.line("=>", t), Step::Done(Reply::Answer("+1.5E+0".to_owned())));
    }

    #[test]
    fn a_lost_question_does_not_leave_every_later_answer_an_older_ones() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        r.sent(Kind::Flag, t);
        assert_eq!(r.give_up(), Reply::Silent);
        r.sent(Kind::Flag, at(t, 1000));
        r.line("1", at(t, 1001));
        r.line("=>", at(t, 1002));
        assert_eq!(r.owed(), 1);
        assert_eq!(r.expired(at(t, 3002)), Some(Reply::Silent));
        // By time: still owed just short of it, as a slow meter's answer may yet come.
        // One taken at a time past QUICK_LINE: nothing proved.
        r.sent(Kind::Flag, at(t, 19_000));
        assert_eq!(r.owed(), 2);
        assert_eq!(r.line("1", t + LOST_AFTER), Step::Passed);
        r.line("=>", t + LOST_AFTER);
        assert_eq!(r.give_up(), Reply::Silent);
        let later = t + LOST_AFTER;
        r.sent(Kind::Flag, later);
        assert_eq!(r.owed(), 1, "written off");
        assert_eq!(r.line("0", later), Step::Good);
        assert_eq!(r.line("=>", later), Step::Done(Reply::Answer("0".to_owned())));
    }

    #[test]
    fn two_quick_lines_in_a_row_say_a_question_was_lost() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        r.sent(Kind::Flag, t);
        assert_eq!(r.give_up(), Reply::Silent);
        // One: dropped. It may be the late one's.
        r.sent(Kind::Flag, at(t, 1000));
        assert_eq!(r.line("1", at(t, 1010)), Step::Passed);
        assert_eq!(r.line("=>", at(t, 1011)), Step::Passed);
        assert_eq!(r.give_up(), Reply::Silent);
        // Two.
        r.sent(Kind::Flag, at(t, 2000));
        assert_eq!(r.owed(), 2);
        assert_eq!(r.line("0", at(t, 2010)), Step::Good);
        assert_eq!(r.line("=>", at(t, 2011)), Step::Done(Reply::Answer("0".to_owned())));
        assert_eq!(r.owed(), 0);
    }

    #[test]
    fn a_dead_meter_owes_no_more_than_the_cap_within_the_time() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        for n in 0..6 {
            r.sent(Kind::Flag, at(t, n));
            r.give_up();
        }
        assert_eq!(r.owed(), MOST_OWED);
    }

    #[test]
    fn each_line_starts_the_wait_again() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        r.sent(Kind::Flag, t);
        assert_eq!(r.line("zzz", at(t, 900)), Step::Junk);
        assert_eq!(r.expired(at(t, 1000)), None);
        assert_eq!(r.expired(at(t, 1900)), Some(Reply::Unusable));
    }

    #[test]
    fn a_refusal_counts_as_a_quick_line_too() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        r.sent(Kind::Reading, t);
        assert_eq!(r.give_up(), Reply::Silent);
        r.sent(Kind::Function, at(t, 1000));
        assert_eq!(r.line("!>", at(t, 1010)), Step::Passed);
        assert_eq!(r.give_up(), Reply::Silent);
        r.sent(Kind::Function, at(t, 2000));
        assert_eq!(r.owed(), 2);
        assert_eq!(r.line("!>", at(t, 2010)), Step::Done(Reply::Refused));
        assert_eq!(r.owed(), 0);
    }

    #[test]
    fn junk_with_its_prompt_leaves_no_question_out_nor_a_longer_wait() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        r.sent(Kind::Reading, t);
        assert_eq!(r.line("nan", t), Step::Junk);
        assert_eq!(r.line("=>", t), Step::Passed);
        assert_eq!(r.expired(at(t, 1000)), Some(Reply::Unusable));
        assert_eq!(r.owed(), 0);
        assert_eq!(r.quiet_allowed(), FIRST_WAIT);
    }

    #[test]
    fn a_silent_meter_owes_little_and_the_wait_grows_to_its_end() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        let mut now = t;
        let mut waits = Vec::new();
        for _ in 0..8 {
            r.sent(Kind::Flag, now);
            waits.push(r.quiet_allowed().as_secs());
            now += r.quiet_allowed();
            assert_eq!(r.expired(now), Some(Reply::Silent));
        }
        assert_eq!(waits, [1, 2, 4, 8, 16, 30, 30, 30]);
        assert!(r.owed() <= MOST_OWED);
    }

    #[test]
    fn the_wait_comes_back_only_on_two_quick_answers() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        r.sent(Kind::Flag, t);
        r.give_up();
        assert_eq!(r.quiet_allowed(), Duration::from_secs(2));
        let mut now = at(t, 1000);
        for (n, expected) in [(1, 2), (2, 1)] {
            r.sent(Kind::Flag, now);
            if n == 1 {
                r.line("1", now);
                r.line("=>", now);
            }
            r.line("1", now);
            assert_eq!(r.line("=>", now), Step::Done(Reply::Answer("1".to_owned())));
            r.sent(Kind::Flag, now);
            assert_eq!(r.quiet_allowed().as_secs(), expected, "after {n}");
            r.line("1", now);
            r.line("=>", now);
            now += Duration::from_millis(1);
        }
    }

    #[test]
    fn an_answer_with_no_prompt_is_good_and_soon_not_waited_for() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        let mut now = t;
        let mut waits = Vec::new();
        for _ in 0..3 {
            r.sent(Kind::Flag, now);
            assert_eq!(r.line("1", now), Step::Good);
            waits.push(r.quiet_allowed());
            now += r.quiet_allowed();
            assert_eq!(r.expired(now), Some(Reply::Answer("1".to_owned())));
            assert_eq!(r.owed(), 0);
        }
        assert_eq!(waits, [FIRST_WAIT, FIRST_WAIT, SHORT_PROMPT_WAIT]);
        // Its late prompt shifts nothing.
        r.sent(Kind::Flag, now);
        assert_eq!(r.line("=>", now), Step::Passed);
        assert_eq!(r.quiet_allowed(), FIRST_WAIT);
        assert_eq!(r.line("0", now), Step::Good);
        assert_eq!(r.line("=>", now), Step::Done(Reply::Answer("0".to_owned())));
    }

    /// A meter of steady latency on its own clock, for a simulated minute. An answer
    /// says which question it answers. Taken from each: how many were right, how many
    /// answered another question, the most out, and the oldest answer taken.
    fn simulate(latency: Duration, queueing: bool) -> (usize, usize, u32, Duration) {
        let start = Instant::now();
        let mut now = start;
        let mut r = Requests::new(now);
        let mut line: VecDeque<(Instant, String)> = VecDeque::new();
        let mut asked: Vec<Instant> = Vec::new();
        let (mut right, mut wrong) = (0, 0);
        let mut most_owed = 0;
        let mut oldest = Duration::ZERO;
        let tick = Duration::from_millis(5);
        let mut out = false;
        while now < start + Duration::from_secs(60) {
            if !out {
                let seq = asked.len();
                asked.push(now);
                r.sent(Kind::Id, now);
                most_owed = most_owed.max(r.owed());
                // A meter that takes one at a time drops what comes while it works.
                if queueing || line.is_empty() {
                    let from = line.back().map_or(now, |(when, _)| (*when).max(now));
                    line.push_back((from + latency, format!("a,b,{seq}")));
                    line.push_back((from + latency + tick, "=>".to_owned()));
                }
                out = true;
                continue;
            }
            let mut done = None;
            while done.is_none() && line.front().is_some_and(|(when, _)| *when <= now) {
                let (_, text) = line.pop_front().unwrap();
                if let Step::Done(reply) = r.line(&text, now) {
                    done = Some(reply);
                }
            }
            match done.or_else(|| r.expired(now)) {
                Some(Reply::Answer(text)) => {
                    let seq: usize = text.rsplit(',').next().unwrap().parse().unwrap();
                    if seq == asked.len() - 1 {
                        right += 1;
                    } else {
                        wrong += 1;
                    }
                    oldest = oldest.max(now.saturating_duration_since(asked[seq]));
                    out = false;
                }
                Some(_) => out = false,
                None => now += tick,
            }
        }
        (right, wrong, most_owed, oldest)
    }

    /// For the report: `--nocapture` prints the table. What is pinned is that nothing
    /// piles up and that an answer taken is not one question behind.
    #[test]
    fn steady_latencies_settle_with_little_owed_on_either_kind_of_meter() {
        for ms in [100, 500, 900, 1100, 1500, 2500, 3500, 6000] {
            for queueing in [true, false] {
                let latency = Duration::from_millis(ms);
                let (right, wrong, most, oldest) = simulate(latency, queueing);
                println!(
                    "{ms} ms queueing {queueing}: {right} right, {wrong} wrong, {most} out, oldest {oldest:?}"
                );
                assert!(most <= MOST_OWED, "{ms} ms: {most} out");
                assert!(right > 0, "{ms} ms, queueing {queueing}: no answer in a minute");
                assert_eq!(wrong, 0, "{ms} ms, queueing {queueing}: an answer one behind");
                assert!(
                    oldest <= 3 * latency + Duration::from_millis(100),
                    "{ms} ms, queueing {queueing}: an answer {oldest:?} old"
                );
            }
        }
    }

    fn limits(samples_only: bool, msec: u64) -> Limits {
        Limits { samples_only, msec }
    }

    fn secs(n: u64) -> Duration {
        Duration::from_secs(n)
    }

    #[test]
    fn a_meter_that_never_spoke_fails_at_its_bound() {
        let t = Instant::now();
        let mut w = Watch::new(Timing::default(), limits(true, 0), t);
        assert_eq!(w.check(t + secs(29)), Next::Wait);
        assert_eq!(w.check(t + secs(30)), Next::GiveUp(GiveUp::NoAnswer(secs(30))));
        assert_eq!(GiveUp::NoAnswer(secs(30)).message(), "no answer from the meter for 30 s");
    }

    #[test]
    fn junk_fails_sooner_than_silence() {
        let t = Instant::now();
        let mut w = Watch::new(Timing::default(), limits(true, 0), t);
        w.heard(&Step::Junk);
        assert_eq!(w.check(t + secs(9)), Next::Wait);
        assert_eq!(w.check(t + secs(10)), Next::GiveUp(GiveUp::NoUsable(secs(10))));
        assert_eq!(
            GiveUp::NoUsable(secs(10)).message(),
            "no usable answer from the meter for 10 s"
        );
    }

    #[test]
    fn answers_with_no_reading_warn_and_go_on_ten_minutes() {
        let t = Instant::now();
        let mut w = Watch::new(Timing::default(), limits(true, 0), t);
        w.heard(&Step::Junk);
        w.heard(&Step::Good);
        assert_eq!(w.check(t + secs(29)), Next::Wait);
        assert_eq!(w.check(t + secs(30)), Next::Warn);
        assert_eq!(w.check(t + secs(599)), Next::Wait);
        assert_eq!(w.check(t + secs(600)), Next::GiveUp(GiveUp::NoReading(secs(600))));
        assert_eq!(GiveUp::NoReading(secs(600)).message(), "no reading from the meter for 600 s");
        assert!(w.silence_message().starts_with("The meter answers and gave no reading for 30 s"));
    }

    #[test]
    fn after_a_sample_a_warning_and_ten_minutes_from_it_each_time() {
        let t = Instant::now();
        let mut w = Watch::new(Timing::default(), limits(true, 0), t);
        assert!(!w.delivered(t + secs(100)));
        assert!(w.sampled());
        assert_eq!(w.check(t + secs(129)), Next::Wait);
        assert_eq!(w.check(t + secs(130)), Next::Warn);
        assert_eq!(w.silence_message(), "No answer from the meter for 30 s; still asking.");
        assert_eq!(w.check(t + secs(699)), Next::Wait);
        assert!(w.delivered(t + secs(699)), "it was told, so it is told it's back");
        assert_eq!(w.check(t + secs(1298)), Next::Warn);
        w.heard(&Step::Junk);
        assert_eq!(w.silence_message(), "No usable answer from the meter for 30 s; still asking.");
        assert_eq!(w.check(t + secs(1299)), Next::GiveUp(GiveUp::NoUsable(secs(600))));
    }

    #[test]
    fn a_time_limit_or_none_warns_and_never_gives_up() {
        let t = Instant::now();
        for l in [limits(false, 500), limits(false, 0)] {
            let mut w = Watch::new(Timing::default(), l, t);
            assert_eq!(w.check(t + secs(30)), Next::Warn);
            assert_eq!(w.check(t + secs(100_000)), Next::Wait);
        }
        let w = Watch::new(Timing::default(), limits(false, 500), t);
        assert!(!w.time_is_up(at(t, 499)));
        assert!(w.time_is_up(at(t, 500)));
        assert!(!Watch::new(Timing::default(), limits(true, 0), t).time_is_up(t + secs(100_000)));
    }

    #[test]
    fn what_a_time_limit_says() {
        let t = Instant::now();
        let mut w = Watch::new(Timing::default(), limits(false, 500), t);
        assert_eq!(w.time_limit_message(), "No answer from the meter before the time limit.");
        w.heard(&Step::Junk);
        assert_eq!(
            w.time_limit_message(),
            "No usable answer from the meter before the time limit."
        );
        w.heard(&Step::Good);
        assert_eq!(w.time_limit_message(), "No reading from the meter before the time limit.");
    }
}
