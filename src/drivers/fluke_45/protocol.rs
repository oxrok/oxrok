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
//! The users manual (PN 855981, chapter 5, pages as printed): an answer ends in CR LF
//! (5-11). After each command comes one of three prompts (5-4): `=>` done, `?>` a
//! Command Error, `!>` an Execution Error. A host must "wait for the meter to return a
//! prompt before sending another command", or the second string is discarded (5-11).
//! `^C` is a device clear, answered with `=>` (5-4). Echo is On at the factory (Table
//! 5-1), and then what the host sends comes back ahead of the answer.
//!
//! So a query here is an exchange: the question, its answer, and the prompt that
//! closes it, read line by line whatever pieces they came in, and the next question
//! goes only after that prompt: see [`Requests`]. No answer can then be taken for
//! another question's.
//!
//! A line has no checksum, and parity isn't looked at. The form of an answer is all
//! there is to check: a flipped bit that leaves a digit a digit gives a wrong reading.

use crate::core::{Mq, MqFlags, Unit};
use std::time::{Duration, Instant};

/// How long a prompt may take after its answer, as `sr_scpi_get_string`'s second in C.
pub const PROMPT_WAIT: Duration = Duration::from_secs(1);
/// How long a prompt is waited for, once two in a row did not come: a meter that sends
/// none is not waited for on each question. The manual has no way to turn prompts off,
/// so this is for a meter that isn't by the manual.
pub const SHORT_PROMPT_WAIT: Duration = Duration::from_millis(50);
/// How long a device clear's prompt is waited for; with none, the next question goes
/// anyway, as on a meter that sends no prompts.
pub const CLEAR_WAIT: Duration = Duration::from_secs(1);
/// How long a scan waits for `*IDN?`'s answer, C's second.
pub const ID_WAIT: Duration = Duration::from_secs(1);
/// A meter that goes unheard this long is told. On a sample limit alone and with
/// nothing heard since the capture began, it ends the capture. Slow is 2.5 readings a
/// second by the manual (5-24), and a range may take a while to settle.
pub const NO_ANSWER: Duration = Duration::from_secs(30);
/// Bytes that make nothing, with no sample yet: a quicker failure than silence.
pub const NO_USABLE: Duration = Duration::from_secs(10);
/// What else ends a capture on a sample limit alone: a meter that had a sample or that
/// answers and gives no reading, as on an external trigger, where a reading waits for
/// the trigger (5-26). Under a time limit or none, a question gets this long, then a
/// device clear, and the round starts again.
pub const GIVE_UP: Duration = Duration::from_secs(600);
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
    /// Whether `line` can be the answer to this.
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
/// alone (`protocol.c:68-71`). `CONT` has its unit wherever it is, where C sets the
/// first channel's (`protocol.c:78`); the manual has CONT on the primary display alone
/// (5-22), so no meter shows that.
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

/// `VAL1?`'s answer. The manual's examples are `+1.2345E+0` (5-11): this takes an
/// optional sign, digits with or without a point and its decimals, `E`, an optional
/// sign and exponent digits, so `5E-1` too. `+1E+9` and `-1E+9` are positive and
/// negative overload (5-11), infinite either way, where C has the first alone
/// (`protocol.c:265`) and reads the second as -1 GV. Refused is what isn't of that
/// form: `sr_atof_ascii` takes hex, `inf` and `nan`, and where it fails C sends what
/// the stack holds (`protocol.c:272`). FORMAT 2 adds a unit (`+1.2345E+0 VDC`,
/// 5-26), which is refused.
pub fn reading(line: &str) -> Option<Reading> {
    match line {
        "+1E+9" => return Some(Reading { value: f32::INFINITY, digits: 0 }),
        "-1E+9" => return Some(Reading { value: f32::NEG_INFINITY, digits: 0 }),
        _ => {}
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

/// `fl45_get_modifiers`, on the primary display's label and digits: the manual has the
/// modifiers on the primary display alone (5-22), where C puts them on both. REL says
/// relative, where C says hold (`protocol.c:164`), and COMP, 64, says nothing, as in
/// C. dB is a voltage in dBm, as oxrok's other meters have it, where C has a power
/// factor (`protocol.c:148`). dB Power is in watts (5-23), where C has dB
/// (`protocol.c:157`).
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
        label.mq = Mq::Voltage;
        label.unit = Unit::DecibelMw;
        label.flags = MqFlags::empty();
        *digits = 2;
    }
    if mods & 0x10 != 0 {
        label.mq = Mq::Power;
        label.unit = Unit::Watt;
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
    /// Lines that made nothing, and its prompt.
    Unusable,
    /// Given up on, and cleared: nothing came, or nothing that made sense.
    Silent,
}

/// What a line came to, while a question is out.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Step {
    /// The question is closed.
    Done(Reply),
    /// An answer of the kind asked, and its prompt is to come.
    Good,
    /// A prompt ahead of an answer, an echo, or a blank line: the wait goes on.
    Passed,
    /// Not of the kind asked.
    Junk,
}

/// One question out at a time, closed by its prompt, as a value over `Instant`.
///
/// Nothing else is ever out, so a line is this question's or an echo of it. A prompt
/// ahead of any answer is passed over, as C does, for a device clear's second form
/// (`<CR><LF>` and a prompt, 5-20) or what a stop left. A line of the wrong kind is
/// junk, and with its prompt the question is closed as unusable.
///
/// An answer is good without its prompt after [`PROMPT_WAIT`], and after
/// [`SHORT_PROMPT_WAIT`] once two in a row went unseen. A question with no answer is
/// waited for with no bound here: the capture's [`Watch`] decides when it is given up,
/// and then a device clear goes before anything else.
#[derive(Debug)]
pub struct Requests {
    out: bool,
    kind: Kind,
    command: String,
    answer: Option<String>,
    junk: bool,
    missed_prompts: u32,
    heard: Instant,
}

impl Requests {
    /// With nothing out.
    pub fn new(now: Instant) -> Requests {
        Requests {
            out: false,
            kind: Kind::Function,
            command: String::new(),
            answer: None,
            junk: false,
            missed_prompts: 0,
            heard: now,
        }
    }

    /// `command`, of this kind, has gone, its write having returned.
    pub fn sent(&mut self, command: &str, kind: Kind, now: Instant) {
        self.out = true;
        self.kind = kind;
        command.trim_end().clone_into(&mut self.command);
        self.answer = None;
        self.junk = false;
        self.heard = now;
    }

    /// The question out or last out, as it went.
    pub fn command(&self) -> &str {
        &self.command
    }

    /// Whether an answer is held, its prompt to come.
    pub fn has_answer(&self) -> bool {
        self.answer.is_some()
    }

    /// Whether a question is out.
    pub fn is_out(&self) -> bool {
        self.out
    }

    /// When a line last came, or the question went.
    pub fn heard_at(&self) -> Instant {
        self.heard
    }

    /// A whole line, its line end gone, came. Control bytes ahead of it (an echoed
    /// `^C`) and an echo of the question ahead of it are dropped.
    pub fn line(&mut self, line: &str, now: Instant) -> Step {
        self.heard = now;
        let line = line.trim_start_matches(|c: char| c.is_ascii_control());
        let line = if self.command.is_empty() {
            line
        } else {
            line.strip_prefix(self.command.as_str()).unwrap_or(line)
        };
        if !self.out || line.is_empty() {
            return Step::Passed;
        }
        match line {
            "=>" => match self.answer.take() {
                Some(answer) => self.close(Reply::Answer(answer)),
                None if self.junk => self.close(Reply::Unusable),
                None => Step::Passed,
            },
            "!>" | "?>" => {
                let reply = self.answer.take().map_or(Reply::Refused, Reply::Answer);
                self.close(reply)
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

    fn close(&mut self, reply: Reply) -> Step {
        self.out = false;
        self.missed_prompts = 0;
        Step::Done(reply)
    }

    /// A read that started at `looked` found nothing: an answer whose prompt didn't
    /// come in its time is good.
    pub fn prompt_late(&mut self, looked: Instant) -> Option<Reply> {
        self.answer.as_ref()?;
        let wait = if self.missed_prompts >= 2 { SHORT_PROMPT_WAIT } else { PROMPT_WAIT };
        if looked.saturating_duration_since(self.heard) < wait {
            return None;
        }
        self.out = false;
        self.missed_prompts += 1;
        self.answer.take().map(Reply::Answer)
    }

    /// The question is given up on, and a device clear is going.
    pub fn cleared(&mut self) {
        self.out = false;
        self.answer = None;
        self.junk = false;
        self.command.clear();
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
    /// Give up on the question out, clear the meter and start the round again.
    Clear,
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
/// meter answered `FUNC1?` first, and then C crashes.
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
            Step::Junk | Step::Done(Reply::Unusable) => Heard::Junk,
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
        } else if silence >= self.timing.give_up {
            // Measured again from here, so that a clear goes once a period.
            self.usable = now;
            self.heard = Heard::Nothing;
            return Next::Clear;
        }
        if silence >= self.timing.no_answer && !self.warned {
            self.warned = true;
            return Next::Warn;
        }
        Next::Wait
    }

    /// Whether the capture has run its time limit.
    pub fn time_is_up(&self, now: Instant) -> bool {
        self.limits.msec > 0
            && now.saturating_duration_since(self.started)
                >= Duration::from_millis(self.limits.msec)
    }

    /// What a capture says at a long silence that it goes on from.
    pub fn silence_message(&self) -> String {
        let secs = self.timing.no_answer.as_secs_f32();
        match self.heard {
            Heard::Nothing => format!("No answer from the meter for {secs} s; still waiting."),
            Heard::Junk => {
                format!("No usable answer from the meter for {secs} s; still waiting.")
            }
            Heard::Answers => format!(
                "The meter answers and gave no reading for {secs} s, as it may on an external \
                 trigger; still waiting."
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
            ("-1E+8", -1e8, -7),
            // Two characters of an exponent.
            ("+1.5E+10", 1.5e10, 0),
        ] {
            assert_eq!(reading(text), Some(Reading { value, digits }), "{text}");
        }
    }

    #[test]
    fn overload_either_way_is_infinite() {
        assert_eq!(reading("+1E+9"), Some(Reading { value: f32::INFINITY, digits: 0 }));
        assert_eq!(reading("-1E+9"), Some(Reading { value: f32::NEG_INFINITY, digits: 0 }));
        assert_eq!(reading("+1.0E+9").map(|r| r.value), Some(1e9));
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
            "+1.2345E+0 VDC",
        ] {
            assert_eq!(reading(no), None, "{no:?}");
        }
    }

    /// What the form can't catch, a number for the report: the manual's frame and the
    /// two overloads, each single bit flipped and each byte lost.
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
        let frames = ["+1.2345E+0", "+1E+9", "-1E+9"];
        for frame in frames {
            let bytes = frame.as_bytes();
            for at in 0..bytes.len() {
                for bit in 0..8 {
                    let mut bad = bytes.to_vec();
                    bad[at] ^= 1 << bit;
                    count(frame, &mut flips, bad);
                }
            }
        }
        // Refused, the same, and a wrong value with nothing to tell.
        assert_eq!(flips, FLIPS);
        let mut losses = (0, 0, 0);
        for frame in frames {
            let bytes = frame.as_bytes();
            for at in 0..bytes.len() {
                let mut bad = bytes.to_vec();
                bad.remove(at);
                count(frame, &mut losses, bad);
            }
        }
        assert_eq!(losses, LOSSES);
    }

    const FLIPS: (u32, u32, u32) = (128, 0, 32);
    const LOSSES: (u32, u32, u32) = (9, 2, 9);

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
            (Mq::Voltage, Unit::DecibelMw, MqFlags::empty(), 2)
        );
        let (dbp, digits) = on(16);
        assert_eq!((dbp.mq, dbp.unit, digits), (Mq::Power, Unit::Watt, 2));
        assert_eq!(on(8 | 32).0.flags, MqFlags::RELATIVE);
    }

    fn at(start: Instant, ms: u64) -> Instant {
        start + Duration::from_millis(ms)
    }

    fn asked(kind: Kind, command: &str, t: Instant) -> Requests {
        let mut r = Requests::new(t);
        r.sent(command, kind, t);
        r
    }

    #[test]
    fn an_answer_is_done_at_its_prompt_and_a_prompt_ahead_is_passed_over() {
        let t = Instant::now();
        let mut r = asked(Kind::Flag, "AUTO?", t);
        assert_eq!(r.line("=>", t), Step::Passed, "a tail, as C has it");
        assert_eq!(r.line("1", t), Step::Good);
        assert!(r.is_out());
        assert_eq!(r.line("=>", t), Step::Done(Reply::Answer("1".to_owned())));
        assert!(!r.is_out());
        // Nothing out: nothing to take.
        assert_eq!(r.line("0", t), Step::Passed);
        assert_eq!(r.line("=>", t), Step::Passed);
    }

    #[test]
    fn an_error_prompt_closes_with_no_answer() {
        let t = Instant::now();
        for p in ["!>", "?>"] {
            let mut r = asked(Kind::Function, "FUNC2?", t);
            assert_eq!(r.line(p, t), Step::Done(Reply::Refused));
            assert!(!r.is_out());
        }
    }

    /// The question waits for its prompt however long: nothing gives it up here.
    #[test]
    fn no_time_ends_a_question_with_no_answer() {
        let t = Instant::now();
        let mut r = asked(Kind::Reading, "VAL1?", t);
        assert_eq!(r.prompt_late(t + Duration::from_secs(100_000)), None);
        assert!(r.is_out());
        assert_eq!(r.line("+1.0E+0", t + Duration::from_secs(45)), Step::Good);
        assert_eq!(
            r.line("=>", t + Duration::from_secs(45)),
            Step::Done(Reply::Answer("+1.0E+0".to_owned()))
        );
    }

    #[test]
    fn junk_with_its_prompt_is_unusable_and_closes() {
        let t = Instant::now();
        let mut r = asked(Kind::Reading, "VAL1?", t);
        assert_eq!(r.line("nan", t), Step::Junk);
        assert_eq!(r.line("=>", t), Step::Done(Reply::Unusable));
        assert!(!r.is_out());
        // An answer and junk after it: the answer.
        let mut r = asked(Kind::Flag, "AUTO?", t);
        assert_eq!(r.line("1", t), Step::Good);
        assert_eq!(r.line("1", t), Step::Junk);
        assert_eq!(r.line("=>", t), Step::Done(Reply::Answer("1".to_owned())));
    }

    #[test]
    fn echoes_are_passed_over_whole_or_ahead_of_the_answer() {
        let t = Instant::now();
        let mut r = asked(Kind::Flag, "AUTO?\n", t);
        assert_eq!(r.line("AUTO?", t), Step::Passed);
        assert_eq!(r.line("1", t), Step::Good);
        assert_eq!(r.line("=>", t), Step::Done(Reply::Answer("1".to_owned())));
        let mut r = asked(Kind::Flag, "AUTO?", t);
        assert_eq!(r.line("AUTO?1", t), Step::Good);
        let mut r = asked(Kind::Flag, "AUTO?", t);
        assert_eq!(r.line("\x03AUTO?", t), Step::Passed);
        assert_eq!(r.line("", t), Step::Passed);
        assert_eq!(r.line("\x03=>", t), Step::Passed);
    }

    #[test]
    fn an_answer_with_no_prompt_is_good_and_soon_not_waited_for() {
        let t = Instant::now();
        let mut r = Requests::new(t);
        let mut now = t;
        let mut waits = Vec::new();
        for _ in 0..3 {
            r.sent("AUTO?", Kind::Flag, now);
            assert_eq!(r.line("1", now), Step::Good);
            let begun = now;
            while r.prompt_late(now).is_none() {
                now += Duration::from_millis(10);
            }
            assert!(!r.is_out());
            waits.push(now - begun);
        }
        assert_eq!(waits, [PROMPT_WAIT, PROMPT_WAIT, SHORT_PROMPT_WAIT]);
        // A prompt that comes again sets it back.
        r.sent("AUTO?", Kind::Flag, now);
        r.line("0", now);
        assert_eq!(r.line("=>", now), Step::Done(Reply::Answer("0".to_owned())));
        r.sent("AUTO?", Kind::Flag, now);
        r.line("0", now);
        assert_eq!(r.prompt_late(now + SHORT_PROMPT_WAIT), None);
    }

    #[test]
    fn a_cleared_question_takes_nothing_more() {
        let t = Instant::now();
        let mut r = asked(Kind::Reading, "VAL1?", t);
        r.cleared();
        assert!(!r.is_out());
        assert_eq!(r.line("+1.0E+0", t), Step::Passed);
        assert_eq!(r.line("=>", t), Step::Passed);
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
        assert_eq!(w.silence_message(), "No answer from the meter for 30 s; still waiting.");
        assert_eq!(w.check(t + secs(699)), Next::Wait);
        assert!(w.delivered(t + secs(699)), "it was told, so it is told it's back");
        assert_eq!(w.check(t + secs(1298)), Next::Warn);
        w.heard(&Step::Junk);
        assert_eq!(w.silence_message(), "No usable answer from the meter for 30 s; still waiting.");
        assert_eq!(w.check(t + secs(1299)), Next::GiveUp(GiveUp::NoUsable(secs(600))));
    }

    #[test]
    fn a_time_limit_or_none_warns_and_clears_once_a_period() {
        let t = Instant::now();
        for l in [limits(false, 500), limits(false, 0)] {
            let mut w = Watch::new(Timing::default(), l, t);
            assert_eq!(w.check(t + secs(30)), Next::Warn);
            assert_eq!(w.check(t + secs(599)), Next::Wait);
            assert_eq!(w.check(t + secs(600)), Next::Clear);
            assert_eq!(w.check(t + secs(601)), Next::Wait);
            assert_eq!(w.check(t + secs(1199)), Next::Wait);
            assert_eq!(w.check(t + secs(1200)), Next::Clear);
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
