// Counterpart in sigrok:
//   libsigrok/src/hardware/siglent-sdl10x0/api.c,
//   libsigrok/src/hardware/siglent-sdl10x0/protocol.c,
//   libsigrok/src/hardware/siglent-sdl10x0/protocol.h,
//   Copyright (C) 2024 Timo Boettcher <timo@timoboettcher.name>,
//   with commits by Bert Vermeulen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! What the SDL1000X says, and how long it may take, with no line attached.
//!
//! All of it is text in SCPI, a line per message. A setting has no answer. A reading
//! is `MEAS:VOLT?` then `MEAS:CURR?`, a frame of the two when both are in, and one
//! question out at a time. The maker's programming guide (`siglentna.com/download/8968`)
//! shows a reading as `7.924678`, a level as `2.674`, the input as `1` or `0` and a mode
//! as `RESISTANCE`. It gives no time for an answer, and no time for anything else that
//! could bound a wait.

use crate::core::Key;
use std::time::Duration;

/// The makers a scan takes, whole: `manufacturers[]`.
pub const MANUFACTURERS: [&str; 1] = ["Siglent Technologies"];
/// The models a scan takes, whole: `models[]`.
pub const MODELS: [&str; 4] = ["SDL1020X-E", "SDL1020X", "SDL1030X-E", "SDL1030X"];

/// What a capture asks, in its turn.
pub const ASK_VOLTAGE: &str = "MEAS:VOLT?";
/// What a capture asks, in its turn.
pub const ASK_CURRENT: &str = "MEAS:CURR?";

/// The sixth character says the wattage: `probe_device`.
pub fn max_power(model: &str) -> f64 {
    let is_300 = model.get(..7).is_some_and(|start| start.eq_ignore_ascii_case("SDL1030"));
    if is_300 { 300.0 } else { 200.0 }
}

/// A number as the manual shows them: digits with a sign, a point and an exponent as
/// may be, and spaces ahead of it as `strtod` takes them. `sr_atof_ascii` also takes
/// nothing at all for 0, `nan`, `inf` and hexadecimal.
pub fn number(text: &str) -> Option<f64> {
    let text = text.trim_start_matches([' ', '\t']);
    let digits = text.strip_prefix(['+', '-']).unwrap_or(text);
    let first = digits.chars().next()?;
    if !(first.is_ascii_digit() || first == '.') {
        return None;
    }
    text.parse().ok().filter(|v: &f64| v.is_finite())
}

/// A number as `sr_scpi_get_float` keeps it, in a float, where it must still be one:
/// C's cast makes `inf` of `1e40`.
pub fn reading(text: &str) -> Option<f32> {
    number(text).map(|v| v as f32).filter(|v| v.is_finite())
}

/// `siglent_sdl10x0_string_to_mode`, which goes by how a name starts, and the word
/// `:FUNC` takes. C refuses the names its own list gives, so a get's answer can't be
/// set back, and lets `Unknown` through as `:FUNC Unknown`, the name of a mode that
/// has no name: here the list's are taken and that isn't.
pub fn regulation(name: &str) -> Option<&'static str> {
    const WORDS: [&str; 4] = ["CURRENT", "VOLTAGE", "POWER", "RESISTANCE"];
    const STARTS: [(&str, &str); 4] =
        [("CC", WORDS[0]), ("CV", WORDS[1]), ("CP", WORDS[2]), ("CR", WORDS[3])];
    WORDS
        .iter()
        .copied()
        .find(|word| *word == name)
        .or_else(|| STARTS.iter().find(|(start, _)| name.starts_with(start)).map(|(_, word)| *word))
}

/// How a key is read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Query {
    /// A number.
    Number(&'static str),
    /// `1` or `0`, or a word for them.
    Bool(&'static str),
    /// Text, as it comes.
    Text(&'static str),
    /// Nothing to ask: always this.
    Always(bool),
}

/// `config_get`'s table. The current's protection level is a number here: `api.c:265`
/// reads it as a boolean, so it fails on what the manual shows, and on a `1` returns a
/// float it never set.
pub fn query(key: &Key) -> Option<Query> {
    Some(match key {
        Key::Enabled => Query::Bool(":INPUT?"),
        Key::Regulation => Query::Text(":FUNC?"),
        Key::Voltage => Query::Number("MEAS:VOLTage?"),
        Key::VoltageTarget => Query::Number(":VOLTage:LEVel?"),
        Key::Current => Query::Number("MEAS:CURRent?"),
        Key::CurrentLimit => Query::Number(":CURRENT:LEVel?"),
        Key::Power => Query::Number("MEAS:POWer?"),
        Key::PowerTarget => Query::Number(":POWer:LEVel?"),
        Key::Resistance => Query::Number("MEAS:RESistance?"),
        Key::ResistanceTarget => Query::Number(":RESistance:LEVel?"),
        Key::OverPowerProtectionEnabled | Key::OverCurrentProtectionEnabled => Query::Always(true),
        Key::OverPowerProtectionActive => Query::Bool(":POWer:PROTection:STATe?"),
        Key::OverPowerProtectionThreshold => Query::Number(":POWer:PROTection:LEVel?"),
        Key::OverCurrentProtectionActive => Query::Bool(":CURRent:PROTection:STATe?"),
        Key::OverCurrentProtectionThreshold => Query::Number(":CURRent:PROTection:LEVel?"),
        _ => return None,
    })
}

/// A number's setting: its command, and the range `config_list` gives.
pub fn setter(key: &Key, max_power: f64) -> Option<(&'static str, f64, f64)> {
    Some(match key {
        Key::VoltageTarget => (":VOLT:LEV:IMM", 0.0, 150.0),
        Key::CurrentLimit => (":CURR:LEV:IMM", 0.0, 30.0),
        Key::PowerTarget => (":POW:LEV:IMM", 0.0, max_power),
        Key::ResistanceTarget => (":RES:LEV:IMM", 0.03, 10000.0),
        Key::OverPowerProtectionThreshold => (":POW:PROT:LEV", 0.0, max_power),
        Key::OverCurrentProtectionThreshold => (":CURR:PROT:LEV", 0.0, 30.0),
        _ => return None,
    })
}

/// What a wait may come to. A choice, as neither the manual nor C has a pace to go by,
/// but for C's own 100 ms, which its author's load kept to. Fields for a test.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bounds {
    /// How long a capture with no sample yet waits, and how long one with samples
    /// goes before it warns.
    pub first: Duration,
    /// How long a capture with samples goes on without an answer.
    pub long: Duration,
    /// How far past a time limit a capture with no sample yet looks for its first.
    pub grace: Duration,
    /// The most that one empty read counts for: what's beyond it was no read's wait
    /// but a stopped process's. Each read counts for at least what it waited, so being
    /// stopped again and again doesn't hold a bound off for ever.
    pub credit: Duration,
}

impl Default for Bounds {
    fn default() -> Bounds {
        Bounds {
            first: Duration::from_secs(10),
            long: Duration::from_secs(600),
            grace: Duration::from_secs(2),
            credit: Duration::from_secs(2),
        }
    }
}

/// How many answers in a row may make nothing.
pub const UNUSABLE: u32 = 3;

/// What a capture does next.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Step {
    /// Go on.
    Wait,
    /// Say this, and go on.
    Warn(String),
    /// Over with the limit met, and say this: there's less than the limit's time held.
    DoneWarning(String),
    /// A failed capture.
    Fail(String),
}

/// A capture's silence and junk, with its time passed in. Silence counts only what
/// reads waited, so a client that holds the queue up is no silence of the load's.
#[derive(Clone, Debug)]
pub struct Watch {
    bounds: Bounds,
    silence: Duration,
    warned: bool,
    unusable: u32,
    samples: u64,
}

impl Watch {
    /// At a capture's start.
    pub fn new(bounds: Bounds) -> Watch {
        Watch { bounds, silence: Duration::ZERO, warned: false, unusable: 0, samples: 0 }
    }

    /// The frames so far.
    pub fn samples(&self) -> u64 {
        self.samples
    }

    /// A read that waited `took` found nothing. `past` is how far beyond a time limit
    /// the capture was as that read began, and `time_limit` whether it has one.
    pub fn empty_read(&mut self, took: Duration, past: Option<Duration>, time_limit: bool) -> Step {
        self.silence += took.min(self.bounds.credit);
        let silence = self.silence.as_secs();
        if let Some(past) = past {
            if self.samples > 0 {
                return Step::DoneWarning(
                    "The time limit came with the load not answering.".to_owned(),
                );
            }
            if past >= self.bounds.grace {
                return Step::DoneWarning(
                    "No reading from the load within the time limit.".to_owned(),
                );
            }
        }
        if self.samples == 0 {
            if self.silence >= self.bounds.first {
                let text = format!("No answer from the load in {silence} s.");
                return if time_limit { Step::DoneWarning(text) } else { Step::Fail(text) };
            }
            return Step::Wait;
        }
        if self.silence >= self.bounds.long {
            return Step::Fail(format!("No answer from the load for {silence} s."));
        }
        if self.silence >= self.bounds.first && !self.warned {
            self.warned = true;
            return Step::Warn(format!(
                "No answer from the load for {silence} s. Waiting for it, up to {} s.",
                self.bounds.long.as_secs()
            ));
        }
        Step::Wait
    }

    /// An answer came. A line to say, if a warning went before it.
    pub fn heard(&mut self) -> Option<String> {
        let silence = std::mem::take(&mut self.silence).as_secs();
        std::mem::take(&mut self.warned)
            .then(|| format!("The load answers again, after {silence} s."))
    }

    /// An answer made nothing: `text`. It ends the capture at the [`UNUSABLE`]th since
    /// the last frame, not the last good number, or a load good for voltage and bad for
    /// current would go on for ever.
    pub fn unusable(&mut self, text: &str, time_limit: bool) -> Step {
        self.unusable += 1;
        if self.unusable < UNUSABLE {
            return Step::Warn(format!("Unusable answer from the load: '{text}'."));
        }
        let text = format!("No usable answer from the load: '{text}'.");
        if time_limit && self.samples == 0 { Step::DoneWarning(text) } else { Step::Fail(text) }
    }

    /// A frame is out.
    pub fn sample(&mut self) {
        self.samples += 1;
        self.unusable = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: Duration = Duration::from_secs(1);

    #[test]
    fn numbers_as_the_manual_shows_them() {
        for (text, value) in [
            ("7.924678", 7.924_678),
            ("-0.5", -0.5),
            ("+3", 3.0),
            (".5", 0.5),
            ("1.5e1", 15.0),
            ("1E-3", 0.001),
            (" 2.674", 2.674),
            ("0", 0.0),
        ] {
            assert_eq!(number(text), Some(value), "{text}");
        }
    }

    #[test]
    fn what_c_makes_a_reading_of_and_this_does_not() {
        for text in [
            "", " ", "abc", "nan", "NaN", "-nan", "inf", "-inf", "infinity", "0x10", "0X1p3",
            "1 V", "1V", "1,5", "1.5 ", "1e", "1e999", "-", ".", "+-1", "1\u{0}",
        ] {
            assert_eq!(number(text), None, "{text:?}");
        }
        assert_eq!(number("1e40"), Some(1e40));
        assert_eq!(reading("1e40"), None, "no float holds it");
        assert_eq!(reading("3.5"), Some(3.5));
    }

    /// No checksum, so what a flipped bit or a lost byte can do to a line. This
    /// counts it, on the manual's own answer and on the other classes of value: the
    /// format can't catch a changed digit, and a lost digit gives another number.
    #[test]
    fn what_damage_gives_a_wrong_value() {
        // A line, then flips that give a wrong value, flips refused, lost bytes that
        // give a wrong value, lost bytes refused.
        let table: [(&str, u32, u32, u32, u32); 5] = [
            ("7.924678", 19, 45, 8, 0),
            ("0.000012", 27, 36, 7, 0),
            ("149.999999", 20, 59, 10, 0),
            ("-0.250000", 26, 46, 4, 0),
            ("1.5e1", 11, 28, 4, 1),
        ];
        for (line, want_wrong, want_refused, want_lost_wrong, want_lost_refused) in table {
            let truth = reading(line);
            let judge = |bytes: &[u8]| -> (u32, u32) {
                match std::str::from_utf8(bytes).ok().and_then(reading) {
                    Some(v) => (u32::from(Some(v) != truth), 0),
                    None => (0, 1),
                }
            };
            let (mut wrong, mut refused) = (0, 0);
            for at in 0..line.len() * 8 {
                let mut flipped = line.as_bytes().to_vec();
                flipped[at / 8] ^= 1 << (at % 8);
                let (w, r) = judge(&flipped);
                wrong += w;
                refused += r;
            }
            assert_eq!((wrong, refused), (want_wrong, want_refused), "flips of {line}");
            let (mut wrong, mut refused) = (0, 0);
            for at in 0..line.len() {
                let mut lost = line.as_bytes().to_vec();
                lost.remove(at);
                let (w, r) = judge(&lost);
                wrong += w;
                refused += r;
            }
            assert_eq!((wrong, refused), (want_lost_wrong, want_lost_refused), "lost of {line}");
        }
    }

    #[test]
    fn the_wattage_goes_by_the_name() {
        assert_eq!(max_power("SDL1020X-E"), 200.0);
        assert_eq!(max_power("SDL1020X"), 200.0);
        assert_eq!(max_power("SDL1030X-E"), 300.0);
        assert_eq!(max_power("sdl1030x"), 300.0);
        assert_eq!(max_power("SDL"), 200.0);
        assert_eq!(max_power("SDL1é30X"), 200.0, "a cut inside a character is no panic");
    }

    #[test]
    fn regulation_by_its_start_or_the_list_s_words() {
        assert_eq!(regulation("CC"), Some("CURRENT"));
        assert_eq!(regulation("CV"), Some("VOLTAGE"));
        assert_eq!(regulation("CP"), Some("POWER"));
        assert_eq!(regulation("CR"), Some("RESISTANCE"));
        assert_eq!(regulation("CCX"), Some("CURRENT"), "strncmp's way");
        for word in ["CURRENT", "VOLTAGE", "POWER", "RESISTANCE"] {
            assert_eq!(regulation(word), Some(word));
        }
        for name in ["Unknown", "LED", "cc", "C", "", "XYZ", "current"] {
            assert_eq!(regulation(name), None, "{name}");
        }
    }

    #[test]
    fn the_tables() {
        assert_eq!(setter(&Key::PowerTarget, 300.0), Some((":POW:LEV:IMM", 0.0, 300.0)));
        assert_eq!(setter(&Key::OverPowerProtectionThreshold, 200.0).unwrap().2, 200.0);
        assert_eq!(setter(&Key::Voltage, 200.0), None);
        assert_eq!(
            query(&Key::OverCurrentProtectionThreshold),
            Some(Query::Number(":CURRent:PROTection:LEVel?"))
        );
        assert_eq!(query(&Key::OverPowerProtectionEnabled), Some(Query::Always(true)));
        assert_eq!(query(&Key::LimitSamples), None);
    }

    #[test]
    fn the_bounds_are_what_they_say() {
        let b = Bounds::default();
        assert_eq!(b.first, Duration::from_secs(10));
        assert_eq!(b.long, Duration::from_secs(600));
        assert_eq!(b.grace, Duration::from_secs(2));
        assert_eq!(b.credit, Duration::from_secs(2));
        assert_eq!(UNUSABLE, 3);
    }

    #[test]
    fn no_sample_fails_at_the_bound_and_not_before() {
        let mut w = Watch::new(Bounds::default());
        for _ in 0..9 {
            assert_eq!(w.empty_read(S, None, false), Step::Wait);
        }
        assert_eq!(
            w.empty_read(S, None, false),
            Step::Fail("No answer from the load in 10 s.".to_owned())
        );
    }

    /// A failing capture with a time limit and nothing captured stays exit 0.
    #[test]
    fn no_sample_under_a_time_limit_only_warns() {
        let mut w = Watch::new(Bounds::default());
        for _ in 0..9 {
            assert_eq!(w.empty_read(S, None, true), Step::Wait);
        }
        assert_eq!(
            w.empty_read(S, None, true),
            Step::DoneWarning("No answer from the load in 10 s.".to_owned())
        );
        let mut w = Watch::new(Bounds::default());
        assert_eq!(w.empty_read(S, Some(S), true), Step::Wait, "inside the grace");
        assert_eq!(
            w.empty_read(S, Some(2 * S), true),
            Step::DoneWarning("No reading from the load within the time limit.".to_owned())
        );
    }

    #[test]
    fn with_samples_it_warns_once_and_goes_on_for_ten_minutes() {
        let mut w = Watch::new(Bounds::default());
        w.sample();
        let mut warnings = Vec::new();
        for second in 1..600 {
            match w.empty_read(S, None, false) {
                Step::Wait => {}
                Step::Warn(text) => warnings.push((second, text)),
                other => panic!("{other:?} at {second}"),
            }
        }
        assert_eq!(
            warnings,
            [(10, "No answer from the load for 10 s. Waiting for it, up to 600 s.".to_owned())]
        );
        assert_eq!(
            w.empty_read(S, None, false),
            Step::Fail("No answer from the load for 600 s.".to_owned())
        );
    }

    #[test]
    fn an_answer_says_it_is_back_and_starts_the_count_over() {
        let mut w = Watch::new(Bounds::default());
        assert_eq!(w.heard(), None);
        w.sample();
        for _ in 0..12 {
            w.empty_read(S, None, false);
        }
        assert_eq!(w.heard().as_deref(), Some("The load answers again, after 12 s."));
        assert_eq!(w.heard(), None, "said once");
        for _ in 0..9 {
            assert_eq!(w.empty_read(S, None, false), Step::Wait);
        }
        assert!(matches!(w.empty_read(S, None, false), Step::Warn(_)), "it warns anew");
    }

    /// Many bounds on a moved clock: answers on time never add up.
    #[test]
    fn silence_does_not_add_across_answers() {
        let mut w = Watch::new(Bounds::default());
        for _ in 0..10_000 {
            assert_eq!(w.empty_read(S, None, false), Step::Wait);
            assert_eq!(w.heard(), None);
        }
    }

    /// A stopped process: the read took a minute of the clock and a stop's worth of
    /// that is credited, and being stopped over and over still gets there.
    #[test]
    fn a_stall_counts_for_little_but_counts() {
        let mut w = Watch::new(Bounds::default());
        for _ in 0..4 {
            assert_eq!(w.empty_read(60 * S, None, false), Step::Wait);
        }
        assert!(matches!(w.empty_read(60 * S, None, false), Step::Fail(_)));
    }

    #[test]
    fn a_time_limit_with_samples_ends_at_an_empty_read_and_says_so() {
        let mut w = Watch::new(Bounds::default());
        w.sample();
        assert_eq!(
            w.empty_read(S, Some(Duration::ZERO), true),
            Step::DoneWarning("The time limit came with the load not answering.".to_owned())
        );
    }

    #[test]
    fn junk_fails_quickly_and_only_a_frame_forgives_it() {
        let mut w = Watch::new(Bounds::default());
        assert_eq!(
            w.unusable("abc", false),
            Step::Warn("Unusable answer from the load: 'abc'.".to_owned())
        );
        assert!(w.heard().is_none(), "a good voltage in between");
        assert!(matches!(w.unusable("abc", false), Step::Warn(_)));
        assert_eq!(
            w.unusable("abc", false),
            Step::Fail("No usable answer from the load: 'abc'.".to_owned())
        );
        let mut w = Watch::new(Bounds::default());
        w.unusable("a", true);
        w.unusable("a", true);
        assert_eq!(
            w.unusable("a", true),
            Step::DoneWarning("No usable answer from the load: 'a'.".to_owned())
        );
        let mut w = Watch::new(Bounds::default());
        w.unusable("a", false);
        w.unusable("a", false);
        w.sample();
        assert!(matches!(w.unusable("a", true), Step::Warn(_)));
        w.unusable("a", true);
        assert!(matches!(w.unusable("a", true), Step::Fail(_)), "with samples it failed");
    }
}
