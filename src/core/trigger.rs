// Counterpart in sigrok:
//   libsigrok/src/trigger.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Trigger definitions.
//!
//! The sigrok model (stages of level and edge conditions, fired in
//! order) plus what hardware triggers commonly add: an occurrence count
//! per stage, a duration, a value range over a group of channels, and
//! how two stages combine. A device says which of these it supports
//! with a [`TriggerFeatures`], and [`Trigger::check`] validates a
//! trigger against it. The command line grammar is in
//! [`crate::core::trigger_syntax`].

use crate::core::error::{Error, Result};

/// What a channel must do for a trigger stage to match.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TriggerMatch {
    /// The level is low.
    Zero,
    /// The level is high.
    One,
    /// A low to high transition.
    Rising,
    /// A high to low transition.
    Falling,
    /// Any transition.
    Edge,
    /// An analog value above the threshold.
    Over,
    /// An analog value below the threshold.
    Under,
}

impl TriggerMatch {
    /// The one letter code used on the command line (`-t D0=r`).
    pub fn code(self) -> char {
        match self {
            TriggerMatch::Zero => '0',
            TriggerMatch::One => '1',
            TriggerMatch::Rising => 'r',
            TriggerMatch::Falling => 'f',
            TriggerMatch::Edge => 'e',
            TriggerMatch::Over => 'o',
            TriggerMatch::Under => 'u',
        }
    }

    /// The match for a one letter code.
    pub fn from_code(c: char) -> Option<Self> {
        Some(match c {
            '0' => TriggerMatch::Zero,
            '1' => TriggerMatch::One,
            'r' => TriggerMatch::Rising,
            'f' => TriggerMatch::Falling,
            'e' => TriggerMatch::Edge,
            'o' => TriggerMatch::Over,
            'u' => TriggerMatch::Under,
            _ => return None,
        })
    }

    /// True for the transition matches.
    pub fn is_edge(self) -> bool {
        matches!(self, TriggerMatch::Rising | TriggerMatch::Falling | TriggerMatch::Edge)
    }
}

/// One condition of a trigger stage.
#[derive(Clone, Debug, PartialEq)]
pub struct TriggerCondition {
    /// The channel index, as the device numbers it.
    pub channel: usize,
    /// What the channel must do.
    pub matches: TriggerMatch,
    /// The threshold for analog matches, in the channel's unit.
    pub value: f32,
}

/// Fire on the Nth event of a stage instead of the first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TriggerCount {
    /// How many events, 1 or more.
    pub count: u32,
    /// Count only events in a row: an edge on one of the stage's edge
    /// channels while the stage's levels do not hold resets the count.
    pub consecutive: bool,
}

/// Fire when the stage's condition has held for a while, or on a pulse
/// of a given width.
///
/// On a stage of level conditions (and a value range) only `Longer` is
/// valid, and the trigger point is inside the hold: the sample that many
/// after the conditions became true.
///
/// On a stage with one rising or falling edge the duration is the width
/// of the pulse that the edge ends: the samples the channel spent at the
/// level it leaves (high for a falling edge, low for a rising one). The
/// trigger point is the edge, so `Longer` fires there at the end of the
/// pulse, not inside it. The stage's level conditions and value range
/// are checked on the edge sample. A pulse whose start was not seen does
/// not count. Devices say they take these with
/// [`TriggerFeatures::pulse`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriggerDuration {
    /// Held for more than this many samples. 0 is any pulse, and not
    /// valid on a stage of levels.
    Longer(u64),
    /// Held for fewer than this many samples. Pulses only.
    Shorter(u64),
    /// Held for at least the first and at most the second number of
    /// samples. Pulses only.
    Between(u64, u64),
}

impl TriggerDuration {
    /// The widths in samples that qualify, both ends inclusive.
    pub fn bounds(self) -> (u64, u64) {
        match self {
            TriggerDuration::Longer(n) => (n.saturating_add(1), u64::MAX),
            TriggerDuration::Shorter(n) => (1, n.saturating_sub(1)),
            TriggerDuration::Between(min, max) => (min, max),
        }
    }
}

/// A value read from a group of channels, within or outside a range.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TriggerRange {
    /// The channels, least significant first.
    pub channels: Vec<usize>,
    /// The lower bound, inclusive.
    pub low: u64,
    /// The upper bound, inclusive.
    pub high: u64,
    /// True to match inside the range, false outside.
    pub inside: bool,
}

/// How the stages of a two stage trigger combine.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TriggerCombine {
    /// Each stage after the previous one.
    #[default]
    Sequence,
    /// Either stage.
    Or,
    /// Both stages, in any order.
    And,
}

/// One stage of a trigger: every condition must hold at the same sample.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TriggerStage {
    /// The conditions.
    pub conditions: Vec<TriggerCondition>,
    /// Fire on the Nth event instead of the first.
    pub count: Option<TriggerCount>,
    /// Fire after the condition held for a while.
    pub duration: Option<TriggerDuration>,
    /// A value term on a group of channels.
    pub range: Option<TriggerRange>,
}

impl TriggerStage {
    /// True when some condition is an edge.
    pub fn has_edge(&self) -> bool {
        self.conditions.iter().any(|c| c.matches.is_edge())
    }

    /// True when some condition is a level.
    pub fn has_level(&self) -> bool {
        self.conditions.iter().any(|c| !c.matches.is_edge())
    }

    /// The rules of a duration or pulse width in stage number `n`.
    fn check_duration(&self, f: &TriggerFeatures, n: usize) -> Result<()> {
        let Some(d) = &self.duration else { return Ok(()) };
        if !f.duration {
            return Err(Error::arg("Trigger durations not supported by device."));
        }
        let longer = matches!(d, TriggerDuration::Longer(_));
        if (self.has_edge() || !longer) && !f.pulse {
            return Err(Error::arg("Pulse width triggers not supported by device."));
        }
        match *d {
            // A pulse is at least one sample, so longer than none is any
            // pulse: what a time below a sample period rounds down to.
            TriggerDuration::Longer(0) if !self.has_edge() => {
                return Err(Error::arg("Invalid trigger duration '0'."));
            }
            TriggerDuration::Shorter(len) if len < 2 => {
                return Err(Error::Arg(format!(
                    "No pulse is shorter than {len} samples in trigger stage {n}."
                )));
            }
            TriggerDuration::Between(min, max) if min == 0 || min > max => {
                return Err(Error::Arg(format!(
                    "No pulse width is within {min}..{max} samples in trigger stage {n}."
                )));
            }
            _ => {}
        }
        if self.has_edge() {
            let mut edges = self.conditions.iter().filter(|c| c.matches.is_edge());
            let one = edges.next().map(|c| c.matches);
            let rise_or_fall = matches!(one, Some(TriggerMatch::Rising | TriggerMatch::Falling));
            if !rise_or_fall || edges.next().is_some() {
                return Err(Error::Arg(format!(
                    "A pulse width needs one rising or falling edge in trigger stage {n}."
                )));
            }
        } else {
            if !longer {
                return Err(Error::Arg(format!(
                    "A pulse width needs the edge that ends the pulse \
                     (r or f) in trigger stage {n}."
                )));
            }
            if !self.has_level() && self.range.is_none() {
                return Err(Error::Arg(format!(
                    "A duration needs a condition in trigger stage {n}."
                )));
            }
        }
        if self.count.is_some() {
            return Err(Error::Arg(format!("Trigger stage {n} has both a count and a duration.")));
        }
        Ok(())
    }

    /// For a pulse width stage (a duration and an edge), the channel and
    /// the edge that ends the pulse. The first edge condition; `check`
    /// makes sure it is the only one, rising or falling.
    pub fn pulse(&self) -> Option<(usize, TriggerMatch)> {
        self.duration?;
        self.conditions.iter().find(|c| c.matches.is_edge()).map(|c| (c.channel, c.matches))
    }
}

/// A trigger: the stages fire in order, each after the previous one,
/// unless `combine` says otherwise.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Trigger {
    /// The stages, in order.
    pub stages: Vec<TriggerStage>,
    /// How two stages combine.
    pub combine: TriggerCombine,
    /// Samples the device takes before the trigger is armed, 0 for
    /// none. A trigger fires on the first match, which may be the first
    /// samples of the acquisition, with nothing before it to show; a
    /// front end that wants a time before the trigger sets that many
    /// samples here. Matching starts with the sample after them, and an
    /// edge from the last of them counts. Devices say they take this
    /// with [`TriggerFeatures::holdoff`].
    pub holdoff: u64,
}

/// What a device's trigger can do beyond level and edge stages.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct TriggerFeatures {
    /// Occurrence counts.
    pub count: bool,
    /// Consecutive counting.
    pub consecutive: bool,
    /// Durations: `Longer` on a stage of levels.
    pub duration: bool,
    /// Pulse widths: a duration on an edge stage, `Shorter` and
    /// `Between`.
    pub pulse: bool,
    /// Value ranges.
    pub range: bool,
    /// Two stages combined with or.
    pub or: bool,
    /// Two stages combined with and.
    pub and: bool,
    /// The most stages, 0 for any number.
    pub max_stages: usize,
    /// A hold-off: samples to take before the trigger is armed.
    pub holdoff: bool,
}

impl Trigger {
    /// A trigger with no stages.
    pub fn new() -> Self {
        Trigger::default()
    }

    fn stage_mut(&mut self, stage: usize) -> &mut TriggerStage {
        while self.stages.len() <= stage {
            self.stages.push(TriggerStage::default());
        }
        &mut self.stages[stage]
    }

    /// Adds a condition to stage `stage`, creating stages as needed.
    pub fn add(&mut self, stage: usize, channel: usize, matches: TriggerMatch) {
        self.stage_mut(stage).conditions.push(TriggerCondition { channel, matches, value: 0.0 });
    }

    /// Sets the occurrence count of a stage.
    pub fn set_count(&mut self, stage: usize, count: TriggerCount) {
        self.stage_mut(stage).count = Some(count);
    }

    /// Sets the duration of a stage.
    pub fn set_duration(&mut self, stage: usize, duration: TriggerDuration) {
        self.stage_mut(stage).duration = Some(duration);
    }

    /// Sets the value range of a stage.
    pub fn set_range(&mut self, stage: usize, range: TriggerRange) {
        self.stage_mut(stage).range = Some(range);
    }

    /// True when every stage has a condition or a range.
    pub fn is_valid(&self) -> bool {
        !self.stages.is_empty()
            && self.stages.iter().all(|s| !s.conditions.is_empty() || s.range.is_some())
    }

    /// Checks the trigger against what a device supports and against
    /// its own rules, with an error message for the user.
    pub fn check(&self, f: &TriggerFeatures) -> Result<()> {
        if !self.is_valid() {
            return Err(Error::arg("Invalid trigger."));
        }
        if self.holdoff > 0 && !f.holdoff {
            return Err(Error::arg("Trigger hold-off not supported by device."));
        }
        if f.max_stages > 0 && self.stages.len() > f.max_stages {
            return Err(Error::Arg(format!("Only {} trigger stages are supported.", f.max_stages)));
        }
        match self.combine {
            TriggerCombine::Sequence => {}
            TriggerCombine::Or if !f.or => {
                return Err(Error::arg(
                    "Trigger stages combined with or are not supported by device.",
                ));
            }
            TriggerCombine::And if !f.and => {
                return Err(Error::arg(
                    "Trigger stages combined with and are not supported by device.",
                ));
            }
            _ if self.stages.len() != 2 => {
                return Err(Error::arg("Or and and combine exactly two trigger stages."));
            }
            _ => {}
        }
        for (i, s) in self.stages.iter().enumerate() {
            let n = i + 1;
            if let Some(c) = &s.count {
                if !f.count {
                    return Err(Error::arg("Trigger counts not supported by device."));
                }
                if c.count == 0 {
                    return Err(Error::arg("Invalid trigger count '0'."));
                }
                if c.consecutive && !f.consecutive {
                    return Err(Error::arg("Consecutive trigger counts not supported by device."));
                }
                if c.consecutive && !s.has_edge() {
                    return Err(Error::Arg(format!(
                        "Consecutive counting needs an edge condition in trigger stage {n}."
                    )));
                }
            }
            s.check_duration(f, n)?;
            if let Some(r) = &s.range {
                if !f.range {
                    return Err(Error::arg("Trigger value ranges not supported by device."));
                }
                let w = r.channels.len();
                if w == 0 || w > 63 {
                    return Err(Error::Arg(format!("Invalid channel range in trigger stage {n}.")));
                }
                let mut sorted = r.channels.clone();
                sorted.sort_unstable();
                sorted.dedup();
                if sorted.len() != w {
                    return Err(Error::Arg(format!("Repeated channel in trigger stage {n}.")));
                }
                let max = (1u64 << w) - 1;
                if r.low > r.high {
                    return Err(Error::Arg(format!(
                        "Trigger value range {}..{} is reversed.",
                        r.low, r.high
                    )));
                }
                if r.high > max {
                    return Err(Error::Arg(format!(
                        "Trigger value {} does not fit {w} channels.",
                        r.high
                    )));
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn simple() -> Trigger {
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::Rising);
        t
    }

    #[test]
    fn checks_against_features() {
        let none = TriggerFeatures::default();
        let all = TriggerFeatures {
            count: true,
            consecutive: true,
            duration: true,
            pulse: true,
            range: true,
            or: true,
            and: true,
            max_stages: 2,
            holdoff: true,
        };
        assert!(simple().check(&none).is_ok());
        // A hold-off is the device's to keep.
        let held = Trigger { holdoff: 100, ..simple() };
        assert!(held.check(&all).is_ok());
        assert!(held.check(&none).is_err());
        assert!(Trigger::new().check(&all).is_err());
        let mut t = simple();
        t.set_count(0, TriggerCount { count: 5, consecutive: true });
        assert!(t.check(&none).unwrap_err().to_string().contains("counts not supported"));
        assert!(t.check(&all).is_ok());
        let mut t = Trigger::new();
        t.add(0, 1, TriggerMatch::One);
        t.set_count(0, TriggerCount { count: 2, consecutive: true });
        assert!(t.check(&all).unwrap_err().to_string().contains("needs an edge"));
        t.stages[0].count = None;
        t.set_duration(0, TriggerDuration::Longer(100));
        assert!(t.check(&all).is_ok());
        // An edge makes it a pulse width, with the level checked on the edge.
        t.add(0, 2, TriggerMatch::Falling);
        assert!(t.check(&all).is_ok());
        let mut t = Trigger::new();
        t.set_range(0, TriggerRange { channels: vec![0, 1, 2], low: 3, high: 9, inside: true });
        assert!(t.check(&all).unwrap_err().to_string().contains("does not fit 3 channels"));
        t.stages[0].range.as_mut().unwrap().high = 5;
        assert!(t.check(&all).is_ok());
        assert!(t.check(&none).unwrap_err().to_string().contains("value ranges"));
        let mut t = simple();
        t.add(1, 1, TriggerMatch::Rising);
        t.combine = TriggerCombine::Or;
        assert!(t.check(&all).is_ok());
        assert!(t.check(&none).unwrap_err().to_string().contains("with or"));
        t.add(2, 2, TriggerMatch::Rising);
        assert!(t.check(&all).unwrap_err().to_string().contains("Only 2"));
    }

    #[test]
    fn checks_pulse_widths() {
        use TriggerDuration::{Between, Longer, Shorter};
        use TriggerMatch::{Edge, Falling, One, Rising};
        let pulse = TriggerFeatures {
            duration: true,
            pulse: true,
            count: true,
            ..TriggerFeatures::default()
        };
        let levels = TriggerFeatures { pulse: false, ..pulse };
        let with = |m: TriggerMatch, d: TriggerDuration| {
            let mut t = Trigger::new();
            t.add(0, 0, m);
            t.set_duration(0, d);
            t
        };
        let err = |t: &Trigger, f: &TriggerFeatures| t.check(f).unwrap_err().to_string();

        // Any of the three on a rising or a falling edge.
        for d in [Longer(1), Shorter(2), Between(1, 1), Between(3, 9)] {
            assert!(with(Falling, d).check(&pulse).is_ok(), "{d:?}");
            assert!(with(Rising, d).check(&pulse).is_ok(), "{d:?}");
            assert!(err(&with(Falling, d), &levels).contains("Pulse width triggers not supported"));
            let none = TriggerFeatures::default();
            assert!(err(&with(Falling, d), &none).contains("durations not supported"));
        }
        // With levels and a range beside the edge.
        let mut t = with(Falling, Shorter(5));
        t.add(0, 1, One);
        assert!(t.check(&pulse).is_ok());

        // Widths that no pulse has.
        // Longer than no sample is any pulse; a level held for none is
        // refused, as before.
        assert!(with(Falling, Longer(0)).check(&pulse).is_ok());
        assert!(err(&with(One, Longer(0)), &pulse).contains("duration '0'"));
        assert!(err(&with(Falling, Shorter(0)), &pulse).contains("shorter than 0"));
        assert!(err(&with(Falling, Shorter(1)), &pulse).contains("shorter than 1"));
        assert!(err(&with(Falling, Between(0, 4)), &pulse).contains("within 0..4"));
        assert!(err(&with(Falling, Between(5, 4)), &pulse).contains("within 5..4"));

        // One edge, rising or falling.
        assert!(err(&with(Edge, Shorter(5)), &pulse).contains("one rising or falling edge"));
        let mut t = with(Falling, Shorter(5));
        t.add(0, 1, Rising);
        assert!(err(&t, &pulse).contains("one rising or falling edge"));

        // A level stage keeps `Longer` only, with or without pulses.
        assert!(with(One, Longer(5)).check(&pulse).is_ok());
        assert!(with(One, Longer(5)).check(&levels).is_ok());
        assert!(err(&with(One, Shorter(5)), &pulse).contains("needs the edge that ends"));
        assert!(err(&with(One, Between(2, 5)), &pulse).contains("needs the edge that ends"));
        assert!(
            err(&with(One, Shorter(5)), &levels).contains("Pulse width triggers not supported")
        );

        // No count with a width.
        let mut t = with(Falling, Shorter(5));
        t.set_count(0, TriggerCount { count: 2, consecutive: false });
        assert!(err(&t, &pulse).contains("both a count and a duration"));

        assert_eq!(Longer(0).bounds(), (1, u64::MAX));
        assert_eq!(Longer(3).bounds(), (4, u64::MAX));
        assert_eq!(Shorter(3).bounds(), (1, 2));
        assert_eq!(Between(2, 7).bounds(), (2, 7));
    }
}
