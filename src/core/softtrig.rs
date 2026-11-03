// Counterpart in sigrok:
//   libsigrok/src/soft-trigger.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! A software trigger for streaming devices: matches a [`Trigger`]
//! against logic samples on the host, one stage after another.
//!
//! Level conditions (`0`, `1`) must all hold on one sample; edge
//! conditions (`r`, `f`, `e`) compare with the previous sample. A stage
//! fires on the first sample where all of its conditions hold, or on the
//! Nth with a count, or once they held for a duration. Then the next
//! stage is armed on the samples that follow. The trigger point is the
//! sample that completed the last stage. Value ranges and the or/and
//! combines are not implemented here; [`SoftTrigger::FEATURES`] says so
//! and [`Trigger::check`] rejects them.

use crate::core::error::Result;
use crate::core::trigger::{Trigger, TriggerDuration, TriggerFeatures, TriggerMatch};

/// One stage, flattened.
#[derive(Clone, Debug)]
struct Stage {
    conds: Vec<(usize, TriggerMatch)>,
    count: u32,
    consecutive: bool,
    /// Samples the condition must hold, 0 for none.
    duration: u64,
    /// Events seen so far.
    hits: u32,
    /// Samples in a row the condition held.
    run: u64,
    /// Whether the condition held on the previous sample.
    held: bool,
}

/// The matcher state.
#[derive(Clone, Debug)]
pub struct SoftTrigger {
    stages: Vec<Stage>,
    unit_size: usize,
    stage: usize,
    prev: Option<Vec<u8>>,
}

impl SoftTrigger {
    /// What this matcher supports.
    pub const FEATURES: TriggerFeatures = TriggerFeatures {
        count: true,
        consecutive: true,
        duration: true,
        range: false,
        or: false,
        and: false,
        max_stages: 0,
    };

    /// Checks `trigger` against [`SoftTrigger::FEATURES`], for a driver's
    /// `set_trigger`.
    pub fn check(trigger: &Trigger) -> Result<()> {
        trigger.check(&Self::FEATURES)
    }

    /// A matcher for `trigger` over samples of `unit_size` bytes.
    /// Conditions on channels beyond the sample width never match.
    /// Returns `None` for a trigger without stages.
    pub fn new(trigger: &Trigger, unit_size: usize) -> Option<Self> {
        if trigger.stages.is_empty() {
            return None;
        }
        let stages = trigger
            .stages
            .iter()
            .map(|s| Stage {
                conds: s.conditions.iter().map(|c| (c.channel, c.matches)).collect(),
                count: s.count.map_or(1, |c| c.count.max(1)),
                consecutive: s.count.is_some_and(|c| c.consecutive),
                duration: match s.duration {
                    Some(TriggerDuration::Longer(n)) => n,
                    None => 0,
                },
                hits: 0,
                run: 0,
                held: false,
            })
            .collect();
        Some(SoftTrigger { stages, unit_size, stage: 0, prev: None })
    }

    /// Bytes per sample.
    pub fn unit_size(&self) -> usize {
        self.unit_size
    }

    /// True once every stage has fired.
    pub fn fired(&self) -> bool {
        self.stage >= self.stages.len()
    }

    fn bit(sample: &[u8], channel: usize) -> Option<bool> {
        sample.get(channel / 8).map(|b| (b >> (channel % 8)) & 1 != 0)
    }

    fn matches_one(&self, sample: &[u8], ch: usize, m: TriggerMatch) -> bool {
        let Some(now) = Self::bit(sample, ch) else { return false };
        let before = self.prev.as_ref().and_then(|p| Self::bit(p, ch));
        match m {
            TriggerMatch::Zero => !now,
            TriggerMatch::One => now,
            TriggerMatch::Rising => before == Some(false) && now,
            TriggerMatch::Falling => before == Some(true) && !now,
            TriggerMatch::Edge => before.is_some_and(|b| b != now),
            TriggerMatch::Over | TriggerMatch::Under => false,
        }
    }

    fn matches(&self, sample: &[u8]) -> bool {
        self.stages[self.stage].conds.iter().all(|&(ch, m)| self.matches_one(sample, ch, m))
    }

    /// Whether the stage's edge conditions all match, and whether its
    /// level conditions all hold, on this sample.
    fn edges_and_levels(&self, sample: &[u8]) -> (bool, bool) {
        let conds = &self.stages[self.stage].conds;
        let edges = conds
            .iter()
            .filter(|(_, m)| m.is_edge())
            .all(|&(ch, m)| self.matches_one(sample, ch, m));
        let levels = conds
            .iter()
            .filter(|(_, m)| !m.is_edge())
            .all(|&(ch, m)| self.matches_one(sample, ch, m));
        (edges, levels)
    }

    /// Advances the current stage by one sample; true when it fired.
    fn step(&mut self, sample: &[u8]) -> bool {
        let matched = self.matches(sample);
        let idx = self.stage;
        let (duration, consecutive, count, held, run) = {
            let s = &self.stages[idx];
            (s.duration, s.consecutive, s.count, s.held, s.run)
        };
        let has_edge = self.stages[idx].conds.iter().any(|(_, m)| m.is_edge());
        if duration > 0 {
            let run = if matched { run + 1 } else { 0 };
            self.stages[idx].run = run;
            return run == duration + 1;
        }
        // A level-only stage counts the times it becomes true. A
        // consecutive count starts over when the counted edge comes
        // while the levels do not hold.
        let event = if has_edge { matched } else { matched && !held };
        let (edges, levels) = self.edges_and_levels(sample);
        let reset = consecutive && has_edge && edges && !levels;
        let st = &mut self.stages[idx];
        st.held = matched;
        if reset {
            st.hits = 0;
        }
        if event {
            st.hits += 1;
            if st.hits >= count {
                return true;
            }
        }
        false
    }

    /// Runs the samples in `data` through the matcher. Returns the
    /// index of the sample that completed the trigger, if it did here.
    /// After that, calls return `None`.
    pub fn feed(&mut self, data: &[u8]) -> Option<usize> {
        if self.fired() {
            return None;
        }
        for (i, sample) in data.chunks_exact(self.unit_size).enumerate() {
            if self.step(sample) {
                self.stage += 1;
                if self.fired() {
                    self.prev = None;
                    return Some(i);
                }
            }
            self.prev = Some(sample.to_vec());
        }
        None
    }
}

/// Keeps the last `limit` samples seen before the trigger.
#[derive(Clone, Debug)]
pub struct PreTriggerBuffer {
    unit_size: usize,
    limit: usize,
    data: Vec<u8>,
}

impl PreTriggerBuffer {
    /// A buffer for `limit` samples of `unit_size` bytes.
    pub fn new(unit_size: usize, limit: usize) -> Self {
        PreTriggerBuffer { unit_size, limit, data: Vec::new() }
    }

    /// Appends `data`, dropping the oldest samples beyond the limit.
    pub fn push(&mut self, data: &[u8]) {
        let max = self.limit * self.unit_size;
        if data.len() >= max {
            self.data.clear();
            self.data.extend_from_slice(&data[data.len() - max..]);
            return;
        }
        self.data.extend_from_slice(data);
        if self.data.len() > max {
            let excess = self.data.len() - max;
            self.data.drain(..excess);
        }
    }

    /// Takes the buffered samples.
    pub fn take(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.data)
    }
}

/// libsigrok's `soft_trigger_logic`, line by line, for drivers whose
/// output must match their C counterpart's sample for sample (fx2lafw):
/// one sample after another against the current stage's matches, a
/// stage that fails after an earlier one matched starts over at
/// stage 0 from the sample after the first match, edges against the
/// previous sample (zero before the first), and a ring of
/// `pre_trigger_samples` samples sent ahead of the trigger. Only level
/// and edge matches; [`SoftTrigger`] is the general matcher.
#[derive(Clone, Debug)]
pub struct SoftTriggerLogic {
    /// Per stage, the matches on enabled channels.
    stages: Vec<Vec<(usize, TriggerMatch)>>,
    unit_size: usize,
    /// The stage being matched.
    pub cur_stage: usize,
    /// Matches evaluated so far (`stl->count`): the very first edge
    /// match evaluated never fires.
    count: u64,
    prev: Vec<u8>,
    ring: Vec<u8>,
    head: usize,
    fill: usize,
}

impl SoftTriggerLogic {
    /// `soft_trigger_logic_new()`: `stages` holds each stage's matches
    /// on enabled channels (`soft_trigger_logic_check()` skips the
    /// others), over samples of `unit_size` bytes.
    pub fn new(
        stages: Vec<Vec<(usize, TriggerMatch)>>,
        unit_size: usize,
        pre_trigger_samples: usize,
    ) -> Self {
        SoftTriggerLogic {
            stages,
            unit_size,
            cur_stage: 0,
            count: 0,
            prev: vec![0; unit_size],
            ring: vec![0; unit_size * pre_trigger_samples],
            head: 0,
            fill: 0,
        }
    }

    fn append(&mut self, mut buf: &[u8]) {
        let size = self.ring.len();
        if buf.len() > size {
            buf = &buf[buf.len() - size..];
        }
        self.fill = (self.fill + buf.len()).min(size);
        while !buf.is_empty() {
            let n = (size - self.head).min(buf.len());
            self.ring[self.head..self.head + n].copy_from_slice(&buf[..n]);
            self.head += n;
            if self.head >= size {
                self.head = 0;
            }
            buf = &buf[n..];
        }
    }

    /// `pre_trigger_send()`: the ring's samples, oldest first, as the
    /// packets the C code sends (two when the ring wrapped).
    fn take_pre(&mut self) -> Vec<Vec<u8>> {
        let size = self.ring.len();
        let mut out = Vec::new();
        if self.fill < size {
            self.head = 0;
        }
        while self.fill > 0 {
            let n = (size - self.head).min(self.fill);
            out.push(self.ring[self.head..self.head + n].to_vec());
            self.head = 0;
            self.fill -= n;
        }
        out
    }

    fn check_match(&mut self, sample: &[u8], ch: usize, m: TriggerMatch) -> bool {
        self.count += 1;
        let bit = |s: &[u8]| s.get(ch / 8).is_some_and(|b| b & (1 << (ch % 8)) != 0);
        let now = bit(sample);
        match m {
            TriggerMatch::Zero => !now,
            TriggerMatch::One => now,
            _ if self.count == 1 => false,
            TriggerMatch::Rising => !bit(&self.prev) && now,
            TriggerMatch::Falling => bit(&self.prev) && !now,
            TriggerMatch::Edge => bit(&self.prev) != now,
            TriggerMatch::Over | TriggerMatch::Under => false,
        }
    }

    /// `soft_trigger_logic_check()` on `buf`: the offset in samples of
    /// the trigger point and the pre-trigger packets to send before it,
    /// or `None` (the samples go into the pre-trigger ring).
    pub fn check(&mut self, buf: &[u8]) -> Option<(usize, Vec<Vec<u8>>)> {
        let unit = self.unit_size as isize;
        let len = buf.len() as isize;
        let mut i: isize = 0;
        while i < len {
            let at = i as usize;
            let sample = buf.get(at..at + self.unit_size).unwrap_or(&buf[at..]).to_vec();
            let matches = self.stages.get(self.cur_stage).cloned().unwrap_or_default();
            let mut found = true;
            for (ch, m) in matches {
                if !self.check_match(&sample, ch, m) {
                    found = false;
                    break;
                }
            }
            let n = sample.len().min(self.unit_size);
            self.prev[..n].copy_from_slice(&sample[..n]);
            if found {
                if self.cur_stage + 1 < self.stages.len() {
                    self.cur_stage += 1;
                } else {
                    self.append(&buf[..at]);
                    let pre = self.take_pre();
                    return Some((at / self.unit_size, pre));
                }
            } else if self.cur_stage > 0 {
                i -= self.cur_stage as isize * unit;
                if i < -1 {
                    i = -1;
                }
                self.cur_stage = 0;
            }
            i += unit;
        }
        self.append(buf);
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::trigger::TriggerCount;

    fn trig(stages: &[&[(usize, TriggerMatch)]]) -> Trigger {
        let mut t = Trigger::new();
        for (i, s) in stages.iter().enumerate() {
            for &(ch, m) in *s {
                t.add(i, ch, m);
            }
        }
        t
    }

    #[test]
    fn levels_and_edges() {
        let t = trig(&[&[(0, TriggerMatch::Rising), (1, TriggerMatch::One)]]);
        let mut s = SoftTrigger::new(&t, 1).unwrap();
        // D0 rises at index 1 with D1 low, at index 4 with D1 high.
        assert_eq!(s.feed(&[0b00, 0b01, 0b00, 0b10, 0b11, 0b11]), Some(4));
        assert!(s.fired());
        assert_eq!(s.feed(&[0, 1]), None);
    }

    #[test]
    fn edges_across_chunks_and_stages() {
        let t = trig(&[&[(0, TriggerMatch::Rising)], &[(1, TriggerMatch::Falling)]]);
        let mut s = SoftTrigger::new(&t, 2).unwrap();
        assert_eq!(s.feed(&[0, 0]), None);
        // D0 rises on the first sample of this chunk, D1 falls two later.
        assert_eq!(s.feed(&[1, 0, 3, 0, 1, 0]), Some(2));
        let t = trig(&[&[(0, TriggerMatch::Edge)]]);
        let mut s = SoftTrigger::new(&t, 1).unwrap();
        // The first sample has no predecessor, so it cannot be an edge.
        assert_eq!(s.feed(&[1, 1, 0]), Some(2));
        let t = trig(&[&[(9, TriggerMatch::One)]]);
        let mut s = SoftTrigger::new(&t, 1).unwrap();
        assert_eq!(s.feed(&[0xff; 4]), None);
        assert!(SoftTrigger::new(&Trigger::new(), 1).is_none());
    }

    #[test]
    fn counts_and_consecutive_counts() {
        // Third rising edge of D0: pulses at 1, 3, 5.
        let mut t = trig(&[&[(0, TriggerMatch::Rising)]]);
        t.set_count(0, TriggerCount { count: 3, consecutive: false });
        let mut s = SoftTrigger::new(&t, 1).unwrap();
        assert_eq!(s.feed(&[0, 1, 0, 1, 0]), None);
        assert_eq!(s.feed(&[1, 0]), Some(0));
        // Third rising edge of D0 while D1 is high; an edge with D1 low
        // resets a consecutive count but not a cumulative one.
        let data = [0b10, 0b11, 0b10, 0b11, 0b00, 0b01, 0b10, 0b11, 0b10, 0b11, 0b10, 0b11];
        let mut t = trig(&[&[(0, TriggerMatch::Rising), (1, TriggerMatch::One)]]);
        t.set_count(0, TriggerCount { count: 3, consecutive: false });
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&data), Some(7));
        t.set_count(0, TriggerCount { count: 3, consecutive: true });
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&data), Some(11));
        // A level-only stage counts the times the level is entered.
        let mut t = trig(&[&[(0, TriggerMatch::One)]]);
        t.set_count(0, TriggerCount { count: 2, consecutive: false });
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&[1, 1, 0, 1, 1]), Some(3));
    }

    #[test]
    fn durations_fire_after_the_hold_time() {
        let mut t = trig(&[&[(0, TriggerMatch::One)]]);
        t.set_duration(0, TriggerDuration::Longer(3));
        let mut s = SoftTrigger::new(&t, 1).unwrap();
        // High for two samples is not enough; the long pulse starts at 4
        // and fires three samples later, across a chunk boundary.
        assert_eq!(s.feed(&[1, 1, 0, 0, 1, 1]), None);
        assert_eq!(s.feed(&[1, 1, 1]), Some(1));
        assert!(SoftTrigger::check(&t).is_ok());
        let mut r = Trigger::new();
        r.set_range(
            0,
            crate::core::trigger::TriggerRange {
                channels: vec![0, 1],
                low: 1,
                high: 2,
                inside: true,
            },
        );
        assert!(SoftTrigger::check(&r).is_err());
    }

    #[test]
    fn pre_trigger_buffer_keeps_the_tail() {
        let mut b = PreTriggerBuffer::new(2, 3);
        b.push(&[1, 1, 2, 2]);
        b.push(&[3, 3, 4, 4]);
        assert_eq!(b.take(), [2, 2, 3, 3, 4, 4]);
        b.push(&[0; 20]);
        assert_eq!(b.take().len(), 6);
    }

    #[test]
    fn the_c_soft_trigger() {
        // D5 rising in a counter: the trigger at 0x20,
        // six samples of pre-trigger sent first, in two pieces since the
        // ring wrapped.
        let data: Vec<u8> = (0..64).collect();
        let mut s = SoftTriggerLogic::new(vec![vec![(5, TriggerMatch::Rising)]], 1, 6);
        assert!(s.check(&data[..3]).is_none());
        let (at, pre) = s.check(&data[3..]).unwrap();
        assert_eq!(at, 29);
        assert_eq!(pre.concat(), (26..32).collect::<Vec<u8>>());
        assert_eq!(pre.len(), 2);
        // A two stage trigger that fails on the second stage starts over
        // one sample after the first stage matched.
        let mut s = SoftTriggerLogic::new(
            vec![vec![(0, TriggerMatch::One)], vec![(1, TriggerMatch::One)]],
            1,
            0,
        );
        assert_eq!(s.check(&[0, 1, 0, 1, 2]).map(|r| r.0), Some(4));
        // The very first edge match evaluated never fires; the previous
        // sample starts as zero.
        let mut s = SoftTriggerLogic::new(vec![vec![(0, TriggerMatch::Rising)]], 1, 0);
        assert_eq!(s.check(&[1, 0, 1]).map(|r| r.0), Some(2));
        let mut s = SoftTriggerLogic::new(
            vec![vec![(1, TriggerMatch::Zero), (0, TriggerMatch::Rising)]],
            1,
            0,
        );
        assert_eq!(s.check(&[1]).map(|r| r.0), Some(0));
    }
}
