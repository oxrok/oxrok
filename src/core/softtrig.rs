// Counterpart in sigrok:
//   libsigrok/src/soft-trigger.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! A software trigger for streaming devices: matches a [`Trigger`]
//! against logic samples on the host.
//!
//! Level conditions (`0`, `1`) and a stage's value range must all hold
//! on one sample; edge conditions (`r`, `f`, `e`) compare with the
//! previous sample. A stage fires on the first sample where all of its
//! conditions hold, or on the Nth with a count, or once they held for a
//! duration. A duration on a stage with an edge is a pulse width: the
//! stage fires on the edge that ends a pulse of that many samples, its
//! levels and range checked there, and only for a pulse whose start the
//! stage saw: one already running at the first sample, after a
//! [`SoftTrigger::rearm`] or when a sequence arms the stage does not
//! count. In a sequence the next stage is armed on the samples that
//! follow; with or/and both stages are matched side by side, and the
//! trigger fires when either fired, or once both did. The trigger point
//! is the sample that completed the trigger.

use crate::core::error::Result;
use crate::core::trigger::{
    Trigger, TriggerCombine, TriggerDuration, TriggerFeatures, TriggerMatch, TriggerRange,
};

/// A sample of at most 8 bytes as a word, channel `n` in bit `n`.
fn word(sample: &[u8]) -> u64 {
    let mut w = [0u8; 8];
    w[..sample.len()].copy_from_slice(sample);
    u64::from_le_bytes(w)
}

/// Samples per word when several fit: 8, 4 or 2 for 1, 2 or 4 byte
/// samples, else 1.
fn lanes(unit_size: usize) -> usize {
    match unit_size {
        1 | 2 | 4 => 8 / unit_size,
        _ => 1,
    }
}

/// `x` in every `unit_size` byte lane of a word.
fn broadcast(x: u64, unit_size: usize) -> u64 {
    (0..lanes(unit_size)).fold(0, |w, k| w | x << (k * unit_size * 8))
}

/// A stage's conditions over a block of samples, one per lane of a
/// word: a filter that says when no sample of the block can match.
#[derive(Clone, Copy, Debug, Default)]
struct Block {
    zero: u64,
    one: u64,
    rise: u64,
    fall: u64,
    change: u64,
    /// The stage can't match at all.
    never: bool,
    /// The lowest and the highest bit of every lane.
    low: u64,
    high: u64,
}

impl Block {
    fn new(m: [u64; 5], never: bool, unit: usize) -> Self {
        let b = |x| broadcast(x, unit);
        Block {
            zero: b(m[0]),
            one: b(m[1]),
            rise: b(m[2]),
            fall: b(m[3]),
            change: b(m[4]),
            never,
            low: b(1),
            high: b(1 << (unit * 8 - 1)),
        }
    }

    /// Whether some lane of `now` may match, its previous samples in
    /// the lanes of `prev`. A lane matches when it fails no condition,
    /// i.e. its lane of `fail` is zero; the zero-lane test can flag a
    /// lane above a zero one, never misses one.
    #[inline]
    fn may_match(&self, now: u64, prev: u64) -> bool {
        let fail = (now & self.zero)
            | (!now & self.one)
            | (self.rise & (prev | !now))
            | (self.fall & (!prev | now))
            | (self.change & !(prev ^ now));
        !self.never && fail.wrapping_sub(self.low) & !fail & self.high != 0
    }
}

/// Which bits of a word a value range reads.
#[derive(Clone, Debug)]
enum Pick {
    /// Ascending neighbours: a shift and a mask.
    Run { shift: u32, mask: u64 },
    /// Any order, least significant first.
    Bits(Vec<u32>),
}

/// A stage's conditions and range as masks over a sample word, for
/// samples of at most 8 bytes.
#[derive(Clone, Debug, Default)]
struct Masks {
    zero: u64,
    one: u64,
    rise: u64,
    fall: u64,
    change: u64,
    /// A pulse width stage: the edge that starts the pulse, the opposite
    /// of the one in `rise` or `fall` that ends it.
    start_rise: u64,
    start_fall: u64,
    /// An edge condition on a channel beyond the sample width.
    edge_never: bool,
    /// A level condition or range channel beyond it, or Over/Under.
    level_never: bool,
    range: Option<(Pick, u64, u64, bool)>,
}

impl Masks {
    /// The block filter: the conditions, and of a range matched inside
    /// the high bits on which its bounds agree (all of them for one
    /// value). A consecutive count with an edge also starts over on
    /// the edge alone, so its filter has only the edges. A pulse width
    /// stage counts the samples between two edges of its channel, so its
    /// filter passes any block where the channel changes, whatever the
    /// levels.
    fn block(&self, unit: usize, edges_only: bool) -> Block {
        if self.start_rise | self.start_fall != 0 {
            let change = self.start_rise | self.start_fall;
            return Block::new([0, 0, 0, 0, change], self.edge_never, unit);
        }
        let (mut zero, mut one) = if edges_only { (0, 0) } else { (self.zero, self.one) };
        if let (false, Some((pick, low, high, true))) = (edges_only, &self.range) {
            let bits: Vec<u32> = match pick {
                Pick::Run { shift, mask } => (0..mask.count_ones()).map(|k| shift + k).collect(),
                Pick::Bits(bits) => bits.clone(),
            };
            let max = if bits.len() < 64 { (1u64 << bits.len()) - 1 } else { u64::MAX };
            let high = (*high).min(max);
            if *low > high {
                return Block::new([0; 5], true, unit);
            }
            // Above the highest bit where the bounds differ, a value in
            // the range has the bounds' bits.
            let differ = 64 - (low ^ high).leading_zeros();
            for (k, &b) in bits.iter().enumerate().skip(differ as usize) {
                if low >> k & 1 == 1 {
                    one |= 1 << b;
                } else {
                    zero |= 1 << b;
                }
            }
        }
        let never = self.edge_never || self.level_never;
        Block::new([zero, one, self.rise, self.fall, self.change], never, unit)
    }

    fn new(
        conds: &[(usize, TriggerMatch)],
        range: Option<&TriggerRange>,
        unit_size: usize,
    ) -> Self {
        let width = unit_size * 8;
        let mut m = Masks::default();
        for &(ch, t) in conds {
            if ch >= width {
                if t.is_edge() {
                    m.edge_never = true;
                } else {
                    m.level_never = true;
                }
                continue;
            }
            let bit = 1u64 << ch;
            match t {
                TriggerMatch::Zero => m.zero |= bit,
                TriggerMatch::One => m.one |= bit,
                TriggerMatch::Rising => m.rise |= bit,
                TriggerMatch::Falling => m.fall |= bit,
                TriggerMatch::Edge => m.change |= bit,
                TriggerMatch::Over | TriggerMatch::Under => m.level_never = true,
            }
        }
        if let Some(r) = range {
            if r.channels.iter().any(|&ch| ch >= width) {
                m.level_never = true;
            } else {
                let first = r.channels.first().copied().unwrap_or(0);
                let run = r.channels.iter().enumerate().all(|(k, &ch)| ch == first + k);
                let pick = if run {
                    let n = r.channels.len();
                    let mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
                    Pick::Run { shift: first as u32, mask }
                } else {
                    Pick::Bits(r.channels.iter().map(|&ch| ch as u32).collect())
                };
                m.range = Some((pick, r.low, r.high, r.inside));
            }
        }
        m
    }

    /// As [`SoftTrigger::edges_and_levels`], on words.
    #[inline]
    fn edges_and_levels(&self, now: u64, prev: Option<u64>) -> (bool, bool) {
        let edges = !self.edge_never
            && match prev {
                _ if self.rise | self.fall | self.change == 0 => true,
                None => false,
                Some(p) => {
                    !p & now & self.rise == self.rise
                        && p & !now & self.fall == self.fall
                        && (p ^ now) & self.change == self.change
                }
            };
        let levels = !self.level_never
            && now & self.one == self.one
            && now & self.zero == 0
            && self.range.as_ref().is_none_or(|(pick, low, high, inside)| {
                let value = match pick {
                    Pick::Run { shift, mask } => (now >> shift) & mask,
                    Pick::Bits(bits) => {
                        bits.iter().enumerate().fold(0, |v, (k, &b)| v | ((now >> b) & 1) << k)
                    }
                };
                (*low..=*high).contains(&value) == *inside
            });
        (edges, levels)
    }

    /// As [`SoftTrigger::starts`], on words.
    #[inline]
    fn starts(&self, now: u64, prev: Option<u64>) -> bool {
        prev.is_some_and(|p| (!p & now & self.start_rise) | (p & !now & self.start_fall) != 0)
    }
}

/// One stage, flattened.
#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)]
struct Stage {
    conds: Vec<(usize, TriggerMatch)>,
    range: Option<TriggerRange>,
    /// The same, for samples of at most 8 bytes.
    masks: Masks,
    /// The block filter, for samples of 1, 2 or 4 bytes.
    block: Block,
    has_edge: bool,
    count: u32,
    consecutive: bool,
    /// Samples the condition must hold, 0 for none.
    duration: u64,
    /// A pulse width stage: the widths that fire, both inclusive.
    pulse: Option<(u64, u64)>,
    /// And the edge that starts its pulse.
    start: Option<(usize, TriggerMatch)>,
    /// Events seen so far.
    hits: u32,
    /// Samples in a row the condition held. Of a pulse width stage, the
    /// samples of the running pulse, 0 when none whose start was seen
    /// is running.
    run: u64,
    /// Whether the condition held on the previous sample.
    held: bool,
    /// Or/and: whether this stage has fired.
    done: bool,
}

/// The matcher state.
#[derive(Clone, Debug)]
pub struct SoftTrigger {
    stages: Vec<Stage>,
    combine: TriggerCombine,
    unit_size: usize,
    /// Sequence: the stage being matched.
    stage: usize,
    fired: bool,
    /// The previous sample, as bytes or (at most 8 bytes) as a word.
    prev: Option<Vec<u8>>,
    prev_word: Option<u64>,
    /// Samples still to pass before the matcher is armed.
    holdoff: u64,
}

impl SoftTrigger {
    /// What this matcher supports.
    pub const FEATURES: TriggerFeatures = TriggerFeatures {
        count: true,
        consecutive: true,
        duration: true,
        pulse: true,
        range: true,
        or: true,
        and: true,
        max_stages: 0,
        holdoff: true,
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
            .map(|s| {
                let conds: Vec<_> = s.conditions.iter().map(|c| (c.channel, c.matches)).collect();
                let mut masks = Masks::new(&conds, s.range.as_ref(), unit_size);
                // The edge that ends a pulse is a condition like any
                // other; the one that starts it is the opposite.
                let start = s.pulse().map(|(ch, m)| {
                    let rises = m == TriggerMatch::Falling;
                    if ch < unit_size * 8 && unit_size <= 8 {
                        let bit = 1u64 << ch;
                        (masks.start_rise, masks.start_fall) =
                            if rises { (bit, 0) } else { (0, bit) };
                    }
                    (ch, if rises { TriggerMatch::Rising } else { TriggerMatch::Falling })
                });
                let consecutive = s.count.is_some_and(|c| c.consecutive);
                let block = if lanes(unit_size) > 1 {
                    masks.block(unit_size, consecutive && s.has_edge())
                } else {
                    Block::default()
                };
                (s, conds, masks, block, start)
            })
            .map(|(s, conds, masks, block, start)| Stage {
                conds,
                range: s.range.clone(),
                masks,
                block,
                has_edge: s.has_edge(),
                count: s.count.map_or(1, |c| c.count.max(1)),
                consecutive: s.count.is_some_and(|c| c.consecutive),
                duration: match s.duration {
                    Some(TriggerDuration::Longer(n)) if start.is_none() => n,
                    _ => 0,
                },
                pulse: start.and(s.duration).map(TriggerDuration::bounds),
                start,
                hits: 0,
                run: 0,
                held: false,
                done: false,
            })
            .collect();
        // Or and and need two stages; anything else runs as a sequence.
        let combine =
            if trigger.stages.len() == 2 { trigger.combine } else { TriggerCombine::Sequence };
        Some(SoftTrigger {
            stages,
            combine,
            unit_size,
            stage: 0,
            fired: false,
            prev: None,
            prev_word: None,
            holdoff: trigger.holdoff,
        })
    }

    /// Arms the matcher again, its stages from the start, with `prev`
    /// as the sample before the next one fed: an edge between them is
    /// seen. `None` forgets it, as a new matcher does. A pulse running
    /// on `prev` started before the matcher was armed, and does not
    /// count for a pulse width stage.
    pub fn rearm(&mut self, prev: Option<&[u8]>) {
        // A counted level-only stage counts entries into its condition: one
        // that already held on `prev` is not entered again by the next
        // sample. A count of one still fires on a level that holds, and a
        // later stage of a sequence is armed afresh, as at the start.
        for idx in 0..self.stages.len() {
            let s = &self.stages[idx];
            let held = s.count > 1
                && s.duration == 0
                && !s.conds.iter().any(|(_, m)| m.is_edge())
                && (self.combine != TriggerCombine::Sequence || idx == 0)
                && prev.is_some_and(|sample| Self::edges_and_levels(s, sample, None).1);
            let s = &mut self.stages[idx];
            (s.hits, s.run, s.held, s.done) = (0, 0, held, false);
        }
        (self.stage, self.fired) = (0, false);
        if self.unit_size <= 8 {
            self.prev_word = prev.map(word);
        } else {
            self.prev = prev.map(<[u8]>::to_vec);
        }
    }

    /// Bytes per sample.
    pub fn unit_size(&self) -> usize {
        self.unit_size
    }

    /// True once the trigger fired.
    pub fn fired(&self) -> bool {
        self.fired
    }

    fn bit(sample: &[u8], channel: usize) -> Option<bool> {
        sample.get(channel / 8).map(|b| (b >> (channel % 8)) & 1 != 0)
    }

    fn matches_one(sample: &[u8], prev: Option<&[u8]>, ch: usize, m: TriggerMatch) -> bool {
        let Some(now) = Self::bit(sample, ch) else { return false };
        let before = prev.and_then(|p| Self::bit(p, ch));
        match m {
            TriggerMatch::Zero => !now,
            TriggerMatch::One => now,
            TriggerMatch::Rising => before == Some(false) && now,
            TriggerMatch::Falling => before == Some(true) && !now,
            TriggerMatch::Edge => before.is_some_and(|b| b != now),
            TriggerMatch::Over | TriggerMatch::Under => false,
        }
    }

    /// Whether the group's value, least significant channel first, is
    /// inside (or outside) the range. A channel beyond the sample width
    /// never matches.
    fn range_holds(sample: &[u8], r: &TriggerRange) -> bool {
        let mut value = 0u64;
        for (k, &ch) in r.channels.iter().enumerate() {
            match Self::bit(sample, ch) {
                Some(b) => value |= u64::from(b) << k,
                None => return false,
            }
        }
        (r.low..=r.high).contains(&value) == r.inside
    }

    /// Whether the stage's edge conditions all match, and whether its
    /// level conditions and value range all hold, on this sample.
    fn edges_and_levels(st: &Stage, sample: &[u8], prev: Option<&[u8]>) -> (bool, bool) {
        let edges = st
            .conds
            .iter()
            .filter(|(_, m)| m.is_edge())
            .all(|&(ch, m)| Self::matches_one(sample, prev, ch, m));
        let levels = st
            .conds
            .iter()
            .filter(|(_, m)| !m.is_edge())
            .all(|&(ch, m)| Self::matches_one(sample, prev, ch, m))
            && st.range.as_ref().is_none_or(|r| Self::range_holds(sample, r));
        (edges, levels)
    }

    /// Whether the pulse of a pulse width stage starts on this sample.
    fn starts(st: &Stage, sample: &[u8], prev: Option<&[u8]>) -> bool {
        st.start.is_some_and(|(ch, m)| Self::matches_one(sample, prev, ch, m))
    }

    /// Advances stage `idx` by one sample, given what `edges_and_levels`
    /// says of it; true when it fired.
    fn step(&mut self, idx: usize, edges: bool, levels: bool, start: bool) -> bool {
        let matched = edges && levels;
        let st = &mut self.stages[idx];
        let has_edge = st.has_edge;
        if let Some((min, max)) = st.pulse {
            // `run` counts a pulse from the edge that started it; the
            // edge that ends it fires when the width fits, and the
            // levels hold on that sample. No edge at all leaves the
            // channel where it was: in the pulse, when one is running.
            if start {
                st.run = 1;
            } else if edges {
                let width = std::mem::take(&mut st.run);
                return levels && (min..=max).contains(&width);
            } else if st.run > 0 {
                st.run = st.run.saturating_add(1);
            }
            return false;
        }
        if st.duration > 0 {
            st.run = if matched { st.run + 1 } else { 0 };
            return st.run == st.duration + 1;
        }
        // A level-only stage counts the times it becomes true. A
        // consecutive count starts over when the counted edge comes
        // while the levels do not hold.
        let event = if has_edge { matched } else { matched && !st.held };
        let reset = st.consecutive && has_edge && edges && !levels;
        st.held = matched;
        if reset {
            st.hits = 0;
        }
        if event {
            st.hits += 1;
            if st.hits >= st.count {
                return true;
            }
        }
        false
    }

    /// Advances the trigger by one sample, which `eval` matches against
    /// a stage and of which `starts` says whether the stage's pulse
    /// starts on it; true when it fired.
    #[inline]
    fn advance(
        &mut self,
        eval: impl Fn(&Stage) -> (bool, bool),
        starts: impl Fn(&Stage) -> bool,
    ) -> bool {
        // Only a pulse width stage asks whether its pulse starts.
        let starts = |st: &Stage| st.pulse.is_some() && starts(st);
        match self.combine {
            TriggerCombine::Sequence => {
                let st = &self.stages[self.stage];
                let ((edges, levels), start) = (eval(st), starts(st));
                if self.step(self.stage, edges, levels, start) {
                    self.stage += 1;
                }
                self.stage >= self.stages.len()
            }
            TriggerCombine::Or | TriggerCombine::And => {
                // Every stage still waiting sees every sample.
                for idx in 0..self.stages.len() {
                    if self.stages[idx].done {
                        continue;
                    }
                    let st = &self.stages[idx];
                    let ((edges, levels), start) = (eval(st), starts(st));
                    if self.step(idx, edges, levels, start) {
                        self.stages[idx].done = true;
                    }
                }
                if self.combine == TriggerCombine::Or {
                    self.stages.iter().any(|s| s.done)
                } else {
                    self.stages.iter().all(|s| s.done)
                }
            }
        }
    }

    /// Whether stage `i` sees the next sample.
    fn active(&self, i: usize) -> bool {
        if self.combine == TriggerCombine::Sequence {
            i == self.stage
        } else {
            !self.stages[i].done
        }
    }

    /// When no sample of the block `now` (lanes) after `prev` can match
    /// a waiting stage, applies what the block does: nothing held, no
    /// duration running. A pulse width stage's channel does not change
    /// in such a block, so a running pulse grows by the block. True
    /// when it skipped the block.
    fn skip_block(&mut self, now: u64, prev: u64) -> bool {
        let prev = now << (self.unit_size * 8) | prev;
        if (0..self.stages.len())
            .any(|i| self.active(i) && self.stages[i].block.may_match(now, prev))
        {
            return false;
        }
        let n = lanes(self.unit_size) as u64;
        for i in 0..self.stages.len() {
            if !self.active(i) {
                continue;
            }
            let st = &mut self.stages[i];
            if st.pulse.is_none() {
                (st.held, st.run) = (false, 0);
            } else if st.run > 0 {
                st.run = st.run.saturating_add(n);
            }
        }
        true
    }

    /// After one sample `p` of a block of samples all equal to `p` went
    /// through `advance` and left the stages where they were: whether
    /// the other `n` change nothing but the duration runs, which they
    /// then advance. They match as `p` did against itself: no edge, no
    /// new event; only a duration that completes inside them needs
    /// each sample. A pulse width stage fires on an edge alone, so its
    /// running pulse just grows.
    fn skip_same(&mut self, n: u64) -> bool {
        let reach = |st: &Stage| st.duration > 0 && st.run > 0 && st.run + n > st.duration;
        if (0..self.stages.len()).any(|i| self.active(i) && reach(&self.stages[i])) {
            return false;
        }
        for i in 0..self.stages.len() {
            let st = &self.stages[i];
            if self.active(i) && (st.duration > 0 || st.pulse.is_some()) && st.run > 0 {
                self.stages[i].run = st.run.saturating_add(n);
            }
        }
        true
    }

    /// Runs the whole samples of at most 8 bytes in `data` through the
    /// matcher one by one, after `prev`. Returns the index of the
    /// sample that completed the trigger, or the last sample. Its own
    /// function, with `prev` a local, so the loop is compiled on its
    /// own whatever `feed` grows to, and carries nothing through memory.
    #[inline(never)]
    fn feed_words(
        &mut self,
        data: &[u8],
        mut prev: Option<u64>,
    ) -> std::result::Result<Option<u64>, usize> {
        for (i, sample) in data.chunks_exact(self.unit_size).enumerate() {
            let now = word(sample);
            if self
                .advance(|st| st.masks.edges_and_levels(now, prev), |st| st.masks.starts(now, prev))
            {
                return Err(i);
            }
            prev = Some(now);
        }
        Ok(prev)
    }

    /// Runs the samples in `data` through the matcher. Returns the
    /// index of the sample that completed the trigger, if it did here.
    /// After that, calls return `None`.
    ///
    /// The samples of the trigger's hold-off pass unmatched: the matcher
    /// is armed after them, with the last of them as the previous
    /// sample, as [`SoftTrigger::rearm`] arms it.
    pub fn feed(&mut self, data: &[u8]) -> Option<usize> {
        if self.fired {
            return None;
        }
        if self.holdoff > 0 {
            let us = self.unit_size;
            let n = data.len() / us;
            let skip = usize::try_from(self.holdoff).map_or(n, |h| h.min(n));
            self.holdoff -= skip as u64;
            if skip > 0 {
                self.rearm(Some(&data[(skip - 1) * us..skip * us]));
            }
            return self.feed_armed(&data[skip * us..]).map(|i| i + skip);
        }
        self.feed_armed(data)
    }

    /// [`SoftTrigger::feed`], past the hold-off.
    fn feed_armed(&mut self, data: &[u8]) -> Option<usize> {
        if self.unit_size <= 8 {
            let us = self.unit_size;
            let (n, lanes) = (data.len() / us, lanes(us));
            let top = 64 - us * 8;
            let ones = broadcast(1, us);
            let mut prev = self.prev_word;
            let mut i = 0;
            // Where to look at a block again after one was not skipped,
            // later each time, so that busy data costs almost nothing.
            let (mut next_block, mut stride) = (0, lanes);
            while i < n {
                // A whole block of samples after a known one: skip it
                // when nothing in it can match, or when it repeats the
                // previous sample.
                if lanes > 1
                    && i >= next_block
                    && i + lanes <= n
                    && let Some(p) = prev
                {
                    next_block = i + stride;
                    stride = (stride * 2).min(lanes * 64);
                    let block = word(&data[i * us..i * us + 8]);
                    if self.skip_block(block, p) {
                        prev = Some(block >> top);
                        i += lanes;
                        (next_block, stride) = (i, lanes);
                        continue;
                    }
                    if block == p.wrapping_mul(ones) {
                        let stage = self.stage;
                        let done = self.stages.iter().filter(|s| s.done).count();
                        if self.advance(|st| st.masks.edges_and_levels(p, Some(p)), |_| false) {
                            self.fired = true;
                            self.prev_word = None;
                            return Some(i);
                        }
                        i += 1;
                        let moved = stage != self.stage
                            || done != self.stages.iter().filter(|s| s.done).count();
                        if !moved && self.skip_same(lanes as u64 - 1) {
                            i += lanes - 1;
                            (next_block, stride) = (i, lanes);
                        }
                        continue;
                    }
                }
                // Sample by sample up to the next block, or to the end
                // when samples have no lanes.
                let stop = if lanes > 1 { next_block.max(i + 1).min(n) } else { n };
                match self.feed_words(&data[i * us..stop * us], prev) {
                    Ok(p) => prev = p,
                    Err(k) => {
                        self.fired = true;
                        self.prev_word = None;
                        return Some(i + k);
                    }
                }
                i = stop;
            }
            self.prev_word = prev;
            return None;
        }
        let mut prev = self.prev.take();
        for (i, sample) in data.chunks_exact(self.unit_size).enumerate() {
            let before = prev.as_deref();
            if self.advance(
                |st| Self::edges_and_levels(st, sample, before),
                |st| Self::starts(st, sample, before),
            ) {
                self.fired = true;
                return Some(i);
            }
            match &mut prev {
                Some(p) => p.copy_from_slice(sample),
                None => prev = Some(sample.to_vec()),
            }
        }
        self.prev = prev;
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
/// and edge matches; [`SoftTrigger`] is the general matcher. One fix:
/// a restart that reaches back past the buffer starts at its first
/// sample, where C starts at its second byte.
#[derive(Clone, Debug)]
pub struct SoftTriggerLogic {
    /// Per stage, the matches on enabled channels.
    stages: Vec<Vec<(usize, TriggerMatch)>>,
    /// The same as masks, for samples of at most 8 bytes.
    masks: Option<Vec<LogicMasks>>,
    /// The first stage's block filter, for samples of 1, 2 or 4 bytes.
    block: Option<Block>,
    unit_size: usize,
    /// The stage being matched.
    pub cur_stage: usize,
    /// Whether a match was evaluated yet (`stl->count` > 0): the very
    /// first match evaluated never fires when it is an edge.
    evaluated: bool,
    prev: Vec<u8>,
    ring: Vec<u8>,
    head: usize,
    fill: usize,
}

/// A stage of [`SoftTriggerLogic`] as masks over a sample word. A
/// channel beyond the sample reads low, as in C.
#[derive(Clone, Copy, Debug, Default)]
struct LogicMasks {
    /// No matches: the stage matches without evaluating any.
    empty: bool,
    first_is_edge: bool,
    /// One, an edge or Over/Under on a channel past bit 63.
    never: bool,
    zero: u64,
    one: u64,
    rise: u64,
    fall: u64,
    change: u64,
}

impl LogicMasks {
    fn new(matches: &[(usize, TriggerMatch)]) -> Self {
        let mut m = LogicMasks {
            empty: matches.is_empty(),
            first_is_edge: matches.first().is_some_and(|(_, t)| t.is_edge()),
            ..LogicMasks::default()
        };
        for &(ch, t) in matches {
            let Some(bit) = 1u64.checked_shl(ch as u32).filter(|_| ch < 64) else {
                m.never |= t != TriggerMatch::Zero;
                continue;
            };
            match t {
                TriggerMatch::Zero => m.zero |= bit,
                TriggerMatch::One => m.one |= bit,
                TriggerMatch::Rising => m.rise |= bit,
                TriggerMatch::Falling => m.fall |= bit,
                TriggerMatch::Edge => m.change |= bit,
                TriggerMatch::Over | TriggerMatch::Under => m.never = true,
            }
        }
        m
    }

    /// Whether all matches hold; `first` for the first evaluation ever.
    #[inline]
    fn matches(&self, now: u64, prev: u64, first: bool) -> bool {
        (!first || !self.first_is_edge)
            && !self.never
            && now & self.zero == 0
            && now & self.one == self.one
            && !prev & now & self.rise == self.rise
            && prev & !now & self.fall == self.fall
            && (prev ^ now) & self.change == self.change
    }
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
        let masks = (unit_size <= 8).then(|| stages.iter().map(|s| LogicMasks::new(s)).collect());
        // A channel beyond the sample reads low: a zero on it always
        // holds, anything else never does.
        let block = stages.first().filter(|s| lanes(unit_size) > 1 && !s.is_empty()).map(|s| {
            let width = unit_size * 8;
            let m = LogicMasks::new(s);
            let inside = (1u64 << width) - 1;
            let beyond = s.iter().any(|&(ch, t)| ch >= width && t != TriggerMatch::Zero);
            Block::new(
                [m.zero & inside, m.one, m.rise, m.fall, m.change],
                m.never || beyond,
                unit_size,
            )
        });
        SoftTriggerLogic {
            stages,
            masks,
            block,
            unit_size,
            cur_stage: 0,
            evaluated: false,
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
        let first = !self.evaluated;
        self.evaluated = true;
        let bit = |s: &[u8]| s.get(ch / 8).is_some_and(|b| b & (1 << (ch % 8)) != 0);
        let now = bit(sample);
        match m {
            TriggerMatch::Zero => !now,
            TriggerMatch::One => now,
            _ if first => false,
            TriggerMatch::Rising => !bit(&self.prev) && now,
            TriggerMatch::Falling => bit(&self.prev) && !now,
            TriggerMatch::Edge => bit(&self.prev) != now,
            TriggerMatch::Over | TriggerMatch::Under => false,
        }
    }

    /// The loop of `soft_trigger_logic_check()` over `buf` from byte
    /// `i` (a sample boundary) to `stop`, with `words` for whole samples
    /// of at most 8 bytes matched as words after `prev_word`. Returns
    /// where it stopped and the last sample as a word, or the byte
    /// offset of the trigger point and its sample as a word. A restart goes back below `i` as it
    /// does in C. Its own function, with `prev_word` a local, so the
    /// loop is compiled on its own whatever `check` grows to, and
    /// carries nothing through memory.
    #[inline(never)]
    fn check_run(
        &mut self,
        buf: &[u8],
        words: bool,
        mut i: isize,
        stop: isize,
        mut prev_word: u64,
    ) -> std::result::Result<(isize, u64), (usize, u64)> {
        let unit = self.unit_size as isize;
        while i < stop {
            let at = i as usize;
            let found = if let (true, Some(masks)) = (words, &self.masks) {
                let now = word(&buf[at..at + self.unit_size]);
                let found = masks.get(self.cur_stage).is_none_or(|m| {
                    m.empty || {
                        let first = !self.evaluated;
                        self.evaluated = true;
                        m.matches(now, prev_word, first)
                    }
                });
                prev_word = now;
                found
            } else {
                let sample = buf.get(at..at + self.unit_size).unwrap_or(&buf[at..]);
                let mut found = true;
                for k in 0..self.stages.get(self.cur_stage).map_or(0, Vec::len) {
                    let (ch, m) = self.stages[self.cur_stage][k];
                    if !self.check_match(sample, ch, m) {
                        found = false;
                        break;
                    }
                }
                let n = sample.len().min(self.unit_size);
                self.prev[..n].copy_from_slice(&sample[..n]);
                found
            };
            if found {
                if self.cur_stage + 1 < self.stages.len() {
                    self.cur_stage += 1;
                } else {
                    return Err((at, prev_word));
                }
            } else if self.cur_stage > 0 {
                // C clamps to byte -1, which is sample 0 only when a
                // sample is one byte; wider samples went out of step
                // with the buffer for the rest of it.
                i -= self.cur_stage as isize * unit;
                if i < -unit {
                    i = -unit;
                }
                self.cur_stage = 0;
            }
            i += unit;
        }
        Ok((i, prev_word))
    }

    /// `soft_trigger_logic_check()` on `buf`: the offset in samples of
    /// the trigger point and the pre-trigger packets to send before it,
    /// or `None` (the samples go into the pre-trigger ring).
    pub fn check(&mut self, buf: &[u8]) -> Option<(usize, Vec<Vec<u8>>)> {
        let unit = self.unit_size as isize;
        let len = buf.len() as isize;
        // Whole samples of at most 8 bytes match as words; the previous
        // one is kept in `prev_word` and written back on the way out.
        let words = self.masks.is_some() && buf.len().is_multiple_of(self.unit_size);
        let mut prev_word = if words { word(&self.prev) } else { 0 };
        // In the first stage a sample that fails only moves on, so a
        // block where none can match is skipped whole. `next_block` is
        // where to look at one again after one was not skipped, later
        // each time, so that busy data costs almost nothing.
        let block = self.block.filter(|_| words && lanes(self.unit_size) > 1);
        let width = self.unit_size * 8;
        let top = 64 - width.min(64);
        let (mut next_block, mut stride) = (0, 8);
        let mut i: isize = 0;
        while i < len {
            let at = i as usize;
            if at >= next_block
                && let Some(b) = &block
                && self.cur_stage == 0
                && at + 8 <= buf.len()
                && self.evaluated
            {
                next_block = at + stride;
                stride = (stride * 2).min(512);
                let now = word(&buf[at..at + 8]);
                if !b.may_match(now, now << width | prev_word) {
                    prev_word = now >> top;
                    i += 8;
                    (next_block, stride) = (at + 8, 8);
                    continue;
                }
            }
            // Sample by sample up to the next block, or to the end
            // without a block filter.
            let stop = if block.is_some() { (next_block as isize).max(i + unit) } else { len };
            let stop = stop.min(len);
            match self.check_run(buf, words, i, stop, prev_word) {
                Ok((next, p)) => (i, prev_word) = (next, p),
                Err((at, p)) => {
                    // The trigger sample is the previous one now, as
                    // in C, not the one before the run.
                    prev_word = p;
                    if words {
                        self.prev.copy_from_slice(&prev_word.to_le_bytes()[..self.unit_size]);
                    }
                    self.append(&buf[..at]);
                    let pre = self.take_pre();
                    return Some((at / self.unit_size, pre));
                }
            }
        }
        if words {
            self.prev.copy_from_slice(&prev_word.to_le_bytes()[..self.unit_size]);
        }
        self.append(buf);
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rearmed inside a counted range, the samples still in it are no new
    /// entry: the count starts with the next time the value comes back.
    #[test]
    fn a_rearm_inside_a_counted_range_is_no_entry() {
        let range = TriggerRange { channels: vec![0, 1], low: 1, high: 1, inside: true };
        let mut t = Trigger::new();
        t.set_range(0, range.clone());
        t.set_count(0, crate::core::trigger::TriggerCount { count: 2, consecutive: false });
        let mut m = SoftTrigger::new(&t, 1).unwrap();
        assert_eq!(m.feed(&[1, 0, 1]), Some(2));
        m.rearm(Some(&[1]));
        assert_eq!(m.feed(&[1, 1, 0, 1]), None, "held, left, entered once");
        assert_eq!(m.feed(&[0, 1]), Some(1), "entered twice");

        // A count of one still fires on a value that holds.
        let mut t = Trigger::new();
        t.set_range(0, range);
        let mut m = SoftTrigger::new(&t, 1).unwrap();
        assert_eq!(m.feed(&[1]), Some(0));
        m.rearm(Some(&[1]));
        assert_eq!(m.feed(&[1]), Some(0));
    }
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

    /// A hold-off passes its samples unmatched, across calls, and arms
    /// the matcher on the sample after them: an edge from the last held
    /// sample counts, a stage is not begun before, and a pulse that began
    /// before is no pulse.
    #[test]
    fn a_hold_off_arms_the_matcher_late() {
        let held =
            |stages: &[&[(usize, TriggerMatch)]], holdoff| Trigger { holdoff, ..trig(stages) };
        // D0 rises at samples 1, 3, 5, 7.
        let data = [0u8, 1, 0, 1, 0, 1, 0, 1];
        let rising: &[&[(usize, TriggerMatch)]] = &[&[(0, TriggerMatch::Rising)]];
        let first = |holdoff| SoftTrigger::new(&held(rising, holdoff), 1).unwrap().feed(&data);
        assert_eq!(first(0), Some(1));
        assert_eq!(first(1), Some(1), "the edge after the one held sample");
        assert_eq!(first(2), Some(3));
        assert_eq!(first(4), Some(5));
        assert_eq!(first(7), Some(7), "the last held sample is the previous one");
        assert_eq!(first(8), None);
        // Across calls: the index is in the call the trigger fires in.
        let mut s = SoftTrigger::new(&held(rising, 10), 1).unwrap();
        assert_eq!(s.feed(&data), None);
        assert_eq!(s.feed(&data), Some(3), "two held, then the rise at 3");
        assert_eq!(s.feed(&data), None, "fired");
        // A level that holds fires on the first sample after.
        let high: &[&[(usize, TriggerMatch)]] = &[&[(0, TriggerMatch::One)]];
        assert_eq!(SoftTrigger::new(&held(high, 4), 1).unwrap().feed(&[1, 1, 1, 1, 1, 1]), Some(4));
        // Stage A in the hold-off does not count: A (D1 rising) and then
        // B (D0 rising) both come again after it.
        let two: &[&[(usize, TriggerMatch)]] =
            &[&[(1, TriggerMatch::Rising)], &[(0, TriggerMatch::Rising)]];
        let data = [0u8, 2, 2, 3, 0, 1, 2, 2, 3];
        assert_eq!(SoftTrigger::new(&held(two, 0), 1).unwrap().feed(&data), Some(3));
        assert_eq!(SoftTrigger::new(&held(two, 2), 1).unwrap().feed(&data), Some(8));
        // A pulse whose start was held is not measured.
        let mut pulse = held(&[&[(0, TriggerMatch::Falling)]], 2);
        pulse.set_duration(0, TriggerDuration::Longer(0));
        let data = [0u8, 1, 1, 0, 0, 1, 1, 0];
        assert_eq!(SoftTrigger::new(&pulse, 1).unwrap().feed(&data), Some(7));
        // Wide samples take the same path.
        let wide: Vec<u8> = data.iter().flat_map(|b| [*b, 0, 0, 0, 0, 0, 0, 0, 0]).collect();
        assert_eq!(SoftTrigger::new(&pulse, 9).unwrap().feed(&wide), Some(7));
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
        assert!(SoftTrigger::check(&r).is_ok());
    }

    fn range(channels: &[usize], low: u64, high: u64, inside: bool) -> TriggerRange {
        TriggerRange { channels: channels.to_vec(), low, high, inside }
    }

    #[test]
    fn value_ranges() {
        // D0-D2 count 0..7: the value enters 3..5 at index 3.
        let count: Vec<u8> = (0..8).collect();
        let mut t = Trigger::new();
        t.set_range(0, range(&[0, 1, 2], 3, 5, true));
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&count), Some(3));
        // Outside 0..5 first holds at 6.
        let mut t = Trigger::new();
        t.set_range(0, range(&[0, 1, 2], 0, 5, false));
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&count), Some(6));
        // The first channel is the least significant: D2 D1 D0 read as
        // 0b011 (= 3) only when D0 is the top bit.
        let mut t = Trigger::new();
        t.set_range(0, range(&[2, 1, 0], 6, 6, true));
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&count), Some(3));
        // A range that already holds on the first sample fires there.
        let mut t = Trigger::new();
        t.set_range(0, range(&[0, 1], 0, 0, true));
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&count), Some(0));
        // Channels beyond the sample width never match.
        let mut t = Trigger::new();
        t.set_range(0, range(&[0, 9], 0, 3, true));
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&count), None);
    }

    #[test]
    fn ranges_qualify_edges_and_count() {
        // D3 rises at 1, 3 and 5; D0-D1 hold 1, 2, then 2.
        let data = [0x00, 0x09, 0x01, 0x0a, 0x02, 0x0a];
        let mut t = trig(&[&[(3, TriggerMatch::Rising)]]);
        t.set_range(0, range(&[0, 1], 2, 2, true));
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&data), Some(3));
        // Entering the range is one event: the second entry fires.
        let mut t = Trigger::new();
        t.set_range(0, range(&[0, 1], 2, 2, true));
        t.set_count(0, TriggerCount { count: 2, consecutive: false });
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&[2, 2, 0, 2]), Some(3));
        // And a range held for a duration.
        let mut t = Trigger::new();
        t.set_range(0, range(&[0, 1], 2, 2, true));
        t.set_duration(0, TriggerDuration::Longer(2));
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&[2, 2, 0, 2, 2, 2]), Some(5));
    }

    #[test]
    fn or_and_combines() {
        // D0 rises at 4, D1 rises at 2.
        let data = [0b00, 0b00, 0b10, 0b10, 0b11];
        let mut t = trig(&[&[(0, TriggerMatch::Rising)], &[(1, TriggerMatch::Rising)]]);
        t.combine = TriggerCombine::Or;
        assert!(SoftTrigger::check(&t).is_ok());
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&data), Some(2));
        // Both, in either order: complete at D0's edge.
        t.combine = TriggerCombine::And;
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&data), Some(4));
        // A sequence needs D0 first, then D1: D1 never rises again.
        t.combine = TriggerCombine::Sequence;
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&data), None);
        // And across chunks, with the stage that fired first remembered.
        t.combine = TriggerCombine::And;
        let mut s = SoftTrigger::new(&t, 1).unwrap();
        assert_eq!(s.feed(&data[..3]), None);
        assert_eq!(s.feed(&data[3..]), Some(1));
        assert!(s.fired());
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

    #[test]
    fn a_restart_before_the_buffer_keeps_wide_samples_in_step() {
        // 2 byte samples: D0 rising, then D15 high, which never is. The
        // first stage matches on the last sample of a buffer and the
        // second fails on the first of the next. The restart must read
        // whole samples: half a sample off, D8 of the second sample
        // reads as D0 and D7 of the fourth as D15.
        let mut s = SoftTriggerLogic::new(
            vec![vec![(0, TriggerMatch::Rising)], vec![(15, TriggerMatch::One)]],
            2,
            0,
        );
        assert!(s.check(&[0x00, 0x00, 0x01, 0x00]).is_none());
        assert!(s.check(&[0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 0x00]).is_none());
        // It still fires on a real match after the restart.
        assert_eq!(s.check(&[0x00, 0x00, 0x01, 0x00, 0x00, 0x80]).map(|r| r.0), Some(2));
    }

    /// xorshift64, for the comparison below.
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }

        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
    }

    fn random_match(rng: &mut Rng) -> TriggerMatch {
        [
            TriggerMatch::Zero,
            TriggerMatch::One,
            TriggerMatch::Rising,
            TriggerMatch::Falling,
            TriggerMatch::Edge,
        ][rng.below(5) as usize]
    }

    /// Samples of `unit` bytes whose bits flip now and then and that
    /// sometimes hold for a stretch, so levels hold, durations run,
    /// blocks of equal samples occur and triggers fire; and the same
    /// padded to 9 bytes, which the matchers read byte by byte.
    fn random_samples(rng: &mut Rng, unit: usize, n: usize) -> (Vec<u8>, Vec<u8>) {
        let (mut narrow, mut wide) = (Vec::new(), Vec::new());
        let mut v = rng.next();
        let mut hold = 0;
        for _ in 0..n {
            if hold > 0 {
                hold -= 1;
            } else {
                v ^= rng.next() & rng.next() & rng.next();
                if rng.below(4) == 0 {
                    hold = rng.below(24);
                }
            }
            let bytes = v.to_le_bytes();
            narrow.extend_from_slice(&bytes[..unit]);
            wide.extend_from_slice(&bytes[..unit]);
            wide.extend(std::iter::repeat_n(0, 9 - unit));
        }
        (narrow, wide)
    }

    /// Random chunk lengths in samples, summing to `n`.
    fn random_chunks(rng: &mut Rng, n: usize) -> Vec<usize> {
        let mut out = Vec::new();
        let mut left = n;
        while left > 0 {
            let k = (1 + rng.below(40) as usize).min(left);
            out.push(k);
            left -= k;
        }
        out
    }

    /// `n` one byte samples, each `(channel, from, to)` high on the
    /// samples `from..to`.
    fn wave(n: usize, high: &[(usize, usize, usize)]) -> Vec<u8> {
        (0..n)
            .map(|i| {
                high.iter()
                    .filter(|&&(_, a, b)| (a..b).contains(&i))
                    .fold(0, |v, &(ch, ..)| v | 1 << ch)
            })
            .collect()
    }

    /// One byte samples as samples of `unit` bytes.
    fn widen(data: &[u8], unit: usize) -> Vec<u8> {
        data.iter()
            .flat_map(|&b| std::iter::once(b).chain(std::iter::repeat_n(0, unit - 1)))
            .collect()
    }

    fn pulse(edge: TriggerMatch, d: TriggerDuration, levels: &[(usize, TriggerMatch)]) -> Trigger {
        let mut t = Trigger::new();
        t.add(0, 0, edge);
        for &(ch, m) in levels {
            t.add(0, ch, m);
        }
        t.set_duration(0, d);
        assert!(SoftTrigger::check(&t).is_ok(), "{t:?}");
        t
    }

    /// Where `t` first fires in `data`, the same for samples of 1, 2, 4,
    /// 3 and 9 bytes and wherever the data is cut in two.
    fn fires_at(t: &Trigger, data: &[u8]) -> Option<usize> {
        let whole = SoftTrigger::new(t, 1).unwrap().feed(data);
        for unit in [1, 2, 4, 3, 9] {
            let wide = widen(data, unit);
            for cut in 0..=data.len() {
                let mut m = SoftTrigger::new(t, unit).unwrap();
                let at = m
                    .feed(&wide[..cut * unit])
                    .or_else(|| m.feed(&wide[cut * unit..]).map(|i| cut + i));
                assert_eq!(at, whole, "unit {unit}, cut at {cut}: {t:?}");
            }
        }
        whole
    }

    #[test]
    fn pulse_widths_fire_on_the_edge_that_ends_the_pulse() {
        use TriggerDuration::{Between, Longer, Shorter};
        use TriggerMatch::{Falling, One, Rising, Zero};
        // D0 rises at 10, falls at 13, rises at 20, falls at 30; D1 is
        // high from 15 to 25.
        let data = wave(40, &[(0, 10, 13), (0, 20, 30), (1, 15, 25)]);
        for (edge, d, levels, at) in [
            (Falling, Between(3, 3), &[][..], Some(13)),
            (Falling, Between(4, 10), &[], Some(30)),
            (Falling, Between(4, 9), &[], None),
            // The low time between the pulses, 13 to 20.
            (Rising, Between(7, 7), &[], Some(20)),
            // Levels are those of the sample where the pulse ends.
            (Falling, Between(1, 100), &[(1, Zero)], Some(13)),
            (Falling, Between(5, 100), &[(1, One)], None),
            (Rising, Between(1, 100), &[(1, One)], Some(20)),
            (Falling, Shorter(4), &[], Some(13)),
            (Falling, Shorter(3), &[], None),
            (Falling, Shorter(11), &[(1, Zero)], Some(13)),
            // Longer fires at the end of the pulse, not inside it.
            (Falling, Longer(2), &[], Some(13)),
            // Longer than no sample: the first pulse seen whole.
            (Falling, Longer(0), &[], Some(13)),
            (Rising, Longer(0), &[], Some(20)),
            (Falling, Longer(3), &[], Some(30)),
            (Falling, Longer(10), &[], None),
            (Rising, Longer(5), &[], Some(20)),
            // The low time before the first pulse has no start: its 10
            // samples are no pulse, nor are the 10 after the last.
            (Rising, Between(10, 10), &[], None),
            (Rising, Longer(7), &[], None),
        ] {
            assert_eq!(fires_at(&pulse(edge, d, levels), &data), at, "{edge:?} {d:?} {levels:?}");
        }
        // A value range is checked on the edge too: D1-D2 read 1 at 20.
        let mut t = pulse(Rising, Between(1, 100), &[]);
        t.set_range(0, range(&[1, 2], 1, 1, true));
        assert_eq!(fires_at(&t, &data), Some(20));
        let mut t = pulse(Falling, Between(1, 100), &[]);
        t.set_range(0, range(&[1, 2], 1, 1, true));
        assert_eq!(fires_at(&t, &data), None);
        // A channel beyond the samples never fires.
        let mut t = Trigger::new();
        t.add(0, 8, Falling);
        t.set_duration(0, Shorter(100));
        assert_eq!(SoftTrigger::new(&t, 1).unwrap().feed(&[0xff, 0, 0xff, 0]), None);
    }

    #[test]
    fn a_pulse_already_running_does_not_count() {
        use TriggerDuration::Between;
        use TriggerMatch::{Falling, Rising};
        let any = pulse(Falling, Between(1, 100), &[]);
        // High on the first sample: the pulse that ends at 5 started
        // before the capture. The next one counts.
        let data = wave(40, &[(0, 0, 5), (0, 10, 13), (0, 20, 30)]);
        assert_eq!(fires_at(&any, &data), Some(13));
        // Rearmed on the trigger sample, the next pulse is seen whole.
        let mut m = SoftTrigger::new(&any, 1).unwrap();
        assert_eq!(m.feed(&data), Some(13));
        m.rearm(Some(&data[13..14]));
        assert_eq!(m.feed(&data[14..]), Some(16));
        // Rearmed inside the pulse, it is not: nothing is left to fire.
        m.rearm(Some(&data[21..22]));
        assert_eq!(m.feed(&data[22..]), None);
        // Rearmed on the sample before the pulse starts, its start is
        // the edge between that sample and the next.
        m.rearm(Some(&data[19..20]));
        assert_eq!(m.feed(&data[20..]), Some(10));
        // Without a previous sample the first one can't be a start.
        m.rearm(None);
        assert_eq!(m.feed(&data[20..]), None);

        // A sequence arms its second stage when D1 rises at 22, inside
        // the pulse on D0 that ends at 30, which so does not count; the
        // one from 34 to 37 does.
        let seq = |d1_rises: usize| {
            let mut t = trig(&[&[(1, Rising)], &[(0, Falling)]]);
            t.set_duration(1, Between(1, 100));
            assert!(SoftTrigger::check(&t).is_ok());
            let data = wave(40, &[(0, 10, 13), (0, 20, 30), (0, 34, 37), (1, d1_rises, 40)]);
            fires_at(&t, &data)
        };
        assert_eq!(seq(22), Some(37));
        assert_eq!(seq(20), Some(37), "armed on the pulse's first sample");
        // Armed on the sample before the pulse starts, the stage sees
        // the start as its first sample.
        assert_eq!(seq(19), Some(30));
        assert_eq!(seq(5), Some(13));
    }

    #[test]
    fn pulse_widths_with_or_and_and() {
        use TriggerDuration::{Between, Longer};
        use TriggerMatch::{Falling, One, Rising};
        // D0 pulses from 10 to 13 and from 20 to 30; D1 rises at 15.
        let data = wave(40, &[(0, 10, 13), (0, 20, 30), (1, 15, 40)]);
        let two = |d: TriggerDuration, combine: TriggerCombine| {
            let mut t = trig(&[&[(0, Falling)], &[(1, Rising)]]);
            t.set_duration(0, d);
            t.combine = combine;
            assert!(SoftTrigger::check(&t).is_ok());
            fires_at(&t, &data)
        };
        assert_eq!(two(Between(3, 3), TriggerCombine::Or), Some(13));
        assert_eq!(two(Between(10, 10), TriggerCombine::Or), Some(15));
        assert_eq!(two(Between(3, 3), TriggerCombine::And), Some(15));
        assert_eq!(two(Between(10, 10), TriggerCombine::And), Some(30));
        assert_eq!(two(Between(5, 5), TriggerCombine::And), None);

        // Long pulses over quiet data, where whole blocks are skipped:
        // the width still counts every sample.
        let data = wave(3000, &[(0, 100, 1100), (0, 1500, 2501)]);
        assert_eq!(fires_at(&pulse(Falling, Between(1000, 1000), &[]), &data), Some(1100));
        assert_eq!(fires_at(&pulse(Falling, Between(1001, 1001), &[]), &data), Some(2501));
        assert_eq!(fires_at(&pulse(Falling, Longer(1001), &[]), &data), None);
        assert_eq!(fires_at(&pulse(Rising, Between(400, 400), &[]), &data), Some(1500));
        // Beside a level held for a duration, whose stage keeps the
        // matcher on blocks of equal samples.
        let data = wave(3000, &[(0, 100, 600), (1, 0, 3000)]);
        let mut t = trig(&[&[(1, One)], &[(0, Falling)]]);
        t.set_duration(0, Longer(1000));
        t.set_duration(1, Between(500, 500));
        t.combine = TriggerCombine::Or;
        assert_eq!(fires_at(&t, &data), Some(600));
        t.set_duration(1, Between(499, 499));
        assert_eq!(fires_at(&t, &data), Some(1000));
    }

    /// Every sample of `data` (one channel a byte's bit, `unit` bytes a
    /// sample) that ends a pulse on `ch` whose start is in the data and
    /// whose width is within `min..=max`, with `level` holding there:
    /// found by walking the samples, with nothing of the matcher.
    fn pulses_by_scan(
        data: &[u8],
        unit: usize,
        ch: usize,
        high: bool,
        (min, max): (u64, u64),
        level: Option<(usize, bool)>,
    ) -> Vec<usize> {
        let bit = |i: usize, ch: usize| data[i * unit + ch / 8] >> (ch % 8) & 1 != 0;
        let n = data.len() / unit;
        let mut out = Vec::new();
        let mut start = None;
        for i in 1..n {
            let (before, now) = (bit(i - 1, ch) == high, bit(i, ch) == high);
            if !before && now {
                start = Some(i);
            } else if before
                && !now
                && let Some(first) = start.take()
            {
                let width = (i - first) as u64;
                if (min..=max).contains(&width) && level.is_none_or(|(c, l)| bit(i, c) == l) {
                    out.push(i);
                }
            }
        }
        out
    }

    #[test]
    fn pulse_widths_match_a_scan_of_the_samples() {
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
        let mut rounds_with_pulses = 0;
        for round in 0..3000 {
            let unit = [1, 2, 4, 3, 8][rng.below(5) as usize];
            let width = unit as u64 * 8;
            let count = 400;
            let (narrow, wide) = random_samples(&mut rng, unit, count);
            let chunks = random_chunks(&mut rng, count);
            let ch = rng.below(width) as usize;
            let high = rng.below(2) == 0;
            let least = 1 + rng.below(6);
            let width_of = match rng.below(3) {
                0 => TriggerDuration::Longer(least),
                1 => TriggerDuration::Shorter(least + 1 + rng.below(30)),
                _ => TriggerDuration::Between(least, least + rng.below(30)),
            };
            let level = (rng.below(2) == 0).then(|| {
                let other = (ch + 1 + rng.below(width - 1) as usize) % width as usize;
                (other, rng.below(2) == 0)
            });
            let mut t = Trigger::new();
            t.add(0, ch, if high { TriggerMatch::Falling } else { TriggerMatch::Rising });
            if let Some((c, l)) = level {
                t.add(0, c, if l { TriggerMatch::One } else { TriggerMatch::Zero });
            }
            t.set_duration(0, width_of);
            assert!(SoftTrigger::check(&t).is_ok());

            let expected = pulses_by_scan(&narrow, unit, ch, high, width_of.bounds(), level);
            rounds_with_pulses += usize::from(!expected.is_empty());
            for (data, us) in [(&narrow, unit), (&wide, 9)] {
                // Rearmed on the trigger sample, where the channel has
                // left the pulse level: no pulse is lost.
                let mut m = SoftTrigger::new(&t, us).unwrap();
                let mut fires = Vec::new();
                let mut base = 0;
                for &k in &chunks {
                    let mut from = 0;
                    while let Some(i) = m.feed(&data[(base + from) * us..(base + k) * us]) {
                        let at = base + from + i;
                        fires.push(at);
                        from += i + 1;
                        m.rearm(Some(&data[at * us..(at + 1) * us]));
                    }
                    base += k;
                }
                assert_eq!(fires, expected, "round {round}, {us} bytes: {t:?}");
            }
        }
        assert!(rounds_with_pulses > 1500, "{rounds_with_pulses}");
    }

    /// libsigrok's matcher over `data` in `chunks`: each trigger point
    /// and its pre-trigger samples' first bytes. Rearmed after each fire
    /// as fx2lafw does, so the sample it keeps as the previous one after
    /// a fire counts too.
    fn logic_fires(
        stages: &[Vec<(usize, TriggerMatch)>],
        chunks: &[usize],
        data: &[u8],
        us: usize,
    ) -> Vec<(usize, Vec<u8>)> {
        let mut stl = SoftTriggerLogic::new(stages.to_vec(), us, 4);
        let mut fires = Vec::new();
        let mut base = 0;
        for &k in chunks {
            let mut from = 0;
            while let Some((at, pre)) = stl.check(&data[(base + from) * us..(base + k) * us]) {
                let pre: Vec<u8> = pre.concat().chunks(us).map(|s| s[0]).collect();
                fires.push((base + from + at, pre));
                from += at + 1;
                stl.cur_stage = 0;
                if from == k {
                    break;
                }
            }
            base += k;
        }
        fires
    }

    #[test]
    fn words_match_like_bytes() {
        let mut rng = Rng(0x2545_F491_4F6C_DD1D);
        let (mut soft_fired, mut logic_fired) = (0, 0);
        for _ in 0..2000 {
            // Lanes of 8, 4 and 2 samples, and widths of one sample a word.
            let unit = [1, 2, 4, 3, 8][rng.below(5) as usize];
            let width = unit as u64 * 8;
            let n = 200;
            let (narrow, wide) = random_samples(&mut rng, unit, n);
            let chunks = random_chunks(&mut rng, n);

            let mut t = Trigger::new();
            let stages = 1 + rng.below(2) as usize;
            for st in 0..stages {
                for _ in 0..rng.below(3) {
                    t.add(st, rng.below(width) as usize, random_match(&mut rng));
                }
                if rng.below(3) == 0 || t.stages.len() <= st {
                    let len = 1 + rng.below(4) as usize;
                    let channels: Vec<usize> = if rng.below(2) == 0 {
                        let first = rng.below(width - len as u64 + 1) as usize;
                        (first..first + len).collect()
                    } else {
                        (0..len).map(|_| rng.below(width) as usize).collect()
                    };
                    let (a, b) = (rng.below(1 << len), rng.below(1 << len));
                    let range = TriggerRange {
                        channels,
                        low: a.min(b),
                        high: a.max(b),
                        inside: rng.below(2) == 0,
                    };
                    t.set_range(st, range);
                }
                match rng.below(5) {
                    // A pulse width: one rising or falling edge, beside
                    // the stage's levels and range.
                    4 => {
                        t.stages[st].conditions.retain(|c| !c.matches.is_edge());
                        let edge = [TriggerMatch::Rising, TriggerMatch::Falling];
                        t.add(st, rng.below(width) as usize, edge[rng.below(2) as usize]);
                        let a = 1 + rng.below(4);
                        let d = match rng.below(3) {
                            0 => TriggerDuration::Longer(a),
                            1 => TriggerDuration::Shorter(a + 1 + rng.below(20)),
                            _ => TriggerDuration::Between(a, a + rng.below(20)),
                        };
                        t.set_duration(st, d);
                    }
                    0 => t.set_count(
                        st,
                        TriggerCount {
                            count: 1 + rng.below(3) as u32,
                            consecutive: rng.below(2) == 0,
                        },
                    ),
                    1 => t.set_duration(st, TriggerDuration::Longer(rng.below(4))),
                    _ => {}
                }
            }
            if stages == 2 {
                t.combine = [TriggerCombine::Sequence, TriggerCombine::Or, TriggerCombine::And]
                    [rng.below(3) as usize];
            }

            let run_soft = |data: &[u8], us: usize| {
                let mut st = SoftTrigger::new(&t, us).unwrap();
                let mut fires = Vec::new();
                let mut base = 0;
                for &k in &chunks {
                    let mut from = 0;
                    while let Some(i) = st.feed(&data[(base + from) * us..(base + k) * us]) {
                        let at = base + from + i;
                        fires.push(at);
                        from += i + 1;
                        st.rearm(Some(&data[at * us..(at + 1) * us]));
                    }
                    base += k;
                }
                fires
            };
            let fires = run_soft(&narrow, unit);
            soft_fired += usize::from(!fires.is_empty());
            assert_eq!(fires, run_soft(&wide, 9), "{t:?}");

            // libsigrok's reads a channel beyond the sample as low, in
            // both paths: a zero there holds, anything else never does.
            let mut plain: Vec<Vec<(usize, TriggerMatch)>> = t
                .stages
                .iter()
                .map(|s| s.conditions.iter().map(|c| (c.channel, c.matches)).collect())
                .collect();
            if rng.below(4) == 0 {
                let st = rng.below(plain.len() as u64) as usize;
                let ch = width as usize + rng.below(80) as usize;
                let m = if rng.below(2) == 0 { TriggerMatch::Zero } else { random_match(&mut rng) };
                plain[st].push((ch, m));
            }
            let fires = logic_fires(&plain, &chunks, &narrow, unit);
            logic_fired += usize::from(!fires.is_empty());
            assert_eq!(fires, logic_fires(&plain, &chunks, &wide, 9), "{plain:?}");
        }
        // Enough of them fire for the comparison to mean something.
        assert!(soft_fired > 500 && logic_fired > 500, "{soft_fired} {logic_fired}");
    }
}
