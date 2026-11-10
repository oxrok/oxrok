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
//! duration. In a sequence the next stage is armed on the samples that
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
    /// the edge alone, so its filter has only the edges.
    fn block(&self, unit: usize, edges_only: bool) -> Block {
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
    /// Events seen so far.
    hits: u32,
    /// Samples in a row the condition held.
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
}

impl SoftTrigger {
    /// What this matcher supports.
    pub const FEATURES: TriggerFeatures = TriggerFeatures {
        count: true,
        consecutive: true,
        duration: true,
        range: true,
        or: true,
        and: true,
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
            .map(|s| {
                let conds: Vec<_> = s.conditions.iter().map(|c| (c.channel, c.matches)).collect();
                let masks = Masks::new(&conds, s.range.as_ref(), unit_size);
                let consecutive = s.count.is_some_and(|c| c.consecutive);
                let block = if lanes(unit_size) > 1 {
                    masks.block(unit_size, consecutive && s.has_edge())
                } else {
                    Block::default()
                };
                (s, conds, masks, block)
            })
            .map(|(s, conds, masks, block)| Stage {
                conds,
                range: s.range.clone(),
                masks,
                block,
                has_edge: s.has_edge(),
                count: s.count.map_or(1, |c| c.count.max(1)),
                consecutive: s.count.is_some_and(|c| c.consecutive),
                duration: match s.duration {
                    Some(TriggerDuration::Longer(n)) => n,
                    None => 0,
                },
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
        })
    }

    /// Arms the matcher again, its stages from the start, with `prev`
    /// as the sample before the next one fed: an edge between them is
    /// seen. `None` forgets it, as a new matcher does.
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
        self.prev = prev.map(<[u8]>::to_vec);
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

    /// Advances stage `idx` by one sample, given what `edges_and_levels`
    /// says of it; true when it fired.
    fn step(&mut self, idx: usize, edges: bool, levels: bool) -> bool {
        let matched = edges && levels;
        let st = &mut self.stages[idx];
        let has_edge = st.has_edge;
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
    /// a stage; true when it fired.
    #[inline]
    fn advance(&mut self, eval: impl Fn(&Stage) -> (bool, bool)) -> bool {
        match self.combine {
            TriggerCombine::Sequence => {
                let (edges, levels) = eval(&self.stages[self.stage]);
                if self.step(self.stage, edges, levels) {
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
                    let (edges, levels) = eval(&self.stages[idx]);
                    if self.step(idx, edges, levels) {
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
    /// duration running. True when it skipped the block.
    fn skip_block(&mut self, now: u64, prev: u64) -> bool {
        let prev = now << (self.unit_size * 8) | prev;
        if (0..self.stages.len())
            .any(|i| self.active(i) && self.stages[i].block.may_match(now, prev))
        {
            return false;
        }
        for i in 0..self.stages.len() {
            if self.active(i) {
                self.stages[i].held = false;
                self.stages[i].run = 0;
            }
        }
        true
    }

    /// After one sample `p` of a block of samples all equal to `p` went
    /// through `advance` and left the stages where they were: whether
    /// the other `n` change nothing but the duration runs, which they
    /// then advance. They match as `p` did against itself: no edge, no
    /// new event; only a duration that completes inside them needs
    /// each sample.
    fn skip_same(&mut self, n: u64) -> bool {
        let reach = |st: &Stage| st.duration > 0 && st.run > 0 && st.run + n > st.duration;
        if (0..self.stages.len()).any(|i| self.active(i) && reach(&self.stages[i])) {
            return false;
        }
        for i in 0..self.stages.len() {
            let st = &self.stages[i];
            if self.active(i) && st.duration > 0 && st.run > 0 {
                self.stages[i].run += n;
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
            if self.advance(|st| st.masks.edges_and_levels(now, prev)) {
                return Err(i);
            }
            prev = Some(now);
        }
        Ok(prev)
    }

    /// Runs the samples in `data` through the matcher. Returns the
    /// index of the sample that completed the trigger, if it did here.
    /// After that, calls return `None`.
    pub fn feed(&mut self, data: &[u8]) -> Option<usize> {
        if self.fired {
            return None;
        }
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
                        if self.advance(|st| st.masks.edges_and_levels(p, Some(p))) {
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
                // Sample by sample up to the next block.
                let stop = next_block.max(i + 1).min(n);
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
            if self.advance(|st| Self::edges_and_levels(st, sample, before)) {
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
    /// offset of the trigger point. A restart goes back below `i` as it
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
    ) -> std::result::Result<(isize, u64), usize> {
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
                    return Err(at);
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
            // Sample by sample up to the next block.
            let stop = (next_block as isize).max(i + unit).min(len);
            match self.check_run(buf, words, i, stop, prev_word) {
                Ok((next, p)) => (i, prev_word) = (next, p),
                Err(at) => {
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

    #[test]
    fn words_match_like_bytes() {
        let mut rng = Rng(0x2545_F491_4F6C_DD1D);
        let (mut soft_fired, mut logic_fired) = (0, 0);
        for _ in 0..2000 {
            let unit = 1 + rng.below(2) as usize;
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
                match rng.below(4) {
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
                        fires.push(base + from + i);
                        from += i + 1;
                        st = SoftTrigger::new(&t, us).unwrap();
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
            let run_logic = |data: &[u8], us: usize| {
                let mut stl = SoftTriggerLogic::new(plain.clone(), us, 4);
                let mut base = 0;
                for &k in &chunks {
                    if let Some((at, pre)) = stl.check(&data[base * us..(base + k) * us]) {
                        let pre: Vec<u8> = pre.concat().chunks(us).map(|s| s[0]).collect();
                        return Some((base + at, pre));
                    }
                    base += k;
                }
                None
            };
            let fire = run_logic(&narrow, unit);
            logic_fired += usize::from(fire.is_some());
            assert_eq!(fire, run_logic(&wide, 9), "{plain:?}");
        }
        // Enough of them fire for the comparison to mean something.
        assert!(soft_fired > 500 && logic_fired > 500, "{soft_fired} {logic_fired}");
    }
}
