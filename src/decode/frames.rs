// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The frame store: a decoder instance's output kept, indexed by start
//! sample overall and per annotation row, counted as it arrives. Moved
//! from oxrok-ui, with the raw values and the binary and meta items
//! added.

use super::radix::raw_value;
use std::collections::HashMap;
use std::sync::Arc;

/// One annotation, placed.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    /// Start sample.
    pub start: u64,
    /// End sample.
    pub end: u64,
    /// Annotation class index.
    pub class: usize,
    /// Annotation row index (see [`super::radix::class_rows`]).
    pub row: usize,
    /// Texts, longest first.
    pub texts: Arc<[String]>,
    /// The raw value and its width in bits, from the binary output.
    pub value: Option<(u64, u32)>,
    /// The error heuristic said so.
    pub error: bool,
}

impl Frame {
    /// The longest text.
    pub fn longest(&self) -> &str {
        self.texts.first().map_or("", String::as_str)
    }

    /// The shortest text.
    pub fn shortest(&self) -> &str {
        self.texts.last().map_or("", String::as_str)
    }
}

/// One binary output item.
#[derive(Clone, Debug, PartialEq)]
pub struct RawBinary {
    /// Start sample.
    pub start: u64,
    /// End sample.
    pub end: u64,
    /// Binary class index.
    pub class: usize,
    /// The bytes.
    pub bytes: Vec<u8>,
}

/// One meta output item.
#[derive(Clone, Debug, PartialEq)]
pub struct MetaItem {
    /// Start sample.
    pub start: u64,
    /// End sample.
    pub end: u64,
    /// The registered name, e.g. `Bitrate`.
    pub name: String,
    /// The value as text.
    pub text: String,
}

/// Inserts `id` keeping `v` sorted by `key`; frames mostly arrive in
/// order, so this is usually a push.
fn insert_sorted(v: &mut Vec<u32>, id: u32, key: impl Fn(u32) -> (u64, u32)) {
    let k = key(id);
    if v.last().is_none_or(|&l| key(l) <= k) {
        v.push(id);
    } else {
        let at = v.partition_point(|&x| key(x) <= k);
        v.insert(at, id);
    }
}

/// An instance's frames, binary and meta items.
#[derive(Clone, Debug, Default)]
pub struct FrameStore {
    frames: Vec<Frame>,
    /// Frame ids in (start, id) order.
    order: Vec<u32>,
    /// Per row: frame ids in (start, id) order.
    rows: Vec<Vec<u32>>,
    /// Heap bytes of the frames' texts, counted as they come.
    text_bytes: usize,
    class_counts: Vec<u64>,
    row_counts: Vec<u64>,
    errors: u64,
    /// The classes whose frames take a value from a binary item.
    value_classes: Vec<usize>,
    /// Raw values by (start, class), from the binary items.
    values: HashMap<(u64, usize), (u64, u32)>,
    /// Frames of a value class that arrived before their binary item.
    awaiting: HashMap<(u64, usize), Vec<u32>>,
    binary: Vec<RawBinary>,
    binary_bytes: usize,
    meta: Vec<MetaItem>,
}

impl FrameStore {
    /// A store whose frames of `value_classes` take raw values.
    pub fn with_value_classes(value_classes: Vec<usize>) -> Self {
        Self { value_classes, ..Self::default() }
    }

    /// How many frames.
    pub fn len(&self) -> usize {
        self.frames.len()
    }

    /// Whether there are none.
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Adds a frame; its value comes from a binary item with the same
    /// start whose class maps to the frame's, now or when it arrives.
    pub fn push(&mut self, mut f: Frame) {
        let key = (f.start, f.class);
        if f.value.is_none() && self.value_classes.contains(&f.class) {
            f.value = self.values.get(&key).copied();
        }
        // The `Arc<[String]>`: its counts and slice, plus each string.
        let n = f.texts.len();
        self.text_bytes += 16
            + n * std::mem::size_of::<String>()
            + f.texts.iter().map(String::capacity).sum::<usize>();
        let id = self.frames.len() as u32;
        let row = f.row;
        if f.class >= self.class_counts.len() {
            self.class_counts.resize(f.class + 1, 0);
        }
        self.class_counts[f.class] += 1;
        if row >= self.row_counts.len() {
            self.row_counts.resize(row + 1, 0);
        }
        self.row_counts[row] += 1;
        if f.error {
            self.errors += 1;
        }
        if f.value.is_none() && self.value_classes.contains(&f.class) {
            self.awaiting.entry(key).or_default().push(id);
        }
        self.frames.push(f);
        let frames = &self.frames;
        let key = |i: u32| (frames[i as usize].start, i);
        insert_sorted(&mut self.order, id, key);
        if self.rows.len() <= row {
            self.rows.resize_with(row + 1, Vec::new);
        }
        insert_sorted(&mut self.rows[row], id, key);
    }

    /// Adds a binary item; `target` is the annotation class its raw value
    /// belongs to, when the decoder has one.
    pub fn push_binary(&mut self, item: RawBinary, target: Option<usize>) {
        if let Some(class) = target
            && let Some(value) = raw_value(&item.bytes)
        {
            let key = (item.start, class);
            self.values.insert(key, value);
            for id in self.awaiting.remove(&key).unwrap_or_default() {
                self.frames[id as usize].value = Some(value);
            }
        }
        self.binary_bytes += item.bytes.len();
        self.binary.push(item);
    }

    /// Adds a meta item.
    pub fn push_meta(&mut self, item: MetaItem) {
        self.meta.push(item);
    }

    /// The store's heap bytes (frames, indexes, texts, binary).
    pub fn memory_bytes(&self) -> usize {
        self.frames.capacity() * std::mem::size_of::<Frame>()
            + self.order.capacity() * 4
            + self.rows.capacity() * std::mem::size_of::<Vec<u32>>()
            + self.rows.iter().map(|r| r.capacity() * 4).sum::<usize>()
            + self.text_bytes
            + self.binary.capacity() * std::mem::size_of::<RawBinary>()
            + self.binary_bytes
            + self.meta.capacity() * std::mem::size_of::<MetaItem>()
    }

    /// The frame with `id`.
    pub fn get(&self, id: u32) -> Option<&Frame> {
        self.frames.get(id as usize)
    }

    /// The k-th frame in start order.
    pub fn by_order(&self, k: usize) -> Option<(u32, &Frame)> {
        let id = *self.order.get(k)?;
        Some((id, &self.frames[id as usize]))
    }

    /// Frame ids in start order.
    pub fn order(&self) -> &[u32] {
        &self.order
    }

    /// Every frame in start order.
    pub fn iter(&self) -> impl Iterator<Item = (u32, &Frame)> + '_ {
        self.order.iter().map(|&i| (i, &self.frames[i as usize]))
    }

    /// The frames starting in `[s0, s1)`, in start order.
    pub fn in_range(&self, s0: u64, s1: u64) -> impl Iterator<Item = (u32, &Frame)> + '_ {
        let first = self.order.partition_point(|&i| self.frames[i as usize].start < s0);
        self.order[first..]
            .iter()
            .map(|&i| (i, &self.frames[i as usize]))
            .take_while(move |(_, f)| f.start < s1)
    }

    /// Frames of `row` overlapping `[s0, s1)`, in start order.
    pub fn in_row(&self, row: usize, s0: u64, s1: u64) -> impl Iterator<Item = (u32, &Frame)> + '_ {
        let ids = self.rows.get(row).map_or(&[][..], Vec::as_slice);
        // Frames in a row rarely overlap; scan back over any that still
        // reach into the window.
        let mut first = ids.partition_point(|&i| self.frames[i as usize].start < s0);
        while first > 0 && self.frames[ids[first - 1] as usize].end > s0 {
            first -= 1;
        }
        ids[first..]
            .iter()
            .map(|&i| (i, &self.frames[i as usize]))
            .take_while(move |(_, f)| f.start < s1)
            .filter(move |(_, f)| f.end > s0 || f.start >= s0)
    }

    /// The first frame of `row` starting after `s`.
    pub fn next_in_row(&self, row: usize, s: u64) -> Option<(u32, &Frame)> {
        let ids = self.rows.get(row)?;
        let k = ids.partition_point(|&i| self.frames[i as usize].start <= s);
        ids.get(k).map(|&i| (i, &self.frames[i as usize]))
    }

    /// The last frame of `row` ending before `s`.
    pub fn prev_in_row(&self, row: usize, s: u64) -> Option<(u32, &Frame)> {
        let ids = self.rows.get(row)?;
        let k = ids.partition_point(|&i| self.frames[i as usize].start < s);
        ids[..k].iter().rev().map(|&i| (i, &self.frames[i as usize])).find(|(_, f)| f.end <= s)
    }

    /// A frame of `row` containing sample `s`.
    pub fn at(&self, row: usize, s: u64) -> Option<(u32, &Frame)> {
        self.in_row(row, s, s + 1).find(|(_, f)| f.start <= s && s < f.end.max(f.start + 1))
    }

    /// The position of frame `id` in start order.
    pub fn order_index(&self, id: u32) -> Option<usize> {
        let f = self.frames.get(id as usize)?;
        let k = (f.start, id);
        let at = self.order.partition_point(|&i| (self.frames[i as usize].start, i) < k);
        (self.order.get(at) == Some(&id)).then_some(at)
    }

    /// Frames of `class` so far.
    pub fn class_count(&self, class: usize) -> u64 {
        self.class_counts.get(class).copied().unwrap_or(0)
    }

    /// Frames of `row` so far.
    pub fn row_count(&self, row: usize) -> u64 {
        self.row_counts.get(row).copied().unwrap_or(0)
    }

    /// Frames the error heuristic flagged.
    pub fn errors(&self) -> u64 {
        self.errors
    }

    /// The binary items, in arrival order.
    pub fn binary(&self) -> &[RawBinary] {
        &self.binary
    }

    /// The meta items, in arrival order.
    pub fn meta(&self) -> &[MetaItem] {
        &self.meta
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(start: u64, end: u64, row: usize, t: &str) -> Frame {
        Frame {
            start,
            end,
            class: row,
            row,
            texts: vec![t.to_owned()].into(),
            value: None,
            error: false,
        }
    }

    #[test]
    fn queries_by_row_and_order() {
        let mut s = FrameStore::default();
        s.push(f(10, 20, 0, "a"));
        s.push(f(30, 40, 0, "b"));
        s.push(f(5, 50, 1, "packet"));
        s.push(f(50, 60, 0, "c"));
        // Out of order on arrival: start order is kept.
        assert_eq!(s.order(), [2, 0, 1, 3]);
        let hits: Vec<&str> = s.in_row(0, 15, 35).map(|(_, f)| f.longest()).collect();
        assert_eq!(hits, ["a", "b"]);
        assert_eq!(s.in_row(1, 45, 46).count(), 1, "a long frame reaching into the window");
        assert_eq!(s.next_in_row(0, 30).unwrap().1.longest(), "c");
        assert_eq!(s.prev_in_row(0, 30).unwrap().1.longest(), "a");
        assert_eq!(s.at(0, 35).unwrap().1.longest(), "b");
        assert!(s.at(0, 25).is_none());
        assert_eq!(s.order_index(3), Some(3));
        assert_eq!(s.order_index(2), Some(0));
        let range: Vec<&str> = s.in_range(10, 50).map(|(_, f)| f.longest()).collect();
        assert_eq!(range, ["a", "b"]);
        assert_eq!(s.class_count(0), 3);
        assert_eq!(s.row_count(1), 1);
        assert_eq!(s.iter().count(), 4);
        assert!(s.memory_bytes() > 0);
    }

    #[test]
    fn values_attach_in_either_order() {
        let mut s = FrameStore::with_value_classes(vec![0]);
        let bin = |start, bytes: &[u8]| RawBinary {
            start,
            end: start + 8,
            class: 0,
            bytes: bytes.to_vec(),
        };
        // Binary first, then the frame.
        s.push_binary(bin(0, &[0x41]), Some(0));
        s.push(f(0, 8, 0, "41"));
        assert_eq!(s.get(0).unwrap().value, Some((0x41, 8)));
        // Frame first, then the binary.
        s.push(f(8, 16, 0, "42"));
        assert_eq!(s.get(1).unwrap().value, None);
        s.push_binary(bin(8, &[0x42]), Some(0));
        assert_eq!(s.get(1).unwrap().value, Some((0x42, 8)));
        // Another class takes nothing; a binary without a target gives nothing.
        s.push(f(16, 24, 1, "x"));
        s.push_binary(bin(16, &[0x43]), None);
        assert_eq!(s.get(2).unwrap().value, None);
        assert_eq!(s.binary().len(), 3);
        s.push_meta(MetaItem { start: 0, end: 1, name: "Bitrate".into(), text: "9600".into() });
        assert_eq!(s.meta()[0].name, "Bitrate");
        let mut e = f(24, 25, 0, "bad");
        e.error = true;
        s.push(e);
        assert_eq!(s.errors(), 1);
    }
}
