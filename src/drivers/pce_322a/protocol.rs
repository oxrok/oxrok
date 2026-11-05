// Counterpart in sigrok:
//   libsigrok/src/hardware/pce-322a/protocol.c,
//   libsigrok/src/hardware/pce-322a/protocol.h,
//   Copyright (C) 2016 George Hopkins <george-hopkins@null.net>,
//   Copyright (C) 2016 Matthieu Guillaumin <matthieu@guillaum.in>,
//   with commits by Gerhard Sittig, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The PCE-322A's wire: two-byte commands and what comes back.
//!
//! The line is 115200/8n1. From the host:
//!
//! | bytes         | what                               |
//! |---------------|------------------------------------|
//! | `ac ff`       | connect                            |
//! | `ca ff`       | disconnect                         |
//! | `aa f1`       | toggle dB(A), dB(C)                |
//! | `aa f2`       | next range                         |
//! | `aa f4`       | toggle fast, slow                  |
//! | `aa f8`       | power off                          |
//! | `ad da`       | ask for the memory's status        |
//! | `d3 da LL HH` | ask for block `HHLL` of the memory |
//!
//! From the meter, unasked, measurements of 13 bytes: `7f`, tenths of a dB in two
//! bytes, big endian, byte 3 with bit 0 for C and bit 1 for slow, byte 4 with the
//! range in bits 0 and 1, bit 2 for max and bit 3 for min, then what no one reads,
//! and `00`. They are found by a window that slides until both ends fit, as there
//! is nothing else to check.
//!
//! The status is 9 bytes: `d1 05 00 01 d2`, the blocks in two bytes, big endian,
//! the length of the last block, and `20`; the wiki says `10` or `20`.
//!
//! The memory is a run of recordings. Each has a header of 11 bytes, `7f f7` or
//! `ff ff` (the driver looks at the bits `7f` and `f7` only), `01 00`, `0a` for
//! dB(A) and `0c` for dB(C), and six bytes of date, followed by its samples, two
//! bytes each, little endian tenths.

use crate::core::MqFlags;

/// The line.
pub const SERIALCOMM: &str = "115200/8n1";
/// Tells the meter a host is there.
pub const CONNECT: [u8; 2] = [0xac, 0xff];
/// Tells the meter the host has left, which stops the stream.
pub const DISCONNECT: [u8; 2] = [0xca, 0xff];
/// Toggles dB(A) and dB(C).
pub const TOGGLE_WEIGHT_FREQ: [u8; 2] = [0xaa, 0xf1];
/// Steps to the next range.
pub const TOGGLE_MEAS_RANGE: [u8; 2] = [0xaa, 0xf2];
/// Toggles fast and slow.
pub const TOGGLE_WEIGHT_TIME: [u8; 2] = [0xaa, 0xf4];
/// Turns the meter off.
pub const POWER_OFF: [u8; 2] = [0xaa, 0xf8];
/// Asks how much of the memory is used.
pub const MEMORY_STATUS: [u8; 2] = [0xad, 0xda];

const FRAME_LEN: usize = 13;
const STATUS_LEN: usize = 9;
const HEADER_LEN: usize = 11;

/// The ranges, by the number the meter gives them.
pub const RANGES: [(u64, u64); 4] = [(30, 130), (30, 80), (50, 100), (80, 130)];

/// The request for a block: its number goes out low byte first.
#[must_use]
pub fn memory_block(block: u16) -> [u8; 4] {
    let [low, high] = block.to_le_bytes();
    [0xd3, 0xda, low, high]
}

/// How many times the range must step, upward and round, from one to the other.
#[must_use]
pub fn range_steps(from: u8, to: u8) -> u8 {
    to.wrapping_sub(from) & 3
}

/// What a value's two bytes mean: tenths of a dB, in the float C sends.
#[must_use]
pub fn tenths(value: u16) -> f32 {
    (f64::from(value) / 10.0) as f32
}

/// How the meter is set, as far as anyone knows.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct Mode {
    /// dB(C) rather than dB(A).
    pub c_weight: bool,
    /// Slow rather than fast.
    pub slow: bool,
    /// The maximum is shown.
    pub max: bool,
    /// The minimum is shown.
    pub min: bool,
    /// Index into [`RANGES`].
    pub range: u8,
}

impl Mode {
    /// The flags of a reading.
    #[must_use]
    pub fn flags(self) -> MqFlags {
        let mut flags =
            if self.c_weight { MqFlags::SPL_FREQ_WEIGHT_C } else { MqFlags::SPL_FREQ_WEIGHT_A }
                | if self.slow { MqFlags::SPL_TIME_WEIGHT_S } else { MqFlags::SPL_TIME_WEIGHT_F };
        if self.max {
            flags |= MqFlags::MAX;
        }
        if self.min {
            flags |= MqFlags::MIN;
        }
        flags
    }
}

/// A window of the last `N` bytes up to `N`.
#[derive(Clone, Debug)]
struct Window<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

impl<const N: usize> Window<N> {
    const fn new() -> Self {
        Window { bytes: [0; N], len: 0 }
    }

    fn push(&mut self, byte: u8) {
        if self.len < N {
            self.bytes[self.len] = byte;
            self.len += 1;
        } else {
            self.bytes.copy_within(1.., 0);
            self.bytes[N - 1] = byte;
        }
    }

    fn full(&self) -> bool {
        self.len == N
    }

    /// The last two bytes, of which there must be two.
    fn last_two(&self) -> (u8, u8) {
        (self.bytes[self.len - 2], self.bytes[self.len - 1])
    }

    fn clear(&mut self) {
        self.len = 0;
    }
}

/// A measurement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Measurement {
    /// Tenths of a dB.
    pub value: u16,
    /// How the meter says it is set.
    pub mode: Mode,
}

/// Finds measurements in the stream.
#[derive(Clone, Debug)]
pub struct Stream(Window<FRAME_LEN>);

impl Default for Stream {
    fn default() -> Self {
        Stream(Window::new())
    }
}

impl Stream {
    /// One byte more, and a measurement if that completes one.
    pub fn feed(&mut self, byte: u8) -> Option<Measurement> {
        self.0.push(byte);
        let b = &self.0.bytes;
        if !self.0.full() || b[0] != 0x7f || b[FRAME_LEN - 1] != 0 {
            return None;
        }
        let measurement = Measurement {
            value: u16::from_be_bytes([b[1], b[2]]),
            mode: Mode {
                c_weight: b[3] & 1 != 0,
                slow: b[3] & 2 != 0,
                range: b[4] & 3,
                // Max goes first if both are set.
                max: b[4] & 4 != 0,
                min: b[4] & 4 == 0 && b[4] & 8 != 0,
            },
        };
        self.0.clear();
        Some(measurement)
    }
}

/// The memory's status.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    /// Blocks used.
    pub used: u16,
    /// The length of the last block.
    pub last: u8,
}

/// Finds the status in what comes.
#[derive(Clone, Debug)]
pub struct StatusWindow(Window<STATUS_LEN>);

impl Default for StatusWindow {
    fn default() -> Self {
        StatusWindow(Window::new())
    }
}

impl StatusWindow {
    /// One byte more, and the status if that completes it. The last byte may be
    /// `20`, all C takes, or the wiki's other value, `10`.
    pub fn feed(&mut self, byte: u8) -> Option<Status> {
        self.0.push(byte);
        let b = &self.0.bytes;
        if !self.0.full()
            || b[..5] != [0xd1, 0x05, 0x00, 0x01, 0xd2]
            || !matches!(b[STATUS_LEN - 1], 0x10 | 0x20)
        {
            return None;
        }
        let status = Status { used: u16::from_be_bytes([b[5], b[6]]), last: b[7] };
        self.0.clear();
        Some(status)
    }
}

/// What a recording's header says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Recording {
    /// `0a` or `0c`, which print as the weighting's letter.
    pub weight: u8,
    /// Year, month, day, hour with two more bits, minute and second.
    pub date: [u8; 6],
}

impl Recording {
    /// `0c` is dB(C), and anything else is dB(A).
    #[must_use]
    pub fn c_weight(&self) -> bool {
        self.weight == 0x0c
    }

    /// What is logged of it, in C's words.
    #[must_use]
    pub fn describe(&self) -> String {
        let [y, mo, d, h, mi, s] = self.date;
        format!(
            "Recording dB({:X}) {y:02x}/{mo:02x}/{d:02x} {:02x}:{mi:02x}:{s:02x} ",
            self.weight,
            h & 0x3f
        )
    }
}

/// What one byte of memory gave.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Item {
    /// Nothing whole yet.
    Nothing,
    /// A recording starts.
    Recording(Recording),
    /// A sample in tenths of a dB.
    Sample(u16),
}

/// Reads the memory, a byte at a time, and says when a block is over.
#[derive(Clone, Debug)]
pub struct Memory {
    window: Window<HEADER_LEN>,
    /// Bytes to take with no look at them.
    skip: usize,
    /// Bytes of this block so far, which goes back to 0 at 256.
    cursor: u8,
    /// Blocks done.
    block: u16,
}

impl Default for Memory {
    fn default() -> Self {
        Memory { window: Window::new(), skip: 1, cursor: 0, block: 0 }
    }
}

impl Memory {
    /// The block asked for next.
    #[must_use]
    pub fn block(&self) -> u16 {
        self.block
    }

    /// Whether what the status said has all come.
    #[must_use]
    pub fn done(&self, status: Status) -> bool {
        self.block >= status.used && self.cursor >= status.last
    }

    /// How many bytes C logs at the end: one block less than it read, and
    /// -256 for an empty memory. It is only a log line, kept as it is.
    #[must_use]
    pub fn logged_size(&self) -> i64 {
        256 * (i64::from(self.block) - 1) + i64::from(self.cursor)
    }

    /// Whether the last byte fed ended a block.
    #[must_use]
    pub fn at_block_start(&self) -> bool {
        self.cursor == 0
    }

    /// One byte more.
    pub fn feed(&mut self, byte: u8) -> Item {
        self.window.push(byte);
        if self.skip == 0 {
            let (a, b) = self.window.last_two();
            if a & 0x7f == 0x7f && b & 0xf7 == 0xf7 {
                self.skip = HEADER_LEN - 2;
            }
        }
        let mut item = Item::Nothing;
        if self.skip == 0 && self.window.full() {
            let w = &self.window.bytes;
            if w[0] & 0x7f == 0x7f && w[1] & 0xf7 == 0xf7 && w[2] == 0x01 && w[3] == 0x00 {
                let mut date = [0; 6];
                date.copy_from_slice(&w[5..11]);
                item = Item::Recording(Recording { weight: w[4], date });
                self.skip = 2;
            }
        }
        if self.skip == 0 {
            let (low, high) = self.window.last_two();
            item = Item::Sample(u16::from_le_bytes([low, high]));
            self.skip = 1;
        } else {
            self.skip -= 1;
        }
        self.cursor = self.cursor.wrapping_add(1);
        if self.cursor == 0 {
            self.block = self.block.wrapping_add(1);
        }
        item
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FRAME: [u8; 13] =
        [0x7f, 0x02, 0x8e, 0x00, 0x00, 0x00, 0x26, 0x11, 0x05, 0x12, 0x30, 0x00, 0x00];
    const HEADER_A: [u8; 11] = [0x7f, 0xf7, 0x01, 0x00, 0x0a, 0x26, 0x11, 0x05, 0x12, 0x00, 0x30];

    fn all(stream: &mut Stream, bytes: &[u8]) -> Vec<Measurement> {
        bytes.iter().filter_map(|b| stream.feed(*b)).collect()
    }

    fn items(memory: &mut Memory, bytes: &[u8]) -> Vec<Item> {
        bytes.iter().map(|b| memory.feed(*b)).filter(|i| *i != Item::Nothing).collect()
    }

    #[test]
    fn commands_are_the_wikis() {
        assert_eq!(CONNECT, [0xac, 0xff]);
        assert_eq!(DISCONNECT, [0xca, 0xff]);
        assert_eq!(TOGGLE_WEIGHT_FREQ, [0xaa, 0xf1]);
        assert_eq!(TOGGLE_MEAS_RANGE, [0xaa, 0xf2]);
        assert_eq!(TOGGLE_WEIGHT_TIME, [0xaa, 0xf4]);
        assert_eq!(POWER_OFF, [0xaa, 0xf8]);
        assert_eq!(MEMORY_STATUS, [0xad, 0xda]);
        assert_eq!(memory_block(0x0102), [0xd3, 0xda, 0x02, 0x01]);
    }

    #[test]
    fn a_measurements_fields() {
        let mut s = Stream::default();
        let m = all(&mut s, &FRAME);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].value, 654);
        assert_eq!(m[0].mode, Mode::default());
        let mut f = FRAME;
        f[3] = 0x03;
        f[4] = 0x07;
        assert_eq!(
            all(&mut s, &f)[0].mode,
            Mode { c_weight: true, slow: true, max: true, min: false, range: 3 }
        );
        f[3] = 0x02;
        f[4] = 0x08;
        assert_eq!(
            all(&mut s, &f)[0].mode,
            Mode { c_weight: false, slow: true, max: false, min: true, range: 0 }
        );
        f[4] = 0x0c;
        let m = all(&mut s, &f)[0].mode;
        assert!(m.max && !m.min, "max goes first");
        f[3] = 0xfc;
        f[4] = 0xf0;
        assert_eq!(all(&mut s, &f)[0].mode, Mode::default(), "the other bits mean nothing");
    }

    #[test]
    fn the_window_finds_a_frame_behind_anything() {
        let mut s = Stream::default();
        assert_eq!(all(&mut s, &[0x55, 0xaa, 0x00]).len(), 0);
        assert_eq!(all(&mut s, &FRAME).len(), 1);
        // No end: the next 7f takes over only once the window has slid to it.
        let mut bad = FRAME;
        bad[12] = 0x01;
        assert_eq!(all(&mut s, &bad).len(), 0);
        assert_eq!(all(&mut s, &FRAME).len(), 1);
    }

    #[test]
    fn a_window_clears_after_a_frame() {
        let mut s = Stream::default();
        let two: Vec<u8> = FRAME.iter().chain(&FRAME).copied().collect();
        assert_eq!(all(&mut s, &two).len(), 2);
        // Twelve bytes short of one, then a 00: no frame out of the last one.
        assert_eq!(all(&mut s, &FRAME[..12]).len(), 0);
        assert_eq!(all(&mut s, &[0x00]).len(), 1);
        assert_eq!(all(&mut s, &[0x00]).len(), 0);
    }

    #[test]
    fn what_a_frame_held_is_no_part_of_the_next() {
        // A value of 7f00 and then a 00: the old frame's bytes from the 7f on would
        // pass for a whole frame if the window kept them.
        let mut s = Stream::default();
        let mut f = FRAME;
        f[1] = 0x7f;
        f[2] = 0x00;
        assert_eq!(all(&mut s, &f).len(), 1);
        assert_eq!(all(&mut s, &[0x00]).len(), 0);
    }

    #[test]
    fn tenths_are_the_float_c_sends() {
        assert_eq!(tenths(654), 65.4_f32);
        assert_eq!(tenths(0), 0.0);
        assert_eq!(tenths(0xffff), 6553.5);
    }

    #[test]
    fn flags_say_what_the_mode_says() {
        let flags = Mode { c_weight: true, slow: true, max: true, min: false, range: 2 }.flags();
        assert!(flags.contains(MqFlags::SPL_FREQ_WEIGHT_C));
        assert!(flags.contains(MqFlags::SPL_TIME_WEIGHT_S));
        assert!(flags.contains(MqFlags::MAX));
        assert!(!flags.contains(MqFlags::SPL_FREQ_WEIGHT_A));
        assert!(!flags.contains(MqFlags::SPL_TIME_WEIGHT_F));
        assert!(!flags.contains(MqFlags::MIN));
        let flags = Mode { min: true, ..Mode::default() }.flags();
        assert!(flags.contains(MqFlags::SPL_FREQ_WEIGHT_A));
        assert!(flags.contains(MqFlags::SPL_TIME_WEIGHT_F));
        assert!(flags.contains(MqFlags::MIN));
    }

    #[test]
    fn range_steps_go_up_and_round() {
        assert_eq!(range_steps(0, 0), 0);
        assert_eq!(range_steps(0, 1), 1);
        assert_eq!(range_steps(0, 3), 3);
        assert_eq!(range_steps(3, 0), 1);
        assert_eq!(range_steps(2, 1), 3);
    }

    #[test]
    fn the_status_and_its_two_endings() {
        let mut w = StatusWindow::default();
        let mut found = None;
        for b in [0x55, 0xd1, 0x05, 0x00, 0x01, 0xd2, 0x01, 0x02, 0x36, 0x20] {
            found = w.feed(b).or(found);
        }
        assert_eq!(found, Some(Status { used: 0x0102, last: 0x36 }));
        let mut found = None;
        for b in [0xd1, 0x05, 0x00, 0x01, 0xd2, 0x00, 0x00, 0x00, 0x10] {
            found = w.feed(b).or(found);
        }
        assert_eq!(found, Some(Status { used: 0, last: 0 }));
    }

    #[test]
    fn a_wrong_status_is_passed_over() {
        for bad in [
            [0xd0, 0x05, 0x00, 0x01, 0xd2, 0, 0, 0, 0x20],
            [0xd1, 0x04, 0x00, 0x01, 0xd2, 0, 0, 0, 0x20],
            [0xd1, 0x05, 0x01, 0x01, 0xd2, 0, 0, 0, 0x20],
            [0xd1, 0x05, 0x00, 0x00, 0xd2, 0, 0, 0, 0x20],
            [0xd1, 0x05, 0x00, 0x01, 0xd3, 0, 0, 0, 0x20],
            [0xd1, 0x05, 0x00, 0x01, 0xd2, 0, 0, 0, 0x21],
        ] {
            let mut w = StatusWindow::default();
            assert!(bad.iter().all(|b| w.feed(*b).is_none()), "{bad:02x?}");
        }
    }

    #[test]
    fn a_recording_then_little_endian_samples() {
        let mut m = Memory::default();
        let mut bytes = HEADER_A.to_vec();
        bytes.extend_from_slice(&[0x8e, 0x02, 0x8f, 0x02]);
        let items = items(&mut m, &bytes);
        assert_eq!(
            items,
            [
                Item::Recording(Recording {
                    weight: 0x0a,
                    date: [0x26, 0x11, 0x05, 0x12, 0x00, 0x30]
                }),
                Item::Sample(0x028e),
                Item::Sample(0x028f),
            ]
        );
    }

    #[test]
    fn the_headers_kinds() {
        let mut bytes = HEADER_A;
        bytes[0] = 0xff;
        bytes[1] = 0xff;
        bytes[4] = 0x0c;
        let mut m = Memory::default();
        let got = items(&mut m, &bytes);
        let [Item::Recording(r)] = got[..] else { panic!("{got:?}") };
        assert!(r.c_weight());
        assert!(!Recording { weight: 0x0b, date: [0; 6] }.c_weight());
        assert!(!Recording { weight: 0x0a, date: [0; 6] }.c_weight());
    }

    #[test]
    fn a_recordings_words_are_cs() {
        let r = Recording { weight: 0x0c, date: [0x26, 0x11, 0x05, 0xd2, 0x00, 0x30] };
        assert_eq!(r.describe(), "Recording dB(C) 26/11/05 12:00:30 ");
    }

    #[test]
    fn what_looks_like_a_header_and_is_not_gives_its_last_two_bytes() {
        let mut bytes = HEADER_A;
        bytes[2] = 0x02;
        let mut m = Memory::default();
        let got = items(&mut m, &bytes);
        assert_eq!(got, [Item::Sample(u16::from_le_bytes([0x00, 0x30]))]);
    }

    #[test]
    fn a_header_wants_01_and_00_both() {
        for at in [2, 3] {
            let mut bytes = HEADER_A;
            bytes[at] = 0x55;
            let mut m = Memory::default();
            assert_eq!(
                items(&mut m, &bytes),
                [Item::Sample(u16::from_le_bytes([0x00, 0x30]))],
                "byte {at}"
            );
        }
    }

    #[test]
    fn a_cursor_counts_blocks_and_done_comes_at_the_last_bytes_length() {
        let mut m = Memory::default();
        let status = Status { used: 1, last: 2 };
        assert!(!m.done(status));
        for _ in 0..255 {
            m.feed(0);
            assert!(!m.at_block_start());
        }
        assert_eq!(m.block(), 0);
        m.feed(0);
        assert!(m.at_block_start());
        assert_eq!(m.block(), 1);
        assert!(!m.done(status));
        m.feed(0);
        assert!(!m.done(status));
        m.feed(0);
        assert!(m.done(status));
        assert_eq!(m.logged_size(), 2);
    }

    #[test]
    fn an_empty_memory_is_done_at_once_and_logs_as_c() {
        let m = Memory::default();
        assert!(m.done(Status { used: 0, last: 0 }));
        assert_eq!(m.logged_size(), -256);
    }

    #[test]
    fn a_full_count_of_blocks_does_not_panic() {
        let mut m = Memory { block: 0xffff, ..Memory::default() };
        m.cursor = 255;
        m.feed(0);
        assert_eq!(m.block(), 0);
        assert!(m.at_block_start());
    }
}
