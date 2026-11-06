// Counterpart in sigrok:
//   libsigrok/src/hardware/appa-55ii/protocol.c,
//   libsigrok/src/hardware/appa-55ii/protocol.h,
//   Copyright (C) 2013 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, Bert Vermeulen, Daniel Elstner,
//   Lars-Peter Clausen, Martin Ling
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The frames of the APPA 55II, as pure functions and a parser with no port.
//!
//! Every frame is `55 55 TYPE SIZE CONTENT SUM`, the sum being that of all the
//! bytes before it, modulo 256, and `SIZE` at most 32. Facts about the content
//! are from `protocol.c` and the sigrok wiki's page on the device:
//!
//! - `00`, live: at 13 the display's mode, at 14 and 17 each probe's signed
//!   little endian value and its flags, bit 0 for tenths, bits 5 and 6 for no
//!   value. The mode's high nibble `2` is hold, and bits 2-3 choose max, min or
//!   average.
//! - `18` starts a log and `19` ends it; `11` has its count of records at 0.
//! - `14` holds a piece of the log's memory, which is records of 20 bytes: the
//!   time at 2-4, each probe at 12 and 14 in tenths, `0x7fff` for none.

use crate::core::MqFlags;

/// The most a frame takes: `APPA_55II_BUF_SIZE`.
pub const BUF_SIZE: usize = 4 + MAX_CONTENT + 1;
/// The largest content a frame may say it has.
pub const MAX_CONTENT: usize = 32;
/// `log_buf`.
pub const LOG_BUF_SIZE: usize = 64;
/// Bytes of a record in the log.
pub const RECORD: usize = 20;
/// Bytes a live frame's probes need.
pub const LIVE_CONTENT: usize = 20;

const LIVE_DATA: u8 = 0x00;
const LOG_METADATA: u8 = 0x11;
const LOG_DATA: u8 = 0x14;
const LOG_START: u8 = 0x18;
const LOG_END: u8 = 0x19;

/// A frame's bytes before its sum are `frame[..4 + size]`: whether the sum after
/// them is right. False when the sum has not come: `appa_55ii_checksum` would read
/// a byte no one has received.
fn checksum_ok(frame: &[u8]) -> bool {
    let size = 4 + usize::from(frame[3]);
    let Some(sum) = frame.get(size) else {
        return false;
    };
    *sum == frame[..size].iter().fold(0u8, |s, b| s.wrapping_add(*b))
}

/// `appa_55ii_packet_valid`, on the bytes that have come. A scan tries it on 25
/// bytes, so a frame longer than that fails here, where C reads the rest from a
/// buffer it did not fill and fails unless that byte is luck.
pub fn packet_valid(buf: &[u8]) -> bool {
    buf.len() >= 4
        && buf[0] == 0x55
        && buf[1] == 0x55
        && usize::from(buf[3]) <= MAX_CONTENT
        && checksum_ok(buf)
}

/// `appa_55ii_flags`.
fn live_flags(mode: u8) -> MqFlags {
    let mut flags = MqFlags::empty();
    if mode & 0xf0 == 0x20 {
        flags |= MqFlags::HOLD;
    }
    match mode & 0x0c {
        0x04 => flags |= MqFlags::MAX,
        0x08 => flags |= MqFlags::MIN,
        0x0c => flags |= MqFlags::AVG,
        _ => {}
    }
    flags
}

/// `appa_55ii_temp`: a probe's value and its digits, from its three bytes.
fn live_temp(probe: &[u8]) -> (f32, i32) {
    let temp = i16::from_le_bytes([probe[0], probe[1]]);
    let flags = probe[2];
    if flags & 0x60 != 0 {
        (f32::INFINITY, 0)
    } else if flags & 1 != 0 {
        (f32::from(temp) / 10.0, 1)
    } else {
        (f32::from(temp), 0)
    }
}

/// A record's probe: tenths of a degree.
fn log_temp(bytes: &[u8]) -> f32 {
    let temp = i16::from_le_bytes([bytes[0], bytes[1]]);
    if temp == 0x7fff { f32::INFINITY } else { f32::from(temp) / 10.0 }
}

/// What the frames in a read amount to.
#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    /// A live frame.
    Live {
        /// Each probe's value and digits.
        probes: [(f32, i32); 2],
        /// The display's mode.
        flags: MqFlags,
    },
    /// A record of the log.
    Record {
        /// Each probe's value.
        probes: [f32; 2],
        /// Hour, minute and second.
        time: [u8; 3],
    },
    /// The log's end, in memory mode.
    LogEnd,
}

/// The receive side of `dev_context`, and what `appa_55ii_receive_data` does with
/// a read.
#[derive(Clone, Debug, Default)]
pub struct Parser {
    buf: Vec<u8>,
    log_buf: Vec<u8>,
    records_due: u32,
    /// This log's metadata has come.
    metadata_seen: bool,
    /// The log's bytes with no metadata have been warned of.
    told_no_metadata: bool,
}

impl Parser {
    /// How much a read may bring: what the buffer has room for.
    pub fn room(&self) -> usize {
        BUF_SIZE - self.buf.len()
    }

    /// Takes a read's bytes and adds what their frames amount to. `memory` says
    /// the capture wants the log; frames of the other kind are passed over. True
    /// when the buffer filled with no frame in it, which only a frame's start with
    /// a size over 32 can do: C ends the capture there and so does this.
    pub fn feed(&mut self, data: &[u8], memory: bool, out: &mut Vec<Event>) -> bool {
        self.buf.extend_from_slice(data);
        let mut at = 0;
        while let Some(next) = self.parse_at(at, memory, out) {
            at = next;
        }
        self.buf.drain(..at);
        if self.buf.len() >= BUF_SIZE {
            self.buf.clear();
            return true;
        }
        false
    }

    /// `appa_55ii_parse_data`: where the next try starts, or `None` for more bytes.
    fn parse_at(&mut self, at: usize, memory: bool, out: &mut Vec<Event>) -> Option<usize> {
        let buf = &self.buf[at..];
        if buf.len() < 5 {
            return None;
        }
        if buf[0] != 0x55 || buf[1] != 0x55 {
            self.out_of_step(memory);
            return Some(at + 1);
        }
        let size = usize::from(buf[3]);
        if buf.len() < 5 + size {
            return None;
        }
        let next = at + 4 + size + 1;
        if !checksum_ok(buf) {
            self.out_of_step(memory);
            return Some(next);
        }
        let kind = buf[2];
        let content: Vec<u8> = buf[4..4 + size].to_vec();
        match kind {
            LIVE_DATA => Self::live(&content, memory, out),
            LOG_METADATA => self.metadata(&content),
            LOG_DATA => {
                if memory {
                    self.log_data(&content, out);
                }
            }
            LOG_START => {
                self.metadata_seen = false;
                self.told_no_metadata = false;
            }
            LOG_END => {
                if memory {
                    if self.records_due > 0 {
                        log::warn!("The log ended {} records short.", self.records_due);
                    }
                    out.push(Event::LogEnd);
                }
            }
            other => log::warn!("Invalid packet type: 0x{other:02x}."),
        }
        Some(next)
    }

    /// A frame with a wrong sum, or bytes that start no frame, while a log comes:
    /// what follows may be out of step with the records. C goes on, and every later
    /// record is read from the wrong bytes. A piece that went without a trace shows
    /// only at the end, as a log short of its count.
    fn out_of_step(&mut self, memory: bool) {
        if memory && self.records_due > 0 {
            log::warn!(
                "A log frame came broken: the {} records left are dropped.",
                self.records_due
            );
            self.records_due = 0;
            self.log_buf.clear();
        }
    }

    fn live(content: &[u8], memory: bool, out: &mut Vec<Event>) {
        if memory {
            return;
        }
        // C reads on into what the buffer held before: the last frame's readings
        // or the next one's bytes, as a sample.
        if content.len() < LIVE_CONTENT {
            log::warn!("A live packet of {} bytes has no readings.", content.len());
            return;
        }
        out.push(Event::Live {
            probes: [live_temp(&content[14..17]), live_temp(&content[17..20])],
            flags: live_flags(content[13]),
        });
    }

    fn metadata(&mut self, content: &[u8]) {
        if let [low, high, ..] = *content {
            self.records_due = u32::from(u16::from_le_bytes([low, high]));
            self.metadata_seen = true;
        } else {
            // C reads the sum and the next byte as the count.
            log::warn!("A log metadata packet of {} bytes has no count.", content.len());
        }
    }

    /// `appa_55ii_log_data`. With no records due, C keeps what comes up to its
    /// buffer's size and then loops for ever on no room: such bytes go here. The
    /// padding of a log's last piece is such bytes.
    fn log_data(&mut self, mut content: &[u8], out: &mut Vec<Event>) {
        while !content.is_empty() {
            if self.records_due == 0 {
                if !self.metadata_seen && !self.told_no_metadata {
                    log::warn!("A log with no metadata ahead of it is dropped.");
                    self.told_no_metadata = true;
                }
                log::debug!("{} bytes of log with no record due.", content.len());
                self.log_buf.clear();
                return;
            }
            let take = content.len().min(LOG_BUF_SIZE - self.log_buf.len());
            self.log_buf.extend_from_slice(&content[..take]);
            content = &content[take..];
            self.parse_records(out);
        }
    }

    /// `appa_55ii_log_data_parse`.
    fn parse_records(&mut self, out: &mut Vec<Event>) {
        let mut at = 0;
        while self.log_buf.len() - at >= RECORD && self.records_due > 0 {
            let r = &self.log_buf[at..at + RECORD];
            out.push(Event::Record {
                probes: [log_temp(&r[12..14]), log_temp(&r[14..16])],
                time: [r[2], r[3], r[4]],
            });
            at += RECORD;
            self.records_due -= 1;
        }
        self.log_buf.drain(..at);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A whole frame around `content`.
    pub(crate) fn frame(kind: u8, content: &[u8]) -> Vec<u8> {
        let mut out = vec![0x55, 0x55, kind, content.len() as u8];
        out.extend_from_slice(content);
        out.push(out.iter().fold(0u8, |s, b| s.wrapping_add(*b)));
        out
    }

    fn live_content(t1: i16, f1: u8, t2: i16, f2: u8, mode: u8) -> [u8; 20] {
        let mut c = [0u8; 20];
        c[13] = mode;
        c[14..16].copy_from_slice(&t1.to_le_bytes());
        c[16] = f1;
        c[17..19].copy_from_slice(&t2.to_le_bytes());
        c[19] = f2;
        c
    }

    fn feed(p: &mut Parser, data: &[u8], memory: bool) -> Vec<Event> {
        let mut out = Vec::new();
        assert!(!p.feed(data, memory, &mut out));
        out
    }

    fn record(t1: i16, t2: i16) -> [u8; 20] {
        let mut r = [0u8; 20];
        r[2..5].copy_from_slice(&[12, 34, 56]);
        r[12..14].copy_from_slice(&t1.to_le_bytes());
        r[14..16].copy_from_slice(&t2.to_le_bytes());
        r
    }

    #[test]
    fn a_frame_is_valid_by_its_sync_size_and_sum() {
        let f = frame(0x19, &[]);
        assert_eq!(f, [0x55, 0x55, 0x19, 0, 0xc3]);
        assert!(packet_valid(&f));
        for i in 0..f.len() {
            let mut bad = f.clone();
            bad[i] ^= 0x01;
            assert!(!packet_valid(&bad), "byte {i}");
        }
    }

    #[test]
    fn a_size_of_32_is_valid_and_33_is_not() {
        assert!(packet_valid(&frame(0x14, &[7; 32])));
        assert!(!packet_valid(&frame(0x14, &[7; 33])));
    }

    #[test]
    fn a_sum_that_has_not_come_is_not_valid() {
        let f = frame(0x14, &[7; 32]);
        assert!(!packet_valid(&f[..25]));
        assert!(!packet_valid(&f[..3]));
        assert!(!packet_valid(&[]));
    }

    #[test]
    fn a_live_frame_gives_both_probes_in_tenths_or_whole() {
        let mut p = Parser::default();
        let f = frame(0, &live_content(234, 0x05, -56, 0x04, 0x01));
        let e = feed(&mut p, &f, false);
        assert_eq!(e, [Event::Live { probes: [(23.4, 1), (-56.0, 0)], flags: MqFlags::empty() }]);
    }

    #[test]
    fn flags_of_no_value_win_over_tenths() {
        for (flag, what) in [(0x20, "no probe"), (0x40, "booting"), (0x60, "both"), (0x25, "+")] {
            assert_eq!(live_temp(&[1, 0, flag]), (f32::INFINITY, 0), "{what}");
        }
        assert_eq!(live_temp(&[0, 0x80, 0x01]), (-3276.8, 1));
        assert_eq!(live_temp(&[0xff, 0x7f, 0x01]), (3276.7, 1), "only a log has 0x7fff for none");
        assert_eq!(live_temp(&[1, 0, 0x90]), (1.0, 0), "other bits say nothing");
    }

    #[test]
    fn the_mode_gives_the_flags() {
        for (mode, flags) in [
            (0x00, MqFlags::empty()),
            (0x01, MqFlags::empty()),
            (0x21, MqFlags::HOLD),
            (0x05, MqFlags::MAX),
            (0x09, MqFlags::MIN),
            (0x0d, MqFlags::AVG),
            (0x2d, MqFlags::HOLD | MqFlags::AVG),
            (0x51, MqFlags::empty()),
            (0x30, MqFlags::empty()),
            (0xa0, MqFlags::empty()),
        ] {
            assert_eq!(live_flags(mode), flags, "{mode:#x}");
        }
    }

    #[test]
    fn a_frame_may_come_in_any_pieces() {
        let f = frame(0, &live_content(10, 5, 20, 5, 1));
        let mut p = Parser::default();
        let mut got = Vec::new();
        for b in &f {
            got.extend(feed(&mut p, &[*b], false));
        }
        assert_eq!(got.len(), 1);
        assert_eq!(p.room(), BUF_SIZE, "nothing kept");
    }

    #[test]
    fn noise_ahead_of_a_frame_is_passed_over_one_byte_at_a_time() {
        let mut p = Parser::default();
        let mut data = vec![0x3f, 0x55, 0x3f, 0x3f];
        data.extend(frame(0, &live_content(10, 5, 20, 5, 1)));
        let (a, b) = data.split_at(BUF_SIZE.min(data.len()));
        let mut got = feed(&mut p, a, false);
        got.extend(feed(&mut p, b, false));
        assert_eq!(got.len(), 1);
    }

    #[test]
    fn a_stray_sync_byte_right_ahead_of_a_frame_costs_that_frame() {
        // 55 55 55 00 ...: a false frame of type 55 and size 0, whose sum fails, and
        // the skip takes the true frame's start with it. As C does.
        let f = frame(0, &live_content(10, 5, 20, 5, 1));
        let mut data = vec![0x55];
        data.extend(&f);
        let mut p = Parser::default();
        let (a, b) = data.split_at(BUF_SIZE.min(data.len()));
        let mut got = feed(&mut p, a, false);
        got.extend(feed(&mut p, b, false));
        assert_eq!(got, []);
        assert_eq!(feed(&mut p, &f, false).len(), 1, "the next is found");
    }

    #[test]
    fn a_wrong_sum_skips_the_whole_frame() {
        let mut f = frame(0, &live_content(10, 5, 20, 5, 1));
        *f.last_mut().unwrap() ^= 1;
        let mut p = Parser::default();
        assert_eq!(feed(&mut p, &f, false), []);
        assert_eq!(p.room(), BUF_SIZE);
    }

    #[test]
    fn a_size_over_32_fills_the_buffer_and_says_so() {
        let mut p = Parser::default();
        let mut out = Vec::new();
        assert!(!p.feed(&[0x55, 0x55, 0, 33], false, &mut out));
        assert!(!p.feed(&[0; 32], false, &mut out), "36 bytes of 37");
        assert!(p.feed(&[0], false, &mut out), "37");
        assert_eq!(out, []);
        assert_eq!(p.room(), BUF_SIZE, "wiped");
    }

    #[test]
    fn a_frame_of_32_fits_the_buffer_exactly() {
        let mut p = Parser::default();
        let mut out = Vec::new();
        assert!(!p.feed(&frame(0x18, &[0; 32]), false, &mut out));
        assert_eq!(p.room(), BUF_SIZE);
    }

    #[test]
    fn a_short_live_frame_is_no_sample() {
        let mut p = Parser::default();
        assert_eq!(feed(&mut p, &frame(0, &[1, 2]), false), []);
        assert_eq!(feed(&mut p, &frame(0, &[0; 19]), false), []);
        assert_eq!(feed(&mut p, &frame(0, &[0; 20]), false).len(), 1);
    }

    #[test]
    fn each_mode_takes_its_own_frames() {
        let live = frame(0, &live_content(10, 5, 20, 5, 1));
        let meta = frame(0x11, &[1, 0, 0, 0, 0, 0, 0, 0]);
        let piece = frame(0x14, &record(100, 200));
        let end = frame(0x19, &[]);
        let mut memory = Parser::default();
        assert_eq!(feed(&mut memory, &live, true), []);
        assert_eq!(feed(&mut memory, &meta, true), []);
        assert_eq!(feed(&mut memory, &piece, true).len(), 1);
        assert_eq!(feed(&mut memory, &end, true), [Event::LogEnd]);
        let mut plain = Parser::default();
        assert_eq!(feed(&mut plain, &meta, false), []);
        assert_eq!(feed(&mut plain, &piece, false), []);
        assert_eq!(feed(&mut plain, &end, false), []);
        assert_eq!(feed(&mut plain, &live, false).len(), 1);
        assert_eq!(plain.log_buf.len(), 0, "a log is nobody's in live mode");
    }

    /// The memory of `n` records in pieces of up to `piece`, with a start and a
    /// metadata ahead and the last piece padded.
    fn log_bytes(records: &[[u8; 20]], piece: usize) -> Vec<Vec<u8>> {
        let mut memory: Vec<u8> = records.iter().flatten().copied().collect();
        memory.resize(memory.len().div_ceil(piece) * piece, 0xff);
        let count = (records.len() as u16).to_le_bytes();
        let mut frames =
            vec![frame(0x18, &[0]), frame(0x11, &[count[0], count[1], 0, 0, 0, 0, 0, 0])];
        frames.extend(memory.chunks(piece).map(|c| frame(0x14, c)));
        frames.push(frame(0x19, &[]));
        frames
    }

    #[test]
    fn records_come_across_the_pieces_and_padding_makes_none() {
        for piece in [1, 7, 20, 32] {
            let frames = log_bytes(&[record(10, 20), record(-30, 0x7fff), record(50, 60)], piece);
            let mut p = Parser::default();
            let mut got = Vec::new();
            for f in &frames {
                got.extend(feed(&mut p, f, true));
            }
            let time = [12, 34, 56];
            assert_eq!(
                got,
                [
                    Event::Record { probes: [1.0, 2.0], time },
                    Event::Record { probes: [-3.0, f32::INFINITY], time },
                    Event::Record { probes: [5.0, 6.0], time },
                    Event::LogEnd,
                ],
                "pieces of {piece}"
            );
        }
    }

    #[test]
    fn the_count_is_little_endian() {
        let mut p = Parser::default();
        feed(&mut p, &frame(0x11, &[0x34, 0x12, 9, 9, 9, 9, 9, 9]), true);
        assert_eq!(p.records_due, 0x1234);
    }

    #[test]
    fn a_metadata_too_short_changes_nothing() {
        let mut p = Parser::default();
        feed(&mut p, &frame(0x11, &[5, 0]), true);
        assert_eq!(p.records_due, 5);
        feed(&mut p, &frame(0x11, &[9]), true);
        assert_eq!(p.records_due, 5);
    }

    #[test]
    fn more_log_than_the_count_is_dropped_without_end() {
        // In C, the bytes pile up and at 64 the loop in log_data never ends.
        let mut p = Parser::default();
        feed(&mut p, &frame(0x11, &[1, 0, 0, 0, 0, 0, 0, 0]), true);
        let mut got = Vec::new();
        for _ in 0..10 {
            got.extend(feed(&mut p, &frame(0x14, &[0; 32]), true));
        }
        assert_eq!(got.len(), 1);
        assert_eq!(p.log_buf.len(), 0);
        assert_eq!(feed(&mut p, &frame(0x19, &[]), true), [Event::LogEnd]);
    }

    #[test]
    fn a_log_with_no_metadata_gives_no_records_and_still_ends() {
        let mut p = Parser::default();
        for _ in 0..10 {
            assert_eq!(feed(&mut p, &frame(0x14, &record(1, 2)), true), []);
        }
        assert!(p.told_no_metadata);
        assert_eq!(feed(&mut p, &frame(0x19, &[]), true), [Event::LogEnd]);
    }

    #[test]
    fn padding_behind_good_metadata_is_not_a_log_with_none() {
        let frames = log_bytes(&[record(10, 20)], 32);
        let mut p = Parser::default();
        for f in &frames {
            feed(&mut p, f, true);
        }
        assert!(!p.told_no_metadata);
        assert_eq!(p.records_due, 0);
    }

    #[test]
    fn a_new_log_asks_for_its_own_metadata() {
        let frames = log_bytes(&[record(10, 20)], 32);
        let mut p = Parser::default();
        for f in &frames {
            feed(&mut p, f, true);
        }
        feed(&mut p, &frame(0x18, &[0]), true);
        feed(&mut p, &frame(0x14, &[0; 32]), true);
        assert!(p.told_no_metadata);
    }

    #[test]
    fn a_broken_sync_in_a_log_ends_the_records() {
        let records: Vec<[u8; 20]> = (0..6).map(|i| record(100 + i, 200 + i)).collect();
        let mut frames = log_bytes(&records, 32);
        frames[3][0] = 0x54;
        let mut p = Parser::default();
        let mut got = Vec::new();
        for f in &frames {
            for chunk in f.chunks(37) {
                got.extend(feed(&mut p, chunk, true));
            }
        }
        assert_eq!(got.len(), 2, "the first piece's record and the end: {got:?}");
        assert_eq!(got[1], Event::LogEnd);
        assert_eq!(p.records_due, 0);
        assert!(!p.told_no_metadata, "it had its metadata");
    }

    #[test]
    fn a_piece_gone_without_a_trace_leaves_records_due_at_the_end() {
        let records: Vec<[u8; 20]> = (0..6).map(|i| record(100 + i, 200 + i)).collect();
        let mut frames = log_bytes(&records, 32);
        frames.remove(3);
        let mut p = Parser::default();
        let mut got = Vec::new();
        for f in &frames {
            got.extend(feed(&mut p, f, true));
        }
        assert_eq!(got.last(), Some(&Event::LogEnd));
        assert_eq!(got.len() - 1 + p.records_due as usize, 6);
        assert!(p.records_due > 0, "the end is what shows it");
    }

    #[test]
    fn noise_outside_a_log_or_in_live_mode_leaves_the_count() {
        let mut live = Parser { records_due: 3, ..Parser::default() };
        feed(&mut live, &[0x3f; 9], false);
        assert_eq!(live.records_due, 3, "live mode");
        let mut memory = Parser { records_due: 3, ..Parser::default() };
        feed(&mut memory, &[0x3f; 9], true);
        assert_eq!(memory.records_due, 0, "the same bytes in memory mode");
    }

    #[test]
    fn a_broken_piece_ends_the_records_and_not_the_log() {
        let frames = log_bytes(&[record(10, 20), record(30, 40), record(50, 60)], 20);
        let mut p = Parser::default();
        let mut got = Vec::new();
        got.extend(feed(&mut p, &frames[0], true));
        got.extend(feed(&mut p, &frames[1], true));
        got.extend(feed(&mut p, &frames[2], true));
        let mut broken = frames[3].clone();
        *broken.last_mut().unwrap() ^= 1;
        got.extend(feed(&mut p, &broken, true));
        got.extend(feed(&mut p, &frames[4], true));
        got.extend(feed(&mut p, &frames[5], true));
        assert_eq!(got.len(), 2, "the first record and the end: {got:?}");
        assert_eq!(got[1], Event::LogEnd);
    }

    #[test]
    fn a_broken_frame_outside_a_log_changes_nothing() {
        let mut f = frame(0x14, &[1; 32]);
        *f.last_mut().unwrap() ^= 1;
        let mut live = Parser { records_due: 3, ..Parser::default() };
        feed(&mut live, &f, false);
        assert_eq!(live.records_due, 3, "live mode keeps C's count");
        let mut memory = Parser::default();
        feed(&mut memory, &f, true);
        assert_eq!(memory.records_due, 0);
    }

    #[test]
    fn several_frames_in_a_read_all_count() {
        let f = frame(0x19, &[]);
        let mut data = f.clone();
        data.extend(&f);
        data.extend(&f);
        assert_eq!(feed(&mut Parser::default(), &data, true).len(), 3);
    }

    #[test]
    fn a_type_no_one_knows_is_passed_over() {
        let mut p = Parser::default();
        assert_eq!(feed(&mut p, &frame(0x77, &[1, 2]), false), []);
        assert_eq!(p.room(), BUF_SIZE);
    }
}
