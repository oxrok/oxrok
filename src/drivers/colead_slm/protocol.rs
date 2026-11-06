// Counterpart in sigrok:
//   libsigrok/src/hardware/colead-slm/protocol.c,
//   libsigrok/src/hardware/colead-slm/protocol.h,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Martin Ling, Daniel Elstner,
//   Aurelien Jacobs, Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The meter's protocol: an announcement, a request, a record.
//!
//! When a measurement is ready, two a second, the meter sends `0x10`. A host
//! that answers `0x20` gets ten bytes: `08 04`, the configuration, five digits,
//! a status and the sum of the nine before it. Nothing says which announcement a
//! record belongs to, and the record's own form is what tells it from the rest.

use crate::core::MqFlags;
use std::fmt;
use std::time::{Duration, Instant};

/// "Measurement ready", from the meter.
pub const READY: u8 = 0x10;
/// "Send measurement", to the meter.
pub const SEND: u8 = 0x20;
/// A record's length.
pub const RECORD_LEN: usize = 10;
/// What a read asks for while no record is expected, `buf[128]` in C. Its size
/// matters: all that a read gives must be the one byte to be an announcement.
pub const IDLE_READ: usize = 128;

/// A record's reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Level {
    /// In dB.
    pub value: f32,
    /// What the record's configuration says of it.
    pub flags: MqFlags,
    /// All five digits were passed over, so the value is C's 0.0 and says nothing.
    pub no_digits: bool,
}

/// Why a record gives no level. The texts are C's own debug lines.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refused {
    /// The first two bytes are not `08 04`.
    Header,
    /// The status is not 1, valid.
    Status,
    /// The sum does not add up.
    Checksum,
    /// The configuration's high nibble is neither normal nor max hold.
    Mode(u8),
    /// The configuration's low nibble is one of the two unused ones.
    Configuration(u8),
}

impl fmt::Display for Refused {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Refused::Header => write!(f, "invalid packet header."),
            Refused::Status => write!(f, "invalid measurement."),
            Refused::Checksum => write!(f, "invalid packet checksum."),
            Refused::Mode(m) => write!(f, "unknown measurement mode 0x{m:02x}"),
            Refused::Configuration(c) => write!(f, "unknown configuration 0x{c:02x}"),
        }
    }
}

/// `process_packet` up to the send, on bytes rather than on C's `char`: where
/// `char` is signed, a sum of 0x80 or more never matches (protocol.c:63) and a
/// digit of 0x80 or more goes in as a negative one (:70-73). No record a meter
/// sends has either, as its largest sum is 108.
///
/// # Errors
///
/// When the record gives no level.
pub fn decode(record: &[u8; RECORD_LEN]) -> Result<Level, Refused> {
    if record[0] != 0x08 || record[1] != 0x04 {
        return Err(Refused::Header);
    }
    if record[8] != 0x01 {
        return Err(Refused::Status);
    }
    let sum = record[..9].iter().fold(0u8, |sum, &b| sum.wrapping_add(b));
    if sum != record[9] {
        return Err(Refused::Checksum);
    }
    let mut value = 0.0f32;
    let mut no_digits = true;
    for &digit in &record[3..8] {
        if digit > 0x09 {
            continue;
        }
        no_digits = false;
        value *= 10.0;
        value += f32::from(digit);
    }
    value /= 10.0;
    let hold = match record[2] >> 4 {
        0x1 => MqFlags::empty(),
        0x2 => MqFlags::HOLD,
        mode => return Err(Refused::Mode(mode)),
    };
    let low = record[2] & 0x0f;
    let flags = configuration(low).ok_or(Refused::Configuration(low))?;
    Ok(Level { value, flags: hold | flags, no_digits })
}

/// Whether this is a well-formed record whose status says "not valid": the one
/// thing with no level in it that shows the meter is there and working.
pub fn says_not_valid(record: &[u8; RECORD_LEN]) -> bool {
    record[0] == 0x08
        && record[1] == 0x04
        && record[8] == 0x00
        && record[..9].iter().fold(0u8, |sum, &b| sum.wrapping_add(b)) == record[9]
}

/// The low nibble's table. Over 0xb the driver says less than the meter does: a
/// calibration has no time weighting there, and the means the same flags
/// whatever their period (protocol.c:132-157).
fn configuration(low: u8) -> Option<MqFlags> {
    use MqFlags as F;
    let a = F::SPL_FREQ_WEIGHT_A;
    let c = F::SPL_FREQ_WEIGHT_C;
    let z = F::SPL_FREQ_WEIGHT_FLAT;
    let fast = F::SPL_TIME_WEIGHT_F;
    let slow = F::SPL_TIME_WEIGHT_S;
    Some(match low {
        0x0 => a | fast,
        0x1 => a | slow,
        0x2 => c | fast,
        0x3 => c | slow,
        0x4 => z | fast,
        0x5 => z | slow,
        0x6 => F::SPL_PCT_OVER_ALARM | a | fast,
        0x7 => F::SPL_PCT_OVER_ALARM | a | slow,
        0x8 | 0x9 => F::SPL_LAT | a | fast,
        0xa | 0xb => F::SPL_LAT | a | slow,
        0xc | 0xd => z,
        _ => return None,
    })
}

/// What a read comes to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// Nothing to do yet.
    Wait,
    /// The meter announced: ask it, and say [`Machine::asked`] if that went.
    Ask,
    /// A record's ten bytes are here.
    Record([u8; RECORD_LEN]),
}

/// `colead_slm_receive_data`'s two states, with no port or clock in it.
#[derive(Clone, Debug, Default)]
pub struct Machine {
    /// How much of a record has come, while one is owed.
    sent: Option<usize>,
    buf: [u8; RECORD_LEN],
}

impl Machine {
    /// How many bytes the next read may take: a record's bytes never run into
    /// the announcement behind them.
    pub fn want(&self) -> usize {
        self.sent.map_or(IDLE_READ, |len| RECORD_LEN - len)
    }

    /// Whether a record is owed.
    pub fn waiting(&self) -> bool {
        self.sent.is_some()
    }

    /// The bytes of one read. All of a read that is more than the one byte
    /// 0x10 is dropped while no record is owed: it is the tail of an old record
    /// or garbage, and an announcement among them goes with it.
    pub fn got(&mut self, data: &[u8]) -> Step {
        let Some(len) = self.sent else {
            return if data == [READY] { Step::Ask } else { Step::Wait };
        };
        let n = data.len().min(RECORD_LEN - len);
        self.buf[len..len + n].copy_from_slice(&data[..n]);
        if len + n == RECORD_LEN {
            self.sent = None;
            return Step::Record(self.buf);
        }
        self.sent = Some(len + n);
        Step::Wait
    }

    /// The request went: what comes now is a record, from its first byte.
    pub fn asked(&mut self) {
        self.sent = Some(0);
    }
}

/// What a capture with only a sample limit has heard for too long.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Silence {
    /// Not a byte: the meter is off, or its cable is out.
    NoBytes,
    /// Bytes, and no level in them.
    NoLevel,
    /// Well-formed records that say their readings are not valid. This one has
    /// started a new bound of its own.
    NotValid,
}

/// How long it has been since a level. The times come in as arguments, so that
/// its tests take no time.
///
/// Time this thread did not run counts for nothing, as a stopped process or a
/// consumer that held a send heard nothing either: a capture is not ended by what
/// came while it could not look, and the look after it may read a level yet. That
/// credit has a limit, [`NoLevel::MOST_CREDIT`], so a process that is stopped for
/// more than a second again and again still comes to its end.
#[derive(Clone, Debug)]
pub struct NoLevel {
    bound: Duration,
    since: Instant,
    looked: Instant,
    heard: bool,
    not_valid: bool,
    /// Said once, for the whole capture.
    said_not_valid: bool,
    credit: Duration,
}

impl NoLevel {
    /// The longest a turn of the capture's loop takes when it runs at all: a
    /// read's wait, a request's write, and room for a loaded box.
    const STALL: Duration = Duration::from_secs(1);
    /// The most time that counts for nothing, between two levels.
    const MOST_CREDIT: Duration = Duration::from_secs(60);

    /// None heard so far, from `now`.
    pub fn new(bound: Duration, now: Instant) -> NoLevel {
        NoLevel {
            bound,
            since: now,
            looked: now,
            heard: false,
            not_valid: false,
            said_not_valid: false,
            credit: Duration::ZERO,
        }
    }

    /// Bytes came.
    pub fn heard(&mut self) {
        self.heard = true;
    }

    /// A well-formed record said "not valid".
    pub fn not_valid(&mut self) {
        self.not_valid = true;
    }

    /// A level went out; `now` is taken after its send.
    pub fn level(&mut self, now: Instant) {
        self.restart(now);
    }

    /// A new bound from `now`, with nothing heard.
    pub fn restart(&mut self, now: Instant) {
        *self = NoLevel { said_not_valid: self.said_not_valid, ..NoLevel::new(self.bound, now) };
    }

    /// What the bound has heard so far.
    pub fn silence(&self) -> Silence {
        match (self.not_valid, self.heard) {
            (true, _) => Silence::NotValid,
            (false, true) => Silence::NoLevel,
            (false, false) => Silence::NoBytes,
        }
    }

    /// Asked once per turn of the loop, after its read.
    pub fn look(&mut self, now: Instant) -> Option<Silence> {
        let gap = now.saturating_duration_since(self.looked);
        self.looked = now;
        let left = Self::MOST_CREDIT.saturating_sub(self.credit);
        if gap > Self::STALL && !left.is_zero() {
            let credit = gap.min(left);
            self.since += credit;
            self.credit += credit;
            return None;
        }
        if now.saturating_duration_since(self.since) < self.bound {
            return None;
        }
        let silence = self.silence();
        if silence == Silence::NotValid {
            // The meter is there. Whether it stays is for the next bound to say.
            let credit = self.credit;
            let said = self.said_not_valid;
            self.restart(now);
            self.credit = credit;
            self.said_not_valid = true;
            return (!said).then_some(silence);
        }
        Some(silence)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(config: u8, digits: [u8; 5], status: u8) -> [u8; RECORD_LEN] {
        let mut r = [0x08, 0x04, config, 0, 0, 0, 0, 0, status, 0];
        r[3..8].copy_from_slice(&digits);
        r[9] = r[..9].iter().fold(0u8, |s, &b| s.wrapping_add(b));
        r
    }

    #[test]
    fn a_level_as_the_wiki_writes_it() {
        let level = decode(&[0x08, 0x04, 0x10, 0x0a, 0x0a, 0x07, 0x05, 0x03, 0x01, 0x40]).unwrap();
        assert_eq!(level.value.to_bits(), 75.3f32.to_bits());
        assert_eq!(level.flags, MqFlags::SPL_FREQ_WEIGHT_A | MqFlags::SPL_TIME_WEIGHT_F);
    }

    #[test]
    fn digits() {
        for (digits, want) in [
            ([0, 0, 0, 0, 0], 0.0f32),
            ([9, 9, 9, 9, 9], 9999.9),
            ([1, 2, 3, 4, 5], 1234.5),
            ([10, 10, 10, 10, 5], 0.5),
            ([1, 10, 2, 10, 3], 12.3),
            ([11, 12, 7, 15, 5], 7.5),
            ([0x7f, 16, 7, 5, 3], 75.3),
            ([10, 10, 10, 10, 10], 0.0),
        ] {
            assert_eq!(decode(&record(0x10, digits, 1)).unwrap().value, want, "{digits:?}");
        }
    }

    #[test]
    fn a_digit_with_the_top_bit_is_passed_over_as_any_other_above_nine() {
        // Where char is signed, C takes 0xff for -1 and goes on, to -1024.7.
        let r = record(0x10, [0xff, 0xff, 7, 5, 3], 1);
        assert!(r[9] < 0x80, "{:#x}", r[9]);
        assert_eq!(decode(&r).unwrap().value, 75.3);
    }

    #[test]
    fn a_sum_with_the_top_bit_is_a_sum() {
        // Where char is signed, C never matches this one.
        let r = record(0x10, [0x60, 0x0a, 7, 5, 3], 1);
        assert!(r[9] >= 0x80, "{:#x}", r[9]);
        assert_eq!(decode(&r).unwrap().value, 75.3);
    }

    #[test]
    fn every_configuration() {
        use MqFlags as F;
        let (a, c, z) = (F::SPL_FREQ_WEIGHT_A, F::SPL_FREQ_WEIGHT_C, F::SPL_FREQ_WEIGHT_FLAT);
        let (fast, slow) = (F::SPL_TIME_WEIGHT_F, F::SPL_TIME_WEIGHT_S);
        let (over, lat) = (F::SPL_PCT_OVER_ALARM, F::SPL_LAT);
        let table = [
            a | fast,
            a | slow,
            c | fast,
            c | slow,
            z | fast,
            z | slow,
            over | a | fast,
            over | a | slow,
            lat | a | fast,
            lat | a | fast,
            lat | a | slow,
            lat | a | slow,
            z,
            z,
        ];
        for (low, want) in table.into_iter().enumerate() {
            let low = u8::try_from(low).unwrap();
            let flags = decode(&record(0x10 | low, [0, 0, 0, 0, 1], 1)).unwrap().flags;
            assert_eq!(flags, want, "{low:#x}");
            let flags = decode(&record(0x20 | low, [0, 0, 0, 0, 1], 1)).unwrap().flags;
            assert_eq!(flags, want | F::HOLD, "hold {low:#x}");
        }
    }

    #[test]
    fn what_gives_no_level_and_in_what_order() {
        let good = record(0x10, [0, 0, 0, 0, 1], 1);
        let mut r = good;
        r[0] = 0x09;
        assert_eq!(decode(&r), Err(Refused::Header));
        let mut r = good;
        r[1] = 0x05;
        assert_eq!(decode(&r), Err(Refused::Header));
        assert_eq!(decode(&record(0x10, [0; 5], 0)), Err(Refused::Status));
        assert_eq!(decode(&record(0x10, [0; 5], 2)), Err(Refused::Status));
        let mut r = good;
        r[9] ^= 1;
        assert_eq!(decode(&r), Err(Refused::Checksum));
        for high in [0x00, 0x30, 0xf0] {
            let want = Err(Refused::Mode(high >> 4));
            assert_eq!(decode(&record(high, [0; 5], 1)), want);
        }
        for low in [0x0e, 0x0f] {
            assert_eq!(decode(&record(0x10 | low, [0; 5], 1)), Err(Refused::Configuration(low)));
        }
        // The order, for what it logs: a bad status ahead of a bad sum, a bad
        // sum ahead of a bad mode, a bad mode ahead of a bad configuration.
        let mut r = record(0x10, [0; 5], 0);
        r[9] ^= 1;
        assert_eq!(decode(&r), Err(Refused::Status));
        let mut r = record(0x00, [0; 5], 1);
        r[9] ^= 1;
        assert_eq!(decode(&r), Err(Refused::Checksum));
        assert_eq!(decode(&record(0x0e, [0; 5], 1)), Err(Refused::Mode(0)));
    }

    #[test]
    fn the_words_are_cs() {
        assert_eq!(Refused::Header.to_string(), "invalid packet header.");
        assert_eq!(Refused::Status.to_string(), "invalid measurement.");
        assert_eq!(Refused::Checksum.to_string(), "invalid packet checksum.");
        assert_eq!(Refused::Mode(3).to_string(), "unknown measurement mode 0x03");
        assert_eq!(Refused::Configuration(0xe).to_string(), "unknown configuration 0x0e");
    }

    #[test]
    fn only_a_read_of_the_one_byte_is_an_announcement() {
        let mut m = Machine::default();
        assert_eq!(m.want(), 128);
        for read in [&[0x10, 0x10][..], &[0xff], &[0x40, 0x10], &[0x10, 0x40], &[]] {
            assert_eq!(m.got(read), Step::Wait, "{read:02x?}");
            assert!(!m.waiting());
        }
        assert_eq!(m.got(&[0x10]), Step::Ask);
    }

    #[test]
    fn an_ask_that_did_not_go_leaves_it_waiting_for_the_next() {
        let mut m = Machine::default();
        assert_eq!(m.got(&[0x10]), Step::Ask);
        assert!(!m.waiting());
        assert_eq!(m.got(&[0x10]), Step::Ask);
    }

    #[test]
    fn a_record_in_pieces_is_read_to_its_end_and_no_further() {
        let good = record(0x10, [0, 0, 0, 0, 1], 1);
        let mut m = Machine::default();
        m.got(&[0x10]);
        m.asked();
        assert_eq!(m.want(), 10);
        assert_eq!(m.got(&good[..3]), Step::Wait);
        assert_eq!(m.want(), 7);
        assert_eq!(m.got(&good[3..9]), Step::Wait);
        assert_eq!(m.want(), 1);
        assert_eq!(m.got(&good[9..]), Step::Record(good));
        assert_eq!(m.want(), 128);
        assert!(!m.waiting());
    }

    #[test]
    fn a_request_starts_a_record_from_nothing() {
        let good = record(0x10, [0, 0, 0, 0, 1], 1);
        let mut m = Machine::default();
        m.asked();
        m.got(&[1, 2, 3]);
        m.asked();
        assert_eq!(m.want(), 10);
        assert_eq!(m.got(&good), Step::Record(good));
    }

    #[test]
    fn announcements_fill_a_late_record_and_then_it_is_in_step() {
        let good = record(0x10, [0, 0, 0, 0, 1], 1);
        let mut m = Machine::default();
        m.got(&[0x10]);
        m.asked();
        // Two announcements ahead of the reply: its last two bytes are over.
        assert_eq!(m.got(&[0x10]), Step::Wait);
        assert_eq!(m.got(&[0x10]), Step::Wait);
        let Step::Record(r) = m.got(&good) else { panic!("no record") };
        assert_eq!(decode(&r), Err(Refused::Header));
        assert_eq!(m.got(&good[8..]), Step::Wait);
        assert_eq!(m.got(&[0x10]), Step::Ask);
    }

    fn secs(s: u64) -> Duration {
        Duration::from_secs(s)
    }

    /// A look every half second up to `to`, and when the first says something.
    fn fires_at(n: &mut NoLevel, t0: Instant, to: u64) -> Option<(u64, Silence)> {
        (1..=to * 2).find_map(|half| {
            let at = Duration::from_millis(500 * half);
            n.look(t0 + at).map(|s| (half, s))
        })
    }

    #[test]
    fn the_bound_comes_at_its_time_from_both_sides() {
        let t0 = Instant::now();
        let mut n = NoLevel::new(secs(15), t0);
        assert_eq!(fires_at(&mut n, t0, 14), None);
        assert_eq!(n.look(t0 + Duration::from_millis(14_999)), None);
        assert_eq!(n.look(t0 + secs(15)), Some(Silence::NoBytes));
    }

    #[test]
    fn bytes_are_no_level_and_say_so() {
        let t0 = Instant::now();
        let mut n = NoLevel::new(secs(15), t0);
        n.heard();
        assert_eq!(fires_at(&mut n, t0, 16), Some((30, Silence::NoLevel)));
    }

    #[test]
    fn a_level_starts_it_over_and_forgets_the_bytes() {
        let t0 = Instant::now();
        let mut n = NoLevel::new(secs(15), t0);
        n.heard();
        assert_eq!(fires_at(&mut n, t0, 10), None);
        let t1 = t0 + secs(10);
        n.level(t1);
        assert_eq!(fires_at(&mut n, t1, 14), None);
        assert_eq!(n.look(t1 + secs(15)), Some(Silence::NoBytes));
    }

    #[test]
    fn credit_comes_to_an_end() {
        // Stopped for 1.2 s, run for 0.05 s, for ever: 60 s of credit, then the bound.
        let t0 = Instant::now();
        let mut n = NoLevel::new(secs(15), t0);
        let step = Duration::from_millis(1250);
        let mut at = t0;
        let fired = (1..200).find_map(|i| {
            at += step;
            n.look(at).map(|s| (i, s))
        });
        let (i, silence) = fired.expect("it never ended");
        assert_eq!(silence, Silence::NoBytes);
        let took = step * i;
        assert!(took >= secs(74) && took <= secs(76), "{took:?}");
    }

    #[test]
    fn a_level_gives_the_credit_back() {
        let t0 = Instant::now();
        let mut n = NoLevel::new(secs(15), t0);
        assert_eq!(n.look(t0 + secs(100)), None);
        let t1 = t0 + secs(100);
        n.level(t1);
        assert_eq!(n.look(t1 + secs(50)), None, "credit for a stop after a level");
    }

    #[test]
    fn not_valid_is_said_at_the_bound_and_starts_another() {
        let t0 = Instant::now();
        let mut n = NoLevel::new(secs(15), t0);
        n.heard();
        n.not_valid();
        assert_eq!(fires_at(&mut n, t0, 16), Some((30, Silence::NotValid)));
        // A second bound of the same goes by without a word, levels or no levels.
        let t1 = t0 + secs(15);
        n.heard();
        n.not_valid();
        assert_eq!(fires_at(&mut n, t1, 16), None);
        let t1 = t1 + secs(16);
        n.level(t1);
        n.heard();
        n.not_valid();
        assert_eq!(fires_at(&mut n, t1, 16), None);
        let t1 = t1 + secs(15);
        // Nothing since then, not a byte: that is the end.
        assert_eq!(fires_at(&mut n, t1, 14), None);
        assert_eq!(n.look(t1 + secs(15)), Some(Silence::NoBytes));
    }

    #[test]
    fn says_not_valid_needs_a_good_header_status_0_and_a_good_sum() {
        assert!(says_not_valid(&record(0x10, [10, 10, 7, 5, 3], 0)));
        assert!(!says_not_valid(&record(0x10, [10, 10, 7, 5, 3], 1)));
        assert!(!says_not_valid(&record(0x10, [10, 10, 7, 5, 3], 2)));
        let mut r = record(0x10, [10, 10, 7, 5, 3], 0);
        r[9] ^= 1;
        assert!(!says_not_valid(&r));
        let mut r = record(0x10, [10, 10, 7, 5, 3], 0);
        r[0] ^= 1;
        r[9] ^= 1;
        assert!(!says_not_valid(&r));
        let mut r = record(0x10, [10, 10, 7, 5, 3], 0);
        r[1] ^= 1;
        r[9] = r[..9].iter().fold(0u8, |s, &b| s.wrapping_add(b));
        assert!(!says_not_valid(&r));
    }

    #[test]
    fn a_record_with_no_digits_says_so() {
        let blank = decode(&record(0x10, [10, 10, 10, 10, 10], 1)).unwrap();
        assert_eq!(blank.value, 0.0);
        assert!(blank.no_digits);
        assert!(decode(&record(0x10, [11, 0x7f, 16, 12, 13], 1)).unwrap().no_digits);
        assert!(!decode(&record(0x10, [10, 10, 10, 10, 0], 1)).unwrap().no_digits);
        assert!(!decode(&record(0x10, [0, 10, 10, 10, 10], 1)).unwrap().no_digits);
    }

    #[test]
    fn the_time_a_stop_or_a_slow_consumer_held_it_counts_for_nothing() {
        let t0 = Instant::now();
        let mut n = NoLevel::new(secs(15), t0);
        assert_eq!(fires_at(&mut n, t0, 14), None);
        let t1 = t0 + secs(14);
        // A minute gone. This look is not the end, nor the next soon after.
        assert_eq!(n.look(t1 + secs(60)), None);
        assert_eq!(n.look(t1 + secs(60) + Duration::from_millis(500)), None);
        // What was left of the bound is one second.
        assert_eq!(n.look(t1 + secs(61)), Some(Silence::NoBytes));
    }
}
