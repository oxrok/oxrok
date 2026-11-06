// Counterpart in sigrok:
//   libsigrok/src/hardware/lecroy-xstream/protocol.c,
//   libsigrok/src/hardware/lecroy-xstream/protocol.h,
//   Copyright (C) 2017 Sven Schnelle <svens@stackframe.org>,
//   Copyright (C) 2013 poljar (Damir Jelić) <poljarinho@gmail.com>,
//   with commits by Soeren Apel, Uwe Hermann, Gerhard Sittig,
//   Sylvain Pelissier, Bert Vermeulen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! What the scope says, with no I/O: its numbers, its blocks and its
//! descriptors, and how long its silence may last.
//!
//! The layout is the maker's `LECROY_2_3` template ("Remote Control Manual", LTXXX-RCM-E
//! Rev B, 2002, Appendix II, and `TEMPLATE?` on a scope). Where this reads a waveform
//! otherwise than libsigrok does:
//!
//! - a sample is `gain * data - offset`, the manual's relation (its worked example:
//!   `2.44141e-7 * -1536 - 0.00054 = -0.000915 V`), where `protocol.c:561` adds;
//! - samples follow `TRIGTIME` and `RISTIME` too, where `protocol.c:556` counts only
//!   the descriptor and the user text, and in a sequence or an interleaved acquisition
//!   gives doubles as samples;
//! - `VERTUNIT` of `A` is amperes and anything else volts, where the sense of
//!   the `strcmp` at `protocol.c:580` makes volts amperes;
//! - `COMM_ORDER` and `COMM_TYPE` are read, as the manual has `HI` for a scope's
//!   default, where C takes every field as little endian words;
//! - what doesn't fit its block is refused, where C reads on past the buffer.
//!
//! What the format can't catch: it has no checksum, so a damaged sample or a damaged
//! gain that stays a finite number reads as a good one.

use crate::core::{Error, Mq, Result, Unit};
use std::time::{Duration, Instant};

/// What libsigrok's struct holds, up to `TRIGGER_TIME`, and so the least a descriptor
/// can be. The manual's is 346 bytes.
pub const MIN_DESCRIPTOR: usize = 304;

/// A number from the scope as an exact fraction: `sr_parse_rational`, whose form it
/// keeps: a sign, digits with a point, an exponent, nothing after. `None` for what
/// isn't, and for what doesn't fit, where C's integers overflow.
pub fn parse_rational(text: &str) -> Option<(i128, i128)> {
    let s = text.trim_start();
    let (negative, s) = match s.as_bytes().first() {
        Some(b'-') => (true, &s[1..]),
        Some(b'+') => (false, &s[1..]),
        _ => (false, s),
    };
    let (mantissa, exponent) = match s.find(['e', 'E']) {
        Some(at) => {
            let e = &s[at + 1..];
            let digits = e.strip_prefix(['+', '-']).unwrap_or(e);
            if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
                return None;
            }
            (&s[..at], e.parse::<i32>().ok()?)
        }
        None => (s, 0),
    };
    let (whole, fraction) = mantissa.split_once('.').unwrap_or((mantissa, ""));
    if (whole.is_empty() && fraction.is_empty())
        || !whole.bytes().chain(fraction.bytes()).all(|b| b.is_ascii_digit())
    {
        return None;
    }
    let mut p: i128 = 0;
    for b in whole.bytes().chain(fraction.bytes()) {
        p = p.checked_mul(10)?.checked_add(i128::from(b - b'0'))?;
    }
    let exponent = exponent.checked_sub(i32::try_from(fraction.len()).ok()?)?;
    let ten = 10i128.checked_pow(exponent.unsigned_abs())?;
    let (p, q) = if exponent >= 0 { (p.checked_mul(ten)?, 1) } else { (p, ten) };
    Some((if negative { -p } else { p }, q))
}

/// Where `text` is in a table of fractions, by value, as `array_float_get` has it.
pub fn table_index(table: &[(u64, u64)], text: &str) -> Option<usize> {
    let (p, q) = parse_rational(text)?;
    table.iter().position(|&(tp, tq)| {
        p.checked_mul(i128::from(tq))
            .is_some_and(|a| q.checked_mul(i128::from(tp)).is_some_and(|b| a == b))
    })
}

/// `%e` of C, or with `upper` `%E`: six places, and an exponent of two digits or more.
pub fn c_exp(value: f64, upper: bool) -> String {
    if value.is_nan() {
        return if upper { "NAN" } else { "nan" }.to_owned();
    }
    if value.is_infinite() {
        let word = if upper { "INF" } else { "inf" };
        return if value < 0.0 { format!("-{word}") } else { word.to_owned() };
    }
    let text = format!("{value:.6e}");
    let (mantissa, exponent) = text.split_once('e').unwrap_or((&text, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    let sign = if exponent < 0 { '-' } else { '+' };
    let e = if upper { 'E' } else { 'e' };
    format!("{mantissa}{e}{sign}{:02}", exponent.abs())
}

/// The source in a `TRIG_SELECT?` answer: its type, then pairs, and the value of the
/// last `SR`. `strtok_r`'s reading, so that empty fields are no fields.
pub fn trigger_source(answer: &str) -> Option<&str> {
    let mut source = None;
    let mut key = "";
    for (i, field) in answer.split(',').filter(|f| !f.is_empty()).enumerate() {
        if i == 0 {
            continue;
        }
        if i % 2 == 1 {
            key = field;
        } else if key == "SR" {
            source = Some(field);
        }
    }
    source
}

/// Where a block's reading has got to.
#[derive(Debug, PartialEq, Eq)]
pub enum Framing {
    /// More bytes, please.
    More,
    /// The block's bytes.
    Whole(Vec<u8>),
}

/// An IEEE 488.2 definite length block, a read at a time.
///
/// `sr_scpi_get_block`'s rules, but for two. A line end left by the block before is
/// skipped: the manual ends a block with one, and on a socket it can come in the next
/// read, which C would refuse as no block. And nothing is allocated from the
/// announced length, only from what comes.
#[derive(Debug, Default)]
pub struct Framer {
    buffer: Vec<u8>,
    /// The bytes before the data, and how many of them.
    data_at: usize,
    length: Option<usize>,
}

impl Framer {
    /// Takes what a read gave.
    ///
    /// # Errors
    ///
    /// On what is no definite length block, with what came.
    pub fn feed(&mut self, bytes: &[u8]) -> Result<Framing> {
        let mut bytes = bytes;
        if self.buffer.is_empty() {
            let skip = bytes.iter().take_while(|b| matches!(b, b'\n' | b'\r')).count();
            bytes = &bytes[skip..];
        }
        self.buffer.extend_from_slice(bytes);
        if self.length.is_none() {
            let Some(first) = self.buffer.first() else { return Ok(Framing::More) };
            if *first != b'#' {
                return Err(self.no_block());
            }
            let Some(count) = self.buffer.get(1) else { return Ok(Framing::More) };
            if !count.is_ascii_digit() {
                return Err(self.no_block());
            }
            let digits = usize::from(count - b'0');
            if digits == 0 {
                log::error!("unsupported INDEFINITE LENGTH ARBITRARY BLOCK RESPONSE");
                return Err(Error::device("The scope sent a block of no stated length."));
            }
            if self.buffer.len() < 2 + digits {
                return Ok(Framing::More);
            }
            let text = &self.buffer[2..2 + digits];
            if !text.iter().all(u8::is_ascii_digit) {
                return Err(self.no_block());
            }
            let length = text.iter().fold(0usize, |n, b| n * 10 + usize::from(b - b'0'));
            if length == 0 {
                return Err(Error::device("The scope sent an empty block."));
            }
            self.data_at = 2 + digits;
            self.length = Some(length);
        }
        let need = self.data_at + self.length.unwrap_or(0);
        if self.buffer.len() < need {
            return Ok(Framing::More);
        }
        // What comes after is the line end, and only that: one question was out.
        self.buffer.truncate(need);
        self.buffer.drain(..self.data_at);
        Ok(Framing::Whole(std::mem::take(&mut self.buffer)))
    }

    /// Bytes came of a block, so a wait is no wait for an answer any more.
    pub fn begun(&self) -> bool {
        !self.buffer.is_empty()
    }

    /// How far a block got, as `(has, announced)`; `None` before its length is known.
    pub fn progress(&self) -> Option<(usize, usize)> {
        Some((self.buffer.len().saturating_sub(self.data_at), self.length?))
    }

    fn no_block(&self) -> Error {
        let head: Vec<u8> = self.buffer.iter().copied().take(16).collect();
        Error::device(format!(
            "The scope's answer is no block: it starts with '{}'.",
            String::from_utf8_lossy(&head).escape_debug()
        ))
    }
}

/// A channel's samples, from a `WAVEFORM?` block.
#[derive(Debug, PartialEq)]
pub struct Waveform {
    /// The samples, in the unit.
    pub samples: Vec<f32>,
    /// What they measure.
    pub mq: Mq,
    /// Their unit.
    pub unit: Unit,
}

/// A descriptor's field in its byte order.
struct Fields<'a> {
    data: &'a [u8],
    little: bool,
}

impl Fields<'_> {
    fn u16(&self, at: usize) -> u16 {
        let b = [self.data[at], self.data[at + 1]];
        if self.little { u16::from_le_bytes(b) } else { u16::from_be_bytes(b) }
    }

    fn u32(&self, at: usize) -> u32 {
        let b = [self.data[at], self.data[at + 1], self.data[at + 2], self.data[at + 3]];
        if self.little { u32::from_le_bytes(b) } else { u32::from_be_bytes(b) }
    }

    fn f32(&self, at: usize) -> f32 {
        f32::from_bits(self.u32(at))
    }
}

/// Reads a block's descriptor and samples, refusing what the document doesn't allow.
///
/// # Errors
///
/// On a block too short for what its descriptor says, an unknown template, and a
/// byte order, a sample size or a scale that the template has no such value for.
pub fn parse_waveform(data: &[u8]) -> Result<Waveform> {
    if data.len() < MIN_DESCRIPTOR {
        return Err(Error::device(format!(
            "The scope's block of {} bytes is too short for a descriptor.",
            data.len()
        )));
    }
    let template = &data[16..32];
    let name = template.split(|b| *b == 0).next().unwrap_or(template);
    if name != b"LECROY_2_2" && name != b"LECROY_2_3" {
        let text = String::from_utf8_lossy(name);
        log::error!("Waveformat template '{text}' not supported.");
        return Err(Error::device(format!("Waveformat template '{text}' not supported.")));
    }
    // The enum at 34 is 0 or 1, in either order, so its two bytes say which order.
    let little = match (data[34], data[35]) {
        (1, 0) => true,
        (0, 0 | 1) => false,
        (a, b) => {
            return Err(Error::device(format!("The scope's COMM_ORDER is {a:#x} {b:#x}.")));
        }
    };
    let fields = Fields { data, little };
    let width = match fields.u16(32) {
        0 => 1,
        1 => 2,
        other => return Err(Error::device(format!("The scope's COMM_TYPE is {other}."))),
    };
    let descriptor = fields.u32(36) as usize;
    if descriptor < MIN_DESCRIPTOR {
        return Err(Error::device(format!(
            "The scope's descriptor says it has {descriptor} bytes."
        )));
    }
    // The blocks ahead of the first data array, in the template's order. On 32 bits
    // a sum of u32 could wrap, so they add up as u64.
    let start = [36, 40, 48, 52].iter().map(|at| u64::from(fields.u32(*at))).sum::<u64>();
    let count = u64::from(fields.u32(116));
    let end = start + count * width;
    if end > data.len() as u64 {
        return Err(Error::device(format!(
            "The scope's descriptor says {count} samples from byte {start}, in a block of {} bytes.",
            data.len()
        )));
    }
    let gain = fields.f32(156);
    let offset = fields.f32(160);
    if !gain.is_finite() || !offset.is_finite() {
        return Err(Error::device(format!(
            "The scope's descriptor has a gain of {gain} and an offset of {offset}."
        )));
    }
    let bytes = &data[start as usize..end as usize];
    let raw = |chunk: &[u8]| -> i16 {
        match (chunk, little) {
            ([a], _) => i16::from(*a as i8),
            ([a, b], true) => i16::from_le_bytes([*a, *b]),
            ([a, b], false) => i16::from_be_bytes([*a, *b]),
            _ => 0,
        }
    };
    let samples =
        bytes.chunks_exact(width as usize).map(|c| f32::from(raw(c)) * gain - offset).collect();
    let unit = &data[196..244];
    let unit = unit.split(|b| *b == 0).next().unwrap_or(unit);
    let (mq, unit) = if unit == b"A" {
        (Mq::Current, Unit::Ampere)
    } else {
        // Default to voltage, as the C's comment has it.
        (Mq::Voltage, Unit::Volt)
    };
    Ok(Waveform { samples, mq, unit })
}

/// The longest a scope that has answered this capture may keep quiet before the
/// capture gives up: a choice, there being nothing in the manual to go by.
const LONG_LIMIT: Duration = Duration::from_secs(600);
/// The longest an unarmed query may go unanswered: it asks for what is on the scope.
const SHORT_BOUND: Duration = Duration::from_secs(30);
/// How long a block may pause between two bytes: a choice too. C's TCP has no limit
/// here and its USB one second, and there is no way to ask for the rest.
const BLOCK_PAUSE: Duration = Duration::from_secs(30);

/// What a wait comes to.
#[derive(Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Go on.
    Wait,
    /// Go on, and say this.
    Say(String),
    /// It's over: this is why.
    Fail(String),
}

/// What a question out comes to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Awaiting {
    /// What the scope has.
    Answer,
    /// The same, behind an acquisition.
    Acquisition,
    /// The rest of a block.
    Rest,
}

/// How long a scope may be silent.
///
/// Armed (`ARM;WAIT`), it has no bound: the manual says `WAIT` with no argument "will
/// wait indefinitely", a trigger may be a rare event, and a sweep takes up to 10000 s
/// here. So a word, and then a word when it's back; a stop or a gone device ends
/// it. Unarmed, it is asked for what it has: at most [`SHORT_BOUND`] or two sweeps,
/// which fails a capture that has had no sample. After a sample that's a word, and
/// [`LONG_LIMIT`] or four sweeps the end. Within a block, [`BLOCK_PAUSE`] ends it, as
/// a block cut short is no silence but what is left of an answer.
///
/// A scope that answers with no samples isn't silent, and its acquisitions do end, so
/// that has a bound too: a word at the short one, and the end at the long one, on
/// usable data and not on bytes ([`Silence::no_samples`]).
///
/// Times are arguments: a caller takes `now` before the read that found nothing, and
/// [`Silence::asked`] after a send, which a slow reader may have kept waiting.
#[derive(Debug)]
pub struct Silence {
    since: Instant,
    awaiting: Awaiting,
    had_sample: bool,
    warned: bool,
    /// Since when answers have had no samples, and whether that was said.
    empty_since: Option<Instant>,
    empty_said: bool,
    bound: Duration,
    limit: Duration,
    block_pause: Duration,
}

impl Silence {
    /// For a timebase in seconds a division, of a scope of ten divisions.
    pub fn new(now: Instant, time_div: f64) -> Silence {
        let sweep = Duration::try_from_secs_f64(time_div * 10.0).unwrap_or(Duration::ZERO);
        Silence {
            since: now,
            awaiting: Awaiting::Answer,
            had_sample: false,
            warned: false,
            empty_since: None,
            empty_said: false,
            bound: SHORT_BOUND.max(sweep * 2),
            limit: LONG_LIMIT.max(sweep * 4),
            block_pause: BLOCK_PAUSE,
        }
    }

    /// With bounds of a test's choosing.
    #[cfg(test)]
    pub fn with_bounds(now: Instant, bound: Duration, limit: Duration, pause: Duration) -> Silence {
        let mut s = Silence::new(now, 0.0);
        s.bound = bound;
        s.limit = limit;
        s.block_pause = pause;
        s
    }

    /// A question went out; `armed` when an acquisition stands before its answer.
    pub fn asked(&mut self, now: Instant, armed: bool) {
        self.since = now;
        self.awaiting = if armed { Awaiting::Acquisition } else { Awaiting::Answer };
    }

    /// Bytes came.
    pub fn heard(&mut self, now: Instant) -> Option<String> {
        self.since = now;
        self.awaiting = Awaiting::Rest;
        std::mem::take(&mut self.warned).then(|| "The scope is sending again.".to_owned())
    }

    /// A channel's samples came.
    pub fn sample(&mut self) {
        self.had_sample = true;
        self.empty_since = None;
        self.empty_said = false;
    }

    /// An answer came with no samples.
    pub fn no_samples(&mut self, now: Instant) -> Verdict {
        let since = *self.empty_since.get_or_insert(now);
        let empty = now.saturating_duration_since(since);
        let seconds = empty.as_secs();
        if empty >= self.limit {
            return Verdict::Fail(format!("The scope has sent no samples in {seconds} s."));
        }
        if empty >= self.bound && !self.empty_said {
            self.empty_said = true;
            return Verdict::Say(format!(
                "The scope has sent no samples in {seconds} s: still asking."
            ));
        }
        Verdict::Wait
    }

    /// A read found nothing, and `now` was taken ahead of it.
    pub fn nothing(&mut self, now: Instant) -> Verdict {
        let quiet = now.saturating_duration_since(self.since);
        let seconds = quiet.as_secs();
        if self.awaiting == Awaiting::Rest {
            if quiet >= self.block_pause {
                return Verdict::Fail(format!(
                    "The scope stopped within a block, {seconds} s ago."
                ));
            }
            return Verdict::Wait;
        }
        if self.awaiting == Awaiting::Acquisition {
            if quiet >= self.bound && !self.warned {
                self.warned = true;
                return Verdict::Say(format!(
                    "No trigger in {seconds} s, or the scope is gone: still waiting."
                ));
            }
            return Verdict::Wait;
        }
        if !self.had_sample {
            if quiet >= self.bound {
                return Verdict::Fail(format!("No answer from the scope in {seconds} s."));
            }
            return Verdict::Wait;
        }
        if quiet >= self.limit {
            return Verdict::Fail(format!("The scope has been silent for {seconds} s."));
        }
        if quiet >= self.bound && !self.warned {
            self.warned = true;
            return Verdict::Say(format!(
                "The scope has been silent for {seconds} s: still waiting."
            ));
        }
        Verdict::Wait
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A block's content by the manual's offsets, little endian unless asked.
    fn wave(samples: &[i16], gain: f32, offset: f32, unit: &str) -> Vec<u8> {
        let mut d = vec![0u8; 346];
        d[..8].copy_from_slice(b"WAVEDESC");
        d[16..26].copy_from_slice(b"LECROY_2_3");
        d[32] = 1;
        d[34] = 1;
        d[36..40].copy_from_slice(&346u32.to_le_bytes());
        d[60..64].copy_from_slice(&(samples.len() as u32 * 2).to_le_bytes());
        d[116..120].copy_from_slice(&(samples.len() as u32).to_le_bytes());
        d[156..160].copy_from_slice(&gain.to_le_bytes());
        d[160..164].copy_from_slice(&offset.to_le_bytes());
        d[196..196 + unit.len()].copy_from_slice(unit.as_bytes());
        for s in samples {
            d.extend_from_slice(&s.to_le_bytes());
        }
        d
    }

    #[test]
    fn the_manual_gives_its_worked_example() {
        // The bytes of its page 40: gain 3483126f, offset 3A0D8EC9, data FA00.
        let gain = f32::from_bits(0x3483_126f);
        let offset = f32::from_bits(0x3a0d_8ec9);
        let w = parse_waveform(&wave(&[0xfa00u16 as i16], gain, offset, "V")).unwrap();
        assert!((w.samples[0] - -0.000_915).abs() < 1e-6, "{}", w.samples[0]);
        assert_eq!((w.mq, w.unit), (Mq::Voltage, Unit::Volt));
    }

    #[test]
    fn units_go_by_the_field() {
        let w = parse_waveform(&wave(&[1], 1.0, 0.0, "A")).unwrap();
        assert_eq!((w.mq, w.unit), (Mq::Current, Unit::Ampere));
        let w = parse_waveform(&wave(&[1], 1.0, 0.0, "AV")).unwrap();
        assert_eq!((w.mq, w.unit), (Mq::Voltage, Unit::Volt));
        let w = parse_waveform(&wave(&[1], 1.0, 0.0, "S")).unwrap();
        assert_eq!((w.mq, w.unit), (Mq::Voltage, Unit::Volt), "the C's default");
    }

    #[test]
    fn samples_come_after_every_block_ahead_of_them() {
        let mut d = wave(&[], 2.0, 0.0, "V");
        d[40..44].copy_from_slice(&4u32.to_le_bytes());
        d[48..52].copy_from_slice(&16u32.to_le_bytes());
        d[52..56].copy_from_slice(&8u32.to_le_bytes());
        d[116..120].copy_from_slice(&2u32.to_le_bytes());
        d.extend_from_slice(&[0x55; 4 + 16 + 8]);
        d.extend_from_slice(&[3, 0, 0xff, 0xff]);
        assert_eq!(parse_waveform(&d).unwrap().samples, [6.0, -2.0]);
    }

    #[test]
    fn the_descriptor_goes_by_its_order_and_its_size() {
        let mut d = vec![0u8; 346];
        d[16..26].copy_from_slice(b"LECROY_2_2");
        d[33] = 1;
        d[36..40].copy_from_slice(&346u32.to_be_bytes());
        d[116..120].copy_from_slice(&2u32.to_be_bytes());
        d[156..160].copy_from_slice(&2.0f32.to_be_bytes());
        d.extend_from_slice(&[0, 3, 0xff, 0xff]);
        assert_eq!(parse_waveform(&d).unwrap().samples, [6.0, -2.0], "big endian");
        let mut d = wave(&[], 2.0, 0.0, "V");
        d[32] = 0;
        d[116..120].copy_from_slice(&2u32.to_le_bytes());
        d.extend_from_slice(&[3, 0xff]);
        assert_eq!(parse_waveform(&d).unwrap().samples, [6.0, -2.0], "bytes");
    }

    #[test]
    fn no_samples_is_a_scope_with_no_acquisition() {
        assert_eq!(parse_waveform(&wave(&[], 1.0, 0.0, "V")).unwrap().samples, []);
    }

    /// The text of what `parse_waveform` refuses.
    fn refusal(data: &[u8]) -> String {
        parse_waveform(data).unwrap_err().to_string()
    }

    #[test]
    fn what_the_document_doesnt_allow_is_refused() {
        let good = wave(&[1, 2], 1.0, 0.0, "V");
        assert!(refusal(&good[..303]).contains("block of 303 bytes is too short"));
        let mut d = good.clone();
        d[16..26].copy_from_slice(b"LECROY_9_9");
        assert!(refusal(&d).contains("template 'LECROY_9_9' not supported"));
        let mut d = good.clone();
        d[34] = 7;
        assert!(refusal(&d).contains("COMM_ORDER is 0x7 0x0"));
        let mut d = good.clone();
        d[32] = 9;
        assert!(refusal(&d).contains("COMM_TYPE is 9"));
        let mut d = good.clone();
        d[36..40].copy_from_slice(&303u32.to_le_bytes());
        assert!(refusal(&d).contains("says it has 303 bytes"));
        let mut d = good.clone();
        d[116..120].copy_from_slice(&3u32.to_le_bytes());
        assert!(refusal(&d).contains("3 samples from byte 346, in a block of 350 bytes"));
        let mut d = good.clone();
        d[116..120].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(refusal(&d).contains("4294967295 samples"), "no allocation from it");
        for len in [36usize, 40, 48, 52] {
            let mut d = good.clone();
            d[len..len + 4].copy_from_slice(&u32::MAX.to_le_bytes());
            assert!(refusal(&d).contains("samples from byte"), "a length at {len} can't wrap");
        }
        let mut d = good.clone();
        d[156..160].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(refusal(&d).contains("a gain of NaN"));
        let mut d = good;
        d[160..164].copy_from_slice(&f32::INFINITY.to_le_bytes());
        assert!(refusal(&d).contains("an offset of inf"));
    }

    /// Every single-bit flip and every lost byte of a frame: refused, or read, and
    /// right or wrong. The format has no checksum, so this states what it costs.
    fn damage(values: &[i16]) -> (usize, usize, usize, usize) {
        let good = wave(values, 1.0 / 8192.0, 0.0, "V");
        let right = parse_waveform(&good).unwrap();
        let (mut refused, mut wrong, mut same) = (0, 0, 0);
        let mut judge = |d: &[u8]| match parse_waveform(d) {
            Err(_) => refused += 1,
            Ok(w) if w == right => same += 1,
            Ok(_) => wrong += 1,
        };
        let mut total = 0;
        for at in 0..good.len() {
            for bit in 0..8 {
                let mut d = good.clone();
                d[at] ^= 1 << bit;
                judge(&d);
                total += 1;
            }
            let mut d = good.clone();
            d.remove(at);
            judge(&d);
            total += 1;
        }
        (total, refused, wrong, same)
    }

    #[test]
    fn what_damage_gets_through_is_known() {
        // Zero, small, middle and the ends of the range, as a failure may need a class.
        let values = [0, 1, -1, 300, -6000, 6000, i16::MAX, i16::MIN];
        let (total, refused, wrong, same) = damage(&values);
        println!("damage: {total} frames, {refused} refused, {wrong} wrong, {same} same");
        assert_eq!(total, refused + wrong + same);
        // The samples and the scale are unprotected: any such flip is a wrong reading.
        assert!(wrong >= 16 * 8, "{wrong}");
        // A wrong template, order or size in every class.
        assert!(refused > 0);
    }

    #[test]
    fn a_block_comes_in_pieces_after_a_line_end() {
        let mut f = Framer::default();
        assert_eq!(f.feed(b"\r\n").unwrap(), Framing::More);
        assert!(!f.begun(), "a line end is what's left of the last one");
        assert_eq!(f.feed(b"#").unwrap(), Framing::More);
        assert!(f.begun());
        assert_eq!(f.feed(b"2").unwrap(), Framing::More);
        assert_eq!(f.feed(b"0").unwrap(), Framing::More);
        assert_eq!(f.progress(), None);
        assert_eq!(f.feed(b"5ab").unwrap(), Framing::More);
        assert_eq!(f.progress(), Some((2, 5)));
        assert_eq!(f.feed(b"cde\n").unwrap(), Framing::Whole(b"abcde".to_vec()));
    }

    #[test]
    fn a_framer_takes_a_whole_block_in_one_read() {
        let mut f = Framer::default();
        assert_eq!(f.feed(b"#13abc\n").unwrap(), Framing::Whole(b"abc".to_vec()));
    }

    #[test]
    fn what_is_no_block_says_so() {
        let bad = |b: &[u8]| Framer::default().feed(b).unwrap_err().to_string();
        assert_eq!(
            bad(b"ALL,#9000"),
            "device error: The scope's answer is no block: it starts with 'ALL,#9000'."
        );
        assert!(bad(b"#x").contains("no block"));
        assert!(bad(b"#2a1").contains("no block"));
        assert!(bad(b"#0abc").contains("no stated length"));
        assert!(bad(b"#10").contains("an empty block"));
        assert!(bad(b"#9000000000").contains("an empty block"));
    }

    #[test]
    fn an_announced_length_allocates_nothing() {
        let mut f = Framer::default();
        assert_eq!(f.feed(b"#9999999999ab").unwrap(), Framing::More);
        assert!(f.buffer.capacity() < 1 << 16);
    }

    #[test]
    fn numbers_read_as_sr_parse_rational_does() {
        assert_eq!(parse_rational("2.000E-03"), Some((2000, 1_000_000)));
        assert_eq!(parse_rational("200E-3"), Some((200, 1000)));
        assert_eq!(parse_rational(" -1.5"), Some((-15, 10)));
        assert_eq!(parse_rational("+.5e1"), Some((5, 1)));
        assert_eq!(parse_rational("5."), Some((5, 1)));
        assert_eq!(parse_rational("1e3"), Some((1000, 1)));
        for bad in ["", ".", "abc", "1.0 V", "1e", "1e+", "0x10", "1.0.0", "--1", "1 ", "1e99999"] {
            assert_eq!(parse_rational(bad), None, "{bad}");
        }
    }

    #[test]
    fn the_tables_match_by_value() {
        let t = [(1, 1000), (2, 1000), (500, 1_000_000_000_000), (1, 1)];
        assert_eq!(table_index(&t, "2.000E-03"), Some(1));
        assert_eq!(table_index(&t, "2E-3"), Some(1));
        assert_eq!(table_index(&t, "500E-12"), Some(2));
        assert_eq!(table_index(&t, "1"), Some(3));
        assert_eq!(table_index(&t, "3E-3"), None);
        assert_eq!(table_index(&t, "-1"), None);
        assert_eq!(table_index(&t, "volts"), None);
    }

    #[test]
    fn c_exp_writes_as_printf() {
        assert_eq!(c_exp(0.001, true), "1.000000E-03");
        assert_eq!(c_exp(1.0, true), "1.000000E+00");
        assert_eq!(c_exp(1000.0, true), "1.000000E+03");
        assert_eq!(c_exp(2e-11, true), "2.000000E-11");
        assert_eq!(c_exp(-0.0, false), "-0.000000e+00");
        assert_eq!(c_exp(1e100, false), "1.000000e+100");
        assert_eq!(c_exp(f64::NAN, false), "nan");
    }

    #[test]
    fn the_source_is_the_value_of_sr() {
        assert_eq!(trigger_source("EDGE,SR,C1,HT,OFF"), Some("C1"));
        assert_eq!(trigger_source("DROP,HT,TI,SR,EX"), Some("EX"));
        assert_eq!(trigger_source("EDGE,,SR,,C2"), Some("C2"), "as strtok has it");
        assert_eq!(trigger_source("EDGE,SR"), None);
        assert_eq!(trigger_source("EDGE"), None);
        assert_eq!(trigger_source("SR,C1"), None, "the first field is the type");
        assert_eq!(trigger_source("EDGE,SR,C1,SR,C2"), Some("C2"));
    }

    const MIN: Duration = Duration::from_secs(60);

    fn silence(t: Instant) -> Silence {
        Silence::with_bounds(t, MIN, 10 * MIN, MIN)
    }

    #[test]
    fn with_no_sample_a_silent_scope_fails_at_the_bound() {
        let t = Instant::now();
        let mut s = silence(t);
        s.asked(t, false);
        assert_eq!(s.nothing(t + MIN / 2), Verdict::Wait);
        assert_eq!(s.nothing(t + MIN), Verdict::Fail("No answer from the scope in 60 s.".into()));
    }

    #[test]
    fn after_a_sample_it_warns_and_goes_on_to_the_limit() {
        let t = Instant::now();
        let mut s = silence(t);
        s.sample();
        s.asked(t, false);
        assert_eq!(s.nothing(t + MIN / 2), Verdict::Wait);
        assert_eq!(
            s.nothing(t + MIN),
            Verdict::Say("The scope has been silent for 60 s: still waiting.".into())
        );
        assert_eq!(s.nothing(t + 2 * MIN), Verdict::Wait, "once");
        assert_eq!(
            s.nothing(t + 10 * MIN),
            Verdict::Fail("The scope has been silent for 600 s.".into())
        );
    }

    #[test]
    fn armed_it_warns_and_never_fails() {
        let t = Instant::now();
        let mut s = silence(t);
        s.asked(t, true);
        assert_eq!(
            s.nothing(t + MIN),
            Verdict::Say("No trigger in 60 s, or the scope is gone: still waiting.".into())
        );
        for hours in 1..1000 {
            assert_eq!(s.nothing(t + 3600 * hours * Duration::from_secs(1)), Verdict::Wait);
        }
    }

    #[test]
    fn answers_with_no_samples_are_bounded_on_samples_and_not_on_bytes() {
        let t = Instant::now();
        let mut s = silence(t);
        assert_eq!(s.no_samples(t), Verdict::Wait);
        assert_eq!(
            s.no_samples(t + MIN),
            Verdict::Say("The scope has sent no samples in 60 s: still asking.".into())
        );
        assert_eq!(s.no_samples(t + 2 * MIN), Verdict::Wait, "once");
        assert_eq!(
            s.no_samples(t + 10 * MIN),
            Verdict::Fail("The scope has sent no samples in 600 s.".into())
        );
        let mut s = silence(t);
        assert_eq!(s.no_samples(t), Verdict::Wait);
        s.heard(t + 5 * MIN);
        assert_eq!(
            s.no_samples(t + 9 * MIN),
            Verdict::Say("The scope has sent no samples in 540 s: still asking.".into()),
            "bytes aren't samples"
        );
        s.sample();
        assert_eq!(s.no_samples(t + 10 * MIN), Verdict::Wait, "a sample starts it over");
        assert_eq!(s.no_samples(t + 10 * MIN + MIN / 2), Verdict::Wait);
    }

    #[test]
    fn its_return_is_said_once() {
        let t = Instant::now();
        let mut s = silence(t);
        s.asked(t, true);
        assert!(matches!(s.nothing(t + MIN), Verdict::Say(_)));
        assert_eq!(s.heard(t + MIN), Some("The scope is sending again.".to_owned()));
        assert_eq!(s.heard(t + MIN), None);
    }

    #[test]
    fn a_block_cut_short_ends_at_its_pause_armed_or_not() {
        let t = Instant::now();
        let mut s = silence(t);
        s.sample();
        s.asked(t, true);
        s.heard(t);
        assert_eq!(s.nothing(t + MIN / 2), Verdict::Wait);
        assert_eq!(
            s.nothing(t + MIN),
            Verdict::Fail("The scope stopped within a block, 60 s ago.".into())
        );
    }

    #[test]
    fn a_new_question_starts_the_clock() {
        let t = Instant::now();
        let mut s = silence(t);
        s.asked(t + 10 * MIN, false);
        assert_eq!(s.nothing(t + 10 * MIN), Verdict::Wait);
        assert_eq!(s.nothing(t), Verdict::Wait, "a time ahead of the question is no time");
    }

    #[test]
    fn slow_timebases_widen_the_bounds() {
        let t = Instant::now();
        let mut s = Silence::new(t, 1000.0);
        assert_eq!(s.nothing(t + Duration::from_secs(19_999)), Verdict::Wait);
        assert!(matches!(s.nothing(t + Duration::from_secs(20_000)), Verdict::Fail(_)));
        let mut s = Silence::new(t, 1e-3);
        assert_eq!(s.nothing(t + Duration::from_secs(29)), Verdict::Wait);
        assert!(matches!(s.nothing(t + Duration::from_secs(30)), Verdict::Fail(_)));
        let mut s = Silence::new(t, 1e-3);
        s.sample();
        assert!(matches!(s.nothing(t + Duration::from_secs(30)), Verdict::Say(_)));
        assert!(matches!(s.nothing(t + Duration::from_secs(599)), Verdict::Wait));
        assert!(matches!(s.nothing(t + Duration::from_secs(600)), Verdict::Fail(_)));
    }
}
