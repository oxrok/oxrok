// Counterpart in sigrok:
//   libsigrok/src/hardware/siglent-sds/protocol.c,
//   libsigrok/src/hardware/siglent-sds/protocol.h,
//   Copyright (C) 2016 mhooijboer <marchelh@gmail.com>,
//   Copyright (C) 2012 Martin Ling <martin-git@earth.li>,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2013 Mathias Grimmberger <mgri@zaphod.sax.de>,
//   Copyright (C) 2018 mhooijboer <marchelh@gmail.com>,
//   with commits by Uwe Hermann, marchelh, Gerhard Sittig, fenugrec
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! What the scope says, with no scope: answers, blocks and how long it may be quiet.
//!
//! `C1:WF? ALL` answers `ALL,#9NNNNNNNNN`, a `LeCroy` `WAVEDESC` of 346 bytes, signed
//! bytes of samples and two line feeds. `Dn:WF? DAT2` answers `DAT2,#9NNNNNNNNN`, a bit
//! per sample and two line feeds. C takes 15 bytes off either, and nothing here does:
//! the guide's example takes 16 off the second (p.268, p.340), and its `#9` counts points,
//! a bit each.
//!
//! A sample has no check of its own. What a flipped bit in the header can do is what
//! [`Waveform::new`] says.

use crate::core::{Error, Result};
use std::time::{Duration, Instant};

/// `:INR?`: a trigger is ready.
pub const TRIG_READY: u32 = 8192;
/// `:INR?`: a signal was acquired.
pub const DATA_ACQUIRED: u32 = 1;
/// Where the descriptor's own length is: `WAVE_DESCRIPTOR`.
const DESC_LENGTH_AT: usize = 36;
/// Where the samples' length is: `WAVE_ARRAY_1`.
const ARRAY_LENGTH_AT: usize = 60;
/// What a descriptor must hold for those two.
pub const DESC_HEAD: usize = 64;
/// How far a block's `#` may be from an answer's start: `DAT2,` and `ALL,` are short.
const PREFIX_LONGEST: usize = 16;

/// `:INR?`: a decimal number or no answer, where C's `sr_atoi` leaves its value
/// unset and goes on with what the stack held.
/// The guide gives it as 0 to 65535 (p.175).
pub fn parse_inr(text: &str) -> Result<u32> {
    let t = text.trim();
    if !t.is_empty()
        && t.bytes().all(|b| b.is_ascii_digit())
        && let Ok(v) = t.parse::<u16>()
    {
        return Ok(u32::from(v));
    }
    Err(Error::device(format!("No usable :INR? answer: '{t}'.")))
}

/// Whether a signal was acquired: `:INR?`'s bit 0. The guide says the register
/// "only returns 0 bit and 13 bit" (p.175): 0 in single mode with no signal, 1 for a
/// signal acquired, 8192 for a trigger ready, 8193 for both (p.25, p.175-176). C takes
/// 8193 right after `ARM` for a trigger, 8192 for a wait and anything else for a scope
/// that did not arm, and in its wait any value but 0 for a trigger. Here only bit 0
/// says a waveform is there, and anything else is a wait.
pub fn acquired(inr: u32) -> bool {
    inr & DATA_ACQUIRED != 0
}

/// A number as `sr_atof_ascii` reads one: all of the text is the number.
fn number(text: &str) -> Option<f32> {
    text.trim().parse::<f32>().ok().filter(|v| v.is_finite())
}

/// `SANU? C1` on an SPO or non-SPO model. The guide gives `28Mpts` for the SPO models
/// and a plain number, `1600`, for the older ones (p.35).
///
/// C takes `Kpts` by 10000 (`protocol.c:916`), and anything with neither suffix as 0
/// samples, which makes a rate of 0 (`:918`). Here `Kpts` is 1000 and a number with or
/// without `pts` is that many, which is a reading of the guide's "others", not a capture
/// of one. Anything else says nothing: 0, as in C, with a word from the caller.
pub fn sample_points(text: &str) -> Result<Option<u64>> {
    let t = text.trim();
    let scaled = |digits: &str, factor: f32| {
        number(digits)
            .map(|v| Some((v * factor) as u64))
            .ok_or_else(|| Error::device(format!("Invalid float converted from '{t}'.")))
    };
    if let Some(digits) = t.strip_suffix("Mpts") {
        scaled(digits, 1e6)
    } else if let Some(digits) = t.strip_suffix("Kpts") {
        scaled(digits, 1e3)
    } else if let Some(v) = number(t.strip_suffix("pts").unwrap_or(t)) {
        Ok(Some(v as u64))
    } else {
        Ok(None)
    }
}

/// `SANU? C1` on an E series: `7.00E+05pts` (p.35). `CHDR OFF` suppresses units in
/// numbers (p.23), which may or may not take `pts` with it: either is taken.
pub fn sample_points_e(text: &str) -> Result<u64> {
    let t = text.trim();
    number(t.strip_suffix("pts").unwrap_or(t))
        .map(|v| v as u64)
        .ok_or_else(|| Error::device(format!("SCPI: not a number: '{}'", text.trim())))
}

/// A rate of `memory depth / (timebase * divisions)`, in C's floats.
pub fn sample_rate(depth: u64, timebase: f32, divisions: i32) -> f32 {
    if timebase > 0.0 { depth as f32 / (timebase * divisions as f32) } else { 0.0 }
}

/// What `TRSE?` says, as far as C reads it.
#[derive(Clone, Debug, PartialEq)]
pub struct TriggerSelect {
    /// The third field, stripped.
    pub source: String,
    /// It begins with `C`, on the field as it came: it has a level to ask for.
    pub is_channel: bool,
    /// What C makes of the fifth field.
    pub position: f32,
}

/// `TRSE?`: `EDGE,SR,C1,HT,OFF`.
///
/// Fewer than four fields is no answer, as in C. With four C dies on a `strlen` of the
/// missing fifth (`protocol.c:802`); here the position is 0.
pub fn trigger_select(text: &str) -> Result<TriggerSelect> {
    let fields: Vec<&str> = text.split(',').collect();
    if fields.len() < 4 {
        return Err(Error::device(format!("TRSE? response not according to spec: '{text}'.")));
    }
    Ok(TriggerSelect {
        source: fields[2].trim().to_owned(),
        is_channel: fields[2].starts_with('C'),
        position: fields.get(4).map_or(0.0, |f| trigger_position(f)),
    })
}

/// The fifth field of `TRSE?`, as `protocol.c:803-815` reads it. It is the kind of
/// hold-off on a real scope (`OFF`, `TI`), which has no unit and so comes to 0; a unit
/// has C's factors, its last two letters compared without case. `s` alone is compared
/// with two letters in C and never matches, so a time in seconds comes to 0 as well.
fn trigger_position(field: &str) -> f32 {
    let lower = field.to_ascii_lowercase();
    let Some(unit) = lower.get(lower.len().saturating_sub(2)..) else { return 0.0 };
    let factor = match unit {
        "us" => 1e9_f64,
        "ns" => 1e6,
        "ms" => 1e3,
        _ => return 0.0,
    };
    let digits = lower.len() - 2;
    (atof(&lower[..digits]) / factor) as f32
}

/// `atof`, for a field C reads with it: what starts as a number, else 0.
fn atof(text: &str) -> f64 {
    let t = text.trim_start();
    let end = t
        .char_indices()
        .take_while(|(i, c)| {
            c.is_ascii_digit()
                || *c == '.'
                || matches!(c, 'e' | 'E')
                || (matches!(c, '+' | '-')
                    && (*i == 0 || matches!(t.as_bytes()[*i - 1], b'e' | b'E')))
        })
        .last()
        .map_or(0, |(i, c)| i + c.len_utf8());
    (0..=end).rev().find_map(|n| t[..n].parse().ok()).unwrap_or(0.0)
}

/// A volt from a sample's code, in C's floats: `protocol.c:592-593`.
pub fn volts(code: u8, vdiv: f32, offset: f32) -> f32 {
    let voltage = f32::from(code as i8) / 25.0;
    (vdiv * voltage) - offset
}

/// The digits of a packet, from the scale: `protocol.c:596-597`.
pub fn digits(vdiv: f32) -> i32 {
    let log = vdiv.log10();
    -(log as i32) + i32::from(log < 0.0)
}

/// What a block's head holds: its prefix and the `#N` length.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockHead {
    /// Read more.
    Partial,
    /// The bytes it took, and the length it says.
    Whole {
        /// Bytes of the head, prefix included.
        taken: usize,
        /// What follows it, by its own word.
        length: usize,
    },
}

/// Looks for the `#N` at the head of an answer. What holds no `#` soon enough is no
/// block, a number's answer read late for one, say.
pub fn block_head(bytes: &[u8]) -> Result<BlockHead> {
    let junk = || {
        Error::device(format!(
            "No usable waveform: the answer starts '{}'.",
            String::from_utf8_lossy(&bytes[..bytes.len().min(PREFIX_LONGEST)])
                .escape_debug()
                .collect::<String>()
        ))
    };
    let Some(at) = bytes.iter().position(|b| *b == b'#') else {
        return if bytes.len() > PREFIX_LONGEST || bytes.contains(&b'\n') {
            Err(junk())
        } else {
            Ok(BlockHead::Partial)
        };
    };
    if at > PREFIX_LONGEST {
        return Err(junk());
    }
    let Some(digit) = bytes.get(at + 1) else { return Ok(BlockHead::Partial) };
    if !(b'1'..=b'9').contains(digit) {
        return Err(junk());
    }
    let count = usize::from(digit - b'0');
    let Some(field) = bytes.get(at + 2..at + 2 + count) else { return Ok(BlockHead::Partial) };
    if !field.iter().all(u8::is_ascii_digit) {
        return Err(junk());
    }
    let length = std::str::from_utf8(field).ok().and_then(|f| f.parse().ok()).ok_or_else(junk)?;
    Ok(BlockHead::Whole { taken: at + 2 + count, length })
}

/// What a waveform's descriptor and head say of what follows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Waveform {
    /// Bytes from the start of the descriptor to the first sample.
    pub descriptor: usize,
    /// Samples to take.
    pub samples: usize,
    /// The descriptor and the head disagree: `samples` is the less of the two.
    pub disagrees: bool,
}

impl Waveform {
    /// From the head's length and the first [`DESC_HEAD`] bytes of the descriptor.
    ///
    /// C reads the two fields and believes them, and a flip there sets how many
    /// bytes it asks for. Here the descriptor must fit the head's length, and when
    /// `descriptor + samples` isn't that length it takes the less, with a warning from
    /// the caller. A flip in a length then costs samples and says so: it can't add
    /// any. That a head counts just those two is from the model and the `LeCroy` format,
    /// no recording, which is why a difference only warns.
    ///
    /// What this can't catch: a flip in a sample, as nothing covers them, and one in
    /// the 282 descriptor bytes no one reads.
    pub fn new(length: usize, head: &[u8]) -> Result<Waveform> {
        let field = |at: usize| -> Result<usize> {
            let bytes = head
                .get(at..at + 4)
                .ok_or_else(|| Error::device("No usable waveform: its descriptor is short."))?;
            Ok(u32::from_le_bytes(bytes.try_into().expect("four bytes")) as usize)
        };
        let descriptor = field(DESC_LENGTH_AT)?;
        let array = field(ARRAY_LENGTH_AT)?;
        if descriptor < DESC_HEAD || descriptor > length {
            return Err(Error::device(format!(
                "No usable waveform: a descriptor of {descriptor} bytes in a block of {length}."
            )));
        }
        let room = length - descriptor;
        Ok(Waveform { descriptor, samples: array.min(room), disagrees: array != room })
    }
}

/// One channel's digital bits into the samples kept so far: `protocol.c:383-411`. A bit
/// per sample, the low bit of each byte first, on bit `index` of a 16 bit sample.
///
/// C loops over `depth` bytes of what may be `depth / 8` and keeps `depth` samples, so
/// it reads past what came, keeps the right ones and has a heap's luck. It takes 15
/// bytes off a head that has 16 (p.268), so its first 8
/// samples are the head's last digit. Neither is copied.
pub fn merge_digital(samples: &mut [u16], data: &[u8], index: usize) {
    for (i, sample) in samples.iter_mut().enumerate() {
        let Some(byte) = data.get(i / 8) else { return };
        if (byte >> (i % 8)) & 1 != 0 {
            *sample |= 1 << index;
        }
    }
}

/// What a wait comes to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quiet {
    /// Go on.
    Wait,
    /// Say so, once, and go on.
    Warn,
    /// It is over.
    GiveUp,
}

/// How long a scope may say nothing, as a rule over times given to it.
///
/// A scope that has sent no sample in this capture may fail at `first`: the longest
/// a block's preparation takes by C's own sleeps, with room. No manual gives a figure;
/// C's bound is `TRANSFER_TIMEOUT` of 1 s on USB and none at all on TCP. After a
/// sample it warns at `first` and goes on to `later`, ten minutes, as a slow scope with
/// a long record is no dead one.
///
/// Time the thread did not run comes off, up to [`Silence::CREDIT`] of a `first`, and
/// time in a send to a slow client isn't silence: set [`Silence::heard`] after it.
#[derive(Clone, Debug)]
pub struct Silence {
    first: Duration,
    later: Duration,
    heard: Instant,
    last_look: Instant,
    credit: Duration,
    spent: Duration,
    sampled: bool,
    warned: bool,
    /// The longest one empty read may take before the rest counts as a stall.
    read: Duration,
}

impl Silence {
    /// How much of `first` a stalled host may be excused.
    pub const CREDIT: u32 = 1;

    /// From `now`.
    pub fn new(now: Instant, first: Duration, later: Duration, read: Duration) -> Silence {
        Silence {
            first,
            later,
            heard: now,
            last_look: now,
            credit: Duration::ZERO,
            spent: Duration::ZERO,
            sampled: false,
            warned: false,
            read,
        }
    }

    /// The scope answered, at `now`: the time after any send to a client.
    pub fn heard(&mut self, now: Instant) {
        self.heard = now;
        self.last_look = now;
        self.credit = Duration::ZERO;
        self.warned = false;
    }

    /// A sample came. From here on silence only warns until `later`.
    pub fn sampled(&mut self, now: Instant) {
        self.sampled = true;
        self.heard(now);
    }

    /// Whether a sample has come.
    pub fn has_sampled(&self) -> bool {
        self.sampled
    }

    /// A new wait with a first bound of its own, as a block's preparation has.
    pub fn set_first(&mut self, first: Duration) {
        self.first = first;
    }

    /// The bound in force.
    pub fn bound(&self) -> Duration {
        self.first
    }

    /// A read that began at `began` found nothing. `began` is taken ahead of the read,
    /// so a stall between the read and this call doesn't count as silence with the
    /// answer already in.
    pub fn nothing(&mut self, began: Instant) -> Quiet {
        // A stall shows as more time since the last look than a read can take.
        let gap = began.saturating_duration_since(self.last_look);
        let allowed = self.first * Silence::CREDIT;
        if gap > self.read * 2 && self.spent < allowed {
            let excuse = gap.saturating_sub(self.read * 2).min(allowed.saturating_sub(self.spent));
            self.credit += excuse;
            self.spent += excuse;
        }
        self.last_look = began;
        let quiet = began.saturating_duration_since(self.heard).saturating_sub(self.credit);
        if !self.sampled {
            return if quiet >= self.first { Quiet::GiveUp } else { Quiet::Wait };
        }
        if quiet >= self.later {
            Quiet::GiveUp
        } else if quiet >= self.first && !self.warned {
            self.warned = true;
            Quiet::Warn
        } else {
            Quiet::Wait
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inr_is_a_number_or_nothing() {
        assert_eq!(parse_inr("8193\n").unwrap(), 8193);
        assert_eq!(parse_inr(" 0").unwrap(), 0);
        assert_eq!(parse_inr("65535").unwrap(), 65535);
        for bad in ["", "OFF", "8193X", "-1", "+5", "1.5", "7.00E+05", "65536", "99999999999"] {
            let e = parse_inr(bad).unwrap_err().to_string();
            assert!(e.contains("No usable :INR? answer"), "{bad}: {e}");
        }
    }

    #[test]
    fn only_bit_0_says_a_signal_was_acquired() {
        assert!(acquired(8193));
        assert!(acquired(1));
        assert!(!acquired(8192), "a trigger ready is no waveform yet");
        assert!(!acquired(0), "single mode with no signal");
        assert!(!acquired(2));
    }

    #[test]
    fn sample_points_mean_what_they_say() {
        assert_eq!(sample_points("7.00Mpts").unwrap(), Some(7_000_000));
        assert_eq!(sample_points("14.00Mpts").unwrap(), Some(14_000_000));
        assert_eq!(sample_points("70.00Kpts").unwrap(), Some(70_000), "C makes 700000 of it");
        assert_eq!(sample_points("700pts").unwrap(), Some(700), "C makes 0 of it");
        assert_eq!(sample_points("1600").unwrap(), Some(1600), "the guide's older models");
        assert_eq!(sample_points("7.00E+05").unwrap(), Some(700_000));
        assert_eq!(sample_points("odd").unwrap(), None);
        assert!(sample_points("x.yMpts").is_err());
        assert!(sample_points("Kpts").is_err());
        assert_eq!(sample_points_e("7.00E+05").unwrap(), 700_000);
        assert_eq!(sample_points_e("1.40E+07").unwrap(), 14_000_000);
        assert_eq!(sample_points_e("7.00E+05pts").unwrap(), 700_000, "p.35");
        for bad in ["7.00Mpts", "", "nan", "inf"] {
            assert!(sample_points_e(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn a_rate_of_nothing_is_nothing() {
        assert_eq!(sample_rate(7000, 5e-4, 14), 1_000_000.0);
        assert_eq!(sample_rate(7000, 0.0, 14), 0.0);
        assert_eq!(sample_rate(7000, -1.0, 14), 0.0);
        assert_eq!(sample_rate(0, 5e-4, 14), 0.0);
    }

    #[test]
    fn trigger_select_reads_as_c_does() {
        let t = trigger_select("EDGE,SR,C1,HT,OFF").unwrap();
        assert_eq!(t.source, "C1");
        assert!(t.is_channel);
        assert_eq!(t.position, 0.0);
        let t = trigger_select("EDGE,SR, EX ,HT,TI").unwrap();
        assert_eq!(t.source, "EX");
        assert!(!t.is_channel, "the field as it came");
        assert_eq!(trigger_select("EDGE,SR,LINE,HT").unwrap().position, 0.0, "C dies here");
        let e = trigger_select("EDGE,SR,C1").unwrap_err().to_string();
        assert!(e.contains("TRSE? response not according to spec: 'EDGE,SR,C1'"), "{e}");
    }

    #[test]
    fn a_trigger_position_has_cs_factors() {
        let at = |f: &str| trigger_select(&format!("EDGE,SR,C1,HT,{f}")).unwrap().position;
        assert_eq!(at("200US"), (200.0_f64 / 1e9) as f32);
        assert_eq!(at("200ns"), (200.0_f64 / 1e6) as f32);
        assert_eq!(at("200MS"), (200.0_f64 / 1e3) as f32);
        assert_eq!(at("200S"), 0.0, "C compares two letters with 's'");
        assert_eq!(at("xUS"), 0.0, "what atof can't read");
        assert_eq!(at("1.5e2us"), (150.0_f64 / 1e9) as f32);
        assert_eq!(at("s"), 0.0);
        assert_eq!(at("é"), 0.0);
    }

    #[test]
    fn a_code_makes_a_volt_as_in_c() {
        assert_eq!(volts(25, 1.0, 0.0), 1.0);
        assert_eq!(volts(0xe7, 1.0, 0.0), -1.0, "signed");
        assert_eq!(volts(25, 2.0, 0.5), 1.5);
        assert_eq!(volts(0x80, 1.0, 0.0), -128.0 / 25.0);
        assert_eq!(digits(1.0), 0);
        assert_eq!(digits(0.5), 1);
        assert_eq!(digits(0.0005), 4);
        assert_eq!(digits(50.0), -1);
    }

    #[test]
    fn a_block_head_is_found_or_refused() {
        assert_eq!(block_head(b"ALL,").unwrap(), BlockHead::Partial);
        assert_eq!(block_head(b"ALL,#").unwrap(), BlockHead::Partial);
        assert_eq!(block_head(b"ALL,#9").unwrap(), BlockHead::Partial);
        assert_eq!(block_head(b"ALL,#900000").unwrap(), BlockHead::Partial);
        assert_eq!(
            block_head(b"ALL,#9000000746WAVE").unwrap(),
            BlockHead::Whole { taken: 15, length: 746 }
        );
        assert_eq!(
            block_head(b"DAT2,#9000000002ab").unwrap(),
            BlockHead::Whole { taken: 16, length: 2 }
        );
        assert_eq!(block_head(b"#14abcd").unwrap(), BlockHead::Whole { taken: 3, length: 4 });
        for junk in [
            &b"7.00E+05\n"[..],
            b"ALL,#0123",
            b"ALL,#A123",
            b"ALL,#3x12",
            b"0123456789abcdefghij",
            b"0123456789abcdefgh#90",
        ] {
            let e = block_head(junk).unwrap_err().to_string();
            assert!(e.contains("No usable waveform"), "{e}");
        }
    }

    fn descriptor(desc: u32, array: u32) -> Vec<u8> {
        let mut d = vec![0u8; DESC_HEAD];
        d[36..40].copy_from_slice(&desc.to_le_bytes());
        d[60..64].copy_from_slice(&array.to_le_bytes());
        d
    }

    #[test]
    fn a_waveform_believes_the_less_of_two_lengths() {
        let w = Waveform::new(1046, &descriptor(346, 700)).unwrap();
        assert_eq!(w, Waveform { descriptor: 346, samples: 700, disagrees: false });
        let w = Waveform::new(1046, &descriptor(346, 800)).unwrap();
        assert_eq!(w, Waveform { descriptor: 346, samples: 700, disagrees: true });
        let w = Waveform::new(1046, &descriptor(346, 100)).unwrap();
        assert_eq!(w, Waveform { descriptor: 346, samples: 100, disagrees: true });
        for (len, desc) in [(1046, 2000), (1046, 10)] {
            let e = Waveform::new(len, &descriptor(desc, 1)).unwrap_err().to_string();
            assert!(e.contains("No usable waveform: a descriptor of"), "{e}");
        }
        assert!(Waveform::new(1046, &[0; 10]).is_err());
    }

    /// Every single bit flipped in what is read of a waveform's head: how many give
    /// wrong samples with no word. This is the sum the report states.
    #[test]
    fn a_flip_in_the_head_says_so_or_costs_nothing() {
        let mut head = b"ALL,#9000001046".to_vec();
        head.extend(descriptor(346, 700));
        let (mut clean, mut said, mut silent_wrong) = (0, 0, 0);
        for bit in 0..head.len() * 8 {
            let mut h = head.clone();
            h[bit / 8] ^= 1 << (bit % 8);
            let outcome = block_head(&h).and_then(|b| match b {
                BlockHead::Partial => Err(Error::device("partial")),
                BlockHead::Whole { taken, length } => {
                    let d = h.get(taken..taken + DESC_HEAD).unwrap_or(&[]);
                    Ok((taken, Waveform::new(length, d)?))
                }
            });
            match outcome {
                Err(_) | Ok((_, Waveform { disagrees: true, .. })) => said += 1,
                Ok((15, Waveform { descriptor: 346, samples: 700, disagrees: false })) => {
                    clean += 1;
                }
                Ok(_) => silent_wrong += 1,
            }
        }
        println!("flips in the head: {said} said, {clean} changed nothing");
        assert_eq!(silent_wrong, 0, "a flip that moves samples and says nothing");
        assert_eq!(said + clean, head.len() * 8);
        assert!(said >= 30, "the lengths are looked at: {said}");
    }

    #[test]
    fn digital_bits_land_on_their_channel() {
        let mut s = vec![0u16; 10];
        merge_digital(&mut s, &[0b0000_0101, 0b0000_0010], 0);
        merge_digital(&mut s, &[0b1000_0000, 0b0000_0001], 9);
        assert_eq!(s, [1, 0, 1, 0, 0, 0, 0, 0x200, 0x200, 1]);
        let mut s = vec![0u16; 16];
        merge_digital(&mut s, &[0xff], 15);
        assert_eq!(s[7], 0x8000);
        assert_eq!(s[8], 0, "what never came stays low");
    }

    fn rule(now: Instant) -> Silence {
        Silence::new(now, Duration::from_secs(30), Duration::from_secs(600), Duration::from_secs(1))
    }

    /// Looks every second, with no stall, for `secs`: what it said, and when.
    fn watch(s: &mut Silence, from: Instant, secs: u64) -> Vec<(u64, Quiet)> {
        (1..=secs)
            .map(|t| (t, s.nothing(from + Duration::from_secs(t))))
            .filter(|(_, q)| *q != Quiet::Wait)
            .collect()
    }

    #[test]
    fn with_no_sample_it_fails_at_the_first_bound_and_no_sooner() {
        let t0 = Instant::now();
        let mut s = rule(t0);
        let said = watch(&mut s, t0, 31);
        assert_eq!(said[0], (30, Quiet::GiveUp));
    }

    #[test]
    fn after_a_sample_it_warns_once_and_fails_at_the_long_limit() {
        let t0 = Instant::now();
        let mut s = rule(t0);
        s.sampled(t0);
        assert!(s.has_sampled());
        let said = watch(&mut s, t0, 601);
        assert_eq!(said[0], (30, Quiet::Warn));
        assert_eq!(said[1], (600, Quiet::GiveUp));
    }

    #[test]
    fn hearing_starts_it_over_and_can_warn_again() {
        let t0 = Instant::now();
        let mut s = rule(t0);
        s.sampled(t0);
        assert_eq!(watch(&mut s, t0, 30).last(), Some(&(30, Quiet::Warn)));
        let t1 = t0 + Duration::from_secs(31);
        s.heard(t1);
        assert_eq!(watch(&mut s, t1, 29), Vec::new());
        assert_eq!(watch(&mut s, t1 + Duration::from_secs(29), 1), [(1, Quiet::Warn)]);
    }

    #[test]
    fn a_bound_spans_many_bounds_when_something_keeps_coming() {
        let t0 = Instant::now();
        let mut s = rule(t0);
        for i in 1..100 {
            let t = t0 + Duration::from_secs(i * 25);
            assert_eq!(s.nothing(t), Quiet::Wait);
            s.heard(t);
        }
    }

    #[test]
    fn a_stall_of_the_host_is_excused_up_to_a_limit() {
        let t0 = Instant::now();
        let mut s = rule(t0);
        // 40 s stopped between two looks, right after hearing.
        assert_eq!(s.nothing(t0 + Duration::from_secs(40)), Quiet::Wait, "not the scope's");
        // Stopped again and again: the credit runs out, and it ends.
        let mut t = t0 + Duration::from_secs(40);
        let mut ended = false;
        for _ in 0..50 {
            t += Duration::from_secs(40);
            if s.nothing(t) == Quiet::GiveUp {
                ended = true;
                break;
            }
        }
        assert!(ended, "a host stopped for ever still ends");
    }

    #[test]
    fn a_time_past_the_look_is_no_silence() {
        // `began` was taken ahead of the read: a look at t=5 is t=5, whatever it is now.
        let t0 = Instant::now();
        let mut s = rule(t0);
        assert_eq!(s.nothing(t0 + Duration::from_secs(5)), Quiet::Wait);
    }

    #[test]
    fn the_first_bound_can_be_set_for_a_long_preparation() {
        let t0 = Instant::now();
        let mut s = rule(t0);
        s.set_first(Duration::from_secs(105));
        assert_eq!(s.bound(), Duration::from_secs(105));
        assert_eq!(watch(&mut s, t0, 104), Vec::new());
    }
}
