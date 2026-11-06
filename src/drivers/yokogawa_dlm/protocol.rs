// Counterpart in sigrok:
//   libsigrok/src/hardware/yokogawa-dlm/protocol.c,
//   libsigrok/src/hardware/yokogawa-dlm/protocol.h,
//   libsigrok/src/hardware/yokogawa-dlm/protocol_wrappers.c,
//   Copyright (C) 2014 abraxa (Soeren Apel) <soeren@apelpie.net>,
//   with commits by Uwe Hermann, Soeren Apel, Bert Vermeulen, Gerhard Sittig,
//   Martin Ling, Daniel Elstner, Stefan Brüns
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The tables of `protocol.c`, what its answers may hold, and the rules of a capture's
//! waits, all pure; the wire goes through [`crate::scpi`], in `protocol_wrappers.c`'s
//! words.
//!
//! The commands are the DLM2000's, from IM 710105-17E, the communication interface
//! manual that `protocol_wrappers.c` names:
//!
//! - settings are `:CHANNEL1:VDIV 5.000000E-01`, and every one but a channel's display
//!   is followed by `*OPC?`, the vertical division and the coupling by two;
//! - `:WAVEFORM:SEND? 1`, after `:WAVEFORM:FORMAT BYTE`, `:WAVEFORM:RECORD 0`,
//!   `:WAVEFORM:START 0`, `:WAVEFORM:END 124999999` and `:WAVEFORM:TRACE`, answers
//!   `#8`, eight digits, signed bytes and a line end. Volts are range times the byte
//!   over 12.5, plus offset (p. 5-290);
//! - every waveform comes in one answer, however long, so a whole frame is in memory
//!   between the answer's end and its packets, as in C. Only the samples the frame
//!   needs are kept, and nothing is reserved from a count the instrument gave.

use std::time::{Duration, Instant};

/// `DLM_DIVISION_FOR_BYTE_FORMAT`.
const DIVISION_FOR_BYTE_FORMAT: f64 = 12.5;

/// `protocol.c`'s `dlm_timebases`.
#[rustfmt::skip]
pub const TIMEBASES: [(u64, u64); 36] = [
    (1, 1_000_000_000), (2, 1_000_000_000), (5, 1_000_000_000),
    (10, 1_000_000_000), (20, 1_000_000_000), (50, 1_000_000_000),
    (100, 1_000_000_000), (200, 1_000_000_000), (500, 1_000_000_000),
    (1, 1_000_000), (2, 1_000_000), (5, 1_000_000),
    (10, 1_000_000), (20, 1_000_000), (50, 1_000_000),
    (100, 1_000_000), (200, 1_000_000), (500, 1_000_000),
    (1, 1000), (2, 1000), (5, 1000),
    (10, 1000), (20, 1000), (50, 1000),
    (100, 1000), (200, 1000), (500, 1000),
    (1, 1), (2, 1), (5, 1),
    (10, 1), (20, 1), (50, 1),
    (100, 1), (200, 1), (500, 1),
];

/// `protocol.c`'s `dlm_vdivs`.
#[rustfmt::skip]
pub const VDIVS: [(u64, u64); 17] = [
    (2, 1000), (5, 1000), (10, 1000), (20, 1000), (50, 1000),
    (100, 1000), (200, 1000), (500, 1000),
    (1, 1), (2, 1), (5, 1), (10, 1), (20, 1), (50, 1), (100, 1), (200, 1), (500, 1),
];

/// `protocol.c`'s `coupling_options`.
pub const COUPLINGS: [&str; 4] = ["AC", "DC", "DC50", "GND"];
/// `dlm_trigger_slopes`.
pub const SLOPES: [&str; 2] = ["r", "f"];
const SOURCES_2CH: [&str; 4] = ["1", "2", "LINE", "EXT"];
const SOURCES_4CH: [&str; 14] = [
    "1", "2", "3", "4", "LINE", "EXT", "BIT1", "BIT2", "BIT3", "BIT4", "BIT5", "BIT6", "BIT7",
    "BIT8",
];
const NAMES_8: [&str; 8] = ["D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7"];
#[rustfmt::skip]
const NAMES_32: [&str; 32] = [
    "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7",
    "B0", "B1", "B2", "B3", "B4", "B5", "B6", "B7",
    "C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7",
    "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7",
];

/// A row of `protocol.c`'s `scope_models`.
#[derive(Debug)]
pub struct Config {
    /// `*IDN?`'s model and the name each goes by.
    pub ids: &'static [(&'static str, &'static str)],
    /// Analog channels, named `1` to `4`.
    pub analog: usize,
    /// Logic channels.
    pub digital: usize,
    /// Groups of logic channels.
    pub pods: usize,
    /// Names of the logic channels.
    pub digital_names: &'static [&'static str],
    /// What the trigger may be set to.
    pub sources: &'static [&'static str],
    /// Divisions across, and down.
    pub xdivs: i32,
    /// Divisions down.
    pub ydivs: i32,
}

/// `protocol.c`'s `scope_models`.
pub const MODELS: [Config; 5] = [
    Config {
        ids: &[("710105", "DLM2022"), ("710115", "DLM2032"), ("710125", "DLM2052")],
        analog: 2,
        digital: 0,
        pods: 0,
        digital_names: &NAMES_8,
        sources: &SOURCES_2CH,
        xdivs: 10,
        ydivs: 8,
    },
    Config {
        ids: &[("710110", "DLM2024"), ("710120", "DLM2034"), ("710130", "DLM2054")],
        analog: 4,
        digital: 8,
        pods: 1,
        digital_names: &NAMES_8,
        sources: &SOURCES_4CH,
        xdivs: 10,
        ydivs: 8,
    },
    Config {
        ids: &[
            ("701307", "DL9040"),
            ("701308", "DL9040L"),
            ("701310", "DL9140"),
            ("701311", "DL9140L"),
            ("701312", "DL9240"),
            ("701313", "DL9240L"),
        ],
        analog: 4,
        digital: 0,
        pods: 0,
        digital_names: &[],
        sources: &SOURCES_4CH,
        xdivs: 10,
        ydivs: 8,
    },
    Config {
        ids: &[("701320", "DL9505L"), ("701321", "DL9510L")],
        analog: 4,
        digital: 16,
        pods: 4,
        digital_names: &NAMES_32,
        sources: &SOURCES_4CH,
        xdivs: 10,
        ydivs: 8,
    },
    Config {
        ids: &[("701330", "DL9705L"), ("701331", "DL9710L")],
        analog: 4,
        digital: 32,
        pods: 4,
        digital_names: &NAMES_32,
        sources: &SOURCES_4CH,
        xdivs: 10,
        ydivs: 8,
    },
];

/// The row and the name of a `*IDN?` model: `dlm_model_get`.
pub fn model_get(id: &str) -> Option<(&'static Config, &'static str)> {
    MODELS.iter().find_map(|config| {
        config.ids.iter().find(|(known, _)| *known == id).map(|(_, name)| (config, *name))
    })
}

/// A number from the instrument. As `sr_atof_ascii` for what the manual's numbers look
/// like; where C makes a reading of anything else, here it is none: an empty answer
/// (0 there), `inf`, `nan`, what overflows a float, and hexadecimal.
pub fn number(text: &str) -> Option<f32> {
    let value: f64 = text.trim_start().parse().ok()?;
    let narrow = value as f32;
    narrow.is_finite().then_some(narrow)
}

/// `:WAVEFORM:LENGTH?`: a count, where `sr_atol` also takes a sign, and `*response =
/// tmp` wraps what doesn't fit (`protocol_wrappers.c:289`).
pub fn length(text: &str) -> Option<u32> {
    text.trim().parse().ok()
}

/// `array_float_get`: the table's index of `2.000E-03`.
///
/// It keeps C's way, truncating the significand, so `2.900E-03` reads as 2 mV. Where it
/// leaves C:
///
/// - No `E`, or a significand of ten bytes or more, is no value here. There C reads
///   past the end, or writes past `ss` (`protocol.c:336-337`).
/// - An exponent is a sign and two digits at most. C takes three bytes, so `1.000E-003`
///   is 1 s there, and an empty one is 0, so `2.000E` is 2 s.
/// - A positive exponent of 3 or more is no entry. C drops the sign (`abs(e)`,
///   `protocol.c:345`), so `2.000E+03` reads there as `2.000E-03`: 2 kV a division is
///   2 mV.
pub fn scaled_index(value: &str, table: &[(u64, u64)]) -> Option<usize> {
    let pos = value.find('E')?;
    let (significand, exponent) = (&value[..pos], &value[pos + 1..]);
    if significand.len() >= 10 || exponent.is_empty() || exponent.len() > 3 {
        return None;
    }
    let mut s = number(significand)?;
    let e: i32 = exponent.parse().ok()?;
    let mut power = e.unsigned_abs();
    if power > 18 || (e > 0 && power >= 3) {
        return None;
    }
    // f is a multiple of 1000: `s *= 10` moves it up or down to the nearest.
    while !power.is_multiple_of(3) {
        s *= 10.0;
        if e < 0 {
            power += 1;
        } else {
            power -= 1;
        }
    }
    if power > 18 {
        return None;
    }
    let f = 10u64.pow(power);
    // Truncated "to circumvent rounding errors", as C says.
    let s_int = s as u32;
    table.iter().position(|entry| u64::from(s_int) == entry.0 && f == entry.1)
}

/// C's `(float)p / q`, printed with `%E`.
pub fn table_text(entry: (u64, u64)) -> String {
    format_e(f64::from(entry.0 as f32 / entry.1 as f32))
}

/// `%E`: six decimals, and an exponent with its sign and at least two digits.
pub fn format_e(value: f64) -> String {
    let text = format!("{value:.6E}");
    let (mantissa, exponent) = text.split_once('E').unwrap_or((&text, "0"));
    let (sign, digits) = exponent.strip_prefix('-').map_or(('+', exponent), |d| ('-', d));
    format!("{mantissa}E{sign}{digits:0>2}")
}

/// The seconds a trigger position in `0..=1` comes to, for `:TRIGGER:DELAY:TIME`.
pub fn delay_for_position(position: f64, timebase: (u64, u64), xdivs: i32) -> f64 {
    -(position - 0.5) * (timebase.0 as f64 / timebase.1 as f64) * f64::from(xdivs)
}

/// The position a `:TRIGGER:DELAY:TIME?` comes to, in C's floats.
pub fn position_for_delay(delay: f32, timebase: (u64, u64), xdivs: i32) -> f32 {
    let mut position =
        (f64::from(delay) / ((timebase.0 as f64 / timebase.1 as f64) * f64::from(xdivs))) as f32;
    position = (f64::from(position) - 0.5) as f32;
    -position
}

/// `:WAVEFORM:SRATE?`'s float as a rate, `None` when it can't be one: C's conversion of
/// that is undefined.
pub fn rate(value: f32) -> Option<u64> {
    (0.0..1.8e19).contains(&value).then_some(value as u64)
}

/// One byte's volts, in C's order of float and double arithmetic
/// (`dlm_analog_samples_send`): `range * voltage` is a float, the division a double.
pub fn volts(byte: u8, range: f32, offset: f32) -> f32 {
    let voltage = f32::from(byte as i8);
    (f64::from(range * voltage) / DIVISION_FOR_BYTE_FORMAT + f64::from(offset)) as f32
}

/// A block's header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    /// `#`, the digit, and the digits.
    pub len: usize,
    /// What it says follows.
    pub bytes: usize,
}

/// What the start of an answer says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Head {
    /// Not enough to tell yet.
    Short,
    /// A header.
    Good(Header),
    /// No header.
    Bad,
}

/// `dlm_block_data_header_process`, on what has come so far. Only `#` and decimal
/// digits: `sr_atoi` also takes spaces and a sign, and C's count of digits runs past
/// its buffer on any byte above `9` (`protocol.c:875-879`). `#0` has no digits, and is
/// 0 bytes, as in C.
pub fn block_head(data: &[u8]) -> Head {
    let Some(first) = data.first() else { return Head::Short };
    if *first != b'#' {
        return Head::Bad;
    }
    let Some(count) = data.get(1) else { return Head::Short };
    if !count.is_ascii_digit() {
        return Head::Bad;
    }
    let digits = usize::from(count - b'0');
    let Some(text) = data.get(2..2 + digits) else {
        // A digit that can't be one is already known to be wrong.
        return if data[2..].iter().all(u8::is_ascii_digit) { Head::Short } else { Head::Bad };
    };
    if !text.iter().all(u8::is_ascii_digit) {
        return Head::Bad;
    }
    let bytes = text.iter().fold(0usize, |n, d| n * 10 + usize::from(d - b'0'));
    Head::Good(Header { len: 2 + digits, bytes })
}

/// How long a capture waits for the scope.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bounds {
    /// Silence before the warning.
    pub warn: Duration,
    /// Silence that ends a capture that has had no sample.
    pub first: Duration,
    /// Silence that ends one that has.
    pub later: Duration,
    /// A pause in a block on `tcp-raw`, where nothing else says it ended, that does.
    pub gap: Duration,
    /// How long a scope may send empty blocks through `:STOP`.
    pub empty: Duration,
    /// The pause between two requests after an empty block.
    pub empty_pause: Duration,
}

impl Default for Bounds {
    /// The manual gives no time for a `:WAVEFORM:SEND?`, and says it doesn't wait for an
    /// acquisition (p. 4-9), so neither a sweep's 5000 s nor anything else sets these:
    /// they are choices, wide for the 250 Mpoints of the largest record. C has no bound
    /// at all, on USBTMC or on TCP.
    fn default() -> Bounds {
        Bounds {
            warn: Duration::from_secs(10),
            first: Duration::from_secs(120),
            later: Duration::from_secs(600),
            gap: Duration::from_secs(5),
            empty: Duration::from_secs(10),
            empty_pause: Duration::from_millis(100),
        }
    }
}

/// What an empty read comes to.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Quiet {
    /// Go on waiting.
    Wait,
    /// Go on waiting, and say this once.
    Warn(String),
    /// The capture ends, for this reason.
    GiveUp(String),
}

/// The rule of a capture's silence, over times given as arguments.
///
/// Time is taken ahead of the read that found nothing, so an answer that lies ready
/// wins, and a stall between the two doesn't pass for silence. It runs from the send
/// of a request, taken after the send returns, and from each byte, so a slow client
/// never reads as a silent scope: a wait on the packet queue lies ahead of a send.
#[derive(Clone, Debug)]
pub struct Silence {
    bounds: Bounds,
    heard: Instant,
    sample_given: bool,
    warned: bool,
}

impl Silence {
    /// A capture whose scope was last heard at `now`.
    pub fn new(bounds: Bounds, now: Instant) -> Silence {
        Silence { bounds, heard: now, sample_given: false, warned: false }
    }

    /// A request went out, and the send returned, at `now`.
    pub fn asked(&mut self, now: Instant) {
        self.heard = now;
    }

    /// A byte came at `now`. `Some` holds what to say, when the scope had been warned
    /// about.
    pub fn heard(&mut self, now: Instant) -> Option<&'static str> {
        self.heard = now;
        std::mem::take(&mut self.warned).then_some("The scope answers again.")
    }

    /// A sample went out: from now on silence only warns, up to the long limit.
    pub fn sample_given(&mut self) {
        self.sample_given = true;
    }

    /// A read asked at `asked` found nothing, and bytes had come: is the block over?
    pub fn gap_over(&self, asked: Instant) -> bool {
        asked.saturating_duration_since(self.heard) >= self.bounds.gap
    }

    /// A read asked at `asked` found nothing.
    pub fn quiet(&mut self, asked: Instant) -> Quiet {
        let silent = asked.saturating_duration_since(self.heard);
        let limit = if self.sample_given { self.bounds.later } else { self.bounds.first };
        if silent >= limit {
            return Quiet::GiveUp(format!(
                "No answer from the scope in {} s.",
                limit.as_secs_f32()
            ));
        }
        if silent >= self.bounds.warn && !std::mem::replace(&mut self.warned, true) {
            return Quiet::Warn(format!(
                "No answer from the scope in {} s; still waiting.",
                self.bounds.warn.as_secs_f32()
            ));
        }
        Quiet::Wait
    }
}

/// The rule of empty blocks: well-formed, so no silence, but they make nothing. C goes
/// on for ever, in the path where it doesn't crash.
#[derive(Clone, Debug)]
pub struct Empties {
    limit: Duration,
    since: Option<Instant>,
}

impl Empties {
    /// No empty block yet.
    pub fn new(bounds: &Bounds) -> Empties {
        Empties { limit: bounds.empty, since: None }
    }

    /// An empty block came at `now`. What it comes to: `Warn` the first time, with C's
    /// words, and `GiveUp` once they have gone on for too long.
    pub fn empty(&mut self, now: Instant) -> Quiet {
        match self.since {
            None => {
                self.since = Some(now);
                Quiet::Warn(
                    "Zero-length waveform data packet received. Live mode not supported yet, \
                     stopping acquisition and retrying."
                        .to_owned(),
                )
            }
            Some(since) if now.saturating_duration_since(since) >= self.limit => {
                Quiet::GiveUp(format!(
                    "The scope sends only empty waveforms: {} s through :STOP.",
                    self.limit.as_secs_f32()
                ))
            }
            Some(_) => Quiet::Wait,
        }
    }
}

/// `dlm_check_channels`: on a DLM2000, channel 4 and logic share their inputs.
/// Other models are as in C, which has a TODO for them.
pub fn channels_conflict(chan4: bool, logic: bool) -> bool {
    chan4 && logic
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_models_are_all_there() {
        for (id, name) in [
            ("710105", "DLM2022"),
            ("710115", "DLM2032"),
            ("710125", "DLM2052"),
            ("710110", "DLM2024"),
            ("710120", "DLM2034"),
            ("710130", "DLM2054"),
            ("701307", "DL9040"),
            ("701308", "DL9040L"),
            ("701310", "DL9140"),
            ("701311", "DL9140L"),
            ("701312", "DL9240"),
            ("701313", "DL9240L"),
            ("701320", "DL9505L"),
            ("701321", "DL9510L"),
            ("701330", "DL9705L"),
            ("701331", "DL9710L"),
        ] {
            assert_eq!(model_get(id).map(|m| m.1), Some(name));
        }
        assert!(model_get("710140").is_none());
        assert!(model_get("").is_none());
    }

    #[test]
    fn a_scaled_value_finds_its_entry() {
        let tb = &TIMEBASES;
        let at = |pair: (u64, u64)| tb.iter().position(|e| *e == pair);
        assert_eq!(scaled_index("1.000E-09", tb), at((1, 1_000_000_000)));
        assert_eq!(scaled_index("10.00E-09", tb), at((10, 1_000_000_000)), "C's own example");
        assert_eq!(scaled_index("2.000E-06", tb), at((2, 1_000_000)));
        assert_eq!(scaled_index("1.000E-08", tb), at((10, 1_000_000_000)));
        assert_eq!(scaled_index("2.000E-07", tb), at((200, 1_000_000_000)));
        assert_eq!(scaled_index("5.000E-01", tb), at((500, 1000)));
        assert_eq!(scaled_index("1.000E+00", tb), at((1, 1)));
        assert_eq!(scaled_index("1.000E+01", tb), at((10, 1)));
        assert_eq!(scaled_index("5.000E+02", tb), at((500, 1)));
        assert_eq!(scaled_index("0.100E-06", tb), None, "0 once truncated, as in C");
        assert_eq!(scaled_index("2.900E-03", &VDIVS), Some(0), "truncated, as in C");
    }

    /// Every entry of both tables comes back from the form the manual prints.
    #[test]
    fn each_table_entry_reads_back() {
        for table in [&TIMEBASES[..], &VDIVS[..]] {
            for (i, entry) in table.iter().enumerate() {
                let text = format!("{:.3E}", entry.0 as f64 / entry.1 as f64);
                let (m, e) = text.split_once('E').unwrap();
                let e: i32 = e.parse().unwrap();
                let form = format!("{m}E{}{:02}", if e < 0 { '-' } else { '+' }, e.abs());
                assert_eq!(scaled_index(&form, table), Some(i), "{form}");
            }
        }
    }

    #[test]
    fn what_is_no_value_is_refused() {
        for text in [
            "",
            "2",
            "2.000",
            "E-03",
            "abc",
            "2.000E",
            "2.000e-03",
            "2.000E-003",
            "2.000E-3x",
            "12345678901E-03",
            "00000002.000E-03",
            "4.000E-03",
            "2.000E+03",
            "1.000E+03",
            "1.000E-10",
            "1.000E-99",
            "nanE-03",
            "-2.000E-03",
        ] {
            assert_eq!(scaled_index(text, &VDIVS), None, "{text:?}");
        }
    }

    #[test]
    fn the_settings_are_written_with_percent_e() {
        assert_eq!(table_text((1, 1_000_000_000)), "1.000000E-09");
        assert_eq!(table_text((2, 1000)), "2.000000E-03");
        assert_eq!(table_text((500, 1)), "5.000000E+02");
        assert_eq!(format_e(-0.0), "-0.000000E+00");
        assert_eq!(format_e(1.5e100), "1.500000E+100");
        assert_eq!(format_e(-5e-3), "-5.000000E-03");
    }

    #[test]
    fn a_position_goes_both_ways_in_cs_floats() {
        let ms = (1, 1000);
        assert_eq!(delay_for_position(0.0, ms, 10), 5e-3);
        assert_eq!(format_e(delay_for_position(0.5, ms, 10)), "-0.000000E+00");
        assert_eq!(delay_for_position(1.0, ms, 10), -5e-3);
        assert_eq!(position_for_delay(0.0, ms, 10), 0.5);
        assert_eq!(position_for_delay(5e-3, ms, 10), 0.0);
    }

    #[test]
    fn volts_are_a_float_then_a_double_then_a_float() {
        assert_eq!(volts(0, 5.0, 0.0), 0.0);
        assert_eq!(volts(25, 5.0, 0.0), 10.0);
        assert_eq!(volts(0xff, 5.0, 0.0), -0.4, "signed");
        assert_eq!(volts(0x80, 5.0, 0.0), -51.2);
        assert_eq!(volts(0x7f, 5.0, 1.0), 51.8);
        // In floats alone some bytes come out another last bit.
        let differ = (0..=255u8)
            .filter(|b| {
                let all_float = 0.3_f32 * f32::from(*b as i8) / 12.5 + 0.1;
                volts(*b, 0.3, 0.1).to_bits() != all_float.to_bits()
            })
            .count();
        assert!(differ > 0);
        // And the product is a float: exact, it rounds elsewhere.
        for range in [0.3_f32, 1.1, 7.7, 12.3] {
            for b in 0..=255u8 {
                let v = f32::from(b as i8);
                let want = (f64::from(range * v) / 12.5 + f64::from(0.1_f32)) as f32;
                assert_eq!(volts(b, range, 0.1).to_bits(), want.to_bits(), "{range} {b}");
            }
        }
    }

    #[test]
    fn a_rate_is_a_float_truncated() {
        assert_eq!(rate(1.25e9), Some(1_250_000_000));
        assert_eq!(rate(0.5), Some(0));
        assert_eq!(rate(-1.0), None);
        assert_eq!(rate(1e30), None);
        assert_eq!(rate(f32::NAN), None);
    }

    #[test]
    fn numbers_are_decimal_and_finite() {
        assert_eq!(number("5.000E+00"), Some(5.0));
        assert_eq!(number(" 1.5"), Some(1.5));
        assert_eq!(number("+1"), Some(1.0));
        for text in ["", "abc", "1.5 ", "1,5", "0x10", "inf", "-inf", "nan", "1e39", "1.0V"] {
            assert_eq!(number(text), None, "{text:?}");
        }
    }

    #[test]
    fn a_length_is_a_count() {
        assert_eq!(length("12500"), Some(12500));
        assert_eq!(length(" 7 "), Some(7));
        assert_eq!(length("4294967295"), Some(u32::MAX));
        for text in ["", "-5", "12.5", "1E4", "abc", "4294967296"] {
            assert_eq!(length(text), None, "{text:?}");
        }
    }

    #[test]
    fn a_block_head() {
        let good = |len, bytes| Head::Good(Header { len, bytes });
        assert_eq!(block_head(b"#800000016abc"), good(10, 16));
        assert_eq!(block_head(b"#3012"), good(5, 12));
        assert_eq!(block_head(b"#0"), good(2, 0));
        assert_eq!(block_head(b"#9999999999"), good(11, 999_999_999));
        for short in [&b""[..], b"#", b"#8", b"#80000"] {
            assert_eq!(block_head(short), Head::Short, "{short:?}");
        }
        for bad in [&b"X"[..], b"\n", b"#A", b"#Z", b"#-", b"#8 ", b"#3 12", b"#3-12", b"#800000x"]
        {
            assert_eq!(block_head(bad), Head::Bad, "{bad:?}");
        }
    }

    fn start() -> (Bounds, Instant) {
        let bounds = Bounds {
            warn: Duration::from_secs(10),
            first: Duration::from_secs(120),
            later: Duration::from_secs(600),
            gap: Duration::from_secs(5),
            empty: Duration::from_secs(10),
            empty_pause: Duration::ZERO,
        };
        (bounds, Instant::now())
    }

    #[test]
    fn silence_before_any_sample_ends_a_capture() {
        let (bounds, t) = start();
        let mut s = Silence::new(bounds, t);
        assert_eq!(s.quiet(t + Duration::from_secs(9)), Quiet::Wait);
        let warned = s.quiet(t + Duration::from_secs(10));
        assert_eq!(
            warned,
            Quiet::Warn("No answer from the scope in 10 s; still waiting.".to_owned())
        );
        assert_eq!(s.quiet(t + Duration::from_secs(11)), Quiet::Wait, "said once");
        assert_eq!(s.quiet(t + Duration::from_secs(119)), Quiet::Wait);
        assert_eq!(
            s.quiet(t + Duration::from_secs(120)),
            Quiet::GiveUp("No answer from the scope in 120 s.".to_owned())
        );
    }

    #[test]
    fn after_a_sample_silence_warns_up_to_a_long_limit() {
        let (bounds, t) = start();
        let mut s = Silence::new(bounds, t);
        s.sample_given();
        assert!(matches!(s.quiet(t + Duration::from_secs(10)), Quiet::Warn(_)));
        assert_eq!(s.quiet(t + Duration::from_secs(599)), Quiet::Wait);
        assert_eq!(
            s.quiet(t + Duration::from_secs(600)),
            Quiet::GiveUp("No answer from the scope in 600 s.".to_owned())
        );
    }

    /// Many bounds on a moved clock: without the reset this ends at once.
    #[test]
    fn hearing_the_scope_starts_it_over_and_says_so_when_it_warned() {
        let (bounds, t) = start();
        let mut s = Silence::new(bounds, t);
        let mut now = t;
        for _ in 0..50 {
            now += Duration::from_secs(9);
            assert_eq!(s.quiet(now), Quiet::Wait);
            assert_eq!(s.heard(now), None, "nothing to take back");
        }
        now += Duration::from_secs(11);
        assert!(matches!(s.quiet(now), Quiet::Warn(_)));
        assert_eq!(s.heard(now), Some("The scope answers again."));
        assert_eq!(s.heard(now), None, "said once");
        assert!(matches!(s.quiet(now + Duration::from_secs(10)), Quiet::Warn(_)), "may warn again");
    }

    #[test]
    fn a_time_taken_before_the_stamp_is_no_silence() {
        let (bounds, t) = start();
        let mut s = Silence::new(bounds, t + Duration::from_secs(5));
        assert_eq!(s.quiet(t), Quiet::Wait);
    }

    #[test]
    fn a_gap_runs_from_the_last_byte() {
        let (bounds, t) = start();
        let mut s = Silence::new(bounds, t);
        assert!(!s.gap_over(t + Duration::from_millis(4999)));
        assert!(s.gap_over(t + Duration::from_secs(5)));
        s.heard(t + Duration::from_secs(3));
        assert!(!s.gap_over(t + Duration::from_secs(5)));
        assert!(s.gap_over(t + Duration::from_secs(8)));
    }

    #[test]
    fn empty_blocks_end_after_their_own_time() {
        let (bounds, t) = start();
        let mut e = Empties::new(&bounds);
        assert_eq!(
            e.empty(t),
            Quiet::Warn(
                "Zero-length waveform data packet received. Live mode not supported yet, \
                 stopping acquisition and retrying."
                    .to_owned()
            )
        );
        assert_eq!(e.empty(t + Duration::from_secs(9)), Quiet::Wait);
        assert_eq!(
            e.empty(t + Duration::from_secs(10)),
            Quiet::GiveUp("The scope sends only empty waveforms: 10 s through :STOP.".to_owned())
        );
    }

    #[test]
    fn the_default_bounds_are_literal() {
        let b = Bounds::default();
        assert_eq!(b.warn, Duration::from_secs(10));
        assert_eq!(b.first, Duration::from_secs(120));
        assert_eq!(b.later, Duration::from_secs(600));
        assert_eq!(b.gap, Duration::from_secs(5));
        assert_eq!(b.empty, Duration::from_secs(10));
        assert_eq!(b.empty_pause, Duration::from_millis(100));
    }

    #[test]
    fn channel_4_and_logic_conflict() {
        assert!(channels_conflict(true, true));
        assert!(!channels_conflict(true, false));
        assert!(!channels_conflict(false, true));
    }
}
