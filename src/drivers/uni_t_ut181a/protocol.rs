// Counterpart in sigrok:
//   libsigrok/src/hardware/uni-t-ut181a/protocol.c,
//   libsigrok/src/hardware/uni-t-ut181a/protocol.h,
//   Copyright (C) 2019-2020 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The UT181A's frames and packets, with no port in sight.
//!
//! A frame is the magic `AB CD`, a 16 bit length of what follows, the payload,
//! and a 16 bit sum of the length's and the payload's bytes, all little endian.
//! `antage/ut181a` sums 2 and the bytes of the payload's length instead. The two
//! differ when that length's low byte is 0xfe or 0xff, as in a chunk of 28 or 85
//! samples, and nothing says which a meter does: this is C's, unsettled.
//! The first payload byte says what the packet is: a reply code (`OK` or `ER`), a
//! measurement, a saved measurement, a recording's info or samples, or a reply
//! with a value and the code of the command it answers. Only that last one says
//! what it answers.
//!
//! A measurement holds a mode word, which gives the quantity's flags and the
//! range table, and values with a unit text, which give the quantity, the unit
//! and a scale. See `docs/DEVICES.md` for what is known of the meter.

use crate::core::{Mq, MqFlags, Unit};

/// The line, `9600/8n1`.
pub const SERIALCOMM: &str = "9600/8n1";
/// `RECV_BUFF_SIZE`: a frame is shorter than this, or no frame.
pub const RECV_BUFF: usize = 4096;
/// `MAX_REC_COUNT`: recordings past this many are not asked for.
pub const MAX_RECORDINGS: usize = 20;

/// `CMD_CODE_SET_MODE`.
const CMD_SET_MODE: u8 = 0x01;
/// `CMD_CODE_SET_RANGE`.
const CMD_SET_RANGE: u8 = 0x02;
/// `CMD_CODE_SET_MONITOR`.
const CMD_SET_MONITOR: u8 = 0x05;
/// `CMD_CODE_GET_SAVED_MEAS`.
const CMD_GET_SAVED: u8 = 0x07;
/// `CMD_CODE_GET_SAVED_COUNT`.
pub const CMD_GET_SAVED_COUNT: u8 = 0x08;
/// `CMD_CODE_GET_REC_INFO`.
pub const CMD_GET_REC_INFO: u8 = 0x0c;
/// `CMD_CODE_GET_REC_SAMPLES`.
const CMD_GET_REC_SAMPLES: u8 = 0x0d;
/// `CMD_CODE_GET_RECS_COUNT`.
pub const CMD_GET_RECS_COUNT: u8 = 0x0e;

const KIND_REPLY_CODE: u8 = 0x01;
const KIND_MEASUREMENT: u8 = 0x02;
const KIND_SAVE: u8 = 0x03;
const KIND_REC_INFO: u8 = 0x04;
const KIND_REC_DATA: u8 = 0x05;
const KIND_REPLY_DATA: u8 = 0x72;

/// The channels, by what they carry in a normal measurement.
pub const CHANNELS: [&str; 5] = ["P1", "P2", "P3", "P4", "bar"];
const CH_MAIN: usize = 0;
const CH_AUX1: usize = 1;
const CH_AUX2: usize = 2;
const CH_AUX3: usize = 3;
const CH_BAR: usize = 4;

/// The 16 bit sum on the wire.
pub fn checksum(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |sum, b| sum.wrapping_add(u16::from(*b)))
}

/// `ut181a_send_frame`'s bytes.
pub fn frame(payload: &[u8]) -> Vec<u8> {
    let mut out = vec![0xab, 0xcd];
    let length = u16::try_from(payload.len() + 2).expect("a command is short");
    out.extend_from_slice(&length.to_le_bytes());
    out.extend_from_slice(payload);
    let sum = checksum(&out[2..]);
    out.extend_from_slice(&sum.to_le_bytes());
    out
}

/// Monitor mode on or off.
pub fn monitor(on: bool) -> Vec<u8> {
    frame(&[CMD_SET_MONITOR, u8::from(on)])
}

/// Sets the mode.
pub fn set_mode(mode: u16) -> Vec<u8> {
    let [low, high] = mode.to_le_bytes();
    frame(&[CMD_SET_MODE, low, high])
}

/// Sets the range; 0 is auto.
pub fn set_range(range: u8) -> Vec<u8> {
    frame(&[CMD_SET_RANGE, range])
}

/// Asks how many saved measurements there are.
pub fn get_saved_count() -> Vec<u8> {
    frame(&[CMD_GET_SAVED_COUNT])
}

/// Asks for a saved measurement, by an index from 0: the wire's count from 1.
pub fn get_saved(index: usize) -> Vec<u8> {
    let [low, high] = (index.wrapping_add(1) as u16).to_le_bytes();
    frame(&[CMD_GET_SAVED, low, high])
}

/// Asks how many recordings there are.
pub fn get_recs_count() -> Vec<u8> {
    frame(&[CMD_GET_RECS_COUNT])
}

/// Asks for a recording's info, by an index from 0.
pub fn get_rec_info(index: usize) -> Vec<u8> {
    let [low, high] = (index.wrapping_add(1) as u16).to_le_bytes();
    frame(&[CMD_GET_REC_INFO, low, high])
}

/// Asks for samples of a recording, both counted from 0.
pub fn get_rec_samples(index: usize, offset: usize) -> Vec<u8> {
    let [low, high] = (index.wrapping_add(1) as u16).to_le_bytes();
    let [a, b, c, d] = (offset.wrapping_add(1) as u32).to_le_bytes();
    frame(&[CMD_GET_REC_SAMPLES, low, high, a, b, c, d])
}

/// One entry of `ut181a_mqopts`.
struct MqOption {
    mq: Mq,
    flags: MqFlags,
    modes: &'static [u16],
}

/// `ut181a_mqopts`, in its order: `--show` lists it, and a mode set is the first
/// of an entry.
fn mq_options() -> [MqOption; 16] {
    let ac_dc = MqFlags::AC | MqFlags::DC;
    [
        MqOption {
            mq: Mq::Voltage,
            flags: MqFlags::AC,
            modes: &[0x1111, 0x1112, 0x2111, 0x2112, 0x1131, 0x2131, 0x1141, 0x1142],
        },
        MqOption {
            mq: Mq::Voltage,
            flags: MqFlags::DC,
            modes: &[0x3111, 0x3112, 0x4111, 0x4112, 0x3131, 0x4121],
        },
        MqOption { mq: Mq::Voltage, flags: ac_dc, modes: &[0x3121, 0x3122, 0x2141, 0x2142] },
        MqOption {
            mq: Mq::Gain,
            flags: MqFlags::empty(),
            modes: &[0x1151, 0x1152, 0x1161, 0x1162],
        },
        MqOption {
            mq: Mq::Current,
            flags: MqFlags::AC,
            modes: &[0xa211, 0xa212, 0xa231, 0x9211, 0x9212, 0x9231, 0x8211, 0x8212, 0x8231],
        },
        MqOption {
            mq: Mq::Current,
            flags: MqFlags::DC,
            modes: &[0xa111, 0xa112, 0xa131, 0x9111, 0x9112, 0x8111, 0x8112, 0x8131],
        },
        MqOption {
            mq: Mq::Current,
            flags: ac_dc,
            modes: &[0xa121, 0xa122, 0x9121, 0x9122, 0x8121, 0x8122, 0x9131],
        },
        MqOption { mq: Mq::Resistance, flags: MqFlags::empty(), modes: &[0x5111, 0x5112] },
        MqOption { mq: Mq::Conductance, flags: MqFlags::empty(), modes: &[0x5311, 0x5312] },
        MqOption { mq: Mq::Continuity, flags: MqFlags::empty(), modes: &[0x5211, 0x5212] },
        MqOption { mq: Mq::Voltage, flags: MqFlags::DIODE | MqFlags::DC, modes: &[0x6111, 0x6112] },
        MqOption { mq: Mq::Capacitance, flags: MqFlags::empty(), modes: &[0x6211, 0x6212] },
        MqOption {
            mq: Mq::Frequency,
            flags: MqFlags::empty(),
            modes: &[0x7111, 0x7112, 0x1121, 0x2121, 0xa221, 0x9221, 0x8221],
        },
        MqOption { mq: Mq::DutyCycle, flags: MqFlags::empty(), modes: &[0x7211, 0x7212] },
        MqOption { mq: Mq::PulseWidth, flags: MqFlags::empty(), modes: &[0x7311, 0x7312] },
        MqOption {
            mq: Mq::Temperature,
            flags: MqFlags::empty(),
            modes: &[
                0x4211, 0x4212, 0x4231, 0x4311, 0x4221, 0x4222, 0x4241, 0x4312, 0x4331, 0x4321,
                0x4322, 0x4341,
            ],
        },
    ]
}

/// The quantity and flags of a mode, `ut181a_get_mqitem_from_mode`.
pub fn mq_of_mode(mode: u16) -> Option<(Mq, MqFlags)> {
    mq_options().iter().find(|o| o.modes.contains(&mode)).map(|o| (o.mq, o.flags))
}

/// The mode a quantity and flags select, `ut181a_get_mode_from_mq_flags`.
pub fn mode_of_mq(mq: Mq, flags: MqFlags) -> Option<u16> {
    mq_options().iter().find(|o| o.mq == mq && o.flags == flags).map(|o| o.modes[0])
}

/// `ut181a_get_mq_flags_list`.
pub fn mq_list() -> Vec<(Mq, MqFlags)> {
    mq_options().iter().map(|o| (o.mq, o.flags)).collect()
}

const RANGES_MV: &[&str] = &["60mV", "600mV"];
const RANGES_V: &[&str] = &["6V", "60V", "600V", "1000V"];
const RANGES_DIODE: &[&str] = &["3.0V"];
const RANGES_UA: &[&str] = &["600uA", "6000uA"];
const RANGES_MA: &[&str] = &["60mA", "600mA"];
const RANGES_A: &[&str] = &["20A"];
const RANGES_OHM: &[&str] =
    &["600\u{2126}", "6k\u{2126}", "60k\u{2126}", "600k\u{2126}", "6M\u{2126}", "60M\u{2126}"];
const RANGES_OHM_600: &[&str] = &["600\u{2126}"];
const RANGES_COND: &[&str] = &["60nS"];
const RANGES_CAPA: &[&str] = &["6nF", "60nF", "600nF", "6uF", "60uF", "600uF", "6mF", "600mF"];
const RANGES_FREQ: &[&str] = &["60Hz", "600Hz", "6kHz", "60kHz", "600kHz", "6MHz", "60MHz"];
const RANGES_FREQ_60K: &[&str] = &["60Hz", "600Hz", "6kHz", "60kHz"];
const RANGES_TEMP_C: &[&str] = &["1000°C"];
const RANGES_TEMP_F: &[&str] = &["1832F"];
/// `range_auto`.
pub const RANGE_AUTO: &str = "auto";

/// `ut181a_get_ranges_list`.
pub fn range_list() -> Vec<String> {
    std::iter::once(RANGE_AUTO)
        .chain(RANGES_MV.iter().copied())
        .chain(RANGES_V.iter().copied())
        .chain(RANGES_UA.iter().copied())
        .chain(RANGES_MA.iter().copied())
        .chain(RANGES_A.iter().copied())
        .chain(RANGES_OHM.iter().copied())
        .chain(RANGES_COND.iter().copied())
        .chain(RANGES_CAPA.iter().copied())
        .chain(RANGES_FREQ.iter().copied())
        .chain(RANGES_TEMP_C.iter().copied())
        .chain(RANGES_TEMP_F.iter().copied())
        .map(str::to_owned)
        .collect()
}

/// A mode's range table, and whether a user can choose in it.
fn ranges_of_mode(mode: u16) -> Option<(&'static [&'static str], bool)> {
    Some(match mode {
        0x1111 | 0x1112 | 0x1121 | 0x1131 | 0x1141 | 0x1142 | 0x1151 | 0x1152 | 0x1161 | 0x1162
        | 0x3111 | 0x3112 | 0x3121 | 0x3122 | 0x3131 => (RANGES_V, true),
        0x2111 | 0x2112 | 0x2121 | 0x2131 | 0x2141 | 0x2142 | 0x4111 | 0x4112 | 0x4121 => {
            (RANGES_MV, true)
        }
        0x5111 | 0x5112 => (RANGES_OHM, true),
        0x5211 | 0x5212 => (RANGES_OHM_600, false),
        0x5311 | 0x5312 => (RANGES_COND, false),
        0x6211 | 0x6212 => (RANGES_CAPA, true),
        0x7111 | 0x7112 => (RANGES_FREQ, true),
        0x7211 | 0x7212 | 0x7311 | 0x7312 => (RANGES_FREQ_60K, true),
        0x8111 | 0x8112 | 0x8121 | 0x8122 | 0x8131 | 0x8211 | 0x8212 | 0x8221 | 0x8231 => {
            (RANGES_UA, true)
        }
        0x9111 | 0x9112 | 0x9121 | 0x9122 | 0x9131 | 0x9211 | 0x9212 | 0x9221 | 0x9231 => {
            (RANGES_MA, true)
        }
        0x4211 | 0x4212 | 0x4221 | 0x4222 | 0x4231 | 0x4241 => (RANGES_TEMP_C, false),
        0x4311 | 0x4312 | 0x4321 | 0x4322 | 0x4331 | 0x4341 => (RANGES_TEMP_F, false),
        0x6111 | 0x6112 => (RANGES_DIODE, false),
        0xa111 | 0xa112 | 0xa121 | 0xa122 | 0xa131 | 0xa211 | 0xa212 | 0xa221 | 0xa231 => {
            (RANGES_A, false)
        }
        _ => return None,
    })
}

/// What `ut181a_set_range_from_text` makes of a text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RangeChoice {
    /// The range byte to send, 0 being auto.
    Send(u8),
    /// No mode read yet, or a text not in the mode's table.
    BadArg,
    /// The mode's range cannot be chosen.
    NotApplicable,
    /// A mode no table has.
    UnknownMode,
}

/// The range byte for a text under a mode.
pub fn range_choice(mode: u16, text: &str) -> RangeChoice {
    if text.is_empty() || mode == 0 {
        return RangeChoice::BadArg;
    }
    if text == RANGE_AUTO {
        return RangeChoice::Send(0);
    }
    match ranges_of_mode(mode) {
        None => RangeChoice::UnknownMode,
        Some((_, false)) => RangeChoice::NotApplicable,
        Some((table, true)) => match table.iter().position(|r| *r == text) {
            Some(i) => RangeChoice::Send(i as u8 + 1),
            None => RangeChoice::BadArg,
        },
    }
}

/// What the head of the last measurement says.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Head {
    /// The mode word.
    pub mode: u16,
    /// The range byte.
    pub range: u8,
    /// `is_auto_range`.
    pub auto_range: bool,
    /// `is_rec`.
    pub recording: bool,
}

impl Head {
    /// `ut181a_get_range_from_packet_bytes`.
    pub fn range_text(&self) -> Option<&'static str> {
        if self.auto_range {
            return Some(RANGE_AUTO);
        }
        if self.mode == 0 {
            return None;
        }
        if self.range == 0 {
            return Some(RANGE_AUTO);
        }
        let (table, _) = ranges_of_mode(self.mode)?;
        table.get(usize::from(self.range) - 1).copied()
    }
}

/// `SR_MQ_*` and `SR_UNIT_*` as a unit text gives them; a text it does not know
/// gives none, and a value of neither, which C sends as 0 and 0.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UnitText {
    /// The prefix's power of ten.
    pub scale: i32,
    /// The quantity.
    pub mq: Option<Mq>,
    /// AC, DC or both.
    pub flags: MqFlags,
    /// The unit.
    pub unit: Option<Unit>,
}

/// `ut181a_get_mq_details_from_text`. Its table has no order but its own: `F`
/// goes ahead of `s` and `S`, and a prefix comes off first.
pub fn parse_unit(text: &[u8]) -> UnitText {
    let (scale, rest) = match text.first() {
        Some(b'p') => (-12, &text[1..]),
        Some(b'n') => (-9, &text[1..]),
        Some(b'u') => (-6, &text[1..]),
        Some(b'm') => (-3, &text[1..]),
        Some(b'k') => (3, &text[1..]),
        Some(b'M') => (6, &text[1..]),
        Some(b'G') => (9, &text[1..]),
        _ => (0, text),
    };
    let table: [(&[u8], Mq, Unit); 12] = [
        (b"F", Mq::Capacitance, Unit::Farad),
        (b"dBV", Mq::Gain, Unit::DecibelVolt),
        (b"dBm", Mq::Gain, Unit::DecibelMw),
        (b"~", Mq::Resistance, Unit::Ohm),
        (b"S", Mq::Conductance, Unit::Siemens),
        (b"%", Mq::DutyCycle, Unit::Percentage),
        (b"s", Mq::PulseWidth, Unit::Second),
        (b"Hz", Mq::Frequency, Unit::Hertz),
        (b"\xb0C", Mq::Temperature, Unit::Celsius),
        (b"\xb0F", Mq::Temperature, Unit::Fahrenheit),
        (b"A", Mq::Current, Unit::Ampere),
        (b"V", Mq::Voltage, Unit::Volt),
    ];
    let found = table.iter().find(|(name, _, _)| rest.starts_with(name));
    let (tail, mq, unit) = match found {
        Some((name, mq, unit)) => (&rest[name.len()..], Some(*mq), Some(*unit)),
        None => (rest, None, None),
    };
    let flags = if tail.starts_with(b"ac+dc") {
        MqFlags::AC | MqFlags::DC
    } else if tail.starts_with(b"AC") {
        MqFlags::AC
    } else if tail.starts_with(b"DC") {
        MqFlags::DC
    } else {
        MqFlags::empty()
    };
    UnitText { scale, mq, flags, unit }
}

/// The power of ten `pow(10, scale)` gives, for the scales a prefix has.
fn power_of_ten(scale: i32) -> f64 {
    match scale {
        -12 => 1e-12,
        -9 => 1e-9,
        -6 => 1e-6,
        -3 => 1e-3,
        3 => 1e3,
        6 => 1e6,
        9 => 1e9,
        _ => 1.0,
    }
}

/// One value for the feed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Item {
    /// The channel's index.
    pub channel: usize,
    /// The quantity, if its unit text had one.
    pub mq: Option<Mq>,
    /// Its flags.
    pub flags: MqFlags,
    /// The unit, if its unit text had one.
    pub unit: Option<Unit>,
    /// The value, scaled, or an overload's infinity.
    pub value: f32,
    /// The digits after the point of the scaled value.
    pub digits: i32,
    /// Whether it counts as a sample: only the first value of a frame does.
    pub counts: bool,
}

/// Makes an item from a value, its precision byte and its unit,
/// `ut181a_feedbuff_setup_value` and what comes ahead of it.
pub fn item(channel: usize, value: f32, prec: u8, unit: &[u8], add: MqFlags, counts: bool) -> Item {
    let text = parse_unit(unit);
    let mut digits = i32::from(prec >> 4);
    let mut value = value;
    if text.scale != 0 {
        value = (f64::from(value) * power_of_ten(text.scale)) as f32;
        digits -= text.scale;
    }
    if prec & 0x02 != 0 {
        value = f32::NEG_INFINITY;
    }
    if prec & 0x01 != 0 {
        value = f32::INFINITY;
    }
    Item { channel, mq: text.mq, flags: text.flags | add, unit: text.unit, value, digits, counts }
}

/// Reads fields in turn, each failing when too few bytes are left.
#[derive(Debug)]
struct Reader<'a> {
    data: &'a [u8],
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        if n > self.data.len() {
            return None;
        }
        let (head, rest) = self.data.split_at(n);
        self.data = rest;
        Some(head)
    }

    fn u8(&mut self) -> Option<u8> {
        self.take(1).map(|b| b[0])
    }

    fn u16(&mut self) -> Option<u16> {
        self.take(2).map(|b| u16::from_le_bytes([b[0], b[1]]))
    }

    fn u32(&mut self) -> Option<u32> {
        self.take(4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn f32(&mut self) -> Option<f32> {
        self.take(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn text(&mut self, size: usize) -> Option<Vec<u8>> {
        self.take(size).map(field_text)
    }

    /// A value, its precision and its unit: an item when all of them came.
    fn value(&mut self, channel: usize, add: MqFlags, counts: bool) -> Option<Item> {
        let value = self.f32()?;
        let prec = self.u8()?;
        let unit = self.text(8)?;
        Some(item(channel, value, prec, &unit, add, counts))
    }

    /// A bar: no precision.
    fn bar(&mut self) -> Option<Item> {
        let value = self.f32()?;
        let unit = self.text(8)?;
        Some(item(CH_BAR, value, 0, &unit, MqFlags::empty(), false))
    }
}

/// `isspace` in the C locale.
fn is_c_space(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

/// `consume_str`: leading space off a field, up to its NUL and one byte short of
/// what is left of it, and trailing space off. C writes its NUL one byte past
/// its buffer here (`protocol.c:1720`); see `docs/DEVICES.md`.
pub fn field_text(field: &[u8]) -> Vec<u8> {
    let start = field.iter().position(|c| !is_c_space(*c)).unwrap_or(field.len());
    let rest = &field[start..];
    let room = rest.len().saturating_sub(1);
    let end = rest.iter().take(room).position(|c| *c == 0).unwrap_or(room);
    let mut text = rest[..end].to_vec();
    while text.last().is_some_and(|c| is_c_space(*c)) {
        text.pop();
    }
    text
}

/// A COMP result, which goes nowhere but the log.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Comp {
    /// INNER, OUTER, BELOW or ABOVE, 0 to 3.
    pub mode: u8,
    /// FAIL rather than PASS.
    pub fail: bool,
    /// The limits' digits.
    pub digits: i32,
    /// The upper or only limit.
    pub high: f32,
    /// The lower one, in the modes with two.
    pub low: Option<f32>,
}

impl Comp {
    /// C's debug line.
    pub fn log(&self) {
        let mode = ["INNER", "OUTER", "BELOW", "ABOVE"][usize::from(self.mode & 3)];
        let verdict = if self.fail { "FAIL" } else { "PASS" };
        match self.low {
            Some(low) => log::debug!(
                "Unprocessed COMP result: mode {mode}, {verdict}, digits {}, low {low:.6}, high {:.6}",
                self.digits,
                self.high
            ),
            None => log::debug!(
                "Unprocessed COMP result: mode {mode}, {verdict}, digits {}, limit {:.6}",
                self.digits,
                self.high
            ),
        }
    }
}

/// A measurement as far as it could be read.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Measurement {
    /// The head was good, so a frame begins, and a wait for a measurement is met.
    pub began: bool,
    /// The values read, in the order they go.
    pub items: Vec<Item>,
    /// A COMP result.
    pub comp: Option<Comp>,
    /// All of it was good, so the frame ends.
    pub whole: bool,
}

/// Reads a measurement, its head into `head` field by field as C has it, so a cut
/// one leaves what it got that far.
pub fn parse_measurement(head: &mut Head, body: &[u8]) -> Measurement {
    let mut out = Measurement::default();
    let mut r = Reader { data: body };
    if read_measurement(head, &mut r, &mut out).is_some() {
        out.whole = true;
    }
    out
}

fn read_measurement(head: &mut Head, r: &mut Reader<'_>, out: &mut Measurement) -> Option<()> {
    let misc = r.u8()?;
    let misc2 = r.u8()?;
    head.recording = misc2 & 0x20 != 0;
    head.auto_range = misc2 & 0x01 != 0;
    let mode = r.u16()?;
    head.mode = mode;
    let (_, mode_flags) = mq_of_mode(mode)?;
    let mut add = mode_flags;
    if misc & 0x80 != 0 {
        add |= MqFlags::HOLD;
    }
    if misc2 & 0x01 != 0 {
        add |= MqFlags::AUTORANGE;
    }
    if add.contains(MqFlags::DIODE) {
        add |= MqFlags::DC;
    }
    head.range = r.u8()?;
    out.began = true;
    let has_aux1 = misc & 0x02 != 0;
    let has_aux2 = misc & 0x04 != 0;
    let has_bar = misc & 0x08 != 0;
    match (misc & 0x70) >> 4 {
        0 => {
            out.items.push(r.value(CH_MAIN, add, true)?);
            if has_aux1 {
                out.items.push(r.value(CH_AUX1, MqFlags::empty(), false)?);
            }
            if has_aux2 {
                out.items.push(r.value(CH_AUX2, MqFlags::empty(), false)?);
            }
            if has_bar {
                out.items.push(r.bar()?);
            }
            if misc2 & 0x10 != 0 {
                let mode = r.u8()?;
                if mode > 3 {
                    return None;
                }
                let fail = r.u8()? != 0;
                let digits = i32::from(r.u8()? & 0x0f);
                let high = r.f32()?;
                let low = if mode <= 1 { Some(r.f32()?) } else { None };
                out.comp = Some(Comp { mode, fail, digits, high, low });
            }
        }
        1 => {
            out.items.push(r.value(CH_MAIN, add | MqFlags::RELATIVE, true)?);
            if has_aux1 {
                out.items.push(r.value(CH_AUX1, MqFlags::REFERENCE, false)?);
            }
            if has_aux2 {
                out.items.push(r.value(CH_AUX2, MqFlags::empty(), false)?);
            }
            if has_bar {
                out.items.push(r.bar()?);
            }
        }
        2 => {
            let current = (r.f32()?, r.u8()?);
            let max = (r.f32()?, r.u8()?);
            r.u32()?;
            let avg = (r.f32()?, r.u8()?);
            r.u32()?;
            let min = (r.f32()?, r.u8()?);
            r.u32()?;
            let unit = r.text(8)?;
            out.items.extend([
                item(CH_MAIN, current.0, current.1, &unit, add, true),
                item(CH_AUX1, max.0, max.1, &unit, MqFlags::MAX, false),
                item(CH_AUX2, avg.0, avg.1, &unit, MqFlags::AVG, false),
                item(CH_AUX3, min.0, min.1, &unit, MqFlags::MIN, false),
            ]);
        }
        4 => {
            out.items.push(r.value(CH_AUX1, add | MqFlags::MAX, true)?);
            out.items.push(r.value(CH_AUX3, MqFlags::MIN, false)?);
        }
        other => {
            log::debug!("Unhandled measurement type ({other}).");
            return None;
        }
    }
    Some(())
}

/// A recording's info: what a capture and a list want of it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RecInfo {
    /// The name, trimmed.
    pub name: Vec<u8>,
    /// The unit text.
    pub unit: Vec<u8>,
    /// Seconds between samples.
    pub interval: u16,
    /// The number of samples.
    pub samples: u32,
}

/// `name`: the name alone, as C keeps it before it reads the rest, and the whole
/// info if all of it came.
pub fn parse_rec_info(body: &[u8]) -> (Option<Vec<u8>>, Option<RecInfo>) {
    let mut r = Reader { data: body };
    let Some(name) = r.text(11) else {
        return (None, None);
    };
    if name.is_empty() {
        return (None, None);
    }
    let rest = (|| {
        let unit = r.text(8)?;
        let interval = r.u16()?;
        r.u32()?;
        let samples = r.u32()?;
        for _ in 0..3 {
            r.f32()?;
            r.u8()?;
        }
        r.u32()?;
        Some(RecInfo { name: name.clone(), unit, interval, samples })
    })();
    (Some(name), rest)
}

/// A chunk of samples, as far as it could be read.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Chunk {
    /// What its count says, all counted however many came.
    pub count: u8,
    /// The value and precision of each sample that came whole.
    pub samples: Vec<(f32, u8)>,
    /// All of them came.
    pub whole: bool,
}

/// None when not even the count is there.
pub fn parse_chunk(body: &[u8]) -> Option<Chunk> {
    let mut r = Reader { data: body };
    let count = r.u8()?;
    let mut out = Chunk { count, samples: Vec::new(), whole: false };
    for _ in 0..count {
        let value = r.f32();
        let prec = r.u8();
        let stamp = r.u32();
        match (value, prec, stamp) {
            (Some(value), Some(prec), Some(_)) => out.samples.push((value, prec)),
            _ => return Some(out),
        }
    }
    out.whole = true;
    Some(out)
}

/// One good frame's meaning.
#[derive(Clone, Debug, PartialEq)]
pub enum Packet {
    /// `OK` or not.
    ReplyCode(bool),
    /// A live measurement.
    Measurement(Measurement),
    /// A saved measurement.
    Saved(Measurement),
    /// A recording's info; the name is there even if the rest was not.
    RecInfo(Option<Vec<u8>>, Option<RecInfo>),
    /// A chunk of samples.
    Chunk(Option<Chunk>),
    /// A value, with the code of its command.
    ReplyData(u8, u16),
    /// A kind this does not know.
    Unknown(u8),
    /// Not what its kind must hold, so a search for a frame starts.
    Bad,
}

impl Packet {
    /// Whether it was all there: what is not, is searched past.
    pub fn good(&self) -> bool {
        match self {
            Packet::Measurement(m) | Packet::Saved(m) => m.whole,
            Packet::RecInfo(_, info) => info.is_some(),
            Packet::Chunk(chunk) => chunk.as_ref().is_some_and(|c| c.whole),
            Packet::Bad => false,
            Packet::ReplyCode(_) | Packet::ReplyData(..) | Packet::Unknown(_) => true,
        }
    }
}

/// `process_packet`'s check of the envelope and its first look: the frame must be
/// one the buffer holds whole, and a failed one is searched past.
pub fn parse_frame(frame: &[u8], head: &mut Head) -> Packet {
    if frame.len() < 6 {
        return Packet::Bad;
    }
    let body = &frame[2..frame.len() - 2];
    let sum = u16::from_le_bytes([frame[frame.len() - 2], frame[frame.len() - 1]]);
    if checksum(body) != sum {
        return Packet::Bad;
    }
    let payload = &body[2..];
    let Some((kind, rest)) = payload.split_first() else {
        log::error!("Insufficient payload data, need packet type.");
        return Packet::Bad;
    };
    match *kind {
        KIND_REPLY_CODE => {
            if rest.len() < 2 {
                return Packet::Bad;
            }
            Packet::ReplyCode(rest[..2] == *b"OK")
        }
        KIND_MEASUREMENT => Packet::Measurement(parse_measurement(head, rest)),
        KIND_SAVE => {
            if rest.len() < 4 {
                return Packet::Bad;
            }
            Packet::Saved(parse_measurement(head, &rest[4..]))
        }
        KIND_REC_INFO => {
            let (name, info) = parse_rec_info(rest);
            Packet::RecInfo(name, info)
        }
        KIND_REC_DATA => Packet::Chunk(parse_chunk(rest)),
        KIND_REPLY_DATA => {
            if rest.len() < 3 {
                return Packet::Bad;
            }
            Packet::ReplyData(rest[0], u16::from_le_bytes([rest[1], rest[2]]))
        }
        other => Packet::Unknown(other),
    }
}

/// The receive buffer and the search for frames, `process_buffer`.
#[derive(Debug, Default)]
pub struct Receiver {
    buffer: Vec<u8>,
}

impl Receiver {
    /// A buffer with nothing in it.
    pub fn new() -> Receiver {
        Receiver::default()
    }

    /// Forgets what has come.
    pub fn clear(&mut self) {
        self.buffer.clear();
    }

    /// How much more a read may bring: never 0, as what a frame fills must end
    /// it or fail it, and a failure drops a byte at least. So C's discarding of a
    /// full buffer (`protocol.c:2848-2853`) has no counterpart here.
    pub fn room(&self) -> usize {
        RECV_BUFF - self.buffer.len()
    }

    /// Bytes from the port.
    pub fn push(&mut self, data: &[u8]) {
        self.buffer.extend_from_slice(data);
    }

    /// Hands over each whole frame in turn: `take` says whether it was good.
    /// What failed is searched past, from its second byte on; with `one_search`
    /// that ends the call, and the next one goes on from there.
    pub fn frames(&mut self, one_search: bool, mut take: impl FnMut(&[u8]) -> bool) {
        loop {
            if self.buffer.len() < 4 {
                return;
            }
            let length = usize::from(u16::from_le_bytes([self.buffer[2], self.buffer[3]]));
            let plausible =
                self.buffer[..2] == [0xab, 0xcd] && length >= 2 && length + 4 < RECV_BUFF;
            if !plausible {
                self.resync();
                if one_search {
                    return;
                }
                continue;
            }
            if length + 4 > self.buffer.len() {
                return;
            }
            if take(&self.buffer[..length + 4]) {
                self.buffer.drain(..length + 4);
            } else {
                self.resync();
                if one_search {
                    return;
                }
            }
        }
    }

    /// To the next magic, or all but a last byte that may start one.
    fn resync(&mut self) {
        if let Some(at) = self.buffer.windows(2).skip(1).position(|w| w == [0xab, 0xcd]) {
            self.buffer.drain(..=at);
        } else {
            let last = self.buffer.len() - 1;
            self.buffer.drain(..last);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn head() -> Head {
        Head::default()
    }

    fn unit8(text: &str) -> Vec<u8> {
        let mut out = text.as_bytes().to_vec();
        out.resize(8, 0);
        out
    }

    fn value(v: f32, prec: u8, unit: &str) -> Vec<u8> {
        let mut out = v.to_le_bytes().to_vec();
        out.push(prec);
        out.extend(unit8(unit));
        out
    }

    fn body(misc: u8, misc2: u8, mode: u16, range: u8, rest: &[Vec<u8>]) -> Vec<u8> {
        let mut out = vec![misc, misc2];
        out.extend_from_slice(&mode.to_le_bytes());
        out.push(range);
        for r in rest {
            out.extend_from_slice(r);
        }
        out
    }

    #[test]
    fn commands_on_the_wire() {
        assert_eq!(monitor(true), [0xab, 0xcd, 4, 0, 5, 1, 0x0a, 0]);
        assert_eq!(monitor(false), [0xab, 0xcd, 4, 0, 5, 0, 9, 0]);
        assert_eq!(get_recs_count(), [0xab, 0xcd, 3, 0, 0x0e, 0x11, 0]);
        assert_eq!(get_saved_count(), [0xab, 0xcd, 3, 0, 8, 0x0b, 0]);
        assert_eq!(set_mode(0x3111), [0xab, 0xcd, 5, 0, 1, 0x11, 0x31, 0x48, 0]);
        assert_eq!(set_range(2), [0xab, 0xcd, 4, 0, 2, 2, 8, 0]);
        // Indices go out from 1.
        assert_eq!(get_saved(0), [0xab, 0xcd, 5, 0, 7, 1, 0, 0x0d, 0]);
        assert_eq!(get_rec_info(1), [0xab, 0xcd, 5, 0, 0x0c, 2, 0, 0x13, 0]);
        assert_eq!(get_rec_samples(1, 8), [0xab, 0xcd, 9, 0, 0x0d, 2, 0, 9, 0, 0, 0, 33, 0]);
    }

    #[test]
    fn a_sum_wraps_at_16_bits() {
        assert_eq!(checksum(&[0xff; 300]), 10_964);
    }

    #[test]
    fn modes_map_both_ways() {
        assert_eq!(mq_of_mode(0x3111), Some((Mq::Voltage, MqFlags::DC)));
        assert_eq!(mq_of_mode(0x1141), Some((Mq::Voltage, MqFlags::AC)));
        assert_eq!(mq_of_mode(0x6112), Some((Mq::Voltage, MqFlags::DIODE | MqFlags::DC)));
        assert_eq!(mq_of_mode(0x4341), Some((Mq::Temperature, MqFlags::empty())));
        assert_eq!(mq_of_mode(0x1234), None);
        assert_eq!(mq_of_mode(0), None);
        assert_eq!(mode_of_mq(Mq::Voltage, MqFlags::AC), Some(0x1111));
        assert_eq!(mode_of_mq(Mq::Voltage, MqFlags::DC), Some(0x3111));
        assert_eq!(mode_of_mq(Mq::Current, MqFlags::AC | MqFlags::DC), Some(0xa121));
        assert_eq!(mode_of_mq(Mq::Temperature, MqFlags::empty()), Some(0x4211));
        assert_eq!(mode_of_mq(Mq::Voltage, MqFlags::empty()), None);
        assert_eq!(mode_of_mq(Mq::Power, MqFlags::empty()), None);
        assert_eq!(mq_list().len(), 16);
    }

    #[test]
    fn every_mode_of_the_list_has_a_range_table_and_the_other_way() {
        let mut in_list = 0;
        for option in &mq_options() {
            for mode in option.modes {
                assert!(ranges_of_mode(*mode).is_some(), "{mode:#x}");
                in_list += 1;
            }
        }
        assert_eq!(in_list, 79);
    }

    #[test]
    fn the_range_of_a_head() {
        let at = |mode, range, auto_range| {
            Head { mode, range, auto_range, recording: false }.range_text()
        };
        assert_eq!(at(0x3111, 3, true), Some("auto"));
        assert_eq!(at(0, 3, false), None);
        assert_eq!(at(0x3111, 0, false), Some("auto"));
        assert_eq!(at(0x3111, 1, false), Some("6V"));
        assert_eq!(at(0x3111, 4, false), Some("1000V"));
        assert_eq!(at(0x3111, 5, false), None);
        assert_eq!(at(0x3111, 9, false), None);
        assert_eq!(at(0x6211, 8, false), Some("600mF"));
        assert_eq!(at(0x5211, 1, false), Some("600\u{2126}"));
        assert_eq!(at(0x1234, 1, false), None);
    }

    #[test]
    fn a_range_chosen() {
        assert_eq!(range_choice(0x3111, "auto"), RangeChoice::Send(0));
        assert_eq!(range_choice(0x3111, "6V"), RangeChoice::Send(1));
        assert_eq!(range_choice(0x3111, "1000V"), RangeChoice::Send(4));
        assert_eq!(range_choice(0x7211, "60kHz"), RangeChoice::Send(4));
        assert_eq!(range_choice(0x7211, "600kHz"), RangeChoice::BadArg);
        assert_eq!(range_choice(0x3111, ""), RangeChoice::BadArg);
        assert_eq!(range_choice(0, "6V"), RangeChoice::BadArg);
        assert_eq!(range_choice(0xa111, "auto"), RangeChoice::Send(0));
        assert_eq!(range_choice(0xa111, "20A"), RangeChoice::NotApplicable);
        assert_eq!(range_choice(0x5211, "600\u{2126}"), RangeChoice::NotApplicable);
        assert_eq!(range_choice(0x1234, "6V"), RangeChoice::UnknownMode);
    }

    #[test]
    fn unit_texts() {
        let u = parse_unit(b"mVDC");
        assert_eq!(
            u,
            UnitText {
                scale: -3,
                mq: Some(Mq::Voltage),
                flags: MqFlags::DC,
                unit: Some(Unit::Volt)
            }
        );
        assert_eq!(parse_unit(b"Vac+dc").flags, MqFlags::AC | MqFlags::DC);
        assert_eq!(parse_unit(b"uAAC").scale, -6);
        assert_eq!(parse_unit(b"uAAC").flags, MqFlags::AC);
        assert_eq!(parse_unit(b"k~").scale, 3);
        assert_eq!(parse_unit(b"k~").unit, Some(Unit::Ohm));
        assert_eq!(parse_unit(b"G~").scale, 9);
        assert_eq!(parse_unit(b"pF").scale, -12);
        assert_eq!(parse_unit(b"nS").unit, Some(Unit::Siemens));
        assert_eq!(parse_unit(b"\xb0C").unit, Some(Unit::Celsius));
        assert_eq!(parse_unit(b"\xb0F").unit, Some(Unit::Fahrenheit));
        assert_eq!(parse_unit(b"%").mq, Some(Mq::DutyCycle));
        assert_eq!(parse_unit(b"ms").unit, Some(Unit::Second));
        assert_eq!(parse_unit(b"MHz").mq, Some(Mq::Frequency));
        assert_eq!(parse_unit(b"dBV").unit, Some(Unit::DecibelVolt));
        assert_eq!(parse_unit(b"dBm").unit, Some(Unit::DecibelMw));
        assert_eq!(parse_unit(b"x"), UnitText::default());
        assert_eq!(parse_unit(b""), UnitText::default());
        // A prefix with nothing behind it is still a scale.
        assert_eq!(parse_unit(b"m").scale, -3);
        assert_eq!(parse_unit(b"m").unit, None);
    }

    #[test]
    fn a_value_scales_as_a_float_product() {
        let i = item(0, 1234.5, 0x20, b"kHz", MqFlags::empty(), true);
        assert_eq!(i.value, 1_234_500.0);
        assert_eq!(i.digits, -1);
        let i = item(0, 1.5, 0x30, b"mV", MqFlags::empty(), true);
        // float * pow(10, -3) as a double, back to a float.
        assert_eq!(i.value, (1.5f64 * 0.001) as f32);
        assert_eq!(i.digits, 6);
        // Past 2^24 the low digits show a quotient.
        let i = item(0, 16_777_217.0, 0, b"m~", MqFlags::empty(), false);
        assert_eq!(i.value, (16_777_216.0f64 * 0.001) as f32);
    }

    #[test]
    fn overloads_win_and_positive_goes_last() {
        let inf = |prec| item(0, 1.0, prec, b"V", MqFlags::empty(), true).value;
        assert_eq!(inf(0x31), f32::INFINITY);
        assert_eq!(inf(0x32), f32::NEG_INFINITY);
        assert_eq!(inf(0x33), f32::INFINITY);
        assert_eq!(item(0, 1.0, 0xf1, b"mV", MqFlags::empty(), true).digits, 18);
    }

    #[test]
    fn field_texts() {
        assert_eq!(field_text(b"VDC\0\0\0\0\0"), b"VDC");
        // C keeps one byte less than its field holds.
        assert_eq!(field_text(b"ABCDEFGH"), b"ABCDEFG");
        assert_eq!(field_text(b"ABCDEFG\0"), b"ABCDEFG");
        // Leading spaces shorten the room too.
        assert_eq!(field_text(b"  ABCDEF"), b"ABCDE");
        assert_eq!(field_text(b" V  \0  x"), b"V");
        assert_eq!(field_text(b"        "), b"");
        assert_eq!(field_text(b"\t\n\x0b\x0c\r V\0"), b"V");
        assert_eq!(field_text(b"\0\0\0\0\0\0\0\0"), b"");
        assert_eq!(field_text(b""), b"");
    }

    #[test]
    fn a_normal_measurement_with_all_it_can_have() {
        let b = body(
            0x8e,
            0x01,
            0x3111,
            2,
            &[
                value(1.0, 0x30, "VDC"),
                value(2.0, 0x20, "Hz"),
                value(3.0, 0x10, "dBV"),
                [4f32.to_le_bytes().to_vec(), unit8("VDC")].concat(),
            ],
        );
        let mut h = head();
        let m = parse_measurement(&mut h, &b);
        assert_eq!(h, Head { mode: 0x3111, range: 2, auto_range: true, recording: false });
        assert!(m.began && m.whole);
        assert_eq!(m.items.len(), 4);
        assert_eq!(m.items[0].channel, 0);
        assert!(m.items[0].counts);
        assert_eq!(m.items[0].flags, MqFlags::DC | MqFlags::HOLD | MqFlags::AUTORANGE);
        assert_eq!(m.items[1].channel, 1);
        assert!(!m.items[1].counts);
        assert_eq!(m.items[1].flags, MqFlags::empty());
        assert_eq!(m.items[2].channel, 2);
        assert_eq!(m.items[3].channel, 4);
        assert_eq!(m.items[3].digits, 0);
    }

    #[test]
    fn a_diode_is_dc_and_a_relative_has_its_flags() {
        let mut h = head();
        let m = parse_measurement(&mut h, &body(0, 0, 0x6111, 1, &[value(0.5, 0x30, "V")]));
        assert_eq!(m.items[0].flags, MqFlags::DIODE | MqFlags::DC);
        let b = body(
            0x1e,
            0,
            0x3112,
            1,
            &[
                value(1.0, 0x30, "VDC"),
                value(2.0, 0x30, "VDC"),
                value(3.0, 0x30, "VDC"),
                [4f32.to_le_bytes().to_vec(), unit8("VDC")].concat(),
            ],
        );
        let m = parse_measurement(&mut h, &b);
        assert!(m.whole);
        assert_eq!(m.items[0].flags, MqFlags::DC | MqFlags::RELATIVE);
        assert_eq!(m.items[1].flags, MqFlags::DC | MqFlags::REFERENCE);
        assert_eq!(m.items[2].flags, MqFlags::DC);
        assert_eq!(m.items[3].channel, 4);
    }

    #[test]
    fn min_max_shares_a_unit_and_peak_has_two_values() {
        let stamp = 7u32.to_le_bytes();
        let mut rest = Vec::new();
        for (v, with_stamp) in [(1f32, false), (2.0, true), (3.0, true), (4.0, true)] {
            rest.extend_from_slice(&v.to_le_bytes());
            rest.push(0x20);
            if with_stamp {
                rest.extend_from_slice(&stamp);
            }
        }
        rest.extend(unit8("mVDC"));
        let mut h = head();
        let m = parse_measurement(&mut h, &body(0x20, 0, 0x4111, 1, &[rest]));
        assert!(m.whole);
        let channels: Vec<usize> = m.items.iter().map(|i| i.channel).collect();
        assert_eq!(channels, [0, 1, 2, 3]);
        assert_eq!(m.items[1].flags, MqFlags::DC | MqFlags::MAX);
        assert_eq!(m.items[2].flags, MqFlags::DC | MqFlags::AVG);
        assert_eq!(m.items[3].flags, MqFlags::DC | MqFlags::MIN);
        assert_eq!(m.items[1].digits, 5);
        let b = body(0x40, 0, 0x3131, 1, &[value(2.0, 0x30, "VDC"), value(1.0, 0x30, "VDC")]);
        let m = parse_measurement(&mut h, &b);
        assert!(m.whole);
        assert_eq!(m.items[0].channel, 1);
        assert!(m.items[0].counts);
        assert_eq!(m.items[0].flags, MqFlags::DC | MqFlags::MAX);
        assert_eq!(m.items[1].channel, 3);
        assert!(!m.items[1].counts);
        assert_eq!(m.items[1].flags, MqFlags::DC | MqFlags::MIN);
    }

    #[test]
    fn comp_has_two_limits_in_two_modes() {
        let mut h = head();
        let limits = |mode: u8, n: usize| {
            let mut out = vec![mode, 1, 0xf3];
            for i in 0..n {
                out.extend_from_slice(&(i as f32 + 1.0).to_le_bytes());
            }
            out
        };
        for (mode, n, low) in [(0, 2, Some(2.0)), (1, 2, Some(2.0)), (2, 1, None), (3, 1, None)] {
            let b = body(0, 0x10, 0x3111, 1, &[value(1.0, 0x30, "VDC"), limits(mode, n)]);
            let m = parse_measurement(&mut h, &b);
            assert!(m.whole, "{mode}");
            let comp = m.comp.expect("a result");
            assert_eq!(comp.mode, mode);
            assert!(comp.fail);
            assert_eq!(comp.digits, 3);
            assert_eq!(comp.high, 1.0);
            assert_eq!(comp.low, low);
        }
        // One limit short, and a mode there is none of.
        let b = body(0, 0x10, 0x3111, 1, &[value(1.0, 0x30, "VDC"), limits(0, 1)]);
        assert!(!parse_measurement(&mut h, &b).whole);
        let b = body(0, 0x10, 0x3111, 1, &[value(1.0, 0x30, "VDC"), limits(4, 2)]);
        let m = parse_measurement(&mut h, &b);
        assert!(m.began && !m.whole);
        assert_eq!(m.items.len(), 1, "what came ahead of it went out");
    }

    #[test]
    fn a_cut_measurement_keeps_what_it_got() {
        let good = body(0x02, 0x21, 0x3111, 4, &[value(1.0, 0x30, "VDC"), value(2.0, 0x30, "Hz")]);
        for cut in 0..good.len() {
            let mut h = Head { mode: 0x1111, range: 9, auto_range: false, recording: false };
            let m = parse_measurement(&mut h, &good[..cut]);
            assert!(!m.whole, "{cut}");
            match cut {
                0 | 1 => assert_eq!((h.mode, h.recording), (0x1111, false), "{cut}"),
                2 | 3 => assert_eq!((h.mode, h.recording), (0x1111, true), "{cut}"),
                4 => {
                    assert_eq!((h.mode, h.range), (0x3111, 9));
                    assert!(!m.began);
                }
                _ => {
                    assert_eq!(h.range, 4);
                    assert!(m.began, "{cut}");
                }
            }
            let want = usize::from(cut >= 5 + 13) + usize::from(cut >= 5 + 26);
            assert_eq!(m.items.len(), want, "{cut}");
        }
        let mut h = head();
        assert!(parse_measurement(&mut h, &good).whole);
    }

    #[test]
    fn a_mode_unknown_keeps_the_word_and_begins_no_frame() {
        let mut h = Head { mode: 0x3111, range: 7, auto_range: true, recording: false };
        let m = parse_measurement(&mut h, &body(0, 0, 0x1234, 1, &[value(1.0, 0x30, "VDC")]));
        assert_eq!(h, Head { mode: 0x1234, range: 7, auto_range: false, recording: false });
        assert!(!m.began);
        assert_eq!(m.items.len(), 0);
    }

    #[test]
    fn a_format_no_one_knows_begins_a_frame_and_stays_open() {
        for format in [3u8, 5, 6, 7] {
            let mut h = head();
            let m = parse_measurement(&mut h, &body(format << 4, 0, 0x3111, 1, &[]));
            assert!(m.began && !m.whole, "{format}");
            assert_eq!(m.items.len(), 0);
        }
    }

    #[test]
    fn a_recording_info() {
        let mut b = b"  My rec \0\0".to_vec();
        b.extend(unit8("mVDC"));
        b.extend_from_slice(&5u16.to_le_bytes());
        b.extend_from_slice(&50u32.to_le_bytes());
        b.extend_from_slice(&10u32.to_le_bytes());
        for _ in 0..3 {
            b.extend_from_slice(&1f32.to_le_bytes());
            b.push(0x10);
        }
        b.extend_from_slice(&0u32.to_le_bytes());
        let (name, info) = parse_rec_info(&b);
        assert_eq!(name.as_deref(), Some(&b"My rec"[..]));
        let info = info.expect("whole");
        assert_eq!(info.unit, b"mVDC");
        assert_eq!(info.interval, 5);
        assert_eq!(info.samples, 10);
        let (name, info) = parse_rec_info(&b[..b.len() - 1]);
        assert!(name.is_some());
        assert_eq!(info, None);
        assert_eq!(parse_rec_info(&b[..10]), (None, None));
        let mut no_name = b.clone();
        no_name[..11].fill(b' ');
        assert_eq!(parse_rec_info(&no_name), (None, None));
    }

    #[test]
    fn a_chunk() {
        let mut b = vec![2];
        for v in [1f32, 2.0] {
            b.extend_from_slice(&v.to_le_bytes());
            b.push(0x10);
            b.extend_from_slice(&0u32.to_le_bytes());
        }
        let c = parse_chunk(&b).unwrap();
        assert_eq!(c.samples, [(1.0, 0x10), (2.0, 0x10)]);
        assert!(c.whole);
        let c = parse_chunk(&b[..b.len() - 1]).unwrap();
        assert_eq!(c.count, 2);
        assert_eq!(c.samples, [(1.0, 0x10)]);
        assert!(!c.whole);
        assert_eq!(parse_chunk(&[]), None);
        let c = parse_chunk(&[0]).unwrap();
        assert!(c.whole && c.samples.is_empty());
    }

    #[test]
    fn frames_by_their_kind() {
        let mut h = head();
        assert_eq!(parse_frame(&frame(b"\x01OK"), &mut h), Packet::ReplyCode(true));
        assert_eq!(parse_frame(&frame(b"\x01ER"), &mut h), Packet::ReplyCode(false));
        assert_eq!(parse_frame(&frame(b"\x01x"), &mut h), Packet::Bad);
        assert_eq!(parse_frame(&frame(&[0x72, 8, 3, 1]), &mut h), Packet::ReplyData(8, 259));
        assert_eq!(parse_frame(&frame(&[0x72, 8, 3]), &mut h), Packet::Bad);
        assert_eq!(parse_frame(&frame(&[0x77]), &mut h), Packet::Unknown(0x77));
        assert_eq!(parse_frame(&frame(&[]), &mut h), Packet::Bad);
        assert_eq!(parse_frame(&frame(&[3, 0, 0, 0]), &mut h), Packet::Bad);
        assert!(matches!(
            parse_frame(&frame(&[3, 0, 0, 0, 0]), &mut h),
            Packet::Saved(Measurement { whole: false, .. })
        ));
        assert_eq!(parse_frame(&frame(&[5]), &mut h), Packet::Chunk(None));
        let mut wrong = frame(b"\x01OK");
        *wrong.last_mut().unwrap() ^= 1;
        assert_eq!(parse_frame(&wrong, &mut h), Packet::Bad);
        assert_eq!(parse_frame(&[0xab, 0xcd, 0, 0], &mut h), Packet::Bad);
    }

    fn drain(r: &mut Receiver, good: &[&[u8]]) -> Vec<Vec<u8>> {
        let mut got = Vec::new();
        r.frames(false, |f| {
            got.push(f.to_vec());
            good.contains(&f)
        });
        got
    }

    #[test]
    fn a_frame_in_pieces_and_two_at_once() {
        let a = frame(b"\x01OK");
        let b = frame(&[0x72, 8, 1, 0]);
        let mut r = Receiver::new();
        r.push(&a[..3]);
        assert_eq!(drain(&mut r, &[&a, &b]).len(), 0);
        r.push(&a[3..]);
        r.push(&b);
        assert_eq!(drain(&mut r, &[&a, &b]), [a.clone(), b.clone()]);
        assert_eq!(r.room(), RECV_BUFF);
    }

    #[test]
    fn it_searches_past_what_fails() {
        let good = frame(b"\x01OK");
        // Garbage ahead.
        let mut r = Receiver::new();
        r.push(&[0, 1, 2, 3, 4]);
        r.push(&good);
        assert_eq!(drain(&mut r, &[&good]), std::slice::from_ref(&good));
        // A length too small, and one no buffer holds.
        for head in [[0xab, 0xcd, 1, 0], [0xab, 0xcd, 0, 0x10]] {
            let mut r = Receiver::new();
            r.push(&head);
            r.push(&good);
            assert_eq!(drain(&mut r, &[&good]), std::slice::from_ref(&good), "{head:02x?}");
        }
        // The longest length a frame can have is waited for, the next is not.
        let mut r = Receiver::new();
        r.push(&[0xab, 0xcd, 0xfb, 0x0f]);
        r.push(&good);
        assert_eq!(drain(&mut r, &[&good]).len(), 0);
        let mut r = Receiver::new();
        r.push(&[0xab, 0xcd, 0xfc, 0x0f]);
        r.push(&good);
        assert_eq!(drain(&mut r, &[&good]), std::slice::from_ref(&good));
        // One that failed: the magic inside it is the next try.
        let mut bad = frame(&[0xab, 0xcd, 2, 0, 9, 9]);
        let inner = bad.clone();
        bad.extend_from_slice(&good);
        let mut r = Receiver::new();
        r.push(&bad);
        let got = drain(&mut r, &[&good]);
        assert_eq!(got[0], inner);
        assert_eq!(got.last(), Some(&good));
    }

    #[test]
    fn one_search_stops_at_it_and_goes_on_at_the_next_call() {
        let good = frame(b"\x01OK");
        let mut r = Receiver::new();
        r.push(&[1, 2, 3, 4]);
        r.push(&good);
        let mut seen = 0;
        r.frames(true, |_| {
            seen += 1;
            true
        });
        assert_eq!(seen, 0);
        r.frames(true, |_| {
            seen += 1;
            true
        });
        assert_eq!(seen, 1);
        // A failed frame too, with what is behind it.
        let mut r = Receiver::new();
        r.push(&good);
        r.push(&good);
        let mut seen = 0;
        r.frames(true, |_| {
            seen += 1;
            false
        });
        assert_eq!(seen, 1);
    }

    #[test]
    fn with_no_magic_it_keeps_one_byte_which_may_start_one() {
        let mut r = Receiver::new();
        r.push(&[1, 2, 3, 4, 5, 0xab]);
        assert_eq!(drain(&mut r, &[]).len(), 0);
        assert_eq!(r.room(), RECV_BUFF - 1);
        let good = frame(b"\x01OK");
        r.push(&good[1..]);
        assert_eq!(drain(&mut r, &[&good]), std::slice::from_ref(&good));
    }
}
