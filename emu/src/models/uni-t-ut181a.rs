// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A UNI-T UT181A multimeter on the CDC-ACM shell: framed binary messages at
//! 9600 8n1, with a live stream, saved readings and stored recordings.
//!
//! The device, and how sure this is:
//!
//! - The frames, the packet layouts, the mode words, the command codes: the
//!   `Protocol.md` of the MIT licensed `antage/ut181a` project, commit 21c5b07,
//!   written against a real meter, and read with its Rust code.
//! - From that code as well: a host waits for a reply code after every command
//!   that sets something (`wait_success`), and an `ER` is what it fears; after a
//!   monitor off it waits for a reply code or a reading; a read of a recording
//!   past its end gives a packet of no samples, which is how it ends a download;
//!   indices count from 1.
//! - The bytes of the COMP fields, and that a chunk of samples may be up to 256
//!   items: comments in libsigrok's driver, from real packets.
//!
//! What is a guess, as no recording was found:
//!
//! - That a monitor on answers `OK` ahead of its first reading. The other host
//!   skips anything but a reading there, so it does not say.
//! - The first reading coming at once. libsigrok gives it 100 ms.
//! - How many samples a chunk has: `per=`, 8 by default.
//! - An `ER` for a saved reading or a recording that is not there, and for a
//!   mode of 0.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id. The real cable is a
//!   CP2110 HID bridge or a BLE adapter: only the byte stream and the line
//!   settings are the device's.
//! - It answers at once, and a reading comes every `period=` offers of a read,
//!   counted from the last frame or command. The meter's own rate is not known.
//! - The first monitor session of a power on sends `first=` readings, 1 by
//!   default, and no more, as that is a tool's scan, whose monitor off would meet
//!   a stream by timing. Later ones send `frames=`, without end by default.
//! - It has no inputs: what it reads is `readings=`, played in turn and then
//!   again. A setting shows in what comes next through the mode and range bytes
//!   alone, the rest being what the list says.
//!
//! A reading in those lists (`readings=`, `saved=`) is one of:
//!
//! - `KIND[FLAGS][~CUT][+PAD]@MODE[.RANGE]:FIELD:...`
//!   - `KIND`: `n` normal, `r` relative, `x` min/max, `p` peak, `tN` the format
//!     number N with no fields, which no real packet has;
//!   - `FLAGS`: `h` hold, `a` auto range, `R` record mode, `v` high voltage,
//!     `l` lead error, `c` COMP;
//!   - `~CUT`: that many bytes off the end, in a frame still good, for a device
//!     gone wrong; `+PAD`: as `pad=` below, for this reading alone, ahead of the
//!     cut, so with both it only moves what is cut; `MODE` is in hex, as is a
//!     `PREC` below;
//!   - `n` and `r` have `m=VALUE/PREC/UNIT`, `1=` and `2=` alike for aux1 and
//!     aux2, `b=VALUE/UNIT` for the bar, and `n` has
//!     `c=MODE/FAIL/DIGITS/HIGH[/LOW]`; a field there sets its flag;
//!   - `x` has `VALUE/PREC`, three of `VALUE/PREC/STAMP` and `UNIT`;
//!   - `p` has two of `VALUE/PREC/UNIT`;
//!   - in a unit `^` is the byte 0xb0 and `_` a space, as is `_` in a name;
//! - `frame:HEX`: a good frame around that payload;
//! - `raw:HEX`: those bytes on the line and nothing else.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `readings=`, `saved=`, lists separated by commas;
//! - `first=N`, `frames=N`, `period=N`, as above;
//! - `rec=NAME/UNIT/INTERVAL/COUNT`, a list of recordings, or `recs=N` of them
//!   named `RECn`. Sample `i` of recording `r`, both from 1, reads
//!   `r * 100 + i + 0.5` at one digit;
//! - `per=N`, samples in a chunk; `over=I:PREC`, sample `I` has this precision
//!   byte in every recording; `have=N`, no sample past the Nth comes, whatever a
//!   record info says, for a device gone wrong;
//! - `fail=CODE:NTH:KIND[:OFFERS]`, a list: the NTH command of that code, 0 for
//!   each of them, is answered as `silent` (nothing, and the event point),
//!   `er`, `bad` (a wrong checksum) or `late` (after that many offers of a read);
//! - `uart=LINE`; `setters=0`, which flags any setting;
//! - `off-reading=1`, a reading ahead of a monitor off's `OK`, which
//!   `antage/ut181a`'s `wait_success_or_measurement` allows for;
//! - `pad=N`, N zero bytes more at the end of a measurement and of a record
//!   info, which is no device. libsigrok's `consume_str` writes one byte past the
//!   caller's 8 byte buffer (`protocol.c:1720`). In the oracle's `-O0` build that
//!   byte is the low byte of the count of bytes left (`unit_buff` at -144 and
//!   `pl_dlen` at -136 in `process_packet`'s frame), so anything behind a unit
//!   fails; a count above 256 survives it, and so C's parser can be shown every
//!   field. Two `-O2` compiles have other bytes there, read from their debug
//!   info: no `-O2` build was run. A field cut behind the first unit can't be
//!   shown in C here: without padding it fails at the first field behind the unit
//!   whatever is cut, and with padding the bytes are there.
//!
//! The checksum is C's, the sum of the bytes of the length and of the payload.
//! `antage/ut181a` sums 2 and the bytes of the payload's length: they differ when
//! that length's low byte is 0xfe or 0xff (a chunk of 28 or 85 samples). Unsettled:
//! no recording has either.
//!
//! The pedantic checks: the envelope, a checksum and a length of each command,
//! only commands that libsigrok's driver has, an index of at least 1, a range
//! the meter has; nothing left of a command when a tool opens. The real commands
//! no path of it sends (reference, min/max, save, delete, start and stop record,
//! hold) are flagged and answered `ER`, itself a guess.
//!
//! What gcov leaves in the C driver: the NULL guards and bad arguments no caller
//! gives (`ut181a_update_recordings`, the feed helpers, `check_len`,
//! `consume_str`); returns of a write that failed (a closed port or a gone one,
//! which the unplug cases reach at other lines); the deadline `api.c:145`, which
//! the first expired wait always goes ahead of; `api.c:409-419` and
//! `ut181a_get_mode_from_mq_flags`, whose counts a crash takes with it
//! (`api.c:412`); the `RSP_TYPE` wait no caller asks for (`protocol.c:1317`);
//! `protocol.c:974` and `:1265`, the timestamp's unit and sample count's wait, as
//! a field cannot hold the text and no caller asks; the 4096 byte overflow
//! (`protocol.c:2851`), which `process_buffer` always keeps it under; and a cut
//! at each field of each format, of which the cases have one per format.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "uni-t-ut181a";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A UNI-T UT181A meter on CDC-ACM; readings=, saved=, first=, frames=, \
                         period=, rec=, recs=, per=, over=, have=, fail=, uart=, setters=, pad=, \
                         off-reading=, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

const MAGIC: [u8; 2] = [0xab, 0xcd];
/// More than the longest command, a start of a recording; what is longer is
/// flagged.
const LONGEST: usize = 64;
/// `recs=`'s and `rec=`'s bound.
const RECORDINGS_MAX: u32 = 200;
/// A sample count's bound, as nothing holds them.
const SAMPLES_MAX: u32 = 100_000;
/// `pad=`'s bound: a frame of libsigrok's must stay under 4096 bytes.
const PAD_MAX: u32 = 3000;
/// 2026-11-05 12:34:56 in the meter's packing.
const STAMP: u32 = 0x1a | 0xb << 6 | 0x5 << 10 | 0xc << 15 | 0x22 << 20 | 0x38 << 26;

/// The mode words of `Protocol.md`.
const MODES: [u16; 79] = [
    0x1111, 0x1112, 0x1121, 0x1131, 0x1141, 0x1142, 0x1151, 0x1152, 0x1161, 0x1162, 0x2111, 0x2112,
    0x2121, 0x2131, 0x2141, 0x2142, 0x3111, 0x3112, 0x3121, 0x3122, 0x3131, 0x4111, 0x4112, 0x4121,
    0x4211, 0x4212, 0x4221, 0x4222, 0x4231, 0x4241, 0x4311, 0x4312, 0x4321, 0x4322, 0x4331, 0x4341,
    0x5111, 0x5112, 0x5211, 0x5212, 0x5311, 0x5312, 0x6111, 0x6112, 0x6211, 0x6212, 0x7111, 0x7112,
    0x7211, 0x7212, 0x7311, 0x7312, 0x8111, 0x8112, 0x8121, 0x8122, 0x8131, 0x8211, 0x8212, 0x8221,
    0x8231, 0x9111, 0x9112, 0x9121, 0x9122, 0x9131, 0x9211, 0x9212, 0x9221, 0x9231, 0xa111, 0xa112,
    0xa121, 0xa122, 0xa131, 0xa211, 0xa212, 0xa221, 0xa231,
];

/// A meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn bad(key: &str, value: &str, want: &str) -> Error {
    Error::Invalid(format!("{key}={value}: {want}"))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("9600/8n1")?;
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        if k == "uart" {
            uart = Line::parse(v)?;
        } else {
            config.option(k, v)?;
        }
    }
    config.check()?;
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "UT181A multimeter");
    Ok(Cdc::new(NAME, &id, faults, Meter::new(config)).behind_uart(uart))
}

impl Config {
    fn option(&mut self, k: &str, v: &str) -> Result<()> {
        match k {
            "readings" => self.readings = parse_list(k, v)?,
            "saved" => self.saved = parse_list(k, v)?,
            "first" => self.first = parse_number(k, v, 0..=100_000)?,
            "frames" => self.frames = Some(parse_number(k, v, 0..=100_000)?),
            "period" => self.period = parse_number(k, v, 1..=100_000)?,
            "rec" => {
                for one in v.split(',') {
                    self.recordings.push(parse_recording(k, one)?);
                }
            }
            "recs" => {
                for i in 1..=parse_number(k, v, 0..=RECORDINGS_MAX)? {
                    self.recordings.push(parse_recording(k, &format!("REC{i}/VDC/1/3"))?);
                }
            }
            "per" => self.per = parse_number(k, v, 1..=255)?,
            "have" => self.have = Some(parse_number(k, v, 0..=SAMPLES_MAX)?),
            "over" => {
                let (index, prec) = v.split_once(':').ok_or_else(|| bad(k, v, "as in 2:11"))?;
                self.over = Some((
                    parse_number(k, index, 1..=SAMPLES_MAX)?,
                    u8::from_str_radix(prec, 16).map_err(|_| bad(k, v, "a hex byte"))?,
                ));
            }
            "fail" => {
                for one in v.split(',') {
                    self.fails.push(parse_fail(k, one)?);
                }
            }
            "setters" => self.setters = parse_number(k, v, 0..=1)? == 1,
            "pad" => self.pad = parse_number(k, v, 0..=PAD_MAX)? as usize,
            "off-reading" => self.off_reading = parse_number(k, v, 0..=1)? == 1,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
        Ok(())
    }

    fn check(&self) -> Result<()> {
        if self.readings.is_empty() {
            return Err(Error::Invalid("readings=: at least one".to_owned()));
        }
        Ok(())
    }
}

/// What a list holds.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Reading {
    /// The packet after its kind byte, with the offset of its mode word known.
    Measurement(Vec<u8>),
    /// A whole payload.
    Payload(Vec<u8>),
    /// Bytes on the line.
    Raw(Vec<u8>),
}

fn hex_bytes(text: &str) -> Option<Vec<u8>> {
    if !text.len().is_multiple_of(2) {
        return None;
    }
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(text.get(i..i + 2)?, 16).ok())
        .collect()
}

fn parse_list(key: &str, value: &str) -> Result<Vec<Reading>> {
    value
        .split(',')
        .map(|one| parse_reading(one).map_err(|why| bad(key, one, &why)))
        .collect()
}

/// A unit or a name in its field, as long as the field or filled with zeros.
fn text_field(out: &mut Vec<u8>, text: &str, size: usize) -> std::result::Result<(), String> {
    let bytes: Vec<u8> = text
        .bytes()
        .map(|c| match c {
            b'^' => 0xb0,
            b'_' => b' ',
            other => other,
        })
        .collect();
    if bytes.len() > size {
        return Err(format!("'{text}': longer than {size}"));
    }
    out.extend_from_slice(&bytes);
    out.resize(out.len() + size - bytes.len(), 0);
    Ok(())
}

fn number<T: std::str::FromStr>(text: &str) -> std::result::Result<T, String> {
    text.parse().map_err(|_| format!("'{text}': no number"))
}

fn prec(text: &str) -> std::result::Result<u8, String> {
    u8::from_str_radix(text, 16).map_err(|_| format!("'{text}': no hex byte"))
}

/// `VALUE/PREC/UNIT`.
fn value_field(out: &mut Vec<u8>, text: &str) -> std::result::Result<(), String> {
    let parts: Vec<&str> = text.split('/').collect();
    let [value, precision, unit] = parts[..] else {
        return Err(format!("'{text}': not VALUE/PREC/UNIT"));
    };
    out.extend_from_slice(&number::<f32>(value)?.to_le_bytes());
    out.push(prec(precision)?);
    text_field(out, unit, 8)
}

/// The fields of a normal or a relative measurement, in the order they go; the
/// flags they set in the misc byte.
fn tagged_fields(fields: &[&str], out: &mut Vec<u8>) -> std::result::Result<u8, String> {
    let mut misc = 0;
    let find = |tag: &str| fields.iter().find_map(|f| f.strip_prefix(tag));
    for f in fields {
        if !["m=", "1=", "2=", "b=", "c="].iter().any(|t| f.starts_with(t)) {
            return Err(format!("'{f}': no such field"));
        }
    }
    let main = find("m=").ok_or("no m= field")?;
    value_field(out, main)?;
    if let Some(aux) = find("1=") {
        value_field(out, aux)?;
        misc |= 0x02;
    }
    if let Some(aux) = find("2=") {
        value_field(out, aux)?;
        misc |= 0x04;
    }
    if let Some(bar) = find("b=") {
        let (value, unit) = bar.split_once('/').ok_or("b= is VALUE/UNIT")?;
        out.extend_from_slice(&number::<f32>(value)?.to_le_bytes());
        text_field(out, unit, 8)?;
        misc |= 0x08;
    }
    if let Some(comp) = find("c=") {
        let parts: Vec<&str> = comp.split('/').collect();
        if parts.len() < 4 || parts.len() > 5 {
            return Err(format!("'{comp}': not MODE/FAIL/DIGITS/HIGH[/LOW]"));
        }
        for byte in &parts[..3] {
            out.push(number::<u8>(byte)?);
        }
        for limit in &parts[3..] {
            out.extend_from_slice(&number::<f32>(limit)?.to_le_bytes());
        }
    }
    Ok(misc)
}

fn minmax_fields(fields: &[&str], out: &mut Vec<u8>) -> std::result::Result<(), String> {
    let [current, tail @ .., unit] = fields else {
        return Err("x has five fields".to_owned());
    };
    if tail.len() != 3 {
        return Err("x has five fields".to_owned());
    }
    let (value, precision) = current.split_once('/').ok_or("not VALUE/PREC")?;
    out.extend_from_slice(&number::<f32>(value)?.to_le_bytes());
    out.push(prec(precision)?);
    for field in tail {
        let parts: Vec<&str> = field.split('/').collect();
        let [value, precision, stamp] = parts[..] else {
            return Err(format!("'{field}': not VALUE/PREC/STAMP"));
        };
        out.extend_from_slice(&number::<f32>(value)?.to_le_bytes());
        out.push(prec(precision)?);
        out.extend_from_slice(&number::<u32>(stamp)?.to_le_bytes());
    }
    text_field(out, unit, 8)
}

fn parse_reading(text: &str) -> std::result::Result<Reading, String> {
    if let Some(hex) = text.strip_prefix("raw:") {
        return hex_bytes(hex).map(Reading::Raw).ok_or_else(|| format!("'{hex}': no hex"));
    }
    if let Some(hex) = text.strip_prefix("frame:") {
        return hex_bytes(hex).map(Reading::Payload).ok_or_else(|| format!("'{hex}': no hex"));
    }
    let mut parts = text.split(':');
    let head = parts.next().unwrap_or_default();
    let fields: Vec<&str> = parts.collect();
    let (kind, place) = head.split_once('@').ok_or("no @MODE")?;
    let (mode, range) = place.split_once('.').unwrap_or((place, "0"));
    let mode = u16::from_str_radix(mode, 16).map_err(|_| format!("'{mode}': no hex mode"))?;
    let range: u8 = number(range)?;
    let (kind, own_pad) = match kind.split_once('+') {
        Some((kind, pad)) => (kind, number::<usize>(pad)?),
        None => (kind, 0),
    };
    let (kind, cut) = match kind.split_once('~') {
        Some((kind, cut)) => (kind, number::<usize>(cut)?),
        None => (kind, 0),
    };
    let (format, flags) = match kind.as_bytes() {
        [b'n', ..] => (0u8, &kind[1..]),
        [b'r', ..] => (1, &kind[1..]),
        [b'x', ..] => (2, &kind[1..]),
        [b'p', ..] => (4, &kind[1..]),
        [b't', digit, ..] if digit.is_ascii_digit() && *digit < b'8' => (digit - b'0', &kind[2..]),
        _ => return Err(format!("'{kind}': no such kind")),
    };
    let mut misc = format << 4;
    let mut misc2 = 0;
    for flag in flags.bytes() {
        match flag {
            b'h' => misc |= 0x80,
            b'a' => misc2 |= 0x01,
            b'v' => misc2 |= 0x02,
            b'l' => misc2 |= 0x08,
            b'c' => misc2 |= 0x10,
            b'R' => misc2 |= 0x20,
            other => return Err(format!("'{}': no such flag", other as char)),
        }
    }
    let mut data = Vec::new();
    match format {
        0 | 1 if kind.as_bytes()[0] != b't' => misc |= tagged_fields(&fields, &mut data)?,
        2 if kind.as_bytes()[0] != b't' => minmax_fields(&fields, &mut data)?,
        4 if kind.as_bytes()[0] != b't' => {
            let [max, min] = fields[..] else {
                return Err("p has two fields".to_owned());
            };
            value_field(&mut data, max)?;
            value_field(&mut data, min)?;
        }
        _ if fields.is_empty() => {}
        _ => return Err(format!("'{kind}': no fields there")),
    }
    let mut body = vec![misc, misc2];
    body.extend_from_slice(&mode.to_le_bytes());
    body.push(range);
    body.extend_from_slice(&data);
    body.resize(body.len() + own_pad, 0);
    body.truncate(body.len().saturating_sub(cut));
    Ok(Reading::Measurement(body))
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Recording {
    name: Vec<u8>,
    unit: Vec<u8>,
    interval: u16,
    count: u32,
}

fn parse_recording(key: &str, text: &str) -> Result<Recording> {
    let parts: Vec<&str> = text.split('/').collect();
    let [name, unit, interval, count] = parts[..] else {
        return Err(bad(key, text, "NAME/UNIT/INTERVAL/COUNT"));
    };
    let field = |text: &str, size| {
        let mut out = Vec::new();
        text_field(&mut out, text, size).map_err(|why| bad(key, text, &why))?;
        Ok::<_, Error>(out)
    };
    Ok(Recording {
        name: field(name, 11)?,
        unit: field(unit, 8)?,
        interval: parse_number(key, interval, 0..=u32::from(u16::MAX))? as u16,
        count: parse_number(key, count, 0..=SAMPLES_MAX)?,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FailKind {
    Silent,
    Er,
    Bad,
    Late(u32),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Fail {
    code: u8,
    /// Which one of that code, from 1; 0 for all.
    nth: u32,
    kind: FailKind,
}

fn parse_fail(key: &str, text: &str) -> Result<Fail> {
    let parts: Vec<&str> = text.split(':').collect();
    let want = "CODE:NTH:silent|er|bad|late[:OFFERS]";
    let (code, nth, kind) = match parts[..] {
        [code, nth, kind] => (code, nth, kind),
        [code, nth, "late", offers] => {
            return Ok(Fail {
                code: u8::from_str_radix(code, 16).map_err(|_| bad(key, text, want))?,
                nth: parse_number(key, nth, 0..=100_000)?,
                kind: FailKind::Late(parse_number(key, offers, 1..=100_000)?),
            });
        }
        _ => return Err(bad(key, text, want)),
    };
    Ok(Fail {
        code: u8::from_str_radix(code, 16).map_err(|_| bad(key, text, want))?,
        nth: parse_number(key, nth, 0..=100_000)?,
        kind: match kind {
            "silent" => FailKind::Silent,
            "er" => FailKind::Er,
            "bad" => FailKind::Bad,
            _ => return Err(bad(key, text, want)),
        },
    })
}

#[derive(Clone, Debug)]
struct Config {
    readings: Vec<Reading>,
    saved: Vec<Reading>,
    first: u32,
    frames: Option<u32>,
    period: u32,
    recordings: Vec<Recording>,
    per: u32,
    over: Option<(u32, u8)>,
    have: Option<u32>,
    fails: Vec<Fail>,
    setters: bool,
    pad: usize,
    off_reading: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            readings: vec![
                parse_reading("na@3111.1:m=5.000/30/VDC").expect("the default parses"),
            ],
            saved: Vec::new(),
            first: 1,
            frames: None,
            period: 25,
            recordings: Vec::new(),
            per: 8,
            over: None,
            have: None,
            fails: Vec::new(),
            setters: true,
            pad: 0,
            off_reading: false,
        }
    }
}

/// `C = N + 2 + sum of the payload`: the sum of the length's bytes and the
/// payload's, in 16 bits.
fn checksum(length_and_payload: &[u8]) -> u16 {
    length_and_payload.iter().fold(0u16, |sum, b| sum.wrapping_add(u16::from(*b)))
}

fn frame(payload: &[u8]) -> Vec<u8> {
    let length = u16::try_from(payload.len() + 2).unwrap_or(u16::MAX);
    let mut out = MAGIC.to_vec();
    out.extend_from_slice(&length.to_le_bytes());
    out.extend_from_slice(payload);
    let sum = checksum(&out[2..]);
    out.extend_from_slice(&sum.to_le_bytes());
    out
}

/// The meter behind its UART, on the CDC shell or on a HID bridge.
#[derive(Clone, Debug)]
pub struct Meter {
    config: Config,
    /// What has come of a frame so far.
    pending: Vec<u8>,
    /// Commands seen, by code.
    seen: [u32; 256],
    next_reading: usize,
    monitor: bool,
    /// Readings this monitor session may still send, `None` for no end.
    budget: Option<u32>,
    monitor_sessions: u32,
    since_frame: u32,
    /// Readings ran out or a command went unanswered: the next offer marks it.
    mark_due: bool,
    /// Answers held back: the offers each still waits, and its bytes.
    held: Vec<(u32, Vec<u8>)>,
    mode: Option<u16>,
    range: Option<u8>,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter {
            config,
            pending: Vec::new(),
            seen: [0; 256],
            next_reading: 0,
            monitor: false,
            budget: None,
            monitor_sessions: 0,
            since_frame: 0,
            mark_due: false,
            held: Vec::new(),
            mode: None,
            range: None,
        }
    }

    /// A measurement body with what has been set.
    fn current(&self, body: &[u8]) -> Vec<u8> {
        let mut body = body.to_vec();
        if body.len() >= 5 {
            if let Some(mode) = self.mode {
                body[2..4].copy_from_slice(&mode.to_le_bytes());
            }
            match self.range {
                Some(0) => body[1] |= 0x01,
                Some(range) => {
                    body[1] &= !0x01;
                    body[4] = range;
                }
                None => {}
            }
        }
        body
    }

    /// The bytes on the line for one reading, as `kind` for a measurement.
    fn wire(&self, reading: &Reading, kind: &[u8]) -> Vec<u8> {
        match reading {
            Reading::Measurement(body) => {
                let mut payload = kind.to_vec();
                payload.extend_from_slice(&self.current(body));
                payload.resize(payload.len() + self.config.pad, 0);
                frame(&payload)
            }
            Reading::Payload(payload) => frame(payload),
            Reading::Raw(bytes) => bytes.clone(),
        }
    }

    fn live_frame(&mut self) -> Vec<u8> {
        let reading = &self.config.readings[self.next_reading % self.config.readings.len()];
        let bytes = self.wire(reading, &[0x02]);
        self.next_reading = self.next_reading.wrapping_add(1);
        bytes
    }

    fn reply_code(ok: bool) -> Vec<u8> {
        frame(if ok { b"\x01OK" } else { b"\x01ER" })
    }

    fn reply_data(code: u8, value: u16) -> Vec<u8> {
        let mut payload = vec![0x72, code];
        payload.extend_from_slice(&value.to_le_bytes());
        frame(&payload)
    }

    fn record_info(&self, rec: &Recording) -> Vec<u8> {
        let mut payload = vec![0x04];
        payload.extend_from_slice(&rec.name);
        payload.extend_from_slice(&rec.unit);
        payload.extend_from_slice(&rec.interval.to_le_bytes());
        payload.extend_from_slice(&(u32::from(rec.interval) * rec.count).to_le_bytes());
        payload.extend_from_slice(&rec.count.to_le_bytes());
        for value in [1.5f32, 1.0, 0.5] {
            payload.extend_from_slice(&value.to_le_bytes());
            payload.push(0x10);
        }
        payload.extend_from_slice(&STAMP.to_le_bytes());
        payload.resize(payload.len() + self.config.pad, 0);
        frame(&payload)
    }

    fn samples(&self, index: u16, rec: &Recording, offset: u32) -> Vec<u8> {
        let count = self.config.have.map_or(rec.count, |have| have.min(rec.count));
        let left = count.saturating_sub(offset - 1);
        let count = left.min(self.config.per);
        let mut payload = vec![0x05, count as u8];
        for i in offset..offset + count {
            let value = f64::from(index) * 100.0 + f64::from(i) + 0.5;
            payload.extend_from_slice(&(value as f32).to_le_bytes());
            let precision = match self.config.over {
                Some((at, precision)) if at == i => precision,
                _ => 0x10,
            };
            payload.push(precision);
            payload.extend_from_slice(&STAMP.to_le_bytes());
        }
        frame(&payload)
    }

    /// Monitor on or off: its reply code, and what comes along with it.
    fn monitor_command(&mut self, arg: u8, port: &mut impl Uart) -> Vec<u8> {
        let on = match arg {
            0 => false,
            1 => true,
            other => {
                port.violation(format!("monitor {other}: neither on nor off"));
                return Self::reply_code(false);
            }
        };
        self.monitor = on;
        self.since_frame = 0;
        let mut out = Self::reply_code(true);
        if !on && self.config.off_reading {
            let mut reading = self.live_frame();
            reading.append(&mut out);
            out = reading;
        }
        if on {
            self.monitor_sessions += 1;
            self.budget = if self.monitor_sessions == 1 {
                Some(self.config.first)
            } else {
                self.config.frames
            };
            if self.budget != Some(0) {
                self.budget = self.budget.map(|n| n - 1);
                out.extend_from_slice(&self.live_frame());
            }
        }
        out
    }

    /// The answer to a good frame's payload, and what it changes.
    fn answer(&mut self, command: &[u8], port: &mut impl Uart) -> Vec<u8> {
        let code = command[0];
        let args = &command[1..];
        let index = (args.len() >= 2).then(|| u16::from_le_bytes([args[0], args[1]]));
        match (code, args.len()) {
            (0x01, 2) => {
                self.setting(code, port);
                let mode = u16::from_le_bytes([args[0], args[1]]);
                if mode == 0 {
                    // A C habit: its range=auto sends this (no path but that).
                    return Self::reply_code(false);
                }
                if !MODES.contains(&mode) {
                    port.violation(format!("set mode {mode:#06x}: no such mode"));
                    return Self::reply_code(false);
                }
                self.mode = Some(mode);
                Self::reply_code(true)
            }
            (0x02, 1) => {
                self.setting(code, port);
                if args[0] > 8 {
                    port.violation(format!("set range {}: no such range", args[0]));
                    return Self::reply_code(false);
                }
                self.range = Some(args[0]);
                Self::reply_code(true)
            }
            (0x05, 1) => self.monitor_command(args[0], port),
            (0x07, 2) => {
                let n = usize::from(index.unwrap_or(0));
                if n == 0 {
                    port.violation("get saved 0: they count from 1");
                    return Self::reply_code(false);
                }
                let Some(reading) = self.config.saved.get(n - 1) else {
                    return Self::reply_code(false);
                };
                let mut kind = vec![0x03];
                kind.extend_from_slice(&STAMP.to_le_bytes());
                self.wire(reading, &kind)
            }
            (0x08, 0) => {
                Self::reply_data(code, u16::try_from(self.config.saved.len()).unwrap_or(u16::MAX))
            }
            (0x0c, 2) => match self.recording(index.unwrap_or(0), code, port) {
                Some(rec) => self.record_info(&rec),
                None => Self::reply_code(false),
            },
            (0x0d, 6) => {
                let n = index.unwrap_or(0);
                let offset = u32::from_le_bytes([args[2], args[3], args[4], args[5]]);
                if offset == 0 {
                    port.violation("get samples at 0: they count from 1");
                    return Self::reply_code(false);
                }
                match self.recording(n, code, port) {
                    Some(rec) => self.samples(n, &rec, offset),
                    None => Self::reply_code(false),
                }
            }
            (0x0e, 0) => Self::reply_data(
                code,
                u16::try_from(self.config.recordings.len()).unwrap_or(u16::MAX),
            ),
            (0x03 | 0x04 | 0x06 | 0x09 | 0x0a | 0x0b | 0x12, _) => {
                port.violation(format!("command {code:#04x}: no path of the driver sends it"));
                Self::reply_code(false)
            }
            _ => {
                port.violation(format!("command {code:#04x} with {} bytes", args.len()));
                Self::reply_code(false)
            }
        }
    }

    fn setting(&self, code: u8, port: &mut impl Uart) {
        if !self.config.setters {
            port.violation(format!("command {code:#04x}: a setting no one asked for"));
        }
    }

    fn recording(&self, index: u16, code: u8, port: &mut impl Uart) -> Option<Recording> {
        if index == 0 {
            port.violation(format!("command {code:#04x} for recording 0: they count from 1"));
            return None;
        }
        self.config.recordings.get(usize::from(index) - 1).cloned()
    }

    /// One good frame's payload.
    fn command(&mut self, command: &[u8], port: &mut impl Uart) {
        if command.is_empty() {
            port.violation("a frame with no command");
            return;
        }
        let code = command[0];
        self.seen[usize::from(code)] += 1;
        let nth = self.seen[usize::from(code)];
        let fail = self
            .config
            .fails
            .iter()
            .find(|f| f.code == code && (f.nth == 0 || f.nth == nth))
            .map(|f| f.kind);
        let mut bytes = self.answer(command, port);
        match fail {
            None => port.send(&bytes),
            Some(FailKind::Silent) => {
                self.mark_due = true;
                if code == 0x05 {
                    // The meter's monitor state went as asked: a reading would
                    // answer it, so none comes.
                    self.budget = Some(0);
                }
            }
            Some(FailKind::Er) => port.send(&Self::reply_code(false)),
            Some(FailKind::Bad) => {
                if let Some(last) = bytes.last_mut() {
                    *last = last.wrapping_add(1);
                }
                port.send(&bytes);
            }
            Some(FailKind::Late(offers)) => self.held.push((offers, bytes)),
        }
    }

    /// Takes the frames that have come whole.
    fn frames(&mut self, port: &mut impl Uart) {
        loop {
            if self.pending.first().is_some_and(|b| *b != MAGIC[0])
                || self.pending.get(1).is_some_and(|b| *b != MAGIC[1])
            {
                port.violation(format!("'{}': no frame magic", show(&self.pending)));
                self.pending.clear();
                return;
            }
            if self.pending.len() < 4 {
                return;
            }
            let length = usize::from(u16::from_le_bytes([self.pending[2], self.pending[3]]));
            if length < 2 || length + 4 > LONGEST {
                port.violation(format!("'{}': a frame of length {length}", show(&self.pending)));
                self.pending.clear();
                return;
            }
            if self.pending.len() < length + 4 {
                return;
            }
            let whole: Vec<u8> = self.pending.drain(..length + 4).collect();
            let sum = u16::from_le_bytes([whole[length + 2], whole[length + 3]]);
            if checksum(&whole[2..length + 2]) != sum {
                port.violation(format!("'{}': a wrong checksum", show(&whole)));
                continue;
            }
            self.command(&whole[4..length + 2], port);
        }
    }
}

/// What a meter's UART needs of what is behind it, here or on a HID bridge.
pub trait Uart {
    /// Bytes to the host.
    fn send(&mut self, data: &[u8]);
    /// What the driver should not have sent.
    fn violation(&mut self, message: impl Into<String>);
    /// The event point.
    fn mark(&mut self);
}

impl Uart for Port<'_> {
    fn send(&mut self, data: &[u8]) {
        Port::send(self, data);
    }
    fn violation(&mut self, message: impl Into<String>) {
        Port::violation(self, message);
    }
    fn mark(&mut self) {
        Port::mark(self);
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        self.take(data, port);
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("'{}': a command left unfinished", show(&self.pending)));
            self.pending.clear();
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        self.offer(port);
    }

    fn reset(&mut self) {
        *self = Meter::new(self.config.clone());
    }
}

impl Meter {
    /// A meter from the options this module takes, the rest being the caller's.
    ///
    /// # Errors
    ///
    /// On an unknown option or a value out of range.
    pub fn from_options(options: &[(String, String)]) -> Result<Meter> {
        let mut config = Config::default();
        for (k, v) in options {
            config.option(k, v)?;
        }
        config.check()?;
        Ok(Meter::new(config))
    }

    /// Power on again.
    #[must_use]
    pub fn power_on(&self) -> Meter {
        Meter::new(self.config.clone())
    }

    /// Bytes from the host.
    pub fn take(&mut self, data: &[u8], port: &mut impl Uart) {
        self.pending.extend_from_slice(data);
        self.frames(port);
    }

    /// One offer of a read with nothing queued.
    pub fn offer(&mut self, port: &mut impl Uart) {
        if self.mark_due {
            self.mark_due = false;
            port.mark();
        }
        let mut i = 0;
        while i < self.held.len() {
            self.held[i].0 -= 1;
            if self.held[i].0 == 0 {
                let (_, bytes) = self.held.remove(i);
                port.send(&bytes);
            } else {
                i += 1;
            }
        }
        if !self.monitor {
            return;
        }
        self.since_frame += 1;
        if self.since_frame < self.config.period {
            return;
        }
        self.since_frame = 0;
        match self.budget {
            Some(0) => {
                // Marked once, a period after the last reading, and never in the
                // first session, a tool's scan: an event there comes before the
                // capture it is for, and before a signal handler is in place.
                self.budget = None;
                self.monitor = false;
                if self.monitor_sessions >= 2 {
                    port.mark();
                }
            }
            Some(n) => {
                self.budget = Some(n - 1);
                let bytes = self.live_frame();
                port.send(&bytes);
            }
            None => {
                let bytes = self.live_frame();
                port.send(&bytes);
            }
        }
    }
}

fn show(bytes: &[u8]) -> String {
    bytes.escape_ascii().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Meter>) -> usize {
        h.v.drain().len()
    }

    /// What a host sends: the libsigrok comment's frame layout, by hand.
    fn cmd(payload: &[u8]) -> Vec<u8> {
        frame(payload)
    }

    #[test]
    fn a_frame_is_as_protocol_md_has_it() {
        // Its checksum: 2 + N + the payload's bytes, here of "05 01".
        assert_eq!(frame(&[0x05, 0x01]), [0xab, 0xcd, 0x04, 0x00, 0x05, 0x01, 0x0a, 0x00]);
        assert_eq!(frame(&[0xff; 300])[2..4], [0x2e, 0x01]);
        assert_eq!(frame(&[0xff; 300])[304..], 11011u16.to_le_bytes(), "wraps at 16 bits");
    }

    #[test]
    fn the_default_reading_comes_at_monitor_on_with_its_ok() {
        let mut h = meter(&[]);
        h.write(&cmd(&[0x05, 0x01]));
        let got = h.read();
        let ok = frame(b"\x01OK");
        assert_eq!(got[..ok.len()], ok);
        let reading = &got[ok.len()..];
        assert_eq!(reading[..4], [0xab, 0xcd, 21, 0]);
        // Kind, misc, misc2 with auto range, mode, range, 5.0.
        assert_eq!(reading[4..10], [0x02, 0x00, 0x01, 0x11, 0x31, 0x01]);
        assert_eq!(reading[10..14], 5.0f32.to_le_bytes());
        assert_eq!(reading[14], 0x30);
        assert_eq!(reading[15..23], *b"VDC\0\0\0\0\0");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_first_session_has_first_readings_the_next_ones_frames() {
        let mut h = meter(&[("period", "2"), ("frames", "2")]);
        h.write(&cmd(&[0x05, 0x01]));
        assert_ne!(h.read(), [0u8; 0]);
        for _ in 0..5 {
            assert_eq!(h.read(), [0u8; 0]);
        }
        h.write(&cmd(&[0x05, 0x00]));
        assert_eq!(h.read(), frame(b"\x01OK"));
        h.write(&cmd(&[0x05, 0x01]));
        assert_eq!(h.read().len(), 9 + 25);
        let mut second = Vec::new();
        for _ in 0..4 {
            second.extend(h.read());
        }
        assert_eq!(second.len(), 25);
        for _ in 0..10 {
            assert_eq!(h.read(), [0u8; 0]);
        }
    }

    #[test]
    fn without_a_bound_it_goes_on_each_period() {
        let mut h = meter(&[("period", "2"), ("first", "1")]);
        h.write(&cmd(&[0x05, 0x01]));
        h.read();
        h.write(&cmd(&[0x05, 0x00]));
        h.read();
        h.write(&cmd(&[0x05, 0x01]));
        h.read();
        for _ in 0..4 {
            let mut got = Vec::new();
            for _ in 0..3 {
                got.extend(h.read());
            }
            assert_ne!(got, [0u8; 0]);
        }
    }

    #[test]
    fn readings_play_in_turn() {
        let mut h = meter(&[("readings", "na@3111:m=1/10/VDC,na@3111:m=2/10/VDC"), ("period", "2")]);
        h.write(&cmd(&[0x05, 0x01]));
        let first = h.read();
        h.write(&cmd(&[0x05, 0x00]));
        h.read();
        h.write(&cmd(&[0x05, 0x01]));
        let second = h.read();
        assert_eq!(first[first.len() - 15..first.len() - 11], 1.0f32.to_le_bytes());
        assert_eq!(second[second.len() - 15..second.len() - 11], 2.0f32.to_le_bytes());
    }

    #[test]
    fn counts_and_saved_readings() {
        let mut h = meter(&[("saved", "n@5111:m=1/10/~,p@3131:1/10/VDC:0/10/VDC")]);
        h.write(&cmd(&[0x08]));
        assert_eq!(h.read(), frame(&[0x72, 0x08, 2, 0]));
        h.write(&cmd(&[0x0e]));
        assert_eq!(h.read(), frame(&[0x72, 0x0e, 0, 0]));
        h.write(&cmd(&[0x07, 1, 0]));
        let got = h.read();
        assert_eq!(got[4], 0x03);
        assert_eq!(got[5..9], STAMP.to_le_bytes());
        assert_eq!(got[9..14], [0, 0, 0x11, 0x51, 0]);
        h.write(&cmd(&[0x07, 2, 0]));
        assert_eq!(h.read()[9], 0x40);
        assert_eq!(violations(&mut h), 0);
        // What is not there: not flagged, as C asks for a first one of none.
        h.write(&cmd(&[0x07, 3, 0]));
        assert_eq!(h.read(), frame(b"\x01ER"));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn recordings_come_in_chunks_and_end_in_an_empty_one() {
        let mut h = meter(&[("rec", "A/VDC/2/3,B_x/mVAC/1/0"), ("per", "2"), ("over", "2:11")]);
        h.write(&cmd(&[0x0e]));
        assert_eq!(h.read(), frame(&[0x72, 0x0e, 2, 0]));
        h.write(&cmd(&[0x0c, 1, 0]));
        let info = h.read();
        assert_eq!(info[4], 0x04);
        assert_eq!(info[5..16], *b"A\0\0\0\0\0\0\0\0\0\0");
        assert_eq!(info[16..24], *b"VDC\0\0\0\0\0");
        assert_eq!(info[24..26], [2, 0]);
        assert_eq!(info[26..30], [6, 0, 0, 0]);
        assert_eq!(info[30..34], [3, 0, 0, 0]);
        assert_eq!(info.len(), 4 + 1 + 48 + 2);
        h.write(&cmd(&[0x0c, 2, 0]));
        assert_eq!(h.read()[5..8], *b"B x");
        h.write(&cmd(&[0x0d, 1, 0, 1, 0, 0, 0]));
        let chunk = h.read();
        assert_eq!(chunk[4..6], [0x05, 2]);
        assert_eq!(chunk[6..10], 101.5f32.to_le_bytes());
        assert_eq!(chunk[10], 0x10);
        assert_eq!(chunk[15..19], 102.5f32.to_le_bytes());
        assert_eq!(chunk[19], 0x11);
        h.write(&cmd(&[0x0d, 1, 0, 3, 0, 0, 0]));
        assert_eq!(h.read()[4..6], [0x05, 1]);
        h.write(&cmd(&[0x0d, 1, 0, 4, 0, 0, 0]));
        assert_eq!(h.read()[4..6], [0x05, 0]);
        h.write(&cmd(&[0x0c, 3, 0]));
        assert_eq!(h.read(), frame(b"\x01ER"));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn have_cuts_what_the_info_says() {
        let mut h = meter(&[("rec", "A/VDC/2/5"), ("per", "2"), ("have", "3")]);
        h.write(&cmd(&[0x0d, 1, 0, 3, 0, 0, 0]));
        assert_eq!(h.read()[4..6], [0x05, 1]);
        h.write(&cmd(&[0x0d, 1, 0, 4, 0, 0, 0]));
        assert_eq!(h.read()[4..6], [0x05, 0]);
    }

    #[test]
    fn recs_makes_that_many() {
        let mut h = meter(&[("recs", "25")]);
        h.write(&cmd(&[0x0e]));
        assert_eq!(h.read(), frame(&[0x72, 0x0e, 25, 0]));
        h.write(&cmd(&[0x0c, 25, 0]));
        let info = h.read();
        assert_eq!(info[5..16], *b"REC25\0\0\0\0\0\0");
        assert_eq!(info[16..24], *b"VDC\0\0\0\0\0");
    }

    #[test]
    fn a_setting_shows_in_the_mode_and_range_bytes() {
        let mut h = meter(&[("period", "2"), ("first", "1")]);
        h.write(&cmd(&[0x01, 0x11, 0x51]));
        assert_eq!(h.read(), frame(b"\x01OK"));
        h.write(&cmd(&[0x02, 0x03]));
        assert_eq!(h.read(), frame(b"\x01OK"));
        h.write(&cmd(&[0x05, 0x01]));
        let got = h.read();
        let at = frame(b"\x01OK").len();
        assert_eq!(got[at + 5..at + 10], [0x00, 0x00, 0x11, 0x51, 0x03]);
        assert_eq!(violations(&mut h), 0);
        h.write(&cmd(&[0x05, 0x00]));
        h.read();
        h.write(&cmd(&[0x02, 0x00]));
        h.read();
        h.write(&cmd(&[0x05, 0x01]));
        let got = h.read();
        assert_eq!(got[at + 6], 0x01, "auto range is back");
    }

    #[test]
    fn the_shapes_of_a_reading() {
        let body = |text: &str| match parse_reading(text).unwrap() {
            Reading::Measurement(b) => b,
            other => panic!("{other:?}"),
        };
        // Normal with all it can have.
        let b = body("nhacRvl@1111.2:m=1/30/VAC:1=2/21/Hz:2=3/12/dBV:b=4/V:c=0/1/3/3.3/1");
        assert_eq!(b[..5], [0x8e, 0x3b, 0x11, 0x11, 2]);
        assert_eq!(b.len(), 5 + 13 * 3 + 12 + 3 + 8);
        // Relative: the format and two flags.
        let b = body("r@3112:m=1/30/VDC:1=2/30/VDC:2=3/30/VDC");
        assert_eq!(b[0], 0x16);
        // Min/max and peak.
        let b = body("x@3111:1/30:2/30/7:3/30/8:4/30/9:VDC");
        assert_eq!(b[0], 0x20);
        assert_eq!(b.len(), 5 + 5 + 9 * 3 + 8);
        let b = body("p@3131:1/30/VDC:2/30/VDC");
        assert_eq!(b[0], 0x40);
        assert_eq!(b.len(), 5 + 13 * 2);
        // A format nothing knows, and a cut one.
        assert_eq!(body("t3@3111")[0], 0x30);
        assert_eq!(body("n~8@3111:m=1/30/VDC").len(), 5 + 13 - 8);
        assert_eq!(body("n~8+100@3111:m=1/30/VDC").len(), 5 + 13 + 100 - 8);
        // A unit as long as its field, a degree and a space.
        let b = body("n@4211:m=1/30/^C_");
        assert_eq!(b[10..18], [0xb0, b'C', b' ', 0, 0, 0, 0, 0]);
        assert_eq!(parse_reading("raw:aabb").unwrap(), Reading::Raw(vec![0xaa, 0xbb]));
        assert_eq!(parse_reading("frame:01").unwrap(), Reading::Payload(vec![1]));
    }

    #[test]
    fn what_a_reading_cannot_be() {
        for text in [
            "n",
            "n+x@3111:m=1/30/V",
            "n~x@3111:m=1/30/V",
            "z@3111",
            "n@3111",
            "nq@3111:m=1/30/V",
            "n@xyz:m=1/30/V",
            "n@3111:q=1",
            "n@3111:m=1/30",
            "n@3111:m=x/30/V",
            "n@3111:m=1/zz/V",
            "n@3111:m=1/30/TOOLONGUN",
            "n@3111:m=1/30/V:b=1",
            "n@3111:m=1/30/V:c=1/2",
            "x@3111:1/30",
            "x@3111:1:2/30/7:3/30/8:4/30/9:V",
            "x@3111:1/30:2/30:3/30/8:4/30/9:V",
            "p@3131:1/30/V",
            "t9@3111",
            "t3@3111:m=1/30/V",
            "raw:abc",
            "frame:zz",
        ] {
            assert!(parse_reading(text).is_err(), "{text}");
        }
    }

    #[test]
    fn the_other_options_refuse_what_they_cannot_take() {
        for (k, v) in [
            ("nothing", "1"),
            ("rec", "A/V"),
            ("rec", "THIS_NAME_IS_TOO_LONG/V/1/1"),
            ("rec", "A/TOOLONGUN/1/1"),
            ("fail", "05:1"),
            ("fail", "xx:1:er"),
            ("fail", "05:1:never"),
            ("fail", "05:1:late:0"),
            ("over", "2"),
            ("over", "2:zz"),
            ("per", "0"),
            ("period", "0"),
            ("pad", "3001"),
        ] {
            let o = [(k.to_owned(), v.to_owned())];
            assert!(function(&o).is_err(), "{k}={v}");
        }
    }

    #[test]
    fn the_faults() {
        let mut h = meter(&[("fail", "0e:1:silent,0e:2:er,0e:3:bad,0e:4:late:2")]);
        h.write(&cmd(&[0x0e]));
        assert_eq!(h.read(), [0u8; 0]);
        h.write(&cmd(&[0x0e]));
        assert_eq!(h.read(), frame(b"\x01ER"));
        h.write(&cmd(&[0x0e]));
        let mut wrong = frame(&[0x72, 0x0e, 0, 0]);
        *wrong.last_mut().unwrap() += 1;
        assert_eq!(h.read(), wrong);
        h.write(&cmd(&[0x0e]));
        assert_eq!(h.read(), [0u8; 0]);
        assert_eq!(h.read(), frame(&[0x72, 0x0e, 0, 0]));
        h.write(&cmd(&[0x0e]));
        assert_eq!(h.read(), frame(&[0x72, 0x0e, 0, 0]));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_fail_on_every_one_of_a_code() {
        let mut h = meter(&[("fail", "08:0:er")]);
        for _ in 0..3 {
            h.write(&cmd(&[0x08]));
            assert_eq!(h.read(), frame(b"\x01ER"));
        }
    }

    #[test]
    fn a_silent_monitor_on_sends_no_reading() {
        let mut h = meter(&[("fail", "05:1:silent"), ("period", "2")]);
        h.write(&cmd(&[0x05, 0x01]));
        for _ in 0..5 {
            assert_eq!(h.read(), [0u8; 0]);
        }
    }

    #[test]
    fn pad_lengthens_measurements_and_record_infos_alone() {
        let mut h = meter(&[("pad", "300"), ("rec", "A/V/1/1")]);
        h.write(&cmd(&[0x05, 0x01]));
        assert_eq!(h.read().len(), 9 + 25 + 300);
        h.write(&cmd(&[0x0c, 1, 0]));
        assert_eq!(h.read().len(), 55 + 300);
        h.write(&cmd(&[0x0d, 1, 0, 1, 0, 0, 0]));
        assert_eq!(h.read().len(), 6 + 2 + 9);
    }

    #[test]
    fn off_reading_comes_ahead_of_the_ok() {
        let mut h = meter(&[("off-reading", "1")]);
        h.write(&cmd(&[0x05, 0x00]));
        let got = h.read();
        assert_eq!(got[4], 0x02);
        assert_eq!(got[got.len() - 9..], frame(b"\x01OK"));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_first_session_marks_no_event_and_the_next_does() {
        let mut h = meter(&[("period", "2"), ("frames", "1")]);
        h.write(&cmd(&[0x05, 0x01]));
        for _ in 0..6 {
            h.read();
        }
        assert!(!h.cdc.mark(), "the scan's");
        h.write(&cmd(&[0x05, 0x00]));
        h.read();
        h.write(&cmd(&[0x05, 0x01]));
        for _ in 0..6 {
            h.read();
        }
        assert!(h.cdc.mark(), "the capture's");
    }

    #[test]
    fn a_mode_of_0_is_er_and_no_violation() {
        let mut h = meter(&[]);
        h.write(&cmd(&[0x01, 0, 0]));
        assert_eq!(h.read(), frame(b"\x01ER"));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn every_pedantic_check_fires() {
        let cases: [&[u8]; 14] = [
            &[0x01, 0x34, 0x12],        // No such mode.
            &[0x02, 9],                 // No such range.
            &[0x05, 2],                 // Monitor neither.
            &[0x07, 0, 0],              // Saved 0.
            &[0x0c, 0, 0],              // Recording 0.
            &[0x0d, 0, 0, 1, 0, 0, 0],  // Samples of recording 0.
            &[0x0d, 1, 0, 0, 0, 0, 0],  // Samples at 0.
            &[0x03, 0, 0, 0, 0],        // Reference: no path.
            &[0x04, 1],                 // Min/max: no path.
            &[0x06],                    // Save: no path.
            &[0x12, 0x5a],              // Hold: no path.
            &[0x05],                    // Too short.
            &[0x40],                    // No such code.
            &[0x08, 1],                 // Too long.
        ];
        for bytes in cases {
            let mut h = meter(&[]);
            h.write(&cmd(bytes));
            assert_eq!(h.read(), frame(b"\x01ER"), "{bytes:02x?}");
            assert_eq!(violations(&mut h), 1, "{bytes:02x?}");
        }
    }

    #[test]
    fn the_envelope_is_checked() {
        let mut wrong = frame(&[0x0e]);
        *wrong.last_mut().unwrap() ^= 1;
        let cases: [Vec<u8>; 5] = [
            vec![0x00, 0xcd, 3, 0],
            vec![0xab, 0x00, 3, 0],
            vec![0xab, 0xcd, 1, 0],
            vec![0xab, 0xcd, 200, 0],
            wrong,
        ];
        for bytes in cases {
            let mut h = meter(&[]);
            h.write(&bytes);
            assert_eq!(violations(&mut h), 1, "{bytes:02x?}");
            assert_eq!(h.read(), [0u8; 0], "{bytes:02x?}");
        }
        let mut h = meter(&[]);
        h.write(&[0xab, 0xcd, 2, 0, 2, 0]);
        assert_eq!(violations(&mut h), 1, "a frame with no command");
    }

    #[test]
    fn a_command_may_come_in_pieces_and_two_in_one_write() {
        let mut h = meter(&[]);
        let whole = cmd(&[0x0e]);
        h.write(&whole[..3]);
        assert_eq!(h.read(), [0u8; 0]);
        h.write(&whole[3..]);
        assert_eq!(h.read(), frame(&[0x72, 0x0e, 0, 0]));
        let mut both = cmd(&[0x0e]);
        both.extend(cmd(&[0x08]));
        h.write(&both);
        let mut want = frame(&[0x72, 0x0e, 0, 0]);
        want.extend(frame(&[0x72, 0x08, 0, 0]));
        assert_eq!(h.read(), want);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_command_left_unfinished_at_an_open_is_flagged() {
        let mut h = meter(&[]);
        h.write(&[0xab]);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_setting_can_be_flagged_as_unasked() {
        let mut h = meter(&[("setters", "0")]);
        h.write(&cmd(&[0x02, 1]));
        assert_eq!(violations(&mut h), 1);
        h.write(&cmd(&[0x01, 0x11, 0x31]));
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn the_wrong_line_passes_nothing() {
        let o = [("uart".to_owned(), "19200/8n1".to_owned())];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true);
        h.write(&cmd(&[0x0e]));
        assert_eq!(h.read(), [0u8; 0]);
    }

    #[test]
    fn a_reset_is_a_power_on() {
        let mut h = meter(&[("fail", "0e:1:silent")]);
        h.write(&cmd(&[0x0e]));
        assert_eq!(h.read(), [0u8; 0]);
        h.write(&cmd(&[0x01, 0x11, 0x51]));
        h.read();
        h.cdc.model.reset();
        assert_eq!(h.cdc.model.mode, None);
        h.write(&cmd(&[0x0e]));
        assert_eq!(h.read(), [0u8; 0], "the count of commands started again");
    }

    #[test]
    fn the_mode_table_has_every_word_once() {
        let mut sorted = MODES.to_vec();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), 79);
        assert!(!MODES.contains(&0));
    }
}
