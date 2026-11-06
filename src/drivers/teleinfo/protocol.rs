// Counterpart in sigrok:
//   libsigrok/src/hardware/teleinfo/protocol.c,
//   libsigrok/src/hardware/teleinfo/protocol.h,
//   Copyright (C) 2013 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, Bert Vermeulen, Daniel Elstner,
//   Frank Stettner, Lars-Peter Clausen, Martin Ling
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Téléinfo stream: what is in a buffer of it.
//!
//! A meter sends groups of LF, label, space, data, space, control, CR for as long
//! as it has power, in frames that start with STX and end with ETX or, when it is
//! interrupted, EOT. Control is the sum of label, space and data, its low six bits,
//! plus 0x20 (Enedis-NOI-CPT_02E, 1.5.2, "mode 1"). All in here is pure.

use crate::core::{Mq, Unit};

/// A capture's receive buffer (`TELEINFO_BUF_SIZE`).
pub const BUF_SIZE: usize = 256;
/// What a scan reads before it looks (api.c's `buf`).
pub const SCAN_BYTES: usize = 292;

const LF: u8 = 0x0a;
const CR: u8 = 0x0d;
/// What a meter sends ahead of an interruption.
pub const EOT: u8 = 0x04;
/// `%8s`.
const LABEL_MAX: usize = 8;
/// `%13s`.
const DATA_MAX: usize = 13;

/// What C's `%s` and a space in a format take for white space.
fn is_space(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

/// A group whose control is right.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Group<'a> {
    /// The label, up to 8 bytes.
    pub label: &'a [u8],
    /// The data, up to 12 bytes.
    pub data: &'a [u8],
}

/// The bytes C's `sscanf` looks at for a group, and where it is in them.
struct Scan<'a> {
    text: &'a [u8],
    at: usize,
}

impl<'a> Scan<'a> {
    fn skip_space(&mut self) {
        while self.text.get(self.at).copied().is_some_and(is_space) {
            self.at += 1;
        }
    }

    /// `%Ns`: white space is skipped, then up to `max` other bytes.
    fn word(&mut self, max: usize) -> Option<&'a [u8]> {
        self.skip_space();
        let start = self.at;
        while self.at - start < max && self.text.get(self.at).is_some_and(|b| !is_space(*b)) {
            self.at += 1;
        }
        (self.at > start).then(|| &self.text[start..self.at])
    }

    /// `%c`.
    fn byte(&mut self) -> Option<u8> {
        let b = self.text.get(self.at).copied();
        self.at += 1;
        b
    }
}

/// The control of `label` and `data`, by C's sum of `char`s, which on the machines
/// the C runs on are signed. On a 7-bit line they are all the same.
fn control(label: &[u8], data: &[u8]) -> i32 {
    let sum: i32 = label
        .iter()
        .chain(b" ")
        .chain(data)
        .fold(0, |sum, b| sum.wrapping_add(i32::from(i8::from_ne_bytes([*b]))));
    (sum & 0x3f) + 0x20
}

/// `teleinfo_parse_group`, on the bytes from a group's LF to the buffer's end: C's
/// `sscanf(str, "\x0A%8s %13s %c%c", ...)`, which ends at a NUL, and its control
/// check.
///
/// As in C, a field may run across white space of any kind, and across a CR: it
/// asks only for four fields whose last is a CR. One thing differs: a control of a
/// space, which a sum's low six bits of 0 give one group in 64 and the
/// specification allows (`PTEC HP..` always has it). In the format's ` %c` the
/// space skips it, so C reads the CR for the control and drops the group.
pub fn parse_group(text: &[u8]) -> Option<Group<'_>> {
    let end = text.iter().position(|b| *b == 0).unwrap_or(text.len());
    let mut scan = Scan { text: &text[..end], at: 0 };
    let label = scan.word(LABEL_MAX)?;
    let data = scan.word(DATA_MAX)?;
    // `data[13]` has no room for the NUL of `%13s`: it clears the label on the
    // oracle's build, whose check then fails. No group of the specification has 13.
    if data.len() == DATA_MAX {
        return None;
    }
    let wanted = control(label, data);
    if wanted == i32::from(b' ') && scan.text[scan.at..].starts_with(b"  \r") {
        return Some(Group { label, data });
    }
    scan.skip_space();
    let got = scan.byte()?;
    let cr = scan.byte()?;
    (cr == CR && i32::from(i8::from_ne_bytes([got])) == wanted).then_some(Group { label, data })
}

/// What `teleinfo_parse_data` finds at the head of `buf`: a group's outcome and how
/// many bytes it used, or `None` while no LF has a CR behind it.
pub fn next_group(buf: &[u8]) -> Option<(Option<Group<'_>>, usize)> {
    let start = buf.iter().position(|b| *b == LF)?;
    let end = start + buf[start..].iter().position(|b| *b == CR)?;
    Some((parse_group(&buf[start..]), end + 1))
}

/// What a meter's contract is, by its `OPTARIF` group, which decides the channels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Optarif {
    /// `BASE`.
    Base,
    /// `HC..`, peak and off-peak.
    Hc,
    /// `EJP.`.
    Ejp,
    /// `BBRx`, Tempo.
    Bbr,
}

impl Optarif {
    fn of(data: &[u8]) -> Option<Optarif> {
        match data {
            b"BASE" => Some(Optarif::Base),
            b"HC.." => Some(Optarif::Hc),
            b"EJP." => Some(Optarif::Ejp),
            d if d.starts_with(b"BBR") => Some(Optarif::Bbr),
            _ => None,
        }
    }

    /// The channels, in the order of api.c's `sr_channel_new` calls.
    pub fn channels(self) -> &'static [&'static str] {
        match self {
            Optarif::Base => &["P", "BASE", "IINST", "PAPP"],
            Optarif::Hc => &["P", "HP", "HC", "IINST", "PAPP"],
            Optarif::Ejp => &["P", "HN", "HPM", "IINST", "PAPP"],
            Optarif::Bbr => &["P", "HPJB", "HPJW", "HPJR", "HCJB", "HCJW", "HCJR", "IINST", "PAPP"],
        }
    }
}

/// `teleinfo_get_optarif`: the last `OPTARIF` group of a buffer says. An unknown one
/// after a known one says "none".
pub fn optarif_of(mut buf: &[u8]) -> Option<Optarif> {
    let mut last = None;
    while let Some((group, used)) = next_group(buf) {
        if let Some(g) = group.filter(|g| g.label == b"OPTARIF") {
            last = Some(g.data);
        }
        buf = &buf[used..];
    }
    last.and_then(Optarif::of)
}

/// What a group means to a capture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Meaning {
    /// A frame's first group, what a sample limit counts.
    Sample,
    /// A reading for the channel of this name, if the device has it.
    Reading {
        /// The channel, as api.c names it.
        channel: &'static str,
        /// The quantity.
        mq: Mq,
        /// The unit.
        unit: Unit,
    },
}

/// `teleinfo_handle_measurement`'s table.
pub fn meaning(label: &[u8]) -> Option<Meaning> {
    let energy = |channel| Meaning::Reading { channel, mq: Mq::Energy, unit: Unit::WattHour };
    Some(match label {
        b"ADCO" => Meaning::Sample,
        b"BASE" => energy("BASE"),
        b"HCHP" => energy("HP"),
        b"HCHC" => energy("HC"),
        b"EJPHN" => energy("HN"),
        b"EJPHPM" => energy("HPM"),
        b"BBRHPJB" => energy("HPJB"),
        b"BBRHPJW" => energy("HPJW"),
        b"BBRHPJR" => energy("HPJR"),
        b"BBRHCJB" => energy("HCJB"),
        b"BBRHCJW" => energy("HCJW"),
        b"BBRHCJR" => energy("HCJR"),
        b"IINST" => Meaning::Reading { channel: "IINST", mq: Mq::Current, unit: Unit::Ampere },
        b"PAPP" => Meaning::Reading { channel: "PAPP", mq: Mq::Power, unit: Unit::VoltAmpere },
        _ => return None,
    })
}

/// C's `atoi` as glibc has it, `(int) strtol(s, NULL, 10)`: a sign, then the digits
/// up to the first other byte, none being 0. Past 64 bits it sticks at the limit,
/// and the cast then keeps the low 32 bits of whatever came.
pub fn atoi(data: &[u8]) -> i32 {
    let (negative, digits) = match data.first() {
        Some(b'-') => (true, &data[1..]),
        Some(b'+') => (false, &data[1..]),
        _ => (false, data),
    };
    let mut value = 0i64;
    for b in digits.iter().take_while(|b| b.is_ascii_digit()) {
        let digit = i64::from(*b - b'0');
        let next = value.checked_mul(10).and_then(|v| v.checked_add(digit));
        value = next.unwrap_or(i64::MAX);
        if next.is_none() {
            break;
        }
    }
    let value = if negative { value.wrapping_neg() } else { value };
    value as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A group, as a meter sends it.
    fn line(label: &str, data: &str) -> Vec<u8> {
        let mut out = vec![LF];
        out.extend_from_slice(label.as_bytes());
        out.push(b' ');
        out.extend_from_slice(data.as_bytes());
        out.push(b' ');
        let control = control(label.as_bytes(), data.as_bytes());
        out.push(u8::try_from(control).unwrap());
        out.push(CR);
        out
    }

    fn parsed(text: &[u8]) -> Option<(String, String)> {
        parse_group(text).map(|g| {
            (
                String::from_utf8_lossy(g.label).into_owned(),
                String::from_utf8_lossy(g.data).into_owned(),
            )
        })
    }

    fn some(label: &str, data: &str) -> Option<(String, String)> {
        Some((label.to_owned(), data.to_owned()))
    }

    #[test]
    fn the_control_is_the_specification_s() {
        // A sum done by hand, in the emulator model's test too.
        assert_eq!(control(b"IINST", b"002"), 0x59);
        assert_eq!(control(b"PTEC", b"HP.."), 0x20);
        assert_eq!(parsed(b"\nIINST 002 Y\r"), some("IINST", "002"));
    }

    #[test]
    fn a_wrong_control_is_no_group() {
        assert_eq!(parsed(b"\nIINST 002 X\r"), None);
        assert_eq!(parsed(b"\nIINST 002 y\r"), None, "no folding of case");
    }

    #[test]
    fn a_control_of_a_space_is_right() {
        assert_eq!(parsed(b"\nPTEC HP..  \r"), some("PTEC", "HP.."));
        // Only with the separator and the control both there.
        assert_eq!(parsed(b"\nPTEC HP.. \r"), None, "no control");
        assert_eq!(parsed(b"\nPTEC HP..\r"), None, "no separator");
        // A space where another is due is no control.
        assert_eq!(parsed(b"\nIINST 002  \r"), None);
    }

    #[test]
    fn a_group_must_end_in_a_cr_after_its_control() {
        assert_eq!(parsed(b"\nIINST 002 Y"), None, "cut");
        assert_eq!(parsed(b"\nIINST 002 Yx\r"), None, "a tail");
        assert_eq!(parsed(b"\nIINST 002 Y \r"), None, "a space behind it");
        assert_eq!(parsed(b"\n"), None);
        assert_eq!(parsed(b"\nIINST"), None);
        assert_eq!(parsed(b"\nIINST 002"), None);
        assert_eq!(parsed(b"\nIINST 002 "), None);
    }

    #[test]
    fn white_space_is_as_in_a_format() {
        assert_eq!(parsed(b"\n\n IINST  002 \t Y\r"), some("IINST", "002"));
        assert_eq!(parsed(b"\nIINST\t002\tY\r"), some("IINST", "002"));
        assert_eq!(parsed(b"\nIINST\x0b002\x0cY\r"), some("IINST", "002"));
        assert_eq!(parsed(b"\nIINST\r\n002 Y\r"), some("IINST", "002"), "across a CR");
    }

    #[test]
    fn a_field_is_cut_at_its_width_and_goes_on_as_the_next() {
        // "BBRHCJBX 000000001 d": eight of a label.
        let eight = line("BBRHCJBX", "000000001");
        assert_eq!(parsed(&eight), some("BBRHCJBX", "000000001"));
        // Nine: the ninth is data, the rest does not fit the format.
        assert_eq!(parsed(&line("BBRHCJBXY", "000000001")), None);
        // Twelve of data are taken, thirteen overrun C's array, and a fourteenth is
        // read for the control.
        assert_eq!(parsed(&line("BASE", "000000000012")), some("BASE", "000000000012"));
        assert_eq!(parsed(&line("BASE", "0000000000013")), None);
        assert_eq!(parsed(&line("BASE", "00000000000014")), None);
        // A label's ninth byte starts the data, with no white space between.
        let mut text = b"\nBASEBASEB 1 ".to_vec();
        text.push(u8::try_from(control(b"BASEBASE", b"B")).unwrap());
        text.push(CR);
        assert_eq!(parsed(&text), None, "1 is the control here");
        let mut text = b"\nBASEBASEB ".to_vec();
        text.push(u8::try_from(control(b"BASEBASE", b"B")).unwrap());
        text.push(CR);
        assert_eq!(parsed(&text), some("BASEBASE", "B"));
    }

    #[test]
    fn a_nul_ends_the_text() {
        assert_eq!(parsed(b"\nIINST 0\x0002 Y\r"), None);
        assert_eq!(parsed(b"\nIINST 002\x00 Y\r"), None);
        assert_eq!(parsed(b"\nIINST 002 Y\x00\r"), None);
    }

    #[test]
    fn high_bytes_sum_as_signed_chars() {
        // 0xe9 is -23, so "BASE 000000001" has 'L', 0x4c, and with it 0x35, '5'.
        // tests/emu/teleinfo's "a high byte in data" shows C agrees.
        assert_eq!(parsed(b"\nBASE 000000001\xe9 5\r"), some("BASE", "000000001\u{fffd}"));
        assert_eq!(parsed(b"\nBASE 000000001\xe9 B\r"), None);
        assert_eq!(parsed(b"\nBASE 000000001 \xcc\r"), None, "a high control is never right");
    }

    #[test]
    fn the_next_group_is_behind_the_first_lf_and_ends_at_the_first_cr() {
        assert_eq!(next_group(b""), None);
        assert_eq!(next_group(b"xx"), None, "no LF");
        assert_eq!(next_group(b"xx\nIINST 002 Y"), None, "no CR behind it");
        assert_eq!(next_group(b"\rxx\n"), None, "a CR ahead of it is nothing");
        let text = b"xx\nIINST 002 Y\ryy";
        let (group, used) = next_group(text).unwrap();
        assert_eq!(group.unwrap().label, b"IINST");
        assert_eq!(&text[used..], b"yy");
        // A bad one uses its bytes as well, and a CR ends it where it stands.
        let text = b"\nIINST\r\n002 Y\r";
        let (group, used) = next_group(text).unwrap();
        assert_eq!(group.unwrap().data, b"002", "the fields ran across");
        assert_eq!(&text[used..], b"\n002 Y\r", "and the bytes go on from the CR");
        assert_eq!(next_group(b"\nnothing\r").unwrap(), (None, 9));
    }

    #[test]
    fn an_option_makes_its_channels() {
        let scan = |data: &str| optarif_of(&line("OPTARIF", data));
        assert_eq!(scan("BASE"), Some(Optarif::Base));
        assert_eq!(scan("HC.."), Some(Optarif::Hc));
        assert_eq!(scan("EJP."), Some(Optarif::Ejp));
        assert_eq!(scan("BBR("), Some(Optarif::Bbr));
        assert_eq!(scan("BBR"), Some(Optarif::Bbr), "a start");
        for none in ["TEMP", "HC", "base", "BASEX", "EJP", "HC.", ""] {
            assert_eq!(scan(none), None, "{none}");
        }
        assert_eq!(Optarif::Base.channels(), ["P", "BASE", "IINST", "PAPP"]);
        assert_eq!(Optarif::Hc.channels(), ["P", "HP", "HC", "IINST", "PAPP"]);
        assert_eq!(Optarif::Ejp.channels(), ["P", "HN", "HPM", "IINST", "PAPP"]);
        assert_eq!(
            Optarif::Bbr.channels(),
            ["P", "HPJB", "HPJW", "HPJR", "HCJB", "HCJW", "HCJR", "IINST", "PAPP"]
        );
    }

    #[test]
    fn the_last_option_of_a_buffer_says() {
        let mut text = line("OPTARIF", "BASE");
        text.extend(line("OPTARIF", "HC.."));
        assert_eq!(optarif_of(&text), Some(Optarif::Hc));
        text.extend(line("OPTARIF", "TEMP"));
        assert_eq!(optarif_of(&text), None, "an unknown one undoes it");
        let mut text = line("OPTARIF", "BASE");
        text.extend(b"\nOPTARIF TEMP x\r\nOPTARIF HC..");
        assert_eq!(optarif_of(&text), Some(Optarif::Base), "wrong and cut ones say nothing");
        assert_eq!(optarif_of(b""), None);
        assert_eq!(optarif_of(&line("ADCO", "031428097115")), None);
    }

    #[test]
    fn labels_mean_what_the_table_says() {
        assert_eq!(meaning(b"ADCO"), Some(Meaning::Sample));
        let reading = |label: &[u8]| match meaning(label) {
            Some(Meaning::Reading { channel, mq, unit }) => (channel, mq, unit),
            other => panic!("{other:?}"),
        };
        for (label, channel) in [
            (b"BASE".as_slice(), "BASE"),
            (b"HCHP", "HP"),
            (b"HCHC", "HC"),
            (b"EJPHN", "HN"),
            (b"EJPHPM", "HPM"),
            (b"BBRHPJB", "HPJB"),
            (b"BBRHPJW", "HPJW"),
            (b"BBRHPJR", "HPJR"),
            (b"BBRHCJB", "HCJB"),
            (b"BBRHCJW", "HCJW"),
            (b"BBRHCJR", "HCJR"),
        ] {
            assert_eq!(reading(label), (channel, Mq::Energy, Unit::WattHour));
        }
        assert_eq!(reading(b"IINST"), ("IINST", Mq::Current, Unit::Ampere));
        assert_eq!(reading(b"PAPP"), ("PAPP", Mq::Power, Unit::VoltAmpere));
        for none in [b"OPTARIF".as_slice(), b"PTEC", b"base", b"", b"P"] {
            assert_eq!(meaning(none), None);
        }
    }

    #[test]
    fn atoi_is_glibc_s() {
        for (text, value) in [
            ("000012345", 12345),
            ("0", 0),
            ("", 0),
            ("ABC", 0),
            ("12A", 12),
            ("1.5", 1),
            ("0x10", 0),
            ("....", 0),
            ("+12", 12),
            ("-12", -12),
            ("--1", 0),
            ("+", 0),
            ("2147483647", 2_147_483_647),
            ("2147483648", -2_147_483_648),
            ("4294967296", 0),
            ("9999999999", 1_410_065_407),
            ("999999999999", -727_379_969),
            // Stuck at LONG_MAX, whose low bits are all ones.
            ("99999999999999999999", -1),
            ("-99999999999999999999", 1),
        ] {
            assert_eq!(atoi(text.as_bytes()), value, "{text}");
        }
    }
}
