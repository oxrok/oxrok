// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Radix: how a decoded value is written, and how a decoder supports
//! changing it. Moved from oxrok-ui, so both front ends render the same
//! text for the same frame.
//!
//! Also the rows and the error heuristic over a [`DecoderInfo`]: which
//! annotation class sits in which row, and which classes are warnings.

use super::info::DecoderInfo;
use std::fmt::Write as _;

/// Numeric rendering.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub enum Radix {
    /// The decoder's own text.
    #[default]
    Native,
    /// `0x41`.
    Hex,
    /// `65`.
    Dec,
    /// `0b01000001`.
    Bin,
    /// `'A'`.
    Ascii,
}

impl Radix {
    /// Every radix.
    pub const ALL: [Radix; 5] = [Radix::Native, Radix::Bin, Radix::Dec, Radix::Hex, Radix::Ascii];

    /// The id on the wire and in the docs.
    pub fn id(self) -> &'static str {
        match self {
            Radix::Native => "native",
            Radix::Hex => "hex",
            Radix::Dec => "dec",
            Radix::Bin => "bin",
            Radix::Ascii => "ascii",
        }
    }

    /// The radix called `id`.
    pub fn from_id(id: &str) -> Option<Radix> {
        Radix::ALL.into_iter().find(|r| r.id() == id)
    }

    /// A label for people.
    pub fn label(self) -> &'static str {
        match self {
            Radix::Native => "Native",
            Radix::Hex => "Hexadecimal",
            Radix::Dec => "Decimal",
            Radix::Bin => "Binary",
            Radix::Ascii => "Ascii",
        }
    }

    /// The value of a sigrok decoder's `format` option for this radix;
    /// `None` leaves the decoder's default.
    pub fn sigrok_format(self) -> Option<&'static str> {
        match self {
            Radix::Native => None,
            Radix::Hex => Some("hex"),
            Radix::Dec => Some("dec"),
            Radix::Bin => Some("bin"),
            Radix::Ascii => Some("ascii"),
        }
    }
}

/// How a decoder's radix reaches its text.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RadixSupport {
    /// The decoder has a `format` option; changing the radix means
    /// decoding again.
    FormatOption,
    /// The decoder has binary output; the raw values are rendered by the
    /// front end, so the radix is free.
    RawValues,
    /// Neither: the decoder's text as it is.
    NativeOnly,
}

impl RadixSupport {
    /// The id on the wire.
    pub fn id(self) -> &'static str {
        match self {
            RadixSupport::FormatOption => "format_option",
            RadixSupport::RawValues => "raw_values",
            RadixSupport::NativeOnly => "native_only",
        }
    }

    /// How `info` supports a radix. With `prefer_raw`, a decoder that has
    /// both a `format` option and binary output (`uart`) is `RawValues`,
    /// so the radix costs nothing per request; without it the `format`
    /// option wins, as in the UI, whose texts then come from the decoder.
    pub fn of(info: &DecoderInfo, prefer_raw: bool) -> RadixSupport {
        let has_format = info.options.iter().any(|o| o.id == "format");
        let has_binary = !info.binary.is_empty();
        match (has_format, has_binary) {
            (true, true) if prefer_raw => RadixSupport::RawValues,
            (true, _) => RadixSupport::FormatOption,
            (false, true) => RadixSupport::RawValues,
            (false, false) => RadixSupport::NativeOnly,
        }
    }
}

/// The radix a new analyzer starts with.
pub fn default_radix(info: &DecoderInfo) -> Radix {
    if info.id == "uart" { Radix::Ascii } else { Radix::Native }
}

/// The annotation class whose raw value binary class `bin` gives: the
/// class with the binary class's id, or `<id>-data`.
pub fn binary_target(info: &DecoderInfo, bin: usize) -> Option<usize> {
    let id = info.binary.get(bin)?.id;
    let data = format!("{id}-data");
    info.annotations.iter().position(|a| a.id == id || a.id == data)
}

/// The raw value of a binary item of at most eight bytes, big-endian,
/// with its width in bits.
pub fn raw_value(bytes: &[u8]) -> Option<(u64, u32)> {
    if bytes.is_empty() || bytes.len() > 8 {
        return None;
    }
    let v = bytes.iter().fold(0u64, |v, &x| v << 8 | u64::from(x));
    Some((v, 8 * bytes.len() as u32))
}

/// Ascii: `\0 \t \n \r` escaped, other non-printables as `.`;
/// printable is 0x20–0x7E or ≥ 0xA0.
pub fn ascii_char(b: u8) -> String {
    match b {
        0 => "\\0".into(),
        b'\t' => "\\t".into(),
        b'\n' => "\\n".into(),
        b'\r' => "\\r".into(),
        0x20..=0x7E => (b as char).to_string(),
        0xA0..=0xFF => char::from(b).to_string(),
        _ => ".".into(),
    }
}

/// Byte arrays.
pub fn format_bytes(bytes: &[u8], radix: Radix) -> String {
    match radix {
        Radix::Hex => bytes.iter().fold("0x".to_owned(), |mut s, b| {
            let _ = write!(s, "{b:02X}");
            s
        }),
        Radix::Dec => bytes.iter().map(|&b| (b as i8).to_string()).collect::<Vec<_>>().join(" "),
        Radix::Bin => bytes.iter().fold("0b".to_owned(), |mut s, b| {
            let _ = write!(s, "{b:08b}");
            s
        }),
        Radix::Ascii => bytes.iter().map(|&b| ascii_char(b)).collect(),
        Radix::Native => bytes.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" "),
    }
}

/// Integers of `bits` width (8 for byte-sized decoder values, 64 for
/// 64-bit ones).
pub fn format_int(v: u64, bits: u32, radix: Radix) -> String {
    let hex_digits = bits.div_ceil(4) as usize;
    match radix {
        Radix::Hex => format!("0x{v:0hex_digits$X}"),
        Radix::Dec => v.to_string(),
        Radix::Bin => format!("0b{v:0width$b}", width = bits as usize),
        Radix::Ascii if bits <= 8 => format!("'{}'", ascii_char(v as u8)),
        Radix::Ascii => v.to_string(),
        // As decoders print values.
        Radix::Native => format!("{v:02X}"),
    }
}

/// `texts` with a trailing hex number equal to `value` (as decoders
/// print it, `%02X`) replaced by `value` in `radix`. Texts without it
/// are kept.
pub fn apply_radix(texts: &[String], value: u64, bits: u32, radix: Radix) -> Vec<String> {
    if radix == Radix::Native {
        return texts.to_vec();
    }
    let hex = format!("{value:02X}");
    let new = format_int(value, bits, radix);
    texts
        .iter()
        .map(|t| {
            let at = t.len().wrapping_sub(hex.len());
            let ends = t.len() >= hex.len()
                && t.is_char_boundary(at)
                && t[at..].eq_ignore_ascii_case(&hex);
            // A whole number: nothing alphanumeric right before it.
            let whole = ends && !t[..at].ends_with(|c: char| c.is_ascii_alphanumeric());
            if whole { format!("{}{new}", &t[..at]) } else { t.clone() }
        })
        .collect()
}

/// The row of each class: the first row that lists it, else a synthetic
/// row per class, numbered after the real rows.
pub fn class_rows(info: &DecoderInfo) -> Vec<usize> {
    let n_rows = info.annotation_rows.len();
    (0..info.annotations.len())
        .map(|c| {
            info.annotation_rows.iter().position(|r| r.classes.contains(&c)).unwrap_or(n_rows + c)
        })
        .collect()
}

/// Every row an analyzer can show: `(id, description)`, real rows
/// first, then the synthetic ones of classes no row lists.
pub fn all_rows(info: &DecoderInfo) -> Vec<(&'static str, &'static str)> {
    let mut rows: Vec<(&'static str, &'static str)> =
        info.annotation_rows.iter().map(|r| (r.id, r.desc)).collect();
    for (c, row) in class_rows(info).into_iter().enumerate() {
        if row >= info.annotation_rows.len() {
            let a = &info.annotations[c];
            rows.push((a.id, a.desc));
        }
    }
    rows
}

/// Error heuristic: the class id, or its row id.
pub fn is_error_class(info: &DecoderInfo, class: usize) -> bool {
    let id = info.annotations.get(class).map_or("", |a| a.id);
    let by_class =
        ["warning", "error", "-err", "frame-error", "parity-err"].iter().any(|k| id.contains(k));
    let row = class_rows(info).get(class).copied().and_then(|r| info.annotation_rows.get(r));
    by_class || row.is_some_and(|r| r.id == "warnings" || r.id.ends_with("-warnings"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(id: &'static str) -> &'static DecoderInfo {
        crate::decoders::ALL.iter().find(|d| d.info.id == id).map(|d| d.info).unwrap()
    }

    /// Where the radix comes from, and the starting radix.
    #[test]
    fn radix_support_and_defaults() {
        let (uart, spi, i2c) = (info("uart"), info("spi"), info("i2c"));
        assert_eq!(RadixSupport::of(uart, false), RadixSupport::FormatOption);
        assert_eq!(RadixSupport::of(uart, true), RadixSupport::RawValues);
        assert_eq!(RadixSupport::of(spi, false), RadixSupport::RawValues);
        assert_eq!(RadixSupport::of(i2c, false), RadixSupport::RawValues);
        assert_eq!(RadixSupport::of(info("timing"), true), RadixSupport::NativeOnly);
        assert_eq!(default_radix(uart), Radix::Ascii);
        assert_eq!(default_radix(spi), Radix::Native);
        // SPI `mosi` → `mosi-data`; I²C ids match exactly.
        let class = |d: &DecoderInfo, id: &str| d.annotations.iter().position(|a| a.id == id);
        let mosi = spi.binary.iter().position(|b| b.id == "mosi").unwrap();
        assert_eq!(binary_target(spi, mosi), class(spi, "mosi-data"));
        let ar = i2c.binary.iter().position(|b| b.id == "address-read").unwrap();
        assert_eq!(binary_target(i2c, ar), class(i2c, "address-read"));
        assert_eq!(Radix::from_id("ascii"), Some(Radix::Ascii));
        assert_eq!(Radix::Hex.sigrok_format(), Some("hex"));
        assert_eq!(raw_value(&[0x12, 0x34]), Some((0x1234, 16)));
        assert_eq!(raw_value(&[0; 9]), None);
    }

    /// U-12: radix formatting of bytes and integers.
    #[test]
    fn u12_radix_formatting() {
        let b = [0x48, 0x00, 0x0A, 0xFF];
        assert_eq!(format_bytes(&b, Radix::Hex), "0x48000AFF");
        assert_eq!(format_bytes(&b, Radix::Dec), "72 0 10 -1");
        assert_eq!(format_bytes(&b, Radix::Bin), "0b01001000000000000000101011111111");
        assert_eq!(format_bytes(&b, Radix::Ascii), "H\\0\\nÿ");
        assert_eq!(format_bytes(&[0x07, b'\t', b'\r', 0x7F], Radix::Ascii), ".\\t\\r.");
        assert_eq!(format_int(0x41, 8, Radix::Hex), "0x41");
        assert_eq!(format_int(0x41, 64, Radix::Hex), "0x0000000000000041");
        assert_eq!(format_int(5, 8, Radix::Bin), "0b00000101");
        assert_eq!(format_int(65, 8, Radix::Dec), "65");
        assert_eq!(format_int(65, 8, Radix::Ascii), "'A'");
        assert_eq!(format_int(7, 8, Radix::Ascii), "'.'");
    }

    /// Decoder texts with the value in the radix.
    #[test]
    fn radix_replaces_the_trailing_value() {
        let t = |v: &[&str]| v.iter().map(|s| (*s).to_owned()).collect::<Vec<_>>();
        let i2c = t(&["Data write: 3A", "DW: 3A", "3A"]);
        assert_eq!(
            apply_radix(&i2c, 0x3A, 8, Radix::Hex),
            t(&["Data write: 0x3A", "DW: 0x3A", "0x3A"])
        );
        assert_eq!(apply_radix(&i2c, 0x3A, 8, Radix::Dec), t(&["Data write: 58", "DW: 58", "58"]));
        assert_eq!(apply_radix(&i2c, 0x3A, 8, Radix::Ascii)[2], "':'");
        assert_eq!(apply_radix(&i2c, 0x3A, 8, Radix::Native), i2c);
        // Not the value, or part of a longer word: unchanged.
        assert_eq!(
            apply_radix(&t(&["Warning", "13A", "ä"]), 0x3A, 8, Radix::Dec),
            t(&["Warning", "13A", "ä"])
        );
        // A 16-bit SPI word.
        assert_eq!(apply_radix(&t(&["1234"]), 0x1234, 16, Radix::Bin), t(&["0b0001001000110100"]));
    }

    #[test]
    fn rows_and_errors() {
        let uart = info("uart");
        let rows = class_rows(uart);
        let data = uart.annotations.iter().position(|a| a.id == "rx-data").unwrap();
        assert_eq!(uart.annotation_rows[rows[data]].id, "rx-data");
        let warn = uart.annotations.iter().position(|a| a.id == "rx-warnings").unwrap();
        assert!(is_error_class(uart, warn));
        assert!(!is_error_class(uart, data));
        let all = all_rows(uart);
        assert_eq!(all.len(), uart.annotation_rows.len());
        assert!(all.iter().any(|(id, _)| *id == "rx-data"));
    }
}
