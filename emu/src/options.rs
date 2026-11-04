// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The values of a model's `-o` options, which any shell's model may parse.

use crate::{Error, Result};

/// Parses `KEY=HEX[,HEX...]`: one or more byte strings, each of `min` to
/// `max` bytes.
///
/// # Errors
///
/// On a malformed hex string or a length out of range.
pub fn parse_hex_list(key: &str, value: &str, min: usize, max: usize) -> Result<Vec<Vec<u8>>> {
    let mut out = Vec::new();
    for hex in value.split(',') {
        if !hex.len().is_multiple_of(2) || !hex.bytes().all(|c| c.is_ascii_hexdigit()) {
            return Err(Error(format!("{key}={hex}: an even number of hex digits")));
        }
        let bytes: Vec<u8> = (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).expect("hex digits"))
            .collect();
        if !(min..=max).contains(&bytes.len()) {
            return Err(Error(format!("{key}={hex}: {min} to {max} bytes")));
        }
        out.push(bytes);
    }
    Ok(out)
}

/// Parses a decimal option within `range`.
///
/// # Errors
///
/// When it is not a number in the range.
pub fn parse_number(key: &str, value: &str, range: std::ops::RangeInclusive<u32>) -> Result<u32> {
    value
        .parse()
        .ok()
        .filter(|n| range.contains(n))
        .ok_or_else(|| Error(format!("{key}={value}: {} to {}", range.start(), range.end())))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_lists_parse_and_their_errors_name_the_option() {
        assert_eq!(parse_hex_list("idn", "4b,4c4d", 1, 2).unwrap(), [vec![0x4b], vec![0x4c, 0x4d]]);
        for (value, says) in
            [("4", "idn=4: an even"), ("zz", "idn=zz: an even"), ("4b4c4d", "1 to 2")]
        {
            let e = parse_hex_list("idn", value, 1, 2).unwrap_err();
            assert!(e.0.contains(says), "{}", e.0);
        }
    }

    #[test]
    fn numbers_keep_to_their_range() {
        assert_eq!(parse_number("n", "3", 1..=3).unwrap(), 3);
        for bad in ["4", "0", "x", ""] {
            assert!(parse_number("n", bad, 1..=3).is_err(), "{bad}");
        }
    }
}
