// Counterpart in sigrok:
//   libsigrok/src/lcr/es51919.c,
//   Copyright (C) 2014 Janne Huttunen <jahuttun@gmail.com>,
//   Copyright (C) 2019 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Daniel Elstner, Frank Stettner, Alexandru Gagniuc, Aurelien Jacobs,
//   Daniel Glöckner, Martin Ling
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Cyrustek ES51919 LCR chipset, as `src/lcr/es51919.c` has reverse engineered
//! it (no official description is public): the meter streams 17 byte packets.
//!
//! ```text
//!  0  1   00 0d
//!  2      flags: 0 hold, 1 reference, 2 delta, 3 calibration, 4 sorting,
//!         5 LCR, 6 auto, 7 parallel
//!  3      bits 5-7 test frequency code: 100, 120, 1 k, 10 k, 100 k, DC
//!  4      tolerance (sorting mode)
//!  5- 9   primary: quantity, value (big endian, signed), info, status
//! 10-14   secondary, the same; info is bits 0-2 the decimals, bits 3-7 the
//!         unit; status bits 0-3: 0 normal, 3 outside limits, others blank
//! 15 16   0d 0a
//! ```
//!
//! Both are in `parse_mq` and the other tables as in C, with the sign of the
//! value (`int16_t`). Another program written against a DE-5000
//! (`4x1md/de5000_lcr_py`) takes only a secondary in % or degrees as signed; C's
//! signed primary is kept, as nothing shows a real meter sending a high bit
//! there. A value outside limits is `value * INFINITY`, so a zero there is NaN,
//! as in C.

use super::{Reading, Settings, pow10};
use crate::core::{Mq, MqFlags, Unit};

/// A packet's size.
pub const PACKET_SIZE: usize = 17;

/// `frequencies[]`, the list a `--show` gives.
pub const FREQUENCIES: [f64; 6] = [0.0, 100.0, 120.0, 1000.0, 10_000.0, 100_000.0];

/// `circuit_models[]`.
pub const CIRCUIT_MODELS: [&str; 4] = ["NONE", "PARALLEL", "SERIES", "AUTO"];

/// The frequency of each code; the last is DC, which `frequencies[]` has first.
const FREQUENCY_OF_CODE: [u64; 6] = [100, 120, 1000, 10_000, 100_000, 0];

/// The C's `units[]`: the unit and its power of ten. Line 4 is `-1`, no unit
/// libsigrok has, which its output shows as it shows none.
const UNITS: [(Unit, i32); 15] = [
    (Unit::Unitless, 0),
    (Unit::Ohm, 0),
    (Unit::Ohm, 3),
    (Unit::Ohm, 6),
    (Unit::Unitless, 0),
    (Unit::Henry, -6),
    (Unit::Henry, -3),
    (Unit::Henry, 0),
    (Unit::Henry, 3),
    (Unit::Farad, -12),
    (Unit::Farad, -9),
    (Unit::Farad, -6),
    (Unit::Farad, -3),
    (Unit::Percentage, 0),
    (Unit::Degree, 0),
];

/// The frame: fixed head and tail.
pub fn valid(pkt: &[u8]) -> bool {
    pkt[0] == 0x00 && pkt[1] == 0x0d && pkt[15] == 0x0d && pkt[16] == 0x0a
}

fn display(pkt: &[u8], secondary: bool) -> &[u8] {
    if secondary { &pkt[10..15] } else { &pkt[5..10] }
}

fn parse_mq(pkt: &[u8], secondary: bool, parallel: bool) -> Option<Mq> {
    let code = u16::from(secondary) << 8 | u16::from(display(pkt, secondary)[0]);
    match code {
        0x001 => Some(if parallel { Mq::ParallelInductance } else { Mq::SeriesInductance }),
        0x002 => Some(if parallel { Mq::ParallelCapacitance } else { Mq::SeriesCapacitance }),
        0x003 | 0x103 => Some(if parallel { Mq::ParallelResistance } else { Mq::SeriesResistance }),
        0x004 => Some(Mq::Resistance),
        0x100 => Some(Mq::Difference),
        0x101 => Some(Mq::DissipationFactor),
        0x102 => Some(Mq::QualityFactor),
        0x104 => Some(Mq::PhaseAngle),
        _ => {
            log::error!("Unknown quantity 0x{code:03x}.");
            None
        }
    }
}

/// The meter's settings. As in C, the model asks for the primary quantity, and
/// so logs an unknown one, unless the meter is on auto.
pub fn settings(pkt: &[u8]) -> Settings {
    let code = usize::from(pkt[3] >> 5);
    let output_freq = FREQUENCY_OF_CODE.get(code).copied().unwrap_or_else(|| {
        log::error!("Unknown output frequency code {code}.");
        0
    });
    let circuit_model = if pkt[2] & 0x40 != 0 {
        "AUTO"
    } else if parse_mq(pkt, false, false) == Some(Mq::Resistance) {
        "NONE"
    } else if pkt[2] & 0x80 != 0 {
        "PARALLEL"
    } else {
        "SERIES"
    };
    Settings { output_freq, circuit_model }
}

/// What a display reads: nothing when it is blank, in calibration or sorting
/// mode, or on a quantity or unit the table does not have.
pub fn reading(pkt: &[u8], secondary: bool) -> Option<Reading> {
    let buf = display(pkt, secondary);
    let state = buf[4] & 0x0f;
    if state != 0 && state != 3 {
        return None;
    }
    if pkt[2] & 0x18 != 0 {
        // Calibration and sorting modes are not supported.
        return None;
    }
    let mut flags = MqFlags::empty();
    if secondary {
        if pkt[2] & 0x04 != 0 {
            flags |= MqFlags::RELATIVE;
        }
    } else {
        if pkt[2] & 0x01 != 0 {
            flags |= MqFlags::HOLD;
        }
        if pkt[2] & 0x02 != 0 {
            flags |= MqFlags::REFERENCE;
        }
    }
    let mq = parse_mq(pkt, secondary, pkt[2] & 0x80 != 0)?;
    let unit_code = buf[3] >> 3;
    let Some(&(unit, exponent)) = UNITS.get(usize::from(unit_code)) else {
        log::error!("Unknown unit {unit_code}.");
        return None;
    };
    let decimals = i32::from(buf[3] & 7);
    let raw = i16::from_be_bytes([buf[1], buf[2]]);
    let mut value = f32::from(raw) * pow10(-decimals);
    value *= if state == 0 { pow10(exponent) } else { f32::INFINITY };
    Some(Reading { value, mq, flags, unit, digits: decimals - exponent })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Flags, config, then quantity, value, info and status of each display.
    fn packet(flags: u8, config: u8, main: [u8; 5], second: [u8; 5]) -> [u8; 17] {
        let mut p = [0u8; 17];
        p[1] = 0x0d;
        p[2] = flags;
        p[3] = config;
        p[5..10].copy_from_slice(&main);
        p[10..15].copy_from_slice(&second);
        p[15] = 0x0d;
        p[16] = 0x0a;
        p
    }

    const MAIN: [u8; 5] = [2, 0x30, 0x39, 0x53, 0];
    const SECOND: [u8; 5] = [1, 0, 0x7b, 0x03, 0];

    #[test]
    fn the_frame_is_head_and_tail() {
        let p = packet(0, 0, MAIN, SECOND);
        assert!(valid(&p));
        for (at, byte) in [(0, 1), (1, 0x0c), (15, 0x0a), (16, 0x0d)] {
            let mut q = p;
            q[at] = byte;
            assert!(!valid(&q), "{at}");
        }
    }

    #[test]
    fn a_capacitance_and_its_factor() {
        let p = packet(0x20, 0x50, MAIN, SECOND);
        let r = reading(&p, false).unwrap();
        assert_eq!(r.mq, Mq::SeriesCapacitance);
        assert_eq!(r.unit, Unit::Farad);
        assert_eq!(r.digits, 12);
        assert_eq!(r.value, 12345.0 * pow10(-3) * pow10(-9));
        assert!((r.value - 1.2345e-8).abs() < 1e-13);
        let r = reading(&p, true).unwrap();
        assert_eq!(r.mq, Mq::DissipationFactor);
        assert_eq!(r.unit, Unit::Unitless);
        assert_eq!(r.digits, 3);
        assert_eq!(r.value, 123.0 * pow10(-3));
    }

    #[test]
    fn the_value_is_signed() {
        let p = packet(0, 0, [2, 0xff, 0xff, 0x50, 0], SECOND);
        assert_eq!(reading(&p, false).unwrap().value, -pow10(-9));
        let p = packet(0, 0, [2, 0x80, 0x00, 0x50, 0], SECOND);
        assert_eq!(reading(&p, false).unwrap().value, -32768.0 * pow10(-9));
    }

    #[test]
    fn outside_limits_is_infinite_and_a_zero_nan() {
        let p = packet(0, 0, [2, 0x4e, 0x20, 0x53, 3], SECOND);
        assert_eq!(reading(&p, false).unwrap().value, f32::INFINITY);
        let p = packet(0, 0, [2, 0xff, 0xff, 0x53, 3], SECOND);
        assert_eq!(reading(&p, false).unwrap().value, f32::NEG_INFINITY);
        let p = packet(0, 0, [2, 0, 0, 0x53, 3], SECOND);
        assert!(reading(&p, false).unwrap().value.is_nan());
    }

    #[test]
    fn status_is_its_low_nibble_and_only_0_and_3_show() {
        for state in 0..=255u8 {
            let p = packet(0, 0, [2, 0, 1, 0x53, state], SECOND);
            assert_eq!(reading(&p, false).is_some(), matches!(state & 15, 0 | 3), "{state}");
        }
    }

    #[test]
    fn flags_go_to_their_display() {
        let p = packet(0x07, 0, MAIN, SECOND);
        let main = reading(&p, false).unwrap().flags;
        assert!(main.contains(MqFlags::HOLD) && main.contains(MqFlags::REFERENCE));
        assert!(!main.contains(MqFlags::RELATIVE));
        let second = reading(&p, true).unwrap().flags;
        assert!(second.contains(MqFlags::RELATIVE));
        assert!(!second.contains(MqFlags::HOLD) && !second.contains(MqFlags::REFERENCE));
        // Each bit alone.
        let flags =
            |bits, secondary| reading(&packet(bits, 0, MAIN, SECOND), secondary).unwrap().flags;
        assert_eq!(flags(0x01, false), MqFlags::HOLD);
        assert_eq!(flags(0x02, false), MqFlags::REFERENCE);
        assert!(flags(0x04, false).is_empty());
        assert!(flags(0x01, true).is_empty());
        assert!(flags(0x02, true).is_empty());
        assert_eq!(flags(0x04, true), MqFlags::RELATIVE);
    }

    #[test]
    fn calibration_and_sorting_read_nothing() {
        for flags in [0x08, 0x10, 0x18] {
            let p = packet(flags, 0, MAIN, SECOND);
            assert_eq!(reading(&p, false), None);
            assert_eq!(reading(&p, true), None);
        }
    }

    #[test]
    fn each_quantity_on_each_display() {
        let mq = |q, flags, secondary| {
            let p = if secondary {
                packet(flags, 0, MAIN, [q, 0, 1, 0, 0])
            } else {
                packet(flags, 0, [q, 0, 1, 0, 0], SECOND)
            };
            reading(&p, secondary).map(|r| r.mq)
        };
        assert_eq!(mq(1, 0, false), Some(Mq::SeriesInductance));
        assert_eq!(mq(1, 0x80, false), Some(Mq::ParallelInductance));
        assert_eq!(mq(2, 0x80, false), Some(Mq::ParallelCapacitance));
        assert_eq!(mq(3, 0, false), Some(Mq::SeriesResistance));
        assert_eq!(mq(3, 0x80, false), Some(Mq::ParallelResistance));
        assert_eq!(mq(4, 0x80, false), Some(Mq::Resistance));
        assert_eq!(mq(0, 0, false), None);
        assert_eq!(mq(5, 0, false), None);
        assert_eq!(mq(0, 0, true), Some(Mq::Difference));
        assert_eq!(mq(1, 0, true), Some(Mq::DissipationFactor));
        assert_eq!(mq(2, 0, true), Some(Mq::QualityFactor));
        assert_eq!(mq(3, 0x80, true), Some(Mq::ParallelResistance));
        assert_eq!(mq(4, 0, true), Some(Mq::PhaseAngle));
        assert_eq!(mq(5, 0, true), None);
    }

    #[test]
    fn units_and_their_powers() {
        let at = |info| {
            let p = packet(0, 0, [2, 0, 1, info, 0], SECOND);
            reading(&p, false).map(|r| (r.unit, r.digits))
        };
        assert_eq!(at(0 << 3), Some((Unit::Unitless, 0)));
        assert_eq!(at(2 << 3), Some((Unit::Ohm, -3)));
        assert_eq!(at(3 << 3 | 2), Some((Unit::Ohm, -4)));
        assert_eq!(at(4 << 3), Some((Unit::Unitless, 0)));
        assert_eq!(at(5 << 3), Some((Unit::Henry, 6)));
        assert_eq!(at(9 << 3 | 1), Some((Unit::Farad, 13)));
        assert_eq!(at(13 << 3), Some((Unit::Percentage, 0)));
        assert_eq!(at(14 << 3), Some((Unit::Degree, 0)));
        assert_eq!(at(15 << 3), None);
        assert_eq!(at(31 << 3), None);
    }

    #[test]
    fn settings_of_a_packet() {
        let freq = |config| settings(&packet(0, config, MAIN, SECOND)).output_freq;
        assert_eq!(
            [freq(0x10), freq(0x30), freq(0x50), freq(0x70), freq(0x90), freq(0xb0)],
            [100, 120, 1000, 10_000, 100_000, 0]
        );
        assert_eq!(freq(0xd0), 0, "unknown code");
        assert_eq!(freq(0xf0), 0, "unknown code");
        let model = |flags, q| settings(&packet(flags, 0, [q, 0, 1, 0, 0], SECOND)).circuit_model;
        assert_eq!(model(0, 2), "SERIES");
        assert_eq!(model(0x80, 2), "PARALLEL");
        assert_eq!(model(0xc0, 2), "AUTO");
        assert_eq!(model(0x80, 4), "NONE", "plain resistance");
        assert_eq!(model(0x40, 4), "AUTO", "auto first");
        assert_eq!(model(0x80, 7), "PARALLEL", "an unknown quantity");
    }
}
