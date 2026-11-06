// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A UNI-T UT181A behind the CP2110 HID bridge: the cable the meter ships with,
//! the UT-D09, as the `cp2110` model's chip with the `uni-t-ut181a` model's meter
//! on its UART. libsigrok's driver names this cable as the meter's; its USB ids
//! are the chip's, as that model has them. Options are the meter's.
//!
//! Emulation artifact: a read's offer is an interrupt IN poll here, about 1 ms
//! apart rather than the CDC shell's 20 ms, so `period=` counts faster, and the
//! FIFO's 480 bytes bound what one answer may hold.

use crate::Result;
use crate::model::DeviceModel;
use crate::models::cp2110::Cp2110;
use crate::models::uni_t_ut181a::Meter;

/// The model name.
pub const NAME: &str = "uni-t-ut181a-cp2110";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A UNI-T UT181A behind a CP2110 HID bridge; the options of uni-t-ut181a \
                         but uart=";

/// A cable and a meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(Cp2110::with_meter(NAME, Meter::from_options(options)?)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Reply, Setup, Violations};

    fn set_feature(dev: &mut dyn DeviceModel, data: &[u8], v: &mut Violations) {
        let s = Setup {
            request_type: 0x21,
            request: 9,
            value: 0x0300 | u16::from(data[0]),
            index: 0,
            length: data.len() as u16,
        };
        assert!(matches!(dev.control(s, data, v), Reply::Data(_)));
    }

    fn cable(baud: u32) -> (Box<dyn DeviceModel>, Violations) {
        let mut dev = build(&[]).unwrap();
        let mut v = Violations::default();
        set_feature(dev.as_mut(), &[0x41, 1], &mut v);
        let b = baud.to_be_bytes();
        set_feature(dev.as_mut(), &[0x50, b[0], b[1], b[2], b[3], 0, 0, 3, 0], &mut v);
        (dev, v)
    }

    /// Monitor on, as one data report.
    const MONITOR_ON: [u8; 9] = [8, 0xab, 0xcd, 4, 0, 5, 1, 0x0a, 0];

    fn read(dev: &mut dyn DeviceModel, v: &mut Violations) -> Vec<u8> {
        let mut out = Vec::new();
        for report in crate::shells::cdc::host::drain_offers(|| dev.in_transfer(1, 64, v)) {
            assert_eq!(usize::from(report[0]), report.len() - 1);
            out.extend_from_slice(&report[1..]);
        }
        out
    }

    #[test]
    fn a_command_in_a_report_gets_its_answer_in_reports() {
        let (mut dev, mut v) = cable(9600);
        dev.out(1, &MONITOR_ON, &mut v);
        let got = read(dev.as_mut(), &mut v);
        // The code and a reading of 25 bytes, across reports where it must.
        assert_eq!(got.len(), 9 + 25);
        assert_eq!(got[..2], [0xab, 0xcd]);
        assert_eq!(v.drain().len(), 0);
    }

    #[test]
    fn at_another_rate_nothing_passes() {
        let (mut dev, mut v) = cable(19_200);
        dev.out(1, &MONITOR_ON, &mut v);
        assert_eq!(read(dev.as_mut(), &mut v), [0u8; 0]);
    }

    #[test]
    fn what_the_meter_flags_is_reported() {
        let (mut dev, mut v) = cable(9600);
        dev.out(1, &[2, 0x00, 0x00], &mut v);
        assert_eq!(v.drain().len(), 1);
    }

    #[test]
    fn a_reset_is_a_power_on_of_both() {
        let (mut dev, mut v) = cable(9600);
        dev.out(1, &MONITOR_ON, &mut v);
        read(dev.as_mut(), &mut v);
        dev.reset();
        // The UART is off again, so the same report is flagged and goes nowhere.
        dev.out(1, &MONITOR_ON, &mut v);
        assert_eq!(read(dev.as_mut(), &mut v), [0u8; 0]);
        assert_eq!(v.drain().len(), 1);
    }
}
