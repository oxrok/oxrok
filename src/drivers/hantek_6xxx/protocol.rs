// Counterpart in sigrok:
//   libsigrok/src/hardware/hantek-6xxx/protocol.c,
//   libsigrok/src/hardware/hantek-6xxx/protocol.h,
//   Copyright (C) 2015 Christer Ekholm <christerekholm@gmail.com>,
//   with commits by Uwe Hermann, Benjamin Larsson, Chriter, Erik Montnemery,
//   Romain Tartière, Markus Siegert, Timo Rothenpieler, litospain
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the Hantek 6022BE/BL and their relatives with
//! sigrok's scope firmware, and the pure parts of the driver.
//!
//! The FX2 boots under the board's own ids, takes the firmware through
//! its boot loader and renumerates as `1d50:608e`, `bcdDevice` telling
//! the boards apart. The firmware (`include/scope.inc` of
//! sigrok-firmware-fx2lafw) takes one byte vendor OUT requests: 0xe0 and
//! 0xe1 the input range of CH1 and CH2, 0xe2 the rate, 0xe3 start (1) or
//! stop (0), 0xe5 the coupling on the boards that have it (bit 0 CH1,
//! bit 4 CH2, set for DC). Every command stops a running acquisition
//! first. Samples come on bulk IN 6, one byte of CH1 and one of CH2,
//! each a point in the ten divisions of the range: 0 is -5 div, 255 is
//! +5 div.
//!
//! The driver keeps one transfer in flight with a 4 s timeout. The first
//! one, 1024 bytes, is thrown away for a glitch the firmware's start
//! leaves; the later ones are sized from what the limits leave, rounded
//! up to a power of two. All a transfer holds is passed on, also what
//! lies past the sample limit.

use crate::core::{Result, Value};
use crate::usb::{ControlRequest, Transport, UsbId};
use std::time::Duration;

/// The ids of every board once the firmware runs.
pub const FIRMWARE_ID: UsbId = UsbId { vendor: 0x1d50, product: 0x608e };
/// The endpoint the samples come from.
pub const ENDPOINT_IN: u8 = 0x86;
/// The interface the driver claims.
pub const INTERFACE: u8 = 0;
/// The configuration the upload selects.
pub const CONFIGURATION: u8 = 1;
/// `FLUSH_PACKET_SIZE`: the first transfer, thrown away.
pub const FLUSH_SIZE: usize = 1024;
const MIN_PACKET_SIZE: u64 = 512;
/// `MAX_PACKET_SIZE`.
const MAX_PACKET_SIZE: u64 = if cfg!(windows) { 2 * 1024 * 1024 } else { 12 * 1024 * 1024 };
/// `libusb_fill_bulk_transfer()`'s timeout. What a transfer holds when it
/// runs out is data like any other: a transfer can be longer than this
/// (1 MiB takes 5.2 s at 100 kHz).
pub const TRANSFER_TIMEOUT: Duration = Duration::from_millis(4000);
const CONTROL_TIMEOUT: Duration = Duration::from_millis(100);

const VDIV_CH1_REG: u8 = 0xe0;
const VDIV_CH2_REG: u8 = 0xe1;
const SAMPLERATE_REG: u8 = 0xe2;
const TRIGGER_REG: u8 = 0xe3;
const COUPLING_REG: u8 = 0xe5;

/// The default range, an index into the board's table: `DEFAULT_VOLTAGE`.
pub const DEFAULT_VOLTAGE: usize = 2;
/// The default rate: `DEFAULT_SAMPLERATE`.
pub const DEFAULT_SAMPLERATE: u64 = 8_000_000;
/// The rate `samplerate_to_reg()` falls back on for one off the list: its
/// last register, 100 kHz.
pub const FALLBACK_SAMPLERATE: u64 = 100_000;
/// `VDIV_MULTIPLIER`: the divisions of the screen.
const VDIV_MULTIPLIER: f32 = 10.0;

/// The rates `--show` lists, and the rate register of each.
pub const SAMPLERATES: [u64; 10] = [
    48_000_000, 30_000_000, 24_000_000, 16_000_000, 8_000_000, 4_000_000, 1_000_000, 500_000,
    200_000, 100_000,
];
const SAMPLERATE_REGS: [u8; 10] = [48, 30, 24, 16, 8, 4, 1, 50, 20, 10];
/// The range register of each index of a board's V/div table.
const VDIV_REGS: [u8; 7] = [1, 2, 5, 10, 11, 12, 13];

const VDIVS: [(u64, u64); 4] = [(1, 1), (500, 1000), (250, 1000), (100, 1000)];
const VDIVS_INSTRUSTAR: [(u64, u64); 6] =
    [(128, 100), (705, 1000), (288, 1000), (140, 1000), (576, 10000), (176, 10000)];
const DC_COUPLING: [&str; 1] = ["DC"];
const ACDC_COUPLING: [&str; 2] = ["AC", "DC"];

/// A board: `struct hantek_6xxx_profile`.
#[derive(Debug, PartialEq, Eq)]
pub struct Profile {
    /// The ids without firmware.
    pub orig: UsbId,
    /// `bcdDevice` with the firmware running.
    pub fw_version: u16,
    /// The vendor name.
    pub vendor: &'static str,
    /// The model name.
    pub model: &'static str,
    /// The firmware file.
    pub firmware: &'static str,
    /// What `coupling` may be set to.
    pub couplings: &'static [&'static str],
    /// Whether the board has the coupling request.
    pub has_coupling: bool,
    /// The ranges, volts per division as a fraction.
    pub vdivs: &'static [(u64, u64)],
}

const fn profile(
    orig: (u16, u16),
    fw_version: u16,
    names: (&'static str, &'static str, &'static str),
    ac_dc: bool,
    vdivs: &'static [(u64, u64)],
) -> Profile {
    Profile {
        orig: UsbId { vendor: orig.0, product: orig.1 },
        fw_version,
        vendor: names.0,
        model: names.1,
        firmware: names.2,
        couplings: if ac_dc { &ACDC_COUPLING } else { &DC_COUPLING },
        has_coupling: ac_dc,
        vdivs,
    }
}

const FW_6022BE: &str = "fx2lafw-hantek-6022be.fw";
const FW_6022BL: &str = "fx2lafw-hantek-6022bl.fw";
const FW_DDS120: &str = "fx2lafw-sainsmart-dds120.fw";
const FW_MDSO: &str = "fx2lafw-yixingdianzi-mdso.fw";
const FW_ISDS205B: &str = "fx2lafw-instrustar-isds205b.fw";

/// `dev_profiles[]`, in its order.
pub static PROFILES: [Profile; 8] = [
    profile((0x04b4, 0x6022), 1, ("Hantek", "6022BE", FW_6022BE), false, &VDIVS),
    profile((0x04b5, 0x6022), 1, ("Hantek", "6022BE", FW_6022BE), false, &VDIVS),
    profile((0x04b4, 0x2020), 1, ("Voltcraft", "DSO2020", FW_6022BE), false, &VDIVS),
    profile((0x8102, 0x8102), 2, ("Sainsmart", "DDS120", FW_DDS120), true, &VDIVS),
    profile((0x04b4, 0x602a), 3, ("Hantek", "6022BL", FW_6022BL), false, &VDIVS),
    profile((0x04b5, 0x602a), 3, ("Hantek", "6022BL", FW_6022BL), false, &VDIVS),
    profile((0xd4a2, 0x5660), 4, ("YiXingDianZi", "MDSO", FW_MDSO), false, &VDIVS),
    profile((0xd4a2, 0x5661), 5, ("Instrustar", "ISDS205B", FW_ISDS205B), true, &VDIVS_INSTRUSTAR),
];

/// Every id the scan looks for.
pub fn scan_ids() -> Vec<UsbId> {
    PROFILES.iter().map(|p| p.orig).chain([FIRMWARE_ID]).collect()
}

/// How a device found on the bus matches a board.
#[derive(Debug, PartialEq, Eq)]
pub enum Found {
    /// Under the board's own ids: the firmware has to be uploaded.
    Bare(&'static Profile),
    /// The firmware runs.
    Running(&'static Profile),
}

/// The scan's walk through `dev_profiles[]`: the first entry either of
/// whose ids matches.
pub fn find_profile(id: UsbId, device_version: u16) -> Option<Found> {
    PROFILES.iter().find_map(|p| {
        if p.orig == id {
            Some(Found::Bare(p))
        } else if id == FIRMWARE_ID && device_version == p.fw_version {
            Some(Found::Running(p))
        } else {
            None
        }
    })
}

/// The rate register of a rate of the list.
pub fn samplerate_reg(rate: u64) -> Option<u8> {
    SAMPLERATES.iter().position(|r| *r == rate).map(|i| SAMPLERATE_REGS[i])
}

/// The coupling of one channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Coupling {
    /// AC, bit clear.
    Ac,
    /// DC, bit set.
    Dc,
}

impl Coupling {
    /// The name `--get` prints and `-c` takes.
    pub fn name(self) -> &'static str {
        match self {
            Coupling::Ac => "AC",
            Coupling::Dc => "DC",
        }
    }
}

/// What `hantek_6xxx_update_coupling()` sends.
pub fn coupling_byte(coupling: [Coupling; 2]) -> u8 {
    let bit = |c| u8::from(c == Coupling::Dc);
    (bit(coupling[1]) << 4) | bit(coupling[0])
}

/// The limits of a capture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Limits {
    /// `limit_time`, 0 for none. It decides a transfer's size over the
    /// samples.
    pub limit_msec: u64,
    /// `limit_samples`, 0 for none.
    pub limit_samples: u64,
}

impl Limits {
    /// The checks of `receive_transfer()` after a transfer: whether the
    /// samples or the time are through.
    pub fn reached(self, received: u64, elapsed_ms: u64) -> bool {
        (self.limit_samples > 0 && received >= self.limit_samples)
            || (self.limit_msec > 0 && elapsed_ms >= self.limit_msec)
    }
}

/// The size of the next transfer, `data_amount()` with the `MIN()` of
/// `read_channel()`: enough for what is left of the limits, or one second
/// of samples without any, rounded up to a power of two, from 512 bytes
/// to `MAX_PACKET_SIZE`. `elapsed_ms` counts from the end of the flush.
///
/// In wide integers: the C code holds the byte count in a `uint32_t` and
/// rounds it with a `uint32_t`, which wraps to 0 past 2^31 and never ends
/// (api.c:540-541).
pub fn data_amount(limits: Limits, rate: u64, elapsed_ms: u64, received: u64) -> usize {
    const NUM_CHANNELS: u128 = 2;
    let data_left = if limits.limit_msec > 0 {
        let time_left = limits.limit_msec.saturating_sub(elapsed_ms);
        u128::from(rate) * u128::from(time_left) * NUM_CHANNELS / 1000
    } else if limits.limit_samples > 0 {
        u128::from(limits.limit_samples.saturating_sub(received)) * NUM_CHANNELS
    } else {
        u128::from(rate) * NUM_CHANNELS
    };
    let wanted = data_left.max(u128::from(MIN_PACKET_SIZE));
    let rounded = wanted.checked_next_power_of_two().unwrap_or(u128::MAX);
    rounded.min(u128::from(MAX_PACKET_SIZE)) as usize
}

/// How the bytes of one channel become volts: `send_chunk()`, in single
/// precision as there.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Scale {
    bit: f32,
    center: f32,
    /// The digits the packets say are significant.
    pub digits: i32,
}

impl Scale {
    /// The scale of a range, volts per division as a fraction.
    pub fn new(vdiv: (u64, u64)) -> Self {
        let range = vdiv.0 as f32 / vdiv.1 as f32 * VDIV_MULTIPLIER;
        let bit = range / 255.0;
        let log = bit.log10();
        // -(int)vdivlog + (vdivlog < 0.0)
        let digits = -(log as i32) + i32::from(log < 0.0);
        Scale { bit, center: range / 2.0, digits }
    }

    /// The volts of one byte.
    pub fn volts(self, byte: u8) -> f32 {
        self.bit * f32::from(byte) - self.center
    }
}

/// The samples of channel `ch` of a transfer. An odd last byte is no
/// sample.
pub fn channel_samples(data: &[u8], ch: usize, scale: Scale) -> Vec<f32> {
    data.as_chunks::<2>().0.iter().map(|s| scale.volts(s[ch])).collect()
}

/// The range of a V/div value in a board's table. Values come reduced
/// from the command line, as `500mV` is 1/2.
pub fn vdiv_index(vdivs: &[(u64, u64)], value: &Value) -> Option<usize> {
    let &Value::Rational(p, q) = value else { return None };
    vdivs.iter().position(|v| u128::from(v.0) * u128::from(q) == u128::from(p) * u128::from(v.1))
}

/// The transport with the firmware's commands. A failed command is
/// logged as `write_control()` does; the C driver goes on after a failed
/// setting, and so does [`super`].
pub struct Link {
    t: Box<dyn Transport>,
}

impl Link {
    /// A link over a device with the firmware running.
    pub fn new(t: Box<dyn Transport>) -> Self {
        Link { t }
    }

    /// `write_control()`.
    fn write_control(&mut self, reg: u8, value: u8) -> Result<()> {
        log::trace!("hantek_6xxx_write_control: 0x{reg:x} 0x{value:x}");
        self.t
            .control_out(ControlRequest::vendor(reg, 0, 0), &[value], CONTROL_TIMEOUT)
            .inspect_err(|e| {
                let name = crate::usb::libusb_error_name(e);
                log::error!("Failed to control transfer: 0x{reg:x}: {name}.");
            })
    }

    /// `hantek_6xxx_update_samplerate()`.
    pub fn update_samplerate(&mut self, rate: u64) {
        // A rate off the list never gets here: the set makes it the
        // fallback (see the driver), so samplerate_to_reg()'s error is
        // logged there.
        let reg = samplerate_reg(rate).unwrap_or(SAMPLERATE_REGS[SAMPLERATE_REGS.len() - 1]);
        log::debug!("update samplerate {reg}");
        let _ = self.write_control(SAMPLERATE_REG, reg);
    }

    /// `hantek_6xxx_update_vdiv()`: both channels, whichever changed.
    pub fn update_vdiv(&mut self, voltage: [usize; 2]) {
        let reg = |i: usize| VDIV_REGS[i.min(VDIV_REGS.len() - 1)];
        let (ch1, ch2) = (reg(voltage[0]), reg(voltage[1]));
        log::debug!("update vdiv {ch1} {ch2}");
        let _ = self.write_control(VDIV_CH1_REG, ch1);
        let _ = self.write_control(VDIV_CH2_REG, ch2);
    }

    /// `hantek_6xxx_update_coupling()`, on a board that has the request.
    pub fn update_coupling(&mut self, has_coupling: bool, coupling: [Coupling; 2]) {
        if has_coupling {
            let byte = coupling_byte(coupling);
            log::debug!("update coupling 0x{byte:x}");
            let _ = self.write_control(COUPLING_REG, byte);
        } else {
            log::debug!("coupling not supported");
        }
    }

    /// `hantek_6xxx_start_data_collecting()`.
    pub fn start(&mut self) -> Result<()> {
        log::debug!("trigger");
        self.write_control(TRIGGER_REG, 1)
    }

    /// `hantek_6xxx_stop_data_collecting()`.
    pub fn stop(&mut self) -> Result<()> {
        self.write_control(TRIGGER_REG, 0)
    }

    /// One transfer of samples, `len` bytes or what it held when its
    /// timeout ran out; nothing by then is
    /// [`crate::core::Error::Timeout`].
    pub fn read(&mut self, len: usize) -> Result<Vec<u8>> {
        log::debug!("Request channel data.");
        self.t.bulk_in(ENDPOINT_IN, len, TRANSFER_TIMEOUT)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;

    const NONE: Limits = Limits { limit_msec: 0, limit_samples: 0 };

    #[test]
    fn transfers_are_powers_of_two_from_512() {
        let samples = |n| Limits { limit_samples: n, ..NONE };
        assert_eq!(data_amount(samples(8), 8_000_000, 0, 0), 512);
        assert_eq!(data_amount(samples(256), 8_000_000, 0, 0), 512);
        assert_eq!(data_amount(samples(257), 8_000_000, 0, 0), 1024);
        assert_eq!(data_amount(samples(100_000), 100_000, 0, 0), 256 * 1024);
        // What is left, after a short transfer.
        assert_eq!(data_amount(samples(1000), 100_000, 0, 900), 512);
        assert_eq!(data_amount(samples(1000), 100_000, 0, 2000), 512);
    }

    /// api.c:530-532: the time left decides, not the samples.
    #[test]
    fn a_time_limit_sizes_the_transfer_from_the_time_left() {
        let both = Limits { limit_msec: 10, limit_samples: 300 };
        assert_eq!(data_amount(both, 100_000, 0, 0), 2048);
        assert_eq!(data_amount(both, 100_000, 4, 0), 2048);
        assert_eq!(data_amount(both, 100_000, 10, 0), 512);
        assert_eq!(data_amount(both, 100_000, 11, 0), 512);
    }

    #[test]
    fn without_a_limit_a_second_of_samples_up_to_the_maximum() {
        assert_eq!(data_amount(NONE, 100_000, 0, 0), 256 * 1024);
        assert_eq!(data_amount(NONE, 1_000_000, 0, 0), 2 * 1024 * 1024);
        assert_eq!(data_amount(NONE, 48_000_000, 0, 0), MAX_PACKET_SIZE as usize);
    }

    /// The limits where the C code's rounding never ends (above 2^31
    /// bytes in a `uint32_t`) or its count wraps.
    #[test]
    fn huge_limits_take_the_maximum() {
        for n in [1_500_000_000, 1 << 31, 1 << 32, u64::MAX] {
            let l = Limits { limit_samples: n, ..NONE };
            assert_eq!(data_amount(l, 100_000, 0, 0), MAX_PACKET_SIZE as usize, "{n}");
        }
        let l = Limits { limit_msec: 30_000, limit_samples: 0 };
        assert_eq!(data_amount(l, 48_000_000, 0, 0), MAX_PACKET_SIZE as usize);
        let l = Limits { limit_msec: u64::MAX, limit_samples: 0 };
        assert_eq!(data_amount(l, 48_000_000, 0, 0), MAX_PACKET_SIZE as usize);
    }

    /// `--time` alone cannot run as a case (wall time): the two checks
    /// of `receive_transfer()`, each alone.
    #[test]
    fn either_limit_ends_the_capture() {
        let samples = Limits { limit_samples: 100, ..NONE };
        assert!(!samples.reached(99, 1_000_000));
        assert!(samples.reached(100, 0));
        let time = Limits { limit_msec: 10, ..NONE };
        assert!(!time.reached(1 << 40, 9));
        assert!(time.reached(0, 10));
        assert!(!NONE.reached(1 << 40, 1 << 40));
    }

    #[test]
    fn volts_and_digits_follow_send_chunk() {
        let s = Scale::new((250, 1000));
        assert_eq!(s.digits, 3, "9.8 mV a step");
        assert!((s.volts(0) + 1.25).abs() < 1e-6);
        assert!((s.volts(255) - 1.25).abs() < 1e-6);
        assert_eq!(Scale::new((1, 1)).digits, 2, "39 mV a step");
        assert_eq!(Scale::new((100, 1000)).digits, 3, "3.9 mV a step");
        assert_eq!(Scale::new((176, 10000)).digits, 4, "0.69 mV a step");
    }

    #[test]
    fn a_transfer_splits_into_channels() {
        let s = Scale::new((1, 1));
        let data = [0u8, 255, 255, 0, 7];
        assert_eq!(channel_samples(&data, 0, s), [s.volts(0), s.volts(255)]);
        assert_eq!(channel_samples(&data, 1, s), [s.volts(255), s.volts(0)]);
    }

    #[test]
    fn the_profiles_follow_the_table() {
        let id = |vendor, product| UsbId { vendor, product };
        assert_eq!(find_profile(id(0x04b4, 0x6022), 0xa001), Some(Found::Bare(&PROFILES[0])));
        assert_eq!(find_profile(id(0x04b5, 0x602a), 0), Some(Found::Bare(&PROFILES[5])));
        assert_eq!(find_profile(FIRMWARE_ID, 1), Some(Found::Running(&PROFILES[0])));
        assert_eq!(find_profile(FIRMWARE_ID, 5), Some(Found::Running(&PROFILES[7])));
        assert_eq!(find_profile(FIRMWARE_ID, 9), None);
        assert_eq!(find_profile(id(0x1234, 0x5678), 1), None);
        assert_eq!(samplerate_reg(500_000), Some(50));
        assert_eq!(samplerate_reg(48_000_000), Some(48));
        assert_eq!(samplerate_reg(2_000_000), None);
    }

    #[test]
    fn coupling_bits() {
        assert_eq!(coupling_byte([Coupling::Dc, Coupling::Dc]), 0x11);
        assert_eq!(coupling_byte([Coupling::Ac, Coupling::Dc]), 0x10);
        assert_eq!(coupling_byte([Coupling::Dc, Coupling::Ac]), 0x01);
    }

    #[test]
    fn a_vdiv_matches_as_a_fraction() {
        let v = |p, q| Value::Rational(p, q);
        assert_eq!(vdiv_index(&VDIVS, &v(1, 2)), Some(1));
        assert_eq!(vdiv_index(&VDIVS, &v(500, 1000)), Some(1));
        assert_eq!(vdiv_index(&VDIVS, &v(2, 1)), None);
        assert_eq!(vdiv_index(&VDIVS_INSTRUSTAR, &v(141, 200)), Some(1));
        assert_eq!(vdiv_index(&VDIVS, &Value::U64(1)), None);
    }

    #[test]
    fn commands_go_as_the_c_driver_sends_them() {
        let text = "# oxrok usb fixture 1\n\
                    device vid=1d50 pid=608e\n\
                    ctl_out 40 e2 0000 0000 08 -> ok\n\
                    ctl_out 40 e0 0000 0000 05 -> ok\n\
                    ctl_out 40 e1 0000 0000 0a -> ok\n\
                    ctl_out 40 e5 0000 0000 10 -> ok\n\
                    ctl_out 40 e3 0000 0000 01 -> ok\n\
                    ctl_out 40 e3 0000 0000 00 -> ok\n";
        let mut l = Link::new(Box::new(Replay::from_text(text).unwrap()));
        l.update_samplerate(8_000_000);
        l.update_vdiv([2, 3]);
        l.update_coupling(false, [Coupling::Ac, Coupling::Dc]);
        l.update_coupling(true, [Coupling::Ac, Coupling::Dc]);
        l.start().unwrap();
        l.stop().unwrap();
    }
}
