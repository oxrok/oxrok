// Counterpart in sigrok:
//   libsigrok/src/hardware/atten-pps3xxx/protocol.c,
//   libsigrok/src/hardware/atten-pps3xxx/protocol.h,
//   libsigrok/src/hardware/atten-pps3xxx/api.c (the models table),
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Martin Ling, Aurelien Jacobs,
//   Daniel Elstner, Lars-Peter Clausen, Gerhard Sittig, Soeren Apel,
//   Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the Atten PPS3000 series.
//!
//! Fixed frames of 24 bytes, at 9600 baud with a ninth data bit nobody uses,
//! which 8n2 absorbs. The device sends a frame only when it gets one.
//!
//! | Byte | Meaning |
//! |---|---|
//! | 0 | `aa` |
//! | 1 | `aa` asks, `20` sets |
//! | 2-13 | each channel's volts x 100 and amperes x 1000, big endian |
//! | 14, 16 | 1 |
//! | 15 | outputs on, a mask |
//! | 18 | over-current protection |
//! | 19 | 0 independent, 1 series, 2 parallel |
//! | 23 | the sum of the rest, in a `20` frame |
//!
//! A frame sets everything: what it does not mean to change has to be sent
//! as well, and the device has no way to read a setpoint back (the sigrok
//! wiki). The answer holds what is measured in the same places.

/// A frame's length, both ways.
pub const FRAME: usize = 24;
/// The channels of the one model the C driver has.
pub const CHANNELS: usize = 3;
/// The channel modes, by the number byte 19 has.
pub const MODES: [&str; 3] = ["Independent", "Series", "Parallel"];

/// Minimum, maximum and step.
pub type Range = [f64; 3];

/// Each channel's voltage range.
pub const VOLTAGE: [Range; CHANNELS] = [[0.0, 32.0, 0.01], [0.0, 32.0, 0.01], [0.0, 6.0, 0.01]];
/// Each channel's current range.
pub const CURRENT: [Range; CHANNELS] = [[0.0, 3.0, 0.001]; CHANNELS];

const HEADER: u8 = 0xaa;
const COMMAND_ASK: u8 = 0xaa;
const COMMAND_SET: u8 = 0x20;

/// The frame that asks and changes nothing. It carries no sum.
pub fn ask() -> [u8; FRAME] {
    let mut frame = [0; FRAME];
    frame[0] = HEADER;
    frame[1] = COMMAND_ASK;
    frame
}

/// One channel's part of what the frontend sets.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Setpoint {
    /// In volts.
    pub voltage: f64,
    /// In amperes.
    pub current: f64,
    /// Whether the output is on.
    pub enabled: bool,
}

/// All a frame sets. Its default is what nothing has set yet.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Settings {
    /// Per channel.
    pub channels: [Setpoint; CHANNELS],
    /// Over-current protection.
    pub ocp: bool,
    /// An index into [`MODES`].
    pub mode: u8,
}

/// The nearest step. C truncates, and with a value `sigrok-cli` parses in
/// single precision that gives 3.29 V for 3.3 (`protocol.c:121,124`). The
/// ranges were checked before, and they fit.
fn units(value: f64, per_unit: f64) -> [u8; 2] {
    ((value * per_unit).round() as u16).to_be_bytes()
}

impl Settings {
    /// The frame that sets all of this.
    pub fn frame(&self) -> [u8; FRAME] {
        let mut frame = [0; FRAME];
        frame[0] = HEADER;
        frame[1] = COMMAND_SET;
        frame[14] = 1;
        frame[16] = 1;
        for (i, channel) in self.channels.iter().enumerate() {
            frame[2 + i * 4..4 + i * 4].copy_from_slice(&units(channel.voltage, 100.0));
            frame[4 + i * 4..6 + i * 4].copy_from_slice(&units(channel.current, 1000.0));
            if channel.enabled {
                frame[15] |= 1 << i;
            }
        }
        frame[18] = u8::from(self.ocp);
        frame[19] = self.mode;
        frame[FRAME - 1] = frame[..FRAME - 1].iter().fold(0, |sum: u8, b| sum.wrapping_add(*b));
        frame
    }
}

/// What came from the device. Its default is what nothing has said yet.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Reported {
    /// Volts per channel, which C keeps in single precision on the way.
    pub voltage: [f32; CHANNELS],
    /// Amperes per channel, likewise.
    pub current: [f32; CHANNELS],
    /// Which outputs are on.
    pub enabled: [bool; CHANNELS],
    /// Over-current protection.
    pub ocp: bool,
    /// An index into [`MODES`].
    pub mode: u8,
}

impl Reported {
    /// Takes in an answer. A mode that is none keeps the last, as in C.
    pub fn update(&mut self, frame: &[u8; FRAME]) {
        let value = |at: usize| u16::from_be_bytes([frame[at], frame[at + 1]]);
        for i in 0..CHANNELS {
            self.voltage[i] = (f64::from(value(2 + i * 4)) / 100.0) as f32;
            self.current[i] = (f64::from(value(4 + i * 4)) / 1000.0) as f32;
            self.enabled[i] = frame[15] & (1 << i) != 0;
        }
        self.ocp = frame[18] != 0;
        if usize::from(frame[19]) < MODES.len() {
            self.mode = frame[19];
        }
    }
}

/// Whether a scan's answer is one of this family's.
pub fn is_scan_answer(frame: &[u8]) -> bool {
    frame.len() == FRAME && frame[0] == HEADER && frame[1] == COMMAND_ASK
}

/// Whether a frame starts as all of them do. C reads any 24 bytes as an
/// answer, so a stream that slipped gives made up readings.
pub fn has_header(frame: &[u8; FRAME]) -> bool {
    frame[0] == HEADER
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn asking_is_two_bytes_and_zeros() {
        let f = ask();
        assert_eq!(f[..2], [0xaa, 0xaa]);
        assert!(f[2..].iter().all(|b| *b == 0), "no sum either: the C sends none");
    }

    #[test]
    fn the_frame_of_nothing_set_is_all_off() {
        let f = Settings::default().frame();
        assert_eq!(f[..2], [0xaa, 0x20]);
        assert_eq!((f[14], f[15], f[16]), (1, 0, 1));
        assert_eq!(f[23], 0xcc);
    }

    #[test]
    fn a_frame_has_each_field_in_its_place() {
        let mut s = Settings { ocp: true, mode: 2, ..Settings::default() };
        s.channels[0] = Setpoint { voltage: 5.0, current: 0.5, enabled: true };
        s.channels[1] = Setpoint { voltage: 32.0, current: 3.0, enabled: false };
        s.channels[2] = Setpoint { voltage: 6.0, current: 0.025, enabled: true };
        let f = s.frame();
        assert_eq!(f[2..6], [0x01, 0xf4, 0x01, 0xf4]);
        assert_eq!(f[6..10], [0x0c, 0x80, 0x0b, 0xb8]);
        assert_eq!(f[10..14], [0x02, 0x58, 0x00, 0x19]);
        assert_eq!((f[15], f[18], f[19]), (0b101, 1, 2));
        let sum = f[..23].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        assert_eq!(f[23], sum);
    }

    #[test]
    fn values_go_to_the_nearest_step() {
        let frame = |voltage, current| {
            let mut s = Settings::default();
            s.channels[0] = Setpoint { voltage, current, enabled: false };
            let f = s.frame();
            (u16::from_be_bytes([f[2], f[3]]), u16::from_be_bytes([f[4], f[5]]))
        };
        // What C truncates to 509 and 289.
        assert_eq!(frame(5.1, 0.29), (510, 290));
        // sigrok-cli's single precision 3.3, 329 in C.
        assert_eq!(frame(f64::from(3.3f32), 0.0), (330, 0));
        assert_eq!(frame(-0.0, -0.0), (0, 0));
    }

    #[test]
    fn an_answer_fills_what_is_reported() {
        let mut f = [0; FRAME];
        f[2..14].copy_from_slice(&[
            0x04, 0xd2, 0x01, 0xf4, 0x01, 0xf4, 0x04, 0xe2, 0x01, 0x4a, 0x00, 0x19,
        ]);
        f[15] = 0b110;
        f[18] = 7;
        f[19] = 1;
        let mut r = Reported::default();
        r.update(&f);
        assert_eq!(r.voltage, [12.34, 5.0, 3.3]);
        assert_eq!(r.current, [0.5, 1.25, 0.025]);
        assert_eq!(r.enabled, [false, true, true]);
        assert!(r.ocp, "any byte but 0");
        assert_eq!(r.mode, 1);
    }

    #[test]
    fn a_mode_that_is_none_keeps_the_last() {
        let mut f = [0; FRAME];
        f[19] = 2;
        let mut r = Reported::default();
        r.update(&f);
        f[19] = 3;
        r.update(&f);
        assert_eq!(r.mode, 2);
    }

    #[test]
    fn a_scan_wants_a_whole_answer_to_its_asking() {
        let mut f = [0; FRAME];
        f[0] = 0xaa;
        f[1] = 0xaa;
        assert!(is_scan_answer(&f));
        assert!(!is_scan_answer(&f[..23]));
        f[1] = 0x20;
        assert!(!is_scan_answer(&f), "a set's answer");
        assert!(has_header(&f));
        f[0] = 0x55;
        assert!(!has_header(&f));
    }
}
