// Counterpart in sigrok:
//   libsigrok/src/hardware/mastech-ms6514/protocol.c,
//   libsigrok/src/hardware/mastech-ms6514/protocol.h,
//   Copyright (C) 2019 Dave Buechi <db@pflutsch.ch>,
//   with commits by Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The frames of the MASTECH MS6514.
//!
//! 9600/8n1 through a CP2102, and the unit sends an 18 byte frame about twice a
//! second once PC-Link is on. The one command is `a1`, "start sending stored
//! data". All of this comes from the C driver and sigrok's wiki; no firmware
//! source was found.
//!
//! A frame starts `65 14` and ends `0d 0a`. Byte 2's bit 0 says a stored value.
//! Bytes 5-6 and 7-8 are the main and aux displays, big endian with no sign, and
//! bytes 11 and 12 their flags: bit 7 negative, bit 6 overload, bit 3 tenths. Their
//! bits 1-0 say what the displays show: 11 what the main has and 12 whether the
//! aux has a MAX, MIN or AVG, which then overrides 11's reading of the aux. Byte
//! 10 has hold in bit 6 and the unit in bits 1-0.

use crate::core::{MqFlags, Unit};

/// Bytes in a frame.
pub const FRAME_SIZE: usize = 18;
/// The capture's receive buffer: what it holds decides how many frames one read
/// takes, and so how far a sample limit overshoots.
pub const BUF_SIZE: usize = 3 * FRAME_SIZE;
/// Start sending stored data.
pub const CMD_GET_STORED: u8 = 0xa1;

/// The channels each pair of functions shows on main and aux, by
/// `aux function << 2 | main function`.
const ASSIGNMENT: [[usize; 2]; 16] = [
    [0, 1],
    [1, 0],
    [2, 0],
    [2, 1],
    [0, 0],
    [1, 1],
    [2, 2],
    [2, 2],
    [0, 0],
    [1, 1],
    [2, 2],
    [2, 2],
    [0, 0],
    [1, 1],
    [2, 2],
    [2, 2],
];

/// Whether `frame` has a frame's head and end.
#[must_use]
pub fn packet_valid(frame: &[u8]) -> bool {
    frame.len() >= FRAME_SIZE && has_head(frame) && has_end(frame)
}

fn has_head(frame: &[u8]) -> bool {
    frame[0] == 0x65 && frame[1] == 0x14
}

fn has_end(frame: &[u8]) -> bool {
    frame[16] == 0x0d && frame[17] == 0x0a
}

/// What the head of a buffer of a frame's size or more means.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// No head: try again one byte on.
    NoHead,
    /// A head and no end: all 18 bytes go, even if a real frame starts in them.
    NoEnd,
    /// A frame.
    Frame,
}

/// See [`Step`]. `buf` has at least [`FRAME_SIZE`] bytes.
#[must_use]
pub fn step(buf: &[u8]) -> Step {
    if !has_head(buf) {
        Step::NoHead
    } else if !has_end(buf) {
        Step::NoEnd
    } else {
        Step::Frame
    }
}

/// Whether a frame holds a stored value.
#[must_use]
pub fn is_stored(frame: &[u8]) -> bool {
    frame[2] & 0x01 == 0x01
}

/// Display `slot`'s reading (0 main, 1 aux) and its decimal digits. An overload is
/// `inf`, at the digits the flags say.
#[must_use]
pub fn temperature(frame: &[u8], slot: usize) -> (f32, i32) {
    let raw = u16::from_be_bytes([frame[5 + slot * 2], frame[6 + slot * 2]]);
    let flags = frame[11 + slot];
    let mut value = f32::from(raw);
    let mut digits = 0;
    if flags & 0x80 != 0 {
        value = -value;
    }
    if flags & 0x08 != 0 {
        // A float by a double in C, narrowed. Both forms round alike, but keep it.
        value = (f64::from(value) / 10.0) as f32;
        digits = 1;
    }
    if flags & 0x40 != 0 {
        value = f32::INFINITY;
    }
    (value, digits)
}

/// Display `slot`'s flags.
#[must_use]
pub fn flags(frame: &[u8], slot: usize) -> MqFlags {
    let mut f = MqFlags::empty();
    if frame[10] & 0x40 != 0 {
        f |= MqFlags::HOLD;
    }
    if slot == 0 && frame[11] & 0x03 > 0x01 {
        f |= MqFlags::RELATIVE;
    }
    if slot == 1 {
        match frame[12] & 0x03 {
            1 => f |= MqFlags::MAX,
            2 => f |= MqFlags::MIN,
            3 => f |= MqFlags::AVG,
            _ => {}
        }
    }
    f
}

/// The frame's unit.
#[must_use]
pub fn unit(frame: &[u8]) -> Unit {
    match frame[10] & 0x03 {
        1 => Unit::Celsius,
        2 => Unit::Fahrenheit,
        3 => Unit::Kelvin,
        _ => Unit::Unitless,
    }
}

/// The channel display `slot` shows.
#[must_use]
pub fn channel(frame: &[u8], slot: usize) -> usize {
    ASSIGNMENT[usize::from(frame[12] & 0x03) << 2 | usize::from(frame[11] & 0x03)][slot]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame with these displays and flag bytes.
    fn frame(main: u16, aux: u16, b10: u8, b11: u8, b12: u8) -> [u8; 18] {
        let [mh, ml] = main.to_be_bytes();
        let [ah, al] = aux.to_be_bytes();
        [0x65, 0x14, 0, 0, 0, mh, ml, ah, al, 1, b10, b11, b12, 1, 2, 3, 0x0d, 0x0a]
    }

    #[test]
    fn the_sizes() {
        assert_eq!(FRAME_SIZE, 18);
        assert_eq!(BUF_SIZE, 54);
        assert_eq!(CMD_GET_STORED, 0xa1);
    }

    #[test]
    fn a_frame_needs_its_head_and_end() {
        let good = frame(1, 2, 1, 8, 8);
        assert!(packet_valid(&good));
        assert_eq!(step(&good), Step::Frame);
        for (at, want) in
            [(0, Step::NoHead), (1, Step::NoHead), (16, Step::NoEnd), (17, Step::NoEnd)]
        {
            let mut bad = good;
            bad[at] ^= 0xff;
            assert!(!packet_valid(&bad), "{at}");
            assert_eq!(step(&bad), want, "{at}");
        }
        assert!(!packet_valid(&good[..17]), "too short");
    }

    #[test]
    fn a_stored_value_is_bit_0_of_byte_2() {
        let mut f = frame(1, 2, 1, 8, 8);
        assert!(!is_stored(&f));
        f[2] = 0xfe;
        assert!(!is_stored(&f));
        f[2] = 0x01;
        assert!(is_stored(&f));
    }

    #[test]
    fn the_readings() {
        let f = frame(0x04d2, 0x0929, 1, 0, 0);
        assert_eq!(temperature(&f, 0), (1234.0, 0));
        assert_eq!(temperature(&f, 1), (2345.0, 0));
        let f = frame(235, 241, 1, 0x08, 0x88);
        assert_eq!(temperature(&f, 0), (23.5, 1));
        assert_eq!(temperature(&f, 1), (-24.1, 1));
        let (v, d) = temperature(&frame(0, 0, 1, 0x80, 0), 0);
        assert_eq!((v.to_bits(), d), ((-0.0f32).to_bits(), 0), "a negative zero stays one");
        let (v, d) = temperature(&frame(65535, 0, 1, 0x48, 0), 0);
        assert_eq!((v, d), (f32::INFINITY, 1), "an overload keeps the digits");
        assert_eq!(
            temperature(&frame(0, 65535, 1, 0, 0xc8), 1),
            (f32::INFINITY, 1),
            "without a sign"
        );
        assert_eq!(temperature(&frame(65535, 0, 1, 0x08, 0), 0), (6553.5, 1));
        // Each display reads its own flags.
        assert_eq!(temperature(&frame(5, 5, 1, 0x40, 0), 1), (5.0, 0));
    }

    #[test]
    fn the_units() {
        assert_eq!(unit(&frame(0, 0, 0x00, 0, 0)), Unit::Unitless);
        assert_eq!(unit(&frame(0, 0, 0x01, 0, 0)), Unit::Celsius);
        assert_eq!(unit(&frame(0, 0, 0x02, 0, 0)), Unit::Fahrenheit);
        assert_eq!(unit(&frame(0, 0, 0x03, 0, 0)), Unit::Kelvin);
        assert_eq!(unit(&frame(0, 0, 0xfd, 0, 0)), Unit::Celsius, "bits 1-0 alone");
    }

    #[test]
    fn the_flags() {
        let e = MqFlags::empty();
        assert_eq!(flags(&frame(0, 0, 1, 0, 0), 0), e);
        assert_eq!(flags(&frame(0, 0, 0x41, 0, 0), 0), MqFlags::HOLD);
        assert_eq!(flags(&frame(0, 0, 0x41, 0, 0), 1), MqFlags::HOLD);
        assert_eq!(flags(&frame(0, 0, 1, 1, 0), 0), e, "T2 on main is no difference");
        assert_eq!(flags(&frame(0, 0, 1, 2, 0), 0), MqFlags::RELATIVE);
        assert_eq!(flags(&frame(0, 0, 1, 3, 0), 0), MqFlags::RELATIVE);
        assert_eq!(flags(&frame(0, 0, 1, 3, 0), 1), e, "main's function, on main");
        assert_eq!(flags(&frame(0, 0, 1, 0, 1), 1), MqFlags::MAX);
        assert_eq!(flags(&frame(0, 0, 1, 0, 2), 1), MqFlags::MIN);
        assert_eq!(flags(&frame(0, 0, 1, 0, 3), 1), MqFlags::AVG);
        assert_eq!(flags(&frame(0, 0, 1, 0, 3), 0), e, "aux's function, on aux");
    }

    #[test]
    fn the_channels_of_each_function() {
        // The wiki's table of byte 11, and each of MAX, MIN and AVG on byte 12.
        let want: [(u8, u8, usize, usize); 16] = [
            (0, 0, 0, 1),
            (0, 1, 1, 0),
            (0, 2, 2, 0),
            (0, 3, 2, 1),
            (1, 0, 0, 0),
            (1, 1, 1, 1),
            (1, 2, 2, 2),
            (1, 3, 2, 2),
            (2, 0, 0, 0),
            (2, 1, 1, 1),
            (2, 2, 2, 2),
            (2, 3, 2, 2),
            (3, 0, 0, 0),
            (3, 1, 1, 1),
            (3, 2, 2, 2),
            (3, 3, 2, 2),
        ];
        for (aux, main, a, b) in want {
            // The flag bits around them count for nothing.
            let f = frame(0, 0, 1, main | 0xc8, aux | 0xc8);
            assert_eq!(channel(&f, 0), a, "{aux} {main}");
            assert_eq!(channel(&f, 1), b, "{aux} {main}");
        }
    }
}
