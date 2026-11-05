// Counterpart in sigrok:
//   libsigrok/src/hardware/uni-t-ut32x/api.c,
//   libsigrok/src/hardware/uni-t-ut32x/protocol.c,
//   libsigrok/src/hardware/uni-t-ut32x/protocol.h,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Gerhard Sittig,
//   Martin Ling, Soeren Apel, Daniel Elstner, Wolfram Sang, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The UT32x's byte stream.
//!
//! The host sends one byte: `01` for live records, `07` for the stored ones, `02` to
//! stop. A record is 19 bytes, ending in CR LF: four characters of tenths of a degree
//! from byte 1, with `:` for a blank and `;` for a minus, the unit at byte 5, the
//! probe at byte 13, and `0` and `1` at bytes 8 and 16 as all that tells it from
//! other bytes that end in CR LF.
//!
//! [`Receiver`] is C's 19 byte buffer: nothing here depends on how reads cut the
//! stream. What ends in CR LF is a packet to try and is dropped if it is short, and a
//! full buffer with no marker is dropped whole. That finds the next record after any
//! noise but one: C loses the marker split over a buffer's end, its LF then starts
//! the next buffer, and every record after it is cut the same way, for good. Here
//! that LF goes too.

#![allow(clippy::doc_markdown)]

use crate::core::{MqFlags, Unit};

/// A record's length.
pub const PACKET_SIZE: usize = 19;
/// Start live records.
pub const CMD_GET_LIVE: u8 = 1;
/// Stop sending.
pub const CMD_STOP: u8 = 2;
/// Start the stored records.
pub const CMD_GET_STORED: u8 = 7;

const CR: u8 = b'\r';
const LF: u8 = b'\n';
const BLANK: u8 = b':';
const NEG: u8 = b';';

/// What a packet turned out to be.
#[derive(Clone, Debug, PartialEq)]
pub enum Parsed {
    /// Not a record: not counted.
    Invalid,
    /// A record with no measurement: a missing probe, a free slot, or something it
    /// cannot read. Counted, so a limit passes over the free slots of the memory.
    Empty,
    /// A reading.
    Reading {
        /// The channel's index.
        channel: usize,
        /// The temperature.
        value: f32,
        /// Its unit, which may be none: stored records have `0` there.
        unit: Unit,
        /// Whether it is a difference.
        flags: MqFlags,
    },
}

/// The temperature in `chars`, or `None` when it is no number.
fn parse_temperature(chars: &[u8]) -> Option<f32> {
    let mut negative = false;
    let mut temp = 0.0f32;
    for &c in chars {
        match c {
            BLANK => {}
            NEG if negative => {
                log::debug!("Double negative sign!");
                return None;
            }
            NEG => negative = true,
            b'0'..=b'9' => temp = temp * 10.0 + f32::from(c - b'0'),
            other => {
                log::debug!("Invalid digit '{other:02x}'!");
                return None;
            }
        }
    }
    temp /= 10.0;
    Some(if negative { -temp } else { temp })
}

/// Reads one packet, of any length.
pub fn parse_packet(pkt: &[u8]) -> Parsed {
    if pkt.len() != PACKET_SIZE
        || pkt[PACKET_SIZE - 2] != CR
        || pkt[PACKET_SIZE - 1] != LF
        || pkt[8] != b'0'
        || pkt[16] != b'1'
    {
        return Parsed::Invalid;
    }
    log::debug!("Processing 19-byte packet.");
    if pkt[1..5] == [NEG; 4] {
        return Parsed::Empty;
    }
    let Some(value) = parse_temperature(&pkt[1..5]) else {
        return Parsed::Empty;
    };
    let unit = match pkt[5] {
        b'1' => Unit::Celsius,
        b'2' => Unit::Fahrenheit,
        b'3' => Unit::Kelvin,
        other => {
            log::debug!("Unknown unit 0x{other:02x}.");
            Unit::Unitless
        }
    };
    let (channel, flags) = match pkt[13] {
        b'0' => (0, MqFlags::empty()),
        b'1' => (1, MqFlags::empty()),
        b'2' | b'3' => (2, MqFlags::RELATIVE),
        other => {
            log::error!("Unknown channel 0x{other:02x}.");
            return Parsed::Empty;
        }
    };
    Parsed::Reading { channel, value, unit, flags }
}

/// C's receive buffer, which finds the packets in the stream.
#[derive(Clone, Debug)]
pub struct Receiver {
    buf: [u8; PACKET_SIZE],
    len: usize,
    /// The buffer dropped last ended in CR.
    split_marker: bool,
}

impl Default for Receiver {
    fn default() -> Receiver {
        Receiver { buf: [0; PACKET_SIZE], len: 0, split_marker: false }
    }
}

impl Receiver {
    /// Where the next read goes, at most what the buffer holds. A buffer that is full
    /// holds no marker, and goes.
    pub fn room(&mut self) -> &mut [u8] {
        if self.len == PACKET_SIZE {
            self.split_marker = self.buf[PACKET_SIZE - 1] == CR;
            self.len = 0;
        }
        &mut self.buf[self.len..]
    }

    /// Takes `n` bytes that have come into [`Receiver::room`], and gives each packet
    /// they end to `packet`.
    pub fn received(&mut self, n: usize, mut packet: impl FnMut(&[u8])) {
        if n == 0 {
            return;
        }
        let mut from = self.len;
        self.len += n;
        if std::mem::take(&mut self.split_marker) && self.buf[from] == LF {
            self.buf.copy_within(from + 1..self.len, from);
            self.len -= 1;
        }
        // A marker may end in the first new byte, so look from the byte before it.
        from = from.saturating_sub(1);
        while let Some(at) = self.buf[from..self.len].windows(2).position(|w| w == [CR, LF]) {
            let end = from + at + 2;
            packet(&self.buf[..end]);
            self.buf.copy_within(end..self.len, 0);
            self.len -= end;
            from = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const T1: &[u8; 19] = b"2:235100012300001\r\n";
    const T2: &[u8; 19] = b"2:241100012301001\r\n";

    fn record(temp: &[u8], unit: u8, probe: u8) -> Vec<u8> {
        let mut r = T1.to_vec();
        r[1..5].copy_from_slice(temp);
        r[5] = unit;
        r[13] = probe;
        r
    }

    fn reading(p: &Parsed) -> (usize, f32, Unit, MqFlags) {
        let Parsed::Reading { channel, value, unit, flags } = p else {
            panic!("{p:?}");
        };
        (*channel, *value, *unit, *flags)
    }

    #[test]
    fn the_values() {
        let v = |t: &[u8; 4]| reading(&parse_packet(&record(t, b'1', b'0'))).1;
        assert_eq!(v(b":235"), 23.5);
        assert_eq!(v(b"0235"), 23.5);
        assert_eq!(v(b"::::"), 0.0);
        assert_eq!(v(b"9999"), 999.9);
        assert!((v(b";:12") + 1.2).abs() < 1e-5);
        assert_eq!(v(b"12;3"), -12.3, "a sign anywhere");
        assert_eq!(v(b"123;"), -12.3, "and last");
        let zero = v(b";:::");
        assert!(zero == 0.0 && zero.is_sign_negative(), "negative zero");
        // The division is C's: one float division of a float that was built up.
        assert_eq!(v(b"0001").to_bits(), (1.0f32 / 10.0).to_bits());
        assert_eq!(v(b"8765").to_bits(), (8765.0f32 / 10.0).to_bits());
    }

    #[test]
    fn the_units_and_the_probes() {
        let f = |u, p| reading(&parse_packet(&record(b":235", u, p)));
        assert_eq!(f(b'1', b'0').2, Unit::Celsius);
        assert_eq!(f(b'2', b'0').2, Unit::Fahrenheit);
        assert_eq!(f(b'3', b'0').2, Unit::Kelvin);
        assert_eq!(f(b'0', b'0').2, Unit::Unitless, "a stored record's");
        assert_eq!(f(b'9', b'0').2, Unit::Unitless);
        assert_eq!(f(b'1', b'0').0, 0);
        assert_eq!(f(b'1', b'0').3, MqFlags::empty());
        assert_eq!(f(b'1', b'1').0, 1);
        assert_eq!(f(b'1', b'1').3, MqFlags::empty());
        for p in *b"23" {
            assert_eq!(f(b'1', p).0, 2);
            assert_eq!(f(b'1', p).3, MqFlags::RELATIVE);
        }
    }

    #[test]
    fn what_counts_and_what_does_not() {
        assert_eq!(parse_packet(&record(b";;;;", b'1', b'0')), Parsed::Empty, "no probe");
        assert_eq!(parse_packet(&record(b";;12", b'1', b'0')), Parsed::Empty, "two signs");
        assert_eq!(parse_packet(&record(b"1x34", b'1', b'0')), Parsed::Empty, "a digit");
        assert_eq!(parse_packet(&record(b"/:::", b'1', b'0')), Parsed::Empty, "below 0");
        assert_eq!(parse_packet(&record(b":235", b'1', b'7')), Parsed::Empty, "a probe");
        for at in [8, 16, 17, 18] {
            let mut r = T1.to_vec();
            r[at] = b'x';
            assert_eq!(parse_packet(&r), Parsed::Invalid, "byte {at}");
        }
        assert_eq!(parse_packet(&T1[..18]), Parsed::Invalid);
        let mut long = T1.to_vec();
        long.push(0);
        assert_eq!(parse_packet(&long), Parsed::Invalid);
        assert_eq!(parse_packet(b""), Parsed::Invalid);
    }

    /// The packets `stream` makes, read `cut` bytes at a time at most.
    fn packets(stream: &[u8], cut: usize) -> Vec<Vec<u8>> {
        let mut rx = Receiver::default();
        let mut out = Vec::new();
        let mut rest = stream;
        while !rest.is_empty() {
            let room = rx.room();
            let n = room.len().min(cut).min(rest.len());
            room[..n].copy_from_slice(&rest[..n]);
            rest = &rest[n..];
            rx.received(n, |p| out.push(p.to_vec()));
        }
        out
    }

    /// The records in `stream`, however it is cut.
    fn records(stream: &[u8]) -> Vec<Vec<u8>> {
        let good = |v: Vec<Vec<u8>>| -> Vec<Vec<u8>> {
            v.into_iter().filter(|p| parse_packet(p) != Parsed::Invalid).collect()
        };
        let whole = good(packets(stream, PACKET_SIZE));
        for cut in [1, 2, 3, 7, 18] {
            assert_eq!(good(packets(stream, cut)), whole, "cut at {cut}");
        }
        whole
    }

    fn joined(parts: &[&[u8]]) -> Vec<u8> {
        parts.concat()
    }

    #[test]
    fn records_come_through_however_reads_cut_them() {
        assert_eq!(records(&joined(&[T1, T2])), [T1.to_vec(), T2.to_vec()]);
        assert_eq!(records(T1), [T1.to_vec()]);
        assert_eq!(records(&T1[..18]), Vec::<Vec<u8>>::new(), "no LF yet");
    }

    #[test]
    fn noise_is_dropped_as_c_drops_it() {
        // A whole buffer of noise goes whole.
        assert_eq!(records(&joined(&[&[b'x'; 19], T1])), [T1.to_vec()]);
        // Two buffers.
        assert_eq!(records(&joined(&[&[b'x'; 38], T1])), [T1.to_vec()]);
        // Short packets go.
        assert_eq!(records(&joined(&[b"short\r\n", T1])), [T1.to_vec()]);
        assert_eq!(records(&joined(&[b"\r\n\r\n", T1])), [T1.to_vec()]);
        assert_eq!(records(&joined(&[b"xx\r\nxx\r\n", T1])), [T1.to_vec()]);
        // Noise ahead cuts the record behind it, whose tail is a short packet.
        assert_eq!(records(&joined(&[b"xxxxx", T1, T2])), [T2.to_vec()]);
        assert_eq!(records(&joined(&[b"xx", T1, T2])), [T2.to_vec()]);
        assert_eq!(records(&joined(&[&[b'x'; 18], T1])), Vec::<Vec<u8>>::new());
    }

    #[test]
    fn a_marker_split_over_the_buffers_end_loses_one_record_and_not_all() {
        // One byte ahead: the buffer ends in the record's CR. In C the LF would start
        // the next buffer, and so on: no record ever again.
        assert_eq!(records(&joined(&[b"x", T1, T2, T1])), [T2.to_vec(), T1.to_vec()]);
        // Noise ending in a marker.
        assert_eq!(records(&joined(&[&[b'x'; 18], b"\r\n", T1])), [T1.to_vec()]);
        // A stream joined at its last byte.
        assert_eq!(records(&joined(&[b"\n", T1, T2])), [T2.to_vec()]);
    }

    #[test]
    fn a_cr_that_ends_a_buffer_with_no_lf_behind_it_changes_nothing() {
        // As in C: the next byte starts the buffer, whatever came before.
        assert_eq!(records(&joined(&[&[b'x'; 18], b"\r", T1])), [T1.to_vec()]);
        let mut rx = Receiver::default();
        let room = rx.room();
        room.fill(b'x');
        room[18] = CR;
        rx.received(19, |_| panic!("no marker"));
        let room = rx.room();
        assert_eq!(room.len(), 19, "dropped");
        room[0] = b'y';
        rx.received(1, |_| panic!("no marker"));
        assert!(!rx.split_marker, "only for the byte right behind it");
        assert_eq!(rx.len, 1);
    }

    #[test]
    fn the_lf_of_a_split_marker_alone_leaves_nothing() {
        let mut rx = Receiver::default();
        rx.room().fill(b'x');
        rx.buf[18] = CR;
        rx.received(19, |_| panic!("no marker"));
        rx.room()[0] = LF;
        rx.received(1, |_| panic!("no packet"));
        assert_eq!(rx.len, 0);
        assert!(!rx.split_marker);
    }

    #[test]
    fn a_read_of_nothing_keeps_the_watch_for_the_lf() {
        let mut rx = Receiver::default();
        rx.room().fill(b'x');
        rx.buf[18] = CR;
        rx.received(19, |_| panic!("no marker"));
        rx.room();
        rx.received(0, |_| panic!("nothing came"));
        assert!(rx.split_marker);
        rx.room()[0] = LF;
        rx.received(1, |_| panic!("no packet"));
        assert_eq!(rx.len, 0);
    }

    #[test]
    fn a_cr_in_the_buffer_and_its_lf_in_the_next_read_are_a_marker() {
        let mut rx = Receiver::default();
        rx.room()[..3].copy_from_slice(b"ab\r");
        let mut got = Vec::new();
        rx.received(3, |p| got.push(p.to_vec()));
        assert_eq!(got.len(), 0);
        rx.room()[0] = LF;
        rx.received(1, |p| got.push(p.to_vec()));
        assert_eq!(got, [b"ab\r\n".to_vec()]);
    }

    #[test]
    fn a_stale_tail_at_a_new_receiver_shifts_nothing() {
        // What a capture finds of a record the last one's stop cut: a new buffer, as C
        // sets `packet_len` to 0 at each start.
        assert_eq!(records(&joined(&[&T1[5..], T2, T1])), [T2.to_vec(), T1.to_vec()]);
    }
}
