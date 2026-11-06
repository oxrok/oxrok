// Counterpart in sigrok:
//   libsigrok/src/hardware/center-3xx/protocol.c,
//   libsigrok/src/hardware/center-3xx/protocol.h,
//   Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Gerhard Sittig, Martin Ling, Aurelien Jacobs,
//   Bert Vermeulen, Daniel Elstner, Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

// This file is generated with its header by tools/attribution.py.

//! The Center 309's answer to `A`, 45 bytes, counting from 0, as libsigrok reads
//! it and as Conrad's description of the K204's interface gives it:
//!
//! - 0: `0x02`.
//! - 1: REC (bit 0), the mode (bits 1 and 2: normal, MAX, MIN, MIN/MAX), T1-T2, REL,
//!   HOLD, low battery, Celsius (bits 3 to 7).
//! - 2: memory full (bit 0), auto power off (bit 7).
//! - 7 to 14: T1 to T4, big endian and signed.
//! - 15 to 38: the same in REL, MIN and MAX, which C does not read. In those modes
//!   it still sends bytes 7 to 14, and takes MAX's and MIN's over limit bits.
//! - 39 to 42: over limit, one bit per channel, in normal mode, REL, MAX and MIN.
//! - 43: one bit per channel: clear for tenths, set for whole degrees.
//! - 44: `0x03`.
//!
//! Only bytes 0 and 44 check a packet. No checksum says anything else.

use crate::core::Unit;

/// The size of the answer.
pub const PACKET_SIZE: usize = 45;
/// C's `SERIAL_BUFSIZE`.
pub const BUFSIZE: usize = 256;
/// The command.
pub const REQUEST: &[u8] = b"A";
/// The default line.
pub const SERIALCOMM: &str = "9600/8n1";
/// The channels of a packet.
pub const CHANNELS: usize = 4;
/// Their names.
pub const CHANNEL_NAMES: [&str; CHANNELS] = ["T1", "T2", "T3", "T4"];

/// One channel of a packet.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Temperature {
    /// In the packet's unit, infinite when over limit.
    pub value: f32,
    /// Decimal digits.
    pub digits: i32,
}

/// What C takes from a packet.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// Celsius or Fahrenheit, for all four.
    pub unit: Unit,
    /// T1 to T4.
    pub temperatures: [Temperature; CHANNELS],
}

/// `center_3xx_packet_valid`.
#[must_use]
pub fn valid(packet: &[u8]) -> bool {
    packet.first() == Some(&0x02) && packet.get(PACKET_SIZE - 1) == Some(&0x03)
}

/// `packet_parse`, on a packet [`valid`] took. The MIN/MAX mode has no over limit
/// byte (C's `TODO: Minmax?`), and REL's byte is left out, as "not available on all
/// models", so neither makes a value infinite.
#[must_use]
pub fn parse(packet: &[u8; PACKET_SIZE]) -> Reading {
    let status = packet[1];
    let over_limit = match (status >> 1) & 0x3 {
        0 => packet[39],
        1 => packet[41],
        2 => packet[42],
        _ => 0,
    };
    let temperatures = std::array::from_fn(|i| {
        let raw = i16::from_be_bytes([packet[7 + 2 * i], packet[8 + 2 * i]]);
        let whole = packet[43] & (1 << i) != 0;
        let mut value = f32::from(raw);
        if !whole {
            value /= 10.0;
        }
        if over_limit & (1 << i) != 0 {
            value = f32::INFINITY;
        }
        Temperature { value, digits: i32::from(!whole) }
    });
    let unit = if status & 0x80 != 0 { Unit::Celsius } else { Unit::Fahrenheit };
    Reading { unit, temperatures }
}

/// `dev_context`'s buffer and the search of `handle_new_data`. It holds less than a
/// packet between two reads.
#[derive(Debug)]
pub struct Buffer {
    bytes: [u8; BUFSIZE],
    len: usize,
    at: usize,
}

impl Default for Buffer {
    fn default() -> Buffer {
        Buffer { bytes: [0; BUFSIZE], len: 0, at: 0 }
    }
}

impl Buffer {
    /// Where a read goes: what is left moves to the front first, so there is room
    /// even if a capture left off in the middle of a read.
    pub fn room(&mut self) -> &mut [u8] {
        self.bytes.copy_within(self.at..self.len, 0);
        self.len -= self.at;
        self.at = 0;
        &mut self.bytes[self.len..]
    }

    /// `count` bytes went into [`Buffer::room`].
    pub fn filled(&mut self, count: usize) {
        self.len = (self.len + count).min(BUFSIZE);
    }

    /// The next whole packet, passing over what cannot begin one.
    pub fn next_packet(&mut self) -> Option<[u8; PACKET_SIZE]> {
        while self.len - self.at >= PACKET_SIZE {
            let window = &self.bytes[self.at..self.at + PACKET_SIZE];
            if valid(window) {
                let packet = window.try_into().expect("the window is a packet long");
                self.at += PACKET_SIZE;
                return Some(packet);
            }
            self.at += 1;
        }
        None
    }

    /// Bytes waiting that are no packet yet.
    #[must_use]
    pub fn waiting(&self) -> usize {
        self.len - self.at
    }

    /// Forgets what is waiting.
    pub fn clear(&mut self) {
        self.len = 0;
        self.at = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The answer in Conrad's document.
    const DOCUMENT: [u8; PACKET_SIZE] = [
        0x02, 0x80, 0x80, 0x01, 0x02, 0x02, 0x02, 0x00, 0xef, 0x7f, 0xff, 0x7f, 0xff, 0x7f, 0xff,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0e, 0, 0, 0,
        0x0e, 0x03,
    ];

    fn packet(status: u8, values: [u16; 4], over: [u8; 4], whole: u8) -> [u8; PACKET_SIZE] {
        let mut p = [0; PACKET_SIZE];
        p[0] = 2;
        p[1] = status;
        for (i, v) in values.iter().enumerate() {
            p[7 + 2 * i..9 + 2 * i].copy_from_slice(&v.to_be_bytes());
        }
        p[39..43].copy_from_slice(&over);
        p[43] = whole;
        p[44] = 3;
        p
    }

    fn values(r: &Reading) -> [f32; 4] {
        r.temperatures.map(|t| t.value)
    }

    #[test]
    fn the_literals_are_c_s() {
        assert_eq!(PACKET_SIZE, 45);
        assert_eq!(BUFSIZE, 256);
        assert_eq!(REQUEST, b"A");
        assert_eq!(SERIALCOMM, "9600/8n1");
        assert_eq!(CHANNEL_NAMES, ["T1", "T2", "T3", "T4"]);
    }

    #[test]
    fn the_documents_answer_reads_as_the_document_says() {
        assert!(valid(&DOCUMENT));
        let r = parse(&DOCUMENT);
        assert_eq!(r.unit, Unit::Celsius);
        assert_eq!(values(&r), [23.9, f32::INFINITY, f32::INFINITY, f32::INFINITY]);
        assert_eq!(r.temperatures.map(|t| t.digits), [1, 0, 0, 0]);
    }

    #[test]
    fn only_the_first_and_last_bytes_check_a_packet() {
        let mut p = DOCUMENT;
        p[0] = 3;
        assert!(!valid(&p));
        let mut p = DOCUMENT;
        p[44] = 2;
        assert!(!valid(&p));
        assert!(!valid(&DOCUMENT[..44]));
        assert!(!valid(&[]));
        assert!(valid(&packet(0xff, [0xffff; 4], [0xff; 4], 0xff)));
    }

    #[test]
    fn values_are_signed_and_big_endian_with_tenths_by_bit() {
        let v = [0x7fff, 0x8000, 0xffff, 0x0102];
        let r = parse(&packet(0x80, v, [0; 4], 0));
        assert_eq!(values(&r), [3276.7, -3276.8, -0.1, 25.8]);
        let r = parse(&packet(0x80, v, [0; 4], 0x0f));
        assert_eq!(values(&r), [32767.0, -32768.0, -1.0, 258.0]);
        for bit in 0..4 {
            let r = parse(&packet(0x80, [10; 4], [0; 4], 1 << bit));
            for i in 0..4 {
                let whole = i == bit;
                assert_eq!(r.temperatures[i].value, if whole { 10.0 } else { 1.0 });
                assert_eq!(r.temperatures[i].digits, i32::from(!whole));
            }
        }
        // The high bits mean nothing.
        let r = parse(&packet(0x80, [10; 4], [0; 4], 0xf0));
        assert_eq!(values(&r), [1.0; 4]);
    }

    #[test]
    fn bit_7_says_celsius() {
        assert_eq!(parse(&packet(0x80, [0; 4], [0; 4], 0)).unit, Unit::Celsius);
        assert_eq!(parse(&packet(0x7f, [0; 4], [0; 4], 0)).unit, Unit::Fahrenheit);
    }

    #[test]
    fn over_limit_is_read_from_the_modes_byte() {
        let inf = f32::INFINITY;
        // Bytes 39, 40, 41, 42 by turn: which ones count in each mode.
        for (status, counting) in [
            (0x80, Some(0)),
            (0x82, Some(2)),
            (0x84, Some(3)),
            (0x86, None),
            // REL's bit does not change the mode's byte.
            (0x90, Some(0)),
            (0x92, Some(2)),
        ] {
            for at in 0..4 {
                let mut over = [0; 4];
                over[at] = 0b0101;
                let r = parse(&packet(status, [10; 4], over, 0));
                let want = if counting == Some(at) { [inf, 1.0, inf, 1.0] } else { [1.0; 4] };
                assert_eq!(values(&r), want, "status {status:#x}, byte {at}");
            }
        }
        // The high bits mean nothing.
        assert_eq!(values(&parse(&packet(0x80, [10; 4], [0xf0; 4], 0))), [1.0; 4]);
        // An over limit value keeps its digits.
        let r = parse(&packet(0x80, [10; 4], [1, 0, 0, 0], 0));
        assert_eq!(r.temperatures[0].digits, 1);
    }

    #[test]
    fn a_value_is_a_float_divided_by_ten_as_c_does() {
        // Not 2345 tenths computed any other way.
        let r = parse(&packet(0x80, [2345, 0, 0, 0], [0; 4], 0));
        assert_eq!(r.temperatures[0].value.to_bits(), (2345.0f32 / 10.0).to_bits());
    }

    fn fill(buffer: &mut Buffer, bytes: &[u8]) {
        buffer.room()[..bytes.len()].copy_from_slice(bytes);
        buffer.filled(bytes.len());
    }

    #[test]
    fn the_search_passes_over_what_begins_no_packet() {
        let mut b = Buffer::default();
        fill(&mut b, &[0xff, 0x02, 0xff]);
        fill(&mut b, &DOCUMENT);
        fill(&mut b, &[0xee]);
        assert_eq!(b.next_packet(), Some(DOCUMENT));
        assert_eq!(b.next_packet(), None);
        assert_eq!(b.waiting(), 1);
        fill(&mut b, &DOCUMENT);
        assert_eq!(b.next_packet(), Some(DOCUMENT));
        assert_eq!(b.next_packet(), None);
        assert_eq!(b.waiting(), 0);
    }

    #[test]
    fn a_packet_cut_in_two_comes_whole() {
        let mut b = Buffer::default();
        fill(&mut b, &DOCUMENT[..20]);
        assert_eq!(b.next_packet(), None);
        assert_eq!(b.waiting(), 20);
        fill(&mut b, &DOCUMENT[20..]);
        assert_eq!(b.next_packet(), Some(DOCUMENT));
    }

    #[test]
    fn what_is_left_is_at_most_a_packet_short_and_the_room_comes_back() {
        let mut b = Buffer::default();
        for _ in 0..1000 {
            let room = b.room().len();
            assert!(room > BUFSIZE - PACKET_SIZE, "{room}");
            let n = room.min(100);
            b.room()[..n].fill(0xaa);
            b.filled(n);
            assert_eq!(b.next_packet(), None);
            assert_eq!(b.waiting(), PACKET_SIZE - 1);
        }
        b.clear();
        assert_eq!(b.waiting(), 0);
        assert_eq!(b.room().len(), BUFSIZE);
    }

    /// A capture that stops in the middle of a read leaves packets behind.
    #[test]
    fn there_is_room_after_a_read_left_half_done() {
        let mut b = Buffer::default();
        let full: Vec<u8> = DOCUMENT.iter().copied().cycle().take(BUFSIZE).collect();
        fill(&mut b, &full);
        assert_eq!(b.room().len(), 0);
        assert_eq!(b.next_packet(), Some(DOCUMENT));
        assert_eq!(b.room().len(), PACKET_SIZE);
        let more = &DOCUMENT[(BUFSIZE % PACKET_SIZE)..];
        fill(&mut b, more);
        for _ in 0..5 {
            assert_eq!(b.next_packet(), Some(DOCUMENT));
        }
        assert_eq!(b.next_packet(), None);
        assert_eq!(b.waiting(), 0);
    }

    #[test]
    fn six_packets_in_all_the_room_there_is() {
        let mut b = Buffer::default();
        let mut sent = 0;
        let mut got = 0;
        let stream: Vec<u8> = DOCUMENT.iter().copied().cycle().take(45 * 6).collect();
        while sent < stream.len() {
            let room = b.room();
            let n = room.len().min(stream.len() - sent);
            room[..n].copy_from_slice(&stream[sent..sent + n]);
            b.filled(n);
            sent += n;
            while b.next_packet().is_some() {
                got += 1;
            }
        }
        assert_eq!(got, 6);
        assert_eq!(b.waiting(), 0);
    }
}
