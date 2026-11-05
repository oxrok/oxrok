// Counterpart in sigrok:
//   libsigrok/src/hardware/atorch/protocol.c,
//   libsigrok/src/hardware/atorch/protocol.h,
//   Copyright (C) 2023 Mathieu Pilato <pilato.mathieu@free.fr>,
//   with commits by Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The reports of an Atorch unit: a stream of messages, each `ff 55`, a type
//! byte, then a body whose length the type fixes.
//!
//! | type | what    | length, all in |
//! |------|---------|-----------------|
//! | `01` | report  | 36              |
//! | `02` | reply   | 8               |
//! | `11` | command | 10              |
//!
//! A report's byte 3 is the unit's type (`02` DC meter or load, `03` USB meter).
//! The readings are big endian, at the offsets in [`Profile::channels`]. The
//! last byte of each message is a sum, which other programs check and some units
//! fail (`esphome-atorch-dl24`'s README has the J7-C, DL24M and DT24PW). C has
//! no check, nor has this.
//!
//! Nothing else about a unit goes through this driver: it only reads.

use crate::core::{Mq, MqFlags, Unit};
use std::fmt::Write as _;

/// `ATORCH_BUFSIZE`.
const BUFSIZE: usize = 128;
/// `MSGLEN_REPORT`.
pub const REPORT_LEN: usize = 36;
/// `MSGLEN_REPLY`.
const REPLY_LEN: usize = 8;
/// `MSGLEN_COMMAND`.
const COMMAND_LEN: usize = 10;
/// `MSGLEN_MIN`: what it takes to look at a message at all.
const MIN_LEN: usize = 4;
/// `header_magic`.
const MAGIC: [u8; 2] = [0xff, 0x55];
/// `MSG_REPORT`.
const MSG_REPORT: u8 = 0x01;
/// `MSG_REPLY`.
const MSG_REPLY: u8 = 0x02;
/// `MSG_COMMAND`.
const MSG_COMMAND: u8 = 0x11;

/// A reading's width on the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Width {
    /// `BVT_BE_UINT16`.
    U16,
    /// `BVT_BE_UINT24`.
    U24,
    /// `BVT_BE_UINT32`.
    U32,
}

/// One channel of a unit: `atorch_channel_desc`.
#[derive(Clone, Copy, Debug)]
pub struct ChannelDesc {
    /// The channel's name.
    pub name: &'static str,
    /// The reading's offset in a report.
    pub offset: usize,
    /// The reading's width.
    pub width: Width,
    /// The scale, as `p / q`.
    pub scale: (u32, u32),
    /// Digits after the point; negative ones before it.
    pub digits: i32,
    /// What it measures.
    pub mq: Mq,
    /// In what.
    pub unit: Unit,
    /// How.
    pub flags: MqFlags,
}

impl ChannelDesc {
    /// The reading in `report`, as the output modules get it: `bv_get_value`
    /// makes a float of the register, and `sr_analog_to_float` multiplies that
    /// by the scale in double precision and goes back to a float.
    pub fn value(&self, report: &Report) -> f32 {
        let b = &report.0[self.offset..];
        #[allow(clippy::cast_precision_loss)]
        let raw = match self.width {
            Width::U16 => f32::from(u16::from_be_bytes([b[0], b[1]])),
            Width::U24 => u32::from_be_bytes([0, b[0], b[1], b[2]]) as f32,
            Width::U32 => u32::from_be_bytes([b[0], b[1], b[2], b[3]]) as f32,
        };
        let scale = f64::from(self.scale.0) / f64::from(self.scale.1);
        #[allow(clippy::cast_possible_truncation)]
        let value = (f64::from(raw) * scale) as f32;
        value
    }
}

const fn volts(
    name: &'static str,
    offset: usize,
    width: Width,
    scale: (u32, u32),
    digits: i32,
) -> ChannelDesc {
    ChannelDesc {
        name,
        offset,
        width,
        scale,
        digits,
        mq: Mq::Voltage,
        unit: Unit::Volt,
        flags: MqFlags::DC,
    }
}

const fn amps(offset: usize, scale: (u32, u32), digits: i32) -> ChannelDesc {
    ChannelDesc {
        name: "I",
        offset,
        width: Width::U24,
        scale,
        digits,
        mq: Mq::Current,
        unit: Unit::Ampere,
        flags: MqFlags::DC,
    }
}

const fn energy(
    name: &'static str,
    offset: usize,
    width: Width,
    scale: (u32, u32),
    digits: i32,
    unit: Unit,
) -> ChannelDesc {
    ChannelDesc {
        name,
        offset,
        width,
        scale,
        digits,
        mq: Mq::Energy,
        unit,
        flags: MqFlags::empty(),
    }
}

const fn temperature(offset: usize) -> ChannelDesc {
    ChannelDesc {
        name: "T",
        offset,
        width: Width::U16,
        scale: (1, 1),
        digits: 0,
        mq: Mq::Temperature,
        unit: Unit::Celsius,
        flags: MqFlags::empty(),
    }
}

/// `atorch_dc_power_meter_channels`.
static DC_CHANNELS: [ChannelDesc; 5] = [
    volts("V", 4, Width::U24, (100, 1000), 1),
    amps(7, (1, 1000), 3),
    energy("C", 10, Width::U24, (10, 1000), 2, Unit::AmpereHour),
    energy("E", 13, Width::U32, (10, 1), -2, Unit::WattHour),
    temperature(24),
];

/// `atorch_usb_power_meter_channels`.
static USB_CHANNELS: [ChannelDesc; 7] = [
    volts("V", 4, Width::U24, (10, 1000), 2),
    amps(7, (10, 1000), 2),
    energy("C", 10, Width::U24, (1, 1000), 3, Unit::AmpereHour),
    energy("E", 13, Width::U32, (10, 1000), 2, Unit::WattHour),
    volts("D-", 17, Width::U16, (10, 1000), 2),
    volts("D+", 19, Width::U16, (10, 1000), 2),
    temperature(21),
];

/// A kind of unit: `atorch_device_profile`.
#[derive(Debug)]
pub struct Profile {
    /// Byte 3 of its reports.
    pub device_type: u8,
    /// `sdi->model`.
    pub name: &'static str,
    /// Its channels.
    pub channels: &'static [ChannelDesc],
}

/// `atorch_profiles`.
static PROFILES: [Profile; 2] = [
    Profile { device_type: 0x02, name: "DC Meter", channels: &DC_CHANNELS },
    Profile { device_type: 0x03, name: "USB Meter", channels: &USB_CHANNELS },
];

/// `find_profile_for_device_type`.
pub fn find_profile(device_type: u8) -> Option<&'static Profile> {
    PROFILES.iter().find(|p| p.device_type == device_type)
}

/// A whole report.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Report(pub [u8; REPORT_LEN]);

impl Report {
    /// The unit's type.
    pub fn device_type(&self) -> u8 {
        self.0[3]
    }
}

/// A message out of the stream.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Message {
    /// A report.
    Report(Report),
    /// A reply or a command: nothing here uses them.
    Other,
}

/// What a read has brought and no message has taken yet: `buf`, `wr_idx` and
/// `rd_idx`, and `locate_next_valid_msg`.
#[derive(Debug)]
pub struct Framer {
    buf: [u8; BUFSIZE],
    wr: usize,
    rd: usize,
}

impl Default for Framer {
    fn default() -> Self {
        Framer { buf: [0; BUFSIZE], wr: 0, rd: 0 }
    }
}

impl Framer {
    /// Where the next read goes, with what has been taken moved out of the way.
    /// After [`Framer::next_message`] said `None` it holds less than a report,
    /// so there is room.
    pub fn room(&mut self) -> &mut [u8] {
        self.buf.copy_within(self.rd..self.wr, 0);
        self.wr -= self.rd;
        self.rd = 0;
        &mut self.buf[self.wr..]
    }

    /// `n` bytes went into [`Framer::room`].
    pub fn filled(&mut self, n: usize) {
        self.wr += n;
    }

    /// The next whole message. Bytes ahead of a magic go one at a time, a magic
    /// with a type nobody knows goes whole, and from a magic and a known type
    /// the type's length is a message, whatever it holds.
    pub fn next_message(&mut self) -> Option<Message> {
        while self.rd + MIN_LEN <= self.wr {
            let at = &self.buf[self.rd..self.wr];
            if at[..MAGIC.len()] != MAGIC {
                self.rd += 1;
                continue;
            }
            let len = match at[2] {
                MSG_REPORT => REPORT_LEN,
                MSG_REPLY => REPLY_LEN,
                MSG_COMMAND => COMMAND_LEN,
                _ => {
                    self.rd += MAGIC.len();
                    continue;
                }
            };
            if at.len() < len {
                return None;
            }
            if log::log_enabled!(log::Level::Debug) {
                log::debug!("Atorch msg: {}", hexdump(&at[..len]));
            }
            let message = if at[2] == MSG_REPORT {
                let mut report = [0; REPORT_LEN];
                report.copy_from_slice(&at[..len]);
                Message::Report(Report(report))
            } else {
                Message::Other
            };
            self.rd += len;
            return Some(message);
        }
        None
    }
}

/// `sr_hexdump_new`: a space between bytes, two after each 8th, three after each
/// 16th.
fn hexdump(data: &[u8]) -> String {
    let mut s = String::with_capacity(3 * data.len() + data.len() / 8 + data.len() / 16);
    for (i, byte) in data.iter().enumerate() {
        if i > 0 {
            s.push(' ');
        }
        if i > 0 && i % 8 == 0 {
            s.push(' ');
        }
        if i > 0 && i % 16 == 0 {
            s.push(' ');
        }
        let _ = write!(s, "{byte:02x}");
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A recording of a DL24, by way of `esphome-atorch-dl24`.
    const DC: [u8; REPORT_LEN] = [
        0xff, 0x55, 0x01, 0x02, 0x00, 0x01, 0x1a, 0x00, 0x00, 0x3c, 0x00, 0x04, 0xd4, 0x00, 0x00,
        0x00, 0x20, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x00, 0x59, 0x0d, 0x36,
        0x3c, 0x00, 0x00, 0x00, 0x00, 0xf0,
    ];
    /// A recording of a USB meter.
    const USB: [u8; REPORT_LEN] = [
        0xff, 0x55, 0x01, 0x03, 0x00, 0x01, 0xf3, 0x00, 0x00, 0x00, 0x00, 0x06, 0x38, 0x00, 0x00,
        0x03, 0x11, 0x00, 0x07, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x12, 0x2e, 0x33, 0x3c, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x4e,
    ];

    fn feed(framer: &mut Framer, bytes: &[u8]) -> Vec<Message> {
        let mut out = Vec::new();
        for chunk in bytes.chunks(64) {
            let room = framer.room();
            room[..chunk.len()].copy_from_slice(chunk);
            framer.filled(chunk.len());
            while let Some(m) = framer.next_message() {
                out.push(m);
            }
        }
        out
    }

    fn values(report: &[u8; REPORT_LEN]) -> Vec<(&'static str, f32)> {
        let r = Report(*report);
        find_profile(r.device_type())
            .unwrap()
            .channels
            .iter()
            .map(|c| (c.name, c.value(&r)))
            .collect()
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn a_dc_report_reads_as_its_table_says() {
        assert_eq!(
            values(&DC),
            [("V", 28.2), ("I", 0.06), ("C", 12.36), ("E", 320.0), ("T", 38.0)]
        );
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn a_usb_report_reads_as_its_table_says() {
        assert_eq!(
            values(&USB),
            [
                ("V", 4.99),
                ("I", 0.0),
                ("C", 1.592),
                ("E", 7.85),
                ("D-", 0.07),
                ("D+", 0.1),
                ("T", 0.0),
            ]
        );
    }

    #[test]
    fn the_tables_digits_and_meanings_are_cs() {
        let dc = find_profile(2).unwrap();
        assert_eq!(dc.name, "DC Meter");
        let digits: Vec<i32> = dc.channels.iter().map(|c| c.digits).collect();
        assert_eq!(digits, [1, 3, 2, -2, 0]);
        let usb = find_profile(3).unwrap();
        assert_eq!(usb.name, "USB Meter");
        let digits: Vec<i32> = usb.channels.iter().map(|c| c.digits).collect();
        assert_eq!(digits, [2, 2, 3, 2, 2, 2, 0]);
        for p in [dc, usb] {
            for c in p.channels {
                let want = matches!(c.mq, Mq::Voltage | Mq::Current);
                assert_eq!(c.flags == MqFlags::DC, want, "{}", c.name);
            }
        }
        assert!(find_profile(1).is_none(), "the AC meter is not in C's table");
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn a_reading_goes_through_a_float_first() {
        // 2^24 + 1 has no float: it reads as 2^24.
        let mut report = DC;
        report[13..17].copy_from_slice(&0x0100_0001u32.to_be_bytes());
        let e = find_profile(2).unwrap().channels[3];
        assert_eq!(e.value(&Report(report)), 167_772_160.0);
    }

    #[test]
    fn messages_come_out_whole_however_they_are_cut() {
        let mut bytes = DC.to_vec();
        bytes.extend_from_slice(&[0xff, 0x55, 0x02, 1, 1, 0, 0, 0x40]);
        bytes.extend_from_slice(&[0xff, 0x55, 0x11, 2, 3, 0, 0, 0, 0, 0x52]);
        bytes.extend_from_slice(&USB);
        let want = [
            Message::Report(Report(DC)),
            Message::Other,
            Message::Other,
            Message::Report(Report(USB)),
        ];
        for cut in 1..bytes.len() {
            let mut framer = Framer::default();
            let mut out = Vec::new();
            for chunk in bytes.chunks(cut) {
                out.extend(feed(&mut framer, chunk));
            }
            assert_eq!(out, want, "cut {cut}");
        }
    }

    #[test]
    fn what_is_no_message_goes() {
        for noise in [
            &[0xaa, 0xbb, 0xcc][..],
            &[0x55, 0xff][..],
            // A magic of no known type goes whole, so its next bytes get a look.
            &[0xff, 0x55, 0x77][..],
            &[0xff, 0x55, 0xff, 0x55, 0x00, 0x00][..],
            // Half a magic.
            &[0xff][..],
            &[0xff, 0xff][..],
        ] {
            let mut framer = Framer::default();
            let mut bytes = noise.to_vec();
            bytes.extend_from_slice(&DC);
            assert_eq!(feed(&mut framer, &bytes), [Message::Report(Report(DC))], "{noise:x?}");
        }
    }

    #[test]
    fn a_reply_and_a_command_take_all_their_bytes() {
        // Each ends in what, with the next two bytes, would look like a report.
        for message in [
            &[0xff, 0x55, 0x02, 1, 1, 0, 0, 0xff][..],
            &[0xff, 0x55, 0x11, 2, 3, 0, 0, 0, 0, 0xff][..],
        ] {
            let mut framer = Framer::default();
            let mut bytes = message.to_vec();
            bytes.extend_from_slice(&[0x55, 0x01]);
            bytes.extend_from_slice(&DC);
            let out = feed(&mut framer, &bytes);
            assert_eq!(out, [Message::Other, Message::Report(Report(DC))], "{message:x?}");
        }
    }

    #[test]
    fn a_false_magic_takes_a_reports_length_as_in_c() {
        let mut framer = Framer::default();
        let mut bytes = vec![0xff, 0x55, 0x01];
        bytes.extend_from_slice(&DC);
        bytes.extend_from_slice(&USB);
        let out = feed(&mut framer, &bytes);
        assert_eq!(out.len(), 2);
        let Message::Report(false_one) = &out[0] else { panic!("{out:?}") };
        assert_eq!(false_one.0[..3], [0xff, 0x55, 0x01]);
        assert_eq!(false_one.0[3..6], DC[..3]);
        // What it left of the real one goes as noise; the next is found.
        assert_eq!(out[1], Message::Report(Report(USB)));
    }

    #[test]
    fn it_waits_for_the_rest_and_never_runs_out_of_room() {
        let mut framer = Framer::default();
        assert_eq!(feed(&mut framer, &DC[..3]), []);
        assert_eq!(feed(&mut framer, &DC[3..35]), []);
        assert_eq!(feed(&mut framer, &DC[35..]), [Message::Report(Report(DC))]);
        // What fills a read to the end: messages and noise, at any cut.
        let mut stream = Vec::new();
        for i in 0..4096usize {
            stream.push(match i % 5 {
                0 => 0xff,
                1 => 0x55,
                2 => [1u8, 2, 0x11, 7][i / 5 % 4],
                _ => (i % 251) as u8,
            });
        }
        let mut framer = Framer::default();
        for chunk in stream.chunks(53) {
            let mut chunk = chunk;
            while !chunk.is_empty() {
                let room = framer.room();
                assert_ne!(room.len(), 0);
                let n = room.len().min(chunk.len());
                room[..n].copy_from_slice(&chunk[..n]);
                framer.filled(n);
                chunk = &chunk[n..];
                while framer.next_message().is_some() {}
                assert!(framer.wr - framer.rd < REPORT_LEN);
            }
        }
    }

    #[test]
    fn the_dump_is_sr_hexdumps() {
        let s = hexdump(&(0u8..18).collect::<Vec<_>>());
        assert_eq!(s, "00 01 02 03 04 05 06 07  08 09 0a 0b 0c 0d 0e 0f   10 11");
    }
}
