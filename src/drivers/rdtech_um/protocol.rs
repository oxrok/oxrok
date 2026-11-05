// Counterpart in sigrok:
//   libsigrok/src/hardware/rdtech-um/protocol.c,
//   libsigrok/src/hardware/rdtech-um/protocol.h,
//   Copyright (C) 2018-2020 Andreas Sandberg <andreas@sandberg.pp.se>,
//   with commits by Gerhard Sittig, Mathieu Pilato
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the `RDTech` UM meters.
//!
//! A host writes `0xf0`; the meter answers with a dump of 130 bytes that has
//! no framing of its own, big endian throughout. It opens with the model's
//! id and ends on two bytes that serve as a check. The UM24C and the UM25C
//! end on `0xfff1`; the UM34C's last byte is the XOR of 32 fixed bytes
//! ([`UM34C_SUM`]). The offsets, and the scale of each reading, are in
//! [`Profile`].
//!
//! [`Assembly`] puts the dumps together from what the port gives. See the
//! driver's module for where it differs from the C.

use crate::core::{Mq, Unit};

/// `RDTECH_UM_BUFSIZE`: bytes held at most.
pub const BUFFER: usize = 256;
/// `POLL_RECV_LEN`: a dump's length.
pub const DUMP: usize = 130;
/// `UM_CMD_POLL`: asks for a dump.
pub const POLL: u8 = 0xf0;

/// The bytes the UM34C's last byte covers.
const UM34C_SUM: [usize; 32] = [
    1, 3, 7, 9, 15, 17, 19, 23, 31, 39, 41, 45, 49, 53, 55, 57, 59, 63, 67, 69, 73, 79, 83, 89, 97,
    99, 109, 111, 113, 119, 121, 127,
];

/// A reading's width on the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Width {
    /// `BVT_BE_UINT16`.
    U16,
    /// `BVT_BE_UINT32`.
    U32,
}

/// One channel of a model.
#[derive(Clone, Copy, Debug)]
pub struct ChannelDesc {
    /// The channel's name.
    pub name: &'static str,
    /// The reading's offset in a dump.
    pub offset: usize,
    /// The reading's width.
    pub width: Width,
    /// The scale, as `p / q`.
    pub scale: (u32, u32),
    /// Digits after the point.
    pub digits: i32,
    /// What it measures.
    pub mq: Mq,
    /// In what.
    pub unit: Unit,
}

impl ChannelDesc {
    /// The reading in `dump`, as the output modules get it: `bv_get_value_len`
    /// makes a float of the register, and `sr_analog_to_float` multiplies that
    /// by the scale in double precision and goes back to a float.
    pub fn value(&self, dump: &[u8; DUMP]) -> f32 {
        let at = self.offset;
        #[allow(clippy::cast_precision_loss)]
        let raw = match self.width {
            Width::U16 => f32::from(u16::from_be_bytes([dump[at], dump[at + 1]])),
            Width::U32 => {
                u32::from_be_bytes([dump[at], dump[at + 1], dump[at + 2], dump[at + 3]]) as f32
            }
        };
        let scale = f64::from(self.scale.0) / f64::from(self.scale.1);
        #[allow(clippy::cast_possible_truncation)]
        let value = (f64::from(raw) * scale) as f32;
        value
    }
}

const fn channel(
    name: &'static str,
    offset: usize,
    width: Width,
    scale: (u32, u32),
    digits: i32,
    mq: Mq,
    unit: Unit,
) -> ChannelDesc {
    ChannelDesc { name, offset, width, scale, digits, mq, unit }
}

const fn volts(name: &'static str, offset: usize) -> ChannelDesc {
    channel(name, offset, Width::U16, (10, 1000), 2, Mq::Voltage, Unit::Volt)
}

const fn temperature() -> ChannelDesc {
    channel("T", 10, Width::U16, (1, 1), 0, Mq::Temperature, Unit::Celsius)
}

/// The energy of the recording over a threshold, in mWh.
const fn energy() -> ChannelDesc {
    channel("E", 106, Width::U32, (1, 1000), 3, Mq::Energy, Unit::WattHour)
}

const DEFAULT_CHANNELS: [ChannelDesc; 6] = [
    volts("V", 2),
    channel("I", 4, Width::U16, (1, 1000), 3, Mq::Current, Unit::Ampere),
    volts("D+", 96),
    volts("D-", 98),
    temperature(),
    energy(),
];

const UM25C_CHANNELS: [ChannelDesc; 6] = [
    channel("V", 2, Width::U16, (1, 1000), 3, Mq::Voltage, Unit::Volt),
    channel("I", 4, Width::U16, (100, 1_000_000), 4, Mq::Current, Unit::Ampere),
    volts("D+", 96),
    volts("D-", 98),
    temperature(),
    energy(),
];

/// How a model checks a dump.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Check {
    EndsFff1,
    Um34cSum,
}

/// One model.
#[derive(Debug)]
pub struct Profile {
    /// The model's name.
    pub name: &'static str,
    /// The id a dump opens with.
    pub id: u16,
    /// Its channels, in order.
    pub channels: &'static [ChannelDesc],
    check: Check,
}

/// `um_profiles`.
pub static PROFILES: [Profile; 3] = [
    Profile { name: "UM24C", id: 0x0963, channels: &DEFAULT_CHANNELS, check: Check::EndsFff1 },
    Profile { name: "UM25C", id: 0x09c9, channels: &UM25C_CHANNELS, check: Check::EndsFff1 },
    Profile { name: "UM34C", id: 0x0d4c, channels: &DEFAULT_CHANNELS, check: Check::Um34cSum },
];

/// The model with this id.
pub fn find_profile(id: u16) -> Option<&'static Profile> {
    PROFILES.iter().find(|p| p.id == id)
}

impl Profile {
    /// Whether a dump passes this model's check.
    pub fn check_ok(&self, dump: &[u8; DUMP]) -> bool {
        match self.check {
            Check::EndsFff1 => dump[DUMP - 2..] == [0xff, 0xf1],
            Check::Um34cSum => {
                UM34C_SUM.iter().fold(0, |sum, &at| sum ^ dump[at]) == dump[DUMP - 1]
            }
        }
    }
}

/// What came of the bytes so far.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Synced {
    /// Bytes dropped ahead of an id.
    pub skipped: usize,
    /// This is the first drop since a dump began on an id.
    pub warn: bool,
}

/// Puts dumps together, from the id on.
///
/// C looks for the id only at the read that brings the second byte of a
/// reply (`protocol.c:254-266`), so what follows depends on how the reads cut
/// the stream, and what it fails to find leaves two bytes which hold back all
/// polls for good (`protocol.c:157-158`). Here any bytes ahead of an id go,
/// whenever they came, and bytes that cannot begin an id don't stay. Bytes
/// that begin with an id are a dump as far as the framing goes, as in C: a
/// false id fails the check at its 130th byte.
#[derive(Debug)]
pub struct Assembly {
    id: [u8; 2],
    held: Vec<u8>,
    warned: bool,
}

impl Assembly {
    /// For a model's id.
    pub fn new(id: u16) -> Assembly {
        Assembly { id: id.to_be_bytes(), held: Vec::with_capacity(BUFFER), warned: false }
    }

    /// How much a read may bring. There is always room: a whole dump is taken
    /// off before the next read.
    pub fn room(&self) -> usize {
        BUFFER - self.held.len()
    }

    /// Whether part of a reply has come and the rest has not.
    pub fn accumulating(&self) -> bool {
        !self.held.is_empty()
    }

    /// How many bytes are held.
    pub fn held(&self) -> usize {
        self.held.len()
    }

    /// Takes what a read gave, up to [`Assembly::room`], and finds the id.
    pub fn feed(&mut self, data: &[u8]) -> Synced {
        self.held.extend_from_slice(&data[..data.len().min(self.room())]);
        let start = self.start_of_id();
        let warn = start > 0 && !self.warned;
        self.warned |= start > 0;
        self.held.drain(..start);
        Synced { skipped: start, warn }
    }

    /// The first offset that may start an id: where it is, or a last byte
    /// that may begin one, or the end.
    fn start_of_id(&self) -> usize {
        let [first, second] = self.id;
        let held = &self.held;
        let whole = held.windows(2).position(|pair| pair == [first, second]);
        whole.unwrap_or_else(
            || {
                if held.last() == Some(&first) { held.len() - 1 } else { held.len() }
            },
        )
    }

    /// The next whole dump.
    pub fn next_dump(&mut self) -> Option<[u8; DUMP]> {
        let dump: [u8; DUMP] = self.held.get(..DUMP)?.try_into().expect("DUMP bytes");
        self.held.drain(..DUMP);
        self.warned = false;
        Some(dump)
    }

    /// Gives up on a reply that stays incomplete: the next starts afresh.
    pub fn drop_partial(&mut self) {
        self.held.clear();
        self.warned = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dump(id: u16) -> [u8; DUMP] {
        let mut d = [0u8; DUMP];
        d[..2].copy_from_slice(&id.to_be_bytes());
        d[DUMP - 2] = 0xff;
        d[DUMP - 1] = 0xf1;
        d
    }

    fn with(d: &mut [u8; DUMP], at: usize, bytes: &[u8]) {
        d[at..at + bytes.len()].copy_from_slice(bytes);
    }

    fn by_name(p: &Profile, name: &str) -> ChannelDesc {
        *p.channels.iter().find(|c| c.name == name).unwrap()
    }

    #[test]
    fn the_ids_find_their_models() {
        assert_eq!(find_profile(0x0963).unwrap().name, "UM24C");
        assert_eq!(find_profile(0x09c9).unwrap().name, "UM25C");
        assert_eq!(find_profile(0x0d4c).unwrap().name, "UM34C");
        assert!(find_profile(0x1234).is_none());
    }

    #[test]
    fn channels_read_their_registers_in_their_scales() {
        let p = find_profile(0x0963).unwrap();
        let mut d = dump(p.id);
        with(&mut d, 2, &[0x02, 0x00]);
        with(&mut d, 4, &[0x00, 0xfa]);
        with(&mut d, 96, &[0x00, 59]);
        with(&mut d, 98, &[0x00, 58]);
        with(&mut d, 10, &[0x00, 27]);
        with(&mut d, 106, &[0, 0, 0x04, 0xd2]);
        let names: Vec<_> = p.channels.iter().map(|c| c.name).collect();
        assert_eq!(names, ["V", "I", "D+", "D-", "T", "E"]);
        assert_eq!(by_name(p, "V").value(&d), 5.12);
        assert_eq!(by_name(p, "I").value(&d), 0.25);
        assert_eq!(by_name(p, "D+").value(&d), 0.59);
        assert_eq!(by_name(p, "D-").value(&d), 0.58);
        assert_eq!(by_name(p, "T").value(&d), 27.0);
        assert_eq!(by_name(p, "E").value(&d), 1.234);
        assert_eq!(by_name(p, "V").digits, 2);
        assert_eq!(by_name(p, "I").digits, 3);
    }

    #[test]
    fn the_um25c_has_finer_scales() {
        let p = find_profile(0x09c9).unwrap();
        let mut d = dump(p.id);
        with(&mut d, 2, &5123u16.to_be_bytes());
        with(&mut d, 4, &2501u16.to_be_bytes());
        assert_eq!(by_name(p, "V").value(&d), 5.123);
        assert_eq!(by_name(p, "I").value(&d), 0.2501);
        assert_eq!(by_name(p, "V").digits, 3);
        assert_eq!(by_name(p, "I").digits, 4);
    }

    /// The register goes through a float first: a bound of what a golden shows
    /// (`E: 16.777218 kWh` for 16777217).
    #[test]
    fn a_large_energy_loses_what_a_float_cannot_hold() {
        let p = find_profile(0x0963).unwrap();
        let mut d = dump(p.id);
        with(&mut d, 106, &16_777_217u32.to_be_bytes());
        assert_eq!(by_name(p, "E").value(&d), 16777.216);
        with(&mut d, 106, &u32::MAX.to_be_bytes());
        assert_eq!(by_name(p, "E").value(&d), 4_294_967.5);
    }

    #[test]
    fn the_checks_are_each_models_own() {
        let p = find_profile(0x0963).unwrap();
        let mut d = dump(p.id);
        assert!(p.check_ok(&d));
        d[DUMP - 1] = 0xf0;
        assert!(!p.check_ok(&d));
        d[DUMP - 1] = 0xf1;
        d[DUMP - 2] = 0xfe;
        assert!(!p.check_ok(&d));

        let p = find_profile(0x0d4c).unwrap();
        let mut d = [0u8; DUMP];
        d[1] = 0x4c;
        d[127] = 0x0f;
        // An even byte is not covered, nor is byte 128; these odd ones are.
        d[2] = 0xaa;
        d[128] = 0x77;
        d[DUMP - 1] = 0x4c ^ 0x0f;
        assert!(p.check_ok(&d));
        d[3] ^= 1;
        assert!(!p.check_ok(&d));
        d[3] ^= 1;
        d[5] ^= 1;
        assert!(p.check_ok(&d), "offset 5 is not in the list");
    }

    #[test]
    fn a_dump_comes_whole_from_pieces() {
        let mut a = Assembly::new(0x0963);
        let d = dump(0x0963);
        assert_eq!(a.feed(&d[..1]), Synced { skipped: 0, warn: false });
        assert!(a.next_dump().is_none());
        a.feed(&d[1..DUMP - 1]);
        assert!(a.next_dump().is_none());
        assert!(a.accumulating());
        a.feed(&d[DUMP - 1..]);
        assert_eq!(a.next_dump().unwrap(), d);
        assert!(!a.accumulating());
    }

    #[test]
    fn noise_ahead_goes_at_any_read() {
        let d = dump(0x0963);
        for cut in 0..4 {
            let mut stream = vec![0xab, 0xcd, 0x09];
            stream.extend_from_slice(&d);
            let mut a = Assembly::new(0x0963);
            let first = a.feed(&stream[..cut]);
            let second = a.feed(&stream[cut..]);
            assert_eq!(first.skipped + second.skipped, 3, "cut {cut}");
            assert!(first.warn ^ second.warn, "once: cut {cut}");
            assert_eq!(a.next_dump().unwrap(), d, "cut {cut}");
        }
    }

    #[test]
    fn junk_leaves_nothing_to_hold_a_poll_back() {
        let mut a = Assembly::new(0x0963);
        assert_eq!(a.feed(&[0x55; 130]), Synced { skipped: 130, warn: true });
        assert!(!a.accumulating());
        assert_eq!(a.feed(&[0x55; 3]), Synced { skipped: 3, warn: false }, "said once");
        let mut a = Assembly::new(0x0963);
        a.feed(&[0x55, 0x09]);
        assert_eq!(a.held(), 1, "it may begin an id");
        a.feed(&[0x55]);
        assert!(!a.accumulating(), "it did not");
    }

    #[test]
    fn a_new_dump_warns_again() {
        let mut a = Assembly::new(0x0963);
        assert!(a.feed(&[0x55]).warn);
        a.feed(&dump(0x0963));
        a.next_dump().unwrap();
        assert!(a.feed(&[0x55]).warn);
        a.drop_partial();
        assert!(a.feed(&[0x55]).warn);
    }

    #[test]
    fn an_id_ahead_of_the_dump_is_taken_for_it() {
        let d = dump(0x0963);
        let mut a = Assembly::new(0x0963);
        let mut stream = vec![0x09, 0x63];
        stream.extend_from_slice(&d);
        assert_eq!(a.feed(&stream).skipped, 0);
        let false_dump = a.next_dump().unwrap();
        assert!(!find_profile(0x0963).unwrap().check_ok(&false_dump));
    }

    #[test]
    fn more_than_the_room_is_cut() {
        let mut a = Assembly::new(0x0963);
        let d = dump(0x0963);
        let mut stream = d.to_vec();
        stream.extend_from_slice(&[0xee; 300]);
        a.feed(&stream);
        assert_eq!(a.held(), BUFFER);
        assert_eq!(a.room(), 0);
    }
}
