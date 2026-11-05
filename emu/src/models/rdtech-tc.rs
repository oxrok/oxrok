// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An `RDTech` TC66C USB power meter on the CDC-ACM shell: the host writes
//! `getva`, and the meter answers 192 bytes, encrypted.
//!
//! The device, and how sure this is:
//!
//! - The key is a protocol fact, a constant of the meter's firmware that any
//!   program reading the meter has to hold. Three sources give the same 32
//!   bytes: the sigrok wiki's `RDTech_TC66C` page, which credits Ben V.
//!   Brown with it; `TheHWcave/TC66`'s `TC66C.py`, `STATIC_KEY`; and
//!   libsigrok's `protocol.c:68-73`. None says how it was found. It keeps no
//!   secret, and nothing here treats it as one.
//! - The cipher: AES in ECB mode with that key, each 16 bytes on their own
//!   (the wiki; `TC66C.py`, `AES.MODE_ECB`). 32 bytes make it AES-256.
//! - The reply: three blocks of 64 bytes starting `pac1`, `pac2` and `pac3`,
//!   integers little endian (the wiki). `pac1`: 4 the name, 8 the version,
//!   12 the serial number, 44 the runs, 48 volts in 100 uV, 52 amperes in
//!   10 uA, 56 watts in 100 uW. `pac2`: 4 ohms, 8 and 16 the mAh of group 0
//!   and 1, 12 and 20 their mWh, 24 and 28 the temperature's sign and
//!   value, 32 and 36 D+ and D- in 10 mV. `pac3` holds nothing known. The
//!   last 4 bytes of each are CRC-16/MODBUS over the 60 before it, zero
//!   extended. Bytes the wiki calls unknown or believed zero are zero.
//! - The name `TC66` and the version `1.14` are the wiki's examples.
//! - The sources disagree on the unit of `pac2`'s ohms, 10 mOhm (the wiki)
//!   against 100 mOhm (`TC66C.py`). The driver does not read it, so it does
//!   not matter here; the value below is in the wiki's.
//! - It is a CDC serial port (the wiki). No line setting is documented and
//!   `TC66C.py` opens it at 115200/8n2 where `protocol.c` uses 8n1, so it is
//!   taken for native USB, where the settings do nothing.
//!
//! Emulation artifacts:
//!
//! - The USB identity. No `lsusb` dump or descriptor listing was found for
//!   this meter (the wiki, `TC66C.py` and a search of the web have none, and
//!   that is not to say none exists), and the C driver never asks for one, so
//!   the ids are pid.codes' test pair and the strings are made up.
//! - It answers at once. The meter's own speed is unknown, and the wiki says
//!   its display refreshes twice a second, so nothing here checks a pace.
//! - The readings are a stand-in: they move by a little per reply, from a
//!   fixed start, and power on starts them over. `v=` and the like play
//!   other values.
//! - The energies count replies, not time.
//!
//! Not modelled: `query`, `gtrec`, `lastp`, `nextp`, `rotat` and boot mode's
//! `update` (the wiki), which the meter has and no path of the C driver
//! sends, so they are flagged, as is Bluetooth's `bgetva\r\n`, which C sends
//! only on a `bt/` port.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `name=HEX`, `fw=HEX`: the 4 bytes of the name and of the version;
//! - `sn=N`, the serial number;
//! - `v=N[,N...]`, `i=`, `dp=`, `dm=`, `e0=`, `e1=`: raw values played in turn
//!   instead of the stand-in's, for volts, amperes, D+, D- and the two
//!   groups' mWh;
//! - `flip=REPLY:BYTE`, repeatable, a bit gone wrong on the way in byte BYTE
//!   of the REPLY-th reply since power on, counted from 1. Under ECB that
//!   ruins one block of 16: bytes 0, 64 and 128 hold a magic, the others a
//!   checksum's data;
//! - `cut=REPLY:LEN`, only the first LEN bytes of that reply go out;
//! - `extra=REPLY:N`, N more bytes after it, 0xa5;
//! - `again=REPLY`, that reply goes out twice;
//! - `late=REPLY:OFFERS`, that reply waits for this many of the host's offers
//!   of a read with nothing queued, which [`Serial::tick`] counts: about 20 ms
//!   each, so how long a wait lasts depends on the host, as nothing here has a
//!   clock. A stand-in for a meter that is slow once;
//! - `answers=N`, replies per power on, then silence and the event point.
//!
//! The pedantic checks: only `getva`, nothing else and nothing left of it when
//! a tool opens, and no `getva` while a late reply is owed, which no C path
//! sends.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, LineEvent, Port, Serial};
use crate::{Error, Result};
use aes::Aes256Enc;
use aes::cipher::{Array, BlockCipherEncrypt, KeyInit};

/// The model name.
pub const NAME: &str = "rdtech-tc";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "An RDTech TC66C USB power meter on CDC-ACM; name=HEX, fw=HEX, sn=, v=, \
                         i=, dp=, dm=, e0=, e1=, flip=REPLY:BYTE, cut=REPLY:LEN, extra=REPLY:N, \
                         again=REPLY, late=REPLY:OFFERS, answers=, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

/// See the module's documentation for where this comes from.
const KEY: [u8; 32] = [
    0x58, 0x21, 0xfa, 0x56, 0x01, 0xb2, 0xf0, 0x26, 0x87, 0xff, 0x12, 0x04, 0x62, 0x2a, 0x4f, 0xb0,
    0x86, 0xf4, 0x02, 0x60, 0x81, 0x6f, 0x9a, 0x0b, 0xa7, 0xf1, 0x06, 0x61, 0x9a, 0xb8, 0x72, 0x88,
];
const POLL: &[u8] = b"getva";
const REPLY_LEN: usize = 192;
const BLOCK_LEN: usize = 64;

/// A meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn four_bytes(key: &str, value: &str) -> Result<[u8; 4]> {
    let bytes = parse_hex_list(key, value, 4, 4)?.remove(0);
    Ok(bytes.try_into().expect("four bytes"))
}

fn raw_list(key: &str, value: &str) -> Result<Vec<u32>> {
    value.split(',').map(|n| parse_number(key, n, 0..=u32::MAX)).collect()
}

/// `REPLY:N`, with REPLY from 1.
fn at_reply(key: &str, value: &str) -> Result<(u32, u32)> {
    let invalid = || Error(format!("{key}={value}: as in 2:100, the reply counted from 1"));
    let (reply, n) = value.split_once(':').ok_or_else(invalid)?;
    let reply = parse_number(key, reply, 1..=u32::MAX)?;
    Ok((reply, parse_number(key, n, 0..=4096)?))
}

/// `REPLY:OFFERS`, with REPLY from 1.
fn at_reply_offers(key: &str, value: &str) -> Result<(u32, u32)> {
    let invalid = || Error(format!("{key}={value}: as in 2:300, the reply counted from 1"));
    let (reply, offers) = value.split_once(':').ok_or_else(invalid)?;
    Ok((parse_number(key, reply, 1..=u32::MAX)?, parse_number(key, offers, 1..=1_000_000)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "name" => config.name = four_bytes(k, v)?,
            "fw" => config.fw = four_bytes(k, v)?,
            "sn" => config.serial = parse_number(k, v, 0..=u32::MAX)?,
            "v" => config.volts = raw_list(k, v)?,
            "i" => config.amps = raw_list(k, v)?,
            "dp" => config.d_plus = raw_list(k, v)?,
            "dm" => config.d_minus = raw_list(k, v)?,
            "e0" => config.energy0 = raw_list(k, v)?,
            "e1" => config.energy1 = raw_list(k, v)?,
            "flip" => {
                let (reply, byte) = at_reply(k, v)?;
                if byte as usize >= REPLY_LEN {
                    return Err(Error(format!("flip={v}: a byte below {REPLY_LEN}")));
                }
                config.flips.push((reply, byte as usize));
            }
            "cut" => {
                let (reply, len) = at_reply(k, v)?;
                config.cuts.push((reply, len as usize));
            }
            "extra" => {
                let (reply, n) = at_reply(k, v)?;
                config.extras.push((reply, n as usize));
            }
            "again" => config.again.push(parse_number(k, v, 1..=u32::MAX)?),
            "late" => config.late.push(at_reply_offers(k, v)?),
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    // Not the meter's: see the module's documentation.
    let id = Identity::new(0x1209, 0x0001, "RDTech", "TC66C");
    Ok(Cdc::new(NAME, &id, faults, Meter::new(config)))
}

#[derive(Clone, Debug)]
struct Config {
    name: [u8; 4],
    fw: [u8; 4],
    serial: u32,
    volts: Vec<u32>,
    amps: Vec<u32>,
    d_plus: Vec<u32>,
    d_minus: Vec<u32>,
    energy0: Vec<u32>,
    energy1: Vec<u32>,
    flips: Vec<(u32, usize)>,
    cuts: Vec<(u32, usize)>,
    extras: Vec<(u32, usize)>,
    again: Vec<u32>,
    late: Vec<(u32, u32)>,
    answers: Option<u32>,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            name: *b"TC66",
            fw: *b"1.14",
            serial: 1234,
            volts: Vec::new(),
            amps: Vec::new(),
            d_plus: Vec::new(),
            d_minus: Vec::new(),
            energy0: Vec::new(),
            energy1: Vec::new(),
            flips: Vec::new(),
            cuts: Vec::new(),
            extras: Vec::new(),
            again: Vec::new(),
            late: Vec::new(),
            answers: None,
        }
    }
}

/// The `n`-th of the values played in turn, or the stand-in's.
fn played(list: &[u32], n: u32, stand_in: u32) -> u32 {
    if list.is_empty() { stand_in } else { list[n as usize % list.len()] }
}

/// CRC-16/MODBUS.
fn modbus_crc(data: &[u8]) -> u16 {
    data.iter().fold(0xffff, |crc, byte| {
        (0..8).fold(crc ^ u16::from(*byte), |c, _| {
            if c & 1 == 1 { (c >> 1) ^ 0xa001 } else { c >> 1 }
        })
    })
}

fn put(block: &mut [u8], at: usize, value: u32) {
    block[at..at + 4].copy_from_slice(&value.to_le_bytes());
}

struct Meter {
    config: Config,
    /// Replies given since power on.
    replies: u32,
    /// What has come of a command so far.
    pending: Vec<u8>,
    /// A late reply and the offers it still waits for.
    held: Option<(u32, Vec<u8>)>,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter { config, replies: 0, pending: Vec::new(), held: None }
    }

    /// The reply in the clear, for the `n`-th reply from 0.
    fn plain(&self, n: u32) -> [u8; REPLY_LEN] {
        let c = &self.config;
        let volts = played(&c.volts, n, 50_123 + 13 * n);
        let amps = played(&c.amps, n, 12_345 + 101 * n);
        let energy0 = played(&c.energy0, n, 1_000 + n);
        let energy1 = played(&c.energy1, n, 2_000 + 2 * n);
        let mut reply = [0; REPLY_LEN];
        let (pac1, rest) = reply.split_at_mut(BLOCK_LEN);
        let (pac2, pac3) = rest.split_at_mut(BLOCK_LEN);
        pac1[..4].copy_from_slice(b"pac1");
        pac1[4..8].copy_from_slice(&c.name);
        pac1[8..12].copy_from_slice(&c.fw);
        put(pac1, 12, c.serial);
        put(pac1, 44, 7);
        put(pac1, 48, volts);
        put(pac1, 52, amps);
        put(pac1, 56, u32::try_from(u64::from(volts) * u64::from(amps) / 100_000).unwrap_or(0));
        pac2[..4].copy_from_slice(b"pac2");
        put(pac2, 4, 4_060);
        put(pac2, 8, energy0 / 5);
        put(pac2, 12, energy0);
        put(pac2, 16, energy1 / 5);
        put(pac2, 20, energy1);
        put(pac2, 28, 27);
        put(pac2, 32, played(&c.d_plus, n, 62));
        put(pac2, 36, played(&c.d_minus, n, 61));
        pac3[..4].copy_from_slice(b"pac3");
        for block in reply.as_chunks_mut::<BLOCK_LEN>().0 {
            let crc = modbus_crc(&block[..BLOCK_LEN - 4]);
            put(block, BLOCK_LEN - 4, u32::from(crc));
        }
        reply
    }

    /// The bytes that go out for the reply numbered `reply`, from 1.
    fn wire(&self, reply: u32) -> Vec<u8> {
        let cipher = Aes256Enc::new(&Array::from(KEY));
        let mut plain = self.plain(reply - 1);
        for chunk in plain.as_chunks_mut::<16>().0 {
            let mut block = Array::from(*chunk);
            cipher.encrypt_block(&mut block);
            *chunk = block.into();
        }
        let mut out = plain.to_vec();
        for (_, byte) in self.config.flips.iter().filter(|(r, _)| *r == reply) {
            out[*byte] ^= 0x01;
        }
        if self.config.again.contains(&reply) {
            let once = out.clone();
            out.extend_from_slice(&once);
        }
        for (_, len) in self.config.cuts.iter().filter(|(r, _)| *r == reply) {
            out.truncate(*len);
        }
        for (_, n) in self.config.extras.iter().filter(|(r, _)| *r == reply) {
            out.resize(out.len() + *n, 0xa5);
        }
        out
    }

    fn poll(&mut self, port: &mut Port<'_>) {
        if self.held.is_some() {
            port.violation("getva while a late reply is owed");
            return;
        }
        if self.config.answers.is_some_and(|n| self.replies >= n) {
            port.mark_event_point();
            return;
        }
        self.replies += 1;
        let wire = self.wire(self.replies);
        match self.config.late.iter().find(|(r, _)| *r == self.replies) {
            Some((_, offers)) => self.held = Some((*offers, wire)),
            None => port.send(&wire),
        }
    }
}

fn show(bytes: &[u8]) -> String {
    bytes.escape_ascii().to_string()
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        // Bounded: what stays is a start of `getva`, shorter than it.
        self.pending.extend_from_slice(data);
        while !self.pending.is_empty() {
            if self.pending.starts_with(POLL) {
                self.pending.drain(..POLL.len());
                self.poll(port);
            } else if POLL.starts_with(&self.pending) {
                return;
            } else {
                port.violation(format!("'{}': only getva is sent", show(&self.pending)));
                self.pending.clear();
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("'{}': a command left unfinished", show(&self.pending)));
            self.pending.clear();
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        let Some((offers, wire)) = &mut self.held else {
            return;
        };
        *offers -= 1;
        if *offers == 0 {
            port.send(wire);
            self.held = None;
        }
    }

    fn reset(&mut self) {
        *self = Meter::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::Line;
    use crate::shells::cdc::host::Host;
    use aes::Aes256Dec;
    use aes::cipher::BlockCipherDecrypt;

    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::default(), true, true)
    }

    fn violations(h: &mut Host<Meter>) -> usize {
        h.v.drain().len()
    }

    fn decrypt(data: &[u8]) -> Vec<u8> {
        let cipher = Aes256Dec::new(&Array::from(KEY));
        let mut out = data.to_vec();
        for chunk in out.as_chunks_mut::<16>().0 {
            let mut block = Array::from(*chunk);
            cipher.decrypt_block(&mut block);
            *chunk = block.into();
        }
        out
    }

    fn poll(h: &mut Host<Meter>) -> Vec<u8> {
        h.write(POLL);
        h.read()
    }

    fn u32_at(data: &[u8], at: usize) -> u32 {
        u32::from_le_bytes(data[at..at + 4].try_into().unwrap())
    }

    fn blocks_good(plain: &[u8]) -> bool {
        plain.as_chunks::<BLOCK_LEN>().0.iter().zip([b"pac1", b"pac2", b"pac3"]).all(
            |(block, magic)| {
                block.starts_with(magic)
                    && u32_at(block, BLOCK_LEN - 4)
                        == u32::from(modbus_crc(&block[..BLOCK_LEN - 4]))
            },
        )
    }

    #[test]
    fn the_cipher_is_aes_256_as_fips_197_has_it() {
        // FIPS 197, appendix C.3.
        let key: [u8; 32] = std::array::from_fn(|i| i as u8);
        let plain: [u8; 16] = std::array::from_fn(|i| (i as u8) * 0x11);
        let mut block = Array::from(plain);
        Aes256Enc::new(&Array::from(key)).encrypt_block(&mut block);
        let want = [
            0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf, 0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49,
            0x60, 0x89,
        ];
        assert_eq!(block.as_slice(), want);
    }

    #[test]
    fn the_crc_is_modbus() {
        // The catalogue's check value of CRC-16/MODBUS.
        assert_eq!(modbus_crc(b"123456789"), 0x4b37);
    }

    #[test]
    fn a_poll_gets_three_good_blocks_in_cipher() {
        let mut h = meter(&[]);
        let reply = poll(&mut h);
        assert_eq!(reply.len(), REPLY_LEN);
        assert!(!reply.starts_with(b"pac1"), "not in the clear");
        let plain = decrypt(&reply);
        assert!(blocks_good(&plain));
        assert_eq!(&plain[4..8], b"TC66");
        assert_eq!(&plain[8..12], b"1.14");
        assert_eq!(u32_at(&plain, 12), 1234);
        assert_eq!(u32_at(&plain, 48), 50_123);
        assert_eq!(u32_at(&plain, 52), 12_345);
        assert_eq!(u32_at(&plain, 56), 6_187, "5.0123 V * 0.12345 A in 100 uW");
        assert_eq!(u32_at(&plain, 64 + 12), 1_000);
        assert_eq!(u32_at(&plain, 64 + 20), 2_000);
        assert_eq!(u32_at(&plain, 64 + 32), 62);
        assert_eq!(u32_at(&plain, 64 + 36), 61);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_readings_move_and_power_on_starts_them_over() {
        let mut h = meter(&[]);
        let first = decrypt(&poll(&mut h));
        let second = decrypt(&poll(&mut h));
        assert_eq!(u32_at(&second, 48), 50_136);
        assert_eq!(u32_at(&second, 64 + 12), 1_001);
        assert_ne!(first, second);
        h.cdc.model.reset();
        assert_eq!(decrypt(&poll(&mut h)), first);
    }

    #[test]
    fn options_play_their_values_in_turn() {
        let mut h = meter(&[
            ("name", "54433600"),
            ("fw", "322e3030"),
            ("sn", "4294967295"),
            ("v", "1,2"),
            ("i", "3"),
            ("dp", "4"),
            ("dm", "5"),
            ("e0", "6"),
            ("e1", "7"),
        ]);
        let one = decrypt(&poll(&mut h));
        assert!(blocks_good(&one));
        assert_eq!(&one[4..8], b"TC6\0");
        assert_eq!(&one[8..12], b"2.00");
        assert_eq!(u32_at(&one, 12), u32::MAX);
        let want = [(48, 1), (52, 3), (64 + 32, 4), (64 + 36, 5), (64 + 12, 6), (64 + 20, 7)];
        for (at, value) in want {
            assert_eq!(u32_at(&one, at), value, "at {at}");
        }
        assert_eq!(u32_at(&decrypt(&poll(&mut h)), 48), 2);
        assert_eq!(u32_at(&decrypt(&poll(&mut h)), 48), 1);
    }

    #[test]
    fn a_flipped_bit_ruins_that_block_of_that_reply() {
        let mut h = meter(&[("flip", "2:64")]);
        assert!(blocks_good(&decrypt(&poll(&mut h))));
        let plain = decrypt(&poll(&mut h));
        assert!(plain.starts_with(b"pac1"));
        assert!(!plain[64..].starts_with(b"pac2"));
        assert_eq!(&plain[128..132], b"pac3");
        assert!(blocks_good(&decrypt(&poll(&mut h))));
    }

    #[test]
    fn cut_extra_and_again_change_what_goes_out() {
        let mut h = meter(&[("cut", "1:100"), ("extra", "2:3"), ("again", "3")]);
        let cut = poll(&mut h);
        assert_eq!(cut.len(), 100);
        let extra = poll(&mut h);
        assert_eq!(extra.len(), REPLY_LEN + 3);
        assert_eq!(&extra[REPLY_LEN..], [0xa5; 3]);
        let twice = poll(&mut h);
        assert_eq!(twice.len(), 2 * REPLY_LEN);
        assert_eq!(twice[..REPLY_LEN], twice[REPLY_LEN..]);
        assert_eq!(poll(&mut h).len(), REPLY_LEN);
    }

    #[test]
    fn a_late_reply_waits_for_its_offers() {
        let mut h = meter(&[("late", "2:3")]);
        assert_eq!(poll(&mut h).len(), REPLY_LEN);
        h.write(POLL);
        h.offer();
        h.offer();
        assert_eq!(h.cdc.model.held.as_ref().map(|(n, _)| *n), Some(1), "two offers taken");
        assert_eq!(h.read().len(), REPLY_LEN, "the third offer brings it");
        assert_eq!(h.cdc.model.held, None);
        assert_eq!(poll(&mut h).len(), REPLY_LEN, "the next is on time");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_request_with_a_late_reply_owed_is_flagged() {
        let mut h = meter(&[("late", "1:5")]);
        h.write(POLL);
        assert_eq!(violations(&mut h), 0);
        h.write(POLL);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn power_on_forgets_a_late_reply() {
        let mut h = meter(&[("late", "1:5")]);
        h.write(POLL);
        h.cdc.model.reset();
        assert_eq!(h.cdc.model.held, None);
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = meter(&[("answers", "1")]);
        assert_eq!(poll(&mut h).len(), REPLY_LEN);
        assert!(!h.cdc.take_event_point());
        assert_eq!(poll(&mut h), b"");
        assert!(h.cdc.take_event_point());
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_command_may_come_in_pieces_or_together() {
        let mut h = meter(&[]);
        h.write(b"ge");
        assert_eq!(h.read(), b"");
        h.write(b"tva");
        assert_eq!(h.read().len(), REPLY_LEN);
        h.write(b"getvagetva");
        assert_eq!(h.read().len(), 2 * REPLY_LEN);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_no_c_path_sends_is_flagged_and_not_answered() {
        for bad in [
            &b"query"[..],
            b"gtrec",
            b"lastp",
            b"nextp",
            b"rotat",
            b"update",
            b"bgetva\r\n",
            b"GETVA",
            b"getvb",
        ] {
            let mut h = meter(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            assert_eq!(h.read(), b"", "{}", show(bad));
        }
    }

    #[test]
    fn what_is_kept_of_a_command_stays_short() {
        let mut h = meter(&[]);
        for part in [&b"getv"[..], &[b'x'; 4096]] {
            h.write(part);
            assert!(h.cdc.model.pending.len() < POLL.len());
        }
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_command_left_unfinished_shows_at_the_next_open() {
        let mut h = meter(&[]);
        h.write(b"get");
        h.set_lines(false, false);
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.cdc.model.pending, b"");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("name", "5443"),
            ("fw", "zz"),
            ("sn", "x"),
            ("v", "1,x"),
            ("flip", "1"),
            ("flip", "0:1"),
            ("flip", "1:192"),
            ("cut", "1:x"),
            ("extra", "x:1"),
            ("again", "0"),
            ("late", "0:5"),
            ("late", "1:0"),
            ("late", "1"),
            ("answers", "x"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("flip", "1:191"), ("cut", "1:0"), ("answers", "0"), ("chunk", "1")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
