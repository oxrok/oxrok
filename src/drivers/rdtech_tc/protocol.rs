// Counterpart in sigrok:
//   libsigrok/src/hardware/rdtech-tc/protocol.c,
//   libsigrok/src/hardware/rdtech-tc/protocol.h,
//   Copyright (C) 2020 Andreas Sandberg <andreas@sandberg.pp.se>,
//   with commits by Gerhard Sittig, Mathieu Pilato
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The TC66C's protocol.
//!
//! The host writes `getva`, with no terminator, and the meter answers 192
//! bytes under AES-256 in ECB mode, with a key every meter shares. It keeps no
//! secret, and [`decrypt`] is the one place here that knows of a cipher. In
//! the clear there are three blocks of 64 bytes, starting `pac1`, `pac2` and
//! `pac3` and ending in a CRC-16/MODBUS over the other 60, extended to 32
//! bits. Integers are little endian. [`CHANNELS`] says which are read.

use crate::core::{Mq, Unit};
use aes::Aes256Dec;
use aes::cipher::{Array, BlockCipherDecrypt, KeyInit};
use std::fmt::Write;

/// `protocol.c`'s `aes_key`.
const KEY: [u8; 32] = [
    0x58, 0x21, 0xfa, 0x56, 0x01, 0xb2, 0xf0, 0x26, 0x87, 0xff, 0x12, 0x04, 0x62, 0x2a, 0x4f, 0xb0,
    0x86, 0xf4, 0x02, 0x60, 0x81, 0x6f, 0x9a, 0x0b, 0xa7, 0xf1, 0x06, 0x61, 0x9a, 0xb8, 0x72, 0x88,
];

/// The poll request on a CDC port. C's `bgetva\r\n` is for its Bluetooth
/// ports, which `serial::open` does not have.
pub const POLL_REQUEST: &[u8] = b"getva";
/// `TC_POLL_LEN`: one reply.
pub const POLL_LEN: usize = 3 * PAC_LEN;
/// `RDTECH_TC_RSPBUFSIZE`: how much is kept of what has come. It holds less
/// than two replies, so one pass over it finds one at most.
pub const RSP_BUF_SIZE: usize = 256;

const PAC_LEN: usize = 64;
const CRC_POS: usize = PAC_LEN - 4;
const MAGICS: [&[u8; 4]; 3] = [b"pac1", b"pac2", b"pac3"];
const OFF_MODEL: usize = 4;
const OFF_FW_VER: usize = 8;
const OFF_SERIAL: usize = 12;
const FIELD_LEN: usize = 4;

/// What a channel reads.
#[derive(Clone, Copy, Debug)]
pub struct ChannelDesc {
    /// Its name.
    pub name: &'static str,
    /// Where its little endian 32 bits are in a reply.
    offset: usize,
    /// The scale, as a fraction.
    scale: (u32, u32),
    /// Digits after the point.
    pub digits: i32,
    /// What it measures.
    pub mq: Mq,
    /// Its unit.
    pub unit: Unit,
}

/// The channels, in `sr_channel_new`'s order.
pub const CHANNELS: [ChannelDesc; 6] = [
    ChannelDesc {
        name: "V",
        offset: 48,
        scale: (100, 1_000_000),
        digits: 4,
        mq: Mq::Voltage,
        unit: Unit::Volt,
    },
    ChannelDesc {
        name: "I",
        offset: 52,
        scale: (10, 1_000_000),
        digits: 5,
        mq: Mq::Current,
        unit: Unit::Ampere,
    },
    ChannelDesc {
        name: "D+",
        offset: 64 + 32,
        scale: (10, 1_000),
        digits: 2,
        mq: Mq::Voltage,
        unit: Unit::Volt,
    },
    ChannelDesc {
        name: "D-",
        offset: 64 + 36,
        scale: (10, 1_000),
        digits: 2,
        mq: Mq::Voltage,
        unit: Unit::Volt,
    },
    ChannelDesc {
        name: "E0",
        offset: 64 + 12,
        scale: (1, 1_000),
        digits: 3,
        mq: Mq::Energy,
        unit: Unit::WattHour,
    },
    ChannelDesc {
        name: "E1",
        offset: 64 + 20,
        scale: (1, 1_000),
        digits: 3,
        mq: Mq::Energy,
        unit: Unit::WattHour,
    },
];

impl ChannelDesc {
    /// Its value in a reply in the clear, as the C reaches it: the 32 bits as
    /// a float (`bv_get_value_len`), then that times the scale in double
    /// precision, back to a float (`sr_analog_to_float`). So above 2^24 the
    /// low bits go, as they do there.
    pub fn value(&self, plain: &[u8; POLL_LEN]) -> f32 {
        let raw = read_u32le(plain, self.offset) as f32;
        let scale = f64::from(self.scale.0) / f64::from(self.scale.1);
        (f64::from(raw) * scale) as f32
    }
}

fn read_u32le(data: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(data[at..at + 4].try_into().expect("four bytes"))
}

/// `sr_crc16` from `SR_CRC16_DEFAULT_INIT`.
fn crc16(data: &[u8]) -> u16 {
    data.iter().fold(0xffff, |crc, byte| {
        (0..8).fold(
            crc ^ u16::from(*byte),
            |c, _| {
                if c & 1 == 1 { (c >> 1) ^ 0xa001 } else { c >> 1 }
            },
        )
    })
}

/// AES-256 ECB: each 16 bytes on their own.
fn decrypt(data: &[u8; POLL_LEN]) -> [u8; POLL_LEN] {
    let cipher = Aes256Dec::new(&Array::from(KEY));
    let mut out = *data;
    for chunk in out.as_chunks_mut::<16>().0 {
        let mut block = Array::from(*chunk);
        cipher.decrypt_block(&mut block);
        *chunk = block.into();
    }
    out
}

/// `process_poll_pkt`: the reply in the clear, or the C's log line and `None`.
/// The checksum field's 32 bits are compared with the 16 of the CRC, so its
/// upper half has to be 0.
pub fn process_poll(data: &[u8; POLL_LEN]) -> Option<[u8; POLL_LEN]> {
    let plain = decrypt(data);
    let blocks = plain.as_chunks::<PAC_LEN>().0;
    if !blocks.iter().zip(MAGICS).all(|(block, magic)| block.starts_with(magic)) {
        log::error!("Invalid poll response packet (magic values).");
        return None;
    }
    let crc_good =
        |block: &[u8; PAC_LEN]| u32::from(crc16(&block[..CRC_POS])) == read_u32le(block, CRC_POS);
    if !blocks.iter().all(crc_good) {
        log::error!("Invalid poll response packet (checksum).");
        return None;
    }
    if log::log_enabled!(log::Level::Trace) {
        log::trace!("check passed on decrypted receive data");
        for (i, chunk) in plain.chunks(32).enumerate() {
            log::trace!("{:04x}  {}", i * 32, hexdump(chunk));
        }
    }
    Some(plain)
}

/// `sr_hexdump_new()`.
fn hexdump(data: &[u8]) -> String {
    let mut text = String::with_capacity(3 * data.len() + data.len() / 8 + data.len() / 16);
    for (i, byte) in data.iter().enumerate() {
        if i > 0 {
            text.push(' ');
            if i % 8 == 0 {
                text.push(' ');
            }
            if i % 16 == 0 {
                text.push(' ');
            }
        }
        let _ = write!(text, "{byte:02x}");
    }
    text
}

/// What a reply says of the meter.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DevInfo {
    /// The name.
    pub model: String,
    /// The firmware's version.
    pub version: String,
    /// The serial number, as `%08u` shows it.
    pub serial: String,
}

/// `g_strndup`: up to the first NUL. Not UTF-8 is shown lossily, where C
/// passes the bytes on.
fn field(plain: &[u8; POLL_LEN], at: usize) -> String {
    let bytes = &plain[at..at + FIELD_LEN];
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(FIELD_LEN);
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

/// The fields of `rdtech_tc_probe`.
pub fn dev_info(plain: &[u8; POLL_LEN]) -> DevInfo {
    DevInfo {
        model: field(plain, OFF_MODEL),
        version: field(plain, OFF_FW_VER),
        serial: format!("{:08}", read_u32le(plain, OFF_SERIAL)),
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use aes::Aes256Enc;
    use aes::cipher::BlockCipherEncrypt;

    /// A reply as a meter sends it, from the layout, for this module's tests
    /// and the driver's.
    pub(crate) fn reply(fill: impl Fn(&mut [u8; POLL_LEN])) -> [u8; POLL_LEN] {
        let mut plain = [0; POLL_LEN];
        for (i, magic) in MAGICS.iter().enumerate() {
            plain[i * PAC_LEN..][..4].copy_from_slice(*magic);
        }
        plain[OFF_MODEL..OFF_MODEL + 4].copy_from_slice(b"TC66");
        plain[OFF_FW_VER..OFF_FW_VER + 4].copy_from_slice(b"1.14");
        plain[OFF_SERIAL..OFF_SERIAL + 4].copy_from_slice(&1234u32.to_le_bytes());
        fill(&mut plain);
        seal(plain)
    }

    /// The checksums, then the cipher.
    fn seal(mut plain: [u8; POLL_LEN]) -> [u8; POLL_LEN] {
        for block in plain.as_chunks_mut::<PAC_LEN>().0 {
            let crc = crc16(&block[..CRC_POS]);
            block[CRC_POS..].copy_from_slice(&u32::from(crc).to_le_bytes());
        }
        encrypt(plain)
    }

    fn encrypt(mut plain: [u8; POLL_LEN]) -> [u8; POLL_LEN] {
        let cipher = Aes256Enc::new(&Array::from(KEY));
        for chunk in plain.as_chunks_mut::<16>().0 {
            let mut block = Array::from(*chunk);
            cipher.encrypt_block(&mut block);
            *chunk = block.into();
        }
        plain
    }

    fn put(plain: &mut [u8; POLL_LEN], at: usize, value: u32) {
        plain[at..at + 4].copy_from_slice(&value.to_le_bytes());
    }

    #[test]
    fn each_16_bytes_go_on_their_own() {
        let mut data = [0; POLL_LEN];
        data[..16].copy_from_slice(&[0x11; 16]);
        data[16..32].copy_from_slice(&[0x11; 16]);
        let plain = decrypt(&data);
        assert_eq!(plain[..16], plain[16..32]);
        assert_ne!(plain[..16], [0x11; 16]);
        assert_eq!(decrypt(&encrypt(plain)), plain);
    }

    #[test]
    fn the_first_block_of_a_known_reply_decrypts_under_the_key() {
        // The first 16 bytes `tests/emu/rdtech-tc/frames-2.tx` has on the wire
        // for the name TC66, version 1.14, serial 1234: through nettle, as
        // the C reads them.
        let mut data = [0; POLL_LEN];
        data[..16].copy_from_slice(&[
            0x6a, 0x30, 0x20, 0xe2, 0x16, 0x2c, 0x6a, 0x77, 0xa8, 0xe4, 0x15, 0xb9, 0x5c, 0x9d,
            0x18, 0x9a,
        ]);
        let plain = decrypt(&data);
        assert_eq!(&plain[..12], b"pac1TC661.14");
        assert_eq!(read_u32le(&plain, 12), 1234);
    }

    #[test]
    fn the_crc_is_modbus() {
        assert_eq!(crc16(b"123456789"), 0x4b37);
    }

    #[test]
    fn a_good_reply_passes_and_says_what_it_is() {
        let plain = process_poll(&reply(|_| {})).unwrap();
        let want = DevInfo {
            model: "TC66".to_owned(),
            version: "1.14".to_owned(),
            serial: "00001234".to_owned(),
        };
        assert_eq!(dev_info(&plain), want);
    }

    #[test]
    fn name_and_version_stop_at_a_nul() {
        let plain = process_poll(&reply(|p| {
            p[OFF_MODEL..OFF_MODEL + 4].copy_from_slice(b"TC\0x");
            p[OFF_FW_VER..OFF_FW_VER + 4].copy_from_slice(b"\0abc");
            put(p, OFF_SERIAL, u32::MAX);
        }))
        .unwrap();
        let info = dev_info(&plain);
        assert_eq!((info.model.as_str(), info.version.as_str()), ("TC", ""));
        assert_eq!(info.serial, "4294967295");
    }

    #[test]
    fn each_magic_counts() {
        for at in [0, 64, 128] {
            let mut plain = decrypt(&reply(|_| {}));
            plain[at + 3] ^= 1;
            assert!(process_poll(&seal(plain)).is_none(), "at {at}");
        }
    }

    #[test]
    fn each_checksum_counts_with_all_of_its_32_bits() {
        for block in 0..3 {
            let at = block * PAC_LEN;
            let mut plain = decrypt(&reply(|_| {}));
            plain[at + 20] ^= 1;
            assert!(process_poll(&encrypt(plain)).is_none(), "data of block {block}");
            let mut plain = decrypt(&reply(|_| {}));
            plain[at + CRC_POS + 2] = 1;
            assert!(process_poll(&encrypt(plain)).is_none(), "upper half of block {block}");
        }
    }

    #[test]
    fn values_scale_in_single_precision_as_the_c() {
        let value = |channel: usize, raw: u32| {
            let plain = decrypt(&reply(|p| put(p, CHANNELS[channel].offset, raw)));
            CHANNELS[channel].value(&plain)
        };
        assert_eq!(value(0, 50_136), (50_136.0f64 * (100.0 / 1e6)) as f32);
        assert_eq!(value(1, 12_446), (12_446.0f64 * (10.0 / 1e6)) as f32);
        assert_eq!(value(2, 62), 0.62);
        assert_eq!(value(3, 61), 0.61);
        assert_eq!(value(4, 1_001), 1.001);
        assert_eq!(value(5, 2_002), 2.002);
        // Through a double, not in single precision all the way.
        let mut plain = [0; POLL_LEN];
        for raw in (0..16_000_000u32).step_by(997) {
            put(&mut plain, CHANNELS[0].offset, raw);
            assert_eq!(CHANNELS[0].value(&plain), (f64::from(raw) * (100.0 / 1e6)) as f32, "{raw}");
        }
        // 2^24 + 1 is no float: it reads as 2^24.
        assert_eq!(value(4, 16_777_217), value(4, 16_777_216));
    }

    #[test]
    fn the_hexdump_is_the_cs() {
        let bytes: Vec<u8> = (0..18).collect();
        assert_eq!(hexdump(&bytes), "00 01 02 03 04 05 06 07  08 09 0a 0b 0c 0d 0e 0f   10 11",);
    }
}
