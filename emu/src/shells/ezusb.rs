// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The EZ-USB shell is vusb's; what is here knows sigrok's fixtures.
//!
//! [`check_ezusb_c`] holds an upload against what libsigrok's
//! `ezusb_upload_firmware()` sends, byte for byte: reset held, the image
//! from address 0 in chunks of 4096 bytes (`FW_CHUNKSIZE`) with only the
//! last one shorter, reset released.
//!
//! The firmware files are stand-ins (`tools/emu-firmware.sh`):
//! SHA-256(`NAME:0`), SHA-256(`NAME:1`), ... cut to the file's size, which
//! [`standin`] rebuilds, so a model knows the exact bytes both tools
//! must upload.

pub use vusb::shells::ezusb::*;

use sha2::{Digest as _, Sha256};

use crate::model::Violations;

/// The chunk size of libsigrok's `ezusb_install_firmware()`.
pub const EZUSB_C_CHUNK: usize = 4096;

/// Holds what the host did against libsigrok's
/// `ezusb_upload_firmware()` uploading `image`: `CPUCS` 1, the image
/// at address 0 in chunks of [`EZUSB_C_CHUNK`] bytes in order (only
/// the last shorter), `CPUCS` 0, and the RAM holding exactly the
/// image. Each difference is reported.
pub fn check_ezusb_c(loader: &Loader, image: &[u8], v: &mut Violations) {
    let mut want = vec![Op::Cpucs(1)];
    for (i, c) in image.chunks(EZUSB_C_CHUNK).enumerate() {
        want.push(Op::Write { address: (i * EZUSB_C_CHUNK) as u16, len: c.len() });
    }
    want.push(Op::Cpucs(0));
    if loader.ops() != want {
        let ops = loader.ops();
        let first = ops.iter().zip(&want).position(|(a, b)| a != b).unwrap_or(0);
        v.report(format!(
            "firmware upload: {} steps where ezusb.c sends {} (reset hold, {} chunks of up to \
             {EZUSB_C_CHUNK} bytes from address 0, reset release); step {} is {:?}, expected {:?}",
            ops.len(),
            want.len(),
            want.len() - 2,
            first + 1,
            ops.get(first),
            want.get(first)
        ));
    }
    if loader.image() != image {
        let at = loader.image().iter().zip(image).position(|(a, b)| a != b);
        v.report(format!(
            "firmware upload: the RAM holds {} bytes that differ from the {} byte image{}",
            loader.image().len(),
            image.len(),
            at.map_or(String::new(), |a| format!(" (first at {a:#06x})"))
        ));
    }
}

/// The bytes `tools/emu-firmware.sh` writes for a stand-in:
/// SHA-256(`name:0`), SHA-256(`name:1`), ... cut to `size` bytes, so no
/// two 32 byte blocks of a file are alike and an upload that repeats or
/// reorders a chunk does not match.
pub fn standin(name: &str, size: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(size + 32);
    let mut i = 0usize;
    while out.len() < size {
        out.extend_from_slice(&Sha256::digest(format!("{name}:{i}").as_bytes()));
        i += 1;
    }
    out.truncate(size);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Reply, Setup};

    fn load(value: u16, length: u16) -> Setup {
        Setup { request_type: 0x40, request: REQ_FIRMWARE_LOAD, value, index: 0, length }
    }

    #[test]
    fn standins_differ_block_by_block() {
        let s = standin("x.fw", 40);
        assert_eq!(&s[..32], &Sha256::digest(b"x.fw:0")[..]);
        assert_eq!(&s[32..], &Sha256::digest(b"x.fw:1")[..8]);
        let big = standin("x.fw", 8192);
        assert_ne!(&big[..4096], &big[4096..], "two 4 KiB chunks must differ");
    }

    #[test]
    fn an_ezusb_c_upload_passes() {
        let image = standin("a.fw", EZUSB_C_CHUNK + 100);
        let mut l = Loader::new(Chip::Fx2lp);
        let mut v = Violations::default();
        assert_eq!(l.control(load(CPUCS, 1), &[1], &mut v), Some((Reply::ack(), false)));
        l.control(load(0, 4096), &image[..4096], &mut v);
        l.control(load(0x1000, 100), &image[4096..], &mut v);
        assert_eq!(l.control(load(CPUCS, 1), &[0], &mut v), Some((Reply::ack(), true)));
        check_ezusb_c(&l, &image, &mut v);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_what_ezusb_c_does_not_send() {
        let image = standin("a.fw", 5000);
        let mut v = Violations::default();
        // Chunks of 2048 are not what ezusb.c sends; a wrong byte shows.
        let mut l = Loader::new(Chip::Fx2lp);
        l.control(load(CPUCS, 1), &[1], &mut v);
        for (i, c) in image.chunks(2048).enumerate() {
            let mut c = c.to_vec();
            if i == 1 {
                c[0] ^= 1;
            }
            l.control(load((i * 2048) as u16, c.len() as u16), &c, &mut v);
        }
        l.control(load(CPUCS, 1), &[0], &mut v);
        assert!(v.is_empty());
        check_ezusb_c(&l, &image, &mut v);
        assert_eq!(v.drain().len(), 2, "chunk sizes and content");
    }
}
