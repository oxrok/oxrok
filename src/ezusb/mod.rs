// Counterpart in sigrok:
//   libsigrok/src/ezusb.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Firmware upload for Cypress EZ-USB FX2 devices.
//!
//! The FX2 boot loader answers vendor request 0xa0 ("firmware load"):
//! `wValue` is the RAM address, the data are the bytes to write. Writing
//! 1 to the CPUCS register at 0xe600 holds the 8051 in reset, writing 0
//! releases it, after which the device renumerates with the descriptors
//! of the new firmware. Firmware files are raw images loaded at address
//! 0, as sigrok ships them, or Intel HEX.

#![forbid(unsafe_code)]

use crate::core::{Error, Result};
use crate::usb::{ControlRequest, Transport};
use std::time::Duration;

const REQ_FIRMWARE_LOAD: u8 = 0xa0;
const CPUCS: u16 = 0xe600;
const CHUNK: usize = 4096;
const TIMEOUT: Duration = Duration::from_millis(100);

/// One block of an image: an address and its bytes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Block {
    /// The load address.
    pub address: u16,
    /// The bytes.
    pub data: Vec<u8>,
}

/// Parses a firmware file: Intel HEX when it starts with `:`, else a
/// raw image at address 0.
pub fn parse_image(data: &[u8]) -> Result<Vec<Block>> {
    if data.first() == Some(&b':') {
        return parse_ihex(data);
    }
    Ok(data
        .chunks(CHUNK)
        .enumerate()
        .map(|(i, c)| Block { address: (i * CHUNK) as u16, data: c.to_vec() })
        .collect())
}

/// Parses Intel HEX records (types 00 data and 01 end; the FX2 has 16
/// bit addresses, so extended records are rejected).
pub fn parse_ihex(text: &[u8]) -> Result<Vec<Block>> {
    let text = std::str::from_utf8(text).map_err(|_| Error::device("firmware: not text"))?;
    let bad = |what: &str, n: usize| Error::Device(format!("firmware line {}: {what}", n + 1));
    let mut blocks: Vec<Block> = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let hex = line.strip_prefix(':').ok_or_else(|| bad("missing ':'", n))?;
        if hex.len() % 2 != 0 || hex.len() < 10 {
            return Err(bad("bad record length", n));
        }
        let bytes: Vec<u8> = (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16))
            .collect::<std::result::Result<_, _>>()
            .map_err(|_| bad("bad hex", n))?;
        let sum = bytes.iter().fold(0u8, |a, b| a.wrapping_add(*b));
        if sum != 0 {
            return Err(bad("checksum mismatch", n));
        }
        let len = usize::from(bytes[0]);
        if bytes.len() != len + 5 {
            return Err(bad("length mismatch", n));
        }
        let address = u16::from_be_bytes([bytes[1], bytes[2]]);
        match bytes[3] {
            0 => {
                let data = &bytes[4..4 + len];
                match blocks.last_mut() {
                    Some(b) if usize::from(b.address) + b.data.len() == usize::from(address) => {
                        b.data.extend_from_slice(data);
                    }
                    _ => blocks.push(Block { address, data: data.to_vec() }),
                }
            }
            1 => break,
            t => return Err(bad(&format!("unsupported record type {t:02x}"), n)),
        }
    }
    if blocks.is_empty() {
        return Err(Error::device("firmware: no data records"));
    }
    Ok(blocks)
}

fn load(t: &mut dyn Transport, address: u16, data: &[u8]) -> Result<()> {
    t.control_out(ControlRequest::vendor(REQ_FIRMWARE_LOAD, address, 0), data, TIMEOUT)
}

/// Holds the CPU in reset (`true`) or lets it run.
pub fn set_cpu_reset(t: &mut dyn Transport, reset: bool) -> Result<()> {
    load(t, CPUCS, &[u8::from(reset)])
}

/// `sr_resource_load()`'s limit in `ezusb_install_firmware()`: wValue,
/// which holds the address, has 16 bits.
const MAX_IMAGE: usize = 1 << 16;

/// Whether every block fits the 16 bit addresses. A longer one would
/// wrap to address 0 and overwrite the start of the image.
fn fits(blocks: &[Block]) -> bool {
    blocks.iter().all(|b| usize::from(b.address) + b.data.len() <= MAX_IMAGE)
}

/// The reset hold, the blocks in chunks of 4096 bytes, the release. With
/// `logged`, what `ezusb.c` logs for the file of that name.
fn install(t: &mut dyn Transport, blocks: &[Block], logged: Option<&str>) -> Result<()> {
    let name = |e: &Error| crate::usb::libusb_error_name(e);
    let reset = |t: &mut dyn Transport, on: bool| {
        if logged.is_some() {
            log::info!("setting CPU reset mode {}...", if on { "on" } else { "off" });
        }
        set_cpu_reset(t, on).inspect_err(|e| {
            if logged.is_some() {
                log::error!("Unable to send control request: {}.", name(e));
            }
        })
    };
    reset(t, true)?;
    if let Some(n) = logged {
        log::info!("Uploading firmware '{n}'.");
    }
    let mut total = 0;
    for b in blocks {
        for (i, chunk) in b.data.chunks(CHUNK).enumerate() {
            load(t, b.address.wrapping_add((i * CHUNK) as u16), chunk).inspect_err(|e| {
                if logged.is_some() {
                    log::error!("Unable to send firmware to device: {}.", name(e));
                }
            })?;
            if logged.is_some() {
                log::info!("Uploaded {} bytes.", chunk.len());
            }
            total += chunk.len();
        }
    }
    if logged.is_some() {
        log::info!("Firmware upload done.");
    } else {
        log::info!("Uploaded {total} bytes of firmware.");
    }
    reset(t, false)
}

/// Uploads a firmware image and starts it. The device renumerates
/// afterwards; the driver looks for it again.
pub fn upload_firmware(t: &mut dyn Transport, image: &[u8]) -> Result<()> {
    let blocks = parse_image(image)?;
    if !fits(&blocks) {
        return Err(Error::device("firmware: image does not fit 64 KiB"));
    }
    install(t, &blocks, None)
}

/// `ezusb_upload_firmware()` after its open and `SET_CONFIGURATION`:
/// [`upload_firmware`] for a raw image, logging what `ezusb.c` logs, so
/// that a driver's errors read as its libsigrok counterpart's (the C
/// code installs the file as one raw image).
pub fn upload_firmware_logged(t: &mut dyn Transport, name: &str, image: &[u8]) -> Result<()> {
    let blocks = [Block { address: 0, data: image.to_vec() }];
    if !fits(&blocks) {
        log::error!("Size {} of '{name}' exceeds limit {MAX_IMAGE}.", image.len());
        return Err(Error::device("firmware: image does not fit 64 KiB"));
    }
    install(t, &blocks, Some(name))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;

    #[test]
    fn ihex_records() {
        let text = ":03000000020300F8\n:0100030022DA\n:00000001FF\n";
        let b = parse_ihex(text.as_bytes()).unwrap();
        assert_eq!(b, [Block { address: 0, data: vec![2, 3, 0, 0x22] }]);
        assert!(parse_ihex(b":03000000020300F7\n").is_err());
        assert!(parse_ihex(b":020000040000FA\n").is_err());
        let raw = parse_image(&[1, 2, 3]).unwrap();
        assert_eq!(raw, [Block { address: 0, data: vec![1, 2, 3] }]);
    }

    /// An image past 64 KiB would wrap to address 0: refused before any
    /// transfer, whichever function uploads it; exactly 64 KiB fits.
    #[test]
    fn images_over_64_kib_are_refused() {
        let text = "# oxrok usb fixture 1\ndevice vid=77a1 pid=01a2\n";
        let mut r = Replay::from_text(text).unwrap();
        let big = vec![0x11u8; MAX_IMAGE + 1];
        assert!(upload_firmware(&mut r, &big).is_err());
        assert!(upload_firmware_logged(&mut r, "x.fw", &big).is_err());
        r.finish().unwrap();
        assert!(fits(&[Block { address: 0, data: vec![0; MAX_IMAGE] }]));
        assert!(!fits(&[Block { address: 0xf000, data: vec![0; 0x1001] }]));
    }

    #[test]
    fn logged_upload_stops_at_a_failed_chunk() {
        let image = vec![0x5au8; CHUNK + 1];
        let text = format!(
            "# oxrok usb fixture 1\n\
             device vid=77a1 pid=01a2\n\
             ctl_out 40 a0 e600 0000 01 -> ok\n\
             ctl_out 40 a0 0000 0000 5a{{{CHUNK}}} -> err device \"endpoint stalled\"\n"
        );
        let mut r = Replay::from_text(&text).unwrap();
        assert!(upload_firmware_logged(&mut r, "x.fw", &image).is_err());
        r.finish().unwrap();
    }

    #[test]
    fn upload_sequence() {
        let image = vec![0xa5u8; CHUNK + 3];
        let text = format!(
            "# oxrok usb fixture 1\n\
             device vid=04b4 pid=8613\n\
             ctl_out 40 a0 e600 0000 01 -> ok\n\
             ctl_out 40 a0 0000 0000 a5{{{CHUNK}}} -> ok\n\
             ctl_out 40 a0 1000 0000 a5a5a5 -> ok\n\
             ctl_out 40 a0 e600 0000 00 -> ok\n"
        );
        let mut r = Replay::from_text(&text).unwrap();
        upload_firmware(&mut r, &image).unwrap();
        r.finish().unwrap();
    }
}
