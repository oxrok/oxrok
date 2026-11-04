// Counterpart in sigrok:
//   libsigrok/src/hardware/greatfet/protocol.c,
//   libsigrok/src/hardware/greatfet/protocol.h,
//   Copyright (C) 2019 Katherine J. Temkin <k@ktemkin.com>,
//   Copyright (C) 2019 Mikaela Szekely <qyriad@gmail.com>,
//   Copyright (C) 2023 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The libgreat command protocol as far as this driver uses it, and the
//! logic analyzer's sample memory.
//!
//! A command is a vendor control OUT, recipient endpoint, request 0x65,
//! holding class and verb as two little endian u32 and the verb's
//! arguments; `wIndex` bit 0 says no answer will be read. An answer is a
//! control IN of the same request. Verbs used, class 0 (core): 1 version
//! string, 3 serial number; class 0x10d (logic analyzer): 0 configure
//! (u32 rate, u8 channels; answers u32 rate, u32 buffer size, u8 endpoint),
//! 1 first pin, 2 alternate pin map, 3 start, 4 stop.
//!
//! Sample memory: 16 channels take two bytes a sample (little endian),
//! 5 to 8 one byte, 3 or 4 half a byte, 2 a quarter and 1 an eighth, the
//! first sample in the least significant bits. The firmware samples as
//! many channels from the first pin up as cover the enabled ones.

use crate::core::{Error, Result};
use crate::usb::{ControlRequest, Transport, libusb_error_name};
use std::time::Duration;

const REQUEST: u8 = 0x65;
const FLAG_SKIP_RESPONSE: u16 = 1;
/// `LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT`.
const REQUEST_TYPE: u8 = 0x42;
const CLASS_CORE: u32 = 0;
const VERB_READ_VERSION: u32 = 1;
const VERB_READ_SERIAL: u32 = 3;
const CLASS_LA: u32 = 0x10d;
const VERB_CONFIGURE: u32 = 0;
const VERB_FIRST_PIN: u32 = 1;
const VERB_ALT_PIN_MAP: u32 = 2;
const VERB_START: u32 = 3;
const VERB_STOP: u32 = 4;
/// `LOGIC_DEFAULT_TIMEOUT`.
const TIMEOUT: Duration = Duration::from_millis(1000);
/// What the C driver asks a string answer for: the allocated size of
/// a `GString` sized for 520 bytes, a power of two.
const STRING_READ: usize = 1024;
/// `want_len` of `greatfet_logic_config()`.
const CONFIGURE_ANSWER: usize = 9;
/// `BANDWIDTH_THRESHOLD`: 42 MHz of bytes, in bits a second.
pub const BANDWIDTH_THRESHOLD: u64 = 42_000_000 * 8;

/// How a capture of some channels uses the sample memory.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    /// The channels asked of the firmware: up to the highest enabled.
    pub capture_channels: u8,
    /// Only channels 8 to 15 are enabled: the firmware starts at pin 8.
    pub use_upper_pins: bool,
    /// Bytes per wire unit: two for 16 channels, else one.
    pub wire_unit: usize,
    /// Samples per byte of sample memory; 1 for 8 and 16 channels.
    pub points_per_byte: usize,
    /// Bits per sample when they are packed; 0 for a byte or more.
    pub channel_shift: u8,
    /// Both banks are enabled, which the firmware may not support.
    pub multi_bank: bool,
}

/// `greatfet_calc_capture_chans()`: from the mask of enabled channels;
/// `None` when none is.
pub fn plan(mask: u16) -> Option<Plan> {
    if mask == 0 {
        return None;
    }
    let highest = 15 - mask.leading_zeros() as u8;
    let (have_upper, have_lower) = (mask & 0xff00 != 0, mask & 0x00ff != 0);
    let use_upper_pins = have_upper && !have_lower;
    let capture_channels = if use_upper_pins { highest - 8 + 1 } else { highest + 1 };
    let fw_channels = capture_channels.next_power_of_two();
    let (wire_unit, points_per_byte) =
        if fw_channels > 8 { (2, 1) } else { (1, usize::from(8 / fw_channels)) };
    Some(Plan {
        capture_channels,
        use_upper_pins,
        wire_unit,
        points_per_byte,
        channel_shift: fw_channels % 8,
        multi_bank: have_upper && !use_upper_pins,
    })
}

impl Plan {
    /// `greatfet_logic_config()`'s estimate of the USB bandwidth, in bits
    /// a second.
    pub fn bandwidth(&self, rate: u32) -> u64 {
        let bits = u64::from(rate) * 8 / self.points_per_byte as u64;
        if self.use_upper_pins { bits } else { bits * self.wire_unit as u64 }
    }

    /// The session's samples of the sample memory `data`, `unit` bytes
    /// each, at most `max` of them when there is a limit.
    /// `greatfet_process_receive_data()`, except that packed samples are
    /// untangled at any `unit` (the C passes them on as they are when the
    /// feed has one byte a sample, `protocol.c:1311`) and the limit is
    /// kept to the sample (the C's to the byte of sample memory).
    pub fn samples(&self, data: &[u8], unit: usize, max: Option<u64>) -> Vec<u8> {
        let whole = |n: usize| max.map_or(n, |m| n.min(usize::try_from(m).unwrap_or(usize::MAX)));
        if self.wire_unit == unit && self.channel_shift == 0 {
            let n = whole(data.len() / unit);
            return data[..n * unit].to_vec();
        }
        let points = self.points_per_byte;
        let n = whole(data.len() * points);
        let bytes = n.div_ceil(points);
        let width = if self.channel_shift == 0 { 8 } else { u32::from(self.channel_shift) };
        let mask = ((1u16 << width) - 1) as u8;
        let mut out = Vec::with_capacity(bytes * points * unit);
        for &b in &data[..bytes] {
            for j in 0..points as u32 {
                let v = (b >> (j * width)) & mask;
                if unit == 1 {
                    out.push(v);
                } else if self.use_upper_pins {
                    out.extend_from_slice(&[0, v]);
                } else {
                    out.extend_from_slice(&[v, 0]);
                }
            }
        }
        out.truncate(n * unit);
        out
    }
}

/// What configure answers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Configured {
    /// The rate the firmware will use.
    pub rate: u32,
    /// The firmware's own buffer; information only.
    pub buffer_size: u32,
    /// The samples' endpoint.
    pub endpoint: u8,
}

/// A string answer: text up to its first NUL when it starts with text, or
/// else the bytes as they came (`greatfet_get_string()`).
fn split_string(answer: &[u8]) -> &[u8] {
    if answer.first().is_some_and(|b| *b != 0) {
        answer.split(|b| *b == 0).next().unwrap_or(answer)
    } else {
        answer
    }
}

/// `greatfet_get_serial_number()` on its answer: text as it is, or the
/// four u32 some firmware sends, the first two zero, as twenty hex digits.
/// `None` for anything else.
pub fn parse_serial(answer: &[u8]) -> Option<String> {
    let text = split_string(answer);
    if text.first().is_some_and(|b| *b != 0) {
        return Some(String::from_utf8_lossy(text).into_owned());
    }
    let words: [u8; 16] = text.try_into().ok()?;
    let word = |i: usize| u32::from_le_bytes([words[i], words[i + 1], words[i + 2], words[i + 3]]);
    if word(0) != 0 || word(4) != 0 {
        return None;
    }
    Some(format!("{:04x}{:08x}{:08x}", word(4), word(8), word(12)))
}

/// `greatfet_get_version_number()` on its answer: what prints of it.
pub fn parse_version(answer: &[u8]) -> String {
    let text = split_string(answer);
    let end = text.iter().position(|b| *b == 0).unwrap_or(text.len());
    String::from_utf8_lossy(&text[..end]).into_owned()
}

/// Commands to the firmware, over the transport.
pub struct Link {
    transport: Box<dyn Transport>,
}

fn command(class: u32, verb: u32, args: &[u8]) -> Vec<u8> {
    let mut c = Vec::with_capacity(8 + args.len());
    c.extend_from_slice(&class.to_le_bytes());
    c.extend_from_slice(&verb.to_le_bytes());
    c.extend_from_slice(args);
    c
}

fn request(index: u16) -> ControlRequest {
    ControlRequest { request_type: REQUEST_TYPE, request: REQUEST, value: 0, index }
}

impl Link {
    /// A link over `transport`.
    pub fn new(transport: Box<dyn Transport>) -> Link {
        Link { transport }
    }

    /// The transport, for the claims and the stream.
    pub fn transport(&mut self) -> &mut dyn Transport {
        self.transport.as_mut()
    }

    /// `greatfet_ctrl_out_in()`, the OUT stage. A busy device's message
    /// is rate limited in the C; it has none here either.
    fn send(&mut self, data: &[u8], index: u16) -> Result<()> {
        self.transport.control_out(request(index), data, TIMEOUT).inspect_err(|e| {
            if !matches!(e, Error::Busy) {
                let name = libusb_error_name(e);
                log::error!("USB out transfer failed: {name} ({})", error_code(name));
            }
        })
    }

    /// `greatfet_ctrl_out_in()` with an answer.
    fn ask(&mut self, data: &[u8], len: usize) -> Result<Vec<u8>> {
        self.send(data, 0)?;
        self.transport.control_in(request(0), len, TIMEOUT).inspect_err(|e| {
            let name = libusb_error_name(e);
            log::error!("USB in transfer failed: {name} ({})", error_code(name));
        })
    }

    /// `greatfet_trivial_request()`, a command with no answer.
    fn tell(&mut self, verb: u32, args: &[u8]) -> Result<()> {
        self.send(&command(CLASS_LA, verb, args), FLAG_SKIP_RESPONSE)
    }

    fn core_string(&mut self, verb: u32) -> Result<Vec<u8>> {
        self.ask(&command(CLASS_CORE, verb, &[]), STRING_READ).inspect_err(|_| {
            log::error!("Cannot get core string.");
        })
    }

    /// The serial number, as text.
    ///
    /// # Errors
    ///
    /// On a failed request or an answer that is no serial number.
    pub fn read_serial(&mut self) -> Result<String> {
        let answer = self.core_string(VERB_READ_SERIAL)?;
        parse_serial(&answer).ok_or_else(|| Error::device("no serial number in the answer"))
    }

    /// The firmware version, as text.
    ///
    /// # Errors
    ///
    /// On a failed request.
    pub fn read_version(&mut self) -> Result<String> {
        Ok(parse_version(&self.core_string(VERB_READ_VERSION)?))
    }

    /// `greatfet_logic_config()` up to the answer: first pin, alternate
    /// map off, configure. The rate goes out in 32 bits, as the C's
    /// `write_u32le_inc()` truncates its u64: the firmware's field is that
    /// wide.
    ///
    /// # Errors
    ///
    /// On a failed request or an answer of another length, logged.
    pub fn configure(&mut self, plan: &Plan, rate: u64) -> Result<Configured> {
        log::debug!("about to config first pin, upper {}", i32::from(plan.use_upper_pins));
        let first = if plan.use_upper_pins { 8 } else { 0 };
        self.tell(VERB_FIRST_PIN, &[first]).inspect_err(|_| {
            log::error!("Cannot configure first capture pin.");
        })?;
        log::debug!("about to config alt pin mapping");
        self.tell(VERB_ALT_PIN_MAP, &[0]).inspect_err(|_| {
            log::error!("Cannot configure alt pin mapping.");
        })?;
        log::debug!("about to config LA, rate {rate}, chans {}", plan.capture_channels);
        let mut args = (rate as u32).to_le_bytes().to_vec();
        args.push(plan.capture_channels);
        let answer = self
            .ask(&command(CLASS_LA, VERB_CONFIGURE, &args), CONFIGURE_ANSWER)
            .inspect_err(|_| log::error!("Cannot configure logic analyzer mode."))?;
        let &[r0, r1, r2, r3, b0, b1, b2, b3, endpoint] = answer.as_slice() else {
            log::warn!("Unexpected LA configuration response length.");
            return Err(Error::device("Unexpected LA configuration response length."));
        };
        let c = Configured {
            rate: u32::from_le_bytes([r0, r1, r2, r3]),
            buffer_size: u32::from_le_bytes([b0, b1, b2, b3]),
            endpoint,
        };
        log::debug!("LA configured, rate {}, buf {}, ep {endpoint}", c.rate, c.buffer_size);
        Ok(c)
    }

    /// `greatfet_logic_start()`.
    ///
    /// # Errors
    ///
    /// On a failed request, logged.
    pub fn start(&mut self) -> Result<()> {
        self.tell(VERB_START, &[]).inspect_err(|_| {
            log::error!("Cannot start logic analyzer capture.");
        })
    }

    /// `greatfet_logic_stop()`: a failure is only worth a warning.
    pub fn stop(&mut self) {
        if self.tell(VERB_STOP, &[]).is_err() {
            log::warn!("Cannot stop logic analyzer capture in the device.");
        }
    }
}

/// The number `libusb_error_name()`'s name stands for, as the C's messages
/// print it.
fn error_code(name: &str) -> i32 {
    match name {
        "LIBUSB_ERROR_ACCESS" => -3,
        "LIBUSB_ERROR_NO_DEVICE" => -4,
        "LIBUSB_ERROR_BUSY" => -6,
        "LIBUSB_ERROR_TIMEOUT" => -7,
        "LIBUSB_ERROR_PIPE" => -9,
        "LIBUSB_ERROR_NOT_SUPPORTED" => -12,
        _ => -1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;

    fn replay(lines: &str) -> Replay {
        Replay::from_text(&format!(
            "# oxrok usb fixture 1\ndevice vid=1d50 pid=60e6 version=0100 interfaces=0:ff:ff:ff,1:ff:ff:ff\n{lines}"
        ))
        .unwrap()
    }

    #[test]
    fn plans_follow_the_c_driver() {
        let p = plan(0x0001).unwrap();
        assert_eq!((p.capture_channels, p.points_per_byte, p.channel_shift), (1, 8, 1));
        let p = plan(0x0002).unwrap();
        assert_eq!((p.capture_channels, p.points_per_byte, p.channel_shift), (2, 4, 2));
        let p = plan(0x0007).unwrap();
        assert_eq!((p.capture_channels, p.points_per_byte, p.channel_shift), (3, 2, 4));
        let p = plan(0x0010).unwrap();
        assert_eq!((p.capture_channels, p.points_per_byte, p.channel_shift), (5, 1, 0));
        assert_eq!(p.wire_unit, 1);
        let p = plan(0x0100).unwrap();
        assert!(p.use_upper_pins && !p.multi_bank);
        assert_eq!((p.capture_channels, p.points_per_byte, p.channel_shift), (1, 8, 1));
        let p = plan(0xff00).unwrap();
        assert_eq!((p.capture_channels, p.wire_unit, p.channel_shift), (8, 1, 0));
        let p = plan(0x0101).unwrap();
        assert!(!p.use_upper_pins && p.multi_bank);
        assert_eq!((p.capture_channels, p.wire_unit, p.points_per_byte), (9, 2, 1));
        assert_eq!(plan(0), None);
    }

    #[test]
    fn bandwidths_are_those_the_c_comments_name() {
        // The maxima of api.c's rate table.
        for (mask, top, over) in [
            (0xffffu16, 20_400_000, 25_500_000),
            (0x00ff, 40_800_000, 51_000_000),
            (0x000f, 68_000_000, 102_000_000),
            (0x0003, 102_000_000, 204_000_000),
        ] {
            let p = plan(mask).unwrap();
            assert!(p.bandwidth(top) <= BANDWIDTH_THRESHOLD, "{mask:#x} at {top}");
            assert!(p.bandwidth(over) > BANDWIDTH_THRESHOLD, "{mask:#x} at {over}");
        }
        assert!(plan(0x0001).unwrap().bandwidth(204_000_000) <= BANDWIDTH_THRESHOLD);
        assert_eq!(plan(0x0100).unwrap().bandwidth(u32::MAX), u64::from(u32::MAX));
    }

    #[test]
    fn samples_unpack_as_the_memory_holds_them() {
        // 2 bits: the first in the low bits.
        let p = plan(0x0003).unwrap();
        assert_eq!(p.samples(&[0b1110_0100], 2, None), vec![0, 0, 1, 0, 2, 0, 3, 0]);
        assert_eq!(p.samples(&[0b1110_0100], 1, None), vec![0, 1, 2, 3]);
        // 1 bit.
        let p = plan(0x0001).unwrap();
        assert_eq!(p.samples(&[0b0000_0101], 1, Some(4)), vec![1, 0, 1, 0]);
        // 4 bits, the upper pins into the high byte.
        let p = plan(0x0f00).unwrap();
        assert_eq!(p.samples(&[0xa5], 2, None), vec![0, 5, 0, 0xa]);
        // 8 bits into a wider feed, and as they are.
        let p = plan(0x00ff).unwrap();
        assert_eq!(p.samples(&[1, 2], 2, None), vec![1, 0, 2, 0]);
        assert_eq!(p.samples(&[1, 2], 1, None), vec![1, 2]);
        let p = plan(0xff00).unwrap();
        assert_eq!(p.samples(&[1, 2], 2, None), vec![0, 1, 0, 2]);
        // 16 bits, a stray byte dropped.
        let p = plan(0xffff).unwrap();
        assert_eq!(p.samples(&[1, 2, 3], 2, None), vec![1, 2]);
    }

    #[test]
    fn a_limit_cuts_to_the_sample() {
        let p = plan(0x0003).unwrap();
        let data = [0xe4, 0xe4];
        assert_eq!(p.samples(&data, 1, Some(3)), vec![0, 1, 2]);
        assert_eq!(p.samples(&data, 1, Some(5)), vec![0, 1, 2, 3, 0]);
        assert_eq!(p.samples(&data, 1, Some(100)).len(), 8);
        assert_eq!(p.samples(&data, 1, Some(u64::MAX)).len(), 8);
        assert_eq!(p.samples(&data, 1, Some(0)).len(), 0);
        let p = plan(0xffff).unwrap();
        assert_eq!(p.samples(&[1, 2, 3, 4], 2, Some(1)), vec![1, 2]);
    }

    #[test]
    fn serial_numbers() {
        assert_eq!(parse_serial(b"GFS123").as_deref(), Some("GFS123"));
        assert_eq!(parse_serial(b"GFS\x00123").as_deref(), Some("GFS"));
        let mut words = vec![0; 8];
        words.extend_from_slice(&0x57cc_67e6u32.to_le_bytes());
        words.extend_from_slice(&0x3018_7657u32.to_le_bytes());
        assert_eq!(parse_serial(&words).as_deref(), Some("000057cc67e630187657"));
        assert_eq!(parse_serial(&[]), None);
        assert_eq!(parse_serial(&[0; 8]), None);
        let mut high = vec![0; 16];
        high[1] = 1;
        assert_eq!(parse_serial(&high), None);
        let mut second = vec![0; 16];
        second[4] = 1;
        assert_eq!(parse_serial(&second), None);
    }

    #[test]
    fn versions() {
        assert_eq!(parse_version(b"v2021.2.1\0"), "v2021.2.1");
        assert_eq!(parse_version(b"v20"), "v20");
        assert_eq!(parse_version(b""), "");
        assert_eq!(parse_version(&[0, 0xff, 0]), "");
    }

    #[test]
    fn configure_sends_the_c_drivers_requests() {
        let r = replay(
            "ctl_out 42 65 0000 0001 0d0100000100000008 -> ok\n\
             ctl_out 42 65 0000 0001 0d0100000200000000 -> ok\n\
             ctl_out 42 65 0000 0000 0d010000000000000100000002 -> ok\n\
             ctl_in 42 65 0000 0000 9 -> 010000000040000081\n",
        );
        let mut link = Link::new(Box::new(r.clone()));
        // 2^32 + 1: the setting goes out in 32 bits.
        let c = link.configure(&plan(0x0200).unwrap(), (1 << 32) + 1).unwrap();
        assert_eq!(c, Configured { rate: 1, buffer_size: 0x4000, endpoint: 0x81 });
        r.finish().unwrap();
    }

    #[test]
    fn a_short_answer_to_configure_is_refused() {
        let r = replay(
            "ctl_out 42 65 0000 0001 0d0100000100000000 -> ok\n\
             ctl_out 42 65 0000 0001 0d0100000200000000 -> ok\n\
             ctl_out 42 65 0000 0000 0d010000000000000100000001 -> ok\n\
             ctl_in 42 65 0000 0000 9 -> 01000000\n",
        );
        let mut link = Link::new(Box::new(r.clone()));
        assert!(link.configure(&plan(0x0001).unwrap(), 1).is_err());
    }
}
