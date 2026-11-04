// Counterpart in sigrok:
//   libsigrok/src/hardware/lascar-el-usb/protocol.c,
//   libsigrok/src/hardware/lascar-el-usb/protocol.h,
//   libsigrok/src/hardware/lascar-el-usb/api.c (the log request, lascar_proc_config()),
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Daniel Elstner,
//   Martin Ling, Soeren Apel, Aurelien Jacobs, Daniel Glöckner,
//   Gerhard Sittig, Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the EL-USB loggers and the decoding of their logs.
//!
//! A `SiLabs` F321 behind `10c4:0002`, bulk endpoints `0x02` and `0x82` of
//! 64 bytes:
//!
//! | command | answer |
//! |---|---|
//! | `00 ff ff` | `02 LEN LEN`, then the configuration block |
//! | `01 LEN LEN`, then the block | `ff` |
//! | `03 ff ff` | `02 LEN LEN`, then the whole sample memory |
//!
//! The block's bytes this driver reads: 0x00 the model id, 0x1c the interval
//! in seconds, 0x1e the stored samples, 0x20 the flags (bit 8 is logging),
//! 0x24 and 0x28 two CO calibration floats, 0x2e the temperature unit, 0x30
//! four bytes of firmware version, 0x34 the serial number. All little endian.
//!
//! Three vendor requests of no documented meaning come before a scan's read
//! and a download ("some sort of mode setup for the `SiLabs` F32x", and "Some
//! of these fail"): their outcome is ignored.
//!
//! The sequence is the C driver's with one difference: `protocol.c` submits
//! its read before writing the command, here the command goes first. Nothing
//! changes on the wire, as a read that comes first is not answered before the
//! command either, and eltuxusb does the same on real units.

use crate::core::analog::{Mq, MqFlags, Unit};
use crate::core::{AnalogData, Error, Result};
use crate::usb::{ControlRequest, Transport};
use std::time::{Duration, Instant};

/// The bulk OUT endpoint.
pub const EP_OUT: u8 = 0x02;
/// The bulk IN endpoint.
pub const EP_IN: u8 = 0x82;
/// `LASCAR_INTERFACE`.
pub const INTERFACE: u8 = 0;
/// `MAX_CONFIGBLOCK_SIZE`.
pub const MAX_CONFIG: usize = 256;
/// The bytes read of the log per transfer (`api.c:360`).
pub const LOG_READ: usize = 4096;

/// What a flush reads, each with this timeout.
const FLUSH_READ: usize = 256;
const FLUSH_TIMEOUT: Duration = Duration::from_millis(5);
/// `SCAN_TIMEOUT`: for both parts of a configuration read together.
const SCAN_TIMEOUT: Duration = Duration::from_millis(100);
/// `BULK_XFER_TIMEOUT`.
const BULK_XFER_TIMEOUT: Duration = Duration::from_secs(10);
const OUT_TIMEOUT: Duration = Duration::from_millis(100);
const CONTROL_TIMEOUT: Duration = Duration::from_millis(50);
/// The per transfer timeout of the log (`api.c:363`).
pub const LOG_TIMEOUT: Duration = Duration::from_millis(100);

/// How a profile logs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogFormat {
    /// Not supported by the C driver: no device.
    Unsupported,
    /// Pairs of temperature and humidity bytes.
    TempRh,
    /// Big endian 16 bit carbon monoxide counts.
    Co,
}

/// One line of `profiles[]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// The first byte of the configuration block.
    pub id: u8,
    /// The model's name.
    pub name: &'static str,
    /// How it logs.
    pub format: LogFormat,
}

const fn profile(id: u8, name: &'static str, format: LogFormat) -> Profile {
    Profile { id, name, format }
}

/// `profiles[]` of `protocol.c`.
pub const PROFILES: [Profile; 21] = [
    profile(1, "EL-USB-1", LogFormat::Unsupported),
    profile(2, "EL-USB-1", LogFormat::Unsupported),
    profile(3, "EL-USB-2", LogFormat::TempRh),
    profile(4, "EL-USB-3", LogFormat::Unsupported),
    profile(5, "EL-USB-4", LogFormat::Unsupported),
    profile(6, "EL-USB-3", LogFormat::Unsupported),
    profile(7, "EL-USB-4", LogFormat::Unsupported),
    profile(8, "EL-USB-LITE", LogFormat::Unsupported),
    profile(9, "EL-USB-CO", LogFormat::Co),
    profile(10, "EL-USB-TC", LogFormat::Unsupported),
    profile(11, "EL-USB-CO300", LogFormat::Co),
    profile(12, "EL-USB-2-LCD", LogFormat::TempRh),
    profile(13, "EL-USB-2+", LogFormat::TempRh),
    profile(14, "EL-USB-1-PRO", LogFormat::Unsupported),
    profile(15, "EL-USB-TC-LCD", LogFormat::Unsupported),
    profile(16, "EL-USB-2-LCD+", LogFormat::TempRh),
    profile(17, "EL-USB-5", LogFormat::Unsupported),
    profile(18, "EL-USB-1-RCG", LogFormat::Unsupported),
    profile(19, "EL-USB-1-LCD", LogFormat::Unsupported),
    profile(20, "EL-OEM-3", LogFormat::Unsupported),
    profile(21, "EL-USB-1-LCD", LogFormat::Unsupported),
];

/// The block's byte `at`, 0 past its end: the C reads its uninitialised
/// buffer there (`protocol.c:96`, 317).
fn byte(config: &[u8], at: usize) -> u8 {
    config.get(at).copied().unwrap_or(0)
}

fn word(config: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([byte(config, at), byte(config, at + 1)])
}

/// `binary32_le_to_float()`.
fn float(config: &[u8], at: usize) -> f32 {
    f32::from_le_bytes([
        byte(config, at),
        byte(config, at + 1),
        byte(config, at + 2),
        byte(config, at + 3),
    ])
}

/// `lascar_identify()`: the profile and the firmware version of a block.
pub fn identify(config: &[u8]) -> Option<(Profile, String)> {
    let id = byte(config, 0);
    if id == 0 {
        return None;
    }
    let Some(profile) = PROFILES.iter().find(|p| p.id == id) else {
        log::debug!("unknown EL-USB modelid {id}");
        return None;
    };
    let raw: Vec<u8> = (0x30..0x34).map(|at| byte(config, at)).collect();
    let end = raw.iter().position(|b| *b == 0).unwrap_or(raw.len());
    let firmware = String::from_utf8_lossy(&raw[..end]).into_owned();
    log::debug!(
        "found {} with firmware version {firmware} serial {}",
        profile.name,
        word(config, 0x34)
    );
    if profile.format == LogFormat::Unsupported {
        log::debug!("unsupported EL-USB logformat for {}", profile.name);
        return None;
    }
    Some((*profile, firmware))
}

/// The flags of a block, `get_flags()`.
pub fn flags(config: &[u8]) -> u16 {
    word(config, 0x20) & 0x1fff
}

/// `set_flags()`: it drops the top three bits of byte 0x21.
pub fn set_flags(config: &mut [u8], flags: u16) {
    let [lo, hi] = flags.to_le_bytes();
    config[0x20] = lo;
    config[0x21] = hi & 0x1f;
}

/// The bit of the flags that says the logger logs. The sigrok wiki says bit
/// 4 of 0x21; the C driver and eltuxusb use bit 0.
pub const FLAG_LOGGING: u16 = 0x0100;

/// How one log decodes, from `lascar_proc_config()`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Decoding {
    /// Pairs of bytes; `fahrenheit` is the block's unit.
    TempRh {
        /// The temperatures are in Fahrenheit.
        fahrenheit: bool,
    },
    /// Counts, with the calibration of the block.
    Co {
        /// The block's "calibration high".
        high: f32,
        /// The block's "calibration low".
        low: f32,
    },
}

/// `lascar_proc_config()` on a block: how the log decodes and its samples.
pub fn proc_config(format: LogFormat, config: &[u8]) -> Result<(Decoding, u32)> {
    let decoding = match format {
        LogFormat::TempRh => {
            let unit = word(config, 0x2e);
            if unit > 1 {
                log::debug!("invalid temperature unit {unit}");
            } else {
                log::debug!(
                    "temperature unit is {}",
                    if unit == 1 { "Fahrenheit" } else { "Celsius" }
                );
            }
            Decoding::TempRh { fahrenheit: unit == 1 }
        }
        LogFormat::Co => {
            let (high, low) = (float(config, 0x24), float(config, 0x28));
            log::debug!("EL-USB-CO calibration high {high:.6} low {low:.6}");
            Decoding::Co { high, low }
        }
        LogFormat::Unsupported => return Err(Error::arg("unsupported EL-USB logformat")),
    };
    let logged = word(config, 0x1e);
    log::debug!("device log contains {logged} samples.");
    Ok((decoding, u32::from(logged)))
}

/// The sample interval a block holds, in ms as `api.c:285` sends it.
pub fn interval_ms(config: &[u8]) -> u64 {
    u64::from(word(config, 0x1c)) * 1000
}

/// The samples of `pair`, a temperature and a humidity byte. `protocol.c:426`,
/// 430 divide integers, so that half a degree and half a percent are lost
/// although its comment says "half-degree increments" and it declares one
/// digit; eltuxusb (`el_parse.py:65`, 147) keeps them.
pub fn decode_pair(pair: [u8; 2], fahrenheit: bool) -> (f32, f32) {
    let temp = if fahrenheit { f32::from(pair[0]) - 40.0 } else { f32::from(pair[0]) / 2.0 - 40.0 };
    (temp, f32::from(pair[1]) / 2.0)
}

/// A CO sample, in single precision as `protocol.c:484` computes it.
pub fn decode_co(pair: [u8; 2], high: f32, low: f32) -> f32 {
    let s = f32::from(u16::from_be_bytes(pair));
    let v = (s * high + low) / 1_000_000.0;
    if v < 0.0 { 0.0 } else { v }
}

/// `lascar_el_usb_dispatch()` with the state it keeps: makes the packets of
/// one transfer. It honours the limit the C stores and never uses, cutting
/// the output: the memory is read to its end all the same.
#[derive(Clone, Debug)]
pub struct Decoder {
    /// How samples decode.
    pub decoding: Decoding,
    /// The samples the device has logged.
    pub logged: u32,
    /// Samples read so far, kept or not.
    pub received: u32,
    /// Samples kept per channel so far.
    pub sent: u64,
    /// The sample limit, 0 for none.
    pub limit: u64,
    /// The first channel, `Temp` or `CO`, is enabled.
    pub first: bool,
    /// `Hum` is enabled.
    pub second: bool,
}

impl Decoder {
    /// Decodes one transfer: a sample is two bytes, and an odd byte at the
    /// end of a transfer goes, as in C.
    pub fn packets(&mut self, buf: &[u8]) -> Vec<AnalogData> {
        let left = self.logged.saturating_sub(self.received);
        let samples = u32::try_from(buf.len() / 2).unwrap_or(u32::MAX).min(left);
        self.received += samples;
        let pairs = buf.as_chunks::<2>().0.iter().take(samples as usize);
        match self.decoding {
            Decoding::TempRh { fahrenheit } => {
                let mut temp = Vec::new();
                let mut rh = Vec::new();
                for pair in pairs {
                    let (t, h) = decode_pair(*pair, fahrenheit);
                    if t == 0.0 && h == 0.0 {
                        // "Skip invalid measurement."
                        continue;
                    }
                    temp.push(t);
                    rh.push(h);
                }
                let keep = self.keep(temp.len());
                temp.truncate(keep);
                rh.truncate(keep);
                let mut out = Vec::new();
                if keep > 0 && self.first {
                    let (unit, digits) =
                        if fahrenheit { (Unit::Fahrenheit, 0) } else { (Unit::Celsius, 1) };
                    out.push(analog(0, temp, Mq::Temperature, unit, digits));
                }
                if keep > 0 && self.second {
                    out.push(analog(1, rh, Mq::RelativeHumidity, Unit::Percentage, 1));
                }
                out
            }
            Decoding::Co { high, low } => {
                let mut co: Vec<f32> = pairs.map(|p| decode_co(*p, high, low)).collect();
                let keep = self.keep(co.len());
                co.truncate(keep);
                if keep > 0 && self.first {
                    // api and protocol set no digits for CO: 0 of sr_analog_init().
                    vec![analog(0, co, Mq::CarbonMonoxide, Unit::Concentration, 0)]
                } else {
                    Vec::new()
                }
            }
        }
    }

    /// How many of `n` new samples the limit lets through, counting them.
    fn keep(&mut self, n: usize) -> usize {
        let keep = if self.limit == 0 {
            n
        } else {
            usize::try_from(self.limit.saturating_sub(self.sent)).unwrap_or(usize::MAX).min(n)
        };
        self.sent += keep as u64;
        keep
    }
}

fn analog(channel: usize, samples: Vec<f32>, mq: Mq, unit: Unit, digits: i32) -> AnalogData {
    AnalogData { channels: vec![channel], samples, mq, flags: MqFlags::empty(), unit, digits }
}

/// An open logger.
pub struct Link {
    /// The device.
    pub transport: Box<dyn Transport>,
}

impl Link {
    /// The vendor requests of `lascar_open()`, each one's outcome ignored.
    pub fn mode_setup(&mut self) {
        for (request, value) in [(0x00, 0xffff), (0x02, 0x0002), (0x02, 0x0001)] {
            let req = ControlRequest::vendor(request, value, 0);
            if let Err(e) = self.transport.control_out(req, &[], CONTROL_TIMEOUT) {
                log::debug!("Mode setup request {request:#04x}/{value:#06x}: {e}.");
            }
        }
    }

    /// "Flush anything the F321 still has queued."
    fn flush(&mut self) -> Result<()> {
        loop {
            match self.transport.bulk_in(EP_IN, FLUSH_READ, FLUSH_TIMEOUT) {
                Ok(d) if !d.is_empty() => {}
                Err(Error::Disconnected) => return Err(Error::Disconnected),
                Ok(_) | Err(_) => return Ok(()),
            }
        }
    }

    /// `lascar_get_config()`: the block as the device gave it.
    ///
    /// # Errors
    ///
    /// When the device does not answer as it should; the debug log says how.
    /// The C closes the device on an invalid header (`protocol.c:155`), while
    /// the session goes on using it; here it stays open.
    pub fn read_config(&mut self) -> Result<Vec<u8>> {
        log::trace!("Reading config block.");
        self.flush()?;
        let start = Instant::now();
        // A refused command shows as the silence that follows.
        if let Err(Error::Disconnected) =
            self.transport.bulk_out(EP_OUT, &[0x00, 0xff, 0xff], OUT_TIMEOUT)
        {
            return Err(Error::Disconnected);
        }
        let header = match self.transport.bulk_in(EP_IN, MAX_CONFIG, left(start)) {
            Ok(h) => h,
            Err(Error::Timeout) => {
                log::debug!("no response");
                return Err(Error::device("no response"));
            }
            Err(_) => Vec::new(),
        };
        if header.len() != 3 {
            log::debug!("expected 3-byte header, got {} bytes", header.len());
            return Err(Error::device("config header"));
        }
        log::trace!(
            "Response to config request: {:#04x} {:#04x} {:#04x} ",
            header[0],
            header[1],
            header[2]
        );
        let len = usize::from(u16::from_le_bytes([header[1], header[2]]));
        if header[0] != 0x02 || len > MAX_CONFIG {
            log::debug!(
                "Invalid response to config request: 0x{:02x} 0x{:02x} 0x{:02x} ",
                header[0],
                header[1],
                header[2]
            );
            return Err(Error::device("config header"));
        }
        let block = match self.transport.bulk_in(EP_IN, len, left(start)) {
            Ok(b) => b,
            Err(Error::Timeout) => {
                log::debug!("Timeout waiting for configuration structure.");
                return Err(Error::device("config block"));
            }
            Err(Error::Disconnected) => return Err(Error::Disconnected),
            Err(_) => Vec::new(),
        };
        if block.len() != len {
            log::debug!("expected {len}-byte structure, got {} bytes", block.len());
            return Err(Error::device("config block"));
        }
        if block.is_empty() {
            return Err(Error::device("empty config block"));
        }
        Ok(block)
    }

    /// `lascar_save_config()`.
    fn save_config(&mut self, config: &[u8]) -> Result<()> {
        log::trace!("Writing config block.");
        self.flush()?;
        let [lo, hi] = u16::try_from(config.len()).unwrap_or(u16::MAX).to_le_bytes();
        // protocol.c:250-260 goes on, whatever became of each.
        for data in [&[0x01, lo, hi][..], config] {
            if let Err(Error::Disconnected) = self.transport.bulk_out(EP_OUT, data, OUT_TIMEOUT) {
                return Err(Error::Disconnected);
            }
        }
        let answer = self.transport.bulk_in(EP_IN, FLUSH_READ, BULK_XFER_TIMEOUT);
        if !matches!(answer.as_deref(), Ok([0xff])) {
            log::debug!("unexpected response after transfer");
            return Err(Error::device("save"));
        }
        Ok(())
    }

    /// `lascar_is_logging()`.
    ///
    /// # Errors
    ///
    /// When the block cannot be read.
    pub fn is_logging(&mut self) -> Result<bool> {
        Ok(flags(&self.read_config()?) & FLAG_LOGGING != 0)
    }

    /// `lascar_start_logging()` or `lascar_stop_logging()`.
    ///
    /// # Errors
    ///
    /// When the block cannot be read or the device does not take it.
    pub fn set_logging(&mut self, on: bool) -> Result<()> {
        let mut config = self.read_config()?;
        let block = apply_logging(&mut config, on);
        let done = self.save_config(block);
        log::info!("{} internal logging.", if on { "Started" } else { "Stopped" });
        done
    }

    /// The start of a download in `dev_acquisition_start()`, to the log's
    /// header: its bytes.
    ///
    /// # Errors
    ///
    /// When the device does not answer as it should; the debug log says how.
    pub fn request_log(&mut self) -> Result<u32> {
        self.mode_setup();
        self.flush()?;
        let out = match self.transport.bulk_out(EP_OUT, &[0x03, 0xff, 0xff], OUT_TIMEOUT) {
            Err(Error::Disconnected) => return Err(Error::Disconnected),
            r => r,
        };
        let header = self.transport.bulk_in(EP_IN, 4, BULK_XFER_TIMEOUT);
        let (Ok(_), Ok(header)) = (out, header) else {
            log::debug!("no response to log transfer request");
            return Err(Error::device("no response to the log request"));
        };
        if header.len() != 3 || header[0] != 2 {
            log::debug!("invalid response to log transfer request");
            return Err(Error::device("invalid response to the log request"));
        }
        Ok(u32::from(u16::from_le_bytes([header[1], header[2]])))
    }
}

/// What is left of `SCAN_TIMEOUT`, at least the millisecond a backend takes
/// (libusb reads 0 as no timeout).
fn left(start: Instant) -> Duration {
    SCAN_TIMEOUT.saturating_sub(start.elapsed()).max(Duration::from_millis(1))
}

/// What a start or a stop changes in a block: the flag, and on a start the
/// seconds until logging begins (`protocol.c:635`). A block too short for
/// the C's offsets is a whole one here, its absent bytes 0.
fn apply_logging(config: &mut Vec<u8>, on: bool) -> &[u8] {
    let len = config.len();
    config.resize(MAX_CONFIG, 0);
    let f = flags(config);
    set_flags(config, if on { f | FLAG_LOGGING } else { f & !FLAG_LOGGING });
    if on {
        config[0x18..0x1c].fill(0);
    }
    &config[..len]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;
    use std::fmt::Write as _;

    const NO_STEPS: &str = "# oxrok usb fixture 1\ndevice vid=10c4 pid=0002\n";

    fn block(id: u8) -> Vec<u8> {
        let mut c = vec![0u8; 128];
        c[0] = id;
        c[0x1c] = 60;
        c[0x1e] = 20;
        c[0x20] = 4;
        c[0x24..0x28].copy_from_slice(&0.5f32.to_le_bytes());
        c[0x28..0x2c].copy_from_slice(&(-40f32).to_le_bytes());
        c[0x30..0x34].copy_from_slice(b"v2.0");
        c[0x34] = 121;
        c[0x35] = 243;
        c
    }

    fn hex(b: &[u8]) -> String {
        let mut out = String::new();
        for x in b {
            write!(out, "{x:02x}").unwrap();
        }
        out
    }

    #[test]
    fn profiles_are_the_c_table() {
        assert_eq!(PROFILES.len(), 21);
        for (i, p) in PROFILES.iter().enumerate() {
            assert_eq!(usize::from(p.id), i + 1);
        }
        let supported: Vec<u8> =
            PROFILES.iter().filter(|p| p.format != LogFormat::Unsupported).map(|p| p.id).collect();
        assert_eq!(supported, [3, 9, 11, 12, 13, 16]);
    }

    #[test]
    fn identify_follows_the_table() {
        let (p, fw) = identify(&block(13)).unwrap();
        assert_eq!((p.name, fw.as_str()), ("EL-USB-2+", "v2.0"));
        assert_eq!(identify(&block(9)).unwrap().0.format, LogFormat::Co);
        assert_eq!(identify(&block(0)), None);
        assert_eq!(identify(&block(1)), None, "unsupported");
        assert_eq!(identify(&block(22)), None, "unknown");
        assert_eq!(identify(&[]), None, "nothing there");
        // A block the C would read past: no panic, no firmware.
        let (p, fw) = identify(&[3]).unwrap();
        assert_eq!((p.id, fw.as_str()), (3, ""));
        // Firmware stops at a NUL.
        let mut c = block(3);
        c[0x32] = 0;
        assert_eq!(identify(&c).unwrap().1, "v2");
    }

    #[test]
    fn flags_keep_thirteen_bits() {
        let mut c = block(3);
        c[0x21] = 0xff;
        assert_eq!(flags(&c), 0x1f04);
        set_flags(&mut c, 0x1f04 & !FLAG_LOGGING);
        assert_eq!(c[0x21], 0x1e);
    }

    #[test]
    fn apply_logging_changes_what_the_c_changes() {
        let mut c = block(3);
        c[0x18..0x1c].fill(9);
        c[0x21] = 0xe2;
        let on = apply_logging(&mut c.clone(), true).to_vec();
        assert_eq!(on.len(), 128);
        assert_eq!(on[0x21], 0x03);
        assert_eq!(&on[0x18..0x1c], &[0; 4]);
        let off = apply_logging(&mut c, false).to_vec();
        assert_eq!(off[0x21], 0x02);
        assert_eq!(&off[0x18..0x1c], &[9; 4], "a stop keeps the start time");
        // Shorter than the fields, as a device may say: no panic, the length stays.
        let mut short = vec![3u8; 8];
        assert_eq!(apply_logging(&mut short, true).len(), 8);
    }

    #[test]
    fn proc_config_reads_the_block() {
        let mut c = block(3);
        c[0x2e] = 1;
        assert_eq!(
            proc_config(LogFormat::TempRh, &c).unwrap(),
            (Decoding::TempRh { fahrenheit: true }, 20)
        );
        c[0x2e] = 7;
        let (d, _) = proc_config(LogFormat::TempRh, &c).unwrap();
        assert_eq!(d, Decoding::TempRh { fahrenheit: false }, "any other is Celsius");
        let (d, _) = proc_config(LogFormat::Co, &block(9)).unwrap();
        assert_eq!(d, Decoding::Co { high: 0.5, low: -40.0 });
        assert!(proc_config(LogFormat::Unsupported, &c).is_err());
        assert_eq!(interval_ms(&block(3)), 60_000);
        let mut c = block(3);
        c[0x1c] = 0xff;
        c[0x1d] = 0xff;
        assert_eq!(interval_ms(&c), 65_535_000, "no wrap");
    }

    #[test]
    fn pairs_decode_with_their_half_steps() {
        assert_eq!(decode_pair([120, 80], false), (20.0, 40.0));
        assert_eq!(decode_pair([127, 83], false), (23.5, 41.5), "the C says 23 and 41");
        assert_eq!(decode_pair([0, 0], false), (-40.0, 0.0));
        assert_eq!(decode_pair([255, 255], false), (87.5, 127.5));
        assert_eq!(decode_pair([120, 80], true), (80.0, 40.0), "Fahrenheit in whole degrees");
        assert_eq!(decode_pair([0, 1], true), (-40.0, 0.5));
    }

    #[test]
    fn co_decodes_big_endian_and_clamps() {
        // (400 * 0.5 - 40) / 1e6
        assert_eq!(decode_co([0x01, 0x90], 0.5, -40.0), 160.0 / 1_000_000.0);
        assert_eq!(decode_co([0, 60], 0.5, -40.0), 0.0, "negative");
        assert_eq!(decode_co([0xff, 0xff], 2e6, 0.0), 65535.0 * 2e6 / 1e6);
        assert!(decode_co([0, 1], f32::NAN, 0.0).is_nan(), "as in C");
    }

    fn decoder(decoding: Decoding, logged: u32, limit: u64) -> Decoder {
        Decoder { decoding, logged, received: 0, sent: 0, limit, first: true, second: true }
    }

    #[test]
    fn a_transfer_decodes_to_a_packet_per_channel() {
        let mut d = decoder(Decoding::TempRh { fahrenheit: false }, 2, 0);
        let p = d.packets(&[120, 80, 122, 82, 0xff, 0xff]);
        assert_eq!(p.len(), 2);
        assert_eq!(p[0].samples, [20.0, 21.0], "past the count goes");
        assert_eq!(p[1].samples, [40.0, 41.0]);
        assert_eq!((p[0].mq, p[0].unit, p[0].digits), (Mq::Temperature, Unit::Celsius, 1));
        assert_eq!(p[1].channels, [1]);
        assert_eq!(d.received, 2);
        assert_eq!(d.packets(&[120, 80]), [], "the count has run out");
        let mut d = decoder(Decoding::TempRh { fahrenheit: true }, 1, 0);
        let p = d.packets(&[120, 80]);
        assert_eq!((p[0].unit, p[0].digits), (Unit::Fahrenheit, 0));
    }

    #[test]
    fn invalid_samples_go_and_an_empty_packet_with_them() {
        let mut d = decoder(Decoding::TempRh { fahrenheit: false }, 3, 0);
        let p = d.packets(&[80, 0, 120, 80, 80, 0]);
        assert_eq!(p[0].samples, [20.0]);
        assert_eq!(d.received, 3, "they count");
        let mut d = decoder(Decoding::TempRh { fahrenheit: false }, 1, 0);
        assert_eq!(d.packets(&[80, 0]), []);
        // Half a degree with no humidity is a measurement here.
        let mut d = decoder(Decoding::TempRh { fahrenheit: false }, 1, 0);
        assert_eq!(d.packets(&[81, 0])[0].samples, [0.5]);
    }

    #[test]
    fn disabled_channels_send_nothing() {
        let mut d = decoder(Decoding::TempRh { fahrenheit: false }, 1, 0);
        d.first = false;
        let p = d.packets(&[120, 80]);
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].channels, [1]);
    }

    #[test]
    fn an_odd_byte_goes() {
        let mut d = decoder(Decoding::Co { high: 1.0, low: 0.0 }, 10, 0);
        assert_eq!(d.packets(&[0, 1, 9])[0].samples.len(), 1);
    }

    #[test]
    fn the_limit_cuts_the_output_and_not_the_count() {
        let mut d = decoder(Decoding::TempRh { fahrenheit: false }, 4, 3);
        let p = d.packets(&[120, 80, 122, 82]);
        assert_eq!(p[0].samples.len(), 2);
        let p = d.packets(&[124, 84, 126, 86]);
        assert_eq!(p[0].samples, [22.0], "cut");
        assert_eq!(p[1].samples, [42.0], "cut alike");
        assert_eq!(d.received, 4);
        assert_eq!(d.packets(&[]), []);
        let mut d = decoder(Decoding::Co { high: 1.0, low: 0.0 }, 4, 1);
        assert_eq!(d.packets(&[0, 1, 0, 2])[0].samples.len(), 1);
        assert_eq!(d.packets(&[0, 3]), []);
    }

    fn replay(steps: &str) -> (Link, Replay) {
        let r = Replay::from_text(&format!("{NO_STEPS}{steps}")).unwrap();
        (Link { transport: Box::new(r.clone()) }, r)
    }

    const FLUSHED: &str = "bulk_in 82 256 -> err timeout\n";

    #[test]
    fn a_config_read_flushes_then_asks() {
        let c = block(3);
        let (mut l, r) = replay(&format!(
            "bulk_in 82 256 -> 5a5a\n{FLUSHED}bulk_out 02 00ffff -> 3\n\
             bulk_in 82 256 -> 028000\nbulk_in 82 128 -> {}\n",
            hex(&c)
        ));
        assert_eq!(l.read_config().unwrap(), c);
        r.finish().unwrap();
    }

    #[test]
    fn config_read_failures() {
        for (steps, what) in [
            ("bulk_in 82 256 -> err timeout\n", "no answer"),
            ("bulk_in 82 256 -> 0280\n", "short header"),
            ("bulk_in 82 256 -> 078000\n", "bad header"),
            ("bulk_in 82 256 -> 020101\n", "257 bytes"),
            ("bulk_in 82 256 -> 028000\nbulk_in 82 128 -> err timeout\n", "no block"),
            ("bulk_in 82 256 -> 028000\nbulk_in 82 128 -> 0300\n", "short block"),
            ("bulk_in 82 256 -> 020000\nbulk_in 82 0 -> -\n", "empty block"),
        ] {
            let (mut l, r) = replay(&format!("{FLUSHED}bulk_out 02 00ffff -> 3\n{steps}"));
            assert!(l.read_config().is_err(), "{what}");
            r.finish().unwrap();
        }
    }

    #[test]
    fn a_save_writes_the_block_and_wants_ff() {
        let c = block(3);
        let mut on = c.clone();
        on[0x21] = 1;
        let steps = |answer: &str| {
            format!(
                "{FLUSHED}bulk_out 02 00ffff -> 3\nbulk_in 82 256 -> 028000\n\
                 bulk_in 82 128 -> {}\n{FLUSHED}bulk_out 02 018000 -> 3\n\
                 bulk_out 02 {} -> 128\nbulk_in 82 256 -> {answer}\n",
                hex(&c),
                hex(&on)
            )
        };
        let (mut l, r) = replay(&steps("ff"));
        l.set_logging(true).unwrap();
        r.finish().unwrap();
        let (mut l, r) = replay(&steps("00"));
        assert!(l.set_logging(true).is_err());
        r.finish().unwrap();
        let (mut l, r) = replay(&steps("err timeout"));
        assert!(l.set_logging(true).is_err());
        r.finish().unwrap();
    }

    #[test]
    fn a_download_starts_with_the_setup_and_a_flush() {
        let start = "ctl_out 40 00 ffff 0000 - -> ok\nctl_out 40 02 0002 0000 - -> ok\n\
                     ctl_out 40 02 0001 0000 - -> ok\nbulk_in 82 256 -> err timeout\n\
                     bulk_out 02 03ffff -> 3\n";
        let (mut l, r) = replay(&format!("{start}bulk_in 82 4 -> 020080\n"));
        assert_eq!(l.request_log().unwrap(), 32768);
        r.finish().unwrap();
        for answer in ["0200", "070080", "err timeout"] {
            let (mut l, r) = replay(&format!("{start}bulk_in 82 4 -> {answer}\n"));
            assert!(l.request_log().is_err(), "{answer}");
            r.finish().unwrap();
        }
    }
}
