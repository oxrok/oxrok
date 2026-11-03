// Counterpart in sigrok:
//   libsigrok/src/hardware/saleae-logic16/protocol.c,
//   libsigrok/src/hardware/saleae-logic16/protocol.h,
//   Copyright (C) 2013 Marcus Comstedt <marcus@mc.pp.se>,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   with commits by Uwe Hermann, Aurelien Jacobs, Daniel Elstner,
//   Andreas Piesk, Lars-Peter Clausen, Peter Zotov, Tim Hatch
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Logic16's host protocol: the FX2 firmware's commands on bulk
//! endpoint 1, the FPGA behind it, and the sample format.
//!
//! Commands go out on bulk endpoint 1, one per transfer of at most 64
//! bytes, scrambled by [`encrypt`]; a command with an answer is
//! followed by a bulk IN on endpoint 1 of the answer's length, which
//! [`decrypt`] undoes. Both restart their state for every transfer.
//!
//! | byte | command | arguments | answer |
//! |---|---|---|---|
//! | `0x7d` | abort, synchronous | `0x55` | `0xaa` |
//! | `0x07` | read EEPROM | `0x33 0x81`, address, length | the bytes |
//! | `0x81` | read FPGA registers | count 1, address | the value |
//! | `0x80` | write FPGA registers | count, (address, value) pairs | none |
//! | `0x7e` | FPGA upload, init | none | none |
//! | `0x7f` | FPGA upload, data | length, up to 62 bytes | none |
//! | `0x7a` | LED table | offset, length, the bytes | none |
//! | `0x7b` | LED mode | animate, T2 reload (LE), divider, repeat | none |
//! | `0x01` | start acquisition | none | none |
//! | `0x02` | abort, asynchronous | none | none |
//!
//! The FPGA is SRAM based: opening the device uploads one of two
//! bitstreams (1.8 V and 3.3 V logic, or 5 V), tells the old register
//! map from the new one by the version register, primes the FPGA with
//! 16 EEPROM bytes and starts the LED's heartbeat. The mcupro clone
//! keeps its own design and answers 0x40 or 0x41 at register 0 before
//! any upload. A capture sets the clock (100 or 160 MHz), the divisor
//! and the channel mask, latches them with the `UPDATE` bit, then
//! `0x01` and `RUNNING` start it.
//!
//! Samples come on bulk endpoint 2 as little endian 16 bit words of 16
//! samples of one channel, the first sample in bit 15, the words going
//! round the selected channels in ascending order.

use crate::core::{Error, Result};
use crate::usb::Transport;
use std::time::Duration;

/// `encrypt()`: what the host sends.
pub fn encrypt(src: &[u8]) -> Vec<u8> {
    let (mut state1, mut state2) = (0x9bu8, 0x54u8);
    src.iter()
        .map(|&v| {
            let t = ((v ^ state2 ^ 0x2b).wrapping_sub(0x05) ^ 0x35).wrapping_sub(0x39);
            let t = ((t ^ state1 ^ 0x5a).wrapping_sub(0xb0) ^ 0x38).wrapping_sub(0x45);
            state2 = t;
            state1 = v;
            t
        })
        .collect()
}

/// `decrypt()`: what the host makes of the device's answers.
pub fn decrypt(src: &[u8]) -> Vec<u8> {
    let (mut state1, mut state2) = (0x9bu8, 0x54u8);
    src.iter()
        .map(|&v| {
            let t = ((v.wrapping_add(0x45) ^ 0x38).wrapping_add(0xb0) ^ 0x5a) ^ state1;
            let t = ((t.wrapping_add(0x39) ^ 0x35).wrapping_add(0x05) ^ 0x2b) ^ state2;
            state1 = t;
            state2 = v;
            t
        })
        .collect()
}

/// `map_eeprom_data()`: the second byte of each primer pair.
pub fn map_eeprom_data(v: u8) -> u8 {
    ((v ^ 0x80).wrapping_add(0x44) ^ 0xd5).wrapping_add(0x69)
}

/// `make_heartbeat()` for the 64 byte LED table: a half sine in 8 steps,
/// twice, then zeros.
pub fn heartbeat() -> [u8; 64] {
    let mut t = [0u8; 64];
    for i in 0..2 {
        for j in 0..8 {
            t[i * 8 + j] = ((j as f64 * std::f64::consts::PI / 8.0).sin() * 255.0) as u8;
        }
    }
    t
}

/// The logic levels, each with its bitstream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VoltageRange {
    /// 1.8 V and 3.3 V logic.
    Low,
    /// 5 V logic.
    High,
}

impl VoltageRange {
    /// Both, in the order of the threshold list.
    pub const ALL: [VoltageRange; 2] = [VoltageRange::Low, VoltageRange::High];

    /// The `voltage_threshold` pair of the range.
    pub fn threshold(self) -> (f64, f64) {
        match self {
            VoltageRange::Low => (0.7, 1.4),
            VoltageRange::High => (1.4, 3.6),
        }
    }

    /// The range of a threshold pair, within 0.1 V of each bound, as
    /// `std_double_tuple_idx()` matches it.
    pub fn from_threshold(low: f64, high: f64) -> Option<Self> {
        Self::ALL.into_iter().find(|r| {
            let (l, h) = r.threshold();
            (l - low).abs() < 0.1 && (h - high).abs() < 0.1
        })
    }

    /// The bitstream file (the driver's name for the 5 V one is `-33`).
    pub fn bitstream(self) -> &'static str {
        match self {
            VoltageRange::Low => "saleae-logic16-fpga-18.bitstream",
            VoltageRange::High => "saleae-logic16-fpga-33.bitstream",
        }
    }
}

/// Which FPGA design answers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Variant {
    /// The Saleae board with the old bitstream's register map.
    Original,
    /// The Saleae board with the new bitstream's register map.
    OriginalNewBitstream,
    /// The mcupro clone, whose FPGA keeps its design.
    Mcupro,
}

/// An FPGA register map with its status and mode bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegisterMap {
    /// The version register.
    pub version: u8,
    /// Status and control.
    pub status: u8,
    /// Channels 0 to 7.
    pub channel_low: u8,
    /// Channels 8 to 15.
    pub channel_high: u8,
    /// The sample rate divisor minus one.
    pub divisor: u8,
    /// The first primer byte.
    pub primer_data1: u8,
    /// The primer's latch.
    pub primer_control: u8,
    /// The mode register.
    pub mode: u8,
    /// The second primer byte.
    pub primer_data2: u8,
    /// Status bit: capturing.
    pub running: u8,
    /// Status bit: latch mode, divisor and channels.
    pub update: u8,
    /// Status bit, always read back set.
    pub unknown1: u8,
    /// Status bit: the FIFO overflowed.
    pub overflow: u8,
    /// Status bit the driver keeps set.
    pub unknown2: u8,
    /// Mode bit: the 160 MHz clock.
    pub clock: u8,
    /// Mode bit of the primer sequence.
    pub mode_unknown1: u8,
    /// Mode bit of the primer sequence.
    pub mode_unknown2: u8,
}

/// `fpga_register_map_old` and its bit maps.
pub const MAP_OLD: RegisterMap = RegisterMap {
    version: 0,
    status: 1,
    channel_low: 2,
    channel_high: 3,
    divisor: 4,
    primer_data1: 6,
    primer_control: 7,
    mode: 10,
    primer_data2: 12,
    running: 0x01,
    update: 0x02,
    unknown1: 0x08,
    overflow: 0x20,
    unknown2: 0x40,
    clock: 0x01,
    mode_unknown1: 0x40,
    mode_unknown2: 0x80,
};

/// `fpga_register_map_new` and its bit maps.
pub const MAP_NEW: RegisterMap = RegisterMap {
    version: 7,
    status: 15,
    channel_low: 1,
    channel_high: 6,
    divisor: 11,
    primer_data1: 14,
    primer_control: 2,
    mode: 4,
    primer_data2: 3,
    running: 0x20,
    update: 0x08,
    unknown1: 0x10,
    overflow: 0x01,
    unknown2: 0x04,
    clock: 0x04,
    mode_unknown1: 0x80,
    mode_unknown2: 0x01,
};

impl Variant {
    /// The register map the variant answers with.
    pub fn map(self) -> &'static RegisterMap {
        match self {
            Variant::OriginalNewBitstream => &MAP_NEW,
            Variant::Original | Variant::Mcupro => &MAP_OLD,
        }
    }
}

const BASE_CLOCK_0: u64 = 100_000_000;
const BASE_CLOCK_1: u64 = 160_000_000;
/// `MAX_SAMPLE_RATE`.
pub const MAX_SAMPLE_RATE: u64 = 100_000_000;
/// `MAX_SAMPLE_RATE_X_CH`: the rate times the channels.
pub const MAX_SAMPLE_RATE_X_CH: u64 = 300_000_000;

/// The clock and divisor of a sample rate, the checks of
/// `logic16_setup_acquisition()`: 100 MHz / n, else 160 MHz / n, n up
/// to 256, the rate at most 100 MHz and at most 300 MHz over the
/// `channels` enabled. `Ok((clock_160, divisor))`.
pub fn clock_setting(samplerate: u64, channels: u32) -> Result<(bool, u64)> {
    let refuse =
        |extra: &str| Err(Error::Arg(format!("Unable to sample at {samplerate}Hz{extra}.")));
    if samplerate == 0 || samplerate > MAX_SAMPLE_RATE {
        return refuse("");
    }
    let setting = if BASE_CLOCK_0.is_multiple_of(samplerate) && BASE_CLOCK_0 / samplerate <= 256 {
        (false, BASE_CLOCK_0 / samplerate)
    } else if BASE_CLOCK_1.is_multiple_of(samplerate) && BASE_CLOCK_1 / samplerate <= 256 {
        (true, BASE_CLOCK_1 / samplerate)
    } else {
        return refuse("");
    };
    if u64::from(channels) * samplerate > MAX_SAMPLE_RATE_X_CH {
        return refuse(" with this many channels");
    }
    Ok(setting)
}

/// Most transfers in flight (`NUM_SIMUL_TRANSFERS`).
pub const MAX_TRANSFERS: usize = 32;

/// Transfer size, queue depth and transfer timeout of a capture, as
/// `get_buffer_size()`, `get_number_of_transfers()` and `get_timeout()`
/// compute them from the bytes per millisecond (`rate * channels /
/// 8000`): 10 ms of data rounded up to 512 bytes, 500 ms in flight but
/// at most 32 transfers, and the time those take plus a quarter. `None`
/// below 8000 bytes a second, where the C code divides by zero
/// (`api.c:544`); [`clock_setting`] refuses those rates first.
pub fn transfer_plan(samplerate: u64, channels: u32) -> Option<(usize, usize, Duration)> {
    // bytes_per_ms() returns an unsigned int.
    let bpm = (samplerate * u64::from(channels) / 8000) as u32 as usize;
    if bpm == 0 {
        return None;
    }
    let size = (10 * bpm + 511) & !511;
    let depth = (500 * bpm / size).min(MAX_TRANSFERS);
    let ms = size * depth / bpm;
    Some((size, depth, Duration::from_millis((ms + ms / 4) as u64)))
}

/// `convert_sample_data()`: words of 16 samples of one channel each,
/// round the enabled channels, into 16 bit samples (channel `n` is bit
/// `n`). The state carries over from one transfer to the next.
#[derive(Clone, Debug)]
pub struct Converter {
    masks: Vec<u16>,
    data: [u16; 16],
    cur: usize,
}

impl Converter {
    /// A converter for the channels set in `channels`.
    pub fn new(channels: u16) -> Self {
        let masks = (0..16).map(|i| 1u16 << i).filter(|m| channels & m != 0).collect();
        Converter { masks, data: [0; 16], cur: 0 }
    }

    /// Converts `src` (a last odd byte is ignored), appending whole
    /// blocks of 16 samples to `out`; returns the samples added.
    pub fn convert(&mut self, src: &[u8], out: &mut Vec<u8>) -> usize {
        let mut added = 0;
        if self.masks.is_empty() {
            return 0;
        }
        for w in src.as_chunks::<2>().0 {
            let sample = u16::from_le_bytes(*w);
            let mask = self.masks[self.cur];
            for (i, d) in self.data.iter_mut().enumerate() {
                if sample & (0x8000 >> i) != 0 {
                    *d |= mask;
                }
            }
            self.cur += 1;
            if self.cur == self.masks.len() {
                self.cur = 0;
                for d in &mut self.data {
                    out.extend_from_slice(&d.to_le_bytes());
                    *d = 0;
                }
                added += 16;
            }
        }
        added
    }
}

const CMD_START_ACQUISITION: u8 = 0x01;
const CMD_ABORT_ACQUISITION_ASYNC: u8 = 0x02;
const CMD_READ_EEPROM: u8 = 0x07;
const CMD_WRITE_LED_TABLE: u8 = 0x7a;
const CMD_SET_LED_MODE: u8 = 0x7b;
const CMD_ABORT_ACQUISITION_SYNC: u8 = 0x7d;
const CMD_FPGA_UPLOAD_INIT: u8 = 0x7e;
const CMD_FPGA_UPLOAD_SEND_DATA: u8 = 0x7f;
const CMD_FPGA_WRITE_REGISTER: u8 = 0x80;
const CMD_FPGA_READ_REGISTER: u8 = 0x81;
const READ_EEPROM_COOKIE1: u8 = 0x33;
const READ_EEPROM_COOKIE2: u8 = 0x81;
const ABORT_ACQUISITION_SYNC_PATTERN: u8 = 0x55;
const EP1_OUT: u8 = 0x01;
const EP1_IN: u8 = 0x81;
const EP1_TIMEOUT: Duration = Duration::from_millis(1000);
/// The bitstream bytes per `0x7f` command.
pub const BITSTREAM_CHUNK: usize = 62;

/// `0x80`: write `regs`, at most 31 (address, value) pairs.
pub fn write_registers_command(regs: &[(u8, u8)]) -> Vec<u8> {
    let mut c = vec![CMD_FPGA_WRITE_REGISTER, regs.len() as u8];
    for &(a, v) in regs {
        c.extend_from_slice(&[a, v]);
    }
    c
}

/// `prime_fpga()`'s 16 register writes: with `mode` the mode register
/// without its `UNKNOWN2` bit, the first writes eight pairs, the others
/// the last six; each latches one EEPROM byte and its
/// [`map_eeprom_data`].
pub fn primer_commands(map: &RegisterMap, mode: u8, eeprom: &[u8; 16]) -> Vec<Vec<u8>> {
    let u1 = map.mode_unknown1 | mode;
    let mut regs = [
        (map.mode, mode),
        (map.mode, u1),
        (map.primer_data2, 0),
        (map.mode, u1 | map.mode_unknown2),
        (map.mode, u1),
        (map.primer_data1, 0),
        (map.primer_control, 1),
        (map.primer_control, 0),
    ];
    eeprom
        .iter()
        .enumerate()
        .map(|(i, &b)| {
            regs[2].1 = b;
            regs[5].1 = map_eeprom_data(b);
            write_registers_command(if i == 0 { &regs } else { &regs[2..] })
        })
        .collect()
}

/// The firmware's command channel and the FPGA behind it, with the
/// driver's knowledge of both (`struct dev_context`).
pub struct Link {
    transport: Box<dyn Transport>,
    /// The design that answers.
    pub variant: Variant,
    /// The bitstream loaded, `None` before the first upload.
    pub voltage_range: Option<VoltageRange>,
}

impl Link {
    /// A link over an open, claimed transport.
    pub fn new(transport: Box<dyn Transport>) -> Self {
        Link { transport, variant: Variant::Original, voltage_range: None }
    }

    /// The transport, for the sample stream.
    pub fn transport(&mut self) -> &mut dyn Transport {
        self.transport.as_mut()
    }

    /// Gives the transport back.
    pub fn into_transport(self) -> Box<dyn Transport> {
        self.transport
    }

    fn map(&self) -> &'static RegisterMap {
        self.variant.map()
    }

    /// `do_ep1_command()`: one scrambled command and, when `reply_len`
    /// is not 0, its answer of exactly that length.
    pub fn command(&mut self, command: &[u8], reply_len: usize) -> Result<Vec<u8>> {
        let what = command[0];
        let fail = |e: String| {
            log::debug!("Failed to send EP1 command {what:#04x}: {e}.");
            Error::Device(format!("EP1 command {what:#04x} failed: {e}"))
        };
        let n = self
            .transport
            .bulk_out(EP1_OUT, &encrypt(command), EP1_TIMEOUT)
            .map_err(|e| fail(e.to_string()))?;
        if n != command.len() {
            return Err(fail(format!("incorrect length {n} != {}", command.len())));
        }
        if reply_len == 0 {
            return Ok(Vec::new());
        }
        let fail = |e: String| {
            log::debug!("Failed to receive reply to EP1 command {what:#04x}: {e}.");
            Error::Device(format!("no reply to EP1 command {what:#04x}: {e}"))
        };
        let reply = self
            .transport
            .bulk_in(EP1_IN, reply_len, EP1_TIMEOUT)
            .map_err(|e| fail(e.to_string()))?;
        if reply.len() != reply_len {
            return Err(fail(format!("incorrect length {} != {reply_len}", reply.len())));
        }
        Ok(decrypt(&reply))
    }

    /// `read_eeprom()`.
    pub fn read_eeprom(&mut self, address: u8, length: u8) -> Result<Vec<u8>> {
        let c = [CMD_READ_EEPROM, READ_EEPROM_COOKIE1, READ_EEPROM_COOKIE2, address, length];
        self.command(&c, usize::from(length))
    }

    /// `read_fpga_register()`.
    pub fn read_register(&mut self, address: u8) -> Result<u8> {
        Ok(self.command(&[CMD_FPGA_READ_REGISTER, 1, address], 1)?[0])
    }

    /// `write_fpga_register()`.
    pub fn write_register(&mut self, address: u8, value: u8) -> Result<()> {
        self.command(&write_registers_command(&[(address, value)]), 0).map(drop)
    }

    /// `abort_acquisition_sync()`: the firmware answers the pattern
    /// inverted.
    pub fn abort_sync(&mut self) -> Result<()> {
        let reply =
            self.command(&[CMD_ABORT_ACQUISITION_SYNC, ABORT_ACQUISITION_SYNC_PATTERN], 1)?[0];
        let expected = !ABORT_ACQUISITION_SYNC_PATTERN;
        if reply != expected {
            let msg = format!(
                "Invalid response for abort acquisition command: {reply:#04x} != {expected:#04x}."
            );
            log::error!("{msg}");
            return Err(Error::Device(msg));
        }
        Ok(())
    }

    /// `logic16_init_device()`: the synchronous abort, the EEPROM's
    /// bytes 8 to 15 (unused, as in the C driver), the mcupro probe
    /// (whose failure says "original"), then the bitstream for `range`.
    pub fn init(&mut self, range: VoltageRange) -> Result<()> {
        self.voltage_range = None;
        self.abort_sync()?;
        self.read_eeprom(8, 8)?;
        let version = self.read_register(0);
        if let Ok(v @ (0x40 | 0x41)) = version {
            log::info!("mcupro Saleae16 detected (FPGA version {v:#04x}).");
            self.variant = Variant::Mcupro;
        } else {
            log::info!("Original Saleae Logic16 detected.");
            self.variant = Variant::Original;
        }
        self.upload_bitstream(range)
    }

    /// `upload_fpga_bitstream()`: nothing when `range`'s bitstream is
    /// loaded; else, but on the mcupro, the bitstream in 62 byte chunks,
    /// then the register map, the primer and the LED.
    pub fn upload_bitstream(&mut self, range: VoltageRange) -> Result<()> {
        if self.voltage_range == Some(range) {
            return Ok(());
        }
        if self.variant != Variant::Mcupro {
            let name = range.bitstream();
            log::info!("Uploading FPGA bitstream '{name}'.");
            let bitstream = crate::core::firmware::read(
                name,
                "extract it with sigrok-util, see docs/DEVICES.md",
            )?;
            self.command(&[CMD_FPGA_UPLOAD_INIT], 0)?;
            for chunk in bitstream.chunks(BITSTREAM_CHUNK) {
                let mut c = vec![CMD_FPGA_UPLOAD_SEND_DATA, chunk.len() as u8];
                c.extend_from_slice(chunk);
                self.command(&c, 0)?;
            }
            log::info!("FPGA bitstream upload ({} bytes) done.", bitstream.len());
        }
        self.setup_register_mapping()?;
        self.prime()?;
        self.configure_led()?;
        self.voltage_range = Some(range);
        Ok(())
    }

    /// `setup_register_mapping()`: the new map when register 0 reads 0
    /// and register 7, its version register, above 0x10.
    fn setup_register_mapping(&mut self) -> Result<()> {
        if self.variant != Variant::Mcupro {
            let reg0 = self.read_register(0)?;
            let reg7 = self.read_register(7)?;
            if reg0 == 0 && reg7 > 0x10 {
                log::info!("Original Saleae Logic16 using new bitstream.");
                self.variant = Variant::OriginalNewBitstream;
            } else {
                log::info!("Original Saleae Logic16 using old bitstream.");
                self.variant = Variant::Original;
            }
        }
        Ok(())
    }

    /// `prime_fpga()`.
    fn prime(&mut self) -> Result<()> {
        let map = self.map();
        let eeprom: [u8; 16] =
            self.read_eeprom(16, 16)?.try_into().map_err(|_| Error::device("short EEPROM read"))?;
        let mode = self.read_register(map.mode)? & !map.mode_unknown2;
        for c in primer_commands(map, mode, &eeprom) {
            self.command(&c, 0)?;
        }
        self.write_register(map.mode, mode)?;
        let version = self.read_register(map.version)?;
        if !matches!(version, 0x10 | 0x13 | 0x40 | 0x41) {
            log::warn!("Unsupported FPGA version: {version:#04x}.");
        }
        Ok(())
    }

    /// `configure_led()`: the heartbeat table in two chunks, then its
    /// mode.
    fn configure_led(&mut self) -> Result<()> {
        let table = heartbeat();
        for (i, chunk) in table.chunks(32).enumerate() {
            let mut c = vec![CMD_WRITE_LED_TABLE, (i * 32) as u8, chunk.len() as u8];
            c.extend_from_slice(chunk);
            self.command(&c, 0)?;
        }
        let [lo, hi] = 6250u16.to_le_bytes();
        self.command(&[CMD_SET_LED_MODE, 1, lo, hi, 0, 1], 0).map(drop)
    }

    /// The rest of `logic16_setup_acquisition()`, once the rate passed
    /// [`clock_setting`]: the bitstream for `range`, the status checked
    /// (a mismatch is logged, as in the C driver), clock, divisor and
    /// channels written and latched, and read back.
    pub fn setup_acquisition(
        &mut self,
        clock_160: bool,
        divisor: u64,
        channels: u16,
        range: VoltageRange,
    ) -> Result<()> {
        self.upload_bitstream(range)?;
        let m = *self.map();
        let check = self.variant != Variant::Mcupro;
        let status = self.read_register(m.status)? & !m.overflow;
        if check && status != m.unknown1 {
            log::debug!(
                "Invalid state at acquisition setup register 1: {status:#04x} != {:#04x}. \
                 Proceeding anyway.",
                m.unknown1
            );
        }
        let mode = if clock_160 { m.clock } else { 0 };
        self.write_register(m.status, m.unknown2)?;
        self.write_register(m.mode, mode)?;
        self.write_register(m.divisor, (divisor - 1) as u8)?;
        let [low, high] = channels.to_le_bytes();
        self.write_register(m.channel_low, low)?;
        self.write_register(m.channel_high, high)?;
        self.write_register(m.status, m.unknown2 | m.update)?;
        self.write_register(m.status, m.unknown2)?;
        let status = self.read_register(m.status)?;
        if check && status != m.unknown2 | m.unknown1 {
            log::debug!(
                "Invalid state at acquisition setup register 1: {status:#04x} != {:#04x}. \
                 Proceeding anyway.",
                m.unknown2 | m.unknown1
            );
        }
        let read = self.read_register(m.mode)?;
        if check && read != mode {
            log::debug!(
                "Invalid state at acquisition setup register 10: {read:#04x} != {mode:#04x}. \
                 Proceeding anyway."
            );
        }
        Ok(())
    }

    /// `logic16_start_acquisition()`.
    pub fn start_acquisition(&mut self) -> Result<()> {
        let m = *self.map();
        self.command(&[CMD_START_ACQUISITION], 0)?;
        self.write_register(m.status, m.unknown2 | m.running)
    }

    /// `logic16_abort_acquisition()`: the asynchronous abort, the
    /// status cleared and checked; registers 8 and 9 read on the old
    /// map, as the C driver does; a FIFO overflow is a warning.
    pub fn abort_acquisition(&mut self) -> Result<()> {
        let m = *self.map();
        let check = self.variant != Variant::Mcupro;
        self.command(&[CMD_ABORT_ACQUISITION_ASYNC], 0)?;
        self.write_register(m.status, 0)?;
        let status = self.read_register(m.status)?;
        if check && status & !m.overflow != m.unknown1 {
            // The C driver prints the status without bit 0x20, the old
            // map's overflow bit, on either map.
            log::debug!(
                "Invalid state at acquisition stop: {:#04x} != {:#04x}.",
                status & !0x20,
                m.unknown1
            );
            return Err(Error::device("the status register reads otherwise than expected"));
        }
        if self.variant == Variant::Original {
            self.read_register(8)?;
            self.read_register(9)?;
        }
        if check && status & m.overflow != 0 {
            log::warn!("FIFO overflow, capture data may be truncated.");
            return Err(Error::device("the sample buffer overflowed"));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap()).collect()
    }

    #[test]
    fn scrambling_matches_the_wire() {
        // The synchronous abort and its answer, as libsigrok sends and
        // reads them (tests/emu/saleae-logic16/flashed-4ch-2m.tx).
        assert_eq!(encrypt(&[0x7d, 0x55]), hex("61d4"));
        assert_eq!(decrypt(&hex("40")), [0xaa]);
        assert_eq!(encrypt(&[0x07, 0x33, 0x81, 8, 8]), hex("dfc6b35160"));
        assert_eq!(encrypt(&[0x81, 1, 0]), hex("55e0c4"));
        assert_eq!(encrypt(&[0x01]), hex("d5"));
        assert_eq!(encrypt(&[0x02]), hex("d8"));
        for len in [1, 2, 17, 64] {
            let data: Vec<u8> = (0..len).map(|i| (i * 37 + 11) as u8).collect();
            assert_eq!(decrypt(&encrypt(&data)), data);
        }
    }

    #[test]
    fn heartbeat_and_eeprom_mapping() {
        let t = heartbeat();
        assert_eq!(&t[..8], [0, 97, 180, 235, 255, 235, 180, 97]);
        assert_eq!(t[..8], t[8..16]);
        assert!(t[16..].iter().all(|&b| b == 0));
        assert_eq!(map_eeprom_data(0), ((0x80u8 + 0x44) ^ 0xd5).wrapping_add(0x69));
        assert_eq!(map_eeprom_data(0x80), (0x44 ^ 0xd5) + 0x69);
    }

    #[test]
    fn clock_settings() {
        assert_eq!(clock_setting(100_000_000, 1).unwrap(), (false, 1));
        assert_eq!(clock_setting(500_000, 16).unwrap(), (false, 200));
        assert_eq!(clock_setting(80_000_000, 2).unwrap(), (true, 2));
        assert_eq!(clock_setting(32_000_000, 2).unwrap(), (true, 5));
        assert_eq!(clock_setting(12_500_000, 8).unwrap(), (false, 8));
        assert_eq!(clock_setting(400_000, 2).unwrap(), (false, 250));
        assert_eq!(clock_setting(640_000, 2).unwrap(), (true, 250));
        let msg = |r: Result<(bool, u64)>| r.unwrap_err().to_string();
        assert!(msg(clock_setting(3_000_000, 1)).contains("Unable to sample at 3000000Hz."));
        assert!(msg(clock_setting(1000, 1)).contains("Unable to sample at 1000Hz."));
        assert!(clock_setting(0, 1).is_err());
        assert!(clock_setting(200_000_000, 1).is_err());
        assert!(
            msg(clock_setting(100_000_000, 4))
                .contains("Unable to sample at 100000000Hz with this many channels.")
        );
        assert!(clock_setting(100_000_000, 3).is_ok());
    }

    #[test]
    fn transfer_plans() {
        // 4 channels at 2 MHz: 1000 bytes a millisecond.
        assert_eq!(transfer_plan(2_000_000, 4), Some((10_240, 32, Duration::from_millis(408))));
        // 16 channels at 500 kHz, 1 at 100 MHz.
        assert_eq!(transfer_plan(500_000, 16).unwrap().0, 10_240);
        assert_eq!(transfer_plan(100_000_000, 1).unwrap().0, 125_440);
        assert_eq!(transfer_plan(16_000_000, 16).unwrap().0, 320_000);
        // 1 kHz on one channel: under 8000 bytes a second.
        assert_eq!(transfer_plan(1000, 1), None);
    }

    #[test]
    fn conversion_follows_the_word_order() {
        // Channels 0 and 3: a word of channel 0 then one of channel 3,
        // first sample in bit 15.
        let mut c = Converter::new(0b1001);
        let mut out = Vec::new();
        assert_eq!(c.convert(&0x8001u16.to_le_bytes(), &mut out), 0);
        assert_eq!(c.convert(&0xc000u16.to_le_bytes(), &mut out), 16);
        let samples: Vec<u16> =
            out.as_chunks::<2>().0.iter().map(|b| u16::from_le_bytes(*b)).collect();
        let mut want = vec![0u16; 16];
        want[0] = 0b1001;
        want[1] = 0b1000;
        want[15] = 0b0001;
        assert_eq!(samples, want);
        assert_eq!(Converter::new(0).convert(&[1, 2], &mut out), 0);
    }

    #[test]
    fn primer_sequence() {
        let eeprom = [0x11u8; 16];
        let cmds = primer_commands(&MAP_OLD, 0x00, &eeprom);
        assert_eq!(cmds.len(), 16);
        let d1 = map_eeprom_data(0x11);
        assert_eq!(
            cmds[0],
            [0x80, 8, 10, 0, 10, 0x40, 12, 0x11, 10, 0xc0, 10, 0x40, 6, d1, 7, 1, 7, 0]
        );
        assert_eq!(cmds[1], [0x80, 6, 12, 0x11, 10, 0xc0, 10, 0x40, 6, d1, 7, 1, 7, 0]);
        let cmds = primer_commands(&MAP_NEW, 0x04, &eeprom);
        assert_eq!(cmds[0][..6], [0x80, 8, 4, 0x04, 4, 0x84]);
    }

    #[test]
    fn thresholds() {
        assert_eq!(VoltageRange::from_threshold(0.7, 1.4), Some(VoltageRange::Low));
        assert_eq!(VoltageRange::from_threshold(1.45, 3.55), Some(VoltageRange::High));
        assert_eq!(VoltageRange::from_threshold(1.0, 2.0), None);
    }

    #[test]
    fn commands_go_scrambled_and_answers_come_back_plain() {
        // The second answer is the pattern itself, not inverted.
        let wrong = encrypt(&[0x55])[0];
        let text = format!(
            "# oxrok usb fixture 1\n\
             device vid=21a9 pid=1001\n\
             bulk_out 01 61d4 -> 2\n\
             bulk_in 81 1 -> 40\n\
             bulk_out 01 61d4 -> 2\n\
             bulk_in 81 1 -> {wrong:02x}\n"
        );
        let r = Replay::from_text(&text).unwrap();
        let check = r.clone();
        let mut link = Link::new(Box::new(r));
        link.abort_sync().unwrap();
        assert!(link.abort_sync().unwrap_err().to_string().contains("0x55 != 0xaa"));
        check.finish().unwrap();
    }
}
