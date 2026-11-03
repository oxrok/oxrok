// Counterpart in sigrok:
//   libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.c,
//   libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.h,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   with commits by Uwe Hermann, Gerhard Sittig, Joerg Alpers,
//   Romain Tartière
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The DSLogic's host protocol: the FX2 firmware's vendor requests, the
//! FPGA behind it, its setting packet and the sample format.
//!
//! | request | direction | data | what |
//! |---|---|---|---|
//! | `0xb0` | IN | 2 bytes | firmware version, major 1 required |
//! | `0xb1` | IN | 1 byte | REVID (1: a CY7C68013A) |
//! | `0xb2` | OUT | flags, two delay bytes | start (`0x30`) or stop (`0x80`) |
//! | `0xb3` | OUT | 3 zero bytes | the FPGA bitstream follows on bulk OUT 2 |
//! | `0xb4` | OUT | length in words, 3 bytes LE | the setting packet follows on bulk OUT 2 |
//! | `0xb8` | OUT | value, address | a register write (`0x78`: the threshold DAC) |
//!
//! The FPGA is SRAM based: every open loads its bitstream (one per
//! model; the first DSLogic has one per threshold range, and choosing
//! the range loads it again) in one bulk transfer, then the threshold
//! register gets `threshold / 5 V * 255`. A capture sends the stop
//! command, the 360 byte setting packet ([`FpgaConfig`]: mode bits,
//! divider of 100 MHz, sample count / 16, trigger position, global
//! trigger word, channel mask and 16 trigger stages, all in
//! type-length-value fields), then the start command. The FPGA answers
//! with a 512 byte trigger packet on bulk IN 6 whose second word is the
//! trigger's sample in the data that follows, then the samples: for
//! each 64 samples, one little endian 64 bit word per enabled channel
//! in index order, bit `b` the channel at sample `b`.
//!
//! What follows the C driver's sequence and what does not is said at
//! each function; the libsigrok bugs not copied are listed in the
//! module documentation of the driver.

#![allow(clippy::doc_markdown)]

use crate::core::{Error, Result, TriggerMatch};
use crate::usb::{ControlRequest, Transport};
use std::time::Duration;

/// DreamSourceLab's vendor id.
pub const VID: u16 = 0x2a0e;

/// A device of the driver's table (`supported_device[]` of `api.c`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// Product id, before and after the firmware upload.
    pub pid: u16,
    /// The model name, which also selects the bitstream.
    pub model: &'static str,
    /// The FX2 firmware file.
    pub firmware: &'static str,
    /// Memory depth in samples per channel (`mem_depth`, "bits").
    pub mem_depth: u64,
}

/// The supported devices, in the order of the C driver's table.
pub const PROFILES: [Profile; 5] = [
    Profile {
        pid: 0x0001,
        model: "DSLogic",
        firmware: "dreamsourcelab-dslogic-fx2.fw",
        mem_depth: 256 * 1024 * 1024,
    },
    Profile {
        pid: 0x0002,
        model: "DSCope",
        firmware: "dreamsourcelab-dscope-fx2.fw",
        mem_depth: 256 * 1024 * 1024,
    },
    Profile {
        pid: 0x0003,
        model: "DSLogic Pro",
        firmware: "dreamsourcelab-dslogic-pro-fx2.fw",
        mem_depth: 256 * 1024 * 1024,
    },
    Profile {
        pid: 0x0020,
        model: "DSLogic Plus",
        firmware: "dreamsourcelab-dslogic-plus-fx2.fw",
        mem_depth: 256 * 1024 * 1024,
    },
    Profile {
        pid: 0x0021,
        model: "DSLogic Basic",
        firmware: "dreamsourcelab-dslogic-basic-fx2.fw",
        mem_depth: 256 * 1024,
    },
];

impl Profile {
    /// The profile of a product id.
    pub fn of(pid: u16) -> Option<&'static Profile> {
        PROFILES.iter().find(|p| p.pid == pid)
    }

    /// Whether the threshold is chosen by the bitstream (the first
    /// DSLogic) rather than written to the threshold register.
    pub fn threshold_by_bitstream(&self) -> bool {
        self.model == "DSLogic"
    }

    /// The FPGA bitstream for `threshold`, as
    /// `dslogic_fpga_firmware_upload()` picks it: the first DSLogic has
    /// the 3.3 V one below 1.4 V and the 5 V one from there.
    pub fn bitstream(&self, threshold: f64) -> &'static str {
        match self.model {
            "DSLogic" if threshold < 1.40 => "dreamsourcelab-dslogic-fpga-3v3.fw",
            "DSLogic" => "dreamsourcelab-dslogic-fpga-5v.fw",
            "DSLogic Pro" => "dreamsourcelab-dslogic-pro-fpga.fw",
            "DSLogic Plus" => "dreamsourcelab-dslogic-plus-fpga.fw",
            "DSLogic Basic" => "dreamsourcelab-dslogic-basic-fpga.fw",
            _ => "dreamsourcelab-dscope-fpga.fw",
        }
    }
}

/// The sample rates; 200 and 400 MHz are the FPGA's half and quarter
/// modes.
pub const SAMPLERATES: [u64; 16] = [
    10_000,
    20_000,
    50_000,
    100_000,
    200_000,
    500_000,
    1_000_000,
    2_000_000,
    5_000_000,
    10_000_000,
    20_000_000,
    25_000_000,
    50_000_000,
    100_000_000,
    200_000_000,
    400_000_000,
];

/// The first DSLogic's threshold ranges, one per bitstream.
pub const THRESHOLDS: [(f64, f64); 2] = [(0.7, 1.4), (1.4, 3.6)];

/// The threshold every open sets when none was chosen (`thresholds[1][0]`).
pub const DEFAULT_THRESHOLD: f64 = 1.4;

/// `std_gvar_min_max_step_thresholds(0.0, 5.0, 0.1)`: the other models'
/// threshold list, 0 V to 5 V in steps of 0.1 V, each a point range. The
/// steps accumulate as in the C code, so 1.4 is `1.4000000000000001`
/// and `--show` marks no entry current, as sigrok-cli does.
pub fn threshold_steps() -> Vec<(f64, f64)> {
    let (min, max, step) = (0.0f64, 5.0f64, 0.1f64);
    let mut out = Vec::new();
    let mut d = min;
    while d <= max + step / 2.0 {
        let v = if d > -step / 2.0 && d < step / 2.0 { 0.0 } else { d };
        out.push((v, v));
        d += step;
    }
    out
}

/// `std_double_tuple_idx()`: the range of the table within 0.1 V of
/// both bounds.
pub fn threshold_index(low: f64, high: f64) -> Option<usize> {
    THRESHOLDS.iter().position(|&(l, h)| (l - low).abs() < 0.1 && (h - high).abs() < 0.1)
}

/// The two bytes of the threshold register write, `0xb8`: the DAC value
/// `threshold / 5 V * 255` (truncated, as the C conversion does), then
/// the register's address. `None` outside 0 to 5 V, where the C
/// conversion to `uint8_t` is undefined.
pub fn threshold_command(threshold: f64) -> Option<[u8; 2]> {
    if !(0.0..=5.0).contains(&threshold) {
        return None;
    }
    let value = (threshold / 5.0 * 255.0) as u8;
    Some([value, ADDR_VTH])
}

const CMD_GET_FW_VERSION: u8 = 0xb0;
const CMD_GET_REVID_VERSION: u8 = 0xb1;
const CMD_START: u8 = 0xb2;
const CMD_CONFIG: u8 = 0xb3;
const CMD_SETTING: u8 = 0xb4;
const CMD_WR_REG: u8 = 0xb8;

const START_FLAGS_STOP: u8 = 1 << 7;
const START_FLAGS_SAMPLE_WIDE: u8 = 1 << 5;
const START_FLAGS_MODE_LA: u8 = 1 << 4;

const ADDR_VTH: u8 = 0x78;

/// `DS_MAX_LOGIC_DEPTH`: deeper captures (per 100 MHz) run length encode.
const MAX_LOGIC_DEPTH: u64 = 16_000_000;
/// `DS_MAX_LOGIC_SAMPLERATE`: the clock the divider divides.
const MAX_LOGIC_SAMPLERATE: u64 = 100_000_000;
/// `DS_MAX_TRIG_PERCENT`.
const MAX_TRIG_PERCENT: u64 = 90;

/// Mode bit: the trigger is enabled.
pub const MODE_TRIG_EN: u16 = 1 << 0;
/// Mode bit: the external clock.
pub const MODE_CLK_TYPE: u16 = 1 << 1;
/// Mode bit: sample on the external clock's falling edge.
pub const MODE_CLK_EDGE: u16 = 1 << 2;
/// Mode bit: run length encoding.
pub const MODE_RLE_MODE: u16 = 1 << 3;
/// Mode bit: 200 MHz.
pub const MODE_HALF_MODE: u16 = 1 << 5;
/// Mode bit: 400 MHz.
pub const MODE_QUAR_MODE: u16 = 1 << 6;
/// Mode bit: stream mode (`continuous`).
pub const MODE_STREAM_MODE: u16 = 1 << 12;

/// Samples per atom: one 64 bit word per channel.
pub const ATOMIC_SAMPLES: u64 = 64;
/// Bytes per channel word of an atom.
pub const ATOMIC_BYTES: usize = 8;

/// The trigger stages of the setting packet.
pub const NUM_TRIGGER_STAGES: usize = 16;

/// `sizeof(struct fpga_config)`.
pub const SETTING_LEN: usize = 360;
/// `sizeof(struct dslogic_trigger_pos)`.
pub const TRIGGER_POS_LEN: usize = 512;

/// The firmware's timeout for every request and the bitstream.
pub const USB_TIMEOUT: Duration = Duration::from_millis(3000);
/// `FW_BUFSIZE`: the bitstream goes out in pieces of this size, so in
/// one transfer.
const FW_BUFSIZE: usize = 1024 * 1024;
/// `FPGA_UPLOAD_DELAY`: the FX2's time to get ready for the bitstream.
const FPGA_UPLOAD_DELAY: Duration = Duration::from_millis(10);

/// What a capture asks of the FPGA.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Capture {
    /// The sample rate, one of [`SAMPLERATES`].
    pub samplerate: u64,
    /// Samples to take, 0 for no limit.
    pub limit_samples: u64,
    /// Percent of the samples before the trigger.
    pub capture_ratio: u64,
    /// Stream mode.
    pub continuous: bool,
    /// The external clock.
    pub external_clock: bool,
    /// Sample on the external clock's falling edge.
    pub falling_edge: bool,
    /// The enabled channels, bit `n` for channel index `n`.
    pub channels: u16,
    /// The trigger's stages, each its matches on enabled channels, as
    /// `set_trigger()` takes them; `None` without a trigger.
    pub trigger: Option<Vec<Vec<(usize, TriggerMatch)>>>,
}

/// `struct fpga_config`, the setting packet, before it is laid out.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FpgaConfig {
    /// The mode bits.
    pub mode: u16,
    /// 100 MHz divided by this is the rate (1 at 200 and 400 MHz).
    pub divider: u32,
    /// Samples / 16.
    pub count: u32,
    /// Samples kept before the trigger, a multiple of 64.
    pub trig_pos: u32,
    /// Enabled channels << 4 | stages - 1; 0 without a trigger.
    pub trig_glb: u16,
    /// The enabled channels.
    pub ch_en: u16,
    /// The stages' masks (a clear bit takes part).
    pub trig_mask: [u16; NUM_TRIGGER_STAGES],
    /// The stages' values.
    pub trig_value: [u16; NUM_TRIGGER_STAGES],
    /// The stages' edges.
    pub trig_edge: [u16; NUM_TRIGGER_STAGES],
}

/// `trigger_point` of `set_trigger()`: the capture ratio's share of
/// the samples, at least one atom, at most 90 % of the memory (10 % in
/// stream mode), rounded down to whole atoms. The C code computes the
/// 90 % bound in 32 bits, which wraps on the boards with 256 Mibit of
/// memory and gives about 10 % (`protocol.c:374-376`); this does not.
pub fn trigger_position(capture: &Capture, mem_depth: u64) -> u32 {
    let point = (capture.capture_ratio * capture.limit_samples / 100).max(ATOMIC_SAMPLES);
    let max =
        if capture.continuous { mem_depth * 10 / 100 } else { mem_depth * MAX_TRIG_PERCENT / 100 };
    let point = point.min(max) & !(ATOMIC_SAMPLES - 1);
    u32::try_from(point).unwrap_or(!(ATOMIC_SAMPLES as u32 - 1))
}

impl FpgaConfig {
    /// `set_trigger()` and the mode, divider and count of
    /// `fpga_configure()`.
    pub fn new(capture: &Capture, mem_depth: u64) -> FpgaConfig {
        let mut cfg = FpgaConfig {
            mode: 0,
            divider: 0,
            count: 0,
            trig_pos: trigger_position(capture, mem_depth),
            trig_glb: 0,
            ch_en: capture.channels,
            trig_mask: [0xffff; NUM_TRIGGER_STAGES],
            trig_value: [0; NUM_TRIGGER_STAGES],
            trig_edge: [0; NUM_TRIGGER_STAGES],
        };
        if let Some(stages) = &capture.trigger {
            // Every match goes to stage 0, whatever its stage, as in the
            // C code; the stage count is in the global word.
            for stage in stages {
                for &(channel, m) in stage {
                    if capture.channels & (1 << channel) == 0 {
                        continue;
                    }
                    let bit = 1u16 << channel;
                    match m {
                        TriggerMatch::One => {
                            cfg.trig_mask[0] &= !bit;
                            cfg.trig_value[0] |= bit;
                        }
                        TriggerMatch::Zero => cfg.trig_mask[0] &= !bit,
                        TriggerMatch::Falling => {
                            cfg.trig_mask[0] &= !bit;
                            cfg.trig_edge[0] |= bit;
                        }
                        TriggerMatch::Rising => {
                            cfg.trig_mask[0] &= !bit;
                            cfg.trig_value[0] |= bit;
                            cfg.trig_edge[0] |= bit;
                        }
                        TriggerMatch::Edge => cfg.trig_edge[0] |= bit,
                        _ => {}
                    }
                }
            }
            let enabled = capture.channels.count_ones() as u16;
            cfg.trig_glb = (enabled << 4) | (stages.len() as u16).wrapping_sub(1);
            if !stages.is_empty() {
                cfg.mode |= MODE_TRIG_EN;
            }
        }
        match capture.samplerate {
            r if r == MAX_LOGIC_SAMPLERATE * 2 => cfg.mode |= MODE_HALF_MODE,
            r if r == MAX_LOGIC_SAMPLERATE * 4 => cfg.mode |= MODE_QUAR_MODE,
            _ => {}
        }
        if capture.continuous {
            cfg.mode |= MODE_STREAM_MODE;
        }
        if capture.external_clock {
            cfg.mode |= MODE_CLK_TYPE;
            if capture.falling_edge {
                cfg.mode |= MODE_CLK_EDGE;
            }
        }
        // Long captures run length encoded, "without this, captured data
        // present errors".
        let depth = MAX_LOGIC_DEPTH * capture.samplerate.div_ceil(MAX_LOGIC_SAMPLERATE);
        if capture.limit_samples > depth && !capture.continuous {
            cfg.mode |= MODE_RLE_MODE;
        }
        cfg.divider =
            u32::try_from(MAX_LOGIC_SAMPLERATE.div_ceil(capture.samplerate.max(1))).unwrap_or(0);
        cfg.count = u32::try_from(capture.limit_samples / 16).unwrap_or(u32::MAX);
        cfg
    }

    /// The packed little endian `struct fpga_config`. The C code leaves
    /// the global trigger word uninitialised without a trigger; it is 0
    /// here.
    pub fn bytes(&self) -> [u8; SETTING_LEN] {
        let mut b = Vec::with_capacity(SETTING_LEN);
        let header = |variable: u16, words: u16| (variable << 8) | words;
        b.extend_from_slice(&0xf5a5_f5a5u32.to_le_bytes());
        b.extend_from_slice(&header(0, 1).to_le_bytes());
        b.extend_from_slice(&self.mode.to_le_bytes());
        b.extend_from_slice(&header(1, 2).to_le_bytes());
        b.extend_from_slice(&self.divider.to_le_bytes());
        b.extend_from_slice(&header(3, 2).to_le_bytes());
        b.extend_from_slice(&self.count.to_le_bytes());
        b.extend_from_slice(&header(5, 2).to_le_bytes());
        b.extend_from_slice(&self.trig_pos.to_le_bytes());
        b.extend_from_slice(&header(7, 1).to_le_bytes());
        b.extend_from_slice(&self.trig_glb.to_le_bytes());
        b.extend_from_slice(&header(8, 1).to_le_bytes());
        b.extend_from_slice(&self.ch_en.to_le_bytes());
        b.extend_from_slice(&header(64, 160).to_le_bytes());
        // Each array twice (trig_mask0 and trig_mask1 and so on), the
        // logic words 2 and the counts 0.
        for array in [&self.trig_mask, &self.trig_value, &self.trig_edge] {
            for _ in 0..2 {
                for w in array {
                    b.extend_from_slice(&w.to_le_bytes());
                }
            }
        }
        for _ in 0..2 * NUM_TRIGGER_STAGES {
            b.extend_from_slice(&2u16.to_le_bytes());
        }
        b.extend_from_slice(&[0; 4 * NUM_TRIGGER_STAGES]);
        b.extend_from_slice(&0xfa5a_fa5au32.to_le_bytes());
        b.try_into().expect("struct fpga_config is 360 bytes")
    }
}

/// The bytes a millisecond of the capture fills (`to_bytes_per_ms()`):
/// in stream mode the rate times the channels; buffered, where the
/// transfer rate matters less, 10 % of high speed USB.
fn bytes_per_ms(samplerate: u64, channels: u32, continuous: bool) -> u64 {
    if continuous {
        samplerate * u64::from(channels) / (1000 * 8)
    } else {
        35_000_000 / (1000 * 10)
    }
}

/// Most sample transfers in flight (`NUM_SIMUL_TRANSFERS`).
pub const MAX_TRANSFERS: usize = 32;

/// Transfer size, queue depth and transfer timeout of a capture, as
/// `get_buffer_size()`, `get_number_of_transfers()` and `get_timeout()`
/// compute them: 10 ms of data rounded up to 512 bytes per channel,
/// 100 ms in flight but at most 32 transfers, and the time those take
/// plus a quarter. `None` when the stream fills less than a byte a
/// millisecond (10 kHz on fewer than one channel), where the C code
/// divides by zero.
pub fn transfer_plan(
    samplerate: u64,
    channels: u32,
    continuous: bool,
) -> Option<(usize, usize, Duration)> {
    let bpm = bytes_per_ms(samplerate, channels, continuous);
    let block = u64::from(channels) * 512;
    if bpm == 0 || block == 0 {
        return None;
    }
    let size = (10 * bpm).div_ceil(block) * block;
    let depth = (100 * bpm).div_ceil(size).min(MAX_TRANSFERS as u64);
    let ms = size * depth / bpm;
    Some((
        usize::try_from(size).ok()?,
        usize::try_from(depth).ok()?,
        Duration::from_millis(ms + ms / 4),
    ))
}

/// `deinterleave_buffer()` with the counts of `receive_transfer()`: the
/// samples in `src` (one 64 bit word per enabled channel of `mask` for
/// each 64 samples) as 16 bit samples, channel `n` in bit `n`, appended
/// to `out`; returns how many, `64 * len / (8 * channels)` as the C
/// code counts them. A transfer cut inside an atom gives the samples
/// its bytes reach; the C code reads the rest of the atom from whatever
/// the buffer held before, here those bits are 0.
pub fn deinterleave(src: &[u8], mask: u16, out: &mut Vec<u8>) -> usize {
    let channels: Vec<u16> = (0..16).filter(|c| mask & (1 << c) != 0).collect();
    if channels.is_empty() {
        return 0;
    }
    let atom = ATOMIC_BYTES * channels.len();
    let count = 64 * src.len() / atom;
    let word = |at: usize| -> u64 {
        let mut b = [0u8; 8];
        let end = src.len().min(at + 8);
        if at < end {
            b[..end - at].copy_from_slice(&src[at..end]);
        }
        u64::from_le_bytes(b)
    };
    for start in (0..src.len()).step_by(atom) {
        let words: Vec<u64> = (0..channels.len()).map(|k| word(start + 8 * k)).collect();
        let first = start / atom * 64;
        for bit in 0..64 {
            if first + bit >= count {
                return count;
            }
            let sample = channels
                .iter()
                .zip(&words)
                .filter(|(_, w)| (*w >> bit) & 1 != 0)
                .fold(0u16, |s, (c, _)| s | 1 << c);
            out.extend_from_slice(&sample.to_le_bytes());
        }
    }
    count
}

/// The firmware's requests over an open, claimed transport.
pub struct Link {
    transport: Box<dyn Transport>,
}

fn fail(what: &str, e: &Error) -> Error {
    log::error!("{what}: {}.", libusb_name(e));
    Error::Device(format!("{what}: {e}"))
}

/// The libusb error name the C driver logs for a failed transfer.
pub fn libusb_name(e: &Error) -> &'static str {
    match e {
        Error::Timeout => "LIBUSB_ERROR_TIMEOUT",
        Error::Disconnected => "LIBUSB_ERROR_NO_DEVICE",
        Error::Busy => "LIBUSB_ERROR_BUSY",
        Error::Device(m) if m.contains("stall") => "LIBUSB_ERROR_PIPE",
        _ => "LIBUSB_ERROR_OTHER",
    }
}

impl Link {
    /// A link over an open transport.
    pub fn new(transport: Box<dyn Transport>) -> Self {
        Link { transport }
    }

    /// The transport, for the claim and the sample stream.
    pub fn transport(&mut self) -> &mut dyn Transport {
        self.transport.as_mut()
    }

    fn vendor_out(&mut self, request: u8, data: &[u8]) -> Result<()> {
        self.transport.control_out(ControlRequest::vendor(request, 0, 0), data, USB_TIMEOUT)
    }

    fn vendor_in(&mut self, request: u8, len: usize) -> Result<Vec<u8>> {
        let req = ControlRequest::vendor(request, 0, 0);
        let data = self.transport.control_in(req, len, USB_TIMEOUT)?;
        if data.len() < len {
            return Err(Error::device("short answer"));
        }
        Ok(data)
    }

    /// `command_get_fw_version()`: major and minor.
    pub fn firmware_version(&mut self) -> Result<(u8, u8)> {
        let v = self
            .vendor_in(CMD_GET_FW_VERSION, 2)
            .map_err(|e| fail("Unable to get version info", &e))?;
        Ok((v[0], v[1]))
    }

    /// `command_get_revid_version()`.
    pub fn revid(&mut self) -> Result<u8> {
        let v = self
            .vendor_in(CMD_GET_REVID_VERSION, 1)
            .map_err(|e| fail("Unable to get REVID", &e))?;
        Ok(v[0])
    }

    /// `command_start_acquisition()`: logic analyzer mode, wide samples.
    pub fn start(&mut self) -> Result<()> {
        self.vendor_out(CMD_START, &[START_FLAGS_MODE_LA | START_FLAGS_SAMPLE_WIDE, 0, 0])
            .map_err(|e| fail("Failed to send start command", &e))
    }

    /// `command_stop_acquisition()`.
    pub fn stop(&mut self) -> Result<()> {
        self.vendor_out(CMD_START, &[START_FLAGS_STOP, 0, 0])
            .map_err(|e| fail("Failed to send stop command", &e))
    }

    /// `dslogic_fpga_firmware_upload()`: the bitstream file read first
    /// (a missing file sends nothing), `0xb3`, a 10 ms pause, then the
    /// file in pieces of 1 MiB, each one bulk transfer that must take it
    /// whole.
    pub fn upload_bitstream(&mut self, name: &str) -> Result<()> {
        log::debug!("Uploading FPGA firmware '{name}'.");
        let bitstream = crate::core::firmware::read(
            name,
            "extract it from DreamSourceLab's DSView, see docs/DEVICES.md",
        )?;
        self.vendor_out(CMD_CONFIG, &[0, 0, 0])
            .map_err(|e| fail("Failed to upload FPGA firmware", &e))?;
        std::thread::sleep(FPGA_UPLOAD_DELAY);
        let mut sum = 0;
        for chunk in bitstream.chunks(FW_BUFSIZE) {
            let n = self
                .transport
                .bulk_out(2, chunk, USB_TIMEOUT)
                .map_err(|e| fail("Unable to configure FPGA firmware", &e))?;
            sum += n;
            log::trace!("Uploaded {sum}/{} bytes.", bitstream.len());
            if n != chunk.len() {
                log::error!("Short transfer while uploading FPGA firmware.");
                return Err(Error::device("short transfer while uploading FPGA firmware"));
            }
        }
        log::debug!("FPGA firmware upload done.");
        Ok(())
    }

    /// `dslogic_set_voltage_threshold()`: the register write, refused
    /// outside 0 to 5 V before anything is sent.
    pub fn set_threshold(&mut self, threshold: f64) -> Result<()> {
        let cmd = threshold_command(threshold)
            .ok_or_else(|| Error::Arg(format!("voltage threshold {threshold} V: 0 to 5 V")))?;
        self.transport
            .control_out(ControlRequest::vendor(CMD_WR_REG, 0, 0), &cmd, USB_TIMEOUT)
            .map_err(|e| fail("Unable to set voltage-threshold register", &e))
    }

    /// `fpga_configure()`: `0xb4` with the packet's length in 16 bit
    /// words, then the packet, which the FPGA must take whole.
    pub fn configure(&mut self, cfg: &FpgaConfig) -> Result<()> {
        log::debug!("Configuring FPGA.");
        let words = (SETTING_LEN / 2) as u32;
        let [a, b, c, _] = words.to_le_bytes();
        self.vendor_out(CMD_SETTING, &[a, b, c])
            .map_err(|e| fail("Failed to send FPGA configure command", &e))?;
        let bytes = cfg.bytes();
        match self.transport.bulk_out(2, &bytes, USB_TIMEOUT) {
            Ok(n) if n == SETTING_LEN => Ok(()),
            Ok(_) => {
                // libusb's return was 0, its name LIBUSB_SUCCESS.
                log::error!("Failed to send FPGA configuration: LIBUSB_SUCCESS.");
                Err(Error::device("the FPGA took the setting packet short"))
            }
            Err(e) => Err(fail("Failed to send FPGA configuration", &e)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;
    use std::fmt::Write as _;

    fn capture(samplerate: u64, limit: u64, channels: u16) -> Capture {
        Capture {
            samplerate,
            limit_samples: limit,
            capture_ratio: 0,
            continuous: false,
            external_clock: false,
            falling_edge: false,
            channels,
            trigger: None,
        }
    }

    fn hex(b: &[u8]) -> String {
        b.iter().fold(String::new(), |mut s, x| {
            let _ = write!(s, "{x:02x}");
            s
        })
    }

    #[test]
    fn the_setting_packet_is_laid_out_as_the_c_struct() {
        // tests/emu/dreamsourcelab-dslogic/flashed-plus-1m.tx: 4
        // channels at 1 MHz, 100 samples; the global word 0 (xxxx there).
        let cfg = FpgaConfig::new(&capture(1_000_000, 100, 0x000f), 256 << 20);
        let b = cfg.bytes();
        assert_eq!(
            hex(&b[..36]),
            "a5f5a5f5010000000201640000000203060000000205400000000107000001080f00a040"
        );
        assert_eq!(b[36..68], [0xff; 32]);
        assert_eq!(b[100..164], [0; 64]);
        assert_eq!(hex(&b[228..232]), "02000200");
        assert_eq!(b[292..356], [0; 64]);
        assert_eq!(hex(&b[356..]), "5afa5afa");
    }

    #[test]
    fn modes_dividers_and_counts() {
        let mut c = capture(200_000_000, 128, 0xffff);
        let cfg = FpgaConfig::new(&c, 256 << 20);
        assert_eq!((cfg.mode, cfg.divider, cfg.count), (MODE_HALF_MODE, 1, 8));
        c.samplerate = 400_000_000;
        assert_eq!(FpgaConfig::new(&c, 256 << 20).mode, MODE_QUAR_MODE);
        c.samplerate = 10_000;
        c.external_clock = true;
        c.falling_edge = true;
        let cfg = FpgaConfig::new(&c, 256 << 20);
        assert_eq!((cfg.mode, cfg.divider), (MODE_CLK_TYPE | MODE_CLK_EDGE, 10_000));
        // The falling edge alone sets nothing.
        c.external_clock = false;
        assert_eq!(FpgaConfig::new(&c, 256 << 20).mode, 0);
        // RLE above 16 Msamples per 100 MHz, not in stream mode.
        let mut c = capture(100_000_000, 16_000_016, 1);
        assert_eq!(FpgaConfig::new(&c, 256 << 20).mode, MODE_RLE_MODE);
        c.limit_samples = 16_000_000;
        assert_eq!(FpgaConfig::new(&c, 256 << 20).mode, 0);
        c.limit_samples = 32_000_000;
        c.samplerate = 200_000_000;
        assert_eq!(FpgaConfig::new(&c, 256 << 20).mode, MODE_HALF_MODE);
        c.continuous = true;
        c.limit_samples = 64_000_000;
        assert_eq!(FpgaConfig::new(&c, 256 << 20).mode, MODE_HALF_MODE | MODE_STREAM_MODE);
    }

    #[test]
    fn triggers_go_to_stage_0() {
        let mut c = capture(1_000_000, 100, 0x000f);
        c.trigger = Some(vec![vec![
            (0, TriggerMatch::Zero),
            (1, TriggerMatch::One),
            (2, TriggerMatch::Falling),
            (3, TriggerMatch::Rising),
            // A disabled channel takes no part.
            (5, TriggerMatch::One),
        ]]);
        let cfg = FpgaConfig::new(&c, 256 << 20);
        assert_eq!(cfg.mode, MODE_TRIG_EN);
        assert_eq!(cfg.trig_glb, 0x40);
        assert_eq!(
            (cfg.trig_mask[0], cfg.trig_value[0], cfg.trig_edge[0]),
            (0xfff0, 0x000a, 0x000c)
        );
        assert!(cfg.trig_mask[1..].iter().all(|&m| m == 0xffff));
        c.trigger = Some(vec![vec![(2, TriggerMatch::Edge)]]);
        let cfg = FpgaConfig::new(&c, 256 << 20);
        assert_eq!((cfg.trig_mask[0], cfg.trig_value[0], cfg.trig_edge[0]), (0xffff, 0, 0x0004));
    }

    #[test]
    fn trigger_positions() {
        let mut c = capture(1_000_000, 400, 0xf);
        c.capture_ratio = 50;
        assert_eq!(trigger_position(&c, 256 << 20), 192);
        c.capture_ratio = 0;
        assert_eq!(trigger_position(&c, 256 << 20), 64);
        // The Basic: 90 % of 262144 samples, in whole atoms.
        c.capture_ratio = 95;
        c.limit_samples = 300_000;
        assert_eq!(trigger_position(&c, 256 << 10), 235_904);
        // The 256 Mibit boards: 90 %, where the C code's 32 bit product
        // wraps to about 10 % (26843520).
        c.limit_samples = 30_000_000;
        assert_eq!(trigger_position(&c, 256 << 20), 28_499_968);
        c.limit_samples = 300_000_000;
        assert_eq!(trigger_position(&c, 256 << 20), 241_591_872);
        c.continuous = true;
        assert_eq!(trigger_position(&c, 256 << 20), 26_843_520);
    }

    #[test]
    fn transfer_plans() {
        // Buffered: 3500 bytes a millisecond whatever the rate.
        assert_eq!(
            transfer_plan(1_000_000, 4, false),
            Some((36_864, 10, Duration::from_millis(131)))
        );
        assert_eq!(transfer_plan(20_000_000, 16, false).unwrap().0, 40_960);
        // Stream mode: 10 kHz on one channel is a byte a millisecond.
        assert_eq!(transfer_plan(10_000, 1, true), Some((512, 1, Duration::from_millis(640))));
        assert_eq!(transfer_plan(1_000_000, 16, true).unwrap().0, 24_576);
        assert_eq!(transfer_plan(10_000, 0, true), None);
    }

    #[test]
    fn deinterleaving() {
        // Channels 1 and 3: a word each per 64 samples.
        let mut src = Vec::new();
        src.extend_from_slice(&0x8000_0000_0000_0001u64.to_le_bytes());
        src.extend_from_slice(&0x0000_0000_0000_0003u64.to_le_bytes());
        let mut out = Vec::new();
        assert_eq!(deinterleave(&src, 0b1010, &mut out), 64);
        let s: Vec<u16> = out.as_chunks::<2>().0.iter().map(|b| u16::from_le_bytes(*b)).collect();
        assert_eq!(s[..3], [0b1010, 0b1000, 0]);
        assert_eq!(s[63], 0b0010);
        // One channel, cut 4 bytes into the second atom: 32 more
        // samples, those of the bytes there are.
        let mut src = vec![0xff; 8];
        src.extend_from_slice(&[0x01, 0, 0, 0x80]);
        let mut out = Vec::new();
        assert_eq!(deinterleave(&src, 1, &mut out), 96);
        assert_eq!(out.len(), 192);
        assert_eq!(out[128..130], [1, 0]);
        assert_eq!(out[190..192], [1, 0]);
        assert_eq!(deinterleave(&src, 0, &mut out), 0);
    }

    #[test]
    fn thresholds() {
        let steps = threshold_steps();
        assert_eq!(steps.len(), 51);
        assert_eq!(steps[0], (0.0, 0.0));
        assert_ne!(steps[14].0, 1.4);
        assert!((steps[50].0 - 5.0).abs() < 1e-9);
        assert_eq!(threshold_index(0.7, 1.4), Some(0));
        assert_eq!(threshold_index(1.45, 3.55), Some(1));
        assert_eq!(threshold_index(1.0, 2.0), None);
        assert_eq!(threshold_command(1.4), Some([0x47, 0x78]));
        assert_eq!(threshold_command(3.0), Some([0x99, 0x78]));
        assert_eq!(threshold_command(5.0), Some([0xff, 0x78]));
        assert_eq!(threshold_command(5.5), None);
        assert_eq!(threshold_command(-0.1), None);
        let dslogic = Profile::of(0x0001).unwrap();
        assert_eq!(dslogic.bitstream(0.7), "dreamsourcelab-dslogic-fpga-3v3.fw");
        assert_eq!(dslogic.bitstream(1.4), "dreamsourcelab-dslogic-fpga-5v.fw");
        assert_eq!(Profile::of(0x0002).unwrap().bitstream(0.0), "dreamsourcelab-dscope-fpga.fw");
    }

    #[test]
    fn commands_on_the_wire() {
        let text = "# oxrok usb fixture 1\n\
                    device vid=2a0e pid=0020\n\
                    ctl_in 40 b0 0000 0000 2 -> 0100\n\
                    ctl_in 40 b1 0000 0000 1 -> 01\n\
                    ctl_out 40 b8 0000 0000 4778 -> ok\n\
                    ctl_out 40 b2 0000 0000 800000 -> ok\n\
                    ctl_out 40 b2 0000 0000 300000 -> ok\n";
        let r = Replay::from_text(text).unwrap();
        let check = r.clone();
        let mut link = Link::new(Box::new(r));
        assert_eq!(link.firmware_version().unwrap(), (1, 0));
        assert_eq!(link.revid().unwrap(), 1);
        link.set_threshold(DEFAULT_THRESHOLD).unwrap();
        link.stop().unwrap();
        link.start().unwrap();
        check.finish().unwrap();
    }
}
