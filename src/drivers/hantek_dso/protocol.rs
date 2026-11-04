// Counterpart in sigrok:
//   libsigrok/src/hardware/hantek-dso/protocol.c,
//   libsigrok/src/hardware/hantek-dso/protocol.h,
//   libsigrok/src/hardware/hantek-dso/api.c (the tables, send_chunk(), receive_transfer()),
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2008 Oleg Khudyakov <prcoder@gmail.com>,
//   with commits by Uwe Hermann, Philipp Marek, Alberto Scotta,
//   Gerhard Sittig, Miroslav Sustek, Romain Tartière, Lars-Peter Clausen,
//   Daniel Elstner, Martin Ling, Soeren Apel, Aurelien Jacobs,
//   Daniel Glöckner, Valentin Ochs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the Hantek DSO-2xxx and 5200 scopes, an FX2 with
//! a firmware the driver uploads.
//!
//! Without firmware the board answers as `04b4:PID`; the firmware
//! renumerates it as `04b5:PID` with interface 0, bulk OUT 2 and bulk IN
//! 6 (`0x86`).
//!
//! Every bulk command is preceded by the control request "begin"
//! (`0xb3`, 10 fixed bytes). The commands: 0 filters, 1 trigger and
//! timebase, 2 force trigger, 3 capture start, 4 enable trigger, 5 get
//! channel data, 6 get capture state, 7 voltage; the DSO-2250 has 0x0b
//! channels, 0x0c trigger source, 0x0d record length, 0x0e sample rate
//! and 0x0f trigger position instead of 1. Two more control requests set
//! the offsets (`0xb4`) and the relays (`0xb5`), and `0xa2` reads the
//! calibration from the EEPROM: 2 channels x 9 ranges x (low, high), big
//! endian.
//!
//! The capture state answer holds the state in byte 0 and, in bytes 1,
//! 3, 2 from high to low, the trigger offset, Gray coded. A frame is
//! `framesize` samples of two bytes, CH2 first; it is the device's ring
//! buffer, so what starts at the trigger offset comes first, what lies
//! before it last.

use crate::core::analog::{Mq, MqFlags, Unit};
use crate::core::{AnalogData, Error, Result};
use crate::usb::{ControlRequest, InterfaceDesc, Transport, UsbId, libusb_error_name};
use std::time::Duration;

/// The configuration the upload selects.
pub const CONFIGURATION: u8 = 1;
/// The interface the driver claims.
pub const INTERFACE: u8 = 0;
const EP_IN: u8 = 0x86;
const EP_OUT: u8 = 0x02;

const CTRL_READ_EEPROM: u8 = 0xa2;
const CTRL_BEGINCOMMAND: u8 = 0xb3;
const CTRL_SETOFFSET: u8 = 0xb4;
const CTRL_SETRELAYS: u8 = 0xb5;
const EEPROM_CHANNEL_OFFSETS: u16 = 0x08;
/// 2 channels x 9 ranges x 2 words.
const LEVELS_LEN: usize = 72;
const BEGIN: [u8; 10] = [0x0f, 0x03, 0x03, 0x03, 0x68, 0xac, 0xfe, 0x00, 0x01, 0x00];

const CMD_SET_FILTERS: u8 = 0x0;
const CMD_SET_TRIGGER_SAMPLERATE: u8 = 0x1;
const CMD_FORCE_TRIGGER: u8 = 0x2;
const CMD_CAPTURE_START: u8 = 0x3;
const CMD_ENABLE_TRIGGER: u8 = 0x4;
const CMD_GET_CHANNELDATA: u8 = 0x5;
const CMD_GET_CAPTURESTATE: u8 = 0x6;
const CMD_SET_VOLTAGE: u8 = 0x7;
const CMD_2250_SET_CHANNELS: u8 = 0xb;
const CMD_2250_SET_TRIGGERSOURCE: u8 = 0xc;
const CMD_2250_SET_RECORD_LENGTH: u8 = 0xd;
const CMD_2250_SET_SAMPLERATE: u8 = 0xe;
const CMD_2250_SET_TRIGGERPOS_AND_BUFFER: u8 = 0xf;

/// The device waits for its trigger.
pub const CAPTURE_EMPTY: u8 = 0;
/// The device fills its buffer.
pub const CAPTURE_FILLING: u8 = 1;
/// A frame is ready.
pub const CAPTURE_READY_8BIT: u8 = 2;
/// A frame is ready, on the DSO-2250.
pub const CAPTURE_READY_2250: u8 = 3;
/// A frame of 9 bit samples is ready, which the C driver does not read.
pub const CAPTURE_READY_9BIT: u8 = 7;
/// "Doesn't matter, we'll try again next time" (api.c:903).
pub const CAPTURE_TIMEOUT: u8 = 127;
/// Empty states after which the capture is started again.
pub const MAX_CAPTURE_EMPTY: u32 = 3;

const TIMEOUT_BEGIN: Duration = Duration::from_millis(200);
const TIMEOUT_CMD: Duration = Duration::from_millis(100);
/// One channel data transfer, a single packet.
const TIMEOUT_DATA: Duration = Duration::from_millis(40);

const FRAMESIZE_SMALL: u64 = 10 * 1024;
const PID_2250: u16 = 0x2250;

/// A board of `dev_profiles[]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// The ids after a cold boot.
    pub orig: UsbId,
    /// The ids once the firmware runs.
    pub fw: UsbId,
    /// The vendor's name.
    pub vendor: &'static str,
    /// The model's name.
    pub model: &'static str,
    /// The two frame sizes, in samples.
    pub buffersizes: [u64; 2],
    /// The file the board takes.
    pub firmware: &'static str,
}

const fn profile(pid: u16, model: &'static str, large: u64, firmware: &'static str) -> Profile {
    Profile {
        orig: UsbId { vendor: 0x04b4, product: pid },
        fw: UsbId { vendor: 0x04b5, product: pid },
        vendor: "Hantek",
        model,
        buffersizes: [FRAMESIZE_SMALL, large],
        firmware,
    }
}

/// `dev_profiles[]`.
pub const PROFILES: [Profile; 5] = [
    profile(0x2090, "DSO-2090", 32 * 1024, "hantek-dso-2090.fw"),
    profile(0x2150, "DSO-2150", 32 * 1024, "hantek-dso-2150.fw"),
    profile(PID_2250, "DSO-2250", 512 * 1024, "hantek-dso-2250.fw"),
    profile(0x5200, "DSO-5200", 14 * 1024, "hantek-dso-5200.fw"),
    profile(0x520a, "DSO-5200A", 512 * 1024, "hantek-dso-5200A.fw"),
];

/// `timebases[]`: 10 us to 400 ms.
pub const TIMEBASES: [(u64, u64); 15] = [
    (10, 1_000_000),
    (20, 1_000_000),
    (40, 1_000_000),
    (100, 1_000_000),
    (200, 1_000_000),
    (400, 1_000_000),
    (1, 1000),
    (2, 1000),
    (4, 1000),
    (10, 1000),
    (20, 1000),
    (40, 1000),
    (100, 1000),
    (200, 1000),
    (400, 1000),
];
const TIME_20US: usize = 1;
const TIME_40US: usize = 2;
const TIME_100US: usize = 3;
const TIME_200US: usize = 4;

/// `samplerates[]`.
pub const SAMPLERATES: [u64; 16] = [
    20_000,
    25_000,
    50_000,
    100_000,
    200_000,
    250_000,
    500_000,
    1_000_000,
    2_000_000,
    5_000_000,
    10_000_000,
    20_000_000,
    25_000_000,
    50_000_000,
    100_000_000,
    125_000_000,
];

/// `vdivs[]`: 10 mV to 5 V.
pub const VDIVS: [(u64, u64); 9] = [
    (10, 1000),
    (20, 1000),
    (50, 1000),
    (100, 1000),
    (200, 1000),
    (500, 1000),
    (1, 1),
    (2, 1),
    (5, 1),
];
const VDIV_100MV: usize = 3;
const VDIV_500MV: usize = 5;
const VDIV_1V: usize = 6;

/// `trigger_sources[]`.
pub const TRIGGER_SOURCES: [&str; 3] = ["CH1", "CH2", "EXT"];
/// `trigger_slopes[]`.
pub const TRIGGER_SLOPES: [&str; 2] = ["r", "f"];
/// `coupling[]`.
pub const COUPLINGS: [&str; 3] = ["AC", "DC", "GND"];
const COUPLING_AC: usize = 0;
const COUPLING_DC: usize = 1;

/// What `-c` sets, with the C driver's defaults.
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    /// Frames to capture, 0 for no limit.
    pub limit_frames: u64,
    /// The sample rate, which only the DSO-2250's commands carry.
    pub samplerate: u64,
    /// Index into [`TIMEBASES`].
    pub timebase: usize,
    /// The channels enabled, CH1 and CH2.
    pub ch_enabled: [bool; 2],
    /// Index into [`VDIVS`] per channel.
    pub voltage: [usize; 2],
    /// Index into [`COUPLINGS`] per channel.
    pub coupling: [usize; 2],
    /// The filters per channel.
    pub filter: [bool; 2],
    /// The vertical position per channel, 0 to 1.
    pub voffset: [f32; 2],
    /// The trigger level, 0 to 1.
    pub voffset_trigger: f32,
    /// The frame's samples.
    pub framesize: u64,
    /// The trigger's slope.
    pub slope_negative: bool,
    /// Index into [`TRIGGER_SOURCES`]; none until one is set.
    pub trigger_source: Option<usize>,
    /// The share of the frame before the trigger, in percent.
    pub capture_ratio: u64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            limit_frames: 0,
            samplerate: 10_000,
            timebase: TIME_100US,
            ch_enabled: [true; 2],
            voltage: [VDIV_500MV; 2],
            coupling: [COUPLING_DC; 2],
            filter: [false; 2],
            voffset: [0.5; 2],
            voffset_trigger: 0.5,
            framesize: FRAMESIZE_SMALL,
            slope_negative: false,
            trigger_source: None,
            capture_ratio: 100,
        }
    }
}

/// The calibration: channel, range, (low, high).
pub type Levels = [[[u16; 2]; 9]; 2];

/// `dso_getmps()`: the IN endpoint's packet size, 0 unless interface 0
/// has OUT 2 and IN 6, in that order and alone. (The C code also wants a
/// single configuration, which the transport cannot tell.)
pub fn endpoint_packet_size(interfaces: &[InterfaceDesc]) -> usize {
    let Some(i) = interfaces.first() else { return 0 };
    match i.endpoints.as_slice() {
        [out, input] if out.address & 0x8f == EP_OUT && input.address & 0x8f == EP_IN => {
            usize::from(input.max_packet_size)
        }
        _ => 0,
    }
}

/// The calibration from the EEPROM's bytes, big endian words.
pub fn parse_levels(data: &[u8; LEVELS_LEN]) -> Levels {
    let mut levels = [[[0; 2]; 9]; 2];
    let mut words = data.as_chunks::<2>().0.iter().map(|w| u16::from_be_bytes(*w));
    for word in levels.iter_mut().flatten().flatten() {
        *word = words.next().expect("72 bytes");
    }
    levels
}

/// `CMD_SET_TRIGGER_SAMPLERATE` of all boards but the DSO-2250. A timebase
/// under 40 us with a large frame is refused.
///
/// # Errors
///
/// That timebase.
pub fn trigger_samplerate_cmd(s: &Settings) -> Result<[u8; 12]> {
    const SLOW_SMALL: [u16; 12] = [
        0xffff, 0xfffc, 0xfff7, 0xffe8, 0xffce, 0xff9c, 0xff07, 0xfe0d, 0xfc19, 0xf63d, 0xec79,
        0xd8f1,
    ];
    const SLOW_LARGE: [u16; 12] = [
        0xffff, 0x0000, 0xfffc, 0xfff7, 0xffe8, 0xffce, 0xff9d, 0xff07, 0xfe0d, 0xfc19, 0xf63d,
        0xec79,
    ];
    let mut c = [0; 12];
    c[0] = CMD_SET_TRIGGER_SAMPLERATE;
    c[2] = match s.trigger_source {
        Some(1) => 0,
        Some(0) => 1,
        _ => 2,
    };
    let small = s.framesize == FRAMESIZE_SMALL;
    c[2] |= (if small { 0x01 } else { 0x02 }) << 2;
    let fast: u8 = if small {
        match s.timebase {
            0 => 0,
            TIME_20US => 1,
            TIME_40US => 2,
            TIME_100US => 3,
            _ => 4,
        }
    } else {
        match s.timebase {
            0 | TIME_20US => {
                log::error!("Timebase < 40us only supported with 10K buffer.");
                return Err(Error::arg("Timebase < 40us only supported with 10K buffer."));
            }
            TIME_40US => 0,
            TIME_100US => 2,
            TIME_200US => 3,
            _ => 4,
        }
    };
    c[2] |= fast << 5;
    // Enabled channels: 00 CH1, 01 CH2, 10 both.
    c[3] = (u8::from(s.ch_enabled[1]) << 1).wrapping_add(u8::from(s.ch_enabled[0])).wrapping_sub(1);
    if s.slope_negative {
        c[3] |= 1 << 3;
    }
    let slow = if s.timebase < TIME_100US {
        0
    } else if small {
        SLOW_SMALL[s.timebase - TIME_100US]
    } else {
        SLOW_LARGE[s.timebase - TIME_100US]
    };
    c[4..6].copy_from_slice(&slow.to_le_bytes());
    let pos: u64 = 0x77fff + 0x8000 * s.capture_ratio / 100;
    let pos = pos.to_le_bytes();
    c[6] = pos[0];
    c[7] = pos[1];
    c[10] = pos[2];
    Ok(c)
}

/// `CMD_2250_SET_TRIGGERSOURCE`.
pub fn cmd_2250_source(s: &Settings) -> [u8; 8] {
    let mut c = [0; 8];
    c[0] = CMD_2250_SET_TRIGGERSOURCE;
    c[2] = match s.trigger_source {
        Some(1) => 3,
        Some(0) => 2,
        _ => 0,
    };
    if s.slope_negative {
        c[2] |= 1 << 3;
    }
    c
}

/// `CMD_2250_SET_RECORD_LENGTH`.
pub fn cmd_2250_length(s: &Settings) -> [u8; 4] {
    [CMD_2250_SET_RECORD_LENGTH, 0, if s.framesize == FRAMESIZE_SMALL { 0x01 } else { 0x02 }, 0]
}

/// `CMD_2250_SET_SAMPLERATE`: a down sampler of `100 MHz / rate - 2`, in
/// one's complement; `None` above 100 MHz.
pub fn cmd_2250_rate(s: &Settings) -> Option<[u8; 8]> {
    const BASE: u64 = 100_000_000;
    if s.samplerate > BASE {
        return None;
    }
    let mut c = [0; 8];
    c[0] = CMD_2250_SET_SAMPLERATE;
    let div = BASE / s.samplerate.max(1);
    if div > 1 {
        c[2] |= 2;
        #[allow(clippy::cast_possible_truncation)]
        let code = !((div - 2) as u16);
        log::debug!("Down sampler value: 0x{code:x}.");
        c[4..6].copy_from_slice(&code.to_le_bytes());
    }
    Some(c)
}

/// `CMD_2250_SET_CHANNELS`: 0 CH1, 2 both, 3 CH2.
pub fn cmd_2250_channels(s: &Settings) -> [u8; 4] {
    let code = u8::from(!s.ch_enabled[0]) + u8::from(s.ch_enabled[1]) * 2;
    [CMD_2250_SET_CHANNELS, 0, code, 0]
}

/// `dso2250_set_triggerpos()`.
fn triggerpos_2250(value: u64, long_buffer: bool, dest: &mut [u8]) {
    let min: u64 = if long_buffer { 0 } else { 0x7d7ff };
    let max: u64 = 0x7ffff;
    let tmp = min + (max - min) * value / 100;
    log::debug!("2250 trigger pos: {value:3}% * [0x{min:x},0x{max:x}] == 0x{tmp:x}");
    let b = tmp.to_le_bytes();
    dest[0] = b[0];
    dest[1] = b[1];
    dest[2] = b[2] & 0x7;
}

/// `CMD_2250_SET_TRIGGERPOS_AND_BUFFER`: the ten bytes the C driver sends
/// ("12 bytes according to documentation?", it says).
pub fn cmd_2250_triggerpos(s: &Settings) -> [u8; 10] {
    let mut c = [0; 10];
    c[0] = CMD_2250_SET_TRIGGERPOS_AND_BUFFER;
    let long = s.framesize != FRAMESIZE_SMALL;
    triggerpos_2250(s.capture_ratio, long, &mut c[2..5]);
    triggerpos_2250(100 - s.capture_ratio, long, &mut c[6..9]);
    c
}

/// `CMD_SET_FILTERS`.
pub fn filters_cmd(s: &Settings) -> [u8; 8] {
    let mut c = [0; 8];
    c[0] = CMD_SET_FILTERS;
    c[1] = 0x0f;
    if s.filter[0] {
        c[2] |= 0x80;
    }
    if s.filter[1] {
        c[2] |= 0x40;
    }
    c
}

/// `CMD_SET_VOLTAGE`: each channel's range within its relay group, CH1 in
/// bits 0-1, CH2 in bits 2-3. On the DSO-2250 the C driver's header 0x08
/// shares bit 3 with CH2's; it stays as the C driver has it, since
/// nothing but a model written from the same lines says what the board
/// makes of it.
pub fn voltage_cmd(s: &Settings, pid: u16) -> [u8; 8] {
    let mut c = [0; 8];
    c[0] = CMD_SET_VOLTAGE;
    if pid == PID_2250 {
        c[2] = 0x08;
    } else {
        c[1] = 0x0f;
        c[2] = 0x30;
    }
    #[allow(clippy::cast_possible_truncation)]
    let code = |ch: usize| (s.voltage[ch] % 3) as u8;
    c[2] |= code(0);
    c[2] |= code(1) << 2;
    c
}

/// `CTRL_SETRELAYS`'s bytes.
pub fn relays(s: &Settings) -> [u8; 17] {
    let mut r = [0; 17];
    r[..8].copy_from_slice(&[0x00, 0x04, 0x08, 0x02, 0x20, 0x40, 0x10, 0x01]);
    let mut flip = |i: usize, on: bool| {
        if on {
            r[i] = !r[i];
        }
    };
    flip(1, s.voltage[0] < VDIV_1V);
    flip(2, s.voltage[0] < VDIV_100MV);
    flip(3, s.coupling[0] != COUPLING_AC);
    flip(4, s.voltage[1] < VDIV_1V);
    flip(5, s.voltage[1] < VDIV_100MV);
    flip(6, s.coupling[1] != COUPLING_AC);
    flip(7, s.trigger_source == Some(2));
    r
}

/// `CTRL_SETOFFSET`'s bytes. Zero levels before the EEPROM was read, as
/// in the C driver: a trigger level set before the first capture sends
/// channel offsets of 0.
pub fn voffsets(s: &Settings, levels: &Levels) -> [u8; 17] {
    #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
    let channel = |ch: usize| {
        let [low, high] = levels[ch][s.voltage[ch]];
        let (low, high) = (i32::from(low), i32::from(high));
        // In single precision, then truncated, as C has it.
        ((high - low) as f32 * s.voffset[ch] + low as f32) as i32
    };
    #[allow(clippy::cast_possible_truncation)]
    let trigger = (254.0_f32 * s.voffset_trigger) as i32;
    let mut o = [0; 17];
    for (i, offset) in [channel(0), channel(1), trigger].into_iter().enumerate() {
        let b = offset.to_le_bytes();
        // `(offset >> 8) | 0x20` in a byte.
        o[2 * i] = b[1] | 0x20;
        o[2 * i + 1] = b[0];
    }
    o
}

/// The state and the decoded trigger offset of a capture state answer;
/// each set bit of the offset inverts all bits with a lower value.
pub fn parse_capture_state(answer: &[u8]) -> Option<(u8, u32)> {
    let [state, high, low, mid, ..] = *answer else { return None };
    let mut offset = u32::from(high) << 16 | u32::from(mid) << 8 | u32::from(low);
    let mut bit = 1_u32;
    for _ in 0..24 {
        if offset & bit != 0 {
            offset ^= bit - 1;
        }
        bit <<= 1;
    }
    Some((state, offset))
}

/// `send_chunk()`: one packet per enabled channel of the frame bytes
/// `buf`, CH2 first in each sample. Nothing for no samples: the C driver
/// sends an empty packet, on which `output/analog.c` frees its buffer
/// twice.
pub fn analog_packets(s: &Settings, buf: &[u8]) -> Vec<AnalogData> {
    let mut packets = Vec::with_capacity(2);
    if buf.len() < 2 {
        return packets;
    }
    for ch in 0..2 {
        if !s.ch_enabled[ch] {
            continue;
        }
        let (p, q) = VDIVS[s.voltage[ch]];
        #[allow(clippy::cast_precision_loss)]
        let range = (p as f32 / q as f32) * 8.0;
        let vdivlog = (range / 255.0).log10();
        #[allow(clippy::cast_possible_truncation)]
        let digits = -(vdivlog as i32) + i32::from(vdivlog < 0.0);
        let samples = buf
            .as_chunks::<2>()
            .0
            .iter()
            .map(|b| range / 255.0 * f32::from(b[1 - ch]) - range / 2.0)
            .collect();
        packets.push(AnalogData {
            channels: vec![ch],
            samples,
            mq: Mq::Voltage,
            flags: MqFlags::empty(),
            unit: Unit::Volt,
            digits,
        });
    }
    packets
}

/// `receive_transfer()`'s bookkeeping of one frame: what lies before the
/// trigger offset is kept for the frame's end.
#[derive(Debug)]
pub struct Frame {
    framesize: u64,
    trigger_offset: u64,
    received: u64,
    buffered: Vec<u8>,
}

impl Frame {
    /// A frame of `framesize` samples.
    pub fn new(framesize: u64, trigger_offset: u32) -> Self {
        Frame {
            framesize,
            trigger_offset: u64::from(trigger_offset),
            received: 0,
            buffered: Vec::new(),
        }
    }

    /// Takes one transfer (half a sample at its end is dropped): what of
    /// it goes out at once.
    pub fn push<'a>(&mut self, data: &'a [u8]) -> &'a [u8] {
        let n = (data.len() / 2) as u64;
        let data = &data[..data.len() & !1];
        let out = if self.received >= self.trigger_offset {
            data
        } else if self.received + n < self.trigger_offset {
            self.buffered.extend_from_slice(data);
            &data[data.len()..]
        } else {
            #[allow(clippy::cast_possible_truncation)]
            let pre = (self.trigger_offset - self.received) as usize * 2;
            self.buffered.extend_from_slice(&data[..pre]);
            log::debug!("Reached trigger point, {} samples buffered.", self.buffered.len() / 2);
            &data[pre..]
        };
        self.received += n;
        out
    }

    /// The frame is whole.
    pub fn complete(&self) -> bool {
        self.received >= self.framesize
    }

    /// The frame bytes held back, to go out at its end.
    pub fn buffered(&self) -> &[u8] {
        &self.buffered
    }
}

/// The open device.
pub struct Link {
    t: Box<dyn Transport>,
    pid: u16,
    /// `epin_maxpacketsize`.
    mps: usize,
    /// The calibration, zero until the first capture reads it.
    pub levels: Levels,
}

impl Link {
    /// A link on a claimed device.
    pub fn new(t: Box<dyn Transport>, pid: u16, mps: usize, levels: Levels) -> Self {
        Link { t, pid, mps, levels }
    }

    /// `dso_close()`.
    pub fn close(mut self) {
        let _ = self.t.release_interface(INTERFACE);
    }

    fn vendor_out(&mut self, request: u8, data: &[u8], timeout: Duration) -> Result<()> {
        self.t.control_out(ControlRequest::vendor(request, 0, 0), data, timeout)
    }

    /// `send_begin()`.
    fn begin(&mut self) -> Result<()> {
        log::debug!("Sending CTRL_BEGINCOMMAND.");
        self.vendor_out(CTRL_BEGINCOMMAND, &BEGIN, TIMEOUT_BEGIN).inspect_err(|e| {
            log::error!("Failed to send begincommand: {}.", libusb_error_name(e));
        })
    }

    /// `send_bulkcmd()`, which logs nothing of its own for the command.
    fn bulkcmd(&mut self, cmd: &[u8]) -> Result<()> {
        self.begin()?;
        self.t.bulk_out(EP_OUT, cmd, TIMEOUT_BEGIN)?;
        Ok(())
    }

    /// The begin and a command with its message.
    fn command(&mut self, cmd: &[u8], what: &str) -> Result<()> {
        self.begin()?;
        self.t.bulk_out(EP_OUT, cmd, TIMEOUT_CMD).inspect_err(|e| {
            log::error!("Failed to {what}: {}.", libusb_error_name(e));
        })?;
        Ok(())
    }

    /// `get_channel_offsets()`.
    fn read_levels(&mut self) -> Result<()> {
        log::debug!("Getting channel offsets.");
        let req = ControlRequest::vendor(CTRL_READ_EEPROM, EEPROM_CHANNEL_OFFSETS, 0);
        let data = self.t.control_in(req, LEVELS_LEN, TIMEOUT_BEGIN).inspect_err(|e| {
            log::error!("Failed to get channel offsets: {}.", libusb_error_name(e));
        })?;
        let Ok(bytes) = <&[u8; LEVELS_LEN]>::try_from(data.as_slice()) else {
            log::error!("Failed to get channel offsets: {} bytes.", data.len());
            return Err(Error::device("short calibration"));
        };
        self.levels = parse_levels(bytes);
        for (ch, levels) in self.levels.iter().enumerate() {
            let list: Vec<String> =
                levels.iter().map(|[l, h]| format!(" {l:04x}-{h:04x}")).collect();
            log::debug!("CH{}:{}", ch + 1, list.concat());
        }
        Ok(())
    }

    /// `dso_set_trigger_samplerate()`.
    ///
    /// # Errors
    ///
    /// The settings refused, or a failed transfer.
    pub fn set_trigger_samplerate(&mut self, s: &Settings) -> Result<()> {
        if self.pid == PID_2250 {
            return self.set_trigger_samplerate_2250(s);
        }
        log::debug!("Preparing CMD_SET_TRIGGER_SAMPLERATE.");
        let cmd = trigger_samplerate_cmd(s)?;
        self.command(&cmd, "set trigger/samplerate")?;
        log::debug!("Sent CMD_SET_TRIGGER_SAMPLERATE.");
        Ok(())
    }

    /// `dso2250_set_trigger_samplerate()`.
    fn set_trigger_samplerate_2250(&mut self, s: &Settings) -> Result<()> {
        self.command(&cmd_2250_source(s), "set trigger/samplerate")?;
        self.command(&cmd_2250_length(s), "set record length")?;
        let Some(rate) = cmd_2250_rate(s) else {
            log::error!("Sample rate > 100MHz not yet supported.");
            return Err(Error::arg("Sample rate > 100MHz not yet supported."));
        };
        self.command(&rate, "set sample rate")?;
        log::debug!("Sent CMD_2250_SET_SAMPLERATE.");
        self.command(&cmd_2250_channels(s), "set channels")?;
        log::debug!("Sent CMD_2250_SET_CHANNELS.");
        self.command(&cmd_2250_triggerpos(s), "set trigger position")
    }

    /// `dso_set_voffsets()`.
    ///
    /// # Errors
    ///
    /// A failed transfer.
    pub fn set_voffsets(&mut self, s: &Settings) -> Result<()> {
        log::debug!("Preparing CTRL_SETOFFSET.");
        let offsets = voffsets(s, &self.levels);
        self.vendor_out(CTRL_SETOFFSET, &offsets, TIMEOUT_CMD).inspect_err(|e| {
            log::error!("Failed to set offsets: {}.", libusb_error_name(e));
        })?;
        log::debug!("Sent CTRL_SETOFFSET.");
        Ok(())
    }

    /// `dso_enable_trigger()`.
    ///
    /// # Errors
    ///
    /// A failed transfer.
    pub fn enable_trigger(&mut self) -> Result<()> {
        log::debug!("Sending CMD_ENABLE_TRIGGER.");
        self.command(&[CMD_ENABLE_TRIGGER, 0], "enable trigger")
    }

    /// `dso_force_trigger()`.
    ///
    /// # Errors
    ///
    /// A failed transfer.
    pub fn force_trigger(&mut self) -> Result<()> {
        log::debug!("Sending CMD_FORCE_TRIGGER.");
        self.command(&[CMD_FORCE_TRIGGER, 0], "force trigger")
    }

    /// `dso_init()`.
    ///
    /// # Errors
    ///
    /// The settings refused, or a failed transfer.
    pub fn init(&mut self, s: &Settings) -> Result<()> {
        log::debug!("Initializing DSO.");
        self.read_levels()?;
        self.set_trigger_samplerate(s)?;
        log::debug!("Preparing CMD_SET_FILTERS.");
        self.command(&filters_cmd(s), "set filters")?;
        log::debug!("Preparing CMD_SET_VOLTAGE.");
        self.command(&voltage_cmd(s, self.pid), "set voltage")?;
        log::debug!("Preparing CTRL_SETRELAYS.");
        let r = relays(s);
        log::debug!("Relays: {r:02x?}");
        self.vendor_out(CTRL_SETRELAYS, &r, TIMEOUT_CMD).inspect_err(|e| {
            log::error!("Failed to set relays: {}.", libusb_error_name(e));
        })?;
        self.set_voffsets(s)?;
        self.enable_trigger()
    }

    /// `dso_get_capturestate()`: the state and the trigger offset. An
    /// answer too short to hold them is a failed poll (the C code reads
    /// its uninitialised buffer then, protocol.c:832, 847-854).
    ///
    /// # Errors
    ///
    /// A failed transfer, or that short answer.
    pub fn capture_state(&mut self) -> Result<(u8, u32)> {
        log::debug!("Sending CMD_GET_CAPTURESTATE.");
        self.bulkcmd(&[CMD_GET_CAPTURESTATE, 0]).inspect_err(|e| {
            log::debug!("Failed to send get_capturestate command: {}.", libusb_error_name(e));
        })?;
        let answer = self.t.bulk_in(EP_IN, 512, TIMEOUT_CMD).inspect_err(|e| {
            log::debug!("Failed to get capturestate: {}.", libusb_error_name(e));
        })?;
        parse_capture_state(&answer).ok_or_else(|| {
            log::debug!("Capture state answer of {} bytes.", answer.len());
            Error::device("short capture state")
        })
    }

    /// `dso_capture_start()`.
    ///
    /// # Errors
    ///
    /// A failed transfer.
    pub fn capture_start(&mut self) -> Result<()> {
        log::debug!("Sending CMD_CAPTURE_START.");
        self.bulkcmd(&[CMD_CAPTURE_START, 0]).inspect_err(|e| {
            log::error!("Failed to send capture_start command: {}.", libusb_error_name(e));
        })
    }

    /// The request of `dso_get_channeldata()`.
    ///
    /// # Errors
    ///
    /// A failed transfer.
    pub fn request_channel_data(&mut self) -> Result<()> {
        log::debug!("Sending CMD_GET_CHANNELDATA.");
        self.bulkcmd(&[CMD_GET_CHANNELDATA, 0]).inspect_err(|e| {
            log::error!("Failed to get channel data: {}.", libusb_error_name(e));
        })
    }

    /// One channel data transfer, a single packet: it holds all of what
    /// it got when it completes, so a timeout loses nothing.
    ///
    /// # Errors
    ///
    /// A failed transfer, a timeout among them.
    pub fn read_channel_data(&mut self) -> Result<Vec<u8>> {
        self.t.bulk_in(EP_IN, self.mps, TIMEOUT_DATA)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::EndpointDesc;

    fn ep(address: u8, size: u16) -> EndpointDesc {
        EndpointDesc { address, transfer_type: 2, max_packet_size: size }
    }

    fn iface(endpoints: Vec<EndpointDesc>) -> Vec<InterfaceDesc> {
        vec![InterfaceDesc { endpoints, ..InterfaceDesc::default() }]
    }

    #[test]
    fn endpoints_as_dso_getmps_wants_them() {
        assert_eq!(endpoint_packet_size(&iface(vec![ep(0x02, 512), ep(0x86, 512)])), 512);
        assert_eq!(endpoint_packet_size(&iface(vec![ep(0x02, 64), ep(0x86, 64)])), 64);
        assert_eq!(endpoint_packet_size(&iface(vec![ep(0x86, 512), ep(0x02, 512)])), 0);
        assert_eq!(endpoint_packet_size(&iface(vec![ep(0x02, 512), ep(0x82, 512)])), 0);
        assert_eq!(endpoint_packet_size(&iface(vec![ep(0x02, 512)])), 0);
        assert_eq!(endpoint_packet_size(&[]), 0);
    }

    /// The bytes of the goldens' `2090 one frame`.
    #[test]
    fn default_commands_are_the_c_drivers() {
        let s = Settings::default();
        assert_eq!(
            trigger_samplerate_cmd(&s).unwrap(),
            [0x01, 0x00, 0x66, 0x02, 0xff, 0xff, 0xff, 0xff, 0, 0, 0x07, 0]
        );
        assert_eq!(filters_cmd(&s), [0, 0x0f, 0, 0, 0, 0, 0, 0]);
        assert_eq!(voltage_cmd(&s, 0x2090), [7, 0x0f, 0x3a, 0, 0, 0, 0, 0]);
        assert_eq!(voltage_cmd(&s, PID_2250), [7, 0, 0x0a, 0, 0, 0, 0, 0]);
        assert_eq!(
            relays(&s),
            [0, 0xfb, 0x08, 0xfd, 0xdf, 0x40, 0xef, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        );
    }

    #[test]
    fn the_timebase_in_both_frame_sizes() {
        let mut s = Settings { timebase: 0, ..Settings::default() };
        let c = trigger_samplerate_cmd(&s).unwrap();
        assert_eq!(c[2] >> 5, 0);
        assert_eq!(c[4..6], [0, 0]);
        for (t, fast) in [(1, 1), (2, 2), (3, 3), (4, 4), (14, 4)] {
            s.timebase = t;
            assert_eq!(trigger_samplerate_cmd(&s).unwrap()[2] >> 5, fast);
        }
        assert_eq!(trigger_samplerate_cmd(&s).unwrap()[4..6], 0xd8f1_u16.to_le_bytes());
        s.framesize = 32 * 1024;
        for t in [0, 1] {
            s.timebase = t;
            assert!(trigger_samplerate_cmd(&s).is_err());
        }
        for (t, fast, slow) in
            [(2, 0, 0_u16), (3, 2, 0xffff), (4, 3, 0), (5, 4, 0xfffc), (14, 4, 0xec79)]
        {
            s.timebase = t;
            let c = trigger_samplerate_cmd(&s).unwrap();
            assert_eq!((c[2] >> 5, u16::from_le_bytes([c[4], c[5]])), (fast, slow), "{t}");
        }
    }

    #[test]
    fn trigger_source_slope_ratio_and_channels() {
        let mut s =
            Settings { trigger_source: Some(1), slope_negative: true, ..Settings::default() };
        s.capture_ratio = 50;
        s.ch_enabled = [true, false];
        let c = trigger_samplerate_cmd(&s).unwrap();
        assert_eq!(c[2] & 3, 0);
        assert_eq!(c[3], 0x08);
        // 0x77fff + 0x4000.
        assert_eq!((c[6], c[7], c[10]), (0xff, 0xbf, 0x07));
        s.trigger_source = Some(0);
        s.ch_enabled = [false, true];
        let c = trigger_samplerate_cmd(&s).unwrap();
        assert_eq!(c[2] & 3, 1);
        assert_eq!(c[3] & 3, 1);
        s.trigger_source = Some(2);
        s.ch_enabled = [false, false];
        let c = trigger_samplerate_cmd(&s).unwrap();
        assert_eq!(c[2] & 3, 2);
        // C's -1 in a byte: sigrok cannot ask for it.
        assert_eq!(c[3], 0xff);
        assert_eq!(relays(&s)[7], !1);
    }

    #[test]
    fn the_2250_commands() {
        let mut s = Settings::default();
        assert_eq!(cmd_2250_source(&s), [0x0c, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(cmd_2250_length(&s), [0x0d, 0, 1, 0]);
        // 100 MHz / 10 kHz = 10000; 9998 = 0x270e; complement 0xd8f1.
        assert_eq!(cmd_2250_rate(&s).unwrap(), [0x0e, 0, 2, 0, 0xf1, 0xd8, 0, 0]);
        assert_eq!(cmd_2250_channels(&s), [0x0b, 0, 2, 0]);
        // The C comment's example: 500 kHz gives ff39.
        s.samplerate = 500_000;
        assert_eq!(cmd_2250_rate(&s).unwrap()[4..6], [0x39, 0xff]);
        s.samplerate = 100_000_000;
        assert_eq!(cmd_2250_rate(&s).unwrap(), [0x0e, 0, 0, 0, 0, 0, 0, 0]);
        s.samplerate = 125_000_000;
        assert_eq!(cmd_2250_rate(&s), None);
        s.trigger_source = Some(1);
        s.slope_negative = true;
        assert_eq!(cmd_2250_source(&s)[2], 3 | 8);
        s.trigger_source = Some(0);
        s.slope_negative = false;
        assert_eq!(cmd_2250_source(&s)[2], 2);
        s.ch_enabled = [false, true];
        assert_eq!(cmd_2250_channels(&s)[2], 3);
        s.ch_enabled = [true, false];
        assert_eq!(cmd_2250_channels(&s)[2], 0);
        s.capture_ratio = 0;
        let small = cmd_2250_triggerpos(&s);
        assert_eq!(small[2..5], [0xff, 0xd7, 0x07]);
        assert_eq!(small[6..9], [0xff, 0xff, 0x07]);
        s.framesize = 512 * 1024;
        s.capture_ratio = 50;
        assert_eq!(cmd_2250_length(&s)[2], 2);
        let large = cmd_2250_triggerpos(&s);
        // 0x7ffff * 50 / 100 = 0x3ffff.
        assert_eq!(large[2..5], [0xff, 0xff, 0x03]);
        assert_eq!(large[2..5], large[6..9]);
    }

    #[test]
    fn voltage_codes_and_relay_groups() {
        let mut s = Settings::default();
        for (v, code, r1, r2) in [
            (0, 0, true, true),
            (1, 1, true, true),
            (2, 2, true, true),
            (3, 0, true, false),
            (5, 2, true, false),
            (6, 0, false, false),
            (8, 2, false, false),
        ] {
            s.voltage = [v, 0];
            assert_eq!(voltage_cmd(&s, 0x2090)[2], 0x30 | code);
            let r = relays(&s);
            assert_eq!((r[1] == !4, r[2] == !8), (r1, r2));
            s.voltage = [0, v];
            assert_eq!(voltage_cmd(&s, 0x2090)[2], 0x30 | code << 2);
            let r = relays(&s);
            assert_eq!((r[4] == !0x20, r[5] == !0x40), (r1, r2));
        }
        s.coupling = [COUPLING_AC, 2];
        let r = relays(&s);
        assert_eq!((r[3], r[6]), (0x02, !0x10));
    }

    #[test]
    fn offsets_from_the_calibration() {
        let mut levels = [[[0; 2]; 9]; 2];
        levels[0][5] = [0x0100, 0x1e00];
        levels[1][5] = [0x0300, 0x1d00];
        let mut s = Settings::default();
        let o = voffsets(&s, &levels);
        // (0x1e00 - 0x0100) / 2 + 0x0100 = 0x0f80.
        assert_eq!(o[..6], [0x2f, 0x80, 0x30, 0x00, 0x20, 0x7f]);
        assert!(o[6..].iter().all(|b| *b == 0));
        // Not read yet: zero.
        s.voffset_trigger = 0.4;
        let o = voffsets(&s, &[[[0; 2]; 9]; 2]);
        // 254 * 0.4f truncated.
        assert_eq!(o[..6], [0x20, 0, 0x20, 0, 0x20, 101]);
    }

    #[test]
    fn levels_are_big_endian() {
        let mut d = [0; 72];
        d[0] = 0x01;
        d[3] = 0x1e;
        d[71] = 7;
        let l = parse_levels(&d);
        assert_eq!(l[0][0], [0x0100, 0x001e]);
        assert_eq!(l[1][8], [0, 7]);
    }

    #[test]
    fn the_capture_state_decodes_gray() {
        let gray = |n: u32| n ^ (n >> 1);
        for n in [0_u32, 1, 2, 1000, 3500, 0x12345, 0xff_ffff] {
            let g = gray(n).to_le_bytes();
            let a = [2, g[2], g[0], g[1]];
            assert_eq!(parse_capture_state(&a), Some((2, n)), "{n}");
        }
        assert_eq!(parse_capture_state(&[2, 0, 0]), None);
    }

    #[test]
    fn samples_scale_to_the_range_with_ch2_first() {
        let s = Settings::default();
        let p = analog_packets(&s, &[0, 255, 255, 0]);
        assert_eq!(p.len(), 2);
        assert_eq!(p[0].channels, [0]);
        assert!((p[0].samples[0] - 2.0).abs() < 1e-5);
        assert!((p[0].samples[1] + 2.0).abs() < 1e-5);
        assert_eq!(p[1].channels, [1]);
        assert!((p[1].samples[0] + 2.0).abs() < 1e-5);
        // log10(4 / 255) = -1.8.
        assert_eq!(p[0].digits, 2);
        let s = Settings { ch_enabled: [false, true], voltage: [5, 8], ..Settings::default() };
        let p = analog_packets(&s, &[255, 0]);
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].channels, [1]);
        assert!((p[0].samples[0] - 20.0).abs() < 1e-4);
        // log10(40 / 255) = -0.8.
        assert_eq!(p[0].digits, 1);
        assert_eq!(analog_packets(&s, &[]), []);
    }

    #[test]
    fn a_frame_goes_out_from_the_trigger_offset() {
        let mut f = Frame::new(4, 3);
        assert_eq!(f.push(&[0, 1, 2, 3]), &[] as &[u8]);
        assert!(!f.complete());
        assert_eq!(f.push(&[4, 5, 6, 7]), &[6, 7]);
        assert!(f.complete());
        assert_eq!(f.buffered(), &[0, 1, 2, 3, 4, 5]);
    }

    #[test]
    fn a_trigger_offset_on_a_transfer_boundary_keeps_the_chunk() {
        let mut f = Frame::new(4, 2);
        assert_eq!(f.push(&[0, 1, 2, 3]), &[] as &[u8]);
        assert_eq!(f.push(&[4, 5, 6, 7]), &[4, 5, 6, 7]);
        assert_eq!(f.buffered(), &[0, 1, 2, 3]);
    }

    #[test]
    fn a_trigger_offset_of_zero_or_past_the_frame() {
        let mut f = Frame::new(2, 0);
        assert_eq!(f.push(&[0, 1, 2, 3]), &[0, 1, 2, 3]);
        assert_eq!(f.buffered(), &[] as &[u8]);
        let mut f = Frame::new(2, 0x00ff_ffff);
        assert_eq!(f.push(&[0, 1, 2, 3]), &[] as &[u8]);
        assert!(f.complete());
        assert_eq!(f.buffered(), &[0, 1, 2, 3]);
    }

    #[test]
    fn half_a_sample_is_dropped() {
        let mut f = Frame::new(4, 0);
        assert_eq!(f.push(&[0, 1, 2]), &[0, 1]);
        assert!(!f.complete());
    }
}
