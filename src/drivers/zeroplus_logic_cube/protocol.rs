// Counterpart in sigrok:
//   libsigrok/src/hardware/zeroplus-logic-cube/api.c (the models, the capture's arithmetic),
//   libsigrok/src/hardware/zeroplus-logic-cube/analyzer.c,
//   libsigrok/src/hardware/zeroplus-logic-cube/analyzer.h,
//   libsigrok/src/hardware/zeroplus-logic-cube/gl_usb.c,
//   libsigrok/src/hardware/zeroplus-logic-cube/protocol.c,
//   libsigrok/src/hardware/zeroplus-logic-cube/protocol.h,
//   Copyright (C) 2010-2012 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2010 Sven Peter <sven@fail0verflow.com>,
//   Copyright (C) 2010 Haxx Enterprises <bushing@gmail.com>,
//   Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Soeren Apel, Gerhard Sittig,
//   Martin Ling, Daniel Elstner, Jan Metzger, Michał Janiszewski,
//   René Pecher, Romain Tartière, Yasushi SHOJI, g-user
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterparts: api.c and protocol.c are
//   GPL-3.0-or-later, as this file is; analyzer.c, analyzer.h and gl_usb.c
//   carry a BSD-style licence, whose notice is below.

//! The ZEROPLUS Logic Cube's wire protocol: `gl_usb.c`, `analyzer.c` and
//! the arithmetic of `protocol.c` and of `dev_acquisition_start()`.
//!
//! A register file behind vendor requests to interface 0: `bRequest`
//! 0x0c with `wValue` 0x83 selects a register (one byte), 0x85 writes it
//! and 0x84 reads it. `bRequest` 0x04 with `wValue` 0x82 and eight bytes
//! (four 0, the size LE32) comes before each bulk IN on endpoint 0x81.
//! Each transfer has 5 s. The sample memory is circular, four bytes per
//! sample whatever the channels; a capture is configured, started and
//! polled until the busy bit clears, its addresses are read, and the
//! memory is read from the now address in 2048 byte packets, the samples
//! that are not wanted thrown away on the host.
//!
//! No ZEROPLUS document, firmware or recording is available: this is the
//! driver's view. `tools/emu/zeroplus-logic-cube.sh` compares it with
//! sigrok-cli through a model built from that same view, so the cases
//! prove the two drivers alike and not that the board behaves so.
//! Commit b9a5614d says `set_triggerbar()`'s offsets match the Windows
//! app, and commit 42ceb777 calls the addresses "still a bit of a
//! mystery"; [`plan`] and [`trigger_bar`] keep the C's arithmetic for
//! that reason, also where a capture then delivers less than its limit.
//!
//! What is not as in C, each with its reason:
//!
//! - A rate outside the list is refused. `zp_set_samplerate()` loops on
//!   `ARRAY_SIZE()`, always true (api.c:148), and reads past the array.
//! - A read of the status or an address that fails is no value: a failed
//!   status ended the wait or not by the bits of the error code
//!   (analyzer.c:411-415; `LIBUSB_ERROR_PIPE` polled for ever,
//!   `LIBUSB_ERROR_NO_DEVICE` read as "not busy"), and a failed address
//!   read made an address of its bits (analyzer.c:639-655).
//! - A short or failed bulk read delivers what it holds and ends the
//!   capture. The C warns and sends its whole buffer, the bytes of the
//!   previous read standing in for those missing (api.c:605-648).
//! - The trigger and its registers are worked out for each capture:
//!   `analyzer_add_triggers()` leaves a former trigger in place when
//!   there is none now (analyzer.c:515-516), and `devc->trigger` is never
//!   cleared (analyzer.c:525).
//! - A limit of 0, which sigrok-cli never starts, takes all of the
//!   memory, not `0 - 1` post-trigger samples (protocol.c:102-120).
//
// analyzer.c, analyzer.h and gl_usb.c carry this notice:
//
// Copyright (C) 2010 Sven Peter <sven@fail0verflow.com>
// Copyright (C) 2010 Haxx Enterprises <bushing@gmail.com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//  THE POSSIBILITY OF SUCH DAMAGE.

use crate::core::{Error, LogicData, Packet, PacketSink, Result, Trigger, TriggerMatch};
use crate::usb::{ControlRequest, Transport, libusb_error_name};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

const TIMEOUT: Duration = Duration::from_secs(5);
const EP1_BULK_IN: u8 = 0x81;
const REQ_READBULK: u16 = 0x82;
const REQ_WRITEADDR: u16 = 0x83;
const REQ_READDATA: u16 = 0x84;
const REQ_WRITEDATA: u16 = 0x85;

/// The size of one bulk read, `PACKET_SIZE` (api.c:27).
pub const PACKET_SIZE: usize = 2048;
/// Bytes per sample in the memory, whatever the channels.
pub const UNIT_SIZE: usize = 4;

/// Failed status polls in a row after which the capture is given up. C
/// has no bound (analyzer.c:411-415); one refused poll must not end a
/// capture, as it does not in C.
pub const MAX_FAILED_POLLS: u32 = 16;

// The registers (analyzer.c:40-109).
const START_STATUS: u8 = 0x20;
const DEV_STATUS: u8 = 0x21;
const FREQUENCY_REG0: u8 = 0x30;
const FREQUENCY_REG1: u8 = 0x31;
const FREQUENCY_REG2: u8 = 0x32;
const FREQUENCY_REG4: u8 = 0x34;
const MEMORY_LENGTH: u8 = 0x35;
const CLOCK_SOURCE: u8 = 0x36;
const TRIGGER_STATUS0: u8 = 0x40;
const TRIGGER_EDGE: u8 = 0x48;
const TRIGGER_COUNT0: u8 = 0x50;
const TRIGGER_COUNT1: u8 = 0x51;
const TRIGGER_LEVEL0: u8 = 0x55;
const RAMSIZE_TRIGGERBAR_ADDRESS0: u8 = 0x60;
const TRIGGERBAR_ADDRESS0: u8 = 0x63;
const DONT_CARE_TRIGGERBAR: u8 = 0x66;
const FILTER_ENABLE: u8 = 0x70;
const FILTER_STATUS: u8 = 0x71;
const ENABLE_DELAY_TIME0: u8 = 0x7a;
const ENABLE_DELAY_TIME1: u8 = 0x7b;
const ENABLE_INSERT_DATA0: u8 = 0x80;
const COMPRESSION_TYPE0: u8 = 0x84;
const COMPRESSION_TYPE1: u8 = 0x85;
const TRIGGER_ADDRESS0: u8 = 0x90;
const NOW_ADDRESS0: u8 = 0x96;
const STOP_ADDRESS0: u8 = 0x9b;
const READ_RAM_STATUS: u8 = 0xa0;

const STATUS_FLAG_NONE: u8 = 0x00;
const STATUS_FLAG_RESET: u8 = 0x01;
const STATUS_FLAG_INIT: u8 = 0x02;
const STATUS_FLAG_GO: u8 = 0x04;
const STATUS_FLAG_READ: u8 = 0x10;
const STATUS_FLAG_20: u8 = 0x20;

const STATUS_BUSY: u8 = 0x01;
const STATUS_READY: u8 = 0x02;

const COMPRESSION_NONE: u16 = 0x0001;

/// `MEMORY_SIZE_8K`, the code of the 8 KiB memory.
pub const MEMORY_SIZE_8K: u8 = 0;
const MEMORY_SIZE_64K: u8 = 1;

/// A board of `zeroplus_models[]` (api.c:44-59).
#[derive(Debug)]
pub struct Profile {
    /// `idProduct`; `idVendor` is [`VENDOR`] on all of them.
    pub pid: u16,
    /// The model's name.
    pub name: &'static str,
    /// 16 or 32.
    pub channels: usize,
    /// Samples per channel, in K.
    pub sample_depth: u64,
    /// The highest rate, in MHz.
    pub max_mhz: u64,
}

/// ZEROPLUS's vendor id.
pub const VENDOR: u16 = 0x0c12;

const fn profile(
    pid: u16,
    name: &'static str,
    channels: usize,
    sample_depth: u64,
    max_mhz: u64,
) -> Profile {
    Profile { pid, name, channels, sample_depth, max_mhz }
}

/// `zeroplus_models[]`.
pub static PROFILES: [Profile; 13] = [
    profile(0x7002, "LAP-16128U", 16, 128, 200),
    profile(0x7007, "LAP-16032U", 16, 32, 200),
    profile(0x7009, "LAP-C(16064)", 16, 64, 100),
    profile(0x700a, "LAP-C(16128)", 16, 128, 200),
    profile(0x700b, "LAP-C(32128)", 32, 128, 200),
    profile(0x700c, "LAP-C(321000)", 32, 1024, 200),
    profile(0x700d, "LAP-C(322000)", 32, 2048, 200),
    profile(0x700e, "LAP-C(16032)", 16, 32, 100),
    profile(0x7016, "LAP-C(162000)", 16, 2048, 200),
    profile(0x7025, "LAP-C(16128+)", 16, 128, 200),
    profile(0x7060, "Logian-16L", 16, 128, 200),
    profile(0x7064, "Logian-16L", 16, 128, 200),
    profile(0x7100, "AKIP-9101", 16, 256, 200),
];

impl Profile {
    /// The board of a product id.
    pub fn of_pid(pid: u16) -> Option<&'static Profile> {
        PROFILES.iter().find(|p| p.pid == pid)
    }

    /// `max_sample_depth`, in samples.
    pub fn max_samples(&self) -> u64 {
        self.sample_depth * 1024
    }

    /// `max_samplerate`, in Hz.
    pub fn max_samplerate(&self) -> u64 {
        self.max_mhz * 1_000_000
    }

    /// The rates `config_list()` gives the board: `samplerates_100[]` or
    /// `samplerates_200[]`.
    pub fn samplerates(&self) -> Vec<u64> {
        SAMPLERATES.iter().copied().filter(|&r| r <= self.max_samplerate()).collect()
    }
}

/// `samplerates_200[]` (api.c:123-142); `samplerates_100[]` is its head.
pub const SAMPLERATES: [u64; 18] = [
    100,
    500,
    1_000,
    5_000,
    25_000,
    50_000,
    100_000,
    200_000,
    400_000,
    800_000,
    1_000_000,
    10_000_000,
    25_000_000,
    50_000_000,
    80_000_000,
    100_000_000,
    150_000_000,
    200_000_000,
];

/// `FREQ_SCALE_*`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scale {
    /// `FREQ_SCALE_HZ`.
    Hz = 0,
    /// `FREQ_SCALE_KHZ`.
    Khz = 1,
    /// `FREQ_SCALE_MHZ`.
    Mhz = 2,
}

/// `zp_set_samplerate()`'s split of a rate into a value and a scale.
pub fn split_rate(rate: u64) -> (u64, Scale) {
    if rate >= 1_000_000 {
        (rate / 1_000_000, Scale::Mhz)
    } else if rate >= 1_000 {
        (rate / 1_000, Scale::Khz)
    } else {
        (rate, Scale::Hz)
    }
}

/// One line of `__analyzer_set_freq()`'s table: selection, divider,
/// multiplier (analyzer.c:298-318). Every listed rate has its line.
pub fn clock_regs(freq: u64, scale: Scale) -> Option<(u8, u8, u8)> {
    Some(match (scale, freq) {
        (Scale::Mhz, 200) => (0, 1, 20),
        (Scale::Mhz, 150) => (0, 1, 15),
        (Scale::Mhz, 100) => (0, 1, 10),
        (Scale::Mhz, 80) => (0, 2, 16),
        (Scale::Mhz, 50) => (0, 2, 10),
        (Scale::Mhz, 25) => (1, 5, 25),
        (Scale::Mhz, 10) => (1, 5, 10),
        (Scale::Mhz, 1) => (16, 5, 5),
        (Scale::Khz, 800) => (17, 5, 8),
        (Scale::Khz, 400) => (32, 5, 20),
        (Scale::Khz, 200) => (32, 5, 10),
        (Scale::Khz, 100) => (32, 5, 5),
        (Scale::Khz, 50) => (33, 5, 5),
        (Scale::Khz, 25) => (49, 5, 25),
        (Scale::Khz, 5) => (50, 5, 10),
        (Scale::Khz, 1) => (64, 5, 5),
        (Scale::Hz, 500) => (64, 10, 5),
        (Scale::Hz, 100) => (68, 5, 8),
        _ => return None,
    })
}

/// `get_memory_size()`: the bytes of a memory length code.
pub fn memory_bytes(code: u8) -> u32 {
    if code == MEMORY_SIZE_8K { 8 * 1024 } else { (32 * 1024) << code }
}

/// `set_limit_samples()`'s memory length code for a limit, already
/// clamped to the board's depth: the smallest of 8, 64 KiB, ... that holds
/// it at four bytes a sample, with 8 KiB for at most 2 K and 64 KiB up to
/// 16 K.
pub fn memory_code(samples: u64) -> u8 {
    if samples <= 2 * 1024 {
        MEMORY_SIZE_8K
    } else if samples <= 16 * 1024 {
        MEMORY_SIZE_64K
    } else {
        // 64 - clz64 is 32 - clz32 in range: `19 - clz(samples - 1)`.
        u8::try_from((64 - (samples - 1).leading_zeros()).saturating_sub(13)).unwrap_or(u8::MAX)
    }
}

/// `set_voltage_threshold()`'s clamp.
pub fn clamp_threshold(volts: f64) -> f64 {
    volts.clamp(-6.0, 6.0)
}

/// `set_voltage_threshold()`'s level register: `round(-9.1 V + 62.6)`.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn threshold_reg(volts: f64) -> u8 {
    (-9.1 * volts + 62.6).round() as u8
}

/// `set_triggerbar()`: the samples before the trigger and after it. The
/// offsets "match USB packet captures from official app/driver"
/// (protocol.c:113).
pub fn trigger_bar(limit: u64, code: u8, trigger: bool, ratio: u64) -> (u32, u32) {
    let memory = u64::from(memory_bytes(code)) / 4;
    let depth = if limit != 0 && limit < memory { limit } else { memory };
    let bar = if trigger { depth * ratio / 100 } else { 0 };
    let ramsize = u32::try_from(depth - bar).unwrap_or(u32::MAX);
    let bar = u32::try_from(bar).unwrap_or(u32::MAX);
    // At a limit of 1 or 2 with a ratio of 100 nothing is left after the
    // trigger and the C's unsigned count wraps to 0xffffffff
    // (protocol.c:113-118). It runs on, and so does this.
    if bar > 2 { (bar - 2, ramsize) } else { (0, ramsize.wrapping_sub(1)) }
}

/// The board's settings, all written by `analyzer_configure()`:
/// `analyzer.c`'s `g_` variables, one set per device.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Regs {
    /// `g_trigger_status`.
    pub trigger_status: [u8; 8],
    /// `g_trigger_edge`.
    pub trigger_edge: u8,
    /// `g_ext_clock`.
    pub ext_clock: bool,
    /// `g_ext_clock_edge` is falling.
    pub ext_falling: bool,
    /// `g_freq_value`.
    pub freq: u64,
    /// `g_freq_scale`.
    pub scale: Scale,
    /// `g_memory_size`.
    pub memory_size: u8,
    /// `g_ramsize_triggerbar_addr`.
    pub ramsize_trigger: u32,
    /// `g_triggerbar_addr`.
    pub triggerbar: u32,
    /// `g_thresh`.
    pub thresh: u8,
}

impl Default for Regs {
    /// Their initial values (analyzer.c:111-126).
    fn default() -> Self {
        Regs {
            trigger_status: [0; 8],
            trigger_edge: 0,
            ext_clock: false,
            ext_falling: false,
            freq: 1,
            scale: Scale::Mhz,
            memory_size: MEMORY_SIZE_8K,
            ramsize_trigger: 2 * 1024,
            triggerbar: 0,
            thresh: 0x31,
        }
    }
}

/// What `analyzer_add_triggers()` makes of a trigger.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TriggerRegs {
    /// `TRIGGER_STATUS0` and on: two bits per channel, 01 high, 10 low.
    pub status: [u8; 8],
    /// `TRIGGER_EDGE`: the top two bits 01 rising, 10 falling, 11 either,
    /// the low five the channel.
    pub edge: u8,
    /// `devc->trigger`.
    pub any: bool,
}

/// `analyzer_add_triggers()`, with `enabled` saying which channels are.
/// The board has one edge register, so a second edge replaces the first
/// (analyzer.c:537-545), and a channel asked to be high and low gets both
/// bits (analyzer.c:532-535). Neither is refused, as nothing says what the
/// board makes of it, but both are warned of, the C being silent.
///
/// # Errors
///
/// A match no register holds (an analog one).
pub fn trigger_regs(trigger: Option<&Trigger>, enabled: &[bool]) -> Result<TriggerRegs> {
    let mut regs = TriggerRegs::default();
    let Some(trigger) = trigger else { return Ok(regs) };
    let mut edge_channel = None;
    for condition in trigger.stages.iter().flat_map(|s| &s.conditions) {
        regs.any = true;
        let channel = condition.channel;
        if !enabled.get(channel).copied().unwrap_or(false) {
            // Ignore disabled channels with a trigger.
            continue;
        }
        let word = channel / 4;
        let pair = channel % 4 * 2;
        let Some(slot) = regs.status.get_mut(word) else {
            return Err(Error::Arg(format!("no channel {channel}")));
        };
        let edge = match condition.matches {
            TriggerMatch::Zero => {
                *slot |= 2 << pair;
                None
            }
            TriggerMatch::One => {
                *slot |= 1 << pair;
                None
            }
            TriggerMatch::Rising => Some(0x40),
            TriggerMatch::Falling => Some(0x80),
            TriggerMatch::Edge => Some(0xc0),
            other => {
                log::error!("Unsupported match {other:?}");
                return Err(Error::not_supported("this trigger match"));
            }
        };
        if *slot >> pair & 3 == 3 {
            log::warn!("Channel {channel} is asked to be high and low: the trigger may not fire.");
        }
        if let Some(kind) = edge {
            if let Some(old) = edge_channel {
                log::warn!("The board has one edge trigger: channel {channel} replaces {old}.");
            }
            edge_channel = Some(channel);
            regs.edge = kind | u8::try_from(channel & 0x1f).unwrap_or(0);
        }
    }
    Ok(regs)
}

/// `gl_usb.c` and `analyzer.c` over a transport.
pub struct Link {
    t: Box<dyn Transport>,
}

impl Link {
    /// A link on an open device.
    pub fn new(t: Box<dyn Transport>) -> Self {
        Link { t }
    }

    /// The transport back, for the close.
    pub fn into_transport(self) -> Box<dyn Transport> {
        self.t
    }

    /// `gl_write_address()`.
    fn write_address(&mut self, reg: u8) -> Result<()> {
        let req = ControlRequest::vendor_interface(0x0c, REQ_WRITEADDR, 0);
        self.t.control_out(req, &[reg], TIMEOUT).inspect_err(|e| {
            log::error!("gl_write_address: {}.", libusb_error_name(e));
        })
    }

    /// `gl_write_data()`.
    fn write_data(&mut self, value: u8) -> Result<()> {
        let req = ControlRequest::vendor_interface(0x0c, REQ_WRITEDATA, 0);
        self.t.control_out(req, &[value], TIMEOUT).inspect_err(|e| {
            log::error!("gl_write_data: {}.", libusb_error_name(e));
        })
    }

    /// `gl_read_data()`.
    fn read_data(&mut self) -> Result<u8> {
        let req = ControlRequest::vendor_interface(0x0c, REQ_READDATA, 0);
        match self.t.control_in(req, 1, TIMEOUT) {
            Ok(v) if v.len() == 1 => Ok(v[0]),
            Ok(_) => {
                log::error!("gl_read_data: LIBUSB_SUCCESS, val=0.");
                Err(Error::device("empty register read"))
            }
            Err(e) => {
                log::error!("gl_read_data: {}, val=0.", libusb_error_name(&e));
                Err(e)
            }
        }
    }

    /// `gl_reg_write()`.
    fn reg_write(&mut self, reg: u8, value: u8) -> Result<()> {
        self.write_address(reg)?;
        self.write_data(value)
    }

    /// `gl_reg_write()` whose outcome no caller looks at (the C ignores
    /// all but those of the rate); its log says what failed.
    fn set(&mut self, reg: u8, value: u8) {
        let _ = self.reg_write(reg, value);
    }

    /// `gl_reg_read()`.
    fn reg_read(&mut self, reg: u8) -> Result<u8> {
        self.write_address(reg)?;
        self.read_data()
    }

    /// `analyzer_write_status()`.
    fn write_status(&mut self, unk: u8, flags: u8) {
        self.set(START_STATUS, unk << 6 | flags);
    }

    /// `analyzer_read_status()`.
    ///
    /// # Errors
    ///
    /// As the transport says; the C takes the error code for a status.
    pub fn read_status(&mut self) -> Result<u8> {
        self.reg_read(DEV_STATUS)
    }

    /// A 24 bit address: the three reads in the order the oracle's build
    /// makes them, high byte first. C leaves that order unspecified
    /// (analyzer.c:641-642).
    fn read_address(&mut self, reg0: u8) -> Result<u32> {
        let high = self.reg_read(reg0 + 2);
        let mid = self.reg_read(reg0 + 1);
        let low = self.reg_read(reg0);
        Ok(u32::from(high?) << 16 | u32::from(mid?) << 8 | u32::from(low?))
    }

    /// `analyzer_reset()`.
    pub fn reset(&mut self) {
        self.write_status(3, STATUS_FLAG_NONE);
        self.write_status(3, STATUS_FLAG_RESET);
    }

    /// `analyzer_initialize()`.
    pub fn initialize(&mut self) {
        self.write_status(1, STATUS_FLAG_NONE);
        self.write_status(1, STATUS_FLAG_INIT);
        self.write_status(1, STATUS_FLAG_NONE);
    }

    /// `analyzer_start()`.
    pub fn start(&mut self) {
        self.initialize();
        self.write_status(1, STATUS_FLAG_GO);
    }

    /// `analyzer_read_start()`, with its `gl_reg_read_buf()` of no bytes.
    pub fn read_start(&mut self) {
        self.write_status(3, STATUS_FLAG_20 | STATUS_FLAG_READ);
        let _ = self.write_address(READ_RAM_STATUS);
    }

    /// `analyzer_read_stop()`.
    pub fn read_stop(&mut self) {
        self.write_status(3, STATUS_FLAG_20);
        self.write_status(3, STATUS_FLAG_NONE);
    }

    /// `gl_read_bulk()`: what arrived of `PACKET_SIZE` bytes. A refused
    /// request is logged and the read made all the same.
    pub fn read_packet(&mut self) -> Vec<u8> {
        let size = u32::try_from(PACKET_SIZE).unwrap_or(0).to_le_bytes();
        let packet = [0, 0, 0, 0, size[0], size[1], size[2], size[3]];
        let req = ControlRequest::vendor_interface(0x04, REQ_READBULK, 0);
        if let Err(e) = self.t.control_out(req, &packet, TIMEOUT) {
            log::error!("gl_read_bulk: libusb_control_transfer: {}.", libusb_error_name(&e));
        }
        match self.t.bulk_in(EP1_BULK_IN, PACKET_SIZE, TIMEOUT) {
            Ok(data) => data,
            Err(e) => {
                log::error!("gl_read_bulk: libusb_bulk_transfer: {}.", libusb_error_name(&e));
                Vec::new()
            }
        }
    }

    /// `__analyzer_set_freq()`: gives up at the first register refused,
    /// and its result is not looked at (analyzer.c:459), so the board may
    /// run on a mixture of rates: the C's habit, and its log says so.
    fn set_freq(&mut self, freq: u64, scale: Scale) -> Result<()> {
        let Some((sel, div, mul)) = clock_regs(freq, scale) else {
            return Err(Error::arg("no such rate"));
        };
        log::debug!(
            "Setting samplerate regs (freq={freq}, scale={}): reg0: {div}, reg1: {mul}, reg2: 2, reg3: {sel}.",
            scale as u8
        );
        self.reg_write(FREQUENCY_REG0, div)?;
        self.reg_write(FREQUENCY_REG1, mul)?;
        self.reg_write(FREQUENCY_REG2, 0x02)?;
        self.reg_write(FREQUENCY_REG4, sel)
    }

    /// The three bytes of `__analyzer_set_ramsize_trigger_address()` and
    /// `__analyzer_set_triggerbar_address()`.
    fn set_24(&mut self, reg0: u8, value: u32) {
        let b = value.to_le_bytes();
        self.set(reg0, b[0]);
        self.set(reg0 + 1, b[1]);
        self.set(reg0 + 2, b[2]);
    }

    /// `analyzer_configure()`. The trigger count (1), the filter (none),
    /// the delay (0), the "insert data" and the compression (none) are
    /// what `dev_open()` and the C's defaults make them.
    pub fn configure(&mut self, regs: &Regs) {
        self.write_status(1, STATUS_FLAG_RESET);
        self.write_status(1, STATUS_FLAG_NONE);
        self.write_status(1, STATUS_FLAG_INIT);
        self.write_status(1, STATUS_FLAG_NONE);
        let _ = self.set_freq(regs.freq, regs.scale);
        self.set(MEMORY_LENGTH, regs.memory_size);
        // analyzer.c:465-472: 1 internal, 2 external rising, 0 falling.
        let source = match (regs.ext_clock, regs.ext_falling) {
            (false, _) => 0x01,
            (true, false) => 0x02,
            (true, true) => 0x00,
        };
        self.set(CLOCK_SOURCE, source);
        for (reg, value) in (TRIGGER_STATUS0..).zip(regs.trigger_status) {
            self.set(reg, value);
        }
        self.set(TRIGGER_EDGE, regs.trigger_edge);
        self.set(TRIGGER_COUNT0, 1);
        self.set(TRIGGER_COUNT1, 0);
        for reg in TRIGGER_LEVEL0..TRIGGER_LEVEL0 + 4 {
            self.set(reg, regs.thresh);
        }
        self.set_24(RAMSIZE_TRIGGERBAR_ADDRESS0, regs.ramsize_trigger);
        self.set_24(TRIGGERBAR_ADDRESS0, regs.triggerbar);
        self.set(DONT_CARE_TRIGGERBAR, 0x01);
        self.set(FILTER_ENABLE, 0);
        for reg in FILTER_STATUS..FILTER_STATUS + 8 {
            self.set(reg, 0);
        }
        self.set(ENABLE_DELAY_TIME0, 0);
        self.set(ENABLE_DELAY_TIME1, 0);
        for (reg, value) in (ENABLE_INSERT_DATA0..).zip([0x12, 0x34, 0x56, 0x78]) {
            self.set(reg, value);
        }
        let compression = COMPRESSION_NONE.to_le_bytes();
        self.set(COMPRESSION_TYPE0, compression[0]);
        self.set(COMPRESSION_TYPE1, compression[1]);
    }
}

/// What came of `dev_acquisition_start()`'s arithmetic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    /// Samples to throw away at the start of the readout.
    pub discard: u32,
    /// Samples to deliver after them.
    pub valid: u32,
    /// The trigger's place among those delivered.
    pub trigger_offset: u32,
}

/// api.c:555-597, in C's unsigned 32 bit arithmetic: `stop - now`
/// underflows at a limit of 3 and the discard then covers the whole
/// readout, as in C. `memory_size` is in samples and a power of two;
/// `wanted` is `ramsize_trigger + triggerbar`.
pub fn plan(
    mut status: u8,
    stop_address: u32,
    mut now_address: u32,
    trigger_address: u32,
    memory_size: u32,
    wanted: u32,
) -> Plan {
    let trigger_now = now_address == trigger_address
        || now_address.wrapping_add(1) % memory_size == trigger_address;
    if now_address == 0 {
        status &= !STATUS_READY;
    }
    let mut discard = 0u32;
    if status & STATUS_READY != 0 {
        // Not wrapped: from here to the end of the memory is old, and the
        // first two samples captured are always bogus.
        discard = discard.wrapping_add(memory_size.wrapping_sub(now_address).wrapping_add(2));
        now_address = 2;
    }
    let valid = stop_address.wrapping_sub(now_address) % memory_size;
    if valid > wanted {
        discard = discard.wrapping_add(valid - wanted);
        now_address = now_address.wrapping_add(valid - wanted);
    }
    let trigger_offset =
        if trigger_now { 0 } else { trigger_address.wrapping_sub(now_address) % memory_size };
    let valid = stop_address.wrapping_sub(now_address) % memory_size;
    Plan { discard, valid, trigger_offset }
}

/// One thing a read gives to send.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Piece {
    /// Samples: this range of the read's bytes.
    Samples(std::ops::Range<usize>),
    /// The trigger: the next sample is its.
    Trigger,
}

/// What to do after one read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Feed {
    /// All of it was thrown away.
    Skip,
    /// The samples wanted are all out: no more reads.
    Done,
    /// These go out.
    Send(Vec<Piece>),
}

/// api.c:601-650, one read at a time.
#[derive(Clone, Debug)]
pub struct Readout {
    plan: Plan,
    samples_read: u32,
}

impl Readout {
    /// A readout to that plan.
    pub fn new(plan: Plan) -> Self {
        Readout { plan, samples_read: 0 }
    }

    /// The samples sent so far.
    pub fn samples_read(&self) -> u32 {
        self.samples_read
    }

    /// One read of `got` bytes, [`PACKET_SIZE`] when it is whole. Only the
    /// whole samples that arrived count, where C takes its buffer whole.
    pub fn feed(&mut self, got: usize) -> Feed {
        let have = u32::try_from(got / UNIT_SIZE).unwrap_or(0);
        if self.plan.discard >= have {
            self.plan.discard -= have;
            return if have == 0 { Feed::Done } else { Feed::Skip };
        }
        let mut start = self.plan.discard as usize * UNIT_SIZE;
        let mut len = have - self.plan.discard;
        self.plan.discard = 0;
        if self.samples_read.wrapping_add(len) >= self.plan.valid {
            len = self.plan.valid.saturating_sub(self.samples_read);
        }
        if len == 0 {
            return Feed::Done;
        }
        let mut pieces = Vec::with_capacity(3);
        let offset = self.plan.trigger_offset;
        if self.samples_read < offset && self.samples_read + len > offset {
            let before = offset - self.samples_read;
            let bytes = before as usize * UNIT_SIZE;
            pieces.push(Piece::Samples(start..start + bytes));
            len -= before;
            self.samples_read += before;
            start += bytes;
        }
        // With no look at `devc->trigger`, as api.c:639-640: a capture with
        // no trigger has a marker where the board reports one in its
        // samples. Commit 4c1433d1 says it triggers at once then, so the
        // marker is true, and only a recording could say otherwise.
        if self.samples_read == offset {
            pieces.push(Piece::Trigger);
        }
        pieces.push(Piece::Samples(start..start + len as usize * UNIT_SIZE));
        self.samples_read += len;
        Feed::Send(pieces)
    }
}

/// How the wait for the data ended.
enum Wait {
    Data,
    Stopped,
    Failed { gone: bool },
}

/// `analyzer_wait_data()`, bounded and stoppable. It polls as fast as the
/// transfers go, as the C does.
fn wait_data(link: &mut Link, stop: &AtomicBool) -> Wait {
    let mut failed = 0;
    loop {
        if stop.load(Ordering::SeqCst) {
            return Wait::Stopped;
        }
        match link.read_status() {
            Ok(status) if status & STATUS_BUSY == 0 => return Wait::Data,
            Ok(_) => failed = 0,
            Err(Error::Disconnected) => return Wait::Failed { gone: true },
            Err(_) => {
                failed += 1;
                if failed >= MAX_FAILED_POLLS {
                    return Wait::Failed { gone: false };
                }
            }
        }
    }
}

/// The status and the three addresses, or the capture cannot be placed.
fn read_addresses(link: &mut Link) -> Result<(u8, u32, u32, u32)> {
    let status = link.read_status()?;
    let stop = link.read_address(STOP_ADDRESS0)?;
    let now = link.read_address(NOW_ADDRESS0)?;
    let trigger = link.read_address(TRIGGER_ADDRESS0)?;
    Ok((status, stop, now, trigger))
}

/// The end of a capture. The link goes back first, so that whoever sees
/// the end can use the device.
fn end(link: Link, slot: &mut dyn FnMut(Link), sink: &PacketSink) {
    slot(link);
    sink.send(Packet::End);
}

/// `dev_acquisition_start()` from `analyzer_configure()` on, on its own
/// thread; `give_back` gets the link before the end packet goes out. A
/// stop sends what `dev_acquisition_stop()` sends (api.c:666), after
/// ending a readout it finds under way.
pub fn run(
    mut link: Link,
    regs: &Regs,
    sink: &PacketSink,
    stop: &AtomicBool,
    give_back: &mut dyn FnMut(Link),
) {
    link.configure(regs);
    link.start();
    log::info!("Waiting for data.");
    match wait_data(&mut link, stop) {
        Wait::Data => {}
        Wait::Stopped => {
            link.reset();
            sink.send(Packet::header());
            end(link, give_back, sink);
            return;
        }
        Wait::Failed { gone } => {
            log::error!("Lost the board while waiting for the data: no samples.");
            if !gone {
                link.reset();
            }
            sink.send(Packet::header());
            end(link, give_back, sink);
            return;
        }
    }
    let (status, stop_address, now_address, trigger_address) = match read_addresses(&mut link) {
        Ok(a) => a,
        Err(e) => {
            log::error!("Cannot read where the capture is ({e}): no samples.");
            sink.send(Packet::header());
            end(link, give_back, sink);
            return;
        }
    };
    let n = memory_bytes(regs.memory_size);
    let memory_size = n / 4;
    log::info!("Status = 0x{status:x}.");
    log::info!("Stop address       = 0x{stop_address:x}.");
    log::info!("Now address        = 0x{now_address:x}.");
    log::info!("Trigger address    = 0x{trigger_address:x}.");
    log::info!("Triggerbar address = 0x{:x}.", regs.triggerbar);
    log::info!("Ramsize trigger    = 0x{:x}.", regs.ramsize_trigger);
    log::info!("Memory size        = 0x{memory_size:x}.");
    sink.send(Packet::header());
    // An empty capture (api.c:548).
    if status & STATUS_READY != 0 && stop_address == 0 {
        end(link, give_back, sink);
        return;
    }
    link.read_start();
    let wanted = regs.ramsize_trigger.wrapping_add(regs.triggerbar);
    let plan = plan(status, stop_address, now_address, trigger_address, memory_size, wanted);
    log::info!("Need to discard {} samples.", plan.discard);
    let stopped = read_samples(&mut link, plan, n, sink, stop);
    link.read_stop();
    if stopped {
        link.reset();
    }
    end(link, give_back, sink);
}

/// The readout: api.c:601-650. True when it ended on a stop.
fn read_samples(
    link: &mut Link,
    plan: Plan,
    memory_bytes: u32,
    sink: &PacketSink,
    stop: &AtomicBool,
) -> bool {
    let mut readout = Readout::new(plan);
    for _ in 0..memory_bytes as usize / PACKET_SIZE {
        if stop.load(Ordering::SeqCst) {
            return true;
        }
        let data = link.read_packet();
        if data.len() != PACKET_SIZE {
            log::warn!("Tried to read {PACKET_SIZE} bytes, actually read {}.", data.len());
        }
        let feed = readout.feed(data.len());
        let done = feed == Feed::Done;
        if let Feed::Send(pieces) = feed {
            for piece in pieces {
                let packet = match piece {
                    Piece::Trigger => Packet::Trigger,
                    Piece::Samples(range) => Packet::Logic(LogicData {
                        unit_size: UNIT_SIZE,
                        data: data[range].to_vec(),
                    }),
                };
                if !sink.send(packet) {
                    return true;
                }
            }
        }
        if data.len() != PACKET_SIZE {
            // What follows would not join on what came: the board's
            // read pointer is unknown, and a halted endpoint stays so.
            log::error!(
                "The readout is cut short: the capture ends after {} samples.",
                readout.samples_read()
            );
            return stop.load(Ordering::SeqCst);
        }
        if done {
            return false;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Trigger;

    #[test]
    fn profiles_and_rates() {
        assert_eq!(PROFILES.len(), 13);
        let p = Profile::of_pid(0x700e).unwrap();
        assert_eq!((p.name, p.channels, p.max_samples()), ("LAP-C(16032)", 16, 32768));
        assert_eq!(p.samplerates().len(), 16);
        assert_eq!(p.samplerates().last(), Some(&100_000_000));
        let p = Profile::of_pid(0x700d).unwrap();
        assert_eq!((p.channels, p.max_samples()), (32, 2_097_152));
        assert_eq!(p.samplerates().len(), 18);
        assert!(Profile::of_pid(0x7000).is_none());
    }

    #[test]
    fn every_listed_rate_has_its_clock() {
        for r in SAMPLERATES {
            let (f, s) = split_rate(r);
            assert!(clock_regs(f, s).is_some(), "{r}");
        }
        assert_eq!(split_rate(25_000_000), (25, Scale::Mhz));
        assert_eq!(clock_regs(25, Scale::Mhz), Some((1, 5, 25)));
        assert_eq!(split_rate(800_000), (800, Scale::Khz));
        assert_eq!(clock_regs(800, Scale::Khz), Some((17, 5, 8)));
        assert_eq!(split_rate(500), (500, Scale::Hz));
        assert_eq!(clock_regs(500, Scale::Hz), Some((64, 10, 5)));
        assert_eq!(clock_regs(46, Scale::Hz), None);
        assert_eq!(clock_regs(3, Scale::Mhz), None);
    }

    #[test]
    fn memory_codes_follow_the_limit() {
        for (samples, code, bytes) in [
            (1, 0, 8 << 10),
            (2048, 0, 8 << 10),
            (2049, 1, 64 << 10),
            (16384, 1, 64 << 10),
            (16385, 2, 128 << 10),
            (32768, 2, 128 << 10),
            (32769, 3, 256 << 10),
            (65536, 3, 256 << 10),
            (262_144, 5, 1 << 20),
            (1_048_576, 7, 4 << 20),
            (2_097_152, 8, 8 << 20),
        ] {
            assert_eq!(memory_code(samples), code, "{samples}");
            assert_eq!(memory_bytes(code), bytes, "{samples}");
        }
    }

    #[test]
    fn thresholds_round_as_in_c() {
        assert_eq!(threshold_reg(1.5), 0x31);
        assert_eq!(threshold_reg(3.3), 33);
        assert_eq!(threshold_reg(0.0), 63);
        assert_eq!(threshold_reg(6.0), 8);
        assert_eq!(threshold_reg(-6.0), 117);
        assert!((clamp_threshold(9.0) - 6.0).abs() < 1e-12);
        assert!((clamp_threshold(-9.0) + 6.0).abs() < 1e-12);
    }

    #[test]
    fn the_trigger_bar_keeps_the_c_offsets() {
        // No trigger: all after it, less one.
        assert_eq!(trigger_bar(1000, 0, false, 50), (0, 999));
        // At full memory.
        assert_eq!(trigger_bar(2048, 0, false, 0), (0, 2047));
        // 500 before, 2 of them off.
        assert_eq!(trigger_bar(1000, 0, true, 50), (498, 500));
        // 1 or 2 before: none, and one off the rest.
        assert_eq!(trigger_bar(100, 0, true, 1), (0, 98));
        assert_eq!(trigger_bar(100, 0, true, 2), (0, 97));
        // A ratio of 100.
        assert_eq!(trigger_bar(1000, 0, true, 100), (998, 0));
        assert_eq!(trigger_bar(1, 0, false, 0), (0, 0));
        // A limit past the memory is the memory.
        assert_eq!(trigger_bar(5000, 0, false, 0), (0, 2047));
    }

    #[test]
    fn the_post_trigger_count_wraps_as_in_c() {
        assert_eq!(trigger_bar(1, 0, true, 100), (0, u32::MAX));
        assert_eq!(trigger_bar(2, 0, true, 100), (0, u32::MAX));
        // One before the trigger and none left, or one left: no wrap at 3.
        assert_eq!(trigger_bar(3, 0, true, 100), (1, 0));
        assert_eq!(trigger_bar(2, 0, true, 50), (0, 0));
    }

    #[test]
    fn a_limit_of_0_takes_the_memory() {
        // C: `0 - 1` post-trigger samples, 0xffffff on the wire.
        assert_eq!(trigger_bar(0, 0, false, 0), (0, 2047));
        assert_eq!(trigger_bar(0, 1, true, 50), (8190, 8192));
    }

    #[test]
    fn trigger_registers() {
        let on = vec![true; 32];
        assert_eq!(trigger_regs(None, &on).unwrap(), TriggerRegs::default());
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::One);
        t.add(0, 1, TriggerMatch::Zero);
        t.add(0, 8, TriggerMatch::One);
        t.add(0, 31, TriggerMatch::Zero);
        t.add(0, 25, TriggerMatch::Rising);
        let r = trigger_regs(Some(&t), &on).unwrap();
        assert_eq!(r.status, [0x09, 0, 0x01, 0, 0, 0, 0, 0x80]);
        assert_eq!(r.edge, 0x59);
        assert!(r.any);
    }

    #[test]
    fn the_last_edge_wins() {
        let on = vec![true; 16];
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::Rising);
        t.add(0, 1, TriggerMatch::Falling);
        assert_eq!(trigger_regs(Some(&t), &on).unwrap().edge, 0x81);
        let mut t = Trigger::new();
        t.add(0, 3, TriggerMatch::Edge);
        assert_eq!(trigger_regs(Some(&t), &on).unwrap().edge, 0xc3);
    }

    #[test]
    fn both_levels_set_both_bits() {
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::Zero);
        t.add(0, 0, TriggerMatch::One);
        assert_eq!(trigger_regs(Some(&t), &[true; 16]).unwrap().status[0], 3);
    }

    #[test]
    fn a_disabled_channel_counts_as_a_trigger_without_registers() {
        let mut t = Trigger::new();
        t.add(0, 2, TriggerMatch::One);
        let r = trigger_regs(Some(&t), &[false; 16]).unwrap();
        assert_eq!(r.status, [0; 8]);
        assert!(r.any);
    }

    #[test]
    fn analog_matches_are_refused() {
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::Over);
        assert!(trigger_regs(Some(&t), &[true; 16]).is_err());
    }

    /// `(status, stop, now, trigger, memory, wanted)` of the goldens'
    /// "default capture registers": 1051 discarded of 2048.
    #[test]
    fn the_plan_of_a_small_capture() {
        let p = plan(2, 0x3e6, 0x3e7, 0, 0x800, 0x3e7);
        assert_eq!(p.discard, 1051);
        assert_eq!(p.valid, 996);
        // The trigger address 0 falls past the samples.
        assert!(p.trigger_offset >= p.valid);
    }

    #[test]
    fn the_plan_when_the_memory_wrapped() {
        // Not ready: the data starts at now.
        let p = plan(0, 0x100, 0x101, 0x180, 0x800, 2047);
        assert_eq!(p.discard, 0);
        assert_eq!(p.valid, 0x7ff);
        // More than wanted: the head goes.
        let p = plan(0, 0x100, 0x101, 0x180, 0x800, 1000);
        assert_eq!(p.discard, 0x7ff - 1000);
        assert_eq!(p.valid, 1000);
    }

    #[test]
    fn a_now_address_of_0_clears_ready() {
        // api.c:563-564: no discard, the bogus words go out.
        let p = plan(2, 0x7ff, 0, 0, 0x800, 2047);
        assert_eq!(p.discard, 0);
        assert_eq!(p.valid, 0x7ff);
        assert_eq!(p.trigger_offset, 0);
    }

    #[test]
    fn a_limit_of_3_discards_everything() {
        // Stop 1, now moved to 2: `stop - now` underflows at api.c:582.
        let p = plan(2, 1, 2, 0, 0x800, 2);
        assert_eq!(p.discard, 0x800 + 0x7ff - 2);
        assert_eq!(p.valid, 2);
    }

    #[test]
    fn a_readout_skips_cuts_and_stops() {
        let plan = Plan { discard: 520, valid: 600, trigger_offset: 1000 };
        let mut r = Readout::new(plan);
        assert_eq!(r.feed(PACKET_SIZE), Feed::Skip);
        // 8 more to discard, 504 samples left of this read.
        assert_eq!(r.feed(PACKET_SIZE), Feed::Send(vec![Piece::Samples(32..2048)]));
        assert_eq!(r.samples_read(), 504);
        // The last 96, then one read more that gives nothing, as in C.
        assert_eq!(r.feed(PACKET_SIZE), Feed::Send(vec![Piece::Samples(0..384)]));
        assert_eq!(r.feed(PACKET_SIZE), Feed::Done);
    }

    #[test]
    fn the_trigger_splits_a_read() {
        let plan = Plan { discard: 0, valid: 400, trigger_offset: 100 };
        let mut r = Readout::new(plan);
        assert_eq!(
            r.feed(PACKET_SIZE),
            Feed::Send(vec![Piece::Samples(0..400), Piece::Trigger, Piece::Samples(400..1600)])
        );
    }

    #[test]
    fn the_trigger_at_the_first_sample() {
        // Whether a trigger was asked for or not, as api.c:639-640.
        let plan = Plan { discard: 0, valid: 10, trigger_offset: 0 };
        let mut r = Readout::new(plan);
        assert_eq!(r.feed(PACKET_SIZE), Feed::Send(vec![Piece::Trigger, Piece::Samples(0..40)]));
    }

    #[test]
    fn a_short_read_gives_only_what_came() {
        let plan = Plan { discard: 5, valid: 2000, trigger_offset: 5000 };
        let mut r = Readout::new(plan);
        // 255 whole samples of 1022 bytes, 5 of them discarded.
        assert_eq!(r.feed(1022), Feed::Send(vec![Piece::Samples(20..1020)]));
        assert_eq!(r.samples_read(), 250);
        // Nothing came at all.
        let mut r = Readout::new(plan);
        assert_eq!(r.feed(0), Feed::Done);
    }
}
