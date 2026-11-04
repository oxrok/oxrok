// Counterpart in sigrok:
//   libsigrok/src/hardware/lecroy-logicstudio/protocol.c,
//   libsigrok/src/hardware/lecroy-logicstudio/protocol.h,
//   Copyright (C) 2015 Tilman Sauerbeck <tilman@code-monkey.de>,
//   with commits by Uwe Hermann, Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the `LeCroy` `LogicStudio` 16.
//!
//! An FX2LP in front of an FPGA. Vendor requests, all with `wIndex` 5444
//! unless said:
//!
//! | request | dir | `wValue` | data | what |
//! |---|---|---|---|---|
//! | `0xb1` | IN | register | 2 bytes, big endian | read a register |
//! | `0xb1` | OUT | 0x12 | (register, value BE) triplets | write registers |
//! | `0xb1` | OUT | 0, `wIndex` 0 | one triplet | a threshold, before an upload |
//! | `0xb2` | OUT | 0x12 | 16 bytes | "status register" |
//! | `0xb3` | OUT | 7 | none | start an FPGA upload |
//! | `0xb4` | IN | 7 | 1 byte | upload verified: 1 |
//!
//! Interrupt IN 1 carries 32 byte status packets, the acquisition id big
//! endian at 2 and the state at 0x1f; state 0x04, "complete", has the time
//! of the latest sample at 6 and of the trigger at 0xc, 48 bit big endian
//! in units of 8 ns. Bulk OUT 6 takes a bitstream of exactly 464196 bytes
//! in 2048 byte chunks, one for 8 channels and one for 16, loaded when the
//! number of channel groups changes. Bulk IN 2 sends the whole 40960 byte
//! sample memory after the fetch registers are written, and the driver
//! reads it from the position the times give. All of this is from the C
//! driver, whose comments call most register meanings unknown; nothing
//! here was seen on a real board.

use crate::core::firmware;
use crate::core::trigger::TriggerCondition;
use crate::core::{Error, Result, TriggerMatch};
use crate::usb::{ControlRequest, Transport};
use std::time::Duration;

const EP_INTR: u8 = 0x81;
const EP_BULK: u8 = 0x82;
const EP_BITSTREAM: u8 = 0x06;

const CMD_READ_WRITE_REGS: u8 = 0xb1;
const CMD_WRITE_STATUS_REG: u8 = 0xb2;
const CMD_START_UPLOAD: u8 = 0xb3;
const CMD_VERIFY_UPLOAD: u8 = 0xb4;

/// `wIndex` of the vendor requests.
const MAGIC_INDEX: u16 = 5444;
/// `USB_TIMEOUT_MS`, for every transfer.
pub const USB_TIMEOUT: Duration = Duration::from_millis(100);

const FPGA_FIRMWARE_8: &str = "lecroy-logicstudio16-8.bitstream";
const FPGA_FIRMWARE_16: &str = "lecroy-logicstudio16-16.bitstream";
const FPGA_FIRMWARE_SIZE: usize = 464_196;
const FPGA_FIRMWARE_CHUNK_SIZE: usize = 2048;
const FIRMWARE_HINT: &str = "extract it with sigrok-util, see docs/DEVICES.md";

const NUM_TRIGGER_STAGES: usize = 2;
const TRIGGER_CFG_SIZE: usize = 45;

const TRIGGER_OP_A: u16 = 0x1000;
const TRIGGER_OP_A_AND_B: u16 = 0x4000;

const REG_ACQUISITION_ID: u8 = 0x00;
const REG_SAMPLERATE: u8 = 0x02;
const REG_PRETRIG_LO: u8 = 0x03;
const REG_PRETRIG_HI: u8 = 0x04;
const REG_POSTTRIG_LO: u8 = 0x05;
const REG_POSTTRIG_HI: u8 = 0x06;
const REG_ARM_TRIGGER: u8 = 0x07;
const REG_FETCH_SAMPLES: u8 = 0x08;
const REG_UNK1_LO: u8 = 0x09;
const REG_UNK1_HI: u8 = 0x0a;
const REG_UNK2_LO: u8 = 0x0b;
const REG_UNK2_HI: u8 = 0x0c;
const REG_UNK3_LO: u8 = 0x0d;
const REG_UNK3_HI: u8 = 0x0e;
const REG_UNK4_LO: u8 = 0x0f;
const REG_UNK4_HI: u8 = 0x10;
const REG_UNK5_LO: u8 = 0x11;
const REG_UNK5_HI: u8 = 0x12;
const REG_UNK6_LO: u8 = 0x13;
const REG_UNK6_HI: u8 = 0x14;
const REG_UNK0_LO: u8 = 0x15;
const REG_UNK0_HI: u8 = 0x16;
const REG_TRIGGER_CFG: u8 = 0x18;
const REG_TRIGGER_COMBINE_OP: u8 = 0x1b;
const REG_SELECT_CHANNELS: u8 = 0x21;
const REG_VOLTAGE_THRESH_EXTERNAL: u8 = 0x22;
const REG_VOLTAGE_THRESH_LOWER_CHANNELS: u8 = 0x23;
const REG_VOLTAGE_THRESH_UPPER_CHANNELS: u8 = 0x24;

/// `SAMPLE_BUF_SIZE`: the size of the sample memory.
pub const SAMPLE_BUF_SIZE: usize = 40960;
/// The bytes shown of it: "the last few bytes seem to contain garbage
/// data" (protocol.c:1035-1039).
const SHOWN_BYTES: usize = (SAMPLE_BUF_SIZE >> 10) * 1000;
/// `INTR_BUF_SIZE`: the size of a status packet.
pub const INTR_BUF_SIZE: usize = 32;
/// `CONV_8TO16_BUF_SIZE` / 2: the samples of one packet with 8 channels.
const CONV_CHUNK_SAMPLES: usize = 4096;
/// The bytes of a sample transfer: the first, and those after it.
const FIRST_TRANSFER: usize = 17 << 10;
const NEXT_TRANSFER: usize = 16 << 10;
/// Empty sample transfers in a row after which the fetch is given up.
/// The C driver resubmits for ever (protocol.c:1019-1027).
const MAX_EMPTY_TRANSFERS: usize = 64;

/// A rate of `samplerates[]` in `protocol.c`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RateInfo {
    /// The sample rate in Hz.
    pub rate: u64,
    /// The offset of the trigger's sample with 8 channels.
    pub trigger_offset: i8,
    /// The value of `REG_SAMPLERATE`.
    pub cfg: u8,
}

const fn info(rate: u64, trigger_offset: i8, cfg: u8) -> RateInfo {
    RateInfo { rate, trigger_offset, cfg }
}

/// `samplerates[]` of `protocol.c`, which has 1 GHz where `api.c`'s list
/// stops at 500 MHz.
pub const RATES: [RateInfo; 19] = [
    info(1_000_000_000, -24, 0x1f),
    info(500_000_000, -6, 0x00),
    info(250_000_000, -4, 0x01),
    info(100_000_000, 2, 0x03),
    info(50_000_000, 4, 0x04),
    info(25_000_000, 8, 0x05),
    info(10_000_000, 4, 0x07),
    info(5_000_000, 8, 0x08),
    info(2_500_000, 8, 0x09),
    info(1_000_000, 8, 0x0b),
    info(500_000, 8, 0x0c),
    info(250_000, 8, 0x0d),
    info(100_000, 8, 0x0f),
    info(50_000, 8, 0x10),
    info(25_000, 8, 0x11),
    info(10_000, 8, 0x13),
    info(5_000, 8, 0x14),
    info(2_500, 8, 0x15),
    info(1_000, 8, 0x17),
];

/// The entry of `rate`.
pub fn rate_info(rate: u64) -> Option<RateInfo> {
    RATES.iter().copied().find(|r| r.rate == rate)
}

/// `struct trigger_config`: the matches of one stage, as channel masks.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StageConfig {
    /// Channels matching a rising edge.
    pub rising: u16,
    /// Channels matching a falling edge.
    pub falling: u16,
    /// Channels matching either edge.
    pub any: u16,
    /// Channels that must be high.
    pub ones: u16,
    /// Channels that must be low.
    pub zeroes: u16,
}

impl StageConfig {
    /// `transform_trigger()`, for the conditions on enabled channels.
    pub fn from_conditions<'a>(conditions: impl Iterator<Item = &'a TriggerCondition>) -> Self {
        let mut s = StageConfig::default();
        for c in conditions {
            let bit = 1u16 << c.channel;
            match c.matches {
                TriggerMatch::Rising => s.rising |= bit,
                TriggerMatch::Falling => s.falling |= bit,
                TriggerMatch::Edge => s.any |= bit,
                TriggerMatch::One => s.ones |= bit,
                TriggerMatch::Zero => s.zeroes |= bit,
                _ => {}
            }
        }
        s
    }

    /// The 45 bytes `program_trigger()` writes for a stage; `last` is the
    /// second stage, whether or not it holds any match.
    pub fn encode(&self, last: bool) -> [u8; TRIGGER_CFG_SIZE] {
        let mut b = [0u8; TRIGGER_CFG_SIZE];
        let put = |b: &mut [u8; TRIGGER_CFG_SIZE], at: usize, v: u16| {
            b[at..at + 2].copy_from_slice(&v.to_le_bytes());
        };
        put(&mut b, 0x00, !(self.rising | self.falling));
        put(&mut b, 0x05, self.rising | self.falling | self.any);
        if self.ones | self.zeroes != 0 {
            b[0x09] = 0x10;
        }
        put(&mut b, 0x0a, self.rising);
        put(&mut b, 0x0f, self.ones | self.zeroes);
        b[0x13] = 0x10;
        put(&mut b, 0x14, self.ones | 0x8000);
        if self.ones == 0x01 {
            put(&mut b, 0x19, self.ones << 1);
        } else {
            put(&mut b, 0x19, self.ones | 0x0001);
        }
        if last {
            b[0x09] = 0x10;
            b[0x28..=0x2b].fill(0xff);
            b[0x2c] = 0x80;
        }
        b
    }
}

/// The registers of `upload_trigger()`'s transfers for a stage at
/// `offset`: each byte as three writes, bit 15 set in the middle one,
/// five bytes a transfer.
fn trigger_transfers(bytes: &[u8; TRIGGER_CFG_SIZE], offset: u8) -> Vec<Vec<(u8, u16)>> {
    (0..TRIGGER_CFG_SIZE)
        .step_by(5)
        .map(|i| {
            (i..i + 5)
                .flat_map(|at| {
                    let addr = u16::from(offset) + at as u16;
                    let value = addr << 8 | u16::from(bytes[at]);
                    [
                        (REG_TRIGGER_CFG, value),
                        (REG_TRIGGER_CFG, value | 0x8000),
                        (REG_TRIGGER_CFG, value),
                    ]
                })
                .collect()
        })
        .collect()
}

/// The registers as they go over the wire.
fn encode_regs(regs: &[(u8, u16)]) -> Vec<u8> {
    regs.iter()
        .flat_map(|&(reg, val)| {
            let [hi, lo] = val.to_be_bytes();
            [reg, hi, lo]
        })
        .collect()
}

/// `transform_sample_count()`.
fn transform_sample_count(samples: u32, groups: u32) -> u32 {
    let d = 8 / groups;
    (samples + 0x1c + d + d - 1) / d
}

/// `calc_unk0()`, both of its results.
const fn unk0() -> (u32, u32) {
    let t: u32 = 20000 / 4;
    ((t + 63) | 63, (t + 63) & !63)
}

/// `ALIGN2_DOWN`.
fn align2_down(n: u64, p: u64) -> u64 {
    (if n > p { n - p } else { n }) & !(p - 1)
}

/// The place of a capture in the sample counter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window {
    /// The first sample to show.
    pub earliest: u64,
    /// The sample the trigger fired at.
    pub trigger: u64,
}

/// What `handle_intr_data()` makes of the "complete" packet. The C code
/// subtracts in 64 bits with a `FIXME: Underflow` (protocol.c:388-392), and
/// its earliest sample has no guard either (373-374): a value below what is
/// subtracted wraps to a place in the memory unrelated to the capture. Here
/// it stops at sample 0. Nothing else differs.
pub fn completion_window(p: &[u8], rate: RateInfo, groups: u32, thousands: u32) -> Window {
    let be48 = |at: usize| {
        let mut v = [0u8; 8];
        v[2..].copy_from_slice(&p[at..at + 6]);
        u64::from_be_bytes(v)
    };
    let divider = 1_000_000_000 / rate.rate;
    let latest = align2_down(be48(0x06) * 8 / divider, u64::from(8 / groups));
    let earliest = latest.saturating_sub(u64::from(thousands) * 1000);
    let mut trigger = i128::from(be48(0x0c) * 8 | u64::from(p[0x13] & 7));
    trigger += i128::from(rate.trigger_offset);
    if groups > 1 {
        trigger -= 8;
    }
    trigger -= 0x18;
    if divider > 1 {
        trigger = (trigger - i128::from(divider)) / i128::from(divider);
    }
    let trigger = u64::try_from(trigger.max(0)).unwrap_or(0) + 1;
    Window { earliest, trigger }
}

/// `sample_to_byte_offset()`.
pub fn sample_to_byte_offset(sample: u64, groups: u32, thousands: u32) -> usize {
    (sample % (u64::from(thousands) << 10)) as usize * groups as usize
}

/// Which stretches of the memory `recv_bulk_transfer()` shows, in order,
/// as (offset, length), and how many of them come before the trigger.
/// Empty stretches are left out: the C code sends them as packets of no
/// bytes, which libsigrok's outputs do not survive (protocol.c:957-960,
/// 980-989).
pub fn window_stretches(mut read: usize, trigger: usize) -> (Vec<(usize, usize)>, usize) {
    let mut left = SHOWN_BYTES;
    let mut out = Vec::new();
    if trigger < read {
        let len = left.min(SAMPLE_BUF_SIZE - read);
        out.push((read, len));
        left -= len;
        read = 0;
    }
    let len = left.min(trigger - read);
    if len > 0 {
        out.push((read, len));
    }
    left -= len;
    read = (read + len) % SAMPLE_BUF_SIZE;
    let before = out.len();
    while left > 0 {
        let len = left.min(SAMPLE_BUF_SIZE - read);
        out.push((read, len));
        left -= len;
        read = (read + len) % SAMPLE_BUF_SIZE;
    }
    (out, before)
}

/// `send_samples()`: the logic packets of a stretch of memory, 16 bit
/// samples either way. With one channel group each byte is a sample,
/// moved up to its group, in packets of 4096 samples at most.
pub fn logic_payloads(samples: &[u8], mask: u16) -> Vec<Vec<u8>> {
    let lower = mask & 0x00ff != 0;
    let upper = mask & 0xff00 != 0;
    if lower && upper {
        return vec![samples.to_vec()];
    }
    samples
        .chunks(CONV_CHUNK_SAMPLES)
        .map(|chunk| chunk.iter().flat_map(|&b| if lower { [b, 0] } else { [0, b] }).collect())
        .collect()
}

/// The `libusb_error` of a failed transfer, as `CTRL_IN failed: %i.` logs it.
fn libusb_code(e: &Error) -> i32 {
    match crate::usb::libusb_error_name(e) {
        "LIBUSB_ERROR_ACCESS" => -3,
        "LIBUSB_ERROR_NO_DEVICE" => -4,
        "LIBUSB_ERROR_BUSY" => -6,
        "LIBUSB_ERROR_TIMEOUT" => -7,
        "LIBUSB_ERROR_PIPE" => -9,
        "LIBUSB_ERROR_NOT_SUPPORTED" => -12,
        _ => -1,
    }
}

/// The transport, and what the device was last told, kept across captures
/// on an open device.
pub struct Link {
    t: Box<dyn Transport>,
    /// `num_enabled_channel_groups`; 0 until a bitstream is loaded.
    groups: u32,
    /// `num_thousand_samples`.
    thousands: u32,
    /// The mask of the channels enabled in this capture.
    mask: u16,
    magic_arm_trigger: u8,
    magic_fetch_samples: u8,
    acquisition_id: u16,
}

/// What a status packet says.
#[derive(Debug, PartialEq, Eq)]
pub enum Status {
    /// The capture goes on, or the packet is about an earlier one.
    Waiting,
    /// The capture is complete: the place of its samples.
    Complete(Window),
}

impl Link {
    /// A link to a device with no bitstream loaded.
    pub fn new(t: Box<dyn Transport>) -> Self {
        Link {
            t,
            groups: 0,
            thousands: 0,
            mask: 0,
            magic_arm_trigger: 0,
            magic_fetch_samples: 0,
            acquisition_id: 0,
        }
    }

    /// The channels enabled in this capture.
    pub fn mask(&self) -> u16 {
        self.mask
    }

    fn request(request: u8, value: u16) -> ControlRequest {
        ControlRequest::vendor(request, value, MAGIC_INDEX)
    }

    /// `read_register()`.
    fn read_register(&mut self, reg: u16) -> Result<u16> {
        let req = Self::request(CMD_READ_WRITE_REGS, reg);
        match self.t.control_in(req, 2, USB_TIMEOUT) {
            Ok(d) if d.len() == 2 => Ok(u16::from_be_bytes([d[0], d[1]])),
            Ok(d) => {
                log::error!("CTRL_IN failed: {}.", d.len());
                Err(Error::device("short register read"))
            }
            Err(e) => {
                log::error!("CTRL_IN failed: {}.", libusb_code(&e));
                Err(e)
            }
        }
    }

    /// `write_registers_sync()`.
    fn write(&mut self, value: u16, index: u16, regs: &[(u8, u16)]) -> Result<()> {
        let req = ControlRequest::vendor(CMD_READ_WRITE_REGS, value, index);
        self.t.control_out(req, &encode_regs(regs), USB_TIMEOUT).inspect_err(|_| {
            log::error!("write_registers_sync({value}/{index}) failed.");
        })
    }

    fn write_regs(&mut self, regs: &[(u8, u16)]) -> Result<()> {
        self.write(0x12, MAGIC_INDEX, regs)
    }

    /// `lls_stop_acquisition()`.
    pub fn stop_acquisition(&mut self) -> Result<()> {
        let m = u16::from(self.magic_arm_trigger);
        self.write_regs(&[(REG_ARM_TRIGGER, m), (REG_ARM_TRIGGER, m | 0x02)])
    }

    /// `upload_fpga_bitstream()`. A failure is logged as in C except for
    /// the name of a refused `0xb3`: the C code names `ret`, always
    /// `SR_ERR` there, rather than `r` (protocol.c:452-454), so it reads
    /// `LIBUSB_ERROR_IO` whatever the device answered.
    fn upload_bitstream(&mut self, name: &str) -> Result<()> {
        let image = firmware::read(name, FIRMWARE_HINT).inspect_err(|_| {
            log::error!("Failed to open resource '{name}' (use loglevel 5/spew for details).");
        })?;
        if image.len() != FPGA_FIRMWARE_SIZE {
            log::error!("Invalid FPGA firmware file size: {} bytes.", image.len());
            return Err(Error::device("wrong size of the FPGA firmware"));
        }
        self.t.control_out(Self::request(CMD_START_UPLOAD, 0x07), &[], USB_TIMEOUT).inspect_err(
            |e| {
                let name = crate::usb::libusb_error_name(e);
                log::error!("Failed to initiate firmware upload: {name}.");
            },
        )?;
        for chunk in image.chunks(FPGA_FIRMWARE_CHUNK_SIZE) {
            let sent = self.t.bulk_out(EP_BITSTREAM, chunk, USB_TIMEOUT);
            if !matches!(sent, Ok(n) if n == chunk.len()) {
                log::error!("FPGA firmware upload failed.");
                return Err(Error::device("the FPGA firmware upload failed"));
            }
        }
        for _ in 0..4 {
            std::thread::sleep(Duration::from_millis(250));
            let req = Self::request(CMD_VERIFY_UPLOAD, 0x07);
            match self.t.control_in(req, 1, USB_TIMEOUT) {
                Ok(d) if d.len() == 1 => {
                    if d[0] == 0x01 {
                        return Ok(());
                    }
                }
                Ok(d) => {
                    log::error!("CTRL_IN failed: {}.", d.len());
                    return Err(Error::device("short verify"));
                }
                Err(e) => {
                    log::error!("CTRL_IN failed: {}.", libusb_code(&e));
                    return Err(e);
                }
            }
        }
        Err(Error::device("the FPGA did not take the firmware"))
    }

    /// `read_0f12()`.
    fn read_0f12(&mut self) -> Result<u64> {
        let mut v = 0;
        for reg in 0x0f..=0x12 {
            v = v << 16 | u64::from(self.read_register(reg)?);
        }
        Ok(v)
    }

    /// `wait_for_dev_to_settle()`.
    fn wait_to_settle(&mut self) -> Result<()> {
        let mut old = self.read_0f12()?;
        for _ in 0..100 {
            let new = self.read_0f12()?;
            if new == old {
                return Ok(());
            }
            old = new;
        }
        Err(Error::device("the device did not settle"))
    }

    /// `program_trigger()`; a stage not given is all zeroes, as
    /// `configure_trigger()`'s `memset`.
    fn program_trigger(&mut self, stages: &[StageConfig]) -> Result<()> {
        let empty = StageConfig::default();
        for (i, offset) in [0x00, 0x40].into_iter().enumerate() {
            let bytes = stages.get(i).unwrap_or(&empty).encode(i == NUM_TRIGGER_STAGES - 1);
            for regs in trigger_transfers(&bytes, offset) {
                self.write_regs(&regs).inspect_err(|_| {
                    log::error!("Failed to upload trigger config.");
                })?;
            }
        }
        let op = if stages.len() > 1 { TRIGGER_OP_A_AND_B } else { TRIGGER_OP_A };
        self.write_regs(&[(REG_TRIGGER_COMBINE_OP, op)])
    }

    /// `lls_setup_acquisition()`, what `config_commit()` runs before a
    /// capture. Every failure is logged as in C. More stages than the
    /// two the device has fail after the channel group is written, with
    /// no message of the driver's (`configure_trigger()`).
    pub fn setup_acquisition(&mut self, mask: u16, stages: &[StageConfig]) -> Result<()> {
        let threshold = [
            (REG_VOLTAGE_THRESH_LOWER_CHANNELS, 0x00c3),
            (REG_VOLTAGE_THRESH_UPPER_CHANNELS, 0x00c2),
            (REG_VOLTAGE_THRESH_EXTERNAL, 0x003e),
        ];
        let lower = mask & 0x00ff != 0;
        let upper = mask & 0xff00 != 0;
        let (thousands, groups) = if lower == upper { (20, 2) } else { (40, 1) };
        let select = u16::from(upper && !lower);
        self.mask = mask;
        let upload = self.groups != groups;
        if upload {
            self.stop_acquisition().inspect_err(|_| {
                log::error!("Cannot stop acquisition for FPGA bitstream upload.");
            })?;
            for t in threshold {
                self.write(0, 0, &[t])?;
            }
            let name = if groups == 1 { FPGA_FIRMWARE_8 } else { FPGA_FIRMWARE_16 };
            self.upload_bitstream(name).inspect_err(|_| {
                log::error!("Firmware not accepted by device.");
            })?;
            self.wait_to_settle().inspect_err(|_| {
                log::error!("Device did not settle in time.");
            })?;
            for t in threshold {
                self.write_regs(&[t])?;
            }
            self.magic_arm_trigger = 0;
            self.magic_fetch_samples = 0;
        }
        self.write_regs(&[(REG_SELECT_CHANNELS, select)])?;
        if stages.len() > NUM_TRIGGER_STAGES {
            return Err(Error::arg("the device has two trigger stages"));
        }
        self.program_trigger(stages)?;
        self.write_regs(&[threshold[0]])?;
        self.write_regs(&[threshold[1]])?;
        if upload {
            const STATUS_REG: [u8; 16] =
                [0x1, 0x0, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6];
            let req = Self::request(CMD_WRITE_STATUS_REG, 0x12);
            self.t.control_out(req, &STATUS_REG, USB_TIMEOUT).inspect_err(|e| {
                let name = crate::usb::libusb_error_name(e);
                log::error!("Failed to write status register: {name}.");
            })?;
        }
        self.thousands = thousands;
        self.groups = groups;
        Ok(())
    }

    /// `lls_start_acquisition()`'s registers, the arm last.
    fn arm_regs(&mut self, rate: RateInfo, capture_ratio: u64) -> [(u8, u16); 17] {
        let (unk0, _) = unk0();
        let total = self.thousands * 1000;
        // At most 40000 * 100 / 100: the ratio is checked at 100.
        let pre = (u64::from(total) * capture_ratio / 100) as u32;
        let post = total - pre;
        let pre = transform_sample_count(pre, self.groups);
        let post = transform_sample_count(post, self.groups);
        let lo = |v: u32| (v & 0xffff) as u16;
        let hi = |v: u32| (v >> 16) as u16;
        let m = u16::from(self.magic_arm_trigger);
        self.acquisition_id = self.acquisition_id.wrapping_add(1);
        self.magic_arm_trigger = 0x0c;
        [
            (REG_ARM_TRIGGER, m),
            (REG_ARM_TRIGGER, m | 0x02),
            (REG_UNK6_LO, 0),
            (REG_UNK6_HI, 0),
            (REG_UNK0_LO, lo(unk0)),
            (REG_UNK0_HI, hi(unk0)),
            (REG_UNK4_LO, 0),
            (REG_UNK4_HI, 0),
            (REG_UNK5_LO, 0),
            (REG_UNK5_HI, 0),
            (REG_ACQUISITION_ID, self.acquisition_id),
            (REG_SAMPLERATE, u16::from(rate.cfg)),
            (REG_PRETRIG_LO, lo(pre)),
            (REG_PRETRIG_HI, hi(pre)),
            (REG_POSTTRIG_LO, lo(post)),
            (REG_POSTTRIG_HI, hi(post)),
            (REG_ARM_TRIGGER, 0x0c | 0x01),
        ]
    }

    /// Arms the trigger.
    pub fn start_acquisition(&mut self, rate: RateInfo, capture_ratio: u64) -> Result<()> {
        let regs = self.arm_regs(rate, capture_ratio);
        self.write_regs(&regs)
    }

    /// `handle_intr_data()` on one status packet.
    pub fn status(&self, p: &[u8], rate: RateInfo) -> Status {
        if u16::from_be_bytes([p[0x02], p[0x03]]) != self.acquisition_id || p[0x1f] != 0x04 {
            return Status::Waiting;
        }
        Status::Complete(completion_window(p, rate, self.groups, self.thousands))
    }

    /// One read of the status endpoint.
    pub fn read_status(&mut self) -> Result<Vec<u8>> {
        self.t.interrupt_in(EP_INTR, INTR_BUF_SIZE, USB_TIMEOUT)
    }

    /// `fetch_samples_async()`'s registers.
    fn fetch_regs(&mut self, earliest: u64) -> [(u8, u16); 12] {
        let unk1 = earliest % (u64::from(self.thousands) << 10);
        let unk1 = unk1 * u64::from(self.groups) / 8;
        let (unk2, unk3) = unk0();
        let lo = |v: u64| (v & 0xffff) as u16;
        let hi = |v: u64| ((v >> 16) & 0xffff) as u16;
        let m = u16::from(self.magic_fetch_samples);
        self.magic_fetch_samples = 0x01;
        [
            (REG_UNK1_LO, lo(unk1)),
            (REG_UNK1_HI, hi(unk1)),
            (REG_FETCH_SAMPLES, m),
            (REG_FETCH_SAMPLES, m | 0x02),
            (REG_UNK1_LO, 0),
            (REG_UNK1_HI, 0),
            (REG_UNK2_LO, lo(u64::from(unk2))),
            (REG_UNK2_HI, hi(u64::from(unk2))),
            (REG_UNK3_LO, lo(u64::from(unk3))),
            (REG_UNK3_HI, hi(u64::from(unk3))),
            (REG_FETCH_SAMPLES, 0x02),
            (REG_FETCH_SAMPLES, 0x03),
        ]
    }

    /// The state of a capture of 16 channels with no setup, for tests.
    #[cfg(test)]
    pub(super) fn with_capture(mut self) -> Self {
        self.groups = 2;
        self.thousands = 20;
        self
    }

    /// Asks for the sample memory.
    pub fn fetch_samples(&mut self, earliest: u64) -> Result<()> {
        let regs = self.fetch_regs(earliest);
        self.write_regs(&regs)
    }

    /// The sample memory: one transfer at a time of 17 KiB, then 16 KiB
    /// at most, each taking what arrived within its timeout, as
    /// `recv_bulk_transfer()`. A transfer that holds nothing goes on;
    /// `give_up` ends such a fetch when it is asked to stop, and so does
    /// a long silence.
    pub fn read_memory(&mut self, give_up: impl Fn() -> bool) -> Result<Vec<u8>> {
        let mut memory = Vec::with_capacity(SAMPLE_BUF_SIZE);
        let mut size = FIRST_TRANSFER;
        let mut empty = 0;
        while memory.len() < SAMPLE_BUF_SIZE {
            let mut stream = self.t.bulk_in_stream_timed(EP_BULK, size, 1, USB_TIMEOUT)?;
            let data = loop {
                match stream.next(USB_TIMEOUT) {
                    Ok(d) if !d.is_empty() => break d,
                    Ok(_) | Err(Error::Timeout) => {}
                    Err(e) => return Err(e),
                }
                empty += 1;
                if empty > MAX_EMPTY_TRANSFERS || give_up() {
                    return Err(Error::Timeout);
                }
            };
            empty = 0;
            let room = SAMPLE_BUF_SIZE - memory.len();
            memory.extend_from_slice(&data[..data.len().min(room)]);
            size = NEXT_TRANSFER.min(SAMPLE_BUF_SIZE - memory.len());
        }
        Ok(memory)
    }

    /// The memory's stretches of a window, see [`window_stretches`].
    pub fn stretches(&self, w: Window) -> (Vec<(usize, usize)>, usize) {
        let read = sample_to_byte_offset(w.earliest, self.groups, self.thousands);
        let trigger = sample_to_byte_offset(w.trigger, self.groups, self.thousands);
        log::trace!("Start reading at offset {read:#06x}, trigger offset {trigger:#06x}.");
        window_stretches(read, trigger)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }

    fn rate(r: u64) -> RateInfo {
        rate_info(r).unwrap()
    }

    fn link(steps: &str) -> Link {
        let text = format!("# oxrok usb fixture 1\ndevice vid=05ff pid=a002\n{steps}");
        Link::new(Box::new(Replay::from_text(&text).unwrap()))
    }

    /// The rates in the C table, 1 GHz among them, and no other.
    #[test]
    fn rate_table() {
        assert_eq!(rate(1_000_000_000).cfg, 0x1f);
        assert_eq!(rate(1_000_000).trigger_offset, 8);
        assert!(rate_info(3_000_000).is_none());
    }

    /// What the C driver sends for `D3=r` (tests/emu, lib 16ch 1M D3
    /// rising): the first transfer of the first stage, the last of the
    /// second.
    #[test]
    fn trigger_transfers_are_the_c_drivers() {
        let stage = StageConfig { rising: 1 << 3, ..StageConfig::default() };
        let t = trigger_transfers(&stage.encode(false), 0x00);
        assert_eq!(t.len(), 9);
        assert_eq!(hex("1800f71880f71800f71801ff1881ff1801ff"), encode_regs(&t[0])[..18]);
        let empty = StageConfig::default().encode(true);
        let t = trigger_transfers(&empty, 0x40);
        assert_eq!(hex("1868ff18e8ff1868ff1869ff18e9ff1869ff"), encode_regs(&t[8])[..18]);
        assert_eq!(empty[0x09], 0x10);
        assert_eq!(empty[0x2c], 0x80);
    }

    #[test]
    fn stage_quirks_are_kept() {
        let d0 = StageConfig { ones: 1, ..StageConfig::default() }.encode(false);
        assert_eq!(d0[0x19], 2, "D0 high alone moves to bit 1");
        let d1 = StageConfig { ones: 2, ..StageConfig::default() }.encode(false);
        assert_eq!(d1[0x19], 3);
        let f = StageConfig { falling: 4, ..StageConfig::default() }.encode(false);
        assert_eq!(f[0x00], 0xfb);
        assert_eq!(f[0x05], 4);
        assert_eq!(f[0x0a], 0, "falling has no bits of its own");
    }

    #[test]
    fn stage_from_conditions() {
        let c = |channel, matches| TriggerCondition { channel, matches, value: 0.0 };
        let list = [
            c(0, TriggerMatch::Rising),
            c(1, TriggerMatch::Falling),
            c(2, TriggerMatch::Edge),
            c(3, TriggerMatch::One),
            c(15, TriggerMatch::Zero),
        ];
        let s = StageConfig::from_conditions(list.iter());
        assert_eq!(s, StageConfig { rising: 1, falling: 2, any: 4, ones: 8, zeroes: 0x8000 });
    }

    /// The arm and the fetch of that capture.
    #[test]
    fn arm_and_fetch_registers() {
        let mut l = link("");
        l.groups = 2;
        l.thousands = 20;
        let arm = encode_regs(&l.arm_regs(rate(1_000_000), 50));
        let want = "0700000700021300001400001513ff1600000f000010000011000012000000000102000b\
                    0309cc0400000509cc06000007000d";
        assert_eq!(arm, hex(want));
        assert_eq!(l.magic_arm_trigger, 0x0c);
        let fetch = encode_regs(&l.fetch_regs(36));
        let want = "0900090a00000800000800020900000a00000b13ff0c00000d13c00e0000080002080003";
        assert_eq!(fetch, hex(want));
        assert_eq!(l.magic_fetch_samples, 1);
    }

    #[test]
    fn transformed_counts() {
        assert_eq!(transform_sample_count(10000, 2), 2508);
        assert_eq!(transform_sample_count(0, 1), (0x1c + 8 + 7) / 8);
    }

    /// The last packet of that capture: the trigger 10004 samples in.
    #[test]
    fn the_complete_packet_gives_the_window() {
        let p = hex("000000010000000000263a9f00000013265b0000000000000000000000000004");
        let mut l = link("");
        l.groups = 2;
        l.thousands = 20;
        l.acquisition_id = 1;
        let Status::Complete(w) = l.status(&p, rate(1_000_000)) else { panic!("not complete") };
        assert_eq!(w, Window { earliest: 36, trigger: 10040 });
        // Another id, or another state.
        l.acquisition_id = 2;
        assert_eq!(l.status(&p, rate(1_000_000)), Status::Waiting);
        l.acquisition_id = 1;
        let mut q = p.clone();
        q[0x1f] = 0x05;
        assert_eq!(l.status(&q, rate(1_000_000)), Status::Waiting);
    }

    /// Where the C code's 64 bit subtractions wrap, the sample is 0.
    #[test]
    fn early_times_stop_at_sample_zero() {
        let mut p = vec![0u8; INTR_BUF_SIZE];
        p[0x1f] = 0x04;
        for r in [1_000_000_000, 1_000_000, 1_000] {
            let w = completion_window(&p, rate(r), 2, 20);
            assert_eq!(w, Window { earliest: 0, trigger: 1 }, "at {r}");
        }
    }

    #[test]
    fn align_down() {
        assert_eq!(align2_down(20, 4), 16);
        assert_eq!(align2_down(4, 4), 4);
        assert_eq!(align2_down(3, 4), 0);
        assert_eq!(align2_down(23, 8), 8);
    }

    #[test]
    fn offsets_wrap_the_memory() {
        assert_eq!(sample_to_byte_offset(20480 + 3, 2, 20), 6);
        assert_eq!(sample_to_byte_offset(40960 + 3, 1, 40), 3);
    }

    /// Before the trigger, after it, and 40000 bytes in all.
    #[test]
    fn stretches_of_a_window() {
        let total = |s: &[(usize, usize)]| s.iter().map(|x| x.1).sum::<usize>();
        // Straight through the memory.
        let (s, before) = window_stretches(100, 200);
        assert_eq!(s, [(100, 100), (200, 39900)]);
        assert_eq!(before, 1);
        assert_eq!(total(&s), 40000);
        // The trigger past the wrap.
        let (s, before) = window_stretches(40000, 500);
        assert_eq!(s, [(40000, 960), (0, 500), (500, 38540)]);
        assert_eq!(before, 2);
        // The post-trigger samples wrap.
        let (s, before) = window_stretches(30000, 35000);
        assert_eq!(s, [(30000, 5000), (35000, 5960), (0, 29040)]);
        assert_eq!(before, 1);
        assert_eq!(total(&s), 40000);
    }

    /// No empty stretch: the trigger where the read starts, and a trigger
    /// after what is shown.
    #[test]
    fn empty_stretches_are_left_out() {
        let (s, before) = window_stretches(100, 100);
        assert_eq!(s, [(100, 40000)]);
        assert_eq!(before, 0);
        let (s, before) = window_stretches(0, 40000);
        assert_eq!(s, [(0, 40000)]);
        assert_eq!(before, 1);
    }

    #[test]
    fn eight_channels_are_widened_in_chunks() {
        let data = vec![0xa5u8; 8192];
        let low = logic_payloads(&data, 0x00ff);
        assert_eq!(low.len(), 2, "no empty third packet");
        assert_eq!(low[0].len(), 8192);
        assert_eq!(low[0][..2], [0xa5, 0]);
        let high = logic_payloads(&data[..3], 0xff00);
        assert_eq!(high, [vec![0, 0xa5, 0, 0xa5, 0, 0xa5]]);
        assert!(logic_payloads(&[], 0xff00).is_empty());
        assert_eq!(logic_payloads(&[1, 2], 0xffff), [vec![1, 2]]);
    }

    #[test]
    fn libusb_codes() {
        assert_eq!(libusb_code(&Error::Timeout), -7);
        assert_eq!(libusb_code(&Error::device("stalled")), -9);
        assert_eq!(libusb_code(&Error::Disconnected), -4);
        assert_eq!(libusb_code(&Error::device("what")), -1);
    }

    /// A short first transfer, then 16 KiB at most, the rest last.
    #[test]
    fn the_memory_comes_in_the_c_drivers_transfers() {
        let mut l = link(
            "bulk_in 82 17408 -> 11{1000}\n\
             bulk_in 82 16384 -> 22{16384}\n\
             bulk_in 82 16384 -> 33{16384}\n\
             bulk_in 82 7192 -> 44{7192}\n",
        );
        let m = l.read_memory(|| false).unwrap();
        assert_eq!(m.len(), SAMPLE_BUF_SIZE);
        assert_eq!(m[999], 0x11);
        assert_eq!(m[1000], 0x22);
        assert_eq!(m[SAMPLE_BUF_SIZE - 1], 0x44);
    }

    /// Timeouts are not fatal, but a device that sends nothing is given up
    /// on: the C driver resubmits for ever.
    #[test]
    fn a_silent_memory_is_given_up_on() {
        let steps = "bulk_in 82 17408 -> err timeout\n".repeat(MAX_EMPTY_TRANSFERS + 1);
        let mut l = link(&steps);
        assert!(matches!(l.read_memory(|| false), Err(Error::Timeout)));
        // And at once when stopping.
        let mut l = link("bulk_in 82 17408 -> err timeout\n");
        assert!(matches!(l.read_memory(|| true), Err(Error::Timeout)));
    }

    #[test]
    fn a_failed_transfer_ends_the_fetch() {
        let mut l = link("bulk_in 82 17408 -> err disconnected\n");
        assert!(matches!(l.read_memory(|| false), Err(Error::Disconnected)));
    }

    #[test]
    fn stop_writes_the_arm_register() {
        let mut l = link("ctl_out 40 b1 0012 1544 070000070002 -> ok\n");
        l.stop_acquisition().unwrap();
        let mut l = link("ctl_out 40 b1 0012 1544 070000070002 -> err device \"stalled\"\n");
        assert!(l.stop_acquisition().is_err());
    }
}
