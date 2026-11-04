// Counterpart in sigrok:
//   libsigrok/src/hardware/sysclk-lwla/lwla.c,
//   libsigrok/src/hardware/sysclk-lwla/lwla.h,
//   libsigrok/src/hardware/sysclk-lwla/lwla1016.c,
//   libsigrok/src/hardware/sysclk-lwla/lwla1034.c,
//   libsigrok/src/hardware/sysclk-lwla/protocol.c,
//   libsigrok/src/hardware/sysclk-lwla/protocol.h,
//   Copyright (C) 2014 Daniel Elstner <daniel.kitta@gmail.com>,
//   Copyright (C) 2015 Daniel Elstner <daniel.kitta@gmail.com>,
//   with commits by Uwe Hermann, Bert Vermeulen, Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The LWLA's wire protocol.
//!
//! An FX2 with its firmware in EEPROM relays 16-bit little endian command
//! words to an FPGA:
//!
//! - endpoint 4 takes the FPGA's bitstream, one bulk OUT behind a 4-byte
//!   big endian length that counts itself;
//! - endpoint 2 takes commands: read register `[1, reg]`, write register
//!   `[2, reg, value]`, read memory `[3 or 6, addr, count]`, write long
//!   registers `[7, first, count, values]`, read long registers
//!   `[8, first, count]`;
//! - endpoint 6 gives the replies.
//!
//! 32-bit values go high half first, in both directions, and a 64-bit
//! value is its low 32 bits and then its high 32 bits, each so.
//!
//! The LWLA1034 has short registers, 64-bit long registers behind a strobe
//! and a 256k x 36-bit memory: each slice of 8 words comes as 9 32-bit
//! words, the last one holding the 8 top nibbles. A word holds 34 channels,
//! bit 34 (one sample more) and bit 35 (a word with the run length
//! follows). The LWLA1016 has short registers only and a 256k x 32-bit
//! memory, read plain (two samples a word) or, with the "-ts" design, as
//! sample and run length.
//!
//! Some FX2s come up limited to 64 bytes per reply (the "short transfer
//! quirk", found at the open), and reads are then cut to fit.
//!
//! A capture is: `setup` while starting, the start sequence, a status poll
//! every 100 ms until the time limit, a full memory or a stop, the capture's
//! length, then the memory read block by block until the sample limit.

use crate::core::{Error, Result};
use crate::usb::{Transport, UsbId, libusb_error_name};
use std::time::Duration;

/// The LWLA1016's USB id.
pub const ID_1016: UsbId = UsbId { vendor: 0x2961, product: 0x6688 };
/// The LWLA1034's USB id.
pub const ID_1034: UsbId = UsbId { vendor: 0x2961, product: 0x6689 };

pub(super) const CONFIGURATION: u8 = 1;
pub(super) const INTERFACE: u8 = 0;
const EP_COMMAND: u8 = 2;
const EP_CONFIG: u8 = 4;
const EP_REPLY: u8 = 0x86;
const USB_TIMEOUT: Duration = Duration::from_millis(1000);
const DRAIN_TIMEOUT: Duration = Duration::from_millis(10);

/// The largest sample limit, a safeguard against overflow in the C.
pub const MAX_LIMIT_SAMPLES: u64 = 1_000_000_000_000;
/// The largest time limit, in ms. The LWLA1016 counts time in 32 bits.
pub const MAX_LIMIT_MSEC: u64 = 1_000_000_000;

const BITSTREAM_MAX_SIZE: u64 = 256 * 1024;
const BITSTREAM_HEADER_SIZE: usize = 4;

/// A logic packet in bytes, a multiple of both unit sizes and of 4 and 5.
pub const PACKET_SIZE: usize = 5000 * 4 * 5;
/// One bulk IN during a capture, two of the FX2's buffers.
const RECV_LEN: usize = 2 * 512;
/// One bulk IN outside a capture, the endpoint's buffer.
const SYNC_RECV_LEN: usize = 512;

const CMD_READ_REG: u16 = 1;
const CMD_WRITE_REG: u16 = 2;
const CMD_READ_MEM32: u16 = 3;
const CMD_READ_MEM36: u16 = 6;
const CMD_WRITE_LREGS: u16 = 7;
const CMD_READ_LREGS: u16 = 8;

/// `STATUS_TRIGGERED` and friends: the LWLA1016's control register bits.
pub(super) const STATUS_CAPTURING: u32 = 1 << 2;
pub(super) const STATUS_TRIGGERED: u32 = 1 << 5;
pub(super) const STATUS_MEM_AVAIL: u32 = 1 << 6;

/// The states a failing transfer's message names (`enum protocol_state`).
pub(super) const STATE_START_CAPTURE: u32 = 2;
pub(super) const STATE_STOP_CAPTURE: u32 = 3;
pub(super) const STATE_READ_PREPARE: u32 = 4;
pub(super) const STATE_READ_FINISH: u32 = 5;
pub(super) const STATE_STATUS_REQUEST: u32 = 8;
pub(super) const STATE_LENGTH_REQUEST: u32 = 9;
pub(super) const STATE_READ_REQUEST: u32 = 10;

/// LWLA1034 short registers.
mod r34 {
    pub const MEM_CTRL: u16 = 0x1074;
    pub const MEM_FILL: u16 = 0x1078;
    pub const MEM_START: u16 = 0x107c;
    pub const CLK_BOOST: u16 = 0x1094;
    pub const LONG_STROBE: u16 = 0x10b0;
    pub const LONG_ADDR: u16 = 0x10b4;
    pub const LONG_LOW: u16 = 0x10b8;
    pub const LONG_HIGH: u16 = 0x10bc;
    pub const MEM_CTRL_WRITE: u32 = 1 << 0;
    pub const MEM_CTRL_CLR_IDX: u32 = 1 << 1;
    pub const LREG_MEM_FILL: u16 = 5;
    pub const LREG_DURATION: usize = 7;
    pub const LREG_STATUS: usize = 9;
    pub const LREG_CAP_CTRL: u32 = 10;
    pub const LREG_TEST_ID: u32 = 100;
    pub const CAP_CTRL_TRG_EN: u32 = 1 << 0;
    pub const CAP_CTRL_CLR_TIMEBASE: u32 = 1 << 2;
    pub const CAP_CTRL_FLUSH_FIFO: u32 = 1 << 4;
    pub const CAP_CTRL_CLR_FIFOFULL: u32 = 1 << 5;
    pub const CAP_CTRL_CLR_COUNTER: u32 = 1 << 6;
    /// The status poll's long registers: `LREG_MEM_FILL` to `LREG_STATUS`.
    pub const READ_LREGS_COUNT: u16 = 5;
    pub const READ_START_ADDR: u64 = 4;
    pub const READ_CHUNK_LEN: u64 = 28 * 8;
    pub const MEMORY_DEPTH: u64 = 256 * 1024;
}

/// LWLA1016 registers.
mod r16 {
    pub const CHAN_MASK: u16 = 0x1000;
    pub const DURATION: u16 = 0x1010;
    pub const MEM_WR_PTR: u16 = 0x1070;
    pub const MEM_CTRL: u16 = 0x107c;
    pub const CAP_COUNT: u16 = 0x10b0;
    /// `REG_TEST_ID` when read, `REG_TRG_SEL` when written.
    pub const TEST_ID: u16 = 0x10b4;
    pub const TRG_SEL: u16 = 0x10b4;
    pub const CAP_CTRL: u16 = 0x10b8;
    pub const DIV_COUNT: u16 = 0x10bc;
    pub const MEM_CTRL_RESET: u32 = 1 << 0;
    pub const MEM_CTRL_WRITE: u32 = 1 << 1;
    pub const CAP_CTRL_FIFO32_FULL: u32 = 1 << 0;
    pub const CAP_CTRL_FIFO64_FULL: u32 = 1 << 1;
    pub const CAP_CTRL_TRG_EN: u32 = 1 << 2;
    pub const CAP_CTRL_FIFO_EMPTY: u32 = 1 << 4;
    pub const READ_START_ADDR: u64 = 2;
    pub const READ_CHUNK_LEN: u64 = 250;
    pub const MEMORY_DEPTH: u64 = 256 * 1024;
}

/// The two boards.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    /// 34 channels, 125 MHz, 36-bit memory.
    Lwla1034,
    /// 16 channels, 100 MHz, 32-bit memory.
    Lwla1016,
}

/// The rates `--show` lists, highest first. The first is the default and
/// the limit, the last the other limit; any rate between goes (the C's
/// `config_set()`).
const RATES_1034: [u64; 20] = [
    125_000_000,
    100_000_000,
    50_000_000,
    20_000_000,
    10_000_000,
    5_000_000,
    2_000_000,
    1_000_000,
    500_000,
    200_000,
    100_000,
    50_000,
    20_000,
    10_000,
    5_000,
    2_000,
    1_000,
    500,
    200,
    100,
];

impl Model {
    /// The model of a USB id.
    pub fn from_id(id: UsbId) -> Option<Model> {
        if id == ID_1034 {
            Some(Model::Lwla1034)
        } else if id == ID_1016 {
            Some(Model::Lwla1016)
        } else {
            None
        }
    }

    /// The name scan shows.
    pub fn name(self) -> &'static str {
        match self {
            Model::Lwla1034 => "LWLA1034",
            Model::Lwla1016 => "LWLA1016",
        }
    }

    /// The number of channels.
    pub fn channels(self) -> usize {
        match self {
            Model::Lwla1034 => 34,
            Model::Lwla1016 => 16,
        }
    }

    /// Bytes per sample in a logic packet.
    pub fn unit_size(self) -> usize {
        self.channels().div_ceil(8)
    }

    /// The listed sample rates.
    pub fn samplerates(self) -> &'static [u64] {
        match self {
            Model::Lwla1034 => &RATES_1034,
            Model::Lwla1016 => &RATES_1034[1..],
        }
    }

    /// The mask of all channels.
    pub fn all_channels(self) -> u64 {
        (1 << self.channels()) - 1
    }
}

/// A signal's edge for the external clock and TRG inputs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Edge {
    /// `r`.
    #[default]
    Positive,
    /// `f`.
    Negative,
}

impl Edge {
    /// The name of the option's value.
    pub fn name(self) -> &'static str {
        match self {
            Edge::Positive => "r",
            Edge::Negative => "f",
        }
    }

    /// The value of an option's name.
    pub fn parse(s: &str) -> Option<Edge> {
        match s {
            "r" => Some(Edge::Positive),
            "f" => Some(Edge::Negative),
            _ => None,
        }
    }
}

/// What decides a capture's registers and which bitstream runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Capture {
    /// The rate asked for, for the limits; see [`divider_1034`].
    pub samplerate: u64,
    /// `limit_time` in ms, 0 for none.
    pub limit_msec: u64,
    /// `limit_samples`, 0 for none.
    pub limit_samples: u64,
    /// The enabled channels.
    pub channel_mask: u64,
    /// Channels in the trigger.
    pub trigger_mask: u64,
    /// Those with an edge.
    pub trigger_edge_mask: u64,
    /// Their levels or slopes.
    pub trigger_values: u64,
    /// LWLA1016: run-length coded memory.
    pub rle: bool,
    /// LWLA1034: runs on the CLK input.
    pub external_clock: bool,
    /// LWLA1034: trigger on the TRG input as well.
    pub trigger_ext: bool,
    /// LWLA1034: the edge of the TRG input.
    pub trigger_slope: Edge,
}

impl Capture {
    /// `clock_boost`: the external clock and rates above 100 MHz run on the
    /// boosted clock.
    pub fn clock_boost(&self) -> bool {
        self.external_clock || self.samplerate > 100_000_000
    }
}

/// The name of the bitstream a board runs: `off` when it is shut down
/// (the LWLA1016 has no such thing, `None`). The first selector for the
/// LWLA1034 is the clock source, the second its edge; for the LWLA1016
/// the run-length coding.
pub fn bitstream_name(
    model: Model,
    active: bool,
    external_clock: bool,
    clock_edge: Edge,
    rle: bool,
) -> Option<&'static str> {
    match model {
        Model::Lwla1034 if !active => Some("sysclk-lwla1034-off.rbf"),
        Model::Lwla1034 if !external_clock => Some("sysclk-lwla1034-int.rbf"),
        Model::Lwla1034 if clock_edge == Edge::Positive => Some("sysclk-lwla1034-extpos.rbf"),
        Model::Lwla1034 => Some("sysclk-lwla1034-extneg.rbf"),
        Model::Lwla1016 if !active => None,
        Model::Lwla1016 if rle => Some("sysclk-lwla1016-100-ts.rbf"),
        Model::Lwla1016 => Some("sysclk-lwla1016-100.rbf"),
    }
}

/// The 16-bit units of a 32-bit value, in the order they go on the wire.
fn words32(value: u32) -> [u16; 2] {
    [(value >> 16) as u16, value as u16]
}

/// The units of a 64-bit value: the low half and then the high one.
fn words64(value: u64) -> [u16; 4] {
    let [a, b] = words32(value as u32);
    let [c, d] = words32((value >> 32) as u32);
    [a, b, c, d]
}

/// A 32-bit value off the wire.
pub fn decode32(b: &[u8]) -> u32 {
    let mut a = [0; 4];
    a.copy_from_slice(&b[..4]);
    u32::from_le_bytes(a).rotate_left(16)
}

fn to_bytes(words: &[u16]) -> Vec<u8> {
    words.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// `[write register, reg, value]`.
pub fn write_reg_command(reg: u16, value: u32) -> Vec<u8> {
    let [a, b] = words32(value);
    to_bytes(&[CMD_WRITE_REG, reg, a, b])
}

/// `[read register, reg]`.
pub fn read_reg_command(reg: u16) -> Vec<u8> {
    to_bytes(&[CMD_READ_REG, reg])
}

/// `[read long registers, first, count]`.
pub fn read_lregs_command(first: u16, count: u16) -> Vec<u8> {
    to_bytes(&[CMD_READ_LREGS, first, count])
}

/// `[read memory, start, count]`.
pub fn read_mem_command(cmd: u16, start: u32, count: u32) -> Vec<u8> {
    let [a, b] = words32(start);
    let [c, d] = words32(count);
    to_bytes(&[cmd, a, b, c, d])
}

/// The clock divider's maximum count on the LWLA1034: an integer division,
/// so that 3 MHz runs at 100 MHz / 33 = 3.03 MHz (`lwla1034.c:390-394`).
/// A boosted clock is undivided: anything above 100 MHz runs at 125 MHz.
pub fn divider_1034(samplerate: u64, clock_boost: bool) -> u64 {
    if samplerate > 0 && samplerate <= 100_000_000 && !clock_boost {
        100_000_000 / samplerate - 1
    } else {
        0
    }
}

/// The same on the LWLA1016 (`lwla1016.c:312-315`).
pub fn divider_1016(samplerate: u64) -> u32 {
    if samplerate > 0 && samplerate < 100_000_000 {
        (100_000_000 / samplerate - 1) as u32
    } else {
        0
    }
}

/// The rate a capture really runs at, where the divider or the boost makes
/// it differ from the rate asked for. `None` on the external clock.
pub fn actual_rate(model: Model, c: &Capture) -> Option<u64> {
    if c.external_clock {
        return None;
    }
    Some(match model {
        Model::Lwla1034 if c.samplerate > 100_000_000 => 125_000_000,
        Model::Lwla1034 => 100_000_000 / (divider_1034(c.samplerate, false) + 1),
        Model::Lwla1016 => 100_000_000 / (u64::from(divider_1016(c.samplerate)) + 1),
    })
}

/// The LWLA1034's bulk write of long registers 0 to 9 in `setup_acquisition`.
pub fn write_lregs_1034(c: &Capture) -> Vec<u8> {
    let mut trigger_mask = c.trigger_mask;
    if c.trigger_ext {
        trigger_mask |= match c.trigger_slope {
            Edge::Positive => 1 << 35,
            Edge::Negative => 1 << 34,
        };
    }
    let values: [u64; 10] = [
        c.channel_mask,
        divider_1034(c.samplerate, c.clock_boost()),
        c.trigger_values,
        c.trigger_edge_mask,
        trigger_mask,
        // A little below the depth, for the pipeline's latency.
        r34::MEMORY_DEPTH - 16,
        0,
        0,
        0,
        0,
    ];
    let mut words = vec![CMD_WRITE_LREGS, 0, 10];
    for v in values {
        words.extend(words64(v));
    }
    to_bytes(&words)
}

/// `(register, value)` pairs written one command each.
pub type Regs = Vec<(u16, u32)>;

fn long_reg(addr: u32, value: u64) -> Regs {
    vec![
        (r34::LONG_ADDR, addr),
        (r34::LONG_LOW, value as u32),
        (r34::LONG_HIGH, (value >> 32) as u32),
        (r34::LONG_STROBE, 0),
    ]
}

/// The start sequence (`STATE_START_CAPTURE`).
pub fn start_regs(model: Model, trigger_mask: u64) -> Regs {
    match model {
        Model::Lwla1034 => long_reg(r34::LREG_CAP_CTRL, u64::from(r34::CAP_CTRL_TRG_EN)),
        Model::Lwla1016 => {
            vec![(r16::CAP_CTRL, r16::CAP_CTRL_TRG_EN | ((trigger_mask & 0xffff) as u32) << 16)]
        }
    }
}

/// The stop sequence (`STATE_STOP_CAPTURE`).
pub fn stop_regs(model: Model) -> Regs {
    match model {
        Model::Lwla1034 => {
            let mut r = long_reg(r34::LREG_CAP_CTRL, 0);
            r.push((r34::CLK_BOOST, 0));
            r
        }
        Model::Lwla1016 => vec![(r16::CAP_CTRL, 0), (r16::DIV_COUNT, 0)],
    }
}

/// The sequence before a readout (`STATE_READ_PREPARE`).
pub fn read_prepare_regs(model: Model) -> Regs {
    match model {
        Model::Lwla1034 => vec![
            (r34::CLK_BOOST, 1),
            (r34::MEM_CTRL, r34::MEM_CTRL_CLR_IDX),
            (r34::MEM_START, r34::READ_START_ADDR as u32),
        ],
        Model::Lwla1016 => vec![(r16::MEM_CTRL, 0)],
    }
}

/// The sequence after a readout (`STATE_READ_FINISH`).
pub fn read_finish_regs(model: Model) -> Regs {
    match model {
        Model::Lwla1034 => vec![(r34::CLK_BOOST, 0)],
        Model::Lwla1016 => vec![(r16::MEM_CTRL, r16::MEM_CTRL_RESET), (r16::MEM_CTRL, 0)],
    }
}

/// The LWLA1034's `capture_init` (`lwla1034.c:357-365`).
fn capture_init_1034() -> Regs {
    let mut r = vec![(r34::MEM_CTRL, r34::MEM_CTRL_CLR_IDX), (r34::MEM_CTRL, r34::MEM_CTRL_WRITE)];
    r.extend(long_reg(
        r34::LREG_CAP_CTRL,
        u64::from(
            r34::CAP_CTRL_CLR_TIMEBASE
                | r34::CAP_CTRL_FLUSH_FIFO
                | r34::CAP_CTRL_CLR_FIFOFULL
                | r34::CAP_CTRL_CLR_COUNTER,
        ),
    ));
    r
}

/// The LWLA1016's `capture_init` (`lwla1016.c:289-299`).
fn capture_init_1016() -> Regs {
    vec![
        (r16::CAP_CTRL, 0),
        (r16::DURATION, 0),
        (r16::MEM_CTRL, r16::MEM_CTRL_RESET),
        (r16::MEM_CTRL, 0),
        (r16::MEM_CTRL, r16::MEM_CTRL_WRITE),
        (r16::CAP_CTRL, r16::CAP_CTRL_FIFO32_FULL | r16::CAP_CTRL_FIFO64_FULL),
        (r16::CAP_CTRL, r16::CAP_CTRL_FIFO_EMPTY),
        (r16::CAP_CTRL, 0),
        (r16::CAP_COUNT, (r16::MEMORY_DEPTH - 5) as u32),
    ]
}

/// The name libusb gives a transfer's status, which the C's asynchronous
/// paths log (`libusb_error_name(transfer->status)`).
pub fn transfer_status_name(e: &Error) -> &'static str {
    match e {
        Error::Timeout => "LIBUSB_TRANSFER_TIMED_OUT",
        Error::Disconnected => "LIBUSB_TRANSFER_NO_DEVICE",
        Error::Device(m) if m.contains("stalled") => "LIBUSB_TRANSFER_STALL",
        _ => "LIBUSB_TRANSFER_ERROR",
    }
}

/// The transport, the board and what the open found out.
pub struct Link {
    transport: Box<dyn Transport>,
    model: Model,
    /// The bitstream running, if known (`active_fpga_config`).
    active: Option<&'static str>,
    /// Replies come in 64 bytes at most.
    pub short_transfer_quirk: bool,
}

impl Link {
    /// A link over an open transport whose interface is claimed.
    pub fn new(transport: Box<dyn Transport>, model: Model) -> Self {
        Link { transport, model, active: None, short_transfer_quirk: false }
    }

    /// The board.
    pub fn model(&self) -> Model {
        self.model
    }

    /// The transport.
    pub fn transport(&mut self) -> &mut dyn Transport {
        self.transport.as_mut()
    }

    /// `drain_usb()`: empties the reply endpoint. A timeout is the end of
    /// it; any other error fails the open.
    pub fn drain(&mut self) -> Result<()> {
        let mut drained = 0;
        loop {
            match self.transport.bulk_in(EP_REPLY, SYNC_RECV_LEN, DRAIN_TIMEOUT) {
                Ok(d) if d.is_empty() => break,
                Ok(d) => drained += d.len(),
                Err(Error::Timeout) => break,
                Err(e) => {
                    log::error!(
                        "Failed to drain USB endpoint {}: {}.",
                        EP_REPLY & 0x7f,
                        libusb_error_name(&e)
                    );
                    return Err(e);
                }
            }
        }
        if drained > 0 {
            log::warn!("Drained {drained} bytes from USB endpoint {}.", EP_REPLY & 0x7f);
        }
        Ok(())
    }

    /// `lwla_send_bitstream()`: the file, checked and sent whole in one
    /// transfer behind its length, then the 30 ms the C found necessary.
    fn send_bitstream(&mut self, name: &str) -> Result<()> {
        let Some(path) = crate::core::firmware::find_file(name) else {
            log::error!("Failed to open resource '{name}' (use loglevel 5/spew for details).");
            return Err(Error::Firmware(name.to_owned()));
        };
        let size = std::fs::metadata(&path)?.len();
        if size == 0 || size > BITSTREAM_MAX_SIZE {
            log::error!("Refusing to load bitstream of unreasonable size ({size} bytes).");
            return Err(Error::Firmware(name.to_owned()));
        }
        let bits = std::fs::read(&path).inspect_err(|_| {
            log::error!("Failed to read bitstream '{name}'.");
        })?;
        let length = bits.len() + BITSTREAM_HEADER_SIZE;
        let mut stream = Vec::with_capacity(length);
        // Fits: the size was checked above.
        stream.extend_from_slice(&(length as u32).to_be_bytes());
        stream.extend_from_slice(&bits);
        log::info!("Downloading FPGA bitstream '{name}'.");
        let sent = self.transport.bulk_out(EP_CONFIG, &stream, USB_TIMEOUT).inspect_err(|e| {
            log::error!("Failed to transfer bitstream: {}.", libusb_error_name(e));
        })?;
        if sent != length {
            log::error!("Failed to transfer bitstream: incorrect length {sent} != {length}.");
            return Err(Error::device("short bitstream transfer"));
        }
        log::info!("FPGA bitstream download of {sent} bytes done.");
        std::thread::sleep(Duration::from_millis(30));
        Ok(())
    }

    /// `apply_fpga_config()`: the bitstream `name` unless it runs already.
    pub fn apply_config(&mut self, name: Option<&'static str>) -> Result<()> {
        let Some(name) = name else { return Ok(()) };
        if self.active == Some(name) {
            return Ok(());
        }
        let ret = self.send_bitstream(name);
        self.active = ret.is_ok().then_some(name);
        ret
    }

    /// `lwla_send_command()`.
    fn send(&mut self, command: &[u8]) -> Result<()> {
        let first = u16::from_le_bytes([command[0], command[1]]);
        match self.transport.bulk_out(EP_COMMAND, command, USB_TIMEOUT) {
            Err(e) => {
                log::debug!("Failed to send command {first}: {}.", libusb_error_name(&e));
                Err(e)
            }
            Ok(n) if n != command.len() => {
                log::debug!(
                    "Failed to send command {first}: incorrect length {n} != {}.",
                    command.len()
                );
                Err(Error::device("short command"))
            }
            Ok(_) => Ok(()),
        }
    }

    /// `lwla_receive_reply()`.
    fn receive(&mut self) -> Result<Vec<u8>> {
        self.transport.bulk_in(EP_REPLY, SYNC_RECV_LEN, USB_TIMEOUT).inspect_err(|e| {
            log::debug!("Failed to receive reply: {}.", libusb_error_name(e));
        })
    }

    /// `lwla_read_reg()`.
    fn read_reg(&mut self, reg: u16) -> Result<u32> {
        self.send(&read_reg_command(reg))?;
        let reply = self.receive()?;
        if reply.len() != 4 {
            log::debug!("Invalid register read response of length {}.", reply.len());
            return Err(Error::device("bad register read response"));
        }
        Ok(decode32(&reply))
    }

    /// `lwla_write_reg()`.
    fn write_reg(&mut self, reg: u16, value: u32) -> Result<()> {
        self.send(&write_reg_command(reg, value))
    }

    /// `lwla_write_regs()`.
    fn write_regs(&mut self, regs: &[(u16, u32)]) -> Result<()> {
        regs.iter().try_for_each(|&(r, v)| self.write_reg(r, v))
    }

    /// The LWLA1034's `read_long_reg()`.
    fn read_long_reg(&mut self, addr: u32) -> Result<u64> {
        self.write_reg(r34::LONG_ADDR, addr)?;
        self.read_reg(r34::LONG_STROBE)?;
        let high = self.read_reg(r34::LONG_HIGH)?;
        let low = self.read_reg(r34::LONG_LOW)?;
        Ok(u64::from(high) << 32 | u64::from(low))
    }

    /// `device_init_check()`: the self test, and the quirk check.
    pub fn init_check(&mut self) -> Result<()> {
        match self.model {
            Model::Lwla1034 => self.init_check_1034(),
            Model::Lwla1016 => self.init_check_1016(),
        }
    }

    fn init_check_1034(&mut self) -> Result<()> {
        // The first read's value, and its errors, are ignored.
        let _ = self.read_long_reg(r34::LREG_TEST_ID);
        let value = self.read_long_reg(r34::LREG_TEST_ID)?;
        if value != 0x1234_5678_8765_4321 {
            log::error!("Received invalid test word 0x{value:016X}.");
            return Err(Error::device("invalid test word"));
        }
        self.detect_short_transfer_quirk()
    }

    /// `detect_short_transfer_quirk()`: a read of 80 bytes, which a limited
    /// FX2 splits at 64.
    fn detect_short_transfer_quirk(&mut self) -> Result<()> {
        const LREG_COUNT: usize = 10;
        self.send(&read_lregs_command(0, LREG_COUNT as u16))?;
        let mut len = self.receive()?.len();
        self.short_transfer_quirk = len == 64;
        if len == 8 * LREG_COUNT {
            return Ok(());
        }
        if len == 64 {
            len = self.receive()?.len();
            if len == 8 * LREG_COUNT - 64 {
                return Ok(());
            }
        }
        log::error!("Received response of unexpected length {len}.");
        Err(Error::device("unexpected response length"))
    }

    fn init_check_1016(&mut self) -> Result<()> {
        const TEST_COUNT: u32 = 24;
        let _ = self.read_reg(r16::TEST_ID);
        let value = self.read_reg(r16::TEST_ID)?;
        if value != 0x1234_5678 {
            log::error!("Received invalid test word 0x{value:08X}.");
            return Err(Error::device("invalid test word"));
        }
        self.write_regs(&[(r16::MEM_CTRL, r16::MEM_CTRL_RESET), (r16::MEM_CTRL, 0)])?;
        self.test_read_memory(0, TEST_COUNT)?;
        // "Issue another read request or the device will stall."
        self.test_read_memory(TEST_COUNT, TEST_COUNT)
    }

    /// `test_read_memory()`: a read that a limited FX2 splits at 64 bytes.
    fn test_read_memory(&mut self, start: u32, count: u32) -> Result<()> {
        self.send(&read_mem_command(CMD_READ_MEM32, start, count))?;
        let mut len = self.receive()?.len();
        self.short_transfer_quirk = len == 64;
        let total = 4 * count as usize;
        let mut got = len;
        while got < total && len == 64 {
            len = self.receive()?.len();
            got += len;
        }
        if got != total {
            log::error!("Invalid read response of unexpected length {len}.");
            return Err(Error::device("unexpected response length"));
        }
        Ok(())
    }

    /// `setup_acquisition()`: everything that goes before the start, its
    /// failures failing the start itself.
    pub fn setup(&mut self, c: &Capture) -> Result<()> {
        match self.model {
            Model::Lwla1034 => {
                self.write_regs(&capture_init_1034())?;
                self.write_reg(r34::CLK_BOOST, u32::from(c.clock_boost()))?;
                self.send(&write_lregs_1034(c))
            }
            Model::Lwla1016 => {
                self.write_reg(r16::CHAN_MASK, c.channel_mask as u32)?;
                self.write_reg(r16::DIV_COUNT, divider_1016(c.samplerate))?;
                self.write_regs(&capture_init_1016())?;
                let trigger =
                    ((c.trigger_edge_mask & 0xffff) << 16 | (c.trigger_values & 0xffff)) as u32;
                self.write_reg(r16::TRG_SEL, trigger)
            }
        }
    }

    /// One write of a capture's sequences, with no check of how much the
    /// device took, as the C's completion callback.
    pub fn send_async(&mut self, command: &[u8]) -> Result<()> {
        self.transport.bulk_out(EP_COMMAND, command, USB_TIMEOUT).map(|_| ())
    }

    /// One read of a capture's replies.
    pub fn receive_async(&mut self) -> Result<Vec<u8>> {
        self.transport.bulk_in(EP_REPLY, RECV_LEN, USB_TIMEOUT)
    }
}

/// What a status poll asks, register by register on the LWLA1016.
pub enum StatusRequest {
    /// LWLA1034: one read of the long registers.
    LongRegs(Vec<u8>),
    /// LWLA1016: the short registers, one read each.
    Regs([u16; 3]),
}

/// The status poll's request.
pub fn status_request(model: Model) -> StatusRequest {
    match model {
        Model::Lwla1034 => {
            StatusRequest::LongRegs(read_lregs_command(r34::LREG_MEM_FILL, r34::READ_LREGS_COUNT))
        }
        Model::Lwla1016 => StatusRequest::Regs([r16::CAP_CTRL, r16::MEM_WR_PTR, r16::DURATION]),
    }
}

/// What a status poll answers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Status {
    /// The status bits at the LWLA1016's positions.
    pub status: u32,
    /// Words in the memory.
    pub mem_fill: u32,
    /// Capture time in ms.
    pub duration: u64,
}

/// The status poll's reply on the LWLA1034, whose length was checked. A
/// boosted clock runs the millisecond counter 25 % fast as well.
pub fn status_1034(reply: &[u8], clock_boost: bool) -> Status {
    let long = |idx: usize| {
        let at = 8 * (idx - usize::from(r34::LREG_MEM_FILL));
        u64::from(decode32(&reply[at + 4..])) << 32 | u64::from(decode32(&reply[at..]))
    };
    let duration = long(r34::LREG_DURATION);
    Status {
        // Shifted left by one to match the LWLA1016's bits.
        status: ((long(r34::LREG_STATUS) & 0x3f) << 1) as u32,
        mem_fill: long(usize::from(r34::LREG_MEM_FILL)) as u32,
        duration: if clock_boost { duration / 5 * 4 + duration % 5 * 4 / 5 } else { duration },
    }
}

/// The status poll's registers on the LWLA1016.
pub fn status_1016(regs: [u32; 3]) -> Status {
    Status { status: regs[0] & 0x7f, mem_fill: regs[1], duration: u64::from(regs[2]) }
}

/// The length register of a model.
pub fn length_reg(model: Model) -> u16 {
    match model {
        Model::Lwla1034 => r34::MEM_FILL,
        Model::Lwla1016 => r16::CAP_COUNT,
    }
}

/// The read's first address and its end, from the length register.
pub fn read_range(model: Model, length: u32) -> (u64, u64) {
    match model {
        Model::Lwla1034 => (r34::READ_START_ADDR, u64::from(length)),
        Model::Lwla1016 => (r16::READ_START_ADDR, u64::from(length) + r16::READ_START_ADDR - 1),
    }
}

/// `read_response()`'s size mismatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WrongSize {
    /// What came.
    pub got: usize,
    /// What the request asks for.
    pub want: usize,
}

/// A capture's memory read: the addresses, the run-length state and the
/// logic packet being filled (`struct acquisition_state`). Addresses are
/// kept in 64 bits, where the C's 32 would wrap on a garbage length.
pub struct Readout {
    model: Model,
    rle: bool,
    short_transfer_quirk: bool,
    samples_max: u64,
    samples_done: u64,
    sample: u64,
    run_len: u64,
    /// LWLA1034: the next word is a run length.
    len_follows: bool,
    mem_addr_done: u64,
    mem_addr_next: u64,
    mem_addr_stop: u64,
    in_index: u64,
    out: Vec<u8>,
}

impl Readout {
    /// A readout of the words from `start` to `stop`, at most `samples_max`
    /// samples.
    pub fn new(
        model: Model,
        rle: bool,
        short_transfer_quirk: bool,
        samples_max: u64,
        (start, stop): (u64, u64),
    ) -> Self {
        Readout {
            model,
            rle,
            short_transfer_quirk,
            samples_max,
            samples_done: 0,
            sample: 0,
            run_len: 0,
            len_follows: false,
            mem_addr_done: start,
            mem_addr_next: start,
            mem_addr_stop: stop,
            in_index: 0,
            out: Vec::new(),
        }
    }

    /// Whether the memory holds nothing to read.
    pub fn is_empty(&self) -> bool {
        self.mem_addr_next >= self.mem_addr_stop
    }

    /// The words to read.
    pub fn words(&self) -> u64 {
        self.mem_addr_stop.saturating_sub(self.mem_addr_next)
    }

    /// Whether another block is due: more words, and samples still wanted.
    pub fn more(&self) -> bool {
        self.samples_done < self.samples_max && self.mem_addr_next < self.mem_addr_stop
    }

    /// `prepare_request()` of `STATE_READ_REQUEST`: the next read's command.
    pub fn next_request(&mut self) -> Vec<u8> {
        let (cmd, count) = match self.model {
            Model::Lwla1034 => {
                let chunk = if self.short_transfer_quirk { 8 } else { r34::READ_CHUNK_LEN };
                // Always a multiple of 8 words: a slice.
                let remaining = (self.mem_addr_stop - self.mem_addr_next).div_ceil(8) * 8;
                (CMD_READ_MEM36, chunk.min(remaining))
            }
            Model::Lwla1016 => {
                let chunk = if self.short_transfer_quirk { 16 } else { r16::READ_CHUNK_LEN };
                (CMD_READ_MEM32, chunk.min(self.mem_addr_stop - self.mem_addr_next))
            }
        };
        let command = read_mem_command(cmd, self.mem_addr_next as u32, count as u32);
        self.mem_addr_next += count;
        command
    }

    /// `handle_read_response()`: all of a reply decoded, each full packet
    /// given to `emit`. `cancelled` is asked before each step.
    pub fn take(
        &mut self,
        reply: &[u8],
        cancelled: &dyn Fn() -> bool,
        emit: &mut dyn FnMut(Vec<u8>),
    ) -> std::result::Result<(), WrongSize> {
        let end = self.mem_addr_next.min(self.mem_addr_stop);
        self.in_index = 0;
        while !cancelled()
            && (self.run_len > 0 || self.mem_addr_done < end)
            && self.samples_done < self.samples_max
        {
            let want = self.expected_len();
            if reply.len() != want {
                return Err(WrongSize { got: reply.len(), want });
            }
            match (self.model, self.rle) {
                (Model::Lwla1034, _) => self.read_response_1034(reply),
                (Model::Lwla1016, true) => self.read_response_1016_rle(reply),
                (Model::Lwla1016, false) => self.read_response_1016(reply),
            }
            if self.out.len() >= PACKET_SIZE {
                emit(std::mem::take(&mut self.out));
            }
        }
        Ok(())
    }

    /// The partly filled packet, if any.
    pub fn flush(&mut self) -> Option<Vec<u8>> {
        (!self.out.is_empty()).then(|| std::mem::take(&mut self.out))
    }

    /// The reply's size, whatever the read has consumed by now.
    fn expected_len(&self) -> usize {
        let words = self.mem_addr_next - self.mem_addr_done + self.in_index;
        let bytes = match self.model {
            Model::Lwla1034 => words.div_ceil(8) * 9 * 4,
            Model::Lwla1016 => words * 4,
        };
        bytes as usize
    }

    /// Samples that fit in the packet and under the limit.
    fn room(&self) -> u64 {
        let free = PACKET_SIZE / self.model.unit_size() - self.out.len() / self.model.unit_size();
        (self.samples_max - self.samples_done).min(free as u64)
    }

    /// The current run put into the packet. True when the packet is full or
    /// the limit reached, which ends this round of decoding.
    fn expand_run(&mut self) -> bool {
        let room = self.room();
        let run = room.min(self.run_len);
        let unit = self.model.unit_size();
        let bytes = self.sample.to_le_bytes();
        for _ in 0..run {
            self.out.extend_from_slice(&bytes[..unit]);
        }
        self.run_len -= run;
        self.samples_done += run;
        run == room
    }

    /// `read_response()` of the LWLA1034: 36-bit words out of their slices.
    fn read_response_1034(&mut self, reply: &[u8]) {
        let words_left = self.mem_addr_next.min(self.mem_addr_stop) - self.mem_addr_done;
        let mut wi = 0;
        loop {
            if self.expand_run() || wi >= words_left {
                break;
            }
            let word = word_1034(reply, self.in_index + wi);
            if self.len_follows {
                self.run_len += word << 1;
                self.len_follows = false;
            } else {
                self.sample = word & Model::Lwla1034.all_channels();
                self.run_len = ((word >> 34) & 1) + 1;
                self.len_follows = word & (1 << 35) != 0;
            }
            wi += 1;
        }
        self.in_index += wi;
        self.mem_addr_done += wi;
    }

    /// `read_response()` of the LWLA1016: two samples a word, the later one
    /// in the word's low half.
    fn read_response_1016(&mut self, reply: &[u8]) {
        let words_left = self.mem_addr_next.min(self.mem_addr_stop) - self.mem_addr_done;
        let run = self.room().min(2 * words_left);
        // Rounded up for an odd limit, whose last sample is cut below.
        let num_words = run.div_ceil(2);
        let from = (self.in_index * 4) as usize;
        for w in reply[from..from + (num_words * 4) as usize].as_chunks::<4>().0 {
            self.out.extend_from_slice(&decode32(w).to_le_bytes());
        }
        self.out.truncate(self.out.len() - ((num_words * 2 - run) * 2) as usize);
        self.in_index += num_words;
        self.mem_addr_done += num_words;
        self.samples_done += run;
    }

    /// `read_response_rle()` of the LWLA1016: sample and run length.
    fn read_response_1016_rle(&mut self, reply: &[u8]) {
        let words_left = self.mem_addr_next.min(self.mem_addr_stop) - self.mem_addr_done;
        let mut wi = 0;
        loop {
            if self.expand_run() || wi >= words_left {
                break;
            }
            let at = ((self.in_index + wi) * 4) as usize;
            let word = u32::from_le_bytes([reply[at], reply[at + 1], reply[at + 2], reply[at + 3]]);
            self.sample = u64::from(word >> 16);
            self.run_len = u64::from(word & 0xffff) + 1;
            wi += 1;
        }
        self.in_index += wi;
        self.mem_addr_done += wi;
    }
}

/// The 36-bit word at `index` of a reply of whole slices.
fn word_1034(reply: &[u8], index: u64) -> u64 {
    let slice = (index / 8 * 9 * 4) as usize;
    let si = index % 8;
    let high_nibbles = u64::from(decode32(&reply[slice + 32..]));
    let word = u64::from(decode32(&reply[slice + si as usize * 4..]));
    word | ((high_nibbles << (4 * si + 4)) & (0xf << 32))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn capture() -> Capture {
        Capture {
            samplerate: 1_000_000,
            limit_msec: 0,
            limit_samples: 0,
            channel_mask: 0x3_ffff_ffff,
            trigger_mask: 0,
            trigger_edge_mask: 0,
            trigger_values: 0,
            rle: true,
            external_clock: false,
            trigger_ext: false,
            trigger_slope: Edge::Positive,
        }
    }

    fn hex(b: &[u8]) -> String {
        use std::fmt::Write as _;
        b.iter().fold(String::new(), |mut s, x| {
            let _ = write!(s, "{x:02x}");
            s
        })
    }

    #[test]
    fn command_words_follow_the_golden_transactions() {
        // From tests/emu/sysclk-lwla/100-hz.tx.
        assert_eq!(hex(&write_reg_command(0x10b4, 100)), "0200b41000006400");
        assert_eq!(hex(&read_reg_command(0x10b0)), "0100b010");
        assert_eq!(hex(&read_lregs_command(0, 10)), "080000000a00");
        assert_eq!(hex(&read_lregs_command(5, 5)), "080005000500");
        assert_eq!(hex(&read_mem_command(CMD_READ_MEM36, 4, 32)), "06000000040000002000");
    }

    #[test]
    fn the_bulk_long_register_write() {
        let mut c = capture();
        c.samplerate = 100;
        c.channel_mask = 7;
        let b = write_lregs_1034(&c);
        assert_eq!(b.len(), 2 * (3 + 40));
        assert_eq!(hex(&b[..6]), "070000000a00");
        // The channels, 999999 = 0xf423f, the memory's threshold.
        assert_eq!(hex(&b[6..14]), "0000070000000000");
        assert_eq!(hex(&b[14..22]), "0f003f4200000000");
        assert_eq!(hex(&b[46..54]), "0300f0ff00000000");
    }

    #[test]
    fn the_external_trigger_edge_sets_its_bit() {
        let mut c = capture();
        c.trigger_ext = true;
        let at = 6 + 8 * 4;
        let b = write_lregs_1034(&c);
        // Bit 35 in the high half: 2^3 at the high half's low unit.
        assert_eq!(hex(&b[at..at + 8]), "0000000000000800");
        c.trigger_slope = Edge::Negative;
        let b = write_lregs_1034(&c);
        assert_eq!(hex(&b[at..at + 8]), "0000000000000400");
    }

    #[test]
    fn dividers_are_integer_divisions() {
        assert_eq!(divider_1034(100, false), 999_999);
        assert_eq!(divider_1034(100_000_000, false), 0);
        assert_eq!(divider_1034(3_000_000, false), 32);
        assert_eq!(divider_1034(60_000_000, false), 0);
        assert_eq!(divider_1034(1_000_000, true), 0);
        assert_eq!(divider_1016(3_000_000), 32);
        assert_eq!(divider_1016(100_000_000), 0);
        let mut c = capture();
        c.samplerate = 3_000_000;
        assert_eq!(actual_rate(Model::Lwla1034, &c), Some(3_030_303));
        c.samplerate = 110_000_000;
        assert_eq!(actual_rate(Model::Lwla1034, &c), Some(125_000_000));
        c.samplerate = 60_000_000;
        assert_eq!(actual_rate(Model::Lwla1016, &c), Some(100_000_000));
        c.external_clock = true;
        assert_eq!(actual_rate(Model::Lwla1034, &c), None);
    }

    #[test]
    fn bitstreams_by_state() {
        use Model::{Lwla1016, Lwla1034};
        let name = bitstream_name;
        assert_eq!(
            name(Lwla1034, false, true, Edge::Negative, true),
            Some("sysclk-lwla1034-off.rbf")
        );
        assert_eq!(
            name(Lwla1034, true, false, Edge::Negative, true),
            Some("sysclk-lwla1034-int.rbf")
        );
        assert_eq!(
            name(Lwla1034, true, true, Edge::Positive, true),
            Some("sysclk-lwla1034-extpos.rbf")
        );
        assert_eq!(
            name(Lwla1034, true, true, Edge::Negative, true),
            Some("sysclk-lwla1034-extneg.rbf")
        );
        assert_eq!(name(Lwla1016, false, false, Edge::Positive, true), None);
        assert_eq!(
            name(Lwla1016, true, false, Edge::Positive, true),
            Some("sysclk-lwla1016-100-ts.rbf")
        );
        assert_eq!(
            name(Lwla1016, true, false, Edge::Positive, false),
            Some("sysclk-lwla1016-100.rbf")
        );
    }

    #[test]
    fn status_replies() {
        // From 100-hz.tx: 14 words, 100 ms, status 0x32 (0x19 shifted).
        let r = [
            0, 0, 14, 0, 0, 0, 0, 0, //
            0, 0, 0, 0, 0, 0, 0, 0, //
            0, 0, 100, 0, 0, 0, 0, 0, //
            0, 0, 0, 0, 0, 0, 0, 0, //
            0, 0, 0x19, 0, 0, 0, 0, 0,
        ];
        let s = status_1034(&r, false);
        assert_eq!(s, Status { status: 0x32, mem_fill: 14, duration: 100 });
        assert_eq!(status_1034(&r, true).duration, 80);
        assert_eq!(status_1016([0xff, 3, 9]), Status { status: 0x7f, mem_fill: 3, duration: 9 });
    }

    #[test]
    fn the_boost_correction_does_not_overflow() {
        let mut r = [0u8; 40];
        // duration = 2^64 - 1 at long register 7.
        r[16..24].fill(0xff);
        // The C's duration * 4 / 5 wraps in 64 bits; this gives the time.
        assert_eq!(status_1034(&r, true).duration, u64::MAX / 5 * 4);
    }

    /// A reply of slices: the 36-bit words `w`, in sets of 8.
    fn slices(w: &[u64]) -> Vec<u8> {
        let mut out = Vec::new();
        for s in w.chunks(8) {
            let mut nibbles = 0u32;
            for (i, v) in s.iter().enumerate() {
                out.extend_from_slice(&((*v as u32).rotate_left(16)).to_le_bytes());
                nibbles |= (((v >> 32) & 0xf) as u32) << (28 - 4 * i);
            }
            for _ in s.len()..8 {
                out.extend_from_slice(&[0; 4]);
            }
            out.extend_from_slice(&nibbles.rotate_left(16).to_le_bytes());
        }
        out
    }

    fn run(r: &mut Readout, reply: &[u8]) -> Vec<Vec<u8>> {
        let mut packets = Vec::new();
        r.take(reply, &|| false, &mut |p| packets.push(p)).unwrap();
        packets.extend(r.flush());
        packets
    }

    fn sample34(v: u64) -> Vec<u8> {
        v.to_le_bytes()[..5].to_vec()
    }

    #[test]
    fn the_1034_decodes_samples_and_runs() {
        let mut r = Readout::new(Model::Lwla1034, true, false, 100, (4, 12));
        assert_eq!(hex(&r.next_request()), "06000000040000000800");
        let words = [
            0x3_0000_0001,
            // One sample more: bit 34.
            (1 << 34) | 2,
            // A run length follows: bit 35, and that word doubled is added.
            (1 << 35) | 3,
            2,
            0,
            0,
            0,
            0,
        ];
        let p = run(&mut r, &slices(&words));
        let want: Vec<u8> = [
            sample34(0x3_0000_0001),
            sample34(2),
            sample34(2),
            sample34(3),
            sample34(3),
            sample34(3),
            sample34(3),
            sample34(3),
            sample34(0),
            sample34(0),
            sample34(0),
            sample34(0),
        ]
        .concat();
        assert_eq!(p, vec![want]);
    }

    #[test]
    fn a_word_past_the_end_is_not_decoded() {
        // Five words of a slice of eight: the reply has the whole slice.
        let mut r = Readout::new(Model::Lwla1034, true, false, 100, (4, 9));
        assert_eq!(hex(&r.next_request()), "06000000040000000800");
        let p = run(&mut r, &slices(&[1, 2, 3, 4, 5, 6, 7, 8]));
        assert_eq!(p[0].len(), 5 * 5);
        assert!(!r.more());
    }

    #[test]
    fn the_limit_cuts_a_run_and_ends_the_reads() {
        let mut r = Readout::new(Model::Lwla1034, true, false, 3, (4, 500));
        r.next_request();
        let mut reply = slices(&[(1 << 35) | 5, 100, 0, 0, 0, 0, 0, 0]);
        reply.resize(1008, 0);
        let p = run(&mut r, &reply);
        assert_eq!(p, vec![[sample34(5), sample34(5), sample34(5)].concat()]);
        assert!(!r.more());
    }

    #[test]
    fn a_wrong_size_is_reported() {
        let mut r = Readout::new(Model::Lwla1034, true, false, 3, (4, 12));
        r.next_request();
        let e = r.take(&[0; 34], &|| false, &mut |_| {}).unwrap_err();
        assert_eq!(e, WrongSize { got: 34, want: 36 });
    }

    #[test]
    fn a_cancel_leaves_the_reply_undecoded() {
        let mut r = Readout::new(Model::Lwla1034, true, false, 3, (4, 12));
        r.next_request();
        r.take(&[0; 36], &|| true, &mut |_| panic!("no packet")).unwrap();
        assert!(r.flush().is_none());
    }

    #[test]
    fn full_packets_go_out_and_the_rest_at_the_end() {
        // One run of 20001 samples: a packet of 20000 and one more.
        let mut r = Readout::new(Model::Lwla1034, true, false, 30000, (4, 12));
        r.next_request();
        // 10000 doubled, plus the 1 of the data word.
        let p = run(&mut r, &slices(&[(1 << 35) | 9, 10000, 0, 0, 0, 0, 0, 0]));
        assert_eq!(p[0].len(), PACKET_SIZE);
        // The rest: 1 of the 20001 and six zero words.
        assert_eq!(p[1].len(), 7 * 5);
    }

    #[test]
    fn the_short_transfer_quirk_cuts_the_reads() {
        let mut r = Readout::new(Model::Lwla1034, true, true, 3, (4, 100));
        assert_eq!(hex(&r.next_request()), "06000000040000000800");
        let mut r = Readout::new(Model::Lwla1016, true, true, 3, (2, 100));
        assert_eq!(hex(&r.next_request()), "03000000020000001000");
    }

    #[test]
    fn the_1016_plain_swaps_the_halves() {
        let mut r = Readout::new(Model::Lwla1016, false, false, 3, (2, 4));
        assert_eq!(hex(&r.next_request()), "03000000020000000200");
        // Word 0x00020001 on the wire LE is 01 00 02 00: first sample 2.
        let p = run(&mut r, &[1, 0, 2, 0, 3, 0, 4, 0]);
        // The limit of 3 cuts the second word's second sample.
        assert_eq!(p, vec![vec![2, 0, 1, 0, 4, 0]]);
    }

    #[test]
    fn the_1016_rle_expands() {
        let mut r = Readout::new(Model::Lwla1016, true, false, 100, (2, 4));
        r.next_request();
        // Sample 5 once (run 0), sample 7 three times (run 2).
        let p = run(&mut r, &[0, 0, 5, 0, 2, 0, 7, 0]);
        assert_eq!(p, vec![vec![5, 0, 7, 0, 7, 0, 7, 0]]);
    }

    #[test]
    fn length_registers() {
        assert_eq!(read_range(Model::Lwla1034, 34), (4, 34));
        assert_eq!(read_range(Model::Lwla1016, 34), (2, 35));
        // No words on the LWLA1016: length 0 gives stop 1, before the start.
        let (a, b) = read_range(Model::Lwla1016, 0);
        assert!(a >= b);
        assert!(Readout::new(Model::Lwla1016, true, false, 1, (a, b)).is_empty());
    }

    #[test]
    fn the_regs_of_the_sequences() {
        assert_eq!(start_regs(Model::Lwla1016, 0xf), vec![(r16::CAP_CTRL, 4 | 0xf << 16)]);
        assert_eq!(start_regs(Model::Lwla1034, 0xf).len(), 4);
        assert_eq!(stop_regs(Model::Lwla1034).len(), 5);
        assert_eq!(stop_regs(Model::Lwla1016).len(), 2);
        assert_eq!(read_prepare_regs(Model::Lwla1034).len(), 3);
        assert_eq!(read_finish_regs(Model::Lwla1016).len(), 2);
    }

    #[test]
    fn transfer_names() {
        assert_eq!(transfer_status_name(&Error::Timeout), "LIBUSB_TRANSFER_TIMED_OUT");
        assert_eq!(
            transfer_status_name(&Error::device("endpoint stalled")),
            "LIBUSB_TRANSFER_STALL"
        );
        assert_eq!(transfer_status_name(&Error::Disconnected), "LIBUSB_TRANSFER_NO_DEVICE");
    }
}
