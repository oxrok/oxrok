// Counterpart in sigrok:
//   libsigrok/src/hardware/sysclk-sla5032/protocol.c,
//   libsigrok/src/hardware/sysclk-sla5032/protocol.h,
//   Copyright (C) 2019 Vitaliy Vorobyov,
//   with commits by Uwe Hermann, Vitaliy Vorobyov
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The SLA5032's wire protocol: an FX2LP relaying commands to a Spartan-6
//! FPGA that holds its bitstream until the power goes.
//!
//! Commands go out on endpoint 4, one per transfer; replies come on
//! endpoint 8 and samples on endpoint 6:
//!
//! | bytes | command | reply |
//! |---|---|---|
//! | `01` | start the bitstream upload | none |
//! | `02 LEN DATA` | a bitstream chunk, up to 250 bytes | none |
//! | `03 ADDR 04` | read register `ADDR / 4` | the value, big endian |
//! | `04 ADDR 04 VALUE` | write register `ADDR / 4`, big endian | none |
//! | `05 HI LO LEN` | read the EEPROM | `LEN` bytes |
//! | `07 03` | prepare a sample chunk | none |
//!
//! Registers: 0 to 2 the trigger (levels or slopes, edge or level, mask;
//! 2 reads as the status, bit 1 triggered, bit 3 done), 1 reads the inputs,
//! 0 the samples after the trigger, 3 the chunk's size, 4 the PLL divider
//! less 1, 5 control bits (bit 0 PLL x2, bit 7 clear for PLL x1.25), 6 and
//! 7 the post- and pre-trigger depths as `(x + 1) * 256` samples, 9 to 12
//! the two PWM generators, 14 sampling, 16 and 17 the keys from EEPROM
//! address 4000.
//!
//! Samples are 6 byte records, a little endian 32-bit value and a little
//! endian 16-bit count less 1, up to `0x100000` in a chunk; a count of
//! `FFFF` ends the capture. [`Decoder`] expands them in blocks of
//! [`BLOCK_SAMPLES`], so nothing is allocated from what the records claim:
//! one chunk can claim 2^36 samples. The C sums them into an `int` that
//! sizes its buffer (`protocol.c:782,795`).

use crate::core::{Error, Result, SoftTriggerLogic};
use crate::usb::{Transport, libusb_error_name};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// The vendor id (`protocol.h`).
pub const USB_VID_SYSCLK: u16 = 0x2961;
/// The product id (`protocol.h`).
pub const USB_PID_SLA5032: u16 = 0x66b0;
/// The configuration the open sets.
pub const USB_CONFIG: u8 = 1;
/// The interface the open claims.
pub const USB_INTERFACE: u8 = 0;

/// The most samples `limit_samples` takes.
pub const MAX_LIMIT_SAMPLES: u64 = 64 * 1024 * 1024;
/// The fewest samples `limit_samples` takes.
pub const MIN_LIMIT_SAMPLES: u64 = 512;

const EP_COMMAND: u8 = 0x04;
const EP_REPLY: u8 = 0x88;
const EP_DATA: u8 = 0x86;

const USB_CMD_TIMEOUT: Duration = Duration::from_secs(5);
const USB_REPLY_TIMEOUT: Duration = Duration::from_secs(500);
const USB_DATA_TIMEOUT: Duration = Duration::from_secs(2);
/// How long a reply is waited for before looking at the stop flag again:
/// `USB_REPLY_TIMEOUT` is too long for a stop to wait for.
const REPLY_SLICE: Duration = Duration::from_millis(100);

const CMD_INIT_FW_UPLOAD: u8 = 1;
const CMD_UPLOAD_FW_CHUNK: u8 = 2;
const CMD_READ_REG: u8 = 3;
const CMD_WRITE_REG: u8 = 4;
const CMD_READ_MEM: u8 = 5;
const CMD_READ_DATA: u8 = 7;

/// The bitstream's file name.
pub const BITSTREAM_NAME: &str = "sysclk-sla5032.bit";
const BITSTREAM_MAX_SIZE: u64 = 512 * 1024;
const BITSTREAM_HEADER_SIZE: usize = 0x69;
const FW_CHUNK_SIZE: usize = 250;
const XILINX_SYNC_WORD: [u8; 4] = [0xaa, 0x99, 0x55, 0x66];
/// The EEPROM address of the keys.
const KEY_ADDRESS: u16 = 4000;

/// A sample on the wire and in a logic packet.
pub const UNIT_SIZE: usize = 4;
/// A record's bytes.
pub const RLE_SAMPLE_SIZE: usize = 6;
/// The records of a full chunk: another one follows.
pub const RLE_SAMPLES_COUNT: usize = 0x10_0000;
const RLE_BUF_SIZE: usize = RLE_SAMPLES_COUNT * RLE_SAMPLE_SIZE;
const RLE_END_MARKER: u16 = 0xffff;
/// The samples of one logic packet at most.
pub const BLOCK_SAMPLES: usize = 1 << 18;

/// `pll_tbl`: the rate, the divider less 1, PLL x2, PLL x1.25.
const PLL_TABLE: [(u64, u32, bool, bool); 20] = [
    (500_000_000, 0, true, true),
    (400_000_000, 0, true, false),
    (250_000_000, 0, false, true),
    (200_000_000, 0, false, false),
    (100_000_000, 1, false, false),
    (50_000_000, 3, false, false),
    (25_000_000, 7, false, false),
    (20_000_000, 9, false, false),
    (10_000_000, 19, false, false),
    (5_000_000, 39, false, false),
    (2_000_000, 99, false, false),
    (1_000_000, 199, false, false),
    (500_000, 399, false, false),
    (200_000, 999, false, false),
    (100_000, 1999, false, false),
    (50_000, 3999, false, false),
    (20_000, 9999, false, false),
    (10_000, 19999, false, false),
    (5_000, 39999, false, false),
    (2_000, 99999, false, false),
];

/// The rates, fastest first (`samplerates` in `api.c`).
pub const SAMPLERATES: [u64; 20] = {
    let mut rates = [0; 20];
    let mut i = 0;
    while i < 20 {
        rates[i] = PLL_TABLE[i].0;
        i += 1;
    }
    rates
};

/// `sla5032_start_sample()`: what register 5's bits go through.
const START_BITS: [(u32, bool); 10] = [
    (2, false),
    (3, false),
    (5, false),
    (6, true),
    (1, true),
    (1, false),
    (8, true),
    (8, false),
    (6, true),
    (2, true),
];

/// `pwm1_hi`, `pwm1_lo`, `pwm2_hi`, `pwm2_lo`.
const PWM: [u32; 4] = [20_000_000 - 1, 200_000 - 1, 15 - 1, 5 - 1];

/// C's `ret` of a failed step, for its log line: `SR_ERR` for a command
/// not taken and a wrong length, libusb's code for a failed read.
pub fn ret_code(e: &Error) -> i32 {
    match libusb_error_name(e) {
        "LIBUSB_ERROR_ACCESS" => -3,
        "LIBUSB_ERROR_NO_DEVICE" => -4,
        "LIBUSB_ERROR_BUSY" => -6,
        "LIBUSB_ERROR_TIMEOUT" => -7,
        "LIBUSB_ERROR_PIPE" => -9,
        "LIBUSB_ERROR_NOT_SUPPORTED" => -12,
        _ => -1,
    }
}

/// The depths of a capture (`sla5032_start_acquisition()`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Depths {
    /// Samples before the trigger: the soft trigger's ring.
    pub pre: u64,
    /// Samples from the trigger on.
    pub post: u64,
    /// Register 7: the pre-trigger depth.
    pub pre_reg: u64,
    /// Register 6: the post-trigger depth.
    pub post_reg: u64,
}

/// The depths for a limit and a ratio: whole units of 256 samples and two
/// units at least, so a limit of 512 gives 1024 samples and one of 5000
/// gives 4608. The excess is cut on the host, by [`Decoder`]; a shortfall
/// is what the device sent, and the command line says so.
pub fn depths(limit: u64, ratio: u64) -> Depths {
    let pre = limit.wrapping_mul(ratio) / 100;
    let post = limit.wrapping_sub(pre);
    Depths { pre, post, pre_reg: (pre / 256).max(2) - 1, post_reg: (post / 256).max(2) - 1 }
}

/// A register read's command.
pub fn read_reg_command(reg: u8) -> [u8; 3] {
    [CMD_READ_REG, reg.wrapping_mul(4), 4]
}

/// A register write's command.
pub fn write_reg_command(reg: u8, value: u32) -> [u8; 7] {
    let v = value.to_be_bytes();
    [CMD_WRITE_REG, reg.wrapping_mul(4), 4, v[0], v[1], v[2], v[3]]
}

/// `load_bitstream()`'s check and framing on a file's contents: 256 bytes
/// of `FF` and what follows the header. The C reads the sync word past the
/// end of a file of 106 to 108 bytes (`protocol.c:532,555`).
pub fn frame_bitstream(file: &[u8]) -> Option<Vec<u8>> {
    let sync = file.get(BITSTREAM_HEADER_SIZE..BITSTREAM_HEADER_SIZE + XILINX_SYNC_WORD.len());
    if sync != Some(XILINX_SYNC_WORD.as_slice()) {
        log::error!("Invalid bitstream signature.");
        return None;
    }
    let mut out = vec![0xff; 0x100];
    out.extend_from_slice(&file[BITSTREAM_HEADER_SIZE..]);
    Some(out)
}

/// `load_bitstream()`: the file, framed.
fn load_bitstream(name: &str) -> Result<Vec<u8>> {
    let refused = || Error::Firmware(name.to_owned());
    let Some(path) = crate::core::firmware::find_file(name) else {
        log::error!("Failed to open resource '{name}' (use loglevel 5/spew for details).");
        return Err(refused());
    };
    let size = std::fs::metadata(&path)?.len();
    if size <= BITSTREAM_HEADER_SIZE as u64 || size > BITSTREAM_MAX_SIZE {
        log::error!("Refusing to load bitstream of unreasonable size ({size} bytes).");
        return Err(refused());
    }
    let file = std::fs::read(&path).inspect_err(|_| {
        log::error!("Failed to read bitstream '{name}'.");
    })?;
    frame_bitstream(&file).ok_or_else(refused)
}

/// What a chunk's records say: how many before the marker, and the samples
/// they claim, in 64 bits (a chunk can claim 2^36; `protocol.c:782`).
pub fn count_records(data: &[u8]) -> (usize, u64) {
    let mut samples = 0;
    for (i, r) in data.as_chunks::<RLE_SAMPLE_SIZE>().0.iter().enumerate() {
        let count = u16::from_le_bytes([r[4], r[5]]);
        if count == RLE_END_MARKER {
            return (i, samples);
        }
        samples += u64::from(count) + 1;
    }
    (data.len() / RLE_SAMPLE_SIZE, samples)
}

/// What [`Decoder`] gives out, in order.
#[derive(Debug, PartialEq, Eq)]
pub enum Output {
    /// Samples.
    Samples(Vec<u8>),
    /// The trigger: the next sample is its point.
    Trigger,
}

/// One capture's samples on their way out: the soft trigger until it fires,
/// then the limit, which is honoured here since the depths cannot do it. The
/// C sends all the memory holds and `sigrok-cli` writes it whole: it counts
/// the limit only for its decoders and its "Device only sent" (`session.c:464`).
pub struct Decoder {
    /// The trigger, until it has fired.
    stl: Option<SoftTriggerLogic>,
    /// Samples the limit still lets out.
    left: u64,
}

impl Decoder {
    /// A decoder for a capture of `limit` samples.
    pub fn new(stl: Option<SoftTriggerLogic>, limit: u64) -> Self {
        Decoder { stl, left: limit }
    }

    /// Whether the limit's samples are all out.
    pub fn limit_met(&self) -> bool {
        self.left == 0
    }

    /// Passes samples on, cut to what the limit lets out.
    fn cut(&mut self, mut data: Vec<u8>, emit: &mut dyn FnMut(Output)) {
        let room = self.left.saturating_mul(UNIT_SIZE as u64);
        if data.len() as u64 > room {
            data.truncate(usize::try_from(room).unwrap_or(usize::MAX));
        }
        self.left -= (data.len() / UNIT_SIZE) as u64;
        if !data.is_empty() {
            emit(Output::Samples(data));
        }
    }

    /// One block: through the soft trigger until it fires (which sends its
    /// ring first), then out.
    fn block(&mut self, data: Vec<u8>, emit: &mut dyn FnMut(Output)) {
        let Some(stl) = self.stl.as_mut() else {
            self.cut(data, emit);
            return;
        };
        let Some((offset, pre)) = stl.check(&data) else { return };
        self.stl = None;
        for p in pre {
            self.cut(p, emit);
        }
        emit(Output::Trigger);
        self.cut(data[offset * UNIT_SIZE..].to_vec(), emit);
    }

    /// The samples of the first `records` records of a chunk, a block at a
    /// time. Past the limit nothing is expanded. `stopped` is asked between
    /// blocks: records can claim far more than a search should go on with
    /// once it was stopped.
    pub fn records(
        &mut self,
        data: &[u8],
        records: usize,
        stopped: &dyn Fn() -> bool,
        emit: &mut dyn FnMut(Output),
    ) {
        let mut block = Vec::new();
        for r in data.as_chunks::<RLE_SAMPLE_SIZE>().0.iter().take(records) {
            let mut run = usize::from(u16::from_le_bytes([r[4], r[5]])) + 1;
            while run > 0 {
                let held = block.len() / UNIT_SIZE;
                let mut n = run.min(BLOCK_SAMPLES - held);
                if self.stl.is_none() {
                    let wanted = self.left.saturating_sub(held as u64);
                    n = n.min(usize::try_from(wanted).unwrap_or(usize::MAX));
                    if n == 0 {
                        self.block(block, emit);
                        return;
                    }
                }
                for _ in 0..n {
                    block.extend_from_slice(&r[..UNIT_SIZE]);
                }
                run -= n;
                if block.len() == BLOCK_SAMPLES * UNIT_SIZE {
                    self.block(std::mem::take(&mut block), emit);
                    if stopped() {
                        return;
                    }
                }
            }
        }
        if !block.is_empty() {
            self.block(block, emit);
        }
    }
}

/// The transport and how far the FPGA is.
pub struct Link {
    transport: Box<dyn Transport>,
    /// `active_fpga_config`: the design runs.
    configured: bool,
    /// A stop was asked for, which ends a wait for a reply.
    stop: Arc<AtomicBool>,
}

impl Link {
    /// A link over an open transport whose interface is claimed.
    pub fn new(transport: Box<dyn Transport>, stop: Arc<AtomicBool>) -> Self {
        Link { transport, configured: false, stop }
    }

    /// The transport.
    pub fn transport(&mut self) -> &mut dyn Transport {
        self.transport.as_mut()
    }

    /// `la_write_cmd_buf()`.
    fn send(&mut self, command: &[u8]) -> Result<()> {
        let cmd = command[0];
        let n = self.transport.bulk_out(EP_COMMAND, command, USB_CMD_TIMEOUT).map_err(|e| {
            let name = libusb_error_name(&e);
            log::debug!("Failed to send command {cmd}: {name}.");
            Error::Device(format!("command {cmd} failed: {name}"))
        })?;
        if n != command.len() {
            log::debug!("Invalid send command response of length {n}.");
            return Err(Error::Device(format!("command {cmd} taken in part")));
        }
        Ok(())
    }

    /// A reply, for up to `USB_REPLY_TIMEOUT`, in slices so that a stop
    /// does not wait for it. A slice cancels a transfer only when the
    /// device has not answered for that long.
    fn reply(&mut self, len: usize) -> Result<Vec<u8>> {
        let start = Instant::now();
        loop {
            let left = USB_REPLY_TIMEOUT.saturating_sub(start.elapsed());
            let wait = left.min(REPLY_SLICE).max(Duration::from_millis(1));
            match self.transport.bulk_in(EP_REPLY, len, wait) {
                Err(Error::Timeout) if !left.is_zero() && !self.stop.load(Ordering::SeqCst) => {}
                other => return other,
            }
        }
    }

    /// `la_read_reg()`.
    pub fn read_reg(&mut self, reg: u8) -> Result<u32> {
        self.send(&read_reg_command(reg))?;
        let reply = self.reply(4)?;
        let Ok(value) = <[u8; 4]>::try_from(reply.as_slice()) else {
            log::debug!("Invalid register read response of length {}.", reply.len());
            return Err(Error::device("short register read"));
        };
        Ok(u32::from_be_bytes(value))
    }

    /// `la_write_reg()`.
    pub fn write_reg(&mut self, reg: u8, value: u32) -> Result<()> {
        self.send(&write_reg_command(reg, value))
    }

    /// `la_set_res_reg_bit()`.
    fn set_reg_bit(&mut self, reg: u8, bit: u32, set: bool) -> Result<()> {
        let v = self.read_reg(reg)?;
        let v = if set { v | 1 << bit } else { v & !(1 << bit) };
        self.write_reg(reg, v)
    }

    /// `la_read_mem()`.
    fn read_mem(&mut self, addr: u16, len: u8) -> Result<Vec<u8>> {
        let [hi, lo] = addr.to_be_bytes();
        self.send(&[CMD_READ_MEM, hi, lo, len])?;
        let reply = self.reply(usize::from(len));
        let got = reply.as_ref().map_or(0, Vec::len);
        if got != usize::from(len) {
            log::debug!("Invalid memory read response of length {got}.");
            return Err(Error::device("short memory read"));
        }
        reply
    }

    /// `la_cfg_fpga_done()`: the keys from the EEPROM into the FPGA.
    fn enter_keys(&mut self) -> Result<()> {
        let key = self.read_mem(KEY_ADDRESS, 8)?;
        let k0 = u32::from_le_bytes([key[0], key[1], key[2], key[3]]);
        let k1 = u32::from_le_bytes([key[4], key[5], key[6], key[7]]);
        log::debug!("cfg fpga done, k0: {k0:08X}, k1: {k1:08X}.");
        self.write_reg(16, k0)?;
        self.write_reg(17, k1)?;
        let reg2 = self.read_reg(2);
        log::debug!("cfg fpga done, reg2: {:08X}.", reg2.as_ref().map_or(0, |v| *v));
        reg2.map(|_| ())
    }

    /// `sla5032_send_bitstream()`.
    fn send_bitstream(&mut self, stream: &[u8]) -> Result<()> {
        log::debug!("Downloading FPGA bitstream '{BITSTREAM_NAME}'.");
        // Only logged: the C goes on whatever it says.
        let reg2 = self.read_reg(2).unwrap_or(0);
        log::debug!("send bitstream, reg2: {reg2:08X}.");
        self.send(&[CMD_INIT_FW_UPLOAD])?;
        for chunk in stream.chunks(FW_CHUNK_SIZE) {
            let mut command = Vec::with_capacity(chunk.len() + 2);
            command.push(CMD_UPLOAD_FW_CHUNK);
            // At most FW_CHUNK_SIZE.
            command.push(chunk.len() as u8);
            command.extend_from_slice(chunk);
            self.send(&command)?;
        }
        // The C ignores this result (protocol.c:646) and the open succeeds.
        // It covers the read of the keys, their writes and the read of
        // register 2 after them, so it does not say the keys are missing.
        // Whether a design without its keys samples at all nothing shows, so
        // the open stays good and the user gets a warning.
        if let Err(e) = self.enter_keys() {
            log::warn!("The FPGA's key step failed: {e}.");
        }
        // Ignored in the C as well; a failure is in the debug log.
        let _ = self.reg14_zero();
        log::debug!("FPGA bitstream download of {} bytes done.", stream.len());
        Ok(())
    }

    /// `sla5032_apply_fpga_config()`: the bitstream, unless the design runs.
    pub fn apply_fpga_config(&mut self) -> Result<()> {
        self.apply_fpga_config_with(load_bitstream)
    }

    /// [`Link::apply_fpga_config`], with where the bitstream comes from.
    fn apply_fpga_config_with(&mut self, load: fn(&str) -> Result<Vec<u8>>) -> Result<()> {
        if self.configured {
            return Ok(());
        }
        let reg2 = self.read_reg(2)?;
        if reg2 & 0xffff_fff1 == 0xa5a5_a5a1 {
            self.configured = true;
            return Ok(());
        }
        log::debug!("FPGA not configured, send bitstream.");
        let stream = load(BITSTREAM_NAME)?;
        self.send_bitstream(&stream)?;
        self.configured = true;
        Ok(())
    }

    /// `sla5032_set_depth()`. The registers are 32 bits wide.
    pub fn set_depth(&mut self, d: &Depths) -> Result<()> {
        self.write_reg(7, d.pre_reg as u32)?;
        self.write_reg(6, d.post_reg as u32)
    }

    /// `sla5032_set_triggers()`.
    pub fn set_triggers(&mut self, values: u32, edges: u32, mask: u32) -> Result<()> {
        log::debug!("set trigger: val: {values:08X}, e_mask: {edges:08X}, mask: {mask:08X}.");
        self.write_reg(0, values)?;
        self.write_reg(1, edges)?;
        self.write_reg(2, mask)
    }

    /// `sla5032_set_samplerate()`.
    pub fn set_samplerate(&mut self, rate: u64) -> Result<()> {
        let Some(&(rate, div, x2, x1_25)) = PLL_TABLE.iter().find(|e| e.0 == rate) else {
            return Err(Error::arg("samplerate"));
        };
        log::debug!("set sample rate: {rate}.");
        self.write_reg(4, div)?;
        self.set_reg_bit(5, 0, x2)?;
        self.set_reg_bit(5, 7, !x1_25)
    }

    /// `sla5032_set_pwm1()` and `sla5032_set_pwm2()`.
    pub fn set_pwm(&mut self) -> Result<()> {
        for (reg, value) in (9..).zip(PWM) {
            self.write_reg(reg, value)?;
        }
        Ok(())
    }

    /// `sla5032_start_sample()`.
    pub fn start_sample(&mut self) -> Result<()> {
        self.write_reg(14, 1)?;
        for (bit, set) in START_BITS {
            self.set_reg_bit(5, bit, set)?;
        }
        Ok(())
    }

    /// `sla5032_get_status()`: register 0, and whether the capture is done.
    pub fn poll(&mut self) -> Result<(u32, bool)> {
        self.read_reg(1)?;
        let captured = self.read_reg(0)?;
        let status = self.read_reg(2)?;
        let done = status & 8 != 0;
        if done {
            log::debug!("get status, reg2: {status:08X}.");
        }
        Ok((captured, done))
    }

    /// `sla5032_set_read_back()`.
    pub fn set_read_back(&mut self) -> Result<()> {
        self.write_reg(5, 0x08)?;
        self.write_reg(5, 0x28)
    }

    /// `sla5032_read_data_chunk()`. A timeout gives what arrived as a short
    /// read ([`Transport::bulk_in`]): a chunk that ends on a packet boundary
    /// with no zero-length packet ends so, and is kept. The C checks `ret`
    /// before `xfer_len` and drops it (`protocol.c:749`), with exit 0.
    pub fn read_chunk(&mut self) -> Result<Vec<u8>> {
        self.send(&[CMD_READ_DATA, 3])?;
        self.write_reg(3, 0x30_0000)?;
        self.set_reg_bit(5, 4, false)?;
        self.set_reg_bit(5, 4, true)?;
        self.transport.bulk_in(EP_DATA, RLE_BUF_SIZE, USB_DATA_TIMEOUT)
    }

    /// `sla5032_write_reg14_zero()`.
    pub fn reg14_zero(&mut self) -> Result<()> {
        self.write_reg(14, 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::TriggerMatch;
    use crate::usb::record::Replay;
    use std::cell::Cell;

    fn record(value: u32, count: u16) -> Vec<u8> {
        let mut r = value.to_le_bytes().to_vec();
        r.extend_from_slice(&count.to_le_bytes());
        r
    }

    fn replay(steps: &[&str]) -> Replay {
        let mut text = String::from(
            "# oxrok usb fixture 1\ndevice vid=2961 pid=66b0 version=0000 interfaces=0:ff:00:00\n",
        );
        for s in steps {
            text.push_str(s);
            text.push('\n');
        }
        Replay::from_text(&text).unwrap()
    }

    fn link(r: &Replay) -> Link {
        Link::new(Box::new(r.clone()), Arc::new(AtomicBool::new(false)))
    }

    fn no_load(_: &str) -> Result<Vec<u8>> {
        panic!("a bitstream was loaded");
    }

    #[allow(clippy::unnecessary_wraps)] // Must be a loader.
    fn small_stream(_: &str) -> Result<Vec<u8>> {
        Ok(vec![0xff; 251])
    }

    /// The upload of [`small_stream`], up to the keys.
    fn upload_steps() -> Vec<String> {
        [
            "bulk_out 04 030804 -> 3",
            "bulk_in 88 4 -> b6ff9c97",
            "bulk_out 04 030804 -> 3",
            "bulk_in 88 4 -> 12ff9c97",
            "bulk_out 04 01 -> 1",
            &format!("bulk_out 04 02fa{} -> 252", "ff".repeat(250)),
            "bulk_out 04 0201ff -> 3",
        ]
        .map(str::to_owned)
        .to_vec()
    }

    #[test]
    fn depths_round_both_ways() {
        // Both have a minimum of two units: 512 asks for 1024.
        let d = depths(512, 50);
        assert_eq!((d.pre, d.post, d.pre_reg, d.post_reg), (256, 256, 1, 1));
        // Whole units downwards: 5000 asks for 4608.
        let d = depths(5000, 50);
        assert_eq!((d.pre, d.post, d.pre_reg, d.post_reg), (2500, 2500, 8, 8));
        assert_eq!((d.pre_reg + 1 + d.post_reg + 1) * 256, 4608);
        // The corners: all of memory and a minimum on top (against
        // protocol.c's comment; a device cannot refuse it).
        let d = depths(MAX_LIMIT_SAMPLES, 0);
        assert_eq!((d.pre_reg, d.post_reg), (1, MAX_LIMIT_SAMPLES / 256 - 1));
        let d = depths(MAX_LIMIT_SAMPLES, 100);
        assert_eq!((d.pre_reg, d.post_reg), (MAX_LIMIT_SAMPLES / 256 - 1, 1));
    }

    #[test]
    fn depths_wrap_as_the_c_does_on_any_ratio() {
        // No panic in a debug build: the option refuses such a ratio, and
        // the C's arithmetic is unsigned.
        let d = depths(512, 200);
        assert_eq!(d.pre, 1024);
        assert_eq!(d.post, 512u64.wrapping_sub(1024));
        assert_eq!(depths(u64::MAX, u64::MAX).pre, 0);
    }

    #[test]
    fn commands() {
        assert_eq!(read_reg_command(2), [3, 8, 4]);
        assert_eq!(write_reg_command(14, 1), [4, 0x38, 4, 0, 0, 0, 1]);
        assert_eq!(write_reg_command(3, 0x30_0000), [4, 0x0c, 4, 0, 0x30, 0, 0]);
    }

    #[test]
    fn the_bitstream_is_framed_and_checked() {
        let mut file = vec![0; BITSTREAM_HEADER_SIZE];
        file.extend_from_slice(&XILINX_SYNC_WORD);
        file.push(7);
        let out = frame_bitstream(&file).unwrap();
        assert_eq!(out.len(), 0x100 + 5);
        assert_eq!(out[0xff], 0xff);
        assert_eq!(&out[0x100..], &[0xaa, 0x99, 0x55, 0x66, 7]);
        // Wrong, and a file too short to hold it (the C reads past its end).
        file[BITSTREAM_HEADER_SIZE] = 0;
        assert!(frame_bitstream(&file).is_none());
        assert!(frame_bitstream(&[0; BITSTREAM_HEADER_SIZE + 2]).is_none());
    }

    #[test]
    fn counting_stops_at_the_marker_and_does_not_overflow() {
        let mut data = record(1, 0);
        data.extend(record(2, 9));
        data.extend(record(0, RLE_END_MARKER));
        data.extend(record(3, 5));
        assert_eq!(count_records(&data), (2, 11));
        // A tail shorter than a record is no record.
        assert_eq!(count_records(&data[..7]), (1, 1));
        assert_eq!(count_records(&[]), (0, 0));
        // 65537 records of 65535 samples and one of 2: protocol.c:782's
        // int wraps to 1 here, and the C writes about 17 GB into 4 bytes.
        let mut data = Vec::new();
        for _ in 0..65537 {
            data.extend(record(0, 65534));
        }
        data.extend(record(0, 1));
        assert_eq!(count_records(&data), (65538, 65537 * 65535 + 2));
    }

    fn run(mut d: Decoder, data: &[u8]) -> Vec<Output> {
        let (n, _) = count_records(data);
        let mut out = Vec::new();
        d.records(data, n, &|| false, &mut |o| out.push(o));
        out
    }

    fn samples(out: &[Output]) -> Vec<u32> {
        out.iter()
            .filter_map(|o| match o {
                Output::Samples(s) => Some(s),
                Output::Trigger => None,
            })
            .flat_map(|s| s.as_chunks::<4>().0)
            .map(|s| u32::from_le_bytes(*s))
            .collect()
    }

    #[test]
    fn records_expand_and_the_limit_cuts() {
        let mut data = record(5, 2);
        data.extend(record(6, 0));
        data.extend(record(7, 3));
        assert_eq!(samples(&run(Decoder::new(None, 100), &data)), [5, 5, 5, 6, 7, 7, 7, 7]);
        // A limit in a run, and exactly at a run's end.
        assert_eq!(samples(&run(Decoder::new(None, 5), &data)), [5, 5, 5, 6, 7]);
        assert_eq!(samples(&run(Decoder::new(None, 4), &data)), [5, 5, 5, 6]);
    }

    #[test]
    fn the_limit_holds_over_chunks() {
        let data = record(5, 9);
        let mut d = Decoder::new(None, 15);
        let mut out = Vec::new();
        for _ in 0..3 {
            d.records(&data, 1, &|| false, &mut |o| out.push(o));
        }
        assert_eq!(samples(&out).len(), 15);
    }

    #[test]
    fn what_records_claim_allocates_nothing() {
        // 64 Ki records of 65535 samples claim about 2^32; the limit lets
        // out 1000 and nothing else is expanded.
        let mut data = Vec::new();
        for _ in 0..0x1_0000 {
            data.extend(record(9, 65534));
        }
        let mut d = Decoder::new(None, 1000);
        let mut out = Vec::new();
        // Not even one block fills, so nothing asks about a stop.
        let never = || -> bool { panic!("a block was expanded") };
        d.records(&data, 0x1_0000, &never, &mut |o| out.push(o));
        assert_eq!(out.len(), 1);
        assert_eq!(samples(&out).len(), 1000);
        // A block at a time when it lets more out.
        let out = run(Decoder::new(None, 600_000), &data);
        assert!(out.iter().all(|o| match o {
            Output::Samples(s) => s.len() <= BLOCK_SAMPLES * UNIT_SIZE,
            Output::Trigger => false,
        }));
        assert_eq!(samples(&out).len(), 600_000);
    }

    #[test]
    fn the_soft_trigger_sends_its_ring_then_the_trigger_then_the_rest() {
        // CH0 rises at the fourth sample, the ring keeps two.
        let stl = SoftTriggerLogic::new(vec![vec![(0, TriggerMatch::Rising)]], 4, 2);
        let mut data = Vec::new();
        for v in [0u32, 2, 4, 1, 3] {
            data.extend(record(v, 0));
        }
        let out = run(Decoder::new(Some(stl), 100), &data);
        let t = out.iter().position(|o| *o == Output::Trigger).unwrap();
        assert_eq!(samples(&out[..t]), [2, 4]);
        assert_eq!(samples(&out[t..]), [1, 3]);
    }

    #[test]
    fn a_trigger_past_a_block_and_the_limit_on_the_ring() {
        let stl = SoftTriggerLogic::new(vec![vec![(0, TriggerMatch::One)]], 4, 8);
        // More than a block of zeros, then the match.
        let mut data = Vec::new();
        for _ in 0..5 {
            data.extend(record(0, 65534));
        }
        data.extend(record(1, 3));
        let out = run(Decoder::new(Some(stl), 10), &data);
        let t = out.iter().position(|o| *o == Output::Trigger).unwrap();
        assert_eq!(samples(&out[..t]), [0; 8]);
        assert_eq!(samples(&out[t..]), [1, 1]);
    }

    #[test]
    fn a_stop_ends_a_long_search() {
        // A trigger that never fires over 4 blocks: a decoder that ignores
        // the stop asks 4 times.
        let stl = SoftTriggerLogic::new(vec![vec![(0, TriggerMatch::One)]], 4, 0);
        let mut data = Vec::new();
        for _ in 0..16 {
            data.extend(record(0, 65534));
        }
        let mut d = Decoder::new(Some(stl), 100);
        let asked = Cell::new(0);
        let stopped = || {
            asked.set(asked.get() + 1);
            asked.get() == 2
        };
        d.records(&data, 16, &stopped, &mut |_| panic!("no samples"));
        assert_eq!(asked.get(), 2);
    }

    /// `n` samples of `value`, in records of the largest run.
    fn run_of(value: u32, mut n: usize) -> Vec<u8> {
        let mut data = Vec::new();
        while n > 0 {
            let run = n.min(65535);
            data.extend(record(value, (run - 1) as u16));
            n -= run;
        }
        data
    }

    /// The samples before the trigger and from it on, on CH0.
    fn around_trigger(m: TriggerMatch, data: &[u8]) -> (Vec<u32>, Vec<u32>) {
        let stl = SoftTriggerLogic::new(vec![vec![(0, m)]], 4, 2);
        let out = run(Decoder::new(Some(stl), 1000), data);
        let t = out.iter().position(|o| *o == Output::Trigger).expect("no trigger");
        (samples(&out[..t]), samples(&out[t..]))
    }

    #[test]
    fn an_edge_on_the_last_sample_of_a_block() {
        let mut data = run_of(0, BLOCK_SAMPLES - 1);
        data.extend(record(1, 0));
        data.extend(record(3, 0));
        let (pre, post) = around_trigger(TriggerMatch::Rising, &data);
        assert_eq!(pre, [0, 0]);
        assert_eq!(post, [1, 3]);
    }

    #[test]
    fn an_edge_on_the_first_sample_of_a_block() {
        // Against the last sample of the block before: rising from a zero
        // that a fresh start would assume too, so falling shows it as well.
        let mut data = run_of(0, BLOCK_SAMPLES);
        data.extend(record(1, 0));
        data.extend(record(3, 0));
        let (pre, post) = around_trigger(TriggerMatch::Rising, &data);
        assert_eq!(pre, [0, 0]);
        assert_eq!(post, [1, 3]);
        let mut data = run_of(1, BLOCK_SAMPLES);
        data.extend(record(2, 0));
        data.extend(record(4, 0));
        let (pre, post) = around_trigger(TriggerMatch::Falling, &data);
        assert_eq!(pre, [1, 1]);
        assert_eq!(post, [2, 4]);
    }

    #[test]
    fn a_limit_on_a_block_boundary() {
        let data = run_of(5, 2 * BLOCK_SAMPLES);
        let out = run(Decoder::new(None, BLOCK_SAMPLES as u64), &data);
        assert_eq!(out.len(), 1);
        assert_eq!(samples(&out).len(), BLOCK_SAMPLES);
    }

    #[test]
    fn a_register_reads_big_endian_and_a_short_reply_fails() {
        let r = replay(&[
            "bulk_out 04 030804 -> 3",
            "bulk_in 88 4 -> 12345678",
            "bulk_out 04 030804 -> 3",
            "bulk_in 88 4 -> 123456",
        ]);
        let mut l = link(&r);
        assert_eq!(l.read_reg(2).unwrap(), 0x1234_5678);
        assert!(l.read_reg(2).is_err());
        r.finish().unwrap();
    }

    #[test]
    fn a_stop_ends_the_wait_for_a_reply() {
        // An answer on offer: a wait that ignores the stop takes it.
        let r = replay(&[
            "bulk_out 04 030804 -> 3",
            "bulk_in 88 4 -> err timeout",
            "bulk_in 88 4 -> 12345678",
        ]);
        let mut l = link(&r);
        l.stop.store(true, Ordering::SeqCst);
        assert!(matches!(l.read_reg(2), Err(Error::Timeout)));
        assert_eq!(r.remaining(), 1);
    }

    #[test]
    fn a_slice_with_no_answer_is_tried_again() {
        let r = replay(&[
            "bulk_out 04 030804 -> 3",
            "bulk_in 88 4 -> err timeout",
            "bulk_in 88 4 -> err timeout",
            "bulk_in 88 4 -> 12345678",
        ]);
        let mut l = link(&r);
        assert_eq!(l.read_reg(2).unwrap(), 0x1234_5678);
        r.finish().unwrap();
    }

    #[test]
    fn a_configured_fpga_takes_no_bitstream() {
        let r = replay(&["bulk_out 04 030804 -> 3", "bulk_in 88 4 -> a5a5a5ab"]);
        let mut l = link(&r);
        l.apply_fpga_config_with(no_load).unwrap();
        // Known now: no second question.
        l.apply_fpga_config_with(no_load).unwrap();
        r.finish().unwrap();
    }

    #[test]
    fn the_upload_goes_in_chunks_with_the_keys_after_it() {
        let mut steps = upload_steps();
        steps.extend(
            [
                "bulk_out 04 050fa008 -> 4",
                "bulk_in 88 8 -> f681136400000000",
                "bulk_out 04 0440 04 641381f6 -> 7",
                "bulk_out 04 04440400000000 -> 7",
                "bulk_out 04 030804 -> 3",
                "bulk_in 88 4 -> a5a5a5a1",
                "bulk_out 04 04380400000000 -> 7",
            ]
            .map(|s| s.replace("0440 04 641381f6", "044004641381f6")),
        );
        let steps: Vec<&str> = steps.iter().map(String::as_str).collect();
        let r = replay(&steps);
        let mut l = link(&r);
        l.apply_fpga_config_with(small_stream).unwrap();
        l.apply_fpga_config_with(no_load).unwrap();
        r.finish().unwrap();
    }

    #[test]
    fn a_failed_key_step_leaves_the_open_good() {
        // protocol.c:646: the keys are not read, the open goes on and
        // register 14 is still written.
        let mut steps = upload_steps();
        steps.extend(
            ["bulk_out 04 050fa008 -> err device \"stalled\"", "bulk_out 04 04380400000000 -> 7"]
                .map(str::to_owned),
        );
        let steps: Vec<&str> = steps.iter().map(String::as_str).collect();
        let r = replay(&steps);
        let mut l = link(&r);
        l.apply_fpga_config_with(small_stream).unwrap();
        l.apply_fpga_config_with(no_load).unwrap();
        r.finish().unwrap();
    }

    #[test]
    fn a_failed_upload_fails_the_open_and_asks_again() {
        let r = replay(&[
            "bulk_out 04 030804 -> 3",
            "bulk_in 88 4 -> b6ff9c97",
            "bulk_out 04 030804 -> 3",
            "bulk_in 88 4 -> 12ff9c97",
            "bulk_out 04 01 -> err device \"stalled\"",
            "bulk_out 04 030804 -> 3",
            "bulk_in 88 4 -> a5a5a5a1",
        ]);
        let mut l = link(&r);
        assert!(l.apply_fpga_config_with(small_stream).is_err());
        l.apply_fpga_config_with(no_load).unwrap();
        r.finish().unwrap();
    }

    #[test]
    fn the_rate_sets_the_divider_and_both_multipliers() {
        // 250 MHz: divider 1, x2 off, x1.25 on (bit 7 clear).
        let r = replay(&[
            "bulk_out 04 04100400000000 -> 7",
            "bulk_out 04 031404 -> 3",
            "bulk_in 88 4 -> 00000081",
            "bulk_out 04 04140400000080 -> 7",
            "bulk_out 04 031404 -> 3",
            "bulk_in 88 4 -> 00000080",
            "bulk_out 04 04140400000000 -> 7",
        ]);
        let mut l = link(&r);
        l.set_samplerate(250_000_000).unwrap();
        // Not in the table: nothing on the wire.
        assert!(l.set_samplerate(3_000_000).is_err());
        r.finish().unwrap();
    }

    #[test]
    fn the_rate_list_is_the_table_in_order() {
        assert_eq!(SAMPLERATES[0], 500_000_000);
        assert_eq!(SAMPLERATES[19], 2000);
        assert!(SAMPLERATES.windows(2).all(|w| w[0] > w[1]));
    }

    #[test]
    fn c_return_codes_for_its_log_line() {
        assert_eq!(ret_code(&Error::Timeout), -7);
        assert_eq!(ret_code(&Error::Disconnected), -4);
        assert_eq!(ret_code(&Error::device("endpoint stalled")), -9);
        // A command not taken and a wrong length are SR_ERR.
        assert_eq!(ret_code(&Error::Device("command 4 failed: LIBUSB_ERROR_PIPE".into())), -1);
    }
}
