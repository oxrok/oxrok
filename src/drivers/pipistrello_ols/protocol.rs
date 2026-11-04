// Counterpart in sigrok:
//   libsigrok/src/hardware/pipistrello-ols/protocol.c,
//   libsigrok/src/hardware/pipistrello-ols/protocol.h,
//   libsigrok/src/hardware/pipistrello-ols/api.c (the counts and the trigger stages),
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, magnuskarlsson, Daniel Elstner,
//   Gerhard Sittig, Martin Ling, Lars-Peter Clausen, Marek Vasut,
//   Soeren Apel, Wolfram Sang, Tilman Sauerbeck
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The protocol of `p-ols`, `protocol.c` and the counts of `api.c`:
//! SUMP commands to the Open Logic Sniffer bitstream through port B of
//! the Pipistrello's FT2232H, its metadata, the sample rate divider,
//! the trigger stages, and the decoding of the samples it sends back.
//!
//! Commands are written one per `ftdi_write_data`: a short command is
//! one byte, a long one the byte and a 32 bit argument, little endian.
//! The board answers `ID` (0x02) with `1ALS` or `1SLO`, and `METADATA`
//! (0x04) with keys whose type is in bits 5-7 (0 a NUL terminated
//! string, 1 a 32 bit big endian integer, 2 a byte; key 0 ends).
//!
//! A capture is set up by the trigger stages (mask, value, config and
//! edge of each), the divider of the 100 MHz clock, the read count and
//! the delay count (both in units of 4 memory words, minus 1), the
//! flags, then `RUN`. The board sends the last read count's words back,
//! newest first, each the bytes of its enabled channel groups. With the
//! RLE flag a word whose top bit (of its top byte) is set is the count
//! of repeats of the word that follows it in the stream; in demux mode
//! (above 100 MHz) a word holds two samples of channels 0-15.
//!
//! Where this differs from the C driver, each a bug the emulator model
//! (`emu/src/models/pipistrello-ols.rs`) found:
//! - the excess of the last buffer beyond the sample limit is taken off
//!   the run being copied, never off a count of 0 (`protocol.c`
//!   557-563 and 597-600 wrap `rle_count` and copy beyond the buffer);
//! - at a limit that is not a multiple of 4, or with RLE, the trigger
//!   marker follows the board's word count rather than taking counts of
//!   words for samples, and a marker before the first sample is no
//!   marker (`protocol.c` 620 tests only -1 and sends samples from
//!   before its buffer);
//! - four stages put the start bit on the last of them, as there is no
//!   fifth for it (`set_trigger` in `api.c` sets it on stage
//!   `num_stages`, which does not exist, so the board never starts);
//! - the delay count is at least one unit (`captureratio=100` sends
//!   0xffffffff);
//! - a board that reports no memory size, a capture without a sample
//!   limit, and channels 16-31 in demux mode are refused rather than
//!   sent as counts of 0xffffffff or read for ever;
//! - the reads stop once the board has sent what was asked for, not only
//!   at the sample limit (all 32 channels at 200 MHz never reach it).

use crate::core::{Trigger, TriggerMatch};

/// `USB_VENDOR_ID`.
pub const VENDOR_ID: u16 = 0x0403;
/// `USB_DEVICE_ID`.
pub const PRODUCT_ID: u16 = 0x6010;
/// `USB_IPRODUCT`, which `ftdi_usb_open_desc` matches.
pub const PRODUCT_STRING: &str = "Pipistrello LX45";
/// `FTDI_BUF_SIZE`: what one read asks libftdi for.
pub const FTDI_BUF_SIZE: usize = 16 * 1024;
/// `NUM_CHANNELS`.
pub const NUM_CHANNELS: usize = 32;
/// `NUM_TRIGGER_STAGES`.
pub const NUM_TRIGGER_STAGES: usize = 4;
/// `CLOCK_RATE`.
pub const CLOCK_RATE: u64 = 100_000_000;
/// `MIN_NUM_SAMPLES`.
pub const MIN_NUM_SAMPLES: u64 = 4;
/// `DEFAULT_SAMPLERATE`.
pub const DEFAULT_SAMPLERATE: u64 = 100_000_000;
/// The sample rates `config_set()` takes: 10 Hz to 200 MHz.
pub const SAMPLERATE_MIN: u64 = 10;
/// See [`SAMPLERATE_MIN`].
pub const SAMPLERATE_MAX: u64 = 200_000_000;

/// `CMD_RESET`.
pub const CMD_RESET: u8 = 0x00;
/// `CMD_RUN`.
pub const CMD_RUN: u8 = 0x01;
/// `CMD_ID`.
pub const CMD_ID: u8 = 0x02;
/// `CMD_METADATA`.
pub const CMD_METADATA: u8 = 0x04;
/// `CMD_SET_DIVIDER`.
pub const CMD_SET_DIVIDER: u8 = 0x80;
/// `CMD_SET_FLAGS`.
pub const CMD_SET_FLAGS: u8 = 0x82;
/// `CMD_CAPTURE_DELAYCOUNT`.
pub const CMD_CAPTURE_DELAYCOUNT: u8 = 0x83;
/// `CMD_CAPTURE_READCOUNT`.
pub const CMD_CAPTURE_READCOUNT: u8 = 0x84;
/// `CMD_SET_TRIGGER_MASK`, plus 4 per stage.
pub const CMD_SET_TRIGGER_MASK: u8 = 0xc0;
/// `CMD_SET_TRIGGER_VALUE`.
pub const CMD_SET_TRIGGER_VALUE: u8 = 0xc1;
/// `CMD_SET_TRIGGER_CONFIG`.
pub const CMD_SET_TRIGGER_CONFIG: u8 = 0xc2;
/// `CMD_SET_TRIGGER_EDGE`.
pub const CMD_SET_TRIGGER_EDGE: u8 = 0xc3;
/// `TRIGGER_START`, in the config's byte 3.
pub const TRIGGER_START: u8 = 1 << 3;

/// `FLAG_INTERNAL_TEST_MODE`.
pub const FLAG_INTERNAL_TEST_MODE: u16 = 1 << 11;
/// `FLAG_EXTERNAL_TEST_MODE`.
pub const FLAG_EXTERNAL_TEST_MODE: u16 = 1 << 10;
/// `FLAG_SWAP_CHANNELS`.
pub const FLAG_SWAP_CHANNELS: u16 = 1 << 9;
/// `FLAG_RLE`.
pub const FLAG_RLE: u16 = 1 << 8;
/// `FLAG_CLOCK_EXTERNAL`.
pub const FLAG_CLOCK_EXTERNAL: u16 = 1 << 6;
/// `FLAG_FILTER`.
pub const FLAG_FILTER: u16 = 1 << 1;
/// `FLAG_DEMUX`.
pub const FLAG_DEMUX: u16 = 1 << 0;

/// A short command.
pub fn short_command(cmd: u8) -> Vec<u8> {
    vec![cmd]
}

/// A long command: the byte and its argument, little endian.
pub fn long_command(cmd: u8, arg: u32) -> Vec<u8> {
    let mut v = vec![cmd];
    v.extend_from_slice(&arg.to_le_bytes());
    v
}

/// What `p_ols_get_metadata()` takes from the metadata block.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Metadata {
    /// The device name (token 0x01 strings, concatenated).
    pub name: String,
    /// "FPGA version X" and "Ancillary version Y", in the order they
    /// came, joined by ", ".
    pub version: String,
    /// The number of usable channels, at most 32.
    pub channels: usize,
    /// The sample memory in bytes, 0 when not reported.
    pub max_samplebytes: u32,
    /// The highest sample rate, 0 when not reported.
    pub max_samplerate: u32,
    /// The protocol version.
    pub protocol_version: u32,
}

/// `p_ols_get_metadata()`. A value cut off by the end of the block is
/// not taken (the C driver reads beyond what it received), and a
/// channel count beyond 32 is taken as 32 (the C driver names channels
/// from beyond its table).
pub fn parse_metadata(buf: &[u8]) -> Metadata {
    let mut m = Metadata::default();
    let mut i = 0;
    while i < buf.len() {
        let key = buf[i];
        i += 1;
        if key == 0 {
            log::debug!("Got metadata key 0x00, metadata ends.");
            break;
        }
        let token = key & 0x1f;
        match key >> 5 {
            0 => {
                let end = buf[i..].iter().position(|&b| b == 0).map_or(buf.len(), |p| i + p);
                let s: String = buf[i..end].iter().map(|&b| char::from(b)).collect();
                i = (end + 1).min(buf.len());
                log::debug!("Got metadata key 0x{key:02x} value '{s}'.");
                let mut version = |what: &str| {
                    if !m.version.is_empty() {
                        m.version.push_str(", ");
                    }
                    m.version.push_str(what);
                    m.version.push_str(&s);
                };
                match token {
                    0x01 => m.name.push_str(&s),
                    0x02 => version("FPGA version "),
                    0x03 => version("Ancillary version "),
                    _ => log::info!("Unknown token 0x{token:02x}: '{s}'"),
                }
            }
            1 => {
                let Some(b) = buf.get(i..i + 4) else {
                    log::debug!("Metadata key 0x{key:02x} cut off.");
                    break;
                };
                let v = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
                i += 4;
                log::debug!("Got metadata key 0x{key:02x} value 0x{v:08x}.");
                match token {
                    0x00 => m.channels = (v as usize).min(NUM_CHANNELS),
                    0x01 => m.max_samplebytes = v,
                    0x02 => {}
                    0x03 => m.max_samplerate = v,
                    0x04 => m.protocol_version = v,
                    _ => log::info!("Unknown token 0x{token:02x}: 0x{v:08x}."),
                }
            }
            2 => {
                let Some(&v) = buf.get(i) else {
                    log::debug!("Metadata key 0x{key:02x} cut off.");
                    break;
                };
                i += 1;
                log::debug!("Got metadata key 0x{key:02x} value 0x{v:02x}.");
                match token {
                    0x00 => m.channels = usize::from(v).min(NUM_CHANNELS),
                    0x01 => m.protocol_version = u32::from(v),
                    _ => log::info!("Unknown token 0x{token:02x}: 0x{v:02x}."),
                }
            }
            _ => {}
        }
    }
    m
}

/// A sample rate as `p_ols_set_samplerate()` sets it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rate {
    /// The rate the divider gives.
    pub samplerate: u64,
    /// The divider of the clock, minus 1.
    pub divider: u32,
    /// Demux mode: two samples per clock, above 100 MHz.
    pub demux: bool,
}

/// `p_ols_set_samplerate()`: `None` beyond the board's maximum (when it
/// gave one). `rate` is at least [`SAMPLERATE_MIN`].
pub fn samplerate(rate: u64, max_samplerate: u32) -> Option<Rate> {
    if max_samplerate != 0 && rate > u64::from(max_samplerate) {
        return None;
    }
    let demux = rate > CLOCK_RATE;
    let divider = if demux { CLOCK_RATE * 2 / rate - 1 } else { CLOCK_RATE / rate - 1 };
    let mut divider = u32::try_from(divider).unwrap_or(u32::MAX - 1);
    let given = |divider: u32| {
        let rate = CLOCK_RATE / (u64::from(divider) + 1);
        if demux { rate * 2 } else { rate }
    };
    // The divider rounds down: never above the board's maximum.
    if max_samplerate != 0 && given(divider) > u64::from(max_samplerate) {
        divider += 1;
    }
    Some(Rate { samplerate: given(divider), divider, demux })
}

/// The enabled channel groups (bytes of the channel mask), one bit each.
pub fn group_mask(channel_mask: u32) -> u8 {
    (0..4).filter(|i| channel_mask & (0xff << (i * 8)) != 0).fold(0, |m, i| m | (1 << i))
}

/// The bytes of memory a word takes for the groups of `mask`: three
/// groups take as many as four.
pub fn group_bytes(mask: u8) -> u32 {
    match mask.count_ones() {
        3 => 4,
        n => n,
    }
}

/// The trigger stages as `pols_convert_trigger()` builds them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stages {
    /// The channels each stage looks at.
    pub mask: [u32; NUM_TRIGGER_STAGES],
    /// Their levels.
    pub value: [u32; NUM_TRIGGER_STAGES],
    /// Which of them are edges.
    pub edge: [u32; NUM_TRIGGER_STAGES],
    /// The number of stages.
    pub count: usize,
}

/// `pols_convert_trigger()`: conditions on disabled channels are left
/// out, as in the C driver.
///
/// # Errors
///
/// The message the C driver logs for more than four stages, or one for
/// a match the board does not have.
pub fn convert_trigger(trigger: Option<&Trigger>, channel_mask: u32) -> Result<Stages, String> {
    let mut s = Stages::default();
    let Some(t) = trigger else { return Ok(s) };
    if t.stages.len() > NUM_TRIGGER_STAGES {
        return Err(format!("This device only supports {NUM_TRIGGER_STAGES} trigger stages."));
    }
    s.count = t.stages.len();
    for (i, stage) in t.stages.iter().enumerate() {
        for c in &stage.conditions {
            let Some(bit) = u32::try_from(c.channel).ok().and_then(|c| 1u32.checked_shl(c)) else {
                continue;
            };
            if channel_mask & bit == 0 {
                continue;
            }
            s.mask[i] |= bit;
            match c.matches {
                TriggerMatch::Zero => {}
                TriggerMatch::One => s.value[i] |= bit,
                TriggerMatch::Rising => {
                    s.value[i] |= bit;
                    s.edge[i] |= bit;
                }
                TriggerMatch::Falling => s.edge[i] |= bit,
                other => return Err(format!("Unsupported trigger match '{}'.", other.code())),
            }
        }
    }
    Ok(s)
}

/// The commands `set_trigger()` sends for stage `stage`: mask, value,
/// config (the level, and the start bit on `start`), edge.
pub fn stage_commands(s: &Stages, stage: usize, start: usize) -> [Vec<u8>; 4] {
    let off = 4 * u8::try_from(stage).unwrap_or(0);
    let level = u32::try_from(stage).unwrap_or(0) << 16;
    let start_bit = if stage == start { u32::from(TRIGGER_START) << 24 } else { 0 };
    [
        long_command(CMD_SET_TRIGGER_MASK + off, s.mask[stage]),
        long_command(CMD_SET_TRIGGER_VALUE + off, s.value[stage]),
        long_command(CMD_SET_TRIGGER_CONFIG + off, level | start_bit),
        long_command(CMD_SET_TRIGGER_EDGE + off, s.edge[stage]),
    ]
}

/// The commands `disable_trigger()` sends for stage `stage`: level 3,
/// nothing to match.
pub fn disabled_stage_commands(stage: usize) -> [Vec<u8>; 4] {
    let off = 4 * u8::try_from(stage).unwrap_or(0);
    [
        long_command(CMD_SET_TRIGGER_MASK + off, 0),
        long_command(CMD_SET_TRIGGER_VALUE + off, 0),
        long_command(CMD_SET_TRIGGER_CONFIG + off, 3 << 16),
        long_command(CMD_SET_TRIGGER_EDGE + off, 0),
    ]
}

/// The settings a capture is planned from.
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    /// `flag_reg`, as the options and the sample rate left it.
    pub flag_reg: u16,
    /// The enabled channels.
    pub channel_mask: u32,
    /// The board's memory in bytes.
    pub max_samplebytes: u32,
    /// The sample limit.
    pub limit_samples: u64,
    /// The capture ratio, in percent.
    pub capture_ratio: u64,
    /// The divider.
    pub divider: u32,
}

/// A capture as `dev_acquisition_start()` sets it up, and what the
/// reads expect.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Plan {
    /// The commands to send, in order, `RUN` last.
    pub commands: Vec<Vec<u8>>,
    /// `flag_reg` afterwards: the disabled groups in bits 2-5.
    pub flag_reg: u16,
    /// Read count, in units of 4 memory words.
    pub readcount: u32,
    /// Delay count, in the same units.
    pub delaycount: u32,
    /// `max_samples`: what the memory holds, as the C counts it.
    pub max_samples: u64,
    /// The memory word, counted from the newest one sent, that the
    /// trigger is at, when there is a trigger.
    pub trigger_word: Option<u64>,
}

/// `dev_acquisition_start()` up to `RUN`, without the C driver's
/// overflows (see the module documentation).
///
/// # Errors
///
/// The message to log when the capture cannot be set up.
pub fn plan(s: &Settings, stages: &Stages) -> Result<Plan, String> {
    let demux = s.flag_reg & FLAG_DEMUX != 0;
    let groups = group_mask(s.channel_mask);
    if groups == 0 {
        return Err("No channels enabled.".into());
    }
    if demux && groups & 0b1100 != 0 {
        return Err("Channels 16-31 cannot be sampled above 100 MHz (demux mode).".into());
    }
    if s.max_samplebytes == 0 {
        return Err("The device reported no sample memory size.".into());
    }
    if s.limit_samples == 0 {
        return Err("A sample limit is needed.".into());
    }
    let max_samples = u64::from(s.max_samplebytes / group_bytes(groups));
    log::debug!("max_samples = {max_samples}");
    log::debug!("limit_samples = {}", s.limit_samples);
    let samplecount = max_samples.min(s.limit_samples);
    log::debug!("Samplecount = {samplecount}");
    let per_count = if demux { 8 } else { 4 };
    let readcount = u32::try_from(samplecount.div_ceil(per_count)).unwrap_or(u32::MAX).max(1);

    let mut commands = Vec::new();
    let trigger_word;
    let delaycount;
    if stages.count > 0 {
        // The C's arithmetic, in double; at least one unit after the
        // trigger, which the board needs to start at all.
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let d = (f64::from(readcount) * (1.0 - s.capture_ratio as f64 / 100.0)) as u32;
        delaycount = d.max(1);
        // The start bit goes on the stage after the user's; with four
        // there is none after them, and it goes on the last one. The
        // marker, in memory words from the newest: api.c:524's
        // (readcount - delaycount) * 4 - num_stages from the oldest.
        // From the firmware's source (hand read, no bench) that is 3
        // samples after the last match at 100 MHz, 2 at 50 MHz, 1 at
        // 33 MHz or less, less num_stages - 1. On the last stage itself
        // the first word the delay counts is the one after the match,
        // so the word at `words` is the match.
        let start = stages.count.min(NUM_TRIGGER_STAGES - 1);
        let words = u64::from(delaycount) * 4;
        trigger_word =
            Some(if start < stages.count { words } else { words + stages.count as u64 - 1 });
        for i in 0..NUM_TRIGGER_STAGES {
            if i <= stages.count {
                log::debug!("Setting p-ols stage {i} trigger.");
                commands.extend(stage_commands(stages, i, start));
            } else {
                log::debug!("Disabling p-ols stage {i} trigger.");
                commands.extend(disabled_stage_commands(i));
            }
        }
    } else {
        log::debug!("Forcing trigger at stage 0.");
        commands.extend(stage_commands(stages, 0, 0));
        delaycount = readcount;
        trigger_word = None;
    }
    commands.push(long_command(CMD_SET_DIVIDER, s.divider & 0x00ff_ffff));
    commands.push(long_command(CMD_CAPTURE_READCOUNT, readcount - 1));
    commands.push(long_command(CMD_CAPTURE_DELAYCOUNT, delaycount - 1));
    let mut flag_reg = s.flag_reg & !0x3c;
    flag_reg |= !(u16::from(groups) << 2) & 0x3c;
    log::debug!("flag_reg = {flag_reg:x}");
    let mut flags = flag_reg;
    if demux {
        flags &= !0x30;
        flags |= !(u16::from(groups) << 4) & 0x30;
    }
    commands.push(long_command(CMD_SET_FLAGS, u32::from(flags)));
    commands.push(short_command(CMD_RUN));
    Ok(Plan { commands, flag_reg, readcount, delaycount, max_samples, trigger_word })
}

/// The samples of a capture, decoded as `p_ols_receive_data()` decodes
/// them, newest first.
pub struct Decoder {
    /// Enabled groups (bit per group) of `flag_reg`.
    groups: u8,
    /// Bytes per sample on the wire (`num_channels`).
    width: usize,
    rle: bool,
    /// Demux and RLE: a unit is a pair of samples.
    pairs: bool,
    limit: u64,
    max_samples: u64,
    /// Bytes per memory word.
    word_bytes: u64,
    /// The bytes the board sends for the read count.
    expected: u64,
    trigger_word: Option<u64>,
    unit: Vec<u8>,
    rle_count: u64,
    /// `cnt_samples`.
    counted: u64,
    bytes: u64,
    samples: Vec<u32>,
    /// The trigger, in samples from the newest.
    trigger: Option<u64>,
}

impl Decoder {
    /// A decoder for a capture of `plan` with sample limit `limit`.
    pub fn new(plan: &Plan, limit: u64) -> Self {
        let flag_reg = plan.flag_reg;
        let groups = !(flag_reg >> 2) as u8 & 0xf;
        let width = groups.count_ones() as usize;
        let demux = flag_reg & FLAG_DEMUX != 0;
        let rle = flag_reg & FLAG_RLE != 0;
        let word_bytes = width as u64 * if demux { 2 } else { 1 };
        Decoder {
            groups,
            width,
            rle,
            pairs: demux && rle,
            limit,
            max_samples: plan.max_samples,
            word_bytes,
            expected: u64::from(plan.readcount) * 4 * word_bytes,
            trigger_word: plan.trigger_word,
            unit: Vec::with_capacity(8),
            rle_count: 0,
            counted: 0,
            bytes: 0,
            samples: Vec::new(),
            trigger: None,
        }
    }

    /// Whether the reads are over: the sample limit, the memory the C
    /// driver counts, or everything the board was asked for.
    pub fn done(&self) -> bool {
        self.samples.len() as u64 >= self.limit
            || self.counted >= self.max_samples
            || self.bytes >= self.expected
    }

    /// The samples decoded so far.
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// Whether nothing was decoded yet.
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// A word of the bytes of the groups in `groups` as a sample of all
    /// 32 channels, the other groups 0.
    fn expand(bytes: &[u8], groups: u8) -> u32 {
        let mut out = [0u8; 4];
        let mut j = 0;
        for (i, b) in out.iter_mut().enumerate() {
            if groups & (1 << i) != 0 {
                *b = bytes[j];
                j += 1;
            }
        }
        u32::from_le_bytes(out)
    }

    /// Takes the bytes of one read. Bytes after the sample limit are
    /// dropped, as the C driver drops the rest of that read.
    pub fn feed(&mut self, data: &[u8]) {
        let unit_len = if self.pairs { 2 * self.width } else { self.width };
        for &b in data {
            if self.samples.len() as u64 >= self.limit {
                return;
            }
            self.bytes += 1;
            self.unit.push(b);
            if self.unit.len() < unit_len {
                continue;
            }
            let unit = std::mem::take(&mut self.unit);
            let start = self.bytes - unit_len as u64;
            let at_trigger = self
                .trigger_word
                .is_some_and(|w| start == w * self.word_bytes && self.trigger.is_none());
            let per_unit = if self.pairs { 2 } else { 1 };
            self.counted += per_unit;
            let before = self.samples.len() as u64;
            if self.rle && unit[unit_len - 1] & 0x80 != 0 {
                let mut v = unit.clone();
                v[unit_len - 1] &= 0x7f;
                let mut n = [0u8; 8];
                n[..unit_len].copy_from_slice(&v);
                self.rle_count = u64::from_le_bytes(n);
                log::trace!("RLE count: {}.", self.rle_count * per_unit);
                if at_trigger {
                    // The count word stands for the run's last sample.
                    self.trigger = Some(before);
                }
                continue;
            }
            let run = self.rle_count + 1;
            self.rle_count = 0;
            if at_trigger {
                // A value word stands for its run's first sample.
                self.trigger = Some(before + (run - 1) * per_unit);
            }
            let room = self.limit - before;
            if self.pairs {
                // The pair's top bit, the older sample's top channel,
                // is the count flag: that channel is lost, and is 0 in
                // both samples, as in a word of one sample.
                let g = self.groups & 0b11;
                let keep = !(if g & 0b10 != 0 { 1u32 << 15 } else { 1 << 7 });
                let newer = Self::expand(&unit[..self.width], g) & keep;
                let older = Self::expand(&unit[self.width..], g) & keep;
                for _ in 0..run {
                    if self.samples.len() as u64 >= self.limit {
                        break;
                    }
                    self.samples.push(newer);
                    if self.samples.len() as u64 >= self.limit {
                        break;
                    }
                    self.samples.push(older);
                }
            } else {
                let sample = Self::expand(&unit, self.groups);
                let n = usize::try_from(run.min(room)).unwrap_or(usize::MAX);
                self.samples.extend(std::iter::repeat_n(sample, n));
            }
            if self.samples.len() as u64 >= self.limit {
                // The rest of this read goes, as in the C driver.
                return;
            }
        }
    }

    /// The samples, oldest first, and the trigger's position among
    /// them when it is one of them.
    pub fn finish(mut self) -> (Vec<u32>, Option<usize>) {
        self.samples.reverse();
        let n = self.samples.len() as u64;
        let pos = self.trigger.filter(|&k| k < n).map(|k| (n - 1 - k) as usize);
        (self.samples, pos)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The model's metadata block (`emu/src/models/pipistrello-ols.rs`).
    const META: &str =
        "0150697069737472656c6c6f204f4c530002332e30370020000000202100100000230bebc200410200";

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    #[test]
    fn metadata_as_the_driver_reads_it() {
        let m = parse_metadata(&hex(META));
        assert_eq!(m.name, "Pipistrello OLS");
        assert_eq!(m.version, "FPGA version 3.07");
        assert_eq!(m.channels, 32);
        assert_eq!(m.max_samplebytes, 0x0010_0000);
        assert_eq!(m.max_samplerate, 200_000_000);
        assert_eq!(m.protocol_version, 2);
        // Both orders of the versions, unknown keys of each type, the
        // 8 bit channel count, a key of type 3 skipped alone.
        let m = parse_metadata(&hex(
            "014f4c5300034100024600034200047878002200001000230bebc200240000000225000000074010410142076000",
        ));
        assert_eq!(m.name, "OLS");
        assert_eq!(m.version, "Ancillary version A, FPGA version F, Ancillary version B");
        assert_eq!((m.channels, m.max_samplebytes, m.protocol_version), (16, 0, 1));
        // A block cut short: the name to its end, nothing else.
        let mut long = vec![0x01];
        long.extend([b'A'; 63]);
        let m = parse_metadata(&long);
        assert_eq!(m.name.len(), 63);
        assert_eq!(m.channels, 0);
        // A 32 bit value cut off is not read from beyond the block.
        assert_eq!(parse_metadata(&hex("2100")).max_samplebytes, 0);
        // More than 32 channels: 32.
        assert_eq!(parse_metadata(&hex("4040")).channels, 32);
    }

    #[test]
    fn samplerates() {
        let r = samplerate(100_000_000, 200_000_000).unwrap();
        assert_eq!(r, Rate { samplerate: 100_000_000, divider: 0, demux: false });
        let r = samplerate(3_000_000, 0).unwrap();
        assert_eq!((r.samplerate, r.divider), (3_030_303, 32));
        let r = samplerate(200_000_000, 200_000_000).unwrap();
        assert_eq!(r, Rate { samplerate: 200_000_000, divider: 0, demux: true });
        let r = samplerate(150_000_000, 0).unwrap();
        assert_eq!((r.samplerate, r.divider, r.demux), (200_000_000, 0, true));
        assert_eq!(samplerate(10, 0).unwrap().divider, 9_999_999);
        assert!(samplerate(200_000_000, 100_000_000).is_none());
        // A maximum the clock does not divide to: the rate below it.
        let r = samplerate(60_000_000, 60_000_000).unwrap();
        assert_eq!((r.samplerate, r.divider), (50_000_000, 1));
    }

    #[test]
    fn groups() {
        assert_eq!(group_mask(0xffff_ffff), 0xf);
        assert_eq!(group_mask(0x0000_ff00), 0b0010);
        assert_eq!(group_mask(0xc000_0003), 0b1001);
        assert_eq!(group_bytes(0b0111), 4);
        assert_eq!(group_bytes(0b0101), 2);
    }

    fn settings() -> Settings {
        Settings {
            flag_reg: FLAG_FILTER,
            channel_mask: 0xffff_ffff,
            max_samplebytes: 0x0010_0000,
            limit_samples: 100,
            capture_ratio: 0,
            divider: 0,
        }
    }

    fn hexes(p: &Plan) -> Vec<String> {
        use std::fmt::Write as _;
        p.commands
            .iter()
            .map(|c| {
                c.iter().fold(String::new(), |mut s, b| {
                    let _ = write!(s, "{b:02x}");
                    s
                })
            })
            .collect()
    }

    #[test]
    fn a_capture_without_trigger_as_the_driver_sends_it() {
        let p = plan(&settings(), &Stages::default()).unwrap();
        // The model's "100 samples" golden.
        assert_eq!(
            hexes(&p),
            [
                "c000000000",
                "c100000000",
                "c200000008",
                "c300000000",
                "8000000000",
                "8418000000",
                "8318000000",
                "8202000000",
                "01"
            ]
        );
        assert_eq!(p.trigger_word, None);
        assert_eq!(p.max_samples, 262_144);
    }

    fn trigger(spec: &[(usize, usize, TriggerMatch)]) -> Trigger {
        let mut t = Trigger::new();
        for &(stage, ch, m) in spec {
            t.add(stage, ch, m);
        }
        t
    }

    #[test]
    fn trigger_stages_as_the_driver_sends_them() {
        let s = Settings { channel_mask: 0xff, limit_samples: 64, capture_ratio: 50, ..settings() };
        let st = convert_trigger(Some(&trigger(&[(0, 4, TriggerMatch::Rising)])), 0xff).unwrap();
        let p = plan(&s, &st).unwrap();
        // The model's "trigger rising" golden.
        assert_eq!(
            hexes(&p)[..8],
            [
                "c010000000",
                "c110000000",
                "c200000000",
                "c310000000",
                "c400000000",
                "c500000000",
                "c600000108",
                "c700000000"
            ]
        );
        assert_eq!(hexes(&p)[16..], ["8000000000", "840f000000", "8307000000", "823a000000", "01"]);
        // Delay 8 units: the match is 32 words from the newest.
        assert_eq!(p.trigger_word, Some(32));
        // Four stages: the start bit on the last.
        let four = trigger(&[
            (0, 3, TriggerMatch::One),
            (1, 3, TriggerMatch::Zero),
            (2, 3, TriggerMatch::One),
            (3, 3, TriggerMatch::Zero),
        ]);
        let st = convert_trigger(Some(&four), 0xff).unwrap();
        let p = plan(&s, &st).unwrap();
        let h = hexes(&p);
        assert_eq!((h[2].as_str(), h[14].as_str()), ("c200000000", "ce00000308"));
        // The word at the delay count is the match itself.
        assert_eq!(p.trigger_word, Some(32));
        // Two and three stages: api.c:524's position.
        for n in [2, 3] {
            let st = Stages { count: n, ..Stages::default() };
            assert_eq!(plan(&s, &st).unwrap().trigger_word, Some(32 + n as u64 - 1), "{n}");
        }
        // Five: refused with the C driver's message.
        let mut five = four.clone();
        five.add(4, 3, TriggerMatch::One);
        assert_eq!(
            convert_trigger(Some(&five), 0xff).unwrap_err(),
            "This device only supports 4 trigger stages."
        );
        // captureratio=100: one unit after the trigger, not 0xffffffff.
        let s100 = Settings { capture_ratio: 100, ..s };
        let st = convert_trigger(Some(&trigger(&[(0, 3, TriggerMatch::One)])), 0xff).unwrap();
        let p = plan(&s100, &st).unwrap();
        assert_eq!(p.delaycount, 1);
        assert!(hexes(&p).contains(&"8300000000".to_owned()));
        // A condition on a disabled channel is left out.
        let st = convert_trigger(Some(&trigger(&[(0, 9, TriggerMatch::One)])), 0xff).unwrap();
        assert_eq!(st.mask[0], 0);
    }

    #[test]
    fn refusals() {
        let st = Stages::default();
        let demux = Settings { flag_reg: FLAG_DEMUX, ..settings() };
        assert!(plan(&demux, &st).unwrap_err().contains("16-31"));
        assert!(plan(&Settings { channel_mask: 0xffff, ..demux }, &st).is_ok());
        assert!(plan(&Settings { max_samplebytes: 0, ..settings() }, &st).is_err());
        assert!(plan(&Settings { limit_samples: 0, ..settings() }, &st).is_err());
        assert!(plan(&Settings { channel_mask: 0, ..settings() }, &st).is_err());
    }

    #[test]
    fn demux_flags_replicate_the_lower_groups() {
        let s = Settings { flag_reg: FLAG_DEMUX, channel_mask: 0xff00, ..settings() };
        let p = plan(&s, &Stages::default()).unwrap();
        // Group 2 alone: groups 1, 3 and 4 off in flag_reg; what is
        // sent has group 3 off and 4 on again (bits 4-5 repeat bits
        // 2-3).
        assert_eq!(p.flag_reg, 0x35);
        assert_eq!(hexes(&p)[7], "8215000000");
        assert_eq!(p.readcount, 13);
    }

    /// Words of one group, newest first, as the board sends them.
    fn one_group(limit: u64, rle: bool) -> Decoder {
        let s = Settings {
            channel_mask: 0xff00,
            limit_samples: limit,
            flag_reg: if rle { FLAG_RLE } else { 0 },
            ..settings()
        };
        Decoder::new(&plan(&s, &Stages::default()).unwrap(), limit)
    }

    #[test]
    fn decodes_newest_first_and_expands_groups() {
        let mut d = one_group(8, false);
        d.feed(&[7, 6, 5, 4, 3, 2, 1, 0]);
        assert!(d.done());
        let (s, t) = d.finish();
        assert_eq!(s, (0..8).map(|v| v << 8).collect::<Vec<u32>>());
        assert_eq!(t, None);
    }

    #[test]
    fn a_limit_not_a_multiple_of_4_keeps_the_newest_samples() {
        // 101 samples: 104 words come, the 3 oldest are dropped, and no
        // count wraps.
        let mut d = one_group(101, false);
        let words: Vec<u8> = (0..104u8).rev().collect();
        d.feed(&words);
        let (s, _) = d.finish();
        assert_eq!(s.len(), 101);
        assert_eq!(s[0], 3 << 8);
        assert_eq!(s[100], 103 << 8);
    }

    #[test]
    fn rle_runs_and_the_excess_off_the_last_run() {
        let mut d = one_group(10, true);
        // Newest first: 5 four times (count 3), then 9 once, then 2
        // eight times (count 7), of which 5 fit.
        d.feed(&[0x83, 5, 9, 0x87, 2, 0x81, 1]);
        let (s, _) = d.finish();
        let v: Vec<u32> = s.iter().map(|x| x >> 8).collect();
        assert_eq!(v, [2, 2, 2, 2, 2, 9, 5, 5, 5, 5]);
    }

    #[test]
    fn the_trigger_marker_follows_the_word_count() {
        let s = Settings { channel_mask: 0xff, limit_samples: 16, capture_ratio: 50, ..settings() };
        let st = convert_trigger(Some(&trigger(&[(0, 1, TriggerMatch::One)])), 0xff).unwrap();
        let p = plan(&s, &st).unwrap();
        assert_eq!(p.trigger_word, Some(8));
        let mut d = Decoder::new(&p, 16);
        d.feed(&(0..16u8).rev().collect::<Vec<_>>());
        // 16 samples, the trigger 8 from the newest: position 7.
        assert_eq!(d.finish().1, Some(7));
        // With RLE: a run of 3 (value word at the trigger word) puts
        // the marker on the run's first sample.
        let p = Plan { flag_reg: p.flag_reg | FLAG_RLE, trigger_word: Some(2), ..p };
        let mut d = Decoder::new(&p, 16);
        d.feed(&[1, 0x82, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
        let (s, t) = d.finish();
        assert_eq!(s.len(), 16);
        // Newest first: 1, then 3 three times: the oldest of those is
        // 3 samples from the newest.
        assert_eq!(t, Some(16 - 1 - 3));
        assert_eq!(s[16 - 1 - 3], 3);
        assert_eq!(s[16 - 1 - 1], 3);
        assert_eq!(s[16 - 1 - 4], 4);
    }

    #[test]
    fn the_marker_on_a_count_word_is_the_newest_sample_of_its_run() {
        let s = Settings { channel_mask: 0xff, limit_samples: 16, ..settings() };
        let p = plan(&s, &Stages::default()).unwrap();
        let p = Plan { flag_reg: p.flag_reg | FLAG_RLE, trigger_word: Some(1), ..p };
        let mut d = Decoder::new(&p, 16);
        // Newest first: 1, then a count word, then 3 three times.
        d.feed(&[1, 0x82, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
        let (s, t) = d.finish();
        assert_eq!(t, Some(16 - 1 - 1));
        assert_eq!(s[16 - 1 - 1], 3);
    }

    #[test]
    fn the_marker_with_demux_pairs() {
        // Channels 0-7 at 200 MHz, ratio 50, one stage: 2 units, the
        // delay 1: the word 4 from the newest, 8 samples.
        let s = Settings {
            flag_reg: FLAG_DEMUX,
            channel_mask: 0xff,
            limit_samples: 16,
            capture_ratio: 50,
            ..settings()
        };
        let st = convert_trigger(Some(&trigger(&[(0, 1, TriggerMatch::One)])), 0xff).unwrap();
        let p = plan(&s, &st).unwrap();
        assert_eq!(p.trigger_word, Some(4));
        let mut d = Decoder::new(&p, 16);
        d.feed(&(0..16u8).rev().collect::<Vec<_>>());
        let (samples, t) = d.finish();
        assert_eq!(t, Some(7));
        assert_eq!(samples[7], 7);
        // With RLE: the newer sample of the run's oldest pair.
        let p = Plan { flag_reg: p.flag_reg | FLAG_RLE, trigger_word: Some(2), ..p };
        let mut d = Decoder::new(&p, 16);
        // Pairs: (1, 2), a count of 1, then (3, 4) twice, the rest.
        d.feed(&[1, 2, 0x01, 0x80, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]);
        let (samples, t) = d.finish();
        assert_eq!(samples.len(), 16);
        // The value word, word 2, holds the run's oldest pair: its newer
        // sample is 4 from the newest.
        assert_eq!(t, Some(16 - 1 - 4));
        assert_eq!(samples[16 - 1 - 4], 3);
    }

    #[test]
    fn demux_rle_pairs() {
        let s = Settings {
            flag_reg: FLAG_DEMUX | FLAG_RLE,
            channel_mask: 0xff,
            limit_samples: 6,
            ..settings()
        };
        let p = plan(&s, &Stages::default()).unwrap();
        let mut d = Decoder::new(&p, 6);
        // A pair (newer 1, older 2) twice, then (3, 4) once. Channel 7,
        // the older sample's top bit, is the flag: 0 in both.
        d.feed(&[0x01, 0x80, 0x81, 2, 3, 4]);
        let (s, _) = d.finish();
        assert_eq!(s, [4, 3, 2, 1, 2, 1]);
    }

    #[test]
    fn the_reads_end_with_what_the_board_sends() {
        // A capture that brings fewer samples than the limit (all 32
        // channels at 200 MHz never reach it in the C driver) ends with
        // the bytes the board was asked for: 2 units of 4 words of 4
        // bytes here.
        let s = Settings { limit_samples: 8, ..settings() };
        let p = plan(&s, &Stages::default()).unwrap();
        let mut d = Decoder::new(&p, 100);
        d.feed(&[0; 16]);
        assert!(!d.done());
        d.feed(&[0; 16]);
        assert!(d.done());
        assert_eq!(d.len(), 8);
    }
}
