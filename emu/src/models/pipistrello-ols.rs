// Counterpart in sigrok:
//   libsigrok/src/hardware/pipistrello-ols/api.c (the scan, the trigger stages, the counts),
//   libsigrok/src/hardware/pipistrello-ols/protocol.c (the commands, the metadata, the sample stream),
//   libsigrok/src/hardware/pipistrello-ols/protocol.h (the opcodes and flags),
//   Pipistrello_ols_verilog 2423ef3, github.com/magnuskarlsson/Pipistrello_ols_verilog (verilog/stage.v, trigger.v, controller.v, rle_enc.v, core.v: stage matching, RLE masks, count widths),
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, magnuskarlsson,
//   Daniel Elstner, Soeren Apel, Gerhard Sittig, Martin Ling,
//   Tilman Sauerbeck, Wolfram Sang, Marek Vasut
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Pipistrello LX45 running the Open Logic Sniffer bitstream, as
//! libsigrok's `pipistrello-ols` driver (`p-ols`) sees it: an FT2232H
//! whose port B carries the SUMP protocol to and from the FPGA, built
//! on the FTDI shell ([`crate::shells::ftdi`]).
//!
//! Every fact below comes from the C driver (`api.c`, `protocol.c`,
//! `protocol.h`); no bitstream source, datasheet or recording of the
//! board was available. What the driver does not pin down is marked
//! "assumed" and is the model's choice.
//!
//! **The chip** is taken for an FT2232H (`0403:6010`, `bcdDevice`
//! 0x0700): the driver opens `0403:6010` (`USB_VENDOR_ID`,
//! `USB_DEVICE_ID`) with `INTERFACE_B`, which only the two port chips
//! have, and the FT2232C shares that id; the H is assumed from the
//! board's published description, not from anything local. Port B must
//! be in 245 FIFO mode from the EEPROM: the driver sets no baud rate
//! beyond libftdi's 9600 Bd of the open, resets the bit mode (back to
//! the EEPROM's mode) and reads 16 KiB at a time. The shell's EEPROM
//! image does not say FIFO (nothing reads it for an FT2232H). Port A
//! (the FPGA's JTAG, assumed) is left to the kernel's `ftdi_sio`, which
//! sets its latency timer at attach; anything else on it is flagged.
//! The product string is `Pipistrello LX45` (`USB_IPRODUCT`, which
//! `ftdi_usb_open_desc` matches); the manufacturer is FTDI's default
//! and there is no serial number (assumed; the driver reads neither).
//!
//! **Commands** are SUMP's, bytes written to port B: a byte with bit 7
//! clear is a short command, one with bit 7 set starts a long one of
//! five bytes, its 32 bit argument little endian. The driver writes
//! each command in a transfer of its own (`write_shortcommand`,
//! `write_longcommand`), which the model checks, and sends only:
//!
//! | byte | command | argument |
//! |---|---|---|
//! | `0x00` | reset | (sent five times: the device could be in a long command) |
//! | `0x01` | run | |
//! | `0x02` | ID | answer `1ALS` (`SLA1` backwards; `1SLO` is the OLS's) |
//! | `0x04` | metadata | answer the TLV block below |
//! | `0x80` | divider | 24 bits, byte 3 zero |
//! | `0x82` | flags | 16 bits, bytes 2 and 3 zero |
//! | `0x83` | delay count | post-trigger words / 4, minus 1 |
//! | `0x84` | read count | words read back / 4, minus 1 |
//! | `0xc0 + 4s` | trigger mask, stage `s` | |
//! | `0xc1 + 4s` | trigger value | |
//! | `0xc2 + 4s` | trigger config | level in bits 16-17, start in bit 27 |
//! | `0xc3 + 4s` | trigger edge (the Pipistrello's) | |
//!
//! Test mode (`0x03`), XON and XOFF (`0x11`, `0x13`) and every other
//! byte are flagged, as are a command split over transfers or two in
//! one, any long command while a capture runs, flags with bits 12 to
//! 15 (the RLE mode, which the driver fixes at 0), both test patterns,
//! or every channel group disabled, a trigger config with a delay, a
//! serial trigger or a channel number (the driver sends none, and the
//! model has none), trigger value or edge bits outside the mask, demux
//! flags whose group bits 4-5 do not repeat bits 2-3 (`api.c` requires
//! the replication, comment before `CMD_SET_FLAGS`), and a run before
//! any read count. Counts are taken as they come, as the OLS takes
//! them: a read count beyond the memory reads back the whole memory, a
//! delay count beyond the read count captures that long (the driver
//! sends 0xffffffff for `captureratio=100`, see below).
//!
//! **The metadata** answer, keys as `p_ols_get_metadata` parses them
//! (type in bits 5-7: 0 a NUL terminated string, 1 a 32 bit big endian
//! integer, 2 a byte; 0x00 ends): `0x01` the name `Pipistrello OLS`,
//! `0x02` the FPGA version `3.07`, `0x20` 32 channels, `0x21` the
//! sample memory in bytes, `0x23` the maximum sample rate 200 MHz,
//! `0x41` protocol version 2, `0x00`. The values are placeholders
//! (assumed): the name follows the driver's long name, the rate its
//! sample rate table; options change them, or replace the block.
//!
//! **Sampling**: 100 MHz divided by the divider plus 1 (`CLOCK_RATE`),
//! or with the demux flag two samples of channels 0-15 per word at that
//! rate. The pins carry the count of samples taken since the run, so
//! channels 0-15 are `t` and 16-31 its complement (sample `t` is
//! `t & 0xffff | !t << 16`), and channel `k` below 16 first rises at
//! `t = 2^k`. A demux word holds samples `2k` (channels 16-31 of the
//! word, groups 3 and 4) and `2k + 1` (groups 1 and 2), the halves the
//! driver puts in that order. Flag bits 2-5 disable channel groups
//! (bytes); a word holds the enabled groups' bytes, in group order, and
//! takes 1, 2 or 4 bytes of memory (3 groups take 4, as the driver
//! says). The test patterns, the channel swap and the noise filter are
//! accepted and not emulated (the pins carry the count regardless).
//!
//! **Triggers**: four stages, each with a mask, a value, an edge mask
//! and a config. At `run` the level is 0. At each word, every stage that
//! has not fired, whose level is at most the current one and whose
//! masked bits equal the value (edge bits also having differed in the
//! word before) fires once: a stage with the start bit starts the
//! capture there, any other raises the level by one from the next word.
//! So the driver's stage after the last one (mask 0, start) fires one
//! word after the user's last stage, and with the start stage's word
//! counted first the word `delay` words from the newest is the match. A
//! start bit on a stage with a mask (only oxrok's four stages send it)
//! counts from the word after its match, as the firmware does at every
//! rate. This is an idealisation: the firmware's pipeline (`stage.v`,
//! `trigger.v`, `controller.v`, `delay_fifo.v`) takes more words, and a
//! hand reading gives, for the word at `trigger_at` with `n` user
//! stages, a place after the last match of 3 at 100 MHz, 2 at 50 MHz, 1
//! at 33 MHz or less, less `n - 1`. Level rises and the start stage's
//! firing cost the same whatever `n`, so the driver's `- num_stages`
//! is not an exact correction, but it lands nearer the match than one
//! fixed offset would. The model's timing does not carry this; a bench
//! would have to confirm it. Without a trigger the driver
//! configures stage 0 alone (mask 0, start), which fires at word 0.
//! These semantics are the OLS's as the driver's configuration implies
//! them (assumed). In demux mode a stage matches when either half of
//! the word does (`stage.v`, `matchL16 | matchH16`), and the driver
//! writes masks to the low half only, so the empty high half matches at
//! every word and every stage fires at once: a trigger at 200 MHz
//! starts the capture at the first words whatever its condition, a
//! libsigrok finding (case `200 MHz trigger`).
//!
//! **Capture**: after the start, `(delay + 1) * 4` more words are
//! written, then the last `(read + 1) * 4` words are sent, newest first,
//! each in its bytes little endian. Words the capture did not write read
//! back as 0 (power-on memory, assumed). With the RLE flag a run of
//! equal words is stored as the word and a count word (the top bit of
//! the word's top byte set, the rest the number of repeats), so the top
//! channel of the enabled groups is lost; the counts count words, and
//! a run still open when the capture ends loses its count. In demux
//! mode the firmware's `rle_enc.v` masks the top channel of both
//! samples of a word (`7f7f`, `7fff7fff`) and does not compress a word
//! whose two samples differ (`demux_mismatch`). A gap against
//! `rle_enc.v`: it stores a run of two as the value twice, where the
//! model stores the value and a count of 1; the samples are the same.
//!
//! **Time** is counted in the host's polls, never the clock: a capture
//! takes no time until the host reads port B, and each read the chip
//! answers with nothing (its two status bytes, after the latency timer
//! on a real chip) stands for one latency period of samples (16 ms, the
//! driver's setting: 1.6 M words at 100 MHz, 0.16 at 10 Hz). Once the
//! capture is complete its bytes are sent in full transfers. With the
//! external clock flag the samples come at `ext-clock=` Hz (assumed
//! wiring, 1 MHz by default) and the divider is ignored (assumed). An
//! FPGA `reset`, a purge of the chip's buffer to the host or the chip's
//! reset drop the bytes not yet sent (the chip's own FIFO is not
//! modelled apart from the FPGA's).
//!
//! The driver's own bugs the model shows. A sample limit that is not a
//! multiple of 4 (8 in demux) gets more samples than asked in the last
//! buffer, and without RLE the driver takes the excess off `rle_count`,
//! 0, which wraps (`protocol.c` 557-563), so its copy loop (597-600)
//! runs off the buffer: sigrok-cli dies of SIGSEGV (cases `101 samples`,
//! `200 MHz 100 samples`). With two
//! or more stages and `captureratio=0` (the default), `trigger_at` is
//! `-num_stages` (`api.c` 524), which `protocol.c` 620 only tests
//! against -1, so 642-645 send `num_stages` samples from before the
//! sample buffer (glibc's chunk header): case `ratio 0 two stages`. A
//! board whose metadata has no memory key (0x21) makes `max_samples`
//! 0, so the read and delay counts go out as 0xffffffff and the driver
//! sends a logic packet of length 0 that sigrok-cli's output dies on
//! (case `no memory capture`). The driver never ends (cases with
//! `--oracle-hangs`): a 4-stage trigger (`3=1010`) gets no start bit
//! (`set_trigger` sets it on stage `num_stages`, 4, which does not
//! exist), so the capture never starts (`four stages`); a 200 MHz
//! capture with the default 32 channels takes each demux word for one
//! sample and waits for the rest (`200 MHz 32 channels`);
//! `captureratio=100` with a trigger sends a delay count of 0xffffffff
//! (`api.c` 523, 562-565), which the device takes as 2^34 words to
//! capture, so at 10 Hz the driver polls for ever (`ratio 100`).
//! sigrok-cli cannot capture without a sample limit
//! (`main.c` 321 needs one, `--time` becomes one), so the driver's
//! read count of 0xffffffff and `g_try_malloc(0)` for a missing limit
//! are out of its reach.
//!
//! Options: `product=`, `manufacturer=`, `serial=` (empty: no string;
//! the driver finds no board whose product is not `Pipistrello LX45`);
//! `id=sla|ols|none|HEX` (the ID answer, `none` an FPGA that does not
//! answer, HEX four bytes); `metadata=none|HEX` (no answer, or the raw
//! block); `name=`, `fpga-version=`, `channels=`, `memory=`,
//! `max-rate=`, `protocol=` (the default block's values; `memory=` is
//! also the memory the board has, whatever the block says); `ext-clock=HZ`; and
//! the FTDI shell's faults ([`Faults`]): `stall=`, `in-stall=`, `idle=`
//! (port B's IN endpoint is `0x83`).

use std::collections::VecDeque;

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Violations};
use crate::shells::ftdi::{
    self, Chip, Event, Faults, Fill, Ftdi, FtdiConfig, FtdiPorts, PortState, bitmode,
};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "pipistrello-ols";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Pipistrello LX45 (FT2232H port B) with the Open Logic Sniffer \
     bitstream, as sigrok's p-ols driver uses it; product=, manufacturer=, serial=, \
     id=sla|ols|none|HEX, metadata=none|HEX, name=, fpga-version=, channels=, memory=, \
     max-rate=, protocol=, ext-clock=HZ; the FTDI shell's faults: stall=REQUEST[/WVALUE], \
     in-stall=N, idle=N";

/// The product string the driver opens (`USB_IPRODUCT`).
pub const PRODUCT: &str = "Pipistrello LX45";
/// The FPGA's sample clock (`CLOCK_RATE`).
pub const CLOCK: u64 = 100_000_000;
/// Port B, 0 based.
const PORT_B: usize = 1;

/// The SUMP command bytes the driver sends (`protocol.h`).
pub mod cmd {
    /// Reset.
    pub const RESET: u8 = 0x00;
    /// Run (arm the trigger).
    pub const RUN: u8 = 0x01;
    /// ID.
    pub const ID: u8 = 0x02;
    /// Test mode (never sent).
    pub const TESTMODE: u8 = 0x03;
    /// Metadata.
    pub const METADATA: u8 = 0x04;
    /// Divider.
    pub const SET_DIVIDER: u8 = 0x80;
    /// Flags.
    pub const SET_FLAGS: u8 = 0x82;
    /// Delay count.
    pub const DELAYCOUNT: u8 = 0x83;
    /// Read count.
    pub const READCOUNT: u8 = 0x84;
    /// The first trigger command (stage 0's mask).
    pub const TRIGGER: u8 = 0xc0;
}

/// The flag register's bits (`protocol.h`).
pub mod flag {
    /// Demux: two samples of channels 0-15 per word.
    pub const DEMUX: u16 = 1 << 0;
    /// Noise filter.
    pub const FILTER: u16 = 1 << 1;
    /// Channel group 1 to 4 disabled: bits 2 to 5.
    pub const GROUPS: u16 = 0x3c;
    /// External clock.
    pub const CLOCK_EXTERNAL: u16 = 1 << 6;
    /// Falling edge of the external clock.
    pub const SLOPE_FALLING: u16 = 1 << 7;
    /// Run length encoding.
    pub const RLE: u16 = 1 << 8;
    /// Swap channels.
    pub const SWAP: u16 = 1 << 9;
    /// External test pattern.
    pub const EXTERNAL_TEST: u16 = 1 << 10;
    /// Internal test pattern.
    pub const INTERNAL_TEST: u16 = 1 << 11;
    /// The RLE mode, which the driver leaves 0.
    pub const RLE_MODE: u16 = 0xf000;
}

/// The trigger config's start bit (`TRIGGER_START << 24`).
const START: u32 = 1 << 27;
/// The trigger config's level field.
const LEVEL: u32 = 3 << 16;

/// One trigger stage's registers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stage {
    /// The channels compared.
    pub mask: u32,
    /// Their levels.
    pub value: u32,
    /// Level (bits 16-17), start (bit 27).
    pub config: u32,
    /// The channels that must also have changed (the Pipistrello's).
    pub edge: u32,
}

impl Stage {
    fn level(self) -> u32 {
        (self.config & LEVEL) >> 16
    }

    fn matches(self, word: u32, prev: Option<u32>) -> bool {
        if (word ^ self.value) & self.mask != 0 {
            return false;
        }
        let edge = self.edge & self.mask;
        edge == 0 || prev.is_some_and(|p| (p ^ self.value) & edge == edge)
    }

    /// The stage on the part of a word `half` selects.
    fn on(self, half: u32) -> Stage {
        Stage { mask: self.mask & half, edge: self.edge & half, ..self }
    }

    /// `stage.v`'s `next_match32Register`: in demux mode either half of
    /// the word matching is a match, so a half with nothing in its mask
    /// always is. The driver writes masks to the low half only, so at
    /// 200 MHz every stage fires at once.
    fn fires(self, word: u32, prev: Option<u32>, demux: bool) -> bool {
        if demux {
            self.on(0xffff).matches(word, prev) || self.on(0xffff_0000).matches(word, prev)
        } else {
            self.matches(word, prev)
        }
    }
}

/// The board as the options make it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Board {
    /// The ID answer, `None` for none.
    pub id: Option<[u8; 4]>,
    /// The metadata answer, `None` for none.
    pub metadata: Option<Vec<u8>>,
    /// The sample memory in bytes.
    pub memory: u32,
    /// The external clock's rate in Hz.
    pub ext_clock: u64,
}

/// The metadata block of the given values.
pub fn metadata(
    name: &str,
    fpga: &str,
    channels: u32,
    memory: u32,
    max_rate: u32,
    protocol: u8,
) -> Vec<u8> {
    let mut m = vec![0x01];
    m.extend_from_slice(name.as_bytes());
    m.extend_from_slice(&[0, 0x02]);
    m.extend_from_slice(fpga.as_bytes());
    m.extend_from_slice(&[0, 0x20]);
    m.extend_from_slice(&channels.to_be_bytes());
    m.push(0x21);
    m.extend_from_slice(&memory.to_be_bytes());
    m.push(0x23);
    m.extend_from_slice(&max_rate.to_be_bytes());
    m.extend_from_slice(&[0x41, protocol, 0x00]);
    m
}

impl Default for Board {
    fn default() -> Board {
        Board {
            id: Some(*b"1ALS"),
            metadata: Some(metadata("Pipistrello OLS", "3.07", 32, 1 << 20, 200_000_000, 2)),
            memory: 1 << 20,
            ext_clock: 1_000_000,
        }
    }
}

/// The pins at sample `t` since the run.
pub fn stimulus(t: u64) -> u32 {
    let low = t as u16;
    u32::from(low) | (u32::from(!low) << 16)
}

/// The bits of the read and delay counts the firmware keeps.
const COUNT_MASK: u32 = 0x03ff_ffff;

/// `rle_enc.v`'s `data_mask` and `demux_mismatch` on a compacted word
/// of `width` bytes: the value stored, the top channel gone (of both
/// samples in demux mode), and whether the two samples of a demux word
/// differ, which stops it being compressed.
fn rle_value(demux: bool, width: u32, word: u32) -> (u32, bool) {
    let (mask, half) = match (demux, width) {
        (true, 2) => (0x7f7f, 8),
        (true, 4) => (0x7fff_7fff, 16),
        _ => (!(1u32 << (8 * width - 1)), 0),
    };
    let v = word & mask;
    (v, half != 0 && (v >> half) != (v & ((1 << half) - 1)))
}

/// A capture in progress, from a run.
#[derive(Clone, Debug)]
struct Capture {
    demux: bool,
    rle: bool,
    /// The enabled groups, bit 0 group 1.
    groups: u8,
    /// Bytes per word sent.
    width: u32,
    /// Clock ticks per word, or 0 for the external clock.
    period: u64,
    ext_clock: u64,
    read_words: u64,
    delay_words: u64,
    stages: [Stage; 4],
    /// Raw words sampled so far.
    t: u64,
    prev: Option<u32>,
    level: u32,
    fired: [bool; 4],
    /// Words written when the start stage fired.
    mark: Option<u64>,
    /// The last words written, at most `read_words`.
    mem: VecDeque<u32>,
    written: u64,
    /// RLE: the value of the open run and its repeats.
    run: Option<(u32, u32)>,
    /// Ticks (or Hz ms) not yet turned into words.
    acc: u64,
    done: bool,
}

impl Capture {
    /// The raw word `k`.
    fn raw(&self, k: u64) -> u32 {
        if self.demux {
            (stimulus(2 * k) << 16) | (stimulus(2 * k + 1) & 0xffff)
        } else {
            stimulus(k)
        }
    }

    /// The enabled groups' bytes of `w`, packed.
    fn compact(&self, w: u32) -> u32 {
        let mut out = 0;
        let mut at = 0;
        for g in 0..4 {
            if self.groups & (1 << g) != 0 {
                out |= ((w >> (8 * g)) & 0xff) << (8 * at);
                at += 1;
            }
        }
        out
    }

    fn rle_flag(&self) -> u32 {
        1 << (8 * self.width - 1)
    }

    fn push(&mut self, word: u32) {
        self.mem.push_back(word);
        if self.mem.len() as u64 > self.read_words {
            self.mem.pop_front();
        }
        self.written += 1;
        if self.mark.is_some_and(|m| self.written.saturating_sub(m) >= self.delay_words) {
            self.done = true;
        }
    }

    /// Samples one word.
    fn sample(&mut self) {
        let w = self.raw(self.t);
        if self.mark.is_none() {
            let (mut start, mut bump) = (false, false);
            for s in 0..4 {
                let st = self.stages[s];
                if self.fired[s] || st.level() > self.level || !st.fires(w, self.prev, self.demux)
                {
                    continue;
                }
                self.fired[s] = true;
                if st.config & START != 0 {
                    start = true;
                } else {
                    bump = true;
                }
            }
            if start {
                // A start stage with a mask of its own counts from the
                // next word; the driver's follows a match by one word.
                let own = self.stages.iter().zip(self.fired).any(|(st, fired)| {
                    fired && st.config & START != 0 && st.mask != 0
                });
                self.mark = Some(self.written + u64::from(own));
            } else if bump {
                self.level += 1;
            }
        }
        self.prev = Some(w);
        self.t += 1;
        let c = self.compact(w);
        if !self.rle {
            self.push(c);
            return;
        }
        let flag = self.rle_flag();
        let (v, mismatch) = rle_value(self.demux, self.width, c);
        match self.run {
            Some((cur, n)) if cur == v && !mismatch && n < flag - 1 => {
                self.run = Some((cur, n + 1));
            }
            run => {
                if let Some((_, n)) = run
                    && n > 0
                {
                    self.push(n | flag);
                    if self.done {
                        return;
                    }
                }
                self.push(v);
                self.run = Some((v, 0));
            }
        }
    }

    /// One latency period of `ms` milliseconds: the words it samples.
    fn advance(&mut self, ms: u8) {
        let (per, unit) = if self.period == 0 {
            (u64::from(ms) * self.ext_clock, 1000)
        } else {
            (u64::from(ms) * (CLOCK / 1000), self.period)
        };
        self.acc += per;
        let mut n = self.acc / unit;
        self.acc %= unit;
        while n > 0 && !self.done {
            self.sample();
            n -= 1;
        }
    }

    /// The bytes sent once done: the last `read_words` words, newest
    /// first.
    fn bytes(&self) -> Vec<u8> {
        let width = self.width as usize;
        let missing = (self.read_words as usize).saturating_sub(self.mem.len());
        let mut out = Vec::with_capacity(self.read_words as usize * width);
        for w in self.mem.iter().rev().copied().chain(std::iter::repeat_n(0, missing)) {
            out.extend_from_slice(&w.to_le_bytes()[..width]);
        }
        out
    }
}

/// The FPGA behind port B.
#[derive(Clone, Debug)]
pub struct Fpga {
    board: Board,
    /// A long command's bytes so far.
    partial: Vec<u8>,
    divider: u32,
    flags: u16,
    read: Option<u32>,
    delay: u32,
    stages: [Stage; 4],
    /// Bytes for the host.
    out: VecDeque<u8>,
    capture: Option<Capture>,
}

impl Fpga {
    /// The FPGA of `board` at power on.
    pub fn new(board: Board) -> Fpga {
        Fpga {
            board,
            partial: Vec::new(),
            divider: 0,
            flags: 0,
            read: None,
            delay: 0,
            stages: [Stage::default(); 4],
            out: VecDeque::new(),
            capture: None,
        }
    }

    fn running(&self) -> bool {
        self.capture.as_ref().is_some_and(|c| !c.done)
    }

    fn short(&mut self, c: u8, v: &mut Violations) {
        if self.running() && c != cmd::RESET {
            v.report(format!("SUMP command {c:#04x} while a capture runs; only a reset stops it"));
        }
        match c {
            cmd::RESET => {
                self.capture = None;
                self.out.clear();
            }
            cmd::RUN => self.run(v),
            cmd::ID => {
                if let Some(id) = self.board.id {
                    self.out.extend(id);
                }
            }
            cmd::METADATA => {
                if let Some(m) = &self.board.metadata {
                    self.out.extend(m.iter().copied());
                }
            }
            cmd::TESTMODE | 0x11 | 0x13 => {
                v.report(format!("SUMP command {c:#04x}: the p-ols driver never sends it"));
            }
            other => v.report(format!("SUMP short command {other:#04x}: no such command")),
        }
    }

    fn long(&mut self, b: &[u8], v: &mut Violations) {
        let (op, arg) = (b[0], u32::from_le_bytes([b[1], b[2], b[3], b[4]]));
        if self.running() {
            v.report(format!("SUMP command {op:#04x} while a capture runs"));
            return;
        }
        match op {
            cmd::SET_DIVIDER => {
                if b[4] != 0 {
                    v.report(format!("divider {arg:#010x}: 24 bits, byte 3 is 0"));
                }
                self.divider = arg & 0x00ff_ffff;
            }
            cmd::SET_FLAGS => {
                if arg > 0xffff {
                    v.report(format!("flags {arg:#010x}: 16 bits, bytes 2 and 3 are 0"));
                }
                let f = arg as u16;
                if f & flag::RLE_MODE != 0 {
                    v.report(format!("flags {f:#06x}: RLE mode bits 12-15, which p-ols keeps 0"));
                }
                if f & flag::INTERNAL_TEST != 0 && f & flag::EXTERNAL_TEST != 0 {
                    v.report(format!("flags {f:#06x}: both test patterns"));
                }
                if f & flag::GROUPS == flag::GROUPS {
                    v.report(format!("flags {f:#06x}: every channel group disabled"));
                }
                // api.c, before CMD_SET_FLAGS: in demux mode the lower two
                // channel_disable bits must be replicated to the upper two.
                if f & flag::DEMUX != 0 && (f >> 2) & 3 != (f >> 4) & 3 {
                    v.report(format!(
                        "flags {f:#06x}: demux with groups 3 and 4 not replicating groups 1 and 2"
                    ));
                }
                self.flags = f;
            }
            cmd::DELAYCOUNT => self.delay = arg,
            cmd::READCOUNT => self.read = Some(arg),
            0xc0..=0xcf => {
                let s = &mut self.stages[usize::from((op - cmd::TRIGGER) / 4)];
                match op & 3 {
                    0 => s.mask = arg,
                    1 => s.value = arg,
                    2 => {
                        if arg & !(LEVEL | START) != 0 {
                            v.report(format!(
                                "trigger config {arg:#010x} (command {op:#04x}): a delay, \
                                 channel or serial mode, which p-ols never sets"
                            ));
                        }
                        s.config = arg;
                    }
                    _ => s.edge = arg,
                }
            }
            other => v.report(format!("SUMP long command {other:#04x}: no such command")),
        }
    }

    fn run(&mut self, v: &mut Violations) {
        let Some(read) = self.read else {
            v.report("SUMP run before any read count");
            return;
        };
        for (i, s) in self.stages.iter().enumerate() {
            if (s.value | s.edge) & !s.mask != 0 {
                v.report(format!(
                    "trigger stage {i}: value {:#010x} or edge {:#010x} outside mask {:#010x}",
                    s.value, s.edge, s.mask
                ));
            }
        }
        let groups = (!(self.flags & flag::GROUPS) >> 2) as u8 & 0xf;
        let width = groups.count_ones();
        // controller.v's fwd and bwd have 26 bits.
        let read_words = (u64::from(read & COUNT_MASK) + 1) * 4;
        let delay_words = (u64::from(self.delay & COUNT_MASK) + 1) * 4;
        let stored = u64::from(if width == 3 { 4 } else { width });
        let capacity = u64::from(self.board.memory) / stored.max(1);
        self.out.clear();
        self.capture = Some(Capture {
            demux: self.flags & flag::DEMUX != 0,
            rle: self.flags & flag::RLE != 0,
            groups,
            width,
            period: if self.flags & flag::CLOCK_EXTERNAL != 0 {
                0
            } else {
                u64::from(self.divider) + 1
            },
            ext_clock: self.board.ext_clock,
            read_words: read_words.min(capacity),
            delay_words,
            stages: self.stages,
            t: 0,
            prev: None,
            level: 0,
            fired: [false; 4],
            mark: None,
            mem: VecDeque::new(),
            written: 0,
            run: None,
            acc: 0,
            done: width == 0,
        });
    }

    /// The bytes the host wrote to port B.
    pub fn write(&mut self, data: &[u8], v: &mut Violations) {
        let mut commands = 0;
        let resumed = !self.partial.is_empty();
        for &b in data {
            if self.partial.is_empty() && b & 0x80 == 0 {
                self.short(b, v);
                commands += 1;
                continue;
            }
            self.partial.push(b);
            if self.partial.len() == 5 {
                let c = std::mem::take(&mut self.partial);
                self.long(&c, v);
                commands += 1;
            }
        }
        if resumed || !self.partial.is_empty() || commands != 1 {
            v.report(format!(
                "a write of {} bytes to the FPGA: p-ols writes each command, 1 or 5 bytes, in \
                 a transfer of its own",
                data.len()
            ));
        }
    }

    /// Up to `max` bytes for the host, after one latency period of
    /// `ms` when it has none.
    pub fn read(&mut self, max: usize, ms: u8) -> Fill {
        if self.out.is_empty()
            && let Some(c) = &mut self.capture
            && !c.done
        {
            c.advance(ms);
            if c.done {
                self.out.extend(c.bytes());
            }
        }
        if self.out.is_empty() {
            return Fill::Idle;
        }
        let n = max.min(self.out.len());
        Fill::Data(self.out.drain(..n).collect())
    }

    /// The chip dropped what it held for the host.
    pub fn purge(&mut self) {
        self.out.clear();
    }
}

impl FtdiPorts for Fpga {
    fn changed(&mut self, port: usize, event: Event, state: &PortState, v: &mut Violations) {
        if port != PORT_B {
            if event != Event::Latency {
                v.report(format!(
                    "{event:?} on port {}: p-ols uses port B only",
                    ftdi::port_letter(port)
                ));
            }
            return;
        }
        match event {
            Event::Bitmode if state.bitmode != bitmode::RESET => v.report(format!(
                "bit mode {:#04x} on port B: the FPGA talks through the 245 FIFO",
                state.bitmode
            )),
            Event::Reset | Event::FlushToHost => self.purge(),
            _ => {}
        }
    }

    fn write(&mut self, port: usize, data: &[u8], _: &PortState, v: &mut Violations) -> Reply {
        if port == PORT_B {
            Fpga::write(self, data, v);
        } else {
            v.report(format!(
                "{} bytes written to port {}: p-ols uses port B only",
                data.len(),
                ftdi::port_letter(port)
            ));
        }
        Reply::ack()
    }

    fn read(&mut self, port: usize, max: usize, state: &PortState, v: &mut Violations) -> Fill {
        if port != PORT_B {
            v.report(format!("read from port {}: p-ols uses port B only", ftdi::port_letter(port)));
            return Fill::Idle;
        }
        Fpga::read(self, max, state.latency)
    }

    fn pins(&mut self, port: usize, _: &PortState, v: &mut Violations) -> u8 {
        v.report(format!("READ_PINS on port {}: p-ols reads no pins", ftdi::port_letter(port)));
        0
    }
}

/// The chip's configuration with these strings.
pub fn config(manufacturer: Option<&str>, product: Option<&str>, serial: Option<&str>) -> FtdiConfig {
    let mut c = FtdiConfig::new(Chip::Ft2232h);
    c.manufacturer = manufacturer.map(str::to_owned);
    c.description = product.map(str::to_owned);
    c.serial = serial.map(str::to_owned);
    c
}

fn parse_hex(key: &str, s: &str) -> Result<Vec<u8>> {
    if !s.is_ascii() || !s.len().is_multiple_of(2) {
        return Err(Error(format!("{key}={s}: an even number of hex digits")));
    }
    (0..s.len())
        .step_by(2)
        .map(|i| {
            u8::from_str_radix(&s[i..i + 2], 16)
                .map_err(|_| Error(format!("{key}={s}: not hex")))
        })
        .collect()
}

fn number<T: std::str::FromStr>(key: &str, s: &str) -> Result<T> {
    s.parse().map_err(|_| Error(format!("{key}={s}: not a number")))
}

/// A board from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a bad value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut board = Board::default();
    let mut faults = Faults::default();
    let (mut manufacturer, mut product, mut serial) =
        (Some("FTDI".to_owned()), Some(PRODUCT.to_owned()), None);
    let mut name = "Pipistrello OLS".to_owned();
    let mut fpga = "3.07".to_owned();
    let (mut channels, mut max_rate, mut protocol) = (32u32, 200_000_000u32, 2u8);
    let mut raw: Option<Option<Vec<u8>>> = None;
    for (k, v) in options {
        let s = (!v.is_empty()).then(|| v.clone());
        match k.as_str() {
            "manufacturer" => manufacturer = s,
            "product" => product = s,
            "serial" => serial = s,
            "id" => {
                board.id = match v.as_str() {
                    "sla" => Some(*b"1ALS"),
                    "ols" => Some(*b"1SLO"),
                    "none" => None,
                    hex => Some(parse_hex(k, hex)?.try_into().map_err(|_| {
                        Error(format!("id={hex}: sla, ols, none or four bytes in hex"))
                    })?),
                }
            }
            "metadata" => {
                raw = Some(if v == "none" { None } else { Some(parse_hex(k, v)?) });
            }
            "name" => name.clone_from(v),
            "fpga-version" => fpga.clone_from(v),
            "channels" => channels = number(k, v)?,
            "memory" => board.memory = number(k, v)?,
            "max-rate" => max_rate = number(k, v)?,
            "protocol" => protocol = number(k, v)?,
            "ext-clock" => board.ext_clock = number(k, v)?,
            other => {
                if !faults.option(other, v)? {
                    return Err(Error(format!("{NAME} has no option {other}")));
                }
            }
        }
    }
    board.metadata = raw.unwrap_or_else(|| {
        Some(metadata(&name, &fpga, channels, board.memory, max_rate, protocol))
    });
    let c = config(manufacturer.as_deref(), product.as_deref(), serial.as_deref());
    let mut model = PipistrelloOls::new(c, board);
    model.chip.set_faults(faults);
    Ok(Box::new(model))
}

/// The emulated board.
pub struct PipistrelloOls {
    chip: Ftdi,
    fpga: Fpga,
}

impl PipistrelloOls {
    /// The board at power on.
    pub fn new(config: FtdiConfig, board: Board) -> PipistrelloOls {
        PipistrelloOls { chip: Ftdi::new(config), fpga: Fpga::new(board) }
    }
}

impl DeviceModel for PipistrelloOls {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        self.chip.descriptors()
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.control(&mut self.fpga, setup, data, v)
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.out(&mut self.fpga, ep, data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        self.chip.in_transfer(&mut self.fpga, ep, max, v)
    }

    fn reset(&mut self) {
        self.chip.reset();
        self.fpga = Fpga::new(self.fpga.board.clone());
    }

    fn log_digest_above(&self) -> Option<usize> {
        Some(256)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn board() -> PipistrelloOls {
        PipistrelloOls::new(config(Some("FTDI"), Some(PRODUCT), None), Board::default())
    }

    fn long(op: u8, arg: u32) -> Vec<u8> {
        let mut b = vec![op];
        b.extend_from_slice(&arg.to_le_bytes());
        b
    }

    /// The payload of an IN transfer of port B, status bytes dropped.
    fn read(dev: &mut PipistrelloOls, v: &mut Violations) -> Vec<u8> {
        let Reply::Data(d) = dev.in_transfer(3, 16384, v) else { panic!() };
        d.chunks(512).flat_map(|p| p[2..].to_vec()).collect()
    }

    fn write(dev: &mut PipistrelloOls, b: &[u8], v: &mut Violations) {
        assert_eq!(dev.out(4, b, v), Reply::ack());
    }

    /// What `dev_acquisition_start` sends without a trigger.
    fn configure(dev: &mut PipistrelloOls, flags: u16, divider: u32, read: u32, v: &mut Violations) {
        write(dev, &long(0xc0, 0), v);
        write(dev, &long(0xc1, 0), v);
        write(dev, &long(0xc2, START), v);
        write(dev, &long(0xc3, 0), v);
        write(dev, &long(cmd::SET_DIVIDER, divider), v);
        write(dev, &long(cmd::READCOUNT, read), v);
        write(dev, &long(cmd::DELAYCOUNT, read), v);
        write(dev, &long(cmd::SET_FLAGS, u32::from(flags)), v);
    }

    /// The capture of the board's FPGA.
    fn dev_capture(dev: &PipistrelloOls) -> &Capture {
        dev.fpga.capture.as_ref().unwrap()
    }

    #[test]
    fn scan_answers() {
        let mut dev = board();
        let mut v = Violations::default();
        for _ in 0..5 {
            write(&mut dev, &[cmd::RESET], &mut v);
        }
        write(&mut dev, &[cmd::ID], &mut v);
        assert_eq!(read(&mut dev, &mut v), b"1ALS");
        write(&mut dev, &[cmd::METADATA], &mut v);
        let m = read(&mut dev, &mut v);
        assert_eq!(&m[..16], b"\x01Pipistrello OLS");
        assert_eq!(&m[m.len() - 3..], &[0x41, 2, 0]);
        assert_eq!(read(&mut dev, &mut v), b"");
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn capture_newest_first() {
        let mut dev = board();
        let mut v = Violations::default();
        // 16 samples, all groups, 100 MHz, filter on.
        configure(&mut dev, flag::FILTER, 0, 3, &mut v);
        write(&mut dev, &[cmd::RUN], &mut v);
        let d = read(&mut dev, &mut v);
        assert_eq!(d.len(), 64);
        assert_eq!(&d[..4], &stimulus(15).to_le_bytes());
        assert_eq!(&d[60..], &[0, 0, 0xff, 0xff]);
        assert_eq!(read(&mut dev, &mut v), b"");
        // Groups 2 and 3 alone (channels 8-23): two bytes per sample.
        configure(&mut dev, flag::FILTER | 0x24, 0, 3, &mut v);
        write(&mut dev, &[cmd::RUN], &mut v);
        let d = read(&mut dev, &mut v);
        assert_eq!(d.len(), 32);
        assert_eq!(&d[..2], &[0x00, 0xf0]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn slow_captures_take_polls() {
        let mut dev = board();
        let mut v = Violations::default();
        // 1 kHz: 16 words a poll; 64 words take four.
        configure(&mut dev, flag::FILTER, 99_999, 15, &mut v);
        write(&mut dev, &[cmd::RUN], &mut v);
        for _ in 0..3 {
            assert_eq!(read(&mut dev, &mut v), b"");
        }
        assert_eq!(read(&mut dev, &mut v).len(), 256);
        // A reset in the middle drops the capture.
        write(&mut dev, &[cmd::RUN], &mut v);
        assert_eq!(read(&mut dev, &mut v), b"");
        write(&mut dev, &[cmd::RESET], &mut v);
        for _ in 0..5 {
            assert_eq!(read(&mut dev, &mut v), b"");
        }
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn trigger_one_word_after_the_user_stage() {
        let mut dev = board();
        let mut v = Violations::default();
        // Channel 4 rising (first at t = 16), then stage 1 starts; half
        // the 64 words before.
        let rows = [(0xc0, 0x10), (0xc1, 0x10), (0xc2, 0), (0xc3, 0x10)];
        for (op, arg) in rows {
            write(&mut dev, &long(op, arg), &mut v);
        }
        for (op, arg) in [(0xc4, 0), (0xc5, 0), (0xc6, (1 << 16) | START), (0xc7, 0)] {
            write(&mut dev, &long(op, arg), &mut v);
        }
        for s in 2..4u8 {
            for (k, arg) in [(0, 0), (1, 0), (2, 3 << 16), (3, 0)] {
                write(&mut dev, &long(0xc0 + 4 * s + k, arg), &mut v);
            }
        }
        write(&mut dev, &long(cmd::SET_DIVIDER, 0), &mut v);
        write(&mut dev, &long(cmd::READCOUNT, 15), &mut v);
        write(&mut dev, &long(cmd::DELAYCOUNT, 7), &mut v);
        write(&mut dev, &long(cmd::SET_FLAGS, u32::from(flag::FILTER)), &mut v);
        write(&mut dev, &[cmd::RUN], &mut v);
        let d = read(&mut dev, &mut v);
        let words: Vec<u32> =
            d.chunks(4).rev().map(|c| u32::from_le_bytes(c.try_into().unwrap())).collect();
        // The start at t = 17; 32 words from there, 32 before.
        assert_eq!(words.len(), 64);
        assert_eq!(words[32], stimulus(17));
        // The 15 words before t = 0 were never written.
        assert_eq!(words[..15], [0; 15]);
        assert_eq!(words[15], stimulus(0));
        // The driver's trigger_at: (16 - 8) * 4 - 1 = 31, the rising edge.
        assert_eq!(words[31] & 0x10, 0x10);
        assert_eq!(words[30] & 0x10, 0);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_start_bit_on_a_stage_with_a_mask_counts_from_the_next_word() {
        let mut dev = board();
        let mut v = Violations::default();
        // Channel 4 high (first at t = 16) with the start bit: half of
        // the 64 words from t = 17 on, so word 31 is the match.
        for (op, arg) in [(0xc0, 0x10), (0xc1, 0x10), (0xc2, START), (0xc3, 0)] {
            write(&mut dev, &long(op, arg), &mut v);
        }
        write(&mut dev, &long(cmd::SET_DIVIDER, 0), &mut v);
        write(&mut dev, &long(cmd::READCOUNT, 15), &mut v);
        write(&mut dev, &long(cmd::DELAYCOUNT, 7), &mut v);
        write(&mut dev, &long(cmd::SET_FLAGS, u32::from(flag::FILTER)), &mut v);
        write(&mut dev, &[cmd::RUN], &mut v);
        let d = read(&mut dev, &mut v);
        let words: Vec<u32> =
            d.chunks(4).rev().map(|c| u32::from_le_bytes(c.try_into().unwrap())).collect();
        assert_eq!(words[31], stimulus(16));
        assert_eq!(words[32], stimulus(17));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn counts_keep_26_bits_as_the_firmware() {
        // 0xffffffff is 2^28 words, not 2^34: done after 168 polls of
        // 1.6 M words at 100 MHz.
        let mut dev = board();
        let mut v = Violations::default();
        configure(&mut dev, flag::FILTER, 0, 3, &mut v);
        write(&mut dev, &long(cmd::DELAYCOUNT, 0xffff_ffff), &mut v);
        write(&mut dev, &[cmd::RUN], &mut v);
        let capture = dev_capture(&dev);
        assert_eq!(capture.delay_words, 1 << 28);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn demux_stages_match_on_either_half() {
        let st = Stage { mask: 0x10, value: 0x10, edge: 0x10, config: 0 };
        // Not in demux mode: the condition holds or not.
        assert!(!st.fires(0, Some(0), false));
        assert!(st.fires(0x10, Some(0), false));
        assert!(!st.fires(0x10, None, false));
        // In demux mode the empty high half matches at every word.
        assert!(st.fires(0, Some(0), true));
        assert!(st.fires(0, None, true));
        // Masks in both halves: either half decides.
        let both = Stage { mask: 0x0001_0001, value: 0x0001_0001, edge: 0, config: 0 };
        assert!(!both.fires(0, None, true));
        assert!(both.fires(0x0001_0000, None, true));
        assert!(both.fires(0x0000_0001, None, true));
        assert!(!both.fires(0x0000_0001, None, false));
    }

    #[test]
    fn rle_masks_as_the_firmware() {
        // Without demux, the top channel of the word.
        assert_eq!(rle_value(false, 1, 0xff), (0x7f, false));
        assert_eq!(rle_value(false, 2, 0xffff), (0x7fff, false));
        assert_eq!(rle_value(false, 4, 0xffff_ffff), (0x7fff_ffff, false));
        // In demux mode of both samples: equal samples compress.
        assert_eq!(rle_value(true, 2, 0xffff), (0x7f7f, false));
        assert_eq!(rle_value(true, 4, 0xffff_ffff), (0x7fff_7fff, false));
        // Samples that differ after the mask do not.
        assert_eq!(rle_value(true, 2, 0x0102), (0x0102, true));
        assert_eq!(rle_value(true, 4, 0x8001_0002), (0x0001_0002, true));
        // Differing only in the masked channel: equal.
        assert_eq!(rle_value(true, 2, 0x8101), (0x0101, false));
    }

    #[test]
    fn rle_and_demux() {
        let mut dev = board();
        let mut v = Violations::default();
        // Group 2 alone (channels 8-15), RLE: a run of 256 is the value
        // with 127 repeats, twice (the count has 7 bits).
        configure(&mut dev, flag::FILTER | flag::RLE | 0x34, 0, 3, &mut v);
        write(&mut dev, &[cmd::RUN], &mut v);
        let mut d = read(&mut dev, &mut v);
        d.reverse();
        assert_eq!(d.len(), 16);
        assert_eq!(&d[..6], &[0x00, 0xff, 0x00, 0xff, 0x01, 0xff]);
        // Demux, groups 1 and 2: a word is samples 2k (high) and 2k+1.
        // Channels 0-7 as the driver asks: groups 2 and 4 disabled,
        // group 3 replicating group 1, so bytes of samples 2k+1 and 2k.
        configure(&mut dev, flag::DEMUX | 0x28, 0, 0, &mut v);
        write(&mut dev, &[cmd::RUN], &mut v);
        let d = read(&mut dev, &mut v);
        assert_eq!(d, [7, 6, 5, 4, 3, 2, 1, 0]);
        configure(&mut dev, flag::DEMUX, 0, 0, &mut v);
        write(&mut dev, &[cmd::RUN], &mut v);
        let d = read(&mut dev, &mut v);
        assert_eq!(&d[..4], &[7, 0, 6, 0]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_what_p_ols_never_sends() {
        let mut dev = board();
        let mut v = Violations::default();
        let mut one = |b: &[u8], what: &str| {
            dev.out(4, b, &mut v);
            assert!(!v.drain().is_empty(), "{what}");
        };
        one(&[cmd::RUN], "run before a read count");
        one(&[0x80, 0, 0], "a split long command");
        one(&[0, 0, 0], "the rest of it");
        one(&[cmd::RESET, cmd::RESET], "two commands at once");
        one(&[cmd::TESTMODE], "test mode");
        one(&[0x05], "no such short command");
        one(&long(0x81, 0), "no such long command");
        one(&long(cmd::SET_DIVIDER, 0x0100_0000), "divider byte 3");
        one(&long(cmd::SET_FLAGS, 0x1_0000), "flags byte 2");
        one(&long(cmd::SET_FLAGS, 0x1000), "RLE mode");
        one(&long(cmd::SET_FLAGS, 0x0c00), "both test patterns");
        one(&long(cmd::SET_FLAGS, 0x3c), "no group");
        one(&long(cmd::SET_FLAGS, 0x31), "demux, groups 3 and 4 not replicated");
        one(&long(0xc2, 5), "a trigger delay");
        let mut dev = board();
        let mut v = Violations::default();
        // Counts as the OLS takes them: a delay beyond the read count
        // captures that long (here, for ever).
        dev.out(4, &long(cmd::SET_FLAGS, 0), &mut v);
        dev.out(4, &long(cmd::READCOUNT, 3), &mut v);
        dev.out(4, &long(cmd::DELAYCOUNT, 0xffff_ffff), &mut v);
        dev.out(4, &[cmd::RUN], &mut v);
        assert!(v.is_empty(), "{v:?}");
        // The capture never ends: commands during it.
        dev.out(4, &[cmd::ID], &mut v);
        dev.out(4, &long(cmd::SET_FLAGS, 0), &mut v);
        assert_eq!(v.drain().len(), 2, "commands while capturing");
        dev.out(4, &long(0xc0, 1), &mut v);
        dev.out(4, &[cmd::RESET], &mut v);
        dev.out(4, &long(0xc1, 2), &mut v);
        dev.out(4, &long(cmd::DELAYCOUNT, 3), &mut v);
        dev.out(4, &[cmd::RUN], &mut v);
        assert_eq!(v.drain().len(), 2, "a capture's command, a value outside the mask");
        dev.out(2, &[0], &mut v);
        assert_eq!(v.drain().len(), 1, "port A");
        dev.in_transfer(1, 512, &mut v);
        assert_eq!(v.drain().len(), 1, "a read of port A");
        let s = Setup { request_type: 0x40, request: 0x0b, value: 0x01ff, index: 2, length: 0 };
        dev.control(s, &[], &mut v);
        assert_eq!(v.drain().len(), 1, "bit-bang on port B");
        let s = Setup { request_type: 0x40, request: 0x09, value: 16, index: 1, length: 0 };
        dev.control(s, &[], &mut v);
        assert!(v.is_empty(), "the kernel's latency timer on port A");
    }

    /// A read count beyond the memory reads back the whole memory.
    #[test]
    fn read_count_beyond_the_memory() {
        let mut dev = PipistrelloOls::new(
            config(Some("FTDI"), Some(PRODUCT), None),
            Board { memory: 64, ..Board::default() },
        );
        let mut v = Violations::default();
        configure(&mut dev, flag::FILTER, 0, 0xffff_ffff, &mut v);
        write(&mut dev, &long(cmd::DELAYCOUNT, 3), &mut v);
        write(&mut dev, &[cmd::RUN], &mut v);
        let d = read(&mut dev, &mut v);
        assert_eq!(d.len(), 64);
        assert_eq!(&d[..4], &stimulus(15).to_le_bytes());
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn options() {
        let o = |k: &str, v: &str| (k.to_owned(), v.to_owned());
        let m = build(&[]).unwrap();
        assert_eq!(m.descriptors().strings, ["FTDI", PRODUCT]);
        assert_eq!(m.descriptors().bcd_device(), 0x0700);
        let m = build(&[o("product", "Other"), o("serial", "X1")]).unwrap();
        assert_eq!(m.descriptors().strings, ["FTDI", "Other", "X1"]);
        assert!(build(&[o("id", "123")]).is_err());
        assert!(build(&[o("id", "31534c4f")]).is_ok());
        assert!(build(&[o("metadata", "zz")]).is_err());
        assert!(build(&[o("speed", "1")]).is_err());
        assert!(build(&[o("in-stall", "2")]).is_ok());
        let mut dev = PipistrelloOls::new(
            config(None, Some(PRODUCT), None),
            Board { id: None, metadata: None, ..Board::default() },
        );
        let mut v = Violations::default();
        write(&mut dev, &[cmd::ID], &mut v);
        assert_eq!(read(&mut dev, &mut v), b"");
        write(&mut dev, &[cmd::METADATA], &mut v);
        assert_eq!(read(&mut dev, &mut v), b"");
    }
}
