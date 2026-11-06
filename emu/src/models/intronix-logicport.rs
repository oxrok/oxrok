// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The Intronix `LogicPort` LA1034: an FT245BL in FIFO mode in front of an
//! SRAM based Cyclone FPGA, configured by the host in bit-bang mode.
//!
//! Sources, strongest first; each behaviour below names its own.
//!
//! - **[rec]** the two sessions recorded on a real unit
//!   (`tests/fixtures/logicport/capture-*.txt`) and the FTDI probe
//!   (`tests/fixtures/ftdi/logicport-probe.txt`). The unit tests hold the
//!   model to them, byte for byte on everything but the status-only reads
//!   between a request and its answer, whose number was the real chip's
//!   latency timer.
//! - **[hil]** the pass criteria of the bench checks in
//!   `hil/oxrok-hil/src/main.rs`, which ran on the two real units.
//! - **[c]** the C driver of libsigrok PR 315 (`intronix-logicport`,
//!   `api.c`, `protocol.c`, `protocol.h`), its comments included.
//! - **[rs]** the register notes in `src/drivers/logicport/protocol.rs`,
//!   for what only oxrok's driver uses (counts, durations, value ranges,
//!   the frequency counter). They were worked out from the bench, so they
//!   are as good as what [hil] checks and no better.
//! - **[guess]** said where it stands. Nothing checks these against a
//!   unit; a case resting on one can only show the driver and the guess
//!   agree.
//!
//! What the recordings show:
//!
//! - The descriptors, the 64 EEPROM words, modem status 0x31 [rec].
//! - Commands: five bytes, `c1` write, `c2` read, `c3` segment; ack is
//!   opcode, sequence number big endian, then the data. A resync's first
//!   ack has sequence 0, each command adds 1 [rec].
//! - The pins, in bit-bang and FIFO mode: bits 5-7 pulled up, bit 3
//!   nSTATUS, bit 4 `CONF_DONE`, bits 0-2 what the host drives, or pulled up
//!   when not driven: 0xef unconfigured in FIFO mode, 0xec after nCONFIG
//!   rises, 0xff after the last bit of a bitstream [rec].
//! - After a reconfiguration, the first answer starts with eleven stale
//!   bytes that two purges did not remove, `ff*8 ee 00 00` [rec]. That
//!   makes the first command fail, and the driver resyncs. What they are
//!   is a guess (bit-bang echo).
//! - The init counter reads a steady 0x10 right after a reconfiguration
//!   [rec]; it counted for 250 ms in the comment [c], which `init-steps=N`
//!   models by reads.
//! - Status 0x73 at the first poll, with bit 4 nobody documents [rec]. Bit
//!   4 is kept set at all times [guess].
//! - The sample memory [rec]: a ring of 2048 rows of 40 bits, reads wrap
//!   within a lane; bit 0 of the flags is set on data rows from the marked
//!   row on (the drivers ignore it); the marked row is 7 samples after the
//!   match; a run of L samples is a data row and, for L >= 2, a count row
//!   holding L - 2; the marked row is a data row even without a transition.
//!   Without compression `post + 1` rows follow the marked row, with
//!   compression `post`. The trigger position register is the number of
//!   rows after the marked row, plus 1.
//!
//! Everything about the capture is simulated from the inputs, so a wrong
//! register or depth changes the samples rather than only a golden. The
//! inputs are given in time, as the ESP32 generator gives them on the bench
//! (`clk=`, `seq=`), not in samples, so that the timebase shows: the bench's
//! "100 samples per period at 100 MHz against 1 MHz" is a property of the
//! model's clock tables, which come from [c] and not from arithmetic on the
//! rate. Time is in half nanoseconds, so the 2.5 ns sample points fit.
//!
//! [guess] items in the capture: no trigger programmed (combine 0) fires at
//! once when pre-filled, as a capture without one does complete [hil]; a
//! trigger only runs once the pre-trigger rows are written, which status
//! bit 6 and `REG_END_PREFILL` suggest; more than one edge in a mask is an
//! OR; compressed memory that stays idle ends the phase at `END_*` with
//! `rle-idle=N` samples elapsed; the qualifier input is the clock input not
//! in use ([hil] only tests CLK2 against CLK1); the 500 MHz trigger position
//! is whatever makes the C driver's empirical 21 samples land on the match,
//! so a case at 500 MHz cannot find a fault in that constant.
//!
//! The FPGA's upload: nCONFIG low releases a configuration only after the
//! `0x100000` write [c]; bits go in on DCLK rising, LSB first [c]; the
//! stream must start with the RBF header [c]. The model takes two kinds of
//! bitstream. A stand-in (`tools/emu-firmware.sh`) is checked block by block
//! and its length exactly, so a reordered or cut upload leaves `CONF_DONE` low
//! with nSTATUS low, as a CRC error does ([guess]; a stand-in cannot show
//! what the real parts do). A real image is told by a digest of its first
//! block and its length (from the vendor's file, build 2371), so that a run
//! against that file works; its bytes are not otherwise checked, and none
//! are here. Anything else is refused (`any-upload=IMAGE:BYTES` takes it,
//! for the unit test that plays the recording's upload).
//!
//! An unconfigured FPGA does not drain the FIFO [c: "doesn't drain the
//! FIFO"]: a write NAKs until the host's timeout. A real FT245 takes its
//! 384 byte buffer first; here it takes nothing (a stated gap: the tool sees
//! the same failure).
//!
//! Faults, off by default: `image=none|N` at power on (7, as [rec] found
//! it), `id=0xNN` the image id read back; `stale=0`; `poweron-silent=N`
//! commands ignored until N resyncs after a configuration ([c]'s "ignore
//! commands for more than a second"); `init-steps=N`, `init-stuck=1`;
//! `nstatus-stuck=1`, `conf-done-stuck=N` uploads, `wrong-image=N`; `silent-after=N`
//! commands, then no answer until power off; `silent-reg=ADDR` that register
//! never answers; `bad-seq=N` the Nth answer's sequence is wrong; `short=N`
//! the Nth readout of a lane stops half way; `fill=N` status polls before
//! done; `cal=+003|none` the calibration string; and the shell's
//! `stall=REQUEST[/WVALUE]`, `in-stall=N`, `idle=N`. Inputs: `d=MASK` the
//! level of D0-D31 ([rec]'s unit read 0x22280000), `clk=HZ`, `clk2=HZ`,
//! `seq=MASK:NS,...` over `d`, by channel bit, `seq-repeat=N`, `lead=NS`
//! from the arm, `volts=V` the high level (3.3, the ESP32), `ring=N` the row
//! the ring starts at, `rle-idle=N`.

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Violations};
use crate::shells::ftdi::{
    Chip, Event, Faults, Fill, Ftdi, FtdiConfig, FtdiPorts, PortState, bitmode,
};
use crate::{Error, Result};
use sha2::{Digest as _, Sha256};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "intronix-logicport";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Intronix LogicPort LA1034 (FT245BL, Cyclone FPGA); image=none|N, id=, \
     d=, clk=, clk2=, seq=, seq-repeat=, lead=, volts=, ring=, rle-idle=, fill=, cal=; faults: stale=0, \
     poweron-silent=, init-steps=, init-stuck=1, nstatus-stuck=1, conf-done-stuck=N, wrong-image=, \
     silent-after=, silent-reg=, bad-seq=, short=, the FTDI shell's stall=, in-stall=, idle=";

const DEPTH: usize = 2048;
const BANK: usize = DEPTH / 2;

const REG_STATUS: u32 = 0x0010_0000;
const REG_ARM: u32 = 0x0010_0002;
const REG_RUN: u32 = 0x0010_0003;
const REG_PRE: u32 = 0x0010_0004;
const REG_POST: u32 = 0x0010_0006;
const REG_THRESHOLD: u32 = 0x0010_0008;
const REG_PREFILL: u32 = 0x0010_000a;
const REG_POSTFILL: u32 = 0x0010_000b;
const REG_COMBINE: u32 = 0x0010_000d;
const REG_IMAGE_ID: u32 = 0x0010_000f;
const REG_LEVEL_A: u32 = 0x0020_0000;
const REG_LEVEL_B: u32 = 0x0040_0000;
const REG_CLOCK: u32 = 0x0100_0000;
const REG_CHANNEL_CFG: u32 = 0x0100_0001;
const REG_COMPRESSION: u32 = 0x0100_0002;
const REG_COMPRESSION_MASK: u32 = 0x0100_0003;
const REG_INVERT: u32 = 0x0100_0008;
const REG_WP: u32 = 0x0200_0000;
const REG_WP_B: u32 = 0x0200_0100;
const MEM_A: u32 = 0x0200_1000;
const MEM_B: u32 = 0x0200_5000;
const MEM_FLAGS: u32 = 0x0200_9000;
const REG_PRESCALER: u32 = 0x2000_0000;
const REG_STEP: u32 = 0x2000_0001;
const REG_FREQ: u32 = 0x4000_0000;
const REG_FREQ_VALUE: u32 = 0x4000_0001;
const REG_INIT: u32 = 0x4000_0005;

/// The rate table: prescaler, step, half nanoseconds per sample [c:
/// `logicport_rates`]. 500 MHz shares 0, 0 with 250 MHz; the image tells.
const RATES: [(u8, u8, u64); 18] = [
    (0x61, 0x00, 2_000_000),
    (0x51, 0x44, 1_000_000),
    (0x51, 0x11, 400_000),
    (0x51, 0x00, 200_000),
    (0x41, 0x44, 100_000),
    (0x41, 0x11, 40_000),
    (0x41, 0x00, 20_000),
    (0x31, 0x44, 10_000),
    (0x31, 0x11, 4_000),
    (0x31, 0x00, 2_000),
    (0x21, 0x44, 1_000),
    (0x21, 0x11, 400),
    (0x21, 0x00, 200),
    (0x11, 0x44, 100),
    (0x11, 0x11, 40),
    (0x11, 0x00, 20),
    (0x01, 0x00, 10),
    (0x00, 0x00, 8),
];

/// Image names as the stand-ins are made [c: `image_names`].
const IMAGES: [&str; 8] = [
    "state-after",
    "state-before-clk1-rise",
    "state-before-clk2-rise",
    "state-before-clk1-fall",
    "state-before-clk2-fall",
    "state-on",
    "timing-ddr",
    "timing-sdr",
];
/// The image sizes in the vendor's file (build 2371), RBF bytes. These
/// numbers are in `src/drivers/logicport/firmware.rs` too.
const REAL_SIZES: [usize; 8] = [66115, 65282, 65431, 65458, 65407, 65937, 64452, 66149];
/// SHA-256 of the 32 bytes after the sync of each image in that file, first
/// eight bytes: how the model tells a real image, so that a run against the
/// vendor's file works. A digest; no vendor bytes.
const REAL_HEADS: [u64; 8] = [
    0x0f69_f09f_bf96_9ff9,
    0x73ba_07ab_c17a_11dd,
    0x9cb0_e972_14bb_7628,
    0xdad2_a55b_235e_4002,
    0xcce5_c2b2_22ae_a779,
    0xf53f_a206_f8fe_dc94,
    0x4142_3a5a_ec7d_6a90,
    0xd1ed_1019_4f2a_6333,
];
/// A stand-in image's RBF bytes (`tools/emu-firmware.sh`): small, each its
/// own, for three or more of the drivers' 10000 byte chunks in the upload.
pub fn standin_size(image: usize) -> usize {
    3000 + 16 * image
}
/// Bytes of RBF sync at the start of each stream.
const RBF_HEAD: usize = 17;
const EXTRA_CLOCKS: u32 = 32;

/// Stand-in image `image`'s RBF byte `block`, in blocks of 32.
fn standin_block(image: usize, block: usize) -> [u8; 32] {
    Sha256::digest(format!("intronix-logicport-{}:{block}", IMAGES[image]).as_bytes()).into()
}

/// A whole stand-in image as RBF, for the tests.
#[cfg(test)]
pub fn standin_rbf(image: usize) -> Vec<u8> {
    let mut v = vec![0xff; 16];
    v.push(0x6a);
    let n = standin_size(image);
    let mut block = 0;
    while v.len() < n {
        v.extend_from_slice(&standin_block(image, block));
        block += 1;
    }
    v.truncate(n);
    v[n - 1] = 0xff;
    v
}

/// One trigger level's registers.
#[derive(Clone, Copy, Debug, Default)]
struct Level {
    pattern_mask: u64,
    pattern_value: u64,
    rising: u64,
    falling: u64,
    value_mask: u64,
    value_low: u64,
    value_high: u64,
    count_high: u32,
    count_low: u32,
    condition: u8,
    source: u8,
    mode: u8,
    consecutive: bool,
}

/// A level's counter.
#[derive(Clone, Copy, Debug, Default)]
struct Counter {
    count: u32,
    prev_true: bool,
    fired: bool,
}

/// `s`'s bits of `mask`, packed with the lowest channel least significant.
fn gather(s: u64, mask: u64) -> u64 {
    let mut out = 0;
    let mut k = 0;
    for bit in 0..40 {
        if mask >> bit & 1 != 0 {
            out |= (s >> bit & 1) << k;
            k += 1;
        }
    }
    out
}

impl Level {
    fn condition_true(&self, s: u64) -> bool {
        let pattern = (s ^ self.pattern_value) & self.pattern_mask == 0;
        let pat = match self.condition & 3 {
            1 => pattern,
            2 => !pattern,
            _ => true,
        };
        let val = match self.condition & 0x1c {
            c @ (0x0c | 0x10) => {
                let (x, lo, hi) = (
                    gather(s, self.value_mask),
                    gather(self.value_low, self.value_mask),
                    gather(self.value_high, self.value_mask),
                );
                let inside = (lo..=hi).contains(&x);
                if c == 0x0c { inside } else { !inside }
            }
            _ => true,
        };
        pat && val
    }

    /// One sample: whether the level fires on it. The counter rules: [rs],
    /// with what [hil] checks: `D0=r*3` fires on the third rise with the
    /// condition true, `D0=r*3c` starts over at a rise with it false, a
    /// duration fires `count_low` samples into the condition and is
    /// cumulative without the consecutive flag.
    fn step(&self, c: &mut Counter, prev: u64, s: u64) -> bool {
        let cond = self.condition_true(s);
        let edge = (!prev & s & self.rising) | (prev & !s & self.falling) != 0;
        let fired = match self.mode {
            1 => {
                match self.source {
                    3 if edge => {
                        if cond {
                            c.count += 1;
                        } else if self.consecutive {
                            c.count = 0;
                        }
                    }
                    3 => {}
                    1 => {
                        if cond && !c.prev_true {
                            c.count += 1;
                        }
                    }
                    _ => {
                        if cond {
                            c.count += 1;
                        } else if self.consecutive {
                            c.count = 0;
                        }
                    }
                }
                c.count > self.count_high
            }
            2 => {
                if cond {
                    c.count += 1;
                } else if self.consecutive {
                    c.count = 0;
                }
                c.count > self.count_low
            }
            _ => false,
        };
        c.prev_true = cond;
        fired
    }
}

/// The inputs, in time. Time is half nanoseconds after power on.
#[derive(Clone, Debug)]
struct Inputs {
    d: u64,
    clk: u64,
    clk2: u64,
    seq: Vec<(u64, u64)>,
    repeat: u32,
    lead: Option<u64>,
    volts: f64,
}

impl Inputs {
    fn seq_len(&self) -> u64 {
        self.seq.iter().map(|s| s.1).sum::<u64>() * u64::from(self.repeat)
    }

    /// The channels' levels at time `t`, the sequence having started at
    /// `start`.
    fn at(&self, t: u64, start: u64) -> u64 {
        let mut v = self.d;
        if !self.seq.is_empty() && t >= start && t - start < self.seq_len() {
            let cycle: u64 = self.seq.iter().map(|s| s.1).sum();
            let mut left = (t - start) % cycle;
            for &(mask, len) in &self.seq {
                if left < len {
                    v |= mask;
                    break;
                }
                left -= len;
            }
        }
        // A level is high in odd half periods: low at time 0, rising after
        // half a period.
        for (bit, hz) in [(32, self.clk), (33, self.clk2)] {
            if hz != 0 && half_index(t, hz) & 1 == 1 {
                v |= 1 << bit;
            }
        }
        v
    }

    fn quiet_after(&self, start: u64) -> Option<u64> {
        (self.clk == 0 && self.clk2 == 0).then(|| start + self.seq_len())
    }
}

/// Half periods of `hz` elapsed at `t`.
fn half_index(t: u64, hz: u64) -> u64 {
    (u128::from(t) * u128::from(hz) / 1_000_000_000) as u64
}

/// The first time half period `i` of `hz` is in force.
fn half_start(i: u64, hz: u64) -> u64 {
    (u128::from(i) * 1_000_000_000).div_ceil(u128::from(hz)) as u64
}

/// A capture being written.
#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)]
struct Capture {
    lanes: Vec<[u8; 5]>,
    /// Bank B in DDR mode.
    lanes_b: Vec<[u8; 4]>,
    t: u64,
    seq_start: u64,
    prev: u64,
    written: u64,
    written_b: u64,
    prefilled: bool,
    fired: bool,
    /// Samples from the match to the marked row.
    marker_left: u32,
    marked: bool,
    after: u64,
    /// The sample count of the match, at DDR.
    match_sample: u64,
    samples: u64,
    last: Option<u64>,
    pending: u64,
    done: bool,
    never: bool,
    trigger_pos: u16,
    a: Counter,
    b: Counter,
    edge_index: Option<u64>,
    idle_left: u64,
}

/// Registers the capture reads.
#[derive(Clone, Debug, Default)]
struct Regs {
    prescaler: u8,
    step: u8,
    clock: u8,
    compression: bool,
    mask: u64,
    invert: u64,
    run: bool,
    pre: u16,
    post: u16,
    dac: u16,
    dac_set: bool,
    combine: u8,
    levels: [Level; 2],
}

/// An upload in progress.
#[derive(Clone, Debug, Default)]
struct Upload {
    bytes: usize,
    bit: u8,
    acc: u8,
    window: Vec<u8>,
    block: usize,
    standin: Option<usize>,
    /// The image, and the length that ends it, once the first block says.
    known: Option<(usize, usize)>,
    bad: bool,
}

/// The FPGA and what the FT245's pins reach.
#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)]
pub struct Fpga {
    // Options.
    power_image: Option<u8>,
    id_override: Option<u8>,
    stale_on: bool,
    poweron_silent: u32,
    init_steps: u32,
    init_stuck: bool,
    nstatus_stuck: bool,
    conf_done_fails: u32,
    wrong_image: Option<u8>,
    silent_after: Option<u32>,
    silent_reg: Option<u32>,
    bad_seq: Option<u32>,
    short: Option<u32>,
    fill: u32,
    ring: usize,
    rle_idle: u64,
    cal: i32,
    /// `any-upload=IMAGE:BYTES`: any stream with the header is this image.
    any_upload: Option<(usize, usize)>,
    inputs: Inputs,
    // The chip's pins.
    latch: u8,
    in_bitbang: bool,
    // The FPGA.
    image: Option<u8>,
    released: bool,
    nstatus: bool,
    conf_done: bool,
    upload: Option<Upload>,
    extra: u32,
    pending_image: Option<u8>,
    stale: bool,
    ignore: u32,
    init_read: u32,
    // The link.
    seq: u16,
    segment: u16,
    hdr: Vec<u8>,
    data: Option<(u8, u16, usize, Vec<u8>)>,
    out: VecDeque<u8>,
    answered: u32,
    lane_reads: u32,
    // The capture.
    regs: Regs,
    capture: Option<Capture>,
    polls: u32,
    time: u64,
    // The frequency counter.
    freq_input: Option<u8>,
    freq_prev: Option<u8>,
    freq_poll: u32,
    freq_reads: u32,
}

impl Fpga {
    /// At power on, with the default inputs [rec].
    fn new() -> Fpga {
        Fpga {
            power_image: Some(7),
            id_override: None,
            stale_on: true,
            poweron_silent: 0,
            init_steps: 0,
            init_stuck: false,
            nstatus_stuck: false,
            conf_done_fails: 0,
            wrong_image: None,
            silent_after: None,
            silent_reg: None,
            bad_seq: None,
            short: None,
            fill: 0,
            ring: 0,
            rle_idle: 20_000,
            cal: -4,
            any_upload: None,
            inputs: Inputs {
                d: 0x2228_0000,
                clk: 1_000_000,
                clk2: 0,
                seq: Vec::new(),
                repeat: 1,
                lead: None,
                volts: 3.3,
            },
            latch: 0x07,
            in_bitbang: false,
            image: Some(7),
            released: false,
            nstatus: true,
            conf_done: true,
            upload: None,
            extra: 0,
            pending_image: None,
            stale: false,
            ignore: 0,
            init_read: 0,
            seq: 0,
            segment: 0,
            hdr: Vec::new(),
            data: None,
            out: VecDeque::new(),
            answered: 0,
            lane_reads: 0,
            regs: Regs::default(),
            capture: None,
            polls: 0,
            time: 0,
            freq_input: None,
            freq_prev: None,
            freq_poll: 0,
            freq_reads: 0,
        }
    }

    fn set_power_image(&mut self, image: Option<u8>) {
        self.power_image = image;
        self.image = image;
        self.conf_done = image.is_some();
    }

    fn period(&self) -> u64 {
        if self.image == Some(6) {
            return 4;
        }
        RATES
            .iter()
            .find(|r| r.0 == self.regs.prescaler && r.1 == self.regs.step)
            .map_or(2_000, |r| r.2)
    }

    /// The volts the threshold DAC code stands for: the inverse of the
    /// transfer function [c: `threshold_dac`], with the unit's calibration.
    fn threshold_volts(&self) -> f64 {
        let code = f64::from(self.regs.dac) - f64::from(self.cal);
        let v = 4.559 - code * 5.875 / 1023.0;
        (v - 1.315) / -0.402_038_109_862_497_4 + 1.315
    }

    /// The inputs as the comparators and the inversion make them [hil: at a
    /// threshold of 4.0 V, 3.3 V logic reads 0, at -1.0 V, 0 V reads 1, and
    /// inverted inputs come back inverted from the sample memory].
    fn sample_at(&self, t: u64, start: u64) -> u64 {
        const ALL: u64 = (1 << 34) - 1;
        let thr = self.threshold_volts();
        let raw = self.inputs.at(t, start);
        let raw = match (self.inputs.volts > thr, thr < 0.0) {
            (true, true) => ALL,
            (true, false) => raw,
            (false, true) => ALL & !raw,
            (false, false) => 0,
        };
        raw ^ self.regs.invert
    }

    /// A clock mode's clock input, edge and qualifier.
    fn state_clock(&self) -> (u64, bool, Option<bool>, u64) {
        let m = self.regs.clock;
        let clk2 = m & 2 != 0;
        let hz = if clk2 { self.inputs.clk2 } else { self.inputs.clk };
        let falling = (m & 1 != 0) != (self.regs.invert >> if clk2 { 33 } else { 32 } & 1 != 0);
        let qualifier = if m & 0x10 != 0 { None } else { Some(m & 8 == 0) };
        (hz, falling, qualifier, if clk2 { 1 << 32 } else { 1 << 33 })
    }

    fn timing(&self) -> bool {
        matches!(self.image, Some(6 | 7))
    }

    /// The offset of the sample from the clock edge, by image [c: comment
    /// on the images; rs: 2.5 ns].
    fn sample_offset(&self) -> i64 {
        match self.image {
            Some(0) => 5,
            Some(1..=4) => -5,
            _ => 0,
        }
    }

    fn arm(&mut self, v: &mut Violations) {
        if !self.regs.dac_set {
            v.report("armed with no threshold written");
        }
        if self.regs.compression && self.period() < 10 {
            v.report("compression above 200 MHz: the C driver and oxrok's refuse it");
        }
        if !self.timing() {
            // The before images are for one clock and effective edge [rs].
            let (_, falling, _, _) = self.state_clock();
            let clk2 = self.regs.clock & 2 != 0;
            let want = match (clk2, falling) {
                (false, false) => 1,
                (true, false) => 2,
                (false, true) => 3,
                (true, true) => 4,
            };
            if matches!(self.image, Some(1..=4)) && self.image != Some(want) {
                v.report(format!(
                    "image {:?} samples before another clock edge than mode {:#04x} takes",
                    self.image, self.regs.clock
                ));
            }
        }
        let period = self.period();
        let seq_start = self.time
            + self.inputs.lead.unwrap_or_else(|| {
                if self.regs.compression {
                    16 * period
                } else {
                    (u64::from(self.regs.pre) + 2) * period
                }
            });
        self.capture = Some(Capture {
            lanes: vec![[0; 5]; DEPTH],
            lanes_b: vec![[0; 4]; BANK],
            t: self.time,
            seq_start,
            prev: self.sample_at(self.time, seq_start),
            written: 0,
            written_b: 0,
            prefilled: false,
            fired: false,
            marker_left: 0,
            marked: false,
            after: 0,
            match_sample: 0,
            samples: 0,
            last: None,
            pending: 0,
            done: false,
            never: false,
            trigger_pos: 0,
            a: Counter::default(),
            b: Counter::default(),
            edge_index: None,
            idle_left: self.rle_idle,
        });
        self.polls = 0;
    }

    fn put_row(c: &mut Capture, ring: usize, value: u64, flags: u8) {
        let row = (ring + c.written as usize) % DEPTH;
        c.lanes[row] =
            [value as u8, (value >> 8) as u8, (value >> 16) as u8, (value >> 24) as u8, flags];
        c.written += 1;
    }

    fn data_flags(value: u64, marked: bool) -> u8 {
        (value >> 32 & 1) as u8 * 2 + (value >> 33 & 1) as u8 * 4 + u8::from(marked)
    }

    /// The trigger logic on one sample.
    #[allow(clippy::many_single_char_names)]
    fn trigger_step(&self, c: &mut Capture, s: u64) {
        let r = &self.regs;
        let prev = c.prev;
        let fire = match r.combine {
            // Nothing programmed: a capture ends anyway [hil], at once [guess].
            0 => true,
            1 => r.levels[0].step(&mut c.a, prev, s),
            2 => r.levels[1].step(&mut c.b, prev, s),
            3 => {
                let a = r.levels[0].step(&mut c.a, prev, s);
                let b = r.levels[1].step(&mut c.b, prev, s);
                a || b
            }
            4 => {
                if !c.a.fired {
                    c.a.fired = r.levels[0].step(&mut c.a, prev, s);
                }
                if !c.b.fired {
                    c.b.fired = r.levels[1].step(&mut c.b, prev, s);
                }
                c.a.fired && c.b.fired
            }
            _ => {
                if c.a.fired {
                    r.levels[1].step(&mut c.b, prev, s)
                } else {
                    c.a.fired = r.levels[0].step(&mut c.a, prev, s);
                    false
                }
            }
        };
        if fire {
            c.fired = true;
            // A duration's mark lands a sample late [hil, and the driver's
            // comment: "measured on the bench at 1 MHz"].
            let late = u32::from(r.levels.iter().any(|l| l.mode == 2));
            c.marker_left = 7 + late;
            c.match_sample = c.samples;
        }
    }

    /// The next sample's time, in state mode the next qualified clock edge.
    fn next_time(&self, c: &mut Capture) -> Option<u64> {
        if self.timing() {
            return Some(c.t + self.period());
        }
        let (hz, falling, qualifier, qual_bit) = self.state_clock();
        if hz == 0 {
            return None;
        }
        // Rising edges are the odd half periods.
        let mut i = if let Some(i) = c.edge_index {
            i + 2
        } else {
            let i = half_index(c.t, hz) + 1;
            if (i & 1 == 1) == falling { i + 1 } else { i }
        };
        for _ in 0..1 << 18 {
            let t = half_start(i, hz);
            let high = self.inputs.at(t, c.seq_start) & qual_bit != 0;
            if qualifier.is_none_or(|q| q == high) {
                c.edge_index = Some(i);
                return Some(t);
            }
            i += 2;
        }
        None
    }

    /// Writes rows until the capture ends or has to wait for the host. The
    /// reply is held up by this, so it stops after a bounded work.
    #[allow(clippy::too_many_lines)]
    fn run_capture(&mut self) {
        let Some(mut c) = self.capture.take() else { return };
        if c.never {
            self.capture = Some(c);
            return;
        }
        let ddr = self.image == Some(6);
        let rle = self.regs.compression;
        let pre = u64::from(self.regs.pre);
        let post = u64::from(self.regs.post);
        let ring = self.ring;
        let quiet = self.inputs.quiet_after(c.seq_start);
        let offset = self.sample_offset();
        let mut budget = 1u32 << 19;
        while !c.done && budget > 0 {
            budget -= 1;
            // A compressed capture waits, while nothing changes, for the
            // host's END_ writes.
            if rle && !c.prefilled && !self.inputs.seq.is_empty() && c.t >= c.seq_start {
                break;
            }
            if rle && quiet.is_some_and(|q| c.t >= q) {
                if !c.prefilled {
                    break;
                }
                if c.fired && c.idle_left == 0 {
                    break;
                }
                c.idle_left = c.idle_left.saturating_sub(1);
            }
            if !rle
                && quiet.is_some_and(|q| c.t >= q)
                && c.prefilled
                && !c.fired
                && self.regs.combine != 0
            {
                // Nothing will change, a trigger never comes: stay running.
                c.never = true;
                break;
            }
            let Some(t) = self.next_time(&mut c) else {
                c.never = true;
                break;
            };
            c.t = t;
            let s = self.sample_at(t.saturating_add_signed(offset), c.seq_start);
            c.prefilled |= c.written >= pre;
            if c.prefilled && !c.fired {
                self.trigger_step(&mut c, s);
            }
            let mut mark = false;
            if c.fired && !c.marked {
                if c.marker_left == 0 {
                    mark = true;
                    c.marked = true;
                } else {
                    c.marker_left -= 1;
                }
            }
            if ddr {
                // Even samples in bank A, odd in bank B [c: read_ddr].
                let row = (c.samples as usize / 2) % BANK;
                let clk = Self::data_flags(s, c.marked);
                if c.samples & 1 == 0 {
                    c.lanes[row] = [s as u8, (s >> 8) as u8, (s >> 16) as u8, (s >> 24) as u8, clk];
                    c.written += 1;
                } else {
                    c.lanes_b[row] = [s as u8, (s >> 8) as u8, (s >> 16) as u8, (s >> 24) as u8];
                    c.lanes[row][4] = (c.lanes[row][4] & 0x0f) | clk << 4;
                    c.written_b += 1;
                }
                if c.marked {
                    c.after += u64::from(c.samples & 1 == 1);
                    if c.after > post && c.samples & 1 == 1 {
                        // The window is the last 2047 samples; the position
                        // lands the C driver's minus 21 on the match.
                        let rows = c.samples.div_ceil(2);
                        let oldest = (rows - BANK as u64) * 2;
                        let idx = c.match_sample - oldest;
                        c.trigger_pos = (DEPTH as u64 - (idx + 21)).max(1) as u16;
                        c.done = true;
                    }
                }
            } else if rle {
                let new_run = c.last.is_none_or(|l| (l ^ s) & self.regs.mask != 0);
                if new_run || mark {
                    if c.pending > 0 {
                        let n = c.pending - 1;
                        let flags = 0x08 | (n >> 32 & 7) as u8;
                        Self::put_row(&mut c, ring, n & 0xffff_ffff, flags);
                        c.pending = 0;
                        if c.marked && !mark {
                            c.after += 1;
                        }
                    }
                    if c.marked && !(c.done) && c.after >= post {
                        c.done = true;
                    } else {
                        let flags = Self::data_flags(s, c.marked);
                        Self::put_row(&mut c, ring, s, flags);
                        if c.marked && !mark {
                            c.after += 1;
                        }
                        c.last = Some(s);
                        if c.marked && c.after >= post {
                            c.done = true;
                        }
                    }
                } else {
                    c.pending += 1;
                }
            } else {
                let flags = Self::data_flags(s, c.marked);
                Self::put_row(&mut c, ring, s, flags);
                if c.marked {
                    if !mark {
                        c.after += 1;
                    }
                    if c.after > post {
                        c.done = true;
                    }
                }
            }
            c.prev = s;
            c.samples += 1;
        }
        if c.done && !ddr {
            c.trigger_pos = (c.after + 1).min(0xffff) as u16;
        }
        self.time = c.t;
        self.capture = Some(c);
    }

    fn end_postfill(&mut self) {
        self.run_capture();
        let Some(c) = self.capture.as_mut() else { return };
        if c.fired && !c.done {
            // A pending count row goes out with the end [guess].
            if c.pending > 0 {
                let n = c.pending - 1;
                let flags = 0x08 | (n >> 32 & 7) as u8;
                let ring = self.ring;
                Self::put_row(c, ring, n & 0xffff_ffff, flags);
                c.pending = 0;
                c.after += 1;
            }
            c.trigger_pos = (c.after + 1).min(0xffff) as u16;
            c.done = true;
        }
    }

    fn status(&mut self) -> u8 {
        self.run_capture();
        let mut s = 0x10;
        let Some(c) = &self.capture else { return s };
        if c.prefilled {
            s |= 0x40;
        }
        if c.fired {
            s |= 0x20;
        }
        if c.done {
            if self.polls < self.fill {
                self.polls += 1;
                return s;
            }
            s |= 1;
            let full = c.written >= if self.image == Some(6) { BANK as u64 } else { DEPTH as u64 };
            if full {
                s |= 2;
            }
        }
        s
    }

    fn write_pointer(&self, bank_b: bool) -> u16 {
        let Some(c) = &self.capture else { return 0 };
        if self.image == Some(6) {
            let w = if bank_b { c.written_b } else { c.written };
            return ((w + BANK as u64 - 1) % BANK as u64) as u16;
        }
        ((self.ring as u64 + c.written + DEPTH as u64 - 1) % DEPTH as u64) as u16
    }

    fn unfilled(&self) -> u16 {
        let w = self.capture.as_ref().map_or(0, |c| c.written) as usize;
        let d = if self.image == Some(6) { BANK } else { DEPTH };
        // x = unfilled + 1, the window starts wp + 1 + x: row 0.
        if self.image == Some(6) {
            return (BANK + d - w.min(d) - 1) as u16;
        }
        (DEPTH - 1 - w.min(DEPTH - 1)) as u16
    }

    /// `len` bytes of memory at offset `off` of the segment.
    fn read_memory(&self, addr: u32, len: usize) -> Vec<u8> {
        let Some(c) = &self.capture else { return vec![0; len] };
        let ddr = self.image == Some(6);
        let wrap = if ddr { BANK } else { DEPTH };
        if addr >= MEM_FLAGS {
            let at = (addr - MEM_FLAGS) as usize;
            return (0..len).map(|i| c.lanes[(at + i) % wrap][4]).collect();
        }
        if addr >= MEM_B {
            let lane = ((addr - MEM_B) / 0x1000) as usize;
            let at = ((addr - MEM_B) & 0xfff) as usize;
            return (0..len).map(|i| c.lanes_b[(at + i) % BANK][lane]).collect();
        }
        let lane = ((addr - MEM_A) / 0x1000) as usize;
        let at = ((addr - MEM_A) & 0xfff) as usize;
        (0..len).map(|i| c.lanes[(at + i) % wrap][lane]).collect()
    }

    /// What a read of a register holds. Unknown addresses are reported.
    fn read_reg(&mut self, addr: u32, len: usize, v: &mut Violations) -> Vec<u8> {
        let le = |x: u64, n: usize| x.to_le_bytes()[..n].to_vec();
        match addr {
            REG_STATUS if len == 1 => vec![self.status()],
            // Only the C and oxrok drivers' uses are known; both widths [rs].
            REG_PREFILL if len == 2 => le(u64::from(self.unfilled()), 2),
            REG_IMAGE_ID if len == 1 => {
                let id = self.image.map_or(0, |i| 0x10 | i);
                vec![self.id_override.unwrap_or(id)]
            }
            REG_CLOCK if len == 2 => {
                le(u64::from(self.capture.as_ref().map_or(0, |c| c.trigger_pos)), 2)
            }
            REG_WP if len == 2 => le(u64::from(self.write_pointer(false)), 2),
            REG_WP_B if len == 2 => le(u64::from(self.write_pointer(true)), 2),
            REG_INIT if len == 1 => {
                if self.init_stuck {
                    return vec![0];
                }
                let v = 0x10u32.saturating_sub(self.init_steps.saturating_sub(self.init_read));
                self.init_read += 1;
                vec![v as u8]
            }
            REG_FREQ if len == 1 => {
                // Ready every third poll: a free running gate [rs].
                self.freq_poll += 1;
                vec![u8::from(!self.freq_poll.is_multiple_of(3))]
            }
            REG_FREQ_VALUE if len == 4 => {
                // The first reading after a select can be the previous
                // input's [rs].
                let input = if self.freq_reads == 0 { self.freq_prev } else { self.freq_input };
                self.freq_reads += 1;
                let hz = match input {
                    Some(0x20) => self.inputs.clk,
                    Some(0x21) => self.inputs.clk2,
                    _ => 0,
                };
                le(hz / 10, 4)
            }
            a if (MEM_A..MEM_FLAGS + DEPTH as u32).contains(&a) => self.read_memory(a, len),
            other => {
                v.report(format!("read of {len} bytes at {other:#010x}, which no driver reads"));
                vec![0; len]
            }
        }
    }

    fn write_level(&mut self, which: usize, off: u32, val: u64, len: usize, v: &mut Violations) {
        let l = &mut self.regs.levels[which];
        match (off, len) {
            (0x00, 5) => l.pattern_mask = val,
            (0x05, 5) => l.pattern_value = val,
            (0x0a, 4) => l.count_high = val as u32,
            (0x0e, 4) => l.count_low = val as u32,
            (0x12, 1) => {
                if val != 1 {
                    v.report("value bit order other than 1: no driver writes it");
                }
            }
            (0x14, 1) => l.condition = val as u8,
            (0x16, 1) => l.source = val as u8,
            (0x17, 1) => l.mode = val as u8,
            (0x18, 1) => l.consecutive = val != 0,
            (0x19, 1) => {
                if val != 0 {
                    v.report("prequalify other than 0: unused");
                }
            }
            (0x1a, 5) => l.value_mask = val,
            (0x1f, 5) => l.value_low = val,
            (0x24, 5) => l.value_high = val,
            (0x29, 5) => l.rising = val,
            (0x2e, 5) => l.falling = val,
            _ => v.report(format!("level write of {len} bytes at offset {off:#04x}")),
        }
        if matches!(off, 0x0a | 0x0e) && val > 0xf_ffff {
            v.report(format!("trigger counter {val:#x}: the counters are 20 bits"));
        }
        if matches!(off, 0x00 | 0x05 | 0x1a | 0x1f | 0x24 | 0x29 | 0x2e) && val >> 34 != 0 {
            v.report(format!("trigger mask {val:#x} beyond the 34 channels"));
        }
    }

    fn write_reg(&mut self, addr: u32, data: &[u8], v: &mut Violations) {
        let mut b = [0u8; 8];
        b[..data.len().min(8)].copy_from_slice(&data[..data.len().min(8)]);
        let val = u64::from_le_bytes(b);
        let len = data.len();
        match (addr, len) {
            (REG_STATUS, 1) => {
                // "Other values are documented as locking the device" [c, rs].
                if val != 0 {
                    v.report(format!("{val:#04x} written to 0x100000, which only ever takes 0"));
                }
                self.released = true;
            }
            (REG_ARM, 1) => {
                if self.regs.run {
                    self.arm(v);
                }
            }
            (REG_RUN, 1) => {
                self.regs.run = val != 0;
                if val == 0 {
                    self.capture = None;
                }
            }
            (REG_PRE, 2) => self.regs.pre = val as u16,
            (REG_POST, 2) => self.regs.post = val as u16,
            (REG_THRESHOLD, 2) => {
                if val > 1023 {
                    v.report(format!("threshold DAC {val}: 10 bits"));
                }
                self.regs.dac = val as u16;
                self.regs.dac_set = true;
            }
            (REG_PREFILL, 1) => {
                self.run_capture();
                if let Some(c) = self.capture.as_mut() {
                    c.prefilled = true;
                }
            }
            (REG_POSTFILL, 1) => self.end_postfill(),
            (REG_COMBINE, 1) => {
                if val > 5 {
                    v.report(format!("trigger combine {val}: 0 to 5"));
                }
                self.regs.combine = val as u8;
            }
            (REG_CLOCK, 1) => self.regs.clock = val as u8,
            (REG_CHANNEL_CFG, 1) => {
                if val != 0 {
                    v.report(format!("channel config {val:#04x}: no driver writes other than 0"));
                }
            }
            (REG_COMPRESSION, 1) => self.regs.compression = val != 0,
            (REG_COMPRESSION_MASK, 5) => self.regs.mask = val,
            (REG_INVERT, 5) => self.regs.invert = val,
            (REG_PRESCALER, 1) => self.regs.prescaler = val as u8,
            (REG_STEP, 1) => {
                self.regs.step = val as u8;
                if !RATES.iter().any(|r| r.0 == self.regs.prescaler && r.1 == self.regs.step) {
                    v.report(format!(
                        "timebase {:#04x}/{val:#04x}: not in the rate table",
                        self.regs.prescaler
                    ));
                }
            }
            (REG_FREQ, 1) => {
                if val > 0x21 {
                    v.report(format!("frequency counter input {val:#x}"));
                }
                self.freq_prev = self.freq_input;
                self.freq_input = Some(val as u8);
                self.freq_poll = 0;
                self.freq_reads = 0;
            }
            (a, _) if (REG_LEVEL_A..REG_LEVEL_A + 0x40).contains(&a) => {
                self.write_level(0, a - REG_LEVEL_A, val, len, v);
            }
            (a, _) if (REG_LEVEL_B..REG_LEVEL_B + 0x40).contains(&a) => {
                self.write_level(1, a - REG_LEVEL_B, val, len, v);
            }
            (a, n) => v.report(format!("write of {n} bytes at {a:#010x}, which no driver writes")),
        }
    }

    fn answer(&mut self, cmd: u8, data: &[u8]) {
        self.answered += 1;
        if self.stale {
            self.stale = false;
            // Eleven bytes ahead of the first answer after a reconfiguration [rec].
            self.out.extend([0xff; 8]);
            self.out.extend([0xee, 0x00, 0x00]);
        }
        let mut seq = self.seq;
        if self.bad_seq == Some(self.answered) {
            seq ^= 0x5a5a;
        }
        self.out.push_back(cmd);
        self.out.extend(seq.to_be_bytes());
        self.out.extend(data);
        self.seq = self.seq.wrapping_add(1);
    }

    fn silent_now(&self, addr: Option<u32>) -> bool {
        self.ignore > 0
            || self.silent_after.is_some_and(|n| self.answered >= n)
            || (addr.is_some() && addr == self.silent_reg)
    }

    fn command(&mut self, cmd: u8, a0: u16, a1: u16, data: &[u8], v: &mut Violations) {
        let addr = u32::from(self.segment) << 16 | u32::from(a0);
        let len = usize::from(a1) + 1;
        match cmd {
            0xc3 => {
                if a1 != 0 {
                    v.report(format!("segment command with count {a1}"));
                }
                if !self.silent_now(None) {
                    self.segment = a0;
                    self.answer(cmd, &[]);
                }
            }
            0xc1 => {
                if self.silent_now(Some(addr)) {
                    return;
                }
                self.write_reg(addr, data, v);
                self.answer(cmd, &[]);
            }
            _ => {
                if self.silent_now(Some(addr)) {
                    return;
                }
                let d = self.read_reg(addr, len, v);
                if (MEM_A..MEM_FLAGS + DEPTH as u32).contains(&addr) && len > 8 {
                    self.lane_reads += 1;
                    if self.short == Some(self.lane_reads) {
                        self.answer(cmd, &d[..len / 2]);
                        // The rest never comes.
                        self.silent_after = Some(0);
                        return;
                    }
                }
                self.answer(cmd, &d);
            }
        }
    }

    /// The FIFO's bytes to the command parser. Zeros between commands are
    /// nothing, and 0x01 resets the parser and the counter [c: the flood of
    /// zeros "drains any partially received command of maximum length, the
    /// final 0x01 resets the parser and the counter"].
    fn fifo_bytes(&mut self, bytes: &[u8], v: &mut Violations) {
        for &b in bytes {
            if let Some((cmd, a0, want, mut buf)) = self.data.take() {
                buf.push(b);
                if buf.len() == want {
                    self.command(cmd, a0, (want - 1) as u16, &buf, v);
                } else {
                    self.data = Some((cmd, a0, want, buf));
                }
                continue;
            }
            if !self.hdr.is_empty() {
                self.hdr.push(b);
                if self.hdr.len() == 5 {
                    let cmd = self.hdr[0];
                    let a0 = u16::from_le_bytes([self.hdr[1], self.hdr[2]]);
                    let a1 = u16::from_le_bytes([self.hdr[3], self.hdr[4]]);
                    self.hdr.clear();
                    if cmd == 0xc1 {
                        self.data = Some((cmd, a0, usize::from(a1) + 1, Vec::new()));
                    } else {
                        self.command(cmd, a0, a1, &[], v);
                    }
                }
                continue;
            }
            match b {
                0 => {}
                1 => {
                    self.seq = 0;
                    self.ignore = self.ignore.saturating_sub(1);
                }
                0xc1..=0xc3 => self.hdr.push(b),
                other => v.report(format!("FIFO byte {other:#04x} outside a command")),
            }
        }
    }

    /// One rising edge of DCLK during an upload.
    fn upload_bit(&mut self, bit: u8) {
        let Some(u) = self.upload.as_mut() else {
            if self.pending_image.is_some() {
                self.extra += 1;
            }
            return;
        };
        u.acc |= bit << u.bit;
        u.bit += 1;
        if u.bit < 8 {
            return;
        }
        let byte = u.acc;
        u.acc = 0;
        u.bit = 0;
        let at = u.bytes;
        u.bytes += 1;
        if u.bad {
            return;
        }
        if at < RBF_HEAD {
            let want = if at < 16 { 0xff } else { 0x6a };
            u.bad = byte != want;
        } else {
            u.window.push(byte);
            if u.window.len() == 32 {
                let block = u.block;
                u.block += 1;
                let bytes = std::mem::take(&mut u.window);
                if block == 0 {
                    u.standin =
                        (0..IMAGES.len()).find(|&i| standin_block(i, 0).as_slice() == bytes);
                    let d: [u8; 32] = Sha256::digest(&bytes).into();
                    let head = u64::from_be_bytes([d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]]);
                    u.known = match (u.standin, REAL_HEADS.iter().position(|&h| h == head)) {
                        (Some(i), _) => Some((i, standin_size(i))),
                        (None, Some(i)) => Some((i, REAL_SIZES[i])),
                        (None, None) => self.any_upload,
                    };
                    u.bad = u.known.is_none();
                } else if let Some(i) = u.standin {
                    // The stand-in's last byte is set by the generator.
                    let mut want = standin_block(i, block);
                    let end = RBF_HEAD + 32 * (block + 1);
                    if end == standin_size(i) {
                        want[31] = 0xff;
                    }
                    if end <= standin_size(i) && want.as_slice() != bytes {
                        u.bad = true;
                    }
                }
            }
        }
        let done = u.known.and_then(|(i, n)| (u.bytes == n).then_some(i));
        if done.is_some() && u.standin.is_some() {
            u.bad |= byte != 0xff;
        }
        if u.bad {
            // A CRC error [guess on how it shows].
            self.nstatus = false;
            return;
        }
        if let Some(image) = done {
            if self.conf_done_fails > 0 {
                self.conf_done_fails -= 1;
                self.upload = None;
            } else {
                self.conf_done = true;
                self.pending_image =
                    Some(self.wrong_image.unwrap_or(u8::try_from(image).unwrap_or(0)));
                self.extra = 0;
                self.upload = None;
            }
        }
    }

    /// The host's byte on the pins in bit-bang mode.
    fn pin_write(&mut self, b: u8, v: &mut Violations) {
        if b & !0x07 != 0 {
            v.report(format!("bit-bang byte {b:#04x}: only DATA0, DCLK and nCONFIG are outputs"));
        }
        let old = self.latch;
        self.latch = b & 0x07;
        if old & 4 != 0 && b & 4 == 0 {
            // nCONFIG low: only a released configuration goes [c].
            if self.image.is_none() || self.released {
                self.image = None;
                self.released = false;
                self.conf_done = false;
                self.nstatus = false;
                self.upload = None;
                self.pending_image = None;
            }
        }
        if old & 4 == 0 && b & 4 != 0 && self.image.is_none() {
            self.nstatus = !self.nstatus_stuck;
            self.upload = Some(Upload::default());
        }
        if old & 2 == 0 && b & 2 != 0 && b & 4 != 0 && self.image.is_none() {
            self.upload_bit(b & 1);
        }
    }

    /// Leaving bit-bang: the FPGA starts if it has its image and its extra
    /// clocks [c: "Cyclone devices need extra DCLK edges"].
    fn leave_bitbang(&mut self, v: &mut Violations) {
        if let Some(image) = self.pending_image {
            if self.extra < EXTRA_CLOCKS {
                v.report(format!(
                    "{} clocks after CONF_DONE; the drivers send {EXTRA_CLOCKS}",
                    self.extra
                ));
            } else {
                self.image = Some(image);
                self.pending_image = None;
                self.stale = self.stale_on;
                self.ignore = self.poweron_silent;
                self.init_read = 0;
                self.seq = 0;
                self.regs = Regs::default();
                self.capture = None;
            }
        }
    }

    fn pins_now(&self) -> u8 {
        let driven = if self.in_bitbang { self.latch } else { 0x07 };
        0xe0 | u8::from(self.conf_done) << 4 | u8::from(self.nstatus) << 3 | driven
    }
}

impl FtdiPorts for Fpga {
    fn changed(&mut self, _: usize, event: Event, state: &PortState, v: &mut Violations) {
        match event {
            Event::Bitmode => {
                let bb = state.bitmode == bitmode::BITBANG;
                if self.in_bitbang && !bb {
                    self.leave_bitbang(v);
                }
                self.in_bitbang = bb;
            }
            Event::FlushToHost | Event::Reset => self.out.clear(),
            _ => {}
        }
    }

    fn write(&mut self, _: usize, data: &[u8], state: &PortState, v: &mut Violations) -> Reply {
        if self.in_bitbang {
            if state.mask != 0x07 {
                v.report(format!(
                    "bit-bang mask {:#04x}: DATA0, DCLK, nCONFIG are the outputs",
                    state.mask
                ));
            }
            for &b in data {
                self.pin_write(b, v);
            }
            return Reply::ack();
        }
        if self.image.is_none() || self.released {
            // An unconfigured FPGA doesn't drain the FIFO [c].
            return Reply::Nak;
        }
        self.fifo_bytes(data, v);
        Reply::ack()
    }

    fn read(&mut self, _: usize, max: usize, _: &PortState, _: &mut Violations) -> Fill {
        if self.out.is_empty() {
            return Fill::Idle;
        }
        let n = max.min(self.out.len());
        Fill::Data(self.out.drain(..n).collect())
    }

    fn pins(&mut self, _: usize, _: &PortState, _: &mut Violations) -> u8 {
        self.pins_now()
    }
}

/// The emulated `LogicPort`.
pub struct LogicPort {
    chip: Ftdi,
    faults: Faults,
    eeprom: Vec<u16>,
    fpga: Fpga,
    fresh: Fpga,
}

/// The EEPROM's 64 words, as read from a real unit [rec:
/// `tests/fixtures/ftdi/logicport-probe.txt`].
const EEPROM: [u16; 64] = [
    0x0000, 0x0403, 0xdc48, 0x0400, 0x6480, 0x001c, 0x0110, 0x1294, 0x32a6, 0x12d8, 0x0312, 0x0049,
    0x006e, 0x0074, 0x0072, 0x006f, 0x006e, 0x0069, 0x0078, 0x0332, 0x004c, 0x006f, 0x0067, 0x0069,
    0x0063, 0x0050, 0x006f, 0x0072, 0x0074, 0x0020, 0x004c, 0x006f, 0x0067, 0x0069, 0x0063, 0x0020,
    0x0041, 0x006e, 0x0061, 0x006c, 0x0079, 0x007a, 0x0065, 0x0072, 0x0312, 0x0037, 0x0031, 0x0035,
    0x0034, 0x0033, 0x0035, 0x0035, 0x0031, 0x0302, 0x0001, 0x302d, 0x3430, 0x3030, 0x3030, 0x3032,
    0x3830, 0x3330, 0x3332, 0x944c,
];

impl LogicPort {
    fn new(config: FtdiConfig, fpga: Fpga, eeprom: Vec<u16>, faults: Faults) -> LogicPort {
        let mut chip = Ftdi::new(config);
        chip.set_eeprom(eeprom.clone());
        chip.port_mut(0).modem_status = 0x31;
        chip.set_faults(faults.clone());
        LogicPort { chip, faults, eeprom, fresh: fpga.clone(), fpga }
    }
}

/// Long data as its digest: a bitstream is the vendor's in a run against the
/// real file, and a readout is long.
pub fn log_rules(_: &[(String, String)]) -> crate::txlog::Rules {
    crate::txlog::Rules::digest_above(crate::txlog::HASH_ABOVE)
}

/// The `seq=MASK:NS,...` list, in half nanoseconds.
fn parse_seq(k: &str, v: &str) -> Result<Vec<(u64, u64)>> {
    let bad = || Error::Invalid(format!("{k}={v}: MASK:NS,..."));
    v.split(',')
        .map(|item| {
            let (m, ns) = item.split_once(':').ok_or_else(bad)?;
            let m = match m.strip_prefix("0x") {
                Some(h) => u64::from_str_radix(h, 16),
                None => m.parse(),
            };
            Ok((m.map_err(|_| bad())?, ns.parse::<u64>().map_err(|_| bad())? * 2))
        })
        .collect()
}

/// A `LogicPort` from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a bad value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut f = Fpga::new();
    let mut faults = Faults::default();
    let mut eeprom: Vec<u16> = EEPROM.to_vec();
    let number = |k: &str, v: &str| -> Result<u64> {
        match v.strip_prefix("0x") {
            Some(h) => u64::from_str_radix(h, 16),
            None => v.parse(),
        }
        .map_err(|_| Error::Invalid(format!("{k}={v}: not a number")))
    };
    for (k, v) in options {
        let n32 = || -> Result<u32> {
            u32::try_from(number(k, v)?).map_err(|_| Error::Invalid(format!("{k}={v}: 32 bits")))
        };
        match k.as_str() {
            "image" => {
                if v == "none" {
                    f.set_power_image(None);
                } else {
                    let i = number(k, v)?;
                    if i > 7 {
                        return Err(Error::Invalid(format!("image={v}: 0 to 7 or none")));
                    }
                    f.set_power_image(Some(i as u8));
                }
            }
            "id" => f.id_override = Some(n32()? as u8),
            "stale" => f.stale_on = n32()? != 0,
            "poweron-silent" => f.poweron_silent = n32()?,
            "init-steps" => f.init_steps = n32()?,
            "init-stuck" => f.init_stuck = n32()? != 0,
            "nstatus-stuck" => f.nstatus_stuck = n32()? != 0,
            "conf-done-stuck" => f.conf_done_fails = n32()?,
            "wrong-image" => f.wrong_image = Some(n32()? as u8),
            "silent-after" => f.silent_after = Some(n32()?),
            "silent-reg" => f.silent_reg = Some(n32()?),
            "bad-seq" => f.bad_seq = Some(n32()?),
            "short" => f.short = Some(n32()?),
            "fill" => f.fill = n32()?,
            "ring" => f.ring = n32()? as usize % DEPTH,
            "any-upload" => {
                let bad = || Error::Invalid(format!("{k}={v}: IMAGE:BYTES"));
                let (i, n) = v.split_once(':').ok_or_else(bad)?;
                f.any_upload = Some((i.parse().map_err(|_| bad())?, n.parse().map_err(|_| bad())?));
            }
            "rle-idle" => f.rle_idle = number(k, v)?,
            "d" => f.inputs.d = number(k, v)?,
            "clk" => f.inputs.clk = number(k, v)?,
            "clk2" => f.inputs.clk2 = number(k, v)?,
            "seq" => f.inputs.seq = parse_seq(k, v)?,
            "seq-repeat" => f.inputs.repeat = n32()?.max(1),
            "lead" => f.inputs.lead = Some(number(k, v)? * 2),
            "volts" => {
                f.inputs.volts =
                    v.parse().map_err(|_| Error::Invalid(format!("volts={v}: not a number")))?;
            }
            "cal" => {
                // The string the driver parses, at the words holding it
                // [rec: byte 0x70 on, after the PnP block at 0x6c].
                let bytes = if v == "none" { b"xxxx".as_slice() } else { v.as_bytes() };
                if bytes.len() != 4 {
                    return Err(Error::Invalid(format!("cal={v}: +NNN, -NNN or none")));
                }
                eeprom[0x37] = u16::from_le_bytes([bytes[0], bytes[1]]);
                eeprom[0x38] = u16::from_le_bytes([bytes[2], bytes[3]]);
                if let Ok(c) = v.parse::<i32>() {
                    f.cal = c;
                } else {
                    f.cal = 0;
                }
                // The word checksum the chip's image ends with stays valid
                // only in the plain image; neither driver looks at it.
            }
            other => {
                if !faults.option(other, v)? {
                    return Err(Error::Invalid(format!("{NAME} has no option {other}")));
                }
            }
        }
    }
    let mut config = FtdiConfig::new(Chip::Bm);
    config.product = 0xdc48;
    config.manufacturer = Some("Intronix".to_owned());
    config.description = Some("LogicPort Logic Analyzer".to_owned());
    config.serial = Some("71543551".to_owned());
    // Word 4 of the EEPROM: bus powered, 200 mA [rec].
    config.attributes = 0x80;
    config.max_power = 200;
    Ok(Box::new(LogicPort::new(config, f, eeprom, faults)))
}

impl DeviceModel for LogicPort {
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
        self.chip.set_eeprom(self.eeprom.clone());
        self.chip.set_faults(self.faults.clone());
        self.fpga = self.fresh.clone();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fmt::Write as _;

    const RLE: &str = include_str!("../../../tests/fixtures/logicport/capture-1m-rle.txt");
    const TEN: &str = include_str!("../../../tests/fixtures/logicport/capture-10m-clk1.txt");
    const PROBE: &str = include_str!("../../../tests/fixtures/ftdi/logicport-probe.txt");

    fn o(k: &str, v: &str) -> (String, String) {
        (k.to_owned(), v.to_owned())
    }

    /// The fixtures' hex, with `XX{N}` for N of a byte.
    #[allow(clippy::many_single_char_names)]
    fn unhex(s: &str) -> Vec<u8> {
        let b = s.as_bytes();
        let mut out = Vec::new();
        let mut i = 0;
        while i < b.len() {
            if b[i] == b'{' {
                let j = i + s[i..].find('}').unwrap();
                let n: usize = s[i + 1..j].parse().unwrap();
                let last = *out.last().unwrap();
                out.extend(std::iter::repeat_n(last, n - 1));
                i = j + 1;
            } else {
                out.push(u8::from_str_radix(&s[i..i + 2], 16).unwrap());
                i += 2;
            }
        }
        out
    }

    /// A control transfer line of a fixture, its answer compared.
    fn ctl(dev: &mut Box<dyn DeviceModel>, line: &str, v: &mut Violations) {
        let f: Vec<&str> = line.split_whitespace().collect();
        let word = |s: &str| u16::from_str_radix(s, 16).unwrap();
        let (req, value, index) = (word(f[2]) as u8, word(f[3]), word(f[4]));
        if f[0] == "ctl_out" {
            let s = Setup { request_type: 0x40, request: req, value, index, length: 0 };
            assert_eq!(dev.control(s, &[], v), Reply::ack(), "{line}");
        } else {
            let length: u16 = f[5].parse().unwrap();
            let s = Setup { request_type: 0xc0, request: req, value, index, length };
            assert_eq!(dev.control(s, &[], v), Reply::Data(unhex(f[7])), "{line}");
        }
    }

    /// Plays a fixture's host side against the model: every control answer
    /// and non-empty bulk IN must be the recording's byte for byte. A
    /// status-only read in the recording is the real chip's latency, and is
    /// skipped; a read with data must get it at once. `upload` stands in for
    /// the digests of the vendor's bitstream, by length.
    fn play(dev: &mut Box<dyn DeviceModel>, text: &str, upload: &mut [u8]) -> usize {
        play_until(dev, text, upload, &mut None)
    }

    /// With `wp` set, stops at the first write pointer read and gives what
    /// the model answers, for the ring position to use.
    fn play_until(
        dev: &mut Box<dyn DeviceModel>,
        text: &str,
        upload: &mut [u8],
        wp: &mut Option<(u16, u16)>,
    ) -> usize {
        let finding = wp.is_some();
        let mut at_wp = false;
        let mut v = Violations::default();
        let mut compared = 0;
        let mut pos = 0;
        for line in text.lines() {
            if line.starts_with("ctl_") {
                ctl(dev, line, &mut v);
                compared += 1;
            } else if let Some(rest) = line.strip_prefix("bulk_out 02 ") {
                let payload = rest.split(' ').next().unwrap();
                if let Some(d) = payload.strip_prefix('@') {
                    let (len, _) = d.split_once(':').unwrap();
                    let len: usize = len.parse().unwrap();
                    // The zeros of a resync have one digest; a bitstream
                    // chunk's is the vendor's.
                    let zeros =
                        d.contains("ad7facb2") || d.contains("2cae6841") || d.contains("226d79c3");
                    let bytes = if zeros {
                        vec![0; len]
                    } else {
                        let end = (pos + len).min(upload.len());
                        let b = upload[pos..end].to_vec();
                        pos = end;
                        b
                    };
                    assert_eq!(dev.out(2, &bytes, &mut v), Reply::ack(), "{line}");
                } else {
                    let bytes = unhex(payload);
                    at_wp = bytes == [0xc2, 0, 0, 1, 0];
                    assert_eq!(dev.out(2, &bytes, &mut v), Reply::ack(), "{line}");
                }
            } else if let Some(rest) = line.strip_prefix("bulk_in 81 ") {
                let want = unhex(rest.split(" -> ").nth(1).unwrap());
                if want.len() == 2 {
                    assert_eq!(want, [0x31, 0x60], "{line}");
                    continue;
                }
                let Reply::Data(got) = dev.in_transfer(1, 4096, &mut v) else { panic!("{line}") };
                if finding && at_wp {
                    let word = |d: &[u8]| u16::from_le_bytes([d[5], d[6]]);
                    *wp = Some((word(&want), word(&got)));
                    return compared;
                }
                assert!(got == want, "{}\n got {}", &line[..line.len().min(80)], hexs(&got));
                compared += 1;
            }
        }
        assert!(v.is_empty(), "{v:?}");
        compared
    }

    fn hexs(b: &[u8]) -> String {
        b.iter().take(48).fold(String::new(), |s, x| s + &format!("{x:02x}"))
    }

    /// The whole 1 MHz compressed session of the real unit: its register
    /// stream, its 0x73, the write pointer, the rows, the trigger position,
    /// from a simulation of a 100 kHz square wave on CLK1 at the unit's
    /// other inputs. `ring` is where the real unit's ring happened to be.
    #[test]
    fn the_compressed_recording_byte_for_byte() {
        let opts = |r: usize| vec![o("clk", "100000"), o("ring", &r.to_string())];
        let mut wp = Some((0, 0));
        play_until(&mut build(&opts(0)).unwrap(), RLE, &mut [], &mut wp);
        let (want, got) = wp.unwrap();
        let ring = (usize::from(want) + DEPTH - usize::from(got)) % DEPTH;
        assert!(play(&mut build(&opts(ring)).unwrap(), RLE, &mut []) > 100);
    }

    /// The 10 MHz session: the reconfiguration with its pins and stale
    /// bytes, the init counter, then the capture. The recording's bitstream
    /// is the vendor's, so an image of the real length with a stand-in's
    /// header goes in its place, chunk by chunk as recorded.
    #[test]
    fn the_reconfiguring_recording_byte_for_byte() {
        let mut rbf = vec![0xff; 16];
        rbf.push(0x6a);
        rbf.resize(REAL_SIZES[7], 0xff);
        let mut upload = Vec::new();
        for &b in &rbf {
            for k in 0..8 {
                let bit = b >> k & 1;
                upload.push(0x04 | bit);
                upload.push(0x06 | bit);
            }
        }
        let opts =
            |r: usize| vec![o("image", "6"), o("ring", &r.to_string()), o("any-upload", "7:66149")];
        let mut wp = Some((0, 0));
        play_until(&mut build(&opts(0)).unwrap(), TEN, &mut upload.clone(), &mut wp);
        let (want, got) = wp.unwrap();
        let ring = (usize::from(want) + DEPTH - usize::from(got)) % DEPTH;
        assert!(play(&mut build(&opts(ring)).unwrap(), TEN, &mut upload) > 100);
    }

    #[test]
    fn the_probe_recording() {
        let mut dev = build(&[o("image", "none")]).unwrap();
        // Its one write, the bit mode, and the read that follow; its pins
        // show a cold FPGA.
        assert!(play(&mut dev, PROBE, &mut []) > 60);
        assert_eq!(dev.descriptors().strings, ["Intronix", "LogicPort Logic Analyzer", "71543551"]);
        assert_eq!(dev.descriptors().product_id(), 0xdc48);
        assert_eq!(dev.descriptors().bcd_device(), 0x0400);
    }

    /// A capture's rows through the C driver's readout (`read_sdr`), for
    /// the bench's criteria: (samples, the match).
    #[allow(clippy::many_single_char_names)]
    fn samples(f: &mut Fpga) -> (Vec<u64>, usize) {
        let status = f.status();
        assert!(status & 1 != 0, "status {status:#x}");
        let mut v = Violations::default();
        let wp = f.write_pointer(false) as usize;
        let x = if status & 2 == 0 { f.unfilled() as usize + 1 } else { 0 };
        let mut n = DEPTH - x;
        let mut start = (wp + 1 + x) % DEPTH;
        let row = |f: &mut Fpga, r: usize, v: &mut Violations| -> [u8; 5] {
            let mut out = [0; 5];
            for (k, lane) in out.iter_mut().take(4).enumerate() {
                *lane = f.read_reg(MEM_A + k as u32 * 0x1000 + r as u32, 1, v)[0];
            }
            out[4] = f.read_reg(MEM_FLAGS + r as u32, 1, v)[0];
            out
        };
        if row(f, start, &mut v)[4] & 8 != 0 && n > 1 {
            start = (start + 1) % DEPTH;
            n -= 1;
        }
        let pos = f.capture.as_ref().unwrap().trigger_pos as usize;
        let idx = (n + 1 - pos.min(n + 1)) % (n + 1);
        let idx = if idx > 0 { idx.min(n) - 1 } else { 0 };
        let mut out = Vec::new();
        let mut last = 0;
        let mut at = 0;
        for i in 0..n {
            let r = row(f, (start + i) % DEPTH, &mut v);
            let low = u64::from(u32::from_le_bytes([r[0], r[1], r[2], r[3]]));
            if i == idx {
                at = out.len();
            }
            if r[4] & 8 != 0 {
                let extra = low | u64::from(r[4] & 7) << 32;
                out.extend(std::iter::repeat_n(last, extra as usize + 1));
            } else {
                last = low | u64::from(r[4] >> 1 & 3) << 32;
                out.push(last);
            }
        }
        assert!(v.is_empty(), "{v:?}");
        (out, at.saturating_sub(7))
    }

    /// Programmed as the drivers do, for 1 MHz at the given inputs.
    fn fpga(options: &[(String, String)], set: impl Fn(&mut Regs)) -> Fpga {
        let mut f = Fpga::new();
        for (k, v) in options {
            match k.as_str() {
                "seq" => f.inputs.seq = parse_seq(k, v).unwrap(),
                "clk" => f.inputs.clk = v.parse().unwrap(),
                _ => panic!(),
            }
        }
        f.inputs.d = 0;
        f.regs.prescaler = 0x31;
        f.regs.pre = 1032;
        f.regs.post = 1016;
        f.regs.dac = 574;
        f.regs.dac_set = true;
        f.regs.mask = (1 << 34) - 1;
        f.regs.run = true;
        set(&mut f.regs);
        let mut v = Violations::default();
        f.arm(&mut v);
        assert!(v.is_empty());
        f
    }

    fn rising(bit: u64) -> Level {
        Level { rising: 1 << bit, source: 3, mode: 1, ..Level::default() }
    }

    /// Bench: "occurrence count D0=r*N: marker on rise N".
    #[test]
    fn counts() {
        for n in [1u32, 5, 17] {
            let mut f = fpga(
                &[o("seq", "1:5000,0:5000,".repeat(30).trim_end_matches(',')), o("clk", "0")],
                |r| {
                    r.combine = 1;
                    r.levels[0] = Level { count_high: n - 1, ..rising(0) };
                },
            );
            let (s, at) = samples(&mut f);
            let rises: Vec<usize> =
                (1..s.len()).filter(|&i| s[i - 1] & 1 == 0 && s[i] & 1 == 1).collect();
            assert_eq!(rises[n as usize - 1], at, "rise {n}");
        }
    }

    /// Bench: six D0 rises with D1 high except on the third: a cumulative
    /// count of 3 fires on rise 4, a consecutive one on rise 6.
    #[test]
    fn consecutive() {
        let mut seq = String::new();
        for k in 1..=6 {
            let d1 = if k == 3 { 0 } else { 2 };
            let _ = write!(seq, "{d1}:5000,{}:5000,", d1 | 1);
        }
        let seq = seq.trim_end_matches(',').to_owned();
        for (consecutive, rise) in [(false, 4), (true, 6)] {
            let mut f = fpga(&[o("seq", &seq), o("clk", "0")], |r| {
                r.combine = 1;
                r.levels[0] = Level {
                    count_high: 2,
                    consecutive,
                    pattern_mask: 2,
                    pattern_value: 2,
                    condition: 1,
                    ..rising(0)
                };
            });
            let (s, at) = samples(&mut f);
            let rises: Vec<usize> =
                (1..s.len()).filter(|&i| s[i - 1] & 1 == 0 && s[i] & 1 == 1).collect();
            assert_eq!(rises[rise - 1], at, "consecutive {consecutive}");
        }
    }

    /// Bench: duration: the marker 100 samples into the long pulse, after
    /// the extra sample the driver takes off.
    #[test]
    fn durations() {
        let seq = "1:50000,0:50000,1:50000,0:50000,1:200000,0:50000".to_owned();
        let mut f = fpga(&[o("seq", &seq), o("clk", "0")], |r| {
            r.combine = 1;
            r.levels[0] = Level {
                pattern_mask: 1,
                pattern_value: 1,
                condition: 1,
                mode: 2,
                count_low: 100,
                consecutive: true,
                ..Level::default()
            };
        });
        let (s, at) = samples(&mut f);
        // `samples` takes off 7, the driver one more for a duration.
        let at = at - 1;
        let rise = (1..=at).rev().find(|&i| s[i - 1] & 1 == 0 && s[i] & 1 == 1).unwrap();
        assert_eq!(at - rise, 100);
    }

    /// Bench: value ranges fire where the value enters, inside or outside.
    #[test]
    fn value_ranges() {
        let count: Vec<String> = (0..8).map(|m| format!("{m}:10000")).collect();
        let seq = [count.join(","), count.join(",")].join(",");
        for (lo, hi, cond, first, before) in [(3, 5, 0x0c, 3, 2), (0, 6, 0x10, 7, 6)] {
            let mut f = fpga(&[o("seq", &seq), o("clk", "0")], |r| {
                r.combine = 1;
                r.levels[0] = Level {
                    value_mask: 7,
                    value_low: lo,
                    value_high: hi,
                    condition: cond,
                    mode: 1,
                    ..Level::default()
                };
            });
            let (s, at) = samples(&mut f);
            assert_eq!((s[at] & 7, s[at - 1] & 7), (first, before));
        }
    }

    /// Bench: or fires on the second level alone, and fires on the first
    /// rise of whichever comes second under and.
    #[test]
    fn combines() {
        let d1 = "2:5000,0:5000,2:5000,0:5000".to_owned();
        let mut f = fpga(&[o("seq", &d1), o("clk", "0")], |r| {
            r.combine = 3;
            r.levels = [rising(0), rising(1)];
        });
        let (s, at) = samples(&mut f);
        assert!(s[at] & 2 != 0 && s[at - 1] & 2 == 0);
        for seq in ["2:5000,0:5000,1:5000,0:5000,", "1:5000,0:5000,2:5000,0:5000,"] {
            let second = if seq.starts_with('2') { 1 } else { 2 };
            let seq = seq.repeat(2).trim_end_matches(',').to_owned();
            let mut f = fpga(&[o("seq", &seq), o("clk", "0")], |r| {
                r.combine = 4;
                r.levels = [rising(0), rising(1)];
            });
            let (s, at) = samples(&mut f);
            assert!(s[at] & second != 0 && s[at - 1] & second == 0, "{seq}");
        }
        // A then B: B's rises before A's don't count.
        let seq = "2:5000,0:5000,1:5000,0:5000,2:5000,0:5000".to_owned();
        let mut f = fpga(&[o("seq", &seq), o("clk", "0")], |r| {
            r.combine = 5;
            r.levels = [rising(0), rising(1)];
        });
        let (s, at) = samples(&mut f);
        let d0_before = (1..at).filter(|&i| s[i] & 1 == 1 && s[i - 1] & 1 == 0).count();
        assert_eq!(d0_before, 1);
        assert!(s[at] & 2 != 0);
    }

    /// Bench: the timebase against a 1 MHz clock.
    #[test]
    fn timebase() {
        for (p, st, per) in
            [(0x11u8, 0x00u8, 100usize), (0x21, 0x00, 10), (0x01, 0x00, 200), (0x00, 0x00, 250)]
        {
            let mut f = fpga(&[o("clk", "1000000")], |r| {
                r.prescaler = p;
                r.step = st;
            });
            let (s, _) = samples(&mut f);
            let clk = 1u64 << 32;
            let rises: Vec<usize> =
                (1..s.len()).filter(|&i| s[i - 1] & clk == 0 && s[i] & clk != 0).collect();
            for w in rises.windows(2) {
                assert!(w[1] - w[0] == per || w[1] - w[0] + 1 == per || w[1] - w[0] == per + 1);
            }
            assert_eq!(s.len(), 2048);
        }
    }

    /// Bench: the threshold against the 3.3 V inputs.
    #[test]
    fn threshold() {
        for (dac, want) in [(578u16, 0x2228_0000u64), (893, 0)] {
            let mut f = fpga(&[o("clk", "0")], |r| r.dac = dac);
            f.inputs.d = 0x2228_0000;
            let (s, _) = samples(&mut f);
            // 893 is 6 V, above 3.3 V.
            assert_eq!(s[0], want, "{dac}");
        }
    }

    #[test]
    fn an_upload_with_a_block_out_of_place_fails() {
        let mut f = Fpga::new();
        let mut v = Violations::default();
        f.set_power_image(None);
        f.in_bitbang = true;
        f.pin_write(0, &mut v);
        f.pin_write(4, &mut v);
        let mut rbf = standin_rbf(7);
        rbf.swap(100, 200);
        for &b in &rbf {
            for k in 0..8 {
                let bit = b >> k & 1;
                f.pin_write(0x04 | bit, &mut v);
                f.pin_write(0x06 | bit, &mut v);
            }
        }
        assert!(!f.conf_done);
        assert!(!f.nstatus);
        // An unknown stream: CONF_DONE stays low.
        let mut f = Fpga::new();
        f.set_power_image(None);
        f.in_bitbang = true;
        f.pin_write(0, &mut v);
        f.pin_write(4, &mut v);
        let mut rbf = vec![0xff; 16];
        rbf.push(0x6a);
        rbf.resize(4000, 0x55);
        for &b in &rbf {
            for k in 0..8 {
                let bit = b >> k & 1;
                f.pin_write(0x04 | bit, &mut v);
                f.pin_write(0x06 | bit, &mut v);
            }
        }
        assert!(!f.conf_done);
        assert!(v.is_empty());
    }

    #[test]
    fn flags_what_the_drivers_never_send() {
        let mut v = Violations::default();
        let mut f = Fpga::new();
        f.write_reg(REG_STATUS, &[1], &mut v);
        assert_eq!(v.drain().len(), 1, "other than 0 at 0x100000");
        f.write_reg(0x0010_0009, &[1], &mut v);
        assert_eq!(v.drain().len(), 1, "unknown register");
        f.write_reg(REG_COMBINE, &[6], &mut v);
        assert_eq!(v.drain().len(), 1, "combine");
        f.write_reg(REG_LEVEL_A + 0x0a, &[0, 0, 0x10, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "20 bit counter");
        f.fifo_bytes(&[0x42], &mut v);
        assert_eq!(v.drain().len(), 1, "stray byte");
        assert!(build(&[o("speed", "1")]).is_err());
    }
}
