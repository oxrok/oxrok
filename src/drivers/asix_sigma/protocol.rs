// Counterpart in sigrok:
//   libsigrok/src/hardware/asix-sigma/protocol.c,
//   libsigrok/src/hardware/asix-sigma/protocol.h,
//   libsigrok/src/hardware/asix-sigma/api.c (dev_acquisition_start's register values),
//   Copyright (C) 2010-2012 Håvard Espeland <gus@ping.uio.no>,
//   Copyright (C) 2010 Martin Stensgård <mastensg@ping.uio.no>,
//   Copyright (C) 2010 Carl Henrik Lunde <chlunde@ping.uio.no>,
//   Copyright (C) 2020 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Uwe Hermann, Daniel Trnka, Lars-Peter Clausen, jry,
//   Niklas Thorne
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The SIGMA's wire protocol and sample memory, `protocol.c`.
//!
//! The SIGMA and SIGMA2 are an FTDI FT245 style chip in front of a
//! Xilinx FPGA without configuration memory. The driver drives the chip
//! through libftdi ([`crate::ftdi::libftdi`]), in two modes:
//!
//! - Asynchronous bit-bang, to configure the FPGA in slave serial mode
//!   at 750 kBd ([`Link::upload_firmware`]): D0 CCLK and D1 PROG
//!   (inverted by the level shifters), D2 to D4 and D7 the "suicide
//!   sequence" that ends a running netlist, D5 INIT (the only input,
//!   mask 0xdf), D6 DIN. Four suicide bursts, the PROG pulse, a purge,
//!   reads of the pins until INIT is high (up to 10 bursts 10 ms apart,
//!   the whole tried 10 times), then the netlist, unscrambled with the
//!   driver's xor sequence and sent as two pin states per bit (CCLK set
//!   with DIN, then CCLK clear). Bit-bang off, purge, drain, then the
//!   netlist must answer its id (0xa6) and read back 0x55 and 0xaa from
//!   its scratch register. The netlist depends on the rate:
//!   `asix-sigma-50.fw` up to 50 MHz (16 channels, 50 MHz divided by 1
//!   to 256), `-100.fw` (8 channels) and `-200.fw` (4 channels).
//! - The FIFO, through which the FPGA takes one byte commands
//!   (`protocol.h`): `0x0n`/`0x1n` the low and high nibble of a
//!   register address, `0x2n` then `0x3n` a data byte written to it,
//!   `0x40` a read of it (`0x41` reads and moves to the next), `0x50`
//!   waits for a memory row fetch, `0x60`/`0x70` fetch a row into
//!   buffer 0 or 1, `0xa0`/`0xb0` send that buffer (1024 bytes).
//!
//! The sample memory is 32768 rows of 64 clusters, a cluster a 16 bit
//! timestamp and 7 events of 16 bits. Positions count events, the row
//! in bits 9 and up, the event in the row (0 to 447) below. Without a pin change no cluster is stored: a gap in the
//! timestamps repeats the last sample. An event is one sample of 16
//! channels up to 50 MHz, two interleaved samples of 8 at 100 MHz, four
//! of 4 at 200 MHz.
//!
//! A capture is polled every 10 ms: the stop and trigger positions and
//! the mode register. It ends when the device says it is complete (the
//! post trigger share of the memory is written), when the driver's time
//! limit runs out (the sample or time limit turned into milliseconds
//! plus two worst case cluster periods, counted from the trigger when
//! there is one), when the memory wraps without a trigger, or on a
//! stop; the driver then forces a stop if needed and reads the memory,
//! 32 rows per read and 50 reads per poll. The hardware trigger
//! position is late: the samples from 28 events before it are checked
//! against the trigger in software, for 56 events, and the marker goes
//! before the first that matches, or at the hardware position.
//!
//! Where this port differs from `protocol.c` on purpose:
//!
//! - A trigger within the first 28 events of the memory gets its marker:
//!   the C driver arms the software check at the start position, which
//!   it compares only after moving past it (`sigma_location_check`
//!   runs after each event's increment).
//! - All of a wrapped memory's last row is decoded: the C driver ends
//!   the readout at event 0 of that row (`alloc_sample_buffer`), so
//!   `download_capture`'s count (`stop.raw & ROW_MASK`) is 0. The row is
//!   whole, from the pass before: no count from the stop position is
//!   needed, and no assumption about the register.
//! - A forced stop that never completes ends after
//!   [`MAX_STOP_POLLS`] reads of the mode register, or
//!   [`MAX_STOP_POLLS_STOPPING`] once a stop was asked for; the C driver
//!   polls for ever.
//!
//! Kept from `protocol.c` on purpose: without a wrap, the last row's
//! count is the low bits of the decremented stop position. If the
//! register points after the last event, as the comment at `protocol.c`
//! 371-377 reads it, that is one event short; if it is late, as
//! `protocol.c` 1433-1435 says of these registers, more would run into
//! a cluster not written, whose timestamp 0 repeats the last sample
//! about 65000 times. The one event is the smaller cost. A hardware
//! recording could settle it: capture a counter with `--time` and check
//! whether the last sample continues the count.

use crate::core::{Error, Packet, PacketSink, Result, Trigger, TriggerMatch};
use crate::ftdi::libftdi::Context;
use std::time::{Duration, Instant};

/// The sample rates the driver suggests (`samplerates[]`); it takes any
/// 50 MHz / N from 200 kHz, and 100 and 200 MHz.
pub const SAMPLERATES: [u64; 10] = [
    200_000,
    500_000,
    1_000_000,
    2_000_000,
    5_000_000,
    10_000_000,
    25_000_000,
    50_000_000,
    100_000_000,
    200_000_000,
];

const MHZ_50: u64 = 50_000_000;
const MHZ_100: u64 = 100_000_000;
const MHZ_200: u64 = 200_000_000;

/// `SIGMA_FIRMWARE_SIZE_LIMIT`.
const FIRMWARE_SIZE_LIMIT: usize = 256 * 1024;

/// The netlists the driver uploads (`firmware_files[]`; the sync and
/// frequency counter netlists are never chosen).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Firmware {
    /// 50 MHz and its divisions, 16 channels.
    Mhz50,
    /// 100 MHz, 8 channels.
    Mhz100,
    /// 200 MHz, 4 channels.
    Mhz200,
}

impl Firmware {
    /// The file name.
    pub fn file(self) -> &'static str {
        match self {
            Firmware::Mhz50 => "asix-sigma-50.fw",
            Firmware::Mhz100 => "asix-sigma-100.fw",
            Firmware::Mhz200 => "asix-sigma-200.fw",
        }
    }

    /// The netlist for a normalised rate, and its channel count.
    pub fn for_rate(rate: u64) -> Option<(Firmware, usize)> {
        match rate {
            r if r <= MHZ_50 => Some((Firmware::Mhz50, 16)),
            MHZ_100 => Some((Firmware::Mhz100, 8)),
            MHZ_200 => Some((Firmware::Mhz200, 4)),
            _ => None,
        }
    }
}

/// `sigma_normalize_samplerate`: 100 and 200 MHz as they are, 200 kHz
/// to 50 MHz rounded up to 50 MHz / N, anything else refused.
pub fn normalize_samplerate(want: u64) -> Option<u64> {
    if want == MHZ_200 || want == MHZ_100 {
        return Some(want);
    }
    if (200_000..=MHZ_50).contains(&want) {
        return Some(MHZ_50 / (MHZ_50 / want));
    }
    None
}

// FPGA commands (`protocol.h`).
const REG_ADDR_LOW: u8 = 0x00;
const REG_ADDR_HIGH: u8 = 0x10;
const REG_DATA_LOW: u8 = 0x20;
const REG_DATA_HIGH_WRITE: u8 = 0x30;
const REG_READ_ADDR: u8 = 0x40;
const REG_ADDR_INC: u8 = 0x01;
const REG_DRAM_WAIT_ACK: u8 = 0x50;
const REG_DRAM_BLOCK: u8 = 0x60;
const REG_DRAM_BLOCK_DATA: u8 = 0xa0;
const REG_DRAM_SEL_N: u8 = 0x10;

/// Write registers (`enum sigma_write_register`).
pub mod wreg {
    /// Clock select.
    pub const CLOCK_SELECT: u8 = 0;
    /// Trigger select: a lookup table word or the parameters.
    pub const TRIGGER_SELECT: u8 = 1;
    /// Trigger select 2.
    pub const TRIGGER_SELECT2: u8 = 2;
    /// Mode.
    pub const MODE: u8 = 3;
    /// The memory row of the next fetch.
    pub const MEMROW: u8 = 4;
    /// Post trigger.
    pub const POST_TRIGGER: u8 = 5;
    /// Trigger in and out options.
    pub const TRIGGER_OPTION: u8 = 6;
    /// The scratch register.
    pub const TEST: u8 = 15;
}

/// Read registers (`enum sigma_read_register`).
pub mod rreg {
    /// The netlist's id.
    pub const ID: u8 = 0;
    /// The trigger position, then the stop position and the mode.
    pub const TRIGGER_POS_LOW: u8 = 1;
    /// Mode.
    pub const MODE: u8 = 7;
}

/// `CLKSEL_*`.
const CLKSEL_CLKSEL8: u8 = 1 << 0;
const CLKSEL_RISING: u8 = 1 << 4;
const CLKSEL_FALLING: u8 = 1 << 5;

/// `TRGSEL2_*`.
pub mod trgsel2 {
    /// The pin of a 100/200 MHz edge trigger.
    pub const PINS_MASK: u8 = 0x07;
    /// The edge, set for falling as the driver uses it.
    pub const PINPOL_RISE: u8 = 1 << 3;
    /// The lookup table address.
    pub const LUT_ADDR_MASK: u8 = 0x0f;
    /// Strobe a lookup table word.
    pub const LUT_WRITE: u8 = 1 << 4;
    /// Trigger programming.
    pub const RESET: u8 = 1 << 5;
    /// LED select 0.
    pub const LEDSEL0: u8 = 1 << 6;
    /// LED select 1.
    pub const LEDSEL1: u8 = 1 << 7;
}

/// `WMR_*`.
pub mod wmr {
    /// Write samples to the memory.
    pub const SDRAMWRITEEN: u8 = 1 << 0;
    /// Read the memory.
    pub const SDRAMREADEN: u8 = 1 << 1;
    /// Reset the trigger.
    pub const TRGRES: u8 = 1 << 2;
    /// Enable the trigger.
    pub const TRGEN: u8 = 1 << 3;
    /// Stop the capture.
    pub const FORCESTOP: u8 = 1 << 4;
    /// Initialise the memory.
    pub const SDRAMINIT: u8 = 1 << 7;
}

/// `RMR_*`.
pub mod rmr {
    /// The write position wrapped.
    pub const ROUND: u8 = 1 << 4;
    /// The trigger matched.
    pub const TRIGGERED: u8 = 1 << 5;
    /// The capture is complete.
    pub const POSTTRIGGERED: u8 = 1 << 6;
}

/// `TRGOPT_*`.
const TRGOPT_TRGOEN: u8 = 1 << 6;
const TRGOPT_TRGOINEN: u8 = 1 << 5;
const TRGOPT_TRGOEVNTEN: u8 = 1 << 4;
const TRGOPT_TRGOOUTEN: u8 = 1 << 3;
const TRGOPT_CLEAR_MASK: u8 = TRGOPT_TRGOINEN | TRGOPT_TRGOEVNTEN | TRGOPT_TRGOOUTEN;

/// Rows of the sample memory.
pub const ROW_COUNT: u64 = 32768;
/// Bytes per row.
pub const ROW_LENGTH_BYTES: usize = 1024;
/// 16 bit words per row.
const ROW_LENGTH_U16: u64 = 512;
/// Bits of a position below the row.
const ROW_SHIFT: u32 = 9;
const ROW_MASK: u32 = (1 << ROW_SHIFT) - 1;
/// Events per cluster.
pub const EVENTS_PER_CLUSTER: u64 = 7;
/// Clusters per row.
pub const CLUSTERS_PER_ROW: u64 = ROW_LENGTH_U16 / (1 + EVENTS_PER_CLUSTER);
/// Events per row.
pub const EVENTS_PER_ROW: u64 = CLUSTERS_PER_ROW * EVENTS_PER_CLUSTER;

/// `SIGMA_MAX_REG_DEPTH`.
const MAX_REG_DEPTH: usize = 32;
/// Rows fetched per read (`lines_per_read`).
pub const LINES_PER_READ: u64 = 32;
/// Reads per poll during a download (`chunks_per_receive_call`).
const CHUNKS_PER_POLL: u32 = 50;
/// How far the software trigger check starts before the hardware's
/// position, and how long it runs.
const TRIG_REWIND: u64 = 4 * EVENTS_PER_CLUSTER;
const TRIG_CHECK: u64 = 8 * EVENTS_PER_CLUSTER;
/// Samples the submit buffer holds (`CHUNK_SIZE` / 2).
const SUBMIT_SAMPLES: usize = 4 * 1024 * 1024 / 2;
/// Mode reads after a forced stop before giving up (the C driver reads
/// for ever).
pub const MAX_STOP_POLLS: u32 = 10_000;
/// The same once a stop was asked for, so that Ctrl-C does not wait for
/// the above: a forced stop that a stop itself began still gets this
/// long to complete and be read.
pub const MAX_STOP_POLLS_STOPPING: u32 = 100;
/// The poll period (`sr_session_source_add`'s timeout).
pub const POLL: Duration = Duration::from_millis(10);

/// The commands that write `data` to register `reg`
/// (`sigma_write_register`): the address, then each byte as two
/// nibbles.
pub fn register_write(reg: u8, data: &[u8]) -> Vec<u8> {
    let mut out = vec![REG_ADDR_LOW | (reg & 0x0f), REG_ADDR_HIGH | (reg >> 4)];
    for d in data {
        out.push(REG_DATA_LOW | (d & 0x0f));
        out.push(REG_DATA_HIGH_WRITE | (d >> 4));
    }
    out
}

/// The commands that read `count` registers from `reg` on
/// (`sigma_get_registers`).
pub fn registers_read(reg: u8, count: usize) -> Vec<u8> {
    let mut out = vec![REG_ADDR_LOW | (reg & 0x0f), REG_ADDR_HIGH | (reg >> 4)];
    out.extend(std::iter::repeat_n(REG_READ_ADDR | REG_ADDR_INC, count));
    out
}

/// The commands that fetch and send `count` rows
/// (`sigma_read_dram`), alternating between the two buffers.
pub fn dram_read(count: usize) -> Vec<u8> {
    let mut out = vec![REG_DRAM_BLOCK, REG_DRAM_WAIT_ACK];
    for chunk in 0..count {
        let sel = chunk % 2 == 1;
        let last = chunk == count - 1;
        if !last {
            out.push(REG_DRAM_BLOCK | if sel { 0 } else { REG_DRAM_SEL_N });
        }
        out.push(REG_DRAM_BLOCK_DATA | if sel { REG_DRAM_SEL_N } else { 0 });
        if !last {
            out.push(REG_DRAM_WAIT_ACK);
        }
    }
    out
}

/// The position a position register's value names (`sigma_read_pos`):
/// one event back, across the row's timestamps when that leaves the
/// row.
pub fn decrement_pos(pos: u32) -> u32 {
    let p = pos.wrapping_sub(1);
    if p & ROW_MASK == ROW_MASK { p.wrapping_sub(CLUSTERS_PER_ROW as u32) } else { p }
}

/// Bit-bang pins (`BB_PIN_*`).
mod pin {
    pub const CCLK: u8 = 1 << 0;
    pub const PROG: u8 = 1 << 1;
    pub const D2: u8 = 1 << 2;
    pub const D3: u8 = 1 << 3;
    pub const INIT: u8 = 1 << 5;
    pub const DIN: u8 = 1 << 6;
    pub const D7: u8 = 1 << 7;
    /// Every pin but INIT is an output.
    pub const MASK: u8 = !INIT;
}

/// `BB_BITRATE`.
const BB_BITRATE: u64 = 750_000;

/// The suicide sequence: D7 held, D2 and D3 alternating.
const SUICIDE: [u8; 8] = [
    pin::D7 | pin::D2,
    pin::D7 | pin::D2,
    pin::D7 | pin::D3,
    pin::D7 | pin::D2,
    pin::D7 | pin::D3,
    pin::D7 | pin::D2,
    pin::D7 | pin::D3,
    pin::D7 | pin::D2,
];

/// The PROG pulse, CCLK idle.
const INIT_ARRAY: [u8; 10] = [
    pin::CCLK,
    pin::CCLK | pin::PROG,
    pin::CCLK | pin::PROG,
    pin::CCLK,
    pin::CCLK,
    pin::CCLK,
    pin::CCLK,
    pin::CCLK,
    pin::CCLK,
    pin::CCLK,
];

/// A netlist file unscrambled (the xor sequence of
/// `sigma_fw_2_bitbang`).
pub fn unscramble(data: &mut [u8]) {
    let mut imm: u32 = 0x3f6d_f2ab;
    for b in data {
        imm = (imm.wrapping_add(0x0a85_3753) % 177).wrapping_add(imm.wrapping_mul(0x0803_4052));
        *b ^= imm as u8;
    }
}

/// The bit-bang samples of a netlist file (`sigma_fw_2_bitbang`):
/// unscrambled, then per bit, most significant first, DIN with CCLK
/// set and DIN alone.
pub fn bitbang_stream(file: &[u8]) -> Vec<u8> {
    let mut data = file.to_vec();
    unscramble(&mut data);
    let mut out = Vec::with_capacity(data.len() * 16);
    for byte in data {
        for bit in (0..8).rev() {
            let v = if byte >> bit & 1 == 1 { pin::DIN } else { 0 };
            out.push(v | pin::CCLK);
            out.push(v);
        }
    }
    out
}

/// The commands of `sigma_fpga_init_la`: read the id, write and read
/// back 0x55 and 0xaa in the scratch register, initialise the memory.
pub fn init_la_commands() -> Vec<u8> {
    let mut out = registers_read(rreg::ID, 0);
    out.push(REG_READ_ADDR);
    for v in [0x55u8, 0xaa] {
        out.extend(register_write(wreg::TEST, &[v]));
        out.push(REG_READ_ADDR);
    }
    out.extend(register_write(wreg::MODE, &[wmr::SDRAMINIT]));
    out
}

/// The trigger as the driver holds it (`struct sigma_trigger`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SigmaTrigger {
    /// Channels that must rise.
    pub rising: u16,
    /// Channels that must fall.
    pub falling: u16,
    /// Channels with a level.
    pub simple_mask: u16,
    /// Their levels.
    pub simple_value: u16,
}

/// `sigma_convert_trigger`: the user's trigger in the driver's form, or
/// `None` without one. Matches on disabled channels are ignored. At 100
/// and 200 MHz one edge of one channel, below one edge and any levels.
///
/// # Errors
///
/// The C driver's message for a trigger the hardware cannot do.
pub fn convert_trigger(
    trigger: Option<&Trigger>,
    enabled: &[bool],
    samplerate: u64,
) -> std::result::Result<Option<SigmaTrigger>, &'static str> {
    let Some(trigger) = trigger else { return Ok(None) };
    let mut t = SigmaTrigger::default();
    let mut edges = 0;
    for stage in &trigger.stages {
        for c in &stage.conditions {
            if !enabled.get(c.channel).copied().unwrap_or(false) {
                continue;
            }
            let bit = 1u16 << (c.channel & 15);
            if samplerate >= MHZ_100 {
                if edges > 0 {
                    return Err("100/200MHz modes limited to single trigger pin.");
                }
                match c.matches {
                    TriggerMatch::Falling => t.falling |= bit,
                    TriggerMatch::Rising => t.rising |= bit,
                    _ => return Err("100/200MHz modes limited to edge trigger."),
                }
                edges += 1;
            } else {
                match c.matches {
                    TriggerMatch::One => {
                        t.simple_value |= bit;
                        t.simple_mask |= bit;
                    }
                    TriggerMatch::Zero => {
                        t.simple_value &= !bit;
                        t.simple_mask |= bit;
                    }
                    TriggerMatch::Falling => {
                        t.falling |= bit;
                        edges += 1;
                    }
                    TriggerMatch::Rising => {
                        t.rising |= bit;
                        edges += 1;
                    }
                    _ => {}
                }
                if edges > 1 {
                    return Err("Limited to 1 edge trigger.");
                }
            }
        }
    }
    Ok(Some(t))
}

/// Trigger select 2 for the 100 and 200 MHz netlists: the first of
/// channels 1 to 8 with an edge (0 without one), the LED, and bit 3
/// for a falling edge.
pub fn fast_trigsel2(t: Option<&SigmaTrigger>) -> u8 {
    let t = t.copied().unwrap_or_default();
    let pin = (0..8u8).find(|p| (t.rising | t.falling) & (1 << p) != 0).unwrap_or(8);
    let mut v = (pin & trgsel2::PINS_MASK) | trgsel2::LEDSEL1;
    if t.falling != 0 {
        v |= trgsel2::PINPOL_RISE;
    }
    v
}

/// The trigger lookup tables (`struct triggerlut`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Lut {
    /// Edge selection 0, one 16 bit table per 4 channels.
    pub m0d: [u16; 4],
    /// Edge selection 1.
    pub m1d: [u16; 4],
    /// The level pattern.
    pub m2d: [u16; 4],
    /// Combines the edge selections.
    pub m3q: u16,
    /// M3S.
    pub m3s: u16,
    /// M4.
    pub m4: u16,
    /// `selres`, `selinc`, `sela`, `cmpa` (the rest is 0).
    pub selres: u8,
    /// See `selres`.
    pub selinc: u8,
}

/// `build_lut_entry`: per group of 4 channels, the table of which of
/// the 16 level combinations meet `value` under `mask`.
pub fn build_lut_entry(value: u16, mask: u16) -> [u16; 4] {
    let mut out = [0u16; 4];
    for (quad, entry) in out.iter_mut().enumerate() {
        *entry = 0xffff;
        for bitidx in 0..16u16 {
            for ch in 0..4 {
                let quadmask = 1u16 << ch;
                let bitmask = quadmask << (quad * 4);
                if mask & bitmask == 0 {
                    continue;
                }
                let value_low = value & bitmask == 0;
                let idx_low = bitidx & quadmask == 0;
                if value_low != idx_low {
                    *entry &= !(1 << bitidx);
                }
            }
        }
    }
    out
}

/// `add_trigger_function` for the one use the driver makes of it: OR
/// an edge (rising or falling) of selection `index` into `mask`.
fn or_edge(rising: bool, index: u32, mask: &mut u16) {
    // x[b][a]: b the level one sample before, a now.
    let x = if rising { [[0, 1], [0, 0]] } else { [[0, 0], [1, 0]] };
    for bitidx in 0..16u16 {
        let a = usize::from(bitidx & (1 << (2 * index)) != 0);
        let b = usize::from(bitidx & (1 << (2 * index + 1)) != 0);
        if x[b][a] == 1 {
            *mask |= 1 << bitidx;
        }
    }
}

/// `sigma_build_basic_trigger`: the lookup tables of a level pattern
/// and at most one edge (the driver's way allows two), or tables that
/// never match without a trigger.
pub fn build_basic_trigger(t: Option<&SigmaTrigger>) -> Lut {
    let mut lut = Lut::default();
    let Some(t) = t else { return lut };
    lut.m4 = 0xa000;
    lut.m3q = 0xffff;
    lut.m2d = build_lut_entry(t.simple_value, t.simple_mask);
    let mut masks = [0u16; 2];
    let mut n = 0;
    for bitidx in 0..16 {
        let mask = 1u16 << bitidx;
        if (t.rising | t.falling) & mask == 0 {
            continue;
        }
        if n == 0 {
            lut.m0d = build_lut_entry(mask, mask);
        } else {
            lut.m1d = build_lut_entry(mask, mask);
        }
        masks[n] = mask;
        n += 1;
        if n == masks.len() {
            break;
        }
    }
    if masks[0] != 0 || masks[1] != 0 {
        lut.m3q = 0;
        for (i, m) in masks.iter().enumerate() {
            let i = i as u32;
            if m & t.rising != 0 {
                or_edge(true, i, &mut lut.m3q);
            }
            if m & t.falling != 0 {
                or_edge(false, i, &mut lut.m3q);
            }
        }
    }
    // Trigger type: event; never reset, counter >= 0.
    lut.selres = 3;
    lut.selinc = 0;
    lut
}

/// The 16 table words (`sigma_write_trigger_lut`): per address, M4,
/// M3S, M3Q in the top nibble, then M2D, M1D, M0D.
pub fn lut_words(lut: &Lut) -> [u16; 16] {
    let bits = |tables: &[u16; 4], bit: u16| {
        tables.iter().enumerate().fold(0u16, |acc, (i, t)| acc | (u16::from(t & bit != 0) << i))
    };
    let mut out = [0u16; 16];
    for (addr, w) in out.iter_mut().enumerate() {
        let bit = 1u16 << addr;
        let m3d = (u16::from(lut.m4 & bit != 0) << 2)
            | (u16::from(lut.m3s & bit != 0) << 1)
            | u16::from(lut.m3q & bit != 0);
        *w = (m3d << 12)
            | (bits(&lut.m2d, bit) << 8)
            | (bits(&lut.m1d, bit) << 4)
            | bits(&lut.m0d, bit);
    }
    out
}

/// The parameter word that follows the tables: `selinc`, `selres`, and
/// the zeros of `sela`, `selb`, `selc` and the prescaler.
pub fn lut_params(lut: &Lut) -> u16 {
    u16::from(lut.selinc & 3) | (u16::from(lut.selres & 3) << 2)
}

/// `sigma_set_acquire_timeout`: the capture time the limits ask for, in
/// ms, with two worst case cluster periods of slack, or `None` without
/// a limit. With a trigger only the post trigger share counts.
pub fn acquire_msecs(
    limit_samples: u64,
    limit_msec: u64,
    samplerate: u64,
    trigger_ratio: Option<u64>,
) -> Option<u64> {
    let share = |v: u64| match trigger_ratio {
        Some(r) => v.saturating_mul(100 - r.min(100)) / 100,
        None => v,
    };
    let count = share(limit_samples);
    let msecs = share(limit_msec);
    let mut acquire = None::<u64>;
    if count != 0 {
        acquire = Some(count.saturating_mul(1000) / samplerate + 1);
    }
    if msecs != 0 && acquire.is_none_or(|a| msecs < a) {
        acquire = Some(msecs);
    }
    acquire.map(|a| a.saturating_add(2 * (1000 * 65536 / samplerate)))
}

/// The clock select bytes of the 50 MHz netlist: the divider, or with an
/// external clock the pin (its index plus 1) and the edges; then the
/// channels that are not there.
pub fn clock_select_50(
    samplerate: u64,
    external: Option<(usize, ClockEdge)>,
    num_channels: usize,
) -> [u8; 4] {
    let pindis = pin_disable(num_channels);
    let (asynch, div) = match external {
        None => (0, (MHZ_50 / samplerate).wrapping_sub(1) as u8),
        Some((pin, edge)) => {
            let edges = match edge {
                ClockEdge::Rising => CLKSEL_RISING,
                ClockEdge::Falling => CLKSEL_FALLING,
                ClockEdge::Either => CLKSEL_RISING | CLKSEL_FALLING,
            };
            (CLKSEL_CLKSEL8, (pin as u8 + 1) | edges)
        }
    };
    let [hi, lo] = pindis.to_be_bytes();
    [asynch, div, hi, lo]
}

/// True when the pin field of the clock select, 4 bits written as the
/// channel's index plus 1, holds `pin`. Channel 16 does not: its 16
/// spills into `CLKSEL_RISING`.
pub fn clock_pin_fits(pin: usize) -> bool {
    pin < 0x0f
}

/// The mask of channels a netlist does not have.
pub fn pin_disable(num_channels: usize) -> u16 {
    !(((1u32 << num_channels) - 1) as u16)
}

/// The trigger option bytes `dev_acquisition_start` writes: trigger out
/// by the trigger, then trigger out enabled.
pub fn trigger_options() -> [u8; 2] {
    let first = TRGOPT_TRGOOUTEN;
    [first, (first & !TRGOPT_CLEAR_MASK) | TRGOPT_TRGOEN]
}

/// The external clock's edges (`enum ext_clock_edge_t`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockEdge {
    /// Rising.
    Rising,
    /// Falling.
    Falling,
    /// Either.
    Either,
}

impl ClockEdge {
    /// The names, in the driver's order.
    pub const NAMES: [&'static str; 3] = ["rising", "falling", "either"];

    /// The name.
    pub fn name(self) -> &'static str {
        Self::NAMES[self as usize]
    }

    /// The edge of a name.
    pub fn from_name(s: &str) -> Option<Self> {
        match s {
            "rising" => Some(ClockEdge::Rising),
            "falling" => Some(ClockEdge::Falling),
            "either" => Some(ClockEdge::Either),
            _ => None,
        }
    }
}

/// `sigma_deinterlace_data_2x8`: sample `idx` of the two 8 channel
/// samples in a 100 MHz event.
pub fn deinterlace_2x8(data: u16, idx: u32) -> u16 {
    let d = data >> idx;
    (0..8).fold(0, |acc, i| acc | ((d >> i) & (1 << i)))
}

/// `sigma_deinterlace_data_4x4`: sample `idx` of the four 4 channel
/// samples in a 200 MHz event.
pub fn deinterlace_4x4(data: u16, idx: u32) -> u16 {
    let d = data >> idx;
    (0..4).fold(0, |acc, i| acc | ((d >> (3 * i)) & (1 << i)))
}

/// A position in the sample memory (`struct sigma_location`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Location {
    /// The row.
    pub line: u64,
    /// The cluster in the row.
    pub cluster: u64,
    /// The event in the cluster.
    pub event: u64,
}

impl Location {
    /// `sigma_location_break_down` of a raw position.
    pub fn from_raw(raw: u64) -> Self {
        let line = (raw / ROW_LENGTH_U16 + ROW_COUNT) % ROW_COUNT;
        let word = raw % ROW_LENGTH_U16;
        Location { line, cluster: word / EVENTS_PER_CLUSTER, event: word % EVENTS_PER_CLUSTER }
    }

    /// `sigma_location_increment`.
    pub fn increment(&mut self) {
        self.event += 1;
        if self.event < EVENTS_PER_CLUSTER {
            return;
        }
        self.event = 0;
        self.cluster += 1;
        if self.cluster < CLUSTERS_PER_ROW {
            return;
        }
        self.cluster = 0;
        self.line += 1;
        if self.line < ROW_COUNT {
            return;
        }
        self.line = 0;
    }

    /// `sigma_location_decrement` with the event.
    pub fn decrement(&mut self) {
        if self.event > 0 {
            self.event -= 1;
            return;
        }
        self.event = EVENTS_PER_CLUSTER - 1;
        if self.cluster > 0 {
            self.cluster -= 1;
            return;
        }
        self.cluster = CLUSTERS_PER_ROW - 1;
        self.line = if self.line > 0 { self.line - 1 } else { ROW_COUNT - 1 };
    }
}

/// Where the decoded samples and trigger markers go.
pub trait Feed {
    /// Samples, 2 bytes each, little endian.
    fn logic(&mut self, data: &[u8]);
    /// The trigger marker.
    fn trigger(&mut self);
}

/// A [`Feed`] into a packet sink.
pub struct SinkFeed<'a>(pub &'a PacketSink);

impl Feed for SinkFeed<'_> {
    fn logic(&mut self, data: &[u8]) {
        self.0.send(Packet::logic(2, data.to_vec()));
    }

    fn trigger(&mut self) {
        self.0.send(Packet::Trigger);
    }
}

/// The interpretation of the sample memory after a capture
/// (`struct sigma_sample_interp`, the submit buffer and the software
/// trigger check).
pub struct Decoder {
    samples_per_event: u64,
    trigger: Option<SigmaTrigger>,
    last_ts: u16,
    last_sample: u16,
    start: Location,
    stop_line: u64,
    /// Events decoded of the last row.
    stop_events: u64,
    trig: Location,
    iter: Location,
    trig_arm: Location,
    armed: bool,
    matched: bool,
    evt_remain: u64,
    lines_total: u64,
    lines_done: u64,
    buffer: Vec<u8>,
    limit: u64,
    samples_read: u64,
}

impl Decoder {
    /// `alloc_sample_buffer`: the rows to read from the decremented stop
    /// and trigger positions (the trigger `u32::MAX` when there is none)
    /// and the mode register. A memory that wrapped is read from the row
    /// after the one being written to the row before it, all of that
    /// row; else from row 0 to the stop position, which is not included
    /// (the C driver's count, see the module documentation).
    pub fn new(
        stop_pos: u32,
        trig_pos: u32,
        mode: u8,
        num_channels: usize,
        trigger: Option<SigmaTrigger>,
        limit_samples: u64,
    ) -> Self {
        let stop_pos = u64::from(stop_pos);
        let row = stop_pos >> ROW_SHIFT;
        let (start, stop_line, stop_events) = if mode & rmr::ROUND != 0 {
            let start = Location::from_raw((row + 1) << ROW_SHIFT);
            (start, (row + ROW_COUNT - 1) % ROW_COUNT, EVENTS_PER_ROW)
        } else {
            let stop = Location::from_raw(stop_pos);
            (Location::default(), stop.line, stop_pos & u64::from(ROW_MASK))
        };
        let trig = Location::from_raw(u64::from(trig_pos));
        let mut d = Decoder {
            samples_per_event: (16 / num_channels.max(1)) as u64,
            trigger,
            last_ts: 0,
            last_sample: 0,
            start,
            stop_line,
            stop_events,
            trig,
            iter: Location::default(),
            trig_arm: Location::from_raw(u64::MAX),
            armed: false,
            matched: false,
            evt_remain: 0,
            lines_total: (stop_line + 1 + ROW_COUNT - start.line) % ROW_COUNT,
            lines_done: 0,
            buffer: Vec::new(),
            limit: limit_samples,
            samples_read: 0,
        };
        // rewind_trig_arm_pos.
        if d.trigger.is_some() {
            d.trig_arm = d.trig;
            for _ in 0..TRIG_REWIND {
                if d.trig_arm == d.start {
                    break;
                }
                d.trig_arm.decrement();
            }
        }
        d
    }

    /// Rows in all.
    pub fn lines_total(&self) -> u64 {
        self.lines_total
    }

    /// Rows decoded.
    pub fn lines_done(&self) -> u64 {
        self.lines_done
    }

    /// True when every row is decoded.
    pub fn done(&self) -> bool {
        self.lines_done >= self.lines_total
    }

    /// The first row of the next read and how many rows it takes.
    pub fn next_fetch(&mut self) -> (u64, u64) {
        if self.lines_done == 0 {
            self.iter = self.start;
        }
        let count = (self.lines_total - self.lines_done).min(LINES_PER_READ);
        (self.iter.line, count)
    }

    /// Decodes `count` rows just read (`fetch_sample_buffer`'s seeding
    /// and `download_capture`'s loop).
    pub fn decode(&mut self, rows: &[u8], count: u64, feed: &mut dyn Feed) {
        if self.lines_done == 0 && rows.len() >= 4 {
            self.last_ts = u16::from_le_bytes([rows[0], rows[1]]);
            let data = u16::from_le_bytes([rows[2], rows[3]]);
            self.last_sample = match self.samples_per_event {
                4 => deinterlace_4x4(data, 0),
                2 => deinterlace_2x8(data, 0),
                _ => data,
            };
            // The check the C driver makes only after each event, made
            // before the first too: a trigger near the start gets its
            // marker.
            self.location_check(feed);
        }
        for row in rows.as_chunks::<ROW_LENGTH_BYTES>().0.iter().take(count as usize) {
            let events =
                if self.iter.line == self.stop_line { self.stop_events } else { EVENTS_PER_ROW };
            self.decode_row(row, events, feed);
            self.lines_done += 1;
        }
    }

    /// `decode_chunk_ts`.
    fn decode_row(&mut self, row: &[u8], events: u64, feed: &mut dyn Feed) {
        let clusters = events.div_ceil(EVENTS_PER_CLUSTER);
        for (i, cluster) in row.as_chunks::<16>().0.iter().take(clusters as usize).enumerate() {
            let n = if i as u64 == clusters - 1 && !events.is_multiple_of(EVENTS_PER_CLUSTER) {
                events % EVENTS_PER_CLUSTER
            } else {
                EVENTS_PER_CLUSTER
            };
            self.decode_cluster(cluster, n, feed);
        }
    }

    /// `sigma_decode_dram_cluster`.
    fn decode_cluster(&mut self, cluster: &[u8], events: u64, feed: &mut dyn Feed) {
        let ts = u16::from_le_bytes([cluster[0], cluster[1]]);
        let tsdiff = ts.wrapping_sub(self.last_ts);
        if tsdiff > 0 {
            let sample = self.last_sample;
            self.check_and_submit(sample, u64::from(tsdiff) * self.samples_per_event, feed);
        }
        self.last_ts = ts.wrapping_add(EVENTS_PER_CLUSTER as u16);
        for evt in 0..events as usize {
            let item = u16::from_le_bytes([cluster[2 + 2 * evt], cluster[3 + 2 * evt]]);
            for idx in 0..self.samples_per_event as u32 {
                let sample = match self.samples_per_event {
                    4 => deinterlace_4x4(item, idx),
                    2 => deinterlace_2x8(item, idx),
                    _ => item,
                };
                self.check_and_submit(sample, 1, feed);
                self.last_sample = sample;
            }
            self.iter.increment();
            self.location_check(feed);
        }
    }

    /// `sample_matches_trigger`.
    fn matches_trigger(&self, sample: u16) -> bool {
        let Some(t) = &self.trigger else { return false };
        if !self.armed {
            return false;
        }
        let last = self.last_sample;
        sample & t.simple_mask == t.simple_value
            && last & t.rising == 0
            && sample & t.rising == t.rising
            && last & t.falling == t.falling
            && sample & t.falling == 0
    }

    /// `check_and_submit_sample`.
    fn check_and_submit(&mut self, sample: u16, count: u64, feed: &mut dyn Feed) {
        if self.matches_trigger(sample) {
            self.marker(feed);
            self.matched = true;
        }
        self.submit(sample, count, feed);
    }

    fn marker(&mut self, feed: &mut dyn Feed) {
        self.flush(feed);
        feed.trigger();
    }

    fn limit_reached(&self) -> bool {
        self.limit != 0 && self.samples_read >= self.limit
    }

    /// `addto_submit_buffer`: without a trigger, up to the sample limit.
    fn submit(&mut self, sample: u16, count: u64, feed: &mut dyn Feed) {
        let unlimited = self.trigger.is_some();
        if !unlimited && self.limit_reached() {
            return;
        }
        for _ in 0..count {
            self.buffer.extend_from_slice(&sample.to_le_bytes());
            if self.buffer.len() == SUBMIT_SAMPLES * 2 {
                self.flush(feed);
            }
            self.samples_read += 1;
            if !unlimited && self.limit_reached() {
                break;
            }
        }
    }

    /// `flush_submit_buffer`.
    pub fn flush(&mut self, feed: &mut dyn Feed) {
        if !self.buffer.is_empty() {
            feed.logic(&self.buffer);
            self.buffer.clear();
        }
    }

    /// `sigma_location_check`: arms the software check at the rewound
    /// position, disarms it after a match or 56 events, and places the
    /// marker at the hardware's position when the check found nothing.
    fn location_check(&mut self, feed: &mut dyn Feed) {
        if self.armed {
            self.evt_remain = self.evt_remain.saturating_sub(1);
            if self.evt_remain == 0 || self.matched {
                self.armed = false;
            }
        }
        if !self.armed && !self.matched && self.trigger.is_some() && self.iter == self.trig_arm {
            self.armed = true;
            self.matched = false;
            self.evt_remain = TRIG_CHECK;
        }
        if self.armed && self.iter == self.trig {
            self.marker(feed);
            self.matched = true;
        }
    }
}

/// A software limit (`struct sr_sw_limits`, the time part): expired once
/// more than `limit` has passed since `start`, never before it started.
#[derive(Clone, Copy, Debug, Default)]
pub struct TimeLimit {
    /// The limit.
    pub limit: Option<Duration>,
    /// When it started.
    pub start: Option<Instant>,
}

impl TimeLimit {
    /// `sr_sw_limits_check` of the time.
    pub fn expired(&self) -> bool {
        match (self.limit, self.start) {
            (Some(l), Some(s)) => s.elapsed() > l,
            _ => false,
        }
    }
}

/// libftdi's port to the SIGMA.
pub struct Link {
    ctx: Context,
}

/// A transfer failed or brought less than asked (`SR_ERR_IO`).
fn io() -> Error {
    Error::device("USB data transfer failed")
}

impl Link {
    /// A link over an opened chip.
    pub fn new(ctx: Context) -> Self {
        Link { ctx }
    }

    /// `sigma_read_raw`: the bytes read, logged on failure.
    fn read_raw(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.ctx.read_data(buf).map_err(|e| {
            log::error!("USB data read failed: {}", e.message);
            io()
        })
    }

    /// `sigma_write_raw`.
    fn write_raw(&mut self, buf: &[u8]) -> Result<usize> {
        match self.ctx.write_data(buf) {
            Ok(n) => {
                if n != buf.len() {
                    log::error!("USB data write length mismatch.");
                }
                Ok(n)
            }
            Err(e) => {
                log::error!("USB data write failed: {}", e.message);
                Err(io())
            }
        }
    }

    /// `sigma_read_sr`: exactly `buf.len()` bytes.
    fn read_sr(&mut self, buf: &mut [u8]) -> Result<()> {
        if self.read_raw(buf)? == buf.len() { Ok(()) } else { Err(io()) }
    }

    /// `sigma_write_sr`: all of `buf`.
    fn write_sr(&mut self, buf: &[u8]) -> Result<()> {
        if self.write_raw(buf)? == buf.len() { Ok(()) } else { Err(io()) }
    }

    /// `sigma_write_register`.
    ///
    /// # Errors
    ///
    /// A failed write.
    pub fn write_register(&mut self, reg: u8, data: &[u8]) -> Result<()> {
        if data.len() > MAX_REG_DEPTH {
            log::error!("Short write buffer for {} bytes to reg {reg}.", data.len());
            return Err(Error::arg("register write too long"));
        }
        self.write_sr(&register_write(reg, data))
    }

    /// `sigma_set_register`.
    ///
    /// # Errors
    ///
    /// A failed write.
    pub fn set_register(&mut self, reg: u8, value: u8) -> Result<()> {
        self.write_register(reg, &[value])
    }

    /// `sigma_get_register`.
    ///
    /// # Errors
    ///
    /// A failed write or read.
    pub fn get_register(&mut self, reg: u8) -> Result<u8> {
        let mut cmd = registers_read(reg, 0);
        cmd.push(REG_READ_ADDR);
        self.write_sr(&cmd)?;
        let mut v = [0u8];
        self.read_sr(&mut v)?;
        Ok(v[0])
    }

    /// `sigma_read_pos`: the decremented stop and trigger positions, and
    /// the mode register.
    ///
    /// # Errors
    ///
    /// A failed write or read.
    pub fn read_pos(&mut self) -> Result<(u32, u32, u8)> {
        self.write_sr(&registers_read(rreg::TRIGGER_POS_LOW, 7))?;
        let mut r = [0u8; 7];
        self.read_sr(&mut r)?;
        let trig = u32::from_le_bytes([r[0], r[1], r[2], 0]);
        let stop = u32::from_le_bytes([r[3], r[4], r[5], 0]);
        Ok((decrement_pos(stop), decrement_pos(trig), r[6]))
    }

    /// `sigma_read_dram`: `count` rows from `start` into `data`.
    ///
    /// # Errors
    ///
    /// A failed write or read.
    pub fn read_dram(&mut self, start: u64, count: u64, data: &mut Vec<u8>) -> Result<()> {
        self.write_register(wreg::MEMROW, &((start & 0xffff) as u16).to_be_bytes())?;
        self.write_sr(&dram_read(count as usize))?;
        data.resize(count as usize * ROW_LENGTH_BYTES, 0);
        self.read_sr(data)
    }

    /// `sigma_write_trigger_lut`: each table word with its strobe, then
    /// the parameters.
    ///
    /// # Errors
    ///
    /// A failed write.
    pub fn write_trigger_lut(&mut self, lut: &Lut) -> Result<()> {
        for (addr, w) in lut_words(lut).iter().enumerate() {
            self.write_register(wreg::TRIGGER_SELECT, &w.to_be_bytes())?;
            let strobe =
                trgsel2::RESET | trgsel2::LUT_WRITE | (addr as u8 & trgsel2::LUT_ADDR_MASK);
            self.set_register(wreg::TRIGGER_SELECT2, strobe)?;
        }
        let mut params = lut_params(lut).to_be_bytes().to_vec();
        // cmpb, cmpa: 0.
        params.extend_from_slice(&[0, 0, 0, 0]);
        self.write_register(wreg::TRIGGER_SELECT, &params)
    }

    /// `sigma_fpga_init_bitbang_once`: suicide, PROG pulse, purge, wait
    /// for INIT. `Error::Timeout` when INIT never came.
    fn fpga_init_bitbang_once(&mut self) -> Result<()> {
        // All four bursts go out whatever happens to one.
        let mut ok = true;
        for _ in 0..4 {
            ok &= self.write_sr(&SUICIDE).is_ok();
        }
        if !ok {
            return Err(io());
        }
        std::thread::sleep(Duration::from_millis(10));
        self.write_sr(&INIT_ARRAY)?;
        std::thread::sleep(Duration::from_millis(10));
        let _ = self.ctx.tcioflush();
        for retry in (0..10).rev() {
            loop {
                let mut data = [0u8];
                let n = self.read_raw(&mut data)?;
                if n == 1 && data[0] & pin::INIT != 0 {
                    return Ok(());
                }
                if n != 1 {
                    break;
                }
            }
            if retry > 0 {
                std::thread::sleep(Duration::from_millis(10));
            }
        }
        Err(Error::Timeout)
    }

    /// `sigma_fpga_init_bitbang`: up to 10 times while INIT does not
    /// come.
    fn fpga_init_bitbang(&mut self) -> Result<()> {
        let mut ret = Err(Error::Timeout);
        for _ in 0..10 {
            ret = self.fpga_init_bitbang_once();
            match ret {
                Err(Error::Timeout) => {}
                _ => return ret,
            }
        }
        ret
    }

    /// `sigma_fpga_init_la`: the netlist must answer its id and the
    /// scratch register.
    fn fpga_init_la(&mut self) -> Result<()> {
        if let Err(e) = self.write_sr(&init_la_commands()) {
            log::error!("Could not request LA start response.");
            return Err(e);
        }
        let mut r = [0u8; 3];
        if self.read_sr(&mut r).is_err() {
            log::error!("Could not receive LA start response.");
            return Err(io());
        }
        let bad = |msg: &str| {
            log::error!("{msg}");
            Err(Error::device(msg.trim_end_matches('.').to_owned()))
        };
        if r[0] != 0xa6 {
            return bad("Unexpected ID response.");
        }
        if r[1] != 0x55 {
            return bad("Unexpected scratch read-back (55).");
        }
        if r[2] != 0xaa {
            return bad("Unexpected scratch read-back (aa).");
        }
        Ok(())
    }

    /// `upload_firmware`: configures the FPGA with `fw` unless `running`
    /// says it runs it already, and keeps `running` up to date. Unlike
    /// the C driver, a failed upload leaves it empty: the old netlist
    /// is gone from the FPGA by then.
    ///
    /// # Errors
    ///
    /// The step that failed, logged with the C driver's message.
    pub fn upload_firmware(&mut self, fw: Firmware, running: &mut Option<Firmware>) -> Result<()> {
        self.upload_with(fw, running, load_firmware)
    }

    fn upload_with(
        &mut self,
        fw: Firmware,
        running: &mut Option<Firmware>,
        load: impl FnOnce(&str) -> Result<Vec<u8>>,
    ) -> Result<()> {
        let name = fw.file();
        if *running == Some(fw) {
            log::info!("Not uploading firmware file '{name}' again.");
            return Ok(());
        }
        *running = None;
        if let Err(e) = self.ctx.set_bitmode(pin::MASK, crate::ftdi::BitMode::BitBang as u8) {
            log::error!("Could not setup cable mode for upload: {}", e.message);
            return Err(e.into());
        }
        if let Err(e) = self.ctx.set_baudrate(BB_BITRATE) {
            log::error!("Could not setup bitrate for upload: {}", e.message);
            return Err(e.into());
        }
        if let Err(e) = self.fpga_init_bitbang() {
            log::error!("Could not initiate firmware upload to hardware");
            return Err(e);
        }
        let stream = match load(name) {
            Ok(s) => s,
            Err(e) => {
                log::error!("Could not prepare file {name} for upload.");
                return Err(e);
            }
        };
        log::info!("Uploading firmware file '{name}'.");
        if let Err(e) = self.write_sr(&stream) {
            log::error!("Could not upload firmware file '{name}'.");
            return Err(e);
        }
        if let Err(e) = self.ctx.set_bitmode(0, crate::ftdi::BitMode::Reset as u8) {
            log::error!("Could not setup cable mode after upload: {}", e.message);
            return Err(e.into());
        }
        let _ = self.ctx.tcioflush();
        let mut pins = [0u8];
        while matches!(self.read_raw(&mut pins), Ok(n) if n > 0) {}
        if let Err(e) = self.fpga_init_la() {
            log::error!("Hardware response after firmware upload failed.");
            return Err(e);
        }
        *running = Some(fw);
        log::info!("Firmware uploaded.");
        Ok(())
    }
}

/// `sigma_fw_2_bitbang` on the file `name` from the firmware
/// directories.
fn load_firmware(name: &str) -> Result<Vec<u8>> {
    let data = crate::core::firmware::read(
        name,
        "install sigrok-firmware's ASIX SIGMA netlists, see docs/DEVICES.md",
    )?;
    if data.len() > FIRMWARE_SIZE_LIMIT {
        return Err(Error::Firmware(format!("{name} is larger than {FIRMWARE_SIZE_LIMIT} bytes")));
    }
    Ok(bitbang_stream(&data))
}

/// The state of an acquisition (`devc->state`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Capture,
    Stopping,
    Download,
    Idle,
}

/// One acquisition, polled on its own thread (`sigma_receive_data`).
pub struct Acquisition {
    /// The trigger, if any.
    pub trigger: Option<SigmaTrigger>,
    /// The channels of the netlist.
    pub num_channels: usize,
    /// The user's sample limit.
    pub limit_samples: u64,
    /// The capture's time limit.
    pub acquire: TimeLimit,
    /// The time limit starts at the trigger.
    pub late_trigger_timeout: bool,
    state: State,
    decoder: Option<Decoder>,
    rows: Vec<u8>,
}

impl Acquisition {
    /// An acquisition in capture.
    pub fn new(
        trigger: Option<SigmaTrigger>,
        num_channels: usize,
        limit_samples: u64,
        acquire: TimeLimit,
        late_trigger_timeout: bool,
    ) -> Self {
        Acquisition {
            trigger,
            num_channels,
            limit_samples,
            acquire,
            late_trigger_timeout,
            state: State::Capture,
            decoder: None,
            rows: Vec::new(),
        }
    }

    /// Polls every 10 ms until the capture is read or fails, then sends
    /// the end. A stop during the capture forces it to end and reads
    /// what it holds; during the readout it ends the acquisition.
    pub fn run(
        &mut self,
        link: &mut Link,
        sink: &PacketSink,
        stop: &std::sync::atomic::AtomicBool,
    ) {
        let mut stop_seen = false;
        loop {
            std::thread::sleep(POLL);
            if !stop_seen && stop.load(std::sync::atomic::Ordering::SeqCst) {
                stop_seen = true;
                if self.state == State::Capture {
                    self.state = State::Stopping;
                } else {
                    break;
                }
            }
            let mut feed = SinkFeed(sink);
            let go_on = match self.state {
                State::Idle => break,
                State::Stopping | State::Download => self.download(link, &mut feed, stop),
                State::Capture => self.capture(link, &mut feed, stop),
            };
            if !go_on || self.state == State::Idle {
                break;
            }
        }
        sink.send(Packet::End);
    }

    /// `sigma_capture_mode`: false when the acquisition is to end.
    fn capture(
        &mut self,
        link: &mut Link,
        feed: &mut dyn Feed,
        stop: &std::sync::atomic::AtomicBool,
    ) -> bool {
        let Ok((_, _, mode)) = link.read_pos() else { return false };
        if mode & rmr::POSTTRIGGERED != 0 {
            return self.download(link, feed, stop);
        }
        if self.acquire.expired() {
            return self.download(link, feed, stop);
        }
        if self.late_trigger_timeout && mode & rmr::TRIGGERED != 0 {
            self.acquire.start = Some(Instant::now());
            self.late_trigger_timeout = false;
        }
        if self.trigger.is_none() && mode & rmr::ROUND != 0 {
            return self.download(link, feed, stop);
        }
        true
    }

    /// `download_capture`: forces the stop if needed, then reads and
    /// decodes up to 50 reads of rows; false when the acquisition is to
    /// end. Sets the state idle once all is read.
    fn download(
        &mut self,
        link: &mut Link,
        feed: &mut dyn Feed,
        stop: &std::sync::atomic::AtomicBool,
    ) -> bool {
        let Ok(mode) = link.get_register(rreg::MODE) else {
            log::error!("Could not determine current device state.");
            return false;
        };
        if mode & rmr::POSTTRIGGERED == 0 {
            log::info!("Downloading sample data.");
            self.state = State::Download;
            if link.set_register(wreg::MODE, wmr::FORCESTOP | wmr::SDRAMWRITEEN).is_err() {
                return false;
            }
            let mut polls = 0;
            loop {
                let Ok(m) = link.get_register(rreg::MODE) else {
                    log::error!("Could not poll for post-trigger state.");
                    return false;
                };
                if m & rmr::POSTTRIGGERED != 0 {
                    break;
                }
                polls += 1;
                let stopping = stop.load(std::sync::atomic::Ordering::SeqCst);
                if polls >= if stopping { MAX_STOP_POLLS_STOPPING } else { MAX_STOP_POLLS } {
                    log::error!("Could not poll for post-trigger state.");
                    return false;
                }
            }
        }
        if self.decoder.is_none() {
            if link.set_register(wreg::MODE, wmr::SDRAMREADEN).is_err() {
                return false;
            }
            let Ok((stop_pos, trig_pos, mode)) = link.read_pos() else {
                log::error!("Could not query capture positions/state.");
                return false;
            };
            let trig_pos = if self.trigger.is_none() || mode & rmr::TRIGGERED == 0 {
                u32::MAX
            } else {
                trig_pos
            };
            self.decoder = Some(Decoder::new(
                stop_pos,
                trig_pos,
                mode,
                self.num_channels,
                self.trigger,
                self.limit_samples,
            ));
        }
        let Some(dec) = self.decoder.as_mut() else { return false };
        let mut chunks = CHUNKS_PER_POLL;
        while !dec.done() {
            let (first, count) = dec.next_fetch();
            if link.read_dram(first, count, &mut self.rows).is_err() {
                return false;
            }
            dec.decode(&self.rows, count, feed);
            chunks -= 1;
            if chunks == 0 {
                dec.flush(feed);
                break;
            }
        }
        if dec.done() {
            dec.flush(feed);
            self.state = State::Idle;
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::TriggerStage;
    use crate::core::trigger::TriggerCondition;

    #[test]
    fn samplerates_normalise_as_the_driver_does() {
        assert_eq!(normalize_samplerate(200_000_000), Some(200_000_000));
        assert_eq!(normalize_samplerate(100_000_000), Some(100_000_000));
        assert_eq!(normalize_samplerate(30_000_000), Some(50_000_000));
        assert_eq!(normalize_samplerate(20_000_000), Some(25_000_000));
        assert_eq!(normalize_samplerate(200_000), Some(200_000));
        assert_eq!(normalize_samplerate(199_999), None);
        assert_eq!(normalize_samplerate(60_000_000), None);
        assert_eq!(normalize_samplerate(150_000_000), None);
        assert_eq!(Firmware::for_rate(25_000_000), Some((Firmware::Mhz50, 16)));
        assert_eq!(Firmware::for_rate(200_000_000), Some((Firmware::Mhz200, 4)));
    }

    #[test]
    fn commands_as_the_emulator_saw_them() {
        // tests/emu/asix-sigma/50-mhz.tx.
        assert_eq!(init_la_commands(), hex("0010400f102535400f102a3a4003102038"));
        assert_eq!(register_write(wreg::TRIGGER_OPTION, &trigger_options()), hex("061028302034"));
        assert_eq!(registers_read(rreg::TRIGGER_POS_LOW, 7), hex("011041414141414141"));
        assert_eq!(register_write(wreg::MEMROW, &[0, 32]), hex("041020302032"));
        let d = dram_read(32);
        assert_eq!(d.len(), 96);
        assert_eq!(&d[..8], &hex("605070a05060b050")[..]);
        assert_eq!(&d[90..], &hex("b05070a050b0")[..]);
        assert_eq!(dram_read(1), hex("6050a0"));
        assert_eq!(clock_select_50(50_000_000, None, 16), [0, 0, 0, 0]);
        assert_eq!(clock_select_50(200_000, None, 16), [0, 249, 0, 0]);
        assert_eq!(clock_select_50(50_000_000, Some((2, ClockEdge::Falling)), 16), [1, 0x23, 0, 0]);
        // Channel 16 as the C driver writes it (api.c:578).
        assert!(clock_pin_fits(14));
        assert!(!clock_pin_fits(15));
        assert_eq!(clock_select_50(50_000_000, Some((15, ClockEdge::Rising)), 16), [1, 0x10, 0, 0]);
        assert_eq!(
            clock_select_50(50_000_000, Some((15, ClockEdge::Falling)), 16),
            [1, 0x30, 0, 0]
        );
        assert_eq!(pin_disable(8), 0xff00);
        assert_eq!(pin_disable(16), 0);
    }

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    #[test]
    fn positions_decrement_across_the_timestamps() {
        assert_eq!(decrement_pos(0x2a), 0x29);
        // Event 0 of row 1: the last event (447) of row 0.
        assert_eq!(decrement_pos(0x200), 447);
        assert_eq!(decrement_pos(0), 0xffff_ffbf);
        let l = Location::from_raw(0x229);
        assert_eq!(l, Location { line: 1, cluster: 41 / 7, event: 41 % 7 });
        let mut m = Location { line: 0, cluster: 0, event: 0 };
        m.decrement();
        assert_eq!(m, Location { line: ROW_COUNT - 1, cluster: 63, event: 6 });
        m.increment();
        assert_eq!(m, Location::default());
    }

    #[test]
    fn the_netlist_goes_out_as_pin_pairs() {
        let mut once = [0u8; 4];
        unscramble(&mut once);
        let s = bitbang_stream(&[0; 4]);
        assert_eq!(s.len(), 64);
        // The first unscrambled byte's top bit, as DIN with and without
        // CCLK.
        let din = if once[0] & 0x80 != 0 { 0x40 } else { 0 };
        assert_eq!(&s[..2], &[din | 1, din]);
        // Scrambling twice is the identity.
        let mut twice = once;
        unscramble(&mut twice);
        assert_eq!(twice, [0; 4]);
    }

    fn trig(conds: &[(usize, TriggerMatch)]) -> Trigger {
        Trigger {
            stages: vec![TriggerStage {
                conditions: conds
                    .iter()
                    .map(|&(channel, matches)| TriggerCondition { channel, matches, value: 0.0 })
                    .collect(),
                ..TriggerStage::default()
            }],
            ..Trigger::default()
        }
    }

    #[test]
    fn triggers_convert_as_the_driver_does() {
        let on = [true; 16];
        let t = convert_trigger(
            Some(&trig(&[(0, TriggerMatch::Zero), (3, TriggerMatch::Rising)])),
            &on,
            50_000_000,
        );
        assert_eq!(
            t,
            Ok(Some(SigmaTrigger { rising: 8, falling: 0, simple_mask: 1, simple_value: 0 }))
        );
        let two = trig(&[(0, TriggerMatch::Rising), (1, TriggerMatch::Falling)]);
        assert_eq!(convert_trigger(Some(&two), &on, 50_000_000), Err("Limited to 1 edge trigger."));
        assert_eq!(
            convert_trigger(Some(&two), &on, 100_000_000),
            Err("100/200MHz modes limited to single trigger pin.")
        );
        assert_eq!(
            convert_trigger(Some(&trig(&[(2, TriggerMatch::One)])), &on, 200_000_000),
            Err("100/200MHz modes limited to edge trigger.")
        );
        // Disabled channels are ignored, the trigger stays.
        let mut off = on;
        off[1] = false;
        assert_eq!(convert_trigger(Some(&two), &off, 50_000_000).unwrap().unwrap().rising, 1);
        assert_eq!(convert_trigger(None, &on, 50_000_000), Ok(None));
        let f = SigmaTrigger { falling: 1 << 5, ..SigmaTrigger::default() };
        assert_eq!(fast_trigsel2(Some(&f)), 0x80 | 0x08 | 5);
        assert_eq!(fast_trigsel2(None), 0x80);
    }

    #[test]
    fn lookup_tables_as_the_emulator_saw_them() {
        // No trigger: all zero but the never resetting parameters.
        let none = build_basic_trigger(None);
        assert_eq!(lut_words(&none), [0; 16]);
        assert_eq!(lut_params(&none), 0);
        // Channel 5 rising, as tests/emu/asix-sigma/trigger-rising.tx
        // has the words written.
        let t = SigmaTrigger { rising: 1 << 4, ..SigmaTrigger::default() };
        let lut = build_basic_trigger(Some(&t));
        assert_eq!(lut.m3q, 0x2222);
        assert_eq!(lut.m0d, [0xffff, 0xaaaa, 0xffff, 0xffff]);
        assert_eq!(lut_params(&lut), 0x000c);
        let w = lut_words(&lut);
        assert_eq!((w[0], w[1], w[2], w[13], w[15]), (0x0f0d, 0x1f0f, 0x0f0d, 0x5f0f, 0x4f0f));
        // A level: channel 1 high.
        assert_eq!(build_lut_entry(1, 1), [0xaaaa, 0xffff, 0xffff, 0xffff]);
        assert_eq!(build_lut_entry(0, 1), [0x5555, 0xffff, 0xffff, 0xffff]);
    }

    #[test]
    fn timeouts_as_the_driver_computes_them() {
        // 100 samples at 50 MHz: 1 ms + 2 * 1 ms.
        assert_eq!(acquire_msecs(100, 0, 50_000_000, None), Some(3));
        assert_eq!(acquire_msecs(0, 0, 50_000_000, None), None);
        assert_eq!(acquire_msecs(0, 2, 50_000_000, None), Some(4));
        // The lesser of both.
        assert_eq!(acquire_msecs(1_000_000_000, 5, 50_000_000, None), Some(7));
        // 200 kHz: 327 ms per cluster.
        assert_eq!(acquire_msecs(40, 0, 200_000, Some(50)), Some(1 + 654));
        assert_eq!(acquire_msecs(40, 0, 200_000, Some(100)), None);
        // Huge limits saturate rather than overflow.
        assert!(acquire_msecs(u64::MAX, 0, 200_000, None).unwrap() > 1 << 40);
        assert!(acquire_msecs(u64::MAX, u64::MAX, 200_000, Some(1)).unwrap() > 1 << 40);
        assert_eq!(acquire_msecs(0, u64::MAX, 50_000_000, None), Some(u64::MAX));
    }

    /// A link over a replayed FT245 (64 byte packets) after libftdi's
    /// open.
    fn link(steps: &str) -> Link {
        linked(steps).0
    }

    /// [`link`], and the replay behind it, to see that it was played out.
    fn linked(steps: &str) -> (Link, crate::usb::record::Replay) {
        let text = format!(
            "# oxrok usb fixture 1\ndevice vid=a600 pid=a000 version=0400 bus=0 addr=0 \
             strings=0,0,0\nclaim 0 -> ok\nmps 81 -> 64\n\
             ctl_out 40 00 0000 0001 - -> ok\nctl_out 40 03 4138 0000 - -> ok\n{steps}"
        );
        let t = crate::usb::record::Replay::from_text(&text).unwrap();
        (Link::new(Context::open(Box::new(t.clone())).unwrap()), t)
    }

    /// `data` as the chip sends it: two status bytes per 64 byte packet.
    fn packets(data: &[u8]) -> String {
        use std::fmt::Write as _;
        let mut s = String::new();
        for p in data.chunks(62) {
            s.push_str("0160");
            for b in p {
                let _ = write!(s, "{b:02x}");
            }
        }
        s
    }

    #[test]
    fn a_stop_during_the_capture_forces_it_to_end_and_reads_it() {
        // sigrok-cli has no way to ask for this, so no emulator case
        // can: the SIGINT lands at a time the transactions would show.
        let steps = format!(
            "bulk_out 02 071040 -> 3\nbulk_in 81 4096 -> {}\n\
             bulk_out 02 03102131 -> 4\n\
             bulk_out 02 071040 -> 3\nbulk_in 81 4096 -> {}\n\
             bulk_out 02 03102230 -> 4\n\
             bulk_out 02 011041414141414141 -> 9\nbulk_in 81 4096 -> {}\n\
             bulk_out 02 041020302030 -> 6\nbulk_out 02 6050a0 -> 3\n\
             bulk_in 81 4096 -> {}\n",
            packets(&[0x01]),
            packets(&[0x40]),
            // No trigger position; stop register 11; the mode.
            packets(&[0, 0, 0, 11, 0, 0, 0x40]),
            packets(&row(0)),
        );
        let mut link = link(&steps);
        let mut acq = Acquisition::new(None, 16, 0, TimeLimit::default(), false);
        let (tx, rx) = std::sync::mpsc::sync_channel(16);
        let stop = std::sync::atomic::AtomicBool::new(true);
        acq.run(&mut link, &PacketSink::new(tx, 0), &stop);
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        assert_eq!(packets.len(), 2, "{packets:?}");
        // Events 0 to 9.
        assert!(matches!(&packets[0], Packet::Logic(l) if l.data.len() == 20), "{packets:?}");
        assert!(matches!(packets[1], Packet::End));
    }

    #[test]
    fn a_stop_during_the_readout_ends_the_acquisition() {
        let mut acq = Acquisition::new(None, 16, 0, TimeLimit::default(), false);
        acq.state = State::Download;
        let (tx, rx) = std::sync::mpsc::sync_channel(16);
        let stop = std::sync::atomic::AtomicBool::new(true);
        // Nothing more goes to the device.
        acq.run(&mut link(""), &PacketSink::new(tx, 0), &stop);
        assert!(matches!(rx.recv().unwrap().1, Packet::End));
    }

    #[test]
    fn a_forced_stop_that_never_ends_gives_up() {
        use std::sync::atomic::AtomicBool;
        for (stop, polls) in [(false, MAX_STOP_POLLS), (true, MAX_STOP_POLLS_STOPPING)] {
            let mut steps = String::from(
                "bulk_out 02 071040 -> 3\nbulk_in 81 4096 -> 016001\nbulk_out 02 03102131 -> 4\n",
            );
            for _ in 0..polls {
                steps.push_str("bulk_out 02 071040 -> 3\nbulk_in 81 4096 -> 016001\n");
            }
            let (mut link, replay) = linked(&steps);
            let mut acq = Acquisition::new(None, 16, 0, TimeLimit::default(), false);
            let mut rec = Rec::default();
            assert!(!acq.download(&mut link, &mut rec, &AtomicBool::new(stop)));
            assert_eq!(replay.remaining(), 0, "stop {stop}");
        }
    }

    /// The read of the positions, with the mode.
    fn position_read(mode: u8) -> String {
        format!(
            "bulk_out 02 011041414141414141 -> 9\nbulk_in 81 4096 -> 0160000000000000{mode:02x}\n"
        )
    }

    /// A download that fails at once: `capture()` returns false only
    /// through it.
    const NO_DOWNLOAD: &str = "bulk_out 02 071040 -> err device \"endpoint stalled\"\n";

    fn capture_once(acq: &mut Acquisition, mode: u8) -> bool {
        use std::sync::atomic::AtomicBool;
        let steps = format!("{}{NO_DOWNLOAD}", position_read(mode));
        let mut rec = Rec::default();
        acq.capture(&mut link(&steps), &mut rec, &AtomicBool::new(false))
    }

    #[test]
    fn the_capture_ends_as_sigma_capture_mode_says() {
        let hour = Some(Duration::from_secs(3600));
        let idle = TimeLimit::default();
        let trigger = Some(SigmaTrigger::default());
        // Nothing to end on.
        assert!(capture_once(&mut Acquisition::new(None, 16, 0, idle, false), 0));
        // The time limit, running and run out.
        let running = TimeLimit { limit: hour, start: Some(Instant::now()) };
        assert!(capture_once(&mut Acquisition::new(None, 16, 0, running, false), 0));
        let over = TimeLimit { limit: Some(Duration::ZERO), start: Some(Instant::now()) };
        std::thread::sleep(Duration::from_millis(2));
        assert!(!capture_once(&mut Acquisition::new(None, 16, 0, over, false), 0));
        // The device says it is complete.
        assert!(!capture_once(&mut Acquisition::new(None, 16, 0, idle, false), rmr::POSTTRIGGERED));
        // The memory wrapped: an end only without a trigger.
        assert!(!capture_once(&mut Acquisition::new(None, 16, 0, idle, false), rmr::ROUND));
        assert!(capture_once(&mut Acquisition::new(trigger, 16, 0, idle, false), rmr::ROUND));
    }

    #[test]
    fn the_time_limit_of_a_trigger_starts_when_it_is_seen() {
        let zero = TimeLimit { limit: Some(Duration::ZERO), start: None };
        let mut acq = Acquisition::new(Some(SigmaTrigger::default()), 16, 0, zero, true);
        // Not triggered: never expires, however short.
        assert!(capture_once(&mut acq, 0));
        assert!(acq.late_trigger_timeout && acq.acquire.start.is_none());
        // Seen: it starts, and this poll goes on.
        assert!(capture_once(&mut acq, rmr::TRIGGERED));
        assert!(!acq.late_trigger_timeout && acq.acquire.start.is_some());
        // The next poll finds it run out.
        std::thread::sleep(Duration::from_millis(2));
        assert!(!capture_once(&mut acq, rmr::TRIGGERED));
    }

    /// One burst of the suicide sequence, the PROG pulse and the purge.
    fn init_steps() -> String {
        format!(
            "{}bulk_out 02 01030301010101010101 -> 10\n\
             ctl_out 40 00 0001 0001 - -> ok\nctl_out 40 00 0002 0001 - -> ok\n",
            "bulk_out 02 8484888488848884 -> 8\n".repeat(4)
        )
    }

    const CABLE: &str = "ctl_out 40 0b 01df 0001 - -> ok\nctl_out 40 03 0000 0000 - -> ok\n";

    /// What follows the netlist: bit-bang off, purge, drain, the answer.
    const AFTER: &str = "ctl_out 40 0b 0000 0001 - -> ok\n\
        ctl_out 40 00 0001 0001 - -> ok\nctl_out 40 00 0002 0001 - -> ok\n\
        bulk_in 81 4096 -> 0160\n\
        bulk_out 02 0010400f102535400f102a3a4003102038 -> 17\n\
        bulk_in 81 4096 -> 0160a655aa\n";

    fn hex_of(data: &[u8]) -> String {
        use std::fmt::Write as _;
        data.iter().fold(String::new(), |mut s, b| {
            let _ = write!(s, "{b:02x}");
            s
        })
    }

    #[test]
    fn the_upload_goes_as_the_driver_sends_it() {
        // INIT absent for one whole attempt (10 reads with nothing), then
        // low in one sample and high in the next.
        let stream = bitbang_stream(&[0x12, 0x34]);
        let steps = format!(
            "{CABLE}{init}{idle}{init}bulk_in 81 4096 -> 01600020\nbulk_out 02 {netlist} -> 32\n{AFTER}",
            init = init_steps(),
            idle = "bulk_in 81 4096 -> 0160\n".repeat(10),
            netlist = hex_of(&stream),
        );
        let (mut link, replay) = linked(&steps);
        let mut running = None;
        link.upload_with(Firmware::Mhz50, &mut running, |name| {
            assert_eq!(name, "asix-sigma-50.fw");
            Ok(bitbang_stream(&[0x12, 0x34]))
        })
        .unwrap();
        assert_eq!(running, Some(Firmware::Mhz50));
        assert_eq!(replay.remaining(), 0);
    }

    #[test]
    fn the_upload_gives_up_after_ten_times_ten_reads() {
        let once = format!("{}{}", init_steps(), "bulk_in 81 4096 -> 0160\n".repeat(10));
        let steps = format!("{CABLE}{}", once.repeat(10));
        let (mut link, replay) = linked(&steps);
        let mut running = Some(Firmware::Mhz100);
        let e = link.upload_with(Firmware::Mhz50, &mut running, |_| unreachable!()).unwrap_err();
        assert!(matches!(e, Error::Timeout), "{e:?}");
        assert_eq!(running, None, "the old netlist is gone");
        assert_eq!(replay.remaining(), 0);
    }

    #[test]
    fn a_failed_suicide_write_still_sends_the_others() {
        let steps = format!(
            "{CABLE}bulk_out 02 8484888488848884 -> 8\n{}",
            "bulk_out 02 8484888488848884 -> err device \"endpoint stalled\"\n".repeat(3)
        );
        let (mut link, replay) = linked(&steps);
        let mut running = None;
        assert!(link.upload_with(Firmware::Mhz50, &mut running, |_| unreachable!()).is_err());
        assert_eq!(replay.remaining(), 0);
    }

    /// A row of memory row `r` for the wrap test: a count that goes on
    /// from row to row, timestamps with it.
    fn counting_row(r: u64) -> Vec<u8> {
        let mut out = Vec::with_capacity(ROW_LENGTH_BYTES);
        for c in 0..CLUSTERS_PER_ROW {
            let tick = (r * EVENTS_PER_ROW + c * EVENTS_PER_CLUSTER) as u16;
            out.extend_from_slice(&tick.to_le_bytes());
            for e in 0..EVENTS_PER_CLUSTER {
                out.extend_from_slice(&tick.wrapping_add(e as u16).to_le_bytes());
            }
        }
        out
    }

    /// Counts samples and checks that each continues the count.
    #[derive(Default)]
    struct Continuous {
        samples: u64,
        last: Option<u16>,
    }

    impl Feed for Continuous {
        fn logic(&mut self, data: &[u8]) {
            for c in data.as_chunks::<2>().0 {
                let v = u16::from_le_bytes(*c);
                if let Some(l) = self.last {
                    assert_eq!(v, l.wrapping_add(1), "after {} samples", self.samples);
                }
                self.last = Some(v);
                self.samples += 1;
            }
        }

        fn trigger(&mut self) {}
    }

    #[test]
    fn a_wrapped_memory_decodes_across_its_end() {
        // Stopped in row 1 after wrapping: rows 2 to 32767, then row 0.
        let mut d = Decoder::new(decrement_pos(1 << 9 | 7), u32::MAX, rmr::ROUND, 16, None, 0);
        let mut feed = Continuous::default();
        let mut fetches = Vec::new();
        while !d.done() {
            let (first, count) = d.next_fetch();
            fetches.push((first, count));
            let mut rows = Vec::with_capacity(count as usize * ROW_LENGTH_BYTES);
            for i in 0..count {
                rows.extend(counting_row((first + i) % ROW_COUNT));
            }
            d.decode(&rows, count, &mut feed);
            d.flush(&mut feed);
        }
        assert_eq!(fetches[0], (2, LINES_PER_READ));
        // The read that crosses the end of the memory, and is the last.
        assert_eq!(fetches.last(), Some(&(32738, 31)));
        assert_eq!(feed.samples, (ROW_COUNT - 1) * EVENTS_PER_ROW);
        assert_eq!(d.iter.line, 1);
    }

    /// One row: cluster 0 at tick 0, cluster 1 at tick 10, `words` the
    /// events of cluster 0 and the first of cluster 1.
    fn row_of(words: &[u16; 8]) -> Vec<u8> {
        let mut r = vec![0u8; ROW_LENGTH_BYTES];
        r[16..18].copy_from_slice(&10u16.to_le_bytes());
        for (i, w) in words[..7].iter().enumerate() {
            r[2 + 2 * i..4 + 2 * i].copy_from_slice(&w.to_le_bytes());
        }
        r[18..20].copy_from_slice(&words[7].to_le_bytes());
        r
    }

    #[test]
    fn events_hold_two_samples_at_100_mhz() {
        let mut words = [0u16; 8];
        words[1] = 0x0001; // Channel 1 high in the first sample.
        words[6] = 0xaaaa; // All 8 high in the second sample.
        words[7] = 0x5555;
        // Register 9, 8 events: 7 of cluster 0, 1 of cluster 1.
        let mut d = Decoder::new(decrement_pos(9), u32::MAX, 0, 8, None, 0);
        let mut rec = Rec::default();
        d.next_fetch();
        d.decode(&row_of(&words), 1, &mut rec);
        d.flush(&mut rec);
        let mut want = vec![0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff];
        // A gap of 3 ticks is 3 events: 6 samples of the last one.
        want.extend([0xff; 6]);
        want.extend([0xff, 0]);
        assert_eq!(rec.samples, want);
    }

    #[test]
    fn events_hold_four_samples_at_200_mhz() {
        let mut words = [0u16; 8];
        words[1] = 0x0002; // Channel 1 high in the second sample.
        words[6] = 0x8888; // All 4 high in the fourth sample.
        words[7] = 0x1111;
        let mut d = Decoder::new(decrement_pos(9), u32::MAX, 0, 4, None, 0);
        let mut rec = Rec::default();
        d.next_fetch();
        d.decode(&row_of(&words), 1, &mut rec);
        d.flush(&mut rec);
        let mut want = vec![0; 4];
        want.extend([0, 1, 0, 0]);
        want.extend([0; 16]);
        want.extend([0, 0, 0, 0xf]);
        // A gap of 3 ticks is 3 events: 12 samples of the last one.
        want.extend([0xf; 12]);
        want.extend([0xf, 0, 0, 0]);
        assert_eq!(rec.samples, want);
    }

    #[test]
    fn a_trigger_lands_on_its_sample_within_an_event() {
        let mut words = [0u16; 8];
        words[1] = 0x0002;
        let t = SigmaTrigger { rising: 1, ..SigmaTrigger::default() };
        let mut d = Decoder::new(decrement_pos(9), decrement_pos(3), 0, 4, Some(t), 0);
        let mut rec = Rec::default();
        d.next_fetch();
        d.decode(&row_of(&words), 1, &mut rec);
        d.flush(&mut rec);
        assert_eq!(rec.triggers, [5]);
    }

    #[test]
    fn the_netlist_scrambles_with_the_known_sequence() {
        use sha2::{Digest as _, Sha256};
        // The first bytes of the sequence, from the C lines compiled
        // (protocol.c:775-777).
        let mut x = [0u8; 8];
        unscramble(&mut x);
        assert_eq!(x, [0x3a, 0xf0, 0xf9, 0x6a, 0x25, 0xf5, 0xf1, 0xc7]);
        // What sigrok-cli sent of the 50 MHz stand-in: the last transfer of
        // tests/emu/asix-sigma/50-mhz.tx, by its digest.
        let mut file = Vec::new();
        for i in 0.. {
            if file.len() >= 2000 {
                break;
            }
            file.extend_from_slice(&Sha256::digest(format!("asix-sigma-50.fw:{i}").as_bytes()));
        }
        file.truncate(2000);
        let stream = bitbang_stream(&file);
        assert_eq!(stream.len(), 32000);
        assert_eq!(
            hex_of(&Sha256::digest(&stream[7 * 4096..])),
            "3733c9f1af7e51dbbc05c45615ab432b4ad109121b47c988ac2894b9a9e83a0a"
        );
    }

    #[test]
    fn the_running_netlist_is_not_uploaded_again() {
        // No request at all: the replay would refuse one.
        let (mut link, replay) = linked("");
        let mut running = Some(Firmware::Mhz50);
        link.upload_firmware(Firmware::Mhz50, &mut running).unwrap();
        assert_eq!(running, Some(Firmware::Mhz50));
        assert_eq!(replay.remaining(), 0);
    }

    #[test]
    fn deinterlacing() {
        assert_eq!(deinterlace_2x8(0b0101_0101_0101_0101, 0), 0xff);
        assert_eq!(deinterlace_2x8(0b0101_0101_0101_0101, 1), 0);
        assert_eq!(deinterlace_4x4(0x1111, 0), 0xf);
        assert_eq!(deinterlace_4x4(0x1111, 1), 0);
        assert_eq!(deinterlace_4x4(0x8888, 3), 0xf);
    }

    /// Feed that records what it got.
    #[derive(Default)]
    struct Rec {
        samples: Vec<u16>,
        triggers: Vec<usize>,
    }

    impl Feed for Rec {
        fn logic(&mut self, data: &[u8]) {
            self.samples.extend(data.as_chunks::<2>().0.iter().map(|c| u16::from_le_bytes(*c)));
        }

        fn trigger(&mut self) {
            self.triggers.push(self.samples.len());
        }
    }

    /// A row whose clusters are consecutive (timestamps 0, 7, 14, ...)
    /// with events counting from `first`.
    fn row(first: u16) -> Vec<u8> {
        let mut out = Vec::new();
        let mut v = first;
        for c in 0..64u16 {
            out.extend_from_slice(&(c * 7).to_le_bytes());
            for _ in 0..7 {
                out.extend_from_slice(&v.to_le_bytes());
                v += 1;
            }
        }
        out
    }

    #[test]
    fn the_last_row_counts_as_the_c_driver_does() {
        // Stop register 11, decremented 10: events 0 to 9 (protocol.c:2065).
        let mut d = Decoder::new(decrement_pos(11), u32::MAX, 0, 16, None, 0);
        assert_eq!(d.lines_total(), 1);
        let (line, count) = d.next_fetch();
        assert_eq!((line, count), (0, 1));
        let mut rec = Rec::default();
        d.decode(&row(0), 1, &mut rec);
        d.flush(&mut rec);
        assert_eq!(rec.samples, (0..10).collect::<Vec<_>>());
        assert!(d.done());
    }

    #[test]
    fn a_late_stop_register_stays_out_of_an_unwritten_cluster() {
        // Register 8 with only cluster 0 written: one more event would
        // take cluster 1's timestamp 0 for a gap of 65529 ticks.
        let mut r = row(0);
        r[16..].fill(0);
        let mut d = Decoder::new(decrement_pos(8), u32::MAX, 0, 16, None, 0);
        let mut rec = Rec::default();
        d.next_fetch();
        d.decode(&r, 1, &mut rec);
        d.flush(&mut rec);
        assert_eq!(rec.samples, (0..7).collect::<Vec<_>>());
    }

    #[test]
    fn the_sample_limit_holds_without_a_trigger() {
        let mut d = Decoder::new(decrement_pos(100), u32::MAX, 0, 16, None, 20);
        let mut rec = Rec::default();
        d.next_fetch();
        d.decode(&row(0), 1, &mut rec);
        d.flush(&mut rec);
        assert_eq!(rec.samples.len(), 20);
    }

    #[test]
    fn a_wrapped_memory_reads_all_of_its_last_row() {
        // Stopped in row 5 after wrapping: rows 6 to 32767, 0 to 4.
        let d = Decoder::new(decrement_pos(5 << 9 | 3), u32::MAX, rmr::ROUND, 16, None, 0);
        assert_eq!(d.lines_total(), ROW_COUNT - 1);
        assert_eq!(d.start.line, 6);
        assert_eq!((d.stop_line, d.stop_events), (4, EVENTS_PER_ROW));
    }

    #[test]
    fn a_gap_repeats_the_last_sample() {
        let mut r = vec![0u8; ROW_LENGTH_BYTES];
        // Cluster 0 at ts 0: 1..=7; cluster 1 at ts 10: 3 ticks later.
        for (c, ts) in [(0usize, 0u16), (1, 10)] {
            r[c * 16..c * 16 + 2].copy_from_slice(&ts.to_le_bytes());
            for e in 0..7 {
                let v = (c * 7 + e + 1) as u16;
                r[c * 16 + 2 + 2 * e..c * 16 + 4 + 2 * e].copy_from_slice(&v.to_le_bytes());
            }
        }
        let mut d = Decoder::new(decrement_pos(15), u32::MAX, 0, 16, None, 0);
        let mut rec = Rec::default();
        d.next_fetch();
        d.decode(&r, 1, &mut rec);
        d.flush(&mut rec);
        let mut want: Vec<u16> = (1..=7).collect();
        want.extend([7, 7, 7]);
        want.extend(8..=14);
        assert_eq!(rec.samples, want);
    }

    #[test]
    fn a_trigger_near_the_start_gets_its_marker() {
        // Channel 1 rising: the count 0, 1, ... rises on bit 0 at 1.
        let t = SigmaTrigger { rising: 1, ..SigmaTrigger::default() };
        // The hardware's position a little late, at event 3.
        let mut d = Decoder::new(decrement_pos(100), decrement_pos(4), 0, 16, Some(t), 0);
        let mut rec = Rec::default();
        d.next_fetch();
        d.decode(&row(0), 1, &mut rec);
        d.flush(&mut rec);
        assert_eq!(rec.triggers, [1]);
        // With a trigger the whole region goes out; no limit is set here.
        assert_eq!(rec.samples.len(), 99);
    }

    #[test]
    fn a_trigger_further_in_is_found_by_software() {
        let t = SigmaTrigger { simple_mask: 0xffff, simple_value: 60, ..SigmaTrigger::default() };
        let mut d = Decoder::new(decrement_pos(200), decrement_pos(66), 0, 16, Some(t), 0);
        let mut rec = Rec::default();
        d.next_fetch();
        d.decode(&row(0), 1, &mut rec);
        d.flush(&mut rec);
        assert_eq!(rec.triggers, [60]);
    }

    #[test]
    fn without_a_match_the_marker_is_at_the_hardware_position() {
        let t =
            SigmaTrigger { simple_mask: 0xffff, simple_value: 0x7fff, ..SigmaTrigger::default() };
        let mut d = Decoder::new(decrement_pos(200), decrement_pos(66), 0, 16, Some(t), 0);
        let mut rec = Rec::default();
        d.next_fetch();
        d.decode(&row(0), 1, &mut rec);
        d.flush(&mut rec);
        assert_eq!(rec.triggers, [65]);
    }
}
