// Counterpart in sigrok:
//   libsigrok/src/hardware/asix-sigma/api.c (the clock select, trigger select 2 and post trigger values of dev_acquisition_start),
//   libsigrok/src/hardware/asix-sigma/protocol.c (the FPGA commands, registers, trigger table and sample memory layout),
//   libsigrok/src/hardware/asix-sigma/protocol.h,
//   Copyright (C) 2010-2012 Håvard Espeland <gus@ping.uio.no>,,
//   Copyright (C) 2010 Martin Stensgård <mastensg@ping.uio.no>,
//   Copyright (C) 2010 Carl Henrik Lunde <chlunde@ping.uio.no>,
//   Copyright (C) 2020 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Niklas Thorne, jry,
//   Daniel Trnka
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The SIGMA's FPGA once a logic analyzer netlist runs in it: the
//! command bytes the host writes through the FTDI FIFO, the register
//! file, the capture into the sample memory and its readout.
//!
//! Commands are one byte, the opcode in the high nibble
//! (`protocol.h`): `0x0n` and `0x1n` set the low and high nibble of the
//! register address, `0x2n` holds a data low nibble that `0x3n`
//! completes and writes, `0x4n` reads the addressed register (bit 0
//! moves the address on, bit 1 down rather than up, within its 16
//! address page), `0x50` waits for a memory row fetch, `0x60`/`0x70`
//! fetch the row `WRITE_MEMROW` points at into buffer 0 or 1 and move
//! on to the next row, `0xa0`/`0xb0` send buffer 0 or 1 (1024 bytes).
//! The data bytes of one register write all go to the same register:
//! several registers take more than one byte (the clock select 4 in the
//! 50 MHz netlist, the trigger select 2 or 6, the memory row 2, the
//! trigger options 2).
//!
//! The sample memory is 32768 rows of 64 clusters, a cluster a 16 bit
//! timestamp and 7 events of 16 bits (`protocol.h`). Positions count
//! events: the row in bits 9 and up, the event within the row (0 to
//! 447) below. An event is one sample of 16 channels at up to 50 MHz,
//! two samples of 8 channels in the 100 MHz netlist, four of 4 in the
//! 200 MHz one, their bits interleaved (`sigma_deinterlace_data_2x8`,
//! `_4x4`); a tick is one event's time, the sample period or 20 ns.
//!
//! What the model makes of what `libsigrok` does not say, each its
//! own choice:
//!
//! - The pins carry a count: sample `n` (since the capture started) is
//!   `n / step` (option `step=`, 2 by default), cut to the netlist's
//!   channels, so channel 1 toggles every `step` samples.
//! - Run length compression: the ticks fall into slots of 7, one
//!   cluster's worth, and a slot is stored only when one of its samples
//!   differs from the sample before it, when it is the first, when it
//!   holds the trigger, or when the 16 bit timestamp has wrapped since
//!   the last stored slot ("a cluster is enforced for each 64K ticks",
//!   `protocol.h`). Each stored slot is a whole cluster.
//! - Time: a capture's time runs only when the host polls the
//!   positions, which `sigma_capture_mode` does on a 10 ms timer: each
//!   read of `READ_TRIGGER_POS_LOW` during a capture is 10 ms of
//!   sampling (the sample rate's ticks, 500000 ticks of 20 ns above
//!   50 MHz and with an external clock, which the model takes as 50
//!   MHz as the driver does). An emulation artifact: a real capture
//!   runs on its own between the polls.
//! - The position registers point after the last event written
//!   (`READ_STOP_POS`) and after the trigger event (`READ_TRIGGER_POS`),
//!   as `sigma_read_pos`'s decrement implies; the trigger pipeline's
//!   latency is not modelled.
//! - `WRITE_POST_TRIGGER` (`capture_ratio * 255 / 100`): the model
//!   takes it as the share of the memory before the trigger, in 256ths,
//!   and stops the capture (`RMR_POSTTRIGGERED`) once `256 - value`
//!   256ths of the memory hold events after the trigger event. Its
//!   unit is not documented in libsigrok.
//! - The trigger unit of the 50 MHz netlist is read from the lookup
//!   table the driver writes (`sigma_write_trigger_lut`): `M2D` is the
//!   level pattern, one 16 bit table per 4 channels; `M0D` and `M1D`
//!   select up to two edge channels; `M3Q`, indexed by both selections
//!   now and one sample before, combines them; level and edge must both
//!   hold. `M4` and `M3S` and the parameter word (counters, prescaler)
//!   are taken only in the form the driver writes them (`M4` 0xa000,
//!   `M3S` 0, `SELRES` "never") and not interpreted. The table is
//!   evaluated at every sample, not on the 50 MHz check interval the
//!   driver's comment describes. In the 100 and 200 MHz netlists the
//!   trigger is one channel's edge, from `WRITE_TRIGGER_SELECT2`'s
//!   final value: the channel in bits 0 to 2, bit 3 for a falling edge
//!   as the driver uses it (its comment says the documentation calls
//!   the bit rising).
//! - `WMR_FORCESTOP` stores one more cluster, the slot at the stop
//!   tick whether its pins changed or not (the hardware "stores
//!   clusters regardless of pin changes" then, `download_capture`'s
//!   comment), and raises `RMR_POSTTRIGGERED` after `stop-polls=N`
//!   reads of the mode register (0 by default).
//! - The external clock byte is taken as the driver writes it: the pin
//!   field (bits 0 to 3) is the channel index plus 1, so channel 16
//!   writes 16, which spills into `CLKSEL_RISING` and leaves the pin
//!   field 0 (`api.c`, `div = clock_pin + 1` against `CLKSEL_PINMASK`).
//!   A known habit of the driver, not flagged; the model's external
//!   clock ignores the pin anyway.
//! - Memory beyond what a capture wrote reads 0.

use std::collections::VecDeque;

use crate::model::Violations;

/// Rows of the sample memory.
pub const ROW_COUNT: u64 = 32768;
/// Bytes per row.
pub const ROW_BYTES: usize = 1024;
/// Clusters per row.
pub const CLUSTERS_PER_ROW: u64 = 64;
/// Events per cluster.
pub const EVENTS_PER_CLUSTER: u64 = 7;
/// Events per row.
pub const EVENTS_PER_ROW: u64 = CLUSTERS_PER_ROW * EVENTS_PER_CLUSTER;
/// Events the memory holds.
pub const MEMORY_EVENTS: u64 = ROW_COUNT * EVENTS_PER_ROW;
/// What `READ_ID` answers.
pub const ID: u8 = 0xa6;
/// Ticks a capture runs per poll at 50 MHz: the driver's 10 ms timer.
pub const TICKS_50MHZ: u64 = 500_000;

/// Write registers (`enum sigma_write_register`).
pub mod wreg {
    /// Clock select.
    pub const CLOCK_SELECT: u8 = 0;
    /// Trigger select: a lookup table word or the parameters.
    pub const TRIGGER_SELECT: u8 = 1;
    /// Trigger select 2: programming mode, table strobes, LEDs.
    pub const TRIGGER_SELECT2: u8 = 2;
    /// Mode.
    pub const MODE: u8 = 3;
    /// Memory row of the next fetch.
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
    /// Trigger position, 24 bits from here.
    pub const TRIGGER_POS_LOW: u8 = 1;
    /// Stop position, 24 bits from here.
    pub const STOP_POS_LOW: u8 = 4;
    /// Mode and status.
    pub const MODE: u8 = 7;
    /// The scratch register.
    pub const TEST: u8 = 15;
}

/// `WRITE_MODE` bits.
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

/// `READ_MODE` bits.
pub mod rmr {
    /// Writing samples.
    pub const SDRAMWRITEEN: u8 = 1 << 0;
    /// Reading the memory.
    pub const SDRAMREADEN: u8 = 1 << 1;
    /// The trigger is enabled.
    pub const TRGEN: u8 = 1 << 3;
    /// The write position has wrapped.
    pub const ROUND: u8 = 1 << 4;
    /// The trigger matched.
    pub const TRIGGERED: u8 = 1 << 5;
    /// The capture is over.
    pub const POSTTRIGGERED: u8 = 1 << 6;
}

/// `WRITE_TRIGGER_SELECT2` bits.
pub mod trgsel2 {
    /// The 100/200 MHz trigger channel.
    pub const PINS_MASK: u8 = 0x07;
    /// The 100/200 MHz edge: set for falling, as the driver uses it.
    pub const PINPOL: u8 = 1 << 3;
    /// The lookup table address of a strobe.
    pub const LUT_ADDR_MASK: u8 = 0x0f;
    /// Strobe the trigger select word into the table.
    pub const LUT_WRITE: u8 = 1 << 4;
    /// Trigger programming mode.
    pub const RESET: u8 = 1 << 5;
}

/// The netlist the FPGA was configured with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Netlist {
    /// `asix-sigma-50.fw`: 50 MHz divided by 1 to 256, or an external clock.
    Mhz50,
    /// `asix-sigma-100.fw`.
    Mhz100,
    /// `asix-sigma-200.fw`.
    Mhz200,
}

impl Netlist {
    /// Samples per event.
    pub fn samples_per_event(self) -> u64 {
        match self {
            Netlist::Mhz50 => 1,
            Netlist::Mhz100 => 2,
            Netlist::Mhz200 => 4,
        }
    }

    /// Channels sampled.
    pub fn channels(self) -> u32 {
        16 / self.samples_per_event() as u32
    }
}

/// What the FPGA can do wrong (the model's options).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FpgaFaults {
    /// What `READ_ID` answers.
    pub id: u8,
    /// Bits of the scratch register that read 1 whatever was written.
    pub scratch_stuck: u8,
    /// Mode register reads after `WMR_FORCESTOP` before
    /// `RMR_POSTTRIGGERED`.
    pub stop_polls: u32,
}

impl Default for FpgaFaults {
    fn default() -> FpgaFaults {
        FpgaFaults { id: ID, scratch_stuck: 0, stop_polls: 0 }
    }
}

/// The sample clock, from `WRITE_CLOCK_SELECT`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Clock {
    /// 50 MHz divided by the value plus 1.
    Divided(u8),
    /// An external clock (pin field and edge bits as written).
    External(u8),
    /// The 100 or 200 MHz netlist's own.
    Fixed,
}

/// The trigger of a capture.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Trigger {
    /// The 50 MHz netlist's lookup table: `M0D`, `M1D`, `M2D` per
    /// quad of channels, and `M3Q`.
    Lut { m0d: [u16; 4], m1d: [u16; 4], m2d: [u16; 4], m3q: u16 },
    /// One channel's edge (100 and 200 MHz).
    Edge { pin: u8, falling: bool },
}

impl Trigger {
    fn matches(&self, prev: u16, now: u16) -> bool {
        match *self {
            Trigger::Lut { m0d, m1d, m2d, m3q } => {
                let table = |t: &[u16; 4], s: u16| {
                    (0..4).all(|q| t[q] >> ((s >> (4 * q)) & 0xf) & 1 == 1)
                };
                let level = table(&m2d, now);
                let index = usize::from(table(&m0d, now))
                    | usize::from(table(&m0d, prev)) << 1
                    | usize::from(table(&m1d, now)) << 2
                    | usize::from(table(&m1d, prev)) << 3;
                level && m3q >> index & 1 == 1
            }
            Trigger::Edge { pin, falling } => {
                let (p, n) = (prev >> pin & 1, now >> pin & 1);
                if falling { p == 1 && n == 0 } else { p == 0 && n == 1 }
            }
        }
    }
}

/// One capture: the stored slots and where it stands.
#[derive(Clone, Debug)]
struct Capture {
    netlist: Netlist,
    step: u64,
    /// Channels the clock select disabled read 0.
    enabled: u16,
    /// Ticks per poll.
    per_poll: u64,
    trigger: Option<Trigger>,
    /// Events after the trigger event before the capture stops.
    post_events: u64,
    /// Sampling.
    running: bool,
    /// Ticks elapsed.
    now: u64,
    /// Ticks looked at so far, whole slots.
    scanned: u64,
    /// The first tick of each stored slot.
    slots: Vec<u64>,
    /// The sample before tick `scanned`.
    last: u16,
    /// The trigger event's index.
    trig: Option<u64>,
    /// Mode register reads left before `RMR_POSTTRIGGERED` after a
    /// forced stop.
    stop_left: Option<u32>,
    /// `RMR_POSTTRIGGERED`.
    over: bool,
}

impl Capture {
    fn sample(&self, n: u64) -> u16 {
        let mask = if self.netlist.channels() == 16 {
            0xffff
        } else {
            (1u16 << self.netlist.channels()) - 1
        };
        (n / self.step) as u16 & mask & self.enabled
    }

    /// The event word of tick `t`: its samples, interleaved.
    fn word(&self, t: u64) -> u16 {
        let spe = self.netlist.samples_per_event();
        if spe == 1 {
            return self.sample(t);
        }
        let channels = self.netlist.channels();
        let mut w = 0u16;
        for k in 0..spe {
            let s = self.sample(t * spe + k);
            for ch in 0..channels {
                w |= (s >> ch & 1) << (u64::from(ch) * spe + k);
            }
        }
        w
    }

    fn events(&self) -> u64 {
        self.slots.len() as u64 * EVENTS_PER_CLUSTER
    }

    /// Samples ticks up to `until`, slot by slot, as long as it runs.
    fn advance(&mut self, until: u64) {
        let spe = self.netlist.samples_per_event();
        while self.running && self.scanned < until {
            let start = self.scanned;
            let first = self.slots.is_empty() && start == 0;
            let mut prev = if first { self.sample(0) } else { self.last };
            let mut changed = false;
            let mut trig_at = None;
            for i in 0..EVENTS_PER_CLUSTER * spe {
                let s = self.sample(start * spe + i);
                changed |= s != prev;
                if trig_at.is_none()
                    && self.trig.is_none()
                    && self.trigger.as_ref().is_some_and(|t| t.matches(prev, s))
                {
                    trig_at = Some(i / spe);
                }
                prev = s;
            }
            self.last = prev;
            let wrapped = self.slots.last().is_some_and(|&l| l >> 16 != start >> 16);
            if first || changed || wrapped || trig_at.is_some() {
                if let Some(t) = trig_at {
                    self.trig = Some(self.events() + t);
                }
                self.slots.push(start);
            }
            self.scanned += EVENTS_PER_CLUSTER;
            if let Some(t) = self.trig
                && self.events() > t + self.post_events
            {
                self.running = false;
                self.over = true;
            }
        }
    }

    /// `WMR_FORCESTOP`: the slot at the stop tick is stored whatever it
    /// holds, and the capture ends.
    fn force_stop(&mut self) {
        self.running = false;
        let spe = self.netlist.samples_per_event();
        let start = self.scanned;
        self.slots.push(start);
        self.last = self.sample((start + EVENTS_PER_CLUSTER) * spe - 1);
        self.scanned += EVENTS_PER_CLUSTER;
    }

    /// A poll: the next 10 ms.
    fn poll(&mut self) {
        if self.running {
            self.now += self.per_poll;
            self.advance(self.now);
        }
    }

    /// The register value of event position `e`: the row in bits 9
    /// and up, the event in the row below.
    fn position(e: u64) -> u32 {
        let row = e / EVENTS_PER_ROW % ROW_COUNT;
        ((row << 9) | (e % EVENTS_PER_ROW)) as u32
    }

    /// Row `r` of the memory as the capture left it.
    fn row(&self, r: u64) -> Vec<u8> {
        let total = self.slots.len() as u64;
        let per_memory = ROW_COUNT * CLUSTERS_PER_ROW;
        let mut out = vec![0u8; ROW_BYTES];
        for c in 0..CLUSTERS_PER_ROW {
            let base = r * CLUSTERS_PER_ROW + c;
            if base >= total {
                continue;
            }
            // The latest pass over the memory.
            let g = base + (total - 1 - base) / per_memory * per_memory;
            let start = self.slots[g as usize];
            let at = c as usize * 16;
            out[at..at + 2].copy_from_slice(&(start as u16).to_le_bytes());
            for i in 0..EVENTS_PER_CLUSTER {
                let w = self.word(start + i);
                let o = at + 2 + 2 * i as usize;
                out[o..o + 2].copy_from_slice(&w.to_le_bytes());
            }
        }
        out
    }
}

/// One of the two row buffers.
#[derive(Clone, Debug, Default)]
struct Bank {
    data: Option<Vec<u8>>,
    sent: bool,
}

/// The FPGA with a logic analyzer netlist.
#[derive(Clone, Debug)]
pub struct Fpga {
    netlist: Netlist,
    faults: FpgaFaults,
    step: u64,
    addr: u8,
    data_low: Option<u8>,
    /// The register a write goes to and the bytes it has had.
    write: Option<(u8, Vec<u8>)>,
    test: u8,
    sdram_ready: bool,
    clock: Option<(Clock, u16)>,
    trgsel2: u8,
    /// The last two byte `WRITE_TRIGGER_SELECT`, for the next strobe.
    lut_word: Option<u16>,
    lut: [Option<u16>; 16],
    params: Option<Vec<u8>>,
    post_trigger: Option<u8>,
    mode: u8,
    memrow: Option<u64>,
    banks: [Bank; 2],
    /// The bank whose fetch was not waited for.
    pending: Option<usize>,
    capture: Option<Capture>,
    readen: bool,
    out: VecDeque<u8>,
    first_poll: FirstPoll,
}

/// The first poll of a capture, the point `arm unplug model` waits for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FirstPoll {
    /// Not yet.
    Waiting,
    /// Answered, not yet taken by the server.
    Marked,
    /// Taken.
    Done,
}

impl Fpga {
    /// The FPGA just configured with `netlist`.
    pub fn new(netlist: Netlist, faults: FpgaFaults, step: u64) -> Fpga {
        Fpga {
            netlist,
            faults,
            step: step.max(1),
            addr: 0,
            data_low: None,
            write: None,
            test: 0,
            sdram_ready: false,
            clock: None,
            trgsel2: 0,
            lut_word: None,
            lut: [None; 16],
            params: None,
            post_trigger: None,
            mode: 0,
            memrow: None,
            banks: Default::default(),
            pending: None,
            capture: None,
            readen: false,
            out: VecDeque::new(),
            first_poll: FirstPoll::Waiting,
        }
    }

    /// The netlist.
    pub fn netlist(&self) -> Netlist {
        self.netlist
    }

    /// Bytes for the host, up to `max`.
    pub fn take(&mut self, max: usize) -> Vec<u8> {
        let n = max.min(self.out.len());
        self.out.drain(..n).collect()
    }

    /// Whether bytes wait for the host.
    pub fn has_output(&self) -> bool {
        !self.out.is_empty()
    }

    /// The FTDI chip dropped what it held for the host.
    pub fn purge(&mut self) {
        self.out.clear();
    }

    /// Whether the first poll of a capture was answered since asked.
    pub fn take_event_point(&mut self) -> bool {
        let marked = self.first_poll == FirstPoll::Marked;
        if marked {
            self.first_poll = FirstPoll::Done;
        }
        marked
    }

    /// The bytes of one OUT transfer, each a command.
    pub fn commands(&mut self, data: &[u8], v: &mut Violations) {
        for &b in data {
            self.command(b, v);
        }
        // A register write ends with its transfer at the latest.
        self.end_write(v);
        if self.data_low.take().is_some() {
            v.report("a data low nibble (0x2n) at the end of a transfer, never completed");
        }
    }

    fn command(&mut self, b: u8, v: &mut Violations) {
        let (op, lo) = (b >> 4, b & 0x0f);
        if op != 0x2 && op != 0x3 {
            self.end_write(v);
            if self.data_low.take().is_some() {
                v.report(format!("command {b:#04x} after a data low nibble without its high one"));
            }
        }
        match op {
            0x0 => self.addr = (self.addr & 0xf0) | lo,
            0x1 => self.addr = (self.addr & 0x0f) | (lo << 4),
            0x2 => {
                if self.data_low.replace(lo).is_some() {
                    v.report("two data low nibbles (0x2n) in a row");
                }
            }
            0x3 => match self.data_low.take() {
                Some(low) => self.write_byte((lo << 4) | low, v),
                None => v.report(format!("data high nibble {b:#04x} without a low one (0x2n)")),
            },
            0x4 => self.read(lo, v),
            0x5 => {
                Self::no_operand(b, v);
                if self.pending.take().is_none() {
                    v.report("REG_DRAM_WAIT_ACK with no row fetch to wait for");
                }
            }
            0x6 | 0x7 => {
                Self::no_operand(b, v);
                self.fetch_row(usize::from(op & 1), v);
            }
            0xa | 0xb => {
                Self::no_operand(b, v);
                self.send_bank(usize::from(op & 1), v);
            }
            0x8 | 0x9 => v.report(format!(
                "REG_DRAM_BLOCK_BEGIN {b:#04x}: the driver reads whole rows (0xa0/0xb0)"
            )),
            _ => v.report(format!("command {b:#04x}: no such opcode")),
        }
    }

    fn no_operand(b: u8, v: &mut Violations) {
        if b & 0x0f != 0 {
            v.report(format!("command {b:#04x}: its low nibble is 0"));
        }
    }

    fn read(&mut self, adjust: u8, v: &mut Violations) {
        if !matches!(adjust, 0 | 1) {
            v.report(format!(
                "REG_READ_ADDR with adjust bits {adjust:#x}: the driver reads in place (0x40) or \
                 moving up (0x41)"
            ));
        }
        let reg = self.addr;
        let value = self.register(reg, v);
        self.out.push_back(value);
        if adjust & 1 != 0 {
            let next = if adjust & 2 != 0 { reg.wrapping_sub(1) } else { reg.wrapping_add(1) };
            self.addr = (reg & 0xf0) | (next & 0x0f);
        }
    }

    fn register(&mut self, reg: u8, v: &mut Violations) -> u8 {
        let pos = |c: &Option<Capture>, f: &dyn Fn(&Capture) -> u32| c.as_ref().map_or(0, f);
        match reg {
            rreg::ID => self.faults.id,
            1..=6 => {
                if reg == rreg::TRIGGER_POS_LOW
                    && let Some(c) = self.capture.as_mut()
                    && c.running
                {
                    c.poll();
                    if self.first_poll == FirstPoll::Waiting {
                        self.first_poll = FirstPoll::Marked;
                    }
                }
                let trig = pos(&self.capture, &|c| c.trig.map_or(0, |t| Capture::position(t + 1)));
                let stop = pos(&self.capture, &|c| Capture::position(c.events()));
                let (value, shift) = if reg < rreg::STOP_POS_LOW {
                    (trig, (reg - 1) * 8)
                } else {
                    (stop, (reg - 4) * 8)
                };
                (value >> shift) as u8
            }
            rreg::MODE => self.read_mode(),
            rreg::TEST => self.test | self.faults.scratch_stuck,
            _ => {
                v.report(format!(
                    "read of register {reg}: the driver reads the id (0), positions and mode (1 \
                     to 7) and the scratch register (15)"
                ));
                0xff
            }
        }
    }

    fn read_mode(&mut self) -> u8 {
        let mut m = 0;
        if self.readen {
            m |= rmr::SDRAMREADEN;
        }
        if self.mode & wmr::TRGEN != 0 {
            m |= rmr::TRGEN;
        }
        if let Some(c) = self.capture.as_mut() {
            if let Some(left) = c.stop_left {
                if left == 0 {
                    c.over = true;
                    c.stop_left = None;
                } else {
                    c.stop_left = Some(left - 1);
                }
            }
            if c.running || (c.stop_left.is_some() && !c.over) {
                m |= rmr::SDRAMWRITEEN;
            }
            if c.events() >= MEMORY_EVENTS {
                m |= rmr::ROUND;
            }
            if c.trig.is_some() {
                m |= rmr::TRIGGERED;
            }
            if c.over {
                m |= rmr::POSTTRIGGERED;
            }
        }
        m
    }

    fn capturing(&self) -> bool {
        self.capture.as_ref().is_some_and(|c| c.running)
    }

    fn write_byte(&mut self, byte: u8, v: &mut Violations) {
        let reg = self.addr;
        let n = match &mut self.write {
            Some((r, bytes)) if *r == reg => {
                bytes.push(byte);
                bytes.len()
            }
            _ => {
                self.end_write(v);
                self.write = Some((reg, vec![byte]));
                1
            }
        };
        if self.capturing() && reg != wreg::MODE {
            v.report(format!("write to register {reg} during a capture"));
        }
        match reg {
            wreg::TEST if n == 1 => self.test = byte,
            wreg::MODE if n == 1 => self.write_mode(byte, v),
            wreg::TRIGGER_SELECT2 if n == 1 => self.write_trgsel2(byte, v),
            wreg::POST_TRIGGER if n == 1 => self.post_trigger = Some(byte),
            _ => {}
        }
    }

    /// The register write in progress is over: check its length and
    /// take the multi-byte values.
    fn end_write(&mut self, v: &mut Violations) {
        let Some((reg, bytes)) = self.write.take() else { return };
        let len = bytes.len();
        let want: &[usize] = match reg {
            wreg::CLOCK_SELECT if self.netlist == Netlist::Mhz50 => &[4],
            wreg::TRIGGER_SELECT => &[2, 6],
            wreg::MEMROW | wreg::TRIGGER_OPTION => &[2],
            wreg::CLOCK_SELECT
            | wreg::TRIGGER_SELECT2
            | wreg::MODE
            | wreg::POST_TRIGGER
            | wreg::TEST => &[1],
            _ => {
                v.report(format!(
                    "write of {len} bytes to register {reg}: the driver writes registers 0 to 6 \
                     and 15"
                ));
                return;
            }
        };
        if !want.contains(&len) {
            v.report(format!(
                "write of {len} bytes to register {reg}; it takes {want:?} in the {:?} netlist",
                self.netlist
            ));
            return;
        }
        match reg {
            wreg::CLOCK_SELECT => self.write_clock(&bytes, v),
            wreg::TRIGGER_SELECT => {
                if self.netlist != Netlist::Mhz50 {
                    v.report(
                        "WRITE_TRIGGER_SELECT in the 100/200 MHz netlist, whose trigger is one \
                         channel's edge in WRITE_TRIGGER_SELECT2",
                    );
                }
                if self.trgsel2 & trgsel2::RESET == 0 {
                    v.report("WRITE_TRIGGER_SELECT outside trigger programming mode");
                }
                if len == 2 {
                    self.lut_word = Some(u16::from_be_bytes([bytes[0], bytes[1]]));
                } else {
                    // The parameter word, then CMPB and CMPA: what
                    // sigma_build_basic_trigger sets with and without a
                    // trigger.
                    if bytes != [0, 0x0c, 0, 0, 0, 0] && bytes != [0; 6] {
                        v.report(format!(
                            "trigger parameters {bytes:02x?}: the model knows SELRES never \
                             (00 0c 00 00 00 00) and all zero, not counters or prescalers"
                        ));
                    }
                    self.params = Some(bytes);
                }
            }
            wreg::MEMROW => {
                let row = u64::from(u16::from_be_bytes([bytes[0], bytes[1]]));
                if row >= ROW_COUNT {
                    v.report(format!("WRITE_MEMROW {row}: the memory has {ROW_COUNT} rows"));
                }
                self.memrow = Some(row % ROW_COUNT);
            }
            // TRGOPT_TRGOOUTEN, then TRGOPT_TRGOEN: the trigger out pin
            // follows the trigger. The pin is not emulated.
            wreg::TRIGGER_OPTION if bytes != [0x08, 0x40] => {
                v.report(format!(
                    "trigger options {bytes:02x?}: the model knows the driver's 08 40 only"
                ));
            }
            _ => {}
        }
    }

    fn write_clock(&mut self, bytes: &[u8], v: &mut Violations) {
        let clock = if self.netlist == Netlist::Mhz50 {
            let (mode, div) = (bytes[0], bytes[1]);
            let disabled = u16::from_be_bytes([bytes[2], bytes[3]]);
            match mode {
                0 => (Clock::Divided(div), disabled),
                1 => {
                    // CLKSEL_PINMASK, CLKSEL_RISING, CLKSEL_FALLING.
                    if div & 0xc0 != 0 || div & 0x30 == 0 {
                        v.report(format!(
                            "external clock byte {div:#04x}: a pin in bits 0 to 3, rising (bit 4) \
                             and/or falling (bit 5)"
                        ));
                    }
                    (Clock::External(div), disabled)
                }
                _ => {
                    v.report(format!("clock select mode {mode:#04x}: 0 (divided) or 1 (CLKSEL8)"));
                    return;
                }
            }
        } else {
            (Clock::Fixed, u16::from(bytes[0]))
        };
        self.clock = Some(clock);
    }

    fn write_trgsel2(&mut self, value: u8, v: &mut Violations) {
        if value & trgsel2::LUT_WRITE != 0 {
            if value & trgsel2::RESET == 0 {
                v.report(format!(
                    "WRITE_TRIGGER_SELECT2 {value:#04x}: a table strobe outside programming mode"
                ));
            }
            if self.netlist != Netlist::Mhz50 {
                v.report("trigger table strobe in the 100/200 MHz netlist");
            }
            match self.lut_word.take() {
                Some(w) => self.lut[usize::from(value & trgsel2::LUT_ADDR_MASK)] = Some(w),
                None => v.report(format!(
                    "trigger table strobe {value:#04x} without a two byte WRITE_TRIGGER_SELECT \
                     before it"
                )),
            }
        } else if value & trgsel2::RESET != 0 && self.trgsel2 & trgsel2::RESET == 0 {
            // Entering programming mode: the table starts over.
            self.lut = [None; 16];
            self.params = None;
        }
        self.trgsel2 = value;
    }

    fn write_mode(&mut self, value: u8, v: &mut Violations) {
        use wmr::{FORCESTOP, SDRAMINIT, SDRAMREADEN, SDRAMWRITEEN, TRGEN, TRGRES};
        match value {
            SDRAMINIT => {
                if self.capturing() {
                    v.report("WMR_SDRAMINIT during a capture");
                }
                self.sdram_ready = true;
                self.capture = None;
                self.readen = false;
            }
            _ if value & !TRGEN == TRGRES | SDRAMWRITEEN => self.start(value & TRGEN != 0, v),
            _ if value == FORCESTOP | SDRAMWRITEEN => match self.capture.as_mut() {
                Some(c) if c.running => {
                    c.force_stop();
                    c.stop_left = Some(self.faults.stop_polls);
                }
                _ => v.report("WMR_FORCESTOP with no capture running"),
            },
            SDRAMREADEN => {
                match &self.capture {
                    Some(c) if c.over => {}
                    Some(_) => v.report("WMR_SDRAMREADEN before the capture stopped"),
                    None => v.report("WMR_SDRAMREADEN with nothing captured"),
                }
                self.readen = true;
            }
            _ => {
                v.report(format!("WRITE_MODE {value:#04x}: not a mode the driver sets"));
                return;
            }
        }
        self.mode = value;
    }

    fn start(&mut self, trgen: bool, v: &mut Violations) {
        if !self.sdram_ready {
            v.report("capture started before WMR_SDRAMINIT");
        }
        if self.capturing() {
            v.report("capture started during a capture");
        }
        if self.trgsel2 & trgsel2::RESET != 0 {
            v.report("capture started in trigger programming mode");
        }
        let Some((clock, disabled)) = self.clock else {
            v.report("capture started before WRITE_CLOCK_SELECT");
            return;
        };
        let per_poll = match clock {
            Clock::Divided(div) => TICKS_50MHZ / (u64::from(div) + 1),
            Clock::External(_) | Clock::Fixed => TICKS_50MHZ,
        };
        let trigger = if trgen {
            if self.post_trigger.is_none() {
                v.report("trigger enabled before WRITE_POST_TRIGGER");
            }
            if self.netlist == Netlist::Mhz50 {
                self.lut_trigger(v)
            } else {
                Some(Trigger::Edge {
                    pin: self.trgsel2 & trgsel2::PINS_MASK,
                    falling: self.trgsel2 & trgsel2::PINPOL != 0,
                })
            }
        } else {
            None
        };
        let pre = u64::from(self.post_trigger.unwrap_or(0));
        let enabled = if self.netlist == Netlist::Mhz50 { !disabled } else { !disabled & 0xff };
        self.readen = false;
        self.first_poll = FirstPoll::Waiting;
        self.banks = Default::default();
        self.pending = None;
        let mut c = Capture {
            netlist: self.netlist,
            step: self.step,
            enabled,
            per_poll,
            trigger,
            post_events: (256 - pre) * MEMORY_EVENTS / 256,
            running: true,
            now: 0,
            scanned: 0,
            slots: Vec::new(),
            last: 0,
            trig: None,
            stop_left: None,
            over: false,
        };
        c.last = c.sample(0);
        self.capture = Some(c);
    }

    fn lut_trigger(&self, v: &mut Violations) -> Option<Trigger> {
        if self.params.is_none() {
            v.report("trigger enabled without the trigger parameters (6 byte WRITE_TRIGGER_SELECT)");
        }
        let missing: Vec<usize> = (0..16).filter(|&a| self.lut[a].is_none()).collect();
        if !missing.is_empty() {
            v.report(format!("trigger enabled with table addresses {missing:?} not written"));
            return None;
        }
        let words: Vec<u16> = self.lut.iter().map(|w| w.unwrap_or(0)).collect();
        let bits = |shift: u32| -> u16 {
            (0..16).fold(0u16, |acc, a| acc | ((words[a] >> shift & 1) << a))
        };
        let quads = |base: u32| -> [u16; 4] { [bits(base), bits(base + 1), bits(base + 2), bits(base + 3)] };
        let (m0d, m1d, m2d) = (quads(0), quads(4), quads(8));
        let (m3q, m3s, m4) = (bits(12), bits(13), bits(14));
        if bits(15) != 0 {
            v.report("trigger table word with bit 15 set, which no table has");
        }
        if m4 != 0xa000 || m3s != 0 {
            v.report(format!(
                "trigger table M4 {m4:#06x}, M3S {m3s:#06x}: the model knows the driver's simple \
                 trigger only (M4 0xa000, M3S 0)"
            ));
        }
        Some(Trigger::Lut { m0d, m1d, m2d, m3q })
    }

    fn fetch_row(&mut self, bank: usize, v: &mut Violations) {
        if self.capturing() {
            v.report("REG_DRAM_BLOCK during a capture");
        }
        if !self.readen {
            v.report("REG_DRAM_BLOCK without WMR_SDRAMREADEN");
        }
        if let Some(p) = self.pending {
            v.report(format!("REG_DRAM_BLOCK while the fetch into buffer {p} was not waited for"));
        }
        if self.banks[bank].data.is_some() && !self.banks[bank].sent {
            v.report(format!("REG_DRAM_BLOCK into buffer {bank}, whose row was not read"));
        }
        let Some(row) = self.memrow else {
            v.report("REG_DRAM_BLOCK before WRITE_MEMROW");
            return;
        };
        let data = self.capture.as_ref().map_or_else(|| vec![0; ROW_BYTES], |c| c.row(row));
        self.banks[bank] = Bank { data: Some(data), sent: false };
        self.memrow = Some((row + 1) % ROW_COUNT);
        self.pending = Some(bank);
    }

    fn send_bank(&mut self, bank: usize, v: &mut Violations) {
        if self.pending == Some(bank) {
            v.report(format!(
                "REG_DRAM_BLOCK_DATA of buffer {bank} before its fetch was waited for"
            ));
        }
        let b = &mut self.banks[bank];
        match &b.data {
            None => {
                v.report(format!("REG_DRAM_BLOCK_DATA of buffer {bank}, never fetched"));
                self.out.extend(std::iter::repeat_n(0, ROW_BYTES));
            }
            Some(d) => {
                if b.sent {
                    v.report(format!("REG_DRAM_BLOCK_DATA of buffer {bank} a second time"));
                }
                self.out.extend(d.iter().copied());
                b.sent = true;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bytes `sigma_write_register` sends.
    fn write_reg(reg: u8, data: &[u8]) -> Vec<u8> {
        let mut b = vec![reg & 0xf, 0x10 | reg >> 4];
        for &d in data {
            b.extend([0x20 | (d & 0xf), 0x30 | d >> 4]);
        }
        b
    }

    fn read_regs(reg: u8, count: usize) -> Vec<u8> {
        let mut b = vec![reg & 0xf, 0x10 | reg >> 4];
        b.extend(std::iter::repeat_n(0x41, count));
        b
    }

    fn ready(netlist: Netlist) -> (Fpga, Violations) {
        let mut f = Fpga::new(netlist, FpgaFaults::default(), 2);
        let mut v = Violations::default();
        // sigma_fpga_init_la.
        let mut c = vec![0x00, 0x10, 0x40];
        c.extend(write_reg(15, &[0x55]));
        c.push(0x40);
        c.extend(write_reg(15, &[0xaa]));
        c.push(0x40);
        c.extend(write_reg(3, &[0x80]));
        f.commands(&c, &mut v);
        assert_eq!(f.take(10), [0xa6, 0x55, 0xaa]);
        (f, v)
    }

    /// The trigger table writes of a capture without a trigger.
    fn program_50(f: &mut Fpga, lut: &[u16; 16], params: &[u8], v: &mut Violations) {
        f.commands(&write_reg(2, &[0x20]), v);
        for (a, w) in lut.iter().enumerate() {
            f.commands(&write_reg(1, &w.to_be_bytes()), v);
            f.commands(&write_reg(2, &[0x30 | a as u8]), v);
        }
        f.commands(&write_reg(1, params), v);
        f.commands(&write_reg(6, &[0x08, 0x40]), v);
        f.commands(&write_reg(2, &[0xc0]), v);
    }

    fn positions(fpga: &mut Fpga, v: &mut Violations) -> (u32, u32, u8) {
        fpga.commands(&read_regs(1, 7), v);
        let regs = fpga.take(7);
        let trig = u32::from_le_bytes([regs[0], regs[1], regs[2], 0]);
        let stop = u32::from_le_bytes([regs[3], regs[4], regs[5], 0]);
        (trig, stop, regs[6])
    }

    #[test]
    fn captures_and_reads_rows() {
        let (mut f, mut v) = ready(Netlist::Mhz50);
        program_50(&mut f, &[0; 16], &[0; 6], &mut v);
        // 1 MHz.
        f.commands(&write_reg(0, &[0, 49, 0, 0]), &mut v);
        f.commands(&write_reg(5, &[127]), &mut v);
        f.commands(&write_reg(3, &[0x05]), &mut v);
        let (_, stop, mode) = positions(&mut f, &mut v);
        // 10000 ticks, a change every 2: every slot stored, 1429 slots.
        assert_eq!(stop, (22 << 9) | (1429 * 7 - 22 * 448));
        assert_eq!(mode, rmr::SDRAMWRITEEN);
        f.commands(&write_reg(3, &[0x11]), &mut v);
        f.commands(&[0x07, 0x10, 0x40], &mut v);
        assert_eq!(f.take(1), [rmr::POSTTRIGGERED]);
        f.commands(&write_reg(3, &[0x02]), &mut v);
        // Two rows, as sigma_read_dram fetches them.
        let mut c = write_reg(4, &[0, 0]);
        c.extend([0x60, 0x50, 0x70, 0xa0, 0x50, 0xb0]);
        f.commands(&c, &mut v);
        let d = f.take(4096);
        assert_eq!(d.len(), 2048);
        // Cluster 0: timestamp 0, samples 0 0 1 1 2 2 3.
        assert_eq!(&d[..16], &[0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 2, 0, 2, 0, 3, 0]);
        // Row 1, cluster 0: timestamp 448.
        assert_eq!(&d[1024..1028], &[0xc0, 0x01, 224, 0]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn run_length_and_the_timestamp_wrap() {
        let mut c = Capture {
            netlist: Netlist::Mhz50,
            step: 100_000,
            enabled: 0xffff,
            per_poll: 500_000,
            trigger: None,
            post_events: 0,
            running: true,
            now: 0,
            scanned: 0,
            slots: Vec::new(),
            last: 0,
            trig: None,
            stop_left: None,
            over: false,
        };
        c.poll();
        // The first slot, the changes at 100000, 200000, ..., and a
        // slot whenever the timestamp wraps; the last slot scanned holds
        // tick 500000.
        let changes = [100_000u64, 200_000, 300_000, 400_000, 500_000];
        for t in changes {
            assert!(c.slots.contains(&(t / 7 * 7)), "{t}");
        }
        assert_eq!(c.slots[0], 0);
        assert!(c.slots.windows(2).all(|w| w[1] - w[0] <= 65536 + 6));
        assert_eq!(c.slots.len(), 1 + 5 + 7);
        // A forced stop stores the slot at the stop tick, unchanged pins
        // and all.
        let at = c.scanned;
        c.force_stop();
        assert_eq!(c.slots.len(), 1 + 5 + 7 + 1);
        assert_eq!(c.slots.last(), Some(&at));
        assert!(!c.running);
    }

    #[test]
    fn interleaves_the_fast_netlists() {
        let mut c = Capture {
            netlist: Netlist::Mhz200,
            step: 1,
            enabled: 0xff,
            per_poll: 0,
            trigger: None,
            post_events: 0,
            running: false,
            now: 0,
            scanned: 0,
            slots: Vec::new(),
            last: 0,
            trig: None,
            stop_left: None,
            over: false,
        };
        // Tick 0 holds samples 0, 1, 2, 3: channel i of sample k at
        // bit 4i + k (sigma_deinterlace_data_4x4).
        let w = c.word(0);
        for k in 0..4u16 {
            let s = (0..4).fold(0, |acc, i| acc | ((w >> (4 * i + k) & 1) << i));
            assert_eq!(s, k);
        }
        c.netlist = Netlist::Mhz100;
        let w = c.word(3);
        for k in 0..2u16 {
            let s = (0..8).fold(0, |acc, i| acc | ((w >> (2 * i + k) & 1) << i));
            assert_eq!(s, 6 + k);
        }
    }

    #[test]
    fn lut_trigger_as_the_driver_builds_it() {
        // channel 3 rising (sigma_build_basic_trigger with risingmask
        // 0x0004): M0D selects channel 3 high, M3Q is OP_RISE on it.
        let m0d = |q: usize, a: usize| -> bool { q != 0 || a & 4 != 0 };
        let mut lut = [0u16; 16];
        for (a, w) in lut.iter_mut().enumerate() {
            let mut m0 = 0;
            for q in 0..4 {
                m0 |= u16::from(m0d(q, a)) << q;
            }
            // OP_RISE on selection 0: now (bit 0) high, before (bit 1) low.
            let m3q = u16::from(a & 3 == 1);
            let m4 = u16::from(a == 13 || a == 15);
            *w = (m4 << 14) | (m3q << 12) | (0xf << 8) | m0;
        }
        let (mut f, mut v) = ready(Netlist::Mhz50);
        program_50(&mut f, &lut, &[0, 0x0c, 0, 0, 0, 0], &mut v);
        f.commands(&write_reg(0, &[0, 0, 0, 0]), &mut v);
        f.commands(&write_reg(5, &[255]), &mut v);
        f.commands(&write_reg(3, &[0x0d]), &mut v);
        let (trig, stop, mode) = positions(&mut f, &mut v);
        // Channel 3 rises at sample 8 (count 4, step 2): event 8, the
        // register points after it.
        assert_eq!(trig, 9);
        assert_eq!(mode & (rmr::TRIGGERED | rmr::POSTTRIGGERED), rmr::TRIGGERED | rmr::POSTTRIGGERED);
        // 1/256 of the memory after the trigger event, whole clusters.
        let events = (8 + 1 + MEMORY_EVENTS / 256).div_ceil(7) * 7;
        assert_eq!(stop, Capture::position(events));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_what_the_driver_never_sends() {
        let (mut f, mut v) = ready(Netlist::Mhz50);
        let mut check = |f: &mut Fpga, c: &[u8], what: &str| {
            f.commands(c, &mut v);
            assert!(!v.drain().is_empty(), "{what}");
        };
        check(&mut f, &[0x81], "block begin");
        check(&mut f, &[0xc0], "unknown opcode");
        check(&mut f, &[0x31], "a high nibble alone");
        check(&mut f, &[0x21, 0x22, 0x33], "two low nibbles");
        check(&mut f, &write_reg(0, &[0, 1]), "a short clock select");
        check(&mut f, &write_reg(7, &[0]), "pin view");
        check(&mut f, &[0x08, 0x10, 0x40], "a read of register 8");
        check(&mut f, &write_reg(3, &[0x05]), "a start with no clock");
        check(&mut f, &[0x60], "a fetch with no row and no capture");
        check(&mut f, &[0xb0], "a buffer never fetched");
        check(&mut f, &[0x50], "a wait with nothing to wait for");
        check(&mut f, &write_reg(3, &[0x11]), "a stop with no capture");
        check(&mut f, &write_reg(6, &[0x08, 0x41]), "other trigger options");
        check(&mut f, &write_reg(2, &[0x10]), "a strobe outside programming mode");
        check(&mut f, &[0x43], "a read moving down");
        let mut f = Fpga::new(Netlist::Mhz100, FpgaFaults::default(), 2);
        check(&mut f, &write_reg(0, &[0, 0, 0, 0]), "the 50 MHz clock select at 100 MHz");
    }

    #[test]
    fn forced_stop_takes_polls() {
        let faults = FpgaFaults { stop_polls: 2, ..FpgaFaults::default() };
        let mut f = Fpga::new(Netlist::Mhz100, faults, 2);
        let mut v = Violations::default();
        f.commands(&write_reg(3, &[0x80]), &mut v);
        f.commands(&write_reg(2, &[0x20]), &mut v);
        f.commands(&write_reg(2, &[0x81]), &mut v);
        f.commands(&write_reg(6, &[0x08, 0x40]), &mut v);
        f.commands(&write_reg(2, &[0x80]), &mut v);
        f.commands(&write_reg(0, &[0]), &mut v);
        f.commands(&write_reg(5, &[127]), &mut v);
        f.commands(&write_reg(3, &[0x05]), &mut v);
        positions(&mut f, &mut v);
        f.commands(&write_reg(3, &[0x11]), &mut v);
        let mut modes = Vec::new();
        for _ in 0..3 {
            f.commands(&[0x07, 0x10, 0x40], &mut v);
            modes.push(f.take(1)[0] & rmr::POSTTRIGGERED);
        }
        assert_eq!(modes, [0, 0, rmr::POSTTRIGGERED]);
        assert!(v.is_empty(), "{v:?}");
    }
}
