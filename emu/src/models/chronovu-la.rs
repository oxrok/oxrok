// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The `ChronoVu` LA8 and LA16 logic analyzers as sigrok's `chronovu-la`
//! driver sees them: an FTDI chip in FIFO mode in front of the
//! analyzer's logic (a sequencer and 8 MiB of SDRAM). Every fact below
//! is from libsigrok's `src/hardware/chronovu-la` (`api.c`,
//! `protocol.c`, `protocol.h`) and `contrib/60-libsigrok.rules` unless
//! said otherwise; nothing else about the device is available here.
//!
//! - USB: FTDI's vendor id 0x0403 (`SCAN_EXPECTED_VENDOR`) with product
//!   id 0x6001 ("old VID/PID") or 0x8867 ("new VID/PID", the default),
//!   both from the udev rules. The driver finds the device by its
//!   product string alone, `ChronoVu LA8` or `ChronoVu LA16`
//!   (`cv_profiles`), and opens it with `ftdi_usb_open_desc` on the ids
//!   it found. Which FTDI chip the boards carry, the manufacturer string
//!   and the serial numbers are not in the sources: the model takes a
//!   full speed single port chip, a BM by default (`chip=r` for an R),
//!   manufacturer `ChronoVu` and serial `CV0000001`, all unverified.
//!   The driver never reads the manufacturer; it reads the serial
//!   number, and the chip type only changes libftdi's 9600 Bd divisor.
//!   A BM without a serial number reports bcdDevice 0x0200 and libftdi
//!   still takes it for a BM (`serial=`).
//! - The command, one bulk OUT transfer (`dev_acquisition_start`): on
//!   the LA8 four bytes, divcount, 0xff ("This byte must always be
//!   0xff"), trigger pattern, trigger mask; on the LA16 eight, divcount,
//!   0xff, then pattern, mask and edge mask as 16 bit values, high byte
//!   (channels 8 to 15) first, as the driver builds them (its comments
//!   call that byte LSB, which nothing here can settle). divcount is 0
//!   to 0xfe, the sample period `(divcount + 1) * 10 ns` on the LA8 and
//!   `* 5 ns` on the LA16 (`cv_samplerate_to_divcount`). A mask bit asks
//!   for the pattern's level on its channel, an edge mask bit (LA16
//!   only) for an edge, rising when the pattern bit is 1 (`protocol.h`,
//!   `trigger_pattern` and the masks).
//! - Eight bytes of 0x01 reset the sequencer (`close_usb_reset_sequencer`,
//!   "Magic sequence"): the capture or readout in progress ends.
//! - The capture: the logic samples its inputs from the command on,
//!   waits for the trigger condition and fills its 8 MiB, then hands the
//!   whole memory to the FIFO; until then the chip has nothing and
//!   answers status-only packets, which is how the driver learns that
//!   the capture is done (`cv_read_block` retries block 0 while it reads
//!   0 bytes, until a wall clock timeout). The readout order is the
//!   inverse of the driver's de-mangling in `cv_read_block`: eight
//!   sections of 1 MiB, section `m` holding samples `16k + 2m` and
//!   `16k + 2m + 1`; on the LA8 a pair's two bytes come in sample order
//!   at divcount 0 and swapped otherwise, on the LA16 each 16 bit sample
//!   low byte first (the driver's index arithmetic and its byte swap in
//!   `cv_send_block_to_session_bus` combined). Since the order is taken
//!   from the driver, the model cannot find a fault in the de-mangling;
//!   what it checks is the command, the sequence and the sizes.
//!
//! Assumptions the sources leave open, each an emulation choice:
//! - The first sample in memory is the one that met the trigger
//!   condition (the first after the command without a trigger): the
//!   driver gets no trigger position from the device and searches the
//!   data for the first match, which on the LA8 then is sample 0. The
//!   `ChronoVu` file format (`src/input/chronovu_la8.c`) has a trigger
//!   position field, so the real device may keep samples before it:
//!   `pretrigger=N` keeps N (no more than were taken since the command),
//!   which runs the driver's pre-trigger split on real data. How many a
//!   real device keeps, if any, is not known here.
//! - The inputs carry `t ^ t >> 8 ^ t >> 16` (LA8, 8 bits) or
//!   `t ^ t >> 16` (LA16, 16 bits) for the `t`-th sample period after
//!   the command: a count for the first 256 or 65536 samples, so every
//!   state and edge condition is met early and at a predictable sample,
//!   and different for every 256 sample block after that, so a misplaced
//!   block shows.
//! - Filling the memory takes `fill=N` status-only reads (default 2),
//!   whatever the rate: time counted in the host's polls, not in the
//!   real `8 Mi * period` (84 ms at 100 MHz, 21 s at 392 kHz). So every
//!   capture here completes, where on a real LA8 one at divcount 119 or
//!   more (below about 833 kHz) cannot: the driver's deadline
//!   (`api.c:491`) adds at most `255 * 0.8388608` (214 us) to a
//!   microsecond clock plus 10 s, so its block 0 retry loop
//!   (`protocol.c:370-383`) ends with "Trigger timed out. Bytes read: 0."
//!   before the memory is full (8 Mi samples take over 10 s there).
//!
//! Checks: the command's length for the model, the 0xff byte, divcount
//! 0xff, pattern bits outside the masks and channels in both masks
//! (neither of which the driver sends), a command while a capture or
//! readout is in progress, any other write, and every bit mode but the
//! reset (the FIFO is the only link to the logic).
//!
//! Faults, off by default: `no-trigger=1` (the inputs never meet the
//! trigger condition, as with nothing connected: a capture with a
//! trigger never ends, status-only reads for ever; the scenario's `no
//! trigger` case stops sigrok-cli after 3 s, before the driver's own
//! 10 s deadline, whose path stays without a case because the number of
//! reads before it depends on the machine), `gap=N` (the FIFO runs dry once, for one
//! latency period, at the `N`th readout transfer, counted from 0: a
//! status-only packet), and the FTDI shell's `stall=REQUEST[/WVALUE]`,
//! `in-stall=N` and `idle=N` (inert here: the driver sets no bit mode).

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Violations};
use crate::shells::ftdi::{
    Chip, Countdown, Event, Faults, Fill, Ftdi, FtdiConfig, FtdiPorts, PortState, bitmode,
};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "chronovu-la";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "ChronoVu LA8/LA16 (FTDI FIFO, 8 MiB SDRAM) as sigrok's chronovu-la \
     uses it; model=la8|la16, pid=0x8867|0x6001 (any), chip=bm|r, product=, serial= (empty: none), \
     fill=N (status-only reads before the readout, 2), pretrigger=N (samples kept before the trigger, 0); faults: no-trigger=1, gap=N, the FTDI \
     shell's stall=REQUEST[/WVALUE], in-stall=N, idle=N";

/// The sample memory, in bytes (`SDRAM_SIZE`).
pub const MEMORY: usize = 8 * 1024 * 1024;
/// One de-mangling section (`cv_read_block`'s 1 MiB).
const SECTION: usize = 1024 * 1024;
/// The sequencer reset (`close_usb_reset_sequencer`).
pub const RESET_SEQUENCER: [u8; 8] = [0x01; 8];

/// The two models.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    /// 8 channels, 100 MHz, state triggers.
    La8,
    /// 16 channels, 200 MHz, state and edge triggers.
    La16,
}

impl Model {
    /// The product string the driver looks for.
    pub fn product(self) -> &'static str {
        match self {
            Model::La8 => "ChronoVu LA8",
            Model::La16 => "ChronoVu LA16",
        }
    }

    /// The length of the command.
    pub fn command_len(self) -> usize {
        match self {
            Model::La8 => 4,
            Model::La16 => 8,
        }
    }

    /// Bytes per sample.
    pub fn unit(self) -> usize {
        match self {
            Model::La8 => 1,
            Model::La16 => 2,
        }
    }

    /// The level of the inputs `t` sample periods after the command.
    pub fn signal(self, t: u64) -> u16 {
        match self {
            Model::La8 => u16::from((t ^ t >> 8 ^ t >> 16) as u8),
            Model::La16 => (t ^ t >> 16) as u16,
        }
    }
}

/// A decoded command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Command {
    /// The clock divider minus 1.
    pub divcount: u8,
    /// The levels (or edge directions) asked for.
    pub pattern: u16,
    /// Channels that must be at the pattern's level.
    pub mask: u16,
    /// Channels that must see an edge (LA16).
    pub edges: u16,
}

impl Command {
    /// Whether sample `t` (with `prev` before it, if any) meets the
    /// trigger condition.
    pub fn matches(&self, prev: Option<u16>, cur: u16) -> bool {
        if (cur ^ self.pattern) & self.mask != 0 {
            return false;
        }
        if self.edges == 0 {
            return true;
        }
        let Some(prev) = prev else { return false };
        // An edge: the level changed, and is now the pattern's.
        (prev ^ cur) & self.edges == self.edges && (cur ^ self.pattern) & self.edges == 0
    }

    /// The first sample period that meets the condition, within the
    /// first `limit`.
    pub fn first_match(&self, model: Model, limit: u64) -> Option<u64> {
        let mut prev = None;
        for t in 0..limit {
            let cur = model.signal(t);
            if self.matches(prev, cur) {
                return Some(t);
            }
            prev = Some(cur);
        }
        None
    }
}

/// Where the sequencer is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    /// Waiting for a command.
    Idle,
    /// Sampling: status-only reads until the memory is full; `start`
    /// is the trigger's sample period, `None` while it never comes.
    Capture { start: Option<u64> },
    /// Handing the memory to the FIFO, `sent` bytes so far, in
    /// `transfers` transfers.
    Readout { start: u64, sent: usize, transfers: u32 },
}

/// The analyzer's logic behind the FIFO.
#[derive(Clone, Debug)]
pub struct Logic {
    model: Model,
    fill: u32,
    no_trigger: bool,
    gap: Option<u32>,
    pretrigger: u64,
    phase: Phase,
    command: Option<Command>,
    busy: Countdown,
}

impl Logic {
    /// The logic of `model` at power on, with its emulation settings
    /// and faults (see the module documentation).
    pub fn new(model: Model, fill: u32, no_trigger: bool, gap: Option<u32>) -> Logic {
        Logic {
            model,
            fill,
            no_trigger,
            gap,
            pretrigger: 0,
            phase: Phase::Idle,
            command: None,
            busy: Countdown::new(fill),
        }
    }

    /// The same logic keeping `samples` samples before the trigger
    /// sample (as many as it has taken since the command, at most).
    #[must_use]
    pub fn with_pretrigger(mut self, samples: u64) -> Logic {
        self.pretrigger = samples;
        self
    }

    /// The last command taken.
    pub fn command(&self) -> Option<Command> {
        self.command
    }

    /// Byte `o` of the readout, for a capture whose first sample is the
    /// `start`-th sample period.
    pub fn readout_byte(&self, start: u64, offset: usize) -> u8 {
        let section = offset / SECTION;
        let within = offset % SECTION;
        match self.model {
            Model::La8 => {
                let odd = within & 1;
                let divcount = self.command.map_or(0, |c| c.divcount);
                let second = if divcount == 0 { odd } else { 1 - odd };
                let n = 16 * (within / 2) + 2 * section + second;
                self.model.signal(start + n as u64) as u8
            }
            Model::La16 => {
                // The driver puts byte r at index 32k + 4m + q + (1 - p)
                // and then swaps each sample's two bytes: in the end at
                // 32k + 4m + (r & 3), samples little endian.
                let at = 32 * (within / 4) + 4 * section + (within & 3);
                let sample = self.model.signal(start + (at / 2) as u64);
                sample.to_le_bytes()[at & 1]
            }
        }
    }

    /// The next transfer of the readout, up to `max` bytes.
    fn readout(&mut self, max: usize) -> Fill {
        let Phase::Readout { start, sent, transfers } = self.phase else { return Fill::Idle };
        if self.gap == Some(transfers) {
            self.gap = None;
            self.phase = Phase::Readout { start, sent, transfers: transfers + 1 };
            return Fill::Idle;
        }
        let n = max.min(MEMORY - sent);
        let data: Vec<u8> = (sent..sent + n).map(|at| self.readout_byte(start, at)).collect();
        self.phase = if sent + n == MEMORY {
            Phase::Idle
        } else {
            Phase::Readout { start, sent: sent + n, transfers: transfers + 1 }
        };
        Fill::Data(data)
    }

    fn take_command(&mut self, data: &[u8], v: &mut Violations) {
        if data.len() != self.model.command_len() {
            v.report(format!(
                "{} byte write: the {:?} takes a {} byte command or the {} byte sequencer reset",
                data.len(),
                self.model,
                self.model.command_len(),
                RESET_SEQUENCER.len()
            ));
            return;
        }
        if data[1] != 0xff {
            v.report(format!("command byte 1 is {:#04x}; it must always be 0xff", data[1]));
        }
        if data[0] == 0xff {
            v.report("command divcount 0xff: 0x00 to 0xfe");
        }
        let c = match self.model {
            Model::La8 => Command {
                divcount: data[0],
                pattern: u16::from(data[2]),
                mask: u16::from(data[3]),
                edges: 0,
            },
            Model::La16 => Command {
                divcount: data[0],
                pattern: u16::from_be_bytes([data[2], data[3]]),
                mask: u16::from_be_bytes([data[4], data[5]]),
                edges: u16::from_be_bytes([data[6], data[7]]),
            },
        };
        if c.pattern & !(c.mask | c.edges) != 0 {
            v.report(format!(
                "trigger pattern {:#06x} outside mask {:#06x} and edge mask {:#06x}",
                c.pattern, c.mask, c.edges
            ));
        }
        if c.mask & c.edges != 0 {
            v.report(format!(
                "channels {:#06x} in both the trigger mask and the edge mask",
                c.mask & c.edges
            ));
        }
        if self.phase != Phase::Idle {
            v.report("a command while a capture or readout is in progress");
        }
        self.command = Some(c);
        let start = if self.no_trigger && (c.mask | c.edges) != 0 {
            None
        } else {
            c.first_match(self.model, 1 << 24).map(|t| t.saturating_sub(self.pretrigger))
        };
        self.phase = Phase::Capture { start };
        self.busy = Countdown::new(self.fill);
        self.busy.arm();
    }
}

impl FtdiPorts for Logic {
    fn changed(&mut self, _: usize, event: Event, state: &PortState, v: &mut Violations) {
        if event == Event::Bitmode && state.bitmode != bitmode::RESET {
            v.report(format!(
                "bit mode {:#04x}: the analyzer talks through the FIFO, never a bit mode",
                state.bitmode
            ));
        }
    }

    fn write(&mut self, _: usize, data: &[u8], _: &PortState, v: &mut Violations) -> Reply {
        if data == RESET_SEQUENCER {
            self.phase = Phase::Idle;
        } else {
            self.take_command(data, v);
        }
        Reply::ack()
    }

    fn read(&mut self, _: usize, max: usize, _: &PortState, _: &mut Violations) -> Fill {
        match self.phase {
            Phase::Idle | Phase::Capture { start: None } => Fill::Idle,
            Phase::Capture { start: Some(_) } if self.busy.busy() => Fill::Idle,
            Phase::Capture { start: Some(start) } => {
                self.phase = Phase::Readout { start, sent: 0, transfers: 0 };
                self.readout(max)
            }
            Phase::Readout { .. } => self.readout(max),
        }
    }

    fn pins(&mut self, _: usize, _: &PortState, v: &mut Violations) -> u8 {
        v.report("READ_PINS: chronovu-la never reads the FIFO's data pins");
        0
    }
}

/// The emulated analyzer.
pub struct ChronovuLa {
    chip: Ftdi,
    logic: Logic,
    fresh: Logic,
}

impl ChronovuLa {
    /// The analyzer: its chip and its logic at power on.
    pub fn new(config: FtdiConfig, logic: Logic) -> ChronovuLa {
        ChronovuLa { chip: Ftdi::new(config), fresh: logic.clone(), logic }
    }

    /// The logic, for the command taken.
    pub fn logic(&self) -> &Logic {
        &self.logic
    }
}

/// Long data as its digest.
pub fn log_rules(_: &[(String, String)]) -> crate::txlog::Rules {
    crate::txlog::Rules::digest_above(crate::txlog::HASH_ABOVE)
}

/// An analyzer from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a bad value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut model = Model::La8;
    let mut chip = Chip::Bm;
    let mut product = 0x8867;
    let mut description = None;
    let mut serial = Some("CV0000001".to_owned());
    let mut fill = 2;
    let mut no_trigger = false;
    let mut gap = None;
    let mut pretrigger = 0;
    let mut faults = Faults::default();
    let number = |k: &str, v: &str| -> Result<u32> {
        match v.strip_prefix("0x") {
            Some(h) => u32::from_str_radix(h, 16),
            None => v.parse(),
        }
        .map_err(|_| Error::Invalid(format!("{k}={v}: not a number")))
    };
    for (k, v) in options {
        match k.as_str() {
            "model" => {
                model = match v.as_str() {
                    "la8" => Model::La8,
                    "la16" => Model::La16,
                    _ => return Err(Error::Invalid(format!("model={v}: la8 or la16"))),
                }
            }
            "chip" => {
                chip = match v.as_str() {
                    "bm" => Chip::Bm,
                    "r" => Chip::R,
                    _ => return Err(Error::Invalid(format!("chip={v}: bm or r"))),
                }
            }
            "pid" => {
                product = u16::try_from(number(k, v)?)
                    .map_err(|_| Error::Invalid(format!("pid={v}: 16 bits")))?;
            }
            "product" => description = Some(v.clone()),
            "serial" => serial = (!v.is_empty()).then(|| v.clone()),
            "fill" => fill = number(k, v)?,
            "no-trigger" => no_trigger = number(k, v)? != 0,
            "gap" => gap = Some(number(k, v)?),
            "pretrigger" => pretrigger = u64::from(number(k, v)?),
            other => {
                if !faults.option(other, v)? {
                    return Err(Error::Invalid(format!("{NAME} has no option {other}")));
                }
            }
        }
    }
    let mut config = FtdiConfig::new(chip);
    config.product = product;
    config.manufacturer = Some("ChronoVu".to_owned());
    let description = description.unwrap_or_else(|| model.product().to_owned());
    config.description = (!description.is_empty()).then_some(description);
    config.serial = serial;
    let mut dev = ChronovuLa::new(config, Logic::new(model, fill, no_trigger, gap).with_pretrigger(pretrigger));
    dev.chip.set_faults(faults);
    Ok(Box::new(dev))
}

impl DeviceModel for ChronovuLa {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        self.chip.descriptors()
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.control(&mut self.logic, setup, data, v)
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        self.chip.out(&mut self.logic, ep, data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        self.chip.in_transfer(&mut self.logic, ep, max, v)
    }

    fn reset(&mut self) {
        self.chip.reset();
        self.logic = self.fresh.clone();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn o(k: &str, v: &str) -> (String, String) {
        (k.to_owned(), v.to_owned())
    }

    fn analyzer(options: &[(String, String)]) -> Box<dyn DeviceModel> {
        build(options).unwrap()
    }

    /// Reads the whole readout as libftdi does, 4096 byte transfers of
    /// 64 byte packets, status bytes stripped; the number of
    /// status-only reads before it.
    fn read_all(dev: &mut dyn DeviceModel, v: &mut Violations) -> (usize, Vec<u8>) {
        let mut idle = 0;
        let mut out = Vec::new();
        while out.len() < MEMORY {
            let Reply::Data(d) = dev.in_transfer(1, 4096, v) else { panic!() };
            if d.len() == 2 {
                assert!(out.is_empty(), "a gap in the readout");
                idle += 1;
                continue;
            }
            for p in d.chunks(64) {
                assert_eq!(&p[..2], &[0x01, 0x60]);
                out.extend_from_slice(&p[2..]);
            }
        }
        (idle, out)
    }

    /// The driver's de-mangling (`cv_read_block`) and the LA16 byte swap
    /// (`cv_send_block_to_session_bus`).
    fn demangle(model: Model, divcount: u8, raw: &[u8]) -> Vec<u8> {
        let mut out = vec![0u8; MEMORY];
        for (i, &byte) in raw.iter().enumerate() {
            let (m, rest) = (i / SECTION, i % SECTION);
            let (p, q) = (i & 1, i & 2);
            let index = match model {
                Model::La8 => 2 * m + (rest / 2) * 16 + if divcount == 0 { p } else { 1 - p },
                Model::La16 => 4 * m + (rest / 4) * 32 + q + (1 - p),
            };
            out[index] = byte;
        }
        if model == Model::La16 {
            for pair in out.chunks_mut(2) {
                pair.swap(0, 1);
            }
        }
        out
    }

    #[test]
    fn la8_capture_as_the_driver_reads_it() {
        for divcount in [0u8, 99] {
            let mut dev = analyzer(&[]);
            let mut v = Violations::default();
            // Idle before the command.
            assert_eq!(dev.in_transfer(1, 4096, &mut v), Reply::Data(vec![0x01, 0x60]));
            // Trigger: channel 7 high.
            assert_eq!(dev.out(2, &[divcount, 0xff, 0x80, 0x80], &mut v), Reply::ack());
            let (idle, raw) = read_all(dev.as_mut(), &mut v);
            assert_eq!(idle, 2);
            let samples = demangle(Model::La8, divcount, &raw);
            let want: Vec<u8> =
                (0..MEMORY as u64).map(|n| Model::La8.signal(128 + n) as u8).collect();
            assert!(samples == want, "divcount {divcount}");
            assert_eq!(&samples[..3], &[0x80, 0x81, 0x82]);
            // Back to idle.
            assert_eq!(dev.in_transfer(1, 4096, &mut v), Reply::Data(vec![0x01, 0x60]));
            assert!(v.is_empty(), "{v:?}");
        }
    }

    #[test]
    fn la16_capture_and_edge_trigger() {
        let mut dev = analyzer(&[o("model", "la16"), o("fill", "0")]);
        let mut v = Violations::default();
        // Rising edge on channel 1: first at sample 2.
        dev.out(2, &[3, 0xff, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02], &mut v);
        let (idle, raw) = read_all(dev.as_mut(), &mut v);
        assert_eq!(idle, 0);
        let f = demangle(Model::La16, 3, &raw);
        for n in [0usize, 1, 7, 70_000, MEMORY / 2 - 1] {
            let s = u16::from_le_bytes([f[2 * n], f[2 * n + 1]]);
            assert_eq!(s, Model::La16.signal(2 + n as u64), "sample {n}");
        }
        assert!(v.is_empty(), "{v:?}");
        // Channel 9 high (mask high byte first): first at 512.
        let c = Command { divcount: 0, pattern: 0x0200, mask: 0x0200, edges: 0 };
        assert_eq!(c.first_match(Model::La16, 1 << 20), Some(512));
        let c = Command { divcount: 0, pattern: 0, mask: 0, edges: 1 };
        assert_eq!(c.first_match(Model::La16, 1 << 20), Some(2), "falling on channel 0");
    }

    #[test]
    fn pretrigger_keeps_samples_before_the_trigger() {
        // Channel 7 high first at 128: 100 samples before it are kept.
        let mut dev = analyzer(&[o("pretrigger", "100"), o("fill", "0")]);
        let mut v = Violations::default();
        dev.out(2, &[1, 0xff, 0x80, 0x80], &mut v);
        let (_, raw) = read_all(dev.as_mut(), &mut v);
        let samples = demangle(Model::La8, 1, &raw);
        assert_eq!(&samples[..2], &[28, 29]);
        assert_eq!(samples[100], 0x80);
        // No more than were taken since the command.
        let mut dev = analyzer(&[o("pretrigger", "1000"), o("fill", "0")]);
        dev.out(2, &[1, 0xff, 0x80, 0x80], &mut v);
        let (_, raw) = read_all(dev.as_mut(), &mut v);
        assert_eq!(demangle(Model::La8, 1, &raw)[0], 0);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn faults() {
        // The trigger never comes.
        let mut dev = analyzer(&[o("no-trigger", "1"), o("fill", "0")]);
        let mut v = Violations::default();
        dev.out(2, &[0, 0xff, 1, 1], &mut v);
        for _ in 0..10 {
            assert_eq!(dev.in_transfer(1, 4096, &mut v), Reply::Data(vec![0x01, 0x60]));
        }
        // The sequencer reset ends it; a new command without a trigger runs.
        dev.out(2, &RESET_SEQUENCER, &mut v);
        dev.out(2, &[0, 0xff, 0, 0], &mut v);
        let Reply::Data(d) = dev.in_transfer(1, 4096, &mut v) else { panic!() };
        assert_eq!(d.len(), 4096);
        assert!(v.is_empty(), "{v:?}");
        // A gap at the second readout transfer.
        let mut dev = analyzer(&[o("gap", "1"), o("fill", "0")]);
        dev.out(2, &[0, 0xff, 0, 0], &mut v);
        let Reply::Data(d) = dev.in_transfer(1, 4096, &mut v) else { panic!() };
        assert_eq!(d.len(), 4096);
        assert_eq!(dev.in_transfer(1, 4096, &mut v), Reply::Data(vec![0x01, 0x60]));
        let Reply::Data(d) = dev.in_transfer(1, 4096, &mut v) else { panic!() };
        assert_eq!(d.len(), 4096);
        assert!(build(&[o("speed", "1")]).is_err());
        assert!(build(&[o("model", "la32")]).is_err());
    }

    #[test]
    fn flags_what_the_driver_never_sends() {
        let mut v = Violations::default();
        let mut dev = analyzer(&[]);
        for (bad, why) in [
            (vec![0u8, 0xff, 0, 0, 0, 0, 0, 0], "an LA16 command to an LA8"),
            (vec![0, 0xff, 0], "short"),
            (vec![0, 0xfe, 0, 0], "byte 1"),
            (vec![0xff, 0xff, 0, 0], "divcount 0xff"),
            (vec![0, 0xff, 1, 0], "pattern outside the mask"),
        ] {
            dev.reset();
            dev.out(2, &bad, &mut v);
            assert_eq!(v.drain().len(), 1, "{why}");
        }
        dev.reset();
        dev.out(2, &[0, 0xff, 0, 0], &mut v);
        dev.out(2, &[0, 0xff, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "a command while capturing");
        let mut dev = analyzer(&[o("model", "la16")]);
        dev.out(2, &[0, 0xff, 0, 1, 0, 1, 0, 1], &mut v);
        assert_eq!(v.drain().len(), 1, "a channel in both masks");
        dev.out(2, &[0, 0xff, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "an LA8 command to an LA16");
        let s = Setup { request_type: 0x40, request: 0x0b, value: 0x0100, index: 1, length: 0 };
        dev.control(s, &[], &mut v);
        assert_eq!(v.drain().len(), 1, "bit-bang");
        let s = Setup { request_type: 0xc0, request: 0x0c, value: 0, index: 1, length: 1 };
        dev.control(s, &[], &mut v);
        assert_eq!(v.drain().len(), 1, "read pins");
    }

    #[test]
    fn descriptors_and_strings() {
        let d = analyzer(&[]).descriptors().clone();
        assert_eq!((d.vendor_id(), d.product_id(), d.bcd_device()), (0x0403, 0x8867, 0x0400));
        assert_eq!(d.strings, ["ChronoVu", "ChronoVu LA8", "CV0000001"]);
        let d = analyzer(&[o("model", "la16"), o("pid", "0x6001"), o("serial", "")])
            .descriptors()
            .clone();
        assert_eq!((d.product_id(), d.bcd_device()), (0x6001, 0x0200));
        assert_eq!(d.strings, ["ChronoVu", "ChronoVu LA16"]);
        let d = analyzer(&[o("chip", "r")]).descriptors().clone();
        assert_eq!(d.bcd_device(), 0x0600);
    }
}
