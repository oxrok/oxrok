// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The oxla host protocol, version 1, and the pure parts of the driver:
//! the ID, the commands, the rates, the trigger terms, the registers of a
//! capture and the decoding of what it left in memory.
//!
//! A UART at 3,000,000 baud, 8N1, no flow control. Values are little
//! endian. The device only sends in answer to a command:
//!
//! | op | name | arguments | answer |
//! |---|---|---|---|
//! | `00` | NOP | | |
//! | `01` | RESET | | |
//! | `02` | ID | | 16 bytes |
//! | `03` | WREG | address (1), value (4) | |
//! | `04` | RREG | address (1) | value (4) |
//! | `05` | ARM | | |
//! | `06` | STOP | | |
//! | `07` | RMEM | word address (3), word count (3) | count × 4 bytes |
//!
//! Eight NOPs and a RESET get the two sides in step ([`RESYNC`]). The ID
//! says everything about the device ([`Id`]); nothing here assumes a clock,
//! a channel count or a memory size.
//!
//! The capture clock is the device's to say, and where its ID says so the
//! host's to choose: a write to `CLOCK` retunes it, the device is ready
//! again when `STATUS` says so, and the ID then has the new clock in Hz.
//! The driver asks each setting once for its clock and keeps the answers
//! in a [`Clocks`], which is where every rate comes from. `CLOCK` keeps its
//! value through a RESET. The document has every other register go back to
//! its default when `CLOCK` is written, and what was captured is lost, so
//! a capture's registers are all written after it.
//!
//! The memory is a ring of M 32 bit words, written from address 0 at ARM
//! for as long as the capture runs: the trigger starts nothing, it starts
//! the count of what is still to come. A capture ends `POST_SAMPLES`
//! samples after the trigger sample, or `POST_WORDS` words after the
//! trigger's word, or at STOP, or when the FIFO ahead of the memory
//! overflowed. The words before the `STOP` register are then the capture,
//! and `TRIG` says where the trigger sample is.
//!
//! In raw mode the samples are a stream of bits, W per sample, the earliest
//! sample and the lowest bit first, cut into words. With 14 channels a
//! sample can straddle two words; M is then a multiple of 7, so that an
//! address says where the samples of its word start ([`first_start`]). The
//! last word may be partly filled, and `STATUS` says by how much. In RLE
//! mode a word is a sample in its low W bits and a count above: the sample
//! was seen count + 1 times, and the trigger sample always starts a word.
//!
//! The board's USB bridge loses bytes when more than about 10 kB come in
//! one go, so memory is read [`RMEM_CHUNK`] words at a time, each request
//! sent when the answer to the one before is in.

use crate::core::{Trigger, TriggerCombine, TriggerMatch};

/// The line parameters of the device.
pub const SERIALCOMM: &str = "3000000/8n1";

/// Does nothing.
pub const OP_NOP: u8 = 0x00;
/// Ends a capture, discarding it, and puts every register back to its
/// default.
pub const OP_RESET: u8 = 0x01;
/// Asks for the 16 bytes of the ID.
pub const OP_ID: u8 = 0x02;
/// Writes a register.
pub const OP_WREG: u8 = 0x03;
/// Reads a register.
pub const OP_RREG: u8 = 0x04;
/// Starts a capture.
pub const OP_ARM: u8 = 0x05;
/// Ends a running capture, keeping what is in memory.
pub const OP_STOP: u8 = 0x06;
/// Reads memory.
pub const OP_RMEM: u8 = 0x07;

/// One sample every DIV + 1 clocks, 24 bits.
pub const REG_DIV: u8 = 0x00;
/// Words written after the word with the trigger sample.
pub const REG_POST_WORDS: u8 = 0x01;
/// The trigger is not looked for until this many words are written.
pub const REG_PRE_WORDS: u8 = 0x02;
/// Samples taken after the trigger sample; [`NO_SAMPLE_LIMIT`] for no
/// limit.
pub const REG_POST_SAMPLES: u8 = 0x03;
/// [`FLAG_RLE`], [`FLAG_DDR`], [`FLAG_TEST`].
pub const REG_FLAGS: u8 = 0x04;
/// Channels not set read as 0.
pub const REG_CHMASK: u8 = 0x05;
/// How the terms make the trigger event, a [`TriggerSetup::mode`].
pub const REG_TMODE: u8 = 0x06;
/// The trigger is the (TCOUNT + 1)th event, 16 bits.
pub const REG_TCOUNT: u8 = 0x07;
/// The capture clock, one of [`CLOCK_SETTINGS`] values. A write starts the
/// capture logic over.
pub const REG_CLOCK: u8 = 0x08;
/// Term A: MASK, VALUE, RISE, FALL, EDGE from here on.
pub const REG_TERM_A: u8 = 0x10;
/// Term B, the same.
pub const REG_TERM_B: u8 = 0x18;
/// The state of the capture, the `STATUS_` bits.
pub const REG_STATUS: u8 = 0x80;
/// Where the trigger sample is.
pub const REG_TRIG: u8 = 0x81;
/// The address after the last word written.
pub const REG_STOP: u8 = 0x82;
/// Words written, up to M where it stays.
pub const REG_COUNT: u8 = 0x83;

/// `FLAGS`: run length encoding.
pub const FLAG_RLE: u32 = 1;
/// `FLAGS`: two samples per clock, only with DIV = 0.
pub const FLAG_DDR: u32 = 2;
/// `FLAGS`: the samples are a counter, one step per sample from 0 at ARM.
pub const FLAG_TEST: u32 = 4;

/// `STATUS`: a capture runs.
#[allow(dead_code)]
pub const STATUS_RUNNING: u32 = 1;
/// `STATUS`: the trigger was seen.
pub const STATUS_TRIGGERED: u32 = 2;
/// `STATUS`: the capture ended.
pub const STATUS_DONE: u32 = 4;
/// `STATUS`: the FIFO ahead of the memory overflowed, which ended the
/// capture.
pub const STATUS_OVERFLOW: u32 = 8;

/// `STATUS`: the SDRAM is ready, as it is again some milliseconds after a
/// change of clock.
pub const STATUS_READY: u32 = 16;

/// The values `CLOCK` takes, from 0, on a device that does not say
/// ([`REG_CLOCKS`]).
pub const CLOCK_SETTINGS: u32 = 3;

/// The format of the registers that describe the bitstream, 1; 0 on a
/// bitstream that has none. They are constants.
pub const REG_DESC: u8 = 0x90;
/// The release the bitstream is of: major, minor and patch, a byte each.
pub const REG_BUILD: u8 = 0x91;
/// The values `CLOCK` takes in bits 7:0, the one at configuration above.
pub const REG_CLOCKS: u8 = 0x92;
/// Logic channels, analog channels and the bits of an analog sample, a
/// byte each from the lowest.
pub const REG_CHANNELS: u8 = 0x93;
/// Trigger terms, the bits of TCOUNT, and a bit per value of TMODE.
pub const REG_TRIGGER: u8 = 0x94;
/// The lowest DIV, the bits of DIV, the bits of a run's count.
pub const REG_LIMITS: u8 = 0x95;
/// The clock in Hz of `CLOCK` = 0, and of the values after it from here on.
pub const REG_CLOCK_HZ: u8 = 0xa0;
/// The registers of a [`Description`] ahead of its clocks, in the order
/// [`Description::new`] takes them.
pub const DESCRIPTION_REGS: [u8; 5] =
    [REG_BUILD, REG_CLOCKS, REG_CHANNELS, REG_TRIGGER, REG_LIMITS];
/// The most clocks a description has: the addresses there are for them.
pub const MAX_CLOCKS: u32 = 8;
/// `POST_SAMPLES` for no limit.
#[allow(dead_code)]
pub const NO_SAMPLE_LIMIT: u32 = 0xffff_ffff;
/// The most words one RMEM asks for.
pub const RMEM_CHUNK: u32 = 2048;
/// The length of the ID.
pub const ID_LEN: usize = 16;
/// What gets the two sides in step: the longest command has six argument
/// bytes, so eight NOPs end whatever was half received, and the RESET
/// undoes what that was.
pub const RESYNC: [u8; 9] = [OP_NOP, 0, 0, 0, 0, 0, 0, 0, OP_RESET];

const CAP_RLE: u8 = 1;
const CAP_DDR: u8 = 2;
const CAP_ADC: u8 = 4;
const CAP_CLOCKS: u8 = 8;
const CAP_DESCRIPTION: u8 = 16;
/// `TRIG` and `STOP` hold an address in 21 bits.
const ADDR_MASK: u32 = 0x1f_ffff;
/// DIV has 24 bits.
const MAX_DIVIDER: u64 = 1 << 24;

/// What a bitstream says of itself beyond its [`Id`], in the registers
/// from [`REG_DESC`]: what the driver otherwise takes from the protocol
/// document.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Description {
    /// The release it is of, major, minor and patch; all 0 for none.
    pub build: [u8; 3],
    /// The clock of each value of `CLOCK`, in Hz.
    pub clocks_hz: Vec<u32>,
    /// The value `CLOCK` has when the FPGA is configured.
    pub clock_at_start: u8,
    /// Channels that are logic.
    pub logic_channels: u8,
    /// Channels that are analog, each of `analog_bits` of the ID's channels.
    pub analog_channels: u8,
    /// Bits of an analog sample.
    pub analog_bits: u8,
    /// Trigger terms.
    pub trigger_terms: u8,
    /// Bits of TCOUNT.
    pub trigger_count_bits: u8,
    /// The values of TMODE there are, a bit each.
    pub trigger_modes: u8,
    /// The lowest DIV.
    pub div_min: u8,
    /// Bits of DIV.
    pub div_bits: u8,
    /// Bits of the count in a run length encoded word; 0 without RLE.
    pub rle_count_bits: u8,
}

impl Description {
    /// The description in the values of [`DESCRIPTION_REGS`] and of
    /// [`REG_CLOCK_HZ`] for each clock.
    pub fn new(regs: [u32; 5], clocks_hz: Vec<u32>) -> Description {
        let [build, clocks, channels, trigger, limits] = regs;
        let byte = |v: u32, n: u32| (v >> (8 * n)) as u8;
        Description {
            build: [byte(build, 2), byte(build, 1), byte(build, 0)],
            clocks_hz,
            clock_at_start: byte(clocks, 1),
            logic_channels: byte(channels, 0),
            analog_channels: byte(channels, 1),
            analog_bits: byte(channels, 2),
            trigger_terms: byte(trigger, 0),
            trigger_count_bits: byte(trigger, 1),
            trigger_modes: byte(trigger, 2),
            div_min: byte(limits, 0),
            div_bits: byte(limits, 1),
            rle_count_bits: byte(limits, 2),
        }
    }

    /// The release it is of, `0.2.0`.
    pub fn build(&self) -> String {
        let [major, minor, patch] = self.build;
        format!("{major}.{minor}.{patch}")
    }

    /// The setting of each of its clocks.
    pub fn clock_settings(&self) -> Vec<ClockSetting> {
        self.clocks_hz
            .iter()
            .zip(0..)
            .filter(|(hz, _)| **hz != 0)
            .map(|(hz, select)| ClockSetting { select: Some(select), hz: *hz })
            .collect()
    }
}

/// What the device says it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Id {
    /// The protocol version, 1.
    pub version: u8,
    /// The capability bits.
    pub caps: u8,
    /// Channels, W.
    pub channels: u8,
    /// Samples per clock at the highest rate, P: 1 or 2.
    pub samples_per_clock: u8,
    /// The capture clock in Hz, as it is now.
    pub clock_hz: u32,
    /// The length of the memory ring in words, M.
    pub mem_words: u32,
}

impl Id {
    /// The ID in the answer to [`OP_ID`].
    ///
    /// # Errors
    ///
    /// What is wrong with it, for a log line: not this device, or one this
    /// driver cannot use.
    pub fn parse(d: &[u8]) -> Result<Id, String> {
        if d.len() != ID_LEN || &d[..4] != b"OXLA" {
            let shown: Vec<String> = d.iter().map(|b| format!("{b:02x}")).collect();
            return Err(format!("not an oxla ID: {}", shown.join(" ")));
        }
        let le = |at: usize| u32::from_le_bytes([d[at], d[at + 1], d[at + 2], d[at + 3]]);
        let id = Id {
            version: d[4],
            caps: d[5],
            channels: d[6],
            samples_per_clock: d[7],
            clock_hz: le(8),
            mem_words: le(12),
        };
        if id.version != 1 {
            return Err(format!("protocol version {} is not supported", id.version));
        }
        if !(1..=16).contains(&id.channels) {
            return Err(format!("{} channels are not supported", id.channels));
        }
        if !(1..=2).contains(&id.samples_per_clock) {
            return Err(format!("{} samples per clock are not supported", id.samples_per_clock));
        }
        if id.clock_hz == 0 {
            return Err("the device reports no clock".to_owned());
        }
        // An address must say where the samples of its word start, across
        // the end of the ring too.
        if id.mem_words == 0
            || id.mem_words > ADDR_MASK + 1
            || !id.mem_words.is_multiple_of(group(id.channels))
        {
            return Err(format!("a memory of {} words is not supported", id.mem_words));
        }
        Ok(id)
    }

    /// The bitstream can run length encode.
    pub fn has_rle(&self) -> bool {
        self.caps & CAP_RLE != 0
    }

    /// The bitstream can take two samples per clock.
    pub fn has_ddr(&self) -> bool {
        self.caps & CAP_DDR != 0 && self.samples_per_clock == 2
    }

    /// The channels are the data of a parallel ADC, which the device
    /// clocks.
    pub fn is_adc(&self) -> bool {
        self.caps & CAP_ADC != 0
    }

    /// The host can change the capture clock, with [`REG_CLOCK`].
    pub fn has_clocks(&self) -> bool {
        self.caps & CAP_CLOCKS != 0
    }

    /// The bitstream describes itself, in the registers from [`REG_DESC`].
    pub fn has_description(&self) -> bool {
        self.caps & CAP_DESCRIPTION != 0
    }

    /// Whether `other` is the same bitstream, at whatever clock.
    pub fn same_board(&self, other: &Id) -> bool {
        Id { clock_hz: 0, ..*self } == Id { clock_hz: 0, ..*other }
    }

    /// Bytes per sample in a logic packet.
    pub fn unit_size(&self) -> usize {
        usize::from(self.channels).div_ceil(8)
    }

    /// Every channel.
    pub fn channel_mask(&self) -> u32 {
        (1 << self.channels) - 1
    }

    /// The samples the memory holds in raw mode.
    pub fn raw_capacity(&self) -> u64 {
        u64::from(self.mem_words) * 32 / u64::from(self.channels)
    }

    /// The name of the bitstream's variant, as its build names it:
    /// `la16-rle`, `la14-ddr`. The clock is not part of it.
    pub fn variant(&self) -> String {
        let mut name = format!("la{}", self.channels);
        for (on, suffix) in
            [(self.has_ddr(), "-ddr"), (self.has_rle(), "-rle"), (self.is_adc(), "-adc")]
        {
            if on {
                name.push_str(suffix);
            }
        }
        name
    }
}

/// `WREG`.
pub fn wreg(addr: u8, value: u32) -> [u8; 6] {
    let v = value.to_le_bytes();
    [OP_WREG, addr, v[0], v[1], v[2], v[3]]
}

/// `RREG`.
pub fn rreg(addr: u8) -> [u8; 2] {
    [OP_RREG, addr]
}

/// `RMEM` of `count` words from `addr`.
pub fn rmem(addr: u32, count: u32) -> [u8; 7] {
    let (a, c) = (addr.to_le_bytes(), count.to_le_bytes());
    [OP_RMEM, a[0], a[1], a[2], c[0], c[1], c[2]]
}

/// The requests that read `count` words from `start`, which wraps at
/// `mem_words`: address and count of each, none above [`RMEM_CHUNK`].
pub fn rmem_chunks(start: u32, count: u32, mem_words: u32) -> impl Iterator<Item = (u32, u32)> {
    let mut done = 0;
    std::iter::from_fn(move || {
        (done < count).then(|| {
            let n = (count - done).min(RMEM_CHUNK);
            let addr = ((u64::from(start) + u64::from(done)) % u64::from(mem_words)) as u32;
            done += n;
            (addr, n)
        })
    })
}

/// One of the capture clocks a device has.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockSetting {
    /// The value of `CLOCK` that gives it; none on a device whose clock is
    /// not the host's to change.
    pub select: Option<u32>,
    /// What the device said it is, in Hz.
    pub hz: u32,
}

/// A sample rate as the device is set to it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rate {
    /// The clock it is a division of.
    pub clock: ClockSetting,
    /// One sample every this many clocks, 1 or more: DIV + 1.
    pub divider: u32,
    /// Two samples per clock, with a divider of 1 only.
    pub ddr: bool,
}

impl Rate {
    /// The rate in Hz, to the nearest.
    pub fn hz(&self) -> u64 {
        let clock = u64::from(self.clock.hz);
        if self.ddr {
            return clock * 2;
        }
        let d = u64::from(self.divider.max(1));
        (clock + d / 2) / d
    }
}

/// The capture clocks of a device, as it reported them: the one place
/// rates come from. A rate is a clock times two, or a clock divided by 1 to
/// 2^24; which of these a number of Hz is depends on what the device says
/// its clocks are, so the driver keeps a [`Rate`] and asks it for its Hz.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Clocks {
    /// Rising, each clock once.
    settings: Vec<ClockSetting>,
    ddr: bool,
    min_divider: u32,
}

impl Clocks {
    /// The clocks of the device of `id`: those it answered with for each
    /// value of `CLOCK`, or with none of them the one of its ID.
    pub fn new(id: &Id, mut settings: Vec<ClockSetting>) -> Clocks {
        if settings.is_empty() {
            settings.push(ClockSetting { select: None, hz: id.clock_hz });
        }
        settings.sort_by_key(|c| c.hz);
        settings.dedup_by_key(|c| c.hz);
        Clocks {
            settings,
            ddr: id.has_ddr(),
            // An ADC's clock is high for the first half of a sample period,
            // which takes two clocks at the least.
            min_divider: if id.is_adc() { 2 } else { 1 },
        }
    }

    /// The clocks of a device that describes itself: those it lists, and
    /// the lowest divider it says it takes.
    pub fn described(id: &Id, description: &Description) -> Clocks {
        let mut clocks = Clocks::new(id, description.clock_settings());
        clocks.min_divider = u32::from(description.div_min) + 1;
        clocks
    }

    /// The clocks, rising.
    pub fn settings(&self) -> &[ClockSetting] {
        &self.settings
    }

    /// The highest rate.
    pub fn fastest(&self) -> Rate {
        let clock = *self.settings.last().expect("a clock");
        Rate { clock, divider: self.min_divider, ddr: self.ddr }
    }

    /// The lowest rate.
    pub fn slowest(&self) -> Rate {
        Rate { clock: self.settings[0], divider: MAX_DIVIDER as u32, ddr: false }
    }

    /// The rate nearest to `hz`, or none when `hz` is beyond the highest
    /// or below the lowest. Of two rates that are as near, the higher; of
    /// two clocks that give it, the one of `now`, which needs no change,
    /// or else the lower.
    pub fn nearest(&self, hz: u64, now: Rate) -> Option<Rate> {
        if hz > self.fastest().hz() || hz < self.slowest().hz().max(1) {
            return None;
        }
        let min = u64::from(self.min_divider);
        let mut all = Vec::new();
        for clock in &self.settings {
            let low = (u64::from(clock.hz) / hz).clamp(min, MAX_DIVIDER);
            let high = (low + 1).min(MAX_DIVIDER);
            if self.ddr {
                all.push(Rate { clock: *clock, divider: 1, ddr: true });
            }
            all.extend([low, high].map(|d| Rate { clock: *clock, divider: d as u32, ddr: false }));
        }
        all.into_iter().min_by_key(|r| {
            (r.hz().abs_diff(hz), u64::MAX - r.hz(), r.clock != now.clock, r.clock.hz)
        })
    }
}

/// One trigger term: a condition per channel, all of which must hold on a
/// sample. A term with nothing set matches every sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Term {
    /// Channels that must equal their bit of `value`.
    pub mask: u32,
    /// The levels of the channels in `mask`.
    pub value: u32,
    /// Channels that must be 1 after 0.
    pub rise: u32,
    /// Channels that must be 0 after 1.
    pub fall: u32,
    /// Channels that must differ from the sample before.
    pub edge: u32,
}

impl Term {
    /// The five registers, in their order from the term's base address.
    pub fn registers(&self) -> [u32; 5] {
        [self.mask, self.value, self.rise, self.fall, self.edge]
    }
}

/// `TMODE`: every sample is the event, so a capture triggers at once.
#[allow(dead_code)]
pub const TMODE_NOW: u32 = 0;
/// `TMODE`: term A.
pub const TMODE_A: u32 = 1;
/// `TMODE`: term B.
#[allow(dead_code)]
pub const TMODE_B: u32 = 2;
/// `TMODE`: A or B.
pub const TMODE_A_OR_B: u32 = 3;
/// `TMODE`: A and B on the same sample.
#[allow(dead_code)]
pub const TMODE_A_AND_B: u32 = 4;
/// `TMODE`: A, then B on a later sample.
pub const TMODE_A_THEN_B: u32 = 5;
/// `TMODE`: B, then A on a later sample.
#[allow(dead_code)]
pub const TMODE_B_THEN_A: u32 = 6;

/// The trigger registers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TriggerSetup {
    /// `TMODE`.
    pub mode: u32,
    /// `TCOUNT`: the trigger is the event of this number, from 0.
    pub count: u32,
    /// Term A.
    pub a: Term,
    /// Term B.
    pub b: Term,
}

/// A stage as a term.
fn term(
    stage: &crate::core::TriggerStage,
    channels: u8,
    enabled: u32,
    n: usize,
) -> Result<Term, String> {
    let mut t = Term::default();
    let mut low = 0;
    for c in &stage.conditions {
        if c.channel >= usize::from(channels) {
            return Err(format!("Trigger stage {n} names channel {}, which is none.", c.channel));
        }
        let bit = 1 << c.channel;
        if enabled & bit == 0 {
            return Err(format!("Trigger stage {n} is on a disabled channel."));
        }
        match c.matches {
            TriggerMatch::Zero => low |= bit,
            TriggerMatch::One => t.value |= bit,
            TriggerMatch::Rising => t.rise |= bit,
            TriggerMatch::Falling => t.fall |= bit,
            TriggerMatch::Edge => t.edge |= bit,
            TriggerMatch::Over | TriggerMatch::Under => {
                return Err("Analog trigger matches are not supported.".to_owned());
            }
        }
    }
    if low & t.value != 0 {
        return Err(format!("Trigger stage {n} wants a channel both low and high."));
    }
    t.mask = low | t.value;
    Ok(t)
}

/// The registers for `trigger`: one stage is term A, two are A then B or,
/// combined with or, A or B. A count is the device's event count, which it
/// has for the trigger as a whole, so it goes with one stage only.
///
/// # Errors
///
/// What the device cannot do of it, for the user.
pub fn trigger_setup(
    trigger: Option<&Trigger>,
    channels: u8,
    enabled: u32,
) -> Result<TriggerSetup, String> {
    let Some(trigger) = trigger else {
        return Ok(TriggerSetup::default());
    };
    let stage = |i: usize| term(&trigger.stages[i], channels, enabled, i + 1);
    let stages = trigger.stages.len();
    if trigger.stages.iter().any(|s| s.duration.is_some() || s.range.is_some()) {
        return Err("Trigger durations and value ranges are not supported.".to_owned());
    }
    if trigger.holdoff > 0 {
        return Err("A trigger hold-off is not supported.".to_owned());
    }
    let counted = trigger.stages.iter().filter(|s| s.count.is_some()).count();
    if counted > 0 && stages != 1 {
        return Err("A trigger count needs a trigger of one stage: the device counts \
                    the whole trigger, not its stages."
            .to_owned());
    }
    match (stages, trigger.combine) {
        (1, _) => {
            let count = match trigger.stages[0].count {
                None => 0,
                Some(c) if c.consecutive => {
                    return Err("Consecutive trigger counts are not supported.".to_owned());
                }
                Some(c) if (1..=0x1_0000).contains(&c.count) => c.count - 1,
                Some(c) => {
                    return Err(format!("Trigger count {} is not within 1..65536.", c.count));
                }
            };
            Ok(TriggerSetup { mode: TMODE_A, count, a: stage(0)?, b: Term::default() })
        }
        (2, TriggerCombine::Sequence) => {
            Ok(TriggerSetup { mode: TMODE_A_THEN_B, count: 0, a: stage(0)?, b: stage(1)? })
        }
        (2, TriggerCombine::Or) => {
            Ok(TriggerSetup { mode: TMODE_A_OR_B, count: 0, a: stage(0)?, b: stage(1)? })
        }
        (2, TriggerCombine::And) => Err("Trigger stages combined with and are not supported: \
                                         the device's and is both terms on one sample, which \
                                         one stage says."
            .to_owned()),
        _ => Err("Only 2 trigger stages are supported.".to_owned()),
    }
}

/// What a capture is asked to be.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// The sample rate.
    pub rate: Rate,
    /// Samples wanted in all, 1 or more.
    pub limit_samples: u64,
    /// The share of them wanted before the trigger, in percent.
    pub capture_ratio: u64,
    /// Run length encoding.
    pub rle: bool,
    /// The counter instead of the inputs.
    pub test_pattern: bool,
    /// The enabled channels.
    pub channel_mask: u32,
}

/// A capture: the clock, the registers to write and what to take of the
/// result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Plan {
    /// Address and value of each register but `CLOCK`, in the order they
    /// go out, between the RESET and the ARM.
    pub registers: Vec<(u8, u32)>,
    /// Samples wanted before the trigger sample.
    pub pre: u64,
    /// Samples wanted from the trigger sample on.
    pub post: u64,
    /// The words are runs.
    pub rle: bool,
    /// The user set a trigger, so its place gets a marker.
    pub marked: bool,
    /// The enabled channels. The device's counter pattern does not go
    /// through its channel mask, so the others are cleared here.
    pub channel_mask: u32,
    /// The clock the registers are for, which the device must be at.
    pub clock: ClockSetting,
}

/// The registers of a capture of `s` with `trigger`.
///
/// The trigger is looked for from the first sample on, as this codebase's
/// triggers are: one that comes early has fewer samples ahead of it than
/// the capture ratio asks for, and the capture is that much shorter. With
/// no trigger the first sample is the trigger sample.
///
/// # Errors
///
/// What the device cannot do of it, for the user.
pub fn plan(id: &Id, s: &Settings, trigger: Option<&Trigger>) -> Result<Plan, String> {
    let setup = trigger_setup(trigger, id.channels, s.channel_mask)?;
    if s.channel_mask & id.channel_mask() == 0 {
        return Err("No channel is enabled.".to_owned());
    }
    if s.rle && !id.has_rle() {
        return Err("This bitstream has no run length encoding.".to_owned());
    }
    if s.rle && s.rate.ddr {
        return Err(format!(
            "Run length encoding takes one sample per clock: {} Hz at the most.",
            s.rate.clock.hz
        ));
    }
    let max = if s.rle { u64::from(u32::MAX) } else { id.raw_capacity() };
    if s.limit_samples == 0 || s.limit_samples > max {
        return Err(format!("A capture is 1 to {max} samples, not {}.", s.limit_samples));
    }
    let pre = match trigger {
        // The trigger sample itself is after the trigger.
        Some(_) => (s.limit_samples * s.capture_ratio.min(100) / 100).min(s.limit_samples - 1),
        None => 0,
    };
    let post = s.limit_samples - pre;
    let last = id.mem_words - 1;
    // Runs before the trigger take a word each at the worst, and what
    // follows must not run over them.
    let kept =
        if s.rle { pre.min(u64::from(last) * s.capture_ratio.min(100) / 100) as u32 } else { 0 };
    let mut flags = 0;
    for (on, flag) in [(s.rle, FLAG_RLE), (s.rate.ddr, FLAG_DDR), (s.test_pattern, FLAG_TEST)] {
        if on {
            flags |= flag;
        }
    }
    let mut registers = vec![
        (REG_DIV, s.rate.divider.max(1) - 1),
        (REG_FLAGS, flags),
        (REG_CHMASK, s.channel_mask & id.channel_mask()),
        (REG_TMODE, setup.mode),
        (REG_TCOUNT, setup.count),
    ];
    for (base, term) in [(REG_TERM_A, setup.a), (REG_TERM_B, setup.b)] {
        registers.extend((base..).zip(term.registers()));
    }
    registers.extend([
        (REG_PRE_WORDS, 0),
        (REG_POST_WORDS, last - kept),
        (REG_POST_SAMPLES, (post - 1) as u32),
    ]);
    let channel_mask = s.channel_mask & id.channel_mask();
    let (marked, clock) = (trigger.is_some(), s.rate.clock);
    Ok(Plan { registers, pre, post, rle: s.rle, marked, channel_mask, clock })
}

/// What the device says of a capture that ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ended {
    /// `STATUS`.
    pub status: u32,
    /// `TRIG`.
    pub trig: u32,
    /// `STOP`.
    pub stop: u32,
    /// `COUNT`.
    pub count: u32,
}

impl Ended {
    /// The trigger was seen.
    pub fn triggered(&self) -> bool {
        self.status & STATUS_TRIGGERED != 0
    }

    /// The memory could not keep up.
    pub fn overflowed(&self) -> bool {
        self.status & STATUS_OVERFLOW != 0
    }

    /// The real bits of the last word written.
    pub fn last_bits(&self) -> u32 {
        ((self.status >> 8) & 0x3f).min(32)
    }
}

/// The part of the memory a capture's samples are in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window {
    /// The address of its first word.
    pub start: u32,
    /// Its length in words, up to the last word written.
    pub words: u32,
    /// Which of its words has the trigger sample, and which of the samples
    /// that start in that word it is; none without a trigger.
    pub trigger: Option<(u32, u32)>,
}

/// The words to read for `plan` after a capture that `ended` so: from far
/// enough ahead of the trigger for the samples wanted before it, to the
/// end. A capture stopped before its trigger is its last words.
pub fn window(id: &Id, plan: &Plan, ended: &Ended) -> Window {
    let mem = id.mem_words;
    let back = |from: u32, by: u32| (from % mem + mem - by) % mem;
    let stop = ended.stop & ADDR_MASK;
    let have = ended.count.min(mem);
    let w = u64::from(id.channels);
    // The words that hold this many samples, at one per run or as bits,
    // where one more word has the sample that straddles into the first,
    // and another is the last of a stopped capture, which may be empty.
    let words_for = |samples: u64| -> u32 {
        let n = if plan.rle { samples } else { (samples * w).div_ceil(32) + 2 };
        u32::try_from(n).unwrap_or(u32::MAX)
    };
    let trig = ended.trig & ADDR_MASK;
    // Words from the oldest to the trigger's.
    let ahead = (trig + mem - back(stop, have)) % mem;
    if ended.triggered() && ahead < have {
        let kept = ahead.min(words_for(plan.pre));
        Window {
            start: back(trig, kept),
            words: kept + (have - ahead),
            trigger: Some((kept, (ended.trig >> 24) & 3)),
        }
    } else {
        let words = have.min(words_for(plan.pre + plan.post));
        Window { start: back(stop, words), words, trigger: None }
    }
}

/// How many addresses it takes for samples to start at the same bits
/// again: 1 when the channels divide a word, 7 with 14 of them.
fn group(channels: u8) -> u32 {
    let w = u32::from(channels);
    let mut common = 1;
    while common < 32 && w.is_multiple_of(common * 2) {
        common *= 2;
    }
    w / common
}

/// The bit of the word at `addr` at which the first sample that starts in
/// it starts: `(W - 32a mod W) mod W`.
pub fn first_start(addr: u32, channels: u8) -> u32 {
    let w = u32::from(channels);
    (w - (32 * (addr % group(channels))) % w) % w
}

/// The samples of raw `words`, which were read from `addr` on and whose
/// last word has `last_bits` real bits: those that start in them and end
/// in them.
pub fn unpack_raw(words: &[u32], addr: u32, channels: u8, last_bits: u32) -> Vec<u16> {
    let w = u32::from(channels);
    let mask = (1u64 << w) - 1;
    let skip = first_start(addr, channels);
    let bits = (words.len() as u64).saturating_sub(1) * 32 + u64::from(last_bits.min(32));
    let total =
        if words.is_empty() { 0 } else { bits.saturating_sub(u64::from(skip)) / u64::from(w) };
    let mut samples = Vec::with_capacity(total as usize);
    let (mut acc, mut have) = (0u64, 0u32);
    for (i, word) in words.iter().enumerate() {
        acc |= u64::from(*word) << have;
        have += 32;
        if i == 0 {
            acc >>= skip;
            have -= skip;
        }
        while have >= w && (samples.len() as u64) < total {
            samples.push((acc & mask) as u16);
            acc >>= w;
            have -= w;
        }
    }
    samples
}

/// Which of the samples [`unpack_raw`] gives for a read from `addr` on is
/// the trigger sample, the one of number `sub` to start in the word
/// `word` words into the read.
pub fn raw_trigger_index(addr: u32, word: u32, sub: u32, channels: u8) -> u64 {
    let w = u64::from(channels);
    let at = 32 * u64::from(word) + u64::from(first_start(addr.wrapping_add(word), channels));
    (at - u64::from(first_start(addr, channels))) / w + u64::from(sub)
}

/// A word of an RLE capture: the sample and how many times it was seen.
pub fn rle_run(word: u32, channels: u8) -> (u16, u64) {
    let w = u32::from(channels);
    ((word & ((1 << w) - 1)) as u16, (u64::from(word) >> w) + 1)
}

/// The runs of the RLE `words` of a whole capture whose last word has
/// `last_bits` real bits. A capture stopped before its first sample still
/// writes its run in progress, with none: a word that is no run.
pub fn rle_runs(words: &[u32], last_bits: u32) -> &[u32] {
    match words {
        [runs @ .., _] if last_bits == 0 => runs,
        _ => words,
    }
}

/// The samples of RLE `words`.
pub fn rle_len(words: &[u32], channels: u8) -> u64 {
    words.iter().map(|x| rle_run(*x, channels).1).sum()
}

/// What to pass on of the samples read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cut {
    /// Samples to drop at the front.
    pub skip: u64,
    /// Samples to pass on after those.
    pub take: u64,
    /// Which of the samples passed on is the trigger sample.
    pub trigger: Option<u64>,
}

/// The cut of `total` samples with the trigger sample at `trigger`: `pre`
/// samples ahead of it and `post` from it on, or what there is of them.
/// Without a trigger, the last `pre + post`.
pub fn cut(total: u64, trigger: Option<u64>, pre: u64, post: u64) -> Cut {
    match trigger {
        Some(at) if at < total => {
            let skip = at - pre.min(at);
            Cut { skip, take: (at - skip + post).min(total - skip), trigger: Some(at - skip) }
        }
        _ => {
            let take = total.min(pre + post);
            Cut { skip: total - take, take, trigger: None }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::TriggerCount;

    fn id(channels: u8, per_clock: u8, caps: u8, clock: u32, mem: u32) -> Id {
        let mut d = b"OXLA".to_vec();
        d.extend([1, caps, channels, per_clock]);
        d.extend(clock.to_le_bytes());
        d.extend(mem.to_le_bytes());
        Id::parse(&d).unwrap()
    }

    /// `la16-rle` at 108 MHz.
    fn la16() -> Id {
        id(16, 1, 1, 108_000_000, 2_097_152)
    }

    /// `la14-ddr` at 135 MHz.
    fn la14() -> Id {
        id(14, 2, 2, 135_000_000, 2_097_151)
    }

    /// `la8-adc` at 54 MHz.
    fn la8_adc() -> Id {
        id(8, 1, 4, 54_000_000, 2_097_152)
    }

    #[test]
    fn the_id_says_what_the_device_is() {
        // As a board with la16-rle answers at 108 MHz.
        let d =
            [0x4f, 0x58, 0x4c, 0x41, 1, 1, 16, 1, 0x00, 0xf3, 0x6f, 0x06, 0x00, 0x00, 0x20, 0x00];
        let id = Id::parse(&d).unwrap();
        assert_eq!((id.channels, id.samples_per_clock), (16, 1));
        assert_eq!((id.clock_hz, id.mem_words), (108_000_000, 2_097_152));
        assert!(id.has_rle() && !id.has_ddr() && !id.is_adc());
        assert_eq!((id.unit_size(), id.channel_mask()), (2, 0xffff));
        assert_eq!(id.raw_capacity(), 4_194_304);
        assert_eq!(id.variant(), "la16-rle");

        let id = la14();
        assert!(!id.has_rle() && id.has_ddr());
        assert_eq!((id.unit_size(), id.channel_mask()), (2, 0x3fff));
        assert_eq!(id.raw_capacity(), 4_793_488);
        assert_eq!(id.variant(), "la14-ddr");
        assert_eq!(la8_adc().variant(), "la8-adc");
        assert_eq!(id8(1, 1).unit_size(), 1);
        assert_eq!(id8(1, 1).raw_capacity(), 8_388_608);
    }

    fn id8(per_clock: u8, caps: u8) -> Id {
        id(8, per_clock, caps, 108_000_000, 2_097_152)
    }

    #[test]
    fn an_id_that_is_not_one_is_refused() {
        let good = [b'O', b'X', b'L', b'A', 1, 0, 16, 1, 0, 0xf3, 0x6f, 6, 0, 0, 0x20, 0];
        assert!(Id::parse(&good).is_ok());
        let with = |at: usize, v: u8| {
            let mut d = good;
            d[at] = v;
            Id::parse(&d)
        };
        assert!(with(0, b'1').unwrap_err().starts_with("not an oxla ID: 31 58"));
        assert!(Id::parse(&good[..15]).is_err());
        assert!(with(4, 2).unwrap_err().contains("version 2"));
        assert!(with(6, 0).is_err());
        assert!(with(6, 17).is_err());
        assert!(with(7, 0).is_err());
        assert!(with(7, 3).is_err());
        // No memory, more than an address reaches, and a ring of 14 bit
        // samples that is no multiple of 7 words.
        assert!(with(14, 0).is_err());
        assert!(with(14, 0x21).is_err());
        assert!(with(6, 14).is_err());
        let mut clockless = good;
        clockless[8..12].fill(0);
        assert!(Id::parse(&clockless).is_err());
    }

    #[test]
    fn commands_are_little_endian() {
        assert_eq!(wreg(REG_POST_SAMPLES, 0x1234_5678), [3, 3, 0x78, 0x56, 0x34, 0x12]);
        assert_eq!(rreg(REG_STATUS), [4, 0x80]);
        assert_eq!(rmem(0x12_3456, 2048), [7, 0x56, 0x34, 0x12, 0x00, 0x08, 0x00]);
        assert_eq!(RESYNC, [0, 0, 0, 0, 0, 0, 0, 0, 1]);
    }

    #[test]
    fn memory_is_read_in_chunks_that_wrap() {
        let chunks: Vec<_> = rmem_chunks(10, 5000, 1 << 21).collect();
        assert_eq!(chunks, [(10, 2048), (2058, 2048), (4106, 904)]);
        // Across the end of a ring of 4000 words.
        let chunks: Vec<_> = rmem_chunks(3000, 4000, 4000).collect();
        assert_eq!(chunks, [(3000, 2048), (1048, 1952)]);
        assert_eq!(rmem_chunks(5, 0, 100).count(), 0);
        assert_eq!(rmem_chunks(0, 2048, 4096).collect::<Vec<_>>(), [(0, 2048)]);
    }

    /// A device with the one clock of its ID.
    fn fixed(id: &Id) -> Clocks {
        Clocks::new(id, Vec::new())
    }

    /// A device that answered 81, 108 and 135 MHz, in the order asked.
    fn three(id: &Id) -> Clocks {
        let hz = [81_000_000, 108_000_000, 135_000_000];
        Clocks::new(id, (0..).zip(hz).map(|(i, hz)| setting(Some(i), hz)).collect())
    }

    fn setting(select: Option<u32>, hz: u32) -> ClockSetting {
        ClockSetting { select, hz }
    }

    #[test]
    fn rates_come_from_the_clock_the_device_reports() {
        let c = fixed(&la16());
        let clock = setting(None, 108_000_000);
        assert_eq!(c.settings(), [clock]);
        let rate = |divider| Rate { clock, divider, ddr: false };
        let hz = |divider| rate(divider).hz();
        let now = c.fastest();
        assert_eq!(now, rate(1));
        assert_eq!((hz(1), hz(2), hz(108)), (108_000_000, 54_000_000, 1_000_000));
        // 108 MHz / 11 is 9818181.8 Hz.
        assert_eq!(hz(11), 9_818_182);
        assert_eq!(c.nearest(1_000_000, now), Some(rate(108)));
        assert_eq!(c.nearest(10_000_000, now), Some(rate(11)));
        assert_eq!(c.nearest(108_000_000, now), Some(rate(1)));
        assert_eq!(c.nearest(100_000_000, now), Some(rate(1)));
        assert_eq!(c.nearest(70_000_000, now), Some(rate(2)));
        assert_eq!(c.nearest(108_000_001, now), None);
        // 108 MHz / 2^24 is 6.4 Hz.
        assert_eq!(c.slowest().hz(), 6);
        assert_eq!(c.nearest(6, now), Some(rate(1 << 24)));
        assert_eq!(c.nearest(5, now), None);
        assert_eq!(c.nearest(7, now), Some(rate(15_428_571)));

        // Another clock, the same code: nothing is a table.
        let c = fixed(&id(16, 1, 0, 50_000_000, 1 << 21));
        let got = c.nearest(1_000_000, c.fastest()).unwrap();
        assert_eq!((got.clock.hz, got.divider), (50_000_000, 50));
    }

    #[test]
    fn the_clock_is_the_one_that_gives_the_rate() {
        let c = three(&la16());
        let s = c.settings();
        assert_eq!(
            s.iter().map(|c| c.hz).collect::<Vec<_>>(),
            [81_000_000, 108_000_000, 135_000_000]
        );
        let rate = |i: usize, divider| Rate { clock: s[i], divider, ddr: false };
        assert_eq!(c.fastest(), rate(2, 1));
        assert_eq!(c.fastest().hz(), 135_000_000);
        assert_eq!(c.slowest(), rate(0, 1 << 24));
        // 81 MHz / 2^24 is 4.8 Hz.
        assert_eq!(c.slowest().hz(), 5);
        let now = rate(1, 1);
        for (hz, want) in [
            (135_000_000, rate(2, 1)),
            (108_000_000, rate(1, 1)),
            (81_000_000, rate(0, 1)),
            (67_500_000, rate(2, 2)),
            (40_500_000, rate(0, 2)),
            // 81 MHz / 8 is 10.125 MHz, nearer than 108 / 11 and 135 / 13.
            (10_000_000, rate(0, 8)),
            (5_000_000, rate(2, 27)),
            // Every clock has it: the one the device is at.
            (27_000_000, rate(1, 4)),
            (1_000_000, rate(1, 108)),
            (5, rate(0, 16_200_000)),
        ] {
            assert_eq!(c.nearest(hz, now), Some(want), "{hz} Hz");
        }
        // At another clock, that one.
        assert_eq!(c.nearest(27_000_000, rate(2, 1)), Some(rate(2, 5)));
        assert_eq!(c.nearest(27_000_000, rate(0, 7)), Some(rate(0, 3)));
        assert_eq!(c.nearest(13_500_000, rate(1, 1)), Some(rate(1, 8)));
        assert_eq!(c.nearest(45_000_000, rate(1, 1)), Some(rate(2, 3)));
        assert_eq!(c.nearest(135_000_001, now), None);
        assert_eq!(c.nearest(4, now), None);
    }

    #[test]
    fn clocks_are_kept_once_each_and_rising() {
        let id = la16();
        let c = Clocks::new(
            &id,
            vec![
                setting(Some(0), 135_000_000),
                setting(Some(1), 81_000_000),
                setting(Some(2), 81_000_000),
            ],
        );
        assert_eq!(c.settings(), [setting(Some(1), 81_000_000), setting(Some(0), 135_000_000)]);
        assert!(id.has_clocks() || fixed(&id).settings()[0].select.is_none());
        let with = Id { caps: 9, ..id };
        assert!(with.has_clocks() && with.has_rle() && !with.has_ddr());
        assert!(with.same_board(&Id { clock_hz: 81_000_000, ..with }));
        assert!(!with.same_board(&id) && !with.same_board(&Id { channels: 8, ..with }));
    }

    #[test]
    fn two_samples_per_clock_is_the_rate_above_the_clock() {
        let c = fixed(&la14());
        let clock = setting(None, 135_000_000);
        let (ddr, whole) =
            (Rate { clock, divider: 1, ddr: true }, Rate { clock, divider: 1, ddr: false });
        assert_eq!(c.fastest(), ddr);
        assert_eq!(c.fastest().hz(), 270_000_000);
        assert_eq!(c.nearest(270_000_000, ddr), Some(ddr));
        assert_eq!(c.nearest(135_000_000, ddr), Some(whole));
        assert_eq!(c.nearest(250_000_000, ddr), Some(ddr));
        assert_eq!(c.nearest(150_000_000, ddr), Some(whole));
        assert_eq!(c.nearest(270_000_001, ddr), None);
        // With three clocks: twice each of them.
        let c = three(&la14());
        let s = c.settings();
        let twice = |i: usize| Rate { clock: s[i], divider: 1, ddr: true };
        assert_eq!(c.nearest(216_000_000, ddr), Some(twice(1)));
        assert_eq!(c.nearest(162_000_000, ddr), Some(twice(0)));
        assert_eq!(c.nearest(200_000_000, ddr), Some(twice(1)));
        assert_eq!(
            c.nearest(140_000_000, ddr).map(|r| (r.hz(), r.ddr)),
            Some((135_000_000, false))
        );
    }

    #[test]
    fn an_adc_takes_two_clocks_a_sample() {
        let c = fixed(&la8_adc());
        let clock = setting(None, 54_000_000);
        let top = Rate { clock, divider: 2, ddr: false };
        assert_eq!(c.fastest(), top);
        assert_eq!(top.hz(), 27_000_000);
        assert_eq!(c.nearest(54_000_000, top), None);
        assert_eq!(c.nearest(27_000_000, top), Some(top));
        assert_eq!(three(&la8_adc()).fastest().hz(), 67_500_000);
    }

    fn trig(stages: &[&[(usize, TriggerMatch)]]) -> Trigger {
        let mut t = Trigger::new();
        for (i, stage) in stages.iter().enumerate() {
            for (ch, m) in *stage {
                t.add(i, *ch, *m);
            }
        }
        t
    }

    #[test]
    fn a_stage_is_a_term() {
        use TriggerMatch::{Edge, Falling, One, Rising, Zero};
        let t = trig(&[&[(0, Rising), (3, One), (4, Zero), (5, Falling), (15, Edge)]]);
        let s = trigger_setup(Some(&t), 16, 0xffff).unwrap();
        assert_eq!(s.mode, TMODE_A);
        assert_eq!(s.count, 0);
        assert_eq!(s.a, Term { mask: 0x18, value: 0x08, rise: 1, fall: 0x20, edge: 0x8000 });
        assert_eq!(s.a.registers(), [0x18, 0x08, 1, 0x20, 0x8000]);
        assert_eq!(s.b, Term::default());
        assert_eq!(trigger_setup(None, 16, 0xffff).unwrap().mode, TMODE_NOW);
    }

    #[test]
    fn two_stages_are_a_then_b_or_a_or_b() {
        use TriggerMatch::{Falling, Rising};
        let mut t = trig(&[&[(5, Rising)], &[(3, Falling)]]);
        let s = trigger_setup(Some(&t), 16, 0xffff).unwrap();
        assert_eq!((s.mode, s.a.rise, s.b.fall), (TMODE_A_THEN_B, 0x20, 8));
        t.combine = TriggerCombine::Or;
        let s = trigger_setup(Some(&t), 16, 0xffff).unwrap();
        assert_eq!((s.mode, s.a.rise, s.b.fall), (TMODE_A_OR_B, 0x20, 8));
        t.combine = TriggerCombine::And;
        assert!(trigger_setup(Some(&t), 16, 0xffff).unwrap_err().contains("and"));
        let three = trig(&[&[(0, Rising)], &[(1, Rising)], &[(2, Rising)]]);
        assert!(trigger_setup(Some(&three), 16, 0xffff).is_err());
    }

    #[test]
    fn a_count_is_the_event_count_of_one_stage() {
        use TriggerMatch::{Edge, Rising};
        let count = |count| TriggerCount { count, consecutive: false };
        let mut t = trig(&[&[(2, Edge)]]);
        t.set_count(0, count(1000));
        assert_eq!(trigger_setup(Some(&t), 16, 0xffff).unwrap().count, 999);
        t.set_count(0, count(65_536));
        assert_eq!(trigger_setup(Some(&t), 16, 0xffff).unwrap().count, 0xffff);
        t.set_count(0, count(65_537));
        assert!(trigger_setup(Some(&t), 16, 0xffff).is_err());
        t.set_count(0, TriggerCount { count: 2, consecutive: true });
        assert!(trigger_setup(Some(&t), 16, 0xffff).is_err());
        let mut t = trig(&[&[(0, Rising)], &[(1, Rising)]]);
        t.set_count(1, count(2));
        assert!(trigger_setup(Some(&t), 16, 0xffff).unwrap_err().contains("one stage"));
    }

    #[test]
    fn a_trigger_the_device_cannot_match_is_refused() {
        use TriggerMatch::{One, Over, Rising, Zero};
        let setup = |t: &Trigger, channels, enabled| trigger_setup(Some(t), channels, enabled);
        assert!(setup(&trig(&[&[(14, Rising)]]), 14, 0x3fff).is_err());
        assert!(setup(&trig(&[&[(13, Rising)]]), 14, 0x3fff).is_ok());
        assert!(setup(&trig(&[&[(3, Rising)]]), 16, 0xfff7).unwrap_err().contains("disabled"));
        assert!(setup(&trig(&[&[(3, One), (3, Zero)]]), 16, 0xffff).is_err());
        assert!(setup(&trig(&[&[(3, Over)]]), 16, 0xffff).is_err());
        let mut held = trig(&[&[(3, One)]]);
        held.holdoff = 10;
        assert!(setup(&held, 16, 0xffff).is_err());
    }

    const CLOCK: ClockSetting = ClockSetting { select: Some(1), hz: 108_000_000 };

    fn settings(limit_samples: u64) -> Settings {
        Settings {
            rate: Rate { clock: CLOCK, divider: 1, ddr: false },
            limit_samples,
            capture_ratio: 0,
            rle: false,
            test_pattern: false,
            channel_mask: 0xffff,
        }
    }

    fn register(plan: &Plan, addr: u8) -> u32 {
        let found: Vec<u32> =
            plan.registers.iter().filter(|(a, _)| *a == addr).map(|(_, v)| *v).collect();
        assert_eq!(found.len(), 1, "register {addr:#x} once");
        found[0]
    }

    #[test]
    fn a_capture_with_no_trigger_starts_at_the_first_sample() {
        let p = plan(&la16(), &settings(1000), None).unwrap();
        let addrs: Vec<u8> = p.registers.iter().map(|(a, _)| *a).collect();
        assert_eq!(
            addrs,
            [0, 4, 5, 6, 7, 0x10, 0x11, 0x12, 0x13, 0x14, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 2, 1, 3]
        );
        assert_eq!(register(&p, REG_DIV), 0);
        assert_eq!(register(&p, REG_FLAGS), 0);
        assert_eq!(register(&p, REG_CHMASK), 0xffff);
        assert_eq!(register(&p, REG_TMODE), TMODE_NOW);
        assert_eq!(register(&p, REG_PRE_WORDS), 0);
        assert_eq!(register(&p, REG_POST_WORDS), 2_097_151);
        // The trigger sample and 999 after it.
        assert_eq!(register(&p, REG_POST_SAMPLES), 999);
        assert_eq!((p.pre, p.post, p.rle, p.marked), (0, 1000, false, false));
        assert_eq!((p.channel_mask, p.clock), (0xffff, CLOCK));
        assert!(!addrs.contains(&REG_CLOCK), "it has a way of its own");
    }

    #[test]
    fn the_capture_ratio_is_the_share_ahead_of_the_trigger() {
        let t = trig(&[&[(5, TriggerMatch::Rising)]]);
        let mut s = settings(1000);
        s.capture_ratio = 30;
        s.rate = Rate { clock: CLOCK, divider: 4, ddr: false };
        s.test_pattern = true;
        s.channel_mask = 0x00ff;
        let p = plan(&la16(), &s, Some(&t)).unwrap();
        assert_eq!((p.pre, p.post, p.marked), (300, 700, true));
        assert_eq!(register(&p, REG_DIV), 3);
        assert_eq!(register(&p, REG_FLAGS), FLAG_TEST);
        assert_eq!(register(&p, REG_CHMASK), 0xff);
        assert_eq!(register(&p, REG_TMODE), TMODE_A);
        assert_eq!(register(&p, REG_TERM_A + 2), 0x20);
        assert_eq!(register(&p, REG_POST_SAMPLES), 699);
        // Without a trigger there is nothing to be ahead of.
        assert_eq!(plan(&la16(), &s, None).unwrap().pre, 0);
        // All of it ahead still leaves the trigger sample.
        s.capture_ratio = 100;
        let p = plan(&la16(), &s, Some(&t)).unwrap();
        assert_eq!((p.pre, p.post), (999, 1));
        assert_eq!(register(&p, REG_POST_SAMPLES), 0);
    }

    #[test]
    fn rle_keeps_room_for_the_runs_ahead_of_the_trigger() {
        let t = trig(&[&[(5, TriggerMatch::Rising)]]);
        let mut s = settings(1000);
        s.rle = true;
        s.capture_ratio = 10;
        let p = plan(&la16(), &s, Some(&t)).unwrap();
        assert_eq!(register(&p, REG_FLAGS), FLAG_RLE);
        assert_eq!(register(&p, REG_POST_WORDS), 2_097_151 - 100);
        assert_eq!(register(&p, REG_POST_SAMPLES), 899);
        assert!(p.rle);
        // A capture longer than the memory has words: its share of them.
        s.limit_samples = 4_000_000_000;
        let p = plan(&la16(), &s, Some(&t)).unwrap();
        assert_eq!(register(&p, REG_POST_WORDS), 2_097_151 - 209_715);
        assert_eq!(register(&p, REG_POST_SAMPLES), 3_599_999_999);
    }

    #[test]
    fn what_the_bitstream_cannot_do_is_refused() {
        let mut s = settings(1000);
        s.rle = true;
        assert!(plan(&id(16, 1, 0, 108_000_000, 1 << 21), &s, None).is_err(), "no RLE in it");
        let ddr = id(8, 2, 3, 108_000_000, 1 << 21);
        s.channel_mask = 0xff;
        s.rate = Rate { clock: CLOCK, divider: 1, ddr: true };
        assert!(plan(&ddr, &s, None).unwrap_err().contains("108000000 Hz"));
        s.rle = false;
        let p = plan(&ddr, &s, None).unwrap();
        assert_eq!(register(&p, REG_FLAGS), FLAG_DDR);
        assert_eq!(register(&p, REG_DIV), 0);

        let mut s = settings(0);
        assert!(plan(&la16(), &s, None).is_err());
        s.limit_samples = 4_194_304;
        assert!(plan(&la16(), &s, None).is_ok());
        s.limit_samples = 4_194_305;
        assert!(plan(&la16(), &s, None).unwrap_err().contains("4194304"));
        s.rle = true;
        assert!(plan(&la16(), &s, None).is_ok(), "runs hold more");
        s.limit_samples = 1 << 32;
        assert!(plan(&la16(), &s, None).is_err());
        let mut s = settings(10);
        s.channel_mask = 0;
        assert!(plan(&la16(), &s, None).is_err());
        s.channel_mask = 0x1_0000;
        assert!(plan(&la16(), &s, None).is_err(), "no such channel");
    }

    /// A counter of `channels` bits from `first` on, `n` samples, as the
    /// words of a raw capture whose first sample starts at bit 0 of word 0.
    fn pack(channels: u8, first: u64, n: u64) -> (Vec<u32>, u32) {
        let w = u32::from(channels);
        let mut words = Vec::new();
        let (mut acc, mut have) = (0u64, 0u32);
        for i in 0..n {
            acc |= ((first + i) & ((1 << w) - 1)) << have;
            have += w;
            if have >= 32 {
                words.push(acc as u32);
                acc >>= 32;
                have -= 32;
            }
        }
        let last = if have > 0 {
            words.push(acc as u32);
            have
        } else {
            32
        };
        (words, last)
    }

    fn counter(channels: u8, first: u64, n: u64) -> Vec<u16> {
        (first..first + n).map(|v| (v & ((1 << channels) - 1)) as u16).collect()
    }

    #[test]
    fn samples_start_where_the_address_says() {
        for a in 0..100 {
            assert_eq!(first_start(a, 16), 0);
            assert_eq!(first_start(a, 8), 0);
        }
        // 32a mod 14 bits of a sample are in the words before.
        let starts: Vec<u32> = (0..8).map(|a| first_start(a, 14)).collect();
        assert_eq!(starts, [0, 10, 6, 2, 12, 8, 4, 0]);
        // The same across the end of a ring that is a multiple of 7 words.
        assert_eq!(first_start(2_097_151, 14), 0);
        assert_eq!(first_start(2_097_150, 14), 4);
        assert_eq!((group(8), group(14), group(16), group(12)), (1, 7, 1, 3));
    }

    #[test]
    fn whole_words_of_eight_and_sixteen_channels() {
        let words = [0x0302_0100, 0x0706_0504];
        assert_eq!(unpack_raw(&words, 0, 8, 32), [0, 1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(unpack_raw(&words, 9, 16, 32), [0x0100, 0x0302, 0x0504, 0x0706]);
        // A last word with three samples of four, and with one of two.
        assert_eq!(unpack_raw(&words, 0, 8, 24), [0, 1, 2, 3, 4, 5, 6]);
        assert_eq!(unpack_raw(&words, 0, 16, 16), [0x0100, 0x0302, 0x0504]);
        assert_eq!(unpack_raw(&words, 0, 16, 0), [0x0100, 0x0302]);
        assert_eq!(unpack_raw(&[], 0, 16, 32), [0u16; 0]);
        let (words, last) = pack(8, 250, 10);
        assert_eq!(last, 16);
        assert_eq!(unpack_raw(&words, 0, 8, last), counter(8, 250, 10));
    }

    #[test]
    fn fourteen_channels_straddle_words() {
        // Sixteen samples fill seven words.
        let (words, last) = pack(14, 0, 16);
        assert_eq!((words.len(), last), (7, 32));
        assert_eq!(unpack_raw(&words, 0, 14, 32), counter(14, 0, 16));

        let (words, last) = pack(14, 16_000, 1000);
        assert_eq!(unpack_raw(&words, 0, 14, last), counter(14, 16_000, 1000));
        // Read from any address: the samples that start in the first word
        // read come first. Sample i starts at bit 14i.
        for from in 1..40usize {
            let first = (32 * from as u64).div_ceil(14);
            let got = unpack_raw(&words[from..], from as u32, 14, last);
            assert_eq!(got, counter(14, 16_000 + first, 1000 - first), "from word {from}");
        }
        // A read that ends before the capture does: whole words, and the
        // sample that runs into the next word is left out.
        for to in 1..40usize {
            let whole = 32 * to as u64 / 14;
            assert_eq!(unpack_raw(&words[..to], 0, 14, 32), counter(14, 16_000, whole));
        }
    }

    #[test]
    fn the_last_word_has_as_many_bits_as_the_status_says() {
        for n in 1..40u64 {
            let (words, last) = pack(14, 5, n);
            assert_eq!(unpack_raw(&words, 0, 14, last), counter(14, 5, n), "{n} samples");
        }
        // Three samples are 42 bits: 10 in the second word. Fewer bits
        // there are fewer samples, not a sample of what is not real.
        let (words, last) = pack(14, 5, 3);
        assert_eq!(last, 10);
        assert_eq!(unpack_raw(&words, 0, 14, 9), counter(14, 5, 2));
        // A window whose only word has less than the first sample's start.
        assert_eq!(unpack_raw(&[0xffff_ffff], 1, 14, 8), [0u16; 0]);
    }

    #[test]
    fn the_trigger_sample_is_counted_from_the_first_read() {
        // 16 channels, two samples a word: word 5, the second in it, in a
        // read that began three words earlier.
        assert_eq!(raw_trigger_index(100, 3, 1, 16), 7);
        assert_eq!(raw_trigger_index(100, 0, 0, 16), 0);
        assert_eq!(raw_trigger_index(0, 2, 3, 8), 11);
        // 14 channels: every sample of a counter, found where TRIG would
        // say it is, from any start of the read.
        let (words, last) = pack(14, 0, 200);
        for sample in 0..200u64 {
            let word = (14 * sample / 32) as u32;
            // Which of the samples that start in its word it is.
            let sub = ((14 * sample % 32) as u32 - first_start(word, 14)) / 14;
            for from in [0, word.saturating_sub(1), word.saturating_sub(9), word] {
                let got = unpack_raw(&words[from as usize..], from, 14, last);
                let at = raw_trigger_index(from, word - from, sub, 14);
                assert_eq!(u64::from(got[at as usize]), sample, "sample {sample} from {from}");
            }
        }
    }

    #[test]
    fn runs_expand() {
        assert_eq!(rle_run(0x0000_abcd, 16), (0xabcd, 1));
        assert_eq!(rle_run(0xffff_abcd, 16), (0xabcd, 65_536));
        assert_eq!(rle_run(0x0002_00ff, 16), (0x00ff, 3));
        assert_eq!(rle_run(0xffff_ffa5, 8), (0xa5, 16_777_216));
        assert_eq!(rle_run(0x0000_0305, 8), (5, 4));
        assert_eq!(rle_len(&[0x0002_00ff, 0x0000_0100, 0xffff_0000], 16), 3 + 1 + 65_536);
        assert_eq!(rle_len(&[], 16), 0);
        // Stopped before the first sample: the word written is no run.
        assert_eq!(rle_runs(&[0x0002_00ff, 0], 0), [0x0002_00ff]);
        assert_eq!(rle_runs(&[0], 0), [0u32; 0]);
        assert_eq!(rle_runs(&[0x0002_00ff, 0], 32), [0x0002_00ff, 0]);
        assert_eq!(rle_runs(&[], 0), [0u32; 0]);
    }

    fn ended(status: u32, trig: u32, stop: u32, count: u32) -> Ended {
        Ended { status: status | STATUS_DONE, trig, stop, count }
    }

    fn planned(pre: u64, post: u64, rle: bool) -> Plan {
        Plan {
            registers: Vec::new(),
            pre,
            post,
            rle,
            marked: true,
            channel_mask: 0xffff,
            clock: CLOCK,
        }
    }

    #[test]
    fn the_status_says_how_it_ended() {
        let e = ended(STATUS_TRIGGERED | 20 << 8, 0, 0, 0);
        assert!(e.triggered() && !e.overflowed());
        assert_eq!(e.last_bits(), 20);
        assert_eq!(ended(32 << 8, 0, 0, 0).last_bits(), 32);
        assert!(ended(STATUS_OVERFLOW, 0, 0, 0).overflowed());
    }

    #[test]
    fn the_window_is_ahead_of_the_trigger_to_the_end() {
        let id = la16();
        // No wrap: 600 words written, the trigger in word 500, 100 samples
        // wanted ahead of it: 50 words and two more.
        let e = ended(STATUS_TRIGGERED, 500 + (1 << 24), 600, 600);
        let w = window(&id, &planned(100, 100, false), &e);
        assert_eq!(w, Window { start: 448, words: 152, trigger: Some((52, 1)) });
        // No more ahead of it than there is.
        let e = ended(STATUS_TRIGGERED, 3, 600, 600);
        let w = window(&id, &planned(100, 100, false), &e);
        assert_eq!(w, Window { start: 0, words: 600, trigger: Some((3, 0)) });
        // The first sample of all.
        let e = ended(STATUS_TRIGGERED, 0, 500, 500);
        let w = window(&id, &planned(0, 1000, false), &e);
        assert_eq!(w, Window { start: 0, words: 500, trigger: Some((0, 0)) });
        // Under RLE a run each at the worst.
        let e = ended(STATUS_TRIGGERED, 500, 600, 600);
        let w = window(&id, &planned(100, 100, true), &e);
        assert_eq!(w, Window { start: 400, words: 200, trigger: Some((100, 0)) });
    }

    #[test]
    fn the_window_follows_the_ring_around() {
        let id = id(16, 1, 0, 108_000_000, 1000);
        // Written all the way around and on to address 50; the trigger at
        // 990, with the oldest word at 50.
        let e = ended(STATUS_TRIGGERED, 990, 50, 1000);
        let w = window(&id, &planned(40, 100, false), &e);
        assert_eq!(w, Window { start: 968, words: 82, trigger: Some((22, 0)) });
        // The trigger just after the end of the ring, its history before.
        let e = ended(STATUS_TRIGGERED, 5, 50, 1000);
        let w = window(&id, &planned(40, 100, false), &e);
        assert_eq!(w, Window { start: 983, words: 67, trigger: Some((22, 0)) });
        // All there is ahead of it: from the oldest word.
        let w = window(&id, &planned(1_000_000, 100, false), &e);
        assert_eq!(w, Window { start: 50, words: 1000, trigger: Some((955, 0)) });
    }

    #[test]
    fn a_capture_stopped_before_its_trigger_is_its_last_words() {
        let id = la16();
        let e = ended(0, 0, 600, 600);
        let w = window(&id, &planned(100, 100, false), &e);
        assert_eq!(w, Window { start: 498, words: 102, trigger: None });
        let w = window(&id, &planned(100, 100, true), &e);
        assert_eq!(w, Window { start: 400, words: 200, trigger: None });
        let w = window(&id, &planned(10_000, 10_000, false), &e);
        assert_eq!(w, Window { start: 0, words: 600, trigger: None });
        // Nothing written.
        let w = window(&id, &planned(100, 100, false), &ended(0, 0, 0, 0));
        assert_eq!(w, Window { start: 0, words: 0, trigger: None });
        // Around a ring of 14 bit samples.
        let e = ended(0, 0, 3, 2_097_151);
        let w = window(&la14(), &planned(0, 16, false), &e);
        assert_eq!(w, Window { start: 2_097_145, words: 9, trigger: None });
    }

    #[test]
    fn the_cut_is_around_the_trigger() {
        // Enough on both sides.
        assert_eq!(
            cut(1000, Some(500), 100, 200),
            Cut { skip: 400, take: 300, trigger: Some(100) }
        );
        // Less ahead of the trigger than wanted, and less after.
        assert_eq!(cut(1000, Some(30), 100, 200), Cut { skip: 0, take: 230, trigger: Some(30) });
        assert_eq!(cut(600, Some(500), 100, 200), Cut { skip: 400, take: 200, trigger: Some(100) });
        assert_eq!(cut(1000, Some(0), 0, 200), Cut { skip: 0, take: 200, trigger: Some(0) });
        // No trigger: the last of them.
        assert_eq!(cut(1000, None, 100, 200), Cut { skip: 700, take: 300, trigger: None });
        assert_eq!(cut(50, None, 100, 200), Cut { skip: 0, take: 50, trigger: None });
        assert_eq!(cut(0, None, 100, 200), Cut { skip: 0, take: 0, trigger: None });
        // A trigger beyond what was read is none.
        assert_eq!(cut(50, Some(60), 10, 200), Cut { skip: 0, take: 50, trigger: None });
    }
}
