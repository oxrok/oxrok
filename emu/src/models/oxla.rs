// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The oxla logic analyzer gateware (a Sipeed Tang Nano 20K) on the CDC-ACM
//! shell: 8, 14 or 16 channels sampled into a ring of 32 bit words, a two
//! term trigger, run length encoding, and a UART at 3,000,000 baud that
//! only ever answers.
//!
//! The device, and how sure this is:
//!
//! - The USB side is a stand-in. The board's bridge is a BL616 that shows
//!   as an FTDI `0403:6010` with two ports, the second the UART, which
//!   `ftdi_sio` binds; there is no FTDI shell for serial models yet
//!   (`docs/ROADMAP.md`), so this is a CDC-ACM function with test ids
//!   behind a UART at 3000000/8n1. What the bridge does to the bytes (about
//!   80 kB/s, and bytes lost when more than about 10 kB come in one go) is
//!   not emulated; `lose=` loses one.
//! - The protocol: `docs/PROTOCOL.md` of the gateware (version 1), read
//!   beside its Verilog (`oxla_host.v`, `oxla_core.v`, `oxla_capture.v`)
//!   and its bring-up tool, `tools/oxla.py`, whose self test the unit
//!   tests here repeat.
//!
//! How it works, from the Verilog:
//!
//! - A command is a byte and 0, 1, 5 or 6 argument bytes. An opcode above
//!   `07` is taken and does nothing, as is a write to an address with no
//!   register; a read of one answers 0.
//! - At ARM the ring is written from address 0. Raw samples are a stream
//!   of bits cut every 32; `TRIG` has the address of the word the trigger
//!   sample starts in and, above, how many samples started in that word
//!   before it. Under RLE a run ends when the sample changes, at the
//!   trigger sample and when its count is full.
//! - The counter pattern takes the place of the pins after the channel
//!   mask, so the mask does nothing to it.
//! - A write to `CLOCK` starts the capture side over: no capture, nothing
//!   in memory, and the status without its ready bit until the SDRAM is
//!   back. The ID then has the new clock: 81 MHz for 0, 108 for 1 and for
//!   3, 135 for 2. RESET leaves the clock as it is; power on is 108 MHz.
//!   The document has the other registers go back to their defaults at
//!   such a write; `oxla_host.v` leaves them, and so does this. A host
//!   that follows the document writes them all after it either way.
//! - The trigger is looked for once `PRE_WORDS` words are written. A
//!   "then" that completed an event short of the count starts over.
//! - A capture ends `POST_SAMPLES` samples after the trigger sample, the
//!   word in progress written with as many bits as it has, or with the
//!   word `POST_WORDS` after the trigger's. A STOP writes what is in
//!   progress too: in raw mode a word with 0 to 32 real bits, 0 when no
//!   sample was waiting, and under RLE the run in progress, or with no
//!   sample yet a word that is no run and 0 real bits.
//! - 512 words wait between the samples and the memory, which takes a
//!   little under a word per clock; a word pushed onto a full queue ends
//!   the capture with the overflow bit.
//!
//! Emulation artifacts:
//!
//! - A capture takes no time. It is made when ARM comes, whatever the
//!   rate, and `polls=` says how many looks at the status still find it
//!   running. One that needs more than [`SEARCH`] samples (a trigger that
//!   does not come, a limit beyond that) never ends: it is still running
//!   at every look, marks the event point at the first, and a STOP ends it
//!   as it stands after those samples.
//! - The point of an overflow is a guess: the memory is taken to write 31
//!   words in 32 clocks, so a word per clock overflows after 16384. What
//!   was still queued is lost, as on the device.
//! - The pins carry the count of clocks since ARM, `slow=N` holding each
//!   value for `N` clocks: at the full rate that is the counter pattern
//!   again, at a divided rate it skips, and two samples of one clock read
//!   the same. The eight clocks a sample takes to the queue are not there,
//!   nor the synchronizer.
//! - A byte sent while memory is read out ends the readout on the device;
//!   here an answer is queued whole at once, so nothing can come during
//!   it. Bytes sent while the device answers are lost on it and taken here.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `channels=8|14|16`, `spc=1|2` (samples per clock), `rle=0|1`,
//!   `adc=0|1`, `mem=WORDS`: the bitstream, as its ID says it. By default
//!   `la16-rle` with 2^21 words, 2^21 - 1 for 14 channels;
//! - `clock=HZ`: a bitstream with this one clock and no `CLOCK` register,
//!   as they were before the host could change it;
//! - `settle=N`: looks at the status after a change of clock that find the
//!   device not ready. A real one takes some milliseconds;
//! - `id=none|HEX`: no answer to the ID command, or these bytes;
//! - `slow=N`: each value of the pins lasts `N` clocks;
//! - `polls=N`: looks at the status that find a capture running that has
//!   ended;
//! - `overflow=N`: the capture ends with the overflow bit after `N` words;
//! - `lose=N`: byte `N` of the memory read out since power on, from 1, is
//!   lost on the way;
//! - `uart=LINE`.
//!
//! The pedantic checks: what the document gives no meaning. An opcode above
//! `07`; a register that is none, or bits beyond its field or the
//! bitstream's channels and abilities, 3 in `CLOCK` among them; a write, an
//! ARM or a memory read while a capture runs, or after a change of clock
//! before the status said ready; an ARM the flags and the divider do not allow
//! together (two samples per clock with a divider or with RLE, an ADC with
//! no divider) or whose `POST_WORDS` would run over the trigger's word; a
//! STOP with nothing running; `TRIG`, `STOP` and `COUNT` read, or memory,
//! with no capture done; more than 2048 words in one read, none, or an
//! address beyond the ring; and half a command left when a tool opens.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "oxla";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "The oxla logic analyzer gateware (Tang Nano 20K) on CDC-ACM; \
                         channels=8|14|16, spc=1|2, rle=0|1, adc=0|1, clock=HZ, mem=WORDS, \
                         id=none|HEX, slow=, polls=, settle=, overflow=, lose=, uart=, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

const OP_RESET: u8 = 0x01;
const OP_ID: u8 = 0x02;
const OP_WREG: u8 = 0x03;
const OP_RREG: u8 = 0x04;
const OP_ARM: u8 = 0x05;
const OP_STOP: u8 = 0x06;
const OP_RMEM: u8 = 0x07;

const R_DIV: u8 = 0x00;
const R_POST_WORDS: u8 = 0x01;
const R_PRE_WORDS: u8 = 0x02;
const R_POST_SAMPLES: u8 = 0x03;
const R_FLAGS: u8 = 0x04;
const R_CHMASK: u8 = 0x05;
const R_TMODE: u8 = 0x06;
const R_TCOUNT: u8 = 0x07;
const R_CLOCK: u8 = 0x08;
const R_A: u8 = 0x10;
const R_B: u8 = 0x18;
const R_STATUS: u8 = 0x80;
const R_TRIG: u8 = 0x81;
const R_STOP: u8 = 0x82;
const R_COUNT: u8 = 0x83;

const F_RLE: u32 = 1;
const F_DDR: u32 = 2;
const F_TEST: u32 = 4;

const ST_RUN: u32 = 1;
const ST_TRIG: u32 = 2;
const ST_DONE: u32 = 4;
const ST_OVERFLOW: u32 = 8;
const ST_READY: u32 = 16;

/// The clocks by the value of `CLOCK`, and the value at power on.
const CLOCKS: [u32; 3] = [81_000_000, 108_000_000, 135_000_000];
const POWER_ON_CLOCK: usize = 1;

/// The most words a host asks for in one read.
const CHUNK: u32 = 2048;
/// Words between the samples and the memory.
const QUEUE: u64 = 512;
/// Samples a capture is made of when ARM comes. Time is the host's, so
/// this has to end within a reply.
const SEARCH: u64 = 1 << 19;

/// How many addresses it takes for samples to start at the same bits
/// again: `W / gcd(W, 32)`.
fn group(channels: u8) -> u32 {
    let w = u32::from(channels);
    let mut common = 1;
    while common < 32 && w.is_multiple_of(common * 2) {
        common *= 2;
    }
    w / common
}

/// The bitstream and how it misbehaves.
#[derive(Clone, Debug)]
struct Board {
    channels: u8,
    per_clock: u8,
    rle: bool,
    adc: bool,
    /// The one clock of a bitstream whose clock the host cannot change.
    fixed: Option<u32>,
    mem: u32,
    /// The answer to the ID command when it is not the board's own.
    id: Option<Vec<u8>>,
    slow: u64,
    polls: u32,
    settle: u32,
    overflow: Option<u64>,
    lose: Option<u64>,
}

impl Board {
    fn mask(&self) -> u32 {
        (1 << self.channels) - 1
    }

    /// The ID with the clock at setting `select`.
    fn id(&self, select: usize) -> Vec<u8> {
        if let Some(id) = &self.id {
            return id.clone();
        }
        let caps = u8::from(self.rle)
            | u8::from(self.per_clock > 1) << 1
            | u8::from(self.adc) << 2
            | u8::from(self.fixed.is_none()) << 3;
        let mut d = b"OXLA".to_vec();
        d.extend([1, caps, self.channels, self.per_clock]);
        d.extend(self.fixed.unwrap_or(CLOCKS[select]).to_le_bytes());
        d.extend(self.mem.to_le_bytes());
        d
    }
}

/// A board from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Analyzer>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("3000000/8n1")?;
    let mut board = Board {
        channels: 16,
        per_clock: 1,
        rle: true,
        adc: false,
        fixed: None,
        mem: 0,
        id: None,
        slow: 1,
        polls: 0,
        settle: 0,
        overflow: None,
        lose: None,
    };
    let mut rle = None;
    let mut mem = None;
    let flag = |k: &str, v: &str| -> Result<bool> { Ok(parse_number(k, v, 0..=1)? == 1) };
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "channels" => {
                board.channels = match v.as_str() {
                    "8" => 8,
                    "14" => 14,
                    "16" => 16,
                    _ => return Err(Error::Invalid(format!("channels={v}: 8, 14 or 16"))),
                };
            }
            "spc" => board.per_clock = u8::try_from(parse_number(k, v, 1..=2)?).expect("2"),
            "rle" => rle = Some(flag(k, v)?),
            "adc" => board.adc = flag(k, v)?,
            "clock" => board.fixed = Some(parse_number(k, v, 1..=u32::MAX)?),
            "mem" => mem = Some(parse_number(k, v, 1..=1 << 21)?),
            "id" if v == "none" => board.id = Some(Vec::new()),
            "id" => board.id = Some(parse_hex_list(k, v, 1, 64)?.remove(0)),
            "slow" => board.slow = u64::from(parse_number(k, v, 1..=1 << 24)?),
            "polls" => board.polls = parse_number(k, v, 0..=100_000)?,
            "settle" => board.settle = parse_number(k, v, 0..=100_000)?,
            "overflow" => board.overflow = Some(u64::from(parse_number(k, v, 0..=1 << 21)?)),
            "lose" => board.lose = Some(u64::from(parse_number(k, v, 1..=u32::MAX)?)),
            "uart" => uart = Line::parse(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    // The packer's RLE takes one sample per clock, so no build has both.
    board.rle = rle.unwrap_or(board.per_clock == 1);
    if board.rle && board.per_clock > 1 {
        return Err(Error::Invalid("rle=1 with spc=2: no bitstream has both".to_owned()));
    }
    let whole = group(board.channels);
    board.mem = mem.unwrap_or((1 << 21) / whole * whole);
    if !board.mem.is_multiple_of(whole) {
        return Err(Error::Invalid(format!(
            "mem={}: a multiple of {whole} words with {} channels",
            board.mem, board.channels
        )));
    }
    // Test ids: see the module's note on the board's bridge.
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "oxla stand-in");
    Ok(Cdc::new(NAME, &id, faults, Analyzer::new(board)).behind_uart(uart))
}

/// A capture, between two samples and once it has ended.
#[derive(Clone, Debug, Default)]
struct Capture {
    /// Samples taken.
    samples: u64,
    /// Words pushed towards the memory.
    words: u64,
    /// Raw: the bits not yet in a word, and how many samples started in the
    /// word in progress.
    acc: u64,
    fill: u32,
    starts: u32,
    /// RLE: the run in progress, the sample and how often it was seen.
    run: Option<(u32, u32)>,
    /// The word of the trigger sample, and which of its word's samples it
    /// is.
    trigger: Option<(u64, u32)>,
    /// Samples after the trigger sample.
    after: u64,
    /// Events still to come before the one that fires.
    count: u32,
    /// A "then" saw its first term.
    seen: bool,
    prev: Option<u32>,
    /// What waits for the memory, in 64ths of a word.
    queued: u64,
    last_bits: u32,
    overflow: bool,
    done: bool,
}

/// The FPGA.
struct Analyzer {
    board: Board,
    /// A command's bytes so far.
    partial: Vec<u8>,
    regs: [u32; 0x20],
    /// The ring, once a capture needs it.
    memory: Vec<u32>,
    capture: Option<Capture>,
    /// Looks at the status that still say running.
    polls_left: u32,
    /// The value of `CLOCK`, and the looks at the status that still find
    /// the device not ready after a write of it.
    select: usize,
    unready: u32,
    /// Bytes of memory read out since power on.
    read_out: u64,
    /// The event point of the capture that waits was marked.
    marked: bool,
}

fn arguments(op: u8) -> usize {
    match op {
        OP_WREG => 5,
        OP_RREG => 1,
        OP_RMEM => 6,
        _ => 0,
    }
}

impl Analyzer {
    fn new(board: Board) -> Analyzer {
        let mut a = Analyzer {
            board,
            partial: Vec::new(),
            regs: [0; 0x20],
            memory: Vec::new(),
            capture: None,
            polls_left: 0,
            select: POWER_ON_CLOCK,
            unready: 0,
            read_out: 0,
            marked: false,
        };
        a.defaults();
        a
    }

    /// RESET: every register but the clock at its default, and no capture.
    fn defaults(&mut self) {
        self.regs = [0; 0x20];
        self.regs[usize::from(R_POST_SAMPLES)] = u32::MAX;
        self.regs[usize::from(R_CHMASK)] = self.board.mask();
        self.capture = None;
        self.polls_left = 0;
    }

    fn reg(&self, addr: u8) -> u32 {
        self.regs[usize::from(addr)]
    }

    fn running(&self) -> bool {
        self.capture.as_ref().is_some_and(|c| !c.done)
    }

    /// The bits a register keeps, or none for an address that is no
    /// register.
    fn field(&self, addr: u8) -> Option<u32> {
        let channels = self.board.mask();
        Some(match addr {
            R_DIV => 0x00ff_ffff,
            R_POST_WORDS | R_PRE_WORDS => 0x003f_ffff,
            R_POST_SAMPLES => u32::MAX,
            R_FLAGS | R_TMODE => 7,
            R_TCOUNT => 0xffff,
            R_CLOCK if self.board.fixed.is_none() => 3,
            R_CHMASK => channels,
            a if (R_A..R_A + 5).contains(&a) || (R_B..R_B + 5).contains(&a) => channels,
            _ => return None,
        })
    }

    fn write(&mut self, addr: u8, value: u32, port: &mut Port<'_>) {
        let Some(field) = self.field(addr) else {
            port.violation(format!("write of {value:#x} to {addr:#04x}: no register"));
            return;
        };
        if self.running() {
            port.violation(format!("register {addr:#04x} written while a capture runs"));
        }
        if value & !field != 0 {
            port.violation(format!("register {addr:#04x} = {value:#x}: bits beyond {field:#x}"));
        }
        if addr == R_CLOCK {
            if value & 3 == 3 {
                port.violation("clock setting 3: there are 0 to 2");
            }
            self.select = if value & 3 == 3 { POWER_ON_CLOCK } else { (value & 3) as usize };
            self.capture = None;
            self.polls_left = 0;
            self.memory = Vec::new();
            self.unready = self.board.settle;
            return;
        }
        self.settled("a register written", port);
        match addr {
            R_FLAGS if value & F_RLE != 0 && !self.board.rle => {
                port.violation("the RLE flag on a bitstream without RLE");
            }
            R_FLAGS if value & F_DDR != 0 && self.board.per_clock < 2 => {
                port.violation("two samples per clock on a bitstream with one");
            }
            R_TMODE if value & 7 == 7 => port.violation("trigger mode 7: there are 0 to 6"),
            _ => {}
        }
        self.regs[usize::from(addr)] = value & field;
    }

    /// Reports `what` when it comes after a change of clock and before
    /// the status said ready.
    fn settled(&self, what: &str, port: &mut Port<'_>) {
        if self.unready > 0 {
            port.violation(format!("{what} after a change of clock, before the device was ready"));
        }
    }

    fn status(&mut self, port: &mut Port<'_>) -> u32 {
        if self.unready > 0 {
            self.unready -= 1;
            return 0;
        }
        let Some(c) = &self.capture else {
            return ST_READY;
        };
        if !c.done {
            // It waits for what does not come: where an event can be timed,
            // at the first look.
            if !self.marked {
                self.marked = true;
                port.mark();
            }
            return ST_READY | ST_RUN;
        }
        if self.polls_left > 0 {
            self.polls_left -= 1;
            return ST_READY | ST_RUN;
        }
        let mut status = ST_READY | ST_DONE | c.last_bits << 8;
        if c.trigger.is_some() {
            status |= ST_TRIG;
        }
        if c.overflow {
            status |= ST_OVERFLOW;
        }
        status
    }

    fn read(&mut self, addr: u8, port: &mut Port<'_>) -> u32 {
        let mem = u64::from(self.board.mem);
        match addr {
            R_STATUS => self.status(port),
            R_TRIG | R_STOP | R_COUNT => {
                let Some(c) = self.capture.as_ref().filter(|c| c.done && self.polls_left == 0)
                else {
                    port.violation(format!("register {addr:#04x} read with no capture done"));
                    return 0;
                };
                match addr {
                    R_TRIG => c.trigger.map_or(0, |(word, sub)| (word % mem) as u32 | sub << 24),
                    R_STOP => (c.words % mem) as u32,
                    _ => c.words.min(mem) as u32,
                }
            }
            other => {
                port.violation(format!("read of {other:#04x}: no register"));
                0
            }
        }
    }

    /// The pins at sample `i`: the count of clocks since ARM.
    fn pins(&self, i: u64) -> u32 {
        let clocks = if self.reg(R_FLAGS) & F_DDR != 0 {
            i / 2
        } else {
            i.wrapping_mul(u64::from(self.reg(R_DIV)) + 1)
        };
        (clocks / self.board.slow) as u32 & self.board.mask() & self.reg(R_CHMASK)
    }

    /// `oxla_trigger`'s `term`: whether the term at `base` matches `now`
    /// after `prev`.
    fn term(&self, base: u8, now: u32, prev: Option<u32>) -> bool {
        let r = |i: u8| self.reg(base + i);
        let (mask, value, rise, fall, edge) = (r(0), r(1), r(2), r(3), r(4));
        // The first sample has none before it and matches no edge.
        let (prev, has) = (prev.unwrap_or(0), if prev.is_some() { u32::MAX } else { 0 });
        (now ^ value) & mask == 0
            && has & now & !prev & rise == rise
            && has & !now & prev & fall == fall
            && has & (now ^ prev) & edge == edge
    }

    /// Whether `now` is the trigger sample.
    fn fires(&self, c: &mut Capture, now: u32) -> bool {
        let a = self.term(R_A, now, c.prev);
        let b = self.term(R_B, now, c.prev);
        let then = |c: &mut Capture, first: bool, second: bool| {
            if c.seen {
                second
            } else {
                c.seen = first;
                false
            }
        };
        let event = match self.reg(R_TMODE) {
            0 => true,
            1 => a,
            2 => b,
            3 => a || b,
            4 => a && b,
            5 => then(c, a, b),
            6 => then(c, b, a),
            _ => false,
        };
        if event && c.count > 0 {
            c.count -= 1;
            c.seen = false;
            return false;
        }
        event
    }

    /// A word on its way to the memory.
    fn push(&mut self, c: &mut Capture, word: u32) {
        if c.queued + 64 > QUEUE * 64 || self.board.overflow.is_some_and(|n| c.words >= n) {
            // What was queued never reaches the memory.
            c.words -= (c.queued / 64).min(c.words);
            c.overflow = true;
            c.last_bits = 32;
            c.done = true;
            return;
        }
        let at = (c.words % u64::from(self.board.mem)) as usize;
        self.memory[at] = word;
        c.words += 1;
        c.queued += 64;
        if let Some((trigger, _)) = c.trigger
            && c.words > trigger
            && c.words - 1 - trigger == u64::from(self.reg(R_POST_WORDS))
        {
            c.last_bits = 32;
            c.done = true;
        }
    }

    /// The end of a capture: what is in progress is written, as the last
    /// sample or a STOP has it. A STOP writes a word in raw mode even with
    /// no bits waiting; the last sample then has just filled one.
    fn finish(&mut self, c: &mut Capture, stopped: bool) {
        if c.done {
            return;
        }
        if self.reg(R_FLAGS) & F_RLE == 0 && c.fill == 0 && !stopped {
            c.last_bits = 32;
            c.done = true;
            return;
        }
        if self.reg(R_FLAGS) & F_RLE != 0 {
            let (word, bits) = match c.run.take() {
                Some((value, seen)) => (value | (seen - 1) << self.board.channels, 32),
                None => (0, 0),
            };
            self.push(c, word);
            c.last_bits = if c.overflow { 32 } else { bits };
        } else {
            let bits = c.fill;
            let word = c.acc as u32;
            c.acc = 0;
            c.fill = 0;
            self.push(c, word);
            c.last_bits = if c.overflow { 32 } else { bits };
        }
        c.done = true;
    }

    /// Takes samples until the capture ends, [`SEARCH`] of them at the
    /// most.
    fn sample(&mut self, c: &mut Capture) {
        let w = u32::from(self.board.channels);
        let flags = self.reg(R_FLAGS);
        let ddr = flags & F_DDR != 0;
        let longest = 1u32 << (32 - w);
        // What the memory takes between two samples, in 64ths of a word:
        // 31 words in 32 clocks.
        let drained = if ddr { 31 } else { 62 * (u64::from(self.reg(R_DIV)) + 1) };
        while !c.done && c.samples < SEARCH {
            let i = c.samples;
            c.samples += 1;
            c.queued = c.queued.saturating_sub(drained);
            let now = if flags & F_TEST != 0 { i as u32 & self.board.mask() } else { self.pins(i) };
            let armed = c.trigger.is_none() && c.words >= u64::from(self.reg(R_PRE_WORDS));
            let fires = armed && self.fires(c, now);
            c.prev = Some(now);
            if flags & F_RLE != 0 {
                match &mut c.run {
                    Some((value, seen)) if *value == now && *seen < longest && !fires => *seen += 1,
                    _ => {
                        if let Some((value, seen)) = c.run.take() {
                            self.push(c, value | (seen - 1) << w);
                        }
                        c.run = Some((now, 1));
                    }
                }
                if fires {
                    c.trigger = Some((c.words, 0));
                }
            } else {
                if fires {
                    c.trigger = Some((c.words, c.starts));
                }
                c.acc |= u64::from(now) << c.fill;
                c.fill += w;
                c.starts += 1;
                if c.fill >= 32 {
                    let word = c.acc as u32;
                    c.acc >>= 32;
                    c.fill -= 32;
                    c.starts = 0;
                    self.push(c, word);
                }
            }
            if c.trigger.is_some() && !c.done {
                if c.after == u64::from(self.reg(R_POST_SAMPLES)) {
                    self.finish(c, false);
                }
                c.after += 1;
            }
        }
    }

    fn arm(&mut self, port: &mut Port<'_>) {
        if self.running() {
            port.violation("ARM while a capture runs");
            return;
        }
        let (flags, div) = (self.reg(R_FLAGS), self.reg(R_DIV));
        if flags & F_DDR != 0 && div != 0 {
            port.violation(format!("two samples per clock with a divider of {}", div + 1));
        }
        if flags & F_DDR != 0 && flags & F_RLE != 0 {
            port.violation("RLE with two samples per clock");
        }
        if self.board.adc && div == 0 {
            port.violation("an ADC with no divider: it gets no clock");
        }
        self.settled("ARM", port);
        if self.reg(R_POST_WORDS) >= self.board.mem {
            port.violation(format!(
                "POST_WORDS {}: what follows runs over the trigger's word",
                self.reg(R_POST_WORDS)
            ));
        }
        if self.memory.is_empty() {
            self.memory = vec![0; self.board.mem as usize];
        }
        self.marked = false;
        let mut c = Capture { count: self.reg(R_TCOUNT), ..Capture::default() };
        self.sample(&mut c);
        self.polls_left = self.board.polls;
        self.capture = Some(c);
    }

    fn stop(&mut self, port: &mut Port<'_>) {
        match self.capture.take() {
            Some(mut c) if !c.done => {
                self.finish(&mut c, true);
                self.capture = Some(c);
            }
            // Ended, and the host has not seen it yet.
            Some(c) if self.polls_left > 0 => {
                self.polls_left = 0;
                self.capture = Some(c);
            }
            other => {
                self.capture = other;
                port.violation("STOP with no capture running");
            }
        }
    }

    fn read_memory(&mut self, addr: u32, count: u32, port: &mut Port<'_>) {
        let mem = self.board.mem;
        if count == 0 || count > CHUNK {
            port.violation(format!("{count} words asked for in one read: 1 to {CHUNK}"));
        }
        if addr >= mem {
            port.violation(format!("memory read from {addr}: the ring has {mem} words"));
        }
        if !self.capture.as_ref().is_some_and(|c| c.done && self.polls_left == 0) {
            port.violation("memory read with no capture done");
        }
        // A read of what was never written: the SDRAM as it powers up,
        // which is anything. Zeros here.
        let mut bytes = Vec::with_capacity(count as usize * 4);
        for i in 0..count {
            let at = ((u64::from(addr) + u64::from(i)) % u64::from(mem)) as usize;
            bytes.extend(self.memory.get(at).copied().unwrap_or(0).to_le_bytes());
        }
        let from = self.read_out;
        self.read_out += bytes.len() as u64;
        if let Some(lost) = self.board.lose
            && (from + 1..=self.read_out).contains(&lost)
        {
            bytes.remove((lost - from - 1) as usize);
        }
        port.send(&bytes);
    }

    fn execute(&mut self, c: &[u8], port: &mut Port<'_>) {
        let le = |b: &[u8]| b.iter().rev().fold(0u32, |v, b| v << 8 | u32::from(*b));
        match c[0] {
            0 => {}
            OP_RESET => self.defaults(),
            OP_ID => {
                let id = self.board.id(self.select);
                port.send(&id);
            }
            OP_WREG => self.write(c[1], le(&c[2..6]), port),
            OP_RREG => {
                let value = self.read(c[1], port);
                port.send(&value.to_le_bytes());
            }
            OP_ARM => self.arm(port),
            OP_STOP => self.stop(port),
            // 21 bits of address, 22 of count.
            OP_RMEM => self.read_memory(le(&c[1..4]) & 0x1f_ffff, le(&c[4..7]) & 0x3f_ffff, port),
            other => port.violation(format!("opcode {other:#04x}: there are 00 to 07")),
        }
    }
}

impl Serial for Analyzer {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for b in data {
            self.partial.push(*b);
            if self.partial.len() == 1 + arguments(self.partial[0]) {
                let command = std::mem::take(&mut self.partial);
                self.execute(&command, port);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.partial.is_empty() {
            port.violation(format!(
                "{} bytes of a command {:#04x} left by the tool before",
                self.partial.len(),
                self.partial[0]
            ));
        }
    }

    fn reset(&mut self) {
        *self = Analyzer::new(self.board.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn board(options: &[(&str, &str)]) -> Host<Analyzer> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("3000000/8n1").unwrap(), true, true)
    }

    /// `la14-ddr` and `la8-ddr`.
    fn la14() -> Host<Analyzer> {
        board(&[("channels", "14"), ("spc", "2")])
    }

    fn la8() -> Host<Analyzer> {
        board(&[("channels", "8"), ("spc", "2")])
    }

    fn violations(h: &mut Host<Analyzer>) -> Vec<String> {
        h.v.drain()
    }

    fn read_all(h: &mut Host<Analyzer>) -> Vec<u8> {
        let mut all = Vec::new();
        loop {
            let got = h.read();
            if got.is_empty() {
                return all;
            }
            all.extend(got);
        }
    }

    fn wreg(h: &mut Host<Analyzer>, addr: u8, value: u32) {
        let mut c = vec![OP_WREG, addr];
        c.extend(value.to_le_bytes());
        h.write(&c);
    }

    fn rreg(h: &mut Host<Analyzer>, addr: u8) -> u32 {
        h.write(&[OP_RREG, addr]);
        let got = read_all(h);
        u32::from_le_bytes(got.try_into().expect("four bytes"))
    }

    fn rmem(h: &mut Host<Analyzer>, addr: u32, count: u32) -> Vec<u32> {
        let (a, c) = (addr.to_le_bytes(), count.to_le_bytes());
        h.write(&[OP_RMEM, a[0], a[1], a[2], c[0], c[1], c[2]]);
        read_all(h)
            .chunks(4)
            .map(|b| u32::from_le_bytes(b.try_into().expect("whole words")))
            .collect()
    }

    /// A term as the tool's `set_term` writes it: `(channel, kind)` with
    /// kind one of `0 1 r f e`.
    fn term(h: &mut Host<Analyzer>, base: u8, spec: &[(u32, char)]) {
        let mut r = [0u32; 5];
        for (ch, kind) in spec {
            let bit = 1 << ch;
            match kind {
                '0' => r[0] |= bit,
                '1' => {
                    r[0] |= bit;
                    r[1] |= bit;
                }
                'r' => r[2] |= bit,
                'f' => r[3] |= bit,
                _ => r[4] |= bit,
            }
        }
        for (i, v) in (0..).zip(r) {
            wreg(h, base + i, v);
        }
    }

    /// What `oxla.py`'s `result()` makes of a capture that ended.
    struct Got {
        status: u32,
        words: Vec<u32>,
        samples: Vec<u32>,
        /// The trigger's word and sample, counted from the first read.
        trigger_word: u32,
        trigger: Option<usize>,
    }

    fn result(h: &mut Host<Analyzer>, want: Option<u32>, rle: bool) -> Got {
        let (w, mem) = (u32::from(h.cdc.model.board.channels), h.cdc.model.board.mem);
        let first = |addr: u32| (w - (32 * (addr % group(w as u8))) % w) % w;
        let status = rreg(h, R_STATUS);
        assert_ne!(status & ST_DONE, 0, "done: {status:#x}");
        let (trig, stop, count) = (rreg(h, R_TRIG), rreg(h, R_STOP), rreg(h, R_COUNT));
        let n = want.map_or(count, |want| want.min(count));
        let start = (stop + mem - n % mem) % mem;
        let mut words = Vec::new();
        while (words.len() as u32) < n {
            let chunk = (n - words.len() as u32).min(CHUNK);
            words.extend(rmem(h, (start + words.len() as u32) % mem, chunk));
        }
        let trigger_word = ((trig & 0x1f_ffff) + mem - start) % mem;
        let mask = (1u64 << w) - 1;
        let mut samples = Vec::new();
        let at = if rle {
            for x in &words {
                samples.extend(std::iter::repeat_n(x & mask as u32, (x >> w) as usize + 1));
            }
            words[..trigger_word as usize].iter().map(|x| (x >> w) as usize + 1).sum()
        } else {
            let skip = first(start);
            let bits = 32 * (u64::from(n).saturating_sub(1)) + u64::from((status >> 8) & 63);
            let total = if n == 0 { 0 } else { bits.saturating_sub(u64::from(skip)) / u64::from(w) };
            let (mut acc, mut have) = (0u64, 0);
            for (i, x) in words.iter().enumerate() {
                acc |= u64::from(*x) << have;
                have += 32;
                if i == 0 {
                    acc >>= skip;
                    have -= skip;
                }
                while have >= w && (samples.len() as u64) < total {
                    samples.push((acc & mask) as u32);
                    acc >>= w;
                    have -= w;
                }
            }
            ((32 * trigger_word + first(start + trigger_word) - skip) / w + ((trig >> 24) & 3))
                as usize
        };
        Got { status, words, samples, trigger_word, trigger: (status & ST_TRIG != 0).then_some(at) }
    }

    fn counts(samples: &[u32], mask: u32) -> bool {
        samples.windows(2).all(|p| p[1] == (p[0] + 1) & mask)
    }

    #[test]
    fn the_id_says_what_the_bitstream_is() {
        let mut h = board(&[]);
        // Eight NOPs and a RESET, then the ID.
        h.write(&[0, 0, 0, 0, 0, 0, 0, 0, OP_RESET, OP_ID]);
        let mut id = b"OXLA".to_vec();
        id.extend([1, 9, 16, 1, 0x00, 0xf3, 0x6f, 0x06, 0x00, 0x00, 0x20, 0x00]);
        assert_eq!(read_all(&mut h), id);
        let mut h = la14();
        h.write(&[OP_ID]);
        let id = read_all(&mut h);
        assert_eq!(id[4..8], [1, 10, 14, 2]);
        assert_eq!(id[8..12], 108_000_000u32.to_le_bytes());
        assert_eq!(id[12..], 2_097_151u32.to_le_bytes());
        let mut h = board(&[("channels", "8"), ("rle", "0"), ("adc", "1"), ("clock", "54000000")]);
        h.write(&[OP_ID]);
        assert_eq!(read_all(&mut h)[4..8], [1, 4, 8, 1]);
        let mut h = board(&[("id", "31534c4f")]);
        h.write(&[OP_ID]);
        assert_eq!(read_all(&mut h), b"1SLO");
        let mut h = board(&[("id", "none")]);
        h.write(&[OP_ID]);
        assert_eq!(read_all(&mut h), b"");
        assert_eq!(rreg(&mut h, R_STATUS), ST_READY);
        assert_eq!(violations(&mut h), Vec::<String>::new());
    }

    #[test]
    fn options_that_make_no_board_are_refused() {
        let build = |o: &[(&str, &str)]| {
            let o: Vec<(String, String)> =
                o.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
            function(&o).map(|_| ())
        };
        assert!(build(&[("channels", "12")]).is_err());
        assert!(build(&[("spc", "2"), ("rle", "1")]).is_err());
        assert!(build(&[("channels", "14"), ("mem", "1000")]).is_err());
        assert!(build(&[("channels", "14"), ("mem", "1001")]).is_ok());
        assert!(build(&[("nope", "1")]).is_err());
        assert!(build(&[("chunk", "7")]).is_ok(), "the shell's");
    }

    #[test]
    fn a_line_that_is_not_the_uart_s_passes_nothing() {
        let o = [];
        let mut h = Host::open(function(&o).unwrap(), Line::default(), true, true);
        h.write(&[OP_ID]);
        assert_eq!(read_all(&mut h), b"");
    }

    /// The memory part of `oxla.py selftest`: the counter from address 0.
    #[test]
    fn the_counter_fills_the_memory() {
        for (mut h, w) in [(board(&[]), 16u32), (la14(), 14), (la8(), 8)] {
            let mask = (1 << w) - 1;
            for div in [0, 3] {
                let ddr = if w != 16 && div == 0 { F_DDR } else { 0 };
                h.write(&[OP_RESET]);
                wreg(&mut h, R_FLAGS, F_TEST | ddr);
                wreg(&mut h, R_DIV, div);
                wreg(&mut h, R_POST_WORDS, 20_000 - 1);
                h.write(&[OP_ARM]);
                let got = result(&mut h, None, false);
                assert_eq!(got.words.len(), 20_000);
                assert_eq!(got.trigger_word, 0);
                assert_eq!(got.status & ST_OVERFLOW, 0);
                assert_eq!(got.samples.len() as u32, 20_000 * 32 / w);
                assert!(got.samples.iter().zip(0..).all(|(v, i)| *v == i & mask), "{w} channels");
            }
            assert_eq!(violations(&mut h), Vec::<String>::new());
        }
    }

    /// The trigger part of the self test, with its cases and its checks.
    #[test]
    #[allow(clippy::verbose_bit_mask)]
    fn the_trigger_modes_fire_where_the_self_test_expects() {
        /// The mode, the terms, the count, and what the trigger sample is.
        type Case = (u32, &'static [(u32, char)], &'static [(u32, char)], u32, fn(u32) -> bool);
        let all: [Case; 7] = [
            (1, &[(5, 'r')], &[], 0, |v| v & 0x3f == 0x20),
            (
                2,
                &[],
                &[(0, '1'), (1, '0'), (2, '1'), (3, '1'), (4, '0'), (5, '1'), (6, '0'), (7, '1')],
                0,
                |v| v & 0xff == 0xad,
            ),
            (3, &[(5, 'r')], &[(3, 'f')], 0, |v| v & 0x3f == 0x20 || v & 0xf == 0),
            (4, &[(5, 'r')], &[(4, 'f'), (0, '0')], 0, |v| v & 0x3f == 0x20),
            (5, &[(5, 'r')], &[(3, 'f')], 2, |v| v & 0x3f == 0x30),
            (6, &[(5, 'r')], &[(3, 'f')], 0, |v| v & 0x3f == 0x20),
            (1, &[(2, 'e')], &[], 999, |v| v & 3 == 0),
        ];
        for (mut h, w) in [(board(&[]), 16u32), (la14(), 14), (la8(), 8)] {
            let ddr = if w == 16 { 0 } else { F_DDR };
            for (mode, a, b, count, ok) in all {
                h.write(&[OP_RESET]);
                wreg(&mut h, R_FLAGS, F_TEST | ddr);
                wreg(&mut h, R_TMODE, mode);
                wreg(&mut h, R_TCOUNT, count);
                term(&mut h, R_A, a);
                term(&mut h, R_B, b);
                wreg(&mut h, R_PRE_WORDS, 500);
                wreg(&mut h, R_POST_WORDS, 1500);
                h.write(&[OP_ARM]);
                let got = result(&mut h, Some(2001), false);
                let at = got.trigger.expect("triggered");
                assert_eq!((got.words.len(), got.trigger_word), (2001, 500), "mode {mode}");
                assert!(counts(&got.samples, (1 << w) - 1));
                assert!(ok(got.samples[at]), "mode {mode}, {w} channels: {:#x}", got.samples[at]);
                assert_eq!(got.status & ST_OVERFLOW, 0);
            }
            assert_eq!(violations(&mut h), Vec::<String>::new());
        }
    }

    #[test]
    fn the_first_of_each_trigger_is_where_the_counter_has_it() {
        // No hold-off: the first match of all. A then B: bit 5 rises at 32,
        // bit 3 falls next at 48; the third such pair ends at 176.
        let mut h = board(&[]);
        for (mode, count, at) in [(1, 0, 32), (5, 0, 48), (5, 2, 176), (6, 0, 32), (3, 0, 16)] {
            h.write(&[OP_RESET]);
            wreg(&mut h, R_FLAGS, F_TEST);
            wreg(&mut h, R_TMODE, mode);
            wreg(&mut h, R_TCOUNT, count);
            term(&mut h, R_A, &[(5, 'r')]);
            term(&mut h, R_B, &[(3, 'f')]);
            wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
            wreg(&mut h, R_POST_SAMPLES, 9);
            h.write(&[OP_ARM]);
            let got = result(&mut h, None, false);
            assert_eq!(got.trigger, Some(at), "mode {mode} count {count}");
            assert_eq!(got.samples.len(), at + 10);
        }
        // The first sample matches a level, and no edge.
        for (kind, at) in [('0', 0), ('e', 1), ('f', 2)] {
            h.write(&[OP_RESET]);
            wreg(&mut h, R_FLAGS, F_TEST);
            wreg(&mut h, R_TMODE, 1);
            term(&mut h, R_A, &[(0, kind)]);
            wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
            wreg(&mut h, R_POST_SAMPLES, 0);
            h.write(&[OP_ARM]);
            assert_eq!(result(&mut h, None, false).trigger, Some(at), "{kind}");
        }
        assert_eq!(violations(&mut h), Vec::<String>::new());
    }

    #[test]
    fn a_count_of_samples_ends_it_with_a_word_partly_filled() {
        for (mut h, w, ddr) in [(board(&[]), 16u32, 0), (la14(), 14, F_DDR), (la8(), 8, F_DDR)] {
            h.write(&[OP_RESET]);
            wreg(&mut h, R_FLAGS, F_TEST | ddr);
            let last = h.cdc.model.board.mem - 1;
            wreg(&mut h, R_POST_WORDS, last);
            wreg(&mut h, R_POST_SAMPLES, 12_345);
            h.write(&[OP_ARM]);
            let got = result(&mut h, None, false);
            assert_eq!(got.samples.len(), 12_346);
            assert!(counts(&got.samples, (1 << w) - 1));
            // 12346 samples of w bits: what is left over in the last word.
            let last = 12_346 * w % 32;
            assert_eq!((got.status >> 8) & 63, if last == 0 { 32 } else { last });
            assert_eq!(violations(&mut h), Vec::<String>::new());
        }
    }

    /// The trigger of fourteen channels, a sample at a time: `TRIG` names
    /// the word it starts in and which of that word's samples it is.
    #[test]
    fn fourteen_channels_say_where_the_trigger_sample_starts() {
        let mut h = la14();
        for at in 0..40u32 {
            h.write(&[OP_RESET]);
            wreg(&mut h, R_FLAGS, F_TEST | F_DDR);
            wreg(&mut h, R_TMODE, 1);
            wreg(&mut h, R_A, 0x3f);
            wreg(&mut h, R_A + 1, at);
            wreg(&mut h, R_POST_WORDS, 0x1f_fffe);
            wreg(&mut h, R_POST_SAMPLES, 3);
            h.write(&[OP_ARM]);
            let trig = {
                assert_ne!(rreg(&mut h, R_STATUS) & ST_TRIG, 0);
                rreg(&mut h, R_TRIG)
            };
            assert_eq!(trig & 0x1f_ffff, 14 * at / 32, "sample {at}");
            let got = result(&mut h, None, false);
            assert_eq!(got.trigger, Some(at as usize));
            assert_eq!(got.samples.len() as u32, at + 4);
        }
        assert_eq!(violations(&mut h), Vec::<String>::new());
    }

    /// The RLE part of the self test.
    #[test]
    fn runs_and_what_ends_them() {
        let mut h = board(&[]);
        h.write(&[OP_RESET]);
        wreg(&mut h, R_FLAGS, F_TEST | F_RLE);
        wreg(&mut h, R_DIV, 3);
        wreg(&mut h, R_TMODE, 1);
        term(&mut h, R_A, &[(0, '1'), (1, '1'), (2, '1'), (3, '1'), (4, '1'), (5, '1'), (6, '1'), (7, '1')]);
        wreg(&mut h, R_PRE_WORDS, 100);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        wreg(&mut h, R_POST_SAMPLES, 50_000);
        h.write(&[OP_ARM]);
        let got = result(&mut h, None, true);
        let after: u32 = got.words[got.trigger_word as usize..].iter().map(|x| (x >> 16) + 1).sum();
        assert_eq!(after, 50_001);
        assert!(counts(&got.samples, 0xffff));
        assert_eq!(got.words[got.trigger_word as usize] & 0xff, 0xff);
        assert_eq!(got.status & ST_OVERFLOW, 0);

        // A word a sample at the full rate is more than the memory takes.
        h.write(&[OP_RESET]);
        wreg(&mut h, R_FLAGS, F_TEST | F_RLE);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        h.write(&[OP_ARM]);
        let status = rreg(&mut h, R_STATUS);
        assert_eq!(status & (ST_DONE | ST_OVERFLOW), ST_DONE | ST_OVERFLOW);
        // The queue gains a word in 32: full after 16353, and the 511 in
        // it then are never written.
        assert_eq!(rreg(&mut h, R_COUNT), 15_842);
        // And at half of it there is time.
        h.write(&[OP_RESET]);
        wreg(&mut h, R_FLAGS, F_TEST | F_RLE);
        wreg(&mut h, R_DIV, 1);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        wreg(&mut h, R_POST_SAMPLES, 99_999);
        h.write(&[OP_ARM]);
        let got = result(&mut h, None, true);
        assert_eq!((got.status & ST_OVERFLOW, got.words.len()), (0, 100_000));

        // The pins, held for 100,000 clocks a value: the count is exact,
        // and a run is as long as its count holds.
        let mut h = board(&[("slow", "100000")]);
        wreg(&mut h, R_FLAGS, F_RLE);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        wreg(&mut h, R_POST_SAMPLES, 300_000);
        h.write(&[OP_ARM]);
        let got = result(&mut h, None, true);
        assert_eq!(got.samples.len(), 300_001);
        let twice = |value: u32| [0xffff_0000 | value, 0x869f_0000 | value];
        assert_eq!(got.words, [twice(0), twice(1), twice(2)].concat().into_iter().chain([3]).collect::<Vec<_>>());
        assert_eq!(violations(&mut h), Vec::<String>::new());
    }

    #[test]
    fn the_ring_is_written_around() {
        // 70 words; bit 12 rises at 4096, in word 1792, which is 42 around.
        let mut h = board(&[("channels", "14"), ("spc", "2"), ("mem", "70")]);
        wreg(&mut h, R_FLAGS, F_TEST | F_DDR);
        wreg(&mut h, R_TMODE, 1);
        term(&mut h, R_A, &[(12, 'r')]);
        wreg(&mut h, R_POST_WORDS, 69);
        wreg(&mut h, R_POST_SAMPLES, 59);
        h.write(&[OP_ARM]);
        assert_eq!(rreg(&mut h, R_STATUS) & 0xff, ST_READY | ST_DONE | ST_TRIG);
        assert_eq!(rreg(&mut h, R_TRIG), 42);
        assert_eq!(rreg(&mut h, R_COUNT), 70);
        assert_eq!(rreg(&mut h, R_STOP), 1819 % 70);
        let got = result(&mut h, None, false);
        let at = got.trigger.unwrap();
        assert_eq!(got.samples[at], 4096);
        assert!(counts(&got.samples, 0x3fff));
        assert_eq!(got.samples.len() - at, 60);
        assert_eq!(violations(&mut h), Vec::<String>::new());
    }

    #[test]
    fn a_capture_that_does_not_end_runs_until_the_stop() {
        // Bit 1 of a counter never rises with bit 0.
        let mut h = board(&[]);
        wreg(&mut h, R_FLAGS, F_TEST);
        wreg(&mut h, R_TMODE, 1);
        term(&mut h, R_A, &[(0, 'r'), (1, 'r')]);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        h.write(&[OP_ARM]);
        for _ in 0..3 {
            assert_eq!(rreg(&mut h, R_STATUS), ST_READY | ST_RUN);
        }
        h.write(&[OP_STOP]);
        // Every sample was in a word: the one written last has no bits.
        assert_eq!(rreg(&mut h, R_STATUS), ST_READY | ST_DONE);
        let got = result(&mut h, Some(1001), false);
        assert_eq!(got.trigger, None);
        assert_eq!(got.samples.len(), 2000);
        assert_eq!(got.samples.last(), Some(&((SEARCH as u32 - 1) & 0xffff)));
        assert_eq!(violations(&mut h), Vec::<String>::new());

        // Under RLE the run in progress is written.
        let mut h = board(&[("slow", "1000000")]);
        wreg(&mut h, R_FLAGS, F_RLE);
        wreg(&mut h, R_TMODE, 1);
        term(&mut h, R_A, &[(3, 'r')]);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        h.write(&[OP_ARM]);
        assert_eq!(rreg(&mut h, R_STATUS), ST_READY | ST_RUN);
        h.write(&[OP_STOP]);
        assert_eq!(rreg(&mut h, R_STATUS), ST_READY | ST_DONE | 32 << 8);
        let got = result(&mut h, None, true);
        assert_eq!(got.samples.len() as u64, SEARCH);
        assert_eq!(violations(&mut h), Vec::<String>::new());
    }

    #[test]
    fn the_faults() {
        // Two looks that find it running.
        let mut h = board(&[("polls", "2")]);
        wreg(&mut h, R_FLAGS, F_TEST);
        wreg(&mut h, R_POST_SAMPLES, 99);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        h.write(&[OP_ARM]);
        assert_eq!(rreg(&mut h, R_STATUS), ST_READY | ST_RUN);
        assert_eq!(rreg(&mut h, R_STATUS), ST_READY | ST_RUN);
        assert_eq!(rreg(&mut h, R_STATUS), ST_READY | ST_DONE | ST_TRIG | 32 << 8);
        assert_eq!(violations(&mut h), Vec::<String>::new());

        // An overflow after 20 words.
        let mut h = board(&[("overflow", "20")]);
        wreg(&mut h, R_FLAGS, F_TEST);
        wreg(&mut h, R_POST_SAMPLES, 99);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        h.write(&[OP_ARM]);
        assert_eq!(rreg(&mut h, R_STATUS) & 0xff, ST_READY | ST_DONE | ST_TRIG | ST_OVERFLOW);

        // Byte 6 of the memory lost: seven bytes for two words.
        let mut h = board(&[("lose", "6")]);
        wreg(&mut h, R_FLAGS, F_TEST);
        wreg(&mut h, R_POST_SAMPLES, 3);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        h.write(&[OP_ARM]);
        assert_ne!(rreg(&mut h, R_STATUS) & ST_DONE, 0);
        h.write(&[OP_RMEM, 0, 0, 0, 2, 0, 0]);
        assert_eq!(read_all(&mut h), [0, 0, 1, 0, 2, 3, 0]);
        h.write(&[OP_RMEM, 0, 0, 0, 2, 0, 0]);
        assert_eq!(read_all(&mut h).len(), 8);
    }

    #[test]
    fn each_thing_the_document_does_not_allow_is_reported() {
        fn one(h: &mut Host<Analyzer>, says: &str) {
            let v = h.v.drain();
            assert!(v.len() == 1 && v[0].contains(says), "{says}: {v:?}");
        }
        let mut h = board(&[("rle", "0"), ("clock", "108000000")]);
        wreg(&mut h, R_CLOCK, 1);
        one(&mut h, "to 0x08: no register");
        let mut h = board(&[("rle", "0")]);
        h.write(&[0x08]);
        one(&mut h, "opcode 0x08");
        wreg(&mut h, 0x09, 1);
        one(&mut h, "to 0x09: no register");
        wreg(&mut h, R_STATUS, 0);
        one(&mut h, "to 0x80: no register");
        wreg(&mut h, R_DIV, 1 << 24);
        one(&mut h, "bits beyond 0xffffff");
        wreg(&mut h, R_POST_WORDS, 1 << 22);
        one(&mut h, "bits beyond 0x3fffff");
        wreg(&mut h, R_FLAGS, 8);
        one(&mut h, "bits beyond 0x7");
        wreg(&mut h, R_TCOUNT, 0x1_0000);
        one(&mut h, "bits beyond 0xffff");
        wreg(&mut h, R_FLAGS, F_RLE);
        one(&mut h, "without RLE");
        wreg(&mut h, R_FLAGS, F_DDR);
        one(&mut h, "bitstream with one");
        wreg(&mut h, R_TMODE, 7);
        one(&mut h, "trigger mode 7");
        assert_eq!(rreg(&mut h, 0x10), 0);
        one(&mut h, "read of 0x10: no register");
        rreg(&mut h, R_TRIG);
        one(&mut h, "0x81 read with no capture done");
        h.write(&[OP_STOP]);
        one(&mut h, "STOP with no capture running");
        h.write(&[OP_RMEM, 0, 0, 0, 1, 0, 0]);
        read_all(&mut h);
        one(&mut h, "memory read with no capture done");

        // Eight channels: bits beyond them, in the mask and in a term.
        let mut h = la8();
        wreg(&mut h, R_CHMASK, 0x100);
        one(&mut h, "bits beyond 0xff");
        wreg(&mut h, R_B + 4, 0x100);
        one(&mut h, "bits beyond 0xff");
        // What the flags and the divider do not allow together.
        wreg(&mut h, R_FLAGS, F_DDR);
        wreg(&mut h, R_DIV, 1);
        wreg(&mut h, R_POST_SAMPLES, 0);
        h.write(&[OP_ARM]);
        one(&mut h, "with a divider of 2");
        h.write(&[OP_RESET]);
        wreg(&mut h, R_POST_WORDS, 1 << 21);
        wreg(&mut h, R_POST_SAMPLES, 0);
        h.write(&[OP_ARM]);
        one(&mut h, "POST_WORDS 2097152");
        let mut h = board(&[("channels", "8"), ("rle", "0"), ("adc", "1")]);
        wreg(&mut h, R_POST_SAMPLES, 0);
        h.write(&[OP_ARM]);
        one(&mut h, "an ADC with no divider");

        // While a capture runs.
        let mut h = board(&[]);
        wreg(&mut h, R_TMODE, 1);
        term(&mut h, R_A, &[(15, '1'), (14, '0')]);
        wreg(&mut h, R_CHMASK, 0x7fff);
        h.write(&[OP_ARM]);
        assert_eq!(rreg(&mut h, R_STATUS), ST_READY | ST_RUN);
        wreg(&mut h, R_DIV, 1);
        one(&mut h, "written while a capture runs");
        h.write(&[OP_ARM]);
        one(&mut h, "ARM while a capture runs");
        h.write(&[OP_RMEM, 0, 0, 0, 1, 0, 0]);
        read_all(&mut h);
        one(&mut h, "memory read with no capture done");
        h.write(&[OP_STOP]);
        assert_ne!(rreg(&mut h, R_STATUS) & ST_DONE, 0);
        assert_eq!(violations(&mut h), Vec::<String>::new());
        // The reads a host may not make of what it has.
        h.write(&[OP_RMEM, 0, 0, 0, 0x01, 0x08, 0]);
        assert_eq!(read_all(&mut h).len(), 4 * 2049);
        one(&mut h, "2049 words asked for in one read");
        h.write(&[OP_RMEM, 0, 0, 0, 0, 0, 0]);
        one(&mut h, "0 words asked for in one read");
        let mut h = board(&[("mem", "1000")]);
        wreg(&mut h, R_POST_SAMPLES, 0);
        wreg(&mut h, R_POST_WORDS, 999);
        h.write(&[OP_ARM]);
        assert_ne!(rreg(&mut h, R_STATUS) & ST_DONE, 0);
        h.write(&[OP_RMEM, 0xe8, 0x03, 0, 1, 0, 0]);
        read_all(&mut h);
        one(&mut h, "memory read from 1000");

        // Half a command, and the next tool opens.
        let mut h = board(&[]);
        h.write(&[OP_WREG, R_DIV, 0]);
        h.set_lines(true, true);
        one(&mut h, "3 bytes of a command 0x03 left");
    }

    #[test]
    fn a_reset_is_power_on_again() {
        let mut h = board(&[]);
        wreg(&mut h, R_FLAGS, F_TEST);
        wreg(&mut h, R_POST_SAMPLES, 7);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        h.write(&[OP_ARM, OP_WREG]);
        h.cdc.model.reset();
        assert_eq!(rreg(&mut h, R_STATUS), ST_READY);
        assert!(h.cdc.model.partial.is_empty() && h.cdc.model.memory.is_empty());
        assert_eq!(violations(&mut h), Vec::<String>::new());
    }

    #[test]
    fn a_change_of_clock_starts_everything_but_the_link_over() {
        let clock = |h: &mut Host<Analyzer>| {
            h.write(&[OP_ID]);
            let id = read_all(h);
            u32::from_le_bytes(id[8..12].try_into().unwrap())
        };
        let mut h = board(&[("settle", "2")]);
        assert_eq!(clock(&mut h), 108_000_000);
        wreg(&mut h, R_FLAGS, F_TEST);
        wreg(&mut h, R_POST_SAMPLES, 7);
        wreg(&mut h, R_POST_WORDS, 0x1f_ffff);
        h.write(&[OP_ARM]);
        assert_ne!(rreg(&mut h, R_STATUS) & ST_DONE, 0);
        for (select, hz) in [(0, 81_000_000), (2, 135_000_000), (1, 108_000_000), (2, 135_000_000)] {
            wreg(&mut h, R_CLOCK, select);
            assert_eq!([rreg(&mut h, R_STATUS), rreg(&mut h, R_STATUS)], [0, 0]);
            assert_eq!(rreg(&mut h, R_STATUS), ST_READY, "no capture any more");
            assert_eq!(clock(&mut h), hz);
        }
        assert_eq!(violations(&mut h), Vec::<String>::new());
        // The registers are as they were, which the document has
        // otherwise: eight samples of the counter again.
        h.write(&[OP_ARM]);
        assert_eq!(rreg(&mut h, R_STATUS), ST_READY | ST_DONE | ST_TRIG | 32 << 8);
        assert_eq!(rreg(&mut h, R_COUNT), 4);
        assert_eq!(rmem(&mut h, 0, 4), [0x0001_0000, 0x0003_0002, 0x0005_0004, 0x0007_0006]);
        // A RESET leaves the clock.
        h.write(&[OP_RESET]);
        assert_eq!(clock(&mut h), 135_000_000);
        assert_eq!(violations(&mut h), Vec::<String>::new());

        // What may not follow a change before the status said ready, and a
        // setting there is not.
        let mut h = board(&[("settle", "1")]);
        wreg(&mut h, R_CLOCK, 0);
        wreg(&mut h, R_DIV, 1);
        assert!(violations(&mut h)[0].contains("a register written after a change of clock"));
        h.write(&[OP_ARM]);
        assert!(violations(&mut h).iter().any(|v| v.contains("ARM after a change of clock")));
        assert_eq!(rreg(&mut h, R_STATUS), 0);
        wreg(&mut h, R_CLOCK, 3);
        assert!(violations(&mut h)[0].contains("clock setting 3"));
        assert_eq!(rreg(&mut h, R_STATUS), 0);
        assert_eq!(clock(&mut h), 108_000_000);
        // A bitstream with one clock says so in its ID.
        let mut h = board(&[("clock", "54000000")]);
        h.write(&[OP_ID]);
        let id = read_all(&mut h);
        assert_eq!((id[5], &id[8..12]), (1, &54_000_000u32.to_le_bytes()[..]));
    }
}
