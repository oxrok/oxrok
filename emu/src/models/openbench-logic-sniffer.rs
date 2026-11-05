// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An Open Bench Logic Sniffer, and the SUMP boards that talk like it, on
//! the CDC-ACM shell: an FPGA logic analyzer with a USB serial port, single
//! byte and five byte commands, and a capture sent newest sample first.
//!
//! The device, and how sure this is:
//!
//! - The USB side: `04d8:fc92` in normal mode, `cdc_acm` on Linux (the
//!   ols-fwloader README; a udev rule on wiki.yobi.be; the `mchpcdc.inf` in
//!   `GadgetFactory/OpenBench-Logic-Sniffer`, which gives the two strings
//!   here, those of the inf and not read from a device). No descriptor dump
//!   was found: the rest of the descriptors are the shell's. It is native
//!   USB behind a PIC, so the line settings do nothing; `uart=` gives a
//!   board behind a real UART instead (an Arduino, for one).
//! - The commands: `SumpProtocolConstants.java` and `SumpCommandWriter.java`
//!   in `jawi/ols`, the Arduino's `gillham/logic_analyzer` and the Verilog
//!   of `magnuskarlsson/Pipistrello_ols_verilog` (`decoder.v`,
//!   `controller.v`). That Verilog is a fork of the Demon Core this board
//!   runs; the board's own 3.07 source was not read, only its `.mcs` looked
//!   at, which shows no strings. So these are hand read, not simulated, and
//!   not from this board.
//! - The metadata: the name is the one `jawi/ols`' profile looks for, the
//!   rest pysump's dump of an OLS: FPGA `3.07`, 24576 bytes of memory,
//!   200 MHz, 32 probes (0x40), protocol 2 (0x41). A wiki says 24K, 12K and
//!   6K for 1, 2 and 4 groups. Nothing shows what it answers for the
//!   ancillary version; `meta=` plays any block.
//! - A board without metadata: an old SUMP design ignores 0x04, so it says
//!   nothing. `meta=none`.
//!
//! How it works, from `controller.v`, `stage.v`, `trigger.v` and `rle_enc.v`:
//!
//! - A command with bit 7 set takes four more bytes, little endian.
//!   `0x81` holds the read count in its low half and the delay count in its
//!   high half; `0x84` and `0x83` hold them whole (26 bits are kept), and
//!   a board with more than 256 KiB takes only those.
//! - A reset clears the trigger stages. A stage then has mask 0, level 0 and
//!   no start bit, so it matches the first word and raises the level. A
//!   driver that leaves stages unset therefore has its second stage active
//!   from the second word, whatever the first one saw. The C driver sets only
//!   the stages it uses (`p-ols` sets the others to level 3), so on this
//!   model `0=10` waits only for its second stage: a finding from the fork's
//!   source, not from a bench.
//! - After a stage with the start bit fires, `(delay + 1) * 4` more words
//!   are written, and the last `(read + 1) * 4` go back, newest first, each
//!   the bytes of its enabled groups. Words the capture never wrote read 0
//!   (assumed). A read count beyond the memory gives the whole memory, one
//!   group taking one byte, two taking two and three or four taking four
//!   (assumed: the fork does so, and a wiki gives 1, 2 and 4 only).
//! - The pins carry the count of samples: `t & 0xffff` and its complement
//!   above, so channel `k` below 16 first rises at `t = 2^k`; `slow=N`
//!   holds each value for `N` samples. In demux mode a word holds samples
//!   `2k` in groups 3 and 4, and `2k + 1` in groups 1 and 2, and a stage
//!   matches if either half does, so a mask on the low half alone fires at
//!   once: every trigger at 200 MHz starts at the first word.
//! - With the RLE flag a run of equal words is the word, then a count word
//!   (top bit of the top byte set, the rest the repeats), which go back the
//!   other way round, so the top channel is lost. A run still open at the
//!   end loses its count. A gap against `rle_enc.v`: it stores a run of two
//!   as the value twice, here the value and a count of 1.
//!
//! Emulation artifacts:
//!
//! - A capture takes no time. It goes out whole as soon as the run command
//!   comes, whatever the rate, the delay or the clock flag, so nothing here
//!   checks how long a tool waits; and `hold` stands for a capture not done.
//! - A reset drops nothing already queued in the shell, as bytes on their
//!   way to the host are no longer the FPGA's. A tool that stops reading
//!   early still gets the rest in its buffer.
//! - With RLE a delay beyond the memory counts as the memory's length, or
//!   the 2^28 words of a read count of 0xffffffff would take too long in a
//!   reply. The firmware would run on; the pins repeat anyway.
//! - The search for a start stops after [`SEARCH`] words, and it never
//!   starts then: the pins repeat by then unless `slow=` is large.
//! - The noise filter, the test patterns, the channel swap and the external
//!   clock are taken and not emulated. The pins carry the count regardless.
//!
//! What the C driver does on it, verified on sigrok-cli's runs: a board with
//! no channel count leaves `unitsize` 0 (`protocol.c:304`), and `hex.c:163`'s
//! loop never ends (cases `no metadata capture`, `no channel key capture`).
//! Every board that does not answer metadata meets this. No memory key makes
//! the read count 0xffff, so the board sends what it has and C keeps the
//! newest (`no memory key capture`). A limit that is not a multiple of 4 gets
//! more than it asked for and keeps the newest, the marker reckoned for all
//! of them (`ratio 50 limit 5`). A capture with no sample sends an empty logic
//! packet, and the output's loop runs off (`unplug waiting`). The marker
//! beyond what came wraps the packet's length, and sigrok-cli dies of SIGSEGV
//! (`sends 3 ratio 50`, `ratio 100 limit 5`); before the first sample, with
//! `captureratio=0` and two stages or more, it sends samples from before the
//! buffer (`ratio 0 two stages`).
//!
//! What no case reaches in the C driver (`--gcov`, 94.7%): `api.c`'s NULL
//! checks and default branches, which sigrok-cli cannot ask for; an ID read
//! that fails with data waiting; `convert_trigger`'s disabled channel, as
//! sigrok-cli takes a trigger only on a channel it enabled; the drains that
//! fail and the writes before a run that fail apart from those of two cases,
//! as an unplug followed by another write is a race; `protocol.c:399`, a read
//! after a poll that said there is data; and line 404, which cannot run.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `id=ols|sla|none|HEX`: the answer to `0x02`, of any length;
//! - `meta=none|HEX`: no answer to `0x04`, or that block;
//! - `name=`, `fpga=`, `channels=`, `rate=`, `protocol=`, `memory=`: the
//!   default block's values; `memory=` is also the memory there is, up to
//!   the shell's 1 MiB queue;
//! - `slow=N`: each value of the pins lasts `N` samples;
//! - `sends=N`: the capture stops after `N` words and marks the event point,
//!   `0` is a capture that never ends;
//! - `uart=LINE`.
//!
//! The pedantic checks: only the commands C has a path for, and the bits it
//! can set in each. Nothing but a reset once a capture is armed, nothing left
//! of a command when a tool opens, a run after its read count and with a
//! group enabled, and `0x81` or `0x83`/`0x84` as the default block's memory
//! says. Two C habits are taken, as nothing refuses them: read and delay
//! counts of `0xffff` and `0xffffffff` (a board with no memory key, and a
//! capture ratio of 100), and a demux capture with groups 3 and 4 enabled
//! apart from 1 and 2, which `p-ols`' comments forbid and `ols` sends.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "openbench-logic-sniffer";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "An Open Bench Logic Sniffer or SUMP board on CDC-ACM; id=ols|sla|none|\
                         HEX, meta=none|HEX, name=, fpga=, channels=, rate=, protocol=, memory=, \
                         slow=, sends=, uart=, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

const RESET: u8 = 0x00;
const RUN: u8 = 0x01;
const ID: u8 = 0x02;
const METADATA: u8 = 0x04;
const SET_DIVIDER: u8 = 0x80;
const SET_SIZE: u8 = 0x81;
const SET_FLAGS: u8 = 0x82;
const SET_DELAY: u8 = 0x83;
const SET_READ: u8 = 0x84;

const FLAG_DEMUX: u16 = 1 << 0;
const FLAG_GROUPS: u16 = 0x3c;
const FLAG_RLE: u16 = 1 << 8;
const FLAG_EXTERNAL_TEST: u16 = 1 << 10;
const FLAG_INTERNAL_TEST: u16 = 1 << 11;
/// Bits 12 to 15: reserved, and the RLE mode, which nothing sets.
const FLAG_NEVER: u16 = 0xf000;

/// The trigger config's start bit and level field.
const START: u32 = 1 << 27;
const LEVEL: u32 = 3 << 16;
/// `controller.v`'s `fwd` and `bwd` have 26 bits.
const COUNT_MASK: u32 = 0x03ff_ffff;
/// The most the shell's queue holds.
const MAX_MEMORY: u32 = 1 << 20;
/// Above this the C uses `0x83` and `0x84`.
const SMALL_MEMORY: u32 = 256 * 1024;
/// Words a run looks through for its start. Time is the host's, so this has
/// to end within a reply.
const SEARCH: u64 = 1 << 19;

#[derive(Clone, Copy, Debug, Default)]
struct Stage {
    mask: u32,
    value: u32,
    config: u32,
}

impl Stage {
    /// `stage.v`: both halves must match, or in demux mode one.
    fn fires(self, word: u32, demux: bool) -> bool {
        let matches = |half: u32| (word ^ self.value) & self.mask & half == 0;
        if demux {
            matches(0xffff) || matches(0xffff_0000)
        } else {
            matches(u32::MAX)
        }
    }

    fn level(self) -> u32 {
        (self.config & LEVEL) >> 16
    }
}

#[derive(Clone, Debug)]
struct Board {
    id: Option<Vec<u8>>,
    metadata: Option<Vec<u8>>,
    memory: u32,
    slow: u64,
    sends: Option<u64>,
    /// `true` if it takes only `0x83` and `0x84`; `None` for a block not
    /// made from `memory=`, which says nothing of it.
    long_counts: Option<bool>,
}

/// The default block from these values.
fn block(name: &str, fpga: &str, channels: u8, memory: u32, rate: u32, protocol: u8) -> Vec<u8> {
    let mut m = vec![0x01];
    m.extend_from_slice(name.as_bytes());
    m.extend_from_slice(&[0, 0x02]);
    m.extend_from_slice(fpga.as_bytes());
    m.extend_from_slice(&[0, 0x21]);
    m.extend_from_slice(&memory.to_be_bytes());
    m.push(0x23);
    m.extend_from_slice(&rate.to_be_bytes());
    m.extend_from_slice(&[0x40, channels, 0x41, protocol, 0x00]);
    m
}

/// A board from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Sniffer>> {
    let mut faults = Faults::default();
    let mut uart = None;
    let mut id = Some(b"1SLO".to_vec());
    let mut meta = None;
    let (mut name, mut fpga) = ("Open Logic Sniffer v1.01".to_owned(), "3.07".to_owned());
    let (mut channels, mut rate, mut protocol) = (32, 200_000_000, 2);
    let (mut memory, mut slow, mut sends) = (24576, 1, None);
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "id" => {
                id = match v.as_str() {
                    "none" => None,
                    "ols" => Some(b"1SLO".to_vec()),
                    "sla" => Some(b"1ALS".to_vec()),
                    _ => Some(parse_hex_list(k, v, 1, 64)?.remove(0)),
                };
            }
            "meta" if v == "none" => meta = Some(None),
            "meta" => meta = Some(Some(parse_hex_list(k, v, 1, 4096)?.remove(0))),
            "name" => name.clone_from(v),
            "fpga" => fpga.clone_from(v),
            "channels" => channels = u8::try_from(parse_number(k, v, 0..=255)?).expect("255"),
            "rate" => rate = parse_number(k, v, 0..=u32::MAX)?,
            "protocol" => protocol = u8::try_from(parse_number(k, v, 0..=255)?).expect("255"),
            "memory" => memory = parse_number(k, v, 4..=MAX_MEMORY)?,
            "slow" => slow = u64::from(parse_number(k, v, 1..=65536)?),
            "sends" => sends = Some(u64::from(parse_number(k, v, 0..=MAX_MEMORY)?)),
            "uart" => uart = Some(Line::parse(v)?),
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    let long_counts = meta.is_none().then_some(memory > SMALL_MEMORY);
    let metadata = match meta {
        Some(given) => given,
        None => Some(block(&name, &fpga, channels, memory, rate, protocol)),
    };
    let board = Board { id, metadata, memory, slow, sends, long_counts };
    let identity = Identity::new(0x04d8, 0xfc92, "Microchip Technology, Inc.", "Communications Port");
    let cdc = Cdc::new(NAME, &identity, faults, Sniffer::new(board));
    Ok(match uart {
        Some(line) => cdc.behind_uart(line),
        None => cdc,
    })
}

/// The enabled groups' bytes of `word`, packed.
fn compact(groups: u8, word: u32) -> u32 {
    let mut out = 0;
    let mut at = 0;
    for g in 0..4 {
        if groups & (1 << g) != 0 {
            out |= ((word >> (8 * g)) & 0xff) << (8 * at);
            at += 1;
        }
    }
    out
}

/// The FPGA.
struct Sniffer {
    board: Board,
    /// A long command's bytes so far.
    partial: Vec<u8>,
    flags: u16,
    read: Option<u32>,
    delay: u32,
    stages: [Stage; 4],
    /// A run came and no reset since.
    armed: bool,
    /// A capture that sends no more marks the event point at the next offer
    /// with nothing queued: the run's own write is still being drained.
    point_due: bool,
}

impl Sniffer {
    fn new(board: Board) -> Sniffer {
        Sniffer {
            board,
            partial: Vec::new(),
            flags: 0,
            read: None,
            delay: 0,
            stages: [Stage::default(); 4],
            armed: false,
            point_due: false,
        }
    }

    fn short(&mut self, command: u8, port: &mut Port<'_>) {
        if self.armed && command != RESET {
            port.violation(format!("command {command:#04x} on an armed capture; only a reset"));
        }
        match command {
            RESET => {
                self.armed = false;
                self.stages = [Stage::default(); 4];
            }
            RUN => self.run(port),
            ID => {
                if let Some(id) = &self.board.id {
                    port.send(id);
                }
            }
            METADATA => {
                if let Some(m) = &self.board.metadata {
                    port.send(m);
                }
            }
            other => port.violation(format!("short command {other:#04x}: no path of C sends it")),
        }
    }

    /// `0x83` and `0x84`, or `0x81`, as the block made the board.
    fn check_counts(&self, op: u8, port: &mut Port<'_>) {
        let long = op != SET_SIZE;
        if self.board.long_counts.is_some_and(|l| l != long) {
            port.violation(format!(
                "command {op:#04x} on a board of {} bytes: the wrong form of the counts",
                self.board.memory
            ));
        }
    }

    fn long(&mut self, b: [u8; 5], port: &mut Port<'_>) {
        let (op, arg) = (b[0], u32::from_le_bytes([b[1], b[2], b[3], b[4]]));
        if self.armed {
            port.violation(format!("command {op:#04x} on an armed capture; only a reset"));
            return;
        }
        match op {
            SET_DIVIDER => {
                if b[4] != 0 {
                    port.violation(format!("divider {arg:#010x}: 24 bits"));
                }
            }
            SET_SIZE => {
                self.check_counts(op, port);
                self.read = Some(arg & 0xffff);
                self.delay = arg >> 16;
            }
            SET_READ => {
                self.check_counts(op, port);
                self.read = Some(arg & COUNT_MASK);
            }
            SET_DELAY => {
                self.check_counts(op, port);
                self.delay = arg & COUNT_MASK;
            }
            SET_FLAGS => {
                let flags = u16::try_from(arg).unwrap_or_else(|_| {
                    port.violation(format!("flags {arg:#010x}: 16 bits"));
                    arg as u16
                });
                if flags & FLAG_NEVER != 0 {
                    port.violation(format!("flags {flags:#06x}: RLE mode and reserved bits"));
                }
                if flags & FLAG_INTERNAL_TEST != 0 && flags & FLAG_EXTERNAL_TEST != 0 {
                    port.violation(format!("flags {flags:#06x}: both test patterns"));
                }
                self.flags = flags;
            }
            0xc0..=0xcf if op & 3 != 3 => {
                let stage = usize::from((op - 0xc0) / 4);
                let s = &mut self.stages[stage];
                match op & 3 {
                    0 => s.mask = arg,
                    1 => s.value = arg,
                    _ => {
                        if arg & !(LEVEL | START) != 0 {
                            port.violation(format!(
                                "trigger config {arg:#010x}: a delay, a channel or a serial mode"
                            ));
                        }
                        if (arg & LEVEL) >> 16 != u32::try_from(stage).expect("below 4") {
                            port.violation(format!(
                                "trigger config {arg:#010x}: not the level of stage {stage}"
                            ));
                        }
                        s.config = arg;
                    }
                }
            }
            other => port.violation(format!("long command {other:#04x}: no path of C sends it")),
        }
    }

    /// The pins at sample `t`.
    fn stimulus(&self, t: u64) -> u32 {
        let low = (t / self.board.slow) as u16;
        u32::from(low) | (u32::from(!low) << 16)
    }

    /// The word sampled `k`-th.
    fn raw(&self, k: u64) -> u32 {
        if self.flags & FLAG_DEMUX != 0 {
            (self.stimulus(2 * k) << 16) | (self.stimulus(2 * k + 1) & 0xffff)
        } else {
            self.stimulus(k)
        }
    }

    /// The word the delay counts from, unless no stage ever starts.
    fn find_start(&self) -> Option<u64> {
        let demux = self.flags & FLAG_DEMUX != 0;
        let mut level = 0;
        let mut fired = [false; 4];
        for k in 0..SEARCH {
            let word = self.raw(k);
            let mut bump = false;
            for (s, stage) in self.stages.iter().enumerate() {
                if fired[s] || stage.level() > level || !stage.fires(word, demux) {
                    continue;
                }
                fired[s] = true;
                if stage.config & START != 0 {
                    // A stage that looks at a pin counts from the next word.
                    return Some(k + u64::from(stage.mask != 0));
                }
                bump = true;
            }
            if bump {
                level += 1;
            }
        }
        None
    }

    /// `rle_enc.v`'s mask on a word of `width` bytes, and whether its two
    /// demux samples differ, which stops it being compressed.
    fn rle_value(&self, width: u32, word: u32) -> (u32, bool) {
        let demux = self.flags & FLAG_DEMUX != 0;
        let (mask, half) = match (demux, width) {
            (true, 2) => (0x7f7f, 8),
            (true, 4) => (0x7fff_7fff, 16),
            _ => (!(1u32 << (8 * width - 1)), 0),
        };
        let v = word & mask;
        (v, half != 0 && (v >> half) != (v & ((1 << half) - 1)))
    }

    /// What memory holds with RLE, the oldest word first.
    fn rle_memory(&self, groups: u8, width: u32, start: u64, read: u64, delay: u64) -> Vec<u32> {
        let flag = 1u32 << (8 * width - 1);
        let delay = delay.min(read);
        let mut memory = std::collections::VecDeque::new();
        let mut written = 0u64;
        let mut mark = None;
        let mut run: Option<(u32, u32)> = None;
        let mut t = 0;
        let push = |memory: &mut std::collections::VecDeque<u32>, written: &mut u64, word| {
            memory.push_back(word);
            if memory.len() as u64 > read {
                memory.pop_front();
            }
            *written += 1;
        };
        loop {
            if t == start {
                mark = Some(written);
            }
            if mark.is_some_and(|m: u64| written - m >= delay) {
                break;
            }
            let (v, mismatch) = self.rle_value(width, compact(groups, self.raw(t)));
            t += 1;
            match run {
                Some((cur, n)) if cur == v && !mismatch && n < flag - 1 => {
                    run = Some((cur, n + 1));
                }
                last => {
                    if let Some((_, n)) = last
                        && n > 0
                    {
                        push(&mut memory, &mut written, n | flag);
                    }
                    push(&mut memory, &mut written, v);
                    run = Some((v, 0));
                }
            }
        }
        memory.into()
    }

    /// The bytes of a capture, newest first.
    fn capture(&self, start: u64, read: u32, groups: u8, width: u32) -> Vec<u8> {
        let stored = u64::from(if width == 3 { 4 } else { width });
        let capacity = u64::from(self.board.memory) / stored;
        let read_words = ((u64::from(read) + 1) * 4).min(capacity);
        let delay_words = (u64::from(self.delay) + 1) * 4;
        let width = width as usize;
        let sent = self.board.sends.map_or(read_words, |n| n.min(read_words));
        let mut out = Vec::with_capacity(usize::try_from(sent).expect("bounded") * width);
        if self.flags & FLAG_RLE != 0 {
            let memory =
                self.rle_memory(groups, width as u32, start, read_words, delay_words);
            let missing = read_words - memory.len() as u64;
            let words = memory.iter().rev().copied().chain((0..missing).map(|_| 0));
            for word in words.take(usize::try_from(sent).expect("bounded")) {
                out.extend_from_slice(&word.to_le_bytes()[..width]);
            }
        } else {
            let end = start + delay_words;
            for i in 0..sent {
                let word = match end.checked_sub(i + 1) {
                    Some(k) => compact(groups, self.raw(k)),
                    None => 0,
                };
                out.extend_from_slice(&word.to_le_bytes()[..width]);
            }
        }
        out
    }

    fn run(&mut self, port: &mut Port<'_>) {
        self.armed = true;
        let Some(read) = self.read else {
            port.violation("a run before any read count");
            return;
        };
        let groups = (!(self.flags & FLAG_GROUPS) >> 2) as u8 & 0xf;
        let width = groups.count_ones();
        if width == 0 {
            port.violation("a run with every channel group disabled");
            return;
        }
        let Some(start) = self.find_start() else {
            self.point_due = true;
            return;
        };
        self.point_due = self.board.sends.is_some();
        let bytes = self.capture(start, read, groups, width);
        port.send(&bytes);
    }
}

impl Serial for Sniffer {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if self.partial.is_empty() && byte & 0x80 == 0 {
                self.short(byte, port);
                continue;
            }
            self.partial.push(byte);
            if let Ok(command) = <[u8; 5]>::try_from(self.partial.as_slice()) {
                self.partial.clear();
                self.long(command, port);
            }
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if std::mem::take(&mut self.point_due) {
            port.mark_event_point();
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.partial.is_empty() {
            port.violation(format!("{:02x?}: a command left unfinished", self.partial));
            self.partial.clear();
        }
    }

    fn reset(&mut self) {
        *self = Sniffer::new(self.board.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn board(options: &[(&str, &str)]) -> Host<Sniffer> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::default(), true, true)
    }

    fn violations(h: &mut Host<Sniffer>) -> usize {
        h.v.drain().len()
    }

    fn long(op: u8, arg: u32) -> Vec<u8> {
        let mut v = vec![op];
        v.extend_from_slice(&arg.to_le_bytes());
        v
    }

    /// A capture as `ols_prepare_acquisition` sets one up, with no trigger.
    fn setup(h: &mut Host<Sniffer>, flags: u32, read: u32, delay: u32) {
        h.write(&long(0xc0, 0));
        h.write(&long(0xc1, 0));
        h.write(&long(0xc2, START));
        h.write(&long(SET_DIVIDER, 0));
        h.write(&long(SET_SIZE, read | delay << 16));
        h.write(&long(SET_FLAGS, flags));
    }

    fn read_all(h: &mut Host<Sniffer>) -> Vec<u8> {
        let mut all = Vec::new();
        loop {
            let got = h.read();
            if got.is_empty() {
                return all;
            }
            all.extend(got);
        }
    }

    #[test]
    fn the_id_and_the_metadata() {
        let mut h = board(&[]);
        for _ in 0..5 {
            h.write(&[RESET]);
        }
        h.write(&[ID]);
        assert_eq!(read_all(&mut h), b"1SLO");
        h.write(&[METADATA]);
        let mut expected = vec![0x01];
        expected.extend_from_slice(b"Open Logic Sniffer v1.01\0\x023.07\0");
        expected.extend_from_slice(&[0x21, 0, 0, 0x60, 0, 0x23, 0x0b, 0xeb, 0xc2, 0]);
        expected.extend_from_slice(&[0x40, 32, 0x41, 2, 0]);
        assert_eq!(read_all(&mut h), expected);
        assert_eq!(violations(&mut h), 0);
        let mut h = board(&[("id", "sla"), ("meta", "none")]);
        h.write(&[ID]);
        assert_eq!(read_all(&mut h), b"1ALS");
        h.write(&[METADATA]);
        assert_eq!(read_all(&mut h), b"");
        let mut h = board(&[("id", "none"), ("meta", "01410000")]);
        h.write(&[ID]);
        assert_eq!(read_all(&mut h), b"");
        h.write(&[METADATA]);
        assert_eq!(read_all(&mut h), [1, 0x41, 0, 0]);
    }

    #[test]
    fn a_capture_goes_newest_first() {
        let mut h = board(&[]);
        // One group of four: read 8 words, delay 8.
        setup(&mut h, 0x38, 1, 1);
        h.write(&[RUN]);
        assert_eq!(read_all(&mut h), [7, 6, 5, 4, 3, 2, 1, 0]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_groups_are_in_order_and_unwritten_words_are_0() {
        let mut h = board(&[]);
        // Groups 1 and 3: the count and its complement. Read 8, delay 4.
        setup(&mut h, 0x28, 1, 0);
        h.write(&[RUN]);
        let got = read_all(&mut h);
        assert_eq!(&got[..4], [3, 0xfc, 2, 0xfd]);
        assert_eq!(&got[8..], [0; 8]);
    }

    #[test]
    fn the_memory_bounds_a_read_count() {
        for (flags, width) in [(0x38, 1), (0x30, 2), (0x20, 3), (0, 4)] {
            let mut h = board(&[("memory", "64")]);
            setup(&mut h, flags, 0xffff, 0);
            h.write(&[RUN]);
            let words = 64 / if width == 3 { 4 } else { width };
            assert_eq!(read_all(&mut h).len(), words * width);
        }
    }

    #[test]
    fn one_stage_starts_at_the_word_after_its_match() {
        let mut h = board(&[]);
        h.write(&long(0xc0, 1 << 4));
        h.write(&long(0xc1, 1 << 4));
        h.write(&long(0xc2, START));
        h.write(&long(SET_SIZE, 1));
        h.write(&long(SET_FLAGS, 0x38));
        h.write(&[RUN]);
        // First match at 16; four words from 17, then the newest eight back.
        assert_eq!(read_all(&mut h), [20, 19, 18, 17, 16, 15, 14, 13]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn stages_left_cleared_by_a_reset_raise_the_level_at_once() {
        let mut h = board(&[]);
        h.write(&[RESET]);
        // Stage 0 can never match: channels 0 and 16 are complements.
        h.write(&long(0xc0, 0x0001_0001));
        h.write(&long(0xc1, 0x0001_0001));
        h.write(&long(0xc2, 0));
        // Stage 1 wants channel 4.
        h.write(&long(0xc4, 1 << 4));
        h.write(&long(0xc5, 1 << 4));
        h.write(&long(0xc6, 1 << 16 | START));
        h.write(&long(SET_SIZE, 0));
        h.write(&long(SET_FLAGS, 0x38));
        h.write(&[RUN]);
        assert_eq!(read_all(&mut h), [20, 19, 18, 17]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_trigger_that_never_comes_sends_nothing() {
        let mut h = board(&[]);
        h.write(&long(0xc0, 0x0001_0001));
        h.write(&long(0xc1, 0x0001_0001));
        h.write(&long(0xc2, START));
        h.write(&long(SET_SIZE, 0));
        h.write(&long(SET_FLAGS, 0x38));
        h.write(&[RUN]);
        assert_eq!(read_all(&mut h), b"");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn demux_words_hold_two_samples_and_any_half_fires() {
        let mut h = board(&[]);
        setup(&mut h, 1, 0, 0);
        h.write(&[RUN]);
        let got = read_all(&mut h);
        // Word 3: sample 7 in groups 1 and 2, sample 6 in 3 and 4.
        assert_eq!(&got[..4], [7, 0, 6, 0]);
        let mut h = board(&[]);
        h.write(&long(0xc0, 0x0001_0001));
        h.write(&long(0xc1, 0x0001_0001));
        h.write(&long(0xc2, START));
        h.write(&long(SET_SIZE, 0));
        h.write(&long(SET_FLAGS, 0x38 | 1));
        h.write(&[RUN]);
        assert_eq!(read_all(&mut h).len(), 4, "channels 0 and 16 are no complements here");
    }

    #[test]
    fn rle_is_the_value_then_the_count_newest_first() {
        let mut h = board(&[("slow", "10")]);
        // One group, the count of tens: values 0 and 1, then 2 and so on.
        setup(&mut h, 0x38 | u32::from(FLAG_RLE), 0, 0);
        h.write(&[RUN]);
        // Written: 0, 89, 1, 89, 2: a run's count comes with the next value,
        // and the delay ends after that. The last four, newest first.
        assert_eq!(read_all(&mut h), [2, 0x89, 1, 0x89]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn rle_masks_the_top_channel_and_caps_a_count() {
        let mut h = board(&[("slow", "200")]);
        setup(&mut h, 0x38 | u32::from(FLAG_RLE), 0, 0);
        h.write(&[RUN]);
        // A count of 127 at most in a byte: 200 samples are 0, ff, 0, c7.
        assert_eq!(read_all(&mut h), [1, 0xc7, 0, 0xff]);
        let mut h = board(&[]);
        h.write(&long(SET_SIZE, 0));
        h.write(&long(SET_FLAGS, u32::from(FLAG_RLE)));
        h.write(&long(0xc2, START));
        h.write(&[RUN]);
        let got = read_all(&mut h);
        assert_eq!(got[3] & 0x80, 0, "channel 31 is lost");
    }

    #[test]
    fn sends_cuts_a_capture_and_marks_the_point() {
        let mut h = board(&[("sends", "3")]);
        setup(&mut h, 0x38, 1, 1);
        h.write(&[RUN]);
        assert_eq!(read_all(&mut h), [7, 6, 5]);
        let mut h = board(&[("sends", "0")]);
        setup(&mut h, 0x38, 1, 1);
        h.write(&[RUN]);
        assert_eq!(read_all(&mut h), b"");
    }

    #[test]
    fn the_event_point_waits_for_an_offer_with_nothing_queued() {
        let mut h = board(&[("sends", "0")]);
        setup(&mut h, 0x38, 1, 1);
        h.write(&[RUN]);
        assert!(!h.cdc.take_event_point(), "the run's write is still draining");
        h.offer();
        assert!(h.cdc.take_event_point());
        // After the words that are sent, not before them.
        let mut h = board(&[("sends", "2")]);
        setup(&mut h, 0x38, 1, 1);
        h.write(&[RUN]);
        assert_eq!(read_all(&mut h), [7, 6]);
        assert!(h.cdc.take_event_point());
        // A capture sent whole has no point.
        let mut h = board(&[]);
        setup(&mut h, 0x38, 1, 1);
        h.write(&[RUN]);
        read_all(&mut h);
        assert!(!h.cdc.take_event_point());
    }

    #[test]
    fn the_counts_have_two_forms() {
        let mut h = board(&[]);
        h.write(&long(SET_READ, 0));
        assert_eq!(violations(&mut h), 1);
        h.write(&long(SET_DELAY, 0));
        assert_eq!(violations(&mut h), 1);
        let mut h = board(&[("memory", "1048576")]);
        h.write(&long(SET_SIZE, 0));
        assert_eq!(violations(&mut h), 1);
        h.write(&long(SET_READ, 0xffff_ffff));
        h.write(&long(SET_DELAY, 0xffff_ffff));
        assert_eq!(violations(&mut h), 0);
        // meta= says nothing of the memory, and either form is taken.
        let mut h = board(&[("meta", "00")]);
        h.write(&long(SET_READ, 0));
        h.write(&long(SET_SIZE, 0));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_read_count_of_all_ones_keeps_the_top_bits_off() {
        let mut h = board(&[("memory", "1048576")]);
        h.write(&long(SET_READ, 0xffff_ffff));
        h.write(&long(SET_DELAY, 0xffff_ffff));
        h.write(&long(0xc2, START));
        h.write(&long(SET_FLAGS, 0x38));
        h.write(&[RUN]);
        assert_eq!(read_all(&mut h).len(), 1 << 20);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_no_c_path_sends_is_flagged() {
        for bad in [
            vec![0x03],
            vec![0x05],
            vec![0x06],
            vec![0x07],
            vec![0x08],
            vec![0x0f],
            vec![0x11],
            vec![0x13],
            long(0x85, 0),
            long(0x9e, 0),
            long(0x9f, 0),
            long(0xc3, 0),
            long(0xd0, 0),
            long(SET_DIVIDER, 1 << 24),
            long(SET_FLAGS, 1 << 16),
            long(SET_FLAGS, u32::from(FLAG_NEVER)),
            long(SET_FLAGS, u32::from(FLAG_INTERNAL_TEST | FLAG_EXTERNAL_TEST)),
            long(0xc2, 1),
            long(0xc2, 1 << 26),
            long(0xc2, 1 << 16),
            long(0xc6, 0),
        ] {
            let mut h = board(&[]);
            h.write(&bad);
            assert_eq!(violations(&mut h), 1, "{bad:02x?}");
        }
    }

    #[test]
    fn a_run_needs_its_count_and_a_group() {
        let mut h = board(&[]);
        h.write(&[RUN]);
        assert_eq!(violations(&mut h), 1);
        let mut h = board(&[]);
        setup(&mut h, 0x3c, 0, 0);
        h.write(&[RUN]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn only_a_reset_comes_after_a_run() {
        let mut h = board(&[]);
        setup(&mut h, 0x38, 0, 0);
        h.write(&[RUN]);
        h.write(&[ID]);
        assert_eq!(violations(&mut h), 1);
        h.write(&long(SET_DIVIDER, 0));
        assert_eq!(violations(&mut h), 1);
        h.write(&[RESET]);
        h.write(&long(SET_DIVIDER, 0));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_command_may_come_in_pieces_and_an_unfinished_one_is_flagged() {
        let mut h = board(&[]);
        h.write(&[SET_DIVIDER, 0]);
        h.write(&[0, 0, 0, ID]);
        assert_eq!(read_all(&mut h), b"1SLO");
        assert_eq!(violations(&mut h), 0);
        h.write(&[SET_DIVIDER, 0]);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_reset_clears_the_stages() {
        let mut h = board(&[]);
        h.write(&long(0xc0, 1));
        h.write(&long(0xc2, START));
        h.write(&[RESET]);
        h.write(&long(SET_SIZE, 0));
        h.write(&long(SET_FLAGS, 0x38));
        h.write(&[RUN]);
        // Nothing has a start bit any more.
        assert_eq!(read_all(&mut h), b"");
    }

    #[test]
    fn behind_a_uart_it_wants_the_line() {
        let mut h = board(&[("uart", "115200/8n1")]);
        h.write(&[ID]);
        assert_eq!(read_all(&mut h), b"");
        h.set_line(Line::parse("115200/8n1").unwrap());
        h.write(&[ID]);
        assert_eq!(read_all(&mut h), b"1SLO");
    }

    #[test]
    fn options_are_checked() {
        for (k, v) in [("nope", "1"), ("memory", "0"), ("memory", "2097152"), ("slow", "0")] {
            let o = vec![(k.to_owned(), v.to_owned())];
            assert!(function(&o).is_err(), "{k}={v}");
        }
        let mut h = board(&[("id", "414243")]);
        h.write(&[ID]);
        assert_eq!(read_all(&mut h), b"ABC");
    }
}
