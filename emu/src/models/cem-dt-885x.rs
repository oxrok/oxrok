// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A CEM DT-8852 sound level meter on the CDC-ACM shell. It talks all the time
//! with no one asking, in packets of a sync byte `a5`, a token and up to three
//! data bytes, and takes one-byte commands that it never acknowledges.
//!
//! The device, and how sure this is:
//!
//! - The USB side: the sigrok wiki's `lsusb` shows a CP2102, `10c4:ea60`. This
//!   model is NOT that: `cp210x` binds those ids whatever the class, so it is a
//!   CDC-ACM function with the shell's descriptors and test ids, an artifact of
//!   the emulation. The tools see a tty either way.
//! - The line, 9600/8n1, the tokens, how many data bytes each has, and the
//!   commands: the wiki's protocol section.
//! - The order of the stream: the `dt8852` Python package (1.1.0, written for a
//!   Trotec SL400 of the same family), in a comment: eight short blocks of range,
//!   hold, threshold, level, display, frequency weighting and time; then one long
//!   block whose first part is store, time weighting, battery and recording. Here a
//!   long block is one in four: see [`CYCLE`].
//! - The stored log: the wiki. `bb`, a big-endian length plus 100, then for each
//!   record `aa` or `cc`, six BCD fields of date and time and an interval, then
//!   `ac` and the levels, and last `dd`. Its two firmware bugs are here: an empty
//!   log still has an `aa`, and the last record ends in half a level, with a length
//!   two more than the data it counts.
//! - The interval's encoding is disputed: the wiki says BCD, and the package reads
//!   the byte as binary, as a C reader would. This model holds a byte, and sends
//!   what `log=` gives it.
//!
//! What is an inference, as no source says:
//!
//! - The value of the data byte after `0b`, `1b` and `1c`. Both sources skip it.
//!   It is 0 here.
//! - The order in which `88` steps the ranges: 30-130, 30-80, 50-100, 80-130. The
//!   wiki only calls it a toggle. Same for `11`, as none, max, min; the wiki names
//!   them "Max/Min/normal".
//! - That the meter sends a stream of `ff` in hold, and nothing else: only a driver's
//!   comment says so, and no other source mentions it.
//! - That `33` silences it at once, and that a meter off hears nothing.
//! - How often the display's reading comes among the bar graph's. `bar=` sets it.
//!
//! Emulation artifacts:
//!
//! - A block goes out per `period=` offers of a read, not 20 per second, and the
//!   level changes per block rather than with the sound.
//! - A command's token goes out at once, ahead of what the stream would bring,
//!   where a real one shows it some blocks later, and often not at all (the wiki:
//!   commands "nearly always have to be resent"). A driver's few ms between a
//!   command and giving up are then not raced by the offers. `deaf=N` loses the
//!   next N commands, and the token that follows shows the old state.
//! - After each open, `startup=` blocks hold no level: an open's flush may or may
//!   not drop the first of them, and no sample may depend on that. So a level
//!   comes a few blocks later than on a real one. The last of them is a long block,
//!   so the first sample has every flag.
//! - The log goes out as one whole, once per open, and the stream stops meanwhile.
//!   A tool keeps asking until it has read `bb`: what comes while the log is being
//!   sent, or later, is not asked again. The wiki says only that it is inserted into
//!   the stream.
//! - The time of day never moves.
//! - Each oxrok open raises the lines twice, where sigrok-cli's does once, and a
//!   model can't tell that from two opens: so nothing here counts opens. The blocks
//!   start over at each, which the startup blocks hide.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `freq=A|C`, `time=F|S`, `hold-mode=none|max|min`, `range=30-130|30-80|50-100|80-130`,
//!   `recording=0|1`, `battery=ok|low`, `store=ok|full`, `threshold=ok|over|under`, the
//!   state at power on;
//! - `levels=HEX[,HEX...]`, two BCD bytes per level, played in turn and over;
//! - `bar=N`, bar graph blocks per reading on the display;
//! - `extra=HEX[,HEX...]`, bytes ahead of the level of the next blocks, one entry
//!   each, for tokens no source names and other noise;
//! - `period=N`, offers of a read per block, the first block of an open at the second
//!   offer: see [`PERIOD`];
//! - `first-block=N`, offers of a read with nothing after each open: a C get's first
//!   read then has no byte to find, as it mostly has none on a real line;
//! - `startup=N`, blocks with no level after each open;
//! - `blocks=N`, blocks per open, then silence and the event point;
//! - `time-only=1`, blocks of nothing but the time of day: a scan finds the meter,
//!   and nothing a get waits for or a capture counts ever comes;
//! - `mark=N`, the event point at the Nth level of an open, with the stream going on;
//! - `hold-after=N`, `ff` in place of blocks after N levels of an open;
//!   `hold-blocks=N`, the same after N blocks of an open; `hold-for=N`, as many `ff`
//!   and then the blocks again;
//! - `log=RECORD[,RECORD...]`, each `a` or `c`, the interval's byte in hex, `:` and
//!   the number of levels; `log=empty` for the wiki's empty log; `dump=HEX` for the
//!   bytes as they are;
//! - `no-level=1`, blocks with no level, ever: a log's capture reads what the meter
//!   said since the open first, and how much that is depends on the host;
//! - `announce=0`, no token at once for a command: what comes is the stream's;
//! - `mute=1`, nothing more after the first command;
//! - `deaf=N`, see above; `setters=0`, which flags any command, for what must
//!   change nothing;
//! - `uart=LINE`.
//!
//! The pedantic check: with `setters=0`, any command. A meter takes any byte it
//! hears, so there is little else a driver can do wrong that it could tell. A byte
//! that is no command is passed over, not flagged, because of the harness: `stty` is
//! the tty's first open, with `ECHO` still on, and sends back whatever this model
//! streams while it runs. So nothing flags a wrong byte from a driver, and a `.tx`
//! has to show it.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "cem-dt-885x";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A CEM DT-8852 sound level meter on CDC-ACM; freq=, time=, hold-mode=, \
                         range=, recording=, battery=, store=, threshold=, levels=, bar=, extra=, \
                         period=, first-block=, startup=, blocks=, time-only=, mark=, hold-after=, \
                         hold-blocks=, hold-for=, log=, dump=, announce=, mute=, no-level=, deaf=, setters=, uart=, and the shell's stall=, \
                         chunk=, in-limit=, in-stall=, out-hold=";

const SYNC: u8 = 0xa5;
const HOLD: u8 = 0xff;
const LOG_START: u8 = 0xbb;
const RECORD_A: u8 = 0xaa;
const RECORD_C: u8 = 0xcc;
const RECORD_LEVELS: u8 = 0xac;
const LOG_END: u8 = 0xdd;
/// The wiki's offset on the log's length.
const LENGTH_OFFSET: usize = 100;
/// Blocks per long one, where the package has 9: at this model's [`PERIOD`] that
/// would miss a driver's 510 ms for the recording token.
const CYCLE: u32 = 4;
/// The time of day: BCD, with the wiki's "first nibble unknown" as the package's PM.
const TIME_OF_DAY: [u8; 3] = [0x32, 0x34, 0x56];
/// The date and time of each record.
const RECORD_START: [u8; 6] = [0x26, 0x11, 0x05, 0x12, 0x34, 0x56];
/// The half a level that ends a log.
const STRAY: u8 = 0x45;
/// The longest log `dump=` takes, and an entry of `extra=` and `levels=`.
const LONGEST_DUMP: usize = 8192;
const LONGEST_EXTRA: usize = 256;
/// The most levels a record of `log=` holds.
const LONGEST_RECORD: u32 = 2000;
/// Blocks with no level after an open.
const STARTUP: u32 = 2;
/// Offers of a read per block. Offers came at some 7 to 12 ms each on the testers:
/// about 60 to 100 ms per block, against the wiki's 50. A shorter one loses
/// the session: with reads completing all the time, a tool's close hits the kernel's
/// `CMD_UNLINK` ahead of `CMD_SUBMIT` (`docs/ROADMAP.md`). Seen with `vhci_hcd`'s
/// "cannot find a urb of seqnum": at 1 within some 20 closes, at 2 within some 30, at
/// 4 about once per 500, and with `chunk=` within 60.
const PERIOD: u32 = 8;
const DEFAULT_LOG: &str = "a01:3,c10:2";

const TOGGLE_POWER: u8 = 0x33;
const TOGGLE_RECORDING: u8 = 0x55;
const TOGGLE_HOLD: u8 = 0x11;
const TOGGLE_TIME: u8 = 0x77;
const TOGGLE_RANGE: u8 = 0x88;
const TOGGLE_FREQ: u8 = 0x99;
const SEND_LOG: u8 = RECORD_LEVELS;

/// The tokens of the ranges, in the order `88` steps them.
const RANGES: [(&str, u8); 4] =
    [("30-130", 0x40), ("30-80", 0x30), ("50-100", 0x4b), ("80-130", 0x4c)];
/// The tokens of the hold modes, in the order `11` steps them.
const HOLD_MODES: [(&str, u8); 3] = [("none", 0x0e), ("max", 0x04), ("min", 0x05)];

/// A meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("9600/8n1")?;
    let mut config = Config::new()?;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "freq" => config.freq_c = choice(k, v, &["A", "C"])? == 1,
            "time" => config.time_slow = choice(k, v, &["F", "S"])? == 1,
            "hold-mode" => {
                config.hold_mode = choice(k, v, &HOLD_MODES.map(|m| m.0))?;
            }
            "range" => config.range = choice(k, v, &RANGES.map(|r| r.0))?,
            "recording" => config.recording = parse_number(k, v, 0..=1)? == 1,
            "battery" => config.battery_low = choice(k, v, &["ok", "low"])? == 1,
            "store" => config.store_full = choice(k, v, &["ok", "full"])? == 1,
            "threshold" => {
                config.threshold = [0x11, 0x07, 0x08][choice(k, v, &["ok", "over", "under"])?];
            }
            "levels" => config.levels = parse_levels(k, v)?,
            "bar" => config.bar = parse_number(k, v, 0..=1000)?,
            "extra" => config.extra = parse_hex_list(k, v, 1, LONGEST_EXTRA)?,
            "period" => config.period = parse_number(k, v, 1..=100_000)?,
            "first-block" => config.first_block = parse_number(k, v, 0..=100_000)?,
            "startup" => config.startup = parse_number(k, v, 0..=1000)?,
            "blocks" => config.blocks = Some(parse_number(k, v, 0..=1_000_000)?),
            "time-only" => config.time_only = parse_number(k, v, 0..=1)? == 1,
            "mark" => config.mark = Some(parse_number(k, v, 1..=1_000_000)?),
            "hold-after" => config.hold_after = Some(parse_number(k, v, 0..=1_000_000)?),
            "hold-blocks" => config.hold_blocks = Some(parse_number(k, v, 0..=1_000_000)?),
            "hold-for" => config.hold_for = Some(parse_number(k, v, 0..=1_000_000)?),
            "log" => config.log = parse_log(k, v)?,
            "dump" => {
                let mut list = parse_hex_list(k, v, 1, LONGEST_DUMP)?;
                config.log = list.remove(0);
            }
            "announce" => config.announce = parse_number(k, v, 0..=1)? == 1,
            "mute" => config.mute = parse_number(k, v, 0..=1)? == 1,
            "no-level" => config.no_level = parse_number(k, v, 0..=1)? == 1,
            "deaf" => config.deaf = parse_number(k, v, 0..=1_000_000)?,
            "setters" => config.setters = parse_number(k, v, 0..=1)? == 1,
            "uart" => uart = Line::parse(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    // Test ids: see the module's note on the real CP2102.
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "CEM DT-8852 stand-in");
    Ok(Cdc::new(NAME, &id, faults, Meter::new(config)).behind_uart(uart))
}

fn choice(key: &str, value: &str, names: &[&str]) -> Result<usize> {
    names
        .iter()
        .position(|n| *n == value)
        .ok_or_else(|| Error::Invalid(format!("{key}={value}: not one of {}", names.join(" "))))
}

fn parse_levels(key: &str, value: &str) -> Result<Vec<[u8; 2]>> {
    parse_hex_list(key, value, 2, 2)?
        .into_iter()
        .map(|l| <[u8; 2]>::try_from(l.as_slice()).map_err(|_| Error::Invalid(key.to_owned())))
        .collect()
}

/// `value` as BCD, with the two digits a level's byte has.
fn bcd(value: u32) -> u8 {
    u8::try_from(((value / 10 % 10) << 4) | (value % 10)).expect("two digits")
}

/// The log's bytes for a `log=`.
fn parse_log(key: &str, value: &str) -> Result<Vec<u8>> {
    let bad = || Error::Invalid(format!("{key}={value}: wants a01:3,c10:2 or empty"));
    if value == "empty" {
        return Ok(vec![LOG_START, 0x00, LENGTH_OFFSET as u8, RECORD_A, LOG_END]);
    }
    let mut body = Vec::new();
    let mut counted = 0_usize;
    // The levels of a log climb, so that a lost or a repeated one shows.
    let mut tenths = 400_u32;
    for record in value.split(',') {
        let (head, count) = record.split_once(':').ok_or_else(bad)?;
        let count = parse_number(key, count, 0..=LONGEST_RECORD)?;
        let token = match head.as_bytes().first() {
            Some(b'a') => RECORD_A,
            Some(b'c') => RECORD_C,
            _ => return Err(bad()),
        };
        let interval = u8::from_str_radix(head.get(1..).ok_or_else(bad)?, 16).map_err(|_| bad())?;
        body.push(token);
        body.extend_from_slice(&RECORD_START);
        body.push(interval);
        counted += 2 + RECORD_START.len();
        body.push(RECORD_LEVELS);
        for _ in 0..count {
            body.extend_from_slice(&[bcd(tenths / 100), bcd(tenths % 100)]);
            tenths = if tenths >= 1299 { 400 } else { tenths + 1 };
        }
        counted += 2 * count as usize;
    }
    body.push(STRAY);
    counted += 1;
    // The wiki's "off by one": one byte less comes than the length says.
    let length = u16::try_from(counted + 1 + LENGTH_OFFSET).map_err(|_| bad())?;
    let mut log = vec![LOG_START];
    log.extend_from_slice(&length.to_be_bytes());
    log.extend_from_slice(&body);
    log.push(LOG_END);
    Ok(log)
}

#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)]
struct Config {
    freq_c: bool,
    time_slow: bool,
    hold_mode: usize,
    range: usize,
    recording: bool,
    battery_low: bool,
    store_full: bool,
    threshold: u8,
    levels: Vec<[u8; 2]>,
    bar: u32,
    extra: Vec<Vec<u8>>,
    period: u32,
    first_block: u32,
    startup: u32,
    blocks: Option<u32>,
    time_only: bool,
    mark: Option<u32>,
    hold_after: Option<u32>,
    hold_blocks: Option<u32>,
    hold_for: Option<u32>,
    log: Vec<u8>,
    announce: bool,
    mute: bool,
    no_level: bool,
    deaf: u32,
    setters: bool,
}

impl Config {
    fn new() -> Result<Config> {
        Ok(Config {
            freq_c: false,
            time_slow: false,
            hold_mode: 0,
            range: 0,
            recording: false,
            battery_low: false,
            store_full: false,
            threshold: 0x11,
            // None is a command's byte: see the note on `stty` under the pedantic check.
            levels: vec![[0x06, 0x54], [0x06, 0x61], [0x07, 0x02], [0x12, 0x98]],
            bar: 1,
            extra: Vec::new(),
            period: PERIOD,
            first_block: 0,
            startup: STARTUP,
            blocks: None,
            time_only: false,
            mark: None,
            hold_after: None,
            hold_blocks: None,
            hold_for: None,
            log: parse_log("log", DEFAULT_LOG)?,
            announce: true,
            mute: false,
            no_level: false,
            deaf: 0,
            setters: true,
        })
    }
}

/// The meter.
struct Meter {
    config: Config,
    freq_c: bool,
    time_slow: bool,
    hold_mode: usize,
    range: usize,
    recording: bool,
    deaf: u32,
    off: bool,
    /// Offers since the last block.
    offers: u32,
    /// Offers left of `first-block=`.
    quiet: u32,
    /// Blocks and levels of this open.
    blocks: u32,
    levels: u32,
    /// `ff` sent since the hold began.
    held: u32,
    /// The next of `levels=` and of `extra=`, of this open: how many blocks a scan
    /// saw must not show in a capture.
    level: usize,
    extra: usize,
    /// Which of the `bar=` a level is.
    bars: u32,
    /// The log has gone out in this open.
    sent_log: bool,
    /// `mute=` has struck.
    muted: bool,
    marked: bool,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter {
            freq_c: config.freq_c,
            time_slow: config.time_slow,
            hold_mode: config.hold_mode,
            range: config.range,
            recording: config.recording,
            deaf: config.deaf,
            off: false,
            offers: 0,
            quiet: 0,
            blocks: 0,
            levels: 0,
            held: 0,
            level: 0,
            extra: 0,
            bars: 0,
            sent_log: false,
            muted: false,
            marked: false,
            config,
        }
    }

    fn freq_packet(&self) -> [u8; 3] {
        [SYNC, if self.freq_c { 0x1c } else { 0x1b }, 0]
    }

    fn time_packet(&self) -> [u8; 2] {
        [SYNC, if self.time_slow { 0x03 } else { 0x02 }]
    }

    fn recording_packet(&self) -> [u8; 2] {
        [SYNC, if self.recording { 0x0a } else { 0x1a }]
    }

    fn range_packet(&self) -> [u8; 2] {
        [SYNC, RANGES[self.range].1]
    }

    fn hold_packet(&self) -> [u8; 2] {
        [SYNC, HOLD_MODES[self.hold_mode].1]
    }

    /// Whether this block is the long one, placed so the last of the startup
    /// blocks is one.
    fn long_block(&self) -> bool {
        (self.blocks + CYCLE - (self.config.startup + CYCLE - 1) % CYCLE) % CYCLE == 0
    }

    /// The level, the display's token and, for a level, the noise ahead of them.
    fn level_part(&mut self, out: &mut Vec<u8>) {
        if self.config.extra.len() > self.extra {
            out.extend_from_slice(&self.config.extra[self.extra]);
            self.extra += 1;
        }
        let level = self.config.levels[self.level % self.config.levels.len()];
        self.level = self.level.wrapping_add(1);
        out.extend_from_slice(&[SYNC, 0x0d, level[0], level[1]]);
        if self.bars < self.config.bar {
            self.bars += 1;
            out.extend_from_slice(&[SYNC, 0x0c]);
        } else {
            self.bars = 0;
            out.extend_from_slice(&[SYNC, 0x0b, 0]);
        }
    }

    /// The next block's bytes.
    fn block(&mut self) -> Vec<u8> {
        let mut out = Vec::new();
        if self.config.time_only {
            self.blocks += 1;
            out.extend_from_slice(&[SYNC, 0x06]);
            out.extend_from_slice(&TIME_OF_DAY);
            return out;
        }
        if self.long_block() {
            out.extend_from_slice(&[SYNC, if self.config.store_full { 0x09 } else { 0x19 }]);
            out.extend_from_slice(&self.time_packet());
            out.extend_from_slice(&[SYNC, if self.config.battery_low { 0x0f } else { 0x1f }]);
            out.extend_from_slice(&self.recording_packet());
        } else {
            out.extend_from_slice(&self.range_packet());
            out.extend_from_slice(&self.hold_packet());
            out.extend_from_slice(&[SYNC, self.config.threshold]);
        }
        if self.blocks >= self.config.startup && !self.config.no_level {
            self.levels += 1;
            self.level_part(&mut out);
        }
        out.extend_from_slice(&self.freq_packet());
        out.extend_from_slice(&[SYNC, 0x06]);
        out.extend_from_slice(&TIME_OF_DAY);
        self.blocks += 1;
        out
    }

    /// Whether `ff` goes out now rather than a block.
    fn holding(&self) -> bool {
        let at = self.config.hold_after.is_some_and(|n| self.levels >= n)
            || self.config.hold_blocks.is_some_and(|n| self.blocks >= n);
        at && self.config.hold_for.is_none_or(|n| self.held < n)
    }

    fn command(&mut self, byte: u8, port: &mut Port<'_>) {
        let known = matches!(
            byte,
            SEND_LOG
                | TOGGLE_POWER
                | TOGGLE_RECORDING
                | TOGGLE_HOLD
                | TOGGLE_TIME
                | TOGGLE_RANGE
                | TOGGLE_FREQ
        );
        if !known {
            return;
        }
        if !self.config.setters {
            port.violation(format!("{byte:02x}: a command no one asked for"));
        }
        if byte == SEND_LOG {
            if !std::mem::replace(&mut self.sent_log, true) && !self.off {
                port.send(&self.config.log);
            }
            return;
        }
        if self.off || self.muted {
            return;
        }
        self.muted = self.config.mute;
        let heard = self.deaf == 0;
        self.deaf = self.deaf.saturating_sub(1);
        let answer: Vec<u8> = match byte {
            TOGGLE_POWER => {
                if heard {
                    self.off = true;
                    Vec::new()
                } else {
                    // Not lost on a meter that hears the next one: it goes on.
                    self.block()
                }
            }
            TOGGLE_RECORDING => {
                if heard {
                    self.recording = !self.recording;
                }
                self.recording_packet().to_vec()
            }
            TOGGLE_HOLD => {
                if heard {
                    self.hold_mode = (self.hold_mode + 1) % HOLD_MODES.len();
                }
                self.hold_packet().to_vec()
            }
            TOGGLE_TIME => {
                if heard {
                    self.time_slow = !self.time_slow;
                }
                self.time_packet().to_vec()
            }
            TOGGLE_RANGE => {
                if heard {
                    self.range = (self.range + 1) % RANGES.len();
                }
                self.range_packet().to_vec()
            }
            _ => {
                if heard {
                    self.freq_c = !self.freq_c;
                }
                self.freq_packet().to_vec()
            }
        };
        if self.config.announce && !self.config.mute {
            port.send(&answer);
        }
    }

    /// Silent after `blocks=` of this open, or once off.
    fn silent(&self) -> bool {
        self.off || self.muted || self.config.blocks.is_some_and(|n| self.blocks >= n)
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for byte in data {
            self.command(*byte, port);
        }
    }

    fn changed(&mut self, event: LineEvent, _port: &mut Port<'_>) {
        if event.opened() {
            self.offers = self.config.period.saturating_sub(2);
            self.quiet = self.config.first_block;
            self.blocks = 0;
            self.levels = 0;
            self.held = 0;
            self.level = 0;
            self.extra = 0;
            self.bars = 0;
            self.sent_log = false;
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if self.quiet > 0 {
            self.quiet -= 1;
            return;
        }
        self.offers = self.offers.saturating_add(1);
        if self.offers < self.config.period {
            return;
        }
        self.offers = 0;
        if self.silent() {
            if !self.off && !self.marked {
                self.marked = true;
                port.mark();
            }
            return;
        }
        if self.holding() {
            self.held = self.held.saturating_add(1);
            port.send(&[HOLD]);
            return;
        }
        let before = self.levels;
        let block = self.block();
        port.send(&block);
        if self.levels != before && self.config.mark == Some(self.levels) && !self.marked {
            self.marked = true;
            port.mark();
        }
    }

    fn reset(&mut self) {
        *self = Meter::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    fn options(options: &[(&str, &str)]) -> Vec<(String, String)> {
        options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    fn meter(o: &[(&str, &str)]) -> Host<Meter> {
        let mut all = vec![("period", "1")];
        all.extend_from_slice(o);
        let cdc = function(&options(&all)).unwrap();
        Host::open(cdc, Line::parse("9600/8n1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Meter>) -> Vec<String> {
        h.v.drain()
    }

    /// One read of the kernel's. `Host::read` reads all there is, and on a meter
    /// that never stops talking there is no end to that.
    fn one(h: &mut Host<Meter>) -> Vec<u8> {
        match h.offer() {
            Reply::Data(d) => d,
            _ => Vec::new(),
        }
    }

    /// The next block, with a bound.
    fn block(h: &mut Host<Meter>) -> Vec<u8> {
        for _ in 0..50 {
            let got = one(h);
            if !got.is_empty() {
                return got;
            }
        }
        Vec::new()
    }

    /// A stream's tokens, by the wiki's data lengths.
    fn tokens(bytes: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut i = 0;
        while i < bytes.len() {
            assert_eq!(bytes[i], SYNC, "{bytes:02x?} at {i}");
            let token = bytes[i + 1];
            out.push(token);
            i += 2 + match token {
                0x06 => 3,
                0x0d => 2,
                0x0b | 0x1b | 0x1c => 1,
                _ => 0,
            };
        }
        out
    }

    #[test]
    fn the_stream_is_the_packages_order_and_startup_has_no_level() {
        let mut h = meter(&[("startup", "3")]);
        assert_eq!(tokens(&block(&mut h)), [0x40, 0x0e, 0x11, 0x1b, 0x06]);
        assert_eq!(tokens(&block(&mut h)), [0x40, 0x0e, 0x11, 0x1b, 0x06]);
        assert_eq!(tokens(&block(&mut h)), [0x19, 0x02, 0x1f, 0x1a, 0x1b, 0x06]);
        assert_eq!(tokens(&block(&mut h)), [0x40, 0x0e, 0x11, 0x0d, 0x0c, 0x1b, 0x06]);
        assert_eq!(tokens(&block(&mut h)), [0x40, 0x0e, 0x11, 0x0d, 0x0b, 0x1b, 0x06]);
        assert_eq!(tokens(&block(&mut h)), [0x40, 0x0e, 0x11, 0x0d, 0x0c, 0x1b, 0x06]);
        // Four blocks on from the last long one.
        assert_eq!(tokens(&block(&mut h)), [0x19, 0x02, 0x1f, 0x1a, 0x0d, 0x0b, 0x1b, 0x06]);
        assert_eq!(violations(&mut h), Vec::<String>::new());
    }

    #[test]
    fn the_state_shows_in_the_stream() {
        let mut h = meter(&[
            ("freq", "C"),
            ("time", "S"),
            ("hold-mode", "max"),
            ("range", "50-100"),
            ("recording", "1"),
            ("battery", "low"),
            ("store", "full"),
            ("threshold", "over"),
            ("startup", "1"),
        ]);
        assert_eq!(tokens(&block(&mut h)), [0x09, 0x03, 0x0f, 0x0a, 0x1c, 0x06]);
        assert_eq!(tokens(&block(&mut h)), [0x4b, 0x04, 0x07, 0x0d, 0x0c, 0x1c, 0x06]);
    }

    #[test]
    fn levels_are_played_in_turn_and_bar_sets_the_displays_share() {
        let mut h = meter(&[("startup", "0"), ("levels", "0123,0456"), ("bar", "0")]);
        let first = block(&mut h);
        assert!(first.windows(7).any(|w| w == [SYNC, 0x0d, 0x01, 0x23, SYNC, 0x0b, 0]));
        assert!(block(&mut h).windows(4).any(|w| w == [SYNC, 0x0d, 0x04, 0x56]));
        assert!(block(&mut h).windows(4).any(|w| w == [SYNC, 0x0d, 0x01, 0x23]));
    }

    #[test]
    fn each_toggle_shows_at_once_and_steps_in_order() {
        let mut h = meter(&[]);
        h.write(&[TOGGLE_FREQ]);
        assert_eq!(one(&mut h), [SYNC, 0x1c, 0]);
        h.write(&[TOGGLE_FREQ]);
        assert_eq!(one(&mut h), [SYNC, 0x1b, 0]);
        h.write(&[TOGGLE_TIME]);
        assert_eq!(one(&mut h), [SYNC, 0x03]);
        h.write(&[TOGGLE_RECORDING]);
        assert_eq!(one(&mut h), [SYNC, 0x0a]);
        for token in [0x04, 0x05, 0x0e] {
            h.write(&[TOGGLE_HOLD]);
            assert_eq!(one(&mut h), [SYNC, token]);
        }
        for token in [0x30, 0x4b, 0x4c, 0x40] {
            h.write(&[TOGGLE_RANGE]);
            assert_eq!(one(&mut h), [SYNC, token]);
        }
    }

    #[test]
    fn first_block_holds_each_open_back() {
        let mut h = meter(&[("first-block", "2")]);
        assert_eq!(one(&mut h), Vec::<u8>::new());
        assert_eq!(one(&mut h), Vec::<u8>::new());
        assert_eq!(one(&mut h)[0], SYNC);
        h.set_lines(true, true);
        assert_eq!(one(&mut h), Vec::<u8>::new());
    }

    #[test]
    fn no_level_has_none_and_mute_ends_all_at_the_first_command() {
        let mut h = meter(&[("no-level", "1"), ("startup", "0")]);
        assert_eq!(tokens(&block(&mut h)), [0x40, 0x0e, 0x11, 0x1b, 0x06]);
        let mut h = meter(&[("mute", "1")]);
        assert_eq!(block(&mut h)[0], SYNC);
        h.write(&[TOGGLE_FREQ]);
        assert_eq!(block(&mut h), Vec::<u8>::new());
        h.write(&[TOGGLE_FREQ]);
        assert_eq!(one(&mut h), Vec::<u8>::new());
    }

    #[test]
    fn announce_0_leaves_it_to_the_stream() {
        let mut h = meter(&[("announce", "0"), ("blocks", "0")]);
        h.write(&[TOGGLE_FREQ]);
        assert_eq!(one(&mut h), Vec::<u8>::new());
    }

    #[test]
    fn a_deaf_meter_loses_commands_and_shows_the_old_state() {
        let mut h = meter(&[("deaf", "1")]);
        h.write(&[TOGGLE_FREQ]);
        assert_eq!(one(&mut h), [SYNC, 0x1b, 0]);
        h.write(&[TOGGLE_FREQ]);
        assert_eq!(one(&mut h), [SYNC, 0x1c, 0]);
    }

    #[test]
    fn power_off_silences_it_for_good_and_a_deaf_one_goes_on() {
        let mut h = meter(&[("deaf", "1")]);
        h.write(&[TOGGLE_POWER]);
        assert!(!one(&mut h).is_empty(), "lost: the next block comes");
        h.write(&[TOGGLE_POWER]);
        for _ in 0..20 {
            assert_eq!(one(&mut h), Vec::<u8>::new());
        }
        // A meter off hears nothing: not even a log.
        h.write(&[TOGGLE_FREQ, SEND_LOG]);
        assert_eq!(one(&mut h), Vec::<u8>::new());
        assert_eq!(violations(&mut h), Vec::<String>::new());
    }

    #[test]
    fn the_log_goes_out_once_with_the_wikis_two_bugs() {
        let mut h = meter(&[("log", "a01:1")]);
        h.write(&[SEND_LOG]);
        // Eight of the head, two of a level, the stray byte: 11, and one more.
        // All of it fits one read.
        assert_eq!(
            one(&mut h),
            [
                LOG_START, 0x00, 112, RECORD_A, 0x26, 0x11, 0x05, 0x12, 0x34, 0x56, 0x01,
                RECORD_LEVELS, 0x04, 0x00, STRAY, LOG_END
            ]
        );
        h.write(&[SEND_LOG, SEND_LOG]);
        assert!(!one(&mut h).contains(&LOG_START), "asked again while it is read");
        assert_eq!(violations(&mut h), Vec::<String>::new());
    }

    #[test]
    fn an_empty_log_has_the_wikis_stray_record_token() {
        let mut h = meter(&[("log", "empty")]);
        h.write(&[SEND_LOG]);
        assert_eq!(one(&mut h), [LOG_START, 0x00, 0x64, RECORD_A, LOG_END]);
    }

    #[test]
    fn the_default_log_climbs_over_two_records() {
        let log = parse_log("log", DEFAULT_LOG).unwrap();
        assert_eq!(&log[..3], [LOG_START, 0x00, 128]);
        assert_eq!(log[3], RECORD_A);
        assert_eq!(log[12..18], [0x04, 0x00, 0x04, 0x01, 0x04, 0x02]);
        assert_eq!(log[18], RECORD_C);
        assert_eq!(log[25], 0x10, "the interval's byte, as given");
        assert_eq!(log[26], RECORD_LEVELS);
        assert_eq!(log[27..31], [0x04, 0x03, 0x04, 0x04]);
        assert_eq!(log[31..], [STRAY, LOG_END]);
    }

    #[test]
    fn dump_is_sent_as_it_is_and_log_is_checked() {
        let mut h = meter(&[("dump", "bb0064dd")]);
        h.write(&[SEND_LOG]);
        assert_eq!(one(&mut h), [LOG_START, 0x00, 0x64, LOG_END]);
        for bad in ["x01:1", "a01", "azz:1", "a01:99999", ""] {
            assert!(parse_log("log", bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn extra_comes_ahead_of_the_next_levels_once() {
        let mut h = meter(&[("startup", "0"), ("extra", "a5e1,0102")]);
        assert!(block(&mut h).windows(4).any(|w| w == [0xa5, 0xe1, SYNC, 0x0d]));
        assert!(block(&mut h).windows(4).any(|w| w == [0x01, 0x02, SYNC, 0x0d]));
        let third = block(&mut h);
        assert!(!third.windows(2).any(|w| w == [0x01, 0x02] || w == [0xa5, 0xe1]));
    }

    #[test]
    fn hold_replaces_the_blocks_and_ends_where_told() {
        let mut h = meter(&[("startup", "0"), ("hold-after", "1"), ("hold-for", "2")]);
        assert!(block(&mut h).contains(&0x0d));
        assert_eq!(block(&mut h), [HOLD]);
        assert_eq!(block(&mut h), [HOLD]);
        assert_eq!(block(&mut h)[0], SYNC);
    }

    #[test]
    fn hold_blocks_counts_blocks_with_no_level() {
        let mut h = meter(&[("hold-blocks", "1")]);
        assert_eq!(block(&mut h)[0], SYNC);
        assert_eq!(block(&mut h), [HOLD]);
    }

    #[test]
    fn time_only_has_nothing_else() {
        let mut h = meter(&[("time-only", "1"), ("startup", "0")]);
        assert_eq!(tokens(&block(&mut h)), [0x06]);
        assert_eq!(tokens(&block(&mut h)), [0x06]);
    }

    #[test]
    fn blocks_bound_the_stream_and_period_paces_it() {
        let mut h = meter(&[("blocks", "2")]);
        assert_eq!(block(&mut h)[0], SYNC);
        assert_eq!(block(&mut h)[0], SYNC);
        assert_eq!(block(&mut h), Vec::<u8>::new());
        h.set_lines(true, true);
        assert_eq!(block(&mut h)[0], SYNC);
        let mut h = meter(&[("period", "3")]);
        assert_eq!(one(&mut h), Vec::<u8>::new());
        assert_eq!(one(&mut h)[0], SYNC, "the first soon, for a scan's 25 ms");
        assert_eq!(one(&mut h), Vec::<u8>::new());
        assert_eq!(one(&mut h), Vec::<u8>::new());
        assert_eq!(one(&mut h)[0], SYNC);
    }

    #[test]
    fn a_new_open_starts_the_blocks_over_and_the_log_may_go_again() {
        let mut h = meter(&[("log", "empty")]);
        h.write(&[SEND_LOG]);
        assert_eq!(one(&mut h)[0], LOG_START);
        assert_eq!(block(&mut h)[0], SYNC);
        h.set_lines(true, true);
        h.write(&[SEND_LOG]);
        assert_eq!(one(&mut h)[0], LOG_START);
    }

    #[test]
    fn the_pedantic_checks_fire() {
        let mut h = meter(&[]);
        // What a tty's echo sends back of the stream is no command.
        h.write(&[0x42, SYNC, 0x5e]);
        assert_eq!(one(&mut h)[0], SYNC, "a block, and no answer");
        assert_eq!(violations(&mut h), Vec::<String>::new());
        let mut h = meter(&[("setters", "0")]);
        h.write(&[0x42]);
        assert_eq!(violations(&mut h), Vec::<String>::new());
        h.write(&[TOGGLE_FREQ]);
        assert_eq!(violations(&mut h).len(), 1, "toggle");
        h.write(&[SEND_LOG]);
        assert_eq!(violations(&mut h).len(), 1, "log");
    }

    #[test]
    fn the_wrong_line_hears_nothing() {
        let cdc = function(&options(&[])).unwrap();
        let mut h = Host::open(cdc, Line::parse("2400/8n1").unwrap(), true, true);
        assert_eq!(one(&mut h), Vec::<u8>::new());
        h.write(&[TOGGLE_FREQ]);
        assert_eq!(one(&mut h), Vec::<u8>::new());
    }

    #[test]
    fn options_are_checked() {
        for bad in [
            ("freq", "Z"),
            ("time", "Z"),
            ("hold-mode", "z"),
            ("range", "1-2"),
            ("battery", "z"),
            ("store", "z"),
            ("threshold", "z"),
            ("levels", "01"),
            ("no-such", "1"),
        ] {
            assert!(function(&options(&[bad])).is_err(), "{bad:?}");
        }
    }
}
