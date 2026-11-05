// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A PCE PCE-322A sound level meter on the CDC-ACM shell: thirteen-byte
//! measurements on a stream of their own, two-byte commands, and a memory read
//! in blocks of 256 bytes.
//!
//! The device, and how sure this is:
//!
//! - The USB side: no source names it. The sigrok wiki's page links a `CP210x`
//!   driver without saying the meter has that chip, and a `CP210x` could not keep
//!   its ids on this shell anyway, so this is a `CDC-ACM` function with test ids, an
//!   artifact of the emulation. The tools see a tty either way.
//! - The line, 115200/8n1: the wiki, and the libsigrok driver.
//! - The commands: the wiki's table, the same as the driver's header. `ac ff`
//!   connects, `ca ff` disconnects, `aa f1` to `aa f8` toggle or act, `ad da` asks
//!   for the memory's status, `d3 da XX YY` for block `YYXX`.
//! - The stream: the wiki says measurements come "without any prompting", in
//!   thirteen bytes, `7f`, the measurement in two, flags or mode in two, one
//!   unknown, a date in six, `00`. The driver gives the rest: the value is big
//!   endian tenths of a dB, byte 3 holds C (bit 0) and slow (bit 1), byte 4 the
//!   range (bits 0 and 1), max (bit 2) and min (bit 3).
//! - The memory's status: the wiki's `d1 05 00 01 d2`, the blocks in two bytes, the
//!   length of the last, and "0x10 or 0x20". What the two last values mean, no
//!   source says, and the driver takes `20` alone. So `20` is the default, and
//!   `status=` plays another.
//! - A recording's header: the wiki's, eleven bytes, `7f f7` or `ff ff`, `01 00`,
//!   flags `0a` (A) or `0c` (C), a date. The driver reads the samples behind it as
//!   little endian tenths, and takes `-1` for a new recording from the header
//!   alone.
//!
//! What is an inference, as no source says. The manual was looked for: PCE's link
//! gave a web page and no PDF, and another copy refused the fetch. No firmware
//! source, recording or other program for this meter was found, which is not to
//! say there is none.
//!
//! - That `ca ff` stops the stream and `ac ff` starts it again. The wiki says the
//!   commands aren't needed for it, and the driver disconnects before it asks for
//!   memory, which would make no sense if the two streams could mix.
//! - That `aa f2` steps the range upwards: the wiki's order, "Auto, 30-80, 50-100,
//!   80-130", which the driver's `MEAS_RANGE_*` numbers follow, with Auto as its
//!   30-130.
//! - What the blocks number. The driver reads blocks 0 to `used`, all 256 bytes but
//!   the last, of which it takes `last`, yet logs `used - 1` blocks of 256 and one
//!   of `last`. Those can't both be right, and nothing says which is. This model
//!   holds `256 * used + last` bytes from block 0 on, so that all the driver reads
//!   is stored.
//! - That the last block is cut to `last` bytes: a real one may send all 256. A
//!   model that does cannot give a stable `.tx`, as the driver closes with bytes
//!   still on the way.
//! - That a meter that was powered off answers nothing, and takes nothing until
//!   its next power on, a new session here. The rate of the stream, of which the
//!   driver says nothing and which it never waits for by time.
//!
//! Emulation artifacts:
//!
//! - The stream waits for the host's first command after an open, where the wiki's
//!   meter streams unasked: a scan writes nothing and holds a port for no set time,
//!   so it would catch frames by timing. `unprompted=1` is the wiki's meter.
//! - A measurement goes out after `period=` offers of a read, counted from the last
//!   frame or command, and never while a command is coming in. So a `.tx` does not
//!   depend on timing. `chunk=1` makes offers come fast: use a long `period=` there.
//! - `frames=N` counts per open, and a meter does not end its stream: it is what
//!   holds a capture to a known number of bytes, since the driver writes nothing
//!   before a limit.
//! - `no-status=1` and `no-block=N` lose a reply, as nothing says a real one does.
//!   The driver has no way out of that wait, so they are its cases.
//!
//! The other commands are the wiki's, and no path of the driver sends them, so each
//! is flagged rather than modelled: `aa f3`, `aa f5`, `aa f6`, `aa f7`, `aa c1`
//! (the driver's `pce_322a_memory_clear` is called by nothing) and `7e`.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `spl=TENTHS`, the first reading, default 654 (65.4 dB); it ripples, so that
//!   frames differ. `freq=a|c`, `time=f|s`, `range=0..3`, `mode=current|max|min`,
//!   how the meter is set at power on;
//! - `unprompted=1`, see above;
//! - `period=N`, offers of a read per frame; `frames=N`, frames per open, then
//!   silence and the event point a period later;
//! - `replies=HEX[,HEX...]`, frames played in turn instead of the computed one;
//! - `records=N` and `samples=N`, recordings in memory, A and C in turn, and the
//!   samples in each; `memory=HEX`, the bytes in memory instead;
//! - `used=N` and `last=N`, what the status says instead of what's stored, with
//!   `ff` for what isn't; `status=HEX`, the status's bytes instead;
//! - `no-status=1`, `no-block=N`, the replies lost, then the event point a period
//!   later;
//! - `uart=LINE`.
//!
//! The pedantic checks: a command no driver path sends, a block past the last, and
//! a command left unfinished when a tool opens.
//!
//! What no case reaches in the driver, with `--gcov`: `config_get`'s other answers
//! (`api.c:126-138,144`, `protocol.c:413-426`), since a get comes before any reading
//! in one run of either tool; the checks on NULLs and unknown keys; `dev_open`'s
//! error return (238), as a port that opens at the scan also opens here; a failed
//! write (`protocol.c:39-41,58-64,455`), as a write straight after an unplug may
//! still pass, which `--unplug out=2:1` showed both ways, so no case holds it; a
//! read that gives nothing after a `G_IO_IN`
//! (287,314,323); `meas_range_set`'s else (450), as `config_set` looks a pair up in
//! its table first; `pce_322a_memory_clear`, called by nothing; and "Exhausted
//! memory blocks" (298-299): a block counter that reaches `used` ends the read at a
//! cursor of `last` or less, so it never goes past `used`.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};
use std::fmt::Write as _;

/// The model name.
pub const NAME: &str = "pce-322a";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A PCE PCE-322A sound level meter on CDC-ACM; spl=, freq=, time=, range=, \
                         mode=, period=, frames=, replies=, records=, samples=, memory=, used=, \
                         last=, status=, no-status=, no-block=, unprompted=, uart=, and the shell's stall=, chunk=, \
                         in-limit=, in-stall=, out-hold=";

const FRAME: usize = 13;
const BLOCK: usize = 256;
/// The longest command, a block's number included.
const LONGEST_COMMAND: usize = 4;
/// What fits in memory, here: a bound on an option, not a fact of the meter.
const MEMORY_MAX: usize = 64 * 1024;
/// About half a second, well past any tool's way from an open to its first write,
/// so a `.tx` with a setting does not depend on it.
const PERIOD: u32 = 25;

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
    let mut uart = Line::parse("115200/8n1")?;
    let mut config = Config::default();
    let mut records = 2_usize;
    let mut samples = 8_usize;
    let mut memory = None;
    let mut used = None;
    let mut last = None;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "spl" => config.spl = word(k, v)?,
            "freq" => config.c_weight = choice(k, v, "a", "c")?,
            "time" => config.slow = choice(k, v, "f", "s")?,
            "range" => config.range = parse_number(k, v, 0..=3)? as u8,
            "mode" => {
                config.mode = match v.as_str() {
                    "current" => 0,
                    "max" => 1 << 2,
                    "min" => 1 << 3,
                    _ => return Err(Error::Invalid(format!("{k}={v}"))),
                };
            }
            "unprompted" => config.unprompted = parse_number(k, v, 0..=1)? == 1,
            "period" => config.period = parse_number(k, v, 1..=100_000)?,
            "frames" => config.frames = Some(parse_number(k, v, 0..=100_000)?),
            "replies" => config.replies = Some(Packets::new(parse_hex_list(k, v, 1, BLOCK)?)),
            "records" => records = parse_number(k, v, 0..=100)? as usize,
            "samples" => samples = parse_number(k, v, 0..=32 * 1024)? as usize,
            "memory" => memory = Some(parse_hex_list(k, v, 1, MEMORY_MAX)?.concat()),
            "used" => used = Some(word(k, v)?),
            "last" => last = Some(parse_number(k, v, 0..=0xff)? as u8),
            "status" => config.status = Some(parse_hex_list(k, v, 1, BLOCK)?.concat()),
            "no-status" => config.no_status = parse_number(k, v, 0..=1)? == 1,
            "no-block" => config.no_block = Some(word(k, v)?),
            "uart" => uart = Line::parse(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let memory = if let Some(bytes) = memory {
        bytes
    } else {
        if records.saturating_mul(samples.saturating_mul(2) + HEADER_LEN) > MEMORY_MAX {
            return Err(Error::Invalid(format!("{NAME}: over {MEMORY_MAX} bytes of memory")));
        }
        recordings(records, samples, config.spl)
    };
    config.used = used.unwrap_or_else(|| u16::try_from(memory.len() / BLOCK).expect("bounded"));
    config.last = last.unwrap_or_else(|| u8::try_from(memory.len() % BLOCK).expect("below 256"));
    config.memory = memory;
    // Test ids: see the module's note on the USB side.
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "PCE-322A stand-in");
    Ok(Cdc::new(NAME, &id, faults, Meter::new(config)).behind_uart(uart))
}

/// A 16-bit value of an option.
fn word(key: &str, value: &str) -> Result<u16> {
    Ok(parse_number(key, value, 0..=0xffff)? as u16)
}

fn choice(key: &str, value: &str, off: &str, on: &str) -> Result<bool> {
    if value == off {
        Ok(false)
    } else if value == on {
        Ok(true)
    } else {
        Err(Error::Invalid(format!("{key}={value}")))
    }
}

const HEADER_LEN: usize = 11;

/// The recordings in memory, each with its header. The date is in BCD, as the
/// driver prints it in hex, and it differs per recording.
fn recordings(records: usize, samples: usize, spl: u16) -> Vec<u8> {
    let mut bytes = Vec::new();
    for r in 0..records {
        let c_weight = r % 2 == 1;
        let minute = u8::try_from(r % 10).expect("a digit");
        bytes.extend_from_slice(&[0x7f, 0xf7, 0x01, 0x00, if c_weight { 0x0c } else { 0x0a }]);
        bytes.extend_from_slice(&[0x26, 0x11, 0x05, 0x12, minute, 0x30]);
        for s in 0..samples {
            let value = spl.wrapping_add(u16::try_from(s % 256).expect("below 256"));
            bytes.extend_from_slice(&value.to_le_bytes());
        }
    }
    bytes
}

fn show(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)]
struct Config {
    spl: u16,
    c_weight: bool,
    slow: bool,
    range: u8,
    /// Byte 4's max and min bits.
    mode: u8,
    unprompted: bool,
    period: u32,
    frames: Option<u32>,
    replies: Option<Packets>,
    memory: Vec<u8>,
    used: u16,
    last: u8,
    status: Option<Vec<u8>>,
    no_status: bool,
    no_block: Option<u16>,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            spl: 654,
            c_weight: false,
            slow: false,
            range: 0,
            mode: 0,
            unprompted: false,
            period: PERIOD,
            frames: None,
            replies: None,
            memory: Vec::new(),
            used: 0,
            last: 0,
            status: None,
            no_status: false,
            no_block: None,
        }
    }
}

#[allow(clippy::struct_excessive_bools)]
struct Meter {
    config: Config,
    replies: Option<Packets>,
    /// What has come of a command so far, shorter than one.
    pending: Vec<u8>,
    connected: bool,
    /// The host has written since the open, or need not.
    heard: bool,
    off: bool,
    c_weight: bool,
    slow: bool,
    range: u8,
    /// Frames sent since the open.
    sent: u32,
    /// Offers since the last frame or command.
    offers: u32,
    /// A reply was lost on purpose, so the event point is due.
    lost_reply: bool,
    marked: bool,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter {
            replies: config.replies.clone(),
            pending: Vec::new(),
            connected: true,
            heard: config.unprompted,
            off: false,
            c_weight: config.c_weight,
            slow: config.slow,
            range: config.range,
            sent: 0,
            offers: 0,
            lost_reply: false,
            marked: false,
            config,
        }
    }

    fn measurement(&self) -> Vec<u8> {
        let spl = self.config.spl.wrapping_add(u16::try_from(self.sent % 8).expect("below 8"));
        let mut f = vec![0; FRAME];
        f[0] = 0x7f;
        f[1..3].copy_from_slice(&spl.to_be_bytes());
        f[3] = u8::from(self.c_weight) | u8::from(self.slow) << 1;
        f[4] = self.range | self.config.mode;
        f[6..12].copy_from_slice(&[0x26, 0x11, 0x05, 0x12, 0x30, 0x00]);
        f
    }

    fn status(&self) -> Vec<u8> {
        if let Some(status) = &self.config.status {
            return status.clone();
        }
        let [high, low] = self.config.used.to_be_bytes();
        vec![0xd1, 0x05, 0x00, 0x01, 0xd2, high, low, self.config.last, 0x20]
    }

    /// Block `n`, with `ff` for what isn't stored.
    fn block(&self, n: u16) -> Vec<u8> {
        let length = if n == self.config.used { usize::from(self.config.last) } else { BLOCK };
        let from = usize::from(n) * BLOCK;
        let mut block: Vec<u8> =
            self.config.memory.iter().skip(from).take(length).copied().collect();
        block.resize(length, 0xff);
        block
    }

    fn command(&mut self, command: &[u8], port: &mut Port<'_>) {
        self.offers = 0;
        self.heard = true;
        if self.off {
            return;
        }
        match *command {
            [0xac, 0xff] => self.connected = true,
            [0xca, 0xff] => self.connected = false,
            [0xaa, 0xf1] => self.c_weight = !self.c_weight,
            [0xaa, 0xf2] => self.range = (self.range + 1) & 3,
            [0xaa, 0xf4] => self.slow = !self.slow,
            [0xaa, 0xf8] => self.off = true,
            [0xad, 0xda] => {
                if self.config.no_status {
                    self.lost_reply = true;
                } else {
                    port.send(&self.status());
                }
            }
            [0xd3, 0xda, low, high] => {
                let n = u16::from_le_bytes([low, high]);
                if n > self.config.used {
                    port.violation(format!("{}: past the last block", show(command)));
                } else if self.config.no_block == Some(n) {
                    self.lost_reply = true;
                } else {
                    port.send(&self.block(n));
                }
            }
            _ => port.violation(format!("{}: no driver path sends this", show(command))),
        }
    }

    fn command_length(first: u8) -> usize {
        if first == 0xd3 { LONGEST_COMMAND } else { 2 }
    }

    fn mark_once(&mut self, port: &mut Port<'_>) {
        if !self.marked {
            self.marked = true;
            port.mark();
        }
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for byte in data {
            self.pending.push(*byte);
            if self.pending.len() >= Self::command_length(self.pending[0]) {
                let command = std::mem::take(&mut self.pending);
                self.command(&command, port);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() {
            if !self.pending.is_empty() {
                port.violation(format!("{}: a command left unfinished", show(&self.pending)));
                self.pending.clear();
            }
            self.sent = 0;
            self.offers = 0;
            self.heard = self.config.unprompted;
            self.replies.clone_from(&self.config.replies);
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if self.off || !self.heard || !self.pending.is_empty() {
            return;
        }
        self.offers = self.offers.saturating_add(1);
        if self.offers < self.config.period {
            return;
        }
        if self.lost_reply {
            self.mark_once(port);
        }
        if !self.connected {
            return;
        }
        if self.config.frames.is_some_and(|n| self.sent >= n) {
            self.mark_once(port);
            return;
        }
        let frame = match &mut self.replies {
            Some(list) => list.take(),
            None => self.measurement(),
        };
        port.send(&frame);
        self.sent += 1;
        self.offers = 0;
    }

    fn reset(&mut self) {
        *self = Meter::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("115200/8n1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Meter>) -> usize {
        h.v.drain().len()
    }

    /// Offers until something comes, with a bound.
    fn next(h: &mut Host<Meter>) -> Vec<u8> {
        for _ in 0..1000 {
            let got = h.read();
            if !got.is_empty() {
                return got;
            }
        }
        Vec::new()
    }

    fn quiet(h: &mut Host<Meter>) -> bool {
        (0..200).all(|_| h.read().is_empty())
    }

    #[test]
    fn a_measurement_has_the_wikis_shape() {
        let mut h = meter(&[("period", "2"), ("unprompted", "1")]);
        let f = next(&mut h);
        assert_eq!(show(&f), "7f028e00000026110512300000");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn it_streams_unasked_and_the_readings_ripple() {
        let mut h = meter(&[("period", "2"), ("unprompted", "1")]);
        let a = next(&mut h);
        let b = next(&mut h);
        assert_eq!(a[1..3], [0x02, 0x8e]);
        assert_eq!(b[1..3], [0x02, 0x8f]);
    }

    #[test]
    fn the_stream_waits_for_the_hosts_first_word_after_each_open() {
        let mut h = meter(&[("period", "2")]);
        assert!(quiet(&mut h));
        h.write(&[0xac, 0xff]);
        assert_eq!(next(&mut h).len(), FRAME);
        h.set_lines(true, true);
        assert!(quiet(&mut h));
    }

    #[test]
    fn disconnect_stops_the_stream_and_connect_starts_it() {
        let mut h = meter(&[("period", "2")]);
        h.write(&[0xca, 0xff]);
        assert!(quiet(&mut h));
        h.write(&[0xac, 0xff]);
        assert_eq!(next(&mut h).len(), FRAME);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_settings_show_in_the_flags() {
        let mut h = meter(&[("period", "2")]);
        h.write(&[0xaa, 0xf1]);
        let f = next(&mut h);
        assert_eq!((f[3], f[4]), (0x01, 0));
        h.write(&[0xaa, 0xf4]);
        let f = next(&mut h);
        assert_eq!((f[3], f[4]), (0x03, 0));
        h.write(&[0xaa, 0xf1]);
        let f = next(&mut h);
        assert_eq!(f[3], 0x02);
    }

    #[test]
    fn the_range_steps_up_and_wraps() {
        let mut h = meter(&[("period", "2"), ("range", "2"), ("unprompted", "1")]);
        assert_eq!(next(&mut h)[4], 2);
        h.write(&[0xaa, 0xf2]);
        assert_eq!(next(&mut h)[4], 3);
        h.write(&[0xaa, 0xf2]);
        assert_eq!(next(&mut h)[4], 0);
    }

    #[test]
    fn the_options_set_how_it_comes_up() {
        let mut h = meter(&[
            ("period", "2"),
            ("freq", "c"),
            ("time", "s"),
            ("range", "1"),
            ("mode", "max"),
            ("spl", "1000"),
            ("unprompted", "1"),
        ]);
        let f = next(&mut h);
        assert_eq!(f[1..3], [0x03, 0xe8]);
        assert_eq!((f[3], f[4]), (0x03, 0x05));
        let mut h = meter(&[("period", "2"), ("mode", "min"), ("unprompted", "1")]);
        assert_eq!(next(&mut h)[4], 0x08);
    }

    #[test]
    fn power_off_ends_the_stream_and_the_answers() {
        let mut h = meter(&[("period", "2"), ("unprompted", "1")]);
        h.write(&[0xaa, 0xf8]);
        assert!(quiet(&mut h));
        h.write(&[0xad, 0xda]);
        h.write(&[0xac, 0xff]);
        assert!(quiet(&mut h));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_status_says_what_is_stored() {
        // 2 * (11 + 16) bytes.
        let mut h = meter(&[("frames", "0")]);
        h.write(&[0xad, 0xda]);
        assert_eq!(show(&next(&mut h)), "d1050001d200003620");
        // 300 bytes.
        let mut h = meter(&[("frames", "0"), ("records", "1"), ("samples", "145")]);
        h.write(&[0xad, 0xda]);
        assert_eq!(show(&next(&mut h)), "d1050001d200012d20");
    }

    #[test]
    fn a_block_is_what_is_stored_and_the_last_is_cut() {
        let bytes: Vec<u8> = (0..=255).chain([0xaa, 0xbb, 0xcc]).collect();
        let memory = show(&bytes);
        let mut h = meter(&[("frames", "0"), ("memory", &memory)]);
        h.write(&[0xad, 0xda]);
        assert_eq!(show(&next(&mut h)), "d1050001d200010320");
        h.write(&[0xd3, 0xda, 0, 0]);
        let mut block = Vec::new();
        while block.len() < BLOCK {
            let got = next(&mut h);
            assert_ne!(got.len(), 0);
            block.extend_from_slice(&got);
        }
        assert_eq!(block[255], 255);
        assert_eq!(block.len(), BLOCK);
        h.write(&[0xd3, 0xda, 1, 0]);
        assert_eq!(show(&next(&mut h)), "aabbcc");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_blocks_number_is_little_endian_and_the_rest_is_ff() {
        let mut h = meter(&[("frames", "0"), ("memory", "aa"), ("used", "258"), ("last", "2")]);
        // Block 0x0102: the last.
        h.write(&[0xd3, 0xda, 0x02, 0x01]);
        assert_eq!(show(&next(&mut h)), "ffff");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn an_empty_last_block_sends_nothing() {
        let mut h = meter(&[("frames", "0"), ("records", "0")]);
        h.write(&[0xad, 0xda]);
        assert_eq!(show(&next(&mut h)), "d1050001d200000020");
        h.write(&[0xd3, 0xda, 0, 0]);
        assert!(quiet(&mut h));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_block_past_the_last_is_flagged() {
        let mut h = meter(&[("frames", "0")]);
        h.write(&[0xd3, 0xda, 1, 0]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn commands_no_driver_path_sends_are_flagged() {
        for command in [
            [0xaa, 0xf3],
            [0xaa, 0xf5],
            [0xaa, 0xf6],
            [0xaa, 0xf7],
            [0xaa, 0xc1],
            [0x7e, 0x00],
            [0x55, 0x55],
        ] {
            let mut h = meter(&[("frames", "0")]);
            h.write(&command);
            assert_eq!(violations(&mut h), 1, "{command:02x?}");
        }
    }

    #[test]
    fn a_command_may_come_in_pieces_and_holds_the_stream() {
        let mut h = meter(&[("period", "2")]);
        h.write(&[0xaa]);
        assert!(quiet(&mut h));
        h.write(&[0xf1]);
        assert_eq!(next(&mut h)[3], 0x01);
        let mut h = meter(&[("frames", "0")]);
        h.write(&[0xd3, 0xda]);
        h.write(&[0x00]);
        assert!(quiet(&mut h));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn an_unfinished_command_at_an_open_is_flagged() {
        let mut h = meter(&[("frames", "0")]);
        h.write(&[0xaa]);
        h.set_lines(false, false);
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn frames_count_per_open_and_replies_play_in_turn() {
        let mut h = meter(&[
            ("period", "2"),
            ("frames", "2"),
            ("replies", "aa,bbcc"),
            ("unprompted", "1"),
        ]);
        assert_eq!(next(&mut h), [0xaa]);
        assert_eq!(next(&mut h), [0xbb, 0xcc]);
        assert!(quiet(&mut h));
        h.set_lines(true, true);
        assert_eq!(next(&mut h), [0xaa]);
    }

    #[test]
    fn lost_replies_stay_lost() {
        let mut h = meter(&[("frames", "0"), ("no-status", "1")]);
        h.write(&[0xad, 0xda]);
        assert!(quiet(&mut h));
        let mut h = meter(&[("frames", "0"), ("no-block", "1"), ("used", "2")]);
        h.write(&[0xd3, 0xda, 1, 0]);
        assert!(quiet(&mut h));
        h.write(&[0xd3, 0xda, 2, 0]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_recording_has_the_wikis_header_and_little_endian_samples() {
        let bytes = recordings(2, 2, 0x0102);
        assert_eq!(show(&bytes[..15]), "7ff701000a26110512003002010301");
        assert_eq!(bytes[15 + 4], 0x0c);
    }

    #[test]
    fn nothing_passes_on_another_line() {
        let o: Vec<(String, String)> = vec![("frames".to_owned(), "0".to_owned())];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true);
        h.write(&[0xad, 0xda]);
        assert!(quiet(&mut h));
    }

    #[test]
    fn options_out_of_range_are_refused() {
        for (k, v) in [
            ("range", "4"),
            ("freq", "z"),
            ("time", "z"),
            ("mode", "z"),
            ("period", "0"),
            ("nonsense", "1"),
            ("last", "256"),
        ] {
            assert!(function(&[(k.to_owned(), v.to_owned())]).is_err(), "{k}={v}");
        }
        let too_much = [
            ("records".to_owned(), "100".to_owned()),
            ("samples".to_owned(), "32768".to_owned()),
        ];
        assert!(function(&too_much).is_err());
    }
}
