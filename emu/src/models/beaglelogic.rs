// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A `BeagleLogic` behind its TCP server, on a socket: the `tcp-server-go` command
//! set over the kernel module's settings. Nothing on it is SCPI, and it has no USB.
//!
//! The device, and how sure this is. All of it is read from the `BeagleLogic`
//! repository (github.com/abhishek-kakkar/BeagleLogic, master, f9dab91a), never run:
//!
//! - **The server,** `tcp-server-go/server.go`. A line per command, cut at `\n` and
//!   trimmed. `samplerate`, `sampleunit`, `triggerflags`, `bufunitsize` and `memalloc`
//!   alone ask, with one argument set: `OK\r\n` or `ERR\r\n`. `version` says
//!   `BeagleLogic 1.0\r\n` (`BeagleLogic Standalone 1.0` on that board). `get` starts
//!   a stream on this same connection, and `close` ends it, with no answer to either.
//!   What it doesn't know gets nothing. It serves one connection at a time: `Accept`
//!   is followed by `connHandler` in the same goroutine, so a second one waits unread
//!   until the first has gone.
//! - **The stream.** `beaglelogicRun` reads `/dev/beaglelogic` and writes what it got.
//!   The module (`kernel/beaglelogic.c`) hands out a buffer only once it is full, so
//!   nothing comes for `bufunitsize` bytes' time at the rate. A one-shot capture ends
//!   after `memalloc` bytes with a read of 0, and nothing tells the client: no end
//!   mark and no close. A module with no buffers fails the open, and the goroutine
//!   leaves in silence (`f_open`'s `-ENOMEM`; `probe` allocates none, so one that is
//!   never given a `memalloc` is like that).
//! - **Settings.** `kernel/beaglelogic.c`: 100 MHz, 8 bits (`sampleunit` 1), one-shot
//!   at probe. `samplerate` takes 1 to 100 MHz and keeps `100M / (100M / rate)`, so
//!   60 MHz becomes 100 MHz; what's outside is `-EINVAL`, so `ERR`. While a capture
//!   runs the module's mutex is held and a set is `-EBUSY`, so `ERR`. Sysfs prints
//!   `sampleunit` as `8bit`, which the server makes `1` (`0` for anything else), and
//!   `triggerflags` as `0:oneshot`, of which it keeps the digit.
//! - **`close`.** The server's flag is read after the next write, so one read's worth
//!   may still come after it (`tail=`). A client that goes without a `close` leaves
//!   the goroutine reading, which the model doesn't copy: the stream ends with the
//!   connection. `tcp-server-node/server.js` differs where no C path looks, except
//!   that a `close` with no `get` kills it: flagged here.
//!
//! Emulation artifacts:
//!
//! - **Size.** A real module has `bufunitsize` 4 MiB and tens of MiB of `memalloc`.
//!   Here it is 4 KiB and 64 KiB unless the options say, to keep streams small. So the
//!   default stream goes at once, in one write at `get`, whatever the rate:
//!   `paced=1` gives a buffer at a time at the rate's real pace.
//! - **Samples.** A counter from 0 at each `get`, little endian at 16 bits: bit `i`
//!   rises first at sample `2^i`. Nothing is on the real pins, of course.
//! - **A continuous stream ends.** After `total=` bytes (four `memalloc`s unless
//!   said), or never with `free=1`, which a `.tx` can hold only with its length
//!   filtered.
//!
//! Options:
//!
//! - `samplerate=`, `sampleunit=`, `triggerflags=`, `bufunitsize=`, `memalloc=`, what
//!   the module holds at power on; `memalloc=0` means no buffers, and silence at `get`;
//! - `version=TEXT`, with `_` for a space;
//! - `say=CMD:TEXT`, the answer to a query of `CMD` from then on (`_` for a space,
//!   and `CMD` may be `version`); `mute=CMD` no answer to it, `late=CMD:T` an answer
//!   after `T` ticks (20 ms each). With a `+` after `CMD`, its set form instead;
//! - `refuse=CMD`, `ERR` to a set of it; `hangup=CMD`, the connection closes instead
//!   of an answer (`CMD+` for a set);
//! - `total=N`, the bytes of a continuous stream; `free=1`, no end to it; `chunk=N`,
//!   that many bytes a tick instead of all at once; `paced=1`, a buffer at a time at
//!   the rate; `first=T`, a wait in ticks before the first byte;
//! - `tail=N`, that many bytes after a `close`;
//! - `mark=N`, the event point (`tcp-model`) once `N` bytes have gone, 0 at `get`.
//!
//! The pedantic checks: a second connection while one is up, as the server wouldn't
//! read it; a command no C path sends (`memalloc N`, `bufunitsize N`, `exit`, `state`,
//! asking for `triggerflags`) or nobody knows; a set's value that isn't a number or,
//! for `sampleunit` and `triggerflags`, isn't 0 or 1; a `get` on a stream; a `close`
//! with none; a line with no end in 256 bytes.

use crate::model::DeviceModel;
use crate::shells::tcp::{NoUsb, Port, Stream};
use crate::txlog::{HASH_ABOVE, Rules};
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "beaglelogic";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A BeagleLogic behind its TCP server, on a socket only; samplerate=, \
                         sampleunit=, triggerflags=, bufunitsize=, memalloc=, version=, say=, \
                         mute=, late=, refuse=, hangup=, total=, free=, chunk=, paced=, first=, tail=, \
                         mark=";

/// `coreclockfreq / 2`, in the module: the rate a divisor of 1 gives.
const TOP_RATE: u32 = 100_000_000;
/// The most of a line kept.
const LINE: usize = 256;
/// A tick of the carrier, in ms.
const TICK_MS: u64 = 20;
/// Bytes a paced or chunked stream sends at most in a tick, whatever it owes: a host
/// that doesn't read must not grow the carrier's queue for long.
const MOST_A_TICK: u64 = 1 << 20;

/// What the options say.
#[derive(Clone, Debug)]
struct Config {
    samplerate: u32,
    sampleunit: u32,
    triggerflags: u32,
    bufunitsize: u32,
    memalloc: u32,
    version: String,
    say: Vec<(String, String)>,
    mute: Vec<String>,
    late: Vec<(String, u32)>,
    refuse: Vec<String>,
    hangup: Vec<String>,
    total: Option<u64>,
    free: bool,
    chunk: Option<u64>,
    paced: bool,
    first: u32,
    tail: u64,
    mark: Option<u64>,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            samplerate: TOP_RATE,
            sampleunit: 1,
            triggerflags: 0,
            bufunitsize: 4096,
            memalloc: 65536,
            version: "BeagleLogic 1.0".to_owned(),
            say: Vec::new(),
            mute: Vec::new(),
            late: Vec::new(),
            refuse: Vec::new(),
            hangup: Vec::new(),
            total: None,
            free: false,
            chunk: None,
            paced: false,
            first: 0,
            tail: 0,
            mark: None,
        }
    }
}

fn number<T: std::str::FromStr>(key: &str, value: &str) -> Result<T> {
    value.parse().map_err(|_| Error::Invalid(format!("{key}={value}: not a number")))
}

fn spaced(text: &str) -> String {
    text.replace('_', " ")
}

fn parse(options: &[(String, String)]) -> Result<Config> {
    let mut c = Config::default();
    for (k, v) in options {
        match k.as_str() {
            "samplerate" => c.samplerate = number(k, v)?,
            "sampleunit" => c.sampleunit = number(k, v)?,
            "triggerflags" => c.triggerflags = number(k, v)?,
            "bufunitsize" => c.bufunitsize = number(k, v)?,
            "memalloc" => c.memalloc = number(k, v)?,
            "version" => c.version = spaced(v),
            "say" => {
                let (cmd, text) = v
                    .split_once(':')
                    .ok_or_else(|| Error::Invalid(format!("say={v}: say=CMD:TEXT")))?;
                c.say.push((cmd.to_owned(), spaced(text)));
            }
            "mute" => c.mute.push(v.clone()),
            "late" => {
                let (cmd, t) = v
                    .split_once(':')
                    .ok_or_else(|| Error::Invalid(format!("late={v}: late=CMD:T")))?;
                c.late.push((cmd.to_owned(), number(k, t)?));
            }
            "refuse" => c.refuse.push(v.clone()),
            "hangup" => c.hangup.push(v.clone()),
            "total" => c.total = Some(number(k, v)?),
            "free" => c.free = v == "1",
            "chunk" => c.chunk = Some(number::<u64>(k, v)?.clamp(1, MOST_A_TICK)),
            "paced" => c.paced = v == "1",
            "first" => c.first = number(k, v)?,
            "tail" => c.tail = number(k, v)?,
            "mark" => c.mark = Some(number(k, v)?),
            _ => return Err(Error::Invalid(format!("{NAME} has no option {k}"))),
        }
    }
    if c.bufunitsize == 0 {
        return Err(Error::Invalid("bufunitsize=0: the module refuses under 32".to_owned()));
    }
    Ok(c)
}

/// A stream under way.
#[derive(Clone, Debug)]
struct Run {
    /// Samples made so far, for the counter.
    sample: u64,
    /// Bytes sent so far.
    sent: u64,
    /// Bytes it sends at all, `None` for no end.
    total: Option<u64>,
    /// Bytes per go, and the ticks from one go to the next.
    chunk: u64,
    period: u32,
    wait: u32,
    /// Bytes a sample, whatever `sampleunit` became since.
    width: u64,
    marked: bool,
}

/// The server and the module behind it.
#[derive(Debug)]
pub struct BeagleLogic {
    config: Config,
    samplerate: u32,
    sampleunit: u32,
    triggerflags: u32,
    bufunitsize: u32,
    memalloc: u32,
    line: Vec<u8>,
    connections: u32,
    run: Option<Run>,
    /// Answers held back, with the ticks left. What comes behind a late one waits.
    later: VecDeque<(u32, Vec<u8>)>,
}

impl BeagleLogic {
    fn new(config: Config) -> BeagleLogic {
        BeagleLogic {
            samplerate: config.samplerate,
            sampleunit: config.sampleunit,
            triggerflags: config.triggerflags,
            bufunitsize: config.bufunitsize,
            memalloc: config.memalloc,
            config,
            line: Vec::new(),
            connections: 0,
            run: None,
            later: VecDeque::new(),
        }
    }

    /// An answer: at once, or behind what is held, or after `late` ticks.
    fn answer(&mut self, port: &mut Port, key: &str, text: &str) {
        if self.config.hangup.iter().any(|h| h == key) {
            port.hang_up();
            return;
        }
        if self.config.mute.iter().any(|m| m == key) {
            return;
        }
        let mut bytes = text.as_bytes().to_vec();
        bytes.extend_from_slice(b"\r\n");
        let late = self.config.late.iter().find(|(k, _)| k == key).map(|(_, t)| *t);
        if late.is_none() && self.later.is_empty() {
            port.send(&bytes);
        } else {
            self.later.push_back((late.unwrap_or(0), bytes));
        }
    }

    /// The module's rule for a rate.
    fn set_rate(&mut self, text: &str, port: &mut Port) -> bool {
        let Ok(rate) = text.parse::<u32>() else {
            port.violation(format!("samplerate {text}: not a number"));
            return false;
        };
        if !(1..=TOP_RATE).contains(&rate) {
            return false;
        }
        self.samplerate = TOP_RATE / (TOP_RATE / rate);
        true
    }

    fn set_flag(field: &mut u32, name: &str, text: &str, port: &mut Port) -> bool {
        match text.parse::<u32>() {
            Ok(n) if n <= 1 => {
                *field = n;
                true
            }
            _ => {
                port.violation(format!("{name} {text}: not 0 or 1"));
                false
            }
        }
    }

    fn query(&mut self, cmd: &str, port: &mut Port) {
        let told = self.config.say.iter().find(|(k, _)| k == cmd).map(|(_, t)| t.clone());
        let own = match cmd {
            "version" => self.config.version.clone(),
            "samplerate" => self.samplerate.to_string(),
            "sampleunit" => self.sampleunit.to_string(),
            "bufunitsize" => self.bufunitsize.to_string(),
            "memalloc" => self.memalloc.to_string(),
            _ => String::new(),
        };
        self.answer(port, cmd, &told.unwrap_or(own));
    }

    fn set(&mut self, cmd: &str, value: &str, port: &mut Port) {
        let key = format!("{cmd}+");
        let done = if self.run.is_some() {
            // The module's mutex is held for all of a capture.
            false
        } else if self.config.refuse.iter().any(|r| r == cmd) {
            false
        } else {
            match cmd {
                "samplerate" => self.set_rate(value, port),
                "sampleunit" => Self::set_flag(&mut self.sampleunit, cmd, value, port),
                "triggerflags" => Self::set_flag(&mut self.triggerflags, cmd, value, port),
                _ => false,
            }
        };
        self.answer(port, &key, if done { "OK" } else { "ERR" });
    }

    fn begin(&mut self, port: &mut Port) {
        if self.run.is_some() {
            port.violation("get on a stream");
            return;
        }
        // The file's open fails, and the goroutine leaves in silence.
        if self.memalloc == 0 {
            if self.config.mark == Some(0) {
                port.mark();
            }
            return;
        }
        let memalloc = u64::from(self.memalloc);
        let bufunit = u64::from(self.bufunitsize);
        let width = if self.sampleunit == 1 { 1 } else { 2 };
        let total = if self.triggerflags == 0 {
            Some(self.config.total.unwrap_or(memalloc).min(memalloc))
        } else if self.config.free {
            None
        } else {
            Some(self.config.total.unwrap_or(4 * memalloc))
        };
        let (chunk, period) = if self.config.paced {
            let bytes_per_s = u64::from(self.samplerate) * width;
            let ms = bufunit * 1000 / bytes_per_s.max(1);
            let period = (ms / TICK_MS).clamp(1, u64::from(u32::MAX)) as u32;
            (bufunit.min(MOST_A_TICK), period)
        } else if let Some(chunk) = self.config.chunk {
            (chunk, 1)
        } else if total.is_none() {
            (bufunit, 1)
        } else {
            (u64::MAX, 0)
        };
        let wait = if self.config.paced { self.config.first.max(period) } else { self.config.first };
        self.run = Some(Run {
            sample: 0,
            sent: 0,
            total,
            chunk,
            period,
            wait,
            width,
            marked: false,
        });
        if self.config.mark == Some(0) {
            port.mark();
            if let Some(run) = &mut self.run {
                run.marked = true;
            }
        }
        if wait == 0 {
            self.go(port);
        }
    }

    /// One go of the stream.
    fn go(&mut self, port: &mut Port) {
        let mark = self.config.mark;
        let Some(run) = &mut self.run else { return };
        let left = run.total.map_or(u64::MAX, |t| t.saturating_sub(run.sent));
        let bytes = run.chunk.min(left);
        if bytes == 0 {
            return;
        }
        let n = usize::try_from(bytes).unwrap_or(usize::MAX);
        let mut out = Vec::with_capacity(n);
        let first_byte = run.sent;
        for at in first_byte..first_byte + bytes {
            let sample = at / run.width;
            let byte = (sample >> (8 * (at % run.width))) & 0xff;
            out.push(byte as u8);
        }
        run.sent += bytes;
        run.sample = run.sent / run.width;
        run.wait = run.period;
        let hit = mark.is_some_and(|m| !run.marked && run.sent >= m);
        if hit {
            run.marked = true;
        }
        port.send(&out);
        if hit {
            port.mark();
        }
    }

    fn end(&mut self, port: &mut Port) {
        if self.run.take().is_none() {
            port.violation("close with no stream");
        }
        let tail = usize::try_from(self.config.tail).unwrap_or(0);
        if tail > 0 {
            port.send(&vec![0xa5; tail]);
        }
    }

    fn command(&mut self, line: &[u8], port: &mut Port) {
        let text = String::from_utf8_lossy(line);
        let words: Vec<&str> = text.trim().split(' ').collect();
        match words.as_slice() {
            ["version" | "samplerate" | "sampleunit" | "bufunitsize" | "memalloc"] => {
                self.query(words[0], port);
            }
            ["samplerate" | "sampleunit" | "triggerflags", value] => {
                let (cmd, value) = (words[0].to_owned(), (*value).to_owned());
                self.set(&cmd, &value, port);
            }
            ["get"] => self.begin(port),
            ["close"] => self.end(port),
            _ => port.violation(format!("no C path sends {text:?}")),
        }
    }
}

impl Stream for BeagleLogic {
    fn received(&mut self, data: &[u8], port: &mut Port) {
        for &b in data {
            if b == b'\n' {
                let line = std::mem::take(&mut self.line);
                self.command(&line, port);
            } else if self.line.len() >= LINE {
                port.violation("a line with no end");
                self.line.clear();
            } else {
                self.line.push(b);
            }
        }
    }

    fn connected(&mut self, port: &mut Port) {
        self.connections += 1;
        if self.connections > 1 {
            port.violation("a second connection, which the server would not read");
        }
    }

    fn closed(&mut self) {
        self.connections = self.connections.saturating_sub(1);
        self.line.clear();
        self.later.clear();
        // Unlike the server's goroutine: see the module's comment.
        self.run = None;
    }

    fn tick(&mut self, port: &mut Port) {
        while let Some((ticks, _)) = self.later.front_mut() {
            if *ticks > 0 {
                *ticks -= 1;
                break;
            }
            if let Some((_, bytes)) = self.later.pop_front() {
                port.send(&bytes);
            }
        }
        let due = match &mut self.run {
            Some(run) if run.chunk != u64::MAX || run.sent == 0 => {
                run.wait = run.wait.saturating_sub(1);
                run.wait == 0
            }
            _ => false,
        };
        if due {
            self.go(port);
        }
    }
}

/// The server on a socket.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    Ok(Box::new(BeagleLogic::new(parse(options)?)))
}

/// No USB: `EMU_TCP=only`.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    parse(options)?;
    Ok(Box::new(NoUsb::new(NAME)))
}

/// A stream is long: its digest in the log.
pub fn log_rules(_options: &[(String, String)]) -> Rules {
    Rules::digest_above(HASH_ABOVE)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model(options: &[(&str, &str)]) -> BeagleLogic {
        let o: Vec<_> = options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        BeagleLogic::new(parse(&o).unwrap())
    }

    fn talk(m: &mut BeagleLogic, text: &str) -> Port {
        let mut p = Port::default();
        m.received(text.as_bytes(), &mut p);
        p
    }

    fn said(p: &Port) -> Vec<u8> {
        p.sent().concat()
    }

    fn flagged(p: &mut Port) -> usize {
        p.violations().drain().len()
    }

    #[test]
    fn it_answers_as_the_server_does() {
        let mut m = model(&[]);
        assert_eq!(said(&talk(&mut m, "version\n")), b"BeagleLogic 1.0\r\n");
        assert_eq!(said(&talk(&mut m, "samplerate\n")), b"100000000\r\n");
        assert_eq!(said(&talk(&mut m, "sampleunit\n")), b"1\r\n");
        assert_eq!(said(&talk(&mut m, "bufunitsize\n")), b"4096\r\n");
        assert_eq!(said(&talk(&mut m, "memalloc\n")), b"65536\r\n");
        assert_eq!(said(&talk(&mut m, "triggerflags 1\n")), b"OK\r\n");
        assert_eq!(said(&talk(&mut m, "sampleunit 0\n")), b"OK\r\n");
        assert_eq!(said(&talk(&mut m, "sampleunit\n")), b"0\r\n");
    }

    #[test]
    fn a_command_may_come_in_pieces_and_two_in_one() {
        let mut m = model(&[]);
        assert_eq!(said(&talk(&mut m, "vers")), b"");
        assert_eq!(said(&talk(&mut m, "ion\nsampleunit\n")), b"BeagleLogic 1.0\r\n1\r\n");
    }

    #[test]
    fn a_rate_goes_to_the_nearest_divisor_from_the_top_and_out_of_range_is_err() {
        let mut m = model(&[]);
        assert_eq!(said(&talk(&mut m, "samplerate 60000000\n")), b"OK\r\n");
        assert_eq!(said(&talk(&mut m, "samplerate\n")), b"100000000\r\n");
        assert_eq!(said(&talk(&mut m, "samplerate 30000000\n")), b"OK\r\n");
        assert_eq!(said(&talk(&mut m, "samplerate\n")), b"33333333\r\n");
        assert_eq!(said(&talk(&mut m, "samplerate 0\n")), b"ERR\r\n");
        assert_eq!(said(&talk(&mut m, "samplerate 100000001\n")), b"ERR\r\n");
        assert_eq!(said(&talk(&mut m, "samplerate\n")), b"33333333\r\n");
    }

    #[test]
    fn a_one_shot_stream_is_its_memory_and_nothing_says_it_ended() {
        let mut m = model(&[("memalloc", "16"), ("sampleunit", "0")]);
        let p = talk(&mut m, "get\n");
        // A counter, little endian.
        assert_eq!(said(&p), [0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0]);
        let mut p = Port::default();
        for _ in 0..100 {
            m.tick(&mut p);
        }
        assert_eq!(said(&p), b"", "no more, and no close");
    }

    #[test]
    fn eight_bits_are_one_byte_each_and_a_second_get_starts_over() {
        let mut m = model(&[("memalloc", "4")]);
        assert_eq!(said(&talk(&mut m, "get\n")), [0, 1, 2, 3]);
        assert_eq!(said(&talk(&mut m, "close\n")), b"");
        assert_eq!(said(&talk(&mut m, "get\n")), [0, 1, 2, 3]);
    }

    #[test]
    fn a_continuous_stream_ends_only_when_told_or_with_free_when_closed() {
        let mut m = model(&[("memalloc", "4"), ("triggerflags", "1")]);
        assert_eq!(said(&talk(&mut m, "get\n")).len(), 16);
        let mut m = model(&[("memalloc", "4"), ("triggerflags", "1"), ("total", "6")]);
        assert_eq!(said(&talk(&mut m, "get\n")).len(), 6);
        let mut m = model(&[("bufunitsize", "32"), ("triggerflags", "1"), ("free", "1")]);
        let p = talk(&mut m, "get\n");
        assert_eq!(said(&p).len(), 32);
        let mut p = Port::default();
        m.tick(&mut p);
        m.tick(&mut p);
        assert_eq!(said(&p).len(), 64);
        let mut p = talk(&mut m, "close\n");
        m.tick(&mut p);
        assert_eq!(said(&p), b"", "nothing after a close");
    }

    #[test]
    fn a_paced_stream_gives_a_buffer_at_a_time_at_the_rate() {
        // 4096 bytes at 40960 samples a second, 8 bits: 100 ms, so 5 ticks.
        let mut m = model(&[("paced", "1"), ("samplerate", "40960"), ("memalloc", "8192")]);
        let p = talk(&mut m, "get\n");
        assert_eq!(said(&p), b"", "nothing until the first buffer is full");
        let mut sizes = Vec::new();
        for _ in 0..20 {
            let mut p = Port::default();
            m.tick(&mut p);
            sizes.push(said(&p).len());
        }
        let at: Vec<_> = sizes.iter().enumerate().filter(|(_, n)| **n > 0).collect();
        assert_eq!(at, [(4, &4096), (9, &4096)]);
    }

    #[test]
    fn at_16_bits_a_paced_buffer_takes_half_the_time() {
        let mut m = model(&[
            ("paced", "1"),
            ("samplerate", "40960"),
            ("memalloc", "4096"),
            ("sampleunit", "0"),
        ]);
        talk(&mut m, "get\n");
        let mut got = None;
        for tick in 0..10 {
            let mut p = Port::default();
            m.tick(&mut p);
            if !said(&p).is_empty() && got.is_none() {
                got = Some(tick);
            }
        }
        assert_eq!(got, Some(1));
    }

    #[test]
    fn chunk_cuts_a_stream_by_ticks_and_first_waits() {
        let mut m = model(&[("memalloc", "5"), ("chunk", "2"), ("first", "1")]);
        assert_eq!(said(&talk(&mut m, "get\n")), b"");
        let mut sizes = Vec::new();
        for _ in 0..5 {
            let mut p = Port::default();
            m.tick(&mut p);
            sizes.push(said(&p).len());
        }
        assert_eq!(sizes, [2, 2, 1, 0, 0]);
    }

    #[test]
    fn a_set_on_a_stream_is_refused_as_the_mutex_is_held() {
        let mut m = model(&[]);
        talk(&mut m, "get\n");
        assert_eq!(said(&talk(&mut m, "samplerate 1000\n")), b"ERR\r\n");
        talk(&mut m, "close\n");
        assert_eq!(said(&talk(&mut m, "samplerate 1000\n")), b"OK\r\n");
    }

    #[test]
    fn no_buffers_means_silence_at_get() {
        let mut m = model(&[("memalloc", "0")]);
        assert_eq!(said(&talk(&mut m, "memalloc\n")), b"0\r\n");
        assert_eq!(said(&talk(&mut m, "get\n")), b"");
        let mut p = Port::default();
        m.tick(&mut p);
        assert_eq!(said(&p), b"");
    }

    #[test]
    fn the_faults() {
        let mut m = model(&[("version", "BeagleLogic_Standalone_1.0")]);
        assert_eq!(said(&talk(&mut m, "version\n")), b"BeagleLogic Standalone 1.0\r\n");
        let mut m = model(&[("say", "samplerate:abc"), ("say", "memalloc:")]);
        assert_eq!(said(&talk(&mut m, "samplerate\n")), b"abc\r\n");
        assert_eq!(said(&talk(&mut m, "memalloc\n")), b"\r\n");
        let mut m = model(&[("mute", "samplerate"), ("mute", "sampleunit+")]);
        assert_eq!(said(&talk(&mut m, "samplerate\n")), b"");
        assert_eq!(said(&talk(&mut m, "samplerate 1000\n")), b"OK\r\n");
        assert_eq!(said(&talk(&mut m, "sampleunit 0\n")), b"");
        assert_eq!(said(&talk(&mut m, "sampleunit\n")), b"0\r\n", "it took effect");
        let mut m = model(&[("refuse", "triggerflags")]);
        assert_eq!(said(&talk(&mut m, "triggerflags 1\n")), b"ERR\r\n");
        let mut m = model(&[("hangup", "memalloc")]);
        assert_eq!(said(&talk(&mut m, "memalloc\n")), b"");
        let mut m = model(&[("tail", "3")]);
        talk(&mut m, "get\n");
        assert_eq!(said(&talk(&mut m, "close\n")), [0xa5, 0xa5, 0xa5]);
    }

    #[test]
    fn a_late_answer_holds_those_behind_it() {
        let mut m = model(&[("late", "samplerate:2")]);
        assert_eq!(said(&talk(&mut m, "samplerate\nsampleunit\n")), b"");
        let mut p = Port::default();
        m.tick(&mut p);
        m.tick(&mut p);
        assert_eq!(said(&p), b"");
        m.tick(&mut p);
        assert_eq!(said(&p), b"100000000\r\n1\r\n");
    }

    #[test]
    fn a_mark_changes_nothing_of_the_stream() {
        let mut m = model(&[("memalloc", "8"), ("chunk", "4"), ("mark", "8"), ("first", "1")]);
        talk(&mut m, "get\n");
        let mut p = Port::default();
        m.tick(&mut p);
        assert_eq!(said(&p).len(), 4);
        let mut p = Port::default();
        m.tick(&mut p);
        assert_eq!(said(&p).len(), 4);
    }

    #[test]
    fn each_check_bites() {
        let mut m = model(&[]);
        for bad in [
            "memalloc 1024\n",
            "bufunitsize 1024\n",
            "exit\n",
            "state\n",
            "triggerflags\n",
            "nonsense\n",
            "sampleunit 2\n",
            "triggerflags 2\n",
            "samplerate many\n",
            "close\n",
        ] {
            let mut p = talk(&mut m, bad);
            assert_eq!(flagged(&mut p), 1, "{bad}");
        }
        let mut p = talk(&mut m, "get\n");
        assert_eq!(flagged(&mut p), 0);
        let mut p = talk(&mut m, "get\n");
        assert_eq!(flagged(&mut p), 1, "a get on a stream");
        let mut p = talk(&mut m, &"x".repeat(LINE + 1));
        assert_eq!(flagged(&mut p), 1, "a line with no end");

        let mut m = model(&[]);
        let mut p = Port::default();
        m.connected(&mut p);
        assert_eq!(flagged(&mut p), 0);
        m.connected(&mut p);
        assert_eq!(flagged(&mut p), 1, "a second connection");
        m.closed();
        m.closed();
        m.connected(&mut p);
        assert_eq!(flagged(&mut p), 0, "one at a time is fine");
    }

    #[test]
    fn options_are_checked() {
        let bad = |o: &[(&str, &str)]| {
            let o: Vec<_> =
                o.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
            parse(&o).is_err()
        };
        assert!(bad(&[("nope", "1")]));
        assert!(bad(&[("memalloc", "many")]));
        assert!(bad(&[("say", "nocolon")]));
        assert!(bad(&[("late", "nocolon")]));
        assert!(bad(&[("bufunitsize", "0")]));
    }

    #[test]
    fn it_has_no_usb() {
        assert_eq!(build(&[]).unwrap().name(), NAME);
    }
}
