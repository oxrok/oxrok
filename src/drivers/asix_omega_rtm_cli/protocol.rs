// Counterpart in sigrok:
//   libsigrok/src/hardware/asix-omega-rtm-cli/api.c,
//   libsigrok/src/hardware/asix-omega-rtm-cli/protocol.c,
//   libsigrok/src/hardware/asix-omega-rtm-cli/protocol.h,
//   Copyright (C) 2021 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The pure parts of `asix-omega-rtm-cli`: the `conn=` spec, the `-version`
//! banner, the stream's decoding and the silence bound.
//!
//! The stream (`omegartmcli -bin`, ASIX's `omegartmcli.dpr` and `omegartmcliU.pas`)
//! is chunks of six bytes, three 16 bit words least significant byte first: the
//! time since the chunk before in 10 ns ticks, then two samples 5 ns apart. The
//! program sends `tslen + 1` of a decoded vector, 1 to 32768. A chunk of `T` ticks
//! follows the one before by `2 * T` samples, of which the last two are its own, so
//! the last sample before it repeats `2 * (T - 1)` times. The first chunk's time
//! means nothing and its first sample is dropped: its second is the value the next
//! chunk's repeats start from (`protocol.c`, `omega_rtm_cli_open`).

use std::time::{Duration, Instant};

/// The one rate: samples 5 ns apart.
pub const SAMPLERATE: u64 = 200_000_000;
/// The bytes of a chunk.
pub const CHUNK: usize = 6;
/// The channel names, `api.c`'s.
pub const CHANNEL_NAMES: [&str; 16] =
    ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16"];
/// The most samples a logic packet holds: `FEED_QUEUE_DEPTH`.
pub const FEED_QUEUE_DEPTH: usize = 256 * 1024;
/// How long the program may say nothing before a warning. It hands over what it
/// decoded at least every 100 ms (`Decode2Flush`), and an idle input still makes a
/// chunk each 32768 ticks.
pub const SILENCE_WARN: Duration = Duration::from_secs(5);
/// How long the program may say nothing, after a warning, before the capture fails.
pub const SILENCE_LIMIT: Duration = Duration::from_secs(600);

/// What `conn=` asks for.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Conn {
    /// The serial number the program gets in `-serial`.
    pub serial: Option<String>,
    /// The connection id the device reports, `conn=` as given.
    pub conn: Option<String>,
}

/// `scan`'s reading of `conn=`, with its messages: `None` is no device.
pub fn parse_conn(conn: Option<&str>) -> Option<Conn> {
    let Some(conn) = conn.filter(|c| !c.is_empty()) else {
        return Some(Conn::default());
    };
    let Some(serno) = conn.strip_prefix("sn=") else {
        log::error!("conn= must specify a serial number.");
        return None;
    };
    let mut serial = (!serno.is_empty()).then(|| serno.to_owned());
    if let Some(s) = &serial {
        log::debug!("User specified serial number: {s}");
        if s.len() == 4 {
            log::debug!("Adding 03 prefix to user specified serial number");
            serial = Some(format!("03{s}"));
        }
    }
    if serial.as_ref().is_some_and(|s| s.len() != 6 && s.len() != 8) {
        log::error!("Serial number must be 03xxxx or A603xxxx");
        serial = None;
    }
    Some(Conn { serial, conn: Some(conn.to_owned()) })
}

/// The version in a `-version` output: what follows `Version ` up to the line's
/// end. C reads the output as a C string, so a NUL ends it.
pub fn parse_version(output: &[u8]) -> Option<String> {
    const KEY: &[u8] = b"Version ";
    let text = output.split(|b| *b == 0).next().unwrap_or_default();
    let at = text.windows(KEY.len()).position(|w| w == KEY)?;
    let rest = &text[at + KEY.len()..];
    let rest = rest.split(|b| *b == b'\n').next().unwrap_or_default();
    let rest = rest.split(|b| *b == b'\r').next().unwrap_or_default();
    (!rest.is_empty()).then(|| String::from_utf8_lossy(rest).into_owned())
}

/// Where decoded samples gather until a packet's worth: `feed_queue_logic`.
#[derive(Debug, Default)]
pub struct Queue {
    buf: Vec<u8>,
}

impl Queue {
    /// Adds `count` copies of `sample`, handing each full packet to `send`. False
    /// once `send` says so.
    pub fn push(
        &mut self,
        sample: [u8; 2],
        count: u64,
        send: &mut dyn FnMut(Vec<u8>) -> bool,
    ) -> bool {
        let mut left = count;
        while left > 0 {
            let room = FEED_QUEUE_DEPTH - self.buf.len() / 2;
            let n = left.min(room as u64) as usize;
            self.buf.reserve(n * 2);
            for _ in 0..n {
                self.buf.extend_from_slice(&sample);
            }
            left -= n as u64;
            if self.buf.len() / 2 == FEED_QUEUE_DEPTH && !self.flush(send) {
                return false;
            }
        }
        true
    }

    /// Hands what there is to `send`, if anything. False once `send` says so.
    pub fn flush(&mut self, send: &mut dyn FnMut(Vec<u8>) -> bool) -> bool {
        if self.buf.is_empty() {
            return true;
        }
        send(std::mem::take(&mut self.buf))
    }
}

/// What a feed came to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fed {
    /// More is wanted.
    More,
    /// The sample limit is met.
    Done,
    /// The sink is gone, or the stop came.
    Gone,
}

/// The stream's decoder: `omega_rtm_cli_process_rawdata`.
#[derive(Debug)]
pub struct Decoder {
    last: [u8; 2],
    /// Samples still wanted, under a limit.
    remain: Option<u64>,
    /// Bytes of a chunk not all here yet.
    partial: Vec<u8>,
    /// Samples sent.
    pub samples: u64,
    /// Chunks whose time was 0, which the program never sends.
    pub zero_stamps: u64,
}

impl Decoder {
    /// A decoder after the first chunk, whose second sample is `last`, with
    /// `limit` samples wanted (0: no limit).
    pub fn new(first: [u8; CHUNK], limit: u64) -> Decoder {
        Decoder {
            last: [first[4], first[5]],
            remain: (limit > 0).then_some(limit),
            partial: Vec::new(),
            samples: 0,
            zero_stamps: 0,
        }
    }

    /// The bytes of a chunk cut off at the end of the stream.
    pub fn leftover(&self) -> usize {
        self.partial.len()
    }

    /// Takes `n` of a limit's samples: how many may go.
    fn take(&mut self, n: u64) -> u64 {
        match &mut self.remain {
            None => n,
            Some(r) => {
                let n = n.min(*r);
                *r -= n;
                n
            }
        }
    }

    fn met(&self) -> bool {
        self.remain == Some(0)
    }

    /// Decodes `data`, the samples going to `queue`. The limit is exact, where C
    /// sends both samples of a chunk when one is left.
    pub fn feed(
        &mut self,
        data: &[u8],
        queue: &mut Queue,
        send: &mut dyn FnMut(Vec<u8>) -> bool,
    ) -> Fed {
        let mut data = data;
        while !data.is_empty() {
            let mut chunk = [0u8; CHUNK];
            if self.partial.is_empty() && data.len() >= CHUNK {
                chunk.copy_from_slice(&data[..CHUNK]);
                data = &data[CHUNK..];
            } else {
                let n = (CHUNK - self.partial.len()).min(data.len());
                self.partial.extend_from_slice(&data[..n]);
                data = &data[n..];
                if self.partial.len() < CHUNK {
                    break;
                }
                chunk.copy_from_slice(&self.partial);
                self.partial.clear();
            }
            match self.chunk(chunk, queue, send) {
                Fed::More => {}
                other => return other,
            }
        }
        Fed::More
    }

    fn chunk(
        &mut self,
        c: [u8; CHUNK],
        queue: &mut Queue,
        send: &mut dyn FnMut(Vec<u8>) -> bool,
    ) -> Fed {
        let stamp = u16::from_le_bytes([c[0], c[1]]);
        if stamp == 0 {
            self.zero_stamps += 1;
        }
        let repeats = self.take(2 * u64::from(stamp.saturating_sub(1)));
        if repeats > 0 {
            if !queue.push(self.last, repeats, send) {
                return Fed::Gone;
            }
            self.samples += repeats;
        }
        if self.met() {
            return Fed::Done;
        }
        for s in [[c[2], c[3]], [c[4], c[5]]] {
            if self.take(1) == 0 {
                break;
            }
            self.last = s;
            if !queue.push(s, 1, send) {
                return Fed::Gone;
            }
            self.samples += 1;
        }
        if self.met() { Fed::Done } else { Fed::More }
    }
}

/// What a look at the silence says.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Nothing to say.
    Wait,
    /// Say this, once, and go on.
    Warn(String),
    /// The capture fails with this.
    Fail(String),
}

/// The bound on a program that says nothing, over [`Instant`]s the caller takes.
/// The time of a read that found nothing is the time taken before it, so a stalled
/// host isn't taken for a silent program.
#[derive(Debug)]
pub struct Silence {
    last: Instant,
    warned: bool,
    warn: Duration,
    limit: Duration,
}

impl Silence {
    /// A bound starting at `now`, the time of the last data.
    pub fn new(now: Instant) -> Silence {
        Silence { last: now, warned: false, warn: SILENCE_WARN, limit: SILENCE_LIMIT }
    }

    /// The same with other times, for tests.
    pub fn with(now: Instant, warn: Duration, limit: Duration) -> Silence {
        Silence { last: now, warned: false, warn, limit }
    }

    /// Data came at `now`: a line when it comes back after a warning.
    pub fn got(&mut self, now: Instant) -> Option<String> {
        let back = self.warned.then(|| {
            format!(
                "Data from the vendor application again, after {:.1} s.",
                now.saturating_duration_since(self.last).as_secs_f64()
            )
        });
        self.last = now;
        self.warned = false;
        back
    }

    /// A read that began at `looked` found nothing.
    pub fn empty(&mut self, looked: Instant) -> Verdict {
        let quiet = looked.saturating_duration_since(self.last);
        if quiet >= self.limit {
            return Verdict::Fail(format!(
                "No data from the vendor application for {} s.",
                quiet.as_secs()
            ));
        }
        if quiet >= self.warn && !self.warned {
            self.warned = true;
            return Verdict::Warn(format!(
                "No data from the vendor application for {} s.",
                quiet.as_secs()
            ));
        }
        Verdict::Wait
    }
}
