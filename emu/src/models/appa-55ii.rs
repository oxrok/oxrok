// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The APPA 55II logging thermometer, and its clone the RS 55II, on the CDC-ACM
//! shell. It only sends: nothing the host could write reaches it.
//!
//! The protocol comes from the sigrok wiki's page on the device, which was
//! reverse engineered, and from what the driver reads. No firmware source or
//! maker's document was found, and none of this has met a real unit.
//!
//! Every frame is `55 55 TYPE SIZE CONTENT SUM`, the sum being that of all the
//! bytes before it, modulo 256. The types:
//!
//! - `00`, live, 20 bytes, about three a second. Bytes 14-15 and 17-18 are the
//!   probes in signed little endian, bytes 16 and 19 their flags: bit 0 for
//!   tenths, bits 2-3 the unit, bit 5 no probe, bit 6 still booting. Byte 13 is
//!   the secondary display's mode: `0x20` in the high nibble is hold, bits 2-3
//!   choose max, min or average.
//! - `18`, a log's start, 1 byte: live frames pause until its end.
//! - `11`, its metadata, 8 bytes: the number of records, a size the wiki gives a
//!   formula for, and zeroes.
//! - `14`, a piece of the log memory, 32 bytes as far as the wiki knows, with a
//!   question mark. The pieces make up records of 20 bytes: hour, minute and
//!   second at 2-4, the probes at 12-13 and 14-15 in tenths, `0x7fff` for none.
//! - `19`, the end, no content: live frames start again.
//!
//! Sending a log is something a user starts on the device. No request exists, so
//! `after=N` stands for that button.
//!
//! Emulation artifacts:
//!
//! - A real unit is a UART behind an optical cable, an IC50 or IC70. This one is
//!   a CDC-ACM function with invented ids, behind 9600/8n1.
//! - Pacing counts offers of a read, about 20 ms each, from each open, so a tool
//!   that closes within a period of the last frame it needed has caused exactly
//!   those. The default of 17 is the unit's three a second: 11 frames came in 3 s
//!   on a tester. A tool flushes what came before its open is done, and a scan
//!   gives up after 500 ms, so the first frame comes between the two, [`LEAD`]
//!   offers from the open.
//! - The stream and the log start over at each open, where a real unit neither
//!   knows of an open nor sends a log twice for one button.
//! - With `burst=`, the first open after power on gets one frame and no log: that
//!   is a scan, which needs one and closes, so what a case's capture gets does not
//!   hang on how far the scan got. Only a model that counts opens can do this.
//! - A log's pieces come one per offer. On the wire they follow one another at
//!   9600 baud, 38 ms each. `gap=0` sends them all at once.
//! - The last piece is padded with `ff`, as an erased EEPROM would read. That the
//!   piece is padded at all is an inference: the wiki's formula counts in whole
//!   pieces, and a driver that keeps the rest shows no more.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `reading=T1/F1/T2/F2/MODE[,...]`, the live frames played in turn: the probes
//!   in decimal, their flags and the mode in hex;
//! - `packet=HEX[,HEX...]`, whole frames played in turn instead, sum included, or
//!   any other bytes: what a line can do to a frame;
//! - `period=N`, offers per live frame (1 to 1000, default 17);
//! - `burst=N`, live frames per open, then silence, with the event point marked a
//!   period after the last frame, one on the first open as above; 0 never sends;
//! - `mute=1`, one frame on the first open and nothing ever after: a thermometer
//!   that was switched off behind a scan;
//! - `records=N`, a log of N records (0 to 400, default none);
//! - `after=N`, live frames before the log (default 1, so that a scan sees one);
//! - `gap=N`, offers from one of a log's frames to the next (default 1), 0 for
//!   all at once; the log starts a period after the last live frame before it;
//! - `piece=N`, bytes per piece of the log (1 to 32, default 32);
//! - `claim=N`, the record count the metadata says, if not the truth;
//! - `bad=N`, the log's Nth frame from 0, the start, has a wrong sum.
//!
//! The pedantic check: a byte written is a violation, as the driver never writes.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "appa-55ii";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "The APPA 55II thermometer on a CDC-ACM cable; \
                         reading=T1/F1/T2/F2/MODE[,...], packet=HEX[,HEX...], period=N, burst=N, \
                         mute=1, records=N, after=N, gap=N, piece=N, claim=N, bad=N, and the shell's \
                         stall=, chunk=, in-limit=, in-stall=, out-hold=";

const LIVE: u8 = 0x00;
const METADATA: u8 = 0x11;
const DATA: u8 = 0x14;
const START: u8 = 0x18;
const END: u8 = 0x19;

/// Offers from an open to the first frame, about 160 ms: after a tool's flush, and
/// well within a scan's 500 ms.
const LEAD: u32 = 10;
/// Bytes of a record in the log.
const RECORD: usize = 20;
/// The flags of a probe in tenths of a degree Celsius.
const TENTHS: u8 = 0x05;
/// The secondary display showing T1, in no special mode.
const PLAIN: u8 = 0x01;
/// A record's probe with nothing connected.
const NO_PROBE: i16 = 0x7fff;
/// What a record's byte 17 goes through, for no known reason.
const CYCLE: [u8; 8] = [0x0a, 0x1e, 0x12, 0x26, 0x1a, 0x0e, 0x22, 0x16];

/// A thermometer from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Thermometer>> {
    let mut faults = Faults::default();
    let mut frames = None;
    let mut period = 17;
    let mut burst = None;
    let mut mute = false;
    let mut records = None;
    let mut after = 1;
    let mut gap = 1;
    let mut piece = 32;
    let mut claim = None;
    let mut bad = None;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "reading" => frames = Some(parse_readings(v)?),
            "packet" => frames = Some(parse_hex_list(k, v, 1, 256)?),
            "period" => period = parse_number(k, v, 1..=1000)?,
            "burst" => burst = Some(parse_number(k, v, 0..=1000)?),
            "mute" => mute = parse_number(k, v, 0..=1)? == 1,
            "records" => records = Some(parse_number(k, v, 0..=400)?),
            "after" => after = parse_number(k, v, 0..=1000)?,
            "gap" => gap = parse_number(k, v, 0..=1000)?,
            "piece" => piece = parse_number(k, v, 1..=32)?,
            "claim" => claim = Some(parse_number(k, v, 0..=65535)?),
            "bad" => bad = Some(parse_number(k, v, 0..=100_000)?),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let frames = frames.unwrap_or_else(default_readings);
    let log = records.map_or_else(Vec::new, |n| log_frames(n, claim, piece as usize, bad));
    let thermometer = Thermometer {
        all_frames: Packets::new(frames.clone()),
        frames: Packets::new(frames),
        period,
        due: 0,
        running: false,
        opens: 0,
        burst,
        mute,
        scan: false,
        burst_left: burst,
        quiet: 0,
        marked: false,
        log,
        log_now: false,
        after,
        gap,
        played: 0,
        queue: VecDeque::new(),
        wait: 0,
        log_due: false,
    };
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "APPA 55II cable");
    let line = Line::parse("9600/8n1").expect("the line parses");
    Ok(Cdc::new(NAME, &id, faults, thermometer).behind_uart(line))
}

/// A whole frame around `content`.
fn frame(kind: u8, content: &[u8]) -> Vec<u8> {
    let mut out = vec![0x55, 0x55, kind, u8::try_from(content.len()).expect("a frame's content")];
    out.extend_from_slice(content);
    let sum = out.iter().fold(0u8, |sum, b| sum.wrapping_add(*b));
    out.push(sum);
    out
}

/// A live frame. The primary and secondary displays show T1 and T2, as the probes
/// are what the driver reads and nothing else in them has a case.
fn live(t1: i16, f1: u8, t2: i16, f2: u8, mode: u8) -> Vec<u8> {
    let (a, b) = (t1.to_le_bytes(), t2.to_le_bytes());
    let content = [
        1, // K type
        1, // degrees Celsius
        a[0], a[1], f1, 0x01, // primary display
        0, 0, 0x80, 0, // no time shown
        b[0], b[1], f2, mode, // secondary display
        a[0], a[1], f1, b[0], b[1], f2,
    ];
    frame(LIVE, &content)
}

fn default_readings() -> Vec<Vec<u8>> {
    vec![
        live(234, TENTHS, -56, TENTHS, PLAIN),
        live(235, TENTHS, -55, TENTHS, PLAIN),
        live(1372, TENTHS, 0, TENTHS, PLAIN),
    ]
}

/// `T1/F1/T2/F2/MODE[,...]`.
fn parse_readings(value: &str) -> Result<Vec<Vec<u8>>> {
    let bad = || Error::Invalid(format!("reading={value}: T1/F1/T2/F2/MODE[,...]"));
    value
        .split(',')
        .map(|one| {
            let f: Vec<&str> = one.split('/').collect();
            let [t1, f1, t2, f2, mode] = f.as_slice() else {
                return Err(bad());
            };
            let temp = |s: &str| s.parse::<i16>().map_err(|_| bad());
            let hex = |s: &str| u8::from_str_radix(s, 16).map_err(|_| bad());
            Ok(live(temp(t1)?, hex(f1)?, temp(t2)?, hex(f2)?, hex(mode)?))
        })
        .collect()
}

/// A record of the log, by its index, under 400: the fields the wiki knows of.
/// The time runs from noon, the probes have a value from the index, and every
/// fifth T2 has no probe.
fn record(index: u16) -> [u8; RECORD] {
    let i = index as i16;
    let t1 = 200 + 3 * i;
    let t2 = if index % 5 == 4 { NO_PROBE } else { i - 40 };
    let place = (u32::from(index) * 20 / 32 + 4) * 32;
    let mut r = [0u8; RECORD];
    r[2] = 12;
    r[3] = (index / 60) as u8;
    r[4] = (index % 60) as u8;
    r[11] = 1;
    r[12..14].copy_from_slice(&t1.to_le_bytes());
    r[14..16].copy_from_slice(&t2.to_le_bytes());
    r[17] = CYCLE[usize::from(index) % CYCLE.len()];
    r[18..20].copy_from_slice(&(place as u16).to_le_bytes());
    r
}

/// The frames of a log: computed when the model is built, as no reply may take
/// long.
fn log_frames(records: u32, claim: Option<u32>, piece: usize, bad: Option<u32>) -> Vec<Vec<u8>> {
    let n = records as usize;
    let mut memory = Vec::with_capacity(n * RECORD);
    for i in 0..records {
        memory.extend_from_slice(&record(i as u16));
    }
    memory.resize(memory.len().div_ceil(piece) * piece, 0xff);
    let count = (claim.unwrap_or(records) as u16).to_le_bytes();
    let size = (((n * RECORD / 32 + 4) * 32) as u16).to_le_bytes();
    let mut frames = vec![
        frame(START, &[0]),
        frame(METADATA, &[count[0], count[1], 0, 0, size[0], size[1], 0, 0]),
    ];
    frames.extend(memory.chunks(piece).map(|p| frame(DATA, p)));
    frames.push(frame(END, &[]));
    if let Some(f) = bad.and_then(|b| frames.get_mut(b as usize))
        && let Some(sum) = f.last_mut()
    {
        *sum = sum.wrapping_add(1);
    }
    frames
}

struct Thermometer {
    /// The frames from the first, for an open.
    all_frames: Packets,
    frames: Packets,
    period: u32,
    /// Offers until the next live frame.
    due: u32,
    /// There has been an open.
    running: bool,
    opens: u32,
    burst: Option<u32>,
    mute: bool,
    /// This open is the first of a burst's or a mute model's: a scan's.
    scan: bool,
    burst_left: Option<u32>,
    quiet: u32,
    marked: bool,
    log: Vec<Vec<u8>>,
    /// The log plays on this open.
    log_now: bool,
    after: u32,
    gap: u32,
    /// Live frames since the open.
    played: u32,
    /// What is left of the log.
    queue: VecDeque<Vec<u8>>,
    wait: u32,
    /// `after=0`: the log's wait starts at the first offer.
    log_due: bool,
}

impl Thermometer {
    /// A period from now, so that a scan that took the last live frame has closed
    /// before the log begins; [`LEAD`] from an open.
    fn start_log(&mut self) {
        self.queue = self.log.iter().cloned().collect();
        self.wait = if self.played == 0 { LEAD } else { self.period };
    }

    /// Plays the log's next frame: false when there is none.
    fn play_log(&mut self, port: &mut Port<'_>) -> bool {
        if self.queue.is_empty() {
            return false;
        }
        if self.wait > 0 {
            self.wait -= 1;
            return true;
        }
        if self.gap == 0 {
            let all: Vec<u8> = self.queue.drain(..).flatten().collect();
            port.send(&all);
            return true;
        }
        if let Some(f) = self.queue.pop_front() {
            port.send(&f);
        }
        self.wait = self.gap - 1;
        // Live frames start again a period after the last, as the stream's count
        // stood still and was 0 when the log began.
        true
    }
}

impl Serial for Thermometer {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        port.violation(format!("{} bytes written to a thermometer that only sends", data.len()));
    }

    fn changed(&mut self, event: LineEvent, _port: &mut Port<'_>) {
        if event.opened() {
            // oxrok's open raises the lines twice, as it holds two descriptors, so
            // only an open that follows a frame is another open.
            if self.opens == 0 || self.played > 0 {
                self.opens = self.opens.saturating_add(1);
            }
            self.scan = self.opens == 1 && (self.burst.is_some() || self.mute);
            let scan = self.scan;
            self.running = true;
            self.frames = self.all_frames.clone();
            self.due = LEAD;
            self.marked = false;
            self.burst_left = match (scan, self.mute) {
                (true, true) => Some(1),
                (false, true) => Some(0),
                (true, false) => self.burst.map(|b| b.min(1)),
                (false, false) => self.burst,
            };
            self.quiet = 0;
            self.played = 0;
            self.queue.clear();
            self.log_now = !scan && !self.log.is_empty();
            self.log_due = self.log_now && self.after == 0;
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if std::mem::take(&mut self.log_due) {
            self.start_log();
        }
        if self.play_log(port) {
            return;
        }
        if !self.running {
            return;
        }
        if self.burst_left != Some(0) {
            self.due = self.due.saturating_sub(1);
            if self.due == 0 {
                self.due = self.period;
                port.send(&self.frames.take());
                self.played = self.played.saturating_add(1);
                if let Some(left) = &mut self.burst_left {
                    *left -= 1;
                }
                if self.log_now && self.played == self.after {
                    self.start_log();
                }
            }
        } else if !self.marked && !self.scan {
            // A period after the last frame, so that the tool has read it
            // before an event lands: bytes just delivered are lost to an
            // unplug's hangup, or not, as the pace has it.
            self.quiet += 1;
            if self.quiet >= self.period {
                port.mark();
                self.marked = true;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    fn opened(options: &[(&str, &str)]) -> Host<Thermometer> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true)
    }

    /// What `n` offers bring.
    fn offers(h: &mut Host<Thermometer>, n: u32) -> Vec<u8> {
        let mut out = Vec::new();
        for _ in 0..n {
            if let Reply::Data(d) = h.offer() {
                out.extend(d);
            }
        }
        out
    }

    /// The frames in `bytes`, each checked; panics on anything else.
    fn split(bytes: &[u8]) -> Vec<(u8, Vec<u8>)> {
        let mut out = Vec::new();
        let mut rest = bytes;
        while !rest.is_empty() {
            assert_eq!(rest[..2], [0x55, 0x55]);
            let size = usize::from(rest[3]);
            let sum = rest[..4 + size].iter().fold(0u8, |s, b| s.wrapping_add(*b));
            assert_eq!(rest[4 + size], sum);
            out.push((rest[2], rest[4..4 + size].to_vec()));
            rest = &rest[5 + size..];
        }
        out
    }

    #[test]
    fn a_live_frame_has_its_bytes_where_the_page_says() {
        // 23.4 and -5.6 degrees, in tenths.
        let f = live(234, TENTHS, -56, TENTHS, 0x24);
        assert_eq!(f.len(), 25);
        assert_eq!(f[..4], [0x55, 0x55, 0x00, 20]);
        assert_eq!(f[4 + 13], 0x24);
        assert_eq!(f[4 + 14..4 + 20], [0xea, 0x00, 0x05, 0xc8, 0xff, 0x05]);
        let sum: u32 = f[..24].iter().map(|b| u32::from(*b)).sum();
        assert_eq!(u32::from(f[24]), sum & 0xff);
    }

    #[test]
    fn frames_come_by_the_period_and_start_over_at_an_open() {
        let mut h = opened(&[("period", "4")]);
        assert_eq!(offers(&mut h, LEAD - 1), b"");
        let first = offers(&mut h, 1);
        assert_eq!(first.len(), 25);
        assert_eq!(offers(&mut h, 3), b"");
        assert_ne!(offers(&mut h, 1), first, "the next reading");
        h.set_lines(true, true);
        assert_eq!(offers(&mut h, LEAD - 1), b"", "the count starts over");
        assert_eq!(offers(&mut h, 1), first, "and the readings");
        assert!(h.v.is_empty());
    }

    #[test]
    fn nothing_comes_within_the_lead_of_an_open() {
        let mut h = opened(&[("period", "1")]);
        assert_eq!(offers(&mut h, 9), b"");
        assert_eq!(offers(&mut h, 1).len(), 25);
    }

    #[test]
    fn the_default_is_the_units_three_a_second() {
        let mut h = opened(&[]);
        assert_eq!(offers(&mut h, LEAD).len(), 25);
        assert_eq!(offers(&mut h, 16), b"");
        assert_eq!(offers(&mut h, 1).len(), 25);
    }

    #[test]
    fn readings_play_in_turn() {
        let mut h = opened(&[("period", "1"), ("reading", "1/4/2/4/1,-3/25/32767/45/2c")]);
        let frames = split(&offers(&mut h, LEAD + 2));
        assert_eq!(frames[0].1[14..20], [1, 0, 4, 2, 0, 4]);
        assert_eq!(frames[1].1[13..20], [0x2c, 0xfd, 0xff, 0x25, 0xff, 0x7f, 0x45]);
        assert_eq!(frames[2].1, frames[0].1, "and from the first again");
    }

    #[test]
    fn a_log_comes_whole_between_live_frames() {
        let mut h = opened(&[("period", "2"), ("records", "3")]);
        let frames = split(&offers(&mut h, LEAD + 2 + 5 + 2));
        let kinds: Vec<u8> = frames.iter().map(|f| f.0).collect();
        assert_eq!(kinds, [LIVE, START, METADATA, DATA, DATA, END, LIVE]);
        // (60 / 32 + 4) * 32 is 160.
        assert_eq!(frames[2].1, [3, 0, 0, 0, 0xa0, 0, 0, 0]);
    }

    #[test]
    fn the_memory_holds_the_records_and_pads_the_last_piece() {
        let mut h = opened(&[("period", "1"), ("after", "0"), ("gap", "0"), ("records", "2")]);
        let frames = split(&offers(&mut h, LEAD + 2));
        let data: Vec<u8> =
            frames.iter().filter(|f| f.0 == DATA).flat_map(|f| f.1.clone()).collect();
        assert_eq!(data.len(), 64);
        // 20.3 and -3.9 degrees.
        assert_eq!(data[20 + 12..20 + 16], [203u8, 0, 0xd9, 0xff]);
        assert_eq!(data[20 + 4], 1, "the second's second");
        assert_eq!(data[40..], [0xff; 24]);
    }

    #[test]
    fn every_fifth_t2_has_no_probe() {
        assert_eq!(record(4)[14..16], [0xff, 0x7f]);
        assert_eq!(record(3)[14..16], [0xdb, 0xff]);
    }

    #[test]
    fn a_short_piece_and_a_gap() {
        let mut h = opened(&[
            ("period", "1"),
            ("after", "0"),
            ("gap", "3"),
            ("piece", "20"),
            ("records", "1"),
        ]);
        assert_eq!(offers(&mut h, LEAD + 1).len(), 6, "the start");
        assert_eq!(offers(&mut h, 2), b"");
        assert_eq!(offers(&mut h, 1).len(), 13, "the metadata");
        assert_eq!(offers(&mut h, 2), b"");
        assert_eq!(offers(&mut h, 1).len(), 25, "one record");
    }

    #[test]
    fn claim_and_bad_change_only_what_they_say() {
        let mut h = opened(&[
            ("period", "1"),
            ("after", "0"),
            ("gap", "0"),
            ("records", "1"),
            ("claim", "9"),
            ("bad", "1"),
        ]);
        let bytes = offers(&mut h, LEAD + 2);
        assert_eq!(bytes[6 + 4], 9);
        let right: u32 = bytes[6..6 + 12].iter().map(|b| u32::from(*b)).sum();
        assert_eq!(u32::from(bytes[6 + 12]), (right + 1) & 0xff);
    }

    #[test]
    fn with_a_burst_a_scan_gets_one_frame_and_no_log() {
        let mut h = opened(&[("period", "2"), ("burst", "3"), ("records", "1")]);
        assert_eq!(offers(&mut h, LEAD + 20).len(), 25, "the first open");
        h.set_lines(true, true);
        let frames = split(&offers(&mut h, LEAD + 2 + 2 + 4 + 2 + 2));
        let kinds: Vec<u8> = frames.iter().map(|f| f.0).collect();
        assert_eq!(kinds, [LIVE, START, METADATA, DATA, END, LIVE, LIVE], "the next");
    }

    #[test]
    fn mute_sends_one_frame_and_no_more() {
        let mut h = opened(&[("period", "2"), ("mute", "1")]);
        assert_eq!(offers(&mut h, LEAD + 20).len(), 25);
        assert!(!h.cdc.mark(), "not at a scan");
        h.set_lines(true, true);
        assert_eq!(offers(&mut h, LEAD + 20), b"");
        assert!(h.cdc.mark(), "the capture's quiet point");
    }

    #[test]
    fn a_burst_ends_in_silence_and_the_point() {
        let mut h = opened(&[("period", "2"), ("burst", "1")]);
        assert_eq!(offers(&mut h, LEAD + 20).len(), 25, "the scan");
        assert!(!h.cdc.mark(), "a scan is left alone");
        h.set_lines(true, true);
        assert_eq!(offers(&mut h, LEAD).len(), 25);
        assert_eq!(offers(&mut h, 1), b"");
        assert!(!h.cdc.mark(), "a period later");
        assert_eq!(offers(&mut h, 1), b"");
        assert!(h.cdc.mark());
        assert_eq!(offers(&mut h, 10), b"");
        assert!(!h.cdc.mark(), "once");
        let mut never = opened(&[("period", "1"), ("burst", "0")]);
        assert_eq!(offers(&mut never, LEAD + 10), b"");
    }

    #[test]
    fn a_byte_written_is_a_violation() {
        let mut h = opened(&[]);
        h.write(b"x");
        assert_eq!(h.v.drain().len(), 1);
    }

    #[test]
    fn a_misset_line_shows_nothing() {
        let mut h = opened(&[("period", "1")]);
        h.set_line(Line::parse("19200/8n1").unwrap());
        assert_eq!(offers(&mut h, LEAD + 5), b"");
        assert!(h.v.is_empty());
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("period", "0"),
            ("piece", "33"),
            ("records", "401"),
            ("reading", "1/2/3"),
            ("reading", "x/4/2/4/1"),
            ("packet", "0"),
            ("nothing", "1"),
        ] {
            assert!(build(&[(bad.0.to_owned(), bad.1.to_owned())]).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("packet", "5555"), ("after", "0"), ("bad", "99")] {
            assert!(build(&[(ok.0.to_owned(), ok.1.to_owned())]).is_ok(), "{ok:?}");
        }
    }
}
