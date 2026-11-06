// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A UNI-T `UT32x` thermometer on a plain serial line, on the CDC-ACM shell. The
//! instrument itself, [`Thermometer`], is also what `uni-t-ut32x-hid` puts behind
//! its CH9325 cable.
//!
//! The device, and how sure this is:
//!
//! - The commands and the record: the sigrok wiki's "UNI-T UT325" page, from
//!   someone's reverse engineering. `01` starts live records, `02` stops, `07`
//!   starts the stored ones, `03` is a calibration mode and `04` twice clears the
//!   memory. A record is 19 bytes: a type digit (`0` stored, `2` live, `6` unknown), four
//!   characters of tenths of a degree with `:` for a blank and `;` for a minus
//!   (`;;;;` is no probe), a unit digit (`1` C, `2` F, `3` K, and `0` "in recall
//!   mode"), a two digit slot number (00 to 99, `00` when live), `0`, hour, minute,
//!   the probe (`0` T1, `1` T2, `2` and `3` T1-T2), two unknown characters, `1`,
//!   CR LF.
//! - The line, 2400/8n1: only the default in libsigrok's scan. The wiki gives none.
//! - The rate: the wiki says 6 Hz, a comment in libsigrok "two or three packets per
//!   second", and a libsigrok commit message (f0e6b41f) "some 60 times per second".
//!   Nothing depends on it here, as a record is due every `period=` offers
//!   of a read, not after a time.
//! - How a memory dump is paced, what it sends for a free slot and how it ends: not
//!   found. A libsigrok comment says unused slots "come through" as no measurement,
//!   so the dump here has one record per slot, 100 of them from the wiki's range,
//!   and then silence. That it starts over at each `07` is a guess too.
//! - No recording and no firmware source were found, and no unit was on the bench.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id, where a real one has a HID
//!   cable (`uni-t-ut32x-hid`): only the byte stream and the line are the device's.
//! - `burst=` and `slots=` make it fall silent, so that a case's count of records
//!   depends on no race with a tool's stop. A real unit goes on until told.
//! - A record goes out at once, in one piece unless the shell's `chunk=` cuts it.
//! - Hour and minute stand still, as there is no clock here.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `live=HEX[,HEX...]`, what a live run sends in turn, 1 to 64 bytes each, so also
//!   noise and records no unit sends;
//! - `stored=HEX[,HEX...]`, the first memory slots, the rest being free;
//! - `slots=N`, how many there are (100);
//! - `period=N`, offers of a read per record (25, about half a second);
//! - `burst=N`, records per start, then silence (no limit);
//! - `uart=LINE`, the line (2400/8n1).
//!
//! The event point (`--unplug model`, `--interrupt model`) is one period after a
//! run has sent its last record, so it needs `burst=` or the end of the memory.
//!
//! The pedantic checks: of the device's commands the host sends only `01`, `02` and
//! `07`. `03` and `04` are real, but no path of the driver sends them, and any other
//! byte is unknown. C habits are taken: a `02` while nothing runs, a start while a
//! run goes on.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, Port, Serial};
use crate::shells::hidcable as cable;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "uni-t-ut32x";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A UNI-T UT32x thermometer on CDC-ACM; live=HEX[,HEX...], \
                         stored=HEX[,HEX...], slots=N (100), period=N (25), burst=N, uart=LINE \
                         (2400/8n1), and the shell's stall=, chunk=, in-limit=, in-stall=, \
                         out-hold=";

/// Start live records.
pub const CMD_LIVE: u8 = 0x01;
/// Stop sending.
pub const CMD_STOP: u8 = 0x02;
/// A calibration mode, says the wiki.
pub const CMD_DEBUG: u8 = 0x03;
/// Twice in a row clears the memory, says the wiki.
pub const CMD_CLEAR: u8 = 0x04;
/// Start the stored records.
pub const CMD_STORED: u8 = 0x07;
/// A record's length.
pub const RECORD_BYTES: usize = 19;

/// Slots in the memory: the wiki's numbers run from 00 to 99.
const SLOTS: u32 = 100;
/// What `period=` is by default: slow enough for a tool's stop to land first.
const PERIOD: u32 = 25;

/// A record, from the fields that vary.
fn record(kind: u8, temp: &[u8], unit: u8, slot: u32, probe: u8) -> Vec<u8> {
    let mut r = vec![kind];
    r.extend_from_slice(temp);
    r.push(unit);
    r.extend_from_slice(format!("{:02}", slot % 100).as_bytes());
    r.extend_from_slice(b"0");
    r.extend_from_slice(b"1230");
    r.push(probe);
    r.extend_from_slice(b"001\r\n");
    r
}

/// What a run is sending.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Run {
    Idle,
    Live,
    Memory,
}

/// The instrument, whatever its cable.
#[derive(Clone, Debug)]
pub struct Thermometer {
    live: Vec<Vec<u8>>,
    stored: Vec<Vec<u8>>,
    slots: u32,
    period: u32,
    burst: Option<u32>,
    run: Run,
    /// Offers since the last record.
    offers: u32,
    /// Records since the last start.
    sent: u32,
    /// The next of `live`, which goes on from one run to the next.
    next_live: usize,
    marked: bool,
}

impl Default for Thermometer {
    fn default() -> Thermometer {
        Thermometer {
            live: vec![
                record(b'2', b":235", b'1', 0, b'0'),
                record(b'2', b":241", b'1', 0, b'1'),
                record(b'2', b";:12", b'1', 0, b'2'),
            ],
            stored: vec![
                record(b'0', b":198", b'0', 0, b'0'),
                record(b'0', b":201", b'0', 1, b'1'),
            ],
            slots: SLOTS,
            period: PERIOD,
            burst: None,
            run: Run::Idle,
            offers: 0,
            sent: 0,
            next_live: 0,
            marked: false,
        }
    }
}

impl Thermometer {
    /// Takes an option if it is one of the instrument's.
    ///
    /// # Errors
    ///
    /// On a value out of range.
    pub fn option(&mut self, key: &str, value: &str) -> Result<bool> {
        match key {
            "live" => self.live = cable::parse_packets(value, 1, 64)?,
            "stored" => self.stored = cable::parse_packets(value, 1, 64)?,
            "slots" => self.slots = parse_number(key, value, 0..=SLOTS)?,
            "period" => self.period = parse_number(key, value, 1..=100_000)?,
            "burst" => self.burst = cable::parse_burst(value)?,
            _ => return Ok(false),
        }
        Ok(true)
    }

    /// What the host's bytes do; what no path of the driver sends goes to
    /// `violation`.
    pub fn commands(&mut self, data: &[u8], mut violation: impl FnMut(String)) {
        for &c in data {
            match c {
                CMD_LIVE => self.start(Run::Live),
                CMD_STORED => self.start(Run::Memory),
                CMD_STOP => self.run = Run::Idle,
                CMD_DEBUG | CMD_CLEAR => {
                    violation(format!("command {c:02x}: real, but the driver never sends it"));
                }
                other => violation(format!("command {other:02x}: not one the device has")),
            }
        }
    }

    fn start(&mut self, run: Run) {
        self.run = run;
        self.offers = 0;
        self.sent = 0;
        self.marked = false;
    }

    /// One offer of a read with nothing queued: a record, when one is due.
    pub fn offer(&mut self) -> Option<Vec<u8>> {
        if self.run == Run::Idle {
            return None;
        }
        self.offers += 1;
        if self.offers < self.period {
            return None;
        }
        self.offers = 0;
        let next = self.next_record();
        if next.is_some() {
            self.sent += 1;
        }
        next
    }

    fn next_record(&mut self) -> Option<Vec<u8>> {
        if self.burst.is_some_and(|b| self.sent >= b) {
            return None;
        }
        match self.run {
            Run::Live => {
                let r = self.live[self.next_live].clone();
                self.next_live = (self.next_live + 1) % self.live.len();
                Some(r)
            }
            Run::Memory if self.sent < self.slots => {
                let slot = self.sent;
                Some(self.stored.get(slot as usize).cloned().unwrap_or_else(|| {
                    record(b'0', b";;;;", b'0', slot, b'0')
                }))
            }
            _ => None,
        }
    }

    /// Whether the event point has come: the first offer of a period that sent
    /// nothing as a run has nothing more to send.
    pub fn take_mark(&mut self) -> bool {
        let done = match self.run {
            Run::Idle => false,
            Run::Live => self.burst.is_some_and(|b| self.sent >= b),
            Run::Memory => {
                self.sent >= self.slots || self.burst.is_some_and(|b| self.sent >= b)
            }
        };
        if done && self.offers + 1 >= self.period && !self.marked {
            self.marked = true;
            return true;
        }
        false
    }

    /// Back to power on, keeping the options.
    pub fn power_on(&mut self) {
        self.run = Run::Idle;
        self.offers = 0;
        self.sent = 0;
        self.next_live = 0;
        self.marked = false;
    }
}

impl Serial for Thermometer {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        self.commands(data, |m| port.violation(m));
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if let Some(r) = self.offer() {
            port.send(&r);
        }
        if self.take_mark() {
            port.mark();
        }
    }

    fn reset(&mut self) {
        self.power_on();
    }
}

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
    let mut meter = Thermometer::default();
    let mut uart = Line::parse("2400/8n1")?;
    for (k, v) in options {
        if faults.option(k, v)? || meter.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "uart" => uart = Line::parse(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "UT32x thermometer");
    Ok(Cdc::new(NAME, &id, faults, meter).behind_uart(uart))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    fn opts(o: &[(&str, &str)]) -> Vec<(String, String)> {
        o.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    fn meter(o: &[(&str, &str)]) -> Host<Thermometer> {
        let cdc = function(&opts(o)).unwrap();
        Host::open(cdc, Line::parse("2400/8n1").unwrap(), true, true)
    }

    /// What `n` offers of a read bring.
    fn offers(h: &mut Host<Thermometer>, n: usize) -> Vec<u8> {
        let mut out = Vec::new();
        for _ in 0..n {
            if let Reply::Data(d) = h.offer() {
                out.extend(d);
            }
        }
        out
    }

    #[test]
    fn a_record_has_the_wikis_layout() {
        let r = record(b'2', b";:12", b'3', 7, b'1');
        assert_eq!(r, b"2;:12307012301001\r\n");
        assert_eq!(r.len(), RECORD_BYTES);
        assert_eq!((r[8], r[13], r[16]), (b'0', b'1', b'1'));
        assert_eq!(&r[6..8], b"07");
    }

    #[test]
    fn it_is_silent_until_told_and_stops_when_told() {
        let mut h = meter(&[("period", "2")]);
        assert_eq!(offers(&mut h, 20), [] as [u8; 0]);
        h.write(&[CMD_LIVE]);
        let got = offers(&mut h, 2);
        assert_eq!(got, record(b'2', b":235", b'1', 0, b'0'));
        h.write(&[CMD_STOP]);
        assert_eq!(offers(&mut h, 20), [] as [u8; 0]);
        assert!(h.v.is_empty(), "{:?}", h.v);
    }

    #[test]
    fn live_records_go_round_and_carry_on_across_a_stop() {
        let mut h = meter(&[("period", "1"), ("live", "41,42")]);
        h.write(&[CMD_LIVE]);
        assert_eq!(offers(&mut h, 3), b"ABA");
        h.write(&[CMD_STOP]);
        h.write(&[CMD_LIVE]);
        assert_eq!(offers(&mut h, 1), b"B");
    }

    #[test]
    fn the_memory_is_its_slots_and_then_silence() {
        let mut h = meter(&[("period", "1"), ("stored", "41"), ("slots", "3")]);
        h.write(&[CMD_STORED]);
        let mut want = b"A".to_vec();
        want.extend(record(b'0', b";;;;", b'0', 1, b'0'));
        want.extend(record(b'0', b";;;;", b'0', 2, b'0'));
        assert_eq!(offers(&mut h, 30), want);
        h.write(&[CMD_STORED]);
        assert_eq!(offers(&mut h, 1), b"A", "from the start");
    }

    #[test]
    fn a_burst_ends_a_run_and_the_next_start_gets_another() {
        let mut h = meter(&[("period", "1"), ("live", "41"), ("burst", "2")]);
        h.write(&[CMD_LIVE]);
        assert_eq!(offers(&mut h, 10), b"AA");
        h.write(&[CMD_LIVE]);
        assert_eq!(offers(&mut h, 10), b"AA");
        let mut none = meter(&[("period", "1"), ("burst", "0")]);
        none.write(&[CMD_LIVE]);
        assert_eq!(offers(&mut none, 5), [] as [u8; 0]);
    }

    #[test]
    fn the_event_point_comes_a_period_after_the_last_record_and_once() {
        let mut m = Thermometer::default();
        assert!(m.option("period", "3").unwrap() && m.option("burst", "1").unwrap());
        m.commands(&[CMD_LIVE], |e| panic!("{e}"));
        assert!(!m.take_mark(), "nothing sent yet");
        assert!(m.offer().is_none() && m.offer().is_none());
        assert!(m.offer().is_some());
        assert!(!m.take_mark(), "a period has not gone by");
        assert!(m.offer().is_none() && m.offer().is_none());
        assert!(m.take_mark());
        assert!(!m.take_mark(), "once");
        let mut stopped = Thermometer::default();
        assert!(!stopped.take_mark(), "idle");
        stopped.commands(&[CMD_STORED], |e| panic!("{e}"));
        assert!(stopped.option("slots", "0").unwrap() && stopped.option("period", "1").unwrap());
        assert!(stopped.offer().is_none());
        assert!(stopped.take_mark(), "an empty memory ends at once");
    }

    #[test]
    fn the_commands_no_path_sends_are_flagged() {
        let mut h = meter(&[]);
        h.write(&[CMD_DEBUG]);
        assert_eq!(h.v.drain().len(), 1);
        h.write(&[CMD_CLEAR, CMD_CLEAR]);
        assert_eq!(h.v.drain().len(), 2);
        h.write(&[0x00, 0x08, 0xff]);
        assert_eq!(h.v.drain().len(), 3);
        h.write(&[CMD_STOP, CMD_STOP, CMD_LIVE, CMD_LIVE, CMD_STORED]);
        assert!(h.v.is_empty(), "C's habits are taken: {:?}", h.v);
    }

    #[test]
    fn at_another_rate_it_hears_nothing() {
        let cdc = function(&opts(&[("period", "1")])).unwrap();
        let mut h = Host::open(cdc, Line::parse("9600/8n1").unwrap(), true, true);
        h.write(&[CMD_LIVE]);
        assert_eq!(offers(&mut h, 5), [] as [u8; 0]);
        let cdc = function(&opts(&[("period", "1"), ("uart", "9600/8n1")])).unwrap();
        let mut h = Host::open(cdc, Line::parse("9600/8n1").unwrap(), true, true);
        h.write(&[CMD_LIVE]);
        assert_eq!(offers(&mut h, 1).len(), RECORD_BYTES);
    }

    #[test]
    fn a_reset_is_power_on() {
        let mut h = meter(&[("period", "1"), ("live", "41,42")]);
        h.write(&[CMD_LIVE]);
        assert_eq!(offers(&mut h, 1), b"A");
        h.cdc.reset();
        h = Host::open(h.cdc, Line::parse("2400/8n1").unwrap(), true, true);
        assert!(offers(&mut h, 3).is_empty(), "idle again");
        h.write(&[CMD_LIVE]);
        assert_eq!(offers(&mut h, 1), b"A", "from the first again");
    }

    #[test]
    fn options_are_checked() {
        assert!(build(&opts(&[("nothing", "1")])).is_err());
        assert!(build(&opts(&[("period", "0")])).is_err());
        assert!(build(&opts(&[("slots", "101")])).is_err());
        assert!(build(&opts(&[("live", "zz")])).is_err());
        assert!(build(&opts(&[("uart", "fast")])).is_err());
        assert!(build(&opts(&[("chunk", "1"), ("burst", "2")])).is_ok());
    }
}
