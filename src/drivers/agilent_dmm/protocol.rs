// Counterpart in sigrok:
//   libsigrok/src/hardware/agilent-dmm/api.c,
//   libsigrok/src/hardware/agilent-dmm/protocol.c,
//   libsigrok/src/hardware/agilent-dmm/protocol.h,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Aurelien Jacobs,
//   Martin Ling, Daniel Elstner, Soeren Apel, Gerd v. Egidy, Gerhard Sittig,
//   Wolfram Sang, Alex Hornung
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The meters' text protocol.
//!
//! A query ends in LF, and the driver then adds CR LF: `FETC?\n\r\n`, as a meter
//! takes an empty line. Only `*IDN?` goes with CR LF alone. An answer ends in CR
//! LF, and comes unasked too: `*N` when the rotary switch moves, `*E` for a
//! command the meter did not take.
//!
//! | Query | Answer | Means |
//! |-------|--------|-------|
//! | `*IDN?` | `Agilent Technologies,U1253B,serial,V2.26` | the scan |
//! | `CONF?`, `CONF? @2` | `"V,0,AC"`, `"VOLT:AC +1.0E+00,+1.0E-04"` | what a display shows, its resolution |
//! | `STAT?` | `"000000000110L00000000"` | modes, one letter each |
//! | `FETC?`, `FETC? @2`, `FETC? @3` | `+1.23475000E+00` | the value; `@3` the temperature |
//! | `LOG:HAND N` and kin | `"01000241100100"` | entry N of a log; `*E` past the last |
//!
//! Each family of meters takes its own set of lines, as [`parse`] has them from
//! the C driver's tables of regular expressions, first match first. C's `.` and
//! `\d` come from `GLib` in `UTF-8` mode: here a line that is no `UTF-8` is unknown,
//! and `\d` is an ASCII digit.
//!
//! [`Meter`] is the C's `dev_context` and its jobs: a job's query goes out when
//! its interval is over, one at a time, and a job ends with its answer or after
//! [`JOB_TIMEOUT`]. It takes the time as an argument, so it needs no port.

use crate::core::{Mq, MqFlags, Unit};
use std::time::{Duration, Instant};

/// A write's time, `SERIAL_WRITE_TIMEOUT_MS`.
pub const WRITE_TIMEOUT: Duration = Duration::from_millis(1);
/// How long a job waits for its answer.
pub const JOB_TIMEOUT: Duration = Duration::from_millis(300);
/// `AGDMM_BUFSIZE` less the NUL: a line and its LF must fit.
pub const LINE_MAX: usize = 255;
/// The identity query.
pub const IDN: &[u8] = b"*IDN?\r\n";
/// `serial_readline`'s buffer in the scan less the NUL.
pub const IDENTITY_MAX: usize = 127;

/// The sources a capture takes, by their `data_source` names.
pub const DATA_SOURCES: [&str; 5] = ["Live", "Log-Hand", "Log-Trig", "Log-Auto", "Log-Export"];
/// `LOG:` commands, one per [`DATA_SOURCES`] entry after `Live`.
const LOG_COMMANDS: [&str; 4] = ["LOG:HAND", "LOG:TRIG", "LOG:AUTO", "LOG:EXPO"];

/// The sets of lines a meter takes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    /// U1231A to U1233A.
    U123x,
    /// U1241A/B, U1242A/B.
    U124x,
    /// U1241C, U1242C.
    U124xc,
    /// U1251A/B to U1253A/B.
    U125x,
    /// U1271A to U1273AX.
    U127x,
    /// U1281A, U1282A.
    U128x,
}

/// One model.
#[derive(Debug, PartialEq, Eq)]
pub struct Profile {
    /// As `*IDN?` says it.
    pub name: &'static str,
    /// Its lines.
    pub family: Family,
    /// The channels, P1 and on.
    pub channels: usize,
}

impl Profile {
    /// Whether it has `LOG:`.
    pub fn has_log(&self) -> bool {
        matches!(self.family, Family::U124xc | Family::U128x)
    }
}

macro_rules! profiles {
    ($($name:literal $family:ident $channels:literal,)*) => {
        /// The series, as `supported_agdmm`.
        pub const PROFILES: &[Profile] = &[
            $(Profile { name: $name, family: Family::$family, channels: $channels },)*
        ];
    };
}

profiles! {
    "U1231A" U123x 1, "U1232A" U123x 1, "U1233A" U123x 1,
    "U1241A" U124x 2, "U1242A" U124x 2, "U1241B" U124x 2, "U1242B" U124x 2,
    "U1241C" U124xc 2, "U1242C" U124xc 2,
    "U1251A" U125x 3, "U1252A" U125x 3, "U1253A" U125x 3,
    "U1251B" U125x 3, "U1252B" U125x 3, "U1253B" U125x 3,
    "U1271A" U127x 3, "U1272A" U127x 3, "U1273A" U127x 3, "U1273AX" U127x 3,
    "U1281A" U128x 3, "U1282A" U128x 3,
}

/// What a scan learns from `*IDN?`.
#[derive(Debug, PartialEq, Eq)]
pub struct Identity {
    /// `Agilent` or `Keysight`.
    pub vendor: &'static str,
    /// The model.
    pub profile: &'static Profile,
    /// All after the third comma.
    pub version: String,
}

/// An `*IDN?` answer, its line end gone. C reads it as a string, so it ends at a
/// NUL. `api.c:146` hands `strcmp` a `NULL` for a line that is empty from there;
/// that is no device here.
pub fn parse_identity(line: &[u8]) -> Option<Identity> {
    let line = until_nul(line);
    let mut fields = line.splitn(4, |&b| b == b',');
    let maker = fields.next()?;
    let vendor = match maker {
        b"Agilent Technologies" => "Agilent",
        b"Keysight Technologies" => "Keysight",
        _ => return None,
    };
    let model = fields.next()?;
    fields.next()?;
    let version = fields.next()?;
    let profile = PROFILES.iter().find(|p| p.name.as_bytes() == model)?;
    Some(Identity { vendor, profile, version: String::from_utf8_lossy(version).into_owned() })
}

fn until_nul(line: &[u8]) -> &[u8] {
    line.iter().position(|&b| b == 0).map_or(line, |at| &line[..at])
}

/// A line a family knows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reply<'a> {
    /// `STAT?`'s letters.
    Stat(&'a [u8]),
    /// The rotary switch moved.
    Switch,
    /// A value, with its exponent for the families whose table captures it.
    Value(&'a str, Option<i32>),
    /// `"V,0,AC"`: function, resolution, coupling.
    ConfFields(&'a str, Option<i32>, Option<&'a str>),
    /// `"VOLT:AC +1.0E+00,+1.0E-04"`: function, the word after a temperature's,
    /// the resolution's exponent.
    Conf(&'a str, Option<&'a str>, Option<i32>),
    /// A log entry, fourteen digits.
    LogEntry(&'a str),
    /// `*E`.
    Error,
}

fn all_digits(text: &str) -> bool {
    !text.is_empty() && text.bytes().all(|b| b.is_ascii_digit())
}

fn quoted(line: &str) -> Option<&str> {
    line.strip_prefix('"')?.strip_suffix('"')
}

/// `"(\d\d.{18}\d)"`, by characters as C's regular expression counts; C then
/// looks at its bytes.
fn stat(line: &str) -> Option<&str> {
    let inner = quoted(line)?;
    let mut chars = inner.chars();
    let two = inner.get(..2).is_some_and(all_digits);
    (two && chars.clone().count() == 21 && chars.next_back().is_some_and(|c| c.is_ascii_digit()))
        .then_some(inner)
}

/// `\*([0-9])`, or `+` of them on a U127x.
fn switch(line: &str, many: bool) -> bool {
    line.strip_prefix('*').is_some_and(|n| all_digits(n) && (many || n.len() == 1))
}

/// `[-+]`.
fn signed(text: &str) -> Option<&str> {
    text.strip_prefix(['-', '+'])
}

/// `[-+][0-9]\.[0-9]{min,max}E([-+][0-9]{2})`: its exponent.
fn number(text: &str, min: usize, max: usize) -> Option<i32> {
    let rest = signed(text)?;
    let rest = rest.strip_prefix(|c: char| c.is_ascii_digit())?.strip_prefix('.')?;
    let (decimals, exponent) = rest.split_once('E')?;
    if !all_digits(decimals) || decimals.len() < min || decimals.len() > max {
        return None;
    }
    if !signed(exponent).is_some_and(|e| all_digits(e) && e.len() == 2) {
        return None;
    }
    exponent.parse().ok()
}

/// `(function) ([-+][0-9\.E\-+]+),(number)`.
fn ranged(
    inner: &str,
    function: impl Fn(&str) -> bool,
    min: usize,
    max: usize,
) -> Option<Reply<'_>> {
    let (name, rest) = inner.split_once(' ')?;
    if !function(name) {
        return None;
    }
    let (range, resolution) = rest.split_once(',')?;
    let digits = signed(range)?;
    if digits.is_empty()
        || !digits.bytes().all(|b| b.is_ascii_digit() || matches!(b, b'.' | b'E' | b'-' | b'+'))
    {
        return None;
    }
    Some(Reply::Conf(name, None, Some(number(resolution, min, max)?)))
}

fn upper(text: &str) -> bool {
    !text.is_empty() && text.bytes().all(|b| b.is_ascii_uppercase())
}

fn coupling_letters(text: &str) -> bool {
    !text.is_empty() && text.bytes().all(|b| matches!(b, b'A' | b'C' | b'D'))
}

/// `(PREFIX:[ACD]+)`.
fn coupled(name: &str, prefix: &str) -> bool {
    name.strip_prefix(prefix).and_then(|r| r.strip_prefix(':')).is_some_and(coupling_letters)
}

/// `CPER:[40]-20mA`.
fn percent_range(name: &str) -> bool {
    matches!(name, "CPER:4-20mA" | "CPER:0-20mA")
}

/// `(function) ([A-Z]+)`.
fn worded(inner: &str, function: impl Fn(&str) -> bool) -> Option<Reply<'_>> {
    let (name, word) = inner.split_once(' ')?;
    (function(name) && upper(word)).then_some(Reply::Conf(name, Some(word), None))
}

/// `TEMP:[A-Z]+`.
fn temp(name: &str) -> bool {
    name.strip_prefix("TEMP:").is_some_and(upper)
}

/// `T[0-9]:[A-Z]+`.
fn thermocouple(name: &str) -> bool {
    let b = name.as_bytes();
    b.len() > 3 && b[0] == b'T' && b[1].is_ascii_digit() && b[2] == b':' && upper(&name[3..])
}

fn bare(inner: &str, names: &[&str]) -> Option<Reply<'static>> {
    names.iter().find(|n| **n == inner).map(|n| {
        // A name of the table, so that the answer needs no borrow of the line.
        let n: &'static str = match *n {
            "DIOD" => "DIOD",
            "TEMP" => "TEMP",
            "SQU" => "SQU",
            "PULS:PDUT" => "PULS:PDUT",
            _ => "PULS:NDUT",
        };
        Reply::Conf(n, None, None)
    })
}

fn conf_u124x(inner: &str) -> Option<Reply<'_>> {
    let function = |n: &str| {
        matches!(n, "VOLT" | "CURR" | "RES" | "CAP" | "FREQ")
            || coupled(n, "VOLT")
            || coupled(n, "CURR")
            || percent_range(n)
    };
    ranged(inner, function, 6, 8)
        .or_else(|| worded(inner, |n| thermocouple(n) || temp(n)))
        .or_else(|| bare(inner, &["DIOD"]))
}

fn conf_u124xc(inner: &str) -> Option<Reply<'_>> {
    let function = |n: &str| {
        matches!(
            n,
            "VOLT"
                | "VOLT:AC"
                | "VOLT:HRAT"
                | "CURR"
                | "CURR:AC"
                | "RES"
                | "CONT"
                | "CAP"
                | "FREQ"
                | "FREQ:AC"
        ) || percent_range(n)
    };
    ranged(inner, function, 8, 8)
        .or_else(|| worded(inner, temp))
        .or_else(|| {
            let (name, word) = inner.split_once(' ')?;
            (name == "NCV" && matches!(word, "HI" | "LO")).then_some(Reply::Conf(
                name,
                Some(word),
                None,
            ))
        })
        .or_else(|| bare(inner, &["DIOD", "TEMP"]))
}

fn conf_u125x(inner: &str) -> Option<Reply<'_>> {
    let function = |n: &str| {
        matches!(n, "VOLT" | "CURR" | "RES" | "CONT" | "COND" | "CAP" | "FREQ" | "PULS:PWID")
            || coupled(n, "VOLT")
            || coupled(n, "CURR")
            || percent_range(n)
            || coupled(n, "PULS:PWID")
    };
    ranged(inner, function, 6, 6)
        .or_else(|| worded(inner, |n| thermocouple(n) || temp(n)))
        .or_else(|| bare(inner, &["DIOD", "PULS:PDUT", "PULS:NDUT"]))
}

fn conf_u128x(inner: &str) -> Option<Reply<'_>> {
    let function = |n: &str| {
        matches!(
            n,
            "VOLT"
                | "CURR"
                | "RES"
                | "CONT"
                | "COND"
                | "CAP"
                | "FREQ"
                | "FC1"
                | "FC100"
                | "PULS:PWID"
        ) || coupled(n, "VOLT")
            || coupled(n, "CURR")
            || coupled(n, "FREQ")
            || percent_range(n)
            || coupled(n, "PULS:PWID")
    };
    ranged(inner, function, 8, 8)
        .or_else(|| worded(inner, temp))
        .or_else(|| {
            let (name, word) = inner.split_once(' ')?;
            (name == "NCV" && matches!(word, "HIGH" | "LOW")).then_some(Reply::Conf(
                name,
                Some(word),
                None,
            ))
        })
        .or_else(|| bare(inner, &["DIOD", "SQU", "PULS:PDUT", "TEMP"]))
}

/// `"(V|MV|...),(\d),(AC|DC...)"`, `"(RES|CAP),(\d)"` and the words alone.
fn conf_fields(inner: &str, family: Family) -> Option<Reply<'_>> {
    let long = family == Family::U127x;
    let mut parts = inner.split(',');
    let name = parts.next()?;
    let Some(resolution) = parts.next() else {
        return (name == "DIOD" || (long && name == "TEMP"))
            .then_some(Reply::ConfFields(name, None, None));
    };
    if resolution.len() != 1 || !all_digits(resolution) {
        return None;
    }
    let resolution = resolution.parse().ok();
    let Some(coupling) = parts.next() else {
        return matches!(name, "RES" | "CAP").then_some(Reply::ConfFields(name, resolution, None));
    };
    let named = matches!(name, "V" | "MV" | "A" | "UA" | "FREQ") || (long && name == "MA");
    let couples = matches!(coupling, "AC" | "DC") || (long && coupling == "ACDC");
    (parts.next().is_none() && named && couples).then_some(Reply::ConfFields(
        name,
        resolution,
        Some(coupling),
    ))
}

/// Which line this is, as a family's table says; `None` is an unknown line.
pub fn parse(family: Family, line: &str) -> Option<Reply<'_>> {
    if let Some(s) = stat(line) {
        return Some(Reply::Stat(s.as_bytes()));
    }
    if switch(line, family == Family::U127x) {
        return Some(Reply::Switch);
    }
    if line.len() == 15 && number(line, 8, 8).is_some() {
        let captured = matches!(family, Family::U124xc | Family::U128x);
        return Some(Reply::Value(line, number(line, 8, 8).filter(|_| captured)));
    }
    let inner = quoted(line);
    let conf = inner.and_then(|inner| match family {
        Family::U123x | Family::U127x => conf_fields(inner, family),
        Family::U124x => conf_u124x(inner),
        Family::U124xc => conf_u124xc(inner),
        Family::U125x => conf_u125x(inner),
        Family::U128x => conf_u128x(inner),
    });
    if conf.is_some() {
        return conf;
    }
    if matches!(family, Family::U124xc | Family::U128x) {
        if let Some(entry) = inner.filter(|e| e.len() == 14 && all_digits(e)) {
            return Some(Reply::LogEntry(entry));
        }
        if line == "*E" {
            return Some(Reply::Error);
        }
    }
    None
}

/// The answers a job can end on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Job {
    /// `FETC?`.
    Fetc,
    /// `CONF?`.
    Conf,
    /// `STAT?`.
    Stat,
    /// `LOG:`.
    Log,
}

/// What an answer says about the job.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Outcome {
    /// That kind of job is over.
    Done(Job),
    /// Run it again at once, for another channel.
    Again,
    /// The capture is over.
    Stop,
    /// Nothing.
    Nothing,
}

/// A job and how long after its start it may run again.
#[derive(Clone, Copy, Debug)]
struct JobSpec {
    job: Job,
    /// `None` is the sample rate's interval.
    interval: Option<Duration>,
}

/// Poll `CONF?` and `STAT?` at 1 Hz, and values at the sample rate.
const JOBS_LIVE: [JobSpec; 3] = [
    JobSpec { job: Job::Fetc, interval: None },
    JobSpec { job: Job::Conf, interval: Some(Duration::from_secs(1)) },
    JobSpec { job: Job::Stat, interval: Some(Duration::from_secs(1)) },
];
/// Poll `LOG:` as fast as the meter answers.
const JOBS_LOG: [JobSpec; 1] = [JobSpec { job: Job::Log, interval: Some(Duration::ZERO) }];

/// A sample for the session.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// The channel, by index.
    pub channel: usize,
    /// The value, NaN on an overload.
    pub value: f32,
    /// What it measures.
    pub mq: Mq,
    /// Its unit.
    pub unit: Unit,
    /// Its flags.
    pub flags: MqFlags,
    /// C's `encoding.digits`, which is what the `analog` output shows.
    pub digits: i32,
}

/// What a line came to.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Received {
    /// The sample it gave.
    pub reading: Option<Reading>,
    /// The capture is over: `*E` on a log.
    pub stop: bool,
}

/// `flags` without what `drop` holds.
fn without(flags: MqFlags, drop: MqFlags) -> MqFlags {
    flags.iter().filter(|f| !drop.contains(*f)).fold(MqFlags::empty(), |all, f| all | f)
}

/// `flag` in `flags`, or not, as `on` says.
fn with(flags: MqFlags, flag: MqFlags, on: bool) -> MqFlags {
    if on { flags | flag } else { without(flags, flag) }
}

/// The driver's state in a capture: C's `dev_context`, which a port rebuilds at
/// each start, as what it would carry over from one capture to the next (the
/// modes, for the first second) has no bearing on the CLI's one capture.
#[derive(Debug)]
#[allow(clippy::struct_excessive_bools)]
pub struct Meter {
    profile: &'static Profile,
    samplerate: u64,
    /// 0 is live, else a place in [`DATA_SOURCES`].
    data_source: usize,
    enabled: Vec<usize>,
    jobs: &'static [JobSpec],
    current: usize,
    running: bool,
    again: bool,
    starts: [Option<Instant>; 3],
    channel: usize,
    conf: usize,
    sample: i32,
    mq: [Option<Mq>; 3],
    unit: [Unit; 3],
    flags: [MqFlags; 3],
    digits: [i32; 3],
    encoding: [i32; 3],
    exponent: [i32; 3],
    temp_aux: bool,
    continuity: bool,
    square_wave: bool,
    db: Option<Unit>,
}

impl Meter {
    /// A capture on `enabled`, channel indices in order, at least one.
    pub fn new(
        profile: &'static Profile,
        enabled: Vec<usize>,
        data_source: usize,
        samplerate: u64,
    ) -> Meter {
        let mut mq = [None; 3];
        let mut unit = [Unit::Volt; 3];
        let mut digits = [0; 3];
        let mut encoding = [0; 3];
        // Scan: the last channel of a meter with more than one is the temperature.
        if profile.channels > 1 {
            let t = profile.channels - 1;
            mq[t] = Some(Mq::Temperature);
            unit[t] = Unit::Celsius;
            digits[t] = 1;
            encoding[t] = 2;
        }
        // Start: nothing known of these until their CONF. A U124x's second
        // channel stays a temperature until then, as it is not in this.
        mq[0] = None;
        if profile.channels > 2 {
            mq[1] = None;
        }
        let first = enabled.first().copied().unwrap_or(0);
        Meter {
            profile,
            samplerate: samplerate.max(1),
            data_source,
            enabled,
            jobs: if data_source == 0 { &JOBS_LIVE } else { &JOBS_LOG },
            current: 0,
            running: false,
            again: false,
            starts: [None; 3],
            channel: first,
            conf: first,
            sample: 1,
            mq,
            unit,
            flags: [MqFlags::empty(); 3],
            digits,
            encoding,
            exponent: [0; 3],
            temp_aux: false,
            continuity: false,
            square_wave: false,
            db: None,
        }
    }

    /// `sr_next_enabled_channel`: round to the next.
    fn next_enabled(&self, channel: usize) -> usize {
        self.enabled
            .iter()
            .copied()
            .find(|c| *c > channel)
            .or_else(|| self.enabled.first().copied())
            .unwrap_or(channel)
    }

    /// `MIN(nb_channels, 2)`: from here on a channel has no `CONF?`.
    fn first_without_conf(&self) -> usize {
        self.profile.channels.min(2)
    }

    fn interval(&self, spec: JobSpec) -> Duration {
        spec.interval.unwrap_or_else(|| Duration::from_millis(1000 / self.samplerate))
    }

    /// `job_in_interval`. A job that never ran is due.
    fn in_interval(&self, now: Instant) -> bool {
        let interval = self.interval(self.jobs[self.current]);
        self.starts[self.current].is_some_and(|start| now.duration_since(start) < interval)
    }

    fn next_job(&mut self, now: Instant) {
        let was = self.current;
        loop {
            self.current = (self.current + 1) % self.jobs.len();
            if !(self.in_interval(now) && self.current != was) {
                break;
            }
        }
    }

    /// `job_run_again`: the query, if the job has one for now.
    fn run_again(&mut self) -> Option<Vec<u8>> {
        self.again = false;
        self.running = true;
        let command = self.command();
        self.running = command.is_some();
        command
    }

    /// `dispatch`: the next query to write.
    pub fn dispatch(&mut self, now: Instant) -> Option<Vec<u8>> {
        if self.again {
            return self.run_again();
        }
        if !self.running {
            self.next_job(now);
        } else if self.starts[self.current]
            .is_none_or(|start| now.duration_since(start) > JOB_TIMEOUT)
        {
            self.running = false;
        }
        if !self.running && !self.in_interval(now) {
            self.starts[self.current] = Some(now);
            return self.run_again();
        }
        None
    }

    /// The job's query, as `send_fetc`, `send_conf`, `send_stat` and `send_log`.
    fn command(&self) -> Option<Vec<u8>> {
        let text = match self.jobs[self.current].job {
            Job::Fetc if self.square_wave => return None,
            Job::Fetc if self.channel > 0 => format!("FETC? @{}", self.channel + 1),
            Job::Fetc => "FETC?".to_owned(),
            Job::Conf if self.conf >= self.first_without_conf() => return None,
            Job::Conf if self.conf > 0 => format!("CONF? @{}", self.conf + 1),
            Job::Conf => "CONF?".to_owned(),
            Job::Stat => "STAT?".to_owned(),
            Job::Log => {
                format!("{} {}", LOG_COMMANDS[self.data_source - 1], self.sample)
            }
        };
        let mut bytes = text.into_bytes();
        bytes.extend_from_slice(b"\n\r\n");
        Some(bytes)
    }

    /// `receive_line`: the line up to its LF.
    pub fn receive(&mut self, line: &[u8]) -> Received {
        let end = line.iter().rposition(|b| !matches!(b, b'\r' | b'\n')).map_or(0, |at| at + 1);
        let line = until_nul(&line[..end]);
        let reply = std::str::from_utf8(line).ok().and_then(|l| parse(self.profile.family, l));
        let Some(reply) = reply else {
            log::debug!("Unknown line '{}'.", line.escape_ascii());
            return Received::default();
        };
        let mut received = Received::default();
        let outcome = self.handle(reply, &mut received);
        match outcome {
            Outcome::Done(job) if job == self.jobs[self.current].job => self.running = false,
            Outcome::Again => self.again = true,
            Outcome::Stop => received.stop = true,
            Outcome::Done(_) | Outcome::Nothing => {}
        }
        received
    }

    fn handle(&mut self, reply: Reply<'_>, received: &mut Received) -> Outcome {
        match reply {
            Reply::Stat(s) => {
                self.stat(s);
                Outcome::Done(Job::Stat)
            }
            Reply::Switch => {
                self.switch();
                Outcome::Nothing
            }
            Reply::Value(text, exponent) => {
                received.reading = self.value(text, exponent);
                self.after_value()
            }
            Reply::ConfFields(name, resolution, coupling) => {
                self.conf_fields(name, resolution.unwrap_or(0), coupling);
                self.after_conf()
            }
            Reply::Conf(name, word, resolution) => {
                self.conf_scpi(name, word, resolution);
                self.after_conf()
            }
            Reply::LogEntry(entry) => match self.log_entry(entry) {
                Some(reading) => {
                    received.reading = Some(reading);
                    Outcome::Done(Job::Log)
                }
                None => Outcome::Nothing,
            },
            Reply::Error if self.data_source != 0 => Outcome::Stop,
            Reply::Error => Outcome::Again,
        }
    }

    /// `recv_switch`: the jobs over from the start, and nothing known.
    fn switch(&mut self) {
        self.current = 0;
        self.running = false;
        self.starts = [None; 3];
        self.mq[0] = None;
        if self.profile.channels > 2 {
            self.mq[1] = None;
        }
    }

    /// `recv_stat_*`, each byte a position in the letters.
    fn stat(&mut self, s: &[u8]) {
        let f = &mut self.flags[0];
        let min_max = MqFlags::MAX | MqFlags::MIN;
        let min_max_avg = min_max | MqFlags::AVG;
        match self.profile.family {
            Family::U123x | Family::U127x => {
                *f = with(*f, min_max, s[0] == b'1');
                *f = with(*f, MqFlags::RELATIVE, s[1] == b'1');
                *f = with(*f, MqFlags::HOLD, s[2] == b'1' || s[3] == b'1');
                self.temp_aux = s[7] == b'1';
                self.continuity = s[16] == b'1';
            }
            Family::U124x => {
                *f = with(*f, min_max, s[0] == b'1');
                *f = with(*f, MqFlags::RELATIVE, s[1] == b'1');
                *f = with(*f, MqFlags::HOLD, s[7] == b'1');
            }
            Family::U124xc => {
                *f = with(*f, min_max_avg, s[0] == b'1');
                *f = with(*f, MqFlags::RELATIVE, s[1] == b'1');
                *f = with(*f, MqFlags::HOLD, s[7] == b'1' || s[11] == b'1');
            }
            Family::U125x => {
                *f = with(*f, MqFlags::MAX, s[4] == b'1');
                *f = with(*f, MqFlags::HOLD, s[7] == b'1');
                self.stat_db(s[2]);
            }
            Family::U128x => {
                *f = with(*f, min_max_avg, s[0] == b'1');
                // The guide says 1 for peak hold on, and C looks for a 4
                // (protocol.c:407). No recording says which.
                *f = with(*f, MqFlags::MAX, s[4] == b'4');
                *f = with(*f, MqFlags::RELATIVE, s[1] == b'1');
                *f = with(*f, MqFlags::HOLD, s[7] == b'1' || s[11] == b'1');
                self.stat_db(s[2]);
            }
        }
    }

    fn stat_db(&mut self, letter: u8) {
        self.db = match letter & !0x20 {
            b'M' => Some(Unit::DecibelMw),
            b'V' => Some(Unit::DecibelVolt),
            _ => None,
        };
        if let Some(unit) = self.db {
            self.unit[0] = unit;
        }
    }

    /// `recv_fetc` up to the sample.
    fn value(&mut self, text: &str, exponent: Option<i32>) -> Option<Reading> {
        let i = self.channel;
        // A second display's TEMP copies the third, so that goes.
        let mq = self.mq[i]?;
        let value = if text == "-9.90000000E+37" || text == "+9.90000000E+37" {
            f32::NAN
        } else {
            // The form is checked, so this cannot fail; C goes through a double.
            #[allow(clippy::cast_possible_truncation)]
            let parsed = text.parse::<f64>().unwrap_or(f64::NAN) as f32;
            if self.exponent[i] == 0 {
                parsed
            } else {
                #[allow(clippy::cast_precision_loss)]
                let scale = 10f32.powf(self.exponent[i] as f32);
                parsed * scale
            }
        };
        if matches!(self.unit[i], Unit::DecibelMw | Unit::DecibelVolt | Unit::Percentage)
            && let Some(exponent) = exponent
        {
            self.digits[i] = self.digits[i].min(4 - exponent);
            self.encoding[i] = self.encoding[i].min(5 - exponent);
        }
        Some(Reading {
            channel: i,
            value,
            mq,
            unit: self.unit[i],
            flags: self.flags[i],
            digits: self.encoding[i] - self.exponent[i],
        })
    }

    fn after_value(&mut self) -> Outcome {
        let was = self.channel;
        self.channel = self.next_enabled(was);
        if self.channel > was { Outcome::Again } else { Outcome::Done(Job::Fetc) }
    }

    fn after_conf(&mut self) -> Outcome {
        let was = self.conf;
        self.conf = self.next_enabled(was);
        if self.conf >= self.first_without_conf() {
            self.conf = self.next_enabled(self.conf);
        }
        if self.conf > was { Outcome::Again } else { Outcome::Done(Job::Conf) }
    }

    fn set_measure(&mut self, i: usize, mq: Mq, unit: Unit, flags: MqFlags, exponent: i32) {
        self.mq[i] = Some(mq);
        self.unit[i] = unit;
        self.flags[i] = flags;
        self.exponent[i] = exponent;
    }

    /// `recv_conf_u123x`.
    fn conf_fields(&mut self, name: &str, resolution: i32, coupling: Option<&str>) {
        let i = self.conf;
        let none = MqFlags::empty();
        let digits = match name {
            "V" => {
                self.set_measure(i, Mq::Voltage, Unit::Volt, none, 0);
                4 - resolution
            }
            "MV" if self.temp_aux => {
                // No way to tell Fahrenheit, C says.
                self.set_measure(i, Mq::Temperature, Unit::Celsius, none, 0);
                1
            }
            "MV" => {
                self.set_measure(i, Mq::Voltage, Unit::Volt, none, -3);
                5 - resolution
            }
            "A" => {
                self.set_measure(i, Mq::Current, Unit::Ampere, none, 0);
                3 - resolution
            }
            "MA" => {
                self.set_measure(i, Mq::Current, Unit::Ampere, none, -3);
                8 - resolution
            }
            "UA" => {
                self.set_measure(i, Mq::Current, Unit::Ampere, none, -6);
                8 - resolution
            }
            "FREQ" => {
                self.set_measure(i, Mq::Frequency, Unit::Hertz, none, 0);
                2 - resolution
            }
            "RES" => {
                if self.continuity {
                    self.set_measure(i, Mq::Continuity, Unit::Boolean, none, 0);
                } else {
                    self.set_measure(i, Mq::Resistance, Unit::Ohm, none, 0);
                }
                1 - resolution
            }
            "DIOD" => {
                self.set_measure(i, Mq::Voltage, Unit::Volt, MqFlags::DIODE | MqFlags::DC, 0);
                3
            }
            "TEMP" => {
                self.set_measure(i, Mq::Temperature, Unit::Celsius, none, 0);
                1
            }
            // CAP: the only other name the tables let through.
            _ => {
                self.set_measure(i, Mq::Capacitance, Unit::Farad, none, 0);
                9 - resolution
            }
        };
        self.digits[i] = digits;
        // "This is based on guess", C says.
        self.encoding[i] = digits + 1;
        match coupling {
            Some("AC") => {
                self.flags[i] |= MqFlags::AC;
                if self.mq[i] == Some(Mq::Voltage) {
                    self.flags[i] |= MqFlags::RMS;
                }
            }
            Some("DC") => self.flags[i] |= MqFlags::DC,
            Some(_) => self.flags[i] |= MqFlags::AC | MqFlags::DC | MqFlags::RMS,
            None => self.flags[i] = without(self.flags[i], MqFlags::AC | MqFlags::DC),
        }
    }

    fn digits_encoding(&mut self, i: usize, digits: i32) {
        self.digits[i] = digits;
        self.encoding[i] = digits + 1;
    }

    /// The flags after `VOLT` or `CURR`, `strncmp` by `strncmp`.
    fn coupling(&mut self, i: usize, name: &str) {
        let Some(after) = name.get(4..).and_then(|r| r.strip_prefix(':')) else {
            self.flags[i] |= MqFlags::DC;
            return;
        };
        if after.starts_with("ACDC") {
            self.flags[i] |= MqFlags::AC | MqFlags::DC | MqFlags::RMS;
        } else if after.starts_with("AC") {
            self.flags[i] |= MqFlags::AC | MqFlags::RMS;
        } else if after.starts_with("DC") {
            self.flags[i] |= MqFlags::DC;
        } else if after.starts_with("HRAT") {
            self.mq[i] = Some(Mq::HarmonicRatio);
            self.unit[i] = Unit::Percentage;
            self.digits_encoding(i, 2);
        }
    }

    /// `recv_conf_u124x_5x`.
    fn conf_scpi(&mut self, name: &str, word: Option<&str>, resolution: Option<i32>) {
        let i = self.conf;
        let none = MqFlags::empty();
        self.square_wave = false;
        if let Some(resolution) = resolution {
            self.digits[i] = -resolution;
            self.encoding[i] = 1 - resolution;
        }
        let u128x = self.profile.family == Family::U128x;
        if name.starts_with("VOLT") {
            self.set_measure(i, Mq::Voltage, Unit::Volt, none, 0);
            if let Some(db) = self.db.filter(|_| i == 0) {
                self.unit[i] = db;
                self.digits_encoding(i, 3);
            }
            self.coupling(i, name);
        } else if name.starts_with("CURR") {
            self.set_measure(i, Mq::Current, Unit::Ampere, none, 0);
            // CURR has no HRAT in any table, so the shared code is C's too.
            self.coupling(i, name);
        } else if name == "RES" {
            self.set_measure(i, Mq::Resistance, Unit::Ohm, none, 0);
        } else if name == "COND" {
            self.set_measure(i, Mq::Conductance, Unit::Siemens, none, 0);
        } else if name == "CAP" {
            self.set_measure(i, Mq::Capacitance, Unit::Farad, none, 0);
        } else if name.starts_with("FREQ") || name.starts_with("FC1") {
            self.set_measure(i, Mq::Frequency, Unit::Hertz, none, 0);
        } else if name.starts_with("PULS:PWID") {
            self.set_measure(i, Mq::PulseWidth, Unit::Second, none, 0);
            self.encoding[i] = self.encoding[i].min(6);
        } else if name.starts_with("PULS:PDUT") {
            self.set_measure(i, Mq::DutyCycle, Unit::Percentage, none, 0);
            self.digits_encoding(i, 3);
        } else if name == "CONT" {
            self.set_measure(i, Mq::Continuity, Unit::Ohm, none, 0);
        } else if name == "DIOD" {
            self.set_measure(i, Mq::Voltage, Unit::Volt, MqFlags::DIODE | MqFlags::DC, 0);
            self.digits_encoding(i, if u128x { 4 } else { 3 });
        } else if name.starts_with("T1") || name.starts_with("T2") || name.starts_with("TEMP") {
            self.mq[i] = Some(Mq::Temperature);
            if word.is_none() && self.profile.channels == 3 {
                // TEMP with no word is the second display's copy of P3.
                self.mq[i] = None;
            } else if word == Some("FAR") {
                self.unit[i] = Unit::Fahrenheit;
            } else {
                self.unit[i] = Unit::Celsius;
            }
            self.flags[i] = none;
            self.exponent[i] = 0;
            self.digits_encoding(i, 1);
        } else if name.starts_with("CPER:") {
            self.set_measure(i, Mq::Current, Unit::Percentage, none, 0);
            self.digits_encoding(i, 2);
        } else if name == "SQU" {
            // Not supported, and FETC? would only give an error.
            self.square_wave = true;
        } else if name == "NCV" {
            let flags = MqFlags::AC;
            if u128x {
                self.set_measure(i, Mq::Voltage, Unit::Volt, flags, -3);
                self.digits[i] = -1;
                self.encoding[i] = 0;
            } else {
                self.set_measure(i, Mq::Voltage, Unit::Volt, flags, 0);
                self.digits_encoding(i, 2);
            }
        } else {
            log::debug!("Unknown first argument '{name}'.");
        }
    }

    /// `recv_log`, and what the two tables of functions give it.
    fn log_entry(&mut self, entry: &str) -> Option<Reading> {
        let b = entry.as_bytes();
        let function: usize = entry[..2].parse().ok()?;
        let (mq, unit, exponent) = log_function(self.profile.family, function).or_else(|| {
            log::debug!("Invalid function.");
            None
        })?;
        let value: i32 = entry[2..7].parse().ok()?;
        let negative = if b[7] & 2 != 0 { -1 } else { 1 };
        let overload = b[8] & 4 != 0;
        let mut exponent = i32::from(b[9] & 0xf) + exponent;
        let alternate = b[10] & 1 != 0;
        let mut flags = MqFlags::empty();
        for (on, flag) in [
            (b[8] & 1 != 0, MqFlags::DC),
            (b[8] & 2 != 0, MqFlags::AC),
            (b[11] & 4 != 0, MqFlags::RELATIVE),
            (b[12] & 1 != 0, MqFlags::AVG),
            (b[12] & 2 != 0, MqFlags::MIN),
            (b[12] & 4 != 0, MqFlags::MAX),
            (function == 5, MqFlags::DIODE | MqFlags::DC),
        ] {
            if on {
                flags |= flag;
            }
        }
        let (mut mq, mut unit) = (mq, unit);
        if alternate {
            if mq == Mq::Resistance {
                mq = Mq::Continuity;
            }
            if unit == Unit::DecibelMw {
                unit = Unit::DecibelVolt;
            }
            if unit == Unit::Celsius {
                unit = Unit::Fahrenheit;
                if self.profile.family == Family::U128x {
                    exponent -= 1;
                }
            }
        }
        let value = if overload {
            f32::NAN
        } else {
            #[allow(clippy::cast_precision_loss)]
            let (whole, power) = ((negative * value) as f32, exponent as f32);
            whole * 10f32.powf(power)
        };
        self.sample = self.sample.wrapping_add(1);
        Some(Reading { channel: self.channel, value, mq, unit, flags, digits: -exponent })
    }
}

/// `recv_log_u124xc`'s and `recv_log_u128x`'s tables.
fn log_function(family: Family, function: usize) -> Option<(Mq, Unit, i32)> {
    use Mq as M;
    use Unit as U;
    const U124XC: [(Mq, Unit, i32); 11] = [
        (M::Voltage, U::Volt, -5),
        (M::Voltage, U::Volt, -4),
        (M::Current, U::Ampere, -7),
        (M::Current, U::Ampere, -3),
        (M::Resistance, U::Ohm, -2),
        (M::Voltage, U::Volt, -3),
        (M::Temperature, U::Celsius, -1),
        (M::Capacitance, U::Farad, -10),
        (M::Frequency, U::Hertz, -2),
        (M::HarmonicRatio, U::Percentage, -2),
        (M::Current, U::Percentage, -2),
    ];
    const U128X: [(Mq, Unit, i32); 14] = [
        (M::Voltage, U::Volt, -6),
        (M::Voltage, U::Volt, -4),
        (M::Current, U::Ampere, -9),
        (M::Current, U::Ampere, -4),
        (M::Resistance, U::Ohm, -3),
        (M::Voltage, U::Volt, -4),
        (M::Temperature, U::Celsius, -1),
        (M::Capacitance, U::Farad, -12),
        (M::Frequency, U::Hertz, -3),
        (M::DutyCycle, U::Percentage, -3),
        (M::PulseWidth, U::Second, -6),
        (M::Voltage, U::DecibelMw, -3),
        (M::Current, U::Percentage, -2),
        (M::Conductance, U::Siemens, -11),
    ];
    let table: &[(Mq, Unit, i32)] = match family {
        Family::U124xc => &U124XC,
        Family::U128x => &U128X,
        _ => return None,
    };
    table.get(function).copied()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile(name: &str) -> &'static Profile {
        PROFILES.iter().find(|p| p.name == name).unwrap()
    }

    fn live(name: &str) -> Meter {
        let p = profile(name);
        Meter::new(p, (0..p.channels).collect(), 0, 5)
    }

    fn one(name: &str) -> Meter {
        Meter::new(profile(name), vec![0], 0, 5)
    }

    fn text(command: Option<Vec<u8>>) -> Option<String> {
        command.map(|c| String::from_utf8(c).unwrap())
    }

    fn reading(m: &mut Meter, line: &str) -> Reading {
        m.receive(line.as_bytes()).reading.unwrap()
    }

    #[test]
    fn the_table_has_the_series() {
        assert_eq!(PROFILES.len(), 21);
        assert_eq!(PROFILES.iter().filter(|p| p.channels == 1).count(), 3);
        assert_eq!(PROFILES.iter().filter(|p| p.channels == 2).count(), 6);
        assert_eq!(PROFILES.iter().filter(|p| p.has_log()).count(), 4);
    }

    #[test]
    fn an_identity_takes_four_fields() {
        let i = parse_identity(b"Agilent Technologies,U1253B,x,V2.26").unwrap();
        assert_eq!((i.vendor, i.profile.name, i.version.as_str()), ("Agilent", "U1253B", "V2.26"));
        let i = parse_identity(b"Keysight Technologies,U1253B,x,a,b,c").unwrap();
        assert_eq!((i.vendor, i.version.as_str()), ("Keysight", "a,b,c"));
        let i = parse_identity(b"Keysight Technologies,U1282A,,").unwrap();
        assert_eq!(i.version, "");
        let i = parse_identity(b"Agilent Technologies,U1253B,x,V1\0more").unwrap();
        assert_eq!(i.version, "V1");
    }

    #[test]
    fn what_is_no_identity() {
        for bad in [
            &b""[..],
            b"\0",
            b"\0\r",
            b"Agilent Technologies",
            b"Agilent Technologies,U1253B",
            b"Agilent Technologies,U1253B,x",
            b"Agilent Technologies,U1253B\0,x,V1",
            b"agilent technologies,U1253B,x,V1",
            b"HP,U1253B,x,V1",
            b"Agilent Technologies,U1253,x,V1",
            b"Agilent Technologies,U1253BX,x,V1",
        ] {
            assert_eq!(parse_identity(bad), None, "{}", bad.escape_ascii());
        }
    }

    #[test]
    fn each_family_knows_its_lines() {
        use Family::{U123x, U124x, U124xc, U125x, U127x, U128x};
        let all = [U123x, U124x, U124xc, U125x, U127x, U128x];
        let cases: [(&str, &[Family]); 37] = [
            ("\"000000000110L00000000\"", &all),
            ("\"00000000011000000000\"", &[]),
            ("\"0000000001100000000000\"", &[]),
            ("\"00000000011000000000x\"", &[]),
            ("\"x0000000011000000000\"", &[]),
            ("*3", &all),
            ("*10", &[U127x]),
            ("*E", &[U124xc, U128x]),
            ("*B", &[]),
            ("+1.23475000E+00", &all),
            ("-9.90000000E+37", &all),
            ("+1.2347500E+00", &[]),
            ("1.23475000E+00", &[]),
            ("+1.23475000e+00", &[]),
            ("\"V,0,AC\"", &[U123x, U127x]),
            ("\"V,0,ACDC\"", &[U127x]),
            ("\"MA,1,DC\"", &[U127x]),
            ("\"UA,10,DC\"", &[]),
            ("\"RES,1\"", &[U123x, U127x]),
            ("\"V,1\"", &[]),
            ("\"DIOD\"", &all),
            ("\"TEMP\"", &[U124xc, U127x, U128x]),
            ("\"SQU\"", &[U128x]),
            ("\"PULS:NDUT\"", &[U125x]),
            ("\"PULS:PDUT\"", &[U125x, U128x]),
            ("\"VOLT +1.000000E+00,+1.000000E-04\"", &[U124x, U125x]),
            ("\"VOLT +1.00000000E+00,+1.00000000E-04\"", &[U124x, U124xc, U128x]),
            ("\"VOLT +1.0000000E+00,+1.0000000E-04\"", &[U124x]),
            ("\"VOLT:HRAT +1.00000000E+00,+1.00000000E-04\"", &[U124xc]),
            ("\"VOLT:CD +1.000000E+00,+1.000000E-04\"", &[U124x, U125x]),
            ("\"FREQ:AC +1.00000000E+00,+1.00000000E-04\"", &[U124xc, U128x]),
            ("\"FC100 +1.00000000E+00,+1.00000000E-04\"", &[U128x]),
            ("\"T1:K CEL\"", &[U124x, U125x]),
            ("\"TEMP:K CEL\"", &[U124x, U124xc, U125x, U128x]),
            ("\"NCV HI\"", &[U124xc]),
            ("\"NCV LOW\"", &[U128x]),
            ("\"00123450000000\"", &[U124xc, U128x]),
        ];
        for (line, families) in cases {
            for f in all {
                assert_eq!(parse(f, line).is_some(), families.contains(&f), "{f:?} {line}");
            }
        }
    }

    #[test]
    fn a_range_and_a_resolution_are_checked() {
        for bad in [
            "\"VOLT 1.000000E+00,+1.000000E-04\"",
            "\"VOLT +,+1.000000E-04\"",
            "\"VOLT +x,+1.000000E-04\"",
            "\"VOLT +1.0,1.000000E-04\"",
            "\"VOLT +1.0,+1.000000E-4\"",
            "\"VOLT +1.0,+1.000000E-04,\"",
            "\"VOLT +1.0\"",
            "\"VOLT  +1.0,+1.000000E-04\"",
        ] {
            assert_eq!(parse(Family::U125x, bad), None, "{bad}");
        }
        assert_eq!(
            parse(Family::U125x, "\"VOLT +1.0E+.-,+1.000000E-04\""),
            Some(Reply::Conf("VOLT", None, Some(-4)))
        );
    }

    #[test]
    fn a_value_has_its_exponent_only_where_the_table_captures_it() {
        assert_eq!(parse(Family::U128x, "+1.0000000E+02"), None, "seven decimals");
        assert_eq!(
            parse(Family::U128x, "+1.00000000E+02"),
            Some(Reply::Value("+1.00000000E+02", Some(2)))
        );
        assert_eq!(
            parse(Family::U125x, "+1.00000000E+02"),
            Some(Reply::Value("+1.00000000E+02", None))
        );
    }

    /// Jobs: CONF, STAT, then FETC, each once an answer lets it, the first
    /// round with nothing in an interval.
    #[test]
    fn the_first_round_and_the_pace() {
        let t = Instant::now();
        let at = |ms| t + Duration::from_millis(ms);
        let mut m = one("U1232A");
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("CONF?\n\r\n"));
        assert_eq!(m.dispatch(at(1)), None, "waits for the answer");
        m.receive(b"\"V,0,DC\"");
        assert_eq!(text(m.dispatch(at(2))).as_deref(), Some("STAT?\n\r\n"));
        m.receive(b"\"000000000000000000000\"");
        assert_eq!(text(m.dispatch(at(3))).as_deref(), Some("FETC?\n\r\n"));
        assert!(m.receive(b"+1.23475000E+00").reading.is_some());
        // 200 ms from this FETC?, not sooner.
        assert_eq!(m.dispatch(at(202)), None);
        assert_eq!(text(m.dispatch(at(203))).as_deref(), Some("FETC?\n\r\n"));
        m.receive(b"+1.23475000E+00");
        assert_eq!(m.dispatch(at(402)), None);
        for ms in [403, 603, 803] {
            assert_eq!(text(m.dispatch(at(ms))).as_deref(), Some("FETC?\n\r\n"));
            m.receive(b"+1.23475000E+00");
        }
        assert_eq!(m.dispatch(at(999)), None);
        // CONF? and STAT? a second from their last, ahead of a FETC? due.
        assert_eq!(text(m.dispatch(at(1000))).as_deref(), Some("CONF?\n\r\n"));
        m.receive(b"\"V,0,DC\"");
        assert_eq!(text(m.dispatch(at(1001))), None, "STAT? has a second from 2 ms");
        assert_eq!(text(m.dispatch(at(1002))).as_deref(), Some("STAT?\n\r\n"));
    }

    #[test]
    fn a_rate_sets_fetcs_interval() {
        let t = Instant::now();
        let at = |ms| t + Duration::from_millis(ms);
        for (rate, ms) in [(1, 1000), (3, 333), (20, 50)] {
            let mut m = Meter::new(profile("U1232A"), vec![0], 0, rate);
            m.dispatch(t);
            m.receive(b"\"V,0,DC\"");
            m.dispatch(t);
            m.receive(b"\"000000000000000000000\"");
            assert_eq!(text(m.dispatch(t)).as_deref(), Some("FETC?\n\r\n"));
            m.receive(b"+1.23475000E+00");
            // Not the jobs of a second, to see only this one: rates under 1 Hz
            // have none, and at 1 Hz CONF? goes first.
            assert_eq!(m.dispatch(at(ms - 1)), None, "{rate}");
            assert!(m.dispatch(at(ms)).is_some(), "{rate}");
        }
    }

    #[test]
    fn a_job_is_given_up_after_its_time() {
        let t = Instant::now();
        let at = |ms| t + Duration::from_millis(ms);
        let mut m = one("U1232A");
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("CONF?\n\r\n"));
        assert_eq!(m.dispatch(at(300)), None, "exactly its time is not over");
        assert!(m.running);
        assert_eq!(m.dispatch(at(301)), None, "given up, and it is in its interval");
        assert!(!m.running);
        assert_eq!(text(m.dispatch(at(302))).as_deref(), Some("STAT?\n\r\n"));
    }

    #[test]
    fn a_job_given_up_lets_the_next_one_run() {
        let t = Instant::now();
        let at = |ms| t + Duration::from_millis(ms);
        let mut m = one("U1232A");
        m.dispatch(t);
        m.receive(b"\"V,0,DC\"");
        assert_eq!(text(m.dispatch(at(1))).as_deref(), Some("STAT?\n\r\n"));
        // A given up STAT?: the FETC? that never ran is next, a pass later.
        assert_eq!(m.dispatch(at(302)), None);
        assert_eq!(text(m.dispatch(at(303))).as_deref(), Some("FETC?\n\r\n"));
    }

    #[test]
    fn channels_go_in_turn_and_the_third_has_no_conf() {
        let t = Instant::now();
        let mut m = live("U1253B");
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("CONF?\n\r\n"));
        m.receive(b"\"VOLT:AC +1.000000E+00,+1.000000E-04\"");
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("CONF? @2\n\r\n"));
        m.receive(b"\"FREQ +1.000000E+03,+1.000000E-01\"");
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("STAT?\n\r\n"));
        m.receive(b"\"000000000000000000000\"");
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("FETC?\n\r\n"));
        assert_eq!(reading(&mut m, "+1.00000000E+00").channel, 0);
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("FETC? @2\n\r\n"));
        assert_eq!(reading(&mut m, "+1.00000000E+00").channel, 1);
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("FETC? @3\n\r\n"));
        let r = reading(&mut m, "+2.25000000E+01");
        assert_eq!((r.channel, r.mq, r.unit, r.digits), (2, Mq::Temperature, Unit::Celsius, 2));
        assert_eq!(m.dispatch(t), None, "the turn is over");
    }

    #[test]
    fn only_the_third_channel_needs_no_conf_at_all() {
        let t = Instant::now();
        let mut m = Meter::new(profile("U1253B"), vec![2], 0, 5);
        assert_eq!(m.dispatch(t), None, "no CONF? for it");
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("STAT?\n\r\n"));
    }

    #[test]
    fn a_two_display_meter_asks_conf_of_its_temperature() {
        let t = Instant::now();
        let mut m = live("U1242B");
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("CONF?\n\r\n"));
        m.receive(b"\"DIOD\"");
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("CONF? @2\n\r\n"));
    }

    #[test]
    fn a_second_display_known_as_a_copy_is_passed_over() {
        let t = Instant::now();
        let mut m = live("U1282A");
        m.dispatch(t);
        m.receive(b"\"DIOD\"");
        m.dispatch(t);
        m.receive(b"\"TEMP\"");
        m.dispatch(t);
        m.receive(b"\"000000000000000000000\"");
        m.dispatch(t);
        assert!(m.receive(b"+1.00000000E+00").reading.is_some());
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("FETC? @2\n\r\n"));
        assert_eq!(m.receive(b"+1.00000000E+00").reading, None);
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("FETC? @3\n\r\n"));
    }

    #[test]
    fn a_switch_starts_over_and_forgets() {
        let t = Instant::now();
        let mut m = one("U1232A");
        m.dispatch(t);
        m.receive(b"\"V,0,DC\"");
        m.dispatch(t);
        m.receive(b"\"000000000000000000000\"");
        m.dispatch(t);
        m.receive(b"*3");
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("CONF?\n\r\n"), "at once");
        assert_eq!(m.receive(b"+1.00000000E+00").reading, None, "no quantity known");
    }

    #[test]
    fn a_line_is_cut_at_its_crs_and_a_nul() {
        let t = Instant::now();
        let mut m = one("U1232A");
        m.dispatch(t);
        m.receive(b"\"V,0,DC\"\r\r");
        m.dispatch(t);
        m.receive(b"\"000000000000000000000\"\0junk");
        m.dispatch(t);
        assert!(m.receive(b"+1.00000000E+00\r\n").reading.is_some());
    }

    #[test]
    fn a_line_with_other_bytes_is_unknown() {
        let mut m = one("U1232A");
        assert_eq!(m.receive(b"\xff\xfe"), Received::default());
        assert_eq!(m.receive(b""), Received::default());
        assert_eq!(m.receive(b"\r\r"), Received::default());
    }

    #[test]
    fn stat_of_each_family() {
        let flags = |name: &str, s: &str| {
            let mut m = one(name);
            m.receive(s.as_bytes());
            m.flags[0]
        };
        let min_max = MqFlags::MAX | MqFlags::MIN;
        let all = min_max | MqFlags::AVG;
        let none = MqFlags::empty();
        assert_eq!(flags("U1232A", "\"100000000000000000000\""), min_max);
        assert_eq!(flags("U1232A", "\"010000000000000000000\""), MqFlags::RELATIVE);
        assert_eq!(flags("U1232A", "\"001000000000000000000\""), MqFlags::HOLD);
        assert_eq!(flags("U1232A", "\"000100000000000000000\""), MqFlags::HOLD);
        assert_eq!(flags("U1242B", "\"100000000000000000000\""), min_max);
        assert_eq!(flags("U1242B", "\"000000010000000000000\""), MqFlags::HOLD);
        assert_eq!(flags("U1242B", "\"000000000001000000000\""), none);
        assert_eq!(flags("U1242C", "\"100000000000000000000\""), all);
        assert_eq!(flags("U1242C", "\"000000000001000000000\""), MqFlags::HOLD);
        assert_eq!(flags("U1253B", "\"000010000000000000000\""), MqFlags::MAX);
        assert_eq!(flags("U1253B", "\"100000000000000000000\""), none);
        assert_eq!(flags("U1282A", "\"000010000000000000000\""), none, "C looks for a 4");
        assert_eq!(flags("U1282A", "\"000040000000000000000\""), MqFlags::MAX);
        // Byte 4 turns MAX off again after byte 0 (protocol.c:394-410).
        assert_eq!(flags("U1282A", "\"100000000000000000000\""), MqFlags::MIN | MqFlags::AVG);
        assert_eq!(flags("U1282A", "\"100040000000000000000\""), all);
    }

    #[test]
    fn stat_turns_flags_off_too() {
        let mut m = one("U1232A");
        m.receive(b"\"110100000000000000000\"");
        assert_eq!(m.flags[0], MqFlags::MAX | MqFlags::MIN | MqFlags::RELATIVE | MqFlags::HOLD);
        m.receive(b"\"000000000000000000000\"");
        assert_eq!(m.flags[0], MqFlags::empty());
    }

    #[test]
    fn stat_of_db_sets_the_unit_at_once_and_conf_the_digits() {
        let t = Instant::now();
        let mut m = one("U1253B");
        m.dispatch(t);
        m.receive(b"\"VOLT:AC +1.000000E+00,+1.000000E-04\"");
        assert_eq!(m.encoding[0], 5);
        m.dispatch(t);
        m.receive(b"\"00v000000000000000000\"");
        assert_eq!(m.unit[0], Unit::DecibelVolt);
        assert_eq!(m.encoding[0], 5, "until the next CONF");
        m.conf = 0;
        m.receive(b"\"VOLT:AC +1.000000E+00,+1.000000E-04\"");
        assert_eq!((m.unit[0], m.digits[0], m.encoding[0]), (Unit::DecibelVolt, 3, 4));
        m.receive(b"\"000000000000000000000\"");
        assert_eq!(m.unit[0], Unit::DecibelVolt, "stays until a CONF says otherwise");
        m.conf = 0;
        m.receive(b"\"VOLT:AC +1.000000E+00,+1.000000E-04\"");
        assert_eq!(m.unit[0], Unit::Volt);
    }

    #[test]
    fn conf_fields_scale_and_digits() {
        let mut m = one("U1272A");
        m.receive(b"\"MV,1,DC\"");
        assert_eq!((m.exponent[0], m.digits[0], m.encoding[0]), (-3, 4, 5));
        let r = reading(&mut m, "+1.23475000E+02");
        assert_eq!(r.digits, 8);
        assert_eq!(r.flags, MqFlags::DC);
        assert_eq!(r.value.to_bits(), (123.475_f32 * 10f32.powf(-3.0)).to_bits());
        for (line, mq, unit, flags, encoding) in [
            ("\"V,1,AC\"", Mq::Voltage, Unit::Volt, MqFlags::AC | MqFlags::RMS, 4),
            ("\"A,1,AC\"", Mq::Current, Unit::Ampere, MqFlags::AC, 3),
            (
                "\"A,1,ACDC\"",
                Mq::Current,
                Unit::Ampere,
                MqFlags::AC | MqFlags::DC | MqFlags::RMS,
                3,
            ),
            ("\"MA,1,DC\"", Mq::Current, Unit::Ampere, MqFlags::DC, 8),
            ("\"UA,1,DC\"", Mq::Current, Unit::Ampere, MqFlags::DC, 8),
            ("\"FREQ,1,AC\"", Mq::Frequency, Unit::Hertz, MqFlags::AC, 2),
            ("\"RES,1\"", Mq::Resistance, Unit::Ohm, MqFlags::empty(), 1),
            ("\"CAP,1\"", Mq::Capacitance, Unit::Farad, MqFlags::empty(), 9),
            // With no coupling C takes its DC off again (protocol.c:647).
            ("\"DIOD\"", Mq::Voltage, Unit::Volt, MqFlags::DIODE, 4),
            ("\"TEMP\"", Mq::Temperature, Unit::Celsius, MqFlags::empty(), 2),
        ] {
            m.receive(line.as_bytes());
            assert_eq!(
                (m.mq[0], m.unit[0], m.flags[0], m.encoding[0]),
                (Some(mq), unit, flags, encoding),
                "{line}"
            );
        }
    }

    #[test]
    fn modes_from_stat_change_the_next_conf() {
        let mut m = one("U1272A");
        m.receive(b"\"000000010000000010000\"");
        m.receive(b"\"MV,1,DC\"");
        assert_eq!((m.mq[0], m.exponent[0], m.digits[0]), (Some(Mq::Temperature), 0, 1));
        m.receive(b"\"RES,1\"");
        assert_eq!((m.mq[0], m.unit[0]), (Some(Mq::Continuity), Unit::Boolean));
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn conf_scpi_each_branch() {
        for (name, line, mq, unit, flags, digits, encoding) in [
            (
                "U1253B",
                "\"VOLT +1.000000E+00,+1.000000E-04\"",
                Mq::Voltage,
                Unit::Volt,
                MqFlags::DC,
                4,
                5,
            ),
            (
                "U1253B",
                "\"VOLT:ACDC +1.000000E+00,+1.000000E-04\"",
                Mq::Voltage,
                Unit::Volt,
                MqFlags::AC | MqFlags::DC | MqFlags::RMS,
                4,
                5,
            ),
            (
                "U1253B",
                "\"VOLT:CD +1.000000E+00,+1.000000E-04\"",
                Mq::Voltage,
                Unit::Volt,
                MqFlags::empty(),
                4,
                5,
            ),
            (
                "U1253B",
                "\"CURR:AC +1.000000E+00,+1.000000E-02\"",
                Mq::Current,
                Unit::Ampere,
                MqFlags::AC | MqFlags::RMS,
                2,
                3,
            ),
            (
                "U1253B",
                "\"CURR +1.000000E+00,+1.000000E-02\"",
                Mq::Current,
                Unit::Ampere,
                MqFlags::DC,
                2,
                3,
            ),
            (
                "U1253B",
                "\"RES +1.000000E+00,+1.000000E+01\"",
                Mq::Resistance,
                Unit::Ohm,
                MqFlags::empty(),
                -1,
                0,
            ),
            (
                "U1253B",
                "\"CONT +1.000000E+00,+1.000000E-01\"",
                Mq::Continuity,
                Unit::Ohm,
                MqFlags::empty(),
                1,
                2,
            ),
            (
                "U1253B",
                "\"COND +1.000000E+00,+1.000000E-09\"",
                Mq::Conductance,
                Unit::Siemens,
                MqFlags::empty(),
                9,
                10,
            ),
            (
                "U1253B",
                "\"CAP +1.000000E+00,+1.000000E-09\"",
                Mq::Capacitance,
                Unit::Farad,
                MqFlags::empty(),
                9,
                10,
            ),
            (
                "U1253B",
                "\"FREQ +1.000000E+00,+1.000000E-01\"",
                Mq::Frequency,
                Unit::Hertz,
                MqFlags::empty(),
                1,
                2,
            ),
            (
                "U1253B",
                "\"PULS:PWID +1.000000E+00,+1.000000E-09\"",
                Mq::PulseWidth,
                Unit::Second,
                MqFlags::empty(),
                9,
                6,
            ),
            ("U1253B", "\"PULS:PDUT\"", Mq::DutyCycle, Unit::Percentage, MqFlags::empty(), 3, 4),
            (
                "U1253B",
                "\"CPER:4-20mA +1.000000E+00,+1.000000E-04\"",
                Mq::Current,
                Unit::Percentage,
                MqFlags::empty(),
                2,
                3,
            ),
            ("U1253B", "\"DIOD\"", Mq::Voltage, Unit::Volt, MqFlags::DIODE | MqFlags::DC, 3, 4),
            ("U1282A", "\"DIOD\"", Mq::Voltage, Unit::Volt, MqFlags::DIODE | MqFlags::DC, 4, 5),
            ("U1253B", "\"T1:K FAR\"", Mq::Temperature, Unit::Fahrenheit, MqFlags::empty(), 1, 2),
            ("U1253B", "\"TEMP:K CEL\"", Mq::Temperature, Unit::Celsius, MqFlags::empty(), 1, 2),
            ("U1242C", "\"TEMP\"", Mq::Temperature, Unit::Celsius, MqFlags::empty(), 1, 2),
            ("U1242C", "\"NCV HI\"", Mq::Voltage, Unit::Volt, MqFlags::AC, 2, 3),
            ("U1282A", "\"NCV LOW\"", Mq::Voltage, Unit::Volt, MqFlags::AC, -1, 0),
            (
                "U1242C",
                "\"VOLT:HRAT +1.00000000E+00,+1.00000000E-04\"",
                Mq::HarmonicRatio,
                Unit::Percentage,
                MqFlags::empty(),
                2,
                3,
            ),
        ] {
            let mut m = one(name);
            m.receive(line.as_bytes());
            assert_eq!(
                (m.mq[0], m.unit[0], m.flags[0], m.digits[0], m.encoding[0]),
                (Some(mq), unit, flags, digits, encoding),
                "{name} {line}"
            );
        }
        let mut m = one("U1282A");
        m.receive(b"\"NCV LOW\"");
        assert_eq!(m.exponent[0], -3);
        assert_eq!(reading(&mut m, "+1.00000000E+00").digits, 3);
    }

    #[test]
    fn a_three_display_temp_with_no_word_is_dropped() {
        let mut m = one("U1282A");
        m.receive(b"\"VOLT +1.00000000E+00,+1.00000000E-04\"");
        assert!(m.mq[0].is_some());
        m.conf = 0;
        m.receive(b"\"TEMP\"");
        assert_eq!(m.mq[0], None);
    }

    #[test]
    fn ndut_keeps_what_came_before_and_t3_too() {
        let mut m = one("U1253B");
        m.receive(b"\"PULS:NDUT\"");
        assert_eq!(m.mq[0], None);
        m.conf = 0;
        m.receive(b"\"VOLT +1.000000E+00,+1.000000E-04\"");
        m.conf = 0;
        m.receive(b"\"PULS:NDUT\"");
        assert_eq!(m.mq[0], Some(Mq::Voltage));
        m.conf = 0;
        m.receive(b"\"T3:K CEL\"");
        assert_eq!(m.mq[0], Some(Mq::Voltage));
    }

    #[test]
    fn square_wave_means_no_fetc_until_a_conf_says_otherwise() {
        let t = Instant::now();
        let mut m = one("U1282A");
        m.dispatch(t);
        m.receive(b"\"SQU\"");
        m.dispatch(t);
        m.receive(b"\"000000000000000000000\"");
        assert_eq!(m.dispatch(t), None, "FETC?'s turn, and it sends nothing");
        assert!(m.square_wave);
        m.receive(b"\"DIOD\"");
        assert!(!m.square_wave);
    }

    #[test]
    fn percent_digits_shrink_with_the_value_and_stay() {
        let mut m = one("U1282A");
        m.receive(b"\"CPER:4-20mA +1.00000000E+00,+1.00000000E-04\"");
        assert_eq!(reading(&mut m, "+1.00000000E+00").digits, 3);
        assert_eq!(reading(&mut m, "+1.00000000E+03").digits, 2);
        assert_eq!(reading(&mut m, "+1.00000000E+00").digits, 2);
        let mut m = one("U1253B");
        m.receive(b"\"CPER:4-20mA +1.000000E+00,+1.000000E-04\"");
        assert_eq!(reading(&mut m, "+1.00000000E+03").digits, 3, "no capture in this table");
    }

    #[test]
    fn overloads_are_nan_and_only_as_c_spells_them() {
        let mut m = one("U1272A");
        m.receive(b"\"MV,1,DC\"");
        assert!(reading(&mut m, "+9.90000000E+37").value.is_nan());
        assert!(reading(&mut m, "-9.90000000E+37").value.is_nan());
        assert!(!reading(&mut m, "+9.90000000E+36").value.is_nan());
        let mut m = one("U1232A");
        m.receive(b"\"V,1,DC\"");
        assert!(reading(&mut m, "+1.00000000E+39").value.is_infinite());
    }

    #[test]
    fn a_late_answer_of_another_kind_does_not_end_this_job() {
        let t = Instant::now();
        let mut m = one("U1232A");
        m.dispatch(t);
        m.receive(b"+1.00000000E+00");
        assert_eq!(m.dispatch(t), None, "still waiting for CONF");
    }

    fn log(name: &str, source: usize) -> Meter {
        Meter::new(profile(name), vec![0], source, 5)
    }

    #[test]
    fn the_log_asks_by_index_each_source_its_own() {
        let t = Instant::now();
        for (source, command) in [
            (1, "LOG:HAND 1\n\r\n"),
            (2, "LOG:TRIG 1\n\r\n"),
            (3, "LOG:AUTO 1\n\r\n"),
            (4, "LOG:EXPO 1\n\r\n"),
        ] {
            assert_eq!(text(log("U1282A", source).dispatch(t)).as_deref(), Some(command));
        }
        let mut m = log("U1282A", 1);
        m.dispatch(t);
        assert_eq!(m.dispatch(t), None);
        assert!(m.receive(b"\"00123450000000\"").reading.is_some());
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("LOG:HAND 2\n\r\n"), "at once");
    }

    #[test]
    fn the_log_ends_at_an_error_and_a_live_capture_asks_again() {
        let t = Instant::now();
        assert!(log("U1282A", 1).receive(b"*E").stop);
        let mut m = one("U1282A");
        m.dispatch(t);
        let r = m.receive(b"*E");
        assert!(!r.stop);
        assert_eq!(text(m.dispatch(t)).as_deref(), Some("CONF?\n\r\n"), "again");
    }

    #[test]
    fn a_bad_function_waits_for_the_jobs_timeout_on_the_same_index() {
        let t = Instant::now();
        let at = |ms| t + Duration::from_millis(ms);
        let mut m = log("U1282A", 1);
        m.dispatch(t);
        assert_eq!(m.receive(b"\"14123450000000\"").reading, None);
        assert_eq!(m.dispatch(at(300)), None);
        assert_eq!(text(m.dispatch(at(301))).as_deref(), Some("LOG:HAND 1\n\r\n"));
        let mut m = log("U1241C", 1);
        m.dispatch(t);
        assert_eq!(m.receive(b"\"11123450000000\"").reading, None);
        assert!(m.receive(b"\"10123450000000\"").reading.is_some(), "the last it has");
    }

    #[test]
    fn a_log_entrys_flags() {
        let entry = |name: &str, line: &str| {
            let mut m = log(name, 1);
            reading(&mut m, line)
        };
        let r = entry("U1282A", "\"00123452000000\"");
        assert_eq!((r.value.is_sign_negative(), r.digits), (true, 6));
        assert_eq!(entry("U1282A", "\"00123450100000\"").flags, MqFlags::DC);
        assert_eq!(entry("U1282A", "\"00123450200000\"").flags, MqFlags::AC);
        assert!(entry("U1282A", "\"00123450400000\"").value.is_nan());
        assert_eq!(entry("U1282A", "\"00123450030000\"").digits, 3);
        assert_eq!(entry("U1282A", "\"00123450000400\"").flags, MqFlags::RELATIVE);
        assert_eq!(entry("U1282A", "\"00123450000010\"").flags, MqFlags::AVG);
        assert_eq!(entry("U1282A", "\"00123450000020\"").flags, MqFlags::MIN);
        assert_eq!(entry("U1282A", "\"00123450000040\"").flags, MqFlags::MAX);
        let r = entry("U1282A", "\"05123450000000\"");
        assert_eq!((r.mq, r.flags), (Mq::Voltage, MqFlags::DIODE | MqFlags::DC));
        let r = entry("U1282A", "\"04123450001000\"");
        assert_eq!((r.mq, r.unit), (Mq::Continuity, Unit::Ohm));
        assert_eq!(entry("U1282A", "\"11123450001000\"").unit, Unit::DecibelVolt);
        let r = entry("U1282A", "\"06123450001000\"");
        assert_eq!((r.unit, r.digits), (Unit::Fahrenheit, 2), "one more digit here");
        let r = entry("U1241C", "\"06123450001000\"");
        assert_eq!((r.unit, r.digits), (Unit::Fahrenheit, 1), "not here");
        assert_eq!(entry("U1282A", "\"00123450000000\"").digits, 6);
        assert_eq!(entry("U1241C", "\"00123450000000\"").digits, 5);
    }

    #[test]
    fn a_log_value_is_a_float_product() {
        let mut m = log("U1282A", 1);
        let r = reading(&mut m, "\"00123450000000\"");
        assert_eq!(r.value.to_bits(), (12345_f32 * 10f32.powf(-6.0)).to_bits());
    }

    #[test]
    fn the_log_tables_end_where_c_has_them() {
        assert!(log_function(Family::U124xc, 10).is_some());
        assert!(log_function(Family::U124xc, 11).is_none());
        assert!(log_function(Family::U128x, 13).is_some());
        assert!(log_function(Family::U128x, 14).is_none());
        assert!(log_function(Family::U125x, 0).is_none());
    }
}
