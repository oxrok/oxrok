// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! Yokogawa DL and DLM oscilloscopes, on USBTMC and on TCP.
//!
//! The device, and how sure this is:
//!
//! - USB: 0b21:0030, "Yokogawa Electric Corporation", "DLM series", high speed, bulk
//!   OUT 1, bulk IN 3, from the `lsusb -v` on sigrok's wiki
//!   (`Yokogawa_DLM2000_series/Info`). The page doesn't say which DLM2000 it is, and the
//!   DL9000's id is a guess: the same one. The real one also has an interrupt IN 2, which
//!   the shell has not. Whether it says `RL1` is unknown: `remote` is on here, so that a
//!   tool that left Yokogawa off `scpi_usbtmc_libusb.c`'s table shows in the `.tx`.
//! - Commands and answers, for the DLM2000: IM 710105-17E, the communication interface
//!   manual that `protocol_wrappers.c` names. `1`, `0` for a display; `2.000E+00` for a
//!   division, `1.00E+00` for a position, `2.00E-06` for a delay, `5.000E+00` for a
//!   range, `1.25E+09` for a rate, a plain `12500` for a length (pp. 5-78 to 5-80,
//!   5-241, 5-289, 5-290); signed bytes behind `#8` and eight digits, volts being range
//!   times data over 12.5 plus offset (p. 5-290).
//! - The DL9000 series: only what libsigrok's table says of them; no manual was read.
//!   `:LOGIC:PODA:BIT9:DISPLAY?` on a DL9505L is what C sends, and is taken.
//! - A zero-length block while the scope runs, which `:STOP` ends: only C's comment at
//!   `protocol.c:1069-1071` says so. The manual has that a `:WAVEFORM:SEND?` during an
//!   acquisition "may be executed regardless" and may be a command error (p. 4-9).
//!
//! Emulation artifacts:
//!
//! - The range is five times the division, the sample rate the record over ten
//!   divisions, with no cap. The manual ties neither down. No golden should be read
//!   as saying a real scope's range.
//! - A ramp per channel, not a signal, so that every byte value comes, and a byte
//!   counter on the logic pod.
//! - A delay the manual's 0 to 10 s refuses is the error -222 in the queue, the value
//!   kept and no violation: C sends those for a position past the middle
//!   (`api.c:331-336`), and no answer says so.
//!
//! The pedantic checks: what the model doesn't have (a bit, a channel, a pod, a
//! trace); a query with response headers still on; `:WAVEFORM:SEND?` with no trace, of
//! a channel switched off, of any other format or record, or with an argument but
//! 1; `:WAVEFORM:RANGE?` and `:WAVEFORM:OFFSET?` of no analog trace; a setting that is
//! no number or no such word.
//!
//! Options, besides the shell's ([`scpi::Config::option`]):
//!
//! - `model=NAME`: a name in `protocol.c`'s table, DLM2024 by default;
//! - `length=N`: `:WAVEFORM:LENGTH?`, and the samples of a block (default 1250);
//! - `on=1,2,D0,...`: the channels and logic bits displayed (default all analog ones);
//! - `logic=1`: `:LOGIC:MODE?` says on;
//! - `running=1`: blocks are empty until `:STOP`; `running=stuck`: `:STOP` changes
//!   nothing;
//! - `cut-at=N`: the Nth block holds half of what its header says;
//! - `claim=N`: every block's header says N, the bytes staying the record's;
//! - `say=HEADER?=TEXT`, as often as needed: that query answers TEXT, always. It takes
//!   the place of a value, a block, or anything else a query answers.

use std::collections::BTreeMap;

use crate::model::{DeviceModel, Speed, Violations};
use crate::options::parse_number;
use crate::shells::scpi::{self, Answer, Instrument, Message, Settings, block};
use crate::shells::tcp::Stream;
use crate::shells::usbtmc::Identity;
use crate::txlog::{HASH_ABOVE, Rules};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "yokogawa-dlm";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Yokogawa DL and DLM scopes on USBTMC and TCP; model=, length=, on=, logic=, \
                         running=, cut-at=, claim=, say=, and the SCPI shell's mute-at=, silent-at=, \
                         late-at=, opc-late=, mark-at=, idn=, eol=, stall=, in-stall=, empty-at=, \
                         bad-tag-at=, message=";

/// A row of `protocol.c`'s `scope_models`.
struct Kind {
    id: &'static str,
    name: &'static str,
    analog: u8,
    digital: u8,
    pods: u8,
}

#[rustfmt::skip]
const KINDS: &[Kind] = &[
    Kind { id: "710105", name: "DLM2022", analog: 2, digital: 0, pods: 0 },
    Kind { id: "710115", name: "DLM2032", analog: 2, digital: 0, pods: 0 },
    Kind { id: "710125", name: "DLM2052", analog: 2, digital: 0, pods: 0 },
    Kind { id: "710110", name: "DLM2024", analog: 4, digital: 8, pods: 1 },
    Kind { id: "710120", name: "DLM2034", analog: 4, digital: 8, pods: 1 },
    Kind { id: "710130", name: "DLM2054", analog: 4, digital: 8, pods: 1 },
    Kind { id: "701307", name: "DL9040", analog: 4, digital: 0, pods: 0 },
    Kind { id: "701308", name: "DL9040L", analog: 4, digital: 0, pods: 0 },
    Kind { id: "701310", name: "DL9140", analog: 4, digital: 0, pods: 0 },
    Kind { id: "701311", name: "DL9140L", analog: 4, digital: 0, pods: 0 },
    Kind { id: "701312", name: "DL9240", analog: 4, digital: 0, pods: 0 },
    Kind { id: "701313", name: "DL9240L", analog: 4, digital: 0, pods: 0 },
    Kind { id: "701320", name: "DL9505L", analog: 4, digital: 16, pods: 4 },
    Kind { id: "701321", name: "DL9510L", analog: 4, digital: 16, pods: 4 },
    Kind { id: "701330", name: "DL9705L", analog: 4, digital: 32, pods: 4 },
    Kind { id: "701331", name: "DL9710L", analog: 4, digital: 32, pods: 4 },
];

/// What `:WAVEFORM:TRACE` chose.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Trace {
    Analog(u8),
    Logic,
}

/// How a running acquisition goes.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Running {
    No,
    UntilStop,
    Stuck,
}

/// A scope.
pub(super) struct Scope {
    kind: &'static Kind,
    length: u32,
    on: Vec<String>,
    logic: bool,
    running: Running,
    cut_at: Option<u32>,
    claim: Option<u32>,
    say: BTreeMap<String, String>,
    trace: Option<Trace>,
    headers: bool,
    blocks: u32,
}

/// A number as the scope writes it: `places` decimals, an exponent of two digits at
/// least and a sign on it.
fn scientific(value: f64, places: usize) -> String {
    let text = format!("{value:.places$E}");
    let (mantissa, exponent) = text.split_once('E').unwrap_or((&text, "0"));
    let (sign, digits) = exponent.strip_prefix('-').map_or(('+', exponent), |d| ('-', d));
    format!("{mantissa}E{sign}{digits:0>2}")
}

impl Scope {
    /// `CHANNELn:` and the rest of a header, for a channel the model has.
    fn channel<'m>(&self, name: &'m str) -> Option<(u8, &'m str)> {
        let (head, tail) = name.split_once(':')?;
        let n: u8 = head.strip_prefix("CHANNEL")?.parse().ok()?;
        (1..=self.kind.analog).contains(&n).then_some((n, tail))
    }

    /// A number with this many decimals, stored under the header.
    fn number(m: &Message<'_>, s: &mut Settings, places: usize, v: &mut Violations) -> Answer {
        match m.args.parse::<f64>() {
            Ok(value) if value.is_finite() => {
                s.set(m.name(), scientific(value, places));
                Answer::Done
            }
            _ => {
                v.report(format!("{}: no number", m.text));
                Answer::Error(-120, "Numeric data error".to_owned())
            }
        }
    }

    fn word(m: &Message<'_>, s: &mut Settings, words: &[&str], v: &mut Violations) -> Answer {
        if !words.contains(&m.args) {
            v.report(format!("{}: no such setting", m.text));
        }
        s.set(m.name(), m.args);
        Answer::Done
    }

    fn switch(m: &Message<'_>, s: &mut Settings, v: &mut Violations) -> Answer {
        match m.args {
            "ON" => s.set(m.name(), "1"),
            "OFF" => s.set(m.name(), "0"),
            _ => v.report(format!("{}: neither ON nor OFF", m.text)),
        }
        Answer::Done
    }

    /// The channel of the trace, for the queries about an analog trace alone.
    fn analog_trace(&self, m: &Message<'_>, v: &mut Violations) -> Option<u8> {
        if let Some(Trace::Analog(c)) = self.trace {
            return Some(c);
        }
        v.report(format!("{}: no analog trace chosen", m.text));
        None
    }

    fn wave(channel: u8, length: usize) -> Vec<u8> {
        (0..length).map(|i| (i * 3 + usize::from(channel) * 37) as u8).collect()
    }

    /// `:WAVEFORM:SEND? 1`.
    fn send(&mut self, m: &Message<'_>, s: &Settings, v: &mut Violations) -> Answer {
        if m.args != "1" {
            v.report(format!("{}: an argument other than 1", m.text));
        }
        for (name, wanted) in [("WAVEFORM:FORMAT", "BYTE"), ("WAVEFORM:RECORD", "0")] {
            if s.get(name) != Some(wanted) {
                v.report(format!("{}: with {name} not {wanted}", m.text));
            }
        }
        let length = self.length as usize;
        let wave = match self.trace {
            Some(Trace::Analog(c)) => {
                if s.get(&format!("CHANNEL{c}:DISPLAY")) != Some("1") {
                    v.report(format!("{}: of channel {c}, which is off", m.text));
                }
                Scope::wave(c, length)
            }
            Some(Trace::Logic) => (0..length).map(|i| i as u8).collect(),
            None => {
                v.report(format!("{}: no trace chosen", m.text));
                Vec::new()
            }
        };
        if self.running != Running::No {
            return Answer::Bytes(block(8, &[], b"\n"));
        }
        self.blocks += 1;
        let keep = if self.cut_at == Some(self.blocks) { length / 2 } else { length };
        let mut bytes = block(8, &wave[..keep.min(wave.len())], b"\n");
        let said = self.claim.unwrap_or(self.length);
        bytes.splice(2..10, format!("{said:08}").into_bytes());
        Answer::Bytes(bytes)
    }

    fn trace(&mut self, m: &Message<'_>, v: &mut Violations) -> Answer {
        self.trace = if m.args == "LOGIC" {
            if self.kind.digital == 0 {
                v.report(format!("{}: this model has no logic", m.text));
            }
            Some(Trace::Logic)
        } else {
            match m.args.parse::<u8>() {
                Ok(c) if (1..=self.kind.analog).contains(&c) => Some(Trace::Analog(c)),
                _ => {
                    v.report(format!("{}: no such trace", m.text));
                    None
                }
            }
        };
        Answer::Done
    }

    fn logic_bit(&self, name: &str) -> bool {
        name.strip_prefix("LOGIC:PODA:BIT")
            .and_then(|r| r.strip_suffix(":DISPLAY"))
            .and_then(|n| n.parse::<u8>().ok())
            .is_some_and(|n| (1..=self.kind.digital).contains(&n))
    }

    fn rate(s: &Settings, length: u32) -> f64 {
        f64::from(length) / (s.number("TIMEBASE:TDIV") * 10.0)
    }
}

impl Instrument for Scope {
    fn idn(&self) -> Option<String> {
        Some(format!("YOKOGAWA,{},EMU0001,F1.00", self.kind.id))
    }

    fn settings(&self) -> Settings {
        let mut s = Settings::default()
            .with("TIMEBASE:TDIV", "1.000E-03")
            .with("TRIGGER:DELAY:TIME", "0.00E+00")
            .with("TRIGGER:ATRIGGER:SIMPLE:SOURCE", "1")
            .with("TRIGGER:ATRIGGER:SIMPLE:SLOPE", "RISE")
            .with("WAVEFORM:FORMAT", "ASCII")
            .with("WAVEFORM:RECORD", "0")
            .with("WAVEFORM:START", "0")
            .with("WAVEFORM:END", "12499");
        for c in 1..=self.kind.analog {
            let shown = self.on.iter().any(|o| *o == c.to_string());
            s.set(&format!("CHANNEL{c}:DISPLAY"), if shown { "1" } else { "0" });
            s.set(&format!("CHANNEL{c}:VDIV"), "1.000E+00");
            s.set(&format!("CHANNEL{c}:POSITION"), "0.00E+00");
            s.set(&format!("CHANNEL{c}:COUPLING"), "DC");
        }
        for bit in 1..=self.kind.digital {
            let shown = self.on.iter().any(|o| *o == format!("D{}", bit - 1));
            s.set(&format!("LOGIC:PODA:BIT{bit}:DISPLAY"), if shown { "1" } else { "0" });
        }
        if self.kind.pods > 0 {
            s.set("LOGIC:MODE", if self.logic { "1" } else { "0" });
        }
        s
    }

    fn command(&mut self, m: &Message<'_>, s: &mut Settings, v: &mut Violations) -> Answer {
        let name = m.name().to_owned();
        if m.header == "COMMUNICATE:HEADER" {
            match m.args {
                "OFF" => self.headers = false,
                "ON" => self.headers = true,
                _ => v.report(format!("{}: neither ON nor OFF", m.text)),
            }
            return Answer::Done;
        }
        if m.is_query() && self.headers && m.header != "*IDN?" && m.header != "*OPC?" {
            v.report(format!("{}: asked with response headers on", m.text));
        }
        if m.is_query()
            && let Some(text) = self.say.get(&m.header)
        {
            return Answer::Text(text.clone());
        }
        if let Some((_, tail)) = self.channel(&name)
            && !m.is_query()
        {
            return match tail {
                "DISPLAY" => Scope::switch(m, s, v),
                "VDIV" => Scope::number(m, s, 3, v),
                "COUPLING" => Scope::word(m, s, &["AC", "DC", "DC50", "GND"], v),
                _ => Answer::Unknown,
            };
        }
        match m.header.as_str() {
            "STOP" => {
                if self.running == Running::UntilStop {
                    self.running = Running::No;
                }
                Answer::Done
            }
            "TIMEBASE:TDIV" => Scope::number(m, s, 3, v),
            "TRIGGER:DELAY:TIME" => match m.args.parse::<f64>() {
                Ok(t) if (0.0..=10.0).contains(&t) && !t.is_sign_negative() => {
                    s.set(&name, scientific(t, 2));
                    Answer::Done
                }
                _ => Answer::Error(-222, "Data out of range".to_owned()),
            },
            "TRIGGER:ATRIGGER:SIMPLE:SOURCE" => {
                let analog = m.args.parse::<u8>().is_ok_and(|c| (1..=self.kind.analog).contains(&c));
                let bit = m
                    .args
                    .strip_prefix("BIT")
                    .and_then(|b| b.parse::<u8>().ok())
                    .is_some_and(|b| (1..=8).contains(&b));
                if !analog && !bit && !matches!(m.args, "EXT" | "LINE") {
                    v.report(format!("{}: no such source", m.text));
                }
                s.set(&name, m.args);
                Answer::Done
            }
            "TRIGGER:ATRIGGER:SIMPLE:SLOPE" => Scope::word(m, s, &["RISE", "FALL"], v),
            "LOGIC:MODE" | "LOGIC:MODE?" if self.kind.pods == 0 => {
                v.report(format!("{}: this model has no logic", m.text));
                Answer::Silent
            }
            "LOGIC:MODE" => Scope::switch(m, s, v),
            "WAVEFORM:FORMAT" => Scope::word(m, s, &["BYTE"], v),
            "WAVEFORM:RECORD" | "WAVEFORM:START" => Scope::word(m, s, &["0"], v),
            "WAVEFORM:END" => Scope::word(m, s, &["124999999"], v),
            "WAVEFORM:TRACE" => self.trace(m, v),
            "WAVEFORM:RANGE?" => match self.analog_trace(m, v) {
                Some(c) => {
                    let range = s.number(&format!("CHANNEL{c}:VDIV")) * 5.0;
                    Answer::Text(scientific(range, 3))
                }
                None => Answer::Silent,
            },
            "WAVEFORM:OFFSET?" => match self.analog_trace(m, v) {
                Some(_) => Answer::Text("0.000E+00".to_owned()),
                None => Answer::Silent,
            },
            "WAVEFORM:SRATE?" => Answer::Text(scientific(Scope::rate(s, self.length), 2)),
            "WAVEFORM:LENGTH?" => Answer::Text(self.length.to_string()),
            "WAVEFORM:SEND?" => self.send(m, s, v),
            _ if self.logic_bit(&name) && !m.is_query() => Scope::switch(m, s, v),
            _ if name.starts_with("LOGIC:PODA:BIT") && !self.logic_bit(&name) => {
                v.report(format!("{}: no such bit on this model", m.text));
                Answer::Silent
            }
            _ => Answer::Unknown,
        }
    }
}

/// The scope and the shell's options, whichever carrier they go on.
pub(super) fn parts(options: &[(String, String)]) -> Result<(Scope, scpi::Config)> {
    let mut config = scpi::Config::default();
    let mut scope = Scope {
        kind: &KINDS[3],
        length: 1250,
        on: Vec::new(),
        logic: false,
        running: Running::No,
        cut_at: None,
        claim: None,
        say: BTreeMap::new(),
        trace: None,
        headers: true,
        blocks: 0,
    };
    let mut on = None;
    for (k, v) in options {
        if config.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => {
                scope.kind = KINDS
                    .iter()
                    .find(|m| m.name == v)
                    .ok_or_else(|| Error::Invalid(format!("model={v}: no such model here")))?;
            }
            "length" => scope.length = parse_number(k, v, 0..=1 << 24)?,
            "on" => on = Some(v.split(',').map(str::to_owned).collect::<Vec<_>>()),
            "logic" => scope.logic = parse_number(k, v, 0..=1)? == 1,
            "running" => {
                scope.running = match v.as_str() {
                    "0" => Running::No,
                    "1" => Running::UntilStop,
                    "stuck" => Running::Stuck,
                    _ => return Err(Error::Invalid(format!("running={v}: 0, 1 or stuck"))),
                };
            }
            "cut-at" => scope.cut_at = Some(parse_number(k, v, 1..=1 << 24)?),
            "claim" => scope.claim = Some(parse_number(k, v, 0..=99_999_999)?),
            "say" => {
                let (header, text) = v
                    .split_once("?=")
                    .ok_or_else(|| Error::Invalid(format!("say={v}: as in A:B?=TEXT")))?;
                scope.say.insert(format!("{}?", header.to_ascii_uppercase()), text.to_owned());
            }
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    scope.on = on.unwrap_or_else(|| (1..=scope.kind.analog).map(|c| c.to_string()).collect());
    Ok((scope, config))
}

/// A scope on USBTMC, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (scope, config) = parts(options)?;
    let mut id = Identity::new(0x0b21, 0x0030, "Yokogawa Electric Corporation", "DLM series");
    id.speed = Speed::High;
    id.remote = true;
    Ok(config.usbtmc(NAME, &id, scope))
}

/// The same scope on a socket.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    let (scope, config) = parts(options)?;
    Ok(config.tcp(scope))
}

/// A frame is long: its digest in the log.
pub fn log_rules(_options: &[(String, String)]) -> Rules {
    Rules::digest_above(HASH_ABOVE)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::tcp::Port;

    fn on_socket(options: &[(&str, &str)]) -> Box<dyn Stream> {
        let pairs: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        build_tcp(&pairs).unwrap()
    }

    /// A line, and the bytes and the violations it makes.
    fn ask(s: &mut Box<dyn Stream>, line: &str) -> (Vec<u8>, Vec<String>) {
        let mut port = Port::default();
        s.received(line.as_bytes(), &mut port);
        let bytes = port.sent().concat();
        (bytes, port.violations().drain())
    }

    /// A scope as a scan leaves it.
    fn scanned(options: &[(&str, &str)]) -> Box<dyn Stream> {
        let mut s = on_socket(options);
        ask(&mut s, ":COMMUNICATE:HEADER OFF\n");
        s
    }

    fn prepared(s: &mut Box<dyn Stream>, trace: &str) {
        for line in [":WAVEFORM:FORMAT BYTE", ":WAVEFORM:RECORD 0", ":WAVEFORM:START 0"] {
            assert_eq!(ask(s, &format!("{line}\n")).1, Vec::<String>::new());
        }
        ask(s, ":WAVEFORM:END 124999999\n");
        ask(s, &format!(":WAVEFORM:TRACE {trace}\n"));
    }

    #[test]
    fn it_answers_as_the_manual_shows() {
        let mut s = scanned(&[]);
        assert_eq!(ask(&mut s, "*IDN?\n").0, b"YOKOGAWA,710110,EMU0001,F1.00\n");
        assert_eq!(ask(&mut s, ":CHANNEL1:DISPLAY?\n").0, b"1\n");
        assert_eq!(ask(&mut s, ":CHANNEL1:VDIV?\n").0, b"1.000E+00\n");
        assert_eq!(ask(&mut s, ":CHANNEL1:POSITION?\n").0, b"0.00E+00\n");
        assert_eq!(ask(&mut s, ":TRIGGER:DELAY:TIME?\n").0, b"0.00E+00\n");
        assert_eq!(ask(&mut s, ":WAVEFORM:LENGTH?\n").0, b"1250\n");
        assert_eq!(ask(&mut s, ":WAVEFORM:SRATE?\n").0, b"1.25E+05\n");
        ask(&mut s, ":CHANNEL1:VDIV 5.000000E-01\n");
        assert_eq!(ask(&mut s, ":CHANNEL1:VDIV?\n").0, b"5.000E-01\n");
        ask(&mut s, ":WAVEFORM:TRACE 1\n");
        assert_eq!(ask(&mut s, ":WAVEFORM:RANGE?\n").0, b"2.500E+00\n");
        assert_eq!(ask(&mut s, ":WAVEFORM:OFFSET?\n").0, b"0.000E+00\n");
    }

    #[test]
    fn a_block_has_eight_digits_and_a_line_end() {
        let mut s = scanned(&[("length", "100")]);
        prepared(&mut s, "2");
        let (block, flags) = ask(&mut s, ":WAVEFORM:SEND? 1\n");
        assert_eq!(flags, Vec::<String>::new());
        assert_eq!(&block[..10], b"#800000100");
        assert_eq!(block.len(), 111);
        assert_eq!(block[10], 74, "channel 2's first");
        assert_eq!(block.last(), Some(&b'\n'));
    }

    #[test]
    fn a_running_scope_sends_empty_blocks_until_stopped() {
        let mut s = scanned(&[("length", "10"), ("running", "1")]);
        prepared(&mut s, "1");
        assert_eq!(ask(&mut s, ":WAVEFORM:SEND? 1\n").0, b"#800000000\n");
        ask(&mut s, ":STOP\n");
        assert_eq!(ask(&mut s, ":WAVEFORM:SEND? 1\n").0.len(), 21);
        let mut s = scanned(&[("length", "10"), ("running", "stuck")]);
        prepared(&mut s, "1");
        ask(&mut s, ":STOP\n");
        assert_eq!(ask(&mut s, ":WAVEFORM:SEND? 1\n").0, b"#800000000\n");
    }

    #[test]
    fn the_faults_of_a_block() {
        let mut s = scanned(&[("length", "10"), ("cut-at", "2")]);
        prepared(&mut s, "1");
        assert_eq!(ask(&mut s, ":WAVEFORM:SEND? 1\n").0.len(), 21);
        let cut = ask(&mut s, ":WAVEFORM:SEND? 1\n").0;
        assert_eq!(&cut[..10], b"#800000010");
        assert_eq!(cut.len(), 16, "half, and what the header says is unchanged");
        let mut s = scanned(&[("length", "10"), ("claim", "7")]);
        prepared(&mut s, "1");
        let claimed = ask(&mut s, ":WAVEFORM:SEND? 1\n").0;
        assert_eq!(&claimed[..10], b"#800000007");
        assert_eq!(claimed.len(), 21);
    }

    #[test]
    fn say_takes_the_place_of_any_answer() {
        let mut s = scanned(&[("say", "WAVEFORM:SEND?=#"), ("say", "CHANNEL2:VDIV?=abc")]);
        assert_eq!(ask(&mut s, ":CHANNEL2:VDIV?\n").0, b"abc\n");
        prepared(&mut s, "1");
        assert_eq!(ask(&mut s, ":WAVEFORM:SEND? 1\n").0, b"#\n");
    }

    #[test]
    fn who_is_shown_comes_from_the_options() {
        let mut s = scanned(&[("on", "2,D3"), ("logic", "1")]);
        assert_eq!(ask(&mut s, ":CHANNEL1:DISPLAY?\n").0, b"0\n");
        assert_eq!(ask(&mut s, ":CHANNEL2:DISPLAY?\n").0, b"1\n");
        assert_eq!(ask(&mut s, ":LOGIC:PODA:BIT4:DISPLAY?\n").0, b"1\n");
        assert_eq!(ask(&mut s, ":LOGIC:PODA:BIT1:DISPLAY?\n").0, b"0\n");
        assert_eq!(ask(&mut s, ":LOGIC:MODE?\n").0, b"1\n");
    }

    #[test]
    fn a_setting_changes_state() {
        let mut s = scanned(&[]);
        ask(&mut s, ":CHANNEL1:DISPLAY OFF\n");
        assert_eq!(ask(&mut s, ":CHANNEL1:DISPLAY?\n").0, b"0\n");
        ask(&mut s, ":CHANNEL1:COUPLING AC\n");
        assert_eq!(ask(&mut s, ":CHANNEL1:COUPLING?\n").0, b"AC\n");
        ask(&mut s, ":TIMEBASE:TDIV 2.000000E-06\n");
        assert_eq!(ask(&mut s, ":TIMEBASE:TDIV?\n").0, b"2.000E-06\n");
        ask(&mut s, ":TRIGGER:ATRIGGER:SIMPLE:SLOPE FALL\n");
        assert_eq!(ask(&mut s, ":TRIGGER:ATRIGGER:SIMPLE:SLOPE?\n").0, b"FALL\n");
        ask(&mut s, ":LOGIC:PODA:BIT1:DISPLAY ON\n");
        assert_eq!(ask(&mut s, ":LOGIC:PODA:BIT1:DISPLAY?\n").0, b"1\n");
        ask(&mut s, ":LOGIC:MODE ON\n");
        assert_eq!(ask(&mut s, ":LOGIC:MODE?\n").0, b"1\n");
    }

    #[test]
    fn a_delay_the_manual_refuses_stays_out_with_no_violation() {
        let mut s = scanned(&[]);
        assert_eq!(ask(&mut s, ":TRIGGER:DELAY:TIME 2.000000E-06\n").1, Vec::<String>::new());
        assert_eq!(ask(&mut s, ":TRIGGER:DELAY:TIME?\n").0, b"2.00E-06\n");
        assert_eq!(ask(&mut s, ":TRIGGER:DELAY:TIME -5.000000E-03\n").1, Vec::<String>::new());
        assert_eq!(ask(&mut s, ":TRIGGER:DELAY:TIME -0.000000E+00\n").1, Vec::<String>::new());
        assert_eq!(ask(&mut s, ":TRIGGER:DELAY:TIME NAN\n").1, Vec::<String>::new());
        assert_eq!(ask(&mut s, ":TRIGGER:DELAY:TIME?\n").0, b"2.00E-06\n", "kept");
        assert!(String::from_utf8(ask(&mut s, "SYST:ERR?\n").0).unwrap().starts_with("-222"));
    }

    /// Every check, firing.
    #[test]
    fn what_a_host_must_not_do_is_flagged() {
        let flagged = |s: &mut Box<dyn Stream>, line: &str| {
            let flags = ask(s, &format!("{line}\n")).1;
            assert_eq!(flags.len(), 1, "{line}: {flags:?}");
        };
        let mut s = on_socket(&[]);
        flagged(&mut s, ":CHANNEL1:DISPLAY?");
        ask(&mut s, ":COMMUNICATE:HEADER OFF\n");
        flagged(&mut s, ":COMMUNICATE:HEADER MAYBE");
        flagged(&mut s, ":CHANNEL1:DISPLAY MAYBE");
        flagged(&mut s, ":CHANNEL1:VDIV abc");
        flagged(&mut s, ":CHANNEL1:COUPLING ICE");
        flagged(&mut s, ":TIMEBASE:TDIV abc");
        flagged(&mut s, ":TRIGGER:ATRIGGER:SIMPLE:SOURCE 9");
        flagged(&mut s, ":TRIGGER:ATRIGGER:SIMPLE:SLOPE UP");
        flagged(&mut s, ":WAVEFORM:FORMAT WORD");
        flagged(&mut s, ":WAVEFORM:RECORD 1");
        flagged(&mut s, ":WAVEFORM:START 1");
        flagged(&mut s, ":WAVEFORM:END 1");
        flagged(&mut s, ":WAVEFORM:TRACE 5");
        flagged(&mut s, ":WAVEFORM:RANGE?");
        flagged(&mut s, ":WAVEFORM:OFFSET?");
        flagged(&mut s, ":LOGIC:PODA:BIT9:DISPLAY?");
        flagged(&mut s, ":NO:SUCH:THING");
        flagged(&mut s, ":CHANNEL5:DISPLAY?");

        let mut s = scanned(&[("on", "2")]);
        ask(&mut s, ":WAVEFORM:FORMAT BYTE\n");
        flagged(&mut s, ":WAVEFORM:SEND? 1");
        ask(&mut s, ":WAVEFORM:TRACE 1\n");
        flagged(&mut s, ":WAVEFORM:SEND? 1");
        ask(&mut s, ":WAVEFORM:TRACE 2\n");
        flagged(&mut s, ":WAVEFORM:SEND? 2");
        let mut s = scanned(&[("on", "1")]);
        ask(&mut s, ":WAVEFORM:FORMAT BYTE\n");
        ask(&mut s, ":WAVEFORM:TRACE 1\n");
        assert_eq!(ask(&mut s, ":WAVEFORM:SEND? 1\n").1, Vec::<String>::new());
        ask(&mut s, ":WAVEFORM:FORMAT WORD\n");
        flagged(&mut s, ":WAVEFORM:SEND? 1");

        let mut s = scanned(&[("model", "DLM2022")]);
        flagged(&mut s, ":LOGIC:MODE?");
        flagged(&mut s, ":LOGIC:MODE ON");
        flagged(&mut s, ":WAVEFORM:TRACE LOGIC");
        flagged(&mut s, ":CHANNEL3:DISPLAY?");
    }

    #[test]
    fn the_table_is_on_usb_too() {
        for kind in KINDS {
            let options = [("model".to_owned(), kind.name.to_owned())];
            let m = build(&options).unwrap();
            assert_eq!(m.descriptors().vendor_id(), 0x0b21);
            assert_eq!(m.descriptors().product_id(), 0x0030);
        }
        let dl = [("model".to_owned(), "DL9705L".to_owned())];
        let (scope, _) = parts(&dl).unwrap();
        assert_eq!(scope.kind.digital, 32);
        assert!(build(&[("model".to_owned(), "DS9".to_owned())]).is_err());
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
        assert!(build(&[("say".to_owned(), "no question".to_owned())]).is_err());
        assert!(build(&[("running".to_owned(), "2".to_owned())]).is_err());
    }

    #[test]
    fn numbers_are_written_as_the_scope_writes_them() {
        assert_eq!(scientific(2e-6, 3), "2.000E-06");
        assert_eq!(scientific(1.25e9, 2), "1.25E+09");
        assert_eq!(scientific(0.0, 2), "0.00E+00");
    }
}
