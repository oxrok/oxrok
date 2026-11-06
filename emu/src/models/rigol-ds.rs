// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! Rigol and Agilent oscilloscopes, on USBTMC and on TCP: the template for a SCPI
//! instrument's model (`docs/USB.md`, "Writing a SCPI model").
//!
//! The device, and how sure this is:
//!
//! - DS1102E (the default): a recording of a real one, firmware 00.02.06.00.01
//!   (`tests/fixtures/rigol_ds/`): text with no line end, block headers of eight
//!   digits, no line end after a block, `CH1` for the trigger source. That
//!   `*OPC?` has no use on a DS1000 is what libsigrok's driver says of it, from
//!   a DS1102D; it is flagged here.
//! - The other series: from what libsigrok's driver sends and what its comments
//!   say of answers (`#9` headers and a line end after each block on a DS2072,
//!   `READ,nnnn` and `IDLE,nnnn`). None was compared with a real one.
//! - USB ids: 1ab1:0588, 04b0, 04b1, 0515 and 0957:0588 are in libsigrok's table of
//!   models that lie about remote control, for those series.
//!
//! Guesses, from memory and from no source, which no test should lean on:
//!
//! - The 250000 byte limit of one read on a DS1000Z.
//! - 1ab1:04ce for the DS1000Z, which is what makes it the model that takes remote
//!   control here; 1ab1:0588 for the VS5102, on the DS1000's table entry; 0957:1755
//!   for the MSO7034A, chosen only to stay off the DSO1000's entry and the remote
//!   table, as a real MSO7000 is not on it.
//! - Every firmware string but the DS1102E's, and the serial numbers.
//! - The bus speeds. And a real scope may have an interrupt endpoint, which the shell
//!   has none of.
//!
//! Emulation artifacts:
//!
//! - Numbers answer as they were set; a real scope reformats them and rounds them
//!   to what it can do. Only `:CHANn:PROB` changes another value, `:CHANn:SCAL`.
//! - A triangle per channel, with no noise, no trigger and no timebase in it.
//! - No digital capture: a model with a pod says all its digital channels are off
//!   and flags `:WAV:SOUR LA`, `:WAV:SOUR Dn` and `:WAV:DATA? DIG`.
//!
//! Options, besides the shell's ([`scpi::Config::option`]):
//!
//! - `model=NAME`: `DS1102E`, `VS5102`, `DS2072`, `DSO1012A`, `DSO1052B`,
//!   `DS1054Z`, `DS4024`, `MSO7034A`, `MSO5074`;
//! - `firmware=TEXT`, and `00.02.03` on a DS1102E gets bare bytes for its blocks;
//! - `trigger=A,B,...`: what `:TRIG:STAT?` plays while running, in turn;
//! - `busy=N`: `:WAV:STAT?` says `READ` N times, then `IDLE`;
//! - `block=N`: a memory read gives at most N bytes per `:WAV:DATA?`;
//! - `short-at=N`: the Nth `:WAV:DATA?` gives half of what it should;
//! - `single-fail=N`: the first N `*ESR?` say the single shot failed;
//! - `frames=N`: the segments in memory (default 2);
//! - `depth=N`: `ACQ:MDEP?` (default 1400).
//!
//! The pedantic checks: only what the model has; `*OPC?` on a DS1000; `:ACQ:MDEP`
//! while stopped, which libsigrok's driver says the DS2000 does not take; a read
//! of the waveform with no `:WAV:SOUR`; a read over the DS1000Z's limit.

use crate::model::{DeviceModel, Speed, Violations};
use crate::options::parse_number;
use crate::shells::scpi::{self, Answer, Instrument, Message, Settings, block};
use crate::shells::tcp::Stream;
use crate::shells::usbtmc::Identity;
use crate::txlog::{HASH_ABOVE, Rules};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "rigol-ds";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Rigol and Agilent scopes on USBTMC and TCP; model=, firmware=, trigger=, \
                         busy=, block=, short-at=, single-fail=, frames=, depth=, and the SCPI \
                         shell's mute-at=, silent-at=, late-at=, opc-late=, mark-at=, idn=, eol=, \
                         framing=, stall=, in-stall=, empty-at=, bad-tag-at=, message=";

/// What a read of one `:WAV:DATA?` takes on a DS1000Z, in bytes.
const READ_LIMIT: usize = 250_000;

/// A series, as far as it shows on the wire.
struct Kind {
    vendor: &'static str,
    name: &'static str,
    /// libsigrok's protocol generation.
    protocol: u8,
    analog: u8,
    digital: bool,
    usb: (u16, u16),
    speed: Speed,
    firmware: &'static str,
    /// Samples of a live frame.
    live: usize,
    /// The digits of a block's length; 0 for bare bytes.
    digits: usize,
    /// `:CHANn:PROB?` says `10X`, not `1.000e+01`.
    probe_x: bool,
}

const RIGOL: &str = "Rigol Technologies";
const AGILENT: &str = "Agilent Technologies";

#[rustfmt::skip]
const KINDS: &[Kind] = &[
    Kind { vendor: RIGOL, name: "DS1102E", protocol: 2, analog: 2, digital: false, usb: (0x1ab1, 0x0588), speed: Speed::Full, firmware: "00.02.06.00.01", live: 600, digits: 8, probe_x: false },
    Kind { vendor: RIGOL, name: "VS5102", protocol: 1, analog: 2, digital: false, usb: (0x1ab1, 0x0588), speed: Speed::Full, firmware: "01.00.00", live: 2048, digits: 0, probe_x: false },
    Kind { vendor: RIGOL, name: "DS2072", protocol: 3, analog: 2, digital: false, usb: (0x1ab1, 0x04b0), speed: Speed::High, firmware: "01.01.00.02", live: 1400, digits: 9, probe_x: false },
    Kind { vendor: AGILENT, name: "DSO1012A", protocol: 3, analog: 2, digital: false, usb: (0x0957, 0x0588), speed: Speed::Full, firmware: "00.04.02", live: 600, digits: 9, probe_x: false },
    Kind { vendor: AGILENT, name: "DSO1052B", protocol: 3, analog: 2, digital: false, usb: (0x0957, 0x0588), speed: Speed::Full, firmware: "00.04.02", live: 600, digits: 9, probe_x: true },
    Kind { vendor: RIGOL, name: "DS1054Z", protocol: 4, analog: 4, digital: false, usb: (0x1ab1, 0x04ce), speed: Speed::High, firmware: "00.04.04.04.03", live: 1200, digits: 9, probe_x: false },
    Kind { vendor: RIGOL, name: "DS4024", protocol: 4, analog: 4, digital: false, usb: (0x1ab1, 0x04b1), speed: Speed::High, firmware: "00.02.03", live: 1400, digits: 9, probe_x: false },
    Kind { vendor: AGILENT, name: "MSO7034A", protocol: 4, analog: 4, digital: false, usb: (0x0957, 0x1755), speed: Speed::High, firmware: "06.20.0002", live: 1000, digits: 9, probe_x: false },
    Kind { vendor: RIGOL, name: "MSO5074", protocol: 5, analog: 4, digital: true, usb: (0x1ab1, 0x0515), speed: Speed::High, firmware: "00.01.03.03.03", live: 1000, digits: 9, probe_x: false },
];

/// A scope.
pub(super) struct Scope {
    kind: &'static Kind,
    firmware: String,
    /// It has a block header. A DS1000 before 0.2.4 and a VS5000 have none.
    framed: bool,
    trigger: Vec<String>,
    trigger_at: usize,
    running: bool,
    source: Option<u8>,
    /// How far a memory read has come.
    position: usize,
    busy: u32,
    block: usize,
    short_at: Option<u32>,
    reads: u32,
    single_fail: u32,
    frames: u32,
    depth: u32,
}

impl Scope {
    /// Answers with a query's value and flags it on a series that lacks it.
    fn only(&self, protocols: &[u8], m: &Message<'_>, v: &mut Violations, answer: Answer) -> Answer {
        if protocols.contains(&self.kind.protocol) {
            answer
        } else {
            v.report(format!("{} on protocol {}", m.text, self.kind.protocol));
            Answer::Silent
        }
    }

    /// How many bytes of the channel the read gets, and from where.
    fn span(&mut self, s: &Settings) -> (usize, usize) {
        let frame = match s.get("WAV:MODE").unwrap_or("NORM") {
            "RAW" => s.number("WAV:POIN") as usize,
            _ => self.kind.live,
        };
        if self.kind.protocol >= 4 {
            let start = (s.number("WAV:START") as usize).max(1);
            let stop = (s.number("WAV:STOP") as usize).max(start);
            (start - 1, stop - start + 1)
        } else if self.kind.protocol == 3 && s.get("WAV:MODE") == Some("RAW") {
            let from = self.position.min(frame);
            let n = (frame - from).min(self.block);
            self.position = from + n;
            (from, n)
        } else {
            (0, frame.min(self.kind.live))
        }
    }

    fn wave(channel: u8, from: usize, n: usize) -> Vec<u8> {
        (from..from + n)
            .map(|i| {
                let t = (i * 3 + usize::from(channel) * 37) % 400;
                let up = if t < 200 { t } else { 400 - t };
                28 + up as u8
            })
            .collect()
    }

    /// `:WAV:DATA?`.
    fn data(&mut self, m: &Message<'_>, s: &Settings, v: &mut Violations) -> Answer {
        let channel = if self.kind.protocol <= 2 {
            m.args.strip_prefix("CHAN").and_then(|c| c.parse().ok())
        } else {
            self.source
        };
        let Some(channel) = channel.filter(|c| (1..=self.kind.analog).contains(c)) else {
            v.report(format!("{}: the read of no channel this model has", m.text));
            return Answer::Silent;
        };
        let (from, mut n) = self.span(s);
        if n > READ_LIMIT {
            v.report(format!("{}: {n} bytes at a time, over {READ_LIMIT}", m.text));
            n = READ_LIMIT;
        }
        self.reads += 1;
        if self.short_at == Some(self.reads) {
            n /= 2;
        }
        let wave = Scope::wave(channel, from, n);
        if !self.framed {
            return Answer::Bytes(wave);
        }
        let end: &[u8] = if self.kind.protocol >= 3 { b"\n" } else { b"" };
        Answer::Bytes(block(self.kind.digits, &wave, end))
    }

    fn trigger_status(&mut self) -> Answer {
        if !self.running {
            return Answer::Text("STOP".to_owned());
        }
        let status = self.trigger[self.trigger_at % self.trigger.len()].clone();
        self.trigger_at += 1;
        Answer::Text(status)
    }
}

impl Instrument for Scope {
    fn idn(&self) -> Option<String> {
        Some(format!("{},{},EMU0001,{}", self.kind.vendor, self.kind.name, self.firmware))
    }

    fn settings(&self) -> Settings {
        let mut s = Settings::default()
            .with("TIM:SCAL", "1.000e-03")
            .with("TIM:OFFS", "0.000e+00")
            .with("TIM:POS", "0.000e+00")
            .with("TRIG:EDGE:SOUR", "CHAN1")
            .with("TRIG:EDGE:SLOP", "POSITIVE")
            .with("TRIG:EDGE:LEV", "0.000e+00")
            .with("TRIG:EDGE:SWE", "AUTO")
            .with("TRIG:MODE", "EDGE")
            .with("ACQ:MEMD", "NORMAL")
            .with("ACQ:MDEP", self.depth.to_string())
            .with("WAV:POIN:MODE", "NORMAL")
            .with("WAV:POIN", "0")
            .with("WAV:FORM", "BYTE")
            .with("WAV:MODE", "NORM")
            .with("WAV:START", "1")
            .with("WAV:STOP", "1")
            .with("WAV:YOR", "0")
            .with("WAV:YREF", "127")
            .with("WAV:XINC", "1.000e-06")
            .with("FUNC:WREP:FCUR", "1")
            .with("FUNC:WREP:FMAX", self.frames.to_string())
            .with("FUNC:WREP:FEND", self.frames.to_string());
        for c in 1..=self.kind.analog {
            s.set(&format!("CHAN{c}:DISP"), "1");
            let probe = if self.kind.probe_x { "1X" } else { "1.000e+00" };
            s.set(&format!("CHAN{c}:PROB"), probe);
            s.set(&format!("CHAN{c}:SCAL"), "1.000e+00");
            s.set(&format!("CHAN{c}:OFFS"), "0.000e+00");
            s.set(&format!("CHAN{c}:COUP"), "DC");
        }
        if self.kind.digital {
            s.set("LA:STAT", "0");
        }
        s
    }

    fn command(&mut self, m: &Message<'_>, s: &mut Settings, v: &mut Violations) -> Answer {
        let v3 = [3, 4, 5];
        match m.header.as_str() {
            "*OPC?" if self.kind.protocol == 2 => {
                v.report("*OPC? on a series where it does not work");
                Answer::Silent
            }
            "*ESR?" if self.single_fail > 0 => {
                self.single_fail -= 1;
                Answer::Text("16".to_owned())
            }
            "RUN" => {
                self.running = true;
                self.trigger_at = 0;
                Answer::Done
            }
            "STOP" => {
                self.running = false;
                Answer::Done
            }
            "SING" => {
                self.running = false;
                self.only(&[3], m, v, Answer::Done)
            }
            "KEY:LOCK" => self.only(&[1, 2], m, v, Answer::Done),
            "ACQ:MDEP" if !self.running => {
                v.report(format!("{}: the depth is set only while it runs", m.text));
                Answer::Done
            }
            "TRIG:STAT?" => self.trigger_status(),
            "WAV:YINC?" => {
                let channel = self.source.unwrap_or(1);
                let scale = s.number(&format!("CHAN{channel}:SCAL"));
                self.only(&v3, m, v, Answer::Text(format!("{:.3e}", scale / 25.0)))
            }
            "WAV:SOUR" => {
                self.source = m.args.strip_prefix("CHAN").and_then(|c| c.parse().ok());
                if self.source.is_none_or(|c| !(1..=self.kind.analog).contains(&c)) {
                    v.report(format!("{}: no such source on this model", m.text));
                }
                self.only(&v3, m, v, Answer::Done)
            }
            "WAV:RES" => {
                self.position = 0;
                self.only(&[3, 4, 5], m, v, Answer::Done)
            }
            "WAV:BEG" => self.only(&[3, 4, 5], m, v, Answer::Done),
            "WAV:END" => self.only(&[3], m, v, Answer::Done),
            "WAV:STAT?" => {
                let word = if self.busy > 0 {
                    self.busy -= 1;
                    "READ"
                } else {
                    "IDLE"
                };
                self.only(&[3], m, v, Answer::Text(format!("{word},{}", self.kind.live)))
            }
            "WAV:DATA?" => self.data(m, s, v),
            "REC:CURR" => {
                let frame = m.args.parse().unwrap_or(1u32).min(self.frames);
                s.set("REC:CURR", frame.to_string());
                self.only(&[5], m, v, Answer::Done)
            }
            "REC:CURR?" => {
                self.only(&[5], m, v, Answer::Text(s.get("REC:CURR").unwrap_or("1").to_owned()))
            }
            // MSO5000: the channel goes after the question mark.
            "LA:DISP?" if self.kind.protocol == 5 => Answer::Text("0".to_owned()),
            header if header.starts_with("CHAN") && header.ends_with(":PROB") => {
                self.set_probe(m, s);
                Answer::Done
            }
            _ => Answer::Unknown,
        }
    }
}

impl Scope {
    /// The probe scales what a division means.
    fn set_probe(&self, m: &Message<'_>, s: &mut Settings) {
        let name = m.name();
        let scal = name.replace("PROB", "SCAL");
        let old = s.get(name).unwrap_or("1").trim_end_matches('X').parse().unwrap_or(1.0_f64);
        let new = m.args.parse().unwrap_or(1.0_f64);
        let scale = s.number(&scal) * new / old;
        s.set(&scal, format!("{scale:.3e}"));
        let probe = if self.kind.probe_x { format!("{new}X") } else { format!("{new:.3e}") };
        s.set(name, probe);
    }
}

/// The scope and the shell's options, whichever carrier they go on.
pub(super) fn parts(options: &[(String, String)]) -> Result<(Scope, scpi::Config)> {
    let mut config = scpi::Config::default();
    let mut kind = &KINDS[0];
    let mut firmware = None;
    let mut trigger = vec!["RUN".to_owned(), "T'D".to_owned()];
    let (mut busy, mut single_fail, mut frames, mut depth) = (0, 0, 2, 1400);
    let (mut chunk, mut short_at) = (usize::MAX, None);
    let mut eol = None;
    for (k, v) in options {
        if k == "eol" {
            eol = Some(v);
        }
        if config.option(k, v)? {
            continue;
        }
        let number = |range| parse_number(k, v, range);
        match k.as_str() {
            "model" => {
                kind = KINDS
                    .iter()
                    .find(|m| m.name == v)
                    .ok_or_else(|| Error::Invalid(format!("model={v}: no such model here")))?;
            }
            "firmware" => firmware = Some(v.clone()),
            "trigger" => trigger = v.split(',').map(str::to_owned).collect(),
            "busy" => busy = number(0..=1000)?,
            "block" => chunk = number(1..=1 << 24)? as usize,
            "short-at" => short_at = Some(number(1..=1 << 24)?),
            "single-fail" => single_fail = number(0..=1000)?,
            "frames" => frames = number(0..=1000)?,
            "depth" => depth = number(1..=1 << 24)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    // A DS1000 and a VS5000 end no text, the others end it with a line feed.
    if kind.protocol <= 2 && eol.is_none() {
        config = config.with_eol(b"");
    }
    let firmware = firmware.unwrap_or_else(|| kind.firmware.to_owned());
    let has_header = kind.digits != 0 && (kind.protocol != 2 || firmware.as_str() >= "00.02.04");
    let scope = Scope {
        kind,
        firmware,
        framed: has_header,
        trigger,
        trigger_at: 0,
        running: false,
        source: None,
        position: 0,
        busy,
        block: chunk,
        short_at,
        reads: 0,
        single_fail,
        frames,
        depth,
    };
    Ok((scope, config))
}

/// A scope on USBTMC, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (scope, config) = parts(options)?;
    let (vendor, product) = scope.kind.usb;
    let mut id = Identity::new(vendor, product, scope.kind.vendor, scope.kind.name);
    id.speed = scope.kind.speed;
    // They say they take remote control. The tools' table says not to believe it for
    // the ids it lists, and for any other it goes on the wire: REN_CONTROL and
    // LOCAL_LOCKOUT at each open, GO_TO_LOCAL at each close.
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

    #[test]
    fn a_ds1102e_answers_as_the_recording_does() {
        let mut s = on_socket(&[]);
        assert_eq!(ask(&mut s, "*IDN?\n").0, b"Rigol Technologies,DS1102E,EMU0001,00.02.06.00.01");
        assert_eq!(ask(&mut s, ":TIM:SCAL?\n").0, b"1.000e-03");
        let (block, flags) = ask(&mut s, ":WAV:DATA? CHAN1\n");
        assert_eq!(&block[..10], b"#800000600");
        assert_eq!(block.len(), 610, "no line end");
        assert_eq!(flags, Vec::<String>::new());
    }

    #[test]
    fn old_firmware_has_no_block_header() {
        let mut s = on_socket(&[("firmware", "00.02.03")]);
        assert_eq!(ask(&mut s, ":WAV:DATA? CHAN2\n").0.len(), 600);
    }

    #[test]
    fn a_ds2072_ends_its_block_and_wants_a_source() {
        let mut s = on_socket(&[("model", "DS2072")]);
        assert_eq!(ask(&mut s, "*OPC?\n").0, b"1\n");
        let (nothing, flags) = ask(&mut s, ":WAV:DATA?\n");
        assert_eq!(nothing, Vec::<u8>::new());
        assert_eq!(flags.len(), 1, "no source");
        ask(&mut s, ":WAV:SOUR CHAN2\n");
        let (block, flags) = ask(&mut s, ":WAV:DATA?\n");
        assert_eq!(&block[..11], b"#9000001400");
        assert_eq!(block.last(), Some(&b'\n'));
        assert_eq!(flags, Vec::<String>::new());
    }

    #[test]
    fn what_the_series_lacks_is_flagged() {
        let mut s = on_socket(&[]);
        assert_eq!(ask(&mut s, "*OPC?\n").1.len(), 1, "on a DS1000");
        assert_eq!(ask(&mut s, ":CHAN3:DISP?\n").1.len(), 1, "two channels");
        assert_eq!(ask(&mut s, ":WAV:BEG\n").1.len(), 1);
        let mut s = on_socket(&[("model", "DS2072")]);
        assert_eq!(ask(&mut s, ":ACQ:MDEP 1400\n").1.len(), 1, "stopped");
        ask(&mut s, ":RUN\n");
        assert_eq!(ask(&mut s, ":ACQ:MDEP 1400\n").1, Vec::<String>::new(), "running");
        assert_eq!(ask(&mut s, ":KEY:LOCK DISABLE\n").1.len(), 1);
    }

    #[test]
    fn a_ds1000z_reads_its_window_and_keeps_to_its_limit() {
        let mut s = on_socket(&[("model", "DS1054Z")]);
        ask(&mut s, ":WAV:SOUR CHAN1\n");
        ask(&mut s, ":WAV:START 1\n:WAV:STOP 30000\n");
        assert_eq!(ask(&mut s, ":WAV:DATA?\n").0.len(), 11 + 30000 + 1);
        ask(&mut s, ":WAV:STOP 1000000\n");
        let (block, flags) = ask(&mut s, ":WAV:DATA?\n");
        assert_eq!(block.len(), 11 + READ_LIMIT + 1, "a host-sent count makes no allocation");
        assert_eq!(flags.len(), 1);
    }

    #[test]
    fn the_trigger_goes_as_the_options_say() {
        let mut s = on_socket(&[("trigger", "WAIT")]);
        assert_eq!(ask(&mut s, ":TRIG:STAT?\n").0, b"STOP", "not running");
        ask(&mut s, ":RUN\n");
        assert_eq!(ask(&mut s, ":TRIG:STAT?\n").0, b"WAIT");
        assert_eq!(ask(&mut s, ":TRIG:STAT?\n").0, b"WAIT");
        let mut s = on_socket(&[]);
        ask(&mut s, ":RUN\n");
        assert_eq!(ask(&mut s, ":TRIG:STAT?\n").0, b"RUN");
        assert_eq!(ask(&mut s, ":TRIG:STAT?\n").0, b"T'D");
    }

    #[test]
    fn a_setting_changes_state_and_a_probe_scales() {
        let mut s = on_socket(&[("model", "DS2072")]);
        ask(&mut s, ":CHAN1:COUP AC\n");
        assert_eq!(ask(&mut s, ":CHAN1:COUP?\n").0, b"AC\n");
        ask(&mut s, ":CHAN1:PROB 10\n");
        assert_eq!(ask(&mut s, ":CHAN1:SCAL?\n").0, b"1.000e1\n");
        assert_eq!(ask(&mut s, ":CHAN1:PROB?\n").0, b"1.000e1\n");
        let mut s = on_socket(&[("model", "DSO1052B")]);
        ask(&mut s, ":CHAN1:PROB 10\n");
        assert_eq!(ask(&mut s, ":CHAN1:PROB?\n").0, b"10X\n");
    }

    #[test]
    fn the_faults_of_the_model() {
        let mut s = on_socket(&[("model", "DS2072"), ("single-fail", "1"), ("busy", "1")]);
        assert_eq!(ask(&mut s, "*ESR?\n").0, b"16\n");
        assert_eq!(ask(&mut s, "*ESR?\n").0, b"0\n");
        assert_eq!(ask(&mut s, ":WAV:STAT?\n").0, b"READ,1400\n");
        assert_eq!(ask(&mut s, ":WAV:STAT?\n").0, b"IDLE,1400\n");
        let mut s = on_socket(&[("short-at", "1")]);
        assert_eq!(ask(&mut s, ":WAV:DATA? CHAN1\n").0.len(), 10 + 300);
        assert_eq!(ask(&mut s, ":WAV:DATA? CHAN1\n").0.len(), 10 + 600);
    }

    #[test]
    fn it_answers_its_own_options_on_usb_too() {
        for kind in KINDS {
            let options = [("model".to_owned(), kind.name.to_owned())];
            let m = build(&options).unwrap();
            assert_eq!(m.descriptors().vendor_id(), kind.usb.0);
            assert_eq!(m.descriptors().product_id(), kind.usb.1);
        }
        assert!(build(&[("model".to_owned(), "DS9".to_owned())]).is_err());
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
    }
}
