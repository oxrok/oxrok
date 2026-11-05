// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A handheld Agilent or Keysight U12xx meter behind its IR cable, on the
//! CDC-ACM shell. A query is ASCII and ends in LF or CR LF; an answer ends in
//! CR LF. A meter also speaks unasked: `*N` when the rotary switch moves, `*E`
//! for a command it did not take.
//!
//! The device, and how sure this is:
//!
//! - The cable: the U1173A and U1173B are USB to serial adapters and the
//!   U1177A is a Bluetooth one, so a real one is a `ttyUSBn` or `rfcomm` and
//!   the meter sits behind a UART. This model gives it a CDC-ACM cable, which
//!   shows the same line settings to the tools. The ids and strings are not a
//!   real cable's.
//! - "U1281A/U1282A Programming Guide" (Keysight; a copy at docs.ampnuts.ru):
//!   9600/8n1, answers ended in CR LF, `*IDN?` as
//!   `Keysight Technologies,U1282A,DPQ1007000,V1.00`, `CONF? [@2]`, `FETC?
//!   [@2|@3]` with `@3` the environment temperature, `+1.23475000E+00` for a
//!   value, the 21 letters of `STAT?`, `*0` to `*9` for the switch with `*9`
//!   the square wave output, `E` for a command error. The guide does not list
//!   the `LOG:` commands.
//! - The sigrok wiki's page on the series (`Agilent_U1231A`): more `*IDN?`
//!   answers, `"V,0,AC"` on the U123x, `"VOLT:AC +1.000000E+00,+1.000000E-04"`
//!   on the U124x and U125x, overload as `+9.90000000E+37`, `LOG:HAND N` and
//!   its kin with 14 digit entries on the U124xC and U128x, `*E` for an index
//!   out of range, and 19200 Bd on some U127x.
//! - That the guide's `CONF?` line is in double quotes on a U128x comes from the
//!   wiki's on other models. No recording was found of any meter.
//!
//! Emulation artifacts:
//!
//! - It answers at once. The guide asks for 200 ms between commands, which no
//!   tool keeps, so that is accepted: the driver polls faster than that with
//!   several channels, and a real meter's answer to it is not known.
//! - `id=`, `conf=`, `conf2=`, `stat=`, `fetc=`, `fetc2=`, `fetc3=` and `log=`
//!   play whole answers, CR LF included, as given. A real meter's come from its
//!   display and its memory; these stand in for a dial that turns, a `*N` ahead
//!   of an answer, and a line gone wrong.
//!
//! Not modelled: `*RST`, `SYST:BATT?`, `TRIG:SOUR` and the rest of what a real
//! meter takes, which no path of the driver sends, so they are flagged; the
//! guide's Xon and Xoff bytes (0x11, 0x13), which it lists with the prompts;
//! the U125x's `LOG? H1`, as the driver has no log on it; a real meter's `*E`
//! to a query it cannot answer, other than `FETC?` in square wave mode (the
//! driver's comment) and as a played answer.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `model=` any of the [`PROFILES`] (default U1253B), the identity, the
//!   channels and the answers;
//! - `uart=LINE|native`, the line instead of 9600/8n1;
//! - `id=HEX[,HEX...]`, answers to `*IDN?` played in turn, or `id=none` for no
//!   answer and the event point;
//! - `conf=`, `conf2=`, `stat=`, `fetc=`, `fetc2=`, `fetc3=`, each `HEX[,HEX...]`:
//!   answers to `CONF?`, `CONF? @2`, `STAT?`, `FETC?`, `FETC? @2` and `FETC? @3`
//!   played in turn;
//! - `log=HEX[,HEX...]|none`: the entries of the log, whole answers, by index
//!   from 1; an index past the last answers `*E`;
//! - `answers=N`, queries other than `*IDN?` answered per power on, then
//!   silence and the event point.
//!
//! The pedantic checks: each query with the line end the driver gives it,
//! `*IDN?` with CR LF and any other with LF and then CR LF (a C habit, the
//! second line an empty one); nothing else but those queries; no query before an
//! `*IDN?`, and no `*IDN?` after one; `@2` and `@3` only on a meter with that
//! many displays, and never `CONF? @3`; `LOG:` only on a meter that has it, on
//! one source, its index the last one or the next from 1, and never with a live
//! query in one session; on a U128x no `FETC?` straight after a `CONF` answer of
//! `"SQU"`;
//! nothing left of a command when a tool opens.
//!
//! What the scenario's cases leave unreached of the C driver: the scan's write
//! going short (`api.c:135-136`), as the tty takes 7 bytes whatever the meter
//! does; the `default:` of `config_get`, `config_set` and `config_list`
//! (`api.c:210-211`, `243-244`, `263-264`), which the core's key checks stop
//! first; the two null checks of `agdmm_receive_data` (`protocol.c:180`, `183`);
//! `agdmm_send`'s `*IDN?` (`protocol.c:223`), which no job sends; and the failed
//! float of `recv_fetc` (`protocol.c:474-476`), which its tables' form leaves no
//! way to.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "agilent-dmm";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "An Agilent or Keysight U12xx meter behind an IR cable on CDC-ACM; \
                         model=, uart=, id=HEX|none, conf=, conf2=, stat=, fetc=, fetc2=, \
                         fetc3=, log=HEX|none, answers=, and the shell's stall=, chunk=, \
                         in-limit=, in-stall=, out-hold=";

/// The longest answer an option may play: past the driver's 255 byte line.
const LONGEST_ANSWER: usize = 1024;
/// The longest command the driver sends is `LOG:HAND` and an index of an int,
/// which `agdmm_send`'s buffer cuts to 28 bytes.
const LONGEST_COMMAND: usize = 32;
const ERROR_PROMPT: &[u8] = b"*E\r\n";

/// How a meter says what it measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Family {
    /// U123x, U127x: `"V,0,AC"`.
    Fields,
    /// U124x, U125x: `"VOLT:AC +1.000000E+00,+1.000000E-04"`.
    Six,
    /// U124xC, U128x: the same with eight decimals, and a log.
    Eight,
}

/// One meter of the series.
#[derive(Clone, Copy, Debug)]
struct Profile {
    name: &'static str,
    identity: &'static str,
    displays: u8,
    family: Family,
}

macro_rules! profile {
    ($name:literal, $identity:literal, $displays:literal, $family:ident) => {
        Profile {
            name: $name,
            identity: concat!($identity, "\r\n"),
            displays: $displays,
            family: Family::$family,
        }
    };
}

/// The series. The firmware versions of U1232A, U1253B, U1272A, U1242C and
/// U1282A are ones the sources show; the others are made up to look like them.
const PROFILES: &[Profile] = &[
    profile!("U1231A", "Agilent Technologies,U1231A,MY52020135,V1.00", 1, Fields),
    profile!("U1232A", "Agilent Technologies,U1232A,MY52020136,V1.00", 1, Fields),
    profile!("U1233A", "Agilent Technologies,U1233A,MY52020137,V1.00", 1, Fields),
    profile!("U1241A", "Agilent Technologies,U1241A,MY50000001,V1.00", 2, Six),
    profile!("U1242A", "Agilent Technologies,U1242A,MY50000002,V1.00", 2, Six),
    profile!("U1241B", "Agilent Technologies,U1241B,MY50000003,V1.00", 2, Six),
    profile!("U1242B", "Agilent Technologies,U1242B,MY50000004,V1.00", 2, Six),
    profile!("U1241C", "Keysight Technologies,U1241C,MY50000005,V1.20", 2, Eight),
    profile!("U1242C", "Keysight Technologies,U1242C,MY50000006,V1.20", 2, Eight),
    profile!("U1251A", "Agilent Technologies,U1251A,MY50000007,V2.26", 3, Six),
    profile!("U1252A", "Agilent Technologies,U1252A,MY50000008,V2.26", 3, Six),
    profile!("U1253A", "Agilent Technologies,U1253A,MY50000009,V2.26", 3, Six),
    profile!("U1251B", "Agilent Technologies,U1251B,MY50000010,V2.26", 3, Six),
    profile!("U1252B", "Agilent Technologies,U1252B,MY50000011,V2.26", 3, Six),
    profile!("U1253B", "Agilent Technologies,U1253B,MY50000012,V2.26", 3, Six),
    profile!("U1271A", "Agilent Technologies,U1271A,MY50000013,V2.04", 3, Fields),
    profile!("U1272A", "Agilent Technologies,U1272A,MY50000014,V2.04", 3, Fields),
    profile!("U1273A", "Agilent Technologies,U1273A,MY50000015,V2.04", 3, Fields),
    profile!("U1273AX", "Agilent Technologies,U1273AX,MY50000016,V2.04", 3, Fields),
    profile!("U1281A", "Keysight Technologies,U1281A,DPQ1007000,V1.00", 3, Eight),
    profile!("U1282A", "Keysight Technologies,U1282A,DPQ1007000,V1.00", 3, Eight),
];

/// A meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn answers_list(key: &str, value: &str) -> Result<Packets> {
    Ok(Packets::new(parse_hex_list(key, value, 1, LONGEST_ANSWER)?))
}

/// One answer, for a default.
fn one(answer: &[u8]) -> Packets {
    Packets::new(vec![answer.to_vec()])
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut name = "U1253B";
    let mut uart = "9600/8n1";
    let mut id = None;
    let mut lists: [Option<Packets>; 6] = Default::default();
    let mut log = None;
    let mut answers = None;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => name = v,
            "uart" => uart = v,
            "id" if v == "none" => id = Some(None),
            "id" => id = Some(Some(answers_list(k, v)?)),
            "conf" => lists[0] = Some(answers_list(k, v)?),
            "conf2" => lists[1] = Some(answers_list(k, v)?),
            "stat" => lists[2] = Some(answers_list(k, v)?),
            "fetc" => lists[3] = Some(answers_list(k, v)?),
            "fetc2" => lists[4] = Some(answers_list(k, v)?),
            "fetc3" => lists[5] = Some(answers_list(k, v)?),
            "log" if v == "none" => log = Some(Vec::new()),
            "log" => log = Some(parse_hex_list(k, v, 1, LONGEST_ANSWER)?),
            "answers" => answers = Some(parse_number(k, v, 0..=100_000)?),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let profile = PROFILES
        .iter()
        .find(|p| p.name == name)
        .ok_or_else(|| Error::Invalid(format!("model={name}: not of the series")))?;
    let [conf, conf2, stat, fetc, fetc2, fetc3] = lists;
    let config = Config {
        profile: *profile,
        id: id.unwrap_or_else(|| Some(one(profile.identity.as_bytes()))),
        conf: conf.unwrap_or_else(|| one(default_conf(profile.family))),
        conf2: conf2.unwrap_or_else(|| one(default_conf2(profile))),
        // The example in the wiki's page.
        stat: stat.unwrap_or_else(|| one(b"\"000000000110L00000000\"\r\n")),
        // The guide's examples, the last a temperature in its range.
        fetc: fetc.unwrap_or_else(|| one(b"+1.23475000E+00\r\n")),
        fetc2: fetc2.unwrap_or_else(|| one(b"+6.00000000E+01\r\n")),
        fetc3: fetc3.unwrap_or_else(|| one(b"+2.25000000E+01\r\n")),
        log: log.unwrap_or_else(|| default_log(profile.family)),
        answers,
    };
    let cable = Identity::new(0x1209, 0x0001, "oxrok-emu", "Agilent IR cable");
    let cdc = Cdc::new(NAME, &cable, faults, Meter::new(config));
    Ok(match uart {
        "native" => cdc,
        line => cdc.behind_uart(Line::parse(line)?),
    })
}

fn default_conf(family: Family) -> &'static [u8] {
    match family {
        Family::Fields => b"\"V,0,AC\"\r\n",
        Family::Six => b"\"VOLT:AC +1.000000E+00,+1.000000E-04\"\r\n",
        Family::Eight => b"\"VOLT:AC +6.00000000E+01,+1.00000000E-03\"\r\n",
    }
}

/// A meter with two displays has the temperature on its second, which the
/// driver knows and asks about all the same.
fn default_conf2(profile: &Profile) -> &'static [u8] {
    if profile.displays == 2 {
        return b"\"TEMP:K CEL\"\r\n";
    }
    match profile.family {
        Family::Fields => b"\"FREQ,0,AC\"\r\n",
        Family::Six => b"\"FREQ +1.000000E+03,+1.000000E-01\"\r\n",
        Family::Eight => b"\"FREQ +1.00000000E+03,+1.00000000E-01\"\r\n",
    }
}

/// The entries the wiki shows, under each family that has a log.
fn default_log(family: Family) -> Vec<Vec<u8>> {
    let entry: &[u8] = match family {
        Family::Eight => b"\"01000241100100\"\r\n",
        Family::Fields | Family::Six => return Vec::new(),
    };
    vec![entry.to_vec(), b"\"00023521000000\"\r\n".to_vec(), entry.to_vec()]
}

#[derive(Clone, Debug)]
struct Config {
    profile: Profile,
    id: Option<Packets>,
    conf: Packets,
    conf2: Packets,
    stat: Packets,
    fetc: Packets,
    fetc2: Packets,
    fetc3: Packets,
    log: Vec<Vec<u8>>,
    answers: Option<u32>,
}

/// What came of a capture's kind of query in this session.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Session {
    Idle,
    Live,
    /// The log's source, by its place in [`SOURCES`], and the last index.
    Log(usize, u64),
}

const SOURCES: [&str; 4] = ["LOG:HAND", "LOG:TRIG", "LOG:AUTO", "LOG:EXPO"];

struct Meter {
    config: Config,
    id: Option<Packets>,
    conf: Packets,
    conf2: Packets,
    stat: Packets,
    fetc: Packets,
    fetc2: Packets,
    fetc3: Packets,
    /// What has come of a command so far.
    pending: Vec<u8>,
    /// A command with a bare LF came, so an empty line is due.
    blank_due: bool,
    identified: bool,
    session: Session,
    /// The last `CONF` answer, of any display, was `"SQU"`.
    square: bool,
    /// The last `CONF?` answer, of the first display, was `"SQU"`.
    square_output: bool,
    answered: u32,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter {
            id: config.id.clone(),
            conf: config.conf.clone(),
            conf2: config.conf2.clone(),
            stat: config.stat.clone(),
            fetc: config.fetc.clone(),
            fetc2: config.fetc2.clone(),
            fetc3: config.fetc3.clone(),
            config,
            pending: Vec::new(),
            blank_due: false,
            identified: false,
            session: Session::Idle,
            square: false,
            square_output: false,
            answered: 0,
        }
    }

    /// Whether a query gets its answer.
    fn answers_left(&mut self, port: &mut Port<'_>) -> bool {
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return false;
        }
        self.answered += 1;
        true
    }

    fn identify(&mut self, port: &mut Port<'_>) {
        if self.session != Session::Idle {
            port.violation("*IDN? after a query");
        }
        self.identified = true;
        match &mut self.id {
            Some(list) => port.send(&list.take()),
            None => port.mark(),
        }
    }

    /// The checks every query but `*IDN?` shares.
    fn before_query(&mut self, port: &mut Port<'_>) {
        if !self.identified {
            port.violation("a query before an *IDN?");
        }
    }

    fn live(&mut self, port: &mut Port<'_>) {
        self.before_query(port);
        if matches!(self.session, Session::Log(..)) {
            port.violation("a live query in a session of log queries");
        }
        self.session = Session::Live;
    }

    fn displays(&self, needed: u8, command: &str, port: &mut Port<'_>) {
        if self.config.profile.displays < needed {
            port.violation(format!(
                "{command} on a {} with {} displays",
                self.config.profile.name, self.config.profile.displays
            ));
        }
    }

    /// The U128x alone has the position, and the driver knows it only there.
    fn has_square_output(&self) -> bool {
        self.config.profile.name.starts_with("U128")
    }

    fn conf(&mut self, display: u8, port: &mut Port<'_>) {
        self.live(port);
        if !self.answers_left(port) {
            return;
        }
        let answer = if display == 1 { self.conf.take() } else { self.conf2.take() };
        self.square = answer.starts_with(b"\"SQU\"");
        if display == 1 {
            self.square_output = self.square;
        }
        port.send(&answer);
    }

    fn fetc(&mut self, display: u8, port: &mut Port<'_>) {
        self.live(port);
        if self.has_square_output() && self.square {
            port.violation("FETC? straight after a CONF answer of SQU");
        }
        if !self.answers_left(port) {
            return;
        }
        // Only the first display is known to do this: the driver's comment.
        if display == 1 && self.has_square_output() && self.square_output {
            port.send(ERROR_PROMPT);
            return;
        }
        let answer = match display {
            1 => self.fetc.take(),
            2 => self.fetc2.take(),
            _ => self.fetc3.take(),
        };
        port.send(&answer);
    }

    fn stat(&mut self, port: &mut Port<'_>) {
        self.live(port);
        if self.answers_left(port) {
            let answer = self.stat.take();
            port.send(&answer);
        }
    }

    fn log(&mut self, source: usize, index: &[u8], port: &mut Port<'_>) {
        self.before_query(port);
        if self.config.profile.family != Family::Eight {
            port.violation(format!("LOG: on a {}", self.config.profile.name));
        }
        let Some(index) = std::str::from_utf8(index).ok().and_then(|i| i.parse::<u64>().ok())
        else {
            port.violation(format!("'{}': no index", index.escape_ascii()));
            return;
        };
        let last = match self.session {
            Session::Idle => 0,
            Session::Live => {
                port.violation("a log query in a session of live queries");
                0
            }
            Session::Log(from, last) => {
                if from != source {
                    port.violation("a log query of another source");
                }
                last
            }
        };
        if index != last + 1 && !(last > 0 && index == last) {
            port.violation(format!("log index {index} after {last}"));
        }
        self.session = Session::Log(source, index);
        if self.answers_left(port) {
            let entry = index
                .checked_sub(1)
                .and_then(|i| usize::try_from(i).ok())
                .and_then(|i| self.config.log.get(i))
                .map_or(ERROR_PROMPT, Vec::as_slice);
            port.send(entry);
        }
    }

    fn command(&mut self, command: &[u8], port: &mut Port<'_>) {
        match command {
            b"CONF?" => self.conf(1, port),
            b"CONF? @2" => {
                self.displays(2, "CONF? @2", port);
                self.conf(2, port);
            }
            b"STAT?" => self.stat(port),
            b"FETC?" => self.fetc(1, port),
            b"FETC? @2" => {
                self.displays(2, "FETC? @2", port);
                self.fetc(2, port);
            }
            b"FETC? @3" => {
                self.displays(3, "FETC? @3", port);
                self.fetc(3, port);
            }
            other => {
                let log = SOURCES.iter().position(|s| {
                    other.strip_prefix(s.as_bytes()).is_some_and(|rest| rest.starts_with(b" "))
                });
                match log {
                    Some(source) => self.log(source, &other[SOURCES[source].len() + 1..], port),
                    None => {
                        port.violation(format!("'{}': no such command", other.escape_ascii()));
                    }
                }
            }
        }
    }

    /// A line up to its LF, the CR before that still on it.
    fn line(&mut self, line: &[u8], port: &mut Port<'_>) {
        if std::mem::take(&mut self.blank_due) {
            if line != b"\r" {
                port.violation(format!(
                    "'{}': no empty line after the last command",
                    line.escape_ascii()
                ));
            }
            return;
        }
        match line {
            b"*IDN?\r" => self.identify(port),
            b"*IDN?" => {
                port.violation("*IDN? without its CR");
                self.identify(port);
            }
            [.., b'\r'] | [] => {
                port.violation(format!("'{}': a line with a CR", line.escape_ascii()));
            }
            command => {
                self.blank_due = true;
                self.command(command, port);
            }
        }
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte == b'\n' {
                let line = std::mem::take(&mut self.pending);
                self.line(&line, port);
            } else if self.pending.len() < LONGEST_COMMAND {
                self.pending.push(byte);
            } else {
                port.violation(format!(
                    "'{}': a command too long",
                    self.pending.escape_ascii()
                ));
                self.pending.clear();
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() {
            if !self.pending.is_empty() {
                port.violation(format!(
                    "'{}': a command left unfinished",
                    self.pending.escape_ascii()
                ));
                self.pending.clear();
            }
            // A write that went short at a close is the tool's business.
            self.blank_due = false;
        }
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
        Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Meter>) -> usize {
        h.v.drain().len()
    }

    fn identified(options: &[(&str, &str)]) -> Host<Meter> {
        let mut h = meter(options);
        h.write(b"*IDN?\r\n");
        h.read();
        assert_eq!(violations(&mut h), 0);
        h
    }

    fn ask(h: &mut Host<Meter>, command: &str) -> Vec<u8> {
        h.write(format!("{command}\n\r\n").as_bytes());
        h.read()
    }

    #[test]
    fn each_model_has_its_identity_and_displays() {
        assert_eq!(PROFILES.len(), 21);
        for p in PROFILES {
            let mut h = meter(&[("model", p.name)]);
            h.write(b"*IDN?\r\n");
            let answer = h.read();
            let text = String::from_utf8(answer).unwrap();
            let fields: Vec<&str> = text.trim_end().split(',').collect();
            assert_eq!(fields.len(), 4, "{text}");
            assert_eq!(fields[1], p.name);
            assert_eq!(violations(&mut h), 0);
            assert!(!ask(&mut h, "FETC?").is_empty());
            assert_eq!(violations(&mut h), 0);
        }
    }

    #[test]
    fn the_guides_identity_and_values() {
        let mut h = meter(&[("model", "U1282A")]);
        h.write(b"*IDN?\r\n");
        assert_eq!(h.read(), b"Keysight Technologies,U1282A,DPQ1007000,V1.00\r\n");
        assert_eq!(ask(&mut h, "FETC?"), b"+1.23475000E+00\r\n");
        assert_eq!(ask(&mut h, "CONF?"), b"\"VOLT:AC +6.00000000E+01,+1.00000000E-03\"\r\n");
    }

    #[test]
    fn nothing_crosses_a_wrong_line() {
        let mut h = Host::open(
            function(&[("uart".to_owned(), "19200/8n1".to_owned())]).unwrap(),
            Line::parse("9600/8n1").unwrap(),
            true,
            true,
        );
        h.write(b"*IDN?\r\n");
        assert_eq!(h.read(), b"");
        let mut h = Host::open(
            function(&[("uart".to_owned(), "native".to_owned())]).unwrap(),
            Line::parse("115200/8n1").unwrap(),
            true,
            true,
        );
        h.write(b"*IDN?\r\n");
        assert!(!h.read().is_empty());
    }

    #[test]
    fn answers_play_in_turn() {
        let mut h = identified(&[("fetc", "61,62"), ("stat", "63")]);
        assert_eq!(ask(&mut h, "FETC?"), b"a");
        assert_eq!(ask(&mut h, "FETC?"), b"b");
        assert_eq!(ask(&mut h, "FETC?"), b"a");
        assert_eq!(ask(&mut h, "STAT?"), b"c");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn each_display_has_its_list() {
        let mut h = identified(&[
            ("conf", "61"),
            ("conf2", "62"),
            ("fetc", "63"),
            ("fetc2", "64"),
            ("fetc3", "65"),
        ]);
        assert_eq!(ask(&mut h, "CONF?"), b"a");
        assert_eq!(ask(&mut h, "CONF? @2"), b"b");
        assert_eq!(ask(&mut h, "FETC?"), b"c");
        assert_eq!(ask(&mut h, "FETC? @2"), b"d");
        assert_eq!(ask(&mut h, "FETC? @3"), b"e");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_command_may_come_in_pieces() {
        let mut h = identified(&[]);
        h.write(b"FE");
        h.write(b"TC?");
        assert_eq!(h.read(), b"");
        h.write(b"\n\r");
        assert_eq!(h.read(), b"+1.23475000E+00\r\n");
        h.write(b"\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn silence_marks_the_event_point() {
        let mut h = meter(&[("id", "none")]);
        h.write(b"*IDN?\r\n");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
        let mut h = identified(&[("answers", "1")]);
        assert!(!h.cdc.mark(), "*IDN? does not count");
        assert!(!ask(&mut h, "CONF?").is_empty());
        assert!(!h.cdc.mark());
        assert_eq!(ask(&mut h, "STAT?"), b"");
        assert!(h.cdc.mark());
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_log_goes_by_index_and_ends_in_an_error() {
        let mut h = identified(&[("model", "U1282A"), ("log", "61,62")]);
        assert_eq!(ask(&mut h, "LOG:AUTO 1"), b"a");
        assert_eq!(ask(&mut h, "LOG:AUTO 1"), b"a", "the driver asks again after a timeout");
        assert_eq!(ask(&mut h, "LOG:AUTO 2"), b"b");
        assert_eq!(ask(&mut h, "LOG:AUTO 3"), b"*E\r\n");
        assert_eq!(violations(&mut h), 0);
        let mut h = identified(&[("model", "U1241C"), ("log", "none")]);
        assert_eq!(ask(&mut h, "LOG:EXPO 1"), b"*E\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn square_wave_mode_answers_fetc_with_an_error() {
        let mut h = identified(&[
            ("model", "U1282A"),
            ("conf", &emu_hex(b"\"SQU\"\r\n")),
            ("conf2", &emu_hex(b"\"TEMP\"\r\n")),
        ]);
        assert_eq!(ask(&mut h, "CONF?"), b"\"SQU\"\r\n");
        assert_eq!(ask(&mut h, "CONF? @2"), b"\"TEMP\"\r\n");
        assert_eq!(ask(&mut h, "FETC?"), b"*E\r\n");
        assert_eq!(violations(&mut h), 0);
        assert_eq!(ask(&mut h, "FETC? @3"), b"+2.25000000E+01\r\n");
    }

    fn emu_hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    #[test]
    fn fetc_straight_after_squ_is_flagged() {
        let mut h =
            identified(&[("model", "U1282A"), ("conf", &emu_hex(b"\"SQU\"\r\n"))]);
        ask(&mut h, "CONF?");
        assert_eq!(violations(&mut h), 0);
        ask(&mut h, "FETC?");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_line_end_of_another_kind_is_flagged() {
        let mut h = meter(&[]);
        h.write(b"*IDN?\n");
        assert_eq!(violations(&mut h), 1, "*IDN? takes CR LF");
        h.write(b"FETC?\r\n");
        assert_eq!(violations(&mut h), 1, "the others a bare LF");
        h.write(b"FETC?\n");
        assert_eq!(violations(&mut h), 0);
        h.write(b"FETC?\n");
        assert_eq!(violations(&mut h), 1, "and then an empty line");
        h.write(b"\n");
        assert_eq!(violations(&mut h), 1, "an empty line has its CR");
    }

    #[test]
    fn an_unknown_command_is_flagged() {
        let mut h = identified(&[]);
        for bad in ["*RST", "SYST:BATT?", "FETC? @4", "CONF? @3", "FETC? @2 ", "LOG:HANDY 1"] {
            ask(&mut h, bad);
            assert_eq!(violations(&mut h), 1, "{bad}");
        }
    }

    #[test]
    fn a_query_before_an_identity_is_flagged() {
        let mut h = meter(&[]);
        ask(&mut h, "FETC?");
        assert_eq!(violations(&mut h), 1);
        let mut h = meter(&[("model", "U1282A")]);
        ask(&mut h, "LOG:HAND 1");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn an_identity_after_a_query_is_flagged() {
        let mut h = identified(&[]);
        ask(&mut h, "FETC?");
        h.write(b"*IDN?\r\n");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_display_the_meter_lacks_is_flagged() {
        let mut h = identified(&[("model", "U1232A")]);
        ask(&mut h, "FETC? @2");
        assert_eq!(violations(&mut h), 1);
        ask(&mut h, "CONF? @2");
        assert_eq!(violations(&mut h), 1);
        let mut h = identified(&[("model", "U1242B")]);
        ask(&mut h, "FETC? @2");
        assert_eq!(violations(&mut h), 0);
        ask(&mut h, "FETC? @3");
        assert_eq!(violations(&mut h), 1);
        assert_eq!(ask(&mut h, "CONF? @2"), b"\"TEMP:K CEL\"\r\n");
        assert_eq!(violations(&mut h), 0, "the driver asks of the temperature too");
    }

    #[test]
    fn the_log_checks() {
        let mut h = identified(&[("model", "U1253B")]);
        ask(&mut h, "LOG:HAND 1");
        assert_eq!(violations(&mut h), 1, "no log on this one");

        let log = [("model", "U1282A")];
        let mut h = identified(&log);
        ask(&mut h, "LOG:HAND x");
        assert_eq!(violations(&mut h), 1, "no index");
        let mut h = identified(&log);
        ask(&mut h, "LOG:HAND 0");
        assert_eq!(violations(&mut h), 1, "from 1");
        let mut h = identified(&log);
        ask(&mut h, "LOG:HAND 2");
        assert_eq!(violations(&mut h), 1, "not the first");
        let mut h = identified(&log);
        ask(&mut h, "LOG:HAND 1");
        ask(&mut h, "LOG:TRIG 2");
        assert_eq!(violations(&mut h), 1, "another source");
        let mut h = identified(&log);
        ask(&mut h, "LOG:HAND 1");
        ask(&mut h, "LOG:HAND 3");
        assert_eq!(violations(&mut h), 1, "one skipped");
        let mut h = identified(&log);
        ask(&mut h, "LOG:HAND 1");
        ask(&mut h, "FETC?");
        assert_eq!(violations(&mut h), 1, "a live query");
        let mut h = identified(&log);
        ask(&mut h, "STAT?");
        ask(&mut h, "LOG:HAND 1");
        assert_eq!(violations(&mut h), 1, "after a live query");
    }

    #[test]
    fn a_command_too_long_is_flagged_and_bounded() {
        let mut h = meter(&[]);
        h.write(&[b'x'; LONGEST_COMMAND + 1]);
        assert_eq!(violations(&mut h), 1);
        assert!(h.cdc.model.pending.len() <= LONGEST_COMMAND);
    }

    #[test]
    fn a_command_left_unfinished_is_flagged_at_the_next_open() {
        let mut h = identified(&[]);
        h.write(b"FETC");
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = identified(&[("fetc", "61,62")]);
        assert_eq!(ask(&mut h, "FETC?"), b"a");
        h.cdc.model.reset();
        h.write(b"*IDN?\r\n");
        h.read();
        assert_eq!(ask(&mut h, "FETC?"), b"a");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn bad_options_are_refused() {
        for bad in [
            ("model", "U9999"),
            ("uart", "fast"),
            ("id", "xx"),
            ("conf", ""),
            ("log", "x"),
            ("answers", "x"),
            ("nosuch", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(function(&o).is_err(), "{bad:?}");
        }
    }
}
