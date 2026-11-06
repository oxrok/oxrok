// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! What every SCPI instrument's model shares, on USBTMC ([`Config::usbtmc`]) and
//! on TCP ([`Config::tcp`]) alike. A model says what its commands do; the
//! [`Engine`] does the rest:
//!
//! - lines, ended by LF, CR LF too, and cut at `;` outside quotes;
//! - `*IDN?` (from [`Instrument::idn`], or `idn=`), `*OPC?`, `*OPC`, `*WAI`,
//!   `*ESR?`, `*CLS` and `SYST:ERR?` with its queue;
//! - [`Settings`]: a header with a value, `HEADER?` answers it and `HEADER X`
//!   sets it, so that most of a model is a list;
//! - an unknown command: a violation, the error `-113`, no answer;
//! - an answer nobody read when the next command comes: gone, with the error
//!   `-410`, as IEEE 488.2 has it (on TCP an answer is on its way at once, so this
//!   is USBTMC's);
//! - the fault options ([`Config::option`]), counted in queries, from 1, `*OPC?`
//!   among them:
//!   - `mute-at=N`: the Nth and all later queries get no answer, and take effect;
//!   - `silent-at=N`: the Nth query alone gets none;
//!   - `late-at=N:T`: the Nth answer waits T ticks, and what comes on in the
//!     meantime waits behind it, so a host that gave up reads it as the answer
//!     to its next question;
//!   - `opc-late=N`: the first N `*OPC?` answer `0`;
//!   - `mark-at=N`: the event point (`--unplug model`, `--unplug tcp-model`), once
//!     the Nth query was answered;
//!   - `idn=TEXT`; `eol=lf|crlf|none`, how text answers end;
//!   - `framing=rigol`: on TCP, the length before each answer that `tcp-rigol`
//!     reads;
//!   - and [`usbtmc::Faults`]'s.
//!
//! A tick is one offer of a bulk IN on USBTMC (host paced, about 20 ms) and 20 ms
//! of wall time on TCP.
//!
//! No driver reads the error queue, and only `rigol-ds`'s single shot check reads
//! `*ESR?`; they are here since the instrument has them and a test can look.

use std::collections::{BTreeMap, VecDeque};

use crate::model::{DeviceModel, Violations};
use crate::options::parse_number;
use crate::shells::tcp::{Port, Stream};
use crate::shells::usbtmc::{self, Messages, Usbtmc};
use crate::{Error, Result};

/// The longest line taken; what is longer is flagged and dropped.
const LONGEST: usize = 1 << 16;

/// One program message.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Message<'a> {
    /// As it came, without the line end.
    pub text: &'a str,
    /// Up to the first space, in upper case and with no leading colon: SCPI minds
    /// neither.
    pub header: String,
    /// After the first space, as it came; empty when there is none.
    pub args: &'a str,
}

impl<'a> Message<'a> {
    fn new(text: &'a str) -> Message<'a> {
        let (header, args) = text.split_once(' ').unwrap_or((text, ""));
        let header = header.strip_prefix(':').unwrap_or(header).to_ascii_uppercase();
        Message { text, header, args: args.trim() }
    }

    /// It asks for an answer, as `:WAV:DATA? CHAN1` and `:CURR:PROT:STAT ON?` do.
    pub fn is_query(&self) -> bool {
        self.header.ends_with('?') || self.text.ends_with('?')
    }

    /// The header without its question mark.
    pub fn name(&self) -> &str {
        self.header.strip_suffix('?').unwrap_or(&self.header)
    }
}

/// What a message comes to.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Answer {
    /// Text, to which the line end goes.
    Text(String),
    /// These bytes and no others. [`block`] makes block data.
    Bytes(Vec<u8>),
    /// Done, and nothing to say: a setting.
    Done,
    /// Heard, and no answer ever, with no error.
    Silent,
    /// This error goes in the queue, with no answer.
    Error(i32, String),
    /// Not the instrument's: the [`Engine`] looks at what it has, then it is
    /// unknown.
    Unknown,
}

/// IEEE 488.2 definite length block data: `#`, the number of digits, the length
/// in that many digits, the bytes, and `end`.
pub fn block(digits: usize, data: &[u8], end: &[u8]) -> Vec<u8> {
    let mut out = format!("#{digits}{:0digits$}", data.len()).into_bytes();
    out.extend_from_slice(data);
    out.extend_from_slice(end);
    out
}

/// A header's value: `HEADER?` answers it, `HEADER VALUE` changes it. Headers are
/// as [`Message::header`] has them.
#[derive(Clone, Debug, Default)]
pub struct Settings(BTreeMap<String, String>);

impl Settings {
    /// Sets a header's value, adding the header.
    pub fn set(&mut self, header: &str, value: impl Into<String>) {
        self.0.insert(header.to_ascii_uppercase(), value.into());
    }

    /// With this header added.
    #[must_use]
    pub fn with(mut self, header: &str, value: impl Into<String>) -> Settings {
        self.set(header, value);
        self
    }

    /// A header's value.
    pub fn get(&self, header: &str) -> Option<&str> {
        self.0.get(&header.to_ascii_uppercase()).map(String::as_str)
    }

    /// A header's value as a number; 0 when it is none.
    pub fn number(&self, header: &str) -> f64 {
        self.get(header).and_then(|v| v.parse().ok()).unwrap_or(0.0)
    }
}

/// An instrument's commands.
pub trait Instrument: Send {
    /// The answer to `*IDN?`.
    fn idn(&self) -> String;

    /// What it starts with; what is here takes no code.
    fn settings(&self) -> Settings {
        Settings::default()
    }

    /// A message, before the [`Engine`] looks at it. `s` holds the settings.
    /// A model flags what the host must not do on `v`.
    fn command(&mut self, m: &Message<'_>, s: &mut Settings, v: &mut Violations) -> Answer {
        let _ = (m, s, v);
        Answer::Unknown
    }
}

/// What a SCPI model's options come to.
#[derive(Clone, Debug)]
pub struct Config {
    mute_at: Option<u32>,
    silent_at: Option<u32>,
    late_at: Option<(u32, u32)>,
    opc_late: u32,
    mark_at: Option<u32>,
    idn: Option<String>,
    eol: Vec<u8>,
    rigol_tcp: bool,
    tmc: usbtmc::Faults,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            mute_at: None,
            silent_at: None,
            late_at: None,
            opc_late: 0,
            mark_at: None,
            idn: None,
            eol: b"\n".to_vec(),
            rigol_tcp: false,
            tmc: usbtmc::Faults::default(),
        }
    }
}

impl Config {
    /// Text answers end so unless `eol=` says otherwise: for an instrument
    /// with another habit.
    #[must_use]
    pub fn with_eol(mut self, eol: &[u8]) -> Config {
        self.eol = eol.to_vec();
        self
    }

    /// Takes `-o KEY=VALUE` if it is one of the module's. All of a model's
    /// options go through here first.
    ///
    /// # Errors
    ///
    /// On a value it cannot use.
    pub fn option(&mut self, key: &str, value: &str) -> Result<bool> {
        let count = |text: &str| parse_number(key, text, 1..=1 << 24);
        match key {
            "mute-at" => self.mute_at = Some(count(value)?),
            "silent-at" => self.silent_at = Some(count(value)?),
            "late-at" => {
                let (nth, ticks) = value
                    .split_once(':')
                    .ok_or_else(|| Error::Invalid(format!("late-at={value}: as in 2:40")))?;
                self.late_at = Some((count(nth)?, count(ticks)?));
            }
            "opc-late" => self.opc_late = parse_number(key, value, 0..=1 << 24)?,
            "mark-at" => self.mark_at = Some(count(value)?),
            "idn" => self.idn = Some(value.to_owned()),
            "eol" => {
                self.eol = match value {
                    "lf" => b"\n".to_vec(),
                    "crlf" => b"\r\n".to_vec(),
                    "none" => Vec::new(),
                    _ => return Err(Error::Invalid(format!("eol={value}: lf, crlf or none"))),
                };
            }
            "framing" => match value {
                "rigol" => self.rigol_tcp = true,
                _ => return Err(Error::Invalid(format!("framing={value}: rigol"))),
            },
            _ => return self.tmc.option(key, value),
        }
        Ok(true)
    }

    /// The instrument on USBTMC.
    pub fn usbtmc<I: Instrument + 'static>(
        self,
        name: &'static str,
        id: &usbtmc::Identity,
        instrument: I,
    ) -> Box<dyn DeviceModel> {
        let tmc = self.tmc.clone();
        Box::new(Usbtmc::new(name, id, tmc, Engine::new(instrument, self)))
    }

    /// The instrument on a socket.
    pub fn tcp<I: Instrument + 'static>(self, instrument: I) -> Box<dyn Stream> {
        let rigol = self.rigol_tcp;
        Box::new(OnSocket { engine: Engine::new(instrument, self), rigol })
    }
}

struct Waiting {
    bytes: Vec<u8>,
    ticks: u32,
}

/// The message layer: an [`Instrument`] with what all of them have.
pub struct Engine<I> {
    instrument: I,
    settings: Settings,
    config: Config,
    line: Vec<u8>,
    answers: VecDeque<Waiting>,
    errors: VecDeque<(i32, String)>,
    esr: u8,
    queries: u32,
    opcs: u32,
    mark: bool,
}

impl<I: Instrument> Engine<I> {
    /// At power on.
    pub fn new(instrument: I, config: Config) -> Engine<I> {
        let settings = instrument.settings();
        Engine {
            instrument,
            settings,
            config,
            line: Vec::new(),
            answers: VecDeque::new(),
            errors: VecDeque::new(),
            esr: 0,
            queries: 0,
            opcs: 0,
            mark: false,
        }
    }

    /// Bytes from the host, in whatever pieces.
    fn input(&mut self, data: &[u8], v: &mut Violations) {
        for b in data {
            if *b == b'\n' {
                let line = std::mem::take(&mut self.line);
                self.line_in(&line, v);
            } else if self.line.len() < LONGEST {
                self.line.push(*b);
            } else {
                v.report(format!("a line of more than {LONGEST} bytes"));
                self.line.clear();
            }
        }
    }

    fn line_in(&mut self, line: &[u8], v: &mut Violations) {
        let Ok(text) = std::str::from_utf8(line) else {
            v.report(format!("a line that is no text: {line:02x?}"));
            self.error(-120, "Invalid character");
            return;
        };
        let text = text.strip_suffix('\r').unwrap_or(text);
        // An answer ready and not read. One held back is still to come.
        let before = self.answers.len();
        self.answers.retain(|a| a.ticks > 0);
        if self.answers.len() < before {
            self.error(-410, "Query INTERRUPTED");
        }
        let mut quoted = false;
        let mut from = 0;
        for (i, c) in text.char_indices() {
            match c {
                '"' => quoted = !quoted,
                ';' if !quoted => {
                    self.execute(text[from..i].trim(), v);
                    from = i + 1;
                }
                _ => {}
            }
        }
        self.execute(text[from..].trim(), v);
    }

    fn error(&mut self, code: i32, text: &str) {
        self.esr |= match code {
            -199..=-100 => 0x20,
            -299..=-200 => 0x10,
            -399..=-300 => 0x08,
            -499..=-400 => 0x04,
            _ => 0,
        };
        self.errors.push_back((code, text.to_owned()));
    }

    fn execute(&mut self, text: &str, v: &mut Violations) {
        if text.is_empty() {
            return;
        }
        let m = Message::new(text);
        let answer = match self.instrument.command(&m, &mut self.settings, v) {
            Answer::Unknown => self.common(&m),
            other => other,
        };
        self.conclude(&m, answer, v);
    }

    /// What every instrument has: the common commands, then the settings.
    fn common(&mut self, m: &Message<'_>) -> Answer {
        match m.header.as_str() {
            "*IDN?" => {
                Answer::Text(self.config.idn.clone().unwrap_or_else(|| self.instrument.idn()))
            }
            "*OPC?" => {
                self.opcs += 1;
                Answer::Text(if self.opcs > self.config.opc_late { "1" } else { "0" }.to_owned())
            }
            "*OPC" | "*WAI" => Answer::Done,
            "*ESR?" => Answer::Text(std::mem::take(&mut self.esr).to_string()),
            "*CLS" => {
                self.errors.clear();
                self.esr = 0;
                Answer::Done
            }
            "SYST:ERR?" | "SYST:ERR:NEXT?" => {
                let (code, text) =
                    self.errors.pop_front().unwrap_or_else(|| (0, "No error".to_owned()));
                Answer::Text(format!("{code},\"{text}\""))
            }
            _ => {
                let name = m.name();
                let Some(value) = self.settings.get(name) else {
                    return Answer::Unknown;
                };
                if m.is_query() {
                    Answer::Text(value.to_owned())
                } else if m.args.is_empty() {
                    Answer::Unknown
                } else {
                    self.settings.set(name, m.args);
                    Answer::Done
                }
            }
        }
    }

    fn conclude(&mut self, m: &Message<'_>, answer: Answer, v: &mut Violations) {
        let bytes = match answer {
            Answer::Text(text) => {
                let mut bytes = text.into_bytes();
                bytes.extend_from_slice(&self.config.eol);
                bytes
            }
            Answer::Bytes(bytes) => bytes,
            Answer::Done | Answer::Silent => Vec::new(),
            Answer::Error(code, text) => {
                self.error(code, &text);
                Vec::new()
            }
            Answer::Unknown => {
                v.report(format!("{}: the instrument has no such command", m.text));
                self.error(-113, "Undefined header");
                Vec::new()
            }
        };
        if !m.is_query() {
            return;
        }
        self.queries += 1;
        let n = self.queries;
        if self.config.mark_at == Some(n) {
            self.mark = true;
        }
        if bytes.is_empty()
            || self.config.mute_at.is_some_and(|at| n >= at)
            || self.config.silent_at == Some(n)
        {
            return;
        }
        let ticks = match self.config.late_at {
            Some((at, ticks)) if at == n => ticks,
            _ => 0,
        };
        self.answers.push_back(Waiting { bytes, ticks });
    }

    /// The next answer that is due.
    fn take(&mut self) -> Option<Vec<u8>> {
        if self.answers.front()?.ticks > 0 {
            return None;
        }
        self.answers.pop_front().map(|a| a.bytes)
    }

    fn pass_time(&mut self) {
        if let Some(a) = self.answers.front_mut() {
            a.ticks = a.ticks.saturating_sub(1);
        }
    }
}

impl<I: Instrument> Messages for Engine<I> {
    fn message(&mut self, data: &[u8], v: &mut Violations) {
        self.input(data, v);
    }

    fn answer(&mut self) -> Option<Vec<u8>> {
        self.take()
    }

    fn tick(&mut self) {
        self.pass_time();
    }

    fn mark(&mut self) -> bool {
        std::mem::take(&mut self.mark)
    }

    fn clear(&mut self) {
        self.line.clear();
        self.answers.clear();
    }
}

/// An [`Engine`] as a stream of bytes.
struct OnSocket<I> {
    engine: Engine<I>,
    rigol: bool,
}

impl<I: Instrument> OnSocket<I> {
    fn out(&mut self, port: &mut Port) {
        while let Some(bytes) = self.engine.take() {
            if self.rigol {
                port.send(&(bytes.len() as u32).to_le_bytes());
            }
            port.send(&bytes);
        }
        if std::mem::take(&mut self.engine.mark) {
            port.mark();
        }
    }
}

impl<I: Instrument> Stream for OnSocket<I> {
    fn received(&mut self, data: &[u8], port: &mut Port) {
        self.engine.input(data, port.violations());
        self.out(port);
    }

    fn tick(&mut self, port: &mut Port) {
        self.engine.pass_time();
        self.out(port);
    }

    fn closed(&mut self) {
        self.engine.line.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A supply and a meter, as `scpi-pps` and `scpi-dmm` speak to them.
    struct Bench;

    impl Instrument for Bench {
        fn idn(&self) -> String {
            "Acme,B1,1,2".to_owned()
        }

        fn settings(&self) -> Settings {
            Settings::default().with("SOUR:VOLT", "1.5").with("OUTP:STAT", "0")
        }

        fn command(&mut self, m: &Message<'_>, s: &mut Settings, _v: &mut Violations) -> Answer {
            match m.header.as_str() {
                "OUTP" => {
                    s.set("OUTP:STAT", if m.args == "ON" { "1" } else { "0" });
                    Answer::Done
                }
                "SYST:REM" => Answer::Done,
                "CONF?" => Answer::Text("\"VOLT +1.0E+01,+3.0E-06\"".to_owned()),
                "BAD" => Answer::Error(-222, "Data out of range".to_owned()),
                "WAV:DATA?" => Answer::Bytes(block(9, b"abc", b"\n")),
                "NEVER?" => Answer::Silent,
                _ => Answer::Unknown,
            }
        }
    }

    struct Rig {
        e: Engine<Bench>,
        v: Violations,
    }

    impl Rig {
        fn new(options: &[(&str, &str)]) -> Rig {
            let mut c = Config::default();
            for (k, v) in options {
                assert!(c.option(k, v).unwrap(), "{k}");
            }
            Rig { e: Engine::new(Bench, c), v: Violations::default() }
        }

        fn send(&mut self, line: &str) {
            self.e.input(line.as_bytes(), &mut self.v);
        }

        /// A line and what comes of it at once, as text.
        fn ask(&mut self, line: &str) -> Option<String> {
            self.send(line);
            self.e.take().map(|b| String::from_utf8(b).unwrap())
        }
    }

    #[test]
    fn the_common_commands_and_the_settings_take_no_code() {
        let mut r = Rig::new(&[]);
        assert_eq!(r.ask("*IDN?\n").unwrap(), "Acme,B1,1,2\n");
        assert_eq!(r.ask("*OPC?\n").unwrap(), "1\n");
        assert_eq!(r.ask(":SOUR:VOLT?\n").unwrap(), "1.5\n");
        assert_eq!(r.ask(":sour:volt 12.000000\n"), None, "SCPI does not mind the case");
        assert_eq!(r.ask(":SOUR:VOLT?\n").unwrap(), "12.000000\n");
        assert_eq!(r.ask(":OUTP ON\n"), None);
        assert_eq!(r.ask(":OUTP:STAT?\n").unwrap(), "1\n", "a set that changes state");
        assert!(r.v.is_empty());
    }

    #[test]
    fn a_line_comes_in_pieces_and_with_several_messages() {
        let mut r = Rig::new(&[]);
        assert_eq!(r.ask("*ID"), None);
        assert_eq!(r.ask("N?\r\n").unwrap(), "Acme,B1,1,2\n");
        assert_eq!(r.ask("*CLS;SYST:REM\n"), None, "scpi-dmm's start");
        assert!(r.v.is_empty());
        assert_eq!(r.ask("CONF?;*OPC?\n").unwrap(), "\"VOLT +1.0E+01,+3.0E-06\"\n");
        assert_eq!(r.e.take().unwrap(), b"1\n", "a second answer stays behind the first");
    }

    #[test]
    fn a_query_with_arguments_is_a_query() {
        for (text, query, name) in [
            (":WAV:DATA? CHAN1", true, "WAV:DATA"),
            (":CURR:PROT:STAT ON?", true, "CURR:PROT:STAT"),
            (":OUTP ON", false, "OUTP"),
            ("*IDN?", true, "*IDN"),
        ] {
            let m = Message::new(text);
            assert_eq!((m.is_query(), m.name()), (query, name), "{text}");
        }
    }

    #[test]
    fn an_unknown_command_is_flagged_and_queued() {
        let mut r = Rig::new(&[]);
        assert_eq!(r.ask(":NO:SUCH?\n"), None);
        let flagged = r.v.drain();
        assert_eq!(flagged, [":NO:SUCH?: the instrument has no such command"]);
        assert_eq!(r.ask("*ESR?\n").unwrap(), "32\n", "a command error");
        assert_eq!(r.ask("*ESR?\n").unwrap(), "0\n", "read and clear");
        assert_eq!(r.ask("BAD\n"), None);
        assert_eq!(r.ask("*ESR?\n").unwrap(), "16\n", "an execution error");
        assert_eq!(r.ask("SYST:ERR?\n").unwrap(), "-113,\"Undefined header\"\n");
        assert_eq!(r.ask("SYST:ERR?\n").unwrap(), "-222,\"Data out of range\"\n");
        assert_eq!(r.ask("SYST:ERR?\n").unwrap(), "0,\"No error\"\n");
        assert_eq!(r.ask("BAD\n*CLS\n"), None);
        assert_eq!(r.ask("SYST:ERR?\n").unwrap(), "0,\"No error\"\n");
        assert!(r.v.is_empty(), "a refused value is no violation");
    }

    #[test]
    fn an_answer_nobody_read_goes_with_an_error() {
        let mut r = Rig::new(&[]);
        r.send("*IDN?\n");
        r.send("*OPC?\n");
        assert_eq!(r.e.take().unwrap(), b"1\n");
        assert_eq!(r.ask("SYST:ERR?\n").unwrap(), "-410,\"Query INTERRUPTED\"\n");
    }

    #[test]
    fn block_data_says_its_length() {
        assert_eq!(block(9, b"abc", b"\n"), b"#9000000003abc\n");
        assert_eq!(block(8, &[0; 2], b""), b"#800000002\0\0");
        let mut r = Rig::new(&[]);
        assert_eq!(r.ask(":WAV:DATA?\n").unwrap(), "#9000000003abc\n");
    }

    #[test]
    fn the_faults_count_queries() {
        let mut r = Rig::new(&[("silent-at", "2")]);
        assert!(r.ask("*IDN?\n").is_some());
        assert!(r.ask("*IDN?\n").is_none());
        assert!(r.ask("*IDN?\n").is_some());

        let mut r = Rig::new(&[("mute-at", "2")]);
        assert!(r.ask("*IDN?\n").is_some());
        assert!(r.ask(":OUTP ON\n").is_none());
        assert!(r.ask(":OUTP:STAT?\n").is_none());
        assert!(r.ask(":OUTP:STAT?\n").is_none());
        assert_eq!(r.e.settings.get("OUTP:STAT"), Some("1"), "a set still takes effect");

        let mut r = Rig::new(&[("opc-late", "2")]);
        let opc: Vec<_> = (0..3).map(|_| r.ask("*OPC?\n").unwrap()).collect();
        assert_eq!(opc, ["0\n", "0\n", "1\n"]);

        let mut r = Rig::new(&[("idn", "Other,X,1,2"), ("eol", "crlf")]);
        assert_eq!(r.ask("*IDN?\n").unwrap(), "Other,X,1,2\r\n");
        let mut r = Rig::new(&[("eol", "none")]);
        assert_eq!(r.ask("*OPC?\n").unwrap(), "1");

        let mut r = Rig::new(&[("mark-at", "1")]);
        assert!(!Messages::mark(&mut r.e));
        r.send("*OPC?\n");
        assert!(Messages::mark(&mut r.e));
        assert!(!Messages::mark(&mut r.e), "once");

        // The silence of Answer::Silent, whatever the faults.
        let mut r = Rig::new(&[]);
        assert!(r.ask(":NEVER?\n").is_none());
        assert!(r.v.is_empty());
    }

    #[test]
    fn a_late_answer_answers_the_next_question_if_the_host_went_on() {
        let mut r = Rig::new(&[("late-at", "1:2")]);
        assert!(r.ask("*IDN?\n").is_none());
        r.e.pass_time();
        assert!(r.e.take().is_none());
        // The host gave up and asks again: the old answer comes first.
        r.send("*OPC?\n");
        r.e.pass_time();
        assert_eq!(r.e.take().unwrap(), b"Acme,B1,1,2\n");
        assert_eq!(r.e.take().unwrap(), b"1\n");
    }

    #[test]
    fn options_refuse_what_they_cannot_use() {
        let mut c = Config::default();
        assert!(c.option("mute-at", "0").is_err());
        assert!(c.option("late-at", "2").is_err());
        assert!(c.option("eol", "cr").is_err());
        assert!(c.option("framing", "other").is_err());
        assert!(!c.option("no-such", "1").unwrap());
    }

    #[test]
    fn it_runs_on_both_carriers() {
        let id = usbtmc::Identity::new(1, 2, "M", "P");
        let shell =
            Usbtmc::new("t", &id, usbtmc::Faults::default(), Engine::new(Bench, Config::default()));
        let mut h = usbtmc::host::Host::new(shell);
        h.send(b"*IDN?\n");
        assert_eq!(h.answer().unwrap(), b"Acme,B1,1,2\n");
        assert!(h.v.is_empty());

        let mut c = Config::default();
        c.option("framing", "rigol").unwrap();
        let mut s = OnSocket { engine: Engine::new(Bench, c), rigol: true };
        let mut port = Port::default();
        s.received(b"*OPC?\n", &mut port);
        assert_eq!(port.sent(), [vec![2u8, 0, 0, 0], b"1\n".to_vec()]);
    }
}
