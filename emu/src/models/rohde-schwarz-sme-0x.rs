// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Rohde&Schwarz SME series signal generator, spoken to in SCPI over its
//! serial remote port, on the CDC-ACM shell: lines of text, a reply only to a
//! query.
//!
//! The device, and how sure this is:
//!
//! - The commands and the maker's and models' names: the C driver, and nothing
//!   else. No recording and no copy of the instrument's remote control chapter
//!   was to hand (the maker's PDF could not be fetched), so these are stand-ins:
//!   the form of the `*IDN?` fields past the model, the text of a `FREQ?` and a
//!   `POW?` reply, LF as the reply's end, and the UART's 9600/8n1. `raw*=`,
//!   `eol=` and `uart=` give the other forms.
//! - The frequency ranges: a dealer's copy of the maker's brochure has 5 kHz to
//!   1.5, 3, 2.2 and 6 GHz for the SME02, SME03, SME03E and SME06. It has no
//!   SME03A, which is from the driver, as is the level range. The driver's table
//!   has the SME06 at 1.5 GHz.
//! - The USB side is none of the instrument's, which has no USB: the serial
//!   port is reached through an adapter. The ids and strings are stand-ins.
//!
//! Emulation artifacts:
//!
//! - It answers at once, and a setting takes no time.
//! - A setting it cannot take (out of range, no number) changes nothing and
//!   says nothing, as a command has no reply. A real SCPI device would keep an
//!   error in a queue that nothing here reads.
//! - Remote state is only a flag for the check below.
//! - `shift=` is a late reply with no clock. A real delay is left to the driver's
//!   tests on a fake port: C's window for it is a race, as its next query goes
//!   out the moment its 1 s ends.
//!
//! Not modelled: everything else the instrument does. A line no C path sends is
//! flagged, though a real one takes much more (long forms, other units, lower
//! case).
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `model=NAME`, the model field of `*IDN?` and the range it brings;
//! - `idn=TEXT`, all of what `*IDN?` says, before the end of line;
//! - `freq=HZ` and `pow=DBM`, the state at power on;
//! - `rawidn=`, `rawfreq=` and `rawpow=` take `HEX|none`, what that query gets
//!   instead of its reply, whole line included: a device gone wrong, or silence;
//! - `eol=lf|crlf|lfcr`, how a reply ends;
//! - `uart=LINE`, the UART's settings;
//! - `ids=VVVV:PPPP`, the USB ids;
//! - `answers=N`, `FREQ?` and `POW?` answered per power on, then silence and the
//!   event point;
//! - `shift=1`, each `FREQ?` or `POW?` gets the answer of the one before, and
//!   the first gets none, which is a late reply with no clock.
//!
//! The pedantic checks: only the lines above, each ended by LF; a value with
//! the unit the driver sends; a query only after a bare LF has been heard since
//! power on (the driver's "remote" command, sent by each scan); nothing left of
//! a line when a tool opens.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "rohde-schwarz-sme-0x";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Rohde&Schwarz SME signal generator, SCPI over a serial port on CDC-ACM; model=NAME, \
                         idn=TEXT, freq=HZ, pow=DBM, rawidn=HEX|none, rawfreq=HEX|none, rawpow=HEX|none, eol=lf|crlf|lfcr, \
                         uart=LINE, ids=VVVV:PPPP, answers=N, shift=1, and the shell's stall=, chunk=, \
                         in-limit=, in-stall=, out-hold=";

/// The longest line worth keeping: a frequency at the top of a double's range
/// does not reach this and no C path gets near it. Past this a line is flagged
/// and dropped.
const LONGEST: usize = 512;
/// The longest reply an option may set.
const LONGEST_RAW: usize = 1024;
/// `idn=` of a stand-in serial number and version.
const IDN_TAIL: &str = "000000/000,1.00";
/// The bottom of every model's range, in Hz.
const FREQ_MIN: f64 = 5e3;
/// The level range of every model, in dBm.
const POW_RANGE: (f64, f64) = (-144.0, 16.0);

/// The top of a model's range, in Hz; an unknown name gets an SME03's.
fn freq_max(model: &str) -> f64 {
    match model {
        "SME02" => 1.5e9,
        "SME03E" => 2.2e9,
        "SME06" => 6e9,
        _ => 3e9,
    }
}

/// A generator from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

/// `HEX`, or `none` for no reply at all.
fn reply_option(key: &str, value: &str) -> Result<Vec<u8>> {
    if value == "none" {
        return Ok(Vec::new());
    }
    Ok(parse_hex_list(key, value, 1, LONGEST_RAW)?.remove(0))
}

fn float_option(key: &str, value: &str) -> Result<f64> {
    value.parse().map_err(|_| Error::Invalid(format!("{key}={value}: no number")))
}

fn ids_option(value: &str) -> Result<(u16, u16)> {
    let bad = || Error::Invalid(format!("ids={value}: as in 0aad:0117"));
    let (vendor, product) = value.split_once(':').ok_or_else(bad)?;
    Ok((
        u16::from_str_radix(vendor, 16).map_err(|_| bad())?,
        u16::from_str_radix(product, 16).map_err(|_| bad())?,
    ))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Generator>> {
    let mut faults = Faults::default();
    let mut config = Config::default();
    let mut uart = Line::parse("9600/8n1")?;
    let mut ids = (0x1209, 0x0001);
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => config.model.clone_from(v),
            "idn" => config.idn = Some(v.clone()),
            "freq" => config.freq = float_option(k, v)?,
            "pow" => config.pow = float_option(k, v)?,
            "rawidn" => config.raw_idn = Some(reply_option(k, v)?),
            "rawfreq" => config.raw_freq = Some(reply_option(k, v)?),
            "rawpow" => config.raw_pow = Some(reply_option(k, v)?),
            "eol" => {
                config.eol = match v.as_str() {
                    "lf" => b"\n",
                    "crlf" => b"\r\n",
                    "lfcr" => b"\n\r",
                    _ => return Err(Error::Invalid(format!("eol={v}: lf, crlf or lfcr"))),
                };
            }
            "uart" => uart = Line::parse(v)?,
            "ids" => ids = ids_option(v)?,
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "shift" => config.shift = parse_number(k, v, 0..=1)? == 1,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let id = Identity::new(ids.0, ids.1, "oxrok-emu", "SME serial stand-in");
    Ok(Cdc::new(NAME, &id, faults, Generator::new(config)).behind_uart(uart))
}

#[derive(Clone, Debug)]
struct Config {
    model: String,
    idn: Option<String>,
    freq: f64,
    pow: f64,
    raw_idn: Option<Vec<u8>>,
    raw_freq: Option<Vec<u8>>,
    raw_pow: Option<Vec<u8>>,
    eol: &'static [u8],
    answers: Option<u32>,
    shift: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            model: "SME03".to_owned(),
            idn: None,
            freq: 100e6,
            pow: -30.0,
            raw_idn: None,
            raw_freq: None,
            raw_pow: None,
            eol: b"\n",
            answers: None,
            shift: false,
        }
    }
}

struct Generator {
    config: Config,
    freq: f64,
    pow: f64,
    /// A bare LF has come since power on.
    remote: bool,
    /// What has come of a line so far.
    pending: Vec<u8>,
    /// `FREQ?` and `POW?` so far.
    queries: u32,
    /// The answer `shift=` holds for the next query.
    held: Option<Vec<u8>>,
}

impl Generator {
    fn new(config: Config) -> Generator {
        Generator {
            freq: config.freq,
            pow: config.pow,
            config,
            remote: false,
            pending: Vec::new(),
            queries: 0,
            held: None,
        }
    }

    fn line_of(&self, text: &str) -> Vec<u8> {
        let mut reply = text.as_bytes().to_vec();
        reply.extend_from_slice(self.config.eol);
        reply
    }

    fn check_remote(&self, shown: &str, port: &mut Port<'_>) {
        if !self.remote {
            port.violation(format!("'{shown}': a query with no remote line since power on"));
        }
    }

    fn identify(&self, port: &mut Port<'_>) {
        if let Some(raw) = &self.config.raw_idn {
            port.send(raw);
            return;
        }
        let text = self
            .config
            .idn
            .clone()
            .unwrap_or_else(|| format!("Rohde&Schwarz,{},{IDN_TAIL}", self.config.model));
        port.send(&self.line_of(&text));
    }

    /// A `FREQ?` or a `POW?`: `raw` or `value`, as this query's turn has it.
    fn reading(&mut self, value: f64, raw: Option<Vec<u8>>, port: &mut Port<'_>) {
        if self.config.answers.is_some_and(|n| self.queries >= n) {
            port.mark();
            return;
        }
        self.queries += 1;
        let reply = raw.unwrap_or_else(|| self.line_of(&value.to_string()));
        if self.config.shift {
            if let Some(before) = self.held.replace(reply) {
                port.send(&before);
            }
            return;
        }
        port.send(&reply);
    }

    /// One whole line, with its line feed taken off.
    fn line(&mut self, line: &[u8], port: &mut Port<'_>) {
        let shown = line.escape_ascii().to_string();
        let Ok(text) = std::str::from_utf8(line) else {
            port.violation(format!("'{shown}': a line no driver path sends"));
            return;
        };
        match text {
            "" => self.remote = true,
            "*IDN?" => {
                self.check_remote(&shown, port);
                self.identify(port);
            }
            "FREQ?" => {
                self.check_remote(&shown, port);
                self.reading(self.freq, self.config.raw_freq.clone(), port);
            }
            "POW?" => {
                self.check_remote(&shown, port);
                self.reading(self.pow, self.config.raw_pow.clone(), port);
            }
            _ => self.setting(text, &shown, port),
        }
    }

    fn setting(&mut self, text: &str, shown: &str, port: &mut Port<'_>) {
        let number = |prefix: &str, unit: &str| {
            text.strip_prefix(prefix)?.strip_suffix(unit)?.parse::<f64>().ok()
        };
        if let Some(hz) = number("FREQ ", "Hz") {
            if (FREQ_MIN..=freq_max(&self.config.model)).contains(&hz) {
                self.freq = hz;
            }
        } else if let Some(dbm) = number("POW ", "dBm") {
            if (POW_RANGE.0..=POW_RANGE.1).contains(&dbm) {
                self.pow = dbm;
            }
        } else {
            port.violation(format!("'{shown}': a line no driver path sends"));
        }
    }
}

impl Serial for Generator {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte == b'\n' {
                let line = std::mem::take(&mut self.pending);
                self.line(&line, port);
            } else if self.pending.len() >= LONGEST {
                port.violation("a line too long".to_owned());
                self.pending.clear();
            } else {
                self.pending.push(byte);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!(
                "'{}': a command left unfinished",
                self.pending.escape_ascii()
            ));
            self.pending.clear();
        }
    }

    fn reset(&mut self) {
        *self = Generator::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn generator(options: &[(&str, &str)]) -> Host<Generator> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Generator>) -> usize {
        h.v.drain().len()
    }

    fn remote(options: &[(&str, &str)]) -> Host<Generator> {
        let mut h = generator(options);
        h.write(b"\n");
        h
    }

    fn ask(h: &mut Host<Generator>, line: &str) -> Vec<u8> {
        h.write(format!("{line}\n").as_bytes());
        h.read()
    }

    #[test]
    fn the_scan_and_the_state() {
        let mut h = remote(&[]);
        assert_eq!(ask(&mut h, "*IDN?"), b"Rohde&Schwarz,SME03,000000/000,1.00\n");
        assert_eq!(ask(&mut h, "FREQ?"), b"100000000\n");
        assert_eq!(ask(&mut h, "POW?"), b"-30\n");
        assert_eq!(ask(&mut h, "FREQ 1000000.5Hz"), b"");
        assert_eq!(ask(&mut h, "POW -10.5dBm"), b"");
        assert_eq!(ask(&mut h, "FREQ?"), b"1000000.5\n");
        assert_eq!(ask(&mut h, "POW?"), b"-10.5\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_it_cannot_take_changes_nothing_and_is_no_violation() {
        let mut h = remote(&[("model", "SME02")]);
        for line in
            ["FREQ nanHz", "FREQ 4999.9Hz", "FREQ 1500000000.1Hz", "POW 16.1dBm", "POW -144.1dBm"]
        {
            assert_eq!(ask(&mut h, line), b"");
        }
        assert_eq!(ask(&mut h, "FREQ?"), b"100000000\n");
        assert_eq!(ask(&mut h, "POW?"), b"-30\n");
        assert_eq!(violations(&mut h), 0);
        for line in ["FREQ 1500000000.0Hz", "FREQ 5000.0Hz", "POW 16.0dBm", "POW -144.0dBm"] {
            ask(&mut h, line);
        }
        assert_eq!(ask(&mut h, "FREQ?"), b"5000\n");
        assert_eq!(ask(&mut h, "POW?"), b"-144\n");
    }

    #[test]
    fn the_ranges_of_the_models() {
        for (model, top, over) in [
            ("SME02", "1500000000", "1500000000.1"),
            ("SME03E", "2200000000", "2200000000.1"),
            ("SME03A", "3000000000", "3000000000.1"),
            ("SME03", "3000000000", "3000000000.1"),
            ("SME06", "6000000000", "6000000000.1"),
        ] {
            let mut h = remote(&[("model", model)]);
            ask(&mut h, &format!("FREQ {top}.0Hz"));
            assert_eq!(ask(&mut h, "FREQ?"), format!("{top}\n").as_bytes(), "{model}");
            ask(&mut h, "FREQ 5000.0Hz");
            ask(&mut h, &format!("FREQ {over}Hz"));
            assert_eq!(ask(&mut h, "FREQ?"), b"5000\n", "{model}");
        }
    }

    #[test]
    fn checks_fire() {
        // A query before the remote line, each kind.
        for query in ["*IDN?", "FREQ?", "POW?"] {
            let mut h = generator(&[]);
            ask(&mut h, query);
            assert_eq!(violations(&mut h), 1, "{query}");
        }
        // Lines no path sends.
        for line in [
            "freq?",
            "FREQ 1000",
            "FREQ 1000kHz",
            "POW 1",
            "POW 1dB",
            "*RST",
            "FREQ?;POW?",
            "FREQ 1.0Hz ",
        ] {
            let mut h = remote(&[]);
            assert_eq!(ask(&mut h, line), b"", "{line}");
            assert_eq!(violations(&mut h), 1, "{line}");
        }
        let mut h = remote(&[]);
        h.write(&[0xff, b'\n']);
        assert_eq!(violations(&mut h), 1);
        // Too long, then on.
        let mut h = remote(&[]);
        h.write(&[b'x'; LONGEST + 1]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn the_remote_line_stays_across_an_open() {
        let mut h = remote(&[]);
        h.set_lines(true, true);
        assert_eq!(ask(&mut h, "POW?"), b"-30\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_line_left_at_an_open_is_flagged() {
        let mut h = remote(&[]);
        h.write(b"FREQ");
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        // It went with the open.
        assert_eq!(ask(&mut h, "POW?"), b"-30\n");
    }

    #[test]
    fn options_change_the_replies() {
        let mut h = remote(&[("model", "SME06"), ("eol", "crlf")]);
        assert_eq!(ask(&mut h, "*IDN?"), b"Rohde&Schwarz,SME06,000000/000,1.00\r\n");
        let mut h = remote(&[("idn", "a,b"), ("eol", "lfcr"), ("freq", "5e3"), ("pow", "1.5")]);
        assert_eq!(ask(&mut h, "*IDN?"), b"a,b\n\r");
        assert_eq!(ask(&mut h, "FREQ?"), b"5000\n\r");
        assert_eq!(ask(&mut h, "POW?"), b"1.5\n\r");
        let mut h =
            remote(&[("rawidn", "410a"), ("rawfreq", "420a"), ("rawpow", "none")]);
        assert_eq!(ask(&mut h, "*IDN?"), b"A\n");
        assert_eq!(ask(&mut h, "FREQ?"), b"B\n");
        assert_eq!(ask(&mut h, "POW?"), b"");
    }

    #[test]
    fn answers_then_silence() {
        let mut h = remote(&[("answers", "1")]);
        assert_ne!(ask(&mut h, "*IDN?").len(), 0);
        assert_eq!(ask(&mut h, "FREQ?"), b"100000000\n");
        assert_eq!(ask(&mut h, "POW?"), b"");
        assert_eq!(ask(&mut h, "FREQ?"), b"");
    }

    #[test]
    fn shift_answers_the_one_before() {
        let mut h = remote(&[("shift", "1")]);
        assert_eq!(ask(&mut h, "FREQ?"), b"");
        assert_eq!(ask(&mut h, "POW?"), b"100000000\n");
        assert_eq!(ask(&mut h, "FREQ?"), b"-30\n");
    }

    #[test]
    fn options_refused() {
        for (k, v) in [
            ("nothing", "1"),
            ("eol", "cr"),
            ("freq", "x"),
            ("pow", "x"),
            ("ids", "0aad"),
            ("ids", "x:1"),
            ("shift", "2"),
            ("rawfreq", "zz"),
        ] {
            assert!(function(&[(k.to_owned(), v.to_owned())]).is_err(), "{k}={v}");
        }
        let c = function(&[("ids".to_owned(), "0aad:0117".to_owned())]);
        assert!(c.is_ok());
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut g = Generator::new(Config::default());
        g.remote = true;
        g.freq = 5e3;
        g.queries = 3;
        g.reset();
        assert!(!g.remote);
        assert!((g.freq - 100e6).abs() < 1.0);
        assert_eq!(g.queries, 0);
    }
}
