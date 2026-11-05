// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Motech LPS-301 bench supply on the CDC-ACM shell: one channel, ASCII
//! lines, and every answer closed by a line saying `OK`.
//!
//! The device, and how sure this is:
//!
//! - The commands, the line and the status bits: the LPS-300 series manual,
//!   chapter 5 (the copy on mikrocontroller.net, attachment 118074, read page
//!   by page as it is a scan). 2400 baud, 8n1. A command ends in CR, LF or both,
//!   one to a string, in either case. A command before the `OK` prompt is
//!   rejected. `STATUS` is a decimal number whose bit 6 is the output, bit 8 the
//!   fan, bit 9 the beeper. `VSET1 12.345` and `ISET2 1.23` are its examples.
//! - The limits: the same manual's table gives the LPS-301 32 V and 2.4 A peak.
//! - That an answer is its line and then `OK`, and that `MODEL` starts `LPS-`
//!   and `VERSION` starts `Ver-`: what libsigrok's driver expects of it, from a
//!   driver written against real units. Firmware 1.17 is said there to answer
//!   `LPS-` with no number, and to fail `VERSION` while the output is on; how it
//!   fails is not said (`version=ERROR` plays one reading).
//! - The line ends: the same driver says "up to 4 in a row", and reads up to
//!   five lines for one answer. So each line here ends CR LF and has one empty
//!   line after it (`blank=1`), a guess that fits that and nothing else.
//! - The width of a reading (`12.000`, `0.2500`): a guess, from the manual's
//!   example and the driver's formats. No recording was found.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id, where the real one is
//!   a 9 pin RS-232 port: only the byte stream and the line settings are the
//!   device's.
//! - It answers at once, so the manual's rejection of a command before `OK`
//!   cannot happen here.
//! - It has no load: the current reads 0 unless `iout1=` says otherwise.
//! - `model=`, `version=`, `status=`, `vout1=`, `iout1=` and `set=` are stand-ins
//!   for a load that changes and for a device gone wrong.
//!
//! An answer in those lists is one of:
//!
//! - `TEXT`: that line and `OK`; `a|b` is two lines, and an empty one counts;
//! - `-`: `OK` alone;
//! - `TEXT^`: the lines without `OK`;
//! - `TEXT~`: those bytes and nothing else, so `~` alone is no answer.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `model=`, `version=`, `status=`, `vout1=`, `iout1=`, `set=`, lists of
//!   answers separated by commas, played in turn and then again;
//! - `uart=LINE`, `eol=cr|lf|crlf`, `blank=N` empty lines after each line;
//! - `output=0|1`, `v1=`, `i1=`, the state at power on, as `5.000` and `0.2500`;
//! - `answers=N`, commands answered per power on, then silence and the event
//!   point;
//! - `setters=0`, which flags any setting.
//!
//! The pedantic checks: only the commands the driver has for an LPS-301, each
//! ended by CR LF, in the format C's `%05.3f` and `%05.4f` give, within the
//! limits; nothing left of a command when a tool opens. `TRACK`, `VDD`, `BEEP`,
//! `CALI`, `HELP`, `LOWA` and channels 2 and 3 are real commands of the series
//! that no path to an LPS-301 sends, so they are flagged and answered `ERROR`,
//! which is itself a guess. C habits are taken, as its range check is two
//! comparisons and nothing shows a refusal: `-0.000` as 0, `  nan` as nothing,
//! and a current under the 5 mA its table lists.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "motech-lps-30x";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Motech LPS-301 supply on CDC-ACM; model=, version=, status=, vout1=, \
                         iout1=, set= (answers in turn), uart=, eol=, blank=, output=, v1=, i1=, \
                         answers=, setters=, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

/// More than the longest command; what is longer is flagged.
const LONGEST: usize = 32;
/// The manual's peak output, in millivolts.
const VOLTS_MAX: u32 = 32_000;
/// The manual's peak output, in tenths of a milliampere.
const AMPS_MAX: u32 = 24_000;
/// `blank=`'s bound.
const BLANK_MAX: u32 = 16;

/// A supply from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn flag(key: &str, value: &str) -> Result<bool> {
    Ok(parse_number(key, value, 0..=1)? == 1)
}

fn answers_list(value: &str) -> Packets {
    Packets::new(value.split(',').map(|t| t.as_bytes().to_vec()).collect())
}

fn set_point(key: &str, value: &str, decimals: usize) -> Result<u32> {
    parse_fixed(value, decimals)
        .ok_or_else(|| Error::Invalid(format!("{key}={value}: with {decimals} decimals")))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Supply>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("2400/8n1")?;
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => config.lists.model = answers_list(v),
            "version" => config.lists.version = answers_list(v),
            "status" => config.lists.status = Some(answers_list(v)),
            "vout1" => config.lists.vout = Some(answers_list(v)),
            "iout1" => config.lists.iout = Some(answers_list(v)),
            "set" => config.lists.set = Some(answers_list(v)),
            "uart" => uart = Line::parse(v)?,
            "eol" => {
                config.eol = match v.as_str() {
                    "cr" => b"\r",
                    "lf" => b"\n",
                    "crlf" => b"\r\n",
                    _ => return Err(Error::Invalid(format!("eol={v}: cr, lf or crlf"))),
                };
            }
            "blank" => config.blank = parse_number(k, v, 0..=BLANK_MAX)?,
            "output" => config.power_on.output = flag(k, v)?,
            "v1" => config.power_on.millivolts = set_point(k, v, 3)?,
            "i1" => config.power_on.tenth_milliamps = set_point(k, v, 4)?,
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "setters" => config.setters = flag(k, v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "LPS-301 power supply");
    Ok(Cdc::new(NAME, &id, faults, Supply::new(config)).behind_uart(uart))
}

#[derive(Clone, Copy, Debug)]
struct Panel {
    millivolts: u32,
    tenth_milliamps: u32,
    output: bool,
}

/// Answers played in turn instead of the device's own.
#[derive(Clone, Debug)]
struct Lists {
    model: Packets,
    version: Packets,
    status: Option<Packets>,
    vout: Option<Packets>,
    iout: Option<Packets>,
    set: Option<Packets>,
}

#[derive(Clone, Debug)]
struct Config {
    lists: Lists,
    eol: &'static [u8],
    blank: u32,
    power_on: Panel,
    answers: Option<u32>,
    setters: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            lists: Lists {
                model: answers_list("LPS-301"),
                version: answers_list("Ver-2.10"),
                status: None,
                vout: None,
                iout: None,
                set: None,
            },
            eol: b"\r\n",
            blank: 1,
            power_on: Panel { millivolts: 12_000, tenth_milliamps: 10_000, output: false },
            answers: None,
            setters: true,
        }
    }
}

struct Supply {
    config: Config,
    now: Panel,
    lists: Lists,
    /// What has come of a line so far.
    pending: Vec<u8>,
    /// A CR came, so the LF is due.
    after_cr: bool,
    answered: u32,
}

impl Supply {
    fn new(config: Config) -> Supply {
        Supply {
            now: config.power_on,
            lists: config.lists.clone(),
            pending: Vec::new(),
            after_cr: false,
            config,
            answered: 0,
        }
    }

    fn status(&self) -> u32 {
        /// Bit 9: the beeper, on as at power on. The fan's bit 8 stays off.
        const BEEPER: u32 = 1 << 9;
        BEEPER | (u32::from(self.now.output) << 6)
    }

    fn end_line(&self, text: &[u8], out: &mut Vec<u8>) {
        out.extend_from_slice(text);
        out.extend_from_slice(self.config.eol);
        for _ in 0..self.config.blank {
            out.extend_from_slice(self.config.eol);
        }
    }

    /// An answer, in the form the module's documentation has.
    fn say(&self, answer: &[u8], port: &mut Port<'_>) {
        if let Some(raw) = answer.strip_suffix(b"~") {
            port.send(raw);
            return;
        }
        let (lines, closed) = match answer.strip_suffix(b"^") {
            Some(lines) => (lines, false),
            None => (answer, true),
        };
        let mut out = Vec::new();
        if lines != b"-" {
            for line in lines.split(|c| *c == b'|') {
                self.end_line(line, &mut out);
            }
        }
        if closed {
            self.end_line(b"OK", &mut out);
        }
        port.send(&out);
    }

    fn query(&mut self, command: Query, port: &mut Port<'_>) {
        let output = self.now.output;
        let text = match command {
            Query::Model => self.lists.model.take(),
            Query::Version => self.lists.version.take(),
            Query::Status => {
                let own = self.status().to_string().into_bytes();
                self.lists.status.as_mut().map_or(own, Packets::take)
            }
            Query::Volts => {
                let millivolts = if output { self.now.millivolts } else { 0 };
                let own = format!("{}.{:03}", millivolts / 1000, millivolts % 1000).into_bytes();
                self.lists.vout.as_mut().map_or(own, Packets::take)
            }
            Query::Amps => {
                let own = b"0.0000".to_vec();
                self.lists.iout.as_mut().map_or(own, Packets::take)
            }
        };
        self.say(&text, port);
    }

    fn setting(&mut self, setting: Setting, text: &str, port: &mut Port<'_>) {
        if !self.config.setters {
            port.violation(format!("'{text}': a setting no one asked for"));
        }
        match setting {
            Setting::Output(on) => self.now.output = on,
            Setting::Volts(Some(v)) => self.now.millivolts = v,
            Setting::Amps(Some(a)) => self.now.tenth_milliamps = a,
            Setting::Volts(None) | Setting::Amps(None) => {}
        }
        let text = self.lists.set.as_mut().map_or_else(|| b"-".to_vec(), Packets::take);
        self.say(&text, port);
    }

    fn line(&mut self, line: &[u8], port: &mut Port<'_>) {
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return;
        }
        self.answered += 1;
        let text = String::from_utf8_lossy(line).into_owned();
        match parse(&text) {
            Parsed::Query(query) => self.query(query, port),
            Parsed::Setting(setting) => self.setting(setting, &text, port),
            Parsed::Bad(why) => {
                port.violation(format!("'{}': {why}", show(line)));
                self.say(b"ERROR", port);
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Query {
    Model,
    Version,
    Status,
    Volts,
    Amps,
}

/// A setting; a value of `None` is the C habit of a NaN, which changes nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Setting {
    Output(bool),
    Volts(Option<u32>),
    Amps(Option<u32>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Parsed {
    Query(Query),
    Setting(Setting),
    Bad(&'static str),
}

/// `decimals` exactly, in units of the last one: `5.000` is 5000.
fn parse_fixed(text: &str, decimals: usize) -> Option<u32> {
    let (whole, part) = text.split_once('.')?;
    let digits = |t: &str| !t.is_empty() && t.bytes().all(|c| c.is_ascii_digit());
    if !digits(whole) || !digits(part) || part.len() != decimals || whole.len() > 2 {
        return None;
    }
    let scale = 10u32.pow(u32::try_from(decimals).ok()?);
    Some(whole.parse::<u32>().ok()? * scale + part.parse::<u32>().ok()?)
}

/// A set point as C's `%05.3f` or `%05.4f` prints it, with its habits.
fn set_value(text: &str, decimals: usize, max: u32) -> std::result::Result<Option<u32>, &'static str> {
    if text.trim_start_matches(' ').trim_start_matches('-') == "nan" && text.len() == 5 {
        return Ok(None);
    }
    if text.strip_prefix('-').is_some_and(|rest| parse_fixed(rest, decimals) == Some(0)) {
        return Ok(Some(0));
    }
    match parse_fixed(text, decimals) {
        Some(v) if v <= max => Ok(Some(v)),
        Some(_) => Err("above the limit"),
        None => Err("not as the driver prints a value"),
    }
}

fn parse(text: &str) -> Parsed {
    match text {
        "MODEL" => return Parsed::Query(Query::Model),
        "VERSION" => return Parsed::Query(Query::Version),
        "STATUS" => return Parsed::Query(Query::Status),
        "VOUT1" => return Parsed::Query(Query::Volts),
        "IOUT1" => return Parsed::Query(Query::Amps),
        "OUT0" => return Parsed::Setting(Setting::Output(false)),
        "OUT1" => return Parsed::Setting(Setting::Output(true)),
        _ => {}
    }
    if let Some(value) = text.strip_prefix("VSET1 ") {
        return match set_value(value, 3, VOLTS_MAX) {
            Ok(v) => Parsed::Setting(Setting::Volts(v)),
            Err(why) => Parsed::Bad(why),
        };
    }
    if let Some(value) = text.strip_prefix("ISET1 ") {
        return match set_value(value, 4, AMPS_MAX) {
            Ok(v) => Parsed::Setting(Setting::Amps(v)),
            Err(why) => Parsed::Bad(why),
        };
    }
    Parsed::Bad("nothing the driver sends to an LPS-301")
}

fn show(bytes: &[u8]) -> String {
    bytes.escape_ascii().to_string()
}

impl Serial for Supply {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            let after_cr = std::mem::take(&mut self.after_cr);
            match byte {
                b'\n' if after_cr => {
                    let line = std::mem::take(&mut self.pending);
                    self.line(&line, port);
                }
                b'\n' => {
                    port.violation(format!("'{}': LF with no CR", show(&self.pending)));
                    self.pending.clear();
                }
                _ if after_cr => {
                    port.violation(format!("'{}': CR with no LF", show(&self.pending)));
                    self.pending.clear();
                    self.pending.push(byte);
                }
                b'\r' => self.after_cr = true,
                _ if self.pending.len() >= LONGEST => {
                    port.violation(format!("'{}': a line too long", show(&self.pending)));
                    self.pending.clear();
                }
                _ => self.pending.push(byte),
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && (!self.pending.is_empty() || self.after_cr) {
            port.violation(format!("'{}': a command left unfinished", show(&self.pending)));
            self.pending.clear();
            self.after_cr = false;
        }
    }

    fn reset(&mut self) {
        *self = Supply::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn supply(options: &[(&str, &str)]) -> Host<Supply> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let line = Line::parse("2400/8n1").unwrap();
        Host::open(function(&o).unwrap(), line, true, true)
    }

    fn violations(h: &mut Host<Supply>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn the_queries_have_their_answers_and_up_to_four_line_ends_in_a_row() {
        let mut h = supply(&[]);
        for (ask, want) in [
            (&b"MODEL\r\n"[..], &b"LPS-301\r\n\r\nOK\r\n\r\n"[..]),
            (b"VERSION\r\n", b"Ver-2.10\r\n\r\nOK\r\n\r\n"),
            (b"STATUS\r\n", b"512\r\n\r\nOK\r\n\r\n"),
            (b"VOUT1\r\n", b"0.000\r\n\r\nOK\r\n\r\n"),
            (b"IOUT1\r\n", b"0.0000\r\n\r\nOK\r\n\r\n"),
        ] {
            h.write(ask);
            assert_eq!(h.read(), want, "{}", show(ask));
        }
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn settings_take_effect_and_answer_ok() {
        let mut h = supply(&[("blank", "0")]);
        h.write(b"VSET1 05.500\r\nISET1 0.2500\r\nOUT1\r\n");
        assert_eq!(h.read(), b"OK\r\nOK\r\nOK\r\n");
        h.write(b"VOUT1\r\nSTATUS\r\nOUT0\r\nSTATUS\r\nVOUT1\r\n");
        assert_eq!(h.read(), b"5.500\r\nOK\r\n576\r\nOK\r\nOK\r\n512\r\nOK\r\n0.000\r\nOK\r\n");
        assert_eq!(h.cdc.model.now.tenth_milliamps, 2500);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_answer_forms() {
        let mut h = supply(&[("blank", "0"), ("vout1", "1.5,-,x^,a|b,|c,2~,~")]);
        for want in [
            &b"1.5\r\nOK\r\n"[..],
            b"OK\r\n",
            b"x\r\n",
            b"a\r\nb\r\nOK\r\n",
            b"\r\nc\r\nOK\r\n",
            b"2",
            b"",
            b"1.5\r\nOK\r\n",
        ] {
            h.write(b"VOUT1\r\n");
            assert_eq!(h.read(), want);
        }
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn line_ends_and_blanks_are_options() {
        let mut h = supply(&[("eol", "cr"), ("blank", "2")]);
        h.write(b"OUT1\r\n");
        assert_eq!(h.read(), b"OK\r\r\r");
        let mut h = supply(&[("eol", "lf"), ("blank", "0")]);
        h.write(b"MODEL\r\n");
        assert_eq!(h.read(), b"LPS-301\nOK\n");
    }

    #[test]
    fn what_no_path_to_an_lps_301_sends_is_flagged_and_gets_error() {
        for bad in [
            &b"TRACK0"[..],
            b"VDD5",
            b"BEEP1",
            b"CALI0",
            b"HELP",
            b"LOWA1",
            b"VOUT2",
            b"IOUT3",
            b"VSET2 1.000",
            b"ISET3 1.0000",
            b"model",
            b"",
            b"OUT2",
        ] {
            let mut h = supply(&[("blank", "0")]);
            h.write(bad);
            h.write(b"\r\n");
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            assert_eq!(h.read(), b"ERROR\r\nOK\r\n");
        }
    }

    #[test]
    fn a_setting_in_another_format_is_flagged_and_changes_nothing() {
        for bad in [
            &b"VSET1 5.0"[..],
            b"VSET1 5.0000",
            b"VSET1 32.001",
            b"VSET1 abcde",
            b"VSET1 inf",
            b"VSET1 100.000",
            b"VSET1",
            b"ISET1 0.25",
            b"ISET1 0.250",
            b"ISET1 2.4001",
            b"ISET1 -1.0000",
        ] {
            let mut h = supply(&[("blank", "0")]);
            h.write(bad);
            h.write(b"\r\n");
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            assert_eq!(h.read(), b"ERROR\r\nOK\r\n");
            assert_eq!(h.cdc.model.now.millivolts, 12_000);
            assert_eq!(h.cdc.model.now.tenth_milliamps, 10_000);
        }
    }

    #[test]
    fn the_c_habits_are_taken() {
        let mut h = supply(&[("blank", "0")]);
        h.write(b"VSET1   nan\r\nISET1   nan\r\nVSET1  -nan\r\n");
        assert_eq!(violations(&mut h), 0);
        assert_eq!(h.cdc.model.now.millivolts, 12_000);
        h.write(b"VSET1 -0.000\r\nISET1 -0.0000\r\n");
        assert_eq!(violations(&mut h), 0);
        assert_eq!(h.cdc.model.now.millivolts, 0);
        assert_eq!(h.cdc.model.now.tenth_milliamps, 0);
        h.write(b"ISET1 0.0010\r\nVSET1 32.000\r\nISET1 2.4000\r\n");
        assert_eq!(violations(&mut h), 0);
        assert_eq!(h.cdc.model.now.millivolts, 32_000);
    }

    #[test]
    fn line_ends_other_than_cr_lf_are_flagged() {
        let mut h = supply(&[]);
        h.write(b"MODEL\n");
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.read(), b"");
        let mut h = supply(&[]);
        h.write(b"MODEL\rVERSION\r\n");
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.read(), b"Ver-2.10\r\n\r\nOK\r\n\r\n");
    }

    #[test]
    fn setters_0_flags_each_setting() {
        for setting in [&b"OUT0\r\n"[..], b"OUT1\r\n", b"VSET1 1.000\r\n", b"ISET1 1.0000\r\n"] {
            let mut h = supply(&[("setters", "0")]);
            h.write(setting);
            assert_eq!(violations(&mut h), 1, "{}", show(setting));
        }
        let mut h = supply(&[("setters", "0")]);
        h.write(b"STATUS\r\nMODEL\r\nVERSION\r\nVOUT1\r\nIOUT1\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn set_answers_in_turn() {
        let mut h = supply(&[("blank", "0"), ("set", "ERROR,x,~")]);
        for want in [&b"ERROR\r\nOK\r\n"[..], b"x\r\nOK\r\n", b""] {
            h.write(b"OUT1\r\n");
            assert_eq!(h.read(), want);
        }
    }

    #[test]
    fn a_long_line_is_flagged_and_what_is_kept_stays_short() {
        let mut h = supply(&[]);
        h.write(&[b'x'; 4096]);
        assert!(h.cdc.model.pending.len() <= LONGEST);
        assert!(violations(&mut h) > 0);
    }

    #[test]
    fn a_command_left_unfinished_shows_at_the_next_open() {
        let mut h = supply(&[]);
        h.write(b"VSET");
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        h.write(b"MODEL\r");
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = supply(&[("answers", "1")]);
        h.write(b"MODEL\r\n");
        assert_ne!(h.read(), b"");
        assert!(!h.cdc.mark());
        h.write(b"VERSION\r\n");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn behind_its_uart_a_wrong_rate_hears_nothing() {
        let o = vec![];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true);
        h.write(b"MODEL\r\n");
        assert_eq!(h.read(), b"");
        h.set_line(Line::parse("2400/8n1").unwrap());
        h.write(b"MODEL\r\n");
        assert_ne!(h.read(), b"");
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = supply(&[("vout1", "1,2"), ("answers", "3"), ("output", "1")]);
        h.write(b"VSET1 1.000\r\nVOUT1\r\nOUT0\r\n");
        h.read();
        h.cdc.model.reset();
        h.write(b"VOUT1\r\nSTATUS\r\n");
        assert_eq!(h.read(), b"1\r\n\r\nOK\r\n\r\n576\r\n\r\nOK\r\n\r\n");
        assert_eq!(h.cdc.model.now.millivolts, 12_000);
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("v1", "1.5"),
            ("i1", "1.000"),
            ("output", "2"),
            ("eol", "x"),
            ("blank", "17"),
            ("uart", "2400"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("vout1", "x"), ("uart", "9600/8n1"), ("v1", "5.000")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
