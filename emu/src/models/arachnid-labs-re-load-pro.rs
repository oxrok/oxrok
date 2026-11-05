// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An Arachnid Labs Re:load Pro electronic load on the CDC-ACM shell: lines
//! of text at 115200/8n1, a request and its answer, and with monitoring on a
//! reading every so often.
//!
//! The device, and how sure this is:
//!
//! - The commands and their answers: the firmware's own source,
//!   `arachnidlabs/reload-pro` at 0b7adce (`firmware/Reload Pro.cydsn/`,
//!   `comms.c`, `utils.c`, `config.h`, `tasks.h`), the newest there is, v1.11.
//!   A line ends at CR or LF, empty lines and NULs count for nothing, and a
//!   word and its argument are split at the first space. Every answer ends in
//!   CR LF. `read` gives four numbers: milliamperes, millivolts, microampere
//!   hours and microwatt hours. `monitor` has no answer. `set` and `uvlo` with
//!   no argument tell the value, and with one out of range give an `err` line
//!   and then the same line as ever. `uvlo 0` turns the limit off, which then
//!   reads 0. An unknown word gives `err Unknown command`.
//! - `set` with no argument and `uvlo`: the changelog says both came with
//!   v1.9, so `uvlo` is unknown to a `version=` below that. `on` and `off`: the
//!   changelog's v1.10 speaks of toggling the load, and whether a serial
//!   command came with it was not examined, so `replies=` plays their refusal.
//! - `set` with no argument hands the firmware's handler a null argument,
//!   and what its address 0 holds decides what it does. The changelog says
//!   it answers, so this answers.
//! - The USB side: an FT230X, `0403:6015`, "Arachnid Labs Ltd", "Re:load Pro"
//!   (`firmware/Reload Pro FTDI Config.xml`), between USB and the chip's UART,
//!   so the line must fit. Its rate is the C driver's, and the default of the
//!   project's `tools/calibrate.py`. The kernel's `ftdi_sio` binds those ids
//!   whatever the class, so this has a stand-in's ids and strings.
//!
//! Emulation artifacts:
//!
//! - It answers at once. The firmware's queue of commands holds one, and a line
//!   that comes before the last was taken is lost, hence the C driver's wait
//!   in its scan. A model has no clock, so nothing here checks pacing; the
//!   port's is checked by a unit test.
//! - A monitoring interval is counted in [`Serial::tick`] offers, about 20 ms
//!   each, and the first line comes one interval after the command, as in the
//!   firmware. `count=` holds the stream after so many lines, where a tool's
//!   stop would otherwise race the next line.
//! - `volts=` and `amps=` are stand-ins for what is on the terminals. There is
//!   no analog side, so `overtemp` and `undervolt` come only when `lines=`
//!   plays them, and nothing turns off with them.
//!
//! Not modelled: `reset`, `clear`, `mode`, `debug`, `dump`, `cal` and `bl`,
//! which the firmware has and no path of the C driver sends, so they are
//! flagged. The panel, whose knob sends `set N` by itself, which `lines=` and
//! `stale=` can play.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `version=TEXT`, what follows `version `, or `none` for no answer, as a
//!   firmware built without its splash screen gives;
//! - `banner=HEX`, the bytes to send for `version` instead of its answer;
//! - `stale=HEX`, bytes sent at the first `monitor 0`: a line on its way when
//!   a tool stops the monitoring that an earlier one left on;
//! - `set=MA`, `uvlo=MV`, `on=0|1`, the state at power on;
//! - `volts=MV`, `amps=MA`, the reading; `amps=` counts while it is on, and
//!   with no `amps=` the current is the setpoint;
//! - `replies=HEX[,HEX...]`, answers played in turn to what comes after the
//!   `version`, instead of the usual one, an empty one being none;
//! - `answers=N`, requests answered after the `version`, then silence and the
//!   event point;
//! - `lines=HEX[,HEX...]`, what monitoring sends first, in turn, ahead of its
//!   readings;
//! - `count=N`, the lines monitoring sends per power on, then silence and the
//!   event point on the next offer;
//! - `period=N`, offers from line to line instead of the interval's own;
//! - `gap=LINE:POS:OFFERS`, monitoring's line number LINE (from 1) stops after
//!   POS bytes for that many offers;
//! - `setters=0`, which flags any setting, for the cases that show a tool
//!   changes nothing it was not asked to.
//!
//! The pedantic checks: only the words above, `monitor` only with 0 or 200,
//! `set` and `uvlo` with a whole number in range, nothing left of a line when
//! a tool opens, no line longer than the firmware's 40 byte buffer (it has no
//! check of its own). One C habit is taken: `-2147483648`, what a NaN comes to
//! on x86, which the firmware's range check refuses by itself.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "arachnid-labs-re-load-pro";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "An Arachnid Labs Re:load Pro electronic load on CDC-ACM; \
                         version=TEXT|none, banner=, stale=, set=, uvlo=, on=, volts=, amps=, \
                         replies=, answers=, lines=, count=, period=, gap=, setters=, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// The firmware's 40 byte line buffer, less the NUL.
const MAX_LINE: usize = 39;
/// The most `set` takes, in milliamperes.
const SET_MAX: i64 = 6000;
/// The most `uvlo` takes, in millivolts.
const UVLO_MAX: i64 = 60000;
/// What a NaN comes to on x86.
const C_NAN: i64 = -2_147_483_648;
/// One offer is about this many milliseconds.
const MS_PER_OFFER: u32 = 20;
/// The one interval the C driver asks for.
const INTERVAL_MS: u32 = 200;

/// A load from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn parse_gap(value: &str) -> Result<Gap> {
    let invalid = || Error::Invalid(format!("gap={value}: LINE:POS:OFFERS"));
    let parts: Vec<&str> = value.split(':').collect();
    let [line, pos, offers] = parts[..] else { return Err(invalid()) };
    let number = |text: &str| text.parse::<u32>().map_err(|_| invalid());
    let (line, pos, offers) = (number(line)?, number(pos)?, number(offers)?);
    if line == 0 || pos == 0 {
        return Err(invalid());
    }
    Ok(Gap { line, pos: pos as usize, offers })
}

fn function(options: &[(String, String)]) -> Result<Cdc<Load>> {
    let mut faults = Faults::default();
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "version" if v == "none" => config.version = None,
            "version" => config.version = Some(v.clone()),
            "banner" => config.banner = Some(parse_hex_list(k, v, 1, 256)?.remove(0)),
            "stale" => config.stale = Some(parse_hex_list(k, v, 1, 256)?.remove(0)),
            "set" => config.set = parse_number(k, v, 0..=6000)?,
            "uvlo" => config.uvlo = parse_number(k, v, 0..=60000)?,
            "on" => config.on = parse_number(k, v, 0..=1)? == 1,
            "volts" => config.volts = parse_number(k, v, 0..=100_000)?,
            "amps" => config.amps = Some(parse_number(k, v, 0..=100_000)?),
            "replies" => config.replies = parse_hex_list(k, v, 0, 256)?,
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "lines" => config.lines = parse_hex_list(k, v, 1, 256)?,
            "count" => config.count = Some(parse_number(k, v, 0..=100_000)?),
            "period" => config.period = Some(parse_number(k, v, 1..=100_000)?),
            "gap" => config.gap = Some(parse_gap(v)?),
            "setters" => config.setters = parse_number(k, v, 0..=1)? == 1,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Re:load Pro stand-in");
    Ok(Cdc::new(NAME, &id, faults, Load::new(config)).behind_uart(Line::parse("115200/8n1")?))
}

#[derive(Clone, Copy, Debug)]
struct Gap {
    line: u32,
    pos: usize,
    offers: u32,
}

#[derive(Clone, Debug)]
struct Config {
    version: Option<String>,
    banner: Option<Vec<u8>>,
    stale: Option<Vec<u8>>,
    /// Milliamperes.
    set: u32,
    /// Millivolts.
    uvlo: u32,
    on: bool,
    /// Millivolts.
    volts: u32,
    /// Milliamperes.
    amps: Option<u32>,
    replies: Vec<Vec<u8>>,
    answers: Option<u32>,
    lines: Vec<Vec<u8>>,
    count: Option<u32>,
    period: Option<u32>,
    gap: Option<Gap>,
    setters: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            version: Some("1.11".to_owned()),
            banner: None,
            stale: None,
            set: 0,
            uvlo: 0,
            on: true,
            volts: 12000,
            amps: None,
            replies: Vec::new(),
            answers: None,
            lines: Vec::new(),
            count: None,
            period: None,
            gap: None,
            setters: true,
        }
    }
}

struct Load {
    config: Config,
    set: u32,
    uvlo: u32,
    on: bool,
    /// What has come of a line so far.
    pending: Vec<u8>,
    /// Offers from one monitoring line to the next, when it is on.
    period: Option<u32>,
    offers: u32,
    /// Monitoring's lines so far.
    sent: u32,
    /// The rest of a line in a [`Gap`], and the offers until it goes.
    tail: Option<(Vec<u8>, u32)>,
    replies_used: usize,
    answered: u32,
    stale_sent: bool,
}

impl Load {
    fn new(config: Config) -> Load {
        Load {
            set: config.set,
            uvlo: config.uvlo,
            on: config.on,
            config,
            pending: Vec::new(),
            period: None,
            offers: 0,
            sent: 0,
            tail: None,
            replies_used: 0,
            answered: 0,
            stale_sent: false,
        }
    }

    fn reading(&self) -> Vec<u8> {
        let amps = if self.on { self.config.amps.unwrap_or(self.set) } else { 0 };
        format!("read {amps} {} 0 0\r\n", self.config.volts).into_bytes()
    }

    /// `uvlo` needs v1.9, by the changelog.
    fn knows_uvlo(&self) -> bool {
        let Some(version) = &self.config.version else { return true };
        let mut parts = version.split('.').map(str::parse::<u32>);
        match (parts.next(), parts.next()) {
            (Some(Ok(major)), Some(Ok(minor))) => (major, minor) >= (1, 9),
            _ => true,
        }
    }

    /// An answer owed to a request after the `version`: what `replies=` has
    /// next, if anything, or the usual one, unless `answers=` has run out.
    fn answer(&mut self, usual: &[u8], port: &mut Port<'_>) {
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return;
        }
        self.answered += 1;
        match self.config.replies.get(self.replies_used) {
            Some(reply) => {
                self.replies_used += 1;
                port.send(reply);
            }
            None => port.send(usual),
        }
    }

    /// The argument of `set` or `uvlo`, in range or as the C habit has it.
    fn value(word: &str, text: &str, max: i64, port: &mut Port<'_>) -> Option<i64> {
        match text.parse::<i64>() {
            Ok(v) if (0..=max).contains(&v) || v == C_NAN => Some(v),
            _ => {
                port.violation(format!("'{word} {text}': not a whole number 0 to {max}"));
                None
            }
        }
    }

    fn setting(&self, line: &str, port: &mut Port<'_>) {
        if !self.config.setters {
            port.violation(format!("'{line}': a setting no one asked for"));
        }
    }

    fn command(&mut self, line: &[u8], port: &mut Port<'_>) {
        let text = String::from_utf8_lossy(line).into_owned();
        let (word, argument) = match text.split_once(' ') {
            Some((word, argument)) => (word, Some(argument)),
            None => (text.as_str(), None),
        };
        match (word, argument) {
            ("version", None) => {
                if let Some(banner) = &self.config.banner {
                    port.send(banner);
                } else if let Some(version) = &self.config.version {
                    port.send(format!("version {version}\r\n").as_bytes());
                }
            }
            ("monitor", Some("0")) => {
                self.period = None;
                if let (Some(stale), false) = (&self.config.stale, self.stale_sent) {
                    self.stale_sent = true;
                    port.send(stale);
                }
            }
            ("monitor", Some("200")) => {
                self.period = Some(self.config.period.unwrap_or(INTERVAL_MS / MS_PER_OFFER));
                self.offers = 0;
            }
            ("read", None) => self.answer(&self.reading(), port),
            ("set", None) => self.answer(format!("set {}\r\n", self.set).as_bytes(), port),
            ("set", Some(argument)) => {
                self.setting(&text, port);
                let Some(v) = Load::value(word, argument, SET_MAX, port) else { return };
                let mut reply = Vec::new();
                match u32::try_from(v) {
                    Ok(ma) => self.set = ma,
                    Err(_) => {
                        reply.extend_from_slice(b"err set current must be between 0 and 6000\r\n");
                    }
                }
                reply.extend_from_slice(format!("set {}\r\n", self.set).as_bytes());
                self.answer(&reply, port);
            }
            ("uvlo", _) if !self.knows_uvlo() => {
                self.answer(b"err Unknown command 'uvlo'\r\n", port);
            }
            ("uvlo", None) => self.answer(format!("uvlo {}\r\n", self.uvlo).as_bytes(), port),
            ("uvlo", Some(argument)) => {
                self.setting(&text, port);
                let Some(v) = Load::value(word, argument, UVLO_MAX, port) else { return };
                let mut reply = Vec::new();
                match u32::try_from(v) {
                    Ok(mv) => self.uvlo = mv,
                    Err(_) => {
                        reply.extend_from_slice(b"err uvlo must be between 0 and 60000\r\n");
                    }
                }
                reply.extend_from_slice(format!("uvlo {}\r\n", self.uvlo).as_bytes());
                self.answer(&reply, port);
            }
            ("on" | "off", None) => {
                self.setting(&text, port);
                self.on = word == "on";
                self.answer(b"ok\r\n", port);
            }
            _ => port.violation(format!("'{text}': no path of the C driver sends this")),
        }
    }

    /// The next of monitoring's lines goes, or its first part.
    fn monitor_line(&mut self, port: &mut Port<'_>) {
        let index = self.sent as usize;
        self.sent += 1;
        let line = match self.config.lines.get(index) {
            Some(line) => line.clone(),
            None => self.reading(),
        };
        match self.config.gap.filter(|g| g.line == self.sent && g.pos < line.len()) {
            Some(gap) => {
                port.send(&line[..gap.pos]);
                self.tail = Some((line[gap.pos..].to_vec(), gap.offers));
            }
            None => port.send(&line),
        }
    }
}

impl Serial for Load {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            match byte {
                b'\r' | b'\n' => {
                    if !self.pending.is_empty() {
                        let line = std::mem::take(&mut self.pending);
                        self.command(&line, port);
                    }
                }
                0 => {}
                _ => {
                    // Bounded: this is where the firmware overflows.
                    if self.pending.len() >= MAX_LINE {
                        port.violation(format!(
                            "'{}': longer than the firmware's buffer",
                            self.pending.escape_ascii()
                        ));
                        self.pending.clear();
                    }
                    self.pending.push(byte);
                }
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("'{}': a line left unfinished", self.pending.escape_ascii()));
            self.pending.clear();
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if let Some((tail, offers)) = &mut self.tail {
            if *offers == 0 {
                port.send(tail);
                self.tail = None;
            } else {
                *offers -= 1;
            }
            return;
        }
        let Some(period) = self.period else { return };
        if self.config.count.is_some_and(|n| self.sent >= n) {
            port.mark();
            return;
        }
        self.offers += 1;
        if self.offers >= period {
            self.offers = 0;
            self.monitor_line(port);
        }
    }

    fn reset(&mut self) {
        *self = Load::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    fn load(options: &[(&str, &str)]) -> Host<Load> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("115200/8n1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Load>) -> usize {
        h.v.drain().len()
    }

    /// What `n` offers bring.
    fn offers(h: &mut Host<Load>, n: u32) -> Vec<u8> {
        let mut out = Vec::new();
        for _ in 0..n {
            if let Reply::Data(d) = h.offer() {
                out.extend(d);
            }
        }
        out
    }

    #[test]
    fn the_scan_gets_its_version() {
        let mut h = load(&[]);
        h.write(b"monitor 0\r\n");
        assert_eq!(h.read(), b"");
        h.write(b"version\r\n");
        assert_eq!(h.read(), b"version 1.11\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn version_none_and_banner_replace_the_answer() {
        let mut h = load(&[("version", "none")]);
        h.write(b"version\r\n");
        assert_eq!(h.read(), b"");
        let mut h = load(&[("banner", "6f6b0d0a")]);
        h.write(b"version\r\n");
        assert_eq!(h.read(), b"ok\r\n");
    }

    #[test]
    fn a_line_may_come_in_pieces_or_together() {
        let mut h = load(&[]);
        h.write(b"se");
        h.write(b"t 1");
        assert_eq!(h.read(), b"");
        h.write(b"000\n");
        assert_eq!(h.read(), b"set 1000\r\n");
        h.write(b"\r\n\n\0se\0t\nuvlo\r");
        assert_eq!(h.read(), b"set 1000\r\nuvlo 0\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn settings_answer_and_take_effect() {
        let mut h = load(&[("volts", "5000")]);
        h.write(b"read\n");
        assert_eq!(h.read(), b"read 0 5000 0 0\r\n");
        h.write(b"set 1500\n");
        assert_eq!(h.read(), b"set 1500\r\n");
        h.write(b"set\n");
        assert_eq!(h.read(), b"set 1500\r\n");
        h.write(b"read\n");
        assert_eq!(h.read(), b"read 1500 5000 0 0\r\n");
        h.write(b"off\n");
        assert_eq!(h.read(), b"ok\r\n");
        h.write(b"read\n");
        assert_eq!(h.read(), b"read 0 5000 0 0\r\n");
        h.write(b"on\n");
        assert_eq!(h.read(), b"ok\r\n");
        h.write(b"uvlo 3300\n");
        assert_eq!(h.read(), b"uvlo 3300\r\n");
        h.write(b"uvlo\n");
        assert_eq!(h.read(), b"uvlo 3300\r\n");
        h.write(b"uvlo 0\n");
        assert_eq!(h.read(), b"uvlo 0\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_limits_are_the_firmwares() {
        let mut h = load(&[]);
        h.write(b"set 6000\n");
        assert_eq!(h.read(), b"set 6000\r\n");
        h.write(b"uvlo 60000\n");
        assert_eq!(h.read(), b"uvlo 60000\r\n");
        assert_eq!(violations(&mut h), 0);
        for bad in [&b"set 6001\n"[..], b"uvlo 60001\n", b"set x\n", b"uvlo -1\n", b"set 1 2\n"] {
            h.write(bad);
            assert_eq!(h.read(), b"", "{}", bad.escape_ascii());
            assert_eq!(violations(&mut h), 1, "{}", bad.escape_ascii());
        }
    }

    #[test]
    fn the_c_habit_of_a_nan_gets_the_firmwares_refusal() {
        let mut h = load(&[("set", "100"), ("uvlo", "200")]);
        h.write(b"set -2147483648\n");
        assert_eq!(h.read(), b"err set current must be between 0 and 6000\r\nset 100\r\n");
        h.write(b"uvlo -2147483648\n");
        assert_eq!(h.read(), b"err uvlo must be between 0 and 60000\r\nuvlo 200\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn uvlo_came_with_v1_9() {
        let mut h = load(&[("version", "1.8")]);
        h.write(b"uvlo\n");
        assert_eq!(h.read(), b"err Unknown command 'uvlo'\r\n");
        h.write(b"uvlo 1000\n");
        assert_eq!(h.read(), b"err Unknown command 'uvlo'\r\n");
        for known in ["1.9", "1.10", "2.0", "x"] {
            let mut h = load(&[("version", known)]);
            h.write(b"uvlo\n");
            assert_eq!(h.read(), b"uvlo 0\r\n", "{known}");
        }
    }

    #[test]
    fn what_no_c_path_sends_is_flagged() {
        for bad in [
            &b"reset\n"[..],
            b"clear\n",
            b"mode\n",
            b"debug\n",
            b"dump\n",
            b"cal x\n",
            b"bl\n",
            b"READ\n",
            b"read 1\n",
            b"version 1\n",
            b"on 1\n",
            b"off 1\n",
            b"monitor\n",
            b"monitor 100\n",
            b"nonsense\n",
        ] {
            let mut h = load(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{}", bad.escape_ascii());
        }
    }

    #[test]
    fn a_line_too_long_is_flagged_and_the_buffer_stays_bounded() {
        let mut h = load(&[]);
        h.write(&[b'x'; MAX_LINE]);
        assert_eq!(violations(&mut h), 0);
        h.write(&[b'x'; 4096]);
        assert!(h.cdc.model.pending.len() <= MAX_LINE);
        assert!(violations(&mut h) > 0);
    }

    #[test]
    fn a_line_left_unfinished_shows_at_the_next_open() {
        let mut h = load(&[]);
        h.write(b"rea");
        h.set_lines(false, false);
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn setters_0_flags_each_setting_and_no_question() {
        for setting in [&b"set 1\n"[..], b"uvlo 1\n", b"on\n", b"off\n"] {
            let mut h = load(&[("setters", "0")]);
            h.write(setting);
            assert_eq!(violations(&mut h), 1, "{}", setting.escape_ascii());
        }
        let mut h = load(&[("setters", "0")]);
        h.write(b"set\nuvlo\nread\nversion\r\nmonitor 0\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn monitoring_paces_by_offers_until_stopped() {
        let mut h = load(&[("set", "1000")]);
        assert_eq!(offers(&mut h, 100), b"", "off until asked");
        h.write(b"monitor 200\r\n");
        assert_eq!(offers(&mut h, 9), b"", "no answer, and an interval to wait");
        assert_eq!(offers(&mut h, 1), b"read 1000 12000 0 0\r\n");
        assert_eq!(offers(&mut h, 10), b"read 1000 12000 0 0\r\n");
        h.write(b"monitor 0\r\n");
        assert_eq!(offers(&mut h, 100), b"");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn lines_count_period_and_gap_shape_the_stream() {
        let mut h = load(&[("period", "2"), ("lines", "6f7665727465"), ("count", "2")]);
        h.write(b"monitor 200\r\n");
        assert_eq!(offers(&mut h, 2), b"overte");
        assert!(!h.cdc.mark());
        assert_eq!(offers(&mut h, 2), b"read 0 12000 0 0\r\n");
        assert_eq!(offers(&mut h, 20), b"", "count= holds it");
        assert!(h.cdc.mark());

        let mut h = load(&[("period", "1"), ("gap", "2:5:3")]);
        h.write(b"monitor 200\r\n");
        assert_eq!(offers(&mut h, 1), b"read 0 12000 0 0\r\n");
        assert_eq!(offers(&mut h, 1), b"read ");
        assert_eq!(offers(&mut h, 3), b"");
        assert_eq!(offers(&mut h, 1), b"0 12000 0 0\r\n");
        assert_eq!(offers(&mut h, 1), b"read 0 12000 0 0\r\n");
    }

    #[test]
    fn stale_goes_once_at_the_first_stop() {
        let mut h = load(&[("stale", "61")]);
        h.write(b"monitor 0\r\n");
        assert_eq!(h.read(), b"a");
        h.write(b"monitor 0\r\n");
        assert_eq!(h.read(), b"");
    }

    #[test]
    fn replies_play_in_turn_and_answers_run_out() {
        let mut h = load(&[("replies", "61,,62")]);
        h.write(b"version\r\n");
        assert_eq!(h.read(), b"version 1.11\r\n", "not one of the replies");
        h.write(b"read\n");
        assert_eq!(h.read(), b"a");
        h.write(b"set\n");
        assert_eq!(h.read(), b"");
        h.write(b"on\n");
        assert_eq!(h.read(), b"b");
        h.write(b"on\n");
        assert_eq!(h.read(), b"ok\r\n");

        let mut h = load(&[("answers", "1")]);
        h.write(b"version\r\n");
        assert_eq!(h.read(), b"version 1.11\r\n", "not counted");
        h.write(b"read\n");
        assert_eq!(h.read(), b"read 0 12000 0 0\r\n");
        assert!(!h.cdc.mark());
        h.write(b"read\n");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn it_sits_behind_a_uart() {
        let mut h = load(&[]);
        h.set_line(Line::parse("9600/8n1").unwrap());
        h.write(b"version\r\n");
        assert_eq!(h.read(), b"");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = load(&[("replies", "61"), ("count", "1"), ("period", "1")]);
        h.write(b"set 100\nmonitor 200\r\n");
        assert_eq!(h.read(), b"aread 100 12000 0 0\r\n");
        h.cdc.model.reset();
        h.write(b"set\n");
        assert_eq!(h.read(), b"a");
        h.write(b"set\n");
        assert_eq!(h.read(), b"set 0\r\n");
        assert_eq!(offers(&mut h, 3), b"", "not monitoring");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("banner", "4"),
            ("set", "6001"),
            ("uvlo", "60001"),
            ("on", "2"),
            ("period", "0"),
            ("gap", "1:2"),
            ("gap", "0:2:3"),
            ("gap", "1:0:3"),
            ("gap", "1:2:3:4"),
            ("gap", "a:b:c"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("version", "none"), ("version", "1.0"), ("replies", "6161,"), ("chunk", "1")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
