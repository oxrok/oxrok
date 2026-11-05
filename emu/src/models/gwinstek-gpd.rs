// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A GW Instek GPD-2303S or GPD-3303S bench supply on the CDC-ACM shell: two
//! channels, ASCII lines ended by LF one way and CR or CR LF the other.
//!
//! The device, and how sure this is:
//!
//! - The commands: `api.c` and `protocol.c`, and `PyGPD3303S` (`gpd3303s.py`,
//!   written against real units): `*IDN?`, `STATUS?`, `ISET<n>?`, `VSET<n>?`,
//!   `IOUT<n>?`, `VOUT<n>?`, `ISET<n>:<v>`, `VSET<n>:<v>` and `OUT<b>`, each
//!   ended by LF. A setting has no answer.
//! - The answers' shape: `PyGPD3303S` drops one unit letter from a number,
//!   `A` or `V`. The widths here (`2.000A`, `12.000V`) are a guess from that:
//!   the vendor's manual could not be read (its copies refused the download).
//! - The line ends: `PyGPD3303S` looks for CR first and takes CR LF on newer
//!   units, so `eol=crlf` by default and `eol=cr` for the old firmware. `eol=lf` is what `protocol.c`'s reader takes too;
//!   nothing shows a unit does it.
//! - The ID: `api.c`'s comment, `GW INSTEK,GPD-2303S,SN:ER915277,V2.10`.
//! - The status: `api.c`'s `sscanf`, eight digits in the order CH1 mode, CH2
//!   mode, tracking twice, beep, output, baud twice. The meaning of a mode's
//!   digit (1 is CV), the tracking pair (01) and the baud pair are an
//!   assumption, unchecked, and the driver reads only the output's digit. The
//!   old firmware's form is `api.c`'s second `sscanf`, on the first of three
//!   lines, which `PyGPD3303S` also reads; the other two lines are stand-ins.
//! - The UART: `PyGPD3303S` says the supply has an FTDI converter, so the
//!   line must fit: `uart=` sets the rate the panel chose, 9600/8n1 by
//!   default, and `api.c` tries 9600, 57600 and 115200.
//! - The junk in the receive buffer: `api.c`'s comment, with its "Invalid
//!   Character." reply. `junk=TEXT` plays it. How the real one answers other
//!   bad input was not read.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id, where the real one is
//!   an FTDI chip behind `ftdi_sio`: only the byte stream and the line
//!   settings are the device's.
//! - It answers at once.
//! - `vout1=`, `iout1=`, `iset1=`, `vset1=` and the same for channel 2 and
//!   `status=` are stand-ins for a load that changes and for a device gone
//!   wrong. In them `~` as the last character sends what is before it with no
//!   line end, so `~` alone is no answer. `chatter=` adds a line after each
//!   `VOUT2?`, which nothing shows a real one does; it reaches what the
//!   driver does with data nobody asked for.
//! - `load=` is one resistor on each channel, so that CC and CV have a cause.
//!
//! Not modelled: `TRACK`, `BEEP`, `RCL`, `SAV`, `BAUD`, `HELP?`, `ERR?`,
//! `REMOTE`, `LOCAL` (no path of the C driver sends them, so they are flagged);
//! the channels of the GPD-4303S, which the driver refuses by its ID, played
//! with `idn=`.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `idn=HEX`, the answer to `*IDN?`, or `idn=none`; `idn-eol=0` leaves its
//!   line end out;
//! - `uart=LINE`, `eol=lf|cr|crlf`, `junk=TEXT` in the buffer at power on;
//! - `vmax=VOLTS`, `imax=AMPS`, what the panel takes (30, 3);
//! - `v1=`, `i1=`, `v2=`, `i2=`, the set points at power on, as `5.000`;
//! - `output=0|1`, `beep=0|1`, `load=OHMS`;
//! - `vout1=TEXT[,TEXT...]` and so on, `status=TEXT[,TEXT...]`, answers played
//!   in turn;
//! - `status-old=1`, the old firmware's three lines;
//! - `chatter=TEXT`;
//! - `answers=N`, queries answered per power on, then silence and the event
//!   point;
//! - `setters=0`, which flags any setting.
//!
//! The pedantic checks: only the commands above, each ended by LF, in the
//! format C's `%05.3lf` gives, within the limits; nothing left of a command
//! when a tool opens. Two C habits are taken, as `api.c`'s range check is two
//! comparisons and no answer path shows a refusal: `-0.000` as 0, and `  nan`
//! as nothing.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "gwinstek-gpd";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A GW Instek GPD-2303S or GPD-3303S supply on CDC-ACM; idn=HEX|none, \
                         idn-eol=, uart=, eol=, junk=, vmax=, imax=, v1=, i1=, v2=, i2=, \
                         output=, beep=, load=, vout1=, iout1=, vset1=, iset1= (and 2), \
                         status=, status-old=, chatter=, answers=, setters=, and the shell's \
                         stall=, chunk=, in-limit=, in-stall=, out-hold=";

const DEFAULT_IDN: &[u8] = b"GW INSTEK,GPD-2303S,SN:ER915277,V2.10";
/// More than the longest command, `VSET1:-0.000`; what is longer is flagged.
const LONGEST: usize = 32;

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

/// A set point with exactly three decimals, in thousandths.
fn parse_milli(text: &str) -> Option<u32> {
    let (whole, part) = text.split_once('.')?;
    let digits = |t: &str| !t.is_empty() && t.bytes().all(|c| c.is_ascii_digit());
    if !digits(whole) || !digits(part) || part.len() != 3 || whole.len() > 2 {
        return None;
    }
    Some(whole.parse::<u32>().ok()? * 1000 + part.parse::<u32>().ok()?)
}

fn parse_limit(key: &str, value: &str) -> Result<u32> {
    let invalid = || Error::Invalid(format!("{key}={value}: a number as in 30 or 3.2"));
    let number: f64 = value.parse().map_err(|_| invalid())?;
    if !(0.0..=1000.0).contains(&number) {
        return Err(invalid());
    }
    Ok((number * 1000.0).round() as u32)
}

fn set_point(key: &str, value: &str) -> Result<u32> {
    parse_milli(value).ok_or_else(|| Error::Invalid(format!("{key}={value}: as in 5.000")))
}

fn answers_list(value: &str) -> Packets {
    Packets::new(value.split(',').map(|t| t.as_bytes().to_vec()).collect())
}

fn function(options: &[(String, String)]) -> Result<Cdc<Supply>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("9600/8n1")?;
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "idn" if v == "none" => config.idn = None,
            "idn" => config.idn = Some(parse_hex_list(k, v, 1, 1024)?.remove(0)),
            "idn-eol" => config.idn_eol = flag(k, v)?,
            "uart" => uart = Line::parse(v)?,
            "eol" => {
                config.eol = match v.as_str() {
                    "lf" => b"\n",
                    "cr" => b"\r",
                    "crlf" => b"\r\n",
                    _ => return Err(Error::Invalid(format!("eol={v}: lf, cr or crlf"))),
                };
            }
            "junk" => config.junk = v.as_bytes().to_vec(),
            "vmax" => config.vmax = parse_limit(k, v)?,
            "imax" => config.imax = parse_limit(k, v)?,
            "v1" => config.power_on.volts[0] = set_point(k, v)?,
            "v2" => config.power_on.volts[1] = set_point(k, v)?,
            "i1" => config.power_on.amps[0] = set_point(k, v)?,
            "i2" => config.power_on.amps[1] = set_point(k, v)?,
            "output" => config.power_on.output = flag(k, v)?,
            "beep" => config.power_on.beep = flag(k, v)?,
            "load" => config.load = Some(parse_number(k, v, 1..=1_000_000)?),
            "vout1" => config.lists.vout[0] = Some(answers_list(v)),
            "vout2" => config.lists.vout[1] = Some(answers_list(v)),
            "iout1" => config.lists.iout[0] = Some(answers_list(v)),
            "iout2" => config.lists.iout[1] = Some(answers_list(v)),
            "vset1" => config.lists.vset[0] = Some(answers_list(v)),
            "vset2" => config.lists.vset[1] = Some(answers_list(v)),
            "iset1" => config.lists.iset[0] = Some(answers_list(v)),
            "iset2" => config.lists.iset[1] = Some(answers_list(v)),
            "status" => config.lists.status = Some(answers_list(v)),
            "status-old" => config.status_old = flag(k, v)?,
            "chatter" => config.chatter = Some(v.as_bytes().to_vec()),
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "setters" => config.setters = flag(k, v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    config.baud = uart.baud;
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "GPD power supply");
    Ok(Cdc::new(NAME, &id, faults, Supply::new(config)).behind_uart(uart))
}

#[derive(Clone, Copy, Debug)]
struct Panel {
    /// Thousandths of a volt, per channel.
    volts: [u32; 2],
    /// Milliamperes, per channel.
    amps: [u32; 2],
    output: bool,
    beep: bool,
}

/// Answers played in turn instead of the device's own.
#[derive(Clone, Debug, Default)]
struct Lists {
    vout: [Option<Packets>; 2],
    iout: [Option<Packets>; 2],
    vset: [Option<Packets>; 2],
    iset: [Option<Packets>; 2],
    status: Option<Packets>,
}

#[derive(Clone, Debug)]
struct Config {
    idn: Option<Vec<u8>>,
    idn_eol: bool,
    baud: u32,
    eol: &'static [u8],
    junk: Vec<u8>,
    vmax: u32,
    imax: u32,
    power_on: Panel,
    load: Option<u32>,
    lists: Lists,
    status_old: bool,
    chatter: Option<Vec<u8>>,
    answers: Option<u32>,
    setters: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            idn: Some(DEFAULT_IDN.to_vec()),
            idn_eol: true,
            baud: 9600,
            eol: b"\r\n",
            junk: Vec::new(),
            vmax: 30_000,
            imax: 3_000,
            power_on: Panel {
                volts: [12_000, 5_000],
                amps: [1_000, 2_500],
                output: false,
                beep: true,
            },
            load: None,
            lists: Lists::default(),
            status_old: false,
            chatter: None,
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
    /// The first line starts with what lay in the buffer at power on.
    junk_left: bool,
    answered: u32,
}

impl Supply {
    fn new(config: Config) -> Supply {
        Supply {
            now: config.power_on,
            lists: config.lists.clone(),
            pending: config.junk.clone(),
            junk_left: !config.junk.is_empty(),
            config,
            answered: 0,
        }
    }

    /// A channel's millivolts and milliamperes and whether it regulates the
    /// voltage, from the set points and the load.
    fn operating_point(&self, ch: usize) -> (u32, u32, bool) {
        if !self.now.output {
            return (0, 0, true);
        }
        let volts = self.now.volts[ch];
        let Some(load) = self.config.load else { return (volts, 0, true) };
        let limit = self.now.amps[ch];
        let wanted = volts / load;
        if wanted <= limit {
            (volts, wanted, true)
        } else {
            let at_limit = u64::from(limit) * u64::from(load);
            (u32::try_from(at_limit).unwrap_or(volts).min(volts), limit, false)
        }
    }

    fn status_line(&self) -> Vec<u8> {
        let bit = u8::from;
        let (cv1, cv2) = (self.operating_point(0).2, self.operating_point(1).2);
        let (b, o) = (bit(self.now.beep), bit(self.now.output));
        if self.config.status_old {
            return format!("{} {} 0 1 {b} X {o} X", bit(cv1), bit(cv2)).into_bytes();
        }
        let baud = match self.config.baud {
            115_200 => "00",
            57_600 => "01",
            _ => "10",
        };
        format!("{}{}01{b}{o}{baud}", bit(cv1), bit(cv2)).into_bytes()
    }

    /// A line of an answer, with the line end unless it says otherwise.
    fn say(&self, text: &[u8], port: &mut Port<'_>) {
        if let Some(raw) = text.strip_suffix(b"~") {
            port.send(raw);
            return;
        }
        let mut line = text.to_vec();
        line.extend_from_slice(self.config.eol);
        port.send(&line);
    }

    fn query(&mut self, command: Query, port: &mut Port<'_>) {
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return;
        }
        self.answered += 1;
        match command {
            Query::Id => match &self.config.idn {
                None => {}
                Some(idn) if self.config.idn_eol => self.say(idn, port),
                Some(idn) => port.send(idn),
            },
            Query::Status => {
                if let Some(list) = &mut self.lists.status {
                    let text = list.take();
                    self.say(&text, port);
                    return;
                }
                let line = self.status_line();
                self.say(&line, port);
                if self.config.status_old {
                    self.say(b"Beep: stand-in", port);
                    self.say(b"Baud: stand-in", port);
                }
            }
            Query::Reading(kind, ch) => {
                let (vout, iout, _) = self.operating_point(ch);
                let (list, own) = match kind {
                    Reading::VSet => (&mut self.lists.vset[ch], volts(self.now.volts[ch])),
                    Reading::ISet => (&mut self.lists.iset[ch], amps(self.now.amps[ch])),
                    Reading::VOut => (&mut self.lists.vout[ch], volts(vout)),
                    Reading::IOut => (&mut self.lists.iout[ch], amps(iout)),
                };
                let text = list.as_mut().map_or(own, Packets::take);
                self.say(&text, port);
                if (kind, ch) == (Reading::VOut, 1)
                    && let Some(chatter) = &self.config.chatter
                {
                    self.say(chatter, port);
                }
            }
        }
    }

    fn setting(&mut self, text: &str, port: &mut Port<'_>) {
        if !self.config.setters {
            port.violation(format!("'{text}': a setting no one asked for"));
        }
        match parse_setting(text) {
            Some(Setting::Output(on)) => self.now.output = on,
            Some(Setting::Volts(_, None) | Setting::Amps(_, None)) => {}
            Some(Setting::Volts(ch, Some(v))) if v <= self.config.vmax => self.now.volts[ch] = v,
            Some(Setting::Amps(ch, Some(a))) if a <= self.config.imax => self.now.amps[ch] = a,
            Some(_) => port.violation(format!("'{text}': above the limit")),
            None => port.violation(format!("'{text}': not as %05.3lf has it")),
        }
    }

    fn line(&mut self, line: &[u8], port: &mut Port<'_>) {
        if std::mem::take(&mut self.junk_left) {
            self.say(b"Invalid Character.", port);
            return;
        }
        if let Some(query) = parse_query(line) {
            self.query(query, port);
        } else if [&b"OUT"[..], b"VSET", b"ISET"].iter().any(|head| line.starts_with(head)) {
            self.setting(&String::from_utf8_lossy(line), port);
        } else {
            port.violation(format!("'{}': no such command", show(line)));
            self.say(b"Invalid Character.", port);
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Reading {
    VSet,
    ISet,
    VOut,
    IOut,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Query {
    Id,
    Status,
    Reading(Reading, usize),
}

fn parse_query(line: &[u8]) -> Option<Query> {
    Some(match line {
        b"*IDN?" => Query::Id,
        b"STATUS?" => Query::Status,
        b"VSET1?" => Query::Reading(Reading::VSet, 0),
        b"VSET2?" => Query::Reading(Reading::VSet, 1),
        b"ISET1?" => Query::Reading(Reading::ISet, 0),
        b"ISET2?" => Query::Reading(Reading::ISet, 1),
        b"VOUT1?" => Query::Reading(Reading::VOut, 0),
        b"VOUT2?" => Query::Reading(Reading::VOut, 1),
        b"IOUT1?" => Query::Reading(Reading::IOut, 0),
        b"IOUT2?" => Query::Reading(Reading::IOut, 1),
        _ => return None,
    })
}

/// A setting; a value of `None` is the C habit of a NaN, which changes nothing.
enum Setting {
    Output(bool),
    Volts(usize, Option<u32>),
    Amps(usize, Option<u32>),
}

fn parse_setting(text: &str) -> Option<Setting> {
    match text {
        "OUT0" => return Some(Setting::Output(false)),
        "OUT1" => return Some(Setting::Output(true)),
        _ => {}
    }
    let (head, value) = text.split_once(':')?;
    let ch = match head.get(4..)? {
        "1" => 0,
        "2" => 1,
        _ => return None,
    };
    let value = match value {
        "  nan" => None,
        "-0.000" => Some(0),
        // %05.3lf's width: nothing shorter than five.
        v if v.len() >= 5 => Some(parse_milli(v)?),
        _ => return None,
    };
    match head.get(..4)? {
        "VSET" => Some(Setting::Volts(ch, value)),
        "ISET" => Some(Setting::Amps(ch, value)),
        _ => None,
    }
}

fn volts(v: u32) -> Vec<u8> {
    format!("{:02}.{:03}V", v / 1000, v % 1000).into_bytes()
}

fn amps(v: u32) -> Vec<u8> {
    format!("{}.{:03}A", v / 1000, v % 1000).into_bytes()
}

fn show(bytes: &[u8]) -> String {
    bytes.escape_ascii().to_string()
}

impl Serial for Supply {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte == b'\n' {
                let line = std::mem::take(&mut self.pending);
                self.line(&line, port);
            } else if self.pending.len() >= LONGEST + self.config.junk.len() {
                port.violation(format!("'{}': a line too long", show(&self.pending)));
                self.pending.clear();
            } else {
                self.pending.push(byte);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() && !self.junk_left {
            port.violation(format!("'{}': a command left unfinished", show(&self.pending)));
            self.pending.clear();
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
        Host::open(function(&o).unwrap(), Line::default(), true, true)
    }

    fn violations(h: &mut Host<Supply>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn the_queries_have_their_answers() {
        let mut h = supply(&[]);
        for (ask, want) in [
            (&b"*IDN?\n"[..], &b"GW INSTEK,GPD-2303S,SN:ER915277,V2.10\r\n"[..]),
            (b"STATUS?\n", b"11011010\r\n"),
            (b"VSET1?\n", b"12.000V\r\n"),
            (b"VSET2?\n", b"05.000V\r\n"),
            (b"ISET1?\n", b"1.000A\r\n"),
            (b"ISET2?\n", b"2.500A\r\n"),
            (b"VOUT1?\n", b"00.000V\r\n"),
            (b"IOUT2?\n", b"0.000A\r\n"),
        ] {
            h.write(ask);
            assert_eq!(h.read(), want, "{}", show(ask));
        }
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn settings_take_effect_and_answer_nothing() {
        let mut h = supply(&[("load", "10")]);
        h.write(b"VSET1:5.000\nISET1:2.225\nVSET2:30.000\nISET2:0.100\nOUT1\n");
        assert_eq!(h.read(), b"");
        h.write(b"VOUT1?\nIOUT1?\nVOUT2?\nIOUT2?\nSTATUS?\n");
        assert_eq!(h.read(), b"05.000V\r\n0.500A\r\n01.000V\r\n0.100A\r\n10011110\r\n");
        h.write(b"OUT0\nVOUT1?\n");
        assert_eq!(h.read(), b"00.000V\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_status_shows_the_rate_and_the_old_form_has_three_lines() {
        let mut h = supply(&[("uart", "9600/8n1"), ("status-old", "1"), ("eol", "cr")]);
        h.write(b"STATUS?\n");
        assert_eq!(h.read(), b"1 1 0 1 1 X 0 X\rBeep: stand-in\rBaud: stand-in\r");
        for (uart, want) in [("57600/8n1", b"11011001\r\n"), ("115200/8n1", b"11011000\r\n")] {
            let mut h = Host::open(
                function(&[("uart".to_owned(), uart.to_owned())]).unwrap(),
                Line::parse(uart).unwrap(),
                true,
                true,
            );
            h.write(b"STATUS?\n");
            assert_eq!(h.read(), want);
        }
    }

    #[test]
    fn what_no_c_path_sends_is_flagged() {
        for bad in [
            &b"BEEP1\n"[..],
            b"TRACK0\n",
            b"RCL1\n",
            b"SAV1\n",
            b"HELP?\n",
            b"ERR?\n",
            b"*idn?\n",
            b"VOUT3?\n",
            b"\n",
            b"X\n",
        ] {
            let mut h = supply(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            assert_eq!(h.read(), b"Invalid Character.\r\n");
        }
    }

    #[test]
    fn a_setting_in_another_format_is_flagged_and_changes_nothing() {
        for bad in [
            &b"VSET1:5.0\n"[..],
            b"VSET1:5.0000\n",
            b"VSET1:30.001\n",
            b"VSET3:1.000\n",
            b"VSET3?\n",
            b"VSETX:1.000\n",
            b"VSET1:abcde\n",
            b"VSET1\n",
            b"ISET1:3.001\n",
            b"ISET1:005.000\n",
            b"OUT2\n",
            b"OUT:1.000\n",
            b"OUT\xc3\xa9:1.000\n",
        ] {
            let mut h = supply(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            h.write(b"VSET1?\nISET1?\n");
            assert_eq!(h.read(), b"12.000V\r\n1.000A\r\n");
        }
    }

    #[test]
    fn the_limits_are_options() {
        let mut h = supply(&[("vmax", "32"), ("imax", "3.2")]);
        h.write(b"VSET1:32.000\nISET2:3.200\n");
        assert_eq!(violations(&mut h), 0);
        h.write(b"VSET1:32.001\nISET2:3.201\n");
        assert_eq!(violations(&mut h), 2);
    }

    #[test]
    fn the_c_habits_are_taken() {
        let mut h = supply(&[]);
        h.write(b"VSET1:  nan\nISET1:  nan\n");
        assert_eq!(violations(&mut h), 0);
        h.write(b"VSET1?\nISET1?\n");
        assert_eq!(h.read(), b"12.000V\r\n1.000A\r\n");
        h.write(b"VSET1:-0.000\nISET1:-0.000\nVSET1?\nISET1?\n");
        assert_eq!(h.read(), b"00.000V\r\n0.000A\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn setters_0_flags_each_setting() {
        for setting in [&b"OUT0\n"[..], b"OUT1\n", b"VSET1:1.000\n", b"ISET2:1.000\n"] {
            let mut h = supply(&[("setters", "0")]);
            h.write(setting);
            assert_eq!(violations(&mut h), 1, "{}", show(setting));
        }
        let mut h = supply(&[("setters", "0")]);
        h.write(b"STATUS?\n*IDN?\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn junk_in_the_buffer_spoils_the_first_line_only() {
        let mut h = supply(&[("junk", "VSE")]);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 0, "it lay there before the open");
        h.write(b"*IDN?\n");
        assert_eq!(h.read(), b"Invalid Character.\r\n");
        h.write(b"*IDN?\n");
        assert_eq!(h.read(), b"GW INSTEK,GPD-2303S,SN:ER915277,V2.10\r\n");
        assert_eq!(violations(&mut h), 0);
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
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = supply(&[("answers", "1")]);
        h.write(b"*IDN?\n");
        assert_ne!(h.read(), b"");
        assert!(!h.cdc.mark());
        h.write(b"IOUT1?\n");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn answers_played_in_turn_and_their_line_ends() {
        let mut h = supply(&[("vout1", "1.5V,x,~,2~"), ("status", "abc"), ("iset2", "9")]);
        for want in [&b"1.5V\r\n"[..], b"x\r\n", b"", b"2", b"1.5V\r\n"] {
            h.write(b"VOUT1?\n");
            assert_eq!(h.read(), want);
        }
        h.write(b"STATUS?\nISET2?\n");
        assert_eq!(h.read(), b"abc\r\n9\r\n");
    }

    #[test]
    fn eol_and_chatter_and_an_id_with_no_end() {
        let mut h =
            supply(&[("eol", "lf"), ("chatter", "hello"), ("idn-eol", "0"), ("idn", "4142")]);
        h.write(b"*IDN?\n");
        assert_eq!(h.read(), b"AB");
        h.write(b"VOUT1?\n");
        assert_eq!(h.read(), b"00.000V\n");
        h.write(b"VOUT2?\n");
        assert_eq!(h.read(), b"00.000V\nhello\n");
        let mut h = supply(&[("idn", "none")]);
        h.write(b"*IDN?\n");
        assert_eq!(h.read(), b"");
    }

    #[test]
    fn behind_its_uart_a_wrong_rate_hears_nothing() {
        let mut h = supply(&[("uart", "57600/8n1")]);
        h.write(b"*IDN?\n");
        assert_eq!(h.read(), b"");
        h.set_line(Line::parse("57600/8n1").unwrap());
        h.write(b"*IDN?\n");
        assert_ne!(h.read(), b"");
    }

    #[test]
    fn a_load_too_heavy_is_constant_current() {
        let mut h = supply(&[("load", "1"), ("output", "1"), ("v1", "12.000"), ("i1", "2.000")]);
        h.write(b"VOUT1?\nIOUT1?\nSTATUS?\n");
        assert_eq!(h.read(), b"02.000V\r\n2.000A\r\n00011110\r\n");
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = supply(&[("vout1", "1,2"), ("answers", "3"), ("junk", "J")]);
        h.write(b"\nVSET1:1.000\nVOUT1?\nVOUT1?\n");
        h.read();
        h.cdc.model.reset();
        h.write(b"\nVSET1?\nVOUT1?\n");
        assert_eq!(h.read(), b"Invalid Character.\r\n12.000V\r\n1\r\n");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("idn", "4"),
            ("v1", "1.5"),
            ("i2", "100.000"),
            ("output", "2"),
            ("eol", "x"),
            ("vmax", "x"),
            ("load", "0"),
            ("uart", "9600"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("idn", "none"), ("chunk", "1"), ("vout2", "x"), ("uart", "115200/8n1")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
