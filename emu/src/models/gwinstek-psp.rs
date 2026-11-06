// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A GW Instek PSP-603, PSP-405 or PSP-2010 bench supply on the CDC-ACM shell:
//! one output, a fixed 2400/8n1 line, and ASCII commands.
//!
//! The device, and how sure this is:
//!
//! - The commands and the status: the PSP programming manual as Texio publishes
//!   it for its rebadged units (`PSP_pm1.pdf`, in Japanese), and `pypsp`, written
//!   against real units. A command ends in CR, an answer in CR LF, and `L` gives
//!   37 characters, `Vvv.vvAa.aaaWwww.wUuuIi.iiPpppFffffff`. The flags are, in
//!   turn: relay, overheated, fine knob, knob unlocked, remote, locked. A
//!   lower-case `u`, `i` or `p` means the panel is setting that limit.
//! - `SV xx.xx`, `SU xx`, `SI x.xx`, `KOE`, `KOD`: the same manual. They have
//!   no answer.
//! - The ranges: `api.c`'s table. 200 W for the power limit is the manual's
//!   example.
//! - `urpsp=1`, an answer ending in CR CR LF: only `protocol.c`'s comment, which
//!   names URPSP1 and URPSP2. The English manual's "Setting for different command
//!   format" should say more, but its copies refused the download.
//! - That the voltage reads as the set point with the output off: `protocol.h`'s
//!   comment, nothing else.
//! - That a value above what the panel takes is held at it: `api.c`'s comment
//!   ("device handles overflow correctly") says only that it does no harm.
//!   That a set point above the voltage limit reads as the limit is a guess.
//! - At 10 A, where `a.aaa` and `i.ii` have no room, the fields here stay as
//!   wide, `10.00` and `10.0`, because the manual calls the frame fixed. That
//!   is a guess: nothing shows a PSP-2010's frame at 10 A.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id, where the real one is
//!   an RS-232 port behind whatever cable, optoisolated and powered from DTR
//!   according to `pypsp`. Only the byte stream and the line settings are the
//!   device's, and the lines' power is not modelled.
//! - It answers at once, where a real `L` takes about 245 ms (`protocol.h`).
//! - `load=` is one resistor, so that CC and CV have a cause.
//! - `replies=` stands in for a panel or a load that changes, and for a device
//!   gone wrong. `babble=1` is a device that never stops talking, which nothing
//!   shows a real one does; it reaches what a driver's wait for silence does.
//!
//! Not modelled: `V`, `A`, `W`, `U`, `I`, `P`, `F`, `SV+` and its like, `SUM`,
//! `SIM`, `SPM`, `KF`, `KN`, `KO`, `EEP`, `B`, `D`, `Q`, `SB`, `SD` and `SP`. No
//! path of the C driver sends them, so they are flagged and get no answer.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `model=603|405|2010`, the panel's ranges and limits at power on;
//! - `uart=LINE`, `urpsp=1|2`;
//! - `v=`, `i=`, the set points at power on, as `12.34` and `1.25`; `u=VOLTS`;
//! - `output=0|1`, `otp=0|1`, `load=OHMS`, `limit-mode=u|i|p`;
//! - `replies=HEX[,HEX...]`, the bytes of the next answers to `L`, whole, with
//!   their line end; then the device's own, so the first goes to a scan;
//! - `answers=N`, `L` answered per power on, then silence and the event point;
//! - `babble=1`, a byte at every offer of a read;
//! - `setters=0`, which flags any setting.
//!
//! The pedantic checks: only the commands above, each ended by CR LF as the C
//! driver sends them, in the formats of C's `%05.2f`, `%d` and `%04.2f`; nothing
//! left of a command when a tool opens. Three C habits are taken, as there is
//! no answer path to refuse them: `SU 5` with one digit, where the manual has
//! two; `-0.00` as 0; and `SU -2147483648`, `SV   nan`, `SI  nan` as nothing,
//! which is what a NaN comes to.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "gwinstek-psp";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A GW Instek PSP-603, PSP-405 or PSP-2010 supply on CDC-ACM; model=, uart=, \
                         urpsp=, v=, i=, u=, output=, otp=, load=, limit-mode=, replies=HEX,..., \
                         answers=, babble=, setters=, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

/// More than the longest command, `SU -2147483648`; what is longer is flagged.
const LONGEST: usize = 32;
/// The power limit at power on, as in the manual's example.
const POWER_LIMIT: u32 = 200;

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

/// `dd.dd` or `d.dd`, in hundredths.
fn parse_centi(text: &str) -> Option<u32> {
    let (whole, part) = text.split_once('.')?;
    let digits = |t: &str| !t.is_empty() && t.bytes().all(|c| c.is_ascii_digit());
    if !digits(whole) || !digits(part) || part.len() != 2 || whole.len() > 2 {
        return None;
    }
    Some(whole.parse::<u32>().ok()? * 100 + part.parse::<u32>().ok()?)
}

fn set_point(key: &str, value: &str) -> Result<u32> {
    parse_centi(value).filter(|v| *v <= 6000).ok_or_else(|| {
        Error::Invalid(format!("{key}={value}: as in 12.34, up to 60.00"))
    })
}

fn function(options: &[(String, String)]) -> Result<Cdc<Supply>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("2400/8n1")?;
    let mut config = Config::default();
    let (mut volts, mut amps, mut limit) = (None, None, None);
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => {
                (config.vmax, config.imax) = match v.as_str() {
                    "603" => (60, 350),
                    "405" => (40, 500),
                    "2010" => (20, 1000),
                    _ => return Err(Error::Invalid(format!("model={v}: 603, 405 or 2010"))),
                };
            }
            "uart" => uart = Line::parse(v)?,
            "urpsp" => {
                config.eol = match v.as_str() {
                    "1" => b"\r\r\n",
                    "2" => b"\r\n",
                    _ => return Err(Error::Invalid(format!("urpsp={v}: 1 or 2"))),
                };
            }
            "v" => volts = Some(set_point(k, v)?),
            "i" => amps = Some(set_point(k, v)?),
            "u" => limit = Some(parse_number(k, v, 0..=99)?),
            "output" => config.output = flag(k, v)?,
            "otp" => config.otp = flag(k, v)?,
            "load" => config.load = Some(parse_number(k, v, 1..=1_000_000)?),
            "limit-mode" => {
                config.limit_mode = match v.as_str() {
                    "u" | "i" | "p" => Some(v.as_bytes()[0]),
                    _ => return Err(Error::Invalid(format!("limit-mode={v}: u, i or p"))),
                };
            }
            "replies" => config.replies = parse_hex_list(k, v, 1, 1024)?,
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "babble" => config.babble = flag(k, v)?,
            "setters" => config.setters = flag(k, v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    config.volts = volts.unwrap_or(1200);
    config.amps = amps.unwrap_or(100);
    config.limit = limit.unwrap_or(config.vmax);
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "PSP power supply");
    Ok(Cdc::new(NAME, &id, faults, Supply::new(config)).behind_uart(uart))
}

#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)] // independent options
struct Config {
    /// What the panel takes: volts.
    vmax: u32,
    /// What the panel takes: hundredths of an ampere.
    imax: u32,
    eol: &'static [u8],
    /// The voltage set point at power on, in hundredths.
    volts: u32,
    /// The current limit at power on, in hundredths.
    amps: u32,
    /// The voltage limit at power on, in volts.
    limit: u32,
    output: bool,
    otp: bool,
    load: Option<u32>,
    limit_mode: Option<u8>,
    replies: Vec<Vec<u8>>,
    answers: Option<u32>,
    babble: bool,
    setters: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            vmax: 60,
            imax: 350,
            eol: b"\r\n",
            volts: 0,
            amps: 0,
            limit: 0,
            output: false,
            otp: false,
            load: None,
            limit_mode: None,
            replies: Vec::new(),
            answers: None,
            babble: false,
            setters: true,
        }
    }
}

struct Supply {
    config: Config,
    volts: u32,
    amps: u32,
    limit: u32,
    output: bool,
    /// What has come of a line so far.
    pending: Vec<u8>,
    /// A CR has ended the line, and the LF the C driver sends is due.
    lf_due: bool,
    /// Answers to `L` so far.
    answered: u32,
}

impl Supply {
    fn new(config: Config) -> Supply {
        Supply {
            volts: config.volts,
            amps: config.amps,
            limit: config.limit,
            output: config.output,
            pending: Vec::new(),
            lf_due: false,
            answered: 0,
            config,
        }
    }

    /// Hundredths of a volt and milliamperes at the terminals.
    fn operating_point(&self) -> (u32, u32) {
        let volts = self.volts.min(self.limit * 100);
        if !self.output {
            return (volts, 0);
        }
        let Some(load) = self.config.load else { return (volts, 0) };
        let limit = self.amps * 10;
        let wanted = volts * 10 / load;
        if wanted <= limit {
            (volts, wanted)
        } else {
            let at_limit = u64::from(limit) * u64::from(load) / 10;
            (u32::try_from(at_limit).unwrap_or(volts).min(volts), limit)
        }
    }

    /// The 37 characters of `L`.
    fn status(&self) -> String {
        let (volts, milliamps) = self.operating_point();
        let tenths_of_watt = u64::from(volts) * u64::from(milliamps) / 10_000;
        let current = if milliamps >= 10_000 {
            format!("{:02}.{:02}", milliamps / 1000, milliamps % 1000 / 10)
        } else {
            format!("{}.{:03}", milliamps / 1000, milliamps % 1000)
        };
        let limit_text = if self.amps >= 1000 {
            format!("{:02}.{}", self.amps / 100, self.amps % 100 / 10)
        } else {
            format!("{}.{:02}", self.amps / 100, self.amps % 100)
        };
        let mode = self.config.limit_mode;
        let letter = |upper: char| {
            let lower = upper.to_ascii_lowercase();
            if mode == Some(lower as u8) { lower } else { upper }
        };
        format!(
            "V{:02}.{:02}A{current}W{:03}.{}{}{:02}{}{limit_text}{}{POWER_LIMIT:03}F{}{}0100",
            volts / 100,
            volts % 100,
            tenths_of_watt / 10,
            tenths_of_watt % 10,
            letter('U'),
            self.limit,
            letter('I'),
            letter('P'),
            u8::from(self.output),
            u8::from(self.config.otp),
        )
    }

    fn all_query(&mut self, port: &mut Port<'_>) {
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return;
        }
        let turn = self.answered as usize;
        self.answered += 1;
        if let Some(reply) = self.config.replies.get(turn) {
            port.send(reply);
            return;
        }
        let mut line = self.status().into_bytes();
        line.extend_from_slice(self.config.eol);
        port.send(&line);
    }

    fn setting(&mut self, text: &str, port: &mut Port<'_>) {
        if !self.config.setters {
            port.violation(format!("'{text}': a setting no one asked for"));
        }
        match parse_setting(text) {
            Some(Setting::Output(on)) => self.output = on,
            Some(Setting::Volts(v)) => self.volts = v.min(self.config.vmax * 100),
            Some(Setting::Limit(v)) => self.limit = v.min(self.config.vmax),
            Some(Setting::Amps(a)) => self.amps = a.min(self.config.imax),
            Some(Setting::Nothing) => {}
            None => port.violation(format!("'{text}': not as the C driver formats it")),
        }
    }

    fn line(&mut self, line: &[u8], port: &mut Port<'_>) {
        if line == b"L" {
            self.all_query(port);
        } else if [&b"SV "[..], b"SU ", b"SI ", b"KOE", b"KOD"].iter().any(|h| line.starts_with(h))
        {
            self.setting(&String::from_utf8_lossy(line), port);
        } else {
            port.violation(format!("'{}': no path of the driver sends it", show(line)));
        }
    }
}

/// A setting; `Nothing` is the C habit of a NaN.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Setting {
    Output(bool),
    Volts(u32),
    Limit(u32),
    Amps(u32),
    Nothing,
}

fn parse_setting(text: &str) -> Option<Setting> {
    match text {
        "KOE" => return Some(Setting::Output(true)),
        "KOD" => return Some(Setting::Output(false)),
        // `%05.2f` and `%04.2f` of a NaN, and `%d` of `ceil` of one on x86.
        "SV   nan" | "SI  nan" | "SU -2147483648" => return Some(Setting::Nothing),
        "SV -0.00" => return Some(Setting::Volts(0)),
        "SI -0.00" => return Some(Setting::Amps(0)),
        _ => {}
    }
    if let Some(value) = text.strip_prefix("SV ") {
        // `%05.2f`: two digits ahead of the point.
        return (value.len() == 5).then(|| parse_centi(value).map(Setting::Volts))?;
    }
    if let Some(value) = text.strip_prefix("SI ") {
        return (value.len() >= 4).then(|| parse_centi(value).map(Setting::Amps))?;
    }
    let value = text.strip_prefix("SU ")?;
    // `%d`: no sign, no leading zero, and only what `ceil` of a range can give.
    let digits = !value.is_empty() && value.bytes().all(|c| c.is_ascii_digit());
    if !digits || value.len() > 2 || (value.len() == 2 && value.starts_with('0')) {
        return None;
    }
    Some(Setting::Limit(value.parse().ok()?))
}

fn show(bytes: &[u8]) -> String {
    bytes.escape_ascii().to_string()
}

impl Serial for Supply {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if std::mem::take(&mut self.lf_due) {
                if byte == b'\n' {
                    continue;
                }
                port.violation("a CR with no LF after it");
            }
            match byte {
                b'\r' => {
                    let line = std::mem::take(&mut self.pending);
                    self.lf_due = true;
                    self.line(&line, port);
                }
                _ if self.pending.len() >= LONGEST || byte == b'\n' => {
                    port.violation(format!("'{}': a line gone wrong", show(&self.pending)));
                    self.pending.clear();
                }
                _ => self.pending.push(byte),
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && (!self.pending.is_empty() || self.lf_due) {
            port.violation(format!("'{}': a command left unfinished", show(&self.pending)));
            self.pending.clear();
            self.lf_due = false;
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if self.config.babble {
            port.send(b"x");
        }
    }

    fn reset(&mut self) {
        *self = Supply::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
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

    fn status(h: &mut Host<Supply>) -> Vec<u8> {
        h.write(b"L\r\n");
        h.read()
    }

    #[test]
    fn the_status_is_37_characters_as_the_manual_says() {
        let mut h = supply(&[]);
        let reply = status(&mut h);
        assert_eq!(reply, b"V12.00A0.000W000.0U60I1.00P200F000100\r\n");
        assert_eq!(reply.len(), 37 + 2);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_manuals_example_comes_out() {
        let mut h = supply(&[
            ("model", "405"),
            ("v", "20.00"),
            ("i", "5.00"),
            ("output", "1"),
            ("load", "8"),
        ]);
        assert_eq!(status(&mut h), b"V20.00A2.500W050.0U40I5.00P200F100100\r\n");
    }

    #[test]
    fn urpsp1_ends_in_cr_cr_lf() {
        let mut h = supply(&[("urpsp", "1")]);
        let reply = status(&mut h);
        assert_eq!(reply.len(), 37 + 3);
        assert!(reply.ends_with(b"\r\r\n"));
    }

    #[test]
    fn settings_take_effect_and_answer_nothing() {
        let mut h = supply(&[("load", "10")]);
        h.write(b"SU 5\r\nSV 05.00\r\nSI 0.25\r\nKOE\r\n");
        assert_eq!(h.read(), b"");
        assert_eq!(status(&mut h), b"V02.50A0.250W000.6U05I0.25P200F100100\r\n");
        h.write(b"SI 3.50\r\n");
        assert_eq!(status(&mut h), b"V05.00A0.500W002.5U05I3.50P200F100100\r\n");
        h.write(b"KOD\r\n");
        assert_eq!(status(&mut h), b"V05.00A0.000W000.0U05I3.50P200F000100\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_frame_stays_37_wide_at_10_amperes() {
        let mut h = supply(&[
            ("model", "2010"),
            ("v", "20.00"),
            ("i", "10.00"),
            ("output", "1"),
            ("load", "1"),
        ]);
        let reply = status(&mut h);
        assert_eq!(reply, b"V10.00A10.00W100.0U20I10.0P200F100100\r\n");
        assert_eq!(reply.len(), 39);
    }

    #[test]
    fn what_the_panel_cannot_hold_is_held_at_its_limit() {
        let mut h = supply(&[("model", "405")]);
        h.write(b"SU 60\r\nSV 60.00\r\nSI 9.00\r\n");
        assert_eq!(status(&mut h), b"V40.00A0.000W000.0U40I5.00P200F000100\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_set_point_above_the_limit_reads_as_the_limit() {
        let mut h = supply(&[]);
        h.write(b"SU 5\r\nSV 20.00\r\n");
        assert_eq!(status(&mut h), b"V05.00A0.000W000.0U05I1.00P200F000100\r\n");
        h.write(b"SU 30\r\n");
        assert_eq!(status(&mut h), b"V20.00A0.000W000.0U30I1.00P200F000100\r\n");
    }

    #[test]
    fn the_flags_and_the_panels_limit_modes_show() {
        let mut h = supply(&[("otp", "1"), ("output", "1")]);
        assert_eq!(status(&mut h)[31..37], *b"110100");
        for (mode, at) in [("u", 18), ("i", 21), ("p", 26)] {
            let mut h = supply(&[("limit-mode", mode)]);
            assert_eq!(status(&mut h)[at], mode.as_bytes()[0]);
        }
    }

    #[test]
    fn what_no_c_path_sends_is_flagged_and_has_no_answer() {
        for bad in [
            &b"V\r\n"[..],
            b"A\r\n",
            b"F\r\n",
            b"KO\r\n",
            b"EEP\r\n",
            b"SV+\r\n",
            b"SP 100\r\n",
            b"l\r\n",
            b"\r\n",
        ] {
            let mut h = supply(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            assert_eq!(h.read(), b"");
        }
    }

    #[test]
    fn a_setting_in_another_format_is_flagged_and_changes_nothing() {
        for bad in [
            &b"SV 5.00\r\n"[..],
            b"SV 05.0\r\n",
            b"SV 005.00\r\n",
            b"SV abcde\r\n",
            b"SV -1.00\r\n",
            b"SU 05\r\n",
            b"SU 100\r\n",
            b"SU -1\r\n",
            b"SU \r\n",
            b"SU 5.0\r\n",
            b"SI 1.5\r\n",
            b"SI 1\r\n",
            b"SI 001.50\r\n",
            b"KOEX\r\n",
            b"KOD \r\n",
        ] {
            let mut h = supply(&[]);
            let before = status(&mut h);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            assert_eq!(status(&mut h), before, "{}", show(bad));
        }
    }

    #[test]
    fn the_c_habits_are_taken() {
        let mut h = supply(&[]);
        let before = status(&mut h);
        h.write(b"SU -2147483648\r\nSV   nan\r\nSI  nan\r\n");
        assert_eq!(violations(&mut h), 0);
        assert_eq!(status(&mut h), before);
        h.write(b"SU 0\r\nSV -0.00\r\nSI -0.00\r\n");
        assert_eq!(status(&mut h), b"V00.00A0.000W000.0U00I0.00P200F000100\r\n");
        h.write(b"SU 5\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_line_end_is_cr_lf() {
        let mut h = supply(&[]);
        h.write(b"L\rL\r\n");
        assert_eq!(violations(&mut h), 1, "a CR with no LF");
        let mut h = supply(&[]);
        h.write(b"L\n");
        assert_eq!(violations(&mut h), 1, "an LF with no CR");
        let mut h = supply(&[]);
        h.write(b"L\r\n");
        assert_eq!(violations(&mut h), 0);
        assert_ne!(h.read(), b"");
    }

    #[test]
    fn a_line_cut_in_two_writes_is_one_line() {
        let mut h = supply(&[]);
        h.write(b"SV 0");
        h.write(b"5.00\r");
        h.write(b"\nL\r\n");
        assert_eq!(violations(&mut h), 0);
        assert_eq!(h.read(), b"V05.00A0.000W000.0U60I1.00P200F000100\r\n");
    }

    #[test]
    fn setters_0_flags_each_setting() {
        for setting in [&b"KOE\r\n"[..], b"KOD\r\n", b"SV 01.00\r\n", b"SU 1\r\n", b"SI 1.00\r\n"] {
            let mut h = supply(&[("setters", "0")]);
            h.write(setting);
            assert_eq!(violations(&mut h), 1, "{}", show(setting));
        }
        let mut h = supply(&[("setters", "0")]);
        h.write(b"L\r\n");
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
        h.write(b"SV");
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        let mut h = supply(&[]);
        h.write(b"L\r");
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = supply(&[("answers", "1")]);
        assert_ne!(status(&mut h), b"");
        assert!(!h.cdc.mark());
        assert_eq!(status(&mut h), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn replies_are_played_in_turn_and_then_its_own() {
        let mut h = supply(&[("replies", "4142,43")]);
        assert_eq!(status(&mut h), b"AB");
        assert_eq!(status(&mut h), b"C");
        assert_eq!(status(&mut h), b"V12.00A0.000W000.0U60I1.00P200F000100\r\n");
    }

    #[test]
    fn behind_its_uart_a_wrong_rate_hears_nothing() {
        let mut h = supply(&[]);
        h.set_line(Line::parse("9600/8n1").unwrap());
        assert_eq!(status(&mut h), b"");
        h.set_line(Line::parse("2400/8n1").unwrap());
        assert_ne!(status(&mut h), b"");
    }

    #[test]
    fn a_load_too_heavy_is_constant_current() {
        let mut h = supply(&[("load", "1"), ("output", "1"), ("v", "12.00"), ("i", "2.00")]);
        assert_eq!(status(&mut h), b"V02.00A2.000W004.0U60I2.00P200F100100\r\n");
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = supply(&[("replies", "41"), ("answers", "2")]);
        assert_eq!(status(&mut h), b"A");
        h.write(b"SV 01.00\r\n");
        assert_ne!(status(&mut h), b"");
        h.cdc.model.reset();
        assert_eq!(status(&mut h), b"A");
        assert_eq!(status(&mut h), b"V12.00A0.000W000.0U60I1.00P200F000100\r\n");
    }

    #[test]
    #[should_panic(expected = "data on every offer")]
    fn a_read_of_a_babbling_model_fails_rather_than_growing() {
        supply(&[("babble", "1")]).read();
    }

    #[test]
    fn babble_sends_at_each_offer() {
        // One offer at a time: `Host::read` offers until nothing comes, which
        // on this device is never.
        let mut quiet = supply(&[]);
        assert!(!matches!(quiet.offer(), Reply::Data(_)));
        let mut h = supply(&[("babble", "1")]);
        for _ in 0..3 {
            assert!(matches!(h.offer(), Reply::Data(d) if d == b"x"));
        }
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("model", "604"),
            ("v", "1.5"),
            ("i", "99.00"),
            ("u", "100"),
            ("output", "2"),
            ("urpsp", "3"),
            ("load", "0"),
            ("limit-mode", "x"),
            ("uart", "2400"),
            ("replies", "4"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("model", "2010"), ("chunk", "1"), ("replies", "41,42"), ("uart", "9600/8n1")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
