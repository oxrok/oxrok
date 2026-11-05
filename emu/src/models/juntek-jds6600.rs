// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Juntek JDS6600 DDS signal generator, two channels, sold under other
//! names too (Joy-IT), on the CDC-ACM shell: text requests and text replies.
//!
//! The device, and how sure this is:
//!
//! - The USB side: a WCH CH340G in front of a microcontroller's UART at
//!   115200/8n1 (`protocol.c`'s notes). The UART is modelled, so another
//!   line passes nothing. The ids and strings are not the chip's: a real
//!   CH340's would bring the `ch341` driver and no ttyACM.
//! - The requests: `:` a letter, a two digit register, `=`, values with
//!   commas, `.`, end of line. `r` reads a register and `w` writes one; `a`
//!   and `b` move arbitrary waveforms. `protocol.c`'s notes and examples,
//!   and the same framing and register numbers in Kristoff Bonne's
//!   `on1arf/jds6600_python` (which ends a line in LF alone). A read's `=0`
//!   means nothing certain; the notes say any value worked. The Joy-IT
//!   protocol PDF could not be fetched, so it is not used here.
//! - The replies: `:rNN=VALUE.` with CR LF on a read, `:ok` with CR LF on a
//!   write, byte for byte in `protocol.c`'s examples. The Python code also
//!   checks for those.
//! - The values: register 0 the model's top frequency in MHz, 1 a serial
//!   number, 20 two enable flags, 21 and 22 a waveform (0 to 16 built in,
//!   101 to 160 arbitrary), 23 and 24 centihertz and a scale code, 25 and 26
//!   millivolts, 27 and 28 tens of millivolts plus 10 V, 29 and 30 tenths of
//!   a percent, 31 tenths of a degree. Both sources agree.
//! - What a serial number looks like: neither source shows one. `1234567890`
//!   is a stand-in, as is the rest of the state at power on, but for `60`,
//!   `1,1` and `103`, which `protocol.c`'s examples show.
//!
//! Emulation artifacts:
//!
//! - It answers at once. The notes measured a real one within 2 ms at the
//!   UART, so this is close, and a model has no clock to check more with.
//! - A write is stored as its text and read back so. A real one may clip or
//!   move a value (the notes say amplitude and offset depend on the rest of
//!   the state, and the Python code says offsets stop at 2.5 V).
//! - The notes say a request typed by hand gets its `:ok` and does nothing.
//!   Nothing here tells a person from a program.
//!
//! Not modelled: modes, sweep, measurement, the counter, profiles and system
//! settings, and the arbitrary waveforms' `a` and `b`, which the real device
//! has and which the driver never sends (its waveform code is compiled out),
//! so they are flagged.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `rNN=TEXT`, the value of a register that reads, at power on;
//! - `rawNN=HEX|none`, what a read of register NN gets instead of its reply,
//!   whole line included: a device gone wrong, or silence;
//! - `ok=HEX|none`, what a write gets instead of `:ok` CR LF;
//! - `after=N`, `rawNN=` and `ok=` take over only after N requests, so that
//!   a scan passes and the fault meets what follows;
//! - `flood=N`, what a read gets instead, also after `after=`: N bytes with
//!   no end of line;
//! - `answers=N`, requests answered per power on, then silence and the
//!   event point; a write that goes unanswered still takes effect;
//! - `setters=0`, which flags any write, for the cases that show a tool
//!   changes nothing it was not asked to.
//!
//! The pedantic checks: only the registers above, reads with `=0`, every line
//! ending in `.` CR LF, each value in the form and range its register has;
//! nothing left of a line when a tool opens. Some values are taken although
//! nothing shows a real one accepts them, as a device that says `:ok` to
//! what it cannot do has no way to refuse: `nan` and `-nan` on what a
//! driver's limits do not catch, changing nothing, and `-0` and negative
//! phases, kept as they came (the Python code sends a negative phase plus
//! 360 degrees, which suggests the firmware might not take the sign).

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::{Error, Result};
use std::collections::BTreeMap;

/// The model name.
pub const NAME: &str = "juntek-jds6600";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Juntek JDS6600 signal generator behind a UART on CDC-ACM; rNN=TEXT, \
                         rawNN=HEX|none, ok=HEX|none, after=N, flood=N, answers=N, setters=0, and the shell's \
                         stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// The UART behind the bridge chip.
const UART: &str = "115200/8n1";
/// The longest request worth keeping: `:w24=6000000000,0.` and its end of
/// line fit several times over. Past this a line is flagged and dropped.
const LONGEST: usize = 64;
/// The longest reply an option may set.
const LONGEST_RAW: usize = 1024;
/// The most `flood=` sends in one go.
const LONGEST_FLOOD: u32 = 64 * 1024;

/// Registers a read may ask for, with their values at power on.
const POWER_ON: [(u8, &str); 14] = [
    (0, "60"),
    (1, "1234567890"),
    (20, "1,1"),
    (21, "103"),
    (22, "0"),
    (23, "100000,0"),
    (24, "100000,0"),
    (25, "5000"),
    (26, "5000"),
    (27, "1000"),
    (28, "1000"),
    (29, "500"),
    (30, "500"),
    (31, "0"),
];

/// A generator from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

/// A register's number from an option's key, after its prefix.
fn register(key: &str, prefix: &str) -> Option<u8> {
    let digits = key.strip_prefix(prefix)?;
    if digits.len() != 2 {
        return None;
    }
    let reg = digits.parse().ok()?;
    POWER_ON.iter().any(|(r, _)| *r == reg).then_some(reg)
}

/// `HEX`, or `none` for no reply at all.
fn reply_option(key: &str, value: &str) -> Result<Vec<u8>> {
    if value == "none" {
        return Ok(Vec::new());
    }
    Ok(parse_hex_list(key, value, 1, LONGEST_RAW)?.remove(0))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Generator>> {
    let mut faults = Faults::default();
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        if let Some(reg) = register(k, "raw") {
            config.raw.insert(reg, reply_option(k, v)?);
        } else if let Some(reg) = register(k, "r") {
            config.power_on.insert(reg, v.clone());
        } else {
            match k.as_str() {
                "ok" => config.bad_ok = Some(reply_option(k, v)?),
                "after" => config.after = parse_number(k, v, 0..=100_000)?,
                "flood" => config.flood = parse_number(k, v, 1..=LONGEST_FLOOD)?,
                "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
                "setters" => config.setters = parse_number(k, v, 0..=1)? == 1,
                other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
            }
        }
    }
    // Not the chip's: see the module's documentation.
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "JDS6600 stand-in");
    Ok(Cdc::new(NAME, &id, faults, Generator::new(config)).behind_uart(Line::parse(UART)?))
}

#[derive(Clone, Debug)]
struct Config {
    power_on: BTreeMap<u8, String>,
    raw: BTreeMap<u8, Vec<u8>>,
    bad_ok: Option<Vec<u8>>,
    after: u32,
    flood: u32,
    answers: Option<u32>,
    setters: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            power_on: POWER_ON.iter().map(|(r, v)| (*r, (*v).to_owned())).collect(),
            raw: BTreeMap::new(),
            bad_ok: None,
            after: 0,
            flood: 0,
            answers: None,
            setters: true,
        }
    }
}

struct Generator {
    config: Config,
    now: BTreeMap<u8, String>,
    /// What has come of a line so far.
    pending: Vec<u8>,
    answered: u32,
}

impl Generator {
    fn new(config: Config) -> Generator {
        Generator { now: config.power_on.clone(), config, pending: Vec::new(), answered: 0 }
    }

    /// Whether the faults have begun, for the request just counted.
    fn faulty(&self) -> bool {
        self.answered > self.config.after
    }

    /// Whether a request gets its answer.
    fn answers_left(&mut self, port: &mut Port<'_>) -> bool {
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return false;
        }
        self.answered += 1;
        true
    }

    /// The top frequency in centihertz, when register 0 holds a number.
    fn top_centihertz(&self) -> Option<u64> {
        let mhz: u64 = self.now.get(&0)?.parse().ok()?;
        mhz.checked_mul(100_000_000)
    }

    fn read(&mut self, reg: u8, value: &str, port: &mut Port<'_>) {
        if !POWER_ON.iter().any(|(r, _)| *r == reg) {
            port.violation(format!("r{reg:02}: a register no driver path reads"));
            return;
        }
        if value != "0" {
            port.violation(format!("r{reg:02}={value}: a read has =0"));
        }
        if !self.answers_left(port) {
            return;
        }
        if self.faulty() && self.config.flood > 0 {
            port.send(&vec![b'x'; self.config.flood as usize]);
            return;
        }
        if self.faulty()
            && let Some(raw) = self.config.raw.get(&reg)
        {
            port.send(raw);
            return;
        }
        let reply = format!(":r{reg:02}={}.\r\n", self.now[&reg]);
        port.send(reply.as_bytes());
    }

    fn write(&mut self, reg: u8, value: &str, port: &mut Port<'_>) {
        if !self.config.setters {
            port.violation(format!("w{reg:02}={value}: a setting no one asked for"));
        }
        let top = self.top_centihertz();
        match check_write(reg, value, top) {
            Written::Store => {
                self.now.insert(reg, value.to_owned());
            }
            Written::Ignore => {}
            Written::Bad(why) => port.violation(format!("w{reg:02}={value}: {why}")),
        }
        if self.answers_left(port) {
            match &self.config.bad_ok {
                Some(reply) if self.faulty() => port.send(reply),
                _ => port.send(b":ok\r\n"),
            }
        }
    }

    /// One whole line, with its line feed taken off.
    fn line(&mut self, line: &[u8], port: &mut Port<'_>) {
        let shown = line.escape_ascii().to_string();
        let Some(body) = line.strip_suffix(b".\r").and_then(|b| std::str::from_utf8(b).ok()) else {
            port.violation(format!("'{shown}': does not end in a period, CR, LF"));
            return;
        };
        let Some((insn, reg, value)) = fields(body) else {
            port.violation(format!("'{shown}': not :IDD=VALUE"));
            return;
        };
        match insn {
            b'r' => self.read(reg, value, port),
            b'w' => self.write(reg, value, port),
            _ => port.violation(format!("'{shown}': an instruction no driver path sends")),
        }
    }
}

/// `:IDD=VALUE`'s letter, two digit register and value.
fn fields(body: &str) -> Option<(u8, u8, &str)> {
    let rest = body.strip_prefix(':')?;
    let (head, value) = rest.split_once('=')?;
    let b = head.as_bytes();
    if b.len() != 3 || !b[1].is_ascii_digit() || !b[2].is_ascii_digit() {
        return None;
    }
    Some((b[0], (b[1] - b'0') * 10 + (b[2] - b'0'), value))
}

/// What a write does to its register.
enum Written {
    Store,
    /// It says `:ok` and nothing changes.
    Ignore,
    Bad(&'static str),
}

/// Plain digits with an optional minus sign, within the range.
fn in_range(value: &str, range: std::ops::RangeInclusive<i64>) -> bool {
    let digits = value.strip_prefix('-').unwrap_or(value);
    !digits.is_empty()
        && digits.bytes().all(|c| c.is_ascii_digit())
        && value.parse().is_ok_and(|n| range.contains(&n))
}

fn is_nan(value: &str) -> bool {
    value == "nan" || value == "-nan"
}

fn scaled(value: &str, range: std::ops::RangeInclusive<i64>, why: &'static str) -> Written {
    if is_nan(value) {
        Written::Ignore
    } else if value == "-0" || in_range(value, range) {
        Written::Store
    } else {
        Written::Bad(why)
    }
}

fn check_write(reg: u8, value: &str, top: Option<u64>) -> Written {
    match reg {
        20 => match value.as_bytes() {
            [a, b',', b] if b"01".contains(a) && b"01".contains(b) => Written::Store,
            _ => Written::Bad("not two flags"),
        },
        21 | 22 => {
            if in_range(value, 0..=16) || in_range(value, 101..=160) {
                Written::Store
            } else {
                Written::Bad("no such waveform")
            }
        }
        23 | 24 => {
            let Some(mantissa) = value.strip_suffix(",0") else {
                return Written::Bad("not a frequency with scale 0");
            };
            if is_nan(mantissa) {
                return Written::Ignore;
            }
            let top = top.map_or(i64::MAX, |t| i64::try_from(t).unwrap_or(i64::MAX));
            if in_range(mantissa, 1..=top) {
                Written::Store
            } else {
                Written::Bad("beyond the model's frequencies")
            }
        }
        25 | 26 => scaled(value, 0..=20_000, "beyond 0 to 20 V"),
        27 | 28 => scaled(value, 1..=1999, "beyond -9.99 to 9.99 V"),
        29 | 30 => scaled(value, 0..=1000, "beyond 0 to 100 %"),
        31 => scaled(value, -3599..=3599, "beyond a whole turn"),
        _ => Written::Bad("a register no driver path writes"),
    }
}

impl Serial for Generator {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte == b'\n' {
                let line = std::mem::take(&mut self.pending);
                self.line(&line, port);
            } else if self.pending.len() < LONGEST {
                self.pending.push(byte);
            } else {
                port.violation(format!(
                    "'{}': a line too long",
                    self.pending.escape_ascii()
                ));
                self.pending.clear();
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("'{}': a line left unfinished", self.pending.escape_ascii()));
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
        Host::open(function(&o).unwrap(), Line::parse(UART).unwrap(), true, true)
    }

    fn violations(h: &mut Host<Generator>) -> usize {
        h.v.drain().len()
    }

    fn ask(h: &mut Host<Generator>, request: &str) -> Vec<u8> {
        h.write(format!("{request}.\r\n").as_bytes());
        h.read()
    }

    #[test]
    fn the_examples_on_the_wire() {
        let mut h = generator(&[]);
        assert_eq!(ask(&mut h, ":r00=0"), b":r00=60.\r\n");
        assert_eq!(ask(&mut h, ":r20=0"), b":r20=1,1.\r\n");
        assert_eq!(ask(&mut h, ":r21=0"), b":r21=103.\r\n");
        assert_eq!(ask(&mut h, ":w24=1234500,0"), b":ok\r\n");
        assert_eq!(ask(&mut h, ":r24=0"), b":r24=1234500,0.\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn every_register_reads_as_at_power_on() {
        let mut h = generator(&[]);
        for (reg, value) in POWER_ON {
            let want = format!(":r{reg:02}={value}.\r\n");
            assert_eq!(ask(&mut h, &format!(":r{reg:02}=0")), want.as_bytes());
        }
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn every_setting_takes_effect() {
        for (reg, value) in [
            (20, "0,1"),
            (21, "16"),
            (22, "160"),
            (23, "1"),
            (24, "6000000000"),
            (25, "20000"),
            (26, "0"),
            (27, "1"),
            (28, "1999"),
            (29, "1000"),
            (30, "0"),
            (31, "3599"),
            (31, "-3599"),
            (31, "-0"),
            (25, "-0"),
            (29, "-0"),
        ] {
            let mut h = generator(&[]);
            let value = if (23..=24).contains(&reg) { format!("{value},0") } else { value.to_owned() };
            assert_eq!(ask(&mut h, &format!(":w{reg:02}={value}")), b":ok\r\n");
            let want = format!(":r{reg:02}={value}.\r\n");
            assert_eq!(ask(&mut h, &format!(":r{reg:02}=0")), want.as_bytes());
            assert_eq!(violations(&mut h), 0, "{reg} {value}");
        }
    }

    #[test]
    fn a_nan_is_taken_and_changes_nothing() {
        for (reg, value, before) in [
            (23, "nan,0", "100000,0"),
            (24, "-nan,0", "100000,0"),
            (25, "nan", "5000"),
            (27, "-nan", "1000"),
            (29, "nan", "500"),
            (31, "-nan", "0"),
        ] {
            let mut h = generator(&[]);
            assert_eq!(ask(&mut h, &format!(":w{reg:02}={value}")), b":ok\r\n");
            let want = format!(":r{reg:02}={before}.\r\n");
            assert_eq!(ask(&mut h, &format!(":r{reg:02}=0")), want.as_bytes());
            assert_eq!(violations(&mut h), 0, "{reg} {value}");
        }
    }

    #[test]
    fn a_value_out_of_form_or_range_is_flagged_and_changes_nothing() {
        for (reg, value) in [
            (20, "1"),
            (20, "2,1"),
            (20, "1,1,1"),
            (21, "17"),
            (21, "100"),
            (22, "161"),
            (22, "x"),
            (23, "0,0"),
            (23, "100"),
            (23, "100,1"),
            (24, "6000000001,0"),
            (24, "x,0"),
            (25, "20001"),
            (26, "-1"),
            (27, "0"),
            (28, "2000"),
            (29, "1001"),
            (30, "-1"),
            (31, "3600"),
            (31, "-3600"),
            (25, ""),
            (31, "1.5"),
        ] {
            let mut h = generator(&[]);
            let before = ask(&mut h, &format!(":r{reg:02}=0"));
            ask(&mut h, &format!(":w{reg:02}={value}"));
            assert_eq!(violations(&mut h), 1, "{reg} {value}");
            assert_eq!(ask(&mut h, &format!(":r{reg:02}=0")), before);
        }
    }

    #[test]
    fn the_top_frequency_follows_register_0() {
        let mut h = generator(&[("r00", "15")]);
        ask(&mut h, ":w23=1500000000,0");
        assert_eq!(violations(&mut h), 0);
        ask(&mut h, ":w23=1500000001,0");
        assert_eq!(violations(&mut h), 1);
        // With no number there, nothing can be said of the limit.
        let mut h = generator(&[("r00", "junk")]);
        ask(&mut h, ":w23=9000000000,0");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_no_driver_path_sends_is_flagged() {
        for bad in [
            &b":r00=0\r\n"[..],
            b":r00=0.\n",
            b":r00=1.\r\n",
            b":r0=0.\r\n",
            b":r000=0.\r\n",
            b"r00=0.\r\n",
            b":r00.\r\n",
            b":r02=0.\r\n",
            b":r32=0.\r\n",
            b":w00=1.\r\n",
            b":w01=1.\r\n",
            b":w32=1.\r\n",
            b":a01=1.\r\n",
            b":b01=0.\r\n",
            b":\xff00=0.\r\n",
            b"\n",
        ] {
            let mut h = generator(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{}", bad.escape_ascii());
        }
    }

    #[test]
    fn a_line_may_come_in_pieces_or_together() {
        let mut h = generator(&[]);
        h.write(b":r0");
        h.write(b"0=0.\r");
        assert_eq!(h.read(), b"");
        h.write(b"\n:r22=0.\r\n");
        assert_eq!(h.read(), b":r00=60.\r\n:r22=0.\r\n");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_is_kept_of_a_line_stays_short() {
        let mut h = generator(&[]);
        h.write(&[b'x'; 4096]);
        assert!(h.cdc.model.pending.len() <= LONGEST);
        assert!(violations(&mut h) > 0);
    }

    #[test]
    fn a_line_left_unfinished_shows_at_the_next_open() {
        let mut h = generator(&[]);
        h.write(b":r00");
        h.set_lines(false, false);
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = generator(&[("answers", "1")]);
        assert_eq!(ask(&mut h, ":r00=0"), b":r00=60.\r\n");
        assert!(!h.cdc.mark());
        assert_eq!(ask(&mut h, ":w25=1000"), b"");
        assert!(h.cdc.mark());
        assert_eq!(h.cdc.model.now[&25], "1000", "a write unanswered still counts");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn raw_replies_stand_in_for_a_register_and_for_ok() {
        let mut h = generator(&[("raw00", "3a78"), ("raw01", "none"), ("ok", "3a6572720d0a")]);
        assert_eq!(ask(&mut h, ":r00=0"), b":x");
        assert_eq!(ask(&mut h, ":r01=0"), b"");
        assert_eq!(ask(&mut h, ":w25=1000"), b":err\r\n");
        let mut h = generator(&[("ok", "none")]);
        assert_eq!(ask(&mut h, ":w25=1000"), b"");
    }

    #[test]
    fn after_holds_the_faults_back() {
        let mut h = generator(&[("after", "2"), ("raw00", "3a78"), ("ok", "none")]);
        assert_eq!(ask(&mut h, ":r00=0"), b":r00=60.\r\n");
        assert_eq!(ask(&mut h, ":w25=1"), b":ok\r\n");
        assert_eq!(ask(&mut h, ":r00=0"), b":x");
        assert_eq!(ask(&mut h, ":w25=1"), b"");
    }

    #[test]
    fn a_flood_has_no_end_of_line() {
        let mut h = generator(&[("after", "1"), ("flood", "100")]);
        assert_eq!(ask(&mut h, ":r00=0"), b":r00=60.\r\n");
        h.write(b":r01=0.\r\n");
        let got = h.read();
        assert_eq!(got.len(), 100);
        assert!(!got.contains(&b'\n'));
    }

    #[test]
    fn setters_0_flags_each_write() {
        let mut h = generator(&[("setters", "0")]);
        ask(&mut h, ":r20=0");
        assert_eq!(violations(&mut h), 0);
        ask(&mut h, ":w20=1,1");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn another_line_passes_nothing() {
        let mut h = generator(&[]);
        assert_eq!(ask(&mut h, ":r00=0"), b":r00=60.\r\n");
        h.set_line(Line::parse("9600/8n1").unwrap());
        assert_eq!(ask(&mut h, ":r00=0"), b"");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = generator(&[("answers", "2")]);
        ask(&mut h, ":w25=1000");
        ask(&mut h, ":r25=0");
        assert_eq!(ask(&mut h, ":r25=0"), b"");
        h.cdc.model.reset();
        assert_eq!(ask(&mut h, ":r25=0"), b":r25=5000.\r\n");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("r02", "1"),
            ("r0", "1"),
            ("raw", "00"),
            ("raw00", "4"),
            ("ok", "zz"),
            ("answers", "x"),
            ("setters", "2"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("r20", "0,1"), ("raw20", "none"), ("ok", "none"), ("chunk", "1")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
