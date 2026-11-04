// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Korad `KAxxxxP` power supply and its many badges on the CDC-ACM shell:
//! one channel, ASCII commands without terminators, fixed width answers.
//!
//! The device, and how sure this is:
//!
//! - The USB side: a Nuvoton virtual COM port, `0416:5011`, native USB, so
//!   the line settings do nothing there (tspi.at's write-up of a KA3005P,
//!   `EEVblog`'s "Korad KA3005P I/O Commands"; `Nicoretti/ka3005p` looks for
//!   the vendor id alone). No descriptor dump was found: the rest of the
//!   descriptors are the shell's. `uart=9600/8n1` gives the RS232 port's
//!   behaviour instead, from the sigrok wiki's 9600/8n1 (tspi.at says
//!   19200, which a native port would not show).
//! - The commands: RND's "320-KA Series Remote Control Syntax V2.0", the
//!   sigrok wiki, `api.c` and `protocol.c`. No terminators, no echo, and a
//!   setting has no answer. `VSET1?`, `VOUT1?`, `ISET1?` and `IOUT1?` give
//!   five characters, `STATUS?` one byte.
//! - The status byte: `protocol.c`'s reading, as this model serves that
//!   driver: bit 0 CH1 CV, bit 1 CH2 CV, bit 4 beep, bit 5 OCP, bit 6
//!   output, bit 7 OVP. The sources disagree on the rest: RND's document
//!   has bit 5 as the panel lock and bit 7 unused, the wiki 0x20 as either
//!   protection and only three bits to be trusted, tspi.at bit 4 as OCP
//!   and bit 7 as OVP, neither to be trusted. `status=` plays any other
//!   byte.
//!   Bit 1 reads 0 and beep 1, from the 0x51 a poster on `EEVblog` saw with
//!   the output on.
//! - The sixth byte of `ISET1?`: once `*IDN?` has been asked since power
//!   on, this answer ends in that reply's byte at index 5, on V2.0 and not
//!   on V2.1 (sigrok wiki; ARTIQ's driver reads the K of `KORAD`).
//!   `sixth=0` leaves it out.
//! - The ID strings the scenario plays are those of `api.c`'s table up to
//!   libsigrok commit d241cc2, which listed them literally. 72-2705's is
//!   made from its name, as its commit gave none in the table.
//!
//! Emulation artifacts:
//!
//! - It answers at once. A real supply takes tens of milliseconds and is
//!   said to miss a request that follows too soon (`protocol.c`'s 80 and
//!   450 ms, commit 0f63fbe); a model has no clock, so nothing here checks
//!   the driver's pacing. The port's is checked by a unit test.
//! - It takes a whole command wherever the writes cut it. The real one
//!   cuts on time and is said to lose the rest (`EEVblog`).
//! - `vout=`, `iout=` and `status=` are stand-ins for a load that changes.
//!
//! Not modelled: `BEEP`, `RCL`, `SAV`, `TRACK` and `LOCK`, which the real
//! device has and no path of the C driver sends, so they are flagged; the
//! firmware that answers `*IDN?` once per power cycle (tspi.at), since a
//! scenario's tool always meets a fresh model and so never asks twice; the
//! firmware that says nothing to `STATUS?` with the output off (`EEVblog`),
//! which `answers=` comes close to; the OEM's binary protocol.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `idn=HEX`, the answer to `*IDN?`, or `idn=none` for no answer;
//! - `vmax=VOLTS`, `imax=AMPS`, what the panel takes (31, 5.1): an `api.c`
//!   table's limits;
//! - `vset=NN.NN`, `iset=N.NNN`, `output=0|1`, `ocp=0|1`, `ovp=0|1`,
//!   `beep=0|1`, the state at power on;
//! - `load=OHMS` on the output, or none; it decides CV or CC;
//! - `vout=TEXT[,TEXT...]`, `iout=TEXT[,TEXT...]`, answers played in turn
//!   instead of the output's, any bytes that have no comma: short or
//!   garbage ones are a device gone wrong;
//! - `status=HEX[,HEX...]`, status bytes played in turn;
//! - `sixth=0|1`;
//! - `answers=N`, queries answered per power on, then silence and the
//!   event point;
//! - `setters=0`, which flags any setting, for the cases that show a tool
//!   changes nothing it was not asked to.
//!
//! The pedantic checks: only the commands above, each in the format C's
//! `%05.2f` and `%05.3f` give, within the limits; nothing left of a command
//! when a tool opens. Two C habits are taken, as nothing shows how a real
//! one refuses them and it has no answer to refuse with (`api.c`'s range
//! check is two comparisons): `VSET1:-0.00` and `ISET1:-0.000`, as 0, and
//! `VSET1:  nan` and `ISET1:  nan`, as nothing.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::{self as cable, Packets};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "korad-kaxxxxp";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Korad KAxxxxP power supply on CDC-ACM; idn=HEX|none, vmax=, imax=, \
                         vset=, iset=, output=, ocp=, ovp=, beep=, load=, vout=, iout=, status=, \
                         sixth=, answers=, setters=, uart=, and the shell's stall=, chunk=, \
                         in-limit=, in-stall=, out-hold=";

const DEFAULT_IDN: &[u8] = b"KORADKA3005PV2.0";
/// The longest command: `VSET1:NN.NN`.
const LONGEST: usize = 11;

/// A supply from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn flag(key: &str, value: &str) -> Result<bool> {
    Ok(cable::parse_number(key, value, 0..=1)? == 1)
}

/// `VSET1:`'s five characters, in hundredths of a volt.
fn parse_centi(text: &str) -> Option<u32> {
    let b = text.as_bytes();
    let digits = [0, 1, 3, 4];
    if b.len() != 5 || b[2] != b'.' || !digits.iter().all(|&i| b[i].is_ascii_digit()) {
        return None;
    }
    Some(digits.iter().fold(0, |n, &i| n * 10 + u32::from(b[i] - b'0')))
}

/// `ISET1:`'s five characters, in milliamperes.
fn parse_milli(text: &str) -> Option<u32> {
    let b = text.as_bytes();
    let digits = [0, 2, 3, 4];
    if b.len() != 5 || b[1] != b'.' || !digits.iter().all(|&i| b[i].is_ascii_digit()) {
        return None;
    }
    Some(digits.iter().fold(0, |n, &i| n * 10 + u32::from(b[i] - b'0')))
}

fn parse_limit(key: &str, value: &str) -> Result<u32> {
    let invalid = || Error(format!("{key}={value}: a number as in 31 or 5.1"));
    let number: f64 = value.parse().map_err(|_| invalid())?;
    if !(0.0..=1000.0).contains(&number) {
        return Err(invalid());
    }
    Ok((number * 1000.0).round() as u32)
}

fn answers_list(value: &str) -> Packets {
    Packets::new(value.split(',').map(|t| t.as_bytes().to_vec()).collect())
}

fn function(options: &[(String, String)]) -> Result<Cdc<Supply>> {
    let mut faults = Faults::default();
    let mut uart = None;
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "idn" if v == "none" => config.idn = None,
            "idn" => config.idn = Some(cable::parse_packets(v, 1, 256)?.remove(0)),
            "vmax" => config.vmax = parse_limit(k, v)? / 10,
            "imax" => config.imax = parse_limit(k, v)?,
            "vset" => {
                config.power_on.vset =
                    parse_centi(v).ok_or_else(|| Error(format!("vset={v}: as in 12.34")))?;
            }
            "iset" => {
                config.power_on.iset =
                    parse_milli(v).ok_or_else(|| Error(format!("iset={v}: as in 1.000")))?;
            }
            "output" => config.power_on.output = flag(k, v)?,
            "ocp" => config.power_on.ocp = flag(k, v)?,
            "ovp" => config.power_on.ovp = flag(k, v)?,
            "beep" => config.power_on.beep = flag(k, v)?,
            "load" => config.load = Some(cable::parse_number(k, v, 1..=1_000_000)?),
            "vout" => config.vout = Some(answers_list(v)),
            "iout" => config.iout = Some(answers_list(v)),
            "status" => {
                let list = cable::parse_packets(v, 1, 1)?;
                config.status = Some(Packets::new(list));
            }
            "sixth" => config.sixth = flag(k, v)?,
            "answers" => config.answers = Some(cable::parse_number(k, v, 0..=100_000)?),
            "setters" => config.setters = flag(k, v)?,
            "uart" => uart = Some(Line::parse(v)?),
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    // The strings tspi.at shows. No such dump says more about the rest.
    let id = Identity::new(0x0416, 0x5011, "Nuvoton", "USB Virtual COM");
    let cdc = Cdc::new(NAME, &id, faults, Supply::new(config));
    Ok(match uart {
        Some(line) => cdc.behind_uart(line),
        None => cdc,
    })
}

#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Copy, Debug)]
struct Panel {
    /// Hundredths of a volt.
    vset: u32,
    /// Milliamperes.
    iset: u32,
    output: bool,
    ocp: bool,
    ovp: bool,
    beep: bool,
}

#[derive(Clone, Debug)]
struct Config {
    idn: Option<Vec<u8>>,
    /// Hundredths of a volt.
    vmax: u32,
    /// Milliamperes.
    imax: u32,
    power_on: Panel,
    load: Option<u32>,
    vout: Option<Packets>,
    iout: Option<Packets>,
    status: Option<Packets>,
    sixth: bool,
    answers: Option<u32>,
    setters: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            idn: Some(DEFAULT_IDN.to_vec()),
            vmax: 3100,
            imax: 5100,
            power_on: Panel {
                vset: 1234,
                iset: 1000,
                output: false,
                ocp: false,
                ovp: false,
                beep: true,
            },
            load: None,
            vout: None,
            iout: None,
            status: None,
            sixth: true,
            answers: None,
            setters: true,
        }
    }
}

struct Supply {
    config: Config,
    now: Panel,
    vout: Option<Packets>,
    iout: Option<Packets>,
    status: Option<Packets>,
    /// What has come of a command so far.
    pending: Vec<u8>,
    idn_asked: bool,
    answered: u32,
}

impl Supply {
    fn new(config: Config) -> Supply {
        Supply {
            now: config.power_on,
            vout: config.vout.clone(),
            iout: config.iout.clone(),
            status: config.status.clone(),
            config,
            pending: Vec::new(),
            idn_asked: false,
            answered: 0,
        }
    }

    /// The output's volts in hundredths, amperes in thousandths and
    /// whether it regulates the voltage, from the settings and the load.
    fn operating_point(&self) -> (u32, u32, bool) {
        if !self.now.output {
            return (0, 0, true);
        }
        let Some(load) = self.config.load else { return (self.now.vset, 0, true) };
        let wanted = u64::from(self.now.vset) * 10 / u64::from(load);
        if wanted <= u64::from(self.now.iset) {
            (self.now.vset, u32::try_from(wanted).expect("below iset"), true)
        } else {
            let volts = u64::from(self.now.iset) * u64::from(load) / 10;
            (u32::try_from(volts).unwrap_or(u32::MAX).min(self.now.vset), self.now.iset, false)
        }
    }

    fn status_byte(&mut self) -> u8 {
        if let Some(list) = &mut self.status {
            return list.take()[0];
        }
        let (_, _, cv) = self.operating_point();
        u8::from(cv)
            | u8::from(self.now.beep) << 4
            | u8::from(self.now.ocp) << 5
            | u8::from(self.now.output) << 6
            | u8::from(self.now.ovp) << 7
    }

    /// Whether a query gets its answer.
    fn answers_left(&mut self, port: &mut Port<'_>) -> bool {
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark_event_point();
            return false;
        }
        self.answered += 1;
        true
    }

    fn query(&mut self, command: &[u8], port: &mut Port<'_>) {
        if !self.answers_left(port) {
            return;
        }
        let (vout, iout, _) = self.operating_point();
        let answer: Vec<u8> = match command {
            b"*IDN?" => {
                self.idn_asked = true;
                match &self.config.idn {
                    Some(idn) => idn.clone(),
                    None => return,
                }
            }
            b"VSET1?" => centi(self.now.vset),
            b"ISET1?" => {
                let mut a = milli(self.now.iset);
                if self.config.sixth && self.idn_asked {
                    a.extend(self.config.idn.iter().filter_map(|i| i.get(5)));
                }
                a
            }
            b"VOUT1?" => match &mut self.vout {
                Some(list) => list.take(),
                None => centi(vout),
            },
            b"IOUT1?" => match &mut self.iout {
                Some(list) => list.take(),
                None => milli(iout),
            },
            _ => vec![self.status_byte()],
        };
        port.send(&answer);
    }

    fn setting(&mut self, command: &[u8], port: &mut Port<'_>) {
        if !self.config.setters {
            port.violation(format!("'{}': a setting no one asked for", show(command)));
        }
        let text = String::from_utf8_lossy(command);
        let on = command.last() == Some(&b'1');
        match &command[..3] {
            b"OUT" => self.now.output = on,
            b"OCP" => self.now.ocp = on,
            b"OVP" => self.now.ovp = on,
            _ if text.ends_with("nan") => {}
            b"VSE" => match value_of(&text, parse_centi) {
                Some(v) if v <= self.config.vmax => self.now.vset = v,
                _ => port.violation(format!("'{text}': not NN.NN up to the limit")),
            },
            _ => match value_of(&text, parse_milli) {
                Some(v) if v <= self.config.imax => self.now.iset = v,
                _ => port.violation(format!("'{text}': not N.NNN up to the limit")),
            },
        }
    }

    /// Takes the complete commands off the front of what has come.
    fn commands(&mut self, port: &mut Port<'_>) {
        while !self.pending.is_empty() {
            match frame(&self.pending) {
                Frame::Query(n) => {
                    let command: Vec<u8> = self.pending.drain(..n).collect();
                    self.query(&command, port);
                }
                Frame::Setting(n) => {
                    let command: Vec<u8> = self.pending.drain(..n).collect();
                    self.setting(&command, port);
                }
                Frame::More => return,
                Frame::Unknown => {
                    port.violation(format!("'{}': no command starts so", show(&self.pending)));
                    self.pending.clear();
                }
            }
        }
    }
}

/// The value after the colon, where `api.c`'s habit of letting -0.0 through
/// (`%05.2f` gives `-0.00`, `%05.3f` gives `-0.000`, six characters) is 0.
fn value_of(text: &str, parse: fn(&str) -> Option<u32>) -> Option<u32> {
    let (_, value) = text.split_once(':')?;
    if value == "-0.00" || value == "-0.000" { Some(0) } else { parse(value) }
}

fn centi(v: u32) -> Vec<u8> {
    format!("{:02}.{:02}", v / 100, v % 100).into_bytes()
}

fn milli(v: u32) -> Vec<u8> {
    format!("{}.{:03}", v / 1000, v % 1000).into_bytes()
}

fn show(bytes: &[u8]) -> String {
    bytes.escape_ascii().to_string()
}

enum Frame {
    Query(usize),
    Setting(usize),
    More,
    Unknown,
}

/// What the bytes start with. A protocol with no terminators can only be
/// cut by knowing its commands.
fn frame(b: &[u8]) -> Frame {
    const QUERIES: [&[u8]; 6] =
        [b"*IDN?", b"STATUS?", b"VSET1?", b"ISET1?", b"VOUT1?", b"IOUT1?"];
    const FLAGS: [&[u8]; 6] = [b"OUT0", b"OUT1", b"OCP0", b"OCP1", b"OVP0", b"OVP1"];
    for q in QUERIES {
        if b.starts_with(q) {
            return Frame::Query(q.len());
        }
    }
    for f in FLAGS {
        if b.starts_with(f) {
            return Frame::Setting(f.len());
        }
    }
    for head in [&b"VSET1:"[..], b"ISET1:"] {
        if b.starts_with(head) {
            return value_frame(b, head.len());
        }
    }
    let starts = |c: &&[u8]| c.starts_with(b);
    if QUERIES.iter().any(starts)
        || FLAGS.iter().any(starts)
        || [&b"VSET1:"[..], b"ISET1:"].iter().any(starts)
    {
        Frame::More
    } else {
        Frame::Unknown
    }
}

/// A value's five characters, or six for the habit's `-0.00` and `-0.000`:
/// a sign says one more comes for the current's, not the voltage's.
fn value_frame(b: &[u8], head: usize) -> Frame {
    let width = match (b.get(head), &b[..head]) {
        (Some(b'-'), b"ISET1:") => 6,
        _ => 5,
    };
    if b.len() >= head + width { Frame::Setting(head + width) } else { Frame::More }
}

impl Serial for Supply {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        // Bounded: each pass of commands() leaves at most the start of one
        // command, shorter than LONGEST, or nothing.
        self.pending.extend_from_slice(data);
        self.commands(port);
        debug_assert!(self.pending.len() < LONGEST);
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
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
    fn the_queries_have_their_widths() {
        let mut h = supply(&[]);
        h.write(b"*IDN?");
        assert_eq!(h.read(), DEFAULT_IDN);
        h.write(b"VSET1?");
        assert_eq!(h.read(), b"12.34");
        h.write(b"VOUT1?");
        assert_eq!(h.read(), b"00.00", "the output is off");
        h.write(b"IOUT1?");
        assert_eq!(h.read(), b"0.000");
        h.write(b"STATUS?");
        assert_eq!(h.read(), [0x11]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn iset_has_a_sixth_byte_once_the_id_was_asked() {
        let mut h = supply(&[]);
        h.write(b"ISET1?");
        assert_eq!(h.read(), b"1.000", "no *IDN? in this power cycle");
        h.write(b"*IDN?");
        h.read();
        h.write(b"ISET1?");
        assert_eq!(h.read(), b"1.000K");
        let mut h = supply(&[("sixth", "0")]);
        h.write(b"*IDN?");
        h.read();
        h.write(b"ISET1?");
        assert_eq!(h.read(), b"1.000");
    }

    #[test]
    fn settings_take_effect_and_answer_nothing() {
        let mut h = supply(&[("load", "10")]);
        h.write(b"VSET1:05.00");
        h.write(b"ISET1:2.225");
        h.write(b"OUT1");
        h.write(b"OCP1");
        h.write(b"OVP1");
        assert_eq!(h.read(), b"");
        h.write(b"VSET1?");
        assert_eq!(h.read(), b"05.00");
        h.write(b"ISET1?");
        assert_eq!(h.read(), b"2.225");
        h.write(b"VOUT1?");
        assert_eq!(h.read(), b"05.00");
        h.write(b"IOUT1?");
        assert_eq!(h.read(), b"0.500", "5 V on 10 ohms");
        h.write(b"STATUS?");
        assert_eq!(h.read(), [0xf1]);
        h.write(b"OUT0");
        h.write(b"OCP0");
        h.write(b"OVP0");
        h.write(b"STATUS?");
        assert_eq!(h.read(), [0x11]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_load_too_heavy_is_constant_current() {
        let mut h = supply(&[("load", "1"), ("output", "1"), ("vset", "12.00"), ("iset", "2.000")]);
        h.write(b"VOUT1?");
        assert_eq!(h.read(), b"02.00");
        h.write(b"IOUT1?");
        assert_eq!(h.read(), b"2.000");
        h.write(b"STATUS?");
        assert_eq!(h.read(), [0x50], "bit 0 clear");
    }

    #[test]
    fn a_command_may_come_in_pieces_or_together() {
        let mut h = supply(&[]);
        h.write(b"VS");
        h.write(b"ET1");
        assert_eq!(h.read(), b"");
        h.write(b"?");
        assert_eq!(h.read(), b"12.34");
        h.write(b"VSET1:0");
        h.write(b"3.30IOUT1?VSET1?");
        assert_eq!(h.read(), b"0.00003.30");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_no_c_path_sends_is_flagged() {
        for bad in [
            &b"BEEP1"[..],
            b"RCL1",
            b"SAV1",
            b"TRACK0",
            b"LOCK",
            b"OUT2",
            b"VSET1?\r\n",
            b"*idn?",
            b"X",
        ] {
            let mut h = supply(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
        }
    }

    #[test]
    fn a_value_in_another_format_is_flagged_and_changes_nothing() {
        for bad in [
            &b"VSET1:5.000"[..],
            b"VSET1:31.01",
            b"VSET1:1234x",
            b"ISET1:05.00",
            b"ISET1:5.101",
            b"ISET1:x.000",
        ] {
            let mut h = supply(&[]);
            h.write(bad);
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            h.write(b"VSET1?");
            assert_eq!(h.read(), b"12.34");
            h.write(b"ISET1?");
            assert_eq!(h.read(), b"1.000");
        }
    }

    #[test]
    fn the_limits_are_options() {
        let mut h = supply(&[("vmax", "61"), ("imax", "3.1")]);
        h.write(b"VSET1:61.00");
        h.write(b"ISET1:3.100");
        assert_eq!(violations(&mut h), 0);
        h.write(b"VSET1:61.01");
        h.write(b"ISET1:3.101");
        assert_eq!(violations(&mut h), 2);
    }

    #[test]
    fn the_c_habit_of_a_negative_zero_is_taken_as_zero() {
        let mut h = supply(&[]);
        h.write(b"VSET1:-0.00");
        h.write(b"ISET1:-0.000");
        assert_eq!(violations(&mut h), 0);
        h.write(b"VSET1?");
        assert_eq!(h.read(), b"00.00");
        h.write(b"ISET1?");
        assert_eq!(h.read(), b"0.000");
    }

    #[test]
    fn the_c_habit_of_a_nan_changes_nothing() {
        let mut h = supply(&[]);
        h.write(b"VSET1:  nan");
        h.write(b"ISET1:  nan");
        assert_eq!(violations(&mut h), 0);
        h.write(b"VSET1?");
        assert_eq!(h.read(), b"12.34");
        h.write(b"ISET1?");
        assert_eq!(h.read(), b"1.000");
    }

    #[test]
    fn setters_0_flags_each_setting() {
        for setting in [&b"OUT0"[..], b"OUT1", b"OCP1", b"OVP0", b"VSET1:01.00", b"ISET1:1.000"] {
            let mut h = supply(&[("setters", "0")]);
            h.write(setting);
            assert_eq!(violations(&mut h), 1, "{}", show(setting));
        }
        let mut h = supply(&[("setters", "0")]);
        h.write(b"STATUS?");
        h.write(b"*IDN?");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_is_kept_of_a_command_stays_short() {
        let mut h = supply(&[]);
        for part in [&b"VSET1:01."[..], b"00VSET1:0", &[b'x'; 4096]] {
            h.write(part);
            assert!(h.cdc.model.pending.len() < LONGEST);
        }
        assert_eq!(violations(&mut h), 2, "VSET1:0xxxx, then the rest");
        assert_eq!(h.cdc.model.pending, b"");
    }

    #[test]
    fn a_command_left_unfinished_shows_at_the_next_open() {
        let mut h = supply(&[]);
        h.write(b"VSET");
        h.set_lines(false, false);
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = supply(&[("answers", "1")]);
        h.write(b"*IDN?");
        assert_eq!(h.read(), DEFAULT_IDN);
        assert!(!h.cdc.take_event_point());
        h.write(b"IOUT1?");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.take_event_point());
        h.write(b"OUT1");
        assert_eq!(violations(&mut h), 0, "a setting still arrives");
    }

    #[test]
    fn answers_played_in_turn() {
        let mut h = supply(&[("vout", "01.00,xx.xx,1."), ("iout", "0.100"), ("status", "01,80")]);
        for want in [&b"01.00"[..], b"xx.xx", b"1.", b"01.00"] {
            h.write(b"VOUT1?");
            assert_eq!(h.read(), want);
        }
        h.write(b"IOUT1?");
        assert_eq!(h.read(), b"0.100");
        h.write(b"STATUS?");
        assert_eq!(h.read(), [0x01]);
        h.write(b"STATUS?");
        assert_eq!(h.read(), [0x80]);
    }

    #[test]
    fn no_id_means_no_answer_and_no_sixth_byte() {
        let mut h = supply(&[("idn", "none")]);
        h.write(b"*IDN?");
        assert_eq!(h.read(), b"");
        h.write(b"ISET1?");
        assert_eq!(h.read(), b"1.000");
    }

    #[test]
    fn the_rs232_port_takes_its_own_line() {
        let mut h = supply(&[("uart", "9600/8n1")]);
        h.write(b"VSET1?");
        assert_eq!(h.read(), b"12.34");
        h.set_line(Line::parse("19200/8n1").unwrap());
        h.write(b"VSET1?");
        assert_eq!(h.read(), b"");
        assert_eq!(violations(&mut h), 0);
        // A native port does not mind.
        let mut h = supply(&[]);
        h.set_line(Line::parse("19200/8n1").unwrap());
        h.write(b"VSET1?");
        assert_eq!(h.read(), b"12.34");
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = supply(&[("vout", "01.00,02.00"), ("answers", "4")]);
        h.write(b"*IDN?");
        h.write(b"VSET1:01.00");
        h.write(b"VOUT1?");
        h.read();
        h.cdc.model.reset();
        h.write(b"VSET1?");
        assert_eq!(h.read(), b"12.34");
        h.write(b"VOUT1?");
        assert_eq!(h.read(), b"01.00");
        h.write(b"ISET1?");
        assert_eq!(h.read(), b"1.000", "*IDN? forgotten");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("idn", "4"),
            ("vset", "1.5"),
            ("iset", "10.00"),
            ("output", "2"),
            ("status", "0102"),
            ("vmax", "x"),
            ("load", "0"),
            ("uart", "9600"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("idn", "none"), ("idn", "4b"), ("chunk", "1"), ("vout", "x"), ("sixth", "0")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
