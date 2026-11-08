// Counterpart in sigrok:
//   libsigrok/src/hardware/scpi-pps/api.c,
//   libsigrok/src/hardware/scpi-pps/protocol.c,
//   libsigrok/src/hardware/scpi-pps/protocol.h,
//   libsigrok/src/hardware/scpi-pps/profiles.c,
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2017,2019 Frank Stettner <frank-stettner@gmx.net>,
//   Copyright (C) 2019 Frank Stettner <frank-stettner@gmx.net>,
//   Copyright (C) 2015 Google, Inc.,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Martin Ling,
//   Aurelien Jacobs, Gerhard Sittig, Soeren Apel, Alexandru Gagniuc,
//   Florian Schmidt, Daniel Elstner, Kota Ando, Marc Schink, Martin Atkins,
//   MGWGIT, Mike Walters, Mike Williams, Nicolas Labriet, Petteri Aimonen,
//   Sahaj Sarup
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! What goes over the line, and how an answer reads.
//!
//! An answer has no checksum and says nothing of what it answers, so on a line with
//! no tags (TCP, serial) nothing tells a late answer from the next one but keeping
//! one question out at a time. [`Link::measure`] asks once and keeps reading, for
//! [`PATIENCE`] reads, as C over TCP does by blocking, rather than asking again: an answer
//! late by a minute is then still the right one. USBTMC's tags let it ask again at
//! each timeout, as C. An answer that came cut short, with no line end, is no reading:
//! the line is found again by asking who the instrument is and dropping what comes
//! before that answer ([`Link`]'s resync), as the tail could otherwise stand for the
//! next answer for good. A question an earlier capture left out is given up at the
//! next start, and its answer drained.
//!
//! A reading is a number of the forms SCPI has, and [`number`] refuses what `strtod`
//! makes something of: an empty answer, where C has 0, and hexadecimal, `nan` and
//! `inf`. What the format can't catch: a digit changed into a digit, a lost digit or
//! sign, and a line end lost along with the next answer's first digits. See the test
//! `what_damage_a_reading_survives`, which counts them.

use super::profiles::{Dialect, Profile};
use crate::core::{Error, Key, Packet, Result, Value};
use crate::scpi::{CmdArg, Scpi};
use std::time::Duration;

/// `enum pps_scpi_cmds`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum Cmd {
    Remote,
    Local,
    Beeper,
    BeeperEnable,
    BeeperDisable,
    SelectChannel,
    GetMeasVoltage,
    GetMeasCurrent,
    GetMeasPower,
    GetMeasFrequency,
    GetVoltageTarget,
    SetVoltageTarget,
    GetFrequencyTarget,
    SetFrequencyTarget,
    GetCurrentLimit,
    SetCurrentLimit,
    GetOutputEnabled,
    SetOutputEnable,
    SetOutputDisable,
    GetOutputRegulation,
    GetOverTemperatureProtection,
    SetOverTemperatureProtectionEnable,
    SetOverTemperatureProtectionDisable,
    GetOverTemperatureProtectionActive,
    GetOverVoltageProtectionEnabled,
    SetOverVoltageProtectionEnable,
    SetOverVoltageProtectionDisable,
    GetOverVoltageProtectionActive,
    GetOverVoltageProtectionThreshold,
    SetOverVoltageProtectionThreshold,
    GetOverCurrentProtectionEnabled,
    SetOverCurrentProtectionEnable,
    SetOverCurrentProtectionDisable,
    GetOverCurrentProtectionActive,
    GetOverCurrentProtectionThreshold,
    SetOverCurrentProtectionThreshold,
    GetOverCurrentProtectionDelay,
    SetOverCurrentProtectionDelay,
}

/// How many read timeouts, 1 s each, a capture on a line with no tags waits for the
/// answer to one question before it counts that as lost and asks again. So a dead
/// question costs no more than this, and only an answer later still can stand one
/// reading ahead.
pub const PATIENCE: u32 = 30;
/// How many a `--get` waits for its answer on a line with no tags. C over TCP waits for
/// ever, and over a serial port 1 s.
pub const GET_PATIENCE: u32 = 5;
/// A capture that has had no reading ends in failure after this long of silence. C
/// asks for ever. A choice: a supply answers a reading in milliseconds to a second.
/// An HP 66xxB measures over `SENS:SWE:POIN` points (0 to 4096) `SENS:SWE:TINT` apart
/// (15.6 us to 31200 s, by the 6613C's guide), which can be far longer, so a capture on
/// one asks and waits twice that ([`sweep_bound`]). Under a time limit there's a
/// warning and no failure.
pub const NO_SAMPLE_BOUND: Duration = Duration::from_secs(180);
/// How long a capture with no reading yet takes answers that are no reading.
pub const NO_USABLE_BOUND: Duration = Duration::from_secs(30);
/// After a reading, how long nothing may come before the capture fails.
pub const LONG_BOUND: Duration = Duration::from_secs(600);
/// How long nothing may come before a warning says so.
pub const WARN_AFTER: Duration = Duration::from_secs(3);
/// How far past a time limit a capture waits for the answer to a question it asked.
pub const LIMIT_GRACE: Duration = Duration::from_secs(5);

/// The bounds a capture on an HP 66xxB takes from its sweep, `points` readings `tint`
/// seconds apart: twice what a reading takes, and no less than the usual ones. `None`
/// for a sweep no longer than the usual bound allows, or one that is no sweep.
pub fn sweep_bound(points: f64, tint: f64) -> Option<Duration> {
    let takes = points * tint;
    if !(takes.is_finite() && takes > 0.0) {
        return None;
    }
    let bound = Duration::try_from_secs_f64(2.0 * takes).unwrap_or(Duration::MAX);
    (bound > NO_SAMPLE_BOUND).then_some(bound)
}

/// What [`Watch`] has to say.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Nothing.
    Go,
    /// Say this, and go on.
    Warn(String),
    /// End the capture in failure with this.
    Fail(String),
}

/// Whether a capture's device still gives anything: a rule with its time passed in.
///
/// Time counts as what the empty reads waited, each at most twice what it asked for,
/// so a stopped process or a slow consumer is no silence of the device's, while one
/// stopped over and over still gets there.
#[derive(Clone, Debug)]
pub struct Watch {
    /// Under a time limit it warns and never fails.
    time_limited: bool,
    /// A reading has come in this capture.
    sampled: bool,
    silent: Duration,
    /// As [`Watch::silent`], for answers that were no reading.
    unusable: Duration,
    warned: bool,
    no_sample_bound: Duration,
    no_usable_bound: Duration,
    long_bound: Duration,
    warn_after: Duration,
}

impl Watch {
    /// With the bounds above.
    pub fn new(time_limited: bool) -> Watch {
        Watch {
            time_limited,
            sampled: false,
            silent: Duration::ZERO,
            unusable: Duration::ZERO,
            warned: false,
            no_sample_bound: NO_SAMPLE_BOUND,
            no_usable_bound: NO_USABLE_BOUND,
            long_bound: LONG_BOUND,
            warn_after: WARN_AFTER,
        }
    }

    /// With a reading's bound raised to `bound`, before and after the first, for an
    /// instrument whose reading takes long.
    #[must_use]
    pub fn waiting(mut self, bound: Duration) -> Watch {
        self.no_sample_bound = self.no_sample_bound.max(bound);
        self.long_bound = self.long_bound.max(bound);
        self
    }

    /// With these bounds, for a test: no reading, silence; no reading, junk; a reading
    /// came; warning.
    #[cfg(test)]
    pub fn with_bounds(time_limited: bool, bounds: [Duration; 4]) -> Watch {
        let [no_sample_bound, no_usable_bound, long_bound, warn_after] = bounds;
        Watch {
            no_sample_bound,
            no_usable_bound,
            long_bound,
            warn_after,
            ..Watch::new(time_limited)
        }
    }

    /// A reading came. The line to say, if it had been quiet long enough for a warning.
    pub fn sample(&mut self) -> Option<&'static str> {
        self.sampled = true;
        self.silent = Duration::ZERO;
        self.unusable = Duration::ZERO;
        std::mem::take(&mut self.warned).then_some("The instrument answers again.")
    }

    /// An answer came, and it was no reading, after `took` of a wait for `asked`.
    pub fn no_reading(&mut self, took: Duration, asked: Duration) -> Verdict {
        self.unusable += took.min(asked * 2).max(Duration::from_millis(10));
        self.silent = Duration::ZERO;
        let (bound, what) = if self.sampled {
            (self.long_bound, "No usable reading")
        } else {
            (self.no_usable_bound, "No usable answer from the instrument")
        };
        self.judge(self.unusable, bound, what)
    }

    /// A read found nothing after `took`, asking for `asked`. Take `took` from ahead
    /// of the read.
    pub fn nothing(&mut self, took: Duration, asked: Duration) -> Verdict {
        self.silent += took.min(asked * 2);
        let bound = if self.sampled { self.long_bound } else { self.no_sample_bound };
        self.judge(self.silent + self.unusable, bound, "No answer from the instrument")
    }

    /// The warning says the time that set it off and not the time gone, which a stall
    /// in the last read would make any number.
    fn judge(&mut self, gone: Duration, bound: Duration, what: &str) -> Verdict {
        let seconds = |d: Duration| d.as_secs_f32().round();
        if gone >= bound && !self.time_limited {
            return Verdict::Fail(format!("{what} in {} s.", seconds(bound)));
        }
        if gone >= self.warn_after && !self.warned {
            self.warned = true;
            return Verdict::Warn(format!(
                "{what} for {} s, still asking.",
                seconds(self.warn_after)
            ));
        }
        Verdict::Go
    }

    /// The warning at the end of a time limit, if nothing came in the capture.
    pub fn at_time_limit(&self) -> Option<&'static str> {
        (!self.sampled).then_some("The time limit ran out with nothing captured.")
    }
}

/// A reading, as `sr_atod_ascii` takes one, less what it makes up: what isn't a number
/// in the forms SCPI has, an empty answer, which it reads as 0, hexadecimal, `nan` and
/// `inf`. Leading white space is fine, as in `strtod`, and trailing white space isn't.
pub fn number(text: &str) -> Option<f64> {
    let text = text.trim_start();
    if !text.starts_with(|c: char| c.is_ascii_digit() || matches!(c, '+' | '-' | '.')) {
        return None;
    }
    text.parse::<f64>().ok().filter(|v| v.is_finite())
}

/// A status register, as `sr_atoi` takes one, less its 0 of an empty answer. C goes on
/// with an uninitialised register when this fails (`api.c:526`, and each `sr_atoi`
/// after it), and this is an error.
pub fn status_word(text: &str) -> Result<i32> {
    text.trim_start()
        .parse()
        .map_err(|_| Error::device(format!("SCPI: not a status register: '{text}'")))
}

/// An on or off. `parse_strict_bool` goes by how a word starts, so `-113,"Undefined"`
/// is no answer to it but `Timeout` is on and `No such query` is off. Here the whole
/// word counts, in the forms it names, as an output's state rests on it.
pub fn state(text: &str) -> Result<bool> {
    match text.trim().to_ascii_lowercase().as_str() {
        "1" | "y" | "yes" | "t" | "true" | "on" => Ok(true),
        "0" | "n" | "no" | "f" | "false" | "off" => Ok(false),
        _ => Err(Error::device(format!("SCPI: not a boolean: '{text}'"))),
    }
}

/// What the regulation query says, as `api.c:508-590` has it.
///
/// # Errors
///
/// An answer that is none of them, with C's log line, and a status that is no number.
pub fn regulation(dialect: Dialect, answer: &str) -> Result<String> {
    let word = match dialect {
        Dialect::Philips => match answer {
            "VOLT" => "CV",
            "CURR" => "CC",
            other => other,
        },
        Dialect::HpComp => {
            let reg = status_word(answer)?;
            match reg {
                r if r & 1 != 0 => "CV",
                r if r & (1 << 1) != 0 => "CC",
                r if r & (1 << 2) != 0 => "UR",
                r if r & (1 << 9) != 0 => "CC-",
                _ => "",
            }
        }
        Dialect::Hp66xxb => {
            let reg = status_word(answer)?;
            match reg {
                r if r & (1 << 8) != 0 => "CV",
                r if r & (1 << 10) != 0 => "CC",
                r if r & (1 << 11) != 0 => "CC-",
                _ => "UR",
            }
        }
        Dialect::Hmp => {
            let reg = status_word(answer)?;
            match reg {
                r if r & 1 != 0 => "CC",
                r if r & (1 << 1) != 0 => "CV",
                _ => "UR",
            }
        }
        Dialect::KeysightE36300a => match status_word(answer)? & 3 {
            1 => "CC",
            2 => "CV",
            // Both low bits: a hardware failure.
            3 => "",
            _ => "UR",
        },
        Dialect::Unknown => answer,
    };
    if matches!(word, "CV" | "CC" | "CC-" | "UR" | "") {
        Ok(word.to_owned())
    } else {
        log::error!("Unknown response to SCPI_CMD_GET_OUTPUT_REGULATION: {word}");
        Err(Error::device(format!("Unknown regulation answer: '{word}'")))
    }
}

/// What came of asking for a reading.
#[derive(Clone, Debug, PartialEq)]
pub enum Reading {
    /// A number.
    Value(f64),
    /// Nothing, in this read's time.
    Silent,
    /// An answer that is no reading.
    Junk(String),
}

/// A bounded read's error: what a transport says of a device that sent bytes that
/// make nothing, not of one gone. Whatever isn't this ends a capture.
fn is_unusable(e: &Error) -> bool {
    match e {
        Error::Device(text) => !text.contains("closed"),
        _ => false,
    }
}

/// The instrument's line.
pub struct Link {
    /// Open while the device is.
    pub scpi: Option<Scpi>,
    profile: &'static Profile,
    /// A question of [`Link::measure`]'s is out, on a line with no tags.
    owed: bool,
    /// For how many reads it has been out.
    waited: u32,
    /// How many it may be out: [`PATIENCE`], or more for a slow reading.
    patience: u32,
    /// What asks who the instrument is, and what the answer holds: to find the line's
    /// place again after an answer cut short.
    identify: (&'static str, String),
}

impl Link {
    /// Not open yet. `identify` is the query that names the instrument and what its
    /// answer holds.
    pub fn new(profile: &'static Profile, identify: (&'static str, String)) -> Link {
        Link { scpi: None, profile, owed: false, waited: 0, patience: PATIENCE, identify }
    }

    /// A capture's start: a question an earlier capture left out is given up, and the
    /// next send drains its answer, as the line is still marked stale; `patience`
    /// reads for one question from now on.
    pub fn begin_capture(&mut self, patience: u32) {
        self.owed = false;
        self.waited = 0;
        self.patience = patience.max(PATIENCE);
    }

    /// Finds the line's place again after an answer cut short, whose tail may still
    /// come and be taken for the next answer, for good: asks who the instrument is and
    /// reads until that answer comes, everything before it dropped. A drain on quiet
    /// can't do this, as a tail may come later than any quiet is waited for.
    ///
    /// # Errors
    ///
    /// A failed write or read that ends a capture.
    fn resync(&mut self) -> Result<()> {
        let (query, name) = self.identify.clone();
        let patience = self.patience;
        log::warn!("An answer came cut short: asking {query} to find where the line is.");
        let scpi = self.scpi()?;
        scpi.send(query)?;
        for _ in 0..patience {
            match scpi.late_answer() {
                Ok(line) if line.contains(&name) => return Ok(()),
                Ok(line) => log::debug!("Dropped '{line}' ahead of the answer to {query}."),
                Err(Error::Timeout) => {}
                Err(e) if is_unusable(&e) => {}
                Err(e) => return Err(e),
            }
        }
        log::warn!("No answer to {query}: the line may still be out of step.");
        Ok(())
    }

    fn scpi(&mut self) -> Result<&mut Scpi> {
        self.scpi.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// A question of `measure`'s is still out.
    pub fn owed(&self) -> bool {
        self.owed
    }

    /// `sr_scpi_cmd`: a setting, after selecting `channel`. A command the table lacks
    /// is nothing, and sends nothing. What was owed is given up: the send drains it.
    ///
    /// # Errors
    ///
    /// A write that failed. Whether it reached the instrument isn't known then.
    pub fn cmd(&mut self, channel: Option<&str>, command: Cmd, args: &[CmdArg<'_>]) -> Result<()> {
        self.owed = false;
        let table = self.profile.commands;
        let selection = channel.map(|name| (Cmd::SelectChannel, name));
        self.scpi()?.cmd(table, selection, &command, args)
    }

    /// `sr_scpi_cmd_resp`: the answer as text, waiting for a late one where nothing
    /// else can be taken for it. See [`GET_PATIENCE`].
    ///
    /// # Errors
    ///
    /// No answer, a failed write, or a command the table lacks.
    pub fn query(
        &mut self,
        channel: Option<&str>,
        command: Cmd,
        args: &[CmdArg<'_>],
    ) -> Result<String> {
        self.owed = false;
        let table = self.profile.commands;
        let selection = channel.map(|name| (Cmd::SelectChannel, name));
        let scpi = self.scpi()?;
        let mut result = scpi.cmd_resp(table, selection, &command, args);
        let mut waited = 1;
        while matches!(result, Err(Error::Timeout)) && scpi.owes_answer() && waited < GET_PATIENCE {
            result = scpi.late_answer();
            waited += 1;
        }
        // Part of an answer is no value, and its tail would be the next one's.
        let cut = scpi.out_of_step() && matches!(result, Ok(_) | Err(Error::Timeout));
        if cut {
            self.resync()?;
            return Err(Error::device("SCPI: the answer came cut short"));
        }
        result
    }

    /// A query of the driver's own, outside the table: the answer as text.
    ///
    /// # Errors
    ///
    /// No answer, or a failed write.
    pub fn ask(&mut self, query: &str) -> Result<String> {
        self.scpi()?.get_string(query)
    }

    /// A reading of `channel`, as one pass of `scpi_pps_receive_data`. On a line with
    /// no tags a question stays out through timeouts: the read goes on until
    /// [`PATIENCE`] has passed, and the next ask comes after the line was drained.
    ///
    /// # Errors
    ///
    /// What ends a capture: a write that failed, or a device gone.
    pub fn measure(&mut self, channel: Option<&str>, command: Cmd) -> Result<Reading> {
        let table = self.profile.commands;
        let selection = channel.map(|name| (Cmd::SelectChannel, name));
        let owed = self.owed;
        let scpi = self.scpi()?;
        let result =
            if owed { scpi.late_answer() } else { scpi.cmd_resp(table, selection, &command, &[]) };
        let tagless = scpi.owes_answer();
        let out_of_step = scpi.out_of_step();
        match result {
            Ok(text) if out_of_step => {
                self.owed = false;
                self.waited = 0;
                self.resync()?;
                Ok(Reading::Junk(format!("'{text}', cut short")))
            }
            Ok(text) => {
                self.owed = false;
                self.waited = 0;
                Ok(number(&text).map_or(Reading::Junk(text), Reading::Value))
            }
            Err(Error::Timeout) if out_of_step => {
                self.owed = false;
                self.waited = 0;
                self.resync()?;
                Ok(Reading::Silent)
            }
            Err(Error::Timeout) => {
                self.waited = if owed { self.waited + 1 } else { 1 };
                self.owed = tagless && self.waited < self.patience;
                Ok(Reading::Silent)
            }
            Err(e) if is_unusable(&e) => {
                self.owed = false;
                Ok(Reading::Junk(e.to_string()))
            }
            Err(e) => {
                self.owed = false;
                Err(e)
            }
        }
    }

    /// `init_acquisition`, whose result C drops, as here.
    pub fn init_acquisition(&mut self) {
        let lines: &[&str] = match self.profile.init {
            super::profiles::Init::None => &[],
            super::profiles::Init::Hp6630a => &["UNMASK 607"],
            super::profiles::Init::Hp6630b => {
                &["STAT:OPER:PTR 3328;NTR 3328;ENAB 3328", "STAT:QUES:PTR 1043;NTR 1043;ENAB 1043"]
            }
        };
        for line in lines {
            let sent = self.scpi().and_then(|s| s.send(line));
            if let Err(e) = sent {
                log::debug!("{line}: {e}.");
                return;
            }
        }
    }

    /// `update_status`, whose result C drops: what it found before a failure still
    /// counts, and a failure only logs. The packets to send come back.
    pub fn update_status(&mut self) -> Vec<Packet> {
        let mut packets = Vec::new();
        let status = self.profile.status;
        let Ok(scpi) = self.scpi() else { return packets };
        let result = match status {
            super::profiles::Status::None => Ok(()),
            super::profiles::Status::Ea => ea_status(scpi, &mut packets),
            super::profiles::Status::Hp6630a => hp_6630a_status(scpi, &mut packets),
            super::profiles::Status::Hp6630b => hp_6630b_status(scpi, &mut packets),
        };
        if let Err(e) = result {
            log::debug!("Status update: {e}.");
        }
        packets
    }
}

fn meta_if(packets: &mut Vec<Packet>, key: Key, on: bool) {
    if on {
        packets.push(Packet::Meta(vec![(key, Value::Bool(true))]));
    }
}

fn int(scpi: &mut Scpi, query: &str) -> Result<i32> {
    let value = scpi.get_int(query)?;
    i32::try_from(value).map_err(|_| Error::device(format!("SCPI: out of range: {value}")))
}

#[allow(clippy::fn_params_excessive_bools)]
/// Regulation from four state bits: `CV`, `CC`, `CC-`, `UR`, nothing, and `None` where
/// more than one is set.
fn regulation_of(cv: bool, cc_pos: bool, cc_neg: bool, unreg: bool) -> Option<&'static str> {
    match (cv, cc_pos, cc_neg, unreg) {
        (true, false, false, false) => Some("CV"),
        (false, true, false, false) => Some("CC"),
        (false, false, true, false) => Some("CC-"),
        (false, false, false, true) => Some("UR"),
        (false, false, false, false) => Some(""),
        _ => None,
    }
}

/// `ea_ps_20xx_update_status`.
fn ea_status(scpi: &mut Scpi, packets: &mut Vec<Packet>) -> Result<()> {
    let ques = int(scpi, "STAT:QUES?")?;
    meta_if(packets, Key::OverVoltageProtectionActive, ques & 1 != 0);
    meta_if(packets, Key::OverCurrentProtectionActive, ques & (1 << 1) != 0);
    meta_if(packets, Key::OverTemperatureProtectionActive, ques & (1 << 3) != 0);
    let oper = int(scpi, "STAT:OPER?")?;
    let (cv, cc) = (oper & (1 << 8) != 0, oper & (1 << 9) != 0);
    if cv || cc {
        match regulation_of(cv, cc, false, false) {
            Some(word) => packets.push(Packet::Meta(vec![(Key::Regulation, Value::from(word))])),
            None => log::debug!("Undefined regulation for EA PS 20xx (CV={cv}, CC={cc})."),
        }
    }
    Ok(())
}

/// `hp_6630a_update_status`.
fn hp_6630a_status(scpi: &mut Scpi, packets: &mut Vec<Packet>) -> Result<()> {
    let fault = int(scpi, "FAULT?")?;
    meta_if(packets, Key::OverVoltageProtectionActive, fault & (1 << 3) != 0);
    meta_if(packets, Key::OverCurrentProtectionActive, fault & (1 << 6) != 0);
    meta_if(packets, Key::OverTemperatureProtectionActive, fault & (1 << 4) != 0);
    let cv = fault & 1 != 0;
    let cc_pos = fault & (1 << 1) != 0;
    let unreg = fault & (1 << 2) != 0;
    let cc_neg = fault & (1 << 9) != 0;
    if cv || cc_pos || unreg || cc_neg {
        match regulation_of(cv, cc_pos, cc_neg, unreg) {
            Some(word) => packets.push(Packet::Meta(vec![(Key::Regulation, Value::from(word))])),
            None => log::debug!(
                "Undefined regulation for HP 66xxA (CV={cv}, CC+={cc_pos}, CC-={cc_neg}, \
                 UR={unreg})."
            ),
        }
    }
    Ok(())
}

/// `hp_6630b_update_status`. No serial poll: oxrok has no GPIB.
fn hp_6630b_status(scpi: &mut Scpi, packets: &mut Vec<Packet>) -> Result<()> {
    let stb = int(scpi, "*STB?")?;
    let (mut unreg, mut changed) = (false, false);
    // The questionable summary bit.
    if stb & (1 << 3) != 0 {
        // The event register first, to clear it.
        let even = int(scpi, "STAT:QUES:EVEN?")?;
        let cond = int(scpi, "STAT:QUES:COND?")?;
        let mut meta = |bit: i32, key: Key| {
            if even & (1 << bit) != 0 {
                packets.push(Packet::Meta(vec![(key, Value::Bool(cond & (1 << bit) != 0))]));
            }
        };
        meta(0, Key::OverVoltageProtectionActive);
        meta(1, Key::OverCurrentProtectionActive);
        meta(4, Key::OverTemperatureProtectionActive);
        unreg = cond & (1 << 10) != 0;
        changed |= even & (1 << 10) != 0;
        // One of those may have turned the output off. Sent even if it didn't.
        let text = scpi.get_string("OUTP:STAT?")?;
        packets.push(Packet::Meta(vec![(Key::Enabled, Value::Bool(state(&text)?))]));
    }
    let (mut cv, mut cc_pos, mut cc_neg) = (false, false, false);
    // The operation summary bit.
    if stb & (1 << 7) != 0 {
        let even = int(scpi, "STAT:OPER:EVEN?")?;
        let cond = int(scpi, "STAT:OPER:COND?")?;
        cv = cond & (1 << 8) != 0;
        cc_pos = cond & (1 << 10) != 0;
        cc_neg = cond & (1 << 11) != 0;
        changed |= even & (1 << 8 | 1 << 10 | 1 << 11) != 0;
    }
    if changed {
        match regulation_of(cv, cc_pos, cc_neg, unreg) {
            // Nothing set happens with an OCP active.
            Some(word) => packets.push(Packet::Meta(vec![(Key::Regulation, Value::from(word))])),
            // CV and CC+ at once, which happens from time to time.
            None => log::debug!(
                "Undefined regulation for HP 66xxB (CV={cv}, CC+={cc_pos}, CC-={cc_neg}, \
                 UR={unreg})."
            ),
        }
    }
    Ok(())
}

/// `hpib_get_revision`: `A01 B02` anywhere makes `A01.B02` of the answer's fourth
/// byte, wherever the match was, as in C. What fails is no version.
pub fn hpib_revision(scpi: &mut Scpi) -> Option<String> {
    let mut response = scpi.get_string("ROM?").ok()?.into_bytes();
    let pattern = |w: &[u8]| {
        w.len() == 7
            && w[0].is_ascii_uppercase()
            && w[1].is_ascii_digit()
            && w[2].is_ascii_digit()
            && w[3] == b' '
            && w[4].is_ascii_uppercase()
            && w[5].is_ascii_digit()
            && w[6].is_ascii_digit()
    };
    if !response.windows(7).any(pattern) {
        return None;
    }
    response[3] = b'.';
    Some(String::from_utf8_lossy(&response).into_owned())
}

#[cfg(test)]
#[allow(clippy::float_cmp, clippy::match_wildcard_for_single_variants)]
mod tests {
    use super::*;

    const S: Duration = Duration::from_secs(1);

    #[test]
    fn numbers_and_what_is_none() {
        for (text, want) in [
            ("12.5", 12.5),
            (" 12.5", 12.5),
            ("+1.250000E+01", 12.5),
            ("-0.25", -0.25),
            (".5", 0.5),
            ("5.", 5.0),
            ("9.9E+37", 9.9e37),
        ] {
            assert_eq!(number(text), Some(want), "{text}");
        }
        for text in ["", " ", "0x10", "nan", "inf", "+inf", "-nan", "12.5 ", "12.5V", "1,5", "-"] {
            assert_eq!(number(text), None, "{text:?}");
        }
    }

    #[test]
    fn status_words_and_what_is_none() {
        assert_eq!(status_word(" 256").unwrap(), 256);
        assert_eq!(status_word("+3").unwrap(), 3);
        for text in ["", "1.0", "x", "256 ", "99999999999"] {
            assert!(status_word(text).is_err(), "{text:?}");
        }
    }

    #[test]
    fn states_and_what_is_none() {
        for text in ["1", "ON", "on", "YES", "Y", "true", "T"] {
            assert!(state(text).unwrap(), "{text}");
        }
        for text in ["0", "OFF", "no", "N", "false", "F"] {
            assert!(!state(text).unwrap(), "{text}");
        }
        for text in ["", "Timeout", "No such query", "2", "ONE", "offline"] {
            assert!(state(text).is_err(), "{text:?}");
        }
    }

    /// A reading's frame, each single-bit flip and each byte lost: how many give another
    /// number with nothing to say so. That is what a format with no checksum can't catch.
    #[test]
    fn what_damage_a_reading_survives() {
        let frame = b"12.0000";
        let mut flips = (0, 0);
        for at in 0..frame.len() {
            for bit in 0..8 {
                let mut damaged = frame.to_vec();
                damaged[at] ^= 1 << bit;
                flips.0 += 1;
                let parsed = std::str::from_utf8(&damaged).ok().and_then(number);
                if parsed.is_some_and(|v| v != 12.0) {
                    flips.1 += 1;
                }
            }
        }
        let mut lost = (0, 0);
        for at in 0..frame.len() {
            let mut damaged = frame.to_vec();
            damaged.remove(at);
            lost.0 += 1;
            let parsed = std::str::from_utf8(&damaged).ok().and_then(number);
            if parsed.is_some_and(|v| v != 12.0) {
                lost.1 += 1;
            }
        }
        // Of 56 flips, 23 still read as a number and another one: a digit into a digit,
        // or a sign or point that moved. Of 7 bytes lost, 3 do: the 1, the 2 and the
        // point. Trailing zeros lost change nothing.
        assert_eq!((flips, lost), ((56, 23), (7, 3)));
    }

    #[test]
    fn regulation_by_dialect() {
        let r = |d, a: &str| regulation(d, a).unwrap();
        for word in ["CV", "CC", "CC-", "UR", ""] {
            assert_eq!(r(Dialect::Unknown, word), word);
        }
        assert_eq!(r(Dialect::Philips, "VOLT"), "CV");
        assert_eq!(r(Dialect::Philips, "CURR"), "CC");
        assert!(regulation(Dialect::Philips, "XX").is_err());
        assert!(regulation(Dialect::Unknown, "XX").is_err());
        assert_eq!(r(Dialect::HpComp, "1"), "CV");
        assert_eq!(r(Dialect::HpComp, "3"), "CV");
        assert_eq!(r(Dialect::HpComp, "2"), "CC");
        assert_eq!(r(Dialect::HpComp, "4"), "UR");
        assert_eq!(r(Dialect::HpComp, "512"), "CC-");
        assert_eq!(r(Dialect::HpComp, "8"), "");
        assert_eq!(r(Dialect::Hp66xxb, "256"), "CV");
        assert_eq!(r(Dialect::Hp66xxb, "1024"), "CC");
        assert_eq!(r(Dialect::Hp66xxb, "2048"), "CC-");
        assert_eq!(r(Dialect::Hp66xxb, "0"), "UR");
        assert_eq!(r(Dialect::Hmp, "1"), "CC");
        assert_eq!(r(Dialect::Hmp, "2"), "CV");
        assert_eq!(r(Dialect::Hmp, "4"), "UR");
        assert_eq!(r(Dialect::KeysightE36300a, "1"), "CC");
        assert_eq!(r(Dialect::KeysightE36300a, "2"), "CV");
        assert_eq!(r(Dialect::KeysightE36300a, "3"), "");
        assert_eq!(r(Dialect::KeysightE36300a, "4"), "UR");
        for d in [Dialect::HpComp, Dialect::Hp66xxb, Dialect::Hmp, Dialect::KeysightE36300a] {
            assert!(regulation(d, "").is_err(), "{d:?}");
            assert!(regulation(d, "x").is_err(), "{d:?}");
        }
    }

    #[test]
    fn four_bits_of_regulation() {
        assert_eq!(regulation_of(true, false, false, false), Some("CV"));
        assert_eq!(regulation_of(false, true, false, false), Some("CC"));
        assert_eq!(regulation_of(false, false, true, false), Some("CC-"));
        assert_eq!(regulation_of(false, false, false, true), Some("UR"));
        assert_eq!(regulation_of(false, false, false, false), Some(""));
        assert_eq!(regulation_of(true, true, false, false), None);
        assert_eq!(regulation_of(false, true, false, true), None);
    }

    #[test]
    fn silence_with_no_reading_ends_at_its_bound() {
        let mut w = Watch::new(false);
        let mut waits = 0u32;
        let verdict = loop {
            waits += 1;
            assert!(waits < 1000);
            match w.nothing(S, S) {
                Verdict::Go => {}
                Verdict::Warn(text) => {
                    assert_eq!(text, "No answer from the instrument for 3 s, still asking.");
                }
                other => break other,
            }
        };
        assert_eq!(waits, 180);
        assert_eq!(verdict, Verdict::Fail("No answer from the instrument in 180 s.".to_owned()));
    }

    #[test]
    fn a_warning_comes_once() {
        let mut w = Watch::new(false);
        let verdicts: Vec<Verdict> = (0..30).map(|_| w.nothing(S, S)).collect();
        let warnings = verdicts.iter().filter(|v| matches!(v, Verdict::Warn(_))).count();
        assert_eq!(warnings, 1);
    }

    /// Spans many bounds' worth of time in reads: a reading resets it, or this fails.
    #[test]
    fn a_reading_keeps_a_capture_alive_for_many_bounds() {
        let mut w = Watch::new(false);
        w.sample();
        for _ in 0..100 {
            for _ in 0..500 {
                assert!(!matches!(w.nothing(S, S), Verdict::Fail(_)));
            }
            w.sample();
        }
    }

    #[test]
    fn after_a_reading_it_warns_goes_on_and_says_when_it_is_back() {
        let mut w = Watch::new(false);
        assert_eq!(w.sample(), None);
        let mut waits = 0u32;
        let mut warned = false;
        let verdict = loop {
            waits += 1;
            assert!(waits < 1000);
            match w.nothing(S, S) {
                Verdict::Go => {}
                Verdict::Warn(_) => warned = true,
                other => break other,
            }
        };
        assert!(warned);
        assert_eq!(waits, 600);
        assert_eq!(verdict, Verdict::Fail("No answer from the instrument in 600 s.".to_owned()));
        let mut w = Watch::new(false);
        w.sample();
        for _ in 0..10 {
            w.nothing(S, S);
        }
        assert_eq!(w.sample(), Some("The instrument answers again."));
        assert_eq!(w.sample(), None);
    }

    #[test]
    fn under_a_time_limit_it_never_fails() {
        let mut w = Watch::new(true);
        for _ in 0..2000 {
            assert!(!matches!(w.nothing(S, S), Verdict::Fail(_)));
        }
        assert_eq!(w.at_time_limit(), Some("The time limit ran out with nothing captured."));
        w.sample();
        assert_eq!(w.at_time_limit(), None);
    }

    #[test]
    fn answers_that_are_no_reading_fail_sooner_than_silence() {
        let mut w = Watch::new(false);
        let tick = Duration::from_millis(100);
        let mut waits = 0u32;
        let verdict = loop {
            waits += 1;
            assert!(waits < 1000);
            match w.no_reading(tick, S) {
                Verdict::Go | Verdict::Warn(_) => {}
                other => break other,
            }
        };
        assert_eq!(waits, 300);
        assert_eq!(
            verdict,
            Verdict::Fail("No usable answer from the instrument in 30 s.".to_owned())
        );
    }

    #[test]
    fn answers_that_are_no_reading_are_no_silence() {
        let mut w = Watch::with_bounds(false, [S * 2, S * 100, S * 100, S * 100]);
        assert_eq!(w.nothing(S, S), Verdict::Go);
        for _ in 0..10 {
            assert_eq!(w.no_reading(S, S), Verdict::Go);
        }
        assert_eq!(w.silent, Duration::ZERO);
    }

    /// A stall in a read counts for twice what the read asked for, at most, so one
    /// stall of an hour is no silence, and stalls without end still get there.
    #[test]
    fn a_stall_counts_for_little_and_stalls_without_end_get_there() {
        let hour = Duration::from_secs(3600);
        let mut w = Watch::new(false);
        assert_eq!(w.nothing(hour, S), Verdict::Go);
        assert_eq!(w.silent, S * 2);
        let mut waits = 0;
        while !matches!(w.nothing(hour, S), Verdict::Fail(_)) {
            waits += 1;
            assert!(waits < 200);
        }
        assert_eq!(waits, 88);
    }
}
