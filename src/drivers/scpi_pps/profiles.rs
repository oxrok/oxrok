// Counterpart in sigrok:
//   libsigrok/src/hardware/scpi-pps/profiles.c,
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2015 Google, Inc.,
//   Copyright (C) 2017,2019 Frank Stettner <frank-stettner@gmx.net>,
//   with commits by Uwe Hermann, Alexandru Gagniuc, Marc Schink,
//   Gerhard Sittig, Aurelien Jacobs, Martin Ling, Daniel Elstner,
//   Florian Schmidt, Kota Ando, Lars-Peter Clausen, MGWGIT, Martin Atkins,
//   Mike Walters, Mike Williams, Nicolas Labriet, Petteri Aimonen,
//   Sahaj Sarup, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `profiles.c`: what each supply has, and how the ones that say so at the scan are
//! asked. The tables below the types are generated from it, in its order, so that a
//! difference from C is one of the few things here by hand: the N5700's
//! `:CURR:PROT:STAT ON` and `OFF`, which `profiles.c:82-83` writes with a question
//! mark, and the probes, whose differences their functions say.

#![allow(clippy::unreadable_literal)]

use super::protocol::{Cmd, number};
use crate::core::{Access, Error, Key, OptionDesc, Result};
use crate::scpi::{HwInfo, Scpi};

/// How a status answer reads: `enum pps_scpi_dialect`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dialect {
    /// The answers are what they say.
    Unknown,
    /// An HP 66xx in COMP language: `STS?` is a status register.
    HpComp,
    /// An HP 66xxB: `STAT:OPER:COND?` and `STAT:QUES:COND?`.
    Hp66xxb,
    /// A PM2800, whose regulation is `VOLT` or `CURR`.
    Philips,
    /// An HMP, by the condition register of an output.
    Hmp,
    /// A Keysight E36300A, by the condition register of an output.
    KeysightE36300a,
}

/// `struct channel_spec`: min, max, programming resolution, spec digits, encoding
/// digits for each.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChannelSpec {
    /// The output's name, which also selects it.
    pub name: &'static str,
    /// Volts.
    pub voltage: [f64; 5],
    /// Amperes.
    pub current: [f64; 5],
    /// Watts.
    pub power: [f64; 5],
    /// Hertz.
    pub frequency: [f64; 5],
    /// Over voltage protection, volts.
    pub ovp: [f64; 5],
    /// Over current protection, amperes.
    pub ocp: [f64; 5],
    /// Over current protection delay, seconds.
    pub ocp_delay: [f64; 5],
}

impl ChannelSpec {
    const EMPTY: ChannelSpec = ChannelSpec {
        name: "",
        voltage: [0.0; 5],
        current: [0.0; 5],
        power: [0.0; 5],
        frequency: [0.0; 5],
        ovp: [0.0; 5],
        ocp: [0.0; 5],
        ocp_delay: [0.0; 5],
    };
}

/// `struct channel_group_spec`, of which a group here has one output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GroupSpec {
    /// The group's name.
    pub name: &'static str,
    /// The output it has.
    pub channel: usize,
    /// AC, where all others are DC. Volts and amperes alone have the flag.
    pub ac: bool,
}

/// What a model's name must be, in place of `pps_profiles`' regular expressions.
#[derive(Clone, Copy)]
pub enum Match {
    /// `NAME`, anywhere.
    Contains(&'static str),
    /// `^NAME$`.
    Whole(&'static str),
    /// `^NAME`.
    Starts(&'static str),
    /// What a test holds against the C's expression.
    By(fn(&str) -> bool),
}

impl Match {
    /// Whether `model` is of this kind.
    pub fn matches(self, model: &str) -> bool {
        match self {
            Match::Contains(name) => model.contains(name),
            Match::Whole(name) => model == name,
            Match::Starts(name) => model.starts_with(name),
            Match::By(test) => test(model),
        }
    }
}

/// How many digits start `text`.
fn digits(text: &str) -> usize {
    text.bytes().take_while(u8::is_ascii_digit).count()
}

/// `620[0-9]{2}P-[0-9]{2,3}-[0-9]{1,3}`, anywhere.
pub fn is_chroma_62000(model: &str) -> bool {
    model.match_indices("620").any(|(at, _)| {
        let rest = &model[at + 3..];
        let Some(rest) = rest.get(2..).filter(|_| digits(rest) >= 2) else { return false };
        let Some(rest) = rest.strip_prefix("P-") else { return false };
        let n = digits(rest);
        (2..=3).contains(&n) && rest[n..].strip_prefix('-').is_some_and(|tail| digits(tail) >= 1)
    })
}

/// `PS 20(42|84)-`, anywhere.
pub fn is_ea_ps_20xx(model: &str) -> bool {
    model.contains("PS 2042-") || model.contains("PS 2084-")
}

/// `^(DP832|DP832A)$`.
pub fn is_dp832(model: &str) -> bool {
    model == "DP832" || model == "DP832A"
}

/// `^PM28[13][123]/[01234]{1,2}$`.
pub fn is_pm2800(model: &str) -> bool {
    let b = model.as_bytes();
    (b.len() == 8 || b.len() == 9)
        && model.starts_with("PM28")
        && matches!(b[4], b'1' | b'3')
        && matches!(b[5], b'1' | b'2' | b'3')
        && b[6] == b'/'
        && b[7..].iter().all(|c| (b'0'..=b'4').contains(c))
}

/// Where a profile's outputs come from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Probe {
    /// Its tables.
    Table,
    /// `chroma_62000p_probe_channels`.
    Chroma62000,
    /// `ea_ps_20xx_probe_channels`.
    EaPs20xx,
    /// `eez_psu_probe_channels`.
    Eez,
    /// `philips_pm2800_probe_channels`.
    Pm2800,
}

/// `init_acquisition`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Init {
    /// Nothing.
    None,
    /// `hp_6630a_init_acquisition`.
    Hp6630a,
    /// `hp_6630b_init_acquisition`.
    Hp6630b,
}

/// `update_status`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// Nothing.
    None,
    /// `ea_ps_20xx_update_status`.
    Ea,
    /// `hp_6630a_update_status`.
    Hp6630a,
    /// `hp_6630b_update_status`.
    Hp6630b,
}

/// `struct scpi_pps`.
pub struct Profile {
    /// The vendor, after `sr_vendor_alias`, without case.
    pub vendor: &'static str,
    /// The model.
    pub model: Match,
    /// How status answers read.
    pub dialect: Dialect,
    /// `devopts`.
    pub device_options: &'static [OptionDesc],
    /// `devopts_cg`.
    pub options: &'static [OptionDesc],
    /// The outputs, where `probe` is [`Probe::Table`].
    pub channels: &'static [ChannelSpec],
    /// The groups, likewise.
    pub groups: &'static [GroupSpec],
    /// The command table.
    pub commands: &'static [(Cmd, &'static str)],
    /// Where the outputs come from.
    pub probe: Probe,
    /// What a capture does first.
    pub init: Init,
    /// What a capture asks at the start of each round.
    pub status: Status,
}

/// The first `n` of a table, as a profile may list fewer than its model's table has.
const fn first<T>(table: &'static [T], n: usize) -> &'static [T] {
    table.split_at(n).0
}

/// The profile for what `*IDN?` or `ID?` said, and the vendor's short name. The first
/// that matches, as in C.
pub fn find(id: &HwInfo) -> Option<(&'static Profile, &str)> {
    let vendor = crate::scpi::vendor_alias(&id.manufacturer);
    PROFILES
        .iter()
        .find(|p| vendor.eq_ignore_ascii_case(p.vendor) && p.model.matches(&id.model))
        .map(|p| (p, vendor))
}

/// `chroma_62000p_probe_channels`, from a name such as `62012P-80-60`. Where `sscanf`
/// meets something else, as the expression allows anything ahead of its match, C goes
/// on with variables it never set; here that's no device.
fn chroma_62000(model: &str) -> Result<(Vec<ChannelSpec>, Vec<GroupSpec>)> {
    let bad = || Error::device(format!("Cannot read a rating in '{model}'."));
    let rest = model.strip_prefix("620").ok_or_else(bad)?;
    let (watts, rest) = rest.split_once("P-").ok_or_else(bad)?;
    let (volts, amps) = rest.split_once('-').ok_or_else(bad)?;
    let amps = &amps[..digits(amps)];
    let watts = watts.parse::<u32>().map_err(|_| bad())?.saturating_mul(100);
    let volts: u32 = volts.parse().map_err(|_| bad())?;
    let amps: u32 = amps.parse().map_err(|_| bad())?;
    log::debug!("Found device rated for {volts} V, {amps} A and {watts} W");
    let out_of_spec = |text: String| {
        log::error!("{text}");
        Error::device(text)
    };
    if volts > 600 {
        return Err(out_of_spec(format!("Probed max voltage of {volts} V is out of spec.")));
    }
    if amps > 120 {
        return Err(out_of_spec(format!("Probed max current of {amps} A is out of spec.")));
    }
    if watts > 5000 {
        return Err(out_of_spec(format!("Probed max power of {watts} W is out of spec.")));
    }
    let spec = ChannelSpec {
        name: "1",
        voltage: [0.0, f64::from(volts), 0.01, 3.0, 3.0],
        current: [0.0, f64::from(amps), 0.01, 4.0, 4.0],
        power: [0.0, f64::from(watts), 0.0, 0.0, 0.0],
        ..ChannelSpec::EMPTY
    };
    Ok((vec![spec], CHROMA_62000_CG.to_vec()))
}

/// One of an EA's nominal values, which come with a unit, as `84.00 V`. The first word
/// is the number. C means to cut the unit off, and with `response[ret] = 0`, `ret`
/// being 0, cuts all of it off (`profiles.c:319`, `:336`, `:353`), so that it has 0 of
/// every nominal value and none of its checks can fail, and a list of it can't show
/// that, as it crashes first (`api.c:814`). So what is no number stays what C gets, 0,
/// with a warning, rather than losing the device.
fn ea_nominal(scpi: &mut Scpi, query: &str, what: &str) -> Result<f64> {
    let response = scpi.get_string(query).inspect_err(|_| {
        log::error!("Failed to get nominal {what}.");
    })?;
    let value = response.split_whitespace().next().and_then(number).filter(|v| *v >= 0.0);
    Ok(value.unwrap_or_else(|| {
        log::warn!("Failed to convert nominal {what} from: '{response}', taking 0.");
        0.0
    }))
}

/// `ea_ps_20xx_probe_channels`.
fn ea_ps_20xx(scpi: &mut Scpi) -> Result<(Vec<ChannelSpec>, Vec<GroupSpec>)> {
    let volts = ea_nominal(scpi, ":SYST:NOM:VOLT?", "voltage")?;
    let amps = ea_nominal(scpi, ":SYST:NOM:CURR?", "current")?;
    let watts = ea_nominal(scpi, ":SYST:NOM:POW?", "power")?;
    let spec = ChannelSpec {
        name: "1",
        voltage: [0.0, volts, 0.01, 2.0, 2.0],
        current: [0.0, amps, 0.01, 2.0, 2.0],
        power: [0.0, watts, 0.0, 2.0, 2.0],
        ..ChannelSpec::EMPTY
    };
    Ok((vec![spec], EA_PS_20XX_CG.to_vec()))
}

/// `eez_psu_channel_names`.
const EEZ_NAMES: [&str; 6] = ["1", "2", "3", "4", "5", "6"];

/// `eez_psu_probe_channels`. C's warning at too many says "first 9 of 6" for 9, with its
/// two values the other way round (`profiles.c:566-567`), and an empty answer is a
/// limit of 0 to `sr_atod_ascii`.
fn eez(scpi: &mut Scpi) -> Result<(Vec<ChannelSpec>, Vec<GroupSpec>)> {
    let count = scpi.get_int(":SYST:CHAN:COUN?").inspect_err(|_| {
        log::error!("Failed to probe EEZ PSU channel count.");
    })?;
    if count < 0 {
        log::error!("Suspicious channel count {count}, ignoring.");
        return Err(Error::device("a negative channel count"));
    }
    let mut count = usize::try_from(count).unwrap_or(usize::MAX);
    if count > EEZ_NAMES.len() {
        log::warn!("Only using first {} of {count} EEZ PSU channels.", EEZ_NAMES.len());
        count = EEZ_NAMES.len();
    }
    let mut specs = Vec::with_capacity(count);
    let mut groups = Vec::with_capacity(count);
    for (i, name) in EEZ_NAMES.iter().copied().enumerate().take(count) {
        scpi.send(&format!(":INST:NSEL {name}")).inspect_err(|_| {
            log::error!("Failed to select {name} to retrieve its limits.");
        })?;
        let mut limit = |query: &str, what: &str| -> Result<f64> {
            let text = scpi.get_string(query);
            text.and_then(|t| {
                number(&t).ok_or_else(|| Error::device(format!("SCPI: not a number: '{t}'")))
            })
            .inspect_err(|_| {
                log::error!("Failed to read the {what} limit for {name}.");
            })
        };
        let amps = limit(":SYST:CHAN:INFO:CURR?", "current")?;
        let volts = limit(":SYST:CHAN:INFO:VOLT?", "voltage")?;
        let watts = limit(":SYST:CHAN:INFO:POW?", "power")?;
        groups.push(GroupSpec { name, channel: i, ac: false });
        specs.push(ChannelSpec {
            name,
            current: [0.0, amps, 0.01, 2.0, 2.0],
            voltage: [0.0, volts, 0.01, 2.0, 2.0],
            power: [0.0, watts, 0.01, 2.0, 2.0],
            ..ChannelSpec::EMPTY
        });
    }
    Ok((specs, groups))
}

/// A PM2800's module: `philips_pm2800_module_specs`, volts, amperes, watts.
type Module = ([f64; 5], [f64; 5], [f64; 5]);

const NO_MODULE: Module = ([0.0; 5], [0.0; 5], [0.0; 5]);
const MOD_30V_10A: Module =
    ([0.0, 30.0, 0.0075, 2.0, 4.0], [0.0, 10.0, 0.0025, 2.0, 4.0], [0.0, 60.0, 0.0, 0.0, 0.0]);
const MOD_60V_5A: Module =
    ([0.0, 60.0, 0.015, 2.0, 3.0], [0.0, 5.0, 0.00125, 2.0, 5.0], [0.0, 60.0, 0.0, 0.0, 0.0]);
const MOD_60V_10A: Module =
    ([0.0, 60.0, 0.015, 2.0, 3.0], [0.0, 10.0, 0.0025, 2.0, 5.0], [0.0, 120.0, 0.0, 0.0, 0.0]);
const MOD_8V_15A: Module =
    ([0.0, 8.0, 0.002, 3.0, 3.0], [-15.0, 15.0, 0.00375, 3.0, 5.0], [0.0, 120.0, 0.0, 0.0, 0.0]);
const MOD_60V_2A: Module =
    ([0.0, 60.0, 0.015, 2.0, 3.0], [-2.0, 2.0, 0.0005, 3.0, 4.0], [0.0, 120.0, 0.0, 0.0, 0.0]);
const MOD_120V_1A: Module =
    ([0.0, 120.0, 0.030, 2.0, 2.0], [-1.0, 1.0, 0.00025, 3.0, 5.0], [0.0, 120.0, 0.0, 0.0, 0.0]);

/// `philips_pm2800_matrix`: chassis, modules, set, and each module. Two lines of the
/// linear chassis say two modules and name one, so C's second output there has the
/// zeros of the table's entry 0, as has this.
const PM2800_MATRIX: [(u8, u8, u8, [Module; 3]); 18] = [
    (1, 1, 0, [MOD_30V_10A, NO_MODULE, NO_MODULE]),
    (1, 1, 1, [MOD_60V_5A, NO_MODULE, NO_MODULE]),
    (1, 2, 0, [MOD_30V_10A, MOD_30V_10A, NO_MODULE]),
    (1, 2, 1, [MOD_60V_5A, MOD_60V_5A, NO_MODULE]),
    (1, 2, 2, [MOD_30V_10A, MOD_60V_5A, NO_MODULE]),
    (1, 2, 3, [MOD_30V_10A, MOD_60V_10A, NO_MODULE]),
    (1, 2, 4, [MOD_60V_5A, MOD_60V_10A, NO_MODULE]),
    (1, 3, 0, [MOD_30V_10A, MOD_30V_10A, MOD_30V_10A]),
    (1, 3, 1, [MOD_60V_5A, MOD_60V_5A, MOD_60V_5A]),
    (1, 3, 2, [MOD_30V_10A, MOD_30V_10A, MOD_60V_5A]),
    (1, 3, 3, [MOD_30V_10A, MOD_60V_5A, MOD_60V_5A]),
    (3, 1, 0, [MOD_60V_2A, NO_MODULE, NO_MODULE]),
    (3, 1, 1, [MOD_120V_1A, NO_MODULE, NO_MODULE]),
    (3, 1, 2, [MOD_8V_15A, NO_MODULE, NO_MODULE]),
    (3, 2, 0, [MOD_60V_2A, NO_MODULE, NO_MODULE]),
    (3, 2, 1, [MOD_120V_1A, NO_MODULE, NO_MODULE]),
    (3, 2, 2, [MOD_60V_2A, MOD_120V_1A, NO_MODULE]),
    (3, 2, 3, [MOD_8V_15A, MOD_8V_15A, NO_MODULE]),
];

const PM2800_NAMES: [&str; 3] = ["1", "2", "3"];

/// `philips_pm2800_probe_channels`.
fn pm2800(model: &str) -> Result<(Vec<ChannelSpec>, Vec<GroupSpec>)> {
    let digit = |at: usize| model.as_bytes().get(at).map_or(0, |b| b.wrapping_sub(b'0'));
    let (chassis, modules, set) = (digit(4), digit(5), digit(7));
    let Some(row) = PM2800_MATRIX.iter().find(|r| r.0 == chassis && r.1 == modules && r.2 == set)
    else {
        log::debug!("Model {model} not found in matrix.");
        return Err(Error::device(format!("Model {model} not found in matrix.")));
    };
    let plural = if modules > 1 { "s" } else { "" };
    log::debug!("Found {modules} output channel{plural}:");
    let mut specs = Vec::new();
    let mut groups = Vec::new();
    for (i, name) in PM2800_NAMES.into_iter().enumerate().take(usize::from(modules)) {
        let (voltage, current, power) = row.3[i];
        specs.push(ChannelSpec { name, voltage, current, power, ..ChannelSpec::EMPTY });
        groups.push(GroupSpec { name, channel: i, ac: false });
    }
    Ok((specs, groups))
}

/// The outputs and groups of a device of `profile`.
///
/// # Errors
///
/// When a probe fails: no device.
pub fn outputs(
    profile: &Profile,
    id: &HwInfo,
    scpi: &mut Scpi,
) -> Result<(Vec<ChannelSpec>, Vec<GroupSpec>)> {
    match profile.probe {
        Probe::Table => Ok((profile.channels.to_vec(), profile.groups.to_vec())),
        Probe::Chroma62000 => chroma_62000(&id.model),
        Probe::EaPs20xx => ea_ps_20xx(scpi),
        Probe::Eez => eez(scpi),
        Probe::Pm2800 => pm2800(&id.model),
    }
}

const AGILENT_N5700A_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:COMM:RLST REM"),
    (Cmd::Local, "SYST:COMM:RLST LOC"),
    (Cmd::GetMeasVoltage, ":MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, "MEAS:CURR?"),
    (Cmd::GetVoltageTarget, ":SOUR:VOLT?"),
    (Cmd::SetVoltageTarget, ":SOUR:VOLT %.6f"),
    (Cmd::GetCurrentLimit, ":SOUR:CURR?"),
    (Cmd::SetCurrentLimit, ":SOUR:CURR %.6f"),
    (Cmd::GetOutputEnabled, ":OUTP:STAT?"),
    (Cmd::SetOutputEnable, ":OUTP ON"),
    (Cmd::SetOutputDisable, ":OUTP OFF"),
    (Cmd::GetOverVoltageProtectionThreshold, ":VOLT:PROT?"),
    (Cmd::SetOverVoltageProtectionThreshold, ":VOLT:PROT %.6f"),
    (Cmd::GetOverCurrentProtectionEnabled, ":CURR:PROT:STAT?"),
    (Cmd::SetOverCurrentProtectionEnable, ":CURR:PROT:STAT ON"),
    (Cmd::SetOverCurrentProtectionDisable, ":CURR:PROT:STAT OFF"),
    (Cmd::GetOverCurrentProtectionThreshold, ":SOUR:CURR?"),
    (Cmd::SetOverCurrentProtectionThreshold, ":SOUR:CURR %.6f"),
];

const BK_9130_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:REMOTE"),
    (Cmd::Local, "SYST:LOCAL"),
    (Cmd::SelectChannel, ":INST:NSEL %s"),
    (Cmd::GetMeasVoltage, ":MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, ":MEAS:CURR?"),
    (Cmd::GetMeasPower, ":MEAS:POWER?"),
    (Cmd::GetVoltageTarget, ":SOUR:VOLT?"),
    (Cmd::SetVoltageTarget, ":SOUR:VOLT %.6f"),
    (Cmd::GetCurrentLimit, ":SOUR:CURR?"),
    (Cmd::SetCurrentLimit, ":SOUR:CURR %.6f"),
    (Cmd::GetOutputEnabled, ":OUTP?"),
    (Cmd::SetOutputEnable, ":OUTP 1"),
    (Cmd::SetOutputDisable, ":OUTP 0"),
    (Cmd::GetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT?"),
    (Cmd::SetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT %.6f"),
];

const CHROMA_61604_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:REM"),
    (Cmd::Local, "SYST:LOC"),
    (Cmd::GetMeasVoltage, ":FETC:VOLT:ACDC?"),
    (Cmd::GetMeasFrequency, ":FETC:FREQ?"),
    (Cmd::GetMeasCurrent, ":FETC:CURR:AC?"),
    (Cmd::GetMeasPower, ":FETC:POW:AC?"),
    (Cmd::GetVoltageTarget, ":SOUR:VOLT:AC?"),
    (Cmd::SetVoltageTarget, ":SOUR:VOLT:AC %.1f"),
    (Cmd::GetFrequencyTarget, ":SOUR:FREQ?"),
    (Cmd::SetFrequencyTarget, ":SOUR:FREQ %.2f"),
    (Cmd::GetOutputEnabled, ":OUTP?"),
    (Cmd::SetOutputEnable, ":OUTP ON"),
    (Cmd::SetOutputDisable, ":OUTP OFF"),
    (Cmd::GetOverVoltageProtectionThreshold, ":SOUR:VOLT:LIM:AC?"),
    (Cmd::SetOverVoltageProtectionThreshold, ":SOUR:VOLT:LIM:AC %.1f"),
    (Cmd::GetOverCurrentProtectionThreshold, ":SOUR:CURR:LIM?"),
    (Cmd::SetOverCurrentProtectionThreshold, ":SOUR:CURR:LIM %.2f"),
];

const CHROMA_62000_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, ":CONF:REM ON"),
    (Cmd::Local, ":CONF:REM OFF"),
    (Cmd::Beeper, ":CONF:BEEP?"),
    (Cmd::BeeperEnable, ":CONF:BEEP ON"),
    (Cmd::BeeperDisable, ":CONF:BEEP OFF"),
    (Cmd::GetMeasVoltage, ":MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, ":MEAS:CURR?"),
    (Cmd::GetMeasPower, ":MEAS:POW?"),
    (Cmd::GetVoltageTarget, ":SOUR:VOLT?"),
    (Cmd::SetVoltageTarget, ":SOUR:VOLT %.2f"),
    (Cmd::GetCurrentLimit, ":SOUR:CURR?"),
    (Cmd::SetCurrentLimit, ":SOUR:CURR %.6f"),
    (Cmd::GetOutputEnabled, ":CONF:OUTP?"),
    (Cmd::SetOutputEnable, ":CONF:OUTP ON"),
    (Cmd::SetOutputDisable, ":CONF:OUTP OFF"),
    (Cmd::GetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT:HIGH?"),
    (Cmd::SetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT:HIGH %.6f"),
    (Cmd::GetOverCurrentProtectionThreshold, ":SOUR:CURR:PROT:HIGH?"),
    (Cmd::SetOverCurrentProtectionThreshold, ":SOUR:CURR:PROT:HIGH %.6f"),
];

const EA_PS_20XX_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:LOCK 1"),
    (Cmd::Local, "SYST:LOCK 0"),
    (Cmd::GetMeasVoltage, ":MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, ":MEAS:CURR?"),
    (Cmd::GetMeasPower, ":MEAS:POW?"),
    (Cmd::GetVoltageTarget, ":VOLT?"),
    (Cmd::SetVoltageTarget, ":VOLT %.2f"),
    (Cmd::GetCurrentLimit, ":CURR?"),
    (Cmd::SetCurrentLimit, ":CURR %.2f"),
    (Cmd::GetOutputEnabled, ":OUTP?"),
    (Cmd::SetOutputEnable, ":OUTP 1"),
    (Cmd::SetOutputDisable, ":OUTP 0"),
    (Cmd::SetOverVoltageProtectionThreshold, ":VOLT:PROT %.2f"),
    (Cmd::SetOverCurrentProtectionThreshold, ":CURR:PROT %.2f"),
];

const EEZ_PSU_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:REMOTE"),
    (Cmd::Local, "SYST:LOCAL"),
    (Cmd::SelectChannel, ":INST:NSEL %s"),
    (Cmd::GetMeasVoltage, ":MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, ":MEAS:CURR?"),
    (Cmd::GetMeasPower, ":MEAS:POWER?"),
    (Cmd::GetOutputRegulation, ":OUTP:MODE?"),
    (Cmd::GetVoltageTarget, ":SOUR:VOLT?"),
    (Cmd::SetVoltageTarget, ":SOUR:VOLT %.2f"),
    (Cmd::GetCurrentLimit, ":SOUR:CURR?"),
    (Cmd::SetCurrentLimit, ":SOUR:CURR %.6f"),
    (Cmd::GetOutputEnabled, ":OUTP?"),
    (Cmd::SetOutputEnable, ":OUTP ON"),
    (Cmd::SetOutputDisable, ":OUTP OFF"),
    (Cmd::GetOverCurrentProtectionThreshold, ":SOUR:CURR:PROT?"),
    (Cmd::GetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT?"),
];

const RIGOL_DP700_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:REMOTE"),
    (Cmd::Local, "SYST:LOCAL"),
    (Cmd::Beeper, "SYST:BEEP:STAT?"),
    (Cmd::BeeperEnable, "SYST:BEEP:STAT ON"),
    (Cmd::BeeperDisable, "SYST:BEEP:STAT OFF"),
    (Cmd::SelectChannel, ":INST:NSEL %s"),
    (Cmd::GetMeasVoltage, ":MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, ":MEAS:CURR?"),
    (Cmd::GetMeasPower, ":MEAS:POWE?"),
    (Cmd::GetVoltageTarget, ":SOUR:VOLT?"),
    (Cmd::SetVoltageTarget, ":SOUR:VOLT %.6f"),
    (Cmd::GetCurrentLimit, ":SOUR:CURR?"),
    (Cmd::SetCurrentLimit, ":SOUR:CURR %.6f"),
    (Cmd::GetOutputEnabled, ":OUTP?"),
    (Cmd::SetOutputEnable, ":OUTP ON"),
    (Cmd::SetOutputDisable, ":OUTP OFF"),
    (Cmd::GetOutputRegulation, ":OUTP:MODE?"),
    (Cmd::GetOverVoltageProtectionEnabled, ":OUTP:OVP?"),
    (Cmd::SetOverVoltageProtectionEnable, ":OUTP:OVP ON"),
    (Cmd::SetOverVoltageProtectionDisable, ":OUTP:OVP OFF"),
    (Cmd::GetOverVoltageProtectionActive, ":OUTP:OVP:QUES?"),
    (Cmd::GetOverVoltageProtectionThreshold, ":OUTP:OVP:VAL?"),
    (Cmd::SetOverVoltageProtectionThreshold, ":OUTP:OVP:VAL %.6f"),
    (Cmd::GetOverCurrentProtectionEnabled, ":OUTP:OCP?"),
    (Cmd::SetOverCurrentProtectionEnable, ":OUTP:OCP:STAT ON"),
    (Cmd::SetOverCurrentProtectionDisable, ":OUTP:OCP:STAT OFF"),
    (Cmd::GetOverCurrentProtectionActive, ":OUTP:OCP:QUES?"),
    (Cmd::GetOverCurrentProtectionThreshold, ":OUTP:OCP:VAL?"),
    (Cmd::SetOverCurrentProtectionThreshold, ":OUTP:OCP:VAL %.6f"),
];

const RIGOL_DP800_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:REMOTE"),
    (Cmd::Local, "SYST:LOCAL"),
    (Cmd::Beeper, "SYST:BEEP:STAT?"),
    (Cmd::BeeperEnable, "SYST:BEEP:STAT ON"),
    (Cmd::BeeperDisable, "SYST:BEEP:STAT OFF"),
    (Cmd::SelectChannel, ":INST:NSEL %s"),
    (Cmd::GetMeasVoltage, ":MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, ":MEAS:CURR?"),
    (Cmd::GetMeasPower, ":MEAS:POWE?"),
    (Cmd::GetVoltageTarget, ":SOUR:VOLT?"),
    (Cmd::SetVoltageTarget, ":SOUR:VOLT %.6f"),
    (Cmd::GetCurrentLimit, ":SOUR:CURR?"),
    (Cmd::SetCurrentLimit, ":SOUR:CURR %.6f"),
    (Cmd::GetOutputEnabled, ":OUTP?"),
    (Cmd::SetOutputEnable, ":OUTP ON"),
    (Cmd::SetOutputDisable, ":OUTP OFF"),
    (Cmd::GetOutputRegulation, ":OUTP:MODE?"),
    (Cmd::GetOverTemperatureProtection, ":SYST:OTP?"),
    (Cmd::SetOverTemperatureProtectionEnable, ":SYST:OTP ON"),
    (Cmd::SetOverTemperatureProtectionDisable, ":SYST:OTP OFF"),
    (Cmd::GetOverVoltageProtectionEnabled, ":OUTP:OVP?"),
    (Cmd::SetOverVoltageProtectionEnable, ":OUTP:OVP ON"),
    (Cmd::SetOverVoltageProtectionDisable, ":OUTP:OVP OFF"),
    (Cmd::GetOverVoltageProtectionActive, ":OUTP:OVP:QUES?"),
    (Cmd::GetOverVoltageProtectionThreshold, ":OUTP:OVP:VAL?"),
    (Cmd::SetOverVoltageProtectionThreshold, ":OUTP:OVP:VAL %.6f"),
    (Cmd::GetOverCurrentProtectionEnabled, ":OUTP:OCP?"),
    (Cmd::SetOverCurrentProtectionEnable, ":OUTP:OCP:STAT ON"),
    (Cmd::SetOverCurrentProtectionDisable, ":OUTP:OCP:STAT OFF"),
    (Cmd::GetOverCurrentProtectionActive, ":OUTP:OCP:QUES?"),
    (Cmd::GetOverCurrentProtectionThreshold, ":OUTP:OCP:VAL?"),
    (Cmd::SetOverCurrentProtectionThreshold, ":OUTP:OCP:VAL %.6f"),
];

const HP_6630A_CMD: &[(Cmd, &str)] = &[
    (Cmd::SetOutputEnable, "OUT 1"),
    (Cmd::SetOutputDisable, "OUT 0"),
    (Cmd::GetMeasVoltage, "VOUT?"),
    (Cmd::GetMeasCurrent, "IOUT?"),
    (Cmd::SetVoltageTarget, "VSET %.4f"),
    (Cmd::SetCurrentLimit, "ISET %.4f"),
    (Cmd::GetOverVoltageProtectionActive, "STS?"),
    (Cmd::SetOverVoltageProtectionThreshold, "OVSET %.4f"),
    (Cmd::SetOverCurrentProtectionEnable, "OCP 1"),
    (Cmd::SetOverCurrentProtectionDisable, "OCP 0"),
    (Cmd::GetOverCurrentProtectionActive, "STS?"),
    (Cmd::GetOverTemperatureProtectionActive, "STS?"),
    (Cmd::GetOutputRegulation, "STS?"),
];

const HP_6630B_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:REM"),
    (Cmd::Local, "SYST:LOC"),
    (Cmd::GetOutputEnabled, "OUTP:STAT?"),
    (Cmd::SetOutputEnable, "OUTP:STAT ON"),
    (Cmd::SetOutputDisable, "OUTP:STAT OFF"),
    (Cmd::GetMeasVoltage, ":MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, ":MEAS:CURR?"),
    (Cmd::GetVoltageTarget, ":SOUR:VOLT?"),
    (Cmd::SetVoltageTarget, ":SOUR:VOLT %.6f"),
    (Cmd::GetCurrentLimit, ":SOUR:CURR?"),
    (Cmd::SetCurrentLimit, ":SOUR:CURR %.6f"),
    (Cmd::GetOverCurrentProtectionEnabled, ":CURR:PROT:STAT?"),
    (Cmd::SetOverCurrentProtectionEnable, ":CURR:PROT:STAT 1"),
    (Cmd::SetOverCurrentProtectionDisable, ":CURR:PROT:STAT 0"),
    (Cmd::GetOverCurrentProtectionActive, "STAT:QUES:COND?"),
    (Cmd::GetOverVoltageProtectionActive, "STAT:QUES:COND?"),
    (Cmd::GetOverVoltageProtectionThreshold, ":VOLT:PROT?"),
    (Cmd::SetOverVoltageProtectionThreshold, ":VOLT:PROT %.6f"),
    (Cmd::GetOverTemperatureProtectionActive, "STAT:QUES:COND?"),
    (Cmd::GetOutputRegulation, "STAT:OPER:COND?"),
];

const KEYSIGHT_E36300A_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:REM"),
    (Cmd::Local, "SYST:LOC"),
    (Cmd::SelectChannel, ":INST:NSEL %s"),
    (Cmd::GetOutputEnabled, "OUTP:STAT?"),
    (Cmd::SetOutputEnable, "OUTP:STAT ON"),
    (Cmd::SetOutputDisable, "OUTP:STAT OFF"),
    (Cmd::GetMeasVoltage, ":MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, ":MEAS:CURR?"),
    (Cmd::GetVoltageTarget, ":SOUR:VOLT?"),
    (Cmd::SetVoltageTarget, ":SOUR:VOLT %.6f"),
    (Cmd::GetCurrentLimit, ":SOUR:CURR?"),
    (Cmd::SetCurrentLimit, ":SOUR:CURR %.6f"),
    (Cmd::GetOverCurrentProtectionEnabled, ":CURR:PROT:STAT?"),
    (Cmd::SetOverCurrentProtectionEnable, ":CURR:PROT:STAT 1"),
    (Cmd::SetOverCurrentProtectionDisable, ":CURR:PROT:STAT 0"),
    (Cmd::GetOverCurrentProtectionActive, ":CURR:PROT:TRIP?"),
    (Cmd::GetOverVoltageProtectionActive, ":VOLT:PROT:TRIP?"),
    (Cmd::GetOverVoltageProtectionThreshold, ":VOLT:PROT?"),
    (Cmd::SetOverVoltageProtectionThreshold, ":VOLT:PROT %.6f"),
    (Cmd::GetOverTemperatureProtectionActive, ":STAT:QUES:INST:ISUM%s:COND?"),
    (Cmd::GetOutputRegulation, ":STAT:QUES:INST:ISUM%s:COND?"),
];

const OWON_P4000_CMD: &[(Cmd, &str)] = &[
    (Cmd::GetMeasVoltage, "MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, "MEAS:CURR?"),
    (Cmd::GetMeasPower, "MEAS:POW?"),
    (Cmd::GetVoltageTarget, "VOLT?"),
    (Cmd::SetVoltageTarget, "VOLT %.6f"),
    (Cmd::GetCurrentLimit, "CURR?"),
    (Cmd::SetCurrentLimit, "CURR %.6f"),
    (Cmd::GetOutputEnabled, "OUTP?"),
    (Cmd::SetOutputEnable, "OUTP 1"),
    (Cmd::SetOutputDisable, "OUTP 0"),
    (Cmd::GetOverVoltageProtectionThreshold, "VOLT:LIM?"),
    (Cmd::SetOverVoltageProtectionThreshold, "VOLT:LIM %.6f"),
    (Cmd::GetOverCurrentProtectionThreshold, "CURR:LIM?"),
    (Cmd::SetOverCurrentProtectionThreshold, "CURR:LIM %.6f"),
];

const OWON_SPE6103_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:REM"),
    (Cmd::Local, "SYST:LOC"),
    (Cmd::GetMeasVoltage, "MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, "MEAS:CURR?"),
    (Cmd::GetMeasPower, "MEAS:POW?"),
    (Cmd::GetVoltageTarget, "VOLT?"),
    (Cmd::SetVoltageTarget, "VOLT %.6f"),
    (Cmd::GetCurrentLimit, "CURR?"),
    (Cmd::SetCurrentLimit, "CURR %.6f"),
    (Cmd::GetOutputEnabled, "OUTP?"),
    (Cmd::SetOutputEnable, "OUTP 1"),
    (Cmd::SetOutputDisable, "OUTP 0"),
    (Cmd::GetOverVoltageProtectionThreshold, "VOLT:LIM?"),
    (Cmd::SetOverVoltageProtectionThreshold, "VOLT:LIM %.6f"),
    (Cmd::GetOverCurrentProtectionThreshold, "CURR:LIM?"),
    (Cmd::SetOverCurrentProtectionThreshold, "CURR:LIM %.6f"),
];

const PHILIPS_PM2800_CMD: &[(Cmd, &str)] = &[
    (Cmd::SelectChannel, ":INST:NSEL %s"),
    (Cmd::GetMeasVoltage, ":MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, ":MEAS:CURR?"),
    (Cmd::GetVoltageTarget, ":SOUR:VOLT?"),
    (Cmd::SetVoltageTarget, ":SOUR:VOLT %.6f"),
    (Cmd::GetCurrentLimit, ":SOUR:CURR?"),
    (Cmd::SetCurrentLimit, ":SOUR:CURR %.6f"),
    (Cmd::GetOutputEnabled, ":OUTP?"),
    (Cmd::SetOutputEnable, ":OUTP ON"),
    (Cmd::SetOutputDisable, ":OUTP OFF"),
    (Cmd::GetOutputRegulation, ":SOUR:FUNC:MODE?"),
    (Cmd::GetOverVoltageProtectionActive, ":SOUR:VOLT:PROT:TRIP?"),
    (Cmd::GetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT:LEV?"),
    (Cmd::SetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT:LEV %.6f"),
    (Cmd::GetOverCurrentProtectionEnabled, ":SOUR:CURR:PROT:STAT?"),
    (Cmd::SetOverCurrentProtectionEnable, ":SOUR:CURR:PROT:STAT ON"),
    (Cmd::SetOverCurrentProtectionDisable, ":SOUR:CURR:PROT:STAT OFF"),
    (Cmd::GetOverCurrentProtectionActive, ":SOUR:CURR:PROT:TRIP?"),
];

const RS_HMC8043_CMD: &[(Cmd, &str)] = &[
    (Cmd::SelectChannel, "INST:NSEL %s"),
    (Cmd::GetMeasVoltage, "MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, "MEAS:CURR?"),
    (Cmd::GetVoltageTarget, "VOLT?"),
    (Cmd::SetVoltageTarget, "VOLT %.6f"),
    (Cmd::GetCurrentLimit, "CURR?"),
    (Cmd::SetCurrentLimit, "CURR %.6f"),
    (Cmd::GetOutputEnabled, "OUTP?"),
    (Cmd::SetOutputEnable, "OUTP ON"),
    (Cmd::SetOutputDisable, "OUTP OFF"),
    (Cmd::GetOverVoltageProtectionActive, "VOLT:PROT:TRIP?"),
    (Cmd::GetOverVoltageProtectionThreshold, "VOLT:PROT:LEV?"),
    (Cmd::SetOverVoltageProtectionThreshold, "VOLT:PROT:LEV %.6f"),
    (Cmd::GetOverVoltageProtectionEnabled, "VOLT:PROT:STAT?"),
    (Cmd::SetOverVoltageProtectionEnable, "VOLT:PROT:STAT ON"),
    (Cmd::SetOverVoltageProtectionDisable, "VOLT:PROT:STAT OFF"),
    (Cmd::GetOverCurrentProtectionActive, "FUSE:TRIP?"),
    (Cmd::GetOverCurrentProtectionEnabled, "FUSE:STAT?"),
    (Cmd::SetOverCurrentProtectionEnable, "FUSE:STAT ON"),
    (Cmd::SetOverCurrentProtectionDisable, "FUSE:STAT OFF"),
    (Cmd::GetOverCurrentProtectionDelay, "FUSE:DEL?"),
    (Cmd::SetOverCurrentProtectionDelay, "FUSE:DEL %.03f"),
];

const RS_NGE100B_CMD: &[(Cmd, &str)] = &[
    (Cmd::SelectChannel, "INST:NSEL %s"),
    (Cmd::GetMeasVoltage, "MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, "MEAS:CURR?"),
    (Cmd::GetVoltageTarget, "VOLT?"),
    (Cmd::SetVoltageTarget, "VOLT %.6f"),
    (Cmd::GetCurrentLimit, "CURR?"),
    (Cmd::SetCurrentLimit, "CURR %.6f"),
    (Cmd::GetOutputEnabled, "OUTP?"),
    (Cmd::SetOutputEnable, "OUTP ON"),
    (Cmd::SetOutputDisable, "OUTP OFF"),
    (Cmd::GetOverVoltageProtectionActive, "VOLT:PROT:TRIP?"),
    (Cmd::GetOverVoltageProtectionThreshold, "VOLT:PROT:LEV?"),
    (Cmd::SetOverVoltageProtectionThreshold, "VOLT:PROT:LEV %.6f"),
    (Cmd::GetOverVoltageProtectionEnabled, "VOLT:PROT:STAT?"),
    (Cmd::SetOverVoltageProtectionEnable, "VOLT:PROT:STAT ON"),
    (Cmd::SetOverVoltageProtectionDisable, "VOLT:PROT:STAT OFF"),
    (Cmd::GetOverCurrentProtectionActive, "FUSE:TRIP?"),
    (Cmd::GetOverCurrentProtectionEnabled, "FUSE:STAT?"),
    (Cmd::SetOverCurrentProtectionEnable, "FUSE:STAT ON"),
    (Cmd::SetOverCurrentProtectionDisable, "FUSE:STAT OFF"),
    (Cmd::GetOverCurrentProtectionDelay, "FUSE:DEL?"),
    (Cmd::SetOverCurrentProtectionDelay, "FUSE:DEL %.03f"),
];

const RS_HMP4040_CMD: &[(Cmd, &str)] = &[
    (Cmd::Remote, "SYST:REM"),
    (Cmd::Local, "SYST:LOC"),
    (Cmd::SelectChannel, "INST:NSEL %s"),
    (Cmd::GetMeasVoltage, "MEAS:VOLT?"),
    (Cmd::GetMeasCurrent, "MEAS:CURR?"),
    (Cmd::GetVoltageTarget, "VOLT?"),
    (Cmd::SetVoltageTarget, "VOLT %.6f"),
    (Cmd::GetCurrentLimit, "CURR?"),
    (Cmd::SetCurrentLimit, "CURR %.6f"),
    (Cmd::GetOutputEnabled, "OUTP?"),
    (Cmd::SetOutputEnable, "OUTP ON"),
    (Cmd::SetOutputDisable, "OUTP OFF"),
    (Cmd::GetOutputRegulation, "STAT:QUES:INST:ISUM%s:COND?"),
    (Cmd::GetOverVoltageProtectionActive, "VOLT:PROT:TRIP?"),
    (Cmd::GetOverVoltageProtectionThreshold, "VOLT:PROT:LEV?"),
    (Cmd::SetOverVoltageProtectionThreshold, "VOLT:PROT:LEV %.6f"),
    (Cmd::GetOverTemperatureProtectionActive, "STAT:QUES:INST:ISUM%s:COND?"),
];

static AGILENT_N5700A_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static AGILENT_N5700A_DEVOPTS_CG: [OptionDesc; 7] = [
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
];

static BK_9130_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static BK_9130_DEVOPTS_CG: [OptionDesc; 6] = [
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
];

static CHROMA_61604_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static CHROMA_61604_DEVOPTS_CG: [OptionDesc; 8] = [
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::OutputFrequency, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::OutputFrequencyTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
];

static CHROMA_62000_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static CHROMA_62000_DEVOPTS_CG: [OptionDesc; 7] = [
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
];

static EA_PS_20XX_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static EA_PS_20XX_DEVOPTS_CG: [OptionDesc; 7] = [
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: false, set: true, list: true },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionThreshold,
        Access { get: false, set: true, list: true },
    ),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
];

static EEZ_PSU_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static EEZ_PSU_DEVOPTS_CG: [OptionDesc; 10] = [
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::Regulation, Access { get: true, set: false, list: false }),
    OptionDesc::new(
        Key::OverVoltageProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionThreshold,
        Access { get: true, set: false, list: false },
    ),
];

static RIGOL_DP700_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static RIGOL_DP700_DEVOPTS_CG: [OptionDesc; 12] = [
    OptionDesc::new(Key::Regulation, Access { get: true, set: false, list: false }),
    OptionDesc::new(
        Key::OverVoltageProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: true },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionThreshold,
        Access { get: true, set: true, list: true },
    ),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
];

static RIGOL_DP800_DEVOPTS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::OverTemperatureProtection, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static RIGOL_DP800_DEVOPTS_CG: [OptionDesc; 12] = [
    OptionDesc::new(Key::Regulation, Access { get: true, set: false, list: false }),
    OptionDesc::new(
        Key::OverVoltageProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
];

static HP_6630A_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static HP_6630A_DEVOPTS_CG: [OptionDesc; 11] = [
    OptionDesc::new(Key::Enabled, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: false, set: true, list: true }),
    OptionDesc::new(Key::CurrentLimit, Access { get: false, set: true, list: true }),
    OptionDesc::new(
        Key::OverVoltageProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: false, set: true, list: true },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionEnabled,
        Access { get: false, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverTemperatureProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(Key::Regulation, Access { get: true, set: false, list: false }),
];

static HP_6630B_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static HP_6630B_DEVOPTS_CG: [OptionDesc; 11] = [
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(
        Key::OverVoltageProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: true },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverTemperatureProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(Key::Regulation, Access { get: true, set: false, list: false }),
];

static KEYSIGHT_E36300A_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static KEYSIGHT_E36300A_DEVOPTS_CG: [OptionDesc; 11] = [
    OptionDesc::new(Key::Enabled, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: false, set: true, list: true }),
    OptionDesc::new(Key::CurrentLimit, Access { get: false, set: true, list: true }),
    OptionDesc::new(
        Key::OverVoltageProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: false, set: true, list: true },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverTemperatureProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(Key::Regulation, Access { get: true, set: false, list: false }),
];

static OWON_P4000_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static OWON_P4000_DEVOPTS_CG: [OptionDesc; 7] = [
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
];

static PHILIPS_PM2800_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static PHILIPS_PM2800_DEVOPTS_CG: [OptionDesc; 10] = [
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(
        Key::OverVoltageProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(Key::Regulation, Access { get: true, set: false, list: false }),
];

static RS_HMC8043_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static RS_HMC8043_DEVOPTS_CG: [OptionDesc; 11] = [
    OptionDesc::new(
        Key::OverVoltageProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(Key::OverCurrentProtectionDelay, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
];

static RS_NGE100B_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static RS_NGE100B_DEVOPTS_CG: [OptionDesc; 11] = [
    OptionDesc::new(
        Key::OverVoltageProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionEnabled,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(
        Key::OverCurrentProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(Key::OverCurrentProtectionDelay, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
];

static RS_HMP4040_DEVOPTS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::LimitMsec, Access { get: true, set: true, list: false }),
];

static RS_HMP4040_DEVOPTS_CG: [OptionDesc; 10] = [
    OptionDesc::new(
        Key::OverVoltageProtectionEnabled,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(
        Key::OverVoltageProtectionThreshold,
        Access { get: true, set: true, list: false },
    ),
    OptionDesc::new(Key::Voltage, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::VoltageTarget, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Current, Access { get: true, set: false, list: false }),
    OptionDesc::new(Key::CurrentLimit, Access { get: true, set: true, list: true }),
    OptionDesc::new(Key::Enabled, Access { get: true, set: true, list: false }),
    OptionDesc::new(
        Key::OverTemperatureProtectionActive,
        Access { get: true, set: false, list: false },
    ),
    OptionDesc::new(Key::Regulation, Access { get: true, set: false, list: false }),
];

static AGILENT_N5767A_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 60.0, 0.0072, 3.0, 4.0],
    current: [0.0, 25.0, 0.003, 3.0, 4.0],
    power: [0.0, 1500.0, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static AGILENT_N5763A_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 12.5, 0.0015, 3.0, 4.0],
    current: [0.0, 120.0, 0.0144, 3.0, 4.0],
    power: [0.0, 1500.0, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static BK_9130_CH: [ChannelSpec; 3] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 30.0, 0.001, 3.0, 3.0],
        current: [0.0, 3.0, 0.001, 3.0, 3.0],
        power: [0.0, 90.0, 0.0, 3.0, 3.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 30.0, 0.001, 3.0, 3.0],
        current: [0.0, 3.0, 0.001, 3.0, 3.0],
        power: [0.0, 90.0, 0.0, 3.0, 3.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "3",
        voltage: [0.0, 5.0, 0.001, 3.0, 3.0],
        current: [0.0, 3.0, 0.001, 3.0, 3.0],
        power: [0.0, 15.0, 0.0, 3.0, 3.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
];

static CHROMA_61604_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 300.0, 0.1, 1.0, 1.0],
    current: [0.0, 16.0, 0.1, 2.0, 2.0],
    power: [0.0, 2000.0, 0.0, 1.0, 1.0],
    frequency: [1.0, 1000.0, 0.01, 0.0, 0.0],
    ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static RIGOL_DP711_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 30.0, 0.01, 3.0, 3.0],
    current: [0.0, 5.0, 0.01, 3.0, 3.0],
    power: [0.0, 150.0, 0.0, 3.0, 3.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.01, 33.0, 0.01, 0.0, 0.0],
    ocp: [0.01, 5.5, 0.01, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static RIGOL_DP712_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 50.0, 0.01, 3.0, 3.0],
    current: [0.0, 3.0, 0.01, 3.0, 3.0],
    power: [0.0, 150.0, 0.0, 3.0, 3.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.01, 55.0, 0.01, 0.0, 0.0],
    ocp: [0.01, 3.3, 0.01, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static RIGOL_DP821A_CH: [ChannelSpec; 2] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 60.0, 0.001, 3.0, 3.0],
        current: [0.0, 1.0, 0.0001, 4.0, 4.0],
        power: [0.0, 60.0, 0.0, 3.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 8.0, 0.001, 3.0, 3.0],
        current: [0.0, 10.0, 0.001, 3.0, 3.0],
        power: [0.0, 80.0, 0.0, 3.0, 3.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
];

static RIGOL_DP831_CH: [ChannelSpec; 3] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 8.0, 0.001, 3.0, 4.0],
        current: [0.0, 5.0, 0.0003, 3.0, 4.0],
        power: [0.0, 40.0, 0.0, 3.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 30.0, 0.001, 3.0, 4.0],
        current: [0.0, 2.0, 0.0001, 3.0, 4.0],
        power: [0.0, 60.0, 0.0, 3.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "3",
        voltage: [0.0, -30.0, 0.001, 3.0, 4.0],
        current: [0.0, 2.0, 0.0001, 3.0, 4.0],
        power: [0.0, 60.0, 0.0, 3.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
];

static RIGOL_DP832_CH: [ChannelSpec; 3] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 30.0, 0.001, 3.0, 4.0],
        current: [0.0, 3.0, 0.001, 3.0, 4.0],
        power: [0.0, 90.0, 0.0, 3.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 30.0, 0.001, 3.0, 4.0],
        current: [0.0, 3.0, 0.001, 3.0, 4.0],
        power: [0.0, 90.0, 0.0, 3.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "3",
        voltage: [0.0, 5.0, 0.001, 3.0, 4.0],
        current: [0.0, 3.0, 0.001, 3.0, 4.0],
        power: [0.0, 15.0, 0.0, 3.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
];

static HP_6632A_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 20.475, 0.005, 3.0, 4.0],
    current: [0.0, 5.1188, 0.00125, 4.0, 5.0],
    power: [0.0, 104.80743, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 22.0, 0.1, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_6633A_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 51.188, 0.0125, 3.0, 4.0],
    current: [0.0, 2.0475, 0.0005, 4.0, 5.0],
    power: [0.0, 104.80743, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 55.0, 0.25, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_6634A_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 102.38, 0.025, 3.0, 4.0],
    current: [0.0, 1.0238, 0.00025, 4.0, 5.0],
    power: [0.0, 104.81664, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 110.0, 0.5, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_6611C_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 8.19, 0.002, 3.0, 4.0],
    current: [0.0, 5.1188, 0.00125, 4.0, 5.0],
    power: [0.0, 41.92297, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 12.0, 0.06, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_6612C_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 20.475, 0.005, 3.0, 4.0],
    current: [0.0, 2.0475, 0.0005, 4.0, 5.0],
    power: [0.0, 41.92256, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 22.0, 0.1, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_6613C_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 51.188, 0.0125, 3.0, 4.0],
    current: [0.0, 1.0238, 0.00025, 4.0, 5.0],
    power: [0.0, 52.40627, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 55.0, 0.25, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_6614C_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 102.38, 0.025, 3.0, 4.0],
    current: [0.0, 0.5118, 0.000125, 4.0, 5.0],
    power: [0.0, 52.39808, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 110.0, 0.5, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_6631B_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 8.19, 0.002, 3.0, 4.0],
    current: [0.0, 10.237, 0.00263, 4.0, 5.0],
    power: [0.0, 83.84103, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 12.0, 0.06, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_6632B_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 20.475, 0.005, 3.0, 4.0],
    current: [0.0, 5.1188, 0.00132, 4.0, 5.0],
    power: [0.0, 104.80743, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 22.0, 0.1, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_66312A_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 20.475, 0.0001, 4.0, 5.0],
    current: [0.0, 2.0475, 0.0001, 4.0, 5.0],
    power: [0.0, 41.92256, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 22.0, 0.01, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_66332A_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 20.475, 0.005, 3.0, 4.0],
    current: [0.0, 5.1188, 0.00132, 4.0, 5.0],
    power: [0.0, 104.80743, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 22.0, 0.1, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_6633B_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 51.188, 0.0125, 3.0, 4.0],
    current: [0.0, 2.0475, 0.000526, 4.0, 5.0],
    power: [0.0, 104.80743, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 55.0, 0.25, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static HP_6634B_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.0, 102.38, 0.025, 3.0, 4.0],
    current: [0.0, 1.0238, 0.000263, 4.0, 5.0],
    power: [0.0, 104.81664, 0.0, 0.0, 0.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.0, 110.0, 0.5, 0.0, 0.0],
    ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static KEYSIGHT_E36311A_CH: [ChannelSpec; 3] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 6.18, 0.001, 3.0, 8.0],
        current: [0.002, 5.15, 0.001, 3.0, 8.0],
        power: [0.0, 31.827, 0.0, 0.0, 0.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.5, 6.6, 0.001, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 25.75, 0.001, 3.0, 8.0],
        current: [0.001, 1.03, 0.001, 3.0, 8.0],
        power: [0.0, 26.5225, 0.0, 0.0, 0.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.5, 27.5, 0.001, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "3",
        voltage: [-25.75, 0.0, 0.001, 3.0, 8.0],
        current: [0.001, 1.03, 0.001, 3.0, 8.0],
        power: [0.0, 26.5225, 0.0, 0.0, 0.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.5, 27.5, 0.001, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
];

static KEYSIGHT_E36312A_CH: [ChannelSpec; 3] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 6.18, 0.001, 3.0, 8.0],
        current: [0.002, 5.15, 0.001, 3.0, 8.0],
        power: [0.0, 31.827, 0.0, 0.0, 0.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.5, 6.6, 0.001, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 25.75, 0.001, 3.0, 8.0],
        current: [0.001, 1.03, 0.001, 3.0, 8.0],
        power: [0.0, 26.5225, 0.0, 0.0, 0.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.5, 27.5, 0.001, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "3",
        voltage: [0.0, 25.75, 0.001, 3.0, 8.0],
        current: [0.001, 1.03, 0.001, 3.0, 8.0],
        power: [0.0, 26.5225, 0.0, 0.0, 0.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.5, 27.5, 0.001, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
];

static KEYSIGHT_E36313A_CH: [ChannelSpec; 3] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 6.18, 0.001, 3.0, 8.0],
        current: [0.002, 10.3, 0.001, 3.0, 8.0],
        power: [0.0, 63.654, 0.0, 0.0, 0.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.5, 6.6, 0.001, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 25.75, 0.001, 3.0, 8.0],
        current: [0.001, 2.06, 0.001, 3.0, 8.0],
        power: [0.0, 53.045, 0.0, 0.0, 0.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.5, 27.5, 0.001, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "3",
        voltage: [0.0, 25.75, 0.001, 3.0, 8.0],
        current: [0.001, 2.06, 0.001, 3.0, 8.0],
        power: [0.0, 53.045, 0.0, 0.0, 0.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.5, 27.5, 0.001, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
];

static OWON_P4603_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.01, 60.0, 0.001, 3.0, 3.0],
    current: [0.001, 3.0, 0.001, 3.0, 3.0],
    power: [0.0, 180.0, 0.0, 3.0, 3.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.01, 61.0, 0.001, 0.0, 0.0],
    ocp: [0.001, 3.1, 0.001, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static OWON_P4305_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.01, 30.0, 0.001, 3.0, 3.0],
    current: [0.001, 5.0, 0.001, 3.0, 3.0],
    power: [0.0, 180.0, 0.0, 3.0, 3.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.01, 31.0, 0.001, 0.0, 0.0],
    ocp: [0.001, 3.1, 0.001, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static OWON_SPE6103_CH: [ChannelSpec; 1] = [ChannelSpec {
    name: "1",
    voltage: [0.01, 60.0, 0.01, 3.0, 3.0],
    current: [0.001, 10.0, 0.001, 3.0, 3.0],
    power: [0.0, 300.0, 0.0, 3.0, 3.0],
    frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
    ovp: [0.01, 61.0, 0.01, 0.0, 0.0],
    ocp: [0.001, 10.1, 0.001, 0.0, 0.0],
    ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
}];

static RS_HMC8043_CH: [ChannelSpec; 3] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 3.0, 0.001, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.01, 10.0, 0.001, 3.0, 4.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 3.0, 0.001, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.01, 10.0, 0.001, 3.0, 4.0],
    },
    ChannelSpec {
        name: "3",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 3.0, 0.001, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.01, 10.0, 0.001, 3.0, 4.0],
    },
];

static RS_NGE100B_CH: [ChannelSpec; 3] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 3.0, 0.001, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.01, 10.0, 0.001, 3.0, 4.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 3.0, 0.001, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.01, 10.0, 0.001, 3.0, 4.0],
    },
    ChannelSpec {
        name: "3",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 3.0, 0.001, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.01, 10.0, 0.001, 3.0, 4.0],
    },
];

static RS_HMP2020_CH: [ChannelSpec; 2] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 10.01, 0.0002, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 5.01, 0.0001, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
];

static RS_HMP2030_CH: [ChannelSpec; 3] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 5.01, 0.0001, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 5.01, 0.0001, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "3",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 5.01, 0.0001, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
];

static RS_HMP4040_CH: [ChannelSpec; 4] = [
    ChannelSpec {
        name: "1",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 10.01, 0.0002, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "2",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 10.01, 0.0002, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "3",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 10.01, 0.0002, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
    ChannelSpec {
        name: "4",
        voltage: [0.0, 32.05, 0.001, 3.0, 4.0],
        current: [0.001, 10.01, 0.0002, 3.0, 4.0],
        power: [0.0, 0.0, 0.0, 0.0, 4.0],
        frequency: [0.0, 0.0, 0.0, 0.0, 0.0],
        ovp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp: [0.0, 0.0, 0.0, 0.0, 0.0],
        ocp_delay: [0.0, 0.0, 0.0, 0.0, 0.0],
    },
];

static AGILENT_N5700A_CG: [GroupSpec; 1] = [GroupSpec { name: "1", channel: 0, ac: false }];

static BK_9130_CG: [GroupSpec; 3] = [
    GroupSpec { name: "1", channel: 0, ac: false },
    GroupSpec { name: "2", channel: 1, ac: false },
    GroupSpec { name: "3", channel: 2, ac: false },
];

static CHROMA_61604_CG: [GroupSpec; 1] = [GroupSpec { name: "1", channel: 0, ac: true }];

static CHROMA_62000_CG: [GroupSpec; 1] = [GroupSpec { name: "1", channel: 0, ac: false }];

static EA_PS_20XX_CG: [GroupSpec; 1] = [GroupSpec { name: "1", channel: 0, ac: false }];

static RIGOL_DP700_CG: [GroupSpec; 1] = [GroupSpec { name: "1", channel: 0, ac: false }];

static RIGOL_DP820_CG: [GroupSpec; 2] = [
    GroupSpec { name: "1", channel: 0, ac: false },
    GroupSpec { name: "2", channel: 1, ac: false },
];

static RIGOL_DP830_CG: [GroupSpec; 3] = [
    GroupSpec { name: "1", channel: 0, ac: false },
    GroupSpec { name: "2", channel: 1, ac: false },
    GroupSpec { name: "3", channel: 2, ac: false },
];

static HP_6630A_CG: [GroupSpec; 1] = [GroupSpec { name: "1", channel: 0, ac: false }];

static HP_6630B_CG: [GroupSpec; 1] = [GroupSpec { name: "1", channel: 0, ac: false }];

static KEYSIGHT_E36300A_CG: [GroupSpec; 3] = [
    GroupSpec { name: "1", channel: 0, ac: false },
    GroupSpec { name: "2", channel: 1, ac: false },
    GroupSpec { name: "3", channel: 2, ac: false },
];

static OWON_P4000_CG: [GroupSpec; 1] = [GroupSpec { name: "1", channel: 0, ac: false }];

static RS_HMC8043_CG: [GroupSpec; 3] = [
    GroupSpec { name: "1", channel: 0, ac: false },
    GroupSpec { name: "2", channel: 1, ac: false },
    GroupSpec { name: "3", channel: 2, ac: false },
];

static RS_NGE100B_CG: [GroupSpec; 3] = [
    GroupSpec { name: "1", channel: 0, ac: false },
    GroupSpec { name: "2", channel: 1, ac: false },
    GroupSpec { name: "3", channel: 2, ac: false },
];

static RS_HMP4040_CG: [GroupSpec; 4] = [
    GroupSpec { name: "1", channel: 0, ac: false },
    GroupSpec { name: "2", channel: 1, ac: false },
    GroupSpec { name: "3", channel: 2, ac: false },
    GroupSpec { name: "4", channel: 3, ac: false },
];

/// Every profile, in `pps_profiles`' order: the first that matches wins.
pub static PROFILES: [Profile; 44] = [
    Profile {
        vendor: "Agilent",
        model: Match::Contains("N5763A"),
        dialect: Dialect::Unknown,
        device_options: &AGILENT_N5700A_DEVOPTS,
        options: &AGILENT_N5700A_DEVOPTS_CG,
        channels: &AGILENT_N5763A_CH,
        groups: &AGILENT_N5700A_CG,
        commands: AGILENT_N5700A_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Agilent",
        model: Match::Contains("N5767A"),
        dialect: Dialect::Unknown,
        device_options: &AGILENT_N5700A_DEVOPTS,
        options: &AGILENT_N5700A_DEVOPTS_CG,
        channels: &AGILENT_N5767A_CH,
        groups: &AGILENT_N5700A_CG,
        commands: AGILENT_N5700A_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "BK",
        model: Match::Whole("9130"),
        dialect: Dialect::Unknown,
        device_options: &BK_9130_DEVOPTS,
        options: &BK_9130_DEVOPTS_CG,
        channels: &BK_9130_CH,
        groups: &BK_9130_CG,
        commands: BK_9130_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Chroma",
        model: Match::Contains("61604"),
        dialect: Dialect::Unknown,
        device_options: &CHROMA_61604_DEVOPTS,
        options: &CHROMA_61604_DEVOPTS_CG,
        channels: &CHROMA_61604_CH,
        groups: &CHROMA_61604_CG,
        commands: CHROMA_61604_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Chroma",
        model: Match::By(is_chroma_62000),
        dialect: Dialect::Unknown,
        device_options: &CHROMA_62000_DEVOPTS,
        options: &CHROMA_62000_DEVOPTS_CG,
        channels: &[],
        groups: &[],
        commands: CHROMA_62000_CMD,
        probe: Probe::Chroma62000,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "EA Elektro-Automatik GmbH & Co. KG",
        model: Match::By(is_ea_ps_20xx),
        dialect: Dialect::Unknown,
        device_options: &EA_PS_20XX_DEVOPTS,
        options: &EA_PS_20XX_DEVOPTS_CG,
        channels: &[],
        groups: &[],
        commands: EA_PS_20XX_CMD,
        probe: Probe::EaPs20xx,
        init: Init::None,
        status: Status::Ea,
    },
    Profile {
        vendor: "Envox",
        model: Match::Starts("EEZ H24005 "),
        dialect: Dialect::Unknown,
        device_options: &EEZ_PSU_DEVOPTS,
        options: &EEZ_PSU_DEVOPTS_CG,
        channels: &[],
        groups: &[],
        commands: EEZ_PSU_CMD,
        probe: Probe::Eez,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "EEZ",
        model: Match::Starts("PSU "),
        dialect: Dialect::Unknown,
        device_options: &EEZ_PSU_DEVOPTS,
        options: &EEZ_PSU_DEVOPTS_CG,
        channels: &[],
        groups: &[],
        commands: EEZ_PSU_CMD,
        probe: Probe::Eez,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Envox",
        model: Match::Starts("BB3 "),
        dialect: Dialect::Unknown,
        device_options: &EEZ_PSU_DEVOPTS,
        options: &EEZ_PSU_DEVOPTS_CG,
        channels: &[],
        groups: &[],
        commands: EEZ_PSU_CMD,
        probe: Probe::Eez,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6632A"),
        dialect: Dialect::HpComp,
        device_options: &HP_6630A_DEVOPTS,
        options: &HP_6630A_DEVOPTS_CG,
        channels: &HP_6632A_CH,
        groups: &HP_6630A_CG,
        commands: HP_6630A_CMD,
        probe: Probe::Table,
        init: Init::Hp6630a,
        status: Status::Hp6630a,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6633A"),
        dialect: Dialect::HpComp,
        device_options: &HP_6630A_DEVOPTS,
        options: &HP_6630A_DEVOPTS_CG,
        channels: &HP_6633A_CH,
        groups: &HP_6630A_CG,
        commands: HP_6630A_CMD,
        probe: Probe::Table,
        init: Init::Hp6630a,
        status: Status::Hp6630a,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6634A"),
        dialect: Dialect::HpComp,
        device_options: &HP_6630A_DEVOPTS,
        options: &HP_6630A_DEVOPTS_CG,
        channels: &HP_6634A_CH,
        groups: &HP_6630A_CG,
        commands: HP_6630A_CMD,
        probe: Probe::Table,
        init: Init::Hp6630a,
        status: Status::Hp6630a,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6611C"),
        dialect: Dialect::Hp66xxb,
        device_options: &HP_6630B_DEVOPTS,
        options: &HP_6630B_DEVOPTS_CG,
        channels: &HP_6611C_CH,
        groups: &HP_6630B_CG,
        commands: HP_6630B_CMD,
        probe: Probe::Table,
        init: Init::Hp6630b,
        status: Status::Hp6630b,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6612C"),
        dialect: Dialect::Hp66xxb,
        device_options: &HP_6630B_DEVOPTS,
        options: &HP_6630B_DEVOPTS_CG,
        channels: &HP_6612C_CH,
        groups: &HP_6630B_CG,
        commands: HP_6630B_CMD,
        probe: Probe::Table,
        init: Init::Hp6630b,
        status: Status::Hp6630b,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6613C"),
        dialect: Dialect::Hp66xxb,
        device_options: &HP_6630B_DEVOPTS,
        options: &HP_6630B_DEVOPTS_CG,
        channels: &HP_6613C_CH,
        groups: &HP_6630B_CG,
        commands: HP_6630B_CMD,
        probe: Probe::Table,
        init: Init::Hp6630b,
        status: Status::Hp6630b,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6614C"),
        dialect: Dialect::Hp66xxb,
        device_options: &HP_6630B_DEVOPTS,
        options: &HP_6630B_DEVOPTS_CG,
        channels: &HP_6614C_CH,
        groups: &HP_6630B_CG,
        commands: HP_6630B_CMD,
        probe: Probe::Table,
        init: Init::Hp6630b,
        status: Status::Hp6630b,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6631B"),
        dialect: Dialect::Hp66xxb,
        device_options: &HP_6630B_DEVOPTS,
        options: &HP_6630B_DEVOPTS_CG,
        channels: &HP_6631B_CH,
        groups: &HP_6630B_CG,
        commands: HP_6630B_CMD,
        probe: Probe::Table,
        init: Init::Hp6630b,
        status: Status::Hp6630b,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6632B"),
        dialect: Dialect::Hp66xxb,
        device_options: &HP_6630B_DEVOPTS,
        options: &HP_6630B_DEVOPTS_CG,
        channels: &HP_6632B_CH,
        groups: &HP_6630B_CG,
        commands: HP_6630B_CMD,
        probe: Probe::Table,
        init: Init::Hp6630b,
        status: Status::Hp6630b,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("66312A"),
        dialect: Dialect::Hp66xxb,
        device_options: &HP_6630B_DEVOPTS,
        options: &HP_6630B_DEVOPTS_CG,
        channels: &HP_66312A_CH,
        groups: &HP_6630B_CG,
        commands: HP_6630B_CMD,
        probe: Probe::Table,
        init: Init::Hp6630b,
        status: Status::Hp6630b,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("66332A"),
        dialect: Dialect::Hp66xxb,
        device_options: &HP_6630B_DEVOPTS,
        options: &HP_6630B_DEVOPTS_CG,
        channels: &HP_66332A_CH,
        groups: &HP_6630B_CG,
        commands: HP_6630B_CMD,
        probe: Probe::Table,
        init: Init::Hp6630b,
        status: Status::Hp6630b,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6633B"),
        dialect: Dialect::Hp66xxb,
        device_options: &HP_6630B_DEVOPTS,
        options: &HP_6630B_DEVOPTS_CG,
        channels: &HP_6633B_CH,
        groups: &HP_6630B_CG,
        commands: HP_6630B_CMD,
        probe: Probe::Table,
        init: Init::Hp6630b,
        status: Status::Hp6630b,
    },
    Profile {
        vendor: "HP",
        model: Match::Contains("6634B"),
        dialect: Dialect::Hp66xxb,
        device_options: &HP_6630B_DEVOPTS,
        options: &HP_6630B_DEVOPTS_CG,
        channels: &HP_6634B_CH,
        groups: &HP_6630B_CG,
        commands: HP_6630B_CMD,
        probe: Probe::Table,
        init: Init::Hp6630b,
        status: Status::Hp6630b,
    },
    Profile {
        vendor: "Keysight",
        model: Match::Contains("E36311A"),
        dialect: Dialect::KeysightE36300a,
        device_options: &KEYSIGHT_E36300A_DEVOPTS,
        options: &KEYSIGHT_E36300A_DEVOPTS_CG,
        channels: &KEYSIGHT_E36311A_CH,
        groups: &KEYSIGHT_E36300A_CG,
        commands: KEYSIGHT_E36300A_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Keysight",
        model: Match::Contains("E36312A"),
        dialect: Dialect::KeysightE36300a,
        device_options: &KEYSIGHT_E36300A_DEVOPTS,
        options: &KEYSIGHT_E36300A_DEVOPTS_CG,
        channels: &KEYSIGHT_E36312A_CH,
        groups: &KEYSIGHT_E36300A_CG,
        commands: KEYSIGHT_E36300A_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Keysight",
        model: Match::Contains("E36313A"),
        dialect: Dialect::KeysightE36300a,
        device_options: &KEYSIGHT_E36300A_DEVOPTS,
        options: &KEYSIGHT_E36300A_DEVOPTS_CG,
        channels: &KEYSIGHT_E36313A_CH,
        groups: &KEYSIGHT_E36300A_CG,
        commands: KEYSIGHT_E36300A_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Rigol",
        model: Match::Whole("DP711"),
        dialect: Dialect::Unknown,
        device_options: &RIGOL_DP700_DEVOPTS,
        options: &RIGOL_DP700_DEVOPTS_CG,
        channels: &RIGOL_DP711_CH,
        groups: &RIGOL_DP700_CG,
        commands: RIGOL_DP700_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Rigol",
        model: Match::Whole("DP712"),
        dialect: Dialect::Unknown,
        device_options: &RIGOL_DP700_DEVOPTS,
        options: &RIGOL_DP700_DEVOPTS_CG,
        channels: &RIGOL_DP712_CH,
        groups: &RIGOL_DP700_CG,
        commands: RIGOL_DP700_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Rigol",
        model: Match::Whole("DP821A"),
        dialect: Dialect::Unknown,
        device_options: &RIGOL_DP800_DEVOPTS,
        options: &RIGOL_DP800_DEVOPTS_CG,
        channels: &RIGOL_DP821A_CH,
        groups: &RIGOL_DP820_CG,
        commands: RIGOL_DP800_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Rigol",
        model: Match::Whole("DP831A"),
        dialect: Dialect::Unknown,
        device_options: &RIGOL_DP800_DEVOPTS,
        options: &RIGOL_DP800_DEVOPTS_CG,
        channels: &RIGOL_DP831_CH,
        groups: &RIGOL_DP830_CG,
        commands: RIGOL_DP800_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Rigol",
        model: Match::By(is_dp832),
        dialect: Dialect::Unknown,
        device_options: &RIGOL_DP800_DEVOPTS,
        options: &RIGOL_DP800_DEVOPTS_CG,
        channels: &RIGOL_DP832_CH,
        groups: &RIGOL_DP830_CG,
        commands: RIGOL_DP800_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "OWON",
        model: Match::Whole("P4305"),
        dialect: Dialect::Unknown,
        device_options: &OWON_P4000_DEVOPTS,
        options: &OWON_P4000_DEVOPTS_CG,
        channels: &OWON_P4305_CH,
        groups: &OWON_P4000_CG,
        commands: OWON_P4000_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "OWON",
        model: Match::Whole("P4603"),
        dialect: Dialect::Unknown,
        device_options: &OWON_P4000_DEVOPTS,
        options: &OWON_P4000_DEVOPTS_CG,
        channels: &OWON_P4603_CH,
        groups: &OWON_P4000_CG,
        commands: OWON_P4000_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "OWON",
        model: Match::Whole("SPE6103"),
        dialect: Dialect::Unknown,
        device_options: &OWON_P4000_DEVOPTS,
        options: &OWON_P4000_DEVOPTS_CG,
        channels: &OWON_SPE6103_CH,
        groups: &OWON_P4000_CG,
        commands: OWON_SPE6103_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Philips",
        model: Match::By(is_pm2800),
        dialect: Dialect::Philips,
        device_options: &PHILIPS_PM2800_DEVOPTS,
        options: &PHILIPS_PM2800_DEVOPTS_CG,
        channels: &[],
        groups: &[],
        commands: PHILIPS_PM2800_CMD,
        probe: Probe::Pm2800,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Rohde&Schwarz",
        model: Match::Contains("HMC8042"),
        dialect: Dialect::Unknown,
        device_options: &RS_HMC8043_DEVOPTS,
        options: &RS_HMC8043_DEVOPTS_CG,
        channels: first(&RS_HMC8043_CH, 2),
        groups: first(&RS_HMC8043_CG, 2),
        commands: RS_HMC8043_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Rohde&Schwarz",
        model: Match::Contains("HMC8043"),
        dialect: Dialect::Unknown,
        device_options: &RS_HMC8043_DEVOPTS,
        options: &RS_HMC8043_DEVOPTS_CG,
        channels: &RS_HMC8043_CH,
        groups: &RS_HMC8043_CG,
        commands: RS_HMC8043_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Rohde&Schwarz",
        model: Match::Contains("NGE102B"),
        dialect: Dialect::Unknown,
        device_options: &RS_NGE100B_DEVOPTS,
        options: &RS_NGE100B_DEVOPTS_CG,
        channels: first(&RS_NGE100B_CH, 2),
        groups: first(&RS_NGE100B_CG, 2),
        commands: RS_NGE100B_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "Rohde&Schwarz",
        model: Match::Contains("NGE103B"),
        dialect: Dialect::Unknown,
        device_options: &RS_NGE100B_DEVOPTS,
        options: &RS_NGE100B_DEVOPTS_CG,
        channels: &RS_NGE100B_CH,
        groups: &RS_NGE100B_CG,
        commands: RS_NGE100B_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "HAMEG",
        model: Match::Contains("HMP4030"),
        dialect: Dialect::Hmp,
        device_options: &RS_HMP4040_DEVOPTS,
        options: &RS_HMP4040_DEVOPTS_CG,
        channels: first(&RS_HMP4040_CH, 3),
        groups: first(&RS_HMP4040_CG, 3),
        commands: RS_HMP4040_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "HAMEG",
        model: Match::Contains("HMP4040"),
        dialect: Dialect::Hmp,
        device_options: &RS_HMP4040_DEVOPTS,
        options: &RS_HMP4040_DEVOPTS_CG,
        channels: &RS_HMP4040_CH,
        groups: &RS_HMP4040_CG,
        commands: RS_HMP4040_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "ROHDE&SCHWARZ",
        model: Match::Contains("HMP2020"),
        dialect: Dialect::Hmp,
        device_options: &RS_HMP4040_DEVOPTS,
        options: &RS_HMP4040_DEVOPTS_CG,
        channels: first(&RS_HMP2020_CH, 2),
        groups: first(&RS_HMP4040_CG, 2),
        commands: RS_HMP4040_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "ROHDE&SCHWARZ",
        model: Match::Contains("HMP2030"),
        dialect: Dialect::Hmp,
        device_options: &RS_HMP4040_DEVOPTS,
        options: &RS_HMP4040_DEVOPTS_CG,
        channels: first(&RS_HMP2030_CH, 3),
        groups: first(&RS_HMP4040_CG, 3),
        commands: RS_HMP4040_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "ROHDE&SCHWARZ",
        model: Match::Contains("HMP4030"),
        dialect: Dialect::Hmp,
        device_options: &RS_HMP4040_DEVOPTS,
        options: &RS_HMP4040_DEVOPTS_CG,
        channels: first(&RS_HMP4040_CH, 3),
        groups: first(&RS_HMP4040_CG, 3),
        commands: RS_HMP4040_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
    Profile {
        vendor: "ROHDE&SCHWARZ",
        model: Match::Contains("HMP4040"),
        dialect: Dialect::Hmp,
        device_options: &RS_HMP4040_DEVOPTS,
        options: &RS_HMP4040_DEVOPTS_CG,
        channels: &RS_HMP4040_CH,
        groups: &RS_HMP4040_CG,
        commands: RS_HMP4040_CMD,
        probe: Probe::Table,
        init: Init::None,
        status: Status::None,
    },
];
