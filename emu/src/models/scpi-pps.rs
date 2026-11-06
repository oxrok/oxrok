// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The bench supplies of libsigrok's `scpi-pps` and `hpib-pps`, on USBTMC and on TCP
//! (`scpi-pps-serial` for a serial port): one model with every profile's commands.
//!
//! How sure this is. The command tables below are `profiles.c`'s, and they are what
//! the model knows: what a real supply answers, and in which form, comes from no
//! recording and from no unit on a bench.
//!
//! - `ROM?`'s `B01 B01` and an EA's unit after its nominal values are in the C
//!   driver's comments. The EA's `84.00 V` is the form its "Remove unit" implies.
//! - The status bits are what `api.c` and `profiles.c` decode. The word a real HP or
//!   Keysight gives was not compared with anything.
//! - `:CURR:PROT:STAT ON?` on an N5700 (`profiles.c:82-83`) is taken as a command
//!   error with no state change, which any SCPI parser would make of a boolean with a
//!   question mark. No unit said so, and the plain form is taken as the manual's
//!   `CURRent:PROTection:STATe <Bool>`, from memory.
//! - That a value beyond a table's range by 10% is refused with `-222` and no change
//!   is a guess at what an instrument does. A real one may clamp. The 10% is there
//!   as tables are lower than what a unit takes: a DP832 is listed at 30 V.
//! - The USB id is a DP832's from memory (1ab1:0e11) for all models, as a change of
//!   ids costs a re-attach. So nothing leans on it, and no id in libsigrok's table of
//!   remote control lies applies.
//! - The answers' number forms: `{:.4}` by default, `+1.200000E+01` on an HP 66xxB and
//!   an N5700, `{:.3}` on an HP in COMP language, are guesses in the shapes a
//!   SCPI standard allows, there so a parser meets more than one.
//!
//! Emulation artifacts: a load of `load=` ohms on each output, a noise of 1 mV times
//! the query's count modulo 8 so that no two readings look alike, and nothing
//! settles, so a set point shows in the next reading. Power on is with all outputs
//! off, unless `on=1`. The model counts in queries, so no wall time is in it.
//!
//! Options, besides the shell's ([`scpi::Config::option`]):
//!
//! - `model=NAME`: as `*IDN?` says it, from the list in [`KINDS`], `_` for a space, as
//!   an option holds none; so too in `say=`, `rom=` and `nominal=`;
//! - `say=TEXT`: what `*IDN?` says;
//! - `load=OHMS`: 8 by default; `on=1`: outputs on at power on;
//! - `channels=N`: an EEZ's `:SYST:CHAN:COUN?`;
//! - `beeper=0`: the beeper off at power on;
//! - `event-at=N:WHAT,...`: at the Nth reading the outputs' state becomes `cv`, `cc`,
//!   `ccn`, `ur` or `idle`, or a protection trips (`ovp`, `ocp`, `otp`) or clears
//!   (`clear`). That reaches the status words and the event registers. No model
//!   event happens without it but those of a load;
//! - `reg=TEXT`: the answer to the regulation query as it stands, no bits;
//! - `status=TEXT`: the answer to every status word query as it stands;
//! - `bad-at=N:TEXT`: the answer to the Nth reading, and `bad=TEXT`: to all;
//! - `hpid=TEXT`: what `ID?` says (`ID?` is only an HP in COMP language's otherwise);
//!   `rom=TEXT`: what `ROM?` says, `mute` for nothing;
//! - `nominal=TEXT`: an EA's answer to the three nominal queries, `chan-info=TEXT`:
//!   an EEZ's to its limits.
//!
//! The pedantic checks: only what a model's table has, what the Engine flags; a
//! channel with no output refuses with `-222`, as a real one would.

use crate::model::{DeviceModel, Violations};
use crate::options::parse_number;
use crate::shells::scpi::{self, Answer, Instrument, Message};
use crate::shells::tcp::Stream;
use crate::shells::usbtmc::Identity;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "scpi-pps";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "SCPI and HP-IB bench supplies on USBTMC and TCP; model=, load=, on=, \
                         channels=, beeper=, event-at=, reg=, status=, bad-at=, bad=, say=, hpid=, rom=, \
                         nominal=, chan-info=, and the SCPI shell's mute-at=, silent-at=, late-at=, \
                         mark-at=, idn=, eol=, stall=, in-stall=, empty-at=, bad-tag-at=, message=";

/// What a table's text does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(clippy::enum_variant_names)]
enum A {
    Beeper,
    BeeperDisable,
    BeeperEnable,
    GetCurrentLimit,
    GetFrequencyTarget,
    GetMeasCurrent,
    GetMeasFrequency,
    GetMeasPower,
    GetMeasVoltage,
    GetOutputEnabled,
    GetOutputRegulation,
    GetOverCurrentProtectionActive,
    GetOverCurrentProtectionDelay,
    GetOverCurrentProtectionEnabled,
    GetOverCurrentProtectionThreshold,
    GetOverTemperatureProtection,
    GetOverTemperatureProtectionActive,
    GetOverVoltageProtectionActive,
    GetOverVoltageProtectionEnabled,
    GetOverVoltageProtectionThreshold,
    GetVoltageTarget,
    Local,
    Remote,
    SelectChannel,
    SetCurrentLimit,
    SetFrequencyTarget,
    SetOutputDisable,
    SetOutputEnable,
    SetOverCurrentProtectionDelay,
    SetOverCurrentProtectionDisable,
    SetOverCurrentProtectionEnable,
    SetOverCurrentProtectionThreshold,
    SetOverTemperatureProtectionDisable,
    SetOverTemperatureProtectionEnable,
    SetOverVoltageProtectionDisable,
    SetOverVoltageProtectionEnable,
    SetOverVoltageProtectionThreshold,
    SetVoltageTarget,
}

/// How a status word reads, as `api.c` decodes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Dialect {
    Plain,
    /// `STS?`, `FAULT?`.
    HpComp,
    /// `STAT:OPER:COND?`, `STAT:QUES:COND?`, `*STB?`.
    Hp66xxb,
    /// `:SOUR:FUNC:MODE?` says `VOLT` or `CURR`.
    Philips,
    /// `..ISUMn:COND?`, bit 0 CC, bit 1 CV, bit 4 OTP.
    Hmp,
}

/// Extra commands a family has outside its table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Extra {
    None,
    Ea,
    Eez,
}

/// A supply as far as it shows on the wire.
struct Kind {
    idn: &'static str,
    /// As it says it; `_` in `model=` stands for a space.
    model: &'static str,
    dialect: Dialect,
    extra: Extra,
    table: &'static [(A, &'static str)],
    /// Each output's lowest and highest volts and highest amperes.
    outputs: &'static [(f64, f64, f64)],
}

const A5: (f64, f64, f64) = (0.0, 32.05, 10.01);
const B5: (f64, f64, f64) = (0.0, 32.05, 5.01);
const C3: (f64, f64, f64) = (0.0, 32.05, 3.0);

#[rustfmt::skip]
const KINDS: &[Kind] = &[
    Kind { idn: "Agilent Technologies", model: "N5763A", dialect: Dialect::Plain, extra: Extra::None, table: AGILENT_N5700A_CMD, outputs: &[(0.0, 12.5, 120.0)] },
    Kind { idn: "Agilent Technologies", model: "N5767A", dialect: Dialect::Plain, extra: Extra::None, table: AGILENT_N5700A_CMD, outputs: &[(0.0, 60.0, 25.0)] },
    Kind { idn: "BK", model: "9130", dialect: Dialect::Plain, extra: Extra::None, table: BK_9130_CMD, outputs: &[(0.0, 30.0, 3.0), (0.0, 30.0, 3.0), (0.0, 5.0, 3.0)] },
    Kind { idn: "Chroma ATE", model: "61604", dialect: Dialect::Plain, extra: Extra::None, table: CHROMA_61604_CMD, outputs: &[(0.0, 300.0, 16.0)] },
    Kind { idn: "CHROMA", model: "62012P-80-60", dialect: Dialect::Plain, extra: Extra::None, table: CHROMA_62000_CMD, outputs: &[(0.0, 80.0, 60.0)] },
    Kind { idn: "EA Elektro-Automatik GmbH & Co. KG", model: "PS 2084-05B", dialect: Dialect::Plain, extra: Extra::Ea, table: EA_PS_20XX_CMD, outputs: &[(0.0, 84.0, 5.0)] },
    Kind { idn: "Envox", model: "EEZ H24005 R3B4", dialect: Dialect::Plain, extra: Extra::Eez, table: EEZ_PSU_CMD, outputs: &[(0.0, 40.0, 5.0), (0.0, 40.0, 5.0), (0.0, 40.0, 5.0), (0.0, 40.0, 5.0), (0.0, 40.0, 5.0), (0.0, 40.0, 5.0)] },
    Kind { idn: "EEZ", model: "PSU 1.02", dialect: Dialect::Plain, extra: Extra::Eez, table: EEZ_PSU_CMD, outputs: &[(0.0, 40.0, 5.0), (0.0, 40.0, 5.0)] },
    Kind { idn: "Envox", model: "BB3 r3b3", dialect: Dialect::Plain, extra: Extra::Eez, table: EEZ_PSU_CMD, outputs: &[(0.0, 50.0, 5.0), (0.0, 50.0, 5.0), (0.0, 50.0, 5.0), (0.0, 50.0, 5.0), (0.0, 50.0, 5.0), (0.0, 50.0, 5.0)] },
    Kind { idn: "HP", model: "6632A", dialect: Dialect::HpComp, extra: Extra::None, table: HP_6630A_CMD, outputs: &[(0.0, 20.475, 5.1188)] },
    Kind { idn: "HP", model: "6633A", dialect: Dialect::HpComp, extra: Extra::None, table: HP_6630A_CMD, outputs: &[(0.0, 51.188, 2.0475)] },
    Kind { idn: "HP", model: "6634A", dialect: Dialect::HpComp, extra: Extra::None, table: HP_6630A_CMD, outputs: &[(0.0, 102.38, 1.0238)] },
    Kind { idn: "HEWLETT-PACKARD", model: "6611C", dialect: Dialect::Hp66xxb, extra: Extra::None, table: HP_6630B_CMD, outputs: &[(0.0, 8.19, 5.1188)] },
    Kind { idn: "HEWLETT-PACKARD", model: "6632B", dialect: Dialect::Hp66xxb, extra: Extra::None, table: HP_6630B_CMD, outputs: &[(0.0, 20.475, 5.1188)] },
    Kind { idn: "HEWLETT-PACKARD", model: "66312A", dialect: Dialect::Hp66xxb, extra: Extra::None, table: HP_6630B_CMD, outputs: &[(0.0, 20.475, 2.0475)] },
    Kind { idn: "Keysight Technologies", model: "E36311A", dialect: Dialect::Hmp, extra: Extra::None, table: KEYSIGHT_E36300A_CMD, outputs: &[(0.0, 6.18, 5.15), (0.0, 25.75, 1.03), (-25.75, 0.0, 1.03)] },
    Kind { idn: "Keysight Technologies", model: "E36312A", dialect: Dialect::Hmp, extra: Extra::None, table: KEYSIGHT_E36300A_CMD, outputs: &[(0.0, 6.18, 5.15), (0.0, 25.75, 1.03), (0.0, 25.75, 1.03)] },
    Kind { idn: "RIGOL TECHNOLOGIES", model: "DP711", dialect: Dialect::Plain, extra: Extra::None, table: RIGOL_DP700_CMD, outputs: &[(0.0, 30.0, 5.0)] },
    Kind { idn: "RIGOL TECHNOLOGIES", model: "DP821A", dialect: Dialect::Plain, extra: Extra::None, table: RIGOL_DP800_CMD, outputs: &[(0.0, 60.0, 1.0), (0.0, 8.0, 10.0)] },
    Kind { idn: "RIGOL TECHNOLOGIES", model: "DP831A", dialect: Dialect::Plain, extra: Extra::None, table: RIGOL_DP800_CMD, outputs: &[(0.0, 8.0, 5.0), (0.0, 30.0, 2.0), (-30.0, 0.0, 2.0)] },
    Kind { idn: "RIGOL TECHNOLOGIES", model: "DP832", dialect: Dialect::Plain, extra: Extra::None, table: RIGOL_DP800_CMD, outputs: &[(0.0, 30.0, 3.0), (0.0, 30.0, 3.0), (0.0, 5.0, 3.0)] },
    Kind { idn: "RIGOL TECHNOLOGIES", model: "DP832A", dialect: Dialect::Plain, extra: Extra::None, table: RIGOL_DP800_CMD, outputs: &[(0.0, 30.0, 3.0), (0.0, 30.0, 3.0), (0.0, 5.0, 3.0)] },
    Kind { idn: "OWON", model: "P4305", dialect: Dialect::Plain, extra: Extra::None, table: OWON_P4000_CMD, outputs: &[(0.0, 30.0, 5.0)] },
    Kind { idn: "OWON", model: "SPE6103", dialect: Dialect::Plain, extra: Extra::None, table: OWON_SPE6103_CMD, outputs: &[(0.0, 60.0, 10.0)] },
    Kind { idn: "PHILIPS", model: "PM2813/11", dialect: Dialect::Philips, extra: Extra::None, table: PHILIPS_PM2800_CMD, outputs: &[(0.0, 60.0, 5.0), (0.0, 60.0, 5.0), (0.0, 60.0, 5.0)] },
    Kind { idn: "PHILIPS", model: "PM2832/01", dialect: Dialect::Philips, extra: Extra::None, table: PHILIPS_PM2800_CMD, outputs: &[(0.0, 120.0, 1.0), (0.0, 120.0, 1.0)] },
    Kind { idn: "Rohde&Schwarz", model: "HMC8042", dialect: Dialect::Plain, extra: Extra::None, table: RS_HMC8043_CMD, outputs: &[C3, C3] },
    Kind { idn: "Rohde&Schwarz", model: "HMC8043", dialect: Dialect::Plain, extra: Extra::None, table: RS_HMC8043_CMD, outputs: &[C3, C3, C3] },
    Kind { idn: "Rohde&Schwarz", model: "NGE103B", dialect: Dialect::Plain, extra: Extra::None, table: RS_NGE100B_CMD, outputs: &[C3, C3, C3] },
    Kind { idn: "HAMEG", model: "HMP4030", dialect: Dialect::Hmp, extra: Extra::None, table: RS_HMP4040_CMD, outputs: &[A5, A5, A5] },
    Kind { idn: "HAMEG", model: "HMP4040", dialect: Dialect::Hmp, extra: Extra::None, table: RS_HMP4040_CMD, outputs: &[A5, A5, A5, A5] },
    Kind { idn: "ROHDE&SCHWARZ", model: "HMP2020", dialect: Dialect::Hmp, extra: Extra::None, table: RS_HMP4040_CMD, outputs: &[A5, B5] },
    Kind { idn: "ROHDE&SCHWARZ", model: "HMP2030", dialect: Dialect::Hmp, extra: Extra::None, table: RS_HMP4040_CMD, outputs: &[B5, B5, B5] },
];

// The tables of profiles.c, in its order. The N5700's two with a question mark are
// written here as the manual has them, and `Supply::outside` refuses C's form.
#[rustfmt::skip]
mod tables {
    use super::A;
    pub(super) const AGILENT_N5700A_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:COMM:RLST REM"),
        (A::Local, "SYST:COMM:RLST LOC"),
        (A::GetMeasVoltage, ":MEAS:VOLT?"),
        (A::GetMeasCurrent, "MEAS:CURR?"),
        (A::GetVoltageTarget, ":SOUR:VOLT?"),
        (A::SetVoltageTarget, ":SOUR:VOLT %.6f"),
        (A::GetCurrentLimit, ":SOUR:CURR?"),
        (A::SetCurrentLimit, ":SOUR:CURR %.6f"),
        (A::GetOutputEnabled, ":OUTP:STAT?"),
        (A::SetOutputEnable, ":OUTP ON"),
        (A::SetOutputDisable, ":OUTP OFF"),
        (A::GetOverVoltageProtectionThreshold, ":VOLT:PROT?"),
        (A::SetOverVoltageProtectionThreshold, ":VOLT:PROT %.6f"),
        (A::GetOverCurrentProtectionEnabled, ":CURR:PROT:STAT?"),
        (A::SetOverCurrentProtectionEnable, ":CURR:PROT:STAT ON"),
        (A::SetOverCurrentProtectionDisable, ":CURR:PROT:STAT OFF"),
        (A::GetOverCurrentProtectionThreshold, ":SOUR:CURR?"),
        (A::SetOverCurrentProtectionThreshold, ":SOUR:CURR %.6f"),
    ];

    pub(super) const BK_9130_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:REMOTE"),
        (A::Local, "SYST:LOCAL"),
        (A::SelectChannel, ":INST:NSEL %s"),
        (A::GetMeasVoltage, ":MEAS:VOLT?"),
        (A::GetMeasCurrent, ":MEAS:CURR?"),
        (A::GetMeasPower, ":MEAS:POWER?"),
        (A::GetVoltageTarget, ":SOUR:VOLT?"),
        (A::SetVoltageTarget, ":SOUR:VOLT %.6f"),
        (A::GetCurrentLimit, ":SOUR:CURR?"),
        (A::SetCurrentLimit, ":SOUR:CURR %.6f"),
        (A::GetOutputEnabled, ":OUTP?"),
        (A::SetOutputEnable, ":OUTP 1"),
        (A::SetOutputDisable, ":OUTP 0"),
        (A::GetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT?"),
        (A::SetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT %.6f"),
    ];

    pub(super) const CHROMA_61604_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:REM"),
        (A::Local, "SYST:LOC"),
        (A::GetMeasVoltage, ":FETC:VOLT:ACDC?"),
        (A::GetMeasFrequency, ":FETC:FREQ?"),
        (A::GetMeasCurrent, ":FETC:CURR:AC?"),
        (A::GetMeasPower, ":FETC:POW:AC?"),
        (A::GetVoltageTarget, ":SOUR:VOLT:AC?"),
        (A::SetVoltageTarget, ":SOUR:VOLT:AC %.1f"),
        (A::GetFrequencyTarget, ":SOUR:FREQ?"),
        (A::SetFrequencyTarget, ":SOUR:FREQ %.2f"),
        (A::GetOutputEnabled, ":OUTP?"),
        (A::SetOutputEnable, ":OUTP ON"),
        (A::SetOutputDisable, ":OUTP OFF"),
        (A::GetOverVoltageProtectionThreshold, ":SOUR:VOLT:LIM:AC?"),
        (A::SetOverVoltageProtectionThreshold, ":SOUR:VOLT:LIM:AC %.1f"),
        (A::GetOverCurrentProtectionThreshold, ":SOUR:CURR:LIM?"),
        (A::SetOverCurrentProtectionThreshold, ":SOUR:CURR:LIM %.2f"),
    ];

    pub(super) const CHROMA_62000_CMD: &[(A, &str)] = &[
        (A::Remote, ":CONF:REM ON"),
        (A::Local, ":CONF:REM OFF"),
        (A::Beeper, ":CONF:BEEP?"),
        (A::BeeperEnable, ":CONF:BEEP ON"),
        (A::BeeperDisable, ":CONF:BEEP OFF"),
        (A::GetMeasVoltage, ":MEAS:VOLT?"),
        (A::GetMeasCurrent, ":MEAS:CURR?"),
        (A::GetMeasPower, ":MEAS:POW?"),
        (A::GetVoltageTarget, ":SOUR:VOLT?"),
        (A::SetVoltageTarget, ":SOUR:VOLT %.2f"),
        (A::GetCurrentLimit, ":SOUR:CURR?"),
        (A::SetCurrentLimit, ":SOUR:CURR %.6f"),
        (A::GetOutputEnabled, ":CONF:OUTP?"),
        (A::SetOutputEnable, ":CONF:OUTP ON"),
        (A::SetOutputDisable, ":CONF:OUTP OFF"),
        (A::GetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT:HIGH?"),
        (A::SetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT:HIGH %.6f"),
        (A::GetOverCurrentProtectionThreshold, ":SOUR:CURR:PROT:HIGH?"),
        (A::SetOverCurrentProtectionThreshold, ":SOUR:CURR:PROT:HIGH %.6f"),
    ];

    pub(super) const EA_PS_20XX_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:LOCK 1"),
        (A::Local, "SYST:LOCK 0"),
        (A::GetMeasVoltage, ":MEAS:VOLT?"),
        (A::GetMeasCurrent, ":MEAS:CURR?"),
        (A::GetMeasPower, ":MEAS:POW?"),
        (A::GetVoltageTarget, ":VOLT?"),
        (A::SetVoltageTarget, ":VOLT %.2f"),
        (A::GetCurrentLimit, ":CURR?"),
        (A::SetCurrentLimit, ":CURR %.2f"),
        (A::GetOutputEnabled, ":OUTP?"),
        (A::SetOutputEnable, ":OUTP 1"),
        (A::SetOutputDisable, ":OUTP 0"),
        (A::SetOverVoltageProtectionThreshold, ":VOLT:PROT %.2f"),
        (A::SetOverCurrentProtectionThreshold, ":CURR:PROT %.2f"),
    ];

    pub(super) const EEZ_PSU_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:REMOTE"),
        (A::Local, "SYST:LOCAL"),
        (A::SelectChannel, ":INST:NSEL %s"),
        (A::GetMeasVoltage, ":MEAS:VOLT?"),
        (A::GetMeasCurrent, ":MEAS:CURR?"),
        (A::GetMeasPower, ":MEAS:POWER?"),
        (A::GetOutputRegulation, ":OUTP:MODE?"),
        (A::GetVoltageTarget, ":SOUR:VOLT?"),
        (A::SetVoltageTarget, ":SOUR:VOLT %.2f"),
        (A::GetCurrentLimit, ":SOUR:CURR?"),
        (A::SetCurrentLimit, ":SOUR:CURR %.6f"),
        (A::GetOutputEnabled, ":OUTP?"),
        (A::SetOutputEnable, ":OUTP ON"),
        (A::SetOutputDisable, ":OUTP OFF"),
        (A::GetOverCurrentProtectionThreshold, ":SOUR:CURR:PROT?"),
        (A::GetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT?"),
    ];

    pub(super) const RIGOL_DP700_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:REMOTE"),
        (A::Local, "SYST:LOCAL"),
        (A::Beeper, "SYST:BEEP:STAT?"),
        (A::BeeperEnable, "SYST:BEEP:STAT ON"),
        (A::BeeperDisable, "SYST:BEEP:STAT OFF"),
        (A::SelectChannel, ":INST:NSEL %s"),
        (A::GetMeasVoltage, ":MEAS:VOLT?"),
        (A::GetMeasCurrent, ":MEAS:CURR?"),
        (A::GetMeasPower, ":MEAS:POWE?"),
        (A::GetVoltageTarget, ":SOUR:VOLT?"),
        (A::SetVoltageTarget, ":SOUR:VOLT %.6f"),
        (A::GetCurrentLimit, ":SOUR:CURR?"),
        (A::SetCurrentLimit, ":SOUR:CURR %.6f"),
        (A::GetOutputEnabled, ":OUTP?"),
        (A::SetOutputEnable, ":OUTP ON"),
        (A::SetOutputDisable, ":OUTP OFF"),
        (A::GetOutputRegulation, ":OUTP:MODE?"),
        (A::GetOverVoltageProtectionEnabled, ":OUTP:OVP?"),
        (A::SetOverVoltageProtectionEnable, ":OUTP:OVP ON"),
        (A::SetOverVoltageProtectionDisable, ":OUTP:OVP OFF"),
        (A::GetOverVoltageProtectionActive, ":OUTP:OVP:QUES?"),
        (A::GetOverVoltageProtectionThreshold, ":OUTP:OVP:VAL?"),
        (A::SetOverVoltageProtectionThreshold, ":OUTP:OVP:VAL %.6f"),
        (A::GetOverCurrentProtectionEnabled, ":OUTP:OCP?"),
        (A::SetOverCurrentProtectionEnable, ":OUTP:OCP:STAT ON"),
        (A::SetOverCurrentProtectionDisable, ":OUTP:OCP:STAT OFF"),
        (A::GetOverCurrentProtectionActive, ":OUTP:OCP:QUES?"),
        (A::GetOverCurrentProtectionThreshold, ":OUTP:OCP:VAL?"),
        (A::SetOverCurrentProtectionThreshold, ":OUTP:OCP:VAL %.6f"),
    ];

    pub(super) const RIGOL_DP800_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:REMOTE"),
        (A::Local, "SYST:LOCAL"),
        (A::Beeper, "SYST:BEEP:STAT?"),
        (A::BeeperEnable, "SYST:BEEP:STAT ON"),
        (A::BeeperDisable, "SYST:BEEP:STAT OFF"),
        (A::SelectChannel, ":INST:NSEL %s"),
        (A::GetMeasVoltage, ":MEAS:VOLT?"),
        (A::GetMeasCurrent, ":MEAS:CURR?"),
        (A::GetMeasPower, ":MEAS:POWE?"),
        (A::GetVoltageTarget, ":SOUR:VOLT?"),
        (A::SetVoltageTarget, ":SOUR:VOLT %.6f"),
        (A::GetCurrentLimit, ":SOUR:CURR?"),
        (A::SetCurrentLimit, ":SOUR:CURR %.6f"),
        (A::GetOutputEnabled, ":OUTP?"),
        (A::SetOutputEnable, ":OUTP ON"),
        (A::SetOutputDisable, ":OUTP OFF"),
        (A::GetOutputRegulation, ":OUTP:MODE?"),
        (A::GetOverTemperatureProtection, ":SYST:OTP?"),
        (A::SetOverTemperatureProtectionEnable, ":SYST:OTP ON"),
        (A::SetOverTemperatureProtectionDisable, ":SYST:OTP OFF"),
        (A::GetOverVoltageProtectionEnabled, ":OUTP:OVP?"),
        (A::SetOverVoltageProtectionEnable, ":OUTP:OVP ON"),
        (A::SetOverVoltageProtectionDisable, ":OUTP:OVP OFF"),
        (A::GetOverVoltageProtectionActive, ":OUTP:OVP:QUES?"),
        (A::GetOverVoltageProtectionThreshold, ":OUTP:OVP:VAL?"),
        (A::SetOverVoltageProtectionThreshold, ":OUTP:OVP:VAL %.6f"),
        (A::GetOverCurrentProtectionEnabled, ":OUTP:OCP?"),
        (A::SetOverCurrentProtectionEnable, ":OUTP:OCP:STAT ON"),
        (A::SetOverCurrentProtectionDisable, ":OUTP:OCP:STAT OFF"),
        (A::GetOverCurrentProtectionActive, ":OUTP:OCP:QUES?"),
        (A::GetOverCurrentProtectionThreshold, ":OUTP:OCP:VAL?"),
        (A::SetOverCurrentProtectionThreshold, ":OUTP:OCP:VAL %.6f"),
    ];

    pub(super) const HP_6630A_CMD: &[(A, &str)] = &[
        (A::SetOutputEnable, "OUT 1"),
        (A::SetOutputDisable, "OUT 0"),
        (A::GetMeasVoltage, "VOUT?"),
        (A::GetMeasCurrent, "IOUT?"),
        (A::SetVoltageTarget, "VSET %.4f"),
        (A::SetCurrentLimit, "ISET %.4f"),
        (A::GetOverVoltageProtectionActive, "STS?"),
        (A::SetOverVoltageProtectionThreshold, "OVSET %.4f"),
        (A::SetOverCurrentProtectionEnable, "OCP 1"),
        (A::SetOverCurrentProtectionDisable, "OCP 0"),
        (A::GetOverCurrentProtectionActive, "STS?"),
        (A::GetOverTemperatureProtectionActive, "STS?"),
        (A::GetOutputRegulation, "STS?"),
    ];

    pub(super) const HP_6630B_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:REM"),
        (A::Local, "SYST:LOC"),
        (A::GetOutputEnabled, "OUTP:STAT?"),
        (A::SetOutputEnable, "OUTP:STAT ON"),
        (A::SetOutputDisable, "OUTP:STAT OFF"),
        (A::GetMeasVoltage, ":MEAS:VOLT?"),
        (A::GetMeasCurrent, ":MEAS:CURR?"),
        (A::GetVoltageTarget, ":SOUR:VOLT?"),
        (A::SetVoltageTarget, ":SOUR:VOLT %.6f"),
        (A::GetCurrentLimit, ":SOUR:CURR?"),
        (A::SetCurrentLimit, ":SOUR:CURR %.6f"),
        (A::GetOverCurrentProtectionEnabled, ":CURR:PROT:STAT?"),
        (A::SetOverCurrentProtectionEnable, ":CURR:PROT:STAT 1"),
        (A::SetOverCurrentProtectionDisable, ":CURR:PROT:STAT 0"),
        (A::GetOverCurrentProtectionActive, "STAT:QUES:COND?"),
        (A::GetOverVoltageProtectionActive, "STAT:QUES:COND?"),
        (A::GetOverVoltageProtectionThreshold, ":VOLT:PROT?"),
        (A::SetOverVoltageProtectionThreshold, ":VOLT:PROT %.6f"),
        (A::GetOverTemperatureProtectionActive, "STAT:QUES:COND?"),
        (A::GetOutputRegulation, "STAT:OPER:COND?"),
    ];

    pub(super) const KEYSIGHT_E36300A_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:REM"),
        (A::Local, "SYST:LOC"),
        (A::SelectChannel, ":INST:NSEL %s"),
        (A::GetOutputEnabled, "OUTP:STAT?"),
        (A::SetOutputEnable, "OUTP:STAT ON"),
        (A::SetOutputDisable, "OUTP:STAT OFF"),
        (A::GetMeasVoltage, ":MEAS:VOLT?"),
        (A::GetMeasCurrent, ":MEAS:CURR?"),
        (A::GetVoltageTarget, ":SOUR:VOLT?"),
        (A::SetVoltageTarget, ":SOUR:VOLT %.6f"),
        (A::GetCurrentLimit, ":SOUR:CURR?"),
        (A::SetCurrentLimit, ":SOUR:CURR %.6f"),
        (A::GetOverCurrentProtectionEnabled, ":CURR:PROT:STAT?"),
        (A::SetOverCurrentProtectionEnable, ":CURR:PROT:STAT 1"),
        (A::SetOverCurrentProtectionDisable, ":CURR:PROT:STAT 0"),
        (A::GetOverCurrentProtectionActive, ":CURR:PROT:TRIP?"),
        (A::GetOverVoltageProtectionActive, ":VOLT:PROT:TRIP?"),
        (A::GetOverVoltageProtectionThreshold, ":VOLT:PROT?"),
        (A::SetOverVoltageProtectionThreshold, ":VOLT:PROT %.6f"),
        (A::GetOverTemperatureProtectionActive, ":STAT:QUES:INST:ISUM%s:COND?"),
        (A::GetOutputRegulation, ":STAT:QUES:INST:ISUM%s:COND?"),
    ];

    pub(super) const OWON_P4000_CMD: &[(A, &str)] = &[
        (A::GetMeasVoltage, "MEAS:VOLT?"),
        (A::GetMeasCurrent, "MEAS:CURR?"),
        (A::GetMeasPower, "MEAS:POW?"),
        (A::GetVoltageTarget, "VOLT?"),
        (A::SetVoltageTarget, "VOLT %.6f"),
        (A::GetCurrentLimit, "CURR?"),
        (A::SetCurrentLimit, "CURR %.6f"),
        (A::GetOutputEnabled, "OUTP?"),
        (A::SetOutputEnable, "OUTP 1"),
        (A::SetOutputDisable, "OUTP 0"),
        (A::GetOverVoltageProtectionThreshold, "VOLT:LIM?"),
        (A::SetOverVoltageProtectionThreshold, "VOLT:LIM %.6f"),
        (A::GetOverCurrentProtectionThreshold, "CURR:LIM?"),
        (A::SetOverCurrentProtectionThreshold, "CURR:LIM %.6f"),
    ];

    pub(super) const OWON_SPE6103_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:REM"),
        (A::Local, "SYST:LOC"),
        (A::GetMeasVoltage, "MEAS:VOLT?"),
        (A::GetMeasCurrent, "MEAS:CURR?"),
        (A::GetMeasPower, "MEAS:POW?"),
        (A::GetVoltageTarget, "VOLT?"),
        (A::SetVoltageTarget, "VOLT %.6f"),
        (A::GetCurrentLimit, "CURR?"),
        (A::SetCurrentLimit, "CURR %.6f"),
        (A::GetOutputEnabled, "OUTP?"),
        (A::SetOutputEnable, "OUTP 1"),
        (A::SetOutputDisable, "OUTP 0"),
        (A::GetOverVoltageProtectionThreshold, "VOLT:LIM?"),
        (A::SetOverVoltageProtectionThreshold, "VOLT:LIM %.6f"),
        (A::GetOverCurrentProtectionThreshold, "CURR:LIM?"),
        (A::SetOverCurrentProtectionThreshold, "CURR:LIM %.6f"),
    ];

    pub(super) const PHILIPS_PM2800_CMD: &[(A, &str)] = &[
        (A::SelectChannel, ":INST:NSEL %s"),
        (A::GetMeasVoltage, ":MEAS:VOLT?"),
        (A::GetMeasCurrent, ":MEAS:CURR?"),
        (A::GetVoltageTarget, ":SOUR:VOLT?"),
        (A::SetVoltageTarget, ":SOUR:VOLT %.6f"),
        (A::GetCurrentLimit, ":SOUR:CURR?"),
        (A::SetCurrentLimit, ":SOUR:CURR %.6f"),
        (A::GetOutputEnabled, ":OUTP?"),
        (A::SetOutputEnable, ":OUTP ON"),
        (A::SetOutputDisable, ":OUTP OFF"),
        (A::GetOutputRegulation, ":SOUR:FUNC:MODE?"),
        (A::GetOverVoltageProtectionActive, ":SOUR:VOLT:PROT:TRIP?"),
        (A::GetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT:LEV?"),
        (A::SetOverVoltageProtectionThreshold, ":SOUR:VOLT:PROT:LEV %.6f"),
        (A::GetOverCurrentProtectionEnabled, ":SOUR:CURR:PROT:STAT?"),
        (A::SetOverCurrentProtectionEnable, ":SOUR:CURR:PROT:STAT ON"),
        (A::SetOverCurrentProtectionDisable, ":SOUR:CURR:PROT:STAT OFF"),
        (A::GetOverCurrentProtectionActive, ":SOUR:CURR:PROT:TRIP?"),
    ];

    pub(super) const RS_HMC8043_CMD: &[(A, &str)] = &[
        (A::SelectChannel, "INST:NSEL %s"),
        (A::GetMeasVoltage, "MEAS:VOLT?"),
        (A::GetMeasCurrent, "MEAS:CURR?"),
        (A::GetVoltageTarget, "VOLT?"),
        (A::SetVoltageTarget, "VOLT %.6f"),
        (A::GetCurrentLimit, "CURR?"),
        (A::SetCurrentLimit, "CURR %.6f"),
        (A::GetOutputEnabled, "OUTP?"),
        (A::SetOutputEnable, "OUTP ON"),
        (A::SetOutputDisable, "OUTP OFF"),
        (A::GetOverVoltageProtectionActive, "VOLT:PROT:TRIP?"),
        (A::GetOverVoltageProtectionThreshold, "VOLT:PROT:LEV?"),
        (A::SetOverVoltageProtectionThreshold, "VOLT:PROT:LEV %.6f"),
        (A::GetOverVoltageProtectionEnabled, "VOLT:PROT:STAT?"),
        (A::SetOverVoltageProtectionEnable, "VOLT:PROT:STAT ON"),
        (A::SetOverVoltageProtectionDisable, "VOLT:PROT:STAT OFF"),
        (A::GetOverCurrentProtectionActive, "FUSE:TRIP?"),
        (A::GetOverCurrentProtectionEnabled, "FUSE:STAT?"),
        (A::SetOverCurrentProtectionEnable, "FUSE:STAT ON"),
        (A::SetOverCurrentProtectionDisable, "FUSE:STAT OFF"),
        (A::GetOverCurrentProtectionDelay, "FUSE:DEL?"),
        (A::SetOverCurrentProtectionDelay, "FUSE:DEL %.03f"),
    ];

    pub(super) const RS_NGE100B_CMD: &[(A, &str)] = &[
        (A::SelectChannel, "INST:NSEL %s"),
        (A::GetMeasVoltage, "MEAS:VOLT?"),
        (A::GetMeasCurrent, "MEAS:CURR?"),
        (A::GetVoltageTarget, "VOLT?"),
        (A::SetVoltageTarget, "VOLT %.6f"),
        (A::GetCurrentLimit, "CURR?"),
        (A::SetCurrentLimit, "CURR %.6f"),
        (A::GetOutputEnabled, "OUTP?"),
        (A::SetOutputEnable, "OUTP ON"),
        (A::SetOutputDisable, "OUTP OFF"),
        (A::GetOverVoltageProtectionActive, "VOLT:PROT:TRIP?"),
        (A::GetOverVoltageProtectionThreshold, "VOLT:PROT:LEV?"),
        (A::SetOverVoltageProtectionThreshold, "VOLT:PROT:LEV %.6f"),
        (A::GetOverVoltageProtectionEnabled, "VOLT:PROT:STAT?"),
        (A::SetOverVoltageProtectionEnable, "VOLT:PROT:STAT ON"),
        (A::SetOverVoltageProtectionDisable, "VOLT:PROT:STAT OFF"),
        (A::GetOverCurrentProtectionActive, "FUSE:TRIP?"),
        (A::GetOverCurrentProtectionEnabled, "FUSE:STAT?"),
        (A::SetOverCurrentProtectionEnable, "FUSE:STAT ON"),
        (A::SetOverCurrentProtectionDisable, "FUSE:STAT OFF"),
        (A::GetOverCurrentProtectionDelay, "FUSE:DEL?"),
        (A::SetOverCurrentProtectionDelay, "FUSE:DEL %.03f"),
    ];

    pub(super) const RS_HMP4040_CMD: &[(A, &str)] = &[
        (A::Remote, "SYST:REM"),
        (A::Local, "SYST:LOC"),
        (A::SelectChannel, "INST:NSEL %s"),
        (A::GetMeasVoltage, "MEAS:VOLT?"),
        (A::GetMeasCurrent, "MEAS:CURR?"),
        (A::GetVoltageTarget, "VOLT?"),
        (A::SetVoltageTarget, "VOLT %.6f"),
        (A::GetCurrentLimit, "CURR?"),
        (A::SetCurrentLimit, "CURR %.6f"),
        (A::GetOutputEnabled, "OUTP?"),
        (A::SetOutputEnable, "OUTP ON"),
        (A::SetOutputDisable, "OUTP OFF"),
        (A::GetOutputRegulation, "STAT:QUES:INST:ISUM%s:COND?"),
        (A::GetOverVoltageProtectionActive, "VOLT:PROT:TRIP?"),
        (A::GetOverVoltageProtectionThreshold, "VOLT:PROT:LEV?"),
        (A::SetOverVoltageProtectionThreshold, "VOLT:PROT:LEV %.6f"),
        (A::GetOverTemperatureProtectionActive, "STAT:QUES:INST:ISUM%s:COND?"),
    ];
}
use tables::{
    AGILENT_N5700A_CMD, BK_9130_CMD, CHROMA_61604_CMD, CHROMA_62000_CMD, EA_PS_20XX_CMD, EEZ_PSU_CMD, RIGOL_DP700_CMD, RIGOL_DP800_CMD, HP_6630A_CMD, HP_6630B_CMD, KEYSIGHT_E36300A_CMD, OWON_P4000_CMD, OWON_SPE6103_CMD, PHILIPS_PM2800_CMD, RS_HMC8043_CMD, RS_NGE100B_CMD, RS_HMP4040_CMD,
};

/// The state an output is in.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
struct Cond {
    cv: bool,
    cc: bool,
    ccn: bool,
    ur: bool,
    ovp: bool,
    ocp: bool,
    otp: bool,
}

#[derive(Clone, Copy, Debug)]
#[allow(clippy::struct_excessive_bools)]
struct Output {
    v: f64,
    i: f64,
    f: f64,
    on: bool,
    ovp_on: bool,
    ovp: f64,
    ocp_on: bool,
    ocp: f64,
    ocp_delay: f64,
    cond: Cond,
}

/// A supply.
pub(super) struct Supply {
    kind: &'static Kind,
    selected: usize,
    outputs: Vec<Output>,
    beeper: bool,
    otp_on: bool,
    load: f64,
    readings: u32,
    /// `event-at=`: the reading it comes at, and what.
    events: Vec<(u32, String)>,
    forced: bool,
    /// `STAT:OPER:EVEN?`, `STAT:QUES:EVEN?` and the rest of a family's event words.
    oper_event: u32,
    ques_event: u32,
    fault: u32,
    oper_enabled: bool,
    ques_enabled: bool,
    channels: Option<usize>,
    reg_text: Option<String>,
    status_text: Option<String>,
    bad_at: Option<(u32, String)>,
    bad: Option<String>,
    say: Option<String>,
    hpid: Option<String>,
    rom: Option<String>,
    nominal: Option<String>,
    chan_info: Option<String>,
}

/// A table's text against what came: `None` if it is another command, else the `%s`
/// in its header if there is one.
#[allow(clippy::option_option)]
fn matches(template: &str, m: &Message<'_>) -> Option<Option<String>> {
    let upper = template.strip_prefix(':').unwrap_or(template).to_ascii_uppercase();
    let (header, args) = upper.split_once(' ').unwrap_or((upper.as_str(), ""));
    if let Some((head, tail)) = header.split_once("%S") {
        let inner = m.header.strip_prefix(head)?.strip_suffix(tail)?;
        return Some(Some(inner.to_owned()));
    }
    if header != m.header {
        return None;
    }
    if args.contains('%') {
        return (!m.args.is_empty()).then_some(None);
    }
    (args.eq_ignore_ascii_case(m.args)).then_some(None)
}

impl Supply {
    fn fmt(&self, value: f64) -> String {
        match self.kind.dialect {
            Dialect::Hp66xxb => nr3(value),
            Dialect::HpComp => format!("{value:.3}"),
            _ if self.kind.model.starts_with('N') => nr3(value),
            _ => format!("{value:.4}"),
        }
    }

    fn output(&self) -> &Output {
        &self.outputs[self.selected]
    }

    fn output_mut(&mut self) -> &mut Output {
        &mut self.outputs[self.selected]
    }

    /// The conditions an output has of itself, with its load; what `event-at=` set
    /// takes over.
    fn settle(&mut self) {
        if self.forced {
            return;
        }
        let load = self.load;
        for o in &mut self.outputs {
            let cc = o.on && o.v.abs() / load > o.i;
            let keep = (o.cond.ovp, o.cond.ocp, o.cond.otp);
            o.cond = Cond { cv: o.on && !cc, cc, ..Cond::default() };
            (o.cond.ovp, o.cond.ocp, o.cond.otp) = keep;
        }
    }

    /// One reading more, and what it brings.
    fn tick(&mut self) -> u32 {
        self.readings += 1;
        let n = self.readings;
        let due: Vec<String> =
            self.events.iter().filter(|(at, _)| *at == n).map(|(_, w)| w.clone()).collect();
        for what in due {
            let before: Vec<Cond> = self.outputs.iter().map(|o| o.cond).collect();
            self.forced = true;
            for o in &mut self.outputs {
                let c = &mut o.cond;
                match what.as_str() {
                    "cv" => (c.cv, c.cc, c.ccn, c.ur) = (true, false, false, false),
                    "cc" => (c.cv, c.cc, c.ccn, c.ur) = (false, true, false, false),
                    "ccn" => (c.cv, c.cc, c.ccn, c.ur) = (false, false, true, false),
                    "ur" => (c.cv, c.cc, c.ccn, c.ur) = (false, false, false, true),
                    "idle" => (c.cv, c.cc, c.ccn, c.ur) = (false, false, false, false),
                    "mixed" => (c.cv, c.cc, c.ccn, c.ur) = (true, true, false, false),
                    "ovp" => c.ovp = true,
                    "ocp" => c.ocp = true,
                    "otp" => c.otp = true,
                    "clear" => (c.ovp, c.ocp, c.otp) = (false, false, false),
                    _ => {}
                }
            }
            self.record(&before);
        }
        n
    }

    /// The event words' transitions, of the first output.
    fn record(&mut self, before: &[Cond]) {
        let (old, new) = (before[0], self.outputs[0].cond);
        let flip = |a: bool, b: bool, bit: u32| if a == b { 0 } else { 1 << bit };
        self.oper_event |=
            flip(old.cv, new.cv, 8) | flip(old.cc, new.cc, 10) | flip(old.ccn, new.ccn, 11);
        self.ques_event |= flip(old.ovp, new.ovp, 0)
            | flip(old.ocp, new.ocp, 1)
            | flip(old.otp, new.otp, 4)
            | flip(old.ur, new.ur, 10);
        // The FAULT register and an EA's: rising edges.
        let rise = |a: bool, b: bool, bit: u32| if !a && b { 1 << bit } else { 0 };
        self.fault |= rise(old.cv, new.cv, 0)
            | rise(old.cc, new.cc, 1)
            | rise(old.ur, new.ur, 2)
            | rise(old.ovp, new.ovp, 3)
            | rise(old.otp, new.otp, 4)
            | rise(old.ocp, new.ocp, 6)
            | rise(old.ccn, new.ccn, 9);
    }

    fn reading(&mut self, value: f64) -> Answer {
        let n = self.tick();
        if let Some((at, text)) = &self.bad_at
            && *at == n
        {
            return Answer::Text(text.clone());
        }
        if let Some(text) = &self.bad {
            return Answer::Text(text.clone());
        }
        let noise = f64::from(n % 8) * 0.001;
        Answer::Text(self.fmt(value + noise))
    }

    fn sts(c: Cond) -> u32 {
        u32::from(c.cv)
            | u32::from(c.cc) << 1
            | u32::from(c.ur) << 2
            | u32::from(c.ovp) << 3
            | u32::from(c.otp) << 4
            | u32::from(c.ocp) << 6
            | u32::from(c.ccn) << 9
    }

    fn oper_cond(c: Cond) -> u32 {
        u32::from(c.cv) << 8 | u32::from(c.cc) << 10 | u32::from(c.ccn) << 11
    }

    fn ques_cond(c: Cond) -> u32 {
        u32::from(c.ovp) | u32::from(c.ocp) << 1 | u32::from(c.otp) << 4 | u32::from(c.ur) << 10
    }

    fn word(&self, value: u32) -> Answer {
        Answer::Text(self.status_text.clone().unwrap_or_else(|| value.to_string()))
    }

    fn yes_no(value: bool) -> Answer {
        Answer::Text(if value { "YES" } else { "NO" }.to_owned())
    }

    fn flag(value: bool) -> Answer {
        Answer::Text(if value { "1" } else { "0" }.to_owned())
    }

    /// A new value for a set point: refused beyond the range.
    fn value(m: &Message<'_>, low: f64, high: f64) -> std::result::Result<f64, Answer> {
        let Ok(v) = m.args.parse::<f64>() else {
            return Err(Answer::Error(-104, "Data type error".to_owned()));
        };
        let span = 0.1 * high.abs().max(low.abs());
        if !v.is_finite() || v < low - span || v > high + span {
            return Err(Answer::Error(-222, "Data out of range".to_owned()));
        }
        Ok(v)
    }

    /// `..ISUMn:COND?`: bit 0 CC, bit 1 CV, bit 4 OTP, of output `n`.
    fn isum(&self, name: Option<String>) -> Answer {
        let Some(c) = name
            .and_then(|n| n.parse::<usize>().ok())
            .and_then(|n| self.outputs.get(n.wrapping_sub(1)))
            .map(|o| o.cond)
        else {
            return Answer::Error(-222, "Data out of range".to_owned());
        };
        if let Some(text) = &self.reg_text {
            return Answer::Text(text.clone());
        }
        self.word(u32::from(c.cc) | u32::from(c.cv) << 1 | u32::from(c.otp) << 4)
    }

    fn regulation(&self, name: Option<String>) -> Answer {
        let c = self.output().cond;
        match self.kind.dialect {
            Dialect::HpComp => {
                if let Some(text) = &self.reg_text {
                    return Answer::Text(text.clone());
                }
                self.word(Self::sts(c))
            }
            Dialect::Hp66xxb => {
                if let Some(text) = &self.reg_text {
                    return Answer::Text(text.clone());
                }
                self.word(Self::oper_cond(c))
            }
            Dialect::Hmp => self.isum(name),
            Dialect::Philips | Dialect::Plain => {
                if let Some(text) = &self.reg_text {
                    return Answer::Text(text.clone());
                }
                let philips = self.kind.dialect == Dialect::Philips;
                let word = match (c.cv, c.cc, c.ccn) {
                    (true, _, _) if philips => "VOLT",
                    (_, true, _) if philips => "CURR",
                    (true, _, _) => "CV",
                    (_, true, _) => "CC",
                    (_, _, true) => "CC-",
                    _ => "UR",
                };
                Answer::Text(word.to_owned())
            }
        }
    }

    fn active(&self, trip: bool, name: Option<String>) -> Answer {
        match self.kind.dialect {
            Dialect::HpComp => self.word(Self::sts(self.output().cond)),
            Dialect::Hp66xxb => self.word(Self::ques_cond(self.output().cond)),
            Dialect::Hmp if name.is_some() => self.isum(name),
            Dialect::Plain if self.kind.table.as_ptr() == RIGOL_DP800_CMD.as_ptr() => {
                Self::yes_no(trip)
            }
            Dialect::Plain if self.kind.table.as_ptr() == RIGOL_DP700_CMD.as_ptr() => {
                Self::yes_no(trip)
            }
            _ => Self::flag(trip),
        }
    }

    #[allow(clippy::too_many_lines)]
    fn act(&mut self, action: A, name: Option<String>, m: &Message<'_>) -> Answer {
        let span = self.kind.outputs[self.selected];
        match action {
            A::Remote | A::Local => Answer::Done,
            A::Beeper => Self::flag(self.beeper),
            A::BeeperEnable => {
                self.beeper = true;
                Answer::Done
            }
            A::BeeperDisable => {
                self.beeper = false;
                Answer::Done
            }
            A::SelectChannel => match m.args.parse::<usize>() {
                Ok(n) if (1..=self.outputs.len()).contains(&n) => {
                    self.selected = n - 1;
                    Answer::Done
                }
                _ => Answer::Error(-222, "Data out of range".to_owned()),
            },
            A::GetMeasVoltage | A::GetMeasCurrent | A::GetMeasPower | A::GetMeasFrequency => {
                self.settle();
                let o = *self.output();
                let volts = if o.on { o.v } else { 0.0 };
                let amps = if o.on { (o.v.abs() / self.load).min(o.i) } else { 0.0 };
                let value = match action {
                    A::GetMeasVoltage => volts.min(amps * self.load).copysign(volts),
                    A::GetMeasCurrent => amps,
                    A::GetMeasPower => volts.abs() * amps,
                    _ => {
                        if o.on {
                            o.f
                        } else {
                            0.0
                        }
                    }
                };
                self.reading(value)
            }
            A::GetVoltageTarget => Answer::Text(self.fmt(self.output().v)),
            A::GetCurrentLimit => Answer::Text(self.fmt(self.output().i)),
            A::GetFrequencyTarget => Answer::Text(self.fmt(self.output().f)),
            A::GetOverVoltageProtectionThreshold => Answer::Text(self.fmt(self.output().ovp)),
            A::GetOverCurrentProtectionThreshold => Answer::Text(self.fmt(self.output().ocp)),
            A::GetOverCurrentProtectionDelay => Answer::Text(self.fmt(self.output().ocp_delay)),
            A::SetVoltageTarget => match Self::value(m, span.0, span.1) {
                Ok(v) => {
                    self.output_mut().v = v;
                    Answer::Done
                }
                Err(a) => a,
            },
            A::SetCurrentLimit => match Self::value(m, 0.0, span.2) {
                Ok(v) => {
                    self.output_mut().i = v;
                    Answer::Done
                }
                Err(a) => a,
            },
            A::SetFrequencyTarget => match Self::value(m, 1.0, 1000.0) {
                Ok(v) => {
                    self.output_mut().f = v;
                    Answer::Done
                }
                Err(a) => a,
            },
            A::SetOverVoltageProtectionThreshold => match Self::value(m, span.0, span.1) {
                Ok(v) => {
                    self.output_mut().ovp = v;
                    Answer::Done
                }
                Err(a) => a,
            },
            A::SetOverCurrentProtectionThreshold => match Self::value(m, 0.0, span.2) {
                Ok(v) => {
                    self.output_mut().ocp = v;
                    Answer::Done
                }
                Err(a) => a,
            },
            A::SetOverCurrentProtectionDelay => match Self::value(m, 0.0, 10.0) {
                Ok(v) => {
                    self.output_mut().ocp_delay = v;
                    Answer::Done
                }
                Err(a) => a,
            },
            A::GetOutputEnabled => Self::flag(self.output().on),
            A::SetOutputEnable => {
                self.output_mut().on = true;
                self.settle();
                Answer::Done
            }
            A::SetOutputDisable => {
                self.output_mut().on = false;
                self.settle();
                Answer::Done
            }
            A::GetOutputRegulation => {
                self.settle();
                self.regulation(name)
            }
            A::GetOverVoltageProtectionEnabled => Self::flag(self.output().ovp_on),
            A::SetOverVoltageProtectionEnable => {
                self.output_mut().ovp_on = true;
                Answer::Done
            }
            A::SetOverVoltageProtectionDisable => {
                self.output_mut().ovp_on = false;
                Answer::Done
            }
            A::GetOverVoltageProtectionActive => self.active(self.output().cond.ovp, name),
            A::GetOverCurrentProtectionEnabled => Self::flag(self.output().ocp_on),
            A::SetOverCurrentProtectionEnable => {
                self.output_mut().ocp_on = true;
                Answer::Done
            }
            A::SetOverCurrentProtectionDisable => {
                self.output_mut().ocp_on = false;
                Answer::Done
            }
            A::GetOverCurrentProtectionActive => self.active(self.output().cond.ocp, name),
            A::GetOverTemperatureProtection => Self::flag(self.otp_on),
            A::SetOverTemperatureProtectionEnable => {
                self.otp_on = true;
                Answer::Done
            }
            A::SetOverTemperatureProtectionDisable => {
                self.otp_on = false;
                Answer::Done
            }
            A::GetOverTemperatureProtectionActive => self.active(self.output().cond.otp, name),
        }
    }

    /// What no table has: the status of the families whose capture asks for it.
    fn outside(&mut self, m: &Message<'_>) -> Answer {
        let extra = self.kind.extra;
        let dialect = self.kind.dialect;
        let c = self.output().cond;
        match (m.header.as_str(), extra, dialect) {
            ("SYST:NOM:VOLT?", Extra::Ea, _) => self.nominal(self.kind.outputs[0].1, "V"),
            ("SYST:NOM:CURR?", Extra::Ea, _) => self.nominal(self.kind.outputs[0].2, "A"),
            ("SYST:NOM:POW?", Extra::Ea, _) => self.nominal(160.0, "W"),
            ("STAT:QUES?", Extra::Ea, _) => {
                // OVP, OCP, OTP in an EA's places.
                let f = std::mem::take(&mut self.fault);
                let q = (f >> 3 & 1) | (f >> 6 & 1) << 1 | (f >> 4 & 1) << 3;
                self.word(q)
            }
            ("STAT:OPER?", Extra::Ea, _) => {
                let e = std::mem::take(&mut self.oper_event);
                let oper = (e >> 8 & 1) << 8 | (e >> 10 & 1) << 9;
                self.word(oper)
            }
            ("SYST:CHAN:COUN?", Extra::Eez, _) => {
                let n = self.channels.unwrap_or(self.outputs.len());
                Answer::Text(n.to_string())
            }
            ("SYST:CHAN:INFO:CURR?", Extra::Eez, _) => self.info(self.output_span().2),
            ("SYST:CHAN:INFO:VOLT?", Extra::Eez, _) => self.info(self.output_span().1),
            ("SYST:CHAN:INFO:POW?", Extra::Eez, _) => {
                let span = self.output_span();
                self.info(span.1 * span.2)
            }
            ("UNMASK", _, Dialect::HpComp) if m.args == "607" => Answer::Done,
            ("FAULT?", _, Dialect::HpComp) => {
                let f = std::mem::take(&mut self.fault);
                self.word(f)
            }
            ("STAT:OPER:PTR", _, Dialect::Hp66xxb) if m.args == "3328" => Answer::Done,
            ("NTR", _, Dialect::Hp66xxb) if matches!(m.args, "3328" | "1043") => Answer::Done,
            ("ENAB", _, Dialect::Hp66xxb) if m.args == "3328" => {
                self.oper_enabled = true;
                Answer::Done
            }
            ("ENAB", _, Dialect::Hp66xxb) if m.args == "1043" => {
                self.ques_enabled = true;
                Answer::Done
            }
            ("STAT:QUES:PTR", _, Dialect::Hp66xxb) if m.args == "1043" => Answer::Done,
            ("*STB?", _, Dialect::Hp66xxb) => {
                let q = self.ques_enabled && self.ques_event != 0;
                let o = self.oper_enabled && self.oper_event != 0;
                self.word(u32::from(q) << 3 | u32::from(o) << 7)
            }
            ("STAT:QUES:EVEN?", _, Dialect::Hp66xxb) => {
                let e = std::mem::take(&mut self.ques_event);
                self.word(e)
            }
            ("STAT:QUES:COND?", _, Dialect::Hp66xxb) => self.word(Self::ques_cond(c)),
            ("STAT:OPER:EVEN?", _, Dialect::Hp66xxb) => {
                let e = std::mem::take(&mut self.oper_event);
                self.word(e)
            }
            ("STAT:OPER:COND?", _, Dialect::Hp66xxb) => self.word(Self::oper_cond(c)),
            // An HP in COMP language says who it is, and any can be told to here.
            ("ID?", _, _) => match (&self.hpid, dialect) {
                (Some(text), _) => Answer::Text(text.clone()),
                (None, Dialect::HpComp) => Answer::Text(format!("HP{}", self.kind.model)),
                _ => Answer::Unknown,
            },
            ("ROM?", _, _) => match (&self.rom, dialect) {
                (Some(text), _) if text == "mute" => Answer::Silent,
                (Some(text), _) => Answer::Text(text.clone()),
                (None, Dialect::HpComp) => Answer::Text("B01 B01".to_owned()),
                _ => Answer::Unknown,
            },
            // A boolean with a question mark, `profiles.c:82-83`: no state changes.
            ("CURR:PROT:STAT", _, _) if m.text.ends_with('?') => {
                Answer::Error(-101, "Invalid character".to_owned())
            }
            _ => Answer::Unknown,
        }
    }

    fn output_span(&self) -> (f64, f64, f64) {
        self.kind.outputs[self.selected]
    }

    fn nominal(&self, value: f64, unit: &str) -> Answer {
        match &self.nominal {
            Some(text) => Answer::Text(text.clone()),
            None => Answer::Text(format!("{value:.2} {unit}")),
        }
    }

    fn info(&self, value: f64) -> Answer {
        match &self.chan_info {
            Some(text) => Answer::Text(text.clone()),
            None => Answer::Text(format!("{value:.2}")),
        }
    }
}

/// The form of a number on an HP: sign, one digit, six places, the exponent in two.
fn nr3(value: f64) -> String {
    let text = format!("{value:+.6e}");
    let (mantissa, exponent) = text.split_once('e').unwrap_or((&text, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    format!("{mantissa}E{exponent:+03}")
}

impl Instrument for Supply {
    fn idn(&self) -> Option<String> {
        if self.say.is_some() {
            return self.say.clone();
        }
        match self.kind.dialect {
            // An HP in COMP language has no `*IDN?`; `idn=` stands in where a case wants.
            Dialect::HpComp => None,
            _ => Some(format!("{},{},EMU0001,01.02.03", self.kind.idn, self.kind.model)),
        }
    }

    fn command(&mut self, m: &Message<'_>, _s: &mut scpi::Settings, _v: &mut Violations) -> Answer {
        let found = self
            .kind
            .table
            .iter()
            .find_map(|(action, text)| matches(text, m).map(|name| (*action, name)));
        match found {
            Some((action, name)) => self.act(action, name, m),
            None => self.outside(m),
        }
    }
}

/// Each output's set points at power on, within its range.
const VOLTS: [f64; 6] = [12.0, 5.0, 3.3, 1.8, 2.5, 9.0];
const AMPS: [f64; 6] = [1.5, 0.5, 2.0, 0.25, 1.0, 0.75];

/// A supply and its shell, from the `-o` options.
pub(super) fn parts(options: &[(String, String)]) -> Result<(Supply, scpi::Config)> {
    let mut config = scpi::Config::default();
    let mut kind = KINDS.iter().find(|k| k.model == "DP832").expect("a default");
    let (mut load, mut on, mut beeper, mut channels) = (8.0, false, true, None);
    let mut events = Vec::new();
    let (mut reg_text, mut status_text, mut bad_at, mut bad) = (None, None, None, None);
    let (mut say, mut hpid, mut rom, mut nominal, mut chan_info) = (None, None, None, None, None);
    let spaced = |text: &str| text.replace('_', " ");
    for (k, v) in options {
        if config.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => {
                kind = KINDS
                    .iter()
                    .find(|m| m.model == v.replace('_', " "))
                    .ok_or_else(|| Error::Invalid(format!("model={v}: no such model here")))?;
            }
            "load" => load = f64::from(parse_number(k, v, 1..=100_000)?),
            "on" => on = v == "1",
            "beeper" => beeper = v == "1",
            "channels" => channels = Some(parse_number(k, v, 0..=1024)? as usize),
            "event-at" => {
                for item in v.split(',') {
                    let (at, what) = item
                        .split_once(':')
                        .ok_or_else(|| Error::Invalid(format!("event-at={v}: as in 3:cc")))?;
                    events.push((parse_number(k, at, 1..=1 << 24)?, what.to_owned()));
                }
            }
            "reg" => reg_text = Some(v.clone()),
            "status" => status_text = Some(v.clone()),
            "bad-at" => {
                let (at, text) = v
                    .split_once(':')
                    .ok_or_else(|| Error::Invalid(format!("bad-at={v}: as in 3:junk")))?;
                bad_at = Some((parse_number(k, at, 1..=1 << 24)?, text.to_owned()));
            }
            "bad" => bad = Some(v.clone()),
            "hpid" => hpid = Some(v.clone()),
            "say" => say = Some(spaced(v)),
            "rom" => rom = Some(spaced(v)),
            "nominal" => nominal = Some(spaced(v)),
            "chan-info" => chan_info = Some(v.clone()),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let outputs = kind
        .outputs
        .iter()
        .enumerate()
        .map(|(n, span)| {
            let v = if span.1 <= 0.0 { -VOLTS[n].min(-span.0) } else { VOLTS[n].min(span.1) };
            Output {
                v,
                i: AMPS[n].min(span.2),
                f: 50.0,
                on,
                ovp_on: false,
                ovp: span.1.abs(),
                ocp_on: false,
                ocp: span.2,
                ocp_delay: 0.01,
                cond: Cond { cv: on, ..Cond::default() },
            }
        })
        .collect();
    let supply = Supply {
        kind,
        selected: 0,
        outputs,
        beeper,
        otp_on: true,
        load,
        readings: 0,
        events,
        forced: false,
        oper_event: 0,
        ques_event: 0,
        fault: 0,
        oper_enabled: false,
        ques_enabled: false,
        channels,
        reg_text,
        status_text,
        bad_at,
        bad,
        say,
        hpid,
        rom,
        nominal,
        chan_info,
    };
    Ok((supply, config))
}

/// A supply on USBTMC, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (supply, config) = parts(options)?;
    let id = Identity::new(0x1ab1, 0x0e11, "oxrok-emu", "SCPI power supply");
    Ok(config.usbtmc(NAME, &id, supply))
}

/// A supply on a socket, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    let (supply, config) = parts(options)?;
    Ok(config.tcp(supply))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::tcp::Port;

    fn on_socket(options: &[(&str, &str)]) -> Box<dyn Stream> {
        let pairs: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        build_tcp(&pairs).unwrap()
    }

    /// A line, and the text it makes, with no line end, and the violations.
    fn say(s: &mut Box<dyn Stream>, line: &str) -> (String, Vec<String>) {
        let mut port = Port::default();
        s.received(format!("{line}\n").as_bytes(), &mut port);
        let text = String::from_utf8(port.sent().concat()).unwrap();
        (text.trim_end().to_owned(), port.violations().drain())
    }

    fn ask(s: &mut Box<dyn Stream>, line: &str) -> String {
        let (text, flags) = say(s, line);
        assert_eq!(flags, Vec::<String>::new(), "{line}");
        text
    }

    #[test]
    fn every_table_text_is_found_by_its_own_shape() {
        for kind in KINDS {
            for (_, template) in kind.table {
                let sent = template.replace("%.6f", "1.000000").replace("%.03f", "1.000");
                let sent = sent.replace("%.4f", "1.0000").replace("%.2f", "1.00");
                let sent = sent.replace("%.1f", "1.0").replace("%s", "1");
                let mut s =
                    on_socket(&[("model", kind.model), ("hpid", "x"), ("rom", "x")]);
                let (_, flags) = say(&mut s, &sent);
                assert_eq!(flags, Vec::<String>::new(), "{} {sent}", kind.model);
            }
        }
    }

    #[test]
    fn a_selection_moves_what_follows() {
        let mut s = on_socket(&[("model", "DP832")]);
        assert_eq!(ask(&mut s, ":INST:NSEL 2"), "");
        assert_eq!(ask(&mut s, ":SOUR:VOLT 3.300000"), "");
        assert_eq!(ask(&mut s, ":SOUR:VOLT?"), "3.3000");
        ask(&mut s, ":INST:NSEL 1");
        assert_eq!(ask(&mut s, ":SOUR:VOLT?"), "12.0000");
    }

    #[test]
    fn what_it_refuses() {
        let mut s = on_socket(&[("model", "DP832")]);
        for line in [":SOUR:VOLT 999.0", ":SOUR:VOLT nan", ":INST:NSEL 4"] {
            assert_eq!(ask(&mut s, line), "", "{line}");
        }
        assert_eq!(ask(&mut s, ":SOUR:VOLT?"), "12.0000");
        assert_eq!(ask(&mut s, "SYST:ERR?"), "-222,\"Data out of range\"");
        assert_eq!(ask(&mut s, "SYST:ERR?"), "-222,\"Data out of range\"");
        let (_, flags) = say(&mut s, ":NO:SUCH");
        assert_eq!(flags.len(), 1);
    }

    #[test]
    fn the_n5700_questioned_boolean_changes_nothing() {
        let mut s = on_socket(&[("model", "N5767A")]);
        assert_eq!(ask(&mut s, ":CURR:PROT:STAT ON?"), "");
        assert_eq!(ask(&mut s, ":CURR:PROT:STAT?"), "0");
        assert_eq!(ask(&mut s, ":CURR:PROT:STAT ON"), "");
        assert_eq!(ask(&mut s, ":CURR:PROT:STAT?"), "1");
    }

    #[test]
    fn a_load_and_the_regulation() {
        let mut s = on_socket(&[("model", "DP832"), ("on", "1"), ("load", "2")]);
        // 12 V on 2 ohms wants 6 A, and the limit is 1.5.
        assert_eq!(ask(&mut s, ":OUTP:MODE?"), "CC");
        ask(&mut s, ":SOUR:VOLT 1.000000");
        assert_eq!(ask(&mut s, ":OUTP:MODE?"), "CV");
        assert_eq!(ask(&mut s, ":MEAS:CURR?"), "0.5010");
    }

    #[test]
    fn the_status_words_of_a_66xxb() {
        let mut s = on_socket(&[("model", "6632B"), ("event-at", "1:cc,2:ovp")]);
        ask(&mut s, "STAT:OPER:PTR 3328;NTR 3328;ENAB 3328");
        ask(&mut s, "STAT:QUES:PTR 1043;NTR 1043;ENAB 1043");
        assert_eq!(ask(&mut s, "*STB?"), "0");
        ask(&mut s, ":MEAS:VOLT?");
        assert_eq!(ask(&mut s, "*STB?"), "128");
        assert_eq!(ask(&mut s, "STAT:OPER:EVEN?"), "1024");
        assert_eq!(ask(&mut s, "STAT:OPER:EVEN?"), "0");
        assert_eq!(ask(&mut s, "STAT:OPER:COND?"), "1024");
        ask(&mut s, ":MEAS:CURR?");
        assert_eq!(ask(&mut s, "*STB?"), "8");
        assert_eq!(ask(&mut s, "STAT:QUES:EVEN?"), "1");
        assert_eq!(ask(&mut s, "STAT:QUES:COND?"), "1");
    }

    #[test]
    fn hmp_status_by_its_output() {
        let mut s = on_socket(&[("model", "HMP4040"), ("on", "1"), ("load", "2")]);
        ask(&mut s, "MEAS:VOLT?");
        assert_eq!(ask(&mut s, "STAT:QUES:INST:ISUM1:COND?"), "1", "12 V on 2 ohms is in CC");
        ask(&mut s, "INST:NSEL 2");
        ask(&mut s, "VOLT 0.500000");
        ask(&mut s, "MEAS:VOLT?");
        assert_eq!(ask(&mut s, "STAT:QUES:INST:ISUM2:COND?"), "2", "0.5 V at 0.25 A is in CV");
        assert_eq!(ask(&mut s, "STAT:QUES:INST:ISUM9:COND?"), "");
    }

    #[test]
    fn hp_ib_says_who_it_is() {
        let mut s = on_socket(&[("model", "6632A")]);
        assert_eq!(ask(&mut s, "ID?"), "HP6632A");
        assert_eq!(ask(&mut s, "ROM?"), "B01 B01");
        assert_eq!(say(&mut s, "*IDN?").1.len(), 1);
        let mut s = on_socket(&[("model", "DP832")]);
        assert_eq!(say(&mut s, "ID?").1.len(), 1);
    }

    #[test]
    fn its_numbers() {
        assert_eq!(nr3(12.5), "+1.250000E+01");
        assert_eq!(nr3(0.0), "+0.000000E+00");
        assert_eq!(nr3(-0.25), "-2.500000E-01");
    }

    #[test]
    fn every_option_and_a_bad_one() {
        assert!(parts(&[("no-such".to_owned(), "1".to_owned())]).is_err());
        assert!(parts(&[("model".to_owned(), "X".to_owned())]).is_err());
        assert!(build(&[]).is_ok());
        assert!(build_tcp(&[]).is_ok());
    }
}
