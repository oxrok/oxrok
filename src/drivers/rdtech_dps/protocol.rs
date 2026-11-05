// Counterpart in sigrok:
//   libsigrok/src/hardware/rdtech-dps/api.c,
//   libsigrok/src/hardware/rdtech-dps/protocol.c,
//   libsigrok/src/hardware/rdtech-dps/protocol.h,
//   Copyright (C) 2018 James Churchill <pelrun@gmail.com>,
//   Copyright (C) 2019 Frank Stettner <frank-stettner@gmx.net>,
//   Copyright (C) 2021 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Mikael Djurfeldt, Soeren Apel, Uwe Hermann, v1ne,
//   Volodymyr Ananiev
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The registers of the `RDTech` (Riden) DPS, DPH and RD power supplies, and
//! their table of models. The bus is in [`crate::modbus`].
//!
//! The two families have the same meaning in different maps. A DPS holds the
//! settings and the measurements from register 0 and its thresholds from
//! `0x52`, an RD from 8 and from 82; an RD also has its model, serial number
//! and firmware at the start. Each reading of the state takes both blocks, as
//! in C, whatever was asked for. A read is tried three times, a write once.
//!
//! A register holds a value times a power of ten that the model's range says.
//! Only the RD6012P has two ranges, and register 20 says which is on.

use crate::core::{Error, Result};
use crate::modbus::Modbus;
use std::sync::atomic::{AtomicBool, Ordering};

/// Which map a model has.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    /// DPS and DPH.
    Dps,
    /// RD.
    Rd,
}

impl Family {
    /// `type_prefix[]` in `api.c`, for a log line.
    pub fn prefix(self) -> &'static str {
        match self {
            Family::Dps => "DPS",
            Family::Rd => "RD",
        }
    }
}

/// One range of a model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Range {
    /// The name `range=` takes.
    pub name: &'static str,
    /// The largest current limit.
    pub max_current: f64,
    /// The largest voltage target.
    pub max_voltage: f64,
    /// Decimals of a current.
    pub current_digits: u32,
    /// Decimals of a voltage.
    pub voltage_digits: u32,
}

/// One line of the table.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Model {
    /// Which map it has.
    pub family: Family,
    /// The number in its model register.
    pub id: u16,
    /// Its name.
    pub name: &'static str,
    /// Its ranges, at least one.
    pub ranges: &'static [Range],
}

const fn range(
    name: &'static str,
    max_current: f64,
    max_voltage: f64,
    current_digits: u32,
    voltage_digits: u32,
) -> Range {
    Range { name, max_current, max_voltage, current_digits, voltage_digits }
}

const fn model(family: Family, id: u16, name: &'static str, ranges: &'static [Range]) -> Model {
    Model { family, id, name, ranges }
}

const R_3005: &[Range] = &[range("5A", 5.0, 30.0, 3, 2)];
const R_5005: &[Range] = &[range("5A", 5.0, 50.0, 3, 2)];
const R_5015: &[Range] = &[range("15A", 15.0, 50.0, 2, 2)];
const R_5020: &[Range] = &[range("20A", 20.0, 50.0, 2, 2)];
const R_8005: &[Range] = &[range("5A", 5.0, 80.0, 3, 2)];
const R_6006: &[Range] = &[range("6A", 6.0, 60.0, 3, 2)];
const R_6006P: &[Range] = &[range("6A", 6.0, 60.0, 4, 3)];
const R_6012: &[Range] = &[range("12A", 12.0, 60.0, 2, 2)];
const R_6012P: &[Range] = &[range("6A", 6.0, 60.0, 4, 3), range("12A", 12.0, 60.0, 3, 3)];
const R_6018: &[Range] = &[range("18A", 18.0, 60.0, 2, 2)];
const R_6024: &[Range] = &[range("24A", 24.0, 60.0, 2, 2)];

/// `api.c`'s `supported_models[]`.
pub const MODELS: [Model; 13] = [
    model(Family::Dps, 3005, "DPS3005", R_3005),
    model(Family::Dps, 5005, "DPS5005", R_5005),
    model(Family::Dps, 5205, "DPH5005", R_5005),
    model(Family::Dps, 5015, "DPS5015", R_5015),
    model(Family::Dps, 5020, "DPS5020", R_5020),
    model(Family::Dps, 8005, "DPS8005", R_8005),
    model(Family::Rd, 60061, "RD6006", R_6006),
    model(Family::Rd, 60062, "RD6006", R_6006),
    model(Family::Rd, 60065, "RD6006P", R_6006P),
    model(Family::Rd, 60121, "RD6012", R_6012),
    model(Family::Rd, 60125, "RD6012P", R_6012P),
    model(Family::Rd, 60181, "RD6018", R_6018),
    model(Family::Rd, 60241, "RD6024", R_6024),
];

/// The model of `family` with this number.
pub fn lookup(family: Family, id: u16) -> Option<&'static Model> {
    MODELS.iter().find(|m| m.family == family && m.id == id)
}

const DPS_USET: u16 = 0x00;
const DPS_ISET: u16 = 0x01;
const DPS_LOCK: u16 = 0x06;
const DPS_ENABLE: u16 = 0x09;
const DPS_MODEL: u16 = 0x0b;
const DPS_OVPSET: u16 = 0x52;
const DPS_OCPSET: u16 = 0x53;

const RD_MODEL: u16 = 0;
const RD_VOLT_TGT: u16 = 8;
const RD_CURR_LIM: u16 = 9;
const RD_ENABLE: u16 = 18;
const RD_RANGE: u16 = 20;
const RD_OVP_THR: u16 = 82;
const RD_OCP_THR: u16 = 83;

/// Registers of a poll's first block: USET up to ENABLE.
const DPS_BLOCK: usize = 10;
/// `VOLT_TGT` up to `ENABLE`.
const RD_BLOCK: usize = 11;
/// `VOLT_TGT` up to `RANGE`.
const RD_BLOCK_WITH_RANGE: usize = 13;

const PROTECT_OVP: u16 = 1;
const PROTECT_OCP: u16 = 2;
const MODE_CC: u16 = 1;

/// A read's tries, `rdtech_dps_read_holding_registers`.
const READ_TRIES: usize = 3;

/// What a probe read off the device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Identity {
    /// The model's number.
    pub id: u16,
    /// The firmware's.
    pub version: u16,
    /// An RD's serial number, 0 where it has none.
    pub serial: u32,
}

impl Identity {
    /// The version as `api.c` prints it for a family.
    pub fn version_text(&self, family: Family) -> String {
        match family {
            Family::Dps => format!("v{}", self.version),
            Family::Rd => format!("v{}.{}", self.version / 100, self.version % 100),
        }
    }
}

/// `rdtech_dps_read_holding_registers`. A stop comes between two tries, each of
/// which may wait 500 ms for nothing, where C has no stop to look at.
fn read_registers_until(
    modbus: &mut Modbus,
    address: u16,
    registers: &mut [u16],
    stop: &AtomicBool,
) -> Result<()> {
    let mut outcome = Ok(());
    for _ in 0..READ_TRIES {
        if stop.load(Ordering::SeqCst) {
            return Err(Error::device("stopped"));
        }
        outcome = modbus.read_holding_registers(address, registers);
        if outcome.is_ok() {
            break;
        }
    }
    outcome
}

fn read_registers(modbus: &mut Modbus, address: u16, registers: &mut [u16]) -> Result<()> {
    read_registers_until(modbus, address, registers, &AtomicBool::new(false))
}

/// `rdtech_dps_get_model_version`.
///
/// # Errors
///
/// What the bus gives after its tries.
pub fn read_identity(modbus: &mut Modbus, family: Family) -> Result<Identity> {
    match family {
        Family::Dps => {
            let mut r = [0; 2];
            read_registers(modbus, DPS_MODEL, &mut r)?;
            log::info!("RDTech DPS/DPH model: {} version: {}", r[0], r[1]);
            Ok(Identity { id: r[0], version: r[1], serial: 0 })
        }
        Family::Rd => {
            let mut r = [0; 4];
            read_registers(modbus, RD_MODEL, &mut r)?;
            let serial = u32::from(r[1]) << 16 | u32::from(r[2]);
            log::info!("RDTech RD model: {} version: {}, serno {}", r[0], r[3], serial);
            Ok(Identity { id: r[0], version: r[3], serial })
        }
    }
}

/// The state of the supply, `rdtech_dps_state` as a read fills it.
#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct State {
    /// The output is on.
    pub output_enabled: bool,
    /// It regulates the current.
    pub regulation_cc: bool,
    /// The over-voltage protection has tripped.
    pub protect_ovp: bool,
    /// The over-current protection has tripped.
    pub protect_ocp: bool,
    /// The voltage setting.
    pub voltage_target: f32,
    /// The current setting.
    pub current_limit: f32,
    /// The over-voltage threshold.
    pub ovp_threshold: f32,
    /// The over-current threshold.
    pub ocp_threshold: f32,
    /// The voltage measured.
    pub voltage: f32,
    /// The current measured.
    pub current: f32,
    /// The power measured.
    pub power: f32,
    /// Which of the model's ranges is on.
    pub range: usize,
}

/// A register's raw value as a C float.
#[allow(clippy::cast_possible_truncation)]
fn scaled(raw: u16, multiplier: f64) -> f32 {
    (f64::from(raw) / multiplier) as f32
}

/// The two blocks of a DPS. The thresholds divide, in the scale of the settings:
/// `rdtech_dps_set_state` writes them so and the RD's path reads them so
/// (`protocol.c:478`, `:480`); `protocol.c:421` and `:423` multiply. The scale
/// itself is `api.c`'s and unproven on a DPS5015 and a DPS5020: `api.c` and
/// `esphome-dps` have a current in hundredths there, `rfinnie/rdserialtool` in
/// thousandths on every model, and it was tested on a DPS5005 alone. The DPS5020
/// protocol V1.2 was not read. Power is in hundredths, as in C.
pub fn decode_dps(block: &[u16; DPS_BLOCK], thresholds: &[u16; 2], link: &Multipliers) -> State {
    State {
        voltage_target: scaled(block[0], link.voltage),
        current_limit: scaled(block[1], link.current),
        voltage: scaled(block[2], link.voltage),
        current: scaled(block[3], link.current),
        power: f32::from(block[4]) / 100.0,
        protect_ovp: block[7] == PROTECT_OVP,
        protect_ocp: block[7] == PROTECT_OCP,
        regulation_cc: block[8] == MODE_CC,
        output_enabled: block[9] != 0,
        ovp_threshold: scaled(thresholds[0], link.voltage),
        ocp_threshold: scaled(thresholds[1], link.current),
        range: 0,
    }
}

/// The two blocks of an RD. `block` runs from `VOLT_TGT`: with two more
/// registers on a model with ranges, the preset and the range. Power is register
/// 13 in hundredths, as in C, and this is unproven: the `riden` Python library
/// has thousandths on the P models and register 12 as its high word, which C and
/// this ignore.
pub fn decode_rd(block: &[u16], thresholds: &[u16; 2], link: &Multipliers) -> State {
    State {
        voltage_target: scaled(block[0], link.voltage),
        current_limit: scaled(block[1], link.current),
        voltage: scaled(block[2], link.voltage),
        current: scaled(block[3], link.current),
        power: f32::from(block[5]) / 100.0,
        protect_ovp: block[8] == PROTECT_OVP,
        protect_ocp: block[8] == PROTECT_OCP,
        regulation_cc: block[9] == MODE_CC,
        output_enabled: block[10] != 0,
        ovp_threshold: scaled(thresholds[0], link.voltage),
        ocp_threshold: scaled(thresholds[1], link.current),
        range: block.get(12).map_or(0, |r| usize::from(*r != 0)),
    }
}

/// What a register holds per volt and per ampere.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Multipliers {
    /// Per volt.
    pub voltage: f64,
    /// Per ampere.
    pub current: f64,
}

impl Multipliers {
    fn of(range: &Range) -> Multipliers {
        Multipliers {
            voltage: 10f64.powi(i32::try_from(range.voltage_digits).expect("a few digits")),
            current: 10f64.powi(i32::try_from(range.current_digits).expect("a few digits")),
        }
    }
}

/// A value as its register holds it, or a refusal. C truncates, so 3.3 V goes
/// out as 3.29 V (`protocol.c:567`), and converts what does not fit without a
/// check, so -1 V goes out as 64.536 V on an RD6012P; here it is the nearest
/// value the register has, or nothing.
///
/// # Errors
///
/// What no register holds, a NaN too.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn register_value(value: f64, multiplier: f64) -> Result<u16> {
    // The sign before the rounding, or -0.004 V would round to a raw 0. A -0.0
    // passes, and is 0; a NaN does not.
    let raw = (value * multiplier).round();
    if value >= 0.0 && raw <= f64::from(u16::MAX) {
        Ok(raw as u16)
    } else {
        Err(Error::arg(format!("{value} does not fit its register")))
    }
}

/// What a setting changes: the subset of `rdtech_dps_set_state`'s state that
/// can be set.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Setting {
    /// The output.
    Output(bool),
    /// The voltage setting.
    VoltageTarget(f64),
    /// The current setting.
    CurrentLimit(f64),
    /// The over-voltage threshold.
    OvpThreshold(f64),
    /// The over-current threshold.
    OcpThreshold(f64),
    /// The panel's lock, which only a DPS has.
    Lock(bool),
    /// Which of the model's ranges is on.
    Range(usize),
}

/// The bus and what the driver knows of the device: `dev_context`.
pub struct Link {
    /// The bus, while open.
    pub modbus: Option<Modbus>,
    /// The model.
    pub model: &'static Model,
    /// The range the values are read in.
    pub range: usize,
    /// Its scale.
    pub multipliers: Multipliers,
    /// A capture runs, and it tracks the range.
    pub acquiring: bool,
    /// What the last state said, for what to report as changed.
    pub last: State,
}

impl Link {
    /// A link for `model`, its first range on. C sets the scale only where
    /// `rdtech_dps_update_range` reads a range (`protocol.c:252-253` returns
    /// first on a model with one, and nothing else sets it, `api.c:219`), so
    /// on all but the RD6012P each reading is divided by 0.0 and each setting
    /// written as 0.
    pub fn new(modbus: Option<Modbus>, model: &'static Model) -> Link {
        Link {
            modbus,
            model,
            range: 0,
            multipliers: Multipliers::of(&model.ranges[0]),
            acquiring: false,
            last: State::default(),
        }
    }

    fn bus(&mut self) -> Result<&mut Modbus> {
        self.modbus.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// Whether the last request could not go out.
    pub fn request_unsent(&self) -> bool {
        self.modbus.as_ref().is_some_and(Modbus::request_unsent)
    }

    /// The range now on.
    pub fn current_range(&self) -> &'static Range {
        &self.model.ranges[self.range]
    }

    /// Goes to a range and its scale.
    pub fn set_range(&mut self, range: usize) {
        self.range = range;
        self.multipliers = Multipliers::of(&self.model.ranges[range]);
    }

    /// `rdtech_dps_update_range`: reads which range is on, unless there is
    /// only one or a capture is tracking it.
    ///
    /// # Errors
    ///
    /// What the bus gives after its tries.
    pub fn update_range(&mut self) -> Result<()> {
        if self.model.ranges.len() <= 1 || self.acquiring {
            return Ok(());
        }
        let mut r = [0; 1];
        read_registers(self.bus()?, RD_RANGE, &mut r)?;
        self.set_range(usize::from(r[0] != 0));
        Ok(())
    }

    /// `rdtech_dps_get_state`: both blocks.
    ///
    /// # Errors
    ///
    /// What the bus gives after its tries.
    pub fn read_state(&mut self) -> Result<State> {
        self.read_state_until(&AtomicBool::new(false))
    }

    /// [`Link::read_state`], giving up between two tries once `stop` is set.
    ///
    /// # Errors
    ///
    /// What the bus gives after its tries, or the stop.
    pub fn read_state_until(&mut self, stop: &AtomicBool) -> Result<State> {
        let multipliers = self.multipliers;
        let model = self.model;
        let bus = self.bus()?;
        let mut thresholds = [0; 2];
        match model.family {
            Family::Dps => {
                let mut block = [0; DPS_BLOCK];
                read_registers_until(bus, DPS_USET, &mut block, stop)?;
                read_registers_until(bus, DPS_OVPSET, &mut thresholds, stop)?;
                Ok(decode_dps(&block, &thresholds, &multipliers))
            }
            Family::Rd => {
                let mut block = [0; RD_BLOCK_WITH_RANGE];
                let count = if model.ranges.len() > 1 { RD_BLOCK_WITH_RANGE } else { RD_BLOCK };
                let block = &mut block[..count];
                read_registers_until(bus, RD_VOLT_TGT, block, stop)?;
                read_registers_until(bus, RD_OVP_THR, &mut thresholds, stop)?;
                Ok(decode_rd(block, &thresholds, &multipliers))
            }
        }
    }

    /// `rdtech_dps_set_state` for one setting. An RD's panel lock is nothing,
    /// and nothing on the wire: `protocol.c:643` "silently succeed".
    ///
    /// # Errors
    ///
    /// A value no register holds, a range the model lacks, and what the bus
    /// gives.
    pub fn set(&mut self, setting: Setting) -> Result<()> {
        let dps = self.model.family == Family::Dps;
        let pick = |dps_register, rd_register| if dps { dps_register } else { rd_register };
        let (register, value) = match setting {
            Setting::Output(on) => (pick(DPS_ENABLE, RD_ENABLE), u16::from(on)),
            Setting::VoltageTarget(volts) => {
                (pick(DPS_USET, RD_VOLT_TGT), register_value(volts, self.multipliers.voltage)?)
            }
            Setting::CurrentLimit(amps) => {
                (pick(DPS_ISET, RD_CURR_LIM), register_value(amps, self.multipliers.current)?)
            }
            Setting::OvpThreshold(volts) => {
                (pick(DPS_OVPSET, RD_OVP_THR), register_value(volts, self.multipliers.voltage)?)
            }
            Setting::OcpThreshold(amps) => {
                (pick(DPS_OCPSET, RD_OCP_THR), register_value(amps, self.multipliers.current)?)
            }
            Setting::Lock(on) => {
                if !dps {
                    return Ok(());
                }
                (DPS_LOCK, u16::from(on))
            }
            Setting::Range(range) => {
                let count = self.model.ranges.len();
                if range >= count {
                    return Err(Error::not_supported("no such range"));
                }
                if count <= 1 {
                    return Ok(());
                }
                let value = u16::try_from(range).expect("a model has few ranges");
                self.bus()?.write_multiple_registers(RD_RANGE, &[value])?;
                // At once, in a capture too, where C waits for the next poll: a
                // setting that follows must not go out in the old scale. The
                // capture still reports the change, from what it reads next.
                self.set_range(range);
                return Ok(());
            }
        };
        self.bus()?.write_multiple_registers(register, &[value])
    }

    /// `rdtech_dps_seed_receive`: what a capture compares its first state with.
    ///
    /// # Errors
    ///
    /// What the bus gives after its tries.
    pub fn seed(&mut self) -> Result<()> {
        let state = self.read_state()?;
        self.last = state;
        self.set_range(state.range);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn multipliers(id: u16, family: Family) -> Multipliers {
        Multipliers::of(&lookup(family, id).unwrap().ranges[0])
    }

    #[test]
    fn the_table_has_its_ranges() {
        assert_eq!(lookup(Family::Dps, 5205).unwrap().name, "DPH5005");
        assert_eq!(lookup(Family::Rd, 60062).unwrap().name, "RD6006");
        assert_eq!(lookup(Family::Rd, 60125).unwrap().ranges.len(), 2);
        assert_eq!(lookup(Family::Dps, 60125), None, "the family counts");
        assert_eq!(lookup(Family::Rd, 5005), None);
        assert!(MODELS.iter().all(|m| !m.ranges.is_empty()));
    }

    #[test]
    fn a_version_prints_per_family() {
        let i = Identity { id: 1, version: 128, serial: 0 };
        assert_eq!(i.version_text(Family::Dps), "v128");
        assert_eq!(i.version_text(Family::Rd), "v1.28");
        let i = Identity { id: 1, version: 105, serial: 0 };
        assert_eq!(i.version_text(Family::Rd), "v1.5", "C's %u.%u");
    }

    #[test]
    fn a_dps_block_decodes_in_its_scale() {
        let block = [1200, 2000, 1199, 1500, 1798, 2400, 1, 1, 1, 1];
        let s = decode_dps(&block, &[4000, 5500], &multipliers(5005, Family::Dps));
        assert_eq!(s.voltage_target, 12.0);
        assert_eq!(s.current_limit, 2.0);
        assert_eq!(s.voltage, 11.99);
        assert_eq!(s.current, 1.5);
        assert_eq!(s.power, 17.98);
        assert!(s.protect_ovp && !s.protect_ocp);
        assert!(s.regulation_cc && s.output_enabled);
        assert_eq!(s.ovp_threshold, 40.0, "divided, as the settings are");
        assert_eq!(s.ocp_threshold, 5.5);
    }

    #[test]
    fn opp_is_neither_ovp_nor_ocp() {
        let block = [0, 0, 0, 0, 0, 0, 0, 3, 0, 0];
        let s = decode_dps(&block, &[0, 0], &multipliers(5005, Family::Dps));
        assert!(!s.protect_ovp && !s.protect_ocp);
    }

    #[test]
    fn an_rd_block_decodes_in_its_scale() {
        let m = multipliers(60125, Family::Rd);
        let mut block = [12000, 20000, 11999, 10000, 7, 1200, 0, 0, 2, 1, 1, 9, 5];
        let s = decode_rd(&block, &[62000, 62000], &m);
        assert_eq!(s.voltage_target, 12.0);
        assert_eq!(s.current_limit, 2.0);
        assert_eq!(s.voltage, 11.999);
        assert_eq!(s.current, 1.0);
        assert_eq!(s.power, 12.0, "the register after the energy");
        assert!(s.protect_ocp && !s.protect_ovp);
        assert!(s.regulation_cc && s.output_enabled);
        assert_eq!(s.ovp_threshold, 62.0);
        assert_eq!(s.ocp_threshold, 6.2);
        assert_eq!(s.range, 1, "any nonzero is the second");
        block[12] = 0;
        assert_eq!(decode_rd(&block, &[0, 0], &m).range, 0);
        assert_eq!(decode_rd(&block[..11], &[0, 0], &m).range, 0, "no register, one range");
    }

    #[test]
    fn a_value_goes_to_the_nearest_the_register_has() {
        assert_eq!(register_value(3.3, 100.0).unwrap(), 330);
        assert_eq!(register_value(2.01, 100.0).unwrap(), 201, "2.01 * 100 is just under 201");
        assert_eq!(register_value(3.3, 1000.0).unwrap(), 3300);
        assert_eq!(register_value(1.25, 1000.0).unwrap(), 1250);
        assert_eq!(register_value(-0.0, 100.0).unwrap(), 0);
        assert_eq!(register_value(655.35, 100.0).unwrap(), 65535);
    }

    #[test]
    fn what_no_register_holds_is_refused() {
        for bad in [655.36, -1.0, -0.004, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(register_value(bad, 100.0).is_err(), "{bad}");
        }
    }

    #[test]
    fn a_link_has_its_scale_with_one_range_too() {
        let link = Link::new(None, lookup(Family::Dps, 5005).unwrap());
        assert_eq!(link.multipliers, Multipliers { voltage: 100.0, current: 1000.0 });
        let link = Link::new(None, lookup(Family::Rd, 60065).unwrap());
        assert_eq!(link.multipliers, Multipliers { voltage: 1000.0, current: 10000.0 });
    }

    #[test]
    fn a_range_changes_the_scale() {
        let mut link = Link::new(None, lookup(Family::Rd, 60125).unwrap());
        assert_eq!(link.multipliers.current, 10000.0);
        link.set_range(1);
        assert_eq!(link.multipliers.current, 1000.0);
        assert_eq!(link.current_range().name, "12A");
    }
}
