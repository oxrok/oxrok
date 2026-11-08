// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The `BayLibre` ACME cape, as the kernel's files of its probes: `oxrok-emu serve
//! baylibre-acme --sysfs DIR` (`shells::sysfs`). libsigrok's driver reaches no
//! device: it reads and writes what the kernel's drivers put in sysfs, at
//! hard-coded paths, and so does this model.
//!
//! - **Slots.** Eight connectors. Connector `i` (1 to 8) has an ina226 power
//!   monitor at I2C address 0x40, 0x41, 0x44, 0x45, 0x42, 0x43, 0x46, 0x47, and
//!   connectors 5 to 8 a tmp435 at 0x4c, 0x49, 0x4f, 0x4b too
//!   (`baylibre-acme/protocol.c:89-95`). The device tree declares the chips
//!   whether a probe is plugged or not, so a declared chip has its
//!   `class/i2c-adapter/i2c-1/1-00XX/name` (`ina226\n`, `tmp435\n`), and only a
//!   chip that answered its driver's probe has a `hwmon/hwmonN` beside it.
//!   `slots=` says, per connector: `ina226` or `tmp435` (a probe of that kind),
//!   `none` (chips declared, no probe: the C's "is there an actual probe
//!   connected?", `protocol.c:192-201`), `empty` (nothing declared) or `other`
//!   (another chip, `lm75`, at the ina226's address). Default
//!   `ina226,ina226,none,none,tmp435,none,none,none`.
//! - **hwmon.** The probes that answered get `class/hwmon/hwmonN`, N counting
//!   from `hwmon=` (0) in connector order, and the chip's device a
//!   `hwmon/hwmonN` link to it. Units as the kernel's documents give them
//!   (`Documentation/ABI/testing/sysfs-class-hwmon`): `in1_input` the bus voltage
//!   in mV, `in0_input` the shunt voltage in mV, `curr1_input` mA, `power1_input`
//!   µW, `temp1_input` and `temp2_input` m°C, each `%ld\n`; `shunt_resistor` µΩ
//!   (`%li\n`) and `update_interval` ms. The values come from register values
//!   through the kernel's own arithmetic (`drivers/hwmon/ina2xx.c`,
//!   `ina2xx_get_value`; `drivers/hwmon/tmp401.c`, `tmp401_register_to_temp`):
//!   the bus register at 1.25 mV a bit, the current register equal to the shunt
//!   register (the driver's calibration of 2048) at `current_lsb_uA =
//!   DIV_ROUND_CLOSEST(2500000, shunt_resistor)`, the power register
//!   `bus * current / 20000` at 25 times that, a temperature register
//!   `DIV_ROUND_CLOSEST(reg * 125, 32)`. So a new `shunt_resistor` scales
//!   current and power as on a real probe. Each read of a value file gives the
//!   next value of its sequence (`pace=read`); with `pace=time` all of a probe's
//!   values move on once per `update_interval`, as the chip's conversions do, and
//!   a read between two conversions gets the same value again.
//! - **Stores** as the kernel's: `shunt_resistor` takes a decimal 1 to 2500000
//!   (`kstrtoul`, `ina2xx_set_shunt`), else EINVAL and the old value stays;
//!   `update_interval` takes a decimal (`kstrtol`) and keeps what the chip can do:
//!   the ina226 the nearest of its averaging counts times 2.2 ms
//!   (`ina226_interval_to_reg`, 35 ms at power on, 16 averages), the tmp435 a
//!   power of two times 125 ms from 125 to 16000 (`tmp401_set_convrate`; 125 ms at
//!   power on, the conversion rate register's 07h, which is the datasheet's and
//!   not in the kernel's source).
//! - **GPIOs** (`/sys/class/gpio`, `Documentation/admin-guide/gpio/sysfs.rst`):
//!   writing N to `export` makes `gpioN` (EBUSY when it is there), with
//!   `direction` (`in`, `out`, `high`, `low`; `out` drives low) and `value` (0 or
//!   1; written only when an output, else EPERM). A revision A probe has its
//!   power switch on GPIO 486, 498, 502, 482, 478, 506, 510, 474 and a line that
//!   reads 1 when it has a switch on 487, 499, 503, 483, 479, 507, 511, 475; a
//!   revision B probe its switch on 489 + 2 * (n - 1) (`protocol.c:97-107`). At
//!   power on the GPIOs of the probes are exported, as a board's start up would:
//!   the sense line of each revision A probe (an input reading 1 for a probe in
//!   `pws=`), and the switch of each probe in `pws=` (an output, `power=on`: 1).
//!   `unexported=all|N,...` leaves GPIOs out. Driving a sense line as an output
//!   is a violation: it fights the probe.
//! - **Revision B probes** (`revb=1,2`: connector numbers) have an EEPROM at the
//!   address 0x10 above their ina226 (`protocol.c:154-159`), reached through
//!   `class/i2c-dev/i2c-1/device`, a link to the adapter: 256 bytes of a 24c02,
//!   the first 61 the C's `struct probe_eeprom` big endian (type 1 for USB,
//!   revision `B`, shunt in µΩ from `eeprom-shunt=`, 20000, the power switch
//!   from `pws=`, a serial and a tag), the rest 0xff. `eeprom-fault=short|rev|type`
//!   spoils it (60 bytes, revision `A`, type 9). Default `revb=2`, `pws=1,2`.
//! - **Faults**, by path under `class/`: `absent=` (no such file), `value=PATH:TEXT`
//!   (the file holds TEXT and a newline for good; an empty TEXT an empty file),
//!   `vanish=PATH@N` (after the Nth read the file is emptied and goes, as for a
//!   probe gone), `readerr=PATH` (a directory in its place: a read fails with
//!   EISDIR, where the kernel would give EIO), `ro=PATH` (mode 0444: a write's
//!   open fails with EACCES, as for a user on a real board), `full=PATH`
//!   (`/dev/full` behind it: the open works and the write fails, ENOSPC where the
//!   kernel's store gives EINVAL). `hwmon-entry=N:NAME` names connector N's
//!   `hwmon/` entry otherwise (`hwmonX`), and `hwmon-entry=N:` leaves it empty.
//!   An armed unplug empties every value file (a probe that stops answering reads
//!   as nothing here, as an emptied file is all a tmpfs can show), and a replug
//!   gives them back.
//!
//! The model has no USB side ([`NoUsb`]).

use std::time::{Duration, Instant};

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::sysfs::{Fs, Tree, Written};
use crate::shells::tcp::NoUsb;
use crate::{Error, Result};

/// The model's name.
pub const NAME: &str = "baylibre-acme";
/// One line for `oxrok-emu list`.
pub const ABOUT: &str = "BayLibre ACME cape: ina226 and tmp435 probes as sysfs files (serve --sysfs); \
    slots=KIND,... (ina226|tmp435|none|empty|other) revb=N,.. pws=N,..|none power=on|off \
    eeprom-shunt=UOHM eeprom-fault=short|rev|type unexported=all|GPIO,.. pws-dir=in|out shunt=UOHM \
    hwmon=N pace=read|time absent=PATH value=PATH:TEXT vanish=PATH@N readerr=PATH ro=PATH full=PATH \
    hwmon-entry=N:NAME";

/// The connectors.
const SLOTS: usize = 8;
/// The ina226 of each connector (`enrg_i2c_addrs`).
const ENRG_ADDRS: [u8; SLOTS] = [0x40, 0x41, 0x44, 0x45, 0x42, 0x43, 0x46, 0x47];
/// The tmp435 of connectors 5 to 8 (`temp_i2c_addrs`).
const TEMP_ADDRS: [u8; SLOTS] = [0, 0, 0, 0, 0x4c, 0x49, 0x4f, 0x4b];
/// Revision A's power switches.
const REVA_PWS: [u32; SLOTS] = [486, 498, 502, 482, 478, 506, 510, 474];
/// Revision A's lines that say a switch is there.
const REVA_SENSE: [u32; SLOTS] = [487, 499, 503, 483, 479, 507, 511, 475];
/// Revision B's power switches.
const REVB_PWS: [u32; SLOTS] = [489, 491, 493, 495, 497, 499, 501, 503];

/// The ina226's `shunt_resistor` at probe (`INA2XX_RSHUNT_DEFAULT`), µΩ.
const SHUNT_DEFAULT: u64 = 10_000;
/// The most `ina2xx_set_shunt` takes for an ina226: 10^9 / 400 µΩ.
const SHUNT_MAX: u64 = 2_500_000;
/// The ina226's averaging counts (`ina226_avg_tab`).
const INA226_AVG: [i64; 8] = [1, 4, 16, 64, 128, 256, 512, 1024];
/// Both conversion times at power on, µs (`INA226_TOTAL_CONV_TIME_DEFAULT`).
const INA226_CONV_US: i64 = 2200;
/// The averages at power on (`INA226_CONFIG_DEFAULT` 0x4527).
const INA226_AVG_DEFAULT: usize = 2;
/// The tmp435's conversion rate at power on (07h: 125 ms).
const TMP435_RATE_DEFAULT: u32 = 7;

/// An EEPROM's bytes: a 24c02.
const EEPROM_LEN: usize = 256;
/// What C reads of it (`EEPROM_SIZE`).
const EEPROM_USED: usize = 61;

/// What a connector holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Slot {
    /// A power probe.
    Ina226,
    /// A temperature probe (connectors 5 to 8).
    Tmp435,
    /// The chips declared, no probe.
    None,
    /// Nothing declared.
    Empty,
    /// Another chip at the ina226's address.
    Other,
}

/// How an EEPROM is spoilt.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EepromFault {
    Short,
    Rev,
    Type,
}

/// The cape as the options have it.
#[derive(Clone, Debug)]
struct Config {
    slots: [Slot; SLOTS],
    revb: [bool; SLOTS],
    pws: [bool; SLOTS],
    power_on: bool,
    eeprom_shunt: u32,
    eeprom_fault: Option<EepromFault>,
    unexported: Option<Vec<u32>>,
    pws_out: bool,
    shunt: u64,
    hwmon: u32,
    time_paced: bool,
    absent: Vec<String>,
    values: Vec<(String, Vec<u8>)>,
    vanish: Vec<(String, u64)>,
    readerr: Vec<String>,
    ro: Vec<String>,
    full: Vec<String>,
    hwmon_entry: Vec<(usize, String)>,
}

impl Default for Config {
    fn default() -> Config {
        let mut revb = [false; SLOTS];
        revb[1] = true;
        let mut pws = [false; SLOTS];
        pws[0] = true;
        pws[1] = true;
        Config {
            slots: [
                Slot::Ina226,
                Slot::Ina226,
                Slot::None,
                Slot::None,
                Slot::Tmp435,
                Slot::None,
                Slot::None,
                Slot::None,
            ],
            revb,
            pws,
            power_on: true,
            eeprom_shunt: 20_000,
            eeprom_fault: None,
            unexported: Some(Vec::new()),
            pws_out: true,
            shunt: SHUNT_DEFAULT,
            hwmon: 0,
            time_paced: false,
            absent: Vec::new(),
            values: Vec::new(),
            vanish: Vec::new(),
            readerr: Vec::new(),
            ro: Vec::new(),
            full: Vec::new(),
            hwmon_entry: Vec::new(),
        }
    }
}

fn slot_list(key: &str, value: &str) -> Result<[bool; SLOTS]> {
    let mut out = [false; SLOTS];
    if value == "none" || value.is_empty() {
        return Ok(out);
    }
    for n in value.split(',') {
        let n = parse_number(key, n, 1..=8)? as usize;
        out[n - 1] = true;
    }
    Ok(out)
}

/// A path under `class/`, with no way out of it.
fn rel_path(key: &str, value: &str) -> Result<String> {
    if value.is_empty()
        || value.starts_with('/')
        || value.split('/').any(|p| p.is_empty() || p == "." || p == "..")
    {
        return Err(Error::Invalid(format!("{key}={value}: a path under class/")));
    }
    Ok(format!("class/{value}"))
}

impl Config {
    /// # Errors
    ///
    /// On an option it does not know or a value out of its range.
    #[allow(clippy::too_many_lines)]
    fn parse(options: &[(String, String)]) -> Result<Config> {
        let mut c = Config::default();
        for (k, v) in options {
            match k.as_str() {
                "slots" => {
                    let kinds: Vec<&str> = v.split(',').collect();
                    if kinds.len() != SLOTS {
                        return Err(Error::Invalid(format!("slots={v}: eight kinds")));
                    }
                    for (i, kind) in kinds.iter().enumerate() {
                        c.slots[i] = match *kind {
                            "ina226" => Slot::Ina226,
                            "tmp435" if TEMP_ADDRS[i] != 0 => Slot::Tmp435,
                            "tmp435" => {
                                return Err(Error::Invalid(format!(
                                    "slots={v}: a tmp435 is on connectors 5 to 8"
                                )));
                            }
                            "none" => Slot::None,
                            "empty" => Slot::Empty,
                            "other" => Slot::Other,
                            other => return Err(Error::Invalid(format!("slots: no kind {other}"))),
                        };
                    }
                }
                "revb" => c.revb = slot_list(k, v)?,
                "pws" => c.pws = slot_list(k, v)?,
                "power" => {
                    c.power_on = match v.as_str() {
                        "on" => true,
                        "off" => false,
                        _ => return Err(Error::Invalid(format!("power={v}: on or off"))),
                    };
                }
                "eeprom-shunt" => c.eeprom_shunt = parse_number(k, v, 0..=u32::MAX)?,
                "eeprom-fault" => {
                    c.eeprom_fault = Some(match v.as_str() {
                        "short" => EepromFault::Short,
                        "rev" => EepromFault::Rev,
                        "type" => EepromFault::Type,
                        _ => return Err(Error::Invalid(format!("eeprom-fault={v}"))),
                    });
                }
                "unexported" => {
                    c.unexported = if v == "all" {
                        None
                    } else {
                        let mut gpios = Vec::new();
                        for g in v.split(',') {
                            gpios.push(parse_number(k, g, 0..=1023)?);
                        }
                        Some(gpios)
                    };
                }
                "pws-dir" => {
                    c.pws_out = match v.as_str() {
                        "out" => true,
                        "in" => false,
                        _ => return Err(Error::Invalid(format!("pws-dir={v}: in or out"))),
                    };
                }
                "shunt" => {
                    c.shunt = u64::from(parse_number(k, v, 1..=u32::try_from(SHUNT_MAX).unwrap_or(u32::MAX))?);
                }
                "hwmon" => c.hwmon = parse_number(k, v, 0..=999)?,
                "pace" => {
                    c.time_paced = match v.as_str() {
                        "read" => false,
                        "time" => true,
                        _ => return Err(Error::Invalid(format!("pace={v}: read or time"))),
                    };
                }
                "absent" => c.absent.push(rel_path(k, v)?),
                "value" => {
                    let (path, text) = v
                        .split_once(':')
                        .ok_or_else(|| Error::Invalid(format!("value={v}: PATH:TEXT")))?;
                    let data = if text.is_empty() { Vec::new() } else { format!("{text}\n").into_bytes() };
                    c.values.push((rel_path(k, path)?, data));
                }
                "vanish" => {
                    let (path, n) = v
                        .split_once('@')
                        .ok_or_else(|| Error::Invalid(format!("vanish={v}: PATH@N")))?;
                    c.vanish.push((rel_path(k, path)?, u64::from(parse_number(k, n, 0..=u32::MAX)?)));
                }
                "readerr" => c.readerr.push(rel_path(k, v)?),
                "ro" => c.ro.push(rel_path(k, v)?),
                "full" => c.full.push(rel_path(k, v)?),
                "hwmon-entry" => {
                    let (n, name) = v
                        .split_once(':')
                        .ok_or_else(|| Error::Invalid(format!("hwmon-entry={v}: N:NAME")))?;
                    let n = parse_number(k, n, 1..=8)? as usize;
                    if name.contains('/') || name == "." || name == ".." {
                        return Err(Error::Invalid(format!("hwmon-entry={v}: a name")));
                    }
                    c.hwmon_entry.push((n - 1, name.to_owned()));
                }
                _ => return Err(Error::Invalid(format!("{NAME} has no option {k}"))),
            }
        }
        // Every path a fault names is one the tree has.
        let probes = probes(&c);
        let known = |p: &str| tree_paths(&c, &probes).iter().any(|q| q == p);
        for p in c
            .absent
            .iter()
            .chain(c.values.iter().map(|(p, _)| p))
            .chain(c.vanish.iter().map(|(p, _)| p))
            .chain(&c.readerr)
            .chain(&c.ro)
            .chain(&c.full)
        {
            if !known(p) {
                return Err(Error::Invalid(format!("{}: the tree has no such file", &p["class/".len()..])));
            }
        }
        Ok(c)
    }

    fn exported(&self, gpio: u32) -> bool {
        self.unexported.as_ref().is_some_and(|out| !out.contains(&gpio))
    }
}

/// What kind of probe answered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Power,
    Temperature,
}

/// A probe that answered its driver, and its hwmon device.
#[derive(Clone, Debug)]
struct Probe {
    slot: usize,
    kind: Kind,
    addr: u8,
    hwmon: u32,
}

impl Probe {
    fn dir(&self) -> String {
        format!("class/hwmon/hwmon{}", self.hwmon)
    }
}

/// The probes, in connector order.
fn probes(c: &Config) -> Vec<Probe> {
    let mut out = Vec::new();
    for (slot, kind) in c.slots.iter().enumerate() {
        let (kind, addr) = match kind {
            Slot::Ina226 => (Kind::Power, ENRG_ADDRS[slot]),
            Slot::Tmp435 => (Kind::Temperature, TEMP_ADDRS[slot]),
            _ => continue,
        };
        let hwmon = c.hwmon + u32::try_from(out.len()).unwrap_or(0);
        out.push(Probe { slot, kind, addr, hwmon });
    }
    out
}

/// The paths of the files a fault may name.
fn tree_paths(c: &Config, probes: &[Probe]) -> Vec<String> {
    let mut out = Vec::new();
    for p in probes {
        let d = p.dir();
        out.push(format!("{d}/name"));
        out.push(format!("{d}/update_interval"));
        match p.kind {
            Kind::Power => {
                for f in ["in0_input", "in1_input", "curr1_input", "power1_input", "shunt_resistor"] {
                    out.push(format!("{d}/{f}"));
                }
            }
            Kind::Temperature => {
                out.push(format!("{d}/temp1_input"));
                out.push(format!("{d}/temp2_input"));
            }
        }
    }
    for (slot, kind) in c.slots.iter().enumerate() {
        if *kind != Slot::Empty {
            out.push(format!("{}/name", device(ENRG_ADDRS[slot])));
            if TEMP_ADDRS[slot] != 0 && *kind != Slot::Other {
                out.push(format!("{}/name", device(TEMP_ADDRS[slot])));
            }
        }
        if c.revb[slot] {
            out.push(format!("{}/eeprom", device(ENRG_ADDRS[slot] + 0x10)));
        }
    }
    for gpio in gpios(c) {
        out.push(format!("class/gpio/gpio{}/value", gpio.0));
        out.push(format!("class/gpio/gpio{}/direction", gpio.0));
    }
    out.push("class/gpio/export".to_owned());
    out.push("class/gpio/unexport".to_owned());
    out
}

fn device(addr: u8) -> String {
    format!("class/i2c-adapter/i2c-1/1-00{addr:02x}")
}

/// A GPIO the board has: its number, whether it is an output, its level, and
/// whether it senses (a revision A probe's switch line).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Gpio(pub u32, pub bool, pub bool, pub bool);

/// The GPIOs of the probes plugged, as they are at power on (exported or not).
fn gpios(c: &Config) -> Vec<Gpio> {
    let mut out = Vec::new();
    for (slot, kind) in c.slots.iter().enumerate() {
        if !matches!(kind, Slot::Ina226 | Slot::Tmp435) {
            continue;
        }
        if c.revb[slot] {
            if c.pws[slot] {
                out.push(Gpio(REVB_PWS[slot], c.pws_out, c.power_on, false));
            }
        } else {
            out.push(Gpio(REVA_SENSE[slot], false, c.pws[slot], true));
            if c.pws[slot] {
                out.push(Gpio(REVA_PWS[slot], c.pws_out, c.power_on, false));
            }
        }
    }
    out
}

/// `DIV_ROUND_CLOSEST` of the kernel, for a positive divisor.
fn div_round_closest(x: i64, d: i64) -> i64 {
    if x >= 0 { (x + d / 2) / d } else { (x - d / 2) / d }
}

/// The current register's µA a bit for a shunt in µΩ (`ina2xx_set_shunt`).
fn current_lsb_ua(shunt: u64) -> i64 {
    div_round_closest(2_500_000_000 / 1000, i64::try_from(shunt).unwrap_or(1))
}

/// An ina226's registers at one conversion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Ina226Regs {
    bus: i64,
    shunt: i64,
}

impl Ina226Regs {
    /// The `k`th conversion of the probe on connector `slot` (0 to 7): about
    /// 5 V and 400 mA at the default shunt.
    fn at(slot: usize, k: u64) -> Ina226Regs {
        let s = slot as u64;
        let bus = 4000 + i64::try_from((k * 37 + s * 11) % 41).unwrap_or(0) - 20;
        let shunt = 1600 + i64::try_from((k * 53 + s * 7) % 97).unwrap_or(0) - 48;
        Ina226Regs { bus, shunt }
    }

    /// `in1_input`, mV.
    fn bus_mv(self) -> i64 {
        div_round_closest(self.bus * 1250, 1000)
    }

    /// `in0_input`, mV: 2.5 µV a bit.
    fn shunt_mv(self) -> i64 {
        div_round_closest(self.shunt, 400)
    }

    /// `curr1_input`, mA, under a `shunt_resistor` of `rshunt` µΩ.
    fn current_ma(self, rshunt: u64) -> i64 {
        div_round_closest(self.shunt * current_lsb_ua(rshunt), 1000)
    }

    /// `power1_input`, µW.
    fn power_uw(self, rshunt: u64) -> i64 {
        let power = self.bus * self.shunt.abs() / 20000;
        power * 25 * current_lsb_ua(rshunt)
    }
}

/// A tmp435's two temperature registers at the `k`th conversion, local and
/// remote: the local one in halves of a degree, the remote one in sixteenths.
fn tmp435_regs(slot: usize, k: u64) -> (i64, i64) {
    let s = slot as u64;
    let local = (25 << 8) + (i64::try_from((k + s) % 4).unwrap_or(0) << 7);
    let remote = (40 << 8) + (i64::try_from((k * 3 + s) % 16).unwrap_or(0) << 4);
    (local, remote)
}

/// m°C of a temperature register (`tmp401_register_to_temp`, standard range).
fn temp_mc(reg: i64) -> i64 {
    div_round_closest(reg * 125, 32)
}

/// `update_interval` of an ina226 with the averaging count at `avg`.
fn ina226_interval(avg: usize) -> i64 {
    div_round_closest(INA226_AVG[avg] * INA226_CONV_US, 1000)
}

/// The averaging count `ina226_interval_to_reg` picks for an interval in ms.
fn ina226_avg_for(interval: i64) -> usize {
    let interval = interval.clamp(0, 32000);
    let avg = div_round_closest(interval * 1000, INA226_CONV_US);
    // find_closest: the first whose midpoint with the next is not below it.
    for i in 0..INA226_AVG.len() - 1 {
        if avg <= div_round_closest(INA226_AVG[i] + INA226_AVG[i + 1], 2) {
            return i;
        }
    }
    INA226_AVG.len() - 1
}

/// `update_interval` of a tmp435 at conversion rate `rate`.
fn tmp435_interval(rate: u32) -> i64 {
    (1i64 << (7 - rate)) * 125
}

/// The rate `tmp401_set_convrate` picks for an interval in ms.
fn tmp435_rate_for(interval: i64) -> u32 {
    let val = interval.clamp(125, 16000);
    let x = val * 4 / (125 * 3);
    // __fls: the index of the highest bit set.
    7 - (63 - x.leading_zeros())
}

/// The kernel's `kstrtoul` in base 10: digits with an optional `+` and one
/// trailing newline.
fn kstrtoul(data: &[u8]) -> Option<u64> {
    let text = std::str::from_utf8(data).ok()?;
    let text = text.strip_suffix('\n').unwrap_or(text);
    let digits = text.strip_prefix('+').unwrap_or(text);
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    digits.parse().ok()
}

/// The kernel's `kstrtol` in base 10.
fn kstrtol(data: &[u8]) -> Option<i64> {
    let text = std::str::from_utf8(data).ok()?;
    let text = text.strip_suffix('\n').unwrap_or(text);
    let (neg, digits) = match text.strip_prefix('-') {
        Some(d) => (true, d),
        None => (false, text.strip_prefix('+').unwrap_or(text)),
    };
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let v: i64 = digits.parse().ok()?;
    Some(if neg { -v } else { v })
}

/// `kstrtol` in base 0, as `value_store` and `export_store` read: `0x` hex,
/// a leading `0` octal.
fn kstrtol0(data: &[u8]) -> Option<i64> {
    let text = std::str::from_utf8(data).ok()?;
    let text = text.strip_suffix('\n').unwrap_or(text);
    let (neg, rest) = match text.strip_prefix('-') {
        Some(d) => (true, d),
        None => (false, text.strip_prefix('+').unwrap_or(text)),
    };
    let v = if let Some(hex) = rest.strip_prefix("0x").or_else(|| rest.strip_prefix("0X")) {
        i64::from_str_radix(hex, 16).ok()?
    } else if rest.len() > 1 && rest.starts_with('0') {
        i64::from_str_radix(&rest[1..], 8).ok()?
    } else {
        if rest.is_empty() || !rest.bytes().all(|b| b.is_ascii_digit()) {
            return None;
        }
        rest.parse().ok()?
    };
    Some(if neg { -v } else { v })
}

/// The EEPROM of a revision B probe.
fn eeprom(c: &Config, slot: usize) -> Vec<u8> {
    let mut e = vec![0xff; EEPROM_LEN];
    let kind: u32 = if c.eeprom_fault == Some(EepromFault::Type) { 9 } else { 1 };
    let rev: u32 = if c.eeprom_fault == Some(EepromFault::Rev) { u32::from(b'A') } else { u32::from(b'B') };
    e[0..4].copy_from_slice(&kind.to_be_bytes());
    e[4..8].copy_from_slice(&rev.to_be_bytes());
    e[8..12].copy_from_slice(&c.eeprom_shunt.to_be_bytes());
    e[12] = u8::from(c.pws[slot]);
    let serial = format!("ACME-{:02X}-0001", ENRG_ADDRS[slot]);
    e[13..13 + 16].fill(0);
    e[13..13 + serial.len()].copy_from_slice(serial.as_bytes());
    e[29..29 + 32].fill(0);
    e[29..29 + 3].copy_from_slice(b"USB");
    if c.eeprom_fault == Some(EepromFault::Short) {
        e.truncate(EEPROM_USED - 1);
    }
    e
}

/// The state of one GPIO.
#[derive(Clone, Copy, Debug)]
#[allow(clippy::struct_excessive_bools)]
struct Line {
    gpio: u32,
    out: bool,
    high: bool,
    sense: bool,
    exported: bool,
}

/// A power probe's or a temperature probe's state.
#[derive(Clone, Debug)]
struct Chip {
    probe: Probe,
    /// `shunt_resistor`, µΩ.
    shunt: u64,
    /// The averaging count (ina226) or the conversion rate (tmp435).
    setting: u32,
    /// Conversions so far, for `pace=time`.
    conversions: u64,
    last: Option<Instant>,
}

impl Chip {
    fn interval(&self) -> i64 {
        match self.probe.kind {
            Kind::Power => ina226_interval(self.setting as usize),
            Kind::Temperature => tmp435_interval(self.setting),
        }
    }
}

/// The cape's files and what they hold.
struct Acme {
    c: Config,
    chips: Vec<Chip>,
    lines: Vec<Line>,
    /// Reads of each paced file so far.
    reads: Vec<(String, u64)>,
    unplugged: bool,
}

impl Acme {
    fn new(c: Config) -> Acme {
        let chips = probes(&c)
            .into_iter()
            .map(|probe| {
                let setting = match probe.kind {
                    Kind::Power => INA226_AVG_DEFAULT as u32,
                    Kind::Temperature => TMP435_RATE_DEFAULT,
                };
                Chip { probe, shunt: c.shunt, setting, conversions: 0, last: None }
            })
            .collect();
        let lines = gpios(&c)
            .into_iter()
            .map(|Gpio(gpio, out, high, sense)| Line { gpio, out, high, sense, exported: c.exported(gpio) })
            .collect();
        Acme { c, chips, lines, reads: Vec::new(), unplugged: false }
    }

    fn reads_of(&self, path: &str) -> u64 {
        self.reads.iter().find(|(p, _)| p == path).map_or(0, |(_, n)| *n)
    }

    /// What a value file holds at its `k`th read or conversion.
    fn value(chip: &Chip, file: &str, k: u64) -> Vec<u8> {
        let slot = chip.probe.slot;
        let v = match file {
            "in0_input" => Ina226Regs::at(slot, k).shunt_mv(),
            "in1_input" => Ina226Regs::at(slot, k).bus_mv(),
            "curr1_input" => Ina226Regs::at(slot, k).current_ma(chip.shunt),
            "power1_input" => Ina226Regs::at(slot, k).power_uw(chip.shunt),
            "temp1_input" => temp_mc(tmp435_regs(slot, k).0),
            "temp2_input" => temp_mc(tmp435_regs(slot, k).1),
            _ => return Vec::new(),
        };
        format!("{v}\n").into_bytes()
    }

    fn value_files(kind: Kind) -> &'static [&'static str] {
        match kind {
            Kind::Power => &["in0_input", "in1_input", "curr1_input", "power1_input"],
            Kind::Temperature => &["temp1_input", "temp2_input"],
        }
    }

    /// The `k` a value file is at.
    fn index(&self, chip: &Chip, path: &str) -> u64 {
        if self.c.time_paced { chip.conversions } else { self.reads_of(path) }
    }

    fn fixed(&self, path: &str) -> Option<&Vec<u8>> {
        self.c.values.iter().rev().find(|(p, _)| p == path).map(|(_, d)| d)
    }

    fn faulted(&self, path: &str) -> bool {
        self.c.absent.iter().any(|p| p == path)
            || self.c.readerr.iter().any(|p| p == path)
            || self.c.full.iter().any(|p| p == path)
    }

    /// Puts a file, as the faults have it.
    fn put(&self, fs: &mut Fs, path: &str, data: Vec<u8>, mode: u32, paced: bool) {
        if self.c.absent.iter().any(|p| p == path) {
            return;
        }
        if self.c.readerr.iter().any(|p| p == path) {
            fs.dir(path);
            return;
        }
        if self.c.full.iter().any(|p| p == path) {
            fs.link(path, "/dev/full");
            return;
        }
        let data = self.fixed(path).cloned().unwrap_or(data);
        let mode = if self.c.ro.iter().any(|p| p == path) { 0o444 } else { mode };
        if paced {
            fs.paced(path, data);
        } else {
            fs.file(path, data, mode);
        }
    }

    fn gpio_files(&self, fs: &mut Fs, line: Line) {
        let d = format!("class/gpio/gpio{}", line.gpio);
        self.put(fs, &format!("{d}/direction"), if line.out { b"out\n".to_vec() } else { b"in\n".to_vec() }, 0o666, false);
        self.put(fs, &format!("{d}/value"), format!("{}\n", u8::from(line.high)).into_bytes(), 0o666, false);
        fs.file(&format!("{d}/active_low"), "0\n", 0o644);
    }

    /// Refreshes every value file of `chip` (after a new shunt or conversion).
    fn refresh(&self, fs: &mut Fs, at: usize) {
        if self.unplugged {
            return;
        }
        let chip = &self.chips[at];
        for f in Self::value_files(chip.probe.kind) {
            let path = format!("{}/{f}", chip.probe.dir());
            if self.faulted(&path) || self.fixed(&path).is_some() || self.vanished(&path) {
                continue;
            }
            fs.set(&path, Self::value(chip, f, self.index(chip, &path)));
        }
    }

    fn vanished(&self, path: &str) -> bool {
        self.c.vanish.iter().any(|(p, n)| p == path && self.reads_of(path) >= *n)
    }

    fn line_at(&mut self, gpio: u32) -> Option<&mut Line> {
        self.lines.iter_mut().find(|l| l.gpio == gpio)
    }

    fn written_gpio(&mut self, rest: &str, data: &[u8], fs: &mut Fs) -> Written {
        let text = std::str::from_utf8(data).unwrap_or("");
        let text = text.strip_suffix('\n').unwrap_or(text);
        match rest {
            "export" => {
                let Some(gpio) = kstrtol0(data).and_then(|g| u32::try_from(g).ok()) else {
                    return Written::Refused("EINVAL");
                };
                if self.line_at(gpio).is_none() {
                    self.lines.push(Line { gpio, out: false, high: false, sense: false, exported: false });
                }
                let line = self.line_at(gpio).copied().unwrap_or(Line {
                    gpio,
                    out: false,
                    high: false,
                    sense: false,
                    exported: false,
                });
                if line.exported {
                    return Written::Refused("EBUSY");
                }
                if let Some(l) = self.line_at(gpio) {
                    l.exported = true;
                }
                self.gpio_files(fs, line);
                Written::Kept
            }
            "unexport" => {
                let Some(gpio) = kstrtol0(data).and_then(|g| u32::try_from(g).ok()) else {
                    return Written::Refused("EINVAL");
                };
                match self.line_at(gpio) {
                    Some(l) if l.exported => {
                        l.exported = false;
                        fs.remove(&format!("class/gpio/gpio{gpio}"));
                        Written::Kept
                    }
                    _ => Written::Refused("EINVAL"),
                }
            }
            _ => {
                let Some((dir, file)) = rest.split_once('/') else { return Written::Kept };
                let Some(gpio) = dir.strip_prefix("gpio").and_then(|g| g.parse::<u32>().ok()) else {
                    return Written::Kept;
                };
                let Some(line) = self.line_at(gpio).copied() else { return Written::Kept };
                let d = format!("class/gpio/gpio{gpio}");
                match file {
                    "direction" => {
                        let (out, high) = match text {
                            "in" => (false, line.high),
                            "out" | "low" => (true, false),
                            "high" => (true, true),
                            _ => return Written::Refused("EINVAL"),
                        };
                        if out && line.sense {
                            fs.violation(format!(
                                "drives GPIO {gpio} as an output: it is a probe's power switch sense line, an input"
                            ));
                        }
                        if let Some(l) = self.line_at(gpio) {
                            l.out = out;
                            l.high = high;
                        }
                        fs.set(&format!("{d}/direction"), if out { "out\n" } else { "in\n" });
                        fs.set(&format!("{d}/value"), format!("{}\n", u8::from(high)));
                        Written::Kept
                    }
                    "value" => {
                        if !line.out {
                            return Written::Refused("EPERM");
                        }
                        let Some(v) = kstrtol0(data) else { return Written::Refused("EINVAL") };
                        if let Some(l) = self.line_at(gpio) {
                            l.high = v != 0;
                        }
                        fs.set(&format!("{d}/value"), format!("{}\n", u8::from(v != 0)));
                        Written::Kept
                    }
                    _ => Written::Kept,
                }
            }
        }
    }
}

impl Tree for Acme {
    fn build(&mut self, fs: &mut Fs) {
        let c = &self.c;
        fs.dir("class");
        fs.dir("class/hwmon");
        // The I2C devices.
        for (slot, kind) in c.slots.iter().enumerate() {
            let enrg = device(ENRG_ADDRS[slot]);
            match kind {
                Slot::Empty => {}
                Slot::Other => self.put(fs, &format!("{enrg}/name"), b"lm75\n".to_vec(), 0o444, false),
                _ => {
                    self.put(fs, &format!("{enrg}/name"), b"ina226\n".to_vec(), 0o444, false);
                    if TEMP_ADDRS[slot] != 0 {
                        let temp = device(TEMP_ADDRS[slot]);
                        self.put(fs, &format!("{temp}/name"), b"tmp435\n".to_vec(), 0o444, false);
                    }
                }
            }
            if c.revb[slot] {
                let rom = device(ENRG_ADDRS[slot] + 0x10);
                self.put(fs, &format!("{rom}/name"), b"24c02\n".to_vec(), 0o444, false);
                self.put(fs, &format!("{rom}/eeprom"), eeprom(c, slot), 0o444, false);
            }
        }
        fs.link("class/i2c-dev/i2c-1/device", "../../i2c-adapter/i2c-1");
        fs.file("class/i2c-dev/i2c-1/name", "i2c-1\n", 0o444);
        // The hwmon devices.
        for chip in &self.chips {
            let p = &chip.probe;
            let dev = device(p.addr);
            match c.hwmon_entry.iter().rev().find(|(s, _)| *s == p.slot) {
                Some((_, name)) if name.is_empty() => fs.dir(&format!("{dev}/hwmon")),
                Some((_, name)) => fs.dir(&format!("{dev}/hwmon/{name}")),
                None => fs.link(&format!("{dev}/hwmon/hwmon{}", p.hwmon), &format!("../../../../hwmon/hwmon{}", p.hwmon)),
            }
            let d = p.dir();
            let (name, interval) = match p.kind {
                Kind::Power => ("ina226", ina226_interval(chip.setting as usize)),
                Kind::Temperature => ("tmp435", tmp435_interval(chip.setting)),
            };
            self.put(fs, &format!("{d}/name"), format!("{name}\n").into_bytes(), 0o444, false);
            self.put(fs, &format!("{d}/update_interval"), format!("{interval}\n").into_bytes(), 0o666, false);
            if p.kind == Kind::Power {
                self.put(fs, &format!("{d}/shunt_resistor"), format!("{}\n", chip.shunt).into_bytes(), 0o666, false);
            }
            for f in Self::value_files(p.kind) {
                let path = format!("{d}/{f}");
                self.put(fs, &path, Self::value(chip, f, 0), 0o444, true);
            }
        }
        // The GPIOs.
        fs.file("class/gpio/export", "", 0o222);
        fs.file("class/gpio/unexport", "", 0o222);
        for line in &self.lines {
            if line.exported {
                self.gpio_files(fs, *line);
            }
        }
    }

    fn read(&mut self, path: &str, fs: &mut Fs) {
        match self.reads.iter_mut().find(|(p, _)| p == path) {
            Some((_, n)) => *n += 1,
            None => self.reads.push((path.to_owned(), 1)),
        }
        let n = self.reads_of(path);
        if self.c.vanish.iter().any(|(p, at)| p == path && n == *at) {
            fs.remove(path);
            return;
        }
        if self.c.time_paced || self.unplugged || self.fixed(path).is_some() {
            return;
        }
        let Some((dir, file)) = path.rsplit_once('/') else { return };
        let Some(chip) = self.chips.iter().find(|ch| ch.probe.dir() == dir) else { return };
        fs.set(path, Self::value(chip, file, n));
    }

    fn written(&mut self, path: &str, data: &[u8], fs: &mut Fs) -> Written {
        if let Some(rest) = path.strip_prefix("class/gpio/") {
            return self.written_gpio(rest, data, fs);
        }
        let Some((dir, file)) = path.rsplit_once('/') else { return Written::Kept };
        let Some(at) = self.chips.iter().position(|ch| ch.probe.dir() == dir) else {
            return Written::Kept;
        };
        match (self.chips[at].probe.kind, file) {
            (Kind::Power, "shunt_resistor") => {
                let Some(v) = kstrtoul(data).filter(|v| (1..=SHUNT_MAX).contains(v)) else {
                    return Written::Refused("EINVAL");
                };
                self.chips[at].shunt = v;
                fs.set(path, format!("{v}\n"));
                self.refresh(fs, at);
                Written::Kept
            }
            (kind, "update_interval") => {
                let Some(v) = kstrtol(data) else { return Written::Refused("EINVAL") };
                let chip = &mut self.chips[at];
                chip.setting = match kind {
                    Kind::Power => ina226_avg_for(v) as u32,
                    Kind::Temperature => tmp435_rate_for(v),
                };
                fs.set(path, format!("{}\n", chip.interval()));
                Written::Kept
            }
            _ => Written::Kept,
        }
    }

    fn tick(&mut self, now: Instant, fs: &mut Fs) {
        if !self.c.time_paced {
            return;
        }
        for at in 0..self.chips.len() {
            let chip = &mut self.chips[at];
            let every = Duration::from_millis(u64::try_from(chip.interval()).unwrap_or(1).max(1));
            match chip.last {
                None => chip.last = Some(now),
                Some(last) if now.duration_since(last) >= every => {
                    chip.last = Some(last + every);
                    chip.conversions += 1;
                    self.refresh(fs, at);
                }
                Some(_) => {}
            }
        }
    }

    /// libudev lists `class` for libusb in sigrok-cli; the driver lists what is
    /// under it.
    fn listed(&self, path: &str) -> bool {
        path != "class"
    }

    fn unplug(&mut self, fs: &mut Fs) {
        for chip in &self.chips {
            for f in Self::value_files(chip.probe.kind) {
                let path = format!("{}/{f}", chip.probe.dir());
                if !self.faulted(&path) && !self.vanished(&path) {
                    fs.set(&path, Vec::new());
                }
            }
        }
        self.unplugged = true;
    }

    fn replug(&mut self, fs: &mut Fs) {
        self.unplugged = false;
        for at in 0..self.chips.len() {
            self.refresh(fs, at);
        }
    }
}

/// No USB: the model is a tree alone.
///
/// # Errors
///
/// When an option is wrong.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Config::parse(options)?;
    Ok(Box::new(NoUsb::new(NAME)))
}

/// The cape's files.
///
/// # Errors
///
/// When an option is wrong.
pub fn build_sysfs(options: &[(String, String)]) -> Result<Box<dyn Tree>> {
    Ok(Box::new(Acme::new(Config::parse(options)?)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::sysfs::{Node, Op};

    fn opts(list: &[(&str, &str)]) -> Vec<(String, String)> {
        list.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    fn built(list: &[(&str, &str)]) -> (Acme, Vec<Op>) {
        let mut acme = Acme::new(Config::parse(&opts(list)).unwrap());
        let mut fs = Fs::default();
        acme.build(&mut fs);
        let (ops, _) = fs.take();
        (acme, ops)
    }

    fn file<'a>(ops: &'a [Op], path: &str) -> Option<&'a Node> {
        ops.iter().rev().find_map(|op| match op {
            Op::Put(p, n) if p == path => Some(n),
            _ => None,
        })
    }

    fn text(ops: &[Op], path: &str) -> String {
        match file(ops, path) {
            Some(Node::File { data, .. }) => String::from_utf8_lossy(data).into_owned(),
            other => panic!("{path}: {other:?}"),
        }
    }

    fn peek(fs: &Fs, path: &str) -> Option<String> {
        fs.ops().iter().rev().find_map(|op| match op {
            Op::Set(p, d) if p == path => Some(String::from_utf8_lossy(d).into_owned()),
            _ => None,
        })
    }

    fn set_text(fs: &mut Fs, path: &str) -> Option<String> {
        let (ops, _) = fs.take();
        ops.iter().rev().find_map(|op| match op {
            Op::Set(p, d) if p == path => Some(String::from_utf8_lossy(d).into_owned()),
            _ => None,
        })
    }

    /// The units of the kernel's files, pinned: in1 mV at 1.25 mV a bit, in0 mV
    /// at 2.5 µV a bit, curr1 mA, power1 µW, temps m°C.
    #[test]
    fn values_have_the_kernels_units() {
        let r = Ina226Regs { bus: 4000, shunt: 1600 };
        assert_eq!(r.bus_mv(), 5000, "in1_input: 4000 * 1.25 mV");
        assert_eq!(r.shunt_mv(), 4, "in0_input: 1600 * 2.5 µV = 4 mV");
        assert_eq!(current_lsb_ua(10_000), 250, "10 mΩ: 250 µA a bit");
        assert_eq!(r.current_ma(10_000), 400, "curr1_input: 1600 * 250 µA");
        assert_eq!(r.power_uw(10_000), 2_000_000, "power1_input: 320 * 6250 µW = 2 W");
        assert_eq!(r.current_ma(20_000), 200);
        assert_eq!(r.power_uw(20_000), 1_000_000);
        assert_eq!(temp_mc(25 << 8), 25_000, "temp: m°C");
        assert_eq!(temp_mc((40 << 8) + 16), 40_063, "40.0625 °C rounds up, as DIV_ROUND_CLOSEST");
        assert_eq!(div_round_closest(-5, 2), -3);
        assert_eq!(div_round_closest(5, 2), 3);
    }

    #[test]
    fn update_intervals_are_what_the_chips_can_do() {
        assert_eq!(ina226_interval(INA226_AVG_DEFAULT), 35);
        assert_eq!(ina226_interval(ina226_avg_for(100)), 141, "C's 1000 / 10 Hz");
        assert_eq!(ina226_interval(ina226_avg_for(2)), 2, "500 Hz");
        assert_eq!(ina226_interval(ina226_avg_for(1000)), 1126, "1 Hz");
        assert_eq!(ina226_interval(ina226_avg_for(0)), 2);
        assert_eq!(ina226_interval(ina226_avg_for(1_000_000)), 2253);
        assert_eq!(tmp435_interval(TMP435_RATE_DEFAULT), 125);
        assert_eq!(tmp435_interval(tmp435_rate_for(100)), 125);
        assert_eq!(tmp435_interval(tmp435_rate_for(1000)), 1000);
        assert_eq!(tmp435_interval(tmp435_rate_for(333)), 250);
        assert_eq!(tmp435_interval(tmp435_rate_for(99_999)), 16000);
    }

    #[test]
    fn the_default_cape_has_two_power_probes_and_a_temperature_probe() {
        let (_, ops) = built(&[]);
        assert_eq!(text(&ops, "class/i2c-adapter/i2c-1/1-0040/name"), "ina226\n");
        assert_eq!(text(&ops, "class/i2c-adapter/i2c-1/1-004c/name"), "tmp435\n");
        assert_eq!(text(&ops, "class/i2c-adapter/i2c-1/1-0044/name"), "ina226\n");
        assert_eq!(
            file(&ops, "class/i2c-adapter/i2c-1/1-0040/hwmon/hwmon0"),
            Some(&Node::Link("../../../../hwmon/hwmon0".into()))
        );
        assert!(file(&ops, "class/i2c-adapter/i2c-1/1-0044/hwmon/hwmon2").is_none());
        assert_eq!(
            file(&ops, "class/i2c-adapter/i2c-1/1-004c/hwmon/hwmon2"),
            Some(&Node::Link("../../../../hwmon/hwmon2".into()))
        );
        assert_eq!(text(&ops, "class/hwmon/hwmon0/in1_input"), format!("{}\n", Ina226Regs::at(0, 0).bus_mv()));
        assert_eq!(text(&ops, "class/hwmon/hwmon0/shunt_resistor"), "10000\n");
        assert_eq!(text(&ops, "class/hwmon/hwmon0/update_interval"), "35\n");
        assert_eq!(text(&ops, "class/hwmon/hwmon2/update_interval"), "125\n");
        assert!(matches!(file(&ops, "class/hwmon/hwmon2/temp2_input"), Some(Node::File { paced: true, mode: 0o444, .. })));
        // Revision A's sense line and switch, revision B's switch.
        assert_eq!(text(&ops, "class/gpio/gpio487/value"), "1\n");
        assert_eq!(text(&ops, "class/gpio/gpio487/direction"), "in\n");
        assert_eq!(text(&ops, "class/gpio/gpio486/direction"), "out\n");
        assert_eq!(text(&ops, "class/gpio/gpio491/value"), "1\n");
        assert_eq!(text(&ops, "class/gpio/gpio479/value"), "0\n", "the tmp435's sense line: no switch");
        assert!(file(&ops, "class/gpio/gpio478/value").is_none());
        // The EEPROM of probe 2, through the i2c-dev link.
        assert_eq!(
            file(&ops, "class/i2c-dev/i2c-1/device"),
            Some(&Node::Link("../../i2c-adapter/i2c-1".into()))
        );
        let Some(Node::File { data, .. }) = file(&ops, "class/i2c-adapter/i2c-1/1-0051/eeprom") else {
            panic!("no EEPROM")
        };
        assert_eq!(data.len(), 256);
        assert_eq!(&data[..13], &[0, 0, 0, 1, 0, 0, 0, b'B', 0, 0, 0x4e, 0x20, 1]);
    }

    #[test]
    fn reads_pace_each_file_and_a_shunt_scales_current_and_power() {
        let (mut acme, _) = built(&[]);
        let p = "class/hwmon/hwmon0/power1_input";
        let mut fs = Fs::default();
        acme.read(p, &mut fs);
        assert_eq!(set_text(&mut fs, p), Some(format!("{}\n", Ina226Regs::at(0, 1).power_uw(10_000))));
        acme.read(p, &mut fs);
        assert_eq!(set_text(&mut fs, p), Some(format!("{}\n", Ina226Regs::at(0, 2).power_uw(10_000))));
        let s = "class/hwmon/hwmon0/shunt_resistor";
        assert_eq!(acme.written(s, b"20000\n", &mut fs), Written::Kept);
        assert_eq!(peek(&fs, s), Some("20000\n".into()));
        assert_eq!(set_text(&mut fs, p), Some(format!("{}\n", Ina226Regs::at(0, 2).power_uw(20_000))));
        for junk in [&b"0\n"[..], b"2500001\n", b"abc\n", b"-1\n", b"", b" 5\n"] {
            assert_eq!(acme.written(s, junk, &mut fs), Written::Refused("EINVAL"), "{junk:?}");
        }
        assert_eq!(acme.written(s, b"+2500000", &mut fs), Written::Kept);
    }

    #[test]
    fn gpios_behave_as_sysfs_and_a_sense_line_driven_is_a_violation() {
        let (mut acme, _) = built(&[("unexported", "486")]);
        let mut fs = Fs::default();
        assert_eq!(acme.written("class/gpio/gpio487/value", b"1\n", &mut fs), Written::Refused("EPERM"));
        assert_eq!(acme.written("class/gpio/export", b"487\n", &mut fs), Written::Refused("EBUSY"));
        assert_eq!(acme.written("class/gpio/export", b"x\n", &mut fs), Written::Refused("EINVAL"));
        assert_eq!(acme.written("class/gpio/export", b"486\n", &mut fs), Written::Kept);
        let (ops, _) = fs.take();
        assert_eq!(text(&ops, "class/gpio/gpio486/direction"), "out\n");
        assert_eq!(acme.written("class/gpio/gpio486/value", b"0\n", &mut fs), Written::Kept);
        assert_eq!(set_text(&mut fs, "class/gpio/gpio486/value"), Some("0\n".into()));
        assert_eq!(acme.written("class/gpio/gpio486/direction", b"high\n", &mut fs), Written::Kept);
        assert_eq!(set_text(&mut fs, "class/gpio/gpio486/value"), Some("1\n".into()));
        assert_eq!(acme.written("class/gpio/gpio486/direction", b"sideways\n", &mut fs), Written::Refused("EINVAL"));
        assert_eq!(acme.written("class/gpio/gpio487/direction", b"out\n", &mut fs), Written::Kept);
        let (_, said) = fs.take();
        assert_eq!(said.len(), 1, "{said:?}");
        assert!(said[0].contains("GPIO 487"), "{said:?}");
    }

    #[test]
    fn faults_shape_the_tree() {
        let (mut acme, ops) = built(&[
            ("absent", "hwmon/hwmon0/shunt_resistor"),
            ("value", "hwmon/hwmon0/in1_input:junk"),
            ("value", "hwmon/hwmon0/curr1_input:"),
            ("readerr", "hwmon/hwmon0/power1_input"),
            ("full", "hwmon/hwmon1/shunt_resistor"),
            ("ro", "hwmon/hwmon1/update_interval"),
            ("vanish", "hwmon/hwmon2/temp1_input@2"),
            ("hwmon-entry", "1:hwmonX"),
            ("hwmon-entry", "2:"),
        ]);
        assert!(file(&ops, "class/hwmon/hwmon0/shunt_resistor").is_none());
        assert_eq!(text(&ops, "class/hwmon/hwmon0/in1_input"), "junk\n");
        assert_eq!(text(&ops, "class/hwmon/hwmon0/curr1_input"), "");
        assert_eq!(file(&ops, "class/hwmon/hwmon0/power1_input"), Some(&Node::Dir));
        assert_eq!(file(&ops, "class/hwmon/hwmon1/shunt_resistor"), Some(&Node::Link("/dev/full".into())));
        assert!(matches!(file(&ops, "class/hwmon/hwmon1/update_interval"), Some(Node::File { mode: 0o444, .. })));
        assert_eq!(file(&ops, "class/i2c-adapter/i2c-1/1-0040/hwmon/hwmonX"), Some(&Node::Dir));
        assert_eq!(file(&ops, "class/i2c-adapter/i2c-1/1-0041/hwmon"), Some(&Node::Dir));
        let mut fs = Fs::default();
        acme.read("class/hwmon/hwmon0/in1_input", &mut fs);
        assert!(fs.ops().is_empty(), "a fixed value stays");
        let t = "class/hwmon/hwmon2/temp1_input";
        acme.read(t, &mut fs);
        assert!(matches!(fs.take().0.as_slice(), [Op::Set(..)]));
        acme.read(t, &mut fs);
        assert_eq!(fs.take().0, [Op::Remove(t.into())]);
    }

    #[test]
    fn options_are_checked() {
        for bad in [
            ("slots", "ina226"),
            ("slots", "tmp435,none,none,none,none,none,none,none"),
            ("revb", "9"),
            ("absent", "../etc/passwd"),
            ("absent", "hwmon/hwmon9/name"),
            ("value", "hwmon/hwmon0/in1_input"),
            ("vanish", "hwmon/hwmon0/in1_input"),
            ("shunt", "0"),
            ("pace", "fast"),
            ("hwmon-entry", "1:a/b"),
            ("nope", "1"),
        ] {
            assert!(Config::parse(&opts(&[bad])).is_err(), "{bad:?}");
        }
        assert!(build_sysfs(&[]).is_ok());
    }

    #[test]
    fn time_pacing_follows_the_update_interval() {
        let (mut acme, _) = built(&[("pace", "time")]);
        let mut fs = Fs::default();
        let t0 = Instant::now();
        acme.tick(t0, &mut fs);
        assert_eq!(fs.take().0, []);
        acme.read("class/hwmon/hwmon0/in1_input", &mut fs);
        assert_eq!(fs.take().0, [], "a read between conversions gets the same value");
        acme.tick(t0 + Duration::from_millis(34), &mut fs);
        assert_eq!(fs.take().0, []);
        acme.tick(t0 + Duration::from_millis(35), &mut fs);
        let p = "class/hwmon/hwmon0/in1_input";
        assert_eq!(set_text(&mut fs, p), Some(format!("{}\n", Ina226Regs::at(0, 1).bus_mv())));
    }

    #[test]
    fn an_unplug_empties_the_values_and_a_replug_gives_them_back() {
        let (mut acme, _) = built(&[]);
        let mut fs = Fs::default();
        acme.unplug(&mut fs);
        let p = "class/hwmon/hwmon0/in1_input";
        assert_eq!(peek(&fs, p), Some(String::new()));
        let _ = fs.take();
        acme.read(p, &mut fs);
        assert_eq!(fs.take().0, []);
        acme.replug(&mut fs);
        assert!(set_text(&mut fs, p).is_some_and(|t| !t.is_empty()));
    }

    #[test]
    fn eeprom_faults() {
        let c = |f| Config { eeprom_fault: Some(f), ..Config::default() };
        assert_eq!(eeprom(&c(EepromFault::Short), 1).len(), EEPROM_USED - 1);
        assert_eq!(eeprom(&c(EepromFault::Rev), 1)[7], b'A');
        assert_eq!(eeprom(&c(EepromFault::Type), 1)[3], 9);
    }
}
