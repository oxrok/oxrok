// Counterpart in sigrok:
//   libsigrok/src/hardware/scpi-pps/api.c,
//   libsigrok/src/hardware/scpi-pps/protocol.c,
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2017,2019 Frank Stettner <frank-stettner@gmx.net>,
//   Copyright (C) 2019 Frank Stettner <frank-stettner@gmx.net>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Martin Ling,
//   Aurelien Jacobs, Gerhard Sittig, Soeren Apel, Alexandru Gagniuc,
//   Florian Schmidt, Daniel Elstner, Kota Ando, Marc Schink, Martin Atkins
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! SCPI power supplies: `scpi-pps`, and `hpib-pps` for the HP 663xA that speak HP's
//! older language on the same line. One device for both, as in C, where a probe sets
//! `scpi_pps_driver_info` on what either scan finds (`api.c:97`), so a device of
//! `hpib-pps` calls itself `scpi-pps`, on both tools.
//!
//! [`profiles`] has what each supply has. A scan asks `*IDN?` (or `ID?` and `ROM?`, and
//! only with `conn=`), a probe for the outputs where a profile has one, says local, and
//! closes. An open says remote and turns a beeper that's on off; a close turns it on
//! again and says local. A setting is a line with no answer. Outputs are selected with
//! the table's command, and only when the name differs from the last one selected.
//! A capture asks for one channel in a pass, 10 ms apart at the least, with the
//! status first at the start of each round; a round is a sample. There is no block, no
//! `*OPC?` and no compound query on this wire but an HP's `STAT:OPER:PTR 3328;NTR
//! 3328;ENAB 3328`, which the instrument takes whole.
//!
//! An output is live, so what this does about a setting that didn't happen:
//!
//! - **A refused setting.** SCPI gives no answer to one, and the driver reads no error
//!   queue. So a value the instrument refuses still exits 0, in C as here. What can be
//!   said is said: a value outside what the output lists gets a warning and goes out
//!   anyway, as the tables are lower than some units go (a DP832 is listed at 30 V).
//!   `--get` asks the instrument each time, so it finds out.
//! - **A failed write** is an error saying the state is unknown, and the output's
//!   `enabled=on` that the CLI sends last doesn't go.
//! - **A failed selection** goes no further: what was to go to that output doesn't
//!   go, and the next one selects again. C remembers the name before the send, so its
//!   next setting would go to the wrong output, if its next `sr_scpi_cmd` didn't hang
//!   on the mutex that error path leaves locked (`scpi.c:1324-1325`).
//! - **A close** forgets the selection, so a later open selects again, as an operator
//!   may have changed it. C keeps it (`actual_channel_name`).
//! - No group means no output: a library client calling [`Device::set`] with a group's
//!   key and no group is refused here as `check_key` refuses it in C, as it would go
//!   to whichever output the instrument has selected. A name that is no group too.
//!
//! Where this leaves `api.c`, `protocol.c` and `profiles.c`:
//!
//! - **The list of a probed supply's output** (Chroma 62000, EA, EEZ, PM2800) reads what
//!   the probe found. C reads the profile's null table and dies (`api.c:814`).
//! - **`:CURR:PROT:STAT ON` on an N5700**, where `profiles.c:82-83` has `ON?` and `OFF?`,
//!   a typo since the profile came in 2015. A boolean with a question mark is an error
//!   on any SCPI parser, so C's OCP key does nothing, unsaid. No unit tried this.
//! - **An EA's nominal values**: see [`profiles`]. C has them all as 0.
//! - **A value in a reply**: see [`protocol::number`], [`protocol::status_word`] and
//!   [`protocol::state`], each stricter than what C calls. An empty answer is no
//!   reading (0 in C) and no status (`UR` on an HP 66xxB); a status that is no number
//!   is an error, where C goes on with a variable it never set (`api.c:526` and on).
//! - **NaN and infinities** are refused before the wire. C sends `:SOUR:VOLT nan`.
//! - **A regulation query that fails** is an error. C reads the variant it never got
//!   (`api.c:514` and on).
//! - **A capture that hears nothing** is bounded: see [`protocol::Watch`]. C asks for
//!   ever, or blocks for ever over TCP, and with a failed read returns ahead of its
//!   limit check (`protocol.c:97-98`), so even a time limit doesn't end it. A time
//!   limit ends it here, with a warning if nothing came and exit 0.
//! - **A connection that closed or a device gone** ends a capture with an error. C goes
//!   on reading 0 bytes, or dies of SIGPIPE at the next write.
//! - **One question out at a time** on a line with no tags: see [`protocol`]. Over TCP
//!   that is C, which blocks. Over a serial port C asks again after 1 s: on the model,
//!   which drops an answer nobody read at the next command, it loses that reading and
//!   stays in step, so an instrument always slower than 1 s gives it none. On one that
//!   queues its answers instead, each reading would be the one before's: that is from
//!   reading the code, with no run behind it.
//! - **A setting stays a double**, as in the driver. sigrok-cli's `strtof` narrows it
//!   first (`session.c:570`), so 12.345678 goes out from sigrok-cli as 12.345678 and
//!   from here as 12.345678, but 123.456789 as 123.456787 and 123.456789.
//! - **With no `conn=`** only USBTMC devices are probed. C also opens serial ports by a
//!   table of USB ids (`scpi_serial.c:43-52`).
//!
//! What's kept as in C: the channel of a capture stays where it is on a failed read,
//! so the next ask is for it again. `update_status`' result is dropped. `init_acquisition`'s
//! too. An EEZ's probe selects with a plain send, which leaves the remembered name
//! alone.

#![forbid(unsafe_code)]

pub mod profiles;
pub mod protocol;

use crate::core::{
    AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error,
    Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::scpi::{self, CmdArg, HwInfo, Resource, Scpi};
use profiles::{ChannelSpec, Dialect, GroupSpec, Profile};
use protocol::{Cmd, LIMIT_GRACE, Link, Reading, Verdict, Watch};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::PowerSupply];
/// `sr_scpi_source_add`'s 10 ms: a pass of the capture at most so often.
const PACE: Duration = Duration::from_millis(10);

/// The `scpi-pps` driver.
pub struct ScpiPpsDriver;
/// The `hpib-pps` driver.
pub struct HpibPpsDriver;

/// `hpib_get_hw_id`: an HP says only `HP6632A`, and what it says goes as the model.
fn hpib_hw_id(scpi: &mut Scpi) -> Result<HwInfo> {
    let model = scpi.get_string("ID?")?;
    let firmware_version = protocol::hpib_revision(scpi).unwrap_or_default();
    Ok(HwInfo {
        manufacturer: "HP".to_owned(),
        model,
        serial_number: String::new(),
        firmware_version,
    })
}

/// What a probe learns from an open line.
type Probed = (&'static Profile, HwInfo, Vec<ChannelSpec>, Vec<GroupSpec>);

/// `probe_device`, but for the close that `sr_scpi_scan_resource` does after it.
fn probe_open_with(scpi: &mut Scpi, hpib: bool) -> Option<Probed> {
    let id = if hpib { hpib_hw_id(scpi) } else { scpi.hw_id() };
    let id = match id {
        Ok(id) => id,
        Err(e) => {
            log::info!("Couldn't get IDN response.");
            log::debug!("{e}");
            return None;
        }
    };
    let (profile, vendor) = profiles::find(&id)?;
    let id = HwInfo { manufacturer: vendor.to_owned(), ..id.clone() };
    let (specs, groups) = match profiles::outputs(profile, &id, scpi) {
        Ok(outputs) => outputs,
        Err(e) => {
            log::debug!("{e}");
            log::error!("Failed to probe for channels.");
            return None;
        }
    };
    if let Some(local) = scpi::cmd_get(profile.commands, &Cmd::Local)
        && let Err(e) = scpi.send(local)
    {
        log::debug!("{local}: {e}.");
    }
    Some((profile, id, specs, groups))
}

fn scan(options: &[(Key, Value)], hpib: bool) -> Result<Vec<Box<dyn Device>>> {
    let mut conn = None;
    let mut serialcomm = None;
    for (key, value) in options {
        match key {
            Key::Conn => conn = value.as_str().map(str::to_owned),
            Key::Serialcomm => serialcomm = value.as_str().map(str::to_owned),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
    }
    // Only with `conn=`, "to not break SCPI devices' operation".
    if hpib && conn.is_none() {
        return Ok(Vec::new());
    }
    let comm = serialcomm.as_deref();
    let found = scpi::scan(conn.as_deref(), comm, |mut s| {
        let found = probe_open_with(&mut s, hpib);
        let _ = s.close();
        Ok(found.map(|(profile, id, specs, groups)| {
            PpsDevice::new(profile, &id, specs, groups, s, comm, hpib)
        }))
    });
    let found = found.unwrap_or_else(|e| {
        log::info!("{e}");
        Vec::new()
    });
    Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
}

impl Driver for ScpiPpsDriver {
    fn id(&self) -> &'static str {
        "scpi-pps"
    }

    fn long_name(&self) -> &'static str {
        "SCPI PPS"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        scan(options, false)
    }
}

impl Driver for HpibPpsDriver {
    fn id(&self) -> &'static str {
        "hpib-pps"
    }

    fn long_name(&self) -> &'static str {
        "HP-IB PPS"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        scan(options, true)
    }
}

/// What a channel of a capture reads.
#[derive(Clone, Debug)]
struct Plan {
    /// Its position in the device's channels.
    index: usize,
    /// The output's name, which selects it.
    output: &'static str,
    command: Cmd,
    mq: Mq,
    unit: Unit,
    flags: MqFlags,
    digits: i32,
}

/// The sample and time limits, shared with a capture as `devc->limits` is in C.
#[derive(Default)]
struct Limits {
    samples: AtomicU64,
    msec: AtomicU64,
}

struct Worker {
    stop: Arc<AtomicBool>,
    /// True until the thread starts to send its end.
    running: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl Worker {
    fn finish(self) {
        self.stop.store(true, Ordering::SeqCst);
        if self.handle.join().is_err() {
            log::warn!("The capture thread panicked.");
        }
    }
}

/// One power supply.
pub struct PpsDevice {
    info: DeviceInfo,
    profile: &'static Profile,
    specs: Vec<ChannelSpec>,
    group_specs: Vec<GroupSpec>,
    channels: Vec<Channel>,
    plans: Vec<Plan>,
    groups: Vec<ChannelGroup>,
    resource: String,
    serialcomm: Option<String>,
    limits: Arc<Limits>,
    link: Arc<Mutex<Link>>,
    /// The scan's line, closed, which an open opens again.
    closed: Option<Scpi>,
    beeper_was_set: bool,
    worker: Option<Worker>,
    /// Bounds for a test, in place of the constants.
    #[cfg(test)]
    watch: Option<Watch>,
}

impl PpsDevice {
    fn new(
        profile: &'static Profile,
        id: &HwInfo,
        specs: Vec<ChannelSpec>,
        group_specs: Vec<GroupSpec>,
        scpi: Scpi,
        serialcomm: Option<&str>,
        _hpib: bool,
    ) -> PpsDevice {
        let has = |command: Cmd| scpi::cmd_get(profile.commands, &command).is_some();
        let ac = group_specs.iter().any(|g| g.ac);
        let dc_or_ac = if ac { MqFlags::AC } else { MqFlags::DC };
        // Volts and amperes alone take a group's flag (`api.c:154-160`).
        let flags = if group_specs.is_empty() { MqFlags::empty() } else { dc_or_ac };
        let kinds = [
            (Cmd::GetMeasVoltage, "V", Mq::Voltage, Unit::Volt, flags),
            (Cmd::GetMeasCurrent, "I", Mq::Current, Unit::Ampere, flags),
            (Cmd::GetMeasPower, "P", Mq::Power, Unit::Watt, MqFlags::empty()),
            (Cmd::GetMeasFrequency, "F", Mq::Frequency, Unit::Hertz, MqFlags::empty()),
        ];
        let mut channels = Vec::new();
        let mut plans = Vec::new();
        for spec in &specs {
            for (command, prefix, mq, unit, flags) in kinds {
                if !has(command) {
                    continue;
                }
                let index = channels.len();
                channels.push(Channel::new(
                    index,
                    ChannelKind::Analog,
                    format!("{prefix}{}", spec.name),
                ));
                let digits = match mq {
                    Mq::Voltage => spec.voltage[4],
                    Mq::Current => spec.current[4],
                    Mq::Power => spec.power[4],
                    _ => spec.frequency[4],
                };
                plans.push(Plan {
                    index,
                    output: spec.name,
                    command,
                    mq,
                    unit,
                    flags,
                    digits: digits as i32,
                });
            }
        }
        let groups = group_specs
            .iter()
            .map(|g| {
                let output = specs[g.channel].name;
                let members = plans.iter().filter(|p| p.output == output).map(|p| p.index);
                ChannelGroup::new(g.name, members.collect())
            })
            .collect();
        let resource = scpi.resource().to_owned();
        PpsDevice {
            info: DeviceInfo {
                driver: "scpi-pps",
                vendor: id.manufacturer.clone(),
                model: id.model.clone(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            profile,
            specs,
            group_specs,
            channels,
            plans,
            groups,
            resource,
            serialcomm: serialcomm.map(str::to_owned),
            limits: Arc::new(Limits::default()),
            link: Arc::new(Mutex::new(Link::new(profile))),
            closed: Some(scpi),
            beeper_was_set: false,
            worker: None,
            #[cfg(test)]
            watch: None,
        }
    }

    /// A device a scan of `scpi` found; [`Device::open`] takes the line. For tests and
    /// for clients with a transport of their own.
    ///
    /// # Errors
    ///
    /// When it answers nothing or is no profile's.
    pub fn from_scpi(mut scpi: Scpi, hpib: bool) -> Result<Self> {
        let found = probe_open_with(&mut scpi, hpib);
        let _ = scpi.close();
        let (profile, id, specs, groups) =
            found.ok_or_else(|| Error::device("No supply this driver knows."))?;
        Ok(Self::new(profile, &id, specs, groups, scpi, None, hpib))
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    /// The output of `group`.
    fn spec_of(&self, group: &str) -> Result<&ChannelSpec> {
        let spec = self.group_specs.iter().find(|g| g.name == group);
        spec.map(|g| &self.specs[g.channel]).ok_or_else(|| Error::ChannelGroup(group.to_owned()))
    }

    /// The option `key` of `group`, or an error before anything goes on the wire.
    fn option(&self, key: &Key, group: Option<&str>) -> Result<OptionDesc> {
        let options = self.options(group)?;
        options
            .into_iter()
            .find(|o| &o.key == key)
            .ok_or_else(|| Error::UnknownKey(key.id().to_owned()))
    }

    /// A setting of an output's set point: warns of a value outside what it lists. The
    /// result of a set says nothing of whether it was taken, so this is all there is.
    fn check_range(&self, key: &Key, value: f64, group: Option<&str>) {
        let Some(spec) = group.and_then(|g| self.spec_of(g).ok()) else { return };
        let range = match key {
            Key::VoltageTarget => spec.voltage,
            Key::CurrentLimit => spec.current,
            Key::OutputFrequencyTarget => spec.frequency,
            Key::OverVoltageProtectionThreshold => spec.ovp,
            Key::OverCurrentProtectionThreshold => spec.ocp,
            Key::OverCurrentProtectionDelay => spec.ocp_delay,
            _ => return,
        };
        let (low, high) = (range[0], range[1]);
        if (low != 0.0 || high != 0.0) && !(low..=high).contains(&value) {
            log::warn!(
                "{} {value} is outside the {low} to {high} this output lists: the instrument \
                 may refuse it, and no answer would say so.",
                key.id()
            );
        }
    }
}

fn number_of(key: &Key, value: &Value) -> Result<f64> {
    let v = value.as_f64().ok_or_else(|| Error::Arg(format!("{} takes a number", key.id())))?;
    if v.is_finite() { Ok(v) } else { Err(Error::Arg(format!("{} can't be {v}", key.id()))) }
}

fn bool_of(key: &Key, value: &Value) -> Result<bool> {
    value.as_bool().ok_or_else(|| Error::Arg(format!("{} takes on or off", key.id())))
}

/// How a get's answer reads.
#[derive(Clone, Copy)]
enum Form {
    Number,
    State,
    Regulation,
    /// A status register's bit in dialects that have one, else a state.
    Active(u32, u32, u32),
}

/// What to ask for a get of `key`, in the case that it's a command.
fn get_command(key: &Key) -> Option<(Cmd, Form)> {
    Some(match key {
        Key::Enabled => (Cmd::GetOutputEnabled, Form::State),
        Key::Voltage => (Cmd::GetMeasVoltage, Form::Number),
        Key::VoltageTarget => (Cmd::GetVoltageTarget, Form::Number),
        Key::OutputFrequency => (Cmd::GetMeasFrequency, Form::Number),
        Key::OutputFrequencyTarget => (Cmd::GetFrequencyTarget, Form::Number),
        Key::Current => (Cmd::GetMeasCurrent, Form::Number),
        Key::CurrentLimit => (Cmd::GetCurrentLimit, Form::Number),
        Key::OverVoltageProtectionEnabled => (Cmd::GetOverVoltageProtectionEnabled, Form::State),
        Key::OverVoltageProtectionActive => {
            (Cmd::GetOverVoltageProtectionActive, Form::Active(3, 0, u32::MAX))
        }
        Key::OverVoltageProtectionThreshold => {
            (Cmd::GetOverVoltageProtectionThreshold, Form::Number)
        }
        Key::OverCurrentProtectionEnabled => (Cmd::GetOverCurrentProtectionEnabled, Form::State),
        Key::OverCurrentProtectionActive => {
            (Cmd::GetOverCurrentProtectionActive, Form::Active(6, 1, u32::MAX))
        }
        Key::OverCurrentProtectionThreshold => {
            (Cmd::GetOverCurrentProtectionThreshold, Form::Number)
        }
        Key::OverCurrentProtectionDelay => (Cmd::GetOverCurrentProtectionDelay, Form::Number),
        Key::OverTemperatureProtection => (Cmd::GetOverTemperatureProtection, Form::State),
        Key::OverTemperatureProtectionActive => {
            (Cmd::GetOverTemperatureProtectionActive, Form::Active(4, 4, 4))
        }
        Key::Regulation => (Cmd::GetOutputRegulation, Form::Regulation),
        _ => return None,
    })
}

/// An on or off that an HP or an HMP has as a bit of a register: `api.c:592-646`.
fn active(dialect: Dialect, bits: (u32, u32, u32), answer: &str) -> Result<bool> {
    let bit = match dialect {
        Dialect::HpComp => Some(bits.0),
        Dialect::Hp66xxb => Some(bits.1),
        Dialect::Hmp | Dialect::KeysightE36300a if bits.2 != u32::MAX => Some(bits.2),
        _ => None,
    };
    match bit {
        Some(bit) => Ok(protocol::status_word(answer)? & (1 << bit) != 0),
        None => protocol::state(answer),
    }
}

/// What a get has to say, from the answer.
fn read_answer(dialect: Dialect, form: Form, answer: &str) -> Result<Value> {
    match form {
        Form::Number => protocol::number(answer)
            .map(Value::F64)
            .ok_or_else(|| Error::device(format!("SCPI: not a number: '{answer}'"))),
        Form::State => protocol::state(answer).map(Value::Bool),
        Form::Regulation => protocol::regulation(dialect, answer).map(Value::Str),
        Form::Active(comp, b, hmp) => active(dialect, (comp, b, hmp), answer).map(Value::Bool),
    }
}

impl Device for PpsDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &self.groups
    }

    fn enable_channel(&mut self, pos: usize, enabled: bool) -> Result<()> {
        let channel =
            self.channels.get_mut(pos).ok_or_else(|| Error::Arg(format!("No channel {pos}.")))?;
        channel.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let channel =
            self.channels.get_mut(pos).ok_or_else(|| Error::Arg(format!("No channel {pos}.")))?;
        name.clone_into(&mut channel.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(match group {
            None => self.profile.device_options.to_vec(),
            Some(g) => {
                self.spec_of(g)?;
                self.profile.options.to_vec()
            }
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if !self.option(key, group)?.access.get {
            return Err(Error::UnknownKey(key.id().to_owned()));
        }
        match key {
            Key::LimitSamples => {
                return Ok(Value::U64(self.limits.samples.load(Ordering::SeqCst)));
            }
            Key::LimitMsec => return Ok(Value::U64(self.limits.msec.load(Ordering::SeqCst))),
            _ => {}
        }
        let dialect = self.profile.dialect;
        // An HMP's are always on, with nothing to ask.
        if dialect == Dialect::Hmp
            && matches!(key, Key::OverVoltageProtectionEnabled | Key::OverTemperatureProtection)
        {
            return Ok(Value::Bool(true));
        }
        let (command, form) =
            get_command(key).ok_or_else(|| Error::UnknownKey(key.id().to_owned()))?;
        // These take the output in the query itself, and select nothing.
        let in_query = matches!(dialect, Dialect::Hmp | Dialect::KeysightE36300a)
            && matches!(key, Key::Regulation | Key::OverTemperatureProtectionActive);
        let mut link = self.link();
        let answer = if in_query {
            let Some(name) = group else {
                log::error!("Need a channel group for regulation or OTP-active query.");
                return Err(Error::UnknownKey(key.id().to_owned()));
            };
            link.query(None, command, &[CmdArg::Str(name)])?
        } else {
            link.query(group, command, &[])?
        };
        read_answer(dialect, form, &answer)
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if !self.option(key, group)?.access.set {
            return Err(Error::UnknownKey(key.id().to_owned()));
        }
        let (command, number) = match key {
            Key::LimitSamples | Key::LimitMsec => {
                let n = value
                    .as_u64()
                    .filter(|n| *n > 0)
                    .ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())))?;
                let slot = if *key == Key::LimitSamples {
                    &self.limits.samples
                } else {
                    &self.limits.msec
                };
                slot.store(n, Ordering::SeqCst);
                return Ok(());
            }
            Key::Enabled if bool_of(key, value)? => (Cmd::SetOutputEnable, None),
            Key::Enabled => (Cmd::SetOutputDisable, None),
            Key::OverVoltageProtectionEnabled if bool_of(key, value)? => {
                (Cmd::SetOverVoltageProtectionEnable, None)
            }
            Key::OverVoltageProtectionEnabled => (Cmd::SetOverVoltageProtectionDisable, None),
            Key::OverCurrentProtectionEnabled if bool_of(key, value)? => {
                (Cmd::SetOverCurrentProtectionEnable, None)
            }
            Key::OverCurrentProtectionEnabled => (Cmd::SetOverCurrentProtectionDisable, None),
            Key::OverTemperatureProtection if bool_of(key, value)? => {
                (Cmd::SetOverTemperatureProtectionEnable, None)
            }
            Key::OverTemperatureProtection => (Cmd::SetOverTemperatureProtectionDisable, None),
            Key::VoltageTarget => (Cmd::SetVoltageTarget, Some(number_of(key, value)?)),
            Key::OutputFrequencyTarget => (Cmd::SetFrequencyTarget, Some(number_of(key, value)?)),
            Key::CurrentLimit => (Cmd::SetCurrentLimit, Some(number_of(key, value)?)),
            Key::OverVoltageProtectionThreshold => {
                (Cmd::SetOverVoltageProtectionThreshold, Some(number_of(key, value)?))
            }
            Key::OverCurrentProtectionThreshold => {
                (Cmd::SetOverCurrentProtectionThreshold, Some(number_of(key, value)?))
            }
            Key::OverCurrentProtectionDelay => {
                (Cmd::SetOverCurrentProtectionDelay, Some(number_of(key, value)?))
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        };
        if let Some(number) = number {
            self.check_range(key, number, group);
        }
        let args: Vec<CmdArg<'_>> = number.map(CmdArg::Float).into_iter().collect();
        self.link().cmd(group, command, &args).map_err(|e| {
            let on = group.map_or(String::new(), |g| format!(" on output {g}"));
            Error::device(format!(
                "Setting {}{on} failed, so whether it took effect isn't known: {e}",
                key.id()
            ))
        })
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let spec = group.map(|g| self.spec_of(g)).transpose()?;
        let unknown = || Error::UnknownKey(key.id().to_owned());
        let spec = spec.ok_or_else(unknown)?;
        if !self.option(key, group)?.access.list {
            return Err(unknown());
        }
        let range = match key {
            Key::VoltageTarget => spec.voltage,
            Key::OutputFrequencyTarget => spec.frequency,
            Key::CurrentLimit => spec.current,
            Key::OverVoltageProtectionThreshold => spec.ovp,
            Key::OverCurrentProtectionThreshold => spec.ocp,
            Key::OverCurrentProtectionDelay => spec.ocp_delay,
            _ => return Err(unknown()),
        };
        Ok(ValueList::RangeStepF64 { min: range[0], max: range[1], step: range[2] })
    }

    /// `dev_open`: remote, and a beeper that is on goes off.
    fn open(&mut self) -> Result<()> {
        if self.link().scpi.is_some() {
            return Ok(());
        }
        let mut scpi = if let Some(scpi) = self.closed.take() {
            scpi
        } else {
            let resource = Resource::parse(&self.resource)?;
            scpi::open_keeping_line(&resource, self.serialcomm.as_deref())?
        };
        if let Err(e) = scpi.reopen() {
            self.closed = Some(scpi);
            return Err(e);
        }
        let mut link = self.link.lock().expect("link lock");
        link.scpi = Some(scpi);
        if let Err(e) = link.cmd(None, Cmd::Remote, &[]) {
            log::debug!("Remote: {e}.");
        }
        self.beeper_was_set = false;
        // A profile with no beeper has no such command, and sends nothing.
        if let Ok(answer) = link.query(None, Cmd::Beeper, &[])
            && matches!(protocol::state(&answer), Ok(true))
        {
            self.beeper_was_set = true;
            if let Err(e) = link.cmd(None, Cmd::BeeperDisable, &[]) {
                log::debug!("Beeper off: {e}.");
            }
        }
        Ok(())
    }

    /// `dev_close`: what the open changed goes back, and the selection is forgotten.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        let mut link = self.link.lock().expect("link lock");
        if link.scpi.is_none() {
            return Ok(());
        }
        if self.beeper_was_set
            && let Err(e) = link.cmd(None, Cmd::BeeperEnable, &[])
        {
            log::debug!("Beeper on: {e}.");
        }
        if let Err(e) = link.cmd(None, Cmd::Local, &[]) {
            log::debug!("Local: {e}.");
        }
        if let Some(mut scpi) = link.scpi.take() {
            scpi.forget_channel();
            let _ = scpi.close();
            self.closed = Some(scpi);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(worker) = self.worker.take() {
            if worker.running.load(Ordering::SeqCst) {
                self.worker = Some(worker);
                return Err(Error::Busy);
            }
            // Its end may be stuck on a full queue: this gives that up.
            worker.finish();
        }
        if self.link().scpi.is_none() {
            return Err(Error::device("device is not open"));
        }
        let enabled = |p: &&Plan| self.channels[p.index].enabled;
        let plans: Vec<Plan> = self.plans.iter().filter(enabled).cloned().collect();
        if plans.is_empty() {
            return Err(Error::arg("No channels enabled."));
        }
        let stop = Arc::new(AtomicBool::new(false));
        let running = Arc::new(AtomicBool::new(true));
        #[cfg(test)]
        let watch = self.watch.clone();
        #[cfg(not(test))]
        let watch: Option<Watch> = None;
        let capture = Capture {
            link: Arc::clone(&self.link),
            limits: Arc::clone(&self.limits),
            plans,
            select: self.groups.len() > 1,
            stop: Arc::clone(&stop),
            whole_device: self.plans.len() > 1,
            watch,
        };
        let (stop2, running2) = (Arc::clone(&stop), Arc::clone(&running));
        let handle = std::thread::Builder::new()
            .name("scpi-pps capture".to_owned())
            .spawn(move || {
                let outcome = capture.run(&sink);
                if let Err(e) = &outcome {
                    log::error!("{e}");
                }
                running2.store(false, Ordering::SeqCst);
                sink.end_while(outcome, || !stop2.load(Ordering::SeqCst));
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, running, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = self.worker.take() {
            worker.finish();
        }
        Ok(())
    }
}

/// One capture: `dev_acquisition_start` and `scpi_pps_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limits: Arc<Limits>,
    plans: Vec<Plan>,
    /// More than one group: each read selects its output.
    select: bool,
    stop: Arc<AtomicBool>,
    whole_device: bool,
    watch: Option<Watch>,
}

impl Capture {
    fn stopped(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    fn time_up(&self, began: Instant) -> bool {
        let msec = self.limits.msec.load(Ordering::SeqCst);
        msec > 0 && began.elapsed() >= Duration::from_millis(msec)
    }

    fn run(&self, sink: &PacketSink) -> Result<()> {
        let go_on = || !self.stopped();
        self.link.lock().expect("link lock").init_acquisition();
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let began = Instant::now();
        let time_limited = self.limits.msec.load(Ordering::SeqCst) > 0;
        let mut watch = self.watch.clone().unwrap_or_else(|| Watch::new(time_limited));
        let mut at = 0;
        let mut samples = 0u64;
        let mut last_pass = None::<Instant>;
        loop {
            if self.stopped() {
                return Ok(());
            }
            if let Some(last) = last_pass {
                let left = PACE.saturating_sub(last.elapsed());
                if !left.is_zero() {
                    std::thread::sleep(left);
                }
            }
            last_pass = Some(Instant::now());
            let plan = &self.plans[at];
            let mut link = self.link.lock().expect("link lock");
            // A status belongs with a new round, not with a question still out.
            let status = if at == 0 && !link.owed() { link.update_status() } else { Vec::new() };
            let channel = self.select.then_some(plan.output);
            let asked = link.scpi.as_ref().map_or(PACE, |s| s.read_timeout);
            let before = Instant::now();
            let reading = link.measure(channel, plan.command);
            let took = before.elapsed();
            let owed = link.owed();
            drop(link);
            for packet in status {
                if !sink.send_while(packet, go_on) {
                    return Ok(());
                }
            }
            let verdict = match reading? {
                Reading::Value(value) => {
                    if let Some(line) = watch.sample() {
                        log::warn!("{line}");
                    }
                    let packet = Packet::Analog(AnalogData {
                        channels: vec![plan.index],
                        samples: vec![value as f32],
                        mq: plan.mq,
                        flags: plan.flags,
                        unit: plan.unit,
                        digits: plan.digits,
                    });
                    if !sink.send_while(packet, go_on) {
                        return Ok(());
                    }
                    at = (at + 1) % self.plans.len();
                    if !self.whole_device {
                        at = 0;
                    }
                    if at == 0 {
                        samples += 1;
                    }
                    let limit = self.limits.samples.load(Ordering::SeqCst);
                    if (limit > 0 && samples >= limit) || self.time_up(began) {
                        return Ok(());
                    }
                    continue;
                }
                Reading::Silent => watch.nothing(took, asked),
                Reading::Junk(text) => {
                    log::debug!("No reading in '{text}'.");
                    watch.no_reading(took, asked)
                }
            };
            match verdict {
                Verdict::Go => {}
                Verdict::Warn(text) => log::warn!("{text}"),
                Verdict::Fail(text) => return Err(Error::device(text)),
            }
            // A question that's out may have its answer a moment past the limit.
            let past = self.time_up(began)
                && (!owed
                    || began.elapsed()
                        >= Duration::from_millis(self.limits.msec.load(Ordering::SeqCst))
                            + LIMIT_GRACE);
            if past {
                if let Some(line) = watch.at_time_limit() {
                    log::warn!("{line}");
                }
                return Ok(());
            }
        }
    }
}

#[cfg(test)]
mod tests;
