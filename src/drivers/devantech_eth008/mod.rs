// Counterpart in sigrok:
//   libsigrok/src/hardware/devantech-eth008/api.c,
//   libsigrok/src/hardware/devantech-eth008/protocol.c,
//   libsigrok/src/hardware/devantech-eth008/protocol.h,
//   Copyright (C) 2023 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Devantech relay boards: ETH002, ETH008, ETH484, ETH8020, ETH1610 and the
//! WIFI002, WIFI008, WIFI484 and WIFI8020. A board is reached over TCP,
//! `conn=tcp-raw/HOST/17494`, or a serial port carrying the same bytes.
//!
//! There's no capture. A board is a set of channel groups with no channels:
//! `DO1` and up switch one relay or output (`enabled`), `DI1` and up read an
//! input (`enabled`), `AI1` and up read an analog input (`voltage`, a number with
//! no unit, as models differ) and `Vsupply` the relays' supply (`voltage`, in V).
//! `enabled` with no group switches all the outputs. No value is kept: a get
//! asks the board, since up to five connections can change it at any time.
//!
//! A scan asks the module info and the serial number. The port then closes, and
//! an open opens it again. C leaves it open (no `serial_close` before
//! `api.c:191` returns) and opens it a second time: over TCP a second connection,
//! with the first never closed. On a tty C doesn't get that far. `api.c:105` opens
//! with no flags, which libserialport takes as read only, so the scan's first
//! write fails with EBADF and no board is ever found; and were it found, the
//! exclusive open of libserialport 0.1.2 and later would refuse the second open.
//! Here a tty works like TCP.
//!
//! An ETH484 has relays 1 to 4 in the first byte of its image, and 8 outputs in
//! the second, so its groups are `DO1` to `DO4` and `DO9` to `DO16`.
//!
//! Not done, as in C: passwords, pulsed outputs and the 12 bit analog command.
//! The module's id says what a board has, so its firmware version is only shown.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

#[cfg(test)]
mod tests;

use crate::core::{
    Access, Channel, ChannelGroup, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{Link, Model, TIMING};
use std::sync::{Mutex, MutexGuard};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multiplexer];
const VENDOR: &str = "Devantech";
/// C opens with no `serialcomm` and leaves the port as it is. This sets the
/// line, as [`crate::serial::open`] always does; TCP takes no notice of it.
const SERIALCOMM: &str = "9600/8n1";

const ACCESS_SET: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 2] =
    [OptionDesc::new(Key::Conn, Access::GET), OptionDesc::new(Key::Enabled, ACCESS_SET)];
static OUTPUT_OPTIONS: [OptionDesc; 1] = [OptionDesc::new(Key::Enabled, Access::GET_SET)];
static INPUT_OPTIONS: [OptionDesc; 1] = [OptionDesc::new(Key::Enabled, Access::GET)];
static VOLTAGE_OPTIONS: [OptionDesc; 1] = [OptionDesc::new(Key::Voltage, Access::GET)];

/// The `devantech-eth008` driver.
pub struct DevantechEth008Driver;

impl Driver for DevantechEth008Driver {
    fn id(&self) -> &'static str {
        "devantech-eth008"
    }

    fn long_name(&self) -> &'static str {
        "Devantech ETH008"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn.filter(|c| !c.is_empty()) else {
            return Ok(Vec::new());
        };
        // A port that doesn't open is no device.
        let port = match crate::serial::open(&conn, serialcomm()?) {
            Ok(port) => port,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        match probe(port, &conn) {
            Ok(Some(device)) => Ok(vec![Box::new(device)]),
            Ok(None) => Ok(Vec::new()),
            Err(e) => {
                log::debug!("Scan failed: {e}.");
                Ok(Vec::new())
            }
        }
    }
}

fn serialcomm() -> Result<Params> {
    Params::parse(SERIALCOMM)
}

/// `probe_device_conn` after the open. The port goes when this returns.
fn probe(port: Box<dyn Port>, conn: &str) -> Result<Option<EthDevice>> {
    let mut link = Link::new(Some(port), TIMING);
    let (code, hardware, firmware) = link.get_model()?;
    let Some(model) = Model::find(code) else {
        log::error!("Unknown model ID 0x{code:02x} (HW {hardware}, FW {firmware}).");
        return Ok(None);
    };
    let serial = link.get_serial()?;
    link.port = None;
    let version = format!("HW{hardware} FW{firmware}");
    Ok(Some(EthDevice::new(model, link, conn.to_owned(), version, serial)))
}

/// What a group is, and the number the board knows it by.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Role {
    Output(u8),
    Input(u8),
    Analog(u8),
    Supply,
}

/// One board.
pub struct EthDevice {
    info: DeviceInfo,
    model: &'static Model,
    conn: String,
    groups: Vec<ChannelGroup>,
    roles: Vec<Role>,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Mutex<Link>,
}

/// The groups `probe_device_conn` makes.
fn groups_of(model: &Model) -> (Vec<ChannelGroup>, Vec<Role>) {
    let mut groups = Vec::new();
    let mut roles = Vec::new();
    let mut add = |name: String, role: Role| {
        groups.push(ChannelGroup::new(name, Vec::new()));
        roles.push(role);
    };
    for number in numbers(model.outputs) {
        if model.output_mask() & 1 << (number - 1) != 0 {
            add(format!("DO{number}"), Role::Output(number));
        }
    }
    for number in numbers(model.inputs) {
        add(format!("DI{number}"), Role::Input(number));
    }
    for number in numbers(model.analog) {
        add(format!("AI{number}"), Role::Analog(number));
    }
    add("Vsupply".to_owned(), Role::Supply);
    (groups, roles)
}

/// 1 to `count`. A table's count is 20 at most.
fn numbers(count: usize) -> impl Iterator<Item = u8> {
    1..=u8::try_from(count).unwrap_or(u8::MAX)
}

impl EthDevice {
    fn new(
        model: &'static Model,
        link: Link,
        conn: String,
        version: String,
        serial: String,
    ) -> EthDevice {
        let (groups, roles) = groups_of(model);
        EthDevice {
            info: DeviceInfo {
                driver: "devantech-eth008",
                vendor: VENDOR.to_owned(),
                model: model.name.to_owned(),
                version,
                serial,
                conn: Some(conn.clone()),
            },
            model,
            conn,
            groups,
            roles,
            pending: None,
            link: Mutex::new(link),
        }
    }

    /// A board on a port already open, for tests.
    pub fn from_port(model: &'static Model, port: Box<dyn Port>) -> EthDevice {
        let mut device = EthDevice::new(
            model,
            Link::new(None, TIMING),
            "test".to_owned(),
            String::new(),
            String::new(),
        );
        device.pending = Some(port);
        device
    }

    fn link(&self) -> MutexGuard<'_, Link> {
        self.link.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn role(&self, group: Option<&str>) -> Result<Option<Role>> {
        let Some(name) = group else { return Ok(None) };
        let at = self
            .groups
            .iter()
            .position(|g| g.name == name)
            .ok_or_else(|| Error::ChannelGroup(name.to_owned()))?;
        Ok(Some(self.roles[at]))
    }
}

impl Device for EthDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &[]
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &self.groups
    }

    fn enable_channel(&mut self, pos: usize, _enabled: bool) -> Result<()> {
        Err(Error::Arg(format!("No channel {pos}.")))
    }

    fn rename_channel(&mut self, pos: usize, _name: &str) -> Result<()> {
        Err(Error::Arg(format!("No channel {pos}.")))
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(match self.role(group)? {
            None => DEV_OPTIONS.to_vec(),
            Some(Role::Output(_)) => OUTPUT_OPTIONS.to_vec(),
            Some(Role::Input(_)) => INPUT_OPTIONS.to_vec(),
            Some(Role::Analog(_) | Role::Supply) => VOLTAGE_OPTIONS.to_vec(),
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let role = self.role(group)?;
        match (key, role) {
            (Key::Conn, None) => Ok(Value::Str(self.conn.clone())),
            (Key::Enabled, Some(Role::Output(number))) => {
                let (outputs, _) = self.link().read_state(self.model)?;
                Ok(Value::Bool(outputs >> (number - 1) & 1 != 0))
            }
            (Key::Enabled, Some(Role::Input(number))) => {
                let (_, inputs) = self.link().read_state(self.model)?;
                Ok(Value::Bool(inputs >> (number - 1) & 1 != 0))
            }
            (Key::Voltage, Some(Role::Analog(number))) => {
                Ok(Value::U64(u64::from(self.link().get_analog(number)?)))
            }
            (Key::Voltage, Some(Role::Supply)) => {
                Ok(Value::F64(f64::from(self.link().get_supply_millivolts()?) / 1000.0))
            }
            (other, _) => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let role = self.role(group)?;
        let on = || value.as_bool().ok_or_else(|| Error::arg("enabled takes on or off"));
        match (key, role) {
            (Key::Enabled, None) => self.link().set_all_outputs(self.model, on()?),
            (Key::Enabled, Some(Role::Output(number))) => self.link().set_output(number, on()?),
            (other, _) => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    /// `std_serial_dev_open`.
    fn open(&mut self) -> Result<()> {
        let port = match self.pending.take() {
            Some(port) => port,
            None => crate::serial::open(&self.conn, serialcomm()?)?,
        };
        self.link().attach(port);
        Ok(())
    }

    /// `std_serial_dev_close`.
    fn close(&mut self) -> Result<()> {
        self.link().port = None;
        Ok(())
    }

    /// `std_dummy_dev_acquisition_start`: there is nothing to acquire. A board
    /// has no channels, so libsigrok's session refuses to start ("has no enabled
    /// channels"); a session here ends at once.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        sink.send(Packet::header());
        sink.send(Packet::End);
        Ok(())
    }

    /// `std_dummy_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        Ok(())
    }
}
