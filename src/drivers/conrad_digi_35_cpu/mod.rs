// Counterpart in sigrok:
//   libsigrok/src/hardware/conrad-digi-35-cpu/api.c,
//   Copyright (C) 2014 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Frank Stettner, Martin Ling, Soeren Apel, Aurelien Jacobs,
//   Daniel Elstner, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Conrad DIGI 35 CPU, a laboratory power supply of 0 to 35 V and 0 to 2.55 A
//! on an RS-232 port that only receives. See [`protocol`].
//!
//! It can't be found or read. The scan opens the port named by `conn=`, closes it
//! and assumes the device is there, so any port that opens lists one. There is no
//! get, so nothing tells a setting that worked from one that went nowhere: a wrong
//! `serialcomm=`, a device switched off, or another device on that port. Only the
//! front panel shows. And as the wiki has it, the output can't be enabled from here.
//!
//! C's `start` and `stop` are dummies, and this has no capture: a session refuses a
//! device with no channels first.
//!
//! Differences from C:
//!
//! - A NaN passes C's two range checks and goes out, as `V-21` or `C-21` on x86-64.
//!   Here it is refused, as an out of range value is.
//! - Each command waits, with a bound, for the tty's queue to empty before the
//!   pause. C returns success on a frame still queued, and fails only once the queue
//!   is full, 16 writes on `cdc_acm`.
//! - After a command cut short, stuck in the queue or met by a port failure, the
//!   supply's state is unknown: it warns so, and sets are refused until a reopen. A
//!   reopen clears this refusal, not what the supply's buffer may hold. C carries on.
//! - Each way a command fails says which it was. C has `Write error` for all, and
//!   this says that only of a write that took part or none of a frame.
//! - A double is narrowed to single precision first, as sigrok-cli's `strtof` has
//!   done to the same values, so both CLIs send the same frame. C's driver keeps the
//!   double (only `session.c:570` narrows), so this costs a library client:
//!   - values at or just under a tie move: 1.15 V goes out as `V011` where libsigrok
//!     sends `V012`, by one step, and 140 of the 35001 voltages on a 1 mV grid and
//!     101 of the 25501 currents on a 0.1 mA grid move down, and none the lists name;
//!   - values off that grid can move up: 1.2499999999 V is `V013` in both CLIs, and
//!     `V012` through libsigrok's double;
//!   - the range is wider: up to about 35.0000019 V and 2.55000007 A are accepted,
//!     and -1e-50 is accepted as `V000`, where C refuses them.

#![forbid(unsafe_code)]

pub mod protocol;

#[cfg(test)]
mod tests;

use crate::core::{
    Access, Channel, ChannelGroup, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{CURRENT_STEP, Command, MAX_CURRENT, MAX_VOLTAGE, VOLTAGE_STEP};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::PowerSupply];
const SET_LIST: Access = Access { get: false, set: true, list: true };
const SET: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::VoltageTarget, SET_LIST),
    OptionDesc::new(Key::CurrentLimit, SET_LIST),
    OptionDesc::new(Key::OverCurrentProtectionEnabled, SET),
];

/// Opens the port again, for each open of the device.
type Opener = Box<dyn FnMut() -> Result<Box<dyn Port>> + Send>;
/// Waits for a time.
type Pause = Box<dyn FnMut(std::time::Duration) + Send>;

/// The driver.
pub struct ConradDigi35CpuDriver;

impl Driver for ConradDigi35CpuDriver {
    fn id(&self) -> &'static str {
        "conrad-digi-35-cpu"
    }

    fn long_name(&self) -> &'static str {
        "Conrad DIGI 35 CPU"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        Ok(scan_with(options, crate::serial::open)?
            .into_iter()
            .map(|d| Box::new(d) as Box<dyn Device>)
            .collect())
    }
}

/// The scan, on `open`. It can only try the port and assume the device is there.
fn scan_with<F>(options: &[(Key, Value)], open: F) -> Result<Option<ConradDevice>>
where
    F: Fn(&str, Params) -> Result<Box<dyn Port>> + Send + 'static,
{
    let mut conn = None;
    let mut serialcomm = protocol::SERIALCOMM.to_owned();
    for (key, value) in options {
        match key {
            Key::Conn => conn = value.as_str().map(str::to_owned),
            Key::Serialcomm => {
                value.as_str().ok_or_else(|| Error::arg("serialcomm"))?.clone_into(&mut serialcomm);
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
    }
    let Some(conn) = conn else {
        return Ok(None);
    };
    let first = Params::parse(&serialcomm).and_then(|params| Ok((open(&conn, params)?, params)));
    let params = match first {
        // The port closes here: only the open could be tried.
        Ok((_port, params)) => params,
        Err(e) => {
            log::error!("{e}");
            return Ok(None);
        }
    };
    log::trace!("Conrad DIGI 35 CPU assumed at {conn}.");
    let name = conn.clone();
    let opener = Box::new(move || open(&name, params));
    Ok(Some(ConradDevice::new(conn, params, opener)))
}

/// One supply, assumed to be there.
pub struct ConradDevice {
    info: DeviceInfo,
    conn: String,
    params: Params,
    opener: Opener,
    port: Option<Box<dyn Port>>,
    /// What a command waits for after it left: a sleep, but for the tests.
    pause: Pause,
    /// A command failed so that the supply's state isn't known. Until a reopen,
    /// nothing more goes out.
    state_unknown: bool,
}

impl ConradDevice {
    fn new(conn: String, params: Params, opener: Opener) -> ConradDevice {
        ConradDevice {
            info: DeviceInfo {
                driver: "conrad-digi-35-cpu",
                vendor: "Conrad".to_owned(),
                model: "DIGI 35 CPU".to_owned(),
                version: String::new(),
                serial: String::new(),
                // C sets no connection_id and has no SR_CONF_CONN, so the scan line
                // has none.
                conn: None,
            },
            conn,
            params,
            opener,
            port: None,
            pause: Box::new(std::thread::sleep),
            state_unknown: false,
        }
    }

    fn send(&mut self, command: Command) -> Result<()> {
        let Some(port) = &mut self.port else {
            return Err(Error::device(format!(
                "the port is not open: {} did not go out",
                command.shown()
            )));
        };
        if self.state_unknown {
            return Err(Error::device(format!(
                "an earlier command failed and the supply's state is unknown: {} did not go \
                 out, reopen the device to send more",
                command.shown()
            )));
        }
        protocol::send(port.as_mut(), &self.params, command, &mut self.pause).map_err(|failure| {
            self.state_unknown = failure.leaves_state_unknown();
            failure.report(command)
        })
    }
}

/// A value from 0 to `max`, as sigrok-cli's `strtof` has it: see the module's note
/// on what that costs. The narrowing is in the wrong layer, as only sigrok-cli does
/// it, and goes when the shared fix for it lands. `refusal` is C's line. A NaN is
/// refused too, where C's `v < 0 || v > max` lets it through.
fn in_range(key: &Key, value: &Value, max: f64, refusal: &str) -> Result<f64> {
    #[allow(clippy::cast_possible_truncation)]
    let single = value.as_f64().map(|v| f64::from(v as f32));
    match single {
        Some(v) if (0.0..=max).contains(&v) => Ok(v),
        _ => {
            log::error!("{refusal}");
            Err(Error::Arg(format!("{} out of range", key.id())))
        }
    }
}

impl Device for ConradDevice {
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
        &[]
    }

    fn enable_channel(&mut self, pos: usize, _enabled: bool) -> Result<()> {
        Err(Error::Arg(format!("No channel {pos}.")))
    }

    fn rename_channel(&mut self, pos: usize, _name: &str) -> Result<()> {
        Err(Error::Arg(format!("No channel {pos}.")))
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        no_group(group)?;
        Ok(DEV_OPTIONS.to_vec())
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        no_group(group)?;
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let command = match key {
            Key::VoltageTarget => {
                let refusal = "Voltage out of range (0 - 35.0)!";
                Command::voltage(in_range(key, value, MAX_VOLTAGE, refusal)?)
            }
            Key::CurrentLimit => {
                let refusal = "Current out of range (0 - 2.55)!";
                Command::current(in_range(key, value, MAX_CURRENT, refusal)?)
            }
            Key::OverCurrentProtectionEnabled => Command::Protection(
                value
                    .as_bool()
                    .ok_or_else(|| Error::arg("over_current_protection_enabled takes on or off"))?,
            ),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        };
        self.send(command)
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        let (max, step) = match key {
            Key::VoltageTarget => (MAX_VOLTAGE, VOLTAGE_STEP),
            Key::CurrentLimit => (MAX_CURRENT, CURRENT_STEP),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        };
        Ok(ValueList::RangeStepF64 { min: 0.0, max, step })
    }

    /// `std_serial_dev_open`: the port, with nothing on the wire.
    fn open(&mut self) -> Result<()> {
        if self.port.is_none() {
            self.port = Some((self.opener)()?);
            self.state_unknown = false;
        }
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.port = None;
        Ok(())
    }

    fn start(&mut self, _sink: PacketSink) -> Result<()> {
        // session.c:790-793: a session refuses a device with no channels, before
        // the driver's dummy start.
        Err(Error::device(format!(
            "conrad-digi-35-cpu device {} has no enabled channels.",
            self.conn
        )))
    }

    fn stop(&mut self) -> Result<()> {
        Ok(())
    }
}

fn no_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        None => Ok(()),
    }
}
