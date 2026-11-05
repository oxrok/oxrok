// Counterpart in sigrok:
//   libsigrok/src/hardware/icstation-usbrelay/api.c,
//   libsigrok/src/hardware/icstation-usbrelay/protocol.c,
//   Copyright (C) 2021-2023 Frank Stettner <frank-stettner@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The `ICStation` relay boards ICSE012A, ICSE013A and ICSE014A on a USB serial
//! port. Each relay is a channel group `R1` to `Rn` with an `enabled` option;
//! without a group, `enabled` switches them all. See [`protocol`].
//!
//! The board can be identified only once per power cycle: the open puts it in
//! command mode, and from then on the scan's request is a relay byte. So a second
//! run finds nothing, and its scan has switched relays. There is no way to read
//! the relays, so a get reads what this process last set, from all off.

#![forbid(unsafe_code)]

pub mod protocol;

#[cfg(test)]
mod tests;

use crate::core::{
    Access, Channel, ChannelGroup, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::Profile;

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multiplexer];
static DEV_OPTIONS: [OptionDesc; 2] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Enabled, Access { get: false, set: true, list: false }),
];
static GROUP_OPTIONS: [OptionDesc; 1] = [OptionDesc::new(Key::Enabled, Access::GET_SET)];

/// Opens the port again, for each open of the device.
type Opener = Box<dyn FnMut() -> Result<Box<dyn Port>> + Send>;

/// The driver.
pub struct IcstationUsbrelayDriver;

impl Driver for IcstationUsbrelayDriver {
    fn id(&self) -> &'static str {
        "icstation-usbrelay"
    }

    fn long_name(&self) -> &'static str {
        "ICStation USBRelay"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = protocol::SERIALCOMM.to_owned();
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => {
                    value
                        .as_str()
                        .ok_or_else(|| Error::arg("serialcomm"))?
                        .clone_into(&mut serialcomm);
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let first = Params::parse(&serialcomm)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (port, params) = match first {
            Ok(first) => first,
            Err(e) => {
                log::debug!("{e}");
                return Ok(Vec::new());
            }
        };
        let name = conn.clone();
        let opener = Box::new(move || crate::serial::open(&name, params));
        Ok(probe(port, conn, opener)
            .map_or_else(Vec::new, |d| vec![Box::new(d) as Box<dyn Device>]))
    }
}

/// The scan after the open. The port goes when this returns, as C closes it
/// on each path; the open that follows flushes what an answer left.
fn probe(mut port: Box<dyn Port>, conn: String, opener: Opener) -> Option<IcstationDevice> {
    let Ok(id) = protocol::identify(port.as_mut()) else {
        log::error!("Cannot retrieve identification details.");
        return None;
    };
    let Some(profile) = protocol::profile(id) else {
        log::warn!("Unknown device identification 0x{id:02x}.");
        return None;
    };
    Some(IcstationDevice::new(profile, conn, opener))
}

/// One board.
pub struct IcstationDevice {
    info: DeviceInfo,
    conn: String,
    groups: Vec<ChannelGroup>,
    mask: u8,
    /// The relays as last set, 1 for on. As in C, all off at the start.
    state: u8,
    opener: Opener,
    port: Option<Box<dyn Port>>,
    /// This object has put the board in command mode. A second `51` would be a
    /// relay byte (see the open).
    started: bool,
}

impl IcstationDevice {
    fn new(profile: Profile, conn: String, opener: Opener) -> IcstationDevice {
        IcstationDevice {
            info: DeviceInfo {
                driver: "icstation-usbrelay",
                vendor: "ICStation".to_owned(),
                model: profile.name.to_owned(),
                version: String::new(),
                serial: String::new(),
                // Off the scan line, as in C: only one board can be behind the
                // conn= its user had to name. A get of conn answers.
                conn: None,
            },
            conn,
            groups: (1..=profile.relays)
                .map(|n| ChannelGroup::new(format!("R{n}"), Vec::new()))
                .collect(),
            mask: protocol::relay_mask(profile.relays),
            state: 0,
            opener,
            port: None,
            started: false,
        }
    }

    /// The bit of a group, `None` for the device.
    fn relay(&self, group: Option<&str>) -> Result<Option<usize>> {
        let Some(g) = group else { return Ok(None) };
        self.groups
            .iter()
            .position(|cg| cg.name == g)
            .map(Some)
            .ok_or_else(|| Error::ChannelGroup(g.to_owned()))
    }

    /// `icstation_usbrelay_switch_cg`: the cache changes only once the byte went
    /// out.
    fn switch(&mut self, relay: Option<usize>, on: bool) -> Result<()> {
        let state = protocol::switched(self.state, self.mask, relay, on);
        let byte = protocol::wire_state(state, self.mask);
        log::trace!("Sending status byte: {byte:x}");
        let sent = match &mut self.port {
            Some(port) => protocol::send_byte(port.as_mut(), byte),
            None => Err(Error::device("the port is not open")),
        };
        if sent.is_err() {
            log::error!("Unable to send status byte.");
            return Err(io_error());
        }
        self.state = state;
        Ok(())
    }
}

/// What C's `SR_ERR_IO` reads as, whatever the cause.
fn io_error() -> Error {
    Error::Io(std::io::Error::other("input/output error"))
}

impl Device for IcstationDevice {
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
        match self.relay(group)? {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(_) => Ok(GROUP_OPTIONS.to_vec()),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        match (key, self.relay(group)?) {
            (Key::Conn, None) => Ok(Value::Str(self.conn.clone())),
            (Key::Enabled, Some(bit)) => Ok(Value::Bool(self.state >> bit & 1 != 0)),
            (other, _) => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let relay = self.relay(group)?;
        match key {
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(|| Error::arg("enabled takes on or off"))?;
                self.switch(relay, on)
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn open(&mut self) -> Result<()> {
        if self.port.is_some() {
            return Ok(());
        }
        let mut port = (self.opener)()?;
        // A second open of the same object sends no 51, where C's dev_open does
        // (api.c:225-226): the board would take it as a relay byte and switch
        // them behind the cache.
        if !self.started {
            if protocol::start(port.as_mut()).is_err() {
                log::error!("Cannot initiate command mode.");
                return Err(io_error());
            }
            self.started = true;
        }
        self.port = Some(port);
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
            "icstation-usbrelay device {} has no enabled channels.",
            self.conn
        )))
    }

    fn stop(&mut self) -> Result<()> {
        Ok(())
    }
}
