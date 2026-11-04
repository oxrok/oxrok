// Counterpart in sigrok:
//   libsigrok/src/device.c,
//   libsigrok/src/hwdriver.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The hardware contract: drivers find devices, devices acquire.

use crate::core::channel::{Channel, ChannelGroup};
use crate::core::error::{Error, Result};
use crate::core::key::Key;
use crate::core::option::{OptionDesc, ValueList};
use crate::core::packet::Packet;
use crate::core::trigger::{Trigger, TriggerFeatures};
use crate::core::value::Value;
use std::sync::mpsc::SyncSender;

/// What kind of instrument a device is. A device can be several.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DeviceClass {
    /// A logic analyzer.
    LogicAnalyzer,
    /// An oscilloscope.
    Oscilloscope,
    /// A multimeter.
    Multimeter,
    /// A demo device, no hardware.
    Demo,
    /// A sound level meter.
    SoundLevelMeter,
    /// A thermometer.
    Thermometer,
    /// A hygrometer.
    Hygrometer,
    /// An energy meter.
    EnergyMeter,
    /// A demodulator.
    Demodulator,
    /// A power supply.
    PowerSupply,
    /// An LCR meter.
    LcrMeter,
    /// An electronic load.
    ElectronicLoad,
    /// A scale.
    Scale,
    /// A signal generator.
    SignalGenerator,
    /// A power meter.
    PowerMeter,
    /// A multiplexer.
    Multiplexer,
    /// A delay generator.
    DelayGenerator,
    /// A frequency counter.
    FrequencyCounter,
}

impl DeviceClass {
    /// The display name, as `--show` prints it.
    pub fn name(self) -> &'static str {
        match self {
            DeviceClass::LogicAnalyzer => "Logic analyzer",
            DeviceClass::Oscilloscope => "Oscilloscope",
            DeviceClass::Multimeter => "Multimeter",
            DeviceClass::Demo => "Demo device",
            DeviceClass::SoundLevelMeter => "Sound level meter",
            DeviceClass::Thermometer => "Thermometer",
            DeviceClass::Hygrometer => "Hygrometer",
            DeviceClass::EnergyMeter => "Energy meter",
            DeviceClass::Demodulator => "Demodulator",
            DeviceClass::PowerSupply => "Power supply",
            DeviceClass::LcrMeter => "LCR meter",
            DeviceClass::ElectronicLoad => "Electronic load",
            DeviceClass::Scale => "Scale",
            DeviceClass::SignalGenerator => "Signal generator",
            DeviceClass::PowerMeter => "Power meter",
            DeviceClass::Multiplexer => "Multiplexer",
            DeviceClass::DelayGenerator => "Delay generator",
            DeviceClass::FrequencyCounter => "Frequency counter",
        }
    }
}

/// Identity of a device.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DeviceInfo {
    /// The driver id, e.g. `demo` or `intronix-logicport`.
    pub driver: &'static str,
    /// Vendor name, possibly empty.
    pub vendor: String,
    /// Model name.
    pub model: String,
    /// Hardware or firmware version, possibly empty.
    pub version: String,
    /// Serial number, possibly empty.
    pub serial: String,
    /// Connection id, e.g. a USB `bus.address`, when the device has one.
    pub conn: Option<String>,
}

/// Where a device sends its packets.
///
/// Sending blocks when the consumer falls behind, which is the
/// backpressure that keeps memory bounded. `send` returns false once the
/// consumer is gone, and the acquisition thread should then stop.
#[derive(Clone, Debug)]
pub struct PacketSink {
    tx: SyncSender<(usize, Packet)>,
    device: usize,
}

impl PacketSink {
    /// A sink that tags packets with `device` on `tx`.
    pub fn new(tx: SyncSender<(usize, Packet)>, device: usize) -> Self {
        PacketSink { tx, device }
    }

    /// Sends one packet. False when nobody listens any more.
    pub fn send(&self, packet: Packet) -> bool {
        self.tx.send((self.device, packet)).is_ok()
    }
}

/// An open or openable device.
///
/// Configuration follows sigrok's model: a device has options, each a
/// [`Key`] with get, set and list access, optionally scoped to a channel
/// group. Acquisition is a thread the device owns: [`Device::start`]
/// spawns it, it sends packets to the sink, and [`Device::stop`] asks it
/// to end. The thread always finishes with [`Packet::End`].
pub trait Device: Send {
    /// Identity.
    fn info(&self) -> &DeviceInfo;

    /// What kind of instrument this is.
    fn classes(&self) -> &[DeviceClass];

    /// All channels, in device order.
    fn channels(&self) -> &[Channel];

    /// Channel groups, possibly none.
    fn channel_groups(&self) -> &[ChannelGroup] {
        &[]
    }

    /// Enables or disables the channel at position `pos` of
    /// [`Device::channels`].
    fn enable_channel(&mut self, pos: usize, enabled: bool) -> Result<()>;

    /// Renames the channel at position `pos`.
    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()>;

    /// The options of the device (`group` is `None`) or of a channel
    /// group.
    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>>;

    /// The current value of `key`.
    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value>;

    /// Sets `key` to `value`.
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()>;

    /// The allowed values of `key`.
    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList>;

    /// Opens the device. Called once before anything else.
    fn open(&mut self) -> Result<()> {
        Ok(())
    }

    /// Closes the device.
    fn close(&mut self) -> Result<()> {
        Ok(())
    }

    /// What the device's trigger can do beyond level and edge stages.
    /// The default is nothing beyond them, with any number of stages.
    fn trigger_features(&self) -> TriggerFeatures {
        TriggerFeatures::default()
    }

    /// Sets the trigger for the next acquisition, or clears it.
    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        match trigger {
            None => Ok(()),
            Some(_) => Err(Error::not_supported("triggers")),
        }
    }

    /// Starts acquiring. Packets go to `sink` from the device's own
    /// thread, starting with [`Packet::Header`] and ending with
    /// [`Packet::End`].
    fn start(&mut self, sink: PacketSink) -> Result<()>;

    /// Asks a running acquisition to end. The thread sends
    /// [`Packet::End`] when it does. Safe to call when not running.
    fn stop(&mut self) -> Result<()>;

    /// The option descriptor of `key`, if the device has it.
    fn option(&self, key: &Key, group: Option<&str>) -> Option<OptionDesc> {
        self.options(group).ok()?.into_iter().find(|o| &o.key == key)
    }

    /// Positions of the channels of `group`, or of all channels.
    fn group_positions(&self, group: Option<&str>) -> Result<Vec<usize>> {
        match group {
            None => Ok((0..self.channels().len()).collect()),
            Some(name) => self
                .channel_groups()
                .iter()
                .find(|g| g.name.eq_ignore_ascii_case(name))
                .map(|g| g.channels.clone())
                .ok_or_else(|| Error::ChannelGroup(name.to_owned())),
        }
    }
}

/// A driver: knows how to find its devices.
pub trait Driver: Send + Sync {
    /// The id, as used with `-d`, e.g. `demo`.
    fn id(&self) -> &'static str;

    /// A one line description, e.g. `Demo driver and pattern generator`.
    fn long_name(&self) -> &'static str;

    /// The keys accepted by [`Driver::scan`].
    fn scan_options(&self) -> &'static [Key] {
        &[]
    }

    /// The keys `-d DRIVER:KEY=VALUE` takes, which `--show` does not list
    /// when they are not [`Driver::scan_options`]: for a port that accepts
    /// what the driver it follows does not advertise.
    fn accepted_scan_options(&self) -> &'static [Key] {
        self.scan_options()
    }

    /// What kind of instruments the driver serves.
    fn classes(&self) -> &'static [DeviceClass];

    /// Finds devices. `options` are scan options, e.g. a connection.
    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>>;
}

/// The drivers a program knows about.
#[derive(Default)]
pub struct Registry {
    drivers: Vec<Box<dyn Driver>>,
}

impl Registry {
    /// An empty registry.
    pub fn new() -> Self {
        Registry::default()
    }

    /// Adds a driver. Drivers are kept sorted by id.
    pub fn register(&mut self, driver: Box<dyn Driver>) {
        self.drivers.push(driver);
        self.drivers.sort_by_key(|d| d.id());
    }

    /// All drivers, sorted by id.
    pub fn drivers(&self) -> impl Iterator<Item = &dyn Driver> {
        self.drivers.iter().map(AsRef::as_ref)
    }

    /// The driver with `id`.
    pub fn find(&self, id: &str) -> Option<&dyn Driver> {
        self.drivers.iter().find(|d| d.id() == id).map(AsRef::as_ref)
    }
}
