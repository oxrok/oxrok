// Counterpart in sigrok:
//   libsigrok/src/device.c,
//   libsigrok/src/hwdriver.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The hardware contract: drivers find devices, devices acquire.

use crate::core::channel::{Channel, ChannelGroup};
use crate::core::error::{Error, Result};
use crate::core::firmware::FirmwareDependency;
use crate::core::key::Key;
use crate::core::option::{OptionDesc, ValueList};
use crate::core::packet::Packet;
use crate::core::trigger::{Trigger, TriggerFeatures};
use crate::core::value::Value;
use crate::usb::UsbId;
use std::cell::Cell;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{SyncSender, TrySendError};
use std::time::{Duration, Instant};

/// What kind of instrument a device is. A device can be several.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
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

/// How long a sink that was told of its device's stop flag waits for room
/// in the queue once the flag is set, for each packet: what a reader that
/// reads needs to take one, and what a close waits for one that does not.
pub const SEND_AFTER_STOP: Duration = Duration::from_millis(500);

/// How often a sender that waits for room in the queue looks at it, and at
/// whether it should go on waiting.
const LOOK: Duration = Duration::from_millis(2);

/// The first of those looks for a sink told of its stop flag, which waits
/// this way on every full queue and not only at the end of a capture: a
/// reader that is a little behind is not waited on for [`LOOK`].
const FIRST_LOOK: Duration = Duration::from_micros(50);

/// Where a device sends its packets.
///
/// Sending blocks when the consumer falls behind, which is the
/// backpressure that keeps memory bounded. `send` returns false once the
/// consumer is gone, and the acquisition thread should then stop.
///
/// A stop or a close joins that thread, and must not wait for ever on a
/// queue nobody reads any more: the driver tells the sink of its stop flag
/// with [`PacketSink::stopped_by`] before the thread gets it, and a consumer
/// that stopped reading is then one that is gone.
#[derive(Clone, Debug)]
pub struct PacketSink {
    tx: SyncSender<(usize, Packet)>,
    device: usize,
    stop: Option<Arc<Stop>>,
}

/// The stop flag a sink was told of, and whether a packet was given up
/// for it. The clones of a sink share it.
#[derive(Debug)]
struct Stop {
    asked: Arc<AtomicBool>,
    after: Duration,
    gave_up: AtomicBool,
}

impl PacketSink {
    /// A sink that tags packets with `device` on `tx`.
    pub fn new(tx: SyncSender<(usize, Packet)>, device: usize) -> Self {
        PacketSink { tx, device, stop: None }
    }

    /// This sink, told of the flag that asks its acquisition to stop: once
    /// `stop` is set, a packet waits for room in the queue for
    /// [`SEND_AFTER_STOP`] and no longer.
    ///
    /// A reader that goes on reading gets every packet it got before, in the
    /// same order, however long the capture takes to end. With one that does
    /// not, the first packet that finds no room within that time is given
    /// up, and so is all that follows it, at once and with no further wait:
    /// `send` and `end` return false as they do when nobody listens, so what
    /// was sent never has a hole in it, and the thread ends for whoever
    /// joins it. Before the flag is set a full queue is waited on for as
    /// long as it takes, as without this.
    #[must_use]
    pub fn stopped_by(self, stop: &Arc<AtomicBool>) -> Self {
        self.stopped_by_within(stop, SEND_AFTER_STOP)
    }

    /// [`PacketSink::stopped_by`] with a wait of `after` in place of
    /// [`SEND_AFTER_STOP`].
    #[must_use]
    pub fn stopped_by_within(mut self, stop: &Arc<AtomicBool>, after: Duration) -> Self {
        self.stop = Some(Arc::new(Stop {
            asked: Arc::clone(stop),
            after,
            gave_up: AtomicBool::new(false),
        }));
        self
    }

    /// Sends one packet. False when nobody listens any more, which for a
    /// sink told of its stop flag ([`PacketSink::stopped_by`]) includes a
    /// queue that stayed full after the stop.
    pub fn send(&self, packet: Packet) -> bool {
        let Some(stop) = &self.stop else {
            return self.tx.send((self.device, packet)).is_ok();
        };
        if stop.gave_up.load(Ordering::SeqCst) {
            return false;
        }
        // The wait is counted from when this packet first found the flag
        // set: a reader that is slow, and reads, loses nothing.
        let stopped_at = Cell::new(None);
        let sent = self.send_looking(packet, FIRST_LOOK, || {
            if !stop.asked.load(Ordering::SeqCst) {
                return true;
            }
            let at = stopped_at.get().unwrap_or_else(Instant::now);
            stopped_at.set(Some(at));
            at.elapsed() < stop.after
        });
        if !sent && stopped_at.get().is_some() {
            stop.gave_up.store(true, Ordering::SeqCst);
        }
        sent
    }

    /// Ends an acquisition: [`Packet::Failed`] when `outcome` is an error,
    /// then [`Packet::End`]. The way every acquisition finishes, so that each
    /// says whether it failed. `Err` is for a capture that ended for any
    /// reason other than a limit, a stop or the natural end of the data; an
    /// error the driver retried past or only warns about is `Ok`. False when
    /// nobody listens any more, as [`PacketSink::send`] says it.
    pub fn end(&self, outcome: Result<()>) -> bool {
        if let Err(e) = outcome
            && !self.send(Packet::Failed(e.to_string()))
        {
            return false;
        }
        self.send(Packet::End)
    }

    /// [`PacketSink::end`] through [`PacketSink::send_while`].
    pub fn end_while(&self, outcome: Result<()>, go_on: impl Fn() -> bool) -> bool {
        if let Err(e) = outcome
            && !self.send_while(Packet::Failed(e.to_string()), &go_on)
        {
            return false;
        }
        self.send_while(Packet::End, go_on)
    }

    /// Sends one packet, waiting for room only while `go_on` says so: a thread
    /// that a close will join must not block for ever on a queue no one is
    /// reading any more. False when nobody listens, or `go_on` said no. The
    /// stop flag of [`PacketSink::stopped_by`] is not looked at here: `go_on`
    /// is the whole of the condition.
    pub fn send_while(&self, packet: Packet, go_on: impl Fn() -> bool) -> bool {
        self.send_looking(packet, LOOK, go_on)
    }

    /// [`PacketSink::send_while`], looking at the queue again after `look`
    /// at first and after twice as long each time, up to [`LOOK`].
    fn send_looking(&self, packet: Packet, mut look: Duration, go_on: impl Fn() -> bool) -> bool {
        let mut item = (self.device, packet);
        loop {
            match self.tx.try_send(item) {
                Ok(()) => return true,
                Err(TrySendError::Disconnected(_)) => return false,
                Err(TrySendError::Full(back)) => {
                    if !go_on() {
                        return false;
                    }
                    item = back;
                    std::thread::sleep(look);
                    look = (look * 2).min(LOOK);
                }
            }
        }
    }
}

/// Starts `dev` on a queue with room for one packet that nobody reads,
/// leaves it `settle` to fill it, and stops and closes it: what the tests
/// of a driver ask of its stop and of its close. One that waits on the
/// queue fails here, and does not hold the suite.
///
/// # Panics
///
/// When the start, the stop or the close fails, or the two take more than
/// `within`.
#[cfg(test)]
pub(crate) fn close_with_an_unread_queue(
    mut dev: Box<dyn Device>,
    settle: Duration,
    within: Duration,
) -> Box<dyn Device> {
    let (tx, rx) = std::sync::mpsc::sync_channel(1);
    dev.start(PacketSink::new(tx, 0)).expect("the start");
    std::thread::sleep(settle);
    let (done, closed) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let outcome = dev.stop().and_then(|()| dev.close());
        let _ = done.send((dev, outcome));
    });
    let (dev, outcome) = closed
        .recv_timeout(within)
        .unwrap_or_else(|_| panic!("the close waited on a queue nobody reads for {within:?}"));
    outcome.expect("the stop and the close");
    drop(rx);
    dev
}

/// An open or openable device.
///
/// Configuration follows sigrok's model: a device has options, each a
/// [`Key`] with get, set and list access, optionally scoped to a channel
/// group. Acquisition is a thread the device owns: [`Device::start`]
/// spawns it, it sends packets to the sink, and [`Device::stop`] asks it
/// to end. The thread always finishes with [`PacketSink::end`], which
/// sends [`Packet::End`] and, before it, [`Packet::Failed`] if it failed.
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

    /// Closes the device at the end of the CLI's capture run, whether or
    /// not its start succeeded. It exists for wire parity with sigrok-cli,
    /// which never calls `sr_dev_close()` there: `sr_exit()` closes through
    /// `std_dev_clear()` with the device still active (`std.c:427-428`,
    /// where `sr_dev_close()` sets it inactive first, `device.c:779`). A
    /// driver whose close depends on that state (sysclk-lwla's shut-down
    /// bitstream) overrides this; a library client calls [`Device::close`]
    /// and gets what `sr_dev_close()` gives.
    fn close_after_capture(&mut self) -> Result<()> {
        self.close()
    }

    /// [`Device::channels`] is already the order a listing shows, by the numbers the C
    /// driver gives them, where [`Channel::index`] is another thing: `yokogawa-dlm`'s C
    /// numbers logic from 32, and here a logic channel's index is its bit. By default a
    /// listing sorts by index, as `sr_dev_inst_channels_get`'s users see them.
    fn channels_listed_as_they_are(&self) -> bool {
        false
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

    /// Whether the device would take `value` for `key`, asked of every `-c` before
    /// any of them goes out, and by [`crate::core::config_set`] before the set, so
    /// that a refusal leaves the device as it was. By default all goes; a driver
    /// whose setting is a line the device can refuse in silence checks here.
    fn check_set(&self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let _ = (key, value, group);
        Ok(())
    }

    /// Whether a capture with `trigger` can start at all, asked before the
    /// output is opened: a refusal here leaves an existing output file
    /// alone. [`Device::start`] may refuse later for other reasons.
    fn check_capture(&self, trigger: Option<&Trigger>) -> Result<()> {
        let _ = trigger;
        Ok(())
    }

    /// The output module a capture was asked to write with (`-O`), told once
    /// before it starts: for a warning when the driver's packets are known not
    /// to suit it. By default nothing happens.
    fn note_output(&self, module: &str) {
        let _ = module;
    }

    /// Starts acquiring. Packets go to `sink` from the device's own
    /// thread, starting with [`Packet::Header`] and ending with
    /// [`PacketSink::end`]. An `Err` here is a capture that never started;
    /// one that fails later reports it through the sink.
    ///
    /// A driver whose `stop` or `close` joins that thread tells the sink of
    /// the flag they set ([`PacketSink::stopped_by`]) before the thread
    /// takes it, or bounds its sends itself ([`PacketSink::send_while`]):
    /// the caller need not be reading the queue when it stops or closes.
    fn start(&mut self, sink: PacketSink) -> Result<()>;

    /// Asks a running acquisition to end. The thread sends
    /// [`Packet::End`] when it does. Safe to call when not running. It
    /// returns in a bounded time whether or not anybody reads the queue,
    /// and so does [`Device::close`].
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

/// A USB device a driver opens itself, with bulk, control or HID
/// transfers, so the user needs access to its node: what the udev rules
/// of `oxrok --udev-rules` cover ([`crate::core::udev`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UsbClaim {
    /// A device by its ids, as [`Driver::scan`] matches it.
    Id {
        /// Vendor and product id.
        id: UsbId,
        /// The vendor's name.
        vendor: &'static str,
        /// The model, or the models that share the id.
        model: &'static str,
    },
    /// Every device with an interface of this class, subclass and
    /// protocol (USBTMC: `fe`, `03`, `01`), which the scan finds by class.
    Class {
        /// `bInterfaceClass`.
        class: u8,
        /// `bInterfaceSubClass`.
        subclass: u8,
        /// `bInterfaceProtocol`.
        protocol: u8,
        /// What such devices are, for the comment in the rules.
        name: &'static str,
    },
}

impl UsbClaim {
    /// A device by its [`UsbId`].
    pub const fn id(id: UsbId, vendor: &'static str, model: &'static str) -> UsbClaim {
        UsbClaim::Id { id, vendor, model }
    }

    /// A device by vendor and product id.
    pub const fn ids(
        vendor_id: u16,
        product_id: u16,
        vendor: &'static str,
        model: &'static str,
    ) -> UsbClaim {
        UsbClaim::Id { id: UsbId { vendor: vendor_id, product: product_id }, vendor, model }
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

    /// The firmware and bitstream files the driver needs: what it
    /// uploads on open, or what the device must already run. None for
    /// the drivers that talk to the device's own firmware.
    fn firmware(&self) -> Vec<FirmwareDependency> {
        Vec::new()
    }

    /// The USB devices the driver opens itself, as [`Driver::scan`]
    /// matches them: what a udev rule must cover for the user to reach
    /// them ([`crate::core::udev`]). None for a serial port behind a
    /// USB cable, which the tty's group covers, and for everything that
    /// is no USB.
    fn usb_claims(&self) -> Vec<UsbClaim> {
        Vec::new()
    }

    /// Whether a capture ends by itself, with no limit to set: the driver
    /// offers none of the keys `--samples`, `--time`, `--frames` and
    /// `--continuous` need, so `-O`, `-o` or `-t` alone starts a capture
    /// (where sigrok-cli prints its help).
    fn ends_captures_itself(&self) -> bool {
        false
    }

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
