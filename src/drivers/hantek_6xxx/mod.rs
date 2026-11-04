// Counterpart in sigrok:
//   libsigrok/src/hardware/hantek-6xxx/api.c,
//   Copyright (C) 2015 Christer Ekholm <christerekholm@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Benjamin Larsson,
//   Gerhard Sittig, Chriter, Aurelien Jacobs, Erik Montnemery,
//   Markus Siegert, Mike Meyer, Romain Tartière, Sebastian Tabares Amaya,
//   Sebastian Zagrodzki, Sergey Alirzaev, Thomas Emmert, litospain, majekw,
//   waka
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! hantek-6xxx: the Hantek 6022BE and 6022BL, Voltcraft DSO-2020,
//! Sainsmart DDS120, `YiXingDianZi` MDSO and Instrustar ISDS205B, two
//! channel USB scopes on an FX2LP with sigrok's own firmware.
//!
//! The scan uploads the firmware to a board that has none and the board
//! renumerates; opening waits for it, finding it by its hub ports. Each
//! capture sends the rate, both ranges and the coupling, starts the
//! scope, throws away one transfer and takes one transfer at a time (see
//! [`protocol`]). Scan, open, settings and transfer sizes follow `api.c`
//! and `protocol.c` step by step, so that the device sees the same
//! transactions as under sigrok-cli (checked with the emulator,
//! `tools/emu/hantek-6xxx.sh`).
//!
//! Where it does not follow the C driver, each with its case:
//! - a disabled CH1 does not make CH2's samples CH1's;
//! - a capture is cut at the sample limit, in the output only;
//! - a rate, range or coupling the scope refuses at the start fails it,
//!   where the C driver goes on and labels what comes with the setting
//!   it asked for;
//! - a transfer that comes back empty or fails is taken again, a bounded
//!   number of times, and a silent scope or a vanished one ends the
//!   capture; the C driver neither resubmits nor stops;
//! - no wrap in the size of a transfer, whose rounding never ends in C;
//! - a rate off the list runs at the 100 kHz the C driver sends for it,
//!   and reads back as such, and transfers are sized for 100 kHz where
//!   the C driver sizes them for the rate asked for;
//! - `coupling` reads back what was set, and `num_vdiv` can be read.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, split_conn};
use protocol::{
    CONFIGURATION, Coupling, DEFAULT_SAMPLERATE, DEFAULT_VOLTAGE, FALLBACK_SAMPLERATE, FIRMWARE_ID,
    FLUSH_SIZE, Found, INTERFACE, Limits, Link, Profile, SAMPLERATES, Scale,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// `MAX_RENUM_DELAY_MS`.
const RENUM_TIMEOUT: Duration = Duration::from_secs(3);
const RENUM_RETRY: Duration = Duration::from_millis(100);
/// Transfers without data in a row after which the capture ends.
/// fx2lafw's `MAX_EMPTY_TRANSFERS`; a transfer that times out with
/// nothing ends it at once, as 4 s of silence is no stream.
const MAX_EMPTY_TRANSFERS: usize = 64;

/// The driver.
pub struct Hantek6xxxDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::Oscilloscope];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::NumVdiv, Access::GET),
];
static GROUP_OPTIONS: [OptionDesc; 2] = [
    OptionDesc::new(Key::Coupling, Access::GET_SET_LIST),
    OptionDesc::new(Key::Vdiv, Access::GET_SET_LIST),
];

impl Driver for Hantek6xxxDriver {
    fn id(&self) -> &'static str {
        "hantek-6xxx"
    }

    fn long_name(&self) -> &'static str {
        "Hantek 6xxx"
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
        let conn = match conn.as_deref().map(split_conn).transpose() {
            Ok(c) => c,
            Err(msg) => {
                // sr_usb_find() finds nothing, and the scan no device.
                log::error!("{msg}");
                return Ok(Vec::new());
            }
        };
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::find_devices(&protocol::scan_ids())? {
            if conn.is_some_and(|c| !c.matches(&info)) {
                continue;
            }
            match protocol::find_profile(info.id(), info.device_version) {
                Some(Found::Running(p)) => {
                    log::debug!("Found a {} {}.", p.vendor, p.model);
                    devices.push(Box::new(Hantek6xxxDevice::new(p, info, None)));
                }
                Some(Found::Bare(p)) => {
                    log::debug!("Found a {} {}.", p.vendor, p.model);
                    let mut dev = Hantek6xxxDevice::new(p, info, None);
                    match upload(&dev.usb, p) {
                        Ok(()) => dev.fw_updated = Some(Instant::now()),
                        Err(e) => {
                            log::debug!("Firmware upload: {e}.");
                            log::error!("Firmware upload failed, name {}.", p.firmware);
                        }
                    }
                    // The address after the renumeration is not known
                    // yet (0xff in the C driver).
                    dev.info.conn = None;
                    devices.push(Box::new(dev));
                }
                None => {}
            }
        }
        Ok(devices)
    }
}

/// `ezusb_upload_firmware()`: open, `SET_CONFIGURATION` 1, the image in
/// chunks of 4096 bytes with the 8051 held in reset, release.
fn upload(info: &UsbDeviceInfo, p: &Profile) -> Result<()> {
    let image = crate::core::firmware::read(
        p.firmware,
        "install sigrok-firmware-fx2lafw, see docs/DEVICES.md",
    )?;
    log::info!("uploading firmware to device on {}", info.conn());
    let mut t = crate::usb::open(info)?;
    t.force_configuration(CONFIGURATION)?;
    crate::ezusb::upload_firmware_logged(t.as_mut(), p.firmware, &image)
}

/// Settings kept between acquisitions.
#[derive(Clone, Copy, Debug)]
struct Settings {
    samplerate: u64,
    limits: Limits,
    /// An index into the board's V/div table, per channel.
    voltage: [usize; 2],
    coupling: [Coupling; 2],
}

/// One scope.
pub struct Hantek6xxxDevice {
    profile: &'static Profile,
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    settings: Settings,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
    /// When the scan uploaded the firmware: the device renumerates and
    /// is looked for again on open.
    fw_updated: Option<Instant>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl Hantek6xxxDevice {
    fn new(
        profile: &'static Profile,
        usb: UsbDeviceInfo,
        transport: Option<Box<dyn Transport>>,
    ) -> Self {
        Hantek6xxxDevice {
            profile,
            info: DeviceInfo {
                driver: "hantek-6xxx",
                vendor: profile.vendor.into(),
                model: profile.model.into(),
                version: String::new(),
                serial: String::new(),
                conn: Some(usb.conn()),
            },
            usb,
            channels: vec![
                Channel::new(0, ChannelKind::Analog, "CH1"),
                Channel::new(1, ChannelKind::Analog, "CH2"),
            ],
            groups: vec![ChannelGroup::new("CH1", vec![0]), ChannelGroup::new("CH2", vec![1])],
            settings: Settings {
                samplerate: DEFAULT_SAMPLERATE,
                limits: Limits { limit_msec: 0, limit_samples: 0 },
                voltage: [DEFAULT_VOLTAGE; 2],
                coupling: [Coupling::Dc; 2],
            },
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
            fw_updated: None,
        }
    }

    /// A device on an already open transport (a replayed recording), its
    /// firmware running.
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        match protocol::find_profile(usb.id(), usb.device_version) {
            Some(Found::Running(p)) => Ok(Self::new(p, usb, Some(transport))),
            _ => Err(Error::arg("not a hantek-6xxx scope with its firmware running")),
        }
    }

    /// The channel of a group name.
    fn group_index(&self, group: &str) -> Result<usize> {
        self.groups
            .iter()
            .position(|g| g.name == group)
            .ok_or_else(|| Error::ChannelGroup(group.to_owned()))
    }

    /// Runs a command on the open device. Closed, there is nothing to
    /// send it to, and the next capture sends the setting anyway.
    fn with_link(&self, f: impl FnOnce(&mut Link)) {
        if let Some(link) = self.link.lock().expect("link lock").as_mut() {
            f(link);
        }
    }

    /// Whether a capture holds the link. The firmware stops sampling at
    /// every command, so a setting in mid capture would end it: the C
    /// driver does so, on the one thread it has.
    fn busy(&self) -> bool {
        self.worker.as_ref().is_some_and(|w| !w.handle.is_finished())
    }
}

/// Whether `d` is the device scanned as `usb`: by its hub ports, which
/// stay the same when it renumerates (`usb_get_port_path()`). A backend
/// that knows no ports falls back to the address, or after an upload to
/// any firmware running on the same bus.
fn same_device(d: &UsbDeviceInfo, usb: &UsbDeviceInfo, after_upload: bool) -> bool {
    if d.bus != usb.bus {
        return false;
    }
    if !usb.ports.is_empty() && !d.ports.is_empty() {
        return d.ports == usb.ports;
    }
    after_upload || d.address == usb.address
}

/// `hantek_6xxx_open()`: the device with the firmware's ids at the
/// scanned port. Like the C code it looks at no `bcdDevice`: a firmware
/// file that reports another board's still opens.
///
/// `level` is that of a failed open: a retry after an upload races the
/// device coming up (`LIBUSB_ERROR_ACCESS` until udev has set the node's
/// permissions), which is no error unless it was the last attempt.
fn try_open(
    usb: &UsbDeviceInfo,
    after_upload: bool,
    level: log::Level,
) -> Result<(UsbDeviceInfo, Box<dyn Transport>)> {
    let found = crate::usb::find_devices(&[FIRMWARE_ID])?
        .into_iter()
        .find(|d| same_device(d, usb, after_upload));
    let info = found.ok_or_else(|| Error::device("device not found"))?;
    let t = crate::usb::open(&info).inspect_err(|e| {
        log::log!(level, "Failed to open device: {}.", crate::usb::libusb_error_name(e));
    })?;
    log::info!("Opened device on {} interface {INTERFACE}.", info.conn());
    Ok((info, t))
}

/// The claim of `dev_open()`. `set_configuration(1)` comes first; it
/// sends nothing when the configuration is active (always on Linux) but
/// macOS can leave a device unconfigured right after its renumeration.
fn claim(t: &mut Box<dyn Transport>) -> Result<()> {
    t.set_configuration(CONFIGURATION)?;
    t.claim_interface(INTERFACE).inspect_err(|e| {
        log::error!("Unable to claim interface: {}.", crate::usb::libusb_error_name(e));
    })
}

impl Device for Hantek6xxxDevice {
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
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        c.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        name.clone_into(&mut c.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(g) => {
                self.group_index(g)?;
                Ok(GROUP_OPTIONS.to_vec())
            }
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            let ch = self.group_index(g)?;
            return Ok(match key {
                Key::Vdiv => {
                    let (p, q) = self.profile.vdivs[self.settings.voltage[ch]];
                    Value::Rational(p, q)
                }
                // What was set: the C driver stores the index into a
                // DC-only board's list as the coupling, 0 being AC
                // (api.c:468-471, 422-423).
                Key::Coupling => Value::Str(self.settings.coupling[ch].name().to_owned()),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            });
        }
        Ok(match key {
            // The C driver sets this, and the switch after it answers
            // SR_ERR_NA (api.c:380-383, 406-407).
            Key::NumVdiv => Value::I32(self.profile.vdivs.len() as i32),
            Key::Samplerate => Value::U64(self.settings.samplerate),
            Key::LimitMsec => Value::U64(self.settings.limits.limit_msec),
            Key::LimitSamples => Value::U64(self.settings.limits.limit_samples),
            // Unknown until the device came back after the upload.
            Key::Conn => match &self.info.conn {
                Some(c) => Value::Str(c.clone()),
                None => return Err(Error::device("the device has not renumerated yet")),
            },
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        if self.busy() {
            return Err(Error::Busy);
        }
        if let Some(g) = group {
            let ch = self.group_index(g)?;
            match key {
                Key::Vdiv => {
                    let idx = protocol::vdiv_index(self.profile.vdivs, value).ok_or_else(bad)?;
                    self.settings.voltage[ch] = idx;
                    let voltage = self.settings.voltage;
                    self.with_link(|l| {
                        // Logged; the next capture sends it again, and
                        // fails if the scope still refuses it.
                        let _ = l.update_vdiv(voltage);
                    });
                }
                Key::Coupling => {
                    let s = value.as_str().ok_or_else(bad)?;
                    if !self.profile.couplings.contains(&s) {
                        return Err(bad());
                    }
                    self.settings.coupling[ch] =
                        if s == "AC" { Coupling::Ac } else { Coupling::Dc };
                    let (has, coupling) = (self.profile.has_coupling, self.settings.coupling);
                    self.with_link(|l| {
                        // Logged; the next capture sends it again, and
                        // fails if the scope still refuses it.
                        let _ = l.update_coupling(has, coupling);
                    });
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
            return Ok(());
        }
        match key {
            Key::Samplerate => {
                let rate = value.as_u64().ok_or_else(bad)?;
                // samplerate_to_reg() sends the last register for a rate
                // off the list, and the C driver keeps saying the rate
                // that was asked for over samples taken at 100 kHz. The
                // same goes out here, and 100 kHz is what is reported,
                // and what transfers are sized for.
                self.settings.samplerate = if protocol::samplerate_reg(rate).is_some() {
                    rate
                } else {
                    log::error!("Failed to convert samplerate: {rate}. It runs at 100 kHz.");
                    FALLBACK_SAMPLERATE
                };
                let rate = self.settings.samplerate;
                self.with_link(|l| {
                    // Logged; the next capture sends it again, and
                    // fails if the scope still refuses it.
                    let _ = l.update_samplerate(rate);
                });
            }
            Key::LimitMsec => self.settings.limits.limit_msec = value.as_u64().ok_or_else(bad)?,
            Key::LimitSamples => {
                self.settings.limits.limit_samples = value.as_u64().ok_or_else(bad)?;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            self.group_index(g)?;
            return Ok(match key {
                Key::Coupling => {
                    ValueList::Str(self.profile.couplings.iter().map(|c| (*c).to_owned()).collect())
                }
                Key::Vdiv => ValueList::Rational(self.profile.vdivs.to_vec()),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            });
        }
        Ok(match key {
            Key::Samplerate => ValueList::U64(SAMPLERATES.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `dev_open()`.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let unable = |e: Error| {
            log::error!("Unable to open device.");
            e
        };
        let (info, mut t) = if let Some(t) = self.pending.take() {
            (t.info().clone(), t)
        } else if let Some(uploaded) = self.fw_updated {
            // Wait for the FX2 to renumerate: it takes 300 ms or more to
            // be gone from the bus, then every 100 ms until 3 s after
            // the upload.
            log::info!("Waiting for device to reset.");
            std::thread::sleep(Duration::from_millis(300));
            let found = loop {
                // The loop's check comes 100 ms after the attempt.
                let last = uploaded.elapsed() + RENUM_RETRY >= RENUM_TIMEOUT;
                let level = if last { log::Level::Error } else { log::Level::Debug };
                match try_open(&self.usb, true, level) {
                    Ok(found) => break found,
                    Err(e) => {
                        std::thread::sleep(RENUM_RETRY);
                        if uploaded.elapsed() >= RENUM_TIMEOUT {
                            log::debug!("Last attempt: {e}.");
                            return Err(unable(e));
                        }
                    }
                }
            };
            log::info!("Device came back after {} ms.", uploaded.elapsed().as_millis());
            self.fw_updated = None;
            if found.0.device_version != self.profile.fw_version {
                log::warn!(
                    "The firmware reports bcdDevice {:04x}, where {} {} has {:04x}.",
                    found.0.device_version,
                    self.profile.vendor,
                    self.profile.model,
                    self.profile.fw_version
                );
            }
            found
        } else {
            try_open(&self.usb, false, log::Level::Error).map_err(unable)?
        };
        claim(&mut t).map_err(unable)?;
        self.info.conn = Some(info.conn());
        self.usb = info;
        *self.link.lock().expect("link lock") = Some(Link::new(t));
        Ok(())
    }

    /// `dev_close()`. Joins a capture that was told to stop.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(w) = self.worker.take() {
            let _ = w.handle.join();
        }
        if self.link.lock().expect("link lock").take().is_some() {
            log::info!("Closing device on {} interface {INTERFACE}.", self.usb.conn());
        }
        Ok(())
    }

    /// `dev_acquisition_start()`.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.busy() {
            return Err(Error::Busy);
        }
        if let Some(w) = self.worker.take() {
            let _ = w.handle.join();
        }
        let link = self.link.lock().expect("link lock").take();
        let Some(mut link) = link else {
            return Err(Error::device("device is not open"));
        };
        // hantek_6xxx_init(), each one sent whatever became of the others.
        // protocol.c:242-247 ignores them all, and the samples then say a
        // rate or a range the scope is not on. A refusal on the wire is an
        // observation: the start fails, with nothing started to stop.
        log::debug!("Initializing");
        let s = self.settings;
        let rate = link.update_samplerate(s.samplerate);
        let vdiv = link.update_vdiv(s.voltage);
        let coupling = link.update_coupling(self.profile.has_coupling, s.coupling);
        if let Err(e) = rate.and(vdiv).and(coupling) {
            *self.link.lock().expect("link lock") = Some(link);
            return Err(Error::Device(format!("the scope refused a setting: {e}")));
        }
        let scales = [
            Scale::new(self.profile.vdivs[s.voltage[0]]),
            Scale::new(self.profile.vdivs[s.voltage[1]]),
        ];
        let acq = Acquisition {
            rate: s.samplerate,
            limits: s.limits,
            scales,
            enabled: [self.channels[0].enabled, self.channels[1].enabled],
        };
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("hantek-6xxx acquisition".into())
            .spawn(move || {
                acq.run(&mut link, &sink, &stop2);
                *slot.lock().expect("link lock") = Some(link);
                sink.send(Packet::End);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// `dev_acquisition_stop()`: only the flag, seen after the transfer
    /// in flight, which ends within its 4 s. No join here: the thread may
    /// be handing packets to the session calling this.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// One acquisition, on its own thread: `receive_transfer()`.
struct Acquisition {
    rate: u64,
    limits: Limits,
    scales: [Scale; 2],
    enabled: [bool; 2],
}

impl Acquisition {
    /// The start, the flush, the transfers until a limit, the stop flag,
    /// a silent or vanished device; then the stop command.
    fn run(&self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) {
        sink.send(Packet::header());
        // Failed, the transfers below find out: the C driver logs it and
        // reads all the same.
        let _ = link.start();
        // What the first transfer holds or how it ends is of no interest,
        // as long as there is a device.
        let flushed = link.read(FLUSH_SIZE);
        if !matches!(flushed, Err(Error::Disconnected)) {
            self.transfers(link, sink, stop);
        }
        log::debug!("Stopping acquisition.");
        let _ = link.stop();
    }

    fn transfers(&self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) {
        let started = Instant::now();
        let elapsed = || started.elapsed().as_millis() as u64;
        let mut received = 0u64;
        let mut failures = 0;
        while !stop.load(Ordering::SeqCst) {
            let amount = protocol::data_amount(self.limits, self.rate, elapsed(), received);
            let data = match link.read(amount) {
                Ok(d) if d.is_empty() => {
                    // api.c:647-649 returns here without the next
                    // transfer, and the capture waits for ever.
                    log::debug!("Empty transfer.");
                    Vec::new()
                }
                Ok(d) => d,
                Err(Error::Disconnected) => {
                    log::error!("The device is gone, the data is incomplete.");
                    return;
                }
                Err(Error::Timeout) => {
                    log::error!(
                        "No data for {:?}, the data is incomplete.",
                        protocol::TRANSFER_TIMEOUT
                    );
                    return;
                }
                Err(e) => {
                    log::debug!("Transfer failed: {e}.");
                    Vec::new()
                }
            };
            // One byte is no sample either.
            if data.len() < 2 {
                failures += 1;
                if failures >= MAX_EMPTY_TRANSFERS {
                    log::error!("{failures} transfers without data, the data is incomplete.");
                    return;
                }
                continue;
            }
            failures = 0;
            let samples = (data.len() / 2) as u64;
            // api.c:651-652 sends it all. The count stays uncut, so the
            // sizes follow the C driver's.
            let keep = self.limits.room(received).min(samples) as usize;
            received += samples;
            if !self.send_chunk(&data[..keep * 2], sink) {
                return;
            }
            if self.limits.reached(received, elapsed()) {
                log::info!("Requested limit reached, stopping after {received} samples.");
                return;
            }
        }
    }

    /// `send_chunk()`: a packet per enabled channel. False when nobody
    /// listens any more.
    fn send_chunk(&self, data: &[u8], sink: &PacketSink) -> bool {
        (0..2).filter(|ch| self.enabled[*ch]).all(|ch| {
            let scale = self.scales[ch];
            // The channel itself. The C driver takes the next of a list
            // that holds the disabled channels too, so without CH1 the
            // samples of CH2 are said to be CH1's (api.c:187-192, 588,
            // 608).
            sink.send(Packet::Analog(AnalogData {
                channels: vec![ch],
                samples: protocol::channel_samples(data, ch, scale),
                mq: Mq::Voltage,
                flags: MqFlags::empty(),
                unit: Unit::Volt,
                digits: scale.digits,
            }))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;
    use protocol::PROFILES;
    use std::sync::mpsc::{Receiver, sync_channel};

    const STOP: &str = "ctl_out 40 e3 0000 0000 00 -> ok\n";
    const START: &str = "ctl_out 40 e3 0000 0000 01 -> ok\n";

    fn acquisition(limit_samples: u64) -> Acquisition {
        Acquisition {
            rate: 100_000,
            limits: Limits { limit_msec: 0, limit_samples },
            scales: [Scale::new((1, 1)), Scale::new((100, 1000))],
            enabled: [true, true],
        }
    }

    fn link(steps: &str) -> Link {
        let text = format!("# oxrok usb fixture 1\ndevice vid=1d50 pid=608e\n{START}{steps}");
        Link::new(Box::new(Replay::from_text(&text).unwrap()))
    }

    fn run(acq: &Acquisition, steps: &str) -> Receiver<(usize, Packet)> {
        let (tx, rx) = sync_channel(4096);
        let sink = PacketSink::new(tx, 0);
        acq.run(&mut link(steps), &sink, &AtomicBool::new(false));
        rx
    }

    fn analog(rx: &Receiver<(usize, Packet)>) -> Vec<AnalogData> {
        rx.try_iter()
            .filter_map(|(_, p)| if let Packet::Analog(a) = p { Some(a) } else { None })
            .collect()
    }

    /// A transfer of `n` zero bytes in a fixture.
    fn data(n: usize) -> String {
        format!("00{{{n}}}")
    }

    #[test]
    fn the_flush_is_thrown_away_and_the_whole_transfer_passed_on() {
        let steps = format!(
            "bulk_in 86 1024 -> {}\nbulk_in 86 512 -> {}\n{STOP}",
            "ff".repeat(1024),
            data(512)
        );
        let packets = analog(&run(&acquisition(8), &steps));
        assert_eq!(packets.len(), 2);
        assert_eq!(packets[0].channels, [0]);
        assert_eq!(packets[1].channels, [1]);
        assert_eq!(packets[0].samples.len(), 8, "cut, where api.c:651-652 sends 256");
        assert_eq!(packets[1].samples.len(), 8, "both channels alike");
        assert_eq!(packets[0].samples[0], -5.0);
        assert_eq!(packets[0].digits, 2);
        assert_eq!(packets[1].digits, 3);
    }

    /// The second transfer is cut to what the limit leaves, and sized from
    /// the samples received, cut or not.
    #[test]
    fn the_cut_is_on_the_last_transfer_only() {
        let steps = format!(
            "bulk_in 86 1024 -> -\nbulk_in 86 1024 -> {}\nbulk_in 86 512 -> {}\n{STOP}",
            data(400),
            data(512)
        );
        let packets = analog(&run(&acquisition(300), &steps));
        let lens: Vec<usize> = packets.iter().map(|p| p.samples.len()).collect();
        assert_eq!(lens, [200, 200, 100, 100]);
    }

    /// A STALL at the start: every setting is sent, the start is not, and
    /// the device can be used again.
    #[test]
    fn a_refused_setting_fails_the_start() {
        let text = "# oxrok usb fixture 1\n\
                    device vid=1d50 pid=608e version=0001\n\
                    config 1 -> ok\n\
                    claim 0 -> ok\n\
                    ctl_out 40 e2 0000 0000 08 -> err device \"endpoint stalled\"\n\
                    ctl_out 40 e0 0000 0000 05 -> ok\n\
                    ctl_out 40 e1 0000 0000 05 -> ok\n";
        let t = Replay::from_text(text).unwrap();
        let mut d = Hantek6xxxDevice::from_transport(Box::new(t)).unwrap();
        d.open().unwrap();
        let (tx, rx) = sync_channel(16);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
        assert_eq!(rx.try_iter().count(), 0, "no header");
        assert!(d.link.lock().unwrap().is_some());
    }

    /// What api.c:647-649 hangs on: an empty transfer is taken again.
    #[test]
    fn empty_and_failed_transfers_are_taken_again() {
        let steps = format!(
            "bulk_in 86 1024 -> -\nbulk_in 86 512 -> -\n\
             bulk_in 86 512 -> err device \"endpoint stalled\"\n\
             bulk_in 86 512 -> {}\n{STOP}",
            data(512)
        );
        assert_eq!(analog(&run(&acquisition(8), &steps)).len(), 2);
    }

    #[test]
    fn a_silent_or_vanished_scope_ends_the_capture() {
        let steps = format!("bulk_in 86 1024 -> -\nbulk_in 86 512 -> err timeout\n{STOP}");
        assert_eq!(analog(&run(&acquisition(8), &steps)).len(), 0);
        let steps = format!("bulk_in 86 1024 -> err disconnected\n{STOP}");
        assert_eq!(analog(&run(&acquisition(8), &steps)).len(), 0);
    }

    /// Bounded, and each failure counted, not only those in a row of one
    /// kind.
    #[test]
    fn failures_in_a_row_are_bounded() {
        let mut steps = "bulk_in 86 1024 -> -\n".to_owned();
        for _ in 0..MAX_EMPTY_TRANSFERS {
            steps.push_str("bulk_in 86 512 -> -\n");
        }
        steps.push_str(STOP);
        assert_eq!(analog(&run(&acquisition(8), &steps)).len(), 0);
    }

    /// CH2 alone is CH2, whatever api.c:588 calls it.
    #[test]
    fn a_disabled_ch1_leaves_ch2_its_name() {
        let acq = Acquisition { enabled: [false, true], ..acquisition(8) };
        let steps = format!("bulk_in 86 1024 -> -\nbulk_in 86 512 -> {}\n{STOP}", data(512));
        let packets = analog(&run(&acq, &steps));
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].channels, [1]);
    }

    fn device(profile: usize) -> Hantek6xxxDevice {
        Hantek6xxxDevice::new(&PROFILES[profile], UsbDeviceInfo::default(), None)
    }

    #[test]
    fn coupling_reads_back_what_was_set() {
        let mut d = device(0);
        assert!(d.set(&Key::Coupling, &Value::Str("AC".into()), Some("CH1")).is_err());
        d.set(&Key::Coupling, &Value::Str("DC".into()), Some("CH1")).unwrap();
        assert_eq!(d.get(&Key::Coupling, Some("CH1")).unwrap(), Value::Str("DC".into()));
        let mut d = device(3);
        d.set(&Key::Coupling, &Value::Str("AC".into()), Some("CH2")).unwrap();
        assert_eq!(d.get(&Key::Coupling, Some("CH2")).unwrap(), Value::Str("AC".into()));
        assert_eq!(d.get(&Key::Coupling, Some("CH1")).unwrap(), Value::Str("DC".into()));
    }

    #[test]
    fn a_rate_off_the_list_reads_back_as_what_it_runs_at() {
        let mut d = device(0);
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(8_000_000));
        d.set(&Key::Samplerate, &Value::U64(2_000_000), None).unwrap();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000));
        d.set(&Key::Samplerate, &Value::U64(48_000_000), None).unwrap();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(48_000_000));
    }

    #[test]
    fn options_follow_the_c_driver() {
        let d = device(7);
        assert_eq!(d.get(&Key::NumVdiv, None).unwrap(), Value::I32(6));
        assert_eq!(device(0).get(&Key::NumVdiv, None).unwrap(), Value::I32(4));
        assert_eq!(d.get(&Key::Vdiv, Some("CH1")).unwrap(), Value::Rational(288, 1000));
        assert!(d.get(&Key::Vdiv, None).is_err());
        assert!(d.get(&Key::Samplerate, Some("CH1")).is_err());
        assert!(d.get(&Key::Vdiv, Some("CH3")).is_err());
        let mut d = device(0);
        assert!(d.set(&Key::Vdiv, &Value::Rational(1, 1), None).is_err());
        assert!(d.set(&Key::Samplerate, &Value::U64(1), Some("CH1")).is_err());
        assert!(d.set(&Key::Vdiv, &Value::Rational(2, 1), Some("CH1")).is_err());
        d.set(&Key::Vdiv, &Value::Rational(1, 2), Some("CH2")).unwrap();
        assert_eq!(d.get(&Key::Vdiv, Some("CH2")).unwrap(), Value::Rational(500, 1000));
    }
}
