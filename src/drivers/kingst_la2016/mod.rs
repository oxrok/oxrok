// Counterpart in sigrok:
//   libsigrok/src/hardware/kingst-la2016/api.c,
//   libsigrok/src/hardware/kingst-la2016/protocol.c (the acquisition),
//   Copyright (C) 2022 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   Copyright (C) 2020 Florian Schmidt <schmidt_florian@gmx.de>,
//   Copyright (C) 2013 Marcus Comstedt <marcus@mc.pp.se>,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   with commits by Kevin Grant, Helge Kruse, Mike Walters, Soeren Apel,
//   Uwe Hermann, Dave Platt
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! kingst-la2016: the Kingst LA2016, LA1016, LA1010, LA5016 and LA5032
//! logic analyzers, an FX2 in front of an FPGA, with two PWM outputs.
//!
//! Every model is the same FX2 (`77a1:01a2`). Without its firmware
//! (`iProduct` 0) the scan uploads `kingst-la-01a2.fw` over
//! [`crate::ezusb`] and waits up to 3 s for the device to come back
//! with `iProduct` 2. The scan then reads the EEPROM, which names the
//! model, and loads the model's FPGA bitstream unless the FPGA runs
//! one the driver can keep (see [`protocol`]). Captures go to the
//! device's sample memory, with its hardware trigger, and are
//! downloaded run length encoded once the FPGA is idle; the LA1010,
//! which has no memory, and `continuous=on` stream instead. Scan, open,
//! setup, polling and download follow `api.c` and `protocol.c` step by
//! step, so that the device sees the same transactions as under
//! sigrok-cli (checked with the emulator, `tools/emu/kingst-la2016.sh`).
//!
//! Bulk IN transfers have the C driver's 500 ms timeout
//! ([`Transport::bulk_in_stream_timed`]): a slow stream's data comes as
//! it arrives, not when 512 KiB have filled.
//!
//! Where the C driver is wrong the port does otherwise: a rate whose
//! divider does not fit 16 bits is refused, a failed poll of the run
//! state ends the capture (the C code takes it for an idle state and
//! downloads), samples past the sample limit are not passed on, and a
//! failure after the bulk transfers were submitted ends the
//! acquisition cleanly (the C driver leaves them in flight and libusb
//! aborts when it frees them).

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error,
    Key, OptionDesc, Packet, PacketSink, Result, Trigger, TriggerMatch, Value, ValueList,
};
use crate::usb::{BulkStream, Transport, UsbDeviceInfo, UsbId, libusb_error_name, split_conn};
use protocol::{Download, Feed, Link, Model, Pwm, Setup, Stream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// The FX2's id, before and after the firmware upload.
pub const ID: UsbId = UsbId { vendor: protocol::VID, product: protocol::PID };

/// The driver.
pub struct La2016Driver;

static CLASSES: [DeviceClass; 2] = [DeviceClass::LogicAnalyzer, DeviceClass::SignalGenerator];
static SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::ProbeNames];
static DEV_OPTIONS: [OptionDesc; 8] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET_LIST),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::VoltageThreshold, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::Continuous, Access::GET_SET),
];
static PWM_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Enabled, Access::GET_SET),
    OptionDesc::new(Key::OutputFrequency, Access::GET_SET),
    OptionDesc::new(Key::DutyCycle, Access::GET_SET),
];
static TRIGGER_MATCHES: [TriggerMatch; 4] =
    [TriggerMatch::Zero, TriggerMatch::One, TriggerMatch::Rising, TriggerMatch::Falling];
/// Level and edge matches; the stage and edge limits are the driver's
/// own messages at the start, as in libsigrok.
const TRIGGER_FEATURES: crate::core::TriggerFeatures = crate::core::TriggerFeatures {
    count: false,
    consecutive: false,
    duration: false,
    range: false,
    or: false,
    and: false,
    max_stages: 0,
};
/// `channel_names_logic[]`.
const LOGIC_NAMES: [&str; 32] = [
    "CH0", "CH1", "CH2", "CH3", "CH4", "CH5", "CH6", "CH7", "CH8", "CH9", "CH10", "CH11", "CH12",
    "CH13", "CH14", "CH15", "CH16", "CH17", "CH18", "CH19", "CH20", "CH21", "CH22", "CH23", "CH24",
    "CH25", "CH26", "CH27", "CH28", "CH29", "CH30", "CH31",
];
/// `channel_names_pwm[]`.
const PWM_NAMES: [&str; 2] = ["PWM1", "PWM2"];
/// libsigrok's poll interval of the run state (`usb_source_add(..., 50,
/// ...)`), which the device's capture progress is paced by.
const POLL_INTERVAL: Duration = Duration::from_millis(50);
/// Polls after the halt of a stop before the capture is given up, 5 s.
const POLLS_AFTER_STOP: usize = 100;
/// Consecutive failed bulk IN transfers after which an acquisition ends,
/// the bound fx2lafw has as `MAX_EMPTY_TRANSFERS`; the C driver of this
/// device resubmits for ever.
const MAX_FAILED_TRANSFERS: usize = 64;

/// `usb_get_port_path()`: `BUS-PORT.PORT...`.
fn port_path(d: &UsbDeviceInfo) -> String {
    let ports: Vec<String> = d.ports.iter().map(u8::to_string).collect();
    format!("{}-{}", d.bus, ports.join("."))
}

/// Whether `d` is the device scanned as `usb`: by its hub ports, which
/// stay the same when it renumerates. A backend that knows no ports
/// falls back to the bus, and to the address too when the device did not
/// renumerate.
fn same_device(d: &UsbDeviceInfo, usb: &UsbDeviceInfo, renumerated: bool) -> bool {
    if d.id() != ID || d.bus != usb.bus {
        return false;
    }
    if !usb.ports.is_empty() && !d.ports.is_empty() {
        return d.ports == usb.ports;
    }
    renumerated || d.address == usb.address
}

/// `iProduct` from the OS's copy of the descriptor; a backend or replay
/// that cannot tell is taken to have the firmware running.
fn iproduct(t: &mut dyn Transport) -> u8 {
    t.string_indexes().map_or(protocol::IPRODUCT_INDEX, |[_, p, _]| p)
}

/// `la2016_open_usb()`'s claim, with its messages.
fn claim(t: &mut dyn Transport) -> Result<()> {
    t.claim_interface(protocol::INTERFACE).inspect_err(|e| match e {
        Error::Busy => {
            log::error!("Cannot claim USB interface. Another program or driver using it?");
        }
        Error::Disconnected => log::error!("Device has been disconnected."),
        e => log::error!("Cannot claim USB interface: {}.", libusb_error_name(e)),
    })
}

/// `la2016_identify_read()` on the opened device: claimed, identified
/// and initialised, then released and closed.
fn identify_read(mut t: Box<dyn Transport>, show_message: bool) -> Result<&'static Model> {
    claim(t.as_mut()).inspect_err(|_| {
        if show_message {
            log::error!("Cannot communicate to MCU firmware.");
        }
    })?;
    let mut link = Link::new(t);
    let r = link.identify_device(show_message).and_then(|m| link.init_hardware(m).map(|()| m));
    link.close();
    r
}

/// `ezusb_upload_firmware()` of the MCU firmware.
fn upload(info: &UsbDeviceInfo, mut t: Box<dyn Transport>) -> Result<()> {
    let name = protocol::mcu_firmware(info.product_id);
    log::info!("USB PID {:04x}, MCU firmware '{name}'.", info.product_id);
    log::info!("uploading firmware to device on {}.{}", info.bus, info.address);
    let image = crate::core::firmware::read(&name, "see docs/DEVICES.md")?;
    t.force_configuration(protocol::CONFIGURATION).inspect_err(|e| {
        log::error!("Unable to set configuration: {}", libusb_error_name(e));
    })?;
    crate::ezusb::upload_firmware_logged(t.as_mut(), &name, &image)
}

/// `la2016_identify_enum()`: the device at the scanned ports, with its
/// firmware running, identified quietly. The device it was found as.
fn identify_enum(usb: &UsbDeviceInfo) -> Result<(UsbDeviceInfo, &'static Model)> {
    for d in crate::usb::find_devices(&[ID])? {
        if !same_device(&d, usb, true) {
            continue;
        }
        let Ok(mut t) = crate::usb::open(&d) else { continue };
        if iproduct(t.as_mut()) != protocol::IPRODUCT_INDEX {
            continue;
        }
        let model = identify_read(t, false)?;
        return Ok((d, model));
    }
    Err(Error::device("device not found"))
}

/// `la2016_identify_wait()`: the device looked for every 200 ms from
/// 1.8 s after the upload until 3 s after it.
fn identify_wait(
    usb: &UsbDeviceInfo,
    uploaded: Instant,
) -> Result<(UsbDeviceInfo, &'static Model)> {
    log::info!("Waiting for device to reset after firmware upload.");
    if let Some(left) = protocol::RENUM_GONE_DELAY.checked_sub(uploaded.elapsed()) {
        std::thread::sleep(left);
    }
    loop {
        let elapsed = uploaded.elapsed();
        log::trace!("Waited {}ms.", elapsed.as_millis());
        match identify_enum(usb) {
            Ok(found) => {
                log::info!("Device came back after {}ms.", elapsed.as_millis());
                return Ok(found);
            }
            Err(e) => {
                std::thread::sleep(protocol::RENUM_POLL_INTERVAL);
                if elapsed >= protocol::RENUM_CHECK_PERIOD {
                    log::error!("Device failed to re-enumerate.");
                    return Err(e);
                }
            }
        }
    }
}

/// The devices `conn` names, as `sr_usb_find()` finds them, or the
/// message why none.
fn conn_devices(conn: &str) -> Result<Vec<UsbDeviceInfo>> {
    let found = match split_conn(conn) {
        Ok(c) => crate::usb::list_devices()?.into_iter().filter(|d| c.matches(d)).collect(),
        Err(msg) => {
            log::error!("{msg}");
            Vec::new()
        }
    };
    if found.is_empty() {
        log::error!("Cannot find the specified connection '{conn}'.");
    }
    Ok(found)
}

impl Driver for La2016Driver {
    fn id(&self) -> &'static str {
        "kingst-la2016"
    }

    fn long_name(&self) -> &'static str {
        "Kingst LA2016"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut probe_names = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::ProbeNames => probe_names = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let filter = match &conn {
            Some(c) => {
                let found = conn_devices(c)?;
                if found.is_empty() {
                    return Ok(Vec::new());
                }
                Some(found)
            }
            None => None,
        };
        let mut found = Vec::new();
        let mut renum = Vec::new();
        for info in crate::usb::list_devices()? {
            if let Some(f) = &filter
                && !f.iter().any(|d| d.bus == info.bus && d.address == info.address)
            {
                log::trace!(
                    "Bus {}, addr {} do not match specified filter.",
                    info.bus,
                    info.address
                );
                continue;
            }
            if info.id() != ID {
                continue;
            }
            let path = port_path(&info);
            log::debug!(
                "USB enum found {:04x}:{:04x} at path {path}, {}.{}.",
                info.vendor_id,
                info.product_id,
                info.bus,
                info.address
            );
            let mut t = match crate::usb::open(&info) {
                Ok(t) => t,
                Err(e) => {
                    log::error!("Cannot open device: {}.", libusb_error_name(&e));
                    log::error!("Cannot communicate to MCU firmware.");
                    log::error!("Unknown or unsupported device type.");
                    continue;
                }
            };
            if iproduct(t.as_mut()) != protocol::IPRODUCT_INDEX {
                // The firmware is missing: uploaded, the device
                // renumerates and is looked for again below.
                log::info!("Uploading MCU firmware to '{path}'.");
                match upload(&info, t) {
                    Ok(()) => renum.push((info, Instant::now())),
                    Err(e) => {
                        log::debug!("Firmware upload: {e}.");
                        log::error!("MCU firmware upload failed.");
                    }
                }
                continue;
            }
            log::info!(
                "USB PID {:04x}, MCU firmware '{}'.",
                info.product_id,
                protocol::mcu_firmware(info.product_id)
            );
            match identify_read(t, true) {
                Ok(model) => found.push((info, model)),
                Err(_) => log::error!("Unknown or unsupported device type."),
            }
        }
        for (info, uploaded) in renum {
            match identify_wait(&info, uploaded) {
                Ok(f) => found.push(f),
                Err(_) => log::debug!("Skipping unusable '{}'.", port_path(&info)),
            }
        }
        Ok(found
            .into_iter()
            .map(|(info, model)| {
                Box::new(La2016Device::new(info, model, probe_names.as_deref())) as Box<dyn Device>
            })
            .collect())
    }
}

/// Settings kept between acquisitions; the device cannot be read back,
/// so the scan sets libsigrok's defaults.
#[derive(Clone, Debug)]
struct Settings {
    samplerate: u64,
    limit_samples: u64,
    limit_msec: u64,
    capture_ratio: u64,
    continuous: bool,
    threshold: usize,
    pwm: [Pwm; 2],
}

/// The channel groups of `get_cg_index()`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Group {
    Logic,
    Pwm(usize),
}

/// One device.
pub struct La2016Device {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    model: &'static Model,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    settings: Settings,
    trigger: Option<Trigger>,
    /// A transport handed in (a replay), opened and initialised.
    pending: Option<Link>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// Errors of a configuration request that does not apply, as libsigrok's
/// `SR_ERR_NA`.
fn not_applicable() -> Error {
    Error::not_supported("not applicable")
}

impl La2016Device {
    fn new(usb: UsbDeviceInfo, model: &'static Model, probe_names: Option<&str>) -> Self {
        let n = model.channel_count.min(LOGIC_NAMES.len());
        let names = match probe_names {
            Some(spec) => crate::core::util::parse_probe_names(spec, &LOGIC_NAMES[..n]),
            None => LOGIC_NAMES[..n].iter().map(|s| (*s).to_owned()).collect(),
        };
        let mut channels: Vec<Channel> = names
            .into_iter()
            .enumerate()
            .map(|(i, name)| Channel::new(i, ChannelKind::Logic, name))
            .collect();
        let logic = channels.len();
        let mut groups = vec![ChannelGroup::new("Logic", (0..logic).collect())];
        for (i, name) in PWM_NAMES.iter().enumerate() {
            let mut c = Channel::new(logic + i, ChannelKind::Analog, *name);
            c.enabled = false;
            channels.push(c);
            groups.push(ChannelGroup::new(*name, vec![logic + i]));
        }
        La2016Device {
            info: DeviceInfo {
                driver: "kingst-la2016",
                vendor: "Kingst".into(),
                model: model.name.into(),
                version: String::new(),
                serial: String::new(),
                conn: Some(usb.conn()),
            },
            usb,
            model,
            channels,
            groups,
            settings: Settings {
                samplerate: model.samplerate,
                limit_samples: 0,
                limit_msec: 0,
                capture_ratio: 50,
                continuous: model.memory_bits == 0,
                threshold: protocol::THRESHOLD_IDX_DFLT,
                pwm: [
                    Pwm { enabled: false, freq: 1e3, duty: 50.0 },
                    Pwm { enabled: false, freq: 100e3, duty: 50.0 },
                ],
            },
            trigger: None,
            pending: None,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording),
    /// its firmware running: identified and initialised on it, and kept
    /// open.
    pub fn from_transport(mut transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        if usb.id() != ID {
            return Err(Error::arg("not a Kingst LA"));
        }
        claim(transport.as_mut())?;
        let mut link = Link::new(transport);
        let model = link.identify_device(true)?;
        link.init_hardware(model)?;
        let mut d = Self::new(usb, model, None);
        d.pending = Some(link);
        Ok(d)
    }

    /// The group `name` stands for.
    fn group(&self, name: Option<&str>) -> Result<Option<Group>> {
        let Some(name) = name else { return Ok(None) };
        match self.groups.iter().position(|g| g.name.eq_ignore_ascii_case(name)) {
            Some(0) => Ok(Some(Group::Logic)),
            Some(i) => Ok(Some(Group::Pwm(i - 1))),
            None => Err(Error::ChannelGroup(name.to_owned())),
        }
    }

    /// The mask of the enabled logic channels (`la2016_prepare_stream()`).
    fn channel_mask(&self) -> u32 {
        self.channels
            .iter()
            .filter(|c| c.kind == ChannelKind::Logic && c.enabled && c.index < 32)
            .fold(0, |m, c| m | 1 << c.index)
    }

    /// Whether the acquisition owns the device, so a PWM setting would
    /// never reach it.
    fn pwm_refused(&self) -> Result<()> {
        if self.link.lock().expect("link lock").is_none() && self.worker.is_some() {
            return Err(Error::Busy);
        }
        Ok(())
    }

    /// `la2016_write_pwm_config()` when the device is open. During a
    /// capture the acquisition owns the device: refused.
    fn write_pwm(&self, idx: usize) -> Result<()> {
        let mut slot = self.link.lock().expect("link lock");
        match slot.as_mut() {
            Some(link) => link.set_pwm_config(&self.settings.pwm, idx),
            None if self.worker.is_some() => Err(Error::Busy),
            None => Ok(()),
        }
    }

    /// `dev_acquisition_start()` up to the start: the setup, then the
    /// run mode, or for a stream the bulk reset and the transfers.
    fn begin(&self, link: &mut Link, start: Instant) -> std::result::Result<Acquisition, Failure> {
        let mask = self.channel_mask();
        let stages: Vec<Vec<(usize, TriggerMatch)>> = self
            .trigger
            .iter()
            .flat_map(|t| &t.stages)
            .map(|st| st.conditions.iter().map(|c| (c.channel, c.matches)).collect())
            .collect();
        let s = &self.settings;
        #[allow(clippy::cast_possible_truncation)]
        let setup = Setup {
            model: self.model,
            voltage: protocol::THRESHOLDS[s.threshold] as f32,
            continuous: s.continuous,
            channel_mask: mask,
            trigger: &stages,
            samplerate: s.samplerate,
            limit_samples: s.limit_samples,
            capture_ratio: s.capture_ratio,
        };
        let trigger_involved = link.setup_acquisition(&setup).map_err(Failure::Setup)?;
        let stream = if s.continuous {
            link.bulk_reset().map_err(Failure::Start)?;
            let stream = link
                .transport()
                .bulk_in_stream_timed(
                    protocol::EP_CAPTURE_DATA,
                    protocol::USB_BUFSZ,
                    protocol::USB_XFER_COUNT,
                    protocol::CAPTURE_TIMEOUT,
                )
                .inspect_err(|e| {
                    log::error!("Cannot submit USB transfer: {}.", libusb_error_name(e));
                })
                .map_err(Failure::Start)?;
            Some(stream)
        } else {
            link.set_run_mode(protocol::RUNMODE_RUN).map_err(Failure::Start)?;
            None
        };
        Ok(Acquisition {
            model: self.model,
            stream,
            channel_mask: mask,
            trigger_involved,
            limit_samples: s.limit_samples,
            limit_msec: s.limit_msec,
            samplerate: s.samplerate,
            start,
            failure: None,
        })
    }
}

/// Where a start failed: libsigrok's session stops the device either
/// way (session.c:831-835), and a failure after the setup also aborts
/// in the driver first, each a halt.
enum Failure {
    Setup(Error),
    Start(Error),
}

impl From<Failure> for Error {
    fn from(f: Failure) -> Error {
        match f {
            Failure::Setup(e) | Failure::Start(e) => e,
        }
    }
}

impl Device for La2016Device {
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
        match self.group(group)? {
            None => Ok(DEV_OPTIONS.to_vec()),
            // The threshold is the device's (WITH_THRESHOLD_DEVCFG): the
            // group has no option, and config_list() answers SR_ERR_NA.
            Some(Group::Logic) => Err(not_applicable()),
            Some(Group::Pwm(_)) => Ok(PWM_OPTIONS.to_vec()),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let s = &self.settings;
        Ok(match (self.group(group)?, key) {
            (Some(Group::Pwm(i)), Key::Enabled) => Value::Bool(s.pwm[i].enabled),
            (Some(Group::Pwm(i)), Key::OutputFrequency) => Value::F64(f64::from(s.pwm[i].freq)),
            (Some(Group::Pwm(i)), Key::DutyCycle) => Value::F64(f64::from(s.pwm[i].duty)),
            (Some(_), _) => return Err(not_applicable()),
            (None, Key::Conn) => Value::Str(self.usb.conn()),
            (None, Key::Samplerate) => Value::U64(s.samplerate),
            (None, Key::LimitSamples) => Value::U64(s.limit_samples),
            (None, Key::LimitMsec) => Value::U64(s.limit_msec),
            (None, Key::CaptureRatio) => Value::U64(s.capture_ratio),
            (None, Key::VoltageThreshold) => {
                let v = protocol::THRESHOLDS[s.threshold];
                Value::RangeF64(v, v)
            }
            (None, Key::Continuous) => Value::Bool(s.continuous),
            (None, other) => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match (self.group(group)?, key) {
            (Some(Group::Pwm(i)), Key::Enabled) => {
                self.pwm_refused()?;
                self.settings.pwm[i].enabled = value.as_bool().ok_or_else(bad)?;
                return self.write_pwm(i);
            }
            (Some(Group::Pwm(i)), Key::OutputFrequency) => {
                let f = value.as_f64().ok_or_else(bad)?;
                if f <= 0.0 || f > protocol::MAX_PWM_FREQ {
                    return Err(bad());
                }
                #[allow(clippy::cast_possible_truncation)]
                let f = f as f32;
                self.pwm_refused()?;
                self.settings.pwm[i].freq = f;
                return self.write_pwm(i);
            }
            (Some(Group::Pwm(i)), Key::DutyCycle) => {
                let d = value.as_f64().ok_or_else(bad)?;
                if d <= 0.0 || d > 100.0 {
                    return Err(bad());
                }
                #[allow(clippy::cast_possible_truncation)]
                let d = d as f32;
                self.pwm_refused()?;
                self.settings.pwm[i].duty = d;
                return self.write_pwm(i);
            }
            (Some(_), _) => return Err(not_applicable()),
            (None, Key::Samplerate) => self.settings.samplerate = value.as_u64().ok_or_else(bad)?,
            (None, Key::LimitSamples) => {
                self.settings.limit_samples = value.as_u64().ok_or_else(bad)?;
            }
            (None, Key::LimitMsec) => self.settings.limit_msec = value.as_u64().ok_or_else(bad)?,
            (None, Key::CaptureRatio) => {
                // sr_config_set() refuses it before the driver sees it
                // (hwdriver.c, check_key()).
                let r = value.as_u64().ok_or_else(bad)?;
                if r > 100 {
                    return Err(bad());
                }
                self.settings.capture_ratio = r;
            }
            (None, Key::VoltageThreshold) => {
                let Value::RangeF64(low, high) = value else { return Err(bad()) };
                self.settings.threshold = protocol::threshold_index(*low, *high).ok_or_else(bad)?;
            }
            (None, Key::Continuous) => {
                let on = value.as_bool().ok_or_else(bad)?;
                if self.model.memory_bits == 0 && !on {
                    return Err(bad());
                }
                self.settings.continuous = on;
            }
            (None, other) => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if self.group(group)?.is_some() {
            return Err(not_applicable());
        }
        Ok(match key {
            Key::Samplerate => ValueList::U64(self.model.rates().to_vec()),
            Key::LimitSamples => ValueList::MinMaxU64(0, protocol::NUM_SAMPLES_MAX),
            Key::VoltageThreshold => {
                ValueList::RangeF64(protocol::THRESHOLDS.iter().map(|&v| (v, v)).collect())
            }
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `dev_open()`: the device found at the scanned ports with its
    /// firmware running, claimed, and the PWM outputs configured.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let mut link = if let Some(l) = self.pending.take() {
            l
        } else {
            let found = crate::usb::find_devices(&[ID])?
                .into_iter()
                .filter(|d| same_device(d, &self.usb, false))
                .find_map(|d| {
                    let mut t = crate::usb::open(&d).ok()?;
                    (iproduct(t.as_mut()) == protocol::IPRODUCT_INDEX).then_some((d, t))
                });
            let Some((d, mut t)) = found else {
                log::error!("Cannot open device.");
                return Err(Error::device("device not found"));
            };
            claim(t.as_mut()).inspect_err(|_| log::error!("Cannot open device."))?;
            self.usb = d;
            Link::new(t)
        };
        for i in 0..self.settings.pwm.len() {
            link.set_pwm_config(&self.settings.pwm, i)?;
        }
        *self.link.lock().expect("link lock") = Some(link);
        Ok(())
    }

    /// `dev_close()`: the interface released, the device closed; the
    /// PWM outputs keep running (`WITH_DEINIT_IN_CLOSE` 0).
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(w) = self.worker.take() {
            let _ = w.handle.join();
        }
        if let Some(link) = self.link.lock().expect("link lock").take() {
            log::info!(
                "Closing device on {} (logical) / {} (physical) interface {}.",
                self.usb.conn(),
                port_path(&self.usb),
                protocol::INTERFACE
            );
            link.close();
        }
        Ok(())
    }

    fn trigger_features(&self) -> crate::core::TriggerFeatures {
        TRIGGER_FEATURES
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&TRIGGER_FEATURES)?;
        }
        self.trigger = trigger;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        match self.worker.take() {
            Some(w) if w.handle.is_finished() => {
                let _ = w.handle.join();
            }
            Some(w) => {
                self.worker = Some(w);
                return Err(Error::Busy);
            }
            None => {}
        }
        let link = self.link.lock().expect("link lock").take();
        let Some(mut link) = link else {
            return Err(Error::device("device is not open"));
        };
        // sr_sw_limits_acquisition_start(): the time limit counts from
        // here.
        let start = Instant::now();
        match self.begin(&mut link, start) {
            Ok(acq) => {
                let slot = Arc::clone(&self.link);
                let stop = Arc::new(AtomicBool::new(false));
                let stop2 = Arc::clone(&stop);
                let handle = std::thread::Builder::new()
                    .name("kingst-la2016 acquisition".into())
                    .spawn(move || {
                        let outcome = acq.run(&mut link, &sink, &stop2);
                        // The link first: whoever sees the end may use it.
                        *slot.lock().expect("link lock") = Some(link);
                        sink.end(outcome);
                    })
                    .map_err(Error::Io)?;
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(f) => {
                // la2016_abort_acquisition() after a failed start, then
                // the session's dev_acquisition_stop(): a halt each.
                if let Failure::Start(_) = f {
                    let _ = link.set_run_mode(protocol::RUNMODE_HALT);
                }
                let _ = link.set_run_mode(protocol::RUNMODE_HALT);
                *self.link.lock().expect("link lock") = Some(link);
                Err(f.into())
            }
        }
    }

    /// Asks the acquisition to end, without waiting: a stop during a
    /// capture still downloads what it holds, and the thread must be able
    /// to hand its packets to the session calling this.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// One acquisition, on its own thread: `la2016_receive_data()`.
struct Acquisition {
    model: &'static Model,
    /// The stream's transfers, submitted at the start.
    stream: Option<Box<dyn BulkStream>>,
    channel_mask: u32,
    trigger_involved: bool,
    limit_samples: u64,
    limit_msec: u64,
    samplerate: u64,
    start: Instant,
    /// Why the capture or the stream gave up.
    failure: Option<Error>,
}

/// Counts consecutive failed transfers.
struct Failures(usize);

impl Failures {
    /// One more failed transfer: whether that is too many.
    fn failed(&mut self) -> bool {
        self.0 += 1;
        if self.0 >= MAX_FAILED_TRANSFERS {
            log::error!(
                "{MAX_FAILED_TRANSFERS} USB transfers failed, giving up: the data is incomplete."
            );
            return true;
        }
        false
    }
}

fn send(sink: &PacketSink, unit_size: usize, feed: Vec<Feed>) {
    for f in feed {
        sink.send(match f {
            Feed::Samples(d) => Packet::logic(unit_size, d),
            Feed::Trigger => Packet::Trigger,
        });
    }
}

impl Acquisition {
    /// The header, then the capture or the stream; the frame ended when
    /// it began. The end is the caller's.
    fn run(mut self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        sink.send(Packet::header());
        let framed = match self.stream.take() {
            Some(stream) => self.stream(link, stream, sink, stop),
            None => self.capture(link, sink, stop),
        };
        if framed {
            sink.send(Packet::FrameEnd);
        }
        self.failure.map_or(Ok(()), Err)
    }

    /// [`Failures::failed`]'s verdict as the failure of the capture.
    fn too_many_failures(&mut self) {
        self.failure = Some(Error::device(format!(
            "{MAX_FAILED_TRANSFERS} USB transfers failed: the data is incomplete"
        )));
    }

    fn time_up(&self) -> bool {
        self.limit_msec != 0 && self.start.elapsed() > Duration::from_millis(self.limit_msec)
    }

    /// The end of a download or stream (`download_finished`): the halt,
    /// and the transfers cancelled as the stream drops.
    fn finish(link: &mut Link, stream: Box<dyn BulkStream>) {
        log::debug!("Download finished, post processing.");
        let _ = link.set_run_mode(protocol::RUNMODE_HALT);
        drop(stream);
        log::debug!("Download finished, done post processing.");
    }

    /// A capture to the sample memory: the run state polled every 50 ms
    /// until idle, halting at the time limit or a stop, then the
    /// download. Whether a frame began.
    fn capture(&mut self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) -> bool {
        let mut halted_for_stop = None;
        loop {
            std::thread::sleep(POLL_INTERVAL);
            let state = match link.run_state() {
                Ok(state) => state,
                Err(e) => {
                    // The C code reads the failure as an idle state and
                    // downloads a capture that has not finished.
                    let _ = link.set_run_mode(protocol::RUNMODE_HALT);
                    self.failure = Some(e);
                    return false;
                }
            };
            if Link::is_idle(state) {
                break;
            }
            match halted_for_stop.as_mut() {
                None if stop.load(Ordering::SeqCst) => {
                    let _ = link.set_run_mode(protocol::RUNMODE_HALT);
                    halted_for_stop = Some(0);
                }
                Some(n) => {
                    *n += 1;
                    if *n > POLLS_AFTER_STOP {
                        log::error!("Device stays busy after the halt, its capture is lost.");
                        self.failure = Some(Error::device(
                            "the device stays busy after the halt, its capture is lost",
                        ));
                        return false;
                    }
                }
                None => {}
            }
            if self.time_up() {
                self.limit_msec = 0;
                log::debug!("Limit reached. Stopping acquisition.");
                let _ = link.set_run_mode(protocol::RUNMODE_HALT);
            }
        }
        log::debug!("Acquisition completion seen (hardware).");
        link.dump_fpga_registers("acquisition complete");
        sink.send(Packet::FrameBegin);
        let (stream, mut download) = match self.start_download(link) {
            Ok(started) => started,
            Err(e) => {
                log::error!("Cannot start acquisition data download.");
                self.failure = Some(e);
                return true;
            }
        };
        log::debug!("Acquisition data download started.");
        let mut stream = stream;
        let unit_size = self.model.unit_size();
        let mut failures = Failures(0);
        // A stop that halted the capture wants what it holds; one that
        // comes during the download cancels it.
        let stop_seen = halted_for_stop.is_some() || stop.load(Ordering::SeqCst);
        while stop_seen || !stop.load(Ordering::SeqCst) {
            let feed = match stream.next(protocol::CAPTURE_TIMEOUT) {
                Ok(data) => {
                    log::debug!(
                        "receive_transfer(): status LIBUSB_SUCCESS received {} bytes.",
                        data.len()
                    );
                    // An empty transfer is no progress either: taken like
                    // a failed one, so a device that sends nothing ends.
                    if !data.is_empty() {
                        failures.0 = 0;
                    } else if failures.failed() {
                        self.too_many_failures();
                        break;
                    }
                    download.chunk(&data)
                }
                Err(Error::Disconnected) => {
                    log::warn!("Lost communication to USB device.");
                    self.failure = Some(Error::Disconnected);
                    break;
                }
                Err(e) => {
                    // Taken as an empty transfer, as receive_transfer()
                    // does: a download with nothing left ends there.
                    log::debug!(
                        "receive_transfer(): status {} received 0 bytes.",
                        libusb_error_name(&e)
                    );
                    if failures.failed() {
                        self.too_many_failures();
                        break;
                    }
                    download.chunk(&[])
                }
            };
            send(sink, unit_size, feed);
            if download.finished() {
                break;
            }
        }
        Self::finish(link, stream);
        true
    }

    /// `la2016_start_download()`: the capture info, the bulk reset, the
    /// window, the transfers submitted, the bulk start.
    fn start_download(&self, link: &mut Link) -> Result<(Box<dyn BulkStream>, Download)> {
        let layout = self.model.layout();
        let ppc = u32::try_from(layout.packets_per_chunk()).expect("5 or 6");
        let info = link.capture_info(ppc)?;
        let n_transfer_packets = info.n_rep_packets / ppc;
        let size = u32::try_from(layout.transfer_size).expect("16 or 32");
        let n_bytes = n_transfer_packets.wrapping_mul(size);
        let read_pos = info.write_pos.wrapping_sub(n_bytes);
        log::debug!("Want to read {n_transfer_packets} xfer-packets starting from pos {read_pos}.");
        link.bulk_reset().inspect_err(|_| log::error!("Cannot reset USB bulk state."))?;
        log::debug!("Will read from 0x{read_pos:08x}, 0x{n_bytes:08x} bytes.");
        link.bulk_window(read_pos, n_bytes)?;
        let stream = link
            .transport()
            .bulk_in_stream_timed(
                protocol::EP_CAPTURE_DATA,
                protocol::USB_BUFSZ,
                protocol::USB_XFER_COUNT,
                protocol::CAPTURE_TIMEOUT,
            )
            .inspect_err(|e| {
                log::error!("Cannot submit USB transfer: {}.", libusb_error_name(e));
                log::error!("Cannot submit USB bulk transfers.");
            })?;
        // A failed start drops the stream, which cancels the transfers.
        link.bulk_start()?;
        let download = Download::new(
            layout,
            &info,
            n_bytes,
            self.trigger_involved,
            self.limit_samples,
            self.samplerate,
        );
        Ok((stream, download))
    }

    /// Streaming: the run mode and the bulk start at the first poll,
    /// then transfers until the sample or time limit or a stop. Whether
    /// a frame began.
    fn stream(
        &mut self,
        link: &mut Link,
        mut stream: Box<dyn BulkStream>,
        sink: &PacketSink,
        stop: &AtomicBool,
    ) -> bool {
        log::debug!("First receive callback in stream mode.");
        sink.send(Packet::FrameBegin);
        if let Err(e) = link.set_run_mode(protocol::RUNMODE_RUN) {
            log::error!("Cannot set 'runmode' to 'run'.");
            self.failure = Some(e);
            return true;
        }
        if let Err(e) = link.bulk_start() {
            // The FPGA runs: halted, as at the end of a stream.
            let _ = link.set_run_mode(protocol::RUNMODE_HALT);
            self.failure = Some(e);
            return true;
        }
        log::debug!("Stream data reception initiated.");
        let unit_size = self.model.unit_size();
        let mut decoder = Stream::new(self.channel_mask, unit_size, self.limit_samples);
        let mut failures = Failures(0);
        while !stop.load(Ordering::SeqCst) {
            let feed = match stream.next(protocol::CAPTURE_TIMEOUT) {
                Ok(data) => {
                    failures.0 = 0;
                    decoder.chunk(&data)
                }
                Err(Error::Disconnected) => {
                    log::warn!("Lost communication to USB device.");
                    self.failure = Some(Error::Disconnected);
                    break;
                }
                // Nothing in the transfer timeout: legal at a slow rate
                // or while a device waits ("timeouts are perfectly legal",
                // protocol.c:1523).
                Err(Error::Timeout) => Vec::new(),
                Err(e) => {
                    log::debug!(
                        "receive_transfer(): status {} received 0 bytes.",
                        libusb_error_name(&e)
                    );
                    if failures.failed() {
                        self.too_many_failures();
                        break;
                    }
                    decoder.chunk(&[])
                }
            };
            send(sink, unit_size, feed);
            if decoder.finished() {
                break;
            }
            if self.time_up() {
                log::debug!("Acquisition end reached (sw limits).");
                break;
            }
        }
        Self::finish(link, stream);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn device(model: usize) -> La2016Device {
        La2016Device::new(UsbDeviceInfo::default(), &protocol::MODELS[model], None)
    }

    #[test]
    fn channels_and_groups_follow_the_c_driver() {
        let d = device(0);
        let names: Vec<&str> = d.channels.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names.len(), 18);
        assert_eq!(names[..2], ["CH0", "CH1"]);
        assert_eq!(names[16..], ["PWM1", "PWM2"]);
        assert!(!d.channels[16].enabled);
        assert_eq!(d.channel_mask(), 0xffff);
        assert_eq!(d.groups[1], ChannelGroup::new("PWM1", vec![16]));
        let d = device(6);
        assert_eq!(d.channels.len(), 34);
        assert_eq!(d.channel_mask(), 0xffff_ffff);
        let d = La2016Device::new(UsbDeviceInfo::default(), &protocol::MODELS[0], Some("a,b"));
        assert_eq!(d.channels[1].name, "b");
        assert_eq!(d.channels[2].name, "CH2");
    }

    #[test]
    fn options_and_defaults() {
        let mut d = device(0);
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(200_000_000));
        assert_eq!(d.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(1.4, 1.4));
        assert_eq!(d.get(&Key::OutputFrequency, Some("PWM1")).unwrap(), Value::F64(1000.0));
        assert_eq!(d.get(&Key::OutputFrequency, Some("pwm2")).unwrap(), Value::F64(100_000.0));
        assert!(d.get(&Key::Samplerate, Some("PWM1")).is_err());
        assert!(d.options(Some("Logic")).is_err());
        assert!(d.get(&Key::VoltageThreshold, Some("Logic")).is_err());
        assert!(d.options(Some("nope")).is_err());
        assert!(d.set(&Key::VoltageThreshold, &Value::RangeF64(3.3, 3.3), None).is_err());
        d.set(&Key::VoltageThreshold, &Value::RangeF64(2.5, 2.5), None).unwrap();
        assert_eq!(d.settings.threshold, 6);
        // Not open: kept for the open.
        d.set(&Key::OutputFrequency, &Value::F64(12345.0), Some("PWM2")).unwrap();
        assert!(d.set(&Key::OutputFrequency, &Value::F64(3e7), Some("PWM2")).is_err());
        assert!(d.set(&Key::DutyCycle, &Value::F64(0.0), Some("PWM2")).is_err());
        assert!(d.set(&Key::DutyCycle, &Value::F64(101.0), Some("PWM2")).is_err());
        assert!(d.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
        assert_eq!(d.get(&Key::Continuous, None).unwrap(), Value::Bool(false));
        // The LA1010 has no memory: it streams, and only streams.
        let mut d = device(7);
        assert_eq!(d.get(&Key::Continuous, None).unwrap(), Value::Bool(true));
        assert!(d.set(&Key::Continuous, &Value::Bool(false), None).is_err());
        assert_eq!(
            d.list(&Key::Samplerate, None).unwrap(),
            ValueList::U64(d.model.rates().to_vec())
        );
    }

    #[test]
    fn devices_are_found_again_by_their_ports() {
        let usb = UsbDeviceInfo {
            vendor_id: protocol::VID,
            product_id: protocol::PID,
            bus: 1,
            address: 5,
            ports: vec![1, 2],
            ..UsbDeviceInfo::default()
        };
        let moved = UsbDeviceInfo { address: 9, ..usb.clone() };
        assert!(same_device(&moved, &usb, false));
        let elsewhere = UsbDeviceInfo { ports: vec![1, 3], ..usb.clone() };
        assert!(!same_device(&elsewhere, &usb, true));
        let no_ports = UsbDeviceInfo { ports: Vec::new(), ..moved };
        assert!(!same_device(&no_ports, &usb, false));
        assert!(same_device(&no_ports, &usb, true));
        assert_eq!(port_path(&usb), "1-1.2");
    }

    /// An acquisition on a replayed link, the stop flag as given: the
    /// packets it sends, the steps played to their end.
    fn run_capture(steps: &str, stop: bool, limit_msec: u64) -> Vec<Packet> {
        let text = format!("# oxrok usb fixture 1\ndevice vid=77a1 pid=01a2\n{steps}");
        let r = crate::usb::record::Replay::from_text(&text).unwrap();
        let mut link = Link::new(Box::new(r.clone()));
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let acq = Acquisition {
            model: &protocol::MODELS[0],
            stream: None,
            channel_mask: 0xffff,
            trigger_involved: false,
            limit_samples: 0,
            limit_msec,
            samplerate: 1_000_000,
            start: Instant::now(),
            failure: None,
        };
        let _ = acq.run(&mut link, &PacketSink::new(tx, 0), &AtomicBool::new(stop));
        r.finish().unwrap();
        rx.try_iter().map(|(_, p)| p).skip(1).collect()
    }

    const DOWNLOAD: &str = "ctl_in 40 20 0010 0000 16 -> 05000000000000001000100000000000\n\
         ctl_out 40 38 0000 0000 - -> ok\n\
         ctl_out 40 20 0008 0000 0000100010000000 -> ok\n\
         ctl_out 40 30 0000 0000 - -> ok\n\
         bulk_in 86 524288 -> 01000200000000000000000000000000\n\
         ctl_out 40 20 0000 0000 00 -> ok\n";

    #[test]
    fn a_stop_halts_the_capture_and_downloads_what_it_holds() {
        let steps = format!(
            "ctl_in 40 20 0000 0000 2 -> ee85\n\
             ctl_out 40 20 0000 0000 00 -> ok\n\
             ctl_in 40 20 0000 0000 2 -> ed85\n{DOWNLOAD}"
        );
        let p = run_capture(&steps, true, 0);
        assert_eq!(p, [Packet::FrameBegin, Packet::logic(2, vec![1, 0, 1, 0]), Packet::FrameEnd]);
    }

    /// protocol.c:1633-1637: the time limit halts, and clears itself so
    /// that it halts once.
    #[test]
    fn a_time_limit_halts_once() {
        let steps = format!(
            "ctl_in 40 20 0000 0000 2 -> ee85\n\
             ctl_out 40 20 0000 0000 00 -> ok\n\
             ctl_in 40 20 0000 0000 2 -> ee85\n\
             ctl_in 40 20 0000 0000 2 -> ed85\n{DOWNLOAD}"
        );
        assert_eq!(run_capture(&steps, false, 1).len(), 3);
    }

    /// The C driver takes the failure for an idle state and downloads.
    #[test]
    fn a_failed_poll_halts_and_ends_without_a_frame() {
        let steps = "ctl_in 40 20 0000 0000 2 -> err device \"endpoint stalled\"\n\
                     ctl_out 40 20 0000 0000 00 -> ok\n";
        assert_eq!(run_capture(steps, false, 0), []);
    }

    /// A failed download start ends the frame; nothing runs any more, so
    /// no halt, as in the C driver.
    #[test]
    fn a_failed_download_start_ends_the_frame() {
        let steps = "ctl_in 40 20 0000 0000 2 -> ed85\n\
                     ctl_in 40 20 0010 0000 16 -> err device \"endpoint stalled\"\n";
        assert_eq!(run_capture(steps, false, 0), [Packet::FrameBegin, Packet::FrameEnd]);
    }

    /// A stop in the 50 ms before the poll that sees idle: the capture is
    /// done and is downloaded all the same.
    #[test]
    fn a_stop_just_before_the_idle_poll_keeps_the_capture() {
        let steps = format!("ctl_in 40 20 0000 0000 2 -> ed85\n{DOWNLOAD}");
        let p = run_capture(&steps, true, 0);
        assert_eq!(p, [Packet::FrameBegin, Packet::logic(2, vec![1, 0, 1, 0]), Packet::FrameEnd]);
    }

    /// A download a timed stream hands over piece by piece, the timeouts
    /// between them taken as empty transfers, as protocol.c:1564-1568.
    #[test]
    fn a_download_takes_partial_data_and_timeouts() {
        let steps = "ctl_in 40 20 0000 0000 2 -> ed85\n\
             ctl_in 40 20 0010 0000 16 -> 0a000000000000002000100000000000\n\
             ctl_out 40 38 0000 0000 - -> ok\n\
             ctl_out 40 20 0008 0000 0000100020000000 -> ok\n\
             ctl_out 40 30 0000 0000 - -> ok\n\
             bulk_in 86 524288 -> 01000200000000000000000000000000\n\
             bulk_in 86 524288 -> err timeout\n\
             bulk_in 86 524288 -> 02000300000000000000000000000000\n\
             ctl_out 40 20 0000 0000 00 -> ok\n";
        let p = run_capture(steps, false, 0);
        assert_eq!(
            p,
            [
                Packet::FrameBegin,
                Packet::logic(2, vec![1, 0, 1, 0]),
                Packet::logic(2, vec![2, 0, 2, 0, 2, 0]),
                Packet::FrameEnd,
            ]
        );
    }

    /// The acquisition owns the device: a PWM setting is refused and not
    /// kept as if it had been sent.
    #[test]
    fn a_pwm_set_during_a_capture_is_refused_and_not_stored() {
        let mut d = device(0);
        d.worker = Some(Worker {
            stop: Arc::new(AtomicBool::new(false)),
            handle: std::thread::spawn(|| {}),
        });
        let before = d.settings.pwm;
        assert!(matches!(d.set(&Key::Enabled, &Value::Bool(true), Some("PWM1")), Err(Error::Busy)));
        assert!(matches!(
            d.set(&Key::OutputFrequency, &Value::F64(5000.0), Some("PWM1")),
            Err(Error::Busy)
        ));
        assert!(matches!(
            d.set(&Key::DutyCycle, &Value::F64(25.0), Some("PWM1")),
            Err(Error::Busy)
        ));
        assert_eq!(d.settings.pwm, before);
    }

    #[test]
    fn too_many_failures_end_a_download() {
        let mut f = Failures(0);
        for _ in 1..MAX_FAILED_TRANSFERS {
            assert!(!f.failed());
        }
        assert!(f.failed());
    }
}
