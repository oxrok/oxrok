// Counterpart in sigrok:
//   libsigrok/src/hardware/dreamsourcelab-dslogic/api.c,
//   libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.c (the acquisition),
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   with commits by Uwe Hermann, Gerhard Sittig, Romain Tartière,
//   Joerg Alpers
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! dreamsourcelab-dslogic: the DreamSourceLab DSLogic, Pro, Plus and
//! Basic, and the DSCope driven as a logic analyzer; an FX2 in front of
//! an FPGA, 16 channels at 10 kHz to 400 MHz, a hardware trigger.
//!
//! The FX2 comes up without firmware (`2a0e:PID` without strings): the
//! scan uploads the model's FX2 firmware over [`crate::ezusb`] and the
//! device renumerates as "DreamSourceLab" / "USB-based Instrument";
//! opening waits for it, reads the firmware version and REVID, claims
//! the interface, loads the FPGA's bitstream and sets the threshold
//! (see [`protocol`]). A capture sends the setting packet and the start,
//! reads the trigger packet, then streams the samples. Scan, open,
//! transfer sizes and the sample stream follow `api.c` and `protocol.c`
//! step by step, so that the device sees the same transactions as under
//! sigrok-cli (checked with the emulator,
//! `tools/emu/dreamsourcelab-dslogic.sh`).
//!
//! libsigrok bugs not copied:
//!
//! - `fpga_configure()` sends the global trigger word uninitialised
//!   when there is no trigger (`set_trigger()` returns at
//!   `protocol.c:381-383` before it sets it); oxrok sends 0.
//! - The 90 % bound of the trigger position is computed in 32 bits
//!   (`protocol.c:374-376`) and wraps to about 10 % on the boards with
//!   256 Mibit of memory; oxrok bounds it at 90 %.
//! - `config_get(SR_CONF_CLOCK_EDGE)` answers `signal_edges[0]`,
//!   "rising", whatever was set (`api.c:444`); oxrok answers the edge
//!   set.
//! - `dev_open()` loads the first DSLogic's bitstream for threshold 0,
//!   the 3.3 V one, and only then sets the default threshold, 1.4 V,
//!   whose range (1.4-3.6, the 5 V bitstream's) `config_get()` reports
//!   (`api.c:355-366`, `protocol.c:242-245`); oxrok reports the range of
//!   the bitstream it loaded, 0.7-1.4, and writes the threshold register
//!   as the C driver does.
//! - A trigger on the capture's last sample leaves no samples after it,
//!   and `receive_transfer()` sends a logic packet of length 0
//!   (`protocol.c:861-862`), on which the hex output's
//!   `logic->length - logic->unitsize` wraps (`output/hex.c:163`) and
//!   sigrok-cli dies of SIGSEGV. oxrok sends no empty packet.
//! - `0=rf` makes two stages, both written to stage 0 (`protocol.c:386-
//!   421`), so sigrok-cli waits for "rising and falling", which is
//!   rising. oxrok refuses more than one stage.
//! - A trigger packet that STALLs or comes short completes into
//!   `trigger_receive()`, which has no branch for it (`protocol.c:1010-
//!   1026`): the capture never ends and a later abort cancels the freed
//!   transfer. oxrok ends the capture with an error.
//! - A threshold outside 0 to 5 V makes the DAC value an out of range
//!   conversion to `uint8_t` (`protocol.c:519`); oxrok refuses it.
//! - A transfer cut inside an atom makes `deinterleave_buffer()` read
//!   the rest of the atom from what the buffer held before
//!   (`protocol.c:732-748`); oxrok takes those bits as 0.
//! - Handles leak in `scan()` when a string read fails, and in
//!   `dslogic_dev_open()` on each failed try; Rust closes them.
//!
//! Where it stays different without being a bug of either: the sample
//! count of the setting packet saturates at 32 bits where the C code
//! truncates (above 64 Gsamples), a version or REVID answer short of
//! its length is refused where the C code takes any answer that is no
//! error and reads what its variable held, and a reopen of the first
//! DSLogic loads the bitstream of the range reported, where the C
//! driver's second open loads the 5 V one. Whether the first DSLogic has
//! the threshold register the open writes is unknown (no source here
//! says, and the model takes the write on every profile).
//!
//! Sample transfers have the C driver's timeout (`get_timeout()`, with
//! [`Transport::bulk_in_stream_timed`]): a transfer that times out
//! delivers what it holds (`protocol.c:802`), so a buffered capture whose
//! FPGA stops partway through a transfer still has that part, and a queue
//! that times out with nothing counts as that many empty transfers, so
//! the capture gives up after as many timeouts as libsigrok's does.

#![forbid(unsafe_code)]
#![allow(clippy::doc_markdown)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error,
    Key, OptionDesc, Packet, PacketSink, Result, Trigger, TriggerMatch, Value, ValueList,
};
use crate::drivers::saleae_logic16::split_conn;
use crate::usb::{Transport, UsbDeviceInfo, UsbId};
use protocol::{Capture, FpgaConfig, Link, PROFILES, Profile, SAMPLERATES, THRESHOLDS, VID};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const CONFIGURATION: u8 = 1;
const INTERFACE: u8 = 0;
const ENDPOINT_IN: u8 = 0x86;
const RENUM_TIMEOUT: Duration = Duration::from_secs(3);
/// Consecutive empty or failed transfers after which the FX2 is taken
/// to have given up (`MAX_EMPTY_TRANSFERS`).
const MAX_EMPTY_TRANSFERS: usize = 64;
/// How often the wait for the trigger packet looks at the stop flag.
const TRIGGER_POLL: Duration = Duration::from_millis(100);
/// The firmware's strings.
const MANUFACTURER: &str = "DreamSourceLab";
const PRODUCT: &str = "USB-based Instrument";

/// The driver.
pub struct DslogicDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 9] = [
    OptionDesc::new(Key::Continuous, Access::GET_SET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::VoltageThreshold, Access::GET_SET_LIST),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::ExternalClock, Access::GET_SET),
    OptionDesc::new(Key::ClockEdge, Access::GET_SET_LIST),
];
/// The FPGA's trigger as the driver uses it: one stage of level and
/// edge matches. The C driver writes the matches of every stage to
/// stage 0 (`protocol.c:386-421`), so "A then B" would become "A and B",
/// and 17 stages would overflow into the channel count: oxrok refuses
/// more than one.
const TRIGGER_FEATURES: crate::core::TriggerFeatures = crate::core::TriggerFeatures {
    count: false,
    consecutive: false,
    duration: false,
    range: false,
    or: false,
    and: false,
    max_stages: 1,
};
static TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];
/// `signal_edges[]`.
const CLOCK_EDGES: [&str; 2] = ["rising", "falling"];
const GROUP: &str = "Logic";

fn ids() -> Vec<UsbId> {
    PROFILES.iter().map(|p| UsbId { vendor: VID, product: p.pid }).collect()
}

impl Driver for DslogicDriver {
    fn id(&self) -> &'static str {
        "dreamsourcelab-dslogic"
    }

    fn long_name(&self) -> &'static str {
        "DreamSourceLab DSLogic"
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
        for info in crate::usb::find_devices(&ids())? {
            if conn.is_some_and(|c| !c.matches(&info)) {
                continue;
            }
            let Some(profile) = Profile::of(info.product_id) else { continue };
            let Some(serial) = read_strings(&info) else { continue };
            let mut dev = DslogicDevice::new(info, profile, serial, None);
            if has_firmware(&dev.usb) {
                log::debug!("Found a DSLogic device.");
            } else {
                match upload(&dev.usb, profile) {
                    Ok(()) => dev.fw_updated = Some(Instant::now()),
                    Err(e) => {
                        log::debug!("Firmware upload: {e}.");
                        log::error!(
                            "Firmware upload failed for device {}.{} (logical), name {}.",
                            dev.usb.bus,
                            dev.usb.address,
                            profile.firmware
                        );
                    }
                }
                // The address after the renumeration is not known yet
                // (0xff in the C driver).
                dev.info.conn = None;
            }
            devices.push(Box::new(dev));
        }
        Ok(devices)
    }
}

/// The string reads of `scan()`: the device opened, and each string it
/// has (a non-zero index) read, the manufacturer, the product and the
/// serial number. A device that does not open or answer is skipped with
/// a warning, as in the C driver. The serial number, or "" without one.
fn read_strings(info: &UsbDeviceInfo) -> Option<String> {
    let mut t = match crate::usb::open(info) {
        Ok(t) => t,
        Err(e) => {
            log::warn!(
                "Failed to open potential device with VID:PID {:04x}:{:04x}: {}.",
                info.vendor_id,
                info.product_id,
                protocol::libusb_name(&e)
            );
            return None;
        }
    };
    let indexes = t.string_indexes().unwrap_or([0; 3]);
    let mut serial = String::new();
    for (index, what) in indexes.into_iter().zip(["manufacturer", "product", "serial number"]) {
        if index == 0 {
            continue;
        }
        match crate::usb::read_string_ascii(t.as_mut(), index) {
            Ok(s) => {
                if what == "serial number" {
                    serial = s;
                }
            }
            Err(e) => {
                log::warn!(
                    "Failed to get {what} string descriptor: {}.",
                    protocol::libusb_name(&e)
                );
                return None;
            }
        }
    }
    Some(serial)
}

/// `usb_match_manuf_prod()`: the device opened again, its manufacturer,
/// then its product string read, the firmware's when it runs. A device
/// without strings (index 0) is read nothing.
fn has_firmware(info: &UsbDeviceInfo) -> bool {
    let Ok(mut t) = crate::usb::open(info) else { return false };
    let Ok([m, p, _]) = t.string_indexes() else { return false };
    crate::usb::read_string_ascii(t.as_mut(), m).is_ok_and(|s| s == MANUFACTURER)
        && crate::usb::read_string_ascii(t.as_mut(), p).is_ok_and(|s| s == PRODUCT)
}

/// `ezusb_upload_firmware()`: open, `SET_CONFIGURATION` 1, the image
/// in chunks of 4096 bytes with the 8051 held in reset, release.
fn upload(info: &UsbDeviceInfo, profile: &Profile) -> Result<()> {
    let image = crate::core::firmware::read(
        profile.firmware,
        "extract it from DreamSourceLab's DSView, see docs/DEVICES.md",
    )?;
    log::info!("Uploading firmware {} to device on {}.", profile.firmware, info.conn());
    let mut t = crate::usb::open(info)?;
    t.force_configuration(CONFIGURATION)?;
    crate::ezusb::upload_firmware(t.as_mut(), &image)
}

/// Settings kept between acquisitions.
#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)]
struct Settings {
    samplerate: u64,
    limit_samples: u64,
    capture_ratio: u64,
    continuous: bool,
    external_clock: bool,
    falling_edge: bool,
    /// The threshold, `None` until the first open sets the default;
    /// on the first DSLogic the lower bound of a [`THRESHOLDS`] range.
    threshold: Option<f64>,
    /// A threshold set while the device was closed, for the open to
    /// write.
    threshold_pending: bool,
}

/// One DSLogic.
pub struct DslogicDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    profile: &'static Profile,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    settings: Settings,
    trigger: Option<Trigger>,
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

impl DslogicDevice {
    fn new(
        usb: UsbDeviceInfo,
        profile: &'static Profile,
        serial: String,
        transport: Option<Box<dyn Transport>>,
    ) -> Self {
        DslogicDevice {
            info: DeviceInfo {
                driver: "dreamsourcelab-dslogic",
                vendor: "DreamSourceLab".into(),
                model: profile.model.into(),
                version: String::new(),
                serial,
                conn: Some(usb.conn()),
            },
            usb,
            profile,
            // Channels named "0" to "15", all in the group "Logic".
            channels: (0..16).map(|i| Channel::new(i, ChannelKind::Logic, i.to_string())).collect(),
            groups: vec![ChannelGroup::new(GROUP, (0..16).collect())],
            settings: Settings {
                samplerate: SAMPLERATES[0],
                limit_samples: 0,
                capture_ratio: 0,
                continuous: false,
                external_clock: false,
                falling_edge: false,
                threshold: None,
                threshold_pending: false,
            },
            trigger: None,
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
            fw_updated: None,
        }
    }

    /// A device on an already open transport (a replayed recording),
    /// its firmware running.
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        let profile = (usb.vendor_id == VID)
            .then(|| Profile::of(usb.product_id))
            .flatten()
            .ok_or_else(|| Error::arg("not a DreamSourceLab DSLogic"))?;
        Ok(Self::new(usb, profile, String::new(), Some(transport)))
    }

    /// The mask of the enabled channels.
    fn channel_mask(&self) -> u16 {
        self.channels.iter().filter(|c| c.enabled).fold(0, |m, c| m | 1 << c.index)
    }

    /// The bitstream for the threshold, as `dslogic_fpga_firmware_upload()`
    /// picks it (threshold 0 before the first open).
    fn bitstream(&self) -> &'static str {
        self.profile.bitstream(self.settings.threshold.unwrap_or(0.0))
    }

    /// The claim and the rest of `dev_open()`: the bitstream, then the
    /// threshold. The first open sets the default threshold, 1.4 V, in
    /// the register; on the first DSLogic, whose threshold is the
    /// bitstream, the register gets it all the same, as in the C
    /// driver, and the threshold is the range of the bitstream loaded.
    fn set_up(&mut self, mut link: Link) -> Result<Link> {
        link.transport().set_configuration(CONFIGURATION)?;
        link.transport().claim_interface(INTERFACE).map_err(|e| {
            let msg = match &e {
                Error::Busy => "Unable to claim USB interface. Another program or driver has \
                                already claimed it."
                    .to_owned(),
                Error::Disconnected => "Device has been disconnected.".to_owned(),
                e => format!("Unable to claim interface: {}.", protocol::libusb_name(e)),
            };
            log::error!("{msg}");
            Error::Device(msg)
        })?;
        link.upload_bitstream(self.bitstream())?;
        self.open_threshold(&mut link)?;
        Ok(link)
    }

    /// The end of `dev_open()`: the first open writes the default to the
    /// register (whether the first DSLogic has that register is unknown;
    /// the C driver writes it there too), a later one nothing, unless a
    /// threshold was set while the device was closed.
    fn open_threshold(&mut self, link: &mut Link) -> Result<()> {
        let by_bitstream = self.profile.threshold_by_bitstream();
        match self.settings.threshold {
            None => {
                link.set_threshold(protocol::DEFAULT_THRESHOLD)?;
                self.settings.threshold =
                    Some(if by_bitstream { THRESHOLDS[0].0 } else { protocol::DEFAULT_THRESHOLD });
            }
            Some(t) if self.settings.threshold_pending && !by_bitstream => link.set_threshold(t)?,
            Some(_) => {}
        }
        self.settings.threshold_pending = false;
        Ok(())
    }

    /// `dslogic_acquisition_start()` up to the trigger transfer: the stop
    /// command, the setting packet, the start command.
    fn begin(&self, link: &mut Link) -> Result<Acquisition> {
        let mask = self.channel_mask();
        let count = mask.count_ones();
        if mask == 0 {
            return Err(Error::arg("no channel enabled"));
        }
        let s = &self.settings;
        let (size, depth, timeout) = protocol::transfer_plan(s.samplerate, count, s.continuous)
            .ok_or_else(|| Error::Arg(format!("Unable to sample at {}Hz.", s.samplerate)))?;
        let trigger = self.trigger.as_ref().map(|t| {
            t.stages
                .iter()
                .map(|st| st.conditions.iter().map(|c| (c.channel, c.matches)).collect())
                .collect()
        });
        let capture = Capture {
            samplerate: s.samplerate,
            limit_samples: s.limit_samples,
            capture_ratio: s.capture_ratio,
            continuous: s.continuous,
            external_clock: s.external_clock,
            falling_edge: s.falling_edge,
            channels: mask,
            trigger,
        };
        if capture.trigger.is_none() {
            log::debug!("No session trigger found");
        }
        let cfg = FpgaConfig::new(&capture, self.profile.mem_depth);
        link.stop()?;
        link.configure(&cfg)?;
        link.start()?;
        log::debug!("Getting trigger.");
        Ok(Acquisition { mask, limit: s.limit_samples, size, depth, timeout })
    }
}

/// The group a request names, which the C driver ignores.
fn check_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) if !g.eq_ignore_ascii_case(GROUP) => Err(Error::ChannelGroup(g.to_owned())),
        _ => Ok(()),
    }
}

/// Whether `d` is the device scanned as `usb`: by its hub ports, which
/// stay the same when it renumerates (`usb_get_port_path()`). A backend
/// that knows no ports falls back to the address, or after an upload to
/// a device on the same bus with the firmware's strings.
fn same_device(d: &UsbDeviceInfo, usb: &UsbDeviceInfo, after_upload: bool) -> bool {
    if d.bus != usb.bus {
        return false;
    }
    if !usb.ports.is_empty() && !d.ports.is_empty() {
        return d.ports == usb.ports;
    }
    if after_upload {
        d.manufacturer.as_deref() == Some(MANUFACTURER)
    } else {
        d.address == usb.address
    }
}

/// `dslogic_dev_open()`: the device of the profile at the scanned port,
/// opened, its firmware version and REVID read, the version's major
/// checked.
fn try_open(
    usb: &UsbDeviceInfo,
    profile: &Profile,
    after_upload: bool,
) -> Result<(UsbDeviceInfo, Link)> {
    let id = UsbId { vendor: VID, product: profile.pid };
    let found =
        crate::usb::find_devices(&[id])?.into_iter().find(|d| same_device(d, usb, after_upload));
    let info = found.ok_or_else(|| Error::device("device not found"))?;
    let t = crate::usb::open(&info).inspect_err(|e| {
        log::error!("Failed to open device: {}.", protocol::libusb_name(e));
    })?;
    let link = identify(Link::new(t), &info)?;
    Ok((info, link))
}

/// The version checks of `dslogic_dev_open()`.
fn identify(mut link: Link, info: &UsbDeviceInfo) -> Result<Link> {
    let (major, minor) = link.firmware_version().inspect_err(|_| {
        log::error!("Failed to get firmware version.");
    })?;
    let revid = link.revid().inspect_err(|_| log::error!("Failed to get REVID."))?;
    // A new major version means an incompatible firmware.
    if major != 1 {
        let msg = format!("Expected firmware version 1.x, got {major}.{minor}.");
        log::error!("{msg}");
        return Err(Error::Device(msg));
    }
    log::info!(
        "Opened device on {} (logical) / {:?} (physical), interface {INTERFACE}, firmware \
         {major}.{minor}.",
        info.conn(),
        info.ports
    );
    log::info!(
        "Detected REVID={revid}, it's a Cypress CY7C68013{}.",
        if revid == 1 { "A (FX2LP)" } else { " (FX2)" }
    );
    Ok(link)
}

impl Device for DslogicDevice {
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
            // STD_CONFIG_LIST() lists nothing for a channel group.
            Some(g) if g.eq_ignore_ascii_case(GROUP) => Ok(Vec::new()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        check_group(group)?;
        let s = &self.settings;
        Ok(match key {
            // Unknown until the device came back after the upload.
            Key::Conn => match &self.info.conn {
                Some(c) => Value::Str(c.clone()),
                None => return Err(Error::device("the device has not renumerated yet")),
            },
            Key::VoltageThreshold => {
                let t = s
                    .threshold
                    .ok_or_else(|| Error::device("the threshold is set when the device opens"))?;
                if self.profile.threshold_by_bitstream() {
                    // std_double_tuple_idx_d0(): the lower bound exactly.
                    #[allow(clippy::float_cmp)]
                    let (low, high) = THRESHOLDS
                        .into_iter()
                        .find(|r| r.0 == t)
                        .ok_or_else(|| Error::device("threshold of no range"))?;
                    Value::RangeF64(low, high)
                } else {
                    Value::RangeF64(t, t)
                }
            }
            Key::LimitSamples => Value::U64(s.limit_samples),
            Key::Samplerate => Value::U64(s.samplerate),
            Key::CaptureRatio => Value::U64(s.capture_ratio),
            Key::ExternalClock => Value::Bool(s.external_clock),
            Key::Continuous => Value::Bool(s.continuous),
            Key::ClockEdge => Value::Str(CLOCK_EDGES[usize::from(s.falling_edge)].into()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        check_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let s = &mut self.settings;
        match key {
            Key::Samplerate => {
                let r = value.as_u64().ok_or_else(bad)?;
                if !SAMPLERATES.contains(&r) {
                    return Err(bad());
                }
                s.samplerate = r;
            }
            Key::LimitSamples => {
                let n = value.as_u64().ok_or_else(bad)?;
                // sr_config_set() refuses 0 (hwdriver.c:746-755).
                if n == 0 {
                    log::error!("Cannot set 'limit_samples' to 0.");
                    return Err(bad());
                }
                s.limit_samples = n;
            }
            Key::CaptureRatio => {
                let r = value.as_u64().ok_or_else(bad)?;
                // sr_config_set() refuses it before the driver sees it
                // (hwdriver.c:757-764).
                if r > 100 {
                    log::error!("Capture ratio must be 0..100.");
                    return Err(Error::arg("capture ratio must be 0..100"));
                }
                s.capture_ratio = r;
            }
            Key::VoltageThreshold => {
                let Value::RangeF64(low, high) = *value else { return Err(bad()) };
                return self.set_threshold(low, high);
            }
            Key::ExternalClock => s.external_clock = value.as_bool().ok_or_else(bad)?,
            Key::Continuous => s.continuous = value.as_bool().ok_or_else(bad)?,
            Key::ClockEdge => {
                let v = value.as_str().ok_or_else(bad)?;
                let i = CLOCK_EDGES.iter().position(|e| *e == v).ok_or_else(bad)?;
                s.falling_edge = i == 1;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        check_group(group)?;
        Ok(match key {
            Key::VoltageThreshold => {
                if self.profile.threshold_by_bitstream() {
                    ValueList::RangeF64(THRESHOLDS.to_vec())
                } else {
                    ValueList::RangeF64(protocol::threshold_steps())
                }
            }
            Key::Samplerate => ValueList::U64(SAMPLERATES.to_vec()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::ClockEdge => ValueList::Str(CLOCK_EDGES.iter().map(|e| (*e).to_owned()).collect()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let unable = |e: Error| {
            log::error!("Unable to open device.");
            e
        };
        let link = if let Some(t) = self.pending.take() {
            let info = t.info().clone();
            identify(Link::new(t), &info).map_err(unable)?
        } else if let Some(uploaded) = self.fw_updated {
            // dev_open(): wait for the FX2 to renumerate, trying every
            // 100 ms until 3 s after the upload.
            log::info!("Waiting for device to reset.");
            std::thread::sleep(Duration::from_millis(300));
            let (info, link) = loop {
                match try_open(&self.usb, self.profile, true) {
                    Ok(found) => break found,
                    Err(e) => {
                        std::thread::sleep(Duration::from_millis(100));
                        log::trace!("Waited {} ms.", uploaded.elapsed().as_millis());
                        if uploaded.elapsed() >= RENUM_TIMEOUT {
                            log::debug!("Last attempt: {e}.");
                            log::error!("Device failed to renumerate.");
                            return Err(Error::device("the device failed to renumerate"));
                        }
                    }
                }
            };
            log::info!("Device came back after {} ms.", uploaded.elapsed().as_millis());
            self.fw_updated = None;
            self.info.conn = Some(info.conn());
            self.usb = info;
            link
        } else {
            log::info!("Firmware upload was not needed.");
            let (info, link) = try_open(&self.usb, self.profile, false).map_err(unable)?;
            self.info.conn = Some(info.conn());
            self.usb = info;
            link
        };
        let link = self.set_up(link)?;
        *self.link.lock().expect("link lock") = Some(link);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(mut link) = self.link.lock().expect("link lock").take() {
            let _ = link.transport().release_interface(INTERFACE);
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
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let link = self.link.lock().expect("link lock").take();
        let Some(mut link) = link else {
            return Err(Error::device("device is not open"));
        };
        match self.begin(&mut link) {
            Ok(acq) => {
                let slot = Arc::clone(&self.link);
                let stop = Arc::new(AtomicBool::new(false));
                let stop2 = Arc::clone(&stop);
                let handle = std::thread::Builder::new()
                    .name("dreamsourcelab-dslogic acquisition".into())
                    .spawn(move || {
                        acq.run(&mut link, &sink, &stop2);
                        sink.send(Packet::End);
                        *slot.lock().expect("link lock") = Some(link);
                    })
                    .map_err(Error::Io)?;
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                // libsigrok's session stops a device whose start failed
                // (session.c:831-835): dslogic_acquisition_stop() sends
                // the stop command.
                let _ = link.stop();
                *self.link.lock().expect("link lock") = Some(link);
                Err(e)
            }
        }
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        Ok(())
    }
}

impl DslogicDevice {
    /// `config_set(SR_CONF_VOLTAGE_THRESHOLD)`. The first DSLogic takes
    /// a range of [`THRESHOLDS`] and loads its bitstream; the others take
    /// the middle of the range, 0 to 5 V, in the threshold register. A
    /// closed device keeps it for the open; a capture refuses it.
    fn set_threshold(&mut self, low: f64, high: f64) -> Result<()> {
        let bad = || Error::arg("invalid value for voltage_threshold");
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let mut slot = self.link.lock().expect("link lock");
        if self.profile.threshold_by_bitstream() {
            let i = protocol::threshold_index(low, high).ok_or_else(bad)?;
            self.settings.threshold = Some(THRESHOLDS[i].0);
            if let Some(link) = slot.as_mut() {
                link.upload_bitstream(self.profile.bitstream(THRESHOLDS[i].0))?;
            }
        } else {
            let t = f64::midpoint(low, high);
            protocol::threshold_command(t).ok_or_else(bad)?;
            match slot.as_mut() {
                Some(link) => link.set_threshold(t)?,
                None => self.settings.threshold_pending = true,
            }
            self.settings.threshold = Some(t);
        }
        Ok(())
    }
}

/// One acquisition, on its own thread: `trigger_receive()` and
/// `receive_transfer()`.
struct Acquisition {
    mask: u16,
    limit: u64,
    size: usize,
    depth: usize,
    timeout: Duration,
}

/// What a transfer leaves the acquisition to do.
#[derive(Debug, PartialEq, Eq)]
enum Next {
    Continue,
    End,
}

/// The counters of `struct dev_context` a capture keeps.
struct State {
    sent: u64,
    empty: usize,
    /// The trigger's sample (`real_pos`), 0 once sent or without one.
    trigger_pos: u64,
}

/// How the wait for the trigger packet ended.
enum TriggerPacket {
    /// The trigger's sample.
    Position(u32),
    /// The capture is over: stopped, the device gone, or an error.
    Over,
}

impl Acquisition {
    /// The trigger packet, then the sample transfers until the sample
    /// limit, the FX2 giving up, the device gone or the stop flag (which
    /// sends the stop command, `dslogic_acquisition_stop()`); the stream
    /// drops last, which cancels the transfers still queued.
    fn run(self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) {
        let real_pos = match Self::trigger_packet(link, stop) {
            TriggerPacket::Position(p) => p,
            TriggerPacket::Over => return,
        };
        // start_transfers().
        for i in 0..self.depth {
            log::info!("submitting transfer: {i}");
        }
        let stream =
            link.transport().bulk_in_stream_timed(ENDPOINT_IN, self.size, self.depth, self.timeout);
        let mut stream = match stream {
            Ok(s) => s,
            Err(e) => {
                log::error!("Failed to submit transfer: {}.", protocol::libusb_name(&e));
                return;
            }
        };
        sink.send(Packet::header());
        let mut state = State { sent: 0, empty: 0, trigger_pos: u64::from(real_pos) };
        loop {
            if stop.load(Ordering::SeqCst) {
                let _ = link.stop();
                break;
            }
            let transfer = match stream.next(self.timeout) {
                Err(Error::Disconnected) => {
                    // LIBUSB_TRANSFER_NO_DEVICE: the C driver ends the
                    // capture without a message.
                    log::debug!("Device has been disconnected.");
                    break;
                }
                other => other,
            };
            if self.transfer(transfer, &mut state, sink) == Next::End {
                break;
            }
        }
        drop(stream);
    }

    /// The trigger transfer of `dslogic_acquisition_start()`: a read of
    /// `struct dslogic_trigger_pos` for as long as the trigger takes,
    /// repeated every [`TRIGGER_POLL`] to look at the stop flag (the C
    /// transfer has no timeout), and `trigger_receive()`.
    fn trigger_packet(link: &mut Link, stop: &AtomicBool) -> TriggerPacket {
        // A plain read rather than a stream of one: the libusb backend's
        // stream reads ahead on its own thread, so it would take the
        // first sample transfer as well, and lose it when dropped.
        let packet = loop {
            if stop.load(Ordering::SeqCst) {
                // dslogic_acquisition_stop(): the stop command, then the
                // transfer cancelled.
                let _ = link.stop();
                log::debug!("Trigger transfer canceled.");
                return TriggerPacket::Over;
            }
            let read =
                link.transport().bulk_in(ENDPOINT_IN, protocol::TRIGGER_POS_LEN, TRIGGER_POLL);
            match read {
                Err(Error::Timeout) => {}
                Ok(p) => break Ok(p),
                Err(e) => break Err(e),
            }
        };
        match packet {
            Ok(p) if p.len() == protocol::TRIGGER_POS_LEN => {
                let word = |i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
                log::info!(
                    "tpos real_pos {} ram_saddr {} cnt_h {} cnt_l {}",
                    word(4),
                    word(8),
                    word(16),
                    word(12)
                );
                TriggerPacket::Position(word(4))
            }
            Err(Error::Disconnected) => {
                log::debug!("Device has been disconnected.");
                TriggerPacket::Over
            }
            other => {
                // trigger_receive() has no branch for this: libsigrok's
                // capture never ends. This one does, with the device
                // stopped.
                let why = match other {
                    Ok(p) => format!("{} bytes of {}", p.len(), protocol::TRIGGER_POS_LEN),
                    Err(e) => protocol::libusb_name(&e).to_owned(),
                };
                log::error!("Failed to read the trigger position: {why}.");
                let _ = link.stop();
                TriggerPacket::Over
            }
        }
    }

    /// One sample transfer, completed (`Ok`) or failed, as
    /// `receive_transfer()` takes it.
    fn transfer(&self, transfer: Result<Vec<u8>>, s: &mut State, sink: &PacketSink) -> Next {
        // A timeout with nothing is every queued transfer back empty,
        // each counted as receive_transfer() counts it; any other failure
        // is one failed transfer.
        let (data, count) = match transfer {
            Ok(d) => (d, 1),
            Err(Error::Timeout) => (Vec::new(), self.depth),
            Err(e) => {
                log::debug!("Transfer failed: {e}.");
                (Vec::new(), 1)
            }
        };
        log::debug!("receive_transfer(): received {} bytes.", data.len());
        if data.is_empty() {
            s.empty += count;
            // Past the limit the FX2 gave up.
            return if s.empty > MAX_EMPTY_TRANSFERS { Next::End } else { Next::Continue };
        }
        s.empty = 0;
        let limit = self.limit;
        if limit == 0 || s.sent < limit {
            let atom = protocol::ATOMIC_BYTES * self.mask.count_ones() as usize;
            if !data.len().is_multiple_of(atom) {
                log::error!("Invalid transfer length!");
            }
            let mut samples = Vec::with_capacity(data.len() * 16);
            let count = protocol::deinterleave(&data, self.mask, &mut samples) as u64;
            let mut n = if limit > 0 && s.sent + count > limit { limit - s.sent } else { count };
            let send = |from: u64, n: u64| {
                if n > 0 {
                    let (from, n) = (from as usize * 2, n as usize * 2);
                    sink.send(Packet::logic(2, samples[from..from + n].to_vec()));
                }
            };
            if s.trigger_pos > s.sent && s.trigger_pos <= s.sent + n {
                let offset = s.trigger_pos - s.sent;
                send(0, offset);
                s.sent += offset;
                s.trigger_pos = 0;
                sink.send(Packet::Trigger);
                n -= offset;
                send(offset, n);
                s.sent += n;
            } else {
                send(0, n);
                s.sent += n;
            }
        }
        if limit > 0 && s.sent >= limit { Next::End } else { Next::Continue }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collect(rx: &std::sync::mpsc::Receiver<(usize, Packet)>) -> Vec<Packet> {
        rx.try_iter().map(|(_, p)| p).collect()
    }

    fn acquisition(limit: u64) -> Acquisition {
        Acquisition { mask: 1, limit, size: 512, depth: 1, timeout: Duration::ZERO }
    }

    /// The 16 bit samples of one channel 0 word.
    fn bits(w: u64, n: usize) -> Vec<u8> {
        (0..n).flat_map(|i| [u8::from(w >> i & 1 != 0), 0]).collect()
    }

    #[test]
    fn the_limit_cuts_the_last_transfer() {
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let sink = PacketSink::new(tx, 0);
        let a = acquisition(100);
        let mut s = State { sent: 0, empty: 0, trigger_pos: 0 };
        let w = 0x0123_4567_89ab_cdefu64;
        assert_eq!(a.transfer(Ok(w.to_le_bytes().to_vec()), &mut s, &sink), Next::Continue);
        assert_eq!(a.transfer(Ok(w.to_le_bytes().to_vec()), &mut s, &sink), Next::End);
        assert_eq!(collect(&rx), [Packet::logic(2, bits(w, 64)), Packet::logic(2, bits(w, 36))]);
    }

    #[test]
    fn the_trigger_goes_at_its_sample() {
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let sink = PacketSink::new(tx, 0);
        let a = acquisition(128);
        let mut s = State { sent: 0, empty: 0, trigger_pos: 70 };
        let w = 0xf0f0_f0f0_0f0f_0f0fu64;
        a.transfer(Ok(w.to_le_bytes().to_vec()), &mut s, &sink);
        assert_eq!(a.transfer(Ok(w.to_le_bytes().to_vec()), &mut s, &sink), Next::End);
        let all = bits(w, 64);
        assert_eq!(
            collect(&rx),
            [
                Packet::logic(2, all.clone()),
                Packet::logic(2, all[..12].to_vec()),
                Packet::Trigger,
                Packet::logic(2, all[12..].to_vec()),
            ]
        );
        assert_eq!(s.trigger_pos, 0);
    }

    #[test]
    fn empty_and_failed_transfers() {
        let (tx, _rx) = std::sync::mpsc::sync_channel(64);
        let sink = PacketSink::new(tx, 0);
        let a = acquisition(0);
        let mut s = State { sent: 0, empty: 0, trigger_pos: 0 };
        for _ in 0..MAX_EMPTY_TRANSFERS {
            assert_eq!(a.transfer(Ok(Vec::new()), &mut s, &sink), Next::Continue);
        }
        assert_eq!(a.transfer(Err(Error::device("stall")), &mut s, &sink), Next::End);
        // A queue of 10 timing out: 7 timeouts make 70 empty transfers.
        let mut a = acquisition(0);
        a.depth = 10;
        let mut s = State { sent: 0, empty: 0, trigger_pos: 0 };
        for _ in 0..6 {
            assert_eq!(a.transfer(Err(Error::Timeout), &mut s, &sink), Next::Continue);
        }
        assert_eq!(a.transfer(Err(Error::Timeout), &mut s, &sink), Next::End);
        let a = acquisition(0);
        let mut s = State { sent: 0, empty: 10, trigger_pos: 0 };
        assert_eq!(a.transfer(Ok(vec![0; 8]), &mut s, &sink), Next::Continue);
        assert_eq!(s.empty, 0);
    }

    /// Runs an acquisition of 64 samples of one channel on `steps`,
    /// with the stop flag as given: the packets, and all steps played.
    fn run(steps: &str, stopped: bool) -> Vec<Packet> {
        let text = format!("# oxrok usb fixture 1\ndevice vid=2a0e pid=0020\n{steps}");
        let r = crate::usb::record::Replay::from_text(&text).unwrap();
        let check = r.clone();
        let mut link = Link::new(Box::new(r));
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let sink = PacketSink::new(tx, 0);
        let mut a = acquisition(64);
        a.depth = 2;
        a.run(&mut link, &sink, &AtomicBool::new(stopped));
        check.finish().unwrap();
        collect(&rx)
    }

    const STOP: &str = "ctl_out 40 b2 0000 0000 800000 -> ok\n";

    #[test]
    fn the_trigger_packet_comes_before_the_samples() {
        // real_pos 8, in the packet's second word.
        let steps = format!(
            "bulk_in 86 512 -> 000000000800000000{{504}}\nbulk_in 86 512 -> {}\n",
            "ff".repeat(8)
        );
        let all = bits(u64::MAX, 64);
        let packets = run(&steps, false);
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert_eq!(
            packets[1..],
            [
                Packet::logic(2, all[..16].to_vec()),
                Packet::Trigger,
                Packet::logic(2, all[16..].to_vec()),
            ]
        );
    }

    #[test]
    fn a_bad_trigger_packet_ends_the_capture_with_the_device_stopped() {
        // libsigrok's capture never ends on these (protocol.c:1010-1026).
        let stall = format!("bulk_in 86 512 -> err device \"endpoint stalled\"\n{STOP}");
        assert_eq!(run(&stall, false), []);
        let short = format!("bulk_in 86 512 -> 00{{24}}\n{STOP}");
        assert_eq!(run(&short, false), []);
    }

    #[test]
    fn a_stop_while_waiting_for_the_trigger_sends_the_stop_command() {
        assert_eq!(run(STOP, true), []);
    }

    #[test]
    fn a_trigger_on_the_last_sample_sends_no_empty_packet() {
        // libsigrok sends a logic packet of length 0 here (protocol.c:861-
        // 862), which kills its hex output.
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let sink = PacketSink::new(tx, 0);
        let a = acquisition(64);
        let mut s = State { sent: 0, empty: 0, trigger_pos: 64 };
        assert_eq!(a.transfer(Ok(vec![0xff; 8]), &mut s, &sink), Next::End);
        assert_eq!(collect(&rx), [Packet::logic(2, bits(u64::MAX, 64)), Packet::Trigger]);
    }

    #[test]
    fn a_transfer_cut_inside_an_atom_of_three_channels_has_zeros_for_the_rest() {
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let sink = PacketSink::new(tx, 0);
        let mut a = acquisition(0);
        a.mask = 0b111;
        let mut s = State { sent: 0, empty: 0, trigger_pos: 0 };
        // Four bytes of channel 0 and nothing of 1 and 2: 10 samples
        // (64 * 4 / 24), where libsigrok reads 1 and 2 from the old buffer.
        assert_eq!(a.transfer(Ok(vec![0xff; 4]), &mut s, &sink), Next::Continue);
        let want: Vec<u8> = (0..10).flat_map(|_| [1, 0]).collect();
        assert_eq!(collect(&rx), [Packet::logic(2, want)]);
        assert_eq!(s.sent, 10);
    }

    #[test]
    fn triggers_of_more_than_one_stage_are_refused() {
        let mut d = device(0x0020);
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::One);
        d.set_trigger(Some(t.clone())).unwrap();
        // "A then B": the C driver would make it "A and B" in stage 0.
        t.add(1, 1, TriggerMatch::Zero);
        let e = d.set_trigger(Some(t)).unwrap_err().to_string();
        assert!(e.contains("Only 1 trigger stages"), "{e}");
        assert_eq!(d.trigger.as_ref().map(|t| t.stages.len()), Some(1));
    }

    #[test]
    fn a_reopen_writes_no_default_threshold() {
        let text = "# oxrok usb fixture 1\n\
                    device vid=2a0e pid=0001\n\
                    ctl_out 40 b8 0000 0000 4778 -> ok\n";
        let r = crate::usb::record::Replay::from_text(text).unwrap();
        let check = r.clone();
        let mut link = Link::new(Box::new(r));
        let mut d = device(0x0001);
        d.open_threshold(&mut link).unwrap();
        assert_eq!(d.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(0.7, 1.4));
        // No second write, and the same bitstream as the first open's.
        d.open_threshold(&mut link).unwrap();
        check.finish().unwrap();
        assert_eq!(d.bitstream(), "dreamsourcelab-dslogic-fpga-3v3.fw");
    }

    fn device(pid: u16) -> DslogicDevice {
        let usb = UsbDeviceInfo { vendor_id: VID, product_id: pid, ..UsbDeviceInfo::default() };
        DslogicDevice::new(usb, Profile::of(pid).unwrap(), String::new(), None)
    }

    #[test]
    fn options_follow_the_c_driver() {
        let mut d = device(0x0020);
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(10_000));
        assert!(d.set(&Key::Samplerate, &Value::U64(7_000_000), None).is_err());
        d.set(&Key::Samplerate, &Value::U64(400_000_000), None).unwrap();
        assert!(d.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
        // The clock edge reads back as set (the C driver: always rising).
        d.set(&Key::ClockEdge, &Value::Str("falling".into()), None).unwrap();
        assert_eq!(d.get(&Key::ClockEdge, None).unwrap(), Value::Str("falling".into()));
        assert!(d.set(&Key::ClockEdge, &Value::Str("sideways".into()), None).is_err());
        // Closed: the threshold waits for the open.
        d.set(&Key::VoltageThreshold, &Value::RangeF64(2.5, 3.5), None).unwrap();
        assert_eq!(d.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(3.0, 3.0));
        assert!(d.settings.threshold_pending);
        assert!(d.set(&Key::VoltageThreshold, &Value::RangeF64(6.0, 6.0), None).is_err());
        assert_eq!(d.list(&Key::VoltageThreshold, None).unwrap(), {
            ValueList::RangeF64(protocol::threshold_steps())
        });
        assert!(d.get(&Key::Samplerate, Some("Logic")).is_ok());
        assert!(d.get(&Key::Samplerate, Some("Other")).is_err());
        d.enable_channel(3, false).unwrap();
        assert_eq!(d.channel_mask(), 0xfff7);
    }

    #[test]
    fn the_first_dslogic_has_two_ranges() {
        let mut d = device(0x0001);
        assert_eq!(d.bitstream(), "dreamsourcelab-dslogic-fpga-3v3.fw");
        assert!(d.get(&Key::VoltageThreshold, None).is_err());
        d.set(&Key::VoltageThreshold, &Value::RangeF64(1.4, 3.6), None).unwrap();
        assert_eq!(d.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(1.4, 3.6));
        assert_eq!(d.bitstream(), "dreamsourcelab-dslogic-fpga-5v.fw");
        assert!(d.set(&Key::VoltageThreshold, &Value::RangeF64(1.0, 2.0), None).is_err());
        assert_eq!(d.list(&Key::VoltageThreshold, None).unwrap(), {
            ValueList::RangeF64(THRESHOLDS.to_vec())
        });
    }
}
