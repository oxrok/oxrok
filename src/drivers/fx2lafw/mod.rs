// Counterpart in sigrok:
//   libsigrok/src/hardware/fx2lafw/api.c,
//   libsigrok/src/hardware/fx2lafw/protocol.c,
//   sigrok-firmware-fx2lafw (the command set).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! fx2lafw: logic analyzers built on the Cypress FX2 running the sigrok
//! `fx2lafw` firmware. The FX2 streams its GPIO port over a bulk
//! endpoint at up to 24 MS/s (8 channels) or 12 MS/s (16 channels);
//! triggers run on the host.
//!
//! The firmware's commands: `0xb0` firmware version, `0xb1` start
//! acquisition with a flags byte and a sample delay, `0xb2` chip
//! revision. Devices without the firmware in EEPROM enumerate as the
//! bare chip or the vendor's product; as in libsigrok, the scan reads
//! each device's strings and uploads the firmware over [`crate::ezusb`]
//! to those that lack it, and opening waits for the device to come
//! back. Scan, open, transfer sizes, frames and the soft trigger follow
//! `api.c` and `protocol.c` step by step, so that the device sees the
//! same transactions as under sigrok-cli (checked with the emulator,
//! `tools/emu/fx2lafw.sh`).
//!
//! The `USBee AX` has an analog input, `A0`: an ADC clocked by the CTL2
//! pulses the firmware makes when the start flags ask for them. With
//! `A0` enabled the samples are 16 bit (so 12 MS/s at most), the logic
//! byte then the ADC's, and the driver sends a logic packet of the
//! first and an analog one of the second in volts, `(code - 128) /
//! 12.8`, as `mso_send_data_proc()` does.

#![forbid(unsafe_code)]

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, SoftTriggerLogic,
    Trigger, TriggerMatch, Unit, Value, ValueList,
};
use crate::usb::{BulkStream, ControlRequest, Transport, UsbDeviceInfo, UsbId};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const CMD_GET_FW_VERSION: u8 = 0xb0;
const CMD_START: u8 = 0xb1;
const CMD_GET_REVID_VERSION: u8 = 0xb2;
/// `CMD_START_FLAGS_CLK_CTL2`: the firmware pulses CTL1/CTL2 once per
/// sample, the clock of the `USBee AX`'s ADC.
const FLAG_CLK_CTL2: u8 = 1 << 4;
const FLAG_WIDE: u8 = 1 << 5;
const FLAG_CLK_48MHZ: u8 = 1 << 6;
const REQUIRED_MAJOR: u8 = 1;
const MAX_SAMPLE_DELAY: u64 = 6 * 256;
const MAX_16BIT_RATE: u64 = 12_000_000;
const CONFIGURATION: u8 = 1;
const INTERFACE: u8 = 0;
const ENDPOINT_IN: u8 = 0x82;
const CONTROL_TIMEOUT: Duration = Duration::from_millis(100);
const RENUM_TIMEOUT: Duration = Duration::from_secs(3);
const MAX_TRANSFERS: usize = 32;
/// Consecutive empty or failed transfers after which the FX2 is taken
/// to have given up.
const MAX_EMPTY_TRANSFERS: usize = MAX_TRANSFERS * 2;

/// The sample rates the firmware can produce.
pub const SAMPLERATES: [u64; 17] = [
    20_000, 25_000, 50_000, 100_000, 200_000, 250_000, 500_000, 1_000_000, 2_000_000, 3_000_000,
    4_000_000, 6_000_000, 8_000_000, 12_000_000, 16_000_000, 24_000_000, 48_000_000,
];

/// `channel_names_analog` of `api.c`; the `USBee AX` has the first.
const ANALOG_NAMES: [&str; 4] = ["A0", "A1", "A2", "A3"];
/// The decimal digits of the analog packets, `sr_analog_init(..., 2)`.
const ANALOG_DIGITS: i32 = 2;

/// A known device: its USB id, names, firmware file and channel count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// Vendor and product id.
    pub id: UsbId,
    /// Vendor name.
    pub vendor: &'static str,
    /// Model name.
    pub model: &'static str,
    /// The firmware file in the firmware directories.
    pub firmware: &'static str,
    /// 16 channels instead of 8.
    pub wide: bool,
    /// One analog channel, `A0` (`DEV_CAPS_AX_ANALOG`, the `USBee AX`).
    pub analog: bool,
}

const fn profile(
    vendor_id: u16,
    product_id: u16,
    vendor: &'static str,
    model: &'static str,
    firmware: &'static str,
    wide: bool,
) -> Profile {
    Profile {
        id: UsbId { vendor: vendor_id, product: product_id },
        vendor,
        model,
        firmware,
        wide,
        analog: false,
    }
}

/// The devices this driver serves.
pub static PROFILES: [Profile; 10] = [
    Profile {
        analog: true,
        ..profile(0x08a9, 0x0014, "CWAV", "USBee AX", "fx2lafw-cwav-usbeeax.fw", false)
    },
    profile(0x08a9, 0x0015, "CWAV", "USBee DX", "fx2lafw-cwav-usbeedx.fw", true),
    profile(0x08a9, 0x0009, "CWAV", "USBee SX", "fx2lafw-cwav-usbeesx.fw", false),
    profile(0x08a9, 0x0005, "CWAV", "USBee ZX", "fx2lafw-cwav-usbeezx.fw", false),
    profile(0x0925, 0x3881, "Saleae", "Logic", "fx2lafw-saleae-logic.fw", false),
    profile(0x04b4, 0x8613, "Cypress", "FX2", "fx2lafw-cypress-fx2.fw", true),
    profile(
        0x16d0,
        0x0498,
        "Braintechnology",
        "USB-LPS",
        "fx2lafw-braintechnology-usb-lps.fw",
        true,
    ),
    profile(0x1d50, 0x608c, "sigrok", "FX2 LA (8ch)", "fx2lafw-sigrok-fx2-8ch.fw", false),
    profile(0x1d50, 0x608d, "sigrok", "FX2 LA (16ch)", "fx2lafw-sigrok-fx2-16ch.fw", true),
    profile(0x1d50, 0x608f, "sigrok", "usb-c-grok", "fx2lafw-usb-c-grok.fw", false),
];

fn find_profile(id: UsbId) -> Option<&'static Profile> {
    PROFILES.iter().find(|p| p.id == id)
}

/// True when the device's strings say the firmware is running.
pub fn has_firmware(info: &UsbDeviceInfo) -> bool {
    info.manufacturer.as_deref() == Some("sigrok") && info.product.as_deref() == Some("fx2lafw")
}

/// The `0xb1` command for `samplerate`: flags byte and the GPIF sample
/// delay, or an error for a rate the clocks cannot divide to. `wide`
/// asks for 16 bit samples, `analog` for the CTL2 clock of the ADC.
pub fn start_command(samplerate: u64, wide: bool, analog: bool) -> Result<[u8; 3]> {
    let mut flags = 0u8;
    let mut delay = 0u64;
    if samplerate > 0 && 48_000_000 % samplerate == 0 {
        flags = FLAG_CLK_48MHZ;
        delay = 48_000_000 / samplerate - 1;
        if delay > MAX_SAMPLE_DELAY {
            delay = 0;
        }
    }
    if delay == 0 && samplerate > 0 && 30_000_000 % samplerate == 0 && samplerate != 48_000_000 {
        flags = 0;
        delay = 30_000_000 / samplerate - 1;
    }
    let ok = samplerate == 48_000_000 || delay > 0;
    if !ok || delay > MAX_SAMPLE_DELAY {
        return Err(Error::Arg(format!("Unable to sample at {samplerate} Hz.")));
    }
    if wide {
        flags |= FLAG_WIDE;
    }
    if analog {
        flags |= FLAG_CLK_CTL2;
    }
    Ok([flags, (delay >> 8) as u8, (delay & 0xff) as u8])
}

/// An ADC code of the `USBee AX` in volts, as `mso_send_data_proc()`
/// rescales it: 0 to 255 is -10 V to +9.92 V, 128 is 0 V.
pub fn adc_volts(code: u8) -> f32 {
    (f32::from(code) - 128.0) / 12.8
}

/// The 16 bit samples of a `USBee AX` with its analog channel enabled,
/// split as `mso_send_data_proc()` does: the low byte of each is the
/// logic sample (D0 to D7), the high byte the ADC's, in volts. A last
/// odd byte is dropped.
pub fn deinterleave(data: &[u8]) -> (Vec<u8>, Vec<f32>) {
    data.as_chunks::<2>().0.iter().map(|[logic, adc]| (*logic, adc_volts(*adc))).unzip()
}

/// Transfer size, queue depth and the timeout of one transfer for a
/// rate, as `get_buffer_size()`, `get_number_of_transfers()` and
/// `get_timeout()` compute them: 10 ms of data rounded up to a multiple
/// of 512, about 500 ms in flight but at most 32 transfers, and the
/// time those take plus a quarter. Like the C code they count one byte
/// per sample whatever the sample width.
pub fn transfer_plan(samplerate: u64) -> (usize, usize, Duration) {
    let bytes_per_ms = (samplerate / 1000) as usize;
    let size = (10 * bytes_per_ms + 511) & !511;
    let depth = (500 * bytes_per_ms / size.max(1)).min(MAX_TRANSFERS);
    let total = size * depth / bytes_per_ms.max(1);
    let ms = total + total / 4;
    (size, depth, Duration::from_millis(ms as u64))
}

/// The driver.
pub struct Fx2lafwDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::ProbeNames];
static DEV_OPTIONS: [OptionDesc; 7] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
];
/// libsigrok's soft trigger: stages of level and edge matches only.
const TRIGGER_FEATURES: crate::core::TriggerFeatures = crate::core::TriggerFeatures {
    count: false,
    consecutive: false,
    duration: false,
    range: false,
    or: false,
    and: false,
    max_stages: 0,
};
static TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];

impl Driver for Fx2lafwDriver {
    fn id(&self) -> &'static str {
        "fx2lafw"
    }

    fn long_name(&self) -> &'static str {
        "fx2lafw (generic driver for FX2 based LAs)"
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
        let ids: Vec<UsbId> = PROFILES.iter().map(|p| p.id).collect();
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for mut info in crate::usb::find_devices(&ids)? {
            if conn.as_deref().is_some_and(|c| !info.matches_conn(c)) {
                continue;
            }
            let profile = find_profile(info.id()).expect("found by id");
            // As scan() in api.c: the strings read from the device, the
            // firmware recognised by them, else uploaded right away.
            let Some(strings) = read_strings(&info) else { continue };
            info.serial = Some(strings[2].clone());
            let has_firmware = match_manuf_prod(&info);
            let mut fw_updated = None;
            if has_firmware {
                log::debug!("Found an fx2lafw device.");
            } else {
                match upload(&info, profile) {
                    // Stored only when the upload worked, as api.c does;
                    // otherwise open tries the device once right away.
                    Ok(()) => fw_updated = Some(Instant::now()),
                    Err(e) => log::error!(
                        "Firmware upload failed for device {}, name {}: {e}",
                        info.conn(),
                        profile.firmware
                    ),
                }
            }
            let mut dev = Fx2lafwDevice::new(info, profile, None);
            if !has_firmware {
                // The address it will have after the upload is not
                // known (address 0xff in the C driver).
                dev.info.conn = None;
            }
            dev.fw_updated = fw_updated;
            if let Some(spec) = &probe_names {
                dev.name_channels(spec);
            }
            devices.push(Box::new(dev));
        }
        Ok(devices)
    }
}

/// The manufacturer, product and serial number strings, read from the
/// device as `scan()` does (an empty string for index 0), or `None`
/// when the device cannot be opened or a read fails.
fn read_strings(info: &UsbDeviceInfo) -> Option<[String; 3]> {
    let mut t = match crate::usb::open(info) {
        Ok(t) => t,
        Err(e) => {
            log::warn!(
                "Failed to open potential device with VID:PID {:04x}:{:04x}: {e}.",
                info.vendor_id,
                info.product_id
            );
            return None;
        }
    };
    let indexes = t.string_indexes().ok()?;
    let mut out: [String; 3] = Default::default();
    for ((o, i), what) in
        out.iter_mut().zip(indexes).zip(["manufacturer", "product", "serial number"])
    {
        if i != 0 {
            match crate::usb::read_string_ascii(t.as_mut(), i) {
                Ok(v) => *o = v,
                Err(e) => {
                    log::warn!("Failed to get {what} string descriptor: {e}.");
                    return None;
                }
            }
        }
    }
    Some(out)
}

/// `usb_match_manuf_prod(dev, "sigrok", "fx2lafw")`: the device is opened
/// again and its manufacturer and product strings read again (none
/// when an index is 0, which libusb refuses without a transfer).
fn match_manuf_prod(info: &UsbDeviceInfo) -> bool {
    let Ok(mut t) = crate::usb::open(info) else { return false };
    let Ok([m, p, _]) = t.string_indexes() else { return false };
    crate::usb::read_string_ascii(t.as_mut(), m).is_ok_and(|s| s == "sigrok")
        && crate::usb::read_string_ascii(t.as_mut(), p).is_ok_and(|s| s == "fx2lafw")
}

/// `ezusb_upload_firmware()`: open, `SET_CONFIGURATION` 1, the image
/// in chunks of 4096 bytes with the 8051 held in reset, release.
fn upload(info: &UsbDeviceInfo, profile: &Profile) -> Result<()> {
    let image = crate::core::firmware::read(
        profile.firmware,
        "install the sigrok fx2lafw firmware, see docs/DEVICES.md",
    )?;
    log::info!("Uploading firmware {} to device on {}.", profile.firmware, info.conn());
    let mut t = crate::usb::open(info)?;
    t.force_configuration(CONFIGURATION)?;
    crate::ezusb::upload_firmware(t.as_mut(), &image)
}

/// Settings kept between acquisitions.
#[derive(Clone, Debug)]
struct Settings {
    samplerate: u64,
    limit_samples: u64,
    limit_frames: u64,
    capture_ratio: u64,
}

/// One FX2 based analyzer.
pub struct Fx2lafwDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    profile: &'static Profile,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    settings: Settings,
    trigger: Option<Trigger>,
    pending: Option<Box<dyn Transport>>,
    transport: Arc<Mutex<Option<Box<dyn Transport>>>>,
    worker: Option<Worker>,
    /// When the scan uploaded the firmware: the device renumerates and
    /// is looked for again on open.
    fw_updated: Option<Instant>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl Fx2lafwDevice {
    fn new(
        usb: UsbDeviceInfo,
        profile: &'static Profile,
        transport: Option<Box<dyn Transport>>,
    ) -> Self {
        let count = if profile.wide { 16 } else { 8 };
        let mut dev = Fx2lafwDevice {
            info: DeviceInfo {
                driver: "fx2lafw",
                vendor: profile.vendor.into(),
                model: profile.model.into(),
                version: String::new(),
                serial: usb.serial.clone().unwrap_or_default(),
                conn: Some(usb.conn()),
            },
            usb,
            profile,
            channels: (0..count).map(Channel::logic).collect(),
            groups: Vec::new(),
            settings: Settings {
                samplerate: SAMPLERATES[0],
                limit_samples: 0,
                limit_frames: 1,
                capture_ratio: 0,
            },
            trigger: None,
            pending: transport,
            transport: Arc::new(Mutex::new(None)),
            worker: None,
            fw_updated: None,
        };
        dev.add_analog_and_groups();
        dev
    }

    /// After the logic channels: the analog one of a `USBee AX`, numbered
    /// after the 8 logic channels the profile has (whatever `probe_names`
    /// left), as `scan()` does; then the groups, `Logic` and one per
    /// analog channel, named as the channel.
    fn add_analog_and_groups(&mut self) {
        let logic = self.channels.len();
        if self.profile.analog {
            let index = if self.profile.wide { 16 } else { 8 };
            self.channels.push(Channel::new(index, ChannelKind::Analog, ANALOG_NAMES[0]));
        }
        self.groups = vec![ChannelGroup::new("Logic", (0..logic).collect())];
        for pos in logic..self.channels.len() {
            self.groups.push(ChannelGroup::new(self.channels[pos].name.clone(), vec![pos]));
        }
    }

    /// A device on an already open transport (a replayed recording).
    /// The firmware is assumed to be running; the profile comes from
    /// the transport's USB id.
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        let profile =
            find_profile(usb.id()).ok_or_else(|| Error::arg("not a known fx2lafw device"))?;
        Ok(Self::new(usb, profile, Some(transport)))
    }

    /// Applies a `probe_names` spec (see
    /// `crate::core::util::parse_probe_names`); a spec starting with `-`
    /// leaves only the named channels.
    /// Only the logic channels take the names, as in `scan()`, which
    /// leaves the analog channel `A0`.
    pub fn name_channels(&mut self, spec: &str) {
        self.channels.retain(|c| c.kind == ChannelKind::Logic);
        let defaults: Vec<&str> = self.channels.iter().map(|c| c.name.as_str()).collect();
        let names = crate::core::util::parse_probe_names(spec, &defaults);
        self.channels.truncate(names.len());
        for (c, n) in self.channels.iter_mut().zip(names) {
            c.name = n;
        }
        self.add_analog_and_groups();
    }

    /// The indexes of the enabled analog channels.
    fn enabled_analog(&self) -> Vec<usize> {
        self.channels
            .iter()
            .filter(|c| c.enabled && c.kind == ChannelKind::Analog)
            .map(|c| c.index)
            .collect()
    }

    /// 16 bit samples, as `configure_channels()` decides: a channel
    /// above D7 enabled on a 16 channel device, or an analog channel
    /// enabled (its ADC is the high byte).
    fn wide(&self) -> bool {
        let high_logic = self.profile.wide
            && self
                .channels
                .iter()
                .any(|c| c.enabled && c.kind == ChannelKind::Logic && c.index >= 8);
        high_logic || !self.enabled_analog().is_empty()
    }
}

/// Whether `d` is the device scanned as `usb`: by its hub ports, which
/// stay the same when it renumerates, as `fx2lafw_dev_open()` compares
/// `usb_get_port_path()` with the connection id. A backend that knows
/// no ports falls back to the address, or after an upload to a device
/// on the same bus whose strings say fx2lafw.
fn same_device(d: &UsbDeviceInfo, usb: &UsbDeviceInfo, after_upload: bool) -> bool {
    if d.bus != usb.bus {
        return false;
    }
    if !usb.ports.is_empty() && !d.ports.is_empty() {
        return d.ports == usb.ports;
    }
    if after_upload { has_firmware(d) } else { d.address == usb.address }
}

/// `fx2lafw_dev_open()`: finds the device of the profile's id at the
/// scanned device's port, opens it and sets it up ([`set_up`]).
fn try_open(
    usb: &UsbDeviceInfo,
    after_upload: bool,
) -> Result<(UsbDeviceInfo, Box<dyn Transport>)> {
    let found = crate::usb::find_devices(&[usb.id()])?
        .into_iter()
        .find(|d| same_device(d, usb, after_upload));
    let info = found.ok_or_else(|| Error::device("device not found"))?;
    let mut t = crate::usb::open(&info)?;
    set_up(t.as_mut())?;
    Ok((info, t))
}

/// The checks of `fx2lafw_dev_open()`, then the claim of `dev_open()`:
/// the firmware version and REVID are read before the interface is
/// claimed, so a device another program holds sees the two reads, as
/// under the C driver. `set_configuration(1)` comes first; it sends
/// nothing when the configuration is active (always on Linux) but is
/// needed on macOS, which can leave a device unconfigured right after
/// renumeration. The same sequence on every OS: on Windows, where
/// `WinUSB` needs a claimed interface for any control transfer, the
/// nusb backend claims it for the first read.
fn set_up(t: &mut dyn Transport) -> Result<()> {
    t.set_configuration(CONFIGURATION)?;
    check_firmware(t)?;
    t.claim_interface(INTERFACE).map_err(|e| match e {
        Error::Busy => Error::device(
            "Unable to claim USB interface. Another program or driver has already claimed it",
        ),
        e => e,
    })
}

/// Reads the firmware version and chip revision; fails on a version
/// the driver does not speak.
fn check_firmware(t: &mut dyn Transport) -> Result<(u8, u8)> {
    let v = t.control_in(ControlRequest::vendor(CMD_GET_FW_VERSION, 0, 0), 2, CONTROL_TIMEOUT)?;
    if v.len() < 2 {
        return Err(Error::device("short firmware version"));
    }
    // A failed REVID read fails the open, as in protocol.c, which reads
    // it before it looks at the version.
    let r =
        t.control_in(ControlRequest::vendor(CMD_GET_REVID_VERSION, 0, 0), 1, CONTROL_TIMEOUT)?;
    let revid = r.first().ok_or_else(|| Error::device("Failed to get REVID."))?;
    if v[0] != REQUIRED_MAJOR {
        // protocol.c logs this and means to bail out, but its ret
        // still holds the REVID read's SR_OK, so libsigrok opens the
        // device anyway.
        return Err(Error::device(format!(
            "expected firmware version {REQUIRED_MAJOR}.x, got {}.{}",
            v[0], v[1]
        )));
    }
    log::debug!(
        "Firmware {}.{}, REVID {revid}: a Cypress CY7C68013{}.",
        v[0],
        v[1],
        if *revid == 1 { "A (FX2LP)" } else { " (FX2)" }
    );
    Ok((v[0], v[1]))
}

impl Device for Fx2lafwDevice {
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
            // config_list() answers SR_ERR_NA for any channel group.
            Some(g) if self.groups.iter().any(|cg| cg.name.eq_ignore_ascii_case(g)) => {
                Ok(Vec::new())
            }
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            // Unknown until the device came back after the upload.
            Key::Conn => match &self.info.conn {
                Some(c) => Value::Str(c.clone()),
                None => return Err(Error::device("the device has not renumerated yet")),
            },
            Key::Samplerate => Value::U64(self.settings.samplerate),
            Key::LimitSamples => Value::U64(self.settings.limit_samples),
            Key::LimitFrames => Value::U64(self.settings.limit_frames),
            Key::CaptureRatio => Value::U64(self.settings.capture_ratio),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::Samplerate => {
                let r = value.as_u64().ok_or_else(bad)?;
                if !SAMPLERATES.contains(&r) {
                    return Err(Error::Arg(format!("unsupported sample rate {r}")));
                }
                self.settings.samplerate = r;
            }
            Key::LimitSamples => self.settings.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::LimitFrames => self.settings.limit_frames = value.as_u64().ok_or_else(bad)?,
            Key::CaptureRatio => {
                let r = value.as_u64().ok_or_else(bad)?;
                if r > 100 {
                    return Err(Error::arg("capture ratio is a percentage"));
                }
                self.settings.capture_ratio = r;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::Samplerate => ValueList::U64(SAMPLERATES.to_vec()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.transport.lock().expect("transport lock").is_some() {
            return Ok(());
        }
        let t = if let Some(mut t) = self.pending.take() {
            set_up(t.as_mut())?;
            t
        } else if let Some(uploaded) = self.fw_updated {
            // dev_open(): wait for the FX2 to renumerate, trying every
            // 100 ms until 3 s after the upload. The retries also cover
            // a claim that fails while the OS still configures the new
            // device.
            log::info!("Waiting for device to reset.");
            std::thread::sleep(Duration::from_millis(300));
            let (info, t) = loop {
                match try_open(&self.usb, true) {
                    Ok(found) => break found,
                    Err(e) => {
                        std::thread::sleep(Duration::from_millis(100));
                        if uploaded.elapsed() >= RENUM_TIMEOUT {
                            log::debug!("Last attempt: {e}.");
                            return Err(Error::device("Device failed to renumerate."));
                        }
                    }
                }
            };
            log::info!("Device came back after {} ms.", uploaded.elapsed().as_millis());
            self.fw_updated = None;
            self.info.conn = Some(info.conn());
            self.usb = info;
            t
        } else {
            log::info!("Firmware upload was not needed.");
            let (info, t) = try_open(&self.usb, false)?;
            self.info.conn = Some(info.conn());
            self.usb = info;
            t
        };
        *self.transport.lock().expect("transport lock") = Some(t);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        drop(self.transport.lock().expect("transport lock").take());
        Ok(())
    }

    fn trigger_features(&self) -> crate::core::TriggerFeatures {
        TRIGGER_FEATURES
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&TRIGGER_FEATURES)?;
            // The soft trigger matches logic bits. libsigrok's takes an
            // analog channel's index for a bit as well (bit 0 of the
            // ADC's byte on the USBee AX); there is no analog trigger.
            for c in t.stages.iter().flat_map(|s| &s.conditions) {
                if let Some(ch) = self
                    .channels
                    .iter()
                    .find(|ch| ch.index == c.channel && ch.kind == ChannelKind::Analog)
                {
                    return Err(Error::Arg(format!(
                        "{}: fx2lafw triggers on logic channels only",
                        ch.name
                    )));
                }
            }
        }
        self.trigger = trigger;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let wide = self.wide();
        let analog = self.enabled_analog();
        let rate = self.settings.samplerate;
        // command_start_acquisition() refuses it after the transfers
        // were submitted; they are cancelled before any completes, so
        // refusing it first sends the device the same.
        if wide && rate > MAX_16BIT_RATE {
            return Err(Error::Arg(format!(
                "Unable to sample at {rate}Hz when collecting 16-bit samples."
            )));
        }
        let command = start_command(rate, wide, !analog.is_empty())?;
        let unit_size = if wide { 2 } else { 1 };
        // soft_trigger_logic_check() skips matches on disabled channels.
        // The trigger runs on the stream's samples, two bytes with the
        // ADC's high byte; the logic channels are the low byte.
        let stl = self.trigger.as_ref().map(|t| {
            let stages = t
                .stages
                .iter()
                .map(|st| {
                    st.conditions
                        .iter()
                        .filter(|c| {
                            self.channels.iter().any(|ch| ch.index == c.channel && ch.enabled)
                        })
                        .map(|c| (c.channel, c.matches))
                        .collect()
                })
                .collect();
            let pre = self.settings.capture_ratio * self.settings.limit_samples / 100;
            SoftTriggerLogic::new(stages, unit_size, pre as usize)
        });
        let acq = Acquisition {
            command,
            samplerate: rate,
            unit_size,
            analog,
            limit_samples: self.settings.limit_samples,
            limit_frames: self.settings.limit_frames,
            stl,
        };
        let transport = self.transport.lock().expect("transport lock").take();
        let Some(mut transport) = transport else {
            return Err(Error::device("device is not open"));
        };
        // The transfers and the start command before start() returns,
        // as fx2lafw_start_acquisition(): a device that refuses the
        // start fails the session's start.
        let stream = match acq.begin(transport.as_mut()) {
            Ok(stream) => stream,
            Err(e) => {
                *self.transport.lock().expect("transport lock") = Some(transport);
                return Err(e);
            }
        };
        let slot = Arc::clone(&self.transport);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("fx2lafw acquisition".into())
            .spawn(move || {
                if let Err(e) = acq.run(stream, &sink, &stop2) {
                    log::error!("Acquisition failed: {e}.");
                }
                sink.send(Packet::End);
                *slot.lock().expect("transport lock") = Some(transport);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        Ok(())
    }
}

/// One acquisition, on its own thread: `start_transfers()` and
/// `receive_transfer()` of `protocol.c`.
struct Acquisition {
    command: [u8; 3],
    samplerate: u64,
    unit_size: usize,
    /// The enabled analog channels: their samples are the high byte.
    analog: Vec<usize>,
    limit_samples: u64,
    limit_frames: u64,
    stl: Option<SoftTriggerLogic>,
}

impl Acquisition {
    /// Samples of the stream to the session: `la_send_data_proc()` as
    /// they are, or with an analog channel `mso_send_data_proc()`, a
    /// logic packet of the low bytes and an analog one of the ADC's.
    fn send(&self, sink: &PacketSink, data: Vec<u8>) {
        if self.analog.is_empty() {
            sink.send(Packet::logic(self.unit_size, data));
            return;
        }
        let (logic, volts) = deinterleave(&data);
        sink.send(Packet::logic(1, logic));
        sink.send(Packet::Analog(AnalogData {
            channels: self.analog.clone(),
            samples: volts,
            mq: Mq::Voltage,
            flags: MqFlags::default(),
            unit: Unit::Volt,
            digits: ANALOG_DIGITS,
        }));
    }

    /// `start_transfers()` and `command_start_acquisition()`: the
    /// transfers queued, then the start command. On a failure the
    /// queued transfers are cancelled as the stream drops.
    fn begin(&self, t: &mut dyn Transport) -> Result<Box<dyn BulkStream>> {
        let (size, depth, _) = transfer_plan(self.samplerate);
        let stream = t.bulk_in_stream(ENDPOINT_IN, size, depth)?;
        t.control_out(ControlRequest::vendor(CMD_START, 0, 0), &self.command, CONTROL_TIMEOUT)
            .inspect_err(|e| log::error!("Unable to send start command: {e}."))?;
        log::debug!("Acquisition started: {:02x?}, {depth} transfers of {size}.", self.command);
        Ok(stream)
    }

    fn run(
        mut self,
        mut stream: Box<dyn BulkStream>,
        sink: &PacketSink,
        stop: &AtomicBool,
    ) -> Result<()> {
        let (_, _, timeout) = transfer_plan(self.samplerate);
        let u = self.unit_size;
        let limit = self.limit_samples;
        let mut trigger_fired = self.stl.is_none();
        sink.send(Packet::header());
        if trigger_fired {
            sink.send(Packet::FrameBegin);
        }
        let mut sent = 0u64;
        let mut frames = 0u64;
        let mut empty = 0usize;
        while !stop.load(Ordering::SeqCst) {
            // receive_transfer(): a transfer that failed (a STALL, an
            // error) counts as an empty one and is resubmitted; only a
            // device that is gone ends the acquisition at once.
            let data = match stream.next(timeout) {
                Ok(d) => d,
                Err(Error::Timeout) => Vec::new(),
                Err(Error::Disconnected) => return Err(Error::Disconnected),
                Err(e) => {
                    log::debug!("Transfer failed: {e}.");
                    Vec::new()
                }
            };
            if data.is_empty() {
                empty += 1;
                if empty > MAX_EMPTY_TRANSFERS {
                    // The FX2 gave up.
                    log::warn!("Device stopped sending data.");
                    return Ok(());
                }
                continue;
            }
            empty = 0;
            let cur = (data.len() / u) as u64;
            let mut processed = 0u64;
            let mut frame_ended;
            let mut final_frame;
            loop {
                if !trigger_fired && self.stl.is_none() {
                    // A frame without trigger ended exactly at the end
                    // of the previous transfer: the next one begins
                    // here. The C code passes its NULL soft trigger to
                    // soft_trigger_logic_check() instead and crashes.
                    sink.send(Packet::FrameBegin);
                    trigger_fired = true;
                }
                if trigger_fired {
                    if limit == 0 || sent < limit {
                        let mut n = cur - processed;
                        if limit > 0 && sent + n > limit {
                            n = limit - sent;
                        }
                        let at = (processed as usize) * u;
                        self.send(sink, data[at..at + n as usize * u].to_vec());
                        sent += n;
                        processed += n;
                    }
                } else if let Some(stl) = self.stl.as_mut() {
                    let at = processed as usize * u;
                    if let Some((offset, pre)) = stl.check(&data[at..]) {
                        let mut pre_samples = 0u64;
                        for p in pre {
                            pre_samples += (p.len() / u) as u64;
                            self.send(sink, p);
                        }
                        sink.send(Packet::Trigger);
                        sink.send(Packet::FrameBegin);
                        sent += pre_samples;
                        let offset = offset as u64;
                        let mut n = cur - processed - offset;
                        if limit > 0 && sent + n > limit {
                            n = limit - sent;
                        }
                        let from = (processed + offset) as usize * u;
                        self.send(sink, data[from..from + n as usize * u].to_vec());
                        sent += n;
                        processed += offset + n;
                        trigger_fired = true;
                    }
                }
                frame_ended = limit > 0 && sent >= limit;
                final_frame = self.limit_frames > 0 && frames + 1 >= self.limit_frames;
                if frame_ended {
                    frames += 1;
                    sent = 0;
                    trigger_fired = false;
                    sink.send(Packet::FrameEnd);
                    // Another trigger may be in the rest of the data.
                    if processed < cur {
                        if let Some(stl) = self.stl.as_mut() {
                            stl.cur_stage = 0;
                        } else {
                            sink.send(Packet::FrameBegin);
                            trigger_fired = true;
                        }
                        if !final_frame {
                            continue;
                        }
                    }
                }
                break;
            }
            if frame_ended && final_frame {
                return Ok(());
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_commands_match_the_firmware_clocks() {
        assert_eq!(start_command(24_000_000, false, false).unwrap(), [FLAG_CLK_48MHZ, 0, 1]);
        assert_eq!(start_command(48_000_000, false, false).unwrap(), [FLAG_CLK_48MHZ, 0, 0]);
        assert_eq!(
            start_command(12_000_000, true, false).unwrap(),
            [FLAG_CLK_48MHZ | FLAG_WIDE, 0, 3]
        );
        assert_eq!(start_command(3_000_000, false, false).unwrap(), [FLAG_CLK_48MHZ, 0, 15]);
        // 48 MHz / 20 kHz exceeds the delay range; 30 MHz / 20 kHz fits.
        assert_eq!(start_command(20_000, false, false).unwrap(), [0, 0x05, 0xdb]);
        assert_eq!(start_command(25_000, false, false).unwrap(), [0, 0x04, 0xaf]);
        assert_eq!(start_command(50_000, false, false).unwrap(), [FLAG_CLK_48MHZ, 0x03, 0xbf]);
        assert!(start_command(7_000_000, false, false).is_err());
        assert!(start_command(0, false, false).is_err());
        // The USBee AX with A0: 16 bit samples and the CTL2 clock.
        assert_eq!(
            start_command(1_000_000, true, true).unwrap(),
            [FLAG_CLK_48MHZ | FLAG_WIDE | FLAG_CLK_CTL2, 0, 47]
        );
        assert_eq!(start_command(20_000, true, true).unwrap(), [0x30, 0x05, 0xdb]);
    }

    #[test]
    fn adc_codes_scale_to_plus_minus_ten_volts() {
        // (code - 128.0f) / 12.8f in single precision, as protocol.c.
        assert_eq!(adc_volts(0), -10.0);
        assert_eq!(adc_volts(128), 0.0);
        assert_eq!(adc_volts(192), 5.0);
        assert_eq!(adc_volts(64), -5.0);
        assert_eq!(adc_volts(255), 9.921_875);
        assert_eq!(adc_volts(129), 0.078_125);
        for c in 0..=255u8 {
            let v = adc_volts(c);
            assert!((-10.0..10.0).contains(&v), "{c}: {v}");
            assert!((v * 12.8 + 128.0 - f32::from(c)).abs() < 1e-3, "{c}: {v}");
        }
    }

    #[test]
    fn deinterleave_takes_the_low_byte_as_logic_and_the_high_as_the_adc() {
        let (logic, volts) = deinterleave(&[0x01, 128, 0xfe, 192, 0x80, 0, 0x55]);
        assert_eq!(logic, [0x01, 0xfe, 0x80], "the odd last byte is dropped");
        assert_eq!(volts, [0.0, 5.0, -10.0]);
        assert_eq!(deinterleave(&[]), (vec![], vec![]));
    }

    fn device(model: &str) -> Fx2lafwDevice {
        let p = PROFILES.iter().find(|p| p.model == model).unwrap();
        Fx2lafwDevice::new(UsbDeviceInfo::default(), p, None)
    }

    #[test]
    fn the_usbee_ax_lists_a0_in_its_own_group() {
        let mut d = device("USBee AX");
        let names: Vec<&str> = d.channels.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "A0"]);
        assert_eq!(d.channels[8], Channel::new(8, ChannelKind::Analog, "A0"));
        assert_eq!(
            d.groups,
            [ChannelGroup::new("Logic", (0..8).collect()), ChannelGroup::new("A0", vec![8])]
        );
        assert_eq!(d.options(Some("A0")).unwrap(), []);
        assert!(d.options(Some("A1")).is_err());
        // A0 makes the samples wide; the logic channels alone do not.
        assert!(d.wide());
        assert_eq!(d.enabled_analog(), [8]);
        d.enable_channel(8, false).unwrap();
        assert!(!d.wide());
        assert_eq!(d.enabled_analog(), [] as [usize; 0]);
        // probe_names renames the logic channels and leaves A0.
        d.name_channels("-CLK,DATA");
        let names: Vec<&str> = d.channels.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["CLK", "DATA", "A0"]);
        assert_eq!(d.channels[2].index, 8);
        assert_eq!(d.groups[1], ChannelGroup::new("A0", vec![2]));
        // No trigger on A0; on D0 as ever.
        let mut d = device("USBee AX");
        let t = crate::core::trigger_syntax::parse(&d, "A0=r").unwrap();
        assert!(d.set_trigger(Some(t)).unwrap_err().to_string().contains("A0"));
        let t = crate::core::trigger_syntax::parse(&d, "D0=r").unwrap();
        d.set_trigger(Some(t)).unwrap();
        // Other boards have no analog channel.
        let d = device("Logic");
        assert_eq!(d.channels.len(), 8);
        assert_eq!(d.groups.len(), 1);
    }

    /// The transfers of a test, then the stop flag.
    struct Transfers(Vec<Vec<u8>>, Arc<AtomicBool>);

    impl BulkStream for Transfers {
        fn next(&mut self, _timeout: Duration) -> Result<Vec<u8>> {
            if self.0.is_empty() {
                self.1.store(true, Ordering::SeqCst);
                return Err(Error::Timeout);
            }
            Ok(self.0.remove(0))
        }
    }

    fn run(acq: Acquisition, transfers: Vec<Vec<u8>>) -> Vec<Packet> {
        let (tx, rx) = std::sync::mpsc::sync_channel(1024);
        let stop = Arc::new(AtomicBool::new(false));
        let stream = Box::new(Transfers(transfers, Arc::clone(&stop)));
        acq.run(stream, &PacketSink::new(tx, 0), &stop).unwrap();
        rx.try_iter().map(|(_, p)| p).filter(|p| !matches!(p, Packet::Header { .. })).collect()
    }

    fn ax_acquisition(limit: u64, stl: Option<SoftTriggerLogic>) -> Acquisition {
        Acquisition {
            command: [0x70, 0, 47],
            samplerate: 1_000_000,
            unit_size: 2,
            analog: vec![8],
            limit_samples: limit,
            limit_frames: 1,
            stl,
        }
    }

    fn analog(samples: Vec<f32>) -> Packet {
        Packet::Analog(AnalogData {
            channels: vec![8],
            samples,
            mq: Mq::Voltage,
            flags: MqFlags::default(),
            unit: Unit::Volt,
            digits: 2,
        })
    }

    #[test]
    fn an_analog_capture_sends_logic_then_volts_per_transfer() {
        let packets = run(ax_acquisition(3, None), vec![vec![1, 128, 2, 192], vec![3, 0, 4, 255]]);
        assert_eq!(
            packets,
            [
                Packet::FrameBegin,
                Packet::logic(1, vec![1, 2]),
                analog(vec![0.0, 5.0]),
                Packet::logic(1, vec![3]),
                analog(vec![-10.0]),
                Packet::FrameEnd,
                // The rest of the transfer: a frame begins, as in protocol.c.
                Packet::FrameBegin,
            ]
        );
    }

    #[test]
    fn a_trigger_with_a0_matches_the_logic_byte_and_splits_the_pre_trigger_samples() {
        // D1 rising, one sample before it: the trigger looks at the low
        // byte of each 16 bit sample only, whatever the ADC reads.
        let trigger = vec![vec![(1, TriggerMatch::Rising)]];
        let stl = SoftTriggerLogic::new(trigger, 2, 1);
        let data = vec![0x00, 0xff, 0x01, 0xff, 0x02, 128, 0x03, 192, 0x04, 0];
        let packets = run(ax_acquisition(3, Some(stl)), vec![data]);
        assert_eq!(
            packets,
            [
                Packet::logic(1, vec![0x01]),
                analog(vec![adc_volts(0xff)]),
                Packet::Trigger,
                Packet::FrameBegin,
                Packet::logic(1, vec![0x02, 0x03]),
                analog(vec![0.0, 5.0]),
                Packet::FrameEnd,
            ]
        );
    }

    #[test]
    fn transfer_plans() {
        assert_eq!(transfer_plan(24_000_000).0, 240_128);
        assert_eq!(transfer_plan(24_000_000).1, 32);
        assert_eq!(transfer_plan(20_000), (512, 19, Duration::from_millis(486 + 121)));
        // 16 bit samples take the same: the C code counts bytes as samples.
        assert_eq!(transfer_plan(1_000_000), (10_240, 32, Duration::from_millis(327 + 81)));
    }

    #[test]
    fn profiles_and_firmware_detection() {
        assert_eq!(find_profile(UsbId { vendor: 0x0925, product: 0x3881 }).unwrap().model, "Logic");
        let mut d = UsbDeviceInfo { manufacturer: Some("sigrok".into()), ..Default::default() };
        assert!(!has_firmware(&d));
        d.product = Some("fx2lafw".into());
        assert!(has_firmware(&d));
    }
}
