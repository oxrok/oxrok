// Counterpart in sigrok:
//   libsigrok/src/hardware/wch-ch32h417-libusb/api.c,
//   libsigrok/src/hardware/wch-ch32h417-libusb/protocol.c (the acquisition),
//   libsigrok/src/hardware/wch-ch32h417-libusb/libusb_wrapper.c (the scan and the open),
//   Copyright (C) 2026 WeAct Studio,
//   with commits by oxrok-bot[bot]
//   (libsigrok libsigrok-unreleased-1859-g4ed8c99c).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! wch-ch32h417-libusb: the WCH CH32H417 logic analyzers, the `WeAct`
//! Studio Logic Analyzer V2 among them: 16 channels, up to 200 MHz over
//! USB 3 and 20 MHz (40 MHz with eight channels) over USB 2, an adjustable
//! input threshold, and on the original board an ADC mode.
//!
//! The driver is not in sigrok. Its counterpart is the vendor's fork,
//! `WeActStudio.U3LogicAnalyzer`, whose libusb driver oxrok's oracle carries
//! (`oxrok/libsigrok`, `src/hardware/wch-ch32h417-libusb`); the protocol
//! is summarised in [`protocol`]. The firmware comes with the board and
//! nothing is uploaded.
//!
//! The scan, the open and the start of a capture send what the C driver
//! sends, in its order. Where this driver differs from it:
//!
//! - `conn=` selects a board (`BUS.ADDRESS`, `VID.PID`, or the `usb/...`
//!   id a scan prints). The C driver logs that it ignores it.
//! - Frames are whole: every frame starts with a frame begin, also
//!   without a trigger, and the pre-trigger samples and the trigger mark
//!   are inside it. The C driver begins no frame without a trigger, begins
//!   one after the samples before the trigger, and after the last frame's
//!   end begins another that holds nothing.
//! - A frame that ends with a transfer starts the next one from a clean
//!   trigger. The C driver resets the trigger only when samples are left
//!   over, and without a trigger then reads through a null pointer.
//! - A capture that ends because the samples stopped coming fails, unless
//!   the board's key ended it. The C driver ends it as any other and
//!   leaves the reason in `link_stop_reason`, which is kept as well.
//! - A trigger in the ADC mode is refused: the C driver matches it on the
//!   raw ADC bytes and sends the samples before it as logic.
//! - ADC samples belong to the input `adc_channel` selects. The C driver
//!   sends one sample for each enabled analog channel's two.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::key::CustomKey;
use crate::core::option::min_max_step_thresholds;
use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, SoftTriggerLogic,
    Trigger, TriggerFeatures, TriggerMatch, Unit, Value, ValueKind, ValueList,
};
use crate::usb::{BulkStream, Transport, UsbDeviceInfo, UsbId, split_conn};
use protocol::{Link, Mode, Profile};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

/// The id of every board, whatever its firmware reports itself as.
pub const ID: UsbId = UsbId { vendor: 0x1a86, product: 0x5537 };
const DRIVER: &str = "wch-ch32h417-libusb";
const INTERFACE: u8 = 0;
const LOGIC_CHANNELS: usize = 16;
/// `LINK_STOP_NONE` and `LINK_STOP_NO_DATA`.
const LINK_STOP_NONE: u64 = 0;
const LINK_STOP_NO_DATA: u64 = 1;

static KEY_HARDWARE_VERSION: CustomKey = CustomKey {
    id: "hardware_version",
    name: "hardware version",
    description: "The firmware version in the high byte and the hardware version in the low one, asked when read.",
    kind: ValueKind::U64,
};
static KEY_USB_VERSION: CustomKey = CustomKey {
    id: "usb_version",
    name: "usb version",
    description: "The link the device is on: 2 for USB 2, 3 for USB 3.",
    kind: ValueKind::U64,
};
static KEY_LINK_STOP_REASON: CustomKey = CustomKey {
    id: "link_stop_reason",
    name: "link stop reason",
    description: "Why the last capture ended on its own: 0 for a limit or a stop, 1 when the device stopped sending samples.",
    kind: ValueKind::U64,
};
static KEY_KEY_START_STOP: CustomKey = CustomKey {
    id: "key_start_stop",
    name: "key start stop",
    description: "Whether a press of the board's key ends a running capture.",
    kind: ValueKind::Bool,
};
static KEY_KEY_PRESSED: CustomKey = CustomKey {
    id: "key_pressed",
    name: "key pressed",
    description: "Whether the board's key was pressed since this was last read, asked when read.",
    kind: ValueKind::Bool,
};
static KEY_LED_BRIGHTNESS: CustomKey = CustomKey {
    id: "led_brightness",
    name: "LED brightness",
    description: "The indicator's brightness in percent, 0 to 100.",
    kind: ValueKind::U64,
};
static KEY_ADC_PRECISION: CustomKey = CustomKey {
    id: "adc_precision",
    name: "adc precision",
    description: "The ADC's resolution in bits, 8 or 10.",
    kind: ValueKind::U64,
};
static KEY_ADC_CHANNEL: CustomKey = CustomKey {
    id: "adc_channel",
    name: "adc channel",
    description: "The analog input the ADC samples, 0 or 1.",
    kind: ValueKind::U64,
};

const HARDWARE_VERSION: Key = Key::Custom(&KEY_HARDWARE_VERSION);
const USB_VERSION: Key = Key::Custom(&KEY_USB_VERSION);
const LINK_STOP_REASON: Key = Key::Custom(&KEY_LINK_STOP_REASON);
const KEY_START_STOP: Key = Key::Custom(&KEY_KEY_START_STOP);
const KEY_PRESSED: Key = Key::Custom(&KEY_KEY_PRESSED);
const LED_BRIGHTNESS: Key = Key::Custom(&KEY_LED_BRIGHTNESS);
const ADC_PRECISION: Key = Key::Custom(&KEY_ADC_PRECISION);
const ADC_CHANNEL: Key = Key::Custom(&KEY_ADC_CHANNEL);

/// The driver.
pub struct Ch32h417Driver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
/// `devopts[]`, in its order.
static DEV_OPTIONS: [OptionDesc; 16] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::NumLogicChannels, Access::GET_SET),
    OptionDesc::new(Key::PatternMode, Access::GET_SET_LIST),
    OptionDesc::new(Key::VoltageThreshold, Access::GET_SET_LIST),
    OptionDesc::new(HARDWARE_VERSION, Access::GET),
    OptionDesc::new(USB_VERSION, Access { get: true, set: false, list: true }),
    OptionDesc::new(LINK_STOP_REASON, Access { get: true, set: false, list: true }),
    OptionDesc::new(KEY_START_STOP, Access::GET_SET),
    OptionDesc::new(KEY_PRESSED, Access::GET),
    OptionDesc::new(LED_BRIGHTNESS, Access::GET_SET),
];
/// The two of `devopts_adc[]` that `devopts[]` lacks. `config_list()`
/// means that list for a channel group, which `std_opts_config_list()`
/// refuses (`api.c:934-938`), so in C no group has options and nothing
/// reaches these two. Here a board with an ADC lists them for the device.
static ADC_OPTIONS: [OptionDesc; 2] = [
    OptionDesc::new(ADC_PRECISION, Access::GET_SET),
    OptionDesc::new(ADC_CHANNEL, Access::GET_SET),
];
/// The soft trigger: stages of level and edge matches.
const TRIGGER_FEATURES: TriggerFeatures = TriggerFeatures {
    count: false,
    consecutive: false,
    duration: false,
    pulse: false,
    range: false,
    or: false,
    and: false,
    max_stages: 0,
    holdoff: false,
};
static TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];

impl Driver for Ch32h417Driver {
    fn id(&self) -> &'static str {
        DRIVER
    }

    fn long_name(&self) -> &'static str {
        "WCH CH32H417 Logic Analyzer (libusb)"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn usb_claims(&self) -> Vec<crate::core::UsbClaim> {
        vec![crate::core::UsbClaim::id(
            ID,
            "WCH",
            "CH32H417 logic analyzer (WeAct Studio Logic Analyzer V2)",
        )]
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::find_devices(&[ID])? {
            if let Some(c) = &conn
                && !conn_matches(c, &info)
            {
                continue;
            }
            let t = match crate::usb::open(&info) {
                Ok(t) => t,
                Err(e) => {
                    log::debug!("Failed to open device on {}: {e}", info.conn());
                    continue;
                }
            };
            match Ch32h417Device::from_transport(t) {
                Ok(dev) => {
                    log::info!("Found CH32H417 device, connection id '{}'", connection_id(&info));
                    devices.push(Box::new(dev));
                }
                Err(e) => log::debug!("Device on {}: {e}", info.conn()),
            }
        }
        log::info!("Found {} CH32H417 device(s)", devices.len());
        Ok(devices)
    }
}

/// The id the C driver gives a device (`usb_get_port_path()`):
/// `usb/BUS-PORT.PORT...`. A backend that knows no ports gives the bus
/// and the address, where C falls back to `libusb/INDEX`.
fn connection_id(info: &UsbDeviceInfo) -> String {
    if info.ports.is_empty() {
        return info.conn();
    }
    let ports: Vec<String> = info.ports.iter().map(u8::to_string).collect();
    format!("usb/{}-{}", info.bus, ports.join("."))
}

/// Whether `conn` names the device: its connection id as a scan prints
/// it, or one of sigrok's forms.
fn conn_matches(conn: &str, info: &UsbDeviceInfo) -> bool {
    conn == connection_id(info) || split_conn(conn).is_ok_and(|c| c.matches(info))
}

/// The serial number as the C scan reads it: the string descriptor,
/// read through the open device, in lower case. A device without one
/// has none.
fn read_serial(t: &mut dyn Transport) -> String {
    let index = t.string_indexes().map_or(0, |[_, _, serial]| serial);
    crate::usb::read_string_ascii(t, index).map(|s| s.to_ascii_lowercase()).unwrap_or_default()
}

/// Settings kept between acquisitions (`struct dev_context`).
#[derive(Clone, Debug)]
struct Settings {
    mode: Mode,
    samplerate: u64,
    limit_samples: u64,
    limit_frames: u64,
    capture_ratio: u64,
    /// `logic_channels`, 1 to 16: more than 8 are two bytes a sample.
    num_channels: i32,
    adc_ten_bit: bool,
    adc_channel: u8,
    /// The threshold last set; 0 until one is, as in C.
    threshold: f64,
    key_stop: bool,
    led_brightness: u8,
}

impl Settings {
    fn two_bytes(&self) -> bool {
        self.num_channels > 8
    }

    /// Bytes a sample takes on the wire.
    fn unit_size(&self) -> usize {
        match self.mode {
            Mode::Logic => 1 + usize::from(self.two_bytes()),
            Mode::Adc => 1 + usize::from(self.adc_ten_bit),
        }
    }
}

/// One board.
pub struct Ch32h417Device {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    profile: Profile,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    settings: Settings,
    /// The link is USB 2: `bcdUSB` below 3.00, learnt at the open.
    usb2: bool,
    trigger: Option<Trigger>,
    /// The link the scan left claimed, until the open takes it.
    pending: Option<Link>,
    /// The open link; an acquisition holds it while it runs.
    link: Arc<Mutex<Option<Link>>>,
    link_stop_reason: Arc<AtomicU64>,
    /// The version last read, which a failed read leaves in place.
    version: Mutex<u16>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// `ch32h417_libusb_open_device()` and
/// `ch32h417_libusb_init_comm_params()`: the interface claimed and the
/// answer endpoint drained.
fn set_up(mut t: Box<dyn Transport>) -> Result<Link> {
    t.claim_interface(INTERFACE).inspect_err(|e| {
        log::error!("Failed to claim interface 0: {}", crate::usb::libusb_error_name(e));
    })?;
    let mut link = Link::new(t);
    link.flush();
    Ok(link)
}

impl Ch32h417Device {
    /// A device on an open transport, as the scan makes one: the serial
    /// number read and matched to a board, the interface claimed, the
    /// answer endpoint drained. Also for a replayed recording.
    pub fn from_transport(mut transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        if usb.id() != ID {
            return Err(Error::arg("not a CH32H417 logic analyzer"));
        }
        let serial = read_serial(transport.as_mut());
        log::debug!("USB serial: {serial}");
        let Some(profile) = Profile::of_serial(&serial) else {
            return Err(Error::Arg(format!(
                "serial number '{serial}' is of no board the driver knows"
            )));
        };
        let link = set_up(transport)?;
        let mut channels: Vec<Channel> = (0..LOGIC_CHANNELS).map(Channel::logic).collect();
        let mut groups = vec![ChannelGroup::new("Logic", (0..LOGIC_CHANNELS).collect())];
        if profile.adc {
            for i in 0..2 {
                let name = format!("A{i}");
                channels.push(Channel::new(LOGIC_CHANNELS + i, ChannelKind::Analog, name.clone()));
                groups.push(ChannelGroup::new(name, vec![LOGIC_CHANNELS + i]));
            }
        }
        Ok(Ch32h417Device {
            info: DeviceInfo {
                driver: DRIVER,
                vendor: profile.vendor.into(),
                model: profile.model.into(),
                version: String::new(),
                serial,
                conn: Some(connection_id(&usb)),
            },
            usb,
            profile,
            channels,
            groups,
            // `ch32h417_libusb_dev_new()`.
            settings: Settings {
                mode: Mode::Logic,
                samplerate: protocol::RATES_USB3[0],
                limit_samples: 0,
                limit_frames: 1,
                capture_ratio: 0,
                num_channels: 16,
                adc_ten_bit: false,
                adc_channel: 0,
                threshold: 0.0,
                key_stop: false,
                led_brightness: 20,
            },
            usb2: false,
            trigger: None,
            pending: Some(link),
            link: Arc::new(Mutex::new(None)),
            link_stop_reason: Arc::new(AtomicU64::new(LINK_STOP_NONE)),
            version: Mutex::new(0),
            worker: None,
        })
    }

    /// The rates of the current mode, link and sample width.
    fn rates(&self) -> &'static [u64] {
        protocol::rates(self.profile, self.settings.mode, self.usb2, self.settings.two_bytes())
    }

    /// Runs `f` on the open link. Not while a capture holds it.
    fn with_link<T>(&self, f: impl FnOnce(&mut Link) -> T) -> Result<T> {
        match self.link.lock().expect("link lock").as_mut() {
            Some(link) => Ok(f(link)),
            None => Err(Error::device("the device is not open, or is capturing")),
        }
    }

    /// The position of the analog channel `adc_channel` selects, on a
    /// board that has one.
    fn adc_position(&self) -> Option<usize> {
        self.channels
            .iter()
            .position(|c| c.index == LOGIC_CHANNELS + usize::from(self.settings.adc_channel))
    }

    /// The soft trigger of a capture: the stages' matches on enabled
    /// channels (`soft_trigger_logic_check()` skips the others), over
    /// samples as wide as the wire's, armed once the pre-trigger samples
    /// are in.
    fn soft_trigger(&self) -> Option<SoftTriggerLogic> {
        let t = self.trigger.as_ref()?;
        let stages = t
            .stages
            .iter()
            .map(|st| {
                st.conditions
                    .iter()
                    .filter(|c| self.channels.iter().any(|ch| ch.index == c.channel && ch.enabled))
                    .map(|c| (c.channel, c.matches))
                    .collect()
            })
            .collect();
        let pre = self.settings.capture_ratio * self.settings.limit_samples / 100;
        Some(SoftTriggerLogic::new(stages, self.settings.unit_size(), pre as usize).arm_when_full())
    }

    /// What a capture cannot start with, known before anything is sent.
    fn check(&self, trigger: Option<&Trigger>) -> Result<u8> {
        let s = &self.settings;
        let index = protocol::rate_index(self.rates(), s.samplerate).ok_or_else(|| {
            log::error!("Invalid samplerate: {}", s.samplerate);
            Error::Arg(format!("the device has no samplerate {} in this mode", s.samplerate))
        })?;
        if s.mode == Mode::Adc {
            if trigger.is_some() {
                return Err(Error::not_supported("a trigger in the ADC mode"));
            }
            if self.adc_position().is_none() {
                return Err(Error::not_supported("the ADC mode on a board without an ADC"));
            }
        }
        Ok(index)
    }

    /// `ch32h417_libusb_acquisition_start()`: the answer endpoint
    /// drained, a stop in case the device still runs, the parameters, the
    /// sample transfers queued, then the start.
    fn begin(&self, link: &mut Link) -> Result<(Box<dyn BulkStream>, Capture)> {
        let s = &self.settings;
        let index = self.check(self.trigger.as_ref())?;
        log::info!("Flushing cmd endpoint...");
        link.flush();
        let endpoint = match s.mode {
            Mode::Logic => {
                if link.stop(Mode::Logic).is_err() {
                    log::warn!("Pre-stop logic command failed, continuing anyway...");
                }
                link.set_logic_params(s.two_bytes(), index, s.key_stop).inspect_err(|_| {
                    log::error!("Failed to set logic params.");
                })?;
                protocol::EP_LOGIC
            }
            Mode::Adc => {
                if link.stop(Mode::Adc).is_err() {
                    log::warn!("Pre-stop ADC command failed, continuing anyway...");
                }
                link.set_adc_params(s.adc_ten_bit, index).inspect_err(|_| {
                    log::error!("Failed to set ADC params.");
                })?;
                link.set_adc_channel(s.adc_channel).inspect_err(|_| {
                    log::error!("Failed to set ADC channel.");
                })?;
                protocol::EP_ADC
            }
        };
        // The transfers go up before the start, as the C ring does.
        let stream = link.transport().bulk_in_stream(
            endpoint,
            protocol::STREAM_TRANSFER,
            protocol::STREAM_DEPTH,
        )?;
        link.start(s.mode).inspect_err(|_| match s.mode {
            Mode::Logic => log::error!("Failed to start logic acquisition."),
            Mode::Adc => log::error!("Failed to start ADC acquisition."),
        })?;
        let stl = self.soft_trigger();
        let analog = (s.mode == Mode::Adc)
            .then(|| self.adc_position().map(|position| Adc { position, ten_bit: s.adc_ten_bit }))
            .flatten();
        Ok((
            stream,
            Capture {
                mode: s.mode,
                frames: Frames::new(s.unit_size(), s.limit_samples, s.limit_frames, stl, analog),
                reason: Arc::clone(&self.link_stop_reason),
            },
        ))
    }
}

impl Device for Ch32h417Device {
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
            None => {
                let mut options = DEV_OPTIONS.to_vec();
                if self.profile.adc {
                    options.extend_from_slice(&ADC_OPTIONS);
                }
                Ok(options)
            }
            // As in C, where the list of a group fails.
            Some(g) if self.groups.iter().any(|x| x.name.eq_ignore_ascii_case(g)) => {
                Err(Error::not_supported("options of a channel group"))
            }
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    /// `config_get()`. It does not look at the channel group, but libsigrok
    /// asks for the group's options first, which fail: nothing of a group
    /// is read or set.
    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if group.is_some() {
            return Err(Error::not_supported("options of a channel group"));
        }
        let s = &self.settings;
        Ok(match key {
            Key::Conn => match &self.info.conn {
                Some(c) => Value::Str(c.clone()),
                None => return Err(Error::arg("the device has no connection id")),
            },
            Key::LimitFrames => Value::U64(s.limit_frames),
            Key::LimitSamples => Value::U64(s.limit_samples),
            Key::Samplerate => Value::U64(s.samplerate),
            Key::CaptureRatio => Value::U64(s.capture_ratio),
            Key::NumLogicChannels => Value::I32(s.num_channels),
            Key::PatternMode => Value::Str(s.mode.name().to_owned()),
            Key::VoltageThreshold => Value::RangeF64(s.threshold, s.threshold),
            k if *k == HARDWARE_VERSION => {
                // A read that fails leaves the version read before.
                let mut version = self.version.lock().expect("version lock");
                if let Ok(Ok(v)) = self.with_link(Link::version) {
                    *version = v;
                }
                Value::U64(u64::from(*version))
            }
            k if *k == USB_VERSION => Value::U64(if self.usb2 { 2 } else { 3 }),
            k if *k == LINK_STOP_REASON => Value::U64(self.link_stop_reason.load(Ordering::SeqCst)),
            k if *k == KEY_START_STOP => {
                if !self.with_link(|l| l.key_available)? {
                    return Err(Error::not_supported("the board's key, with this firmware"));
                }
                Value::Bool(s.key_stop)
            }
            k if *k == KEY_PRESSED => Value::Bool(self.with_link(Link::key_pressed)?),
            k if *k == LED_BRIGHTNESS => Value::U64(u64::from(s.led_brightness)),
            k if *k == ADC_PRECISION => Value::U64(if s.adc_ten_bit { 10 } else { 8 }),
            k if *k == ADC_CHANNEL => Value::U64(u64::from(s.adc_channel)),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `config_set()`, with the same refusal for a group.
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if group.is_some() {
            return Err(Error::not_supported("options of a channel group"));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::Samplerate => {
                let rate = value.as_u64().ok_or_else(bad)?;
                if !self.rates().contains(&rate) {
                    return Err(Error::Arg(format!(
                        "the device has no samplerate {rate} in this mode"
                    )));
                }
                self.settings.samplerate = rate;
                log::info!("Samplerate set to {rate} Hz");
            }
            Key::LimitFrames => self.settings.limit_frames = value.as_u64().ok_or_else(bad)?,
            Key::LimitSamples => self.settings.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::CaptureRatio => {
                let ratio = value.as_u64().ok_or_else(bad)?;
                if ratio > 100 {
                    log::error!("Invalid capture ratio: {ratio} (must be 0-100)");
                    return Err(Error::arg("capture ratio must be 0..100"));
                }
                self.settings.capture_ratio = ratio;
            }
            Key::NumLogicChannels => {
                let Value::I32(n) = *value else { return Err(bad()) };
                if !(1..=16).contains(&n) {
                    log::error!("Invalid channel count: {n} (must be 1 to 16)");
                    return Err(Error::arg("logic_channels must be 1..16"));
                }
                log::info!("Channel count set to {n}");
                // Only a change picks the table again, and only over USB 2
                // does the width pick it at all.
                if self.settings.num_channels != n {
                    self.settings.num_channels = n;
                    if self.usb2 && self.settings.mode == Mode::Logic {
                        self.settings.samplerate = self.rates()[0];
                    }
                }
            }
            Key::PatternMode => {
                let name = value.as_str().ok_or_else(bad)?;
                let mode = Mode::NAMES.iter().find(|(n, _)| *n == name).ok_or_else(bad)?.1;
                if mode == Mode::Adc && !self.profile.adc {
                    log::error!("This device has no ADC, so it cannot be set to the ADC mode");
                    return Err(Error::arg("this device has no ADC"));
                }
                if mode != self.settings.mode {
                    self.settings.mode = mode;
                    self.settings.samplerate = self.rates()[0];
                    log::info!("Device mode set to {}", mode.name());
                }
            }
            Key::VoltageThreshold => {
                let Value::RangeF64(low, high) = *value else { return Err(bad()) };
                let threshold = f64::midpoint(low, high);
                let code = protocol::threshold_code(self.profile, threshold).inspect_err(|e| {
                    log::error!("{e}");
                })?;
                // Kept before it is sent, as in C.
                self.settings.threshold = threshold;
                self.with_link(|l| l.set_threshold(code))??;
            }
            k if *k == KEY_START_STOP => {
                let on = value.as_bool().ok_or_else(bad)?;
                if !self.with_link(|l| l.key_available)? {
                    log::warn!(
                        "This firmware does not report the board key; update it to use the key \
                         as a start/stop button"
                    );
                    return Err(Error::not_supported("the board's key, with this firmware"));
                }
                self.settings.key_stop = on;
                log::info!("Key start/stop {}", if on { "enabled" } else { "disabled" });
            }
            k if *k == LED_BRIGHTNESS => {
                let level = value.as_u64().ok_or_else(bad)?;
                let Some(level) = u8::try_from(level).ok().filter(|l| *l <= 100) else {
                    log::error!("Invalid indicator brightness: {level} (must be 0-100)");
                    return Err(Error::arg("led_brightness must be 0..100"));
                };
                // Sent at once: the LED is all it changes.
                self.settings.led_brightness = level;
                self.with_link(|l| l.set_led_brightness(level))??;
            }
            k if *k == ADC_PRECISION => match value.as_u64().ok_or_else(bad)? {
                8 => self.settings.adc_ten_bit = false,
                10 => self.settings.adc_ten_bit = true,
                other => {
                    log::error!("Invalid ADC precision: {other} (must be 8 or 10)");
                    return Err(Error::arg("adc_precision must be 8 or 10"));
                }
            },
            k if *k == ADC_CHANNEL => match value.as_u64().ok_or_else(bad)? {
                c @ 0..=1 => self.settings.adc_channel = c as u8,
                other => {
                    log::error!("Invalid ADC channel: {other} (must be 0 or 1)");
                    return Err(Error::arg("adc_channel must be 0 or 1"));
                }
            },
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Ok(match key {
            Key::Samplerate => ValueList::U64(self.rates().to_vec()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            // Only a board with an ADC lists its mode.
            Key::PatternMode => ValueList::Str(
                Mode::NAMES
                    .iter()
                    .filter(|(_, m)| *m == Mode::Logic || self.profile.adc)
                    .map(|(n, _)| (*n).to_owned())
                    .collect(),
            ),
            Key::VoltageThreshold => {
                let (low, high, step) = protocol::threshold_range(self.profile);
                ValueList::RangeF64(min_max_step_thresholds(low, high, step))
            }
            k if *k == USB_VERSION => ValueList::U64(vec![2, 3]),
            k if *k == LINK_STOP_REASON => ValueList::U64(vec![LINK_STOP_NONE, LINK_STOP_NO_DATA]),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `dev_open()`: the link the scan left, or the device opened again;
    /// the rates of the link it is on, the fastest of them selected; and
    /// the key count as it stands.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let mut link = match self.pending.take() {
            Some(link) => link,
            None => set_up(crate::usb::open(&self.usb)?).inspect_err(|_| {
                log::error!("Failed to open device on {}", self.usb.conn());
            })?,
        };
        if self.settings.mode == Mode::Logic {
            self.usb2 = match link.transport().usb_version() {
                Ok(bcd) => {
                    let usb2 = bcd < 0x0300;
                    log::info!(
                        "USB {} device detected (bcdUSB=0x{bcd:04x})",
                        if usb2 { "2.0" } else { "3.0" }
                    );
                    usb2
                }
                Err(e) => {
                    log::debug!("The USB version: {e}");
                    log::warn!("Failed to get device descriptor, assuming USB 3.0");
                    false
                }
            };
            self.settings.samplerate = self.rates()[0];
        }
        link.learn_key();
        *self.link.lock().expect("link lock") = Some(link);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        self.pending = None;
        drop(self.link.lock().expect("link lock").take());
        Ok(())
    }

    fn trigger_features(&self) -> TriggerFeatures {
        TRIGGER_FEATURES
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&TRIGGER_FEATURES)?;
        }
        self.trigger = trigger;
        Ok(())
    }

    fn check_capture(&self, trigger: Option<&Trigger>) -> Result<()> {
        self.check(trigger).map(|_| ())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let Some(mut link) = self.link.lock().expect("link lock").take() else {
            return Err(Error::device("device is not open"));
        };
        self.link_stop_reason.store(LINK_STOP_NONE, Ordering::SeqCst);
        match self.begin(&mut link) {
            Ok((stream, capture)) => {
                let slot = Arc::clone(&self.link);
                let stop = Arc::new(AtomicBool::new(false));
                let stop2 = Arc::clone(&stop);
                // A stop or a close joins the thread, which must not wait on
                // a queue nobody reads.
                let sink = sink.stopped_by(&stop);
                let handle = std::thread::Builder::new()
                    .name("wch-ch32h417 acquisition".into())
                    .spawn(move || {
                        let outcome = capture.run(stream, &mut link, &sink, &stop2);
                        sink.end(outcome);
                        *slot.lock().expect("link lock") = Some(link);
                    })
                    .map_err(Error::Io)?;
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                // libsigrok's session stops a device whose start failed
                // (session.c:831-835), and the stop sends the stop command.
                if link.stop(self.settings.mode).is_err() {
                    log::warn!("Failed to send stop command on finish.");
                }
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

/// The ADC input a capture's samples belong to.
#[derive(Clone, Copy, Debug)]
struct Adc {
    /// The channel's position among the device's.
    position: usize,
    ten_bit: bool,
}

/// One acquisition, on its own thread: `data_read_thread()` and
/// `finish_acquisition()`.
struct Capture {
    mode: Mode,
    frames: Frames,
    reason: Arc<AtomicU64>,
}

impl Capture {
    /// Takes transfers until the frames are done, the stop flag, the
    /// device gone, or [`protocol::SILENCE`] without one. Then the stop
    /// command, and the stream drops, which cancels the transfers queued.
    fn run(
        mut self,
        mut stream: Box<dyn BulkStream>,
        link: &mut Link,
        sink: &PacketSink,
        stop: &AtomicBool,
    ) -> Result<()> {
        sink.send(Packet::header());
        let mut outcome = Ok(());
        while !stop.load(Ordering::SeqCst) {
            match stream.next(protocol::SILENCE) {
                Ok(data) => {
                    if self.frames.feed(&data, sink) {
                        log::debug!("The frame limit is met.");
                        break;
                    }
                }
                Err(Error::Timeout) => {
                    // A board that fell silent because its key was pressed
                    // ended the capture; any other silence is a link that
                    // cannot carry the rate, or a board that was pulled.
                    if link.key_pressed() {
                        log::info!("No data for 500 ms, stopped by the key");
                    } else {
                        log::info!("No data for 500 ms, device may have stopped");
                        self.reason.store(LINK_STOP_NO_DATA, Ordering::SeqCst);
                        outcome = Err(Error::device(
                            "the device sent no samples for 500 ms: it stops when the USB link \
                             cannot carry the samplerate",
                        ));
                    }
                    break;
                }
                Err(e) => {
                    // The C ring puts a failed transfer back and ends on
                    // the silence that follows.
                    log::debug!("Sample transfer failed: {e}.");
                    self.reason.store(LINK_STOP_NO_DATA, Ordering::SeqCst);
                    outcome = Err(e);
                    break;
                }
            }
        }
        self.frames.finish(sink);
        if link.stop(self.mode).is_err() {
            log::warn!("Failed to send stop command on finish.");
        }
        drop(stream);
        outcome
    }
}

/// What a capture makes of the sample stream: frames of `limit` samples,
/// each from its trigger when there is one.
struct Frames {
    unit: usize,
    /// Samples a frame holds; 0 for one frame without an end.
    limit: u64,
    /// Frames to take; 0 for no limit.
    max_count: u64,
    stl: Option<SoftTriggerLogic>,
    analog: Option<Adc>,
    /// The current frame's trigger fired, or there is none to wait for.
    fired: bool,
    /// A frame begin was sent with no frame end yet.
    open: bool,
    sent: u64,
    /// Frames completed.
    count: u64,
}

impl Frames {
    fn new(
        unit: usize,
        limit: u64,
        limit_frames: u64,
        stl: Option<SoftTriggerLogic>,
        analog: Option<Adc>,
    ) -> Self {
        let fired = stl.is_none();
        Frames {
            unit,
            limit,
            max_count: limit_frames,
            stl,
            analog,
            fired,
            open: false,
            sent: 0,
            count: 0,
        }
    }

    /// Sends `data` as the capture's kind of samples.
    fn emit(&self, data: &[u8], sink: &PacketSink) {
        if data.is_empty() {
            return;
        }
        match self.analog {
            None => sink.send(Packet::logic(self.unit, data.to_vec())),
            Some(adc) => sink.send(Packet::Analog(AnalogData {
                channels: vec![adc.position],
                samples: protocol::adc_volts(data, adc.ten_bit),
                mq: Mq::Voltage,
                flags: MqFlags::empty(),
                unit: Unit::Volt,
                digits: 2,
            })),
        };
    }

    /// Samples the current frame still takes of `n`.
    fn room(&self, n: usize) -> usize {
        if self.limit == 0 {
            return n;
        }
        usize::try_from(self.limit.saturating_sub(self.sent)).map_or(n, |r| r.min(n))
    }

    /// One transfer's samples. True when the last frame is complete.
    fn feed(&mut self, data: &[u8], sink: &PacketSink) -> bool {
        let unit = self.unit;
        // Half a sample at the end cannot be used, and keeping it would
        // shift every sample after it by a byte.
        let data = &data[..data.len() - data.len() % unit];
        let total = data.len() / unit;
        let mut done = 0;
        while done < total {
            if self.fired {
                if !self.open {
                    sink.send(Packet::FrameBegin);
                    self.open = true;
                }
                let n = self.room(total - done);
                self.emit(&data[done * unit..(done + n) * unit], sink);
                self.sent += n as u64;
                done += n;
            } else {
                let found = self.stl.as_mut().and_then(|stl| stl.check(&data[done * unit..]));
                let Some((offset, pre)) = found else { break };
                sink.send(Packet::FrameBegin);
                self.open = true;
                for p in pre {
                    self.sent += (p.len() / unit) as u64;
                    sink.send(Packet::logic(unit, p));
                }
                sink.send(Packet::Trigger);
                let from = done + offset;
                let n = self.room(total - from);
                self.emit(&data[from * unit..(from + n) * unit], sink);
                self.sent += n as u64;
                done = from + n;
                self.fired = true;
            }
            if self.limit > 0 && self.sent >= self.limit {
                sink.send(Packet::FrameEnd);
                self.open = false;
                self.count += 1;
                self.sent = 0;
                if self.max_count > 0 && self.count >= self.max_count {
                    return true;
                }
                // The next frame waits for its own trigger.
                if let Some(stl) = self.stl.as_mut() {
                    stl.reset();
                    self.fired = false;
                }
            }
        }
        false
    }

    /// Ends a frame a stop or a failure cut short.
    fn finish(&mut self, sink: &PacketSink) {
        if std::mem::take(&mut self.open) {
            sink.send(Packet::FrameEnd);
        }
    }
}

#[cfg(test)]
mod tests;
