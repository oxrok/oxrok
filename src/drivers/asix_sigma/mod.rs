// Counterpart in sigrok:
//   libsigrok/src/hardware/asix-sigma/api.c,
//   Copyright (C) 2010-2012 Håvard Espeland <gus@ping.uio.no>,
//   Copyright (C) 2010 Martin Stensgård <mastensg@ping.uio.no>,
//   Copyright (C) 2010 Carl Henrik Lunde <chlunde@ping.uio.no>,
//   Copyright (C) 2020 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Niklas Thorne, jry
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! asix-sigma: the ASIX SIGMA and SIGMA2, 16 channel logic analyzers
//! sampling at 50 MHz / N (200 kHz to 50 MHz, 16 channels), 100 MHz (8)
//! or 200 MHz (4), with an external clock on any channel, a level and
//! edge trigger and a 32 MiB run length compressed sample memory read
//! after the capture ([`protocol`]).
//!
//! The scan looks at every device `a600:a000` (SIGMA, SIGMA2) and
//! `a600:a004` (OMEGA), reads its serial number, a hexadecimal number
//! whose upper 16 bits tell the models apart (`a601` SIGMA, `a602`
//! SIGMA2, `a603` OMEGA, which is found but skipped, as in libsigrok,
//! which has no support for it). Opening is libftdi's
//! `ftdi_usb_open_desc_index` by vendor, product and serial number:
//! the matching device's serial number read again, then libftdi's open
//! (reset, 9600 Bd). Every acquisition configures the FPGA when the
//! rate needs another netlist than the one running, which takes a
//! netlist file from sigrok-firmware (`asix-sigma-50.fw`, `-100.fw`,
//! `-200.fw`).
//!
//! The device sees the same requests as under sigrok-cli, through
//! [`crate::ftdi::libftdi`], checked with the emulator
//! (`tools/emu/asix-sigma.sh`), except where this port does what
//! libsigrok's driver means to: a trigger near the start of the memory
//! gets its marker, the last event of a capture is decoded (the C
//! driver drops it, and all of the last row of a wrapped memory), and
//! channel 16 is refused as the external clock, which the clock select
//! register's 4 bit pin field (written as the index plus 1) cannot
//! name: the C driver writes 16, which sets `CLKSEL_RISING` and leaves
//! the pin field 0. A set during an acquisition is kept for the next
//! one, as in the C driver, which reads its settings only at the start.
//! Unlike libsigrok's driver, the netlist the FPGA runs is forgotten
//! when the device is closed, so the next open configures it again.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Trigger, TriggerMatch, Value, ValueList,
};
use crate::ftdi::libftdi::{Context, FtdiError};
use crate::usb::{Transport, UsbDeviceInfo};
use protocol::{ClockEdge, Link, SAMPLERATES, TimeLimit, wmr, wreg};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// ASIX's vendor id.
pub const VENDOR: u16 = 0xa600;
/// The SIGMA and SIGMA2.
pub const PRODUCT_SIGMA: u16 = 0xa000;
/// The OMEGA.
pub const PRODUCT_OMEGA: u16 = 0xa004;

/// The channel names, as the cable labels them.
const CHANNEL_NAMES: [&str; 16] =
    ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16"];

/// The serial number buffer of the scan: the string is cut to 15
/// characters.
const SERNO_MAX: usize = 15;

/// The driver.
pub struct AsixSigmaDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::ProbeNames];
static DEV_OPTIONS: [OptionDesc; 9] = [
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::ExternalClock, Access::GET_SET),
    OptionDesc::new(Key::ExternalClockSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::ClockEdge, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
];
static TRIGGER_MATCHES: [TriggerMatch; 4] =
    [TriggerMatch::Zero, TriggerMatch::One, TriggerMatch::Rising, TriggerMatch::Falling];

/// The model a serial number names (its upper 16 bits), as the scan
/// sorts them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    /// `a601`.
    Sigma,
    /// `a602`.
    Sigma2,
    /// `a603`, which the driver does not support.
    Omega,
}

impl Model {
    /// The model of a serial number, if it is one of ASIX's.
    pub fn from_serno(serno: u64) -> Option<Self> {
        match serno >> 16 {
            0xa601 => Some(Model::Sigma),
            0xa602 => Some(Model::Sigma2),
            0xa603 => Some(Model::Omega),
            _ => None,
        }
    }

    /// The name.
    pub fn name(self) -> &'static str {
        match self {
            Model::Sigma => "SIGMA",
            Model::Sigma2 => "SIGMA2",
            Model::Omega => "OMEGA",
        }
    }
}

/// The serial number as `sr_atoul_base(text, ..., 16)` and the scan's
/// end check read it: leading white space, a sign, an optional `0x`,
/// hexadecimal digits, trailing white space and nothing else. An empty
/// string is 0, as `strtoul` makes it.
pub fn parse_serno(text: &str) -> Option<u64> {
    let space = |c: char| matches!(c, ' ' | '\t' | '\n' | '\x0b' | '\x0c' | '\r');
    let t = text.trim_start_matches(space);
    let (neg, body) = match t.as_bytes().first() {
        Some(b'-') => (true, &t[1..]),
        Some(b'+') => (false, &t[1..]),
        _ => (false, t),
    };
    let hex_digit = |s: &str| s.bytes().next().is_some_and(|b| b.is_ascii_hexdigit());
    let digits = match body.get(..2) {
        Some("0x" | "0X") if hex_digit(&body[2..]) => &body[2..],
        _ => body,
    };
    let n = digits.bytes().take_while(u8::is_ascii_hexdigit).count();
    if n == 0 {
        // No conversion: strtoul leaves the end at the start.
        return t.is_empty().then_some(0);
    }
    let value = u64::from_str_radix(&digits[..n], 16).ok()?;
    if !digits[n..].trim_start_matches(space).is_empty() {
        return None;
    }
    Some(if neg { value.wrapping_neg() } else { value })
}

/// A `conn` spec as `sr_usb_split_conn()` reads it (the same rules as
/// the saleae-logic16 port's).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Conn {
    Ids(u16, u16),
    BusAddress(u8, u8),
}

/// `sr_usb_find()`'s reading of `conn`, or the message it logs.
fn split_conn(conn: &str) -> std::result::Result<Conn, &'static str> {
    const INVALID: &str = "Invalid input, or neither VID:PID nor bus.address specified.";
    const NEITHER: &str = "Could neither determine VID:PID nor bus.address numbers.";
    let Some((a, b)) = conn.split_once('.') else { return Err(NEITHER) };
    let hex = |s: &str| s.len() == 4 && s.bytes().all(|c| c.is_ascii_hexdigit());
    if hex(a) && hex(b) {
        let (vid, pid) =
            (u16::from_str_radix(a, 16).unwrap_or(0), u16::from_str_radix(b, 16).unwrap_or(0));
        return if vid != 0 && pid != 0 { Ok(Conn::Ids(vid, pid)) } else { Err(NEITHER) };
    }
    let dec = |s: &str| !s.is_empty() && s.bytes().all(|c| c.is_ascii_digit());
    if !(dec(a) && dec(b)) {
        return Err(NEITHER);
    }
    let num = |s: &str| s.parse::<u64>().unwrap_or(u64::MAX);
    let (bus, addr) = (num(a), num(b));
    if bus > 255 || addr > 127 {
        return Err(INVALID);
    }
    match (u8::try_from(bus), u8::try_from(addr)) {
        (Ok(bus @ 1..), Ok(addr @ 1..)) => Ok(Conn::BusAddress(bus, addr)),
        _ => Err(NEITHER),
    }
}

impl Conn {
    fn matches(self, d: &UsbDeviceInfo) -> bool {
        match self {
            Conn::Ids(vid, pid) => d.vendor_id == vid && d.product_id == pid,
            Conn::BusAddress(bus, addr) => d.bus == bus && d.address == addr,
        }
    }
}

/// The serial number and model of an opened device, as the scan reads
/// them, or the warning that skips it.
fn identify(t: &mut dyn Transport) -> std::result::Result<(String, Model), String> {
    let index = t.string_indexes().map_or(0, |i| i[2]);
    if index == 0 {
        return Err("Cannot get serial number (index 0).".into());
    }
    let text = crate::usb::read_string_ascii(t, index)
        .map_err(|e| format!("Cannot get serial number ({e})."))?;
    let text: String = text.chars().take(SERNO_MAX).collect();
    let Some(serno) = parse_serno(&text) else {
        return Err(format!("Cannot interpret serial number {text}."));
    };
    match Model::from_serno(serno) {
        Some(Model::Omega) => {
            log::info!("Found OMEGA, serno {text}.");
            Err("OMEGA support is not implemented yet.".into())
        }
        Some(m) => {
            log::info!("Found {}, serno {text}.", m.name());
            Ok((text, m))
        }
        None => Err(format!("Unknown serno {text}, skipping.")),
    }
}

impl Driver for AsixSigmaDriver {
    fn id(&self) -> &'static str {
        "asix-sigma"
    }

    fn long_name(&self) -> &'static str {
        "ASIX SIGMA/SIGMA2"
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
        let conn = match conn.as_deref().map(split_conn).transpose() {
            Ok(c) => c,
            Err(msg) => {
                log::error!("{msg}");
                return Ok(Vec::new());
            }
        };
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::list_devices()? {
            if conn.is_some_and(|c| !c.matches(&info)) {
                continue;
            }
            if info.vendor_id != VENDOR
                || !(info.product_id == PRODUCT_SIGMA || info.product_id == PRODUCT_OMEGA)
            {
                continue;
            }
            let mut t = match crate::usb::open(&info) {
                Ok(t) => t,
                Err(e) => {
                    log::warn!(
                        "Cannot open USB device {:04x}.{:04x}: {e}.",
                        info.vendor_id,
                        info.product_id
                    );
                    continue;
                }
            };
            let found = identify(t.as_mut());
            drop(t);
            match found {
                Ok((serial, model)) => {
                    devices.push(Box::new(SigmaDevice::new(
                        info,
                        serial,
                        model,
                        probe_names.as_deref(),
                        None,
                    )));
                }
                Err(msg) => log::warn!("{msg}"),
            }
        }
        Ok(devices)
    }
}

/// Settings kept between acquisitions.
#[derive(Clone, Debug)]
struct Settings {
    samplerate: u64,
    use_ext_clock: bool,
    clock_pin: usize,
    clock_edge: ClockEdge,
    limit_msec: u64,
    limit_samples: u64,
    capture_ratio: u64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            samplerate: SAMPLERATES[0],
            use_ext_clock: false,
            clock_pin: 0,
            clock_edge: ClockEdge::Rising,
            limit_msec: 0,
            limit_samples: 0,
            capture_ratio: 50,
        }
    }
}

/// One SIGMA or SIGMA2.
pub struct SigmaDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    /// The names the scan gave, which the clock source takes.
    channel_names: Vec<String>,
    settings: Settings,
    trigger: Option<Trigger>,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl SigmaDevice {
    fn new(
        usb: UsbDeviceInfo,
        serial: String,
        model: Model,
        probe_names: Option<&str>,
        transport: Option<Box<dyn Transport>>,
    ) -> Self {
        let names = crate::core::util::parse_probe_names(probe_names.unwrap_or(""), &CHANNEL_NAMES);
        SigmaDevice {
            info: DeviceInfo {
                driver: "asix-sigma",
                vendor: "ASIX".into(),
                model: model.name().into(),
                version: String::new(),
                serial,
                conn: Some(usb.conn()),
            },
            usb,
            channels: names
                .iter()
                .enumerate()
                .map(|(i, n)| Channel::new(i, ChannelKind::Logic, n.clone()))
                .collect(),
            channel_names: names,
            settings: Settings::default(),
            trigger: None,
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording),
    /// its serial number read from it first, as the scan does.
    ///
    /// # Errors
    ///
    /// When the transport is not a SIGMA or SIGMA2 the scan would take.
    pub fn from_transport(mut transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        if usb.vendor_id != VENDOR || usb.product_id != PRODUCT_SIGMA {
            return Err(Error::arg("not an ASIX SIGMA"));
        }
        let (serial, model) = identify(transport.as_mut()).map_err(Error::Device)?;
        Ok(Self::new(usb, serial, model, None, Some(transport)))
    }

    /// `sigma_ftdi_open`: `ftdi_usb_open_desc_index` with the vendor,
    /// product and serial number: each device with those ids opened and
    /// its serial number read until one matches, then libftdi's open on
    /// it.
    fn ftdi_open(&mut self) -> std::result::Result<Context, FtdiError> {
        let fail = |code, message| FtdiError { code, message };
        let candidates: Vec<Option<UsbDeviceInfo>> = if self.pending.is_some() {
            vec![None]
        } else {
            crate::usb::list_devices()
                .map_err(|_| fail(-12, "libusb_get_device_list() failed"))?
                .into_iter()
                .filter(|d| {
                    d.vendor_id == self.usb.vendor_id && d.product_id == self.usb.product_id
                })
                .map(Some)
                .collect()
        };
        for info in candidates {
            let mut t = match (info, self.pending.take()) {
                (None, Some(t)) => t,
                (Some(info), _) => {
                    crate::usb::open(&info).map_err(|_| fail(-4, "usb_open() failed"))?
                }
                (None, None) => break,
            };
            let index = t.string_indexes().map_or(0, |i| i[2]);
            let serial = crate::usb::read_string_ascii(t.as_mut(), index)
                .map_err(|_| fail(-9, "unable to fetch serial number"))?;
            if serial != self.info.serial {
                continue;
            }
            return Context::open(t);
        }
        Err(fail(-3, "device not found"))
    }

    /// `dev_acquisition_start` up to the header: the trigger, the netlist
    /// for the rate, the time limit, the trigger, clock and mode
    /// registers.
    fn begin(&mut self, link: &mut Link) -> Result<(protocol::Acquisition, bool)> {
        let enabled: Vec<bool> = {
            let mut v = vec![false; CHANNEL_NAMES.len()];
            for c in &self.channels {
                if let Some(e) = v.get_mut(c.index) {
                    *e = c.enabled;
                }
            }
            v
        };
        let s = &mut self.settings;
        let trig = protocol::convert_trigger(self.trigger.as_ref(), &enabled, s.samplerate)
            .map_err(|msg| {
                log::error!("{msg}");
                log::error!("Could not configure triggers.");
                Error::arg(msg.trim_end_matches('.').to_owned())
            })?;
        let mut rate_changed = false;
        if s.use_ext_clock {
            if s.samplerate != 50_000_000 {
                log::info!("External clock, forcing 50MHz samplerate.");
                rate_changed = true;
            }
            s.samplerate = 50_000_000;
            // The pin field holds 4 bits, written as the index plus 1.
            if s.clock_pin + 1 > 0x0f {
                let name = &self.channel_names[s.clock_pin];
                log::error!(
                    "Channel {name} cannot be the external clock: the clock select register has no code for it."
                );
                return Err(Error::arg(format!("channel {name} cannot be the external clock")));
            }
        }
        // sigma_set_samplerate.
        let rate = protocol::normalize_samplerate(s.samplerate)
            .ok_or_else(|| Error::arg("invalid samplerate"))?;
        let (fw, num_channels) =
            protocol::Firmware::for_rate(rate).ok_or_else(|| Error::arg("invalid samplerate"))?;
        link.upload_firmware(fw)?;
        // sigma_set_acquire_timeout.
        let ms = protocol::acquire_msecs(
            s.limit_samples,
            s.limit_msec,
            s.samplerate,
            trig.map(|_| s.capture_ratio),
        );
        let mut acquire = TimeLimit { limit: ms.map(Duration::from_millis), start: None };
        let late = ms.is_some() && trig.is_some();
        if ms.is_some() && trig.is_none() {
            acquire.start = Some(Instant::now());
        }
        // Trigger programming.
        link.set_register(wreg::TRIGGER_SELECT2, protocol::trgsel2::RESET)?;
        let mut trgsel2 = 0;
        if s.samplerate >= 100_000_000 {
            link.set_register(wreg::TRIGGER_SELECT2, 0x81)?;
            trgsel2 = protocol::fast_trigsel2(trig.as_ref());
        } else if s.samplerate <= 50_000_000 {
            link.write_trigger_lut(&protocol::build_basic_trigger(trig.as_ref()))?;
            trgsel2 = protocol::trgsel2::LEDSEL1 | protocol::trgsel2::LEDSEL0;
        }
        link.write_register(wreg::TRIGGER_OPTION, &protocol::trigger_options())?;
        link.set_register(wreg::TRIGGER_SELECT2, trgsel2)?;
        // Clock and channels.
        if s.samplerate > 50_000_000 {
            let pindis = protocol::pin_disable(num_channels);
            link.set_register(wreg::CLOCK_SELECT, (pindis & 0xff) as u8)?;
        } else {
            let ext = s.use_ext_clock.then_some((s.clock_pin, s.clock_edge));
            let bytes = protocol::clock_select_50(s.samplerate, ext, num_channels);
            link.write_register(wreg::CLOCK_SELECT, &bytes)?;
        }
        link.set_register(wreg::POST_TRIGGER, (s.capture_ratio * 255 / 100) as u8)?;
        let mut mode = wmr::TRGRES | wmr::SDRAMWRITEEN;
        if trig.is_some() {
            mode |= wmr::TRGEN;
        }
        link.set_register(wreg::MODE, mode)?;
        let acq = protocol::Acquisition::new(trig, num_channels, s.limit_samples, acquire, late);
        Ok((acq, rate_changed))
    }
}

impl Device for SigmaDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
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
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let s = &self.settings;
        Ok(match key {
            Key::Conn => Value::Str(self.usb.conn()),
            Key::Samplerate => Value::U64(s.samplerate),
            Key::ExternalClock => Value::Bool(s.use_ext_clock),
            Key::ExternalClockSource => Value::Str(self.channel_names[s.clock_pin].clone()),
            Key::ClockEdge => Value::Str(s.clock_edge.name().into()),
            Key::LimitMsec => Value::U64(s.limit_msec),
            Key::LimitSamples => Value::U64(s.limit_samples),
            Key::CaptureRatio => Value::U64(s.capture_ratio),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let s = &mut self.settings;
        match key {
            Key::Samplerate => {
                let want = value.as_u64().ok_or_else(bad)?;
                let have = protocol::normalize_samplerate(want).ok_or_else(bad)?;
                if have != want {
                    log::info!(
                        "Adjusted samplerate {} to {}.",
                        crate::core::util::samplerate_string(want),
                        crate::core::util::samplerate_string(have)
                    );
                }
                s.samplerate = have;
            }
            Key::ExternalClock => s.use_ext_clock = value.as_bool().ok_or_else(bad)?,
            Key::ExternalClockSource => {
                let name = value.as_str().ok_or_else(bad)?;
                s.clock_pin = self.channel_names.iter().position(|n| n == name).ok_or_else(bad)?;
            }
            Key::ClockEdge => {
                s.clock_edge = value.as_str().and_then(ClockEdge::from_name).ok_or_else(bad)?;
            }
            Key::LimitMsec => s.limit_msec = value.as_u64().ok_or_else(bad)?,
            Key::LimitSamples => s.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::CaptureRatio => {
                let r = value.as_u64().ok_or_else(bad)?;
                // sr_config_set() refuses it before the driver sees it
                // (hwdriver.c, check_key()).
                if r > 100 {
                    log::error!("Capture ratio must be 0..100.");
                    return Err(Error::arg("capture ratio must be 0..100"));
                }
                s.capture_ratio = r;
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
            Key::ExternalClockSource => ValueList::Str(self.channel_names.clone()),
            Key::ClockEdge => {
                ValueList::Str(ClockEdge::NAMES.iter().map(|n| (*n).to_owned()).collect())
            }
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("asix-sigma lock").is_some() {
            return Ok(());
        }
        let ctx = self.ftdi_open().map_err(|e| {
            log::error!("Cannot open device ({}): {}.", e.code, e.message);
            Error::from(e)
        })?;
        *self.link.lock().expect("asix-sigma lock") = Some(Link::new(ctx));
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        drop(self.link.lock().expect("asix-sigma lock").take());
        Ok(())
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        // Checked at the start, where the C driver converts it, against
        // the rate and the channels of that moment.
        self.trigger = trigger;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let Some(mut link) = self.link.lock().expect("asix-sigma lock").take() else {
            return Err(Error::device("device is not open"));
        };
        let (mut acq, rate_changed) = match self.begin(&mut link) {
            Ok(v) => v,
            Err(e) => {
                *self.link.lock().expect("asix-sigma lock") = Some(link);
                return Err(e);
            }
        };
        let samplerate = self.settings.samplerate;
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("asix-sigma acquisition".into())
            .spawn(move || {
                sink.send(Packet::header());
                if rate_changed {
                    sink.send(Packet::meta(Key::Samplerate, Value::U64(samplerate)));
                }
                acq.run(&mut link, &sink, &stop2);
                *slot.lock().expect("asix-sigma lock") = Some(link);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serial_numbers_read_as_strtoul_does() {
        assert_eq!(parse_serno("A6010A3C"), Some(0xa601_0a3c));
        assert_eq!(parse_serno("  a6020b71 "), Some(0xa602_0b71));
        assert_eq!(parse_serno("0xA6010001"), Some(0xa601_0001));
        assert_eq!(parse_serno("A601XYZ"), None);
        assert_eq!(parse_serno(""), Some(0));
        assert_eq!(parse_serno("-"), None);
        assert_eq!(parse_serno("0x"), None);
        assert_eq!(parse_serno("-1"), Some(u64::MAX));
        assert_eq!(parse_serno("1ffffffffffffffff"), None);
        assert_eq!(Model::from_serno(0xa603_0d52), Some(Model::Omega));
        assert_eq!(Model::from_serno(0xa604_0001), None);
    }

    #[test]
    fn conn_specs_follow_sr_usb_split_conn() {
        assert_eq!(split_conn("a600.a000"), Ok(Conn::Ids(0xa600, 0xa000)));
        assert_eq!(split_conn("3.7"), Ok(Conn::BusAddress(3, 7)));
        assert!(split_conn("300.1").is_err());
        assert!(split_conn("1.200").is_err());
        assert!(split_conn("0.0").is_err());
        assert!(split_conn("nonsense").is_err());
    }

    /// The scan's and the open's reads of the serial number, libftdi's
    /// open (reset, 9600 Bd on a BM).
    const OPENED: &str = "# oxrok usb fixture 1\n\
        device vid=a600 pid=a000 version=0400 bus=0 addr=0 strings=1,2,3\n\
        ctl_in 00 06 0300 0000 4 -> 04030904\n\
        ctl_in 00 06 0303 0409 255 -> 120341003600300031003000410033004300\n\
        ctl_in 00 06 0300 0000 4 -> 04030904\n\
        ctl_in 00 06 0303 0409 255 -> 120341003600300031003000410033004300\n\
        claim 0 -> ok\nmps 81 -> 64\n\
        ctl_out 40 00 0000 0001 - -> ok\nctl_out 40 03 4138 0000 - -> ok\n";

    fn device(steps: &str) -> SigmaDevice {
        let text = format!("{OPENED}{steps}");
        let t = crate::usb::record::Replay::from_text(&text).unwrap();
        SigmaDevice::from_transport(Box::new(t)).unwrap()
    }

    #[test]
    fn opens_by_serial_number() {
        let mut dev = device("");
        assert_eq!(dev.info.model, "SIGMA");
        assert_eq!(dev.info.serial, "A6010A3C");
        dev.open().unwrap();
        dev.close().unwrap();
    }

    #[test]
    fn settings_as_the_driver_takes_them() {
        let mut dev = device("");
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(200_000));
        dev.set(&Key::Samplerate, &Value::U64(30_000_000), None).unwrap();
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(50_000_000));
        assert!(dev.set(&Key::Samplerate, &Value::U64(100_000), None).is_err());
        dev.set(&Key::ExternalClockSource, &Value::Str("7".into()), None).unwrap();
        assert_eq!(dev.get(&Key::ExternalClockSource, None).unwrap(), Value::Str("7".into()));
        assert!(dev.set(&Key::ExternalClockSource, &Value::Str("17".into()), None).is_err());
        assert!(dev.set(&Key::ClockEdge, &Value::Str("sideways".into()), None).is_err());
        dev.set(&Key::ClockEdge, &Value::Str("either".into()), None).unwrap();
        assert_eq!(dev.get(&Key::ClockEdge, None).unwrap(), Value::Str("either".into()));
        assert!(dev.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
    }

    #[test]
    fn channel_16_is_no_external_clock() {
        let mut dev = device("");
        dev.open().unwrap();
        dev.set(&Key::ExternalClock, &Value::Bool(true), None).unwrap();
        dev.set(&Key::ExternalClockSource, &Value::Str("16".into()), None).unwrap();
        let (tx, _rx) = std::sync::mpsc::sync_channel(4);
        // Refused before anything goes to the device.
        assert!(dev.start(PacketSink::new(tx, 0)).is_err());
        dev.close().unwrap();
    }
}
