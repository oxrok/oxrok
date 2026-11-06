// Counterpart in sigrok:
//   libsigrok/src/hardware/microchip-pickit2/api.c,
//   libsigrok/src/hardware/microchip-pickit2/protocol.c,
//   libsigrok/src/hardware/microchip-pickit2/protocol.h,
//   Copyright (C) 2018-2019 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! The Microchip PICkit2 programmer as a three channel logic analyzer:
//! 1024 samples at up to 1 MHz on pins 4 to 6, with a hardware trigger
//! and a few pre-trigger positions. The driver is `microchip-pickit2`.
//!
//! The device is a HID with a 64 byte interrupt endpoint each way.
//! libsigrok reaches it with raw USB interrupt transfers. Linux keeps
//! `usbhid` off the PICkit2 (it is on the kernel's HID ignore list, so
//! there is no hidraw node), so there this driver does the same, over
//! [`crate::usb`]; macOS does not let a program claim a HID interface,
//! and Windows binds its HID driver, so elsewhere it speaks HID reports.

#![forbid(unsafe_code)]
#![allow(clippy::doc_markdown)]

use crate::core::{
    Access, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error,
    Key, OptionDesc, Packet, PacketSink, Result, Trigger, TriggerFeatures, TriggerMatch, Value,
    ValueList,
};
use crate::hid::HidDevice;
use crate::usb::{Transport, UsbDeviceInfo};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

const VENDOR_ID: u16 = 0x04d8;
const PRODUCT_ID: u16 = 0x0033;
const CHANNEL_COUNT: usize = 3;
/// Samples per capture.
pub const SAMPLE_COUNT: usize = 1024;
const RAW_LEN: usize = 4 * 128;
const PACKET: usize = 64;
const CMD_READ: u8 = 0xac;
const CMD_PAD: u8 = 0xad;
const CMD_SETUP: u8 = 0xb8;
const CMD_SETPOS: u8 = 0xb9;
const SEL_BANK0: u8 = 0x06;
const TIMEOUT: Duration = Duration::from_millis(250);
const USB_INTERFACE: u8 = 0;
const EP_OUT: u8 = 0x01;
const EP_IN: u8 = 0x81;

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::ProbeNames];
const CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static DEV_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET_LIST),
];
const TRIGGER_MATCHES: [TriggerMatch; 4] =
    [TriggerMatch::Zero, TriggerMatch::One, TriggerMatch::Rising, TriggerMatch::Falling];
/// Capture ratio presets: an index into the trigger position table.
pub static CAPTURE_RATIOS: &[u64] = &[0, 1, 2, 3, 4, 5, 6];
/// Sample rates.
pub static SAMPLERATES: &[u64] =
    &[5_000, 10_000, 25_000, 50_000, 100_000, 250_000, 500_000, 1_000_000];
const CHANNEL_NAMES: [&str; 3] = ["pin4", "pin5", "pin6"];
/// Trigger position words per capture ratio preset: 10, 50 and 90
/// percent of the first window, then 10 percent plus one to three
/// window widths. Their derivation is unknown, as in libsigrok.
const TRIGGER_POSITIONS: [u16; 7] = [0x0000, 0x03cc, 0x000a, 0x0248, 0x07b4, 0x0b9c, 0x0f84];
const CHANNEL_TRIGGER_MASKS: [u8; 3] = [0x04, 0x08, 0x10];

/// The driver.
pub struct Pickit2Driver;

impl Driver for Pickit2Driver {
    fn id(&self) -> &'static str {
        "microchip-pickit2"
    }

    fn long_name(&self) -> &'static str {
        "Microchip PICkit2"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = format!("{VENDOR_ID:04x}.{PRODUCT_ID:04x}");
        let mut probe_names = None;
        for (key, value) in options {
            match key {
                Key::Conn => {
                    value.as_str().ok_or_else(|| Error::arg("conn"))?.clone_into(&mut conn);
                }
                Key::ProbeNames => probe_names = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        // A USB replay file stands in for the device on every system, so
        // the recording made on Linux replays on the others too.
        if cfg!(target_os = "linux") || crate::usb::replaying() {
            // As libsigrok: VID.PID in hex or bus.address.
            for info in crate::usb::list_devices()? {
                if !info.matches_conn(&conn) {
                    continue;
                }
                let mut dev = Pickit2Device::new(Source::Usb(info));
                if let Some(spec) = &probe_names {
                    dev.name_channels(spec);
                }
                devices.push(Box::new(dev));
            }
            return Ok(devices);
        }
        let (vid, pid) = match conn.split_once('.') {
            Some((v, p)) => match (u16::from_str_radix(v, 16), u16::from_str_radix(p, 16)) {
                (Ok(vid), Ok(pid)) => (vid, pid),
                _ => return Err(Error::Arg(format!("Invalid conn '{conn}': use VID.PID."))),
            },
            None => return Err(Error::Arg(format!("Invalid conn '{conn}': use VID.PID."))),
        };
        for info in crate::hid::list()? {
            if info.vendor_id != vid || info.product_id != pid {
                continue;
            }
            let mut dev = Pickit2Device::new(Source::Hid(info, conn.clone()));
            if let Some(spec) = &probe_names {
                dev.name_channels(spec);
            }
            devices.push(Box::new(dev));
        }
        Ok(devices)
    }
}

/// Settings kept between acquisitions.
#[derive(Clone, Copy, Debug)]
struct Settings {
    samplerate_idx: usize,
    limit_samples: u64,
    capture_ratio_idx: usize,
}

/// Where a scanned PICkit2 is, to open it.
pub enum Source {
    /// A HID, with the `conn` it was found by.
    Hid(crate::hid::HidInfo, String),
    /// A USB device, reached with interrupt transfers.
    Usb(UsbDeviceInfo),
}

/// The packets to and from endpoint 1, over HID reports or raw USB.
pub trait Link: Send {
    /// Sends one 64 byte packet.
    ///
    /// # Errors
    ///
    /// When the transport fails or takes less than the packet.
    fn send(&mut self, packet: &[u8; PACKET]) -> Result<()>;
    /// Reads one packet into `buf`, waiting at most `timeout`; its
    /// length, or 0 when none came.
    ///
    /// # Errors
    ///
    /// When the transport fails.
    fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize>;
    /// Gives the device back, before it is closed.
    ///
    /// # Errors
    ///
    /// When the transport fails.
    fn close(&mut self) -> Result<()> {
        Ok(())
    }
}

/// HID reports: output reports out, input reports in.
pub struct HidLink(pub Box<dyn HidDevice>);

impl Link for HidLink {
    fn send(&mut self, packet: &[u8; PACKET]) -> Result<()> {
        // HID output reports carry the report id first; this device
        // has none, so it is zero.
        let mut report = [0u8; 1 + PACKET];
        report[1..].copy_from_slice(packet);
        self.0.write_report(&report)
    }

    fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        self.0.read_report(buf, timeout)
    }
}

/// Interrupt transfers on the claimed interface 0, as libsigrok does.
pub struct UsbLink(pub Box<dyn Transport>);

impl UsbLink {
    /// Claims the interface (detaching a kernel driver, as libsigrok's
    /// `dev_open` does).
    ///
    /// # Errors
    ///
    /// When the interface cannot be claimed.
    pub fn claim(mut transport: Box<dyn Transport>) -> Result<UsbLink> {
        transport.claim_interface(USB_INTERFACE)?;
        Ok(UsbLink(transport))
    }
}

impl Link for UsbLink {
    fn send(&mut self, packet: &[u8; PACKET]) -> Result<()> {
        let sent = self.0.interrupt_out(EP_OUT, packet, TIMEOUT)?;
        if sent != PACKET {
            return Err(Error::device(format!("USB short send: {sent}/{PACKET} bytes.")));
        }
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        match self.0.interrupt_in(EP_IN, PACKET, timeout) {
            Ok(d) => {
                let n = d.len().min(buf.len());
                buf[..n].copy_from_slice(&d[..n]);
                Ok(n)
            }
            Err(Error::Timeout) => Ok(0),
            Err(e) => Err(e),
        }
    }

    fn close(&mut self) -> Result<()> {
        self.0.release_interface(USB_INTERFACE)
    }
}

/// One PICkit2.
pub struct Pickit2Device {
    info: DeviceInfo,
    source: Option<Source>,
    conn: String,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    settings: Settings,
    trigger: Option<Trigger>,
    pending: Option<Box<dyn Link>>,
    dev: Arc<Mutex<Option<Box<dyn Link>>>>,
    worker: Option<(Arc<AtomicBool>, JoinHandle<()>)>,
}

impl Pickit2Device {
    /// A device for a listed PICkit2, opened on `open`. A USB device's
    /// connection is its `bus.address`, as libsigrok reports it; a HID's
    /// is the `conn` it was found by.
    pub fn new(source: Source) -> Pickit2Device {
        let conn = match &source {
            Source::Hid(_, conn) => conn.clone(),
            Source::Usb(info) => info.conn(),
        };
        let mut d = Self::with_link(None, conn);
        d.source = Some(source);
        d
    }

    /// A device on an already open HID, e.g. a fake in tests.
    pub fn with_device(dev: Option<Box<dyn HidDevice>>, conn: String) -> Pickit2Device {
        Self::with_link(dev.map(|d| Box::new(HidLink(d)) as Box<dyn Link>), conn)
    }

    /// A device on an already open link, e.g. a replayed USB transport.
    pub fn with_link(dev: Option<Box<dyn Link>>, conn: String) -> Pickit2Device {
        let channels: Vec<Channel> = CHANNEL_NAMES
            .iter()
            .enumerate()
            .map(|(i, n)| Channel::new(i, ChannelKind::Logic, *n))
            .collect();
        Pickit2Device {
            info: DeviceInfo {
                driver: "microchip-pickit2",
                vendor: "Microchip".to_owned(),
                model: "PICkit2".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: Some(conn.clone()),
            },
            source: None,
            conn,
            groups: vec![ChannelGroup::new("Logic", (0..channels.len()).collect())],
            channels,
            settings: Settings {
                samplerate_idx: SAMPLERATES.len() - 1,
                limit_samples: SAMPLE_COUNT as u64,
                capture_ratio_idx: 0,
            },
            trigger: None,
            pending: dev,
            dev: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// Applies a `probe_names=` spec: comma separated names in order.
    pub fn name_channels(&mut self, spec: &str) {
        for (ch, name) in self.channels.iter_mut().zip(spec.split(',')) {
            let name = name.trim();
            if !name.is_empty() && name != "-" {
                name.clone_into(&mut ch.name);
            }
        }
    }
}

/// The SETUP packet for a trigger on the enabled channels: per channel
/// the condition code of libsigrok (0 none, 1 zero, 2 one, 3 rising,
/// 4 falling).
pub fn setup_packet(
    conditions: [u8; CHANNEL_COUNT],
    ratio_idx: usize,
    samplerate: u64,
) -> [u8; PACKET] {
    let (mut en, mut lvl, mut edge) = (0u8, 0u8, 0u8);
    for (i, cond) in conditions.iter().enumerate() {
        if *cond == 0 {
            continue;
        }
        en |= CHANNEL_TRIGGER_MASKS[i];
        if matches!(cond, 2 | 3) {
            lvl |= CHANNEL_TRIGGER_MASKS[i];
        }
        if matches!(cond, 3 | 4) {
            edge |= CHANNEL_TRIGGER_MASKS[i];
        }
    }
    let rep = u8::from(en != 0);
    let div = (1_000_000 / samplerate - 1) as u8;
    let pos = TRIGGER_POSITIONS[ratio_idx.min(TRIGGER_POSITIONS.len() - 1)];
    let mut p = [CMD_PAD; PACKET];
    let head = [CMD_SETUP, 0x01, en, lvl, edge, rep, (pos & 0xff) as u8, (pos >> 8) as u8, div];
    p[..head.len()].copy_from_slice(&head);
    p
}

/// Converts the raw dump (two samples per byte, newest first) to one
/// sample per byte with pin4 in bit 0, pin5 in bit 1 and pin6 in bit 2.
pub fn convert_samples(raw: &[u8]) -> Vec<u8> {
    let nibble = |n: u8| {
        let mut b = 0u8;
        if n & 0x04 != 0 {
            b |= 0x01;
        }
        if n & 0x08 != 0 {
            b |= 0x02;
        }
        if n & 0x01 != 0 {
            b |= 0x04;
        }
        b
    };
    let mut out = Vec::with_capacity(raw.len() * 2);
    for b in raw.iter().rev() {
        out.push(nibble(b & 0x0f));
        out.push(nibble(b >> 4));
    }
    out
}

fn padded(bytes: &[u8]) -> [u8; PACKET] {
    let mut p = [CMD_PAD; PACKET];
    p[..bytes.len()].copy_from_slice(bytes);
    p
}

/// Sends one packet. As in libsigrok (`pickit2_usb_send_recv`), a
/// failed or short send is logged and otherwise ignored: the answer
/// the driver then waits for, or does not get, decides.
fn send(dev: &mut dyn Link, packet: &[u8; PACKET]) {
    if let Err(e) = dev.send(packet) {
        log::error!("USB transmit error: {e}");
    }
}

/// Reads one 64 byte packet, waiting for it.
fn recv(dev: &mut dyn Link, stop: &AtomicBool) -> Result<Option<[u8; PACKET]>> {
    let mut buf = [0u8; crate::hid::MAX_REPORT];
    loop {
        if stop.load(Ordering::SeqCst) {
            return Ok(None);
        }
        let n = match dev.recv(&mut buf, TIMEOUT) {
            Ok(n) => n,
            // libsigrok logs a failed read (a STALL, say) and reads
            // again; a vanished device ends the capture here, where
            // libsigrok would poll it for ever.
            Err(e) if !gone(&e) => {
                log::error!("USB receive error: {e}.");
                continue;
            }
            Err(e) => return Err(e),
        };
        if n == PACKET {
            let mut p = [0u8; PACKET];
            p.copy_from_slice(&buf[..PACKET]);
            return Ok(Some(p));
        }
        // libsigrok logs a short packet and keeps reading.
        if n > 0 {
            log::error!("USB short recv: {n}/{PACKET} bytes.");
        }
    }
}

/// Whether a transport error says the device has left the bus.
fn gone(e: &Error) -> bool {
    matches!(e, Error::Disconnected)
}

/// Reads the 512 raw sample bytes: two banks, two halves each, two
/// READ packets per half.
fn retrieve(dev: &mut dyn Link, stop: &AtomicBool) -> Result<Option<Vec<u8>>> {
    let mut raw = Vec::with_capacity(RAW_LEN);
    for bank in 0..2u8 {
        for offset in [0x00u8, 0x80] {
            send(dev, &padded(&[CMD_SETPOS, offset, SEL_BANK0 + bank]));
            for _ in 0..2 {
                send(dev, &padded(&[CMD_READ]));
                match recv(dev, stop)? {
                    Some(p) => raw.extend_from_slice(&p),
                    None => return Ok(None),
                }
            }
        }
    }
    Ok(Some(raw))
}

impl Device for Pickit2Device {
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
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        ch.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        name.clone_into(&mut ch.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some("Logic") => Ok(Vec::new()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        Ok(match key {
            Key::Conn => Value::Str(self.conn.clone()),
            Key::Samplerate => Value::U64(SAMPLERATES[self.settings.samplerate_idx]),
            Key::LimitSamples => Value::U64(self.settings.limit_samples),
            Key::CaptureRatio => Value::U64(CAPTURE_RATIOS[self.settings.capture_ratio_idx]),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::Samplerate => {
                let v = value.as_u64().ok_or_else(bad)?;
                self.settings.samplerate_idx =
                    SAMPLERATES.iter().position(|r| *r == v).ok_or_else(bad)?;
            }
            Key::CaptureRatio => {
                let v = value.as_u64().ok_or_else(bad)?;
                // Unknown values are ignored, as in libsigrok.
                if let Some(i) = CAPTURE_RATIOS.iter().position(|r| *r == v) {
                    self.settings.capture_ratio_idx = i;
                }
            }
            Key::LimitSamples => self.settings.limit_samples = value.as_u64().ok_or_else(bad)?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Ok(match key {
            Key::Samplerate => ValueList::U64(SAMPLERATES.to_vec()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::CaptureRatio => ValueList::U64(CAPTURE_RATIOS.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn trigger_features(&self) -> TriggerFeatures {
        TriggerFeatures { max_stages: 1, ..TriggerFeatures::default() }
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            if t.stages.len() > 1 {
                return Err(Error::not_supported("the PICkit2 takes one trigger stage"));
            }
            for c in t.stages.iter().flat_map(|s| &s.conditions) {
                if c.channel >= CHANNEL_COUNT || !TRIGGER_MATCHES.contains(&c.matches) {
                    return Err(Error::not_supported("trigger condition"));
                }
            }
        }
        self.trigger = trigger;
        Ok(())
    }

    fn open(&mut self) -> Result<()> {
        if self.dev.lock().expect("dev lock").is_some() {
            return Ok(());
        }
        let dev: Box<dyn Link> = match self.pending.take() {
            Some(d) => d,
            None => match self.source.as_ref().ok_or_else(|| Error::device("no device to open"))? {
                Source::Hid(info, _) => Box::new(HidLink(info.open()?)),
                Source::Usb(info) => Box::new(UsbLink::claim(crate::usb::open(info)?)?),
            },
        };
        *self.dev.lock().expect("dev lock") = Some(dev);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(mut dev) = self.dev.lock().expect("dev lock").take() {
            dev.close()?;
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let dev = self.dev.lock().expect("dev lock").take();
        let Some(dev) = dev else {
            return Err(Error::device("device is not open"));
        };
        let mut conditions = [0u8; CHANNEL_COUNT];
        for c in self.trigger.iter().flat_map(|t| &t.stages).flat_map(|s| &s.conditions) {
            if !self.channels[c.channel].enabled {
                continue;
            }
            conditions[c.channel] = match c.matches {
                TriggerMatch::Zero => 1,
                TriggerMatch::One => 2,
                TriggerMatch::Rising => 3,
                TriggerMatch::Falling => 4,
                _ => 0,
            };
        }
        let ratio_idx = if self.trigger.is_some() { self.settings.capture_ratio_idx } else { 0 };
        let setup = setup_packet(conditions, ratio_idx, SAMPLERATES[self.settings.samplerate_idx]);
        let limit = self.settings.limit_samples;
        let slot = Arc::clone(&self.dev);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("microchip-pickit2 acquisition".to_owned())
            .spawn(move || {
                let mut dev = dev;
                let outcome = run(dev.as_mut(), &sink, &stop2, &setup, limit);
                if let Err(e) = &outcome {
                    log::error!("Acquisition failed: {e}.");
                }
                sink.end(outcome);
                *slot.lock().expect("dev lock") = Some(dev);
            })
            .map_err(Error::Io)?;
        self.worker = Some((stop, handle));
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some((stop, handle)) = self.worker.take() {
            stop.store(true, Ordering::SeqCst);
            let _ = handle.join();
        }
        Ok(())
    }
}

/// One capture: SETUP, wait for the status (the trigger, or a cancel
/// with the device's button), read the banks, send the samples.
fn run(
    dev: &mut dyn Link,
    sink: &PacketSink,
    stop: &AtomicBool,
    setup: &[u8; PACKET],
    limit: u64,
) -> Result<()> {
    send(dev, setup);
    sink.send(Packet::header());
    let Some(status) = recv(dev, stop)? else {
        log::warn!("Cannot terminate by software, need either data trigger or cancel button.");
        return Ok(());
    };
    let status = u16::from_le_bytes([status[0], status[1]]);
    if status & 0x4004 == 0x4004 {
        log::info!("User cancelled acquisition.");
        return Ok(());
    }
    let Some(raw) = retrieve(dev, stop)? else {
        return Ok(());
    };
    let mut samples = convert_samples(&raw);
    if limit > 0 && (limit as usize) < samples.len() {
        samples.truncate(limit as usize);
    }
    sink.send(Packet::logic(1, samples));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::session::Flow;
    use crate::core::trigger::TriggerCondition;
    use crate::core::{Session, TriggerStage};
    use crate::hid::SharedFake;

    /// A capture against a scripted device: the status report, then
    /// the eight READ answers with a counter pattern.
    #[test]
    fn capture_through_the_session() {
        let nibble = |k: usize| {
            let mut n = 0u8;
            if k & 1 != 0 {
                n |= 0x04;
            }
            if k & 2 != 0 {
                n |= 0x08;
            }
            if k & 4 != 0 {
                n |= 0x01;
            }
            n
        };
        let mut raw = vec![0u8; RAW_LEN];
        for (i, b) in raw.iter_mut().enumerate() {
            let j = RAW_LEN - 1 - i;
            *b = nibble(2 * j) | nibble(2 * j + 1) << 4;
        }
        let fake = SharedFake::default();
        fake.push_input(&[0u8; PACKET]);
        for chunk in raw.chunks(PACKET) {
            fake.push_input(chunk);
        }
        let mut dev = Pickit2Device::with_device(Some(Box::new(fake.clone())), "04d8.0033".into());
        dev.open().unwrap();
        dev.set(&Key::Samplerate, &Value::U64(250_000), None).unwrap();
        dev.set(&Key::CaptureRatio, &Value::U64(2), None).unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(12), None).unwrap();
        let mut trigger = Trigger::default();
        trigger.stages.push(TriggerStage {
            conditions: vec![TriggerCondition {
                channel: 1,
                matches: TriggerMatch::Rising,
                value: 0.0,
            }],
            ..Default::default()
        });
        let mut session = Session::new();
        session.add_device(Box::new(dev));
        session.set_trigger(Some(trigger));
        let mut samples = Vec::new();
        session
            .run(|_, p| {
                if let Packet::Logic(l) = p {
                    samples.extend_from_slice(&l.data);
                }
                Flow::Continue
            })
            .unwrap();
        assert_eq!(samples, (0..12u8).map(|k| k & 7).collect::<Vec<_>>());
        let out = fake.output();
        // SETUP: rising on pin5 (0x08), position preset 2, divider 3.
        assert_eq!(&out[0][..10], &[0, CMD_SETUP, 1, 0x08, 0x08, 0x08, 1, 0x0a, 0x00, 3]);
        assert_eq!(&out[1][..4], &[0, CMD_SETPOS, 0x00, SEL_BANK0]);
        assert_eq!(out[2][1], CMD_READ);
        assert_eq!(out.len(), 1 + 4 * 3);
    }

    /// The same capture over raw USB, as on Linux: the interface is
    /// claimed, every packet is a 64 byte interrupt transfer on
    /// endpoint 1, a timed out IN while the trigger is pending is
    /// polled again, and close releases the interface.
    #[test]
    fn capture_over_usb_interrupt_transfers() {
        use crate::usb::record::{Replay, hex};
        use std::fmt::Write as _;
        let pad = |b: &[u8]| hex(&padded(b));
        let mut text = String::from(
            "# oxrok usb fixture 1\ndevice vid=04d8 pid=0033 bus=1 addr=5\nclaim 0 -> ok\n",
        );
        let _ = writeln!(text, "int_out 01 {} -> 64", pad(&[CMD_SETUP, 1, 0, 0, 0, 0, 0, 0, 0]));
        text += "int_in 81 64 -> err timeout\n";
        let _ = writeln!(text, "int_in 81 64 -> {}", hex(&[0u8; PACKET]));
        for (bank, offset) in [(0u8, 0u8), (0, 0x80), (1, 0), (1, 0x80)] {
            let _ =
                writeln!(text, "int_out 01 {} -> 64", pad(&[CMD_SETPOS, offset, SEL_BANK0 + bank]));
            for _ in 0..2 {
                let _ = writeln!(text, "int_out 01 {} -> 64", pad(&[CMD_READ]));
                let _ = writeln!(text, "int_in 81 64 -> {}", hex(&[0x44u8; PACKET]));
            }
        }
        text += "release 0 -> ok\n";
        let replay = Replay::from_text(&text).unwrap();
        let link = UsbLink::claim(Box::new(replay)).unwrap();
        let mut dev = Pickit2Device::with_link(Some(Box::new(link)), "1.5".into());
        dev.open().unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        let mut samples = Vec::new();
        for (_, p) in rx {
            match p {
                Packet::Logic(l) => samples.extend_from_slice(&l.data),
                Packet::End => break,
                _ => {}
            }
        }
        // 0x44: pin4 in both nibbles. A call off the recording would
        // have failed the capture before its samples.
        assert_eq!(samples, [1, 1, 1, 1]);
        // The release is the recording's last step: the replay took
        // every other one before it.
        dev.close().unwrap();
    }

    /// A scripted link: sends fail as listed, reads answer in turn.
    struct FlakyLink {
        send_fails: Vec<bool>,
        sent: usize,
        answers: std::collections::VecDeque<Vec<u8>>,
        /// Reads that fail before the answers, with these errors.
        errors: std::collections::VecDeque<Error>,
    }

    impl Link for FlakyLink {
        fn send(&mut self, _packet: &[u8; PACKET]) -> Result<()> {
            let fail = self.send_fails.get(self.sent).copied().unwrap_or(false);
            self.sent += 1;
            if fail { Err(Error::device("USB short send: 0/64 bytes.")) } else { Ok(()) }
        }

        fn recv(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<usize> {
            if let Some(e) = self.errors.pop_front() {
                return Err(e);
            }
            let a = self.answers.pop_front().unwrap_or_default();
            buf[..a.len()].copy_from_slice(&a);
            Ok(a.len())
        }
    }

    /// As in libsigrok, failed sends (here the SETUP and a READ) are
    /// logged and ignored, a failed read is read again, and a short
    /// packet is read past: the capture
    /// goes on with the answers that come.
    #[test]
    fn send_errors_and_short_reads_do_not_end_a_capture() {
        let mut answers = std::collections::VecDeque::new();
        answers.push_back(vec![0u8; 3]); // short: read again
        answers.push_back(vec![0u8; PACKET]); // the status
        for _ in 0..8 {
            answers.push_back(vec![0x44u8; PACKET]);
        }
        let mut link = FlakyLink {
            send_fails: vec![true, false, true],
            sent: 0,
            answers,
            errors: [Error::device("endpoint stalled")].into(),
        };
        let (tx, rx) = std::sync::mpsc::sync_channel(8);
        let sink = PacketSink::new(tx, 0);
        let setup = setup_packet([0, 0, 0], 0, 1_000_000);
        run(&mut link, &sink, &AtomicBool::new(false), &setup, 2).unwrap();
        drop(sink);
        let samples: Vec<u8> = rx
            .into_iter()
            .filter_map(|(_, p)| match p {
                Packet::Logic(l) => Some(l.data),
                _ => None,
            })
            .flatten()
            .collect();
        assert_eq!(samples, [1, 1]);
        assert_eq!(link.sent, 1 + 4 * 3);
    }

    /// A device that left the bus ends the capture, where libsigrok
    /// reads it for ever.
    #[test]
    fn a_vanished_device_ends_a_capture() {
        let mut link = FlakyLink {
            send_fails: Vec::new(),
            sent: 0,
            answers: std::collections::VecDeque::new(),
            errors: [Error::device("endpoint stalled"), Error::Disconnected].into(),
        };
        let (tx, _rx) = std::sync::mpsc::sync_channel(8);
        let sink = PacketSink::new(tx, 0);
        let setup = setup_packet([0, 0, 0], 0, 1_000_000);
        let e = run(&mut link, &sink, &AtomicBool::new(false), &setup, 2).unwrap_err();
        assert!(gone(&e), "{e}");
        assert!(link.errors.is_empty());
    }

    #[test]
    fn setup_and_conversion() {
        let p = setup_packet([3, 0, 1], 1, 250_000);
        // Rising on pin4 (enable, level and edge bits), zero on pin6.
        assert_eq!(&p[..9], &[CMD_SETUP, 1, 0x04 | 0x10, 0x04, 0x04, 1, 0xcc, 0x03, 3]);
        assert_eq!(p[9], CMD_PAD);
        let p = setup_packet([0, 0, 0], 0, 1_000_000);
        assert_eq!(&p[2..9], &[0, 0, 0, 0, 0, 0, 0]);
        // The last raw byte holds the first two samples; the low nibble
        // comes first.
        let mut raw = vec![0u8; RAW_LEN];
        raw[RAW_LEN - 1] = 0x8c | 0x01; // low: pin4+pin5+pin6, high: pin5
        let s = convert_samples(&raw);
        assert_eq!(s.len(), SAMPLE_COUNT);
        assert_eq!(s[0], 0x07);
        assert_eq!(s[1], 0x02);
        assert_eq!(s[2], 0);
    }
}
