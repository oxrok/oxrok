// Counterpart in sigrok:
//   libsigrok/src/hardware/ikalogic-scanalogic2/api.c,
//   Copyright (C) 2013 Marc Schink <sigrok-dev@marcschink.de>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Soeren Apel, Martin Ling, Daniel Elstner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! ikalogic-scanalogic2: the IKALOGIC Scanalogic-2, four channels at up to
//! 20 MHz with 32 KiB of sample memory per channel, a low speed HID device
//! that does everything through feature reports ([`protocol`]).
//!
//! It reaches the device as the C driver does, with plain control
//! transfers on a claimed interface, not through hidapi: the scan and the
//! open detach usbhid if it is bound, and it stays detached after oxrok
//! ends ([`crate::usb::Transport::detach_kernel_driver`]), as libusb leaves
//! it. Hidraw would serve the reports, but usbhid's own requests then come
//! between them.
//!
//! The scan resets the device, reads its info report, resets it and leaves
//! it idle; the open resets it and leaves it idle; a capture asks for the
//! samples and reads them all back, one channel after the other, then
//! resets it. Settings are read when a capture starts, so a change during
//! one counts from the next. A limit is honoured on the host, as the device
//! samples whole bytes. All of it as under sigrok-cli, checked with the
//! emulator (`tools/emu/ikalogic-scanalogic2.sh`); never on a board.
//!
//! The differences from the C driver:
//! - A two stage trigger is refused, where the C logs the error, drops it
//!   and captures with no trigger (api.c:342).
//! - A report too short for what is used out of it ends the capture, where
//!   the C ignores `actual_length` and uses what its buffer held
//!   (protocol.c:245-375).
//! - Of a trigger on several channels the device takes only the last, as in
//!   C (protocol.c:495-513), but with a warning.
//! - An info report that does not start `0a` gets a warning
//!   (protocol.c:674-678) and is otherwise used, as in C.
//! - The serial is unsigned, where the C prints it with `%d` (api.c:110).
//! - A short info report says so, where the C names its length as a libusb
//!   error (protocol.c:668).
//! - A capture with no limit, which only a library client can start, ends
//!   where the C divides by zero (protocol.c:317).
//! - A device that does not get ready after a capture ends it, after a
//!   hundred status reads, where the C reads and resets for ever.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Trigger, TriggerMatch, Value, ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, UsbId};
use protocol::{NUM_CHANNELS, Plan};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

const DRIVER: &str = "ikalogic-scanalogic2";
/// `USB_VID_PID`.
pub const USB_ID: UsbId = UsbId { vendor: 0x20a0, product: 0x4123 };
const DEFAULT_SAMPLERATE: u64 = 1_250;

/// The driver.
pub struct Scanalogic2Driver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::LimitSamples, Access { get: false, set: true, list: true }),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
];
static TRIGGER_MATCHES: [TriggerMatch; 3] =
    [TriggerMatch::Rising, TriggerMatch::Falling, TriggerMatch::Edge];

impl Driver for Scanalogic2Driver {
    fn id(&self) -> &'static str {
        DRIVER
    }

    fn long_name(&self) -> &'static str {
        "IKALOGIC Scanalogic-2"
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    /// `scan()`. A device whose info report cannot be read is no device.
    fn scan(&self, _options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for usb in crate::usb::find_devices(&[USB_ID])? {
            match read_info(&usb) {
                Ok(info) => devices.push(Box::new(Scanalogic2Device::new(usb, info, None))),
                Err(_) => log::warn!("Failed to get device information: -1."),
            }
        }
        Ok(devices)
    }
}

/// `sl2_get_device_info()`, with its open.
fn read_info(usb: &UsbDeviceInfo) -> Result<protocol::DeviceInfo> {
    let mut t = crate::usb::open(usb).inspect_err(|e| {
        log::error!("Failed to open device: {}.", crate::usb::libusb_error_name(e));
    })?;
    protocol::get_device_info(t.as_mut())
}

/// One Scanalogic-2.
pub struct Scanalogic2Device {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    samplerate: u64,
    limit_samples: u64,
    capture_ratio: u64,
    trigger: Option<Trigger>,
    wait: Duration,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Box<dyn Transport>>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl Scanalogic2Device {
    fn new(
        usb: UsbDeviceInfo,
        info: protocol::DeviceInfo,
        transport: Option<Box<dyn Transport>>,
    ) -> Self {
        Scanalogic2Device {
            info: DeviceInfo {
                driver: DRIVER,
                vendor: "IKALOGIC".into(),
                model: "Scanalogic-2".into(),
                version: format!("{}.{}", info.major, info.minor),
                serial: info.serial.to_string(),
                // The driver has no conn key.
                conn: None,
            },
            usb,
            channels: (0..NUM_CHANNELS)
                .map(|i| Channel::new(i, ChannelKind::Logic, i.to_string()))
                .collect(),
            samplerate: DEFAULT_SAMPLERATE,
            limit_samples: 0,
            capture_ratio: 0,
            trigger: None,
            wait: protocol::WAIT_DATA_READY,
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording), the
    /// scan's report given.
    pub fn from_transport(transport: Box<dyn Transport>, info: protocol::DeviceInfo) -> Self {
        let usb = transport.info().clone();
        Self::new(usb, info, Some(transport))
    }

    fn enabled(&self) -> [bool; NUM_CHANNELS] {
        let mut enabled = [false; NUM_CHANNELS];
        for c in &self.channels {
            enabled[c.index] = c.enabled;
        }
        enabled
    }

    /// `dev_acquisition_start()`'s numbers.
    fn plan(&self) -> Plan {
        let enabled = self.enabled();
        let trigger = protocol::convert_trigger(self.trigger.as_ref(), &enabled);
        if trigger.ignored > 0 {
            log::warn!(
                "The device triggers on one channel: only the last match counts, on channel {}.",
                trigger.channel - 1
            );
        }
        log::debug!(
            "Trigger set to channel {:#04x} and type {:#04x}.",
            trigger.channel,
            trigger.kind
        );
        let channels: Vec<u8> = (0..NUM_CHANNELS as u8).filter(|c| enabled[*c as usize]).collect();
        log::debug!("Number of enabled channels: {}.", channels.len());
        let rate = protocol::samplerate_code(self.samplerate).unwrap_or(0);
        let plan = Plan::new(self.limit_samples, self.capture_ratio, rate, trigger, channels);
        log::info!("Pre trigger samples: {}.", plan.pre_samples);
        log::debug!("Pre trigger sample bytes: {}.", plan.pre_bytes);
        log::debug!("Post trigger sample bytes: {}.", plan.post_bytes);
        plan
    }
}

impl Device for Scanalogic2Device {
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
        Ok(match key {
            Key::Samplerate => Value::U64(self.samplerate),
            Key::CaptureRatio => Value::U64(self.capture_ratio),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => {
                // sl2_set_limit_samples(). libsigrok's core refuses 0 first
                // (hwdriver.c).
                let limit = value.as_u64().ok_or_else(bad)?;
                if limit == 0 {
                    return Err(Error::Arg(format!("Invalid number of limit samples: {limit}.")));
                }
                self.limit_samples = limit.min(protocol::MAX_SAMPLES);
                log::debug!("Limit samples set to {}.", self.limit_samples);
            }
            Key::Samplerate => {
                let rate = value.as_u64().ok_or_else(bad)?;
                if protocol::samplerate_code(rate).is_none() {
                    return Err(Error::Arg(format!("no rate of {rate} Hz")));
                }
                self.samplerate = rate;
            }
            Key::CaptureRatio => {
                // sr_config_set() refuses it before the driver sees it
                // (hwdriver.c:757-764).
                let ratio = value.as_u64().ok_or_else(bad)?;
                if ratio > 100 {
                    return Err(Error::arg("Capture ratio must be 0..100."));
                }
                self.capture_ratio = ratio;
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
            Key::Samplerate => ValueList::U64(protocol::SAMPLERATES.to_vec()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::LimitSamples => ValueList::MinMaxU64(0, protocol::MAX_SAMPLES),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `dev_open()`.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let mut t = match self.pending.take() {
            Some(t) => t,
            None => crate::usb::open(&self.usb).inspect_err(|e| {
                log::error!("Failed to open device: {}.", crate::usb::libusb_error_name(e));
            })?,
        };
        protocol::open(t.as_mut())?;
        *self.link.lock().expect("link lock") = Some(t);
        Ok(())
    }

    /// `dev_close()`, after the thread that [`Device::stop`] only asks to
    /// end. A release that fails has nothing left to do, as in the C.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(w) = self.worker.take() {
            let _ = w.handle.join();
        }
        if let Some(mut t) = self.link.lock().expect("link lock").take() {
            if let Err(e) = t.release_interface(protocol::INTERFACE) {
                log::debug!("Releasing the interface: {e}.");
            }
        }
        Ok(())
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&self.trigger_features())?;
        }
        self.trigger = trigger;
        Ok(())
    }

    /// [`protocol::check_trigger`], and a capture with no channel enabled,
    /// where the C reads `channel_map[-1]` (protocol.c:302).
    fn check_capture(&self, trigger: Option<&Trigger>) -> Result<()> {
        if let Some(t) = trigger {
            protocol::check_trigger(t)?;
        }
        if !self.channels.iter().any(|c| c.enabled) {
            return Err(Error::arg("No channel is enabled."));
        }
        Ok(())
    }

    /// `dev_acquisition_start()`.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(w) = self.worker.take() {
            // The thread hands the transport back before its last packet.
            if self.link.lock().expect("link lock").is_none() {
                self.worker = Some(w);
                return Err(Error::Busy);
            }
            let _ = w.handle.join();
        }
        self.check_capture(self.trigger.as_ref())?;
        let Some(mut t) = self.link.lock().expect("link lock").take() else {
            return Err(Error::device("device is not open"));
        };
        let plan = self.plan();
        let wait = self.wait;
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("ikalogic-scanalogic2 acquisition".into())
            .spawn(move || {
                protocol::run(t.as_mut(), plan, wait, &sink, &stop2);
                *slot.lock().expect("link lock") = Some(t);
                sink.send(Packet::End);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the capture to end, as `dev_acquisition_stop()`: the thread may
    /// have packets to send, so [`Device::close`] joins it, not this.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;
    use std::fmt::Write as _;
    use std::sync::mpsc::sync_channel;

    const HEAD: &str = "# oxrok usb fixture 1\n\
        device vid=20a0 pid=4123 version=0100 bus=0 addr=0 strings=1,2,0\n";

    fn hex(bytes: &[u8]) -> String {
        let mut s = String::new();
        for b in bytes {
            let _ = write!(s, "{b:02x}");
        }
        s
    }

    fn out(bytes: &[u8; 128]) -> String {
        format!("ctl_out 21 09 0300 0000 {} -> ok\n", hex(bytes))
    }

    fn command(c: u8) -> [u8; 128] {
        let mut b = [0; 128];
        b[0] = c;
        b
    }

    fn input(bytes: &[u8]) -> String {
        format!("ctl_in 21 01 0300 0000 128 -> {}\n", hex(bytes))
    }

    fn status(code: u8) -> [u8; 128] {
        let mut b = [0; 128];
        b[0] = 5;
        b[1] = code;
        b
    }

    fn opened() -> String {
        format!("{HEAD}claim 0 -> ok\n{}{}", out(&command(2)), out(&command(7)))
    }

    const INFO: protocol::DeviceInfo =
        protocol::DeviceInfo { serial: 3_000_000_000, major: 1, minor: 3 };

    fn device(steps: &str) -> Scanalogic2Device {
        let text = format!("{}{steps}", opened());
        let mut d =
            Scanalogic2Device::from_transport(Box::new(Replay::from_text(&text).unwrap()), INFO);
        d.wait = Duration::from_millis(1);
        d
    }

    fn capture(dev: &mut Scanalogic2Device) -> Vec<Packet> {
        dev.open().unwrap();
        let (tx, rx) = sync_channel(16);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        dev.close().unwrap();
        packets
    }

    #[test]
    fn the_serial_is_unsigned() {
        let d = device("");
        assert_eq!(d.info().serial, "3000000000");
        assert_eq!(d.info().version, "1.3");
        assert_eq!(d.channels().len(), 4);
    }

    #[test]
    fn a_capture_of_one_packet_per_channel() {
        let plan = Plan::new(8, 0, 10, protocol::DeviceTrigger::NONE, vec![0]);
        let sample = plan.sample_command();
        let mut packet = [0xaa; 128];
        packet[..4].copy_from_slice(&[5, 0, 0, 0]);
        let mut reset = sample;
        reset[0] = protocol::CMD_RESET;
        let mut idle = sample;
        idle[0] = protocol::CMD_IDLE;
        let steps = format!(
            "{}{}{}{}{}{}{}{}{}release 0 -> ok\n",
            out(&sample),
            // Waits once, 1 ms here.
            input(&status(0x62)),
            input(&status(protocol::STATUS_DATA_READY)),
            input(&packet),
            input(&status(1)),
            input(&status(1)),
            out(&reset),
            input(&status(protocol::STATUS_DEVICE_READY)),
            out(&idle),
        );
        let mut d = device(&steps);
        d.enable_channel(1, false).unwrap();
        d.enable_channel(2, false).unwrap();
        d.enable_channel(3, false).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(8), None).unwrap();
        let packets = capture(&mut d);
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert_eq!(packets[1], Packet::logic(1, vec![1, 0, 1, 0, 1, 0, 1, 0]));
        assert_eq!(packets[2], Packet::End);
    }

    #[test]
    fn a_failed_transfer_ends_the_capture_with_no_more_requests() {
        let plan = Plan::new(8, 0, 10, protocol::DeviceTrigger::NONE, vec![0, 1, 2, 3]);
        let steps = format!(
            "{}ctl_in 21 01 0300 0000 128 -> err device \"endpoint stalled\"\nrelease 0 -> ok\n",
            out(&plan.sample_command())
        );
        let mut d = device(&steps);
        d.set(&Key::LimitSamples, &Value::U64(8), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(packets.len(), 2);
        assert_eq!(packets[1], Packet::End);
    }

    #[test]
    fn options_as_the_c_driver_lists_them() {
        let mut d = device("");
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(1250));
        assert_eq!(d.get(&Key::CaptureRatio, None).unwrap(), Value::U64(0));
        assert!(d.set(&Key::Samplerate, &Value::U64(3_000_000), None).is_err());
        d.set(&Key::Samplerate, &Value::U64(2_500_000), None).unwrap();
        assert_eq!(d.plan().rate_code, 3);
        assert!(d.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
        // Clamped, as sl2_set_limit_samples().
        d.set(&Key::LimitSamples, &Value::U64(1 << 40), None).unwrap();
        assert_eq!(d.limit_samples, protocol::MAX_SAMPLES);
        assert_eq!(
            d.list(&Key::LimitSamples, None).unwrap(),
            ValueList::MinMaxU64(0, protocol::MAX_SAMPLES)
        );
    }

    #[test]
    fn nothing_starts_with_a_two_stage_trigger_or_no_channel() {
        let mut d = device("");
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::Rising);
        assert!(d.check_capture(Some(&t)).is_ok());
        t.add(1, 0, TriggerMatch::Falling);
        assert!(d.check_capture(Some(&t)).is_err());
        for i in 0..4 {
            d.enable_channel(i, false).unwrap();
        }
        assert!(d.check_capture(None).is_err());
    }

    #[test]
    fn a_stop_ends_a_status_wait_without_its_interval() {
        let plan = Plan::new(8, 0, 10, protocol::DeviceTrigger::NONE, vec![0, 1, 2, 3]);
        let mut reset = plan.sample_command();
        reset[0] = protocol::CMD_RESET;
        let mut idle = plan.sample_command();
        idle[0] = protocol::CMD_IDLE;
        let steps = format!(
            "{}{}{}{}{}{}release 0 -> ok\n",
            out(&plan.sample_command()),
            input(&status(0x61)),
            input(&status(0x61)),
            out(&reset),
            input(&status(protocol::STATUS_DEVICE_READY)),
            out(&idle),
        );
        let mut d = device(&steps);
        d.wait = Duration::from_secs(3600);
        d.set(&Key::LimitSamples, &Value::U64(8), None).unwrap();
        d.open().unwrap();
        let (tx, rx) = sync_channel(16);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The replay answers at once, so by now it is in its long wait.
        assert!(matches!(rx.recv().unwrap().1, Packet::Header { .. }));
        std::thread::sleep(Duration::from_millis(500));
        d.stop().unwrap();
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        d.close().unwrap();
        assert_eq!(packets, [Packet::End]);
    }
}
