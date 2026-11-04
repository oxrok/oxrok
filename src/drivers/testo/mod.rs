// Counterpart in sigrok:
//   libsigrok/src/hardware/testo/api.c,
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Soeren Apel,
//   Daniel Elstner, Martin Ling, Romain Tartière
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! testo: the Testo 435/635/735 multifunction meter, an MCU behind an FT232R
//! (`128d:0003`) that libsigrok reaches with plain libusb, so this driver goes
//! through [`Transport`] and not through `crate::ftdi`.
//!
//! The scan looks at the strings of every USB device, as `scan()` does:
//! a manufacturer starting with `testo` and the product `testo 435/635/735`.
//! Which channels a device has depends on its probes, so the scan asks it for
//! one answer ([`protocol::probe_channels`]) and names a channel per
//! measurement.
//!
//! Stock libsigrok finds no Testo: `b258c09f` (2017) dropped the assignment
//! of `sdi->driver` from `scan()` as already done by `std_scan_complete()`,
//! but the probe runs before that and `sr_dev_open()` refuses a NULL driver
//! (`device.c:730`). The oracle this port is compared with has that line
//! restored (`tools/oracle-patches/libsigrok-testo-sdi-driver.patch`). So
//! no libsigrok since then has driven this device; the C's capture path is
//! weak evidence, and neither it nor this port has met one here.
//!
//! Unlike the C, this port: strips the chip's status bytes from every packet;
//! takes two measurements of a unit, and another order of them; asks again
//! after any damaged answer; ends a capture or a probe the device does not
//! answer; and finds nothing for a `conn=` that names nothing. See
//! [`protocol`] and `docs/DEVICES.md`.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    PacketSink, Result, Value, ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, libusb_error_name, split_conn};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

/// The start of the manufacturer string, `api.c:109`.
const MANUFACTURER: &str = "testo";
/// The product string, `api.c:116`: "Hardcode the 435 for now".
const PRODUCT: &str = "testo 435/635/735";

/// The driver.
pub struct TestoDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];

impl Driver for TestoDriver {
    fn id(&self) -> &'static str {
        "testo"
    }

    fn long_name(&self) -> &'static str {
        "Testo"
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
        // `sr_usb_find()` finding nothing leaves the C without a filter
        // (api.c:75): a conn that names nothing selects every device.
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
            let Ok(mut t) = crate::usb::open(&info) else { continue };
            if !is_testo(t.as_mut()) {
                continue;
            }
            drop(t);
            if let Some(device) = TestoDevice::probed(info) {
                devices.push(Box::new(device));
            }
        }
        Ok(devices)
    }
}

/// The strings of `scan()`, both read whatever the first says, and the check.
fn is_testo(t: &mut dyn Transport) -> bool {
    let [manufacturer, product, _] = t.string_indexes().unwrap_or_default();
    let mut read = |index: u8, what: &str| {
        if index == 0 {
            return String::new();
        }
        crate::usb::read_string_ascii(t, index).unwrap_or_else(|e| {
            log::warn!("Failed to get {what} string descriptor: {}.", libusb_error_name(&e));
            String::new()
        })
    };
    let manufacturer = read(manufacturer, "manufacturer");
    let product = read(product, "product");
    manufacturer.starts_with(MANUFACTURER) && product == PRODUCT
}

/// `dev_open()`'s claim. The transport detaches a kernel driver (`ftdi_sio`
/// knows this id) when the claim says busy.
fn claim(t: &mut dyn Transport) -> Result<()> {
    t.claim_interface(0).inspect_err(|e| {
        log::error!("Failed to claim interface: {}.", libusb_error_name(e));
    })
}

/// A running capture.
struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// A Testo 435.
pub struct TestoDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    units: Vec<u8>,
    limits: protocol::Limits,
    /// A transport handed in by [`TestoDevice::from_transport`], claimed.
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Box<dyn Transport>>>>,
    worker: Option<Worker>,
}

impl TestoDevice {
    /// Opens and probes a device; `None` for a device
    /// the probe fails on, which the C drops without a word of its own
    /// (`api.c:131`).
    fn probed(usb: UsbDeviceInfo) -> Option<TestoDevice> {
        let mut t = crate::usb::open(&usb).ok()?;
        claim(t.as_mut()).ok()?;
        let units = protocol::probe_channels(t.as_mut());
        let _ = t.release_interface(0);
        let units = units.inspect_err(|e| log::debug!("The probe failed: {e}.")).ok()?;
        Some(TestoDevice::new(usb, units))
    }

    fn new(usb: UsbDeviceInfo, units: Vec<u8>) -> TestoDevice {
        let channels = units
            .iter()
            .enumerate()
            .map(|(i, u)| {
                let name = protocol::unit_info(*u).map_or("", |info| info.0);
                Channel::new(i, ChannelKind::Analog, name)
            })
            .collect();
        TestoDevice {
            info: DeviceInfo {
                driver: "testo",
                vendor: "Testo".to_owned(),
                model: "435/635/735".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            usb,
            channels,
            units,
            limits: protocol::Limits::default(),
            pending: None,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an open transport (a replayed recording), probed as
    /// the scan does.
    ///
    /// # Errors
    ///
    /// When the probe fails.
    pub fn from_transport(mut transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        claim(transport.as_mut())?;
        let units = protocol::probe_channels(transport.as_mut())?;
        let mut device = Self::new(usb, units);
        device.pending = Some(transport);
        Ok(device)
    }

    /// Joins a capture that has ended, or waits for one that was stopped.
    fn join(&mut self) {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
    }
}

impl Device for TestoDevice {
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
            Key::LimitSamples => Value::U64(self.limits.samples),
            Key::LimitMsec => Value::U64(self.limits.msec),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => self.limits.samples = value.as_u64().ok_or_else(bad)?,
            Key::LimitMsec => self.limits.msec = value.as_u64().ok_or_else(bad)?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn open(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("testo lock");
        if link.is_some() {
            return Ok(());
        }
        if let Some(t) = self.pending.take() {
            *link = Some(t);
            return Ok(());
        }
        let mut t = crate::usb::open(&self.usb)?;
        claim(t.as_mut())?;
        *link = Some(t);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.join();
        if let Some(mut t) = self.link.lock().expect("testo lock").take() {
            let _ = t.release_interface(0);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.as_ref().is_some_and(|w| !w.handle.is_finished()) {
            return Err(Error::Busy);
        }
        self.join();
        let Some(mut t) = self.link.lock().expect("testo lock").take() else {
            return Err(Error::device("device is not open"));
        };
        if let Err(e) = protocol::set_serial_params(t.as_mut()) {
            *self.link.lock().expect("testo lock") = Some(t);
            return Err(e);
        }
        let (units, limits) = (self.units.clone(), self.limits);
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("testo acquisition".into())
            .spawn(move || {
                protocol::run(t.as_mut(), &units, limits, &sink, &stop2);
                *slot.lock().expect("testo lock") = Some(t);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the capture to end. `close` joins it: the thread may still be
    /// sending packets to whoever called this.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Drop for TestoDevice {
    fn drop(&mut self) {
        self.join();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Packet;
    use crate::usb::record::Replay;
    use std::sync::mpsc::sync_channel;

    const SCAN: &str = "# oxrok usb fixture 1\n\
        device vid=128d pid=0003 version=0600 bus=0 addr=0 strings=1,2,0\n\
        claim 0 -> ok\n\
        ctl_out 40 03 001a 0000 - -> ok\nctl_out 40 04 0008 0000 - -> ok\n\
        ctl_out 40 02 0008 0000 - -> ok\nctl_out 40 01 0303 0000 - -> ok\n\
        bulk_in 81 128 -> 0160\n\
        bulk_out 02 12000000010155d1b7 -> 9\n\
        bulk_in 81 128 -> 0160210000000100020000ac410100000000344203000000f6\n";
    const START: &str = "ctl_out 40 03 001a 0000 - -> ok\nctl_out 40 04 0008 0000 - -> ok\n\
        ctl_out 40 02 0008 0000 - -> ok\nctl_out 40 01 0303 0000 - -> ok\n\
        bulk_out 02 12000000010155d1b7 -> 9\n\
        bulk_in 81 128 -> 0160210000000100020000ae410100000000364203000033e2\n";

    fn device(steps: &str) -> TestoDevice {
        let t = Replay::from_text(steps).expect("the fixture parses");
        TestoDevice::from_transport(Box::new(t)).expect("the probe passes")
    }

    #[test]
    fn the_probe_names_the_channels() {
        let d = device(SCAN);
        let names: Vec<&str> = d.channels().iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["Temperature", "Humidity"]);
        assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(0));
        assert!(d.get(&Key::Conn, None).is_err(), "not in devopts");
    }

    #[test]
    fn a_capture_stops_at_its_limit_without_another_request() {
        let mut d = device(&format!("{SCAN}{START}"));
        d.open().unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let (tx, rx) = sync_channel(16);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let packets: Vec<Packet> = rx.iter().map(|p| p.1).collect();
        assert!(matches!(packets[0], Packet::Header { .. }));
        let Packet::Analog(a) = &packets[1] else { panic!("{:?}", packets[1]) };
        assert_eq!(
            (a.channels.as_slice(), a.samples.as_slice(), a.digits),
            (&[0][..], &[21.75][..], 2)
        );
        let Packet::Analog(a) = &packets[2] else { panic!("{:?}", packets[2]) };
        assert_eq!((a.channels.as_slice(), a.samples.as_slice()), (&[1][..], &[45.5][..]));
        assert_eq!(packets[3], Packet::End);
        assert_eq!(packets.len(), 4);
        d.close().unwrap();
    }

    #[test]
    fn a_failed_baud_rate_fails_the_start_with_no_packet() {
        let mut d = device(&format!("{SCAN}ctl_out 40 03 001a 0000 - -> err timeout\n"));
        d.open().unwrap();
        let (tx, rx) = sync_channel(16);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
        assert!(rx.iter().next().is_none());
    }

    #[test]
    fn the_limits_are_options() {
        let mut d = device(SCAN);
        d.set(&Key::LimitMsec, &Value::U64(150), None).unwrap();
        assert_eq!(d.get(&Key::LimitMsec, None).unwrap(), Value::U64(150));
        assert!(d.set(&Key::Samplerate, &Value::U64(1), None).is_err());
        assert_eq!(d.options(None).unwrap().len(), 3);
    }
}
