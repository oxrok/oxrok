// Counterpart in sigrok:
//   libsigrok/src/hardware/dcttech-usbrelay/api.c,
//   libsigrok/src/hardware/dcttech-usbrelay/protocol.c,
//   libsigrok/src/hardware/dcttech-usbrelay/protocol.h,
//   Copyright (C) 2021 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! The dcttech USB relay boards: one to eight relays behind a V-USB
//! HID device, switched with feature reports. Each relay is a channel
//! group `R1` to `Rn` with an `enabled` option; without a group,
//! `enabled` switches them all. The driver is `dcttech-usbrelay`.
//!
//! The USB id is shared with many V-USB projects, so the vendor and
//! product strings identify the board, and the board's serial number
//! comes from its report, which `conn=sn=SERIAL` filters on. `conn`
//! also takes `VID.PID` or the device node (`/dev/hidrawN` on Linux),
//! which is what the device reports as its connection where the
//! system has one, as libsigrok does.

#![forbid(unsafe_code)]

use crate::core::{
    Access, Channel, ChannelGroup, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Value, ValueList,
};
use crate::hid::HidDevice;

const VENDOR_ID: u16 = 0x16c0;
const PRODUCT_ID: u16 = 0x05df;
const VENDOR_STRING: &str = "www.dcttech.com";
const PRODUCT_PREFIX: &str = "USBRelay";
const REPORT_BYTES: usize = 8;
const SERIAL_LENGTH: usize = 5;
const STATE_INDEX: usize = 7;

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multiplexer];
static DEV_OPTIONS: [OptionDesc; 2] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Enabled, Access { get: false, set: true, list: false }),
];
static GROUP_OPTIONS: [OptionDesc; 1] = [OptionDesc::new(Key::Enabled, Access::GET_SET)];

/// The driver.
pub struct UsbRelayDriver;

impl Driver for UsbRelayDriver {
    fn id(&self) -> &'static str {
        "dcttech-usbrelay"
    }

    fn long_name(&self) -> &'static str {
        "dcttech usbrelay"
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
                Key::Conn => conn = value.as_str().map(str::to_owned).filter(|c| !c.is_empty()),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let mut want_serial = None;
        let mut want_id = None;
        let mut want_path = None;
        if let Some(c) = &conn {
            if let Some(sn) = c.strip_prefix("sn=") {
                want_serial = Some(sn.to_owned());
            } else if let Some((v, p)) = c.split_once('.')
                && let (Ok(vid), Ok(pid)) = (u16::from_str_radix(v, 16), u16::from_str_radix(p, 16))
            {
                want_id = Some((vid, pid));
            } else if c.starts_with('/') {
                want_path = Some(c.clone());
            } else {
                return Err(Error::Arg(format!(
                    "Invalid conn '{c}': use sn=SERIAL, VID.PID or a device path."
                )));
            }
        }
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::hid::list()? {
            if want_path.is_some() {
                if info.path != want_path {
                    continue;
                }
            } else {
                let (vid, pid) = want_id.unwrap_or((VENDOR_ID, PRODUCT_ID));
                if info.vendor_id != vid || info.product_id != pid {
                    continue;
                }
            }
            // As libsigrok: a device named explicitly is not checked
            // against the vendor and product strings.
            if want_id.is_none()
                && want_path.is_none()
                && (info.manufacturer.as_deref() != Some(VENDOR_STRING)
                    || !info.product.starts_with(PRODUCT_PREFIX))
            {
                continue;
            }
            let Some(count) = relay_count(&info.product) else { continue };
            let mut dev = match info.open() {
                Ok(d) => d,
                Err(e) => {
                    log::info!("Cannot open {}: {e}", info.product);
                    continue;
                }
            };
            let Ok((serial, state)) = read_state(dev.as_mut()) else {
                log::warn!("Skipping {}: no usable report.", info.product);
                continue;
            };
            if want_serial.as_deref().is_some_and(|s| s != serial) {
                continue;
            }
            let conn = info
                .path
                .clone()
                .unwrap_or_else(|| format!("{:04x}.{:04x}", info.vendor_id, info.product_id));
            let vendor = info.manufacturer.clone().unwrap_or_default();
            devices.push(Box::new(UsbRelayDevice::new(
                dev,
                vendor,
                info.product.clone(),
                serial,
                conn,
                count,
                state,
            )));
        }
        Ok(devices)
    }
}

/// The relay count from the trailing digits of the product string.
pub fn relay_count(product: &str) -> Option<usize> {
    let digits: String = product.chars().rev().take_while(char::is_ascii_digit).collect();
    let n: usize = digits.chars().rev().collect::<String>().parse().ok()?;
    (n > 0).then_some(n)
}

/// Reads the board's report: its serial number and the relay state.
fn read_state(dev: &mut dyn HidDevice) -> Result<(String, u8)> {
    let mut report = [0u8; 1 + REPORT_BYTES];
    let n = dev.get_feature(&mut report)?;
    if n != report.len() {
        return Err(Error::device(format!("Unexpected HID report length {n}.")));
    }
    let serial: String = report[1..=SERIAL_LENGTH].iter().map(|b| *b as char).collect();
    if serial.bytes().any(|b| !(0x20..=0x7e).contains(&b)) {
        return Err(Error::device("non-printable serial number in the report"));
    }
    Ok((serial, report[1 + STATE_INDEX]))
}

/// One relay board.
pub struct UsbRelayDevice {
    dev: Box<dyn HidDevice>,
    info: DeviceInfo,
    conn: String,
    groups: Vec<ChannelGroup>,
    count: usize,
    state: u8,
}

impl UsbRelayDevice {
    /// A board on an open HID device.
    pub fn new(
        dev: Box<dyn HidDevice>,
        vendor: String,
        product: String,
        serial: String,
        conn: String,
        count: usize,
        state: u8,
    ) -> UsbRelayDevice {
        UsbRelayDevice {
            dev,
            info: DeviceInfo {
                driver: "dcttech-usbrelay",
                vendor,
                model: product,
                version: String::new(),
                serial,
                conn: Some(conn.clone()),
            },
            conn,
            groups: (1..=count).map(|n| ChannelGroup::new(format!("R{n}"), Vec::new())).collect(),
            count,
            state,
        }
    }

    fn relay(&self, group: Option<&str>) -> Result<Option<usize>> {
        let Some(g) = group else { return Ok(None) };
        let n = self
            .groups
            .iter()
            .position(|cg| cg.name == g)
            .ok_or_else(|| Error::ChannelGroup(g.to_owned()))?;
        Ok(Some(n + 1))
    }

    fn update_state(&mut self) -> Result<()> {
        let (_, state) = read_state(self.dev.as_mut())?;
        self.state = state & ((1u16 << self.count) - 1) as u8;
        Ok(())
    }

    /// Switches one relay (`Some(number)`) or all of them.
    fn switch(&mut self, relay: Option<usize>, on: bool) -> Result<()> {
        let mut report = [0u8; 1 + REPORT_BYTES];
        match relay {
            None => report[1] = if on { 0xfe } else { 0xfc },
            Some(n) => {
                report[1] = if on { 0xff } else { 0xfd };
                report[2] = n as u8;
            }
        }
        // libsigrok reports a failed write as SR_ERR_IO, whatever the
        // cause.
        self.dev
            .set_feature(&report)
            .map_err(|_| Error::Io(std::io::Error::other("input/output error")))?;
        let _ = self.update_state();
        Ok(())
    }
}

impl Device for UsbRelayDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &[]
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &self.groups
    }

    fn enable_channel(&mut self, pos: usize, _enabled: bool) -> Result<()> {
        Err(Error::Arg(format!("No channel {pos}.")))
    }

    fn rename_channel(&mut self, pos: usize, _name: &str) -> Result<()> {
        Err(Error::Arg(format!("No channel {pos}.")))
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        match self.relay(group)? {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(_) => Ok(GROUP_OPTIONS.to_vec()),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        match (key, self.relay(group)?) {
            (Key::Conn, None) => Ok(Value::Str(self.conn.clone())),
            (Key::Enabled, Some(n)) => Ok(Value::Bool(self.state >> (n - 1) & 1 != 0)),
            (other, _) => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let relay = self.relay(group)?;
        match key {
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(|| Error::arg("enabled takes on or off"))?;
                self.switch(relay, on)
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn open(&mut self) -> Result<()> {
        // As in libsigrok, a board whose state cannot be read opens all
        // the same, with the state it had.
        let _ = self.update_state();
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        // Nothing to acquire; the session ends at once.
        sink.send(Packet::header());
        sink.end(Ok(()));
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hid::SharedFake;

    fn report(state: u8) -> Vec<u8> {
        let mut r = vec![0u8; 9];
        r[1..6].copy_from_slice(b"ABCDE");
        r[8] = state;
        r
    }

    #[test]
    fn counts_and_switches() {
        assert_eq!(relay_count("USBRelay2"), Some(2));
        assert_eq!(relay_count("USBRelay8"), Some(8));
        assert_eq!(relay_count("USBRelay"), None);
        let fake = SharedFake::default();
        {
            let mut f = fake.0.lock().unwrap();
            f.feature_answers.push_back(report(0x01));
            f.feature_answers.push_back(report(0x03));
            f.feature_answers.push_back(report(0x00));
        }
        let (serial, state) = read_state(&mut fake.clone()).unwrap();
        assert_eq!((serial.as_str(), state), ("ABCDE", 1));
        let mut dev = UsbRelayDevice::new(
            Box::new(fake.clone()),
            VENDOR_STRING.into(),
            "USBRelay2".into(),
            serial,
            "16c0.05df".into(),
            2,
            state,
        );
        assert_eq!(dev.channel_groups().len(), 2);
        assert_eq!(dev.get(&Key::Enabled, Some("R1")).unwrap(), Value::Bool(true));
        assert_eq!(dev.get(&Key::Enabled, Some("R2")).unwrap(), Value::Bool(false));
        dev.set(&Key::Enabled, &Value::Bool(true), Some("R2")).unwrap();
        assert_eq!(dev.get(&Key::Enabled, Some("R2")).unwrap(), Value::Bool(true));
        dev.set(&Key::Enabled, &Value::Bool(false), None).unwrap();
        assert_eq!(dev.get(&Key::Enabled, Some("R1")).unwrap(), Value::Bool(false));
        let sent = fake.features();
        assert_eq!(sent[0][..3], [0, 0xff, 2]);
        assert_eq!(sent[1][..2], [0, 0xfc]);
        assert!(dev.get(&Key::Enabled, Some("R3")).is_err());
    }

    /// A board that refuses every request after the scan.
    struct Refusing;

    impl HidDevice for Refusing {
        fn read_report(&mut self, _buf: &mut [u8], _timeout: std::time::Duration) -> Result<usize> {
            Err(Error::device("refused"))
        }

        fn write_report(&mut self, _data: &[u8]) -> Result<()> {
            Err(Error::device("refused"))
        }

        fn get_feature(&mut self, _buf: &mut [u8]) -> Result<usize> {
            Err(Error::device("hid: ioctl(GET_FEATURE) error: Broken pipe"))
        }

        fn set_feature(&mut self, _data: &[u8]) -> Result<()> {
            Err(Error::device("hid: ioctl(SET_FEATURE) error: Broken pipe"))
        }
    }

    /// As in libsigrok: a failed state read does not stop the open,
    /// and a failed switch is an I/O error.
    #[test]
    fn refused_requests() {
        let mut dev = UsbRelayDevice::new(
            Box::new(Refusing),
            VENDOR_STRING.into(),
            "USBRelay2".into(),
            "ABCDE".into(),
            "/dev/hidraw0".into(),
            2,
            0,
        );
        dev.open().unwrap();
        assert_eq!(dev.get(&Key::Enabled, Some("R1")).unwrap(), Value::Bool(false));
        let e = dev.set(&Key::Enabled, &Value::Bool(true), Some("R2")).unwrap_err();
        assert_eq!(e.to_string(), "input/output error");
    }
}
