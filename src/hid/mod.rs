// Counterpart in sigrok:
//   libsigrok/src/serial_hid.c (the hidapi calls),
//   Copyright (C) 2017-2019 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! USB HID transport: list the HID devices on the machine, open one,
//! and exchange reports with it.
//!
//! The interface is blocking, like the serial and USB transports, and
//! follows the hidapi conventions that the chip adapters in
//! the `serial` module were written against: the first byte of an outgoing
//! report is its report id, 0 when the device does not number its
//! reports, and an incoming report carries its id in the first byte
//! only when the device numbers them.
//!
//! The backend is `async-hid`, which talks to `IOHIDManager` on macOS,
//! hidraw on Linux and the Win32 HID API on Windows without any C code.

#![forbid(unsafe_code)]

pub mod record;

use crate::core::{Error, Result};
use async_hid::{AsyncHidFeatureHandle, AsyncHidRead, AsyncHidWrite, HidBackend};
use futures_lite::future::{block_on, or};
use futures_lite::stream::StreamExt;
use std::time::Duration;

/// The largest report any supported device sends.
pub const MAX_REPORT: usize = 64;

/// An open HID device.
pub trait HidDevice: Send {
    /// Reads one input report into `buf`, waiting at most `timeout`.
    /// Returns its length, or 0 when none arrived in time.
    fn read_report(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize>;
    /// Sends one output report; `data[0]` is the report id.
    fn write_report(&mut self, data: &[u8]) -> Result<()>;
    /// Reads a feature report; `buf[0]` holds the report id to ask for.
    fn get_feature(&mut self, buf: &mut [u8]) -> Result<usize>;
    /// Sends a feature report; `data[0]` is the report id.
    fn set_feature(&mut self, data: &[u8]) -> Result<()>;
}

/// A HID device on the machine.
#[derive(Debug)]
pub struct HidInfo {
    /// USB vendor id.
    pub vendor_id: u16,
    /// USB product id.
    pub product_id: u16,
    /// Product string.
    pub product: String,
    /// Manufacturer string, when the system reports it.
    pub manufacturer: Option<String>,
    /// Serial number, when the device has one.
    pub serial: Option<String>,
    /// The device node, where the system has one (`/dev/hidrawN` on
    /// Linux); what libsigrok reports as the connection.
    pub path: Option<String>,
    /// The backend's device; none for a device of a replay file.
    device: Option<async_hid::Device>,
}

/// On Linux the kernel's HID name joins manufacturer and product
/// (`www.dcttech.com USBRelay2`); hidapi, which libsigrok uses, reports
/// the USB device's own product string instead, read from sysfs. This
/// returns the device node and that product string.
#[cfg(target_os = "linux")]
fn linux_details(id: &async_hid::DeviceId) -> (Option<String>, Option<String>) {
    let async_hid::DeviceId::DevPath(sysfs) = id else { return (None, None) };
    let node = sysfs.file_name().map(|n| format!("/dev/{}", n.to_string_lossy()));
    let mut dir = sysfs.join("device");
    let mut product = None;
    for _ in 0..6 {
        if let Ok(p) = std::fs::read_to_string(dir.join("product"))
            && !p.trim().is_empty()
        {
            product = Some(p.trim().to_owned());
            break;
        }
        match dir.parent() {
            Some(parent) => dir = parent.to_path_buf(),
            None => break,
        }
    }
    (node, product)
}

#[cfg(not(target_os = "linux"))]
fn linux_details(_id: &async_hid::DeviceId) -> (Option<String>, Option<String>) {
    (None, None)
}

fn map(e: &async_hid::HidError) -> Error {
    Error::device(format!("hid: {e}"))
}

/// The HID devices the current user can open. A device with several
/// top level collections is listed once. With `OXROK_HID_REPLAY=FILE`
/// (or `OXROK_USB_REPLAY`) set, only the device of the file's first
/// session, which [`HidInfo::open`] serves from the file.
pub fn list() -> Result<Vec<HidInfo>> {
    if crate::core::sessions::replaying() {
        return replay_devices();
    }
    block_on(async {
        let backend = HidBackend::default();
        let mut stream = backend.enumerate().await.map_err(|e| map(&e))?;
        let mut out: Vec<HidInfo> = Vec::new();
        while let Some(d) = stream.next().await {
            if out.iter().any(|o| o.device.as_ref().is_some_and(|x| x.id == d.id)) {
                continue;
            }
            let (path, product) = linux_details(&d.id);
            out.push(HidInfo {
                vendor_id: d.vendor_id,
                product_id: d.product_id,
                product: product.unwrap_or_else(|| d.name.clone()),
                manufacturer: d.manufacturer.clone(),
                serial: d.serial_number.clone(),
                path,
                device: Some(d),
            });
        }
        Ok(out)
    })
}

const REPLAY_VAR: &str = "OXROK_HID_REPLAY";

fn replay_devices() -> Result<Vec<HidInfo>> {
    let Some(text) = crate::core::sessions::first_session(REPLAY_VAR, record::MAGIC)? else {
        return Ok(Vec::new());
    };
    let d = record::Fixture::parse(&text)?.device;
    Ok(vec![HidInfo {
        vendor_id: d.vendor_id,
        product_id: d.product_id,
        product: d.product.unwrap_or_default(),
        manufacturer: d.manufacturer,
        serial: d.serial,
        path: d.path,
        device: None,
    }])
}

impl HidInfo {
    /// Opens the device for reading and writing. With
    /// `OXROK_HID_RECORD=FILE` in the environment the session is
    /// recorded and written to `FILE` when the device is closed, later
    /// sessions of the process appended ([`record`]). A device listed
    /// from an `OXROK_HID_REPLAY` file is served the file's next
    /// session.
    pub fn open(&self) -> Result<Box<dyn HidDevice>> {
        let Some(device) = &self.device else {
            let text = crate::core::sessions::next_session(REPLAY_VAR, record::MAGIC)?;
            let replay = record::SharedReplay::new(record::Replay::from_text(&text)?);
            let check = replay.clone();
            crate::core::sessions::register(move || check.finish());
            return Ok(Box::new(replay));
        };
        block_on(async {
            let (reader, writer) = device.open().await.map_err(|e| map(&e))?;
            let feature = device.open_feature_handle().await.map_err(|e| map(&e))?;
            let native = Box::new(Native { reader, writer, feature }) as Box<dyn HidDevice>;
            if let Ok(file) = std::env::var("OXROK_HID_RECORD")
                && !file.is_empty()
            {
                log::info!("Recording the HID session to {file}.");
                let line = record::DeviceLine {
                    vendor_id: self.vendor_id,
                    product_id: self.product_id,
                    manufacturer: self.manufacturer.clone(),
                    product: Some(self.product.clone()),
                    serial: self.serial.clone(),
                    path: self.path.clone(),
                };
                return Ok(
                    Box::new(record::Recorder::to_file(native, line, file)) as Box<dyn HidDevice>
                );
            }
            Ok(native)
        })
    }
}

struct Native {
    reader: async_hid::DeviceReader,
    writer: async_hid::DeviceWriter,
    feature: async_hid::DeviceFeatureHandle,
}

impl HidDevice for Native {
    fn read_report(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let read = async { self.reader.read_input_report(buf).await.map_err(|e| map(&e)) };
        let expire = async {
            async_io::Timer::after(timeout).await;
            Ok(0)
        };
        block_on(or(read, expire))
    }

    fn write_report(&mut self, data: &[u8]) -> Result<()> {
        block_on(self.writer.write_output_report(data)).map_err(|e| map(&e))
    }

    fn get_feature(&mut self, buf: &mut [u8]) -> Result<usize> {
        let n = block_on(self.feature.read_feature_report(buf)).map_err(|e| map(&e))?;
        // hidapi counts the report id byte in the length; the macOS
        // backend returns only the data length for unnumbered reports.
        #[cfg(target_os = "macos")]
        let n = if buf.first() == Some(&0) { (n + 1).min(buf.len()) } else { n };
        Ok(n)
    }

    fn set_feature(&mut self, data: &[u8]) -> Result<()> {
        block_on(self.feature.write_feature_report(data)).map_err(|e| map(&e))
    }
}

/// A scripted device for tests: input reports are served from a queue,
/// everything sent is kept.
#[derive(Default)]
pub struct Fake {
    /// Input reports still to deliver; an empty report is a timeout.
    pub input: std::collections::VecDeque<Vec<u8>>,
    /// Output reports received.
    pub output: Vec<Vec<u8>>,
    /// Feature reports received.
    pub features: Vec<Vec<u8>>,
    /// Feature reports to answer `get_feature` with.
    pub feature_answers: std::collections::VecDeque<Vec<u8>>,
}

impl HidDevice for Fake {
    fn read_report(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let Some(r) = self.input.pop_front() else {
            return Err(Error::device("fake hid: no more input"));
        };
        if r.is_empty() {
            // A timeout takes its time, as on a real device.
            std::thread::sleep(timeout);
        }
        let n = r.len().min(buf.len());
        buf[..n].copy_from_slice(&r[..n]);
        Ok(n)
    }

    fn write_report(&mut self, data: &[u8]) -> Result<()> {
        self.output.push(data.to_vec());
        Ok(())
    }

    fn get_feature(&mut self, buf: &mut [u8]) -> Result<usize> {
        let Some(r) = self.feature_answers.pop_front() else {
            return Err(Error::device("fake hid: no feature answer"));
        };
        let n = r.len().min(buf.len());
        buf[..n].copy_from_slice(&r[..n]);
        Ok(n)
    }

    fn set_feature(&mut self, data: &[u8]) -> Result<()> {
        self.features.push(data.to_vec());
        Ok(())
    }
}

/// A [`Fake`] shared between a test and the code under test, so the
/// test can script input and inspect what was sent.
#[derive(Clone, Default)]
pub struct SharedFake(pub std::sync::Arc<std::sync::Mutex<Fake>>);

impl SharedFake {
    /// Queues an input report.
    pub fn push_input(&self, report: &[u8]) {
        self.0.lock().expect("fake lock").input.push_back(report.to_vec());
    }

    /// The output reports sent so far.
    pub fn output(&self) -> Vec<Vec<u8>> {
        self.0.lock().expect("fake lock").output.clone()
    }

    /// The feature reports sent so far.
    pub fn features(&self) -> Vec<Vec<u8>> {
        self.0.lock().expect("fake lock").features.clone()
    }
}

impl HidDevice for SharedFake {
    fn read_report(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        self.0.lock().expect("fake lock").read_report(buf, timeout)
    }

    fn write_report(&mut self, data: &[u8]) -> Result<()> {
        self.0.lock().expect("fake lock").write_report(data)
    }

    fn get_feature(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.0.lock().expect("fake lock").get_feature(buf)
    }

    fn set_feature(&mut self, data: &[u8]) -> Result<()> {
        self.0.lock().expect("fake lock").set_feature(data)
    }
}
