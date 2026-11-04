//! The libusb backend through `rusb`.

use crate::core::Error;
use crate::usb::{BulkStream, ControlRequest, Handle, Result, Transport, UsbDeviceInfo};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, sync_channel};
use std::time::{Duration, Instant};

/// `bcdDevice` from what rusb's `Version::from_bcd` left of it: its major
/// folds the two high nibbles into a decimal, 11 for 0x1100. That cannot
/// be undone for a value that is not BCD: 0x0a00 and 0x1000 both give 10,
/// and come back as 0x1000. nusb hands out the field itself.
fn bcd_version(major: u8, minor: u8, sub_minor: u8) -> u16 {
    (u16::from(major / 10) << 12)
        | (u16::from(major % 10) << 8)
        | (u16::from(minor) << 4)
        | u16::from(sub_minor)
}

fn map(e: rusb::Error) -> Error {
    match e {
        rusb::Error::Timeout => Error::Timeout,
        rusb::Error::Busy => Error::Busy,
        rusb::Error::Access => Error::Io(std::io::Error::new(
            std::io::ErrorKind::PermissionDenied,
            "access denied (permission denied)",
        )),
        rusb::Error::NoDevice => Error::Disconnected,
        rusb::Error::Pipe => Error::device("endpoint stalled"),
        other => Error::Device(other.to_string()),
    }
}

pub(crate) fn list() -> Result<Vec<UsbDeviceInfo>> {
    let mut out = Vec::new();
    for d in rusb::devices().map_err(map)?.iter() {
        let Ok(desc) = d.device_descriptor() else { continue };
        // Strings need an open handle; skip them when the device is not
        // ours to open. Drivers read them again after opening.
        let (manufacturer, product, serial) = match d.open() {
            Ok(h) => (
                h.read_manufacturer_string_ascii(&desc).ok(),
                h.read_product_string_ascii(&desc).ok(),
                h.read_serial_number_string_ascii(&desc).ok(),
            ),
            Err(_) => (None, None, None),
        };
        let v = desc.device_version();
        let interfaces = d
            .active_config_descriptor()
            .map(|c| {
                c.interfaces()
                    .filter_map(|i| {
                        i.descriptors().next().map(|d| crate::usb::InterfaceClass {
                            number: d.interface_number(),
                            class: d.class_code(),
                            subclass: d.sub_class_code(),
                            protocol: d.protocol_code(),
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();
        out.push(UsbDeviceInfo {
            vendor_id: desc.vendor_id(),
            product_id: desc.product_id(),
            device_version: bcd_version(v.major(), v.minor(), v.sub_minor()),
            bus: d.bus_number(),
            address: d.address(),
            ports: d.port_numbers().unwrap_or_default(),
            manufacturer,
            product,
            serial,
            interfaces,
            handle: Handle::Rusb { bus: d.bus_number(), address: d.address() },
        });
    }
    Ok(out)
}

pub(crate) fn open(info: &UsbDeviceInfo) -> Result<Box<dyn Transport>> {
    let Handle::Rusb { bus, address } = info.handle else {
        return Err(Error::not_supported("not a libusb device"));
    };
    let device = rusb::devices()
        .map_err(map)?
        .iter()
        .find(|d| d.bus_number() == bus && d.address() == address)
        .ok_or_else(|| Error::device("device is gone"))?;
    let handle = device.open().map_err(map)?;
    let _ = handle.set_auto_detach_kernel_driver(true);
    Ok(Box::new(Rusb { info: info.clone(), device, handle: Arc::new(handle), claimed: Vec::new() }))
}

struct Rusb {
    info: UsbDeviceInfo,
    device: rusb::Device<rusb::GlobalContext>,
    handle: Arc<rusb::DeviceHandle<rusb::GlobalContext>>,
    claimed: Vec<u8>,
}

impl Transport for Rusb {
    fn info(&self) -> &UsbDeviceInfo {
        &self.info
    }

    fn set_configuration(&mut self, configuration: u8) -> Result<()> {
        if self.handle.active_configuration().is_ok_and(|c| c == configuration) {
            return Ok(());
        }
        self.claimed.clear();
        self.handle.set_active_configuration(configuration).map_err(map)
    }

    fn force_configuration(&mut self, configuration: u8) -> Result<()> {
        self.claimed.clear();
        self.handle.set_active_configuration(configuration).map_err(map)
    }

    fn string_indexes(&mut self) -> Result<[u8; 3]> {
        let d = self.device.device_descriptor().map_err(map)?;
        Ok([
            d.manufacturer_string_index().unwrap_or(0),
            d.product_string_index().unwrap_or(0),
            d.serial_number_string_index().unwrap_or(0),
        ])
    }

    fn detach_kernel_driver(&mut self, interface: u8) -> Result<()> {
        // libusb's op_release_interface attaches a driver on release
        // whenever the handle auto-detaches, detached or not.
        let _ = self.handle.set_auto_detach_kernel_driver(false);
        // Only an answer of "yes" detaches, as the C's `== 1`: usbfs
        // (another program) counts as none, and an error as none too.
        if self.handle.kernel_driver_active(interface).unwrap_or(false) {
            self.handle.detach_kernel_driver(interface).map_err(map)?;
            log::info!(
                "Detached the kernel driver from interface {interface}: it stays detached \
                 until the device is plugged in again."
            );
        }
        Ok(())
    }

    fn claim_interface(&mut self, interface: u8) -> Result<()> {
        if self.claimed.contains(&interface) {
            return Ok(());
        }
        self.handle.claim_interface(interface).map_err(map)?;
        self.claimed.push(interface);
        Ok(())
    }

    fn release_interface(&mut self, interface: u8) -> Result<()> {
        if let Some(pos) = self.claimed.iter().position(|&i| i == interface) {
            self.claimed.remove(pos);
            self.handle.release_interface(interface).map_err(map)?;
        }
        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        self.claimed.clear();
        self.handle.reset().map_err(map)
    }

    fn max_packet_size(&mut self, endpoint: u8) -> Result<usize> {
        let config = self.device.active_config_descriptor().map_err(map)?;
        for i in config.interfaces() {
            for d in i.descriptors() {
                for e in d.endpoint_descriptors() {
                    if e.address() == endpoint {
                        return Ok(usize::from(e.max_packet_size()));
                    }
                }
            }
        }
        Err(Error::device("no such endpoint"))
    }

    fn interfaces(&mut self) -> Result<Vec<crate::usb::InterfaceDesc>> {
        let config = self.device.active_config_descriptor().map_err(map)?;
        let mut out = Vec::new();
        for i in config.interfaces() {
            let Some(d) = i.descriptors().next() else { continue };
            out.push(crate::usb::InterfaceDesc {
                class: crate::usb::InterfaceClass {
                    number: d.interface_number(),
                    class: d.class_code(),
                    subclass: d.sub_class_code(),
                    protocol: d.protocol_code(),
                },
                endpoints: d
                    .endpoint_descriptors()
                    .map(|e| crate::usb::EndpointDesc {
                        address: e.address(),
                        transfer_type: e.transfer_type() as u8,
                        max_packet_size: e.max_packet_size(),
                    })
                    .collect(),
            });
        }
        Ok(out)
    }

    fn control_out(&mut self, req: ControlRequest, data: &[u8], timeout: Duration) -> Result<()> {
        self.handle
            .write_control(
                req.request_type & 0x7f,
                req.request,
                req.value,
                req.index,
                data,
                timeout,
            )
            .map(|_| ())
            .map_err(map)
    }

    fn control_in(
        &mut self,
        req: ControlRequest,
        len: usize,
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; len];
        let n = self
            .handle
            .read_control(
                req.request_type | 0x80,
                req.request,
                req.value,
                req.index,
                &mut buf,
                timeout,
            )
            .map_err(map)?;
        buf.truncate(n);
        Ok(buf)
    }

    fn bulk_out(&mut self, endpoint: u8, data: &[u8], timeout: Duration) -> Result<usize> {
        self.handle.write_bulk(endpoint, data, timeout).map_err(map)
    }

    fn interrupt_out(&mut self, endpoint: u8, data: &[u8], timeout: Duration) -> Result<usize> {
        self.handle.write_interrupt(endpoint, data, timeout).map_err(map)
    }

    fn interrupt_in(&mut self, endpoint: u8, len: usize, timeout: Duration) -> Result<Vec<u8>> {
        let mps = self.max_packet_size(endpoint)?.max(1);
        let len = len.div_ceil(mps).max(1) * mps;
        let mut buf = vec![0u8; len];
        let n = self.handle.read_interrupt(endpoint, &mut buf, timeout).map_err(map)?;
        buf.truncate(n);
        Ok(buf)
    }

    /// libusb's synchronous API has no queue, so a thread reads back to
    /// back into a channel. The gap between two reads is the thread's
    /// turnaround, which is short but not zero; the nusb backend keeps
    /// transfers queued in the kernel instead.
    fn bulk_in_stream(
        &mut self,
        endpoint: u8,
        transfer_size: usize,
        depth: usize,
    ) -> Result<Box<dyn BulkStream>> {
        let mps = self.max_packet_size(endpoint)?.max(1);
        let size = transfer_size.div_ceil(mps).max(1) * mps;
        let (tx, rx) = sync_channel(depth.max(1));
        let stop = Arc::new(AtomicBool::new(false));
        let handle = Arc::clone(&self.handle);
        let stop2 = Arc::clone(&stop);
        std::thread::Builder::new()
            .name("usb bulk stream".into())
            .spawn(move || {
                while !stop2.load(Ordering::SeqCst) {
                    let mut buf = vec![0u8; size];
                    let r = match handle.read_bulk(endpoint, &mut buf, Duration::from_millis(100)) {
                        Ok(n) => {
                            buf.truncate(n);
                            Ok(buf)
                        }
                        Err(rusb::Error::Timeout) => continue,
                        Err(e) => Err(map(e)),
                    };
                    let failed = r.is_err();
                    if tx.send(r).is_err() || failed {
                        break;
                    }
                }
            })
            .map_err(Error::Io)?;
        Ok(Box::new(RusbStream { rx, stop }))
    }

    /// The thread reads one packet at a time, so a stop is seen within
    /// 100 ms whatever `transfer_timeout` is, and hands on what it has at each
    /// `transfer_timeout`, at a short packet, or when `transfer_size` is
    /// full. Lossless but slower than whole transfers: a device that
    /// cannot pause (a fast stream) may overflow on this backend.
    fn bulk_in_stream_timed(
        &mut self,
        endpoint: u8,
        transfer_size: usize,
        depth: usize,
        transfer_timeout: Duration,
    ) -> Result<Box<dyn BulkStream>> {
        let mps = self.max_packet_size(endpoint)?.max(1);
        let size = transfer_size.div_ceil(mps).max(1) * mps;
        let (tx, rx) = sync_channel(depth.max(1));
        let stop = Arc::new(AtomicBool::new(false));
        let handle = Arc::clone(&self.handle);
        let stop2 = Arc::clone(&stop);
        std::thread::Builder::new()
            .name("usb bulk stream".into())
            .spawn(move || {
                let mut packet = vec![0u8; mps];
                let mut buf = Vec::with_capacity(size);
                let mut deadline = Instant::now() + transfer_timeout;
                while !stop2.load(Ordering::SeqCst) {
                    let left = deadline.saturating_duration_since(Instant::now());
                    if left.is_zero() {
                        if !buf.is_empty() && tx.send(Ok(std::mem::take(&mut buf))).is_err() {
                            break;
                        }
                        deadline = Instant::now() + transfer_timeout;
                        continue;
                    }
                    // Whole milliseconds at least: libusb takes 0 as no timeout.
                    let wait = left.min(Duration::from_millis(100)).max(Duration::from_millis(1));
                    match handle.read_bulk(endpoint, &mut packet, wait) {
                        Ok(n) => {
                            buf.extend_from_slice(&packet[..n]);
                            if n < mps || buf.len() >= size {
                                if tx.send(Ok(std::mem::take(&mut buf))).is_err() {
                                    break;
                                }
                                deadline = Instant::now() + transfer_timeout;
                            }
                        }
                        Err(rusb::Error::Timeout) => {}
                        Err(e) => {
                            if !buf.is_empty() {
                                let _ = tx.send(Ok(std::mem::take(&mut buf)));
                            }
                            let _ = tx.send(Err(map(e)));
                            break;
                        }
                    }
                }
            })
            .map_err(Error::Io)?;
        Ok(Box::new(RusbStream { rx, stop }))
    }

    fn bulk_in(&mut self, endpoint: u8, len: usize, timeout: Duration) -> Result<Vec<u8>> {
        let mps = self.max_packet_size(endpoint)?.max(1);
        let len = len.div_ceil(mps).max(1) * mps;
        let mut buf = vec![0u8; len];
        match self.handle.read_bulk(endpoint, &mut buf, timeout) {
            Ok(n) => {
                buf.truncate(n);
                Ok(buf)
            }
            Err(e) => Err(map(e)),
        }
    }
}

struct RusbStream {
    rx: Receiver<Result<Vec<u8>>>,
    stop: Arc<AtomicBool>,
}

impl BulkStream for RusbStream {
    fn next(&mut self, timeout: Duration) -> Result<Vec<u8>> {
        match self.rx.recv_timeout(timeout) {
            Ok(r) => r,
            Err(RecvTimeoutError::Timeout) => Err(Error::Timeout),
            Err(RecvTimeoutError::Disconnected) => Err(Error::device("stream ended")),
        }
    }
}

impl Drop for RusbStream {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
    }
}

#[cfg(test)]
mod tests {
    use super::bcd_version;

    /// Through rusb's own decoding, so the fold it does is the real one.
    fn round_trip(bcd: u16) -> u16 {
        let v = rusb::Version::from_bcd(bcd);
        bcd_version(v.major(), v.minor(), v.sub_minor())
    }

    #[test]
    fn bcd_device_survives_rusb() {
        for bcd in [0x0000, 0x0100, 0x0200, 0x0912, 0x1100, 0x1200, 0x2345, 0x9999] {
            assert_eq!(round_trip(bcd), bcd);
        }
        // Not BCD: the limit named at bcd_version.
        assert_eq!(round_trip(0x0a00), 0x1000);
    }
}
