//! The `nusb` backend.

use crate::core::Error;
use crate::usb::{BulkStream, ControlRequest, Handle, Result, Transport, UsbDeviceInfo};
use nusb::MaybeFuture;
use nusb::transfer::{
    Buffer, Bulk, ControlIn, ControlOut, ControlType, In, Interrupt, Out, Recipient,
};
use std::collections::HashMap;
use std::time::Duration;

fn map(e: &nusb::Error, what: &str) -> Error {
    use nusb::ErrorKind as K;
    match e.kind() {
        K::Busy => Error::Busy,
        K::Disconnected => Error::device(format!("{what}: device disconnected")),
        K::PermissionDenied => Error::Io(std::io::Error::new(
            std::io::ErrorKind::PermissionDenied,
            format!("{what}: {e} (permission denied)"),
        )),
        K::Unsupported => Error::NotSupported(format!("{what}: {e}")),
        _ => Error::Device(format!("{what}: {e}")),
    }
}

pub(crate) fn list() -> Result<Vec<UsbDeviceInfo>> {
    let devices = nusb::list_devices().wait().map_err(|e| map(&e, "usb"))?;
    Ok(devices.map(|d| info(&d)).collect())
}

fn info(d: &nusb::DeviceInfo) -> UsbDeviceInfo {
    #[cfg(target_os = "linux")]
    let bus = d.busnum();
    // libusb derives the bus number from the location id, so this
    // matches the conn= values sigrok users have.
    #[cfg(target_os = "macos")]
    let bus = (d.location_id() >> 24) as u8;
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    let bus = 0;
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    let address = d.device_address();
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    let address = 0;
    UsbDeviceInfo {
        vendor_id: d.vendor_id(),
        product_id: d.product_id(),
        device_version: d.device_version(),
        bus,
        address,
        #[cfg(any(target_os = "linux", target_os = "macos", target_os = "windows"))]
        ports: d.port_chain().to_vec(),
        #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
        ports: Vec::new(),
        manufacturer: d.manufacturer_string().map(str::to_owned),
        product: d.product_string().map(str::to_owned),
        serial: d.serial_number().map(str::to_owned),
        interfaces: d
            .interfaces()
            .map(|i| crate::usb::InterfaceClass {
                number: i.interface_number(),
                class: i.class(),
                subclass: i.subclass(),
                protocol: i.protocol(),
            })
            .collect(),
        handle: Handle::Nusb(d.id()),
    }
}

pub(crate) fn open(info: &UsbDeviceInfo) -> Result<Box<dyn Transport>> {
    let Handle::Nusb(id) = &info.handle else {
        return Err(Error::not_supported("not a nusb device"));
    };
    let devices = nusb::list_devices().wait().map_err(|e| map(&e, "usb"))?;
    let found = devices
        .into_iter()
        .find(|d| d.id() == *id)
        .ok_or_else(|| Error::device("device is gone"))?;
    let device = found.open().wait().map_err(|e| map(&e, "usb"))?;
    Ok(Box::new(Nusb {
        info: info.clone(),
        device,
        interfaces: HashMap::new(),
        in_endpoints: HashMap::new(),
        out_endpoints: HashMap::new(),
        int_in_endpoints: HashMap::new(),
        int_out_endpoints: HashMap::new(),
    }))
}

struct Nusb {
    info: UsbDeviceInfo,
    device: nusb::Device,
    interfaces: HashMap<u8, nusb::Interface>,
    in_endpoints: HashMap<u8, nusb::Endpoint<Bulk, In>>,
    out_endpoints: HashMap<u8, nusb::Endpoint<Bulk, Out>>,
    int_in_endpoints: HashMap<u8, nusb::Endpoint<Interrupt, In>>,
    int_out_endpoints: HashMap<u8, nusb::Endpoint<Interrupt, Out>>,
}

fn control_type(req: ControlRequest) -> (ControlType, Recipient) {
    let kind = match req.kind() {
        0 => ControlType::Standard,
        1 => ControlType::Class,
        _ => ControlType::Vendor,
    };
    let recipient = match req.recipient() {
        1 => Recipient::Interface,
        2 => Recipient::Endpoint,
        3 => Recipient::Other,
        _ => Recipient::Device,
    };
    (kind, recipient)
}

fn transfer_error(e: nusb::transfer::TransferError) -> Error {
    use nusb::transfer::TransferError as T;
    match e {
        T::Cancelled => Error::Timeout,
        T::Disconnected => Error::device("device disconnected"),
        T::Stall => Error::device("endpoint stalled"),
        other => Error::Device(other.to_string()),
    }
}

/// Claims an interface, detaching the kernel driver only when one is
/// bound, as libsigrok's drivers do (`libusb_kernel_driver_active`,
/// then `libusb_detach_kernel_driver`). nusb's detach-and-claim
/// reattaches on release whether or not it detached anything, and on
/// Linux that makes the kernel probe its drivers for the interface
/// again: `usbhid` reads the serial number string while it probes even
/// a device on its ignore list (the `PICkit2`), a control transfer
/// libsigrok never causes.
fn claim(
    device: &nusb::Device,
    interface: u8,
) -> std::result::Result<nusb::Interface, nusb::Error> {
    claim_or_detach(
        || device.claim_interface(interface).wait(),
        || device.detach_and_claim_interface(interface).wait(),
        |e| e.kind() == nusb::ErrorKind::Busy,
    )
}

/// The policy of [`claim`]: a plain claim first, the detaching one
/// only when the plain one finds the interface `busy` (a kernel driver
/// holds it).
fn claim_or_detach<T, E>(
    claim: impl FnOnce() -> std::result::Result<T, E>,
    detach_and_claim: impl FnOnce() -> std::result::Result<T, E>,
    busy: impl Fn(&E) -> bool,
) -> std::result::Result<T, E> {
    match claim() {
        Err(e) if busy(&e) => detach_and_claim(),
        r => r,
    }
}

impl Nusb {
    /// The claimed interface that owns `endpoint`.
    fn interface_for(&self, endpoint: u8) -> Result<&nusb::Interface> {
        self.interfaces
            .values()
            .find(|i| {
                i.descriptor().is_some_and(|d| d.endpoints().any(|e| e.address() == endpoint))
            })
            .ok_or_else(|| {
                Error::device(format!("endpoint 0x{endpoint:02x} is not on a claimed interface"))
            })
    }

    fn in_endpoint(&mut self, endpoint: u8) -> Result<&mut nusb::Endpoint<Bulk, In>> {
        if !self.in_endpoints.contains_key(&endpoint) {
            let ep = self
                .interface_for(endpoint)?
                .endpoint::<Bulk, In>(endpoint)
                .map_err(|e| map(&e, "usb"))?;
            self.in_endpoints.insert(endpoint, ep);
        }
        Ok(self.in_endpoints.get_mut(&endpoint).expect("inserted"))
    }

    fn out_endpoint(&mut self, endpoint: u8) -> Result<&mut nusb::Endpoint<Bulk, Out>> {
        if !self.out_endpoints.contains_key(&endpoint) {
            let ep = self
                .interface_for(endpoint)?
                .endpoint::<Bulk, Out>(endpoint)
                .map_err(|e| map(&e, "usb"))?;
            self.out_endpoints.insert(endpoint, ep);
        }
        Ok(self.out_endpoints.get_mut(&endpoint).expect("inserted"))
    }

    fn int_in_endpoint(&mut self, endpoint: u8) -> Result<&mut nusb::Endpoint<Interrupt, In>> {
        if !self.int_in_endpoints.contains_key(&endpoint) {
            let ep = self
                .interface_for(endpoint)?
                .endpoint::<Interrupt, In>(endpoint)
                .map_err(|e| map(&e, "usb"))?;
            self.int_in_endpoints.insert(endpoint, ep);
        }
        Ok(self.int_in_endpoints.get_mut(&endpoint).expect("inserted"))
    }

    fn int_out_endpoint(&mut self, endpoint: u8) -> Result<&mut nusb::Endpoint<Interrupt, Out>> {
        if !self.int_out_endpoints.contains_key(&endpoint) {
            let ep = self
                .interface_for(endpoint)?
                .endpoint::<Interrupt, Out>(endpoint)
                .map_err(|e| map(&e, "usb"))?;
            self.int_out_endpoints.insert(endpoint, ep);
        }
        Ok(self.int_out_endpoints.get_mut(&endpoint).expect("inserted"))
    }

    fn drop_endpoints(&mut self) {
        self.in_endpoints.clear();
        self.out_endpoints.clear();
        self.int_in_endpoints.clear();
        self.int_out_endpoints.clear();
    }

    /// Opens the device again. The handle caches the descriptors it
    /// read when it was opened; after a configuration change they are
    /// stale.
    fn reopen(&mut self) -> Result<()> {
        self.drop_endpoints();
        self.interfaces.clear();
        let Handle::Nusb(id) = &self.info.handle else { return Ok(()) };
        let devices = nusb::list_devices().wait().map_err(|e| map(&e, "usb"))?;
        if let Some(d) = devices.into_iter().find(|d| d.id() == *id) {
            self.device = d.open().wait().map_err(|e| map(&e, "reopen"))?;
        }
        Ok(())
    }
}

impl Transport for Nusb {
    fn info(&self) -> &UsbDeviceInfo {
        &self.info
    }

    fn set_configuration(&mut self, configuration: u8) -> Result<()> {
        let active = self.device.active_configuration().map(|c| c.configuration_value());
        log::debug!("active configuration: {active:?}, wanted {configuration}");
        if active.as_ref().is_ok_and(|c| *c == configuration) {
            return Ok(());
        }
        self.drop_endpoints();
        self.interfaces.clear();
        self.device
            .set_configuration(configuration)
            .wait()
            .map_err(|e| map(&e, &format!("set configuration {configuration}")))?;
        self.reopen()
    }

    fn force_configuration(&mut self, configuration: u8) -> Result<()> {
        self.drop_endpoints();
        self.interfaces.clear();
        self.device
            .set_configuration(configuration)
            .wait()
            .map_err(|e| map(&e, &format!("set configuration {configuration}")))?;
        self.reopen()
    }

    fn string_indexes(&mut self) -> Result<[u8; 3]> {
        let d = self.device.device_descriptor();
        let i = |x: Option<std::num::NonZeroU8>| x.map_or(0, std::num::NonZeroU8::get);
        Ok([
            i(d.manufacturer_string_index()),
            i(d.product_string_index()),
            i(d.serial_number_string_index()),
        ])
    }

    fn claim_interface(&mut self, interface: u8) -> Result<()> {
        if self.interfaces.contains_key(&interface) {
            return Ok(());
        }
        let claimed = match claim(&self.device, interface) {
            Ok(i) => i,
            Err(e) if e.kind() == nusb::ErrorKind::NotFound => {
                // Right after a firmware renumeration macOS can hand out
                // a device whose descriptors still show no interfaces.
                // Setting the configuration again makes it re-read them,
                // which is what libusb does on every open.
                let seen =
                    self.device.configurations().map(|c| c.interfaces().count()).sum::<usize>();
                log::debug!(
                    "interface {interface} not found ({seen} interfaces seen), reconfiguring"
                );
                let config =
                    self.device.active_configuration().map_or(1, |c| c.configuration_value());
                self.device
                    .set_configuration(config)
                    .wait()
                    .map_err(|e| map(&e, &format!("set configuration {config}")))?;
                self.reopen()?;
                claim(&self.device, interface)
                    .map_err(|e| map(&e, &format!("claim interface {interface}")))?
            }
            Err(e) => return Err(map(&e, &format!("claim interface {interface}"))),
        };
        self.interfaces.insert(interface, claimed);
        Ok(())
    }

    fn release_interface(&mut self, interface: u8) -> Result<()> {
        self.drop_endpoints();
        if let Some(i) = self.interfaces.remove(&interface) {
            i.release().wait().map_err(|e| map(&e, "usb"))?;
        }
        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        self.drop_endpoints();
        self.interfaces.clear();
        self.device.reset().wait().map_err(|e| map(&e, "reset"))
    }

    fn max_packet_size(&mut self, endpoint: u8) -> Result<usize> {
        let i = self.interface_for(endpoint)?;
        let d = i.descriptor().ok_or_else(|| Error::device("no interface descriptor"))?;
        d.endpoints()
            .find(|e| e.address() == endpoint)
            .map(|e| e.max_packet_size())
            .ok_or_else(|| Error::device("no such endpoint"))
    }

    fn interfaces(&mut self) -> Result<Vec<crate::usb::InterfaceDesc>> {
        let config = self
            .device
            .active_configuration()
            .map_err(|e| Error::device(format!("configuration: {e}")))?;
        let mut out = Vec::new();
        for group in config.interfaces() {
            let Some(d) = group.alt_settings().next() else { continue };
            out.push(crate::usb::InterfaceDesc {
                class: crate::usb::InterfaceClass {
                    number: d.interface_number(),
                    class: d.class(),
                    subclass: d.subclass(),
                    protocol: d.protocol(),
                },
                endpoints: d
                    .endpoints()
                    .map(|e| crate::usb::EndpointDesc {
                        address: e.address(),
                        transfer_type: e.transfer_type() as u8,
                        max_packet_size: e.max_packet_size() as u16,
                    })
                    .collect(),
            });
        }
        Ok(out)
    }

    fn control_out(&mut self, req: ControlRequest, data: &[u8], timeout: Duration) -> Result<()> {
        let (control_type, recipient) = control_type(req);
        let out = ControlOut {
            control_type,
            recipient,
            request: req.request,
            value: req.value,
            index: req.index,
            data,
        };
        let r = if let Some(i) = self.interfaces.values().next() {
            i.control_out(out, timeout).wait()
        } else {
            #[cfg(not(target_os = "windows"))]
            {
                self.device.control_out(out, timeout).wait()
            }
            #[cfg(target_os = "windows")]
            {
                return Err(Error::device("claim an interface before control transfers"));
            }
        };
        r.map_err(transfer_error)
    }

    fn control_in(
        &mut self,
        req: ControlRequest,
        len: usize,
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        let (control_type, recipient) = control_type(req);
        let inp = ControlIn {
            control_type,
            recipient,
            request: req.request,
            value: req.value,
            index: req.index,
            length: u16::try_from(len).map_err(|_| Error::arg("control transfer too long"))?,
        };
        let r = if let Some(i) = self.interfaces.values().next() {
            i.control_in(inp, timeout).wait()
        } else {
            #[cfg(not(target_os = "windows"))]
            {
                self.device.control_in(inp, timeout).wait()
            }
            #[cfg(target_os = "windows")]
            {
                return Err(Error::device("claim an interface before control transfers"));
            }
        };
        r.map_err(transfer_error)
    }

    fn bulk_out(&mut self, endpoint: u8, data: &[u8], timeout: Duration) -> Result<usize> {
        let ep = self.out_endpoint(endpoint)?;
        let mut buf = Buffer::new(data.len());
        buf.extend_from_slice(data);
        let c = ep.transfer_blocking(buf, timeout);
        match c.status {
            Ok(()) => Ok(c.actual_len),
            Err(nusb::transfer::TransferError::Cancelled) if c.actual_len > 0 => Ok(c.actual_len),
            Err(e) => Err(transfer_error(e)),
        }
    }

    fn bulk_in_stream(
        &mut self,
        endpoint: u8,
        transfer_size: usize,
        depth: usize,
    ) -> Result<Box<dyn BulkStream>> {
        self.in_endpoint(endpoint)?;
        let mut ep = self.in_endpoints.remove(&endpoint).expect("inserted");
        let mps = ep.max_packet_size().max(1);
        let size = transfer_size.div_ceil(mps).max(1) * mps;
        for _ in 0..depth.max(1) {
            ep.submit(ep.allocate(size));
        }
        Ok(Box::new(NusbStream { ep, size, owed: false }))
    }

    fn bulk_in(&mut self, endpoint: u8, len: usize, timeout: Duration) -> Result<Vec<u8>> {
        let ep = self.in_endpoint(endpoint)?;
        let mps = ep.max_packet_size().max(1);
        let len = len.div_ceil(mps).max(1) * mps;
        let c = ep.transfer_blocking(Buffer::new(len), timeout);
        match c.status {
            Ok(()) => Ok(c.buffer.into_vec()),
            Err(nusb::transfer::TransferError::Cancelled) if c.actual_len > 0 => {
                let mut v = c.buffer.into_vec();
                v.truncate(c.actual_len);
                Ok(v)
            }
            Err(e) => Err(transfer_error(e)),
        }
    }

    fn interrupt_out(&mut self, endpoint: u8, data: &[u8], timeout: Duration) -> Result<usize> {
        let ep = self.int_out_endpoint(endpoint)?;
        let mut buf = Buffer::new(data.len());
        buf.extend_from_slice(data);
        let c = ep.transfer_blocking(buf, timeout);
        match c.status {
            Ok(()) => Ok(c.actual_len),
            Err(nusb::transfer::TransferError::Cancelled) if c.actual_len > 0 => Ok(c.actual_len),
            Err(e) => Err(transfer_error(e)),
        }
    }

    fn interrupt_in(&mut self, endpoint: u8, len: usize, timeout: Duration) -> Result<Vec<u8>> {
        let ep = self.int_in_endpoint(endpoint)?;
        let mps = ep.max_packet_size().max(1);
        let len = len.div_ceil(mps).max(1) * mps;
        let c = ep.transfer_blocking(Buffer::new(len), timeout);
        match c.status {
            Ok(()) => Ok(c.buffer.into_vec()),
            Err(e) => Err(transfer_error(e)),
        }
    }
}

struct NusbStream {
    ep: nusb::Endpoint<Bulk, In>,
    size: usize,
    /// A transfer returned by the last `next`, not yet resubmitted.
    owed: bool,
}

impl BulkStream for NusbStream {
    fn next(&mut self, timeout: Duration) -> Result<Vec<u8>> {
        // The transfer handed out last goes back into the queue once
        // the caller has dealt with it and asks for more, as libusb
        // drivers resubmit at the end of their callback: one that stops
        // after a transfer leaves no new transfer behind it.
        if std::mem::take(&mut self.owed) {
            self.ep.submit(self.ep.allocate(self.size));
        }
        let Some(c) = self.ep.wait_next_complete(timeout) else {
            return Err(Error::Timeout);
        };
        self.owed = true;
        match c.status {
            Ok(()) => Ok(c.buffer.into_vec()),
            Err(e) if c.actual_len > 0 => {
                log::debug!("transfer ended with {e} after {} bytes", c.actual_len);
                let mut v = c.buffer.into_vec();
                v.truncate(c.actual_len);
                Ok(v)
            }
            Err(e) => Err(transfer_error(e)),
        }
    }
}

impl Drop for NusbStream {
    fn drop(&mut self) {
        self.ep.cancel_all();
        while self.ep.pending() > 0 {
            if self.ep.wait_next_complete(Duration::from_secs(1)).is_none() {
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::claim_or_detach;
    use std::cell::Cell;

    #[derive(Debug, PartialEq)]
    enum E {
        Busy,
        Gone,
    }

    /// The detaching claim runs only when a driver holds the interface,
    /// so a release never binds a kernel driver that was not bound.
    #[test]
    fn detaches_only_a_busy_interface() {
        let detached = Cell::new(0);
        let detach = || {
            detached.set(detached.get() + 1);
            Ok::<_, E>("detached")
        };
        let busy = |e: &E| *e == E::Busy;
        assert_eq!(claim_or_detach(|| Ok("claimed"), detach, busy), Ok("claimed"));
        assert_eq!(detached.get(), 0);
        assert_eq!(claim_or_detach(|| Err(E::Gone), detach, busy), Err(E::Gone));
        assert_eq!(detached.get(), 0);
        assert_eq!(claim_or_detach(|| Err(E::Busy), detach, busy), Ok("detached"));
        assert_eq!(detached.get(), 1);
    }
}
