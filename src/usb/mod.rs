// Counterpart in sigrok:
//   libsigrok/src/usb.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! USB access for oxrok drivers.
//!
//! Drivers talk to a [`Transport`]: control, bulk and interrupt
//! transfers with timeouts, interface claims and resets. Two backends implement it,
//! selected by cargo feature: `nusb` (default, pure Rust) and `libusb`
//! (through `rusb`, for platforms nusb does not cover). The drivers never
//! see which one is in use.
//!
//! [`record::Recorder`] wraps any transport and writes every call and
//! its result to a fixture; [`record::Replay`] plays a fixture back and
//! fails on the first deviation. That is how drivers get CI coverage
//! without hardware.

use crate::core::{Error, Result};
use std::fmt;
use std::time::Duration;

#[cfg(feature = "nusb")]
mod nusb_backend;
pub mod record;
#[cfg(feature = "libusb")]
mod rusb_backend;

/// Vendor and product id of a USB device.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct UsbId {
    /// `idVendor`.
    pub vendor: u16,
    /// `idProduct`.
    pub product: u16,
}

impl fmt::Display for UsbId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:04x}:{:04x}", self.vendor, self.product)
    }
}

/// The class of one interface, as enumeration reports it without
/// opening the device.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InterfaceClass {
    /// Interface number.
    pub number: u8,
    /// `bInterfaceClass`.
    pub class: u8,
    /// `bInterfaceSubClass`.
    pub subclass: u8,
    /// `bInterfaceProtocol`.
    pub protocol: u8,
}

/// One endpoint of an interface.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EndpointDesc {
    /// `bEndpointAddress`, bit 7 set for IN.
    pub address: u8,
    /// Transfer type: 0 control, 1 isochronous, 2 bulk, 3 interrupt.
    pub transfer_type: u8,
    /// `wMaxPacketSize`.
    pub max_packet_size: u16,
}

/// An interface of the active configuration with its endpoints.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct InterfaceDesc {
    /// Class, subclass and protocol.
    pub class: InterfaceClass,
    /// The endpoints of the first alternate setting.
    pub endpoints: Vec<EndpointDesc>,
}

/// What enumeration knows about a device before it is opened.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct UsbDeviceInfo {
    /// `idVendor`.
    pub vendor_id: u16,
    /// `idProduct`.
    pub product_id: u16,
    /// `bcdDevice`, which FTDI uses for the chip type.
    pub device_version: u16,
    /// Bus number. On Windows always 0.
    pub bus: u8,
    /// Address on the bus. On Windows always 0.
    pub address: u8,
    /// Manufacturer string, if the OS cached it or it could be read.
    pub manufacturer: Option<String>,
    /// Product string.
    pub product: Option<String>,
    /// Serial number string.
    pub serial: Option<String>,
    /// The interface classes, for picking devices by class (USBTMC,
    /// HID) before opening them.
    pub interfaces: Vec<InterfaceClass>,
    /// Backend specific handle used to open the device again.
    pub(crate) handle: Handle,
}

impl UsbDeviceInfo {
    /// The vendor and product id.
    pub fn id(&self) -> UsbId {
        UsbId { vendor: self.vendor_id, product: self.product_id }
    }

    /// The `bus.address` string sigrok uses for `conn=`.
    pub fn conn(&self) -> String {
        format!("{}.{}", self.bus, self.address)
    }

    /// True when `conn` (`bus.address`, or `vid.pid` in hex) names this
    /// device.
    pub fn matches_conn(&self, conn: &str) -> bool {
        if let Some((a, b)) = conn.split_once('.')
            && let (Ok(bus), Ok(addr)) = (a.parse::<u8>(), b.parse::<u8>())
        {
            return self.bus == bus && self.address == addr;
        }
        if let Some((v, p)) = conn.split_once('.')
            && let (Ok(vid), Ok(pid)) = (u16::from_str_radix(v, 16), u16::from_str_radix(p, 16))
        {
            return self.vendor_id == vid && self.product_id == pid;
        }
        false
    }
}

/// Backend specific identity, enough to open the device.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) enum Handle {
    /// Not openable, e.g. a replayed device.
    #[default]
    None,
    #[cfg(feature = "nusb")]
    Nusb(nusb::DeviceId),
    #[cfg(feature = "libusb")]
    Rusb { bus: u8, address: u8 },
}

/// A control request, without its data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlRequest {
    /// `bmRequestType` without the direction bit, which the call sets
    /// (`control_in` reads, `control_out` writes).
    pub request_type: u8,
    /// `bRequest`.
    pub request: u8,
    /// `wValue`.
    pub value: u16,
    /// `wIndex`.
    pub index: u16,
}

impl ControlRequest {
    /// A vendor request to the device.
    pub fn vendor(request: u8, value: u16, index: u16) -> Self {
        ControlRequest { request_type: 0x40, request, value, index }
    }

    /// A vendor request to an interface.
    pub fn vendor_interface(request: u8, value: u16, index: u16) -> Self {
        ControlRequest { request_type: 0x41, request, value, index }
    }

    /// The recipient bits of `bmRequestType`.
    pub fn recipient(&self) -> u8 {
        self.request_type & 0x1f
    }

    /// The type bits of `bmRequestType`: 0 standard, 1 class, 2 vendor.
    pub fn kind(&self) -> u8 {
        (self.request_type >> 5) & 3
    }
}

/// One open USB device.
///
/// Every call blocks with the given timeout. `bulk_in` returns what
/// arrived before the deadline, so a short read is not an error; a
/// timeout with nothing received is [`Error::Timeout`].
pub trait Transport: Send {
    /// What enumeration said about the device.
    fn info(&self) -> &UsbDeviceInfo;

    /// Selects a configuration. Needed on hosts that leave a freshly
    /// enumerated device unconfigured until a driver claims it.
    fn set_configuration(&mut self, configuration: u8) -> Result<()>;

    /// Claims an interface. On Linux a kernel driver bound to it is
    /// detached first.
    fn claim_interface(&mut self, interface: u8) -> Result<()>;

    /// Releases an interface claimed before.
    fn release_interface(&mut self, interface: u8) -> Result<()>;

    /// Resets the device. Claims survive on some platforms and not on
    /// others; drivers reclaim afterwards.
    fn reset(&mut self) -> Result<()>;

    /// `wMaxPacketSize` of an endpoint of a claimed interface.
    fn max_packet_size(&mut self, endpoint: u8) -> Result<usize>;

    /// The interfaces of the active configuration with their endpoints.
    fn interfaces(&mut self) -> Result<Vec<InterfaceDesc>>;

    /// A control transfer that sends `data`.
    fn control_out(&mut self, req: ControlRequest, data: &[u8], timeout: Duration) -> Result<()>;

    /// A control transfer that reads up to `len` bytes.
    fn control_in(&mut self, req: ControlRequest, len: usize, timeout: Duration)
    -> Result<Vec<u8>>;

    /// Writes to a bulk OUT endpoint. Returns the number of bytes the
    /// device took, which is less than `data.len()` only on a timeout.
    fn bulk_out(&mut self, endpoint: u8, data: &[u8], timeout: Duration) -> Result<usize>;

    /// Reads up to `len` bytes from a bulk IN endpoint. `len` is rounded
    /// up to a multiple of the packet size by the backend.
    fn bulk_in(&mut self, endpoint: u8, len: usize, timeout: Duration) -> Result<Vec<u8>>;

    /// Writes one interrupt OUT transfer. Returns the number of bytes
    /// the device took, which is less than `data.len()` only on a
    /// timeout.
    fn interrupt_out(&mut self, endpoint: u8, data: &[u8], timeout: Duration) -> Result<usize>;

    /// Reads one interrupt IN transfer of up to `len` bytes, rounded up
    /// to a multiple of the packet size by the backend. Nothing within
    /// `timeout` is [`Error::Timeout`].
    fn interrupt_in(&mut self, endpoint: u8, len: usize, timeout: Duration) -> Result<Vec<u8>>;

    /// Opens a stream of bulk IN transfers that keeps `depth` transfers
    /// of `transfer_size` bytes in flight, for devices that stream
    /// faster than one blocking read after another can drain them. The
    /// endpoint is exclusive to the stream until it is dropped.
    fn bulk_in_stream(
        &mut self,
        endpoint: u8,
        transfer_size: usize,
        depth: usize,
    ) -> Result<Box<dyn BulkStream>>;
}

/// A queue of bulk IN transfers, see [`Transport::bulk_in_stream`].
pub trait BulkStream: Send {
    /// The next completed transfer, in order. A short transfer is not
    /// an error; nothing completing within `timeout` is
    /// [`Error::Timeout`].
    fn next(&mut self, timeout: Duration) -> Result<Vec<u8>>;
}

/// Which backend `list_devices` and `open` use.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Backend {
    /// The `nusb` crate.
    Nusb,
    /// libusb through the `rusb` crate.
    Libusb,
}

impl Backend {
    /// The backend in use: `OXROK_USB_BACKEND=libusb` or `nusb` when set
    /// and compiled in, else the first compiled one, nusb first.
    pub fn current() -> Option<Backend> {
        let wanted = std::env::var("OXROK_USB_BACKEND").ok();
        let mut available = Vec::new();
        if cfg!(feature = "nusb") {
            available.push(Backend::Nusb);
        }
        if cfg!(feature = "libusb") {
            available.push(Backend::Libusb);
        }
        match wanted.as_deref() {
            Some("libusb") if available.contains(&Backend::Libusb) => Some(Backend::Libusb),
            Some("nusb") if available.contains(&Backend::Nusb) => Some(Backend::Nusb),
            Some(other) => {
                log::warn!("OXROK_USB_BACKEND={other} is not compiled in, using the default.");
                available.first().copied()
            }
            None => available.first().copied(),
        }
    }

    /// The backend's name.
    pub fn name(self) -> &'static str {
        match self {
            Backend::Nusb => "nusb",
            Backend::Libusb => "libusb",
        }
    }
}

/// Every USB device the backend can see. String descriptors are those
/// the OS has cached; opening the device is not needed.
pub fn list_devices() -> Result<Vec<UsbDeviceInfo>> {
    match Backend::current() {
        #[cfg(feature = "nusb")]
        Some(Backend::Nusb) => nusb_backend::list(),
        #[cfg(feature = "libusb")]
        Some(Backend::Libusb) => rusb_backend::list(),
        _ => Err(Error::not_supported("no USB backend compiled in")),
    }
}

/// The devices with one of the given ids.
pub fn find_devices(ids: &[UsbId]) -> Result<Vec<UsbDeviceInfo>> {
    Ok(list_devices()?.into_iter().filter(|d| ids.contains(&d.id())).collect())
}

/// Opens a device found by [`list_devices`].
///
/// With `OXROK_USB_RECORD=FILE` in the environment the session is
/// recorded and written to `FILE` when the transport is dropped. With
/// several devices the file name gets `.N` appended from the second one.
pub fn open(info: &UsbDeviceInfo) -> Result<Box<dyn Transport>> {
    let transport = match &info.handle {
        #[cfg(feature = "nusb")]
        Handle::Nusb(_) => nusb_backend::open(info)?,
        #[cfg(feature = "libusb")]
        Handle::Rusb { .. } => rusb_backend::open(info)?,
        Handle::None => return Err(Error::not_supported("this device cannot be opened")),
    };
    if let Ok(path) = std::env::var("OXROK_USB_RECORD") {
        return Ok(Box::new(record::Recorder::to_file(transport, path)));
    }
    Ok(transport)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conn_matching() {
        let d = UsbDeviceInfo {
            vendor_id: 0x0403,
            product_id: 0xdc48,
            bus: 2,
            address: 7,
            ..Default::default()
        };
        assert!(d.matches_conn("2.7"));
        assert!(!d.matches_conn("2.8"));
        assert!(d.matches_conn("0403.dc48"));
        assert!(!d.matches_conn("0403.6001"));
        assert!(!d.matches_conn("garbage"));
        assert_eq!(d.conn(), "2.7");
        assert_eq!(d.id().to_string(), "0403:dc48");
    }

    #[test]
    fn request_fields() {
        let r = ControlRequest::vendor(0x0b, 0x0107, 1);
        assert_eq!(r.kind(), 2);
        assert_eq!(r.recipient(), 0);
        assert_eq!(ControlRequest::vendor_interface(1, 0, 0).recipient(), 1);
    }
}
