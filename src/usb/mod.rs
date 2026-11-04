// Counterpart in sigrok:
//   libsigrok/src/usb.c,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2015 Daniel Elstner <daniel.kitta@gmail.com>,
//   with commits by Gerhard Sittig, Aurelien Jacobs, Joel Holdsworth,
//   Martin Ling, Soeren Apel, Uffe Jakobsen, Vincent Palatin
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
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
    /// The hub ports from the root to the device, which stay the same
    /// when it renumerates (libsigrok's `usb_get_port_path()`); empty
    /// when the backend does not know them.
    pub ports: Vec<u8>,
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

/// A `conn` spec as `sr_usb_split_conn()` reads it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Conn {
    /// `VID.PID`, four hex digits each.
    Ids(u16, u16),
    /// `BUS.ADDRESS` in decimal.
    BusAddress(u8, u8),
}

/// `sr_usb_split_conn()` and the checks of `sr_usb_find()`: four hex
/// digits each are a `VID.PID` (tried first, so `1000.1000` is one),
/// else decimal digits a `BUS.ADDRESS` with the bus at most 255 and the
/// address at most 127. `Err` is the message `sr_usb_find()` logs; zero
/// in either half, or no match, cannot name a device.
pub fn split_conn(conn: &str) -> std::result::Result<Conn, &'static str> {
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
    // strtoul() saturates; anything that does not fit is out of range.
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
    /// True when this spec names `d`.
    pub fn matches(self, d: &UsbDeviceInfo) -> bool {
        match self {
            Conn::Ids(vid, pid) => d.id() == UsbId { vendor: vid, product: pid },
            Conn::BusAddress(bus, addr) => d.bus == bus && d.address == addr,
        }
    }
}

/// Backend specific identity, enough to open the device.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) enum Handle {
    /// Not openable, e.g. a replayed device.
    #[default]
    None,
    /// A device of the `OXROK_USB_REPLAY` file: opened as its next
    /// session.
    Replay,
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

    /// Sends `SET_CONFIGURATION` even when `configuration` is the
    /// active one, as `libusb_set_configuration()` does on Linux; for
    /// drivers whose transactions follow a libsigrok driver's.
    fn force_configuration(&mut self, configuration: u8) -> Result<()> {
        self.set_configuration(configuration)
    }

    /// The string descriptor indexes of the device descriptor
    /// (manufacturer, product, serial number; 0 for none), from what
    /// the OS cached at enumeration, without a transfer.
    fn string_indexes(&mut self) -> Result<[u8; 3]> {
        Err(Error::not_supported("string descriptor indexes"))
    }

    /// Detaches the kernel driver bound to an interface, if any, as
    /// `libusb_kernel_driver_active` and `libusb_detach_kernel_driver` do:
    /// another program's claim is left alone, and the driver stays detached
    /// after a release, until the device is plugged in again or something
    /// binds it. For a C driver that does this itself on an interface a
    /// kernel driver binds, whose probe would otherwise send requests of
    /// its own at every release. No transfer, so a recording has no line
    /// for it. A no-op for nusb off Linux and on Windows; libusb on macOS
    /// implements both calls (`darwin_usb.c`), and there the detach fails
    /// with `LIBUSB_ERROR_ACCESS` without root or an entitlement.
    fn detach_kernel_driver(&mut self, interface: u8) -> Result<()> {
        let _ = interface;
        Ok(())
    }

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
    /// up to a multiple of the packet size by the backend. A timeout gives
    /// what arrived before it as an `Ok` short read, and is
    /// [`Error::Timeout`] only when nothing did; a STALL or a disconnect is
    /// an error and gives no partial data.
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

    /// [`Transport::bulk_in_stream`] with libusb's per transfer timeout,
    /// for drivers that follow a C driver's habit of taking whatever a
    /// timed out transfer holds: when nothing completes for
    /// `transfer_timeout`, [`BulkStream::next`] cancels the transfers in
    /// flight, returns the data they had received (nothing is
    /// [`Error::Timeout`]) and submits them again. So a slow device's data
    /// comes every `transfer_timeout`, not when a transfer has filled, and
    /// a device with no short packet to end its last transfer cannot hold
    /// the data back. A transport without its own form (a replay reads
    /// its recorded steps either way) falls back to the plain stream.
    fn bulk_in_stream_timed(
        &mut self,
        endpoint: u8,
        transfer_size: usize,
        depth: usize,
        transfer_timeout: Duration,
    ) -> Result<Box<dyn BulkStream>> {
        let _ = transfer_timeout;
        self.bulk_in_stream(endpoint, transfer_size, depth)
    }
}

/// Reads string descriptor `index` as `libusb_get_string_descriptor_ascii()`
/// does: the language table (4 bytes) first, then the string in its
/// first language (up to 255 bytes), each character outside ASCII
/// made `?`. Index 0 is refused without a transfer, as libusb does.
pub fn read_string_ascii(t: &mut dyn Transport, index: u8) -> Result<String> {
    const GET_DESCRIPTOR: u8 = 6;
    const STRING: u16 = 3 << 8;
    const TIMEOUT: Duration = Duration::from_millis(1000);
    if index == 0 {
        return Err(Error::arg("string descriptor index 0"));
    }
    let std_in =
        |value, index| ControlRequest { request_type: 0, request: GET_DESCRIPTOR, value, index };
    let langs = t.control_in(std_in(STRING, 0), 4, TIMEOUT)?;
    if langs.len() < 4 {
        return Err(Error::device("short string descriptor language table"));
    }
    let lang = u16::from_le_bytes([langs[2], langs[3]]);
    let d = t.control_in(std_in(STRING | u16::from(index), lang), 255, TIMEOUT)?;
    if d.len() < 2 || d[1] != 3 || usize::from(d[0]) > d.len() {
        return Err(Error::device("bad string descriptor"));
    }
    Ok(d[2..usize::from(d[0])]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|&[lo, hi]| if hi == 0 && lo < 0x80 { char::from(lo) } else { '?' })
        .collect())
}

/// The name libusb's `libusb_error_name()` gives the error a transfer
/// failed with, for drivers that log as their libsigrok counterpart
/// does: a STALL is `LIBUSB_ERROR_PIPE`, a vanished device
/// `LIBUSB_ERROR_NO_DEVICE`, anything not mapped `LIBUSB_ERROR_IO`.
pub fn libusb_error_name(e: &Error) -> &'static str {
    match e {
        Error::Timeout => "LIBUSB_ERROR_TIMEOUT",
        Error::Disconnected => "LIBUSB_ERROR_NO_DEVICE",
        Error::Busy => "LIBUSB_ERROR_BUSY",
        Error::Device(m) if m.contains("stalled") => "LIBUSB_ERROR_PIPE",
        Error::Io(io) if io.kind() == std::io::ErrorKind::PermissionDenied => "LIBUSB_ERROR_ACCESS",
        Error::NotSupported(_) => "LIBUSB_ERROR_NOT_SUPPORTED",
        _ => "LIBUSB_ERROR_IO",
    }
}

/// A queue of bulk IN transfers, see [`Transport::bulk_in_stream`].
pub trait BulkStream: Send {
    /// The next completed transfer, in order. A short transfer is not
    /// an error; nothing completing within `timeout` is
    /// [`Error::Timeout`].
    fn next(&mut self, timeout: Duration) -> Result<Vec<u8>>;
}

/// One transfer a backend's queue gave back: the bytes it holds, and
/// how it ended ([`Error::Timeout`] when it was cancelled).
#[cfg_attr(not(any(feature = "nusb", test)), allow(dead_code))]
pub(crate) struct Completed {
    pub(crate) data: Vec<u8>,
    pub(crate) status: Result<()>,
}

/// What [`timed_next`] needs of a backend's queue of transfers.
#[cfg_attr(not(any(feature = "nusb", test)), allow(dead_code))]
pub(crate) trait TransferQueue {
    /// Submits one more transfer.
    fn submit(&mut self);
    /// The next completed transfer, in order.
    fn wait(&mut self, timeout: Duration) -> Option<Completed>;
    /// Cancels the transfers in flight, which complete all the same.
    fn cancel_all(&mut self);
    /// Transfers not yet given back.
    fn pending(&self) -> usize;
}

/// [`BulkStream::next`] of a timed stream, see
/// [`Transport::bulk_in_stream_timed`]. `owed` says the transfer handed
/// out last is still to be resubmitted. The cancelled transfers' data
/// comes out as one piece, in order; the first of them is then owed like
/// any transfer handed out, the others go back in at once.
#[cfg_attr(not(any(feature = "nusb", test)), allow(dead_code))]
pub(crate) fn timed_next(
    q: &mut impl TransferQueue,
    depth: usize,
    owed: &mut bool,
    timeout: Duration,
    transfer_timeout: Duration,
) -> Result<Vec<u8>> {
    /// How long cancelled transfers may take to come back.
    const CANCEL_WAIT: Duration = Duration::from_secs(1);
    if std::mem::take(owed) {
        q.submit();
    }
    let wait = timeout.min(transfer_timeout);
    if let Some(c) = q.wait(wait) {
        *owed = true;
        return match c.status {
            Err(e) if c.data.is_empty() => Err(e),
            _ => Ok(c.data),
        };
    }
    if wait < transfer_timeout {
        return Err(Error::Timeout);
    }
    q.cancel_all();
    let mut data = Vec::new();
    let mut failure = None;
    while q.pending() > 0 {
        let Some(c) = q.wait(CANCEL_WAIT) else { break };
        data.extend_from_slice(&c.data);
        match c.status {
            Ok(()) | Err(Error::Timeout) => {}
            Err(e) => failure = failure.or(Some(e)),
        }
    }
    let handed_out = !data.is_empty() || failure.is_some();
    *owed = handed_out;
    for _ in q.pending() + usize::from(handed_out)..depth {
        q.submit();
    }
    match failure {
        _ if !data.is_empty() => Ok(data),
        Some(e) => Err(e),
        None => Err(Error::Timeout),
    }
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
    if crate::core::sessions::replaying() {
        return replay_devices();
    }
    match Backend::current() {
        #[cfg(feature = "nusb")]
        Some(Backend::Nusb) => nusb_backend::list(),
        #[cfg(feature = "libusb")]
        Some(Backend::Libusb) => rusb_backend::list(),
        _ => Err(Error::not_supported("no USB backend compiled in")),
    }
}

const REPLAY_VAR: &str = "OXROK_USB_REPLAY";

/// With `OXROK_USB_REPLAY=FILE` (or `OXROK_HID_REPLAY`) set: the device
/// of the file's first session, which [`open`] serves from the file,
/// else none.
fn replay_devices() -> Result<Vec<UsbDeviceInfo>> {
    let Some(text) = crate::core::sessions::first_session(REPLAY_VAR, record::MAGIC)? else {
        return Ok(Vec::new());
    };
    let mut info = record::Fixture::parse(&text)?.device;
    info.handle = Handle::Replay;
    Ok(vec![info])
}

/// Whether listings show the devices of a replay file (the
/// `OXROK_USB_REPLAY` or `OXROK_HID_REPLAY` variable) rather than the
/// machine's.
pub fn replaying() -> bool {
    crate::core::sessions::replaying()
}

/// The devices with one of the given ids.
pub fn find_devices(ids: &[UsbId]) -> Result<Vec<UsbDeviceInfo>> {
    Ok(list_devices()?.into_iter().filter(|d| ids.contains(&d.id())).collect())
}

/// Opens a device found by [`list_devices`].
///
/// With `OXROK_USB_RECORD=FILE` in the environment the session is
/// recorded and written to `FILE` when the transport is dropped; later
/// sessions of the same process are appended. A device listed from an
/// `OXROK_USB_REPLAY` file is served the file's next session.
pub fn open(info: &UsbDeviceInfo) -> Result<Box<dyn Transport>> {
    let transport = match &info.handle {
        #[cfg(feature = "nusb")]
        Handle::Nusb(_) => nusb_backend::open(info)?,
        #[cfg(feature = "libusb")]
        Handle::Rusb { .. } => rusb_backend::open(info)?,
        Handle::Replay => {
            let text = crate::core::sessions::next_session(REPLAY_VAR, record::MAGIC)?;
            let replay = record::Replay::from_text(&text)?;
            let check = replay.clone();
            crate::core::sessions::register(move || check.finish());
            return Ok(Box::new(replay));
        }
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
    fn conn_specs_follow_sr_usb_split_conn() {
        let invalid = Err("Invalid input, or neither VID:PID nor bus.address specified.");
        let neither = Err("Could neither determine VID:PID nor bus.address numbers.");
        assert_eq!(split_conn("21a9.1001"), Ok(Conn::Ids(0x21a9, 0x1001)));
        assert_eq!(split_conn("1000.1000"), Ok(Conn::Ids(0x1000, 0x1000)));
        assert_eq!(split_conn("3.7"), Ok(Conn::BusAddress(3, 7)));
        assert_eq!(split_conn("255.127"), Ok(Conn::BusAddress(255, 127)));
        assert_eq!(split_conn("3.128"), invalid);
        assert_eq!(split_conn("256.1"), invalid);
        assert_eq!(split_conn("1.99999999999999999999999"), invalid);
        assert_eq!(split_conn("0.0"), neither);
        assert_eq!(split_conn("0.5"), neither);
        assert_eq!(split_conn("0000.1001"), neither);
        assert_eq!(split_conn("21a.1001"), neither);
        assert_eq!(split_conn("nonsense"), neither);
        assert_eq!(split_conn("1.x"), neither);
        assert_eq!(split_conn("1.2.3"), neither);
    }

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
    fn strings_read_as_libusb_does() {
        let text = "# oxrok usb fixture 1\n\
            device vid=0925 pid=3881\n\
            ctl_in 00 06 0300 0000 4 -> 04030904\n\
            ctl_in 00 06 0301 0409 255 -> 0a03730069006700e900\n";
        let mut r = record::Replay::from_text(text).unwrap();
        // Non-ASCII made '?', index 0 refused without a transfer.
        assert_eq!(read_string_ascii(&mut r, 1).unwrap(), "sig?");
        assert!(read_string_ascii(&mut r, 0).is_err());
        r.finish().unwrap();
    }

    /// A queue of `transfers` in flight whose first holds `partial`
    /// bytes when cancelled, unless a completion is scripted.
    #[derive(Default)]
    struct FakeQueue {
        in_flight: usize,
        submitted: usize,
        cancelled: bool,
        partial: Vec<u8>,
        script: Vec<Completed>,
    }

    impl TransferQueue for FakeQueue {
        fn submit(&mut self) {
            self.in_flight += 1;
            self.submitted += 1;
        }
        fn wait(&mut self, _timeout: Duration) -> Option<Completed> {
            if !self.script.is_empty() {
                self.in_flight -= 1;
                return Some(self.script.remove(0));
            }
            if !self.cancelled || self.in_flight == 0 {
                return None;
            }
            self.in_flight -= 1;
            Some(Completed { data: std::mem::take(&mut self.partial), status: Err(Error::Timeout) })
        }
        fn cancel_all(&mut self) {
            self.cancelled = true;
        }
        fn pending(&self) -> usize {
            self.in_flight
        }
    }

    const T: Duration = Duration::from_millis(500);

    #[test]
    fn a_timed_stream_gives_the_partial_data_and_resubmits() {
        let mut q = FakeQueue { in_flight: 8, partial: vec![1; 512], ..FakeQueue::default() };
        let mut owed = false;
        assert_eq!(timed_next(&mut q, 8, &mut owed, T, T).unwrap(), vec![1; 512]);
        // The one handed out is owed, as a completed transfer is; the
        // seven others are back in flight.
        assert!(owed);
        assert_eq!((q.in_flight, q.submitted), (7, 7));
        // Asked again: the owed one goes in, and nothing comes this time.
        q.cancelled = false;
        let e = timed_next(&mut q, 8, &mut owed, T, T);
        assert!(matches!(e, Err(Error::Timeout)));
        assert!(!owed);
        assert_eq!(q.in_flight, 8);
    }

    #[test]
    fn a_timed_stream_is_a_plain_one_while_transfers_complete() {
        let mut q = FakeQueue {
            in_flight: 2,
            script: vec![Completed { data: vec![7; 4], status: Ok(()) }],
            ..FakeQueue::default()
        };
        let mut owed = false;
        assert_eq!(timed_next(&mut q, 2, &mut owed, T, T).unwrap(), vec![7; 4]);
        assert!(owed && !q.cancelled);
        assert_eq!(q.in_flight, 1);
    }

    #[test]
    fn a_timed_stream_waits_no_longer_than_asked_without_cancelling() {
        let mut q = FakeQueue { in_flight: 2, ..FakeQueue::default() };
        let mut owed = false;
        let short = Duration::from_millis(1);
        assert!(matches!(timed_next(&mut q, 2, &mut owed, short, T), Err(Error::Timeout)));
        assert!(!q.cancelled);
    }

    #[test]
    fn a_timed_stream_reports_a_failure_found_while_cancelling() {
        // Nothing completes at first; on the cancel the device is gone.
        struct Gone(FakeQueue);
        impl TransferQueue for Gone {
            fn submit(&mut self) {
                self.0.submit();
            }
            fn wait(&mut self, t: Duration) -> Option<Completed> {
                if !self.0.cancelled {
                    return None;
                }
                self.0.wait(t).map(|c| Completed { status: Err(Error::Disconnected), ..c })
            }
            fn cancel_all(&mut self) {
                self.0.cancel_all();
            }
            fn pending(&self) -> usize {
                self.0.pending()
            }
        }
        let mut g = Gone(FakeQueue { in_flight: 2, ..FakeQueue::default() });
        let mut owed = false;
        assert!(matches!(timed_next(&mut g, 2, &mut owed, T, T), Err(Error::Disconnected)));
        assert!(owed);
    }

    #[test]
    fn libusb_error_names() {
        assert_eq!(libusb_error_name(&Error::device("endpoint stalled")), "LIBUSB_ERROR_PIPE");
        assert_eq!(libusb_error_name(&Error::Disconnected), "LIBUSB_ERROR_NO_DEVICE");
        assert_eq!(libusb_error_name(&Error::Timeout), "LIBUSB_ERROR_TIMEOUT");
        assert_eq!(libusb_error_name(&Error::device("babble")), "LIBUSB_ERROR_IO");
    }

    #[test]
    fn request_fields() {
        let r = ControlRequest::vendor(0x0b, 0x0107, 1);
        assert_eq!(r.kind(), 2);
        assert_eq!(r.recipient(), 0);
        assert_eq!(ControlRequest::vendor_interface(1, 0, 0).recipient(), 1);
    }
}
