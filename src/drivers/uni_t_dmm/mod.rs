// Counterpart in sigrok:
//   libsigrok/src/hardware/uni-t-dmm/api.c,
//   libsigrok/src/hardware/uni-t-dmm/protocol.c,
//   libsigrok/src/hardware/uni-t-dmm/protocol.h,
//   Copyright (C) 2012-2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Gerhard Sittig, Bert Vermeulen,
//   Daniel Elstner, Martin Ling, Soeren Apel, Elen Eisendle, Karl Palsson
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The meters of libsigrok's `uni-t-dmm` driver on their UT-D04 cable,
//! reached as that driver reaches them: raw USB, with the HID chip set
//! up and read by hand.
//!
//! The cable is a WCH CH9325 (or, on old ones, a Hoitek HE2325U) UART to
//! HID bridge. A poll is one interrupt IN transfer of 8 bytes on endpoint
//! `0x82`: `0xf0 | n`, then `n` UART bytes (at most 7), padded. A meter's
//! packet thus spans reports, and the driver looks for it in the bytes it
//! has so far.
//!
//! The sequence, all of it on the first poll, after the header:
//!
//! 1. Detach a kernel driver bound to interface 0, for good: `usbhid` is
//!    not given it back at the end (the C leaves it detached too, so the
//!    `hidraw` node stays away until a replug).
//! 2. Claim interface 0.
//! 3. `SET_REPORT` of feature report 0 with `baud_lo baud_hi 00 00 03`.
//!    The 3 is 8 data bits, whatever the meter's frame.
//!
//! So on the 7o1 meters (`ES519xx`, `UT71x`) the parity bit arrives in bit 7
//! of each byte and is masked off here. The other meters are 8n1.
//!
//! Each id has a `serial_dmm` twin on the other route. `conn=VID.PID`
//! and `conn=BUS.ADDRESS` come here; no `conn=` and `conn=hid...`, which
//! libsigrok's driver does not have, go to the twin through the HID
//! layer, as they did before this driver existed. So do ids of another
//! chip (a CP2110 cannot take a CH9325 report, though the C sends it),
//! and off Linux a CH9325's ids when the HID layer sees that cable, since
//! a program cannot claim a HID interface there (nusb: busy on macOS,
//! unsupported on Windows with `HidUsb`). A cable bound to `WinUSB`, as
//! Zadig does, is no HID device any more and stays on raw USB.
//!
//! The parsers are `crate::dmm`'s, shared with `serial_dmm`, and the
//! table is `serial_dmm::models::hid_models`.
//!
//! What is deliberately not as in the C:
//! - a report's count nibble above 7 is cut to 7 (protocol.c:
//!   `buf[1 + i]` for `i` up to 14 reads past the 8 byte `buf`);
//! - `dev_close` releases the interface and closes the device, where the
//!   C's is a dummy (`std_dummy_dev_close /* TODO */`), so a second open
//!   sets the chip up again;
//! - the log's `DMM packet` line holds the packet, where the C's always
//!   holds 14 bytes (`log_dmm_packet`).

#![forbid(unsafe_code)]

use super::serial_dmm::SerialDmmDriver;
use super::serial_dmm::models::Model;
use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::Params;
use crate::serial::hid::Chip;
use crate::usb::{Conn, ControlRequest, Transport, UsbDeviceInfo, libusb_error_name, split_conn};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
/// `serialcomm` serves the HID route; the C's list has `conn` alone.
const ACCEPTED_SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];

const INTERFACE: u8 = 0;
const ENDPOINT_IN: u8 = 0x82;
/// `CHUNK_SIZE`: a report.
const CHUNK_SIZE: usize = 8;
/// UART bytes a report can hold.
const CHUNK_DATA: usize = CHUNK_SIZE - 1;
const USB_TIMEOUT: Duration = Duration::from_millis(1000);
/// The session source's timeout in `dev_acquisition_start`.
const POLL_INTERVAL: Duration = Duration::from_millis(10);

/// The driver of one meter.
pub struct UniTDmmDriver(pub &'static Model);

/// What a `conn=` of these ids means.
#[derive(Debug, PartialEq, Eq)]
enum Route {
    /// The cable through the HID layer: the `serial_dmm` twin.
    Hid,
    /// Raw USB, as libsigrok's driver.
    Usb(Conn),
    /// Nothing `sr_usb_find` can look for, with its message.
    Invalid(&'static str),
}

/// `on_hid` says whether these ids are better reached through the HID
/// layer here: [`cable_on_hid`].
fn route(conn: Option<&str>, on_hid: impl Fn(u16, u16) -> bool) -> Route {
    match conn {
        None => Route::Hid,
        Some(c) if crate::serial::hid::is_hid_name(c) => Route::Hid,
        Some(c) => match split_conn(c) {
            Ok(Conn::Ids(v, p)) if other_chip(v, p) || on_hid(v, p) => Route::Hid,
            Ok(c) => Route::Usb(c),
            Err(msg) => Route::Invalid(msg),
        },
    }
}

/// A known cable that is no CH9325 or HE2325U, whose report this is not.
fn other_chip(vid: u16, pid: u16) -> bool {
    Chip::for_usb_id(vid, pid).is_some_and(|c| !matches!(c, Chip::Ch9325))
}

/// Off Linux (a replay stands in for raw USB everywhere): the HID layer
/// sees this known cable, so a claim of it could only fail. Decided here
/// alone, so that a capture's calls are the same on every system.
fn cable_on_hid(vid: u16, pid: u16) -> bool {
    !cfg!(target_os = "linux")
        && !crate::usb::replaying()
        && Chip::for_usb_id(vid, pid).is_some()
        && crate::hid::list()
            .is_ok_and(|l| l.iter().any(|d| d.vendor_id == vid && d.product_id == pid))
}

/// The driver for a serial port, for a `conn=` that is none of this
/// driver's: the UT372 has no such twin.
fn serial_hint(id: &str) -> Option<String> {
    let ser = format!("{id}-ser");
    super::serial_dmm::models::find(&ser).map(|_| format!("For a serial port, use {ser}."))
}

impl Driver for UniTDmmDriver {
    fn id(&self) -> &'static str {
        self.0.id
    }

    fn long_name(&self) -> &'static str {
        self.0.long_name()
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn accepted_scan_options(&self) -> &'static [Key] {
        &ACCEPTED_SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let conn = options.iter().find_map(|(k, v)| match k {
            Key::Conn => v.as_str(),
            _ => None,
        });
        let conn = match route(conn, cable_on_hid) {
            Route::Hid => return SerialDmmDriver(self.0).scan(options),
            Route::Invalid(msg) => {
                log::error!("{msg}");
                if let Some(hint) = serial_hint(self.0.id) {
                    log::warn!("{hint}");
                }
                return Ok(Vec::new());
            }
            Route::Usb(c) => c,
        };
        // Only the HID route has a line to set: sr_driver_scan refuses any
        // option the driver does not list (hwdriver.c check_options).
        if options.iter().any(|(k, _)| matches!(k, Key::Serialcomm)) {
            log::error!("serialcomm does not apply to raw USB: conn=hid/ch9325 takes it.");
            return Ok(Vec::new());
        }
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::list_devices()? {
            if conn.matches(&info) {
                devices.push(Box::new(UniTDmmDevice::new(self.0, Some(info), None)));
            }
        }
        Ok(devices)
    }
}

/// The line setting of `hid_chip_init`.
fn line_report(baud: u16) -> [u8; 5] {
    let [lo, hi] = baud.to_le_bytes();
    [lo, hi, 0x00, 0x00, 0x03]
}

/// Whether the meter's line has a parity bit, which the chip passes in
/// bit 7: `get_and_handle_data` names the `ES519xx` and `UT71x` parsers, the
/// table's 7o1 meters.
fn masks_parity(params: &Params) -> bool {
    params.data_bits == 7
}

/// What a report holds: its UART bytes.
fn report_data(report: &[u8; CHUNK_SIZE]) -> &[u8] {
    let n = usize::from(report[0] & 0x0f).min(CHUNK_DATA);
    &report[1..=n]
}

/// The meter's packets in what has arrived: `get_and_handle_data`'s
/// search, stepping a byte at a time over what starts no packet. Hands
/// each packet to `found` and keeps what may be the start of the next.
fn take_packets(
    buf: &mut Vec<u8>,
    size: usize,
    valid: fn(&[u8]) -> bool,
    mut found: impl FnMut(&[u8]),
) {
    let mut pos = 0;
    while buf.len() - pos >= size {
        if valid(&buf[pos..pos + size]) {
            found(&buf[pos..pos + size]);
            pos += size;
        } else {
            pos += 1;
        }
    }
    buf.drain(..pos);
}

/// One meter on its cable.
pub struct UniTDmmDevice {
    model: &'static Model,
    device_info: DeviceInfo,
    channels: Vec<Channel>,
    usb: Option<UsbDeviceInfo>,
    limit_samples: u64,
    limit_msec: u64,
    /// The open device, while no capture has it.
    link: Arc<Mutex<Option<Link>>>,
    pending: Option<Box<dyn Transport>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

struct Link {
    transport: Box<dyn Transport>,
    /// `first_run` is over: the interface is claimed, the line set.
    ready: bool,
    claimed: bool,
    /// The bytes of a packet not whole yet (`protocol_buf`).
    buf: Vec<u8>,
}

impl UniTDmmDevice {
    fn new(
        model: &'static Model,
        usb: Option<UsbDeviceInfo>,
        pending: Option<Box<dyn Transport>>,
    ) -> Self {
        UniTDmmDevice {
            model,
            device_info: DeviceInfo {
                driver: model.id,
                vendor: model.vendor.to_owned(),
                model: model.model.to_owned(),
                version: String::new(),
                serial: String::new(),
                // config_get is NULL in the C, so nothing shows it.
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "P1".to_owned())],
            usb,
            limit_samples: 0,
            limit_msec: 0,
            link: Arc::new(Mutex::new(None)),
            pending,
            worker: None,
        }
    }

    /// A device of `model` on an already open transport, e.g. a replay.
    pub fn from_transport(model: &'static Model, transport: Box<dyn Transport>) -> Self {
        Self::new(model, None, Some(transport))
    }
}

impl Device for UniTDmmDevice {
    fn info(&self) -> &DeviceInfo {
        &self.device_info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
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
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    /// The C driver has no `config_get`, although it lists the limits as
    /// readable: `sr_config_get` answers "invalid argument".
    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        Err(Error::Arg(format!("{} cannot be read", key.id())))
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => self.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::LimitMsec => self.limit_msec = value.as_u64().ok_or_else(bad)?,
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
        let mut link = self.link.lock().expect("link lock");
        if link.is_some() {
            return Ok(());
        }
        let transport = if let Some(t) = self.pending.take() {
            t
        } else {
            let info = self.usb.as_ref().ok_or_else(|| Error::device("device is gone"))?;
            crate::usb::open(info)?
        };
        *link = Some(Link { transport, ready: false, claimed: false, buf: Vec::new() });
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(mut link) = self.link.lock().expect("link lock").take()
            && link.claimed
            && let Err(e) = link.transport.release_interface(INTERFACE)
        {
            log::debug!("Failed to release interface {INTERFACE}: {e}.");
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let link = self.link.lock().expect("link lock").take();
        let Some(mut link) = link else {
            return Err(Error::device("device is not open"));
        };
        let acq = Acquisition {
            model: self.model,
            params: Params::parse(self.model.serialcomm)?,
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
        };
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name(format!("{} acquisition", self.model.id))
            .spawn(move || {
                let outcome = acq.run(&mut link, &sink, &stop2);
                sink.end(outcome);
                *slot.lock().expect("link lock") = Some(link);
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

/// One acquisition, on its own thread.
struct Acquisition {
    model: &'static Model,
    params: Params,
    limit_samples: u64,
    limit_msec: u64,
}

impl Acquisition {
    /// `hid_chip_init`, with its messages.
    fn chip_init(&self, link: &mut Link) -> Result<()> {
        if let Err(e) = link.transport.detach_kernel_driver(INTERFACE) {
            log::error!("Failed to detach kernel driver: {}.", libusb_error_name(&e));
            return Err(e);
        }
        if let Err(e) = link.transport.claim_interface(INTERFACE) {
            log::error!("Failed to claim interface {INTERFACE}: {}.", libusb_error_name(&e));
            if !cfg!(target_os = "linux") {
                log::error!("The system may hold on to a HID cable: try conn=hid/ch9325.");
            }
            return Err(e);
        }
        link.claimed = true;
        let report = line_report(self.params.baud as u16);
        log::debug!("Sending initial HID feature report.");
        // Class, interface; bRequest 9 is SET_REPORT, wValue the feature
        // report 0, wIndex interface 0.
        let set_report = ControlRequest { request_type: 0x21, request: 9, value: 0x300, index: 0 };
        if let Err(e) = link.transport.control_out(set_report, &report, USB_TIMEOUT) {
            log::error!("HID feature report error: {}.", libusb_error_name(&e));
            return Err(e);
        }
        Ok(())
    }

    /// `get_and_handle_data`: one poll. `Ok(false)` when the sink is gone.
    fn poll(&self, link: &mut Link, sink: &PacketSink, samples: &mut u64) -> Result<bool> {
        let got = link.transport.interrupt_in(ENDPOINT_IN, CHUNK_SIZE, USB_TIMEOUT)?;
        let Ok(report) = <[u8; CHUNK_SIZE]>::try_from(got.as_slice()) else {
            log::error!("Short packet: received {}/{CHUNK_SIZE} bytes.", got.len());
            return Err(Error::device("short report"));
        };
        if report[0] == 0xf0 {
            return Ok(true);
        }
        let mask = if masks_parity(&self.params) { 0x7f } else { 0xff };
        link.buf.extend(report_data(&report).iter().map(|b| b & mask));
        let p = &self.model.protocol;
        let mut open = true;
        take_packets(&mut link.buf, p.packet_size, p.valid, |packet| {
            log::debug!("DMM packet: {packet:02x?}");
            let readings = (p.parse)(packet);
            if readings.is_empty() {
                log::debug!("Invalid DMM packet, ignoring.");
            }
            for r in readings {
                open &= sink.send(Packet::Analog(AnalogData {
                    channels: vec![r.channel],
                    samples: vec![r.value],
                    mq: r.mq,
                    flags: r.flags,
                    unit: r.unit,
                    digits: r.digits,
                }));
                *samples += 1;
            }
        });
        Ok(open)
    }

    /// `uni_t_dmm_receive_data` until a limit, a stop or an error. A
    /// stop may have to wait for the read in progress (up to its 1 s).
    fn run(&self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        sink.send(Packet::header());
        let started = Instant::now();
        let limit = Duration::from_millis(self.limit_msec);
        let mut samples = 0u64;
        loop {
            std::thread::sleep(POLL_INTERVAL);
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            if !link.ready {
                if let Err(e) = self.chip_init(link) {
                    log::error!("HID chip init failed: -1.");
                    return Err(e);
                }
                link.ready = true;
                link.buf.clear();
            }
            match self.poll(link, sink, &mut samples) {
                Ok(true) => {}
                Ok(false) => return Ok(()),
                Err(_) if stop.load(Ordering::SeqCst) => return Ok(()),
                Err(Error::Device(m)) if m == "short report" => return Err(Error::Device(m)),
                Err(e) => {
                    log::error!("USB receive error: {}.", libusb_error_name(&e));
                    return Err(e);
                }
            }
            if self.limit_samples > 0 && samples >= self.limit_samples {
                return Ok(());
            }
            if self.limit_msec > 0 && started.elapsed() >= limit {
                return Ok(());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::serial_dmm::models;
    use super::*;
    use crate::usb::InterfaceDesc;
    use std::sync::atomic::AtomicUsize;
    use std::sync::mpsc::sync_channel;

    #[test]
    fn conn_picks_the_route() {
        let route = |c| route(c, |_, _| false);
        assert_eq!(route(None), Route::Hid);
        assert_eq!(route(Some("hid/ch9325")), Route::Hid);
        assert_eq!(route(Some("hid")), Route::Hid);
        assert_eq!(route(Some("1a86.e008")), Route::Usb(Conn::Ids(0x1a86, 0xe008)));
        assert_eq!(route(Some("04fa.2490")), Route::Usb(Conn::Ids(0x04fa, 0x2490)));
        assert_eq!(route(Some("1234.5678")), Route::Usb(Conn::Ids(0x1234, 0x5678)));
        assert_eq!(route(Some("3.7")), Route::Usb(Conn::BusAddress(3, 7)));
        assert!(matches!(route(Some("notusb")), Route::Invalid(_)));
        assert!(matches!(route(Some("/dev/ttyUSB0")), Route::Invalid(_)));
        // The other chips' ids go to the twin, which knows their reports.
        assert_eq!(route(Some("10c4.ea80")), Route::Hid);
        assert_eq!(route(Some("0820.0001")), Route::Hid);
        assert_eq!(route(Some("1244.d237")), Route::Hid);
    }

    #[test]
    fn off_linux_a_cable_the_hid_layer_sees_goes_there() {
        let seen = |v, p| (v, p) == (0x1a86, 0xe008);
        assert_eq!(route(Some("1a86.e008"), seen), Route::Hid);
        // A cable on WinUSB is no HID device, and an address names no ids.
        assert_eq!(route(Some("1a86.e008"), |_, _| false), Route::Usb(Conn::Ids(0x1a86, 0xe008)));
        assert_eq!(route(Some("3.7"), |_, _| true), Route::Usb(Conn::BusAddress(3, 7)));
        // On Linux and in a replay raw USB stays, as sigrok-cli's command.
        if cfg!(target_os = "linux") {
            assert!(!cable_on_hid(0x1a86, 0xe008));
        }
    }

    #[test]
    fn a_conn_that_is_none_points_to_the_serial_twin() {
        assert_eq!(serial_hint("uni-t-ut61e").unwrap(), "For a serial port, use uni-t-ut61e-ser.");
        assert_eq!(serial_hint("uni-t-ut372"), None);
    }

    /// A chip that answers every poll for ever, as a real one does.
    struct AlwaysAnswers {
        info: UsbDeviceInfo,
        polls: Arc<AtomicUsize>,
    }

    impl Transport for AlwaysAnswers {
        fn info(&self) -> &UsbDeviceInfo {
            &self.info
        }
        fn set_configuration(&mut self, _: u8) -> Result<()> {
            Ok(())
        }
        fn claim_interface(&mut self, _: u8) -> Result<()> {
            Ok(())
        }
        fn release_interface(&mut self, _: u8) -> Result<()> {
            Ok(())
        }
        fn reset(&mut self) -> Result<()> {
            Ok(())
        }
        fn max_packet_size(&mut self, _: u8) -> Result<usize> {
            Ok(CHUNK_SIZE)
        }
        fn interfaces(&mut self) -> Result<Vec<InterfaceDesc>> {
            Ok(Vec::new())
        }
        fn control_out(&mut self, _: ControlRequest, _: &[u8], _: Duration) -> Result<()> {
            Ok(())
        }
        fn control_in(&mut self, _: ControlRequest, _: usize, _: Duration) -> Result<Vec<u8>> {
            Ok(Vec::new())
        }
        fn bulk_out(&mut self, _: u8, d: &[u8], _: Duration) -> Result<usize> {
            Ok(d.len())
        }
        fn bulk_in(&mut self, _: u8, _: usize, _: Duration) -> Result<Vec<u8>> {
            Ok(Vec::new())
        }
        fn interrupt_out(&mut self, _: u8, d: &[u8], _: Duration) -> Result<usize> {
            Ok(d.len())
        }
        fn interrupt_in(&mut self, _: u8, _: usize, _: Duration) -> Result<Vec<u8>> {
            self.polls.fetch_add(1, Ordering::SeqCst);
            Ok(vec![0xf0, 0, 0, 0, 0, 0, 0, 0])
        }
        fn bulk_in_stream(
            &mut self,
            _: u8,
            _: usize,
            _: usize,
        ) -> Result<Box<dyn crate::usb::BulkStream>> {
            Err(Error::not_supported("bulk stream"))
        }
    }

    /// The stop of a real Ctrl-C: the chip goes on answering, so no read
    /// fails, and only the check at the top of the poll loop ends it.
    #[test]
    fn a_stop_ends_a_capture_on_a_chip_that_keeps_answering() {
        let polls = Arc::new(AtomicUsize::new(0));
        let transport = AlwaysAnswers { info: UsbDeviceInfo::default(), polls: Arc::clone(&polls) };
        let model = models::find("uni-t-ut61e").unwrap();
        let mut device = UniTDmmDevice::from_transport(model, Box::new(transport));
        device.open().unwrap();
        let (tx, _rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let start = Instant::now();
        while polls.load(Ordering::SeqCst) < 3 {
            assert!(start.elapsed() < Duration::from_secs(10), "no polls");
            std::thread::sleep(Duration::from_millis(1));
        }
        let (done, stopped) = sync_channel(1);
        std::thread::spawn(move || {
            device.stop().unwrap();
            done.send(()).unwrap();
        });
        assert!(stopped.recv_timeout(Duration::from_secs(10)).is_ok(), "the capture ran on");
    }

    #[test]
    fn line_report_is_hid_chip_inits() {
        assert_eq!(line_report(19200), [0x00, 0x4b, 0, 0, 3]);
        assert_eq!(line_report(2400), [0x60, 0x09, 0, 0, 3]);
        assert_eq!(line_report(9600), [0x80, 0x25, 0, 0, 3]);
    }

    #[test]
    fn report_data_follows_the_count() {
        assert_eq!(report_data(&[0xf0, 1, 2, 3, 4, 5, 6, 7]), &[] as &[u8]);
        assert_eq!(report_data(&[0xf2, 0x55, 0x77, 0, 0, 0, 0, 0]), &[0x55, 0x77]);
        assert_eq!(report_data(&[0xf7, 1, 2, 3, 4, 5, 6, 7]), &[1, 2, 3, 4, 5, 6, 7]);
        // The C goes by the low nibble alone.
        assert_eq!(report_data(&[0x01, 9, 0, 0, 0, 0, 0, 0]), &[9]);
        // More than a report holds: the C reads past its buffer.
        assert_eq!(report_data(&[0xff, 1, 2, 3, 4, 5, 6, 7]), &[1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn packets_are_found_across_noise_and_reports() {
        fn valid(p: &[u8]) -> bool {
            p[0] == 0xaa
        }
        let mut seen = Vec::new();
        let mut buf = vec![1, 2, 0xaa, 3];
        take_packets(&mut buf, 3, valid, |p| seen.push(p.to_vec()));
        assert_eq!(seen, Vec::<Vec<u8>>::new());
        assert_eq!(buf, [0xaa, 3], "a start is kept");
        buf.push(4);
        buf.extend([0xaa, 5, 6, 7, 8]);
        take_packets(&mut buf, 3, valid, |p| seen.push(p.to_vec()));
        assert_eq!(seen, [vec![0xaa, 3, 4], vec![0xaa, 5, 6]]);
        assert_eq!(buf, [7, 8], "fewer than a packet, kept as the C keeps them");
    }

    #[test]
    fn the_table_is_the_cs() {
        let table = models::hid_models();
        assert_eq!(table.len(), 27);
        let baud = |id: &str| Params::parse(models::find(id).unwrap().serialcomm).unwrap();
        // Note 1 of api.c: 19230 on the wire, 19200 for the chip.
        assert_eq!(baud("uni-t-ut61e").baud, 19200);
        assert_eq!(baud("tenma-72-7750").baud, 19200);
        assert_eq!(baud("voltcraft-vc870").baud, 9600);
        for m in table {
            let p = Params::parse(m.serialcomm).unwrap();
            assert!(p.baud <= 19200, "{}", m.id);
            assert!(models::find(m.id).is_some());
            let es = matches!(m.id, "tenma-72-7750" | "uni-t-ut60g" | "uni-t-ut61e");
            let ut71 = m.protocol.packet_size == 11;
            assert_eq!(masks_parity(&p), es || ut71, "{}", m.id);
        }
    }
}
