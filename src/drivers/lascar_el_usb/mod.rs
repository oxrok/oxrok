// Counterpart in sigrok:
//   libsigrok/src/hardware/lascar-el-usb/api.c,
//   libsigrok/src/hardware/lascar-el-usb/protocol.c (the receive callback),
//   eltuxusb (Romain Aviolat), GPLv3, version 0.3,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Daniel Elstner,
//   Frank Stettner, Martin Ling, Soeren Apel, Aurelien Jacobs,
//   Daniel Glöckner, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! lascar-el-usb: Lascar `EasyLog` EL-USB data loggers (`10c4:0002`). The
//! logger keeps a log in its EEPROM; a capture downloads it, it does not
//! stream. The scan takes `conn=`, as the id is `SiLabs`' and no Lascar's,
//! and tells the model from the first byte of the configuration block.
//! Temperature and humidity models (EL-USB-2, 2-LCD, 2+, 2-LCD+) have the
//! channels `Temp` and `Hum`, the CO ones (EL-USB-CO, CO300) `CO`. The others
//! of the C table are found and refused, as in C. Its protocol is in
//! [`protocol`].
//!
//! Every read and write follows `api.c` and `protocol.c` (checked with the
//! emulator, `tools/emu/lascar-el-usb.sh`), except where the C is wrong:
//! - half degrees and half percents, lost to integer divisions at
//!   `protocol.c:426`, 430, are kept;
//! - `--samples` cuts the output; `api.c:174` stores it and nothing reads it;
//! - a download whose device stops, or whose endpoint is halted, ends after
//!   [`MAX_FAILED_READS`] reads in a row bring nothing; `protocol.c:558`
//!   goes on for ever;
//! - nothing past the buffer: `api.c:312` reads 256 bytes into `resp[4]`, and
//!   a device with enough queued kills it;
//! - an invalid configuration header leaves the device open:
//!   `protocol.c:155` closes it, and everyone goes on using the handle.
//! - `Temp` and `Hum` have their own indexes, both 0 at `protocol.c:336`.
//!
//! What it keeps from C:
//! - CO's `/ 1e6` and its 0 digits (`protocol.c:484`). `libsigrok.h:293-298`
//!   defines `SR_UNIT_CONCENTRATION` as normalised 0 to 1, so the division is
//!   deliberate, while `src/analog.c:70` labels the unit "ppm". The `analog` output
//!   shows 0 for all realistic readings: anything under 500,000 ppm, if `s *
//!   high + low` is ppm, which is inferred. Use `-O csv` and multiply by 1e6.
//!   6 digits would print a mislabelled `0.000155 ppm`.
//! - `datalog=on` sets bit 0 of 0x21 and zeroes 0x18 to 0x1b, and no more
//!   (`protocol.c:630-637`). eltuxusb's start (`el_ui.py`, `start_recording`)
//!   writes 0x21 = `00000011`, a sample count of 0, a new start time and every
//!   other field. So after this the count, the time and the downloaded bit
//!   (bit 1 is "data NOT downloaded", `el_ui.py:169-185`) stay as they were.
//!   Download first: the effect on stored data is unverified on hardware.
//! - The top three bits of byte 0x21 cleared on every write
//!   (`protocol.c:591`).

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Value, ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, split_conn};
use protocol::{Decoder, EP_IN, INTERFACE, LOG_READ, LOG_TIMEOUT, Link, LogFormat, Profile};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

const DRIVER: &str = "lascar-el-usb";
/// Reads in a row with nothing to show before a download gives up: 3 s of
/// timeouts on a silent device. The C has no end. No recording says how slow
/// a real EEPROM read gets; eltuxusb waits 1 s for a block.
pub const MAX_FAILED_READS: u32 = 30;

/// The driver.
pub struct LascarDriver;

static CLASSES: [DeviceClass; 2] = [DeviceClass::Thermometer, DeviceClass::Hygrometer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::Datalog, Access::GET_SET),
];

impl Driver for LascarDriver {
    fn id(&self) -> &'static str {
        DRIVER
    }

    fn long_name(&self) -> &'static str {
        "Lascar EL-USB"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    /// `scan()`: nothing without `conn=`, and only what a block identifies.
    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else { return Ok(Vec::new()) };
        let conn = match split_conn(&conn) {
            Ok(c) => c,
            Err(msg) => {
                log::error!("{msg}");
                return Ok(Vec::new());
            }
        };
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::list_devices()? {
            if conn.matches(&info)
                && let Some(dev) = probe(&info)
            {
                devices.push(Box::new(dev));
            }
        }
        Ok(devices)
    }
}

/// `lascar_scan()`: the mode setup and a configuration read of one device.
/// libusb lets the C read with no claim, which `usbfs` takes for it; the
/// claim here is the same, on the wire and to another program holding it.
fn probe(info: &UsbDeviceInfo) -> Option<LascarDevice> {
    let t = crate::usb::open(info)
        .inspect_err(|e| {
            log::debug!("failed to open device for scan: {}", crate::usb::libusb_error_name(e));
        })
        .ok()?;
    let mut link = Link { transport: t };
    link.mode_setup();
    if let Err(e) = link.transport.claim_interface(INTERFACE) {
        log::debug!("Failed to claim interface: {}.", crate::usb::libusb_error_name(&e));
        return None;
    }
    let config = link.read_config().ok();
    let _ = link.transport.release_interface(INTERFACE);
    drop(link);
    let (profile, firmware) = protocol::identify(&config?)?;
    Some(LascarDevice::new(info.clone(), profile, firmware))
}

/// One logger.
pub struct LascarDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    profile: Profile,
    channels: Vec<Channel>,
    limit_samples: u64,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl LascarDevice {
    fn new(usb: UsbDeviceInfo, profile: Profile, firmware: String) -> Self {
        let names: &[&str] =
            if profile.format == LogFormat::Co { &["CO"] } else { &["Temp", "Hum"] };
        LascarDevice {
            info: DeviceInfo {
                driver: DRIVER,
                vendor: "Lascar".into(),
                model: profile.name.into(),
                version: firmware,
                serial: String::new(),
                conn: Some(usb.conn()),
            },
            usb,
            profile,
            channels: names
                .iter()
                .enumerate()
                .map(|(i, n)| Channel::new(i, ChannelKind::Analog, *n))
                .collect(),
            limit_samples: 0,
            pending: None,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording), of the
    /// model a configuration block says.
    ///
    /// # Errors
    ///
    /// When the block names no supported model.
    pub fn from_transport(transport: Box<dyn Transport>, config: &[u8]) -> Result<Self> {
        let usb = transport.info().clone();
        let (profile, firmware) = protocol::identify(config)
            .ok_or_else(|| Error::device("not a supported EL-USB model"))?;
        let mut dev = Self::new(usb, profile, firmware);
        dev.pending = Some(transport);
        Ok(dev)
    }

    fn enabled(&self, pos: usize) -> bool {
        self.channels.get(pos).is_some_and(|c| c.enabled)
    }

    fn with_link<T>(&self, f: impl FnOnce(&mut Link) -> Result<T>) -> Result<T> {
        let mut guard = self.link.lock().expect("link lock");
        let link = guard.as_mut().ok_or_else(|| Error::device("device is not open"))?;
        f(link)
    }
}

impl Device for LascarDevice {
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
        match key {
            Key::Conn => Ok(Value::Str(self.usb.conn())),
            Key::LimitSamples => Ok(Value::U64(self.limit_samples)),
            Key::Datalog => self.with_link(Link::is_logging).map(Value::Bool),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => {
                self.limit_samples = value.as_u64().ok_or_else(bad)?;
                Ok(())
            }
            Key::Datalog => {
                let on = value.as_bool().ok_or_else(bad)?;
                self.with_link(|l| l.set_logging(on))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    /// `dev_open()`.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let mut t = if let Some(t) = self.pending.take() {
            t
        } else {
            // sr_usb_open() looks for the bus and address again.
            let found = crate::usb::list_devices()?
                .into_iter()
                .find(|d| d.bus == self.usb.bus && d.address == self.usb.address)
                .ok_or_else(|| Error::device("Failed to open device: gone"))?;
            crate::usb::open(&found)
                .map_err(|e| Error::device(format!("Failed to open device: {e}")))?
        };
        if let Err(e) = t.claim_interface(INTERFACE) {
            // A device gone since the scan: sr_usb_open() fails on it in C.
            if matches!(e, Error::Disconnected) {
                return Err(Error::device("Failed to open device: gone"));
            }
            log::error!("Failed to claim interface: {}.", crate::usb::libusb_error_name(&e));
            return Err(e);
        }
        *self.link.lock().expect("link lock") = Some(Link { transport: t });
        Ok(())
    }

    /// `dev_close()`.
    fn close(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        if let Some(mut link) = self.link.lock().expect("link lock").take() {
            let _ = link.transport.release_interface(INTERFACE);
        }
        Ok(())
    }

    /// `dev_acquisition_start()`: the log request happens here, so that a
    /// refusal is a failed start as in C; the blocks come on a thread.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(w) = self.worker.take() {
            if !w.handle.is_finished() {
                self.worker = Some(w);
                return Err(Error::Busy);
            }
            let _ = w.handle.join();
        }
        let Some(mut link) = self.link.lock().expect("link lock").take() else {
            return Err(Error::device("device is not open"));
        };
        let begun = begin(&mut link, self.profile.format, self.limit_samples);
        let mut plan = match begun {
            Ok(p) => p,
            Err(e) => {
                *self.link.lock().expect("link lock") = Some(link);
                return Err(e);
            }
        };
        plan.decoder.first = self.enabled(0);
        plan.decoder.second = self.enabled(1);
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("lascar-el-usb acquisition".into())
            .spawn(move || {
                sink.send(Packet::header());
                sink.send(Packet::Meta(vec![(Key::SampleInterval, Value::U64(plan.interval_ms))]));
                if plan.decoder.logged > 0 {
                    let mut decoder = plan.decoder;
                    let read = plan.first_read;
                    run(&mut link, &mut decoder, plan.log_size, read, &sink, &stop2);
                }
                sink.send(Packet::End);
                *slot.lock().expect("link lock") = Some(link);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the download to end; `close` joins the thread.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// What a download needs once it is under way.
struct Plan {
    decoder: Decoder,
    log_size: u32,
    /// The first read of the log, done before the start returns.
    first_read: Option<Result<Vec<u8>>>,
    interval_ms: u64,
}

/// The part of `dev_acquisition_start()` that may fail: the block, then the
/// log's request. A log of no samples asks for nothing.
fn begin(link: &mut Link, format: LogFormat, limit: u64) -> Result<Plan> {
    let config = link.read_config()?;
    let (decoding, logged) = protocol::proc_config(format, &config)?;
    log::debug!("Starting log retrieval.");
    let log_size = if logged == 0 { 0 } else { link.request_log()? };
    let mut first_read = None;
    if logged > 0 {
        // The submit of api.c:364 fails a start when the device has gone.
        let read = link.transport.bulk_in(EP_IN, LOG_READ, LOG_TIMEOUT);
        if matches!(read, Err(Error::Disconnected)) {
            log::error!("Unable to submit transfer: LIBUSB_ERROR_NO_DEVICE.");
            return Err(Error::Disconnected);
        }
        first_read = Some(read);
    }
    let decoder =
        Decoder { decoding, logged, received: 0, sent: 0, limit, first: true, second: true };
    Ok(Plan { decoder, log_size, first_read, interval_ms: protocol::interval_ms(&config) })
}

/// `lascar_el_usb_receive_transfer()` as a loop: reads until the header's
/// bytes have come, the device is gone, a stop, or [`MAX_FAILED_READS`]. A
/// timed out read holds what came (`Transport::bulk_in`), and a failed one
/// is skipped, as in C, with the count of bytes as it was.
fn run(
    link: &mut Link,
    decoder: &mut Decoder,
    log_size: u32,
    mut first_read: Option<Result<Vec<u8>>>,
    sink: &PacketSink,
    stop: &AtomicBool,
) {
    let mut received = 0u32;
    let mut failed = 0u32;
    while !stop.load(Ordering::SeqCst) {
        let read = first_read
            .take()
            .unwrap_or_else(|| link.transport.bulk_in(EP_IN, LOG_READ, LOG_TIMEOUT));
        match read {
            Ok(data) => {
                failed = if data.is_empty() { failed + 1 } else { 0 };
                for packet in decoder.packets(&data) {
                    if !sink.send(Packet::Analog(packet)) {
                        return;
                    }
                }
                received = received.saturating_add(u32::try_from(data.len()).unwrap_or(u32::MAX));
                log::trace!("received {received}/{log_size} bytes");
                if received >= log_size {
                    return;
                }
            }
            Err(Error::Disconnected) => return,
            Err(Error::Timeout) => {
                if received >= log_size {
                    return;
                }
                failed += 1;
            }
            Err(e) => {
                log::debug!("Log read failed: {e}.");
                failed += 1;
            }
        }
        if failed >= MAX_FAILED_READS {
            log::error!("The device stopped sending its log after {received} of {log_size} bytes.");
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;
    use std::fmt::Write as _;

    const NO_STEPS: &str = "# oxrok usb fixture 1\ndevice vid=10c4 pid=0002\n";

    fn analog_of(p: &Packet) -> &crate::core::AnalogData {
        let Packet::Analog(a) = p else { panic!("{p:?}") };
        a
    }

    fn decoder(logged: u32, limit: u64) -> Decoder {
        Decoder {
            decoding: protocol::Decoding::TempRh { fahrenheit: false },
            logged,
            received: 0,
            sent: 0,
            limit,
            first: true,
            second: true,
        }
    }

    fn run_on(
        steps: &str,
        mut dec: Decoder,
        log_size: u32,
        stop: &AtomicBool,
    ) -> (Vec<Packet>, Replay) {
        let r = Replay::from_text(&format!("{NO_STEPS}{steps}")).unwrap();
        let mut link = Link { transport: Box::new(r.clone()) };
        let (tx, rx) = std::sync::mpsc::sync_channel(4096);
        run(&mut link, &mut dec, log_size, None, &PacketSink::new(tx, 0), stop);
        (rx.try_iter().map(|(_, p)| p).collect(), r)
    }

    #[test]
    fn a_download_reads_to_the_headers_bytes() {
        // Samples in the first block, erased bytes after: read, not decoded.
        let (p, r) = run_on(
            "bulk_in 82 4096 -> 78507a52\nbulk_in 82 4096 -> ffffffff\n",
            decoder(2, 0),
            8,
            &AtomicBool::new(false),
        );
        assert_eq!(p.len(), 2);
        assert_eq!(analog_of(&p[0]).samples, [20.0, 21.0]);
        r.finish().unwrap();
    }

    #[test]
    fn a_limit_keeps_the_reads() {
        let (p, r) = run_on(
            "bulk_in 82 4096 -> 78507a52\nbulk_in 82 4096 -> 78507a52\n",
            decoder(4, 1),
            8,
            &AtomicBool::new(false),
        );
        assert_eq!(p.len(), 2, "one sample per channel");
        assert_eq!(analog_of(&p[0]).samples, [20.0]);
        r.finish().unwrap();
    }

    #[test]
    fn a_timeout_counts_what_came_and_goes_on() {
        let (p, r) = run_on(
            "bulk_in 82 4096 -> err timeout\nbulk_in 82 4096 -> 7850\n",
            decoder(1, 0),
            2,
            &AtomicBool::new(false),
        );
        assert_eq!(p.len(), 2);
        r.finish().unwrap();
    }

    #[test]
    fn no_memory_ends_at_the_first_timeout() {
        let (p, r) = run_on(
            "bulk_in 82 4096 -> err timeout\nbulk_in 82 4096 -> 7850\n",
            decoder(1, 0),
            0,
            &AtomicBool::new(false),
        );
        assert_eq!(p, []);
        assert_eq!(r.remaining(), 1, "the last step is left unread");
    }

    #[test]
    fn a_stall_is_skipped_and_the_next_read_goes_on() {
        let (p, r) = run_on(
            "bulk_in 82 4096 -> err device \"endpoint stalled\"\nbulk_in 82 4096 -> 7850\n",
            decoder(1, 0),
            2,
            &AtomicBool::new(false),
        );
        assert_eq!(p.len(), 2);
        r.finish().unwrap();
    }

    #[test]
    fn a_silent_or_halted_device_ends_at_the_bound() {
        for step in ["bulk_in 82 4096 -> err timeout\n", "bulk_in 82 4096 -> -\n"] {
            let mut text = String::new();
            for _ in 0..MAX_FAILED_READS {
                text.push_str(step);
            }
            // One step more, which the bound leaves unread.
            text.push_str(step);
            let (p, r) = run_on(&text, decoder(1, 0), 512, &AtomicBool::new(false));
            assert_eq!(p, []);
            assert_eq!(r.remaining(), 1, "the last step is left unread");
        }
        let mut text = String::new();
        for _ in 0..=MAX_FAILED_READS {
            let _ = writeln!(text, "bulk_in 82 4096 -> err device \"endpoint stalled\"");
        }
        let (_, r) = run_on(&text, decoder(1, 0), 512, &AtomicBool::new(false));
        assert_eq!(r.remaining(), 1, "the last step is left unread");
    }

    #[test]
    fn a_read_with_data_resets_the_count() {
        let mut text = String::new();
        for _ in 0..2 {
            for _ in 0..MAX_FAILED_READS - 1 {
                text.push_str("bulk_in 82 4096 -> err timeout\n");
            }
            text.push_str("bulk_in 82 4096 -> 7850\n");
        }
        let (p, r) = run_on(&text, decoder(2, 0), 4, &AtomicBool::new(false));
        assert_eq!(p.len(), 4);
        r.finish().unwrap();
    }

    #[test]
    fn a_disconnect_ends_it_at_once() {
        let (p, r) = run_on(
            "bulk_in 82 4096 -> 7850\nbulk_in 82 4096 -> err disconnected\n\
             bulk_in 82 4096 -> 7850\n",
            decoder(9, 0),
            512,
            &AtomicBool::new(false),
        );
        assert_eq!(p.len(), 2);
        assert_eq!(r.remaining(), 1, "the last step is left unread");
    }

    #[test]
    fn a_stop_ends_it_with_data_on_offer() {
        // The fixture offers data; a mutant that ignores the stop reads it.
        let (p, r) = run_on("bulk_in 82 4096 -> 7850\n", decoder(1, 0), 2, &AtomicBool::new(true));
        assert_eq!(p, [], "stopped before any read");
        assert_eq!(r.remaining(), 1, "the offered read is left");
    }

    #[test]
    fn scan_needs_a_conn() {
        assert!(LascarDriver.scan(&[]).unwrap().is_empty());
        let bad = [(Key::Conn, Value::Str("nonsense".into()))];
        assert!(LascarDriver.scan(&bad).unwrap().is_empty());
        let unknown = [(Key::Samplerate, Value::U64(1))];
        assert!(LascarDriver.scan(&unknown).is_err());
    }

    #[test]
    fn options_are_the_c_lists() {
        assert_eq!(LascarDriver.scan_options(), &[Key::Conn]);
        assert_eq!(LascarDriver.classes().len(), 2);
    }

    fn device() -> LascarDevice {
        let r = Replay::from_text(NO_STEPS).unwrap();
        let mut c = vec![0u8; 128];
        c[0] = 3;
        LascarDevice::from_transport(Box::new(r), &c).unwrap()
    }

    #[test]
    fn from_transport_takes_the_model_from_the_block() {
        let d = device();
        assert_eq!(d.info().model, "EL-USB-2");
        assert_eq!(
            d.channels().iter().map(|c| c.name.as_str()).collect::<Vec<_>>(),
            ["Temp", "Hum"]
        );
        let r = Replay::from_text(NO_STEPS).unwrap();
        let mut c = vec![0u8; 128];
        c[0] = 9;
        let d = LascarDevice::from_transport(Box::new(r.clone()), &c).unwrap();
        assert_eq!(d.channels().len(), 1);
        c[0] = 1;
        assert!(LascarDevice::from_transport(Box::new(r), &c).is_err());
    }

    #[test]
    fn limit_samples_reads_back() {
        let mut d = device();
        assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(0));
        d.set(&Key::LimitSamples, &Value::U64(7), None).unwrap();
        assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(7));
        assert!(d.get(&Key::Samplerate, None).is_err());
        assert!(d.get(&Key::Datalog, None).is_err(), "not open");
        assert!(d.set(&Key::Datalog, &Value::U64(1), None).is_err(), "a bool");
    }
}
