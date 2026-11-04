// Counterpart in sigrok:
//   libsigrok/src/hardware/greatfet/api.c,
//   libsigrok/src/hardware/greatfet/protocol.c (the transfers, the sample data),
//   Copyright (C) 2019 Katherine J. Temkin <k@ktemkin.com>,
//   Copyright (C) 2019 Mikaela Szekely <qyriad@gmail.com>,
//   Copyright (C) 2023 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! greatfet: the Great Scott Gadgets `GreatFET` One (`1d50:60e6`), an
//! LPC4330 whose SGPIO block streams up to 16 logic channels. Scan and
//! capture follow `api.c` and `protocol.c`, so that the device sees the same
//! transactions as under sigrok-cli (checked with the emulator,
//! `tools/emu/greatfet.sh`; no real board was at hand).
//!
//! The firmware is `greatscottgadgets/greatfet`; it speaks libgreat
//! ([`protocol`]) on its control endpoint and streams on bulk IN 0x81 until
//! it is told to stop. The firmware answers the rate it picked, at least the
//! one asked for; a rate that would take more USB bandwidth than
//! [`protocol::BANDWIDTH_THRESHOLD`] is refused before the start, as in the C,
//! whose comment says the board becomes unreachable if it starts such a
//! capture (`protocol.c:575`; only that comment says so).
//!
//! Where this differs from the C on purpose:
//!
//! - The stop goes out after a failed transfer. The C's goes out from
//!   inside the transfer callback there, where `libusb_control_transfer()`
//!   answers `LIBUSB_ERROR_BUSY`, and the board keeps sampling. On a limit or
//!   a stop by hand the C's stop runs outside the callback and does go out.
//! - A rate of 2^32 or more is refused before anything is sent: the
//!   firmware's field has 32 bits (`logic_analyzer.c`, `<IB`) and the C
//!   sends the rest of it (`protocol.c:544`), so 2^32 runs at 204 MHz.
//! - A warning for an answered rate the board probably doesn't run at: below
//!   [`MIN_DIVIDED_RATE`], or over [`CLOCK`], where no samples come.
//! - The stop goes out when the transfers can't be submitted after a start:
//!   `api.c:524` counts that as a start that never happened.
//! - Samples received before a failure reach the session. The C frees its
//!   feed queue without a flush at the first transfer freed
//!   (`protocol.c:961` tests the count the wrong way round, and
//!   `protocol.c:978`).
//! - Packed samples are unpacked at any feed width (`protocol.c:1311` takes
//!   a feed of one byte for a stream of whole samples) and a limit is kept
//!   to the sample.
//! - Samples go to the session as they arrive; the C's queue holds up to
//!   4 MiB of them until it fills or the capture stops.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error,
    Key, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::usb::{BulkStream, Transport, UsbDeviceInfo, UsbId, libusb_error_name, split_conn};
use protocol::{BANDWIDTH_THRESHOLD, Link, Plan};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// The board's id.
pub const ID: UsbId = UsbId { vendor: 0x1d50, product: 0x60e6 };
const SAMPLES_INTERFACE: u8 = 1;
/// `TRANSFER_BUFFER_SIZE`, `TRANSFER_POOL_SIZE`.
const TRANSFER_SIZE: usize = 256 * 1024;
const TRANSFER_COUNT: usize = 16;
/// `usb_source_add()`'s period, which is when the C looks at its limits.
const POLL: Duration = Duration::from_millis(50);
/// `DEFAULT_SAMPLERATE`.
const DEFAULT_SAMPLERATE: u64 = 34_000_000;
/// The SGPIO clock (`platform_clock.c`'s default, and `api.c:67`).
const CLOCK: u32 = 204_000_000;
/// The lowest rate of the SGPIO's 12 bit divider. The firmware writes a larger
/// divider without a mask and answers the rate it computed, so a lower
/// rate is probably not what the board runs at. Inferred: NXP's register
/// description has `PRESET` in bits 11:0 and AN11275 a 12 bit counter; no
/// board has shown it.
const MIN_DIVIDED_RATE: u32 = CLOCK / 4096;
/// What a refused capture says; the driver has logged why.
const START_REFUSED: &str = "the capture did not start";

/// The sample rates of `api.c`; the firmware picks one at least as high
/// as whatever it is asked.
pub const SAMPLERATES: [u64; 15] = [
    1_000_000,
    2_000_000,
    4_000_000,
    8_500_000,
    10_200_000,
    12_000_000,
    17_000_000,
    20_400_000,
    25_500_000,
    34_000_000,
    40_800_000,
    51_000_000,
    68_000_000,
    102_000_000,
    204_000_000,
];

const CHANNEL_NAMES: [&str; 16] = [
    "SGPIO0", "SGPIO1", "SGPIO2", "SGPIO3", "SGPIO4", "SGPIO5", "SGPIO6", "SGPIO7", "SGPIO8",
    "SGPIO9", "SGPIO10", "SGPIO11", "SGPIO12", "SGPIO13", "SGPIO14", "SGPIO15",
];

/// The driver.
pub struct GreatfetDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::ProbeNames];
static DEV_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::Continuous, Access::GET),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];

impl Driver for GreatfetDriver {
    fn id(&self) -> &'static str {
        "greatfet"
    }

    fn long_name(&self) -> &'static str {
        "Great Scott Gadgets GreatFET One"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    /// `scan()`: `conn=` takes `sr_usb_find()`'s forms, of any device as
    /// in the C, or `sn=TEXT`, part of the USB serial number string. Each
    /// board must answer its serial number and version.
    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut names = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::ProbeNames => names = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let (conn, want_serial) = match conn.as_deref() {
            Some(c) => match c.strip_prefix("sn=") {
                Some(s) => {
                    log::info!("Searching default 1d50.60e6 and serial number {s}.");
                    (None, Some(s.to_owned()))
                }
                None => (Some(c), None),
            },
            None => (None, None),
        };
        let found: Vec<UsbDeviceInfo> = match conn {
            None => crate::usb::find_devices(&[ID])?,
            Some(c) => match split_conn(c) {
                Ok(c) => crate::usb::list_devices()?.into_iter().filter(|d| c.matches(d)).collect(),
                Err(msg) => {
                    log::error!("{msg}");
                    return Ok(Vec::new());
                }
            },
        };
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in found {
            // Failure here is silent, there are legal reasons why a device
            // can't be reached.
            let Ok(mut t) = crate::usb::open(&info) else { continue };
            if let Some(want) = &want_serial
                && !usb_serial_has(t.as_mut(), want)
            {
                continue;
            }
            let mut link = Link::new(t);
            let Ok(serial) = link.read_serial().inspect_err(|_| {
                log::error!("Cannot get serial number.");
            }) else {
                continue;
            };
            let Ok(version) = link.read_version().inspect_err(|_| {
                log::error!("Cannot get firmware version.");
            }) else {
                continue;
            };
            devices.push(Box::new(GreatfetDevice::new(info, version, serial, names.as_deref())));
        }
        Ok(devices)
    }
}

/// The C's check of the USB serial number, a string of at most 63
/// characters read into its 64 byte buffer: a part of it matches.
fn usb_serial_has(t: &mut dyn Transport, want: &str) -> bool {
    let Ok([_, _, index]) = t.string_indexes() else { return false };
    if index == 0 {
        return false;
    }
    let Ok(text) = crate::usb::read_string_ascii(t, index) else { return false };
    let text: String = text.chars().take(63).collect();
    let found = text.contains(want);
    log::debug!("got serno {text}, checking {want}, match {}", i32::from(found));
    found
}

/// One `GreatFET`.
pub struct GreatfetDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    /// Bytes per sample in the session: all channels, whatever the wire.
    unit: usize,
    samplerate: u64,
    limit_samples: u64,
    limit_msec: u64,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl GreatfetDevice {
    fn new(usb: UsbDeviceInfo, version: String, serial: String, probe_names: Option<&str>) -> Self {
        let names = match probe_names {
            Some(spec) => crate::core::util::parse_probe_names(spec, &CHANNEL_NAMES),
            None => CHANNEL_NAMES.iter().map(|s| (*s).to_owned()).collect(),
        };
        let channels: Vec<Channel> = names
            .iter()
            .take(16)
            .enumerate()
            .map(|(i, name)| {
                let mut c = Channel::new(i, ChannelKind::Logic, name.clone());
                c.enabled = i < 8;
                c
            })
            .collect();
        let group = ChannelGroup::new("Logic", (0..channels.len()).collect());
        GreatfetDevice {
            info: DeviceInfo {
                driver: "greatfet",
                vendor: "Great Scott Gadgets".into(),
                model: "GreatFET".into(),
                version,
                serial,
                conn: Some(usb.conn()),
            },
            usb,
            unit: channels.len().div_ceil(8),
            channels,
            groups: vec![group],
            samplerate: DEFAULT_SAMPLERATE,
            limit_samples: 0,
            limit_msec: 0,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording).
    ///
    /// # Errors
    ///
    /// When the transport is no `GreatFET` or does not answer.
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        if usb.id() != ID {
            return Err(Error::arg("not a GreatFET"));
        }
        let mut link = Link::new(transport);
        let serial = link.read_serial()?;
        let version = link.read_version()?;
        let dev = Self::new(usb, version, serial, None);
        *dev.link.lock().expect("link lock") = Some(link);
        Ok(dev)
    }

    fn channel_mask(&self) -> u16 {
        self.channels.iter().filter(|c| c.enabled).fold(0, |m, c| m | 1 << c.index)
    }

    /// A capture is running: its thread has not given the link back.
    fn busy(&self) -> bool {
        self.worker.as_ref().is_some_and(|w| !w.handle.is_finished())
            && self.link.lock().expect("link lock").is_none()
    }

    /// The rate as the firmware's field holds it; more is refused, where
    /// the C truncates it (`protocol.c:544`).
    fn wire_rate(&self) -> Result<u32> {
        u32::try_from(self.samplerate).map_err(|_| {
            Error::Arg(format!(
                "sample rate {} does not fit the firmware's 32 bit field",
                self.samplerate
            ))
        })
    }

    /// `greatfet_start_acquisition()`. A failed claim does not stop the start,
    /// as in the C.
    fn begin(&mut self, link: &mut Link, plan: &Plan) -> Result<Box<dyn BulkStream>> {
        let configured = link.configure(plan, self.wire_rate()?)?;
        let rate = configured.rate;
        if u64::from(rate) != self.samplerate {
            log::info!("Configuration feedback, want rate {}, got rate {rate}", self.samplerate);
            self.samplerate = u64::from(rate);
        }
        log::info!("Capture configuration: {} channels.", plan.capture_channels);
        let bandwidth = plan.bandwidth(rate);
        log::info!("Resulting USB bandwidth: {bandwidth} bps.");
        if bandwidth > BANDWIDTH_THRESHOLD {
            log::error!("Configuration exceeds bandwidth limit. Aborting.");
            return Err(Error::device(START_REFUSED));
        }
        if let Some(warning) = rate_warning(rate) {
            log::warn!("Sample rate {rate}: {warning}");
        }
        let claim = link.transport().claim_interface(SAMPLES_INTERFACE);
        link.start()?;
        // A claim that failed fails the C's submissions after the start. The
        // backends differ in when a stream on an unclaimed interface says
        // so (the libusb one at its first transfer), so it stops here.
        if let Err(e) = claim {
            log::debug!("Cannot claim the samples interface: {e}.");
            stop_unless_gone(link, &e);
            return Err(e);
        }
        // Submitted after the start: the C's comment says the other order
        // gives libusb IO errors. The endpoint is the firmware's to name.
        link.transport()
            .bulk_in_stream(configured.endpoint, TRANSFER_SIZE, TRANSFER_COUNT)
            .inspect_err(|e| {
                log::debug!("submit bulk xfer failed: {}", libusb_error_name(e));
                // The start went through: the board is sampling.
                stop_unless_gone(link, e);
            })
    }
}

/// The stop after a start that failed further on; a board that left hears
/// nothing.
fn stop_unless_gone(link: &mut Link, e: &Error) {
    if !matches!(e, Error::Disconnected) {
        link.stop();
    }
}

/// What is doubtful about an answered rate. The firmware answers either
/// without complaint and the capture goes on, so this only warns.
fn rate_warning(rate: u32) -> Option<&'static str> {
    if rate > CLOCK {
        Some("over the board's 204 MHz clock, the firmware will send no samples.")
    } else if rate < MIN_DIVIDED_RATE {
        Some("below what the 12 bit divider reaches, the board likely samples faster.")
    } else {
        None
    }
}

impl Device for GreatfetDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &self.groups
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
            // config_list() answers SR_ERR_NA for its empty devopts_cg.
            Some(g) if self.groups.iter().any(|cg| cg.name.eq_ignore_ascii_case(g)) => {
                Err(Error::not_supported("the channel group has no options"))
            }
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if group.is_some() {
            return Err(Error::not_supported("not applicable"));
        }
        Ok(match key {
            Key::Conn => Value::Str(self.usb.conn()),
            Key::Continuous => Value::Bool(true),
            Key::Samplerate => Value::U64(self.samplerate),
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::LimitMsec => Value::U64(self.limit_msec),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// Any rate is taken, as in `config_set()`: the firmware picks. All of
    /// these go out or count when a capture starts, so one set during a
    /// capture is refused rather than kept unused.
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let number =
            || value.as_u64().ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())));
        if !matches!(key, Key::Samplerate | Key::LimitSamples | Key::LimitMsec) {
            return Err(Error::UnknownKey(key.id().to_owned()));
        }
        if self.busy() {
            return Err(Error::Busy);
        }
        match key {
            Key::Samplerate => {
                let rate = number()?;
                // sr_config_set()'s check_key(), hwdriver.c:752.
                if rate == 0 {
                    return Err(Error::arg("invalid argument"));
                }
                self.samplerate = rate;
            }
            Key::LimitSamples => self.limit_samples = number()?,
            _ => self.limit_msec = number()?,
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::Samplerate => Ok(ValueList::U64(SAMPLERATES.to_vec())),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `dev_open()`: nothing but the open. The samples interface is
    /// claimed for each capture and the command interface never.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let t = crate::usb::open(&self.usb)?;
        *self.link.lock().expect("link lock") = Some(Link::new(t));
        Ok(())
    }

    /// Ends a capture still running and joins its thread.
    fn close(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        drop(self.link.lock().expect("link lock").take());
        Ok(())
    }

    /// A refusal that needs no device.
    fn check_capture(&self, trigger: Option<&crate::core::Trigger>) -> Result<()> {
        let _ = trigger;
        self.wire_rate().map(|_| ())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.busy() {
            return Err(Error::Busy);
        }
        // A worker with the link back has only its End left to send; one
        // that has not finished then ends by itself.
        if let Some(w) = self.worker.take()
            && w.handle.is_finished()
        {
            let _ = w.handle.join();
        }
        self.wire_rate()?;
        let plan = protocol::plan(self.channel_mask())
            .ok_or_else(|| Error::arg("no channel is enabled"))?;
        log::debug!(
            "unit {}, dense {} -> shift {}, points {}",
            plan.wire_unit,
            i32::from(plan.channel_shift != 0),
            plan.channel_shift,
            plan.points_per_byte
        );
        if plan.multi_bank {
            log::warn!("Multi-bank capture, check firmware support!");
        }
        let link = self.link.lock().expect("link lock").take();
        let Some(mut link) = link else {
            return Err(Error::device("device is not open"));
        };
        // sr_sw_limits_acquisition_start(): before the requests.
        let started = Instant::now();
        let stream = match self.begin(&mut link, &plan) {
            Ok(stream) => stream,
            Err(e) => {
                *self.link.lock().expect("link lock") = Some(link);
                return Err(match e {
                    Error::Disconnected => Error::Disconnected,
                    _ => Error::device(START_REFUSED),
                });
            }
        };
        let capture = Capture {
            plan,
            unit: self.unit,
            samplerate: self.samplerate,
            limit_samples: self.limit_samples,
            limit_time: (self.limit_msec > 0).then(|| Duration::from_millis(self.limit_msec)),
            started,
        };
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("greatfet acquisition".into())
            .spawn(move || {
                run(stream, &mut link, &capture, &sink, &stop2);
                // Before the End, so that a client can start again at once.
                *slot.lock().expect("link lock") = Some(link);
                sink.send(Packet::End);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the capture to end; `close` joins its thread, which may still
    /// be sending packets.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// What the acquisition thread needs.
struct Capture {
    plan: Plan,
    unit: usize,
    samplerate: u64,
    limit_samples: u64,
    limit_time: Option<Duration>,
    started: Instant,
}

/// The acquisition thread: the header, the transfers until a limit, a stop
/// or a failure, the stop request unless the board left. The caller gives
/// the link back and sends the end.
fn run(
    mut stream: Box<dyn BulkStream>,
    link: &mut Link,
    capture: &Capture,
    sink: &PacketSink,
    stop: &AtomicBool,
) {
    sink.send(Packet::header());
    sink.send(Packet::meta(Key::Samplerate, Value::U64(capture.samplerate)));
    let mut sent = 0;
    let board_left = loop {
        if stop.load(Ordering::SeqCst)
            || capture.limit_time.is_some_and(|t| capture.started.elapsed() > t)
        {
            break false;
        }
        match stream.next(POLL) {
            Ok(data) => {
                let remain = (capture.limit_samples > 0).then(|| capture.limit_samples - sent);
                let samples = capture.plan.samples(&data, capture.unit, remain);
                sent += (samples.len() / capture.unit) as u64;
                if !samples.is_empty() && !sink.send(Packet::logic(capture.unit, samples)) {
                    break false;
                }
                if capture.limit_samples > 0 && sent >= capture.limit_samples {
                    break false;
                }
            }
            Err(Error::Timeout) => {}
            Err(Error::Disconnected) => {
                log::error!("Device gone during USB transfer. Aborting.");
                break true;
            }
            Err(e) => {
                let name = libusb_error_name(&e);
                if name == "LIBUSB_ERROR_PIPE" {
                    log::error!("Device firmware is stalled on USB transfer. Aborting.");
                } else {
                    log::error!("USB transfer failed ({name}). Aborting.");
                }
                break false;
            }
        }
    };
    if !board_left {
        link.stop();
    }
    drop(stream);
    let _ = link.transport().release_interface(SAMPLES_INTERFACE);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::ControlRequest;
    use crate::usb::record::Replay;
    use std::sync::atomic::AtomicUsize;

    fn device(names: Option<&str>) -> GreatfetDevice {
        GreatfetDevice::new(UsbDeviceInfo::default(), "v1".into(), "s".into(), names)
    }

    fn stopping_link() -> (Replay, Link) {
        let r = Replay::from_text(
            "# oxrok usb fixture 1\ndevice vid=1d50 pid=60e6 version=0100 interfaces=0:ff:ff:ff,1:ff:ff:ff\n\
             ctl_out 42 65 0000 0001 0d01000004000000 -> ok\n",
        )
        .unwrap();
        (r.clone(), Link::new(Box::new(r)))
    }

    fn capture(mask: u16, unit: usize, limit: u64) -> Capture {
        Capture {
            plan: protocol::plan(mask).unwrap(),
            unit,
            samplerate: 1_000_000,
            limit_samples: limit,
            limit_time: None,
            started: Instant::now(),
        }
    }

    /// A stream that always offers data, and sets the flag at its third
    /// answer, so that an ignored stop or limit shows as a run that does
    /// not end (the bound below fails it) or as too many samples.
    struct Endless {
        stop: Arc<AtomicBool>,
        answers: u32,
    }

    impl BulkStream for Endless {
        fn next(&mut self, _timeout: Duration) -> Result<Vec<u8>> {
            self.answers += 1;
            // Under the test channel's 64 packets, or the send blocks for ever.
            assert!(self.answers < 32, "the capture went on");
            if self.answers == 3 {
                self.stop.store(true, Ordering::SeqCst);
            }
            Ok(vec![0xff; 4])
        }
    }

    /// The samples sent.
    fn packets(rx: &std::sync::mpsc::Receiver<(usize, Packet)>) -> usize {
        rx.try_iter()
            .map(|(_, p)| if let Packet::Logic(l) = p { l.data.len() / l.unit_size } else { 0 })
            .sum()
    }

    #[test]
    fn a_stop_ends_a_stream_that_goes_on() {
        let (r, mut link) = stopping_link();
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let stop = Arc::new(AtomicBool::new(false));
        let stream = Box::new(Endless { stop: Arc::clone(&stop), answers: 0 });
        run(stream, &mut link, &capture(0xff, 2, 0), &PacketSink::new(tx, 0), &stop);
        r.finish().unwrap();
        assert_eq!(packets(&rx), 12);
    }

    #[test]
    fn a_limit_ends_a_stream_that_goes_on_to_the_sample() {
        let (r, mut link) = stopping_link();
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        // Never set: the limit alone must end it.
        let stop = AtomicBool::new(false);
        let flag = Arc::new(AtomicBool::new(false));
        let stream = Box::new(Endless { stop: flag, answers: 0 });
        // 1 channel, 8 samples a byte.
        run(stream, &mut link, &capture(0x1, 2, 37), &PacketSink::new(tx, 0), &stop);
        r.finish().unwrap();
        assert_eq!(packets(&rx), 37);
    }

    #[test]
    fn a_time_limit_ends_a_stream_that_goes_on() {
        let (r, mut link) = stopping_link();
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let stop = AtomicBool::new(false);
        let flag = Arc::new(AtomicBool::new(false));
        let stream = Box::new(Endless { stop: flag, answers: 0 });
        let mut c = capture(0xff, 2, 0);
        c.limit_time = Some(Duration::ZERO);
        std::thread::sleep(Duration::from_millis(2));
        run(stream, &mut link, &c, &PacketSink::new(tx, 0), &stop);
        r.finish().unwrap();
        assert_eq!(packets(&rx), 0);
    }

    /// A stream that fails with `error` after one answer.
    struct Failing(Option<Error>, bool);

    impl BulkStream for Failing {
        fn next(&mut self, _timeout: Duration) -> Result<Vec<u8>> {
            if std::mem::replace(&mut self.1, false) {
                return Ok(vec![0xff; 4]);
            }
            Err(self.0.take().expect("asked again after the failure"))
        }
    }

    #[test]
    fn a_failed_transfer_keeps_what_came_and_stops_the_board() {
        let (r, mut link) = stopping_link();
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let stream = Box::new(Failing(Some(Error::device("endpoint stalled")), true));
        run(
            stream,
            &mut link,
            &capture(0xff, 2, 0),
            &PacketSink::new(tx, 0),
            &AtomicBool::new(false),
        );
        r.finish().unwrap();
        assert_eq!(packets(&rx), 4);
    }

    /// A transport that answers everything and counts the control writes.
    struct Spy {
        info: UsbDeviceInfo,
        writes: Arc<AtomicUsize>,
        claim: fn() -> Result<()>,
        stream: fn() -> Result<Box<dyn BulkStream>>,
    }

    impl Spy {
        fn new(stream: fn() -> Result<Box<dyn BulkStream>>) -> (Spy, Arc<AtomicUsize>) {
            let writes = Arc::new(AtomicUsize::new(0));
            let info = UsbDeviceInfo::default();
            (Spy { info, writes: Arc::clone(&writes), claim: || Ok(()), stream }, writes)
        }
    }

    impl Transport for Spy {
        fn info(&self) -> &UsbDeviceInfo {
            &self.info
        }
        fn set_configuration(&mut self, _configuration: u8) -> Result<()> {
            Ok(())
        }
        fn claim_interface(&mut self, _interface: u8) -> Result<()> {
            (self.claim)()
        }
        fn release_interface(&mut self, _interface: u8) -> Result<()> {
            Ok(())
        }
        fn reset(&mut self) -> Result<()> {
            Ok(())
        }
        fn max_packet_size(&mut self, _endpoint: u8) -> Result<usize> {
            Ok(512)
        }
        fn interfaces(&mut self) -> Result<Vec<crate::usb::InterfaceDesc>> {
            Ok(Vec::new())
        }
        fn control_out(&mut self, _r: ControlRequest, _d: &[u8], _t: Duration) -> Result<()> {
            self.writes.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }
        /// Configure's answer: 1 MHz, the buffer, the endpoint.
        fn control_in(&mut self, _r: ControlRequest, _l: usize, _t: Duration) -> Result<Vec<u8>> {
            Ok(vec![0x40, 0x42, 0x0f, 0, 0, 0x40, 0, 0, 0x81])
        }
        fn bulk_out(&mut self, _e: u8, data: &[u8], _t: Duration) -> Result<usize> {
            Ok(data.len())
        }
        fn bulk_in(&mut self, _e: u8, _l: usize, _t: Duration) -> Result<Vec<u8>> {
            Ok(Vec::new())
        }
        fn interrupt_out(&mut self, _e: u8, data: &[u8], _t: Duration) -> Result<usize> {
            Ok(data.len())
        }
        fn interrupt_in(&mut self, _e: u8, _l: usize, _t: Duration) -> Result<Vec<u8>> {
            Ok(Vec::new())
        }
        fn bulk_in_stream(
            &mut self,
            _endpoint: u8,
            _size: usize,
            _count: usize,
        ) -> Result<Box<dyn BulkStream>> {
            (self.stream)()
        }
    }

    fn gone() -> Result<Box<dyn BulkStream>> {
        Err(Error::Disconnected)
    }

    fn refused() -> Result<Box<dyn BulkStream>> {
        Err(Error::device("refused"))
    }

    #[allow(clippy::unnecessary_wraps)]
    fn ends_at_once() -> Result<Box<dyn BulkStream>> {
        Ok(Box::new(Failing(Some(Error::device("endpoint stalled")), false)))
    }

    #[test]
    fn a_board_that_left_gets_no_stop() {
        let (spy, writes) = Spy::new(gone);
        let mut link = Link::new(Box::new(spy));
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let stream = Box::new(Failing(Some(Error::Disconnected), false));
        run(
            stream,
            &mut link,
            &capture(0xff, 2, 0),
            &PacketSink::new(tx, 0),
            &AtomicBool::new(false),
        );
        assert_eq!(packets(&rx), 0);
        assert_eq!(writes.load(Ordering::SeqCst), 0);
    }

    /// First pin, alternate map, configure, start: four writes, then the stop.
    #[test]
    fn a_stream_that_fails_after_the_start_stops_the_board_unless_it_left() {
        let plan = protocol::plan(0xff).unwrap();
        for (stream, expected) in [(refused as fn() -> Result<Box<dyn BulkStream>>, 5), (gone, 4)] {
            let (spy, writes) = Spy::new(stream);
            let mut link = Link::new(Box::new(spy));
            assert!(device(None).begin(&mut link, &plan).is_err());
            assert_eq!(writes.load(Ordering::SeqCst), expected);
        }
    }

    #[test]
    fn a_failed_claim_stops_the_board_after_its_start() {
        let plan = protocol::plan(0xff).unwrap();
        let (mut spy, writes) = Spy::new(ends_at_once);
        spy.claim = || Err(Error::Busy);
        let mut link = Link::new(Box::new(spy));
        assert!(device(None).begin(&mut link, &plan).is_err());
        assert_eq!(writes.load(Ordering::SeqCst), 5);
    }

    #[test]
    fn a_rate_over_32_bits_is_refused_before_the_wire() {
        for rate in [1u64 << 32, (1 << 32) + 1, 5_000_000_000] {
            let (spy, writes) = Spy::new(ends_at_once);
            let mut d = device(None);
            *d.link.lock().unwrap() = Some(Link::new(Box::new(spy)));
            d.set(&Key::Samplerate, &Value::U64(rate), None).unwrap();
            assert!(d.check_capture(None).is_err());
            let (tx, _rx) = std::sync::mpsc::sync_channel(64);
            assert!(d.start(PacketSink::new(tx, 0)).is_err());
            assert_eq!(writes.load(Ordering::SeqCst), 0);
        }
        let mut d = device(None);
        d.set(&Key::Samplerate, &Value::U64(u64::from(u32::MAX)), None).unwrap();
        assert!(d.check_capture(None).is_ok());
    }

    #[test]
    fn doubtful_rates_are_named() {
        assert_eq!(rate_warning(49_804), None, "a divider of 4096 fits");
        assert!(rate_warning(49_803).is_some());
        assert!(rate_warning(10_000).is_some());
        assert_eq!(rate_warning(1_000_000), None);
        assert_eq!(rate_warning(CLOCK), None);
        assert!(rate_warning(CLOCK + 1).is_some());
    }

    /// Lets the capture of the test below end when it says so.
    static GATE_OPEN: AtomicBool = AtomicBool::new(false);

    struct Gated;

    impl BulkStream for Gated {
        fn next(&mut self, _timeout: Duration) -> Result<Vec<u8>> {
            if GATE_OPEN.load(Ordering::SeqCst) {
                return Err(Error::device("endpoint stalled"));
            }
            std::thread::sleep(Duration::from_millis(1));
            Err(Error::Timeout)
        }
    }

    #[allow(clippy::unnecessary_wraps)]
    fn gated() -> Result<Box<dyn BulkStream>> {
        Ok(Box::new(Gated))
    }

    /// The End comes after the link went back: while the test holds the
    /// link's lock, the thread cannot have sent it.
    #[test]
    fn a_capture_gives_the_link_back_before_its_end() {
        let (spy, _) = Spy::new(gated);
        let mut d = device(None);
        *d.link.lock().unwrap() = Some(Link::new(Box::new(spy)));
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let guard = d.link.lock().unwrap();
        GATE_OPEN.store(true, Ordering::SeqCst);
        while let Ok((_, p)) = rx.recv_timeout(Duration::from_millis(200)) {
            assert!(!matches!(p, Packet::End), "the End came before the link went back");
        }
        drop(guard);
        while !matches!(rx.recv().unwrap().1, Packet::End) {}
        assert!(d.link.lock().unwrap().is_some());
        // So a new capture starts at once.
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        while !matches!(rx.recv().unwrap().1, Packet::End) {}
        d.close().unwrap();
    }

    #[test]
    fn options_follow_the_c_driver() {
        let mut d = device(None);
        assert_eq!(d.channels.len(), 16);
        assert_eq!(d.unit, 2);
        assert_eq!(d.channel_mask(), 0x00ff);
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(34_000_000));
        assert_eq!(d.get(&Key::Continuous, None).unwrap(), Value::Bool(true));
        // Any rate is taken; the firmware picks.
        d.set(&Key::Samplerate, &Value::U64(3), None).unwrap();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(3));
        d.set(&Key::LimitMsec, &Value::U64(9), None).unwrap();
        assert_eq!(d.get(&Key::LimitMsec, None).unwrap(), Value::U64(9));
        assert!(d.set(&Key::Conn, &Value::U64(1), None).is_err());
        assert!(d.get(&Key::Samplerate, Some("Logic")).is_err());
        assert!(matches!(d.options(Some("Logic")), Err(Error::NotSupported(_))));
        assert!(d.set(&Key::Samplerate, &Value::U64(0), None).is_err());
        assert!(d.options(Some("nope")).is_err());
        assert_eq!(d.list(&Key::Samplerate, None).unwrap(), ValueList::U64(SAMPLERATES.to_vec()));
    }

    #[test]
    fn probe_names_set_channels_and_the_unit() {
        let d = device(Some("-a,b"));
        assert_eq!(d.channels.len(), 2);
        assert_eq!(d.unit, 1);
        assert_eq!(d.channels[1].name, "b");
        let d = device(Some("-a,b,c,d,e,f,g,h,i"));
        assert_eq!(d.unit, 2);
        // More than the channels are ignored, and padding fills up.
        let d = device(Some("CLK"));
        assert_eq!(d.channels.len(), 16);
        assert_eq!((d.channels[0].name.as_str(), d.channels[1].name.as_str()), ("CLK", "SGPIO1"));
    }

    #[test]
    fn from_transport_reads_the_board() {
        let r = Replay::from_text(
            "# oxrok usb fixture 1\ndevice vid=1d50 pid=60e6 version=0100 interfaces=0:ff:ff:ff,1:ff:ff:ff\n\
             ctl_out 42 65 0000 0000 0000000003000000 -> ok\n\
             ctl_in 42 65 0000 0000 1024 -> 0000000000000000e667cc5757761830\n\
             ctl_out 42 65 0000 0000 0000000001000000 -> ok\n\
             ctl_in 42 65 0000 0000 1024 -> 7631\n",
        )
        .unwrap();
        let d = GreatfetDevice::from_transport(Box::new(r.clone())).unwrap();
        assert_eq!(d.info.serial, "000057cc67e630187657");
        assert_eq!(d.info.version, "v1");
        r.finish().unwrap();
    }
}
