// Counterpart in sigrok:
//   libsigrok/src/hardware/sysclk-lwla/api.c,
//   libsigrok/src/hardware/sysclk-lwla/protocol.c (the acquisition),
//   Copyright (C) 2014 Daniel Elstner <daniel.kitta@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Soeren Apel, Martin Ling, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! sysclk-lwla: the Sysclk LWLA1034 (34 channels, 125 MHz) and LWLA1016
//! (16 channels, 100 MHz), an FX2 with its firmware in EEPROM relaying word
//! commands to an FPGA that takes its bitstream on every open.
//!
//! The wire protocol is in [`protocol`]. Open, setup, the capture's
//! requests and their failures follow `api.c` and `protocol.c` step by
//! step, so that the device sees the same transactions as under sigrok-cli
//! (checked with the emulator, `tools/emu/sysclk-lwla.sh`). The C's capture
//! is a state machine on one asynchronous transfer at a time; here it is a
//! sequence of blocking transfers on the acquisition thread, which puts the
//! same requests on the bus. A failure ends a capture with exit 0, as in the
//! C, which has no way to tell the session.
//!
//! Where this differs from the C (`docs/DEVICES.md`): a `conn=` that matches
//! nothing selects nothing, a stop writes its whole sequence, samples
//! decoded before a failure or a stop are passed on, a rate the divider
//! cannot reach gets a warning, and a trigger match other than 0, 1, r and
//! f is refused when it is set.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Trigger, TriggerMatch, Value, ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, libusb_error_name, split_conn};
use protocol::{
    Capture, Edge, Link, MAX_LIMIT_MSEC, MAX_LIMIT_SAMPLES, Model, Readout, Regs,
    STATE_LENGTH_REQUEST, STATE_READ_FINISH, STATE_READ_PREPARE, STATE_READ_REQUEST,
    STATE_START_CAPTURE, STATE_STATUS_REQUEST, STATE_STOP_CAPTURE, STATUS_CAPTURING,
    STATUS_MEM_AVAIL, STATUS_TRIGGERED, Status, StatusRequest,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// How often a capture's status is asked (`poll_interval_ms`).
const POLL_INTERVAL: Duration = Duration::from_millis(100);
/// How often a wait for the next poll looks at the stop flag.
const STOP_CHECK: Duration = Duration::from_millis(5);
/// The open's tries (`api.c:233`).
const OPEN_TRIES: usize = 3;

const TRIGGER_SOURCE_CH: &str = "CH";
const TRIGGER_SOURCE_TRG: &str = "TRG";

/// The driver.
pub struct LwlaDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const GET_SET: Access = Access::GET_SET;
static OPTIONS_1034: [OptionDesc; 8] = [
    OptionDesc::new(Key::LimitSamples, GET_SET),
    OptionDesc::new(Key::LimitMsec, GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::ExternalClock, GET_SET),
    OptionDesc::new(Key::ClockEdge, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSlope, Access::GET_SET_LIST),
];
static OPTIONS_1016: [OptionDesc; 5] = [
    OptionDesc::new(Key::LimitSamples, GET_SET),
    OptionDesc::new(Key::LimitMsec, GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::Rle, GET_SET),
];
static TRIGGER_MATCHES: [TriggerMatch; 4] =
    [TriggerMatch::Zero, TriggerMatch::One, TriggerMatch::Rising, TriggerMatch::Falling];
/// One stage of levels and edges (`api.c:504-507`).
const TRIGGER_FEATURES: crate::core::TriggerFeatures = crate::core::TriggerFeatures {
    count: false,
    consecutive: false,
    duration: false,
    range: false,
    or: false,
    and: false,
    max_stages: 0,
};

impl Driver for LwlaDriver {
    fn id(&self) -> &'static str {
        "sysclk-lwla"
    }

    fn long_name(&self) -> &'static str {
        "Sysclk LWLA series"
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
        let conn = match conn.as_deref().map(split_conn).transpose() {
            Ok(c) => c,
            Err(msg) => {
                // sr_usb_find() refuses the spec. The C then lists every
                // LWLA (api.c:104, an empty list reads as no conn=); here
                // a selection that selects nothing lists nothing.
                log::error!("{msg}");
                return Ok(Vec::new());
            }
        };
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::list_devices()? {
            if conn.is_some_and(|c| !c.matches(&info)) {
                continue;
            }
            let Some(model) = Model::from_id(info.id()) else {
                if conn.is_some() {
                    log::warn!(
                        "USB device {} ({:04x}:{:04x}) is not a Sysclk LWLA.",
                        info.conn(),
                        info.vendor_id,
                        info.product_id
                    );
                }
                continue;
            };
            devices.push(Box::new(LwlaDevice::new(info, model, None)));
        }
        Ok(devices)
    }
}

/// Settings kept between acquisitions (`struct dev_context`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Settings {
    samplerate: u64,
    limit_msec: u64,
    limit_samples: u64,
    rle: bool,
    external_clock: bool,
    clock_edge: Edge,
    trigger_ext: bool,
    trigger_slope: Edge,
}

/// One LWLA.
pub struct LwlaDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    model: Model,
    channels: Vec<Channel>,
    settings: Settings,
    trigger: Option<Trigger>,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// `prepare_trigger_masks()`, for the channels enabled: mask, values and
/// edge mask.
fn trigger_masks(trigger: Option<&Trigger>, channels: &[Channel]) -> (u64, u64, u64) {
    let (mut mask, mut values, mut edges) = (0, 0, 0);
    let Some(stage) = trigger.and_then(|t| t.stages.first()) else {
        return (mask, values, edges);
    };
    for c in &stage.conditions {
        if !channels.iter().any(|ch| ch.index == c.channel && ch.enabled) {
            continue;
        }
        let bit = 1u64 << c.channel;
        mask |= bit;
        if matches!(c.matches, TriggerMatch::One | TriggerMatch::Rising) {
            values |= bit;
        }
        if matches!(c.matches, TriggerMatch::Rising | TriggerMatch::Falling) {
            edges |= bit;
        }
    }
    (mask, values, edges)
}

/// `init_acquisition_state()`: the most samples and the longest time, each
/// derived from the other on the internal clock when only one is set. With
/// no limit at all, the largest values there are.
fn limits(c: &Capture) -> (u64, u64) {
    let mut duration_max = if c.limit_msec > 0 { c.limit_msec } else { MAX_LIMIT_MSEC };
    let mut samples_max = if c.limit_samples > 0 { c.limit_samples } else { MAX_LIMIT_SAMPLES };
    if !c.external_clock {
        if c.limit_msec == 0 && c.limit_samples > 0 {
            duration_max = c.limit_samples * 1000 / c.samplerate + 1;
        } else if c.limit_samples == 0 && c.limit_msec > 0 {
            samples_max = c.limit_msec * c.samplerate / 1000;
        }
    }
    (samples_max, duration_max)
}

impl LwlaDevice {
    fn new(usb: UsbDeviceInfo, model: Model, transport: Option<Box<dyn Transport>>) -> Self {
        LwlaDevice {
            info: DeviceInfo {
                driver: "sysclk-lwla",
                vendor: "Sysclk".into(),
                model: model.name().into(),
                version: String::new(),
                serial: String::new(),
                // api.c never sets sdi->connection_id.
                conn: None,
            },
            usb,
            model,
            channels: (0..model.channels())
                .map(|i| Channel::new(i, ChannelKind::Logic, format!("CH{}", i + 1)))
                .collect(),
            settings: Settings {
                samplerate: model.samplerates()[0],
                limit_msec: 0,
                limit_samples: 0,
                rle: true,
                external_clock: false,
                clock_edge: Edge::Positive,
                trigger_ext: false,
                trigger_slope: Edge::Positive,
            },
            trigger: None,
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording).
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        let model =
            Model::from_id(usb.id()).ok_or_else(|| Error::arg("not a Sysclk LWLA device"))?;
        Ok(Self::new(usb, model, Some(transport)))
    }

    fn bitstream(&self, active: bool) -> Option<&'static str> {
        let s = &self.settings;
        protocol::bitstream_name(self.model, active, s.external_clock, s.clock_edge, s.rle)
    }

    /// What this capture's registers and limits come from.
    fn capture(&self) -> Capture {
        let s = &self.settings;
        let (trigger_mask, trigger_values, trigger_edge_mask) =
            trigger_masks(self.trigger.as_ref(), &self.channels);
        Capture {
            samplerate: s.samplerate,
            limit_msec: s.limit_msec,
            limit_samples: s.limit_samples,
            channel_mask: self
                .channels
                .iter()
                .filter(|c| c.enabled)
                .fold(0, |m, c| m | 1 << c.index),
            trigger_mask,
            trigger_edge_mask,
            trigger_values,
            rle: s.rle,
            external_clock: self.model == Model::Lwla1034 && s.external_clock,
            trigger_ext: self.model == Model::Lwla1034 && s.trigger_ext,
            trigger_slope: s.trigger_slope,
        }
    }

    /// One try of `dev_open()`'s loop. An inner `Err` is a failure of the
    /// bitstream or the self test, which the loop tries again; an outer
    /// `Err` ends it.
    fn try_open(&mut self) -> Result<std::result::Result<Link, Error>> {
        let mut t = match self.pending.take() {
            Some(t) => t,
            None => crate::usb::open(&self.usb).inspect_err(|e| {
                log::debug!("Opening: {e}.");
            })?,
        };
        t.force_configuration(protocol::CONFIGURATION).inspect_err(|e| {
            log::error!("Failed to set USB configuration: {}.", libusb_error_name(e));
        })?;
        t.claim_interface(protocol::INTERFACE).inspect_err(|e| {
            log::error!("Failed to claim interface: {}.", libusb_error_name(e));
        })?;
        let mut link = Link::new(t, self.model);
        link.drain()?;
        // "This delay appears to be necessary for reliable operation."
        std::thread::sleep(Duration::from_millis(30));
        let ready = link.apply_config(self.bitstream(true)).and_then(|()| link.init_check());
        Ok(ready.map(|()| link))
    }

    /// The end of a close: the thread joined, the bitstream `shut_down` if
    /// any, the interface released.
    fn finish(&mut self, shut_down: Option<&'static str>) -> Result<()> {
        self.stop()?;
        if let Some(w) = self.worker.take() {
            let _ = w.handle.join();
        }
        if let Some(mut link) = self.link.lock().expect("link lock").take() {
            // A failure only warns, and sigrok-cli's exit stays 0.
            if link.apply_config(shut_down).is_err() {
                log::warn!("Unable to shut down device.");
            }
            if let Err(e) = link.transport().release_interface(protocol::INTERFACE) {
                log::debug!("Releasing the interface: {e}.");
            }
        }
        Ok(())
    }

    /// `config_commit()` and `lwla_start_acquisition()` up to the first
    /// write of the start: what fails the start itself. That first write's
    /// other failures come as a transfer's, on the thread.
    fn begin(&mut self, link: &mut Link) -> Result<Acquisition> {
        let failed = || Error::arg("Failed to start session.");
        if self.trigger.as_ref().is_some_and(|t| t.stages.len() > 1) {
            log::error!("This device only supports 1 trigger stage.");
            return Err(failed());
        }
        link.apply_config(self.bitstream(true)).map_err(|e| {
            log::debug!("Commit: {e}.");
            log::error!("Failed to apply FPGA configuration.");
            failed()
        })?;
        let c = self.capture();
        let (samples_max, duration_max) = limits(&c);
        if c.limit_msec > 0 {
            log::info!("Acquisition time limit {} ms.", c.limit_msec);
        }
        if c.limit_samples > 0 {
            log::info!("Acquisition sample count limit {}.", c.limit_samples);
        }
        if c.external_clock {
            if self.settings.clock_edge == Edge::Positive {
                log::info!("External clock, rising edge.");
            } else {
                log::info!("External clock, falling edge.");
            }
        } else {
            log::info!("Internal clock, samplerate {}.", c.samplerate);
            // The C takes any rate in range without a word, and its limits
            // and the output's header keep the rate asked for.
            if let Some(actual) = protocol::actual_rate(self.model, &c)
                && actual != c.samplerate
            {
                log::warn!("Samplerate {} Hz really runs at {actual} Hz.", c.samplerate);
            }
        }
        link.setup(&c).map_err(|e| {
            log::debug!("Setup: {e}.");
            log::error!("Failed to set up device for acquisition.");
            failed()
        })?;
        let start = protocol::start_regs(self.model, c.trigger_mask);
        let (first_reg, first_value) = start[0];
        let first = link.send_async(&protocol::write_reg_command(first_reg, first_value));
        if matches!(first, Err(Error::Disconnected)) {
            // submit_request() fails, and so does the start (protocol.c:561).
            log::error!("Submit transfer failed: LIBUSB_ERROR_NO_DEVICE.");
            return Err(failed());
        }
        Ok(Acquisition {
            model: self.model,
            capture: c,
            samples_max,
            duration_max,
            first,
            start,
            stop: Arc::new(AtomicBool::new(false)),
        })
    }
}

impl Device for LwlaDevice {
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
        match (group, self.model) {
            (None, Model::Lwla1034) => Ok(OPTIONS_1034.to_vec()),
            (None, Model::Lwla1016) => Ok(OPTIONS_1016.to_vec()),
            (Some(g), _) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let s = &self.settings;
        let is_1034 = self.model == Model::Lwla1034;
        Ok(match key {
            Key::Samplerate => Value::U64(s.samplerate),
            Key::LimitMsec => Value::U64(s.limit_msec),
            Key::LimitSamples => Value::U64(s.limit_samples),
            Key::Rle if !is_1034 => Value::Bool(s.rle),
            Key::ExternalClock if is_1034 => Value::Bool(s.external_clock),
            Key::ClockEdge if is_1034 => Value::Str(s.clock_edge.name().into()),
            Key::TriggerSource if is_1034 => {
                let name = if s.trigger_ext { TRIGGER_SOURCE_TRG } else { TRIGGER_SOURCE_CH };
                Value::Str(name.into())
            }
            Key::TriggerSlope if is_1034 => Value::Str(s.trigger_slope.name().into()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let is_1034 = self.model == Model::Lwla1034;
        let rates = self.model.samplerates();
        let s = &mut self.settings;
        match key {
            Key::Samplerate => {
                // Any rate between the list's ends.
                let v = value.as_u64().ok_or_else(bad)?;
                if v < rates[rates.len() - 1] || v > rates[0] {
                    return Err(Error::arg("samplerate"));
                }
                s.samplerate = v;
            }
            Key::LimitMsec => {
                let v = value.as_u64().ok_or_else(bad)?;
                if v > MAX_LIMIT_MSEC {
                    return Err(bad());
                }
                s.limit_msec = v;
            }
            Key::LimitSamples => {
                let v = value.as_u64().ok_or_else(bad)?;
                if v > MAX_LIMIT_SAMPLES {
                    return Err(bad());
                }
                s.limit_samples = v;
            }
            Key::Rle if !is_1034 => {
                let Value::Bool(v) = value else { return Err(bad()) };
                s.rle = *v;
            }
            Key::ExternalClock if is_1034 => {
                let Value::Bool(v) = value else { return Err(bad()) };
                s.external_clock = *v;
            }
            Key::ClockEdge if is_1034 => {
                s.clock_edge = value.as_str().and_then(Edge::parse).ok_or_else(bad)?;
            }
            Key::TriggerSource if is_1034 => {
                s.trigger_ext = match value.as_str() {
                    Some(TRIGGER_SOURCE_CH) => false,
                    Some(TRIGGER_SOURCE_TRG) => true,
                    _ => return Err(bad()),
                };
            }
            Key::TriggerSlope if is_1034 => {
                s.trigger_slope = value.as_str().and_then(Edge::parse).ok_or_else(bad)?;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let edges = || ValueList::Str(vec!["r".into(), "f".into()]);
        let is_1034 = self.model == Model::Lwla1034;
        Ok(match key {
            Key::Samplerate => ValueList::U64(self.model.samplerates().to_vec()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::ClockEdge | Key::TriggerSlope if is_1034 => edges(),
            Key::TriggerSource if is_1034 => {
                ValueList::Str(vec![TRIGGER_SOURCE_CH.into(), TRIGGER_SOURCE_TRG.into()])
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        // "Try the whole shebang three times, fingers crossed."
        let mut last = Error::device("device not opened");
        for _ in 0..OPEN_TRIES {
            match self.try_open()? {
                Ok(link) => {
                    if link.short_transfer_quirk {
                        log::warn!("Short transfer quirk detected! Memory reads will be slow.");
                    }
                    *self.link.lock().expect("link lock") = Some(link);
                    return Ok(());
                }
                Err(e) => last = e,
            }
        }
        Err(last)
    }

    /// `dev_close()` as `sr_dev_close()` calls it: the device inactive, so
    /// the shut-down bitstream goes out.
    fn close(&mut self) -> Result<()> {
        let name = self.bitstream(false);
        self.finish(name)
    }

    /// `dev_close()` as `sr_exit()` calls it after sigrok-cli's capture:
    /// the device still active and its bitstream unchanged, so that the
    /// FPGA stays on.
    fn close_after_capture(&mut self) -> Result<()> {
        self.finish(None)
    }

    fn trigger_features(&self) -> crate::core::TriggerFeatures {
        TRIGGER_FEATURES
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&TRIGGER_FEATURES)?;
            // api.c:527-533 refuses these at the commit, for enabled
            // channels; a match that cannot be set is refused whatever the
            // channel, since it may be enabled later. The CLI's parser
            // goes by TRIGGER_MATCHES and never sends one.
            let bad = t
                .stages
                .iter()
                .flat_map(|s| &s.conditions)
                .find(|c| !TRIGGER_MATCHES.contains(&c.matches));
            if let Some(c) = bad {
                log::error!("Unsupported trigger match for CH{}.", c.channel + 1);
                return Err(Error::Arg(format!(
                    "Unsupported trigger match for CH{}.",
                    c.channel + 1
                )));
            }
        }
        self.trigger = trigger;
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
        match self.begin(&mut link) {
            Ok(acq) => {
                let slot = Arc::clone(&self.link);
                let stop = Arc::clone(&acq.stop);
                let handle = std::thread::Builder::new()
                    .name("sysclk-lwla acquisition".into())
                    .spawn(move || {
                        acq.run(&mut link, &sink);
                        *slot.lock().expect("link lock") = Some(link);
                    })
                    .map_err(Error::Io)?;
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                *self.link.lock().expect("link lock") = Some(link);
                Err(e)
            }
        }
    }

    /// Asks the thread to end, without waiting: it may still send packets,
    /// which the session drains. [`Device::close`] joins it.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// A capture's failure, which ended it; the log has said why.
struct Failed(Error);
type Step<T = ()> = std::result::Result<T, Failed>;

/// One capture, on its own thread: `transfer_event()` and its callbacks as
/// one sequence.
struct Acquisition {
    model: Model,
    capture: Capture,
    samples_max: u64,
    duration_max: u64,
    /// How the start's first write went.
    first: Result<()>,
    start: Regs,
    stop: Arc<AtomicBool>,
}

/// A failed write of a capture: `submit_transfer()` when the device is
/// gone, else `transfer_out_completed()`.
fn out_failed(state: u32, e: Error) -> Failed {
    if matches!(e, Error::Disconnected) {
        log::error!("Submit transfer failed: LIBUSB_ERROR_NO_DEVICE.");
    } else {
        log::error!(
            "Transfer to device failed (state {state}): {}.",
            protocol::transfer_status_name(&e)
        );
    }
    Failed(e)
}

/// A failed read of a capture: `transfer_in_completed()`.
fn in_failed(state: u32, e: Error) -> Failed {
    if matches!(e, Error::Disconnected) {
        log::error!("Submit transfer failed: LIBUSB_ERROR_NO_DEVICE.");
    } else {
        log::error!(
            "Transfer from device failed (state {state}): {}.",
            protocol::transfer_status_name(&e)
        );
    }
    Failed(e)
}

impl Acquisition {
    fn run(mut self, link: &mut Link, sink: &PacketSink) {
        sink.send(Packet::header());
        // The C ends such a capture as any other, and its exit status stays
        // 0 (protocol.c:299-309).
        let outcome = self.capture(link, sink).map_err(|Failed(e)| e);
        log::info!("Acquisition stopped.");
        sink.end(outcome);
    }

    fn cancelled(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    /// A sequence of register writes. A stop always writes all of it. The
    /// C skips the rest of a sequence once a cancel is asked
    /// (protocol.c:341, since be64f90b; protocol.c:292-293 and 354 still
    /// submit the whole stop). Its stop on the LWLA1034 is then only the long
    /// register's address, with no strobe, which would leave the capture
    /// running if the strobe is what latches a long register
    /// (lwla1034.c:72 reads so; no device has confirmed it). On the LWLA1016
    /// the first write already stops the capture and only `DIV_COUNT` = 0 is
    /// skipped.
    fn write_seq(link: &mut Link, state: u32, regs: &[(u16, u32)]) -> Step {
        for &(reg, value) in regs {
            link.send_async(&protocol::write_reg_command(reg, value))
                .map_err(|e| out_failed(state, e))?;
        }
        Ok(())
    }

    /// A register read: its request and reply (`next_reg_read()`,
    /// `read_reg_response()`).
    fn read_reg(link: &mut Link, state: u32, reg: u16) -> Step<u32> {
        link.send_async(&protocol::read_reg_command(reg)).map_err(|e| out_failed(state, e))?;
        let reply = link.receive_async().map_err(|e| in_failed(state, e))?;
        if reply.len() != 4 {
            log::error!("Received size {} doesn't match expected size 4.", reply.len());
            return Err(Failed(Error::device(format!(
                "received size {} doesn't match expected size 4",
                reply.len()
            ))));
        }
        Ok(protocol::decode32(&reply))
    }

    /// The status poll (`STATE_STATUS_REQUEST`).
    fn status(&self, link: &mut Link) -> Step<Status> {
        match protocol::status_request(self.model) {
            StatusRequest::LongRegs(command) => {
                let state = STATE_STATUS_REQUEST;
                link.send_async(&command).map_err(|e| out_failed(state, e))?;
                let reply = link.receive_async().map_err(|e| in_failed(state, e))?;
                if reply.len() != 40 {
                    log::error!("Received size {} doesn't match expected size 40.", reply.len());
                    return Err(Failed(Error::device(format!(
                        "received size {} doesn't match expected size 40",
                        reply.len()
                    ))));
                }
                Ok(protocol::status_1034(&reply, self.capture.clock_boost()))
            }
            StatusRequest::Regs(regs) => {
                let mut values = [0; 3];
                for (v, reg) in values.iter_mut().zip(regs) {
                    *v = Self::read_reg(link, STATE_STATUS_REQUEST, reg)?;
                }
                Ok(protocol::status_1016(values))
            }
        }
    }

    /// Waits for the next poll: false when a stop came first.
    fn wait_for_poll(&self) -> bool {
        let start = Instant::now();
        while start.elapsed() < POLL_INTERVAL {
            if self.cancelled() {
                return false;
            }
            std::thread::sleep(STOP_CHECK);
        }
        !self.cancelled()
    }

    fn capture(&mut self, link: &mut Link, sink: &PacketSink) -> Step {
        let start = std::mem::take(&mut self.start);
        std::mem::replace(&mut self.first, Ok(()))
            .map_err(|e| out_failed(STATE_START_CAPTURE, e))?;
        Self::write_seq(link, STATE_START_CAPTURE, &start[1..])?;
        log::info!("Acquisition started.");
        let stop = protocol::stop_regs(self.model);
        let mut status = 0;
        loop {
            if !self.wait_for_poll() {
                return Self::write_seq(link, STATE_STOP_CAPTURE, &stop);
            }
            let s = self.status(link)?;
            if self.cancelled() {
                return Self::write_seq(link, STATE_STOP_CAPTURE, &stop);
            }
            log::trace!(
                "Captured {} words, {} ms, status 0x{:02X}.",
                s.mem_fill,
                s.duration,
                s.status
            );
            if !status & s.status & STATUS_TRIGGERED != 0 {
                log::info!("Capture triggered.");
            }
            status = s.status;
            if s.duration >= self.duration_max {
                log::debug!("Time limit reached, stopping capture.");
                Self::write_seq(link, STATE_STOP_CAPTURE, &stop)?;
                if self.cancelled() {
                    return Ok(());
                }
                break;
            } else if s.status & STATUS_TRIGGERED == 0 {
                log::trace!("Waiting for trigger.");
            } else if s.status & STATUS_MEM_AVAIL == 0 {
                log::debug!("Capture memory filled.");
                break;
            } else if s.status & STATUS_CAPTURING != 0 {
                log::trace!("Sampling in progress.");
            }
        }
        self.read_out(link, sink)
    }

    /// The capture's length, the memory read and the sequence after it.
    fn read_out(&self, link: &mut Link, sink: &PacketSink) -> Step {
        let finish = protocol::read_finish_regs(self.model);
        let reg = protocol::length_reg(self.model);
        let length = Self::read_reg(link, STATE_LENGTH_REQUEST, reg)?;
        if self.cancelled() {
            return Self::write_seq(link, STATE_READ_FINISH, &finish);
        }
        let mut readout = Readout::new(
            self.model,
            self.capture.rle,
            link.short_transfer_quirk,
            self.samples_max,
            protocol::read_range(self.model, length),
        );
        if readout.is_empty() {
            return Self::write_seq(link, STATE_READ_FINISH, &finish);
        }
        log::debug!("{} words in capture buffer.", readout.words());
        Self::write_seq(link, STATE_READ_PREPARE, &protocol::read_prepare_regs(self.model))?;
        if self.cancelled() {
            return Self::write_seq(link, STATE_READ_FINISH, &finish);
        }
        let unit = self.model.unit_size();
        let send = |data: Vec<u8>| {
            if !sink.send(Packet::logic(unit, data)) {
                self.stop.store(true, Ordering::SeqCst);
            }
        };
        let mut emit = send;
        Self::read_blocks(&mut readout, link, &|| self.cancelled(), &mut emit)?;
        // The samples are out: what fails from here is logged and no failure.
        let _ = Self::write_seq(link, STATE_READ_FINISH, &finish);
        Ok(())
    }

    /// The memory reads, until the limit, the end or a stop. What was
    /// decoded goes out however they end. The C loses the samples not yet
    /// in a sent packet: on a wrong reply size (protocol.c:216-219), on a
    /// failed transfer (protocol.c:325-330, then 299-309) and on a stop
    /// (protocol.c:237); they were read all right.
    fn read_blocks(
        readout: &mut Readout,
        link: &mut Link,
        cancelled: &dyn Fn() -> bool,
        emit: &mut dyn FnMut(Vec<u8>),
    ) -> Step {
        let got = loop {
            let got = Self::read_block(readout, link, cancelled, emit);
            if got.is_err() || cancelled() || !readout.more() {
                break got;
            }
        };
        if let Some(p) = readout.flush() {
            emit(p);
        }
        got
    }

    /// One memory read (`STATE_READ_REQUEST`).
    fn read_block(
        readout: &mut Readout,
        link: &mut Link,
        cancelled: &dyn Fn() -> bool,
        emit: &mut dyn FnMut(Vec<u8>),
    ) -> Step {
        let state = STATE_READ_REQUEST;
        let command = readout.next_request();
        link.send_async(&command).map_err(|e| out_failed(state, e))?;
        let reply = link.receive_async().map_err(|e| in_failed(state, e))?;
        readout.take(&reply, cancelled, emit).map_err(|w| {
            log::error!("Received size {} does not match expected size {}.", w.got, w.want);
            Failed(Error::device(format!(
                "received size {} does not match expected size {}",
                w.got, w.want
            )))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::trigger::{TriggerCondition, TriggerStage};
    use crate::usb::record::Replay;
    use std::cell::Cell;

    fn hex(b: &[u8]) -> String {
        use std::fmt::Write as _;
        b.iter().fold(String::new(), |mut s, x| {
            let _ = write!(s, "{x:02x}");
            s
        })
    }

    /// A replay of `steps` after a fixture's device line.
    fn replay(model: Model, steps: &[String]) -> Replay {
        let pid = if model == Model::Lwla1034 { 6689 } else { 6688 };
        let mut text = format!(
            "# oxrok usb fixture 1\ndevice vid=2961 pid={pid} version=0000 interfaces=0:ff:ff:ff\n"
        );
        for s in steps {
            text.push_str(s);
            text.push('\n');
        }
        Replay::from_text(&text).unwrap()
    }

    /// The steps of a sequence of register writes.
    fn writes(regs: &[(u16, u32)]) -> Vec<String> {
        regs.iter()
            .map(|&(r, v)| format!("bulk_out 02 {} -> 8", hex(&protocol::write_reg_command(r, v))))
            .collect()
    }

    /// A device whose link runs on `r`, with `load` for its bitstreams.
    fn open_on(r: &Replay, model: Model, load: protocol::LoadBitstream) -> LwlaDevice {
        let d = device(model);
        let mut link = Link::new(Box::new(r.clone()), model);
        link.load = load;
        *d.link.lock().unwrap() = Some(link);
        d
    }

    fn no_bitstream(name: &str) -> Result<Vec<u8>> {
        panic!("{name} loaded");
    }

    #[allow(clippy::unnecessary_wraps)] // Must be a LoadBitstream.
    fn one_byte(_: &str) -> Result<Vec<u8>> {
        Ok(vec![0xaa])
    }

    #[test]
    fn close_sends_the_shut_down_bitstream() {
        let steps = ["bulk_out 04 00000005aa -> 5".to_owned(), "release 0 -> ok".to_owned()];
        let r = replay(Model::Lwla1034, &steps);
        open_on(&r, Model::Lwla1034, one_byte).close().unwrap();
        r.finish().unwrap();
    }

    #[test]
    fn the_lwla1016_has_no_shut_down_bitstream() {
        let r = replay(Model::Lwla1016, &["release 0 -> ok".to_owned()]);
        open_on(&r, Model::Lwla1016, no_bitstream).close().unwrap();
        r.finish().unwrap();
    }

    #[test]
    fn close_after_capture_leaves_the_fpga_on() {
        let r = replay(Model::Lwla1034, &["release 0 -> ok".to_owned()]);
        open_on(&r, Model::Lwla1034, no_bitstream).close_after_capture().unwrap();
        r.finish().unwrap();
    }

    fn acquisition(model: Model, stop: bool) -> Acquisition {
        Acquisition {
            model,
            capture: device(model).capture(),
            samples_max: 1000,
            duration_max: 1000,
            first: Ok(()),
            start: protocol::start_regs(model, 0),
            stop: Arc::new(AtomicBool::new(stop)),
        }
    }

    fn sink() -> (PacketSink, std::sync::mpsc::Receiver<(usize, Packet)>) {
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        (PacketSink::new(tx, 0), rx)
    }

    #[test]
    fn a_stop_in_the_status_wait_writes_the_whole_sequence() {
        for model in [Model::Lwla1034, Model::Lwla1016] {
            let start = protocol::start_regs(model, 0);
            let mut steps = writes(&start[1..]);
            steps.extend(writes(&protocol::stop_regs(model)));
            let r = replay(model, &steps);
            let mut link = Link::new(Box::new(r.clone()), model);
            let (sink, rx) = sink();
            assert!(acquisition(model, true).capture(&mut link, &sink).is_ok());
            r.finish().unwrap();
            assert!(rx.try_recv().is_err());
        }
    }

    #[test]
    fn a_stop_after_the_length_writes_the_whole_finish() {
        let model = Model::Lwla1016;
        let mut steps = vec![
            format!("bulk_out 02 {} -> 4", hex(&protocol::read_reg_command(0x10b0))),
            "bulk_in 86 1024 -> 00000800".to_owned(),
        ];
        steps.extend(writes(&protocol::read_finish_regs(model)));
        let r = replay(model, &steps);
        let mut link = Link::new(Box::new(r.clone()), model);
        let (sink, _rx) = sink();
        assert!(acquisition(model, true).read_out(&mut link, &sink).is_ok());
        r.finish().unwrap();
    }

    #[test]
    fn a_stop_during_the_readout_ends_the_reads_and_keeps_the_samples() {
        // Two blocks wanted; the stop comes after the first was decoded,
        // leaving a packet partly filled, which protocol.c:237 would drop.
        let model = Model::Lwla1016;
        let mut readout = Readout::new(model, false, false, 1000, (2, 300));
        let command = protocol::read_mem_command(3, 2, 250);
        let steps = [
            format!("bulk_out 02 {} -> 10", hex(&command)),
            format!("bulk_in 86 1024 -> {}", hex(&[0x11; 1000])),
        ];
        let r = replay(model, &steps);
        let mut link = Link::new(Box::new(r.clone()), model);
        let asked = Cell::new(0);
        let cancelled = || {
            asked.set(asked.get() + 1);
            asked.get() > 1
        };
        let mut packets = Vec::new();
        assert!(
            Acquisition::read_blocks(&mut readout, &mut link, &cancelled, &mut |p| packets.push(p))
                .is_ok()
        );
        r.finish().unwrap();
        assert_eq!(packets, vec![vec![0x11; 1000]]);
    }

    #[test]
    fn a_failed_read_keeps_the_samples() {
        let model = Model::Lwla1016;
        let mut readout = Readout::new(model, false, false, 1000, (2, 300));
        let steps = [
            format!("bulk_out 02 {} -> 10", hex(&protocol::read_mem_command(3, 2, 250))),
            format!("bulk_in 86 1024 -> {}", hex(&[0x11; 1000])),
            format!("bulk_out 02 {} -> 10", hex(&protocol::read_mem_command(3, 252, 48))),
            "bulk_in 86 1024 -> 0011".to_owned(),
        ];
        let r = replay(model, &steps);
        let mut link = Link::new(Box::new(r.clone()), model);
        let mut packets = Vec::new();
        assert!(
            Acquisition::read_blocks(&mut readout, &mut link, &|| false, &mut |p| packets.push(p))
                .is_err()
        );
        r.finish().unwrap();
        assert_eq!(packets, vec![vec![0x11; 1000]]);
    }

    #[test]
    fn a_match_the_device_lacks_is_refused() {
        let mut d = device(Model::Lwla1034);
        assert!(d.set_trigger(Some(trigger(&[(0, TriggerMatch::Edge)]))).is_err());
        for m in TRIGGER_MATCHES {
            d.set_trigger(Some(trigger(&[(0, m)]))).unwrap();
        }
    }

    fn device(model: Model) -> LwlaDevice {
        LwlaDevice::new(UsbDeviceInfo::default(), model, None)
    }

    #[test]
    fn options_follow_the_model() {
        let d = device(Model::Lwla1034);
        assert_eq!(d.options(None).unwrap().len(), 8);
        assert_eq!(d.channels.len(), 34);
        assert_eq!(d.channels[33].name, "CH34");
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(125_000_000));
        assert!(d.get(&Key::Rle, None).is_err());
        let d = device(Model::Lwla1016);
        assert_eq!(d.options(None).unwrap().len(), 5);
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        assert_eq!(d.get(&Key::Rle, None).unwrap(), Value::Bool(true));
        assert!(d.get(&Key::ExternalClock, None).is_err());
    }

    #[test]
    fn refused_values() {
        let mut d = device(Model::Lwla1034);
        assert!(d.set(&Key::Samplerate, &Value::U64(99), None).is_err());
        assert!(d.set(&Key::Samplerate, &Value::U64(125_000_001), None).is_err());
        // Anything between the ends goes.
        d.set(&Key::Samplerate, &Value::U64(3_000_000), None).unwrap();
        assert!(d.set(&Key::LimitMsec, &Value::U64(MAX_LIMIT_MSEC + 1), None).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::U64(MAX_LIMIT_SAMPLES + 1), None).is_err());
        assert!(d.set(&Key::ClockEdge, &Value::Str("x".into()), None).is_err());
        assert!(d.set(&Key::TriggerSource, &Value::Str("x".into()), None).is_err());
        d.set(&Key::TriggerSource, &Value::Str("TRG".into()), None).unwrap();
        assert_eq!(d.get(&Key::TriggerSource, None).unwrap(), Value::Str("TRG".into()));
        assert!(d.set(&Key::Rle, &Value::Bool(true), None).is_err());
    }

    #[test]
    fn limits_derive_on_the_internal_clock_only() {
        let c = device(Model::Lwla1034).capture();
        let c = Capture { samplerate: 1000, ..c };
        assert_eq!(
            limits(&Capture { limit_samples: 64, ..c }),
            (64, 65),
            "64 samples at 1 kHz, and 1 ms more"
        );
        assert_eq!(limits(&Capture { limit_msec: 300, ..c }), (300, 300));
        assert_eq!(limits(&Capture { limit_msec: 300, limit_samples: 5, ..c }), (5, 300));
        // Shorter than a sample: no samples.
        assert_eq!(limits(&Capture { limit_msec: 5, samplerate: 100, ..c }).0, 0);
        assert_eq!(limits(&c), (MAX_LIMIT_SAMPLES, MAX_LIMIT_MSEC));
        let ext = Capture { external_clock: true, ..c };
        assert_eq!(limits(&Capture { limit_samples: 64, ..ext }), (64, MAX_LIMIT_MSEC));
        assert_eq!(limits(&Capture { limit_msec: 9, ..ext }), (MAX_LIMIT_SAMPLES, 9));
    }

    #[test]
    fn the_largest_limits_do_not_overflow() {
        let c = device(Model::Lwla1034).capture();
        limits(&Capture { limit_samples: MAX_LIMIT_SAMPLES, samplerate: 100, ..c });
        limits(&Capture { limit_msec: MAX_LIMIT_MSEC, samplerate: 125_000_000, ..c });
    }

    fn trigger(conditions: &[(usize, TriggerMatch)]) -> Trigger {
        let mut t = Trigger::new();
        t.stages.push(TriggerStage {
            conditions: conditions
                .iter()
                .map(|&(channel, matches)| TriggerCondition { channel, matches, value: 0.0 })
                .collect(),
            count: None,
            duration: None,
            range: None,
        });
        t
    }

    #[test]
    fn trigger_masks_skip_disabled_channels() {
        let mut d = device(Model::Lwla1034);
        let t = trigger(&[
            (0, TriggerMatch::Zero),
            (1, TriggerMatch::One),
            (2, TriggerMatch::Rising),
            (3, TriggerMatch::Falling),
            (33, TriggerMatch::Rising),
        ]);
        assert_eq!(
            trigger_masks(Some(&t), &d.channels),
            (0x2_0000_000f, 0x2_0000_0006, 0x2_0000_000c)
        );
        d.enable_channel(33, false).unwrap();
        assert_eq!(trigger_masks(Some(&t), &d.channels), (0xf, 6, 0xc));
        // No trigger clears what an earlier one set, where the C keeps it
        // (api.c:501-502).
        assert_eq!(trigger_masks(None, &d.channels), (0, 0, 0));
    }

    #[test]
    fn the_channel_mask_is_the_enabled_ones() {
        let mut d = device(Model::Lwla1016);
        assert_eq!(d.capture().channel_mask, 0xffff);
        d.enable_channel(1, false).unwrap();
        assert_eq!(d.capture().channel_mask, 0xfffd);
    }

    #[test]
    fn a_stop_does_not_wait_for_the_thread() {
        let mut d = device(Model::Lwla1016);
        // With no thread it is safe, and sends nothing.
        d.stop().unwrap();
        d.close().unwrap();
    }
}
