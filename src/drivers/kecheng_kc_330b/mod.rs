// Counterpart in sigrok:
//   libsigrok/src/hardware/kecheng-kc-330b/api.c,
//   libsigrok/src/hardware/kecheng-kc-330b/protocol.c (the acquisition),
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Martin Ling,
//   Soeren Apel, Daniel Elstner, Frank Stettner, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Kecheng KC-330B sound level meter: live readings and the meter's own
//! log, over vendor bulk transfers. The driver is `kecheng-kc-330b`.
//!
//! `-c data_source=Live`, the default, polls at the sample interval;
//! `-c data_source=Memory` reads the log, as much of it as `--samples`
//! allows, and ends the capture by itself. So `--continuous` reads a whole
//! log. The protocol is in [`protocol`].
//!
//! **Reading a log never clears it.** The C's comment says a configure
//! clears the log (`api.c:430`). Nothing else shows that, and this driver
//! takes it as true: a log capture sends no configure, unless the user also
//! set something other than `data_source` (`-c sample_interval=...`, a
//! weighting) and the whole log has been read and written out. A refused
//! start, a cut (`--samples`), a stop and a failed read or write send none.
//! A live capture and a `--set` outside memory mode do configure, so they
//! clear it, by that comment; the C's live capture does too.
//!
//! `conn=` is taken and not listed, as the C lists no scan option. The C's
//! `kecheng_kc_330b_set_date_time()` (`protocol.c:232`) is called by nothing
//! (`api.c:141`), and this has no counterpart of it.
//!
//! Where this does not follow the C, because the C is wrong:
//! - `sample_interval` is a `uint64` in ms: libsigrok's type for the key
//!   (`hwdriver.c:113`) and sigrok-cli's reading of it (`session.c:365`). The
//!   C's tuples end `--show` and any set with a `GLib` critical
//!   (`api.c:215`, `api.c:259`) and, in the meta packet, every read of a log
//!   (`api.c:367`).
//! - A log request is 4 bytes and the C asks for 1 to have gone out
//!   (`api.c:405`), so its reads fail on a request that went out; it never
//!   asks for a second chunk (`protocol.c:78` tests its idle state twice)
//!   and sends one sample per chunk (`protocol.c:177`). This reads them all.
//! - An answer that fails or does not parse leaves the C waiting for ever
//!   (`protocol.c:149`, `protocol.c:153`). This asks again at the next
//!   interval, up to [`MAX_BAD_ANSWERS`] in a row, and ends the capture.
//! - The C's close sends the configuration when it is not dirty
//!   (`api.c:190`), against its comment: a `--set` never reaches the meter,
//!   and a `--show` or `--get` configures it. [`Device::close`] sends it when
//!   a setting really changed and not in memory mode. The capture's close
//!   keeps the C's wire for a live capture
//!   ([`Device::close_after_capture`]), which has already configured.
//! - The stop of a refused log start configures (`session.c:831`,
//!   `api.c:429`), so the C clears a log it never read, against its comment
//!   at `api.c:433`. This sends nothing.
//! - The C's timeouts (5, 10, 15 and 40 ms) miss on a loaded host, and a
//!   configure's ack takes about 32 ms by its comment. These are longer
//!   ([`protocol::OUT_TIMEOUT`] and on), which changes no successful transfer.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::analog::{Mq, MqFlags, Unit};
use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, libusb_error_name, split_conn};
use protocol::{Settings, Source};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// Answers in a row a live capture takes that fail or do not parse.
pub const MAX_BAD_ANSWERS: u32 = 5;

/// How often the pause between live requests looks at the stop flag and
/// the clock: the C's event source (`api.c:381`).
const POLL: Duration = Duration::from_millis(10);

static CLASSES: [DeviceClass; 1] = [DeviceClass::SoundLevelMeter];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 7] = [
    OptionDesc::new(Key::Continuous, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::SampleInterval, Access::GET_SET_LIST),
    // The C lists a get and answers SR_ERR_NA (api.c:219); oxrok's --get
    // words that from the access.
    OptionDesc::new(Key::Datalog, Access { get: false, set: false, list: false }),
    OptionDesc::new(Key::SplWeightFreq, Access::GET_SET_LIST),
    OptionDesc::new(Key::SplWeightTime, Access::GET_SET_LIST),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
];

/// The driver.
pub struct Kc330bDriver;

/// The identify the scan sends; a meter that does not answer it is none.
fn identify(t: &mut dyn Transport) -> Result<String> {
    t.claim_interface(protocol::INTERFACE)?;
    let result = (|| {
        t.bulk_out(protocol::EP_OUT, &[protocol::IDENTIFY], protocol::OUT_TIMEOUT).inspect_err(
            |e| log::debug!("Failed to send Identify command: {}", libusb_error_name(e)),
        )?;
        let answer = t
            .bulk_in(protocol::EP_IN, protocol::IDENTIFY_READ, protocol::ANSWER_TIMEOUT)
            .inspect_err(|e| log::debug!("Failed to receive response: {}", libusb_error_name(e)))?;
        protocol::parse_identify(&answer).ok_or_else(|| {
            log::debug!("Invalid response to Identify command");
            Error::device("invalid response to Identify command")
        })
    })();
    let _ = t.release_interface(protocol::INTERFACE);
    result
}

impl Driver for Kc330bDriver {
    fn id(&self) -> &'static str {
        "kecheng-kc-330b"
    }

    fn long_name(&self) -> &'static str {
        "Kecheng KC-330B"
    }

    /// Taken, not listed: the C lists no scan option (`api.c:296`) and
    /// `--show` follows it.
    fn accepted_scan_options(&self) -> &'static [Key] {
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
                log::error!("{msg}");
                return Ok(Vec::new());
            }
        };
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::list_devices()? {
            if info.vendor_id != protocol::VENDOR || info.product_id != protocol::PRODUCT {
                continue;
            }
            if conn.is_some_and(|c| !c.matches(&info)) {
                continue;
            }
            let Ok(mut t) = crate::usb::open(&info) else { continue };
            let Ok(model) = identify(t.as_mut()) else { continue };
            drop(t);
            let conn_text = conn.is_some().then(|| info.conn());
            let mut dev = Kc330bDevice::new(model, Some(info), None);
            dev.info.conn = conn_text;
            devices.push(Box::new(dev));
        }
        Ok(devices)
    }
}

/// What the acquisition thread and the device share.
struct Shared {
    /// The transport while open and not acquiring.
    transport: Option<Box<dyn Transport>>,
    settings: Settings,
    /// What the meter last took, or the defaults the C assumes of it: only
    /// a real change makes the settings dirty.
    sent: Settings,
    /// The last capture's start was refused: sigrok-cli closes nothing then.
    refused: bool,
    /// The last capture read every sample of the log.
    read_all: bool,
}

impl Shared {
    /// `kecheng_kc_330b_configure()`: its failures are logged and left to
    /// the caller to ignore, as every caller in the C does.
    fn configure(&mut self) {
        let Some(t) = self.transport.as_mut() else { return };
        if configure(t.as_mut(), &self.settings).is_ok() {
            self.sent = self.settings;
        }
    }

    /// The settings changed since the meter last took them.
    fn dirty(&self) -> bool {
        self.settings != self.sent
    }

    /// As [`Shared::dirty`], not counting the data source: choosing to read
    /// the log is no reason to clear it.
    fn dirty_beyond_source(&self) -> bool {
        Settings { source: self.sent.source, ..self.settings } != self.sent
    }
}

fn configure(t: &mut dyn Transport, settings: &Settings) -> Result<()> {
    log::debug!("Configuring device.");
    t.bulk_out(protocol::EP_OUT, &settings.command(), protocol::OUT_TIMEOUT)
        .inspect_err(|e| log::debug!("Failed to configure device: {}", libusb_error_name(e)))?;
    let answer = t.bulk_in(protocol::EP_IN, 1, protocol::CONFIGURE_TIMEOUT).inspect_err(|e| {
        log::debug!("Failed to configure device (no ack): {}", libusb_error_name(e));
    })?;
    if answer != [protocol::CONFIGURE | 0x80] {
        log::debug!("Failed to configure device: invalid response {answer:02x?}");
        return Err(Error::device("invalid response to the configure command"));
    }
    Ok(())
}

/// Whether the meter is activated.
fn status(t: &mut dyn Transport) -> Result<bool> {
    log::debug!("Getting device status.");
    t.bulk_out(protocol::EP_OUT, &[protocol::STATUS], protocol::OUT_TIMEOUT)
        .inspect_err(|e| log::debug!("Failed to get status: {}", libusb_error_name(e)))?;
    let answer = t
        .bulk_in(protocol::EP_IN, 1, protocol::ANSWER_TIMEOUT)
        .inspect_err(|e| log::debug!("Failed to get status (no ack): {}", libusb_error_name(e)))?;
    protocol::parse_status(&answer).ok_or_else(|| {
        log::debug!("Failed to get status: invalid response {answer:02x?}");
        Error::device("invalid response to the status command")
    })
}

fn log_info(t: &mut dyn Transport) -> Result<protocol::LogInfo> {
    log::debug!("Getting logging info.");
    t.bulk_out(protocol::EP_OUT, &[protocol::LOG_INFO], protocol::OUT_TIMEOUT)
        .inspect_err(|e| log::debug!("Failed to get status: {}", libusb_error_name(e)))?;
    let answer = t
        .bulk_in(protocol::EP_IN, 9, protocol::ANSWER_TIMEOUT)
        .inspect_err(|e| log::debug!("Failed to get status (no ack): {}", libusb_error_name(e)))?;
    protocol::parse_log_info(&answer).ok_or_else(|| {
        log::debug!("Failed to get log info: invalid response {answer:02x?}");
        Error::device("invalid response to the log info command")
    })
}

/// What a capture does once its start has gone through.
enum Job {
    /// The first request went out.
    Live { interval: Duration, limit: u64, sent: Instant, flags: MqFlags },
    /// The first request went out, for the chunk at 0 of `total`.
    Log { total: usize, whole: bool, flags: MqFlags },
    /// The log is empty: a header and an end (`api.c:357`).
    Nothing,
}

fn analog(samples: Vec<f32>, flags: MqFlags) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![0],
        samples,
        mq: Mq::SoundPressureLevel,
        flags,
        unit: Unit::DecibelSpl,
        digits: 1,
    })
}

fn flags_of(slow: bool, c_weight: bool) -> MqFlags {
    let time = if slow { MqFlags::SPL_TIME_WEIGHT_S } else { MqFlags::SPL_TIME_WEIGHT_F };
    let freq = if c_weight { MqFlags::SPL_FREQ_WEIGHT_C } else { MqFlags::SPL_FREQ_WEIGHT_A };
    time | freq
}

/// The live polling: the answer to the request gone out, then the next
/// request once the interval has passed, until the limit or a stop.
fn run_live(
    t: &mut dyn Transport,
    sink: &PacketSink,
    stop: &AtomicBool,
    (interval, limit, mut sent, flags): (Duration, u64, Instant, MqFlags),
) -> Result<()> {
    let mut samples = 0u64;
    let mut bad = 0;
    loop {
        match t.bulk_in(protocol::EP_IN, 3, protocol::DATA_TIMEOUT) {
            Ok(answer) => {
                if let Some(level) = protocol::parse_live(&answer) {
                    bad = 0;
                    samples = samples.saturating_add(1);
                    if !sink.send(analog(vec![level], flags)) || (limit != 0 && samples >= limit) {
                        return Ok(());
                    }
                } else {
                    log::debug!("Received invalid SPL packet.");
                    bad += 1;
                }
            }
            Err(Error::Disconnected) => return Err(Error::Disconnected),
            Err(e) => {
                log::debug!("Failed to read a level: {}", libusb_error_name(&e));
                bad += 1;
            }
        }
        if bad >= MAX_BAD_ANSWERS {
            log::error!("{bad} bad answers in a row: giving up.");
            return Err(Error::device(format!("{bad} bad answers in a row")));
        }
        while sent.elapsed() <= interval {
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            std::thread::sleep(POLL.min(interval.saturating_sub(sent.elapsed())));
        }
        if stop.load(Ordering::SeqCst) {
            return Ok(());
        }
        if let Err(e) = t.bulk_out(protocol::EP_OUT, &[protocol::LIVE], protocol::OUT_TIMEOUT) {
            log::debug!("Failed to request new acquisition: {}", libusb_error_name(&e));
            return Err(e);
        }
        sent = Instant::now();
    }
}

/// The log's chunks, the first one's request gone out. Whether every
/// sample of `total` came, or a stop came first; an error ends it too.
fn run_log(
    t: &mut dyn Transport,
    sink: &PacketSink,
    stop: &AtomicBool,
    total: usize,
    flags: MqFlags,
) -> Result<bool> {
    let mut done = 0;
    loop {
        let want = protocol::log_request(done, total);
        let answer = match t.bulk_in(
            protocol::EP_IN,
            protocol::log_answer_len(&want),
            protocol::DATA_TIMEOUT,
        ) {
            Ok(a) => a,
            Err(e) => {
                log::error!("Failed to read the log: {}.", libusb_error_name(&e));
                return Err(e);
            }
        };
        let Some(levels) = protocol::parse_log_data(&answer) else {
            log::debug!("Received invalid stored SPL packet.");
            log::error!("Failed to read the log: an invalid answer.");
            return Err(Error::device("failed to read the log: an invalid answer"));
        };
        let got = levels.len();
        if got != usize::from(want[3]) {
            log::error!("Failed to read the log: {got} samples of {}.", want[3]);
            if got > 0 {
                sink.send(analog(levels, flags));
            }
            return Err(Error::device(format!(
                "failed to read the log: {got} samples of {}",
                want[3]
            )));
        }
        if !sink.send(analog(levels, flags)) {
            return Ok(false);
        }
        done += got;
        if done >= total {
            return Ok(true);
        }
        if stop.load(Ordering::SeqCst) {
            return Ok(false);
        }
        let next = protocol::log_request(done, total);
        if let Err(e) = t.bulk_out(protocol::EP_OUT, &next, protocol::OUT_TIMEOUT) {
            log::error!("Failed to request next chunk: {}.", libusb_error_name(&e));
            return Err(e);
        }
    }
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// One meter.
pub struct Kc330bDevice {
    info: DeviceInfo,
    usb: Option<UsbDeviceInfo>,
    channels: Vec<Channel>,
    limit_samples: u64,
    shared: Arc<Mutex<Shared>>,
    worker: Option<Worker>,
}

impl Kc330bDevice {
    fn new(
        model: String,
        usb: Option<UsbDeviceInfo>,
        transport: Option<Box<dyn Transport>>,
    ) -> Kc330bDevice {
        Kc330bDevice {
            info: DeviceInfo {
                driver: "kecheng-kc-330b",
                vendor: "Kecheng".to_owned(),
                model,
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            usb,
            channels: vec![Channel::new(0, ChannelKind::Analog, "SPL")],
            limit_samples: 0,
            shared: Arc::new(Mutex::new(Shared {
                transport,
                settings: Settings::default(),
                sent: Settings::default(),
                refused: false,
                read_all: false,
            })),
            worker: None,
        }
    }

    /// A meter on an open transport, for tests and replays.
    pub fn from_transport(transport: Box<dyn Transport>, model: impl Into<String>) -> Self {
        Kc330bDevice::new(model.into(), None, Some(transport))
    }

    fn shared(&self) -> std::sync::MutexGuard<'_, Shared> {
        self.shared.lock().expect("kc-330b state lock")
    }

    fn join(&mut self) {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
    }

    /// The start up to the first request. An error refuses it.
    fn prepare(&mut self) -> Result<(Job, Vec<(Key, Value)>)> {
        let limit = self.limit_samples;
        let mut shared = self.shared();
        let shared = &mut *shared;
        let t = shared.transport.as_mut().ok_or_else(|| Error::device("device is not open"))?;
        match shared.settings.source {
            Source::Live => {
                // Forced, and a failure ignored (api.c:340).
                if configure(t.as_mut(), &shared.settings).is_ok() {
                    shared.sent = shared.settings;
                }
                if !status(t.as_mut())? {
                    // The C's comment: it reads 30.0 until the button is
                    // pressed, so it goes on.
                    log::error!("Device is inactive");
                }
                let flags = flags_of(shared.settings.slow, shared.settings.c_weight);
                let interval =
                    Duration::from_millis(protocol::INTERVALS_MS[shared.settings.interval]);
                t.bulk_out(protocol::EP_OUT, &[protocol::LIVE], protocol::OUT_TIMEOUT)
                    .inspect_err(|e| {
                        log::debug!("Failed to start acquisition: {}", libusb_error_name(e));
                    })?;
                Ok((Job::Live { interval, limit, sent: Instant::now(), flags }, Vec::new()))
            }
            Source::Memory => {
                let info = log_info(t.as_mut())?;
                // The log's weightings are its samples' (api.c:354), not
                // settings: a later configure must not write them.
                if info.stored == 0 {
                    return Ok((Job::Nothing, Vec::new()));
                }
                let stored = usize::from(info.stored);
                let total = match usize::try_from(limit) {
                    Ok(l) if l != 0 && l < stored => l,
                    _ => stored,
                };
                t.bulk_out(
                    protocol::EP_OUT,
                    &protocol::log_request(0, total),
                    protocol::OUT_TIMEOUT,
                )
                .inspect_err(|e| {
                    log::debug!("Failed to start acquisition: {}", libusb_error_name(e));
                })?;
                let meta =
                    vec![(Key::SampleInterval, Value::U64(protocol::INTERVALS_MS[info.interval]))];
                let flags = flags_of(info.slow, info.c_weight);
                Ok((Job::Log { total, whole: total == stored, flags }, meta))
            }
        }
    }
}

impl Device for Kc330bDevice {
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
        let c =
            self.channels.get_mut(pos).ok_or_else(|| Error::Arg(format!("No channel {pos}.")))?;
        c.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let c =
            self.channels.get_mut(pos).ok_or_else(|| Error::Arg(format!("No channel {pos}.")))?;
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
        let s = self.shared().settings;
        Ok(match key {
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::SampleInterval => Value::U64(protocol::INTERVALS_MS[s.interval]),
            Key::SplWeightFreq => Value::Str(if s.c_weight { "C" } else { "A" }.to_owned()),
            Key::SplWeightTime => Value::Str(if s.slow { "S" } else { "F" }.to_owned()),
            Key::DataSource => {
                Value::Str(if s.source == Source::Live { "Live" } else { "Memory" }.to_owned())
            }
            // Datalog: "There really isn't a way to be sure the device is
            // logging."
            other => return Err(Error::NotSupported(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        if self.worker.as_ref().is_some_and(|w| !w.handle.is_finished()) {
            return Err(Error::Busy);
        }
        let text = value.as_str().unwrap_or_default();
        // SR_ERR_ARG's wording, as sr_strerror() has it.
        let invalid = || Error::Io(std::io::Error::other("invalid argument"));
        match key {
            Key::LimitSamples => {
                // sr_config_set() refuses 0 before the C sees it (hwdriver.c).
                self.limit_samples = value.as_u64().filter(|n| *n != 0).ok_or_else(invalid)?;
                return Ok(());
            }
            Key::SampleInterval => {
                let ms = value.as_u64().ok_or_else(invalid)?;
                let i = protocol::INTERVALS_MS.iter().position(|v| *v == ms).ok_or_else(|| {
                    Error::Arg(format!(
                        "sample_interval is in ms, one of {:?}",
                        protocol::INTERVALS_MS
                    ))
                })?;
                self.shared().settings.interval = i;
            }
            Key::SplWeightFreq => match text {
                "A" => self.shared().settings.c_weight = false,
                "C" => self.shared().settings.c_weight = true,
                _ => return Err(invalid()),
            },
            Key::SplWeightTime => match text {
                "F" => self.shared().settings.slow = false,
                "S" => self.shared().settings.slow = true,
                _ => return Err(invalid()),
            },
            Key::DataSource => match text {
                "Live" => self.shared().settings.source = Source::Live,
                "Memory" => self.shared().settings.source = Source::Memory,
                _ => return Err(invalid()),
            },
            other => return Err(Error::NotSupported(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let strings = |v: &[&str]| ValueList::Str(v.iter().map(|s| (*s).to_owned()).collect());
        match key {
            Key::SampleInterval => Ok(ValueList::U64(protocol::INTERVALS_MS.to_vec())),
            Key::SplWeightFreq => Ok(strings(&["A", "C"])),
            Key::SplWeightTime => Ok(strings(&["F", "S"])),
            Key::DataSource => Ok(strings(&["Live", "Memory"])),
            other => Err(Error::NotSupported(other.id().to_owned())),
        }
    }

    fn open(&mut self) -> Result<()> {
        let usb = self.usb.clone();
        let mut shared = self.shared();
        if shared.transport.is_some() {
            return Ok(());
        }
        let info = usb.ok_or_else(|| Error::device("no USB device to open"))?;
        let mut t = crate::usb::open(&info)?;
        // libusb_set_configuration() goes out whatever is active (api.c:164).
        t.force_configuration(protocol::CONFIGURATION).inspect_err(|e| {
            log::error!("Failed to set configuration: {}.", libusb_error_name(e));
        })?;
        t.claim_interface(protocol::INTERFACE).inspect_err(|e| {
            log::error!("Failed to claim interface: {}.", libusb_error_name(e));
        })?;
        shared.transport = Some(t);
        Ok(())
    }

    /// Closes as `dev_close()` means to: a configuration not yet sent
    /// goes out, so that a client can configure without a capture. Not in
    /// memory mode: by `api.c:430` it would clear the log.
    fn close(&mut self) -> Result<()> {
        self.join();
        let mut shared = self.shared();
        if shared.dirty() && shared.settings.source == Source::Live {
            shared.configure();
        }
        if let Some(mut t) = shared.transport.take() {
            let _ = t.release_interface(protocol::INTERFACE);
        }
        Ok(())
    }

    /// The end of the CLI's capture. Live, as `dev_close()` does after
    /// sigrok-cli's capture, for the same wire: a configure unless the settings
    /// are dirty (`api.c:190`), which is none after a refused start, where
    /// sigrok-cli does not get this far. A log, only for a setting beyond the
    /// data source and only once every sample has reached the output: the
    /// session stops a device whose output failed, also at its last packet.
    fn close_after_capture(&mut self) -> Result<()> {
        let stopped = self.worker.as_ref().is_some_and(|w| w.stop.load(Ordering::SeqCst));
        self.join();
        let mut shared = self.shared();
        let configure = match shared.settings.source {
            Source::Live => !shared.dirty() && !shared.refused,
            Source::Memory => shared.read_all && !stopped && shared.dirty_beyond_source(),
        };
        if configure {
            shared.configure();
        }
        if let Some(mut t) = shared.transport.take() {
            let _ = t.release_interface(protocol::INTERFACE);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        match self.worker.take() {
            Some(w) if w.handle.is_finished() => {
                let _ = w.handle.join();
            }
            Some(w) => {
                self.worker = Some(w);
                return Err(Error::Busy);
            }
            None => {}
        }
        {
            let mut shared = self.shared();
            shared.refused = false;
            shared.read_all = false;
        }
        let (job, meta) = match self.prepare() {
            Ok(j) => j,
            Err(e) => {
                // No configure, where the session's stop has one
                // (session.c:831, api.c:429): the log was not read.
                self.shared().refused = true;
                return Err(Error::device(e.to_string()));
            }
        };
        let Some(mut t) = self.shared().transport.take() else {
            return Err(Error::device("device is not open"));
        };
        let shared = Arc::clone(&self.shared);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("kc-330b acquisition".into())
            .spawn(move || {
                sink.send(Packet::header());
                if !meta.is_empty() {
                    sink.send(Packet::Meta(meta));
                }
                let mut read_all = false;
                let outcome = match job {
                    Job::Live { interval, limit, sent, flags } => {
                        run_live(t.as_mut(), &sink, &stop2, (interval, limit, sent, flags))
                    }
                    Job::Log { total, whole, flags } => {
                        run_log(t.as_mut(), &sink, &stop2, total, flags).map(|complete| {
                            read_all = complete && whole;
                        })
                    }
                    Job::Nothing => Ok(()),
                };
                // The transport first: whoever sees the end may use it.
                let mut s = shared.lock().expect("kc-330b state lock");
                s.transport = Some(t);
                s.read_all = read_all;
                drop(s);
                sink.end(outcome);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;
    use std::sync::mpsc::sync_channel;

    /// The configure a fixture offers after the steps a test expects, as a
    /// sentinel: a driver that sends it takes the steps and they no longer
    /// remain, whereas with no step left a configure would only fail, and a
    /// failure is ignored.
    const MEMORY_CONFIGURE: [&str; 2] = ["bulk_out 02 01002878000001 -> 7", "bulk_in 81 1 -> 81"];
    /// The same with the C weighting set.
    const MEMORY_C_CONFIGURE: [&str; 2] = ["bulk_out 02 01002878000101 -> 7", "bulk_in 81 1 -> 81"];
    const LIVE_CONFIGURE: [&str; 2] = ["bulk_out 02 01002878000000 -> 7", "bulk_in 81 1 -> 81"];

    /// One step of a fixture per line, in the dialect of `docs/USB.md`, and
    /// a handle on the same cursor.
    fn device(steps: &[&str]) -> (Kc330bDevice, Replay) {
        let mut text = String::from(
            "# oxrok usb fixture 1\ndevice vid=1041 pid=8101 version=0100 bus=0 addr=1\n",
        );
        for s in steps {
            text.push_str(s);
            text.push('\n');
        }
        let replay = Replay::from_text(&text).unwrap();
        let probe = replay.clone();
        (Kc330bDevice::from_transport(Box::new(replay), "KC-330B"), probe)
    }

    fn with(steps: &[&str], sentinel: [&'static str; 2]) -> (Kc330bDevice, Replay) {
        let mut all = steps.to_vec();
        all.extend(sentinel);
        device(&all)
    }

    fn collect(d: &mut Kc330bDevice) -> Vec<Packet> {
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let mut out = Vec::new();
        for (_, p) in &rx {
            let end = p == Packet::End;
            out.push(p);
            if end {
                break;
            }
        }
        out
    }

    fn levels(packets: &[Packet]) -> Vec<f32> {
        packets
            .iter()
            .filter_map(|p| if let Packet::Analog(a) = p { Some(a.samples.clone()) } else { None })
            .flatten()
            .collect()
    }

    fn memory(d: &mut Kc330bDevice) {
        d.set(&Key::DataSource, &Value::Str("Memory".into()), None).unwrap();
    }

    fn c_weight(d: &mut Kc330bDevice) {
        d.set(&Key::SplWeightFreq, &Value::Str("C".into()), None).unwrap();
    }

    /// A log of two samples, read whole.
    const TWO: [&str; 4] = [
        "bulk_out 02 05 -> 1",
        "bulk_in 81 9 -> 850228780101000002",
        "bulk_out 02 07000002 -> 4",
        "bulk_in 81 5 -> 87012c0139",
    ];

    #[test]
    fn options_follow_the_cs_order_and_values() {
        let (mut d, _) = device(&[]);
        let keys: Vec<_> = d.options(None).unwrap().iter().map(|o| o.key.id().to_owned()).collect();
        assert_eq!(
            keys,
            [
                "continuous",
                "limit_samples",
                "sample_interval",
                "datalog",
                "spl_weight_freq",
                "spl_weight_time",
                "data_source"
            ]
        );
        assert_eq!(d.get(&Key::SampleInterval, None).unwrap(), Value::U64(125));
        assert!(d.get(&Key::Datalog, None).is_err());
        d.set(&Key::SampleInterval, &Value::U64(60000), None).unwrap();
        assert_eq!(d.shared().settings.command()[1], 6);
        let hint = d.set(&Key::SampleInterval, &Value::U64(126), None).unwrap_err().to_string();
        assert!(hint.contains("in ms") && hint.contains("125, 500"), "{hint}");
        assert!(d.set(&Key::SplWeightFreq, &Value::Str("Z".into()), None).is_err());
        assert!(d.set(&Key::SplWeightTime, &Value::Str("X".into()), None).is_err());
        assert!(d.set(&Key::DataSource, &Value::Str("Tape".into()), None).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
        assert!(d.get(&Key::SampleInterval, Some("nope")).is_err());
    }

    #[test]
    fn close_sends_nothing_when_nothing_changed() {
        let (mut d, probe) = device(&LIVE_CONFIGURE);
        d.close().unwrap();
        assert_eq!(probe.remaining(), 2);
    }

    #[test]
    fn close_sends_a_changed_setting() {
        let (mut d, probe) = device(&["bulk_out 02 01012878000000 -> 7", "bulk_in 81 1 -> 81"]);
        d.set(&Key::SampleInterval, &Value::U64(500), None).unwrap();
        d.close().unwrap();
        assert_eq!(probe.remaining(), 0);
    }

    #[test]
    fn a_set_to_what_it_is_changes_nothing() {
        let (mut d, probe) = device(&LIVE_CONFIGURE);
        d.set(&Key::SampleInterval, &Value::U64(125), None).unwrap();
        d.set(&Key::SplWeightFreq, &Value::Str("A".into()), None).unwrap();
        d.set(&Key::SplWeightTime, &Value::Str("F".into()), None).unwrap();
        d.set(&Key::DataSource, &Value::Str("Live".into()), None).unwrap();
        assert!(!d.shared().dirty());
        d.close().unwrap();
        assert_eq!(probe.remaining(), 2);
    }

    #[test]
    fn a_set_undone_changes_nothing() {
        let (mut d, probe) = device(&LIVE_CONFIGURE);
        memory(&mut d);
        d.set(&Key::DataSource, &Value::Str("Live".into()), None).unwrap();
        d.close().unwrap();
        assert_eq!(probe.remaining(), 2);
    }

    #[test]
    fn close_in_memory_mode_sends_nothing() {
        let (mut d, probe) = device(&MEMORY_C_CONFIGURE);
        memory(&mut d);
        c_weight(&mut d);
        d.close().unwrap();
        assert_eq!(probe.remaining(), 2);
    }

    #[test]
    fn a_whole_log_is_not_cleared_for_choosing_memory() {
        let (mut d, probe) = with(&TWO, MEMORY_CONFIGURE);
        memory(&mut d);
        let got = collect(&mut d);
        assert_eq!(levels(&got), [30.0, 31.3]);
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 2, "no configure");
    }

    #[test]
    fn a_whole_log_with_another_setting_applies_it() {
        let (mut d, probe) = with(&TWO, MEMORY_C_CONFIGURE);
        memory(&mut d);
        c_weight(&mut d);
        d.set(&Key::SplWeightTime, &Value::Str("F".into()), None).unwrap();
        let got = collect(&mut d);
        d.join();
        assert_eq!(probe.remaining(), 2, "not before the output has had it all");
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 0, "the user's settings, not the log's");
        let Some(Packet::Analog(a)) = got.iter().find(|p| matches!(p, Packet::Analog(_))) else {
            panic!("no samples");
        };
        assert_eq!(a.flags, flags_of(true, true), "the samples carry the log's");
        assert_eq!(d.get(&Key::SplWeightTime, None).unwrap(), Value::Str("F".into()));
    }

    #[test]
    fn a_log_is_not_cleared_when_the_output_failed() {
        let (mut d, probe) = with(&TWO, MEMORY_C_CONFIGURE);
        memory(&mut d);
        c_weight(&mut d);
        let got = collect(&mut d);
        assert_eq!(levels(&got).len(), 2);
        // What the session does when the consumer asks to stop.
        d.stop().unwrap();
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 2);
    }

    #[test]
    fn a_log_comes_in_blocks() {
        let mut first = String::from("bulk_in 81 127 -> 87");
        first.push_str(&"012c".repeat(63));
        let (mut d, probe) = device(&[
            "bulk_out 02 05 -> 1",
            "bulk_in 81 9 -> 850228780101000040",
            "bulk_out 02 0700003f -> 4",
            &first,
            "bulk_out 02 07000101 -> 4",
            "bulk_in 81 3 -> 870139",
        ]);
        memory(&mut d);
        let got = collect(&mut d);
        let l = levels(&got);
        assert_eq!(l.len(), 64);
        assert_eq!(l[63], 31.3);
        assert!(got.contains(&Packet::meta(Key::SampleInterval, Value::U64(1000))));
        assert_eq!(probe.remaining(), 0);
    }

    #[test]
    fn a_limit_cuts_the_log_and_leaves_it_on_the_meter() {
        let (mut d, probe) = with(
            &[
                "bulk_out 02 05 -> 1",
                "bulk_in 81 9 -> 850228780101000064",
                "bulk_out 02 07000002 -> 4",
                "bulk_in 81 5 -> 87012c0139",
            ],
            MEMORY_C_CONFIGURE,
        );
        memory(&mut d);
        c_weight(&mut d);
        d.limit_samples = 2;
        assert_eq!(levels(&collect(&mut d)), [30.0, 31.3]);
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 2);
    }

    #[test]
    fn a_stop_ends_the_log_between_chunks() {
        let mut chunk = String::from("bulk_in 81 127 -> 87");
        chunk.push_str(&"012c".repeat(63));
        let (mut d, probe) = with(
            &[
                "bulk_out 02 05 -> 1",
                "bulk_in 81 9 -> 8502287801010000fc",
                "bulk_out 02 0700003f -> 4",
                &chunk,
                // On offer: a run that ignores the stop reads it.
                "bulk_out 02 0700013f -> 4",
                &chunk,
            ],
            MEMORY_C_CONFIGURE,
        );
        memory(&mut d);
        c_weight(&mut d);
        let (tx, rx) = sync_channel(1);
        // Taken from the queue only after the stop: the first chunk's send
        // waits behind the header until then.
        d.start(PacketSink::new(tx, 0)).unwrap();
        d.stop().unwrap();
        let mut samples = 0;
        for (_, p) in &rx {
            if let Packet::Analog(a) = &p {
                samples += a.samples.len();
            }
            if p == Packet::End {
                break;
            }
        }
        assert_eq!(samples, 63, "one chunk, the stop seen before the next");
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 4, "the second chunk and the configure");
    }

    #[test]
    fn a_failed_chunk_keeps_what_came_and_the_log() {
        let (mut d, probe) = with(
            &[
                "bulk_out 02 05 -> 1",
                "bulk_in 81 9 -> 850228780101000005",
                "bulk_out 02 07000005 -> 4",
                "bulk_in 81 11 -> 87012c0139",
            ],
            MEMORY_C_CONFIGURE,
        );
        memory(&mut d);
        c_weight(&mut d);
        assert_eq!(levels(&collect(&mut d)), [30.0, 31.3]);
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 2);
    }

    #[test]
    fn a_refused_log_start_sends_no_configure() {
        for answer in ["bulk_in 81 9 -> c50228780101000064", "bulk_in 81 9 -> err timeout"] {
            let (mut d, probe) = with(&["bulk_out 02 05 -> 1", answer], MEMORY_C_CONFIGURE);
            memory(&mut d);
            c_weight(&mut d);
            let (tx, _rx) = sync_channel(4);
            assert!(d.start(PacketSink::new(tx, 0)).is_err());
            assert_eq!(probe.remaining(), 2, "{answer}");
            d.close_after_capture().unwrap();
            assert_eq!(probe.remaining(), 2, "{answer}: and none at the close");
        }
        // With nothing but the data source set, as a plain read has it.
        let (mut d, probe) =
            with(&["bulk_out 02 05 -> 1", "bulk_in 81 9 -> err timeout"], MEMORY_CONFIGURE);
        memory(&mut d);
        let (tx, _rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 2);
    }

    #[test]
    fn a_refused_log_request_sends_no_configure() {
        let (mut d, probe) = with(
            &[
                "bulk_out 02 05 -> 1",
                "bulk_in 81 9 -> 850228780101000064",
                "bulk_out 02 0700003f -> err device \"endpoint stalled\"",
            ],
            MEMORY_C_CONFIGURE,
        );
        memory(&mut d);
        c_weight(&mut d);
        let (tx, _rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 2);
    }

    #[test]
    fn an_empty_log_is_a_header_and_an_end() {
        let (mut d, probe) = with(
            &["bulk_out 02 05 -> 1", "bulk_in 81 9 -> 850228780101000000"],
            MEMORY_C_CONFIGURE,
        );
        memory(&mut d);
        c_weight(&mut d);
        let got = collect(&mut d);
        assert!(matches!(got[0], Packet::Header { .. }));
        assert_eq!(got.len(), 2);
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 2);
    }

    /// The start of a live capture, up to the first request.
    const LIVE: [&str; 4] = [
        "bulk_out 02 01002878000000 -> 7",
        "bulk_in 81 1 -> 81",
        "bulk_out 02 04 -> 1",
        "bulk_in 81 1 -> 84",
    ];
    const ASK: &str = "bulk_out 02 08 -> 1";
    const BAD: &str = "bulk_in 81 3 -> c8012c";
    const GOOD: &str = "bulk_in 81 3 -> 880151";

    #[test]
    fn live_configures_again_at_the_capture_s_close_as_the_c_does() {
        let (mut d, probe) = device(&[LIVE.as_slice(), &[ASK, GOOD], &LIVE_CONFIGURE].concat());
        d.limit_samples = 1;
        assert_eq!(levels(&collect(&mut d)), [33.7]);
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 0);
    }

    #[test]
    fn live_takes_a_bad_answer_and_goes_on() {
        let (mut d, probe) = device(&[LIVE.as_slice(), &[ASK, BAD, ASK, GOOD]].concat());
        d.limit_samples = 1;
        assert_eq!(levels(&collect(&mut d)), [33.7]);
        assert_eq!(probe.remaining(), 0);
    }

    #[test]
    fn live_gives_up_at_the_bound_with_more_on_offer() {
        let mut steps = LIVE.to_vec();
        // Five, not the constant: a changed bound must fail here.
        for _ in 0..5 {
            steps.extend([ASK, BAD]);
        }
        // A driver that goes on gets a good answer and has a sample.
        steps.extend([ASK, GOOD]);
        let (mut d, probe) = device(&steps);
        d.limit_samples = 1;
        assert_eq!(levels(&collect(&mut d)), [0.0f32; 0]);
        assert_eq!(probe.remaining(), 2, "the bound ended it, not the fixture");
    }

    #[test]
    fn the_bound_counts_bad_answers_in_a_row() {
        let mut steps = LIVE.to_vec();
        for _ in 0..4 {
            steps.extend([ASK, BAD]);
        }
        steps.extend([ASK, GOOD]);
        for _ in 0..4 {
            steps.extend([ASK, BAD]);
        }
        steps.extend([ASK, GOOD]);
        let (mut d, probe) = device(&steps);
        d.limit_samples = 2;
        assert_eq!(levels(&collect(&mut d)), [33.7, 33.7]);
        assert_eq!(probe.remaining(), 0);
    }

    #[test]
    fn inactive_goes_on_and_a_bad_status_refuses() {
        let (mut d, _) = device(&[
            "bulk_out 02 01002878000000 -> 7",
            "bulk_in 81 1 -> 81",
            "bulk_out 02 04 -> 1",
            "bulk_in 81 1 -> a4",
            ASK,
            "bulk_in 81 3 -> 88012c",
        ]);
        d.limit_samples = 1;
        assert_eq!(levels(&collect(&mut d)), [30.0]);

        let (mut d, probe) = with(
            &[
                "bulk_out 02 01002878000000 -> 7",
                "bulk_in 81 1 -> 81",
                "bulk_out 02 04 -> 1",
                "bulk_in 81 1 -> c4",
            ],
            LIVE_CONFIGURE,
        );
        let (tx, _rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
        d.close_after_capture().unwrap();
        assert_eq!(probe.remaining(), 2, "sigrok-cli closes nothing after a refusal");
    }

    #[test]
    fn flags_name_the_weightings() {
        assert_eq!(flags_of(false, false), MqFlags::SPL_TIME_WEIGHT_F | MqFlags::SPL_FREQ_WEIGHT_A);
        assert_eq!(flags_of(true, true), MqFlags::SPL_TIME_WEIGHT_S | MqFlags::SPL_FREQ_WEIGHT_C);
    }

    #[test]
    fn no_conn_listed() {
        let driver = Kc330bDriver;
        assert_eq!(driver.scan_options().len(), 0);
        assert_eq!(driver.accepted_scan_options(), [Key::Conn]);
    }
}
