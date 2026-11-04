// Counterpart in sigrok:
//   libsigrok/src/hardware/hantek-4032l/api.c,
//   libsigrok/src/hardware/hantek-4032l/protocol.c (the capture),
//   Copyright (C) 2016 Andreas Zschunke <andreas.zschunke@gmx.net>,
//   Copyright (C) 2017 Andrej Valek <andy@skyrain.eu>,
//   Copyright (C) 2017 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Gerhard Sittig, Romain Tartière
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! hantek-4032l: the Hantek 4032L logic analyzer, 32 channels in two
//! groups of 16 (A and B), with a sample memory of up to 64M samples per
//! channel and hardware triggers.
//!
//! The board needs no firmware. Opening claims interface 0 and asks the
//! status, which holds the FPGA version: version 0 has no external clock
//! and takes one data transfer at a time. A capture is a reset request,
//! a configure, status polls until the capture is complete, a get, and
//! the memory's stream (see [`protocol`]). The sample limit is what the
//! board captures: the driver rounds it up to a multiple of 512 and reads
//! all of it.
//!
//! The sequence follows `api.c` and `protocol.c` step by step, so that
//! the device sees the same transactions as under sigrok-cli (checked
//! with the emulator, `tools/emu/hantek-4032l.sh`), which includes the
//! driver's habits: every status command is followed by two reads of
//! which the first counts, and a read that fails leaves the buffer as it
//! was, so the status is judged on the answer before it. No firmware,
//! FPGA source or recording of this device is at hand, so what the C
//! driver sends against its own comments (a capture ratio of 100, PWM
//! 4096) stays as it is.
//!
//! Where the port differs, because the C driver only logs a failed
//! transfer at debug level and goes on:
//!
//! - A refused configure or get ends the capture: the board was never
//!   told, so no status or stream can come (the C driver polls for ever).
//! - Failed transfers are counted: [`MAX_FAILURES`] in a row end the
//!   capture, and from the get on so do timeouts, as the stream is
//!   complete in the board's memory by then. While the status is polled a
//!   timeout is only a wait: `protocol.c` asks again on any status other
//!   than 2, which is how it waits for a trigger, and has no bound on
//!   silence either, and whether a board can fall silent during that wait
//!   is not known. A trigger that never comes waits until the capture is
//!   stopped, as in the C driver.
//! - A count of 4 Gi samples and more is refused (`api.c` loses its upper
//!   half), and the end word is only looked for in the data that came
//!   (`protocol.c` reads past them when the last transfer is full).
//!
//! All data transfers stay queued until the stream ends, where the C
//! driver frees those the remaining samples no longer need. They stay
//! empty and are cancelled when the stream is dropped; the device sees
//! no other transaction.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error,
    Key, OptionDesc, Packet, PacketSink, Result, Trigger, TriggerFeatures, TriggerMatch, Value,
    ValueList,
};
use crate::usb::{BulkStream, ControlRequest, Transport, UsbDeviceInfo, UsbId, split_conn};
use protocol::{
    CMD_CONFIGURE, CMD_GET, CMD_STATUS, CmdPacket, DATA_TRANSFER, END_MAGIC, EP_IN, EP_OUT,
    HwTrigger, POLL_SIZE, START_MAGIC, STATUS_COMPLETE, STATUS_MAGIC, word_at,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

/// The device's id.
pub const ID: UsbId = UsbId { vendor: protocol::VID, product: protocol::PID };
const INTERFACE: u8 = 0;
const NUM_CHANNELS: usize = 32;
/// `H4032L_USB_TIMEOUT`, for commands and each transfer.
const TIMEOUT: Duration = Duration::from_millis(500);
/// The wait after the reset request (`h4032l_start()`).
const RESET_WAIT: Duration = Duration::from_millis(20);
/// The reads `h4032l_get_fpga_version()` gives the status's magic. A
/// status command gets as many here, where the C driver reads on for ever,
/// before it is sent again: the open has refused more leftovers already.
const VERSION_READS: usize = 10;
/// Failed transfers in a row after which the capture ends. The C driver
/// has no limit; fx2lafw's `MAX_EMPTY_TRANSFERS` is the size taken.
pub const MAX_FAILURES: usize = 64;

/// The driver.
pub struct Hantek4032lDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
/// `devopts[]`.
static DEV_OPTIONS: [OptionDesc; 8] = [
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET_LIST),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::ExternalClock, Access::GET_SET),
    OptionDesc::new(Key::ExternalClockSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::ClockEdge, Access::GET_SET_LIST),
];
/// `devopts_fpga_zero[]`: the first five of [`DEV_OPTIONS`].
const FPGA_ZERO_OPTIONS: usize = 5;
/// `devopts_cg[]`.
static GROUP_OPTIONS: [OptionDesc; 1] =
    [OptionDesc::new(Key::VoltageThreshold, Access::GET_SET_LIST)];
/// `trigger_matches[]`.
static TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];
/// One stage of levels and an edge ("Only one trigger stage supported
/// for now").
const TRIGGER_FEATURES: TriggerFeatures = TriggerFeatures {
    count: false,
    consecutive: false,
    duration: false,
    range: false,
    or: false,
    and: false,
    max_stages: 1,
};

impl Driver for Hantek4032lDriver {
    fn id(&self) -> &'static str {
        "hantek-4032l"
    }

    fn long_name(&self) -> &'static str {
        "Hantek 4032L"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    /// `scan()`: every device with the ids, or those `conn` names. The
    /// device is not touched.
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
                // sr_usb_find() finds nothing, and the scan no device.
                log::error!("{msg}");
                return Ok(Vec::new());
            }
        };
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::find_devices(&[ID])? {
            if conn.is_none_or(|c| c.matches(&info)) {
                devices.push(Box::new(Hantek4032lDevice::new(info, None)));
            }
        }
        Ok(devices)
    }
}

/// `hantek_4032l` settings kept between acquisitions.
#[derive(Clone, Copy, Debug)]
struct Settings {
    /// The index into [`protocol::RATE_CODES`].
    rate: usize,
    capture_ratio: u64,
    external_clock: bool,
    clock_source: usize,
    clock_edge: usize,
    thresholds: [f64; 2],
}

/// The transport and the read buffer (`devc->buf`), which a failed read
/// leaves as it was.
struct Link {
    transport: Box<dyn Transport>,
    buf: [u8; POLL_SIZE],
}

impl Link {
    /// One read into the buffer: the bytes that came.
    fn poll(&mut self) -> Result<usize> {
        let data = self.transport.bulk_in(EP_IN, POLL_SIZE, TIMEOUT)?;
        let n = data.len().min(POLL_SIZE);
        self.buf[..n].copy_from_slice(&data[..n]);
        Ok(n)
    }

    /// `h4032l_get_fpga_version()`.
    fn fpga_version(&mut self, packet: &CmdPacket) -> Result<u32> {
        let name = crate::usb::libusb_error_name;
        self.transport.bulk_out(EP_OUT, &packet.bytes(CMD_STATUS), TIMEOUT).inspect_err(|e| {
            log::error!("Unable to send FPGA version request: {}.", name(e));
        })?;
        for _ in 0..VERSION_READS {
            self.poll().inspect_err(|e| {
                log::error!("Unable to receive FPGA version: {}.", name(e));
            })?;
            if let Some(version) = protocol::status_fpga_version(&self.buf) {
                log::debug!("FPGA version: {version:#x}.");
                return Ok(version);
            }
        }
        log::error!("Unable to get FPGA version.");
        Err(Error::device("unable to get the FPGA version"))
    }
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// One Hantek 4032L.
pub struct Hantek4032lDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    settings: Settings,
    /// `devc->cmd_pkt`: its sample count is a setting, and the open's
    /// status command carries what is in it.
    packet: CmdPacket,
    /// 0 until the open has read it.
    fpga_version: u32,
    trigger: Option<Trigger>,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

impl Hantek4032lDevice {
    fn new(usb: UsbDeviceInfo, transport: Option<Box<dyn Transport>>) -> Self {
        // Channel n is A(n/2) for even n and B(n/2) for odd n.
        let name = |i: usize| format!("{}{}", if i.is_multiple_of(2) { 'A' } else { 'B' }, i / 2);
        let threshold = protocol::default_threshold();
        Hantek4032lDevice {
            info: DeviceInfo {
                driver: "hantek-4032l",
                vendor: "Hantek".into(),
                model: "4032L".into(),
                version: String::new(),
                serial: String::new(),
                conn: Some(usb.conn()),
            },
            usb,
            channels: (0..NUM_CHANNELS)
                .map(|i| Channel::new(i, ChannelKind::Logic, name(i)))
                .collect(),
            groups: vec![
                ChannelGroup::new("A", (0..NUM_CHANNELS).step_by(2).collect()),
                ChannelGroup::new("B", (1..NUM_CHANNELS).step_by(2).collect()),
            ],
            settings: Settings {
                rate: 0,
                capture_ratio: 5,
                external_clock: false,
                clock_source: 0,
                clock_edge: 0,
                thresholds: [threshold; 2],
            },
            packet: CmdPacket::default(),
            fpga_version: 0,
            trigger: None,
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on an already open transport (a replayed recording).
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        if usb.id() != ID {
            return Err(Error::arg("not a Hantek 4032L"));
        }
        Ok(Self::new(usb, Some(transport)))
    }

    /// `h4032l_dev_open()`: the device with the ids at the scanned port.
    fn find_and_open(&mut self) -> Result<Box<dyn Transport>> {
        let usb = &self.usb;
        let found = crate::usb::find_devices(&[ID])?.into_iter().find(|d| {
            d.bus == usb.bus
                && if usb.ports.is_empty() || d.ports.is_empty() {
                    d.address == usb.address
                } else {
                    d.ports == usb.ports
                }
        });
        let Some(info) = found else { return Err(Error::device("device not found")) };
        let transport = crate::usb::open(&info).inspect_err(|e| {
            log::error!("Failed to open device: {}.", crate::usb::libusb_error_name(e));
        })?;
        self.info.conn = Some(info.conn());
        self.usb = info;
        Ok(transport)
    }

    /// The packet of a capture, from the settings and the trigger, as
    /// `dev_acquisition_start()` fills it. Trigger 1 is built anew: the C
    /// driver clears only the enable flags (api.c:521-523), so a capture
    /// with no trigger after a triggered one sends the old `trigger[0]`
    /// again, and a level trigger after an edge trigger the old edge
    /// channel. A second stage does not get here ([`Trigger::check`] in
    /// `set_trigger`); `Err` is the message to log.
    fn capture_packet(&self) -> std::result::Result<CmdPacket, &'static str> {
        let s = &self.settings;
        let samples = self.packet.samples;
        let trigger = match self.trigger.as_ref().and_then(|t| t.stages.first()) {
            None => None,
            Some(stage) => {
                let conditions = stage.conditions.iter().map(|c| (c.channel, c.matches));
                Some(HwTrigger::from_conditions(conditions)?)
            }
        };
        Ok(CmdPacket {
            rate: if s.external_clock {
                protocol::external_clock_code(s.clock_source, s.clock_edge)
            } else {
                u8::try_from(s.rate).expect("36 codes")
            },
            pwm: s.thresholds.map(protocol::voltage_to_pwm),
            samples,
            pre: u32::try_from(u64::from(samples) * s.capture_ratio / 100)
                .expect("the ratio is at most 100"),
            trigger,
        })
    }
}

/// The index of a channel group's name.
fn group_index(group: &str) -> Result<usize> {
    match group {
        "A" => Ok(0),
        "B" => Ok(1),
        other => Err(Error::ChannelGroup(other.to_owned())),
    }
}

impl Device for Hantek4032lDevice {
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
            Some(g) => {
                group_index(g)?;
                Ok(GROUP_OPTIONS.to_vec())
            }
            // No external clock and edges on FPGA version 0.
            None if self.fpga_version == 0 => Ok(DEV_OPTIONS[..FPGA_ZERO_OPTIONS].to_vec()),
            None => Ok(DEV_OPTIONS.to_vec()),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let s = &self.settings;
        Ok(match (group, key) {
            (Some(g), Key::VoltageThreshold) => {
                let t = s.thresholds[group_index(g)?];
                Value::RangeF64(t, t)
            }
            (None, Key::Samplerate) => Value::U64(protocol::RATE_CODES[s.rate]),
            (None, Key::CaptureRatio) => Value::U64(s.capture_ratio),
            (None, Key::LimitSamples) => Value::U64(u64::from(self.packet.samples)),
            (None, Key::ExternalClock) => Value::Bool(s.external_clock),
            (None, Key::ExternalClockSource) => {
                Value::Str(protocol::CLOCK_SOURCES[s.clock_source].into())
            }
            (None, Key::ClockEdge) => Value::Str(protocol::CLOCK_EDGES[s.clock_edge].into()),
            (None, Key::Conn) => match &self.info.conn {
                Some(c) => Value::Str(c.clone()),
                None => return Err(Error::UnknownKey(key.id().to_owned())),
            },
            _ => return Err(Error::UnknownKey(key.id().to_owned())),
        })
    }

    /// As in the C driver a set reaches the device with the next capture,
    /// so one during a capture waits for that.
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let s = &mut self.settings;
        match (group, key) {
            (Some(g), Key::VoltageThreshold) => {
                let Value::RangeF64(low, high) = value else { return Err(bad()) };
                s.thresholds[group_index(g)?] = (low + high) / 2.0;
            }
            (None, Key::Samplerate) => {
                let rate = value.as_u64().ok_or_else(bad)?;
                let Some(code) = protocol::rate_code(rate) else {
                    log::error!("Invalid sample rate.");
                    return Err(Error::arg("invalid sample rate"));
                };
                s.rate = code;
            }
            (None, Key::CaptureRatio) => {
                let r = value.as_u64().ok_or_else(bad)?;
                // sr_config_set() refuses it before the driver sees it
                // (hwdriver.c:757-764); api.c:410 itself takes any value.
                if r > 100 {
                    return Err(Error::arg("capture ratio must be 0..100"));
                }
                s.capture_ratio = r;
            }
            (None, Key::LimitSamples) => {
                let limit = value.as_u64().ok_or_else(bad)?;
                self.packet.samples = protocol::round_samples(limit).map_err(|rounded| {
                    log::error!("Invalid sample range 2k...64M: {rounded}.");
                    Error::arg("invalid sample range")
                })?;
            }
            (None, Key::ExternalClock) => s.external_clock = value.as_bool().ok_or_else(bad)?,
            (None, Key::ExternalClockSource) => {
                let name = value.as_str().ok_or_else(bad)?;
                s.clock_source =
                    protocol::CLOCK_SOURCES.iter().position(|n| *n == name).ok_or_else(bad)?;
            }
            (None, Key::ClockEdge) => {
                let name = value.as_str().ok_or_else(bad)?;
                s.clock_edge =
                    protocol::CLOCK_EDGES.iter().position(|n| *n == name).ok_or_else(bad)?;
            }
            _ => return Err(Error::UnknownKey(key.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let names = |l: &[&str]| ValueList::Str(l.iter().map(|n| (*n).to_owned()).collect());
        Ok(match key {
            Key::Samplerate => ValueList::U64(protocol::SAMPLERATES.to_vec()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::VoltageThreshold if group.is_some() => ValueList::RangeF64(protocol::thresholds()),
            Key::LimitSamples => ValueList::MinMaxU64(protocol::SAMPLES_MIN, protocol::SAMPLES_MAX),
            Key::ClockEdge => names(&protocol::CLOCK_EDGES),
            Key::ExternalClockSource => names(&protocol::CLOCK_SOURCES),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `dev_open()`: the device opened, its interface claimed, its FPGA
    /// version read.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let transport = if let Some(t) = self.pending.take() {
            t
        } else {
            self.find_and_open().inspect_err(|_| log::error!("Unable to open device."))?
        };
        let mut link = Link { transport, buf: [0; POLL_SIZE] };
        link.transport.claim_interface(INTERFACE).inspect_err(|e| match e {
            Error::Busy => log::error!(
                "Unable to claim USB interface. Another program or driver has already claimed it."
            ),
            Error::Disconnected => log::error!("Device has been disconnected."),
            e => log::error!("Unable to claim interface: {}.", crate::usb::libusb_error_name(e)),
        })?;
        self.fpga_version = link.fpga_version(&self.packet)?;
        *self.link.lock().expect("link lock") = Some(link);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(w) = self.worker.take() {
            let _ = w.handle.join();
        }
        if let Some(link) = self.link.lock().expect("link lock").take() {
            log::info!(
                "Closing device on {} (logical) / {:?} (physical) interface {INTERFACE}.",
                self.usb.conn(),
                self.usb.ports
            );
            drop(link);
        }
        Ok(())
    }

    fn trigger_features(&self) -> TriggerFeatures {
        TRIGGER_FEATURES
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&TRIGGER_FEATURES)?;
        }
        self.trigger = trigger;
        Ok(())
    }

    /// `dev_acquisition_start()` and `h4032l_start()`: the packet, the
    /// reset request and its wait, then the thread takes over with the
    /// configure.
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
        let packet = self.capture_packet().map_err(|msg| {
            log::error!("{msg}");
            Error::arg(msg)
        })?;
        self.packet = packet;
        let Some(mut link) = self.link.lock().expect("link lock").take() else {
            return Err(Error::device("device is not open"));
        };
        let reset = ControlRequest::vendor(protocol::REQ_RESET, 0, 0);
        if let Err(e) = link.transport.control_out(reset, &protocol::RESET_DATA, TIMEOUT) {
            log::error!("Failed to send vendor request {}.", crate::usb::libusb_error_name(&e));
            *self.link.lock().expect("link lock") = Some(link);
            return Err(Error::device("failed to send the reset request"));
        }
        std::thread::sleep(RESET_WAIT);
        let acq = Acquisition::new(packet, self.fpga_version);
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("hantek-4032l acquisition".into())
            .spawn(move || {
                acq.run(&mut link, &sink, &stop2);
                // The link first: whoever sees the end may use it.
                *slot.lock().expect("link lock") = Some(link);
                sink.send(Packet::End);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// `h4032l_stop()` with nothing to send: the thread looks at the flag
    /// between transfers. It is not waited for here, as it must be able
    /// to hand its packets to the session calling this; `close` joins it.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// Why a capture ended early; what was to say has been logged.
struct Over;

/// One capture, on its own thread: the states of `h4032l_usb_callback()`
/// and `h4032l_data_transfer_callback()` written as a sequence.
struct Acquisition {
    packet: CmdPacket,
    fpga_version: u32,
    /// `devc->remaining_samples`.
    remaining: u32,
    /// `devc->sent_samples`.
    sent: u32,
    /// `devc->trigger_pos`.
    trigger_pos: u32,
    /// Commands that failed, in a row.
    failed_commands: usize,
    /// Reads or data transfers that failed, in a row.
    failed_reads: usize,
}

impl Acquisition {
    fn new(packet: CmdPacket, fpga_version: u32) -> Self {
        Acquisition {
            packet,
            fpga_version,
            remaining: packet.samples,
            sent: 0,
            trigger_pos: packet.pre,
            failed_commands: 0,
            failed_reads: 0,
        }
    }

    fn run(mut self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) {
        // Everything to say has been said on the way out.
        let _ = self.capture(link, sink, stop);
    }

    fn capture(
        &mut self,
        link: &mut Link,
        sink: &PacketSink,
        stop: &AtomicBool,
    ) -> std::result::Result<(), Over> {
        self.vital_command(link, CMD_CONFIGURE, "configure")?;
        self.wait_for_capture(link, stop)?;
        checked(stop)?;
        self.vital_command(link, CMD_GET, "get")?;
        // Trigger has been captured.
        if !sink.send(Packet::header()) {
            return Err(Over);
        }
        let n = self.stream_start(link, stop)?;
        // The first read holds the start word and the first samples.
        let count = (n / 4 - 1).min(self.remaining as usize);
        let first = link.buf;
        self.send_samples(&first[4..4 + count * 4], sink)?;
        checked(stop)?;
        self.stream_data(link, sink, stop)
    }

    /// A command the capture cannot go on without: the C driver logs a
    /// failed one at debug level and waits for an answer that cannot come.
    fn vital_command(
        &mut self,
        link: &mut Link,
        cmd: u16,
        name: &str,
    ) -> std::result::Result<(), Over> {
        match link.transport.bulk_out(EP_OUT, &self.packet.bytes(cmd), TIMEOUT) {
            Ok(_) => Ok(()),
            Err(Error::Disconnected) => Err(gone()),
            Err(e) => {
                log::error!("The {name} command failed: {}.", crate::usb::libusb_error_name(&e));
                Err(Over)
            }
        }
    }

    /// A status command, then two reads per answer, of which the first to
    /// show the magic is the one looked at, until the status is complete.
    /// A failed status command or read goes on as in the C driver. A read
    /// that takes nothing leaves the buffer as it was, so the answer before
    /// it counts.
    fn wait_for_capture(
        &mut self,
        link: &mut Link,
        stop: &AtomicBool,
    ) -> std::result::Result<(), Over> {
        loop {
            checked(stop)?;
            log::debug!("New command: status.");
            match link.transport.bulk_out(EP_OUT, &self.packet.bytes(CMD_STATUS), TIMEOUT) {
                Ok(_) => self.failed_commands = 0,
                Err(Error::Disconnected) => return Err(gone()),
                Err(Error::Timeout) => log::debug!("The status command timed out."),
                Err(e) => {
                    log::debug!("The status command failed: {e}.");
                    self.failed_commands += 1;
                    if self.failed_commands >= MAX_FAILURES {
                        return Err(gave_up());
                    }
                }
            }
            let mut answered = false;
            for _ in 0..VERSION_READS {
                checked(stop)?;
                self.status_read(link)?;
                if word_at(&link.buf, 0) == STATUS_MAGIC {
                    answered = true;
                    break;
                }
            }
            if !answered {
                log::debug!("No status answer, asking again.");
                continue;
            }
            let complete = word_at(&link.buf, 2) == STATUS_COMPLETE;
            checked(stop)?;
            self.status_read(link)?;
            if complete {
                return Ok(());
            }
        }
    }

    /// A read while the status is polled; a timeout is only a wait.
    fn status_read(&mut self, link: &mut Link) -> std::result::Result<(), Over> {
        match link.poll() {
            Ok(_) => self.failed_reads = 0,
            Err(Error::Disconnected) => return Err(gone()),
            Err(Error::Timeout) => log::debug!("A status read timed out."),
            Err(e) => {
                log::debug!("A status read failed: {e}.");
                self.failed_reads += 1;
                if self.failed_reads >= MAX_FAILURES {
                    return Err(gave_up());
                }
            }
        }
        Ok(())
    }

    /// The reads up to the start word; the length of the one that has it.
    /// A packet without it is dropped, however many come (they are the
    /// board talking, and a stop ends this).
    fn stream_start(
        &mut self,
        link: &mut Link,
        stop: &AtomicBool,
    ) -> std::result::Result<usize, Over> {
        loop {
            checked(stop)?;
            match link.poll() {
                Ok(n) if n >= 4 && word_at(&link.buf, 0) == START_MAGIC => {
                    self.failed_reads = 0;
                    return Ok(n);
                }
                Ok(_) => {
                    self.failed_reads = 0;
                    log::debug!("Mismatch magic number of start poll.");
                }
                Err(Error::Disconnected) => return Err(gone()),
                Err(e) => {
                    log::debug!("A read of the stream's start failed: {e}.");
                    self.failed_reads += 1;
                    if self.failed_reads >= MAX_FAILURES {
                        return Err(gave_up());
                    }
                }
            }
        }
    }

    /// `h4032l_start_data_transfers()` and the data transfers' callback.
    fn stream_data(
        &mut self,
        link: &mut Link,
        sink: &PacketSink,
        stop: &AtomicBool,
    ) -> std::result::Result<(), Over> {
        let depth = protocol::data_transfers(self.remaining, self.fpga_version);
        let stream = link.transport.bulk_in_stream_timed(EP_IN, DATA_TRANSFER, depth, TIMEOUT);
        let mut stream: Box<dyn BulkStream> = stream.map_err(|e| {
            log::error!("Failed to submit transfer: {}.", crate::usb::libusb_error_name(&e));
            log::error!("Can not start data transfers: -1");
            Over
        })?;
        while self.remaining > 0 {
            checked(stop)?;
            let data = match stream.next(TIMEOUT) {
                Ok(data) if !data.is_empty() => data,
                Err(Error::Disconnected) => {
                    log::error!("Data transfer failed: LIBUSB_ERROR_NO_DEVICE.");
                    return Err(Over);
                }
                other => {
                    if let Err(e) = other {
                        log::debug!("A data transfer failed: {e}.");
                    }
                    self.failed_reads += 1;
                    if self.failed_reads >= MAX_FAILURES {
                        return Err(gave_up());
                    }
                    continue;
                }
            };
            self.failed_reads = 0;
            let count = (data.len() / 4).min(self.remaining as usize);
            self.send_samples(&data[..count * 4], sink)?;
            if self.remaining == 0 {
                check_end_word(&data, count);
            }
        }
        Ok(())
    }

    /// `send_data()` for samples of 4 bytes each: the trigger marker at
    /// `trigger_pos`, also without a trigger, as the C driver does.
    fn send_samples(&mut self, samples: &[u8], sink: &PacketSink) -> std::result::Result<(), Over> {
        let count = u32::try_from(samples.len() / 4).expect("one transfer");
        self.remaining -= count;
        let delivered = if self.trigger_pos >= self.sent && self.trigger_pos - self.sent < count {
            let at = (self.trigger_pos - self.sent) as usize * 4;
            (at == 0 || sink.send(Packet::logic(4, samples[..at].to_vec())))
                && sink.send(Packet::Trigger)
                && sink.send(Packet::logic(4, samples[at..].to_vec()))
        } else {
            count == 0 || sink.send(Packet::logic(4, samples.to_vec()))
        };
        self.sent += count;
        if delivered { Ok(()) } else { Err(Over) }
    }
}

/// The stop flag as the end of a capture.
fn checked(stop: &AtomicBool) -> std::result::Result<(), Over> {
    if stop.load(Ordering::SeqCst) { Err(Over) } else { Ok(()) }
}

/// A device gone: in the C driver the next transfer does not go in.
fn gone() -> Over {
    log::error!("Failed to submit transfer: LIBUSB_ERROR_NO_DEVICE.");
    Over
}

/// [`MAX_FAILURES`] reached.
fn gave_up() -> Over {
    log::error!("Too many failed transfers, ending the capture.");
    Over
}

/// The end word follows the last sample. Protocol.c:194 reads it past
/// what came when the last transfer is full of samples; a word that did
/// not come is not looked at.
fn check_end_word(data: &[u8], samples: usize) {
    if data.len() < (samples + 1) * 4 {
        log::debug!("The last transfer ends without the end word.");
    } else if word_at(data, samples) != END_MAGIC {
        log::error!("Mismatch magic number of end poll.");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;
    use std::fmt::Write as _;
    use std::sync::mpsc::Receiver;

    /// Fixture text for bytes.
    trait HexString {
        fn hex_string(&self) -> String;
    }

    impl HexString for [u8] {
        fn hex_string(&self) -> String {
            self.iter().fold(String::new(), |mut s, b| {
                let _ = write!(s, "{b:02x}");
                s
            })
        }
    }

    fn packets(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
        rx.try_iter().map(|(_, p)| p).collect()
    }

    fn device() -> Hantek4032lDevice {
        Hantek4032lDevice::new(UsbDeviceInfo::default(), None)
    }

    #[test]
    fn channels_alternate_between_the_groups() {
        let d = device();
        let names: Vec<&str> = d.channels.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names[..4], ["A0", "B0", "A1", "B1"]);
        assert_eq!(names[31], "B15");
        assert_eq!(d.groups[0].channels[1], 2);
        assert_eq!(d.groups[1].channels[15], 31);
    }

    #[test]
    fn the_external_clock_needs_a_known_fpga_version() {
        let mut d = device();
        assert_eq!(d.options(None).unwrap().len(), 5);
        d.fpga_version = 0x4303;
        assert_eq!(d.options(None).unwrap().len(), 8);
        assert_eq!(d.options(Some("A")).unwrap().len(), 1);
        assert!(d.options(Some("C")).is_err());
    }

    #[test]
    fn settings_read_back_and_refusals_change_nothing() {
        let mut d = device();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        d.set(&Key::Samplerate, &Value::U64(625_000), None).unwrap();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(625_000));
        assert!(d.set(&Key::Samplerate, &Value::U64(3_000_000), None).is_err());
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(625_000));

        d.set(&Key::LimitSamples, &Value::U64(5000), None).unwrap();
        assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(5120));
        assert!(d.set(&Key::LimitSamples, &Value::U64((1 << 32) + 2048), None).is_err());
        assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(5120));

        d.set(&Key::VoltageThreshold, &Value::RangeF64(-6.0, 6.0), Some("B")).unwrap();
        assert_eq!(d.get(&Key::VoltageThreshold, Some("B")).unwrap(), Value::RangeF64(0.0, 0.0));
        assert!(d.get(&Key::VoltageThreshold, None).is_err());

        assert!(d.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
        d.set(&Key::CaptureRatio, &Value::U64(100), None).unwrap();

        d.set(&Key::ClockEdge, &Value::Str("both".into()), None).unwrap();
        assert!(d.set(&Key::ClockEdge, &Value::Str("up".into()), None).is_err());
        assert_eq!(d.get(&Key::ClockEdge, None).unwrap(), Value::Str("both".into()));
    }

    #[test]
    fn the_capture_packet_takes_the_settings() {
        let mut d = device();
        d.set(&Key::LimitSamples, &Value::U64(2048), None).unwrap();
        d.set(&Key::CaptureRatio, &Value::U64(100), None).unwrap();
        let p = d.capture_packet().unwrap();
        // A ratio of 100 makes the count equal to the samples, as in C.
        assert_eq!((p.samples, p.pre), (2048, 2048));
        assert_eq!(p.rate, 0);
        assert_eq!(p.pwm, [1174, 1174]);
        d.set(&Key::ExternalClock, &Value::Bool(true), None).unwrap();
        d.set(&Key::ExternalClockSource, &Value::Str("BCLK".into()), None).unwrap();
        d.set(&Key::ClockEdge, &Value::Str("falling".into()), None).unwrap();
        assert_eq!(d.capture_packet().unwrap().rate, 0x29);
    }

    /// `n` samples counting from `from`, as bytes.
    fn samples(from: u32, n: u32) -> Vec<u8> {
        (from..from + n).flat_map(u32::to_le_bytes).collect()
    }

    #[test]
    fn the_trigger_marker_falls_at_its_sample_and_once() {
        let (tx, rx) = std::sync::mpsc::sync_channel(16);
        let sink = PacketSink::new(tx, 0);
        let p = CmdPacket { samples: 8, pre: 3, ..CmdPacket::default() };
        let mut a = Acquisition::new(p, 1);
        assert!(a.send_samples(&samples(0, 2), &sink).is_ok());
        assert!(a.send_samples(&samples(2, 4), &sink).is_ok());
        assert!(a.send_samples(&samples(6, 2), &sink).is_ok());
        assert_eq!(a.remaining, 0);
        assert_eq!(
            packets(&rx),
            [
                Packet::logic(4, samples(0, 2)),
                Packet::logic(4, samples(2, 1)),
                Packet::Trigger,
                Packet::logic(4, samples(3, 3)),
                Packet::logic(4, samples(6, 2)),
            ]
        );
    }

    #[test]
    fn a_marker_at_the_first_sample_has_nothing_before_it_and_past_the_end_none() {
        let (tx, rx) = std::sync::mpsc::sync_channel(16);
        let sink = PacketSink::new(tx, 0);
        let mut a = Acquisition::new(CmdPacket { samples: 2, pre: 0, ..CmdPacket::default() }, 1);
        assert!(a.send_samples(&samples(0, 2), &sink).is_ok());
        assert_eq!(packets(&rx), [Packet::Trigger, Packet::logic(4, samples(0, 2))]);
        let mut a = Acquisition::new(CmdPacket { samples: 2, pre: 2, ..CmdPacket::default() }, 1);
        assert!(a.send_samples(&samples(0, 2), &sink).is_ok());
        assert_eq!(packets(&rx), [Packet::logic(4, samples(0, 2))]);
    }

    /// A status packet with status `s`, in hex.
    fn status(s: u8) -> String {
        format!("7f031a2b00000000{s:02x}0000000000000003430000")
    }

    /// The fixture of a capture of 2048 samples up to the status polls;
    /// `steps` holds what follows the configure.
    fn fixture(steps: &str) -> String {
        let p = CmdPacket { pwm: [1174, 1174], samples: 2048, pre: 102, rate: 0, trigger: None };
        let cfg: String = p.bytes(CMD_CONFIGURE).hex_string();
        format!(
            "# oxrok usb fixture 1\ndevice vid=04b5 pid=4032\nclaim 0 -> ok\n\
             bulk_out 02 {open} -> 84\nbulk_in 86 512 -> {answer}\n\
             ctl_out 40 b3 0000 0000 0f030303000000000000 -> ok\n\
             bulk_out 02 {cfg} -> 84\n{steps}",
            open =
                CmdPacket { samples: 16384, ..CmdPacket::default() }.bytes(CMD_STATUS).hex_string(),
            answer = status(0),
        )
    }

    /// Runs a capture on `text`, returning what the session got. The
    /// fixture must have been played to its end.
    fn capture(text: &str) -> Vec<Packet> {
        let replay = Replay::from_text(text).unwrap();
        let check = replay.clone();
        let mut d = Hantek4032lDevice::from_transport(Box::new(replay)).unwrap();
        d.open().unwrap();
        assert_eq!(d.fpga_version, 0x4303);
        d.set(&Key::LimitSamples, &Value::U64(2048), None).unwrap();
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let mut got = Vec::new();
        loop {
            let (_, p) = rx.recv().unwrap();
            let end = p == Packet::End;
            got.push(p);
            if end {
                break;
            }
        }
        d.close().unwrap();
        check.finish().unwrap();
        got
    }

    fn hex(p: &CmdPacket, cmd: u16) -> String {
        p.bytes(cmd).hex_string()
    }

    fn capture_packet_2048() -> CmdPacket {
        CmdPacket { pwm: [1174, 1174], samples: 2048, pre: 102, ..CmdPacket::default() }
    }

    #[test]
    fn a_refused_configure_ends_the_capture_without_a_header() {
        let text = fixture("").replace(" -> 84\n\n", " -> 84\n");
        // The configure line of the fixture answers a STALL.
        let last = text.rfind("-> 84").unwrap();
        let text = format!("{}-> err device \"endpoint stalled\"\n", &text[..last]);
        assert_eq!(capture(&text), [Packet::End]);
    }

    #[test]
    fn failed_status_reads_are_counted_and_end_the_capture() {
        let st = hex(&capture_packet_2048(), CMD_STATUS);
        let mut steps = String::new();
        let mut reads = 0;
        while reads < MAX_FAILURES {
            let _ = writeln!(steps, "bulk_out 02 {st} -> 84");
            for _ in 0..2 {
                if reads < MAX_FAILURES {
                    steps.push_str("bulk_in 86 512 -> err device \"endpoint stalled\"\n");
                    reads += 1;
                }
            }
        }
        assert_eq!(capture(&fixture(&steps)), [Packet::End]);
    }

    #[test]
    fn a_status_read_that_fails_is_judged_on_the_answer_before_it() {
        let p = capture_packet_2048();
        let (st, get) = (hex(&p, CMD_STATUS), hex(&p, CMD_GET));
        let mut first = "7f021a2b".to_owned();
        first.push_str(&samples(0, 127).hex_string());
        let data = |from, n| -> String { samples(from, n).hex_string() };
        let steps = format!(
            "bulk_out 02 {st} -> 84\n\
             bulk_in 86 512 -> err device \"endpoint stalled\"\n\
             bulk_in 86 512 -> {answer}\n\
             bulk_out 02 {st} -> 84\n\
             bulk_in 86 512 -> {done}\n\
             bulk_in 86 512 -> {done}\n\
             bulk_out 02 {get} -> 84\n\
             bulk_in 86 512 -> 00000000\n\
             bulk_in 86 512 -> {first}\n\
             bulk_in 86 2048 -> {a}\n\
             bulk_in 86 2048 -> {b}\n\
             bulk_in 86 2048 -> {c}\n\
             bulk_in 86 2048 -> {d}7f033c4d\n",
            answer = status(1),
            done = status(2),
            a = data(127, 512),
            b = data(639, 512),
            c = data(1151, 512),
            d = data(1663, 385),
        );
        let got = capture(&fixture(&steps));
        assert!(matches!(got[0], Packet::Header { .. }));
        assert_eq!(got.iter().filter(|p| **p == Packet::Trigger).count(), 1);
        assert_eq!(got.last(), Some(&Packet::End));
        let bytes: Vec<u8> = got
            .iter()
            .filter_map(|p| match p {
                Packet::Logic(l) => Some(l.data.clone()),
                _ => None,
            })
            .flatten()
            .collect();
        assert_eq!(bytes, samples(0, 2048));
    }

    /// A replay that sets the stop flag after a number of reads, as a
    /// stop arrives while the status is polled.
    struct StopAfter {
        replay: Replay,
        reads: usize,
        stop: Arc<AtomicBool>,
    }

    impl Transport for StopAfter {
        fn info(&self) -> &UsbDeviceInfo {
            self.replay.info()
        }
        fn set_configuration(&mut self, _: u8) -> Result<()> {
            unreachable!()
        }
        fn claim_interface(&mut self, _: u8) -> Result<()> {
            unreachable!()
        }
        fn release_interface(&mut self, _: u8) -> Result<()> {
            unreachable!()
        }
        fn reset(&mut self) -> Result<()> {
            unreachable!()
        }
        fn max_packet_size(&mut self, _: u8) -> Result<usize> {
            unreachable!()
        }
        fn interfaces(&mut self) -> Result<Vec<crate::usb::InterfaceDesc>> {
            unreachable!()
        }
        fn control_out(&mut self, _: ControlRequest, _: &[u8], _: Duration) -> Result<()> {
            unreachable!()
        }
        fn control_in(&mut self, _: ControlRequest, _: usize, _: Duration) -> Result<Vec<u8>> {
            unreachable!()
        }
        fn bulk_out(&mut self, ep: u8, data: &[u8], t: Duration) -> Result<usize> {
            self.replay.bulk_out(ep, data, t)
        }
        fn bulk_in(&mut self, ep: u8, len: usize, t: Duration) -> Result<Vec<u8>> {
            self.reads -= 1;
            if self.reads == 0 {
                self.stop.store(true, Ordering::SeqCst);
            }
            self.replay.bulk_in(ep, len, t)
        }
        fn interrupt_out(&mut self, _: u8, _: &[u8], _: Duration) -> Result<usize> {
            unreachable!()
        }
        fn interrupt_in(&mut self, _: u8, _: usize, _: Duration) -> Result<Vec<u8>> {
            unreachable!()
        }
        fn bulk_in_stream(&mut self, _: u8, _: usize, _: usize) -> Result<Box<dyn BulkStream>> {
            unreachable!()
        }
    }

    const NO_STEPS: &str = "# oxrok usb fixture 1\ndevice vid=04b5 pid=4032\n";

    #[test]
    fn status_answers_of_1_wait_until_a_stop() {
        let st = hex(&capture_packet_2048(), CMD_STATUS);
        let mut text = NO_STEPS.to_owned();
        for _ in 0..3 {
            let answer = status(1);
            let _ = write!(
                text,
                "bulk_out 02 {st} -> 84\nbulk_in 86 512 -> {answer}\nbulk_in 86 512 -> {answer}\n"
            );
        }
        let replay = Replay::from_text(&text).unwrap();
        let check = replay.clone();
        let stop = Arc::new(AtomicBool::new(false));
        let stopper = StopAfter { replay, reads: 6, stop: Arc::clone(&stop) };
        let mut link = Link { transport: Box::new(stopper), buf: [0; POLL_SIZE] };
        let mut a = Acquisition::new(capture_packet_2048(), 1);
        assert!(a.wait_for_capture(&mut link, &stop).is_err());
        // All three rounds were asked and no get, which is not in the fixture.
        check.finish().unwrap();
    }

    #[test]
    fn a_refused_status_command_and_silent_reads_reach_the_limit() {
        // The buffer holds no magic, as after an earlier capture's stream,
        // so the reads never end by themselves.
        let st = hex(&capture_packet_2048(), CMD_STATUS);
        let refused = format!("bulk_out 02 {st} -> err device \"endpoint stalled\"\n");
        let silent = "bulk_in 86 512 -> err timeout\n".repeat(VERSION_READS);
        let mut text = NO_STEPS.to_owned();
        for _ in 1..MAX_FAILURES {
            text.push_str(&refused);
            text.push_str(&silent);
        }
        text.push_str(&refused);
        let replay = Replay::from_text(&text).unwrap();
        let check = replay.clone();
        let mut link = Link { transport: Box::new(replay), buf: [0; POLL_SIZE] };
        let mut a = Acquisition::new(capture_packet_2048(), 1);
        assert!(a.wait_for_capture(&mut link, &AtomicBool::new(false)).is_err());
        check.finish().unwrap();
    }

    /// Status 2 at once, the get, and a first read of 127 samples.
    fn up_to_the_data() -> String {
        let p = capture_packet_2048();
        let (st, get) = (hex(&p, CMD_STATUS), hex(&p, CMD_GET));
        format!(
            "bulk_out 02 {st} -> 84\n\
             bulk_in 86 512 -> {done}\n\
             bulk_in 86 512 -> {done}\n\
             bulk_out 02 {get} -> 84\n\
             bulk_in 86 512 -> 7f021a2b{first}\n",
            done = status(2),
            first = samples(0, 127).hex_string(),
        )
    }

    #[test]
    fn a_stream_that_stays_silent_ends_the_capture_with_what_came() {
        let steps = up_to_the_data() + &"bulk_in 86 2048 -> err timeout\n".repeat(MAX_FAILURES);
        let got = capture(&fixture(&steps));
        assert!(matches!(got[0], Packet::Header { .. }));
        assert_eq!(got[1], Packet::logic(4, samples(0, 102)));
        assert_eq!(got[2], Packet::Trigger);
        assert_eq!(got[3], Packet::logic(4, samples(102, 25)));
        assert_eq!(got[4..], [Packet::End]);
    }

    #[test]
    fn a_device_gone_in_the_data_ends_the_capture() {
        let steps = up_to_the_data() + "bulk_in 86 2048 -> err disconnected\n";
        let got = capture(&fixture(&steps));
        assert_eq!(got.last(), Some(&Packet::End));
        assert_eq!(got.len(), 5);
    }

    #[test]
    fn a_stop_ends_the_capture_before_the_next_transfer() {
        let replay =
            Replay::from_text("# oxrok usb fixture 1\ndevice vid=04b5 pid=4032\n").unwrap();
        let check = replay.clone();
        let mut link = Link { transport: Box::new(replay), buf: [0; POLL_SIZE] };
        let (tx, rx) = std::sync::mpsc::sync_channel(4);
        let stop = AtomicBool::new(true);
        let mut a = Acquisition::new(capture_packet_2048(), 1);
        assert!(a.wait_for_capture(&mut link, &stop).is_err());
        assert!(a.stream_start(&mut link, &stop).is_err());
        drop(tx);
        check.finish().unwrap();
        assert_eq!(packets(&rx), Vec::<Packet>::new());
    }

    #[test]
    fn the_end_word_is_looked_for_in_what_came() {
        // Neither call may panic: a full last transfer holds no end word.
        check_end_word(&samples(0, 512), 512);
        check_end_word(&END_MAGIC.to_le_bytes(), 0);
    }
}
