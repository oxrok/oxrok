// Counterpart in sigrok:
//   libsigrok/src/hardware/sysclk-sla5032/api.c,
//   libsigrok/src/hardware/sysclk-sla5032/protocol.c (the acquisition),
//   Copyright (C) 2014 Daniel Elstner <daniel.kitta@gmail.com>,
//   Copyright (C) 2019 Vitaliy Vorobyov,
//   with commits by Uwe Hermann, Vitaliy Vorobyov
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! sysclk-sla5032: the Sysclk SLA5032 (32 channels, 500 MHz), an FX2LP with
//! its firmware in EEPROM relaying commands to an FPGA that holds its
//! bitstream until the power goes, with RLE sample memory.
//!
//! The wire protocol is in [`protocol`]. The open, the start and the capture
//! follow `api.c` and `protocol.c` step by step, so that the device sees the
//! same transactions as under sigrok-cli (checked with the emulator,
//! `tools/emu/sysclk-sla5032.sh`). The capture runs on a thread: a status
//! poll every 100 ms, then the memory read back in chunks of records.
//!
//! The device's memory is all it can send, in units of 256 samples: a limit
//! of 512 gets 1024 samples, one of 5000 gets 4608. The excess is cut here, a
//! shortfall is what the device sent. The trigger is found on the host, in
//! the samples read back (the registers are written as well).
//!
//! Where this differs from the C (`docs/DEVICES.md`): a `conn=` that matches
//! nothing selects nothing; the limit is honoured for any client; records
//! are summed in 64 bits and expanded in blocks, whatever they claim; a chunk
//! that ends without a short packet is kept, not dropped at the timeout; a
//! failure that ends a capture (exit 0, as in the C, which has no way to tell
//! the session), a chunk cut short and a failed key step get a warning, not
//! only a debug line or nothing;
//! and a stop does not wait out the 500 s a reply is given.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, SoftTriggerLogic, Trigger, TriggerMatch, Value, ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, libusb_error_name, split_conn};
use protocol::{
    Decoder, Depths, Link, MAX_LIMIT_SAMPLES, MIN_LIMIT_SAMPLES, Output, RLE_SAMPLES_COUNT,
    SAMPLERATES, UNIT_SIZE, USB_CONFIG, USB_INTERFACE, USB_PID_SLA5032, USB_VID_SYSCLK,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// How often a capture's status is asked (`poll_interval_ms`).
const POLL_INTERVAL: Duration = Duration::from_millis(100);
/// How often a wait for the next poll looks at the stop flag.
const STOP_CHECK: Duration = Duration::from_millis(5);
/// `NUM_CHANNELS`.
const NUM_CHANNELS: usize = 32;
/// `capture_ratios` (`api.c`): the list, which `set` does not hold it to.
const CAPTURE_RATIOS: [u64; 8] = [0, 10, 20, 30, 50, 70, 90, 100];

/// The driver.
pub struct Sla5032Driver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::LimitSamples, Access::GET_SET_LIST),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::Rle, Access::GET),
];
static TRIGGER_MATCHES: [TriggerMatch; 4] =
    [TriggerMatch::Zero, TriggerMatch::One, TriggerMatch::Rising, TriggerMatch::Falling];
/// One stage of levels and edges; the stages are checked at the start
/// (`api.c:395-398`).
const TRIGGER_FEATURES: crate::core::TriggerFeatures = crate::core::TriggerFeatures {
    count: false,
    consecutive: false,
    duration: false,
    range: false,
    or: false,
    and: false,
    max_stages: 0,
};

impl Driver for Sla5032Driver {
    fn id(&self) -> &'static str {
        "sysclk-sla5032"
    }

    fn long_name(&self) -> &'static str {
        "Sysclk SLA5032"
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
                // SLA5032 (api.c:116, an empty list reads as no conn=); here
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
            if info.vendor_id != USB_VID_SYSCLK || info.product_id != USB_PID_SLA5032 {
                if conn.is_some() {
                    log::warn!(
                        "USB device {} ({:04x}:{:04x}) is not a Sysclk SLA5032.",
                        info.conn(),
                        info.vendor_id,
                        info.product_id
                    );
                }
                continue;
            }
            devices.push(Box::new(Sla5032Device::new(info, None)));
        }
        Ok(devices)
    }
}

/// `prepare_trigger_masks()`, for the channels enabled: values, edge mask
/// and mask.
fn trigger_masks(trigger: Option<&Trigger>, channels: &[Channel]) -> (u32, u32, u32) {
    let (mut values, mut edges, mut mask) = (0, 0, 0);
    let Some(stage) = trigger.and_then(|t| t.stages.first()) else {
        return (values, edges, mask);
    };
    for c in &stage.conditions {
        if !channels.iter().any(|ch| ch.index == c.channel && ch.enabled) {
            continue;
        }
        let bit = 1u32 << c.channel;
        mask |= bit;
        if matches!(c.matches, TriggerMatch::One | TriggerMatch::Rising) {
            values |= bit;
        }
        if matches!(c.matches, TriggerMatch::Rising | TriggerMatch::Falling) {
            edges |= bit;
        }
    }
    (values, edges, mask)
}

/// The soft trigger of a capture (`soft_trigger_logic_new()`), on the
/// channels enabled, with a ring of `pre` samples.
fn soft_trigger(trigger: &Trigger, channels: &[Channel], pre: u64) -> SoftTriggerLogic {
    let stages = trigger
        .stages
        .iter()
        .map(|s| {
            s.conditions
                .iter()
                .filter(|c| channels.iter().any(|ch| ch.index == c.channel && ch.enabled))
                .map(|c| (c.channel, c.matches))
                .collect()
        })
        .collect();
    // At most the limit's 64 Mi samples: the option bounds it.
    SoftTriggerLogic::new(stages, UNIT_SIZE, usize::try_from(pre).unwrap_or(usize::MAX))
}

/// One SLA5032.
pub struct Sla5032Device {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    samplerate: u64,
    limit_samples: u64,
    capture_ratio: u64,
    trigger: Option<Trigger>,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl Sla5032Device {
    fn new(usb: UsbDeviceInfo, transport: Option<Box<dyn Transport>>) -> Self {
        Sla5032Device {
            info: DeviceInfo {
                driver: "sysclk-sla5032",
                vendor: "Sysclk".into(),
                model: "SLA5032".into(),
                version: String::new(),
                serial: String::new(),
                // api.c never sets sdi->connection_id.
                conn: None,
            },
            usb,
            channels: (0..NUM_CHANNELS)
                .map(|i| Channel::new(i, ChannelKind::Logic, format!("CH{i}")))
                .collect(),
            samplerate: SAMPLERATES[0],
            limit_samples: MAX_LIMIT_SAMPLES,
            capture_ratio: 50,
            trigger: None,
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            stop: Arc::new(AtomicBool::new(false)),
            worker: None,
        }
    }

    /// A capture is running: its thread has not given the link back.
    fn busy(&self) -> bool {
        self.worker.as_ref().is_some_and(|w| !w.is_finished())
            && self.link.lock().expect("link lock").is_none()
    }

    /// A device on an already open transport (a replayed recording).
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        if usb.vendor_id != USB_VID_SYSCLK || usb.product_id != USB_PID_SLA5032 {
            return Err(Error::arg("not a Sysclk SLA5032"));
        }
        Ok(Self::new(usb, Some(transport)))
    }

    /// `config_commit()` and `sla5032_start_acquisition()` up to the source:
    /// what fails the start itself.
    fn begin(&self, link: &mut Link) -> Result<Acquisition> {
        let failed = || Error::arg("Failed to start session.");
        if self.trigger.as_ref().is_some_and(|t| t.stages.len() > 1) {
            log::error!("This device only supports 1 trigger stage.");
            return Err(failed());
        }
        let (values, edges, mask) = trigger_masks(self.trigger.as_ref(), &self.channels);
        link.apply_fpga_config().map_err(|e| {
            log::debug!("Commit: {e}.");
            log::error!("Failed to apply FPGA configuration.");
            failed()
        })?;
        let d = protocol::depths(self.limit_samples, self.capture_ratio);
        log::debug!(
            "start acquision, smp lim: {}, cap ratio: {}.",
            self.limit_samples,
            self.capture_ratio
        );
        log::debug!("start acquision, pre: {}, post: {}.", d.pre, d.post);
        log::debug!("start acquision, pre: {:x}, post: {:x}.", d.pre_reg, d.post_reg);
        self.write_setup(link, &d, values, edges, mask).map_err(|e| {
            log::debug!("Start: {e}.");
            failed()
        })?;
        let stl = self.trigger.as_ref().map(|t| soft_trigger(t, &self.channels, d.pre));
        Ok(Acquisition {
            decoder: Decoder::new(stl, self.limit_samples),
            stop: Arc::clone(&self.stop),
            sent: false,
        })
    }

    /// The registers of a start, in the C's order.
    fn write_setup(
        &self,
        link: &mut Link,
        d: &Depths,
        values: u32,
        edges: u32,
        mask: u32,
    ) -> Result<()> {
        link.set_depth(d)?;
        link.set_triggers(values, edges, mask)?;
        link.set_samplerate(self.samplerate)?;
        link.set_pwm()?;
        link.start_sample()
    }
}

impl Device for Sla5032Device {
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
            None => Ok(OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::Samplerate => Value::U64(self.samplerate),
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::CaptureRatio => Value::U64(self.capture_ratio),
            Key::Rle => Value::Bool(true),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::Samplerate => {
                let v = value.as_u64().ok_or_else(bad)?;
                if !SAMPLERATES.contains(&v) {
                    return Err(bad());
                }
                self.samplerate = v;
            }
            Key::LimitSamples => {
                let v = value.as_u64().ok_or_else(bad)?;
                if !(MIN_LIMIT_SAMPLES..=MAX_LIMIT_SAMPLES).contains(&v) {
                    return Err(bad());
                }
                self.limit_samples = v;
            }
            Key::CaptureRatio => {
                // Any ratio (api.c:332), up to what sr_config_set() lets by.
                let v = value.as_u64().ok_or_else(bad)?;
                if v > 100 {
                    return Err(bad());
                }
                self.capture_ratio = v;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::Samplerate => ValueList::U64(SAMPLERATES.to_vec()),
            Key::LimitSamples => ValueList::MinMaxU64(MIN_LIMIT_SAMPLES, MAX_LIMIT_SAMPLES),
            Key::CaptureRatio => ValueList::U64(CAPTURE_RATIOS.to_vec()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `dev_open()`: the configuration, the interface, the bitstream if the
    /// FPGA has none.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let mut t = match self.pending.take() {
            Some(t) => t,
            None => crate::usb::open(&self.usb)?,
        };
        t.force_configuration(USB_CONFIG).inspect_err(|e| {
            log::error!("Failed to set USB configuration: {}.", libusb_error_name(e));
        })?;
        t.claim_interface(USB_INTERFACE).inspect_err(|e| {
            log::error!("Failed to claim interface: {}.", libusb_error_name(e));
        })?;
        // A close sets it, and a reply must not then count as stopped.
        self.stop.store(false, Ordering::SeqCst);
        let mut link = Link::new(t, Arc::clone(&self.stop));
        link.apply_fpga_config()?;
        *self.link.lock().expect("link lock") = Some(link);
        Ok(())
    }

    /// `dev_close()`: the thread joined, the interface released.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(w) = self.worker.take() {
            let _ = w.join();
        }
        if let Some(mut link) = self.link.lock().expect("link lock").take()
            && let Err(e) = link.transport().release_interface(USB_INTERFACE)
        {
            log::debug!("Releasing the interface: {e}.");
        }
        Ok(())
    }

    fn trigger_features(&self) -> crate::core::TriggerFeatures {
        TRIGGER_FEATURES
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&TRIGGER_FEATURES)?;
            // api.c:414-420 refuses these at the commit, for enabled
            // channels; a match that cannot be set is refused whatever the
            // channel, since it may be enabled later. The CLI's parser goes
            // by TRIGGER_MATCHES and never sends one.
            let bad = t
                .stages
                .iter()
                .flat_map(|s| &s.conditions)
                .find(|c| !TRIGGER_MATCHES.contains(&c.matches));
            if let Some(c) = bad {
                let msg = format!("Unsupported trigger match for CH{}.", c.channel);
                log::error!("{msg}");
                return Err(Error::Arg(msg));
            }
        }
        self.trigger = trigger;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.busy() {
            return Err(Error::Busy);
        }
        // The capture before this one is over, as in the C, which returns to
        // idle. Its thread has at most its End left to send, so it is joined
        // only once finished: with End unread and the queue full, a join
        // here would wait for ever.
        if let Some(w) = self.worker.take()
            && w.is_finished()
        {
            let _ = w.join();
        }
        let link = self.link.lock().expect("link lock").take();
        let Some(mut link) = link else {
            return Err(Error::device("device is not open"));
        };
        self.stop.store(false, Ordering::SeqCst);
        match self.begin(&mut link) {
            Ok(acq) => {
                let slot = Arc::clone(&self.link);
                let handle = std::thread::Builder::new()
                    .name("sysclk-sla5032 acquisition".into())
                    .spawn(move || {
                        sink.send(Packet::header());
                        acq.run(&mut link, &sink);
                        // Back before End: a client may start again on it.
                        *slot.lock().expect("link lock") = Some(link);
                        sink.send(Packet::End);
                    })
                    .map_err(Error::Io)?;
                self.worker = Some(handle);
                Ok(())
            }
            Err(e) => {
                *self.link.lock().expect("link lock") = Some(link);
                Err(e)
            }
        }
    }

    /// Asks the thread to end, without waiting: it may still send packets,
    /// which the session drains. [`Device::close`] joins it. As in the C, a
    /// stop puts nothing on the wire while the capture waits
    /// (`dev_acquisition_stop()`).
    fn stop(&mut self) -> Result<()> {
        self.stop.store(true, Ordering::SeqCst);
        Ok(())
    }
}

/// One capture, on its own thread: `la_prepare_data()`.
struct Acquisition {
    decoder: Decoder,
    stop: Arc<AtomicBool>,
    /// Some sample has gone out.
    sent: bool,
}

impl Acquisition {
    fn run(mut self, link: &mut Link, sink: &PacketSink) {
        self.capture(link, sink);
    }

    fn stopped(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    /// Waits for the next poll: false when a stop came first.
    fn wait_for_poll(&self) -> bool {
        let start = Instant::now();
        while start.elapsed() < POLL_INTERVAL {
            if self.stopped() {
                return false;
            }
            std::thread::sleep(STOP_CHECK);
        }
        !self.stopped()
    }

    /// A failure that ends the capture: register 14 written, as on every
    /// way out of the C's callback. The C says so only in its debug log, and
    /// its exit status stays 0, as it does here: a driver has no way to
    /// tell the session. A wait for a reply that a stop ended is no failure,
    /// and a command now would come with that reply unread.
    fn failed(&self, link: &mut Link, e: &Error) {
        if self.stopped() && matches!(e, Error::Timeout) {
            return;
        }
        let _ = link.reg14_zero();
        log::warn!("The capture ended early: {e}.");
    }

    fn capture(&mut self, link: &mut Link, sink: &PacketSink) {
        let captured = loop {
            if !self.wait_for_poll() {
                return;
            }
            match link.poll() {
                Ok((captured, true)) => break captured,
                Ok(_) => {}
                Err(e) => return self.failed(link, &e),
            }
        };
        log::debug!("acquision done, status: {captured}.");
        if let Err(e) = link.set_read_back() {
            return self.failed(link, &e);
        }
        loop {
            let data = match link.read_chunk() {
                Ok(data) => data,
                Err(e) => {
                    self.failed(link, &e);
                    log::debug!("acquision done, ret: {}.", protocol::ret_code(&e));
                    return;
                }
            };
            log::debug!("acquision done, xfer_len: {}.", data.len());
            if data.is_empty() {
                self.no_samples(link);
                return;
            }
            log::debug!(
                "acquision done, rle_samples_count: {}.",
                data.len() / protocol::RLE_SAMPLE_SIZE
            );
            let (records, samples) = protocol::count_records(&data);
            log::debug!("acquision done, samples_count: {samples}.");
            if samples == 0 {
                log::debug!("acquision done, no samples.");
                self.no_samples(link);
                return;
            }
            self.decode(&data, records, sink);
            // The marker ends a capture and a full chunk goes on. Anything
            // else was cut short, which the C takes for the end in silence.
            if records != RLE_SAMPLES_COUNT && records == data.len() / protocol::RLE_SAMPLE_SIZE {
                log::warn!(
                    "The capture ended early: a chunk of {} bytes has no end marker.",
                    data.len()
                );
            }
            if records != RLE_SAMPLES_COUNT || self.stopped() {
                break;
            }
        }
        log::debug!("acquision stop, rle_samples_count < RLE_SAMPLES_COUNT.");
        let _ = link.reg14_zero();
    }

    /// The read-back's end with no data in this chunk. After samples it is
    /// the normal end of a capture that filled its last chunk with no
    /// marker; before any, the user should know.
    fn no_samples(&self, link: &mut Link) {
        let _ = link.reg14_zero();
        if !self.sent {
            log::warn!("The capture ended early: the device sent no samples.");
        }
    }

    /// One chunk's samples to the session.
    fn decode(&mut self, data: &[u8], records: usize, sink: &PacketSink) {
        let stop = Arc::clone(&self.stop);
        let sent = &mut self.sent;
        let mut emit = |o: Output| {
            let packet = match o {
                Output::Samples(s) => {
                    *sent = true;
                    Packet::logic(UNIT_SIZE, s)
                }
                Output::Trigger => Packet::Trigger,
            };
            if !sink.send(packet) {
                stop.store(true, Ordering::SeqCst);
            }
        };
        let stopped = {
            let stop = Arc::clone(&self.stop);
            move || stop.load(Ordering::SeqCst)
        };
        self.decoder.records(data, records, &stopped, &mut emit);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::trigger::{TriggerCondition, TriggerStage};
    use crate::usb::record::Replay;

    fn replay(steps: &[String]) -> Replay {
        let mut text = String::from(
            "# oxrok usb fixture 1\ndevice vid=2961 pid=66b0 version=0000 interfaces=0:ff:00:00\n",
        );
        for s in steps {
            text.push_str(s);
            text.push('\n');
        }
        Replay::from_text(&text).unwrap()
    }

    fn hex(b: &[u8]) -> String {
        use std::fmt::Write as _;
        b.iter().fold(String::new(), |mut s, x| {
            let _ = write!(s, "{x:02x}");
            s
        })
    }

    fn sink() -> (PacketSink, std::sync::mpsc::Receiver<(usize, Packet)>) {
        let (tx, rx) = std::sync::mpsc::sync_channel(4096);
        (PacketSink::new(tx, 0), rx)
    }

    /// What came out of a capture: samples, whether a trigger came, whether
    /// the end did.
    fn collect(rx: &std::sync::mpsc::Receiver<(usize, Packet)>) -> (usize, bool, bool) {
        let (mut samples, mut trigger, mut end) = (0, false, false);
        while let Ok((_, p)) = rx.try_recv() {
            match p {
                Packet::Logic(l) => samples += l.len(),
                Packet::Trigger => trigger = true,
                Packet::End => end = true,
                _ => {}
            }
        }
        (samples, trigger, end)
    }

    fn acquisition(limit: u64, stop: bool) -> Acquisition {
        Acquisition {
            decoder: Decoder::new(None, limit),
            stop: Arc::new(AtomicBool::new(stop)),
            sent: false,
        }
    }

    fn link(r: &Replay, stop: &Arc<AtomicBool>) -> Link {
        Link::new(Box::new(r.clone()), Arc::clone(stop))
    }

    fn read_reg_steps(reg: u8, value: u32) -> Vec<String> {
        vec![
            format!("bulk_out 04 {} -> 3", hex(&protocol::read_reg_command(reg))),
            format!("bulk_in 88 4 -> {value:08x}"),
        ]
    }

    fn write_reg_step(reg: u8, value: u32) -> String {
        format!("bulk_out 04 {} -> 7", hex(&protocol::write_reg_command(reg, value)))
    }

    /// A status poll that says done.
    fn done_poll() -> Vec<String> {
        let mut s = read_reg_steps(1, 0);
        s.extend(read_reg_steps(0, 0));
        s.extend(read_reg_steps(2, 0xa5a5_a5ab));
        s
    }

    fn read_back() -> Vec<String> {
        vec![write_reg_step(5, 0x08), write_reg_step(5, 0x28)]
    }

    /// A chunk's request, then its reply.
    fn chunk_steps(reply: &str) -> Vec<String> {
        let mut s = vec!["bulk_out 04 0703 -> 2".to_owned(), write_reg_step(3, 0x30_0000)];
        s.extend(read_reg_steps(5, 0x28));
        s.push(write_reg_step(5, 0x28));
        s.extend(read_reg_steps(5, 0x28));
        s.push(write_reg_step(5, 0x38));
        s.push(format!("bulk_in 86 6291456 -> {reply}"));
        s
    }

    fn records_hex(n: usize, count: u16) -> String {
        let mut d = Vec::new();
        for i in 0..n {
            d.extend_from_slice(&(i as u32).to_le_bytes());
            d.extend_from_slice(&count.to_le_bytes());
        }
        hex(&d)
    }

    const MARKER: &str = "00000000ffff";

    fn capture_steps(reply: &str) -> Vec<String> {
        let mut s = done_poll();
        s.extend(read_back());
        s.extend(chunk_steps(reply));
        s
    }

    fn run_on(steps: &[String], limit: u64) -> (usize, bool, bool) {
        let r = replay(steps);
        let mut acq = acquisition(limit, false);
        let stop = Arc::clone(&acq.stop);
        let mut l = link(&r, &stop);
        let (s, rx) = sink();
        acq.capture(&mut l, &s);
        s.send(Packet::End);
        r.finish().unwrap();
        collect(&rx)
    }

    #[test]
    fn a_capture_reads_back_and_ends_with_register_14() {
        let reply = format!("{}{MARKER}", records_hex(3, 1));
        let mut steps = capture_steps(&reply);
        steps.push(write_reg_step(14, 0));
        assert_eq!(run_on(&steps, 100), (6, false, true));
    }

    #[test]
    fn the_limit_cuts_what_the_device_sends_and_keeps_the_transactions() {
        // The chunk is read to its marker whatever the limit.
        let reply = format!("{}{MARKER}", records_hex(3, 1));
        let mut steps = capture_steps(&reply);
        steps.push(write_reg_step(14, 0));
        assert_eq!(run_on(&steps, 4), (4, false, true));
    }

    #[test]
    fn a_chunk_that_timed_out_with_data_is_kept() {
        // bulk_in's answer on a timeout is a short read: 1536 bytes of
        // records, on a packet boundary, the marker last (protocol.c:749
        // drops it). It takes no more transfers than a short packet would.
        let reply = format!("{}{MARKER}", records_hex(255, 0));
        let mut steps = capture_steps(&reply);
        steps.push(write_reg_step(14, 0));
        assert_eq!(run_on(&steps, 1000), (255, false, true));
    }

    #[test]
    fn a_failed_chunk_ends_with_register_14() {
        let mut steps = capture_steps("err timeout");
        steps.push(write_reg_step(14, 0));
        assert_eq!(run_on(&steps, 100), (0, false, true));
    }

    #[test]
    fn a_failed_poll_ends_with_register_14() {
        let mut steps = vec!["bulk_out 04 030404 -> err device \"stalled\"".to_owned()];
        steps.push(write_reg_step(14, 0));
        assert_eq!(run_on(&steps, 100), (0, false, true));
    }

    #[test]
    fn an_empty_chunk_and_one_with_only_the_marker_end_it() {
        let mut steps = capture_steps("-");
        steps.push(write_reg_step(14, 0));
        assert_eq!(run_on(&steps, 100), (0, false, true));
        let mut steps = capture_steps(MARKER);
        steps.push(write_reg_step(14, 0));
        assert_eq!(run_on(&steps, 100), (0, false, true));
    }

    #[test]
    fn a_stop_while_waiting_puts_nothing_on_the_wire() {
        // A whole capture on offer, none of it taken: a call that differs
        // leaves a replay where it is, so finish() alone would not tell.
        let steps = capture_steps(MARKER);
        let r = replay(&steps);
        let mut acq = acquisition(100, true);
        let stop = Arc::clone(&acq.stop);
        let mut l = link(&r, &stop);
        let (s, rx) = sink();
        acq.capture(&mut l, &s);
        assert_eq!(r.remaining(), steps.len());
        assert_eq!(collect(&rx), (0, false, false));
    }

    /// A capture whose consumer has gone after the first of its full chunks,
    /// which is a stop; the steps as `steps` offers them. What is left.
    fn stopped_after_a_chunk(steps: &[String]) -> usize {
        let r = replay(steps);
        let stop = Arc::new(AtomicBool::new(false));
        let mut acq =
            Acquisition { decoder: Decoder::new(None, 8), stop: Arc::clone(&stop), sent: false };
        let mut l = link(&r, &stop);
        let (tx, rx) = std::sync::mpsc::sync_channel(4096);
        drop(rx);
        acq.capture(&mut l, &PacketSink::new(tx, 0));
        assert!(stop.load(Ordering::SeqCst));
        r.remaining()
    }

    #[test]
    fn a_stop_between_chunks_asks_for_no_more() {
        // The next chunk on offer, data and all, with register 14 only after
        // it: nothing of it is taken.
        let mut steps = capture_steps("00{6291456}");
        let more = chunk_steps(MARKER);
        let left = more.len() + 1;
        steps.extend(more);
        steps.push(write_reg_step(14, 0));
        assert_eq!(stopped_after_a_chunk(&steps), left);
    }

    #[test]
    fn a_stop_between_chunks_writes_register_14() {
        let mut steps = capture_steps("00{6291456}");
        steps.push(write_reg_step(14, 0));
        assert_eq!(stopped_after_a_chunk(&steps), 0);
    }

    #[test]
    fn an_open_takes_back_the_stop_of_a_close() {
        let mut d = Sla5032Device::from_transport(Box::new(replay(&open_steps()))).unwrap();
        d.stop().unwrap();
        d.open().unwrap();
        assert!(!d.stop.load(Ordering::SeqCst));
    }

    #[test]
    fn a_full_chunk_with_no_stop_asks_for_the_next() {
        let full = "00{6291456}";
        let mut steps = capture_steps(full);
        steps.extend(chunk_steps(MARKER));
        steps.push(write_reg_step(14, 0));
        assert_eq!(run_on(&steps, 8), (8, false, true));
    }

    fn device() -> Sla5032Device {
        let r = replay(&[]);
        Sla5032Device::from_transport(Box::new(r)).unwrap()
    }

    /// A start's steps for a limit of 1024 at 500 MHz, register 5 from `r5`
    /// (the C's order); the value register 5 holds after it.
    fn start_steps(r5: &mut u32) -> Vec<String> {
        let mut s: Vec<String> = [(7, 1), (6, 1), (0, 0), (1, 0), (2, 0), (4, 0)]
            .iter()
            .map(|&(reg, value)| write_reg_step(reg, value))
            .collect();
        let bit = |s: &mut Vec<String>, r5: &mut u32, bit: u32, set: bool| {
            s.extend(read_reg_steps(5, *r5));
            *r5 = if set { *r5 | 1 << bit } else { *r5 & !(1 << bit) };
            s.push(write_reg_step(5, *r5));
        };
        bit(&mut s, r5, 0, true);
        bit(&mut s, r5, 7, false);
        for (reg, value) in [(9, 20_000_000 - 1), (10, 200_000 - 1), (11, 14), (12, 4), (14, 1)] {
            s.push(write_reg_step(reg, value));
        }
        for (b, set) in [
            (2, false),
            (3, false),
            (5, false),
            (6, true),
            (1, true),
            (1, false),
            (8, true),
            (8, false),
            (6, true),
            (2, true),
        ] {
            bit(&mut s, r5, b, set);
        }
        s
    }

    fn open_steps() -> Vec<String> {
        let mut s = vec!["config 1 -> ok".to_owned(), "claim 0 -> ok".to_owned()];
        s.extend(read_reg_steps(2, 0xa5a5_a5a1));
        s
    }

    /// Reads a session's packets up to End: the samples.
    fn to_the_end(rx: &std::sync::mpsc::Receiver<(usize, Packet)>) -> usize {
        let mut samples = 0;
        loop {
            match rx.recv_timeout(Duration::from_secs(30)).expect("no End").1 {
                Packet::Logic(l) => samples += l.len(),
                Packet::End => return samples,
                _ => {}
            }
        }
    }

    #[test]
    fn a_second_capture_runs_on_the_open_device() {
        // Right after End, with no wait for the thread to finish.
        let reply = format!("{}{MARKER}", records_hex(2, 1));
        let mut steps = open_steps();
        let mut r5 = 0;
        for _ in 0..3 {
            steps.extend(start_steps(&mut r5));
            steps.extend(capture_steps(&reply));
            steps.push(write_reg_step(14, 0));
            r5 = 0x28;
        }
        let r = replay(&steps);
        let mut d = Sla5032Device::from_transport(Box::new(r.clone())).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1024), None).unwrap();
        d.open().unwrap();
        for _ in 0..3 {
            let (s, rx) = sink();
            d.start(s).unwrap();
            assert_eq!(to_the_end(&rx), 4);
        }
        d.close().unwrap();
        r.finish().unwrap();
    }

    #[test]
    fn a_start_during_a_capture_is_refused_and_a_stop_lets_the_next_one_in() {
        let mut steps = open_steps();
        let mut r5 = 0;
        steps.extend(start_steps(&mut r5));
        // Polls that say not done, for a machine slower than the first wait.
        for _ in 0..100 {
            steps.extend(read_reg_steps(1, 0));
            steps.extend(read_reg_steps(0, 0));
            steps.extend(read_reg_steps(2, 0xa5a5_a5a1));
        }
        let r = replay(&steps);
        let mut d = Sla5032Device::from_transport(Box::new(r)).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1024), None).unwrap();
        d.open().unwrap();
        let (s, rx) = sink();
        d.start(s).unwrap();
        let (s2, _rx2) = sink();
        assert!(matches!(d.start(s2), Err(Error::Busy)));
        d.stop().unwrap();
        to_the_end(&rx);
        // The link is back: this start gets as far as the wire, not Busy.
        let (s3, _rx3) = sink();
        assert!(!matches!(d.start(s3), Err(Error::Busy)));
        d.stop().unwrap();
        d.close().unwrap();
    }

    #[test]
    fn the_options_are_held_to_the_cs_bounds() {
        let mut d = device();
        assert_eq!(d.get(&Key::LimitSamples, None).unwrap().as_u64(), Some(MAX_LIMIT_SAMPLES));
        assert_eq!(d.get(&Key::Samplerate, None).unwrap().as_u64(), Some(500_000_000));
        assert_eq!(d.get(&Key::CaptureRatio, None).unwrap().as_u64(), Some(50));
        assert!(matches!(d.get(&Key::Rle, None).unwrap(), Value::Bool(true)));
        assert!(d.set(&Key::LimitSamples, &Value::U64(511), None).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::U64(MAX_LIMIT_SAMPLES + 1), None).is_err());
        d.set(&Key::LimitSamples, &Value::U64(512), None).unwrap();
        assert!(d.set(&Key::Samplerate, &Value::U64(3_000_000), None).is_err());
        d.set(&Key::Samplerate, &Value::U64(2000), None).unwrap();
        // A ratio the list does not hold is taken (api.c:332).
        d.set(&Key::CaptureRatio, &Value::U64(15), None).unwrap();
        assert!(d.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
        assert!(d.set(&Key::Rle, &Value::Bool(false), None).is_err());
        assert!(d.options(Some("x")).is_err());
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
    fn trigger_masks_take_the_channels_enabled() {
        let mut d = device();
        let t = trigger(&[
            (3, TriggerMatch::One),
            (4, TriggerMatch::Zero),
            (5, TriggerMatch::Rising),
            (6, TriggerMatch::Falling),
            (31, TriggerMatch::Rising),
        ]);
        assert_eq!(
            trigger_masks(Some(&t), &d.channels),
            (1 << 3 | 1 << 5 | 1 << 31, 1 << 5 | 1 << 6 | 1 << 31, 0b111_1000 | 1 << 31)
        );
        d.enable_channel(3, false).unwrap();
        assert_eq!(trigger_masks(Some(&t), &d.channels).2, 0b111_0000 | 1 << 31);
        assert_eq!(trigger_masks(None, &d.channels), (0, 0, 0));
    }

    #[test]
    fn an_either_edge_is_refused() {
        let mut d = device();
        assert!(d.set_trigger(Some(trigger(&[(3, TriggerMatch::Edge)]))).is_err());
        d.set_trigger(Some(trigger(&[(3, TriggerMatch::Rising)]))).unwrap();
    }

    #[test]
    fn two_stages_are_refused_at_the_start_with_nothing_on_the_wire() {
        let mut d = device();
        let r = replay(&[]);
        let mut l = link(&r, &d.stop);
        let mut t = trigger(&[(3, TriggerMatch::One)]);
        t.stages.push(t.stages[0].clone());
        d.set_trigger(Some(t)).unwrap();
        assert!(d.begin(&mut l).is_err());
        r.finish().unwrap();
    }

    #[test]
    fn a_start_writes_in_the_cs_order_and_a_failure_fails_it() {
        // Register 7 is the first write; its failure stops all the rest.
        let d = device();
        let r = replay(&[
            "bulk_out 04 030804 -> 3".to_owned(),
            "bulk_in 88 4 -> a5a5a5a1".to_owned(),
            format!(
                "bulk_out 04 {} -> err device \"stalled\"",
                hex(&protocol::write_reg_command(7, 0x1ffff))
            ),
        ]);
        let mut l = link(&r, &d.stop);
        assert!(d.begin(&mut l).is_err());
        r.finish().unwrap();
    }
}
