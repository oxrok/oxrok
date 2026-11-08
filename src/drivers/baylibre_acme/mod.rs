// Counterpart in sigrok:
//   libsigrok/src/hardware/baylibre-acme/api.c,
//   libsigrok/src/hardware/baylibre-acme/protocol.c,
//   Copyright (C) 2015 Bartosz Golaszewski <bgolaszewski@baylibre.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Daniel Elstner,
//   Daniel Lezcano, Martin Ling, Soeren Apel, Aurelien Jacobs, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The `BayLibre` ACME cape (Another Cute Measurement Equipment), on a
//! `BeagleBone` Black: up to eight probes, ina226 power monitors and tmp435
//! thermometers, which the kernel's ina2xx and tmp401 drivers make hwmon
//! devices. Linux only, and nothing but files: the driver reads and writes what
//! the kernel puts in sysfs ([`protocol`]).
//!
//! A scan looks at each connector's chips and makes a group `Probe_N` of each
//! probe that answered: `P<N>_ENRG_PWR`, `P<N>_ENRG_CURR`, `P<N>_ENRG_VOL` (W, A,
//! V) for a power probe, `P<N>_TEMP_IN`, `P<N>_TEMP_OUT` (°C) for a temperature
//! probe. A power probe's group has `probe_factor`, its shunt in mΩ; a probe with
//! a power switch has `power_off`. A capture reads every enabled channel at each
//! tick of `samplerate` (1 to 500 Hz, 10 by default), the first tick one period
//! after the start, until `limit_samples` or `limit_time`.
//!
//! Where it goes its own way (`docs/DEVICES.md` has the rest):
//!
//! - A value file is read as the kernel writes it, a number and a newline. One
//!   that can't be read, reads nothing or holds something else is said, and its
//!   channel is read no more in that capture, as C does on a failed read, where C
//!   makes 0 of junk and a value of a stack buffer it never filled of an empty
//!   read; nor is C's -1 sent for the failed read. A capture with no channel left
//!   to read fails.
//! - A sample rate of 1 Hz captures, where C's timer refuses a period of 10^9 ns;
//!   0 is refused, where C divides by it.
//! - When the capture falls behind, C sends the last values again for each tick
//!   missed and counts one sample: so does this, but each frame counts toward
//!   `limit_samples`, which is never passed.
//! - GPIOs are exported first when they need it (C never manages to export one),
//!   and a write that fails says so (C checks none of them).

#![forbid(unsafe_code)]

pub mod gpio;
pub mod protocol;

#[cfg(test)]
mod tests;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, MqFlags, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use protocol::{
    ChannelType, Limits, MAX_PROBES, MAX_SAMPLE_RATE, Probe, ProbeType, Sysfs,
    TEMP_PRB_START_INDEX, Timer, ValueFile,
};

const CLASSES: [DeviceClass; 2] = [DeviceClass::Thermometer, DeviceClass::PowerMeter];
static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
];
/// The sample rate at a scan, Hz.
const DEFAULT_SAMPLERATE: u64 = 10;
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// The longest sleep between looks at the stop flag.
const STOP_CHECK: Duration = Duration::from_millis(20);

/// The `baylibre-acme` driver.
#[derive(Default)]
pub struct BaylibreAcmeDriver {
    sysfs: Sysfs,
}

impl BaylibreAcmeDriver {
    /// A driver whose `/sys` is under `root`, for tests.
    pub fn with_root(root: impl Into<std::path::PathBuf>) -> Self {
        BaylibreAcmeDriver { sysfs: Sysfs::new(root) }
    }
}

impl Driver for BaylibreAcmeDriver {
    fn id(&self) -> &'static str {
        "baylibre-acme"
    }

    fn long_name(&self) -> &'static str {
        "BayLibre ACME (Another Cute Measurement Equipment)"
    }

    fn scan_options(&self) -> &'static [Key] {
        &[]
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    /// `scan`: each connector's power probe, and on connectors 5 to 8 a
    /// temperature probe where there is no power probe.
    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        if let Some((key, _)) = options.first() {
            return Err(Error::UnknownKey(key.id().to_owned()));
        }
        let sysfs = &self.sysfs;
        if !sysfs.is_sane() {
            return Ok(Vec::new());
        }
        let mut probes = Vec::new();
        for i in 0..MAX_PROBES {
            let num = protocol::probe_num(i);
            let enrg = protocol::enrg_addr(i);
            if sysfs.detect_probe(enrg, num, protocol::ENRG_PROBE_NAME) {
                match sysfs.register_probe(ProbeType::Energy, enrg, num) {
                    Some(p) => probes.push(p),
                    None => log::error!("Error registering power probe {num}"),
                }
            } else if i >= TEMP_PRB_START_INDEX {
                let temp = protocol::temp_addr(i);
                if sysfs.detect_probe(temp, num, protocol::TEMP_PROBE_NAME) {
                    match sysfs.register_probe(ProbeType::Temperature, temp, num) {
                        Some(p) => probes.push(p),
                        None => log::error!("Error registering temp probe {num}"),
                    }
                }
            }
        }
        // No probe, no ACME.
        if probes.is_empty() {
            return Ok(Vec::new());
        }
        Ok(vec![Box::new(AcmeDevice::new(sysfs.clone(), probes))])
    }
}

/// A channel: what it measures and the probe it is on.
#[derive(Clone, Copy, Debug)]
struct ChannelInfo {
    kind: ChannelType,
    probe: usize,
}

/// The cape.
pub struct AcmeDevice {
    info: DeviceInfo,
    sysfs: Sysfs,
    probes: Vec<Probe>,
    channels: Vec<Channel>,
    infos: Vec<ChannelInfo>,
    groups: Vec<ChannelGroup>,
    samplerate: u64,
    limit_samples: u64,
    limit_msec: u64,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl AcmeDevice {
    fn new(sysfs: Sysfs, probes: Vec<Probe>) -> AcmeDevice {
        let mut channels = Vec::new();
        let mut infos = Vec::new();
        let mut groups = Vec::new();
        for (at, probe) in probes.iter().enumerate() {
            let mut members = Vec::new();
            for &kind in ChannelType::of(probe.kind) {
                members.push(channels.len());
                channels.push(Channel::new(
                    channels.len(),
                    ChannelKind::Analog,
                    kind.name(probe.num),
                ));
                infos.push(ChannelInfo { kind, probe: at });
            }
            groups.push(ChannelGroup::new(probe.name.clone(), members));
        }
        AcmeDevice {
            info: DeviceInfo {
                driver: "baylibre-acme",
                vendor: "BayLibre".to_owned(),
                model: "ACME".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            sysfs,
            probes,
            channels,
            infos,
            groups,
            samplerate: DEFAULT_SAMPLERATE,
            limit_samples: 0,
            limit_msec: 0,
            worker: None,
        }
    }

    /// The probe of a group; C finds the first of a name, and two can share one
    /// (a revision B probe at 0x44 and a temperature probe on connector 5).
    fn probe(&self, group: Option<&str>, key: &Key) -> Result<&Probe> {
        let Some(name) = group else {
            return Err(Error::arg(format!("{} needs a channel group.", key.id())));
        };
        self.probes
            .iter()
            .find(|p| p.name == name)
            .ok_or_else(|| Error::ChannelGroup(name.to_owned()))
    }

    fn end_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            if worker.handle.join().is_err() {
                log::error!("The capture thread died.");
            }
        }
    }
}

impl Device for AcmeDevice {
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
        let ch =
            self.channels.get_mut(pos).ok_or_else(|| Error::arg(format!("No channel {pos}.")))?;
        ch.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let ch =
            self.channels.get_mut(pos).ok_or_else(|| Error::arg(format!("No channel {pos}.")))?;
        name.clone_into(&mut ch.name);
        Ok(())
    }

    /// `config_list`'s `SR_CONF_DEVICE_OPTIONS`: a group's depend on its probe.
    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        let Some(name) = group else { return Ok(DEV_OPTIONS.to_vec()) };
        let probe = self
            .probes
            .iter()
            .find(|p| p.name == name)
            .ok_or_else(|| Error::ChannelGroup(name.to_owned()))?;
        let mut out = Vec::new();
        if probe.kind == ProbeType::Energy {
            out.push(OptionDesc::new(Key::ProbeFactor, Access::GET_SET));
        }
        if probe.has_pws {
            out.push(OptionDesc::new(Key::PowerOff, Access::GET_SET));
        }
        Ok(out)
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        match key {
            Key::LimitSamples => Ok(Value::U64(self.limit_samples)),
            Key::LimitMsec => Ok(Value::U64(self.limit_msec)),
            Key::Samplerate => Ok(Value::U64(self.samplerate)),
            Key::ProbeFactor => {
                let probe = self.probe(group, key)?;
                self.sysfs.get_shunt(probe).map(Value::U64)
            }
            Key::PowerOff => {
                let probe = self.probe(group, key)?;
                self.sysfs.read_power_state(probe).map(Value::Bool)
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let number =
            || value.as_u64().ok_or_else(|| Error::arg(format!("{} takes a number", key.id())));
        match key {
            Key::LimitSamples => self.limit_samples = number()?,
            Key::LimitMsec => self.limit_msec = number()?,
            Key::Samplerate => {
                let samplerate = number()?;
                if samplerate > MAX_SAMPLE_RATE {
                    log::error!("Maximum sample rate is {MAX_SAMPLE_RATE}");
                    return Err(protocol::sr_error("invalid samplerate"));
                }
                if samplerate == 0 {
                    return Err(Error::arg("Minimum sample rate is 1."));
                }
                self.samplerate = samplerate;
                self.sysfs.maybe_set_update_interval(&self.probes, samplerate);
            }
            Key::ProbeFactor => {
                let shunt = number()?;
                let probe = self.probe(group, key)?;
                self.sysfs.set_shunt(probe, shunt)?;
            }
            Key::PowerOff => {
                let off = value.as_bool().ok_or_else(|| Error::arg("power_off takes on or off"))?;
                let probe = self.probe(group, key)?;
                self.sysfs.set_power_off(probe, off)?;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        match (key, group) {
            (Key::Samplerate, None) => {
                Ok(ValueList::RangeStepU64 { min: 1, max: MAX_SAMPLE_RATE, step: 1 })
            }
            _ => Err(Error::UnknownKey(key.id().to_owned())),
        }
    }

    /// `std_dummy_dev_open`.
    fn open(&mut self) -> Result<()> {
        Ok(())
    }

    /// `std_dummy_dev_close`; a capture still running ends.
    fn close(&mut self) -> Result<()> {
        self.end_worker();
        Ok(())
    }

    /// `dev_acquisition_start`: every channel's file is opened, enabled or not,
    /// as in C, and one that won't open is no capture.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        self.end_worker();
        let mut files = Vec::with_capacity(self.channels.len());
        for (ch, info) in self.channels.iter().zip(&self.infos) {
            match self.sysfs.open_channel(self.probes[info.probe].hwmon, info.kind) {
                Ok(file) => files.push(file),
                Err(e) => {
                    log::error!("Error opening channel {}", ch.name);
                    return Err(Error::device(format!(
                        "Error opening channel {}: {}",
                        ch.name,
                        protocol::strerror(&e)
                    )));
                }
            }
        }
        // Every file stays open to the end, and they close in channel order, as
        // in C; the disabled ones are not read.
        let reads = self
            .channels
            .iter()
            .zip(&self.infos)
            .zip(files)
            .map(|((ch, info), file)| Read {
                index: ch.index,
                name: ch.name.clone(),
                hwmon: self.probes[info.probe].hwmon,
                kind: info.kind,
                file,
                value: 0.0,
                readable: ch.enabled,
            })
            .collect();
        let capture = Capture {
            reads,
            samplerate: self.samplerate,
            limits: Limits { samples: self.limit_samples, msec: self.limit_msec },
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("baylibre-acme acquisition".to_owned())
            .spawn(move || {
                let outcome = capture.run(&sink, &stop2);
                send_end(&sink, &stop2, outcome);
            })?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Drop for AcmeDevice {
    fn drop(&mut self) {
        self.end_worker();
    }
}

/// `End`, waiting for room in the queue for as long as nobody asked to stop, and
/// after a stop only for [`END_AFTER_STOP`]: a close joins this thread.
fn send_end(sink: &PacketSink, stop: &AtomicBool, outcome: Result<()>) {
    let stopped_at = std::cell::Cell::new(None);
    sink.end_while(outcome, || {
        if !stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = stopped_at.get().unwrap_or_else(Instant::now);
        stopped_at.set(Some(at));
        at.elapsed() < END_AFTER_STOP
    });
}

/// An enabled channel in a capture.
struct Read {
    index: usize,
    name: String,
    hwmon: i64,
    kind: ChannelType,
    file: ValueFile,
    /// The last value read, which a tick missed sends again.
    value: f32,
    /// Whether it is read: an enabled channel whose read has not failed.
    readable: bool,
}

impl Read {
    fn packet(&self) -> Packet {
        Packet::Analog(AnalogData {
            channels: vec![self.index],
            samples: vec![self.value],
            mq: self.kind.mq(),
            flags: MqFlags::empty(),
            unit: self.kind.unit(),
            digits: self.kind.digits(),
        })
    }

    /// `read_sample`: a channel whose read fails is said, and read no more.
    fn sample(&mut self) {
        match self.file.read() {
            Ok(raw) => self.value = protocol::scale(raw, self.kind.digits()),
            Err(e) => {
                log::error!(
                    "Error reading from channel {} (hwmon: {}): {e}",
                    self.name,
                    self.hwmon
                );
                self.readable = false;
            }
        }
    }
}

/// One capture, on its own thread: `bl_acme_receive_data` on each tick.
struct Capture {
    reads: Vec<Read>,
    samplerate: u64,
    limits: Limits,
}

impl Capture {
    /// A frame of the channels still read; false when nobody takes it.
    fn frame(&self, sink: &PacketSink, go_on: &impl Fn() -> bool) -> bool {
        if !sink.send_while(Packet::FrameBegin, go_on) {
            return false;
        }
        for read in self.reads.iter().filter(|r| r.readable) {
            if !sink.send_while(read.packet(), go_on) {
                return false;
            }
        }
        sink.send_while(Packet::FrameEnd, go_on)
    }

    fn run(mut self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        let mut timer = Timer::new(Instant::now(), self.samplerate);
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let started = Instant::now();
        let any_enabled = self.reads.iter().any(|r| r.readable);
        let mut samples = 0u64;
        let mut missed = 0u64;
        let outcome = loop {
            if !go_on() {
                break Ok(());
            }
            let now = Instant::now();
            let due = timer.next();
            if now < due {
                std::thread::sleep((due - now).min(STOP_CHECK));
                continue;
            }
            let ticks = timer.take(now);
            // We were not able to process the previous expirations: overloaded.
            missed += ticks.saturating_sub(1);
            // The first frame reads; the ticks missed send the same values again,
            // at once.
            for read in self.reads.iter_mut().filter(|r| r.readable) {
                read.sample();
            }
            if any_enabled && self.reads.iter().all(|r| !r.readable) {
                break Err(Error::device("No channel can be read any more."));
            }
            let mut sent_all = true;
            for _ in 0..ticks {
                sent_all = self.frame(sink, &go_on);
                if !sent_all {
                    break;
                }
                samples += 1;
                if self.limits.met(samples, started.elapsed()) {
                    break;
                }
            }
            if !sent_all || self.limits.met(samples, started.elapsed()) {
                break Ok(());
            }
        };
        if missed > 0 {
            log::warn!("{missed} samples missed");
        }
        outcome
    }
}
