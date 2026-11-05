// Counterpart in sigrok:
//   libsigrok/src/hardware/manson-hcs-3xxx/api.c,
//   libsigrok/src/hardware/manson-hcs-3xxx/protocol.c,
//   Copyright (C) 2014 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2014 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Aurelien Jacobs,
//   Gerhard Sittig, Martin Ling, Michael Klengel, Soeren Apel, turboaffe,
//   Daniel Elstner, Jan Luebbe, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Manson HCS-3xxx switching power supplies, one output each, on the USB to
//! serial bridge built in.
//!
//! `conn=` names the port and `serialcomm=` overrides 9600/8n1. A scan asks
//! `GMOD`, `GETD` and `GMAX` and sets nothing. `--get` and `--show` answer what
//! is known and send nothing, so `voltage_target` and `current_limit` read 0
//! until they have been set: as in C, as no command of its reads them.
//!
//! A capture asks `GETD` again as soon as the last reply has come, or
//! 500 ms after an unanswered one, and a sample is a voltage and a current.
//! A setting during a capture waits out the reply on its way and drops it.
//!
//! No reply is no reading, where C waits for ever (a scan, a setting) or asks
//! for ever (a capture): a scan finds nothing and a setting fails. A capture
//! that only a sample limit could end ends after [`NO_READING_BOUND`] with no
//! reading. Under a time limit or with no limit it keeps asking, as C does, and
//! goes on when the device answers again: a log survives a supply that went
//! quiet, or a burst of noise. A reply that is not eight digits and a mode is
//! no reading either, where C makes up 0 V, and what came of a reply is
//! forgotten once it is 500 ms late, where C splices it into the next.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{Link, Model};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::PowerSupply];
const SERIALCOMM: &str = "9600/8n1";
static DEV_OPTIONS: [OptionDesc; 8] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::VoltageTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::Enabled, Access::GET_SET),
];

/// How long one pass of a capture waits for a byte: the session's timeout in
/// `serial_source_add`, and what a stop waits for at worst.
const POLL: Duration = Duration::from_millis(10);
/// How long a capture that only a sample limit could end goes without a
/// reading. C has no bound, so such a capture of a silent device never ends. A
/// time limit or a client's stop ends the rest.
pub const NO_READING_BOUND: Duration = Duration::from_secs(5);

/// The `manson-hcs-3xxx` driver.
pub struct MansonDriver;

impl Driver for MansonDriver {
    fn id(&self) -> &'static str {
        "manson-hcs-3xxx"
    }

    fn long_name(&self) -> &'static str {
        "Manson HCS-3xxx"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = SERIALCOMM.to_owned();
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => {
                    value
                        .as_str()
                        .ok_or_else(|| Error::arg("serialcomm"))?
                        .clone_into(&mut serialcomm);
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        // Every failure of the port here is no device, as `scan` returns
        // NULL for each.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        log::info!("Probing serial port {conn}.");
        match probe(port) {
            Ok(Some(link)) => Ok(vec![Box::new(MansonDevice::new(link, conn, params))]),
            Ok(None) => Ok(Vec::new()),
            Err(e) => {
                log::debug!("Scan failed: {e}.");
                Ok(Vec::new())
            }
        }
    }
}

/// `scan` after the open. The port goes when this returns, as the scan's
/// `serial_close`; `dev_open` opens it again.
fn probe(port: Box<dyn Port>) -> Result<Option<Link>> {
    // Which model does not matter to the asking.
    let mut link = Link::new(Some(port), &protocol::MODELS[0]);
    let reply = link.ask("GMOD\r")?;
    let id = protocol::first_line(&reply);
    let Some(model) = protocol::lookup(id) else {
        log::error!("Unknown model ID '{}' detected, aborting.", String::from_utf8_lossy(id));
        return Ok(None);
    };
    link.model = model;
    let reply = link.ask("GETD\r")?;
    link.take_getd(&reply);
    let reply = link.ask("GMAX\r")?;
    (link.voltage_max, link.current_max) =
        protocol::parse_gmax(protocol::first_line(&reply), model);
    link.port = None;
    Ok(Some(link))
}

/// One power supply.
pub struct MansonDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limit_samples: u64,
    limit_msec: u64,
    /// [`NO_READING_BOUND`], but for tests.
    no_reading_bound: Duration,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl MansonDevice {
    fn new(link: Link, conn: String, params: Params) -> Self {
        MansonDevice {
            info: DeviceInfo {
                driver: "manson-hcs-3xxx",
                vendor: "Manson".to_owned(),
                model: link.model.name.to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "CH1".to_owned())],
            conn,
            params,
            limit_samples: 0,
            limit_msec: 0,
            no_reading_bound: NO_READING_BOUND,
            pending: None,
            link: Arc::new(Mutex::new(link)),
            worker: None,
        }
    }

    /// A device of `model` on a port that stands in for the hardware, without
    /// a scan's reading but for the limits. [`Device::open`] takes it.
    pub fn from_port(model: &'static Model, limits: (f32, f32), port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut link = Link::new(None, model);
        (link.voltage_max, link.current_max) = limits;
        let mut device = Self::new(link, conn, params);
        device.pending = Some(port);
        device
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }
}

/// A value as `config_set` has it: what `sigrok-cli`'s single precision makes
/// of it, within `min` and `max`. A NaN passes C's two comparisons and goes out
/// as `VOLTnan` to a live output; here it is refused. A -0.0 passes them too
/// and goes out as `CURR-00`, which nothing says a supply parses; here it is 0.
fn in_range(key: &Key, value: &Value, min: f64, max: f32) -> Result<f64> {
    match value.as_f64().map(|v| f64::from(v as f32)) {
        Some(v) if (min..=f64::from(max)).contains(&v) => Ok(v + 0.0),
        _ => Err(Error::Arg(format!("{} out of range", key.id()))),
    }
}

fn no_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        None => Ok(()),
    }
}

impl Device for MansonDevice {
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
        no_group(group)?;
        Ok(DEV_OPTIONS.to_vec())
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        no_group(group)?;
        let link = self.link();
        Ok(match key {
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::LimitMsec => Value::U64(self.limit_msec),
            Key::Voltage => Value::F64(f64::from(link.voltage)),
            Key::VoltageTarget => Value::F64(f64::from(link.voltage_target)),
            Key::Current => Value::F64(f64::from(link.current)),
            Key::CurrentLimit => Value::F64(f64::from(link.current_limit)),
            Key::Enabled => Value::Bool(link.output_enabled),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            // `core::config_set` refuses a 0 and says so; this is for a library
            // caller that comes straight here.
            Key::LimitSamples => {
                self.limit_samples = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::LimitMsec => {
                self.limit_msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::VoltageTarget => {
                let mut link = self.link();
                let volts = in_range(key, value, protocol::VOLTAGE_MIN, link.voltage_max)?;
                link.apply(&protocol::voltage_command(volts))?;
                link.voltage_target = volts as f32;
            }
            Key::CurrentLimit => {
                let mut link = self.link();
                let amps = in_range(key, value, protocol::CURRENT_MIN, link.current_max)?;
                let command = protocol::current_command(amps, link.model);
                link.apply(&command)?;
                link.current_limit = amps as f32;
            }
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(bad)?;
                let mut link = self.link();
                link.settle();
                link.send(&protocol::output_command(on)).inspect_err(|_| {
                    log::error!("Could not send SR_CONF_ENABLED command.");
                })?;
                link.read_ack()
                    .inspect_err(|_| {
                        log::error!("Could not read SR_CONF_ENABLED reply.");
                    })
                    .map_err(|e| match e {
                        // It went out, and a reply late or lost says nothing.
                        Error::Timeout if on => {
                            Error::device("no answer from the device: the output may be on")
                        }
                        other => other,
                    })?;
                link.output_enabled = on;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        let link = self.link();
        Ok(match key {
            Key::VoltageTarget => ValueList::RangeStepF64 {
                min: protocol::VOLTAGE_MIN,
                max: f64::from(link.voltage_max),
                step: protocol::VOLTAGE_STEP,
            },
            Key::CurrentLimit => ValueList::RangeStepF64 {
                min: protocol::CURRENT_MIN,
                max: f64::from(link.current_max),
                step: link.model.current_step,
            },
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        if link.port.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&self.conn, self.params)?,
        };
        link.port = Some(port);
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        self.link().port = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(worker) = &self.worker {
            if !worker.handle.is_finished() {
                return Err(Error::Busy);
            }
            let worker = self.worker.take().expect("just seen");
            let _ = worker.handle.join();
        }
        {
            let mut link = self.link();
            if link.port.is_none() {
                return Err(Error::device("device is not open"));
            }
            link.reply_pending = false;
            // C keeps what a stopped capture had of a reply, and reads the
            // next one's first after it (`dev_acquisition_start` resets the
            // flag alone).
            link.buffer = protocol::ReplyBuffer::default();
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            no_reading_bound: self.no_reading_bound,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("manson-hcs-3xxx acquisition".to_owned())
            .spawn(move || {
                capture.run(&sink, &stop2);
                sink.send(Packet::End);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending. It sends
    /// nothing to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// One capture, on its own thread: `hcs_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    limit_msec: u64,
    no_reading_bound: Duration,
}

/// What a pass has learnt.
struct Progress {
    samples: u64,
    /// When the request on its way went out.
    sent_at: Option<Instant>,
    /// Since when there has been no reading.
    quiet_since: Instant,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) {
        sink.send(Packet::header());
        let started = Instant::now();
        let limit = Duration::from_millis(self.limit_msec);
        let mut progress = Progress { samples: 0, sent_at: None, quiet_since: started };
        let only_samples_can_end = self.limit_samples > 0 && self.limit_msec == 0;
        while !stop.load(Ordering::SeqCst) {
            if !self.pass(sink, &mut progress) {
                return;
            }
            if self.limit_samples > 0 && progress.samples >= self.limit_samples {
                return;
            }
            if self.limit_msec > 0 && started.elapsed() >= limit {
                return;
            }
            if only_samples_can_end && progress.quiet_since.elapsed() > self.no_reading_bound {
                log::error!("Acquisition failed: no reading for {:?}.", self.no_reading_bound);
                return;
            }
            if !self.request(&mut progress) {
                return;
            }
        }
    }

    /// A byte, if one comes. False when the capture is over.
    fn pass(&self, sink: &PacketSink, progress: &mut Progress) -> bool {
        let mut link = self.link.lock().expect("link lock");
        let byte = match link.read_byte(POLL) {
            Ok(byte) => byte,
            Err(e) => {
                log::error!("Acquisition failed: {e}.");
                return false;
            }
        };
        let Some(reply) = byte.and_then(|b| link.buffer.push(b)) else {
            return true;
        };
        link.reply_pending = false;
        if reply.is_empty() || !link.take_getd(&reply) {
            return true;
        }
        progress.quiet_since = Instant::now();
        progress.samples += 1;
        let (voltage, current) = (link.voltage, link.current);
        // Not with the link: a full queue must not hold up a client's get or set.
        drop(link);
        send_sample(sink, voltage, current)
    }

    /// The next `GETD` if it is time, as the end of `hcs_receive_data`. False
    /// when the capture is over.
    fn request(&self, progress: &mut Progress) -> bool {
        let mut link = self.link.lock().expect("link lock");
        if std::mem::take(&mut link.reading_dropped) {
            progress.quiet_since = Instant::now();
        }
        if link.reply_pending {
            if progress.sent_at.is_some_and(|at| at.elapsed() > protocol::REQ_TIMEOUT) {
                link.reply_pending = false;
                // C keeps it, and it joins the next reply as one reading.
                link.buffer = protocol::ReplyBuffer::default();
            }
            return true;
        }
        // C goes on after a port that failed, for ever without a time limit
        // (`protocol.c:245`, its result only ends that pass).
        if let Err(e) = link.send("GETD\r") {
            log::error!("Acquisition failed: {e}.");
            return false;
        }
        progress.sent_at = Some(Instant::now());
        link.reply_pending = true;
        true
    }
}

/// `send_sample`: a voltage and a current on the one channel.
fn send_sample(sink: &PacketSink, voltage: f32, current: f32) -> bool {
    sink.send(analog(voltage, Mq::Voltage, MqFlags::DC, Unit::Volt))
        && sink.send(analog(current, Mq::Current, MqFlags::empty(), Unit::Ampere))
}

fn analog(value: f32, mq: Mq, flags: MqFlags, unit: Unit) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![0],
        samples: vec![value],
        mq,
        flags,
        unit,
        digits: 2,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit
    /// that does not bite must fail, not hang the run.
    const CAPTURE_BOUND: Duration = Duration::from_secs(10);

    /// What the fake saw.
    #[derive(Default)]
    struct Seen {
        writes: Vec<(Instant, Vec<u8>)>,
    }

    /// A supply that answers as the protocol says, unless told otherwise, and
    /// notes each write and when.
    struct Fake {
        seen: Arc<Mutex<Seen>>,
        input: VecDeque<u8>,
        /// Replies to `GETD`, each once, then [`Fake::reading`].
        getd: VecDeque<Vec<u8>>,
        reading: Vec<u8>,
        /// The reply to a setting.
        ack: Vec<u8>,
        gmod: Vec<u8>,
        fail_writes_after: Option<usize>,
    }

    impl Fake {
        fn new() -> (Fake, Arc<Mutex<Seen>>) {
            let seen = Arc::new(Mutex::new(Seen::default()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                getd: VecDeque::new(),
                reading: b"050000251\rOK\r".to_vec(),
                ack: b"OK\r".to_vec(),
                gmod: b"3202\rOK\r".to_vec(),
                fail_writes_after: None,
            };
            (fake, seen)
        }
    }

    impl Port for Fake {
        fn name(&self) -> &'static str {
            "/dev/ttyFAKE"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            if self.input.is_empty() {
                std::thread::sleep(timeout);
                return Ok(0);
            }
            let n = self.input.len().min(buf.len());
            for b in &mut buf[..n] {
                *b = self.input.pop_front().expect("counted");
            }
            Ok(n)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            {
                let mut seen = self.seen.lock().unwrap();
                if self.fail_writes_after.is_some_and(|n| seen.writes.len() >= n) {
                    return Err(Error::device("Broken pipe"));
                }
                seen.writes.push((Instant::now(), data.to_vec()));
            }
            let answer = match data {
                b"GMOD\r" => self.gmod.clone(),
                b"GMAX\r" => b"360100\rOK\r".to_vec(),
                b"GETD\r" => self.getd.pop_front().unwrap_or_else(|| self.reading.clone()),
                _ => self.ack.clone(),
            };
            self.input.extend(answer);
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.input.clear();
            Ok(())
        }
    }

    fn writes(seen: &Arc<Mutex<Seen>>) -> Vec<String> {
        let seen = seen.lock().unwrap();
        seen.writes.iter().map(|(_, w)| String::from_utf8_lossy(w).into_owned()).collect()
    }

    fn device(fake: Fake) -> MansonDevice {
        let model = protocol::lookup(b"3202").unwrap();
        let mut d = MansonDevice::from_port(model, (36.0, 10.0), Box::new(fake));
        d.open().unwrap();
        d
    }

    /// The packets up to and with the End, within the bound.
    fn until_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
        let deadline = Instant::now() + CAPTURE_BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            assert!(!left.is_zero(), "no End after {CAPTURE_BOUND:?}");
            let (_, p) = rx.recv_timeout(left).expect("a packet in time");
            let done = matches!(p, Packet::End);
            out.push(p);
            if done {
                return out;
            }
        }
    }

    fn capture(d: &mut MansonDevice) -> Vec<Packet> {
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        until_end(&rx)
    }

    fn readings(packets: &[Packet]) -> Vec<f32> {
        packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some(a.samples[0]),
                _ => None,
            })
            .collect()
    }

    fn getds(seen: &Arc<Mutex<Seen>>) -> usize {
        writes(seen).iter().filter(|w| *w == "GETD\r").count()
    }

    #[test]
    fn a_scan_asks_three_questions_and_sets_nothing() {
        let (fake, seen) = Fake::new();
        let link = probe(Box::new(fake)).unwrap().expect("a model");
        assert_eq!(writes(&seen), ["GMOD\r", "GETD\r", "GMAX\r"]);
        assert_eq!(link.model.name, "HCS-3202");
        assert_eq!(link.voltage, 5.0);
        assert_eq!(link.current, 0.25);
        assert_eq!((link.voltage_max, link.current_max), (36.0, 10.0));
        assert!(link.output_enabled);
        assert!(link.port.is_none(), "closed, as the C's scan");
    }

    #[test]
    fn a_scan_stops_at_an_unknown_model() {
        let (mut fake, seen) = Fake::new();
        fake.gmod = b"9999\rOK\r".to_vec();
        assert!(probe(Box::new(fake)).unwrap().is_none());
        assert_eq!(writes(&seen), ["GMOD\r"]);
    }

    #[test]
    fn a_scan_of_a_silent_port_ends() {
        let (mut fake, seen) = Fake::new();
        fake.gmod = Vec::new();
        let started = Instant::now();
        assert!(matches!(probe(Box::new(fake)), Err(Error::Timeout)));
        assert_eq!(writes(&seen), ["GMOD\r"]);
        assert!(started.elapsed() < protocol::REPLY_TIMEOUT * 2);
    }

    #[test]
    fn get_and_list_send_nothing() {
        let (fake, seen) = Fake::new();
        let d = device(fake);
        for key in [
            Key::Voltage,
            Key::VoltageTarget,
            Key::Current,
            Key::CurrentLimit,
            Key::Enabled,
            Key::LimitSamples,
            Key::LimitMsec,
        ] {
            d.get(&key, None).unwrap();
        }
        d.list(&Key::VoltageTarget, None).unwrap();
        d.list(&Key::CurrentLimit, None).unwrap();
        assert_eq!(writes(&seen).len(), 0);
        assert_eq!(d.get(&Key::VoltageTarget, None).unwrap().as_f64(), Some(0.0));
    }

    #[test]
    fn a_group_is_refused() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        assert!(d.get(&Key::Voltage, Some("CH1")).is_err());
        assert!(d.set(&Key::Enabled, &Value::Bool(true), Some("CH1")).is_err());
        assert_eq!(writes(&seen).len(), 0);
    }

    #[test]
    fn settings_send_their_command_and_read_back() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::VoltageTarget, &Value::F64(5.0), None).unwrap();
        d.set(&Key::CurrentLimit, &Value::F64(2.5), None).unwrap();
        d.set(&Key::Enabled, &Value::Bool(true), None).unwrap();
        assert!(matches!(d.get(&Key::Enabled, None), Ok(Value::Bool(true))));
        d.set(&Key::Enabled, &Value::Bool(false), None).unwrap();
        assert_eq!(writes(&seen), ["VOLT050\r", "CURR025\r", "SOUT0\r", "SOUT1\r"]);
        assert_eq!(d.get(&Key::VoltageTarget, None).unwrap().as_f64(), Some(5.0));
        assert_eq!(d.get(&Key::CurrentLimit, None).unwrap().as_f64(), Some(2.5));
        assert!(matches!(d.get(&Key::Enabled, None), Ok(Value::Bool(false))));
    }

    #[test]
    fn a_value_out_of_range_sends_nothing() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        for v in [0.9, 36.1, -1.0, f64::NAN, f64::INFINITY] {
            assert!(d.set(&Key::VoltageTarget, &Value::F64(v), None).is_err(), "{v}");
        }
        for v in [-0.01, 10.1, f64::NAN] {
            assert!(d.set(&Key::CurrentLimit, &Value::F64(v), None).is_err(), "{v}");
        }
        assert_eq!(writes(&seen).len(), 0);
    }

    #[test]
    fn negative_zero_goes_out_as_zero() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::CurrentLimit, &Value::F64(-0.0), None).unwrap();
        assert_eq!(writes(&seen), ["CURR000\r"]);
    }

    #[test]
    fn a_setting_not_answered_ok_is_not_kept() {
        for ack in [&b"ERR\r"[..], b""] {
            let (mut fake, seen) = Fake::new();
            fake.ack = ack.to_vec();
            let mut d = device(fake);
            assert!(d.set(&Key::VoltageTarget, &Value::F64(5.0), None).is_err());
            assert!(d.set(&Key::CurrentLimit, &Value::F64(5.0), None).is_err());
            assert!(d.set(&Key::Enabled, &Value::Bool(true), None).is_err());
            assert_eq!(d.get(&Key::VoltageTarget, None).unwrap().as_f64(), Some(0.0));
            assert_eq!(d.get(&Key::CurrentLimit, None).unwrap().as_f64(), Some(0.0));
            assert!(matches!(d.get(&Key::Enabled, None), Ok(Value::Bool(false))));
            assert_eq!(writes(&seen).len(), 3);
        }
    }

    #[test]
    fn a_limit_sends_no_getd_after_its_last_sample() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(readings(&packets), [5.0, 0.25, 5.0, 0.25, 5.0, 0.25]);
        assert_eq!(getds(&seen), 3);
    }

    #[test]
    fn a_sample_is_a_voltage_and_a_current_on_the_one_channel() {
        let (fake, _) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let packets = capture(&mut d);
        let analog: Vec<&AnalogData> = packets
            .iter()
            .filter_map(|p| if let Packet::Analog(a) = p { Some(a) } else { None })
            .collect();
        assert_eq!(analog.len(), 2);
        assert_eq!(analog[0].channels, [0]);
        assert_eq!(analog[0].unit, Unit::Volt);
        assert_eq!(analog[0].flags, MqFlags::DC);
        assert_eq!(analog[1].channels, [0]);
        assert_eq!(analog[1].unit, Unit::Ampere);
        assert_eq!(analog[1].flags, MqFlags::empty());
    }

    #[test]
    fn a_time_limit_ends_a_capture() {
        let (fake, _) = Fake::new();
        let mut d = device(fake);
        d.set(&Key::LimitMsec, &Value::U64(50), None).unwrap();
        let started = Instant::now();
        let packets = capture(&mut d);
        assert!(started.elapsed() >= Duration::from_millis(50));
        assert_ne!(readings(&packets).len(), 0);
    }

    #[test]
    fn a_stop_ends_a_capture_that_never_lacks_data() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + CAPTURE_BOUND;
        while getds(&seen) < 2 {
            assert!(Instant::now() < deadline, "no second request");
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        let _ = until_end(&rx);
        let after = getds(&seen);
        std::thread::sleep(Duration::from_millis(30));
        assert_eq!(getds(&seen), after, "no request after the End");
    }

    #[test]
    fn under_a_sample_limit_silence_asks_again_each_500_ms_and_ends_at_the_bound() {
        let (mut fake, seen) = Fake::new();
        fake.reading = Vec::new();
        let mut d = device(fake);
        d.no_reading_bound = Duration::from_millis(1600);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let started = Instant::now();
        let packets = capture(&mut d);
        assert!(started.elapsed() >= d.no_reading_bound);
        assert_eq!(readings(&packets).len(), 0);
        let seen = seen.lock().unwrap();
        assert_eq!(seen.writes.len(), 4, "at 0, 0.5, 1 and 1.5 s");
        for pair in seen.writes.windows(2) {
            let gap = pair[1].0 - pair[0].0;
            assert!(gap >= protocol::REQ_TIMEOUT, "{gap:?}");
            assert!(gap < protocol::REQ_TIMEOUT * 2, "{gap:?}");
        }
    }

    #[test]
    fn under_a_time_limit_silence_goes_past_the_bound_and_readings_resume() {
        let (mut fake, seen) = Fake::new();
        fake.reading = Vec::new();
        fake.getd = [Vec::new(), Vec::new(), b"050000251\rOK\r".to_vec()].into_iter().collect();
        let mut d = device(fake);
        d.no_reading_bound = Duration::from_millis(100);
        d.set(&Key::LimitMsec, &Value::U64(1700), None).unwrap();
        let started = Instant::now();
        let packets = capture(&mut d);
        assert!(started.elapsed() >= Duration::from_millis(1700), "the time limit ended it");
        assert_eq!(readings(&packets), [5.0, 0.25], "after two requests went unanswered");
        assert!(getds(&seen) >= 4, "and it went on asking");
    }

    #[test]
    fn with_no_limit_silence_waits_for_the_stop() {
        let (mut fake, seen) = Fake::new();
        fake.reading = Vec::new();
        let mut d = device(fake);
        d.no_reading_bound = Duration::from_millis(100);
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND).unwrap().1, Packet::Header { .. }));
        assert!(rx.recv_timeout(Duration::from_millis(1100)).is_err(), "no End past the bound");
        assert!(getds(&seen) >= 3, "and it goes on asking");
        d.stop().unwrap();
        let _ = until_end(&rx);
    }

    #[test]
    fn with_no_limit_garbage_goes_on_too() {
        let (mut fake, _) = Fake::new();
        fake.reading = b"xxxxyyyyz\rOK\r".to_vec();
        let mut d = device(fake);
        d.no_reading_bound = Duration::from_millis(50);
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND).unwrap().1, Packet::Header { .. }));
        assert!(rx.recv_timeout(Duration::from_millis(200)).is_err(), "no End past the bound");
        d.stop().unwrap();
        let _ = until_end(&rx);
    }

    #[test]
    fn a_reading_starts_the_bound_afresh() {
        let (mut fake, seen) = Fake::new();
        let good = || b"050000251\rOK\r".to_vec();
        // Each is 500 ms late, the bound is 700 ms, and they take 1 s in all.
        fake.getd = [Vec::new(), good(), Vec::new(), good()].into_iter().collect();
        let mut d = device(fake);
        d.no_reading_bound = Duration::from_millis(700);
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(readings(&packets), [5.0, 0.25, 5.0, 0.25]);
        assert_eq!(getds(&seen), 4);
    }

    #[test]
    fn nothing_but_garbage_sends_no_sample_and_ends_at_the_bound() {
        let (mut fake, seen) = Fake::new();
        fake.reading = b"xxxxyyyyz\rOK\r".to_vec();
        let mut d = device(fake);
        d.no_reading_bound = Duration::from_millis(50);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(readings(&packets).len(), 0);
        assert!(getds(&seen) > 1);
    }

    #[test]
    fn a_reply_with_no_end_makes_up_nothing() {
        let (mut fake, seen) = Fake::new();
        // Past the buffer, and the OK that ends it is lost with it.
        let mut long = vec![b'7'; 4000];
        long.extend(b"123401000\rOK\r");
        fake.getd = [long].into_iter().collect();
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(readings(&packets), [5.0, 0.25]);
        assert_eq!(getds(&seen), 2);
    }

    #[test]
    fn a_failed_write_ends_a_capture() {
        let (mut fake, seen) = Fake::new();
        fake.fail_writes_after = Some(2);
        let mut d = device(fake);
        let started = Instant::now();
        let packets = capture(&mut d);
        // At once: not by the bound on requests, which takes seconds.
        assert!(started.elapsed() < protocol::REQ_TIMEOUT);
        assert_eq!(readings(&packets), [5.0, 0.25, 5.0, 0.25]);
        assert_eq!(getds(&seen), 2);
    }

    #[test]
    fn a_setting_during_a_capture_goes_out() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + CAPTURE_BOUND;
        while getds(&seen) < 2 {
            assert!(Instant::now() < deadline, "no second request");
            std::thread::sleep(Duration::from_millis(1));
        }
        d.set(&Key::Enabled, &Value::Bool(false), None).unwrap();
        assert!(writes(&seen).iter().any(|w| w == "SOUT1\r"));
        d.stop().unwrap();
        let _ = until_end(&rx);
    }

    #[test]
    fn a_setting_with_a_reading_on_its_way_drops_that_first() {
        let (fake, seen) = Fake::new();
        let mut link = Link::new(Some(Box::new(fake)), protocol::lookup(b"3202").unwrap());
        link.send("GETD\r").unwrap();
        link.reply_pending = true;
        link.apply("VOLT050\r").expect("the OK is the setting's");
        assert!(!link.reply_pending);
        assert_eq!(writes(&seen), ["GETD\r", "VOLT050\r"]);
        assert_eq!(link.read_byte(Duration::ZERO).unwrap(), None, "nothing left over");
    }

    #[test]
    fn a_fragment_is_forgotten_when_its_request_expires() {
        let (mut fake, seen) = Fake::new();
        fake.getd = [b"12".to_vec()].into_iter().collect();
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        assert_eq!(readings(&capture(&mut d)), [5.0, 0.25], "not 12 and what follows");
        assert_eq!(getds(&seen), 2);
    }

    #[test]
    fn a_new_capture_starts_with_an_empty_buffer() {
        let (mut fake, _) = Fake::new();
        fake.getd = [b"12".to_vec()].into_iter().collect();
        let mut d = device(fake);
        d.set(&Key::LimitMsec, &Value::U64(30), None).unwrap();
        assert_eq!(readings(&capture(&mut d)).len(), 0);
        d.set(&Key::LimitMsec, &Value::U64(10_000), None).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        assert_eq!(readings(&capture(&mut d)), [5.0, 0.25], "not 12 and what follows");
    }

    #[test]
    fn a_nul_loses_only_its_own_reply() {
        let (mut fake, seen) = Fake::new();
        fake.getd = [b"x\0OK\r".to_vec()].into_iter().collect();
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        assert_eq!(readings(&capture(&mut d)), [5.0, 0.25]);
        assert_eq!(getds(&seen), 2);
    }

    #[test]
    fn a_reply_is_read_up_to_its_bound_and_no_further() {
        let (mut fake, _) = Fake::new();
        fake.gmod = vec![b'x'; protocol::REPLY_MAX + 9];
        let mut link = Link::new(Some(Box::new(fake)), protocol::lookup(b"3202").unwrap());
        assert!(link.ask("GMOD\r").is_err());
        let mut left = 0;
        while link.read_byte(Duration::from_millis(1)).unwrap().is_some() {
            left += 1;
        }
        assert_eq!(left, 9);
    }

    #[test]
    fn nothing_starts_on_a_closed_device() {
        let model = protocol::lookup(b"3202").unwrap();
        let (fake, _) = Fake::new();
        let mut d = MansonDevice::from_port(model, (36.0, 10.0), Box::new(fake));
        let (tx, _rx) = sync_channel(16);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
    }

    #[test]
    fn settings_that_drop_readings_are_no_silence() {
        let (mut fake, _) = Fake::new();
        fake.reading = Vec::new();
        let mut d = device(fake);
        // Each setting waits out the unanswered GETD, 500 ms, and three of them
        // go past the bound. The clock restarts with each.
        d.no_reading_bound = Duration::from_millis(700);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        for _ in 0..3 {
            std::thread::sleep(Duration::from_millis(20));
            d.set(&Key::Enabled, &Value::Bool(false), None).unwrap();
        }
        let mut ended = false;
        while let Ok((_, p)) = rx.try_recv() {
            ended |= matches!(p, Packet::End);
        }
        assert!(!ended, "the bound took the settings for silence");
        d.stop().unwrap();
        let _ = until_end(&rx);
    }

    #[test]
    fn a_timed_out_enabled_on_says_the_output_may_be_on() {
        let (mut fake, _) = Fake::new();
        fake.ack = Vec::new();
        let mut d = device(fake);
        let e = d.set(&Key::Enabled, &Value::Bool(true), None).unwrap_err();
        assert!(e.to_string().contains("the output may be on"), "{e}");
        let e = d.set(&Key::Enabled, &Value::Bool(false), None).unwrap_err();
        assert!(!e.to_string().contains("may be on"), "{e}");
    }

    /// Runs `f` on its own thread: it must end within the bound.
    fn within_bound(what: &str, f: impl FnOnce() + Send + 'static) {
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            f();
            let _ = tx.send(());
        });
        assert!(rx.recv_timeout(CAPTURE_BOUND).is_ok(), "{what} still blocked");
    }

    #[test]
    fn a_full_queue_does_not_hold_the_link() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The header fills the queue, nobody reads, and the worker waits to
        // send its first sample.
        let deadline = Instant::now() + CAPTURE_BOUND;
        while getds(&seen) < 1 {
            assert!(Instant::now() < deadline, "no request");
            std::thread::sleep(Duration::from_millis(1));
        }
        std::thread::sleep(Duration::from_millis(50));
        let writes_before = getds(&seen);
        within_bound("get and set", move || {
            d.get(&Key::Voltage, None).unwrap();
            d.set(&Key::Enabled, &Value::Bool(false), None).unwrap();
            // The reader goes, the worker's send fails, and close can join.
            drop(rx);
            d.close().unwrap();
        });
        assert_eq!(getds(&seen), writes_before, "the worker was waiting");
    }

    #[test]
    fn a_running_capture_closes() {
        let (fake, _) = Fake::new();
        let mut d = device(fake);
        let (tx, _rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        d.close().unwrap();
        assert!(d.link().port.is_none());
    }
}
