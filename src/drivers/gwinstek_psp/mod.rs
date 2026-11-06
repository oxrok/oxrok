// Counterpart in sigrok:
//   libsigrok/src/hardware/gwinstek-psp/api.c,
//   libsigrok/src/hardware/gwinstek-psp/protocol.c,
//   Copyright (C) 2023 ettom <36895504+ettom@users.noreply.github.com>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! GW Instek PSP-603, PSP-405 and PSP-2010 power supplies, one output, on a
//! serial port.
//!
//! `conn=` names the port, `serialcomm=` the line (2400/8n1) and `force_detect=`
//! the model, which the device cannot be asked for: only the ranges depend on it,
//! and a wrong one is not noticed. A scan asks `L` once, looks at what is left to
//! tell the two line ends apart, sets nothing and closes the port. An open opens
//! it again and sends nothing.
//!
//! A `--get` or `--show` answers what the last `L` said, asking again only if that
//! was over 245 ms ago, as a capture does. On a real unit, whose answer takes that
//! long, each sample of a capture is new. Each command waits for the 50 ms the last
//! one needs. A key the device does not have asks nothing, as sigrok-cli looks in
//! the option list first (`main.c:81`), and this driver is asked for one ahead of
//! each capture, while the output is still to go on.
//!
//! A status that does not read leaves no reading: C sends what it last had, parts
//! of the failed status among it, and counts a sample, where here no sample comes
//! until one reads. Two kinds of failure differ. A whole answer that does not read
//! is a device that is there, as when its panel is setting a limit, so the capture
//! waits for it. Nothing, or part of an answer, is silence, and where a sample limit
//! alone could end the capture that ends after [`NO_REPLY_BOUND`] of it. A port that
//! fails ends the capture, as nothing will come of it. See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{Link, Model, Poll};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 3] = [Key::Conn, Key::Serialcomm, Key::ForceDetect];
const CLASSES: [DeviceClass; 1] = [DeviceClass::PowerSupply];
/// `scan`'s default.
const SERIALCOMM: &str = "2400/8n1";
const CHANNEL_V: usize = 0;
const CHANNEL_I: usize = 1;

static DEV_OPTIONS: [OptionDesc; 10] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::VoltageTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::OverTemperatureProtectionActive, Access::GET),
    OptionDesc::new(Key::Enabled, Access::GET_SET),
];

/// `serial_source_add`'s timeout: the time between two samples.
const SAMPLE_INTERVAL: Duration = Duration::from_millis(50);
/// How long a capture that only a sample limit can end goes on hearing nothing, or
/// only parts of answers, where C counts what it last had. A failed read is at
/// least 2 s, so this allows a second try. A whole answer that does not read resets
/// it: touching the panel must not end a capture.
pub const NO_REPLY_BOUND: Duration = Duration::from_secs(5);
/// How long the End waits for room in the queue once a stop has come. A client
/// that stops drains (`Session` does); one that does not must not hold up a close.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How often a wait looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);

/// The `gwinstek-psp` driver.
pub struct PspDriver;

impl Driver for PspDriver {
    fn id(&self) -> &'static str {
        "gwinstek-psp"
    }

    fn long_name(&self) -> &'static str {
        "GW Instek PSP series"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = None;
        let mut force_detect = None;
        for (key, value) in options {
            let text = || value.as_str().ok_or_else(|| Error::arg(key.id())).map(str::to_owned);
            match key {
                Key::Conn => conn = Some(text()?),
                Key::Serialcomm => serialcomm = Some(text()?),
                Key::ForceDetect => force_detect = Some(text()?),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let Some(force_detect) = force_detect else {
            log::error!("The gwinstek-psp driver requires the force_detect parameter");
            return Ok(Vec::new());
        };
        let Some(model) = protocol::model_lookup(&force_detect) else {
            log::error!("Unsupported model ID '{force_detect}', aborting.");
            return Ok(Vec::new());
        };
        // A port that does not open is no device, as `serial_open` in `scan`.
        let opened = Params::parse(serialcomm.as_deref().unwrap_or(SERIALCOMM))
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        match probe(port, params, model) {
            Ok(Some(link)) => Ok(vec![Box::new(PspDevice::new(link, conn))]),
            Ok(None) => {
                log::debug!("Scan failed.");
                Ok(Vec::new())
            }
            Err(e) => {
                log::debug!("Scan failed: {e}.");
                Ok(Vec::new())
            }
        }
    }
}

/// `scan` after the open. The port goes when this returns, as `serial_close`.
fn probe(port: Box<dyn Port>, params: Params, model: &'static Model) -> Result<Option<Link>> {
    let mut link = Link::new(Some(port), params, model);
    if link.poll(&AtomicBool::new(false))? != Poll::Fresh {
        return Ok(None);
    }
    link.check_terminator()?;
    link.take_initial_voltage_target();
    link.port = None;
    Ok(Some(link))
}

/// One power supply.
pub struct PspDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    limit_samples: u64,
    limit_msec: u64,
    no_reply_bound: Duration,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    /// Set ahead of the End, so that a client that saw it can start again.
    done: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl PspDevice {
    fn new(link: Link, conn: String) -> Self {
        let names = [(CHANNEL_V, "V"), (CHANNEL_I, "I")];
        PspDevice {
            info: DeviceInfo {
                driver: "gwinstek-psp",
                vendor: "GW Instek".to_owned(),
                model: link.model.name.to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: Some(conn.clone()),
            },
            channels: names
                .iter()
                .map(|(i, n)| Channel::new(*i, ChannelKind::Analog, (*n).to_owned()))
                .collect(),
            conn,
            limit_samples: 0,
            limit_msec: 0,
            no_reply_bound: NO_REPLY_BOUND,
            pending: None,
            link: Arc::new(Mutex::new(link)),
            worker: None,
        }
    }

    /// A device of `model` on a port that stands in for the hardware, without a
    /// scan's reading. [`Device::open`] takes it.
    pub fn from_port(model: &'static Model, port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device = Self::new(Link::new(None, params, model), conn);
        device.pending = Some(port);
        device
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }
}

/// A value within `range`, each side in single precision: sigrok-cli's `strtof`
/// comes in there and `api.c` compares it to a double. The tables are exact in
/// single precision, so that is C's result too. A NaN passes C's two comparisons
/// (`api.c:254`, `:277`) and goes out as `SU -2147483648` and `SV   nan` to a live
/// output, and a -0.0 as `-0.00`, which nothing says a supply parses; here the one
/// is refused and the other is 0.
fn in_range(key: &Key, value: &Value, range: &protocol::Range) -> Result<f64> {
    let refused = || Error::Arg(format!("{} out of range", key.id()));
    let v = value.as_f64().ok_or_else(refused)?;
    let single = v as f32;
    if single >= range[0] as f32 && single <= range[1] as f32 {
        Ok(v + 0.0)
    } else {
        Err(refused())
    }
}

impl Device for PspDevice {
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
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    /// C lists both limits as readable and `config_get` has no case for them
    /// (`api.c:39-40`, `:228`); here they say what was set, and with none set
    /// nothing, so that `--show` stays as C's.
    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        match key {
            Key::Conn => return Ok(Value::from(self.conn.as_str())),
            Key::LimitSamples | Key::LimitMsec => {
                let limit =
                    if *key == Key::LimitSamples { self.limit_samples } else { self.limit_msec };
                return if limit > 0 {
                    Ok(Value::U64(limit))
                } else {
                    Err(Error::not_supported(key.id()))
                };
            }
            _ => {}
        }
        if !DEV_OPTIONS.iter().any(|o| o.key == *key && o.access.get) {
            return Err(Error::UnknownKey(key.id().to_owned()));
        }
        let mut link = self.link();
        if matches!(link.poll(&AtomicBool::new(false))?, Poll::Unreadable | Poll::Silent) {
            return Err(Error::device("no status from the device"));
        }
        Ok(match key {
            Key::Voltage => Value::F64(f64::from(link.voltage_or_0)),
            Key::VoltageTarget => Value::F64(f64::from(link.voltage_target)),
            Key::Current => Value::F64(f64::from(link.status.current)),
            Key::CurrentLimit => Value::F64(f64::from(link.status.current_limit)),
            Key::Enabled => Value::Bool(link.output_enabled),
            Key::OverTemperatureProtectionActive => Value::Bool(link.status.otp_active),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
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
                let volts = in_range(key, value, &link.model.voltage)?;
                link.set_voltage(volts)?;
            }
            Key::CurrentLimit => {
                let mut link = self.link();
                let amps = in_range(key, value, &link.model.current)?;
                link.set_current(amps)?;
            }
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(bad)?;
                self.link().set_output(on)?;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        let model = self.link().model;
        let range = match key {
            Key::VoltageTarget => model.voltage,
            Key::CurrentLimit => model.current,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        };
        Ok(ValueList::RangeStepF64 { min: range[0], max: range[1], step: range[2] })
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
            None => crate::serial::open(&self.conn, link.params)?,
        };
        link.port = Some(port);
        Ok(())
    }

    /// `dev_close`: what is unread is dropped, and nothing goes on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let mut link = self.link();
        if let Some(port) = &mut link.port
            && let Err(e) = port.purge()
        {
            log::debug!("Could not flush the port: {e}.");
        }
        link.port = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(worker) = &self.worker {
            if !worker.done.load(Ordering::SeqCst) {
                return Err(Error::Busy);
            }
            let worker = self.worker.take().expect("just seen");
            // The capture is over, and this bounds only its End's wait for room.
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
        {
            let mut link = self.link();
            if link.port.is_none() {
                return Err(Error::device("device is not open"));
            }
            link.forget_next_request();
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            no_reply_bound: self.no_reply_bound,
            enabled: [self.channels[CHANNEL_V].enabled, self.channels[CHANNEL_I].enabled],
        };
        let stop = Arc::new(AtomicBool::new(false));
        let done = Arc::new(AtomicBool::new(false));
        let (stop2, done2) = (Arc::clone(&stop), Arc::clone(&done));
        let handle = std::thread::Builder::new()
            .name("gwinstek-psp acquisition".to_owned())
            .spawn(move || {
                let outcome = capture.run(&sink, &stop2);
                done2.store(true, Ordering::SeqCst);
                send_end(&sink, &stop2, outcome);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, done, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending. Nothing goes
    /// to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// A device dropped in mid capture must not leave its thread polling.
impl Drop for PspDevice {
    fn drop(&mut self) {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
    }
}

/// One capture, on its own thread: `gwinstek_psp_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    limit_msec: u64,
    no_reply_bound: Duration,
    /// `-C`, which C ignores: its packets go out for both channels.
    enabled: [bool; 2],
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let started = Instant::now();
        let limit = Duration::from_millis(self.limit_msec);
        let only_samples_can_end = self.limit_samples > 0 && self.limit_msec == 0;
        let mut samples = 0u64;
        // When the device last gave a whole answer, whether or not it read.
        let mut last_heard = started;
        loop {
            if !wait(SAMPLE_INTERVAL, stop) {
                return Ok(());
            }
            // The lock is held for the poll alone, so that a setting gets in
            // between two, and the packets go out with no lock.
            let polled = {
                let mut link = self.link.lock().expect("link lock");
                poll_packets(&mut link, self.enabled, stop)
            };
            match polled {
                Ok(Pass::Sample(packets)) => {
                    last_heard = Instant::now();
                    if !packets.into_iter().all(|p| sink.send_while(p, go_on)) {
                        return Ok(());
                    }
                    samples += 1;
                }
                Ok(Pass::Unreadable) => last_heard = Instant::now(),
                Ok(Pass::Silent) => {
                    if only_samples_can_end && last_heard.elapsed() >= self.no_reply_bound {
                        log::error!("No status from the device for {:?}.", self.no_reply_bound);
                        return Err(Error::device(format!(
                            "no status from the device for {:?}",
                            self.no_reply_bound
                        )));
                    }
                }
                Ok(Pass::Stopped) => return Ok(()),
                // The port failed: nothing more will come of this capture.
                Err(e) => {
                    log::error!("Acquisition failed: {e}.");
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

/// What one pass of a capture came to.
enum Pass {
    /// What changed, then the two readings.
    Sample(Vec<Packet>),
    /// No sample, from a device that gave a whole answer.
    Unreadable,
    /// No sample, and nothing or only part of an answer.
    Silent,
    /// A stop came.
    Stopped,
}

/// One pass: a status, and the packets it comes to.
fn poll_packets(link: &mut Link, enabled: [bool; 2], stop: &AtomicBool) -> Result<Pass> {
    let before = link.status;
    match link.poll(stop)? {
        Poll::Cached | Poll::Fresh => {}
        Poll::Unreadable => return Ok(Pass::Unreadable),
        Poll::Silent => return Ok(Pass::Silent),
        Poll::Stopped => return Ok(Pass::Stopped),
    }
    let mut packets = Vec::new();
    if before.otp_active != link.status.otp_active {
        packets.push(Packet::meta(Key::OverTemperatureProtectionActive, link.status.otp_active));
    }
    if before.output_enabled != link.status.output_enabled {
        packets.push(Packet::meta(Key::Enabled, link.status.output_enabled));
    }
    if target_news(link) {
        log::debug!("Updating session voltage target to {:.2}", link.voltage_target);
        packets.push(Packet::meta(Key::VoltageTarget, f64::from(link.voltage_target)));
        link.set_voltage_target = link.voltage_target;
        link.set_voltage_target_updated = Some(Instant::now());
    }
    if enabled[CHANNEL_V] {
        packets.push(analog(CHANNEL_V, link.voltage_or_0, Mq::Voltage, Unit::Volt, 2));
    }
    if enabled[CHANNEL_I] {
        packets.push(analog(CHANNEL_I, link.status.current, Mq::Current, Unit::Ampere, 3));
    }
    Ok(Pass::Sample(packets))
}

/// Whether the device reports a voltage target other than the last set, and
/// said so over a second after that was set. Never set is C's time 0, a second
/// before anything the device can have said.
fn target_news(link: &Link) -> bool {
    // Exact, as `protocol.c` compares them.
    #[allow(clippy::float_cmp)]
    if link.set_voltage_target == link.voltage_target {
        return false;
    }
    let Some(reported) = link.voltage_target_updated else { return false };
    link.set_voltage_target_updated.is_none_or(|set| set + Duration::from_secs(1) < reported)
}

/// The End, for which a client that goes on reading is waited for without a
/// bound, and after a stop only for [`END_AFTER_STOP`].
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

/// Waits for `time`: false when a stop came first.
fn wait(time: Duration, stop: &AtomicBool) -> bool {
    let due = Instant::now() + time;
    loop {
        if stop.load(Ordering::SeqCst) {
            return false;
        }
        let left = due.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return true;
        }
        std::thread::sleep(left.min(STOP_CHECK));
    }
}

fn analog(channel: usize, value: f32, mq: Mq, unit: Unit, digits: i32) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![channel],
        samples: vec![value],
        mq,
        flags: MqFlags::DC,
        unit,
        digits,
    })
}

#[cfg(test)]
// The values come from text that a float holds exactly, and are compared as sent.
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::atomic::AtomicUsize;
    use std::sync::mpsc::{Receiver, RecvTimeoutError, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit
    /// that does not bite must fail, not hang the run.
    const CAPTURE_BOUND: Duration = Duration::from_secs(20);
    const ON: &[u8] = b"V12.00A0.500W006.0U60I1.00P200F100100";
    const ON_11: &[u8] = b"V11.00A0.750W008.2U60I1.00P200F100100";
    const OFF: &[u8] = b"V12.00A0.000W000.0U60I1.00P200F000100";
    const OFF_7: &[u8] = b"V07.00A0.000W000.0U60I1.00P200F000100";
    const HOT: &[u8] = b"V12.00A0.500W006.0U60I1.00P200F110100";

    type Received = Receiver<(usize, Packet)>;
    type Seen = Arc<Mutex<Vec<(Instant, Vec<u8>)>>>;

    /// A supply that answers each `L` from a list, noting each write and when.
    struct Fake {
        seen: Seen,
        input: VecDeque<u8>,
        /// Answers to `L` in turn, whole; empty is silence; then `then`.
        replies: VecDeque<Vec<u8>>,
        then: Vec<u8>,
        /// Bytes every read has, whatever was asked.
        babble: bool,
        fail_writes_after: Option<usize>,
        fail_reads: bool,
        purges: Arc<AtomicUsize>,
        /// Sets this flag at the read of this number, from 1.
        stop_at_read: Option<(u32, Arc<AtomicBool>)>,
    }

    impl Fake {
        fn new(then: &[u8]) -> (Fake, Seen) {
            let seen = Arc::new(Mutex::new(Vec::new()));
            let mut reply = then.to_vec();
            if !reply.is_empty() {
                reply.extend_from_slice(b"\r\n");
            }
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                replies: VecDeque::new(),
                then: reply,
                babble: false,
                fail_writes_after: None,
                fail_reads: false,
                purges: Arc::new(AtomicUsize::new(0)),
                stop_at_read: None,
            };
            (fake, seen)
        }

        fn with(mut self, replies: &[&[u8]]) -> Fake {
            for r in replies {
                let mut reply = r.to_vec();
                reply.extend_from_slice(b"\r\n");
                self.replies.push_back(reply);
            }
            self
        }

        fn with_raw(mut self, replies: &[&[u8]]) -> Fake {
            self.replies.extend(replies.iter().map(|r| r.to_vec()));
            self
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
            if let Some((at, stop)) = &mut self.stop_at_read {
                *at = at.saturating_sub(1);
                if *at == 0 {
                    stop.store(true, Ordering::SeqCst);
                }
            }
            if self.fail_reads {
                return Err(Error::device("Input/output error"));
            }
            if self.babble {
                std::thread::sleep(Duration::from_millis(1));
                buf[0] = b'x';
                return Ok(1);
            }
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
            let mut seen = self.seen.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
            if self.fail_writes_after.is_some_and(|n| seen.len() >= n) {
                return Err(Error::device("Broken pipe"));
            }
            seen.push((Instant::now(), data.to_vec()));
            if data == b"L\r\n" {
                let reply = self.replies.pop_front().unwrap_or_else(|| self.then.clone());
                self.input.extend(reply);
            }
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.input.clear();
            self.purges.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }
    }

    /// A supply at 2400 baud: its first byte `latency` after the `L`, then 4.17 ms a
    /// byte, and each answer's V one higher than the last, so a stale one shows. A
    /// purge takes only what has arrived, as a tty's does.
    struct Uart {
        seen: Seen,
        input: VecDeque<(Instant, u8)>,
        answers: u32,
        latency: Duration,
        line_end: &'static [u8],
    }

    impl Uart {
        fn new(latency_ms: u64, line_end: &'static [u8]) -> (Uart, Seen) {
            let seen = Arc::new(Mutex::new(Vec::new()));
            let uart = Uart {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                answers: 0,
                latency: Duration::from_millis(latency_ms),
                line_end,
            };
            (uart, seen)
        }

        fn arrived(&self) -> usize {
            let now = Instant::now();
            self.input.iter().take_while(|(at, _)| *at <= now).count()
        }
    }

    impl Port for Uart {
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
            let due = Instant::now() + timeout;
            loop {
                let n = self.arrived().min(buf.len());
                if n > 0 {
                    for b in &mut buf[..n] {
                        *b = self.input.pop_front().expect("counted").1;
                    }
                    return Ok(n);
                }
                if Instant::now() >= due {
                    return Ok(0);
                }
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let now = Instant::now();
            self.seen.lock().unwrap().push((now, data.to_vec()));
            if data == b"L\r\n" {
                self.answers += 1;
                let mut reply =
                    format!("V{:02}.00A0.500W006.0U60I1.00P200F100100", self.answers).into_bytes();
                reply.extend_from_slice(self.line_end);
                let mut at = self.input.back().map_or(now, |(last, _)| now.max(*last));
                at += self.latency;
                for byte in reply {
                    at += Duration::from_micros(4170);
                    self.input.push_back((at, byte));
                }
            }
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            let n = self.arrived();
            self.input.drain(..n);
            Ok(())
        }
    }

    fn voltage(d: &PspDevice) -> f64 {
        match d.get(&Key::Voltage, None) {
            Ok(Value::F64(v)) => v,
            other => panic!("{other:?}"),
        }
    }

    fn params() -> Params {
        Params::parse(SERIALCOMM).unwrap()
    }

    fn model() -> &'static Model {
        &protocol::MODELS[0]
    }

    /// As a scan leaves it, then opened: one `L` has gone.
    fn device(fake: Fake) -> PspDevice {
        let mut d = PspDevice::from_port(model(), Box::new(fake));
        d.open().unwrap();
        let mut link = d.link();
        assert_eq!(link.poll(&AtomicBool::new(false)).unwrap(), Poll::Fresh);
        link.take_initial_voltage_target();
        drop(link);
        d
    }

    fn start(d: &mut PspDevice) -> Received {
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        rx
    }

    /// All a capture sends up to its End, which must come within
    /// [`CAPTURE_BOUND`]: a stop or a limit that does not bite fails.
    fn packets(rx: &Received) -> Vec<Packet> {
        let deadline = Instant::now() + CAPTURE_BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(left) {
                Ok((_, Packet::End)) => return out,
                Ok((_, packet)) => out.push(packet),
                Err(RecvTimeoutError::Timeout) => panic!("no End within the bound"),
                Err(RecvTimeoutError::Disconnected) => panic!("no End"),
            }
        }
    }

    /// (channel, value) of the readings.
    fn readings(all: &[Packet]) -> Vec<(usize, f32)> {
        all.iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some((a.channels[0], a.samples[0])),
                _ => None,
            })
            .collect()
    }

    fn metas(all: &[Packet]) -> Vec<(Key, Value)> {
        all.iter()
            .filter_map(|p| match p {
                Packet::Meta(items) => Some(items[0].clone()),
                _ => None,
            })
            .collect()
    }

    fn writes(seen: &Seen) -> Vec<Vec<u8>> {
        seen.lock().unwrap().iter().map(|(_, w)| w.clone()).collect()
    }

    fn count_l(seen: &Seen) -> usize {
        writes(seen).iter().filter(|w| *w == b"L\r\n").count()
    }

    /// Waits, with a bound, until `what` holds.
    fn wait_for(what: impl Fn() -> bool) {
        wait_within(CAPTURE_BOUND, what);
    }

    /// For what takes a good part of [`CAPTURE_BOUND`] at full speed already.
    fn wait_for_long(what: impl Fn() -> bool) {
        wait_within(CAPTURE_BOUND * 6, what);
    }

    fn wait_within(bound: Duration, what: impl Fn() -> bool) {
        let deadline = Instant::now() + bound;
        while !what() {
            assert!(Instant::now() < deadline, "did not happen within the bound");
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    #[test]
    fn a_scan_sends_one_l_and_finds_the_line_end() {
        let (fake, seen) = Fake::new(ON);
        let link = probe(Box::new(fake), params(), model()).unwrap().unwrap();
        assert_eq!(writes(&seen), vec![b"L\r\n".to_vec()]);
        assert_eq!(link.status.voltage, 12.0);
        assert!(link.port.is_none(), "the port is closed");
    }

    #[test]
    fn urpsp1_is_seen_and_reads_are_a_byte_longer_from_then_on() {
        let raw: &[u8] = b"V12.00A0.500W006.0U60I1.00P200F100100\r\r\n";
        let (fake, seen) = Fake::new(b"");
        let fake = fake.with_raw(&[raw, b"V11.00A0.500W006.0U60I1.00P200F100100\r\r\n"]);
        let mut link = probe(Box::new(fake), params(), model()).unwrap().unwrap();
        link.port = Some({
            let (mut again, _) = Fake::new(b"");
            again.seen = Arc::clone(&seen);
            again = again.with_raw(&[b"V11.00A0.500W006.0U60I1.00P200F100100\r\r\nV"]);
            Box::new(again)
        });
        std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
        assert_eq!(link.poll(&AtomicBool::new(false)).unwrap(), Poll::Fresh);
        assert_eq!(link.status.voltage, 11.0);
        // Read to the end of the longer terminator: only the V is left.
        let mut left = [0u8; 8];
        let n = link.port.as_mut().unwrap().read(&mut left, Duration::from_millis(1)).unwrap();
        assert_eq!(&left[..n], b"V");
    }

    #[test]
    fn two_bytes_left_is_no_device_and_what_is_left_goes() {
        let (fake, _) = Fake::new(b"V12.00A0.000W000.0U60I1.00P200F000100xx");
        assert!(probe(Box::new(fake), params(), model()).is_err());
        let (fake, _) = Fake::new(b"V12.00A0.000W000.0U60I1.00P200F0001000");
        assert!(probe(Box::new(fake), params(), model()).unwrap().is_some());
    }

    #[test]
    fn a_scan_of_a_silent_device_finds_nothing() {
        let (fake, _) = Fake::new(b"");
        assert!(probe(Box::new(fake), params(), model()).unwrap().is_none());
    }

    #[test]
    fn a_drain_of_a_device_that_keeps_talking_is_bounded() {
        let (mut fake, _) = Fake::new(b"");
        fake.babble = true;
        let began = Instant::now();
        assert!(probe(Box::new(fake), params(), model()).unwrap().is_none());
        let took = began.elapsed();
        assert!(took >= protocol::DRAIN_BOUND, "{took:?}");
        assert!(took < protocol::DRAIN_BOUND + Duration::from_secs(3), "{took:?}");
    }

    #[test]
    fn the_initial_target_is_the_voltage_unless_at_the_current_limit() {
        for (reply, want) in [
            (OFF, 12.0f32),
            (ON, 12.0),
            // At the limit, the voltage is not the target.
            (&b"V02.00A2.000W004.0U60I2.00P200F100100"[..], 0.0),
            // 1.99 from 2.00 is under 0.01 in single precision.
            (b"V12.00A1.990W000.0U60I2.00P200F100100", 0.0),
            (b"V12.00A1.980W000.0U60I2.00P200F100100", 12.0),
            // At the limit with the output off is nothing to it.
            (b"V07.00A2.000W004.0U60I2.00P200F000100", 7.0),
        ] {
            let (fake, _) = Fake::new(reply);
            let link = probe(Box::new(fake), params(), model()).unwrap().unwrap();
            assert_eq!(link.voltage_target, want, "{}", reply.escape_ascii());
        }
    }

    #[test]
    fn a_get_inside_the_time_asks_nothing_and_one_after_asks_again() {
        let (fake, seen) = Fake::new(ON_11);
        let began = Instant::now();
        let d = device(fake.with(&[ON]));
        let first = voltage(&d);
        // Only if this was in the time: a slow box has every right to ask again.
        if began.elapsed() < protocol::STATUS_POLL_TIME {
            assert_eq!(count_l(&seen), 1);
            assert_eq!(first, 12.0);
        }
        let asked = count_l(&seen);
        std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
        assert_eq!(voltage(&d), 11.0);
        assert_eq!(count_l(&seen), asked + 1);
    }

    #[test]
    fn get_conn_and_the_limits_ask_nothing() {
        let (fake, seen) = Fake::new(ON);
        let mut d = device(fake);
        std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
        assert!(matches!(d.get(&Key::Conn, None), Ok(Value::Str(s)) if s == "/dev/ttyFAKE"));
        assert!(d.get(&Key::LimitSamples, None).is_err());
        assert!(d.get(&Key::LimitMsec, None).is_err());
        d.set(&Key::LimitSamples, &Value::U64(7), None).unwrap();
        assert!(matches!(d.get(&Key::LimitSamples, None), Ok(Value::U64(7))));
        d.set(&Key::LimitMsec, &Value::U64(9), None).unwrap();
        assert!(matches!(d.get(&Key::LimitMsec, None), Ok(Value::U64(9))));
        assert_eq!(count_l(&seen), 1);
    }

    #[test]
    fn a_get_with_the_output_off_reads_0_and_the_target() {
        let (fake, _) = Fake::new(OFF);
        let d = device(fake);
        assert!(matches!(d.get(&Key::Voltage, None), Ok(Value::F64(v)) if v == 0.0));
        assert!(matches!(d.get(&Key::VoltageTarget, None), Ok(Value::F64(v)) if v == 12.0));
        assert!(matches!(d.get(&Key::Enabled, None), Ok(Value::Bool(false))));
        assert!(matches!(
            d.get(&Key::OverTemperatureProtectionActive, None),
            Ok(Value::Bool(false))
        ));
    }

    #[test]
    fn a_get_that_fails_says_so_and_a_later_one_reads() {
        let (fake, _) = Fake::new(ON);
        let d = device(fake.with(&[ON, b"garbage"]));
        std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
        assert!(d.get(&Key::Voltage, None).is_err());
        assert!(matches!(d.get(&Key::Voltage, None), Ok(Value::F64(v)) if v == 12.0));
    }

    #[test]
    fn a_get_with_the_port_closed_fails() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        d.close().unwrap();
        std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
        assert!(d.get(&Key::Voltage, None).is_err());
        assert!(matches!(d.get(&Key::Conn, None), Ok(Value::Str(_))));
    }

    #[test]
    fn a_status_that_fails_part_way_stores_nothing() {
        let (fake, _) = Fake::new(ON);
        let d = device(fake.with(&[ON, b"V22.00A9.500Wxxx.xU60I1.00P200F000100"]));
        std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
        assert!(d.get(&Key::Current, None).is_err());
        let link = d.link();
        assert_eq!(link.status.current, 0.5);
        assert_eq!(link.status.voltage, 12.0);
    }

    #[test]
    fn settings_send_the_limit_only_if_it_differs_and_read_back() {
        let (fake, seen) = Fake::new(OFF);
        let mut d = device(fake);
        d.set(&Key::VoltageTarget, &Value::F64(5.0), None).unwrap();
        d.set(&Key::VoltageTarget, &Value::F64(4.5), None).unwrap();
        d.set(&Key::VoltageTarget, &Value::F64(20.0), None).unwrap();
        d.set(&Key::CurrentLimit, &Value::F64(1.5), None).unwrap();
        d.set(&Key::Enabled, &Value::Bool(true), None).unwrap();
        d.set(&Key::Enabled, &Value::Bool(false), None).unwrap();
        let sent: Vec<String> =
            writes(&seen).iter().map(|w| String::from_utf8_lossy(w).into_owned()).collect();
        assert_eq!(
            sent,
            [
                "L\r\n",
                "SU 5\r\n",
                "SV 05.00\r\n",
                "SV 04.50\r\n",
                "SU 20\r\n",
                "SV 20.00\r\n",
                "SI 1.50\r\n",
                "KOE\r\n",
                "KOD\r\n"
            ]
        );
        // What a get answers until the next L; six commands' waits have passed,
        // so a get itself would ask.
        let link = d.link();
        assert_eq!(link.voltage_target, 20.0);
        assert_eq!(link.status.current_limit, 1.5);
        assert!(!link.output_enabled);
    }

    #[test]
    fn the_output_told_is_what_a_get_says_and_the_meta_waits_for_the_device() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake.with(&[OFF]));
        d.set(&Key::Enabled, &Value::Bool(true), None).unwrap();
        {
            let link = d.link();
            assert!(link.output_enabled, "told");
            assert!(!link.status.output_enabled, "what the device last said");
        }
        d.set(&Key::LimitSamples, &Value::U64(8), None).unwrap();
        let rx = start(&mut d);
        let found = metas(&packets(&rx));
        assert!(found.iter().any(|(k, v)| *k == Key::Enabled && matches!(v, Value::Bool(true))));
    }

    #[test]
    fn the_limit_equal_to_what_the_device_said_goes_unsent() {
        let (fake, seen) = Fake::new(b"V05.00A0.000W000.0U05I1.00P200F000100");
        let mut d = device(fake);
        d.set(&Key::VoltageTarget, &Value::F64(5.0), None).unwrap();
        assert_eq!(writes(&seen), vec![b"L\r\n".to_vec(), b"SV 05.00\r\n".to_vec()]);
    }

    #[test]
    fn commands_keep_50_ms_apart() {
        let (fake, seen) = Fake::new(OFF);
        let mut d = device(fake);
        d.set(&Key::VoltageTarget, &Value::F64(5.0), None).unwrap();
        d.set(&Key::CurrentLimit, &Value::F64(1.0), None).unwrap();
        d.set(&Key::Enabled, &Value::Bool(true), None).unwrap();
        let times: Vec<Instant> = seen.lock().unwrap().iter().map(|(t, _)| *t).collect();
        assert_eq!(times.len(), 5);
        let least = times.windows(2).map(|w| w[1] - w[0]).min().unwrap();
        assert!(least >= Duration::from_millis(50), "{least:?}");
    }

    #[test]
    fn values_out_of_range_are_refused_and_nothing_goes() {
        let (fake, seen) = Fake::new(OFF);
        let mut d = device(fake);
        for (key, bad) in [
            (Key::VoltageTarget, 60.01),
            (Key::VoltageTarget, -1.0),
            (Key::VoltageTarget, f64::NAN),
            (Key::CurrentLimit, 3.51),
            (Key::CurrentLimit, -0.01),
            (Key::CurrentLimit, f64::NAN),
        ] {
            assert!(d.set(&key, &Value::F64(bad), None).is_err(), "{key:?} {bad}");
        }
        assert_eq!(count_l(&seen), 1);
        assert_eq!(writes(&seen).len(), 1);
    }

    #[test]
    fn each_models_maximum_is_taken_and_minus_0_is_0() {
        for (m, volts, amps, sent) in [
            (0, 60.0, 3.5, ["SU 60\r\n", "SV 60.00\r\n", "SI 3.50\r\n"]),
            (1, 40.0, 5.0, ["SU 40\r\n", "SV 40.00\r\n", "SI 5.00\r\n"]),
            (2, 20.0, 10.0, ["SU 20\r\n", "SV 20.00\r\n", "SI 10.00\r\n"]),
        ] {
            let (fake, seen) = Fake::new(OFF);
            let mut d = PspDevice::from_port(&protocol::MODELS[m], Box::new(fake));
            d.open().unwrap();
            d.set(&Key::VoltageTarget, &Value::F64(volts), None).unwrap();
            d.set(&Key::CurrentLimit, &Value::F64(amps), None).unwrap();
            let got: Vec<String> =
                writes(&seen).iter().map(|w| String::from_utf8_lossy(w).into_owned()).collect();
            assert_eq!(got, sent);
        }
        let (fake, seen) = Fake::new(OFF);
        let mut d = device(fake);
        d.set(&Key::VoltageTarget, &Value::F64(-0.0), None).unwrap();
        d.set(&Key::CurrentLimit, &Value::F64(-0.0), None).unwrap();
        let got: Vec<String> =
            writes(&seen).iter().map(|w| String::from_utf8_lossy(w).into_owned()).collect();
        assert_eq!(got, ["L\r\n", "SU 0\r\n", "SV 00.00\r\n", "SI 0.00\r\n"]);
    }

    #[test]
    fn a_setting_that_cannot_go_out_says_so_and_keeps_what_was() {
        let (mut fake, _) = Fake::new(OFF);
        fake.fail_writes_after = Some(1);
        let mut d = device(fake);
        assert!(d.set(&Key::VoltageTarget, &Value::F64(5.0), None).is_err());
        assert!(d.set(&Key::CurrentLimit, &Value::F64(1.0), None).is_err());
        assert!(d.set(&Key::Enabled, &Value::Bool(true), None).is_err());
        let link = d.link();
        assert_eq!(link.voltage_target, 12.0);
        assert_eq!(link.status.voltage_limit, 60);
        assert_eq!(link.status.current_limit, 1.0);
        assert!(!link.output_enabled);
    }

    #[test]
    fn a_limit_of_samples_is_that_many_pairs_at_the_pace_of_the_timer() {
        let (fake, seen) = Fake::new(ON);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(8), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        let all = packets(&rx);
        let took = began.elapsed();
        assert!(took >= Duration::from_millis(8 * 50), "{took:?}");
        assert_eq!(readings(&all).len(), 16);
        assert_eq!(readings(&all)[..2], [(CHANNEL_V, 12.0), (CHANNEL_I, 0.5)]);
        assert!(matches!(all[0], Packet::Header { .. }));
        // The status time goes from the clock read ahead of each write, not from
        // when the write went, so count them: the scan's, and one per status time.
        let most = 2 + took.as_millis() / protocol::STATUS_POLL_TIME.as_millis();
        let asked = count_l(&seen) as u128;
        assert!((2..=most).contains(&asked), "{asked} L in {took:?}");
    }

    #[test]
    fn the_digits_and_flags_are_cs() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        let all = packets(&rx);
        let analogs: Vec<&AnalogData> = all
            .iter()
            .filter_map(|p| if let Packet::Analog(a) = p { Some(a) } else { None })
            .collect();
        assert_eq!(analogs.len(), 2);
        assert_eq!(
            (analogs[0].mq, analogs[0].unit, analogs[0].digits),
            (Mq::Voltage, Unit::Volt, 2)
        );
        assert_eq!(
            (analogs[1].mq, analogs[1].unit, analogs[1].digits),
            (Mq::Current, Unit::Ampere, 3)
        );
        assert_eq!(analogs[0].flags, MqFlags::DC);
        assert_eq!(analogs[1].flags, MqFlags::DC);
    }

    #[test]
    fn a_disabled_channel_sends_nothing_and_still_counts() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        d.enable_channel(CHANNEL_V, false).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&packets(&rx)), [(CHANNEL_I, 0.5); 3]);
    }

    #[test]
    fn a_time_limit_ends_it() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        d.set(&Key::LimitMsec, &Value::U64(200), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        let all = packets(&rx);
        let took = began.elapsed();
        assert!(took >= Duration::from_millis(200), "{took:?}");
        assert!(took < Duration::from_secs(5), "{took:?}");
        assert_ne!(readings(&all), [] as [(usize, f32); 0]);
    }

    #[test]
    fn with_the_output_off_the_target_comes_once_as_a_meta() {
        let (fake, _) = Fake::new(OFF);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        let rx = start(&mut d);
        let all = packets(&rx);
        let found = metas(&all);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].0, Key::VoltageTarget);
        assert!(matches!(found[0].1, Value::F64(v) if v == 12.0));
        // Ahead of the first reading, which is 0.
        assert!(matches!(all[1], Packet::Meta(_)));
        assert_eq!(readings(&all)[0], (CHANNEL_V, 0.0));
    }

    #[test]
    fn with_the_output_on_no_target_comes() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(10), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(metas(&packets(&rx)).len(), 0);
    }

    #[test]
    fn what_changes_comes_as_a_meta_ahead_of_its_sample() {
        let (fake, _) = Fake::new(OFF);
        let fake = fake.with(&[ON, HOT, ON]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(26), None).unwrap();
        let rx = start(&mut d);
        let all = packets(&rx);
        let found = metas(&all);
        assert_eq!(found.len(), 4, "{found:?}");
        assert_eq!(found[0].0, Key::OverTemperatureProtectionActive);
        assert!(matches!(found[0].1, Value::Bool(true)));
        assert_eq!(found[1].0, Key::OverTemperatureProtectionActive);
        assert!(matches!(found[1].1, Value::Bool(false)));
        assert_eq!(found[2].0, Key::Enabled);
        assert!(matches!(found[2].1, Value::Bool(false)));
        assert_eq!(found[3].0, Key::VoltageTarget);
        assert!(matches!(found[3].1, Value::F64(v) if v == 12.0));
    }

    #[test]
    fn a_new_target_waits_for_a_second_after_the_last() {
        let (fake, _) = Fake::new(OFF_7);
        // Twice: the capture's first look may be the scan's L or one of its own.
        let mut d = device(fake.with(&[OFF, OFF]));
        d.set(&Key::LimitSamples, &Value::U64(36), None).unwrap();
        let rx = start(&mut d);
        let all = packets(&rx);
        let found = metas(&all);
        assert_eq!(found.len(), 2, "{found:?}");
        assert!(matches!(found[0].1, Value::F64(v) if v == 12.0));
        assert!(matches!(found[1].1, Value::F64(v) if v == 7.0));
        // At 50 ms a sample, about 20 lie between the two, and a slower clock has
        // fewer. `target_news_is_cs_condition` holds the second to the millisecond.
        let first = all.iter().position(|p| matches!(p, Packet::Meta(_))).unwrap();
        let second = all.iter().rposition(|p| matches!(p, Packet::Meta(_))).unwrap();
        let between = readings(&all[first..second]).len() / 2;
        assert!(between >= 1, "{between}");
    }

    #[test]
    fn target_news_is_cs_condition() {
        let (fake, _) = Fake::new(OFF);
        let d = device(fake);
        let mut link = d.link();
        let now = Instant::now();
        link.set_voltage_target = 5.0;
        link.voltage_target = 5.0;
        link.voltage_target_updated = Some(now);
        link.set_voltage_target_updated = None;
        assert!(!target_news(&link), "the same");
        link.voltage_target = 7.0;
        assert!(target_news(&link), "never set, and said");
        link.voltage_target_updated = None;
        assert!(!target_news(&link), "never said");
        link.voltage_target_updated = Some(now + Duration::from_millis(1001));
        link.set_voltage_target_updated = Some(now);
        assert!(target_news(&link), "over a second later");
        link.voltage_target_updated = Some(now + Duration::from_millis(1000));
        assert!(!target_news(&link), "exactly a second is not over");
        link.voltage_target = f32::NAN;
        link.set_voltage_target = f32::NAN;
        link.voltage_target_updated = Some(now + Duration::from_millis(1001));
        assert!(target_news(&link), "a NaN differs from itself");
    }

    #[test]
    fn a_status_that_does_not_read_is_no_sample_and_the_next_one_is() {
        let (fake, seen) = Fake::new(ON_11);
        let fake = fake.with(&[ON, b"V22.00A9.500Wxxx.xU60I1.00P200F000100"]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(10), None).unwrap();
        let rx = start(&mut d);
        let all = packets(&rx);
        let values = readings(&all);
        assert_eq!(values.len(), 20);
        assert!(!values.contains(&(CHANNEL_I, 9.5)), "the failed status's current");
        assert!(!values.contains(&(CHANNEL_V, 22.0)));
        assert!(values.contains(&(CHANNEL_V, 11.0)), "the status after it");
        assert_eq!(values.iter().filter(|(c, _)| *c == CHANNEL_V).count(), 10);
        assert!(count_l(&seen) >= 3);
    }

    #[test]
    fn silence_ends_a_sample_limit_after_the_bound_and_not_before() {
        let (fake, _) = Fake::new(b"");
        let fake = fake.with(&[ON]);
        let mut d = device(fake);
        // A hand on the bound, so the test is short: its value is asserted below.
        d.no_reply_bound = Duration::from_millis(2500);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        let all = packets(&rx);
        let took = began.elapsed();
        assert!(took >= Duration::from_millis(2500), "{took:?}");
        assert!(took < Duration::from_secs(10), "{took:?}");
        let samples = readings(&all).len() / 2;
        assert!(samples < 1000);
        // Only what came ahead of the silence: no failed poll is counted.
        assert!(samples <= 8, "{samples}");
        assert_eq!(NO_REPLY_BOUND, Duration::from_secs(5));
    }

    #[test]
    fn silence_under_a_time_limit_lasts_until_the_limit() {
        let (fake, seen) = Fake::new(b"");
        let mut d = device(fake.with(&[ON]));
        d.no_reply_bound = Duration::from_millis(100);
        d.set(&Key::LimitMsec, &Value::U64(3000), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        let all = packets(&rx);
        let took = began.elapsed();
        assert!(took >= Duration::from_millis(3000), "{took:?}");
        assert!(took < Duration::from_secs(8), "{took:?}");
        assert!(count_l(&seen) >= 2);
        assert!(readings(&all).len() / 2 <= 8);
    }

    #[test]
    fn a_late_reply_carries_on_without_a_bound() {
        let (fake, _) = Fake::new(ON_11);
        let fake = fake.with_raw(&[b"V12.00A0.500W006.0U60I1.00P200F100100\r\n", b""]);
        let mut d = device(fake);
        d.no_reply_bound = Duration::from_secs(30);
        d.set(&Key::LimitSamples, &Value::U64(8), None).unwrap();
        let rx = start(&mut d);
        let values = readings(&packets(&rx));
        assert_eq!(values.len(), 16);
        assert!(values.contains(&(CHANNEL_V, 11.0)));
    }

    /// One glitch after a long good run: the bound is from the last answer heard.
    #[test]
    fn the_bound_runs_from_the_last_answer_and_not_the_start() {
        let (fake, seen) = Fake::new(ON_11);
        // At one an L, about 245 ms apart: over 5 s of them, then one that is silent.
        let mut d = device(fake.with(&[ON; 22]).with_raw(&[b""]));
        d.set(&Key::LimitSamples, &Value::U64(1_000_000), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        // Past the bound from the start, and on through the glitch.
        wait_for_long(|| count_l(&seen) >= 26);
        assert!(began.elapsed() >= NO_REPLY_BOUND, "{:?}", began.elapsed());
        assert!(!rx.try_iter().any(|(_, p)| matches!(p, Packet::End)), "the glitch ended it");
        d.stop().unwrap();
        packets(&rx);
    }

    /// Nothing but an unreadable frame, with a bound far shorter than the wait for
    /// each: the capture goes on, where nothing arriving would have ended it.
    #[test]
    fn a_whole_frame_that_does_not_read_is_not_silence() {
        let (fake, seen) = Fake::new(b"V12.00A0.500W006.0u60I1.00P200F100100");
        let mut d = device(fake.with(&[ON, ON]));
        d.no_reply_bound = Duration::from_millis(10);
        d.set(&Key::LimitSamples, &Value::U64(1_000_000), None).unwrap();
        let rx = start(&mut d);
        let asked = count_l(&seen);
        wait_for(|| count_l(&seen) >= asked + 4);
        assert!(!rx.try_iter().any(|(_, p)| matches!(p, Packet::End)), "the panel ended it");
        d.stop().unwrap();
        packets(&rx);
    }

    #[test]
    fn part_of_a_frame_is_silence() {
        let (fake, _) = Fake::new(b"");
        let fake = fake.with(&[ON, ON]).with_raw(&[b"V12.00A0.5"]);
        let mut d = device(fake);
        d.no_reply_bound = Duration::from_millis(10);
        d.set(&Key::LimitSamples, &Value::U64(1_000_000), None).unwrap();
        let rx = start(&mut d);
        packets(&rx);
    }

    #[test]
    fn silence_does_not_end_a_capture_with_no_limit() {
        let (fake, seen) = Fake::new(b"");
        let mut d = device(fake.with(&[ON]));
        d.no_reply_bound = Duration::from_millis(10);
        let rx = start(&mut d);
        wait_for(|| count_l(&seen) >= 4);
        assert!(!rx.try_iter().any(|(_, p)| matches!(p, Packet::End)), "the bound ended it");
        d.stop().unwrap();
        packets(&rx);
    }

    /// A stop just after the `L`, and part way through its answer. Either way what
    /// comes after reads its own, where it was one behind for good.
    #[test]
    fn a_stop_during_a_reply_leaves_no_later_read_behind() {
        for after_l in [Duration::ZERO, Duration::from_millis(150)] {
            let (uart, seen) = Uart::new(80, b"\r\n");
            let mut d = PspDevice::from_port(model(), Box::new(uart));
            d.open().unwrap();
            let rx = start(&mut d);
            wait_for(|| count_l(&seen) >= 3);
            std::thread::sleep(after_l);
            d.stop().unwrap();
            packets(&rx);
            std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
            for _ in 0..3 {
                let v = voltage(&d);
                assert_eq!(v, count_l(&seen) as f64, "{after_l:?}");
                std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
            }
        }
    }

    /// The same for the next capture, straight after: nothing has come yet.
    #[test]
    fn a_capture_after_a_stop_during_a_reply_reads_its_own() {
        let (uart, seen) = Uart::new(80, b"\r\n");
        let mut d = PspDevice::from_port(model(), Box::new(uart));
        d.open().unwrap();
        let rx = start(&mut d);
        wait_for(|| count_l(&seen) >= 3);
        d.stop().unwrap();
        packets(&rx);
        // Enough to go past what was kept, to answers of its own.
        d.set(&Key::LimitSamples, &Value::U64(12), None).unwrap();
        let rx = start(&mut d);
        let got: Vec<f32> = readings(&packets(&rx))
            .iter()
            .filter(|(c, _)| *c == CHANNEL_V)
            .map(|(_, v)| *v)
            .collect();
        // Each is an L's own, in turn. The first may be the last status kept.
        let last = *got.last().unwrap();
        assert_eq!(last, count_l(&seen) as f32, "{got:?}");
    }

    /// A stop during the settling itself leaves the rest for the next one.
    #[test]
    fn a_stop_while_settling_keeps_what_is_still_to_come() {
        let (uart, seen) = Uart::new(80, b"\r\n");
        let mut d = PspDevice::from_port(model(), Box::new(uart));
        d.open().unwrap();
        let stop = AtomicBool::new(true);
        {
            let mut link = d.link();
            std::thread::sleep(Duration::from_millis(1));
            assert_eq!(link.poll(&stop).unwrap(), Poll::Stopped, "stopped in its read");
            assert_eq!(link.poll(&stop).unwrap(), Poll::Stopped, "and in the settling");
        }
        assert_eq!(count_l(&seen), 1, "no L while one is on its way");
        std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
        assert_eq!(voltage(&d), 2.0);
        assert_eq!(count_l(&seen), 2);
    }

    /// The wait is for the answer: a look at what has come so far would miss it, and
    /// it would then be taken for the answer to the next `L`.
    #[test]
    fn the_next_poll_waits_out_the_reply_a_stop_left() {
        let (uart, _) = Uart::new(80, b"\r\n");
        let mut link = Link::new(Some(Box::new(uart)), params(), model());
        let began = Instant::now();
        assert_eq!(link.poll(&AtomicBool::new(true)).unwrap(), Poll::Stopped);
        link.poll(&AtomicBool::new(false)).unwrap();
        // 80 ms, and 39 bytes at 4.17 ms each.
        assert!(began.elapsed() >= Duration::from_millis(240), "{:?}", began.elapsed());
    }

    /// Up to the time the answer had from its `L`, and no longer: this device never
    /// answers, and the time is already over when the next poll comes.
    #[test]
    fn the_wait_for_the_reply_a_stop_left_is_bounded_from_its_l() {
        let (fake, _) = Fake::new(b"");
        let mut link = Link::new(Some(Box::new(fake)), params(), model());
        assert_eq!(link.poll(&AtomicBool::new(true)).unwrap(), Poll::Stopped);
        std::thread::sleep(Duration::from_millis(1100));
        let began = Instant::now();
        assert_eq!(link.poll(&AtomicBool::new(false)).unwrap(), Poll::Silent);
        // The new L's second and the drain's: a wait for the old one would be more.
        assert!(began.elapsed() < Duration::from_secs(5), "{:?}", began.elapsed());
    }

    /// What is beyond the answer a stop left goes with it.
    #[test]
    fn what_follows_the_reply_a_stop_left_goes_too() {
        let (fake, _) = Fake::new(ON_11);
        let fake = fake.with_raw(&[b"V12.00A0.500W006.0U60I1.00P200F100100\r\nxx"]);
        let mut link = Link::new(Some(Box::new(fake)), params(), model());
        assert_eq!(link.poll(&AtomicBool::new(true)).unwrap(), Poll::Stopped);
        std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
        assert_eq!(link.poll(&AtomicBool::new(false)).unwrap(), Poll::Fresh);
        assert_eq!(link.status.voltage, 11.0);
    }

    /// A stop in the drain after a frame that did not read: what the drain did not
    /// get to goes ahead of the next `L`.
    #[test]
    fn a_stop_in_a_drain_leaves_nothing_for_the_next_read() {
        let stop = Arc::new(AtomicBool::new(false));
        let (mut fake, _) = Fake::new(ON_11);
        fake = fake.with_raw(&[b"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\r\nmore"]);
        fake.stop_at_read = Some((2, Arc::clone(&stop)));
        let mut link = Link::new(Some(Box::new(fake)), params(), model());
        assert_eq!(link.poll(&stop).unwrap(), Poll::Stopped);
        std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
        assert_eq!(link.poll(&AtomicBool::new(false)).unwrap(), Poll::Fresh);
        assert_eq!(link.status.voltage, 11.0);
    }

    /// An answer that takes the status time means an `L` a sample: the time runs
    /// from ahead of the `L`, not from the end of its read.
    #[test]
    fn on_a_real_unit_each_sample_is_new() {
        let (uart, _) = Uart::new(80, b"\r\n");
        let mut d = PspDevice::from_port(model(), Box::new(uart));
        d.open().unwrap();
        d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        let rx = start(&mut d);
        let got: Vec<f32> = readings(&packets(&rx))
            .iter()
            .filter(|(c, _)| *c == CHANNEL_V)
            .map(|(_, v)| *v)
            .collect();
        assert_eq!(got, [1.0, 2.0, 3.0, 4.0]);
    }

    /// URPSP1's last byte comes a byte's time after the read of the short line end:
    /// the 20 ms wait is what sees it, and without it each later read is misplaced.
    #[test]
    fn the_late_byte_of_a_longer_line_end_is_waited_for() {
        let (uart, _) = Uart::new(0, b"\r\r\n");
        let mut link = Link::new(Some(Box::new(uart)), params(), model());
        let never = AtomicBool::new(false);
        assert_eq!(link.poll(&never).unwrap(), Poll::Fresh);
        link.check_terminator().unwrap();
        for want in [2.0, 3.0] {
            std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
            assert_eq!(link.poll(&never).unwrap(), Poll::Fresh);
            assert_eq!(link.status.voltage, want);
        }
    }

    #[test]
    fn a_key_the_device_does_not_have_asks_nothing() {
        let (fake, seen) = Fake::new(ON);
        let d = device(fake);
        std::thread::sleep(protocol::STATUS_POLL_TIME + Duration::from_millis(5));
        for key in [Key::Samplerate, Key::Continuous] {
            assert!(matches!(d.get(&key, None), Err(Error::UnknownKey(_))), "{key:?}");
        }
        assert_eq!(count_l(&seen), 1);
    }

    /// A close comes back with its capture over: the End has already gone.
    #[test]
    fn a_close_joins_the_capture() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        let rx = start(&mut d);
        d.close().unwrap();
        assert!(d.worker.is_none());
        assert!(rx.try_iter().any(|(_, p)| matches!(p, Packet::End)), "no End yet");
    }

    #[test]
    fn a_close_drops_what_is_unread() {
        let (fake, _) = Fake::new(ON);
        let purges = Arc::clone(&fake.purges);
        let mut d = device(fake);
        let before = purges.load(Ordering::SeqCst);
        d.close().unwrap();
        assert_eq!(purges.load(Ordering::SeqCst), before + 1);
    }

    #[test]
    fn a_port_that_fails_ends_the_capture() {
        let (mut fake, seen) = Fake::new(ON);
        fake.fail_writes_after = Some(2);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(10_000), None).unwrap();
        let rx = start(&mut d);
        let all = packets(&rx);
        assert!(readings(&all).len() / 2 < 20, "{}", readings(&all).len());
        assert_eq!(writes(&seen).len(), 2, "no write after the one that failed");
        // A continuous capture too, where no bound applies.
        let (mut fake, _) = Fake::new(ON);
        fake.fail_writes_after = Some(1);
        let mut d = device(fake);
        let rx = start(&mut d);
        packets(&rx);
    }

    #[test]
    fn a_read_that_fails_ends_the_capture() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        {
            let mut link = d.link();
            let (mut bad, _) = Fake::new(ON);
            bad.fail_reads = true;
            link.port = Some(Box::new(bad));
        }
        let rx = start(&mut d);
        packets(&rx);
    }

    #[test]
    fn a_stop_ends_a_capture_that_would_go_on() {
        let (fake, seen) = Fake::new(ON);
        let mut d = device(fake);
        let rx = start(&mut d);
        wait_for(|| count_l(&seen) >= 3);
        let ls = count_l(&seen);
        d.stop().unwrap();
        packets(&rx);
        std::thread::sleep(Duration::from_millis(300));
        assert!(count_l(&seen) <= ls + 1, "polling went on");
        d.close().unwrap();
    }

    #[test]
    fn a_stop_is_seen_in_the_wait_for_a_reply() {
        let (fake, seen) = Fake::new(b"");
        let mut d = device(fake.with(&[ON]));
        let rx = start(&mut d);
        wait_for(|| count_l(&seen) >= 2);
        let began = Instant::now();
        d.stop().unwrap();
        packets(&rx);
        let took = began.elapsed();
        assert!(took < Duration::from_millis(900), "{took:?}");
        d.close().unwrap();
    }

    /// Closes on a thread of its own, so a close that hangs fails the test.
    fn close_within(mut d: PspDevice, bound: Duration) {
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            d.close().unwrap();
            let _ = tx.send(());
        });
        assert!(rx.recv_timeout(bound).is_ok(), "the close did not come back");
    }

    /// A close must come back with the queue full and no one reading: at a
    /// sample, which comes when the queue is full after the header.
    #[test]
    fn a_close_does_not_wait_on_a_full_queue() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx.clone(), 0)).unwrap();
        let (_, first) = rx.recv_timeout(CAPTURE_BOUND).unwrap();
        assert!(matches!(first, Packet::Header { .. }));
        tx.send((0, Packet::End)).unwrap();
        // The first sample is due after 50 ms: give it time to meet the full queue.
        std::thread::sleep(Duration::from_millis(150));
        close_within(d, Duration::from_secs(5));
        drop(rx);
    }

    /// And at the header, with no room from the start.
    #[test]
    fn a_close_does_not_wait_at_the_header() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        let (tx, rx) = sync_channel(1);
        tx.send((0, Packet::End)).unwrap();
        d.start(PacketSink::new(tx, 0)).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        close_within(d, Duration::from_secs(5));
        drop(rx);
    }

    #[test]
    fn a_stop_ends_the_wait_between_samples() {
        let began = Instant::now();
        assert!(!wait(Duration::from_secs(10), &AtomicBool::new(true)));
        assert!(began.elapsed() < Duration::from_secs(5));
        assert!(wait(Duration::from_millis(1), &AtomicBool::new(false)));
    }

    #[test]
    fn a_client_gone_ends_the_capture() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        let rx = start(&mut d);
        drop(rx);
        wait_for(|| d.worker.as_ref().unwrap().done.load(Ordering::SeqCst));
    }

    #[test]
    fn a_capture_can_start_as_soon_as_the_last_one_sent_its_end() {
        for _ in 0..5 {
            let (fake, _) = Fake::new(ON);
            let mut d = device(fake);
            d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
            let rx = start(&mut d);
            assert_eq!(readings(&packets(&rx)).len(), 2);
            let rx = start(&mut d);
            assert_eq!(readings(&packets(&rx)).len(), 2);
        }
    }

    /// A capture that has ended is no longer busy, even with its End still
    /// waiting, and the new one must not wait on an End no one will take.
    #[test]
    fn a_start_does_not_wait_on_an_end_no_one_takes() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        // No room at all: with no one in `recv`, the End can only wait.
        let (tx, rx) = sync_channel(0);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let mut got = 0;
        while got < 2 {
            let (_, packet) = rx.recv_timeout(CAPTURE_BOUND).unwrap();
            got += readings(&[packet]).len();
        }
        wait_for(|| d.worker.as_ref().unwrap().done.load(Ordering::SeqCst));
        let (result_tx, result_rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let (tx, _rx) = sync_channel(1024);
            let started = d.start(PacketSink::new(tx, 0)).is_ok();
            let _ = result_tx.send(started);
            d.close().unwrap();
        });
        assert_eq!(result_rx.recv_timeout(Duration::from_secs(5)), Ok(true));
        drop(rx);
    }

    #[test]
    fn a_second_start_while_running_is_refused() {
        let (fake, _) = Fake::new(ON);
        let mut d = device(fake);
        let rx = start(&mut d);
        let (tx, _rx2) = sync_channel(4);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.stop().unwrap();
        packets(&rx);
        d.close().unwrap();
    }

    #[test]
    fn a_start_with_no_port_is_refused() {
        let (fake, _) = Fake::new(ON);
        let mut d = PspDevice::from_port(model(), Box::new(fake));
        let (tx, _rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
    }

    #[test]
    fn a_setting_gets_in_during_a_capture() {
        let (fake, seen) = Fake::new(ON);
        let mut d = device(fake);
        let rx = start(&mut d);
        d.set(&Key::CurrentLimit, &Value::F64(1.25), None).unwrap();
        assert!(writes(&seen).contains(&b"SI 1.25\r\n".to_vec()));
        d.stop().unwrap();
        packets(&rx);
        d.close().unwrap();
    }

    #[test]
    fn a_dropped_device_stops_its_thread() {
        let (fake, seen) = Fake::new(ON);
        let mut d = device(fake);
        let rx = start(&mut d);
        wait_for(|| count_l(&seen) >= 2);
        drop(d);
        packets(&rx);
    }

    #[test]
    fn the_options_are_cs() {
        let (fake, _) = Fake::new(ON);
        let d = device(fake);
        let keys: Vec<Key> = d.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
        assert_eq!(
            keys,
            [
                Key::Conn,
                Key::Continuous,
                Key::LimitSamples,
                Key::LimitMsec,
                Key::Voltage,
                Key::VoltageTarget,
                Key::Current,
                Key::CurrentLimit,
                Key::OverTemperatureProtectionActive,
                Key::Enabled,
            ]
        );
        assert!(d.options(Some("V")).is_err());
        let list = d.list(&Key::VoltageTarget, None).unwrap();
        assert!(matches!(list, ValueList::RangeStepF64 { min, max, step }
            if min == 0.0 && max == 60.0 && step == 0.01));
        let list = d.list(&Key::CurrentLimit, None).unwrap();
        assert!(matches!(list, ValueList::RangeStepF64 { min, max, step }
            if min == 0.0 && max == 3.5 && step == 0.01));
        assert!(d.list(&Key::Voltage, None).is_err());
    }

    #[test]
    fn a_scan_without_what_it_needs_finds_nothing_and_opens_nothing() {
        let driver = PspDriver;
        assert_eq!(driver.scan(&[]).unwrap().len(), 0);
        let conn = (Key::Conn, Value::from("/dev/ttyDOESNOTEXIST"));
        assert_eq!(driver.scan(std::slice::from_ref(&conn)).unwrap().len(), 0);
        let model = (Key::ForceDetect, Value::from("PSP-999"));
        assert_eq!(driver.scan(&[conn.clone(), model]).unwrap().len(), 0);
        let model = (Key::ForceDetect, Value::from("PSP-603"));
        assert_eq!(driver.scan(&[conn, model]).unwrap().len(), 0);
        assert!(matches!(
            driver.scan(&[(Key::Samplerate, Value::U64(1))]),
            Err(Error::UnknownKey(_))
        ));
    }
}
