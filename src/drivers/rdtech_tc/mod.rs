// Counterpart in sigrok:
//   libsigrok/src/hardware/rdtech-tc/api.c,
//   Copyright (C) 2020 Andreas Sandberg <andreas@sandberg.pp.se>,
//   with commits by Gerhard Sittig, Mathieu Pilato
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The `RDTech` TC66C USB power meter, on a USB CDC port.
//!
//! `conn=` names the port, as this driver does not look for one, and
//! `serialcomm=` overrides 115200/8n1. A scan asks once and takes the name,
//! version and serial number from the reply, then closes the port; an open
//! opens it again.
//!
//! A capture asks `getva` at most every 100 ms, and only with no part of a
//! reply in hand and something received since the last request, as C does.
//! A reply is a frame of six analog channels. So a meter that keeps a reply
//! back is asked nothing more, and C waits for it with no end. Under `--time`
//! or `--continuous` so does this driver, as a late reply recovers there. Only
//! a frame limit alone could then never end the capture, so there a reply owed
//! for [`NO_REPLY_BOUND`] does. That has a price: a reply later than that loses
//! what C would still get. A request that fails is tried again on the next
//! pass, as in C, which goes on for ever: here [`MAX_WRITE_FAILURES`] in a row
//! end the capture. A bad reply ends it, as in C.
//!
//! C's Bluetooth ports (`bt/...`, `bgetva\r\n`) are not here: `serial::open`
//! has none.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    MqFlags, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{CHANNELS, POLL_LEN, POLL_REQUEST, RSP_BUF_SIZE};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::EnergyMeter];
const SERIALCOMM: &str = "115200/8n1";
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];

/// `PROBE_TO_MS`: how long a scan waits for its reply.
const PROBE_TIMEOUT: Duration = Duration::from_millis(1000);
/// `POLL_PERIOD_MS`: the time from one request to the next at the least.
const POLL_PERIOD: Duration = Duration::from_millis(100);
/// The period of `serial_source_add`'s timeout: one look at the limits, the
/// stop and the next request, whatever comes or does not.
const SOURCE_PERIOD: Duration = Duration::from_millis(50);
/// How long a reply may be owed before a capture with only a frame limit ends.
/// C has no bound: the session goes on without a request for ever, and takes a
/// reply however late. A real meter's pace is not known; this leaves it room
/// many times its 2 Hz display. A time limit or a client's stop ends the rest.
pub const NO_REPLY_BOUND: Duration = Duration::from_secs(5);
/// Failed requests in a row that end a capture. Each pass tries again, so with
/// nothing to read they are a [`SOURCE_PERIOD`] apart, a quarter of a second.
pub const MAX_WRITE_FAILURES: u32 = 5;
/// The pause after a read that gave nothing before its time, as a hung up tty
/// gives 0 at once.
const EMPTY_READ_PAUSE: Duration = Duration::from_millis(1);

/// The `rdtech-tc` driver.
pub struct RdtechTcDriver;

impl Driver for RdtechTcDriver {
    fn id(&self) -> &'static str {
        "rdtech-tc"
    }

    fn long_name(&self) -> &'static str {
        "RDTech TC66C USB power meter"
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
        // A port that fails here is no device, as `rdtech_tc_scan` returns
        // NULL for it.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (mut port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        let Some(info) = probe(port.as_mut()) else {
            log::error!("Failed to find a supported RDTech TC device.");
            return Ok(Vec::new());
        };
        Ok(vec![Box::new(RdtechTcDevice::new(&info, conn, params))])
    }
}

/// `rdtech_tc_probe`: asks once, and what a good reply says.
fn probe(port: &mut dyn Port) -> Option<protocol::DevInfo> {
    log::debug!("Poll request '{}'.", String::from_utf8_lossy(POLL_REQUEST));
    if port.write(POLL_REQUEST).is_err() {
        log::error!("Failed to send probe request.");
        return None;
    }
    let mut reply = [0; POLL_LEN];
    if !matches!(crate::serial::read_blocking(port, &mut reply, PROBE_TIMEOUT), Ok(POLL_LEN)) {
        log::error!("Failed to read probe response.");
        return None;
    }
    let Some(plain) = protocol::process_poll(&reply) else {
        log::error!("Unrecognized TC device!");
        return None;
    };
    Some(protocol::dev_info(&plain))
}

/// One meter.
pub struct RdtechTcDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limit_frames: u64,
    limit_msec: u64,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// [`NO_REPLY_BOUND`], but for tests.
    no_reply_bound: Duration,
    /// The open port, with the capture while one runs.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl RdtechTcDevice {
    fn new(info: &protocol::DevInfo, conn: String, params: Params) -> Self {
        let channels = CHANNELS
            .iter()
            .enumerate()
            .map(|(i, c)| Channel::new(i, ChannelKind::Analog, c.name.to_owned()))
            .collect();
        RdtechTcDevice {
            info: DeviceInfo {
                driver: "rdtech-tc",
                vendor: "RDTech".to_owned(),
                model: info.model.clone(),
                version: info.version.clone(),
                serial: info.serial.clone(),
                // `conn` is not in `devopts`, so sigrok-cli names none.
                conn: None,
            },
            channels,
            conn,
            params,
            limit_frames: 0,
            limit_msec: 0,
            pending: None,
            no_reply_bound: NO_REPLY_BOUND,
            port: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device on a port that stands in for the hardware, with no scan.
    /// [`Device::open`] takes it.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let info = protocol::DevInfo {
            model: "TC66".to_owned(),
            version: String::new(),
            serial: String::new(),
        };
        let mut device = Self::new(&info, conn, params);
        device.pending = Some(port);
        device
    }
}

fn no_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        None => Ok(()),
    }
}

impl Device for RdtechTcDevice {
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
        match key {
            Key::LimitFrames => Ok(Value::U64(self.limit_frames)),
            Key::LimitMsec => Ok(Value::U64(self.limit_msec)),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitFrames => self.limit_frames = value.as_u64().ok_or_else(bad)?,
            Key::LimitMsec => self.limit_msec = value.as_u64().ok_or_else(bad)?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut slot = self.port.lock().expect("port lock");
        if slot.is_some() {
            return Ok(());
        }
        *slot = Some(match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&self.conn, self.params)?,
        });
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        *self.port.lock().expect("port lock") = None;
        Ok(())
    }

    /// `dev_acquisition_start`: the first request goes out here, and if it
    /// cannot the start fails, as in C.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(worker) = &self.worker {
            if !worker.handle.is_finished() {
                return Err(Error::Busy);
            }
            let worker = self.worker.take().expect("just seen");
            let _ = worker.handle.join();
        }
        {
            let mut slot = self.port.lock().expect("port lock");
            let port = slot.as_mut().ok_or_else(|| Error::device("device is not open"))?;
            port.write(POLL_REQUEST).inspect_err(|_| {
                log::error!("Unable to send poll request.");
            })?;
        }
        let capture = Capture {
            port: Arc::clone(&self.port),
            enabled: self.channels.iter().map(|c| c.enabled).collect(),
            limit_frames: self.limit_frames,
            limit: (self.limit_msec > 0).then(|| Duration::from_millis(self.limit_msec)),
            no_reply_bound: self.no_reply_bound,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("rdtech-tc acquisition".to_owned())
            .spawn(move || {
                sink.send(Packet::header());
                let outcome = capture.run(&sink, &stop2);
                sink.end(outcome);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end and does not wait: it may be sending. Nothing
    /// goes to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// One capture, on its own thread: `rdtech_tc_receive_data`.
struct Capture {
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    enabled: Vec<bool>,
    limit_frames: u64,
    limit: Option<Duration>,
    no_reply_bound: Duration,
}

/// What has come and when the last request went: `dev_context`'s fields.
struct Rx {
    buf: [u8; RSP_BUF_SIZE],
    len: usize,
    /// `rx_after_tx`: what has come since the last request.
    since_request: usize,
    requested: Instant,
    /// Since when nothing has come of a reply owed.
    quiet_since: Instant,
    /// Requests that failed since the last that went out.
    write_failures: u32,
}

enum Pass {
    /// Nothing that ends the capture.
    Go,
    /// The capture is over: a limit, or nobody listening.
    End,
    /// The capture failed.
    Failed(Error),
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let started = Instant::now();
        let mut rx = Rx {
            buf: [0; RSP_BUF_SIZE],
            len: 0,
            since_request: 0,
            requested: started,
            quiet_since: started,
            write_failures: 0,
        };
        let mut frames = 0;
        let mut guard = self.port.lock().expect("port lock");
        let Some(port) = guard.as_mut() else {
            return Err(Error::device("device is not open"));
        };
        while !stop.load(Ordering::SeqCst) {
            match self.receive(port.as_mut(), &mut rx, &mut frames, sink) {
                Pass::Go => {}
                Pass::End => return Ok(()),
                Pass::Failed(e) => return Err(e),
            }
            if self.limit.is_some_and(|limit| started.elapsed() > limit) {
                return Ok(());
            }
            match self.poll(port.as_mut(), &mut rx) {
                Pass::Go => {}
                Pass::End => return Ok(()),
                Pass::Failed(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// One look at the port, `recv_poll_data` when something came: a reply
    /// whole is a frame. What follows it stays for the next one.
    fn receive(
        &self,
        port: &mut dyn Port,
        rx: &mut Rx,
        frames: &mut u64,
        sink: &PacketSink,
    ) -> Pass {
        let looked = Instant::now();
        let n = match port.read(&mut rx.buf[rx.len..], SOURCE_PERIOD) {
            Ok(n) => n,
            Err(e) => {
                log::error!("Acquisition failed: {e}.");
                return Pass::Failed(e);
            }
        };
        if n == 0 && looked.elapsed() < SOURCE_PERIOD {
            std::thread::sleep(EMPTY_READ_PAUSE);
        }
        if n > 0 {
            rx.len += n;
            rx.since_request += n;
            rx.quiet_since = Instant::now();
        }
        if rx.len < POLL_LEN {
            return Pass::Go;
        }
        let reply: &[u8; POLL_LEN] = rx.buf[..POLL_LEN].try_into().expect("the length");
        let Some(plain) = protocol::process_poll(reply) else {
            log::error!("Failed to process poll packet.");
            return Pass::Failed(Error::device("failed to process poll packet"));
        };
        if !self.send_frame(&plain, sink) {
            return Pass::End;
        }
        *frames += 1;
        if self.limit_frames > 0 && *frames >= self.limit_frames {
            return Pass::End;
        }
        rx.buf.copy_within(POLL_LEN..rx.len, 0);
        rx.len -= POLL_LEN;
        Pass::Go
    }

    /// A frame of the channels that are on. C sends all six whatever `-C`
    /// said (`protocol.c:266-274`).
    fn send_frame(&self, plain: &[u8; POLL_LEN], sink: &PacketSink) -> bool {
        let values = CHANNELS.iter().enumerate().filter(|(i, _)| self.enabled[*i]);
        sink.send(Packet::FrameBegin)
            && values.into_iter().all(|(i, c)| {
                sink.send(Packet::Analog(AnalogData {
                    channels: vec![i],
                    samples: vec![c.value(plain)],
                    mq: c.mq,
                    flags: MqFlags::default(),
                    unit: c.unit,
                    digits: c.digits,
                }))
            })
            && sink.send(Packet::FrameEnd)
    }

    /// `rdtech_tc_poll` with no force: nothing with part of a reply in hand
    /// or with none of it come, the next request once the period is over.
    fn poll(&self, port: &mut dyn Port, rx: &mut Rx) -> Pass {
        if rx.len > 0 || rx.since_request == 0 {
            let only_frames_can_end = self.limit_frames > 0 && self.limit.is_none();
            if only_frames_can_end && rx.quiet_since.elapsed() > self.no_reply_bound {
                log::error!("No reply from the device for {:?}.", self.no_reply_bound);
                return Pass::Failed(Error::device(format!(
                    "no reply from the device for {:?}",
                    self.no_reply_bound
                )));
            }
            return Pass::Go;
        }
        if rx.requested.elapsed() < POLL_PERIOD {
            return Pass::Go;
        }
        if let Err(e) = port.write(POLL_REQUEST) {
            log::error!("Unable to send poll request.");
            log::debug!("{e}");
            rx.write_failures += 1;
            return if rx.write_failures >= MAX_WRITE_FAILURES {
                Pass::Failed(e)
            } else {
                Pass::Go
            };
        }
        rx.write_failures = 0;
        rx.requested = Instant::now();
        rx.quiet_since = rx.requested;
        rx.since_request = 0;
        Pass::Go
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::tests::reply;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit
    /// that does not bite must fail, not hang the run.
    const CAPTURE_BOUND: Duration = Duration::from_secs(10);

    /// What the fake saw.
    #[derive(Default)]
    struct Seen {
        writes: crate::serial::WriteLog,
        /// Writes tried, those that failed too.
        attempts: usize,
    }

    /// A meter: each write brings the next scripted answer, the last again
    /// when they run out.
    struct Fake {
        seen: Arc<Mutex<Seen>>,
        input: VecDeque<u8>,
        answers: Vec<Vec<u8>>,
        fail_writes_after: Option<usize>,
        /// The attempts, from 0, that fail.
        fail_attempts: Vec<usize>,
        fail_reads: bool,
    }

    impl Fake {
        fn new(answers: Vec<Vec<u8>>) -> (Fake, Arc<Mutex<Seen>>) {
            let seen = Arc::new(Mutex::new(Seen::default()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                answers,
                fail_writes_after: None,
                fail_attempts: Vec::new(),
                fail_reads: false,
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
            if self.fail_reads {
                return Err(Error::device("Input/output error"));
            }
            if self.input.is_empty() {
                std::thread::sleep(timeout.min(Duration::from_millis(2)));
                return Ok(0);
            }
            let n = self.input.len().min(buf.len());
            for b in &mut buf[..n] {
                *b = self.input.pop_front().expect("counted");
            }
            Ok(n)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let mut seen = self.seen.lock().unwrap();
            let attempt = seen.attempts;
            seen.attempts += 1;
            if self.fail_attempts.contains(&attempt)
                || self.fail_writes_after.is_some_and(|n| seen.writes.len() >= n)
            {
                return Err(Error::device("Broken pipe"));
            }
            let number = seen.writes.len();
            seen.writes.push((Instant::now(), data.to_vec()));
            let answer = self.answers.get(number).or(self.answers.last());
            self.input.extend(answer.into_iter().flatten());
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.input.clear();
            Ok(())
        }
    }

    fn good() -> Vec<u8> {
        reply(|_| {}).to_vec()
    }

    fn device(fake: Fake) -> RdtechTcDevice {
        let mut d = RdtechTcDevice::from_port(Box::new(fake));
        d.no_reply_bound = Duration::from_millis(150);
        d.open().unwrap();
        d
    }

    fn start(d: &mut RdtechTcDevice) -> Receiver<(usize, Packet)> {
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        rx
    }

    /// The packets to the End, which has to come within the bound.
    fn to_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
        let deadline = Instant::now() + CAPTURE_BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            assert!(!left.is_zero(), "no End after {CAPTURE_BOUND:?}");
            let (_, p) = rx.recv_timeout(left).expect("a packet in time");
            let end = matches!(p, Packet::End);
            out.push(p);
            if end {
                return out;
            }
        }
    }

    fn capture(d: &mut RdtechTcDevice) -> Vec<Packet> {
        to_end(&start(d))
    }

    fn frames(packets: &[Packet]) -> usize {
        packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count()
    }

    fn analog_channels(packets: &[Packet]) -> Vec<usize> {
        packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some(a.channels[0]),
                _ => None,
            })
            .collect()
    }

    fn writes(seen: &Arc<Mutex<Seen>>) -> usize {
        seen.lock().unwrap().writes.len()
    }

    #[test]
    fn a_frame_is_one_request_and_six_channels_in_order() {
        let (fake, seen) = Fake::new(vec![good()]);
        let mut d = device(fake);
        d.set(&Key::LimitFrames, &Value::U64(3), None).unwrap();
        let packets = capture(&mut d);
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert!(matches!(packets[1], Packet::FrameBegin));
        assert!(matches!(packets[8], Packet::FrameEnd));
        assert_eq!(frames(&packets), 3);
        assert_eq!(analog_channels(&packets[..9]), [0, 1, 2, 3, 4, 5]);
        assert_eq!(analog_channels(&packets).len(), 18);
        assert_eq!(writes(&seen), 3, "the meter would answer a fourth");
        assert!(seen.lock().unwrap().writes.iter().all(|(_, w)| w == b"getva"));
    }

    #[test]
    fn requests_keep_the_period() {
        let (fake, seen) = Fake::new(vec![good()]);
        let mut d = device(fake);
        d.set(&Key::LimitFrames, &Value::U64(4), None).unwrap();
        capture(&mut d);
        let seen = seen.lock().unwrap();
        assert_eq!(seen.writes.len(), 4);
        for pair in seen.writes.windows(2) {
            assert!(pair[1].0 - pair[0].0 >= POLL_PERIOD, "{:?}", pair[1].0 - pair[0].0);
        }
    }

    #[test]
    fn a_silent_meter_is_asked_nothing_more_and_ends_the_capture_at_the_bound() {
        let (fake, seen) = Fake::new(vec![Vec::new(), good()]);
        let mut d = device(fake);
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let started = Instant::now();
        let packets = capture(&mut d);
        assert!(started.elapsed() >= d.no_reply_bound);
        assert_eq!(frames(&packets), 0);
        assert_eq!(writes(&seen), 1, "a second request would be answered");
    }

    #[test]
    fn a_time_limit_goes_past_the_bound() {
        let (fake, seen) = Fake::new(vec![Vec::new()]);
        let mut d = device(fake);
        d.set(&Key::LimitMsec, &Value::U64(600), None).unwrap();
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let started = Instant::now();
        let packets = capture(&mut d);
        assert!(started.elapsed() >= Duration::from_millis(600), "{:?}", started.elapsed());
        assert_eq!(frames(&packets), 0);
        assert_eq!(writes(&seen), 1);
    }

    #[test]
    fn with_no_limit_a_silent_meter_waits_for_the_stop() {
        let (fake, seen) = Fake::new(vec![Vec::new()]);
        let mut d = device(fake);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND).unwrap().1, Packet::Header { .. }));
        let waited = d.no_reply_bound * 3;
        assert!(rx.recv_timeout(waited).is_err(), "nothing, not an End, well past the bound");
        d.stop().unwrap();
        to_end(&rx);
        assert_eq!(writes(&seen), 1);
    }

    #[test]
    fn part_of_a_reply_in_hand_stops_the_requests() {
        let mut cut = good();
        cut.truncate(100);
        let (fake, seen) = Fake::new(vec![cut, good()]);
        let mut d = device(fake);
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(frames(&packets), 0);
        assert_eq!(writes(&seen), 1, "a second request would be answered");
    }

    #[test]
    fn what_follows_a_reply_stops_the_requests_too() {
        let mut long = good();
        long.extend_from_slice(&[0xa5; 10]);
        let (fake, seen) = Fake::new(vec![long, good()]);
        let mut d = device(fake);
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(frames(&packets), 1);
        assert_eq!(writes(&seen), 1, "a second request would be answered");
    }

    #[test]
    fn a_reply_twice_is_two_frames_of_one_request() {
        let with_volts =
            |volts: u32| reply(|p| p[48..52].copy_from_slice(&volts.to_le_bytes())).to_vec();
        let mut twice = with_volts(10_000);
        twice.extend_from_slice(&with_volts(20_000));
        let (fake, seen) = Fake::new(vec![twice, good()]);
        let mut d = device(fake);
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(frames(&packets), 2);
        assert_eq!(writes(&seen), 1);
        let volts: Vec<f32> = packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) if a.channels[0] == 0 => Some(a.samples[0]),
                _ => None,
            })
            .collect();
        assert_eq!(volts, [1.0, 2.0], "the second reply, moved up from behind the first");
    }

    #[test]
    fn a_reply_in_pieces_is_one_frame() {
        let (fake, seen) = Fake::new(vec![good()]);
        let mut d = RdtechTcDevice::from_port(Box::new(Pieces(fake)));
        d.open().unwrap();
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        assert_eq!(frames(&capture(&mut d)), 1);
        assert_eq!(writes(&seen), 1);
    }

    /// A port that gives one byte per read.
    struct Pieces(Fake);

    impl Port for Pieces {
        fn name(&self) -> &str {
            self.0.name()
        }
        fn set_params(&mut self, p: Params) -> Result<()> {
            self.0.set_params(p)
        }
        fn set_handshake(&mut self, r: Option<bool>, d: Option<bool>) -> Result<()> {
            self.0.set_handshake(r, d)
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            self.0.read(&mut buf[..1], timeout)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            self.0.write(data)
        }
        fn purge(&mut self) -> Result<()> {
            self.0.purge()
        }
    }

    #[test]
    fn only_the_channels_that_are_on_are_sent() {
        let (fake, _) = Fake::new(vec![good()]);
        let mut d = device(fake);
        for pos in [0, 1, 3, 5] {
            d.enable_channel(pos, false).unwrap();
        }
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(frames(&packets), 2);
        assert_eq!(analog_channels(&packets), [2, 4, 2, 4]);
    }

    #[test]
    fn a_bad_reply_ends_the_capture_and_keeps_the_frames_before() {
        let mut bad = good();
        bad[100] ^= 1;
        let (fake, seen) = Fake::new(vec![good(), bad, good()]);
        let packets = capture(&mut device(fake));
        assert_eq!(frames(&packets), 1);
        assert_eq!(writes(&seen), 2, "a third request would be answered");
    }

    #[test]
    fn a_stop_ends_a_meter_that_always_answers() {
        let (fake, seen) = Fake::new(vec![good()]);
        let mut d = device(fake);
        let rx = start(&mut d);
        rx.recv_timeout(CAPTURE_BOUND).unwrap();
        rx.recv_timeout(CAPTURE_BOUND).unwrap();
        d.stop().unwrap();
        to_end(&rx);
        let asked = writes(&seen);
        std::thread::sleep(POLL_PERIOD * 2);
        assert_eq!(writes(&seen), asked, "nothing asked after the End");
    }

    #[test]
    fn a_time_limit_ends_a_meter_that_always_answers() {
        let (fake, _) = Fake::new(vec![good()]);
        let mut d = device(fake);
        d.no_reply_bound = CAPTURE_BOUND * 2;
        d.set(&Key::LimitMsec, &Value::U64(250), None).unwrap();
        let started = Instant::now();
        let packets = capture(&mut d);
        assert!(started.elapsed() >= Duration::from_millis(250));
        assert!(frames(&packets) >= 2, "{}", frames(&packets));
    }

    #[test]
    fn a_first_request_that_fails_fails_the_start() {
        let (mut fake, _) = Fake::new(vec![good()]);
        fake.fail_writes_after = Some(0);
        let (tx, _rx) = sync_channel(16);
        assert!(device(fake).start(PacketSink::new(tx, 0)).is_err());
    }

    #[test]
    fn a_request_that_fails_is_tried_again() {
        let (mut fake, seen) = Fake::new(vec![good()]);
        fake.fail_attempts = (1..).take(MAX_WRITE_FAILURES as usize - 1).collect();
        let mut d = device(fake);
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        assert_eq!(frames(&capture(&mut d)), 2, "one short of the bound");
        assert_eq!(writes(&seen), 2);
        assert_eq!(seen.lock().unwrap().attempts, 1 + MAX_WRITE_FAILURES as usize);
    }

    #[test]
    fn only_failures_in_a_row_count() {
        let (mut fake, seen) = Fake::new(vec![good()]);
        // Every other request fails, more of them than the bound.
        let rounds = 2 * MAX_WRITE_FAILURES as usize;
        fake.fail_attempts = (0..rounds).map(|i| 2 * i + 1).collect();
        let mut d = device(fake);
        let limit = u64::try_from(rounds).unwrap();
        d.set(&Key::LimitFrames, &Value::U64(limit), None).unwrap();
        assert_eq!(frames(&capture(&mut d)), rounds);
        assert_eq!(writes(&seen), rounds);
    }

    #[test]
    fn requests_that_keep_failing_end_the_capture_at_the_bound() {
        let (mut fake, seen) = Fake::new(vec![good()]);
        fake.fail_writes_after = Some(1);
        let mut d = device(fake);
        // No limit: only the bound can end this.
        d.no_reply_bound = CAPTURE_BOUND * 2;
        let packets = capture(&mut d);
        assert_eq!(frames(&packets), 1);
        assert_eq!(writes(&seen), 1);
        assert_eq!(seen.lock().unwrap().attempts, 1 + MAX_WRITE_FAILURES as usize);
    }

    #[test]
    fn a_read_that_fails_ends_the_capture() {
        let (mut fake, _) = Fake::new(vec![good()]);
        fake.fail_reads = true;
        let packets = capture(&mut device(fake));
        assert_eq!(frames(&packets), 0);
    }

    #[test]
    fn a_probe_wants_one_good_reply() {
        let (mut fake, seen) = Fake::new(vec![good()]);
        let info = probe(&mut fake).unwrap();
        assert_eq!(info.model, "TC66");
        assert_eq!(writes(&seen), 1);
        let (mut fake, _) = Fake::new(vec![good()[..191].to_vec()]);
        assert_eq!(probe(&mut fake), None);
        let mut garbled = good();
        garbled[0] ^= 1;
        let (mut fake, _) = Fake::new(vec![garbled]);
        assert_eq!(probe(&mut fake), None);
        let (mut fake, _) = Fake::new(vec![good()]);
        fake.fail_writes_after = Some(0);
        assert_eq!(probe(&mut fake), None);
    }

    #[test]
    fn the_limits_read_back() {
        let (fake, _) = Fake::new(vec![good()]);
        let mut d = device(fake);
        assert_eq!(d.get(&Key::LimitFrames, None).unwrap(), Value::U64(0));
        d.set(&Key::LimitFrames, &Value::U64(3), None).unwrap();
        d.set(&Key::LimitMsec, &Value::U64(250), None).unwrap();
        assert_eq!(d.get(&Key::LimitFrames, None).unwrap(), Value::U64(3));
        assert_eq!(d.get(&Key::LimitMsec, None).unwrap(), Value::U64(250));
        assert!(d.get(&Key::Conn, None).is_err(), "as C's config_get");
    }
}
