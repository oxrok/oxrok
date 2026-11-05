// Counterpart in sigrok:
//   libsigrok/src/hardware/fluke-dmm/api.c,
//   libsigrok/src/hardware/fluke-dmm/protocol.c,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Martin Ling,
//   Daniel Elstner, Soeren Apel, Gerhard Sittig, Mathieu Pilato,
//   Andreas Sandberg
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Fluke handheld meters (87, 89, 187, 189, 287, 289) and the `ScopeMeter`
//! 199B, behind an infrared cable.
//!
//! `conn=` names the port. Without `serialcomm=` a scan tries 115200, 9600
//! and 1200 Bd, three `ID`s each, and the first that answers with a model of
//! the table is the device.
//!
//! A capture sends `QM`. On a handheld each answer is a sample, and the next
//! `QM` goes 100 ms after the last, or a second after it when no answer came.
//! On a `ScopeMeter` the answer says what the display shows, and each answer
//! then asks for the next value (`QM 11` or `QM 19`), as fast as the meter
//! gives them, while a `QM` every second keeps up with the display (3.5 s
//! without an answer). A silent meter is waited for, as it may come back: a
//! `--samples` capture of one ends only with an interrupt, or with a port
//! that fails, as with the cable pulled.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port, read_blocking};
use protocol::{Decoder, LINE_MAX, Profile};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
/// `scan_conn`: 287/289, 87/89/187/189, the 190 series.
const SCAN_LINES: [&str; 3] = ["115200/8n1", "9600/8n1", "1200/8n1"];
/// Tries of `ID` at one line.
const TRIES: usize = 3;
/// "The Scopemeter 199B, at least, requires this after all the 115k/9.6k
/// confusion."
const BETWEEN_LINES: Duration = Duration::from_millis(5);
/// `serial_source_add`'s 50 ms: how long the capture waits for a byte
/// before it looks at its limits and timers.
const POLL: Duration = Duration::from_millis(50);
const SET: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, SET),
    OptionDesc::new(Key::LimitMsec, SET),
];

/// The `fluke-dmm` driver.
pub struct FlukeDriver;

impl Driver for FlukeDriver {
    fn id(&self) -> &'static str {
        "fluke-dmm"
    }

    fn long_name(&self) -> &'static str {
        "Fluke 8x/18x/28x series DMMs"
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
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => serialcomm = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let found = match serialcomm {
            Some(text) => probe(&conn, &text),
            None => SCAN_LINES.iter().find_map(|line| {
                let found = probe(&conn, line);
                if found.is_none() {
                    std::thread::sleep(BETWEEN_LINES);
                }
                found
            }),
        };
        Ok(found
            .map(|(identity, params)| -> Box<dyn Device> {
                Box::new(FlukeDevice::new(identity.profile, identity.version, conn, params))
            })
            .into_iter()
            .collect())
    }
}

/// `fluke_scan`: the port opened and closed around up to three tries. Every
/// failure of the port here is no device.
fn probe(conn: &str, serialcomm: &str) -> Option<(protocol::Identity, Params)> {
    let opened = Params::parse(serialcomm)
        .and_then(|params| Ok((crate::serial::open(conn, params)?, params)));
    let (mut port, params) = match opened {
        Ok(opened) => opened,
        Err(e) => {
            log::error!("{e}");
            return None;
        }
    };
    identify(port.as_mut()).map(|identity| (identity, params))
}

/// The tries. A port that fails is logged and tried again, as C does: what
/// follows the last try is the next line's open, which says more.
fn identify(port: &mut dyn Port) -> Option<protocol::Identity> {
    for _ in 0..TRIES {
        if let Err(e) = port.purge() {
            log::debug!("{e}");
        }
        match protocol::ask_identity(port) {
            Ok(Some(identity)) => return Some(identity),
            Ok(None) => {}
            Err(e) => {
                log::debug!("{e}");
                log::error!("Unable to send ID string");
            }
        }
    }
    None
}

/// One meter.
pub struct FlukeDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    profile: &'static Profile,
    limit_samples: u64,
    limit_msec: u64,
    /// A port to use instead of opening `conn`, for tests and replays.
    pending: Option<Box<dyn Port>>,
    /// The port while it is open and no capture holds it: a capture gives
    /// it back before its `End`, so a client may start again on seeing that.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    capturing: Arc<AtomicBool>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl FlukeDevice {
    fn new(profile: &'static Profile, version: String, conn: String, params: Params) -> Self {
        FlukeDevice {
            info: DeviceInfo {
                driver: "fluke-dmm",
                vendor: "Fluke".to_owned(),
                model: profile.name.to_owned(),
                version,
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "P1".to_owned())],
            conn,
            params,
            profile,
            limit_samples: 0,
            limit_msec: 0,
            pending: None,
            port: Arc::new(Mutex::new(None)),
            capturing: Arc::new(AtomicBool::new(false)),
            worker: None,
        }
    }

    /// A device of `profile` on a port that stands in for the hardware,
    /// without a scan. [`Device::open`] takes it.
    pub fn from_port(profile: &'static Profile, port: Box<dyn Port>) -> Self {
        let params = Params::parse(SCAN_LINES[0]).expect("the table parses");
        let conn = port.name().to_owned();
        let mut device = Self::new(profile, String::new(), conn, params);
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

impl Device for FlukeDevice {
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

    /// `config_get` is `NULL` there.
    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        no_group(group)?;
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => self.limit_samples = value.as_u64().ok_or_else(bad)?,
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

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let mut port = self
            .port
            .lock()
            .expect("port lock")
            .take()
            .ok_or_else(|| Error::device("device is not open"))?;
        self.capturing.store(true, Ordering::SeqCst);
        let capture = Capture {
            profile: self.profile,
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            channels: self.channels.iter().map(|c| c.index).collect(),
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let slot = Arc::clone(&self.port);
        let capturing = Arc::clone(&self.capturing);
        let handle = std::thread::Builder::new()
            .name("fluke-dmm acquisition".to_owned())
            .spawn(move || {
                capture.run(port.as_mut(), &sink, &stop2);
                *slot.lock().expect("port lock") = Some(port);
                capturing.store(false, Ordering::SeqCst);
                sink.send(Packet::End);
            })
            .map_err(Error::Io)?;
        // A worker that came before has given its port back and sent, or is
        // sending, its End: it holds nothing that this one needs.
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending. It
    /// sends nothing to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// One capture: `dev_acquisition_start` and `fluke_receive_data`.
struct Capture {
    profile: &'static Profile,
    limit_samples: u64,
    limit_msec: u64,
    channels: Vec<usize>,
}

impl Capture {
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) {
        let started = Instant::now();
        let limit = Duration::from_millis(self.limit_msec);
        if !sink.send(Packet::header()) {
            return;
        }
        if let Err(e) = port.write(b"QM\r") {
            log::debug!("{e}");
            log::error!("Unable to send QM.");
            return;
        }
        let mut sent_at = Instant::now();
        let mut waiting = true;
        let mut samples = 0u64;
        let mut decoder = Decoder::new(self.profile.family);
        let mut lines = Lines::default();
        while !stop.load(Ordering::SeqCst) {
            let mut byte = [0];
            let got = match read_blocking(port, &mut byte, POLL) {
                Ok(got) => got,
                Err(e) => {
                    log::error!("Read failed: {e}");
                    return;
                }
            };
            if let Some(line) = (got > 0).then(|| lines.take(byte[0])).flatten() {
                let handled = decoder.line(&line);
                waiting &= !handled.answered;
                // The reading first, as `handle_line` has it: a write that
                // fails must not lose a sample the meter has given.
                if let Some(reading) = handled.reading {
                    samples += 1;
                    if !sink.send(self.analog(reading)) {
                        return;
                    }
                }
                if let Some(Err(e)) = handled.ask.as_ref().map(|ask| port.write(ask.as_bytes())) {
                    log::debug!("{e}");
                    log::error!("Cannot send QM (measurement).");
                    return;
                }
            }
            if self.limit_samples > 0 && samples >= self.limit_samples {
                return;
            }
            if self.limit_msec > 0 && started.elapsed() > limit {
                return;
            }
            let elapsed = sent_at.elapsed();
            if (!waiting && elapsed > self.profile.poll_period) || elapsed > self.profile.timeout {
                // C logs this and asks again every second for ever
                // (protocol.c:132-133): here a port gone ends the capture.
                if let Err(e) = port.write(b"QM\r") {
                    log::debug!("{e}");
                    log::error!("Unable to send QM.");
                    return;
                }
                sent_at = Instant::now();
                waiting = true;
            }
        }
    }

    fn analog(&self, r: protocol::Reading) -> Packet {
        Packet::Analog(AnalogData {
            channels: self.channels.clone(),
            samples: vec![r.value],
            mq: r.mq,
            flags: r.flags,
            unit: r.unit,
            digits: r.digits,
        })
    }
}

/// Cuts a capture's bytes into lines. C stops reading for good once its
/// buffer is full with no CR (protocol.c:106), and no limit but time ends the
/// run; here that line is dropped and the next one read.
#[derive(Default)]
struct Lines {
    line: Vec<u8>,
    overlong: bool,
}

impl Lines {
    /// A byte, and the line it ends.
    fn take(&mut self, byte: u8) -> Option<Vec<u8>> {
        if byte == b'\r' {
            let line = std::mem::take(&mut self.line);
            return (!std::mem::take(&mut self.overlong)).then_some(line);
        }
        if !self.overlong {
            self.line.push(byte);
            if self.line.len() == LINE_MAX {
                log::warn!("A line of over {} bytes dropped.", LINE_MAX - 1);
                self.line.clear();
                self.overlong = true;
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::protocol::PROFILES;
    use super::*;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit
    /// that does not bite must fail, not hang the run.
    const BOUND: Duration = Duration::from_secs(5);

    type Received = Receiver<(usize, Packet)>;

    /// A meter's profile for a test, on times short enough to wait for.
    static QUICK: Profile = Profile {
        name: "quick",
        family: protocol::Family::Fields,
        poll_period: Duration::from_millis(30),
        timeout: Duration::from_millis(120),
    };
    static QUICK_SCOPE: Profile = Profile {
        name: "quick",
        family: protocol::Family::Scope,
        poll_period: Duration::from_millis(200),
        timeout: Duration::from_millis(600),
    };

    #[derive(Default)]
    struct Seen {
        writes: Vec<(Instant, Vec<u8>)>,
        purges: usize,
        open: bool,
    }

    type Answer = fn(usize, &[u8]) -> Vec<u8>;

    /// A meter: `answer` gets the number of the write and its bytes.
    struct Fake {
        seen: Arc<Mutex<Seen>>,
        input: VecDeque<u8>,
        answer: Answer,
        fail_writes_from: Option<usize>,
        fail_reads: bool,
    }

    impl Fake {
        fn new(answer: Answer) -> (Fake, Arc<Mutex<Seen>>) {
            let seen = Arc::new(Mutex::new(Seen::default()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                answer,
                fail_writes_from: None,
                fail_reads: false,
            };
            (fake, seen)
        }
    }

    impl Drop for Fake {
        fn drop(&mut self) {
            // Not an unwrap: a test that failed holding the lock must fail
            // as itself, not abort the run here.
            if let Ok(mut seen) = self.seen.lock() {
                seen.open = false;
            }
        }
    }

    impl Port for Fake {
        fn name(&self) -> &'static str {
            "fake"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            if self.fail_reads {
                return Err(Error::device("read failed"));
            }
            if let Some(b) = self.input.pop_front() {
                buf[0] = b;
                return Ok(1);
            }
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
            Ok(0)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let mut seen = self.seen.lock().unwrap();
            let n = seen.writes.len();
            if self.fail_writes_from.is_some_and(|from| n >= from) {
                return Err(Error::device("write failed"));
            }
            seen.writes.push((Instant::now(), data.to_vec()));
            self.input.extend((self.answer)(n, data));
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.seen.lock().unwrap().purges += 1;
            self.input.clear();
            Ok(())
        }
    }

    fn reading(_: usize, _: &[u8]) -> Vec<u8> {
        b"0\r1.5E0,VDC,NORMAL,NONE\r".to_vec()
    }

    fn silent(_: usize, _: &[u8]) -> Vec<u8> {
        Vec::new()
    }

    fn device(profile: &'static Profile, fake: Fake) -> FlukeDevice {
        let mut d = FlukeDevice::from_port(profile, Box::new(fake));
        d.open().unwrap();
        d
    }

    fn start(d: &mut FlukeDevice) -> Received {
        let (tx, rx) = sync_channel(4096);
        let sink = PacketSink::new(tx, 0);
        d.start(sink).unwrap();
        rx
    }

    /// Up to the `End`. A capture that keeps sending past its limit fails
    /// here at the bound rather than running on.
    fn packets_to_end(rx: &Received) -> Vec<Packet> {
        let deadline = Instant::now() + BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(left) {
                Ok((_, Packet::End)) => return out,
                Ok((_, p)) => out.push(p),
                Err(e) => panic!("no End: {e}, after {} packets", out.len()),
            }
        }
    }

    fn values(packets: &[Packet]) -> Vec<f32> {
        packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some(a.samples[0]),
                _ => None,
            })
            .collect()
    }

    fn writes(seen: &Arc<Mutex<Seen>>) -> Vec<Vec<u8>> {
        seen.lock().unwrap().writes.iter().map(|(_, w)| w.clone()).collect()
    }

    #[test]
    fn the_table_is_cs() {
        let times = |p: &Profile| (p.poll_period.as_millis(), p.timeout.as_millis());
        let names: Vec<_> = PROFILES.iter().map(|p| p.name).collect();
        assert_eq!(names, ["87", "89", "187", "189", "199B", "287", "289"]);
        for p in &PROFILES {
            let expected = if p.name == "199B" { (1000, 3500) } else { (100, 1000) };
            assert_eq!(times(p), expected, "{}", p.name);
        }
    }

    #[test]
    fn a_limit_of_samples_ends_it_after_that_many_questions() {
        let (fake, seen) = Fake::new(reading);
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        let rx = start(&mut d);
        let packets = packets_to_end(&rx);
        assert!(matches!(packets[0], Packet::Header { .. }), "{packets:?}");
        assert_eq!(values(&packets), [1.5, 1.5, 1.5]);
        assert_eq!(writes(&seen), [b"QM\r"; 3]);
        d.close().unwrap();
    }

    #[test]
    fn the_header_goes_before_the_first_question() {
        let (mut fake, seen) = Fake::new(reading);
        fake.fail_writes_from = Some(0);
        let mut d = device(&QUICK, fake);
        let rx = start(&mut d);
        let packets = packets_to_end(&rx);
        assert_eq!(packets.len(), 1, "the header alone: {packets:?}");
        assert_eq!(writes(&seen).len(), 0);
        d.close().unwrap();
    }

    #[test]
    fn an_answered_question_paces_at_the_poll_period() {
        let (fake, seen) = Fake::new(reading);
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        let rx = start(&mut d);
        packets_to_end(&rx);
        let seen = seen.lock().unwrap();
        for pair in seen.writes.windows(2) {
            let gap = pair[1].0 - pair[0].0;
            assert!(gap > QUICK.poll_period, "{gap:?}");
            assert!(gap < QUICK.timeout, "{gap:?}");
        }
    }

    #[test]
    fn a_silent_meter_is_asked_again_only_at_the_timeout() {
        let (fake, seen) = Fake::new(silent);
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitMsec, &Value::U64(400), None).unwrap();
        let rx = start(&mut d);
        let packets = packets_to_end(&rx);
        assert_eq!(values(&packets), []);
        let seen = seen.lock().unwrap();
        assert!(seen.writes.len() >= 2, "{}", seen.writes.len());
        for pair in seen.writes.windows(2) {
            let gap = pair[1].0 - pair[0].0;
            assert!(gap > QUICK.timeout, "{gap:?}");
        }
    }

    #[test]
    fn a_time_limit_ends_a_silent_capture() {
        let (fake, _) = Fake::new(silent);
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitMsec, &Value::U64(50), None).unwrap();
        let t = Instant::now();
        packets_to_end(&start(&mut d));
        assert!(t.elapsed() >= Duration::from_millis(50));
        assert!(t.elapsed() < BOUND);
    }

    #[test]
    fn a_stop_ends_a_capture_that_has_no_limit() {
        let (fake, seen) = Fake::new(reading);
        let mut d = device(&QUICK, fake);
        let rx = start(&mut d);
        let (_, first) = rx.recv_timeout(BOUND).unwrap();
        assert!(matches!(first, Packet::Header { .. }));
        let (_, second) = rx.recv_timeout(BOUND).unwrap();
        assert!(matches!(second, Packet::Analog(_)));
        d.stop().unwrap();
        packets_to_end(&rx);
        let after = writes(&seen).len();
        std::thread::sleep(QUICK.poll_period * 3);
        assert_eq!(writes(&seen).len(), after, "nothing more is sent");
        d.close().unwrap();
    }

    #[test]
    fn a_stop_sends_nothing_of_its_own() {
        let (fake, seen) = Fake::new(silent);
        let mut d = device(&QUICK, fake);
        let rx = start(&mut d);
        rx.recv_timeout(BOUND).unwrap();
        d.stop().unwrap();
        packets_to_end(&rx);
        d.close().unwrap();
        assert_eq!(writes(&seen), [b"QM\r"]);
    }

    #[test]
    fn a_port_gone_ends_the_capture() {
        let (mut fake, seen) = Fake::new(silent);
        fake.fail_writes_from = Some(1);
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        packets_to_end(&start(&mut d));
        assert_eq!(writes(&seen).len(), 1);
        d.close().unwrap();

        let (mut fake, _) = Fake::new(silent);
        fake.fail_reads = true;
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        packets_to_end(&start(&mut d));
        d.close().unwrap();
    }

    fn scope_answers(_: usize, data: &[u8]) -> Vec<u8> {
        match data {
            b"QM\r" => b"0\r11,1,3,1,1,0,1\r".to_vec(),
            _ => b"0\r+1.5E+00\r".to_vec(),
        }
    }

    #[test]
    fn a_write_that_fails_loses_no_sample() {
        let (mut fake, _) = Fake::new(scope_answers);
        fake.fail_writes_from = Some(2);
        let mut d = device(&QUICK_SCOPE, fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let packets = packets_to_end(&start(&mut d));
        assert_eq!(values(&packets), [1.5]);
        d.close().unwrap();
    }

    #[test]
    fn a_scope_asks_once_more_after_the_last_sample() {
        let (fake, seen) = Fake::new(scope_answers);
        let mut d = device(&QUICK_SCOPE, fake);
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let packets = packets_to_end(&start(&mut d));
        assert_eq!(values(&packets), [1.5, 1.5]);
        let w = writes(&seen);
        assert_eq!(w, [&b"QM\r"[..], b"QM 11\r", b"QM 11\r", b"QM 11\r"]);
        d.close().unwrap();
    }

    #[test]
    fn a_scope_keeps_the_display_in_view_while_it_reads() {
        let (fake, seen) = Fake::new(scope_answers);
        let mut d = device(&QUICK_SCOPE, fake);
        d.set(&Key::LimitMsec, &Value::U64(500), None).unwrap();
        packets_to_end(&start(&mut d));
        let w = writes(&seen);
        let metas = w.iter().filter(|w| w.as_slice() == b"QM\r").count();
        assert!((2..=4).contains(&metas), "{metas} of {}", w.len());
        d.close().unwrap();
    }

    fn overlong_then_good(n: usize, _: &[u8]) -> Vec<u8> {
        if n == 0 {
            let mut line = b"0\r".to_vec();
            // What follows the dropped part would read as a line of its own.
            line.extend(std::iter::repeat_n(b'x', LINE_MAX));
            line.extend_from_slice(b"9.5E0,VDC,NORMAL,NONE\r");
            line
        } else {
            b"0\r2.5E0,VDC,NORMAL,NONE\r".to_vec()
        }
    }

    #[test]
    fn a_line_too_long_is_dropped_and_the_next_read() {
        let (fake, seen) = Fake::new(overlong_then_good);
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let packets = packets_to_end(&start(&mut d));
        assert_eq!(values(&packets), [2.5]);
        assert_eq!(writes(&seen).len(), 2);
        d.close().unwrap();
    }

    fn one_past_the_longest(n: usize, _: &[u8]) -> Vec<u8> {
        if n > 0 {
            return b"0\r2.5E0,VDC,NORMAL,NONE\r".to_vec();
        }
        let mut line = b"0\r1.5E0,VDC,NORMAL,NONE,".to_vec();
        line.extend(std::iter::repeat_n(b'x', LINE_MAX - 22));
        line.push(b'\r');
        line
    }

    #[test]
    fn a_line_with_no_room_for_its_cr_is_dropped() {
        let (fake, _) = Fake::new(one_past_the_longest);
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let packets = packets_to_end(&start(&mut d));
        assert_eq!(values(&packets), [2.5]);
        d.close().unwrap();
    }

    fn longest_line(_: usize, _: &[u8]) -> Vec<u8> {
        let mut line = b"0\r1.5E0,VDC,NORMAL,NONE,".to_vec();
        line.extend(std::iter::repeat_n(b'x', LINE_MAX - 1 - 22));
        line.push(b'\r');
        line
    }

    #[test]
    fn the_longest_line_fits() {
        let (fake, _) = Fake::new(longest_line);
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let packets = packets_to_end(&start(&mut d));
        assert_eq!(values(&packets), [1.5]);
        d.close().unwrap();
    }

    #[test]
    fn a_capture_can_start_again_at_its_end() {
        let (fake, seen) = Fake::new(reading);
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        for round in 1..=3 {
            packets_to_end(&start(&mut d));
            assert_eq!(writes(&seen).len(), round);
        }
        d.close().unwrap();
    }

    /// The port goes back before the `End`, so a client that starts on seeing
    /// it is never told the device is busy. With a queue of one left unread
    /// the `End` cannot be sent: the port must be back all the same.
    #[test]
    fn the_port_goes_back_before_the_end() {
        let (fake, _) = Fake::new(reading);
        let mut d = device(&QUICK, fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        rx.recv_timeout(BOUND).unwrap();
        let deadline = Instant::now() + BOUND;
        while d.port.lock().unwrap().is_none() || d.capturing.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline, "the port did not come back");
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(matches!(rx.recv_timeout(BOUND), Ok((_, Packet::Analog(_)))));
        assert!(matches!(rx.recv_timeout(BOUND), Ok((_, Packet::End))));
        d.close().unwrap();
    }

    #[test]
    fn a_capture_refuses_a_second_at_once() {
        let (fake, _) = Fake::new(silent);
        let mut d = device(&QUICK, fake);
        let rx = start(&mut d);
        let (tx2, _rx2) = sync_channel(8);
        assert!(matches!(d.start(PacketSink::new(tx2, 0)), Err(Error::Busy)));
        d.stop().unwrap();
        packets_to_end(&rx);
        d.close().unwrap();
    }

    #[test]
    fn start_needs_an_open_port() {
        let (fake, _) = Fake::new(silent);
        let mut d = FlukeDevice::from_port(&QUICK, Box::new(fake));
        let (tx, _rx) = sync_channel(8);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
    }

    #[test]
    fn close_drops_the_port_and_a_close_during_a_capture_joins() {
        let (fake, seen) = Fake::new(silent);
        seen.lock().unwrap().open = true;
        let mut d = device(&QUICK, fake);
        let rx = start(&mut d);
        rx.recv_timeout(BOUND).unwrap();
        d.close().unwrap();
        assert!(!seen.lock().unwrap().open);
        assert!(matches!(rx.try_recv(), Ok((_, Packet::End))));
    }

    #[test]
    fn the_scan_tries_three_times_and_purges_each() {
        fn refuse(n: usize, _: &[u8]) -> Vec<u8> {
            if n < 2 { b"1\r".to_vec() } else { b"0\rFLUKE 287,V1.16,1\r".to_vec() }
        }
        let (mut fake, seen) = Fake::new(refuse);
        let identity = identify(&mut fake).unwrap();
        assert_eq!(identity.profile.name, "287");
        assert_eq!(identity.version, "V1.16");
        let seen = seen.lock().unwrap();
        assert_eq!(seen.writes.len(), 3);
        assert_eq!(seen.purges, 3);
    }

    #[test]
    fn the_scan_stops_after_three_and_goes_on_past_a_write_that_failed() {
        fn refuse_all(_: usize, _: &[u8]) -> Vec<u8> {
            b"1\r".to_vec()
        }
        let (mut fake, seen) = Fake::new(refuse_all);
        assert!(identify(&mut fake).is_none());
        assert_eq!(writes(&seen).len(), TRIES);
        let (mut fake, seen) = Fake::new(refuse_all);
        fake.fail_writes_from = Some(0);
        assert!(identify(&mut fake).is_none());
        assert_eq!(seen.lock().unwrap().purges, TRIES);
    }

    #[test]
    fn options_are_set_only() {
        let (fake, _) = Fake::new(silent);
        let mut d = FlukeDevice::from_port(&QUICK, Box::new(fake));
        let keys: Vec<_> = d.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
        assert_eq!(keys, [Key::Continuous, Key::LimitSamples, Key::LimitMsec]);
        assert!(d.get(&Key::LimitSamples, None).is_err());
        d.set(&Key::LimitMsec, &Value::U64(5), None).unwrap();
        assert!(d.set(&Key::Conn, &Value::from("x"), None).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::from("x"), None).is_err());
        assert!(d.options(Some("group")).is_err());
    }
}
