// Counterpart in sigrok:
//   libsigrok/src/hardware/cem-dt-885x/protocol.c,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Aurelien Jacobs, Daniel Elstner,
//   Frank Stettner, Martin Ling
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake meter. The fake lets its bytes out one per read when told
//! to, as a line at 9600 baud does, never by the clock.

use super::protocol::{
    CMD_TOGGLE_HOLD_MAX_MIN, CMD_TOGGLE_MEAS_RANGE, CMD_TOGGLE_POWER_OFF, CMD_TOGGLE_RECORDING,
    CMD_TOGGLE_WEIGHT_FREQ, CMD_TOGGLE_WEIGHT_TIME, Waited,
};
use super::*;
use crate::core::Result as CoreResult;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender, sync_channel};

/// How long a test waits for what must happen: a stop or a limit that does not
/// bite must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);

#[derive(Default)]
#[allow(clippy::struct_excessive_bools)]
struct Meter {
    /// What has arrived, ready to read.
    input: VecDeque<u8>,
    /// Said over and over when nothing has arrived.
    stream: Vec<u8>,
    /// Every command, in order.
    writes: Vec<u8>,
    /// Commands that go unheard, and whether a meter off is silent.
    deaf: u32,
    freq_c: bool,
    off: bool,
    /// What goes out on the first log request.
    log: Vec<u8>,
    /// Bytes per read: 0 for all there is.
    per_read: usize,
    /// Reads that find nothing between those that find something.
    gap: u32,
    gap_left: u32,
    reads: u64,
    /// Bytes read so far, so a test can wait for them.
    bytes_read: u64,
    read_error: bool,
    write_error: bool,
}

#[derive(Clone, Default)]
struct Fake(Arc<Mutex<Meter>>);

impl Fake {
    fn with(setup: impl FnOnce(&mut Meter)) -> Fake {
        let fake = Fake::default();
        // A poisoned lock is a failed test's: this must not panic again.
        setup(&mut fake.0.lock().unwrap());
        fake
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Meter> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn writes(&self) -> Vec<u8> {
        self.lock().writes.clone()
    }

    fn device(&self) -> CemDt885xDevice {
        let mut device = CemDt885xDevice::from_port(Box::new(self.clone()));
        // An open flushes: what a test put there comes after it.
        let input = std::mem::take(&mut self.lock().input);
        device.open().unwrap();
        self.lock().input = input;
        device
    }
}

impl Port for Fake {
    fn name(&self) -> &'static str {
        "/dev/ttyFAKE"
    }
    fn set_params(&mut self, _: Params) -> CoreResult<()> {
        Ok(())
    }
    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> CoreResult<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> CoreResult<usize> {
        let mut m = self.lock();
        m.reads += 1;
        if m.read_error {
            return Err(Error::device("gone"));
        }
        if m.gap_left > 0 {
            m.gap_left -= 1;
            drop(m);
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
            return Ok(0);
        }
        m.gap_left = m.gap;
        if m.input.is_empty() && !m.off {
            let stream = m.stream.clone();
            m.input.extend(stream);
        }
        let room = if m.per_read == 0 { buf.len() } else { m.per_read.min(buf.len()) };
        let n = m.input.len().min(room);
        if n == 0 {
            drop(m);
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
            return Ok(0);
        }
        for b in &mut buf[..n] {
            *b = m.input.pop_front().expect("counted");
        }
        m.bytes_read += n as u64;
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> CoreResult<()> {
        let mut m = self.lock();
        if m.write_error {
            return Err(Error::device("gone"));
        }
        for byte in data {
            m.writes.push(*byte);
            if m.off {
                continue;
            }
            if *byte == CMD_TRANSFER_MEMORY {
                let log = std::mem::take(&mut m.log);
                m.input.extend(log);
                continue;
            }
            if m.deaf > 0 {
                m.deaf -= 1;
            } else {
                match *byte {
                    CMD_TOGGLE_WEIGHT_FREQ => m.freq_c = !m.freq_c,
                    CMD_TOGGLE_POWER_OFF => {
                        m.off = true;
                        m.input.clear();
                    }
                    _ => {}
                }
            }
            if *byte == CMD_TOGGLE_WEIGHT_FREQ {
                let token = if m.freq_c { 0x1c } else { 0x1b };
                m.input.extend([0xa5, token, 0]);
            }
        }
        Ok(())
    }
    fn purge(&mut self) -> CoreResult<()> {
        self.lock().input.clear();
        Ok(())
    }
}

/// `level` in the stream's order, as a readout with nothing else.
fn readout(high: u8, low: u8) -> Vec<u8> {
    vec![0xa5, 0x0d, high, low, 0xa5, 0x0b, 0]
}

type Received = Receiver<(usize, Packet)>;

fn start(device: &mut CemDt885xDevice) -> Received {
    let (tx, rx) = sync_channel(1024);
    device.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// The packets up to the End, or a failure once [`BOUND`] is over.
fn until_end(rx: &Received) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut packets = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((_, Packet::End)) => return packets,
            Ok((_, packet)) => {
                // A limit that does not bite on a stream that never ends.
                assert!(packets.len() < 100_000, "no end in sight");
                packets.push(packet);
            }
            Err(RecvTimeoutError::Timeout) => panic!("no End within {BOUND:?}"),
            Err(RecvTimeoutError::Disconnected) => panic!("no End: the sender left"),
        }
    }
}

fn levels(packets: &[Packet]) -> Vec<f32> {
    packets
        .iter()
        .filter_map(|p| if let Packet::Analog(a) = p { Some(a.samples.clone()) } else { None })
        .flatten()
        .collect()
}

/// A level's flags.
fn first_flags(packets: &[Packet]) -> MqFlags {
    packets
        .iter()
        .find_map(|p| if let Packet::Analog(a) = p { Some(a.flags) } else { None })
        .expect("a level")
}

fn wait_for(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + BOUND;
    while !done() {
        assert!(Instant::now() < deadline, "{what}");
        std::thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn the_bounds_are_what_they_say() {
    assert_eq!(SILENCE_BOUND, Duration::from_secs(3));
    assert_eq!(SCAN_WINDOW, Duration::from_millis(250));
    assert_eq!(END_AFTER_STOP, Duration::from_millis(500));
}

#[test]
fn a_scan_finds_the_sync_byte_where_it_comes() {
    let mut fake = Fake::with(|m| {
        m.input.extend([1, 2, 3, 0xa5]);
        m.per_read = 1;
    });
    assert!(hears_a_meter(&mut fake, SCAN_WINDOW));
    assert_eq!(fake.lock().bytes_read, 4, "and reads no further");
}

#[test]
fn a_scan_of_no_sync_byte_is_no_meter_and_its_window_ends() {
    let mut fake = Fake::with(|m| m.stream = vec![0xff]);
    let started = Instant::now();
    assert!(!hears_a_meter(&mut fake, Duration::from_millis(30)));
    assert!(started.elapsed() >= Duration::from_millis(30));
    let mut silent = Fake::default();
    assert!(!hears_a_meter(&mut silent, Duration::from_millis(10)));
}

#[test]
fn a_scan_stops_on_an_error() {
    let mut fake = Fake::with(|m| m.read_error = true);
    assert!(!hears_a_meter(&mut fake, Duration::from_secs(60)));
    assert_eq!(fake.lock().reads, 1);
}

#[test]
fn the_options_are_cs_in_cs_order() {
    let device = Fake::default().device();
    let keys: Vec<Key> = device.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(
        keys,
        [
            Key::Continuous,
            Key::LimitSamples,
            Key::SplWeightFreq,
            Key::SplWeightTime,
            Key::SplMeasurementRange,
            Key::Datalog,
            Key::HoldMax,
            Key::HoldMin,
            Key::PowerOff,
            Key::DataSource,
        ]
    );
    assert!(device.options(Some("x")).is_err());
    assert_eq!(
        device.list(&Key::SplMeasurementRange, None).unwrap(),
        ValueList::RangeU64(vec![(30, 130), (30, 80), (50, 100), (80, 130)])
    );
    assert_eq!(device.get(&Key::PowerOff, None).unwrap(), Value::Bool(false));
    assert_eq!(device.get(&Key::DataSource, None).unwrap(), Value::from("Live"));
    assert_eq!(device.get(&Key::LimitSamples, None).unwrap(), Value::U64(0));
}

#[test]
fn a_get_waits_for_its_token_one_byte_at_a_time() {
    let fake = Fake::with(|m| {
        m.per_read = 1;
        m.input.extend([0xa5, 0x40, 0xa5, 0x1c, 0, 0xa5, 0x03, 0xa5, 0x1f]);
    });
    let device = fake.device();
    assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("C"));
    assert_eq!(device.get(&Key::SplWeightTime, None).unwrap(), Value::from("S"));
    assert_eq!(fake.lock().input.len(), 2, "no byte past the token");
    assert_eq!(fake.writes(), Vec::<u8>::new());
}

#[test]
fn each_state_is_read_from_its_token() {
    for (bytes, key, value) in [
        (vec![0xa5, 0x1b, 0], Key::SplWeightFreq, Value::from("A")),
        (vec![0xa5, 0x02], Key::SplWeightTime, Value::from("F")),
        (vec![0xa5, 0x0a], Key::Datalog, Value::Bool(true)),
        (vec![0xa5, 0x1a], Key::Datalog, Value::Bool(false)),
        (vec![0xa5, 0x40], Key::SplMeasurementRange, Value::RangeU64(30, 130)),
        (vec![0xa5, 0x30], Key::SplMeasurementRange, Value::RangeU64(30, 80)),
        (vec![0xa5, 0x4b], Key::SplMeasurementRange, Value::RangeU64(50, 100)),
        (vec![0xa5, 0x4c], Key::SplMeasurementRange, Value::RangeU64(80, 130)),
        (vec![0xa5, 0x04], Key::HoldMax, Value::Bool(true)),
        (vec![0xa5, 0x05], Key::HoldMax, Value::Bool(false)),
        (vec![0xa5, 0x0e], Key::HoldMax, Value::Bool(false)),
        (vec![0xa5, 0x05], Key::HoldMin, Value::Bool(true)),
        (vec![0xa5, 0x04], Key::HoldMin, Value::Bool(false)),
    ] {
        let device = Fake::with(|m| m.input.extend(bytes.iter())).device();
        assert_eq!(device.get(&key, None).unwrap(), value, "{bytes:02x?}");
    }
}

#[test]
fn a_range_and_a_hold_seen_are_not_waited_for_again() {
    let fake = Fake::with(|m| m.input.extend([0xa5, 0x4c, 0xa5, 0x04]));
    let device = fake.device();
    let value = Value::RangeU64(80, 130);
    assert_eq!(device.get(&Key::SplMeasurementRange, None).unwrap(), value);
    assert_eq!(device.get(&Key::SplMeasurementRange, None).unwrap(), value, "with no byte left");
    assert_eq!(device.get(&Key::HoldMax, None).unwrap(), Value::Bool(true));
    assert_eq!(device.get(&Key::HoldMax, None).unwrap(), Value::Bool(true), "with no byte left");
}

#[test]
fn a_wait_does_not_take_the_last_waits_token() {
    // Two gets, the second starting on the data of a packet: C's check of the token
    // after that byte would find the first get's.
    let fake = Fake::with(|m| m.input.extend([0xa5, 0x1c, 0, 0x55, 0xa5, 0x1b, 0]));
    let device = fake.device();
    assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("C"));
    assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("A"));
}

#[test]
fn a_silent_meter_fails_a_get_after_its_bound_and_not_before() {
    let device = Fake::default().device();
    device.link().silence = Duration::from_millis(40);
    // Not the wait's own bound that ends it.
    device.link().wait_bound = BOUND;
    let started = Instant::now();
    assert!(device.get(&Key::SplWeightFreq, None).is_err());
    assert!(started.elapsed() >= Duration::from_millis(40));
    assert!(started.elapsed() < BOUND);
    assert_eq!(protocol::WAIT_SILENCE, Duration::from_secs(1));
}

#[test]
fn a_byte_late_within_the_silence_is_still_read() {
    let fake = Fake::default();
    let device = fake.device();
    device.link().silence = Duration::from_secs(20);
    let late = fake.clone();
    let feeder = std::thread::spawn(move || {
        // A read has come up empty: only then does the meter speak.
        wait_for("reads", || late.lock().reads > 3);
        late.lock().input.extend([0xa5, 0x1c, 0]);
    });
    assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("C"));
    feeder.join().unwrap();
}

#[test]
fn a_token_that_never_comes_fails_a_get_after_its_bound() {
    let fake = Fake::with(|m| m.stream = vec![0xa5, 0x40]);
    let device = fake.device();
    device.link().wait_bound = Duration::from_millis(40);
    let started = Instant::now();
    assert!(device.get(&Key::SplWeightFreq, None).is_err());
    assert!(started.elapsed() >= Duration::from_millis(40));
}

#[test]
fn a_read_error_fails_a_get_at_once() {
    let device = Fake::with(|m| m.read_error = true).device();
    let started = Instant::now();
    assert!(device.get(&Key::SplWeightFreq, None).is_err());
    assert!(started.elapsed() < BOUND);
}

#[test]
fn a_set_of_what_the_meter_says_sends_nothing() {
    let fake = Fake::with(|m| m.input.extend([0xa5, 0x1b, 0]));
    let mut device = fake.device();
    device.set(&Key::SplWeightFreq, &Value::from("A"), None).unwrap();
    assert_eq!(fake.writes(), Vec::<u8>::new());
}

#[test]
fn a_set_toggles_until_the_token_says_so() {
    let fake = Fake::with(|m| {
        m.input.extend([0xa5, 0x1b, 0]);
        m.deaf = 2;
    });
    let mut device = fake.device();
    device.set(&Key::SplWeightFreq, &Value::from("C"), None).unwrap();
    assert_eq!(fake.writes(), [CMD_TOGGLE_WEIGHT_FREQ; 3]);
    assert!(fake.lock().freq_c);
}

#[test]
fn a_toggle_on_a_deaf_meter_fails_after_its_bound() {
    let fake = Fake::with(|m| {
        m.input.extend([0xa5, 0x1b, 0]);
        m.deaf = u32::MAX;
    });
    let mut device = fake.device();
    device.link().toggle_bound = Duration::from_millis(40);
    let started = Instant::now();
    assert!(device.set(&Key::SplWeightFreq, &Value::from("C"), None).is_err());
    assert!(started.elapsed() >= Duration::from_millis(40));
    assert!(fake.writes().len() > 1);
}

#[test]
fn what_came_before_a_command_is_not_its_answer() {
    // A burst of announcements of the old state already in the queue, as a capture
    // leaves. Reading one as an answer would toggle twice, and leave it where it was.
    let fake = Fake::with(|m| {
        m.per_read = 1;
        m.freq_c = false;
    });
    let mut device = fake.device();
    let rx = {
        fake.lock().input.extend([0xa5, 0x1b, 0]);
        fake.lock().input.extend(readout(0x06, 0x55));
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        start(&mut device)
    };
    until_end(&rx);
    fake.lock().input.extend([0xa5, 0x1b, 0, 0xa5, 0x1b, 0, 0xa5, 0x1b, 0]);
    device.set(&Key::SplWeightFreq, &Value::from("C"), None).unwrap();
    assert_eq!(fake.writes(), [CMD_TOGGLE_WEIGHT_FREQ]);
    assert!(fake.lock().freq_c);
}

/// A meter with each toggle's state on the next read: for the other settings.
fn toggle_test(start_bytes: &[u8], key: &Key, value: &Value, cmd: u8, next: &[u8]) {
    let fake = Fake::with(|m| m.input.extend(start_bytes.iter()));
    let mut device = fake.device();
    let feeder = {
        let fake = fake.clone();
        let next = next.to_vec();
        std::thread::spawn(move || {
            wait_for("the command", || !fake.writes().is_empty());
            fake.lock().input.extend(next);
        })
    };
    device.set(key, value, None).unwrap();
    feeder.join().unwrap();
    assert_eq!(fake.writes(), [cmd], "{key:?}");
}

#[test]
fn each_setting_has_its_command() {
    toggle_test(
        &[0xa5, 0x02],
        &Key::SplWeightTime,
        &Value::from("S"),
        CMD_TOGGLE_WEIGHT_TIME,
        &[0xa5, 0x03],
    );
    toggle_test(
        &[0xa5, 0x1a],
        &Key::Datalog,
        &Value::Bool(true),
        CMD_TOGGLE_RECORDING,
        &[0xa5, 0x0a],
    );
    toggle_test(
        &[0xa5, 0x0e],
        &Key::HoldMax,
        &Value::Bool(true),
        CMD_TOGGLE_HOLD_MAX_MIN,
        &[0xa5, 0x04],
    );
    toggle_test(
        &[0xa5, 0x04],
        &Key::HoldMin,
        &Value::Bool(false),
        CMD_TOGGLE_HOLD_MAX_MIN,
        &[0xa5, 0x0e],
    );
    toggle_test(
        &[0xa5, 0x40],
        &Key::SplMeasurementRange,
        &Value::RangeU64(30, 80),
        CMD_TOGGLE_MEAS_RANGE,
        &[0xa5, 0x30],
    );
}

#[test]
fn a_range_that_is_not_the_one_wanted_toggles_again_on_its_own_time() {
    let fake = Fake::with(|m| m.input.extend([0xa5, 0x40]));
    let mut device = fake.device();
    let feeder = {
        let fake = fake.clone();
        std::thread::spawn(move || {
            wait_for("the first", || !fake.writes().is_empty());
            fake.lock().input.extend([0xa5, 0x30]);
            wait_for("the second", || fake.writes().len() > 1);
            fake.lock().input.extend([0xa5, 0x4b]);
        })
    };
    device.link().silence = Duration::from_secs(20);
    // With no byte the wait can't run out, so feed one as the time is up: the other
    // range's announcement of the next block.
    let announcer = {
        let fake = fake.clone();
        std::thread::spawn(move || {
            wait_for("the first", || !fake.writes().is_empty());
            std::thread::sleep(Duration::from_millis(15));
            fake.lock().input.extend([0xa5, 0x30]);
        })
    };
    device.set(&Key::SplMeasurementRange, &Value::RangeU64(50, 100), None).unwrap();
    feeder.join().unwrap();
    announcer.join().unwrap();
    assert_eq!(fake.writes(), [CMD_TOGGLE_MEAS_RANGE; 2]);
    assert_eq!(
        device.get(&Key::SplMeasurementRange, None).unwrap(),
        Value::RangeU64(50, 100),
        "C's get would still say 30-130"
    );
}

#[test]
fn a_wait_with_a_time_ends_at_a_packet_past_it() {
    let fake = Fake::with(|m| m.stream = vec![0xa5, 0x40]);
    let device = fake.device();
    let waited = device.link().wait_for_token(&[0x1b], Some(Duration::from_millis(10))).unwrap();
    assert_eq!(waited, Waited::TimedOut);
}

#[test]
fn what_a_capture_saw_answers_and_a_set_keeps_it_true() {
    let mut stream = vec![0xa5, 0x0a, 0xa5, 0x1b, 0, 0xa5, 0x02, 0xa5, 0x40, 0xa5, 0x0e];
    stream.extend(readout(0x06, 0x55));
    let fake = Fake::with(|m| m.input.extend(stream));
    let mut device = fake.device();
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    let rx = start(&mut device);
    until_end(&rx);
    // Nothing is left to read, and no wait comes.
    fake.lock().input.clear();
    device.link().silence = Duration::from_millis(1);
    assert_eq!(
        device.get(&Key::Datalog, None).unwrap(),
        Value::Bool(true),
        "C reads the last token instead (protocol.c:487)"
    );
    assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("A"));
    assert_eq!(device.get(&Key::SplWeightTime, None).unwrap(), Value::from("F"));
    assert_eq!(device.get(&Key::HoldMax, None).unwrap(), Value::Bool(false));
    // A set of what it says sends nothing. C's flag would go stale after a toggle.
    device.set(&Key::SplWeightFreq, &Value::from("A"), None).unwrap();
    device.set(&Key::Datalog, &Value::Bool(true), None).unwrap();
    assert_eq!(fake.writes(), Vec::<u8>::new());
    device.link().silence = Duration::from_secs(20);
    device.set(&Key::SplWeightFreq, &Value::from("C"), None).unwrap();
    assert_eq!(fake.writes(), [CMD_TOGGLE_WEIGHT_FREQ]);
    assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("C"));
    fake.lock().writes.clear();
    device.set(&Key::SplWeightFreq, &Value::from("C"), None).unwrap();
    assert_eq!(fake.writes(), Vec::<u8>::new(), "C would toggle here");
}

#[test]
fn a_set_keeps_the_hold_and_recording_a_capture_saw_true() {
    let mut stream = vec![0xa5, 0x1a, 0xa5, 0x0e, 0xa5, 0x1b, 0];
    stream.extend(readout(0x06, 0x55));
    let fake = Fake::with(|m| m.input.extend(stream));
    let mut device = fake.device();
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    let rx = start(&mut device);
    until_end(&rx);
    fake.lock().input.clear();
    device.link().silence = Duration::from_secs(20);
    let feeder = {
        let fake = fake.clone();
        std::thread::spawn(move || {
            wait_for("the first", || !fake.writes().is_empty());
            fake.lock().input.extend([0xa5, 0x0a]);
            wait_for("the second", || fake.writes().len() > 1);
            fake.lock().input.extend([0xa5, 0x04]);
        })
    };
    device.set(&Key::Datalog, &Value::Bool(true), None).unwrap();
    device.set(&Key::HoldMax, &Value::Bool(true), None).unwrap();
    feeder.join().unwrap();
    assert_eq!(fake.writes(), [CMD_TOGGLE_RECORDING, CMD_TOGGLE_HOLD_MAX_MIN]);
    assert_eq!(device.get(&Key::Datalog, None).unwrap(), Value::Bool(true));
    assert_eq!(device.get(&Key::HoldMax, None).unwrap(), Value::Bool(true));
    assert_eq!(device.get(&Key::HoldMin, None).unwrap(), Value::Bool(false));
    // Sets of those send nothing: C's old state would toggle both again.
    device.set(&Key::Datalog, &Value::Bool(true), None).unwrap();
    device.set(&Key::HoldMax, &Value::Bool(true), None).unwrap();
    assert_eq!(fake.writes().len(), 2);
}

#[test]
fn a_bad_value_is_refused_and_sends_nothing() {
    let fake = Fake::with(|m| m.stream = vec![0xa5, 0x40]);
    let mut device = fake.device();
    for (key, value) in [
        (Key::SplWeightFreq, Value::from("Z")),
        (Key::SplWeightTime, Value::from("Z")),
        (Key::DataSource, Value::from("Cloud")),
        (Key::SplMeasurementRange, Value::RangeU64(10, 20)),
        (Key::SplMeasurementRange, Value::U64(1)),
        (Key::LimitSamples, Value::U64(0)),
        (Key::Datalog, Value::U64(1)),
        (Key::HoldMax, Value::U64(1)),
        (Key::PowerOff, Value::U64(1)),
    ] {
        assert!(device.set(&key, &value, None).is_err(), "{key:?}");
    }
    assert!(device.set(&Key::Conn, &Value::from("x"), None).is_err());
    assert!(device.get(&Key::Conn, None).is_err());
    assert!(device.list(&Key::Conn, None).is_err());
    assert_eq!(fake.writes(), Vec::<u8>::new());
}

#[test]
fn a_write_error_fails_a_set() {
    let fake = Fake::with(|m| {
        m.input.extend([0xa5, 0x1b, 0]);
        m.write_error = true;
    });
    let mut device = fake.device();
    assert!(device.set(&Key::SplWeightFreq, &Value::from("C"), None).is_err());
    assert!(device.set(&Key::PowerOff, &Value::Bool(true), None).is_err());
}

#[test]
fn power_off_says_it_until_the_meter_stops_and_off_says_nothing() {
    let fake = Fake::with(|m| {
        m.stream = vec![0xa5, 0x40];
        m.deaf = 2;
    });
    let mut device = fake.device();
    device.set(&Key::PowerOff, &Value::Bool(false), None).unwrap();
    assert_eq!(fake.writes(), Vec::<u8>::new());
    device.set(&Key::PowerOff, &Value::Bool(true), None).unwrap();
    assert_eq!(fake.writes(), [CMD_TOGGLE_POWER_OFF; 3]);
    assert!(fake.lock().off);
    assert!(device.link().port.is_some());
}

#[test]
fn power_off_of_a_meter_that_goes_on_fails_after_its_bound() {
    let fake = Fake::with(|m| {
        m.stream = vec![0xa5, 0x40];
        m.deaf = u32::MAX;
    });
    let mut device = fake.device();
    device.link().toggle_bound = Duration::from_secs(1);
    assert!(device.set(&Key::PowerOff, &Value::Bool(true), None).is_err());
    assert!(fake.writes().len() >= 2, "a slow sleep leaves room for more than one");
}

#[test]
fn a_capture_ends_at_its_limit_with_its_flags() {
    let mut stream = vec![0xa5, 0x1c, 0, 0xa5, 0x03];
    stream.extend(readout(0x06, 0x55));
    stream.extend(readout(0x12, 0x98));
    let fake = Fake::with(|m| m.stream = stream);
    let mut device = fake.device();
    device.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
    let rx = start(&mut device);
    let packets = until_end(&rx);
    assert!(matches!(packets[0], Packet::Header { .. }));
    assert_eq!(levels(&packets), [65.5, 129.8, 65.5]);
    assert_eq!(first_flags(&packets), MqFlags::SPL_FREQ_WEIGHT_C | MqFlags::SPL_TIME_WEIGHT_S);
    let Packet::Analog(a) = &packets[1] else { panic!("{:?}", packets[1]) };
    assert_eq!((a.mq, a.unit, a.digits), (Mq::SoundPressureLevel, Unit::DecibelSpl, 1));
    assert_eq!(fake.writes(), Vec::<u8>::new());
}

#[test]
fn a_stop_ends_a_capture_on_a_meter_that_goes_on() {
    let fake = Fake::with(|m| m.stream = readout(0x06, 0x55));
    let mut device = fake.device();
    let rx = start(&mut device);
    let deadline = Instant::now() + BOUND;
    let mut got = 0;
    while got < 3 {
        let (_, packet) =
            rx.recv_timeout(deadline.saturating_duration_since(Instant::now())).unwrap();
        if matches!(packet, Packet::Analog(_)) {
            got += 1;
        }
    }
    device.stop().unwrap();
    until_end(&rx);
    device.close().unwrap();
    assert!(device.link().port.is_none());
}

#[test]
fn a_full_queue_does_not_hold_a_close() {
    let fake = Fake::with(|m| m.stream = readout(0x06, 0x55));
    let mut device = fake.device();
    let (tx, _rx): (SyncSender<(usize, Packet)>, Received) = sync_channel(1);
    device.start(PacketSink::new(tx, 0)).unwrap();
    wait_for("levels in the way", || fake.lock().bytes_read > 0);
    let (done_tx, done_rx) = sync_channel(1);
    std::thread::spawn(move || {
        device.close().unwrap();
        let _ = done_tx.send(());
    });
    done_rx.recv_timeout(BOUND).expect("the close came back");
}

#[test]
fn a_close_joins_the_capture() {
    let fake = Fake::with(|m| m.stream = readout(0x06, 0x55));
    let mut device = fake.device();
    let rx = start(&mut device);
    wait_for("levels", || fake.lock().bytes_read > 0);
    device.close().unwrap();
    assert_eq!(Arc::strong_count(&device.link), 1, "the thread is gone");
    drop(rx);
}

#[test]
fn a_start_after_a_capture_whose_end_is_stuck_does_not_hang() {
    let fake = Fake::with(|m| m.stream = readout(0x06, 0x55));
    let mut device = fake.device();
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    // Room for the header and the level: the End waits, and no one has said stop.
    let (tx, stuck) = sync_channel(2);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let capturing = Arc::clone(&device.capturing);
    let (done_tx, done_rx) = sync_channel(1);
    std::thread::spawn(move || {
        wait_for("the capture over", || !capturing.load(Ordering::SeqCst));
        let rx = start(&mut device);
        let _ = done_tx.send(levels(&until_end(&rx)));
    });
    let second = done_rx.recv_timeout(BOUND).expect("the start came back");
    assert_eq!(second, [65.5], "a second capture ran");
    drop(stuck);
}

#[test]
fn a_second_capture_on_an_open_port_starts_clean_and_nothing_shifts() {
    // The first stops at its level and drops the rest of that read, which stops
    // inside a packet: what follows it is read by the second.
    let fake = Fake::with(|m| {
        m.input.extend(readout(0x06, 0x55));
        m.input.extend([0xa5, 0x0d, 0x07]); // Its last byte has not come.
    });
    let mut device = fake.device();
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    let rx = start(&mut device);
    assert_eq!(levels(&until_end(&rx)), [65.5]);
    {
        let mut m = fake.lock();
        m.input.extend([0x02, 0xa5, 0x0b, 0]); // The tail, and a display token.
        m.input.extend(readout(0x07, 0x03));
    }
    device.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    let rx = start(&mut device);
    // The tail is passed over, so the display token that follows has the last whole
    // level there was, and the next is right.
    assert_eq!(levels(&until_end(&rx)), [65.5, 70.3]);
}

#[test]
fn a_setting_right_after_a_capture_reads_on_from_where_it_stopped() {
    let fake = Fake::with(|m| {
        m.per_read = 1;
        m.input.extend(readout(0x06, 0x55));
        m.input.extend([0xa5, 0x0d, 0x07, 0x02, 0xa5, 0x0b, 0, 0xa5, 0x1c, 0]);
    });
    let mut device = fake.device();
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    let rx = start(&mut device);
    until_end(&rx);
    assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("C"));
}

#[test]
fn a_log_is_asked_for_once_per_byte_up_to_its_start_and_its_end_ends_the_capture() {
    let fake = Fake::with(|m| {
        m.per_read = 1;
        m.input.extend([0xa5, 0x40, 0xa5, 0x40]);
        m.log = vec![
            0xbb, 0, 112, 0xaa, 0x26, 0x11, 5, 0x12, 0x34, 0x56, 0x10, 0xac, 0x04, 0x00, 0x04,
            0x01, 0x45, 0xdd, 0xa5, 0x40, 0xa5, 0x0d, 0x06, 0x55,
        ];
    });
    let mut device = fake.device();
    device.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
    assert_eq!(device.get(&Key::DataSource, None).unwrap(), Value::from("Memory"));
    let rx = start(&mut device);
    let packets = until_end(&rx);
    assert_eq!(levels(&packets), [40.0, 40.1]);
    assert_eq!(first_flags(&packets), MqFlags::SPL_FREQ_WEIGHT_A);
    assert!(packets.iter().any(|p| {
        matches!(p, Packet::Meta(m) if m[0] == (Key::SampleInterval, Value::U64(16_000)))
    }));
    assert_eq!(fake.writes(), [CMD_TRANSFER_MEMORY; 4], "one per byte, none for the bb");
    assert_ne!(fake.lock().input.len(), 0, "nothing past the end is read");
}

#[test]
fn a_second_capture_of_the_log_asks_again() {
    let fake = Fake::with(|m| {
        m.per_read = 1;
        m.stream = vec![0xa5, 0x40];
        m.log = vec![0xbb, 0, 100, 0xac, 0x05, 0x00, 0xdd];
    });
    let mut device = fake.device();
    device.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
    let rx = start(&mut device);
    assert_eq!(levels(&until_end(&rx)), [50.0]);
    fake.lock().log = vec![0xbb, 0, 100, 0xac, 0x06, 0x00, 0xdd];
    fake.lock().input.clear();
    fake.lock().writes.clear();
    let rx = start(&mut device);
    assert_eq!(levels(&until_end(&rx)), [60.0], "C goes on with live levels here");
    assert_ne!(fake.writes().len(), 0);
    // Back to Live: nothing asked.
    fake.lock().writes.clear();
    device.set(&Key::DataSource, &Value::from("Live"), None).unwrap();
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    fake.lock().input.extend(readout(0x07, 0x00));
    let rx = start(&mut device);
    assert_eq!(levels(&until_end(&rx)), [70.0]);
    assert_eq!(fake.writes(), Vec::<u8>::new());
}

#[test]
fn a_log_is_cut_at_the_limit_inside_a_packet() {
    let mut log = vec![0xbb, 0, 100, 0xac];
    log.extend([0x05, 0x00].repeat(300));
    log.push(0xdd);
    let fake = Fake::with(|m| {
        m.input.extend([0xa5, 0x40]);
        m.log = log;
    });
    let mut device = fake.device();
    device.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
    device.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
    let rx = start(&mut device);
    assert_eq!(levels(&until_end(&rx)).len(), 3);
    assert_ne!(fake.lock().input.len(), 0, "it stops reading at the packet that held it");
}

#[test]
fn asking_for_the_log_stops_with_the_capture() {
    // No bb ever: the limit ends it, and each byte up to there asked.
    let fake = Fake::with(|m| {
        m.per_read = 1;
        m.input.extend(readout(0x06, 0x55));
        m.input.extend(readout(0x06, 0x55));
    });
    let mut device = fake.device();
    device.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    let rx = start(&mut device);
    until_end(&rx);
    assert_eq!(fake.writes().len(), 7, "the byte that ended it asks too");
}

#[test]
fn a_write_error_while_asking_does_not_end_the_capture() {
    let fake = Fake::with(|m| {
        m.write_error = true;
        m.stream = readout(0x06, 0x55);
    });
    let mut device = fake.device();
    device.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
    device.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    let rx = start(&mut device);
    assert_eq!(levels(&until_end(&rx)), [65.5, 65.5]);
}

#[test]
fn silence_ends_a_limited_capture_after_its_bound_and_not_before() {
    let fake = Fake::default();
    let mut device = fake.device();
    device.silence_bound = Duration::from_millis(50);
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    let started = Instant::now();
    let rx = start(&mut device);
    assert_eq!(levels(&until_end(&rx)), Vec::<f32>::new());
    assert!(started.elapsed() >= Duration::from_millis(50));
}

#[test]
fn bytes_with_no_level_keep_a_limited_capture_going() {
    let fake = Fake::with(|m| m.stream = vec![0xa5, 0x40]);
    let mut device = fake.device();
    device.silence_bound = Duration::from_millis(5);
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    let rx = start(&mut device);
    std::thread::sleep(Duration::from_millis(60));
    assert!(device.capturing.load(Ordering::SeqCst), "still going past the bound");
    device.stop().unwrap();
    until_end(&rx);
}

#[test]
fn only_a_gap_with_no_byte_counts_as_silence() {
    // Each read that finds bytes starts the bound again, and they are never far apart.
    let fake = Fake::with(|m| {
        m.stream = vec![0xa5, 0x40];
        m.gap = 3;
    });
    let mut device = fake.device();
    device.silence_bound = Duration::from_millis(500);
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    let rx = start(&mut device);
    std::thread::sleep(Duration::from_millis(1500));
    assert!(device.capturing.load(Ordering::SeqCst), "still going past the bound");
    device.stop().unwrap();
    until_end(&rx);
}

#[test]
fn silence_does_not_end_a_capture_with_no_limit() {
    let fake = Fake::default();
    let mut device = fake.device();
    device.silence_bound = Duration::from_millis(5);
    let rx = start(&mut device);
    std::thread::sleep(Duration::from_millis(60));
    assert!(device.capturing.load(Ordering::SeqCst), "a stop ends this");
    device.stop().unwrap();
    until_end(&rx);
}

#[test]
fn silence_ends_a_capture_of_the_log() {
    let fake = Fake::default();
    let mut device = fake.device();
    device.silence_bound = Duration::from_millis(5);
    device.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
    let rx = start(&mut device);
    until_end(&rx);
}

#[test]
fn a_read_error_ends_a_capture() {
    let fake = Fake::with(|m| m.read_error = true);
    let mut device = fake.device();
    let rx = start(&mut device);
    assert_eq!(levels(&until_end(&rx)), Vec::<f32>::new());
}

#[test]
fn settings_and_reads_of_the_port_wait_for_the_capture() {
    let fake = Fake::with(|m| m.stream = vec![0xa5, 0x40]);
    let mut device = fake.device();
    let rx = start(&mut device);
    wait_for("the capture", || device.capturing.load(Ordering::SeqCst));
    for key in [Key::SplWeightFreq, Key::SplWeightTime, Key::SplMeasurementRange, Key::Datalog] {
        assert!(device.get(&key, None).is_err(), "{key:?}");
    }
    assert!(device.get(&Key::HoldMax, None).is_err());
    assert!(device.get(&Key::HoldMin, None).is_err());
    assert_eq!(device.get(&Key::LimitSamples, None).unwrap(), Value::U64(0));
    assert!(device.set(&Key::SplWeightFreq, &Value::from("C"), None).is_err());
    assert!(device.set(&Key::LimitSamples, &Value::U64(1), None).is_err());
    let (tx, _other) = sync_channel(1);
    assert!(matches!(device.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
    device.stop().unwrap();
    until_end(&rx);
    assert_eq!(fake.writes(), Vec::<u8>::new());
}

#[test]
fn no_start_without_an_open_and_a_disabled_channel_sends_no_level() {
    let fake = Fake::with(|m| m.stream = readout(0x06, 0x55));
    let mut device = CemDt885xDevice::from_port(Box::new(fake.clone()));
    let (tx, _rx) = sync_channel(4);
    assert!(device.start(PacketSink::new(tx, 0)).is_err());
    device.open().unwrap();
    device.enable_channel(0, false).unwrap();
    device.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    let rx = start(&mut device);
    assert_eq!(levels(&until_end(&rx)), Vec::<f32>::new(), "and the limit still counts");
}
