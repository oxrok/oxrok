// Counterpart in sigrok:
//   libsigrok/src/hardware/cem-dt-885x/protocol.c,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Martin Ling, Soeren Apel, Daniel Elstner,
//   Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on two fake meters. [`Fake`] counts: it lets bytes out as it is read,
//! never by the clock. [`Clocked`] is a line at 9600 baud with blocks at 20 Hz, for
//! the one thing that needs a clock: a meter that takes a command while it talks.

use super::protocol::{
    CMD_TOGGLE_HOLD_MAX_MIN, CMD_TOGGLE_MEAS_RANGE, CMD_TOGGLE_POWER_OFF, CMD_TOGGLE_RECORDING,
    CMD_TOGGLE_WEIGHT_FREQ, CMD_TOGGLE_WEIGHT_TIME,
};
use super::*;
use crate::core::Result as CoreResult;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender, sync_channel};

/// How long a test waits for what must happen: a stop or a limit that does not
/// bite must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);

/// A test on its own thread with a bound, as a capture that doesn't stop also holds
/// its device's drop. The thread is left behind if it hangs.
macro_rules! bounded_test {
    ($(#[$meta:meta])* fn $name:ident() $body:block) => {
        #[test]
        $(#[$meta])*
        fn $name() {
            within(|| $body);
        }
    };
}

fn within(test: impl FnOnce() + Send + 'static) {
    let (done, finished) = sync_channel(1);
    let thread = std::thread::spawn(move || {
        test();
        let _ = done.send(());
    });
    match finished.recv_timeout(BOUND * 3) {
        Ok(()) => {}
        Err(RecvTimeoutError::Timeout) => panic!("the test hung"),
        Err(RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(thread.join().expect_err("it did not finish"));
        }
    }
}

#[derive(Default)]
#[allow(clippy::struct_excessive_bools)]
struct Meter {
    /// What has arrived, ready to read.
    input: VecDeque<u8>,
    /// Said over and over when nothing has arrived.
    stream: Vec<u8>,
    /// Says the state, as it is, when nothing has arrived.
    live: bool,
    /// Every command, in order.
    writes: Vec<u8>,
    /// Commands that go unheard.
    deaf: u32,
    freq_c: bool,
    slow: bool,
    recording: bool,
    hold: usize,
    range: usize,
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
    /// The longest time between two reads: a stall of the host's.
    last_read: Option<Instant>,
    longest_gap: Duration,
    read_error: bool,
    write_error: bool,
    /// `write_timeout` takes its time and sends nothing.
    line_full: bool,
    plain_writes: u32,
    /// `write_timeouts`, sent or not.
    asks: u32,
}

const HOLDS: [u8; 3] = [0x0e, 0x04, 0x05];
const RANGE_TOKENS: [u8; 4] = [0x40, 0x30, 0x4b, 0x4c];

impl Meter {
    fn say_state(&mut self) {
        let block = [
            0xa5,
            RANGE_TOKENS[self.range],
            0xa5,
            HOLDS[self.hold],
            0xa5,
            if self.recording { 0x0a } else { 0x1a },
            0xa5,
            if self.slow { 0x03 } else { 0x02 },
            0xa5,
            if self.freq_c { 0x1c } else { 0x1b },
            0,
        ];
        self.input.extend(block);
    }

    fn command(&mut self, byte: u8) {
        // A bound that doesn't bite must not eat the memory first.
        if self.writes.len() < 100_000 {
            self.writes.push(byte);
        }
        if self.off {
            return;
        }
        if byte == CMD_TRANSFER_MEMORY {
            let log = std::mem::take(&mut self.log);
            self.input.extend(log);
            return;
        }
        if self.deaf > 0 {
            self.deaf -= 1;
            return;
        }
        match byte {
            CMD_TOGGLE_WEIGHT_FREQ => self.freq_c = !self.freq_c,
            CMD_TOGGLE_WEIGHT_TIME => self.slow = !self.slow,
            CMD_TOGGLE_RECORDING => self.recording = !self.recording,
            CMD_TOGGLE_HOLD_MAX_MIN => self.hold = (self.hold + 1) % HOLDS.len(),
            CMD_TOGGLE_MEAS_RANGE => self.range = (self.range + 1) % RANGE_TOKENS.len(),
            CMD_TOGGLE_POWER_OFF => {
                self.off = true;
                self.live = false;
                self.stream.clear();
            }
            _ => {}
        }
    }
}

#[derive(Clone, Default)]
struct Fake(Arc<Mutex<Meter>>);

impl Fake {
    fn with(setup: impl FnOnce(&mut Meter)) -> Fake {
        let fake = Fake::default();
        setup(&mut fake.lock());
        fake
    }

    fn live() -> Fake {
        Fake::with(|m| m.live = true)
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Meter> {
        // A failed test's lock must not panic again in a drop.
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
        let now = Instant::now();
        if let Some(last) = m.last_read {
            m.longest_gap = m.longest_gap.max(now - last);
        }
        m.last_read = Some(now);
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
            if m.live {
                m.say_state();
            }
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
        m.plain_writes += 1;
        if m.write_error {
            return Err(Error::device("gone"));
        }
        for byte in data {
            m.command(*byte);
        }
        Ok(())
    }
    fn write_timeout(&mut self, data: &[u8], timeout: Duration) -> CoreResult<usize> {
        let mut m = self.lock();
        m.asks += 1;
        if m.write_error {
            return Err(Error::device("gone"));
        }
        if m.line_full {
            drop(m);
            std::thread::sleep(timeout);
            return Ok(0);
        }
        for byte in data {
            m.command(*byte);
        }
        Ok(data.len())
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

/// Why the capture failed, from its `Packet::Failed`.
fn failure(packets: &[Packet]) -> Option<String> {
    packets.iter().find_map(|p| if let Packet::Failed(why) = p { Some(why.clone()) } else { None })
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

fn error_text(result: CoreResult<impl std::fmt::Debug>) -> String {
    result.expect_err("it should have failed").to_string()
}

bounded_test! {
    fn the_bounds_are_what_they_say() {
        assert_eq!(SILENCE_BOUND, Duration::from_secs(3));
        assert_eq!(LEVEL_BOUND, Duration::from_secs(5));
        assert_eq!(SCAN_WINDOW, Duration::from_millis(500));
        assert_eq!(END_AFTER_STOP, Duration::from_millis(500));
        assert_eq!(ASK_WAIT, Duration::from_millis(100));
    }
}

bounded_test! {
    fn a_scan_finds_the_sync_byte_where_it_comes() {
        let mut fake = Fake::with(|m| {
            m.input.extend([1, 2, 3, 0xa5]);
            m.per_read = 1;
        });
        assert!(hears_a_meter(&mut fake, SCAN_WINDOW));
        assert_eq!(fake.lock().bytes_read, 4, "and reads no further");
    }
}

bounded_test! {
    fn a_scan_of_no_sync_byte_is_no_meter_and_its_window_ends() {
        let mut fake = Fake::with(|m| m.stream = vec![0xff]);
        let started = Instant::now();
        assert!(!hears_a_meter(&mut fake, Duration::from_millis(30)));
        let took = started.elapsed();
        assert!(took >= Duration::from_millis(30));
        assert!(took < Duration::from_secs(20), "{took:?}");
        let mut silent = Fake::default();
        assert!(!hears_a_meter(&mut silent, Duration::from_millis(10)));
    }
}

bounded_test! {
    fn a_scan_stops_on_an_error() {
        let mut fake = Fake::with(|m| m.read_error = true);
        assert!(!hears_a_meter(&mut fake, Duration::from_secs(60)));
        assert_eq!(fake.lock().reads, 1);
    }
}

bounded_test! {
    fn an_open_drops_what_is_queued() {
        let fake = Fake::with(|m| m.input.extend([1, 2, 3]));
        let mut device = CemDt885xDevice::from_port(Box::new(fake.clone()));
        device.open().unwrap();
        assert_eq!(fake.lock().input.len(), 0);
    }
}

bounded_test! {
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
}

bounded_test! {
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
}

bounded_test! {
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
}

bounded_test! {
    fn a_range_and_a_hold_seen_are_not_waited_for_again() {
        let fake = Fake::with(|m| m.input.extend([0xa5, 0x4c, 0xa5, 0x04]));
        let device = fake.device();
        let value = Value::RangeU64(80, 130);
        assert_eq!(device.get(&Key::SplMeasurementRange, None).unwrap(), value);
        assert_eq!(device.get(&Key::SplMeasurementRange, None).unwrap(), value, "with no byte left");
        assert_eq!(device.get(&Key::HoldMax, None).unwrap(), Value::Bool(true));
        assert_eq!(device.get(&Key::HoldMax, None).unwrap(), Value::Bool(true), "with no byte left");
    }
}

bounded_test! {
    fn a_wait_does_not_take_the_last_waits_token() {
        // Two gets, the second starting on the data of a packet: C's check of the token
        // after that byte would find the first get's.
        let fake = Fake::with(|m| m.input.extend([0xa5, 0x1c, 0, 0x55, 0xa5, 0x1b, 0]));
        let device = fake.device();
        assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("C"));
        assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("A"));
    }
}

bounded_test! {
    fn a_wait_starts_clean_after_one_that_ended_inside_a_packet() {
        // A silent end just after an a5: the next byte must not be read as its token.
        let fake = Fake::with(|m| m.input.extend([0xa5]));
        let device = fake.device();
        device.link().silence = Duration::from_millis(20);
        assert!(device.get(&Key::SplWeightFreq, None).is_err());
        fake.lock().input.extend([0x1c, 0, 0xa5, 0x1b, 0]);
        assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("A"));
    }
}

bounded_test! {
    fn a_silent_meter_fails_a_get_after_its_bound_and_not_before() {
        let device = Fake::default().device();
        device.link().silence = Duration::from_millis(40);
        // Not the wait's own bound that ends it.
        device.link().wait_bound = BOUND;
        let started = Instant::now();
        assert_eq!(
            error_text(device.get(&Key::SplWeightFreq, None)),
            "device error: no byte from the meter in 40 ms"
        );
        assert!(started.elapsed() >= Duration::from_millis(40));
        assert!(started.elapsed() < BOUND);
        assert_eq!(protocol::WAIT_SILENCE, Duration::from_secs(1));
    }
}

bounded_test! {
    fn each_byte_starts_the_silence_again() {
        // Junk far apart, for longer than the bound, and then the token.
        let fake = Fake::with(|m| {
            m.gap = 5;
            m.input.extend([0x00; 40]);
            m.input.extend([0xa5, 0x1c, 0]);
        });
        let device = fake.device();
        device.link().silence = Duration::from_millis(200);
        device.link().wait_bound = BOUND;
        let got = device.get(&Key::SplWeightFreq, None);
        if got.is_err() {
            // A stall of the host's is silence too.
            assert!(fake.lock().longest_gap >= Duration::from_millis(150), "{got:?}");
        }
    }
}

bounded_test! {
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
}

bounded_test! {
    fn a_token_that_never_comes_fails_a_get_after_its_bound() {
        let fake = Fake::with(|m| m.stream = vec![0xa5, 0x40]);
        let device = fake.device();
        device.link().wait_bound = Duration::from_millis(40);
        let started = Instant::now();
        assert_eq!(
            error_text(device.get(&Key::SplWeightFreq, None)),
            "device error: the meter did not say the frequency weighting in 40 ms"
        );
        assert!(started.elapsed() >= Duration::from_millis(40));
    }
}

bounded_test! {
    fn a_silent_meter_ends_at_the_waits_bound_when_that_comes_first() {
        let device = Fake::default().device();
        device.link().silence = BOUND;
        device.link().wait_bound = Duration::from_millis(30);
        let started = Instant::now();
        assert!(device.get(&Key::SplWeightFreq, None).is_err());
        assert!(started.elapsed() < Duration::from_secs(20), "bytes never came to look at it");
    }
}

bounded_test! {
    fn a_recording_get_and_a_range_set_have_times_of_their_own() {
        // The long block's token and the range's, against a wait with room for far more.
        let fake = Fake::with(|m| m.stream = vec![0xa5, 0x1f]);
        let mut device = fake.device();
        device.link().wait_bound = BOUND;
        let started = Instant::now();
        assert_eq!(
            error_text(device.get(&Key::Datalog, None)),
            "device error: the meter did not say the recording state in 1200 ms"
        );
        assert!(started.elapsed() >= Duration::from_millis(1200));
        assert!(started.elapsed() < Duration::from_secs(20));
        let started = Instant::now();
        assert_eq!(
            error_text(device.set(&Key::SplMeasurementRange, &Value::RangeU64(30, 80), None)),
            "device error: the meter did not say the range in 600 ms"
        );
        assert!(started.elapsed() >= Duration::from_millis(600));
        assert!(started.elapsed() < Duration::from_secs(20));
    }
}

bounded_test! {
    fn a_stray_log_start_does_not_outlast_any_bound() {
        // bb and no dd: the parser never comes back to where C looks at its time.
        let with_log = |wanted: &[u8]| {
            let fake = Fake::with(|m| {
                m.input.extend(wanted);
                m.input.extend([0xbb, 0, 0x64]);
                m.stream = vec![0x12];
            });
            let device = fake.device();
            device.link().announce = Some(Duration::from_millis(30));
            (fake, device)
        };
        let (_fake, device) = with_log(&[]);
        device.link().wait_bound = Duration::from_millis(30);
        assert!(device.get(&Key::SplWeightFreq, None).is_err());
        let (fake, mut device) = with_log(&[0xa5, 0x1b, 0]);
        assert_eq!(
            error_text(device.set(&Key::SplWeightFreq, &Value::from("C"), None)),
            "device error: the meter did not say the frequency weighting in 30 ms; its state is \
             unknown"
        );
        assert_eq!(fake.writes(), [CMD_TOGGLE_WEIGHT_FREQ], "no second command in the dark");
    }
}

bounded_test! {
    fn a_read_error_fails_a_get_at_once() {
        let device = Fake::with(|m| m.read_error = true).device();
        let started = Instant::now();
        assert!(device.get(&Key::SplWeightFreq, None).is_err());
        assert!(started.elapsed() < BOUND);
    }
}

bounded_test! {
    fn a_set_of_what_the_meter_says_sends_nothing() {
        let fake = Fake::with(|m| m.input.extend([0xa5, 0x1b, 0]));
        let mut device = fake.device();
        device.set(&Key::SplWeightFreq, &Value::from("A"), None).unwrap();
        assert_eq!(fake.writes(), Vec::<u8>::new());
    }
}

bounded_test! {
    fn a_set_sends_once_per_announcement_until_it_is_heard() {
        let fake = Fake::with(|m| {
            m.live = true;
            m.per_read = 1;
            m.deaf = 2;
        });
        let mut device = fake.device();
        device.set(&Key::SplWeightFreq, &Value::from("C"), None).unwrap();
        assert_eq!(fake.writes(), [CMD_TOGGLE_WEIGHT_FREQ; 3]);
        assert!(fake.lock().freq_c);
    }
}

bounded_test! {
    fn each_setting_has_its_command_and_ends_in_the_state_asked() {
        // Each with what stepping it takes on a meter that starts at its first state.
        let cases = [
            (Key::SplWeightFreq, Value::from("C"), vec![CMD_TOGGLE_WEIGHT_FREQ]),
            (Key::SplWeightTime, Value::from("S"), vec![CMD_TOGGLE_WEIGHT_TIME]),
            (Key::Datalog, Value::Bool(true), vec![CMD_TOGGLE_RECORDING]),
            (Key::HoldMax, Value::Bool(true), vec![CMD_TOGGLE_HOLD_MAX_MIN]),
            (Key::HoldMin, Value::Bool(true), vec![CMD_TOGGLE_HOLD_MAX_MIN; 2]),
            (Key::SplMeasurementRange, Value::RangeU64(30, 80), vec![CMD_TOGGLE_MEAS_RANGE]),
            (Key::SplMeasurementRange, Value::RangeU64(50, 100), vec![CMD_TOGGLE_MEAS_RANGE; 2]),
            (Key::SplMeasurementRange, Value::RangeU64(80, 130), vec![CMD_TOGGLE_MEAS_RANGE; 3]),
        ];
        for (key, value, commands) in cases {
            let fake = Fake::live();
            let mut device = fake.device();
            device.set(&key, &value, None).unwrap();
            assert_eq!(fake.writes(), commands, "{key:?}");
            let m = fake.lock();
            let right = match key {
                Key::SplWeightFreq => m.freq_c,
                Key::SplWeightTime => m.slow,
                Key::Datalog => m.recording,
                Key::HoldMax => m.hold == 1,
                Key::HoldMin => m.hold == 2,
                _ => m.range == commands.len(),
            };
            assert!(right, "{key:?}");
        }
        // Off from max: on through min.
        let fake = Fake::with(|m| {
            m.live = true;
            m.hold = 1;
        });
        fake.device().set(&Key::HoldMax, &Value::Bool(false), None).unwrap();
        assert_eq!(fake.writes(), [CMD_TOGGLE_HOLD_MAX_MIN; 2]);
        assert_eq!(fake.lock().hold, 0);
    }
}

bounded_test! {
    fn a_token_that_says_it_is_believed_only_when_the_next_agrees() {
        // A command the meter takes late: the first says yes, the next says no, and so a
        // second command, itself confirmed.
        let fake = Fake::default();
        let mut device = fake.device();
        fake.lock().input.extend([0xa5, 0x1b, 0]);
        let feeder = {
            let fake = fake.clone();
            std::thread::spawn(move || {
                wait_for("the first", || !fake.writes().is_empty());
                std::thread::sleep(protocol::PACKET_TIME * 2);
                fake.lock().input.extend([0xa5, 0x1c, 0, 0xa5, 0x1b, 0]);
                wait_for("the second", || fake.writes().len() > 1);
                std::thread::sleep(protocol::PACKET_TIME * 2);
                fake.lock().input.extend([0xa5, 0x1c, 0, 0xa5, 0x1c, 0]);
            })
        };
        device.link().silence = BOUND;
        device.set(&Key::SplWeightFreq, &Value::from("C"), None).unwrap();
        feeder.join().unwrap();
        assert_eq!(fake.writes(), [CMD_TOGGLE_WEIGHT_FREQ; 2]);
    }
}

bounded_test! {
    fn no_next_announcement_is_an_error_that_says_the_state_is_unknown() {
        let fake = Fake::default();
        let mut device = fake.device();
        fake.lock().input.extend([0xa5, 0x1b, 0]);
        let feeder = {
            let fake = fake.clone();
            std::thread::spawn(move || {
                wait_for("the first", || !fake.writes().is_empty());
                std::thread::sleep(protocol::PACKET_TIME * 2);
                fake.lock().input.extend([0xa5, 0x1c, 0]);
            })
        };
        device.link().silence = Duration::from_millis(300);
        device.link().announce = Some(BOUND);
        assert_eq!(
            error_text(device.set(&Key::SplWeightFreq, &Value::from("C"), None)),
            "device error: no byte from the meter in 300 ms; its state is unknown"
        );
        feeder.join().unwrap();
        assert_eq!(fake.writes(), [CMD_TOGGLE_WEIGHT_FREQ], "stopped sending once it said so");
    }
}

bounded_test! {
    fn what_completes_within_a_packets_time_of_a_command_is_no_answer() {
        // The old state, already on its way, right behind the command.
        let fake = Fake::with(|m| {
            m.live = true;
            m.per_read = 1;
        });
        let mut device = fake.device();
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        fake.lock().input.extend(readout(0x06, 0x55));
        let rx = start(&mut device);
        until_end(&rx);
        {
            let mut m = fake.lock();
            m.input.clear();
            // A set from what the capture saw goes straight to its command.
            m.input.extend([0xa5, 0x1b, 0, 0xa5, 0x1b, 0, 0xa5, 0x1b, 0]);
        }
        device.link().parser.flags = MqFlags::SPL_FREQ_WEIGHT_A;
        device.set(&Key::SplWeightFreq, &Value::from("C"), None).unwrap();
        assert_eq!(fake.writes(), [CMD_TOGGLE_WEIGHT_FREQ]);
        assert!(fake.lock().freq_c);
    }
}

bounded_test! {
    fn a_toggle_on_a_deaf_meter_fails_after_its_bound() {
        let fake = Fake::with(|m| {
            m.live = true;
            m.deaf = u32::MAX;
        });
        let mut device = fake.device();
        device.link().toggle_bound = Some(Duration::from_millis(40));
        let started = Instant::now();
        assert_eq!(
            error_text(device.set(&Key::SplWeightFreq, &Value::from("C"), None)),
            "device error: the meter did not take the command in 40 ms; its state is unknown"
        );
        assert!(started.elapsed() >= Duration::from_millis(40));
        assert!(fake.writes().len() > 1);
    }
}

bounded_test! {
    fn a_failed_set_leaves_what_a_capture_saw() {
        let mut stream = vec![0xa5, 0x1a, 0xa5, 0x4c, 0xa5, 0x0e, 0xa5, 0x1b, 0];
        stream.extend(readout(0x06, 0x55));
        let fake = Fake::with(|m| m.input.extend(stream));
        let mut device = fake.device();
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut device);
        until_end(&rx);
        {
            let mut m = fake.lock();
            m.deaf = u32::MAX;
            m.input.clear();
            m.stream = vec![0xa5, 0x1a, 0xa5, 0x4c, 0xa5, 0x0e, 0xa5, 0x1b, 0];
        }
        device.link().toggle_bound = Some(Duration::from_millis(1));
        device.link().announce = Some(Duration::from_millis(30));
        for (key, value) in [
            (Key::SplWeightFreq, Value::from("C")),
            (Key::Datalog, Value::Bool(true)),
            (Key::HoldMax, Value::Bool(true)),
            (Key::SplMeasurementRange, Value::RangeU64(30, 80)),
        ] {
            assert!(device.set(&key, &value, None).is_err(), "{key:?}");
        }
        assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("A"));
        assert_eq!(device.get(&Key::Datalog, None).unwrap(), Value::Bool(false));
        assert_eq!(device.get(&Key::HoldMax, None).unwrap(), Value::Bool(false));
        assert_eq!(device.get(&Key::SplMeasurementRange, None).unwrap(), Value::RangeU64(80, 130));
    }
}

bounded_test! {
    fn what_a_capture_saw_answers_and_a_set_keeps_it_true() {
        let mut stream = vec![0xa5, 0x0a, 0xa5, 0x1b, 0, 0xa5, 0x02, 0xa5, 0x40, 0xa5, 0x0e];
        stream.extend(readout(0x06, 0x55));
        let fake = Fake::with(|m| {
            m.input.extend(stream);
            m.recording = true;
        });
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
        device.link().silence = BOUND;
        fake.lock().live = true;
        device.set(&Key::SplWeightFreq, &Value::from("C"), None).unwrap();
        assert_eq!(fake.writes(), [CMD_TOGGLE_WEIGHT_FREQ]);
        assert_eq!(device.get(&Key::SplWeightFreq, None).unwrap(), Value::from("C"));
        fake.lock().writes.clear();
        device.set(&Key::SplWeightFreq, &Value::from("C"), None).unwrap();
        assert_eq!(fake.writes(), Vec::<u8>::new(), "C would toggle here");
    }
}

bounded_test! {
    fn a_set_keeps_the_hold_and_recording_a_capture_saw_true() {
        let mut stream = vec![0xa5, 0x1a, 0xa5, 0x0e, 0xa5, 0x1b, 0];
        stream.extend(readout(0x06, 0x55));
        let fake = Fake::with(|m| m.input.extend(stream));
        let mut device = fake.device();
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut device);
        until_end(&rx);
        {
            let mut m = fake.lock();
            m.input.clear();
            m.live = true;
        }
        device.set(&Key::Datalog, &Value::Bool(true), None).unwrap();
        device.set(&Key::HoldMax, &Value::Bool(true), None).unwrap();
        assert_eq!(fake.writes(), [CMD_TOGGLE_RECORDING, CMD_TOGGLE_HOLD_MAX_MIN]);
        assert_eq!(device.get(&Key::Datalog, None).unwrap(), Value::Bool(true));
        assert_eq!(device.get(&Key::HoldMax, None).unwrap(), Value::Bool(true));
        assert_eq!(device.get(&Key::HoldMin, None).unwrap(), Value::Bool(false));
        // Sets of those send nothing: C's old state would toggle both again.
        device.set(&Key::Datalog, &Value::Bool(true), None).unwrap();
        device.set(&Key::HoldMax, &Value::Bool(true), None).unwrap();
        assert_eq!(fake.writes().len(), 2);
    }
}

bounded_test! {
    fn a_range_set_is_remembered_for_a_get_as_c_does_not() {
        let fake = Fake::live();
        let mut device = fake.device();
        device.set(&Key::SplMeasurementRange, &Value::RangeU64(50, 100), None).unwrap();
        fake.lock().live = false;
        assert_eq!(device.get(&Key::SplMeasurementRange, None).unwrap(), Value::RangeU64(50, 100));
    }
}

bounded_test! {
    fn a_bad_value_is_refused_says_why_and_sends_nothing() {
        let fake = Fake::with(|m| m.stream = vec![0xa5, 0x40]);
        let mut device = fake.device();
        for (key, value, why) in [
            (Key::SplWeightFreq, Value::from("Z"), "must be A or C"),
            (Key::SplWeightTime, Value::from("Z"), "must be F or S"),
            (Key::DataSource, Value::from("Cloud"), "must be Live or Memory"),
            (Key::SplMeasurementRange, Value::RangeU64(10, 20), "must be 30-130"),
            (Key::SplMeasurementRange, Value::U64(1), "must be 30-130"),
            (Key::SplWeightFreq, Value::U64(1), "must be A or C"),
        ] {
            let text = error_text(device.set(&key, &value, None));
            assert!(text.contains(why), "{key:?}: {text}");
        }
        for (key, value) in [
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
}

bounded_test! {
    fn a_write_error_fails_a_set() {
        let fake = Fake::with(|m| {
            m.input.extend([0xa5, 0x1b, 0]);
            m.write_error = true;
        });
        let mut device = fake.device();
        assert!(device.set(&Key::SplWeightFreq, &Value::from("C"), None).is_err());
        assert!(device.set(&Key::PowerOff, &Value::Bool(true), None).is_err());
    }
}

bounded_test! {
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
}

bounded_test! {
    fn power_off_gives_the_next_token_its_time() {
        let fake = Fake::default();
        let mut device = fake.device();
        let started = Instant::now();
        device.set(&Key::PowerOff, &Value::Bool(true), None).unwrap();
        assert!(started.elapsed() >= Duration::from_millis(25));
    }
}

bounded_test! {
    fn power_off_flushes_ahead_of_each_look() {
        // What is queued says nothing of the meter now.
        let fake = Fake::with(|m| m.input.extend([0xa5, 0x40, 0xa5, 0x40]));
        let mut device = fake.device();
        device.set(&Key::PowerOff, &Value::Bool(true), None).unwrap();
        assert_eq!(fake.writes(), [CMD_TOGGLE_POWER_OFF]);
    }
}

bounded_test! {
    fn a_read_error_in_power_off_is_a_meter_gone() {
        let fake = Fake::with(|m| {
            m.read_error = true;
            m.deaf = u32::MAX;
        });
        let mut device = fake.device();
        device.set(&Key::PowerOff, &Value::Bool(true), None).unwrap();
        assert_eq!(fake.writes(), [CMD_TOGGLE_POWER_OFF]);
    }
}

bounded_test! {
    fn power_off_of_a_meter_that_goes_on_fails_after_its_bound() {
        let fake = Fake::with(|m| {
            m.stream = vec![0xa5, 0x40];
            m.deaf = u32::MAX;
        });
        let mut device = fake.device();
        device.link().toggle_bound = Some(Duration::from_secs(1));
        assert_eq!(
            error_text(device.set(&Key::PowerOff, &Value::Bool(true), None)),
            "device error: the meter did not turn off in 1 s"
        );
        assert!(fake.writes().len() >= 2, "a slow sleep leaves room for more than one");
    }
}

bounded_test! {
    fn a_port_that_will_not_open_again_after_a_power_off_is_no_failure() {
        // As C, which does not look at it. This name opens nothing.
        let fake = Fake::default();
        let mut device = CemDt885xDevice::new(
            Link::new(Params::parse(SERIALCOMM).unwrap()),
            "/dev/cem-dt-885x-no-such-port".to_owned(),
            true,
        );
        device.pending = Some(Box::new(fake.clone()));
        device.open().unwrap();
        device.set(&Key::PowerOff, &Value::Bool(true), None).unwrap();
        assert!(device.link().port.is_none());
        assert_eq!(fake.writes(), [CMD_TOGGLE_POWER_OFF]);
    }
}

bounded_test! {
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
        assert_eq!(failure(&packets), None, "a limit met");
        assert_eq!(first_flags(&packets), MqFlags::SPL_FREQ_WEIGHT_C | MqFlags::SPL_TIME_WEIGHT_S);
        let Packet::Analog(a) = &packets[1] else { panic!("{:?}", packets[1]) };
        assert_eq!((a.mq, a.unit, a.digits), (Mq::SoundPressureLevel, Unit::DecibelSpl, 1));
        assert_eq!(fake.writes(), Vec::<u8>::new());
    }
}

bounded_test! {
    fn a_capture_starting_at_a_display_token_sends_no_level_it_does_not_have() {
        let fake = Fake::with(|m| m.input.extend([0xa5, 0x0c, 0xa5, 0x0d, 0x06, 0x55, 0xa5, 0x0c]));
        let mut device = fake.device();
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut device);
        assert_eq!(levels(&until_end(&rx)), [65.5], "C: [0.0, 65.5]");
    }
}

bounded_test! {
    fn a_stop_ends_a_capture_on_a_meter_that_goes_on() {
        let fake = Fake::with(|m| m.stream = readout(0x06, 0x55));
        let mut device = fake.device();
        let rx = start(&mut device);
        let deadline = Instant::now() + BOUND;
        let mut got = 0;
        while got < 3 {
            let left = deadline.saturating_duration_since(Instant::now());
            let (_, packet) = rx.recv_timeout(left).unwrap();
            if matches!(packet, Packet::Analog(_)) {
                got += 1;
            }
        }
        device.stop().unwrap();
        assert_eq!(failure(&until_end(&rx)), None, "a stop");
        device.close().unwrap();
        assert!(device.link().port.is_none());
    }
}

bounded_test! {
    fn a_full_queue_does_not_hold_a_close() {
        let fake = Fake::with(|m| m.stream = readout(0x06, 0x55));
        let mut device = fake.device();
        let (tx, _rx): (SyncSender<(usize, Packet)>, Received) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        wait_for("levels in the way", || fake.lock().bytes_read > 0);
        device.close().unwrap();
    }
}

bounded_test! {
    fn a_full_queue_does_not_hold_the_header_past_a_stop() {
        let (tx, rx) = sync_channel(1);
        let sink = PacketSink::new(tx, 0);
        assert!(sink.send(Packet::End), "the queue is now full");
        let fake = Fake::default();
        let mut device = fake.device();
        device.start(sink).unwrap();
        std::thread::sleep(Duration::from_millis(5));
        device.close().unwrap();
        assert_eq!(fake.lock().reads, 0, "it never got past the header");
        drop(rx);
    }
}

bounded_test! {
    fn a_close_joins_the_capture() {
        // A queue no one reads: the End waits half a second after the stop, and a close
        // that does not join is back long before that.
        let fake = Fake::with(|m| m.stream = readout(0x06, 0x55));
        let mut device = fake.device();
        let (tx, rx) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        wait_for("levels", || fake.lock().bytes_read > 0);
        device.close().unwrap();
        assert_eq!(Arc::strong_count(&device.link), 1, "the thread is gone");
        drop(rx);
    }
}

bounded_test! {
    fn a_start_after_a_capture_whose_end_is_stuck_does_not_hang_and_joins_it() {
        let fake = Fake::with(|m| m.stream = readout(0x06, 0x55));
        let mut device = fake.device();
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        // Room for the header and the level: the End waits, and no one has said stop.
        let (tx, stuck) = sync_channel(2);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let capturing = Arc::clone(&device.capturing);
        wait_for("the capture over", || !capturing.load(Ordering::SeqCst));
        let rx = start(&mut device);
        // The old thread has given up its End and left, its sender with it.
        assert!(stuck.try_recv().is_ok(), "the header");
        assert!(stuck.try_recv().is_ok(), "the level");
        assert!(matches!(
            stuck.try_recv(),
            Err(std::sync::mpsc::TryRecvError::Disconnected)
        ));
        assert_eq!(levels(&until_end(&rx)), [65.5], "a second capture ran");
    }
}

bounded_test! {
    fn a_stopped_capture_waits_a_moment_with_its_end() {
        let fake = Fake::with(|m| m.stream = readout(0x06, 0x55));
        let mut device = fake.device();
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let (tx, rx) = sync_channel(2);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let capturing = Arc::clone(&device.capturing);
        wait_for("the capture over", || !capturing.load(Ordering::SeqCst));
        device.stop().unwrap();
        // Well inside END_AFTER_STOP, even at a sleep twelve times as long, and far
        // past a first look at the queue: a reader who comes now still gets it.
        std::thread::sleep(Duration::from_millis(20));
        assert!(rx.recv_timeout(BOUND).is_ok(), "the header");
        assert!(rx.recv_timeout(BOUND).is_ok(), "the level");
        let end = rx.recv_timeout(BOUND);
        assert!(matches!(end, Ok((_, Packet::End))), "{end:?}");
    }
}

bounded_test! {
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
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut device);
        // The tail is passed over, and the display token behind it has no level of
        // this capture: the next is the first, and right.
        assert_eq!(levels(&until_end(&rx)), [70.3]);
    }
}

bounded_test! {
    fn a_capture_stopped_inside_a_packet_leaves_nothing_for_the_next() {
        let fake = Fake::with(|m| m.input.extend([0xa5, 0x0d, 0x07]));
        let mut device = fake.device();
        let rx = start(&mut device);
        wait_for("the bytes", || fake.lock().bytes_read == 3);
        device.stop().unwrap();
        until_end(&rx);
        // Its last byte, which must not finish the old packet's level of 70.2.
        {
            let mut m = fake.lock();
            m.input.extend([0x02, 0xa5, 0x0b, 0]);
            m.input.extend(readout(0x07, 0x03));
        }
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut device);
        assert_eq!(levels(&until_end(&rx)), [70.3]);
    }
}

bounded_test! {
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
}

bounded_test! {
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
        assert_eq!(failure(&packets), None, "the log's end");
        assert_eq!(first_flags(&packets), MqFlags::SPL_FREQ_WEIGHT_A);
        assert!(packets.iter().any(|p| {
            matches!(p, Packet::Meta(m) if m[0] == (Key::SampleInterval, Value::U64(16_000)))
        }));
        assert_eq!(fake.writes(), [CMD_TRANSFER_MEMORY; 4], "one per byte, none for the bb");
        assert_ne!(fake.lock().input.len(), 0, "nothing past the end is read");
        assert_eq!(fake.lock().plain_writes, 0, "and never a write with no bound of its own");
    }
}

bounded_test! {
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
}

bounded_test! {
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
}

bounded_test! {
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
}

bounded_test! {
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
}

bounded_test! {
    fn a_line_that_will_not_take_a_request_stops_the_asking_for_that_read() {
        // 256 bytes in one read: asking for each would hold a close for 25 s.
        let fake = Fake::with(|m| {
            m.line_full = true;
            m.input.extend([0x12; 256]);
        });
        let mut device = fake.device();
        device.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
        let rx = start(&mut device);
        wait_for("a request", || fake.lock().asks > 0);
        device.close().unwrap();
        assert!(fake.lock().asks <= 2, "{}", fake.lock().asks);
        drop(rx);
        // And with no stop to end it, one for the whole read.
        let fake = Fake::with(|m| {
            m.line_full = true;
            m.input.extend([0x12; 4]);
        });
        let device = fake.device();
        let capture = Capture {
            link: Arc::clone(&device.link),
            limit_samples: 0,
            from_memory: true,
            enabled: true,
            silence_bound: SILENCE_BOUND,
            level_bound: LEVEL_BOUND,
        };
        let pass = capture.pass(0, &mut true, &AtomicBool::new(false)).unwrap();
        assert_eq!(pass.bytes, 4);
        assert_eq!(fake.lock().asks, 1);
    }
}

bounded_test! {
    fn a_stop_ends_a_read_with_bytes_left_and_requests_to_send() {
        let fake = Fake::with(|m| m.input.extend([0x12; 256]));
        let stop = AtomicBool::new(true);
        let device = fake.device();
        let capture = Capture {
            link: Arc::clone(&device.link),
            limit_samples: 0,
            from_memory: true,
            enabled: true,
            silence_bound: SILENCE_BOUND,
            level_bound: LEVEL_BOUND,
        };
        let mut wants_log = true;
        let pass = capture.pass(0, &mut wants_log, &stop).unwrap();
        assert_eq!(pass.bytes, 256);
        assert_eq!(fake.lock().asks, 0);
    }
}

bounded_test! {
    fn silence_ends_a_limited_capture_after_its_bound_and_not_before() {
        let fake = Fake::default();
        let mut device = fake.device();
        device.silence_bound = Duration::from_millis(50);
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let started = Instant::now();
        let rx = start(&mut device);
        let packets = until_end(&rx);
        assert_eq!(levels(&packets), Vec::<f32>::new());
        assert_eq!(failure(&packets).as_deref(), Some("device error: no byte from the meter in 50 ms"));
        assert!(started.elapsed() >= Duration::from_millis(50));
    }
}

bounded_test! {
    fn bytes_with_no_level_end_a_limited_capture_after_their_bound_and_not_before() {
        // A stray bb too, with no end in sight, where bytes alone would go on for ever.
        for stream in [vec![0xa5, 0x40], vec![0xbb, 0x12]] {
            let fake = Fake::with(|m| m.stream = stream);
            let mut device = fake.device();
            device.level_bound = Duration::from_millis(50);
            device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
            let started = Instant::now();
            let rx = start(&mut device);
            let packets = until_end(&rx);
            assert_eq!(levels(&packets), Vec::<f32>::new());
            assert!(failure(&packets).is_some_and(|why| why.contains("no level from the meter")));
            assert!(started.elapsed() >= Duration::from_millis(50));
        }
    }
}

bounded_test! {
    fn levels_start_the_bound_on_no_level_again() {
        // A level per some reads, past the bound in all.
        let fake = Fake::with(|m| {
            m.stream = readout(0x06, 0x55);
            m.gap = 3;
        });
        let mut device = fake.device();
        device.level_bound = Duration::from_millis(500);
        device.set(&Key::LimitSamples, &Value::U64(1_000_000), None).unwrap();
        let rx = start(&mut device);
        wait_for("far past the bound", || fake.lock().reads > 800);
        let ended = !device.capturing.load(Ordering::SeqCst);
        assert!(!ended || fake.lock().longest_gap >= Duration::from_millis(400), "ended early");
        device.stop().unwrap();
        until_end(&rx);
    }
}

bounded_test! {
    fn only_a_gap_with_no_byte_counts_as_silence() {
        // Each read that finds bytes starts the bound again. Counted in reads, so a stall
        // of the host's can't end it unless a read saw one that long.
        let fake = Fake::with(|m| {
            m.stream = vec![0xa5, 0x40];
            m.gap = 3;
        });
        let mut device = fake.device();
        device.silence_bound = Duration::from_millis(500);
        device.level_bound = BOUND;
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut device);
        wait_for("far past the bound", || fake.lock().reads > 800);
        let ended = !device.capturing.load(Ordering::SeqCst);
        assert!(!ended || fake.lock().longest_gap >= Duration::from_millis(400), "ended early");
        device.stop().unwrap();
        until_end(&rx);
    }
}

bounded_test! {
    fn silence_does_not_end_a_capture_with_no_limit() {
        let fake = Fake::default();
        let mut device = fake.device();
        device.silence_bound = Duration::from_millis(5);
        device.level_bound = Duration::from_millis(5);
        let rx = start(&mut device);
        wait_for("reads past the bound", || fake.lock().reads > 60);
        assert!(device.capturing.load(Ordering::SeqCst), "a stop ends this");
        device.stop().unwrap();
        until_end(&rx);
    }
}

bounded_test! {
    fn silence_ends_a_capture_of_the_log() {
        let fake = Fake::default();
        let mut device = fake.device();
        device.silence_bound = Duration::from_millis(5);
        device.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
        let rx = start(&mut device);
        assert!(failure(&until_end(&rx)).is_some_and(|why| why.contains("no byte")));
    }
}

bounded_test! {
    fn a_read_error_ends_a_capture_as_a_failed_one() {
        let fake = Fake::with(|m| m.read_error = true);
        let mut device = fake.device();
        let rx = start(&mut device);
        let packets = until_end(&rx);
        assert_eq!(levels(&packets), Vec::<f32>::new());
        assert_eq!(failure(&packets).as_deref(), Some("device error: gone"));
    }
}

bounded_test! {
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
}

bounded_test! {
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
}

// ---- A meter on its own clock. ----

const BYTE: Duration = Duration::from_micros(1042);
const BLOCK: Duration = Duration::from_millis(50);

/// When the meter takes a command it has heard.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Takes {
    /// At once, so the very next packet says so: the worst case for a toggle.
    AtOnce,
    /// At a block's start, one command per block.
    AtABlock,
}

struct Sim {
    /// Each byte and when it has arrived.
    rx: VecDeque<(Instant, u8)>,
    commands: VecDeque<(Instant, u8)>,
    next_block: Instant,
    block_no: u32,
    long_every: u32,
    freq_c: bool,
    slow: bool,
    recording: bool,
    hold: usize,
    range: usize,
    loss_pct: u32,
    takes: Takes,
    /// Bytes are let out only at multiples of this, as a cable with a latency timer
    /// or a stall does.
    burst: Option<Duration>,
    start: Instant,
    rng: u64,
    longest_gap: Duration,
    last_read: Option<Instant>,
    commands_seen: u32,
}

impl Sim {
    fn random(&mut self) -> u32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        (self.rng >> 8) as u32
    }

    fn step(&mut self, command: u8) {
        if self.random() % 100 < self.loss_pct {
            return;
        }
        match command {
            CMD_TOGGLE_WEIGHT_FREQ => self.freq_c = !self.freq_c,
            CMD_TOGGLE_WEIGHT_TIME => self.slow = !self.slow,
            CMD_TOGGLE_RECORDING => self.recording = !self.recording,
            CMD_TOGGLE_HOLD_MAX_MIN => self.hold = (self.hold + 1) % HOLDS.len(),
            CMD_TOGGLE_MEAS_RANGE => self.range = (self.range + 1) % RANGE_TOKENS.len(),
            _ => {}
        }
    }

    fn take_commands(&mut self, at: Instant) {
        while self.commands.front().is_some_and(|c| c.0 <= at) {
            let (_, command) = self.commands.pop_front().expect("looked");
            self.step(command);
        }
    }

    /// Blocks in the package's order, each packet with the state when it starts.
    fn advance(&mut self) {
        let now = Instant::now();
        while self.next_block <= now {
            let mut at = self.next_block;
            if self.takes == Takes::AtABlock {
                let mut last = None;
                while self.commands.front().is_some_and(|c| c.0 <= at) {
                    last = self.commands.pop_front();
                }
                if let Some((_, command)) = last {
                    self.step(command);
                }
            }
            let long = self.block_no % self.long_every == self.long_every - 1;
            let kinds: &[u8] =
                if long { &[1, 2, 3, 4, 7, 8, 9, 10] } else { &[5, 6, 11, 7, 8, 9, 10] };
            for kind in kinds {
                if self.takes == Takes::AtOnce {
                    self.take_commands(at);
                }
                let packet: Vec<u8> = match kind {
                    1 => vec![0xa5, 0x19],
                    2 => vec![0xa5, if self.slow { 3 } else { 2 }],
                    3 => vec![0xa5, 0x1f],
                    4 => vec![0xa5, if self.recording { 0x0a } else { 0x1a }],
                    5 => vec![0xa5, RANGE_TOKENS[self.range]],
                    6 => vec![0xa5, HOLDS[self.hold]],
                    11 => vec![0xa5, 0x11],
                    7 => vec![0xa5, 0x0d, 0x06, 0x54],
                    8 => vec![0xa5, 0x0b, 0],
                    9 => vec![0xa5, if self.freq_c { 0x1c } else { 0x1b }, 0],
                    _ => vec![0xa5, 6, 0x32, 0x34, 0x56],
                };
                for byte in packet {
                    at += BYTE;
                    self.rx.push_back((at, byte));
                }
            }
            self.next_block += BLOCK;
            self.block_no += 1;
        }
        if self.takes == Takes::AtOnce {
            self.take_commands(now);
        }
    }

    /// The time before which bytes have come out.
    fn horizon(&self, now: Instant) -> Instant {
        let Some(burst) = self.burst else { return now };
        let done = (now - self.start).as_micros() / burst.as_micros().max(1);
        self.start + burst * u32::try_from(done).unwrap_or(0)
    }
}

#[derive(Clone)]
struct Clocked(Arc<Mutex<Sim>>);

impl Clocked {
    fn new(
        seed: u64,
        loss_pct: u32,
        takes: Takes,
        burst: Option<Duration>,
        long_every: u32,
    ) -> Self {
        let now = Instant::now();
        Clocked(Arc::new(Mutex::new(Sim {
            rx: VecDeque::new(),
            commands: VecDeque::new(),
            next_block: now + Duration::from_millis(seed % 50),
            block_no: u32::try_from(seed % u64::from(long_every)).unwrap_or(0),
            long_every,
            freq_c: false,
            slow: false,
            recording: false,
            hold: 0,
            range: 0,
            loss_pct,
            takes,
            burst,
            start: now,
            rng: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            longest_gap: Duration::ZERO,
            last_read: None,
            commands_seen: 0,
        })))
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Sim> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

impl Port for Clocked {
    fn name(&self) -> &'static str {
        "/dev/ttyCLOCK"
    }
    fn set_params(&mut self, _: Params) -> CoreResult<()> {
        Ok(())
    }
    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> CoreResult<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> CoreResult<usize> {
        let end = Instant::now() + timeout;
        loop {
            {
                let mut s = self.lock();
                s.advance();
                let now = Instant::now();
                if let Some(last) = s.last_read {
                    s.longest_gap = s.longest_gap.max(now - last);
                }
                s.last_read = Some(now);
                let horizon = s.horizon(now);
                let mut n = 0;
                while n < buf.len() && s.rx.front().is_some_and(|b| b.0 <= horizon) {
                    buf[n] = s.rx.pop_front().expect("looked").1;
                    n += 1;
                }
                if n > 0 {
                    return Ok(n);
                }
            }
            if Instant::now() >= end {
                return Ok(0);
            }
            std::thread::sleep(Duration::from_micros(200));
        }
    }
    fn write(&mut self, data: &[u8]) -> CoreResult<()> {
        let mut s = self.lock();
        s.advance();
        let at = Instant::now() + BYTE;
        s.commands_seen += u32::try_from(data.len()).unwrap_or(0);
        for byte in data {
            s.commands.push_back((at, *byte));
        }
        Ok(())
    }
    fn purge(&mut self) -> CoreResult<()> {
        let mut s = self.lock();
        s.advance();
        let now = Instant::now();
        while s.rx.front().is_some_and(|b| b.0 <= now) {
            s.rx.pop_front();
        }
        Ok(())
    }
}

#[derive(Default, Debug)]
struct Tally {
    right: u32,
    wrong: u32,
    failed: u32,
    /// A stall of the host's took the line's pacing away: not counted.
    stalled: u32,
    commands: u32,
    time: Duration,
}

fn set_one(link: &mut Link, command: u8) -> CoreResult<()> {
    match command {
        CMD_TOGGLE_WEIGHT_FREQ => link.weight_freq_set(true),
        CMD_TOGGLE_WEIGHT_TIME => link.weight_time_set(true),
        CMD_TOGGLE_RECORDING => link.recording_set(true),
        CMD_TOGGLE_HOLD_MAX_MIN => link.holdmode_set(MqFlags::MAX),
        _ => link.meas_range_set(0x30),
    }
}

fn clocked_trials(
    n: u64,
    command: u8,
    loss_pct: u32,
    takes: Takes,
    burst: Option<Duration>,
    long_every: u32,
) -> Tally {
    let mut tally = Tally::default();
    for i in 0..n {
        let meter = Clocked::new(1000 + i * 7919, loss_pct, takes, burst, long_every);
        let mut link = Link::new(Params::parse(SERIALCOMM).expect("parses"));
        link.port = Some(Box::new(meter.clone()));
        let started = Instant::now();
        let result = set_one(&mut link, command);
        tally.time += started.elapsed();
        // Past anything still on its way.
        std::thread::sleep(Duration::from_millis(5));
        let mut s = meter.lock();
        s.advance();
        let right = match command {
            CMD_TOGGLE_WEIGHT_FREQ => s.freq_c,
            CMD_TOGGLE_WEIGHT_TIME => s.slow,
            CMD_TOGGLE_RECORDING => s.recording,
            CMD_TOGGLE_HOLD_MAX_MIN => s.hold == 1,
            _ => s.range == 1,
        };
        tally.commands += s.commands_seen;
        if s.longest_gap > burst.unwrap_or(BLOCK) + Duration::from_millis(30) {
            tally.stalled += 1;
        } else if result.is_err() {
            tally.failed += 1;
        } else if right {
            tally.right += 1;
        } else {
            tally.wrong += 1;
        }
    }
    tally
}

bounded_test! {
    fn a_set_on_a_clocked_meter_never_says_success_with_it_in_the_other_state() {
        for (command, loss, runs) in [
            (CMD_TOGGLE_WEIGHT_FREQ, 0, 8),
            (CMD_TOGGLE_WEIGHT_FREQ, 50, 8),
            (CMD_TOGGLE_WEIGHT_FREQ, 90, 4),
            (CMD_TOGGLE_HOLD_MAX_MIN, 50, 4),
            (CMD_TOGGLE_MEAS_RANGE, 0, 4),
            (CMD_TOGGLE_MEAS_RANGE, 50, 4),
        ] {
            let tally = clocked_trials(runs, command, loss, Takes::AtOnce, None, 9);
            assert_eq!(tally.wrong, 0, "{command:02x} at {loss}%: {tally:?}");
            assert_eq!(tally.failed, 0, "{command:02x} at {loss}%: {tally:?}");
        }
        // One long block in three, or this takes a minute: the state said rarely.
        for (command, loss) in [(CMD_TOGGLE_WEIGHT_TIME, 0), (CMD_TOGGLE_RECORDING, 50)] {
            let tally = clocked_trials(3, command, loss, Takes::AtOnce, None, 3);
            assert_eq!(tally.wrong, 0, "{command:02x} at {loss}%: {tally:?}");
            assert_eq!(tally.failed, 0, "{command:02x} at {loss}%: {tally:?}");
        }
    }
}

bounded_test! {
    fn a_set_on_a_meter_that_takes_commands_at_a_block_is_right_too() {
        let tally = clocked_trials(8, CMD_TOGGLE_WEIGHT_FREQ, 50, Takes::AtABlock, None, 9);
        assert_eq!((tally.wrong, tally.failed), (0, 0), "{tally:?}");
    }
}

bounded_test! {
    fn a_set_with_bytes_in_bursts_keeps_its_tokens() {
        // A flush at each command lost the token in a burst.
        for burst in [16, 50] {
            let burst = Some(Duration::from_millis(burst));
            let tally = clocked_trials(8, CMD_TOGGLE_WEIGHT_FREQ, 70, Takes::AtOnce, burst, 9);
            assert_eq!((tally.wrong, tally.failed), (0, 0), "{burst:?}: {tally:?}");
        }
    }
}

/// `cargo test -- --ignored --nocapture cem_dt_885x::tests::clocked_table`: the
/// numbers in the report, too long for a run of the tests.
#[test]
#[ignore = "takes minutes"]
fn clocked_table() {
    for (name, command, long_every, runs) in [
        ("freq", CMD_TOGGLE_WEIGHT_FREQ, 9, 100),
        ("hold", CMD_TOGGLE_HOLD_MAX_MIN, 9, 100),
        ("range", CMD_TOGGLE_MEAS_RANGE, 9, 100),
        ("time", CMD_TOGGLE_WEIGHT_TIME, 9, 20),
        ("recording", CMD_TOGGLE_RECORDING, 9, 20),
    ] {
        for (loss, takes, burst) in [
            (0, Takes::AtOnce, None),
            (50, Takes::AtOnce, None),
            (70, Takes::AtOnce, None),
            (90, Takes::AtOnce, None),
            (70, Takes::AtABlock, None),
            (70, Takes::AtOnce, Some(16)),
            (70, Takes::AtOnce, Some(50)),
        ] {
            let burst = burst.map(Duration::from_millis);
            let t = clocked_trials(runs, command, loss, takes, burst, long_every);
            let done = u64::from(t.right + t.wrong + t.failed + t.stalled).max(1);
            eprintln!(
                "TABLE {name:9} loss={loss:2}% {takes:?} burst={burst:?} n={runs}: right={} \
                 WRONG={} failed={} stalled={} commands/set={:.1} ms/set={}",
                t.right,
                t.wrong,
                t.failed,
                t.stalled,
                f64::from(t.commands) / done as f64,
                (t.time / u32::try_from(done).unwrap_or(1)).as_millis(),
            );
        }
    }
}
