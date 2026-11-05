// Counterpart in sigrok:
//   libsigrok/src/hardware/bkprecision-1856d/protocol.c,
//   Copyright (C) 2021 LUMERIIX,
//   Copyright (C) 2024 Daniel Anselmi <danselmi@gmx.ch>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake port: a counter that answers at the line's own rate, so
//! that a tail can be on its way when a flush comes.

use super::*;
use crate::serial::Params;
use std::collections::VecDeque;
use std::sync::PoisonError;
use std::sync::mpsc::{Receiver, RecvTimeoutError, TryRecvError, sync_channel};

/// How long a test waits for what must happen: a stop or a limit that does not
/// bite must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);
/// A byte at 9600/8n1.
const BYTE: Duration = Duration::from_micros(1042);

/// Runs a test on a thread of its own, and fails it if it has not come back in
/// time. Then the device's drop may be joining a capture for ever: the thread is
/// left behind, until the test binary exits.
fn in_time(test: impl FnOnce() + Send + 'static) {
    let (done, finished) = sync_channel(1);
    let thread = std::thread::spawn(move || {
        test();
        let _ = done.send(());
    });
    match finished.recv_timeout(2 * BOUND) {
        Ok(()) => {}
        Err(RecvTimeoutError::Timeout) => panic!("not over after {:?}", 2 * BOUND),
        Err(RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(thread.join().expect_err("it did not finish"));
        }
    }
}

/// What a request gets.
#[derive(Clone)]
enum Answer {
    Frame(Vec<u8>),
    /// Its first byte arrives after this long.
    Late(Vec<u8>, Duration),
    /// After this long: 1.0 for the input A and 2.0 for C, as of the request.
    ByInput(Duration),
    /// Comes when the next request goes, just ahead of that one's answer.
    UntilNext(Vec<u8>),
    Silent,
}

fn frame(value: f64) -> Vec<u8> {
    format!("{value:>10.5} Hz \r").into_bytes()
}

const EMPTY: &[u8; 15] = b"         0    \r";

#[derive(Clone, Copy, PartialEq)]
enum Writes {
    Go,
    /// What a device that stopped reading gets.
    NoRoom,
    Fail,
    /// Three in four fail, so never as many as end a capture.
    FailThreeInFour,
}

struct State {
    writes: Vec<(Instant, Vec<u8>)>,
    answers: VecDeque<Answer>,
    /// What every request gets once `answers` has run out.
    otherwise: Answer,
    /// Each byte, and when it arrives.
    line: VecDeque<(Instant, u8)>,
    byte_time: Duration,
    writes_do: Writes,
    /// The input the last F chose.
    input_a: bool,
    /// What [`Answer::UntilNext`] holds.
    held: Vec<u8>,
    /// How long a write takes, ahead of being seen.
    write_time: Duration,
    /// When this many writes have gone, one read comes back empty after this long:
    /// a stall outside the wait.
    stall_at: Option<(usize, Duration)>,
    /// The port is gone.
    dropped: bool,
}

type Shared = Arc<Mutex<State>>;

fn lock(state: &Shared) -> std::sync::MutexGuard<'_, State> {
    state.lock().unwrap_or_else(PoisonError::into_inner)
}

struct Fake(Shared);

impl Drop for Fake {
    fn drop(&mut self) {
        lock(&self.0).dropped = true;
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
        let stall = {
            let mut state = lock(&self.0);
            match state.stall_at {
                Some((writes, time)) if state.writes.len() == writes => {
                    state.stall_at = None;
                    Some(time)
                }
                _ => None,
            }
        };
        if let Some(time) = stall {
            std::thread::sleep(time);
            return Ok(0);
        }
        let got = {
            let mut state = lock(&self.0);
            let now = Instant::now();
            let mut got = 0;
            while got < buf.len() && state.line.front().is_some_and(|(at, _)| *at <= now) {
                buf[got] = state.line.pop_front().expect("just seen").1;
                got += 1;
            }
            got
        };
        if got == 0 {
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
        }
        Ok(got)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.write_timeout(data, Duration::ZERO).map(|_| ())
    }
    fn write_timeout(&mut self, data: &[u8], _: Duration) -> Result<usize> {
        let time = lock(&self.0).write_time;
        if !time.is_zero() {
            std::thread::sleep(time);
        }
        let mut state = lock(&self.0);
        let now = Instant::now();
        state.writes.push((now, data.to_vec()));
        match state.writes_do {
            Writes::NoRoom => return Ok(0),
            Writes::Fail => return Err(Error::device("Input/output error")),
            Writes::FailThreeInFour if !state.writes.len().is_multiple_of(4) => {
                return Err(Error::device("Input/output error"));
            }
            Writes::Go | Writes::FailThreeInFour => {}
        }
        match data {
            b"F0\r" => state.input_a = true,
            b"F2\r" => state.input_a = false,
            _ => {}
        }
        if data == DATA_REQUEST {
            let answer = state.answers.pop_front().unwrap_or_else(|| state.otherwise.clone());
            let held = std::mem::take(&mut state.held);
            let mut at = state.line.back().map_or(now, |(at, _)| *at).max(now);
            for byte in held {
                at += state.byte_time;
                state.line.push_back((at, byte));
            }
            let (bytes, delay) = match answer {
                Answer::UntilNext(bytes) => {
                    state.held = bytes;
                    (Vec::new(), Duration::ZERO)
                }
                Answer::Frame(bytes) => (bytes, Duration::ZERO),
                Answer::Late(bytes, delay) => (bytes, delay),
                Answer::ByInput(delay) => (frame(if state.input_a { 1.0 } else { 2.0 }), delay),
                Answer::Silent => (Vec::new(), Duration::ZERO),
            };
            let mut at = state.line.back().map_or(now, |(at, _)| *at).max(now + delay);
            for byte in bytes {
                at += state.byte_time;
                state.line.push_back((at, byte));
            }
        }
        Ok(data.len())
    }
    /// Drops only what has arrived.
    fn purge(&mut self) -> Result<()> {
        let mut state = lock(&self.0);
        let now = Instant::now();
        state.line.retain(|(at, _)| *at > now);
        Ok(())
    }
}

fn counter(answers: Vec<Answer>, otherwise: Answer) -> (Bkprecision1856d, Shared) {
    let state = Arc::new(Mutex::new(State {
        writes: Vec::new(),
        answers: answers.into(),
        otherwise,
        line: VecDeque::new(),
        byte_time: BYTE,
        writes_do: Writes::Go,
        input_a: false,
        held: Vec::new(),
        write_time: Duration::ZERO,
        stall_at: None,
        dropped: false,
    }));
    let mut device = Bkprecision1856d::from_port(Box::new(Fake(Arc::clone(&state))));
    device.open().unwrap();
    (device, state)
}

fn set_limit(device: &mut Bkprecision1856d, limit: u64) {
    device.set(&Key::LimitSamples, &Value::U64(limit), None).unwrap();
}

fn begin(device: &mut Bkprecision1856d) -> Receiver<(usize, Packet)> {
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// The readings of a capture that did not fail.
fn readings(rx: &Receiver<(usize, Packet)>) -> Vec<f32> {
    let (seen, failure) = finish(rx);
    assert_eq!(failure, None);
    seen
}

/// The readings up to the End, which must come in time, and the failure ahead of it.
/// The Header comes first.
fn finish(rx: &Receiver<(usize, Packet)>) -> (Vec<f32>, Option<String>) {
    let due = Instant::now() + BOUND;
    let mut seen = Vec::new();
    let mut failure = None;
    let mut first = true;
    loop {
        let left = due.saturating_duration_since(Instant::now());
        let (_, packet) = rx.recv_timeout(left).expect("no End in time");
        if first {
            assert!(matches!(packet, Packet::Header { .. }), "the Header goes first");
            first = false;
            continue;
        }
        match packet {
            Packet::End => return (seen, failure),
            Packet::Failed(why) => failure = Some(why),
            Packet::Analog(a) => seen.extend_from_slice(&a.samples),
            other => panic!("{other:?}"),
        }
    }
}

fn commands(state: &Shared) -> Vec<Vec<u8>> {
    lock(state).writes.iter().map(|(_, c)| c.clone()).collect()
}

fn wire(list: &[&[u8; 3]]) -> Vec<Vec<u8>> {
    list.iter().map(|c| c.to_vec()).collect()
}

fn wait_for(state: &Shared, what: &str, done: impl Fn(&State) -> bool) {
    let due = Instant::now() + BOUND;
    while !done(&lock(state)) {
        assert!(Instant::now() < due, "not in time: {what}");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn count(state: &Shared, command: &[u8]) -> usize {
    commands(state).iter().filter(|c| c.as_slice() == command).count()
}

#[test]
fn a_capture_is_a_round_per_reading_and_the_limit_ends_it() {
    in_time(|| {
        let answers = vec![Answer::Frame(frame(1.5)), Answer::Frame(frame(2.5))];
        // A third answer is offered, and no request may get it.
        let (mut device, state) = counter(answers, Answer::Frame(frame(3.5)));
        set_limit(&mut device, 2);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [1.5, 2.5]);
        assert_eq!(commands(&state), wire(&[b"F2\r", b"G0\r", b"D0\r", b"G0\r", b"D0\r"]));
        device.close().unwrap();
        assert_eq!(commands(&state).len(), 5, "a close sends nothing");
    });
}

#[test]
fn the_settings_choose_the_commands_and_stay_off_the_wire_until_then() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
        set_limit(&mut device, 1);
        device.set(&Key::GateTime, &Value::Rational(1, 10), None).unwrap();
        device.set(&Key::DataSource, &Value::from("A"), None).unwrap();
        assert_eq!(commands(&state), Vec::<Vec<u8>>::new());
        assert_eq!(device.get(&Key::GateTime, None).unwrap(), Value::Rational(100, 1000));
        assert_eq!(device.get(&Key::DataSource, None).unwrap(), Value::from("A"));
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [1.0]);
        assert_eq!(commands(&state), wire(&[b"F0\r", b"G1\r", b"D0\r"]));
    });
}

#[test]
fn a_reading_has_cs_digits_and_unit() {
    in_time(|| {
        let answer = Answer::Frame(b"   1.00000MHz \r".to_vec());
        let (mut device, _state) = counter(vec![], answer);
        set_limit(&mut device, 1);
        let rx = begin(&mut device);
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        let Packet::Analog(a) = rx.recv_timeout(BOUND).unwrap().1 else {
            panic!("no reading");
        };
        assert_eq!(a.samples, [1e6]);
        assert_eq!(a.digits, -1);
        assert_eq!(a.mq, Mq::Frequency);
        assert_eq!(a.unit, Unit::Hertz);
        assert_eq!(a.flags, MqFlags::empty());
        assert_eq!(a.channels, [0]);
    });
}

#[test]
fn each_gate_sleeps_its_time_ahead_of_the_request() {
    in_time(|| {
        for (gate, period, millis) in [
            (b"G0\r", (10, 1000), 40),
            (b"G1\r", (100, 1000), 80),
            (b"G2\r", (1, 1), 80),
            (b"G3\r", (10, 1), 800),
        ] {
            let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
            set_limit(&mut device, 2);
            device.set(&Key::GateTime, &Value::Rational(period.0, period.1), None).unwrap();
            let rx = begin(&mut device);
            assert_eq!(readings(&rx).len(), 2);
            let writes = lock(&state).writes.clone();
            for pair in writes.windows(2) {
                if pair[0].1 == gate && pair[1].1 == DATA_REQUEST {
                    let gap = pair[1].0 - pair[0].0;
                    assert!(gap >= Duration::from_millis(millis), "{gate:?}: {gap:?}");
                }
            }
            assert!(writes.iter().any(|(_, c)| c == gate));
        }
    });
}

#[test]
fn an_answer_has_100_ms_from_the_request_and_then_g_and_d_go_again() {
    in_time(|| {
        let (mut device, state) = counter(vec![Answer::Silent], Answer::Frame(frame(4.0)));
        set_limit(&mut device, 1);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [4.0]);
        let writes = lock(&state).writes.clone();
        // No second F, and the wait is at least the answer's.
        assert_eq!(writes[0].1, b"F2\r");
        assert_eq!(writes[1].1, b"G0\r");
        assert_eq!(writes[2].1, b"D0\r");
        assert_eq!(writes[3].1, b"G0\r");
        assert_eq!(writes[4].1, b"D0\r");
        let gap = writes[3].0 - writes[2].0;
        assert!(gap >= Duration::from_millis(100), "{gap:?}");
    });
}

#[test]
fn the_sleep_after_a_gate_does_not_use_up_the_answers_time() {
    in_time(|| {
        // After an expiry C sends G3 and D0 again at once, as its timer then runs from
        // before the sleep. An answer that lies ready wins, so a stall costs nothing here.
        let (mut device, state) = counter(vec![Answer::Silent], Answer::Frame(frame(4.0)));
        set_limit(&mut device, 1);
        device.set(&Key::GateTime, &Value::Rational(10, 1), None).unwrap();
        let rx = begin(&mut device);
        assert_eq!(readings(&rx).len(), 1);
        let writes = lock(&state).writes.clone();
        let requests: Vec<Instant> =
            writes.iter().filter(|(_, c)| c == DATA_REQUEST).map(|(at, _)| *at).collect();
        // The missed one has its 100 ms and a sleep, and the next one its 15 bytes.
        assert!(requests[1] - requests[0] >= Duration::from_millis(900));
        let gates = count(&state, b"G3\r");
        assert_eq!(gates, 2, "the second answer had no time");
    });
}

#[test]
fn a_late_answer_does_not_stay_ahead_of_the_requests() {
    in_time(|| {
        // The first answer comes while the second request is out, and the second
        // right behind it, on its way or there when the third round flushes.
        let answers =
            vec![Answer::Late(frame(1.0), Duration::from_millis(250)), Answer::Frame(frame(2.0))];
        let (mut device, state) = counter(answers, Answer::Frame(frame(3.0)));
        set_limit(&mut device, 2);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [1.0, 3.0]);
        assert!(count(&state, b"D0\r") >= 3);
    });
}

#[test]
fn an_answer_cut_short_brings_the_next_ones_back_in_step() {
    in_time(|| {
        // 14 bytes, so the next answer's first byte ends the buffer, and its other
        // 14 are on their way while the driver flushes.
        let answers = vec![Answer::Frame(b"   1.00000 Hz\r".to_vec())];
        let (mut device, state) = counter(answers, Answer::Frame(frame(2.0)));
        set_limit(&mut device, 2);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [2.0, 2.0]);
        assert_eq!(count(&state, b"F2\r"), 2, "one resync: F again");
    });
}

#[test]
fn what_a_stop_left_on_the_way_does_not_shift_the_next_capture() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
        let rx = begin(&mut device);
        // A stop with a reply on its way, and at once a new capture.
        wait_for(&state, "the second request", |s| {
            s.writes.iter().filter(|(_, c)| c == DATA_REQUEST).count() >= 2
        });
        device.stop().unwrap();
        for sample in readings(&rx) {
            assert_eq!(sample, 1.0);
        }
        let written = commands(&state).len();
        lock(&state).otherwise = Answer::Frame(frame(9.0));
        set_limit(&mut device, 1);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [9.0]);
        let second = commands(&state).split_off(written);
        assert_eq!(second[..3], wire(&[b"F2\r", b"G0\r", b"D0\r"])[..]);
        assert_eq!(second.iter().filter(|c| c.as_slice() == b"F2\r").count(), 1, "no resync");
    });
}

#[test]
fn a_setting_in_a_capture_takes_effect_at_the_next_round() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
        let rx = begin(&mut device);
        wait_for(&state, "a request", |s| s.writes.iter().any(|(_, c)| c == DATA_REQUEST));
        device.set(&Key::DataSource, &Value::from("A"), None).unwrap();
        device.set(&Key::GateTime, &Value::Rational(1, 10), None).unwrap();
        wait_for(&state, "the new gate", |s| s.writes.iter().any(|(_, c)| c == b"G1\r"));
        device.stop().unwrap();
        readings(&rx);
        let all = commands(&state);
        let f = all.iter().position(|c| c == b"F0\r").expect("the input went out");
        let g = all.iter().position(|c| c == b"G1\r").expect("the gate went out");
        assert!(f < g, "as C: the input ahead of the gate");
        // And once: the next rounds have it.
        assert_eq!(count(&state, b"F0\r"), 1);
    });
}

#[test]
fn a_stop_in_the_gates_sleep_sends_no_request() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
        device.set(&Key::GateTime, &Value::Rational(10, 1), None).unwrap();
        let rx = begin(&mut device);
        wait_for(&state, "the gate", |s| s.writes.iter().any(|(_, c)| c == b"G3\r"));
        device.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
        assert_eq!(commands(&state), wire(&[b"F2\r", b"G3\r"]));
    });
}

#[test]
fn a_stop_ends_a_capture_on_a_device_that_always_answers() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
        let rx = begin(&mut device);
        wait_for(&state, "two requests", |s| {
            s.writes.iter().filter(|(_, c)| c == DATA_REQUEST).count() >= 2
        });
        device.stop().unwrap();
        readings(&rx);
        // No more after the End.
        let written = commands(&state).len();
        std::thread::sleep(Duration::from_millis(150));
        assert_eq!(commands(&state).len(), written);
    });
}

#[test]
fn a_stop_ends_a_capture_on_a_device_with_nothing_to_show() {
    in_time(|| {
        // No gate and no sleep in these rounds, so only the loop can see the stop.
        let (mut device, state) = counter(vec![], Answer::Frame(EMPTY.to_vec()));
        let rx = begin(&mut device);
        wait_for(&state, "two requests", |s| {
            s.writes.iter().filter(|(_, c)| c == DATA_REQUEST).count() >= 2
        });
        device.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
    });
}

#[test]
fn a_close_takes_the_port_from_a_capture_that_runs() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
        let rx = begin(&mut device);
        wait_for(&state, "a request", |s| s.writes.iter().any(|(_, c)| c == DATA_REQUEST));
        device.close().unwrap();
        // The thread is over, so nothing brings the port back after this.
        std::thread::sleep(Duration::from_millis(100));
        let (tx, _other) = sync_channel(4);
        assert!(device.start(PacketSink::new(tx, 0)).is_err(), "the port is gone");
        drop(rx);
    });
}

#[test]
fn a_close_has_the_capture_over_and_the_port_gone_when_it_returns() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
        // Most of its time the thread is in a write.
        lock(&state).write_time = Duration::from_millis(100);
        let rx = begin(&mut device);
        wait_for(&state, "a request", |s| s.writes.iter().any(|(_, c)| c == DATA_REQUEST));
        device.close().unwrap();
        assert!(lock(&state).dropped, "the port stays open");
        let written = commands(&state).len();
        std::thread::sleep(Duration::from_millis(300));
        assert_eq!(commands(&state).len(), written, "a write after the close");
        drop(rx);
    });
}

#[test]
fn a_stop_leaves_the_end_time_to_reach_a_client_that_reads() {
    in_time(|| {
        let (mut device, _state) = counter(vec![], Answer::Frame(frame(1.0)));
        set_limit(&mut device, 1);
        // Room for one: the reading waits there, and the End behind it.
        let (tx, rx) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        let due = Instant::now() + BOUND;
        while device.port.lock().unwrap_or_else(PoisonError::into_inner).is_none() {
            assert!(Instant::now() < due, "the capture did not end");
            std::thread::sleep(Duration::from_millis(1));
        }
        device.stop().unwrap();
        // Long enough for the thread to have looked at the stop, well inside the 500 ms.
        // Not a sleep: on a slow runner one could outlast them.
        let stopped = Instant::now();
        while stopped.elapsed() < Duration::from_millis(50) {
            std::thread::yield_now();
        }
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::End));
    });
}

#[test]
fn the_silence_bound_is_as_said() {
    let least = Duration::from_secs(5);
    let seconds: Vec<u64> = (0..4).map(|gate| silence_bound(gate, least).as_secs()).collect();
    assert_eq!(seconds, [5, 5, 5, 30]);
    assert_eq!(SILENCE_BOUND_LEAST, least);
    assert_eq!(ANSWER_TIMEOUT, Duration::from_millis(100));
    assert_eq!(MAX_FAILED_WRITES, 4);
}

#[test]
fn a_silent_device_ends_a_capture_with_a_limit_and_not_before_the_bound() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Silent);
        device.least_silence = Duration::from_millis(400);
        set_limit(&mut device, 1);
        let began = Instant::now();
        let rx = begin(&mut device);
        let (seen, failure) = finish(&rx);
        assert_eq!(seen, Vec::<f32>::new());
        assert!(failure.is_some_and(|w| w.contains("no usable answer")), "the capture failed");
        assert!(began.elapsed() >= Duration::from_millis(400));
        assert!(count(&state, b"G0\r") >= 2, "it asked again in the meantime");
    });
}

#[test]
fn the_bound_is_at_least_three_gates() {
    in_time(|| {
        let (mut device, _state) = counter(vec![], Answer::Silent);
        device.least_silence = Duration::from_millis(100);
        set_limit(&mut device, 1);
        device.set(&Key::GateTime, &Value::Rational(1, 1), None).unwrap();
        let began = Instant::now();
        let rx = begin(&mut device);
        let (seen, failure) = finish(&rx);
        assert_eq!(seen, Vec::<f32>::new());
        assert!(failure.is_some_and(|w| w.contains("no usable answer")), "the capture failed");
        assert!(began.elapsed() >= Duration::from_secs(3), "{:?}", began.elapsed());
    });
}

#[test]
fn a_slow_client_is_no_silent_device() {
    in_time(|| {
        // It takes each packet after longer than the bound, and has all it asked for.
        let (mut device, _state) = counter(vec![], Answer::Frame(frame(1.0)));
        device.least_silence = Duration::from_millis(400);
        set_limit(&mut device, 4);
        let (tx, rx) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let mut got = 0;
        loop {
            std::thread::sleep(Duration::from_millis(600));
            match rx.recv_timeout(BOUND).expect("no End in time").1 {
                Packet::Analog(_) => got += 1,
                Packet::End => break,
                _ => {}
            }
        }
        assert_eq!(got, 4);
    });
}

#[test]
fn an_answer_that_lies_ready_goes_ahead_of_the_timer() {
    in_time(|| {
        // Ready at 5 ms, and the driver looks after 150 ms: as C's check does with
        // its revents, it takes the answer, and neither a gate nor a request goes.
        let answer = Answer::Late(frame(1.0), Duration::from_millis(5));
        let (mut device, state) = counter(vec![answer], Answer::Frame(frame(2.0)));
        lock(&state).stall_at = Some((3, Duration::from_millis(150)));
        set_limit(&mut device, 1);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [1.0]);
        assert_eq!(commands(&state), wire(&[b"F2\r", b"G0\r", b"D0\r"]));
    });
}

#[test]
fn an_answer_that_lies_ready_goes_ahead_of_the_bound() {
    in_time(|| {
        let answer = Answer::Late(frame(1.0), Duration::from_millis(5));
        let (mut device, state) = counter(vec![answer], Answer::Silent);
        device.least_silence = Duration::from_millis(50);
        lock(&state).stall_at = Some((3, Duration::from_millis(150)));
        set_limit(&mut device, 1);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [1.0]);
    });
}

#[test]
fn each_byte_of_an_answer_gives_the_next_its_time() {
    in_time(|| {
        // 900 ms for a frame, under 100 ms from byte to byte. A stall may cost a round,
        // and a timer that the bytes do not start again costs six.
        let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
        lock(&state).byte_time = Duration::from_millis(60);
        set_limit(&mut device, 1);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [1.0]);
        let gates = count(&state, b"G0\r");
        assert!(gates <= 3, "{gates}");
    });
}

#[test]
fn an_answer_gives_the_next_request_its_time() {
    in_time(|| {
        // The second answer comes 80 ms after its request, itself 40 ms after the
        // first answer: inside the 100 ms only if they start at that request.
        let answers =
            vec![Answer::Frame(frame(1.0)), Answer::Late(frame(2.0), Duration::from_millis(80))];
        let (mut device, state) = counter(answers, Answer::Silent);
        set_limit(&mut device, 2);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [1.0, 2.0]);
        assert_eq!(commands(&state), wire(&[b"F2\r", b"G0\r", b"D0\r", b"G0\r", b"D0\r"]));
    });
}

#[test]
fn a_frame_after_one_that_is_no_frequency_is_not_flushed() {
    in_time(|| {
        // No flush in that path, as in C: the frame lying behind it is the next.
        let mut both = b"   1.00000 s  \r".to_vec();
        both.extend_from_slice(&frame(5.0));
        let (mut device, _state) = counter(vec![Answer::Frame(both)], Answer::Frame(frame(6.0)));
        set_limit(&mut device, 1);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [5.0]);
    });
}

#[test]
fn a_new_input_in_a_capture_flushes_what_answers_the_old_one() {
    in_time(|| {
        // Each answer at 60 ms. The input changes with a retry's answer on its way.
        let answer = Answer::ByInput(Duration::from_millis(60));
        let (mut device, state) = counter(vec![], answer);
        let rx = begin(&mut device);
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)));
        device.set(&Key::DataSource, &Value::from("A"), None).unwrap();
        wait_for(&state, "the new input", |s| s.input_a);
        let at_f = Instant::now();
        let due = at_f + BOUND;
        let mut values = Vec::new();
        while values.len() < 3 {
            assert!(Instant::now() < due, "no readings in time");
            if let Packet::Analog(a) = rx.recv_timeout(BOUND).unwrap().1 {
                values.extend_from_slice(&a.samples);
            }
        }
        device.stop().unwrap();
        // The reading that went out ahead of F may be in the queue; no later one is old.
        assert_eq!(values[1..], [1.0, 1.0], "{values:?}");
    });
}

#[test]
fn a_second_capture_on_another_input_does_not_read_the_old_one() {
    in_time(|| {
        for millis in [60, 90] {
            let answer = Answer::ByInput(Duration::from_millis(millis));
            let (mut device, state) = counter(vec![], answer);
            let rx = begin(&mut device);
            // A stop with an answer on its way, and no gap.
            wait_for(&state, "the second request", |s| {
                s.writes.iter().filter(|(_, c)| c == DATA_REQUEST).count() >= 2
            });
            device.stop().unwrap();
            readings(&rx);
            device.set(&Key::DataSource, &Value::from("A"), None).unwrap();
            set_limit(&mut device, 1);
            let rx = begin(&mut device);
            assert_eq!(readings(&rx), [1.0], "at {millis} ms");
        }
    });
}

#[test]
fn a_new_input_waits_out_the_answer_to_the_last_request() {
    in_time(|| {
        // The first request's answer comes with the second, so the reading that
        // shows the new input goes with an answer 70 ms away. The gate's sleep is
        // over by then, and the wait for that request's 100 ms is not.
        let answers = vec![
            Answer::UntilNext(frame(2.0)),
            Answer::Late(frame(2.0), Duration::from_millis(70)),
        ];
        let (mut device, state) = counter(answers, Answer::ByInput(Duration::ZERO));
        lock(&state).byte_time = Duration::ZERO;
        set_limit(&mut device, 2);
        let rx = begin(&mut device);
        wait_for(&state, "a request", |s| s.writes.iter().any(|(_, c)| c == DATA_REQUEST));
        device.set(&Key::DataSource, &Value::from("A"), None).unwrap();
        assert_eq!(readings(&rx), [2.0, 1.0]);
    });
}

#[test]
fn a_silent_device_with_no_limit_is_waited_for_until_a_stop() {
    in_time(|| {
        let (mut device, _state) = counter(vec![], Answer::Silent);
        device.least_silence = Duration::from_millis(100);
        let rx = begin(&mut device);
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert!(matches!(
            rx.recv_timeout(Duration::from_millis(500)),
            Err(RecvTimeoutError::Timeout)
        ));
        device.stop().unwrap();
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::End));
    });
}

#[test]
fn an_answer_in_the_bound_starts_it_again() {
    in_time(|| {
        // The second request goes out after the first's 100 ms and a sleep, so its
        // answer comes after the bound, counted from the start: only the first
        // answer's restart lets it in.
        let late = |value, millis| Answer::Late(frame(value), Duration::from_millis(millis));
        let (mut device, _state) = counter(vec![late(1.0, 700), late(2.0, 900)], Answer::Silent);
        device.least_silence = Duration::from_millis(1000);
        set_limit(&mut device, 2);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [1.0, 2.0]);
    });
}

#[test]
fn nothing_to_show_is_no_silence_and_asks_with_no_new_gate() {
    in_time(|| {
        // 130 answers at the line's rate take over 2 s, well past the bound. A stall
        // may cost a request more, and a gate with it; an answer of this kind that
        // sent a gate would send over a hundred.
        let mut answers = vec![Answer::Frame(EMPTY.to_vec()); 130];
        answers.push(Answer::Frame(frame(7.0)));
        let (mut device, state) = counter(answers, Answer::Frame(frame(7.0)));
        device.least_silence = Duration::from_secs(1);
        set_limit(&mut device, 1);
        let began = Instant::now();
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [7.0]);
        assert!(began.elapsed() >= Duration::from_secs(2));
        assert!(count(&state, b"D0\r") >= 131);
        let gates = count(&state, b"G0\r");
        assert!(gates < 40, "{gates}");
    });
}

#[test]
fn answers_of_no_use_are_silence() {
    in_time(|| {
        let answer = Answer::Frame(b"   1.00000 s  \r".to_vec());
        let (mut device, state) = counter(vec![], answer);
        device.least_silence = Duration::from_millis(400);
        set_limit(&mut device, 1);
        let began = Instant::now();
        let rx = begin(&mut device);
        let (seen, failure) = finish(&rx);
        assert_eq!(seen, Vec::<f32>::new());
        assert!(failure.is_some_and(|w| w.contains("no usable answer")), "the capture failed");
        assert!(began.elapsed() >= Duration::from_millis(400));
        // Each sends the input and the gate again, as C does.
        assert!(count(&state, b"F2\r") >= 2);
        assert!(count(&state, b"G0\r") >= 2);
    });
}

#[test]
fn not_a_frequency_keeps_the_line_and_asks_again() {
    in_time(|| {
        let answers = vec![Answer::Frame(b"   1.00000 s  \r".to_vec())];
        let (mut device, state) = counter(answers, Answer::Frame(frame(2.0)));
        set_limit(&mut device, 1);
        let rx = begin(&mut device);
        assert_eq!(readings(&rx), [2.0]);
        assert_eq!(commands(&state), wire(&[b"F2\r", b"G0\r", b"D0\r", b"F2\r", b"G0\r", b"D0\r"]),);
    });
}

#[test]
fn a_port_that_fails_ends_a_capture_even_with_no_limit() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Silent);
        lock(&state).writes_do = Writes::Fail;
        let rx = begin(&mut device);
        let (seen, failure) = finish(&rx);
        assert_eq!(seen, Vec::<f32>::new());
        assert!(failure.is_some_and(|w| w.contains("could not go out")), "the capture failed");
        // F, G, D, then a G and a D that the fourth failure ends after.
        let written = commands(&state).len();
        assert!((4..=5).contains(&written), "{written}");
    });
}

#[test]
fn a_write_with_no_room_is_no_failure() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Silent);
        lock(&state).writes_do = Writes::NoRoom;
        let rx = begin(&mut device);
        wait_for(&state, "more than a failure would allow", |s| s.writes.len() > 7);
        device.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
    });
}

#[test]
fn a_write_that_works_clears_the_count_of_failures() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Silent);
        lock(&state).writes_do = Writes::FailThreeInFour;
        let rx = begin(&mut device);
        wait_for(&state, "more than the failures allow", |s| s.writes.len() > 12);
        device.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
    });
}

#[test]
fn a_start_after_a_capture_whose_end_is_stuck_does_not_hang() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
        set_limit(&mut device, 1);
        // Room for one, read once: the Analog waits there, and End cannot go.
        let (tx, stuck) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(stuck.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        // The port comes back ahead of the End, which is what lets this through.
        let due = Instant::now() + BOUND;
        let rx = loop {
            let (tx, rx) = sync_channel(64);
            match device.start(PacketSink::new(tx, 0)) {
                Ok(()) => break rx,
                Err(Error::Busy) => {
                    assert!(Instant::now() < due, "the port did not come back");
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(e) => panic!("{e}"),
            }
        };
        // The old thread is over, so its sink is gone, ahead of the new capture's end.
        assert!(matches!(stuck.try_recv().map(|(_, p)| p), Ok(Packet::Analog(_))));
        assert!(matches!(stuck.try_recv(), Err(TryRecvError::Disconnected)));
        assert_eq!(readings(&rx), [1.0]);
        assert_eq!(count(&state, b"F2\r"), 2);
    });
}

#[test]
fn a_close_does_not_hang_on_a_queue_no_one_reads() {
    in_time(|| {
        let (mut device, state) = counter(vec![], Answer::Frame(frame(1.0)));
        let (tx, stuck) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        // The Header is in the queue, and the first reading has nowhere to go.
        wait_for(&state, "a request", |s| s.writes.iter().any(|(_, c)| c == DATA_REQUEST));
        std::thread::sleep(Duration::from_millis(50));
        device.close().unwrap();
        drop(stuck);
        // And a close has the port gone.
        let (tx, _rx) = sync_channel(4);
        assert!(device.start(PacketSink::new(tx, 0)).is_err());
    });
}

#[test]
fn a_second_start_is_busy() {
    in_time(|| {
        let (mut device, _state) = counter(vec![], Answer::Frame(frame(1.0)));
        let rx = begin(&mut device);
        let (tx, _other) = sync_channel(4);
        assert!(matches!(device.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        device.stop().unwrap();
        readings(&rx);
    });
}

#[test]
fn a_scan_takes_what_is_named_and_opens_nothing() {
    let driver = Bkprecision1856dDriver;
    assert!(driver.scan(&[]).unwrap().is_empty());
    let conn = Value::from("/dev/no-such-node");
    let found = driver.scan(&[(Key::Conn, conn)]).unwrap();
    assert_eq!(found.len(), 1);
    let info = found[0].info();
    assert_eq!(info.vendor, "BK Precision");
    assert_eq!(info.model, "bk-1856d");
    assert_eq!(info.conn, None);
    assert_eq!(found[0].channels()[0].name, "P1");
    assert_eq!(driver.scan_options(), [Key::Conn]);
    assert_eq!(driver.classes(), [DeviceClass::FrequencyCounter]);
}

#[test]
fn the_options_and_their_lists_are_cs() {
    let (mut device, _state) = counter(vec![], Answer::Silent);
    assert_eq!(device.get(&Key::GateTime, None).unwrap(), Value::Rational(10, 1000));
    assert_eq!(device.get(&Key::DataSource, None).unwrap(), Value::from("C"));
    assert_eq!(device.get(&Key::LimitSamples, None).unwrap(), Value::U64(0));
    let options = device.options(None).unwrap();
    let keys: Vec<Key> = options.iter().map(|o| o.key.clone()).collect();
    assert_eq!(keys, [Key::Continuous, Key::GateTime, Key::LimitSamples, Key::DataSource]);
    assert!(options[2].access.set && !options[2].access.get && !options[2].access.list);
    let ValueList::Rational(gates) = device.list(&Key::GateTime, None).unwrap() else {
        panic!("no rationals");
    };
    assert_eq!(gates, [(10, 1000), (100, 1000), (1, 1), (10, 1)]);
    let ValueList::Str(inputs) = device.list(&Key::DataSource, None).unwrap() else {
        panic!("no strings");
    };
    assert_eq!(inputs, ["A", "C"]);
    assert!(device.list(&Key::LimitSamples, None).is_err());
    assert!(device.get(&Key::Conn, None).is_err());
    assert!(device.set(&Key::GateTime, &Value::Rational(2, 1), None).is_err());
    assert!(device.set(&Key::GateTime, &Value::U64(1), None).is_err());
    assert!(device.set(&Key::DataSource, &Value::from("a"), None).is_err());
    assert!(device.set(&Key::DataSource, &Value::U64(0), None).is_err());
    assert!(device.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(device.set(&Key::Conn, &Value::from("x"), None).is_err());
    assert_eq!(device.get(&Key::GateTime, None).unwrap(), Value::Rational(10, 1000));
}

#[test]
fn a_start_on_a_closed_device_is_refused() {
    let (mut device, _state) = counter(vec![], Answer::Silent);
    device.close().unwrap();
    let (tx, _rx) = sync_channel(4);
    assert!(device.start(PacketSink::new(tx, 0)).is_err());
}
