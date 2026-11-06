// Counterpart in sigrok:
//   libsigrok/src/hardware/tondaj-sl-814/api.c,
//   libsigrok/src/hardware/tondaj-sl-814/protocol.c,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Martin Ling,
//   Daniel Elstner, Gerhard Sittig, Soeren Apel, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

use super::protocol::{
    GIVE_UP, GiveUp, INIT, LONGEST_WAIT, NO_ANSWER, PACE, QUIET, REQUEST, parse_reading,
};
use super::*;
use crate::core::{MqFlags, Packet};
use crate::serial::WriteLog;
use std::collections::VecDeque;
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test waits for what must happen: a stop or a limit that does not
/// bite must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);
/// Bounds that no test of anything else should meet.
const FAR: Duration = Duration::from_secs(600);
/// 12.3 dB(A) F.
const OK: [u8; 4] = [0x00, 0x7b, 0x01, 0x0d];
/// 45.6 dB(A) F.
const OK2: [u8; 4] = [0x01, 0xc8, 0x01, 0x0d];
const V_OK: f32 = 12.3;
const V_OK2: f32 = 45.6;
const GOOD_INIT: [u8; 2] = [0x05, 0x0d];
/// A byte at 9600 8e1.
const LINE_RATE: Duration = Duration::from_micros(1146);

/// What the meter does with a command.
#[derive(Clone, Debug)]
enum Reply {
    /// Those bytes, at the fake's pace.
    Now(Vec<u8>),
    /// Nothing.
    Silent,
    /// Those bytes, when the test says so.
    OnRelease(Vec<u8>),
    /// Those bytes, while the next read returns empty after a stall of this long.
    AcrossAStall(Vec<u8>, Duration),
}

/// How bytes on their way come. A read past their time finds them all, so a stall
/// or slow sleeps in the reader never cost a byte, as on a tty.
#[derive(Clone, Copy, Debug, Default)]
enum Spacing {
    /// All at once.
    #[default]
    None,
    /// A byte every this long: the line's rate.
    Line(Duration),
    /// So many bytes every 50 ms: a cable's latency timer.
    Burst(usize),
}

impl State {
    fn arrive(&mut self, bytes: impl IntoIterator<Item = u8>) {
        let now = Instant::now();
        let mut queued = self.arrives.len();
        let mut last = self.arrives.back().copied().unwrap_or(now).max(now);
        for byte in bytes {
            self.input.push_back(byte);
            match self.spacing {
                Spacing::None => {}
                Spacing::Line(per_byte) => {
                    last += per_byte;
                    self.arrives.push_back(last);
                }
                Spacing::Burst(size) => {
                    queued += 1;
                    if queued.is_multiple_of(size) {
                        last += Duration::from_millis(50);
                    }
                    self.arrives.push_back(last + Duration::from_millis(50));
                }
            }
        }
    }
}

#[derive(Default)]
struct State {
    writes: WriteLog,
    script: VecDeque<Reply>,
    /// What each request gets once the script is out.
    otherwise: Vec<u8>,
    /// Bytes on their way.
    input: VecDeque<u8>,
    /// Bytes that come when the test says so.
    held: Vec<u8>,
    /// When each of `input` is there, if that is not at once.
    arrives: VecDeque<Instant>,
    spacing: Spacing,
    fail_writes_after: Option<usize>,
    /// What [`Reply::AcrossAStall`] has for the next read.
    across_a_stall: Option<(Vec<u8>, Duration)>,
    /// A write takes this long, as behind a line that holds it.
    write_blocks: Duration,
    writes_time_out: bool,
    fail_reads: bool,
    /// Reads never run out of this.
    endless: Option<u8>,
}

/// A meter whose bytes come on the Nth read, not at a clock time, unless a test
/// asks for its line's rate.
#[derive(Clone)]
struct Fake(Arc<Mutex<State>>);

impl Fake {
    fn new() -> Fake {
        Fake(Arc::new(Mutex::new(State { otherwise: OK.to_vec(), ..State::default() })))
    }

    fn with<T>(&self, f: impl FnOnce(&mut State) -> T) -> T {
        f(&mut self.0.lock().unwrap())
    }

    fn script(&self, replies: impl IntoIterator<Item = Reply>) {
        self.with(|s| s.script.extend(replies));
    }

    fn release(&self) {
        self.with(|s| {
            let held = std::mem::take(&mut s.held);
            s.arrive(held);
        });
    }

    fn writes(&self) -> Vec<Vec<u8>> {
        self.with(|s| s.writes.iter().map(|(_, w)| w.clone()).collect())
    }

    fn wait_for(&self, what: &str, done: impl Fn(&State) -> bool) {
        let deadline = Instant::now() + BOUND;
        loop {
            if self.with(|s| done(s)) {
                return;
            }
            assert!(Instant::now() < deadline, "no {what} in time");
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    fn wait_for_writes(&self, n: usize) {
        self.wait_for("writes", |s| s.writes.len() >= n);
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
        if let Some((bytes, stall)) = self.with(|s| s.across_a_stall.take()) {
            std::thread::sleep(stall);
            self.with(|s| s.arrive(bytes));
            return Ok(0);
        }
        let got = {
            let mut s = self.0.lock().unwrap();
            if s.fail_reads {
                return Err(Error::device("hung up"));
            }
            if let Some(byte) = s.endless {
                buf.fill(byte);
                return Ok(buf.len());
            }
            let now = Instant::now();
            let n = if matches!(s.spacing, Spacing::None) {
                s.input.len().min(buf.len())
            } else {
                s.arrives.iter().take_while(|at| **at <= now).count().min(buf.len())
            };
            for b in &mut buf[..n] {
                *b = s.input.pop_front().expect("counted");
                s.arrives.pop_front();
            }
            n
        };
        if got == 0 {
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
        }
        Ok(got)
    }
    fn write(&mut self, _: &[u8]) -> Result<()> {
        panic!("every write of this driver has a timeout");
    }
    fn write_timeout(&mut self, data: &[u8], _: Duration) -> Result<usize> {
        let blocks = self.with(|s| s.write_blocks);
        std::thread::sleep(blocks);
        let mut s = self.0.lock().unwrap();
        if s.fail_writes_after.is_some_and(|n| s.writes.len() >= n) {
            return Err(Error::device("Broken pipe"));
        }
        s.writes.push((Instant::now(), data.to_vec()));
        if s.writes_time_out {
            return Ok(0);
        }
        if data == INIT && s.script.is_empty() {
            s.arrive(GOOD_INIT);
            return Ok(data.len());
        }
        match s.script.pop_front() {
            Some(Reply::Now(bytes)) => s.arrive(bytes),
            Some(Reply::Silent) => {}
            Some(Reply::OnRelease(bytes)) => s.held = bytes,
            Some(Reply::AcrossAStall(bytes, stall)) => s.across_a_stall = Some((bytes, stall)),
            // The meter's sequence number: the request's plus one.
            None => {
                let mut otherwise = s.otherwise.clone();
                if let [_, _, sequence, _] = &mut otherwise[..] {
                    *sequence = data[1].wrapping_add(1);
                }
                s.arrive(otherwise);
            }
        }
        Ok(data.len())
    }
    fn purge(&mut self) -> Result<()> {
        panic!("C has no flush here, and nothing should drop what a reply still owes");
    }
}

type Received = Receiver<(usize, Packet)>;

/// The pace short, and nothing at a bound.
const QUICK: Timing = Timing {
    pace: Duration::from_millis(2),
    quiet: Duration::ZERO,
    no_answer: FAR,
    longest_wait: FAR,
    give_up: FAR,
};

/// With these times, and the real ones for what they don't name.
fn timing(pace: Duration, quiet: Duration, no_answer: Duration) -> Timing {
    Timing { pace, quiet, no_answer, ..Timing::default() }
}

fn device_with(fake: &Fake, timing: Timing) -> TondajDevice {
    let mut d = TondajDevice::from_port(Box::new(fake.clone()));
    d.timing = timing;
    d.open().unwrap();
    d
}

fn device(fake: &Fake) -> TondajDevice {
    device_with(fake, QUICK)
}

fn start(d: &mut TondajDevice) -> Received {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

fn limit(d: &mut TondajDevice, samples: u64) {
    d.set(&Key::LimitSamples, &Value::U64(samples), None).unwrap();
}

/// What a capture sent, up to its End, which must come within [`BOUND`].
struct Seen {
    values: Vec<f32>,
    failed: Option<String>,
}

fn seen(rx: &Received) -> Seen {
    let deadline = Instant::now() + BOUND;
    let mut out = Seen { values: Vec::new(), failed: None };
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left).expect("no End in time").1 {
            Packet::Analog(a) => out.values.push(a.samples[0]),
            Packet::Failed(why) => out.failed = Some(why),
            Packet::End => return out,
            _ => {}
        }
    }
}

/// A capture that must end well.
fn readings(rx: &Received) -> Vec<f32> {
    let got = seen(rx);
    assert_eq!(got.failed, None);
    got.values
}

/// Runs a test that can hang on its own thread, within [`BOUND`] and a margin.
fn bounded(test: impl FnOnce() + Send + 'static) {
    let (tx, rx) = sync_channel(1);
    let handle = std::thread::spawn(move || {
        test();
        let _ = tx.send(());
    });
    match rx.recv_timeout(BOUND * 2) {
        Ok(()) => handle.join().unwrap(),
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(handle.join().unwrap_err());
        }
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("the test hung"),
    }
}

// The pure parts.

#[test]
fn the_wikis_examples_read_as_it_says() {
    let (a, c, f, s) = (
        MqFlags::SPL_FREQ_WEIGHT_A,
        MqFlags::SPL_FREQ_WEIGHT_C,
        MqFlags::SPL_TIME_WEIGHT_F,
        MqFlags::SPL_TIME_WEIGHT_S,
    );
    for (aa, bb, value, flags) in [
        (0x09, 0xaf, 43.1, a | s),
        (0x89, 0xcb, 45.9, c | s),
        (0x8a, 0x6c, 62.0, c | s),
        (0x82, 0x99, 66.5, c | f),
        (0x92, 0x85, 64.5, c | f),
        (0xa3, 0x93, 91.5, c | f),
        (0xb3, 0xf2, 101.0, c | f),
    ] {
        let reading = parse_reading(aa, bb);
        assert_eq!(reading.value, value, "{aa:02x} {bb:02x}");
        assert_eq!(reading.flags, flags, "{aa:02x} {bb:02x}");
    }
}

#[test]
fn every_bit_of_the_reading_does_what_it_should() {
    assert_eq!(parse_reading(0x00, 0x00).value, 0.0);
    assert_eq!(parse_reading(0x07, 0xff).value, 204.7);
    // Bit 6, unknown, and the level in bits 5 and 4 change nothing.
    for high in [0x10, 0x20, 0x30, 0x40, 0x70] {
        assert_eq!(parse_reading(high | 0x01, 0x23), parse_reading(0x01, 0x23), "{high:02x}");
    }
    assert!(parse_reading(0x80, 0).flags.contains(MqFlags::SPL_FREQ_WEIGHT_C));
    assert!(!parse_reading(0x80, 0).flags.contains(MqFlags::SPL_FREQ_WEIGHT_A));
    assert!(parse_reading(0x08, 0).flags.contains(MqFlags::SPL_TIME_WEIGHT_S));
    assert!(!parse_reading(0x08, 0).flags.contains(MqFlags::SPL_TIME_WEIGHT_F));
}

#[test]
fn the_commands_and_the_times_are_as_written() {
    assert_eq!(INIT, [0x10, 0x04, 0x0d]);
    assert_eq!(REQUEST, [0x30, 0x00, 0x0d]);
    assert_eq!(SERIALCOMM, "9600/8e1");
    assert_eq!(PACE, Duration::from_millis(500));
    assert_eq!(QUIET, Duration::from_millis(100));
    assert_eq!(NO_ANSWER, Duration::from_secs(5));
    assert_eq!(LONGEST_WAIT, Duration::from_secs(30));
    assert_eq!(GIVE_UP, Duration::from_secs(600));
}

#[test]
fn each_way_of_giving_up_has_its_own_words() {
    let after = Duration::from_secs(5);
    assert_eq!(GiveUp::NoInitReply.message(after), "no answer to init for 5 s");
    assert_eq!(GiveUp::NoReply.message(after), "no answer to a request for 5 s");
    assert_eq!(GiveUp::NoUsableAnswer.message(after), "no usable answer for 5 s");
    assert_eq!(time_limit_message(false), "No answer from the meter before the time limit.");
    assert_eq!(time_limit_message(true), "No usable answer from the meter before the time limit.");
    assert_eq!(GiveUp::NoUsableAnswer.message(GIVE_UP), "no usable answer for 600 s");
    assert_eq!(silence_message(false, after), "No answer from the meter for 5 s; still asking.");
    assert_eq!(
        silence_message(true, after),
        "No usable answer from the meter for 5 s; still asking."
    );
    assert_eq!(
        short_write_message(Command::Init, 0),
        "Only 0 of 3 bytes of the init command went out."
    );
    assert_eq!(
        short_write_message(Command::Request, 2),
        "Only 2 of 3 bytes of the request went out."
    );
}

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

const REAL: Timing = Timing {
    pace: PACE,
    quiet: QUIET,
    no_answer: NO_ANSWER,
    longest_wait: LONGEST_WAIT,
    give_up: GIVE_UP,
};
const SAMPLES: Limits = Limits { samples_only: true, msec: 0 };
const NO_LIMIT: Limits = Limits { samples_only: false, msec: 0 };

fn secs(n: u64) -> Duration {
    Duration::from_secs(n)
}

/// As far as a request out at `at`, with init done just then.
fn out_at(limits: Limits, at: Instant) -> Exchange {
    let mut x = Exchange::new(REAL, limits, at);
    let bytes = x.bytes(Command::Init);
    assert_eq!(bytes, INIT);
    x.sent(Command::Init, at);
    assert_eq!(x.take(&GOOD_INIT, at), Heard::InitOk);
    assert_eq!(x.bytes(Command::Request), REQUEST);
    x.sent(Command::Request, at);
    x
}

/// Answers the request out with its own sequence number, as a sample handed on.
fn answer(x: &mut Exchange, sequence: u8, at: Instant) {
    assert_eq!(x.take(&[1, 2, sequence.wrapping_add(1), 0x0d], at), Heard::Reading(1, 2));
    x.delivered(at);
}

#[test]
fn a_command_waits_its_pace_from_the_start_and_from_each_reply() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, SAMPLES, t0);
    assert_eq!(x.empty_read(t0 + ms(499)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(500)), Next::Send(Command::Init));
    assert_eq!(x.bytes(Command::Init), INIT);
    x.sent(Command::Init, t0 + ms(500));
    assert_eq!(x.want(64), 2);
    assert_eq!(x.take(&GOOD_INIT, t0 + ms(501)), Heard::InitOk);
    assert_eq!(x.empty_read(t0 + ms(1000)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(1001)), Next::Send(Command::Request));
    assert_eq!(x.bytes(Command::Request), REQUEST);
    x.sent(Command::Request, t0 + ms(1001));
    assert_eq!(x.want(64), 4);
    assert_eq!(x.take(&OK[..1], t0 + ms(1002)), Heard::Part);
    assert_eq!(x.want(64), 3);
    assert_eq!(x.take(&OK[1..], t0 + ms(1003)), Heard::Reading(OK[0], OK[1]));
    x.delivered(t0 + ms(1010));
    assert_eq!(x.empty_read(t0 + ms(1509)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(1510)), Next::Send(Command::Request));
}

#[test]
fn a_wrong_reply_is_told_and_its_command_goes_again_at_the_pace_with_cs_number() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, SAMPLES, t0);
    x.sent(Command::Init, t0);
    assert_eq!(x.take(&[0x06, 0x0d], t0), Heard::BadInit([0x06, 0x0d]));
    assert_eq!(x.empty_read(t0 + ms(499)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(500)), Next::Send(Command::Init));
    x.sent(Command::Init, t0 + ms(500));
    assert_eq!(x.take(&GOOD_INIT[..1], t0 + ms(500)), Heard::Part);
    assert_eq!(x.take(&GOOD_INIT[1..], t0 + ms(500)), Heard::InitOk);
    for (bad, what) in [([1, 2, 0x02, 0x0d], "sequence"), ([1, 2, 0x01, 0x0a], "end")] {
        assert_eq!(x.bytes(Command::Request), REQUEST, "{what}");
        x.sent(Command::Request, t0 + ms(1000));
        assert_eq!(x.take(&bad, t0 + ms(1000)), Heard::BadReply(bad), "{what}");
    }
}

/// On a sample limit with no sample yet, nothing asks again: the wire is as in C up
/// to a failure in each wait's words.
#[test]
fn before_a_first_sample_a_sample_limit_fails_at_the_bound_from_both_sides() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, SAMPLES, t0);
    x.sent(Command::Init, t0 + ms(500));
    assert_eq!(x.empty_read(t0 + ms(5499)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(5500)), Next::GiveUp(GiveUp::NoInitReply, NO_ANSWER));
    assert_eq!(x.take(&GOOD_INIT, t0 + ms(5000)), Heard::InitOk);
    x.sent(Command::Request, t0 + ms(5500));
    assert_eq!(x.empty_read(t0 + ms(10_499)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(10_500)), Next::GiveUp(GiveUp::NoReply, NO_ANSWER));
    assert_eq!(x.take(&OK[..1], t0 + ms(10_500)), Heard::Part);
    assert_eq!(x.empty_read(t0 + ms(10_500)), Next::GiveUp(GiveUp::NoUsableAnswer, NO_ANSWER));
}

/// One out at a time, numbered 1, 2, ..., waiting 5, 10, 20, 30, 30 s.
#[test]
fn a_silence_asks_again_with_ever_longer_waits_and_new_numbers() {
    let t0 = Instant::now();
    let mut x = out_at(NO_LIMIT, t0);
    assert_eq!(x.empty_read(t0 + ms(4999)), Next::Wait);
    assert_eq!(x.empty_read(t0 + secs(5)), Next::Warn);
    let mut at = t0;
    for (sequence, wait) in [(1u8, 5u64), (2, 10), (3, 20), (4, 30), (5, 30)] {
        assert_eq!(x.empty_read(at + secs(wait) - ms(1)), Next::Wait, "{sequence}");
        at += secs(wait);
        assert_eq!(x.empty_read(at), Next::Send(Command::Request), "{sequence}");
        assert_eq!(x.bytes(Command::Request), [0x30, sequence, 0x0d]);
        x.sent(Command::Request, at);
    }
    assert_eq!(x.owed(), Some(Command::Request));
}

#[test]
fn init_asks_again_in_the_same_words() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, NO_LIMIT, t0);
    x.sent(Command::Init, t0);
    assert_eq!(x.empty_read(t0 + secs(5)), Next::Warn);
    assert_eq!(x.empty_read(t0 + secs(5)), Next::Send(Command::Init));
    assert_eq!(x.bytes(Command::Init), INIT);
    x.sent(Command::Init, t0 + secs(5));
    assert_eq!(x.take(&GOOD_INIT, t0 + secs(6)), Heard::InitOk);
}

#[test]
fn a_reply_that_is_quick_sets_the_wait_back_and_a_slow_one_leaves_it() {
    let t0 = Instant::now();
    let mut x = out_at(NO_LIMIT, t0);
    assert_eq!(x.empty_read(t0 + secs(5)), Next::Warn);
    assert_eq!(x.bytes(Command::Request), [0x30, 1, 0x0d]);
    x.sent(Command::Request, t0 + secs(5));
    // 6 s after the one it answers: the wait stays at 10 s, so the next isn't asked twice.
    answer(&mut x, 1, t0 + secs(11));
    assert_eq!(x.bytes(Command::Request), REQUEST);
    x.sent(Command::Request, t0 + secs(12));
    assert_eq!(x.empty_read(t0 + secs(21)), Next::Wait);
    // 5 s after it: back to the first wait.
    answer(&mut x, 0, t0 + secs(17));
    assert_eq!(x.bytes(Command::Request), REQUEST);
    x.sent(Command::Request, t0 + secs(18));
    assert_eq!(x.empty_read(t0 + ms(22_999)), Next::Wait);
    assert_eq!(x.empty_read(t0 + secs(23)), Next::Send(Command::Request));
}

/// A number asked in this capture and no longer out is late; one never asked is wrong.
#[test]
fn a_late_reply_is_told_by_its_number_and_the_command_stays_out() {
    let t0 = Instant::now();
    let mut x = out_at(NO_LIMIT, t0);
    let (warned, again) = (t0 + secs(5), t0 + secs(5));
    assert_eq!(x.empty_read(warned), Next::Warn);
    assert_eq!(x.empty_read(again), Next::Send(Command::Request));
    assert_eq!(x.bytes(Command::Request), [0x30, 1, 0x0d]);
    x.sent(Command::Request, again);
    assert_eq!(x.take(&[9, 9, 1, 0x0d], t0 + secs(6)), Heard::Late(0));
    assert_eq!(x.owed(), Some(Command::Request));
    assert_eq!(x.want(64), 4);
    assert_eq!(x.take(&[1, 2, 2, 0x0d], t0 + secs(7)), Heard::Reading(1, 2));
    x.delivered(t0 + secs(7));
    // The next is C's own again, and what is left of the last round is late.
    assert_eq!(x.bytes(Command::Request), REQUEST);
    x.sent(Command::Request, t0 + secs(8));
    assert_eq!(x.take(&[9, 9, 2, 0x0d], t0 + secs(8)), Heard::Late(1));
    assert_eq!(x.take(&[9, 9, 3, 0x0d], t0 + secs(8)), Heard::BadReply([9, 9, 3, 0x0d]));
}

/// With nothing asked again, any other number is wrong, as in C.
#[test]
fn with_nothing_asked_again_another_number_is_a_wrong_reply() {
    let t0 = Instant::now();
    let mut x = out_at(NO_LIMIT, t0);
    assert_eq!(x.take(&[9, 9, 0, 0x0d], t0), Heard::BadReply([9, 9, 0, 0x0d]));
}

#[test]
fn the_numbers_skip_0_as_they_come_round_and_then_all_are_late() {
    let t0 = Instant::now();
    let mut x = out_at(NO_LIMIT, t0);
    for sequence in 1..=255u8 {
        assert_eq!(x.bytes(Command::Request), [0x30, sequence, 0x0d]);
        x.sent(Command::Request, t0);
    }
    assert_eq!(x.bytes(Command::Request), [0x30, 1, 0x0d]);
    x.sent(Command::Request, t0);
    assert_eq!(x.take(&[9, 9, 0x77, 0x0d], t0), Heard::Late(0x76));
}

/// Past a first sample, on a sample limit: told once, asked again, and given up on
/// after 10 minutes, from both sides.
#[test]
fn after_a_sample_a_sample_limit_warns_and_goes_on_for_ten_minutes() {
    let t0 = Instant::now();
    let mut x = out_at(SAMPLES, t0);
    answer(&mut x, 0, t0);
    assert_eq!(x.bytes(Command::Request), REQUEST);
    x.sent(Command::Request, t0);
    assert_eq!(x.empty_read(t0 + secs(5)), Next::Warn);
    let (mut sends, mut warns, mut at) = (0, 0, t0 + secs(5));
    while at < t0 + secs(600) - ms(1) {
        match x.empty_read(at) {
            Next::Wait => {}
            Next::Warn => warns += 1,
            Next::Send(command) => {
                let _ = x.bytes(command);
                x.sent(command, at);
                sends += 1;
            }
            other => panic!("{other:?} at {:?}", at - t0),
        }
        at += ms(500);
    }
    assert_eq!(warns, 0, "once only");
    // At 5, 15, 35, 65, then every 30 s up to 600.
    assert_eq!(sends, 21);
    assert_eq!(x.empty_read(t0 + secs(600) - ms(1)), Next::Wait);
    assert_eq!(x.empty_read(t0 + secs(600)), Next::GiveUp(GiveUp::NoReply, GIVE_UP));
}

/// A reply heard in that time takes it all back, and no sample's limit moves it.
#[test]
fn a_reading_after_a_long_silence_starts_the_bound_again() {
    let t0 = Instant::now();
    let mut x = out_at(SAMPLES, t0);
    answer(&mut x, 0, t0);
    x.sent(Command::Request, t0);
    assert_eq!(x.empty_read(t0 + secs(5)), Next::Warn);
    assert_eq!(x.bytes(Command::Request), [0x30, 1, 0x0d]);
    x.sent(Command::Request, t0 + secs(5));
    answer(&mut x, 1, t0 + secs(599));
    assert_eq!(x.bytes(Command::Request), REQUEST);
    x.sent(Command::Request, t0 + secs(600));
    assert_eq!(x.empty_read(t0 + secs(1199)), Next::Send(Command::Request));
}

#[test]
fn with_no_limit_a_silence_never_ends_and_warns_once() {
    let t0 = Instant::now();
    let mut x = out_at(NO_LIMIT, t0);
    let mut warns = 0;
    for n in 0..4000 {
        let at = t0 + secs(n);
        match x.empty_read(at) {
            Next::Wait => {}
            Next::Warn => warns += 1,
            Next::Send(command) => {
                let _ = x.bytes(command);
                x.sent(command, at);
            }
            other => panic!("{other:?} at {n} s"),
        }
    }
    assert_eq!(warns, 1);
}

/// A time limit past the bound runs to its end, in the same state as a short one, so
/// the status doesn't depend on the length asked for.
#[test]
fn a_time_limit_ends_a_silence_at_its_limit_however_long() {
    for limit in [1500u64, 7000, 700_000] {
        let t0 = Instant::now();
        let limits = Limits { samples_only: false, msec: limit };
        let mut x = out_at(limits, t0);
        let mut at = t0;
        let end = loop {
            at += ms(100);
            match x.empty_read(at) {
                Next::Wait | Next::Warn => {}
                Next::Send(command) => {
                    let _ = x.bytes(command);
                    x.sent(command, at);
                }
                other => break other,
            }
            assert!(at < t0 + ms(limit + 1000), "no end");
        };
        assert_eq!(end, Next::TimeUp, "{limit}");
        assert!(at > t0 + ms(limit), "{limit}");
    }
}

/// C takes a reply that comes after the limit in the pass that reads it: the limit
/// has no say until C's next pass, a pace after the command, would find nothing.
#[test]
fn a_reply_after_the_limit_within_the_pace_is_still_taken() {
    let t0 = Instant::now();
    let limits = Limits { samples_only: false, msec: 100 };
    let mut x = out_at(limits, t0);
    assert!(x.time_is_up(t0 + ms(101)));
    assert!(!x.time_is_up(t0 + ms(100)));
    assert_eq!(x.empty_read(t0 + ms(499)), Next::Wait, "time is up, and the pace is not");
    assert!(matches!(x.take(&OK, t0 + ms(499)), Heard::Reading(..)));
    let mut x = out_at(limits, t0);
    assert_eq!(x.empty_read(t0 + ms(500)), Next::TimeUp);
}

/// Good replies for many bounds on a moved clock, with no real gaps anywhere: the
/// bound only holds if each one sets it back.
#[test]
fn a_meter_that_answers_is_never_given_up_on() {
    let t0 = Instant::now();
    let mut now = t0;
    let mut x = Exchange::new(REAL, SAMPLES, t0);
    now += ms(500);
    assert_eq!(x.empty_read(now), Next::Send(Command::Init));
    x.sent(Command::Init, now);
    x.take(&GOOD_INIT, now);
    for _ in 0..3000 {
        now += ms(500);
        assert_eq!(x.empty_read(now), Next::Send(Command::Request));
        assert_eq!(x.bytes(Command::Request), REQUEST);
        x.sent(Command::Request, now);
        now += ms(100);
        assert_eq!(x.empty_read(now), Next::Wait);
        assert!(matches!(x.take(&OK, now), Heard::Reading(..)));
        x.delivered(now);
    }
    assert_eq!(x.after_junk(now), Next::Wait);
}

/// A client that kept the send waiting: that is no silence of the meter's.
#[test]
fn time_in_a_blocked_send_counts_for_nothing() {
    let t0 = Instant::now();
    let mut x = out_at(SAMPLES, t0);
    assert!(matches!(x.take(&OK, t0), Heard::Reading(..)));
    let after = t0 + secs(60);
    x.delivered(after);
    assert_eq!(x.empty_read(after + ms(499)), Next::Wait);
    assert_eq!(x.empty_read(after + ms(500)), Next::Send(Command::Request));
    assert_eq!(x.after_junk(after + ms(500)), Next::Wait);
    // And a whole bound lost ahead of a request's return: its reply has its full time.
    let _ = x.bytes(Command::Request);
    x.sent(Command::Request, after + secs(60));
    assert_eq!(x.empty_read(after + secs(61)), Next::Wait);
}

/// Wrong replies at the pace: each request is progress for nothing, and they end.
#[test]
fn wrong_replies_run_out_their_bound_though_requests_go() {
    let t0 = Instant::now();
    let mut now = t0;
    let mut x = Exchange::new(REAL, SAMPLES, t0);
    x.sent(Command::Init, now);
    x.take(&GOOD_INIT, now);
    let mut rounds = 0;
    loop {
        now += ms(500);
        assert_eq!(x.empty_read(now), Next::Send(Command::Request));
        let _ = x.bytes(Command::Request);
        x.sent(Command::Request, now);
        assert!(matches!(x.take(&[1, 2, 3, 4], now), Heard::BadReply(_)));
        rounds += 1;
        match x.after_junk(now) {
            Next::Wait => {}
            Next::GiveUp(why, after) => {
                assert_eq!((why, after), (GiveUp::NoUsableAnswer, NO_ANSWER));
                break;
            }
            other => panic!("{other:?}"),
        }
        assert!(rounds < 100, "no end");
    }
    assert_eq!(rounds, 10, "5 s at 500 ms");
}

/// The same past a sample and with no limit: told once, and no failure short of the
/// 10 minutes on a sample limit.
#[test]
fn wrong_replies_past_a_sample_warn_and_go_on() {
    for limits in [SAMPLES, NO_LIMIT] {
        let t0 = Instant::now();
        let mut x = out_at(limits, t0);
        answer(&mut x, 0, t0);
        assert_eq!(x.after_junk(t0 + ms(4999)), Next::Wait);
        assert_eq!(x.after_junk(t0 + secs(5)), Next::Warn);
        assert_eq!(x.after_junk(t0 + secs(6)), Next::Wait, "once");
        let end = x.after_junk(t0 + secs(600));
        if limits.samples_only {
            assert_eq!(end, Next::GiveUp(GiveUp::NoUsableAnswer, GIVE_UP));
        } else {
            assert_eq!(end, Next::Wait);
        }
    }
}

#[test]
fn a_flood_with_nothing_out_holds_the_command_back_and_runs_out_its_bound() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, SAMPLES, t0);
    assert_eq!(x.want(64), 64);
    assert_eq!(x.take(&[0xaa], t0 + ms(450)), Heard::Dropped);
    // What came moves a command that was due, by the quiet.
    assert_eq!(x.empty_read(t0 + ms(549)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(550)), Next::Send(Command::Init));
    let mut now = t0 + ms(450);
    loop {
        match x.after_junk(now) {
            Next::Wait => {}
            Next::GiveUp(why, _) => {
                assert_eq!(why, GiveUp::NoUsableAnswer);
                break;
            }
            other => panic!("{other:?}"),
        }
        now += ms(50);
        assert_eq!(x.take(&[0xaa], now), Heard::Dropped);
        assert_eq!(x.empty_read(now), Next::Wait, "still arriving");
        assert!(now < t0 + ms(5001));
    }
    assert_eq!(now, t0 + ms(5000));
}

/// A read goes ahead of the timer: a stalled host past its time still looks.
#[test]
fn the_read_ahead_of_a_due_command_does_not_wait_but_does_happen() {
    let t0 = Instant::now();
    let x = Exchange::new(REAL, SAMPLES, t0);
    assert_eq!(x.read_wait(ms(5), t0), ms(5));
    assert_eq!(x.read_wait(ms(5), t0 + ms(498)), ms(2));
    assert_eq!(x.read_wait(ms(5), t0 + ms(900)), Duration::ZERO);
}

/// What a simulated meter on its own clock shows of a steady latency.
#[derive(Debug, Default)]
struct Steady {
    most_owed: usize,
    owed_at_end: usize,
    samples: u32,
    late: u32,
    wrong: u32,
    asked_again: u32,
    /// How old a reading is when it is taken, from the command the meter answered.
    oldest: Duration,
    /// How much older that is than it looks, from the request out: a reply that
    /// answers an earlier one than it seems to.
    most_understated: Duration,
}

/// A meter that answers each command `latency` after it, with the number in it, for a
/// simulated minute. It reads the value when the command gets to it.
fn steady(latency: Duration, length: Duration) -> Steady {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, NO_LIMIT, t0);
    let mut now = t0;
    let mut sent_at: VecDeque<(Instant, [u8; 3])> = VecDeque::new();
    let mut line = VecDeque::new();
    let mut answers = VecDeque::new();
    let mut last_sent = t0;
    let mut seen = Steady::default();
    while now < t0 + length {
        now += ms(1);
        if let Some((at, command)) =
            sent_at.pop_front_if(|(at, _)| now.saturating_duration_since(*at) >= latency)
        {
            if command[0] == INIT[0] {
                line.extend(GOOD_INIT);
            } else {
                line.extend([1, 2, command[1].wrapping_add(1), 0x0d]);
            }
            answers.push_back(at);
        }
        if !line.is_empty() {
            let bytes: Vec<u8> = (0..x.want(64).min(line.len()))
                .map(|_| line.pop_front().expect("counted"))
                .collect();
            match x.take(&bytes, now) {
                Heard::InitOk | Heard::Dropped | Heard::Part | Heard::BadInit(_) => {}
                Heard::Late(_) => seen.late += 1,
                Heard::BadReply(_) => seen.wrong += 1,
                Heard::Reading(..) => {
                    x.delivered(now);
                    seen.samples += 1;
                    // The answer that came last is what this holds, whatever it says.
                    let at = answers.pop_back().unwrap_or(now);
                    answers.clear();
                    let age = now.saturating_duration_since(at);
                    let seen_as = now.saturating_duration_since(last_sent);
                    seen.oldest = seen.oldest.max(age);
                    seen.most_understated = seen.most_understated.max(age.saturating_sub(seen_as));
                }
            }
            continue;
        }
        match x.empty_read(now) {
            Next::Wait | Next::Warn => {}
            Next::Send(command) => {
                if x.owed().is_some() {
                    seen.asked_again += 1;
                }
                let bytes = x.bytes(command);
                x.sent(command, now);
                last_sent = now;
                sent_at.push_back((now, bytes));
                seen.most_owed = seen.most_owed.max(sent_at.len());
            }
            other => panic!("{other:?}"),
        }
    }
    seen.owed_at_end = sent_at.len();
    seen
}

/// Below the first wait nothing is asked twice: one is out, and a reading is as old as
/// the latency.
#[test]
fn a_steady_latency_of_2500_ms_owes_one_reply_and_ages_a_reading_by_that() {
    let seen = steady(ms(2500), secs(60));
    println!("2.5 s: {seen:?}");
    assert_eq!(seen.most_owed, 1);
    assert_eq!(seen.asked_again, 0);
    // 3 s a round after the init's.
    assert!((18..=20).contains(&seen.samples), "{seen:?}");
    assert_eq!(seen.oldest, ms(2500));
}

/// Above it, one is asked again and then the wait stays past the latency. The late
/// reply is known by its number, and a reading is never older than the latency. With
/// the same number each time, the late reply would answer the next request and one
/// would be owed for good: a mutant shows that, with `--nocapture`.
#[test]
fn a_steady_latency_of_6_s_settles_with_nothing_owed_behind_the_one_out() {
    let seen = steady(secs(6), secs(60));
    println!("6 s: {seen:?}");
    assert!(seen.asked_again <= 2, "{seen:?}");
    assert!(seen.late <= 2, "{seen:?}");
    assert!(seen.owed_at_end <= 1, "{seen:?}");
    assert!(seen.samples >= 5, "{seen:?}");
    assert_eq!(seen.oldest, secs(6));
    assert_eq!(seen.most_understated, Duration::ZERO, "it answers the request out");
}

/// Past the longest wait a reply is always one asked again, so no reading comes, and
/// what is owed has a bound of its own. On a sample limit that ends in its failure.
#[test]
fn a_meter_slower_than_the_longest_wait_has_no_reading_and_no_backlog() {
    let seen = steady(secs(40), secs(600));
    println!("40 s: {seen:?}");
    assert_eq!(seen.samples, 0, "{seen:?}");
    assert!(seen.most_owed <= 4, "{seen:?}");
}

// Whole captures on the fake.

#[test]
fn init_then_one_request_per_sample_and_none_past_the_limit() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([
            Reply::Now(GOOD_INIT.to_vec()),
            Reply::Now(OK.to_vec()),
            Reply::Now(OK2.to_vec()),
        ]);
        let mut d = device(&fake);
        limit(&mut d, 2);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_OK, V_OK2]);
        d.close().unwrap();
        assert_eq!(fake.writes(), [INIT.to_vec(), REQUEST.to_vec(), REQUEST.to_vec()]);
    });
}

#[test]
fn a_packet_says_what_c_says() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::Now(vec![0x89, 0xcb, 0x01, 0x0d])]);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let mut sent = Vec::new();
        loop {
            match rx.recv_timeout(BOUND).expect("no End").1 {
                Packet::Analog(a) => sent.push(a),
                Packet::End => break,
                _ => {}
            }
        }
        assert_eq!(sent.len(), 1);
        let a = &sent[0];
        assert_eq!((a.channels.as_slice(), a.samples.as_slice()), (&[0][..], &[45.9f32][..]));
        assert_eq!(a.mq, Mq::SoundPressureLevel);
        assert_eq!(a.unit, Unit::DecibelSpl);
        assert_eq!(a.digits, 1);
        assert_eq!(a.flags, MqFlags::SPL_FREQ_WEIGHT_C | MqFlags::SPL_TIME_WEIGHT_S);
    });
}

fn failure(fake: &Fake, limit_to: u64) -> Seen {
    let timing = timing(Duration::from_millis(1), Duration::ZERO, ms(30));
    let mut d = device_with(fake, timing);
    limit(&mut d, limit_to);
    let rx = start(&mut d);
    seen(&rx)
}

#[test]
fn each_silence_fails_the_capture_in_its_own_words() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Silent]);
        let got = failure(&fake, 1);
        assert_eq!(got.failed.as_deref(), Some("device error: no answer to init for 0.03 s"));
        assert_eq!(fake.writes(), [INIT.to_vec()], "nothing asked twice");

        let fake = Fake::new();
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::Silent]);
        let got = failure(&fake, 1);
        assert_eq!(got.failed.as_deref(), Some("device error: no answer to a request for 0.03 s"));
        assert_eq!(fake.writes(), [INIT.to_vec(), REQUEST.to_vec()], "nothing asked twice");

        let fake = Fake::new();
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::Now(OK[..2].to_vec())]);
        let got = failure(&fake, 1);
        assert_eq!(got.failed.as_deref(), Some("device error: no usable answer for 0.03 s"));
    });
}

#[test]
fn half_an_init_reply_is_no_usable_answer() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Now(GOOD_INIT[..1].to_vec())]);
        let got = failure(&fake, 1);
        assert_eq!(got.failed.as_deref(), Some("device error: no usable answer for 0.03 s"));
    });
}

#[test]
fn wrong_replies_for_ever_fail_with_samples_unmet() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.otherwise = vec![1, 2, 3, 4]);
        let got = failure(&fake, 1);
        assert_eq!(got.values, Vec::<f32>::new());
        assert_eq!(got.failed.as_deref(), Some("device error: no usable answer for 0.03 s"));
    });
}

#[test]
fn a_flood_of_bytes_fails_a_samples_capture_and_a_stop_ends_one() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.endless = Some(0xaa));
        let got = failure(&fake, 1);
        assert_eq!(got.failed.as_deref(), Some("device error: no usable answer for 0.03 s"));
        assert_eq!(fake.writes(), Vec::<Vec<u8>>::new(), "the line never went quiet");

        // Far from the bound: only the stop can end this.
        let fake = Fake::new();
        fake.with(|s| s.endless = Some(0xaa));
        let mut d = device(&fake);
        let rx = start(&mut d);
        d.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
    });
}

#[test]
fn a_time_limit_ends_a_silent_wait_with_no_failure_having_asked_again() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.otherwise = Vec::new());
        fake.script([Reply::Now(GOOD_INIT.to_vec())]);
        let mut d = device_with(&fake, timing(ms(2), Duration::ZERO, ms(20)));
        d.set(&Key::LimitMsec, &Value::U64(80), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), Vec::<f32>::new());
        d.close().unwrap();
        // How far it got is the clock's: what went is in this order, and the asking
        // again is pinned where nothing is timed, and in a golden.
        let writes = fake.writes();
        let mut want = vec![INIT.to_vec(), REQUEST.to_vec()];
        want.extend((1..=writes.len() as u8).map(|zz| vec![0x30, zz, 0x0d]));
        assert_eq!(writes, want[..writes.len()]);
    });
}

/// The reviewer's: a meter that answers once and loses one reply. C never ends.
#[test]
fn a_lost_reply_is_asked_again_and_the_capture_goes_on() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::Now(OK.to_vec()), Reply::Silent]);
        let mut d = device_with(&fake, timing(ms(2), Duration::ZERO, ms(30)));
        limit(&mut d, 3);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_OK; 3]);
        d.close().unwrap();
        let (i, r) = (INIT.to_vec(), REQUEST.to_vec());
        assert_eq!(fake.writes(), [i, r.clone(), r.clone(), vec![0x30, 1, 0x0d], r]);
    });
}

#[test]
fn with_no_limit_a_dead_meter_goes_on_until_a_stop() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.otherwise = Vec::new());
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::Now(OK.to_vec())]);
        let mut d = device_with(&fake, timing(ms(2), Duration::ZERO, ms(5)));
        let rx = start(&mut d);
        // Several waits past any bound a sample limit would have.
        fake.wait_for_writes(6);
        d.stop().unwrap();
        assert_eq!(readings(&rx), [V_OK]);
    });
}

#[test]
fn past_a_sample_a_dead_meter_fails_a_sample_limit_at_the_long_bound() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.otherwise = Vec::new());
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::Now(OK.to_vec())]);
        let timing = Timing { give_up: ms(50), ..timing(ms(2), Duration::ZERO, ms(5)) };
        let mut d = device_with(&fake, timing);
        limit(&mut d, 2);
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(got.values, [V_OK]);
        assert_eq!(got.failed.as_deref(), Some("device error: no answer to a request for 0.05 s"));
        d.close().unwrap();
        assert!(fake.writes().len() >= 3, "{:02x?}", fake.writes());
    });
}

/// A stall past the bound just after a read found nothing, with the answer in by
/// then: the look that counts is the next, so no bound holds against it.
#[test]
fn a_stall_after_an_empty_read_is_no_silence() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([
            Reply::AcrossAStall(GOOD_INIT.to_vec(), ms(60)),
            Reply::AcrossAStall(OK.to_vec(), ms(60)),
        ]);
        let mut d = device_with(&fake, timing(ms(1), Duration::ZERO, ms(30)));
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_OK]);
    });
}

/// A write that a line holds past the bound: a bound counted from before it fails a
/// meter that answers as soon as it hears.
#[test]
fn a_write_held_past_the_bound_is_no_silence() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| {
            s.write_blocks = ms(60);
            s.spacing = Spacing::Line(Duration::from_millis(1));
        });
        let mut d = device_with(&fake, timing(ms(1), Duration::ZERO, ms(30)));
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_OK]);
    });
}

#[test]
fn a_time_limit_ends_a_capture_flooded_with_bytes() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.endless = Some(0xaa));
        let mut d = device(&fake);
        d.set(&Key::LimitMsec, &Value::U64(20), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), Vec::<f32>::new());
    });
}

#[test]
fn a_read_that_fails_fails_the_capture() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.fail_reads = true);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(seen(&rx).failed.as_deref(), Some("device error: hung up"));
    });
}

/// With nothing out, C reads nothing, so a gone port is found by its next write, in
/// its words.
#[test]
fn a_read_that_fails_with_nothing_out_leaves_it_to_the_write() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| {
            s.fail_reads = true;
            s.fail_writes_after = Some(0);
        });
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(seen(&rx).failed.as_deref(), Some("device error: Broken pipe"));
    });
}

#[test]
fn the_failures_of_the_port_say_what_c_says() {
    let e = Error::device("gone");
    for (said, want) in [
        (send_error_message(Command::Init, &e), "Error sending init command: device error: gone."),
        (
            send_error_message(Command::Request, &e),
            "Error sending request command: device error: gone.",
        ),
        (read_error_message(Command::Init, &e), "Error reading init reply: device error: gone."),
        (read_error_message(Command::Request, &e), "Error reading packet: device error: gone."),
    ] {
        assert_eq!(said, want);
    }
}

#[test]
fn a_write_that_fails_fails_the_capture_and_ends_the_writes() {
    bounded(|| {
        for after in [0, 1, 2] {
            let fake = Fake::new();
            fake.with(|s| s.fail_writes_after = Some(after));
            let mut d = device(&fake);
            limit(&mut d, 5);
            let rx = start(&mut d);
            let got = seen(&rx);
            assert_eq!(got.failed.as_deref(), Some("device error: Broken pipe"), "{after}");
            d.close().unwrap();
            assert_eq!(fake.writes().len(), after);
        }
    });
}

/// `serial_write_blocking` returns 0 on a timeout, which is no error in C: a meter
/// that does not read looks like a silent one.
#[test]
fn a_write_that_times_out_is_no_error_and_the_bound_finds_it() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.writes_time_out = true);
        let got = failure(&fake, 1);
        assert_eq!(got.failed.as_deref(), Some("device error: no answer to init for 0.03 s"));
    });
}

/// C reads `aa 12 34 01`, then `0d 12 34 01` and so on for good, so its capture
/// never ends. Only one byte goes wrong here: a second, on a list that came round,
/// brought C back into step.
#[test]
fn a_stray_byte_ahead_of_a_reply_costs_that_reading_only() {
    bounded(|| {
        for spacing in [Spacing::None, Spacing::Line(LINE_RATE)] {
            let fake = Fake::new();
            fake.with(|s| s.spacing = spacing);
            let mut stray = vec![0xaa];
            stray.extend(OK2);
            fake.script([
                Reply::Now(GOOD_INIT.to_vec()),
                Reply::Now(stray),
                Reply::Now(OK.to_vec()),
                Reply::Now(OK2.to_vec()),
            ]);
            let timing = timing(ms(15), ms(15), FAR);
            let mut d = device_with(&fake, timing);
            limit(&mut d, 2);
            let rx = start(&mut d);
            assert_eq!(readings(&rx), [V_OK, V_OK2], "{spacing:?}");
            d.close().unwrap();
            assert_eq!(fake.writes().len(), 4, "{spacing:?}");
        }
    });
}

#[test]
fn a_stray_byte_behind_a_reply_shifts_nothing() {
    bounded(|| {
        let fake = Fake::new();
        let mut stray = OK.to_vec();
        stray.push(0xaa);
        fake.script([
            Reply::Now(GOOD_INIT.to_vec()),
            Reply::Now(stray),
            Reply::Now(OK2.to_vec()),
            Reply::Now(OK.to_vec()),
        ]);
        let timing = timing(ms(15), ms(15), FAR);
        let mut d = device_with(&fake, timing);
        limit(&mut d, 3);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_OK, V_OK2, V_OK]);
        d.close().unwrap();
        assert_eq!(fake.writes().len(), 4);
    });
}

#[test]
fn a_wrong_init_is_asked_again_and_a_wrong_reply_again() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([
            Reply::Now(vec![0x06, 0x0d]),
            Reply::Now(GOOD_INIT.to_vec()),
            Reply::Now(vec![1, 2, 2, 0x0d]),
            Reply::Now(vec![1, 2, 1, 0x0a]),
            Reply::Now(OK.to_vec()),
        ]);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_OK]);
        d.close().unwrap();
        let (i, r) = (INIT.to_vec(), REQUEST.to_vec());
        assert_eq!(fake.writes(), [i.clone(), i, r.clone(), r.clone(), r]);
    });
}

/// The line's real rate, 1.15 ms a byte, and the real times: a capture of two
/// samples takes 1.5 s.
#[test]
fn at_the_lines_rate_with_the_real_pace_replies_come_whole() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.spacing = Spacing::Line(LINE_RATE));
        fake.script([
            Reply::Now(GOOD_INIT.to_vec()),
            Reply::Now(OK.to_vec()),
            Reply::Now(OK2.to_vec()),
        ]);
        let mut d = device_with(&fake, REAL);
        limit(&mut d, 2);
        let rx = start(&mut d);
        let t0 = Instant::now();
        assert_eq!(readings(&rx), [V_OK, V_OK2]);
        assert!(t0.elapsed() >= ms(1500), "{:?}", t0.elapsed());
    });
}

/// A cable that hands bytes over in bursts: a stray byte's reply comes in pieces,
/// and what the quiet reads away must be all of it.
#[test]
fn in_bursts_the_tail_of_a_bad_reply_is_read_away_before_the_next_request() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.spacing = Spacing::Burst(4));
        let mut stray = vec![0xaa, 0xbb];
        stray.extend(OK2);
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::Now(stray), Reply::Now(OK.to_vec())]);
        let timing = timing(QUIET, QUIET, FAR);
        let mut d = device_with(&fake, timing);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_OK]);
    });
}

#[test]
fn a_stop_ends_a_wait_and_nothing_is_written_after_a_close() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::OnRelease(OK.to_vec())]);
        let mut d = device(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(2);
        d.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
        fake.release();
        d.close().unwrap();
        let writes = fake.writes();
        std::thread::sleep(ms(20));
        assert_eq!(fake.writes(), writes);
        assert_eq!(writes.len(), 2);
    });
}

/// With no stop asked for and a meter still answering: only a close that joins ends
/// the writes and lets the port go by the time it returns.
#[test]
fn a_close_ends_a_running_capture_and_lets_the_port_go() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(3);
        d.close().unwrap();
        assert_eq!(Arc::strong_count(&fake.0), 1, "the capture still holds the port");
        let writes = fake.writes();
        std::thread::sleep(ms(20));
        assert_eq!(fake.writes(), writes);
        drop(rx);
    });
}

/// Far from any limit, data on offer, and a queue that fills: the send gives way.
#[test]
fn a_close_with_a_queue_no_one_reads_does_not_hang() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        fake.wait_for_writes(2);
        d.close().unwrap();
        drop(rx);
    });
}

/// The client holds the send for ten bounds. A reply was never late.
#[test]
fn a_slow_client_is_no_silence() {
    bounded(|| {
        let fake = Fake::new();
        let timing = Timing { give_up: ms(30), ..timing(ms(1), Duration::ZERO, ms(30)) };
        let mut d = device_with(&fake, timing);
        limit(&mut d, 3);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let mut values = Vec::new();
        let mut failed = None;
        loop {
            std::thread::sleep(ms(60));
            match rx.recv_timeout(BOUND).expect("no End").1 {
                Packet::Analog(a) => values.push(a.samples[0]),
                Packet::Failed(why) => failed = Some(why),
                Packet::End => break,
                _ => {}
            }
        }
        assert_eq!(failed, None);
        assert_eq!(values, [V_OK; 3]);
    });
}

/// A reply the last capture left on its way is no sample of the next, whether it
/// comes in the wait ahead of init or after it.
#[test]
fn a_late_reply_of_the_last_capture_is_no_sample() {
    bounded(|| {
        // In the wait ahead of init: read away.
        let fake = Fake::new();
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::OnRelease(OK2.to_vec())]);
        let timing = timing(ms(50), ms(5), FAR);
        let mut d = device_with(&fake, timing);
        let rx = start(&mut d);
        fake.wait_for_writes(2);
        d.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
        d.end_worker();
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::Now(OK.to_vec())]);
        limit(&mut d, 1);
        let rx = start(&mut d);
        fake.release();
        assert_eq!(readings(&rx), [V_OK]);
        d.close().unwrap();
        let (i, r) = (INIT.to_vec(), REQUEST.to_vec());
        assert_eq!(fake.writes(), [i.clone(), r.clone(), i, r]);

        // After init went: a wrong init reply, and init again.
        let fake = Fake::new();
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::OnRelease(OK2.to_vec())]);
        let mut d = device(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(2);
        d.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
        d.end_worker();
        let late = fake.with(|s| std::mem::take(&mut s.held));
        fake.script([Reply::Now(late), Reply::Now(GOOD_INIT.to_vec()), Reply::Now(OK.to_vec())]);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_OK]);
        d.close().unwrap();
        let (i, r) = (INIT.to_vec(), REQUEST.to_vec());
        assert_eq!(fake.writes(), [i.clone(), r.clone(), i.clone(), i, r]);
    });
}

/// C keeps its state from the last capture, and asks no init. A meter restarted
/// meanwhile is the reason not to: every sigrok-cli run sends one anyway.
#[test]
fn each_capture_starts_at_init() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        limit(&mut d, 1);
        for _ in 0..2 {
            let rx = start(&mut d);
            assert_eq!(readings(&rx), [V_OK]);
        }
        d.close().unwrap();
        let (i, r) = (INIT.to_vec(), REQUEST.to_vec());
        assert_eq!(fake.writes(), [i.clone(), r.clone(), i, r]);
    });
}

/// A start after a capture whose End is stuck on a full queue: the old worker's
/// stop is set and it is joined, and its sink goes with it.
#[test]
fn a_restart_gets_past_an_end_stuck_on_a_full_queue_and_lets_the_old_sink_go() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        limit(&mut d, 1);
        let (tx, old) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The header fills it; a reading and End wait.
        fake.wait_for("the capture to be stuck", |s| !s.writes.is_empty());
        let deadline = Instant::now() + BOUND;
        while d.capturing.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline, "the capture went on");
            let got = old.try_recv();
            if got.is_ok() {
                // One packet read lets the reading out; End then waits on the next.
                continue;
            }
            std::thread::sleep(ms(1));
        }
        let rx = start(&mut d);
        let _ = old.try_iter().count();
        assert!(old.recv_timeout(BOUND).is_err(), "the old sink was released");
        assert_eq!(readings(&rx), [V_OK]);
    });
}

#[test]
fn a_disabled_channel_sends_nothing_and_still_counts() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        d.enable_channel(0, false).unwrap();
        limit(&mut d, 2);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), Vec::<f32>::new());
        d.close().unwrap();
        assert_eq!(fake.writes().len(), 3);
    });
}

#[test]
fn a_start_with_the_device_closed_or_busy_is_refused() {
    let fake = Fake::new();
    let mut d = TondajDevice::from_port(Box::new(fake.clone()));
    let (tx, _rx) = sync_channel(4);
    assert!(d.start(PacketSink::new(tx, 0)).is_err());
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Silent]);
        let mut d = device(&fake);
        let _rx = start(&mut d);
        let (tx, _rx2) = sync_channel(4);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.close().unwrap();
    });
}

#[test]
fn the_driver_says_what_c_says() {
    let driver = TondajSl814Driver;
    assert_eq!(driver.id(), "tondaj-sl-814");
    assert_eq!(driver.long_name(), "Tondaj SL-814");
    assert_eq!(driver.scan_options(), [Key::Conn, Key::Serialcomm]);
    assert_eq!(driver.classes(), [DeviceClass::SoundLevelMeter]);
    assert!(driver.scan(&[]).unwrap().is_empty(), "no conn, no device");
    let gone = [(Key::Conn, Value::Str("/dev/ttyOXROKNONE".to_owned()))];
    assert!(driver.scan(&gone).unwrap().is_empty(), "a port that does not open");
}

#[test]
fn a_device_says_what_c_says() {
    let mut d = TondajDevice::from_port(Box::new(Fake::new()));
    assert_eq!((d.info().vendor.as_str(), d.info().model.as_str()), ("Tondaj", "SL-814"));
    assert_eq!(d.info().conn, None);
    assert_eq!(d.channels().len(), 1);
    assert_eq!(d.channels()[0].name, "P1");
    let keys: Vec<Key> = d.options(None).unwrap().into_iter().map(|o| o.key).collect();
    assert_eq!(keys, [Key::Continuous, Key::LimitSamples, Key::LimitMsec]);
    assert!(d.get(&Key::LimitSamples, None).is_err());
    assert!(d.list(&Key::LimitSamples, None).is_err());
    assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::LimitMsec, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::Samplerate, &Value::U64(1), None).is_err());
    d.set(&Key::LimitMsec, &Value::U64(9), None).unwrap();
    assert_eq!(d.limit_msec, 9);
    // What the scenario's filter `A` takes off the end of its line.
    let zero = crate::core::config_set(&mut d, &Key::LimitSamples, &Value::U64(0), None);
    assert_eq!(zero.unwrap_err().to_string(), "invalid argument: it cannot be 0");
}
