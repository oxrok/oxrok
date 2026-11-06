// Counterpart in sigrok:
//   libsigrok/src/hardware/tondaj-sl-814/api.c,
//   libsigrok/src/hardware/tondaj-sl-814/protocol.c,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Martin Ling,
//   Daniel Elstner, Gerhard Sittig, Soeren Apel, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

use super::protocol::{INIT, NO_ANSWER, PACE, QUIET, REQUEST, parse_reading};
use super::*;
use crate::core::{MqFlags, Packet};
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
    writes: Vec<Vec<u8>>,
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
        self.with(|s| s.writes.clone())
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
        let mut s = self.0.lock().unwrap();
        if s.fail_writes_after.is_some_and(|n| s.writes.len() >= n) {
            return Err(Error::device("Broken pipe"));
        }
        s.writes.push(data.to_vec());
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
            None => {
                let otherwise = s.otherwise.clone();
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
const QUICK: Timing =
    Timing { pace: Duration::from_millis(2), quiet: Duration::ZERO, no_answer: FAR };

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
}

#[test]
fn each_way_of_giving_up_has_its_own_words() {
    let after = Duration::from_secs(5);
    assert_eq!(GiveUp::NoInitReply.message(after), "no answer to init for 5 s");
    assert_eq!(GiveUp::NoReply.message(after), "no answer to a request for 5 s");
    assert_eq!(GiveUp::NoUsableAnswer.message(after), "no usable answer for 5 s");
    assert_eq!(time_limit_message(false), "No answer from the meter before the time limit.");
    assert_eq!(time_limit_message(true), "No usable answer from the meter before the time limit.");
}

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

const REAL: Timing = Timing { pace: PACE, quiet: QUIET, no_answer: NO_ANSWER };

#[test]
fn a_command_waits_its_pace_from_the_start_and_from_each_reply() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, t0);
    assert_eq!(x.empty_read(t0 + ms(499)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(500)), Next::Send(Command::Init));
    x.sent(Command::Init, t0 + ms(500));
    assert_eq!(x.want(64), 2);
    assert_eq!(x.take(&GOOD_INIT, t0 + ms(501)), Heard::InitOk);
    assert_eq!(x.empty_read(t0 + ms(1000)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(1001)), Next::Send(Command::Request));
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
fn a_wrong_reply_is_told_and_its_command_goes_again_at_the_pace() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, t0);
    x.sent(Command::Init, t0);
    assert_eq!(x.take(&[0x06, 0x0d], t0), Heard::BadInit([0x06, 0x0d]));
    assert_eq!(x.empty_read(t0 + ms(499)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(500)), Next::Send(Command::Init));
    x.sent(Command::Init, t0 + ms(500));
    assert_eq!(x.take(&GOOD_INIT[..1], t0 + ms(500)), Heard::Part);
    assert_eq!(x.take(&GOOD_INIT[1..], t0 + ms(500)), Heard::InitOk);
    x.sent(Command::Request, t0 + ms(1000));
    for (bad, what) in [([1, 2, 0x02, 0x0d], "sequence"), ([1, 2, 0x01, 0x0a], "end")] {
        assert_eq!(x.take(&bad, t0 + ms(1000)), Heard::BadReply(bad), "{what}");
        x.sent(Command::Request, t0 + ms(1000));
    }
}

#[test]
fn the_bound_on_a_reply_holds_from_both_sides_in_each_wait() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, t0);
    x.sent(Command::Init, t0 + ms(500));
    assert_eq!(x.empty_read(t0 + ms(5499)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(5500)), Next::GiveUp(GiveUp::NoInitReply));
    x.take(&GOOD_INIT, t0 + ms(5000));
    x.sent(Command::Request, t0 + ms(5500));
    assert_eq!(x.empty_read(t0 + ms(10_499)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(10_500)), Next::GiveUp(GiveUp::NoReply));
    x.take(&OK[..1], t0 + ms(10_500));
    assert_eq!(x.empty_read(t0 + ms(10_500)), Next::GiveUp(GiveUp::NoUsableAnswer));
}

/// Good replies for many bounds on a moved clock, with no real gaps anywhere: the
/// bound only holds if each one sets it back.
#[test]
fn a_meter_that_answers_is_never_given_up_on() {
    let t0 = Instant::now();
    let mut now = t0;
    let mut x = Exchange::new(REAL, t0);
    now += ms(500);
    assert_eq!(x.empty_read(now), Next::Send(Command::Init));
    x.sent(Command::Init, now);
    x.take(&GOOD_INIT, now);
    for _ in 0..1000 {
        now += ms(500);
        assert_eq!(x.empty_read(now), Next::Send(Command::Request));
        x.sent(Command::Request, now);
        now += ms(100);
        assert_eq!(x.empty_read(now), Next::Wait);
        assert!(matches!(x.take(&OK, now), Heard::Reading(..)));
        x.delivered(now);
    }
    assert!(!x.junk_too_long(now));
}

/// A client that kept the send waiting: that is no silence of the meter's.
#[test]
fn time_in_a_blocked_send_counts_for_nothing() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, t0);
    x.sent(Command::Request, t0);
    assert!(matches!(x.take(&OK, t0), Heard::Reading(..)));
    let after = t0 + Duration::from_secs(60);
    x.delivered(after);
    assert_eq!(x.empty_read(after + ms(499)), Next::Wait);
    assert_eq!(x.empty_read(after + ms(500)), Next::Send(Command::Request));
    assert!(!x.junk_too_long(after + ms(500)));
    // And a whole bound lost ahead of a request's return: its reply has its full time.
    x.sent(Command::Request, after + Duration::from_secs(60));
    assert_eq!(x.empty_read(after + Duration::from_secs(61)), Next::Wait);
}

/// Wrong replies at the pace: each request is progress for nothing, and they end.
#[test]
fn wrong_replies_run_out_their_bound_though_requests_go() {
    let t0 = Instant::now();
    let mut now = t0;
    let mut x = Exchange::new(REAL, t0);
    x.sent(Command::Init, now);
    x.take(&GOOD_INIT, now);
    let mut rounds = 0;
    loop {
        now += ms(500);
        assert_eq!(x.empty_read(now), Next::Send(Command::Request));
        x.sent(Command::Request, now);
        assert!(matches!(x.take(&[1, 2, 3, 4], now), Heard::BadReply(_)));
        rounds += 1;
        if x.junk_too_long(now) {
            break;
        }
        assert!(rounds < 100, "no end");
    }
    assert_eq!(rounds, 10, "5 s at 500 ms");
}

#[test]
fn a_flood_with_nothing_out_holds_the_command_back_and_runs_out_its_bound() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, t0);
    assert_eq!(x.want(64), 64);
    assert_eq!(x.take(&[0xaa], t0 + ms(450)), Heard::Dropped);
    // What came moves a command that was due, by the quiet.
    assert_eq!(x.empty_read(t0 + ms(549)), Next::Wait);
    assert_eq!(x.empty_read(t0 + ms(550)), Next::Send(Command::Init));
    let mut now = t0 + ms(450);
    while !x.junk_too_long(now) {
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
    let x = Exchange::new(REAL, t0);
    assert_eq!(x.read_wait(ms(5), t0), ms(5));
    assert_eq!(x.read_wait(ms(5), t0 + ms(498)), ms(2));
    assert_eq!(x.read_wait(ms(5), t0 + ms(900)), Duration::ZERO);
}

#[test]
fn a_time_limit_takes_a_wait_only_when_a_pass_of_c_would_have_found_nothing() {
    let t0 = Instant::now();
    let mut x = Exchange::new(REAL, t0);
    assert!(!x.overdue(t0 + Duration::from_secs(60)), "nothing out");
    x.sent(Command::Init, t0);
    assert!(!x.overdue(t0 + ms(499)));
    assert!(x.overdue(t0 + ms(500)));
}

/// A meter on its own clock, 2.5 s behind every request, for a simulated minute. C
/// and this have one out, as neither asks again on a silence.
#[test]
fn a_steady_latency_of_2500_ms_owes_one_reply_and_ages_a_reading_by_that() {
    let t0 = Instant::now();
    let latency = ms(2500);
    let mut x = Exchange::new(REAL, t0);
    let mut now = t0;
    let mut sent_at = VecDeque::new();
    let (mut most_owed, mut samples, mut oldest) = (0, 0, Duration::ZERO);
    let mut commands = 0;
    while now < t0 + Duration::from_secs(60) {
        now += ms(1);
        if sent_at.front().is_some_and(|at| now.saturating_duration_since(*at) >= latency) {
            let at: Instant = sent_at.pop_front().expect("counted");
            let reply: &[u8] = if commands == 1 { &GOOD_INIT } else { &OK };
            match x.take(reply, now) {
                Heard::InitOk => {}
                Heard::Reading(..) => {
                    x.delivered(now);
                    samples += 1;
                    oldest = oldest.max(now.saturating_duration_since(at));
                }
                other => panic!("{other:?}"),
            }
            continue;
        }
        match x.empty_read(now) {
            Next::Wait => {}
            Next::Send(command) => {
                x.sent(command, now);
                sent_at.push_back(now);
                commands += 1;
                most_owed = most_owed.max(sent_at.len());
            }
            Next::GiveUp(why) => panic!("{why:?}"),
        }
    }
    assert_eq!(most_owed, 1);
    // 3 s a round after the init's.
    assert!((18..=20).contains(&samples), "{samples}");
    assert_eq!(oldest, latency);
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
    let timing =
        Timing { pace: Duration::from_millis(1), quiet: Duration::ZERO, no_answer: ms(30) };
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
fn a_time_limit_ends_a_silent_wait_with_no_failure() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Now(GOOD_INIT.to_vec()), Reply::Silent]);
        let mut d = device(&fake);
        d.set(&Key::LimitMsec, &Value::U64(20), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), Vec::<f32>::new());
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
            let timing = Timing { pace: ms(15), quiet: ms(15), no_answer: FAR };
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
        let timing = Timing { pace: ms(15), quiet: ms(15), no_answer: FAR };
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
        let timing = Timing { pace: QUIET, quiet: QUIET, no_answer: FAR };
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
        let timing = Timing { pace: ms(1), quiet: Duration::ZERO, no_answer: ms(30) };
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
        let timing = Timing { pace: ms(50), quiet: ms(5), no_answer: FAR };
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
}
