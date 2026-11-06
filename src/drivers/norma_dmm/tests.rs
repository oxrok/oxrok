// Counterpart in sigrok:
//   libsigrok/src/hardware/norma-dmm/api.c,
//   libsigrok/src/hardware/norma-dmm/protocol.c,
//   Copyright (C) 2013 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Martin Ling, Soeren Apel, Daniel Elstner, Gerhard Sittig, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

use super::*;
use crate::core::{Mq, MqFlags, Unit};
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test waits for what must happen: a stop or a limit that does not
/// bite must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);
/// The wiki's line: 24.91 V DC.
const S: &[u8] = b"03030024913100400040\r\n";
/// 0.01 V DC.
const S2: &[u8] = b"00030000013100400040\r\n";
const V_S: f32 = 2491.0 * 0.01;
const V_S2: f32 = 1.0 * 0.01;

/// What the meter does with a request.
#[derive(Clone, Debug)]
enum Reply {
    /// Those bytes, at the fake's pace.
    Now(Vec<u8>),
    /// Nothing.
    Silent,
    /// Those bytes, once the next request has been written.
    AtNext(Vec<u8>),
    /// Those bytes, when the test says so.
    OnRelease(Vec<u8>),
}

#[derive(Default)]
struct State {
    writes: crate::serial::WriteLog,
    script: VecDeque<Reply>,
    /// What every request gets once the script is out, if anything.
    otherwise: Option<Vec<u8>>,
    /// Bytes on their way.
    input: VecDeque<u8>,
    /// Bytes that come at the next request.
    at_next: Vec<u8>,
    /// Bytes that come when the test says so.
    held: Vec<u8>,
    reads: u64,
    delivered: usize,
    /// One byte per this many reads; 0 for all at once.
    pace: u64,
    fail_writes_after: Option<usize>,
    /// A write that times out, which is no error.
    writes_time_out: bool,
    fail_reads: bool,
    /// Reads never run out of this.
    endless: Option<u8>,
}

/// A meter whose bytes come on the Nth read, not at a clock time.
#[derive(Clone)]
struct Fake(Arc<Mutex<State>>);

impl Fake {
    fn new() -> Fake {
        let state = State { otherwise: Some(S.to_vec()), ..State::default() };
        Fake(Arc::new(Mutex::new(state)))
    }

    /// At about the line's rate: a read of nothing takes 1 ms, 4800 baud is 2 ms
    /// a byte.
    fn at_4800() -> Fake {
        let fake = Fake::new();
        fake.with(|s| s.pace = 2);
        fake
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
            s.input.extend(held);
        });
    }

    fn writes(&self) -> Vec<Vec<u8>> {
        self.with(|s| s.writes.iter().map(|(_, w)| w.clone()).collect())
    }

    fn write_times(&self) -> Vec<Instant> {
        self.with(|s| s.writes.iter().map(|(at, _)| *at).collect())
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
            s.reads += 1;
            let room =
                if s.pace == 0 { buf.len() } else { usize::from(s.reads.is_multiple_of(s.pace)) };
            let n = s.input.len().min(buf.len()).min(room);
            for b in &mut buf[..n] {
                *b = s.input.pop_front().expect("counted");
            }
            s.delivered += n;
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
        s.writes.push((Instant::now(), data.to_vec()));
        let next = std::mem::take(&mut s.at_next);
        s.input.extend(next);
        if s.writes_time_out {
            return Ok(0);
        }
        match s.script.pop_front() {
            Some(Reply::Now(bytes)) => s.input.extend(bytes),
            Some(Reply::Silent) => {}
            Some(Reply::AtNext(bytes)) => s.at_next = bytes,
            Some(Reply::OnRelease(bytes)) => s.held = bytes,
            None => {
                let otherwise = s.otherwise.clone().unwrap_or_default();
                s.input.extend(otherwise);
            }
        }
        Ok(data.len())
    }
    fn purge(&mut self) -> Result<()> {
        self.0.lock().unwrap().input.clear();
        Ok(())
    }
}

type Received = Receiver<(usize, Packet)>;

fn device(fake: &Fake) -> NormaDevice {
    let mut d = NormaDevice::from_port(Box::new(fake.clone()), 5);
    d.open().unwrap();
    d
}

fn start(d: &mut NormaDevice) -> Received {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

fn limit(d: &mut NormaDevice, samples: u64) {
    d.set(&Key::LimitSamples, &Value::U64(samples), None).unwrap();
}

/// The values a capture sends up to its End, which must come within [`BOUND`].
fn readings(rx: &Received) -> Vec<f32> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left).expect("no End in time").1 {
            Packet::Analog(a) => out.push(a.samples[0]),
            Packet::End => return out,
            _ => {}
        }
    }
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
        // A panic drops the sender.
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(handle.join().unwrap_err());
        }
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("the test hung"),
    }
}

fn params() -> Params {
    Params::parse(SERIALCOMM).unwrap()
}

fn waits(read_ms: u64, charge_ms: u64) -> ScanWaits {
    ScanWaits {
        settle: Duration::ZERO,
        charge: Duration::from_millis(charge_ms),
        read: Duration::from_millis(read_ms),
    }
}

const DM950: &[u8] = b"1834 065 V1.06,IF V1.02\r\n";

#[test]
fn the_literals_are_c_s() {
    assert_eq!(SERIALCOMM, "4800/8n1/dtr=1/rts=0/flow=1");
    assert_eq!(REQUEST_TIMEOUT, Duration::from_millis(2000));
    assert_eq!(POLL_INTERVAL, Duration::from_millis(100));
    assert_eq!(SCAN_WAITS.settle, Duration::from_millis(150));
    assert_eq!(SCAN_WAITS.charge, Duration::from_secs(5));
    assert_eq!(SCAN_WAITS.read, Duration::from_millis(2000));
    assert_eq!(ID_TRIES, 7);
    assert_eq!(IDN, b"IDN?\r\n");
    assert_eq!(STATUS, b"STATUS?\r\n");
}

#[test]
fn the_drivers_are_named_as_c_s() {
    assert_eq!(NormaDmmDriver.id(), "norma-dmm");
    assert_eq!(NormaDmmDriver.long_name(), "Norma DM9x0 DMMs");
    assert_eq!(SiemensB102xDriver.id(), "siemens-b102x");
    assert_eq!(SiemensB102xDriver.long_name(), "Siemens B102x DMMs");
    assert!(NormaDmmDriver.scan(&[]).unwrap().is_empty());
    let bad = [(Key::Conn, Value::from("/dev/ttyFAKE")), (Key::Serialcomm, Value::from("fast"))];
    assert!(SiemensB102xDriver.scan(&bad).unwrap().is_empty());
}

#[test]
fn a_scan_finds_a_meter_at_the_first_try() {
    let mut fake = Fake::new();
    fake.script([Reply::Now(DM950.to_vec())]);
    let found = probe(&mut fake, params(), waits(1000, 0));
    assert_eq!(found, Some((5, "V1.06,IF V1.02".to_owned())));
    assert_eq!(fake.writes(), [IDN.to_vec()]);
}

#[test]
fn a_silent_port_gets_seven_tries_and_no_wait_of_5_s() {
    bounded(|| {
        let mut fake = Fake::new();
        fake.with(|s| s.otherwise = None);
        // A wait would outlast the test's own bound.
        let found = probe(&mut fake, params(), waits(5, 3_600_000));
        assert_eq!(found, None);
        assert_eq!(fake.writes().len(), 7);
    });
}

#[test]
fn under_cr_lf_junk_takes_two_tries_each_and_no_wait() {
    bounded(|| {
        let mut fake = Fake::new();
        fake.with(|s| s.otherwise = Some(b"junk\r\n".to_vec()));
        assert_eq!(probe(&mut fake, params(), waits(5, 3_600_000)), None);
        assert_eq!(fake.writes().len(), 7);
    });
}

#[test]
fn under_lf_the_fourth_try_waits() {
    let mut fake = Fake::new();
    fake.with(|s| s.otherwise = Some(b"junk\n".to_vec()));
    assert_eq!(probe(&mut fake, params(), waits(50, 60)), None);
    let times = fake.write_times();
    assert_eq!(times.len(), 7);
    let gap = |i: usize| times[i + 1] - times[i];
    assert!(gap(3) >= Duration::from_millis(60), "{:?}", gap(3));
    assert!(gap(2) < gap(3), "{:?} {:?}", gap(2), gap(3));
}

#[test]
fn a_failed_write_ends_the_scan() {
    // A meter's answer waits behind the junk: a scan that goes on reads it.
    let mut fake = Fake::new();
    fake.with(|s| {
        s.otherwise = None;
        s.fail_writes_after = Some(1);
        s.input.extend(b"junk\n");
        s.input.extend(DM950);
    });
    assert_eq!(probe(&mut fake, params(), waits(5, 0)), None);
    assert_eq!(fake.writes().len(), 1);
    assert_eq!(fake.with(|s| s.input.len()), DM950.len());
}

#[test]
fn a_scan_write_that_times_out_is_no_error() {
    let mut fake = Fake::new();
    fake.with(|s| s.writes_time_out = true);
    assert_eq!(probe(&mut fake, params(), waits(5, 0)), None);
    assert_eq!(fake.writes().len(), 7);
}

#[test]
fn a_line_stops_at_its_end_or_at_what_fits_or_at_its_time() {
    let mut fake = Fake::new();
    fake.with(|s| s.input.extend(b"ab\r\ncd\n"));
    assert_eq!(read_line(&mut fake, BOUND), b"ab");
    assert_eq!(read_line(&mut fake, BOUND), b"");
    assert_eq!(read_line(&mut fake, BOUND), b"cd");
    fake.with(|s| s.input.extend([b'x'; 60]));
    assert_eq!(read_line(&mut fake, BOUND).len(), 49);
    assert_eq!(fake.with(|s| s.input.len()), 11);
    fake.with(|s| s.input.clear());
    fake.with(|s| s.input.extend(b"part"));
    assert_eq!(read_line(&mut fake, Duration::from_millis(5)), b"part");
}

#[test]
fn a_read_that_fails_counts_as_nothing_until_its_time() {
    let mut fake = Fake::new();
    fake.with(|s| s.fail_reads = true);
    let began = Instant::now();
    assert_eq!(read_line(&mut fake, Duration::from_millis(20)), b"");
    assert!(began.elapsed() >= Duration::from_millis(20));
}

#[test]
fn a_capture_asks_once_per_reading_and_no_more() {
    bounded(|| {
        let fake = Fake::at_4800();
        let mut d = device(&fake);
        limit(&mut d, 3);
        let began = Instant::now();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S, V_S, V_S]);
        assert_eq!(fake.writes(), [STATUS.to_vec(), STATUS.to_vec(), STATUS.to_vec()]);
        assert!(fake.write_times()[0] - began >= Duration::from_millis(100));
        d.close().unwrap();
    });
}

#[test]
fn a_sample_has_what_the_line_says() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let deadline = Instant::now() + BOUND;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(left).expect("no sample").1 {
                Packet::Analog(a) => {
                    assert_eq!(a.channels, [0]);
                    assert_eq!(a.samples, [V_S]);
                    assert_eq!(a.mq, Mq::Voltage);
                    assert_eq!(a.unit, Unit::Volt);
                    assert_eq!(a.flags, MqFlags::DC | MqFlags::AUTORANGE);
                    assert_eq!(a.digits, 2);
                    break;
                }
                Packet::End => panic!("End before the sample"),
                _ => {}
            }
        }
        d.close().unwrap();
    });
}

#[test]
fn what_is_no_sample_does_not_count_and_the_next_request_goes() {
    bounded(|| {
        let fake = Fake::at_4800();
        fake.script([
            Reply::Now(b"03032024913100400040\r\n".to_vec()),
            Reply::Now(b"03030024913400400040\r\n".to_vec()),
            Reply::Now(b"abc\r\n".to_vec()),
            Reply::Now(b"\r\n".to_vec()),
            Reply::Now(S2.to_vec()),
        ]);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S2]);
        assert_eq!(fake.writes().len(), 5);
        d.close().unwrap();
    });
}

#[test]
fn a_time_limit_ends_a_silent_capture_and_only_the_time_does() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.otherwise = None);
        let mut d = device(&fake);
        d.set(&Key::LimitMsec, &Value::U64(150), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [0.0f32; 0]);
        assert!(began.elapsed() >= Duration::from_millis(150));
        // A stall can take the first pass past the limit, and then nothing goes.
        assert!(fake.writes().len() <= 1);
        d.close().unwrap();
    });
}

#[test]
fn a_stop_ends_a_capture_that_has_data() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(3);
        d.stop().unwrap();
        let _ = readings(&rx);
        let after = fake.writes().len();
        d.close().unwrap();
        assert_eq!(fake.writes().len(), after);
    });
}

#[test]
fn a_stop_ends_a_capture_under_a_flood_with_no_line_end() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.endless = Some(b'0'));
        let mut d = device(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        d.stop().unwrap();
        assert_eq!(readings(&rx), [0.0f32; 0]);
        d.close().unwrap();
    });
}

/// 2 s, from both sides: no sooner, and a bound on later.
#[test]
fn a_request_expires_at_2_s_and_the_next_goes() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Silent]);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S]);
        let times = fake.write_times();
        assert_eq!(times.len(), 2);
        let gap = times[1] - times[0];
        assert!(gap >= Duration::from_millis(2000), "{gap:?}");
        assert!(gap < BOUND, "{gap:?}");
        d.close().unwrap();
    });
}

fn quick(fake: &Fake) -> NormaDevice {
    let mut d = device(fake);
    d.request_timeout = Duration::from_millis(40);
    d
}

/// C reads 03030024 and the next reply as one, and cuts that at 20 digits: 24.03.
#[test]
fn part_of_a_line_is_dropped_when_its_request_expires() {
    bounded(|| {
        let fake = Fake::at_4800();
        fake.script([Reply::Now(b"03030024".to_vec())]);
        // The real 2 s: a reply at the line's rate has to fit inside a request's
        // life at any speed of this machine.
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S]);
        assert_eq!(fake.writes().len(), 2);
        d.close().unwrap();
    });
}

#[test]
fn two_replies_run_together_are_no_reading() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Now(b"0303002403030024913100400040\r\n".to_vec())]);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S]);
        assert_eq!(fake.writes().len(), 2);
        d.close().unwrap();
    });
}

/// C stops reading for good at the 255th byte.
#[test]
fn a_line_that_fills_the_buffer_is_one_bad_line() {
    bounded(|| {
        let fake = Fake::new();
        let mut long = vec![b'0'; 1000];
        long.extend_from_slice(b"\r\n");
        fake.script([Reply::Now(long)]);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S]);
        assert_eq!(fake.writes().len(), 2);
        d.close().unwrap();
    });
}

#[test]
fn the_buffer_holds_255_with_its_bound() {
    let mut b = LineBuffer::default();
    for _ in 0..254 {
        assert_eq!(b.push(b'0'), None);
    }
    assert_eq!(b.push(b'\n').map(|l| l.len()), Some(255));
    for _ in 0..1000 {
        assert_eq!(b.push(b'0'), None);
    }
    assert_eq!(b.bytes.len(), 255);
    assert_eq!(b.push(b'\n').map(|l| l.len()), Some(255));
}

/// A reply is a whole status, so one that is late is a true reading that is
/// old, and nothing after it shifts.
#[test]
fn a_late_reply_shifts_nothing() {
    bounded(|| {
        let fake = Fake::at_4800();
        fake.script([Reply::AtNext(S2.to_vec())]);
        let mut d = device(&fake);
        limit(&mut d, 3);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S2, V_S, V_S]);
        d.close().unwrap();
    });
}

#[test]
fn a_late_reply_asks_nothing_until_the_other_is_answered() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::AtNext(S2.to_vec()), Reply::OnRelease(S.to_vec())]);
        // The real 2 s: the second one's wait, then 4 s, must outlast the test's
        // look at the writes at any speed of this machine.
        let mut d = device(&fake);
        limit(&mut d, 2);
        let rx = start(&mut d);
        fake.wait_for("the late reply to be read", |s| s.delivered >= S2.len());
        // Its line is in; the second request is out, and nothing more goes.
        std::thread::sleep(Duration::from_millis(10));
        assert_eq!(fake.writes().len(), 2);
        fake.release();
        assert_eq!(readings(&rx), [V_S2, V_S]);
        assert_eq!(fake.writes().len(), 2);
        d.close().unwrap();
    });
}

/// One late reply, a limit on it, a pause: with the other reply on the wire, C and
/// a port that forgets the count take it as the next capture's first reading.
#[test]
fn every_reply_left_behind_is_dropped_whatever_the_gap() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::AtNext(S2.to_vec()), Reply::Now(S2.to_vec())]);
        let mut d = quick(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S2]);
        fake.wait_for("the other reply", |s| s.input.len() == S2.len());
        // Longer than a request's life, which must not give the old one a pass.
        std::thread::sleep(Duration::from_millis(120));
        d.request_timeout = Duration::from_secs(3600);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S]);
        assert_eq!(fake.writes().len(), 3);
        d.close().unwrap();
    });
}

/// What was left may be lost, not late: the expiry stops looking for it.
#[test]
fn a_reply_left_behind_that_never_comes_costs_one_wait() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Silent]);
        let mut d = quick(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        d.stop().unwrap();
        assert_eq!(readings(&rx), [0.0f32; 0]);
        let asked = fake.writes().len();
        limit(&mut d, 1);
        fake.script([Reply::Now(S2.to_vec())]);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S2]);
        assert_eq!(fake.writes().len(), asked + 1);
        d.close().unwrap();
    });
}

/// A meter at `latency` of the time in ms, in order, on a clock of our own, so no
/// machine can stretch it. `dead` holds the requests until then; `lost` loses those
/// numbered so. What it says: the most and the last replies due, and requests sent.
fn simulate(
    latency: impl Fn(u64) -> u64,
    dead: u64,
    lost: &[u32],
    limit_ms: u64,
) -> (usize, usize, u32) {
    let base = Instant::now();
    let at = |ms: u64| base + Duration::from_millis(ms);
    let mut r = Requests::new(0, REQUEST_TIMEOUT, base);
    let mut due: VecDeque<u64> = VecDeque::new();
    let mut held = 0u32;
    let mut free = 0;
    let (mut most, mut sent) = (0, 0u32);
    for t in (100..limit_ms).step_by(10) {
        if t >= dead {
            for _ in 0..std::mem::take(&mut held) {
                free = free.max(t) + latency(t) + 46;
                due.push_back(free);
            }
        }
        // A pass reads one line.
        if due.front().is_some_and(|d| *d <= t) {
            due.pop_front();
            r.line(at(t));
        }
        let turn = r.turn(at(t));
        if turn == Turn::Expired {
            r.expire();
        }
        if turn != Turn::Wait {
            sent += 1;
            r.asked(at(t));
            if t < dead {
                held += 1;
            } else if !lost.contains(&sent) {
                free = free.max(t) + latency(t) + 46;
                due.push_back(free);
            }
        }
        most = most.max(due.len() + held as usize);
    }
    (most, due.len() + held as usize, sent)
}

#[test]
fn a_steady_latency_past_the_expiry_does_not_pile_up() {
    let (most, _, sent) = simulate(|_| 2500, 0, &[], 60_000);
    assert!(most <= 3, "{most}");
    // It keeps reading, at about the meter's rate.
    assert!(sent >= 20, "{sent}");
}

#[test]
fn what_piled_up_drains_when_the_meter_is_back() {
    let (most, last, sent) = simulate(|t| if t < 20_000 { 2500 } else { 200 }, 0, &[], 45_000);
    assert!(most <= 3, "{most}");
    assert!(last <= 1, "{last}");
    assert!(sent >= 70, "{sent}");
    let (most, last, sent) = simulate(|_| 200, 10_000, &[], 40_000);
    assert!(most <= 4, "{most}");
    assert!(last <= 1, "{last}");
    assert!(sent >= 70, "{sent}");
}

/// The rate comes back: a count that stayed up by one would ask once per wait.
#[test]
fn a_lost_reply_costs_one_wait_and_not_the_rate() {
    let (_, last, sent) = simulate(|_| 100, 0, &[3], 30_000);
    assert!(last <= 1, "{last}");
    assert!(sent >= 120, "{sent}");
}

/// Literal times, as a changed constant must fail.
#[test]
fn an_unanswered_meter_is_asked_after_2_4_8_8_s() {
    let base = Instant::now();
    let at = |ms: u64| base + Duration::from_millis(ms);
    let mut r = Requests::new(0, REQUEST_TIMEOUT, base);
    assert_eq!(r.turn(at(100)), Turn::Ask);
    r.asked(at(100));
    let mut last = 100;
    for wait in [2000, 4000, 8000, 8000] {
        assert_eq!(r.turn(at(last + wait)), Turn::Wait, "{wait}");
        assert_eq!(r.turn(at(last + wait + 1)), Turn::Expired, "{wait}");
        r.expire();
        last += wait + 1;
        r.asked(at(last));
    }
    assert_eq!(r.in_flight, 5);
}

#[test]
fn a_reply_in_good_time_sets_the_wait_back() {
    let base = Instant::now();
    let at = |ms: u64| base + Duration::from_millis(ms);
    let mut r = Requests::new(0, REQUEST_TIMEOUT, base);
    r.asked(at(0));
    r.expire();
    assert_eq!(r.wait, Duration::from_secs(4));
    r.asked(at(2001));
    // In good time, with the first one still counted: not yet.
    r.line(at(2100));
    assert_eq!(r.wait, Duration::from_secs(4));
    assert_eq!(r.turn(at(2100)), Turn::Wait);
    r.line(at(2200));
    assert_eq!(r.wait, Duration::from_secs(2));
    assert_eq!(r.turn(at(2200)), Turn::Ask);
}

#[test]
fn a_line_too_slow_keeps_the_wait() {
    let base = Instant::now();
    let at = |ms: u64| base + Duration::from_millis(ms);
    let mut r = Requests::new(0, REQUEST_TIMEOUT, base);
    r.asked(at(0));
    r.expire();
    r.asked(at(2001));
    r.line(at(2500));
    r.line(at(5000));
    assert_eq!(r.wait, Duration::from_secs(4));
}

#[test]
fn an_expiry_after_a_line_counts_the_rest_as_lost_and_the_wait_stays() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_TIMEOUT, base);
    r.asked(base);
    r.asked(base);
    r.line(base);
    r.expire();
    assert_eq!(r.in_flight, 0);
    assert_eq!(r.wait, Duration::from_secs(2));
}

#[test]
fn all_that_a_capture_left_are_dropped() {
    let base = Instant::now();
    let mut r = Requests::new(2, REQUEST_TIMEOUT, base);
    assert_eq!(r.turn(base), Turn::Wait);
    assert!(r.line(base));
    assert!(r.line(base));
    assert!(!r.line(base));
    assert_eq!(r.turn(base), Turn::Ask);
}

/// They were lost, not late: nothing counts them from here, and no wait grows.
#[test]
fn an_expiry_gives_up_on_all_that_a_capture_left() {
    let base = Instant::now();
    let mut r = Requests::new(2, REQUEST_TIMEOUT, base);
    r.expire();
    assert_eq!(r.in_flight, 0);
    assert_eq!(r.to_drop, 0);
    assert_eq!(r.wait, Duration::from_secs(2));
    assert!(!r.line(base));
}

#[test]
fn the_limit_ends_at_its_time() {
    let base = Instant::now();
    let at = |ms: u64| base + Duration::from_millis(ms);
    assert!(!time_is_up(150, base, at(150)));
    assert!(time_is_up(150, base, at(151)));
    assert!(!time_is_up(0, base, at(1_000_000)));
}

#[test]
fn the_silence_is_told_once_and_a_line_starts_it_over() {
    let base = Instant::now();
    let at = |ms: u64| base + Duration::from_millis(ms);
    let after = Duration::from_secs(30);
    let mut r = Requests::new(0, REQUEST_TIMEOUT, base);
    assert!(!r.silence_to_tell(after, at(29_999)));
    r.line(at(10_000));
    assert!(!r.silence_to_tell(after, at(30_000)));
    assert!(r.silence_to_tell(after, at(40_000)));
    assert!(!r.silence_to_tell(after, at(80_000)));
    assert_eq!(SILENCE_WARNING, Duration::from_secs(30));
    assert_eq!(MAX_WAIT, Duration::from_secs(8));
}

#[test]
fn cr_alone_is_told_once() {
    let mut r = Requests::new(0, REQUEST_TIMEOUT, Instant::now());
    assert!(!r.cr_alone_to_tell(b"0303"));
    assert!(r.cr_alone_to_tell(b"03030024913100400040\r"));
    assert!(!r.cr_alone_to_tell(b"03030024913100400040\r"));
}

/// No bound on a silent `--samples`: the warning does not end it.
#[test]
fn the_warning_does_not_end_a_silent_capture() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.otherwise = None);
        let mut d = quick(&fake);
        d.silence_warning = Duration::from_millis(1);
        limit(&mut d, 1);
        let rx = start(&mut d);
        fake.wait_for_writes(3);
        assert!(rx.try_iter().all(|(_, p)| !matches!(p, Packet::End)));
        d.stop().unwrap();
        d.close().unwrap();
    });
}

/// C keeps its request state and its buffer from one capture to the next: the
/// wire stays, and the old reading is no sample of the new capture.
#[test]
fn a_reply_a_capture_left_behind_is_no_sample_of_the_next() {
    bounded(|| {
        let fake = Fake::at_4800();
        fake.script([Reply::OnRelease(S2.to_vec())]);
        let mut d = device(&fake);
        // Nothing here is to expire, however long this machine sleeps.
        d.request_timeout = Duration::from_secs(3600);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        d.stop().unwrap();
        assert_eq!(readings(&rx), [0.0f32; 0]);
        assert_eq!(fake.writes().len(), 1);

        limit(&mut d, 1);
        let rx = start(&mut d);
        // No second request while the first may still be answered.
        std::thread::sleep(Duration::from_millis(150));
        assert_eq!(fake.writes().len(), 1);
        fake.release();
        assert_eq!(readings(&rx), [V_S]);
        assert_eq!(fake.writes().len(), 2);
        d.close().unwrap();
    });
}

#[test]
fn a_stop_in_a_reply_leaves_no_part_of_it_for_the_next_capture() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.pace = 8);
        fake.script([Reply::Now(S2.to_vec())]);
        let mut d = device(&fake);
        let rx = start(&mut d);
        fake.wait_for("part of the reply", |s| s.delivered >= 5);
        d.stop().unwrap();
        assert_eq!(readings(&rx), [0.0f32; 0]);
        assert!(fake.with(|s| !s.input.is_empty()), "the stop came too late to test");

        fake.with(|s| s.pace = 1);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S]);
        assert_eq!(fake.writes().len(), 2);
        d.close().unwrap();
    });
}

/// What the "time" mutant and its kin must meet: limit on the last line, nothing
/// out, so a second capture has no wait.
#[test]
fn a_capture_after_a_clean_end_starts_at_once_with_nothing_left() {
    bounded(|| {
        let fake = Fake::at_4800();
        let mut d = device(&fake);
        limit(&mut d, 2);
        for _ in 0..2 {
            let rx = start(&mut d);
            assert_eq!(readings(&rx), [V_S, V_S]);
        }
        assert_eq!(fake.writes().len(), 4);
        d.close().unwrap();
    });
}

/// A queue of one left unread: the sample cannot be sent. A plain blocking send
/// hangs here, in the close or the restart.
#[test]
fn a_close_does_not_hang_on_a_full_queue() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        limit(&mut d, 1);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        fake.wait_for_writes(1);
        let link = Arc::clone(&d.link);
        d.close().unwrap();
        // The worker holds its clone until it has given up on the queue, which
        // takes it far longer than this: a close joins.
        assert_eq!(Arc::strong_count(&link), 2);
        drop(rx);
    });
}

#[test]
fn a_restart_after_an_end_stuck_on_a_full_queue_does_not_hang() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        limit(&mut d, 1);
        let (tx, old) = sync_channel(2);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Header, the sample, and End has no room.
        let capturing = Arc::clone(&d.capturing);
        let deadline = Instant::now() + BOUND;
        while capturing.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline, "the capture did not end");
            std::thread::sleep(Duration::from_millis(1));
        }
        let rx = start(&mut d);
        // The old worker is gone, its End given up: a start joins it.
        while old.try_recv().is_ok() {}
        assert!(matches!(old.try_recv(), Err(std::sync::mpsc::TryRecvError::Disconnected)));
        assert_eq!(readings(&rx), [V_S]);
        d.close().unwrap();
    });
}

#[test]
fn a_start_during_a_capture_is_busy() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.otherwise = None);
        let mut d = device(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        let (tx, _second) = sync_channel(1);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.stop().unwrap();
        let _ = readings(&rx);
        d.close().unwrap();
    });
}

#[test]
fn a_failed_request_ends_the_capture() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.fail_writes_after = Some(1));
        let mut d = device(&fake);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [V_S]);
        assert_eq!(fake.writes().len(), 1);
        d.close().unwrap();
    });
}

/// `serial_write_blocking` says 0: no error, and the request runs out as any.
#[test]
fn a_request_that_times_out_is_no_error() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.writes_time_out = true);
        let mut d = quick(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(3);
        d.stop().unwrap();
        assert_eq!(readings(&rx), [0.0f32; 0]);
        d.close().unwrap();
    });
}

/// A gone port reads nothing in C, and its next request is what ends the capture.
#[test]
fn failed_reads_do_not_end_a_capture_until_a_request_fails() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = quick(&fake);
        fake.with(|s| {
            s.fail_reads = true;
            s.fail_writes_after = Some(2);
        });
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [0.0f32; 0]);
        assert_eq!(fake.writes().len(), 2);
        // The second waited for the first to expire, as no line came.
        let times = fake.write_times();
        assert!(times[1] - times[0] >= Duration::from_millis(40));
        d.close().unwrap();
    });
}

#[test]
fn a_disabled_channel_sends_nothing_and_the_limit_still_counts() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        d.enable_channel(0, false).unwrap();
        limit(&mut d, 2);
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [0.0f32; 0]);
        assert_eq!(fake.writes().len(), 2);
        d.close().unwrap();
    });
}

#[test]
fn the_options_are_c_s() {
    let fake = Fake::new();
    let mut d = device(&fake);
    let keys: Vec<Key> = d.options(None).unwrap().into_iter().map(|o| o.key).collect();
    assert_eq!(keys, [Key::Continuous, Key::LimitSamples, Key::LimitMsec]);
    assert!(d.options(None).unwrap().iter().all(|o| !o.access.get && !o.access.list));
    assert!(d.get(&Key::LimitSamples, None).is_err());
    assert!(d.list(&Key::LimitSamples, None).is_err());
    assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::LimitMsec, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::Continuous, &Value::Bool(true), None).is_err());
    assert_eq!(d.info().conn, None);
    assert_eq!(d.channels()[0].name, "P1");
    assert_eq!(d.info().model, "DM950");
    assert_eq!(d.info().vendor, "Norma");
}

#[test]
fn a_closed_device_has_no_capture() {
    let fake = Fake::new();
    let mut d = device(&fake);
    d.close().unwrap();
    let (tx, _rx) = sync_channel(1);
    assert!(d.start(PacketSink::new(tx, 0)).is_err());
}
