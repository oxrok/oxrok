// Counterpart in sigrok:
//   libsigrok/src/hardware/center-3xx/api.c,
//   libsigrok/src/hardware/center-3xx/protocol.c,
//   Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Martin Ling,
//   Soeren Apel, Daniel Elstner, Gerhard Sittig, Wolfram Sang,
//   Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

// This file is generated with its header by tools/attribution.py.

use super::*;
use crate::core::Unit;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test waits for what must happen: a stop or a limit that does not
/// bite must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);
/// More requests than any test here needs. A capture that asks without a pause
/// gets an error past this, so it fails a test: with no cap, a mutant that lost
/// count of its requests grew this log and the fake's queue at 700 MB/s.
const MOST_WRITES: usize = 10_000;
/// A wait far past any test's own, for what must not be what ended a test.
const FAR: Duration = Duration::from_secs(3600);

/// A packet in Celsius, in tenths, with T1 as given.
fn pkt(t1: i16) -> Vec<u8> {
    let mut p = vec![0; PACKET_SIZE];
    p[0] = 2;
    p[1] = 0x80;
    p[7..9].copy_from_slice(&t1.to_be_bytes());
    p[44] = 3;
    p
}

fn tenths(raw: i16) -> f32 {
    f32::from(raw) / 10.0
}

/// How the bytes of a reply come out.
#[derive(Clone, Copy, Debug)]
enum Pace {
    /// All of them at the next read.
    Now,
    /// One byte per this many reads, whatever the clock says.
    PerReads(u64),
    /// At the rate of the line: a byte per this long, in the fake's own clock.
    Clock(Duration),
    /// Only every so long, all that has arrived by then.
    Burst(Duration),
}

/// What the meter does with a request.
#[derive(Clone, Debug)]
enum Reply {
    /// Those bytes, at the fake's pace.
    Now(Vec<u8>),
    /// Nothing.
    Silent,
    /// Those bytes, when the test says so.
    OnRelease(Vec<u8>),
}

struct State {
    writes: crate::serial::WriteLog,
    /// The longest wait a read was given, and the last a write was.
    longest_read: Duration,
    write_timeout: Duration,
    /// When a byte last went to the driver.
    last_byte: Option<Instant>,
    script: VecDeque<Reply>,
    /// What every request gets once the script is out, if anything.
    otherwise: Option<Vec<u8>>,
    /// Bytes on their way, with when each may be read, if the pace is by the clock.
    input: VecDeque<(Option<Instant>, u8)>,
    held: Vec<u8>,
    reads: u64,
    pace: Pace,
    line_free: Option<Instant>,
    fail_writes_after: Option<usize>,
    writes_time_out: bool,
    fail_reads: bool,
    /// Reads never run out of this.
    endless: Option<u8>,
    /// Nor of this, a packet to a read.
    endless_packets: Option<Vec<u8>>,
    /// Set while the device must be closed: a read or write then is a finding.
    closed: Arc<AtomicBool>,
    used_after_close: bool,
}

/// A thermometer whose bytes come by reads, or on a clock of its own where it says
/// so. A fake on a clock tests what the data must be, not how long it took.
#[derive(Clone)]
struct Fake(Arc<Mutex<State>>);

impl Fake {
    fn new() -> Fake {
        Fake(Arc::new(Mutex::new(State {
            writes: crate::serial::WriteLog::default(),
            longest_read: Duration::ZERO,
            write_timeout: Duration::ZERO,
            last_byte: None,
            script: VecDeque::new(),
            otherwise: Some(pkt(239)),
            input: VecDeque::new(),
            held: Vec::new(),
            reads: 0,
            pace: Pace::Now,
            line_free: None,
            fail_writes_after: None,
            writes_time_out: false,
            fail_reads: false,
            endless: None,
            endless_packets: None,
            closed: Arc::new(AtomicBool::new(false)),
            used_after_close: false,
        })))
    }

    fn with<T>(&self, f: impl FnOnce(&mut State) -> T) -> T {
        f(&mut self.0.lock().unwrap())
    }

    fn script(&self, replies: impl IntoIterator<Item = Reply>) {
        self.with(|s| s.script.extend(replies));
    }

    fn silent(&self) {
        self.with(|s| s.otherwise = None);
    }

    fn release(&self) {
        self.with(|s| {
            let held = std::mem::take(&mut s.held);
            s.queue(&held);
        });
    }

    fn feed(&self, bytes: &[u8]) {
        self.with(|s| s.queue(bytes));
    }

    fn writes(&self) -> usize {
        self.with(|s| s.writes.len())
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

impl State {
    fn queue(&mut self, bytes: &[u8]) {
        let now = Instant::now();
        for &b in bytes {
            let at = match self.pace {
                Pace::Now | Pace::PerReads(_) => None,
                Pace::Clock(per) | Pace::Burst(per) => {
                    let free = self.line_free.unwrap_or(now).max(now);
                    // A burst comes when its period is up; the line itself is 9600.
                    let per_byte =
                        if matches!(self.pace, Pace::Clock(_)) { per } else { LINE_BYTE };
                    self.line_free = Some(free + per_byte);
                    Some(free + per_byte)
                }
            };
            self.input.push_back((at, b));
        }
    }
}

/// A byte at 9600 8n1.
const LINE_BYTE: Duration = Duration::from_micros(1042);

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
            if s.closed.load(Ordering::SeqCst) {
                s.used_after_close = true;
            }
            s.longest_read = s.longest_read.max(timeout);
            if s.fail_reads {
                return Err(Error::device("hung up"));
            }
            if let Some(byte) = s.endless {
                buf.fill(byte);
                return Ok(buf.len());
            }
            s.reads += 1;
            if let Some(bytes) = s.endless_packets.clone() {
                // A mutant that never ends gets an error: what it sent is kept by the test.
                if s.reads > 20_000 {
                    return Err(Error::device("too many reads"));
                }
                drop(s);
                std::thread::sleep(Duration::from_micros(100));
                let n = bytes.len().min(buf.len());
                buf[..n].copy_from_slice(&bytes[..n]);
                return Ok(n);
            }
            let now = Instant::now();
            let room = match s.pace {
                Pace::PerReads(n) => usize::from(s.reads.is_multiple_of(n)),
                Pace::Burst(every) => {
                    // Only on the period's first read: none in between.
                    let phase = now.duration_since(BASE.get_or_init(Instant::now).to_owned());
                    if phase.as_millis() % every.as_millis().max(1) < 5 { buf.len() } else { 0 }
                }
                _ => buf.len(),
            };
            let mut n = 0;
            while n < room.min(buf.len()) {
                match s.input.front() {
                    Some((at, b)) if at.is_none_or(|at| at <= now) => {
                        buf[n] = *b;
                        n += 1;
                        s.input.pop_front();
                    }
                    _ => break,
                }
            }
            if n > 0 {
                s.last_byte = Some(now);
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
    fn write_timeout(&mut self, data: &[u8], timeout: Duration) -> Result<usize> {
        let mut s = self.0.lock().unwrap();
        s.write_timeout = timeout;
        if s.closed.load(Ordering::SeqCst) {
            s.used_after_close = true;
        }
        if s.fail_writes_after.is_some_and(|n| s.writes.len() >= n) {
            return Err(Error::device("Broken pipe"));
        }
        if s.writes.len() >= MOST_WRITES {
            return Err(Error::device("too many requests"));
        }
        s.writes.push((Instant::now(), data.to_vec()));
        if s.writes_time_out {
            return Ok(0);
        }
        match s.script.pop_front() {
            Some(Reply::Now(bytes)) => s.queue(&bytes),
            Some(Reply::Silent) => {}
            Some(Reply::OnRelease(bytes)) => s.held = bytes,
            None => {
                let otherwise = s.otherwise.clone().unwrap_or_default();
                s.queue(&otherwise);
            }
        }
        Ok(data.len())
    }
    fn purge(&mut self) -> Result<()> {
        panic!("C flushes at an open only, and this driver never does");
    }
}

static BASE: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();

type Received = Receiver<(usize, Packet)>;

/// A device with a quick pass, and the bounds far off.
fn quick(fake: &Fake) -> CenterDevice {
    let mut d = CenterDevice::from_port(Box::new(fake.clone()));
    d.pass = Duration::from_millis(2);
    d.request_wait = FAR;
    d.silence_bound = FAR;
    d.outage_bound = FAR;
    d.settle = Duration::from_millis(1);
    d.open().unwrap();
    d
}

fn start(d: &mut CenterDevice) -> Received {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

fn limit(d: &mut CenterDevice, samples: u64) {
    d.set(&Key::LimitSamples, &Value::U64(samples), None).unwrap();
}

/// What a capture sends up to its End, which must come within [`BOUND`].
fn packets(rx: &Received) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left).expect("no End in time").1 {
            Packet::End => return out,
            p => out.push(p),
        }
    }
}

/// T1 of each sample a capture sends.
fn firsts(packets: &[Packet]) -> Vec<f32> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) if a.channels == [0] => Some(a.samples[0]),
            _ => None,
        })
        .collect()
}

fn failure(packets: &[Packet]) -> Option<&str> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(why) => Some(why.as_str()),
        _ => None,
    })
}

/// Waits by the clock, a little at a time: a sleep of the whole time runs long under
/// a slowed clock, past what a test needs to come first.
fn wait_for_time(time: Duration) {
    let began = Instant::now();
    while began.elapsed() < time {
        std::thread::sleep(Duration::from_millis(1));
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

#[test]
fn the_literals_are_these() {
    assert_eq!(PASS, Duration::from_millis(500));
    assert_eq!(REQUEST_WAIT, Duration::from_secs(2));
    assert_eq!(MAX_HOLD, Duration::from_secs(30));
    assert_eq!(MOST_OUT, 2);
    assert_eq!(SILENCE_BOUND, Duration::from_secs(10));
    assert_eq!(OUTAGE_BOUND, Duration::from_secs(600));
    assert_eq!(SETTLE, Duration::from_millis(25));
    assert_eq!(STOP_CHECK, Duration::from_millis(5));
    assert_eq!(END_AFTER_STOP, Duration::from_millis(500));
    let d = CenterDevice::from_port(Box::new(Fake::new()));
    assert_eq!(d.pass, Duration::from_millis(500));
    assert_eq!(d.request_wait, Duration::from_secs(2));
    assert_eq!(d.silence_bound, Duration::from_secs(10));
    assert_eq!(d.outage_bound, Duration::from_secs(600));
    assert_eq!(d.settle, Duration::from_millis(25));
    assert_eq!(d.end_after_stop, Duration::from_millis(500));
}

/// Each wait of a capture is short, so a stop is seen; a write has C's timeout.
#[test]
fn a_read_waits_5_ms_at_most_and_a_write_has_c_s_timeout() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = CenterDevice::from_port(Box::new(fake.clone()));
        d.pass = Duration::from_millis(40);
        d.open().unwrap();
        limit(&mut d, 2);
        let rx = start(&mut d);
        packets(&rx);
        assert!(fake.with(|s| s.longest_read) <= Duration::from_millis(5));
        let params = Params::parse(SERIALCOMM).unwrap();
        assert_eq!(fake.with(|s| s.write_timeout), serial_timeout(&params, 1));
    });
}

#[test]
fn the_drivers_are_named_as_c_s() {
    assert_eq!(Center309Driver.id(), "center-309");
    assert_eq!(Center309Driver.long_name(), "Center 309");
    assert_eq!(VoltcraftK204Driver.id(), "voltcraft-k204");
    assert_eq!(VoltcraftK204Driver.long_name(), "Voltcraft K204");
    assert_eq!(Brand::Center309.vendor(), "Center");
    assert_eq!(Brand::Center309.model(), "309");
    assert_eq!(Brand::VoltcraftK204.vendor(), "Voltcraft");
    assert_eq!(Brand::VoltcraftK204.model(), "K204");
    let d = CenterDevice::from_port(Box::new(Fake::new()));
    assert_eq!(d.classes(), [DeviceClass::Thermometer]);
    let names: Vec<_> = d.channels().iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["T1", "T2", "T3", "T4"]);
    assert!(d.channels().iter().all(|c| c.kind == ChannelKind::Analog && c.enabled));
}

#[test]
fn a_scan_with_no_conn_or_no_port_finds_nothing() {
    assert_eq!(Center309Driver.scan(&[]).unwrap().len(), 0);
    let conn = (Key::Conn, Value::from("/dev/oxrok-center-3xx-none"));
    assert_eq!(Center309Driver.scan(&[conn]).unwrap().len(), 0);
}

#[test]
fn the_device_has_no_get_and_takes_only_positive_limits() {
    let mut d = CenterDevice::from_port(Box::new(Fake::new()));
    assert!(d.get(&Key::LimitSamples, None).is_err());
    assert!(d.list(&Key::LimitSamples, None).is_err());
    assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::LimitMsec, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::Continuous, &Value::Bool(true), None).is_err());
    assert!(d.set(&Key::LimitMsec, &Value::U64(1), None).is_ok());
}

#[test]
fn a_capture_asks_once_per_reading_with_what_the_packet_says() {
    bounded(|| {
        let fake = Fake::new();
        let mut packet = pkt(239);
        // T2 whole and over limit, T3 -5.0, T4 256 whole.
        packet[9..11].copy_from_slice(&7i16.to_be_bytes());
        packet[11..13].copy_from_slice(&(-50i16).to_be_bytes());
        packet[13..15].copy_from_slice(&256i16.to_be_bytes());
        packet[39] = 0b0010;
        packet[43] = 0b1010;
        fake.with(|s| s.otherwise = Some(packet));
        let mut d = quick(&fake);
        limit(&mut d, 2);
        let rx = start(&mut d);
        let got = packets(&rx);
        assert!(matches!(got[0], Packet::Header { .. }));
        let analog: Vec<&AnalogData> = got
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some(a),
                _ => None,
            })
            .collect();
        assert_eq!(analog.len(), 8);
        let want = [
            (0, 23.9, 1),
            (1, f32::INFINITY, 0),
            (2, -5.0, 1),
            (3, 256.0, 0),
            (0, 23.9, 1),
            (1, f32::INFINITY, 0),
            (2, -5.0, 1),
            (3, 256.0, 0),
        ];
        for (a, (ch, value, digits)) in analog.iter().zip(want) {
            assert_eq!(a.channels, [ch]);
            assert_eq!(a.samples, [value]);
            assert_eq!(a.digits, digits);
            assert_eq!(a.mq, Mq::Temperature);
            assert_eq!(a.unit, Unit::Celsius);
            assert_eq!(a.flags, MqFlags::empty());
        }
        assert_eq!(fake.writes(), 2);
        assert_eq!(fake.with(|s| s.writes[0].1.clone()), b"A");
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_channel_off_sends_nothing_and_a_sample_still_counts() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = quick(&fake);
        d.enable_channel(0, false).unwrap();
        d.enable_channel(2, false).unwrap();
        limit(&mut d, 2);
        let rx = start(&mut d);
        let got = packets(&rx);
        let channels: Vec<usize> = got
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some(a.channels[0]),
                _ => None,
            })
            .collect();
        assert_eq!(channels, [1, 3, 1, 3]);
    });
}

#[test]
fn a_read_holding_two_packets_sends_both_past_the_limit() {
    bounded(|| {
        let fake = Fake::new();
        let two = [pkt(1), pkt(2)].concat();
        fake.script([Reply::Now(two)]);
        let mut d = quick(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(1), tenths(2)]);
        assert_eq!(fake.writes(), 1);
    });
}

#[test]
fn the_first_request_waits_its_pass_and_the_next_a_pass_after_the_bytes() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = CenterDevice::from_port(Box::new(fake.clone()));
        d.pass = Duration::from_millis(100);
        d.open().unwrap();
        limit(&mut d, 3);
        let began = Instant::now();
        let rx = start(&mut d);
        packets(&rx);
        let times = fake.write_times();
        assert_eq!(times.len(), 3);
        assert!(times[0] - began >= Duration::from_millis(100));
        // The least gap: a loaded box only stretches one.
        assert!(times[1] - times[0] >= Duration::from_millis(100));
        assert!(times[2] - times[1] >= Duration::from_millis(100));
    });
}

/// C's `fd_source_check` looks at `revents` first, and only with a timeout at the
/// clock. So nothing asks while bytes are ready, even long past the pass.
#[test]
fn a_ready_answer_wins_over_the_timer() {
    bounded(|| {
        let fake = Fake::new();
        // More than a read takes.
        fake.feed(&vec![0xee; 300]);
        fake.feed(&pkt(5));
        let mut d = quick(&fake);
        d.pass = Duration::ZERO;
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(5)]);
        // With a pass of 0, a timer ahead of the read would ask at once.
        assert_eq!(fake.writes(), 0);
    });
}

#[test]
fn a_time_limit_ends_a_silent_capture_and_only_the_time_does() {
    bounded(|| {
        let fake = Fake::new();
        fake.silent();
        let mut d = quick(&fake);
        d.set(&Key::LimitMsec, &Value::U64(300), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert!(began.elapsed() >= Duration::from_millis(300));
        assert_eq!(failure(&got), None);
        assert_eq!(fake.writes(), 1, "no second request ahead of its wait");
    });
}

/// The first pass goes before the limit is looked at, as C's callback does.
#[test]
fn a_time_limit_under_the_pass_still_asks_once() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = quick(&fake);
        d.pass = Duration::from_millis(50);
        d.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(fake.writes(), 1);
        assert_eq!(firsts(&got), [] as [f32; 0]);
    });
}

#[test]
fn a_stop_ends_a_capture_that_has_data() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = quick(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(3);
        d.stop().unwrap();
        assert_eq!(failure(&packets(&rx)), None);
        let writes = fake.writes();
        d.close().unwrap();
        assert_eq!(fake.writes(), writes);
    });
}

#[test]
fn a_stop_ends_a_capture_under_a_flood() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.endless = Some(0xee));
        let mut d = quick(&fake);
        let rx = start(&mut d);
        std::thread::sleep(Duration::from_millis(20));
        d.stop().unwrap();
        let began = Instant::now();
        packets(&rx);
        assert!(began.elapsed() < Duration::from_secs(10), "{:?}", began.elapsed());
        assert_eq!(fake.writes(), 0);
    });
}

#[test]
fn a_silent_capture_with_only_samples_fails_saying_nothing_came() {
    bounded(|| {
        let fake = Fake::new();
        fake.silent();
        let mut d = quick(&fake);
        d.silence_bound = Duration::from_secs(1);
        limit(&mut d, 1);
        let began = Instant::now();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(failure(&got), Some("device error: no answer from the thermometer for 1 s"));
        assert!(began.elapsed() >= Duration::from_secs(1));
        assert_eq!(firsts(&got), [] as [f32; 0]);
    });
}

#[test]
fn bytes_that_make_no_packet_fail_with_their_own_words() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.otherwise = Some(vec![0xee; PACKET_SIZE]));
        let mut d = quick(&fake);
        d.silence_bound = Duration::from_secs(1);
        d.request_wait = Duration::from_millis(50);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(
            failure(&packets(&rx)),
            Some("device error: no usable answer from the thermometer for 1 s")
        );
    });
}

/// A stream of junk leaves no quiet to ask in, and no packet: the bound is on what
/// counts, not on the bytes.
#[test]
fn a_flood_does_not_keep_a_samples_capture_going() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.endless = Some(0xee));
        let mut d = quick(&fake);
        d.silence_bound = Duration::from_secs(1);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(
            failure(&packets(&rx)),
            Some("device error: no usable answer from the thermometer for 1 s")
        );
    });
}

/// No bound where a time limit, or only a stop, ends it.
#[test]
fn the_bound_is_for_a_samples_capture_alone() {
    bounded(|| {
        let fake = Fake::new();
        fake.silent();
        let mut d = quick(&fake);
        d.silence_bound = Duration::from_millis(20);
        limit(&mut d, 1);
        d.set(&Key::LimitMsec, &Value::U64(200), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(failure(&packets(&rx)), None);

        let fake = Fake::new();
        fake.silent();
        let mut d = quick(&fake);
        d.silence_bound = Duration::from_millis(20);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        std::thread::sleep(Duration::from_millis(100));
        assert!(rx.try_iter().all(|(_, p)| !matches!(p, Packet::Failed(_) | Packet::End)));
        d.stop().unwrap();
        assert_eq!(failure(&packets(&rx)), None);
    });
}

/// A send a client keeps waiting is no silence of the meter's: the stamp is after
/// the send, so the capture is not failed for it however long it was kept.
#[test]
fn a_slow_client_does_not_end_a_capture_with_a_false_error() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = quick(&fake);
        d.silence_bound = Duration::from_millis(300);
        d.outage_bound = Duration::from_millis(300);
        limit(&mut d, 3);
        // Room for little: each send waits for this thread.
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        let mut got = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let packet = rx.recv_timeout(left).expect("no End in time").1;
            if matches!(packet, Packet::End) {
                break;
            }
            if matches!(packet, Packet::Analog(_)) && got.len() % 4 == 3 {
                std::thread::sleep(Duration::from_millis(600));
            }
            got.push(packet);
        }
        assert_eq!(failure(&got), None);
        assert_eq!(firsts(&got).len(), 3);
    });
}

#[test]
fn a_lost_reply_is_asked_again_after_its_wait() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Silent, Reply::Now(pkt(5))]);
        let mut d = quick(&fake);
        d.request_wait = Duration::from_millis(40);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(5)]);
        let t = fake.write_times();
        assert_eq!(t.len(), 2);
        assert!(t[1] - t[0] >= Duration::from_millis(40));
    });
}

/// C keeps the part, and the next reply lands behind it: T1's byte, a 2, makes a
/// false packet where the next one's byte 7 is a 3.
#[test]
fn part_of_a_reply_is_dropped_ahead_of_the_next_request() {
    bounded(|| {
        let fake = Fake::new();
        let mut first = pkt(0x0002);
        first.truncate(9);
        // Byte 8 is 2, and 36 further on from the next packet's start is its end.
        let mut next = pkt(77);
        next[36] = 3;
        fake.script([Reply::Now(first), Reply::Now(next)]);
        let mut d = quick(&fake);
        d.request_wait = Duration::from_millis(40);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(77)]);
    });
}

/// A part left, then a reply cut short: put together they would make a window with
/// no later one to lose to. Only the drop ahead of a request stops that.
#[test]
fn what_is_left_and_a_cut_reply_make_no_false_packet() {
    bounded(|| {
        let fake = Fake::new();
        let mut part = vec![2];
        part.resize(9, 0x11);
        let mut cut = pkt(5);
        cut.truncate(40);
        // 44 on from the part's start.
        cut[35] = 3;
        fake.script([Reply::Now(part), Reply::Now(cut), Reply::Now(pkt(77))]);
        let mut d = quick(&fake);
        d.request_wait = Duration::from_millis(20);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(77)]);
    });
}

/// A meter that never goes quiet must still come to its limit.
#[test]
fn a_stream_of_packets_with_no_quiet_ends_at_its_limit() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.endless_packets = Some(pkt(5)));
        let mut d = quick(&fake);
        limit(&mut d, 3);
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(failure(&got), None);
        assert!(firsts(&got).len() >= 3);
    });
}

/// In C a pass with bytes and no packet clears `request_new_packet`, so this ends
/// its polling for good.
#[test]
fn a_stray_byte_between_replies_stops_nothing() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Now(pkt(1)), Reply::Now(pkt(2))]);
        let mut d = quick(&fake);
        d.pass = Duration::from_millis(30);
        limit(&mut d, 2);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        fake.wait_for("the first reply read", |s| s.input.is_empty());
        fake.feed(&[0xee]);
        assert_eq!(firsts(&packets(&rx)), [tenths(1), tenths(2)]);
        assert_eq!(fake.writes(), 2);
    });
}

/// A stray 2 ahead of a packet whose byte 43 is 3 makes one starting a byte early,
/// whose T1 is the wrong bytes. C reads that; a request clears what is left.
#[test]
fn bytes_left_ahead_of_a_request_make_no_false_packet() {
    bounded(|| {
        let fake = Fake::new();
        let mut one = pkt(1);
        one.push(2);
        let mut two = pkt(0x0123);
        two[43] = 3;
        fake.script([Reply::Now(one), Reply::Now(two)]);
        let mut d = quick(&fake);
        limit(&mut d, 2);
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(firsts(&got), [tenths(1), 0x0123 as f32]);
    });
}

/// A reply older than the wait says the meter is slow: the next request waits half as
/// long again as that reply took. The least gap: a loaded box only stretches it. The
/// test starts again if a stall let the pass run out ahead of the reply.
#[test]
fn a_slow_reply_holds_the_next_request_back() {
    bounded(|| {
        for _ in 0..20 {
            let fake = Fake::new();
            fake.script([Reply::OnRelease(pkt(1))]);
            let mut d = quick(&fake);
            d.request_wait = Duration::from_millis(100);
            // Past the pass, so the reply can get older than the wait.
            d.pass = Duration::from_millis(400);
            limit(&mut d, 2);
            let rx = start(&mut d);
            fake.wait_for_writes(1);
            let late = fake.with(|s| std::mem::take(&mut s.held));
            // Older than the wait of the request out, and ahead of the pass.
            wait_for_time(Duration::from_millis(150));
            let early = fake.writes() > 1;
            let fed = Instant::now();
            fake.feed(&late);
            let got = firsts(&packets(&rx));
            if early {
                continue;
            }
            assert_eq!(got, [tenths(1), tenths(239)]);
            let times = fake.write_times();
            assert_eq!(times.len(), 2);
            assert!(times[1] - fed >= Duration::from_millis(225), "{:?}", times[1] - fed);
            return;
        }
        panic!("the pass came ahead of the reply each time");
    });
}

/// Two requests out and both replies in one read: the second is of oxrok's making,
/// so the limit holds. With no expiry, two packets in a read both go, as in C.
#[test]
fn a_reply_to_an_extra_request_does_not_pass_the_limit() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::OnRelease(pkt(1)), Reply::OnRelease(pkt(2))]);
        let mut d = quick(&fake);
        d.request_wait = Duration::from_millis(20);
        limit(&mut d, 1);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        let first = fake.with(|s| std::mem::take(&mut s.held));
        fake.wait_for_writes(2);
        fake.with(|s| {
            let second = std::mem::take(&mut s.held);
            s.queue(&[first, second].concat());
        });
        assert_eq!(firsts(&packets(&rx)), [tenths(1)]);
    });
}

/// C holds nothing between two captures but its buffer and a static flag.
#[test]
fn a_reply_left_on_its_way_is_no_sample_of_the_next_capture() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::OnRelease(pkt(1))]);
        let mut d = quick(&fake);
        d.request_wait = Duration::from_secs(5);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        d.stop().unwrap();
        packets(&rx);
        // It comes in the next capture.
        limit(&mut d, 1);
        let rx = start(&mut d);
        fake.release();
        assert_eq!(firsts(&packets(&rx)), [tenths(239)]);
        assert_eq!(fake.writes(), 2);
    });
}

#[test]
fn part_of_a_reply_left_on_its_way_is_finished_and_dropped() {
    bounded(|| {
        let fake = Fake::new();
        let whole = pkt(1);
        fake.script([Reply::Now(whole[..20].to_vec())]);
        let mut d = quick(&fake);
        d.request_wait = Duration::from_secs(5);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        fake.wait_for("the part read", |s| s.input.is_empty());
        std::thread::sleep(Duration::from_millis(10));
        d.stop().unwrap();
        packets(&rx);
        limit(&mut d, 1);
        fake.feed(&whole[20..]);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(239)]);
        assert_eq!(fake.writes(), 2);
    });
}

/// C's static flag stays down here: its next capture never asks.
#[test]
fn a_reply_left_behind_that_never_comes_costs_one_wait() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Silent]);
        let mut d = quick(&fake);
        d.request_wait = Duration::from_secs(5);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        d.stop().unwrap();
        packets(&rx);
        d.request_wait = Duration::from_millis(100);
        limit(&mut d, 1);
        let began = Instant::now();
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(239)]);
        assert!(began.elapsed() >= Duration::from_millis(100));
        assert_eq!(fake.writes(), 2);
    });
}

#[test]
fn a_capture_after_a_clean_end_asks_at_its_first_pass() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = quick(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        packets(&rx);
        // Far off: only a request that does not wait for it can end this.
        d.request_wait = FAR;
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)).len(), 1);
        assert_eq!(fake.writes(), 2);
    });
}

#[test]
fn a_reopen_forgets_what_was_out_and_what_had_come() {
    bounded(|| {
        let fake = Fake::new();
        let whole = pkt(1);
        fake.script([Reply::Now(whole[..20].to_vec())]);
        let mut d = quick(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        fake.wait_for("the part read", |s| s.input.is_empty());
        std::thread::sleep(Duration::from_millis(10));
        d.stop().unwrap();
        packets(&rx);
        d.close().unwrap();
        d.pending = Some(Box::new(fake.clone()));
        d.open().unwrap();
        limit(&mut d, 1);
        // A tail after the open's flush finishes nothing: the part is gone.
        fake.feed(&whole[20..]);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(239)]);
        assert_eq!(fake.writes(), 2);
    });
}

#[test]
fn a_failed_write_fails_the_capture() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.fail_writes_after = Some(1));
        let mut d = quick(&fake);
        limit(&mut d, 5);
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(failure(&got), Some("device error: unable to send a request"));
        assert_eq!(firsts(&got), [tenths(239)]);
    });
}

#[test]
fn a_failed_read_fails_the_capture() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.fail_reads = true);
        let mut d = quick(&fake);
        limit(&mut d, 5);
        let rx = start(&mut d);
        assert_eq!(failure(&packets(&rx)), Some("device error: the port gone"));
        assert_eq!(fake.writes(), 0);
    });
}

/// What a read held goes out ahead of the read that fails.
#[test]
fn a_failed_read_after_a_packet_keeps_the_packet() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Now(pkt(1))]);
        fake.silent();
        let mut d = quick(&fake);
        let rx = start(&mut d);
        let deadline = Instant::now() + BOUND;
        let mut got = Vec::new();
        while firsts(&got).is_empty() {
            let left = deadline.saturating_duration_since(Instant::now());
            got.push(rx.recv_timeout(left).expect("no packet in time").1);
        }
        fake.with(|s| s.fail_reads = true);
        assert_eq!(failure(&packets(&rx)), Some("device error: the port gone"));
    });
}

#[test]
fn the_log_lines_of_a_failed_port_are_these() {
    let e = Error::device("hung up");
    assert_eq!(send_error_line(&e), "Error sending 'A' command: device error: hung up.");
    assert_eq!(read_error_line(&e), "Serial port read error: device error: hung up.");
}

#[test]
fn a_capture_that_asks_without_a_pause_is_stopped_by_the_fake() {
    bounded(|| {
        let fake = Fake::new();
        fake.silent();
        fake.with(|s| {
            for _ in 0..MOST_WRITES {
                s.writes.push((Instant::now(), Vec::new()));
            }
        });
        let mut d = quick(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(failure(&packets(&rx)), Some("device error: unable to send a request"));
    });
}

#[test]
fn a_write_that_times_out_is_no_error() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.writes_time_out = true);
        let mut d = quick(&fake);
        d.set(&Key::LimitMsec, &Value::U64(50), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(failure(&packets(&rx)), None);
        assert_eq!(fake.writes(), 1);
    });
}

#[test]
fn at_the_lines_rate_the_data_is_whole() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.pace = Pace::Clock(LINE_BYTE));
        fake.script([Reply::Now(pkt(1)), Reply::Now(pkt(2)), Reply::Now(pkt(3))]);
        let mut d = quick(&fake);
        // A pass shorter than a reply's 47 ms, the worst case for the wait.
        d.pass = Duration::from_millis(100);
        d.request_wait = Duration::from_secs(2);
        limit(&mut d, 3);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(1), tenths(2), tenths(3)]);
    });
}

/// A byte per read, whatever the clock does: a pass ahead of the last byte asks for
/// nothing, as the request is still out.
#[test]
fn a_byte_at_a_time_makes_whole_packets_and_one_request_each() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.pace = Pace::PerReads(3));
        fake.script([Reply::Now(pkt(1)), Reply::Now(pkt(2))]);
        let mut d = quick(&fake);
        d.pass = Duration::ZERO;
        limit(&mut d, 2);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(1), tenths(2)]);
        assert_eq!(fake.writes(), 2);
    });
}

#[test]
fn bytes_in_bursts_every_50_ms_lose_nothing() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.pace = Pace::Burst(Duration::from_millis(50)));
        fake.script([Reply::Now(pkt(1)), Reply::Now(pkt(2)), Reply::Now(pkt(3))]);
        let mut d = quick(&fake);
        d.pass = Duration::from_millis(200);
        d.request_wait = Duration::from_secs(2);
        limit(&mut d, 3);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(1), tenths(2), tenths(3)]);
        assert_eq!(fake.writes(), 3);
    });
}

/// A reply slower than a pass and a stop in it, on the clock: what the next capture
/// reads must still be whole packets of its own.
#[test]
fn a_stop_during_a_reply_at_the_lines_rate_leaves_the_next_capture_clean() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.pace = Pace::Clock(LINE_BYTE));
        fake.script([Reply::Now(pkt(1))]);
        fake.with(|s| s.otherwise = Some(pkt(2)));
        let mut d = quick(&fake);
        d.request_wait = Duration::from_secs(1);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        d.stop().unwrap();
        packets(&rx);
        limit(&mut d, 2);
        let rx = start(&mut d);
        let got = firsts(&packets(&rx));
        assert_eq!(got, [tenths(2), tenths(2)]);
    });
}

/// The overlap rule in a capture, with the reply's last byte a read behind.
#[test]
fn a_stray_start_byte_does_not_shift_the_reading() {
    bounded(|| {
        let fake = Fake::new();
        let mut reply = pkt(2910);
        reply[9..11].copy_from_slice(&2800i16.to_be_bytes());
        reply[43] = 3;
        let mut wire = vec![2];
        wire.extend_from_slice(&reply);
        fake.with(|s| s.pace = Pace::PerReads(2));
        fake.script([Reply::Now(wire)]);
        let mut d = quick(&fake);
        d.settle = Duration::from_millis(20);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [2910.0]);
    });
}

/// A library client is left with no tty.
#[test]
fn a_close_gives_the_port_back() {
    let fake = Fake::new();
    let mut d = quick(&fake);
    assert!(d.link.lock().unwrap().port.is_some());
    d.close().unwrap();
    assert!(d.link.lock().unwrap().port.is_none());
}

/// A byte puts the next request off by a pass, as the source's timer in C. The test
/// starts again if a stall let the first request go ahead of the byte.
#[test]
fn bytes_put_the_next_request_off() {
    bounded(|| {
        let pass = Duration::from_millis(300);
        for _ in 0..20 {
            let fake = Fake::new();
            fake.silent();
            let mut d = quick(&fake);
            d.pass = pass;
            let rx = start(&mut d);
            wait_for_time(Duration::from_millis(100));
            fake.feed(&[0xee]);
            fake.wait_for("the byte read", |s| s.input.is_empty());
            let last_byte = fake.with(|s| s.last_byte).unwrap();
            let early = fake.writes() > 0;
            fake.wait_for_writes(1);
            let asked = fake.write_times()[0];
            d.stop().unwrap();
            packets(&rx);
            if early {
                continue;
            }
            assert!(asked - last_byte >= pass, "{:?}", asked - last_byte);
            return;
        }
        panic!("the first request came ahead of the byte each time");
    });
}

/// A reply dropped as the capture before's is no sample: the bound does not move for
/// it, and with none yet the capture fails rather than going on.
#[test]
fn a_dropped_reply_is_no_sample_to_the_bound() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::OnRelease(pkt(1))]);
        fake.with(|s| s.otherwise = None);
        let mut d = quick(&fake);
        d.request_wait = Duration::from_secs(5);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        d.stop().unwrap();
        packets(&rx);
        d.silence_bound = Duration::from_millis(200);
        limit(&mut d, 1);
        fake.release();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(firsts(&got), [] as [f32; 0]);
        assert_eq!(
            failure(&got),
            Some("device error: no usable answer from the thermometer for 0 s")
        );
    });
}

/// One that was never sent is not waited for by the next capture.
#[test]
fn a_failed_write_leaves_nothing_out() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.fail_writes_after = Some(0));
        let mut d = quick(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(failure(&packets(&rx)), Some("device error: unable to send a request"));
        fake.with(|s| s.fail_writes_after = None);
        // With a wait that never ends, only a request with nothing out can go.
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)), [tenths(239)]);
    });
}

/// A queue left full at the start must not hold a stop up in the header's send.
#[test]
fn a_stop_ends_a_capture_stuck_on_its_header() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = quick(&fake);
        let (tx, rx) = sync_channel(1);
        let sink = PacketSink::new(tx, 0);
        assert!(sink.send(Packet::FrameBegin));
        d.start(sink).unwrap();
        std::thread::sleep(Duration::from_millis(10));
        d.stop().unwrap();
        d.close().unwrap();
        drop(rx);
        assert_eq!(fake.writes(), 0);
    });
}

/// After a stop an End still gets its time on a full queue. A far time, so no stall
/// can pass for the giving up: what fails here is a thread that gives up at once.
#[test]
fn an_end_waits_its_time_after_a_stop() {
    bounded(|| {
        let fake = Fake::new();
        fake.silent();
        let mut d = quick(&fake);
        d.end_after_stop = BOUND;
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        fake.wait_for_writes(1);
        d.stop().unwrap();
        // The Header holds the queue, so End waits. Give it time to see the stop.
        std::thread::sleep(Duration::from_millis(50));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::End));
    });
}

/// An outage after a sample warns, and the meter's return ends it: no failure.
#[test]
fn an_outage_after_a_sample_goes_on() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Now(pkt(1)), Reply::OnRelease(pkt(2))]);
        fake.silent();
        let mut d = quick(&fake);
        d.silence_bound = Duration::from_millis(50);
        limit(&mut d, 2);
        let rx = start(&mut d);
        fake.wait_for_writes(2);
        // Well past the bound, by what the capture has seen.
        std::thread::sleep(Duration::from_millis(200));
        fake.release();
        let got = packets(&rx);
        assert_eq!(failure(&got), None);
        assert_eq!(firsts(&got), [tenths(1), tenths(2)]);
    });
}

#[test]
fn an_outage_past_its_bound_fails_after_a_sample() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::Now(pkt(1))]);
        fake.silent();
        let mut d = quick(&fake);
        d.request_wait = Duration::from_millis(20);
        d.silence_bound = Duration::from_millis(20);
        d.outage_bound = Duration::from_secs(1);
        limit(&mut d, 2);
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(firsts(&got), [tenths(1)]);
        assert_eq!(failure(&got), Some("device error: no answer from the thermometer for 1 s"));
    });
}

#[test]
fn a_closed_device_has_no_capture() {
    let mut d = CenterDevice::from_port(Box::new(Fake::new()));
    let (tx, _rx) = sync_channel(4);
    assert!(d.start(PacketSink::new(tx, 0)).is_err());
}

#[test]
fn nothing_touches_the_port_after_a_close() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = quick(&fake);
        let _rx = start(&mut d);
        fake.wait_for_writes(2);
        d.close().unwrap();
        let closed = fake.with(|s| Arc::clone(&s.closed));
        closed.store(true, Ordering::SeqCst);
        let writes = fake.writes();
        std::thread::sleep(Duration::from_millis(30));
        assert_eq!(fake.writes(), writes);
        assert!(!fake.with(|s| s.used_after_close));
    });
}

#[test]
fn a_close_does_not_hang_on_a_full_queue() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = quick(&fake);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        fake.wait_for_writes(1);
        std::thread::sleep(Duration::from_millis(20));
        let began = Instant::now();
        d.close().unwrap();
        assert!(began.elapsed() < Duration::from_secs(10), "{:?}", began.elapsed());
        drop(rx);
    });
}

/// A capture's End stuck on a queue nobody reads must not hold up the next, and the
/// old sink goes with the old thread.
#[test]
fn a_restart_after_an_end_stuck_on_a_full_queue_does_not_hang() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = quick(&fake);
        limit(&mut d, 1);
        let (tx, old) = sync_channel(5);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Header and four channels fill it; End waits.
        fake.wait_for("the reply read", |s| s.writes.len() == 1 && s.input.is_empty());
        let deadline = Instant::now() + BOUND;
        while d.capturing.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline, "the capture did not end");
            std::thread::sleep(Duration::from_millis(1));
        }
        let rx = start(&mut d);
        assert_eq!(firsts(&packets(&rx)).len(), 1);
        // The old worker is gone, and its sink with it.
        let mut left = Vec::new();
        while let Ok((_, packet)) = old.try_recv() {
            left.push(packet);
        }
        assert!(matches!(old.try_recv(), Err(std::sync::mpsc::TryRecvError::Disconnected)));
        assert!(!left.iter().any(|p| matches!(p, Packet::End)));
    });
}

#[test]
fn a_start_during_a_capture_is_refused() {
    bounded(|| {
        let fake = Fake::new();
        fake.silent();
        let mut d = quick(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        let (tx, _second) = sync_channel(4);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.stop().unwrap();
        packets(&rx);
    });
}

fn at(base: Instant, ms: u64) -> Instant {
    base + Duration::from_millis(ms)
}

#[test]
fn an_unanswered_thermometer_is_asked_every_2_s() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    assert_eq!(r.turn(base), Turn::Ask);
    r.asked(base);
    let mut t = 0;
    for _ in 0..20 {
        assert_eq!(r.turn(at(base, t + 1999)), Turn::Wait);
        t += 2000;
        assert_eq!(r.turn(at(base, t)), Turn::Expired);
        r.expire();
        r.asked(at(base, t));
    }
    // A meter that is off has lost the rest.
    assert_eq!(r.out, 2);
    assert!(r.expired);
}

/// With two out, a reply counts from the older, so it holds; the hold running out with
/// nothing more says one was lost; and a reply with one out, in good time, ends it.
#[test]
fn a_reply_with_two_out_holds_and_the_hold_running_out_takes_one_off() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    r.asked(base);
    r.expire();
    r.asked(at(base, 2000));
    assert!(!r.packet(at(base, 2100)));
    assert_eq!(r.out, 1);
    assert_eq!(r.hold, Duration::from_millis(3150));
    assert_eq!(r.turn(at(base, 2100 + 3149)), Turn::Wait);
    assert_eq!(r.turn(at(base, 2100 + 3150)), Turn::Expired);
    r.expire();
    assert_eq!(r.out, 0);
    r.asked(at(base, 5250));
    assert_eq!(r.out, 1);
    // Soon after its request, but it may be one that was queued: one more to try.
    assert!(!r.packet(at(base, 5350)));
    assert_eq!(r.out, 0);
    assert_eq!(r.hold, Duration::from_millis(3150));
    assert_eq!(r.turn(at(base, 5350)), Turn::Ask);
    r.asked(at(base, 5850));
    assert_eq!(r.turn(at(base, 5900)), Turn::Wait);
    assert!(!r.packet(at(base, 5950)));
    assert_eq!(r.hold, Duration::ZERO);
    assert_eq!(r.turn(at(base, 5950)), Turn::Ask);
}

/// A meter gone quiet in a hold is soon asked every 2 s.
#[test]
fn silence_halves_a_hold() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    r.asked(base);
    r.packet(at(base, 8000));
    assert_eq!(r.hold, Duration::from_secs(12));
    r.asked(at(base, 20_000));
    r.expire();
    assert_eq!(r.hold, Duration::from_secs(6));
    r.asked(at(base, 26_000));
    r.expire();
    assert_eq!(r.hold, Duration::from_secs(3));
    assert_eq!(r.turn(at(base, 26_000 + 2999)), Turn::Wait);
    assert_eq!(r.turn(at(base, 26_000 + 3000)), Turn::Expired);
}

/// A request clears what was heard: an expiry with nothing since is silence, and
/// takes nothing off.
#[test]
fn a_request_clears_what_was_heard() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    r.asked(base);
    r.asked(base);
    r.packet(base);
    assert!(r.heard);
    r.asked(base);
    assert!(!r.heard);
    let out = r.out;
    r.expire();
    assert_eq!(r.out, out);
}

/// The older of two out, then the gap between replies.
#[test]
fn a_hold_is_half_as_long_again_as_the_meter_seems_to_take() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    r.asked(base);
    r.asked(at(base, 2000));
    assert!(!r.packet(at(base, 2500)));
    assert_eq!(r.out, 1);
    assert_eq!(r.hold, Duration::from_millis(3750));
    // 1.5 s from its request and 1 s since the last packet: it is keeping up.
    assert!(!r.packet(at(base, 3500)));
    assert_eq!(r.out, 0);
    assert_eq!(r.hold, Duration::ZERO);
    r.asked(at(base, 10_000));
    r.asked(at(base, 12_000));
    r.packet(at(base, 15_000));
    assert_eq!(r.hold, Duration::from_millis(5000 * 3 / 2));
    r.packet(at(base, 18_500));
    assert_eq!(r.hold, Duration::from_millis(3500 * 3 / 2));
    // A slow reply after a slow one, with two asked between, goes by the gap.
    r.asked(at(base, 19_000));
    r.asked(at(base, 27_000));
    r.packet(at(base, 28_500));
    assert_eq!(r.hold, Duration::from_secs(15));
}

#[test]
fn a_hold_has_its_top() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    r.asked(base);
    r.packet(at(base, 1_000_000));
    assert_eq!(r.hold, Duration::from_secs(30));
}

/// Back in good time after a hold, the rate is C's again.
#[test]
fn a_reply_in_good_time_ends_a_hold() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    r.asked(base);
    r.packet(at(base, 5000));
    assert!(r.hold > Duration::ZERO);
    assert_eq!(r.out, 0);
    r.asked(at(base, 20_000));
    r.packet(at(base, 20_100));
    assert_eq!(r.hold, Duration::from_millis(7500), "one reply is not proof");
    assert_eq!(r.turn(at(base, 20_100)), Turn::Ask);
    r.asked(at(base, 20_600));
    r.packet(at(base, 20_700));
    assert_eq!(r.hold, Duration::ZERO);
    assert_eq!(r.turn(at(base, 20_700)), Turn::Ask);
}

#[test]
fn all_that_a_capture_left_are_dropped() {
    let base = Instant::now();
    let mut r = Requests::new(2, REQUEST_WAIT, base);
    assert_eq!(r.turn(base), Turn::Wait);
    assert!(r.packet(base));
    assert!(r.packet(base));
    assert!(!r.packet(base));
    assert_eq!(r.turn(base), Turn::Ask);
}

/// They were lost, not late: nothing counts them from here.
#[test]
fn an_expiry_gives_up_on_all_that_a_capture_left() {
    let base = Instant::now();
    let mut r = Requests::new(2, REQUEST_WAIT, base);
    r.expire();
    assert_eq!(r.out, 0);
    assert_eq!(r.to_drop, 0);
    assert!(!r.packet(base));
}

/// A meter on our own clock: it either queues each request or, as the K204's owner
/// reports of his, ignores one that comes while it has a reply to give.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Meter {
    Queues,
    Ignores,
}

/// What a simulated meter did.
#[derive(Debug, Default)]
struct Simulated {
    most_owed: usize,
    owed: usize,
    sent: u32,
    read: u32,
    /// The oldest sample, from its own request, in ms: over all, and in the last fifth.
    oldest: u64,
    oldest_late: u64,
    /// The longest with no sample, in ms, and from the end of the outage to a sample.
    biggest_gap: u64,
    back_after: Option<u64>,
}

/// `latency` of the time in ms, no machine stretching it. Requests numbered in `lost`
/// are lost, and all between the times of `off`. Passes as in the capture: a request
/// only with no bytes for 500 ms, read before the timer.
fn simulate(
    meter: Meter,
    latency: impl Fn(u64) -> u64,
    lost: &[u32],
    off: (u64, u64),
    limit_ms: u64,
) -> Simulated {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    let mut due: VecDeque<(u64, u64)> = VecDeque::new();
    let mut free = 0;
    let mut timer = 500;
    let mut last_sample = 0;
    let mut sim = Simulated::default();
    for t in (0..limit_ms).step_by(10) {
        let now = at(base, t);
        if due.front().is_some_and(|(d, _)| *d <= t) {
            let (_, asked) = due.pop_front().expect("looked");
            sim.read += 1;
            sim.oldest = sim.oldest.max(t - asked);
            if t >= limit_ms / 5 * 4 {
                sim.oldest_late = sim.oldest_late.max(t - asked);
            }
            sim.biggest_gap = sim.biggest_gap.max(t - last_sample);
            last_sample = t;
            if off.1 > 0 && t >= off.1 && sim.back_after.is_none() {
                sim.back_after = Some(t - off.1);
            }
            r.packet(now);
            timer = t + 500;
        } else if t >= timer {
            let turn = r.turn(now);
            if turn == Turn::Expired {
                r.expire();
            }
            if turn != Turn::Wait {
                sim.sent += 1;
                r.asked(now);
                let heard = !(lost.contains(&sim.sent) || (t >= off.0 && t < off.1));
                if heard && (meter == Meter::Queues || due.is_empty()) {
                    // 47 ms for the bytes.
                    free = free.max(t) + latency(t) + 47;
                    due.push_back((free, t));
                }
            }
            timer = t + 500;
        }
        sim.most_owed = sim.most_owed.max(due.len());
    }
    sim.owed = due.len();
    sim
}

const LATENCIES: [u64; 12] =
    [100, 1000, 1900, 2100, 2500, 3500, 4100, 5000, 8000, 9000, 12_000, 20_000];

/// Ten simulated minutes at each latency, of both meters. A meter that queues takes
/// what went out in the first silence, one per 2 s up to its first reply, and drains it
/// with no more going in: by the last fifth no sample is older than the latency.
/// `-- --nocapture` prints the table.
#[test]
fn no_latency_piles_up_on_either_meter() {
    println!("meter latency | samples/min | most owed | owed at end | oldest | oldest late | sent");
    for meter in [Meter::Queues, Meter::Ignores] {
        for latency in LATENCIES {
            let sim = simulate(meter, |_| latency, &[], (0, 0), 600_000);
            let per_min = f64::from(sim.read) / 10.0;
            println!(
                "{meter:?} {latency} | {per_min:.1} | {} | {} | {} | {} | {}",
                sim.most_owed, sim.owed, sim.oldest, sim.oldest_late, sim.sent
            );
            let most = (latency / 1000 + 3) as usize;
            assert!(sim.most_owed <= most, "{meter:?} {latency}: {sim:?}");
            assert!(sim.owed <= 2, "{meter:?} {latency}: {sim:?}");
            // What queued in the first silence has gone by the last fifth.
            assert!(sim.oldest_late <= latency + 100, "{meter:?} {latency}: {sim:?}");
            if meter == Meter::Ignores {
                assert_eq!(sim.most_owed, 1, "{latency}: {sim:?}");
            }
            // It keeps reading: at least one sample per three of its latencies.
            let least = 600_000 / (3 * latency + 3000);
            assert!(u64::from(sim.read) >= least, "{meter:?} {latency}: {sim:?}");
        }
    }
}

/// A meter back from an outage of any length answers the next request, and with the
/// wait at 2 s that is at most 2.1 s and a pass off.
#[test]
fn a_meter_back_from_an_outage_is_found_within_2_s() {
    for meter in [Meter::Queues, Meter::Ignores] {
        for off_ms in [1000, 3000, 5500, 6500, 7000, 8000, 9500, 15_000, 60_000, 300_000] {
            for phase in (0..650).step_by(50) {
                let start = 30_000 + phase;
                let sim =
                    simulate(meter, |_| 100, &[], (start, start + off_ms), start + off_ms + 30_000);
                let back = sim.back_after.expect("it came back");
                assert!(back <= 2750, "{meter:?} off {off_ms} phase {phase}: {sim:?}");
                assert!(
                    sim.biggest_gap <= off_ms + 3400,
                    "{meter:?} off {off_ms} phase {phase}: {sim:?}"
                );
            }
        }
    }
}

/// The rate comes back: a count that stayed up by one would ask once per wait.
#[test]
fn a_lost_reply_costs_one_wait_and_one_hold_and_not_the_rate() {
    for meter in [Meter::Queues, Meter::Ignores] {
        let sim = simulate(meter, |_| 100, &[3], (0, 0), 30_000);
        assert!(sim.owed <= 1, "{sim:?}");
        // 24 s at 650 ms a reply.
        assert!(sim.read >= 36, "{sim:?}");
    }
}

#[test]
fn what_piled_up_drains_when_the_meter_is_back_to_speed() {
    let sim = simulate(Meter::Queues, |t| if t < 20_000 { 2500 } else { 200 }, &[], (0, 0), 60_000);
    assert!(sim.most_owed <= 2, "{sim:?}");
    assert!(sim.owed <= 1, "{sim:?}");
    assert!(sim.oldest_late <= 300, "{sim:?}");
    // Over the last 30 s at 750 ms a reply.
    assert!(sim.read >= 40, "{sim:?}");
}

#[test]
fn a_meter_faster_than_the_wait_is_asked_at_c_s_rate() {
    let sim = simulate(Meter::Queues, |_| 0, &[], (0, 0), 10_000);
    // One per 500 ms of quiet and 47 ms of bytes, and no expiry.
    assert!(sim.sent >= 17, "{sim:?}");
    assert!(sim.most_owed <= 1, "{sim:?}");
}

fn silence(base: Instant) -> Silence {
    Silence::new(Duration::from_secs(10), Duration::from_secs(600), base)
}

/// With no sample yet, a `--samples` capture has nothing that says a meter is there.
#[test]
fn with_no_sample_yet_only_samples_fails_at_10_s() {
    let base = Instant::now();
    let ten = Duration::from_secs(10);
    let mut s = silence(base);
    assert_eq!(s.check(true, at(base, 9_999)), None);
    assert_eq!(s.check(true, at(base, 10_000)), Some(Outage::Fail(Silent::NoBytes, ten)));
    let mut s = silence(base);
    s.bytes();
    assert_eq!(s.check(true, at(base, 10_000)), Some(Outage::Fail(Silent::NoPacket, ten)));
}

/// Many bounds on a moved clock, with samples between.
#[test]
fn a_sample_starts_the_silence_again() {
    let base = Instant::now();
    let mut s = silence(base);
    for i in 0..10u64 {
        let t = i * 9_000;
        assert_eq!(s.sample(at(base, t)), None);
        assert_eq!(s.check(true, at(base, t + 9_000)), None);
    }
    let last = 9 * 9_000;
    assert_eq!(s.check(true, at(base, last + 9_999)), None);
    assert_eq!(s.check(true, at(base, last + 10_000)), Some(Outage::Warn(Silent::NoBytes)));
}

#[test]
fn after_a_sample_only_samples_warns_once_and_fails_at_10_minutes() {
    let base = Instant::now();
    let mut s = silence(base);
    s.sample(base);
    assert_eq!(s.check(true, at(base, 9_999)), None);
    assert_eq!(s.check(true, at(base, 10_000)), Some(Outage::Warn(Silent::NoBytes)));
    assert_eq!(s.check(true, at(base, 10_500)), None);
    assert_eq!(s.check(true, at(base, 599_999)), None);
    assert_eq!(
        s.check(true, at(base, 600_000)),
        Some(Outage::Fail(Silent::NoBytes, Duration::from_secs(600)))
    );
}

/// `--continuous` and `--time` get the same mark, and no end.
#[test]
fn with_another_way_to_end_it_only_warns() {
    let base = Instant::now();
    let mut s = silence(base);
    assert_eq!(s.check(false, at(base, 9_999)), None);
    assert_eq!(s.check(false, at(base, 10_000)), Some(Outage::Warn(Silent::NoBytes)));
    assert_eq!(s.check(false, at(base, 10_500)), None);
    assert_eq!(s.check(false, at(base, 100_000_000)), None);
}

#[test]
fn a_sample_after_the_warning_says_how_long_and_arms_the_next() {
    let base = Instant::now();
    let mut s = silence(base);
    s.sample(base);
    s.bytes();
    assert_eq!(s.check(false, at(base, 10_000)), Some(Outage::Warn(Silent::NoPacket)));
    assert_eq!(s.sample(at(base, 15_000)), Some(Duration::from_secs(15)));
    assert_eq!(s.sample(at(base, 16_000)), None);
    assert_eq!(s.check(false, at(base, 26_000)), Some(Outage::Warn(Silent::NoBytes)));
}

#[test]
fn the_words_of_an_outage_are_these() {
    let ten = Duration::from_secs(10);
    assert_eq!(
        outage_warning(Silent::NoBytes, ten),
        "No answer from the thermometer for 10 s; still asking."
    );
    assert_eq!(
        outage_warning(Silent::NoPacket, ten),
        "No usable answer from the thermometer for 10 s: bytes came, but no whole packet; still \
         asking."
    );
    assert_eq!(back_line(ten), "The thermometer answers again, after 10 s with no sample.");
    assert_eq!(
        outage_failure(Silent::NoBytes, ten),
        (
            "No answer from the thermometer for 10 s.".to_owned(),
            "no answer from the thermometer for 10 s".to_owned()
        )
    );
    assert_eq!(
        outage_failure(Silent::NoPacket, ten),
        (
            "No usable answer from the thermometer for 10 s: bytes came, but no whole packet."
                .to_owned(),
            "no usable answer from the thermometer for 10 s".to_owned()
        )
    );
    assert_eq!(
        burst_warning(46),
        "A reply of 46 bytes, where 45 were due: this sample may be wrong."
    );
    assert_eq!(bursts_line(3), "3 replies of this capture were of the wrong length.");
}

/// The review's replay of damaged replies: what a burst alone gives, on the buffer as a
/// capture settles it. `--nocapture` prints the rates.
mod replay {
    use super::protocol::{Reading, parse};
    use super::*;

    /// Replies per class and kind of damage, each second one damaged.
    const REPLIES: u32 = 4000;

    struct Rng(u64);
    impl Rng {
        fn below(&mut self, m: u64) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x % m
        }
    }

    #[derive(Clone, Copy, Debug)]
    enum Class {
        Room,
        Hot,
        Mixed,
        Document,
    }

    #[derive(Clone, Copy, Debug, PartialEq)]
    enum Damage {
        LostByte,
        LostRun,
        Inserted,
        Inserted02,
        Ahead,
        Ahead02,
        Flip,
        Cut,
    }

    fn reply(r: &mut Rng, class: Class) -> [u8; PACKET_SIZE] {
        let mut p = [0u8; PACKET_SIZE];
        p[..7].copy_from_slice(&[2, 0x80, 0x80, 1, 2, 2, 2]);
        p[44] = 3;
        let v;
        let mut pick = |from: u64, span: u64| i16::try_from(from + r.below(span)).unwrap();
        match class {
            Class::Room => v = [pick(150, 200), pick(150, 200), pick(150, 200), pick(150, 200)],
            Class::Hot => {
                v = [pick(200, 1000), pick(200, 1000), pick(200, 1000), pick(200, 1000)];
                p[43] = 0x0f;
            }
            Class::Mixed => {
                v = [pick(200, 800), pick(200, 800), pick(150, 800), pick(150, 800)];
                p[43] = 3;
            }
            Class::Document => {
                v = [pick(150, 200), 0x7fff, 0x7fff, 0x7fff];
                p[39] = 0x0e;
                p[43] = 0x0e;
            }
        }
        for (i, value) in v.iter().enumerate() {
            p[7 + 2 * i..9 + 2 * i].copy_from_slice(&value.to_be_bytes());
        }
        p
    }

    fn damage(r: &mut Rng, kind: Damage, p: &[u8]) -> Vec<u8> {
        let mut b = p.to_vec();
        match kind {
            Damage::LostByte => {
                b.remove(r.below(45) as usize);
            }
            Damage::LostRun => {
                let at = r.below(40) as usize;
                let end = (at + 1 + r.below(20) as usize).min(b.len());
                b.drain(at..end);
            }
            Damage::Inserted => b.insert(r.below(46) as usize, r.below(256) as u8),
            Damage::Inserted02 => b.insert(r.below(46) as usize, 2),
            Damage::Ahead => {
                for _ in 0..=r.below(3) {
                    b.insert(0, r.below(256) as u8);
                }
            }
            Damage::Ahead02 => b.insert(0, 2),
            Damage::Flip => b[r.below(45) as usize] ^= 1 << r.below(8),
            Damage::Cut => b = b[r.below(44) as usize + 1..].to_vec(),
        }
        b
    }

    fn key(reading: &Reading) -> ([u32; 4], bool) {
        (reading.temperatures.map(|t| t.value.to_bits()), reading.unit == Unit::Celsius)
    }

    #[test]
    fn framing_damage_is_put_right_or_warned_of() {
        println!("class damage | damaged | wrong samples | of them warned of | good lost");
        for class in [Class::Room, Class::Hot, Class::Mixed, Class::Document] {
            for kind in [
                Damage::LostByte,
                Damage::LostRun,
                Damage::Inserted,
                Damage::Inserted02,
                Damage::Ahead,
                Damage::Ahead02,
                Damage::Flip,
                Damage::Cut,
            ] {
                let mut r = Rng(0x1234_5678_9abc_def1 ^ (class as u64) << 8 ^ (kind as u64) << 16);
                let (mut wrong, mut warned, mut lost) = (0u32, 0u32, 0u32);
                for i in 0..REPLIES {
                    let good = reply(&mut r, class);
                    let want = key(&parse(&good));
                    let damaged = i % 2 == 0;
                    let wire = if damaged { damage(&mut r, kind, &good) } else { good.to_vec() };
                    let mut b = Buffer::default();
                    b.room()[..wire.len()].copy_from_slice(&wire);
                    b.filled(wire.len());
                    let mut got_good = false;
                    while let Some(p) = b.next_packet(true) {
                        if key(&parse(&p)) == want {
                            got_good = true;
                        } else {
                            wrong += 1;
                            warned += u32::from(wire.len() % PACKET_SIZE != 0);
                        }
                    }
                    lost += u32::from(!damaged && !got_good);
                }
                println!("{class:?} {kind:?} | {} | {wrong} | {warned} | {lost}", REPLIES / 2);
                assert_eq!(lost, 0, "{class:?} {kind:?}");
                if kind == Damage::Ahead02 || kind == Damage::Ahead && wrong > 0 {
                    assert_eq!(wrong, 0, "{class:?} {kind:?}");
                }
                // A flip keeps its length, and nothing can tell it.
                if kind != Damage::Flip {
                    assert_eq!(warned, wrong, "{class:?} {kind:?}");
                }
            }
        }
    }
}

#[test]
fn the_time_limit_ends_after_its_time() {
    let base = Instant::now();
    assert!(!time_is_up(0, base, at(base, 1_000_000)));
    assert!(!time_is_up(10, base, at(base, 10)));
    assert!(time_is_up(10, base, at(base, 11)));
    // A clock that went back ends nothing.
    assert!(!time_is_up(10, at(base, 100), base));
}
