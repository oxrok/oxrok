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
    writes: Vec<(Instant, Vec<u8>)>,
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
            writes: Vec::new(),
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
            if s.fail_reads {
                return Err(Error::device("hung up"));
            }
            if let Some(byte) = s.endless {
                buf.fill(byte);
                return Ok(buf.len());
            }
            s.reads += 1;
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
    assert_eq!(MAX_WAIT, Duration::from_secs(8));
    assert_eq!(SILENCE_BOUND, Duration::from_secs(10));
    let d = CenterDevice::from_port(Box::new(Fake::new()));
    assert_eq!(d.pass, Duration::from_millis(500));
    assert_eq!(d.request_wait, Duration::from_secs(2));
    assert_eq!(d.silence_bound, Duration::from_secs(10));
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

#[test]
fn a_late_reply_counts_and_asks_nothing_until_the_other_is_answered() {
    bounded(|| {
        let fake = Fake::new();
        fake.script([Reply::OnRelease(pkt(1)), Reply::OnRelease(pkt(2))]);
        fake.with(|s| s.otherwise = None);
        let mut d = quick(&fake);
        // The second wait is 2 s, past any stall here.
        d.request_wait = Duration::from_secs(1);
        limit(&mut d, 2);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        // The first expires and goes again; its reply is on its way.
        let late = fake.with(|s| std::mem::take(&mut s.held));
        fake.wait_for_writes(2);
        fake.feed(&late);
        fake.wait_for("the late reply read", |s| s.input.is_empty());
        // The second reply still holds off the third request: many passes go by.
        let reads = fake.with(|s| s.reads);
        fake.wait_for("passes", |s| s.reads >= reads + 20);
        assert_eq!(fake.writes(), 2);
        fake.release();
        assert_eq!(firsts(&packets(&rx)), [tenths(1), tenths(2)]);
        assert_eq!(fake.writes(), 2);
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
        fake.with(|s| s.writes.resize(MOST_WRITES, (Instant::now(), Vec::new())));
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
fn an_unanswered_thermometer_is_asked_after_2_4_8_8_s() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    assert_eq!(r.turn(base), Turn::Ask);
    r.asked(base);
    let mut t = 0;
    for wait in [2000, 4000, 8000, 8000, 8000] {
        assert_eq!(r.turn(at(base, t + wait - 1)), Turn::Wait, "{wait}");
        t += wait;
        assert_eq!(r.turn(at(base, t)), Turn::Expired, "{wait}");
        r.expire();
        r.asked(at(base, t));
    }
    // Nothing else stops them, but they stay counted.
    assert_eq!(r.out, 6);
}

#[test]
fn a_reply_in_good_time_sets_the_wait_back() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    r.asked(base);
    r.expire();
    r.asked(at(base, 2000));
    assert_eq!(r.wait, Duration::from_secs(4));
    assert!(!r.packet(at(base, 2100)));
    assert_eq!(r.out, 1);
    assert_eq!(r.wait, Duration::from_secs(4), "one is still out");
    assert!(!r.packet(at(base, 2200)));
    assert_eq!(r.out, 0);
    assert_eq!(r.wait, Duration::from_secs(2));
    assert_eq!(r.turn(at(base, 2200)), Turn::Ask);
}

#[test]
fn a_reply_too_slow_keeps_the_wait() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    r.asked(base);
    r.expire();
    r.asked(at(base, 2000));
    r.packet(at(base, 2500));
    r.packet(at(base, 4500));
    assert_eq!(r.out, 0);
    assert_eq!(r.wait, Duration::from_secs(4));
}

#[test]
fn an_expiry_after_a_packet_counts_the_rest_as_lost_and_the_wait_stays() {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    r.asked(base);
    r.asked(base);
    r.packet(base);
    r.expire();
    assert_eq!(r.out, 0);
    assert_eq!(r.wait, Duration::from_secs(2));
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

/// They were lost, not late: nothing counts them from here, and no wait grows.
#[test]
fn an_expiry_gives_up_on_all_that_a_capture_left() {
    let base = Instant::now();
    let mut r = Requests::new(2, REQUEST_WAIT, base);
    r.expire();
    assert_eq!(r.out, 0);
    assert_eq!(r.to_drop, 0);
    assert_eq!(r.wait, Duration::from_secs(2));
    assert!(!r.packet(base));
}

/// What a simulated meter did.
#[derive(Debug)]
struct Simulated {
    /// The most replies owed at any time.
    most_owed: usize,
    /// What is owed at the end.
    owed: usize,
    /// Requests sent.
    sent: u32,
    /// Replies read.
    read: u32,
    /// The oldest sample, by how long it took from its own request, in ms.
    oldest: u64,
}

/// A meter at `latency` of the time in ms, in order, on a clock of our own, so no
/// machine can stretch it. Requests numbered in `lost` are lost. Passes as in the
/// capture: a request only with no bytes for 500 ms, read before the timer.
fn simulate(latency: impl Fn(u64) -> u64, lost: &[u32], limit_ms: u64) -> Simulated {
    let base = Instant::now();
    let mut r = Requests::new(0, REQUEST_WAIT, base);
    let mut due: VecDeque<(u64, u64)> = VecDeque::new();
    let mut free = 0;
    let mut timer = 500;
    let mut sim = Simulated { most_owed: 0, owed: 0, sent: 0, read: 0, oldest: 0 };
    for t in (0..limit_ms).step_by(10) {
        let now = at(base, t);
        if due.front().is_some_and(|(d, _)| *d <= t) {
            let (_, asked) = due.pop_front().expect("looked");
            sim.read += 1;
            sim.oldest = sim.oldest.max(t - asked);
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
                if !lost.contains(&sim.sent) {
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

/// At a steady 2.5 s, for a simulated minute: 2 replies owed at the most and one
/// at the end, the oldest sample 3.1 s from its request (the one behind the first
/// expiry) and the rest 2.55 s, and a reply about every 3 s.
#[test]
fn a_steady_latency_past_the_expiry_does_not_pile_up() {
    let sim = simulate(|_| 2500, &[], 60_000);
    assert!(sim.most_owed <= 2, "{sim:?}");
    assert!(sim.owed <= 1, "{sim:?}");
    assert!(sim.oldest <= 3100, "{sim:?}");
    assert!(sim.read >= 18, "{sim:?}");
    assert!(sim.sent <= sim.read + 2, "{sim:?}");
}

#[test]
fn what_piled_up_drains_when_the_meter_is_back() {
    let sim = simulate(|t| if t < 20_000 { 2500 } else { 200 }, &[], 45_000);
    assert!(sim.most_owed <= 2, "{sim:?}");
    assert!(sim.owed <= 1, "{sim:?}");
    // Over 25 s at 750 ms a reply, once the wait has come back.
    assert!(sim.read >= 30, "{sim:?}");
}

/// The rate comes back: a count that stayed up by one would ask once per wait. The
/// 2 s until it asks again, and 4 s until the count is let go, cost 6 s of 30.
#[test]
fn a_lost_reply_costs_6_s_and_not_the_rate() {
    let sim = simulate(|_| 100, &[3], 30_000);
    assert!(sim.owed <= 1, "{sim:?}");
    // 24 s at 650 ms a reply.
    assert!(sim.read >= 35, "{sim:?}");
}

#[test]
fn a_meter_faster_than_a_pass_is_asked_at_c_s_rate() {
    let sim = simulate(|_| 0, &[], 10_000);
    // One per 500 ms of quiet and 47 ms of bytes, and no expiry.
    assert!(sim.sent >= 17, "{sim:?}");
    assert!(sim.most_owed <= 1, "{sim:?}");
}

#[test]
fn the_silence_bound_runs_from_the_last_sample() {
    let base = Instant::now();
    let bound = Duration::from_secs(10);
    let mut s = Silence::new(bound, base);
    // Many bounds on a moved clock, with samples between.
    for i in 0..10u64 {
        let t = i * 9_000;
        s.sample(at(base, t));
        assert_eq!(s.verdict(at(base, t + 9_000)), None);
    }
    let last = 9 * 9_000;
    assert_eq!(s.verdict(at(base, last + 9_999)), None);
    assert_eq!(s.verdict(at(base, last + 10_000)), Some(Silent::NoBytes));
}

#[test]
fn bytes_do_not_reset_the_bound_and_change_the_verdict() {
    let base = Instant::now();
    let mut s = Silence::new(Duration::from_secs(10), base);
    for t in 0..100 {
        s.bytes();
        assert_eq!(s.verdict(at(base, t * 99)), None);
    }
    assert_eq!(s.verdict(at(base, 10_000)), Some(Silent::NoPacket));
    // A sample starts it again, and says nothing has come since.
    s.sample(at(base, 10_000));
    assert_eq!(s.verdict(at(base, 20_000)), Some(Silent::NoBytes));
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
