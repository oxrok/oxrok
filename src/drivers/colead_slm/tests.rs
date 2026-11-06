// Counterpart in sigrok:
//   libsigrok/src/hardware/colead-slm/protocol.c,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Martin Ling, Daniel Elstner,
//   Aurelien Jacobs, Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

#![allow(
    clippy::float_cmp,
    clippy::assert_is_empty,
    clippy::single_match_else,
    clippy::unnecessary_literal_bound
)]

use super::*;
use crate::core::{Mq, MqFlags};
use protocol::{READY, RECORD_LEN};
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// 75.3 dB, A weighted, fast.
const GOOD: [u8; RECORD_LEN] = [0x08, 0x04, 0x10, 0x0a, 0x0a, 0x07, 0x05, 0x03, 0x01, 0x40];
/// 80.0 dB, A weighted, slow, max hold.
const OTHER: [u8; RECORD_LEN] = [0x08, 0x04, 0x21, 0x0a, 0x0a, 0x08, 0x00, 0x00, 0x01, 0x4a];
/// A bad status.
const INVALID: [u8; RECORD_LEN] = [0x08, 0x04, 0x10, 0x0a, 0x0a, 0x07, 0x05, 0x03, 0x00, 0x3f];

/// The most a fake or a helper keeps: a mutant that ignores a stop or a limit must
/// fail a test, not fill the memory.
const MAX_LOG: usize = 4096;

/// Runs a test on its own thread with a bound: a stop or a limit a mutant ignores
/// fails here, as the device's drop would join for ever.
fn within<T: Send + 'static>(seconds: u64, test: impl FnOnce() -> T + Send + 'static) -> T {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _ = tx.send(test());
    });
    rx.recv_timeout(Duration::from_secs(seconds))
        .unwrap_or_else(|e| panic!("no end within {seconds} s: {e}"))
}

#[derive(Clone, Debug)]
enum Read {
    /// These bytes, as one read gives them; what the buffer leaves stays.
    Bytes(Vec<u8>),
    /// A read that fails.
    Fail,
}

/// What the test can see of a port the device owns.
#[derive(Default)]
struct Shared {
    writes: Vec<u8>,
    /// Write attempts, whatever came of them.
    attempts: usize,
    reads: usize,
    closed: bool,
    writes_after_close: usize,
}

/// A port that gives its reads in turn, one per call, whatever the time, and then
/// nothing, or [`Script::repeat`] for ever. A read gets its bytes at once, so this
/// shows nothing of pace.
struct Script {
    reads: VecDeque<Read>,
    /// What each read after the script gives.
    repeat: Option<Vec<u8>>,
    /// What each write gives, in turn, and then 1.
    write_results: VecDeque<Result<usize>>,
    /// How long the first read takes, for a limit that runs out during it.
    first_read_delay: Duration,
    /// What each write in turn puts behind the reads: a record that its request
    /// brings.
    held_until_write: Vec<Vec<u8>>,
    shared: Arc<Mutex<Shared>>,
}

impl Script {
    fn new(reads: &[&[u8]]) -> (Script, Arc<Mutex<Shared>>) {
        let shared = Arc::new(Mutex::new(Shared::default()));
        let script = Script {
            reads: reads.iter().map(|r| Read::Bytes(r.to_vec())).collect(),
            repeat: None,
            write_results: VecDeque::new(),
            first_read_delay: Duration::ZERO,
            held_until_write: Vec::new(),
            shared: Arc::clone(&shared),
        };
        (script, shared)
    }
}

impl Port for Script {
    fn name(&self) -> &str {
        "script"
    }
    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }
    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        self.shared.lock().map(|mut s| s.reads += 1).ok();
        std::thread::sleep(std::mem::take(&mut self.first_read_delay));
        let item = self.reads.pop_front();
        let bytes = match item {
            Some(Read::Fail) => return Err(Error::Io(std::io::ErrorKind::BrokenPipe.into())),
            Some(Read::Bytes(b)) => b,
            None => match &self.repeat {
                Some(r) => {
                    // A flood, not a spin: a stop a mutant ignores stays cheap.
                    std::thread::sleep(Duration::from_micros(100));
                    r.clone()
                }
                None => {
                    std::thread::sleep(timeout.min(Duration::from_millis(5)));
                    return Ok(0);
                }
            },
        };
        let n = bytes.len().min(buf.len());
        buf[..n].copy_from_slice(&bytes[..n]);
        if n < bytes.len() {
            self.reads.push_front(Read::Bytes(bytes[n..].to_vec()));
        }
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.write_timeout(data, Duration::ZERO).map(|_| ())
    }
    fn write_timeout(&mut self, data: &[u8], _: Duration) -> Result<usize> {
        if let Ok(mut s) = self.shared.lock() {
            s.attempts += 1;
            if s.closed {
                s.writes_after_close += 1;
            }
        }
        match self.write_results.pop_front() {
            Some(Ok(0)) => Ok(0),
            Some(Err(e)) => Err(e),
            _ => {
                if let Ok(mut s) = self.shared.lock()
                    && s.writes.len() < MAX_LOG
                {
                    s.writes.extend_from_slice(data);
                }
                if !self.held_until_write.is_empty() {
                    let bytes = self.held_until_write.remove(0);
                    self.reads.push_back(Read::Bytes(bytes));
                }
                Ok(data.len())
            }
        }
    }
    fn purge(&mut self) -> Result<()> {
        Ok(())
    }
}

impl Drop for Script {
    fn drop(&mut self) {
        if let Ok(mut s) = self.shared.lock() {
            s.closed = true;
        }
    }
}

/// A meter at the line's real rate, on the clock: an announcement each `period`
/// and a record `latency` behind a request, a byte per 4.17 ms at 2400 Bd. With
/// `every`, what has come shows only at multiples of it, as a cable that queues.
struct Line {
    start: Instant,
    period: Duration,
    latency: Duration,
    every: Option<Duration>,
    /// Bytes and when they arrive, in order.
    arriving: VecDeque<(Duration, u8)>,
    announced: u32,
    records: Vec<[u8; RECORD_LEN]>,
    asked: usize,
    shared: Arc<Mutex<Shared>>,
}

impl Line {
    const BYTE: Duration = Duration::from_micros(4167);

    fn new(records: &[[u8; RECORD_LEN]], every: Option<Duration>) -> (Line, Arc<Mutex<Shared>>) {
        let shared = Arc::new(Mutex::new(Shared::default()));
        let line = Line {
            start: Instant::now(),
            period: Duration::from_millis(100),
            latency: Duration::ZERO,
            every,
            arriving: VecDeque::new(),
            announced: 0,
            records: records.to_vec(),
            asked: 0,
            shared: Arc::clone(&shared),
        };
        (line, shared)
    }

    /// The announcements due by `now`, in their place among what arrives.
    fn announce(&mut self, now: Duration) {
        while self.period * (self.announced + 1) <= now {
            self.announced += 1;
            let at = self.period * self.announced;
            // One UART: it waits for what is on the line.
            let at = self.arriving.back().map_or(at, |(t, _)| at.max(*t + Self::BYTE));
            self.arriving.push_back((at, READY));
        }
    }

    /// What can be seen at `now`.
    fn visible(&self, now: Duration) -> Duration {
        match self.every {
            Some(every) => every * u32::try_from(now.as_nanos() / every.as_nanos()).unwrap(),
            None => now,
        }
    }
}

impl Port for Line {
    fn name(&self) -> &str {
        "line"
    }
    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }
    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let deadline = Instant::now() + timeout;
        loop {
            let now = self.start.elapsed();
            self.announce(now);
            let seen = self.visible(now);
            let mut n = 0;
            while n < buf.len() && self.arriving.front().is_some_and(|(t, _)| *t <= seen) {
                buf[n] = self.arriving.pop_front().expect("just looked").1;
                n += 1;
            }
            if n > 0 {
                return Ok(n);
            }
            if Instant::now() >= deadline {
                return Ok(0);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.write_timeout(data, Duration::ZERO).map(|_| ())
    }
    fn write_timeout(&mut self, data: &[u8], _: Duration) -> Result<usize> {
        let mut shared = self.shared.lock().expect("shared");
        if shared.writes.len() < MAX_LOG {
            shared.writes.extend_from_slice(data);
        }
        drop(shared);
        let now = self.start.elapsed();
        self.announce(now);
        let record = self.records[self.asked % self.records.len()];
        self.asked += 1;
        let begin = now + self.latency;
        let begin = self.arriving.back().map_or(begin, |(t, _)| begin.max(*t));
        let mut at = begin;
        for byte in record {
            at += Self::BYTE;
            self.arriving.push_back((at, byte));
        }
        Ok(data.len())
    }
    fn purge(&mut self) -> Result<()> {
        Ok(())
    }
}

/// An opened device on `port` with these limits.
fn device(port: impl Port + 'static, samples: u64, msec: u64) -> SlmDevice {
    let mut d = SlmDevice::from_port(Box::new(port));
    d.limits.samples.store(samples, Ordering::SeqCst);
    d.limits.msec.store(msec, Ordering::SeqCst);
    d.open().unwrap();
    d
}

/// All the packets of a capture, up to its `End`, with an overall deadline.
fn drain(rx: &Receiver<(usize, Packet)>, deadline: Duration) -> Vec<Packet> {
    let until = Instant::now() + deadline;
    let mut out = Vec::new();
    loop {
        let left = until.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((_, Packet::End)) => return out,
            Ok((_, p)) => {
                out.push(p);
                assert!(out.len() < MAX_LOG, "no End after {MAX_LOG} packets");
            }
            Err(e) => panic!("no End within {deadline:?}: {e}; had {out:?}"),
        }
    }
}

/// A capture on `d` to its `End`.
fn capture(d: &mut SlmDevice) -> Vec<Packet> {
    let (tx, rx) = sync_channel(256);
    d.start(PacketSink::new(tx, 0)).unwrap();
    drain(&rx, Duration::from_secs(30))
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

fn failure(packets: &[Packet]) -> Option<&str> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(why) => Some(why.as_str()),
        _ => None,
    })
}

#[test]
fn constants_are_the_ones_said() {
    assert_eq!(SERIALCOMM, "2400/8n1");
    assert_eq!(SEND, 0x20);
    assert_eq!(READY, 0x10);
    assert_eq!(READ_TIMEOUT, Duration::from_millis(150));
    assert_eq!(NO_LEVEL_BOUND, Duration::from_secs(15));
    assert_eq!(TIME_GRACE, Duration::from_secs(1));
    assert_eq!(END_AFTER_STOP, Duration::from_millis(500));
}

#[test]
fn a_scan_needs_conn_and_opens_nothing() {
    let driver = ColeadSlmDriver;
    assert!(driver.scan(&[]).unwrap().is_empty());
    let conn = (Key::Conn, Value::Str("/dev/no-such-port".to_owned()));
    let found = driver.scan(std::slice::from_ref(&conn)).unwrap();
    assert_eq!(found.len(), 1);
    let info = found[0].info();
    assert_eq!((info.vendor.as_str(), info.model.as_str()), ("Colead", "SL-5868P"));
    assert_eq!(found[0].channels()[0].name, "P1");
    // A serialcomm is looked at by the open.
    let bad = (Key::Serialcomm, Value::Str("fast".to_owned()));
    let mut found = driver.scan(&[conn, bad]).unwrap();
    assert!(found[0].open().is_err());
}

#[test]
fn there_is_no_get_and_only_limits_can_be_set() {
    let (script, _) = Script::new(&[]);
    let mut d = device(script, 0, 0);
    assert!(d.get(&Key::LimitSamples, None).is_err());
    assert!(d.list(&Key::LimitSamples, None).is_err());
    assert!(d.set(&Key::LimitFrames, &Value::U64(1), None).is_err());
    d.set(&Key::LimitSamples, &Value::U64(7), None).unwrap();
    d.set(&Key::LimitMsec, &Value::U64(8), None).unwrap();
    assert_eq!(d.limits.samples.load(Ordering::SeqCst), 7);
    assert_eq!(d.limits.msec.load(Ordering::SeqCst), 8);
    assert!(d.set(&Key::LimitMsec, &Value::Str("x".to_owned()), None).is_err());
    let keys: Vec<Key> = d.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(keys, [Key::Continuous, Key::LimitSamples, Key::LimitMsec]);
}

#[test]
fn a_level_goes_out_with_its_flags() {
    within(20, || {
        let (mut script, shared) = Script::new(&[&[READY]]);
        script.held_until_write.push(OTHER.to_vec());
        let mut d = device(script, 1, 0);
        let packets = capture(&mut d);
        assert!(failure(&packets).is_none(), "{packets:?}");
        let Packet::Analog(a) = &packets[1] else { panic!("{packets:?}") };
        assert_eq!(a.samples, [80.0]);
        assert_eq!(a.channels, [0]);
        assert!(matches!(a.mq, Mq::SoundPressureLevel));
        assert_eq!(a.digits, 1);
        let want = MqFlags::HOLD | MqFlags::SPL_FREQ_WEIGHT_A | MqFlags::SPL_TIME_WEIGHT_S;
        assert_eq!(a.flags, want);
        assert_eq!(shared.lock().unwrap().writes, [SEND]);
    });
}

#[test]
fn the_limit_counts_levels_and_nothing_else_and_ends_there() {
    within(20, || {
        let (script, shared) =
            Script::new(&[&[READY], &INVALID, &[READY], &GOOD, &[READY], &OTHER, &[READY], &GOOD]);
        let mut d = device(script, 2, 0);
        let packets = capture(&mut d);
        assert_eq!(values(&packets), [75.3, 80.0]);
        assert!(failure(&packets).is_none());
        // A request per announcement, and the fourth announcement left unread.
        let shared = shared.lock().unwrap();
        assert_eq!(shared.writes, [SEND, SEND, SEND]);
        assert_eq!(shared.reads, 6);
    });
}

#[test]
fn no_request_without_a_read_of_the_one_byte() {
    within(20, || {
        let (script, shared) = Script::new(&[
            &[READY, READY],
            &[0xff],
            &[0x40, READY],
            &[READY, 0x40],
            &[READY],
            &GOOD,
        ]);
        let mut d = device(script, 1, 0);
        let packets = capture(&mut d);
        assert_eq!(values(&packets), [75.3]);
        assert_eq!(shared.lock().unwrap().writes, [SEND]);
    });
}

#[test]
fn a_record_read_in_pieces_is_one_record() {
    within(20, || {
        let mut reads: Vec<&[u8]> = vec![&[READY]];
        reads.extend(GOOD.chunks(1));
        let (script, shared) = Script::new(&reads);
        let mut d = device(script, 1, 0);
        assert_eq!(values(&capture(&mut d)), [75.3]);
        assert_eq!(shared.lock().unwrap().writes, [SEND]);
    });
}

#[test]
fn a_reply_with_bytes_too_many_stays_out_of_the_record() {
    within(20, || {
        let mut long = GOOD.to_vec();
        long.push(0xff);
        let (script, shared) = Script::new(&[&[READY], &long, &[READY], &OTHER]);
        let mut d = device(script, 2, 0);
        assert_eq!(values(&capture(&mut d)), [75.3, 80.0]);
        assert_eq!(shared.lock().unwrap().writes, [SEND, SEND]);
    });
}

#[test]
fn a_late_record_after_an_announcement_loses_a_level_and_shifts_nothing() {
    within(20, || {
        let (script, shared) = Script::new(&[&[READY], &[READY], &GOOD, &[READY], &OTHER]);
        let mut d = device(script, 1, 0);
        // The record starts a byte late, so it is refused; what is over is
        // read alone and dropped.
        assert_eq!(values(&capture(&mut d)), [80.0]);
        assert_eq!(shared.lock().unwrap().writes, [SEND, SEND]);
    });
}

#[test]
fn a_time_limit_ends_silence_at_its_time_from_both_sides() {
    within(20, || {
        let (script, _) = Script::new(&[]);
        let mut d = device(script, 0, 300);
        let began = Instant::now();
        let packets = capture(&mut d);
        assert!(began.elapsed() >= Duration::from_millis(300), "{:?}", began.elapsed());
        assert!(failure(&packets).is_none(), "{packets:?}");
        assert!(values(&packets).is_empty());
    });
}

#[test]
fn what_a_read_gives_comes_before_the_timer_and_a_level_before_the_end() {
    within(20, || {
        // The limit runs out in the first read. C sends the request, takes the
        // level and looks at the time behind it: one level, and no more asked.
        let (mut script, shared) = Script::new(&[&[READY], &GOOD, &[READY], &OTHER]);
        script.first_read_delay = Duration::from_millis(20);
        let mut d = device(script, 0, 1);
        let packets = capture(&mut d);
        assert_eq!(values(&packets), [75.3]);
        assert!(failure(&packets).is_none());
        assert_eq!(shared.lock().unwrap().writes, [SEND]);
    });
}

#[test]
fn a_time_limit_ends_a_flood_with_no_level_soon_after_its_grace() {
    within(20, || {
        let (mut script, _) = Script::new(&[]);
        script.repeat = Some(vec![0xff]);
        let mut d = device(script, 0, 100);
        let began = Instant::now();
        let packets = capture(&mut d);
        let took = began.elapsed();
        assert!(took >= Duration::from_millis(1100), "{took:?}");
        assert!(failure(&packets).is_none(), "{packets:?}");
    });
}

fn with_bound(mut d: SlmDevice, bound: Duration) -> SlmDevice {
    d.no_level_bound = bound;
    d
}

#[test]
fn a_sample_limit_on_silence_fails_and_says_no_bytes() {
    within(20, || {
        let (script, shared) = Script::new(&[]);
        let mut d = with_bound(device(script, 1, 0), Duration::from_secs(1));
        let began = Instant::now();
        let packets = capture(&mut d);
        assert!(began.elapsed() >= Duration::from_secs(1));
        assert_eq!(
            failure(&packets),
            Some(
                "device error: no bytes from the meter for 1 s: it announces two measurements a \
                 second, so it is off, on another line setting, or its cable is out"
            )
        );
        assert!(shared.lock().unwrap().writes.is_empty());
    });
}

#[test]
fn a_sample_limit_on_bytes_with_no_level_fails_and_says_so() {
    for reads in [
        // One stray byte, then silence.
        vec![vec![0xff]],
        // An announcement, its request, and no record.
        vec![vec![READY]],
        // Part of a record.
        vec![vec![READY], GOOD[..4].to_vec()],
    ] {
        within(20, move || {
            let refs: Vec<&[u8]> = reads.iter().map(Vec::as_slice).collect();
            let (script, _) = Script::new(&refs);
            let mut d = with_bound(device(script, 1, 0), Duration::from_secs(1));
            let packets = capture(&mut d);
            assert_eq!(
                failure(&packets),
                Some("device error: no usable level from the meter for 1 s, though bytes came")
            );
        });
    }
}

#[test]
fn a_sample_limit_ends_for_ever_refused_records() {
    within(20, || {
        let (mut script, shared) = Script::new(&[]);
        // More rounds than the bound's time reads, then a flood.
        script.reads = (0..5000)
            .flat_map(|_| [Read::Bytes(vec![READY]), Read::Bytes(INVALID.to_vec())])
            .collect();
        script.repeat = Some(vec![0xff]);
        let mut d = with_bound(device(script, 1, 0), Duration::from_millis(1000));
        let packets = capture(&mut d);
        assert_eq!(
            failure(&packets),
            Some("device error: no usable level from the meter for 1 s, though bytes came")
        );
        assert!(!shared.lock().unwrap().writes.is_empty());
    });
}

#[test]
fn levels_restart_the_bound() {
    within(20, || {
        let (script, _) = Script::new(&[]);
        let mut port = script;
        // Levels for longer than the bound.
        for _ in 0..3 {
            port.reads.push_back(Read::Bytes(vec![READY]));
            port.reads.push_back(Read::Bytes(GOOD.to_vec()));
        }
        let mut d = with_bound(device(port, 3, 0), Duration::from_secs(1));
        let packets = capture(&mut d);
        assert_eq!(values(&packets).len(), 3);
        assert!(failure(&packets).is_none());
    });
}

#[test]
fn with_a_time_limit_or_no_limit_the_bound_does_not_apply() {
    within(20, || {
        let (script, _) = Script::new(&[]);
        let mut d = with_bound(device(script, 1, 1500), Duration::from_millis(1000));
        let began = Instant::now();
        let packets = capture(&mut d);
        assert!(began.elapsed() >= Duration::from_millis(1500));
        assert!(failure(&packets).is_none(), "{packets:?}");

        let (script, _) = Script::new(&[]);
        let mut d = with_bound(device(script, 0, 0), Duration::from_millis(100));
        let (tx, rx) = sync_channel(16);
        d.start(PacketSink::new(tx, 0)).unwrap();
        std::thread::sleep(Duration::from_millis(400));
        d.stop().unwrap();
        let packets = drain(&rx, Duration::from_secs(10));
        assert!(failure(&packets).is_none(), "{packets:?}");
    });
}

#[test]
fn a_slow_consumer_is_no_silence() {
    within(30, || {
        let (script, _) = Script::new(&[&[READY], &GOOD, &[READY], &GOOD]);
        let mut d = with_bound(device(script, 2, 0), Duration::from_secs(1));
        // Room for the header alone: the first level waits on this consumer for
        // longer than the bound.
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // By the clock: a slow sleep must not stretch this past the test's bound.
        let began = Instant::now();
        while began.elapsed() < Duration::from_millis(2500) {
            std::thread::sleep(Duration::from_millis(5));
        }
        let packets = drain(&rx, Duration::from_secs(10));
        assert_eq!(values(&packets).len(), 2);
        assert!(failure(&packets).is_none(), "{packets:?}");
    });
}

#[test]
fn a_read_that_fails_ends_the_capture_with_its_error() {
    within(20, || {
        let (mut script, _) = Script::new(&[&[READY]]);
        script.reads.push_back(Read::Fail);
        let mut d = device(script, 3, 0);
        let packets = capture(&mut d);
        assert!(failure(&packets).is_some_and(|f| f.contains("roken pipe")), "{packets:?}");
    });
}

#[test]
fn a_write_that_fails_ends_the_capture_with_its_error() {
    within(20, || {
        let (mut script, shared) = Script::new(&[&[READY], &[READY]]);
        script.write_results.push_back(Err(Error::Io(std::io::ErrorKind::BrokenPipe.into())));
        let mut d = device(script, 1, 0);
        let packets = capture(&mut d);
        assert!(failure(&packets).is_some_and(|f| f.contains("roken pipe")), "{packets:?}");
        assert_eq!(shared.lock().unwrap().attempts, 1);
    });
}

#[test]
fn a_write_with_no_room_waits_for_the_next_announcement() {
    within(20, || {
        // The bytes of a record that was never asked for can't be read as one.
        let (mut script, shared) = Script::new(&[&[READY], &GOOD, &[READY], &OTHER]);
        script.write_results.push_back(Ok(0));
        let mut d = device(script, 1, 0);
        let packets = capture(&mut d);
        assert_eq!(values(&packets), [80.0]);
        assert!(failure(&packets).is_none());
        let shared = shared.lock().unwrap();
        assert_eq!(shared.attempts, 2);
        assert_eq!(shared.writes, [SEND]);
    });
}

#[test]
fn a_stop_ends_a_capture_that_bytes_keep_busy() {
    within(20, || {
        let (mut script, shared) = Script::new(&[]);
        script.repeat = Some(vec![READY]);
        let mut d = device(script, 0, 0);
        let (tx, rx) = sync_channel(16);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let until = Instant::now() + Duration::from_secs(10);
        while shared.lock().unwrap().reads < 20 {
            assert!(Instant::now() < until, "no reads");
            std::thread::yield_now();
        }
        d.stop().unwrap();
        let packets = drain(&rx, Duration::from_secs(5));
        assert!(failure(&packets).is_none(), "{packets:?}");
        d.close().unwrap();
        let shared = shared.lock().unwrap();
        assert!(shared.closed);
        assert_eq!(shared.writes_after_close, 0);
    });
}

#[test]
fn a_close_waits_for_the_thread_and_nothing_is_written_after_it() {
    within(20, || {
        let (mut script, shared) = Script::new(&[]);
        script.repeat = Some(vec![READY]);
        let mut d = device(script, 0, 0);
        let (tx, rx) = sync_channel(MAX_LOG);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let until = Instant::now() + Duration::from_secs(10);
        while shared.lock().unwrap().attempts == 0 {
            assert!(Instant::now() < until, "no request");
            std::thread::yield_now();
        }
        // No stop first, and the thread busy: the close itself ends it.
        d.close().unwrap();
        assert!(shared.lock().unwrap().closed, "the close left the port open");
        let attempts = shared.lock().unwrap().attempts;
        std::thread::sleep(Duration::from_millis(30));
        let shared = shared.lock().unwrap();
        assert_eq!(shared.attempts, attempts);
        assert_eq!(shared.writes_after_close, 0);
        drop(rx);
    });
}

#[test]
fn a_close_gets_past_a_full_queue() {
    within(20, || {
        let (mut script, _) = Script::new(&[]);
        for _ in 0..50 {
            script.reads.push_back(Read::Bytes(vec![READY]));
            script.reads.push_back(Read::Bytes(GOOD.to_vec()));
        }
        let mut d = device(script, 0, 0);
        let (tx, _rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        std::thread::sleep(Duration::from_millis(50));
        d.close().unwrap();
    });
}

#[test]
fn a_consumer_that_leaves_ends_the_capture() {
    within(20, || {
        let (mut script, _) = Script::new(&[]);
        script.repeat = Some(vec![READY]);
        let mut d = device(script, 0, 0);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        drop(rx);
        d.close().unwrap();
    });
}

#[test]
fn a_second_capture_starts_with_no_record_owed() {
    within(20, || {
        // The first capture stops with half a record in its hands: the second
        // starts from nothing, and the tail is dropped.
        let (script, shared) = Script::new(&[&[READY], &GOOD[..4]]);
        let mut d = device(script, 0, 0);
        let (tx, rx) = sync_channel(16);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let until = Instant::now() + Duration::from_secs(10);
        while shared.lock().unwrap().reads < 4 {
            assert!(Instant::now() < until, "no reads");
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        assert!(values(&drain(&rx, Duration::from_secs(5))).is_empty());
        // The old sink goes with its thread: nothing holds it after the restart.
        let (tx2, rx2) = sync_channel(16);
        let until = Instant::now() + Duration::from_secs(10);
        loop {
            match d.start(PacketSink::new(tx2.clone(), 0)) {
                Ok(()) => break,
                Err(_) => {
                    assert!(Instant::now() < until, "still busy");
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
        }
        drop(tx2);
        assert!(rx.recv().is_err(), "the first capture's sink is still held");
        d.stop().unwrap();
        assert!(failure(&drain(&rx2, Duration::from_secs(5))).is_none());
    });
}

/// Three levels from `line`: each is a record the meter holds, whatever a stall
/// makes of how many requests that takes.
fn three_levels(line: Line, shared: &Mutex<Shared>) {
    let mut d = device(line, 3, 0);
    let packets = capture(&mut d);
    assert!(failure(&packets).is_none(), "{packets:?}");
    let got = values(&packets);
    assert_eq!(got.len(), 3, "{got:?}");
    assert!(got.iter().all(|v| *v == 75.3 || *v == 80.0), "{got:?}");
    let writes = shared.lock().unwrap().writes.clone();
    assert!(writes.len() >= 3 && writes.iter().all(|b| *b == SEND), "{writes:02x?}");
}

#[test]
fn a_meter_at_the_lines_rate() {
    within(60, || {
        let (line, shared) = Line::new(&[GOOD, OTHER], None);
        three_levels(line, &shared);
    });
}

#[test]
fn a_cable_that_queues_for_50_ms() {
    within(60, || {
        let (line, shared) = Line::new(&[GOOD, OTHER], Some(Duration::from_millis(50)));
        three_levels(line, &shared);
    });
}

#[test]
fn levels_for_longer_than_the_bound_are_not_silence() {
    within(60, || {
        // Each comes within the bound, and all of them take longer than it.
        let (line, shared) = Line::new(&[GOOD], None);
        let mut d = with_bound(device(line, 15, 0), Duration::from_secs(1));
        let packets = capture(&mut d);
        assert!(failure(&packets).is_none(), "{packets:?}");
        assert_eq!(values(&packets).len(), 15);
        assert!(shared.lock().unwrap().writes.len() >= 15);
    });
}

/// The rule of one request at a time, run on a simulated meter with no real time:
/// announcements every 500 ms, a reply `latency` behind each request, for a minute.
/// What comes in each read is one byte. Gives the most replies ever owed, the
/// requests sent, and the levels that came with how old each was at its arrival.
fn simulate(latency_ms: u64) -> (u32, u32, Vec<(f32, u64)>) {
    /// A record whose value says which request it answers.
    fn record(n: u32) -> [u8; RECORD_LEN] {
        let d = |p: u32| u8::try_from(n / p % 10).unwrap();
        let mut r = [0x08, 0x04, 0x10, d(1000), d(100), d(10), d(1), 0, 0x01, 0];
        r[9] = r[..9].iter().fold(0u8, |s, &b| s.wrapping_add(b));
        r
    }
    let mut machine = Machine::default();
    let mut owed = 0u32;
    let mut most = 0u32;
    let mut sent = 0u32;
    let mut levels = Vec::new();
    // Bytes on their way, in order of arrival: when, what, and whether it ends a
    // reply.
    let mut flying: VecDeque<(u64, u8, bool)> = VecDeque::new();
    let mut last_asked = 0u64;
    for now in 0..60_000u64 {
        if now > 0 && now % 500 == 0 {
            let place = flying.iter().position(|b| b.0 > now);
            flying.insert(place.unwrap_or(flying.len()), (now, READY, false));
        }
        while flying.front().is_some_and(|b| b.0 <= now) {
            let (_, byte, ends) = flying.pop_front().expect("just looked");
            if ends {
                owed -= 1;
            }
            match machine.got(&[byte]) {
                Step::Wait => {}
                Step::Ask => {
                    machine.asked();
                    sent += 1;
                    owed += 1;
                    most = most.max(owed);
                    last_asked = now;
                    for (i, b) in (0u64..).zip(record(sent)) {
                        let at = now + latency_ms + i * 4;
                        let place = flying.iter().position(|f| f.0 > at);
                        flying.insert(place.unwrap_or(flying.len()), (at, b, i == 9));
                    }
                }
                Step::Record(r) => {
                    if let Ok(level) = protocol::decode(&r) {
                        levels.push((level.value, now - last_asked));
                    }
                }
            }
        }
    }
    (most, sent, levels)
}

#[test]
fn a_meter_that_answers_quickly_gives_a_level_per_announcement() {
    let (most, sent, levels) = simulate(5);
    assert_eq!(most, 1);
    assert_eq!(sent, 119);
    assert_eq!(levels.len(), 119);
    // Each is the answer to the last request, not an older one.
    for (i, (value, age)) in levels.iter().enumerate() {
        let n = u32::try_from(i + 1).unwrap();
        assert!((value * 10.0 - (n * 10) as f32).abs() < 0.5, "{value} for request {n}");
        assert!(*age < 100, "{age} ms");
    }
}

#[test]
fn a_steady_latency_of_two_and_a_half_seconds_owes_one_and_shifts_nothing() {
    let (most, sent, levels) = simulate(2500);
    // No backlog: one out at a time. And no level, since announcements fill a
    // record that late, as in C: no wrong one either.
    assert_eq!(most, 1, "sent {sent}");
    assert_eq!(sent, 20);
    assert!(levels.is_empty(), "{levels:?}");
}
