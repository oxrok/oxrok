// Counterpart in sigrok:
//   libsigrok/src/hardware/kern-scale/api.c,
//   libsigrok/src/hardware/kern-scale/protocol.c,
//   Copyright (C) 2015 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Gerhard Sittig, Daniel Elstner,
//   Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake port. A fake's bytes go by counts of reads, stretched by
//! a load but never lost to it, so nothing here times a race: limits on real time
//! are one-sided or wide, and each capture has an overall bound.

use super::*;
use crate::core::{MqFlags, Unit};
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// Far above anything a test waits for, however slow the box: it only makes a
/// stop or a limit that does not bite fail rather than hang.
const BOUND: Duration = Duration::from_secs(60);
/// What a byte takes at 1200/8n2.
const BYTE_TIME: Duration = Duration::from_micros(9167);

type Received = Receiver<(usize, Packet)>;

const P127: &[u8] = b"+00127.20 G S\r\n";

#[derive(Default)]
struct Line {
    /// What is on its way, with the head first.
    queued: VecDeque<u8>,
    /// Bytes a read gives at most; 0 for all.
    per_read: usize,
    /// How long a read that gives bytes takes per byte: the line's rate.
    byte_time: Duration,
    /// Reads with no bytes between two that have some: a line in bursts.
    gap_reads: usize,
    gap_left: usize,
    /// What a read finds whenever nothing is queued: a scale that streams.
    stream: Option<Vec<u8>>,
    read_fails: bool,
    /// Reads that fail before they stop failing.
    failed_reads: usize,
    write_fails: bool,
    writes: Vec<Vec<u8>>,
    /// Writes after the device was told to close; a test sets `closed`.
    closed: bool,
    writes_after_close: usize,
    purges: usize,
    reads: usize,
    /// What goes in the queue at each write, once: an answer.
    on_write: VecDeque<Vec<u8>>,
}

#[derive(Clone, Default)]
struct Shared(Arc<Mutex<Line>>);

impl Shared {
    /// The lock, even after a failed test: a fake's user may drop late.
    fn line(&self) -> std::sync::MutexGuard<'_, Line> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn port(&self) -> Box<dyn Port> {
        Box::new(FakePort(self.clone()))
    }

    fn with(self, f: impl FnOnce(&mut Line)) -> Self {
        f(&mut self.line());
        self
    }

    /// Until `n` reads happened, within the bound.
    fn wait_for_reads(&self, n: usize) {
        let deadline = Instant::now() + BOUND;
        while self.line().reads < n {
            assert!(Instant::now() < deadline, "{n} reads did not happen");
            std::thread::sleep(Duration::from_millis(1));
        }
    }
}

struct FakePort(Shared);

impl Port for FakePort {
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
        let mut line = self.0.line();
        line.reads += 1;
        if line.read_fails {
            return Err(Error::Disconnected);
        }
        if line.failed_reads > 0 {
            line.failed_reads -= 1;
            return Err(Error::Disconnected);
        }
        if line.gap_left > 0 {
            line.gap_left -= 1;
            drop(line);
            std::thread::sleep(timeout);
            return Ok(0);
        }
        if line.queued.is_empty()
            && let Some(p) = line.stream.clone()
        {
            line.queued.extend(p);
        }
        if line.queued.is_empty() {
            drop(line);
            std::thread::sleep(timeout);
            return Ok(0);
        }
        let most = if line.per_read == 0 { buf.len() } else { line.per_read.min(buf.len()) };
        let n = most.min(line.queued.len());
        for b in &mut buf[..n] {
            *b = line.queued.pop_front().expect("counted");
        }
        line.gap_left = line.gap_reads;
        let wait = line.byte_time * u32::try_from(n).expect("small");
        drop(line);
        std::thread::sleep(wait);
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut line = self.0.line();
        if line.closed {
            line.writes_after_close += 1;
        }
        if line.write_fails {
            return Err(Error::Disconnected);
        }
        line.writes.push(data.to_vec());
        if let Some(answer) = line.on_write.pop_front() {
            line.queued.extend(answer);
        }
        Ok(())
    }
    fn purge(&mut self) -> Result<()> {
        let mut line = self.0.line();
        line.queued.clear();
        line.purges += 1;
        Ok(())
    }
}

/// An open device on the line, what the line has queued being for after the
/// open's flush.
fn device(line: &Shared) -> KernScaleDevice {
    let mut d = KernScaleDevice::from_port(line.port());
    let queued = std::mem::take(&mut line.line().queued);
    d.open().unwrap();
    line.line().queued = queued;
    d
}

fn device_with_bound(line: &Shared, bound: Duration) -> KernScaleDevice {
    let mut d = device(line);
    d.silence_bound = bound;
    d
}

fn start(d: &mut KernScaleDevice) -> Received {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// A capture's packets up to its End, which must come within the bound.
fn packets(rx: &Received) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        let p = rx.recv_timeout(left).expect("no End in time").1;
        if p == Packet::End {
            return out;
        }
        // A limit that does not bite must fail here, not fill the memory.
        assert!(out.len() < 10_000, "no End after {} packets", out.len());
        out.push(p);
    }
}

fn values(got: &[Packet]) -> Vec<f32> {
    got.iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some(a.samples[0]),
            _ => None,
        })
        .collect()
}

fn failure(got: &[Packet]) -> Option<&str> {
    got.iter().find_map(|p| match p {
        Packet::Failed(why) => Some(why.as_str()),
        _ => None,
    })
}

/// Runs a test that may hang on its own thread, within the bound.
fn bounded(test: impl FnOnce() + Send + 'static) {
    let (tx, rx) = sync_channel(1);
    let thread = std::thread::spawn(move || {
        test();
        let _ = tx.send(());
    });
    match rx.recv_timeout(BOUND * 2) {
        Ok(()) => thread.join().unwrap(),
        // A panic drops the sender: report it as itself.
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(thread.join().unwrap_err());
        }
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("the test hung"),
    }
}

fn params() -> Params {
    Params::parse("1200/8n2").unwrap()
}

#[test]
fn the_constants_as_the_c_has_them() {
    assert_eq!(DETECT_TIMEOUT, Duration::from_millis(3000));
    assert_eq!(DETECT_BYTES, 128);
    assert_eq!(BUF_SIZE, 256);
    assert_eq!(READ_TIMEOUT, Duration::from_millis(50));
    assert_eq!(SILENCE_BOUND, Duration::from_secs(10));
    assert_eq!(O1, b"O1\r\n");
    assert_eq!(SERIALCOMM, "1200/8n2");
    let d = KernScaleDriver;
    assert_eq!(d.id(), "kern-ew-6200-2nm");
    assert_eq!(d.long_name(), "KERN EW 6200-2NM");
}

#[test]
fn a_scan_without_a_port_finds_nothing_and_does_not_fail() {
    let d = KernScaleDriver;
    assert!(d.scan(&[]).unwrap().is_empty());
    let conn = (Key::Conn, Value::Str("/dev/ttyNOSUCH".to_owned()));
    assert!(d.scan(std::slice::from_ref(&conn)).unwrap().is_empty());
    let bad = (Key::Serialcomm, Value::Str("fast".to_owned()));
    assert!(d.scan(&[conn, bad]).unwrap().is_empty());
}

#[test]
fn a_probe_sends_o1_and_steps_over_the_answer() {
    let line = Shared::default().with(|l| {
        l.on_write.push_back([b"A00\r\n".as_slice(), P127, b"more"].concat());
    });
    let mut port = FakePort(line.clone());
    assert!(probe(&mut port, &params(), "x", Duration::from_secs(30)));
    assert_eq!(line.line().writes, [O1]);
    assert_eq!(line.line().queued.len(), 4, "bytes it did not need stay unread");
}

#[test]
fn a_probe_that_cannot_write_finds_nothing_and_does_not_listen() {
    let line = Shared::default().with(|l| {
        l.write_fails = true;
        l.queued.extend(P127);
    });
    let mut port = FakePort(line.clone());
    assert!(!probe(&mut port, &params(), "x", Duration::from_secs(30)));
    assert_eq!(line.line().reads, 0);
}

#[test]
fn a_search_gives_up_when_its_buffer_is_full() {
    // 128 bytes and no packet in them is no device, even though one follows.
    let mut input = vec![b'?'; 114];
    input.extend(P127);
    let line = Shared::default().with(|l| l.queued.extend(input));
    assert!(!detect(&mut FakePort(line.clone()), &params(), Duration::from_secs(30)));
    assert_eq!(line.line().queued.len(), 1, "the byte it did not read");
    // A packet that ends within the 128 is found.
    let mut input = vec![b'?'; 113];
    input.extend(P127);
    let line = Shared::default().with(|l| l.queued.extend(input));
    assert!(detect(&mut FakePort(line), &params(), Duration::from_secs(30)));
}

#[test]
fn a_search_needs_15_bytes_for_a_14_byte_packet() {
    bounded(|| {
        let line = Shared::default().with(|l| l.queued.extend(b"+0026.90 G S\r\n"));
        assert!(!detect(&mut FakePort(line.clone()), &params(), Duration::from_millis(100)));
        let line = line.with(|l| l.queued.extend(b"+0026.90 G S\r\n+"));
        assert!(detect(&mut FakePort(line), &params(), Duration::from_secs(30)));
    });
}

#[test]
fn a_search_gives_up_on_time_and_a_failing_read_is_no_byte() {
    bounded(|| {
        let line = Shared::default();
        let at = Instant::now();
        assert!(!detect(&mut FakePort(line), &params(), Duration::from_millis(100)));
        assert!(at.elapsed() >= Duration::from_millis(100));
    });
    let line = Shared::default().with(|l| {
        l.failed_reads = 2;
        l.queued.extend(P127);
    });
    assert!(detect(&mut FakePort(line), &params(), Duration::from_secs(30)));
}

#[test]
fn the_keys_are_as_the_c_lists_them() {
    let mut d = device(&Shared::default());
    let options = d.options(None).unwrap();
    let keys: Vec<Key> = options.iter().map(|o| o.key.clone()).collect();
    assert_eq!(keys, [Key::Continuous, Key::LimitSamples, Key::LimitMsec]);
    assert!(options.iter().all(|o| !o.access.get && !o.access.list));
    assert_eq!(options.iter().filter(|o| o.access.set).count(), 2);
    assert!(d.get(&Key::LimitSamples, None).is_err());
    assert!(d.list(&Key::LimitSamples, None).is_err());
    assert!(d.set(&Key::LimitFrames, &Value::U64(1), None).is_err());
    assert!(d.set(&Key::LimitSamples, &Value::Str("x".to_owned()), None).is_err());
    assert!(d.options(Some("g")).is_err());
    assert_eq!(d.info().vendor, "KERN");
    assert_eq!(d.info().model, "EW 6200-2NM");
    assert_eq!(d.info().conn, None);
    assert_eq!(d.channels().len(), 1);
    assert_eq!(d.channels()[0].name, "Mass");
    assert_eq!(d.classes(), [DeviceClass::Scale]);
}

#[test]
fn a_capture_sends_o1_then_its_samples_and_stops_at_the_limit() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(P127.to_vec()));
        let mut d = device(&line);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        let got = packets(&start(&mut d));
        assert_eq!(line.line().writes, [O1]);
        assert!(matches!(got[0], Packet::Header { .. }));
        assert_eq!(values(&got), [127.2; 3]);
        assert_eq!(failure(&got), None);
        let Packet::Analog(a) = &got[1] else { panic!("{got:?}") };
        assert_eq!(a.channels, [0]);
        assert_eq!(a.mq, Mq::Mass);
        assert_eq!(a.unit, Unit::Gram);
        assert_eq!(a.digits, 2);
        assert_eq!(a.flags, MqFlags::default());
    });
}

#[test]
fn a_read_holding_two_packets_sends_both_past_the_limit() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.queued.extend(P127);
            l.queued.extend(P127);
            l.queued.extend(P127);
        });
        let mut d = device(&line);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        assert_eq!(values(&packets(&start(&mut d))).len(), 3);
    });
}

#[test]
fn the_limit_is_looked_at_after_each_read() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.per_read = 15;
            l.queued.extend(P127);
            l.queued.extend(P127);
            l.queued.extend(P127);
        });
        let mut d = device(&line);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        assert_eq!(values(&packets(&start(&mut d))).len(), 1);
    });
}

#[test]
fn packets_of_each_size_come_through_however_the_bytes_fall() {
    let mixed = [b"+0000.01 G S\r\n".as_slice(), b"+00000.02 G U\r\n", b"-0000.03 CT S\r\n"];
    let stream = mixed.concat();
    for per_read in [0, 1, 2, 7, 14, 15, 16] {
        let stream = stream.clone();
        bounded(move || {
            let line = Shared::default().with(|l| {
                l.per_read = per_read;
                l.queued.extend(&stream);
            });
            let mut d = device(&line);
            d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
            let got = packets(&start(&mut d));
            assert_eq!(values(&got).len(), 3, "{per_read} a read");
        });
    }
}

#[test]
fn a_14_byte_stream_loses_no_packet() {
    bounded(|| {
        // C reads every second one, and needs one more byte for the last.
        let mut bytes = Vec::new();
        for i in 1..=6 {
            bytes.extend_from_slice(format!("+0000.0{i} G S\r\n").as_bytes());
        }
        let line = Shared::default().with(|l| l.queued.extend(&bytes));
        let mut d = device(&line);
        d.set(&Key::LimitSamples, &Value::U64(6), None).unwrap();
        let got = packets(&start(&mut d));
        assert_eq!(values(&got), [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]);
    });
}

#[test]
fn a_cut_packet_as_captured_costs_itself_and_no_more() {
    bounded(|| {
        // sigrok-dumps' "various values": 6 bytes, 0xf0, then whole packets.
        let line = Shared::default().with(|l| {
            l.queued.extend(b"+0000.\xf0");
            l.queued.extend(b"+0000.00 G S\r\n");
            l.queued.extend(b"+0026.98 G U\r\n");
        });
        let mut d = device(&line);
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        assert_eq!(values(&packets(&start(&mut d))), [0.0, 26.98]);
    });
}

#[test]
fn a_line_at_its_real_rate_gives_every_packet() {
    bounded(|| {
        // 15 bytes at 1200/8n2 take over 137 ms, so most reads are short.
        let line = Shared::default().with(|l| {
            l.byte_time = BYTE_TIME;
            l.per_read = 1;
            l.stream = Some(P127.to_vec());
        });
        let mut d = device(&line);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        assert_eq!(values(&packets(&start(&mut d))), [127.2; 3]);
    });
}

#[test]
fn a_line_in_bursts_gives_every_packet() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.gap_reads = 1;
            l.per_read = 8;
            l.stream = Some(P127.to_vec());
        });
        let mut d = device(&line);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        assert_eq!(values(&packets(&start(&mut d))), [127.2; 3]);
    });
}

#[test]
fn a_channel_that_is_off_gets_nothing_and_its_packets_count() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(P127.to_vec()));
        let mut d = device(&line);
        d.enable_channel(0, false).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let got = packets(&start(&mut d));
        assert_eq!(values(&got), [0.0f32; 0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_time_limit_ends_a_silent_capture_at_its_time() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(&line);
        d.set(&Key::LimitMsec, &Value::U64(100), None).unwrap();
        let at = Instant::now();
        let got = packets(&start(&mut d));
        assert!(at.elapsed() >= Duration::from_millis(100));
        assert_eq!(values(&got), [0.0f32; 0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_time_limit_ends_a_capture_that_gets_packets() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(P127.to_vec());
            l.byte_time = Duration::from_millis(1);
        });
        let mut d = device(&line);
        d.set(&Key::LimitMsec, &Value::U64(100), None).unwrap();
        let got = packets(&start(&mut d));
        assert_ne!(values(&got), [0.0f32; 0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn silence_says_so_at_its_bound_from_either_side() {
    let t = Instant::now();
    let mut s = Silence::new(Duration::from_secs(10), t);
    assert!(!s.over(t));
    assert!(!s.over(t + Duration::from_millis(9999)));
    assert!(s.over(t + Duration::from_secs(10)));
    s.heard(t + Duration::from_secs(9));
    assert!(!s.over(t + Duration::from_secs(10)));
    assert!(!s.over(t + Duration::from_millis(18_999)));
    assert!(s.over(t + Duration::from_secs(19)));
    // A clock that is behind is no silence, and no panic.
    assert!(!s.over(t));
}

#[test]
fn a_silent_scale_at_a_sample_limit_fails_and_says_what_happened() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device_with_bound(&line, Duration::from_millis(100));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let at = Instant::now();
        let got = packets(&start(&mut d));
        assert!(at.elapsed() >= Duration::from_millis(100));
        assert!(matches!(got[1..], [Packet::Failed(_)]), "the header and the failure: {got:?}");
        assert!(
            failure(&got).is_some_and(|w| w.contains("no usable packet from the scale for 0 s")),
            "{got:?}"
        );
    });
}

#[test]
fn bytes_that_make_no_packet_do_not_hold_off_the_bound() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(vec![b'?'; 20]);
            l.byte_time = Duration::from_millis(1);
        });
        let mut d = device_with_bound(&line, Duration::from_millis(100));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let got = packets(&start(&mut d));
        assert!(failure(&got).is_some(), "{got:?}");
    });
}

#[test]
fn a_late_packet_is_on_time_inside_the_bound() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.gap_left = 3;
            l.queued.extend(P127);
        });
        // Wide: the three empty reads take 150 ms, or a few times that.
        let mut d = device_with_bound(&line, BOUND);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let got = packets(&start(&mut d));
        assert_eq!(values(&got), [127.2]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_packet_ready_at_the_bound_wins_over_it() {
    bounded(|| {
        // The first read has it, and comes before any look at the clock, however
        // short the bound.
        let line = Shared::default().with(|l| l.queued.extend(P127));
        let mut d = device_with_bound(&line, Duration::ZERO);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let got = packets(&start(&mut d));
        assert_eq!(values(&got), [127.2]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_slow_client_is_no_silence() {
    bounded(|| {
        // Each packet's send waits on a queue the test empties slowly, longer than
        // the bound, and no time passes on the line.
        let bound = Duration::from_millis(100);
        let line = Shared::default().with(|l| l.stream = Some(P127.to_vec()));
        let mut d = device_with_bound(&line, bound);
        d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let mut got = Vec::new();
        loop {
            std::thread::sleep(bound * 2);
            let p = rx.recv_timeout(BOUND).expect("no End in time").1;
            if p == Packet::End {
                break;
            }
            got.push(p);
        }
        assert_eq!(values(&got).len(), 4);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn with_no_limit_silence_goes_on_until_a_stop() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device_with_bound(&line, Duration::ZERO);
        let rx = start(&mut d);
        line.wait_for_reads(5);
        d.stop().unwrap();
        let got = packets(&rx);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_time_limit_keeps_silence_past_the_bound() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device_with_bound(&line, Duration::from_millis(10));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        d.set(&Key::LimitMsec, &Value::U64(150), None).unwrap();
        let at = Instant::now();
        let got = packets(&start(&mut d));
        assert!(at.elapsed() >= Duration::from_millis(150));
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_port_that_hangs_up_ends_the_capture() {
    bounded(|| {
        let line = Shared::default().with(|l| l.read_fails = true);
        // Far from the silence's bound, which must not be what ends it.
        let mut d = device_with_bound(&line, BOUND * 10);
        d.set(&Key::LimitSamples, &Value::U64(5), None).unwrap();
        let got = packets(&start(&mut d));
        assert!(matches!(got[1..], [Packet::Failed(_)]), "the header and the failure: {got:?}");
        // No limit at all still ends.
        let line = Shared::default().with(|l| l.read_fails = true);
        let mut d = device(&line);
        let got = packets(&start(&mut d));
        assert!(failure(&got).is_some());
    });
}

#[test]
fn a_write_that_fails_refuses_the_start_and_keeps_the_port() {
    bounded(|| {
        let line = Shared::default().with(|l| l.write_fails = true);
        let mut d = device(&line);
        let (tx, rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
        assert!(rx.try_recv().is_err(), "no header");
        line.line().write_fails = false;
        let (tx, _rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_ok(), "the port is still there");
        d.close().unwrap();
    });
}

#[test]
fn a_stop_ends_a_capture_while_bytes_flow_and_writes_nothing() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(P127.to_vec());
            l.byte_time = Duration::from_millis(1);
        });
        let mut d = device(&line);
        let rx = start(&mut d);
        line.wait_for_reads(5);
        d.stop().unwrap();
        let got = packets(&rx);
        assert_eq!(failure(&got), None);
        assert_eq!(line.line().writes.len(), 1, "stop has nothing to send");
    });
}

#[test]
fn a_stop_ends_a_capture_on_noise() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(vec![b'?'; 40]);
            l.byte_time = Duration::from_millis(1);
        });
        let mut d = device(&line);
        let rx = start(&mut d);
        line.wait_for_reads(5);
        d.stop().unwrap();
        assert_eq!(failure(&packets(&rx)), None);
    });
}

#[test]
fn a_stop_does_not_join_and_a_close_does() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(P127.to_vec()));
        let mut d = device(&line);
        // A queue of one left unread: the thread blocks on its second packet.
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Past the look at the flag, it has a packet to send and no room.
        line.wait_for_reads(1);
        d.stop().unwrap();
        d.close().unwrap();
        assert!(d.worker.is_none());
        // All the thread's senders are gone with it.
        while rx.recv_timeout(BOUND).is_ok() {}
        line.line().closed = true;
        assert_eq!(line.line().writes_after_close, 0);
    });
}

#[test]
fn a_consumer_that_left_ends_the_capture() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(P127.to_vec()));
        let mut d = device(&line);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        drop(rx);
        // The worker gives the port back as it goes.
        let deadline = Instant::now() + BOUND;
        while d.port.lock().unwrap().is_none() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
    });
}

#[test]
fn a_start_during_a_capture_is_busy() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(&line);
        let rx = start(&mut d);
        let (tx, _rx2) = sync_channel(1);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.stop().unwrap();
        packets(&rx);
    });
}

#[test]
fn a_start_after_an_end_stuck_on_a_full_queue_does_not_hang() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(&line);
        d.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
        // The header fills the queue, so the End cannot go.
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The port is back before the End.
        let deadline = Instant::now() + BOUND;
        while d.port.lock().unwrap().is_none() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        // A new capture on a new queue, with the first queue left unread.
        let (tx2, rx2) = sync_channel(1024);
        d.start(PacketSink::new(tx2, 0)).unwrap();
        // The start told the old thread and joined it: it is gone, with its End
        // given up, and the old sink with it.
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert_eq!(
            rx.recv_timeout(BOUND).err(),
            Some(std::sync::mpsc::RecvTimeoutError::Disconnected)
        );
        packets(&rx2);
    });
}

#[test]
fn a_second_capture_does_not_read_what_came_since_the_first() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            // The first capture's sample, and a head with no end.
            l.queued.extend(P127);
            l.queued.extend(b"+00001");
        });
        let mut d = device(&line);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        assert_eq!(values(&packets(&start(&mut d))), [127.2]);
        assert_eq!(line.line().purges, 1, "the open's");
        // Old by now, then a tail that would finish the first's head.
        line.line().queued.extend(b"+00099.00 G S\r\n");
        line.line().on_write.push_back(b".00 G S\r\n+00002.00 G S\r\n".to_vec());
        assert_eq!(values(&packets(&start(&mut d))), [2.0]);
        assert_eq!(line.line().purges, 2);
        assert_eq!(line.line().writes, [O1, O1]);
    });
}

#[test]
fn a_reopen_does_not_flush_twice() {
    bounded(|| {
        let line = Shared::default().with(|l| l.queued.extend(P127));
        let mut d = device(&line);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        // The first capture of an open keeps what comes after the open's flush.
        assert_eq!(values(&packets(&start(&mut d))), [127.2]);
        assert_eq!(line.line().purges, 1);
    });
}

#[test]
fn no_start_when_not_open() {
    let line = Shared::default();
    let mut d = KernScaleDevice::from_port(line.port());
    let (tx, _rx) = sync_channel(4);
    assert!(d.start(PacketSink::new(tx, 0)).is_err());
    assert_eq!(line.line().writes.len(), 0);
}

#[test]
fn a_limit_set_during_a_capture_takes_effect() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(P127.to_vec());
            l.byte_time = Duration::from_millis(1);
        });
        let mut d = device(&line);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        assert_eq!(failure(&packets(&rx)), None);
    });
}
