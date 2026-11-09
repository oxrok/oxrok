// Counterpart in sigrok:
//   libsigrok/src/hardware/asix-omega-rtm-cli/api.c,
//   libsigrok/src/hardware/asix-omega-rtm-cli/protocol.c,
//   Copyright (C) 2021 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Tests of the `asix-omega-rtm-cli` driver: the pure parts, the capture over a
//! fake stream, and (on Unix) the child process through `sh`.

use super::*;
use crate::core::Packet;
use child::{Ended, Exit, Got, Stream};
use protocol::{FEED_QUEUE_DEPTH, Fed, Queue, Silence, Verdict, parse_conn, parse_version};
use std::collections::VecDeque;
use std::sync::Mutex;
use std::sync::mpsc;

fn chunk(stamp: u16, a: u16, b: u16) -> [u8; CHUNK] {
    let mut c = [0u8; CHUNK];
    c[0..2].copy_from_slice(&stamp.to_le_bytes());
    c[2..4].copy_from_slice(&a.to_le_bytes());
    c[4..6].copy_from_slice(&b.to_le_bytes());
    c
}

/// The first chunk of the model's stream: its time and first sample mean nothing.
const FIRST: [u8; CHUNK] = [0x5a, 0x5a, 0xad, 0xde, 0, 0];

fn stream(chunks: &[[u8; CHUNK]]) -> Vec<u8> {
    chunks.concat()
}

/// The model's first chunks after the first: 1 3, 2 6, then 6 twice and 7 5.
fn sample_stream() -> Vec<u8> {
    stream(&[chunk(1, 1, 3), chunk(1, 2, 6), chunk(2, 7, 5), chunk(40, 4, 12), chunk(1, 13, 15)])
}

/// The samples of a decode, as 16 bit values.
fn decode(first: [u8; CHUNK], limit: u64, pieces: &[&[u8]]) -> (Vec<u16>, Fed) {
    let mut d = Decoder::new(first, limit);
    let mut q = Queue::default();
    let mut out = Vec::new();
    let mut fed = Fed::More;
    {
        let mut send = |data: Vec<u8>| {
            out.extend(data.chunks(2).map(|s| u16::from_le_bytes([s[0], s[1]])));
            true
        };
        for p in pieces {
            fed = d.feed(p, &mut q, &mut send);
            if fed != Fed::More {
                break;
            }
        }
        q.flush(&mut send);
    }
    (out, fed)
}

#[test]
fn conn_as_c_reads_it() {
    assert_eq!(parse_conn(None), Some(protocol::Conn::default()));
    assert_eq!(parse_conn(Some("")), Some(protocol::Conn::default()));
    assert_eq!(parse_conn(Some("031234")), None);
    let c = |s: &str| parse_conn(Some(s)).unwrap();
    assert_eq!(c("sn="), protocol::Conn { serial: None, conn: Some("sn=".into()) });
    assert_eq!(c("sn=1234").serial.as_deref(), Some("031234"));
    assert_eq!(c("sn=031234").serial.as_deref(), Some("031234"));
    assert_eq!(c("sn=A6031234").serial.as_deref(), Some("A6031234"));
    // Other lengths: an error, and the device without -serial, conn= kept.
    assert_eq!(c("sn=12345"), protocol::Conn { serial: None, conn: Some("sn=12345".into()) });
    assert_eq!(c("sn=123456789").serial, None);
    // C checks no digit: the program says what it makes of them.
    assert_eq!(c("sn=zz1234").serial.as_deref(), Some("zz1234"));
}

#[test]
fn the_version_as_c_finds_it() {
    let v = |b: &[u8]| parse_version(b);
    let banner = b"omegartmcli.exe Omega Real-Time Mode\r\nVersion 2016-12-14\r\n\
        Copyright (c) 1991-2016 ASIX s.r.o.\r\nEmail: support@asix.net\r\n";
    assert_eq!(v(banner).as_deref(), Some("2016-12-14"));
    assert_eq!(v(b"Version 3.0\rjunk\n").as_deref(), Some("3.0"));
    assert_eq!(v(b"xVersion 1 2 3").as_deref(), Some("1 2 3"));
    assert_eq!(v(b"Version 1\nVersion 2\n").as_deref(), Some("1"));
    assert_eq!(v(b"Version 1\0junk").as_deref(), Some("1"));
    assert_eq!(v(b"version 2016\r\n"), None);
    assert_eq!(v(b"Version \r\n"), None);
    assert_eq!(v(b"Version"), None);
    assert_eq!(v(b""), None);
    // A NUL ends C's string, and what is after it isn't looked at.
    assert_eq!(v(b"x\0Version 1"), None);
}

#[test]
fn chunks_repeat_the_last_sample_and_add_two() {
    let (s, fed) = decode(FIRST, 0, &[&sample_stream()]);
    assert_eq!(fed, Fed::More);
    let mut want = vec![1, 3, 2, 6, 6, 6, 7, 5];
    want.extend(std::iter::repeat_n(5, 78));
    want.extend([4, 12, 13, 15]);
    assert_eq!(s, want);
}

/// C's check gives one sample more when one is left (`protocol.c`): here the
/// limit is exact at every count.
#[test]
fn a_limit_is_exact_at_every_count() {
    let (all, _) = decode(FIRST, 0, &[&sample_stream()]);
    for limit in 1..=all.len() as u64 {
        let (s, fed) = decode(FIRST, limit, &[&sample_stream()]);
        assert_eq!(s, all[..limit as usize], "limit {limit}");
        assert_eq!(fed, Fed::Done, "limit {limit}");
    }
    let (s, fed) = decode(FIRST, all.len() as u64 + 1, &[&sample_stream()]);
    assert_eq!((s.len(), fed), (all.len(), Fed::More));
}

#[test]
fn a_stream_cut_anywhere_decodes_alike() {
    let data = sample_stream();
    let (all, _) = decode(FIRST, 0, &[&data]);
    for at in 0..=data.len() {
        let (s, _) = decode(FIRST, 0, &[&data[..at], &data[at..]]);
        assert_eq!(s, all, "cut at {at}");
    }
    let bytes: Vec<&[u8]> = data.chunks(1).collect();
    assert_eq!(decode(FIRST, 0, &bytes).0, all);
    let sevens: Vec<&[u8]> = data.chunks(7).collect();
    assert_eq!(decode(FIRST, 0, &sevens).0, all);
    // A chunk not all here waits, and says so at the end.
    let mut d = Decoder::new(FIRST, 0);
    let mut q = Queue::default();
    d.feed(&data[..9], &mut q, &mut |_| true);
    assert_eq!(d.leftover(), 3);
}

#[test]
fn the_first_chunk_gives_the_value_runs_start_from() {
    // A run first: the first chunk's second sample repeats.
    let (s, _) = decode(chunk(9, 0xdead, 0x00aa), 0, &[&chunk(3, 1, 2)]);
    assert_eq!(s, [0xaa, 0xaa, 0xaa, 0xaa, 1, 2]);
}

/// The program sends 1 to 32768; C takes 0 as 1, and so does this.
#[test]
fn a_time_of_zero_is_one() {
    let mut d = Decoder::new(FIRST, 0);
    let mut q = Queue::default();
    let mut n = 0;
    d.feed(&stream(&[chunk(0, 1, 2), chunk(1, 3, 4)]), &mut q, &mut |v| {
        n += v.len() / 2;
        true
    });
    q.flush(&mut |v| {
        n += v.len() / 2;
        true
    });
    assert_eq!((n, d.zero_stamps), (4, 1));
}

#[test]
fn packets_hold_at_most_a_queue() {
    // Five chunks of 32768 ticks: 65534 repeats and two samples each.
    let data = stream(&[chunk(32768, 1, 1); 5]);
    let mut d = Decoder::new(FIRST, 0);
    let mut q = Queue::default();
    let mut sizes = Vec::new();
    d.feed(&data, &mut q, &mut |v| {
        sizes.push(v.len() / 2);
        true
    });
    q.flush(&mut |v| {
        sizes.push(v.len() / 2);
        true
    });
    assert_eq!(sizes.iter().sum::<usize>(), 5 * 65536);
    assert!(sizes.iter().all(|s| *s <= FEED_QUEUE_DEPTH), "{sizes:?}");
    assert_eq!(sizes[0], FEED_QUEUE_DEPTH);
}

#[test]
fn a_sink_gone_ends_the_decode() {
    let data = stream(&[chunk(32768, 1, 1); 5]);
    let mut d = Decoder::new(FIRST, 0);
    let mut q = Queue::default();
    assert_eq!(d.feed(&data, &mut q, &mut |_| false), Fed::Gone);
}

#[test]
fn silence_warns_fails_and_says_when_back() {
    let t0 = Instant::now();
    let s = |ms| t0 + Duration::from_millis(ms);
    let mut q = Silence::with(t0, Duration::from_millis(100), Duration::from_millis(1000));
    assert_eq!(q.empty(s(99)), Verdict::Wait);
    assert!(matches!(q.empty(s(100)), Verdict::Warn(t) if t.contains("No data")));
    // Once.
    assert_eq!(q.empty(s(500)), Verdict::Wait);
    assert!(q.got(s(600)).is_some_and(|t| t.contains("again, after 0.6 s")));
    assert_eq!(q.got(s(601)), None);
    assert_eq!(q.empty(s(700)), Verdict::Wait);
    assert!(matches!(q.empty(s(1601)), Verdict::Fail(t) if t.contains("No data")));
    // The real bounds: a burst every 100 ms is far inside them.
    let mut real = Silence::new(t0);
    assert_eq!(real.empty(s(4999)), Verdict::Wait);
    assert!(matches!(real.empty(s(5000)), Verdict::Warn(_)));
    assert_eq!(real.empty(s(29_999)), Verdict::Wait);
    assert!(matches!(real.empty(s(30_000)), Verdict::Fail(_)));
}

/// The bounds the docs and the emulator cases count on.
#[test]
fn the_bounds_are_thirty_seconds() {
    assert_eq!(START_WAIT, Duration::from_secs(30));
    assert_eq!(VERSION_WAIT, Duration::from_secs(30));
    assert_eq!(protocol::SILENCE_WARN, Duration::from_secs(5));
    assert_eq!(protocol::SILENCE_LIMIT, Duration::from_secs(30));
    assert_eq!(child::CLOSE_GRACE, Duration::from_secs(2));
}

#[test]
fn a_run_fills_as_one_sample_at_a_time_would() {
    let mut q = Queue::default();
    let mut got = Vec::new();
    for (sample, n) in [([1u8, 2u8], 0u64), ([3, 4], 1), ([5, 6], 2), ([7, 8], 65534), ([9, 10], 3)]
    {
        assert!(q.push(sample, n, &mut |v| {
            got.extend(v);
            true
        }));
    }
    q.flush(&mut |v| {
        got.extend(v);
        true
    });
    let mut want = Vec::new();
    for (sample, n) in [([3u8, 4u8], 1usize), ([5, 6], 2), ([7, 8], 65534), ([9, 10], 3)] {
        for _ in 0..n {
            want.extend_from_slice(&sample);
        }
    }
    assert_eq!(got, want);
}

/// A program's stream from a script, in a test.
struct Fake {
    script: VecDeque<Got>,
    exit: Option<Exit>,
    closed: Arc<AtomicBool>,
    /// What comes when the script is out: data for ever, or nothing.
    endless: Option<Vec<u8>>,
}

impl Fake {
    fn new(script: Vec<Got>) -> (Fake, Arc<AtomicBool>) {
        let closed = Arc::new(AtomicBool::new(false));
        let fake = Fake {
            script: script.into(),
            exit: Some(Exit { success: true, text: "exit status: 0".into() }),
            closed: Arc::clone(&closed),
            endless: None,
        };
        (fake, closed)
    }
}

impl Stream for Fake {
    fn read(&mut self, timeout: Duration) -> Got {
        if let Some(g) = self.script.pop_front() {
            // A scripted nothing takes the read's whole wait, as a pipe's would.
            if g == Got::Nothing {
                std::thread::sleep(timeout);
            }
            return g;
        }
        if let Some(d) = &self.endless {
            return Got::Data(d.clone());
        }
        std::thread::sleep(timeout.min(Duration::from_millis(1)));
        Got::Nothing
    }

    fn exited(&mut self, _timeout: Duration) -> Option<Exit> {
        self.exit.clone()
    }

    fn close(self: Box<Self>) -> Ended {
        self.closed.store(true, Ordering::SeqCst);
        Ended::Exited(Exit { success: true, text: "exit status: 0".into() })
    }
}

fn data(chunks: &[[u8; CHUNK]]) -> Got {
    Got::Data(stream(chunks))
}

/// What a capture sent: its samples, and its failure if it failed.
#[derive(Debug, Default)]
struct Sent {
    header: bool,
    samples: Vec<u16>,
    failed: Option<String>,
}

fn collect(rx: &mpsc::Receiver<(usize, Packet)>) -> Sent {
    let mut sent = Sent::default();
    loop {
        let (_, p) = rx.recv_timeout(Duration::from_secs(20)).expect("a capture that ends");
        match p {
            Packet::Header { .. } => sent.header = true,
            Packet::Logic(l) => {
                assert!(sent.header, "logic before the header");
                assert_eq!(l.unit_size, 2);
                sent.samples.extend(l.data.chunks(2).map(|s| u16::from_le_bytes([s[0], s[1]])));
            }
            Packet::Failed(t) => sent.failed = Some(t),
            Packet::End => return sent,
            other => panic!("{other:?}"),
        }
    }
}

fn capture(limit_samples: u64, limit: Option<Duration>, silence: Silence) -> Capture {
    Capture { decoder: Decoder::new(FIRST, limit_samples), limit, began: Instant::now(), silence }
}

/// Runs a capture on a thread, as `start` does, and collects what it sent.
fn run(c: Capture, fake: Fake, rest: Vec<u8>) -> Sent {
    let (tx, rx) = mpsc::sync_channel(64);
    let stop = Arc::new(AtomicBool::new(false));
    let t =
        std::thread::spawn(move || c.run(Box::new(fake), &rest, &PacketSink::new(tx, 0), &stop));
    let sent = collect(&rx);
    t.join().unwrap();
    sent
}

fn quiet() -> Silence {
    Silence::new(Instant::now())
}

#[test]
fn a_capture_ends_at_its_limit_and_closes_the_program() {
    let (fake, closed) = Fake::new(vec![Got::Nothing, Got::Data(sample_stream())]);
    let sent = run(capture(5, None, quiet()), fake, Vec::new());
    assert_eq!(sent.samples, [1, 3, 2, 6, 6]);
    assert_eq!(sent.failed, None);
    assert!(closed.load(Ordering::SeqCst));
}

#[test]
fn what_came_with_the_first_chunk_goes_first() {
    let (fake, _) = Fake::new(vec![data(&[chunk(1, 2, 6)])]);
    let sent = run(capture(4, None, quiet()), fake, chunk(1, 1, 3).to_vec());
    assert_eq!(sent.samples, [1, 3, 2, 6]);
}

#[test]
fn the_programs_own_end_ends_the_capture_well() {
    let (fake, closed) = Fake::new(vec![data(&[chunk(1, 1, 3)]), Got::End(None)]);
    let sent = run(capture(1000, None, quiet()), fake, Vec::new());
    assert_eq!((sent.samples, sent.failed), (vec![1, 3], None));
    assert!(closed.load(Ordering::SeqCst));
}

#[test]
fn an_end_with_an_error_fails_the_capture() {
    let (mut fake, _) = Fake::new(vec![data(&[chunk(1, 1, 3)]), Got::End(None)]);
    fake.exit = Some(Exit { success: false, text: "exit status: 1".into() });
    let sent = run(capture(1000, None, quiet()), fake, Vec::new());
    assert_eq!(sent.samples, [1, 3]);
    let failed = sent.failed.expect("failed");
    assert!(failed.contains("ended the stream (exit status: 1), after 2 samples"), "{failed}");

    // Its stdout gone with the program still there.
    let (mut fake, _) = Fake::new(vec![Got::End(None)]);
    fake.exit = None;
    let failed = run(capture(1000, None, quiet()), fake, Vec::new()).failed.expect("failed");
    assert!(failed.contains("its output closed, and it still runs"), "{failed}");

    // A read error.
    let (fake, _) = Fake::new(vec![Got::End(Some("Input/output error".into()))]);
    let failed = run(capture(1000, None, quiet()), fake, Vec::new()).failed.expect("failed");
    assert!(failed.contains("a read failed: Input/output error"), "{failed}");
}

#[test]
fn an_end_inside_a_chunk_drops_its_bytes() {
    let mut cut = stream(&[chunk(1, 1, 3), chunk(1, 2, 6)]);
    cut.truncate(9);
    let (fake, _) = Fake::new(vec![Got::Data(cut), Got::End(None)]);
    let sent = run(capture(1000, None, quiet()), fake, Vec::new());
    assert_eq!((sent.samples, sent.failed), (vec![1, 3], None));
}

#[test]
fn a_silent_program_fails_the_capture_at_the_bound() {
    let (fake, closed) = Fake::new(vec![data(&[chunk(1, 1, 3)])]);
    let silence =
        Silence::with(Instant::now(), Duration::from_millis(20), Duration::from_millis(60));
    let sent = run(capture(1000, None, silence), fake, Vec::new());
    assert_eq!(sent.samples, [1, 3]);
    assert!(sent.failed.expect("failed").contains("No data from the vendor application"));
    assert!(closed.load(Ordering::SeqCst));
}

/// The program's pace: a burst of what it decoded every 100 ms, ten of the
/// capture's reads apart. An idle input is a chunk of 32768 ticks each 327.68 us,
/// so 305 of them a burst.
#[test]
fn bursts_at_the_programs_pace() {
    let burst = stream(&[chunk(32768, 0, 0); 305]);
    let mut script = Vec::new();
    for _ in 0..4 {
        script.extend(std::iter::repeat_with(|| Got::Nothing).take(10));
        script.push(Got::Data(burst.clone()));
    }
    let (fake, _) = Fake::new(script);
    let limit = 3 * 305 * 65536 + 1;
    let sent = run(capture(limit, None, quiet()), fake, Vec::new());
    assert_eq!(sent.failed, None);
    assert_eq!(sent.samples.len() as u64, limit);
}

#[test]
fn a_time_limit_ends_a_stream_that_goes_on() {
    let (mut fake, closed) = Fake::new(Vec::new());
    fake.endless = Some(stream(&[chunk(1, 1, 3); 100]));
    let began = Instant::now();
    let sent = run(capture(0, Some(Duration::from_millis(50)), quiet()), fake, Vec::new());
    assert!(began.elapsed() >= Duration::from_millis(50));
    assert_eq!(sent.failed, None);
    assert!(!sent.samples.is_empty() && sent.samples.len().is_multiple_of(2));
    assert!(closed.load(Ordering::SeqCst));
}

#[test]
fn a_stop_ends_a_capture_between_reads() {
    let (mut fake, closed) = Fake::new(Vec::new());
    fake.endless = Some(stream(&[chunk(1, 1, 3); 10]));
    let (tx, rx) = mpsc::sync_channel(4);
    let stop = Arc::new(AtomicBool::new(false));
    let stop2 = Arc::clone(&stop);
    let c = capture(0, None, quiet());
    let t = std::thread::spawn(move || c.run(Box::new(fake), &[], &PacketSink::new(tx, 0), &stop2));
    // A full queue and nobody reading: the stop must still get through.
    let began = Instant::now();
    while !t.is_finished() && began.elapsed() < Duration::from_millis(100) {
        std::thread::sleep(Duration::from_millis(5));
    }
    stop.store(true, Ordering::SeqCst);
    let began = Instant::now();
    while !t.is_finished() {
        assert!(began.elapsed() < Duration::from_secs(10), "a stop that never lands");
        std::thread::sleep(Duration::from_millis(5));
    }
    t.join().unwrap();
    assert!(closed.load(Ordering::SeqCst));
    drop(rx);
}

/// A stop with a consumer that keeps up: the stop comes between reads, never
/// inside a blocked send.
#[test]
fn a_stop_ends_a_capture_that_keeps_flowing() {
    let (mut fake, closed) = Fake::new(Vec::new());
    fake.endless = Some(stream(&[chunk(1, 1, 3); 10]));
    // Room enough that a send never waits, so only the stop can end it.
    let (tx, rx) = mpsc::sync_channel(1 << 16);
    let drain = std::thread::spawn(move || while rx.recv().is_ok() {});
    let stop = Arc::new(AtomicBool::new(false));
    let stop2 = Arc::clone(&stop);
    let c = capture(0, None, quiet());
    let t = std::thread::spawn(move || c.run(Box::new(fake), &[], &PacketSink::new(tx, 0), &stop2));
    std::thread::sleep(Duration::from_millis(50));
    stop.store(true, Ordering::SeqCst);
    let began = Instant::now();
    while !t.is_finished() {
        assert!(began.elapsed() < Duration::from_secs(10), "a stop that never lands");
        std::thread::sleep(Duration::from_millis(5));
    }
    t.join().unwrap();
    drain.join().unwrap();
    assert!(closed.load(Ordering::SeqCst));
}

#[test]
fn a_consumer_gone_ends_the_capture() {
    let (mut fake, closed) = Fake::new(Vec::new());
    fake.endless = Some(stream(&[chunk(1, 1, 3); 10]));
    let (tx, rx) = mpsc::sync_channel(4);
    drop(rx);
    let stop = AtomicBool::new(false);
    let c = capture(0, None, quiet());
    let t = std::thread::spawn(move || c.run(Box::new(fake), &[], &PacketSink::new(tx, 0), &stop));
    let began = Instant::now();
    while !t.is_finished() {
        assert!(began.elapsed() < Duration::from_secs(10), "a capture that never ends");
        std::thread::sleep(Duration::from_millis(5));
    }
    t.join().unwrap();
    assert!(closed.load(Ordering::SeqCst));
}

/// A device whose program is a list of fakes, one per start, and the arguments
/// each start gave.
fn device(
    conn: Option<&str>,
    fakes: Vec<Fake>,
) -> (AsixOmegaRtmCliDevice, Arc<Mutex<Vec<Vec<String>>>>) {
    let fakes = Arc::new(Mutex::new(VecDeque::from(fakes)));
    let args = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::clone(&args);
    let opener: Opener = Arc::new(move |a: &[&str]| {
        seen.lock().unwrap().push(a.iter().map(|s| (*s).to_owned()).collect());
        match fakes.lock().unwrap().pop_front() {
            Some(f) => Ok(Box::new(f) as Box<dyn Stream>),
            None => Err(std::io::Error::from(std::io::ErrorKind::NotFound)),
        }
    });
    let spec = parse_conn(conn).unwrap();
    (AsixOmegaRtmCliDevice::new("2016-12-14".into(), spec, None, opener), args)
}

fn start_and_collect(d: &mut AsixOmegaRtmCliDevice) -> Result<Sent> {
    let (tx, rx) = mpsc::sync_channel(64);
    d.start(PacketSink::new(tx, 0))?;
    let sent = collect(&rx);
    d.stop()?;
    Ok(sent)
}

#[test]
fn a_start_takes_its_first_chunk_in_pieces() {
    let (fake, closed) = Fake::new(vec![
        Got::Data(FIRST[..2].to_vec()),
        Got::Nothing,
        Got::Data(FIRST[2..5].to_vec()),
        Got::Data([&FIRST[5..], &chunk(1, 1, 3)[..]].concat()),
        data(&[chunk(1, 2, 6)]),
    ]);
    let (mut d, args) = device(Some("sn=1234"), vec![fake]);
    d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
    let sent = start_and_collect(&mut d).unwrap();
    assert_eq!(sent.samples, [1, 3, 2, 6]);
    assert_eq!(*args.lock().unwrap(), [["-bin", "-serial", "031234"]]);
    assert!(closed.load(Ordering::SeqCst));
}

#[test]
fn a_start_fails_on_an_end_before_the_first_chunk() {
    let (mut fake, closed) = Fake::new(vec![Got::Data(vec![1, 2, 3]), Got::End(None)]);
    fake.exit = Some(Exit { success: false, text: "exit status: 1".into() });
    let (mut d, args) = device(None, vec![fake]);
    let sent = start_and_collect(&mut d).unwrap();
    assert!(!sent.header, "a header before the first chunk");
    let e = sent.failed.expect("failed");
    assert!(e.contains("ended before its first chunk, at 3 of 6 bytes (exit status: 1)"), "{e}");
    assert!(closed.load(Ordering::SeqCst));
    assert_eq!(*args.lock().unwrap(), [["-bin"]]);
}

#[test]
fn a_start_fails_on_a_silent_program_at_the_bound() {
    let (fake, closed) = Fake::new(Vec::new());
    let (mut d, _) = device(None, vec![fake]);
    d.set_start_wait(Duration::from_millis(30));
    let sent = start_and_collect(&mut d).unwrap();
    assert!(sent.failed.expect("failed").contains("No first chunk"));
    assert!(closed.load(Ordering::SeqCst));
}

/// A stop while the program is still on its way to its first chunk: C's start
/// blocks there, here the stop gets through at once.
#[test]
fn a_stop_reaches_the_wait_for_the_first_chunk() {
    let (fake, closed) = Fake::new(Vec::new());
    let (mut d, _) = device(None, vec![fake]);
    let (tx, rx) = mpsc::sync_channel(64);
    d.start(PacketSink::new(tx, 0)).unwrap();
    std::thread::sleep(Duration::from_millis(30));
    let began = Instant::now();
    d.stop().unwrap();
    assert!(began.elapsed() < Duration::from_secs(5));
    assert!(closed.load(Ordering::SeqCst));
    let sent = collect(&rx);
    assert!(!sent.header && sent.failed.is_none(), "{sent:?}");
}

/// The time limit starts once the first chunk is in, as C's
/// `sr_sw_limits_acquisition_start` after `omega_rtm_cli_open`: a program slower
/// to start than the limit still gives its samples.
#[test]
fn the_time_limit_starts_after_the_first_chunk() {
    let mut script: Vec<Got> = std::iter::repeat_with(|| Got::Nothing).take(15).collect();
    script.push(Got::Data(FIRST.to_vec()));
    script.push(data(&[chunk(1, 1, 3)]));
    let (fake, _) = Fake::new(script);
    let (mut d, _) = device(None, vec![fake]);
    d.set(&Key::LimitMsec, &Value::U64(100), None).unwrap();
    let sent = start_and_collect(&mut d).unwrap();
    assert_eq!((sent.samples, sent.failed), (vec![1, 3], None));
}

#[test]
fn a_start_fails_when_the_program_cannot_run() {
    let (mut d, _) = device(None, Vec::new());
    let (tx, _rx) = mpsc::sync_channel(64);
    assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Io(_))));
}

#[test]
fn a_second_start_waits_for_the_first_to_end() {
    let (mut long, _) = Fake::new(vec![Got::Data(FIRST.to_vec())]);
    long.endless = Some(stream(&[chunk(1, 1, 3); 10]));
    let (again, _) = Fake::new(vec![Got::Data([&FIRST[..], &chunk(1, 1, 3)[..]].concat())]);
    let (mut d, _) = device(None, vec![long, again]);
    let (tx, rx) = mpsc::sync_channel(4);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let (tx2, _rx2) = mpsc::sync_channel(4);
    assert!(matches!(d.start(PacketSink::new(tx2, 0)), Err(Error::Busy)));
    d.stop().unwrap();
    drop(rx);
    d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    assert_eq!(start_and_collect(&mut d).unwrap().samples, [1, 3]);
}

#[test]
fn options_as_c_has_them() {
    let (d, _) = device(None, Vec::new());
    let keys: Vec<(Key, Access)> =
        d.options(None).unwrap().into_iter().map(|o| (o.key, o.access)).collect();
    assert_eq!(
        keys,
        [
            (Key::LimitMsec, Access::GET_SET),
            (Key::LimitSamples, Access::GET_SET),
            (Key::Conn, Access::NONE),
            (Key::Samplerate, Access { get: true, set: false, list: true }),
        ]
    );
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(200_000_000));
    assert_eq!(d.list(&Key::Samplerate, None).unwrap(), ValueList::U64(vec![200_000_000]));
    assert!(d.get(&Key::Conn, None).is_err());
    assert!(d.options(Some("x")).is_err());
    let (mut d, _) = device(Some("sn=1234"), Vec::new());
    assert_eq!(d.options(None).unwrap()[2].access, Access::GET);
    assert_eq!(d.get(&Key::Conn, None).unwrap(), Value::Str("sn=1234".into()));
    assert_eq!(d.info().serial, "031234");
    d.set(&Key::LimitMsec, &Value::U64(250), None).unwrap();
    assert_eq!(d.get(&Key::LimitMsec, None).unwrap(), Value::U64(250));
    assert!(d.set(&Key::Samplerate, &Value::U64(1), None).is_err());
}

#[test]
fn probe_names_name_the_channels() {
    let spec = parse_conn(None).unwrap();
    let opener: Opener =
        Arc::new(|_: &[&str]| Err(std::io::Error::from(std::io::ErrorKind::NotFound)));
    let d = AsixOmegaRtmCliDevice::new("1".into(), spec.clone(), Some("a,b"), Arc::clone(&opener));
    let names: Vec<&str> = d.channels().iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names.len(), 16);
    assert_eq!(&names[..3], ["a", "b", "3"]);
    let d = AsixOmegaRtmCliDevice::new("1".into(), spec, Some("-uart"), opener);
    assert_eq!(d.channels().len(), 2);
}

#[test]
fn a_banner_makes_a_device_or_none() {
    let exe = OsString::from("omegartmcli");
    let spec = || parse_conn(None).unwrap();
    let d = device_from(b"x\r\nVersion 2016-12-14\r\n", spec(), None, exe.clone()).unwrap();
    assert_eq!(d.info().version, "2016-12-14");
    assert_eq!(d.info().vendor, "ASIX");
    assert_eq!(d.info().model, "OMEGA RTM CLI");
    assert!(device_from(b"", spec(), None, exe.clone()).is_none());
    assert!(device_from(b"\0Version 1", spec(), None, exe.clone()).is_none());
    assert!(device_from(b"no version", spec(), None, exe).is_none());
}

#[cfg(unix)]
mod process {
    use super::*;
    use child::{Child, VersionError, run_version};
    use std::os::unix::fs::PermissionsExt as _;

    fn sh(script: &str) -> Child {
        Child::spawn(&OsString::from("/bin/sh"), &["-c", script]).unwrap()
    }

    /// Whether process `pid` is gone, reaped: no zombie left.
    fn gone(pid: u32) -> bool {
        !std::path::Path::new(&format!("/proc/{pid}")).exists()
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn the_program_runs_in_its_own_group() {
        let c = sh("exec cat >/dev/null");
        let pid = c.id();
        let stat = std::fs::read_to_string(format!("/proc/{pid}/stat")).unwrap();
        let after = &stat[stat.rfind(')').unwrap() + 2..];
        let pgrp: u32 = after.split(' ').nth(2).unwrap().parse().unwrap();
        assert_eq!(pgrp, pid);
        assert!(matches!(Box::new(c).close(), Ended::Exited(_)));
        assert!(gone(pid));
    }

    #[test]
    fn a_closed_stdin_ends_a_program_that_reads_it() {
        let c = sh("exec cat >/dev/null");
        let pid = c.id();
        let began = Instant::now();
        match Box::new(c).close() {
            Ended::Exited(e) => assert!(e.success, "{e:?}"),
            Ended::Killed => panic!("killed"),
        }
        assert!(began.elapsed() < child::CLOSE_GRACE);
        assert!(cfg!(not(target_os = "linux")) || gone(pid));
    }

    #[test]
    fn a_closed_stdout_ends_a_program_that_writes() {
        // `yes` ends at its next write, by EPIPE or SIGPIPE.
        let mut c = sh("exec yes");
        assert!(matches!(c.read(Duration::from_secs(5)), Got::Data(_)));
        let began = Instant::now();
        assert!(matches!(Box::new(c).close(), Ended::Exited(_)));
        assert!(began.elapsed() < child::CLOSE_GRACE);
    }

    #[test]
    fn a_program_that_ends_on_nothing_is_killed_and_reaped() {
        let c = sh("trap '' PIPE HUP INT TERM; exec sleep 30 </dev/null >/dev/null");
        let pid = c.id();
        let began = Instant::now();
        assert_eq!(Box::new(c).close(), Ended::Killed);
        assert!(began.elapsed() >= child::CLOSE_GRACE);
        assert!(began.elapsed() < child::CLOSE_GRACE + Duration::from_secs(5));
        assert!(cfg!(not(target_os = "linux")) || gone(pid));
    }

    /// A wrapper that doesn't `exec` (`omegartmcli` a script that runs wine): the
    /// kill takes the program it runs too.
    #[test]
    fn a_kill_takes_what_a_wrapper_runs() {
        let mut c = sh("sleep 30 </dev/null >/dev/null & echo $!; wait; true");
        let mut line = Vec::new();
        while !line.ends_with(b"\n") {
            match c.read(Duration::from_secs(5)) {
                Got::Data(d) => line.extend(d),
                other => panic!("{other:?}"),
            }
        }
        let inner: u32 = String::from_utf8(line).unwrap().trim().parse().unwrap();
        assert_eq!(Box::new(c).close(), Ended::Killed);
        let began = Instant::now();
        while !gone(inner) {
            assert!(began.elapsed() < Duration::from_secs(5), "the wrapped program lives on");
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    #[test]
    fn a_dropped_program_is_ended_too() {
        let c = sh("exec sleep 30");
        let pid = c.id();
        drop(c);
        assert!(cfg!(not(target_os = "linux")) || gone(pid));
    }

    #[test]
    fn the_end_of_a_stream_comes_with_its_status() {
        let mut c = sh("printf ab; exit 3");
        let mut got = Vec::new();
        loop {
            match c.read(Duration::from_secs(5)) {
                Got::Data(d) => got.extend(d),
                Got::End(None) => break,
                other => panic!("{other:?}"),
            }
        }
        assert_eq!(got, b"ab");
        let e = c.exited(Duration::from_secs(5)).unwrap();
        assert!(!e.success);
        assert!(e.text.contains('3'), "{e:?}");
    }

    #[test]
    fn version_runs_to_its_end_and_takes_no_stdin() {
        let sh = OsString::from("/bin/sh");
        // The program's own -version ends with status 1, which means nothing.
        let mut cmd = std::process::Command::new("true");
        let _ = cmd.status();
        let out = run_version_of(&sh, "read x && echo got; printf 'Version 9\\r\\n'; exit 1");
        assert_eq!(out, b"Version 9\r\n");
    }

    /// `run_version` of a script: `sh -c SCRIPT -version`, the script's `$0` is
    /// `-version`.
    fn run_version_of(sh: &OsString, script: &str) -> Vec<u8> {
        let dir = std::env::temp_dir().join(format!(
            "oxrok-asix-{}-{}",
            std::process::id(),
            script.len()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let exe = dir.join(format!("p{}", script.len()));
        std::fs::write(&exe, format!("#!{}\n{script}\n", sh.to_string_lossy())).unwrap();
        std::fs::set_permissions(&exe, std::fs::Permissions::from_mode(0o755)).unwrap();
        let out = run_fresh(&exe.into_os_string(), Duration::from_secs(10));
        let _ = std::fs::remove_dir_all(&dir);
        out.unwrap()
    }

    /// `run_version` of a script just written, which a test running beside this
    /// one may hold open for an instant across its fork: `ETXTBSY` is tried again.
    fn run_fresh(exe: &OsString, bound: Duration) -> std::result::Result<Vec<u8>, VersionError> {
        for _ in 0..50 {
            match run_version(exe, bound) {
                Err(VersionError::Spawn(e)) if e.raw_os_error() == Some(26) => {
                    std::thread::sleep(Duration::from_millis(20));
                }
                other => return other,
            }
        }
        run_version(exe, bound)
    }

    /// A descriptor oxrok was handed without `FD_CLOEXEC` (a CI runner gives two):
    /// glib's spawn closes it in the child, and so must this, for both runs.
    #[cfg(target_os = "linux")]
    #[test]
    fn a_program_gets_no_inherited_descriptor() {
        use rustix::io::{FdFlags, fcntl_setfd};
        use std::os::fd::{AsFd as _, AsRawFd as _};
        let file = std::fs::File::open("/dev/null").unwrap();
        fcntl_setfd(file.as_fd(), FdFlags::empty()).unwrap();
        // Whether the shell has the descriptor's number open.
        let test = format!("[ -e /proc/$$/fd/{} ] && echo open || echo closed", file.as_raw_fd());
        let mut c = sh(&format!("{test}; exec cat >/dev/null"));
        let mut got = Vec::new();
        while !got.ends_with(b"\n") {
            match c.read(Duration::from_secs(5)) {
                Got::Data(d) => got.extend(d),
                other => panic!("{other:?}"),
            }
        }
        assert_eq!(String::from_utf8(got).unwrap().trim(), "closed");
        drop(c);
        let out =
            run_version_of(&OsString::from("/bin/sh"), &format!("echo \"Version $({test})\""));
        assert_eq!(protocol::parse_version(&out).as_deref(), Some("closed"));
        drop(file);
    }

    /// `-version` stays in oxrok's process group: an oxrok a terminal's Ctrl-C ends
    /// during a scan, before it has a handler, takes the program along.
    #[cfg(target_os = "linux")]
    #[test]
    fn version_runs_in_our_group() {
        let sh = OsString::from("/bin/sh");
        let out =
            run_version_of(&sh, "read -r a b c d e rest < /proc/$$/stat; echo \"Version $e\"");
        let stat = std::fs::read_to_string("/proc/self/stat").unwrap();
        let ours = stat[stat.rfind(')').unwrap() + 2..].split(' ').nth(2).unwrap().to_owned();
        assert_eq!(protocol::parse_version(&out), Some(ours));
    }

    #[test]
    fn version_failures() {
        assert!(matches!(
            run_version(&OsString::from("/nonexistent/omegartmcli"), Duration::from_secs(5)),
            Err(VersionError::NotFound)
        ));
        let dir = std::env::temp_dir().join(format!("oxrok-asix-noexec-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let exe = dir.join("omegartmcli");
        std::fs::write(&exe, "#!/bin/sh\n").unwrap();
        let e = run_version(&exe.clone().into_os_string(), Duration::from_secs(5));
        assert!(matches!(e, Err(VersionError::Spawn(_))), "{e:?}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_version_that_hangs_is_killed_at_the_bound() {
        let dir = std::env::temp_dir().join(format!("oxrok-asix-hang-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let exe = dir.join("omegartmcli");
        std::fs::write(&exe, "#!/bin/sh\nprintf 'Version 1\\r\\n'\nexec sleep 30\n").unwrap();
        std::fs::set_permissions(&exe, std::fs::Permissions::from_mode(0o755)).unwrap();
        let began = Instant::now();
        let e = run_fresh(&exe.into_os_string(), Duration::from_millis(300));
        assert!(matches!(e, Err(VersionError::Timeout(_))), "{e:?}");
        assert!(began.elapsed() < Duration::from_secs(5));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The whole capture through a real child: the first chunk, two samples, and
    /// a program that ends at its closed stdin.
    #[test]
    fn a_capture_through_a_real_program() {
        let script = "printf '\\132\\132\\255\\336\\000\\000\\001\\000\\001\\000\\003\\000'; exec cat >/dev/null";
        let opener: Opener = Arc::new(move |_: &[&str]| {
            Child::spawn(&OsString::from("/bin/sh"), &["-c", script])
                .map(|c| Box::new(c) as Box<dyn Stream>)
        });
        let mut d = AsixOmegaRtmCliDevice::new("1".into(), parse_conn(None).unwrap(), None, opener);
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let sent = start_and_collect(&mut d).unwrap();
        assert_eq!((sent.samples, sent.failed), (vec![1, 3], None));
    }
}
