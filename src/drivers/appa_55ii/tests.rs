// Counterpart in sigrok:
//   libsigrok/src/hardware/appa-55ii/api.c,
//   libsigrok/src/hardware/appa-55ii/protocol.c,
//   Copyright (C) 2013 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Martin Ling, Gerhard Sittig, Soeren Apel, Daniel Elstner, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on fake ports: one that releases bytes on the Nth read, and one that
//! paces them at the line's real rate.

use super::*;
use std::collections::VecDeque;
use std::sync::PoisonError;
use std::sync::mpsc::{Receiver, RecvTimeoutError, sync_channel};

/// How long a test waits for a capture before it fails: a stop or a limit that does
/// not bite must fail, not hang the run.
const CAPTURE_BOUND: Duration = Duration::from_secs(20);

/// Runs a test on a thread of its own, and fails it if it has not come back in
/// time: a stop that does not bite would make the device's drop join for ever.
fn in_time(test: impl FnOnce() + Send + 'static) {
    let (done, finished) = sync_channel(1);
    let thread = std::thread::spawn(move || {
        test();
        let _ = done.send(());
    });
    match finished.recv_timeout(2 * CAPTURE_BOUND) {
        Ok(()) => {}
        Err(RecvTimeoutError::Timeout) => panic!("not over after {:?}", 2 * CAPTURE_BOUND),
        Err(RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(thread.join().expect_err("it did not finish"));
        }
    }
}

fn frame(kind: u8, content: &[u8]) -> Vec<u8> {
    let mut out = vec![0x55, 0x55, kind, content.len() as u8];
    out.extend_from_slice(content);
    out.push(out.iter().fold(0u8, |s, b| s.wrapping_add(*b)));
    out
}

/// A live frame in tenths.
fn live(t1: i16, t2: i16) -> Vec<u8> {
    let mut c = [0u8; 20];
    c[13] = 0x01;
    c[14..16].copy_from_slice(&t1.to_le_bytes());
    c[16] = 0x05;
    c[17..19].copy_from_slice(&t2.to_le_bytes());
    c[19] = 0x05;
    frame(0x00, &c)
}

/// A log of `temps` as whole frames, in the wiki's pieces of 32.
fn log(temps: &[(i16, i16)]) -> Vec<Vec<u8>> {
    let mut memory = Vec::new();
    for (t1, t2) in temps {
        let mut r = [0u8; 20];
        r[12..14].copy_from_slice(&t1.to_le_bytes());
        r[14..16].copy_from_slice(&t2.to_le_bytes());
        memory.extend_from_slice(&r);
    }
    memory.resize(memory.len().div_ceil(32) * 32, 0xff);
    let n = (temps.len() as u16).to_le_bytes();
    let mut frames = vec![frame(0x18, &[0]), frame(0x11, &[n[0], n[1], 0, 0, 0, 0, 0, 0])];
    frames.extend(memory.chunks(32).map(|p| frame(0x14, p)));
    frames.push(frame(0x19, &[]));
    frames
}

/// What a fake is doing, seen from a test.
#[derive(Default)]
struct Seen {
    reads: usize,
    read_sizes: Vec<usize>,
    written: usize,
    purges: usize,
    /// Reads fail from now on.
    gone: bool,
    /// Offered by a test while a capture runs.
    more: VecDeque<u8>,
    /// A read waits until this goes down.
    hold: bool,
    /// The port was closed.
    dropped: bool,
}

type Shared = Arc<Mutex<Seen>>;

fn lock(seen: &Shared) -> std::sync::MutexGuard<'_, Seen> {
    seen.lock().unwrap_or_else(PoisonError::into_inner)
}

/// A port with its bytes in steps: each read brings the next step, an empty one
/// being no data. When they run out, what a test put in `more`, then nothing.
struct Fake {
    seen: Shared,
    steps: VecDeque<Vec<u8>>,
    pending: VecDeque<u8>,
}

impl Fake {
    fn new(steps: Vec<Vec<u8>>) -> (Fake, Shared) {
        let seen = Shared::default();
        (Fake { seen: Arc::clone(&seen), steps: steps.into(), pending: VecDeque::new() }, seen)
    }
}

impl Drop for Fake {
    fn drop(&mut self) {
        lock(&self.seen).dropped = true;
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
        let mut seen = lock(&self.seen);
        seen.reads += 1;
        seen.read_sizes.push(buf.len());
        while seen.hold {
            drop(seen);
            std::thread::sleep(Duration::from_millis(1));
            seen = lock(&self.seen);
        }
        if seen.gone {
            return Err(Error::device("port gone"));
        }
        if self.pending.is_empty() {
            if let Some(step) = self.steps.pop_front() {
                self.pending.extend(step);
            } else {
                let more: Vec<u8> = seen.more.drain(..).collect();
                self.pending.extend(more);
            }
        }
        drop(seen);
        if self.pending.is_empty() {
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
            return Ok(0);
        }
        let n = buf.len().min(self.pending.len());
        for b in &mut buf[..n] {
            *b = self.pending.pop_front().expect("counted");
        }
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        lock(&self.seen).written += data.len();
        Ok(())
    }
    fn purge(&mut self) -> Result<()> {
        let mut seen = lock(&self.seen);
        seen.purges += 1;
        seen.more.clear();
        self.pending.clear();
        Ok(())
    }
}

/// A UART at 9600 baud: bytes come at a clock's rate, and a purge drops only those
/// that have come. Each frame follows the last by `pause`.
struct Uart {
    input: VecDeque<(Instant, u8)>,
}

/// A byte at 9600/8n1.
const BYTE: Duration = Duration::from_micros(1042);

impl Uart {
    fn new(frames: &[Vec<u8>], pause: Duration) -> Uart {
        let mut at = Instant::now();
        let mut input = VecDeque::new();
        for f in frames {
            at += pause;
            for b in f {
                at += BYTE;
                input.push_back((at, *b));
            }
        }
        Uart { input }
    }

    fn come(&self) -> usize {
        let now = Instant::now();
        self.input.iter().take_while(|(at, _)| *at <= now).count()
    }
}

impl Port for Uart {
    fn name(&self) -> &'static str {
        "/dev/ttyUART"
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
            let n = self.come().min(buf.len());
            if n > 0 {
                for b in &mut buf[..n] {
                    *b = self.input.pop_front().expect("counted").1;
                }
                return Ok(n);
            }
            if Instant::now() >= deadline {
                return Ok(0);
            }
            std::thread::sleep(Duration::from_micros(200));
        }
    }
    fn write(&mut self, _: &[u8]) -> Result<()> {
        panic!("this device takes nothing");
    }
    fn purge(&mut self) -> Result<()> {
        self.input.drain(..self.come());
        Ok(())
    }
}

fn device(port: Box<dyn Port>) -> Appa55iiDevice {
    let mut d = Appa55iiDevice::from_port(port);
    d.open().unwrap();
    d
}

fn start(d: &mut Appa55iiDevice) -> Receiver<(usize, Packet)> {
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// What the packets until the End say, within the bound.
fn until_end(rx: &Receiver<(usize, Packet)>) -> Vec<AnalogData> {
    let deadline = Instant::now() + CAPTURE_BOUND;
    let mut got = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((_, Packet::Analog(a))) => got.push(a),
            Ok((_, Packet::End)) => return got,
            Ok(_) => {}
            Err(e) => panic!("no End: {e}"),
        }
    }
}

/// Each channel's values in `packets`, as (channel, value).
fn values(packets: &[AnalogData]) -> Vec<(usize, f32)> {
    packets.iter().flat_map(|a| a.channels.iter().copied().zip(a.samples.iter().copied())).collect()
}

/// Waits for `what`, within the bound.
fn wait_for(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + CAPTURE_BOUND;
    while !done() {
        assert!(Instant::now() < deadline, "{what}: not within {CAPTURE_BOUND:?}");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn params() -> Params {
    Params::parse(SERIALCOMM).unwrap()
}

// A scan.

/// A scan of `steps`, with a time that only a stall of a minute could meet.
fn scan_finds(steps: Vec<Vec<u8>>) -> (bool, Shared) {
    let (mut fake, seen) = Fake::new(steps);
    (detect(&mut fake, &params(), Duration::from_secs(60)), seen)
}

/// The same where nothing is to be found and the bytes run out: it waits for its
/// time, which may then be short.
fn scan_times_out(steps: Vec<Vec<u8>>) -> bool {
    let (mut fake, _) = Fake::new(steps);
    detect(&mut fake, &params(), Duration::from_millis(100))
}

#[test]
fn a_scan_finds_a_frame_at_its_first_try_and_reads_no_further() {
    let mut bytes = live(1, 2);
    bytes.push(0x77);
    let (found, seen) = scan_finds(vec![bytes]);
    assert!(found);
    assert_eq!(lock(&seen).reads, 25, "a byte at a time, up to the frame's end");
    assert!(lock(&seen).read_sizes.iter().all(|n| *n == 1));
}

#[test]
fn a_scan_passes_over_25_bytes_and_no_more() {
    for (noise, expected) in [(0, true), (1, true), (25, true), (26, false)] {
        let mut bytes = vec![0x3f; noise];
        bytes.extend(live(1, 2));
        let (found, seen) = scan_finds(vec![bytes]);
        assert_eq!(found, expected, "{noise} bytes of noise");
        assert_eq!(lock(&seen).reads, (noise + 25).min(50), "50 bytes are all it looks at");
    }
}

#[test]
fn a_short_frame_needs_25_bytes_behind_its_start() {
    let end = frame(0x19, &[]);
    assert!(!scan_times_out(vec![end.clone()]), "5 bytes alone");
    let mut bytes = end;
    bytes.extend([0x3f; 19]);
    assert!(!scan_times_out(vec![bytes.clone()]), "24 bytes");
    bytes.push(0x3f);
    let (found, seen) = scan_finds(vec![bytes]);
    assert!(found, "25");
    assert_eq!(lock(&seen).reads, 25);
}

#[test]
fn a_frame_longer_than_a_try_is_not_found() {
    assert!(!scan_times_out(vec![frame(0x14, &[0x3f; 32])]));
}

#[test]
fn a_silent_port_ends_a_scan_at_the_time_and_not_before() {
    let (mut fake, _seen) = Fake::new(vec![]);
    let start = Instant::now();
    assert!(!detect(&mut fake, &params(), Duration::from_millis(200)));
    let took = start.elapsed();
    assert!(took >= Duration::from_millis(200), "{took:?}");
    assert!(took < Duration::from_secs(10), "{took:?}");
    assert_eq!(DETECT_TIMEOUT, Duration::from_millis(500));
}

#[test]
fn a_port_that_fails_finds_nothing() {
    let (mut fake, seen) = Fake::new(vec![]);
    lock(&seen).gone = true;
    assert!(!detect(&mut fake, &params(), DETECT_TIMEOUT));
}

// Options.

#[test]
fn options_are_as_c_lists_them() {
    let (fake, _) = Fake::new(vec![]);
    let mut d = device(Box::new(fake));
    let keys: Vec<Key> = d.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(keys, [Key::Continuous, Key::LimitSamples, Key::LimitMsec, Key::DataSource]);
    assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::from("Live"));
    assert_eq!(
        d.list(&Key::DataSource, None).unwrap(),
        ValueList::Str(vec!["Live".to_owned(), "Memory".to_owned()])
    );
    d.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
    assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::from("Memory"));
    let refused = d.set(&Key::DataSource, &Value::from("memory"), None).unwrap_err();
    assert_eq!(refused.to_string(), "invalid argument: invalid value for data_source");
    assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::from("Memory"), "unchanged");
    d.set(&Key::LimitSamples, &Value::U64(9), None).unwrap();
    d.set(&Key::LimitMsec, &Value::U64(8), None).unwrap();
    assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(9));
    assert_eq!(d.get(&Key::LimitMsec, None).unwrap(), Value::U64(8));
    assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::LimitMsec, &Value::U64(0), None).is_err());
    assert!(d.get(&Key::Voltage, None).is_err());
    assert!(d.get(&Key::DataSource, Some("x")).is_err());
    assert!(d.set(&Key::Voltage, &Value::U64(1), None).is_err());
    assert!(d.list(&Key::Voltage, None).is_err());
    assert_eq!(d.info().conn, None, "C lists no conn");
    assert_eq!(d.info().vendor, "APPA");
    assert_eq!(d.info().model, "55II");
    assert_eq!(d.channels()[1].name, "T2");
}

// Live.

#[test]
fn a_limit_ends_a_live_capture_after_the_read_that_reached_it() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![live(100, 200), vec![], live(110, 210), live(120, 220)]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        let got = until_end(&rx);
        assert_eq!(
            values(&got),
            [(0, 10.0), (1, 20.0), (0, 11.0), (1, 21.0)],
            "one packet per probe"
        );
        assert_eq!(lock(&seen).reads, 3, "the third frame was not read");
        assert_eq!(lock(&seen).written, 0);
        assert!(got.iter().all(|a| a.mq == Mq::Temperature && a.unit == Unit::Celsius));
    });
}

#[test]
fn a_read_asks_for_no_more_than_the_buffer_has_room_for() {
    in_time(|| {
        let f = live(1, 2);
        let (fake, seen) = Fake::new(vec![f[..7].to_vec(), f[7..].to_vec()]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        until_end(&rx);
        assert_eq!(lock(&seen).read_sizes, [37, 30]);
    });
}

#[test]
fn a_disabled_probe_is_left_out_and_the_sample_still_counts() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![live(100, 200), live(110, 210), live(120, 220)]);
        let mut d = device(Box::new(fake));
        d.enable_channel(0, false).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)), [(1, 20.0), (1, 21.0)]);
        assert_eq!(lock(&seen).reads, 2);
    });
}

#[test]
fn with_both_off_samples_still_count() {
    in_time(|| {
        let (fake, _) = Fake::new(vec![live(100, 200)]);
        let mut d = device(Box::new(fake));
        d.enable_channel(0, false).unwrap();
        d.enable_channel(1, false).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)), []);
    });
}

// A time limit.

#[test]
fn a_time_limit_ends_a_capture_on_a_silent_device() {
    in_time(|| {
        let (fake, _) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitMsec, &Value::U64(150), None).unwrap();
        let begun = Instant::now();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)), []);
        assert!(begun.elapsed() >= Duration::from_millis(150), "{:?}", begun.elapsed());
    });
}

#[test]
fn a_time_limit_ends_a_capture_on_a_stream() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitMsec, &Value::U64(150), None).unwrap();
        let rx = start(&mut d);
        let keep_on = Arc::new(AtomicBool::new(true));
        let feeder = {
            let (seen, keep_on) = (Arc::clone(&seen), Arc::clone(&keep_on));
            std::thread::spawn(move || {
                while keep_on.load(Ordering::SeqCst) {
                    lock(&seen).more.extend(live(100, 200));
                    std::thread::sleep(Duration::from_millis(1));
                }
            })
        };
        let got = until_end(&rx);
        keep_on.store(false, Ordering::SeqCst);
        feeder.join().unwrap();
        assert!(!got.is_empty(), "it read while the time ran");
    });
}

// A stop and the port going.

#[test]
fn a_stop_ends_a_capture_that_data_keeps_alive() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        let rx = start(&mut d);
        lock(&seen).more.extend(live(100, 200));
        let deadline = Instant::now() + CAPTURE_BOUND;
        loop {
            let (_, p) = rx.recv_timeout(deadline - Instant::now()).expect("the samples");
            if matches!(p, Packet::Analog(_)) {
                break;
            }
        }
        d.stop().unwrap();
        // More on offer, and the End must still come.
        lock(&seen).more.extend(live(100, 200));
        until_end(&rx);
        d.close().unwrap();
    });
}

#[test]
fn a_stop_does_not_wait_for_a_capture_stuck_on_a_full_queue() {
    in_time(|| {
        let frames: Vec<Vec<u8>> = (0..5).map(|_| live(100, 200)).collect();
        let (fake, seen) = Fake::new(frames);
        let mut d = device(Box::new(fake));
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Unread: the header fills the queue, and the first sample has no room.
        wait_for("the read that blocks", || lock(&seen).reads >= 1);
        let begun = Instant::now();
        d.stop().unwrap();
        assert!(begun.elapsed() < Duration::from_secs(5), "a stop does not join");
        d.close().unwrap();
        assert!(d.worker.is_none());
        drop(rx);
    });
}

#[test]
fn a_stop_does_not_wait_for_a_read_in_progress() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![]);
        lock(&seen).hold = true;
        let mut d = device(Box::new(fake));
        let rx = start(&mut d);
        wait_for("the read", || lock(&seen).reads >= 1);
        // A stop that joins would wait for the read, which waits for this thread.
        d.stop().unwrap();
        lock(&seen).hold = false;
        until_end(&rx);
    });
}

#[test]
fn a_close_waits_for_the_capture_and_closes_the_port() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        let _rx = start(&mut d);
        wait_for("the first read", || lock(&seen).reads >= 1);
        d.close().unwrap();
        assert!(lock(&seen).dropped, "the capture held the port, and has given it back");
    });
}

#[test]
fn a_device_dropped_in_a_capture_ends_it_and_closes_the_port() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        let _rx = start(&mut d);
        wait_for("the first read", || lock(&seen).reads >= 1);
        drop(d);
        assert!(lock(&seen).dropped);
    });
}

#[test]
fn a_port_that_goes_ends_the_capture() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        let rx = start(&mut d);
        wait_for("the first read", || lock(&seen).reads >= 1);
        lock(&seen).gone = true;
        assert_eq!(values(&until_end(&rx)), []);
    });
}

#[test]
fn a_sink_that_goes_ends_the_capture() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        let rx = start(&mut d);
        drop(rx);
        lock(&seen).more.extend(live(100, 200));
        wait_for("the port back", || d.port.lock().unwrap().is_some());
    });
}

// A log.

#[test]
fn a_log_ends_at_its_end_and_nothing_goes_after_it() {
    in_time(|| {
        let mut steps = log(&[(100, 200), (110, 0x7fff), (-120, 220)]);
        // What C would parse on to, in the same read.
        let last = steps.last_mut().unwrap();
        last.extend(frame(0x14, &[0; 20]));
        let live_after = live(1, 2);
        steps.push(live_after);
        let total = steps.len();
        let (fake, seen) = Fake::new(steps);
        let mut d = device(Box::new(fake));
        d.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
        let rx = start(&mut d);
        let got = until_end(&rx);
        assert_eq!(
            values(&got),
            [(0, 10.0), (1, 20.0), (0, 11.0), (1, f32::INFINITY), (0, -12.0), (1, 22.0)]
        );
        assert_eq!(got.len(), 3, "one packet per record");
        assert!(got.iter().all(|a| a.digits == 1 && a.flags == MqFlags::empty()));
        assert!(lock(&seen).reads <= total);
    });
}

#[test]
fn a_record_of_one_probe_has_its_value() {
    in_time(|| {
        let (fake, _) = Fake::new(log(&[(100, 200), (110, 210)]));
        let mut d = device(Box::new(fake));
        d.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
        d.enable_channel(0, false).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)), [(1, 20.0), (1, 21.0)]);
    });
}

#[test]
fn a_limit_in_a_log_lets_the_piece_finish() {
    in_time(|| {
        let temps: Vec<(i16, i16)> = (0..6).map(|i| (i * 10, 0)).collect();
        let (fake, seen) = Fake::new(log(&temps));
        let mut d = device(Box::new(fake));
        d.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        let got = until_end(&rx);
        assert_eq!(got.len(), 3, "the second piece of 32 finishes the second and third");
        assert_eq!(lock(&seen).reads, 4, "start, metadata and two pieces");
    });
}

#[test]
fn a_live_stream_does_not_end_a_memory_capture_nor_a_log_a_live_one() {
    in_time(|| {
        // Memory: a stream with no log goes on until the stop.
        let (fake, seen) = Fake::new(vec![live(1, 2), live(3, 4)]);
        let mut d = device(Box::new(fake));
        d.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        wait_for("the frames read", || lock(&seen).reads >= 3);
        d.stop().unwrap();
        assert_eq!(values(&until_end(&rx)), []);

        // Live: a log's end ends nothing.
        let mut steps = log(&[(100, 200)]);
        steps.push(live(1, 2));
        let (fake, _) = Fake::new(steps);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)), [(0, 0.1), (1, 0.2)]);
    });
}

#[test]
fn a_source_set_in_a_capture_takes_effect() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        let rx = start(&mut d);
        lock(&seen).more.extend(live(100, 200));
        let deadline = Instant::now() + CAPTURE_BOUND;
        let mut got = Vec::new();
        while got.len() < 2 {
            let (_, p) = rx.recv_timeout(deadline - Instant::now()).expect("the first reading");
            if let Packet::Analog(a) = p {
                got.push(a);
            }
        }
        // After frames have come: a capture that read it once stays live, takes the
        // next frame and passes over the end.
        d.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
        lock(&seen).more.extend(live(999, 999));
        wait_for("the frame read", || lock(&seen).more.is_empty());
        lock(&seen).more.extend(frame(0x19, &[]));
        got.extend(until_end(&rx));
        assert_eq!(values(&got), [(0, 10.0), (1, 20.0)], "the second was passed over");
    });
}

// A device that falls silent.

#[test]
fn the_silence_bound_is_ten_seconds() {
    assert_eq!(SILENCE_BOUND, Duration::from_secs(10));
    let (fake, _) = Fake::new(vec![]);
    assert_eq!(Appa55iiDevice::from_port(Box::new(fake)).silence_bound, Duration::from_secs(10));
}

#[test]
fn silence_ends_a_live_capture_that_only_a_sample_limit_can_end() {
    in_time(|| {
        let (fake, _) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        d.silence_bound = Duration::from_millis(150);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let begun = Instant::now();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)), []);
        assert!(begun.elapsed() >= Duration::from_millis(150), "{:?}", begun.elapsed());
    });
}

#[test]
fn a_byte_of_any_kind_puts_the_silence_off() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        d.silence_bound = Duration::from_secs(2);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let begun = Instant::now();
        let rx = start(&mut d);
        // Noise, and no sample, for longer than the bound.
        let mut last = begun;
        while begun.elapsed() < Duration::from_millis(2500) {
            lock(&seen).more.push_back(0x3f);
            last = Instant::now();
            std::thread::sleep(Duration::from_millis(10));
            assert!(d.port.lock().unwrap().is_none(), "over at {:?}", begun.elapsed());
        }
        assert_eq!(values(&until_end(&rx)), []);
        assert!(last.elapsed() >= Duration::from_secs(2), "{:?}", last.elapsed());
    });
}

/// Starts a capture with a short bound and waits until it has gone far past it.
fn still_runs_after_the_bound(set: impl FnOnce(&mut Appa55iiDevice)) {
    let (fake, seen) = Fake::new(vec![]);
    let mut d = device(Box::new(fake));
    d.silence_bound = Duration::from_millis(20);
    set(&mut d);
    let rx = start(&mut d);
    // Each empty read sleeps a millisecond at least.
    wait_for("the reads", || lock(&seen).reads >= 200);
    assert!(d.port.lock().unwrap().is_none(), "it ended");
    d.stop().unwrap();
    until_end(&rx);
}

#[test]
fn silence_ends_nothing_with_no_limit() {
    in_time(|| still_runs_after_the_bound(|_| {}));
}

#[test]
fn silence_ends_nothing_under_a_time_limit() {
    in_time(|| {
        still_runs_after_the_bound(|d| {
            d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
            d.set(&Key::LimitMsec, &Value::U64(3_600_000), None).unwrap();
        });
    });
}

#[test]
fn silence_ends_nothing_in_memory_mode() {
    in_time(|| {
        still_runs_after_the_bound(|d| {
            d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
            d.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
        });
    });
}

// What a line does.

#[test]
fn a_size_over_32_ends_the_capture() {
    in_time(|| {
        let mut bad = vec![0x55, 0x55, 0x00, 33];
        bad.extend([0u8; 40]);
        let (fake, seen) = Fake::new(vec![live(100, 200), bad, live(1, 2)]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(5), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)), [(0, 10.0), (1, 20.0)]);
        assert_eq!(lock(&seen).read_sizes.len(), 2, "the second read filled it");
    });
}

#[test]
fn a_broken_frame_costs_that_reading_alone() {
    in_time(|| {
        let mut bad = live(555, 555);
        *bad.last_mut().unwrap() ^= 1;
        let (fake, _) = Fake::new(vec![live(100, 200), bad, live(110, 210)]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)), [(0, 10.0), (1, 20.0), (0, 11.0), (1, 21.0)]);
    });
}

#[test]
fn a_log_with_a_broken_piece_keeps_no_shifted_reading() {
    in_time(|| {
        let temps: Vec<(i16, i16)> = (0..6).map(|i| (100 + i, 200 + i)).collect();
        let mut frames = log(&temps);
        *frames[3].last_mut().unwrap() ^= 1;
        let (fake, _) = Fake::new(frames);
        let mut d = device(Box::new(fake));
        d.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)), [(0, 10.0), (1, 20.0)], "only the first piece's");
    });
}

// Again, on an open device.

#[test]
fn a_second_capture_starts_clean_on_the_rest_of_a_frame() {
    in_time(|| {
        // A frame's tail comes first, as a stop in mid-frame leaves it, and must not
        // be joined to anything.
        let f = live(100, 200);
        let (fake, seen) = Fake::new(vec![f.clone()]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)).len(), 2);
        assert_eq!(lock(&seen).purges, 1, "the open's");

        let rx = start(&mut d);
        let mut bytes = f[10..].to_vec();
        bytes.extend(live(110, 210));
        bytes.extend(live(120, 220));
        lock(&seen).more.extend(bytes);
        assert_eq!(values(&until_end(&rx)), [(0, 11.0), (1, 21.0)]);
        assert_eq!(lock(&seen).purges, 2, "and this start's");
    });
}

#[test]
fn what_waited_between_captures_is_not_a_reading() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![live(100, 200)]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        until_end(&rx);
        lock(&seen).more.extend(live(999, 999));
        let rx = start(&mut d);
        wait_for("the purge", || lock(&seen).purges == 2);
        lock(&seen).more.extend(live(110, 210));
        assert_eq!(values(&until_end(&rx)), [(0, 11.0), (1, 21.0)]);
    });
}

#[test]
fn a_first_capture_drops_nothing_more_than_the_open_did() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![live(100, 200)]);
        let mut d = device(Box::new(fake));
        assert_eq!(lock(&seen).purges, 1);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        until_end(&rx);
        assert_eq!(lock(&seen).purges, 1);
    });
}

#[test]
fn a_new_open_is_a_first_capture_again() {
    in_time(|| {
        let (fake, _) = Fake::new(vec![live(100, 200)]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        until_end(&rx);
        d.close().unwrap();
        let (fake, seen) = Fake::new(vec![live(110, 210)]);
        d.pending = Some(Box::new(fake));
        d.open().unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&until_end(&rx)), [(0, 11.0), (1, 21.0)]);
        assert_eq!(lock(&seen).purges, 1, "only the open's");
    });
}

#[test]
fn a_log_after_a_capture_has_no_count_from_before() {
    in_time(|| {
        // A count left over would turn this log's bytes into records.
        let meta = frame(0x11, &[9, 0, 0, 0, 0, 0, 0, 0]);
        let (fake, seen) = Fake::new(vec![meta]);
        let mut d = device(Box::new(fake));
        d.set(&Key::DataSource, &Value::from("Memory"), None).unwrap();
        let rx = start(&mut d);
        wait_for("the metadata read", || lock(&seen).reads >= 2);
        lock(&seen).more.extend(frame(0x19, &[]));
        until_end(&rx);
        let rx = start(&mut d);
        let mut rest = frame(0x14, &[0; 20]);
        rest.extend(frame(0x19, &[]));
        lock(&seen).more.extend(rest);
        assert_eq!(values(&until_end(&rx)), []);
    });
}

#[test]
fn a_start_while_one_runs_is_refused_and_one_after_it_goes() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![]);
        let mut d = device(Box::new(fake));
        let rx = start(&mut d);
        wait_for("the first read", || lock(&seen).reads >= 1);
        let (tx2, _rx2) = sync_channel(4096);
        assert!(matches!(d.start(PacketSink::new(tx2, 0)), Err(Error::Busy)));
        d.stop().unwrap();
        until_end(&rx);
        let rx = start(&mut d);
        d.stop().unwrap();
        until_end(&rx);
    });
}

#[test]
fn a_start_after_an_end_stuck_on_a_full_queue_does_not_hang() {
    in_time(|| {
        let (fake, _) = Fake::new(vec![live(100, 200), live(110, 210)]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        // Room for the header and the two probes: the End has none, unread.
        let (tx, rx) = sync_channel(3);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for("the port back", || d.port.lock().unwrap().is_some());
        // The port is back before the End: that End can't be out, as no one reads.
        let begun = Instant::now();
        let (tx2, rx2) = sync_channel(4096);
        d.start(PacketSink::new(tx2, 0)).unwrap();
        assert!(begun.elapsed() < Duration::from_secs(10), "{:?}", begun.elapsed());
        let (mut analog, mut ended) = (0, false);
        while let Ok((_, p)) = rx.try_recv() {
            analog += usize::from(matches!(p, Packet::Analog(_)));
            ended |= matches!(p, Packet::End);
        }
        assert_eq!((analog, ended), (2, false), "that End was given up, with the stop");
        assert_eq!(values(&until_end(&rx2)), [(0, 11.0), (1, 21.0)]);
    });
}

#[test]
fn the_end_comes_after_the_port_is_back() {
    in_time(|| {
        // No End can be seen with the port not back: a client may start on it. With
        // a queue that has no room for it, the port has to be back without it.
        let (fake, _) = Fake::new(vec![live(100, 200)]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let (tx, rx) = sync_channel(3);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for("the port back", || d.port.lock().unwrap().is_some());
        std::thread::sleep(Duration::from_millis(5));
        let mut seen_end = false;
        while let Ok((_, p)) = rx.try_recv() {
            seen_end |= matches!(p, Packet::End);
        }
        assert!(!seen_end, "there was no room for it");
    });
}

#[test]
fn a_close_with_no_capture_and_a_close_after_one_are_quiet() {
    in_time(|| {
        let (fake, seen) = Fake::new(vec![live(100, 200)]);
        let mut d = device(Box::new(fake));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        until_end(&rx);
        d.close().unwrap();
        d.close().unwrap();
        assert!(d.worker.is_none());
        assert_eq!(lock(&seen).written, 0, "this driver never writes");
        let (tx, _rx) = sync_channel(1);
        assert!(d.start(PacketSink::new(tx, 0)).is_err(), "not open");
    });
}

// At the line's real rate.

#[test]
fn frames_at_9600_baud_read_right_in_whatever_pieces_come() {
    in_time(|| {
        let frames: Vec<Vec<u8>> = (0..4).map(|i| live(100 + i, 200 + i)).collect();
        let uart = Uart::new(&frames, Duration::from_millis(5));
        let mut d = device(Box::new(uart));
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(
            values(&until_end(&rx)),
            [(0, 10.0), (1, 20.0), (0, 10.1), (1, 20.1), (0, 10.2), (1, 20.2)]
        );
    });
}

#[test]
fn a_stop_in_the_middle_of_a_frame_leaves_the_next_capture_in_step() {
    in_time(|| {
        // The line runs on: 52 s of frames, each a value above the last. The stop
        // lands where it lands, a frame's tail may still be on its way behind the
        // purge, and readings must come only from whole frames.
        let frames: Vec<Vec<u8>> = (0..2000).map(|i| live(100 + i, 5000 + i)).collect();
        let uart = Uart::new(&frames, Duration::ZERO);
        let mut d = device(Box::new(uart));
        let rx = start(&mut d);
        std::thread::sleep(Duration::from_millis(40));
        d.stop().unwrap();
        let first = values(&until_end(&rx));
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        let rx = start(&mut d);
        let second = values(&until_end(&rx));
        assert_eq!(second.len(), 6);
        let mut last = 0.0;
        for pair in first.chunks(2).chain(second.chunks(2)) {
            assert_eq!((pair[0].0, pair[1].0), (0, 1));
            assert!((pair[1].1 - pair[0].1 - 490.0).abs() < 0.01, "{pair:?}");
            assert!(pair[0].1 > last, "{pair:?} after {last}: in order, none twice");
            last = pair[0].1;
        }
    });
}
