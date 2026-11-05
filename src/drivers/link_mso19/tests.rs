// Counterpart in sigrok:
//   libsigrok/src/hardware/link-mso19/protocol.c,
//   Copyright (C) 2011 Daniel Ribeiro <drwyrm@gmail.com>,
//   Copyright (C) 2012 Renato Caldas <rmsc@fe.up.pt>,
//   Copyright (C) 2013 Lior Elazary <lelazary@yahoo.com>,
//   Copyright (C) 2022 Paul Kasemir <paul.kasemir@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Daniel Elstner, Gerhard Sittig, Martin Ling, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake port: what cannot be timed against the emulator. The fake
//! releases bytes on the Nth read, never at a clock time, so a slow host changes
//! nothing but how long a test takes.

use super::protocol::{frame, read_buffer_request, read_status_request, word};
use super::*;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, RecvTimeoutError, sync_channel};

const SERIAL: &str = "4287434350012000123";
/// A wait for what must happen, however slow the host. A fault hits it.
const BOUND: Duration = Duration::from_secs(60);

/// A device behind a line: answers its status and dump requests in turn, on the
/// reads to come.
#[derive(Default)]
struct Line {
    /// What was written, a write each.
    written: Vec<Vec<u8>>,
    /// What has arrived and a read gets.
    arrived: VecDeque<u8>,
    /// What is on its way: each read moves this much to `arrived`, a real read of
    /// a chunk taking the line's time.
    coming: VecDeque<u8>,
    /// Per read. 0 is everything at once.
    chunk: usize,
    /// The answers to status requests in turn; empty means none.
    statuses: VecDeque<u8>,
    /// What a dump request answers.
    dump: Vec<u8>,
    /// Reads so far.
    reads: usize,
    /// A read fails from then on.
    fail: bool,
    /// Nothing more arrives.
    hold: bool,
    /// A dump request sets `hold`.
    hold_dump: bool,
    /// What each read takes, whether or not it brings anything.
    pause: Duration,
}

#[derive(Clone)]
struct Fake(Arc<Mutex<Line>>);

impl Fake {
    fn new(statuses: &[u8]) -> Fake {
        Fake(Arc::new(Mutex::new(Line {
            statuses: statuses.iter().copied().collect(),
            dump: dump_bytes(),
            ..Line::default()
        })))
    }

    fn line(&self) -> std::sync::MutexGuard<'_, Line> {
        // A failed test holds it, and a drop still comes through here.
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn frames(&self) -> Vec<Vec<u8>> {
        self.line().written.clone()
    }

    fn count(&self, words: &[[u8; 2]]) -> usize {
        let wanted = frame(words);
        self.line().written.iter().filter(|w| **w == wanted).count()
    }

    fn set_chunk(&self, chunk: usize) {
        self.line().chunk = chunk;
    }
}

impl Port for Fake {
    fn name(&self) -> &'static str {
        "fake"
    }

    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }

    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut line = self.line();
        line.reads += 1;
        if line.fail {
            return Err(Error::Io(std::io::Error::other("gone")));
        }
        let take = match (line.hold, line.chunk) {
            (true, _) => 0,
            (false, 0) => line.coming.len(),
            (false, chunk) => chunk.min(line.coming.len()),
        };
        let pause = line.pause;
        for _ in 0..take {
            let b = line.coming.pop_front().expect("counted");
            line.arrived.push_back(b);
        }
        let n = buf.len().min(line.arrived.len());
        for slot in &mut buf[..n] {
            *slot = line.arrived.pop_front().expect("counted");
        }
        drop(line);
        std::thread::sleep(if n == 0 { timeout } else { pause });
        Ok(n)
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut line = self.line();
        line.written.push(data.to_vec());
        if data == frame(&read_status_request()) {
            if let Some(status) = line.statuses.pop_front() {
                line.coming.push_back(status);
            }
        } else if data == frame(&read_buffer_request()) {
            let dump = line.dump.clone();
            line.coming.extend(dump);
            line.hold = line.hold_dump;
        }
        Ok(())
    }

    fn purge(&mut self) -> Result<()> {
        self.line().arrived.clear();
        Ok(())
    }
}

/// The samples of a dump: analog 2 * i and logic i, with the high bits as noise.
fn dump_bytes() -> Vec<u8> {
    let mut out = Vec::new();
    for i in 0..NUM_SAMPLES {
        let analog = (i * 2) & 0x3ff;
        let logic = i & 0xff;
        out.push((analog & 0x3f) as u8 | 0x40);
        out.push((((analog >> 6) & 0xf) | ((logic & 3) << 4)) as u8 | 0x80);
        out.push(((logic >> 2) & 0x3f) as u8 | 0xc0);
    }
    out
}

fn open_device(statuses: &[u8]) -> (MsoDevice, Fake) {
    let fake = Fake::new(statuses);
    let mut device = MsoDevice::from_port(Box::new(fake.clone()), SERIAL).unwrap();
    device.open().unwrap();
    (device, fake)
}

fn sink(depth: usize) -> (PacketSink, Receiver<(usize, Packet)>) {
    let (tx, rx) = sync_channel(depth);
    (PacketSink::new(tx, 0), rx)
}

/// The packets up to End, with a deadline of their own: a capture that keeps
/// sending fails here rather than hanging.
fn packets(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((_, Packet::End)) => return out,
            Ok((_, p)) => out.push(p),
            Err(RecvTimeoutError::Timeout) => panic!("no End within the bound"),
            Err(RecvTimeoutError::Disconnected) => panic!("no End before the sink went"),
        }
    }
}

/// `f` on its own thread: a stop that does not bite fails, where the device's
/// drop would hang the test.
fn bounded<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    let (tx, rx) = sync_channel(1);
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    rx.recv_timeout(BOUND).expect("the test ran over its bound")
}

fn shape(packets: &[Packet]) -> Vec<String> {
    packets
        .iter()
        .map(|p| match p {
            Packet::Header { .. } => "header".to_owned(),
            Packet::Trigger => "trigger".to_owned(),
            Packet::Logic(l) => format!("logic {}", l.data.len()),
            Packet::Analog(a) => format!("analog {}", a.samples.len()),
            other => format!("{other:?}"),
        })
        .collect()
}

#[test]
fn an_open_resets_and_reads_a_status() {
    let (_device, fake) = open_device(&[0x21]);
    assert_eq!(
        fake.frames(),
        [
            frame(&[word(15, 0), word(14, 0x40), word(14, 0)]),
            frame(&read_status_request()),
            frame(&[word(14, 0x11)]),
        ],
    );
}

#[test]
fn an_open_fails_on_a_status_no_device_gives_or_none() {
    for statuses in [&[0xffu8][..], &[0x00], &[]] {
        let fake = Fake::new(statuses);
        let mut device = MsoDevice::from_port(Box::new(fake.clone()), SERIAL).unwrap();
        assert!(device.open().is_err(), "{statuses:?}");
        // No reset of the sequencer follows.
        assert_eq!(fake.frames().len(), 2, "{statuses:?}");
    }
}

#[test]
fn a_capture_writes_all_in_order_then_polls_then_dumps() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x33, 0x34, 0x36]);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        let got = packets(&rx);
        assert_eq!(
            shape(&got),
            ["header", "logic 512", "analog 512", "trigger", "logic 512", "analog 512"],
        );
        let written = fake.frames();
        let status = frame(&read_status_request());
        // The open's three, then a start's, as `dev_acquisition_start` has them.
        assert_eq!(written[3], frame(&[word(14, 0x11)]));
        assert_eq!(written[4], frame(&[word(9, 0x9f), word(10, 0x0e)]));
        assert_eq!(written[5], frame(&[word(12, 0x01), word(13, 0x57), word(14, 0xb0)]));
        assert_eq!(written[6], frame(&[word(12, 0x8f), word(13, 0xff), word(14, 0xb0)]));
        assert_eq!(written[8], frame(&[word(14, 0x91), word(14, 0x92), word(14, 0x90)]));
        assert_eq!(written[9], status);
        // Its status and the one the loop reads, which says "waiting"; a poll; the
        // one that says "ready"; the dump.
        assert_eq!(written[10], status);
        assert_eq!(written[11], status);
        assert_eq!(written[12], frame(&read_buffer_request()));
        assert_eq!(written.len(), 13);
    });
}

#[test]
fn the_samples_decode_and_split_at_the_trigger() {
    bounded(|| {
        let (mut device, _fake) = open_device(&[0x21, 0x33, 0x36]);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        let got = packets(&rx);
        let Packet::Logic(first) = &got[1] else { panic!("{:?}", shape(&got)) };
        let Packet::Analog(analog) = &got[2] else { panic!("{:?}", shape(&got)) };
        let Packet::Logic(second) = &got[4] else { panic!("{:?}", shape(&got)) };
        assert_eq!(first.unit_size, 1);
        assert_eq!(first.data[..4], [0, 1, 2, 3]);
        assert_eq!(second.data[0], 0);
        assert_eq!(second.data[1], 1);
        assert_eq!(analog.channels, [4]);
        assert_eq!(analog.digits, 3);
        assert_eq!(analog.flags, MqFlags::DC);
        // Analog 2 at 4.2874 mV per bit and a 10x probe.
        let want = (512.0 - 2.0) * 0.004_287_4 * 10.0;
        assert!((f64::from(analog.samples[1]) - want).abs() < 1e-3);
    });
}

#[test]
fn a_position_puts_the_trigger_or_leaves_it_out() {
    for (position, want) in [
        (0.0, vec!["header", "trigger", "logic 1024", "analog 1024"]),
        (1.0, vec!["header", "logic 1023", "analog 1023", "trigger", "logic 1", "analog 1"]),
        (-2.5, vec!["header", "logic 1024", "analog 1024"]),
    ] {
        let got = bounded(move || {
            let (mut device, _fake) = open_device(&[0x21, 0x33, 0x36]);
            device.set(&Key::HorizTriggerpos, &Value::F64(position), None).unwrap();
            let (tx, rx) = sink(16);
            device.start(tx).unwrap();
            shape(&packets(&rx))
        });
        assert_eq!(got, want, "{position}");
    }
}

#[test]
fn ready_at_the_first_status_still_asks_for_the_dump() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x36, 0x36]);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(shape(&packets(&rx)).len(), 6);
        assert_eq!(fake.count(&read_buffer_request()), 1);
    });
}

#[test]
fn a_status_read_at_the_start_that_fails_refuses_it() {
    for statuses in [&[0x21u8, 0xff][..], &[0x21]] {
        let statuses = statuses.to_vec();
        bounded(move || {
            let (mut device, _fake) = open_device(&statuses);
            let (tx, _rx) = sink(16);
            assert!(device.start(tx).is_err());
            // The port came back: a client can try again.
            assert!(device.port().is_some());
        });
    }
}

#[test]
fn an_invalid_status_in_the_polls_ends_with_no_samples() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x33, 0xff, 0x36]);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(shape(&packets(&rx)), ["header"]);
        assert_eq!(fake.count(&read_buffer_request()), 0);
    });
}

#[test]
fn a_silent_device_ends_at_the_bound_and_not_before() {
    assert_eq!(SILENCE_BOUND, Duration::from_secs(5));
    bounded(|| {
        let (mut device, _fake) = open_device(&[0x21, 0x33]);
        device.silence_bound = Duration::from_millis(300);
        let at = Instant::now();
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(shape(&packets(&rx)), ["header"]);
        assert!(at.elapsed() >= Duration::from_millis(300), "{:?}", at.elapsed());
    });
}

#[test]
fn a_dump_that_stops_short_ends_at_the_bound() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x33, 0x36]);
        fake.line().dump.truncate(100);
        device.silence_bound = Duration::from_millis(100);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(shape(&packets(&rx)), ["header"]);
    });
}

#[test]
fn a_dump_slower_than_the_bound_in_all_is_not_silence() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x33, 0x36]);
        // Each read brings a little, so the whole takes over the bound while a read
        // that has something is not silence, whatever a read takes.
        fake.set_chunk(8);
        fake.line().pause = Duration::from_millis(1);
        device.silence_bound = Duration::from_millis(100);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(shape(&packets(&rx)).len(), 6);
    });
}

#[test]
fn a_read_that_takes_longer_than_the_bound_and_brings_bytes_is_not_silence() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x33, 0x36]);
        fake.line().pause = Duration::from_millis(60);
        device.silence_bound = Duration::from_millis(20);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(shape(&packets(&rx)).len(), 6);
    });
}

#[test]
fn a_close_joins_the_capture() {
    bounded(|| {
        let mut statuses = vec![0x21];
        statuses.extend([0x34; 2000]);
        let (mut device, _fake) = open_device(&statuses);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        device.close().unwrap();
        // The thread holds a reference until it has ended.
        assert_eq!(Arc::strong_count(&device.shared), 1);
        drop(rx);
    });
}

#[test]
fn a_capture_and_a_second_at_the_line_s_rate() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x33, 0x36, 0x33, 0x34, 0x36]);
        fake.set_chunk(64);
        fake.line().pause = Duration::from_micros(1400);
        for _ in 0..2 {
            let (tx, rx) = sink(16);
            device.start(tx).unwrap();
            assert_eq!(shape(&packets(&rx)).len(), 6);
            wait_for("the port back", || device.port().is_some());
        }
        assert_eq!(fake.count(&read_buffer_request()), 2);
    });
}

#[test]
fn a_read_that_fails_ends_the_capture() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x33]);
        // No bound ends this one first.
        device.silence_bound = BOUND * 10;
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        fake.line().fail = true;
        assert_eq!(shape(&packets(&rx)), ["header"]);
    });
}

/// Waits, with a bound, until `f` says so.
fn wait_for(what: &str, f: impl Fn() -> bool) {
    let deadline = Instant::now() + BOUND;
    while !f() {
        assert!(Instant::now() < deadline, "no {what} within the bound");
        std::thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn a_stop_ends_the_polls_of_a_trigger_that_never_fires() {
    bounded(|| {
        let mut statuses = vec![0x21];
        statuses.extend([0x34; 2000]);
        let (mut device, fake) = open_device(&statuses);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        // Polling goes on, each answered: no silence ends this.
        let polls = fake.count(&read_status_request());
        wait_for("polls", || fake.count(&read_status_request()) >= polls + 3);
        device.stop().unwrap();
        assert_eq!(shape(&packets(&rx)), ["header"]);
    });
}

#[test]
fn a_stop_in_the_silence_ends_it_with_no_wait_for_the_bound() {
    bounded(|| {
        let (mut device, _fake) = open_device(&[0x21, 0x33]);
        device.silence_bound = BOUND * 10;
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        wait_for("the header", || matches!(rx.try_recv(), Ok((_, Packet::Header { .. }))));
        device.stop().unwrap();
        assert_eq!(shape(&packets(&rx)), Vec::<String>::new());
    });
}

#[test]
fn a_close_on_a_full_queue_does_not_hang() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x33, 0x36]);
        let (tx, rx) = sink(1);
        device.start(tx).unwrap();
        // The header fills the queue, and no one reads: the samples cannot go.
        wait_for("the dump request", || fake.count(&read_buffer_request()) == 1);
        device.close().unwrap();
        drop(rx);
    });
}

#[test]
fn a_start_after_an_end_stuck_on_a_full_queue_does_not_hang() {
    bounded(|| {
        let (mut device, _fake) = open_device(&[0x21, 0x33, 0x36, 0x33, 0x36]);
        let (tx, rx) = sink(1);
        device.start(tx).unwrap();
        // All but the last packet taken: it fills the queue and End cannot go.
        for _ in 0..5 {
            assert!(rx.recv_timeout(BOUND).is_ok());
        }
        wait_for("the port back", || device.port().is_some());
        let (tx, rx2) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(shape(&packets(&rx2)).len(), 6);
        drop(rx);
    });
}

#[test]
fn a_stop_in_the_dump_does_not_shift_the_next_capture() {
    bounded(|| {
        // First capture: the dump is held on its way, and a stop lands in it.
        let (mut device, fake) = open_device(&[0x21, 0x33, 0x36, 0x33, 0x36]);
        fake.line().hold_dump = true;
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        wait_for("the dump request", || fake.count(&read_buffer_request()) == 1);
        device.stop().unwrap();
        assert_eq!(shape(&packets(&rx)), ["header"]);
        wait_for("the port back", || device.port().is_some());
        // The dump is still on its way, a packet per read. Unsettled, its bytes
        // would be read as statuses.
        {
            let mut line = fake.line();
            assert!(!line.coming.is_empty());
            line.hold = false;
            line.hold_dump = false;
            // The line's rate: 64 bytes in 1.4 ms at 460800 baud.
            line.chunk = 64;
            line.pause = Duration::from_micros(1400);
        }
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(shape(&packets(&rx)).len(), 6);
        assert_eq!(fake.count(&read_buffer_request()), 2);
    });
}

#[test]
fn a_late_status_at_a_start_is_not_read_by_the_next() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21]);
        // The reply comes after the wait is up.
        let (tx, _rx) = sink(16);
        assert!(device.start(tx).is_err());
        fake.line().coming.push_back(0xff);
        fake.line().statuses.extend([0x33, 0x36]);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(shape(&packets(&rx)).len(), 6);
    });
}

#[test]
fn a_finished_capture_leaves_nothing_to_wait_for() {
    bounded(|| {
        let (mut device, _fake) = open_device(&[0x21, 0x33, 0x36, 0x33, 0x36]);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(packets(&rx).len(), 6);
        wait_for("the port back", || device.port().is_some());
        assert!(!device.shared.in_flight.load(Ordering::SeqCst));
        // And a setting right after it goes out with no read, in between captures.
        device.set(&Key::Samplerate, &Value::U64(100), None).unwrap();
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(packets(&rx).len(), 6);
    });
}

#[test]
fn surplus_after_a_dump_is_gone_at_the_next_start() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x33, 0x36, 0x33, 0x36]);
        fake.line().dump.extend([0xff; 50]);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(packets(&rx).len(), 6);
        wait_for("the port back", || device.port().is_some());
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert_eq!(packets(&rx).len(), 6);
    });
}

#[test]
fn a_setting_in_a_capture_is_refused_and_a_get_answers() {
    bounded(|| {
        let mut statuses = vec![0x21];
        statuses.extend([0x34; 2000]);
        let (mut device, _fake) = open_device(&statuses);
        let (tx, rx) = sink(16);
        device.start(tx).unwrap();
        assert!(matches!(
            device.set(&Key::HorizTriggerpos, &Value::F64(0.0), None),
            Err(Error::Busy)
        ));
        assert_eq!(device.get(&Key::Samplerate, None).unwrap().as_u64(), Some(10_000));
        let (tx2, _rx2) = sink(16);
        assert!(matches!(device.start(tx2), Err(Error::Busy)));
        device.stop().unwrap();
        packets(&rx);
    });
}

#[test]
fn a_refused_setting_changes_nothing() {
    let (mut device, fake) = open_device(&[0x21]);
    let writes = fake.frames().len();
    for (key, value, group) in [
        (Key::LimitSamples, Value::U64(1023), None),
        (Key::Samplerate, Value::U64(3_000_000), None),
        (Key::Samplerate, Value::U64((1 << 32) + 100), None),
        (Key::TriggerSource, Value::from("SPI"), None),
        (Key::HorizTriggerpos, Value::F64(1.5), None),
        (Key::HorizTriggerpos, Value::F64(-10.5), None),
        (Key::HorizTriggerpos, Value::F64(f64::NAN), None),
        (Key::Coupling, Value::from("GND"), Some("DSO")),
        (Key::TriggerSlope, Value::from("T->F"), Some("DSO")),
        (Key::TriggerSlope, Value::from("Rising"), Some("LA")),
        (Key::LogicThreshold, Value::from("5V"), Some("LA")),
        (Key::ProbeFactor, Value::U64(0), Some("DSO")),
        (Key::ProbeFactor, Value::U64(65_536), Some("DSO")),
        (Key::ProbeFactor, Value::U64(65_537), Some("DSO")),
        (Key::TriggerLevel, Value::F64(f64::NAN), Some("DSO")),
        (Key::Offset, Value::F64(f64::NAN), Some("DSO")),
    ] {
        assert!(device.set(&key, &value, group).is_err(), "{key:?} {value:?}");
    }
    assert_eq!(fake.frames().len(), writes);
    assert_eq!(device.get(&Key::Samplerate, None).unwrap().as_u64(), Some(10_000));
    assert_eq!(device.get(&Key::ProbeFactor, Some("DSO")).unwrap().as_u64(), Some(10));
}

#[test]
fn a_setting_is_kept_and_a_rate_goes_out_at_once() {
    let (mut device, fake) = open_device(&[0x21]);
    device.set(&Key::Samplerate, &Value::U64(200_000_000), None).unwrap();
    assert_eq!(fake.count(&[word(9, 0x02), word(10, 0x05)]), 1);
    assert_eq!(device.get(&Key::Samplerate, None).unwrap().as_u64(), Some(200_000_000));
    device.set(&Key::ProbeFactor, &Value::U64(65_535), Some("DSO")).unwrap();
    assert_eq!(device.get(&Key::ProbeFactor, Some("DSO")).unwrap().as_u64(), Some(65_535));
    device.set(&Key::LimitSamples, &Value::U64(1024), None).unwrap();
}

#[test]
fn a_setting_on_a_closed_device_with_a_rate_says_so() {
    let fake = Fake::new(&[]);
    let mut device = MsoDevice::from_port(Box::new(fake), SERIAL).unwrap();
    assert!(device.set(&Key::Samplerate, &Value::U64(100), None).is_err());
    assert_eq!(device.get(&Key::Samplerate, None).unwrap().as_u64(), Some(10_000));
}

#[test]
fn a_trigger_goes_to_the_logic_registers_by_channel() {
    let (mut device, _fake) = open_device(&[0x21]);
    let mut trigger = Trigger::new();
    trigger.add(0, 0, TriggerMatch::One);
    trigger.add(0, 3, TriggerMatch::Zero);
    trigger.add(0, 7, TriggerMatch::One);
    device.set_trigger(Some(trigger)).unwrap();
    assert_eq!(device.trigger_registers(), (0x81, 0x76));
    device.set_trigger(None).unwrap();
    assert_eq!(device.trigger_registers(), (0, 0xff));
}

#[test]
fn a_trigger_on_a_disabled_channel_is_left_out_and_dso_is_not_logic_4() {
    let (mut device, _fake) = open_device(&[0x21]);
    let mut trigger = Trigger::new();
    trigger.add(0, 4, TriggerMatch::One);
    device.set_trigger(Some(trigger)).unwrap();
    assert_eq!(device.trigger_registers(), (0x10, 0xef));
    // Position 5 is logic 4; DSO, at 0, has the same index and stays on.
    device.enable_channel(5, false).unwrap();
    assert_eq!(device.trigger_registers(), (0, 0xff));
}

#[test]
fn channels_are_as_c_lists_them() {
    let (device, _fake) = open_device(&[0x21]);
    let names: Vec<_> = device.channels().iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["DSO", "0", "1", "2", "3", "4", "5", "6", "7"]);
    assert_eq!(device.channels()[0].index, 4);
    assert_eq!(device.channels()[5].index, 4);
}

fn usb_port(vid: u16, pid: u16, number: Option<&str>) -> serial::UsbPort {
    serial::UsbPort {
        name: "/dev/ttyACM0".to_owned(),
        vid,
        pid,
        serial: number.map(str::to_owned),
        manufacturer: Some("Maker".to_owned()),
        product: None,
    }
}

#[test]
fn a_port_is_a_device_by_its_ids_and_its_number() {
    assert!(device_of(&usb_port(0x3195, 0xf191, Some(SERIAL))).is_none());
    assert!(device_of(&usb_port(0x3196, 0xf190, Some(SERIAL))).is_none());
    assert!(device_of(&usb_port(0x3195, 0xf190, None)).is_none());
    assert!(device_of(&usb_port(0x3195, 0xf190, Some("x"))).is_none());
    let device = device_of(&usb_port(0x3195, 0xf190, Some(SERIAL))).unwrap();
    assert_eq!(device.info().vendor, "Maker");
    assert_eq!(device.info().model, "");
    assert_eq!(device.info().version, "r2");
    assert_eq!(device.info().serial, SERIAL);
    assert_eq!(device.info().conn, None);
}

#[test]
fn conn_picks_a_port_by_its_name() {
    let ports = vec![usb_port(1, 2, None)];
    assert_eq!(find_ports(ports.clone(), None).len(), 1);
    assert_eq!(find_ports(ports.clone(), Some("/dev/ttyACM0")).len(), 1);
    assert_eq!(find_ports(ports, Some("/dev/ttyACM1")).len(), 0);
}

#[test]
fn settling_waits_for_the_line_and_only_when_something_is_out() {
    let fake = Fake::new(&[]);
    let mut port = fake.clone();
    fake.line().coming.extend([1, 2, 3, 4]);
    fake.set_chunk(1);
    // Nothing out: it does not read.
    let flag = AtomicBool::new(false);
    settle(&mut port, &flag, Duration::from_secs(5)).unwrap();
    assert_eq!(fake.line().reads, 0);
    let flag = AtomicBool::new(true);
    settle(&mut port, &flag, BOUND).unwrap();
    assert!(fake.line().coming.is_empty());
    assert!(fake.line().arrived.is_empty());
    assert!(!flag.load(Ordering::SeqCst));
}

#[test]
fn settling_has_a_bound() {
    let fake = Fake::new(&[]);
    let mut port = fake.clone();
    fake.line().coming.extend(std::iter::repeat_n(0, 100_000));
    fake.set_chunk(1);
    fake.line().pause = Duration::from_millis(1);
    let flag = AtomicBool::new(true);
    let at = Instant::now();
    settle(&mut port, &flag, Duration::from_millis(50)).unwrap();
    assert!(at.elapsed() >= Duration::from_millis(50));
    assert!(at.elapsed() < BOUND);
    assert!(!fake.line().coming.is_empty());
}

#[test]
fn the_times_are_as_stated() {
    assert_eq!(STATUS_TIMEOUT, Duration::from_millis(100));
    assert_eq!(POLL_READ, Duration::from_millis(10));
    assert_eq!(QUIET, Duration::from_millis(20));
    assert_eq!(SETTLE_BOUND, Duration::from_secs(2));
}

#[test]
fn a_poll_is_paced_by_c_s_read() {
    bounded(|| {
        let (mut device, fake) = open_device(&[0x21, 0x33, 0x33, 0x33, 0x36]);
        let (tx, rx) = sink(16);
        let at = Instant::now();
        device.start(tx).unwrap();
        packets(&rx);
        // Least: each of the three statuses the loop reads has its 10 ms. A loaded
        // host can only stretch this.
        assert!(at.elapsed() >= Duration::from_millis(30), "{:?}", at.elapsed());
        assert_eq!(fake.count(&read_status_request()), 5);
    });
}
