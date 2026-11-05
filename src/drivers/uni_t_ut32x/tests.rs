//! Captures on a fake meter: bytes come by the count of reads, not at a time, and
//! each test that can hang runs under its own bound.

#![allow(clippy::float_cmp, clippy::assert_is_empty)]

use super::*;
use crate::core::{MqFlags, Unit};
use crate::hid::SharedFake;
use crate::serial::hid::{Chip, HidPort};
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver as Rx, sync_channel};

const T1: &[u8; 19] = b"2:235100012300001\r\n";
const T2: &[u8; 19] = b"2:241100012301001\r\n";
const NO_PROBE: &[u8; 19] = b"2;;;;100012300001\r\n";
const BOUND: Duration = Duration::from_secs(30);
/// A byte's time at 2400/8n1.
const BYTE_TIME: Duration = Duration::from_micros(4167);

type Received = Rx<(usize, Packet)>;

#[derive(Default)]
struct Meter {
    /// What is on its way, in order.
    line: VecDeque<u8>,
    /// Bytes a read gives at most; 0 is 19.
    per_read: usize,
    /// Reads to wait before anything comes, once.
    hold: usize,
    /// Nothing comes before this.
    hold_until: Option<Instant>,
    /// Go on for ever with this, once `01` has come and until `02`.
    stream: Vec<u8>,
    /// How far into `stream`.
    at: usize,
    running: bool,
    /// What a `02` leaves on the way: this much of the stream.
    tail_after_stop: usize,
    /// Each read takes a byte's time, for the line's real rate.
    paced: bool,
    writes: Vec<u8>,
    reads: usize,
    fail_read_after: Option<usize>,
    fail_write: bool,
}

#[derive(Clone, Default)]
struct Fake(Arc<Mutex<Meter>>);

impl Fake {
    fn lock(&self) -> std::sync::MutexGuard<'_, Meter> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn with(records: &[&[u8]]) -> Fake {
        let f = Fake::default();
        f.lock().line = records.concat().into();
        f
    }

    fn writes(&self) -> Vec<u8> {
        self.lock().writes.clone()
    }

    fn left(&self) -> usize {
        self.lock().line.len()
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
        let (n, paced) = {
            let mut m = self.lock();
            m.reads += 1;
            if m.fail_read_after.is_some_and(|f| m.reads > f) {
                return Err(Error::device("gone"));
            }
            if m.hold > 0 {
                m.hold -= 1;
                (0, false)
            } else if m.hold_until.is_some_and(|t| Instant::now() < t) {
                (0, false)
            } else {
                if m.running && m.line.is_empty() && !m.stream.is_empty() {
                    let b = m.stream[m.at];
                    m.at = (m.at + 1) % m.stream.len();
                    m.line.push_back(b);
                }
                let most = if m.per_read == 0 { 19 } else { m.per_read };
                let n = buf.len().min(most).min(m.line.len());
                for b in &mut buf[..n] {
                    *b = m.line.pop_front().expect("counted");
                }
                (n, m.paced)
            }
        };
        if n == 0 {
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
        } else if paced {
            std::thread::sleep(BYTE_TIME * n as u32);
        }
        Ok(n)
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut m = self.lock();
        m.writes.extend_from_slice(data);
        for c in data {
            match *c {
                CMD_GET_LIVE | CMD_GET_STORED => {
                    m.running = true;
                    m.at = 0;
                }
                CMD_STOP => {
                    m.running = false;
                    for _ in 0..m.tail_after_stop {
                        let b = m.stream[m.at];
                        m.at = (m.at + 1) % m.stream.len();
                        m.line.push_back(b);
                    }
                }
                _ => {}
            }
        }
        if m.fail_write {
            return Err(Error::device("no way out"));
        }
        Ok(())
    }

    fn purge(&mut self) -> Result<()> {
        Ok(())
    }
}

fn device(fake: &Fake) -> Ut32xDevice {
    let mut d = Ut32xDevice::new("fake".to_owned(), Params::parse("2400/8n1").unwrap());
    *d.slot() = Some(Box::new(fake.clone()));
    d.is_open = true;
    d
}

fn start(d: &mut Ut32xDevice) -> Received {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// Runs a test that can hang on its own thread, and fails if it does.
fn within(test: impl FnOnce() + Send + 'static) {
    let (done, finished) = sync_channel(1);
    let thread = std::thread::spawn(move || {
        test();
        let _ = done.send(());
    });
    if finished.recv_timeout(BOUND).is_err() {
        assert!(thread.is_finished(), "the test did not end in {BOUND:?}");
        thread.join().expect("the test failed");
    }
}

/// The analog packets up to `End`, which must come within the bound.
fn capture(rx: &Received) -> Vec<AnalogData> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left).expect("no End in time").1 {
            Packet::Analog(a) => out.push(a),
            Packet::End => return out,
            _ => {}
        }
    }
}

fn values(rx: &Received) -> Vec<f32> {
    capture(rx).iter().map(|a| a.samples[0]).collect()
}

/// Until `what` holds, within the bound.
fn wait_for(what: &str, mut holds: impl FnMut() -> bool) {
    let deadline = Instant::now() + BOUND;
    while !holds() {
        assert!(Instant::now() < deadline, "{what}: not in time");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn set_samples(d: &mut Ut32xDevice, n: u64) {
    d.set(&Key::LimitSamples, &Value::U64(n), None).unwrap();
}

fn set_msec(d: &mut Ut32xDevice, n: u64) {
    d.set(&Key::LimitMsec, &Value::U64(n), None).unwrap();
}

#[test]
fn the_literal_bounds() {
    assert_eq!(TIME_GRACE, Duration::from_secs(2));
    assert_eq!(SILENCE_BOUND, Duration::from_secs(5));
    assert_eq!(DEFAULT_SERIALCOMM, "2400/8n1");
    assert_eq!((CMD_GET_LIVE, CMD_STOP, CMD_GET_STORED), (1, 2, 7));
}

#[test]
fn a_scan_needs_a_conn_and_a_port_that_opens() {
    assert!(Ut32xDriver.scan(&[]).unwrap().is_empty());
    let conn = |c: &str| (Key::Conn, Value::Str(c.to_owned()));
    assert!(Ut32xDriver.scan(&[conn("/dev/nothing-here")]).unwrap().is_empty());
    let bad = (Key::Serialcomm, Value::Str("fast".to_owned()));
    assert!(Ut32xDriver.scan(&[conn("/dev/nothing-here"), bad]).unwrap().is_empty());
    assert!(Ut32xDriver.scan(&[(Key::LimitSamples, Value::U64(1))]).is_err());
}

#[test]
fn the_settings() {
    let mut d = Ut32xDevice::new("fake".to_owned(), Params::parse("2400/8n1").unwrap());
    assert_eq!(d.get(&Key::DataSource, None).unwrap().as_str(), Some("Live"));
    d.set(&Key::DataSource, &Value::Str("Memory".to_owned()), None).unwrap();
    assert_eq!(d.get(&Key::DataSource, None).unwrap().as_str(), Some("Memory"));
    assert!(d.set(&Key::DataSource, &Value::Str("memory".to_owned()), None).is_err());
    assert_eq!(d.get(&Key::DataSource, None).unwrap().as_str(), Some("Memory"));
    set_samples(&mut d, 7);
    set_msec(&mut d, 70);
    assert_eq!(d.get(&Key::LimitSamples, None).unwrap().as_u64(), Some(7));
    assert_eq!(d.get(&Key::LimitMsec, None).unwrap().as_u64(), Some(70));
    assert!(d.get(&Key::Samplerate, None).is_err());
    assert!(d.set(&Key::Samplerate, &Value::U64(1), None).is_err());
    assert!(d.list(&Key::Samplerate, None).is_err());
    assert!(
        matches!(d.list(&Key::DataSource, None).unwrap(), ValueList::Str(l) if l == DATA_SOURCES)
    );
    assert_eq!(d.channels().iter().map(|c| c.name.as_str()).collect::<Vec<_>>(), CHANNEL_NAMES);
    assert!(d.info().conn.is_none(), "as its --scan line");
}

#[test]
fn a_capture_is_the_command_the_records_and_the_stop() {
    within(|| {
        let fake = Fake::with(&[T1, T2]);
        let mut d = device(&fake);
        set_samples(&mut d, 2);
        let rx = start(&mut d);
        let got = capture(&rx);
        assert_eq!(fake.writes(), [1, 2]);
        assert_eq!(got.iter().map(|a| a.channels[0]).collect::<Vec<_>>(), [0, 1]);
        assert_eq!(got[0].samples, [23.5]);
        assert_eq!(got[0].digits, 1);
        assert_eq!(
            (got[0].mq, got[0].unit, got[0].flags),
            (Mq::Temperature, Unit::Celsius, MqFlags::empty())
        );
        d.close().unwrap();
    });
}

#[test]
fn the_header_comes_before_the_command() {
    within(|| {
        // A queue of one, unread: the header fills it, and the write follows at once.
        let fake = Fake::with(&[T1]);
        let mut d = device(&fake);
        set_samples(&mut d, 1);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        wait_for("the command", || !fake.writes().is_empty());
        assert_eq!(fake.writes()[0], 1);
        capture(&rx);
    });
}

#[test]
fn the_memory_is_asked_for_with_its_own_command_and_free_slots_count() {
    within(|| {
        let fake = Fake::with(&[T1, NO_PROBE, NO_PROBE, T2, T1]);
        let mut d = device(&fake);
        d.set(&Key::DataSource, &Value::Str("Memory".to_owned()), None).unwrap();
        set_samples(&mut d, 4);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [23.5, 24.1]);
        assert_eq!(fake.writes(), [7, 2]);
        assert_eq!(fake.left(), 19, "the one behind the limit is not read");
    });
}

#[test]
fn what_is_no_record_does_not_count() {
    within(|| {
        let mut other = *T1;
        other[8] = b'x';
        let fake = Fake::with(&[&other, b"short\r\n", T2, T1]);
        let mut d = device(&fake);
        set_samples(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [24.1]);
        assert_eq!(fake.left(), 19);
    });
}

#[test]
fn a_channel_that_is_off_gets_nothing_and_its_records_count() {
    within(|| {
        let fake = Fake::with(&[T1, T2, T1]);
        let mut d = device(&fake);
        d.enable_channel(0, false).unwrap();
        set_samples(&mut d, 2);
        let rx = start(&mut d);
        let got = capture(&rx);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].channels, [1]);
        assert_eq!(fake.left(), 19, "the limit came on the second");
    });
}

#[test]
fn a_time_limit_ends_on_the_record_behind_it() {
    within(|| {
        // Released on the read count, well past the limit of 1 ms.
        let fake = Fake::with(&[T1, T2]);
        fake.lock().hold = 200;
        let mut d = device(&fake);
        set_msec(&mut d, 1);
        d.bounds.time_grace = BOUND;
        let rx = start(&mut d);
        assert_eq!(values(&rx), [23.5], "that one is sent, as in C, and no other");
        assert_eq!(fake.writes(), [1, 2]);
    });
}

#[test]
fn a_silent_time_limit_ends_after_its_grace_and_not_before() {
    within(|| {
        let fake = Fake::default();
        let mut d = device(&fake);
        set_msec(&mut d, 200);
        d.bounds.time_grace = Duration::from_millis(300);
        d.bounds.silence = Duration::from_millis(1);
        let began = Instant::now();
        let rx = start(&mut d);
        assert!(capture(&rx).is_empty());
        assert!(began.elapsed() >= Duration::from_millis(500), "{:?}", began.elapsed());
        assert_eq!(fake.writes(), [1, 2], "with the stop");
    });
}

#[test]
fn a_silent_sample_limit_ends_after_the_bound_and_not_before() {
    within(|| {
        let fake = Fake::default();
        let mut d = device(&fake);
        set_samples(&mut d, 1);
        d.bounds.silence = Duration::from_millis(300);
        let began = Instant::now();
        let rx = start(&mut d);
        assert!(capture(&rx).is_empty());
        assert!(began.elapsed() >= Duration::from_millis(300), "{:?}", began.elapsed());
        assert_eq!(fake.writes(), [1, 2]);
    });
}

#[test]
fn noise_does_not_hold_off_the_bound() {
    within(|| {
        let fake = Fake::default();
        {
            let mut m = fake.lock();
            m.stream = b"no\r\n".to_vec();
        }
        let mut d = device(&fake);
        set_samples(&mut d, 1);
        d.bounds.silence = Duration::from_millis(200);
        let rx = start(&mut d);
        assert!(capture(&rx).is_empty());
        assert!(fake.lock().reads > 1);
    });
}

#[test]
fn a_record_starts_the_bound_over() {
    within(|| {
        // The record comes at 60% of the bound. A stall can put it behind the bound,
        // which says nothing, so a longer bound then goes again.
        for bound_ms in [500, 2000, 8000] {
            let bound = Duration::from_millis(bound_ms);
            let wait = bound * 6 / 10;
            let fake = Fake::with(&[T1]);
            let began = Instant::now();
            fake.lock().hold_until = Some(began + wait);
            let mut d = device(&fake);
            set_samples(&mut d, 2);
            d.bounds.silence = bound;
            let rx = start(&mut d);
            let got = values(&rx);
            if got.is_empty() {
                continue;
            }
            assert_eq!(got, [23.5]);
            assert!(began.elapsed() >= wait + bound, "{:?}", began.elapsed());
            return;
        }
        panic!("stalled at every bound");
    });
}

#[test]
fn with_no_limit_nothing_but_a_stop_ends_it() {
    within(|| {
        let fake = Fake::default();
        let mut d = device(&fake);
        d.bounds.time_grace = Duration::from_millis(1);
        d.bounds.silence = Duration::from_millis(1);
        let rx = start(&mut d);
        wait_for("reads", || fake.lock().reads > 50);
        assert!(!matches!(
            std::iter::from_fn(|| rx.try_recv().ok()).last().map(|p| p.1),
            Some(Packet::End)
        ));
        d.stop().unwrap();
        capture(&rx);
        assert_eq!(fake.writes(), [1, 2]);
    });
}

#[test]
fn a_stop_ends_a_capture_on_a_meter_that_keeps_sending_and_sends_the_02() {
    within(|| {
        let fake = Fake::default();
        {
            let mut m = fake.lock();
            m.stream = T1.to_vec();
            m.per_read = 1;
        }
        let mut d = device(&fake);
        let rx = start(&mut d);
        let first = rx.recv_timeout(BOUND).unwrap();
        assert!(matches!(first.1, Packet::Header { .. }));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)));
        d.stop().unwrap();
        capture(&rx);
        assert_eq!(fake.writes(), [1, 2]);
        d.close().unwrap();
    });
}

#[test]
fn a_stop_with_a_full_queue_and_a_close_do_not_hang() {
    within(|| {
        let fake = Fake::default();
        {
            let mut m = fake.lock();
            m.stream = T1.to_vec();
        }
        let mut d = device(&fake);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for("a read", || fake.lock().reads > 0);
        // Nothing reads the queue, and the thread is on a send.
        d.close().unwrap();
        assert_eq!(fake.writes(), [1, 2]);
        assert!(d.slot().is_none(), "the port is closed");
        drop(rx);
    });
}

#[test]
fn the_02_and_the_port_are_done_before_end() {
    within(|| {
        // Header and the sample fill the queue, so End cannot go.
        let fake = Fake::with(&[T1]);
        let mut d = device(&fake);
        set_samples(&mut d, 1);
        let (tx, rx) = sync_channel(2);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for("the port back", || d.slot().is_some());
        assert_eq!(fake.writes(), [1, 2], "before End can have gone");
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::End));
    });
}

#[test]
fn a_start_after_an_end_stuck_on_a_full_queue_does_not_hang() {
    within(|| {
        let fake = Fake::with(&[T1, T2]);
        let mut d = device(&fake);
        set_samples(&mut d, 1);
        let (tx, _unread) = sync_channel(2);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_for("the port back", || d.slot().is_some());
        let rx = start(&mut d);
        assert_eq!(values(&rx), [24.1], "the next record");
        assert_eq!(fake.writes(), [1, 2, 1, 2]);
    });
}

#[test]
fn a_start_during_a_capture_is_refused_and_one_on_a_closed_device_too() {
    within(|| {
        let fake = Fake::default();
        let mut d = device(&fake);
        let rx = start(&mut d);
        let (tx, _rx2) = sync_channel(4);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.close().unwrap();
        capture(&rx);
        let (tx, _rx3) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
    });
}

/// A capture at the line's real rate that a limit ends, leaving `tail` bytes of the
/// next record on the way, and the capture behind it.
fn second_capture_after_a_tail(tail: usize) {
    let fake = Fake::default();
    {
        let mut m = fake.lock();
        m.stream = [T1.as_slice(), T2.as_slice()].concat();
        m.per_read = 1;
        m.paced = true;
        m.tail_after_stop = tail;
    }
    let mut d = device(&fake);
    set_samples(&mut d, 1);
    let rx = start(&mut d);
    assert_eq!(values(&rx), [23.5]);
    wait_for("the port back", || d.slot().is_some());
    // A setting right behind it goes nowhere near the line.
    set_samples(&mut d, 2);
    let written = fake.writes().len();
    let rx = start(&mut d);
    // The tail is T2's head, and a new start sends T1 first: whole records only.
    let got = values(&rx);
    assert_eq!(got.len(), 2, "tail {tail}: {got:?}");
    for v in got {
        assert!(v == 23.5 || v == 24.1, "tail {tail}: a shifted value {v}");
    }
    assert_eq!(fake.writes().len(), written + 2);
    d.close().unwrap();
}

#[test]
fn a_tail_of_the_last_capture_shifts_nothing() {
    for tail in [18, 19, 20] {
        within(move || second_capture_after_a_tail(tail));
    }
}

#[test]
fn a_late_record_cut_at_any_length_is_passed_over() {
    // Without a meter's time: every tail of a record, then whole ones. It takes at
    // most one whole record to be in step again.
    within(|| {
        for cut in 0..T1.len() {
            let fake = Fake::with(&[&T1[cut..], T2, T2, T2]);
            let mut d = device(&fake);
            set_samples(&mut d, 2);
            let rx = start(&mut d);
            let got = values(&rx);
            assert_eq!(got.len(), 2, "cut {cut}");
            assert!(
                got.iter().all(|v| *v == 24.1 || (cut == 0 && *v == 23.5)),
                "cut {cut}: {got:?}"
            );
        }
    });
}

#[test]
fn a_failed_write_is_taken_as_c_takes_it() {
    within(|| {
        let fake = Fake::with(&[T1]);
        fake.lock().fail_write = true;
        let mut d = device(&fake);
        set_samples(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [23.5], "the capture goes on, and ends");
        assert_eq!(fake.writes(), [1, 2]);
    });
}

#[test]
fn a_failed_read_ends_the_capture() {
    within(|| {
        let fake = Fake::with(&[T1]);
        fake.lock().per_read = 1;
        fake.lock().fail_read_after = Some(25);
        let mut d = device(&fake);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [23.5]);
        assert_eq!(fake.writes(), [1, 2]);
        wait_for("the port back", || d.slot().is_some());
    });
}

#[test]
fn a_limit_set_during_a_capture_bites_at_the_next_record() {
    within(|| {
        let fake = Fake::default();
        {
            let mut m = fake.lock();
            m.stream = T1.to_vec();
            m.per_read = 1;
            m.paced = true;
        }
        let mut d = device(&fake);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)));
        set_samples(&mut d, 2);
        capture(&rx);
        assert_eq!(fake.writes(), [1, 2]);
    });
}

#[test]
fn on_the_cable_the_bytes_behind_the_count_are_zeros() {
    // C leaves them uninitialised (`serial_hid_ch9325.c:121,139-141`), and the
    // emulator cases mask them.
    let hid = SharedFake::default();
    let mut port = HidPort::open(
        Chip::Ch9325,
        Box::new(hid.clone()),
        "hid/ch9325".to_owned(),
        Params::parse("2400/8n1").unwrap(),
    )
    .unwrap();
    port.write(&[CMD_GET_LIVE]).unwrap();
    port.write(&[CMD_STOP]).unwrap();
    assert_eq!(hid.output(), [vec![1, 1, 0, 0, 0, 0, 0, 0], vec![1, 2, 0, 0, 0, 0, 0, 0]]);
    assert_eq!(hid.features(), [vec![0, 0x60, 0x09, 0, 0, 3]]);
}
