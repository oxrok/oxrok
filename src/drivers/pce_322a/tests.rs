// Counterpart in sigrok:
//   libsigrok/src/hardware/pce-322a/protocol.c,
//   Copyright (C) 2016 George Hopkins <george-hopkins@null.net>,
//   Copyright (C) 2016 Matthieu Guillaumin <matthieu@guillaum.in>,
//   with commits by Gerhard Sittig, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The capture on a fake [`Port`]: limits, stops, what one capture leaves for the
//! next, and the waits, which an emulator model cannot check.

use super::*;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test waits before it fails: a stop or a limit that does not bite
/// must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);

fn frame(value: u16, flags: u8, mode: u8) -> Vec<u8> {
    let [high, low] = value.to_be_bytes();
    vec![0x7f, high, low, flags, mode, 0, 0x26, 0x11, 0x05, 0x12, 0x30, 0, 0]
}

fn header(weight: u8) -> Vec<u8> {
    vec![0x7f, 0xf7, 0x01, 0x00, weight, 0x26, 0x11, 0x05, 0x12, 0x00, 0x30]
}

fn status(used: u16, last: u8) -> Vec<u8> {
    let [high, low] = used.to_be_bytes();
    vec![0xd1, 0x05, 0x00, 0x01, 0xd2, high, low, last, 0x20]
}

#[derive(Default)]
struct Meter {
    writes: Vec<Vec<u8>>,
    /// On the way: they arrive at `per_read` bytes a read, a stand-in for the line.
    coming: VecDeque<u8>,
    /// Arrived: the bytes a purge drops, and only these.
    arrived: VecDeque<u8>,
    per_read: usize,
    /// A byte arrives with every read, for ever, as a meter gone wrong might send.
    noise: bool,
    /// Frames the stream has left, each after `period` reads while connected.
    stream: VecDeque<Vec<u8>>,
    period: usize,
    reads_since_frame: usize,
    connected: bool,
    /// What `ad da` gets. Nothing if none.
    status: Option<Vec<u8>>,
    /// What each `d3 da` gets, by number.
    blocks: Vec<Vec<u8>>,
    reads: usize,
    read_fails: bool,
    /// Writes are refused from this count on.
    fail_writes_after: Option<usize>,
    purges: usize,
}

/// A meter: the test sets what comes, and sees what goes. It holds a lock only for
/// the moment of a call.
#[derive(Clone)]
struct Fake(Arc<Mutex<Meter>>);

impl Fake {
    fn new() -> Fake {
        Fake(Arc::new(Mutex::new(Meter {
            per_read: 256,
            period: 1,
            connected: true,
            ..Meter::default()
        })))
    }

    fn lock(&self) -> MutexGuard<'_, Meter> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn stream(&self, frames: impl IntoIterator<Item = Vec<u8>>) {
        self.lock().stream.extend(frames);
    }

    fn writes(&self) -> Vec<Vec<u8>> {
        self.lock().writes.clone()
    }

    fn device(&self) -> Pce322aDevice {
        let mut device = Pce322aDevice::from_port(Box::new(self.clone()));
        device.waits = Waits {
            silence: Duration::from_secs(600),
            poll: Duration::from_millis(1),
            quiet: Duration::from_millis(20),
            quiet_bound: Duration::from_secs(5),
        };
        device.open().unwrap();
        device
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
    fn read(&mut self, buf: &mut [u8], _: Duration) -> Result<usize> {
        let mut m = self.lock();
        if m.read_fails {
            return Err(Error::device("Input/output error"));
        }
        m.reads += 1;
        m.reads_since_frame += 1;
        if m.connected && m.reads_since_frame >= m.period && !m.stream.is_empty() {
            let f = m.stream.pop_front().expect("not empty");
            m.coming.extend(f);
            m.reads_since_frame = 0;
        }
        if m.noise {
            m.arrived.push_back(0x55);
        }
        for _ in 0..m.per_read {
            let Some(b) = m.coming.pop_front() else { break };
            m.arrived.push_back(b);
        }
        let mut n = 0;
        while n < buf.len() {
            let Some(b) = m.arrived.pop_front() else { break };
            buf[n] = b;
            n += 1;
        }
        drop(m);
        if n == 0 {
            std::thread::sleep(Duration::from_millis(1));
        }
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut m = self.lock();
        if m.fail_writes_after.is_some_and(|n| m.writes.len() >= n) {
            return Err(Error::device("Broken pipe"));
        }
        m.writes.push(data.to_vec());
        match data {
            [0xac, 0xff] => m.connected = true,
            [0xca, 0xff] => m.connected = false,
            [0xad, 0xda] => {
                if let Some(status) = m.status.clone() {
                    m.coming.extend(status);
                }
            }
            [0xd3, 0xda, low, high] => {
                let n = usize::from(u16::from_le_bytes([*low, *high]));
                if let Some(block) = m.blocks.get(n).cloned() {
                    m.coming.extend(block);
                }
            }
            _ => {}
        }
        Ok(())
    }
    fn purge(&mut self) -> Result<()> {
        let mut m = self.lock();
        m.arrived.clear();
        m.purges += 1;
        Ok(())
    }
}

/// Runs a test's body on its own thread, so that what hangs fails.
fn bounded(body: impl FnOnce() + Send + 'static) {
    let (tx, rx) = sync_channel(1);
    let handle = std::thread::spawn(move || {
        body();
        let _ = tx.send(());
    });
    match rx.recv_timeout(BOUND) {
        Ok(()) => handle.join().unwrap(),
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(handle.join().unwrap_err())
        }
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("hung for {BOUND:?}"),
    }
}

fn collect(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut packets = Vec::new();
    while Instant::now() < deadline {
        let Ok((_, packet)) = rx.recv_timeout(Duration::from_millis(50)) else { continue };
        let end = matches!(packet, Packet::End);
        packets.push(packet);
        if end {
            return packets;
        }
    }
    panic!("no End within {BOUND:?}, after {} packets", packets.len());
}

fn run(device: &mut Pce322aDevice) -> Vec<Packet> {
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    collect(&rx)
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

fn limit(device: &mut Pce322aDevice, n: u64) {
    device.set(&Key::LimitSamples, &Value::U64(n), None).unwrap();
}

fn memory_source(device: &mut Pce322aDevice) {
    device.set(&Key::DataSource, &Value::Str("Memory".to_owned()), None).unwrap();
}

/// Waits, with a bound, for a condition on the fake.
fn wait_for(fake: &Fake, what: &str, done: impl Fn(&Meter) -> bool) {
    let deadline = Instant::now() + BOUND;
    while !done(&fake.lock()) {
        assert!(Instant::now() < deadline, "no {what} within {BOUND:?}");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn get_str(device: &Pce322aDevice, key: &Key) -> String {
    device.get(key, None).unwrap().as_str().unwrap().to_owned()
}

#[test]
fn the_constants_are_what_the_module_says() {
    assert_eq!(SILENCE_BOUND, Duration::from_secs(5));
    assert_eq!(POLL, Duration::from_millis(150));
    assert_eq!(QUIET, Duration::from_millis(50));
    assert_eq!(QUIET_BOUND, Duration::from_secs(1));
    assert_eq!(END_AFTER_STOP, Duration::from_secs(1));
}

#[test]
fn a_scan_without_conn_finds_nothing() {
    assert!(Pce322aDriver.scan(&[]).unwrap().is_empty());
}

#[test]
fn a_scan_of_a_port_that_does_not_open_finds_nothing() {
    let options = [(Key::Conn, Value::Str("/dev/oxrok-no-such-port".to_owned()))];
    assert!(Pce322aDriver.scan(&options).unwrap().is_empty());
}

#[test]
fn an_open_connects_and_a_close_disconnects() {
    let fake = Fake::new();
    let mut device = fake.device();
    assert_eq!(fake.writes(), [vec![0xac, 0xff]]);
    device.close().unwrap();
    assert_eq!(fake.writes(), [vec![0xac, 0xff], vec![0xca, 0xff]]);
    device.close().unwrap();
    assert_eq!(fake.writes().len(), 2, "no port, nothing to say");
}

#[test]
fn a_failed_connect_fails_the_open_and_holds_no_port() {
    let fake = Fake::new();
    fake.lock().fail_writes_after = Some(0);
    let mut device = Pce322aDevice::from_port(Box::new(fake.clone()));
    assert!(device.open().is_err());
    assert!(device.get(&Key::PowerOff, None).is_ok());
    let (tx, _rx) = sync_channel(1);
    assert!(device.start(PacketSink::new(tx, 0)).is_err());
}

#[test]
fn settings_toggle_only_when_they_change_something() {
    let fake = Fake::new();
    let mut device = fake.device();
    let str_value = |s: &str| Value::Str(s.to_owned());
    device.set(&Key::SplWeightFreq, &str_value("A"), None).unwrap();
    device.set(&Key::SplWeightTime, &str_value("F"), None).unwrap();
    device.set(&Key::SplMeasurementRange, &Value::RangeU64(30, 130), None).unwrap();
    assert_eq!(fake.writes().len(), 1, "only the connect");
    device.set(&Key::SplWeightFreq, &str_value("C"), None).unwrap();
    device.set(&Key::SplWeightFreq, &str_value("C"), None).unwrap();
    device.set(&Key::SplWeightTime, &str_value("S"), None).unwrap();
    device.set(&Key::SplWeightTime, &str_value("S"), None).unwrap();
    assert_eq!(fake.writes()[1..], [vec![0xaa, 0xf1], vec![0xaa, 0xf4]]);
    assert_eq!(get_str(&device, &Key::SplWeightFreq), "C");
    assert_eq!(get_str(&device, &Key::SplWeightTime), "S");
    device.set(&Key::SplWeightFreq, &str_value("A"), None).unwrap();
    device.set(&Key::SplWeightTime, &str_value("F"), None).unwrap();
    assert_eq!(fake.writes()[3..], [vec![0xaa, 0xf1], vec![0xaa, 0xf4]]);
}

#[test]
fn a_range_steps_from_where_the_last_setting_left_it() {
    let fake = Fake::new();
    let mut device = fake.device();
    let toggle = vec![0xaa, 0xf2];
    device.set(&Key::SplMeasurementRange, &Value::RangeU64(30, 80), None).unwrap();
    assert_eq!(fake.writes()[1..], [vec![0xaa, 0xf2]]);
    device.set(&Key::SplMeasurementRange, &Value::RangeU64(50, 100), None).unwrap();
    assert_eq!(fake.writes()[1..], [toggle.clone(), toggle.clone()]);
    assert_eq!(device.get(&Key::SplMeasurementRange, None).unwrap(), Value::RangeU64(50, 100));
    // Down means round: 2 to 1 is three steps.
    device.set(&Key::SplMeasurementRange, &Value::RangeU64(30, 80), None).unwrap();
    assert_eq!(fake.writes().len(), 1 + 2 + 3);
    assert_eq!(device.get(&Key::SplMeasurementRange, None).unwrap(), Value::RangeU64(30, 80));
}

#[test]
fn a_range_stops_at_the_first_failed_toggle_and_knows_how_far_it_got() {
    let fake = Fake::new();
    let mut device = fake.device();
    // The connect and one toggle.
    fake.lock().fail_writes_after = Some(2);
    assert!(device.set(&Key::SplMeasurementRange, &Value::RangeU64(80, 130), None).is_err());
    assert_eq!(fake.writes().len(), 2);
    assert_eq!(device.get(&Key::SplMeasurementRange, None).unwrap(), Value::RangeU64(30, 80));
}

#[test]
fn a_failed_toggle_is_an_error_and_changes_nothing() {
    let fake = Fake::new();
    let mut device = fake.device();
    fake.lock().fail_writes_after = Some(1);
    assert!(device.set(&Key::SplWeightFreq, &Value::Str("C".to_owned()), None).is_err());
    assert!(device.set(&Key::SplWeightTime, &Value::Str("S".to_owned()), None).is_err());
    assert!(device.set(&Key::PowerOff, &Value::Bool(true), None).is_err());
    assert_eq!(get_str(&device, &Key::SplWeightFreq), "A");
    assert_eq!(get_str(&device, &Key::SplWeightTime), "F");
}

#[test]
fn power_off_sends_only_when_true_and_reads_false() {
    let fake = Fake::new();
    let mut device = fake.device();
    device.set(&Key::PowerOff, &Value::Bool(false), None).unwrap();
    assert_eq!(fake.writes().len(), 1);
    device.set(&Key::PowerOff, &Value::Bool(true), None).unwrap();
    assert_eq!(fake.writes()[1], [0xaa, 0xf8]);
    assert_eq!(device.get(&Key::PowerOff, None).unwrap(), Value::Bool(false));
}

#[test]
fn what_no_list_holds_is_refused() {
    let fake = Fake::new();
    let mut device = fake.device();
    let s = |v: &str| Value::Str(v.to_owned());
    assert!(device.set(&Key::SplWeightFreq, &s("Z"), None).is_err());
    assert!(device.set(&Key::SplWeightTime, &s("X"), None).is_err());
    assert!(device.set(&Key::DataSource, &s("Tape"), None).is_err());
    assert!(device.set(&Key::SplMeasurementRange, &Value::RangeU64(30, 100), None).is_err());
    assert!(device.set(&Key::Continuous, &Value::Bool(true), None).is_err());
    assert_eq!(fake.writes().len(), 1, "nothing went out");
}

#[test]
fn the_lists_are_cs() {
    let device = Fake::new().device();
    let strings = |v: &[&str]| ValueList::Str(v.iter().map(|&s| s.to_owned()).collect());
    assert_eq!(device.list(&Key::SplWeightFreq, None).unwrap(), strings(&["A", "C"]));
    assert_eq!(device.list(&Key::SplWeightTime, None).unwrap(), strings(&["F", "S"]));
    assert_eq!(device.list(&Key::DataSource, None).unwrap(), strings(&["Live", "Memory"]));
    assert_eq!(
        device.list(&Key::SplMeasurementRange, None).unwrap(),
        ValueList::RangeU64(vec![(30, 130), (30, 80), (50, 100), (80, 130)])
    );
}

#[test]
fn a_limit_ends_a_live_capture_and_what_follows_stays_unread() {
    bounded(|| {
        let fake = Fake::new();
        fake.stream([frame(654, 0, 0), frame(655, 0, 0), frame(656, 0, 0)]);
        let mut device = fake.device();
        limit(&mut device, 2);
        let packets = run(&mut device);
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert_eq!(values(&packets), [65.4, 65.5]);
        assert!(!fake.lock().stream.is_empty(), "the limit did not stop the reads");
    });
}

#[test]
fn a_reading_has_the_meters_flags_and_the_gets_follow() {
    bounded(|| {
        let fake = Fake::new();
        fake.stream([frame(1000, 0x03, 0x06), frame(2000, 0x00, 0x09)]);
        let mut device = fake.device();
        limit(&mut device, 1);
        let packets = run(&mut device);
        let Packet::Analog(a) = &packets[1] else { panic!("{:?}", packets[1]) };
        assert_eq!(a.channels, [0]);
        assert_eq!(a.samples, [100.0]);
        assert_eq!(a.mq, Mq::SoundPressureLevel);
        assert_eq!(a.unit, Unit::DecibelSpl);
        assert_eq!(a.digits, 1);
        assert_eq!(
            a.flags,
            Mode { c_weight: true, slow: true, max: true, min: false, range: 2 }.flags()
        );
        assert_eq!(get_str(&device, &Key::SplWeightFreq), "C");
        assert_eq!(get_str(&device, &Key::SplWeightTime), "S");
        assert_eq!(device.get(&Key::SplMeasurementRange, None).unwrap(), Value::RangeU64(50, 100));
        // So a set now compares with what the meter said: no toggle.
        let before = fake.writes().len();
        device.set(&Key::SplWeightFreq, &Value::Str("C".to_owned()), None).unwrap();
        assert_eq!(fake.writes().len(), before);
        // The next frame's flags replace them all.
        let packets = run(&mut device);
        let Packet::Analog(a) = &packets[1] else { panic!("{:?}", packets[1]) };
        assert_eq!(a.flags, Mode { min: true, range: 1, ..Mode::default() }.flags());
    });
}

#[test]
fn each_capture_counts_its_samples_from_zero() {
    bounded(|| {
        let fake = Fake::new();
        fake.stream([frame(1, 0, 0), frame(2, 0, 0), frame(3, 0, 0), frame(4, 0, 0)]);
        let mut device = fake.device();
        limit(&mut device, 2);
        assert_eq!(values(&run(&mut device)).len(), 2);
        assert_eq!(values(&run(&mut device)).len(), 2, "where C ends at the first");
    });
}

#[test]
fn a_second_live_capture_drops_what_queued_up_and_the_first_does_not() {
    bounded(|| {
        let fake = Fake::new();
        let mut device = fake.device();
        // Arrived before the first capture: it counts, as C would read it.
        fake.lock().arrived.extend(frame(100, 0, 0));
        limit(&mut device, 1);
        assert_eq!(fake.lock().purges, 1, "the open's");
        assert_eq!(values(&run(&mut device)), [10.0], "a purge at the first capture");
        assert_eq!(fake.lock().purges, 1);
        // An old one, then a new.
        fake.lock().arrived.extend(frame(200, 0, 0));
        fake.stream([frame(300, 0, 0)]);
        assert_eq!(values(&run(&mut device)), [30.0]);
    });
}

#[test]
fn a_frame_across_reads_is_whole() {
    bounded(|| {
        let fake = Fake::new();
        fake.lock().per_read = 1;
        fake.stream([frame(654, 0, 0)]);
        let mut device = fake.device();
        limit(&mut device, 1);
        assert_eq!(values(&run(&mut device)), [65.4]);
    });
}

#[test]
fn a_stop_ends_a_capture_that_data_keeps_coming_to() {
    bounded(|| {
        let fake = Fake::new();
        fake.stream((0..100_000).map(|_| frame(654, 0, 0)));
        let mut device = fake.device();
        let (tx, rx) = sync_channel(16);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let mut seen = 0;
        while seen < 4 {
            if let (_, Packet::Analog(_)) = rx.recv_timeout(BOUND).unwrap() {
                seen += 1;
            }
        }
        device.stop().unwrap();
        collect(&rx);
        assert!(!fake.lock().stream.is_empty(), "it read them all");
    });
}

#[test]
fn a_stop_with_the_queue_full_ends_the_capture() {
    bounded(|| {
        let fake = Fake::new();
        fake.stream((0..1000).map(|_| frame(654, 0, 0)));
        let mut device = fake.device();
        let (tx, rx) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        // The first frame's reading waits on the queue, left unread: the close
        // joins whatever the thread is doing.
        wait_for(&fake, "a read", |m| m.reads >= 1);
        device.close().unwrap();
        assert_eq!(fake.writes().last().unwrap(), &[0xca, 0xff]);
        // Joined means the sink has gone: what is queued, and then nothing more can
        // come. A thread still there would put its End in the room this makes.
        assert!(matches!(rx.try_recv(), Ok((_, Packet::Header { .. }))));
        assert!(matches!(rx.try_recv(), Err(std::sync::mpsc::TryRecvError::Disconnected)));
    });
}

#[test]
fn a_start_after_a_capture_stuck_on_its_end_does_not_hang() {
    bounded(|| {
        let fake = Fake::new();
        fake.stream([frame(1, 0, 0), frame(2, 0, 0), frame(3, 0, 0)]);
        let mut device = fake.device();
        limit(&mut device, 1);
        let (tx, rx) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        // The header goes, so the reading fills the queue of one, and End waits.
        let (_, first) = rx.recv_timeout(BOUND).unwrap();
        assert!(matches!(first, Packet::Header { .. }));
        let deadline = Instant::now() + BOUND;
        let (tx2, rx2) = sync_channel(64);
        // Busy until the old capture has passed its last read: asked again.
        loop {
            match device.start(PacketSink::new(tx2.clone(), 0)) {
                Ok(()) => break,
                Err(Error::Busy) => {
                    assert!(Instant::now() < deadline, "still busy");
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(e) => panic!("{e}"),
            }
        }
        assert_eq!(values(&collect(&rx2)).len(), 1);
        drop(rx);
    });
}

#[test]
fn a_set_goes_out_during_a_capture_and_the_source_is_refused() {
    bounded(|| {
        let fake = Fake::new();
        let mut device = fake.device();
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        wait_for(&fake, "a read", |m| m.reads > 2);
        device.set(&Key::SplWeightFreq, &Value::Str("C".to_owned()), None).unwrap();
        assert_eq!(fake.writes().last().unwrap(), &[0xaa, 0xf1]);
        assert!(matches!(
            device.set(&Key::DataSource, &Value::Str("Memory".to_owned()), None),
            Err(Error::Busy)
        ));
        assert!(matches!(device.start(PacketSink::new(sync_channel(1).0, 0)), Err(Error::Busy)));
        device.stop().unwrap();
        collect(&rx);
    });
}

#[test]
fn a_read_error_ends_a_capture_without_a_limit() {
    bounded(|| {
        let fake = Fake::new();
        fake.lock().read_fails = true;
        let mut device = fake.device();
        assert_eq!(values(&run(&mut device)), [0.0_f32; 0]);
    });
}

#[test]
fn a_silent_meter_ends_a_limited_capture_at_the_bound_and_not_before() {
    bounded(|| {
        let fake = Fake::new();
        let mut device = fake.device();
        device.waits.silence = Duration::from_millis(300);
        limit(&mut device, 1);
        let began = Instant::now();
        assert_eq!(values(&run(&mut device)), [0.0_f32; 0]);
        assert!(began.elapsed() >= Duration::from_millis(300), "{:?}", began.elapsed());
    });
}

#[test]
fn a_meter_sending_only_junk_ends_a_limited_capture_at_the_bound() {
    bounded(|| {
        let fake = Fake::new();
        fake.lock().noise = true;
        let mut device = fake.device();
        device.waits.silence = Duration::from_millis(100);
        limit(&mut device, 1);
        assert_eq!(values(&run(&mut device)), [0.0_f32; 0]);
    });
}

#[test]
fn a_meter_slower_than_nothing_but_within_the_bound_is_waited_for() {
    bounded(|| {
        let fake = Fake::new();
        // A frame per 50 reads, each at least 1 ms, well within 5 s.
        fake.lock().period = 50;
        fake.stream((0..4).map(|i| frame(100 + i, 0, 0)));
        let mut device = fake.device();
        device.waits.silence = SILENCE_BOUND;
        limit(&mut device, 4);
        assert_eq!(values(&run(&mut device)).len(), 4);
    });
}

#[test]
fn with_no_limit_a_silent_meter_is_waited_for_until_a_stop() {
    bounded(|| {
        let fake = Fake::new();
        let mut device = fake.device();
        device.waits.silence = Duration::from_millis(1);
        let (tx, rx) = sync_channel(4);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let (_, first) = rx.recv_timeout(BOUND).unwrap();
        assert!(matches!(first, Packet::Header { .. }));
        // Far past the bound, by reads rather than by time.
        wait_for(&fake, "many reads", |m| m.reads > 50);
        assert!(rx.try_recv().is_err(), "it ended");
        device.stop().unwrap();
        collect(&rx);
    });
}

fn one_recording() -> Vec<u8> {
    let mut bytes = header(0x0a);
    bytes.extend_from_slice(&[0x8e, 0x02, 0x8f, 0x02]);
    bytes
}

#[test]
fn memory_reads_status_then_blocks_and_ends_by_itself() {
    bounded(|| {
        let fake = Fake::new();
        let mut block0 = one_recording();
        block0.resize(256, 0);
        // A header's 11 bytes make the samples cross into the next block: its first
        // byte is the last sample's.
        let mut second = vec![0];
        second.extend(header(0x0c));
        second.extend_from_slice(&[0x10, 0x00]);
        fake.lock().status = Some(status(1, u8::try_from(second.len()).unwrap()));
        fake.lock().blocks = vec![block0, second];
        // A stream that would say otherwise, had it not been disconnected.
        fake.stream([frame(999, 0, 0)]);
        fake.lock().period = 1_000_000;
        let mut device = fake.device();
        memory_source(&mut device);
        // The limit is not reached: the zeros of block 0 are samples.
        let packets = run(&mut device);
        let v = values(&packets);
        assert_eq!(v[..3], [-1.0, 65.4, 65.5]);
        assert_eq!(*v.last().unwrap(), 1.6);
        assert_eq!(v[v.len() - 2], -1.0);
        let writes = fake.writes();
        assert_eq!(
            writes,
            [
                vec![0xac, 0xff],
                vec![0xca, 0xff],
                vec![0xad, 0xda],
                vec![0xd3, 0xda, 0, 0],
                vec![0xd3, 0xda, 1, 0],
            ]
        );
        // dB(C) from the second recording's header.
        let Some(Packet::Analog(a)) = packets.iter().rev().nth(1) else { panic!("{packets:?}") };
        assert!(a.flags.contains(crate::core::MqFlags::SPL_FREQ_WEIGHT_C));
        assert_eq!(get_str(&device, &Key::SplWeightFreq), "C");
    });
}

#[test]
fn an_empty_memory_asks_for_its_one_block_and_ends() {
    bounded(|| {
        let fake = Fake::new();
        fake.lock().status = Some(status(0, 0));
        let mut device = fake.device();
        memory_source(&mut device);
        assert_eq!(values(&run(&mut device)), [0.0_f32; 0]);
        assert_eq!(fake.writes().last().unwrap(), &[0xd3, 0xda, 0, 0]);
    });
}

#[test]
fn memory_ends_at_the_last_length_with_more_on_the_way() {
    bounded(|| {
        let fake = Fake::new();
        let mut block = one_recording();
        let length = u8::try_from(block.len()).unwrap();
        // What a real one might send past the length: more samples.
        block.extend_from_slice(&[1, 2, 3, 4]);
        fake.lock().status = Some(status(0, length));
        fake.lock().blocks = vec![block];
        let mut device = fake.device();
        memory_source(&mut device);
        assert_eq!(values(&run(&mut device)), [-1.0, 65.4, 65.5]);
    });
}

#[test]
fn a_limit_cuts_the_output_of_memory_but_not_the_requests() {
    bounded(|| {
        let fake = Fake::new();
        let mut block0 = one_recording();
        block0.resize(256, 0);
        fake.lock().status = Some(status(1, 2));
        fake.lock().blocks = vec![block0, vec![0x10, 0x00]];
        let mut device = fake.device();
        memory_source(&mut device);
        limit(&mut device, 2);
        assert_eq!(values(&run(&mut device)), [-1.0, 65.4], "the -1 counts, as in C");
        assert_eq!(fake.writes().last().unwrap(), &[0xd3, 0xda, 1, 0]);
    });
}

#[test]
fn a_status_behind_stale_bytes_is_found_and_one_in_10_is_taken() {
    bounded(|| {
        let fake = Fake::new();
        let mut late = frame(654, 0, 0);
        late.extend(status(0, 0)[..8].iter().copied());
        late.push(0x10);
        fake.lock().status = Some(late);
        let mut device = fake.device();
        memory_source(&mut device);
        run(&mut device);
        assert_eq!(fake.writes().last().unwrap(), &[0xd3, 0xda, 0, 0]);
    });
}

#[test]
fn silence_ends_a_memory_capture_at_any_step_without_a_limit() {
    bounded(|| {
        // No status.
        let fake = Fake::new();
        let mut device = fake.device();
        device.waits.silence = Duration::from_millis(100);
        memory_source(&mut device);
        run(&mut device);
        assert_eq!(fake.writes().last().unwrap(), &[0xad, 0xda]);
        // A block lost. The request starts the wait again, so the status's time
        // does not count against it: 0 blocks asked for at the bound.
        let fake = Fake::new();
        fake.lock().status = Some(status(1, 0));
        let mut device = fake.device();
        device.waits.silence = Duration::from_millis(100);
        memory_source(&mut device);
        let began = Instant::now();
        run(&mut device);
        assert!(began.elapsed() >= Duration::from_millis(100));
        assert_eq!(fake.writes().last().unwrap(), &[0xd3, 0xda, 0, 0]);
    });
}

#[test]
fn a_blocks_wait_counts_from_its_request_not_from_the_status() {
    bounded(|| {
        let fake = Fake::new();
        fake.lock().status = Some(status(0, 4));
        fake.lock().blocks = vec![vec![0x00, 0x8e, 0x02, 0x00]];
        let mut device = fake.device();
        // The pause alone outlasts the bound, and a block must still be waited for.
        device.waits.silence = Duration::from_secs(1);
        device.waits.poll = Duration::from_millis(1200);
        memory_source(&mut device);
        assert_eq!(values(&run(&mut device)).len(), 2, "the block was given up on");
    });
}

#[test]
fn a_request_waits_the_sources_time_when_nothing_comes() {
    bounded(|| {
        let fake = Fake::new();
        fake.lock().status = Some(status(0, 0));
        let mut device = fake.device();
        device.waits.poll = Duration::from_millis(120);
        memory_source(&mut device);
        let began = Instant::now();
        run(&mut device);
        // Two waits: before the status and before the block.
        assert!(began.elapsed() >= Duration::from_millis(240), "{:?}", began.elapsed());
    });
}

#[test]
fn a_byte_there_before_a_request_ends_that_wait_and_stays_for_the_read() {
    bounded(|| {
        let fake = Fake::new();
        // Arrives while the status is read: it follows the status in one reply.
        let mut reply = status(0, 2);
        reply.extend_from_slice(&[0x11, 0x22]);
        fake.lock().status = Some(reply);
        let mut device = fake.device();
        device.waits.poll = Duration::from_secs(60);
        fake.lock().arrived.push_back(0x55);
        memory_source(&mut device);
        // A byte is there at both waits, so neither takes its time; and the two after
        // the status are the block's.
        assert_eq!(values(&run(&mut device)), [protocol::tenths(0x2211)]);
    });
}

#[test]
fn a_stop_in_mid_block_keeps_its_tail_from_the_next_capture() {
    bounded(|| {
        let fake = Fake::new();
        let mut block = one_recording();
        // A tail that passes for a reading of 999.9, at the line's pace: what a
        // purge would miss.
        block.resize(100, 0);
        block.extend(frame(9999, 0, 0));
        block.extend(frame(9999, 0, 0));
        fake.lock().status = Some(status(1, 0));
        fake.lock().blocks = vec![block];
        fake.lock().per_read = 2;
        let mut device = fake.device();
        memory_source(&mut device);
        // A queue of two, unread: the capture stalls on its readings, within the block.
        let (tx, rx) = sync_channel(2);
        device.start(PacketSink::new(tx, 0)).unwrap();
        wait_for(&fake, "the block's request", |m| m.writes.len() >= 4);
        fake.lock().per_read = 0;
        device.stop().unwrap();
        collect(&rx);
        fake.lock().per_read = 2;
        assert!(!fake.lock().coming.is_empty(), "the tail has come: this tests nothing");
        // A live capture right away.
        device.set(&Key::DataSource, &Value::Str("Live".to_owned()), None).unwrap();
        limit(&mut device, 1);
        fake.stream([frame(654, 0, 0)]);
        let writes_before = fake.writes().len();
        assert_eq!(values(&run(&mut device)), [65.4]);
        assert_eq!(fake.writes()[writes_before..], [vec![0xac, 0xff]], "it connects again");
    });
}

#[test]
fn a_memory_capture_after_one_stopped_does_not_read_the_old_tail_as_status() {
    bounded(|| {
        let fake = Fake::new();
        let mut block = vec![0; 40];
        // The old status, deep in what comes late.
        block.extend(status(0, 1));
        fake.lock().status = Some(status(1, 0));
        fake.lock().blocks = vec![block];
        fake.lock().per_read = 1;
        let mut device = fake.device();
        memory_source(&mut device);
        // A queue of two, unread: the capture stalls on its readings, within the block.
        let (tx, rx) = sync_channel(2);
        device.start(PacketSink::new(tx, 0)).unwrap();
        wait_for(&fake, "the block's request", |m| m.writes.len() >= 4);
        fake.lock().per_read = 0;
        device.stop().unwrap();
        collect(&rx);
        fake.lock().per_read = 1;
        assert!(!fake.lock().coming.is_empty(), "the tail has come: this tests nothing");
        fake.lock().status = Some(status(0, 0));
        fake.lock().blocks = vec![];
        assert_eq!(values(&run(&mut device)), [0.0_f32; 0]);
        // Status 0, 0 asks block 0 and ends with nothing read: a stale status with
        // last=1 would wait for a byte that never comes, for the test's bound.
        assert_eq!(fake.writes().last().unwrap(), &[0xd3, 0xda, 0, 0]);
    });
}

#[test]
fn settling_has_a_bound_if_the_line_never_goes_quiet() {
    bounded(|| {
        let fake = Fake::new();
        fake.lock().status = Some(status(0, 0));
        let mut device = fake.device();
        memory_source(&mut device);
        run(&mut device);
        // The meter goes on regardless, a byte per read.
        fake.lock().noise = true;
        device.waits.quiet_bound = Duration::from_millis(50);
        let began = Instant::now();
        // Silence means the wait for a status: the bound ends this.
        device.waits.silence = Duration::from_millis(1);
        fake.lock().status = None;
        run(&mut device);
        assert!(began.elapsed() >= Duration::from_millis(50));
        assert!(fake.writes().iter().filter(|w| *w == &[0xad, 0xda]).count() >= 1);
    });
}

#[test]
fn a_stop_ends_settling() {
    bounded(|| {
        let fake = Fake::new();
        fake.lock().status = Some(status(0, 0));
        let mut device = fake.device();
        memory_source(&mut device);
        run(&mut device);
        let requests = fake.writes().len();
        fake.lock().noise = true;
        device.waits.quiet_bound = Duration::from_secs(600);
        let (tx, rx) = sync_channel(4);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let (_, first) = rx.recv_timeout(BOUND).unwrap();
        assert!(matches!(first, Packet::Header { .. }));
        device.stop().unwrap();
        collect(&rx);
        assert_eq!(fake.writes().len(), requests, "no request came after it");
    });
}

#[test]
fn a_close_waits_for_the_capture_and_then_disconnects() {
    bounded(|| {
        let fake = Fake::new();
        let mut device = fake.device();
        let (tx, _rx) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        wait_for(&fake, "a read", |m| m.reads > 1);
        device.close().unwrap();
        assert_eq!(fake.writes().last().unwrap(), &[0xca, 0xff]);
    });
}

#[test]
fn a_reopen_starts_a_clean_slate() {
    bounded(|| {
        let fake = Fake::new();
        fake.lock().status = Some(status(0, 0));
        let mut device = fake.device();
        memory_source(&mut device);
        run(&mut device);
        device.close().unwrap();
        device.pending = Some(Box::new(fake.clone()));
        device.open().unwrap();
        device.set(&Key::DataSource, &Value::Str("Live".to_owned()), None).unwrap();
        limit(&mut device, 1);
        fake.stream([frame(654, 0, 0)]);
        let before = fake.lock().purges;
        assert_eq!(values(&run(&mut device)), [65.4]);
        assert_eq!(fake.lock().purges, before, "the open did it");
        let writes = fake.writes();
        assert_eq!(writes[writes.len() - 1], [0xac, 0xff], "the open's alone");
        assert_eq!(writes[writes.len() - 2], [0xca, 0xff]);
    });
}

#[test]
fn a_memory_capture_with_a_failed_status_write_ends_in_silence() {
    bounded(|| {
        let fake = Fake::new();
        let mut device = fake.device();
        device.waits.silence = Duration::from_millis(30);
        memory_source(&mut device);
        // Only the connect went through.
        fake.lock().fail_writes_after = Some(1);
        assert_eq!(values(&run(&mut device)), [0.0_f32; 0]);
        assert_eq!(fake.writes().len(), 1);
    });
}

#[test]
fn tenths_cover_every_u16_without_a_panic() {
    assert_eq!(
        values(&[Packet::Analog(AnalogData {
            channels: vec![0],
            samples: vec![protocol::tenths(u16::MAX)],
            mq: Mq::SoundPressureLevel,
            flags: Mode::default().flags(),
            unit: Unit::DecibelSpl,
            digits: 1,
        })]),
        [6553.5]
    );
}
