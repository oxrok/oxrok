// Counterpart in sigrok:
//   libsigrok/src/hardware/mastech-ms6514/api.c,
//   libsigrok/src/hardware/mastech-ms6514/protocol.c,
//   Copyright (C) 2019 Dave Buechi <db@pflutsch.ch>,
//   with commits by Gerhard Sittig, Wolfram Sang, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

use super::*;
use crate::core::{MqFlags, Unit};
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// A bound no good run comes near, under any load; a test past it fails.
const BOUND: Duration = Duration::from_secs(60);

/// The far end of the cable. A byte comes off `wire` only when a read releases
/// it, so what has arrived is a matter of reads and not of the clock, and a purge
/// drops only what has arrived.
#[derive(Default)]
struct Line {
    wire: VecDeque<u8>,
    arrived: VecDeque<u8>,
    /// Bytes a read lets arrive; 0 means all there is.
    per_read: usize,
    /// Played again whenever the wire runs out.
    forever: Vec<u8>,
    /// Frames of a dump, onto the wire when `a1` is written.
    dump: Vec<u8>,
    reads: usize,
    purges: usize,
    writes: Vec<Vec<u8>>,
    /// The most any read asked for.
    largest_read: usize,
    fail_reads: bool,
    fail_writes: bool,
    fail_purges: bool,
    /// Packets the sink held at each write, for the order of header and `a1`.
    log: Vec<&'static str>,
}

#[derive(Clone)]
struct Fake(Arc<Mutex<Line>>);

impl Fake {
    fn new(line: Line) -> Fake {
        Fake(Arc::new(Mutex::new(line)))
    }

    fn line(&self) -> MutexGuard<'_, Line> {
        // A failed test may hold it, and a drop still writes through here.
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

impl Port for Fake {
    fn name(&self) -> &'static str {
        "/dev/fake"
    }
    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }
    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [u8], _: Duration) -> Result<usize> {
        let got = {
            let mut l = self.line();
            l.reads += 1;
            l.largest_read = l.largest_read.max(buf.len());
            if l.fail_reads {
                return Err(Error::device("gone"));
            }
            if l.wire.is_empty() && !l.forever.is_empty() {
                let more = l.forever.clone();
                l.wire.extend(more);
            }
            let n = if l.per_read == 0 { l.wire.len() } else { l.per_read.min(l.wire.len()) };
            for _ in 0..n {
                let b = l.wire.pop_front().expect("counted");
                l.arrived.push_back(b);
            }
            let got = buf.len().min(l.arrived.len());
            for b in &mut buf[..got] {
                *b = l.arrived.pop_front().expect("counted");
            }
            got
        };
        if got == 0 {
            // An empty read takes time, with the lock free.
            std::thread::sleep(Duration::from_millis(1));
        }
        Ok(got)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut l = self.line();
        l.log.push("write");
        l.writes.push(data.to_vec());
        if l.fail_writes {
            return Err(Error::device("gone"));
        }
        if data == [0xa1] {
            let dump = l.dump.clone();
            l.wire.extend(dump);
        }
        Ok(())
    }
    fn purge(&mut self) -> Result<()> {
        let mut l = self.line();
        if l.fail_purges {
            return Err(Error::device("gone"));
        }
        l.purges += 1;
        l.arrived.clear();
        Ok(())
    }
}

/// A frame in tenths of a degree C: T1 and T2, with no function.
fn frame(main: u16, aux: u16) -> Vec<u8> {
    let [mh, ml] = main.to_be_bytes();
    let [ah, al] = aux.to_be_bytes();
    vec![0x65, 0x14, 0, 0, 0, mh, ml, ah, al, 1, 1, 8, 8, 1, 2, 3, 0x0d, 0x0a]
}

fn stored(main: u16, aux: u16) -> Vec<u8> {
    let mut f = frame(main, aux);
    f[2] = 1;
    f
}

fn frames(values: &[u16]) -> Vec<u8> {
    values.iter().flat_map(|v| frame(*v, *v + 1)).collect()
}

type Rx = Receiver<(usize, Packet)>;

fn sink(room: usize) -> (PacketSink, Rx) {
    let (tx, rx) = sync_channel(room);
    (PacketSink::new(tx, 0), rx)
}

fn device(fake: &Fake) -> MastechMs6514Device {
    let mut d = MastechMs6514Device::from_port(Box::new(fake.clone()));
    d.open().unwrap();
    d
}

/// Runs a test on its own thread, so one that hangs fails.
fn bounded(test: impl FnOnce() + Send + 'static) {
    let (done, finished) = std::sync::mpsc::channel();
    let handle = std::thread::spawn(move || {
        test();
        let _ = done.send(());
    });
    match finished.recv_timeout(BOUND) {
        Ok(()) => handle.join().unwrap(),
        // The thread panicked: show that, not a timeout.
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(handle.join().unwrap_err())
        }
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("it hung"),
    }
}

/// All packets up to the `End`, within the bound.
fn until_end(rx: &Rx) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut got = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        let (_, p) = rx.recv_timeout(left).expect("no End came");
        if matches!(p, Packet::End) {
            return got;
        }
        got.push(p);
    }
}

fn readings(packets: &[Packet]) -> Vec<(usize, f32)> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some((a.channels[0], a.samples[0])),
            _ => None,
        })
        .collect()
}

/// A capture of `samples` over what `fake` plays, with `set` applied first.
fn capture(fake: &Fake, set: impl FnOnce(&mut MastechMs6514Device)) -> Vec<Packet> {
    let mut d = device(fake);
    set(&mut d);
    let (s, rx) = sink(4096);
    d.start(s).unwrap();
    let got = until_end(&rx);
    d.close().unwrap();
    got
}

fn limit(d: &mut MastechMs6514Device, n: u64) {
    d.set(&Key::LimitSamples, &Value::U64(n), None).unwrap();
}

fn memory(d: &mut MastechMs6514Device) {
    d.set(&Key::DataSource, &Value::Str("Memory".to_owned()), None).unwrap();
}

fn on_wire(bytes: &[u8], per_read: usize) -> Fake {
    Fake::new(Line { wire: bytes.iter().copied().collect(), per_read, ..Line::default() })
}

fn params() -> Params {
    Params::parse("9600/8n1").unwrap()
}

#[test]
fn the_literals() {
    assert_eq!(SCAN_TIME, Duration::from_millis(1000));
    assert_eq!(SCAN_BYTES, 36);
    assert_eq!(SERIALCOMM, "9600/8n1");
    assert_eq!(MastechMs6514Driver.id(), "mastech-ms6514");
    assert_eq!(MastechMs6514Driver.long_name(), "MASTECH MS6514");
    assert_eq!(MastechMs6514Driver.classes(), [DeviceClass::Thermometer]);
    assert_eq!(MastechMs6514Driver.scan_options(), [Key::Conn, Key::Serialcomm]);
}

#[test]
fn a_scan_finds_a_frame_and_reads_no_further() {
    bounded(|| {
        let mut bytes = frame(1, 2);
        bytes.extend(frame(3, 4));
        let mut fake = on_wire(&bytes, 1);
        assert!(detect(&mut fake, &params(), SCAN_TIME));
        let l = fake.line();
        assert_eq!(l.wire.len() + l.arrived.len(), 18, "the second frame is unread");
        assert_eq!(l.largest_read, 1, "byte by byte");
        assert!(l.writes.is_empty(), "a scan writes nothing");
    });
}

#[test]
fn a_scan_has_room_for_a_frame_cut_by_the_open_and_no_more() {
    bounded(|| {
        let f = frame(1, 2);
        let mut bytes = vec![0x3f; 18];
        bytes.extend(&f);
        let mut fake = on_wire(&bytes, 0);
        assert!(detect(&mut fake, &params(), SCAN_TIME), "18 bytes ahead of it");
        let mut bytes = vec![0x3f; 19];
        bytes.extend(&f);
        let mut fake = on_wire(&bytes, 0);
        assert!(!detect(&mut fake, &params(), SCAN_TIME), "19 bytes ahead of it");
        assert_eq!(fake.line().arrived.len(), 1, "36 bytes read");
    });
}

#[test]
fn a_scan_wants_a_head_and_an_end() {
    bounded(|| {
        let mut no_end = frame(1, 2);
        no_end[17] = 0;
        let mut fake = on_wire(&no_end, 0);
        assert!(!detect(&mut fake, &params(), Duration::from_millis(20)));
        let mut no_head = frame(1, 2);
        no_head[0] = 0;
        let mut fake = on_wire(&no_head, 0);
        assert!(!detect(&mut fake, &params(), Duration::from_millis(20)));
    });
}

#[test]
fn a_scan_listens_for_its_time() {
    bounded(|| {
        // From below: a silent port takes all of it. From above: a frame that comes
        // after half of it, which C's 500 ms would miss, is found.
        let mut fake = on_wire(&[], 0);
        let start = Instant::now();
        assert!(!detect(&mut fake, &params(), SCAN_TIME));
        assert!(start.elapsed() >= Duration::from_millis(1000));
        assert!(start.elapsed() < BOUND);
        let gone = Fake::new(Line { fail_reads: true, ..Line::default() });
        let mut port = gone.clone();
        let start = Instant::now();
        assert!(!detect(&mut port, &params(), Duration::from_millis(30)));
        assert!(start.elapsed() >= Duration::from_millis(30), "a failed read is no byte");
    });
}

#[test]
fn the_options() {
    let fake = on_wire(&[], 0);
    let mut d = device(&fake);
    assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::Str("Live".to_owned()));
    assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(0));
    assert_eq!(d.get(&Key::LimitMsec, None).unwrap(), Value::U64(0));
    memory(&mut d);
    assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::Str("Memory".to_owned()));
    for bad in ["memory", "", "Nothing"] {
        let err = d.set(&Key::DataSource, &Value::Str(bad.to_owned()), None).unwrap_err();
        assert_eq!(err.to_string(), "invalid argument: invalid value for data_source", "{bad}");
    }
    assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::Str("Memory".to_owned()));
    d.set(&Key::DataSource, &Value::Str("Live".to_owned()), None).unwrap();
    assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::Str("Live".to_owned()));
    assert_eq!(
        d.list(&Key::DataSource, None).unwrap(),
        ValueList::Str(vec!["Live".to_owned(), "Memory".to_owned()])
    );
    assert!(d.get(&Key::Conn, None).is_err());
    assert!(d.set(&Key::Samplerate, &Value::U64(1), None).is_err());
    assert!(d.get(&Key::DataSource, Some("x")).is_err());
    assert_eq!(d.info().conn, None, "as C, which can't read conn back");
    let names: Vec<&str> = d.channels().iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["T1", "T2", "T1-T2"]);
    let keys: Vec<Key> = d.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(keys, [Key::Continuous, Key::LimitSamples, Key::LimitMsec, Key::DataSource]);
    // A setting reaches no device.
    assert_eq!(fake.line().writes.len(), 0);
}

#[test]
fn a_capture_sends_a_frames_readings() {
    let mut f = frame(235, 241);
    f[11] = 0x88; // negative
    f[10] = 0x42; // hold, F
    bounded(move || {
        let fake = on_wire(&f, 0);
        let got = capture(&fake, |d| limit(d, 1));
        assert!(matches!(got[0], Packet::Header { .. }), "{got:?}");
        assert_eq!(got.len(), 3);
        let Packet::Analog(a) = &got[1] else { panic!("{got:?}") };
        assert_eq!(
            (a.channels.as_slice(), a.samples.as_slice()),
            ([0].as_slice(), [-23.5].as_slice())
        );
        assert_eq!(
            (a.mq, a.unit, a.flags, a.digits),
            (Mq::Temperature, Unit::Fahrenheit, MqFlags::HOLD, 1)
        );
        let Packet::Analog(a) = &got[2] else { panic!("{got:?}") };
        assert_eq!(
            (a.channels.as_slice(), a.samples.as_slice()),
            ([1].as_slice(), [24.1].as_slice())
        );
    });
}

#[test]
fn a_limit_comes_after_a_whole_read() {
    bounded(|| {
        let fake = on_wire(&frames(&[10, 20, 30, 40]), 0);
        let got = capture(&fake, |d| limit(d, 1));
        assert_eq!(
            readings(&got),
            [(0, 1.0), (1, 1.1), (0, 2.0), (1, 2.1), (0, 3.0), (1, 3.1)],
            "three frames fit a read, and the fourth waits"
        );
        assert_eq!(fake.line().largest_read, 54);
        assert_eq!(fake.line().arrived.len(), 18);
        assert_eq!(fake.line().reads, 1);
    });
}

#[test]
fn a_limit_is_exact_a_frame_at_a_time() {
    bounded(|| {
        let fake = on_wire(&frames(&[10, 20, 30, 40]), 18);
        let got = capture(&fake, |d| limit(d, 2));
        assert_eq!(readings(&got).len(), 4);
        assert_eq!(fake.line().wire.len(), 36, "nothing more was let in");
    });
}

#[test]
fn no_limit_runs_on() {
    // A limit of 0 is none: all frames go, and a stop ends it.
    bounded(|| {
        let fake = Fake::new(Line { forever: frame(10, 20), per_read: 18, ..Line::default() });
        let mut d = device(&fake);
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let mut seen = 0;
        while seen < 40 {
            if matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)) {
                seen += 1;
            }
        }
        d.stop().unwrap();
        until_end(&rx);
        d.close().unwrap();
    });
}

#[test]
fn a_read_takes_only_the_room_left() {
    bounded(|| {
        // 53 bytes in a row are no frame; the tail of 17 waits in the buffer.
        let mut bytes = frames(&[10, 20]);
        bytes.extend(&frame(30, 40)[..17]);
        let fake = on_wire(&bytes, 53);
        let mut d = device(&fake);
        limit(&mut d, 3);
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        // Two frames sent, 17 bytes held, and the limit not reached.
        let deadline = Instant::now() + BOUND;
        while fake.line().reads < 3 {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        fake.line().wire.push_back(0x0a);
        let got = until_end(&rx);
        d.close().unwrap();
        assert_eq!(readings(&got).len(), 6);
        assert_eq!(readings(&got)[4], (0, 3.0));
    });
}

#[test]
fn frames_come_together_from_single_bytes() {
    bounded(|| {
        let fake = on_wire(&frames(&[10, 20]), 1);
        let got = capture(&fake, |d| limit(d, 2));
        assert_eq!(readings(&got), [(0, 1.0), (1, 1.1), (0, 2.0), (1, 2.1)]);
    });
}

#[test]
fn it_finds_its_step_again() {
    bounded(|| {
        let mut bytes = vec![0x3f, 0x65, 0x3f];
        bytes.extend(frame(10, 11));
        let fake = on_wire(&bytes, 0);
        assert_eq!(readings(&capture(&fake, |d| limit(d, 1))), [(0, 1.0), (1, 1.1)]);
    });
}

#[test]
fn a_head_with_no_end_drops_18_bytes() {
    bounded(|| {
        // The false head's 18 bytes reach 2 bytes into the real frame, the rest
        // of which goes by a byte at a time.
        let mut bytes = vec![0x65, 0x14];
        bytes.extend(&[0; 14]);
        bytes.extend(frame(10, 11));
        bytes.extend(frame(20, 21));
        let fake = on_wire(&bytes, 0);
        assert_eq!(readings(&capture(&fake, |d| limit(d, 1))), [(0, 2.0), (1, 2.1)]);
        // The same bytes one at a time.
        let fake = on_wire(&bytes, 1);
        assert_eq!(readings(&capture(&fake, |d| limit(d, 1))), [(0, 2.0), (1, 2.1)]);
    });
}

#[test]
fn a_reading_goes_out_by_the_channel_it_names() {
    bounded(|| {
        // T2 on main and T1 on aux.
        let mut f = frame(10, 20);
        f[11] |= 1;
        let cases: [(&[usize], &[(usize, f32)]); 4] = [
            (&[0, 1, 2], &[(1, 1.0), (0, 2.0)]),
            (&[0], &[(0, 2.0)]),
            (&[1], &[(1, 1.0)]),
            (&[2], &[]),
        ];
        for (enabled, want) in cases {
            let fake = on_wire(&f, 0);
            let got = capture(&fake, |d| {
                limit(d, 1);
                for i in 0..3 {
                    d.enable_channel(i, enabled.contains(&i)).unwrap();
                }
            });
            assert_eq!(readings(&got), want, "{enabled:?}");
        }
        // T1-T2 on main and T1-T2 MAX on aux, with T1-T2 alone.
        let mut f = frame(10, 20);
        f[11] |= 2;
        f[12] |= 1;
        let fake = on_wire(&f, 0);
        let got = capture(&fake, |d| {
            limit(d, 1);
            d.enable_channel(0, false).unwrap();
            d.enable_channel(1, false).unwrap();
        });
        assert_eq!(readings(&got), [(2, 1.0), (2, 2.0)]);
    });
}

#[test]
fn memory_asks_after_the_header_and_ends_at_the_live_frame() {
    bounded(|| {
        let mut dump = stored(10, 11);
        dump.extend(stored(20, 21));
        dump.extend(frame(30, 31));
        dump.extend(stored(40, 41));
        let fake = Fake::new(Line { dump, ..Line::default() });
        let mut d = device(&fake);
        memory(&mut d);
        // The queue holds no more than the header until it is read: a write that
        // comes ahead of the header would be seen with nothing in the queue.
        let (s, rx) = sink(1);
        d.start(s).unwrap();
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        let got = until_end(&rx);
        d.close().unwrap();
        assert_eq!(
            readings(&got),
            [(0, 1.0), (1, 1.1), (0, 2.0), (1, 2.1)],
            "none past the live frame"
        );
        assert_eq!(fake.line().writes, [vec![0xa1]]);
    });
}

#[test]
fn memory_with_a_room_for_the_header_alone_has_not_asked() {
    bounded(|| {
        // The header can't leave a full queue, so a1 must not have gone yet.
        let fake = Fake::new(Line { ..Line::default() });
        let mut d = device(&fake);
        memory(&mut d);
        let (tx, rx) = sync_channel(1);
        tx.send((0, Packet::End)).unwrap();
        d.start(PacketSink::new(tx, 0)).unwrap();
        std::thread::sleep(Duration::from_millis(30));
        assert!(fake.line().writes.is_empty(), "a1 ahead of the header");
        // Let it go.
        rx.recv_timeout(BOUND).unwrap();
        let deadline = Instant::now() + BOUND;
        while fake.line().writes.is_empty() {
            assert!(Instant::now() < deadline, "a1 never went");
            std::thread::sleep(Duration::from_millis(1));
        }
        d.close().unwrap();
    });
}

#[test]
fn live_asks_for_nothing_and_takes_a_stored_frame() {
    bounded(|| {
        let fake = on_wire(&stored(10, 11), 0);
        let got = capture(&fake, |d| limit(d, 1));
        assert_eq!(readings(&got), [(0, 1.0), (1, 1.1)]);
        assert_eq!(fake.line().writes.len(), 0);
    });
}

#[test]
fn a_failed_request_is_no_failed_capture() {
    bounded(|| {
        let fake = Fake::new(Line { fail_writes: true, ..Line::default() });
        fake.line().wire.extend(stored(10, 11));
        let got = capture(&fake, |d| {
            memory(d);
            limit(d, 1);
        });
        assert_eq!(readings(&got), [(0, 1.0), (1, 1.1)]);
    });
}

#[test]
fn a_limit_ends_a_dump() {
    bounded(|| {
        let mut dump = stored(10, 11);
        dump.extend(stored(20, 21));
        let fake = Fake::new(Line { dump, per_read: 18, ..Line::default() });
        let got = capture(&fake, |d| {
            memory(d);
            limit(d, 1);
        });
        assert_eq!(readings(&got), [(0, 1.0), (1, 1.1)]);
    });
}

#[test]
fn a_stop_ends_a_capture_that_has_data() {
    bounded(|| {
        let fake = Fake::new(Line { forever: frame(10, 20), ..Line::default() });
        let mut d = device(&fake);
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)));
        d.stop().unwrap();
        // Reading on, so that nothing but the stop can end it.
        until_end(&rx);
        d.close().unwrap();
    });
}

#[test]
fn a_time_limit_ends_a_silent_capture_at_its_time() {
    bounded(|| {
        let fake = on_wire(&[], 0);
        let mut d = device(&fake);
        d.set(&Key::LimitMsec, &Value::U64(200), None).unwrap();
        let (s, rx) = sink(64);
        let start = Instant::now();
        d.start(s).unwrap();
        let got = until_end(&rx);
        assert!(start.elapsed() >= Duration::from_millis(200), "{:?}", start.elapsed());
        assert_eq!(got.len(), 1, "the header alone");
        d.close().unwrap();
    });
}

#[test]
fn a_time_limit_ends_a_stream() {
    bounded(|| {
        let fake = Fake::new(Line { forever: frame(10, 20), per_read: 18, ..Line::default() });
        let mut d = device(&fake);
        d.set(&Key::LimitMsec, &Value::U64(50), None).unwrap();
        let (s, rx) = sink(4096);
        d.start(s).unwrap();
        // Filling the queue ends in a stop, so read it: a limit alone ends this.
        until_end(&rx);
        d.close().unwrap();
    });
}

#[test]
fn a_limit_of_samples_waits_on_a_silent_unit() {
    bounded(|| {
        let fake = on_wire(&frame(10, 11), 0);
        let mut d = device(&fake);
        limit(&mut d, 2);
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let mut got = Vec::new();
        while got.len() < 3 {
            got.push(rx.recv_timeout(BOUND).unwrap().1);
        }
        // Reads go on and nothing ends it, for 100 reads.
        let from = fake.line().reads;
        let deadline = Instant::now() + BOUND;
        while fake.line().reads < from + 100 {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(rx.try_recv().is_err(), "it ended");
        // And a frame that comes late counts.
        fake.line().wire.extend(frame(20, 21));
        let got = until_end(&rx);
        assert_eq!(readings(&got), [(0, 2.0), (1, 2.1)]);
        d.close().unwrap();
    });
}

#[test]
fn a_failed_read_ends_the_capture() {
    bounded(|| {
        let fake = Fake::new(Line { fail_reads: true, ..Line::default() });
        let got = capture(&fake, |d| limit(d, 2));
        assert_eq!(got.len(), 1, "the header alone");
    });
}

#[test]
fn a_full_queue_does_not_hold_a_stop_or_a_close() {
    bounded(|| {
        let fake = Fake::new(Line { forever: frame(10, 20), ..Line::default() });
        let mut d = device(&fake);
        // Nobody reads: the header takes the room and the first reading waits.
        let (s, _rx) = sink(1);
        d.start(s).unwrap();
        let deadline = Instant::now() + BOUND;
        while fake.line().reads == 0 {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        d.close().unwrap();
        assert!(d.port().is_none());
    });
}

#[test]
fn a_full_queue_does_not_hold_a_close_at_the_header() {
    bounded(|| {
        let fake = on_wire(&frame(10, 11), 0);
        let mut d = device(&fake);
        // No room even for the header.
        let (tx, _rx) = sync_channel(1);
        tx.send((0, Packet::End)).unwrap();
        d.start(PacketSink::new(tx, 0)).unwrap();
        d.close().unwrap();
        assert_eq!(fake.line().reads, 0, "it can't have got past the header");
    });
}

#[test]
fn a_close_ends_a_capture_and_the_port_stays_gone() {
    bounded(|| {
        let fake = Fake::new(Line { forever: frame(10, 20), per_read: 18, ..Line::default() });
        let mut d = device(&fake);
        let (s, rx) = sink(4096);
        d.start(s).unwrap();
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)));
        // No stop first: the close says so itself, and waits for the thread.
        d.close().unwrap();
        let reads = fake.line().reads;
        // Longer than a read's wait: a thread left behind would read, and give
        // the port back.
        std::thread::sleep(Duration::from_millis(150));
        assert_eq!(fake.line().reads, reads, "it read on");
        assert!(d.port().is_none(), "it came back");
    });
}

#[test]
fn the_port_comes_back_ahead_of_an_end_that_cannot_go() {
    bounded(|| {
        let fake = on_wire(&frame(10, 11), 0);
        let mut d = device(&fake);
        limit(&mut d, 1);
        // Room for the header and the two readings, and not the End.
        let (s, rx) = sink(3);
        d.start(s).unwrap();
        let deadline = Instant::now() + BOUND;
        while d.port().is_none() {
            assert!(Instant::now() < deadline, "End came first");
            std::thread::sleep(Duration::from_millis(1));
        }
        drop(rx);
        d.close().unwrap();
    });
}

#[test]
fn a_start_after_an_end_stuck_on_a_full_queue_does_not_hang() {
    bounded(|| {
        let fake = on_wire(&frames(&[10, 20]), 18);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let (s, stuck) = sink(3);
        d.start(s).unwrap();
        let deadline = Instant::now() + BOUND;
        while d.capturing.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let got = until_end(&rx);
        assert_eq!(readings(&got), [(0, 2.0), (1, 2.1)]);
        // The first capture gave up its End, so a reader coming late sees none.
        let late: Vec<Packet> = stuck.try_iter().map(|(_, p)| p).collect();
        assert_eq!(late.len(), 3, "{late:?}");
        assert!(!late.iter().any(|p| matches!(p, Packet::End)), "{late:?}");
        std::thread::sleep(Duration::from_millis(20));
        assert!(stuck.try_recv().is_err(), "an End was still waiting");
        d.close().unwrap();
    });
}

#[test]
fn a_start_during_a_capture_is_busy() {
    bounded(|| {
        let fake = on_wire(&[], 0);
        let mut d = device(&fake);
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let (s2, _rx2) = sink(1);
        assert!(matches!(d.start(s2), Err(Error::Busy)));
        d.stop().unwrap();
        until_end(&rx);
        d.close().unwrap();
    });
}

#[test]
fn a_start_with_no_port_fails() {
    let fake = on_wire(&[], 0);
    let mut d = MastechMs6514Device::from_port(Box::new(fake));
    let (s, _rx) = sink(1);
    assert!(d.start(s).is_err());
}

#[test]
fn the_open_flushes_and_the_first_capture_does_not_again() {
    bounded(|| {
        let fake = Fake::new(Line::default());
        fake.line().arrived.extend(frame(1, 2));
        let mut d = device(&fake);
        assert_eq!(fake.line().purges, 1);
        assert!(fake.line().arrived.is_empty());
        d.open().unwrap();
        assert_eq!(fake.line().purges, 1, "open already");
        fake.line().arrived.extend(frame(10, 11));
        limit(&mut d, 1);
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let got = until_end(&rx);
        assert_eq!(readings(&got), [(0, 1.0), (1, 1.1)], "what came since the open counts");
        assert_eq!(fake.line().purges, 1);
        d.close().unwrap();
    });
}

/// Waits for the capture's end and starts another, of `limit` samples.
fn second_capture(d: &mut MastechMs6514Device) -> Vec<Packet> {
    let (s, rx) = sink(4096);
    d.start(s).unwrap();
    until_end(&rx)
}

#[test]
fn a_second_capture_takes_nothing_old() {
    bounded(|| {
        let fake = on_wire(&frame(10, 11), 0);
        let mut d = device(&fake);
        limit(&mut d, 1);
        assert_eq!(readings(&second_capture(&mut d)), [(0, 1.0), (1, 1.1)]);
        // Frames queue up on the open port between the two.
        fake.line().arrived.extend(frames(&[20, 30]));
        fake.line().wire.extend(frame(40, 41));
        assert_eq!(readings(&second_capture(&mut d)), [(0, 4.0), (1, 4.1)]);
        assert_eq!(fake.line().purges, 2);
        d.close().unwrap();
    });
}

#[test]
fn a_frame_cut_by_a_stop_does_not_shift_the_next_capture() {
    bounded(|| {
        // Byte paced: the first capture ends in the middle of the next frame.
        let values: Vec<u16> = (1..=8).map(|v| v * 10).collect();
        let fake = on_wire(&frames(&values), 1);
        let mut d = device(&fake);
        limit(&mut d, 1);
        assert_eq!(readings(&second_capture(&mut d)), [(0, 1.0), (1, 1.1)]);
        fake.line().per_read = 0;
        let cut: Vec<u8> = {
            let mut l = fake.line();
            let at = l.wire.len() - 5 * 18 - 7;
            l.wire.drain(..at).collect()
        };
        assert_ne!(cut.len(), 0);
        // 7 bytes of a frame come first now, and all that follows is in step.
        limit(&mut d, 5);
        let got = readings(&second_capture(&mut d));
        assert_eq!(got.len(), 10, "{got:?}");
        assert_eq!(got[0], (0, 4.0), "{got:?}");
        assert_eq!(got[9], (1, 8.1), "{got:?}");
        d.close().unwrap();
    });
}

#[test]
fn a_stored_frame_after_a_dump_counts_in_a_live_capture() {
    bounded(|| {
        let mut dump = stored(10, 11);
        dump.extend(frame(20, 21));
        let fake = Fake::new(Line { dump, ..Line::default() });
        let mut d = device(&fake);
        memory(&mut d);
        assert_eq!(readings(&second_capture(&mut d)), [(0, 1.0), (1, 1.1)]);
        d.set(&Key::DataSource, &Value::Str("Live".to_owned()), None).unwrap();
        fake.line().wire.extend(stored(30, 31));
        limit(&mut d, 1);
        assert_eq!(readings(&second_capture(&mut d)), [(0, 3.0), (1, 3.1)]);
        d.close().unwrap();
    });
}

#[test]
fn a_failed_purge_keeps_the_port() {
    bounded(|| {
        let fake = on_wire(&frame(10, 11), 0);
        let mut d = device(&fake);
        limit(&mut d, 1);
        second_capture(&mut d);
        fake.line().fail_purges = true;
        let (s, _rx) = sink(64);
        assert!(d.start(s).is_err());
        assert!(d.port().is_some(), "the port is gone");
        fake.line().fail_purges = false;
        fake.line().wire.extend(frame(20, 21));
        assert_eq!(readings(&second_capture(&mut d)), [(0, 2.0), (1, 2.1)]);
        d.close().unwrap();
    });
}

#[test]
fn an_open_during_a_capture_does_nothing() {
    bounded(|| {
        let fake = on_wire(&[], 0);
        let mut d = device(&fake);
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        // A port it would take now, while the capture holds the real one.
        let other = on_wire(&[], 0);
        d.pending = Some(Box::new(other.clone()));
        d.open().unwrap();
        assert_eq!(other.line().purges, 0);
        assert!(d.pending.is_some());
        assert!(d.port().is_none());
        d.stop().unwrap();
        until_end(&rx);
        d.close().unwrap();
    });
}

#[test]
fn a_setting_after_a_capture_touches_nothing() {
    bounded(|| {
        let fake = on_wire(&frame(10, 11), 0);
        let mut d = device(&fake);
        limit(&mut d, 1);
        second_capture(&mut d);
        let reads = fake.line().reads;
        memory(&mut d);
        limit(&mut d, 4);
        assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::Str("Memory".to_owned()));
        assert_eq!(fake.line().writes.len(), 0);
        assert_eq!(fake.line().reads, reads);
        d.close().unwrap();
    });
}

#[test]
fn a_setting_during_a_capture_counts_for_the_next() {
    bounded(|| {
        let fake = on_wire(&[], 0);
        let mut d = device(&fake);
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        memory(&mut d);
        fake.line().wire.extend(frame(10, 11));
        let got: Vec<Packet> = (0..3).map(|_| rx.recv_timeout(BOUND).unwrap().1).collect();
        assert_eq!(readings(&got), [(0, 1.0), (1, 1.1)], "still live");
        d.stop().unwrap();
        until_end(&rx);
        assert_eq!(fake.line().writes.len(), 0);
        d.close().unwrap();
    });
}

#[test]
fn a_close_and_an_open_start_afresh() {
    bounded(|| {
        let fake = on_wire(&frame(10, 11), 0);
        let mut d = MastechMs6514Device::from_port(Box::new(fake.clone()));
        d.open().unwrap();
        limit(&mut d, 1);
        second_capture(&mut d);
        d.close().unwrap();
        assert!(d.port().is_none());
        let (s, _rx) = sink(1);
        assert!(d.start(s).is_err(), "closed");
        // The open flushes, and what comes after it is this capture's.
        d.pending = Some(Box::new(fake.clone()));
        d.open().unwrap();
        assert_eq!(fake.line().purges, 2);
        fake.line().arrived.extend(frame(20, 21));
        assert_eq!(readings(&second_capture(&mut d)), [(0, 2.0), (1, 2.1)]);
        assert_eq!(fake.line().purges, 2);
        d.close().unwrap();
    });
}

#[test]
fn a_drop_ends_a_running_capture() {
    bounded(|| {
        let fake = Fake::new(Line { forever: frame(10, 20), per_read: 18, ..Line::default() });
        let mut d = device(&fake);
        let (s, _rx) = sink(4);
        d.start(s).unwrap();
        drop(d);
        let reads = fake.line().reads;
        std::thread::sleep(Duration::from_millis(20));
        assert_eq!(fake.line().reads, reads, "it read on");
    });
}
