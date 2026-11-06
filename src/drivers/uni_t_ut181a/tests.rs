// Counterpart in sigrok:
//   libsigrok/src/hardware/uni-t-ut181a/protocol.c,
//   Copyright (C) 2019-2020 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake port: a meter's frames, released by reads made, or at the
//! line's rate where what a capture leaves behind is the point.

use super::protocol::{frame, get_rec_info, get_rec_samples, get_saved, get_saved_count, monitor};
use super::*;
use crate::core::MqFlags;
use std::collections::{HashSet, VecDeque};
use std::sync::PoisonError;
use std::sync::mpsc::{Receiver as Rx, RecvTimeoutError, sync_channel};

/// How long a test waits for a capture before it fails: a stop or a limit that
/// does not bite must fail, not hang the run.
const CAPTURE_BOUND: Duration = Duration::from_secs(20);

/// Runs a test that starts a capture on a thread of its own, and fails it if it
/// has not come back in time. A stop that does not bite fails the wait for the
/// End, and then the device's drop would join that capture for ever.
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

/// One byte per millisecond, near enough to 9600 8n1.
const BYTE: Duration = Duration::from_millis(1);

#[derive(Default)]
struct State {
    writes: Vec<Vec<u8>>,
    /// What has arrived.
    ready: VecDeque<u8>,
    /// Bytes on their way: released on a later read, by its count.
    by_reads: VecDeque<(usize, Vec<u8>)>,
    /// Bytes on their way at the line's rate: arrive at a time.
    by_time: VecDeque<(Instant, u8)>,
    reads: usize,
    /// Answer at the line's rate rather than at once.
    paced: bool,
    /// Every answer starts this much later, on a paced line.
    answer_after: Duration,
    /// Answers given, by command code.
    seen: Vec<u8>,
    /// Readings for monitor mode, one at the command, the next every few reads.
    readings: VecDeque<Vec<u8>>,
    /// Reads between readings.
    every: usize,
    since_reading: usize,
    monitor: bool,
    saved: Vec<Vec<u8>>,
    /// Name, unit, interval, samples.
    recordings: Vec<(&'static str, &'static str, u16, u32)>,
    per: u32,
    have: Option<u32>,
    /// Codes that get no answer, each time.
    silent: HashSet<u8>,
    /// Which answer of a code to hold back, by how many reads.
    late: Vec<(u8, usize, usize)>,
    /// Codes answered `ER`.
    refused: HashSet<u8>,
    /// What a read fails with.
    read_error: Option<&'static str>,
    /// Every read takes this long, bytes or none.
    each_read: Duration,
    /// A chunk's last byte does not come, in a frame still good.
    cut_chunks: bool,
    /// An answer of this code comes this much later, on a paced line.
    code_delay: Option<(u8, Duration)>,
    /// A reading comes ahead of a monitor off's `OK`, as `antage/ut181a` allows.
    reading_before_off: Option<Vec<u8>>,
    /// Behind the answer of this code the head of a frame comes at once, and its
    /// tail some reads later.
    stale_behind: Option<(u8, Vec<u8>, Vec<u8>)>,
}

#[derive(Clone)]
struct Fake(Arc<Mutex<State>>);

impl Fake {
    fn new() -> Fake {
        let state = State { every: 3, per: 2, ..State::default() };
        Fake(Arc::new(Mutex::new(state)))
    }

    fn state(&self) -> std::sync::MutexGuard<'_, State> {
        self.0.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn writes(&self) -> Vec<Vec<u8>> {
        self.state().writes.clone()
    }

    fn with(self, change: impl FnOnce(&mut State)) -> Fake {
        change(&mut self.state());
        self
    }

    fn device(&self) -> UniTUt181aDevice {
        let mut d = UniTUt181aDevice::from_port(Box::new(self.clone())).expect("a meter");
        d.open().expect("open");
        d
    }

    fn reset_writes(&self) {
        self.state().writes.clear();
    }
}

impl State {
    fn deliver(&mut self, bytes: Vec<u8>, code: u8) {
        let nth = self.seen.iter().rev().filter(|c| **c == code).count();
        if let Some(&(_, _, reads)) = self.late.iter().find(|(c, n, _)| *c == code && *n == nth) {
            let at = self.reads + reads;
            self.by_reads.push_back((at, bytes));
            return;
        }
        if self.paced {
            let mut at = self.by_time.back().map_or_else(Instant::now, |(t, _)| *t);
            at = at.max(Instant::now()) + self.answer_after;
            if let Some((delayed, by)) = self.code_delay
                && delayed == code
            {
                at += by;
            }
            for byte in bytes {
                at += BYTE;
                self.by_time.push_back((at, byte));
            }
            return;
        }
        self.ready.extend(bytes);
        if let Some((_, head, tail)) = self.stale_behind.take_if(|(c, _, _)| *c == code) {
            self.ready.extend(head);
            self.by_reads.push_back((self.reads + 3, tail));
        }
    }

    fn command(&mut self, payload: &[u8]) {
        let code = payload[0];
        self.seen.push(code);
        if self.silent.contains(&code) {
            return;
        }
        if self.refused.contains(&code) {
            self.deliver(frame(b"\x01ER"), code);
            return;
        }
        let index = |at: usize| usize::from(u16::from_le_bytes([payload[at], payload[at + 1]]));
        let answer = match code {
            0x01 | 0x02 => frame(b"\x01OK"),
            0x05 => {
                self.monitor = payload[1] == 1;
                self.since_reading = 0;
                let mut out = frame(b"\x01OK");
                if self.monitor
                    && let Some(reading) = self.readings.pop_front()
                {
                    out.extend(reading);
                }
                if !self.monitor
                    && let Some(mut reading) = self.reading_before_off.clone()
                {
                    reading.extend(out);
                    out = reading;
                }
                out
            }
            0x07 => match self.saved.get(index(1).wrapping_sub(1)) {
                Some(payload) => frame(payload),
                None => frame(b"\x01ER"),
            },
            0x08 => frame(&[0x72, 0x08, self.saved.len() as u8, 0]),
            0x0c => match self.recordings.get(index(1).wrapping_sub(1)) {
                Some(rec) => frame(&rec_info(rec.0, rec.1, rec.2, rec.3)),
                None => frame(b"\x01ER"),
            },
            0x0d => {
                let n = index(1);
                let at = u32::from_le_bytes([payload[3], payload[4], payload[5], payload[6]]);
                match self.recordings.get(n.wrapping_sub(1)) {
                    Some(rec) => {
                        let total = self.have.map_or(rec.3, |h| h.min(rec.3));
                        let count = total.saturating_sub(at - 1).min(self.per);
                        let mut out = vec![0x05, count as u8];
                        for i in at..at + count {
                            out.extend_from_slice(&(n as f32 * 100.0 + i as f32).to_le_bytes());
                            out.push(0x10);
                            out.extend_from_slice(&0u32.to_le_bytes());
                        }
                        if self.cut_chunks {
                            out.pop();
                        }
                        frame(&out)
                    }
                    None => frame(b"\x01ER"),
                }
            }
            0x0e => frame(&[0x72, 0x0e, self.recordings.len() as u8, 0]),
            other => panic!("command {other:#x}"),
        };
        self.deliver(answer, code);
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
        let pause = self.state().each_read;
        std::thread::sleep(pause);
        let mut s = self.state();
        if let Some(why) = s.read_error {
            return Err(Error::device(why));
        }
        s.reads += 1;
        let reads = s.reads;
        while s.by_reads.front().is_some_and(|(at, _)| *at <= reads) {
            let (_, bytes) = s.by_reads.pop_front().expect("looked");
            s.ready.extend(bytes);
        }
        if s.monitor {
            s.since_reading += 1;
            if s.since_reading >= s.every {
                s.since_reading = 0;
                if let Some(reading) = s.readings.pop_front() {
                    s.ready.extend(reading);
                }
            }
        }
        let now = Instant::now();
        while s.by_time.front().is_some_and(|(at, _)| *at <= now) {
            let (_, byte) = s.by_time.pop_front().expect("looked");
            s.ready.push_back(byte);
        }
        if s.ready.is_empty() {
            drop(s);
            std::thread::sleep(timeout.min(Duration::from_millis(2)));
            return Ok(0);
        }
        let mut n = 0;
        while n < buf.len() {
            let Some(byte) = s.ready.pop_front() else { break };
            buf[n] = byte;
            n += 1;
        }
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut s = self.state();
        s.writes.push(data.to_vec());
        let payload = &data[4..data.len() - 2];
        s.command(payload);
        Ok(())
    }
    /// Drops only what has arrived: what is on its way goes on.
    fn purge(&mut self) -> Result<()> {
        let mut s = self.state();
        let now = Instant::now();
        while s.by_time.front().is_some_and(|(at, _)| *at <= now) {
            s.by_time.pop_front();
        }
        s.ready.clear();
        Ok(())
    }
}

fn unit8(text: &str) -> Vec<u8> {
    let mut out = text.as_bytes().to_vec();
    out.resize(8, 0);
    out
}

fn rec_info(name: &str, unit: &str, interval: u16, samples: u32) -> Vec<u8> {
    let mut out = vec![0x04];
    let mut field = name.as_bytes().to_vec();
    field.resize(11, 0);
    out.extend(field);
    out.extend(unit8(unit));
    out.extend_from_slice(&interval.to_le_bytes());
    out.extend_from_slice(&(u32::from(interval) * samples).to_le_bytes());
    out.extend_from_slice(&samples.to_le_bytes());
    for _ in 0..3 {
        out.extend_from_slice(&1f32.to_le_bytes());
        out.push(0x10);
    }
    out.extend_from_slice(&0u32.to_le_bytes());
    out
}

/// A measurement's payload, normal, of one value in volts at mode V DC.
fn reading(value: f32) -> Vec<u8> {
    let mut out = vec![0x02, 0, 0x01, 0x11, 0x31, 1];
    out.extend_from_slice(&value.to_le_bytes());
    out.push(0x30);
    out.extend(unit8("VDC"));
    out
}

fn saved_reading(value: f32) -> Vec<u8> {
    let mut out = vec![0x03, 0, 0, 0, 0];
    out.extend_from_slice(&reading(value)[1..]);
    out
}

fn readings(values: &[f32]) -> VecDeque<Vec<u8>> {
    values.iter().map(|v| frame(&reading(*v))).collect()
}

/// A fake with a scan's reading, and these next.
fn live(values: &[f32]) -> Fake {
    let mut all = vec![5.0];
    all.extend_from_slice(values);
    Fake::new().with(|s| s.readings = readings(&all))
}

fn drain(rx: &Rx<(usize, Packet)>) -> Vec<Packet> {
    let deadline = Instant::now() + CAPTURE_BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((_, Packet::End)) => return out,
            Ok((_, packet)) => out.push(packet),
            Err(e) => panic!("no End: {e:?} after {out:?}"),
        }
    }
}

fn capture(device: &mut UniTUt181aDevice) -> Vec<Packet> {
    let (tx, rx) = sync_channel(4096);
    device.start(PacketSink::new(tx, 0)).expect("start");
    drain(&rx)
}

fn values(packets: &[Packet]) -> Vec<(usize, f32)> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some((a.channels[0], a.samples[0])),
            _ => None,
        })
        .collect()
}

fn set_u64(device: &mut UniTUt181aDevice, key: &Key, value: u64) {
    device.set(key, &Value::U64(value), None).expect("a limit");
}

/// Waits, with a bound, for a condition on the fake.
fn wait_until(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + CAPTURE_BOUND;
    while !done() {
        assert!(Instant::now() < deadline, "no {what}");
        std::thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn a_scan_asks_for_a_reading_then_the_recordings() {
    let fake = live(&[]).with(|s| s.recordings = vec![("One", "VDC", 1, 3), ("Two", "mVAC", 5, 2)]);
    let d = fake.device();
    assert_eq!(
        fake.writes(),
        [
            monitor(true),
            monitor(false),
            protocol::get_recs_count(),
            get_rec_info(0),
            get_rec_info(1),
        ]
    );
    let ValueList::Str(names) = d.list(&Key::DataSource, None).unwrap() else { panic!() };
    assert_eq!(names, ["Live", "Save", "One", "Two"]);
    assert_eq!(d.get(&Key::Range, None).unwrap(), Value::from("auto"));
    assert_eq!(d.get(&Key::MeasuredQuantity, None).unwrap(), Value::Mq(Mq::Voltage, MqFlags::DC));
    assert_eq!(d.get(&Key::Datalog, None).unwrap(), Value::Bool(false));
    assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::from("Live"));
    assert_eq!(d.get(&Key::Conn, None).unwrap(), Value::from("/dev/ttyFAKE"));
    assert_eq!(d.info().conn.as_deref(), Some("/dev/ttyFAKE"));
}

#[test]
fn a_scan_takes_at_most_twenty_recordings() {
    let names: Vec<_> = (0..25).map(|_| ("R", "VDC", 1, 1)).collect();
    let fake = live(&[]).with(|s| s.recordings = names);
    let d = fake.device();
    let ValueList::Str(list) = d.list(&Key::DataSource, None).unwrap() else { panic!() };
    assert_eq!(list.len(), 22);
    let info = fake.writes().iter().filter(|w| w[4] == 0x0c).count();
    assert_eq!(info, 40);
}

#[test]
fn a_first_reading_may_take_a_while_and_not_for_ever() {
    // Over C's 100 ms, in reads that take 2 ms and a pause of 1 ms each, and well
    // inside the 2 s when sleeps run twelve times as long.
    let fake = live(&[]).with(|s| s.late = vec![(0x05, 1, 40)]);
    assert!(UniTUt181aDevice::from_port(Box::new(fake)).is_ok());
    let fake = Fake::new();
    let began = Instant::now();
    assert!(UniTUt181aDevice::from_port(Box::new(fake.clone())).is_err());
    assert!(began.elapsed() >= Duration::from_secs(2), "{:?}", began.elapsed());
    // No monitor off: C gives up there as well.
    assert_eq!(fake.writes(), [monitor(true)]);
}

#[test]
fn a_scan_fails_on_a_count_or_an_info_that_does_not_come() {
    for code in [0x0e, 0x0c] {
        let fake = live(&[]).with(|s| {
            s.recordings = vec![("One", "VDC", 1, 3)];
            s.silent.insert(code);
        });
        assert!(UniTUt181aDevice::from_port(Box::new(fake)).is_err(), "{code:#x}");
    }
    // An ER counts for nothing there either.
    let fake = live(&[]).with(|s| {
        s.refused.insert(0x0e);
    });
    assert!(UniTUt181aDevice::from_port(Box::new(fake)).is_err());
}

#[test]
fn a_silent_monitor_off_is_no_failure() {
    let fake = live(&[]).with(|s| s.late = vec![(0x05, 2, 100_000)]);
    assert!(UniTUt181aDevice::from_port(Box::new(fake)).is_ok());
}

#[test]
fn a_sample_limit_ends_live_with_monitor_off_last() {
    in_time(|| {
        let fake = live(&[1.0, 2.0, 3.0, 4.0]);
        let mut d = fake.device();
        fake.reset_writes();
        set_u64(&mut d, &Key::LimitSamples, 2);
        let packets = capture(&mut d);
        assert_eq!(values(&packets), [(0, 1.0), (0, 2.0)]);
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert_eq!(fake.writes(), [monitor(true), monitor(false)]);
        // The frames close.
        let begins = packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count();
        let ends = packets.iter().filter(|p| matches!(p, Packet::FrameEnd)).count();
        assert_eq!((begins, ends), (2, 2));
    });
}

#[test]
fn a_frame_limit_counts_frames() {
    in_time(|| {
        let fake = live(&[1.0, 2.0, 3.0, 4.0]);
        let mut d = fake.device();
        set_u64(&mut d, &Key::LimitFrames, 3);
        assert_eq!(values(&capture(&mut d)).len(), 3);
    });
}

/// A normal measurement with a first value and more behind it.
fn with_aux(value: f32, aux: f32) -> Vec<u8> {
    let mut out = vec![0x02, 0x02, 0x00, 0x11, 0x31, 1];
    for v in [value, aux] {
        out.extend_from_slice(&v.to_le_bytes());
        out.push(0x30);
        out.extend(unit8("VDC"));
    }
    out
}

#[test]
fn a_sample_limit_drops_the_rest_of_its_frame_but_a_frame_limit_does_not() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        fake.state().readings = [frame(&with_aux(1.0, 2.0))].into();
        set_u64(&mut d, &Key::LimitSamples, 1);
        assert_eq!(values(&capture(&mut d)), [(0, 1.0)]);
    });
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        fake.state().readings = [frame(&with_aux(1.0, 2.0))].into();
        set_u64(&mut d, &Key::LimitFrames, 1);
        assert_eq!(values(&capture(&mut d)), [(0, 1.0), (1, 2.0)]);
    });
}

#[test]
fn a_frame_that_fails_stays_open_and_the_next_closes_it() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        let mut cut = with_aux(1.0, 2.0);
        cut.truncate(cut.len() - 1);
        // Behind a failed frame a search takes one step per read that brought
        // bytes: the second reading waits for the third.
        fake.state().readings = [frame(&cut), frame(&reading(3.0)), frame(&reading(4.0))].into();
        set_u64(&mut d, &Key::LimitSamples, 2);
        let packets = capture(&mut d);
        assert_eq!(values(&packets), [(0, 1.0), (0, 3.0)]);
        let begins = packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count();
        let ends = packets.iter().filter(|p| matches!(p, Packet::FrameEnd)).count();
        // One for the first two and an empty one for what the third brought
        // behind the limit, as C's start of a frame does not look at the state.
        assert_eq!((begins, ends), (2, 2));
    });
}

#[test]
fn garbage_ahead_of_a_reading_is_passed_over() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        let frames = vec![vec![1, 2, 3, 4, 5, 6], frame(&reading(1.0)), frame(&reading(2.0))];
        fake.state().readings = frames.into();
        set_u64(&mut d, &Key::LimitSamples, 1);
        assert_eq!(values(&capture(&mut d)), [(0, 1.0)]);
    });
}

#[test]
fn a_flood_of_false_magics_cannot_hold_a_live_capture() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        // Each read brings more than a step of the search takes away: a frame
        // whose length no buffer holds fails at each of its false magics.
        let flood: Vec<u8> = [0xab, 0xcd, 0xff, 0xff].repeat(1500);
        let mut stream = vec![flood.clone(), flood];
        stream.extend((0..2000).map(|_| frame(&reading(1.0))));
        {
            let mut s = fake.state();
            s.readings = stream.into();
            s.every = 1;
        }
        set_u64(&mut d, &Key::LimitSamples, 1);
        assert_eq!(values(&capture(&mut d)).len(), 1);
    });
}

#[test]
fn a_time_limit_ends_a_silent_meter() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        fake.reset_writes();
        set_u64(&mut d, &Key::LimitMsec, 50);
        let began = Instant::now();
        assert_eq!(values(&capture(&mut d)).len(), 0);
        assert!(began.elapsed() >= Duration::from_millis(50));
        assert_eq!(fake.writes(), [monitor(true), monitor(false)]);
    });
}

#[test]
fn live_silence_ends_a_limit_of_samples_only() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        d.silence.live = Duration::from_millis(60);
        set_u64(&mut d, &Key::LimitSamples, 5);
        let began = Instant::now();
        capture(&mut d);
        assert!(began.elapsed() >= Duration::from_millis(60));
        assert_eq!(fake.writes().last(), Some(&monitor(false)));
    });
}

#[test]
fn what_comes_starts_the_silence_again() {
    in_time(|| {
        // Each read brings a reading, however late it runs, and they take longer
        // than the bound all told.
        let fake = live(&[]);
        let mut d = fake.device();
        d.silence.live = Duration::from_millis(50);
        let many: Vec<f32> = (0..40).map(|i| i as f32).collect();
        {
            let mut s = fake.state();
            s.readings = readings(&many);
            s.every = 1;
            s.each_read = Duration::from_millis(5);
        }
        set_u64(&mut d, &Key::LimitSamples, 40);
        assert_eq!(values(&capture(&mut d)).len(), 40);
    });
}

#[test]
fn the_bounds_on_silence_are_what_they_say() {
    assert_eq!(DOWNLOAD_SILENCE, Duration::from_secs(5));
    assert_eq!(LIVE_SILENCE, Duration::from_secs(10));
    assert_eq!(REPLY_WAIT, Duration::from_millis(500));
    assert_eq!(SETTLE_MAX, Duration::from_secs(3));
    assert_eq!(QUIET, Duration::from_millis(100));
    assert_eq!(FIRST_READING, Duration::from_secs(2));
}

#[test]
fn silence_does_not_end_a_time_limit_longer_than_it_or_a_continuous_capture() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        d.silence.live = Duration::from_millis(30);
        set_u64(&mut d, &Key::LimitMsec, 200);
        set_u64(&mut d, &Key::LimitSamples, 5);
        let began = Instant::now();
        capture(&mut d);
        assert!(began.elapsed() >= Duration::from_millis(200), "{:?}", began.elapsed());
    });
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        d.silence.live = Duration::from_millis(10);
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(rx.recv_timeout(CAPTURE_BOUND), Ok((_, Packet::Header { .. }))));
        match rx.recv_timeout(Duration::from_millis(100)) {
            Err(RecvTimeoutError::Timeout) => {}
            other => panic!("{other:?}"),
        }
        d.stop().unwrap();
        drain(&rx);
    });
}

#[test]
fn a_stop_ends_live_on_a_stream_that_goes_on() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        let many: Vec<f32> = (0..2000).map(|i| i as f32).collect();
        fake.state().readings = readings(&many);
        fake.reset_writes();
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Something has come, so the stream is really running.
        let deadline = Instant::now() + CAPTURE_BOUND;
        loop {
            let (_, packet) = rx.recv_timeout(CAPTURE_BOUND).unwrap();
            if matches!(packet, Packet::Analog(_)) {
                break;
            }
            assert!(Instant::now() < deadline);
        }
        d.stop().unwrap();
        drain(&rx);
        assert_eq!(fake.writes().last(), Some(&monitor(false)));
        assert!(!fake.state().readings.is_empty(), "there was more to read");
    });
}

#[test]
fn a_saved_download_asks_for_each_in_turn_and_ends() {
    in_time(|| {
        let fake = live(&[]).with(|s| {
            s.saved = vec![saved_reading(1.0), saved_reading(2.0), saved_reading(3.0)];
        });
        let mut d = fake.device();
        fake.reset_writes();
        d.set(&Key::DataSource, &Value::from("Save"), None).unwrap();
        let packets = capture(&mut d);
        assert_eq!(values(&packets), [(0, 1.0), (0, 2.0), (0, 3.0)]);
        assert_eq!(
            fake.writes(),
            [get_saved_count(), get_saved(0), get_saved(1), get_saved(2)],
            "no monitor mode and no more requests"
        );
    });
}

#[test]
fn a_limit_stops_the_requests_though_more_are_saved() {
    in_time(|| {
        let fake = live(&[]).with(|s| s.saved = (0..9).map(|i| saved_reading(i as f32)).collect());
        let mut d = fake.device();
        fake.reset_writes();
        d.set(&Key::DataSource, &Value::from("Save"), None).unwrap();
        set_u64(&mut d, &Key::LimitSamples, 2);
        assert_eq!(values(&capture(&mut d)).len(), 2);
        assert_eq!(fake.writes().len(), 3, "the count and two");
    });
}

#[test]
fn an_er_ends_a_download() {
    in_time(|| {
        // None saved: C asks for the first anyway, and waits for ever on its ER.
        // The bound would end it too, later than this test's.
        let fake = live(&[]);
        let mut d = fake.device();
        d.silence.download = 10 * CAPTURE_BOUND;
        d.set(&Key::DataSource, &Value::from("Save"), None).unwrap();
        assert_eq!(values(&capture(&mut d)).len(), 0);
    });
}

#[test]
fn a_download_that_goes_silent_ends_after_its_bound() {
    in_time(|| {
        let fake = live(&[]).with(|s| {
            s.saved = vec![saved_reading(1.0)];
            s.silent.insert(0x07);
        });
        let mut d = fake.device();
        d.silence.download = Duration::from_millis(60);
        d.set(&Key::DataSource, &Value::from("Save"), None).unwrap();
        let began = Instant::now();
        assert_eq!(values(&capture(&mut d)).len(), 0);
        assert!(began.elapsed() >= Duration::from_millis(60));
    });
}

#[test]
fn a_saved_answer_cut_asks_for_no_more() {
    in_time(|| {
        let mut cut = saved_reading(1.0);
        cut.truncate(cut.len() - 4);
        let fake = live(&[]).with(|s| s.saved = vec![cut, saved_reading(2.0)]);
        let mut d = fake.device();
        d.silence.download = Duration::from_millis(60);
        fake.reset_writes();
        d.set(&Key::DataSource, &Value::from("Save"), None).unwrap();
        capture(&mut d);
        assert_eq!(fake.writes(), [get_saved_count(), get_saved(0)]);
    });
}

#[test]
fn a_recording_comes_in_chunks_of_its_own() {
    in_time(|| {
        let fake = live(&[]).with(|s| {
            s.recordings = vec![("One", "VDC", 2, 5), ("Two", "mVAC", 7, 5)];
        });
        let mut d = fake.device();
        fake.reset_writes();
        d.set(&Key::DataSource, &Value::from("Two"), None).unwrap();
        let packets = capture(&mut d);
        let got = values(&packets);
        assert_eq!(got.len(), 5);
        // Recording 2's values: the C driver reads recording 1 past the first.
        assert!((got[4].1 - 0.205).abs() < 1e-5, "{got:?}");
        assert_eq!(
            fake.writes(),
            [get_rec_info(1), get_rec_samples(1, 0), get_rec_samples(1, 2), get_rec_samples(1, 4)]
        );
        // The interval follows the header. The unit and its scale are the info's.
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert_eq!(packets[1], Packet::Meta(vec![(Key::SampleInterval, Value::U64(7))]));
        let Packet::Analog(a) = &packets[3] else { panic!("{:?}", packets[3]) };
        assert_eq!((a.mq, a.unit, a.flags), (Mq::Voltage, Unit::Volt, MqFlags::AC));
        assert_eq!(a.digits, 1 + 3);
    });
}

#[test]
fn a_limit_in_a_chunk_drops_the_rest_and_asks_for_no_more() {
    in_time(|| {
        let fake = live(&[]).with(|s| s.recordings = vec![("One", "VDC", 1, 9)]);
        let mut d = fake.device();
        fake.reset_writes();
        d.set(&Key::DataSource, &Value::from("1"), None).unwrap();
        set_u64(&mut d, &Key::LimitSamples, 3);
        let packets = capture(&mut d);
        assert_eq!(values(&packets).len(), 3);
        assert_eq!(fake.writes().len(), 3, "the info and two chunks");
        // Nothing goes out of what is past the limit, not even empty frames.
        let begins = packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count();
        assert_eq!(begins, 3);
    });
}

#[test]
fn an_empty_chunk_ends_where_c_asks_for_ever() {
    in_time(|| {
        let fake = live(&[]).with(|s| {
            s.recordings = vec![("One", "VDC", 1, 9)];
            s.have = Some(2);
        });
        let mut d = fake.device();
        fake.reset_writes();
        d.set(&Key::DataSource, &Value::from("One"), None).unwrap();
        assert_eq!(values(&capture(&mut d)).len(), 2);
        assert_eq!(fake.writes().len(), 3, "the info, the samples, the one with none");
    });
}

#[test]
fn a_chunk_cut_keeps_what_came_and_asks_for_no_more() {
    in_time(|| {
        let fake = live(&[]).with(|s| s.recordings = vec![("One", "VDC", 1, 9)]);
        let mut d = fake.device();
        d.silence.download = Duration::from_millis(60);
        fake.reset_writes();
        fake.state().cut_chunks = true;
        d.set(&Key::DataSource, &Value::from("One"), None).unwrap();
        // Two said, and the second short of a byte.
        assert_eq!(values(&capture(&mut d)).len(), 1);
        assert_eq!(fake.writes().len(), 2, "the info and one chunk");
    });
}

#[test]
fn what_lay_in_the_queue_answers_no_setting_and_is_no_reading() {
    let fake = live(&[]);
    let mut d = fake.device();
    {
        // Its own answer comes behind: in one read the last would win anyway.
        let mut s = fake.state();
        s.paced = true;
        s.ready.extend(frame(b"\x01ER"));
    }
    d.set(&Key::Range, &Value::from("6V"), None).expect("its own OK");
    fake.state().paced = false;
    in_time(move || {
        fake.state().ready.extend(frame(&reading(9.0)));
        fake.state().readings = readings(&[1.0]);
        set_u64(&mut d, &Key::LimitSamples, 1);
        assert_eq!(values(&capture(&mut d)), [(0, 1.0)]);
    });
}

#[test]
fn a_recording_with_no_samples_ends_at_its_first_answer() {
    in_time(|| {
        let fake = live(&[]).with(|s| s.recordings = vec![("One", "VDC", 1, 0)]);
        let mut d = fake.device();
        d.set(&Key::DataSource, &Value::from("One"), None).unwrap();
        assert_eq!(values(&capture(&mut d)).len(), 0);
    });
}

#[test]
fn a_start_says_how_it_failed() {
    let fake = live(&[]).with(|s| {
        s.recordings = vec![("One", "VDC", 1, 1)];
        s.saved = vec![saved_reading(1.0)];
    });
    let mut d = fake.device();
    d.link().reply_wait = Duration::from_millis(30);
    let start = |d: &mut UniTUt181aDevice, source: &str| {
        d.set(&Key::DataSource, &Value::from(source), None).unwrap();
        let (tx, _rx) = sync_channel(16);
        d.start(PacketSink::new(tx, 0)).unwrap_err().to_string()
    };
    // Refused, within a wait far too short for this to be its expiry.
    fake.state().refused.insert(0x08);
    fake.state().refused.insert(0x0c);
    let refused = "device error: the meter refused the request";
    assert_eq!(start(&mut d, "Save"), refused);
    assert_eq!(start(&mut d, "One"), refused);
    {
        let mut s = fake.state();
        s.refused.clear();
        s.silent.insert(0x08);
        s.silent.insert(0x0c);
    }
    assert_eq!(start(&mut d, "Save"), "timeout");
    assert_eq!(start(&mut d, "One"), "timeout");
}

#[test]
fn an_er_ends_a_wait_for_data_at_once() {
    let fake = live(&[]);
    let d = fake.device();
    // A wait this long would fail the test's bound if it ran out.
    d.link().reply_wait = 2 * CAPTURE_BOUND;
    fake.state().refused.insert(0x0e);
    in_time(move || {
        assert_eq!(
            d.list(&Key::DataSource, None).unwrap_err().to_string(),
            "device error: the meter refused the request"
        );
    });
}

#[test]
fn a_start_fails_on_a_count_or_an_info_that_does_not_come() {
    let fake = live(&[]).with(|s| {
        s.recordings = vec![("One", "VDC", 1, 1)];
        s.saved = vec![saved_reading(1.0)];
    });
    let mut d = fake.device();
    d.link().reply_wait = Duration::from_millis(30);
    fake.state().silent.insert(0x08);
    fake.state().silent.insert(0x0c);
    for source in ["Save", "One"] {
        d.set(&Key::DataSource, &Value::from(source), None).unwrap();
        let (tx, _rx) = sync_channel(16);
        assert!(d.start(PacketSink::new(tx, 0)).is_err(), "{source}");
    }
}

#[test]
fn a_read_that_fails_ends_a_capture() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        fake.reset_writes();
        fake.state().read_error = Some("hung up");
        assert_eq!(values(&capture(&mut d)).len(), 0);
        assert_eq!(fake.writes(), [monitor(true)], "nothing more to a port that failed");
    });
}

#[test]
fn a_full_queue_holds_no_one_for_ever() {
    in_time(|| {
        // Room for the header alone, and nobody reads it.
        let many: Vec<f32> = (0..500).map(|i| i as f32).collect();
        let fake = live(&[]);
        let mut d = fake.device();
        fake.state().readings = readings(&many);
        let (tx, rx) = sync_channel(1);
        let reads = fake.state().reads;
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The header fills the queue, and the first reading has to wait on it.
        wait_until("a read", || fake.state().reads > reads);
        d.stop().unwrap();
        d.close().unwrap();
        assert!(!d.busy());
        drop(rx);
    });
}

#[test]
fn a_start_after_an_end_stuck_on_a_full_queue_does_not_hang() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        set_u64(&mut d, &Key::LimitMsec, 1);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Header in the queue, End waiting; the capture itself is over.
        wait_until("the capture's end", || !d.busy());
        let (tx2, rx2) = sync_channel(4096);
        d.start(PacketSink::new(tx2, 0)).unwrap();
        drain(&rx2);
        drop(rx);
    });
}

#[test]
fn a_capture_and_a_second_one_on_an_open_device() {
    in_time(|| {
        let fake = live(&[1.0, 2.0]);
        let mut d = fake.device();
        set_u64(&mut d, &Key::LimitSamples, 1);
        assert_eq!(values(&capture(&mut d)), [(0, 1.0)]);
        assert_eq!(values(&capture(&mut d)), [(0, 2.0)]);
    });
}

/// What a request gets on the line, with what the last capture left in flight.
#[test]
fn a_stop_during_a_saved_answer_leaves_nothing_for_the_next_capture() {
    in_time(|| {
        // The next capture's answers come at the line's rate, a frame in 25 ms, and
        // then later than the quiet that follows what is owed.
        for latency in [0u64, 300] {
            let fake =
                live(&[]).with(|s| s.saved = (1..=4).map(|i| saved_reading(i as f32)).collect());
            let mut d = fake.device();
            d.set(&Key::DataSource, &Value::from("Save"), None).unwrap();
            {
                let mut s = fake.state();
                s.paced = true;
                s.answer_after = Duration::from_millis(latency);
            }
            let (tx, rx) = sync_channel(4096);
            d.start(PacketSink::new(tx, 0)).unwrap();
            // Its first answer has come, so the next request is out and answering.
            loop {
                let (_, packet) = rx.recv_timeout(CAPTURE_BOUND).unwrap();
                if matches!(packet, Packet::Analog(_)) {
                    break;
                }
            }
            d.stop().unwrap();
            drain(&rx);
            fake.reset_writes();
            let got = values(&capture(&mut d));
            assert_eq!(got, [(0, 1.0), (0, 2.0), (0, 3.0), (0, 4.0)], "{latency} ms");
            assert_eq!(fake.writes().len(), 5, "one chain of requests: {:?}", fake.writes());
        }
    });
}

#[test]
fn a_setting_right_after_a_capture_gets_its_own_answer() {
    in_time(|| {
        let fake = live(&[1.0]);
        let mut d = fake.device();
        set_u64(&mut d, &Key::LimitSamples, 1);
        fake.state().paced = true;
        fake.state().answer_after = Duration::from_millis(40);
        capture(&mut d);
        // The monitor off's OK is on its way, and this setting is refused: only
        // its own ER may answer it.
        fake.state().refused.insert(0x02);
        assert!(d.set(&Key::Range, &Value::from("6V"), None).is_err());
    });
}

#[test]
fn a_refused_setting_is_refused_however_late_the_monitor_off_is_answered() {
    in_time(|| {
        for latency in [40u64, 110, 300] {
            let fake = live(&[1.0]);
            let mut d = fake.device();
            set_u64(&mut d, &Key::LimitSamples, 1);
            {
                let mut s = fake.state();
                s.paced = true;
                s.answer_after = Duration::from_millis(latency);
            }
            capture(&mut d);
            fake.state().refused.insert(0x02);
            let r = d.set(&Key::Range, &Value::from("6V"), None);
            assert!(r.is_err(), "{latency} ms: {r:?}");
            assert_eq!(d.get(&Key::Range, None).unwrap(), Value::from("auto"), "{latency} ms");
        }
    });
}

#[test]
fn a_reading_ahead_of_the_monitor_off_s_ok_answers_nothing() {
    in_time(|| {
        // A scan meets one, and takes it for a reading.
        let mut mv = reading(1.0);
        mv[3..5].copy_from_slice(&0x2111u16.to_le_bytes());
        let fake = live(&[]).with(|s| s.reading_before_off = Some(frame(&mv)));
        let mut d = fake.device();
        assert_eq!(
            d.get(&Key::MeasuredQuantity, None).unwrap(),
            Value::Mq(Mq::Voltage, MqFlags::AC)
        );
        // A capture leaves one with its OK on the way, and the OK does not answer
        // the next setting, nor is the reading any answer to it.
        set_u64(&mut d, &Key::LimitSamples, 1);
        {
            let mut s = fake.state();
            s.readings = readings(&[1.0]);
            s.paced = true;
            s.answer_after = Duration::from_millis(150);
        }
        assert_eq!(values(&capture(&mut d)), [(0, 1.0)]);
        fake.state().refused.insert(0x02);
        assert!(d.set(&Key::Range, &Value::from("6V"), None).is_err());
    });
}

#[test]
fn a_live_capture_takes_no_stale_saved_answer() {
    in_time(|| {
        for latency in [40u64, 150] {
            let fake = live(&[9.0, 9.0]).with(|s| {
                s.saved = (1..=4).map(|i| saved_reading(i as f32)).collect();
            });
            let mut d = fake.device();
            d.set(&Key::DataSource, &Value::from("Save"), None).unwrap();
            {
                let mut s = fake.state();
                s.paced = true;
                s.answer_after = Duration::from_millis(latency);
                s.every = 1_000_000;
            }
            let (tx, rx) = sync_channel(4096);
            d.start(PacketSink::new(tx, 0)).unwrap();
            loop {
                let (_, packet) = rx.recv_timeout(CAPTURE_BOUND).unwrap();
                if matches!(packet, Packet::Analog(_)) {
                    break;
                }
            }
            d.stop().unwrap();
            drain(&rx);
            d.set(&Key::DataSource, &Value::from("Live"), None).unwrap();
            set_u64(&mut d, &Key::LimitSamples, 1);
            assert_eq!(values(&capture(&mut d)), [(0, 9.0)], "{latency} ms");
        }
    });
}

#[test]
fn each_capture_takes_its_own_kind_of_packet_alone() {
    in_time(|| {
        // What no wait has dropped: a saved answer and a chunk in a live capture.
        let chunk = frame(&[0x05, 1, 0, 0, 0x10, 0x41, 0x10, 0, 0, 0, 0]);
        let fake = live(&[]);
        let mut d = fake.device();
        fake.state().readings =
            [frame(&saved_reading(3.0)), chunk.clone(), frame(&reading(1.0))].into();
        set_u64(&mut d, &Key::LimitSamples, 1);
        assert_eq!(values(&capture(&mut d)), [(0, 1.0)]);
        // A live reading and a chunk in a saved download.
        let fake = live(&[]).with(|s| s.saved = vec![saved_reading(1.0), saved_reading(2.0)]);
        let mut d = fake.device();
        d.set(&Key::DataSource, &Value::from("Save"), None).unwrap();
        let strays = [frame(&reading(8.0)), chunk.clone()].concat();
        fake.state().stale_behind = Some((0x07, strays, Vec::new()));
        assert_eq!(values(&capture(&mut d)), [(0, 1.0), (0, 2.0)]);
        // A live reading and a saved answer in a recording.
        let fake = live(&[]).with(|s| s.recordings = vec![("One", "VDC", 1, 2)]);
        let mut d = fake.device();
        d.set(&Key::DataSource, &Value::from("One"), None).unwrap();
        let strays = [frame(&reading(8.0)), frame(&saved_reading(8.0))].concat();
        fake.state().stale_behind = Some((0x0d, strays, Vec::new()));
        assert_eq!(values(&capture(&mut d)), [(0, 101.0), (0, 102.0)]);
    });
}

#[test]
fn a_late_record_info_labels_no_other_recording() {
    in_time(|| {
        // One's info is on its way for the list that gave up on it, later than two
        // reply waits at the line: then the start asks for Two's and gets it.
        let fake = live(&[]).with(|s| {
            s.recordings = vec![("One", "VDC", 1, 3), ("Two", "mVAC", 1, 2)];
        });
        let mut d = fake.device();
        d.set(&Key::DataSource, &Value::from("Two"), None).unwrap();
        d.link().reply_wait = Duration::from_millis(20);
        {
            let mut s = fake.state();
            s.paced = true;
            s.code_delay = Some((0x0c, Duration::from_millis(200)));
        }
        assert!(d.list(&Key::DataSource, None).is_err());
        {
            let mut s = fake.state();
            s.code_delay = None;
        }
        d.link().reply_wait = Duration::from_millis(500);
        d.link().settle_max = Duration::from_millis(1);
        d.link().owed = None;
        // The late info comes now: as an answer it says One; refused, or the right
        // unit. Either way no volts for millivolts.
        let (tx, rx) = sync_channel(4096);
        match d.start(PacketSink::new(tx, 0)) {
            Err(e) => {
                assert_eq!(e.to_string(), "device error: the meter's info is of another recording");
                assert_eq!(d.link().names[1], "Two", "the list's name stays, for the next try");
            }
            Ok(()) => {
                for packet in drain(&rx) {
                    if let Packet::Analog(a) = packet {
                        assert_eq!(a.flags, MqFlags::AC, "{:?}", a.samples);
                    }
                }
            }
        }
    });
}

#[test]
fn an_info_of_another_name_is_refused() {
    let fake = live(&[]).with(|s| {
        s.recordings = vec![("One", "VDC", 1, 3), ("Two", "mVAC", 1, 2)];
    });
    let mut d = fake.device();
    d.set(&Key::DataSource, &Value::from("Two"), None).unwrap();
    fake.state().recordings[1].0 = "Other";
    let (tx, _rx) = sync_channel(16);
    let e = d.start(PacketSink::new(tx, 0)).unwrap_err();
    assert_eq!(e.to_string(), "device error: the meter's info is of another recording");
    assert_eq!(d.link().names[1], "Two");
}

#[test]
fn a_selected_recording_the_list_has_lost_is_no_panic() {
    let fake = live(&[]).with(|s| {
        s.recordings = vec![("One", "VDC", 1, 3), ("Two", "VDC", 1, 2), ("Three", "VDC", 1, 2)];
    });
    let mut d = fake.device();
    d.set(&Key::DataSource, &Value::from("Three"), None).unwrap();
    fake.state().recordings.truncate(1);
    d.list(&Key::DataSource, None).unwrap();
    assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::from("Three"));
    let (tx, _rx) = sync_channel(16);
    assert_eq!(
        d.start(PacketSink::new(tx, 0)).unwrap_err().to_string(),
        "device error: no such recording"
    );
}

#[test]
fn a_mode_set_leaves_no_range_of_the_last_mode() {
    let fake = live(&[]);
    let mut d = fake.device();
    d.set(&Key::Range, &Value::from("60V"), None).unwrap();
    assert_eq!(d.get(&Key::Range, None).unwrap(), Value::from("60V"));
    d.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Resistance, MqFlags::empty()), None).unwrap();
    assert_eq!(d.get(&Key::Range, None).unwrap(), Value::from("auto"));
}

#[test]
fn a_line_that_never_falls_silent_holds_a_setting_for_a_bound() {
    in_time(|| {
        let fake = live(&[1.0]);
        let mut d = fake.device();
        set_u64(&mut d, &Key::LimitSamples, 1);
        capture(&mut d);
        d.link().settle_max = Duration::from_millis(100);
        {
            // A stream past the test's own bound, if nothing else ended the wait.
            let mut s = fake.state();
            s.readings = (0..20_000).map(|_| frame(&reading(2.0))).collect();
            s.every = 1;
            s.each_read = Duration::from_millis(4);
            s.monitor = true;
        }
        d.set(&Key::Range, &Value::from("6V"), None).unwrap();
        assert!(fake.state().readings.len() > 1000, "well before they ran out");
    });
}

#[test]
fn a_settle_waits_for_the_quiet_when_nothing_is_known_to_be_owed() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        {
            // A late ER on its way, ahead of the setting's own OK.
            let mut s = fake.state();
            s.paced = true;
            s.answer_after = Duration::from_millis(200);
            let at = Instant::now() + Duration::from_millis(40);
            s.by_time.extend(frame(b"\x01ER").into_iter().map(|b| (at, b)));
        }
        d.link().unsettled = true;
        d.set(&Key::Range, &Value::from("6V"), None).expect("its own OK");
    });
}

#[test]
fn a_settle_leaves_no_head_of_a_frame_to_swallow_the_next_answer() {
    let fake = live(&[]);
    let d = fake.device();
    {
        let mut link = d.link();
        // A head that says more is to come than any answer has.
        link.rx.push(&[0xab, 0xcd, 0xe8, 0x03, 0x72]);
        link.unsettled = true;
    }
    assert!(d.list(&Key::DataSource, None).is_ok());
}

#[test]
fn the_head_of_a_frame_left_behind_makes_no_sample_with_its_tail() {
    let stale = frame(&reading(9.0));
    let (head, tail) = stale.split_at(10);
    in_time({
        let (head, tail) = (head.to_vec(), tail.to_vec());
        move || {
            // In a live capture: the head kept from a wait, the tail still to come.
            let fake = live(&[]);
            let mut d = fake.device();
            d.link().rx.push(&head);
            {
                let mut s = fake.state();
                let at = s.reads;
                s.by_reads.push_back((at + 1, tail.clone()));
                s.readings = readings(&[1.0, 1.0, 1.0]);
                // The meter's answer comes behind the tail.
                s.paced = true;
                s.answer_after = Duration::from_millis(30);
            }
            set_u64(&mut d, &Key::LimitSamples, 1);
            assert_eq!(values(&capture(&mut d)), [(0, 1.0)]);
            // In a download: behind the count, ahead of the first answer.
            let fake = live(&[]).with(|s| s.saved = vec![saved_reading(1.0)]);
            let mut d = fake.device();
            d.set(&Key::DataSource, &Value::from("Save"), None).unwrap();
            let saved = frame(&saved_reading(9.0));
            let (head, tail) = saved.split_at(10);
            fake.state().stale_behind = Some((0x08, head.to_vec(), tail.to_vec()));
            assert_eq!(values(&capture(&mut d)), [(0, 1.0)]);
        }
    });
}

#[test]
fn a_late_ok_does_not_answer_the_next_setting() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        d.link().reply_wait = Duration::from_millis(20);
        fake.state().paced = true;
        // Later than the quiet too, so it is what is owed that holds the next one.
        fake.state().answer_after = Duration::from_millis(250);
        // The first has its OK late, and fails unless a stall has let it come. The
        // second is refused.
        let _ = d.set(&Key::Range, &Value::from("6V"), None);
        {
            let mut s = fake.state();
            s.answer_after = Duration::ZERO;
            s.refused.insert(0x02);
        }
        d.link().reply_wait = Duration::from_millis(500);
        assert!(d.set(&Key::Range, &Value::from("60V"), None).is_err());
    });
}

#[test]
fn a_late_record_info_does_not_answer_the_next_list() {
    in_time(|| {
        let fake = live(&[]).with(|s| s.recordings = vec![("One", "VDC", 1, 1)]);
        let d = fake.device();
        d.link().reply_wait = Duration::from_millis(20);
        fake.state().paced = true;
        fake.state().answer_after = Duration::from_millis(50);
        // It fails, unless a stall has let the answer come.
        let _ = d.list(&Key::DataSource, None);
        fake.state().answer_after = Duration::ZERO;
        d.link().reply_wait = Duration::from_millis(500);
        // Two recordings by now: a count of one from the last round would show.
        fake.state().recordings.push(("Two", "VDC", 1, 1));
        let ValueList::Str(names) = d.list(&Key::DataSource, None).unwrap() else { panic!() };
        assert_eq!(names, ["Live", "Save", "One", "Two"]);
    });
}

#[test]
fn settings_go_out_and_show() {
    let fake = live(&[]);
    let mut d = fake.device();
    fake.reset_writes();
    d.set(&Key::Range, &Value::from("60V"), None).unwrap();
    d.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Voltage, MqFlags::AC), None).unwrap();
    d.set(&Key::Range, &Value::from("auto"), None).unwrap();
    assert_eq!(
        fake.writes(),
        [protocol::set_range(2), protocol::set_mode(0x1111), protocol::set_range(0)]
    );
    assert_eq!(d.get(&Key::Range, None).unwrap(), Value::from("auto"));
    assert_eq!(d.get(&Key::MeasuredQuantity, None).unwrap(), Value::Mq(Mq::Voltage, MqFlags::AC));
    d.set(&Key::Range, &Value::from("6V"), None).unwrap();
    assert_eq!(d.get(&Key::Range, None).unwrap(), Value::from("6V"));
}

#[test]
fn settings_that_fail() {
    let fake = live(&[]);
    let mut d = fake.device();
    fake.state().refused.insert(0x01);
    fake.state().refused.insert(0x02);
    let quantity = Value::Mq(Mq::Voltage, MqFlags::AC);
    let text = |r: Result<()>| r.unwrap_err().to_string();
    let refused = "device error: the meter refused the request";
    assert_eq!(text(d.set(&Key::Range, &Value::from("6V"), None)), refused);
    assert_eq!(text(d.set(&Key::MeasuredQuantity, &quantity, None)), refused);
    assert_eq!(d.get(&Key::Range, None).unwrap(), Value::from("auto"), "not what was refused");
    let fake = live(&[]);
    let mut d = fake.device();
    d.link().reply_wait = Duration::from_millis(20);
    fake.state().silent.insert(0x02);
    assert_eq!(text(d.set(&Key::Range, &Value::from("6V"), None)), "timeout");
    fake.reset_writes();
    // What is refused here sends nothing.
    assert_eq!(
        text(d.set(&Key::Range, &Value::from("7V"), None)),
        "invalid argument: invalid value for range"
    );
    let none = Value::Mq(Mq::Power, MqFlags::empty());
    assert_eq!(
        text(d.set(&Key::MeasuredQuantity, &none, None)),
        "not supported: measured_quantity"
    );
    assert_eq!(text(d.set(&Key::Datalog, &Value::Bool(true), None)), "not supported: datalog");
    assert_eq!(fake.writes(), [[0u8; 0]; 0]);
}

#[test]
fn a_range_cannot_be_chosen_in_a_mode_with_one() {
    let mut fixed = reading(1.0);
    fixed[3..5].copy_from_slice(&0xa111u16.to_le_bytes());
    let fake = Fake::new().with(|s| s.readings = [frame(&fixed)].into());
    let mut d = fake.device();
    fake.reset_writes();
    assert!(matches!(d.set(&Key::Range, &Value::from("20A"), None), Err(Error::NotSupported(_))));
    assert_eq!(fake.writes(), [[0u8; 0]; 0]);
}

#[test]
fn settings_and_lists_in_a_capture_leave_the_wire_alone() {
    in_time(|| {
        let fake = live(&[]);
        let mut d = fake.device();
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        wait_until("a capture", || d.busy());
        fake.reset_writes();
        assert!(matches!(d.set(&Key::Range, &Value::from("6V"), None), Err(Error::Busy)));
        let quantity = Value::Mq(Mq::Voltage, MqFlags::AC);
        assert!(matches!(d.set(&Key::MeasuredQuantity, &quantity, None), Err(Error::Busy)));
        assert!(d.list(&Key::DataSource, None).is_ok());
        assert_eq!(fake.writes(), [[0u8; 0]; 0]);
        // A second start is refused, and a get answers.
        let (tx2, _rx2) = sync_channel(1);
        assert!(matches!(d.start(PacketSink::new(tx2, 0)), Err(Error::Busy)));
        assert_eq!(d.get(&Key::Range, None).unwrap(), Value::from("auto"));
        d.stop().unwrap();
        drain(&rx);
    });
}

#[test]
fn data_sources_by_name_and_by_number() {
    let fake = live(&[])
        .with(|s| s.recordings = vec![("Same", "V", 1, 1), ("Same", "V", 1, 1), ("9", "V", 1, 1)]);
    let mut d = fake.device();
    let set =
        |d: &mut UniTUt181aDevice, text: &str| d.set(&Key::DataSource, &Value::from(text), None);
    let source = |d: &UniTUt181aDevice| d.source;
    set(&mut d, "Save").unwrap();
    assert_eq!(source(&d), SOURCE_SAVE);
    assert_eq!(d.get(&Key::DataSource, None).unwrap(), Value::from("Save"));
    set(&mut d, "Live").unwrap();
    assert_eq!(source(&d), SOURCE_LIVE);
    set(&mut d, "Same").unwrap();
    assert_eq!(source(&d), SOURCE_RECORDINGS, "the first of a name");
    set(&mut d, "2").unwrap();
    assert_eq!(source(&d), SOURCE_RECORDINGS + 1);
    set(&mut d, " 2 ").unwrap();
    assert_eq!(source(&d), SOURCE_RECORDINGS + 1);
    set(&mut d, "9").unwrap();
    assert_eq!(source(&d), SOURCE_RECORDINGS + 2, "a name goes ahead of a number");
    for bad in ["0", "4", "-1", "x", "", "2x", "99999999999"] {
        assert!(matches!(set(&mut d, bad), Err(Error::Arg(_))), "'{bad}'");
    }
}

#[test]
fn c_atoi_takes_what_sr_atoi_takes() {
    assert_eq!(c_atoi("12"), Some(12));
    assert_eq!(c_atoi("  12  "), Some(12));
    assert_eq!(c_atoi("+12"), Some(12));
    assert_eq!(c_atoi("-3"), Some(-3));
    assert_eq!(c_atoi(""), Some(0));
    assert_eq!(c_atoi("x"), None);
    assert_eq!(c_atoi("1x"), None);
    assert_eq!(c_atoi("+"), None);
    assert_eq!(c_atoi("2147483647"), Some(i32::MAX));
    assert_eq!(c_atoi("2147483648"), None);
}

#[test]
fn a_recording_flag_shows_as_datalog() {
    let mut rec = reading(1.0);
    rec[2] = 0x20;
    let fake = Fake::new().with(|s| s.readings = [frame(&rec)].into());
    let d = fake.device();
    assert_eq!(d.get(&Key::Datalog, None).unwrap(), Value::Bool(true));
    assert_eq!(d.get(&Key::Range, None).unwrap(), Value::from("6V"));
}

#[test]
fn limits_are_stored_and_0_refused() {
    let fake = live(&[]);
    let mut d = fake.device();
    for key in [Key::LimitSamples, Key::LimitFrames, Key::LimitMsec] {
        assert_eq!(d.get(&key, None).unwrap(), Value::U64(0));
        set_u64(&mut d, &key, 7);
        assert_eq!(d.get(&key, None).unwrap(), Value::U64(7));
        assert!(d.set(&key, &Value::U64(0), None).is_err());
    }
}

#[test]
fn the_tables_of_a_device() {
    let fake = live(&[]);
    let d = fake.device();
    let names: Vec<&str> = d.channels().iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["P1", "P2", "P3", "P4", "bar"]);
    assert!(
        matches!(d.list(&Key::MeasuredQuantity, None).unwrap(), ValueList::Mq(l) if l.len() == 16)
    );
    let ValueList::Str(ranges) = d.list(&Key::Range, None).unwrap() else { panic!() };
    assert_eq!(ranges.first().map(String::as_str), Some("auto"));
    assert!(ranges.contains(&"1832F".to_owned()));
    assert!(!ranges.contains(&"3.0V".to_owned()), "the diode's is not listed");
    assert!(d.options(Some("x")).is_err());
    assert!(d.get(&Key::Samplerate, None).is_err());
}
