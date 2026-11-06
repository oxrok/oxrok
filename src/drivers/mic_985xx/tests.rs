// Counterpart in sigrok:
//   libsigrok/src/hardware/mic-985xx/protocol.c,
//   Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Martin Ling, Aurelien Jacobs, Gerhard Sittig,
//   Bert Vermeulen, Daniel Elstner, Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The frames, the rules over `Instant`, and the driver on a fake port.
//!
//! A fake's bytes go by counts of reads, or at a rate that only stretches, so
//! nothing here times a race: limits on real time are one-sided or wide, and each
//! capture has an overall bound, so that a stop or a limit that does not bite
//! fails rather than hangs.

use super::protocol::SILENCE;
use super::*;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, RecvTimeoutError, sync_channel};

/// Far above anything a test waits for, however slow the box.
const BOUND: Duration = Duration::from_secs(60);
/// The 38400/8n2 line: eleven bits a byte.
const BYTE_TIME: Duration = Duration::from_micros(287);

/// The most a capture may send a test: a limit or a stop that does not bite on a
/// stream must fail here, not fill the memory.
const MAX_PACKETS: usize = 20_000;

type Received = Receiver<(usize, Packet)>;

/// Runs a test on its own thread with a bound. A stop that does not bite fails
/// the test's wait, and then the device's drop joins for ever.
fn bounded(body: impl FnOnce() + Send + 'static) {
    let (done, finished) = std::sync::mpsc::channel();
    let thread = std::thread::spawn(move || {
        body();
        let _ = done.send(());
    });
    match finished.recv_timeout(2 * BOUND) {
        Ok(()) | Err(RecvTimeoutError::Disconnected) => thread.join().unwrap(),
        Err(RecvTimeoutError::Timeout) => panic!("the test did not end within its bound"),
    }
}

#[derive(Default)]
struct Line {
    /// What is on its way, with the head first.
    queued: VecDeque<u8>,
    /// Bytes a read gives at most; 0 for all.
    per_read: usize,
    /// How long a read that gives bytes takes per byte: the line's rate.
    byte_time: Duration,
    /// Reads that give nothing before the queue flows.
    hold: usize,
    /// How long a read takes before it gives what is queued: a stall of the box.
    stall: Duration,
    /// Reads give nothing, at once: signals landing in the wait.
    instant_empty: bool,
    /// What a read finds whenever nothing is queued: a logger that goes on.
    stream: Option<Vec<u8>>,
    /// Bytes appear only this often, as a cable's latency timer has them.
    burst_gap: Duration,
    next_burst: Option<Instant>,
    /// The bytes one burst lets through.
    burst_size: usize,
    /// Fails from this read on (counted from 1).
    fail_at_read: Option<usize>,
    write_fails: bool,
    /// The command finds no room: `write_timeout` gives 0.
    no_room: bool,
    write_timeouts: Vec<Duration>,
    writes: Vec<(Instant, Vec<u8>)>,
    purges: usize,
    reads: usize,
    /// Writes since the port was dropped, once it was.
    writes_after_drop: Option<usize>,
}

#[derive(Clone, Default)]
struct Shared(Arc<Mutex<Line>>);

impl Shared {
    /// The lock, even after a failed test: a fake's user may drop late.
    fn line(&self) -> std::sync::MutexGuard<'_, Line> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn with(self, f: impl FnOnce(&mut Line)) -> Self {
        f(&mut self.line());
        self
    }

    fn port(&self) -> Box<dyn Port> {
        Box::new(FakePort(self.clone()))
    }

    fn queue(&self, bytes: &[u8]) {
        self.line().queued.extend(bytes);
    }

    fn writes(&self) -> usize {
        self.line().writes.len()
    }

    fn wait_for_writes(&self, n: usize) {
        let deadline = Instant::now() + BOUND;
        while self.writes() < n {
            assert!(Instant::now() < deadline, "{n} writes did not come");
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    fn wait_for_reads(&self, n: usize) {
        let deadline = Instant::now() + BOUND;
        while self.line().reads < n {
            assert!(Instant::now() < deadline, "{n} reads did not come");
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
        if line.fail_at_read.is_some_and(|n| line.reads >= n) {
            return Err(Error::Disconnected);
        }
        if line.instant_empty {
            return Ok(0);
        }
        if line.hold > 0 {
            line.hold -= 1;
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
        let mut most = if line.per_read == 0 { buf.len() } else { line.per_read.min(buf.len()) };
        if !line.burst_gap.is_zero() {
            let now = Instant::now();
            let next = *line.next_burst.get_or_insert(now);
            if now < next {
                let wait = (next - now).min(timeout);
                drop(line);
                std::thread::sleep(wait);
                return Ok(0);
            }
            line.next_burst = Some(now + line.burst_gap);
            most = most.min(line.burst_size);
        }
        let stall = line.stall;
        let n = most.min(line.queued.len());
        for b in &mut buf[..n] {
            *b = line.queued.pop_front().expect("counted");
        }
        let wait = line.byte_time * u32::try_from(n).expect("small") + stall;
        drop(line);
        std::thread::sleep(wait);
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut line = self.0.line();
        if let Some(n) = &mut line.writes_after_drop {
            *n += 1;
        }
        if line.write_fails {
            return Err(Error::Disconnected);
        }
        line.writes.push((Instant::now(), data.to_vec()));
        Ok(())
    }
    fn write_timeout(&mut self, data: &[u8], timeout: Duration) -> Result<usize> {
        let mut line = self.0.line();
        line.write_timeouts.push(timeout);
        if line.no_room {
            line.writes.push((Instant::now(), Vec::new()));
            return Ok(0);
        }
        drop(line);
        self.write(data)?;
        Ok(data.len())
    }
    fn purge(&mut self) -> Result<()> {
        let mut line = self.0.line();
        line.queued.clear();
        line.purges += 1;
        Ok(())
    }
}

impl Drop for FakePort {
    fn drop(&mut self) {
        self.0.line().writes_after_drop = Some(0);
    }
}

/// A logger on `line`, open. What is queued comes after the open's flush, which
/// counts for nothing.
fn device(model: &'static Model, line: &Shared) -> MicDevice {
    let mut d = MicDevice::from_port(model, line.port());
    let queued = std::mem::take(&mut line.line().queued);
    d.open().unwrap();
    let mut l = line.line();
    l.queued = queued;
    l.purges = 0;
    drop(l);
    d
}

fn m98581() -> &'static Model {
    &MODELS[0]
}

fn m98583() -> &'static Model {
    &MODELS[1]
}

fn start(d: &mut MicDevice) -> Received {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// All of a capture, with one overall deadline: a per-packet timeout never fires
/// on a capture that keeps sending.
fn to_end(rx: &Received) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut got = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((_, Packet::End)) => return got,
            Ok((_, p)) => {
                assert!(got.len() < MAX_PACKETS, "no End after {MAX_PACKETS} packets");
                got.push(p);
            }
            Err(e) => panic!("no End within the bound: {e:?}, after {} packets", got.len()),
        }
    }
}

/// `(channel, value)` of each analog packet.
fn readings(got: &[Packet]) -> Vec<(usize, f32)> {
    got.iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some((a.channels[0], a.samples[0])),
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

fn limit_samples(d: &mut MicDevice, n: u64) {
    d.set(&Key::LimitSamples, &Value::U64(n), None).unwrap();
}

fn limit_msec(d: &mut MicDevice, n: u64) {
    d.set(&Key::LimitMsec, &Value::U64(n), None).unwrap();
}

/// A whole 98583 capture to `n` samples on `line`.
fn capture_98583(line: &Shared, n: u64) -> Vec<Packet> {
    let mut d = device(m98583(), line);
    limit_samples(&mut d, n);
    let rx = start(&mut d);
    let got = to_end(&rx);
    d.close().unwrap();
    got
}

// The frames.

#[test]
fn a_frame_of_each_is_valid() {
    assert!(m98581().valid(b"v 230\r"));
    assert!(m98583().valid(b"v 230 610\r"));
    assert_eq!(m98581().packet_size(), 6);
    assert_eq!(m98583().packet_size(), 10);
}

#[test]
fn each_byte_of_a_frame_is_checked() {
    for at in 0..6 {
        let mut f = *b"v 230\r";
        f[at] = b'x';
        assert!(!m98581().valid(&f), "byte {at}");
    }
    for at in 0..10 {
        let mut f = *b"v 230 610\r";
        f[at] = b'x';
        assert!(!m98583().valid(&f), "byte {at}");
    }
    assert!(!m98581().valid(b"v 23\xb20\r"));
    assert!(!m98583().valid(b"v 230 61\xb2\r"));
}

#[test]
fn a_field_is_in_tenths() {
    assert_eq!(m98581().parse(b"v 235\r"), (23.5, None));
    assert_eq!(m98583().parse(b"v 235 612\r"), (23.5, Some(61.2)));
    assert_eq!(m98583().parse(b"v 000 999\r"), (0.0, Some(99.9)));
    assert_eq!(m98583().parse(b"v 009 010\r"), (0.9, Some(1.0)));
}

#[test]
fn the_table_says_what_cs_says() {
    assert_eq!(m98581().id, "mic-98581");
    assert_eq!(m98583().id, "mic-98583");
    assert_eq!(m98581().long_name, "MIC 98581");
    assert_eq!(m98583().long_name, "MIC 98583");
    for m in &MODELS {
        assert_eq!(m.serialcomm, "38400/8n2");
    }
    assert_eq!(REALTIME, "S 1 M 2 32 3\r");
    assert_eq!(BUF_SIZE, 256);
    assert_eq!(SILENCE, Duration::from_millis(100));
    assert_eq!(NO_READING_BOUND, Duration::from_secs(30));
}

// The rules, with no time passing.

#[test]
fn the_command_waits_for_100_ms_of_silence() {
    let t0 = Instant::now();
    let mut r = Realtime::new(t0);
    assert!(!r.empty(t0 + Duration::from_millis(99)));
    assert!(r.empty(t0 + Duration::from_millis(100)));
}

#[test]
fn bytes_put_the_command_off() {
    let t0 = Instant::now();
    let mut r = Realtime::new(t0);
    r.heard(t0 + Duration::from_millis(90));
    assert!(!r.empty(t0 + Duration::from_millis(100)));
    assert!(!r.empty(t0 + Duration::from_millis(189)));
    assert!(r.empty(t0 + Duration::from_millis(190)));
}

#[test]
fn a_wait_runs_to_the_timer_and_is_never_0() {
    let t0 = Instant::now();
    let r = Realtime::new(t0);
    assert_eq!(r.wait(t0), Duration::from_millis(100));
    assert_eq!(r.wait(t0 + Duration::from_millis(60)), Duration::from_millis(40));
    assert_eq!(r.wait(t0 + Duration::from_secs(5)), Duration::from_millis(1));
}

/// A logger whose answer takes a steady 2.5 s, in a simulated minute: one command
/// goes out, so nothing is owed and no reading is older than its own delay.
#[test]
fn a_slow_logger_for_a_minute_gets_one_command() {
    let t0 = Instant::now();
    let mut r = Realtime::new(t0);
    let mut sent = Vec::new();
    let mut readings_at = Vec::new();
    let mut next_reading = None;
    for ms in 0..60_000u64 {
        let now = t0 + Duration::from_millis(ms);
        if next_reading.is_some_and(|at| now >= at) {
            r.heard(now);
            readings_at.push(now);
            next_reading = Some(now + Duration::from_millis(2500));
        } else if r.empty(now) {
            sent.push(now);
            next_reading = Some(now + Duration::from_millis(2500));
        }
    }
    assert_eq!(sent, vec![t0 + SILENCE]);
    assert_eq!(readings_at.len(), 23);
}

#[test]
fn the_bound_counts_from_both_sides() {
    let t0 = Instant::now();
    let w = Watch::new(Duration::from_secs(30), t0);
    assert_eq!(w.starved(t0 + Duration::from_millis(29_999)), None);
    assert_eq!(w.starved(t0 + Duration::from_secs(30)), Some(Starved::NoBytes));
}

#[test]
fn only_a_reading_resets_the_bound() {
    let t0 = Instant::now();
    let mut w = Watch::new(Duration::from_secs(30), t0);
    w.heard();
    assert_eq!(w.starved(t0 + Duration::from_secs(30)), Some(Starved::NoReading));
    w.reading(t0 + Duration::from_secs(20));
    assert_eq!(w.starved(t0 + Duration::from_secs(49)), None);
    assert_eq!(w.starved(t0 + Duration::from_secs(50)), Some(Starved::NoBytes), "bytes forgotten");
}

#[test]
fn what_a_failed_read_and_command_say() {
    bounded(|| {
        let e = Error::device("gone");
        assert_eq!(read_error_line(&e), "Serial port read error: device error: gone.");
        assert_eq!(
            command_error_line(&e),
            "Error sending 'S 1 M 2 32 3\r' command: device error: gone."
        );
    });
}

#[test]
fn what_a_starved_capture_says() {
    assert_eq!(
        starved_words(Starved::NoBytes, m98583(), 30),
        (
            "No bytes from the logger for 30 s: it is off, its cable is out, or it did not take \
             the command."
                .to_owned(),
            "no bytes from the logger for 30 s".to_owned()
        )
    );
    assert_eq!(
        starved_words(Starved::NoReading, m98581(), 30),
        (
            "No usable data from the logger for 30 s: bytes came, but none made a reading of a \
             MIC 98581."
                .to_owned(),
            "no usable data from the logger for 30 s".to_owned()
        )
    );
}

// The driver's face.

#[test]
fn the_drivers_list_as_cs_do() {
    let d = MicDriver(m98583());
    assert_eq!(d.id(), "mic-98583");
    assert_eq!(d.long_name(), "MIC 98583");
    assert_eq!(d.scan_options(), [Key::Conn, Key::Serialcomm]);
    assert_eq!(d.classes(), [DeviceClass::Thermometer, DeviceClass::Hygrometer]);
    assert_eq!(MicDriver(m98581()).classes(), [DeviceClass::Thermometer]);
}

#[test]
fn a_scan_with_no_port_finds_nothing() {
    let d = MicDriver(m98583());
    assert_eq!(d.scan(&[]).unwrap().len(), 0, "no conn");
    let gone = [(Key::Conn, Value::Str("/dev/ttyNOSUCHPORT".into()))];
    assert_eq!(d.scan(&gone).unwrap().len(), 0);
    let bad = [
        (Key::Conn, Value::Str("/dev/ttyNOSUCHPORT".into())),
        (Key::Serialcomm, Value::Str("fast".into())),
    ];
    assert_eq!(d.scan(&bad).unwrap().len(), 0);
}

#[test]
fn channels_and_keys() {
    bounded(|| {
        let line = Shared::default();
        let d = device(m98583(), &line);
        let names: Vec<_> = d.channels().iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["Temperature", "Humidity"]);
        let d = device(m98581(), &line);
        let names: Vec<_> = d.channels().iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["Temperature"]);
        let options = d.options(None).unwrap();
        let keys: Vec<_> = options.iter().map(|o| o.key.clone()).collect();
        assert_eq!(keys, [Key::Continuous, Key::LimitSamples, Key::LimitMsec]);
        assert!(!options[1].access.get && options[1].access.set);
        assert!(!options[2].access.get && options[2].access.set);
    });
}

// Captures.

#[test]
fn a_capture_says_the_header_the_readings_and_their_units() {
    bounded(|| {
        let line = Shared::default();
        line.queue(b"v 235 612\r");
        let got = capture_98583(&line, 1);
        assert!(matches!(got[0], Packet::Header { .. }), "{got:?}");
        let Packet::Analog(t) = &got[1] else { panic!("{got:?}") };
        assert_eq!(
            (t.channels.as_slice(), t.samples.as_slice(), t.mq, t.unit, t.digits),
            ([0].as_slice(), [23.5].as_slice(), Mq::Temperature, Unit::Celsius, 1)
        );
        assert_eq!(t.flags, MqFlags::empty());
        let Packet::Analog(h) = &got[2] else { panic!("{got:?}") };
        assert_eq!(
            (h.channels.as_slice(), h.samples.as_slice(), h.mq, h.unit, h.digits),
            ([1].as_slice(), [61.2].as_slice(), Mq::RelativeHumidity, Unit::Percentage, 1)
        );
        assert_eq!(got.len(), 3);
    });
}

#[test]
fn a_98581_says_temperature_alone() {
    bounded(|| {
        let line = Shared::default();
        line.queue(b"v 235\r");
        let mut d = device(m98581(), &line);
        limit_samples(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&to_end(&rx)), [(0, 23.5)]);
    });
}

#[test]
fn a_reading_that_comes_first_means_no_command() {
    bounded(|| {
        let line = Shared::default();
        line.queue(b"v 230 610\r");
        capture_98583(&line, 1);
        assert_eq!(line.writes(), 0);
    });
}

/// `fd_source_check` looks at `revents` first: a stall past the timer, with the
/// answer ready, is no silence.
#[test]
fn a_ready_answer_wins_over_the_timer_after_a_stall() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stall = Duration::from_millis(150);
            l.stream = Some(b"v 230 610\r".to_vec());
        });
        capture_98583(&line, 3);
        assert_eq!(line.writes(), 0, "each read found bytes");
    });
}

#[test]
fn a_silence_gets_one_command_with_its_timeout() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(m98583(), &line);
        let rx = start(&mut d);
        line.wait_for_writes(1);
        let reads = line.line().reads;
        line.wait_for_reads(reads + 3);
        d.stop().unwrap();
        assert!(failure(&to_end(&rx)).is_none());
        d.close().unwrap();
        let l = line.line();
        assert_eq!(l.writes.len(), 1, "once a capture");
        assert_eq!(l.writes[0].1, b"S 1 M 2 32 3\r");
        // 10 ms and 13 bytes of 11 bits at 38400.
        assert_eq!(l.write_timeouts, [Duration::from_millis(13)]);
    });
}

#[test]
fn an_early_empty_read_is_no_silence_and_does_not_spin() {
    bounded(|| {
        let line = Shared::default().with(|l| l.instant_empty = true);
        let begun = Instant::now();
        let mut d = device(m98583(), &line);
        let rx = start(&mut d);
        line.wait_for_writes(1);
        let at = line.line().writes[0].0;
        assert!(at - begun >= SILENCE, "after {:?}", at - begun);
        let reads = line.line().reads;
        let elapsed = begun.elapsed();
        d.stop().unwrap();
        to_end(&rx);
        assert!(reads as u128 <= elapsed.as_millis() + 2, "{reads} reads in {elapsed:?}");
    });
}

#[test]
fn the_limit_is_looked_at_after_all_of_a_read() {
    bounded(|| {
        let line = Shared::default();
        line.queue(b"v 230 610\rv 240 620\rv 250 630\rv 260 640\r");
        line.line().per_read = 30;
        let got = capture_98583(&line, 1);
        assert_eq!(
            readings(&got),
            [(0, 23.0), (1, 61.0), (0, 24.0), (1, 62.0), (0, 25.0), (1, 63.0)],
            "the three of the read"
        );
    });
}

#[test]
fn a_frame_at_the_lines_rate_comes_whole() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.per_read = 1;
            l.byte_time = BYTE_TIME;
        });
        line.queue(b"v 230 610\rv 240 620\r");
        let got = capture_98583(&line, 2);
        assert_eq!(readings(&got), [(0, 23.0), (1, 61.0), (0, 24.0), (1, 62.0)]);
        assert_eq!(line.writes(), 0, "bytes were coming");
    });
}

#[test]
fn bursts_every_50_ms_lose_and_shift_nothing() {
    bounded(|| {
        let mut stream = Vec::new();
        for t in 200..210 {
            stream.extend_from_slice(format!("v {t} {}\r", t + 100).as_bytes());
        }
        let line = Shared::default().with(|l| {
            l.burst_gap = Duration::from_millis(50);
            l.burst_size = 7;
        });
        line.queue(&stream);
        let got = capture_98583(&line, 10);
        let values = readings(&got);
        assert_eq!(values.len(), 20, "{values:?}");
        for (i, pair) in values.chunks(2).enumerate() {
            let (t, h) = ((200 + i) as f32 / 10.0, (300 + i) as f32 / 10.0);
            assert_eq!(pair, [(0, t), (1, h)], "reading {i}");
        }
    });
}

/// With the first read held past the timer, the command goes out, and the bytes
/// that then come are read as ever.
#[test]
fn a_late_reading_after_the_command() {
    bounded(|| {
        let line = Shared::default().with(|l| l.hold = 2);
        line.queue(b"v 230 610\r");
        let got = capture_98583(&line, 1);
        assert_eq!(readings(&got), [(0, 23.0), (1, 61.0)]);
        assert_eq!(line.writes(), 1);
    });
}

#[test]
fn noise_and_false_starts_are_stepped_over() {
    bounded(|| {
        let line = Shared::default();
        line.queue(b"??v 230 v 240 620\rv 2x0 610\r?v 250 630\r");
        let got = capture_98583(&line, 2);
        assert_eq!(readings(&got), [(0, 24.0), (1, 62.0), (0, 25.0), (1, 63.0)]);
    });
}

#[test]
fn a_long_read_of_noise_keeps_the_buffer_within_its_size() {
    bounded(|| {
        let line = Shared::default();
        let mut noise = vec![b'?'; 1000];
        noise.extend_from_slice(b"v 230 610\r");
        line.queue(&noise);
        line.line().per_read = 0;
        let got = capture_98583(&line, 1);
        assert_eq!(readings(&got), [(0, 23.0), (1, 61.0)]);
        assert!(failure(&got).is_none());
    });
}

// How a capture ends.

fn short_bound(d: &mut MicDevice) -> Duration {
    d.bound = Duration::from_millis(300);
    d.bound
}

#[test]
fn a_silent_logger_ends_at_the_bound_at_a_sample_limit() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(m98583(), &line);
        let bound = short_bound(&mut d);
        limit_samples(&mut d, 5);
        let begun = Instant::now();
        let rx = start(&mut d);
        let got = to_end(&rx);
        assert!(begun.elapsed() >= bound, "after {:?}", begun.elapsed());
        assert!(failure(&got).unwrap().contains("no bytes from the logger"), "{got:?}");
        assert_eq!(line.writes(), 1, "it was told");
    });
}

#[test]
fn bytes_with_no_frame_end_at_the_bound_at_a_sample_limit() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(b"v 230\r".to_vec());
            l.byte_time = BYTE_TIME;
        });
        let mut d = device(m98583(), &line);
        let bound = short_bound(&mut d);
        limit_samples(&mut d, 1);
        let begun = Instant::now();
        let rx = start(&mut d);
        let got = to_end(&rx);
        assert!(begun.elapsed() >= bound);
        assert!(failure(&got).unwrap().contains("no usable data from the logger"), "{got:?}");
        assert_eq!(readings(&got), Vec::<(usize, f32)>::new());
    });
}

#[test]
fn a_bound_does_not_end_a_capture_a_time_limit_can() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(m98583(), &line);
        let bound = short_bound(&mut d);
        limit_samples(&mut d, 5);
        limit_msec(&mut d, 900);
        let begun = Instant::now();
        let rx = start(&mut d);
        let got = to_end(&rx);
        assert!(begun.elapsed() >= 3 * bound, "after {:?}", begun.elapsed());
        assert!(failure(&got).is_none(), "{got:?}");
    });
}

#[test]
fn a_bound_does_not_end_a_continuous_capture() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(m98583(), &line);
        let bound = short_bound(&mut d);
        let rx = start(&mut d);
        assert!(matches!(rx.recv_timeout(BOUND), Ok((_, Packet::Header { .. }))));
        std::thread::sleep(2 * bound);
        assert!(rx.try_recv().is_err(), "nothing but the header");
        d.stop().unwrap();
        assert!(failure(&to_end(&rx)).is_none());
    });
}

#[test]
fn a_time_limit_ends_a_capture_without_failure() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(m98583(), &line);
        limit_msec(&mut d, 50);
        let begun = Instant::now();
        let rx = start(&mut d);
        let got = to_end(&rx);
        assert!(begun.elapsed() >= Duration::from_millis(50));
        assert!(failure(&got).is_none(), "{got:?}");
        assert_eq!(line.writes(), 1, "the first silence ran before the limit was looked at");
    });
}

#[test]
fn a_time_limit_stops_a_stream() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(b"v 230 610\r".to_vec());
            l.byte_time = BYTE_TIME;
        });
        let mut d = device(m98583(), &line);
        limit_msec(&mut d, 50);
        let rx = start(&mut d);
        assert!(failure(&to_end(&rx)).is_none());
    });
}

#[test]
fn a_read_that_fails_fails_the_capture_with_its_port_back() {
    bounded(|| {
        let line = Shared::default().with(|l| l.fail_at_read = Some(2));
        line.queue(b"v 230 610\r");
        let mut d = device(m98583(), &line);
        limit_samples(&mut d, 5);
        let rx = start(&mut d);
        let got = to_end(&rx);
        assert_eq!(readings(&got), [(0, 23.0), (1, 61.0)]);
        assert_eq!(failure(&got), Some(Error::Disconnected.to_string().as_str()), "{got:?}");
        assert!(matches!(got.last(), Some(Packet::Failed(_))), "last before End: {got:?}");
        assert!(d.port.lock().unwrap().is_some());
    });
}

#[test]
fn a_failed_command_ends_nothing_and_is_not_sent_again() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.write_fails = true;
            l.hold = 3;
        });
        line.queue(b"v 230 610\r");
        let got = capture_98583(&line, 1);
        assert_eq!(readings(&got), [(0, 23.0), (1, 61.0)]);
        assert!(failure(&got).is_none(), "{got:?}");
        assert_eq!(line.line().write_timeouts.len(), 1, "one try");
    });
}

#[test]
fn a_command_with_no_room_ends_nothing() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.no_room = true;
            l.hold = 3;
        });
        line.queue(b"v 230 610\r");
        let got = capture_98583(&line, 1);
        assert_eq!(readings(&got), [(0, 23.0), (1, 61.0)]);
        assert!(failure(&got).is_none(), "{got:?}");
        assert_eq!(line.line().write_timeouts.len(), 1, "one try");
    });
}

#[test]
fn a_consumer_gone_ends_the_capture_quietly() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(b"v 230 610\r".to_vec()));
        let mut d = device(m98583(), &line);
        let rx = start(&mut d);
        drop(rx);
        let deadline = Instant::now() + BOUND;
        while d.port.lock().unwrap().is_none() {
            assert!(Instant::now() < deadline, "the port did not come back");
            std::thread::sleep(Duration::from_millis(2));
        }
    });
}

#[test]
fn a_stop_with_a_stream_ends_it_and_says_nothing_failed() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(b"v 230 610\r".to_vec()));
        let mut d = device(m98583(), &line);
        let rx = start(&mut d);
        line.wait_for_reads(5);
        d.stop().unwrap();
        let got = to_end(&rx);
        assert!(failure(&got).is_none());
        d.close().unwrap();
        assert!(d.worker.is_none());
    });
}

#[test]
fn a_slow_client_is_no_silence() {
    bounded(|| {
        // A queue of one: the second reading's temperature waits for this test.
        let line = Shared::default();
        line.queue(b"v 230 610\r");
        let mut d = device(m98583(), &line);
        let bound = short_bound(&mut d);
        limit_samples(&mut d, 2);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        // Blocked on a full queue past the bound, then the logger's next is ready.
        std::thread::sleep(2 * bound);
        line.queue(b"v 240 620\r");
        let mut got = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(left).expect("the capture ends") {
                (_, Packet::End) => break,
                (_, p) => got.push(p),
            }
        }
        assert!(failure(&got).is_none(), "{got:?}");
        assert_eq!(readings(&got).len(), 4);
    });
}

/// The silence counts from the end of the sends, whatever the client took.
#[test]
fn a_slow_client_does_not_bring_the_command_forward() {
    bounded(|| {
        let line = Shared::default();
        line.queue(b"v 230 610\r");
        let mut d = device(m98583(), &line);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Past the timer, blocked with the reading's first packet.
        std::thread::sleep(2 * SILENCE);
        let unblocked = Instant::now();
        for _ in 0..3 {
            rx.recv_timeout(BOUND).expect("the header and the reading");
        }
        line.wait_for_writes(1);
        let at = line.line().writes[0].0;
        assert!(at - unblocked >= SILENCE, "after {:?}", at - unblocked);
        d.stop().unwrap();
        drop(rx);
    });
}

#[test]
fn a_close_with_a_full_queue_does_not_hang() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(b"v 230 610\r".to_vec()));
        let mut d = device(m98583(), &line);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        line.wait_for_reads(1);
        let (done_tx, done_rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            d.close().unwrap();
            let _ = done_tx.send(d);
        });
        let d = done_rx.recv_timeout(BOUND).expect("the close came back");
        drop(rx);
        drop(d);
    });
}

// A second capture on the open port.

#[test]
fn a_second_capture_sends_its_own_command() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(m98583(), &line);
        limit_msec(&mut d, 150);
        let rx = start(&mut d);
        to_end(&rx);
        assert_eq!(line.writes(), 1);
        let rx = start(&mut d);
        to_end(&rx);
        assert_eq!(line.writes(), 2, "C's static would keep this one back");
    });
}

#[test]
fn a_second_capture_drops_what_queued_and_the_head_of_a_frame() {
    bounded(|| {
        let line = Shared::default().with(|l| l.per_read = 8);
        // The first capture ends with the head of a frame in its buffer.
        line.queue(b"v 230 610\rv 240 ");
        let mut d = device(m98583(), &line);
        limit_samples(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(readings(&to_end(&rx)), [(0, 23.0), (1, 61.0)]);
        assert_eq!(line.line().purges, 0, "a first capture has the open's flush");
        // What is left of it, and an old reading.
        line.queue(b"v 250 650\r");
        let rx = {
            let (tx, rx) = sync_channel(1024);
            d.start(PacketSink::new(tx, 0)).unwrap();
            rx
        };
        // Its tail would have made the head of the last capture a frame.
        let reads = line.line().reads;
        line.wait_for_reads(reads + 1);
        line.queue(b"620\rv 260 660\r");
        let got = to_end(&rx);
        assert_eq!(line.line().purges, 1);
        assert_eq!(readings(&got), [(0, 26.0), (1, 66.0)], "no old reading, no false frame");
    });
}

#[test]
fn a_reopen_is_a_first_capture_again() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(m98583(), &line);
        limit_msec(&mut d, 1);
        let rx = start(&mut d);
        to_end(&rx);
        d.close().unwrap();
        d.pending = Some(line.port());
        d.open().unwrap();
        line.line().purges = 0;
        let rx = start(&mut d);
        to_end(&rx);
        assert_eq!(line.line().purges, 0);
    });
}

#[test]
fn a_start_while_the_end_is_stuck_does_not_hang_and_frees_the_old_sink() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(m98583(), &line);
        limit_msec(&mut d, 1);
        let (tx, old) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The header fills the queue, and the End cannot go: the port comes back first.
        let deadline = Instant::now() + BOUND;
        while d.port.lock().unwrap().is_none() {
            assert!(Instant::now() < deadline, "the port did not come back");
            std::thread::sleep(Duration::from_millis(2));
        }
        let rx = start(&mut d);
        assert!(matches!(old.recv_timeout(BOUND), Ok((_, Packet::Header { .. }))));
        assert_eq!(
            old.recv_timeout(BOUND).map(|(_, p)| matches!(p, Packet::End)),
            Err(RecvTimeoutError::Disconnected),
            "the old sink went with its thread"
        );
        to_end(&rx);
    });
}

#[test]
fn nothing_is_written_after_a_close() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(m98583(), &line);
        let rx = start(&mut d);
        line.wait_for_reads(1);
        d.close().unwrap();
        assert!(d.port.lock().unwrap().is_none());
        assert_eq!(line.line().writes_after_drop, Some(0));
        drop(rx);
        let writes = line.writes();
        std::thread::sleep(2 * SILENCE);
        assert_eq!(line.writes(), writes);
        assert_eq!(line.line().writes_after_drop, Some(0));
    });
}

#[test]
fn a_start_with_no_port_is_refused() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(m98583(), &line);
        d.close().unwrap();
        let (tx, _rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
    });
}
