// Counterpart in sigrok:
//   libsigrok/src/hardware/serial-lcr/api.c,
//   libsigrok/src/hardware/serial-lcr/protocol.c,
//   Copyright (C) 2014 Janne Huttunen <jahuttun@gmail.com>,
//   Copyright (C) 2019 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Uwe Hermann, Valentin Ochs, Vitaliy Vorobyov,
//   Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake port. A fake's bytes go by counts of reads, stretched
//! by a load but never lost to it, so nothing here times a race: limits on real
//! time are one-sided or wide, and each capture has an overall bound.

use super::*;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// Far above anything a test waits for, however slow the box: it only makes a
/// stop or a limit that does not bite fail rather than hang.
const BOUND: Duration = Duration::from_secs(60);

type Received = Receiver<(usize, Packet)>;

#[derive(Default)]
struct Line {
    /// What is on its way, with the head first.
    queued: VecDeque<u8>,
    /// Bytes a read gives at most; 0 for all.
    per_read: usize,
    /// How long a read that gives bytes takes per byte: the line's rate.
    byte_time: Duration,
    /// Reads that give nothing before the queue flows: an answer that is late.
    hold: usize,
    /// What each request is answered with, in turn; none for silence.
    answers: VecDeque<Vec<u8>>,
    /// What a read finds whenever nothing is queued: a meter that streams.
    stream: Option<Vec<u8>>,
    read_fails: bool,
    /// Reads that fail before they stop failing.
    failed_reads: usize,
    write_fails: bool,
    /// Requests take no room: `write_timeout` gives 0.
    no_room: bool,
    /// The timeout each `write_timeout` got.
    write_timeouts: Vec<Duration>,
    /// When requests were written, and whether they met bytes on the way.
    writes: Vec<Instant>,
    writes_during_a_reply: usize,
    purges: usize,
    reads: usize,
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

    /// Until `n` requests were written, within the bound.
    fn wait_for_writes(&self, n: usize) {
        let deadline = Instant::now() + BOUND;
        while self.line().writes.len() < n {
            assert!(Instant::now() < deadline, "{n} requests were not written");
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    /// The least time between two requests.
    fn least_gap(&self) -> Duration {
        let line = self.line();
        line.writes.windows(2).map(|w| w[1] - w[0]).min().expect("two requests")
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
        let most = if line.per_read == 0 { buf.len() } else { line.per_read.min(buf.len()) };
        let n = most.min(line.queued.len());
        for b in &mut buf[..n] {
            *b = line.queued.pop_front().expect("counted");
        }
        let wait = line.byte_time * u32::try_from(n).expect("small");
        drop(line);
        std::thread::sleep(wait);
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut line = self.0.line();
        if line.write_fails {
            return Err(Error::Disconnected);
        }
        assert_eq!(data, b"N", "a request is one N");
        line.writes.push(Instant::now());
        if !line.queued.is_empty() {
            line.writes_during_a_reply += 1;
        }
        if let Some(answer) = line.answers.pop_front() {
            line.queued.extend(answer);
        }
        Ok(())
    }
    fn write_timeout(&mut self, data: &[u8], timeout: Duration) -> Result<usize> {
        self.0.line().write_timeouts.push(timeout);
        if self.0.line().no_room {
            self.0.line().writes.push(Instant::now());
            return Ok(0);
        }
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

fn model(id: &str) -> &'static Model {
    MODELS.iter().find(|m| m.id == id).expect("in the table")
}

fn es_model() -> &'static Model {
    model("deree-de5000")
}

fn vc_model() -> &'static Model {
    model("voltcraft-4080")
}

/// An ES51919 packet reading `value` on the primary, at 1 kHz, series.
fn es_packet(value: u16) -> Vec<u8> {
    let [hi, lo] = value.to_be_bytes();
    vec![0, 0x0d, 0x20, 0x50, 0, 2, hi, lo, 0x53, 0, 1, 0, 0x7b, 3, 0, 0x0d, 0x0a]
}

/// A vc4080 packet reading `value` (five digits) on the main display, with
/// parity bits.
fn vc_packet(value: &str) -> Vec<u8> {
    format!("LQAPA{value}30134210711414062__________\r\n")
        .bytes()
        .map(|b| if b.count_ones() % 2 == 1 { b | 0x80 } else { b })
        .collect()
}

/// A blank primary and secondary.
fn es_blank() -> Vec<u8> {
    let mut p = es_packet(1);
    p[9] = 1;
    p[14] = 1;
    p
}

/// An open device on the line, what the line has queued being for after the
/// open's flush.
fn device(model: &'static Model, line: &Shared) -> LcrDevice {
    let mut d = LcrDevice::from_port(model, line.port());
    let queued = std::mem::take(&mut line.line().queued);
    d.open().unwrap();
    line.line().queued = queued;
    d
}

fn start(d: &mut LcrDevice) -> Received {
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
        out.push(p);
    }
}

/// The values on `channel` in those packets.
fn values(got: &[Packet], channel: usize) -> Vec<f32> {
    got.iter()
        .filter_map(|p| match p {
            Packet::Analog(a) if a.channels == [channel] => Some(a.samples[0]),
            _ => None,
        })
        .collect()
}

fn frames(got: &[Packet]) -> usize {
    got.iter().filter(|p| **p == Packet::FrameEnd).count()
}

fn metas(got: &[Packet]) -> Vec<(Key, Value)> {
    got.iter()
        .filter_map(|p| match p {
            Packet::Meta(m) => Some(m[0].clone()),
            _ => None,
        })
        .collect()
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

#[test]
fn the_table_of_drivers() {
    let mut registry = crate::core::Registry::default();
    register_all(&mut registry);
    for (id, name, chip) in [
        ("deree-de5000", "DER EE DE-5000", Chip::Es51919),
        ("mastech-ms5308", "MASTECH MS5308", Chip::Es51919),
        ("peaktech-2170", "PeakTech 2170", Chip::Es51919),
        ("uni-t-ut612", "UNI-T UT612", Chip::Es51919),
        ("peaktech-2165", "PeakTech 2165", Chip::Vc4080),
        ("voltcraft-4080", "Voltcraft 4080", Chip::Vc4080),
    ] {
        let m = model(id);
        assert_eq!(m.long_name, name);
        assert_eq!(format!("{} {}", m.vendor, m.model), name);
        assert_eq!(m.chip, chip);
        assert!(registry.find(id).is_some(), "{id}");
    }
}

#[test]
fn a_scan_without_a_port_finds_nothing_and_does_not_fail() {
    let d = SerialLcrDriver(es_model());
    assert!(d.scan(&[]).unwrap().is_empty());
    let conn = (Key::Conn, Value::Str("/dev/ttyNOSUCH".to_owned()));
    assert!(d.scan(std::slice::from_ref(&conn)).unwrap().is_empty());
    let bad = (Key::Serialcomm, Value::Str("fast".to_owned()));
    assert!(d.scan(&[conn, bad]).unwrap().is_empty());
}

fn params(chip: Chip) -> Params {
    Params::parse(chip.serialcomm()).unwrap()
}

fn detect_on(line: &Shared, chip: Chip, timeout: Duration) -> (Option<Vec<u8>>, usize) {
    let mut port = FakePort(line.clone());
    detect(&mut port, chip, &params(chip), timeout)
}

#[test]
fn a_search_gives_up_when_its_buffer_is_full() {
    // As serial_stream_detect: 128 bytes and no packet in them is no device,
    // even though one follows.
    let mut input = vec![b'?'; 112];
    input.extend(es_packet(1));
    let line = Shared::default().with(|l| l.queued.extend(input));
    let (found, read) = detect_on(&line, Chip::Es51919, Duration::from_secs(30));
    assert_eq!(found, None);
    assert_eq!(read, 128);
    assert_eq!(line.line().queued.len(), 1, "the byte it did not read");
    // A packet that ends within the 128 is found, and the rest stays.
    let mut input = vec![b'?'; 111];
    input.extend(es_packet(2));
    input.push(0x55);
    let line = Shared::default().with(|l| l.queued.extend(input));
    let (found, read) = detect_on(&line, Chip::Es51919, Duration::from_secs(30));
    assert_eq!(found, Some(es_packet(2)));
    assert_eq!(read, 128);
    assert_eq!(line.line().queued.len(), 1);
}

#[test]
fn a_search_gives_up_at_its_time_and_a_failed_read_is_no_data() {
    for read_fails in [false, true] {
        let line = Shared::default().with(|l| l.read_fails = read_fails);
        let at = Instant::now();
        let (found, read) = detect_on(&line, Chip::Vc4080, Duration::from_millis(150));
        assert_eq!((found, read), (None, 0));
        assert!(at.elapsed() >= Duration::from_millis(150), "{read_fails}");
        // Unhurried: 1200 Bd gives a read of over 10 ms, and a load only
        // makes fewer.
        let reads = line.line().reads;
        assert!((1..=16).contains(&reads), "{read_fails}: {reads}");
    }
}

#[test]
fn a_search_goes_on_after_a_failed_read() {
    let line = Shared::default().with(|l| {
        l.failed_reads = 3;
        l.queued.extend(vc_packet("12345"));
    });
    assert_eq!(detect_on(&line, Chip::Vc4080, BOUND).0, Some(vc_packet("12345")));
    assert_eq!(line.line().failed_reads, 0);
}

#[test]
fn a_search_takes_a_packet_cut_in_pieces() {
    let line = Shared::default().with(|l| {
        l.per_read = 1;
        l.queued.extend(vc_packet("12345"));
    });
    assert_eq!(detect_on(&line, Chip::Vc4080, BOUND).0, Some(vc_packet("12345")));
}

fn probe_on(line: &Shared, model: &Model) -> Probed {
    let mut port = FakePort(line.clone());
    probe(&mut port, model, &params(model.chip), "/dev/ttyFAKE")
}

#[test]
fn a_probe_asks_twice_and_flushes_between() {
    let line = Shared::default().with(|l| {
        l.answers.push_back(vc_packet("11111"));
        l.answers.push_back(
            b"LQBSA11111".iter().copied().chain(vc_packet("11111")[10..].iter().copied()).collect(),
        );
    });
    assert_eq!(
        probe_on(&line, vc_model()),
        Probed::Found(Some(Settings { output_freq: 120, circuit_model: "SERIES" })),
        "the second packet's"
    );
    let l = line.line();
    assert_eq!(l.writes.len(), 2);
    assert_eq!(l.purges, 1);
}

#[test]
fn a_flush_drops_what_followed_the_first_packet() {
    let line = Shared::default().with(|l| {
        l.queued.extend(es_packet(1));
        // Before the flush, and it says something else.
        let mut other = es_packet(1);
        other[3] = 0x10;
        l.queued.extend(other);
        // After the flush the meter is silent.
    });
    // The fake's queue holds both: the flush drops the second.
    assert_eq!(probe_on(&line, es_model()), Probed::Found(None));
    assert_eq!(line.line().purges, 1);
}

#[test]
fn a_probe_with_no_packet_finds_nothing() {
    // The first listen's 3000 ms, and no second.
    let line = Shared::default();
    let at = Instant::now();
    assert_eq!(probe_on(&line, es_model()), Probed::Nothing);
    assert!(at.elapsed() >= Duration::from_millis(3000));
    assert_eq!(line.line().purges, 0);
}

#[test]
fn the_probes_times_are_literal() {
    assert_eq!(DETECT_TIMEOUT, Duration::from_millis(3000));
    assert_eq!(SETTINGS_TIMEOUT, Duration::from_millis(1500));
    assert_eq!(READ_TIMEOUT, Duration::from_millis(50));
    assert_eq!(BUF_SIZE, 128);
    assert_eq!(END_AFTER_STOP, Duration::from_millis(500));
    assert_eq!(SILENCE_BOUND, Duration::from_secs(10));
}

#[test]
fn a_found_meter_with_no_settings_is_still_a_device() {
    let line = Shared::default().with(|l| l.answers.push_back(vc_packet("12345")));
    // The second listen hears nothing, for its 1500 ms.
    let at = Instant::now();
    assert_eq!(probe_on(&line, vc_model()), Probed::Found(None));
    assert!(at.elapsed() >= Duration::from_millis(1500));
}

#[test]
fn options_and_values() {
    let line = Shared::default();
    let params = params(Chip::Es51919);
    let settings = Settings { output_freq: 1000, circuit_model: "SERIES" };
    let mut d =
        LcrDevice::new(es_model(), "x".to_owned(), params, Some(settings), Some(line.port()));
    let keys: Vec<Key> = d.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(
        keys,
        [
            Key::Continuous,
            Key::LimitFrames,
            Key::LimitMsec,
            Key::OutputFrequency,
            Key::EquivCircuitModel
        ]
    );
    let access = |k: &Key| d.option(k, None).unwrap().access;
    assert!(!access(&Key::LimitMsec).get && access(&Key::LimitMsec).set);
    assert!(access(&Key::LimitFrames).get && access(&Key::LimitFrames).set);
    assert!(access(&Key::OutputFrequency).get && !access(&Key::OutputFrequency).set);
    assert!(access(&Key::OutputFrequency).list);
    assert!(access(&Key::EquivCircuitModel).get && !access(&Key::EquivCircuitModel).set);
    assert!(access(&Key::EquivCircuitModel).list);
    assert_eq!(d.get(&Key::OutputFrequency, None).unwrap(), Value::F64(1000.0));
    assert_eq!(d.get(&Key::EquivCircuitModel, None).unwrap(), Value::Str("SERIES".to_owned()));
    d.set(&Key::LimitFrames, &Value::U64(3), None).unwrap();
    assert_eq!(d.get(&Key::LimitFrames, None).unwrap(), Value::U64(3));
    d.set(&Key::LimitMsec, &Value::U64(7), None).unwrap();
    assert!(d.set(&Key::OutputFrequency, &Value::F64(1.0), None).is_err());
    assert!(d.set(&Key::LimitFrames, &Value::Str("x".to_owned()), None).is_err());
    assert!(d.get(&Key::Samplerate, None).is_err());
    assert!(d.get(&Key::LimitFrames, Some("x")).is_err());
    assert_eq!(
        d.list(&Key::OutputFrequency, None).unwrap(),
        ValueList::F64(vec![0.0, 100.0, 120.0, 1000.0, 10_000.0, 100_000.0])
    );
    assert_eq!(
        d.list(&Key::EquivCircuitModel, None).unwrap(),
        ValueList::Str(
            vec!["NONE", "PARALLEL", "SERIES", "AUTO"].into_iter().map(str::to_owned).collect()
        )
    );
    assert!(d.list(&Key::LimitFrames, None).is_err());
    // Nothing known: the frequency reads 0, the model not at all.
    let d = LcrDevice::from_port(vc_model(), Shared::default().port());
    assert_eq!(d.get(&Key::OutputFrequency, None).unwrap(), Value::F64(0.0));
    assert!(d.get(&Key::EquivCircuitModel, None).is_err());
    let access = d.option(&Key::EquivCircuitModel, None).unwrap().access;
    assert!(!access.get && access.list, "not to be read yet, still listed");
    assert_eq!(d.list(&Key::OutputFrequency, None).unwrap(), ValueList::F64(vec![120.0, 1000.0]));
}

#[test]
fn a_capture_needs_an_open_port() {
    let mut d = LcrDevice::from_port(es_model(), Shared::default().port());
    let (tx, _rx) = sync_channel(4);
    assert!(d.start(PacketSink::new(tx, 0)).is_err());
}

#[test]
fn an_open_flushes_once() {
    let line = Shared::default().with(|l| l.queued.extend(es_packet(1)));
    let mut d = LcrDevice::from_port(es_model(), line.port());
    d.open().unwrap();
    assert!(line.line().queued.is_empty());
    d.open().unwrap();
    assert_eq!(line.line().purges, 1);
}

#[test]
fn frames_say_what_changed_before_their_values() {
    bounded(|| {
        let mut other = es_packet(2);
        other[3] = 0x10;
        other[2] = 0xa0;
        // Then the frequency alone.
        let mut last = other.clone();
        last[3] = 0x30;
        let line = Shared::default().with(|l| {
            l.queued.extend(es_packet(1));
            l.queued.extend(es_packet(1));
            l.queued.extend(other);
            l.queued.extend(last);
        });
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(4), None).unwrap();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert!(matches!(got[0], Packet::Header { .. }), "{:?}", got[0]);
        assert_eq!(got[1], Packet::meta(Key::OutputFrequency, 1000.0));
        assert_eq!(got[2], Packet::meta(Key::EquivCircuitModel, "SERIES".to_owned()));
        assert_eq!(got[3], Packet::FrameBegin);
        assert!(matches!(&got[4], Packet::Analog(a) if a.channels == [0]));
        assert!(matches!(&got[5], Packet::Analog(a) if a.channels == [1]));
        assert_eq!(got[6], Packet::FrameEnd);
        assert_eq!(got[7], Packet::FrameBegin, "nothing changed");
        assert_eq!(frames(&got), 4);
        let m = metas(&got);
        assert_eq!(m.len(), 5, "{m:?}");
        assert_eq!(m[4], (Key::OutputFrequency, Value::F64(120.0)));
        assert_eq!(m[2], (Key::OutputFrequency, Value::F64(100.0)));
        assert_eq!(m[3], (Key::EquivCircuitModel, Value::Str("PARALLEL".to_owned())));
        // A get sees the capture's last, and the next capture starts anew.
        assert_eq!(d.get(&Key::OutputFrequency, None).unwrap(), Value::F64(120.0));
        assert_eq!(
            d.get(&Key::EquivCircuitModel, None).unwrap(),
            Value::Str("PARALLEL".to_owned())
        );
        line.line().queued.extend(es_packet(1));
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(metas(&got).len(), 2, "{got:?}");
    });
}

#[test]
fn a_start_forgets_what_the_scan_knew() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.hold = 1_000_000;
        });
        let params = params(Chip::Es51919);
        let settings = Settings { output_freq: 1000, circuit_model: "SERIES" };
        let mut d =
            LcrDevice::new(es_model(), "x".to_owned(), params, Some(settings), Some(line.port()));
        d.open().unwrap();
        let rx = start(&mut d);
        let deadline = Instant::now() + BOUND;
        while d.get(&Key::EquivCircuitModel, None).is_ok() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(d.get(&Key::OutputFrequency, None).unwrap(), Value::F64(0.0));
        d.stop().unwrap();
        packets(&rx);
    });
}

#[test]
fn a_packet_with_nothing_to_show_counts_for_nothing() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            // A read at a time, so that a limit could bite between them.
            l.per_read = 5;
            l.queued.extend(es_blank());
            l.queued.extend(es_packet(7));
            l.queued.extend(es_blank());
            l.queued.extend(es_packet(8));
        });
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(frames(&got), 2);
        assert_eq!(values(&got, 0).len(), 2);
    });
}

#[test]
fn a_channel_that_is_off_keeps_a_frame_of_the_other() {
    bounded(|| {
        let line = Shared::default().with(|l| l.queued.extend(es_packet(1)));
        let mut d = device(es_model(), &line);
        d.enable_channel(0, false).unwrap();
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(frames(&got), 1);
        assert_eq!(values(&got, 0).len(), 0);
        assert_eq!(values(&got, 1).len(), 1);
    });
    bounded(|| {
        // Off, and the other display blank: no frame, so no count and no Meta.
        let mut p = es_packet(1);
        p[14] = 1;
        let line = Shared::default().with(|l| {
            l.queued.extend(p);
            l.queued.extend(es_packet(1));
        });
        let mut d = device(es_model(), &line);
        d.enable_channel(0, false).unwrap();
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(frames(&got), 1);
        assert_eq!(metas(&got).len(), 2);
        assert!(line.line().queued.is_empty(), "the second packet was needed");
    });
}

#[test]
fn a_read_of_several_packets_goes_out_whole_past_the_limit() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            for v in 1..=3 {
                l.queued.extend(es_packet(v));
            }
        });
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(frames(&packets(&rx)), 3);
    });
}

#[test]
fn a_limit_in_a_read_cut_small_stops_at_once() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.per_read = 5;
            for v in 1..=3 {
                l.queued.extend(es_packet(v));
            }
        });
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(frames(&packets(&rx)), 1);
        assert!(!line.line().queued.is_empty(), "the rest stays on the line");
    });
}

#[test]
fn noise_is_stepped_over_a_byte_at_a_time() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.queued.extend(vec![0x3f; 300]);
            l.queued.extend(es_packet(5));
            // Part of a packet before a whole one.
            l.queued.extend(&es_packet(6)[..9]);
            l.queued.extend(es_packet(7));
        });
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(values(&got, 0).len(), 2);
        assert!((values(&got, 0)[0] - 5.0 * 1e-12).abs() < 1e-14, "{got:?}");
        assert!((values(&got, 0)[1] - 7.0 * 1e-12).abs() < 1e-14, "{got:?}");
    });
}

#[test]
fn a_time_limit_ends_a_silent_capture_from_both_sides() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitMsec, &Value::U64(300), None).unwrap();
        let at = Instant::now();
        let rx = start(&mut d);
        let got = packets(&rx);
        assert!(at.elapsed() >= Duration::from_millis(300), "{:?}", at.elapsed());
        assert_eq!(got.len(), 1, "the header alone");
    });
}

#[test]
fn a_time_limit_ends_a_stream_too() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(es_packet(1));
            l.byte_time = Duration::from_millis(1);
        });
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitMsec, &Value::U64(100), None).unwrap();
        let rx = start(&mut d);
        assert!(frames(&packets(&rx)) >= 1);
    });
}

#[test]
fn a_limit_set_under_a_capture_applies_as_in_c() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(es_packet(1));
            l.byte_time = Duration::from_millis(1);
        });
        let mut d = device(es_model(), &line);
        let rx = start(&mut d);
        // Frames are coming, with no limit.
        let deadline = Instant::now() + BOUND;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if rx.recv_timeout(left).expect("frames").1 == Packet::FrameEnd {
                break;
            }
        }
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        packets(&rx);
    });
}

#[test]
fn a_meter_that_is_asked_is_asked_after_a_silence_and_no_sooner_than_500_ms() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(vc_model(), &line);
        let at = Instant::now();
        let rx = start(&mut d);
        line.wait_for_writes(3);
        d.stop().unwrap();
        packets(&rx);
        let first = line.line().writes[0];
        assert!(first - at >= Duration::from_millis(50), "{:?}", first - at);
        assert!(line.least_gap() >= Duration::from_millis(500), "{:?}", line.least_gap());
    });
}

#[test]
fn a_streaming_meter_is_never_asked() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitMsec, &Value::U64(120), None).unwrap();
        let rx = start(&mut d);
        packets(&rx);
        assert_eq!(line.line().writes.len(), 0);
    });
}

/// 1200 Bd, ten bits a byte.
const VC_BYTE: Duration = Duration::from_micros(8333);

#[test]
fn at_the_lines_rate_the_requests_wait_for_the_answers() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.per_read = 1;
            l.byte_time = VC_BYTE;
            for v in ["11111", "22222", "33333"] {
                l.answers.push_back(vc_packet(v));
            }
        });
        let mut d = device(vc_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(3), None).unwrap();
        let rx = start(&mut d);
        let got = packets(&rx);
        let main = values(&got, 0);
        assert_eq!(main.len(), 3, "{got:?}");
        for (v, want) in main.iter().zip([1.1111f32, 2.2222, 3.3333]) {
            assert!((v - want).abs() < 1e-3, "{main:?}");
        }
        let l = line.line();
        assert_eq!(l.writes_during_a_reply, 0);
        assert_eq!(l.writes.len(), 3);
    });
}

#[test]
fn an_answer_longer_than_the_interval_is_not_talked_over() {
    // Noise and a packet taking 650 ms at the line's rate, over the 500 ms.
    bounded(|| {
        let mut long = vec![0x3f; 39];
        long.extend(vc_packet("11111"));
        let line = Shared::default().with(|l| {
            l.per_read = 1;
            l.byte_time = VC_BYTE;
            l.answers.push_back(long);
            l.answers.push_back(vc_packet("22222"));
        });
        let mut d = device(vc_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&packets(&rx), 0).len(), 2);
        assert_eq!(line.line().writes_during_a_reply, 0);
    });
}

#[test]
fn the_interval_counts_from_the_last_request_after_a_long_answer() {
    // A request that went late, because its forerunner's answer outlasted the
    // interval, still puts the next one a whole interval away.
    bounded(|| {
        let mut long = vec![0x3f; 39];
        long.extend(vc_packet("11111"));
        let line = Shared::default().with(|l| {
            l.per_read = 1;
            l.byte_time = VC_BYTE;
            l.answers.push_back(long);
        });
        let mut d = device(vc_model(), &line);
        let rx = start(&mut d);
        line.wait_for_writes(3);
        d.stop().unwrap();
        packets(&rx);
        assert!(line.least_gap() >= Duration::from_millis(500), "{:?}", line.least_gap());
    });
}

#[test]
fn the_interval_holds_across_captures_on_one_open_device() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            for v in ["11111", "22222", "33333"] {
                l.answers.push_back(vc_packet(v));
            }
        });
        let mut d = device(vc_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        for _ in 0..3 {
            let rx = start(&mut d);
            assert_eq!(frames(&packets(&rx)), 1);
        }
        assert_eq!(line.line().writes.len(), 3);
        assert!(line.least_gap() >= Duration::from_millis(500), "{:?}", line.least_gap());
    });
}

#[test]
fn a_request_may_wait_500_ms_for_room() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(vc_model(), &line);
        let rx = start(&mut d);
        line.wait_for_writes(1);
        d.stop().unwrap();
        packets(&rx);
        assert_eq!(line.line().write_timeouts[0], Duration::from_millis(500));
    });
}

#[test]
fn a_packet_is_stepped_over_whole() {
    // A meter in calibration, no frame, with a head at byte 4 and the next
    // packet's tail 15 bytes on from it, reading 4.369 nF if it were looked at.
    bounded(|| {
        let p = vec![0, 0x0d, 0x0d, 0x0a, 0, 0x0d, 0, 0, 0, 2, 0x11, 0x11, 0x53, 0, 0, 0x0d, 0x0a];
        let line = Shared::default().with(|l| {
            l.queued.extend(&p);
            l.queued.extend(&p);
            l.queued.extend(es_packet(7));
        });
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        let main = values(&packets(&rx), 0);
        assert_eq!(main.len(), 1, "{main:?}");
        assert!((main[0] - 7.0 * 1e-12).abs() < 1e-14, "{main:?}");
    });
}

/// A device as [`device`], whose silence has this bound.
fn device_with_bound(model: &'static Model, line: &Shared, bound: Duration) -> LcrDevice {
    let mut d = device(model, line);
    d.silence_bound = bound;
    d
}

#[test]
fn silence_means_what_the_chip_makes_it_mean() {
    let at = Instant::now();
    let later = |ms| at + Duration::from_millis(ms);
    let bound = Duration::from_secs(10);
    // An ES51919 at a frame limit, from both sides of the bound.
    let mut s = Silence::new(bound, at);
    assert_eq!(s.verdict(Chip::Es51919, true, later(9_999)), Silent::Listen);
    assert_eq!(s.verdict(Chip::Es51919, true, later(10_000)), Silent::End);
    // Bytes put it off.
    let mut s = Silence::new(bound, at);
    s.heard(later(5_000));
    assert_eq!(s.verdict(Chip::Es51919, true, later(14_999)), Silent::Listen);
    assert_eq!(s.verdict(Chip::Es51919, true, later(15_000)), Silent::End);
    // Something else can end the capture: no say.
    let mut s = Silence::new(bound, at);
    assert_eq!(s.verdict(Chip::Es51919, false, later(100_000)), Silent::Listen);
    // A Voltcraft: a warning once, at any limit, and it never ends.
    for only_frames in [true, false] {
        let mut s = Silence::new(bound, at);
        assert_eq!(s.verdict(Chip::Vc4080, only_frames, later(9_999)), Silent::Listen);
        assert_eq!(s.verdict(Chip::Vc4080, only_frames, later(10_000)), Silent::Warn);
        assert_eq!(s.verdict(Chip::Vc4080, only_frames, later(10_001)), Silent::Listen);
        // Bytes do not bring the warning back.
        s.heard(later(20_000));
        assert_eq!(s.verdict(Chip::Vc4080, only_frames, later(40_000)), Silent::Listen);
    }
}

#[test]
fn a_silent_es51919_at_a_frame_limit_ends() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device_with_bound(es_model(), &line, Duration::from_millis(100));
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let at = Instant::now();
        let rx = start(&mut d);
        assert_eq!(packets(&rx).len(), 1, "the header");
        assert!(at.elapsed() >= Duration::from_millis(100));
    });
}

#[test]
fn a_time_limit_keeps_a_silent_es51919_past_the_bound() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device_with_bound(es_model(), &line, Duration::from_millis(50));
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        d.set(&Key::LimitMsec, &Value::U64(300), None).unwrap();
        let at = Instant::now();
        let rx = start(&mut d);
        packets(&rx);
        assert!(at.elapsed() >= Duration::from_millis(300));
    });
}

/// Whether a capture is still going once its bound has long passed.
fn still_going_past_the_bound(rx: &Received, wait: Duration) -> bool {
    let until = Instant::now() + wait;
    loop {
        let left = until.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((_, Packet::End)) => return false,
            Ok(_) => {}
            Err(_) => return true,
        }
    }
}

#[test]
fn a_silent_es51919_with_no_limit_goes_on() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device_with_bound(es_model(), &line, Duration::from_millis(50));
        let rx = start(&mut d);
        assert!(still_going_past_the_bound(&rx, Duration::from_millis(300)));
        d.stop().unwrap();
        packets(&rx);
    });
}

#[test]
fn noise_counts_as_bytes_as_calibration_has_no_frames() {
    bounded(|| {
        // Blank packets, a byte at a time: a capture of no frames, with bytes.
        let line = Shared::default().with(|l| {
            l.per_read = 1;
            l.byte_time = Duration::from_millis(1);
            l.stream = Some(es_blank());
        });
        let mut d = device_with_bound(es_model(), &line, Duration::from_secs(1));
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert!(still_going_past_the_bound(&rx, Duration::from_secs(2)));
        d.stop().unwrap();
        assert_eq!(frames(&packets(&rx)), 0);
    });
}

#[test]
fn a_silent_voltcraft_at_a_frame_limit_goes_on_and_keeps_asking() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device_with_bound(vc_model(), &line, Duration::from_millis(50));
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        line.wait_for_writes(2);
        assert!(still_going_past_the_bound(&rx, Duration::from_millis(100)));
        d.stop().unwrap();
        packets(&rx);
    });
}

#[test]
fn a_late_answer_shifts_nothing() {
    bounded(|| {
        // The first request is answered after the next was asked for: the
        // answers come together, and each still reads what it read.
        let line = Shared::default().with(|l| {
            l.hold = 16;
            l.answers.push_back(vc_packet("11111"));
            l.answers.push_back(vc_packet("22222"));
            l.answers.push_back(vc_packet("33333"));
        });
        let mut d = device(vc_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(3), None).unwrap();
        let rx = start(&mut d);
        let got = packets(&rx);
        let main = values(&got, 0);
        assert!(main.len() >= 2, "{got:?}");
        for (v, want) in main.iter().zip([1.1111f32, 2.2222, 3.3333]) {
            assert!((v - want).abs() < 1e-3, "{main:?}");
        }
        assert!(line.line().writes.len() >= 2);
    });
}

#[test]
fn a_stop_during_an_answer_leaves_no_half_for_the_next_capture() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.per_read = 1;
            l.byte_time = VC_BYTE;
            l.answers.push_back(vc_packet("11111"));
        });
        let mut d = device(vc_model(), &line);
        let rx = start(&mut d);
        // A stop lands with part of the answer read and the rest on its way.
        let deadline = Instant::now() + BOUND;
        while line.line().queued.len() > 19 || line.line().writes.is_empty() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        let got = packets(&rx);
        assert_eq!(frames(&got), 0, "{got:?}");
        // The next capture: what is left of the answer goes the way of noise,
        // with a tail like the first's head in phase, which in C makes a packet
        // of the old head and a new tail. Here the head is gone.
        let left = line.line().queued.len();
        assert!(left > 0 && left < 39, "{left}");
        line.line().per_read = 0;
        line.line().byte_time = Duration::ZERO;
        line.line().queued.clear();
        line.line().queued.extend(&vc_packet("22222")[39 - left..]);
        line.line().queued.extend(vc_packet("33333"));
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        let got = packets(&rx);
        let main = values(&got, 0);
        assert_eq!(main.len(), 1, "{got:?}");
        assert!((main[0] - 3.3333).abs() < 1e-3, "{main:?}");
    });
}

#[test]
fn a_setting_right_after_a_capture_touches_no_line() {
    bounded(|| {
        let line = Shared::default().with(|l| l.queued.extend(es_packet(1)));
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        packets(&rx);
        d.set(&Key::LimitFrames, &Value::U64(9), None).unwrap();
        d.set(&Key::LimitMsec, &Value::U64(9), None).unwrap();
        let l = line.line();
        assert_eq!(l.purges, 1, "the open's");
        assert_eq!(l.writes.len(), 0);
    });
}

#[test]
fn a_stop_ends_a_capture_that_is_offered_data() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(es_packet(1));
            l.byte_time = Duration::from_millis(1);
        });
        let mut d = device(es_model(), &line);
        let rx = start(&mut d);
        let deadline = Instant::now() + BOUND;
        while line.line().reads < 3 {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        packets(&rx);
    });
}

#[test]
fn a_stop_does_not_join() {
    bounded(|| {
        // A queue of one, unread: the thread is stuck on it. A stop returns,
        // and a close gives up the End within its bound.
        let line = Shared::default().with(|l| {
            l.stream = Some(es_packet(1));
        });
        let mut d = device(es_model(), &line);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        while !matches!(rx.try_recv(), Ok((_, Packet::Header { .. }))) {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        // Let it fill the queue.
        while line.line().reads == 0 {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        d.close().unwrap();
        drop(rx);
    });
}

#[test]
fn a_start_after_an_end_stuck_on_a_full_queue_does_not_hang() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(es_model(), &line);
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
        // The start told the old thread and joined it: it is gone, with its
        // End given up. Had it only been let go, it would now send that End.
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert_eq!(
            rx.recv_timeout(BOUND).err(),
            Some(std::sync::mpsc::RecvTimeoutError::Disconnected)
        );
        packets(&rx2);
    });
}

#[test]
fn an_end_waits_for_room_while_nobody_asked_to_stop() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
        // The header fills a queue of one, and the port's return says the End
        // is what the thread has left. It gets through once there is room.
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        while d.port.lock().unwrap().is_none() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        std::thread::sleep(Duration::from_millis(5));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert_eq!(rx.recv_timeout(BOUND).unwrap().1, Packet::End);
    });
}

#[test]
fn a_close_under_a_capture_waits_for_its_thread() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(es_packet(1));
            l.byte_time = Duration::from_millis(1);
        });
        let mut d = device(es_model(), &line);
        let rx = start(&mut d);
        d.close().unwrap();
        assert!(d.worker.is_none());
        // The thread is through: it gave the port back, and the close took it.
        assert!(d.port.lock().unwrap().is_none());
        drop(rx);
    });
}

#[test]
fn a_search_warns_above_two_packets_dropped() {
    // What the emulator's noise cases show on stderr, with the other side here.
    assert!(!dropped_much(17, 17));
    assert!(!dropped_much(51, 17));
    assert!(dropped_much(52, 17));
    assert!(!dropped_much(117, 39));
    assert!(dropped_much(118, 39));
}

#[test]
fn a_start_during_a_capture_says_busy() {
    bounded(|| {
        let line = Shared::default().with(|l| l.hold = 1_000_000);
        let mut d = device(es_model(), &line);
        let rx = start(&mut d);
        let (tx, _rx2) = sync_channel(4);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.stop().unwrap();
        packets(&rx);
    });
}

#[test]
fn a_port_that_hangs_up_ends_the_capture() {
    bounded(|| {
        let line = Shared::default().with(|l| l.read_fails = true);
        // Far from the silence's bound, which must not be what ends it.
        let mut d = device_with_bound(es_model(), &line, BOUND * 10);
        d.set(&Key::LimitFrames, &Value::U64(5), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(packets(&rx).len(), 1, "the header");
    });
}

#[test]
fn a_consumer_that_left_ends_the_capture() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(es_packet(1));
            l.byte_time = Duration::from_millis(1);
        });
        let mut d = device(es_model(), &line);
        let rx = start(&mut d);
        drop(rx);
        // The port coming back says the thread is through.
        let deadline = Instant::now() + BOUND;
        while d.port.lock().unwrap().is_none() {
            assert!(Instant::now() < deadline, "the capture went on");
            std::thread::sleep(Duration::from_millis(1));
        }
    });
}

#[test]
fn a_request_that_fails_or_finds_no_room_is_only_a_silence() {
    for fail in [false, true] {
        bounded(move || {
            let line = Shared::default().with(|l| {
                if fail {
                    l.write_fails = true;
                } else {
                    l.no_room = true;
                }
            });
            let mut d = device(vc_model(), &line);
            let rx = start(&mut d);
            // The capture goes on, and asks again.
            let deadline = Instant::now() + BOUND;
            if fail {
                while line.line().reads < 20 {
                    assert!(Instant::now() < deadline);
                    std::thread::sleep(Duration::from_millis(2));
                }
            } else {
                line.wait_for_writes(2);
                assert!(line.least_gap() >= Duration::from_millis(500));
            }
            d.stop().unwrap();
            packets(&rx);
        });
    }
}

#[test]
fn a_close_after_a_capture_gives_the_port_up() {
    bounded(|| {
        let line = Shared::default().with(|l| l.queued.extend(es_packet(1)));
        let mut d = device(es_model(), &line);
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        packets(&rx);
        d.close().unwrap();
        assert!(d.port.lock().unwrap().is_none());
        let (tx, _rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err(), "not open");
    });
}
