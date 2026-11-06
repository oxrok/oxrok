// Counterpart in sigrok:
//   libsigrok/src/hardware/teleinfo/api.c,
//   libsigrok/src/hardware/teleinfo/protocol.c,
//   Copyright (C) 2013 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Martin Ling, Daniel Elstner, Gerhard Sittig, Soeren Apel, Wolfram Sang,
//   Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake port. A fake's bytes go by counts of reads, stretched by a
//! load but never lost to it, so nothing here times a race: limits on real time are
//! one-sided or wide, and each capture has an overall bound.

use super::*;
use crate::core::{Mq, Unit};
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// Far above anything a test waits for, however slow the box: it only makes a stop
/// or a limit that does not bite fail rather than hang.
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
    /// Bytes come only on every Nth read, the others time out: a cable that holds
    /// them for 50 ms and gives a burst; 0 for every read.
    every: usize,
    /// Bytes of the queue come by the clock, one per this time from this start: a
    /// slow sleep or a stall in the reader delays no byte, as on a real line.
    paced: Option<(Instant, Duration)>,
    /// How many bytes a paced line has given.
    given: u32,
    /// What a read finds whenever nothing is queued: a meter that streams.
    stream: Option<Vec<u8>>,
    read_fails: bool,
    /// Reads that fail before they stop failing.
    failed_reads: usize,
    /// What was written, which no test should see.
    writes: crate::serial::WriteLog,
    /// Reads that give nothing, at once, before the queue flows.
    empty_reads: usize,
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

    fn queue(&self, bytes: &[u8]) {
        self.line().queued.extend(bytes.iter().copied());
    }

    /// Until `n` reads were made, within the bound.
    fn wait_for_reads(&self, n: usize) {
        let deadline = Instant::now() + BOUND;
        while self.line().reads < n {
            assert!(Instant::now() < deadline, "{n} reads were not made");
            std::thread::sleep(Duration::from_millis(2));
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
        if line.empty_reads > 0 {
            line.empty_reads -= 1;
            return Ok(0);
        }
        let held = line.every > 0 && !line.reads.is_multiple_of(line.every);
        if line.queued.is_empty()
            && !held
            && let Some(p) = line.stream.clone()
        {
            line.queued.extend(p);
        }
        if line.queued.is_empty() || held {
            drop(line);
            std::thread::sleep(timeout);
            return Ok(0);
        }
        let mut most = if line.per_read == 0 { buf.len() } else { line.per_read.min(buf.len()) };
        if let Some((from, byte_time)) = line.paced {
            let due = u32::try_from(from.elapsed().as_micros() / byte_time.as_micros());
            let due = due.expect("small").saturating_sub(line.given);
            if due == 0 {
                drop(line);
                std::thread::sleep(timeout.min(byte_time));
                return Ok(0);
            }
            most = most.min(due as usize);
        }
        let n = most.min(line.queued.len());
        line.given += u32::try_from(n).expect("small");
        for b in &mut buf[..n] {
            *b = line.queued.pop_front().expect("counted");
        }
        let wait = line.byte_time * u32::try_from(n).expect("small");
        drop(line);
        std::thread::sleep(wait);
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.0.line().writes.push((Instant::now(), data.to_vec()));
        Ok(())
    }
    fn purge(&mut self) -> Result<()> {
        let mut line = self.0.line();
        line.queued.clear();
        line.purges += 1;
        Ok(())
    }
}

/// A group, as a meter sends it.
fn group(label: &str, data: &str) -> Vec<u8> {
    let sum: u32 = label.bytes().chain(*b" ").chain(data.bytes()).map(u32::from).sum();
    let control = u8::try_from((sum & 0x3f) + 0x20).unwrap();
    let mut out = vec![b'\n'];
    out.extend_from_slice(label.as_bytes());
    out.push(b' ');
    out.extend_from_slice(data.as_bytes());
    out.push(b' ');
    out.push(control);
    out.push(b'\r');
    out
}

/// The groups of these (label, data) pairs in one frame.
fn frame(groups: &[(&str, &str)]) -> Vec<u8> {
    let mut out = vec![2];
    for (label, data) in groups {
        out.extend(group(label, data));
    }
    out.push(3);
    out
}

const ADCO: (&str, &str) = ("ADCO", "031428097115");

/// A meter's whole frame.
fn base_frame(index: &str) -> Vec<u8> {
    frame(&[
        ADCO,
        ("OPTARIF", "BASE"),
        ("ISOUSC", "30"),
        ("BASE", index),
        ("PTEC", "TH.."),
        ("IINST", "002"),
        ("IMAX", "030"),
        ("PAPP", "00460"),
        ("HHPHC", "A"),
        ("MOTDETAT", "000000"),
    ])
}

fn device(optarif: Optarif, line: &Shared) -> TeleinfoDevice {
    let mut d = TeleinfoDevice::from_port(optarif, line.port());
    let queued = std::mem::take(&mut line.line().queued);
    d.open().unwrap();
    line.line().queued = queued;
    d
}

fn limit_samples(d: &mut TeleinfoDevice, n: u64) {
    d.set(&Key::LimitSamples, &Value::U64(n), None).unwrap();
}

fn limit_msec(d: &mut TeleinfoDevice, n: u64) {
    d.set(&Key::LimitMsec, &Value::U64(n), None).unwrap();
}

fn start(d: &mut TeleinfoDevice) -> Received {
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

/// Bounds short enough for a test: this first, this behind an EOT, this after a
/// sample.
fn bounds(first: u64, interrupted: u64, after_sample: u64) -> Bounds {
    Bounds {
        first: Duration::from_millis(first),
        interrupted: Duration::from_millis(interrupted),
        after_sample: Duration::from_millis(after_sample),
    }
}

fn params() -> Params {
    Params::parse(SERIALCOMM).unwrap()
}

/// What a scan's 292 bytes come to, from a frame and padding.
fn scan_bytes(text: &[u8]) -> Vec<u8> {
    let mut out = text.to_vec();
    out.resize(SCAN_BYTES, b'0');
    out
}

#[test]
fn the_constants_are_c_s_and_the_specification_s() {
    assert_eq!(SCAN_BYTES, 292);
    assert_eq!(BUF_SIZE, 256);
    assert_eq!(SCAN_TIME, Duration::from_millis(3000));
    assert_eq!(READ_TIME, Duration::from_millis(50));
    assert_eq!(SERIALCOMM, "1200/7e1");
}

#[test]
fn a_scan_without_a_conn_or_a_port_finds_nothing_and_does_not_fail() {
    assert!(TeleinfoDriver.scan(&[]).unwrap().is_empty());
    let conn = (Key::Conn, Value::Str("/dev/ttyDOESNOTEXIST".to_owned()));
    assert!(TeleinfoDriver.scan(std::slice::from_ref(&conn)).unwrap().is_empty());
    let comm = (Key::Serialcomm, Value::Str("fast".to_owned()));
    assert!(TeleinfoDriver.scan(&[conn, comm]).unwrap().is_empty());
}

#[test]
fn a_scan_reads_292_bytes_and_no_more_and_finds_the_option() {
    let line = Shared::default();
    let mut bytes = scan_bytes(&frame(&[("OPTARIF", "HC..")]));
    bytes.extend(b"more");
    line.queue(&bytes);
    let found = detect(line.port().as_mut(), &params(), BOUND);
    assert_eq!(found, Probed::Found(Optarif::Hc));
    assert_eq!(line.line().queued.len(), 4, "what follows stays on the way");
    assert_eq!(line.line().writes.len(), 0);
}

#[test]
fn a_scan_takes_its_bytes_in_pieces() {
    let line = Shared::default().with(|l| l.per_read = 7);
    line.queue(&scan_bytes(&frame(&[("OPTARIF", "EJP.")])));
    assert_eq!(detect(line.port().as_mut(), &params(), BOUND), Probed::Found(Optarif::Ejp));
}

#[test]
fn a_scan_gives_up_at_its_time_with_too_few_bytes_even_with_an_option() {
    let line = Shared::default();
    line.queue(&frame(&[("OPTARIF", "BASE")]));
    let before = Instant::now();
    let found = detect(line.port().as_mut(), &params(), Duration::from_millis(100));
    assert_eq!(found, Probed::TooFew);
    assert!(before.elapsed() >= Duration::from_millis(100), "it waits out its time");
}

#[test]
fn a_scan_with_its_bytes_and_no_known_option_finds_nothing() {
    let line = Shared::default();
    line.queue(&scan_bytes(&frame(&[("OPTARIF", "TEMP")])));
    assert_eq!(detect(line.port().as_mut(), &params(), BOUND), Probed::Unknown);
    let line = Shared::default();
    line.queue(&scan_bytes(b""));
    assert_eq!(detect(line.port().as_mut(), &params(), BOUND), Probed::Unknown);
    assert!(line.line().queued.is_empty(), "looked only when all was in");
}

#[test]
fn a_failed_read_is_no_data_and_the_scan_goes_on() {
    let line = Shared::default().with(|l| l.failed_reads = 2);
    line.queue(&scan_bytes(&frame(&[("OPTARIF", "BASE")])));
    assert_eq!(detect(line.port().as_mut(), &params(), BOUND), Probed::Found(Optarif::Base));
    assert_eq!(line.line().reads, 3);
}

#[test]
fn a_scan_on_a_port_that_keeps_failing_ends_at_its_time() {
    let line = Shared::default().with(|l| l.read_fails = true);
    let found = detect(line.port().as_mut(), &params(), Duration::from_millis(100));
    assert_eq!(found, Probed::TooFew);
}

#[test]
fn a_scan_of_the_standard_form_says_so() {
    let line = Shared::default();
    line.queue(&scan_bytes(b"\x02\nADSC\t041876097115\tx\r"));
    assert_eq!(detect(line.port().as_mut(), &params(), BOUND), Probed::Standard);
    // A known option outranks it, and too few bytes say nothing of it.
    let line = Shared::default();
    let mut text = group("OPTARIF", "BASE");
    text.extend(b"\nADSC\t041876097115\tx\r");
    line.queue(&scan_bytes(&text));
    assert_eq!(detect(line.port().as_mut(), &params(), BOUND), Probed::Found(Optarif::Base));
    let line = Shared::default();
    line.queue(b"\x02\nADSC\t041876097115\tx\r");
    let found = detect(line.port().as_mut(), &params(), Duration::from_millis(50));
    assert_eq!(found, Probed::TooFew);
}

/// A line's real rate, 120 bytes a second by the clock: a scan takes 2.43 s of C's
/// 3 s, which is the time it is given here, whatever a slow sleep does to its reads.
#[test]
fn a_scan_at_the_lines_rate_fits_in_its_time() {
    let line = Shared::default().with(|l| {
        l.stream = Some(base_frame("000012000"));
        l.paced = Some((Instant::now(), Duration::from_micros(8333)));
    });
    let before = Instant::now();
    assert_eq!(detect(line.port().as_mut(), &params(), SCAN_TIME), Probed::Found(Optarif::Base));
    assert!(before.elapsed() >= Duration::from_millis(2400), "the bytes took their time");
}

#[test]
fn a_device_says_what_c_s_says() {
    let d = TeleinfoDevice::from_port(Optarif::Hc, Shared::default().port());
    assert_eq!(d.info().driver, "teleinfo");
    assert_eq!(d.info().vendor, "EDF");
    assert_eq!(d.info().model, "Teleinfo");
    assert_eq!(d.info().conn, None, "no conn in its options");
    assert_eq!(TeleinfoDriver.long_name(), "Teleinfo");
    assert_eq!(TeleinfoDriver.classes(), [DeviceClass::EnergyMeter]);
    assert_eq!(TeleinfoDriver.scan_options(), [Key::Conn, Key::Serialcomm]);
    let names: Vec<_> = d.channels().iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["P", "HP", "HC", "IINST", "PAPP"]);
    assert!(d.channels().iter().all(|c| c.enabled && c.kind == ChannelKind::Analog));
}

#[test]
fn its_options_can_only_be_set() {
    let mut d = TeleinfoDevice::from_port(Optarif::Base, Shared::default().port());
    let options = d.options(None).unwrap();
    let keys: Vec<_> = options.iter().map(|o| o.key.clone()).collect();
    assert_eq!(keys, [Key::Continuous, Key::LimitSamples, Key::LimitMsec]);
    assert!(options[1..].iter().all(|o| o.access.set && !o.access.get && !o.access.list));
    assert!(d.options(Some("x")).is_err());
    assert!(d.get(&Key::LimitSamples, None).is_err());
    assert!(d.get(&Key::LimitSamples, Some("x")).is_err());
    assert!(d.list(&Key::LimitSamples, None).is_err());
    assert!(d.list(&Key::LimitSamples, Some("x")).is_err());
    assert!(d.set(&Key::Samplerate, &Value::U64(1), None).is_err());
    assert!(d.set(&Key::LimitSamples, &Value::U64(1), Some("x")).is_err());
    assert!(d.set(&Key::LimitSamples, &Value::Str("x".to_owned()), None).is_err());
    d.set(&Key::LimitMsec, &Value::U64(5), None).unwrap();
    assert_eq!(d.limits.msec.load(Ordering::SeqCst), 5);
}

#[test]
fn a_capture_sends_each_reading_with_its_unit() {
    bounded(|| {
        let line = Shared::default();
        line.queue(&base_frame("000012345"));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert!(matches!(got[0], Packet::Header { .. }), "{got:?}");
        assert_eq!(failure(&got), None);
        let analog: Vec<_> = got
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some(a),
                _ => None,
            })
            .collect();
        assert_eq!(analog.len(), 3);
        assert_eq!(analog[0].channels, [1]);
        assert_eq!(analog[0].samples, [12345.0]);
        assert_eq!((analog[0].mq, analog[0].unit), (Mq::Energy, Unit::WattHour));
        assert_eq!(analog[0].digits, 0);
        assert_eq!(analog[0].flags, MqFlags::empty());
        assert_eq!(analog[1].channels, [2]);
        assert_eq!(analog[1].samples, [2.0]);
        assert_eq!((analog[1].mq, analog[1].unit), (Mq::Current, Unit::Ampere));
        assert_eq!(analog[2].channels, [3]);
        assert_eq!(analog[2].samples, [460.0]);
        assert_eq!((analog[2].mq, analog[2].unit), (Mq::Power, Unit::VoltAmpere));
        assert_eq!(line.line().writes.len(), 0, "a meter takes nothing");
    });
}

#[test]
fn a_big_index_is_its_nearest_float() {
    bounded(|| {
        let line = Shared::default();
        line.queue(&frame(&[
            ("BASE", "123456789"),
            ("BASE", "016777217"),
            ("BASE", "033554435"),
            ("BASE", "067108869"),
            ADCO,
        ]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        // 8 Wh steps, then 2, 4 and 8.
        assert_eq!(values(&got, 1), [123_456_792.0, 16_777_216.0, 33_554_436.0, 67_108_872.0]);
    });
}

#[test]
fn each_option_feeds_its_channels_and_no_others() {
    bounded(|| {
        let all = [
            ("BASE", "000000001"),
            ("HCHP", "000000002"),
            ("HCHC", "000000003"),
            ("EJPHN", "000000004"),
            ("EJPHPM", "000000005"),
            ("BBRHPJB", "000000006"),
            ("BBRHPJW", "000000007"),
            ("BBRHPJR", "000000008"),
            ("BBRHCJB", "000000011"),
            ("BBRHCJW", "000000012"),
            ("BBRHCJR", "000000013"),
            ADCO,
        ];
        for (optarif, wanted) in [
            (Optarif::Base, vec![(1, 1.0)]),
            (Optarif::Hc, vec![(1, 2.0), (2, 3.0)]),
            (Optarif::Ejp, vec![(1, 4.0), (2, 5.0)]),
            (Optarif::Bbr, vec![(1, 6.0), (2, 7.0), (3, 8.0), (4, 11.0), (5, 12.0), (6, 13.0)]),
        ] {
            let line = Shared::default();
            line.queue(&frame(&all));
            let mut d = device(optarif, &line);
            limit_samples(&mut d, 1);
            let got = packets(&start(&mut d));
            let readings: Vec<(usize, f32)> = got
                .iter()
                .filter_map(|p| match p {
                    Packet::Analog(a) => Some((a.channels[0], a.samples[0])),
                    _ => None,
                })
                .collect();
            assert_eq!(readings, wanted, "{optarif:?}");
        }
    });
}

#[test]
fn a_channel_that_is_off_gets_nothing_and_a_renamed_one_keeps_its_readings() {
    bounded(|| {
        let line = Shared::default();
        line.queue(&base_frame("000012345"));
        let mut d = device(Optarif::Base, &line);
        d.enable_channel(1, false).unwrap();
        d.rename_channel(2, "amps").unwrap();
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert!(values(&got, 1).is_empty(), "off");
        assert_eq!(values(&got, 2), [2.0], "under its new name");
        assert_eq!(d.channels()[2].name, "amps");
        assert!(d.enable_channel(9, true).is_err());
        assert!(d.rename_channel(9, "x").is_err());
    });
}

#[test]
fn a_limit_ends_within_a_read_at_the_end_of_a_frame() {
    bounded(|| {
        let line = Shared::default();
        // By its next ADCO, with no ETX, then by its ETX.
        line.queue(&frame(&[ADCO, ("BASE", "000000001"), ADCO, ("BASE", "000000002")]));
        line.queue(&frame(&[ADCO, ("BASE", "000000003")]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [1.0], "one frame, whatever else that read held");
        let line = Shared::default();
        line.queue(&frame(&[ADCO, ("BASE", "000000001")]));
        line.queue(&frame(&[ADCO, ("BASE", "000000002")]));
        line.queue(&frame(&[ADCO, ("BASE", "000000003")]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 2);
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [1.0, 2.0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn an_etx_with_no_adco_and_a_frame_cut_by_an_eot_are_no_samples() {
    bounded(|| {
        let line = Shared::default();
        line.queue(b"\x03\x03");
        line.queue(&frame(&[("BASE", "000000001")]));
        line.queue(&group("ADCO", "031428097115"));
        line.queue(&group("BASE", "000000002"));
        line.queue(b"\x04");
        line.queue(&frame(&[ADCO, ("BASE", "000000003")]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [1.0, 2.0, 3.0]);
        assert_eq!(failure(&got), None);
    });
}

/// `--samples 1` at the real rate, from a frame's ADCO on: a read ends after each
/// few bytes, and all of the frame must still come out. This is what C, which counts
/// at ADCO and looks after each read, by reading would cut.
#[test]
fn samples_1_at_the_lines_rate_gives_the_whole_frame() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.paced = Some((Instant::now(), Duration::from_micros(8333)));
        });
        line.queue(&base_frame("000012345"));
        line.queue(&base_frame("000012346"));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [12345.0]);
        assert_eq!(values(&got, 2), [2.0]);
        assert_eq!(values(&got, 3), [460.0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_value_that_is_no_reading_is_dropped_between_good_ones() {
    bounded(|| {
        let line = Shared::default();
        let mut bytes = vec![2];
        bytes.extend(group("BASE", "000012000"));
        // One flipped bit 6: the control stands.
        let mut flipped = group("BASE", "000012100");
        flipped[12] ^= 0x40;
        assert_eq!(protocol::next_group(&flipped).unwrap().1, flipped.len());
        assert!(protocol::next_group(&flipped).unwrap().0.is_some(), "the control passes");
        bytes.extend(flipped);
        for (label, data) in
            [("BASE", "12"), ("BASE", "+00012345"), ("IINST", "0q2"), ("PAPP", "460")]
        {
            bytes.extend(group(label, data));
        }
        bytes.extend(group("BASE", "000012001"));
        bytes.extend(group("ADCO", "031428097115"));
        bytes.push(3);
        line.queue(&bytes);
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [12000.0, 12001.0]);
        assert_eq!(values(&got, 2), [0f32; 0]);
        assert_eq!(values(&got, 3), [0f32; 0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn the_limit_ends_the_capture_after_the_read_that_met_it() {
    bounded(|| {
        let first = frame(&[ADCO]);
        let line = Shared::default().with(|l| l.per_read = first.len());
        line.queue(&first);
        line.queue(&frame(&[("BASE", "000000001"), ADCO, ("BASE", "000000002")]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert!(values(&got, 1).is_empty(), "{got:?}");
        assert!(!line.line().queued.is_empty(), "the rest is not read");
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn limit_0_is_no_limit() {
    bounded(|| {
        let line = Shared::default();
        line.queue(&base_frame("000012345"));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 0);
        limit_msec(&mut d, 200);
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [12345.0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_limit_set_under_a_capture_counts() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(frame(&[ADCO])));
        let mut d = device(Optarif::Base, &line);
        let rx = start(&mut d);
        line.wait_for_reads(3);
        limit_samples(&mut d, 1);
        assert_eq!(failure(&packets(&rx)), None);
    });
}

/// A meter at its line's rate, a byte per read, so groups cross many reads.
#[test]
fn a_capture_at_the_lines_rate_gives_what_a_whole_read_does() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.paced = Some((Instant::now(), Duration::from_micros(8333)));
        });
        line.queue(&frame(&[("BASE", "000000001"), ("IINST", "002"), ("PAPP", "00003")]));
        line.queue(&frame(&[ADCO]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [1.0]);
        assert_eq!(values(&got, 2), [2.0]);
        assert_eq!(values(&got, 3), [3.0]);
        assert!(line.line().queued.is_empty(), "the frame's ETX ended it");
        assert_eq!(failure(&got), None);
    });
}

/// A cable that holds bytes for its latency, and gives them in bursts.
#[test]
fn a_capture_of_bursts_gives_the_same() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.per_read = 64;
            l.every = 2;
        });
        line.queue(&base_frame("000012345"));
        line.queue(&frame(&[ADCO]));
        line.queue(&frame(&[ADCO]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 2);
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [12345.0]);
        assert_eq!(values(&got, 3), [460.0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn bad_groups_are_stepped_over() {
    bounded(|| {
        let line = Shared::default();
        line.queue(b"\x02junk\r\nBASE 1 x\r");
        line.queue(&group("BASE", "000000002"));
        line.queue(b"\x03\x04\x02");
        line.queue(&frame(&[("ADCO", "1")]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [2.0]);
    });
}

#[test]
fn a_control_of_a_space_counts_for_a_reading_and_a_sample() {
    bounded(|| {
        let line = Shared::default();
        // By C it is dropped: "BASE 000000399" and "ADCO 000000000075" have one.
        assert_eq!(group("BASE", "000000399").last_chunk::<2>(), Some(b" \r"));
        line.queue(&group("BASE", "000000399"));
        let adco = (0..1000)
            .map(|i| format!("{i:012}"))
            .find(|d| group("ADCO", d).ends_with(b"  \r"))
            .unwrap();
        line.queue(&group("ADCO", &adco));
        line.queue(b"\x03");
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [399.0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_full_buffer_fails_the_capture_and_says_so() {
    bounded(|| {
        let line = Shared::default();
        line.queue(&[b'0'; BUF_SIZE]);
        line.queue(&frame(&[ADCO]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), Some("device error: 256 bytes came with no group in them"));
        assert_eq!(line.line().queued.len(), frame(&[ADCO]).len(), "read no further");
    });
}

#[test]
fn a_group_that_ends_at_the_last_byte_of_the_buffer_is_taken() {
    bounded(|| {
        for (zeros, wanted) in [(238, Some(1.0)), (239, None)] {
            let line = Shared::default().with(|l| l.per_read = 64);
            line.queue(&vec![b'0'; zeros]);
            line.queue(&group("BASE", "000000001"));
            line.queue(&frame(&[ADCO]));
            let mut d = device(Optarif::Base, &line);
            limit_samples(&mut d, 1);
            let got = packets(&start(&mut d));
            assert_eq!(values(&got, 1).first().copied(), wanted, "{zeros}");
            assert_eq!(failure(&got).is_some(), wanted.is_none(), "{zeros}");
        }
    });
}

#[test]
fn a_time_limit_ends_a_silent_capture_from_both_sides() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(Optarif::Base, &line);
        limit_msec(&mut d, 150);
        let before = Instant::now();
        let got = packets(&start(&mut d));
        assert!(before.elapsed() >= Duration::from_millis(150), "not before its time");
        assert_eq!(failure(&got), None);
        assert!(line.line().reads >= 1, "a read comes first");
    });
}

#[test]
fn a_time_limit_ends_a_stream() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(base_frame("000012345")));
        let mut d = device(Optarif::Base, &line);
        limit_msec(&mut d, 100);
        let got = packets(&start(&mut d));
        assert_ne!(values(&got, 1), [0f32; 0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_time_limit_above_the_bound_is_not_cut_by_it() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(Optarif::Base, &line);
        d.bounds = bounds(50, 50, 50);
        limit_samples(&mut d, 1);
        limit_msec(&mut d, 250);
        let before = Instant::now();
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), None);
        assert!(before.elapsed() >= Duration::from_millis(250));
    });
}

#[test]
fn no_limit_has_no_bound() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(Optarif::Base, &line);
        d.bounds = bounds(20, 20, 20);
        let rx = start(&mut d);
        line.wait_for_reads(8);
        d.stop().unwrap();
        let got = packets(&rx);
        assert_eq!(failure(&got), None, "only a stop ended it");
    });
}

#[test]
fn a_silent_meter_with_no_sample_yet_fails_the_capture() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(Optarif::Base, &line);
        d.bounds = bounds(200, 60_000, 60_000);
        limit_samples(&mut d, 1);
        let before = Instant::now();
        let got = packets(&start(&mut d));
        assert_eq!(
            failure(&got),
            Some("device error: no bytes from the meter for 0 s, with samples to go")
        );
        assert!(before.elapsed() >= Duration::from_millis(200));
    });
}

#[test]
fn a_silent_meter_after_a_sample_is_waited_for_up_to_its_bound() {
    bounded(|| {
        let line = Shared::default();
        line.queue(&frame(&[ADCO, ("BASE", "000000001")]));
        let mut d = device(Optarif::Base, &line);
        d.bounds = bounds(20, 60_000, 400);
        limit_samples(&mut d, 2);
        let before = Instant::now();
        let got = packets(&start(&mut d));
        assert_eq!(values(&got, 1), [1.0]);
        assert_eq!(
            failure(&got),
            Some("device error: no bytes from the meter for 0 s, with samples to go")
        );
        assert!(before.elapsed() >= Duration::from_millis(400), "not at the first bound");
    });
}

#[test]
fn a_stream_with_no_adco_at_a_sample_limit_fails_and_says_bytes_came() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(frame(&[("BASE", "000000001")]));
        });
        let mut d = device(Optarif::Base, &line);
        d.bounds = bounds(200, 60_000, 60_000);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert_eq!(
            failure(&got),
            Some(
                "device error: bytes come, but no ADCO group for 0 s, with samples to go: is the \
                 line 1200/7e1, and the meter in historic mode?"
            )
        );
        assert!(!values(&got, 1).is_empty(), "its readings went out");
    });
}

#[test]
fn a_stream_of_wrong_frames_is_no_sample_either() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.stream = Some(b"\nADCO 031428097115 x\r".to_vec());
        });
        let mut d = device(Optarif::Base, &line);
        d.bounds = bounds(200, 60_000, 60_000);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert!(failure(&got).unwrap().contains("no ADCO group"), "{got:?}");
    });
}

/// A consumer that takes longer per packet than the bound, on reads that hold no
/// `ADCO`: that time is not the meter's.
#[test]
fn a_slow_consumer_does_not_fail_the_capture() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.per_read = 16;
            l.stream = Some(frame(&[
                ("BASE", "000000001"),
                ("BASE", "000000002"),
                ("BASE", "000000003"),
                ADCO,
            ]));
        });
        let mut d = device(Optarif::Base, &line);
        d.bounds = bounds(100, 100, 100);
        limit_samples(&mut d, 3);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let (_, p) = rx.recv_timeout(left).expect("no End in time");
            assert!(!matches!(p, Packet::Failed(_)), "{p:?}");
            if p == Packet::End {
                break;
            }
            std::thread::sleep(Duration::from_millis(200));
        }
    });
}

/// Bytes ready when a read comes win over a timer that would have run out in the
/// gap before it: the first read gives nothing at once, the pause outlasts the
/// bound, and the frame is there for the next.
#[test]
fn a_ready_frame_wins_over_the_timer() {
    bounded(|| {
        let line = Shared::default().with(|l| l.empty_reads = 1);
        line.queue(&frame(&[ADCO, ("BASE", "000000001")]));
        let mut d = device(Optarif::Base, &line);
        d.bounds = bounds(100, 100, 100);
        d.empty_pause = Duration::from_millis(300);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), None);
        assert_eq!(values(&got, 1), [1.0]);
    });
}

const LIMITED: bool = true;
const FREE: bool = false;

#[test]
fn the_constants_of_the_waits() {
    assert_eq!(BOUNDS.first, Duration::from_secs(10));
    assert_eq!(BOUNDS.interrupted, Duration::from_secs(300));
    assert_eq!(BOUNDS.after_sample, Duration::from_secs(600));
}

#[test]
fn what_each_wait_says() {
    let s = Duration::from_secs(10);
    for (lack, on, text) in [
        (
            Lack::NoBytes,
            true,
            "no bytes from the meter for 10 s: is it on, and its cable in? Waiting on",
        ),
        (Lack::NoBytes, false, "no bytes from the meter for 10 s, with samples to go"),
        (
            Lack::NoSample,
            true,
            "bytes come, but no ADCO group for 10 s: is the line 1200/7e1, and the meter in \
             historic mode? Waiting on",
        ),
        (
            Lack::NoSample,
            false,
            "bytes come, but no ADCO group for 10 s, with samples to go: is the line 1200/7e1, \
             and the meter in historic mode?",
        ),
        (
            Lack::Interrupted,
            true,
            "the meter announced an interruption (EOT), and no sample for 10 s. Waiting on",
        ),
        (
            Lack::Interrupted,
            false,
            "the meter announced an interruption (EOT) and sent nothing for 10 s",
        ),
        (
            Lack::InterruptedNoSample,
            false,
            "no ADCO group for 10 s, around the meter's interruptions (EOT)",
        ),
    ] {
        assert_eq!(said(lack, s, on), text);
    }
}

#[test]
fn no_sample_yet_fails_at_the_first_bound_from_both_sides() {
    let now = Instant::now();
    let s = Duration::from_secs;
    let mut w = Watch::new(BOUNDS, now);
    assert_eq!(w.verdict(LIMITED, now + Duration::from_millis(9999)), Wait::Fine);
    assert_eq!(w.verdict(LIMITED, now + s(10)), Wait::Fail(Lack::NoBytes, s(10)));
    let mut w = Watch::new(BOUNDS, now);
    w.heard(b'x', now + s(9));
    assert_eq!(w.verdict(LIMITED, now + s(9)), Wait::Fine, "one stray byte");
    assert_eq!(w.verdict(LIMITED, now + s(10)), Wait::Fail(Lack::NoSample, s(10)));
}

#[test]
fn after_a_sample_it_warns_once_and_fails_at_the_long_bound() {
    let now = Instant::now();
    let s = Duration::from_secs;
    let mut w = Watch::new(BOUNDS, now);
    w.heard(b'x', now);
    w.counted(now);
    assert_eq!(w.verdict(LIMITED, now + s(9)), Wait::Fine);
    assert_eq!(w.verdict(LIMITED, now + s(10)), Wait::Warn(Lack::NoBytes, s(10)));
    assert_eq!(w.verdict(LIMITED, now + s(11)), Wait::Fine, "once");
    assert_eq!(w.verdict(LIMITED, now + s(599)), Wait::Fine);
    assert_eq!(w.verdict(LIMITED, now + s(600)), Wait::Fail(Lack::NoBytes, s(600)));
    // Bytes that bring no ADCO: a warning of their own, and the long bound from it.
    let mut w = Watch::new(BOUNDS, now);
    w.counted(now);
    w.heard(b'x', now + s(9));
    assert_eq!(w.verdict(LIMITED, now + s(10)), Wait::Warn(Lack::NoSample, s(10)));
    w.heard(b'x', now + s(599));
    assert_eq!(w.verdict(LIMITED, now + s(599)), Wait::Fine);
    assert_eq!(w.verdict(LIMITED, now + s(600)), Wait::Fail(Lack::NoSample, s(600)));
    // What comes back clears the warning for a next spell.
    let mut w = Watch::new(BOUNDS, now);
    w.counted(now);
    assert_eq!(w.verdict(LIMITED, now + s(10)), Wait::Warn(Lack::NoBytes, s(10)));
    w.heard(b'x', now + s(11));
    w.counted(now + s(11));
    assert_eq!(w.verdict(LIMITED, now + s(21)), Wait::Warn(Lack::NoBytes, s(10)));
}

#[test]
fn with_no_sample_limit_alone_it_warns_and_never_fails() {
    let now = Instant::now();
    let s = Duration::from_secs;
    let mut w = Watch::new(BOUNDS, now);
    assert_eq!(w.verdict(FREE, now + s(10)), Wait::Warn(Lack::NoBytes, s(10)));
    assert_eq!(w.verdict(FREE, now + s(100_000)), Wait::Fine);
    let mut w = Watch::new(BOUNDS, now);
    w.heard(b'x', now + s(9));
    assert_eq!(w.verdict(FREE, now + s(10)), Wait::Warn(Lack::NoSample, s(10)));
    let mut w = Watch::new(BOUNDS, now);
    w.heard(EOT, now);
    assert_eq!(w.verdict(FREE, now + s(10)), Wait::Warn(Lack::Interrupted, s(10)));
    assert_eq!(w.verdict(FREE, now + s(100_000)), Wait::Fine);
}

#[test]
fn an_eot_lengthens_the_wait_and_what_follows_it_starts_over() {
    let now = Instant::now();
    let s = Duration::from_secs;
    let at = |secs| now + s(secs);
    let mut w = Watch::new(BOUNDS, now);
    w.counted(at(1));
    w.heard(EOT, at(1));
    assert_eq!(w.verdict(LIMITED, at(10)), Wait::Fine);
    assert_eq!(w.verdict(LIMITED, at(11)), Wait::Warn(Lack::Interrupted, s(10)));
    assert_eq!(w.verdict(LIMITED, at(300)), Wait::Fine);
    assert_eq!(w.verdict(LIMITED, at(301)), Wait::Fail(Lack::Interrupted, s(300)));
    // It comes back after 2 minutes: the wait for a sample starts there.
    let mut w = Watch::new(BOUNDS, now);
    w.heard(EOT, at(1));
    w.heard(2, at(121));
    assert_eq!(w.verdict(LIMITED, at(122)), Wait::Fine);
    assert_eq!(w.verdict(LIMITED, at(130)), Wait::Fine);
    assert_eq!(w.verdict(LIMITED, at(131)), Wait::Fail(Lack::NoBytes, s(10)));
    // An EOT that a read holds ahead of other bytes is no interruption.
    let mut w = Watch::new(BOUNDS, now);
    w.heard(b'x', at(1));
    assert_eq!(w.verdict(LIMITED, at(11)), Wait::Fail(Lack::NoBytes, s(10)));
}

/// Groups with no `ADCO`, each read ending in an EOT: each brings the clock of a
/// return along, so the cap goes by the `ADCO` itself.
#[test]
fn a_stream_of_eots_with_no_adco_has_a_bound() {
    let now = Instant::now();
    let s = Duration::from_secs;
    let mut w = Watch::new(BOUNDS, now);
    let mut t = 0;
    while t < 299 {
        t += 1;
        w.heard(EOT, now + s(t));
        assert_eq!(w.verdict(LIMITED, now + s(t)), Wait::Fine, "{t}");
    }
    w.heard(EOT, now + s(300));
    assert_eq!(w.verdict(LIMITED, now + s(300)), Wait::Fail(Lack::InterruptedNoSample, s(300)));
    // An ADCO lifts it.
    let mut w = Watch::new(BOUNDS, now);
    w.counted(now + s(250));
    w.heard(EOT, now + s(300));
    assert_eq!(w.verdict(LIMITED, now + s(300)), Wait::Fine);
}

#[test]
fn the_excuse_moves_all_the_clocks() {
    let now = Instant::now();
    let s = Duration::from_secs;
    let mut w = Watch::new(BOUNDS, now);
    w.excuse(s(100));
    assert_eq!(w.verdict(LIMITED, now + s(109)), Wait::Fine);
    assert_eq!(w.verdict(LIMITED, now + s(110)), Wait::Fail(Lack::NoBytes, s(10)));
    let mut w = Watch::new(BOUNDS, now);
    w.excuse(s(100));
    w.heard(b'x', now + s(109));
    assert_eq!(w.verdict(LIMITED, now + s(110)), Wait::Fail(Lack::NoSample, s(10)));
}

#[test]
fn an_interruption_says_so() {
    bounded(|| {
        let line = Shared::default();
        line.queue(b"\x02\nBASE 0\x04");
        let mut d = device(Optarif::Base, &line);
        d.bounds = bounds(50, 300, 60_000);
        limit_samples(&mut d, 1);
        let before = Instant::now();
        let got = packets(&start(&mut d));
        let why = failure(&got).unwrap();
        assert_eq!(
            why,
            "device error: the meter announced an interruption (EOT) and sent nothing for 0 s"
        );
        assert!(before.elapsed() >= Duration::from_millis(300), "the long bound");
    });
}

#[test]
fn a_port_that_hangs_up_fails_the_capture() {
    bounded(|| {
        let line = Shared::default().with(|l| l.read_fails = true);
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let got = packets(&start(&mut d));
        assert!(failure(&got).is_some());
        assert_eq!(line.line().reads, 1, "one is enough");
    });
}

#[test]
fn a_stop_ends_a_capture_that_is_offered_data() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(base_frame("000012345")));
        let mut d = device(Optarif::Base, &line);
        let rx = start(&mut d);
        assert!(rx.recv_timeout(BOUND).is_ok(), "the header");
        line.wait_for_reads(2);
        d.stop().unwrap();
        assert_eq!(failure(&packets(&rx)), None);
        let reads = line.line().reads;
        d.close().unwrap();
        assert_eq!(line.line().reads, reads, "nothing after the End");
        assert_eq!(line.line().writes.len(), 0);
    });
}

#[test]
fn a_stop_between_bytes_of_a_slow_line_comes_soon() {
    bounded(|| {
        let line = Shared::default().with(|l| {
            l.per_read = 1;
            l.byte_time = Duration::from_micros(8333);
            l.stream = Some(base_frame("000012345"));
        });
        let mut d = device(Optarif::Base, &line);
        let rx = start(&mut d);
        line.wait_for_reads(2);
        d.stop().unwrap();
        let reads_then = line.line().reads;
        assert_eq!(failure(&packets(&rx)), None);
        assert!(line.line().reads <= reads_then + 1, "at most the read under way");
    });
}

#[test]
fn a_stop_gets_through_a_queue_nobody_reads() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(base_frame("000012345")));
        let mut d = device(Optarif::Base, &line);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        line.wait_for_reads(1);
        d.stop().unwrap();
        d.close().unwrap();
        drop(rx);
    });
}

#[test]
fn an_end_waits_for_room_while_nobody_asked_to_stop() {
    bounded(|| {
        let line = Shared::default();
        line.queue(&frame(&[ADCO]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The header fills the queue, and the End waits behind it, whatever the time.
        std::thread::sleep(END_AFTER_STOP + Duration::from_millis(50));
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert_eq!(rx.recv_timeout(BOUND).unwrap().1, Packet::End);
    });
}

#[test]
fn the_port_comes_back_before_the_end() {
    bounded(|| {
        let line = Shared::default();
        line.queue(&frame(&[ADCO]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        // The queue holds the header, unread, so the End cannot have been sent.
        while d.port.lock().unwrap().is_none() {
            assert!(Instant::now() < deadline, "the port did not come back");
            std::thread::sleep(Duration::from_millis(2));
        }
        drop(rx);
    });
}

#[test]
fn a_start_after_an_end_stuck_on_a_full_queue_does_not_hang_and_lets_the_old_sink_go() {
    bounded(|| {
        let line = Shared::default();
        line.queue(&frame(&[ADCO]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        let (tx, old) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        while d.port.lock().unwrap().is_none() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        let rx = start(&mut d);
        line.queue(&frame(&[ADCO]));
        assert_eq!(failure(&packets(&rx)), None);
        // The old thread gave up and dropped its sink.
        let mut got = Vec::new();
        while let Ok((_, p)) = old.recv_timeout(BOUND) {
            got.push(p);
        }
        assert_eq!(got.len(), 1, "the End gave up, and the sender is gone: {got:?}");
        assert!(matches!(got[0], Packet::Header { .. }));
    });
}

#[test]
fn a_second_capture_drops_what_came_between_and_keeps_no_head() {
    bounded(|| {
        let line = Shared::default();
        line.queue(&frame(&[("BASE", "000000001"), ADCO]));
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        assert_eq!(values(&packets(&start(&mut d)), 1), [1.0]);
        assert_eq!(line.line().purges, 1, "the open's");
        // What came since: a head of a group the second capture must not finish.
        line.queue(b"\nBASE 00000");
        let rx = {
            let (tx, rx) = sync_channel(1024);
            d.start(PacketSink::new(tx, 0)).unwrap();
            rx
        };
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        line.queue(&frame(&[("BASE", "000000002"), ADCO]));
        assert_eq!(values(&packets(&rx), 1), [2.0]);
        assert_eq!(line.line().purges, 2, "the second start's");
        // A reopen starts over.
        d.close().unwrap();
    });
}

#[test]
fn a_first_capture_keeps_what_came_after_the_open() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(Optarif::Base, &line);
        limit_samples(&mut d, 1);
        line.queue(&frame(&[("BASE", "000000001"), ADCO]));
        assert_eq!(values(&packets(&start(&mut d)), 1), [1.0]);
        assert_eq!(line.line().purges, 1, "only the open's");
    });
}

/// A close joins: by its return the thread has let the port go, and so has the
/// device, so the fake has one owner left and nothing reads or writes any more.
#[test]
fn a_close_under_a_capture_waits_for_its_thread() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(base_frame("000012345")));
        let mut d = device(Optarif::Base, &line);
        let rx = start(&mut d);
        line.wait_for_reads(2);
        d.close().unwrap();
        assert_eq!(Arc::strong_count(&line.0), 1, "someone still holds the port");
        let reads = line.line().reads;
        std::thread::sleep(READ_TIME * 2);
        assert_eq!(line.line().reads, reads, "a read after the close");
        assert_eq!(line.line().writes.len(), 0);
        drop(rx);
    });
}

#[test]
fn a_consumer_that_left_ends_the_capture() {
    bounded(|| {
        let line = Shared::default().with(|l| l.stream = Some(base_frame("000012345")));
        let mut d = device(Optarif::Base, &line);
        let rx = start(&mut d);
        drop(rx);
        d.close().unwrap();
    });
}

#[test]
fn a_start_with_no_port_is_an_error_and_a_close_gives_it_up() {
    bounded(|| {
        let line = Shared::default();
        let mut d = device(Optarif::Base, &line);
        d.close().unwrap();
        let (tx, _rx) = sync_channel(1);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
        let line = Shared::default().with(|l| l.stream = Some(base_frame("000012345")));
        let mut d = device(Optarif::Base, &line);
        let _rx = start(&mut d);
        let (tx, _rx2) = sync_channel(1);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.close().unwrap();
    });
}
