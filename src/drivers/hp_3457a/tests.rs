// Counterpart in sigrok:
//   libsigrok/src/hardware/hp-3457a/api.c,
//   libsigrok/src/hardware/hp-3457a/protocol.c,
//   Copyright (C) 2016 Alexandru Gagniuc <mr.nuke.me@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

#![allow(
    clippy::unreadable_literal,
    clippy::float_cmp,
    clippy::needless_pass_by_value,
    clippy::unnecessary_literal_bound
)]

use super::protocol::{Silence, Silent, Steps, Then, UNUSABLE_LIMIT, parse_number};
use super::*;
use crate::serial::WriteLog;
use std::collections::VecDeque;
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test waits for what must happen: a stop or a limit that does not bite
/// must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);
/// Bounds that no test of anything else should meet.
const FAR: Duration = Duration::from_secs(3600);
/// A read's timeout in tests.
const READ: Duration = Duration::from_millis(5);

fn far() -> Bounds {
    Bounds {
        first: FAR,
        warn: FAR,
        give_up: FAR,
        owed: BOUND,
        query: BOUND,
        setting: BOUND,
        tail: READ,
    }
}

/// What the meter does with a question.
#[derive(Clone, Debug)]
enum Reply {
    /// This, at once.
    Now(&'static str),
    /// This, after so many reads that found nothing.
    Late(u32, &'static str),
    /// This, once this long has passed, as a slow measurement on the clock.
    After(Duration, &'static str),
    /// These pieces, each its own read: an answer the stack cut.
    Pieces(&'static [&'static str]),
    /// These pieces, each so many empty reads after the one before.
    Spaced(u32, &'static [&'static str]),
    /// This, and this again in a read of its own: an answer too many.
    Twice(&'static str),
    /// This, once the test says so.
    OnRelease(&'static str),
    /// Nothing, ever.
    Silent,
}

#[derive(Default)]
struct Line {
    writes: WriteLog,
    readings: VecDeque<Reply>,
    /// What every further trigger gets, if the list has run out.
    otherwise: Option<Reply>,
    hires: VecDeque<Reply>,
    ranges: VecDeque<Reply>,
    plcs: VecDeque<Reply>,
    chans: VecDeque<Reply>,
    /// Pieces on their way, each with reads to wait and a time to wait for.
    coming: VecDeque<(u32, Instant, Vec<u8>)>,
    /// A trigger came with a measurement under way.
    too_fast: u32,
    gone: bool,
    /// Fails each write from this one on.
    writes_fail_at: Option<usize>,
    reading: u32,
    plc: &'static str,
    complete: bool,
    /// Writes after the connection closed.
    after_close: u32,
    /// Drains by `Scpi::send`, when an answer is taken for lost: a capture's look at the
    /// line is shorter than a read.
    discards: u32,
    /// Set as an answer is handed over.
    stop_on_delivery: Option<Arc<AtomicBool>>,
    closed: bool,
}

impl Line {
    fn queue(&mut self, reply: Reply) {
        let now = Instant::now();
        match reply {
            Reply::Now(text) => self.coming.push_back((0, now, with_end(text))),
            Reply::Late(reads, text) => self.coming.push_back((reads, now, with_end(text))),
            Reply::After(wait, text) => self.coming.push_back((0, now + wait, with_end(text))),
            Reply::Pieces(pieces) => {
                for piece in pieces {
                    self.coming.push_back((0, now, piece.as_bytes().to_vec()));
                }
            }
            Reply::Spaced(reads, pieces) => {
                for piece in pieces {
                    self.coming.push_back((reads, now, piece.as_bytes().to_vec()));
                }
            }
            Reply::Twice(text) => {
                self.coming.push_back((0, now, with_end(text)));
                self.coming.push_back((0, now, with_end(text)));
            }
            Reply::OnRelease(text) => self.coming.push_back((u32::MAX, now, with_end(text))),
            Reply::Silent => {}
        }
    }

    fn command(&mut self, text: &str) {
        match text {
            "?" | "TRIG SGL" => {
                if !self.coming.is_empty() {
                    self.too_fast += 1;
                }
                self.reading += 1;
                let reply = self.readings.pop_front().or(self.otherwise.clone());
                self.queue(reply.unwrap_or(Reply::Now(READINGS[(self.reading % 4) as usize])));
            }
            "RMATH HIRES" => {
                let reply = self.hires.pop_front().unwrap_or(Reply::Now("+5.0E-08"));
                self.queue(reply);
            }
            "RANGE?" => {
                let reply = self.ranges.pop_front().unwrap_or(Reply::Now("+3.0E+01"));
                self.queue(reply);
            }
            "CHAN?" => {
                let reply = self.chans.pop_front().unwrap_or(Reply::Now("+0.0E+00"));
                self.queue(reply);
            }
            "NPLC?" => {
                let plc = if self.plc.is_empty() { "+1.0E+00" } else { self.plc };
                let reply = self.plcs.pop_front().unwrap_or(Reply::Now(plc));
                self.queue(reply);
            }
            _ => {
                if let Some(plc) = text.strip_prefix("NPLC ") {
                    self.plc = if plc == "1E+01" { "+1.0E+01" } else { "+1.0E+00" };
                }
            }
        }
    }

    /// The commands so far, in order.
    fn sent(&self) -> Vec<String> {
        self.writes
            .iter()
            .map(|(_, bytes)| String::from_utf8_lossy(bytes).trim_end().to_owned())
            .collect()
    }
}

fn with_end(text: &str) -> Vec<u8> {
    format!("{text}\r\n").into_bytes()
}

/// A plain meter's readings, none alike, so that a shift shows.
const READINGS: [&str; 4] = ["+4.0E+00", "+1.0E+00", "+2.0E+00", "+3.0E+00"];

#[derive(Clone)]
struct Fake(Arc<Mutex<Line>>);

impl Fake {
    fn line(&self) -> std::sync::MutexGuard<'_, Line> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

impl ScpiTransport for Fake {
    fn resource(&self) -> &str {
        "fake"
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        let mut line = self.line();
        if line.closed {
            line.after_close += 1;
        }
        if line.gone || line.writes_fail_at.is_some_and(|at| line.writes.len() >= at) {
            return Err(Error::device("the write failed"));
        }
        line.writes.push((Instant::now(), data.to_vec()));
        let text = String::from_utf8_lossy(data).trim_end().to_owned();
        line.command(&text);
        Ok(())
    }

    fn read_begin(&mut self) -> Result<()> {
        self.line().complete = false;
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        {
            let mut line = self.line();
            if line.gone {
                return Err(Error::device("the instrument closed the TCP connection"));
            }
            let now = Instant::now();
            if let Some((reads, at, bytes)) = line.coming.front_mut() {
                if *reads == 0 && *at <= now {
                    let n = bytes.len().min(buf.len());
                    buf[..n].copy_from_slice(&bytes[..n]);
                    line.coming.pop_front();
                    line.complete = true;
                    if let Some(stop) = line.stop_on_delivery.take() {
                        stop.store(true, Ordering::SeqCst);
                    }
                    return Ok(n);
                }
                *reads = reads.saturating_sub(1);
            }
        }
        std::thread::sleep(timeout.min(Duration::from_millis(1)));
        Ok(0)
    }

    fn read_complete(&self) -> bool {
        self.line().complete
    }

    fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
        if timeout >= READ {
            self.line().discards += 1;
        }
        let mut chunk = [0u8; 256];
        self.read_data(&mut chunk, timeout).map(Some)
    }

    fn close(&mut self) -> Result<()> {
        self.line().closed = true;
        Ok(())
    }

    fn reopen(&mut self) -> Result<()> {
        self.line().closed = false;
        Ok(())
    }
}

fn fake() -> Fake {
    Fake(Arc::new(Mutex::new(Line::default())))
}

fn device(line: &Fake, opt: &str, bounds: Bounds) -> Hp3457aDevice {
    let mut d = Hp3457aDevice::from_transport(Box::new(line.clone()), opt);
    d.bounds = bounds;
    if let Some(scpi) = d.closed.as_mut() {
        scpi.read_timeout = READ;
    }
    d
}

/// Runs a test on a thread with a bound: a stop that does not bite must fail.
fn bounded<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    rx.recv_timeout(BOUND * 2).expect("the test did not end in time")
}

fn sink(depth: usize) -> (PacketSink, Receiver<(usize, Packet)>) {
    let (tx, rx) = sync_channel(depth);
    (PacketSink::new(tx, 0), rx)
}

/// Packets up to the End, with a bound and a cap.
fn drain(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let began = Instant::now();
    let mut out = Vec::new();
    while began.elapsed() < BOUND && out.len() < 10_000 {
        match rx.recv_timeout(BOUND) {
            Ok((_, Packet::End)) => {
                out.push(Packet::End);
                return out;
            }
            Ok((_, p)) => out.push(p),
            Err(_) => break,
        }
    }
    panic!("no End within the bound: {} packets", out.len());
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

fn failed(packets: &[Packet]) -> Option<String> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(text) => Some(text.clone()),
        _ => None,
    })
}

/// A whole capture: open, limit, start, packets, close.
fn capture(d: &mut Hp3457aDevice, limit: u64) -> Vec<Packet> {
    d.open().unwrap();
    d.set(&Key::LimitSamples, &Value::U64(limit), None).unwrap();
    let (s, rx) = sink(64);
    d.start(s).unwrap();
    let packets = drain(&rx);
    d.close().unwrap();
    packets
}

// Numbers.

#[test]
fn a_number_is_what_a_meter_means_by_one() {
    for (text, want) in [
        ("+1.234567E+00", 1.234567),
        (" 1.5", 1.5),
        ("\t-2e3", -2000.0),
        ("5.", 5.0),
        (".5", 0.5),
        ("1", 1.0),
        ("+9.9E+37", 9.9e37),
        ("-0.0E+00", 0.0),
        ("0E-999", 0.0),
    ] {
        assert_eq!(parse_number(text), Some(want), "{text:?}");
    }
    for text in [
        "",
        " ",
        "x",
        "1.5 ",
        "1.5\r",
        "0x10",
        "inf",
        "-INF",
        "infinity",
        "nan",
        "NAN",
        "1e999",
        "1e-999",
        "1,5",
        "+",
        "-",
        ".",
        "e5",
        "1e",
        "1\u{0}",
        "1.2.3",
        "+1.0E+00+",
    ] {
        assert_eq!(parse_number(text), None, "{text:?}");
    }
}

/// What the format can't catch, and how much: nothing in an answer checks its digits,
/// so most flips and losses are a wrong number. The counts go in the report.
#[test]
fn damage_to_a_reading_is_counted() {
    let frame = "+1.234567E+00";
    let good = parse_number(frame).unwrap();
    let judge = |bytes: &[u8]| match std::str::from_utf8(bytes).ok().and_then(parse_number) {
        Some(v) if v == good => "same",
        Some(_) => "wrong",
        None => "refused",
    };
    let (mut same, mut wrong, mut refused) = (0, 0, 0);
    for i in 0..frame.len() {
        for bit in 0..8 {
            let mut b = frame.as_bytes().to_vec();
            b[i] ^= 1 << bit;
            match judge(&b) {
                "same" => same += 1,
                "wrong" => wrong += 1,
                _ => refused += 1,
            }
        }
    }
    assert_eq!((same, wrong, refused), (2, 30, 72), "single bit flips of 104");
    let (mut same, mut wrong, mut refused) = (0, 0, 0);
    for i in 0..frame.len() {
        let mut b = frame.as_bytes().to_vec();
        b.remove(i);
        match judge(&b) {
            "same" => same += 1,
            "wrong" => wrong += 1,
            _ => refused += 1,
        }
    }
    assert_eq!((same, wrong, refused), (4, 8, 1), "lost bytes of 13");
}

#[test]
fn a_revision_is_two_numbers_or_nothing() {
    for (text, want) in [
        ("2.0,8.0", "2.8"),
        ("2.9,7.9,1", "2.7"),
        ("-1.5, 8.0", "-1.8"),
        ("2", "0.0"),
        ("", "0.0"),
        ("1.9,x", "0.0"),
        ("1,2,x", "0.0"),
        ("x", "0.0"),
        ("nan,inf", "0.0"),
        ("1 ,2", "0.0"),
        ("1,,2", "0.0"),
        ("3e9,1", "-2147483648.1"),
    ] {
        assert_eq!(protocol::revision(text), want, "{text:?}");
    }
}

#[test]
fn a_card_is_what_opt_names() {
    let id = |text| protocol::find_card(text).map(|c| c.id);
    assert_eq!(id("+0.0E+00"), Some(0));
    assert_eq!(id("0.5"), Some(0));
    assert_eq!(id("+4.4491E+04"), Some(44491));
    assert_eq!(id("44491.9"), Some(44491));
    assert_eq!(id("44492"), Some(44492));
    for text in ["-1", "44493", "", "x", "inf", "nan"] {
        assert_eq!(id(text), None, "{text:?}");
    }
    let name = |text| protocol::find_card(text).map(|c| (c.group, c.channels));
    assert_eq!(name("0"), Some(("rear", 1)));
    assert_eq!(name("44491"), Some(("44491a", 14)));
    assert_eq!(name("44492"), Some(("44492a", 10)));
}

#[test]
fn a_quantity_is_its_command() {
    let (ac, dc, four) = (MqFlags::AC, MqFlags::DC, MqFlags::FOUR_WIRE);
    let none = MqFlags::empty();
    for (mq, flags, command, unit) in [
        (Mq::Voltage, none, "DCV", Unit::Volt),
        (Mq::Voltage, dc, "DCV", Unit::Volt),
        (Mq::Voltage, ac, "ACV", Unit::Volt),
        (Mq::Voltage, ac | dc, "ACDCV", Unit::Volt),
        (Mq::Current, none, "DCI", Unit::Ampere),
        (Mq::Current, ac, "ACI", Unit::Ampere),
        (Mq::Current, ac | dc, "ACDCI", Unit::Ampere),
        (Mq::Resistance, none, "OHM", Unit::Ohm),
        (Mq::Resistance, four, "OHMF", Unit::Ohm),
        (Mq::Frequency, none, "FREQ", Unit::Hertz),
        (Mq::Frequency, ac, "FREQ", Unit::Hertz),
    ] {
        assert_eq!(protocol::mq_command(mq, flags), Some((command.to_owned(), unit)), "{command}");
    }
    assert_eq!(protocol::mq_command(Mq::Temperature, none), None);
    assert_eq!(protocol::mq_command(Mq::Voltage, four), None);
    assert_eq!(protocol::mq_command(Mq::Voltage, ac | four), None);
    assert_eq!(protocol::mq_command(Mq::Resistance, ac), None);
}

#[test]
fn an_integration_time_goes_out_with_one_digit() {
    for (plc, want) in [
        (10.0, "NPLC 1E+01"),
        (15.0, "NPLC 2E+01"),
        (25.0, "NPLC 2E+01"),
        (0.1, "NPLC 1E-01"),
        (0.15, "NPLC 2E-01"),
        (100.0, "NPLC 1E+02"),
        (0.000002, "NPLC 2E-06"),
        (1.0, "NPLC 1E+00"),
    ] {
        assert_eq!(protocol::plc_command(plc).as_deref(), Some(want), "{plc}");
    }
    for plc in [101.0, 0.0000001, 0.000001, f32::NAN, f32::INFINITY, -1.0] {
        assert_eq!(protocol::plc_command(plc), None, "{plc}");
    }
    assert!(!protocol::high_resolution(None));
    assert!(!protocol::high_resolution(Some(9.99)));
    assert!(protocol::high_resolution(Some(10.0)));
}

#[test]
fn digits_go_as_c_goes_on_x86() {
    for (high, value, range, want) in [
        (false, 1.234567, 30.0, 6),
        (false, 10.0, 30.0, 7),
        (false, 10.5, 30.0, 7),
        (false, 9.999999, 30.0, 6),
        (false, 0.999999, 30.0, 5),
        (true, 1.234567, 30.0, 7),
        (true, 10.5, 30.0, 8),
        (false, 0.0, 30.0, 7),
        (false, -0.0, 30.0, 7),
        // Below zero the ratio's logarithm is no number, and all the digits stay.
        (false, -9.999999, 30.0, 7),
        (false, -0.001, 30.0, 7),
        (true, -1.0e-7, 30.0, 8),
        // (int) goes towards zero, so a range under 1 has its power one too high.
        (false, 0.01, 0.03, 6),
        // A range of nothing: its logarithm is minus infinity, its power 0.
        (false, 1.0, 0.0, 7),
        (false, -1.0, 0.0, 7),
        // A range below zero is a NaN on the way to the same.
        (false, 1.0, -3.0, 7),
        // Wrapped into int8_t: 7 - 301.
        (false, 1e-300, 30.0, -38),
    ] {
        assert_eq!(protocol::digits(high, value, range), want, "{high} {value} {range}");
    }
}

// The steps of a measurement.

#[test]
fn a_sample_takes_a_reading_and_a_range() {
    let mut s = Steps::new(false, vec![-1], 2);
    assert_eq!(s.answer("+1.5E+00"), (None, Then::Ask("RANGE?")));
    let (sample, then) = s.answer("+3.0E+01");
    assert_eq!(sample.map(|x| x.value), Some(1.5));
    assert_eq!(then, Then::Ask("?"));
    s.answer("+2.5E+00");
    let (sample, then) = s.answer("+3.0E+01");
    assert_eq!(sample.map(|x| x.value), Some(2.5));
    assert_eq!(then, Then::Done, "no trigger after the last, by commit e2626373");
}

#[test]
fn high_resolution_adds_its_register() {
    let mut s = Steps::new(true, vec![-1], 0);
    assert_eq!(s.answer("+1.5E+00"), (None, Then::Ask("RMATH HIRES")));
    assert_eq!(s.answer("+5.0E-01"), (None, Then::Ask("RANGE?")));
    let (sample, then) = s.answer("+3.0E+01");
    assert_eq!(sample.map(|x| x.value), Some(2.0));
    assert_eq!(then, Then::Ask("?"));
}

#[test]
fn an_answer_that_is_no_number_starts_the_measurement_again() {
    for at in 0..3 {
        let mut s = Steps::new(true, vec![-1], 0);
        for _ in 0..at {
            assert!(matches!(s.answer("1"), (None, Then::Ask(_))));
        }
        assert_eq!(s.answer("x"), (None, Then::Ask("?")), "at step {at}");
        assert_eq!(s.answer("1"), (None, Then::Ask("RMATH HIRES")), "back at the reading");
    }
}

#[test]
fn sixteen_answers_of_nothing_fail_and_a_sample_starts_the_count_again() {
    assert_eq!(UNUSABLE_LIMIT, 16);
    let mut s = Steps::new(false, vec![-1], 0);
    for _ in 0..15 {
        assert_eq!(s.answer("x"), (None, Then::Ask("?")));
    }
    s.answer("1");
    assert!(s.answer("1").0.is_some(), "a sample");
    for _ in 0..15 {
        assert_eq!(s.answer("x"), (None, Then::Ask("?")));
    }
    assert_eq!(
        s.answer("x").1,
        Then::Fail(vec![
            "No usable answer from the meter in 16 tries; the last was 'x'.".to_owned()
        ])
    );
}

#[test]
fn a_good_reading_with_a_bad_range_does_not_start_the_count_again() {
    let mut s = Steps::new(false, vec![-1], 0);
    for _ in 0..15 {
        s.answer("1");
        assert_eq!(s.answer("x"), (None, Then::Ask("?")));
    }
    s.answer("1");
    assert!(matches!(s.answer("x").1, Then::Fail(_)));
}

#[test]
fn the_ninth_sample_of_a_scan_checks_the_channel() {
    let mut s = Steps::new(false, vec![3, 7], 0);
    let mut places = Vec::new();
    for n in 1..=19 {
        s.answer("1");
        let (sample, then) = s.answer("1");
        places.push(sample.unwrap().place);
        if n == 9 || n == 19 {
            assert_eq!(then, Then::Ask("CHAN?"), "sample {n}");
            // Odd samples leave the scan on its second channel.
            assert_eq!(s.answer("7").1, Then::Ask("?"), "sample {n}");
        } else {
            assert_eq!(then, Then::Ask("?"), "sample {n}");
        }
    }
    assert_eq!(&places[..4], [0, 1, 0, 1]);
}

#[test]
fn a_single_channel_is_never_asked_where_it_is() {
    let mut s = Steps::new(false, vec![-1], 0);
    for _ in 0..30 {
        s.answer("1");
        assert_eq!(s.answer("1").1, Then::Ask("?"));
    }
}

#[test]
fn a_channel_that_does_not_say_or_is_elsewhere_fails() {
    let at_ninth = || {
        let mut s = Steps::new(false, vec![0, 1], 0);
        for _ in 0..8 {
            s.answer("1");
            s.answer("1");
        }
        s.answer("1");
        assert_eq!(s.answer("1").1, Then::Ask("CHAN?"));
        s
    };
    let mut s = at_ninth();
    assert_eq!(
        s.answer("x").1,
        Then::Fail(vec!["Cannot check channel synchronization.".to_owned()])
    );
    // 9 samples in, the tenth channel of two is the second.
    let mut s = at_ninth();
    assert_eq!(s.answer("+1.0E+00").1, Then::Ask("?"));
    let mut s = at_ninth();
    assert_eq!(
        s.answer("+1.5E+00").1,
        Then::Fail(vec![
            "Current channel and scan advance out of sync.".to_owned(),
            "Expected channel 1, but device says 1".to_owned(),
        ])
    );
}

// Silence, on a moved clock.

fn secs(n: u64) -> Duration {
    Duration::from_secs(n)
}

/// Empty reads a second apart from `from` for `n` seconds; what the last said.
fn quiet(s: &mut Silence, from: Instant, n: u64) -> Vec<Silent> {
    (1..=n).map(|i| s.empty_read(from + secs(i), secs(1))).filter(|r| *r != Silent::Wait).collect()
}

#[test]
fn the_bounds_are_a_minute_and_ten_minutes() {
    let b = Bounds::default();
    assert_eq!(b.first, secs(60));
    assert_eq!(b.warn, secs(60));
    assert_eq!(b.give_up, secs(600));
    assert_eq!(b.owed, secs(10));
    assert_eq!(b.query, secs(30));
    assert_eq!(b.setting, secs(5));
    assert_eq!(b.tail, Duration::from_millis(50));
}

#[test]
fn with_no_sample_a_limited_capture_fails_at_the_first_bound() {
    let t = Instant::now();
    let mut s = Silence::new(Bounds::default(), true, t);
    assert_eq!(s.empty_read(t + secs(59), secs(59)), Silent::Wait);
    assert_eq!(
        s.empty_read(t + secs(60), secs(1)),
        Silent::GiveUp("No answer from the meter in 60 s.".to_owned())
    );
}

#[test]
fn after_a_sample_it_warns_once_and_goes_on_to_ten_minutes() {
    let t = Instant::now();
    let mut s = Silence::new(Bounds::default(), true, t);
    assert_eq!(s.heard(t, true), None);
    s.asked(t);
    let said = quiet(&mut s, t, 599);
    assert_eq!(
        said,
        [Silent::Warn("The meter has said nothing for 60 s; still waiting.".to_owned())]
    );
    assert_eq!(
        s.empty_read(t + secs(600), secs(1)),
        Silent::GiveUp("No answer from the meter in 600 s.".to_owned())
    );
}

#[test]
fn no_limit_warns_and_never_gives_up() {
    let t = Instant::now();
    let mut s = Silence::new(Bounds::default(), false, t);
    let said = quiet(&mut s, t, 5000);
    assert_eq!(
        said,
        [Silent::Warn("The meter has said nothing for 60 s; still waiting.".to_owned())]
    );
}

#[test]
fn an_answer_is_told_after_a_warning_and_each_question_starts_again() {
    let t = Instant::now();
    let mut s = Silence::new(Bounds::default(), true, t);
    s.heard(t, true);
    s.asked(t);
    // Across many bounds, with real gaps nowhere: a reset removed must show.
    let mut now = t;
    for _ in 0..20 {
        now += secs(590);
        assert_eq!(
            s.empty_read(now, secs(590)),
            Silent::Warn("The meter has said nothing for 60 s; still waiting.".to_owned())
        );
        assert_eq!(s.heard(now, false), Some("The meter answers again, after 590 s.".to_owned()));
        assert_eq!(s.heard(now, false), None, "once");
        s.asked(now);
    }
}

#[test]
fn time_the_thread_did_not_run_is_excused_up_to_a_limit() {
    let t = Instant::now();
    let mut s = Silence::new(Bounds::default(), true, t);
    // Stopped for 100 s between two reads a second long: 30 s of it are excused.
    assert_eq!(s.empty_read(t + secs(1), secs(1)), Silent::Wait);
    assert_eq!(s.empty_read(t + secs(89), secs(1)), Silent::Wait, "59 s of silence");
    assert!(matches!(s.empty_read(t + secs(91), secs(1)), Silent::GiveUp(_)), "over the cap");
    // A stall of the same length with the credit used up is no more silent for it.
    let mut s = Silence::new(Bounds::default(), true, t);
    assert!(matches!(s.empty_read(t + secs(60), secs(60)), Silent::GiveUp(_)), "no gap, no credit");
}

// A capture on a fake.

#[test]
fn a_capture_asks_in_the_order_c_asks() {
    let line = fake();
    let mut d = device(&line, "", far());
    let packets = bounded(move || capture(&mut d, 2));
    assert_eq!(values(&packets), [1.0, 2.0]);
    assert_eq!(failed(&packets), None);
    assert_eq!(
        line.line().sent(),
        [
            "PRESET",
            "INBUF ON",
            "TRIG HOLD",
            "NPLC?",
            "TERM FRONT",
            "TRIG SGL",
            "RANGE?",
            "?",
            "RANGE?",
            "SADV HOLD",
            "TRIG AUTO",
        ]
    );
    assert_eq!(line.line().after_close, 0, "nothing goes after the close");
}

/// The real rate: a measurement well over a read's timeout, on the clock. The port must
/// wait it out with one question, as a second trigger earns the meter's error and then
/// an answer too many.
#[test]
fn a_slow_measurement_is_waited_for_with_one_question() {
    let line = fake();
    {
        let mut l = line.line();
        l.otherwise = Some(Reply::After(READ * 6, "+7.0E+00"));
    }
    let mut d = device(&line, "", far());
    let packets = bounded(move || capture(&mut d, 3));
    assert_eq!(values(&packets), [7.0, 7.0, 7.0]);
    let l = line.line();
    assert_eq!(l.too_fast, 0, "a trigger with a measurement under way");
    assert_eq!(l.sent().iter().filter(|c| *c == "?" || *c == "TRIG SGL").count(), 3);
}

#[test]
fn a_late_answer_shifts_nothing() {
    let line = fake();
    line.line().ranges.push_back(Reply::Late(40, "+3.0E+01"));
    let mut d = device(&line, "", far());
    let packets = bounded(move || capture(&mut d, 2));
    assert_eq!(values(&packets), [1.0, 2.0]);
    assert_eq!(line.line().too_fast, 0);
    // It came, so nothing is owed: no drain ahead of the next question, which costs a
    // read's timeout per late answer and could eat the next answer.
    assert_eq!(line.line().discards, 0);
}

#[test]
fn an_answer_the_stack_cut_is_one_answer() {
    let line = fake();
    line.line().readings.push_back(Reply::Pieces(&["+1.2", "5E+00\r\n"]));
    let mut d = device(&line, "", far());
    let packets = bounded(move || capture(&mut d, 1));
    assert_eq!(values(&packets), [1.25]);
    let sent = line.line().sent();
    assert_eq!(sent.iter().filter(|c| *c == "RANGE?").count(), 1, "{sent:?}");
}

#[test]
fn an_answer_with_no_line_end_stands_after_the_tail() {
    let line = fake();
    line.line().readings.push_back(Reply::Pieces(&["+1.25E+00"]));
    let mut d = device(&line, "", far());
    d.line_ends = false;
    assert_eq!(values(&bounded(move || capture(&mut d, 1))), [1.25]);
}

#[test]
fn a_silent_meter_fails_a_limited_capture_with_no_sample() {
    let line = fake();
    line.line().otherwise = Some(Reply::Silent);
    let bounds = Bounds { first: Duration::from_millis(40), ..far() };
    let mut d = device(&line, "", bounds);
    let packets = bounded(move || capture(&mut d, 1));
    assert_eq!(values(&packets), Vec::<f32>::new());
    assert_eq!(failed(&packets).as_deref(), Some("device error: No answer from the meter in 0 s."));
    assert_eq!(line.line().sent().iter().filter(|c| *c == "?").count(), 0, "not asked again");
}

#[test]
fn a_silence_after_a_sample_goes_on_past_the_first_bound() {
    let line = fake();
    {
        let mut l = line.line();
        l.readings.push_back(Reply::Now("+1.0E+00"));
        l.readings.push_back(Reply::After(Duration::from_millis(120), "+2.0E+00"));
    }
    let bounds = Bounds { first: Duration::from_millis(30), ..far() };
    let mut d = device(&line, "", bounds);
    let packets = bounded(move || capture(&mut d, 2));
    assert_eq!(values(&packets), [1.0, 2.0]);
    assert_eq!(failed(&packets), None);
}

#[test]
fn a_silence_after_a_sample_fails_at_the_long_bound() {
    let line = fake();
    {
        let mut l = line.line();
        l.readings.push_back(Reply::Now("+1.0E+00"));
        l.readings.push_back(Reply::Silent);
    }
    let bounds = Bounds { give_up: Duration::from_millis(40), ..far() };
    let mut d = device(&line, "", bounds);
    let packets = bounded(move || capture(&mut d, 2));
    assert_eq!(values(&packets), [1.0]);
    assert!(failed(&packets).is_some());
}

#[test]
fn a_meter_with_no_sense_in_it_fails_quicker_than_silence() {
    let line = fake();
    line.line().otherwise = Some(Reply::Now("garbage"));
    let mut d = device(&line, "", far());
    let packets = bounded(move || capture(&mut d, 1));
    assert_eq!(
        failed(&packets).as_deref(),
        Some("device error: No usable answer from the meter in 16 tries; the last was 'garbage'.")
    );
    assert_eq!(line.line().reading, 16, "one measurement for each");
}

#[test]
fn a_gone_connection_fails_the_capture() {
    let line = fake();
    line.line().readings.push_back(Reply::Silent);
    let mut d = device(&line, "", far());
    let l2 = line.clone();
    let packets = bounded(move || {
        d.open().unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let began = Instant::now();
        while !l2.line().sent().iter().any(|c| c == "TRIG SGL") {
            assert!(began.elapsed() < BOUND);
            std::thread::sleep(Duration::from_millis(1));
        }
        l2.line().gone = true;
        let packets = drain(&rx);
        d.close().unwrap();
        packets
    });
    assert_eq!(
        failed(&packets).as_deref(),
        Some("device error: the instrument closed the TCP connection")
    );
}

#[test]
fn a_write_that_fails_ends_the_capture() {
    let line = fake();
    // After the open's 4, TERM FRONT, TRIG SGL and the first RANGE?.
    line.line().writes_fail_at = Some(6);
    let mut d = device(&line, "", far());
    let packets = bounded(move || capture(&mut d, 3));
    assert_eq!(failed(&packets).as_deref(), Some("device error: the write failed"));
}

#[test]
fn a_stop_ends_a_silent_capture_and_the_close_sends_its_two() {
    let line = fake();
    line.line().otherwise = Some(Reply::Silent);
    let mut d = device(&line, "", far());
    let l2 = line.clone();
    let packets = bounded(move || {
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        // The trigger is out before the stop.
        let began = Instant::now();
        while !l2.line().sent().iter().any(|c| c == "TRIG SGL") {
            assert!(began.elapsed() < BOUND);
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        let packets = drain(&rx);
        d.close().unwrap();
        packets
    });
    assert_eq!(failed(&packets), None, "a stop is no failure");
    let sent = line.line().sent();
    assert_eq!(&sent[sent.len() - 2..], ["SADV HOLD", "TRIG AUTO"]);
    assert_eq!(sent.iter().filter(|c| *c == "TRIG AUTO").count(), 1);
}

#[test]
fn a_stop_ends_a_capture_that_has_answers_coming() {
    let line = fake();
    let mut d = device(&line, "", far());
    let packets = bounded(move || {
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let (_, first) = rx.recv_timeout(BOUND).unwrap();
        assert!(matches!(first, Packet::Header { .. }));
        d.stop().unwrap();
        let packets = drain(&rx);
        d.close().unwrap();
        packets
    });
    assert_eq!(packets.last(), Some(&Packet::End));
    assert!(packets.len() < 10_000);
}

#[test]
fn a_close_with_a_full_queue_ends() {
    let line = fake();
    let mut d = device(&line, "", far());
    bounded(move || {
        d.open().unwrap();
        // Nobody reads: the header takes the queue and the first sample has no room.
        let (s, rx) = sink(1);
        d.start(s).unwrap();
        std::thread::sleep(READ);
        d.close().unwrap();
        drop(rx);
    });
}

#[test]
fn a_start_after_a_stop_waits_for_what_is_owed() {
    let line = fake();
    {
        let mut l = line.line();
        // The first capture's reading comes only after the stop: not to be taken for
        // the second capture's.
        l.readings.push_back(Reply::Late(40, "+9.0E+00"));
        l.readings.push_back(Reply::Now("+5.0E+00"));
    }
    let mut d = device(&line, "", far());
    let l2 = line.clone();
    let packets = bounded(move || {
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let began = Instant::now();
        while !l2.line().sent().iter().any(|c| c == "TRIG SGL") {
            assert!(began.elapsed() < BOUND);
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        assert_eq!(values(&drain(&rx)), Vec::<f32>::new());
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let packets = drain(&rx);
        d.close().unwrap();
        packets
    });
    assert_eq!(values(&packets), [5.0]);
    assert_eq!(line.line().too_fast, 0);
}

#[test]
fn a_setting_after_a_stop_waits_for_what_is_owed() {
    let line = fake();
    line.line().readings.push_back(Reply::Late(40, "+9.0E+00"));
    let mut d = device(&line, "", far());
    let l2 = line.clone();
    let plc = bounded(move || {
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let began = Instant::now();
        while !l2.line().sent().iter().any(|c| c == "TRIG SGL") {
            assert!(began.elapsed() < BOUND);
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        drain(&rx);
        d.set(&Key::AdcPowerlineCycles, &Value::F64(10.0), None).unwrap();
        d.get(&Key::AdcPowerlineCycles, None).unwrap()
    });
    assert_eq!(plc, Value::F64(10.0), "not the reading of 9");
}

#[test]
fn an_owed_answer_that_never_comes_is_waited_for_within_a_bound() {
    let line = fake();
    line.line().readings.push_back(Reply::Silent);
    let bounds = Bounds { owed: Duration::from_millis(20), ..far() };
    let mut d = device(&line, "", bounds);
    let l2 = line.clone();
    bounded(move || {
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let began = Instant::now();
        while !l2.line().sent().iter().any(|c| c == "TRIG SGL") {
            assert!(began.elapsed() < BOUND);
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        drain(&rx);
        d.set(&Key::AdcPowerlineCycles, &Value::F64(10.0), None).unwrap();
    });
}

#[test]
fn a_setting_during_a_capture_is_refused() {
    let line = fake();
    line.line().otherwise = Some(Reply::Silent);
    let mut d = device(&line, "", far());
    bounded(move || {
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let v = Value::F64(10.0);
        assert!(matches!(d.set(&Key::AdcPowerlineCycles, &v, None), Err(Error::Busy)));
        let (s2, _rx2) = sink(4);
        assert!(matches!(d.start(s2), Err(Error::Busy)));
        d.stop().unwrap();
        drain(&rx);
        d.close().unwrap();
    });
}

// Terminals and cards.

fn enable(d: &mut Hp3457aDevice, on: &[usize]) {
    for pos in 0..d.channels.len() {
        d.enable_channel(pos, on.contains(&pos)).unwrap();
    }
}

#[test]
fn channels_are_as_c_names_them() {
    let line = fake();
    let names = |d: &Hp3457aDevice| {
        d.channels.iter().map(|c| (c.name.clone(), c.enabled)).collect::<Vec<_>>()
    };
    let d = device(&line, "x", far());
    assert_eq!(names(&d), [("Front".to_owned(), true)]);
    let d = device(&line, "0", far());
    assert_eq!(names(&d), [("Front".to_owned(), true), ("rear".to_owned(), false)]);
    assert_eq!(d.groups.iter().map(|g| g.name.as_str()).collect::<Vec<_>>(), ["Front", "rear"]);
    let d = device(&line, "44492", far());
    assert_eq!(d.channels.len(), 11);
    assert_eq!(d.channels[10].name, "44492a9");
    assert_eq!(d.groups[1].channels.len(), 10);
}

#[test]
fn rear_terminals_get_no_scan_list() {
    let line = fake();
    let mut d = device(&line, "0", far());
    enable(&mut d, &[1]);
    let packets = bounded(move || capture(&mut d, 1));
    assert_eq!(values(&packets).len(), 1);
    let sent = line.line().sent();
    assert!(sent.contains(&"TERM REAR".to_owned()), "{sent:?}");
    assert!(!sent.iter().any(|c| c.starts_with("SLIST") || c == "SADV AUTO"), "{sent:?}");
}

#[test]
fn a_card_gets_its_scan_list_with_the_leading_comma() {
    let line = fake();
    let mut d = device(&line, "44491", far());
    enable(&mut d, &[2, 8]);
    let packets = bounded(move || capture(&mut d, 3));
    let channels: Vec<usize> = packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some(a.channels[0]),
            _ => None,
        })
        .collect();
    assert_eq!(channels, [2, 8, 2]);
    let sent = line.line().sent();
    let at = sent.iter().position(|c| c == "TERM REAR").unwrap();
    assert_eq!(sent[at..at + 4], ["TERM REAR", "SLIST ,1,7", "SADV AUTO", "TRIG SGL"]);
}

#[test]
fn front_with_rear_or_nothing_is_refused_and_sends_nothing() {
    let line = fake();
    let mut d = device(&line, "44491", far());
    d.open().unwrap();
    let before = line.line().sent().len();
    enable(&mut d, &[0, 3]);
    let refused = d.check_capture(None).unwrap_err().to_string();
    assert_eq!(refused, "invalid argument: Can not use front and rear channels at the same time!");
    let (s, _rx) = sink(4);
    assert!(d.start(s).is_err());
    enable(&mut d, &[]);
    let refused = d.check_capture(None).unwrap_err().to_string();
    assert_eq!(refused, "invalid argument: Need at least one active channel!");
    assert_eq!(line.line().sent().len(), before);
}

#[test]
fn a_reopen_forgets_what_a_preset_may_change() {
    let line = fake();
    let mut d = device(&line, "0", far());
    enable(&mut d, &[1]);
    let l2 = line.clone();
    bounded(move || {
        d.open().unwrap();
        d.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Current, MqFlags::empty()), None).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        drain(&rx);
        d.close().unwrap();
        d.open().unwrap();
        d.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Current, MqFlags::empty()), None).unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        drain(&rx);
        d.close().unwrap();
        let sent = l2.line().sent();
        assert_eq!(sent.iter().filter(|c| *c == "DCI").count(), 2, "{sent:?}");
        assert_eq!(sent.iter().filter(|c| *c == "TERM REAR").count(), 2, "{sent:?}");
    });
}

// Settings.

#[test]
fn a_quantity_goes_out_only_if_it_changes_with_its_flags() {
    let line = fake();
    let mut d = device(&line, "", far());
    d.open().unwrap();
    let mut mq = |m, f| d.set(&Key::MeasuredQuantity, &Value::Mq(m, f), None);
    mq(Mq::Voltage, MqFlags::empty()).unwrap();
    mq(Mq::Voltage, MqFlags::empty()).unwrap();
    mq(Mq::Voltage, MqFlags::AC).unwrap();
    mq(Mq::Current, MqFlags::AC).unwrap();
    assert!(matches!(mq(Mq::Temperature, MqFlags::empty()), Err(Error::NotSupported(_))));
    assert!(matches!(mq(Mq::Voltage, MqFlags::FOUR_WIRE), Err(Error::NotSupported(_))));
    let sent = line.line().sent();
    assert_eq!(sent[4..], ["DCV", "ACV", "ACI"]);
}

#[test]
fn a_samples_label_is_the_last_quantity_set() {
    let line = fake();
    let mut d = device(&line, "", far());
    let packets = bounded(move || {
        d.open().unwrap();
        d.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Voltage, MqFlags::empty()), None).unwrap();
        d.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Voltage, MqFlags::AC), None).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let packets = drain(&rx);
        d.close().unwrap();
        packets
    });
    let Some(Packet::Analog(a)) = packets.iter().find(|p| matches!(p, Packet::Analog(_))) else {
        panic!("no sample");
    };
    assert_eq!((a.mq, a.flags, a.unit), (Mq::Voltage, MqFlags::AC, Unit::Volt));
}

#[test]
fn no_quantity_is_no_unit() {
    let line = fake();
    let mut d = device(&line, "", far());
    let packets = bounded(move || capture(&mut d, 1));
    let Some(Packet::Analog(a)) = packets.iter().find(|p| matches!(p, Packet::Analog(_))) else {
        panic!("no sample");
    };
    assert_eq!((a.flags, a.unit), (MqFlags::empty(), Unit::Unitless));
}

#[test]
fn a_time_reads_back_what_the_meter_chose() {
    let line = fake();
    let mut d = device(&line, "", far());
    d.open().unwrap();
    assert_eq!(d.get(&Key::AdcPowerlineCycles, None).unwrap(), Value::F64(1.0));
    d.set(&Key::AdcPowerlineCycles, &Value::F64(10.0), None).unwrap();
    assert_eq!(d.get(&Key::AdcPowerlineCycles, None).unwrap(), Value::F64(10.0));
    let sent = line.line().sent();
    assert_eq!(sent[4..], ["NPLC 1E+01", "NPLC?"]);
}

#[test]
fn a_time_refused_sends_nothing_and_keeps_what_was() {
    let line = fake();
    let mut d = device(&line, "", far());
    d.open().unwrap();
    for bad in [101.0, 0.000001, f64::NAN, -1.0] {
        let e = d.set(&Key::AdcPowerlineCycles, &Value::F64(bad), None).unwrap_err();
        assert!(matches!(e, Error::Arg(_)), "{bad}");
    }
    assert_eq!(line.line().sent().len(), 4);
    assert_eq!(d.get(&Key::AdcPowerlineCycles, None).unwrap(), Value::F64(1.0));
}

#[test]
fn a_time_that_does_not_read_back_is_not_known_and_says_so() {
    let line = fake();
    {
        let mut l = line.line();
        l.plcs.push_back(Reply::Now("+1.0E+00"));
        l.plcs.push_back(Reply::Now("x"));
    }
    let mut d = device(&line, "", far());
    d.open().unwrap();
    let e = d.set(&Key::AdcPowerlineCycles, &Value::F64(10.0), None).unwrap_err().to_string();
    assert_eq!(e, "device error: NPLC? says 'x'; the setting may or may not have been taken");
    let e = d.get(&Key::AdcPowerlineCycles, None).unwrap_err().to_string();
    assert_eq!(e, "device error: the meter did not say its integration time");
    // The next that reads back puts it right.
    d.set(&Key::AdcPowerlineCycles, &Value::F64(10.0), None).unwrap();
    assert_eq!(d.get(&Key::AdcPowerlineCycles, None).unwrap(), Value::F64(10.0));
}

#[test]
fn a_time_with_no_answer_is_not_known_either() {
    let line = fake();
    {
        let mut l = line.line();
        l.plcs.push_back(Reply::Now("+1.0E+00"));
        l.plcs.push_back(Reply::Silent);
    }
    let bounds = Bounds { setting: Duration::from_millis(20), ..far() };
    let mut d = device(&line, "", bounds);
    let done = bounded(move || {
        d.open().unwrap();
        let e = d.set(&Key::AdcPowerlineCycles, &Value::F64(10.0), None).unwrap_err();
        (matches!(e, Error::Timeout), d.get(&Key::AdcPowerlineCycles, None).is_err())
    });
    assert_eq!(done, (true, true));
}

#[test]
fn a_time_not_known_at_an_open_gets_no_high_resolution() {
    let line = fake();
    line.line().plcs.push_back(Reply::Now(""));
    let mut d = device(&line, "", far());
    let packets = bounded(move || capture(&mut d, 1));
    assert_eq!(values(&packets).len(), 1);
    assert!(!line.line().sent().contains(&"RMATH HIRES".to_owned()));
}

#[test]
fn a_silent_time_at_an_open_is_not_known_and_opens() {
    let line = fake();
    line.line().plcs.push_back(Reply::Silent);
    let bounds = Bounds { query: Duration::from_millis(20), ..far() };
    let mut d = device(&line, "", bounds);
    bounded(move || {
        d.open().unwrap();
        assert!(d.get(&Key::AdcPowerlineCycles, None).is_err());
    });
}

#[test]
fn high_resolution_asks_its_register_from_ten_plc() {
    let line = fake();
    line.line().plc = "+1.0E+01";
    let mut d = device(&line, "", far());
    let packets = bounded(move || capture(&mut d, 1));
    assert_eq!(values(&packets), [1.00000005]);
    assert!(line.line().sent().contains(&"RMATH HIRES".to_owned()));
}

#[test]
fn an_open_whose_writes_fail_is_refused_and_can_be_tried_again() {
    let line = fake();
    line.line().gone = true;
    let mut d = device(&line, "", far());
    assert!(d.open().is_err());
    line.line().gone = false;
    d.open().unwrap();
    assert_eq!(d.get(&Key::AdcPowerlineCycles, None).unwrap(), Value::F64(1.0));
}

#[test]
fn a_set_or_start_on_a_closed_device_is_refused() {
    let line = fake();
    let mut d = device(&line, "", far());
    assert!(d.set(&Key::AdcPowerlineCycles, &Value::F64(10.0), None).is_err());
    let (s, _rx) = sink(4);
    assert!(d.start(s).is_err());
    assert_eq!(line.line().sent().len(), 0);
}

#[test]
fn keys_no_table_has_are_refused() {
    let line = fake();
    let mut d = device(&line, "", far());
    assert!(matches!(d.get(&Key::LimitSamples, None), Err(Error::UnknownKey(_))));
    assert!(matches!(d.set(&Key::LimitMsec, &Value::U64(1), None), Err(Error::UnknownKey(_))));
    assert!(matches!(d.list(&Key::MeasuredQuantity, None), Err(Error::UnknownKey(_))));
}

// The scan.

/// The probe's questions, from a scripted line.
fn probed(id: Reply, rev: Reply, opt: Reply) -> (Option<Hp3457aDevice>, Vec<String>) {
    probed_within(Duration::from_millis(300), id, rev, opt)
}

/// [`probed`] with a bound of `query` on each question.
fn probed_within(
    query: Duration,
    id: Reply,
    rev: Reply,
    opt: Reply,
) -> (Option<Hp3457aDevice>, Vec<String>) {
    struct Probe(Fake, Reply, Reply, Reply);
    impl ScpiTransport for Probe {
        fn resource(&self) -> &str {
            "fake"
        }
        fn send(&mut self, data: &[u8]) -> Result<()> {
            let text = String::from_utf8_lossy(data).trim_end().to_owned();
            let mut line = self.0.line();
            line.writes.push((Instant::now(), data.to_vec()));
            match text.as_str() {
                "ID?" => line.queue(self.3.clone()),
                "REV?" => line.queue(self.1.clone()),
                "OPT?" => line.queue(self.2.clone()),
                _ => {}
            }
            Ok(())
        }
        fn read_begin(&mut self) -> Result<()> {
            self.0.read_begin()
        }
        fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            self.0.read_data(buf, timeout)
        }
        fn read_complete(&self) -> bool {
            self.0.read_complete()
        }
        fn close(&mut self) -> Result<()> {
            self.0.close()
        }
    }
    let line = fake();
    let mut scpi = Scpi::new(Box::new(Probe(line.clone(), rev, opt, id)));
    scpi.read_timeout = READ;
    let bounds = Bounds { query, ..far() };
    let device = probe(scpi, &bounds);
    let sent = line.line().sent();
    (device, sent)
}

#[test]
fn a_scan_takes_what_says_it_is_a_3457a() {
    let (d, sent) = probed(Reply::Now("HP3457A"), Reply::Now("2.0,8.0"), Reply::Now("44491"));
    let d = d.expect("a meter");
    assert_eq!(sent, ["END ALWAYS", "ID?", "REV?", "OPT?"]);
    assert_eq!(d.info.version, "2.8");
    assert_eq!(d.info.vendor, "Hewlett-Packard");
    assert_eq!(d.info.model, "3457A");
    assert_eq!(d.channels.len(), 15);
    for id in ["HP3478A", "HP3457", "hp3457a", ""] {
        assert!(probed(Reply::Now(id), Reply::Now("2,8"), Reply::Now("0")).0.is_none(), "{id:?}");
    }
}

#[test]
fn a_scan_outlives_a_silent_rev_and_opt() {
    let (d, sent) = probed(Reply::Now("HP3457A"), Reply::Silent, Reply::Now("44491"));
    let d = d.expect("a meter");
    assert_eq!(d.info.version, "0.0");
    assert_eq!(d.channels.len(), 1);
    // A late REV? would be read as this one's answer.
    assert_eq!(sent, ["END ALWAYS", "ID?", "REV?"]);
    let (d, _) = probed(Reply::Now("HP3457A"), Reply::Now("2,8"), Reply::Silent);
    assert_eq!(d.expect("a meter").channels.len(), 1);
}

#[test]
fn id_shows_whether_the_line_ends_its_answers() {
    // Released by reads, not by the clock, so that a stall or a slow sleep can't
    // change what a test sees: a bound of BOUND is never met here.
    let ends =
        |id| probed_within(BOUND, id, Reply::Now("2,8"), Reply::Now("-1")).0.map(|d| d.line_ends);
    assert_eq!(ends(Reply::Now("HP3457A")), Some(true));
    assert_eq!(ends(Reply::Pieces(&["HP3457A"])), Some(false));
    // Part of the id waits for the rest, past any read's timeout.
    assert_eq!(ends(Reply::Spaced(40, &["HP34", "57A\r\n"])), Some(true));
    // The whole id gets one read's timeout for its end: an end that follows at once
    // makes a line that ends its answers; one that never comes, a line with none.
    assert_eq!(ends(Reply::Pieces(&["HP3457A", "\r\n"])), Some(true));
    assert_eq!(ends(Reply::Pieces(&["HP3457A\r", "\n"])), Some(true));
    let mut late_end =
        probed_within(BOUND, Reply::Pieces(&["HP3457A"]), Reply::Now("2,8"), Reply::Now("-1"));
    assert_eq!(late_end.0.take().map(|d| d.line_ends), Some(false));
    assert_eq!(ends(Reply::Pieces(&["XX"])), None);
    assert!(probed(Reply::Silent, Reply::Now("2,8"), Reply::Now("-1")).0.is_none());
}

// An answer in pieces, on a line that ends its answers.

#[test]
fn an_answer_cut_for_longer_than_a_read_is_one_answer() {
    let line = fake();
    {
        let mut l = line.line();
        // CR and LF apart, and cut 5 bytes in, on a reading and on a range.
        l.readings.push_back(Reply::Spaced(30, &["+1.5E+00\r", "\n"]));
        l.readings.push_back(Reply::Spaced(30, &["+2.5E", "+00\r\n"]));
        l.ranges.push_back(Reply::Spaced(30, &["+3.0E+01\r", "\n"]));
        l.ranges.push_back(Reply::Spaced(30, &["+3.0E", "+01\r\n"]));
    }
    let mut d = device(&line, "", far());
    let packets = bounded(move || capture(&mut d, 2));
    assert_eq!(values(&packets), [1.5, 2.5]);
    assert_eq!(failed(&packets), None);
    let digits: Vec<i32> = packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some(a.digits),
            _ => None,
        })
        .collect();
    assert_eq!(digits, [6, 6], "each range was read as a range of 30");
    let l = line.line();
    assert_eq!(l.sent().iter().filter(|c| *c == "RANGE?").count(), 2);
    assert_eq!(l.sent().iter().filter(|c| *c == "?").count(), 1, "no measurement again");
}

#[test]
fn a_fragment_that_never_ends_is_silence_and_not_an_answer() {
    let line = fake();
    line.line().readings.push_back(Reply::Pieces(&["+1.2"]));
    let bounds = Bounds { first: Duration::from_millis(40), ..far() };
    let mut d = device(&line, "", bounds);
    let packets = bounded(move || capture(&mut d, 1));
    assert_eq!(values(&packets), Vec::<f32>::new());
    assert!(failed(&packets).is_some_and(|f| f.contains("No answer from the meter")));
    assert!(!line.line().sent().contains(&"RANGE?".to_owned()));
}

#[test]
fn more_than_any_answer_with_no_line_end_makes_nothing() {
    const LONG: &str = concat!(
        "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",
        "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",
        "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",
    );
    assert!(LONG.len() > protocol::LONGEST_ANSWER);
    let line = fake();
    line.line().readings.push_back(Reply::Pieces(&[LONG]));
    let mut d = device(&line, "", far());
    assert_eq!(values(&bounded(move || capture(&mut d, 1))), [2.0]);
}

#[test]
fn two_lines_to_one_question_are_no_answer() {
    let line = fake();
    line.line().readings.push_back(Reply::Now("+9.0E+00\r\n+9.0E+00"));
    let mut d = device(&line, "", far());
    assert_eq!(values(&bounded(move || capture(&mut d, 1))), [2.0]);
}

// What comes with nothing asked.

#[test]
fn an_extra_answer_to_a_reading_loses_that_measurement_and_no_more() {
    let line = fake();
    line.line().readings.push_back(Reply::Twice("+9.0E+00"));
    let mut d = device(&line, "", far());
    assert_eq!(values(&bounded(move || capture(&mut d, 2))), [2.0, 3.0]);
    let l = line.line();
    assert_eq!(l.sent().iter().filter(|c| *c == "RANGE?").count(), 2, "none for the first");
}

#[test]
fn an_extra_answer_to_a_range_does_not_shift_what_follows() {
    let line = fake();
    line.line().ranges.push_back(Reply::Twice("+3.0E+01"));
    let mut d = device(&line, "", far());
    assert_eq!(values(&bounded(move || capture(&mut d, 3))), [1.0, 2.0, 3.0]);
}

#[test]
fn what_lies_on_the_line_ahead_of_a_capture_is_not_its_first_reading() {
    let line = fake();
    let mut d = device(&line, "", far());
    let packets = bounded({
        let line = line.clone();
        move || {
            d.open().unwrap();
            line.line().queue(Reply::Now("+9.0E+00"));
            d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
            let (s, rx) = sink(64);
            d.start(s).unwrap();
            let packets = drain(&rx);
            d.close().unwrap();
            packets
        }
    });
    assert_eq!(values(&packets), [1.0]);
}

#[test]
fn strays_count_as_answers_that_make_nothing() {
    let mut s = Steps::new(false, vec![0], 0);
    for _ in 0..15 {
        assert_eq!(s.stray(3), Then::Ask("?"));
    }
    assert_eq!(
        s.stray(3),
        Then::Fail(vec![
            "No usable answer from the meter in 16 tries; the last was '3 bytes unasked'."
                .to_owned()
        ])
    );
}

// What is owed.

/// Runs up to `command` going out unanswered, then stops, and hands back the device.
fn stopped_at(line: &Fake, command: &str, nth: usize, limit: u64) -> Hp3457aDevice {
    let mut d = device(line, "", far());
    d.open().unwrap();
    d.set(&Key::LimitSamples, &Value::U64(limit), None).unwrap();
    let (s, rx) = sink(64);
    d.start(s).unwrap();
    let began = Instant::now();
    while line.line().sent().iter().filter(|c| *c == command).count() < nth {
        assert!(began.elapsed() < BOUND, "no {command}");
        std::thread::sleep(Duration::from_millis(1));
    }
    d.stop().unwrap();
    drain(&rx);
    d.end_worker();
    d
}

#[test]
fn each_question_a_stop_leaves_unanswered_is_owed() {
    for (command, nth, plc) in
        [("TRIG SGL", 1, ""), ("?", 1, ""), ("RANGE?", 2, ""), ("RMATH HIRES", 1, "+1.0E+01")]
    {
        let line = fake();
        {
            let mut l = line.line();
            l.plc = plc;
            match command {
                "TRIG SGL" => l.readings.push_back(Reply::Silent),
                "?" => {
                    l.readings.push_back(Reply::Now("+1.0E+00"));
                    l.readings.push_back(Reply::Silent);
                }
                "RANGE?" => {
                    l.ranges.push_back(Reply::Now("+3.0E+01"));
                    l.ranges.push_back(Reply::Silent);
                }
                _ => l.hires.push_back(Reply::Silent),
            }
        }
        let cmd = command.to_owned();
        let l2 = line.clone();
        let owed = bounded(move || stopped_at(&l2, &cmd, nth, 0).owed);
        assert!(owed, "{command}");
    }
}

#[test]
fn nothing_is_owed_once_a_limit_is_met() {
    let line = fake();
    let mut d = device(&line, "", far());
    let owed = bounded(move || {
        d.open().unwrap();
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        drain(&rx);
        d.end_worker();
        d.owed
    });
    assert!(!owed);
}

#[test]
fn a_stop_with_an_answer_in_hand_asks_nothing_more() {
    let line = fake();
    line.line().readings.push_back(Reply::OnRelease("+1.0E+00"));
    let mut d = device(&line, "", far());
    let l2 = line.clone();
    let packets = bounded(move || {
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let began = Instant::now();
        while !l2.line().sent().iter().any(|c| c == "TRIG SGL") {
            assert!(began.elapsed() < BOUND);
            std::thread::sleep(Duration::from_millis(1));
        }
        {
            let mut l = l2.line();
            l.stop_on_delivery = Some(Arc::clone(&d.worker.as_ref().unwrap().stop));
            l.coming.front_mut().unwrap().0 = 0;
        }
        let packets = drain(&rx);
        d.end_worker();
        assert!(!d.owed, "the answer came");
        packets
    });
    assert_eq!(values(&packets), Vec::<f32>::new());
    assert!(!line.line().sent().contains(&"RANGE?".to_owned()));
}

#[test]
fn a_question_an_open_gave_up_on_is_waited_for() {
    let line = fake();
    line.line().plcs.push_back(Reply::Late(200, "+7.0E+00"));
    let bounds = Bounds { query: Duration::from_millis(20), ..far() };
    let mut d = device(&line, "", bounds);
    let packets = bounded(move || {
        d.open().unwrap();
        assert!(d.owed);
        assert!(d.get(&Key::AdcPowerlineCycles, None).is_err());
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let packets = drain(&rx);
        d.close().unwrap();
        packets
    });
    assert_eq!(values(&packets), [1.0, 2.0], "not the 7 of NPLC?");
}

#[test]
fn a_question_a_setting_gave_up_on_is_waited_for() {
    let line = fake();
    {
        let mut l = line.line();
        l.plcs.push_back(Reply::Now("+1.0E+00"));
        l.plcs.push_back(Reply::Late(200, "+7.0E+00"));
    }
    let bounds = Bounds { setting: Duration::from_millis(20), ..far() };
    let mut d = device(&line, "", bounds);
    let plc = bounded(move || {
        d.open().unwrap();
        let v = Value::F64(10.0);
        assert!(matches!(d.set(&Key::AdcPowerlineCycles, &v, None), Err(Error::Timeout)));
        assert!(d.owed);
        // A second try would read the first one's answer, were it not waited out.
        let again = d.set(&Key::AdcPowerlineCycles, &v, None);
        (again.is_ok(), d.get(&Key::AdcPowerlineCycles, None).ok())
    });
    assert_eq!(plc, (true, Some(Value::F64(10.0))));
}

/// Past `owed` nothing says the answer is coming. It lands where it lands, and the look
/// finds what it has displaced: no value of the late answer's goes out here, as it comes
/// ahead of the reading, and in any case one sample at most would.
#[test]
fn an_answer_later_than_settling_is_caught_by_the_look() {
    let line = fake();
    line.line().plcs.push_back(Reply::Late(300, "+7.0E+00"));
    let bounds =
        Bounds { query: Duration::from_millis(20), owed: Duration::from_millis(20), ..far() };
    let mut d = device(&line, "", bounds);
    let packets = bounded(move || capture(&mut d, 2));
    let got = values(&packets);
    assert_eq!(got.len(), 2);
    assert!(!got.contains(&7.0), "{got:?}");
    assert_eq!(failed(&packets), None);
    assert!(got[0] < got[1], "in step: {got:?}");
}

// Terminals.

#[test]
fn the_terminals_are_selected_once_after_each_open() {
    let line = fake();
    let mut d = device(&line, "", far());
    let l2 = line.clone();
    bounded(move || {
        for _ in 0..2 {
            d.open().unwrap();
            for _ in 0..2 {
                d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
                let (s, rx) = sink(64);
                d.start(s).unwrap();
                drain(&rx);
            }
            d.close().unwrap();
        }
    });
    let sent = l2.line().sent();
    assert_eq!(sent.iter().filter(|c| *c == "TERM FRONT").count(), 2, "{sent:?}");
}

#[test]
fn a_limit_set_during_a_capture_is_refused() {
    let line = fake();
    line.line().otherwise = Some(Reply::Silent);
    let mut d = device(&line, "", far());
    bounded(move || {
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        assert!(matches!(d.set(&Key::LimitSamples, &Value::U64(1), None), Err(Error::Busy)));
        d.stop().unwrap();
        drain(&rx);
        d.close().unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    });
}

#[test]
fn nothing_is_probed_with_no_conn() {
    assert_eq!(Hp3457aDriver.scan(&[]).unwrap().len(), 0);
}
