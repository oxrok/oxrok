// Counterpart in sigrok:
//   libsigrok/src/hardware/fluke-45/api.c,
//   libsigrok/src/hardware/fluke-45/protocol.c,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2017 John Chajecki <subs@qcontinuum.plus.com>,
//   with commits by Gerhard Sittig, JohnCh
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

use super::*;
use crate::core::{Mq, MqFlags, Unit};
use crate::scpi::serial::Serial;
use crate::serial::{Params, Port, WriteLog};
use std::collections::{HashMap, VecDeque};
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test waits for what must happen: a stop or a limit that does not
/// bite must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);
/// Bounds that no test of anything else should meet.
const FAR: Duration = Duration::from_secs(3600);
const QUICK: Timing = Timing { no_answer: FAR, no_usable: FAR, give_up: FAR };
/// A byte at 9600 8n1.
const LINE_RATE: Duration = Duration::from_micros(1042);

/// What a query gets, for the Nth.
#[derive(Clone, Debug)]
enum Say {
    /// These bytes.
    Bytes(Vec<u8>),
    /// Nothing.
    Silent,
    /// These bytes, when the test says so.
    OnRelease(Vec<u8>),
    /// These bytes, across a stall in the next read, which then finds nothing.
    AcrossAStall(Vec<u8>, Duration),
}

#[derive(Clone, Copy, Debug, Default)]
enum Spacing {
    /// All at once.
    #[default]
    None,
    /// A byte every this long.
    Line(Duration),
    /// So many bytes every 50 ms: a cable's latency timer.
    Burst(usize),
}

#[derive(Default)]
struct State {
    writes: WriteLog,
    /// What the Nth query gets, from 1, if not the meter's own.
    said: HashMap<usize, Say>,
    func: Vec<u8>,
    func2: Option<Vec<u8>>,
    prompt: bool,
    /// The readings, in turn and coming round.
    values: Vec<&'static str>,
    values2: Vec<&'static str>,
    next: usize,
    /// Bytes on their way, with when each is there.
    input: VecDeque<(Instant, u8)>,
    held: Vec<u8>,
    across_a_stall: Option<(Vec<u8>, Duration)>,
    spacing: Spacing,
    fail_writes_after: Option<usize>,
    fail_reads: bool,
    /// Reads never run out of this.
    endless: Option<u8>,
    /// Writes that came with something still on its way to the reader.
    early: usize,
    purges: usize,
}

impl State {
    fn arrive(&mut self, bytes: &[u8]) {
        let now = Instant::now();
        let mut last = self.input.back().map_or(now, |(at, _)| *at).max(now);
        for (i, b) in bytes.iter().enumerate() {
            match self.spacing {
                Spacing::None => {}
                Spacing::Line(per) => last += per,
                Spacing::Burst(size) => {
                    if (self.input.len() + i).is_multiple_of(size) {
                        last += Duration::from_millis(50);
                    }
                }
            }
            self.input.push_back((last, *b));
        }
    }

    fn own_answer(&mut self, command: &str) -> Vec<u8> {
        let mut text = match command {
            "*IDN?\n" => b"FLUKE, 45, 7654321, 1.6 D1.6".to_vec(),
            "FUNC1?\n" => self.func.clone(),
            "FUNC2?\n" => self.func2.clone().unwrap_or_else(|| b"!>".to_vec()),
            "AUTO?\n" => b"1".to_vec(),
            "MOD?\n" => b"0".to_vec(),
            "VAL1?\n" => {
                let value = self.values[self.next % self.values.len()];
                self.next += 1;
                value.as_bytes().to_vec()
            }
            "VAL2?\n" if self.func2.is_some() => self.values2[0].as_bytes().to_vec(),
            "VAL2?\n" => b"!>".to_vec(),
            other => panic!("the driver sent {other:?}"),
        };
        if text == b"!>" || !self.prompt {
            text.extend_from_slice(b"\r\n");
        } else {
            text.extend_from_slice(b"\r\n=>\r\n");
        }
        text
    }
}

/// A meter.
#[derive(Clone)]
struct Fake(Arc<Mutex<State>>);

impl Fake {
    fn new() -> Fake {
        Fake(Arc::new(Mutex::new(State {
            func: b"VDC".to_vec(),
            prompt: true,
            values: vec!["+1.0000E+0", "+2.0000E+0", "+3.0000E+0", "+4.0000E+0"],
            values2: vec!["+6.0000E+1"],
            ..State::default()
        })))
    }

    fn with<T>(&self, f: impl FnOnce(&mut State) -> T) -> T {
        f(&mut self.0.lock().unwrap())
    }

    fn say(&self, nth: usize, say: Say) {
        self.with(|s| s.said.insert(nth, say));
    }

    fn say_text(&self, nth: usize, text: &str) {
        self.say(nth, Say::Bytes(format!("{text}\r\n=>\r\n").into_bytes()));
    }

    fn say_only(&self, nth: usize, text: &str) {
        self.say(nth, Say::Bytes(format!("{text}\r\n").into_bytes()));
    }

    fn release(&self) {
        self.with(|s| {
            let held = std::mem::take(&mut s.held);
            s.arrive(&held);
        });
    }

    fn writes(&self) -> Vec<String> {
        self.with(|s| {
            s.writes.iter().map(|(_, w)| String::from_utf8_lossy(w).trim_end().to_owned()).collect()
        })
    }

    fn wait_for_writes(&self, n: usize) {
        let deadline = Instant::now() + BOUND;
        while self.with(|s| s.writes.len()) < n {
            assert!(Instant::now() < deadline, "no {n} writes in time");
            std::thread::sleep(Duration::from_millis(1));
        }
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
        if let Some((bytes, stall)) = self.with(|s| s.across_a_stall.take()) {
            std::thread::sleep(stall);
            self.with(|s| s.arrive(&bytes));
            return Ok(0);
        }
        let got = {
            let mut s = self.0.lock().unwrap();
            if s.fail_reads {
                return Err(Error::device("hung up"));
            }
            if let Some(byte) = s.endless {
                buf.fill(byte);
                return Ok(buf.len());
            }
            let now = Instant::now();
            let n = s.input.iter().take_while(|(at, _)| *at <= now).count().min(buf.len());
            for b in &mut buf[..n] {
                *b = s.input.pop_front().expect("counted").1;
            }
            n
        };
        if got == 0 {
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
        }
        Ok(got)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut s = self.0.lock().unwrap();
        if s.fail_writes_after.is_some_and(|n| s.writes.len() >= n) {
            return Err(Error::device("Broken pipe"));
        }
        if !s.input.is_empty() {
            s.early += 1;
        }
        s.writes.push((Instant::now(), data.to_vec()));
        let nth = s.writes.len();
        let command = String::from_utf8_lossy(data).into_owned();
        // A reading is taken whatever is said for it.
        let own = s.own_answer(&command);
        // A meter keeps its order: what is held back holds back what follows.
        if !s.held.is_empty() {
            let bytes = match s.said.remove(&nth) {
                Some(Say::Bytes(bytes)) => bytes,
                Some(_) => Vec::new(),
                None => own,
            };
            s.held.extend_from_slice(&bytes);
            return Ok(());
        }
        match s.said.remove(&nth) {
            None => s.arrive(&own),
            Some(Say::Bytes(bytes)) => s.arrive(&bytes),
            Some(Say::Silent) => {}
            Some(Say::OnRelease(bytes)) => s.held = bytes,
            Some(Say::AcrossAStall(bytes, stall)) => s.across_a_stall = Some((bytes, stall)),
        }
        Ok(())
    }
    fn purge(&mut self) -> Result<()> {
        self.with(|s| s.purges += 1);
        Ok(())
    }
}

type Received = Receiver<(usize, Packet)>;

fn scpi(fake: &Fake) -> Scpi {
    Scpi::new(Box::new(Serial::from_port(Box::new(fake.clone()))))
}

fn device(fake: &Fake) -> Fluke45Device {
    device_with(fake, QUICK)
}

fn device_with(fake: &Fake, timing: Timing) -> Fluke45Device {
    let mut d = Fluke45Device::from_scpi(scpi(fake));
    d.timing = timing;
    d.open().unwrap();
    d
}

fn start(d: &mut Fluke45Device) -> Received {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

fn limit(d: &mut Fluke45Device, samples: u64) {
    d.set(&Key::LimitSamples, &Value::U64(samples), None).unwrap();
}

/// What a capture sent, up to its End, which must come within [`BOUND`].
struct Seen {
    packets: Vec<AnalogData>,
    failed: Option<String>,
}

fn seen(rx: &Received) -> Seen {
    let deadline = Instant::now() + BOUND;
    let mut out = Seen { packets: Vec::new(), failed: None };
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left).expect("no End in time").1 {
            Packet::Analog(a) => out.packets.push(a),
            Packet::Failed(why) => out.failed = Some(why),
            Packet::End => return out,
            _ => {}
        }
    }
}

/// A capture that must end well, by its values.
fn values(rx: &Received) -> Vec<f32> {
    let got = seen(rx);
    assert_eq!(got.failed, None);
    got.packets.iter().map(|p| p.samples[0]).collect()
}

/// Runs a test that can hang on its own thread, within [`BOUND`] and a margin.
fn bounded(test: impl FnOnce() + Send + 'static) {
    let (tx, rx) = sync_channel(1);
    let handle = std::thread::spawn(move || {
        test();
        let _ = tx.send(());
    });
    match rx.recv_timeout(BOUND * 2) {
        Ok(()) => handle.join().unwrap(),
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(handle.join().unwrap_err());
        }
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("the test hung"),
    }
}

/// A round with one display, and with what each query is, from 1.
const ROUND: [&str; 6] = ["FUNC1?", "AUTO?", "VAL1?", "MOD?", "FUNC2?", "AUTO?"];

#[test]
fn a_round_asks_as_c_does_and_ends_after_it() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        limit(&mut d, 2);
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(got.failed, None);
        assert_eq!(got.packets.len(), 2);
        let p = &got.packets[0];
        assert_eq!((p.channels.as_slice(), p.samples.as_slice()), (&[0][..], &[1.0][..]));
        assert_eq!((p.mq, p.unit, p.digits), (Mq::Voltage, Unit::Volt, 4));
        assert_eq!(p.flags, MqFlags::DC | MqFlags::AUTORANGE);
        assert_eq!(got.packets[1].samples, [2.0]);
        let mut twice = ROUND.to_vec();
        twice.extend(ROUND);
        assert_eq!(fake.writes(), twice);
        d.close().unwrap();
        assert_eq!(fake.with(|s| s.purges), 0, "nothing here drops what the line owes");
    });
}

/// A command goes only after the last one's prompt, at the line's own rate too.
#[test]
fn no_command_goes_ahead_of_a_prompt_at_the_lines_rate() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.spacing = Spacing::Line(LINE_RATE));
        let mut d = device(&fake);
        limit(&mut d, 2);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [1.0, 2.0]);
        d.close().unwrap();
        assert_eq!(fake.with(|s| s.early), 0);
        assert_eq!(fake.writes().len(), 2 * ROUND.len());
    });
}

/// What a read gets is not what a line is: a cable's latency timer has no regard.
#[test]
fn lines_are_found_in_whatever_pieces_they_come() {
    bounded(|| {
        for size in [1, 3, 7, 64] {
            let fake = Fake::new();
            fake.with(|s| s.spacing = Spacing::Burst(size));
            let mut d = device(&fake);
            limit(&mut d, 1);
            let rx = start(&mut d);
            assert_eq!(values(&rx), [1.0], "bursts of {size}");
            d.close().unwrap();
        }
    });
}

#[test]
fn an_answer_and_its_prompt_in_one_read_are_two_lines() {
    bounded(|| {
        let fake = Fake::new();
        // The fake gives what has arrived at one go: C's `VDC\r\n=>`.
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [1.0]);
    });
}

#[test]
fn a_time_limit_shorter_than_a_round_finishes_the_round_as_c_does() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.spacing = Spacing::Line(LINE_RATE));
        let mut d = device(&fake);
        d.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(values(&rx), [1.0]);
        assert_eq!(fake.writes(), ROUND);
    });
}

#[test]
fn two_displays_give_two_packets_for_one_sample() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.func2 = Some(b"FREQ".to_vec()));
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(got.packets.len(), 2);
        assert_eq!(got.packets[1].channels, [1]);
        assert_eq!((got.packets[1].mq, got.packets[1].samples[0]), (Mq::Frequency, 60.0));
        assert_eq!(
            fake.writes(),
            ["FUNC1?", "AUTO?", "VAL1?", "MOD?", "FUNC2?", "AUTO?", "VAL2?", "MOD?"]
        );
    });
}

#[test]
fn a_disabled_channel_is_asked_its_function_only() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.func2 = Some(b"FREQ".to_vec()));
        let mut d = device(&fake);
        d.enable_channel(0, false).unwrap();
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [60.0]);
        assert_eq!(fake.writes(), ["FUNC1?", "AUTO?", "FUNC2?", "AUTO?", "VAL2?", "MOD?"]);
    });
}

#[test]
fn an_overload_is_infinite_with_no_digits() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.values = vec!["+1E+9"]);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(got.packets[0].samples[0], f32::INFINITY);
        assert_eq!(got.packets[0].digits, 0);
    });
}

/// Refused, for each query of the round: C crashes on `VAL1?`'s and goes on from
/// the others.
#[test]
fn a_query_the_meter_refuses() {
    bounded(|| {
        for (nth, name) in ROUND.iter().enumerate().take(4) {
            let fake = Fake::new();
            fake.say_only(nth + 1, "!>");
            let mut d = device(&fake);
            limit(&mut d, 1);
            let rx = start(&mut d);
            let got = seen(&rx);
            assert_eq!(got.failed, None, "{name}");
            let writes = fake.writes();
            match *name {
                // The reading that was taken for it isn't said: the next is.
                "FUNC1?" | "VAL1?" => {
                    assert_eq!(writes[..2], ["FUNC1?", "AUTO?"], "{name}");
                    assert_eq!(writes[writes.len() - 4..], ROUND[2..], "{name}");
                    let expected = if *name == "VAL1?" { 2.0 } else { 1.0 };
                    assert_eq!(got.packets[0].samples, [expected], "{name}");
                }
                "AUTO?" => {
                    assert_eq!(got.packets[0].flags, MqFlags::DC, "{name}");
                    assert_eq!(writes, ROUND, "{name}");
                }
                _ => {
                    assert_eq!(got.packets[0].flags, MqFlags::DC | MqFlags::AUTORANGE, "{name}");
                    assert_eq!(writes, ROUND, "{name}");
                }
            }
        }
    });
}

/// A reading that isn't one: no sample, where C sends what the stack holds.
#[test]
fn what_is_no_reading_makes_no_sample_and_the_next_comes() {
    bounded(|| {
        for bad in ["nan", "0x10", "+1.2345", "+1.2E+0x", "1"] {
            let fake = Fake::new();
            fake.say_text(3, bad);
            let mut d = device(&fake);
            limit(&mut d, 1);
            let rx = start(&mut d);
            assert_eq!(values(&rx), [2.0], "{bad}");
        }
    });
}

#[test]
fn modifiers_that_are_none_are_no_flags_and_good_ones_are() {
    bounded(|| {
        let fake = Fake::new();
        fake.say_text(4, "abc");
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(got.packets[0].flags, MqFlags::DC | MqFlags::AUTORANGE);
        d.close().unwrap();

        let fake = Fake::new();
        fake.say_text(4, "40");
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let got = seen(&rx);
        let p = &got.packets[0];
        assert_eq!(
            (p.mq, p.unit, p.flags, p.digits),
            (Mq::PowerFactor, Unit::DecibelMw, MqFlags::RELATIVE, 2)
        );
    });
}

/// The late answer to `AUTO?` must not pass for `MOD?`'s, whose form it has, nor
/// `VAL1?`'s late one for a later reading.
#[test]
fn a_late_answer_shifts_nothing_after_it() {
    bounded(|| {
        let fake = Fake::new();
        // FUNC1? AUTO?(late) VAL1? MOD?: 1 would say MN.
        fake.say(2, Say::OnRelease(b"1\r\n=>\r\n".to_vec()));
        let mut d = device(&fake);
        limit(&mut d, 2);
        let rx = start(&mut d);
        fake.wait_for_writes(3);
        // Now out: AUTO? and VAL1?, whose answer waits behind the late one.
        fake.release();
        let got = seen(&rx);
        assert_eq!(got.failed, None);
        assert_eq!(got.packets[0].samples, [1.0]);
        assert_eq!(got.packets[1].samples, [2.0]);
        // The first went without its AUTO?, given up on; neither has a MOD? of 1.
        assert_eq!(got.packets[0].flags, MqFlags::DC);
        assert_eq!(got.packets[1].flags, MqFlags::DC | MqFlags::AUTORANGE);
    });
}

/// A prompt left by a scan or a stop, ahead of the first answer: C's own case.
#[test]
fn a_prompt_ahead_of_the_first_answer_is_passed_over() {
    bounded(|| {
        let fake = Fake::new();
        fake.say(1, Say::Bytes(b"=>\r\nVDC\r\n=>\r\n".to_vec()));
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [1.0]);
        assert_eq!(fake.writes(), ROUND);
    });
}

#[test]
fn a_meter_with_no_prompt_works_and_is_soon_not_waited_for() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.prompt = false);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let began = Instant::now();
        let rx = start(&mut d);
        assert_eq!(values(&rx), [1.0]);
        assert_eq!(fake.writes(), ROUND);
        let waited = began.elapsed();
        assert!(waited >= Duration::from_secs(2), "two waits, the rest short: {waited:?}");
    });
}

/// A line cut short and the next answer don't make one: `set 15` of `set 1500`.
#[test]
fn a_line_cut_short_is_not_the_head_of_the_next() {
    bounded(|| {
        let fake = Fake::new();
        fake.say(3, Say::Bytes(b"+1.".to_vec()));
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [2.0]);
    });
}

/// The time is taken ahead of the read that found nothing, and the answer is taken
/// when a stall let it in under the wait's end.
#[test]
fn a_stall_across_a_wait_is_no_silence() {
    bounded(|| {
        let fake = Fake::new();
        fake.say(1, Say::AcrossAStall(b"VDC\r\n=>\r\n".to_vec(), Duration::from_millis(1500)));
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [1.0]);
        assert_eq!(fake.writes(), ROUND, "nothing lost and nothing asked twice");
    });
}

#[test]
fn a_silent_meter_fails_at_the_bound_on_a_sample_limit() {
    bounded(|| {
        let fake = Fake::new();
        for nth in 1..100 {
            fake.say(nth, Say::Silent);
        }
        let timing = Timing { no_answer: Duration::from_millis(1500), ..QUICK };
        let mut d = device_with(&fake, timing);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(got.failed.as_deref(), Some("device error: no answer from the meter for 1.5 s"));
        assert_eq!(got.packets.len(), 0);
        // C goes on from a failed FUNC1? to FUNC2?, with no AUTO?.
        assert_eq!(fake.writes()[..2], ["FUNC1?", "FUNC2?"]);
    });
}

/// `VAL1?` silent, as on a blank display with an external trigger, with the rest
/// answered: no bound but the long one, and C's NULL does not come in.
#[test]
fn a_silent_reading_is_no_sample_and_answers_keep_the_long_bound() {
    bounded(|| {
        let fake = Fake::new();
        for round in 0..20 {
            fake.say(3 + round * 4, Say::Silent);
        }
        let timing = Timing {
            no_answer: Duration::from_millis(1),
            no_usable: Duration::from_millis(1),
            give_up: Duration::from_millis(1200),
        };
        let mut d = device_with(&fake, timing);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(
            got.failed.as_deref(),
            Some("device error: no reading from the meter for 1.2 s")
        );
    });
}

#[test]
fn junk_fails_at_its_own_bound() {
    bounded(|| {
        let fake = Fake::new();
        for nth in 1..100 {
            fake.say_text(nth, "zzz");
        }
        let timing = Timing { no_usable: Duration::from_millis(50), ..QUICK };
        let mut d = device_with(&fake, timing);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(
            got.failed.as_deref(),
            Some("device error: no usable answer from the meter for 0.05 s")
        );
    });
}

/// A stream with no line end, for every read's whole buffer: no sample limit runs
/// for ever because bytes flow.
#[test]
fn a_flood_with_no_line_end_ends_at_a_bound_and_at_a_stop() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.endless = Some(b'x'));
        let timing = Timing { no_usable: Duration::from_millis(100), ..QUICK };
        let mut d = device_with(&fake, timing);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(
            got.failed.as_deref(),
            Some("device error: no usable answer from the meter for 0.1 s")
        );
        d.close().unwrap();

        let fake = Fake::new();
        fake.with(|s| s.endless = Some(b'x'));
        let mut d = device(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        d.stop().unwrap();
        assert_eq!(seen(&rx).failed, None);
        let began = Instant::now();
        d.close().unwrap();
        assert!(began.elapsed() < Duration::from_secs(3));
    });
}

#[test]
fn a_time_limit_ends_a_silent_capture_well_within_a_second_or_so() {
    bounded(|| {
        let fake = Fake::new();
        for nth in 1..100 {
            fake.say(nth, Say::Silent);
        }
        let mut d = device(&fake);
        d.set(&Key::LimitMsec, &Value::U64(100), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(got.failed, None, "a time limit with nothing stays exit 0");
        assert!(began.elapsed() < Duration::from_secs(10), "{:?}", began.elapsed());
    });
}

/// Past the limit, a wait grown long is cut to C's second, or it has 8 s to run.
#[test]
fn a_time_limit_cuts_a_wait_grown_long() {
    bounded(|| {
        let fake = Fake::new();
        for nth in 1..100 {
            fake.say(nth, Say::Silent);
        }
        let mut d = device(&fake);
        d.set(&Key::LimitMsec, &Value::U64(5000), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        assert_eq!(seen(&rx).failed, None);
        assert!(began.elapsed() < Duration::from_secs(12), "{:?}", began.elapsed());
    });
}

/// With no stop, only the close can end this thread, which holds the fake.
#[test]
fn a_close_stops_and_joins() {
    bounded(|| {
        let fake = Fake::new();
        for nth in 1..100 {
            fake.say(nth, Say::Silent);
        }
        let mut d = device(&fake);
        let _rx = start(&mut d);
        fake.wait_for_writes(1);
        d.close().unwrap();
        assert_eq!(Arc::strong_count(&fake.0), 1, "the thread still holds the port");
    });
}

/// A line cut short does not join the answer to the next question.
#[test]
fn a_new_question_forgets_part_of_a_line() {
    let fake = Fake::new();
    let mut link = Link::new(scpi(&fake));
    link.ask("AUTO?\n", Kind::Flag).unwrap();
    link.take(b"+1.");
    link.ask("MOD?\n", Kind::Modifiers).unwrap();
    link.take(b"5E+0\r\n");
    assert_eq!(link.lines.pop_front().unwrap(), b"5E+0");
}

#[test]
fn a_line_longer_than_any_is_heard_once_and_ends_with_its_line_end() {
    let fake = Fake::new();
    let mut link = Link::new(scpi(&fake));
    link.take(&[b'x'; LONGEST_LINE + 10]);
    assert_eq!(link.lines.len(), 1);
    link.take(b"\n1\r\n");
    assert_eq!(link.lines.len(), 3);
    assert_eq!(link.lines[2], b"1");
}

#[test]
fn a_port_that_fails_fails_the_capture() {
    bounded(|| {
        let fake = Fake::new();
        fake.with(|s| s.fail_reads = true);
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(seen(&rx).failed.as_deref(), Some("device error: hung up"));
        d.close().unwrap();

        let fake = Fake::new();
        fake.with(|s| s.fail_writes_after = Some(2));
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(seen(&rx).failed.as_deref(), Some("device error: Broken pipe"));
    });
}

#[test]
fn a_stop_in_a_wait_ends_it_and_a_close_joins() {
    bounded(|| {
        let fake = Fake::new();
        for nth in 1..100 {
            fake.say(nth, Say::Silent);
        }
        let mut d = device(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(1);
        let began = Instant::now();
        d.stop().unwrap();
        assert_eq!(seen(&rx).failed, None);
        d.close().unwrap();
        assert!(began.elapsed() < Duration::from_secs(3), "{:?}", began.elapsed());
        let writes = fake.writes().len();
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(fake.writes().len(), writes, "no write after a close");
    });
}

/// What a stop left out goes with its question, and a second capture starts in step.
#[test]
fn a_second_capture_is_not_one_behind_after_a_stop_in_a_reply() {
    bounded(|| {
        let fake = Fake::new();
        fake.say(3, Say::OnRelease(b"+9.0000E+0\r\n=>\r\n".to_vec()));
        let mut d = device(&fake);
        let rx = start(&mut d);
        fake.wait_for_writes(3);
        d.stop().unwrap();
        // The stop is in the settle now, and the answer comes while it waits.
        fake.release();
        assert_eq!(seen(&rx).failed, None);
        limit(&mut d, 1);
        let rx = start(&mut d);
        let got = seen(&rx);
        assert_eq!(got.failed, None);
        assert_eq!(got.packets[0].samples, [2.0], "not the 9 of the capture before");
        assert_eq!(got.packets[0].flags, MqFlags::DC | MqFlags::AUTORANGE);
    });
}

/// A start after a capture whose End is stuck on a full queue does not hang.
#[test]
fn a_start_after_an_end_stuck_on_a_full_queue_goes() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        limit(&mut d, 1);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Queue of one, and nobody reads: wait for its thread to stick.
        fake.wait_for_writes(1);
        let deadline = Instant::now() + BOUND;
        loop {
            match d.start(PacketSink::new(sync_channel(1).0, 0)) {
                Ok(()) => break,
                Err(Error::Busy) => {
                    assert!(Instant::now() < deadline, "still busy");
                    d.stop().unwrap();
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(e) => panic!("{e}"),
            }
        }
        drop(rx);
        d.stop().unwrap();
        d.close().unwrap();
    });
}

#[test]
fn the_sink_goes_at_the_end_and_nothing_holds_it() {
    bounded(|| {
        let fake = Fake::new();
        let mut d = device(&fake);
        limit(&mut d, 1);
        let rx = start(&mut d);
        assert_eq!(values(&rx), [1.0]);
        d.close().unwrap();
        assert!(rx.recv().is_err(), "the old sink is released");
    });
}

// The scan.

#[test]
fn a_probe_asks_once_and_takes_the_id() {
    let fake = Fake::new();
    let found = probe(scpi(&fake), Some("9600/8n1")).unwrap();
    assert_eq!(fake.writes(), ["*IDN?"]);
    assert_eq!(found.info.vendor, "FLUKE");
    assert_eq!(found.info.model, "45");
    assert_eq!(found.info.serial, "7654321");
    assert_eq!(found.info.version, "1.6 D1.6");
    assert_eq!(found.serialcomm.as_deref(), Some("9600/8n1"));
    assert_eq!(found.resource, "/dev/ttyFAKE");
    let names: Vec<&str> = found.channels.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["P1", "P2"]);
    assert!(found.channels.iter().all(|c| c.enabled));
}

#[test]
fn a_probe_asks_once_more_after_silence() {
    let fake = Fake::new();
    fake.say(1, Say::Silent);
    assert!(probe(scpi(&fake), None).is_some());
    assert_eq!(fake.writes(), ["*IDN?", "*IDN?"]);
    let fake = Fake::new();
    fake.say(1, Say::Silent);
    fake.say(2, Say::Silent);
    assert!(probe(scpi(&fake), None).is_none());
    assert_eq!(fake.writes(), ["*IDN?", "*IDN?"]);
}

#[test]
fn a_probe_takes_only_this_meter_and_any_case_of_its_maker() {
    for (idn, is) in [
        ("fluke, 45, 1, 2", true),
        ("IDN FLUKE,45,1,2", true),
        ("FLUKE,45,2", true),
        ("FLUKE, 8846A, 1, 2", false),
        ("Fluke, 45A, 1, 2", false),
        ("ACME, 45, 1, 2", false),
    ] {
        let fake = Fake::new();
        fake.say_text(1, idn);
        assert_eq!(probe(scpi(&fake), None).is_some(), is, "{idn}");
        assert_eq!(fake.writes(), ["*IDN?"], "{idn}");
    }
}

#[test]
fn only_a_serial_conn_is_probed() {
    for options in [
        vec![],
        vec![(Key::Conn, Value::Str("usbtmc".to_owned()))],
        vec![(Key::Conn, Value::Str("tcp-raw/127.0.0.1/1".to_owned()))],
        vec![(Key::Conn, Value::Str(String::new()))],
    ] {
        assert!(Fluke45Driver.scan(&options).unwrap().is_empty());
    }
    assert_eq!(Fluke45Driver.scan_options(), [Key::Conn, Key::Serialcomm]);
    assert_eq!(Fluke45Driver.id(), "fluke-45");
    assert_eq!(Fluke45Driver.long_name(), "Fluke 45");
}

#[test]
fn keys() {
    let fake = Fake::new();
    let mut d = device(&fake);
    assert!(d.get(&Key::LimitSamples, None).is_err());
    assert!(d.list(&Key::LimitSamples, None).is_err());
    assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::Conn, &Value::U64(1), None).is_err());
    assert_eq!(d.options(None).unwrap().len(), 3);
    assert_eq!(d.classes(), [DeviceClass::Multimeter]);
    assert!(d.enable_channel(2, true).is_err());
    assert!(d.rename_channel(2, "x").is_err());
    d.rename_channel(0, "x").unwrap();
    assert_eq!(d.channels()[0].name, "x");
    d.close().unwrap();
    let (tx, _rx) = sync_channel(1);
    assert!(d.start(PacketSink::new(tx, 0)).is_err(), "not open");
    assert!(d.list(&Key::LimitMsec, None).is_err());
}
