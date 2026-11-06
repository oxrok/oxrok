// Counterpart in sigrok:
//   libsigrok/src/hardware/lecroy-xstream/api.c,
//   libsigrok/src/hardware/lecroy-xstream/protocol.c,
//   Copyright (C) 2017 Sven Schnelle <svens@stackframe.org>,
//   with commits by Uwe Hermann, Soeren Apel, Gerhard Sittig,
//   Sylvain Pelissier, Bert Vermeulen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a scripted line. What a scope says comes by the Nth read, so a loaded
//! machine slows the fake and the driver alike; the emulator's cases hold the rest.

use super::*;
use crate::scpi::{HwInfo, ScpiTransport};
use crate::serial::WriteLog;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// What a question comes to: pieces, each after so many reads that find nothing.
type Pieces = Vec<(u32, Vec<u8>)>;
type Script = Box<dyn FnMut(&str) -> Pieces + Send>;

#[derive(Default)]
struct Line {
    writes: WriteLog,
    coming: VecDeque<(u32, Vec<u8>)>,
    closes: u32,
    opens: u32,
    open: bool,
    discards: u32,
    writes_while_closed: u32,
    /// The send fails from then on.
    broken: bool,
}

struct Fake {
    line: Arc<Mutex<Line>>,
    script: Script,
}

impl ScpiTransport for Fake {
    fn resource(&self) -> &'static str {
        "fake"
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        let text = String::from_utf8_lossy(data).trim_end().to_owned();
        let pieces = (self.script)(&text);
        let mut line = self.line.lock().unwrap();
        if line.broken {
            return Err(Error::Disconnected);
        }
        if !line.open {
            line.writes_while_closed += 1;
        }
        line.writes.push((Instant::now(), data.to_vec()));
        line.coming.extend(pieces);
        Ok(())
    }

    fn read_begin(&mut self) -> Result<()> {
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<usize> {
        let mut line = self.line.lock().unwrap();
        let Some(front) = line.coming.front_mut() else {
            drop(line);
            std::thread::sleep(Duration::from_millis(1));
            return Ok(0);
        };
        if front.0 > 0 {
            front.0 -= 1;
            drop(line);
            std::thread::sleep(Duration::from_millis(1));
            return Ok(0);
        }
        let n = buf.len().min(front.1.len());
        buf[..n].copy_from_slice(&front.1[..n]);
        front.1.drain(..n);
        if front.1.is_empty() {
            line.coming.pop_front();
        }
        Ok(n)
    }

    fn read_complete(&self) -> bool {
        self.line.lock().unwrap().coming.is_empty()
    }

    fn close(&mut self) -> Result<()> {
        let mut line = self.line.lock().unwrap();
        line.closes += 1;
        line.open = false;
        Ok(())
    }

    fn reopen(&mut self) -> Result<()> {
        let mut line = self.line.lock().unwrap();
        line.opens += 1;
        line.open = true;
        Ok(())
    }

    fn discard(&mut self, _timeout: Duration) -> Result<Option<usize>> {
        let mut line = self.line.lock().unwrap();
        line.discards += 1;
        // Only what is due: what is held back is still on its way.
        if line.coming.front().is_some_and(|c| c.0 == 0) {
            let n = line.coming.pop_front().map_or(0, |c| c.1.len());
            return Ok(Some(n));
        }
        if let Some(front) = line.coming.front_mut() {
            front.0 -= 1;
        }
        Ok(Some(0))
    }
}

/// A block as a scope sends it, with its line end.
fn block(samples: &[i16], gain: f32, unit: &[u8]) -> Vec<u8> {
    let mut d = vec![0u8; 346];
    d[..8].copy_from_slice(b"WAVEDESC");
    d[16..26].copy_from_slice(b"LECROY_2_3");
    d[32] = 1;
    d[34] = 1;
    d[36..40].copy_from_slice(&346u32.to_le_bytes());
    d[116..120].copy_from_slice(&(samples.len() as u32).to_le_bytes());
    d[156..160].copy_from_slice(&gain.to_le_bytes());
    d[196..196 + unit.len()].copy_from_slice(unit);
    for s in samples {
        d.extend_from_slice(&s.to_le_bytes());
    }
    let mut out = format!("#9{:09}", d.len()).into_bytes();
    out.extend_from_slice(&d);
    out.push(b'\n');
    out
}

/// A scope's answers to the driver's questions at once, and no others.
fn plain(question: &str) -> Pieces {
    let at_once = |text: &str| vec![(0, format!("{text}\n").into_bytes())];
    if question.ends_with(":WAVEFORM?") {
        return vec![(0, block(&[100, 200, 300], 0.5, b"V"))];
    }
    match question.rsplit_once(':').map_or(question, |(_, tail)| tail) {
        "TRACE?" => at_once("ON"),
        "VDIV?" => at_once("1.000E+00"),
        "OFFSET?" | "TRIG_DELAY?" => at_once("0"),
        "COUPLING?" => at_once("D1M"),
        "TIME_DIV?" => at_once("1.000E-03"),
        "TRIG_SELECT?" => at_once("EDGE,SR,C1,HT,OFF"),
        "TRIG_SLOPE?" => at_once("POS"),
        "*OPC?" => at_once("1"),
        _ => Vec::new(),
    }
}

fn device(script: impl FnMut(&str) -> Pieces + Send + 'static) -> (LecroyDevice, Arc<Mutex<Line>>) {
    let line = Arc::new(Mutex::new(Line::default()));
    let fake = Fake { line: Arc::clone(&line), script: Box::new(script) };
    let id = HwInfo {
        manufacturer: "LECROY".to_owned(),
        model: "X".to_owned(),
        serial_number: "1".to_owned(),
        firmware_version: "2".to_owned(),
    };
    let mut d = LecroyDevice::new(&id, Scpi::new(Box::new(fake)), [true; 4]);
    d.open().unwrap();
    line.lock().unwrap().writes = WriteLog::default();
    (d, line)
}

fn sent(line: &Arc<Mutex<Line>>) -> Vec<String> {
    let line = line.lock().unwrap();
    line.writes.iter().map(|w| String::from_utf8_lossy(&w.1).trim_end().to_owned()).collect()
}

/// Runs a test on its own thread with a bound: a stop ignored must fail, not hang.
fn bounded(test: impl FnOnce() + Send + 'static) {
    let (tx, rx) = sync_channel(1);
    std::thread::spawn(move || {
        test();
        let _ = tx.send(());
    });
    match rx.recv_timeout(Duration::from_secs(60)) {
        Ok(()) => {}
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("the test hangs"),
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => panic!("the test panicked"),
    }
}

/// Packets to the end, with bounds on time and on how many.
fn packets(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let mut out = Vec::new();
    while let Ok((_, p)) = rx.recv_timeout(Duration::from_secs(20)) {
        let end = matches!(p, Packet::End);
        out.push(p);
        if end {
            return out;
        }
        assert!(out.len() < 10_000, "a limit was ignored");
    }
    panic!("no end: {} packets", out.len());
}

/// Waits for a packet that `wanted` picks.
fn until(rx: &Receiver<(usize, Packet)>, wanted: impl Fn(&Packet) -> bool) {
    for _ in 0..10_000 {
        let (_, p) = rx.recv_timeout(Duration::from_secs(20)).expect("no such packet");
        if wanted(&p) {
            return;
        }
    }
    panic!("no such packet in 10000");
}

fn failure(packets: &[Packet]) -> Option<&str> {
    packets.iter().find_map(|p| if let Packet::Failed(why) = p { Some(why.as_str()) } else { None })
}

fn start(d: &mut LecroyDevice) -> Receiver<(usize, Packet)> {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

fn one_channel(d: &mut LecroyDevice) {
    for pos in 1..4 {
        d.enable_channel(pos, false).unwrap();
    }
}

#[test]
fn a_frame_goes_channel_by_channel_and_ends_at_the_limit() {
    bounded(|| {
        let (mut d, line) = device(plain);
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        d.enable_channel(1, false).unwrap();
        d.enable_channel(3, false).unwrap();
        line.lock().unwrap().writes = WriteLog::default();
        let got = packets(&start(&mut d));
        let order: Vec<&str> = got
            .iter()
            .map(|p| match p {
                Packet::Header { .. } => "header",
                Packet::FrameBegin => "begin",
                Packet::Analog(_) => "analog",
                Packet::FrameEnd => "end",
                Packet::End => "last",
                _ => "other",
            })
            .collect();
        assert_eq!(order, ["header", "begin", "analog", "analog", "end", "last"]);
        let Packet::Analog(a) = &got[2] else { panic!() };
        assert_eq!(a.channels, [0]);
        assert_eq!(a.samples, [50.0, 100.0, 150.0]);
        assert_eq!(
            (a.mq, a.unit, a.digits),
            (crate::core::Mq::Voltage, crate::core::Unit::Volt, 2)
        );
        let Packet::Analog(a) = &got[3] else { panic!() };
        assert_eq!(a.channels, [2]);
        assert_eq!(sent(&line), ["C1:WAVEFORM?", "TIME_DIV?", "C3:WAVEFORM?"]);
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(300), "3 / (1 ms * 10)");
    });
}

#[test]
fn the_next_frame_is_armed_and_asked_apart() {
    bounded(|| {
        let (mut d, line) = device(plain);
        one_channel(&mut d);
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        line.lock().unwrap().writes = WriteLog::default();
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), None);
        assert_eq!(got.iter().filter(|p| matches!(p, Packet::FrameEnd)).count(), 2);
        assert_eq!(sent(&line), ["C1:WAVEFORM?", "TIME_DIV?", "ARM;WAIT;*OPC", "C1:WAVEFORM?"]);
    });
}

#[test]
fn no_data_arms_in_one_line_and_asks_the_rate_again() {
    bounded(|| {
        let mut left = 2;
        let (mut d, line) = device(move |q| {
            if q.ends_with(":WAVEFORM?") && left > 0 {
                left -= 1;
                return vec![(0, block(&[], 1.0, b"V"))];
            }
            plain(q)
        });
        one_channel(&mut d);
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        line.lock().unwrap().writes = WriteLog::default();
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), None);
        assert_eq!(
            sent(&line),
            [
                "C1:WAVEFORM?",
                "ARM;WAIT;*OPC;C1:WAVEFORM?",
                "ARM;WAIT;*OPC;C1:WAVEFORM?",
                "TIME_DIV?"
            ]
        );
    });
}

#[test]
fn a_stop_ends_a_wait_for_a_trigger() {
    bounded(|| {
        let mut first = true;
        let (mut d, line) = device(move |q| {
            if q.ends_with(":WAVEFORM?") && !std::mem::take(&mut first) {
                return Vec::new();
            }
            plain(q)
        });
        one_channel(&mut d);
        let rx = start(&mut d);
        until(&rx, |p| matches!(p, Packet::FrameEnd));
        // Armed and silent: the thread is only in its wait now.
        while sent(&line).last().map(String::as_str) != Some("C1:WAVEFORM?")
            || sent(&line).len() < 4
        {
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        let rest = packets(&rx);
        assert_eq!(failure(&rest), None, "a stop is no failure");
        let before = line.lock().unwrap().writes.len();
        d.close().unwrap();
        assert_eq!(line.lock().unwrap().writes.len(), before, "a close says nothing");
    });
}

#[test]
fn a_stop_with_an_answer_out_settles_the_line_before_the_next_question() {
    bounded(|| {
        let mut waves = 0;
        let (mut d, line) = device(move |q| {
            if q.ends_with(":WAVEFORM?") {
                waves += 1;
                if waves == 1 {
                    // Long after the stop, as a late trigger is.
                    return vec![(100_000, b"STALE".to_vec())];
                }
            }
            plain(q)
        });
        one_channel(&mut d);
        let rx = start(&mut d);
        until(&rx, |p| matches!(p, Packet::Header { .. }));
        while sent(&line).is_empty() {
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        assert_eq!(line.lock().unwrap().discards, 0);
        // It never comes after all, or the next *OPC? would wait behind it.
        line.lock().unwrap().coming.clear();
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        assert!(line.lock().unwrap().discards > 0, "the send looked at the line first");
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), None);
    });
}

/// With bounds a test can wait out. The limit's never reached by a test not about it.
fn short(now: Instant, _time_div: f64) -> Silence {
    Silence::with_bounds(
        now,
        Duration::from_millis(80),
        Duration::from_millis(400),
        Duration::from_millis(80),
    )
}

#[test]
fn a_scope_silent_from_the_start_fails_at_the_bound() {
    bounded(|| {
        let (mut d, _line) =
            device(|q| if q.ends_with(":WAVEFORM?") { Vec::new() } else { plain(q) });
        d.silence = short;
        let got = packets(&start(&mut d));
        assert!(
            failure(&got).is_some_and(|w| w.contains("No answer from the scope in 0 s")),
            "{got:?}"
        );
        assert!(!got.iter().any(|p| matches!(p, Packet::FrameBegin)));
    });
}

#[test]
fn a_scope_silent_after_a_sample_fails_only_at_the_limit() {
    bounded(|| {
        let mut waves = 0;
        let (mut d, _line) = device(move |q| {
            if q.ends_with(":WAVEFORM?") {
                waves += 1;
                if waves == 2 {
                    return Vec::new();
                }
            }
            plain(q)
        });
        d.silence = short;
        let began = Instant::now();
        let got = packets(&start(&mut d));
        assert!(failure(&got).is_some_and(|w| w.contains("has been silent for 0 s")), "{got:?}");
        assert!(began.elapsed() >= Duration::from_millis(400), "the limit, and not the bound");
    });
}

#[test]
fn an_armed_scope_does_not_fail_on_silence() {
    bounded(|| {
        let mut waves = 0;
        let (mut d, _line) = device(move |q| {
            if q.ends_with(":WAVEFORM?") {
                waves += 1;
                if waves == 1 {
                    return vec![(0, block(&[], 1.0, b"V"))];
                }
                // Past the bound and the limit, read by read.
                return vec![(1000, block(&[1], 1.0, b"V"))];
            }
            plain(q)
        });
        one_channel(&mut d);
        d.silence = short;
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), None, "{got:?}");
    });
}

#[test]
fn a_block_cut_short_fails_and_is_said_for_what_it_is() {
    bounded(|| {
        let (mut d, _line) = device(|q| {
            if q.ends_with(":WAVEFORM?") {
                let mut b = block(&[1, 2, 3], 1.0, b"V");
                b.truncate(b.len() / 2);
                return vec![(0, b)];
            }
            plain(q)
        });
        d.silence = short;
        let got = packets(&start(&mut d));
        assert!(failure(&got).is_some_and(|w| w.contains("stopped within a block")), "{got:?}");
        assert!(!got.iter().any(|p| matches!(p, Packet::Analog(_))));
    });
}

#[test]
fn bytes_that_are_no_block_fail_at_once() {
    bounded(|| {
        let (mut d, line) = device(|q| {
            if q.ends_with(":WAVEFORM?") {
                return vec![(0, b"ALL,#9000000001x\n".to_vec())];
            }
            plain(q)
        });
        // The bound far off, so that only the bytes can end it.
        d.silence = Silence::new;
        let began = Instant::now();
        let got = packets(&start(&mut d));
        assert!(
            failure(&got).is_some_and(|w| w.contains("no block: it starts with 'ALL,#9")),
            "{got:?}"
        );
        assert!(began.elapsed() < Duration::from_secs(25));
        assert_eq!(sent(&line).len(), 1, "nothing more is asked");
    });
}

#[test]
fn a_bad_descriptor_fails_the_capture() {
    bounded(|| {
        let (mut d, _line) = device(|q| {
            if q.ends_with(":WAVEFORM?") {
                let mut b = block(&[1], 1.0, b"V");
                b[11 + 16] = b'X';
                return vec![(0, b)];
            }
            plain(q)
        });
        let got = packets(&start(&mut d));
        assert!(failure(&got).is_some_and(|w| w.contains("template 'XECROY_2_3' not supported")));
    });
}

#[test]
fn the_rate_with_no_answer_fails_where_c_would_hang() {
    bounded(|| {
        // The open asks it too, so it goes silent after the open.
        let mut asked = 0;
        let (mut d, _line) = device(move |q| {
            if q == "TIME_DIV?" {
                asked += 1;
                if asked > 1 {
                    return Vec::new();
                }
            }
            plain(q)
        });
        one_channel(&mut d);
        let got = packets(&start(&mut d));
        assert!(failure(&got).is_some(), "{got:?}");
        assert!(!got.iter().any(|p| matches!(p, Packet::Analog(_))));
    });
}

#[test]
fn a_timebase_of_no_use_gives_no_rate_and_keeps_the_samples() {
    bounded(|| {
        let mut asked = 0;
        let (mut d, _line) = device(move |q| {
            if q == "TIME_DIV?" {
                asked += 1;
                if asked > 1 {
                    return vec![(0, b"0\n".to_vec())];
                }
            }
            plain(q)
        });
        one_channel(&mut d);
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), None);
        assert!(got.iter().any(|p| matches!(p, Packet::Analog(_))));
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(0));
    });
}

#[test]
fn a_gone_line_fails_the_capture() {
    bounded(|| {
        let (mut d, line) = device(plain);
        one_channel(&mut d);
        let rx = start(&mut d);
        until(&rx, |p| matches!(p, Packet::FrameEnd));
        line.lock().unwrap().broken = true;
        let rest = packets(&rx);
        assert!(failure(&rest).is_some(), "{rest:?}");
    });
}

#[test]
fn a_line_end_past_a_reads_edge_is_no_answer() {
    bounded(|| {
        let (mut d, _line) = device(|q| {
            if q.ends_with(":WAVEFORM?") {
                let mut b = block(&[10], 2.0, b"V");
                b.pop();
                return vec![(0, b), (0, b"\n".to_vec())];
            }
            plain(q)
        });
        one_channel(&mut d);
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), None, "{got:?}");
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100));
    });
}

#[test]
fn a_block_at_a_real_pace_arrives_in_pieces() {
    bounded(|| {
        let (mut d, _line) = device(|q| {
            if q.ends_with(":WAVEFORM?") {
                let b = block(&[7; 200], 1.0, b"V");
                return b.chunks(50).map(|c| (3, c.to_vec())).collect();
            }
            plain(q)
        });
        one_channel(&mut d);
        d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), None, "{got:?}");
        let n: usize =
            got.iter().map(|p| if let Packet::Analog(a) = p { a.len() } else { 0 }).sum();
        assert_eq!(n, 400);
    });
}

/// A read that asks, as USBTMC's: nothing comes, the read says so, and it asks again.
struct Asking {
    answers: VecDeque<Result<Vec<u8>>>,
    begins: Arc<Mutex<u32>>,
    current: Vec<u8>,
}

impl ScpiTransport for Asking {
    fn resource(&self) -> &'static str {
        "asking"
    }

    fn send(&mut self, _data: &[u8]) -> Result<()> {
        Ok(())
    }

    fn read_begin(&mut self) -> Result<()> {
        *self.begins.lock().unwrap() += 1;
        self.current = self.answers.pop_front().unwrap_or(Err(Error::Timeout))?;
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<usize> {
        if self.current.is_empty() {
            return Err(Error::Timeout);
        }
        let n = buf.len().min(self.current.len());
        buf[..n].copy_from_slice(&self.current[..n]);
        self.current.drain(..n);
        Ok(n)
    }

    fn read_complete(&self) -> bool {
        self.current.is_empty()
    }
}

#[test]
fn a_read_that_found_nothing_asks_again_and_one_within_a_block_does_not() {
    bounded(|| {
        let begins = Arc::new(Mutex::new(0));
        let whole = block(&[1, 2], 1.0, b"V");
        let answers = VecDeque::from([Err(Error::Timeout), Err(Error::Timeout), Ok(whole.clone())]);
        let fake = Asking { answers, begins: Arc::clone(&begins), current: Vec::new() };
        let mut scpi = Scpi::new(Box::new(fake));
        let mut silence = Silence::new(Instant::now(), 1e-3);
        let stop = AtomicBool::new(false);
        let Read::Block(got) = read_block(&mut scpi, &mut silence, &stop).unwrap() else {
            panic!()
        };
        assert_eq!(got.len(), whole.len() - 12, "no head and no line end");
        assert_eq!(*begins.lock().unwrap(), 3, "asked three times");
        assert_eq!(scpi.read_timeout, READ_TIMEOUT, "as a get wants it");

        let begins = Arc::new(Mutex::new(0));
        let answers = VecDeque::from([Ok(whole[..100].to_vec()), Err(Error::Disconnected)]);
        let fake = Asking { answers, begins: Arc::clone(&begins), current: Vec::new() };
        let mut scpi = Scpi::new(Box::new(fake));
        let mut silence = Silence::with_bounds(
            Instant::now(),
            Duration::from_millis(30),
            Duration::from_millis(30),
            Duration::from_millis(30),
        );
        let e = read_block(&mut scpi, &mut silence, &stop).map(|_| ()).unwrap_err();
        assert!(e.to_string().contains("stopped within a block"), "{e}");
        assert_eq!(*begins.lock().unwrap(), 1, "the rest is of the same asking");
    });
}

#[test]
fn a_close_joins_a_capture_on_a_full_queue() {
    bounded(|| {
        let (mut d, _line) = device(plain);
        one_channel(&mut d);
        let (tx, _unread) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        d.close().unwrap();
        assert!(d.worker.is_none());
    });
}

#[test]
fn a_restart_does_not_wait_for_a_stuck_end_and_lets_the_old_sink_go() {
    bounded(|| {
        let (mut d, _line) = device(plain);
        one_channel(&mut d);
        let (tx, old) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        d.limit_frames = 1;
        let rx = start(&mut d);
        let got = packets(&rx);
        assert_eq!(failure(&got), None);
        // The queue held one; after it nobody is sending any more.
        let _ = old.recv_timeout(Duration::from_secs(5));
        assert!(old.recv_timeout(Duration::from_secs(5)).is_err(), "the old worker let go");
    });
}

#[test]
fn settings_say_what_c_says_and_are_kept() {
    let (mut d, line) = device(plain);
    let rational = |p, q| Value::Rational(p, q);
    d.set(&Key::Timebase, &rational(1, 500), None).unwrap();
    assert_eq!(d.get(&Key::Timebase, None).unwrap(), rational(2, 1000));
    d.set(&Key::Vdiv, &rational(1, 200), Some("CH2")).unwrap();
    assert_eq!(d.get(&Key::Vdiv, Some("CH2")).unwrap(), rational(5, 1000));
    d.set(&Key::Coupling, &Value::Str("D50".to_owned()), Some("CH4")).unwrap();
    d.set(&Key::TriggerSource, &Value::Str("LINE".to_owned()), None).unwrap();
    d.set(&Key::TriggerSlope, &Value::Str("NEG".to_owned()), None).unwrap();
    d.set(&Key::HorizTriggerpos, &Value::F64(0.25), None).unwrap();
    assert_eq!(d.get(&Key::HorizTriggerpos, None).unwrap(), Value::F64(0.25));
    assert_eq!(
        sent(&line),
        [
            "TIME_DIV 2.000000E-03",
            "*OPC?",
            "C2:VDIV 5.000000E-03",
            "*OPC?",
            "*OPC?",
            "C4:COUPLING D50",
            "*OPC?",
            "*OPC?",
            "TRIG_SELECT EDGE,SR,LINE",
            "*OPC?",
            "LINE:TRIG_SLOPE NEG",
            "*OPC?",
            "TRIG POS 5.000000e-03 S",
            "*OPC?",
        ]
    );
}

#[test]
fn a_setting_refused_by_silence_keeps_the_state_and_says_so() {
    bounded(|| {
        let (mut d, _line) = device(|q| if q == "*OPC?" { Vec::new() } else { plain(q) });
        d.scpi.lock().unwrap().as_mut().unwrap().opc_silence = Duration::ZERO;
        let was = d.get(&Key::Timebase, None).unwrap();
        assert!(d.set(&Key::Timebase, &Value::Rational(1, 1), None).is_err());
        assert_eq!(d.get(&Key::Timebase, None).unwrap(), was, "not what was asked");
        assert!(d.enable_channel(0, false).is_err());
        assert!(d.channels()[0].enabled, "not off: nothing said so");
    });
}

#[test]
fn a_setting_during_a_capture_is_refused_and_not_kept() {
    bounded(|| {
        let (mut d, line) =
            device(|q| if q.ends_with(":WAVEFORM?") { Vec::new() } else { plain(q) });
        one_channel(&mut d);
        let rx = start(&mut d);
        until(&rx, |p| matches!(p, Packet::Header { .. }));
        let was = d.get(&Key::Timebase, None).unwrap();
        assert!(matches!(d.set(&Key::Timebase, &Value::Rational(1, 1), None), Err(Error::Busy)));
        assert_eq!(d.get(&Key::Timebase, None).unwrap(), was);
        d.stop().unwrap();
        assert!(!sent(&line).iter().any(|w| w.starts_with("TIME_DIV ")));
    });
}

#[test]
fn nan_is_no_trigger_position() {
    let (mut d, line) = device(plain);
    for value in [f64::NAN, -0.1, 1.5] {
        assert!(d.set(&Key::HorizTriggerpos, &Value::F64(value), None).is_err());
    }
    assert_eq!(sent(&line), Vec::<String>::new());
}

#[test]
fn a_channel_goes_on_the_wire_only_when_it_changes() {
    let (mut d, line) = device(plain);
    d.enable_channel(0, true).unwrap();
    assert_eq!(sent(&line), Vec::<String>::new(), "it was on");
    d.enable_channel(0, false).unwrap();
    assert_eq!(sent(&line), ["C1:TRACE OFF", "*OPC?"]);
    d.close().unwrap();
    d.enable_channel(1, false).unwrap();
    assert_eq!(sent(&line).len(), 2, "a closed scope hears nothing");
    assert_eq!(line.lock().unwrap().writes_while_closed, 0);
    assert!(!d.channels()[1].enabled);
}

#[test]
fn a_capture_puts_right_what_changed_since_the_scan() {
    bounded(|| {
        let (mut d, line) = device(|q| {
            if q == "C1:TRACE?" || q == "C2:TRACE?" {
                return vec![(0, b"OFF\n".to_vec())];
            }
            plain(q)
        });
        d.close().unwrap();
        d.open().unwrap();
        d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
        d.enable_channel(1, false).unwrap();
        d.enable_channel(2, false).unwrap();
        d.enable_channel(3, false).unwrap();
        line.lock().unwrap().writes = WriteLog::default();
        let got = packets(&start(&mut d));
        assert_eq!(failure(&got), None);
        assert_eq!(sent(&line)[0], "C1:TRACE ON", "on in the list, off on the scope");
    });
}

#[test]
fn a_capture_with_nothing_on_does_not_start() {
    let (mut d, line) = device(plain);
    for pos in 0..4 {
        d.enable_channel(pos, false).unwrap();
    }
    line.lock().unwrap().writes = WriteLog::default();
    let (tx, rx) = sync_channel(4);
    assert!(d.start(PacketSink::new(tx, 0)).is_err());
    assert!(rx.try_recv().is_err(), "no header");
    assert_eq!(sent(&line), Vec::<String>::new());
}

/// The answers an open needs, with one replaced, and what it comes to.
fn open_with(question: &'static str, answer: &'static str) -> Result<()> {
    let line = Arc::new(Mutex::new(Line::default()));
    let script = move |q: &str| {
        if q == question { vec![(0, format!("{answer}\n").into_bytes())] } else { plain(q) }
    };
    let fake = Fake { line, script: Box::new(script) };
    let id = HwInfo::default();
    LecroyDevice::new(&id, Scpi::new(Box::new(fake)), [true; 4]).open()
}

#[test]
fn an_open_refuses_what_it_cannot_place() {
    assert!(open_with("C1:VDIV?", "1.000E+00").is_ok());
    for (question, answer, says) in [
        ("C3:VDIV?", "7.5E-01", "A vertical scale off the table: '7.5E-01'"),
        ("C2:COUPLING?", "XYZ", "An unknown coupling: 'XYZ'"),
        ("TIME_DIV?", "abc", "A timebase off the table: 'abc'"),
        ("TRIG_SELECT?", "EDGE", "A trigger with no source: 'EDGE'"),
        ("TRIG_SELECT?", "EDGE,SR,EX", "An unknown trigger source: 'EX'"),
        ("C1:TRIG_SLOPE?", "WINDOW", "An unknown trigger slope: 'WINDOW'"),
    ] {
        let e = open_with(question, answer).unwrap_err().to_string();
        assert!(e.contains(says), "{e}");
    }
    assert!(open_with("C1:TRACE?", "maybe").is_err());
    assert!(open_with("TRIG_DELAY?", "abc").is_err());
    assert!(open_with("C4:OFFSET?", "abc").is_err());
}

#[test]
fn a_failed_open_leaves_a_device_that_can_open_again() {
    let mut bad = true;
    let line = Arc::new(Mutex::new(Line::default()));
    let script = move |q: &str| {
        if q == "TIME_DIV?" && std::mem::take(&mut bad) {
            return vec![(0, b"abc\n".to_vec())];
        }
        plain(q)
    };
    let fake = Fake { line: Arc::clone(&line), script: Box::new(script) };
    let mut d = LecroyDevice::new(&HwInfo::default(), Scpi::new(Box::new(fake)), [true; 4]);
    assert!(d.open().is_err());
    assert_eq!(line.lock().unwrap().closes, 1);
    d.open().unwrap();
    assert_eq!(line.lock().unwrap().opens, 2);
}

#[test]
fn the_tables_are_the_cs() {
    assert_eq!(TIMEBASES.len(), 42);
    assert_eq!(TIMEBASES[0], (20, 1_000_000_000_000));
    assert_eq!(TIMEBASES[41], (1000, 1));
    assert_eq!(VDIVS[0], (1, 1000));
    assert_eq!(VDIVS[14], (50, 1));
}
