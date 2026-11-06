// Counterpart in sigrok:
//   libsigrok/src/hardware/atorch/protocol.c,
//   Copyright (C) 2023 Mathieu Pilato <pilato.mathieu@free.fr>,
//   with commits by Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

use super::*;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, RecvTimeoutError, sync_channel};

/// How long a test waits before it fails: a stop or a limit that does not bite
/// must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(20);

const QUICK: Pace = Pace {
    probe: Duration::from_millis(50),
    wait: Duration::from_millis(1),
    grace: Duration::from_millis(40),
    silence: Duration::from_millis(60),
};
/// Limits and bounds that no test here should reach by waiting.
const NEVER: Duration = Duration::from_secs(3600);
/// For all but the tests of a bound: a stall must not end a capture early.
const CALM: Pace = Pace { grace: NEVER, silence: NEVER, ..QUICK };

/// What the fake holds and saw. A byte goes from `coming` to `input` on the read
/// its step names, whatever the time; a purge drops `input` alone, as a purge
/// drops only what has arrived.
#[derive(Default)]
struct State {
    /// A step: from this read on, these bytes have arrived.
    coming: VecDeque<(u64, Vec<u8>)>,
    input: VecDeque<u8>,
    /// Bytes a read gives at most: 0 for all, 1 for a byte-paced line.
    per_read: usize,
    /// Each read that finds nothing brings this: for a stream with no end.
    endless: Option<Vec<u8>>,
    reads: u64,
    purges: u32,
    writes: u32,
    fail_read: bool,
    /// Reads fail from this read on.
    fail_from: Option<u64>,
    /// Set by the read that gives bytes: a stop that lands inside a read.
    stop_on_read: Option<Arc<AtomicBool>>,
}

/// The port the device holds, and through `state` the test.
#[derive(Clone, Default)]
struct Fake {
    state: Arc<Mutex<State>>,
}

impl Fake {
    fn with(&self, f: impl FnOnce(&mut State)) {
        f(&mut self.state.lock().unwrap_or_else(std::sync::PoisonError::into_inner));
    }

    fn get<T>(&self, f: impl FnOnce(&State) -> T) -> T {
        f(&self.state.lock().unwrap_or_else(std::sync::PoisonError::into_inner))
    }

    /// These bytes come in with the next read, past an open's purge.
    fn send(&self, bytes: &[u8]) {
        self.send_after(1, bytes);
    }

    /// These bytes have arrived already, for a purge to drop.
    fn arrived(&self, bytes: &[u8]) {
        self.with(|s| s.input.extend(bytes));
    }

    /// These bytes have arrived by the time `reads` more reads have gone.
    fn send_after(&self, reads: u64, bytes: &[u8]) {
        self.with(|s| {
            let at = s.reads + reads;
            s.coming.push_back((at, bytes.to_vec()));
        });
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
        let got = {
            let mut guard = self.state.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
            let s = &mut *guard;
            s.reads += 1;
            if s.fail_read || s.fail_from.is_some_and(|at| s.reads >= at) {
                return Err(Error::device("the port went"));
            }
            while s.coming.front().is_some_and(|(at, _)| *at <= s.reads) {
                let (_, bytes) = s.coming.pop_front().expect("looked");
                s.input.extend(bytes);
            }
            if s.input.is_empty()
                && let Some(bytes) = s.endless.clone()
            {
                s.input.extend(bytes);
            }
            let room = if s.per_read > 0 { buf.len().min(s.per_read) } else { buf.len() };
            let n = room.min(s.input.len());
            for slot in buf.iter_mut().take(n) {
                *slot = s.input.pop_front().expect("counted");
            }
            if n > 0
                && let Some(stop) = &s.stop_on_read
            {
                stop.store(true, Ordering::SeqCst);
            }
            n
        };
        if got == 0 {
            std::thread::sleep(timeout);
        }
        Ok(got)
    }
    fn write(&mut self, _: &[u8]) -> Result<()> {
        self.with(|s| s.writes += 1);
        Ok(())
    }
    fn purge(&mut self) -> Result<()> {
        self.with(|s| {
            s.purges += 1;
            s.input.clear();
        });
        Ok(())
    }
}

/// A DC report, whose energy register says which it is: channel 3 reads 10 times
/// that.
fn report(n: u32) -> Vec<u8> {
    let mut r = vec![0u8; protocol::REPORT_LEN];
    r[..4].copy_from_slice(&[0xff, 0x55, 0x01, 0x02]);
    r[4..7].copy_from_slice(&[0x00, 0x01, 0x1a]);
    r[13..17].copy_from_slice(&n.to_be_bytes());
    r
}

fn dc() -> &'static Profile {
    protocol::find_profile(2).unwrap()
}

fn device(fake: &Fake, pace: Pace) -> AtorchDevice {
    let mut d = AtorchDevice::from_port(dc(), Box::new(fake.clone()), pace);
    d.open().unwrap();
    d
}

/// A capture with a queue that takes everything.
fn capture(d: &mut AtorchDevice) -> Receiver<(usize, Packet)> {
    let (tx, rx) = sync_channel(65536);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// Everything up to the End, or a failure: an overall bound, as a capture that
/// goes on keeps a per-packet wait from firing.
fn until_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        let packet = rx.recv_timeout(left).expect("the capture ended in time").1;
        if matches!(packet, Packet::End) {
            return out;
        }
        out.push(packet);
        assert!(out.len() < 60000, "a capture that does not end");
    }
}

/// The reports among `packets`, by their numbers.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn numbers(packets: &[Packet]) -> Vec<u32> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) if a.channels == [3] => Some((a.samples[0] / 10.0) as u32),
            _ => None,
        })
        .collect()
}

/// `n` frames' numbers, or a failure, with the capture still running.
fn frames(rx: &Receiver<(usize, Packet)>, n: usize) -> Vec<u32> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    while out.len() < n {
        let left = deadline.saturating_duration_since(Instant::now());
        let packet = rx.recv_timeout(left).expect("the frame came in time").1;
        assert!(!matches!(packet, Packet::End), "ended after {out:?}");
        out.extend(numbers(std::slice::from_ref(&packet)));
    }
    out
}

/// Runs `f` on a thread of its own: false if it took past the bound.
fn within_bound(f: impl FnOnce() + Send + 'static) -> bool {
    let (tx, rx) = sync_channel(1);
    std::thread::spawn(move || {
        f();
        let _ = tx.send(());
    });
    !matches!(rx.recv_timeout(BOUND), Err(RecvTimeoutError::Timeout))
}

#[test]
fn the_times_are_these() {
    let pace = Pace::default();
    assert_eq!(pace.probe, Duration::from_secs(10));
    assert_eq!(pace.grace, Duration::from_secs(3));
    assert_eq!(pace.silence, Duration::from_secs(30));
    assert_eq!(END_AFTER_STOP, Duration::from_secs(1));
}

/// A probe on its own thread, with a bound.
fn run_probe(fake: &Fake, pace: Pace) -> Option<&'static str> {
    let (tx, rx) = sync_channel(1);
    let mut port = fake.clone();
    std::thread::spawn(move || {
        let _ = tx.send(probe(&mut port, &pace).map(|p| p.name).ok());
    });
    rx.recv_timeout(BOUND).expect("the probe ended")
}

const SLOW_PROBE: Pace = Pace { probe: NEVER, ..QUICK };

#[test]
fn a_probe_goes_by_the_type_in_the_first_report() {
    let fake = Fake::default();
    fake.send(&report(1));
    assert_eq!(run_probe(&fake, SLOW_PROBE), Some("DC Meter"));
    let fake = Fake::default();
    let mut usb = report(1);
    usb[3] = 3;
    fake.send(&usb);
    assert_eq!(run_probe(&fake, SLOW_PROBE), Some("USB Meter"));
    assert_eq!(fake.get(|s| s.writes), 0, "it only reads");
}

#[test]
fn a_probe_reads_a_report_a_byte_at_a_time_and_takes_all_of_a_read() {
    let fake = Fake::default();
    fake.with(|s| s.per_read = 1);
    fake.send(&report(1));
    assert_eq!(run_probe(&fake, SLOW_PROBE), Some("DC Meter"));

    // C finds nothing here: see "scan a reply and a report in one read".
    let fake = Fake::default();
    fake.send(&[0xff, 0x55, 0x02, 1, 1, 0, 0, 0x40]);
    fake.send(&report(1));
    assert_eq!(run_probe(&fake, SLOW_PROBE), Some("DC Meter"));
}

#[test]
fn a_probe_passes_over_what_is_no_report() {
    let fake = Fake::default();
    fake.send(&[0xaa, 0xbb, 0xff, 0x55, 0x77]);
    fake.send(&[0xff, 0x55, 0x11, 2, 3, 0, 0, 0, 0, 0x52]);
    fake.send_after(5, &report(1));
    assert_eq!(run_probe(&fake, SLOW_PROBE), Some("DC Meter"));
}

#[test]
fn a_probe_stops_at_an_unknown_type_and_does_not_read_on() {
    let fake = Fake::default();
    // An AC report, then a good one a read later: the first decides.
    let mut ac = report(1);
    ac[3] = 1;
    fake.send(&[0xaa, 0xbb]);
    fake.send(&ac);
    fake.send_after(3, &report(2));
    assert_eq!(run_probe(&fake, SLOW_PROBE), None);
    assert_eq!(fake.get(|s| s.reads), 1);
    assert_eq!(fake.get(|s| s.coming.len()), 1);
}

#[test]
fn the_unknown_type_is_the_reports_own_with_noise_ahead() {
    // C says `buf[3]`, here 0x55.
    let fake = Fake::default();
    let mut ac = report(1);
    ac[3] = 1;
    fake.send(&[&[0xaa, 0xbb][..], &ac[..]].concat());
    let mut port = fake.clone();
    let end = probe(&mut port, &SLOW_PROBE).map(|p| p.name);
    assert_eq!(end, Err(ProbeEnd::UnknownType(1)));
    assert_eq!(unknown_type_message(1), "Unrecognized device type (0x0001).");
    assert_eq!(unknown_type_message(0xab), "Unrecognized device type (0x00ab).");
}

#[test]
fn a_probe_takes_a_report_behind_a_reply_though_the_next_read_fails() {
    let fake = Fake::default();
    fake.with(|s| s.fail_from = Some(2));
    fake.send(&[&[0xff, 0x55, 0x02, 1, 1, 0, 0, 0x40][..], &report(1)[..]].concat());
    assert_eq!(run_probe(&fake, SLOW_PROBE), Some("DC Meter"));
}

#[test]
fn a_silent_probe_goes_on_to_its_time_and_ends_there() {
    let fake = Fake::default();
    let started = Instant::now();
    assert_eq!(run_probe(&fake, QUICK), None);
    assert!(started.elapsed() >= QUICK.probe, "{:?}", started.elapsed());
}

#[test]
fn a_probe_that_finds_only_replies_ends_too() {
    let fake = Fake::default();
    fake.with(|s| s.endless = Some(vec![0xff, 0x55, 0x02, 1, 1, 0, 0, 0x40]));
    assert_eq!(run_probe(&fake, QUICK), None);
}

#[test]
fn a_read_that_fails_ends_a_probe_at_once() {
    let fake = Fake::default();
    fake.with(|s| s.fail_read = true);
    assert_eq!(run_probe(&fake, SLOW_PROBE), None);
    assert_eq!(fake.get(|s| s.reads), 1);
}

#[test]
fn a_frame_holds_the_channels_in_order() {
    let fake = Fake::default();
    fake.send(&report(7));
    let mut d = device(&fake, CALM);
    d.limits.frames.store(1, Ordering::SeqCst);
    let rx = capture(&mut d);
    let packets = until_end(&rx);
    assert!(matches!(packets[0], Packet::Header { .. }));
    assert!(matches!(packets[1], Packet::FrameBegin));
    assert!(matches!(packets[packets.len() - 1], Packet::FrameEnd));
    let analog: Vec<&AnalogData> = packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some(a),
            _ => None,
        })
        .collect();
    assert_eq!(analog.len(), 5);
    for (i, a) in analog.iter().enumerate() {
        assert_eq!(a.channels, [i]);
        assert_eq!(a.samples.len(), 1);
    }
    assert_eq!(analog[3].digits, -2);
    assert_eq!(numbers(&packets), [7]);
    d.close().unwrap();
    assert_eq!(fake.get(|s| s.writes), 0, "it only reads");
}

#[test]
fn only_the_channels_that_are_on_are_sent() {
    let fake = Fake::default();
    fake.send(&report(7));
    let mut d = device(&fake, CALM);
    for i in [0, 1, 2, 4] {
        d.enable_channel(i, false).unwrap();
    }
    d.limits.frames.store(1, Ordering::SeqCst);
    let rx = capture(&mut d);
    let packets = until_end(&rx);
    let analog = packets.iter().filter(|p| matches!(p, Packet::Analog(_))).count();
    assert_eq!(analog, 1);
    assert_eq!(numbers(&packets), [7]);
    d.close().unwrap();
}

#[test]
fn a_frame_limit_is_the_end_even_in_the_middle_of_a_read() {
    let fake = Fake::default();
    fake.send(&[report(1), report(2), report(3)].concat());
    let mut d = device(&fake, CALM);
    d.limits.frames.store(2, Ordering::SeqCst);
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [1, 2]);
    d.close().unwrap();
}

#[test]
fn all_of_a_read_is_taken_with_nothing_more_to_come() {
    let fake = Fake::default();
    fake.send(&[report(1), report(2)].concat());
    let mut d = device(&fake, CALM);
    d.limits.frames.store(2, Ordering::SeqCst);
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [1, 2]);
    d.close().unwrap();
}

#[test]
fn a_byte_paced_stream_with_noise_gives_every_report() {
    let fake = Fake::default();
    fake.with(|s| s.per_read = 1);
    let mut stream = Vec::new();
    for n in 1..=5 {
        stream.extend_from_slice(&[0xaa, 0xff]);
        stream.extend_from_slice(&report(n));
        stream.extend_from_slice(&[0xff, 0x55, 0x02, 1, 1, 0, 0, 0x40]);
    }
    fake.send(&stream);
    let mut d = device(&fake, CALM);
    d.limits.frames.store(5, Ordering::SeqCst);
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [1, 2, 3, 4, 5]);
    d.close().unwrap();
}

#[test]
fn a_time_limit_ends_at_the_first_report_past_it() {
    let fake = Fake::default();
    let mut d = device(&fake, Pace { grace: NEVER, ..QUICK });
    d.limits.msec.store(1, Ordering::SeqCst);
    // 5 empty reads wait 1 ms each, at least: the limit is past. The second report
    // comes in the same read.
    fake.send_after(6, &[report(1), report(2)].concat());
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [1]);
    d.close().unwrap();
}

#[test]
fn no_report_past_a_time_limit_ends_it_after_the_grace_and_not_before() {
    let fake = Fake::default();
    let mut d = device(&fake, Pace { silence: Duration::ZERO, ..QUICK });
    d.limits.msec.store(10, Ordering::SeqCst);
    let started = Instant::now();
    let rx = capture(&mut d);
    let packets = until_end(&rx);
    assert!(started.elapsed() >= Duration::from_millis(10) + QUICK.grace);
    assert_eq!(numbers(&packets), Vec::<u32>::new());
    d.close().unwrap();
}

#[test]
fn what_ends_a_quiet_capture() {
    const MS: Duration = Duration::from_millis(1);
    let p = Pace {
        probe: NEVER,
        wait: MS,
        grace: Duration::from_millis(100),
        silence: Duration::from_millis(500),
    };
    // A time limit: 200 ms and the grace.
    assert_eq!(overdue(&p, 0, 200, MS * 300, MS * 300), None, "at the bound");
    assert_eq!(overdue(&p, 0, 200, MS * 301, MS * 301), Some(Overdue::Time));
    assert_eq!(overdue(&p, 0, 200, MS * 299, MS * 299), None);
    // From the start, not from the last report.
    assert_eq!(overdue(&p, 0, 200, MS * 301, MS), Some(Overdue::Time));
    // With a frame limit too, the silence does not count: a time limit above it.
    assert_eq!(overdue(&p, 5, 60_000, MS * 900, MS * 900), None);
    assert_eq!(overdue(&p, 5, 60_000, MS * 60_101, MS), Some(Overdue::Time));
    // A frame limit alone: from the last report.
    assert_eq!(overdue(&p, 5, 0, MS * 900, MS * 499), None);
    assert_eq!(overdue(&p, 5, 0, MS * 900, MS * 500), Some(Overdue::Silence));
    // Neither: only the user.
    assert_eq!(overdue(&p, 0, 0, NEVER, NEVER), None);
    // A limit at its top does not wrap.
    assert_eq!(overdue(&p, 0, u64::MAX, NEVER, NEVER), None);
}

#[test]
fn a_frame_limit_alone_ends_on_silence_with_what_it_has() {
    let fake = Fake::default();
    fake.send(&report(1));
    let mut d = device(&fake, QUICK);
    d.limits.frames.store(5, Ordering::SeqCst);
    let started = Instant::now();
    let rx = capture(&mut d);
    let packets = until_end(&rx);
    assert_eq!(numbers(&packets), [1]);
    assert!(started.elapsed() >= QUICK.silence);
    d.close().unwrap();
}

#[test]
fn no_limit_waits_for_the_user_past_any_bound() {
    let fake = Fake::default();
    let mut d = device(&fake, QUICK);
    let rx = capture(&mut d);
    // Wait until reads show that all the bounds have gone by.
    let started = Instant::now();
    while started.elapsed() < QUICK.silence + QUICK.grace + QUICK.grace {
        std::thread::sleep(Duration::from_millis(5));
    }
    let packets: Vec<Packet> = rx.try_iter().map(|(_, p)| p).collect();
    assert_eq!(packets.len(), 1, "the header and no End");
    d.stop().unwrap();
    until_end(&rx);
    d.close().unwrap();
}

#[test]
fn a_stop_ends_a_quiet_capture() {
    let fake = Fake::default();
    let mut d = device(&fake, CALM);
    d.limits.frames.store(5, Ordering::SeqCst);
    d.limits.msec.store(3_600_000, Ordering::SeqCst);
    let rx = capture(&mut d);
    // A read has gone: it is in its loop.
    let deadline = Instant::now() + BOUND;
    while fake.get(|s| s.reads) == 0 {
        assert!(Instant::now() < deadline, "no read");
        std::thread::sleep(Duration::from_millis(1));
    }
    d.stop().unwrap();
    until_end(&rx);
    d.close().unwrap();
}

#[test]
fn a_stop_ends_a_capture_that_gets_reports_all_the_time() {
    let fake = Fake::default();
    fake.with(|s| s.endless = Some(report(9)));
    let mut d = device(&fake, CALM);
    let rx = capture(&mut d);
    assert_eq!(frames(&rx, 3), [9, 9, 9]);
    d.stop().unwrap();
    until_end(&rx);
    d.close().unwrap();
}

#[test]
fn a_stop_in_the_middle_of_a_read_sends_no_more_frames() {
    let fake = Fake::default();
    let stop = Arc::new(AtomicBool::new(false));
    fake.with(|s| s.stop_on_read = Some(Arc::clone(&stop)));
    fake.send(&[report(1), report(2)].concat());
    let capture =
        Capture { profile: dc(), enabled: vec![true; 5], limits: Arc::default(), pace: CALM };
    let (tx, rx) = sync_channel(64);
    let sink = PacketSink::new(tx, 0);
    let mut port = fake.clone();
    assert!(within_bound(move || {
        let _ = capture.run(&mut port, &sink, &stop);
    }));
    let packets: Vec<Packet> = rx.try_iter().map(|(_, p)| p).collect();
    assert_eq!(numbers(&packets), Vec::<u32>::new());
}

#[test]
fn a_close_does_not_hang_on_any_send_with_a_queue_nobody_reads() {
    // The header, a frame's begin, 5 readings and its end, then the next frame:
    // whichever send the queue's size stops at.
    for size in [1, 3, 7, 8] {
        let fake = Fake::default();
        fake.send(&[report(1), report(2)].concat());
        let mut d = device(&fake, CALM);
        let (tx, rx) = sync_channel(size);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        while fake.get(|s| s.reads) == 0 {
            assert!(Instant::now() < deadline, "no read");
            std::thread::sleep(Duration::from_millis(1));
        }
        // Time to reach the send that blocks: a stall only makes this test weaker.
        std::thread::sleep(Duration::from_millis(30));
        assert!(
            within_bound(move || {
                let _ = d.close();
            }),
            "queue of {size}"
        );
        drop(rx);
    }
}

#[test]
fn a_close_has_ended_the_capture_when_it_returns() {
    // A long wait in a read, so that the thread is in it when the close comes.
    let fake = Fake::default();
    let pace = Pace { grace: NEVER, silence: NEVER, wait: Duration::from_millis(200), ..QUICK };
    let mut d = device(&fake, pace);
    let rx = capture(&mut d);
    let deadline = Instant::now() + BOUND;
    while fake.get(|s| s.reads) == 0 {
        assert!(Instant::now() < deadline, "no read");
        std::thread::sleep(Duration::from_millis(1));
    }
    assert!(within_bound(move || {
        let _ = d.close();
    }));
    // The device holds no port, and no thread holds one: only this `fake` is left.
    assert_eq!(Arc::strong_count(&fake.state), 1);
    drop(rx);
}

#[test]
fn a_report_starts_the_silence_over() {
    let silence = Duration::from_millis(200);
    let fake = Fake::default();
    let mut d = device(&fake, Pace { grace: NEVER, silence, ..QUICK });
    d.limits.frames.store(5, Ordering::SeqCst);
    let started = Instant::now();
    let rx = capture(&mut d);
    while started.elapsed() < silence * 3 / 5 {
        std::thread::sleep(Duration::from_millis(1));
    }
    let sent = Instant::now();
    fake.send(&report(1));
    let packets = until_end(&rx);
    // A stall may end it ahead of the report, which says nothing. With the report
    // taken, any speed gives this.
    if numbers(&packets) == [1] {
        assert!(sent.elapsed() >= silence, "{:?}", sent.elapsed());
    }
    d.close().unwrap();
}

/// A queue left unread that holds what is sent up to the End and no more.
fn stuck_capture(fake: &Fake) -> (AtorchDevice, Receiver<(usize, Packet)>) {
    let mut d = device(fake, CALM);
    d.limits.frames.store(1, Ordering::SeqCst);
    fake.send(&report(1));
    // The header, a frame's begin, 5 readings and its end: 8.
    let (tx, rx) = sync_channel(8);
    d.start(PacketSink::new(tx, 0)).unwrap();
    // Port back, End stuck.
    let deadline = Instant::now() + BOUND;
    while d.port.lock().unwrap().is_none() {
        assert!(Instant::now() < deadline, "the port did not come back ahead of the End");
        std::thread::sleep(Duration::from_millis(1));
    }
    (d, rx)
}

#[test]
fn a_close_with_an_end_nobody_reads_does_not_hang() {
    let fake = Fake::default();
    let (mut d, rx) = stuck_capture(&fake);
    assert!(within_bound(move || {
        let _ = d.close();
    }));
    drop(rx);
}

#[test]
fn a_start_with_an_end_nobody_reads_does_not_hang() {
    let fake = Fake::default();
    let (mut d, old_rx) = stuck_capture(&fake);
    fake.send_after(2, &report(2));
    let (done_tx, done_rx) = sync_channel(1);
    std::thread::spawn(move || {
        let rx = capture(&mut d);
        let got = numbers(&until_end(&rx));
        let _ = d.close();
        let _ = done_tx.send(got);
    });
    assert_eq!(done_rx.recv_timeout(BOUND).expect("it started"), [2]);
    drop(old_rx);
}

#[test]
fn a_start_while_one_runs_is_busy_and_a_start_after_works() {
    let fake = Fake::default();
    let mut d = device(&fake, CALM);
    let rx = capture(&mut d);
    let (tx, _rx2) = sync_channel(4);
    assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
    d.stop().unwrap();
    until_end(&rx);
    // As soon as the End is seen.
    let rx = capture(&mut d);
    d.stop().unwrap();
    until_end(&rx);
    d.close().unwrap();
}

#[test]
fn a_start_on_a_closed_device_says_so() {
    let fake = Fake::default();
    let mut d = AtorchDevice::from_port(dc(), Box::new(fake), QUICK);
    let (tx, _rx) = sync_channel(4);
    assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Device(_))));
}

#[test]
fn the_first_capture_keeps_what_came_since_the_open_a_second_does_not() {
    let fake = Fake::default();
    let mut d = device(&fake, CALM);
    let opened = fake.get(|s| s.purges);
    assert_eq!(opened, 1, "the open's");
    d.limits.frames.store(1, Ordering::SeqCst);
    // Came between the open and the start: C has it too.
    fake.arrived(&report(1));
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [1]);
    assert_eq!(fake.get(|s| s.purges), opened);
    // Gathered while no capture ran, and old by the next one.
    fake.arrived(&[report(2), report(3)].concat());
    fake.send_after(2, &report(4));
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [4]);
    assert_eq!(fake.get(|s| s.purges), opened + 1);
    d.close().unwrap();
}

#[test]
fn a_new_open_has_a_first_capture_again() {
    let fake = Fake::default();
    let mut d = device(&fake, CALM);
    d.limits.frames.store(1, Ordering::SeqCst);
    fake.send(&report(1));
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [1]);
    d.close().unwrap();
    d.pending = Some(Box::new(fake.clone()));
    d.open().unwrap();
    let opened = fake.get(|s| s.purges);
    fake.arrived(&report(2));
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [2]);
    assert_eq!(fake.get(|s| s.purges), opened);
    d.close().unwrap();
}

#[test]
fn a_capture_cut_inside_a_report_leaves_nothing_to_the_next() {
    let fake = Fake::default();
    let mut d = device(&fake, CALM);
    d.limits.frames.store(1, Ordering::SeqCst);
    // A report and the first half of the next, in one read.
    let second = report(2);
    fake.send(&[&report(1)[..], &second[..18]].concat());
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [1]);
    // The tail is on its way past any purge. It is noise, and the report after it
    // is read true.
    fake.send_after(2, &[&second[18..], &report(3)[..]].concat());
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [3]);
    d.close().unwrap();
}

#[test]
fn a_capture_cut_inside_a_read_does_not_shift_a_byte_paced_next_one() {
    let fake = Fake::default();
    fake.with(|s| s.per_read = 1);
    let mut d = device(&fake, CALM);
    d.limits.frames.store(2, Ordering::SeqCst);
    let mut stream: Vec<u8> = Vec::new();
    for n in 1..=9 {
        stream.extend_from_slice(&report(n));
    }
    // What stays queued at the end of the first capture goes by the purge; the
    // reports have numbers in order, so a shifted one would not be 3, 4 or later.
    fake.send(&stream[..20 + 2 * protocol::REPORT_LEN]);
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [1, 2]);
    fake.send_after(2, &stream[3 * protocol::REPORT_LEN..]);
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [4, 5]);
    d.close().unwrap();
}

#[test]
fn a_limit_set_during_a_capture_counts() {
    let fake = Fake::default();
    fake.send(&[report(1), report(2)].concat());
    let mut d = device(&fake, CALM);
    let rx = capture(&mut d);
    assert_eq!(frames(&rx, 2), [1, 2]);
    d.set(&Key::LimitFrames, &Value::U64(3), None).unwrap();
    assert!(matches!(d.get(&Key::LimitFrames, None), Ok(Value::U64(3))));
    fake.send(&[report(3), report(4)].concat());
    assert_eq!(numbers(&until_end(&rx)), [3]);
    d.close().unwrap();
}

#[test]
fn a_read_that_fails_ends_a_capture_and_gives_the_port_back() {
    let fake = Fake::default();
    fake.send(&report(1));
    let mut d = device(&fake, CALM);
    d.limits.frames.store(5, Ordering::SeqCst);
    let rx = capture(&mut d);
    assert_eq!(frames(&rx, 1), [1]);
    fake.with(|s| s.fail_read = true);
    until_end(&rx);
    assert!(d.port.lock().unwrap().is_some());
    d.close().unwrap();
}

#[test]
fn a_capture_takes_a_report_behind_a_reply_though_the_next_read_fails() {
    let fake = Fake::default();
    fake.with(|s| s.fail_from = Some(2));
    fake.send(&[&[0xff, 0x55, 0x02, 1, 1, 0, 0, 0x40][..], &report(1)[..]].concat());
    let mut d = device(&fake, CALM);
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), [1]);
    d.close().unwrap();
}

#[test]
fn a_full_stream_of_what_is_no_report_does_not_reset_the_silence() {
    let fake = Fake::default();
    fake.with(|s| s.endless = Some(vec![0xff, 0x55, 0x02, 1, 1, 0, 0, 0x40]));
    let mut d = device(&fake, QUICK);
    d.limits.frames.store(1, Ordering::SeqCst);
    let rx = capture(&mut d);
    assert_eq!(numbers(&until_end(&rx)), Vec::<u32>::new());
    d.close().unwrap();
}

#[test]
fn limits_read_back_and_a_zero_time_is_refused() {
    let fake = Fake::default();
    let mut d = device(&fake, CALM);
    assert!(matches!(d.get(&Key::LimitFrames, None), Ok(Value::U64(0))));
    assert!(matches!(d.get(&Key::LimitMsec, None), Ok(Value::U64(0))));
    d.set(&Key::LimitMsec, &Value::U64(500), None).unwrap();
    assert!(matches!(d.get(&Key::LimitMsec, None), Ok(Value::U64(500))));
    assert!(d.set(&Key::LimitMsec, &Value::U64(0), None).is_err());
    assert!(matches!(d.get(&Key::LimitMsec, None), Ok(Value::U64(500))));
    d.set(&Key::LimitFrames, &Value::U64(0), None).unwrap();
    assert!(d.get(&Key::Samplerate, None).is_err());
    assert!(d.set(&Key::Samplerate, &Value::U64(1), None).is_err());
    assert!(d.list(&Key::LimitFrames, None).is_err());
    assert!(d.get(&Key::LimitFrames, Some("x")).is_err());
    d.close().unwrap();
}

#[test]
fn a_scan_needs_a_conn_and_takes_an_empty_one_for_none() {
    assert!(AtorchDriver.scan(&[]).unwrap().is_empty());
    let empty = [(Key::Conn, Value::Str(String::new()))];
    assert!(AtorchDriver.scan(&empty).unwrap().is_empty());
}

#[test]
fn the_device_says_what_it_is() {
    let fake = Fake::default();
    let d = AtorchDevice::from_port(protocol::find_profile(3).unwrap(), Box::new(fake), QUICK);
    assert_eq!(d.info().vendor, "Atorch");
    assert_eq!(d.info().model, "USB Meter");
    assert_eq!(d.info().conn, None);
    let names: Vec<&str> = d.channels().iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["V", "I", "C", "E", "D-", "D+", "T"]);
    assert_eq!(d.classes().len(), 3);
    assert_eq!(AtorchDriver.long_name(), "atorch meters and loads");
}
