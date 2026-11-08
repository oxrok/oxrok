// Counterpart in sigrok:
//   libsigrok/src/hardware/hameg-hmo/api.c,
//   libsigrok/src/hardware/hameg-hmo/protocol.c,
//   Copyright (C) 2013 poljar (Damir Jelić) <poljarinho@gmail.com>,
//   Copyright (C) 2018 Guido Trentalancia <guido@trentalancia.com>,
//   with commits by Uwe Hermann, Gerhard Sittig, Lars-Peter Clausen,
//   Bert Vermeulen, Soeren Apel, Martin Ling, Stefan Brüns, Aurelien Jacobs,
//   Daniel Elstner, Valentin Ochs, Mathias Katzer, Johannes Römer, xandinator
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

use super::protocol::{
    Head, capture_over, channels_conflict, floats, format_e, keep_pattern, parse_head,
    parse_rational, table_index, table_value, too_slow, trigger_position, trigger_seconds,
};
use super::*;
use crate::scpi::{HwInfo, ScpiTransport};
use crate::serial::WriteLog;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test waits for what must happen: a stop or a limit that does not bite must
/// fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);
/// Bounds no test of anything else should meet, even when a host stalls.
const FAR: Bounds = Bounds {
    first: Duration::from_secs(600),
    warn: Duration::from_secs(600),
    limit: Duration::from_secs(600),
    junk: Duration::from_secs(600),
};

const fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

/// What the line gives next.
#[derive(Clone, Debug)]
enum Piece {
    Bytes(Vec<u8>),
    /// A read finds nothing, whatever it waits for: lateness with no clock.
    Gap,
    /// As [`Piece::Gap`], until the test lets go.
    Hold,
}

fn text(s: &str) -> Vec<Piece> {
    vec![Piece::Bytes(format!("{s}\n").into_bytes())]
}

fn blocked(data: &[u8]) -> Vec<u8> {
    let mut out = format!("#{}{}", data.len().to_string().len(), data.len()).into_bytes();
    out.extend_from_slice(data);
    out.push(b'\n');
    out
}

fn float_block(values: &[f32]) -> Vec<Piece> {
    let data: Vec<u8> = values.iter().flat_map(|v| v.to_le_bytes()).collect();
    vec![Piece::Bytes(blocked(&data))]
}

type Script = Box<dyn FnMut(&str) -> Option<Vec<Piece>> + Send>;

#[derive(Default)]
struct Wire {
    sent: WriteLog,
    coming: VecDeque<Piece>,
    closes: u32,
    reopens: u32,
    read: bool,
    released: bool,
    complete: bool,
}

/// A scope on a line with no tags. It answers in pieces, and what a read doesn't find
/// it finds next time: the line's time is its reads, so a loaded machine slows both.
struct Line {
    wire: Arc<Mutex<Wire>>,
    script: Script,
}

/// What every scope here says, unless a test's script says otherwise.
#[allow(clippy::match_same_arms)]
fn usual(line: &str) -> Vec<Piece> {
    let last = line.rsplit(';').next().unwrap_or(line);
    match last {
        "*OPC?" => text("1"),
        ":TIM:SCAL?" => text("1.000E-03"),
        ":TIM:DIV?" => text("12"),
        ":TIM:POS?" => text("0.000E+00"),
        ":TRIG:A:SOUR?" => text("CH1"),
        ":TRIG:A:EDGE:SLOP?" => text("POS"),
        ":TRIG:A:PATT:SOUR?" => text("\"XX\""),
        ":ACQ:HRES?" | ":ACQ:PEAK?" => text("OFF"),
        ":ACQ:SRAT?" => text("1.000E+06"),
        ":CHAN1:STAT?" => text("1"),
        _ if last.ends_with(":STAT?") => text("0"),
        _ if last.ends_with(":SCAL?") => text("1.000E+00"),
        _ if last.ends_with(":POS?") => text("0.000E+00"),
        _ if last.ends_with(":COUP?") => text("DCL"),
        _ if last.ends_with(":UNIT?") => text("V"),
        _ if last.ends_with(":THR?") && last.starts_with(":POD") => text("TTL"),
        _ if last.ends_with(":TECH?") => text("TTL"),
        _ if last.starts_with(":CHAN") && last.ends_with(":DATA?") => {
            float_block(&[1.5, -2.5, 0.25])
        }
        _ if last.ends_with(":DATA?") => vec![Piece::Bytes(blocked(&[1, 2, 3, 4]))],
        _ => Vec::new(),
    }
}

impl ScpiTransport for Line {
    #[allow(clippy::unnecessary_literal_bound)]
    fn resource(&self) -> &str {
        "fake"
    }

    fn send(&mut self, data: &[u8]) -> crate::core::Result<()> {
        let line = String::from_utf8_lossy(data).trim_end().to_owned();
        let answer = (self.script)(&line).unwrap_or_else(|| usual(&line));
        let mut wire = self.wire.lock().unwrap();
        wire.sent.push((Instant::now(), data.to_vec()));
        wire.coming.extend(answer);
        Ok(())
    }

    fn read_begin(&mut self) -> crate::core::Result<()> {
        self.wire.lock().unwrap().complete = false;
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], _timeout: Duration) -> crate::core::Result<usize> {
        // Not at full speed, as a read would wait.
        std::thread::sleep(ms(1));
        let mut wire = self.wire.lock().unwrap();
        wire.read = true;
        match wire.coming.pop_front() {
            None | Some(Piece::Gap) => Ok(0),
            Some(Piece::Hold) => {
                if !wire.released {
                    wire.coming.push_front(Piece::Hold);
                }
                Ok(0)
            }
            Some(Piece::Bytes(bytes)) => {
                let n = buf.len().min(bytes.len());
                buf[..n].copy_from_slice(&bytes[..n]);
                if n < bytes.len() {
                    wire.coming.push_front(Piece::Bytes(bytes[n..].to_vec()));
                } else {
                    wire.complete = true;
                }
                Ok(n)
            }
        }
    }

    fn read_complete(&self) -> bool {
        self.wire.lock().unwrap().complete
    }

    fn discard(&mut self, _timeout: Duration) -> crate::core::Result<Option<usize>> {
        Ok(Some(0))
    }

    fn close(&mut self) -> crate::core::Result<()> {
        self.wire.lock().unwrap().closes += 1;
        Ok(())
    }

    fn reopen(&mut self) -> crate::core::Result<()> {
        let mut wire = self.wire.lock().unwrap();
        wire.reopens += 1;
        // A new connection can't receive an old answer.
        wire.coming.clear();
        Ok(())
    }
}

fn scope(
    name: &str,
    script: impl FnMut(&str) -> Option<Vec<Piece>> + Send + 'static,
) -> (HamegDevice, Arc<Mutex<Wire>>) {
    let wire = Arc::new(Mutex::new(Wire::default()));
    let line = Line { wire: Arc::clone(&wire), script: Box::new(script) };
    let mut scpi = Scpi::new(Box::new(line));
    scpi.opc_silence = ms(40);
    let id = HwInfo {
        manufacturer: "HAMEG".to_owned(),
        model: name.to_owned(),
        serial_number: "1".to_owned(),
        firmware_version: "2".to_owned(),
    };
    let mut d = HamegDevice::new(models::find(name).unwrap(), &id, scpi);
    d.bounds = Some(FAR);
    (d, wire)
}

fn plain(name: &str) -> (HamegDevice, Arc<Mutex<Wire>>) {
    scope(name, |_| None)
}

fn sent(wire: &Arc<Mutex<Wire>>) -> Vec<String> {
    let wire = wire.lock().unwrap();
    wire.sent.iter().map(|(_, d)| String::from_utf8_lossy(d).trim_end().to_owned()).collect()
}

fn sent_after(wire: &Arc<Mutex<Wire>>, from: usize) -> Vec<String> {
    sent(wire).split_off(from)
}

fn start(d: &mut HamegDevice) -> Receiver<(usize, Packet)> {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// What a capture sent, up to its End, which must come within [`BOUND`].
fn drain(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let began = Instant::now();
    let mut out = Vec::new();
    while began.elapsed() < BOUND && out.len() < 100_000 {
        match rx.recv_timeout(BOUND) {
            Ok((_, Packet::End)) => return out,
            Ok((_, p)) => out.push(p),
            Err(e) => panic!("no End: {e}"),
        }
    }
    panic!("no End in {} packets", out.len());
}

/// A test that could hang fails instead.
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
        Err(e) => panic!("the test hung: {e}"),
    }
}

/// Waits for a condition, with a bound.
fn until(what: &str, mut done: impl FnMut() -> bool) {
    let began = Instant::now();
    while !done() {
        assert!(began.elapsed() < BOUND, "never: {what}");
        std::thread::sleep(ms(1));
    }
}

fn frames_in(packets: &[Packet]) -> usize {
    packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count()
}

fn failure(packets: &[Packet]) -> Option<&str> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(why) => Some(why.as_str()),
        _ => None,
    })
}

// The pure parts.

#[test]
fn numbers_go_out_as_g_ascii_formatd_writes_them() {
    assert_eq!(format_e(1.5), "1.500000E+00");
    assert_eq!(format_e(1e-3), "1.000000E-03");
    assert_eq!(format_e(-2.5e-3), "-2.500000E-03");
    assert_eq!(format_e(0.0), "0.000000E+00");
    assert_eq!(format_e(-0.0), "-0.000000E+00");
    assert_eq!(format_e(1e100), "1.000000E+100");
    assert_eq!(format_e(f64::NAN), "NAN");
    assert_eq!(format_e(f64::INFINITY), "INF");
    assert_eq!(format_e(f64::NEG_INFINITY), "-INF");
    assert_eq!(table_value((1, 1_000_000_000)), "1.000000E-09");
    assert_eq!(table_value((500, 1_000_000)), "5.000000E-04");
    assert_eq!(table_value((50, 1)), "5.000000E+01");
}

#[test]
fn rationals_read_as_sr_parse_rational_reads_them() {
    for (s, want) in [
        ("2.000E-03", Some((2000, 1_000_000))),
        ("1", Some((1, 1))),
        ("-1.5", Some((-15, 10))),
        ("+.5", Some((5, 10))),
        ("5.", Some((5, 1))),
        ("1E+02", Some((100, 1))),
        ("1e2", Some((100, 1))),
        ("  1", Some((1, 1))),
        ("", None),
        (".", None),
        ("E3", None),
        ("1E", None),
        ("1 ", None),
        ("volts", None),
        ("1.0.0", None),
        ("0x10", None),
        ("99999999999999999999", None),
        ("1E+99", None),
    ] {
        assert_eq!(parse_rational(s), want, "{s}");
    }
}

#[test]
fn an_answer_finds_its_entry_by_value() {
    let compact = models::find("HMO1024").unwrap().timebases;
    assert_eq!(table_index("2.000E-09", compact), Some(0));
    assert_eq!(table_index("1.000E-09", compact), None, "no 1 ns");
    assert_eq!(table_index("5.0E+01", compact), Some(31));
    assert_eq!(table_index("0.002", compact), Some(table_index("2E-3", compact).unwrap()));
    assert_eq!(table_index("3.000E-03", compact), None);
    assert_eq!(table_index("-1E-03", compact), None);
    let full = models::find("HMO3054").unwrap().timebases;
    assert_eq!(table_index("1.000E-09", full), Some(0));
}

#[test]
fn a_frame_ends_a_capture_as_c_says_but_for_the_frames_alone() {
    assert!(capture_over(1, 0, 1000, 0), "no limit: one frame");
    assert!(capture_over(1, 1, 1000, 0));
    assert!(!capture_over(1, 3, 1000, 0), "--frames 3 alone, where C stops");
    assert!(!capture_over(2, 3, 1000, 0));
    assert!(capture_over(3, 3, 1000, 0));
    assert!(capture_over(1, 0, 1000, 5000), "a limit above a frame still ends at one");
    assert!(!capture_over(1, 2, 1000, 5000));
    assert!(capture_over(2, 2, 1000, 5000));
    assert!(capture_over(1, 2, 1000, 1000), "the limit is met");
    assert!(capture_over(1, 2, 1000, 999));
    assert!(!capture_over(1, 2, 999, 1000));
}

#[test]
fn a_blocks_head_says_what_it_can() {
    assert_eq!(parse_head(b""), Head::More);
    assert_eq!(parse_head(b"#"), Head::More);
    assert_eq!(parse_head(b"#3"), Head::More);
    assert_eq!(parse_head(b"#31"), Head::More);
    assert_eq!(parse_head(b"#3100"), Head::Block { head: 5, len: 100 });
    assert_eq!(parse_head(b"#10x"), Head::Block { head: 3, len: 0 });
    assert_eq!(parse_head(b"#9000000016"), Head::Block { head: 11, len: 16 });
    assert_eq!(parse_head(b"#0abc"), Head::Indefinite);
    assert_eq!(parse_head(b"#x"), Head::Not);
    assert_eq!(parse_head(b"#3x"), Head::Not);
    assert_eq!(parse_head(b"#31x0"), Head::Not);
    assert_eq!(parse_head(b"#3-12"), Head::Not, "C's sr_atol takes it");
    assert_eq!(parse_head(b"NOT"), Head::Not);
}

/// Every single bit flip and every lost byte of a block's head: what still reads as a
/// block with another length is what the format can't catch, and its count is reported.
#[test]
fn damage_to_a_blocks_head() {
    let good = b"#41000";
    let (mut same, mut other_length, mut refused) = (0, 0, 0);
    let mut look = |damaged: &[u8]| match parse_head(damaged) {
        Head::Block { head: 6, len: 1000 } => same += 1,
        Head::Block { .. } => other_length += 1,
        _ => refused += 1,
    };
    for i in 0..good.len() {
        for bit in 0..8 {
            let mut d = good.to_vec();
            d[i] ^= 1 << bit;
            look(&d);
        }
        let mut d = good.to_vec();
        d.remove(i);
        look(&d);
    }
    assert_eq!(same, 0);
    // A flip to another digit, in the digits: the wrong length, which a read then finds
    // short or long. Nothing in the format says otherwise.
    assert_eq!((other_length, refused), (17, 37));
}

#[test]
fn a_crawling_block_ends_and_a_long_one_does_not() {
    let limit = Duration::from_secs(600);
    assert!(!too_slow(Duration::from_secs(599), 0, limit));
    assert!(too_slow(Duration::from_secs(600), 0, limit));
    assert!(too_slow(Duration::from_secs(600), 599_999, limit));
    assert!(!too_slow(Duration::from_secs(600), 600_000, limit), "1 kB/s");
    assert!(!too_slow(Duration::from_secs(8000), 96_000_000, limit), "a long record at 115200");
}

#[test]
fn bounds_take_the_screen_for_a_pace() {
    let fast = Bounds::for_sweep(ms(12));
    assert_eq!(fast.first, Duration::from_secs(30));
    assert_eq!(fast.warn, Duration::from_secs(30));
    assert_eq!(fast.limit, Duration::from_secs(600));
    assert_eq!(fast.junk, Duration::from_secs(5));
    let slow = Bounds::for_sweep(Duration::from_secs(600));
    assert_eq!(slow.first.as_secs(), 1200);
    assert_eq!(slow.limit.as_secs(), 1200);
}

const TEST_BOUNDS: Bounds = Bounds {
    first: Duration::from_secs(30),
    warn: Duration::from_secs(60),
    limit: Duration::from_secs(600),
    junk: Duration::from_secs(5),
};

/// Looks every second from `from` to `to` seconds, and says the first that isn't Wait.
fn walk(wait: &mut Wait, t0: Instant, from: u64, to: u64, junk: bool) -> Option<(u64, Verdict)> {
    for s in from..=to {
        let v = wait.look(t0 + Duration::from_secs(s), junk);
        if v != Verdict::Wait {
            return Some((s, v));
        }
    }
    None
}

#[test]
fn silence_with_no_block_fails_at_its_bound_and_not_before() {
    let t0 = Instant::now();
    let mut w = Wait::new(TEST_BOUNDS, t0);
    assert_eq!(
        walk(&mut w, t0, 1, 40, false),
        Some((30, Verdict::Fail("The scope has sent no block in 30 s.".to_owned())))
    );
}

#[test]
fn silence_after_a_block_warns_once_and_fails_at_the_long_limit() {
    let t0 = Instant::now();
    let mut w = Wait::new(TEST_BOUNDS, t0);
    assert!(!w.block());
    w.asked(t0);
    assert_eq!(walk(&mut w, t0, 1, 59, false), None, "well past the first bound");
    let warned = "The scope has sent no block for 60 s; waiting up to 600 s.".to_owned();
    assert_eq!(walk(&mut w, t0, 60, 60, false), Some((60, Verdict::Warn(warned))));
    assert_eq!(
        walk(&mut w, t0, 61, 700, false),
        Some((600, Verdict::Fail("The scope has sent no block for 600 s.".to_owned())))
    );
}

#[test]
fn a_block_says_the_wait_is_over_when_it_had_warned() {
    let t0 = Instant::now();
    let mut w = Wait::new(TEST_BOUNDS, t0);
    w.block();
    w.asked(t0);
    assert!(matches!(walk(&mut w, t0, 1, 60, false), Some((60, Verdict::Warn(_)))));
    assert!(w.block());
    assert!(!w.block(), "said once");
}

#[test]
fn each_request_starts_its_own_wait() {
    let t0 = Instant::now();
    let mut w = Wait::new(TEST_BOUNDS, t0);
    for n in 0..10u64 {
        let at = t0 + Duration::from_secs(n * 25);
        w.asked(at);
        assert_eq!(w.look(at + Duration::from_secs(1), false), Verdict::Wait);
        assert_eq!(w.look(at + Duration::from_secs(2), false), Verdict::Wait);
    }
}

#[test]
fn junk_fails_sooner_and_from_when_it_came() {
    let t0 = Instant::now();
    let mut w = Wait::new(TEST_BOUNDS, t0);
    assert_eq!(walk(&mut w, t0, 1, 10, false), None);
    let why = "No usable answer from the scope in 5 s: what it sent is no block.".to_owned();
    assert_eq!(w.look(t0 + Duration::from_secs(11), true), Verdict::Wait);
    // Silence after the junk doesn't clear it.
    assert_eq!(walk(&mut w, t0, 12, 15, false), None);
    assert_eq!(walk(&mut w, t0, 16, 16, false), Some((16, Verdict::Fail(why))));
}

#[test]
fn a_stall_counts_for_nothing_up_to_its_credit() {
    let t0 = Instant::now();
    let mut w = Wait::new(TEST_BOUNDS, t0);
    // Stopped for 29 s at 29 s: the look that follows says nothing.
    assert_eq!(walk(&mut w, t0, 1, 29, false), None);
    assert_eq!(w.look(t0 + Duration::from_secs(58), false), Verdict::Wait);
    assert_eq!(walk(&mut w, t0, 59, 58, false), None);
    // 29 s of silence seen so far, so one more second ends it.
    assert_eq!(
        w.look(t0 + Duration::from_secs(59), false),
        Verdict::Fail("The scope has sent no block in 30 s.".to_owned())
    );
    // Stopped again and again: the credit runs out, at 60 s, and so does the wait.
    let mut w = Wait::new(TEST_BOUNDS, t0);
    let mut at = 0;
    let verdict = loop {
        at += 10;
        let v = w.look(t0 + Duration::from_secs(at), false);
        if v != Verdict::Wait {
            break v;
        }
        assert!(at < 1000, "never ended");
    };
    assert!(matches!(verdict, Verdict::Fail(_)));
    assert!(at <= 100, "ended at {at} s");
}

#[test]
fn a_store_folds_pods_in_and_keeps_to_the_first_length() {
    let mut s = LogicStore::default();
    s.queue(0, 2, &[1, 2, 3]);
    s.queue(1, 2, &[4, 5, 6, 7, 8]);
    assert_eq!(s.take(2, 0).unwrap(), [1, 4, 2, 5, 3, 6], "longer: cut, where C writes past");
    assert_eq!(s.take(2, 0), None, "it was taken");
    s.queue(1, 2, &[9, 8]);
    assert_eq!(s.take(2, 0).unwrap(), [0, 9, 0, 8], "pod 2 alone");
    s.queue(0, 2, &[1, 2, 3]);
    s.queue(1, 2, &[4]);
    assert_eq!(s.take(2, 0).unwrap(), [1, 4, 2, 0, 3, 0], "shorter: zeros");
    s.queue(0, 2, &[1, 2, 3]);
    s.queue(1, 2, &[4, 5, 6]);
    assert_eq!(s.take(2, 2).unwrap(), [1, 4, 2, 5], "a limit counts samples");
}

#[test]
fn trigger_arithmetic_is_c_s() {
    let place = trigger_position(3e-3, 1e-3, 12);
    assert_eq!(place.to_bits(), 0.25f32.to_bits());
    assert_eq!(trigger_position(0.0, 1e-3, 12), 0.5);
    assert_eq!(trigger_seconds(0.25, 1e-3, 12), 3e-3);
    assert_eq!(format_e(trigger_seconds(0.5, 1e-3, 12)), "-0.000000E+00", "C sends a -0");
    assert!(!trigger_position(1.0, 1e-3, 0).is_finite(), "no divisions, no panic");
    assert!(trigger_position(0.0, 1e-3, 0).is_nan());
}

#[test]
fn analog_and_pods_conflict_by_input() {
    assert!(channels_conflict(&[false, false, true, false], &[true, false]));
    assert!(channels_conflict(&[false, false, false, true], &[false, true]));
    assert!(!channels_conflict(&[false, false, true, false], &[false, true]));
    assert!(!channels_conflict(&[false, false, false, true], &[true, false]));
    assert!(!channels_conflict(&[true, true], &[true]), "two channels, one pod");
}

#[test]
fn a_block_is_little_endian_floats_and_what_is_over_goes() {
    assert_eq!(floats(&[0, 0, 0x80, 0x3f, 0, 0, 0, 0xc0, 9]), [1.0, -2.0]);
    assert_eq!(floats(&[1, 2]), Vec::<f32>::new());
}

#[test]
fn a_pattern_is_kept_to_its_room() {
    assert_eq!(keep_pattern("10X"), "10X");
    assert_eq!(keep_pattern(&"1".repeat(25)), "1".repeat(20));
    assert_eq!(keep_pattern(&format!("{}é", "1".repeat(19))), "1".repeat(19), "a whole character");
}

// The device, on the fake line.

#[test]
fn an_open_asks_in_cs_order() {
    let (mut d, wire) = plain("HMO1002");
    d.open().unwrap();
    let want = [
        ":CHAN1:STAT?",
        ":CHAN1:SCAL?",
        ":CHAN1:POS?",
        ":CHAN1:COUP?",
        ":PROB1:SET:ATT:UNIT?",
        ":CHAN2:STAT?",
        ":CHAN2:SCAL?",
        ":CHAN2:POS?",
        ":CHAN2:COUP?",
        ":PROB2:SET:ATT:UNIT?",
        ":LOG0:STAT?",
        ":LOG1:STAT?",
        ":LOG2:STAT?",
        ":LOG3:STAT?",
        ":LOG4:STAT?",
        ":LOG5:STAT?",
        ":LOG6:STAT?",
        ":LOG7:STAT?",
        ":POD1:STAT?",
        ":POD1:THR?",
        ":TIM:SCAL?",
        ":TIM:DIV?",
        ":TIM:POS?",
        ":TRIG:A:SOUR?",
        ":TRIG:A:EDGE:SLOP?",
        ":TRIG:A:PATT:SOUR?",
        ":ACQ:HRES?",
        ":ACQ:PEAK?",
        ":ACQ:SRAT?",
    ];
    assert_eq!(sent(&wire), want);
    let on: Vec<&str> =
        d.channels().iter().filter(|c| c.enabled).map(|c| c.name.as_str()).collect();
    assert_eq!(on, ["CH1"], "as the scope says, with all enabled at the scan");
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(1_000_000));
    assert_eq!(d.get(&Key::TriggerPattern, None).unwrap(), Value::Str("XX".to_owned()));
    assert_eq!(d.get(&Key::HorizTriggerpos, None).unwrap(), Value::F64(0.5));
    assert_eq!(d.get(&Key::NumHdiv, None).unwrap(), Value::I32(12));
}

#[test]
fn the_second_dialect_numbers_its_thresholds_by_channel() {
    let (mut d, wire) = scope("RTB2004", |q| match q {
        ":DIG8:TECH?" => Some(text("MAN")),
        ":DIG8:THR?" => Some(text("2.500E+00")),
        _ => None,
    });
    d.open().unwrap();
    let asked = sent(&wire);
    for q in [":LOG1:STAT?", ":DIG0:TECH?", ":LOG2:STAT?", ":DIG8:TECH?", ":DIG8:THR?"] {
        assert!(asked.iter().any(|a| a == q), "{q}");
    }
    assert!(!asked.iter().any(|a| a == ":DIG0:THR?"), "TTL has no level");
    let group = Some("POD2");
    assert_eq!(d.get(&Key::LogicThreshold, group).unwrap(), Value::Str("MAN".to_owned()));
    assert_eq!(d.get(&Key::LogicThresholdCustom, group).unwrap(), Value::F64(2.5));
    assert!(d.get(&Key::LogicThresholdCustom, Some("POD1")).is_err(), "not in custom mode");
}

#[test]
fn a_threshold_nobody_knows_is_asked_for_twice() {
    let (mut d, wire) = scope("HMO1002", |q| match q {
        ":POD1:THR?" => Some(text("BOGUS")),
        _ => None,
    });
    assert!(d.open().is_err());
    assert_eq!(sent(&wire).iter().filter(|q| *q == ":POD1:THR?").count(), 2);
}

#[test]
fn user_is_user1_at_the_second_asking() {
    let (mut d, wire) = scope("HMO1002", |q| match q {
        ":POD1:THR?" => Some(text("USER")),
        ":POD1:THR:UDL1?" => Some(text("3.000E+00")),
        _ => None,
    });
    d.open().unwrap();
    assert!(sent(&wire).iter().any(|q| q == ":POD1:THR:UDL1?"));
    assert_eq!(d.get(&Key::LogicThreshold, Some("POD1")).unwrap(), Value::Str("USER1".to_owned()));
}

#[test]
fn a_compact_scope_says_the_timebase_it_has() {
    let (mut d, _) = scope("HMO1024", |q| match q {
        ":TIM:SCAL?" => Some(text("1.000E-03")),
        ":TIM:POS?" => Some(text("3.000E-03")),
        _ => None,
    });
    d.open().unwrap();
    assert_eq!(d.get(&Key::Timebase, None).unwrap(), Value::Rational(1, 1000), "C says 2 ms");
    assert_eq!(d.get(&Key::HorizTriggerpos, None).unwrap(), Value::F64(0.25));
    let (mut d, _) = scope("HMO1024", |q| (q == ":TIM:SCAL?").then(|| text("5.000E+01")));
    d.open().unwrap();
    assert_eq!(d.get(&Key::Timebase, None).unwrap(), Value::Rational(50, 1), "past C's table");
    let (mut d, _) = scope("HMO1024", |q| (q == ":TIM:SCAL?").then(|| text("1.000E-09")));
    let e = d.open().unwrap_err().to_string();
    assert!(e.contains("no such time base here: '1.000E-09'"), "{e}");
    let (mut d, _) = scope("HMO1002", |q| (q == ":TIM:SCAL?").then(|| text("1.000E-09")));
    d.open().unwrap();
    assert_eq!(d.get(&Key::Timebase, None).unwrap(), Value::Rational(1, 1_000_000_000));
}

#[test]
fn answers_that_are_no_numbers_refuse_the_open() {
    for (q, a) in [
        (":CHAN1:SCAL?", "volts"),
        (":CHAN1:POS?", "x"),
        (":TIM:SCAL?", "fast"),
        (":TIM:DIV?", "twelve"),
        (":TIM:POS?", ""),
        (":ACQ:SRAT?", "1.0.0"),
        (":CHAN1:STAT?", "maybe"),
    ] {
        let (mut d, _) = scope("HMO1002", move |line| (line == q).then(|| text(a)));
        assert!(d.open().is_err(), "{q} {a}");
    }
}

#[test]
fn a_capture_asks_for_each_channel_and_a_pods_first_only() {
    let (mut d, wire) = plain("HMO3054");
    d.open().unwrap();
    let from = sent(&wire).len();
    for name in ["CH1", "CH4", "D0", "D3", "D15"] {
        let pos = d.channels().iter().position(|c| c.name == name).unwrap();
        d.enable_channel(pos, true).unwrap();
    }
    d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
    assert!(d.start(sink_for_test()).is_err(), "CH4 and pod 2 share an input");
    assert_eq!(sent_after(&wire, from), Vec::<String>::new(), "refused with nothing sent");
    let pos = d.channels().iter().position(|c| c.name == "CH4").unwrap();
    d.enable_channel(pos, false).unwrap();
    bounded_capture(d, move |d, packets| {
        let _ = d;
        assert_eq!(frames_in(packets), 1);
    });
    let writes = sent_after(&wire, from);
    assert_eq!(writes[..4], [":LOG0:STAT 1", ":LOG3:STAT 1", ":LOG15:STAT 1", ":POD1:STAT 1"]);
    assert_eq!(writes[4..6], [":POD2:STAT 1", ":ACQ:SRAT?"], "the rate is read again");
    assert_eq!(
        writes[6..],
        [
            ":FORM:BORD LSBF;:FORM REAL,32;:CHAN1:DATA?",
            ":FORM UINT,8;:POD1:DATA?",
            ":FORM UINT,8;:POD2:DATA?"
        ]
    );
}

fn sink_for_test() -> PacketSink {
    let (tx, rx) = sync_channel(1);
    std::mem::forget(rx);
    PacketSink::new(tx, 0)
}

/// Runs a capture on its own thread with a bound, then `check`s its packets.
fn bounded_capture(
    mut d: HamegDevice,
    check: impl FnOnce(&mut HamegDevice, &[Packet]) + Send + 'static,
) {
    bounded(move || {
        let rx = start(&mut d);
        let packets = drain(&rx);
        check(&mut d, &packets);
        d.close().unwrap();
    });
}

#[test]
fn a_frame_is_as_c_meant_it() {
    let (mut d, _) = plain("HMO1002");
    d.open().unwrap();
    bounded_capture(d, |_, p| {
        assert!(matches!(p[0], Packet::Header { .. }));
        assert!(matches!(p[1], Packet::FrameBegin));
        let Packet::Analog(a) = &p[2] else { panic!("{:?}", p[2]) };
        assert_eq!(a.channels, [0]);
        assert_eq!(a.samples, [1.5, -2.5, 0.25], "what C's sr_analog_init clears");
        assert_eq!((a.mq, a.unit, a.digits), (Mq::Voltage, Unit::Volt, 2));
        assert!(matches!(p[3], Packet::FrameEnd));
        assert_eq!(p.len(), 4);
    });
}

#[test]
fn a_probe_in_amperes_says_so() {
    let (mut d, _) = scope("HMO1002", |q| (q == ":PROB1:SET:ATT:UNIT?").then(|| text("AMP")));
    d.open().unwrap();
    bounded_capture(d, |_, p| {
        let Packet::Analog(a) = &p[2] else { panic!("{:?}", p[2]) };
        assert_eq!((a.mq, a.unit), (Mq::Current, Unit::Ampere));
    });
}

fn only(d: &mut HamegDevice, names: &[&str]) {
    for pos in 0..d.channels().len() {
        let on = names.contains(&d.channels()[pos].name.as_str());
        d.enable_channel(pos, on).unwrap();
    }
}

fn logic_of(p: &Packet) -> Option<(usize, Vec<u8>)> {
    match p {
        Packet::Logic(l) => Some((l.unit_size, l.data.clone())),
        _ => None,
    }
}

#[test]
fn pods_come_in_units_of_how_far_they_reach() {
    let (mut d, _) = plain("HMO3054");
    d.open().unwrap();
    only(&mut d, &["D0"]);
    bounded_capture(d, |_, p| {
        let logic: Vec<_> = p.iter().filter_map(logic_of).collect();
        assert_eq!(logic, [(1, vec![1, 2, 3, 4])]);
    });
    let (mut d, _) = plain("HMO3054");
    d.open().unwrap();
    only(&mut d, &["D8"]);
    bounded_capture(d, |_, p| {
        let logic: Vec<_> = p.iter().filter_map(logic_of).collect();
        assert_eq!(logic, [(2, vec![0, 1, 0, 2, 0, 3, 0, 4])], "pod 2 alone");
    });
    let (mut d, _) = plain("HMO3054");
    d.open().unwrap();
    only(&mut d, &["D0", "D8"]);
    d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    bounded_capture(d, |_, p| {
        let logic: Vec<_> = p.iter().filter_map(logic_of).collect();
        assert_eq!(logic, [(2, vec![1, 1, 2, 2])]);
        assert!(matches!(p.last(), Some(Packet::FrameEnd)), "the logic comes ahead of it");
    });
}

#[test]
fn one_pod_keeps_to_a_limit_as_well() {
    let (mut d, _) = plain("HMO3054");
    d.open().unwrap();
    only(&mut d, &["D0"]);
    d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    bounded_capture(d, |_, p| {
        let logic: Vec<_> = p.iter().filter_map(logic_of).collect();
        assert_eq!(logic, [(1, vec![1, 2])]);
    });
}

#[test]
fn the_acquisition_modes_are_on_unless_they_say_off() {
    let (mut d, _) = scope("HMO1002", |q| (q == ":ACQ:HRES?").then(|| text("AUTO")));
    d.open().unwrap();
    assert_eq!(d.get(&Key::HighResolution, None).unwrap(), Value::Bool(true));
    assert_eq!(d.get(&Key::PeakDetection, None).unwrap(), Value::Bool(false));
    let (mut d, _) = scope("HMO1002", |q| (q == ":ACQ:PEAK?").then(|| text("ON")));
    d.open().unwrap();
    assert_eq!(d.get(&Key::HighResolution, None).unwrap(), Value::Bool(false));
    assert_eq!(d.get(&Key::PeakDetection, None).unwrap(), Value::Bool(true));
}

/// With the link's lock held, the End can't come unless it comes ahead of the link.
#[test]
fn the_link_is_back_before_the_end() {
    let (mut d, _) = plain("HMO1002");
    d.open().unwrap();
    bounded(move || {
        let (tx, rx) = sync_channel(1);
        let slot = Arc::clone(&d.scpi);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let held = slot.lock().unwrap();
        loop {
            let (_, p) = rx.recv_timeout(BOUND).expect("no FrameEnd");
            if matches!(p, Packet::FrameEnd) {
                break;
            }
        }
        assert!(rx.recv_timeout(ms(100)).is_err(), "an End with the link still out");
        drop(held);
        assert!(matches!(rx.recv_timeout(BOUND).expect("no End").1, Packet::End));
        d.close().unwrap();
    });
}

#[test]
fn limits_end_a_capture_as_the_rule_says() {
    let count = |frames: u64, samples: u64| {
        let (mut d, _) = plain("HMO1002");
        d.open().unwrap();
        d.set(&Key::LimitFrames, &Value::U64(frames), None).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(samples), None).unwrap();
        let rx = start(&mut d);
        let packets = drain(&rx);
        d.close().unwrap();
        let samples: Vec<usize> = packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some(a.samples.len()),
                _ => None,
            })
            .collect();
        (frames_in(&packets), samples)
    };
    bounded(move || {
        assert_eq!(count(3, 0), (3, vec![3, 3, 3]));
        assert_eq!(count(0, 0), (1, vec![3]));
        assert_eq!(count(0, 2), (1, vec![2]));
        assert_eq!(count(3, 2), (1, vec![2]));
        assert_eq!(count(2, 100), (2, vec![3, 3]));
    });
}

/// A real scope's block comes in pieces, with gaps in it shorter than a read's wait.
#[test]
fn a_block_in_pieces_comes_whole() {
    let (mut d, _) = scope("HMO1002", |q| {
        q.ends_with(":CHAN1:DATA?").then(|| {
            let Piece::Bytes(all) = float_block(&[1.0, 2.0, 3.0]).remove(0) else { unreachable!() };
            let mut pieces: Vec<Piece> = Vec::new();
            for b in all {
                pieces.push(Piece::Bytes(vec![b]));
            }
            pieces
        })
    });
    d.open().unwrap();
    d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    bounded_capture(d, |_, p| {
        let Packet::Analog(a) = &p[2] else { panic!("{:?}", p[2]) };
        assert_eq!(a.samples, [1.0], "--samples 1 isn't a sample of a frame cut");
        assert!(matches!(p.last(), Some(Packet::FrameEnd)));
    });
}

#[test]
fn a_line_end_left_ahead_of_a_block_is_passed_over() {
    let (mut d, _) = scope("HMO1002", |q| {
        q.ends_with(":CHAN1:DATA?").then(|| {
            let Piece::Bytes(block) = float_block(&[7.0]).remove(0) else { unreachable!() };
            let mut all = b"\r\n".to_vec();
            all.extend_from_slice(&block);
            vec![Piece::Bytes(all), Piece::Hold]
        })
    });
    d.open().unwrap();
    d.bounds = Some(Bounds { junk: ms(60), ..FAR });
    bounded_capture(d, |_, p| {
        assert_eq!(failure(p), None);
        let Packet::Analog(a) = &p[2] else { panic!("{:?}", p[2]) };
        assert_eq!(a.samples, [7.0]);
    });
}

/// A late block is taken with no second request, and one frame begins, where C sends a
/// frame begin at each try.
#[test]
fn a_late_block_lands_in_its_own_frame() {
    let (mut d, wire) = scope("HMO1002", |q| {
        q.ends_with(":CHAN1:DATA?").then(|| {
            let mut out = vec![Piece::Gap, Piece::Gap, Piece::Gap];
            out.extend(float_block(&[9.0]));
            out
        })
    });
    d.open().unwrap();
    let from = sent(&wire).len();
    bounded(move || {
        let rx = start(&mut d);
        let p = drain(&rx);
        assert_eq!(frames_in(&p), 1);
        assert!(failure(&p).is_none(), "{p:?}");
        let requests = sent_after(&wire, from).iter().filter(|q| q.ends_with("DATA?")).count();
        assert_eq!(requests, 1);
        d.close().unwrap();
    });
}

#[test]
fn a_silent_scope_fails_a_capture_with_no_block() {
    let (mut d, _) = scope("HMO1002", |q| q.ends_with("DATA?").then(|| vec![Piece::Hold]));
    d.open().unwrap();
    d.bounds = Some(Bounds { first: ms(60), ..FAR });
    bounded_capture(d, |_, p| {
        assert_eq!(failure(p), Some("device error: The scope has sent no block in 0 s."));
        assert!(!p.iter().any(|x| matches!(x, Packet::Analog(_))));
    });
}

#[test]
fn after_a_block_silence_warns_and_goes_on() {
    let reads = Arc::new(Mutex::new(0));
    let (mut d, wire) = scope("HMO1002", move |q| {
        q.ends_with("DATA?").then(|| {
            let mut n = reads.lock().unwrap();
            *n += 1;
            if *n == 2 { vec![Piece::Hold] } else { float_block(&[1.0]) }
        })
    });
    d.open().unwrap();
    // A margin for slow runners: the first block must be read before the first bound.
    d.bounds =
        Some(Bounds { first: ms(500), warn: ms(500), limit: ms(600_000), junk: ms(600_000) });
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    bounded(move || {
        let rx = start(&mut d);
        // Well past the first bound, with the scope silent.
        until("the second request", || {
            sent(&wire).iter().filter(|q| q.ends_with("DATA?")).count() >= 2
        });
        std::thread::sleep(ms(800));
        assert!(!d.worker.as_ref().unwrap().handle.is_finished(), "it ended at the first bound");
        {
            let mut w = wire.lock().unwrap();
            w.released = true;
            w.coming.clear();
            w.coming.extend(float_block(&[2.0]));
        }
        let p = drain(&rx);
        assert_eq!(failure(&p), None, "{p:?}");
        assert_eq!(frames_in(&p), 2);
        d.close().unwrap();
    });
}

#[test]
fn after_a_block_silence_ends_at_the_long_limit() {
    let reads = Arc::new(Mutex::new(0));
    let (mut d, _) = scope("HMO1002", move |q| {
        q.ends_with("DATA?").then(|| {
            let mut n = reads.lock().unwrap();
            *n += 1;
            if *n == 2 { vec![Piece::Hold] } else { float_block(&[1.0]) }
        })
    });
    d.open().unwrap();
    d.bounds = Some(Bounds { first: ms(600_000), warn: ms(5), limit: ms(60), junk: ms(600_000) });
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    bounded_capture(d, |_, p| {
        assert_eq!(frames_in(p), 2);
        assert_eq!(failure(p), Some("device error: The scope has sent no block for 0 s."));
    });
}

#[test]
fn text_for_a_block_is_a_quick_failure_in_its_own_words() {
    for junk in ["NOT A BLOCK\n", "#0xyz\n", "#3x00\n", "#", "#3"] {
        let (mut d, _) = scope("HMO1002", move |q| {
            q.ends_with("DATA?").then(|| vec![Piece::Bytes(junk.as_bytes().to_vec()), Piece::Hold])
        });
        d.open().unwrap();
        d.bounds = Some(Bounds { junk: ms(60), ..FAR });
        bounded_capture(d, move |_, p| {
            assert_eq!(
                failure(p),
                Some(
                    "device error: No usable answer from the scope in 0 s: what it sent is no block."
                ),
                "{junk}"
            );
        });
    }
}

#[test]
fn junk_then_a_block_is_a_block() {
    let (mut d, _) = scope("HMO1002", |q| {
        q.ends_with("DATA?").then(|| {
            let mut out = vec![Piece::Bytes(b"LATE ANSWER\n".to_vec()), Piece::Gap];
            out.extend(float_block(&[5.0]));
            out
        })
    });
    d.open().unwrap();
    bounded_capture(d, |_, p| {
        assert_eq!(failure(p), None);
        let Packet::Analog(a) = &p[2] else { panic!("{:?}", p[2]) };
        assert_eq!(a.samples, [5.0]);
    });
}

/// A block that pauses partway, for many reads' timeouts, is still one block.
#[test]
fn a_pause_inside_a_block_is_waited_out() {
    let (mut d, _) = scope("HMO1002", |q| {
        q.ends_with("DATA?").then(|| {
            let Piece::Bytes(all) = float_block(&[1.0, 2.0, 3.0, 4.0]).remove(0) else {
                unreachable!()
            };
            let mut out = vec![Piece::Bytes(all[..10].to_vec())];
            out.extend(std::iter::repeat_n(Piece::Gap, 50));
            out.push(Piece::Bytes(all[10..].to_vec()));
            out
        })
    });
    d.open().unwrap();
    bounded_capture(d, |_, p| {
        let Packet::Analog(a) = &p[2] else { panic!("{:?}", p[2]) };
        assert_eq!(a.samples, [1.0, 2.0, 3.0, 4.0]);
        assert_eq!(failure(p), None);
    });
}

/// The same on the clock: a pause of 300 ms inside a bound of 20 s, wide enough for a
/// sleep twelve times as long under the shim.
#[test]
fn a_pause_inside_a_block_on_the_clock() {
    let (mut d, wire) = scope("HMO1002", |q| {
        q.ends_with("DATA?").then(|| {
            let Piece::Bytes(all) = float_block(&[5.0, 6.0]).remove(0) else { unreachable!() };
            vec![Piece::Bytes(all[..6].to_vec()), Piece::Hold, Piece::Bytes(all[6..].to_vec())]
        })
    });
    d.open().unwrap();
    d.bounds = Some(Bounds { first: ms(20_000), ..FAR });
    bounded(move || {
        let rx = start(&mut d);
        until("the pause", || matches!(wire.lock().unwrap().coming.front(), Some(Piece::Hold)));
        std::thread::sleep(ms(300));
        wire.lock().unwrap().released = true;
        let p = drain(&rx);
        let Packet::Analog(a) = &p[2] else { panic!("{:?}", p[2]) };
        assert_eq!(a.samples, [5.0, 6.0]);
        d.close().unwrap();
    });
}

#[test]
fn a_block_cut_short_is_handed_on_and_the_next_stays_in_step() {
    let n = Arc::new(Mutex::new(0));
    let (mut d, _) = scope("HMO1002", move |q| {
        q.ends_with("DATA?").then(|| {
            let mut k = n.lock().unwrap();
            *k += 1;
            if *k == 1 {
                let mut cut = b"#212".to_vec();
                cut.extend_from_slice(&1.0f32.to_le_bytes());
                vec![Piece::Bytes(cut)]
            } else {
                float_block(&[2.0, 3.0])
            }
        })
    });
    d.open().unwrap();
    // How long a block may pause before what came is all there is.
    d.bounds = Some(Bounds { first: ms(100), ..FAR });
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    bounded_capture(d, |_, p| {
        let got: Vec<Vec<f32>> = p
            .iter()
            .filter_map(|x| match x {
                Packet::Analog(a) => Some(a.samples.clone()),
                _ => None,
            })
            .collect();
        assert_eq!(got, [vec![1.0], vec![2.0, 3.0]]);
        assert_eq!(failure(p), None);
    });
}

#[test]
fn an_empty_block_is_no_crash_and_no_packet() {
    let (mut d, _) =
        scope("HMO1002", |q| q.ends_with("DATA?").then(|| vec![Piece::Bytes(b"#10\n".to_vec())]));
    d.open().unwrap();
    bounded_capture(d, |_, p| {
        assert_eq!(failure(p), None);
        assert_eq!(frames_in(p), 1);
        assert!(!p.iter().any(|x| matches!(x, Packet::Analog(_))));
    });
}

/// A stop in the wait for a block ends the capture, and what is owed can't reach the
/// next one: it connects again.
#[test]
fn a_stop_in_a_wait_ends_it_and_the_next_use_connects_again() {
    let (mut d, wire) = scope("HMO1002", |q| q.ends_with("DATA?").then(|| vec![Piece::Hold]));
    d.open().unwrap();
    let reopens = wire.lock().unwrap().reopens;
    let from = sent(&wire).len();
    bounded(move || {
        let rx = start(&mut d);
        until("the request", || sent(&wire).len() > from);
        wire.lock().unwrap().read = false;
        until("a read", || wire.lock().unwrap().read);
        d.stop().unwrap();
        let p = drain(&rx);
        assert_eq!(failure(&p), None);
        assert_eq!(wire.lock().unwrap().reopens, reopens, "no wire at a stop");
        d.set(&Key::TriggerSource, &Value::Str("CH2".to_owned()), None).unwrap();
        assert_eq!(wire.lock().unwrap().reopens, reopens + 1);
        assert_eq!(d.get(&Key::TriggerSource, None).unwrap(), Value::Str("CH2".to_owned()));
        assert_eq!(wire.lock().unwrap().reopens, reopens + 1, "once");
        d.close().unwrap();
    });
}

/// A capture's answer all taken leaves nothing for the next: no reconnect.
#[test]
fn a_finished_capture_owes_nothing() {
    let (mut d, wire) = plain("HMO1002");
    d.open().unwrap();
    bounded(move || {
        let reopens = wire.lock().unwrap().reopens;
        let rx = start(&mut d);
        drain(&rx);
        let rx = start(&mut d);
        let p = drain(&rx);
        assert_eq!(frames_in(&p), 1, "a second capture on an open device");
        assert_eq!(wire.lock().unwrap().reopens, reopens);
        d.close().unwrap();
    });
}

#[test]
fn a_stop_between_a_crawling_blocks_bytes_ends_it() {
    let (mut d, wire) = scope("HMO1002", |q| {
        q.ends_with("DATA?").then(|| {
            let mut out = vec![Piece::Bytes(b"#3999".to_vec()), Piece::Bytes(vec![0; 4])];
            out.push(Piece::Hold);
            out
        })
    });
    d.open().unwrap();
    bounded(move || {
        let rx = start(&mut d);
        until("the head", || wire.lock().unwrap().coming.len() == 1);
        d.stop().unwrap();
        drain(&rx);
        assert!(d.worker.is_none());
        d.close().unwrap();
    });
}

#[test]
fn a_close_joins_and_nothing_is_written_after_it() {
    let (mut d, wire) = scope("HMO1002", |q| q.ends_with("DATA?").then(|| vec![Piece::Hold]));
    d.open().unwrap();
    bounded(move || {
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let from = sent(&wire).len();
        until("the request", || sent(&wire).iter().any(|q| q.ends_with("DATA?")));
        d.close().unwrap();
        let closes = wire.lock().unwrap().closes;
        assert!(closes >= 1, "closed");
        let count = sent(&wire).len();
        std::thread::sleep(ms(30));
        assert_eq!(sent(&wire).len(), count, "from {from}");
        drop(rx);
    });
}

/// With the queue full and nobody reading, a restart must give the old End up.
#[test]
fn a_start_after_a_capture_whose_end_is_stuck_does_not_hang() {
    let (mut d, _) = plain("HMO1002");
    d.open().unwrap();
    bounded(move || {
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The header fills the queue: all else waits on it, with the stop looked at.
        until("the header", || matches!(rx.try_recv(), Ok((_, Packet::Header { .. }))));
        until("a stuck capture", || d.capturing.load(Ordering::SeqCst) || d.worker.is_some());
        std::thread::sleep(ms(20));
        d.stop().unwrap();
        let rx2 = start(&mut d);
        let p = drain(&rx2);
        assert_eq!(frames_in(&p), 1);
        d.close().unwrap();
        drop(rx);
    });
}

#[test]
fn a_set_during_a_capture_is_refused_and_sends_nothing() {
    let (mut d, wire) = scope("HMO1002", |q| q.ends_with("DATA?").then(|| vec![Piece::Hold]));
    d.open().unwrap();
    bounded(move || {
        let rx = start(&mut d);
        until("the request", || sent(&wire).iter().any(|q| q.ends_with("DATA?")));
        let count = sent(&wire).len();
        let e = d.set(&Key::TriggerSlope, &Value::Str("NEG".to_owned()), None);
        assert!(matches!(e, Err(Error::Busy)), "{e:?}");
        assert_eq!(sent(&wire).len(), count);
        assert_eq!(d.get(&Key::TriggerSlope, None).unwrap(), Value::Str("POS".to_owned()));
        assert!(matches!(d.start(sink_for_test()), Err(Error::Busy)));
        // A limit is no command, and the running capture sees it, as C reads its limit
        // at each frame.
        d.set(&Key::LimitFrames, &Value::U64(9), None).unwrap();
        d.stop().unwrap();
        drain(&rx);
        d.close().unwrap();
    });
}

#[allow(clippy::needless_pass_by_value)]
fn set_ok(d: &mut HamegDevice, key: Key, value: Value, group: Option<&str>) {
    d.set(&key, &value, group).unwrap();
}

/// Every setting's words, literal, and what a get says after it.
#[test]
fn settings_are_spelled_as_c_spells_them() {
    let (mut d, wire) = plain("HMO3054");
    d.open().unwrap();
    let mut last = sent(&wire).len();
    let mut says = |want: &[&str]| {
        let new = sent_after(&wire, last);
        last += new.len();
        let want: Vec<String> = want.iter().map(|s| (*s).to_owned()).collect();
        assert_eq!(new, want);
    };
    set_ok(&mut d, Key::Timebase, Value::Rational(1, 500), None);
    says(&[":TIM:SCAL 2.000000E-03", "*OPC?", ":ACQ:SRAT?"]);
    set_ok(&mut d, Key::HorizTriggerpos, Value::F64(0.25), None);
    says(&[":TIM:POS 6.000000E-03", "*OPC?"]);
    assert_eq!(d.get(&Key::HorizTriggerpos, None).unwrap(), Value::F64(0.25));
    set_ok(&mut d, Key::TriggerSource, Value::Str("D15".to_owned()), None);
    says(&[":TRIG:A:SOUR D15", "*OPC?"]);
    set_ok(&mut d, Key::TriggerSlope, Value::Str("EITH".to_owned()), None);
    says(&[":TRIG:A:TYPE EDGE;:TRIG:A:EDGE:SLOP EITH", "*OPC?"]);
    set_ok(&mut d, Key::TriggerPattern, Value::Str("10101010101010101010".to_owned()), None);
    says(&[
        ":TRIG:A:TYPE LOGIC;:TRIG:A:PATT:FUNC AND;:TRIG:A:PATT:COND \"TRUE\";\
         :TRIG:A:PATT:MODE OFF;:TRIG:A:PATT:SOUR \"10101010101010101010\"",
        "*OPC?",
    ]);
    set_ok(&mut d, Key::HighResolution, Value::Bool(true), None);
    says(&[":ACQ:HRES AUTO", "*OPC?", ":ACQ:PEAK OFF", "*OPC?"]);
    set_ok(&mut d, Key::HighResolution, Value::Bool(false), None);
    says(&[":ACQ:HRES OFF", "*OPC?"]);
    set_ok(&mut d, Key::PeakDetection, Value::Bool(true), None);
    says(&[":ACQ:PEAK AUTO", "*OPC?", ":ACQ:HRES OFF", "*OPC?"]);
    assert_eq!(d.get(&Key::PeakDetection, None).unwrap(), Value::Bool(true));
    assert_eq!(d.get(&Key::HighResolution, None).unwrap(), Value::Bool(false));
    set_ok(&mut d, Key::Vdiv, Value::Rational(1, 2), Some("CH3"));
    says(&[":CHAN3:SCAL 5.000000E-01", "*OPC?"]);
    assert_eq!(d.get(&Key::Vdiv, Some("CH3")).unwrap(), Value::Rational(500, 1000));
    set_ok(&mut d, Key::Coupling, Value::Str("GND".to_owned()), Some("CH2"));
    says(&[":CHAN2:COUP GND", "*OPC?"]);
    set_ok(&mut d, Key::LogicThreshold, Value::Str("ECL".to_owned()), Some("POD2"));
    says(&[":POD2:THR ECL", "*OPC?"]);
    set_ok(&mut d, Key::LogicThresholdCustom, Value::F64(1.5), Some("POD2"));
    says(&[":POD2:THR:UDL2 1.500000E+00", "*OPC?", ":POD2:THR USER2", "*OPC?"]);
    // Where C's state stays at ECL, with the scope in USER2.
    let group = Some("POD2");
    assert_eq!(d.get(&Key::LogicThreshold, group).unwrap(), Value::Str("USER2".to_owned()));
    assert_eq!(d.get(&Key::LogicThresholdCustom, group).unwrap(), Value::F64(1.5));
}

#[test]
fn an_rth_gets_its_custom_threshold_where_c_dies() {
    let (mut d, wire) = plain("RTH1002");
    d.open().unwrap();
    let from = sent(&wire).len();
    set_ok(&mut d, Key::LogicThresholdCustom, Value::F64(2.5), Some("POD1"));
    assert_eq!(
        sent_after(&wire, from),
        [":DIG1:THR 2.500000E+00", "*OPC?", ":DIG1:TECH USER2", "*OPC?"]
    );
}

#[test]
fn a_custom_threshold_on_an_rtb_is_man_on_its_channel_number() {
    let (mut d, wire) = plain("RTB2002");
    d.open().unwrap();
    let from = sent(&wire).len();
    set_ok(&mut d, Key::LogicThresholdCustom, Value::F64(-1.5), Some("POD2"));
    assert_eq!(
        sent_after(&wire, from),
        [":DIG8:THR -1.500000E+00", "*OPC?", ":DIG8:TECH MAN", "*OPC?"]
    );
}

#[test]
fn what_is_out_of_range_sends_nothing_nan_too() {
    let (mut d, wire) = plain("HMO3054");
    d.open().unwrap();
    let count = sent(&wire).len();
    for place in [-0.1, 1.1, f64::NAN, f64::INFINITY] {
        assert!(d.set(&Key::HorizTriggerpos, &Value::F64(place), None).is_err(), "{place}");
    }
    for level in [-2.1, 8.1, f64::NAN] {
        let group = Some("POD1");
        assert!(d.set(&Key::LogicThresholdCustom, &Value::F64(level), group).is_err(), "{level}");
    }
    let empty = Value::Str(String::new());
    assert!(d.set(&Key::TriggerPattern, &empty, None).is_err());
    let long = Value::Str("1".repeat(21));
    assert!(d.set(&Key::TriggerPattern, &long, None).is_err());
    assert!(d.set(&Key::Vdiv, &Value::Rational(1, 1), None).is_err(), "no group");
    assert!(d.set(&Key::Vdiv, &Value::Rational(1, 1), Some("POD1")).is_err(), "a pod");
    assert!(d.set(&Key::Vdiv, &Value::Rational(20, 1), Some("CH1")).is_err(), "not in the table");
    assert!(d.set(&Key::Coupling, &Value::Str("DC".to_owned()), None).is_err());
    let group = Some("CH1");
    assert!(d.set(&Key::LogicThreshold, &Value::Str("TTL".to_owned()), group).is_err());
    assert!(d.set(&Key::Timebase, &Value::Rational(3, 1000), None).is_err());
    assert_eq!(sent(&wire).len(), count);
    set_ok(&mut d, Key::HorizTriggerpos, Value::F64(0.0), None);
    set_ok(&mut d, Key::HorizTriggerpos, Value::F64(1.0), None);
    set_ok(&mut d, Key::LogicThresholdCustom, Value::F64(-2.0), Some("POD1"));
    set_ok(&mut d, Key::LogicThresholdCustom, Value::F64(8.0), Some("POD1"));
}

/// `*OPC?` doesn't come, and the scope may have the setting: the error says nothing wrong
/// about it, the state keeps what it knew, and the next get asks.
#[test]
fn a_setting_with_no_answer_leaves_the_next_get_to_ask() {
    let silent = Arc::new(AtomicBool::new(false));
    let silent2 = Arc::clone(&silent);
    let taken = Arc::new(Mutex::new("POS"));
    let taken2 = Arc::clone(&taken);
    let (mut d, wire) = scope("HMO1002", move |q| {
        if q == "*OPC?" && silent2.load(Ordering::SeqCst) {
            return Some(Vec::new());
        }
        if q == ":TRIG:A:EDGE:SLOP?" {
            return Some(text(&taken2.lock().unwrap()));
        }
        if q.ends_with("SLOP NEG") {
            *taken2.lock().unwrap() = "NEG";
        }
        None
    });
    d.open().unwrap();
    silent.store(true, Ordering::SeqCst);
    let e = d.set(&Key::TriggerSlope, &Value::Str("NEG".to_owned()), None).unwrap_err();
    assert!(e.to_string().contains("no answer to *OPC?"), "{e}");
    silent.store(false, Ordering::SeqCst);
    let from = sent(&wire).len();
    assert_eq!(d.get(&Key::TriggerSlope, None).unwrap(), Value::Str("NEG".to_owned()));
    assert!(sent_after(&wire, from).len() > 20, "it read the state again");
    let from = sent(&wire).len();
    assert_eq!(d.get(&Key::TriggerSlope, None).unwrap(), Value::Str("NEG".to_owned()));
    assert_eq!(sent(&wire).len(), from, "once");
    let on: Vec<&str> =
        d.channels().iter().filter(|c| c.enabled).map(|c| c.name.as_str()).collect();
    assert_eq!(on, ["CH1"]);
}

/// The second half of a pair fails: what the scope took first is known.
#[test]
fn a_pair_of_settings_keeps_what_got_through() {
    let opcs = Arc::new(Mutex::new(0));
    let opcs2 = Arc::clone(&opcs);
    let (mut d, _) = scope("HMO1002", move |q| {
        if q == "*OPC?" {
            let mut n = opcs2.lock().unwrap();
            *n += 1;
            return (*n > 1).then(Vec::new);
        }
        None
    });
    d.open().unwrap();
    assert!(d.set(&Key::HighResolution, &Value::Bool(true), None).is_err());
    assert!(d.state().high_resolution, "C says off, with the scope on");
    assert!(d.doubt.load(Ordering::SeqCst));
}

#[test]
fn a_get_answers_only_its_own_group() {
    let (mut d, _) = plain("HMO1002");
    d.open().unwrap();
    assert!(matches!(d.get(&Key::Vdiv, None), Err(Error::ChannelGroup(_))));
    assert!(matches!(d.get(&Key::Vdiv, Some("POD1")), Err(Error::NotSupported(_))));
    assert!(matches!(d.get(&Key::LogicThreshold, Some("CH1")), Err(Error::NotSupported(_))));
    assert!(matches!(d.get(&Key::Timebase, Some("POD9")), Err(Error::ChannelGroup(_))));
    assert_eq!(d.get(&Key::NumVdiv, Some("CH1")).unwrap(), Value::I32(8));
    assert_eq!(d.get(&Key::LimitFrames, None).unwrap(), Value::U64(0));
    assert!(matches!(d.get(&Key::LimitSamples, None), Err(Error::UnknownKey(_))));
    assert!(d.list(&Key::Vdiv, None).is_err());
    assert!(d.list(&Key::Coupling, None).is_err());
    assert!(d.list(&Key::LogicThreshold, None).is_err());
    assert_eq!(d.options(None).unwrap().len(), 11);
    assert_eq!(d.options(Some("CH2")).unwrap().len(), 3);
    assert_eq!(d.options(Some("POD1")).unwrap().len(), 2);
    assert!(d.options(Some("POD2")).is_err(), "one pod");
}

#[test]
fn groups_are_as_c_makes_them() {
    let (d, _) = plain("HMO3054");
    let groups: Vec<(&str, usize)> =
        d.channel_groups().iter().map(|g| (g.name.as_str(), g.channels.len())).collect();
    assert_eq!(groups, [("CH1", 1), ("CH2", 1), ("CH3", 1), ("CH4", 1), ("POD1", 8), ("POD2", 8)]);
    assert_eq!(d.channels()[4].name, "D0");
    assert_eq!(d.channels()[4].index, 0);
    assert_eq!(d.channels()[0].index, 0);
    assert_eq!(d.channel_groups()[5].channels[0], 12, "by place in the list");
    assert_eq!(d.classes(), &[DeviceClass::Oscilloscope]);
    assert_eq!(HamegHmoDriver.classes().len(), 2);
    assert_eq!(models::find("RTH1004").unwrap().digital, 8);
    assert!(protocol::known_manufacturer("HAMEG"));
    assert!(protocol::known_manufacturer("Rohde&Schwarz"));
    assert!(!protocol::known_manufacturer("hameg"));
}
