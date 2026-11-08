// Counterpart in sigrok:
//   libsigrok/src/hardware/ipdbg-la/protocol.c,
//   Copyright (C) 2016 Eva Kissling <eva.kissling@bluewin.ch>,
//   with commits by Uwe Hermann, Daniel Anselmi, danselmi
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

use super::*;
use crate::core::TriggerStage;
use crate::core::trigger::TriggerCondition;
use crate::serial::WriteLog;
use protocol::{
    MAX_ADDR_WIDTH, MAX_DATA_WIDTH, SILENCE_LIMIT, SILENCE_WARNING, TriggerFields, configuration,
    convert_trigger, delay_value, parse_widths, split_conn,
};
use std::collections::VecDeque;
use std::net::TcpListener;
use std::sync::mpsc::{Receiver, sync_channel};

/// What a read of the fake finds.
#[derive(Clone, Debug)]
enum Step {
    Data(Vec<u8>),
    /// As a read that returned this late with these bytes.
    Late(Duration, Vec<u8>),
    Empty,
    Gone,
    Fail,
}

#[derive(Default)]
struct Shared {
    steps: VecDeque<Step>,
    writes: WriteLog,
    reads: usize,
    purges: usize,
    closed: usize,
    /// A write fails from this many on.
    fail_writes_from: Option<usize>,
    /// Bytes that have arrived since before a capture: a purge takes them.
    stale: Vec<u8>,
    /// `steps` wait for the arming, as a dump does. A purge shuts this again.
    gated: bool,
    armed: bool,
}

#[derive(Clone, Default)]
struct Fake(Arc<Mutex<Shared>>);

impl Fake {
    fn lock(&self) -> std::sync::MutexGuard<'_, Shared> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn with(steps: Vec<Step>) -> Fake {
        let f = Fake::default();
        f.lock().steps = steps.into();
        f
    }

    fn feed(&self, bytes: &[u8]) {
        self.lock().steps.push_back(Step::Data(bytes.to_vec()));
    }

    fn written(&self) -> Vec<u8> {
        self.lock().writes.iter().flat_map(|(_, w)| w.clone()).collect()
    }
}

impl Wire for Fake {
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let step = {
            let mut s = self.lock();
            s.reads += 1;
            if !s.stale.is_empty() {
                let n = s.stale.len().min(buf.len());
                buf[..n].copy_from_slice(&s.stale[..n]);
                s.stale.drain(..n);
                return Ok(n);
            }
            if s.gated && !s.armed { None } else { s.steps.pop_front() }
        };
        match step {
            None | Some(Step::Empty) => {
                std::thread::sleep(timeout.min(Duration::from_millis(1)));
                Ok(0)
            }
            Some(Step::Late(wait, d)) => {
                std::thread::sleep(wait);
                let n = d.len().min(buf.len());
                buf[..n].copy_from_slice(&d[..n]);
                Ok(n)
            }
            Some(Step::Gone) => Err(Error::Disconnected),
            Some(Step::Fail) => Err(Error::Timeout),
            Some(Step::Data(d)) => {
                let n = d.len().min(buf.len());
                buf[..n].copy_from_slice(&d[..n]);
                if n < d.len() {
                    self.lock().steps.push_front(Step::Data(d[n..].to_vec()));
                }
                Ok(n)
            }
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut s = self.lock();
        if s.fail_writes_from.is_some_and(|n| s.writes.len() >= n) {
            return Err(Error::Disconnected);
        }
        s.writes.push((Instant::now(), data.to_vec()));
        if data.last() == Some(&protocol::CMD_START) {
            s.armed = true;
        }
        Ok(())
    }

    fn purge(&mut self) -> Result<()> {
        let mut s = self.lock();
        s.purges += 1;
        s.stale.clear();
        s.armed = false;
        Ok(())
    }

    fn close(self: Box<Self>) {
        self.lock().closed += 1;
    }
}

fn opener_of(fakes: Vec<Fake>) -> (Opener, Arc<Mutex<usize>>) {
    let opens = Arc::new(Mutex::new(0));
    let count = Arc::clone(&opens);
    let fakes = Mutex::new(VecDeque::from(fakes));
    let opener: Opener = Arc::new(move || {
        *count.lock().expect("count") += 1;
        match fakes.lock().expect("fakes").pop_front() {
            Some(f) => Ok(Box::new(f) as Box<dyn Wire>),
            None => Err(Error::device("no fake left")),
        }
    });
    (opener, opens)
}

/// A device of `data` and `addr` bits on `fake`, opened.
fn device(data: u32, addr: u32, fake: &Fake) -> IpdbgLaDevice {
    fake.lock().gated = true;
    let (opener, _) = opener_of(vec![fake.clone()]);
    let mut d = IpdbgLaDevice::new(Widths { data, addr }, opener);
    d.open().expect("an open");
    d
}

/// Samples of a dump of `n` samples of two bytes: the ramp.
fn ramp(n: u16) -> Vec<u8> {
    (0..n).flat_map(u16::to_le_bytes).collect()
}

/// Runs `f` on its own thread with a bound, so that a mutant fails and doesn't hang.
fn bounded<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    rx.recv_timeout(Duration::from_secs(60)).expect("the test ended within its bound")
}

fn collect(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let mut packets = Vec::new();
    while packets.len() < 64 {
        let (_, p) = rx.recv_timeout(Duration::from_secs(30)).expect("a packet");
        let end = p == Packet::End;
        packets.push(p);
        if end {
            break;
        }
    }
    packets
}

fn capture_of(mut d: IpdbgLaDevice) -> Vec<Packet> {
    let (tx, rx) = sync_channel(64);
    d.start(PacketSink::new(tx, 0)).expect("a start");
    collect(&rx)
}

fn logic(p: &Packet) -> Option<&[u8]> {
    match p {
        Packet::Logic(l) => Some(&l.data),
        _ => None,
    }
}

fn wait_for(mut f: impl FnMut() -> bool) {
    let began = Instant::now();
    while !f() {
        assert!(began.elapsed() < Duration::from_secs(30), "what must have happened didn't");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn trigger_of(conditions: &[(usize, TriggerMatch)]) -> Trigger {
    Trigger {
        stages: vec![TriggerStage {
            conditions: conditions
                .iter()
                .map(|(channel, matches)| TriggerCondition {
                    channel: *channel,
                    matches: *matches,
                    value: 0.0,
                })
                .collect(),
            ..TriggerStage::default()
        }],
        ..Trigger::default()
    }
}

// Pure parts.

#[test]
fn a_configuration_with_no_trigger_is_what_c_sends() {
    let f = convert_trigger(None, 2, |_| true);
    assert_eq!(
        configuration(&f, 3, 2),
        [
            0xF0, 0xF1, 0xF3, 0, 0, 0xF0, 0xF1, 0xF7, 0, 0, 0xF0, 0xF9, 0xFB, 0, 0, 0xF0, 0xF9,
            0xFF, 0, 0, 0xF0, 0xF5, 0xF6, 0, 0, 0x0F, 0x1F, 0, 3
        ]
    );
}

#[test]
fn fields_go_high_byte_first_and_each_to_its_command() {
    let f = TriggerFields {
        mask: vec![0x01, 0x02],
        value: vec![0x03, 0x04],
        mask_last: vec![0x05, 0x06],
        value_last: vec![0x07, 0x08],
        edge_mask: vec![0x09, 0x0A],
    };
    assert_eq!(
        configuration(&f, 0x0B0C, 2),
        [
            0xF0, 0xF1, 0xF3, 0x02, 0x01, 0xF0, 0xF1, 0xF7, 0x04, 0x03, 0xF0, 0xF9, 0xFB, 0x06,
            0x05, 0xF0, 0xF9, 0xFF, 0x08, 0x07, 0xF0, 0xF5, 0xF6, 0x0A, 0x09, 0x0F, 0x1F, 0x0B,
            0x0C
        ]
    );
}

#[test]
fn a_reset_and_an_escape_in_a_field_are_escaped_and_nothing_else_is() {
    let f = TriggerFields { mask: vec![0x55], value: vec![0xEE], ..Default::default() };
    let wire = configuration(&f, 0x55EE, 2);
    assert_eq!(&wire[..4], &[0xF0, 0xF1, 0xF3, 0x55]);
    assert_eq!(wire[4], 0x55);
    assert_eq!(&wire[5..9], &[0xF0, 0xF1, 0xF7, 0x55]);
    assert_eq!(wire[9], 0xEE);
    assert_eq!(&wire[wire.len() - 4..], &[0x55, 0x55, 0x55, 0xEE]);
    // No command needs one, and a plain value none.
    assert!(!configuration(&convert_trigger(None, 1, |_| true), 0x54, 1).contains(&0x55));
}

#[test]
fn each_match_sets_what_the_c_sets() {
    let f = |m| convert_trigger(Some(&trigger_of(&[(9, m)])), 2, |_| true);
    let one = f(TriggerMatch::One);
    assert_eq!((one.mask[1], one.value[1], one.mask_last[1], one.edge_mask[1]), (2, 2, 0, 0));
    let zero = f(TriggerMatch::Zero);
    assert_eq!((zero.mask[1], zero.value[1], zero.mask_last[1], zero.edge_mask[1]), (2, 0, 0, 0));
    let r = f(TriggerMatch::Rising);
    assert_eq!((r.mask[1], r.value[1], r.mask_last[1], r.value_last[1]), (2, 2, 2, 0));
    let down = f(TriggerMatch::Falling);
    assert_eq!((down.mask[1], down.value[1], down.mask_last[1], down.value_last[1]), (2, 0, 2, 2));
    let e = f(TriggerMatch::Edge);
    assert_eq!((e.mask[1], e.mask_last[1], e.edge_mask[1]), (0, 0, 2));
    assert_eq!(e.mask[0], 0);
    assert_eq!(e.edge_mask[0], 0);
}

#[test]
fn a_later_match_changes_an_earlier_one_as_in_c() {
    // api: an edge clears what a level set, and leaves its value.
    let t = trigger_of(&[(0, TriggerMatch::One), (0, TriggerMatch::Edge)]);
    let f = convert_trigger(Some(&t), 1, |_| true);
    assert_eq!((f.mask[0], f.value[0], f.edge_mask[0]), (0, 1, 1));
    let t = trigger_of(&[(0, TriggerMatch::Edge), (0, TriggerMatch::Zero)]);
    let f = convert_trigger(Some(&t), 1, |_| true);
    assert_eq!((f.mask[0], f.value[0], f.edge_mask[0]), (1, 0, 0));
}

#[test]
fn a_rising_match_clears_what_a_falling_one_left_in_the_last_value() {
    let t = trigger_of(&[(0, TriggerMatch::Falling), (0, TriggerMatch::Rising)]);
    let f = convert_trigger(Some(&t), 1, |_| true);
    assert_eq!((f.value[0], f.value_last[0]), (1, 0));
}

#[test]
fn a_disabled_channel_or_one_past_the_sample_sets_nothing() {
    let t = trigger_of(&[(1, TriggerMatch::One), (2, TriggerMatch::One), (99, TriggerMatch::One)]);
    let f = convert_trigger(Some(&t), 1, |c| c != 1);
    assert_eq!(f.mask[0], 4);
}

#[test]
fn every_stage_counts() {
    let mut t = trigger_of(&[(0, TriggerMatch::One)]);
    t.stages.push(trigger_of(&[(1, TriggerMatch::One)]).stages.remove(0));
    assert_eq!(convert_trigger(Some(&t), 1, |_| true).mask[0], 3);
}

#[test]
fn the_delay_rounds_as_c_does() {
    for (limit, ratio, want) in [
        (512, 50, 255),
        (8, 50, 3),
        (1, 50, 0),
        (2, 50, 0),
        (150, 33, 49),
        (16, 99, 14),
        (16, 100, 15),
        (16, 0, 0),
        (171, 50, 85),
        (477, 50, 238),
        (1 << 30, 100, (1 << 30) - 1),
    ] {
        assert_eq!(delay_value(limit, ratio), want, "{limit} at {ratio}");
    }
}

#[test]
fn widths_read_low_byte_first_and_a_core_cannot_have_more_than_the_limits() {
    assert_eq!(parse_widths(&[0x10, 0, 0, 0, 9, 0, 0, 0]), Ok((16, 9)));
    assert_eq!(parse_widths(&[0, 1, 0, 0, 0, 0, 0, 0]), Ok((256, 0)));
    let of = |d: u32, a: u32| {
        let mut b = [0; 8];
        b[..4].copy_from_slice(&d.to_le_bytes());
        b[4..].copy_from_slice(&a.to_le_bytes());
        parse_widths(&b)
    };
    assert_eq!(MAX_ADDR_WIDTH, 30);
    assert_eq!(MAX_DATA_WIDTH, 65536);
    assert!(of(65536, 30).is_ok());
    assert_eq!(
        of(65537, 0).unwrap_err(),
        "Device reports 65537 channels, more than the 65536 here."
    );
    assert_eq!(
        of(8, 31).unwrap_err(),
        "Device reports 31 address bits, more than the 30 a core can have."
    );
}

/// The sizes have no check of their own: every single bit flipped in a good answer
/// of 16 and 9, and how many give a wrong width that goes unnoticed.
#[test]
fn what_the_widths_cannot_catch() {
    let good = [0x10, 0, 0, 0, 9, 0, 0, 0];
    let (mut refused, mut wrong) = (0, Vec::new());
    for bit in 0..64 {
        let mut a = good;
        a[bit / 8] ^= 1 << (bit % 8);
        match parse_widths(&a) {
            Err(_) => refused += 1,
            Ok(widths) => wrong.push(widths),
        }
    }
    assert_eq!(wrong.len(), 21, "{wrong:?}");
    assert_eq!(refused, 43);
    // All wrong, none good: 16 flips in the channels' low bytes, and the 5 of the
    // address's low byte that stay under 31 bits.
    assert!(wrong.iter().all(|w| *w != (16, 9)));
}

/// A lost byte moves the address's low byte into the channels' top: refused unless
/// it comes to something small.
#[test]
fn a_lost_byte_in_the_sizes() {
    let good = [0x10, 0, 0, 0, 9, 0, 0, 0];
    for lost in 0..8 {
        let mut a = good.to_vec();
        a.remove(lost);
        // What the tries wait for: a short answer, which is no device.
        assert_eq!(a.len(), 7);
        let f = Fake::with(vec![Step::Data(b"IDBG".to_vec()), Step::Data(a), Step::Gone]);
        assert_eq!(probe(&mut f.clone()), None, "byte {lost}");
    }
}

/// The id: every single bit flipped is refused, so no flip passes for a core.
#[test]
fn every_flip_of_the_id_is_no_device() {
    for bit in 0..32 {
        let mut id = *b"IDBG";
        id[bit / 8] ^= 1 << (bit % 8);
        let f = Fake::with(vec![Step::Data(id.to_vec())]);
        assert_eq!(probe(&mut f.clone()), None, "bit {bit}");
    }
}

/// Undoes the escapes of one field, as `IpdbgEscaping.vhd` does.
fn unescape(wire: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut escaped = false;
    for &b in wire {
        if escaped {
            out.push(b);
            escaped = false;
        } else if b == 0x55 {
            escaped = true;
        } else {
            assert_ne!(b, 0xEE, "a reset in a field");
            out.push(b);
        }
    }
    assert!(!escaped);
    out
}

#[test]
fn every_value_of_a_field_comes_out_of_the_cores_escape_layer_whole() {
    for b in 0..=255u8 {
        for high in [0, 0x55, 0xEE, b] {
            let f = TriggerFields { mask: vec![b, high], ..convert_trigger(None, 2, |_| true) };
            let wire = configuration(&f, u64::from(b) | u64::from(high) << 8, 2);
            let at = |w: [u8; 3]| wire.windows(3).position(|x| x == w).expect("a command");
            let n = wire.windows(2).position(|x| x == [0x0F, 0x1F]).expect("the delay");
            assert_eq!(unescape(&wire[n + 2..]), [high, b]);
            assert_eq!(unescape(&wire[3..at([0xF0, 0xF1, 0xF7])]), [high, b]);
        }
    }
}

/// Every width of a delay: `addr_bytes` from the low end, high byte first.
#[test]
fn a_delay_takes_the_bytes_of_the_address() {
    let f = convert_trigger(None, 0, |_| true);
    for (bytes, want) in [
        (0, &[][..]),
        (1, &[0x04]),
        (2, &[0x03, 0x04]),
        (3, &[0x02, 0x03, 0x04]),
        (4, &[0x01, 0x02, 0x03, 0x04]),
    ] {
        let wire = configuration(&f, 0x0102_0304, bytes);
        let at = wire.iter().position(|x| *x == 0x1F).expect("the delay");
        assert_eq!(&wire[at + 1..], want, "{bytes} bytes");
    }
}

#[test]
fn conn_splits_as_g_strsplit_does() {
    assert_eq!(split_conn("tcp-raw/h/1"), Some(("h", Some("1"))));
    assert_eq!(split_conn("x/h/1"), Some(("h", Some("1"))));
    assert_eq!(split_conn("/h/1"), Some(("h", Some("1"))));
    assert_eq!(split_conn("x/h/1/2"), Some(("h", Some("1/2"))));
    assert_eq!(split_conn("x/h"), Some(("h", None)));
    assert_eq!(split_conn("x"), None);
}

#[test]
fn a_connect_says_what_failed() {
    let lookup = connect("127.0.0.1", Some("1/x")).err().expect("no number").to_string();
    assert!(
        lookup.contains("Address lookup failed: 127.0.0.1:1/x: the port is no number"),
        "{lookup}"
    );
    let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
    let port = listener.local_addr().expect("an address").port();
    drop(listener);
    let refused = connect("127.0.0.1", Some(&port.to_string())).err().expect("refused").to_string();
    assert!(refused.contains(&format!("Failed to connect to 127.0.0.1:{port}: ")), "{refused}");
    let none = connect("127.0.0.1", None).err().expect("port 0").to_string();
    assert!(none.contains("127.0.0.1:(no port)"), "{none}");
}

// The rule on silence, over times given to it.

fn at(start: Instant, s: u64) -> Instant {
    start + Duration::from_secs(s)
}

#[test]
fn waiting_for_a_trigger_warns_once_and_never_fails() {
    assert_eq!(SILENCE_WARNING, Duration::from_secs(30));
    let t = Instant::now();
    let mut r = Silence::new(t, 1024);
    for s in 1..30 {
        r.look(at(t, s));
        assert_eq!(r.empty(), Verdict::Wait, "at {s}");
    }
    let mut warned = 0;
    for s in 30..100_000 {
        r.look(at(t, s));
        match r.empty() {
            Verdict::Wait => {}
            Verdict::Warn(text) => {
                assert_eq!(
                    text,
                    "No data for 30 s: the core is waiting for its trigger or its sample signal. \
                     A stop ends this."
                );
                warned += 1;
            }
            Verdict::Fail(text) => panic!("failed: {text}"),
        }
    }
    assert_eq!(warned, 1);
}

#[test]
fn a_dump_that_stops_warns_and_then_fails_at_its_limit() {
    assert_eq!(SILENCE_LIMIT, Duration::from_secs(600));
    let t = Instant::now();
    let mut r = Silence::new(t, 1024);
    r.look(t);
    assert_eq!(r.got(t, 100), None);
    for s in 1..30 {
        r.look(at(t, s));
        assert_eq!(r.empty(), Verdict::Wait, "at {s}");
    }
    r.look(at(t, 30));
    assert_eq!(
        r.empty(),
        Verdict::Warn(
            "No data for 30 s inside a dump, at 100 of 1024 bytes: still waiting, for up to 600 s."
                .to_owned()
        )
    );
    // Looks a second apart: no stall, and no second warning.
    for s in 31..600 {
        r.look(at(t, s));
        assert_eq!(r.empty(), Verdict::Wait, "at {s}");
    }
    r.look(at(t, 600));
    assert_eq!(
        r.empty(),
        Verdict::Fail("No data for 600 s inside a dump, at 100 of 1024 bytes.".to_owned())
    );
}

#[test]
fn a_byte_starts_the_silence_again_and_says_it_is_back() {
    let t = Instant::now();
    let mut r = Silence::new(t, 1024);
    r.look(t);
    let _ = r.got(t, 1);
    // Many bounds, with real gaps nowhere: removing the reset fails this.
    let mut now = 0;
    for _ in 0..10 {
        for _ in 0..590 {
            now += 1;
            r.look(at(t, now));
            assert!(!matches!(r.empty(), Verdict::Fail(_)), "at {now}");
        }
        assert_eq!(
            r.got(at(t, now), 1),
            Some(format!("Data again after 590 s, at {} of 1024 bytes.", 1 + (now / 590))),
        );
    }
}

#[test]
fn time_the_thread_did_not_run_is_excused_up_to_a_cap() {
    let t = Instant::now();
    let mut r = Silence::new(t, 1024);
    r.look(t);
    let _ = r.got(t, 1);
    // A stall of 40 s: no warning at 40 of wall clock.
    r.look(at(t, 40));
    assert_eq!(r.empty(), Verdict::Wait);
    // A process stopped again and again gets 60 s, and no more.
    let mut now = 40;
    for _ in 0..50 {
        now += 11;
        r.look(at(t, now));
        let _ = r.empty();
    }
    assert_eq!(now, 590);
    let mut failed_at = None;
    for s in 591..700 {
        r.look(at(t, s));
        if matches!(r.empty(), Verdict::Fail(_)) {
            failed_at = Some(s);
            break;
        }
    }
    assert_eq!(failed_at, Some(660));
}

#[test]
fn a_byte_ends_what_a_stall_excused() {
    let t = Instant::now();
    let mut r = Silence::new(t, 1024);
    r.look(t);
    let _ = r.got(t, 1);
    r.look(at(t, 50));
    let _ = r.got(at(t, 50), 1);
    // The next silence has no credit of the last: a warning at 30 s of it.
    for s in 51..80 {
        r.look(at(t, s));
        assert_eq!(r.empty(), Verdict::Wait, "at {s}");
    }
    r.look(at(t, 80));
    assert!(matches!(r.empty(), Verdict::Warn(_)));
}

#[test]
fn the_bound_goes_by_the_time_ahead_of_the_read() {
    let t = Instant::now();
    let mut r = Silence::new(t, 1024);
    r.look(t);
    let _ = r.got(t, 1);
    for s in 1..=599 {
        r.look(at(t, s));
        let _ = r.empty();
    }
    // No look since 599: the read found nothing as of then, however late it returned.
    assert!(!matches!(r.empty(), Verdict::Fail(_)));
}

// A scan.

#[test]
fn a_probe_sends_what_c_sends_and_reads_the_widths() {
    let f =
        Fake::with(vec![Step::Data(b"IDBG".to_vec()), Step::Data(vec![0x10, 0, 0, 0, 9, 0, 0, 0])]);
    let (opener, _) = opener_of(vec![f.clone()]);
    let d = scan_with(&opener).expect("a device");
    assert_eq!(f.written(), [0xEE, 0xEE, 0xBB, 0xAA]);
    assert_eq!(f.lock().closed, 1, "a scan closes");
    assert_eq!(d.channels.len(), 16);
    assert_eq!(d.channels[15].name, "CH15");
    assert_eq!(d.limit_samples, 512);
    assert_eq!(d.get(&Key::LimitSamples, None).expect("a limit"), Value::U64(512));
    assert_eq!(d.get(&Key::CaptureRatio, None).expect("a ratio"), Value::U64(50));
    assert_eq!(d.info.model, "IPDBG LA");
    assert_eq!(d.info.version, "v1.0");
    assert!(d.info.conn.is_none());
}

#[test]
fn an_answer_in_pieces_is_one_answer() {
    let f = Fake::with(vec![
        Step::Data(b"ID".to_vec()),
        Step::Empty,
        Step::Data(b"BG".to_vec()),
        Step::Data(vec![8, 0, 0]),
        Step::Empty,
        Step::Data(vec![0, 4, 0, 0, 0]),
    ]);
    assert_eq!(probe(&mut f.clone()), Some(Widths { data: 8, addr: 4 }));
}

#[test]
fn whatever_id_is_not_idbg_is_no_device_and_asks_no_more() {
    for id in [&b"idbg"[..], b"xXXX"] {
        let f = Fake::with(vec![Step::Data(id.to_vec())]);
        assert_eq!(probe(&mut f.clone()), None);
        assert_eq!(f.written(), [0xEE, 0xEE, 0xBB]);
    }
}

#[test]
fn a_short_id_or_a_close_is_no_device() {
    for steps in [vec![Step::Data(b"ID".to_vec()), Step::Gone], vec![Step::Gone], vec![Step::Fail]]
    {
        let f = Fake::with(steps);
        assert_eq!(probe(&mut f.clone()), None);
        assert_eq!(f.written(), [0xEE, 0xEE, 0xBB]);
    }
}

#[test]
fn a_silent_core_is_given_up_on_at_2_s() {
    assert_eq!(protocol::RECEIVE_WAIT, Duration::from_secs(2));
    let f = Fake::default();
    let mut wire = f.clone();
    let mut id = [0u8; 4];
    let began = Instant::now();
    assert_eq!(receive_blocking(&mut wire, &mut id), 0);
    assert!(began.elapsed() >= Duration::from_millis(1900), "{:?}", began.elapsed());
    // Sleeps run long on a loaded box, so what bounds it from above is what it asked for.
    assert!(f.lock().reads <= 2100, "{} reads", f.lock().reads);
}

#[test]
fn an_answer_that_waited_out_a_stall_is_still_taken() {
    let f = Fake::with(vec![Step::Late(Duration::from_millis(2300), b"IDBG".to_vec())]);
    let mut wire = f.clone();
    let mut id = [0u8; 4];
    assert_eq!(receive_blocking(&mut wire, &mut id), 4);
    assert_eq!(&id, b"IDBG");
}

#[test]
fn bytes_do_not_give_the_wait_back_as_in_c() {
    // A byte doesn't start the 2 s again: a core that sends one byte a second is gone
    // by then.
    let mut steps = Vec::new();
    for _ in 0..3 {
        steps.push(Step::Late(Duration::from_millis(900), b"I".to_vec()));
    }
    let f = Fake::with(steps);
    let mut wire = f.clone();
    let mut id = [0u8; 8];
    let got = receive_blocking(&mut wire, &mut id);
    assert_eq!(got, 3);
    assert!(f.lock().steps.is_empty());
}

#[test]
fn a_late_byte_does_not_start_the_wait_again() {
    // Past the 2 s with one byte in, the next empty read ends it, with the rest offered.
    let f = Fake::with(vec![
        Step::Late(Duration::from_millis(2100), b"I".to_vec()),
        Step::Empty,
        Step::Data(b"DBG".to_vec()),
    ]);
    let mut wire = f.clone();
    let mut id = [0u8; 4];
    assert_eq!(receive_blocking(&mut wire, &mut id), 1);
    assert_eq!(f.lock().steps.len(), 1);
}

#[test]
fn short_sizes_or_widths_past_the_limits_are_no_device() {
    let f =
        Fake::with(vec![Step::Data(b"IDBG".to_vec()), Step::Data(vec![8, 0, 0, 0, 4]), Step::Gone]);
    assert_eq!(probe(&mut f.clone()), None);
    let f =
        Fake::with(vec![Step::Data(b"IDBG".to_vec()), Step::Data(vec![8, 0, 0, 0, 31, 0, 0, 0])]);
    assert_eq!(probe(&mut f.clone()), None);
    let (opener, _) = opener_of(vec![f]);
    assert!(scan_with(&opener).is_none());
}

#[test]
fn a_scan_with_no_connection_is_no_device() {
    let (opener, _) = opener_of(vec![]);
    assert!(scan_with(&opener).is_none());
    let d = IpdbgLaDriver;
    assert!(d.scan(&[]).expect("no conn").is_empty());
    let one = [(Key::Conn, Value::Str("tcp-raw".to_owned()))];
    assert!(d.scan(&one).expect("one field").is_empty());
}

// Settings.

#[test]
fn a_limit_past_the_memory_leaves_the_limit() {
    let f = Fake::default();
    let mut d = device(16, 9, &f);
    d.set(&Key::LimitSamples, &Value::U64(100), None).expect("a limit");
    d.set(&Key::LimitSamples, &Value::U64(513), None).expect("as C");
    assert_eq!(d.get(&Key::LimitSamples, None).expect("a limit"), Value::U64(100));
    d.set(&Key::LimitSamples, &Value::U64(512), None).expect("the most");
    assert_eq!(d.get(&Key::LimitSamples, None).expect("a limit"), Value::U64(512));
}

#[test]
fn what_the_core_keys_refuse_a_library_client_too() {
    let f = Fake::default();
    let mut d = device(16, 9, &f);
    assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
    d.set(&Key::CaptureRatio, &Value::U64(100), None).expect("the most");
    assert!(d.set(&Key::Samplerate, &Value::U64(1), None).is_err());
    assert!(d.get(&Key::Samplerate, None).is_err());
    assert!(d.get(&Key::LimitSamples, Some("g")).is_err());
    assert!(d.list(&Key::Samplerate, None).is_err());
    assert_eq!(
        d.list(&Key::TriggerMatch, None).expect("a list"),
        ValueList::TriggerMatches(vec![
            TriggerMatch::Zero,
            TriggerMatch::One,
            TriggerMatch::Rising,
            TriggerMatch::Falling,
            TriggerMatch::Edge
        ])
    );
}

#[test]
fn a_disabled_channels_trigger_stays_off_the_wire() {
    let f = Fake::with(vec![Step::Data(ramp(8))]);
    let mut d = device(16, 3, &f);
    d.enable_channel(3, false).expect("a channel");
    d.set_trigger(Some(trigger_of(&[(3, TriggerMatch::One)]))).expect("a trigger");
    let _ = capture_of(d);
    assert_eq!(f.written()[3], 0);
}

// A capture.

#[test]
fn a_capture_cuts_the_dump_at_the_limit_around_the_trigger() {
    let f = Fake::with(vec![Step::Data(ramp(512))]);
    let mut d = device(16, 9, &f);
    d.set(&Key::LimitSamples, &Value::U64(8), None).expect("a limit");
    let packets = capture_of(d);
    assert!(matches!(packets[0], Packet::Header { .. }));
    assert_eq!(logic(&packets[1]).expect("before"), &ramp(3)[..]);
    assert_eq!(packets[2], Packet::Trigger);
    assert_eq!(logic(&packets[3]).expect("after"), &ramp(8)[6..]);
    assert_eq!(packets[4], Packet::End);
    assert_eq!(packets.len(), 5);
    let wire = f.written();
    assert_eq!(*wire.last().expect("a write"), 0xFE);
    assert_eq!(wire[wire.len() - 3..wire.len() - 1], [0, 3], "the delay");
}

#[test]
fn the_header_goes_after_the_delay_and_before_the_arming() {
    let f = Fake::default();
    let mut d = device(16, 9, &f);
    // A queue of one, left full: the header waits, and so must the arming.
    let (tx, rx) = sync_channel(1);
    tx.send((0, Packet::FrameEnd)).expect("room");
    d.start(PacketSink::new(tx, 0)).expect("a start");
    std::thread::sleep(Duration::from_millis(50));
    assert_ne!(f.written().last(), Some(&0xFE), "no arming ahead of the header");
    assert_eq!(rx.recv().expect("mine").1, Packet::FrameEnd);
    assert!(matches!(
        rx.recv_timeout(Duration::from_secs(30)).expect("a header").1,
        Packet::Header { .. }
    ));
    wait_for(|| f.written().last() == Some(&0xFE));
    d.stop().expect("a stop");
}

#[test]
fn no_sample_ahead_of_the_trigger_means_no_logic_packet_ahead_of_it() {
    let f = Fake::with(vec![Step::Data(ramp(8))]);
    let mut d = device(16, 3, &f);
    d.set(&Key::CaptureRatio, &Value::U64(0), None).expect("a ratio");
    let packets = capture_of(d);
    assert_eq!(packets[1], Packet::Trigger);
    assert_eq!(logic(&packets[2]).expect("all"), &ramp(8)[..]);
    assert_eq!(packets.len(), 4);
}

#[test]
fn ratio_100_leaves_one_sample_after_the_trigger() {
    let f = Fake::with(vec![Step::Data(ramp(8))]);
    let mut d = device(16, 3, &f);
    d.set(&Key::CaptureRatio, &Value::U64(100), None).expect("a ratio");
    let packets = capture_of(d);
    assert_eq!(logic(&packets[1]).expect("before"), &ramp(7)[..]);
    assert_eq!(logic(&packets[3]).expect("after"), &ramp(8)[14..]);
}

#[test]
fn a_dump_in_pieces_with_gaps_is_one_dump() {
    let whole = ramp(512);
    let mut steps = Vec::new();
    for piece in whole.chunks(37) {
        steps.push(Step::Data(piece.to_vec()));
        steps.push(Step::Empty);
    }
    let f = Fake::with(steps);
    let d = device(16, 9, &f);
    let packets = capture_of(d);
    assert_eq!(logic(&packets[1]).expect("before"), &whole[..510]);
    assert_eq!(logic(&packets[3]).expect("after"), &whole[510..]);
}

#[test]
fn it_ends_only_when_the_whole_memory_came_whatever_the_limit() {
    // 8 samples asked, 16 in memory: the dump's rest must be read, or it is taken
    // for the next capture.
    let f = Fake::with(vec![Step::Data(ramp(16)[..20].to_vec())]);
    let mut d = device(16, 4, &f);
    d.set(&Key::LimitSamples, &Value::U64(8), None).expect("a limit");
    let (tx, rx) = sync_channel(64);
    d.start(PacketSink::new(tx, 0)).expect("a start");
    assert!(matches!(
        rx.recv_timeout(Duration::from_secs(30)).expect("a header").1,
        Packet::Header { .. }
    ));
    assert!(rx.recv_timeout(Duration::from_millis(100)).is_err(), "nothing before the whole dump");
    f.feed(&ramp(16)[20..]);
    let rest = collect(&rx);
    assert_eq!(logic(&rest[0]).expect("before"), &ramp(3)[..]);
    assert_eq!(logic(&rest[2]).expect("after"), &ramp(8)[6..]);
}

#[test]
fn bytes_past_a_dump_are_not_samples() {
    let mut bytes = ramp(8);
    bytes.extend_from_slice(&[0xA5; 3]);
    let f = Fake::with(vec![Step::Data(bytes)]);
    let d = device(16, 3, &f);
    let packets = capture_of(d);
    assert_eq!(logic(&packets[3]).expect("after"), &ramp(8)[6..]);
}

#[test]
fn a_close_inside_a_dump_fails_it_and_says_where() {
    let f = Fake::with(vec![Step::Data(ramp(100)), Step::Gone]);
    let d = device(16, 9, &f);
    let packets = capture_of(d);
    assert_eq!(
        packets[1],
        Packet::Failed(
            "device error: The server closed the connection, at 200 of 1024 bytes.".to_owned()
        )
    );
    assert_eq!(packets[2], Packet::End);
}

#[test]
fn a_close_before_the_dump_fails_it_at_once() {
    let f = Fake::with(vec![Step::Gone]);
    let d = device(16, 9, &f);
    let packets = capture_of(d);
    assert_eq!(
        packets[1],
        Packet::Failed(
            "device error: The server closed the connection, at 0 of 1024 bytes.".to_owned()
        )
    );
}

#[test]
fn a_failed_read_fails_it() {
    let f = Fake::with(vec![Step::Fail]);
    let packets = capture_of(device(16, 9, &f));
    assert!(matches!(&packets[1], Packet::Failed(_)), "{packets:?}");
}

#[test]
fn a_failed_arming_fails_the_capture() {
    let f = Fake::default();
    f.lock().fail_writes_from = Some(1);
    let packets = capture_of(device(16, 9, &f));
    assert!(matches!(&packets[1], Packet::Failed(_)), "{packets:?}");
}

#[test]
fn a_failed_configuration_refuses_the_start_and_keeps_the_wire() {
    let f = Fake::default();
    f.lock().fail_writes_from = Some(0);
    let mut d = device(16, 9, &f);
    let (tx, _rx) = sync_channel(4);
    assert!(d.start(PacketSink::new(tx, 0)).is_err());
    f.lock().fail_writes_from = None;
    let (tx, rx) = sync_channel(4);
    d.start(PacketSink::new(tx, 0)).expect("a start");
    assert!(matches!(
        rx.recv_timeout(Duration::from_secs(30)).expect("a header").1,
        Packet::Header { .. }
    ));
    d.stop().expect("a stop");
}

#[test]
fn a_start_needs_an_open() {
    let (opener, _) = opener_of(vec![]);
    let mut d = IpdbgLaDevice::new(Widths { data: 8, addr: 3 }, opener);
    let (tx, _rx) = sync_channel(4);
    assert!(d.start(PacketSink::new(tx, 0)).is_err());
    assert!(d.open().is_err());
}

// A stop.

#[test]
fn a_stop_waiting_for_the_trigger_resets_and_sends_no_samples() {
    bounded(|| {
        let f = Fake::default();
        let mut d = device(16, 9, &f);
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).expect("a start");
        wait_for(|| f.written().last() == Some(&0xFE));
        d.stop().expect("a stop");
        assert_eq!(f.written().last(), Some(&0xEE));
        let packets = collect(&rx);
        assert_eq!(packets.len(), 2, "{packets:?}");
        assert_eq!(packets[1], Packet::End);
    });
}

#[test]
fn a_stop_inside_a_dump_reads_its_rest_and_then_resets() {
    bounded(|| {
        let f = Fake::with(vec![Step::Data(ramp(100))]);
        let mut d = device(16, 9, &f);
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).expect("a start");
        wait_for(|| f.lock().steps.is_empty() && f.lock().reads > 1);
        let feeder = f.clone();
        let stopping = std::thread::spawn(move || {
            d.stop().expect("a stop");
            d
        });
        // Not reset while the rest is on its way: release it by the reads.
        let reads = f.lock().reads;
        wait_for(|| f.lock().reads > reads + 2);
        assert_ne!(f.written().last(), Some(&0xEE), "a reset ahead of the rest");
        feeder.feed(&ramp(512)[200..]);
        let d = stopping.join().expect("a stop's thread");
        assert_eq!(f.written().last(), Some(&0xEE));
        let packets = collect(&rx);
        assert!(!packets.iter().any(|p| logic(p).is_some()), "{packets:?}");
        drop(d);
    });
}

#[test]
fn a_stop_gives_up_on_a_silent_dump_and_the_next_capture_connects_again() {
    bounded(|| {
        let f = Fake::default();
        let capture = Capture {
            unit_size: 2,
            delay_bytes: 0,
            keep_bytes: 8,
            total_bytes: 8,
            stop_silence: Duration::from_millis(50),
        };
        let began = Instant::now();
        assert!(capture.stop_in(&mut f.clone(), 2), "doubtful");
        assert!(began.elapsed() >= Duration::from_millis(50));
        assert_eq!(f.written(), [0xEE]);

        // No dump begun: no wait, and nothing doubtful.
        let f = Fake::default();
        let capture = Capture { stop_silence: Duration::from_secs(600), ..capture };
        assert!(!capture.stop_in(&mut f.clone(), 0));
        assert_eq!(f.written(), [0xEE]);
    });
}

#[test]
fn a_stop_silence_is_5_s() {
    assert_eq!(STOP_SILENCE, Duration::from_secs(5));
}

#[test]
fn a_read_that_fails_in_a_stop_still_resets() {
    let f = Fake::with(vec![Step::Gone]);
    let capture = Capture {
        unit_size: 2,
        delay_bytes: 0,
        keep_bytes: 8,
        total_bytes: 8,
        stop_silence: Duration::from_secs(600),
    };
    assert!(!capture.stop_in(&mut f.clone(), 2));
    assert_eq!(f.written(), [0xEE]);
}

#[test]
fn a_doubtful_line_gets_a_new_connection_and_a_reset() {
    bounded(|| {
        let first = Fake::default();
        let second = Fake::with(vec![Step::Data(ramp(8))]);
        let (opener, opens) = opener_of(vec![first.clone(), second.clone()]);
        let mut d = IpdbgLaDevice::new(Widths { data: 16, addr: 3 }, opener);
        d.open().expect("an open");
        d.lock().doubtful = true;
        let packets = {
            let (tx, rx) = sync_channel(64);
            d.start(PacketSink::new(tx, 0)).expect("a start");
            collect(&rx)
        };
        assert_eq!(*opens.lock().expect("count"), 2);
        assert_eq!(first.lock().closed, 1);
        assert_eq!(second.written()[0], 0xEE);
        assert!(logic(&packets[1]).is_some(), "{packets:?}");
    });
}

#[test]
fn a_second_capture_drops_what_has_arrived() {
    bounded(|| {
        let f = Fake::with(vec![Step::Data(ramp(8))]);
        let mut d = device(16, 3, &f);
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).expect("a start");
        let first = collect(&rx);
        assert_eq!(logic(&first[3]).expect("after"), &ramp(8)[6..]);
        // A tail of the first, and it must not count.
        f.lock().stale = vec![0xA5; 3];
        wait_for(|| d.lock().wire.is_some());
        let purges = f.lock().purges;
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).expect("a second start");
        assert_eq!(f.lock().purges, purges + 1);
        f.feed(&ramp(8));
        let second = collect(&rx);
        assert_eq!(logic(&second[1]).expect("before"), &ramp(3)[..]);
    });
}

#[test]
fn a_close_joins_and_writes_nothing_after() {
    bounded(|| {
        let f = Fake::default();
        let mut d = device(16, 9, &f);
        let (tx, _rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).expect("a start");
        wait_for(|| f.written().last() == Some(&0xFE));
        d.close().expect("a close");
        let writes = f.lock().writes.len();
        assert_eq!(f.lock().closed, 1);
        std::thread::sleep(Duration::from_millis(30));
        assert_eq!(f.lock().writes.len(), writes);
        assert!(d.worker.is_none());
    });
}

#[test]
fn a_full_queue_does_not_hold_a_close() {
    bounded(|| {
        let f = Fake::with(vec![Step::Data(ramp(8))]);
        let mut d = device(16, 3, &f);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).expect("a start");
        // The header fills it; nobody reads.
        wait_for(|| rx.try_recv().is_ok());
        wait_for(|| f.written().last() == Some(&0xFE));
        wait_for(|| f.lock().steps.is_empty());
        d.close().expect("a close");
        drop(rx);
    });
}

#[test]
fn the_wire_goes_back_ahead_of_the_end() {
    bounded(|| {
        let f = Fake::with(vec![Step::Data(ramp(8))]);
        let mut d = device(16, 3, &f);
        d.set(&Key::CaptureRatio, &Value::U64(0), None).expect("a ratio");
        // A queue of one, with the last samples left in it: the End can't go.
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).expect("a start");
        assert!(matches!(rx.recv().expect("a header").1, Packet::Header { .. }));
        assert_eq!(rx.recv().expect("a trigger").1, Packet::Trigger);
        wait_for(|| d.lock().wire.is_some());
        drop(rx);
    });
}

#[test]
fn a_restart_after_a_stuck_end_does_not_hang() {
    bounded(|| {
        let f = Fake::with(vec![Step::Data(ramp(8))]);
        let mut d = device(16, 3, &f);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).expect("a start");
        let _ = rx.recv_timeout(Duration::from_secs(30)).expect("a header");
        wait_for(|| rx.try_recv().is_ok());
        // Reads one at a time until the wire is back; the End may wait in the queue.
        wait_for(|| {
            let _ = rx.try_recv();
            d.lock().wire.is_some()
        });
        f.feed(&ramp(8));
        let (tx2, rx2) = sync_channel(64);
        d.start(PacketSink::new(tx2, 0)).expect("a second start");
        let packets = collect(&rx2);
        assert!(packets.iter().any(|p| logic(p).is_some()), "{packets:?}");
    });
}

#[test]
fn the_device_opens_again_after_a_close() {
    let a = Fake::default();
    let b = Fake::default();
    let (opener, opens) = opener_of(vec![a, b]);
    let mut d = IpdbgLaDevice::new(Widths { data: 8, addr: 3 }, opener);
    d.open().expect("an open");
    d.close().expect("a close");
    d.open().expect("a second open");
    assert_eq!(*opens.lock().expect("count"), 2);
}

#[test]
fn a_memory_of_one_sample_and_no_address_bytes_works() {
    let f = Fake::with(vec![Step::Data(vec![7])]);
    let d = device(8, 0, &f);
    let packets = capture_of(d);
    assert_eq!(packets[1], Packet::Trigger);
    assert_eq!(logic(&packets[2]).expect("it"), [7]);
    let wire = f.written();
    assert_eq!(wire[wire.len() - 3..], [0x0F, 0x1F, 0xFE]);
}

#[test]
fn a_wide_core_keeps_its_bytes_in_order() {
    // 9 bits take two bytes and 65 take nine.
    let f = Fake::with(vec![Step::Data((0..18).collect())]);
    let d = device(65, 1, &f);
    let packets = capture_of(d);
    let Packet::Logic(after) = &packets[2] else { panic!("{packets:?}") };
    assert_eq!(after.unit_size, 9);
    assert_eq!(after.data, (0..18).collect::<Vec<u8>>());
}

// A real pace and a real socket.

/// A byte-paced dump on a socket at about 100 kB/s, in the sizes of a JTAG server's
/// polls, with a test of the first bytes' arrival being no end.
#[test]
fn a_paced_dump_on_a_socket_comes_whole() {
    bounded(|| {
        let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
        let port = listener.local_addr().expect("an address").port().to_string();
        let whole = ramp(4096);
        let dump = whole.clone();
        let server = std::thread::spawn(move || {
            use std::io::{Read as _, Write as _};
            let (mut s, _) = listener.accept().expect("a connection");
            let mut one = [0u8; 1];
            loop {
                s.read_exact(&mut one).expect("a command");
                if one[0] == 0xFE {
                    break;
                }
            }
            for piece in dump.chunks(512) {
                s.write_all(piece).expect("a write");
                std::thread::sleep(Duration::from_millis(5));
            }
        });
        let host_port = port.clone();
        let opener: Opener = Arc::new(move || connect("127.0.0.1", Some(&host_port)));
        let mut d = IpdbgLaDevice::new(Widths { data: 16, addr: 12 }, opener);
        d.open().expect("an open");
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).expect("a start");
        let packets = collect(&rx);
        assert_eq!(logic(&packets[1]).expect("before").len(), 2 * 2047);
        let mut all = logic(&packets[1]).expect("before").to_vec();
        all.extend_from_slice(logic(&packets[3]).expect("after"));
        assert_eq!(all, whole);
        server.join().expect("a server");
    });
}

/// `OpenOCD`'s host serves one connection a core: the next one finds the last closed.
#[test]
fn a_close_waits_for_the_server_to_close_first() {
    bounded(|| {
        let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
        let port = listener.local_addr().expect("an address").port();
        let (closed_tx, closed_rx) = std::sync::mpsc::channel();
        let server = std::thread::spawn(move || {
            use std::io::Read as _;
            let (mut s, _) = listener.accept().expect("a connection");
            let mut buf = [0u8; 16];
            while !matches!(s.read(&mut buf), Ok(0) | Err(_)) {}
            // The client's FIN has come, and it should still be waiting.
            // By the clock and no sleep: under the sleep shim a sleep outlasts the 2 s.
            let fin = Instant::now();
            while fin.elapsed() < Duration::from_millis(200) {
                std::thread::yield_now();
            }
            closed_tx.send(Instant::now()).expect("the test");
        });
        let wire = connect("127.0.0.1", Some(&port.to_string())).expect("a connection");
        let began = Instant::now();
        wire.close();
        let done = Instant::now();
        let closed = closed_rx.recv().expect("the server");
        assert!(done >= closed, "the close returned before the server closed");
        // A margin for a loaded runner: the bound that matters is the other test's.
        assert!(done.duration_since(began) < CLOSE_WAIT + Duration::from_secs(20));
        server.join().expect("a server");
    });
}

#[test]
fn a_close_gives_up_on_a_server_that_keeps_it_open() {
    bounded(|| {
        assert_eq!(CLOSE_WAIT, Duration::from_secs(2));
        let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
        let port = listener.local_addr().expect("an address").port();
        let (done_tx, done_rx) = std::sync::mpsc::channel::<()>();
        let server = std::thread::spawn(move || {
            let (_s, _) = listener.accept().expect("a connection");
            let _ = done_rx.recv();
        });
        let wire = connect("127.0.0.1", Some(&port.to_string())).expect("a connection");
        let began = Instant::now();
        wire.close();
        assert!(began.elapsed() >= Duration::from_millis(1900));
        done_tx.send(()).expect("the server");
        server.join().expect("a server");
    });
}

#[test]
fn a_gone_socket_is_a_failed_capture() {
    bounded(|| {
        let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
        let port = listener.local_addr().expect("an address").port().to_string();
        let server = std::thread::spawn(move || {
            use std::io::{Read as _, Write as _};
            let (mut s, _) = listener.accept().expect("a connection");
            let mut one = [0u8; 1];
            loop {
                s.read_exact(&mut one).expect("a command");
                if one[0] == 0xFE {
                    break;
                }
            }
            s.write_all(&[1, 2, 3]).expect("a write");
        });
        let opener: Opener = Arc::new(move || connect("127.0.0.1", Some(&port)));
        let mut d = IpdbgLaDevice::new(Widths { data: 16, addr: 9 }, opener);
        d.open().expect("an open");
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).expect("a start");
        let packets = collect(&rx);
        assert_eq!(
            packets[1],
            Packet::Failed(
                "device error: The server closed the connection, at 3 of 1024 bytes.".to_owned()
            )
        );
        server.join().expect("a server");
    });
}
