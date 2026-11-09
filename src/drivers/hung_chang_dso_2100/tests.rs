// Counterpart in sigrok:
//   libsigrok/src/hardware/hung-chang-dso-2100/protocol.c,
//   Copyright (C) 2015 Daniel Glöckner <daniel-gl@gmx.net>,
//   with commits by Uwe Hermann, Daniel Elstner, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Tests of the DSO-2100 driver, on a fake scope behind a fake port.

use super::protocol::{self, Port, READOUT, SIGNATURE, STEPS, Settings, state};
use super::*;
use crate::parport::{C1284_NAUTOFD, C1284_NINIT, C1284_NSELECTIN, C1284_NSTROBE};
use std::sync::mpsc::{Receiver, sync_channel};
use std::time::Instant;

/// The scope as the lines show it (libieee1284's bits, a set bit high).
#[derive(Debug, Default)]
struct Scope {
    control: u8,
    reverse: bool,
    data: u8,
    state: u8,
    /// Position in the chain after a `0x0f`, `0x03` or `0x14`.
    chain: usize,
    /// `0x99`s that find no trigger, before one does.
    misses: u32,
    /// Reads of the mailbox that give 0 before the state, after a write.
    late: u32,
    /// Readouts with a bad signature still to come.
    bad_sigs: u32,
    /// From this mailbox write on, nothing is posted.
    silent_after: Option<u32>,
    /// The port went away: reads give 0.
    gone: bool,
    /// The port goes away at this readout strobe.
    gone_at_strobe: Option<usize>,
    pos: usize,
    writes: u32,
    /// Every mailbox write, in order.
    mailbox: Vec<u8>,
    /// Readouts and the ADC each read.
    readouts: Vec<u8>,
    released: bool,
    /// Port operations, for counting.
    ops: usize,
}

/// The chain's states from `0x0f`, as the firmware's comment orders them.
const CHAIN: [u8; 16] = [
    0x0f, 0x0f, 0x05, 0x08, 0x09, 0x06, 0x07, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x10, 0x11, 0x12, 0x13,
];

fn sample(i: usize) -> u8 {
    (i % 200) as u8 + 28
}

impl Scope {
    fn new() -> Scope {
        Scope { state: state::IDLE, control: 0x0f, ..Scope::default() }
    }

    fn high(&self, bit: u8) -> bool {
        self.control & bit != 0
    }

    fn post(&mut self, value: u8) {
        self.writes += 1;
        self.mailbox.push(value);
        if self.silent_after.is_some_and(|n| self.writes > n) {
            self.state = 0;
            return;
        }
        self.state = match self.state {
            state::IDLE => match value {
                2 => state::ARMED,
                3 => {
                    self.pos = 0;
                    state::EXTRACTED
                }
                4 => state::SCROLL,
                _ => state::IDLE,
            },
            state::ARMED => match value {
                0x99 if self.misses > 0 => {
                    self.misses -= 1;
                    state::ARMED
                }
                0x99 | 0xff => {
                    self.pos = 0;
                    state::EXTRACTED
                }
                _ => state::ARMED,
            },
            state::EXTRACTED | state::SCROLL if value == 4 || value == 0xff => {
                self.chain = 1;
                CHAIN[0]
            }
            s if CHAIN.contains(&s) => {
                if self.chain == CHAIN.len() {
                    state::IDLE
                } else {
                    self.chain += 1;
                    CHAIN[self.chain - 1]
                }
            }
            s => s,
        };
        self.late = 0;
    }

    fn lines_read(&mut self) -> u8 {
        if self.gone {
            return 0;
        }
        if !self.reverse {
            return self.data;
        }
        let (sel, init, afd, stb) = (
            self.high(C1284_NSELECTIN),
            self.high(C1284_NINIT),
            self.high(C1284_NAUTOFD),
            self.high(C1284_NSTROBE),
        );
        if sel && !init && afd && !stb {
            protocol::ID
        } else if sel && !init && !afd && !stb {
            if self.late > 0 {
                self.late -= 1;
                0
            } else {
                self.state
            }
        } else if !sel && !init && !stb {
            match self.pos {
                p if p < READOUT => sample(p),
                p if p < READOUT + 2 => SIGNATURE[p - READOUT],
                p if p == READOUT + 2 => {
                    if self.bad_sigs > 0 {
                        0
                    } else {
                        SIGNATURE[2]
                    }
                }
                _ => 0xff,
            }
        } else {
            0xff
        }
    }
}

/// A port onto a shared [`Scope`].
#[derive(Clone)]
struct FakePort(Arc<Mutex<Scope>>);

impl FakePort {
    fn new(scope: Scope) -> FakePort {
        FakePort(Arc::new(Mutex::new(scope)))
    }

    fn scope(&self) -> std::sync::MutexGuard<'_, Scope> {
        self.0.lock().expect("scope lock")
    }
}

impl Port for FakePort {
    fn read_data(&mut self) -> Result<u8> {
        let mut s = self.scope();
        s.ops += 1;
        Ok(s.lines_read())
    }

    fn write_data(&mut self, value: u8) -> Result<()> {
        let mut s = self.scope();
        s.ops += 1;
        s.data = value;
        Ok(())
    }

    fn write_control(&mut self, control: u8) -> Result<()> {
        let mut s = self.scope();
        s.ops += 1;
        let before = s.control;
        s.control = control & 0x0f;
        if s.gone {
            return Ok(());
        }
        let fell = before & C1284_NSTROBE != 0 && control & C1284_NSTROBE == 0;
        let rose = before & C1284_NSTROBE == 0 && control & C1284_NSTROBE != 0;
        let sel = control & C1284_NSELECTIN != 0;
        let init = control & C1284_NINIT != 0;
        let afd = control & C1284_NAUTOFD != 0;
        if fell && sel && init && !afd {
            assert!(!s.reverse, "mailbox strobe with the port reversed");
            let v = s.data;
            s.post(v);
        }
        if rose && !sel && !init {
            if s.pos == 0 {
                let adc = if afd { 2 } else { 1 };
                s.readouts.push(adc);
            }
            s.pos += 1;
            if s.pos == READOUT + 3 && s.bad_sigs > 0 {
                s.bad_sigs -= 1;
            }
            if s.gone_at_strobe == Some(s.pos) {
                s.gone = true;
            }
        }
        Ok(())
    }

    fn data_dir(&mut self, reverse: bool) -> Result<()> {
        let mut s = self.scope();
        s.ops += 1;
        s.reverse = reverse;
        Ok(())
    }

    fn gone(&self) -> bool {
        self.scope().gone
    }

    fn release(&mut self) {
        self.scope().released = true;
    }
}

const FAST: Times = Times { poll: Duration::from_millis(1), silent_polls: 50 };

fn device(scope: Scope) -> (Dso2100, FakePort) {
    let port = FakePort::new(scope);
    (Dso2100::on_port(Box::new(port.clone()), FAST), port)
}

/// Runs a capture to its end, on a thread with a bound.
fn capture(d: &mut Dso2100) -> Vec<Packet> {
    let (tx, rx) = sync_channel(64);
    d.start(PacketSink::new(tx, 0)).expect("start");
    collect(&rx)
}

fn collect(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let mut out = Vec::new();
    loop {
        let (_, p) = rx.recv_timeout(Duration::from_secs(30)).expect("a packet within 30 s");
        let end = p == Packet::End;
        out.push(p);
        if end {
            return out;
        }
    }
}

fn samples(packets: &[Packet]) -> Vec<f32> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some(a.samples.clone()),
            _ => None,
        })
        .flatten()
        .collect()
}

fn failed(packets: &[Packet]) -> Option<&str> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(e) => Some(e.as_str()),
        _ => None,
    })
}

#[test]
fn tables_agree() {
    assert_eq!(BUFFERSIZES.len(), STEPS.len());
    // Each buffer size is the samples of its step and those before it.
    let mut total = 0;
    for (i, s) in STEPS.iter().enumerate() {
        total += u64::from(s.num) * if s.interleave { 2 } else { 1 };
        assert_eq!(BUFFERSIZES[i], total, "step {i}");
    }
    assert_eq!(VDIVS.len(), VDIV_CODES.len());
}

#[test]
fn factors_and_digits() {
    // 1 V/div, probe 10: 10/32 V a count.
    let f = protocol::factor(10, 0x31);
    assert!((f - 0.3125).abs() < 1e-7);
    assert_eq!(protocol::digits(f), 1);
    // 10 mV/div, probe 1: 1/3200.
    let f = protocol::factor(1, 0x01);
    assert!((f - 1.0 / 3200.0).abs() < 1e-9);
    assert_eq!(protocol::digits(f), 4);
    // 5 V/div, probe 100: 15.625 V a count.
    assert_eq!(protocol::digits(protocol::factor(100, 0x33)), -1);
    assert_eq!(protocol::to_volts(&[0x80, 0x81, 0x7f], 0.5), [0.0, 0.5, -0.5]);
}

#[test]
fn check_id_and_reset() {
    let mut p = FakePort::new(Scope::new());
    assert!(protocol::check_id(&mut p).unwrap());
    assert!(!p.scope().reverse, "reset leaves the port forward");
    assert_eq!(p.scope().control, C1284_NSTROBE | C1284_NAUTOFD | C1284_NSELECTIN);
    let mut absent = FakePort::new(Scope { gone: true, ..Scope::new() });
    assert!(!protocol::check_id(&mut absent).unwrap());
}

#[test]
fn read_mbox_reads_twice_and_waits_from_the_first_zero() {
    let mut p = FakePort::new(Scope::new());
    p.data_dir(true).unwrap();
    assert_eq!(protocol::read_mbox(&mut p, protocol::POLL_WAIT).unwrap(), state::IDLE);
    // write_control x2, two reads, write_control x2.
    assert_eq!(p.scope().ops, 1 + 6);
    // A late answer at the second read is taken, however short the wait.
    p.scope().late = 1;
    assert_eq!(protocol::read_mbox(&mut p, Duration::ZERO).unwrap(), state::IDLE);
    // One that does not come: 0 after the wait.
    p.scope().late = u32::MAX;
    let t = Instant::now();
    assert_eq!(protocol::read_mbox(&mut p, Duration::from_millis(20)).unwrap(), 0);
    assert!(t.elapsed() >= Duration::from_millis(20));
}

#[test]
fn move_to_walks_the_chain_with_the_settings() {
    let mut p = FakePort::new(Scope::new());
    p.data_dir(true).unwrap();
    let s = Settings { channel: 2, rate: 7, cctl: [0x82, 0x0b], edge: 1, ..Settings::default() };
    protocol::write_mbox(&mut p, 4).unwrap();
    protocol::move_to(&mut p, state::IDLE, &s).unwrap();
    assert_eq!(
        p.scope().mailbox,
        [4, 0xff, 2, 2, 8, 2, 2, 0x82, 0x0b, 101, 2, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
    );
    protocol::move_to(&mut p, state::ARMED, &s).unwrap();
    assert_eq!(p.scope().mailbox.last(), Some(&2));
    // From armed to idle: 0xff, then 4 and the chain.
    protocol::move_to(&mut p, state::IDLE, &s).unwrap();
    assert_eq!(p.scope().mailbox[19..21], [0xff, 4]);
}

#[test]
fn move_to_gives_up() {
    let mut silent = FakePort::new(Scope { silent_after: Some(0), ..Scope::new() });
    silent.data_dir(true).unwrap();
    silent.scope().state = 0;
    // A silent scope: 40 tries of 100 ms, 0xff and 4 in turn.
    let start = Instant::now();
    let e = protocol::move_to(&mut silent, state::IDLE, &Settings::default()).unwrap_err();
    assert!(matches!(e, protocol::MoveError::Timeout(0)), "{e:?}");
    assert!(start.elapsed() >= Duration::from_secs(4));
    let m = silent.scope().mailbox.clone();
    assert_eq!(m.len(), 40);
    assert_eq!(m[..4], [0xff, 4, 0xff, 4]);
    // A state no firmware has.
    let mut q = FakePort::new(Scope::new());
    q.data_dir(true).unwrap();
    q.scope().state = 0x42;
    let e = protocol::move_to(&mut q, state::IDLE, &Settings::default()).unwrap_err();
    assert!(matches!(e, protocol::MoveError::Data(0x42)));
    // A port gone: at once.
    let mut g = FakePort::new(Scope { gone: true, ..Scope::new() });
    let start = Instant::now();
    let e = protocol::move_to(&mut g, state::IDLE, &Settings::default()).unwrap_err();
    assert!(matches!(e, protocol::MoveError::Port(Error::Disconnected)));
    assert!(start.elapsed() < Duration::from_secs(1));
}

#[test]
fn one_frame_of_a() {
    let (mut d, port) = device(Scope { misses: 2, ..Scope::new() });
    d.enable_channel(0, true).unwrap();
    let p = capture(&mut d);
    assert!(matches!(p[0], Packet::Header { .. }));
    assert_eq!(p[1], Packet::FrameBegin);
    let Packet::Analog(first) = &p[2] else { panic!("{:?}", p[2]) };
    assert_eq!(first.len(), 6);
    assert_eq!(first.channels, [0]);
    assert_eq!(first.digits, 1);
    assert_eq!(p[3], Packet::Trigger);
    let Packet::Analog(rest) = &p[4] else { panic!("{:?}", p[4]) };
    assert_eq!(rest.len(), READOUT - 6);
    assert_eq!(p[5], Packet::FrameEnd);
    assert_eq!(p[6], Packet::End);
    let volts = samples(&p);
    assert_eq!(volts.len(), READOUT);
    assert!((volts[0] - (f32::from(sample(0)) - 128.0) * 0.3125).abs() < 1e-6);
    let s = port.scope();
    assert_eq!(s.readouts, [1]);
    // Commit (4, 0xff, the chain), arm, three polls, back to idle.
    assert_eq!(s.mailbox[..2], [4, 0xff]);
    assert_eq!(s.mailbox[18..22], [2, 0x99, 0x99, 0x99]);
    assert_eq!(s.mailbox[22], 4);
    assert_eq!(s.state, state::IDLE);
}

#[test]
fn channel_b_and_buffer_1500() {
    let (mut d, port) = device(Scope::new());
    d.enable_channel(1, true).unwrap();
    d.set(&Key::Buffersize, &Value::U64(1500), None).unwrap();
    let p = capture(&mut d);
    assert_eq!(samples(&p).len(), 1500);
    assert!(failed(&p).is_none());
    let s = port.scope();
    assert_eq!(s.readouts, [2, 2]);
    // The second readout's chain: channel 2, step 100 + 1, shift 2 + 1.
    let chain = s.mailbox.iter().rposition(|&v| v == 3).unwrap();
    let before = &s.mailbox[..chain];
    let second = before.len() - 16;
    assert_eq!(before[second..second + 2], [2, 2]);
    assert_eq!(before[second + 4], 101);
    assert_eq!(before[second + 7], 3);
}

#[test]
fn interleaved_steps_read_both_adcs() {
    let (mut d, port) = device(Scope::new());
    d.enable_channel(0, true).unwrap();
    d.set(&Key::Buffersize, &Value::U64(5998), None).unwrap();
    let p = capture(&mut d);
    assert!(failed(&p).is_none(), "{:?}", failed(&p));
    assert_eq!(samples(&p).len(), 5998);
    assert_eq!(port.scope().readouts, [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2]);
}

#[test]
fn two_frames_rearm() {
    let (mut d, port) = device(Scope::new());
    d.enable_channel(0, true).unwrap();
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    let p = capture(&mut d);
    assert_eq!(p.iter().filter(|p| **p == Packet::FrameEnd).count(), 2);
    assert_eq!(port.scope().readouts.len(), 2);
}

#[test]
fn a_bad_signature_is_read_again() {
    let (mut d, port) = device(Scope { bad_sigs: 1, ..Scope::new() });
    d.enable_channel(0, true).unwrap();
    let p = capture(&mut d);
    assert!(failed(&p).is_none());
    assert_eq!(samples(&p).len(), READOUT);
    assert_eq!(port.scope().readouts.len(), 2);
}

#[test]
fn four_bad_signatures_fail() {
    let (mut d, port) = device(Scope { bad_sigs: 4, ..Scope::new() });
    d.enable_channel(0, true).unwrap();
    let p = capture(&mut d);
    assert_eq!(failed(&p), Some("device error: Failed to read frame without transfer errors"));
    assert_eq!(samples(&p), Vec::<f32>::new());
    assert_eq!(port.scope().readouts.len(), 4);
}

#[test]
fn silence_while_polling_fails_at_the_bound() {
    let (mut d, port) = device(Scope { silent_after: Some(19), ..Scope::new() });
    d.enable_channel(0, true).unwrap();
    let p = capture(&mut d);
    assert_eq!(failed(&p), Some("device error: No answer from the DSO-2100 in 50 trigger polls."));
    // The first 0x99 went unanswered and no other was written; the way
    // back to idle then writes 0xff and 4 in turn.
    let s = port.scope();
    assert_eq!(
        s.mailbox.iter().rposition(|&v| v == 0x99),
        s.mailbox.iter().position(|&v| v == 0x99)
    );
}

#[test]
fn a_port_gone_fails_at_once() {
    let (mut d, _port) = device(Scope { gone_at_strobe: Some(500), ..Scope::new() });
    d.enable_channel(0, true).unwrap();
    let t = Instant::now();
    let p = capture(&mut d);
    assert_eq!(failed(&p), Some("device disconnected"));
    assert!(t.elapsed() < Duration::from_secs(2), "no 100 ms waits");
}

#[test]
fn an_unknown_state_in_the_poll_fails() {
    let (mut d, port) = device(Scope { misses: u32::MAX, ..Scope::new() });
    d.enable_channel(0, true).unwrap();
    let (tx, rx) = sync_channel(64);
    d.start(PacketSink::new(tx, 0)).unwrap();
    wait_polls(&port, 2);
    // Armed, then the scope says scroll mode.
    port.scope().state = state::SCROLL;
    let p = collect(&rx);
    assert_eq!(failed(&p), Some("device error: Unexpected state 0x14 while checking for trigger"));
    // As in C, nothing takes the scope back: the last write is the poll's.
    assert_eq!(port.scope().mailbox.last(), Some(&0x99));
}

/// Waits, with a bound, until the capture polled `n` times.
#[allow(clippy::naive_bytecount)]
fn wait_polls(port: &FakePort, n: usize) {
    let t = Instant::now();
    while port.scope().mailbox.iter().filter(|&&v| v == 0x99).count() < n {
        assert!(t.elapsed() < Duration::from_secs(10), "polls");
        std::thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn a_stop_while_armed_aborts_to_idle() {
    let (mut d, port) = device(Scope { misses: u32::MAX, ..Scope::new() });
    d.enable_channel(0, true).unwrap();
    let (tx, rx) = sync_channel(64);
    d.start(PacketSink::new(tx, 0)).unwrap();
    wait_polls(&port, 3);
    d.stop().unwrap();
    let p = collect(&rx);
    assert!(failed(&p).is_none());
    let s = port.scope();
    let abort = s.mailbox.iter().rposition(|&v| v == 0xff).unwrap();
    assert_eq!(s.mailbox[abort + 1], 4);
    assert_eq!(s.state, state::IDLE);
    drop(s);
    d.close().unwrap();
    assert!(port.scope().released);
}

#[test]
fn start_refuses_an_odd_state() {
    let (mut d, port) = device(Scope::new());
    d.enable_channel(0, true).unwrap();
    port.scope().state = 0x42;
    let (tx, _rx) = sync_channel(64);
    let e = d.start(PacketSink::new(tx, 0)).unwrap_err();
    assert!(e.to_string().contains("0x42"), "{e}");
    // The port is back for a close.
    d.close().unwrap();
    assert!(port.scope().released);
}

#[test]
fn one_channel_at_a_time() {
    let (mut d, _port) = device(Scope::new());
    assert!(d.channels().iter().all(|c| !c.enabled));
    d.enable_channel(0, true).unwrap();
    assert!(d.enable_channel(1, true).is_err());
    d.enable_channel(0, false).unwrap();
    d.enable_channel(1, true).unwrap();
    assert_eq!(d.config.proto.channel, 2);
    assert_eq!(d.config.enabled_channel, Some(1));
}

#[test]
fn settings_as_c() {
    let (mut d, _port) = device(Scope::new());
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
    assert_eq!(d.get(&Key::Buffersize, None).unwrap(), Value::U64(1000));
    assert_eq!(d.get(&Key::TriggerSource, None).unwrap(), Value::Str("A".into()));
    assert_eq!(d.get(&Key::TriggerSlope, None).unwrap(), Value::Str("f".into()));
    assert_eq!(d.get(&Key::Vdiv, Some("B")).unwrap(), Value::Rational(1, 1));
    assert_eq!(d.get(&Key::Coupling, Some("A")).unwrap(), Value::Str("DC".into()));
    assert_eq!(d.get(&Key::ProbeFactor, Some("A")).unwrap(), Value::U64(10));
    assert_eq!(d.get(&Key::Conn, None).unwrap(), Value::Str("parport0".into()));
    assert!(d.get(&Key::Vdiv, None).is_err());
    assert!(d.get(&Key::Samplerate, Some("A")).is_err());
    d.set(&Key::Samplerate, &Value::U64(2_500), None).unwrap();
    assert_eq!(d.config.proto.rate, 18);
    assert!(d.set(&Key::Samplerate, &Value::U64(3_000_000), None).is_err());
    d.set(&Key::TriggerSource, &Value::Str("EXT".into()), None).unwrap();
    assert_eq!(d.config.proto.cctl[0], 0x71);
    d.set(&Key::Vdiv, &Value::Rational(1, 100), Some("A")).unwrap();
    assert_eq!(d.config.proto.cctl[0], 0x41);
    d.set(&Key::Coupling, &Value::Str("GND".into()), Some("A")).unwrap();
    assert_eq!(d.config.proto.cctl[0], 0x45);
    assert_eq!(d.get(&Key::Vdiv, Some("A")).unwrap(), Value::Rational(10, 1000));
    assert!(d.set(&Key::ProbeFactor, &Value::U64(0), Some("B")).is_err());
    d.set(&Key::TriggerSlope, &Value::Str("r".into()), None).unwrap();
    assert_eq!(d.config.proto.edge, 1);
    assert!(d.set(&Key::Buffersize, &Value::U64(1001), None).is_err());
    assert!(d.list(&Key::Vdiv, None).is_err());
    assert_eq!(d.list(&Key::Samplerate, None).unwrap(), ValueList::U64(SAMPLERATES.to_vec()));
}

#[test]
fn commit_from_each_kind_of_state() {
    // Idle: 4 to scroll mode, then the chain from 0xff.
    let (mut d, port) = device(Scope::new());
    d.enable_channel(0, true).unwrap();
    let mut link = d.link.lock().unwrap().take().unwrap();
    d.arm(&mut link).unwrap();
    assert_eq!(port.scope().mailbox[..2], [4, 0xff]);
    assert_eq!(port.scope().state, state::ARMED);
    // Armed already: no 4, the way back through 0xff, then armed again.
    port.scope().mailbox.clear();
    d.arm(&mut link).unwrap();
    assert_eq!(port.scope().mailbox[..2], [0xff, 4]);
    assert_eq!(port.scope().mailbox.last(), Some(&2));
    // A mailbox that reads 0 twice first: waited for, within the 20 ms.
    let (mut d, port) = device(Scope { late: 2, ..Scope::new() });
    d.enable_channel(0, true).unwrap();
    let mut link = d.link.lock().unwrap().take().unwrap();
    d.arm(&mut link).unwrap();
    assert_eq!(port.scope().mailbox[..2], [4, 0xff]);
    assert_eq!(port.scope().state, state::ARMED);
}
