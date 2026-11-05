// Counterpart in sigrok:
//   libsigrok/src/hardware/maynuo-m97/api.c,
//   libsigrok/src/hardware/maynuo-m97/protocol.c,
//   Copyright (C) 2015 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, François Revol,
//   Gerhard Sittig, Daniel Elstner, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake port: a load's coils and registers, as Modbus frames.

use super::*;
use crate::modbus::{crc16, frame};
use std::collections::{HashMap, VecDeque};
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test waits for a capture before it fails: a stop or a limit
/// that does not bite must fail, not hang the run.
const CAPTURE_BOUND: Duration = Duration::from_secs(20);

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Good,
    Silent,
    OtherSlave,
    NoWrite,
}

/// A load, as its frames show it.
struct State {
    writes: Vec<(Instant, Vec<u8>)>,
    registers: HashMap<u16, u16>,
    /// What a coil read gives, by address.
    coils: HashMap<u16, u8>,
    mode: Mode,
    /// Refuse the frames that are numbered here, counting from 1.
    refuse: Vec<usize>,
}

type Shared = Arc<Mutex<State>>;

struct Fake {
    state: Shared,
    input: VecDeque<u8>,
}

impl Fake {
    fn new() -> (Fake, Shared) {
        let state = Arc::new(Mutex::new(State {
            writes: Vec::new(),
            registers: HashMap::new(),
            coils: HashMap::new(),
            mode: Mode::Good,
            refuse: Vec::new(),
        }));
        (Fake { state: Arc::clone(&state), input: VecDeque::new() }, state)
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
        if self.input.is_empty() {
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
            return Ok(0);
        }
        let n = buf.len().min(self.input.len());
        for b in &mut buf[..n] {
            *b = self.input.pop_front().expect("counted");
        }
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut state = self.state.lock().unwrap();
        state.writes.push((Instant::now(), data.to_vec()));
        let number = state.writes.len();
        match state.mode {
            Mode::NoWrite => return Err(Error::device("Broken pipe")),
            Mode::Silent => return Ok(()),
            Mode::Good | Mode::OtherSlave => {}
        }
        let (body, crc) = data.split_at(data.len() - 2);
        assert_eq!(crc16(body).to_le_bytes(), crc, "a frame's CRC");
        let at = u16::from_be_bytes([body[2], body[3]]);
        let count = u16::from_be_bytes([body[4], body[5]]);
        let out = if state.refuse.contains(&number) {
            vec![body[1] | 0x80, 4]
        } else {
            match body[1] {
                1 => vec![1, 1, state.coils.get(&at).copied().unwrap_or(0)],
                3 => {
                    let mut out = vec![3, u8::try_from(count * 2).unwrap()];
                    for r in at..at + count {
                        let value = state.registers.get(&r).copied().unwrap_or(0);
                        out.extend_from_slice(&value.to_be_bytes());
                    }
                    out
                }
                5 => body[1..6].to_vec(),
                0x10 => {
                    for i in 0..count {
                        let word = 7 + 2 * usize::from(i);
                        let value = u16::from_be_bytes([body[word], body[word + 1]]);
                        state.registers.insert(at + i, value);
                    }
                    body[1..6].to_vec()
                }
                other => panic!("function {other}"),
            }
        };
        let slave = if state.mode == Mode::OtherSlave { 2 } else { 1 };
        drop(state);
        self.input.extend(frame(slave, &out));
        Ok(())
    }
    fn purge(&mut self) -> Result<()> {
        self.input.clear();
        Ok(())
    }
}

fn model() -> &'static Model {
    protocol::lookup(101).unwrap()
}

fn closed(fake: Fake) -> MaynuoDevice {
    MaynuoDevice::from_port(model(), Box::new(fake))
}

fn opened() -> (MaynuoDevice, Shared) {
    let (fake, state) = Fake::new();
    let mut d = closed(fake);
    d.open().unwrap();
    (d, state)
}

fn writes(state: &Shared) -> Vec<Vec<u8>> {
    state.lock().unwrap().writes.iter().map(|(_, w)| w.clone()).collect()
}

fn set_float_at(state: &Shared, at: u16, value: f32) {
    let words = protocol::float_to(value);
    let mut s = state.lock().unwrap();
    s.registers.insert(at, words[0]);
    s.registers.insert(at + 1, words[1]);
}

fn set_mode(state: &Shared, mode: Mode) {
    state.lock().unwrap().mode = mode;
}

const PC1_ON: [u8; 8] = [0x01, 0x05, 0x05, 0x00, 0xff, 0x00, 0x8c, 0xf6];
const PC1_OFF: [u8; 8] = [0x01, 0x05, 0x05, 0x00, 0x00, 0x00, 0xcd, 0x06];
/// A read of four registers from `U`, 0x0B00.
const POLL: [u8; 6] = [0x01, 0x03, 0x0b, 0x00, 0x00, 0x04];

fn poll_frame() -> Vec<u8> {
    frame(1, &POLL[1..])
}

fn float_write(at: u16, value: f32) -> Vec<u8> {
    let [ah, al] = at.to_be_bytes();
    let b = value.to_be_bytes();
    frame(1, &[0x10, ah, al, 0, 2, 4, b[0], b[1], b[2], b[3]])
}

fn command(value: u16) -> Vec<u8> {
    let [vh, vl] = value.to_be_bytes();
    frame(1, &[0x10, 0x0a, 0x00, 0, 1, 2, vh, vl])
}

fn packets_until_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let deadline = Instant::now() + CAPTURE_BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        assert!(!left.is_zero(), "no End after {CAPTURE_BOUND:?}");
        let (_, p) = rx.recv_timeout(left).expect("a packet in time");
        let done = matches!(p, Packet::End);
        out.push(p);
        if done {
            return out;
        }
    }
}

fn capture(d: &mut MaynuoDevice) -> Vec<Packet> {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    packets_until_end(&rx)
}

fn wait_for_frame(rx: &Receiver<(usize, Packet)>) {
    let deadline = Instant::now() + CAPTURE_BOUND;
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        assert!(!left.is_zero(), "no frame after {CAPTURE_BOUND:?}");
        let (_, p) = rx.recv_timeout(left).expect("a packet in time");
        assert!(!matches!(p, Packet::End), "the capture ended");
        if matches!(p, Packet::FrameEnd) {
            return;
        }
    }
}

fn frames_in(packets: &[Packet]) -> usize {
    packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count()
}

#[test]
fn the_identity_shows_as_the_scan_leaves_it() {
    let d =
        MaynuoDevice::new(model(), 123, "/dev/x".to_owned(), Params::parse(SERIALCOMM).unwrap(), 1);
    assert_eq!(d.info().vendor, "Maynuo");
    assert_eq!(d.info().model, "M9812");
    assert_eq!(d.info().version, "v12.3");
    assert_eq!(d.channels().iter().map(|c| c.name.as_str()).collect::<Vec<_>>(), ["V1", "I1"]);
    assert_eq!(d.channel_groups(), [ChannelGroup::new("1", vec![0, 1])]);
    assert_eq!(d.classes(), [DeviceClass::ElectronicLoad]);
}

#[test]
fn an_open_sets_pc1_and_a_close_clears_it() {
    let (mut d, state) = opened();
    assert_eq!(writes(&state), [PC1_ON.to_vec()]);
    d.close().unwrap();
    assert_eq!(writes(&state), [PC1_ON.to_vec(), PC1_OFF.to_vec()]);
    d.close().unwrap();
    assert_eq!(writes(&state).len(), 2, "a second close has no port to write to");
}

#[test]
fn nothing_looks_at_how_pc1_went() {
    let (fake, state) = Fake::new();
    state.lock().unwrap().refuse = vec![1, 2];
    let mut d = closed(fake);
    d.open().unwrap();
    d.close().unwrap();
    assert_eq!(writes(&state), [PC1_ON.to_vec(), PC1_OFF.to_vec()]);
}

#[test]
fn a_device_dropped_open_clears_pc1_once() {
    let (d, state) = opened();
    drop(d);
    assert_eq!(writes(&state), [PC1_ON.to_vec(), PC1_OFF.to_vec()]);
    let (mut d, state) = opened();
    d.close().unwrap();
    drop(d);
    assert_eq!(writes(&state).len(), 2);
}

#[test]
fn a_device_never_opened_sends_nothing() {
    let (fake, state) = Fake::new();
    drop(closed(fake));
    assert_eq!(writes(&state).len(), 0);
}

#[test]
fn a_setting_goes_to_its_register_as_a_float() {
    let (mut d, state) = opened();
    d.set(&Key::VoltageTarget, &Value::F64(3.3), Some("1")).unwrap();
    d.set(&Key::CurrentLimit, &Value::F64(1.25), Some("1")).unwrap();
    d.set(&Key::OverVoltageProtectionThreshold, &Value::F64(62.5), Some("1")).unwrap();
    d.set(&Key::OverCurrentProtectionThreshold, &Value::F64(6.5), Some("1")).unwrap();
    assert_eq!(
        writes(&state)[1..],
        [
            float_write(0x0a03, 3.3),
            float_write(0x0a01, 1.25),
            float_write(0x0a36, 62.5),
            float_write(0x0a34, 6.5),
        ]
    );
}

#[test]
fn a_value_out_of_the_list_goes_out_and_one_no_number_does_not() {
    let (mut d, state) = opened();
    d.set(&Key::VoltageTarget, &Value::F64(151.0), Some("1")).unwrap();
    d.set(&Key::CurrentLimit, &Value::F64(-1.0), Some("1")).unwrap();
    assert_eq!(writes(&state)[1..], [float_write(0x0a03, 151.0), float_write(0x0a01, -1.0)]);
    let before = writes(&state).len();
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 1e39] {
        for key in [
            Key::VoltageTarget,
            Key::CurrentLimit,
            Key::OverVoltageProtectionThreshold,
            Key::OverCurrentProtectionThreshold,
        ] {
            assert!(d.set(&key, &Value::F64(value), Some("1")).is_err(), "{key:?} {value}");
        }
    }
    assert!(d.set(&Key::VoltageTarget, &Value::from("5"), Some("1")).is_err());
    assert_eq!(writes(&state).len(), before);
}

#[test]
fn the_input_reads_the_mode_and_writes_it_back() {
    let (mut d, state) = opened();
    state.lock().unwrap().registers.insert(protocol::SETMODE, 0x0302);
    d.set(&Key::Enabled, &Value::Bool(true), Some("1")).unwrap();
    d.set(&Key::Enabled, &Value::Bool(false), Some("1")).unwrap();
    let mode_read = frame(1, &[3, 0x0b, 0x04, 0, 1]);
    assert_eq!(
        writes(&state)[1..],
        [mode_read.clone(), command(42), command(2), mode_read, command(43), command(2),],
        "the low byte of the mode, after the input"
    );
}

#[test]
fn the_input_ends_where_it_is_refused() {
    let mode_read = frame(1, &[3, 0x0b, 0x04, 0, 1]);
    for (step, sent) in [(2, 1), (3, 2), (4, 3)] {
        let (mut d, state) = opened();
        state.lock().unwrap().refuse = vec![step];
        assert!(d.set(&Key::Enabled, &Value::Bool(true), Some("1")).is_err(), "{step}");
        let all = [mode_read.clone(), command(42), command(0)];
        assert_eq!(writes(&state)[1..], all[..sent], "{step}");
    }
}

#[test]
fn each_key_reads_its_place() {
    let (d, state) = opened();
    set_float_at(&state, protocol::U, 12.5);
    set_float_at(&state, protocol::UFIX, 13.5);
    set_float_at(&state, protocol::I, 1.5);
    set_float_at(&state, protocol::IFIX, 2.5);
    set_float_at(&state, protocol::UMAX, 100.0);
    set_float_at(&state, protocol::IMAX, 20.0);
    for (key, value) in [
        (Key::Voltage, 12.5),
        (Key::VoltageTarget, 13.5),
        (Key::Current, 1.5),
        (Key::CurrentLimit, 2.5),
        (Key::OverVoltageProtectionThreshold, 100.0),
        (Key::OverCurrentProtectionThreshold, 20.0),
    ] {
        assert_eq!(d.get(&key, Some("1")).unwrap(), Value::F64(value), "{key:?}");
    }
    for (key, coil) in [
        (Key::Enabled, protocol::ISTATE),
        (Key::OverVoltageProtectionActive, protocol::UOVER),
        (Key::OverCurrentProtectionActive, protocol::IOVER),
        (Key::OverTemperatureProtectionActive, protocol::HEAT),
    ] {
        assert_eq!(d.get(&key, Some("1")).unwrap(), Value::Bool(false), "{key:?}");
        state.lock().unwrap().coils.insert(coil, 1);
        assert_eq!(d.get(&key, Some("1")).unwrap(), Value::Bool(true), "{key:?}");
    }
    let before = writes(&state).len();
    for key in [
        Key::OverVoltageProtectionEnabled,
        Key::OverCurrentProtectionEnabled,
        Key::OverTemperatureProtection,
    ] {
        assert_eq!(d.get(&key, Some("1")).unwrap(), Value::Bool(true), "{key:?}");
    }
    assert_eq!(writes(&state).len(), before, "these ask nothing");
}

#[test]
fn a_coil_is_the_lowest_bit_of_the_byte() {
    // The manual's example: 0x48 is ISTATE off.
    let (d, state) = opened();
    state.lock().unwrap().coils.insert(protocol::ISTATE, 0x48);
    assert_eq!(d.get(&Key::Enabled, Some("1")).unwrap(), Value::Bool(false));
    state.lock().unwrap().coils.insert(protocol::ISTATE, 0x49);
    assert_eq!(d.get(&Key::Enabled, Some("1")).unwrap(), Value::Bool(true));
}

#[test]
fn a_regulation_is_unreg_and_then_the_mode() {
    let (d, state) = opened();
    state.lock().unwrap().registers.insert(protocol::SETMODE, 2);
    assert_eq!(d.get(&Key::Regulation, Some("1")).unwrap(), Value::from("CV"));
    assert_eq!(
        writes(&state)[1..],
        [frame(1, &[1, 0x05, 0x25, 0, 1]), frame(1, &[3, 0x0b, 0x04, 0, 1])]
    );
    state.lock().unwrap().coils.insert(protocol::UNREG, 1);
    let before = writes(&state).len();
    assert_eq!(d.get(&Key::Regulation, Some("1")).unwrap(), Value::from("UR"));
    assert_eq!(writes(&state).len(), before + 1, "no mode to ask for");
}

#[test]
fn a_failed_get_says_so() {
    let (d, state) = opened();
    state.lock().unwrap().refuse = vec![2];
    assert!(d.get(&Key::Voltage, Some("1")).is_err());
    let (d, state) = opened();
    state.lock().unwrap().refuse = vec![3];
    assert!(d.get(&Key::Regulation, Some("1")).is_err(), "the mode's read");
}

#[test]
fn the_options_are_the_devices_and_the_groups() {
    let (d, _) = opened();
    let keys = |group| d.options(group).unwrap().iter().map(|o| o.key.clone()).collect::<Vec<_>>();
    assert_eq!(keys(None), [Key::Continuous, Key::LimitSamples, Key::LimitMsec]);
    assert_eq!(keys(Some("1")).len(), 14);
    assert_eq!(keys(Some("1"))[0], Key::Enabled);
    assert!(matches!(d.options(Some("2")), Err(Error::ChannelGroup(_))));
}

#[test]
fn a_list_has_the_models_limit_and_its_step() {
    let (d, _) = opened();
    assert_eq!(
        d.list(&Key::VoltageTarget, Some("1")).unwrap(),
        ValueList::RangeStepF64 { min: 0.0, max: 150.0, step: 0.001 }
    );
    assert_eq!(
        d.list(&Key::CurrentLimit, Some("1")).unwrap(),
        ValueList::RangeStepF64 { min: 0.0, max: 30.0, step: 0.0001 }
    );
    assert!(d.list(&Key::VoltageTarget, None).is_err());
    assert!(d.list(&Key::Voltage, Some("1")).is_err());
}

#[test]
fn limits_are_kept_and_0_is_refused() {
    let (mut d, _) = opened();
    d.set(&Key::LimitSamples, &Value::U64(7), None).unwrap();
    d.set(&Key::LimitMsec, &Value::U64(9), None).unwrap();
    assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(7));
    assert_eq!(d.get(&Key::LimitMsec, None).unwrap(), Value::U64(9));
    assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::LimitMsec, &Value::U64(0), None).is_err());
}

#[test]
fn a_capture_asks_each_time_and_ends_at_its_limit() {
    let (mut d, state) = opened();
    set_float_at(&state, protocol::U, 12.5);
    set_float_at(&state, protocol::I, 1.25);
    d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
    let packets = capture(&mut d);
    assert_eq!(frames_in(&packets), 3);
    assert_eq!(
        writes(&state)[1..],
        [poll_frame(), poll_frame(), poll_frame()],
        "four registers from U, three times and no more"
    );
    assert!(matches!(packets[0], Packet::Header { .. }));
    assert!(matches!(packets[1], Packet::FrameBegin));
    let (Packet::Analog(v), Packet::Analog(i)) = (&packets[2], &packets[3]) else {
        panic!("{packets:?}");
    };
    assert_eq!(
        (v.channels.as_slice(), v.samples.as_slice(), v.mq, v.unit, v.flags, v.digits),
        (&[0][..], &[12.5][..], Mq::Voltage, Unit::Volt, MqFlags::DC, 3)
    );
    assert_eq!(
        (i.channels.as_slice(), i.samples.as_slice(), i.mq, i.unit, i.flags, i.digits),
        (&[1][..], &[1.25][..], Mq::Current, Unit::Ampere, MqFlags::DC, 4)
    );
    assert!(matches!(packets[4], Packet::FrameEnd));
    assert!(matches!(packets.last(), Some(Packet::End)));
    d.close().unwrap();
    assert_eq!(*writes(&state).last().unwrap(), PC1_OFF.to_vec());
}

#[test]
fn the_polls_keep_their_distance() {
    let (mut d, state) = opened();
    d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
    capture(&mut d);
    let state = state.lock().unwrap();
    let at: Vec<Instant> = state.writes.iter().map(|(t, _)| *t).collect();
    // A loaded box only stretches a gap: the least is what is asserted.
    let least = at[1..].windows(2).map(|w| w[1].duration_since(w[0])).min().unwrap();
    assert!(least >= Duration::from_millis(10), "{least:?}");
}

#[test]
fn a_start_needs_an_open_device() {
    let (fake, _) = Fake::new();
    let mut d = closed(fake);
    let (tx, rx) = sync_channel(4);
    assert!(d.start(PacketSink::new(tx, 0)).is_err());
    assert!(rx.try_recv().is_err(), "no Header");
}

#[test]
fn a_time_limit_ends_a_capture() {
    let (mut d, _) = opened();
    d.set(&Key::LimitMsec, &Value::U64(100), None).unwrap();
    let started = Instant::now();
    let packets = capture(&mut d);
    assert!(started.elapsed() >= Duration::from_millis(100));
    assert!(frames_in(&packets) >= 1);
}

#[test]
fn a_stop_ends_a_capture_that_has_data() {
    let (mut d, state) = opened();
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    wait_for_frame(&rx);
    d.stop().unwrap();
    packets_until_end(&rx);
    let after = writes(&state).len();
    std::thread::sleep(Duration::from_millis(40));
    assert_eq!(writes(&state).len(), after, "no read after the End");
}

#[test]
fn a_close_does_not_hang_on_a_queue_no_one_reads() {
    let (mut d, state) = opened();
    // The Header fills it, and the first frame's packets cannot go.
    let (tx, rx) = sync_channel(1);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let deadline = Instant::now() + CAPTURE_BOUND;
    while writes(&state).len() < 2 {
        assert!(Instant::now() < deadline, "no poll");
        std::thread::sleep(Duration::from_millis(1));
    }
    let (done_tx, done_rx) = sync_channel(1);
    std::thread::spawn(move || {
        d.close().unwrap();
        done_tx.send(()).unwrap();
    });
    done_rx.recv_timeout(CAPTURE_BOUND).expect("the close came back");
    assert_eq!(*writes(&state).last().unwrap(), PC1_OFF.to_vec());
    drop(rx);
}

#[test]
fn a_setting_gets_in_during_a_capture() {
    let (mut d, state) = opened();
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    wait_for_frame(&rx);
    d.set(&Key::CurrentLimit, &Value::F64(2.0), Some("1")).unwrap();
    assert!(writes(&state).contains(&float_write(0x0a01, 2.0)));
    wait_for_frame(&rx);
    d.stop().unwrap();
    packets_until_end(&rx);
}

#[test]
fn a_sample_limit_on_a_silent_load_ends_at_the_bound() {
    let (mut d, state) = opened();
    set_mode(&state, Mode::Silent);
    d.no_reply_bound = Duration::from_millis(1200);
    d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    let started = Instant::now();
    let packets = capture(&mut d);
    let took = started.elapsed();
    assert!(took >= Duration::from_millis(1200), "{took:?}");
    assert_eq!(frames_in(&packets), 0);
    // At 500 ms each, a read that passes 1.2 s is the third.
    assert_eq!(writes(&state).len(), 1 + 3);
}

#[test]
fn the_bound_is_five_seconds_and_in_time_not_in_reads() {
    assert_eq!(NO_REPLY_BOUND, Duration::from_secs(5));
    // Another slave's reply fails at its first byte: failed reads come in
    // milliseconds.
    let (mut d, state) = opened();
    d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
    d.no_reply_bound = Duration::from_millis(600);
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    wait_for_frame(&rx);
    set_mode(&state, Mode::OtherSlave);
    let started = Instant::now();
    packets_until_end(&rx);
    assert!(started.elapsed() >= Duration::from_millis(590), "{:?}", started.elapsed());
}

#[test]
fn a_time_limit_goes_past_the_bound_and_recovers() {
    let (mut d, state) = opened();
    d.set(&Key::LimitMsec, &Value::U64(2000), None).unwrap();
    d.no_reply_bound = Duration::from_millis(100);
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    wait_for_frame(&rx);
    set_mode(&state, Mode::Silent);
    std::thread::sleep(Duration::from_millis(700));
    while rx.try_recv().is_ok() {}
    set_mode(&state, Mode::Good);
    wait_for_frame(&rx);
    packets_until_end(&rx);
}

#[test]
fn continuous_goes_on_through_a_quiet_and_recovers() {
    let (mut d, state) = opened();
    d.no_reply_bound = Duration::from_millis(100);
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    wait_for_frame(&rx);
    set_mode(&state, Mode::Silent);
    std::thread::sleep(Duration::from_millis(700));
    while rx.try_recv().is_ok() {}
    set_mode(&state, Mode::Good);
    wait_for_frame(&rx);
    d.stop().unwrap();
    packets_until_end(&rx);
}

#[test]
fn requests_that_cannot_go_out_end_any_capture() {
    assert_eq!(MAX_UNSENT_POLLS, 5);
    let (mut d, state) = opened();
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    wait_for_frame(&rx);
    set_mode(&state, Mode::NoWrite);
    let before = writes(&state).len();
    packets_until_end(&rx);
    let tries = writes(&state).len() - before;
    assert!((5..=7).contains(&tries), "{tries}");
}

#[test]
fn a_failed_read_takes_no_sample_and_the_next_does() {
    let (mut d, state) = opened();
    // The second poll is refused.
    state.lock().unwrap().refuse = vec![3];
    d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    let packets = capture(&mut d);
    assert_eq!(frames_in(&packets), 2);
    assert_eq!(writes(&state)[1..], [poll_frame(), poll_frame(), poll_frame()]);
}
