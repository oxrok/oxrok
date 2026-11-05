// Counterpart in sigrok:
//   libsigrok/src/hardware/devantech-eth008/api.c,
//   libsigrok/src/hardware/devantech-eth008/protocol.c,
//   Copyright (C) 2023 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver against a fake board on a fake port, which gives its bytes up one
//! to a read, as a line does, and whose purge drops only what has arrived. No
//! read here counts on the clock to release anything: a held reply is released
//! on the Nth read after the test says so.

use super::protocol::{Link, MODELS, Model, TIMING, Timing};
use super::*;
use crate::core::PacketSink;
use crate::serial::Port;
use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Plenty, for anything that must not time out here.
const LONG: Timing = Timing {
    reply: Duration::from_secs(60),
    quiet: Duration::from_millis(500),
    settle_max: Duration::from_secs(60),
};
/// For a request that is meant to go unanswered.
const SHORT: Timing = Timing {
    reply: Duration::from_millis(20),
    quiet: Duration::from_millis(500),
    settle_max: Duration::from_secs(60),
};

#[derive(Default)]
struct Board {
    code: u8,
    outputs: u32,
    output_bytes: usize,
    inputs: u32,
    input_bytes: usize,
    /// Answers to give instead of the board's own, in turn.
    script: VecDeque<Vec<u8>>,
    /// Requests whose answers wait for a release.
    hold_next: usize,
    /// What is held back, in the order it would have gone.
    held: Vec<u8>,
    /// A held reply goes on its way at this many more reads.
    release_in: Option<usize>,
    /// On its way: one byte arrives per read.
    on_the_way: VecDeque<u8>,
    arrived: VecDeque<u8>,
    /// Every write, as it came.
    written: Vec<Vec<u8>>,
    /// Reads since the last write.
    reads_since_write: usize,
    /// Reads done at each write, before it.
    reads_before_write: Vec<usize>,
    purges: usize,
    /// Reads give an error.
    gone: bool,
    /// An answer has arrived whole by the time its write returns.
    burst: bool,
    /// What's on its way moves only every this many reads; 0 and 1 are each.
    pace: usize,
    reads: usize,
    /// When the last byte was given to a read.
    last_byte: Option<Instant>,
    /// For each write, how long after the last byte given to a read.
    gap_before_write: Vec<Duration>,
}

impl Board {
    fn answer(&mut self, request: &[u8]) -> Vec<u8> {
        if let Some(scripted) = self.script.pop_front() {
            return scripted;
        }
        match request {
            [0x10] => vec![self.code, 2, 3],
            [0x77] => vec![0, 0x04, 0xa3, 0x12, 0x34, 0x56],
            [0x78] => vec![125],
            [0x24] => self.outputs.to_le_bytes()[..self.output_bytes].to_vec(),
            [0x25] => self.inputs.to_be_bytes()[4 - self.input_bytes..].to_vec(),
            [0x32, n] => vec![0x01, *n],
            [0x20 | 0x21, _, 0] | [0x23, ..] => vec![0],
            _ => Vec::new(),
        }
    }
}

#[derive(Clone)]
struct Fake(Arc<Mutex<Board>>);

impl Fake {
    fn new(model: &Model) -> Fake {
        Fake(Arc::new(Mutex::new(Board {
            code: model.code,
            output_bytes: model.output_bytes,
            input_bytes: model.input_bytes,
            ..Board::default()
        })))
    }

    fn lock(&self) -> MutexGuard<'_, Board> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn written(&self) -> Vec<Vec<u8>> {
        self.lock().written.clone()
    }
}

impl Port for Fake {
    fn name(&self) -> &'static str {
        "fake"
    }

    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }

    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut b = self.lock();
        if b.gone {
            return Err(Error::Disconnected);
        }
        b.reads_since_write += 1;
        if let Some(n) = b.release_in {
            if n <= 1 {
                b.release_in = None;
                let held = std::mem::take(&mut b.held);
                b.on_the_way.extend(held);
            } else {
                b.release_in = Some(n - 1);
            }
        }
        b.reads += 1;
        if (b.pace <= 1 || b.reads.is_multiple_of(b.pace))
            && let Some(byte) = b.on_the_way.pop_front()
        {
            b.arrived.push_back(byte);
        }
        let Some(byte) = b.arrived.pop_front() else {
            drop(b);
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
            return Ok(0);
        };
        buf[0] = byte;
        b.last_byte = Some(Instant::now());
        Ok(1)
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut b = self.lock();
        if let Some(last) = b.last_byte {
            b.gap_before_write.push(last.elapsed());
        }
        b.written.push(data.to_vec());
        let reads = std::mem::take(&mut b.reads_since_write);
        b.reads_before_write.push(reads);
        let answer = b.answer(data);
        // What's still held goes first: a board answers in order.
        if b.hold_next > 0 || !b.held.is_empty() {
            b.hold_next = b.hold_next.saturating_sub(1);
            b.held.extend(answer);
        } else if b.burst {
            b.arrived.extend(answer);
        } else {
            b.on_the_way.extend(answer);
        }
        Ok(())
    }

    fn purge(&mut self) -> Result<()> {
        let mut b = self.lock();
        b.purges += 1;
        b.arrived.clear();
        Ok(())
    }
}

fn model(code: u8) -> &'static Model {
    Model::find(code).expect("a model")
}

/// An open device with these times.
fn device(code: u8, timing: Timing) -> (EthDevice, Fake) {
    let model = model(code);
    let fake = Fake::new(model);
    let mut device = EthDevice::from_port(model, Box::new(fake.clone()));
    device.link = Mutex::new(Link::new(None, timing));
    device.open().expect("an open");
    (device, fake)
}

fn get_on(device: &EthDevice, group: &str) -> Result<Value> {
    device.get(&Key::Enabled, Some(group))
}

fn bytes(written: &[Vec<u8>]) -> Vec<&[u8]> {
    written.iter().map(Vec::as_slice).collect()
}

fn names(device: &EthDevice) -> Vec<String> {
    device.channel_groups().iter().map(|g| g.name.clone()).collect()
}

#[test]
fn the_table_is_c_s() {
    let rows: Vec<_> = MODELS
        .iter()
        .map(|m| {
            (
                m.code,
                m.name,
                m.outputs,
                m.inputs,
                m.analog,
                m.output_bytes,
                m.input_bytes,
                m.missing,
            )
        })
        .collect();
    assert_eq!(
        rows,
        [
            (18, "ETH002", 2, 0, 0, 1, 0, 0),
            (19, "ETH008", 8, 0, 0, 1, 0, 0),
            (20, "ETH484", 16, 8, 4, 2, 2, 0x00f0),
            (21, "ETH8020", 20, 8, 8, 3, 4, 0),
            (22, "WIFI484", 16, 8, 4, 2, 2, 0x00f0),
            (24, "WIFI8020", 20, 8, 8, 3, 4, 0),
            (26, "WIFI002", 2, 0, 0, 1, 0, 0),
            (28, "WIFI008", 8, 0, 0, 1, 0, 0),
            (52, "ETH1610", 10, 16, 16, 2, 2, 0),
        ]
    );
    assert!(Model::find(0).is_none() && Model::find(23).is_none());
}

#[test]
fn the_output_masks() {
    let masks: Vec<_> = MODELS.iter().map(Model::output_mask).collect();
    assert_eq!(masks, [0x3, 0xff, 0xff0f, 0xf_ffff, 0xff0f, 0xf_ffff, 0x3, 0xff, 0x3ff]);
}

#[test]
fn the_times_are_these() {
    assert_eq!(TIMING.reply, Duration::from_secs(1));
    assert_eq!(TIMING.quiet, Duration::from_millis(300));
    assert_eq!(TIMING.settle_max, Duration::from_secs(1));
}

#[test]
fn a_probe_asks_the_module_then_the_serial_number_and_closes() {
    let fake = Fake::new(model(20));
    let device = probe(Box::new(fake.clone()), "conn").expect("a probe").expect("a board");
    assert_eq!(bytes(&fake.written()), [&[0x10u8][..], &[0x77]]);
    let info = device.info();
    assert_eq!(
        (info.vendor.as_str(), info.model.as_str(), info.version.as_str(), info.serial.as_str()),
        ("Devantech", "ETH484", "HW2 FW3", "0004a3123456")
    );
    assert_eq!(info.conn.as_deref(), Some("conn"));
    assert!(device.link().port.is_none(), "the probe keeps no port");
    assert_eq!(Arc::strong_count(&fake.0), 1, "the probe's port has gone");
}

#[test]
fn a_probe_of_an_unknown_id_asks_no_more() {
    let fake = Fake::new(model(19));
    fake.lock().code = 99;
    assert!(probe(Box::new(fake.clone()), "conn").expect("a probe").is_none());
    assert_eq!(bytes(&fake.written()), [&[0x10u8][..]]);
}

#[test]
fn a_probe_fails_on_a_short_answer_at_either_step() {
    for (script, asked) in [(vec![vec![19u8, 2]], 1), (vec![vec![19, 2, 3], vec![0; 5]], 2)] {
        let fake = Fake::new(model(19));
        fake.lock().script = script.into();
        // TIMING's reply: a short answer takes its second here.
        assert!(probe(Box::new(fake.clone()), "conn").is_err());
        assert_eq!(fake.written().len(), asked);
    }
}

#[test]
fn groups_are_c_s_and_skip_the_missing_outputs() {
    let (eth484, _) = device(20, LONG);
    assert_eq!(
        names(&eth484),
        [
            "DO1", "DO2", "DO3", "DO4", "DO9", "DO10", "DO11", "DO12", "DO13", "DO14", "DO15",
            "DO16", "DI1", "DI2", "DI3", "DI4", "DI5", "DI6", "DI7", "DI8", "AI1", "AI2", "AI3",
            "AI4", "Vsupply"
        ]
    );
    let (eth002, _) = device(18, LONG);
    assert_eq!(names(&eth002), ["DO1", "DO2", "Vsupply"]);
    let (eth1610, _) = device(52, LONG);
    let all = names(&eth1610);
    assert_eq!(all.len(), 10 + 16 + 16 + 1);
    assert_eq!(all[9], "DO10");
    assert_eq!(all[25], "DI16");
    assert_eq!(all[41], "AI16");
}

#[test]
fn options_are_c_s_by_kind_of_group() {
    let (d, _) = device(20, LONG);
    let table = |group| {
        d.options(group)
            .expect("options")
            .iter()
            .map(|o| (o.key.clone(), o.access.get, o.access.set, o.access.list))
            .collect::<Vec<_>>()
    };
    assert_eq!(table(None), [(Key::Conn, true, false, false), (Key::Enabled, false, true, false)]);
    assert_eq!(table(Some("DO1")), [(Key::Enabled, true, true, false)]);
    assert_eq!(table(Some("DI1")), [(Key::Enabled, true, false, false)]);
    assert_eq!(table(Some("AI1")), [(Key::Voltage, true, false, false)]);
    assert_eq!(table(Some("Vsupply")), [(Key::Voltage, true, false, false)]);
    assert!(matches!(d.options(Some("DO5")), Err(Error::ChannelGroup(_))));
}

#[test]
fn a_relay_reads_its_bit_from_low_byte_first_and_the_inputs_go_too() {
    let (d, fake) = device(20, LONG);
    fake.lock().outputs = 0x8001;
    fake.lock().inputs = 0x0081;
    assert_eq!(get_on(&d, "DO1").expect("DO1"), Value::Bool(true));
    assert_eq!(bytes(&fake.written()), [&[0x24u8][..], &[0x25]]);
    assert_eq!(get_on(&d, "DO2").expect("DO2"), Value::Bool(false));
    assert_eq!(get_on(&d, "DO16").expect("DO16"), Value::Bool(true));
    assert_eq!(get_on(&d, "DO15").expect("DO15"), Value::Bool(false));
}

#[test]
fn an_input_reads_its_bit_from_high_byte_first_on_every_width() {
    let (d, fake) = device(20, LONG);
    fake.lock().inputs = 0x0081;
    for (group, on) in [("DI1", true), ("DI2", false), ("DI8", true)] {
        assert_eq!(get_on(&d, group).expect(group), Value::Bool(on), "{group}");
    }
    let (d, fake) = device(21, LONG);
    fake.lock().inputs = 0xffff_ff80;
    assert_eq!(get_on(&d, "DI8").expect("DI8"), Value::Bool(true));
    assert_eq!(get_on(&d, "DI1").expect("DI1"), Value::Bool(false));
    let (d, fake) = device(52, LONG);
    fake.lock().inputs = 0x8000;
    assert_eq!(get_on(&d, "DI16").expect("DI16"), Value::Bool(true));
    assert_eq!(get_on(&d, "DI15").expect("DI15"), Value::Bool(false));
}

#[test]
fn a_board_with_no_inputs_is_not_asked_for_them() {
    let (d, fake) = device(19, LONG);
    fake.lock().outputs = 0x80;
    assert_eq!(get_on(&d, "DO8").expect("DO8"), Value::Bool(true));
    assert_eq!(bytes(&fake.written()), [&[0x24u8][..]]);
}

#[test]
fn three_byte_outputs_reach_the_twentieth() {
    let (d, fake) = device(21, LONG);
    fake.lock().outputs = 0x08_0100;
    for (group, on) in [("DO20", true), ("DO9", true), ("DO1", false), ("DO19", false)] {
        assert_eq!(get_on(&d, group).expect(group), Value::Bool(on), "{group}");
    }
}

#[test]
fn analog_is_a_plain_number_and_the_supply_is_in_volts() {
    let (d, fake) = device(20, LONG);
    assert_eq!(d.get(&Key::Voltage, Some("AI3")).expect("AI3"), Value::U64(0x0103));
    assert_eq!(d.get(&Key::Voltage, Some("Vsupply")).expect("Vsupply"), Value::F64(12.5));
    assert_eq!(bytes(&fake.written()), [&[0x32u8, 3][..], &[0x78]]);
    fake.lock().script = vec![vec![255u8]].into();
    assert_eq!(d.get(&Key::Voltage, Some("Vsupply")).expect("Vsupply"), Value::F64(25.5));
    fake.lock().script = vec![vec![0xffu8, 0xff]].into();
    assert_eq!(d.get(&Key::Voltage, Some("AI1")).expect("AI1"), Value::U64(0xffff));
}

#[test]
fn conn_is_answered_with_no_request() {
    let (d, fake) = device(19, LONG);
    assert_eq!(d.get(&Key::Conn, None).expect("conn"), Value::Str("test".to_owned()));
    assert_eq!(fake.written().len(), 0);
}

#[test]
fn what_a_group_does_not_have_is_refused_and_sends_nothing() {
    let (mut d, fake) = device(20, LONG);
    for (key, group) in [
        (Key::Enabled, None),
        (Key::Voltage, None),
        (Key::Voltage, Some("DO1")),
        (Key::Enabled, Some("AI1")),
        (Key::Enabled, Some("Vsupply")),
        (Key::Conn, Some("DO1")),
    ] {
        assert!(d.get(&key, group).is_err(), "{key:?} {group:?}");
    }
    for (key, group) in [
        (Key::Enabled, Some("DI1")),
        (Key::Enabled, Some("AI1")),
        (Key::Enabled, Some("Vsupply")),
        (Key::Voltage, None),
        (Key::Voltage, Some("AI1")),
    ] {
        assert!(d.set(&key, &Value::Bool(true), group).is_err(), "{key:?} {group:?}");
    }
    assert!(get_on(&d, "DO5").is_err());
    assert!(d.set(&Key::Enabled, &Value::Bool(true), Some("DO5")).is_err());
    assert!(d.set(&Key::Enabled, &Value::U64(1), None).is_err());
    assert_eq!(fake.written().len(), 0);
}

#[test]
fn set_one_relay_by_its_number_with_no_pulse() {
    let (mut d, fake) = device(20, LONG);
    d.set(&Key::Enabled, &Value::Bool(true), Some("DO9")).expect("on");
    d.set(&Key::Enabled, &Value::Bool(false), Some("DO16")).expect("off");
    assert_eq!(bytes(&fake.written()), [&[0x20u8, 9, 0][..], &[0x21, 16, 0]]);
}

#[test]
fn set_all_writes_the_mask_low_byte_first_at_each_width() {
    for (code, on) in [
        (18, &[0x23u8, 0x03][..]),
        (19, &[0x23, 0xff]),
        (20, &[0x23, 0x0f, 0xff]),
        (21, &[0x23, 0xff, 0xff, 0x0f]),
        (52, &[0x23, 0xff, 0x03]),
    ] {
        let (mut d, fake) = device(code, LONG);
        d.set(&Key::Enabled, &Value::Bool(true), None).expect("on");
        d.set(&Key::Enabled, &Value::Bool(false), None).expect("off");
        let off: Vec<u8> =
            std::iter::once(0x23).chain(std::iter::repeat_n(0, on.len() - 1)).collect();
        assert_eq!(fake.written(), [on.to_vec(), off], "{code}");
    }
}

#[test]
fn a_set_fails_on_any_answer_but_0() {
    for answer in [1u8, 2, 0xff] {
        let (mut d, fake) = device(19, LONG);
        fake.lock().script = vec![vec![answer]].into();
        let e = d.set(&Key::Enabled, &Value::Bool(true), None).expect_err("refused");
        assert_eq!(e.to_string(), "data is invalid");
        fake.lock().script = vec![vec![answer]].into();
        assert!(d.set(&Key::Enabled, &Value::Bool(true), Some("DO1")).is_err());
    }
}

#[test]
fn a_refused_set_leaves_the_link_in_step() {
    let (mut d, fake) = device(19, LONG);
    fake.lock().script = vec![vec![1u8]].into();
    assert!(d.set(&Key::Enabled, &Value::Bool(true), None).is_err());
    assert!(get_on(&d, "DO1").is_ok());
    // The board answered, so one read for its byte and none to wait for quiet.
    assert_eq!(fake.lock().reads_before_write[1], 1);
}

#[test]
fn an_answer_short_by_a_byte_fails() {
    let (d, fake) = device(20, SHORT);
    fake.lock().script = vec![vec![0u8]].into();
    let e = get_on(&d, "DO1").expect_err("short");
    assert_eq!(e.to_string(), "data is invalid");
    assert_eq!(bytes(&fake.written()), [&[0x24u8][..]], "no inputs asked after that");
}

#[test]
fn the_inputs_failing_fail_a_relay_s_get() {
    let (d, fake) = device(20, SHORT);
    fake.lock().script = vec![vec![1u8, 0], vec![]].into();
    assert!(get_on(&d, "DO1").is_err());
    assert_eq!(bytes(&fake.written()), [&[0x24u8][..], &[0x25]]);
}

#[test]
fn an_answer_paced_a_byte_to_a_read_is_whole() {
    let (d, fake) = device(21, LONG);
    fake.lock().outputs = 0x08_0000;
    assert_eq!(get_on(&d, "DO20").expect("DO20"), Value::Bool(true));
}

#[test]
fn a_byte_too_many_does_not_shift_the_next_answer() {
    let fake = Fake::new(model(19));
    fake.lock().burst = true;
    fake.lock().script = vec![vec![19u8, 2, 3, 0xff]].into();
    let device = probe(Box::new(fake.clone()), "conn").expect("a probe").expect("a board");
    assert_eq!(device.info().serial, "0004a3123456");
}

#[test]
fn what_a_board_has_no_channel_for_is_masked() {
    for (code, outputs, inputs, want) in [
        (20, 0xffff, 0xffff, (0xff0f, 0xff)),
        (21, 0xff_ffff, 0xffff_ffff, (0xf_ffff, 0xff)),
        (52, 0xffff, 0xffff, (0x3ff, 0xffff)),
        (18, 0xff, 0, (0x3, 0)),
    ] {
        let model = model(code);
        let fake = Fake::new(model);
        fake.lock().outputs = outputs;
        fake.lock().inputs = inputs;
        let mut link = Link::new(Some(Box::new(fake)), LONG);
        assert_eq!(link.read_state(model).expect("a read"), want, "{code}");
    }
}

#[test]
fn every_request_starts_with_a_purge() {
    let (d, fake) = device(20, LONG);
    assert!(get_on(&d, "DO1").is_ok());
    assert_eq!(fake.lock().purges, 2);
}

/// A get that goes unanswered within SHORT.reply, with its answer held and what
/// follows behind it.
fn get_goes_unanswered(d: &EthDevice, fake: &Fake) {
    fake.lock().hold_next = 1;
    let began = Instant::now();
    assert!(get_on(d, "DO1").is_err());
    assert!(began.elapsed() >= SHORT.reply, "gave up after {:?}", began.elapsed());
}

#[test]
fn a_late_answer_does_not_pass_for_the_next_get() {
    let (d, fake) = device(19, SHORT);
    fake.lock().outputs = 0x01;
    get_goes_unanswered(&d, &fake);
    // Late, and comes only once the next get has begun to wait.
    {
        let mut b = fake.lock();
        b.release_in = Some(3);
        b.outputs = 0x00;
    }
    let value = get_on(&d, "DO1").expect("the next get");
    assert_eq!(value, Value::Bool(false), "the late 0x01 was taken for this answer");
    assert_eq!(fake.written().len(), 2);
    assert!(fake.lock().held.is_empty(), "the late answer did come");
    assert!(fake.lock().arrived.is_empty());
}

#[test]
fn a_late_answer_does_not_pass_for_a_set_s() {
    let (mut d, fake) = device(19, SHORT);
    fake.lock().outputs = 0x01;
    get_goes_unanswered(&d, &fake);
    fake.lock().release_in = Some(3);
    // The late answer is 0x01, which a set would read as a failure.
    d.set(&Key::Enabled, &Value::Bool(true), None).expect("the set");
}

#[test]
fn a_late_answer_does_not_pass_for_a_reading_after_a_failed_set() {
    let (mut d, fake) = device(19, SHORT);
    fake.lock().hold_next = 1;
    assert!(d.set(&Key::Enabled, &Value::Bool(true), None).is_err());
    fake.lock().release_in = Some(3);
    // The late answer is 0, which would read as a supply of 0 V.
    assert_eq!(d.get(&Key::Voltage, Some("Vsupply")).expect("supply"), Value::F64(12.5));
}

#[test]
fn half_an_answer_then_the_rest_late_does_not_shift_the_next() {
    let (d, fake) = device(20, SHORT);
    fake.lock().outputs = 0xaaaa;
    // Half now, half held.
    fake.lock().script = vec![vec![0xaau8]].into();
    assert!(get_on(&d, "DO1").is_err());
    fake.lock().held = vec![0xaa];
    fake.lock().release_in = Some(3);
    fake.lock().outputs = 0x0000;
    assert_eq!(get_on(&d, "DO1").expect("next"), Value::Bool(false));
}

#[test]
fn the_wait_after_an_unanswered_request_comes_before_the_request() {
    let (d, fake) = device(19, SHORT);
    get_goes_unanswered(&d, &fake);
    fake.lock().held.clear();
    let began = Instant::now();
    assert!(get_on(&d, "DO1").is_ok());
    assert!(began.elapsed() >= SHORT.quiet, "quiet for only {:?}", began.elapsed());
    // Quiet ends it, well before settle_max's minute. A stall of 30 s fails this.
    assert!(began.elapsed() < Duration::from_secs(30), "{:?}", began.elapsed());
}

/// Each byte is well inside `quiet` of the one before, and all of them take
/// longer than it. Whenever the next request goes, by then the line has been
/// quiet for all of `quiet`, however slow this host: a stall only stretches a gap.
#[test]
fn what_keeps_coming_keeps_the_next_request_waiting() {
    let timing = Timing {
        reply: Duration::from_millis(20),
        quiet: Duration::from_millis(50),
        settle_max: Duration::from_secs(60),
    };
    let (d, fake) = device(19, timing);
    fake.lock().hold_next = 1;
    assert!(get_on(&d, "DO1").is_err());
    {
        let mut b = fake.lock();
        b.held = vec![0x55; 100];
        b.pace = 5;
        b.release_in = Some(1);
        b.last_byte = None;
        b.gap_before_write.clear();
    }
    let _ = get_on(&d, "DO1");
    let b = fake.lock();
    assert_eq!(b.written.len(), 2);
    assert_eq!(b.gap_before_write.len(), 1, "a byte came before the request");
    assert!(b.gap_before_write[0] >= timing.quiet, "{:?}", b.gap_before_write[0]);
}

#[test]
fn only_the_next_request_waits() {
    let (d, fake) = device(19, SHORT);
    get_goes_unanswered(&d, &fake);
    fake.lock().held.clear();
    assert!(get_on(&d, "DO1").is_ok());
    assert!(get_on(&d, "DO1").is_ok());
    // The wait reads the line many times; a request in step, once for its byte.
    let b = fake.lock();
    assert!(b.reads_before_write[1] > 3, "{}", b.reads_before_write[1]);
    assert_eq!(b.reads_before_write[2], 1);
}

#[test]
fn a_line_that_never_falls_silent_holds_a_request_only_so_long() {
    let timing = Timing {
        reply: Duration::from_millis(20),
        quiet: Duration::from_secs(30),
        settle_max: Duration::from_millis(100),
    };
    let (d, fake) = device(19, timing);
    fake.lock().hold_next = 1;
    assert!(get_on(&d, "DO1").is_err());
    fake.lock().held.clear();
    let began = Instant::now();
    assert!(get_on(&d, "DO1").is_ok());
    assert!(began.elapsed() >= Duration::from_millis(100));
    assert!(began.elapsed() < Duration::from_secs(15), "{:?}", began.elapsed());
}

#[test]
fn an_open_is_in_step_again() {
    let model = model(19);
    let (mut d, fake) = device(19, SHORT);
    get_goes_unanswered(&d, &fake);
    d.close().expect("a close");
    assert!(d.link().port.is_none());
    let again = Fake::new(model);
    d.pending = Some(Box::new(again.clone()));
    d.open().expect("an open");
    assert!(get_on(&d, "DO1").is_ok());
    assert_eq!(again.lock().reads_before_write[0], 0, "no wait on a new port");
}

#[test]
fn a_closed_device_refuses_and_a_gone_port_says_so() {
    let (mut d, fake) = device(19, LONG);
    fake.lock().gone = true;
    assert!(matches!(get_on(&d, "DO1"), Err(Error::Disconnected)));
    d.close().expect("a close");
    assert!(get_on(&d, "DO1").is_err());
    assert!(d.set(&Key::Enabled, &Value::Bool(true), None).is_err());
}

#[test]
fn a_port_given_back_after_a_failure_still_serves() {
    let (d, fake) = device(19, SHORT);
    fake.lock().script = vec![vec![]].into();
    assert!(get_on(&d, "DO1").is_err());
    assert!(d.link().port.is_some());
    assert!(get_on(&d, "DO1").is_ok());
}

#[test]
fn a_capture_starts_and_ends_at_once() {
    let (mut d, _fake) = device(19, LONG);
    let (tx, packets) = std::sync::mpsc::sync_channel(8);
    d.start(PacketSink::new(tx, 0)).expect("a start");
    let mut got = Vec::new();
    while let Ok((_, packet)) = packets.recv_timeout(Duration::from_secs(20)) {
        let end = matches!(packet, Packet::End);
        got.push(packet);
        if end {
            break;
        }
    }
    assert_eq!(got.len(), 2);
    assert!(matches!(got[0], Packet::Header { .. }));
    d.stop().expect("a stop");
}

#[test]
fn no_conn_or_an_empty_one_finds_nothing_and_a_bad_port_too() {
    let driver = DevantechEth008Driver;
    assert!(driver.scan(&[]).expect("a scan").is_empty());
    let conn = |c: &str| vec![(Key::Conn, Value::Str(c.to_owned()))];
    assert!(driver.scan(&conn("")).expect("a scan").is_empty());
    assert!(driver.scan(&conn("/dev/nonesuch-eth008")).expect("a scan").is_empty());
    assert!(driver.scan(&conn("tcp-raw/127.0.0.1")).expect("a scan").is_empty());
}

#[test]
fn a_scan_and_a_get_over_tcp() {
    use std::io::{Read, Write};
    let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("a listener");
    let port = listener.local_addr().expect("an address").port();
    // One connection for the scan, one for the open, as a board takes up to five.
    let board = std::thread::spawn(move || {
        for _ in 0..2 {
            let (mut s, _) = listener.accept().expect("a connection");
            let mut request = [0u8; 8];
            loop {
                match s.read(&mut request) {
                    Ok(0) | Err(_) => break,
                    Ok(_) => {}
                }
                let answer: &[u8] = match request[0] {
                    0x10 => &[19, 2, 3],
                    0x77 => &[0, 4, 0xa3, 0x12, 0x34, 0x56],
                    0x24 => &[0x80],
                    _ => &[],
                };
                s.write_all(answer).expect("an answer");
            }
        }
    });
    let conn = format!("tcp-raw/127.0.0.1/{port}");
    let driver = DevantechEth008Driver;
    let mut devices = driver.scan(&[(Key::Conn, Value::Str(conn.clone()))]).expect("a scan");
    assert_eq!(devices.len(), 1);
    let mut device = devices.pop().expect("a board");
    assert_eq!(device.info().conn.as_deref(), Some(conn.as_str()));
    device.open().expect("an open");
    assert_eq!(device.get(&Key::Enabled, Some("DO8")).expect("a get"), Value::Bool(true));
    device.close().expect("a close");
    drop(device);
    board.join().expect("the board");
}
