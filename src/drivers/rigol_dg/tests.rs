// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later

//! The driver on a fake instrument: what the model can't show, as it answers at once
//! and has no clock. Faults are played once, by the question they answer.

#![allow(clippy::float_cmp)]

use super::protocol::{self, CREDIT_CAP, MODELS};
use super::*;
use crate::scpi::ScpiTransport;
use crate::serial::WriteLog;
use std::collections::{HashMap, VecDeque};
use std::sync::mpsc::{Receiver, sync_channel};

/// A test's bound on anything that could hang.
const BOUND: Duration = Duration::from_secs(30);

/// One answer in a fake's script.
#[derive(Clone, Debug)]
enum Say {
    /// This text and a line end.
    Text(&'static str),
    /// Nothing.
    Silent,
    /// This text, after that many more reads of the line: a late answer.
    Late(usize, &'static str),
    /// This text, when the line is next looked at to drop what came: late past the
    /// read, in time for the drain, whatever the clock says.
    ForTheDrain(&'static str),
    /// This text at once, and the second behind it by the next read: an answer owed.
    TextThenOwed(&'static str, &'static str),
    /// The line is gone.
    Gone,
}

#[derive(Default)]
#[allow(clippy::struct_excessive_bools)]
struct Line {
    written: WriteLog,
    script: HashMap<&'static str, VecDeque<Say>>,
    arrived: VecDeque<u8>,
    coming: Vec<(usize, Vec<u8>)>,
    for_the_drain: Vec<u8>,
    complete: bool,
    readings: u32,
    connected: bool,
    writes_while_closed: usize,
    gone: bool,
    /// Answers carry a tag, as USBTMC's do.
    tags: bool,
    /// What a read takes when it has nothing: a stand-in for a real line's wait.
    wait: Duration,
    /// Each query's answer waits this long on the clock: the real pace of a line.
    pace: Duration,
    paced_until: Option<Instant>,
}

impl Line {
    fn tick(&mut self) {
        let mut due = Vec::new();
        self.coming.retain_mut(|(reads, bytes)| {
            if *reads == 0 {
                due.append(bytes);
                return false;
            }
            *reads -= 1;
            true
        });
        self.arrived.extend(due);
    }

    fn answer_of(&mut self, query: &str) -> Vec<u8> {
        match query {
            "*OPC?" | "SOUR1:OUTPUT:STATE?" => b"1\n".to_vec(),
            "COUN:STAT?" | "OUTP1:STATE?" | "OUTP2:STATE?" => b"OFF\n".to_vec(),
            "COUN:MEAS?" => {
                self.readings += 1;
                let f = 1000 + self.readings;
                format!("{f}.0,1.0E-3,50.0,5.0E-4,5.0E-4\n").into_bytes()
            }
            "*IDN?" => b"Rigol Technologies,DG1062Z,S1,1.0\n".to_vec(),
            "SOUR1:APPL?" | "SOUR2:APPL?" => {
                b"\"SIN,1.000000E+03,5.000000E+00,0.000000E+00,0.000000E+00\"\n".to_vec()
            }
            "SOUR1:FUNC?" => b"SIN\n".to_vec(),
            "SOUR1:FREQ?" | "SOUR1:VOLT?" | "SOUR1:VOLT:OFFS?" | "SOUR1:PHAS?" => {
                b"2.000000E+00\n".to_vec()
            }
            // Limits, the model's stand-ins.
            q if q.ends_with(":VOLT? MIN") => b"2.000000E-03\n".to_vec(),
            q if q.ends_with(":VOLT? MAX") => b"1.000000E+01\n".to_vec(),
            q if q.ends_with(":VOLT:OFFS? MIN") => b"-2.500000E+00\n".to_vec(),
            q if q.ends_with(":VOLT:OFFS? MAX") => b"2.500000E+00\n".to_vec(),
            q if q.ends_with(":DCYC? MIN") => b"1.000000E+01\n".to_vec(),
            q if q.ends_with(":DCYC? MAX") => b"9.000000E+01\n".to_vec(),
            other => panic!("the fake has no answer to {other}"),
        }
    }
}

#[derive(Clone, Default)]
struct Fake(Arc<Mutex<Line>>);

impl Fake {
    fn new() -> Fake {
        let fake = Fake::default();
        fake.line().connected = true;
        fake.line().wait = Duration::from_millis(1);
        fake
    }

    fn line(&self) -> std::sync::MutexGuard<'_, Line> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Plays `says` to `query`, in turn, once each.
    fn script(&self, query: &'static str, says: &[Say]) {
        self.line().script.entry(query).or_default().extend(says.iter().cloned());
    }

    fn scpi(&self) -> Scpi {
        let mut scpi = Scpi::new(Box::new(self.clone()));
        scpi.read_timeout = Duration::from_millis(20);
        scpi.opc_silence = Duration::from_millis(100);
        scpi
    }

    /// What was written since the last look, without the `*OPC?`s, as lines.
    fn commands(&self) -> Vec<String> {
        self.all_writes().into_iter().filter(|w| w != "*OPC?").collect()
    }

    fn all_writes(&self) -> Vec<String> {
        self.line()
            .written
            .iter()
            .map(|(_, w)| String::from_utf8_lossy(w).trim_end().to_owned())
            .collect()
    }

    fn count(&self, command: &str) -> usize {
        self.all_writes().iter().filter(|w| *w == command).count()
    }

    /// Until `command` has gone `n` times, within a bound.
    fn wait_for(&self, command: &str, n: usize) {
        let deadline = Instant::now() + BOUND;
        while self.count(command) < n {
            assert!(Instant::now() < deadline, "{command} went {} times", self.count(command));
            std::thread::sleep(Duration::from_millis(1));
        }
    }
}

impl ScpiTransport for Fake {
    fn resource(&self) -> &'static str {
        "fake"
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        let mut line = self.line();
        if line.gone {
            return Err(Error::Disconnected);
        }
        if !line.connected {
            line.writes_while_closed += 1;
        }
        line.written.push((Instant::now(), data.to_vec()));
        // What is owed goes ahead of what this asks: a device keeps its order.
        if line.paced_until.is_none() {
            line.tick();
        }
        let text = String::from_utf8_lossy(data).trim_end().to_owned();
        if !text.contains('?') {
            return Ok(());
        }
        let said = line.script.get_mut(text.as_str()).and_then(VecDeque::pop_front);
        let mut bytes = match said {
            None => line.answer_of(&text),
            Some(Say::Text(t)) => format!("{t}\n").into_bytes(),
            Some(Say::Silent) => return Ok(()),
            Some(Say::Late(reads, t)) => {
                line.coming.push((reads, format!("{t}\n").into_bytes()));
                return Ok(());
            }
            Some(Say::ForTheDrain(t)) => {
                line.for_the_drain.extend_from_slice(t.as_bytes());
                return Ok(());
            }
            Some(Say::TextThenOwed(t, owed)) => {
                line.coming.push((0, format!("{owed}\n").into_bytes()));
                format!("{t}\n").into_bytes()
            }
            Some(Say::Gone) => {
                line.gone = true;
                return Ok(());
            }
        };
        let pace = line.pace;
        if pace.is_zero() {
            line.arrived.extend(bytes.drain(..));
        } else {
            line.paced_until = Some(Instant::now() + pace);
            line.coming.push((0, bytes));
        }
        Ok(())
    }

    fn read_begin(&mut self) -> Result<()> {
        self.line().complete = false;
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut line = self.line();
        if line.gone {
            return Err(Error::Disconnected);
        }
        if line.paced_until.is_none_or(|until| Instant::now() >= until) {
            line.paced_until = None;
            line.tick();
        }
        if line.arrived.is_empty() {
            let wait = line.wait;
            drop(line);
            std::thread::sleep(timeout.min(wait));
            return Ok(0);
        }
        // One line a read, so that two answers on the line stay two: merging them would
        // make a read of neither, which puts a shift right by accident.
        let line_end =
            line.arrived.iter().position(|b| *b == b'\n').map_or(line.arrived.len(), |at| at + 1);
        let n = line_end.min(buf.len());
        for slot in &mut buf[..n] {
            *slot = line.arrived.pop_front().expect("counted");
        }
        line.complete = true;
        Ok(n)
    }

    fn read_complete(&self) -> bool {
        self.line().complete
    }

    fn close(&mut self) -> Result<()> {
        self.line().connected = false;
        Ok(())
    }

    fn reopen(&mut self) -> Result<()> {
        self.line().connected = true;
        Ok(())
    }

    fn tags_answers(&self) -> bool {
        self.line().tags
    }

    fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
        let mut line = self.line();
        line.tick();
        let drained = std::mem::take(&mut line.for_the_drain);
        line.arrived.extend(drained);
        let n = line.arrived.len();
        line.arrived.clear();
        let wait = line.wait;
        drop(line);
        std::thread::sleep(timeout.min(wait));
        Ok(Some(n))
    }
}

/// A device of this model, found and open, with nothing written yet.
fn device(model: &str) -> (RigolDgDevice, Fake) {
    patient_device(model, Duration::from_millis(100))
}

/// [`device`], with `opc_silence` for the `*OPC?` patience instead of the fake's
/// 100 ms: for a test whose late answer must not be given up on.
fn patient_device(model: &str, opc_silence: Duration) -> (RigolDgDevice, Fake) {
    let fake = Fake::new();
    let idn: &'static str =
        Box::leak(format!("Rigol Technologies,{model},S1,1.0").into_boxed_str());
    fake.script("*IDN?", &[Say::Text(idn)]);
    let mut scpi = fake.scpi();
    scpi.opc_silence = opc_silence;
    let mut device = RigolDgDevice::from_scpi(scpi).unwrap();
    device.open().unwrap();
    fake.line().written = WriteLog::default();
    device.poll = Duration::from_millis(1);
    device.counter_delay = Duration::ZERO;
    (device, fake)
}

type Received = Receiver<(usize, Packet)>;

fn start(device: &mut RigolDgDevice) -> Received {
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// All packets up to the End, within [`BOUND`] and a cap: a stop or limit that doesn't
/// bite fails. With a count cap for a mutant that sends for ever.
fn packets(rx: &Received) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        let (_, packet) = rx.recv_timeout(left).expect("no End in time");
        let end = matches!(packet, Packet::End);
        out.push(packet);
        assert!(out.len() < 100_000, "a limit was ignored");
        if end {
            return out;
        }
    }
}

fn frequencies(packets: &[Packet]) -> Vec<f32> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) if a.mq == Mq::Frequency => Some(a.samples[0]),
            _ => None,
        })
        .collect()
}

fn failure(packets: &[Packet]) -> Option<String> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(why) => Some(why.clone()),
        _ => None,
    })
}

/// Runs `test` on a thread with a bound, so a stop ignored fails instead of hanging.
fn bounded(test: impl FnOnce() + Send + 'static) {
    let (tx, rx) = std::sync::mpsc::channel();
    let handle = std::thread::spawn(move || {
        test();
        let _ = tx.send(());
    });
    match rx.recv_timeout(BOUND * 2) {
        Ok(()) => handle.join().unwrap(),
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(handle.join().unwrap_err())
        }
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("the test hung"),
    }
}

#[test]
fn a_scan_says_who_and_goes_back_to_local() {
    let fake = Fake::new();
    fake.script("*IDN?", &[Say::Text("RIGOL TECHNOLOGIES,dg812,SN5,00.03")]);
    let device = RigolDgDevice::from_scpi(fake.scpi()).unwrap();
    assert_eq!(fake.all_writes(), ["*IDN?", "*OPC?", "SYST:KLOC:STATE OFF"]);
    assert!(!fake.line().connected, "a probe closes");
    let info = device.info();
    assert_eq!(info.vendor, "RIGOL TECHNOLOGIES");
    assert_eq!(info.model, "dg812");
    assert_eq!(info.serial, "SN5");
    assert_eq!(info.version, "00.03");
    assert_eq!(info.conn, None);
    let names: Vec<_> = device.channels().iter().map(|c| c.name.clone()).collect();
    assert_eq!(names, ["CH1", "CH2", "FREQ1", "PERIOD1", "DUTY1", "WIDTH1"]);
    assert_eq!(device.channel_groups()[1].name, "2");
    assert_eq!(device.channel_groups()[1].channels, [1]);
}

#[test]
fn another_instrument_is_none_and_is_closed() {
    for idn in ["Acme,DG812,1,2", "Rigol Technologies,DG8,1,2", "a,b"] {
        let idn: &'static str = Box::leak(idn.to_owned().into_boxed_str());
        let fake = Fake::new();
        fake.script("*IDN?", &[Say::Text(idn)]);
        assert!(probe(fake.scpi()).is_none(), "{idn}");
        assert_eq!(fake.all_writes(), ["*IDN?"], "{idn}");
        assert!(!fake.line().connected);
    }
    let fake = Fake::new();
    fake.script("*IDN?", &[Say::Silent]);
    assert!(probe(fake.scpi()).is_none());
}

#[test]
fn an_mso_has_no_local_and_no_counter() {
    let (device, fake) = device("MSO5074");
    assert_eq!(fake.all_writes(), Vec::<String>::new());
    let names: Vec<_> = device.channels().iter().map(|c| c.name.clone()).collect();
    assert_eq!(names, ["CH1", "CH2"]);
    assert_eq!(device.options(None).unwrap(), Vec::new());
    assert!(device.options(Some("1")).unwrap().iter().any(|o| o.key == Key::DutyCycle));
    assert!(device.options(Some("3")).is_err());
}

#[test]
fn a_close_goes_back_to_local_and_once() {
    let (mut device, fake) = device("DG1062Z");
    device.close().unwrap();
    assert_eq!(fake.all_writes(), ["*OPC?", "SYST:KLOC:STATE OFF"]);
    assert!(!fake.line().connected);
    device.close().unwrap();
    assert_eq!(fake.all_writes().len(), 2);
    assert_eq!(fake.line().writes_while_closed, 0);
    assert!(device.get(&Key::Amplitude, Some("1")).is_err(), "closed");
    assert!(device.set(&Key::Amplitude, &Value::F64(1.0), Some("1")).is_err());
    assert_eq!(fake.line().writes_while_closed, 0);
    device.open().unwrap();
    assert!(fake.line().connected);
    assert_eq!(device.get(&Key::Amplitude, Some("1")).unwrap(), Value::F64(5.0));
}

#[test]
fn gets_ask_each_time_with_an_opc_first() {
    let (device, fake) = device("DG1062Z");
    assert_eq!(device.get(&Key::Enabled, Some("1")).unwrap(), Value::Bool(false));
    assert_eq!(fake.all_writes(), ["*OPC?", "OUTP1:STATE?"]);
    fake.line().written = WriteLog::default();
    assert_eq!(device.get(&Key::PatternMode, Some("2")).unwrap(), Value::from("Sine"));
    assert_eq!(fake.all_writes(), ["*OPC?", "SOUR2:APPL?"]);
    fake.script("SOUR1:APPL?", &[Say::Text("\"SIN,2.5,1,2,3\"")]);
    assert_eq!(device.get(&Key::OutputFrequency, Some("1")).unwrap(), Value::F64(2.5));
    fake.script("SOUR1:APPL?", &[Say::Text("\"SIN,2.5,1,2,3\"")]);
    assert_eq!(device.get(&Key::Amplitude, Some("1")).unwrap(), Value::F64(1.0));
    fake.script("SOUR1:APPL?", &[Say::Text("\"SIN,2.5,1,2,3\"")]);
    assert_eq!(device.get(&Key::Offset, Some("1")).unwrap(), Value::F64(2.0));
    fake.script("SOUR1:APPL?", &[Say::Text("\"SIN,2.5,1,2,3\"")]);
    assert_eq!(device.get(&Key::Phase, Some("1")).unwrap(), Value::F64(3.0));
    assert!(device.get(&Key::Samplerate, Some("1")).is_err());
    assert!(device.get(&Key::Samplerate, None).is_err());
    assert!(device.get(&Key::Amplitude, Some("3")).is_err());
    assert_eq!(device.get(&Key::LimitSamples, None).unwrap(), Value::U64(0));
}

#[test]
fn a_value_the_instrument_did_not_give_is_an_error_and_says_so() {
    let (device, fake) = device("DG1062Z");
    fake.script("SOUR1:APPL?", &[Say::Text("\"SIN,abc,1\"")]);
    let e = device.get(&Key::OutputFrequency, Some("1")).unwrap_err().to_string();
    assert!(e.contains("The instrument gave no usable value for output_frequency."), "{e}");
    fake.script("SOUR1:APPL?", &[Say::Text("\"SIN,abc,1\"")]);
    assert_eq!(device.get(&Key::Amplitude, Some("1")).unwrap(), Value::F64(1.0));
    fake.script("SOUR1:APPL?", &[Say::Text("\"SIN,abc,1\"")]);
    assert!(device.get(&Key::Offset, Some("1")).is_err());
    fake.script("SOUR1:APPL?", &[Say::Silent]);
    assert!(matches!(device.get(&Key::Phase, Some("1")), Err(Error::Timeout)));
    fake.script("OUTP1:STATE?", &[Say::Text("maybe")]);
    let e = device.get(&Key::Enabled, Some("1")).unwrap_err().to_string();
    assert!(e.contains("not a boolean: 'maybe'"), "{e}");
}

#[test]
fn a_duty_cycle_is_asked_by_its_waveform() {
    let (device, fake) = device("DG1062Z");
    let e = device.get(&Key::DutyCycle, Some("1")).unwrap_err().to_string();
    assert!(e.contains("output_duty_cycle on a waveform of Sine"), "{e}");
    assert_eq!(fake.all_writes(), ["*OPC?", "SOUR1:APPL?"], "and nothing more is asked");
    fake.line().written = WriteLog::default();
    fake.script("SOUR1:APPL?", &[Say::Text("\"SQU,1,1,1,1\"")]);
    fake.script("SOUR1:FUNC:SQU:DCYC?", &[Say::Text("2.500000E+01")]);
    assert_eq!(device.get(&Key::DutyCycle, Some("1")).unwrap(), Value::F64(25.0));
    assert_eq!(fake.commands(), ["SOUR1:APPL?", "SOUR1:FUNC:SQU:DCYC?"]);
    fake.script("SOUR1:APPL?", &[Say::Text("\"PULSE,1,1,1,1\"")]);
    fake.script("SOUR1:FUNC:PULS:DCYC?", &[Say::Text("abc")]);
    let e = device.get(&Key::DutyCycle, Some("1")).unwrap_err().to_string();
    assert!(e.contains("not a number: 'abc'"), "{e}");
}

#[test]
fn an_mso_says_its_values_one_by_one_and_keeps_what_it_can() {
    let (device, fake) = device("MSO5074");
    fake.script("SOUR1:FUNC?", &[Say::Text("SINC")]);
    assert_eq!(device.get(&Key::PatternMode, Some("1")).unwrap(), Value::from("Sinc"));
    assert_eq!(
        fake.commands(),
        ["SOUR1:FUNC?", "SOUR1:FREQ?", "SOUR1:VOLT?", "SOUR1:VOLT:OFFS?", "SOUR1:PHAS?"]
    );
    assert_eq!(fake.count("*OPC?"), 5);
    fake.script("SOUR1:VOLT?", &[Say::Silent]);
    assert!(device.get(&Key::Amplitude, Some("1")).is_err());
    fake.script("SOUR1:VOLT?", &[Say::Text("abc")]);
    assert_eq!(device.get(&Key::OutputFrequency, Some("1")).unwrap(), Value::F64(2.0));
    assert_eq!(device.get(&Key::Enabled, Some("1")).unwrap(), Value::Bool(true));
    // Its square has no duty cycle, and it still asks *OPC? first, as C.
    fake.script("SOUR1:FUNC?", &[Say::Text("SQU")]);
    assert!(matches!(device.get(&Key::DutyCycle, Some("1")), Err(Error::NotSupported(_))));
}

/// Each key's bytes, after the state and an `*OPC?`.
#[test]
fn a_setting_sends_this() {
    let cases: [(Key, Value, &str); 5] = [
        (Key::Enabled, Value::Bool(true), "OUTP1:STATE ON"),
        (Key::Enabled, Value::Bool(false), "OUTP1:STATE OFF"),
        (Key::PatternMode, Value::from("square"), "SOUR1:FUNC SQU"),
        (Key::OutputFrequency, Value::F64(2000.5), "SOUR1:FREQ 2000.500000"),
        (Key::Phase, Value::F64(90.0), "SOUR1:PHAS 90.000000"),
    ];
    for (key, value, sent) in cases {
        let (mut device, fake) = device("DG1062Z");
        device.set(&key, &value, Some("1")).unwrap();
        assert_eq!(fake.all_writes(), ["*OPC?", "SOUR1:APPL?", "*OPC?", sent], "{sent}");
    }
    // What moves with the load asks the instrument's limits just before.
    for (key, value, query, sent) in [
        (Key::Amplitude, 1.5, "SOUR1:VOLT?", "SOUR1:VOLT 1.500000"),
        (Key::Offset, -0.25, "SOUR1:VOLT:OFFS?", "SOUR1:VOLT:OFFS -0.250000"),
    ] {
        let (mut device, fake) = device("DG1062Z");
        device.set(&key, &Value::F64(value), Some("1")).unwrap();
        let min = format!("{query} MIN");
        let max = format!("{query} MAX");
        assert_eq!(
            fake.all_writes(),
            ["*OPC?", "SOUR1:APPL?", "*OPC?", "*OPC?", &min, "*OPC?", &max, sent],
            "{sent}"
        );
    }
}

/// A value the instrument would clamp is refused, and no setting goes out.
#[test]
fn a_value_the_instrument_would_clamp_is_not_sent() {
    for (wave, key, value, word) in [
        (
            "SIN",
            Key::OutputFrequency,
            70e6,
            "output_frequency 70000000 Hz is outside 0.000001 to 60000000",
        ),
        ("SIN", Key::OutputFrequency, 0.0, "output_frequency 0 Hz is outside"),
        ("SIN", Key::Phase, 361.0, "phase 361 degrees is outside 0 to 360"),
        ("SIN", Key::Amplitude, 50.0, "amplitude 50 Vpp is outside 0.002 to 10"),
        ("SIN", Key::Amplitude, 0.001, "amplitude 0.001 Vpp is outside"),
        ("SIN", Key::Offset, 3.0, "offset 3 V is outside -2.5 to 2.5"),
        ("PULSE", Key::DutyCycle, 100.0, "output_duty_cycle 100 % is outside 0.001 to 99.999"),
        ("PULSE", Key::DutyCycle, 95.0, "output_duty_cycle 95 % is outside 10 to 90"),
        ("SQU", Key::DutyCycle, 5.0, "output_duty_cycle 5 % is outside 10 to 90"),
    ] {
        let (mut device, fake) = device("DG1062Z");
        let answer: &'static str = Box::leak(format!("\"{wave},1,1,1,1\"").into_boxed_str());
        fake.script("SOUR1:APPL?", &[Say::Text(answer)]);
        let e = device.set(&key, &Value::F64(value), Some("1")).unwrap_err();
        assert!(matches!(e, Error::Arg(_)), "{e:?}");
        assert!(e.to_string().contains(word), "{e}");
        assert!(e.to_string().contains("not sent"), "{e}");
        let writes = fake.all_writes();
        assert!(!writes.iter().any(|w| !w.contains('?')), "a setting went out: {writes:?}");
    }
}

/// Limits that don't come: the value goes, with a word that the instrument clamps.
#[test]
fn limits_that_do_not_come_let_the_value_go() {
    let (mut device, fake) = device("DG1062Z");
    fake.script("SOUR1:VOLT? MIN", &[Say::Silent]);
    device.set(&Key::Amplitude, &Value::F64(50.0), Some("1")).unwrap();
    assert_eq!(fake.count("SOUR1:VOLT 50.000000"), 1);
    let (mut device, fake) = self::device("DG1062Z");
    fake.script("SOUR1:VOLT:OFFS? MAX", &[Say::Text("abc")]);
    device.set(&Key::Offset, &Value::F64(3.0), Some("1")).unwrap();
    assert_eq!(fake.count("SOUR1:VOLT:OFFS 3.000000"), 1);
    // An MSO5000's way of asking isn't known: nothing is asked.
    let (mut device, fake) = self::device("MSO5074");
    device.set(&Key::Amplitude, &Value::F64(50.0), Some("1")).unwrap();
    assert!(!fake.all_writes().iter().any(|w| w.contains("MAX") || w.contains("MIN")));
    assert_eq!(fake.count("SOUR1:VOLT 50.000000"), 1);
}

/// A pattern keeps the amplitude: `FUNC`, not a bare `APPL`, which would take 5 Vpp.
#[test]
fn a_pattern_is_func_on_both_tables() {
    let (mut device, fake) = device("DG1062Z");
    device.set(&Key::PatternMode, &Value::from("DC"), Some("2")).unwrap();
    assert_eq!(fake.commands(), ["SOUR2:APPL?", "SOUR2:FUNC DC"]);
    assert!(!fake.all_writes().iter().any(|w| w.contains("APPL:")));
    let (mut device, fake) = self::device("MSO5074");
    device.set(&Key::PatternMode, &Value::from("Sinc"), Some("1")).unwrap();
    assert_eq!(fake.count("SOUR1:FUNC SINC"), 1);
}

#[test]
fn a_set_of_a_duty_cycle_goes_by_the_waveform() {
    let (mut device, fake) = device("DG1062Z");
    fake.script("SOUR2:APPL?", &[Say::Text("\"SQU,1,1,1,1\"")]);
    device.set(&Key::DutyCycle, &Value::F64(25.0), Some("2")).unwrap();
    assert_eq!(
        fake.commands(),
        [
            "SOUR2:APPL?",
            "SOUR2:FUNC:SQU:DCYC? MIN",
            "SOUR2:FUNC:SQU:DCYC? MAX",
            "SOUR2:FUNC:SQU:DCYC 25.000000"
        ]
    );
    fake.line().written = WriteLog::default();
    fake.script("SOUR2:APPL?", &[Say::Text("\"PULSE,1,1,1,1\"")]);
    device.set(&Key::DutyCycle, &Value::F64(75.0), Some("2")).unwrap();
    assert_eq!(
        fake.commands(),
        [
            "SOUR2:APPL?",
            "SOUR2:FUNC:PULS:DCYC? MIN",
            "SOUR2:FUNC:PULS:DCYC? MAX",
            "SOUR2:FUNC:PULS:DCYC 75.000000"
        ]
    );
    let (mut device, fake) = self::device("MSO5074");
    fake.script("SOUR1:FUNC?", &[Say::Text("PULS")]);
    device.set(&Key::DutyCycle, &Value::F64(75.0), Some("1")).unwrap();
    assert!(fake.all_writes().contains(&"SOUR1:PULS:DCYC 75.000000".to_owned()));
}

/// A waveform that lacks a key: nothing of the setting goes out, and it says why.
#[test]
fn what_a_waveform_lacks_is_refused() {
    for (wave, key, word) in [
        ("NOISE", Key::OutputFrequency, "output_frequency on a waveform of Noise"),
        ("NOISE", Key::Phase, "phase on a waveform of Noise"),
        ("DC", Key::Amplitude, "amplitude on a waveform of DC"),
        ("SIN", Key::DutyCycle, "output_duty_cycle on a waveform of Sine"),
    ] {
        let (mut device, fake) = device("DG1062Z");
        let answer: &'static str = Box::leak(format!("\"{wave},1,1,1,1\"").into_boxed_str());
        fake.script("SOUR1:APPL?", &[Say::Text(answer)]);
        let e = device.set(&key, &Value::F64(1.0), Some("1")).unwrap_err();
        assert!(e.to_string().contains(word), "{e}");
        assert!(matches!(e, Error::NotSupported(_)));
        assert_eq!(fake.all_writes(), ["*OPC?", "SOUR1:APPL?", "*OPC?"]);
    }
    // An MSO's square has no duty, by its table.
    let (mut device, fake) = device("MSO5074");
    fake.script("SOUR1:FUNC?", &[Say::Text("SQU")]);
    assert!(device.set(&Key::DutyCycle, &Value::F64(1.0), Some("1")).is_err());
    assert!(!fake.all_writes().iter().any(|w| w.contains("DCYC")));
}

#[test]
fn a_set_that_cannot_read_the_state_sends_nothing() {
    let (mut device, fake) = device("DG1062Z");
    fake.script("SOUR1:APPL?", &[Say::Text("\"FOO,1,1,1,1\"")]);
    assert!(device.set(&Key::Enabled, &Value::Bool(true), Some("1")).is_err());
    assert_eq!(fake.all_writes(), ["*OPC?", "SOUR1:APPL?"]);
    fake.script("SOUR1:APPL?", &[Say::Silent]);
    assert!(device.set(&Key::Enabled, &Value::Bool(true), Some("1")).is_err());
    assert!(!fake.all_writes().iter().any(|w| w.contains("STATE")));
}

#[test]
fn a_value_that_is_none_goes_nowhere() {
    let (mut device, fake) = device("DG1062Z");
    for value in [Value::F64(f64::NAN), Value::F64(f64::INFINITY), Value::F64(f64::NEG_INFINITY)] {
        assert!(device.set(&Key::Amplitude, &value, Some("1")).is_err());
    }
    assert!(device.set(&Key::Amplitude, &Value::from("x"), Some("1")).is_err());
    assert!(device.set(&Key::Enabled, &Value::F64(1.0), Some("1")).is_err());
    assert!(device.set(&Key::PatternMode, &Value::F64(1.0), Some("1")).is_err());
    assert!(device.set(&Key::Samplerate, &Value::F64(1.0), Some("1")).is_err());
    assert!(device.set(&Key::Samplerate, &Value::U64(1), None).is_err());
    assert!(device.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(device.set(&Key::Amplitude, &Value::F64(1.0), Some("9")).is_err());
    assert_eq!(fake.all_writes(), Vec::<String>::new());
}

#[test]
fn an_unknown_pattern_sends_only_the_look() {
    let (mut device, fake) = device("DG1062Z");
    let e = device.set(&Key::PatternMode, &Value::from("Triangle"), Some("1")).unwrap_err();
    assert!(e.to_string().contains("no pattern 'Triangle'"), "{e}");
    assert_eq!(fake.all_writes(), ["*OPC?", "SOUR1:APPL?", "*OPC?"]);
}

#[test]
fn a_failed_write_says_so() {
    let (mut device, fake) = device("DG1062Z");
    fake.script("*OPC?", &[Say::Text("1"), Say::Text("1")]);
    fake.script("SOUR1:APPL?", &[Say::Text("\"SIN,1,1,1,1\"")]);
    let _ = device.get(&Key::Amplitude, Some("1")).unwrap();
    fake.line().gone = true;
    assert!(matches!(
        device.set(&Key::Amplitude, &Value::F64(1.0), Some("1")),
        Err(Error::Disconnected)
    ));
}

#[test]
fn lists() {
    let (device, fake) = device("DG1032Z");
    let ValueList::Str(names) = device.list(&Key::PatternMode, Some("1")).unwrap() else {
        panic!("a list of names");
    };
    assert_eq!(names, ["Sine", "Square", "Ramp", "Pulse", "Arb", "Noise", "DC"]);
    assert_eq!(fake.all_writes(), Vec::<String>::new(), "the names need no asking");
    assert_eq!(
        device.list(&Key::Phase, Some("1")).unwrap(),
        ValueList::RangeStepF64 { min: 0.0, max: 360.0, step: 0.001 }
    );
    assert_eq!(
        device.list(&Key::OutputFrequency, Some("1")).unwrap(),
        ValueList::RangeStepF64 { min: 1e-6, max: 30e6, step: 1e-6 }
    );
    assert_eq!(fake.all_writes(), ["*OPC?", "SOUR1:APPL?"]);
    fake.script("SOUR1:APPL?", &[Say::Text("\"NOISE,DEF,1,1,DEF\"")]);
    assert_eq!(
        device.list(&Key::OutputFrequency, Some("1")).unwrap(),
        ValueList::RangeStepF64 { min: 30e6, max: 30e6, step: 0.0 }
    );
    fake.script("SOUR1:APPL?", &[Say::Silent]);
    assert!(device.list(&Key::OutputFrequency, Some("1")).is_err());
    assert!(device.list(&Key::Amplitude, Some("1")).is_err());
    assert!(device.list(&Key::PatternMode, None).is_err());
    assert!(device.list(&Key::PatternMode, Some("7")).is_err());
    let (device, _) = self::device("MSO5102");
    assert_eq!(
        device.list(&Key::Phase, Some("2")).unwrap(),
        ValueList::RangeStepF64 { min: 0.0, max: 360.0, step: 0.1 }
    );
}

#[test]
fn a_frame_is_as_c_sends_it() {
    let packets = frame(2, &[1000.0, 0.001, 50.0, 5e-4, 7.0]);
    assert!(matches!(packets[0], Packet::FrameBegin));
    assert!(matches!(packets[5], Packet::FrameEnd));
    let expected = [
        (2, 1000.0f32, Mq::Frequency, Unit::Hertz, 10),
        (3, 0.001, Mq::Time, Unit::Second, 10),
        (4, 50.0, Mq::DutyCycle, Unit::Percentage, 3),
        (5, 5e-4, Mq::PulseWidth, Unit::Second, 10),
    ];
    for (packet, (channel, value, mq, unit, digits)) in packets[1..5].iter().zip(expected) {
        let Packet::Analog(a) = packet else { panic!("{packet:?}") };
        assert_eq!(a.channels, [channel]);
        assert_eq!(a.samples, [value]);
        assert_eq!(a.mq, mq);
        assert_eq!(a.unit, unit);
        assert_eq!(a.digits, digits);
        assert_eq!(a.flags, MqFlags::empty());
    }
}

#[test]
fn the_constants_are_these() {
    assert_eq!(POLL, Duration::from_millis(100));
    assert_eq!(COUNTER_DELAY, Duration::from_secs(1));
    assert_eq!(MODELS.len(), 18);
    assert_eq!(CREDIT_CAP, Duration::from_secs(60));
}

#[test]
fn a_capture_turns_the_counter_on_and_off_around_its_readings() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.limits.samples.store(2, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), [1001.0, 1002.0], "in order");
        assert!(matches!(got[0], Packet::Header { .. }));
        assert_eq!(failure(&got), None);
        assert_eq!(
            fake.all_writes(),
            [
                "COUN:STAT?",
                "*OPC?",
                "COUN:STAT ON",
                "*OPC?",
                "COUN:MEAS?",
                "*OPC?",
                "COUN:MEAS?",
                "*OPC?",
                "COUN:STAT OFF",
            ]
        );
        device.close().unwrap();
        assert_eq!(fake.line().writes_while_closed, 0);
    });
}

#[test]
fn a_counter_already_running_is_left_running() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        fake.script("COUN:STAT?", &[Say::Text("run,1")]);
        device.limits.samples.store(1, Ordering::SeqCst);
        let rx = start(&mut device);
        assert_eq!(frequencies(&packets(&rx)), [1001.0]);
        assert_eq!(fake.commands(), ["COUN:STAT?", "COUN:MEAS?"]);
        // The next capture looks again.
        fake.line().written = WriteLog::default();
        let rx = start(&mut device);
        assert_eq!(frequencies(&packets(&rx)), [1002.0]);
        assert_eq!(fake.commands(), ["COUN:STAT?", "COUN:STAT ON", "COUN:MEAS?", "COUN:STAT OFF"]);
    });
}

#[test]
fn a_stopped_counter_goes_on_and_ends_off_as_in_c() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        fake.script("COUN:STAT?", &[Say::Text("STOP")]);
        device.limits.samples.store(1, Ordering::SeqCst);
        let rx = start(&mut device);
        packets(&rx);
        assert_eq!(fake.commands(), ["COUN:STAT?", "COUN:STAT ON", "COUN:MEAS?", "COUN:STAT OFF"]);
    });
}

/// Channel 2 isn't touched, and no question is asked of it.
#[test]
fn channel_2_stays_as_it_is_on_a_dg812() {
    bounded(|| {
        let (mut device, fake) = device("DG812");
        device.limits.samples.store(1, Ordering::SeqCst);
        let rx = start(&mut device);
        packets(&rx);
        assert_eq!(fake.commands(), ["COUN:STAT?", "COUN:STAT ON", "COUN:MEAS?", "COUN:STAT OFF"]);
    });
}

/// A DG800's and DG900's counter: a second after each way, by the code's asking.
#[test]
fn a_dg8_waits_after_each_change_of_its_counter() {
    bounded(|| {
        let (mut device, fake) = device("DG811");
        let delay = Duration::from_millis(150);
        device.counter_delay = delay;
        device.limits.samples.store(1, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), [1001.0]);
        let line = fake.line();
        let at = |what: &str| {
            line.written
                .iter()
                .find(|(_, w)| String::from_utf8_lossy(w).trim_end() == what)
                .map(|(t, _)| *t)
                .unwrap()
        };
        assert!(at("COUN:MEAS?").duration_since(at("COUN:STAT ON")) >= delay, "after on");
        drop(line);
        // A close after the capture's End: the off's wait is over by then.
        let off = {
            let line = fake.line();
            line.written
                .iter()
                .find(|(_, w)| String::from_utf8_lossy(w).trim_end() == "COUN:STAT OFF")
                .map(|(t, _)| *t)
                .unwrap()
        };
        assert!(off.elapsed() >= delay, "the End comes after the wait");
    });
    // And not on a DG1000Z.
    bounded(|| {
        let (mut device, _) = device("DG1062Z");
        device.counter_delay = Duration::from_secs(120);
        device.limits.samples.store(1, Ordering::SeqCst);
        let rx = start(&mut device);
        assert_eq!(frequencies(&packets(&rx)), [1001.0]);
    });
}

/// At a real line's pace a time limit shorter than the poll still has its reading,
/// where it could end with none: C checks a limit only after one.
#[test]
fn a_time_shorter_than_the_poll_has_its_reading_at_a_real_pace() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.poll = POLL;
        fake.line().pace = Duration::from_millis(5);
        device.limits.msec.store(20, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), [1001.0]);
        assert_eq!(failure(&got), None);
    });
}

/// A reading comes when asked, the poll apart: nothing faster than C's 100 ms.
#[test]
fn the_rounds_keep_their_poll() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.poll = Duration::from_millis(50);
        device.limits.samples.store(3, Ordering::SeqCst);
        let rx = start(&mut device);
        assert_eq!(frequencies(&packets(&rx)).len(), 3);
        let line = fake.line();
        let asked: Vec<Instant> = line
            .written
            .iter()
            .filter(|(_, w)| w.starts_with(b"COUN:MEAS?"))
            .map(|(t, _)| *t)
            .collect();
        let least = asked.windows(2).map(|w| w[1].duration_since(w[0])).min().unwrap();
        assert!(least >= Duration::from_millis(50), "{least:?}");
    });
}

#[test]
fn a_silent_start_is_refused_and_still_turns_the_counter_off() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        fake.script("COUN:STAT?", &[Say::Silent]);
        let rx = start(&mut device);
        let got = packets(&rx);
        let why = failure(&got).unwrap();
        assert!(why.contains("No answer about the counter's state"), "{why}");
        assert!(!got.iter().any(|p| matches!(p, Packet::Header { .. })), "no header");
        assert_eq!(fake.commands(), ["COUN:STAT?", "COUN:STAT OFF"]);
    });
}

fn fast_bounds() -> Bounds {
    Bounds {
        silent_first: Duration::from_millis(300),
        silent_after: Duration::from_millis(300),
        junk_first: Duration::from_millis(200),
        junk_after: Duration::from_millis(200),
    }
}

/// With a far bound: one silent reading isn't the end of it.
#[test]
fn one_silent_read_is_asked_again_with_a_word_in_no_failure() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        fake.script("COUN:MEAS?", &[Say::Silent, Say::Text("7.0,1,2,3,4")]);
        device.limits.samples.store(2, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), [7.0, 1001.0]);
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn a_counter_that_never_answers_fails_at_its_bound() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.bounds = fast_bounds();
        fake.script("COUN:MEAS?", &vec![Say::Silent; 100_000]);
        device.limits.samples.store(1, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), Vec::<f32>::new());
        assert_eq!(failure(&got).unwrap(), "device error: no answer from the counter in 0.3 s");
        assert_eq!(fake.count("COUN:STAT OFF"), 1, "it still turns it off");
    });
}

#[test]
fn a_counter_that_stops_answering_fails_at_the_later_bound() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.bounds = Bounds { silent_first: Duration::from_secs(600), ..fast_bounds() };
        fake.script("COUN:MEAS?", &[Say::Text("7.0,1,2,3,4")]);
        fake.script("COUN:MEAS?", &vec![Say::Silent; 100_000]);
        device.limits.samples.store(5, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), [7.0]);
        assert_eq!(failure(&got).unwrap(), "device error: no answer from the counter in 0.3 s");
    });
}

/// A bound that doesn't apply: with the first bound long, a mutant using it after a
/// reading would not end; with the later bound long, one using it before would not.
#[test]
fn a_counter_silent_from_the_start_goes_by_the_first_bound() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.bounds = Bounds { silent_after: Duration::from_secs(600), ..fast_bounds() };
        fake.script("COUN:MEAS?", &vec![Say::Silent; 100_000]);
        let rx = start(&mut device);
        assert!(failure(&packets(&rx)).is_some());
    });
}

#[test]
fn a_time_limit_with_nothing_captured_is_no_failure() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.bounds = Bounds { silent_first: Duration::from_secs(600), ..fast_bounds() };
        fake.script("COUN:MEAS?", &vec![Say::Silent; 100_000]);
        device.limits.msec.store(100, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), Vec::<f32>::new());
        assert_eq!(failure(&got), None);
    });
}

#[test]
fn answers_that_make_no_reading_fail_quicker_and_say_what_they_were() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.bounds = Bounds { silent_first: Duration::from_secs(600), ..fast_bounds() };
        fake.script("COUN:MEAS?", &vec![Say::Text("1,2,3"); 100_000]);
        device.limits.samples.store(1, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), Vec::<f32>::new());
        assert_eq!(
            failure(&got).unwrap(),
            "device error: no usable answer from the counter in 0.2 s: '1,2,3'"
        );
    });
}

/// Of each kind, as a stream in step and one that isn't reach other branches. No 0 of
/// `abc`, and no 1.5 of `1.5xyz`.
#[test]
fn what_is_no_number_makes_no_reading() {
    for answer in ["a,b,c,d,e", "1.5xyz,1,2,3,4", "1,2,3,4", "", "1", "inf,1,2,3,4", "0x10,1,2,3,4"]
    {
        let answer: &'static str = Box::leak(answer.to_owned().into_boxed_str());
        bounded(move || {
            let (mut device, fake) = device("DG1062Z");
            fake.script("COUN:MEAS?", &[Say::Text(answer)]);
            device.limits.samples.store(1, Ordering::SeqCst);
            let rx = start(&mut device);
            let got = packets(&rx);
            assert_eq!(frequencies(&got), [1001.0], "{answer}");
            assert_eq!(failure(&got), None, "{answer}");
        });
    }
}

/// An answer late past the read: no tag says what it answers. It is dropped, and every
/// later reading is its own.
#[test]
fn a_late_answer_shifts_nothing() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        fake.script("COUN:MEAS?", &[Say::ForTheDrain("7.0,1,2,3,4\n")]);
        device.limits.samples.store(3, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), [1001.0, 1002.0, 1003.0]);
        assert_eq!(failure(&got), None);
    });
}

/// A late answer past the drain, read as `*OPC?`'s: the wrong form for it, and the wrong
/// form for what comes next, which drains the line and takes the next in step.
#[test]
fn a_very_late_answer_makes_a_round_of_nothing_and_goes_back_in_step() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        fake.script("COUN:MEAS?", &[Say::Late(400, "7.0,1,2,3,4")]);
        device.limits.samples.store(4, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        let f = frequencies(&got);
        assert_eq!(f.len(), 4, "{f:?}");
        assert_eq!(failure(&got), None);
        // Each reading is one of the fake's, in order: the ask the very late answer
        // was for may be read, and none stands ahead of a later one.
        let mut sorted = f.clone();
        sorted.sort_by(f32::total_cmp);
        assert!(sorted == f || f[0] == 7.0, "{f:?}");
    });
}

/// An answer owed when one that makes no reading comes: the line is drained, or every
/// reading after it would be one behind for good.
#[test]
fn what_makes_no_reading_drains_what_is_owed() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.bounds = fast_bounds();
        fake.script("COUN:MEAS?", &[Say::TextThenOwed("1", "9.0,1,2,3,4")]);
        device.limits.samples.store(1, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), [1001.0]);
        assert_eq!(failure(&got), None);
        assert_eq!(fake.count("COUN:MEAS?"), 2, "one more question, and in step");
    });
}

/// `*OPC?` that takes seconds at a real pace: nothing is lost by it.
///
/// The late answer is ten reads away, and a read is a millisecond's sleep. The
/// device's patience is in time: with the fake's 100 ms, ten reads that take
/// longer (a macOS runner under nextest's load, oxrok#83) end the asking, and the
/// answer then lands ahead of the counter's reading, which is drained as junk
/// (1002 for 1001). Seconds of patience keep the test about the late answer.
#[test]
fn an_opc_that_is_slow_is_waited_for() {
    bounded(|| {
        let (mut device, fake) = patient_device("DG1062Z", Duration::from_secs(10));
        fake.script(
            "*OPC?",
            &[Say::Text("1"), Say::Text("0"), Say::Late(10, "1"), Say::Text("0"), Say::Text("1")],
        );
        device.limits.samples.store(1, Ordering::SeqCst);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), [1001.0]);
        assert_eq!(failure(&got), None);
    });
}

/// C goes on after an `*OPC?` nobody answers: so does a setting.
#[test]
fn a_silent_opc_does_not_hold_a_setting_back() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        fake.script("*OPC?", &[Say::Silent, Say::Silent]);
        device.set(&Key::Amplitude, &Value::F64(1.0), Some("1")).unwrap();
        assert_eq!(fake.count("SOUR1:VOLT 1.000000"), 1);
    });
}

/// One silent `*OPC?` may take its long bound. Then the next isn't asked to.
#[test]
fn the_next_opc_is_quick_once_one_was_silent_and_long_again_when_heard() {
    let fake = Fake::new();
    let mut scpi = fake.scpi();
    scpi.opc_silence = Duration::from_millis(100);
    let mut link = Link::new(scpi, &MODELS[0]);
    link.doubtful();
    assert_eq!(link.scpi.opc_silence, Duration::from_millis(100), "never longer");
    let mut scpi = fake.scpi();
    scpi.opc_silence = Duration::from_secs(600);
    let mut link = Link::new(scpi, &MODELS[0]);
    link.doubtful();
    assert_eq!(link.scpi.opc_silence, Duration::from_secs(2));
    link.heard();
    assert_eq!(link.scpi.opc_silence, Duration::from_secs(600));
    link.doubtful();
    link.opc();
    assert_eq!(link.scpi.opc_silence, Duration::from_secs(600), "an answer to it counts");
    // A failed *OPC? itself makes the next one quick. Short here for this one alone:
    // what it was built with is what it goes back from.
    let mut scpi = fake.scpi();
    scpi.opc_silence = Duration::from_secs(600);
    let mut link = Link::new(scpi, &MODELS[0]);
    link.scpi.opc_silence = Duration::from_millis(30);
    fake.script("*OPC?", &[Say::Silent]);
    link.opc();
    assert_eq!(link.scpi.opc_silence, Duration::from_secs(2));
}

#[test]
fn a_line_that_goes_ends_the_capture_at_once() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.bounds = Bounds {
            silent_first: Duration::from_secs(600),
            silent_after: Duration::from_secs(600),
            ..fast_bounds()
        };
        fake.script("COUN:MEAS?", &[Say::Text("7.0,1,2,3,4"), Say::Gone]);
        let rx = start(&mut device);
        let got = packets(&rx);
        assert_eq!(frequencies(&got), [7.0]);
        assert!(failure(&got).is_some());
        device.close().unwrap();
    });
}

#[test]
fn a_stop_ends_it_and_turns_the_counter_off() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        let rx = start(&mut device);
        fake.wait_for("COUN:MEAS?", 3);
        device.stop().unwrap();
        let got = packets(&rx);
        assert_eq!(failure(&got), None);
        assert_eq!(fake.count("COUN:STAT OFF"), 1);
        let writes = fake.all_writes();
        assert_eq!(writes.last().unwrap(), "COUN:STAT OFF");
    });
}

/// A stop while the counter is silent, in a round of `*OPC?`: not the 600 s of its bound.
#[test]
fn a_stop_during_a_silent_opc_lands() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        fake.script("*OPC?", &[Say::Text("1"), Say::Silent]);
        device.link().scpi.opc_silence = Duration::from_secs(600);
        let rx = start(&mut device);
        fake.wait_for("*OPC?", 2);
        std::thread::sleep(Duration::from_millis(5));
        device.stop().unwrap();
        let got = packets(&rx);
        assert_eq!(failure(&got), None);
        assert_eq!(frequencies(&got), Vec::<f32>::new());
    });
}

#[test]
fn a_close_joins_and_nothing_is_written_after_it() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        let rx = start(&mut device);
        fake.wait_for("COUN:MEAS?", 2);
        device.close().unwrap();
        assert!(device.worker.is_none());
        let writes = fake.all_writes().len();
        std::thread::sleep(Duration::from_millis(20));
        assert_eq!(fake.all_writes().len(), writes);
        assert_eq!(fake.line().writes_while_closed, 0);
        assert_eq!(fake.all_writes().last().unwrap(), "SYST:KLOC:STATE OFF");
        // The worker has let go of its sink, and the End arrived or was given up.
        drop(rx);
        assert_eq!(Arc::strong_count(&device.link), 1, "the worker let go of the link");
    });
}

/// A queue nobody reads, and no stop: the End waits for ever, and the start that follows
/// has to tell it to give up, or its join hangs.
#[test]
fn a_restart_after_a_stuck_end_does_not_hang() {
    bounded(|| {
        let (mut device, _fake) = device("DG1062Z");
        let (tx, rx_old) = sync_channel(1);
        device.limits.samples.store(1, Ordering::SeqCst);
        device.start(PacketSink::new(tx, 0)).unwrap();
        // All but the frame's end, which fills the queue with the End behind it.
        for _ in 0..6 {
            rx_old.recv_timeout(BOUND).unwrap();
        }
        let deadline = Instant::now() + BOUND;
        while device.capturing.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline, "the capture never ended");
            std::thread::sleep(Duration::from_millis(1));
        }
        let rx = start(&mut device);
        assert_eq!(frequencies(&packets(&rx)).len(), 1);
        drop(rx_old);
    });
}

/// A stop while a frame waits on a queue nobody reads: the frame gives up, the counter
/// goes off, and the capture ends.
#[test]
fn a_stop_lands_on_a_full_queue() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        let (tx, rx) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        fake.wait_for("COUN:MEAS?", 1);
        std::thread::sleep(Duration::from_millis(20));
        device.stop().unwrap();
        let deadline = Instant::now() + BOUND;
        while device.capturing.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline, "the frame never gave up");
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(fake.count("COUN:STAT OFF"), 1);
        drop(rx);
    });
}

/// A slow client is no silent counter: time on a full queue isn't silence. With the
/// bound shorter than the client's wait, and a silent round right after it.
#[test]
fn a_slow_client_is_no_silence() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        device.bounds = Bounds { silent_after: Duration::from_secs(1), ..fast_bounds() };
        fake.script("COUN:MEAS?", &[Say::Text("7.0,1,2,3,4"), Say::Silent]);
        device.limits.samples.store(2, Ordering::SeqCst);
        let (tx, rx) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        fake.wait_for("COUN:MEAS?", 1);
        std::thread::sleep(Duration::from_millis(1500));
        let got = packets(&rx);
        assert_eq!(frequencies(&got), [7.0, 1001.0]);
        assert_eq!(failure(&got), None);
    });
}

/// A limit set on a running capture counts at once, as in C.
#[test]
fn a_limit_set_in_a_capture_counts() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        let rx = start(&mut device);
        fake.wait_for("COUN:MEAS?", 2);
        device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let got = packets(&rx);
        assert_ne!(frequencies(&got), Vec::<f32>::new());
        assert_eq!(failure(&got), None);
    });
}

/// A setting on a running capture goes between two rounds, whole. A get too, and what
/// it reads is its own answer, not a reading.
#[test]
fn a_set_in_a_capture_goes_between_rounds() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        let rx = start(&mut device);
        fake.wait_for("COUN:MEAS?", 2);
        device.set(&Key::Amplitude, &Value::F64(1.25), Some("1")).unwrap();
        assert_eq!(device.get(&Key::Amplitude, Some("1")).unwrap(), Value::F64(5.0));
        fake.wait_for("COUN:MEAS?", 5);
        device.stop().unwrap();
        let got = packets(&rx);
        assert_eq!(failure(&got), None);
        let values = frequencies(&got);
        assert!(values.windows(2).all(|w| w[1] == w[0] + 1.0), "{values:?}");
        let writes = fake.all_writes();
        // The whole setting goes between two rounds: no counter's question inside it.
        let at = writes.iter().position(|w| w == "SOUR1:VOLT 1.250000").unwrap();
        assert_eq!(
            writes[at - 7..at],
            [
                "*OPC?",
                "SOUR1:APPL?",
                "*OPC?",
                "*OPC?",
                "SOUR1:VOLT? MIN",
                "*OPC?",
                "SOUR1:VOLT? MAX"
            ]
        );
    });
}

#[test]
fn a_second_start_is_refused_while_the_first_runs() {
    bounded(|| {
        let (mut device, fake) = device("DG1062Z");
        let rx = start(&mut device);
        fake.wait_for("COUN:MEAS?", 1);
        let (tx, _rx2) = sync_channel(1);
        assert!(matches!(device.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        device.stop().unwrap();
        packets(&rx);
    });
}

#[test]
fn a_start_before_an_open_is_refused() {
    let (mut device, _) = device("DG1062Z");
    device.close().unwrap();
    let (tx, _rx) = sync_channel(1);
    assert!(device.start(PacketSink::new(tx, 0)).is_err());
}

/// An MSO has nothing to ask: a header, no writes, and an end at a stop.
#[test]
fn an_mso_capture_only_waits_for_its_stop() {
    bounded(|| {
        let (mut device, fake) = device("MSO5074");
        let rx = start(&mut device);
        let (_, first) = rx.recv_timeout(BOUND).unwrap();
        assert!(matches!(first, Packet::Header { .. }));
        std::thread::sleep(Duration::from_millis(20));
        device.stop().unwrap();
        let got = packets(&rx);
        assert_eq!(failure(&got), None);
        assert_eq!(fake.all_writes(), Vec::<String>::new());
        assert_eq!(protocol::MODELS[12].model, "MSO5354");
    });
}
