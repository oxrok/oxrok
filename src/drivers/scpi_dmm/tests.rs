// Counterpart in sigrok:
//   libsigrok/src/hardware/scpi-dmm/api.c,
//   libsigrok/src/hardware/scpi-dmm/protocol.c,
//   Copyright (C) 2018 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Timo Kokkonen, Petteri Aimonen, Adrian Godwin,
//   Electro707, Frank Stettner, Mike Walters, Mike Williams, Sahaj Sarup,
//   Sylvain Pelissier
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver over a fake line: on its own clock, a read that finds nothing moves the
//! clock by its timeout, so a loaded machine slows the fake and the driver alike, and
//! answers wait in line behind each other as a device's do.

use super::*;
use crate::core::{Mq, MqFlags, Unit};
use crate::scpi::ScpiTransport;
use crate::serial::WriteLog;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a read waits here, on the line's clock in its ticks of 1 ms.
const READ: Duration = Duration::from_millis(5);

type Pieces = Vec<(u64, Vec<u8>)>;

/// What the meter says to a line, and its state.
trait Script: Send {
    fn answer(&mut self, line: &str) -> Pieces;
}

/// A meter that answers at once, from a model of its own: what a function query says,
/// and its readings, once each, then the last one again.
struct Bench {
    idn: &'static str,
    function: Vec<&'static str>,
    readings: VecDeque<(u64, String)>,
    last: String,
    opc: bool,
    /// `*OPC?` silent this many times.
    opc_silent: u32,
    /// Bad reads to give to the next `*OPC?`.
    taken: u32,
}

impl Bench {
    fn new(idn: &'static str, function: &'static str) -> Bench {
        Bench {
            idn,
            function: vec![function],
            readings: VecDeque::new(),
            last: "+1.50000000E+00".to_owned(),
            opc: true,
            opc_silent: 0,
            taken: 0,
        }
    }

    fn reads(mut self, readings: &[&str]) -> Bench {
        self.readings = readings.iter().map(|r| (0, (*r).to_owned())).collect();
        self
    }

    fn reads_paced(mut self, pace: u64, readings: &[&str]) -> Bench {
        self.readings = readings.iter().map(|r| (pace, (*r).to_owned())).collect();
        self
    }
}

const K34465: &str = "Keysight Technologies,34465A,S1,F1";
const K34401: &str = "HEWLETT-PACKARD,34401A,S1,F1";
const GW: &str = "GW,GDM8251A,S1,F1";
const OWON: &str = "OWON,XDM2041,S1,F1";
const VOLT: &str = "\"VOLT +1.00000E+01,+1.00000E-04\"";

impl Script for Bench {
    fn answer(&mut self, line: &str) -> Pieces {
        let text = |t: &str| vec![(0, format!("{t}\n").into_bytes())];
        match line {
            "*IDN?" => text(self.idn),
            "*OPC?" if !self.opc => Vec::new(),
            "*OPC?" if self.opc_silent > 0 => {
                self.opc_silent -= 1;
                Vec::new()
            }
            "*OPC?" => text("1"),
            "CONF?" | "CONF:STAT:FUNC?" | "FUNC?" => {
                let answer = if self.function.len() > 1 {
                    self.function.remove(0)
                } else {
                    self.function[0]
                };
                text(answer)
            }
            "SENS:DET:RATE?" => text("Fast"),
            "FETCH?" | "READ?" | "VAL1?" | "MEAS1?" => {
                self.taken += 1;
                let (pace, reading) = self.readings.pop_front().unwrap_or((0, self.last.clone()));
                vec![(pace, format!("{reading}\n").into_bytes())]
            }
            _ if line.ends_with(":RANGE:AUTO?") => text("0"),
            _ if line.ends_with(":RANGE?") => text("+1.00000E+01,+1.00000E-04"),
            _ => Vec::new(),
        }
    }
}

#[derive(Default)]
struct Line {
    writes: WriteLog,
    now: u64,
    coming: VecDeque<(u64, Vec<u8>, bool)>,
    /// What reads give instead of bytes, in turn.
    errors: VecDeque<Error>,
    /// All reads and writes fail from here on.
    gone: bool,
    /// So many reads come back at once with nothing, as a signal cuts them short.
    cut_short: u32,
    /// How often the driver drained the line.
    discards: u32,
    /// The driver asked to read again for a request already made.
    wait_out: bool,
    /// Device clears.
    clears: u32,
    closed: u32,
    reopened: u32,
}

/// A fake on a line, tags or none.
struct Fake {
    script: Box<dyn Script>,
    line: Arc<Mutex<Line>>,
    tags: bool,
    done: bool,
}

impl ScpiTransport for Fake {
    fn resource(&self) -> &'static str {
        "fake"
    }
    fn send(&mut self, data: &[u8]) -> Result<()> {
        let text = String::from_utf8_lossy(data).trim_end().to_owned();
        let mut line = self.line.lock().unwrap();
        if line.gone {
            return Err(Error::Disconnected);
        }
        line.writes.push((Instant::now(), data.to_vec()));
        // USBTMC: a new message drops what no read took, by its tag.
        if self.tags {
            line.coming.clear();
        }
        let mut at = line.now;
        // A compound line is cut as the instrument cuts it.
        for part in text.split(';') {
            let pieces = self.script.answer(part);
            let n = pieces.len();
            for (i, (delay, bytes)) in pieces.into_iter().enumerate() {
                at += delay;
                line.coming.push_back((at, bytes, i + 1 == n));
            }
        }
        Ok(())
    }
    fn read_begin(&mut self) -> Result<()> {
        self.done = false;
        Ok(())
    }
    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let wait = (timeout.as_millis() as u64).max(1);
        let mut line = self.line.lock().unwrap();
        if line.gone {
            return Err(Error::Disconnected);
        }
        if let Some(e) = line.errors.pop_front() {
            return Err(e);
        }
        if line.cut_short > 0 {
            line.cut_short -= 1;
            return Ok(0);
        }
        if let Some(due) = line.coming.front().map(|c| c.0)
            && due <= line.now + wait
        {
            line.now = line.now.max(due);
            let (_, bytes, last) = line.coming.pop_front().unwrap();
            buf[..bytes.len()].copy_from_slice(&bytes);
            self.done = last;
            return Ok(bytes.len());
        }
        line.now += wait;
        drop(line);
        std::thread::sleep(timeout);
        Ok(0)
    }
    fn read_complete(&self) -> bool {
        self.done
    }
    fn tags_answers(&self) -> bool {
        self.tags
    }
    fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
        if self.tags {
            return Ok(None);
        }
        self.line.lock().unwrap().discards += 1;
        let mut buf = [0u8; 4096];
        let n = self.read_data(&mut buf, timeout)?;
        Ok(Some(n))
    }
    fn close(&mut self) -> Result<()> {
        self.line.lock().unwrap().closed += 1;
        Ok(())
    }
    fn wait_out_requests(&mut self, on: bool) {
        self.line.lock().unwrap().wait_out = on;
    }
    fn clear(&mut self) -> Result<bool> {
        let mut line = self.line.lock().unwrap();
        line.clears += 1;
        line.coming.clear();
        Ok(self.tags)
    }
    fn reopen(&mut self) -> Result<()> {
        self.line.lock().unwrap().reopened += 1;
        Ok(())
    }
}

fn on_line(script: impl Script + 'static, tags: bool) -> (Scpi, Arc<Mutex<Line>>) {
    let line = Arc::new(Mutex::new(Line::default()));
    let fake = Fake { script: Box::new(script), line: Arc::clone(&line), tags, done: false };
    let mut scpi = Scpi::new(Box::new(fake));
    scpi.read_timeout = READ;
    (scpi, line)
}

/// A device through its scan, its timeout short again, and its writes taken away.
fn found(script: impl Script + 'static, tags: bool) -> (ScpiDmmDevice, Arc<Mutex<Line>>) {
    let (scpi, line) = on_line(script, tags);
    let mut device = ScpiDmmDevice::from_scpi(scpi).unwrap();
    device.closed.as_mut().unwrap().read_timeout = READ;
    device.open().unwrap();
    line.lock().unwrap().writes = WriteLog::default();
    (device, line)
}

fn written(line: &Arc<Mutex<Line>>) -> Vec<String> {
    let line = line.lock().unwrap();
    line.writes.iter().map(|(_, w)| String::from_utf8_lossy(w).trim_end().to_owned()).collect()
}

/// `f`, failing at a bound where it would hang: a mutant must fail.
fn within<R: Send + 'static>(f: impl FnOnce() -> R + Send + 'static) -> R {
    let (done, result) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _ = done.send(f());
    });
    result.recv_timeout(Duration::from_secs(120)).expect("it should have ended by now")
}

/// What a capture sends, up to its End, with a cap, and with no lock held.
#[derive(Debug, Default)]
struct Got {
    values: Vec<f32>,
    data: Vec<AnalogData>,
    failed: Option<String>,
    ended: bool,
}

fn drain(rx: &Receiver<(usize, Packet)>, bound: Duration) -> Got {
    let began = Instant::now();
    let mut got = Got::default();
    while began.elapsed() < bound && got.data.len() < 5000 {
        match rx.recv_timeout(Duration::from_millis(50)) {
            Ok((_, Packet::Analog(a))) => {
                got.values.extend_from_slice(&a.samples);
                got.data.push(a);
            }
            Ok((_, Packet::Failed(why))) => got.failed = Some(why),
            Ok((_, Packet::End)) => {
                got.ended = true;
                return got;
            }
            Ok(_) | Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => return got,
        }
    }
    got
}

fn capture(device: &mut ScpiDmmDevice) -> Got {
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    drain(&rx, Duration::from_secs(60))
}

fn samples(device: &mut ScpiDmmDevice, n: u64) {
    device.set(&Key::LimitSamples, &Value::U64(n), None).unwrap();
}

#[test]
fn the_scan_of_each_dialect() {
    for (idn, function, want) in [
        ("Agilent Technologies,34405A,S,F", VOLT, vec!["*IDN?", "*OPC?", "SYST:LOC"]),
        ("Agilent Technologies,34460A,S,F", VOLT, vec!["*IDN?", "*OPC?", "SYST:LOC"]),
        (K34465, VOLT, vec!["*IDN?", "*OPC?", "SYST:LOC"]),
        (GW, "01", vec!["*IDN?", "*OPC?", "SYST:LOC"]),
        ("GWInstek,GDM9060,S,F", VOLT, vec!["*IDN?", "*OPC?", "SYST:LOC"]),
        (K34401, VOLT, vec!["*IDN?"]),
        ("Agilent Technologies,34410A,S,F", VOLT, vec!["*IDN?"]),
        ("Siglent Technologies,SDM3055,S,F", VOLT, vec!["*IDN?"]),
        (OWON, "\"VOLT\"", vec!["*IDN?", "*OPC?", "*OPC?", "SYST:LOC"]),
    ] {
        let (scpi, line) = on_line(Bench::new(idn, function), false);
        let device = probe(scpi).unwrap();
        assert_eq!(written(&line), want, "{idn}");
        assert_eq!(line.lock().unwrap().closed, 1, "{idn}: a probe closes");
        assert_eq!(device.info.conn.as_deref(), Some("fake"));
    }
}

#[test]
fn another_maker_or_model_is_nothing_and_closes() {
    for idn in ["Acme,34405A,1,1", "Agilent,34405B,1,1", "HP,34405A,1,1", "x", "Agilent,34405A"] {
        let (scpi, line) = on_line(Bench::new(idn, VOLT), false);
        assert!(probe(scpi).is_none(), "{idn}");
        assert_eq!(written(&line), ["*IDN?"], "{idn}");
        assert_eq!(line.lock().unwrap().closed, 1, "{idn}");
    }
    // The short name is no alias, and the long one with another case is.
    let (scpi, _) = on_line(Bench::new("Agilent,34405A,1,2", VOLT), false);
    assert!(probe(scpi).is_some());
    let (scpi, _) = on_line(Bench::new("AGILENT TECHNOLOGIES,34405A,1,2", VOLT), false);
    assert_eq!(probe(scpi).unwrap().info.vendor, "AGILENT TECHNOLOGIES");
}

#[test]
fn an_owon_with_no_opc_is_never_asked_again() {
    let mut bench = Bench::new(OWON, "\"VOLT\"");
    bench.opc = false;
    let (scpi, line) = on_line(bench, false);
    let mut device = probe(scpi).unwrap();
    assert_eq!(written(&line), ["*IDN?", "*OPC?", "SYST:LOC"]);
    device.closed.as_mut().unwrap().read_timeout = READ;
    device.open().unwrap();
    line.lock().unwrap().writes = WriteLog::default();
    samples(&mut device, 1);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [1.5]);
    assert_eq!(written(&line), ["FUNC?", "FUNC?", "MEAS1?"], "the start's and the round's");
}

#[test]
fn an_owon_with_opc_is_asked_as_any() {
    let (mut device, line) = found(Bench::new(OWON, "\"VOLT AC\""), false);
    samples(&mut device, 1);
    let got = within(move || {
        let got = capture(&mut device);
        device.close_after_capture().unwrap();
        got
    });
    assert_eq!(got.data[0].flags, MqFlags::AC);
    assert_eq!(
        written(&line),
        [
            "*OPC?", "FUNC?", "*OPC?", "*OPC?", "FUNC?", "*OPC?", "*OPC?", "MEAS1?", "*OPC?",
            "SYST:LOC"
        ]
    );
}

#[test]
fn a_read_timeout_is_the_tables() {
    for (idn, want) in [(K34401, 1500), (GW, 2500), (K34465, 1000)] {
        let (scpi, _) = on_line(Bench::new(idn, VOLT), false);
        let device = probe(scpi).unwrap();
        // The 1000 is `Scpi::new`'s, which these tests set to [`READ`].
        let timeout = device.closed.as_ref().unwrap().read_timeout;
        let want = if want == 1000 { READ } else { Duration::from_millis(want) };
        assert_eq!(timeout, want, "{idn}");
    }
}

#[test]
fn a_capture_is_cs_rounds_on_the_agilent_reader() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    samples(&mut device, 2);
    let got = within(move || {
        let got = capture(&mut device);
        device.close_after_capture().unwrap();
        got
    });
    assert_eq!(got.values, [1.5, 1.5]);
    assert!(got.ended);
    assert_eq!(got.failed, None);
    let a = &got.data[0];
    assert_eq!((a.mq, a.flags, a.unit, a.digits), (Mq::Voltage, MqFlags::DC, Unit::Volt, 4));
    assert_eq!(
        written(&line),
        [
            "*OPC?", "CONF?", "*OPC?", "CONF?", "*OPC?", "INIT", "*OPC?", "*OPC?", "CONF?",
            "*OPC?", "FETCH?", "*OPC?", "*OPC?", "CONF?", "*OPC?", "FETCH?", "*OPC?", "ABORT",
            "*OPC?", "SYST:LOC",
        ]
    );
}

#[test]
fn a_capture_is_cs_rounds_on_the_gw_reader() {
    let (mut device, line) = found(Bench::new(GW, "15"), false);
    samples(&mut device, 1);
    let got = within(move || {
        let got = capture(&mut device);
        device.close_after_capture().unwrap();
        got
    });
    let a = &got.data[0];
    assert_eq!((a.mq, a.unit, a.digits), (Mq::Temperature, Unit::Fahrenheit, 4), "Fast is 4");
    assert_eq!(
        written(&line),
        [
            "*OPC?",
            "CONF:STAT:FUNC?",
            "*OPC?",
            "SENS:DET:RATE?",
            "*OPC?",
            "*CLS;SYST:REM",
            "*OPC?",
            "*OPC?",
            "CONF:STAT:FUNC?",
            "*OPC?",
            "*OPC?",
            "VAL1?",
            "*OPC?",
            "SYST:LOC",
        ],
        "no stop command in its table"
    );
}

#[test]
fn an_overload_by_a_gw_code_is_infinity_in_the_packet() {
    let (mut device, _line) = found(Bench::new(GW, "13").reads(&["1.2E+3"]), false);
    samples(&mut device, 1);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [f32::INFINITY]);
    let (mut device, _line) = found(Bench::new(K34465, VOLT).reads(&["-9.9E+37"]), false);
    samples(&mut device, 1);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [f32::NEG_INFINITY]);
}

#[test]
fn every_close_says_local() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    device.close().unwrap();
    assert_eq!(written(&line), ["*OPC?", "SYST:LOC"], "after a get or a set too");
    assert_eq!(line.lock().unwrap().closed, 2, "the probe's and this one");
    device.open().unwrap();
    assert_eq!(line.lock().unwrap().reopened, 2, "found's open and this one");
    device.close_after_capture().unwrap();
    assert_eq!(written(&line), ["*OPC?", "SYST:LOC", "*OPC?", "SYST:LOC"]);
    assert_eq!(line.lock().unwrap().closed, 3);
    // Closed, a close has nothing to say.
    device.close().unwrap();
    assert_eq!(written(&line).len(), 4);
    // An HP has no local command, and nothing goes for it.
    let (mut device, line) = found(Bench::new(K34401, VOLT), false);
    device.close().unwrap();
    assert_eq!(written(&line), Vec::<String>::new());
}

#[test]
fn gets_and_sets_are_on_the_wire_with_their_opc() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    assert_eq!(device.get(&Key::Conn, None).unwrap(), Value::Str("fake".to_owned()));
    assert_eq!(device.get(&Key::LimitSamples, None).unwrap(), Value::U64(0));
    assert_eq!(device.get(&Key::LimitMsec, None).unwrap(), Value::U64(0));
    assert_eq!(written(&line), Vec::<String>::new(), "no wire for those");
    assert_eq!(
        device.get(&Key::MeasuredQuantity, None).unwrap(),
        Value::Mq(Mq::Voltage, MqFlags::DC)
    );
    assert_eq!(written(&line), ["*OPC?", "CONF?"]);
    line.lock().unwrap().writes = WriteLog::default();
    assert_eq!(device.get(&Key::Range, None).unwrap(), Value::Str("10.000000".to_owned()));
    assert_eq!(
        written(&line),
        ["*OPC?", "CONF?", "*OPC?", "VOLT:DC:RANGE:AUTO?", "*OPC?", "VOLT:DC:RANGE?"]
    );
    line.lock().unwrap().writes = WriteLog::default();
    let quantity = Value::Mq(Mq::Voltage, MqFlags::DC | MqFlags::DIODE);
    device.set(&Key::MeasuredQuantity, &quantity, None).unwrap();
    assert_eq!(written(&line), ["*OPC?", "CONF:DIOD"]);
    line.lock().unwrap().writes = WriteLog::default();
    device.set(&Key::Range, &Value::Str("auTO".to_owned()), None).unwrap();
    device.set(&Key::Range, &Value::Str("1e1".to_owned()), None).unwrap();
    device.set(&Key::Range, &Value::Str("max".to_owned()), None).unwrap();
    assert_eq!(
        written(&line),
        [
            "*OPC?",
            "CONF?",
            "*OPC?",
            "CONF:VOLT:DC AUTO",
            "*OPC?",
            "CONF?",
            "*OPC?",
            "CONF:VOLT:DC 1e1",
            "*OPC?",
            "CONF?",
            "*OPC?",
            "CONF:VOLT:DC max",
        ],
        "a number or a word goes out as written, AUTO in capitals"
    );
}

#[test]
fn a_range_that_would_be_another_command_goes_nowhere() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    for bad in ["10;*RST", "10\n*RST", "10\r", "volts", ""] {
        let e = device.set(&Key::Range, &Value::Str(bad.to_owned()), None).unwrap_err();
        assert!(matches!(e, Error::Arg(_)), "{bad:?}: {e}");
    }
    let e = device.set(&Key::Range, &Value::Str("10;*RST".to_owned()), None).unwrap_err();
    assert_eq!(
        e.to_string(),
        "invalid argument: a range is a number or AUTO, MIN, MAX or DEF, not '10;*RST'"
    );
    assert_eq!(written(&line), Vec::<String>::new(), "before the wire");
}

#[test]
fn what_a_set_refuses_sends_nothing() {
    let (mut device, line) = found(Bench::new(K34465, "\"TEMP THER,5000,+1E+00,+1E-01\""), false);
    assert!(device.set(&Key::Range, &Value::Str(String::new()), None).is_err());
    let none = Value::Mq(Mq::Power, MqFlags::empty());
    assert!(matches!(device.set(&Key::MeasuredQuantity, &none, None), Err(Error::NotSupported(_))));
    assert!(device.set(&Key::MeasuredQuantity, &Value::U64(1), None).is_err());
    assert!(device.set(&Key::Conn, &Value::U64(1), None).is_err());
    assert_eq!(written(&line), Vec::<String>::new());
    assert!(matches!(
        device.set(&Key::Range, &Value::Str("1".to_owned()), None),
        Err(Error::NotSupported(_))
    ));
    assert_eq!(written(&line), ["*OPC?", "CONF?"], "no setting on a function with no range");
    assert!(device.get(&Key::Range, None).is_err());
    assert!(device.get(&Key::Conn, Some("x")).is_err());
    assert!(device.list(&Key::Range, None).is_err());
}

#[test]
fn a_model_with_no_range_has_no_key() {
    let (mut device, line) = found(Bench::new(K34401, VOLT), false);
    let keys: Vec<Key> = device.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(
        keys,
        [Key::Continuous, Key::Conn, Key::LimitSamples, Key::LimitMsec, Key::MeasuredQuantity]
    );
    assert!(device.get(&Key::Range, None).is_err());
    assert!(device.set(&Key::Range, &Value::Str("1".to_owned()), None).is_err());
    assert_eq!(written(&line), Vec::<String>::new(), "before any I/O");
    let (device, _) = found(Bench::new(K34465, VOLT), false);
    assert_eq!(device.options(None).unwrap().len(), 6);
    assert!(device.options(Some("x")).is_err());
    let ValueList::Mq(list) = device.list(&Key::MeasuredQuantity, None).unwrap() else {
        panic!("a list of quantities");
    };
    assert_eq!(list.len(), 11);
    assert_eq!(list[8], (Mq::Voltage, MqFlags::DC | MqFlags::DIODE));
}

#[test]
fn the_34405a_delays_a_function_by_its_flag() {
    // On the 200 ms of its table, by the time a set takes.
    let (scpi, line) = on_line(Bench::new("Agilent Technologies,34405A,S,F", VOLT), false);
    let mut device = probe(scpi).unwrap();
    device.closed.as_mut().unwrap().read_timeout = READ;
    device.open().unwrap();
    line.lock().unwrap().writes = WriteLog::default();
    let t0 = Instant::now();
    device.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Voltage, MqFlags::AC), None).unwrap();
    assert!(t0.elapsed() >= Duration::from_millis(200));
    assert_eq!(written(&line), ["*OPC?", "CONF:VOLT:AC"]);
    // No flag, no delay: the least of a few, as a loaded box only stretches one.
    let least = (0..5)
        .map(|_| {
            let t0 = Instant::now();
            device.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Voltage, MqFlags::DC), None).unwrap();
            t0.elapsed()
        })
        .min()
        .unwrap();
    assert!(least < Duration::from_millis(200), "{least:?}");
}

#[test]
fn a_time_limit_shorter_than_a_real_pace_still_has_its_reading() {
    // Half of a read's timeout, as a real meter's half a second to its second, and a
    // limit gone long before: C's limit is looked at after a round.
    let bench = Bench::new(K34465, VOLT).reads_paced(3, &["+2.5E+00"]);
    let (mut device, line) = found(bench, false);
    device.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
    let got = within(move || capture(&mut device));
    assert_eq!(got.values[0], 2.5);
    assert!(got.ended);
    assert_eq!(got.failed, None);
    assert!(written(&line).contains(&"FETCH?".to_owned()));
}

#[test]
fn a_read_a_signal_cuts_short_is_no_silence() {
    // With the time limit gone, a read taken for silence would end the capture there.
    let (mut device, line) = found(Bench::new(K34465, VOLT).reads_paced(1, &["+2.5E+00"]), false);
    device.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
    line.lock().unwrap().cut_short = 40;
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [2.5], "{:?}", got.failed);
}

#[test]
fn a_limit_ends_at_a_rounds_end_and_at_no_other_time() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    samples(&mut device, 3);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values.len(), 3);
    assert_eq!(written(&line).iter().filter(|w| *w == "FETCH?").count(), 3);
}

#[test]
fn a_reading_slower_than_a_read_is_waited_for_with_one_question_out() {
    // 3.2 reads late, each of them, and each a value of its own, so that a shift shows.
    let bench = Bench::new(K34465, VOLT).reads_paced(16, &["+1E+00", "+2E+00", "+3E+00"]);
    let (mut device, line) = found(bench, false);
    samples(&mut device, 3);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [1.0, 2.0, 3.0]);
    assert_eq!(got.failed, None);
    assert_eq!(written(&line).iter().filter(|w| *w == "FETCH?").count(), 3, "none again");
}

#[test]
fn a_reading_slower_than_a_read_is_read_for_again_where_answers_have_tags() {
    // On USBTMC a second request while one is out gets a real meter's old answer under
    // the wrong tag: one question, read for again.
    let bench = Bench::new(K34465, VOLT).reads_paced(12, &["+1E+00"]);
    let (mut device, line) = found(bench, true);
    samples(&mut device, 1);
    let got = within(move || capture(&mut device));
    assert_eq!(got.failed, None);
    assert_eq!(got.values, [1.0]);
    let asked = written(&line).iter().filter(|w| *w == "FETCH?").count();
    assert_eq!(asked, 1);
}

#[test]
fn an_open_asks_to_read_again_for_a_request_already_made() {
    let (scpi, line) = on_line(Bench::new(K34465, VOLT), true);
    let mut device = probe(scpi).unwrap();
    assert!(!line.lock().unwrap().wait_out, "the scan is C's");
    device.open().unwrap();
    assert!(line.lock().unwrap().wait_out);
}

#[test]
fn an_opc_slower_than_a_read_is_read_for_again_where_answers_have_tags() {
    /// The round's first `*OPC?` comes 2.4 reads late.
    struct SlowOpc(Bench, u32);
    impl Script for SlowOpc {
        fn answer(&mut self, line: &str) -> Pieces {
            if line == "*OPC?" {
                self.1 += 1;
                if self.1 == 5 {
                    return vec![(12, b"1\n".to_vec())];
                }
            }
            self.0.answer(line)
        }
    }
    let (mut device, line) = found(SlowOpc(Bench::new(K34465, VOLT), 0), true);
    samples(&mut device, 1);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [1.5]);
    let opcs = written(&line).iter().filter(|w| *w == "*OPC?").count();
    // The start's three, the round's three, the stop's one.
    assert_eq!(opcs, 7, "none asked again");
}

#[test]
fn a_question_given_up_on_usbtmc_is_cleared() {
    let (mut device, line) = found(mute("FETCH?", 0), true);
    device.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
    let got = within(move || capture(&mut device));
    assert!(got.ended);
    assert_eq!(line.lock().unwrap().clears, 1, "the FETCH? given up");
}

#[test]
fn an_unknown_function_drains_the_line_before_the_round_again() {
    let mut bench = Bench::new(K34465, VOLT);
    bench.function = vec![VOLT, VOLT, "\"PER 1,1\"", VOLT];
    let (mut device, line) = found(bench, false);
    samples(&mut device, 1);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [1.5]);
    assert!(line.lock().unwrap().discards > 0, "the next send waited for the line");
}

#[test]
fn a_reading_twice_on_a_line_with_no_tags() {
    /// The second reading comes again, after the `*OPC?` that follows was asked.
    struct Twice(Bench, u32);
    impl Script for Twice {
        fn answer(&mut self, line: &str) -> Pieces {
            let mut pieces = self.0.answer(line);
            if line == "FETCH?" {
                self.1 += 1;
                if self.1 == 2 {
                    pieces.push((1, pieces[0].1.clone()));
                }
            }
            pieces
        }
    }
    let bench = Bench::new(K34465, VOLT).reads(&["+1E+00", "+2E+00", "+3E+00"]);
    let (mut device, _line) = found(Twice(bench, 0), false);
    samples(&mut device, 3);
    let got = within(move || capture(&mut device));
    // The copy answers an *OPC?, and what follows is out of step until a function
    // query shows it: the round starts again, and no reading is another's.
    assert_eq!((got.values, got.failed), (vec![1.0, 2.0, 3.0], None));
}

#[test]
fn a_reading_resets_the_count_of_rounds_started_again() {
    // Three in a row twice, with a reading between: each burst is within the bound.
    let per = "\"PER 1,1\"";
    let mut bench = Bench::new(K34465, VOLT);
    bench.function = vec![VOLT, VOLT, per, per, per, VOLT, per, per, per, VOLT];
    let (mut device, _line) = found(bench, false);
    samples(&mut device, 2);
    let got = within(move || capture(&mut device));
    assert_eq!((got.values.len(), got.failed), (2, None));
}

#[test]
fn a_slow_reading_leaves_nothing_to_drain() {
    // Each reading takes more than a read: once it has come the line owes nothing, so no
    // round waits out a read's quiet before its next send.
    let bench = Bench::new(K34465, VOLT).reads_paced(8, &["+1E+00", "+2E+00", "+3E+00", "+4E+00"]);
    let (mut device, line) = found(bench, false);
    samples(&mut device, 4);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [1.0, 2.0, 3.0, 4.0]);
    assert_eq!(line.lock().unwrap().discards, 0);
}

/// A meter that never answers this.
struct Mute {
    bench: Bench,
    query: &'static str,
    /// After how many answered.
    after: u32,
}

impl Script for Mute {
    fn answer(&mut self, line: &str) -> Pieces {
        if line == self.query {
            if self.after == 0 {
                return Vec::new();
            }
            self.after -= 1;
        }
        self.bench.answer(line)
    }
}

fn mute(query: &'static str, after: u32) -> Mute {
    Mute { bench: Bench::new(K34465, VOLT), query, after }
}

fn bounds(device: &mut ScpiDmmDevice, no_reading: u64, give_up: u64) {
    device.silence = (Duration::from_millis(no_reading), Duration::from_millis(give_up));
}

#[test]
fn a_meter_silent_from_the_start_fails_at_its_bound() {
    let (mut device, line) = found(mute("FETCH?", 0), false);
    samples(&mut device, 1);
    bounds(&mut device, 30, 100_000);
    let got = within(move || {
        let got = capture(&mut device);
        device.close_after_capture().unwrap();
        got
    });
    let why = got.failed.unwrap();
    // How long it took goes with the machine: the pure rule pins the seconds.
    assert!(why.starts_with("device error: No reading in "), "{why}");
    assert_eq!(got.values, Vec::<f32>::new());
    let writes = written(&line);
    assert_eq!(writes.iter().filter(|w| *w == "FETCH?").count(), 1, "one question: no tags");
    assert_eq!(&writes[writes.len() - 4..], ["*OPC?", "ABORT", "*OPC?", "SYST:LOC"]);
}

#[test]
fn a_meter_silent_after_a_reading_goes_on_to_the_long_bound() {
    let (mut device, _line) = found(mute("FETCH?", 2), false);
    samples(&mut device, 3);
    // Far past the short bound before the long one fails it.
    bounds(&mut device, 10, 120);
    let t0 = Instant::now();
    let got = within(move || capture(&mut device));
    assert_eq!(got.values.len(), 2);
    let why = got.failed.unwrap();
    assert!(why.starts_with("device error: No reading for "), "{why}");
    assert!(t0.elapsed() >= Duration::from_millis(120));
}

#[test]
fn a_silence_that_ends_goes_on_with_the_right_readings() {
    // The third is 40 reads late, past the warning, and the fourth is on time.
    let bench = Bench::new(K34465, VOLT).reads_paced(0, &["+1E+00", "+2E+00"]);
    let mut bench = bench;
    bench.readings.push_back((200, "+3E+00".to_owned()));
    bench.readings.push_back((0, "+4E+00".to_owned()));
    let (mut device, line) = found(bench, false);
    samples(&mut device, 4);
    bounds(&mut device, 20, 100_000);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [1.0, 2.0, 3.0, 4.0]);
    assert_eq!(got.failed, None);
    assert_eq!(written(&line).iter().filter(|w| *w == "FETCH?").count(), 4);
}

#[test]
fn a_time_limit_ends_a_silent_capture_with_exit_0() {
    let (mut device, _line) = found(mute("FETCH?", 0), false);
    device.set(&Key::LimitMsec, &Value::U64(20), None).unwrap();
    // The short bound would fail it if a time limit did not come first.
    bounds(&mut device, 1, 1);
    let got = within(move || capture(&mut device));
    assert_eq!(got.failed, None);
    assert!(got.ended);
    assert_eq!(got.values, Vec::<f32>::new());
}

#[test]
fn a_silent_opc_under_a_time_limit_does_not_outlast_it_by_its_own_bound() {
    let (mut device, _line) = found(mute("*OPC?", 40), false);
    device.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
    let t0 = Instant::now();
    let got = within(move || capture(&mut device));
    assert_eq!(got.failed, None);
    assert!(got.ended);
    assert!(t0.elapsed() < OPC_SILENCE, "{:?}", t0.elapsed());
}

#[test]
fn neither_limit_warns_and_goes_on_until_it_is_stopped() {
    let (mut device, line) = found(mute("FETCH?", 0), false);
    bounds(&mut device, 1, 1);
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    // Well past both bounds, by what was asked: nothing has ended it.
    let got = drain(&rx, Duration::from_millis(100));
    assert!(!got.ended, "{got:?}");
    device.stop().unwrap();
    let got = drain(&rx, Duration::from_secs(30));
    assert!(got.ended);
    assert_eq!(got.failed, None);
    within(move || device.close().unwrap());
    let writes = written(&line);
    assert_eq!(&writes[writes.len() - 3..], ["ABORT", "*OPC?", "SYST:LOC"]);
}

#[test]
fn a_late_answer_after_a_stop_does_not_shift_what_follows() {
    // A reading a hundred reads late, then a stop: the next capture must get its own.
    let bench = Bench::new(K34465, VOLT).reads_paced(500, &["+7E+00"]);
    let (mut device, _line) = found(bench, false);
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let got = drain(&rx, Duration::from_millis(60));
    assert_eq!(got.values, Vec::<f32>::new(), "it isn't due");
    device.stop().unwrap();
    let got = drain(&rx, Duration::from_secs(30));
    assert!(got.ended);
    assert_eq!(got.failed, None);
    // Later than any drain, on a line with no tags, it lands on the next question: a
    // start refused, or anything but this value as a reading.
    within(move || {
        samples(&mut device, 1);
        let (tx, rx) = sync_channel(64);
        if device.start(PacketSink::new(tx, 0)).is_ok() {
            assert_ne!(drain(&rx, Duration::from_secs(60)).values.first(), Some(&7.0));
        }
    });
}

#[test]
fn what_makes_no_reading_fails_at_once_and_says_what() {
    for (reading, why) in [
        ("", "No usable reading: ''"),
        ("x", "No usable reading: 'x'"),
        ("nan", "No usable reading: 'nan'"),
        ("1e999", "No usable reading: '1e999'"),
        ("0x10", "No usable reading: '0x10'"),
    ] {
        let (mut device, line) = found(Bench::new(K34465, VOLT).reads(&[reading]), false);
        samples(&mut device, 2);
        let got = within(move || capture(&mut device));
        assert_eq!(got.values, Vec::<f32>::new(), "{reading}");
        assert_eq!(got.failed.as_deref(), Some(&*format!("device error: {why}")), "{reading}");
        let writes = written(&line);
        assert_eq!(writes.iter().filter(|w| *w == "FETCH?").count(), 1, "{reading}");
        // The real reading may be behind what made none: the stop waits for quiet.
        assert!(line.lock().unwrap().discards > 0, "{reading}");
    }
}

#[test]
fn what_makes_no_reading_under_a_time_limit() {
    // Nothing captured: a word and no failure.
    let (mut device, _) = found(Bench::new(K34465, VOLT).reads(&["x"]), false);
    device.set(&Key::LimitMsec, &Value::U64(100_000), None).unwrap();
    let got = within(move || capture(&mut device));
    assert_eq!(got.failed, None);
    assert!(got.ended);
    // A reading before: a failed capture.
    let (mut device, _) = found(Bench::new(K34465, VOLT).reads(&["1", "x"]), false);
    device.set(&Key::LimitMsec, &Value::U64(100_000), None).unwrap();
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [1.0]);
    assert_eq!(got.failed.as_deref(), Some("device error: No usable reading: 'x'"));
}

#[test]
fn a_function_it_does_not_know_starts_the_round_again_then_ends_a_capture() {
    // Once: the round starts again, and has its reading.
    let mut bench = Bench::new(K34465, VOLT);
    bench.function = vec![VOLT, VOLT, "\"PER 1,1\"", VOLT];
    let (mut device, line) = found(bench, false);
    samples(&mut device, 1);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [1.5], "{:?}", got.failed);
    let writes = written(&line);
    assert_eq!(writes.iter().filter(|w| *w == "CONF?").count(), 4);
    assert_eq!(writes.iter().filter(|w| *w == "FETCH?").count(), 1);

    // For good: RESYNCS rounds start again, and the next one fails.
    let mut bench = Bench::new(K34465, VOLT);
    bench.function = vec![VOLT, VOLT, "\"PER 1,1\""];
    let (mut device, line) = found(bench, false);
    samples(&mut device, 1);
    let got = within(move || capture(&mut device));
    assert_eq!(
        got.failed.as_deref(),
        Some("device error: Unknown measurement quantity: 'PER 1,1\"'")
    );
    let writes = written(&line);
    assert_eq!(writes.iter().filter(|w| *w == "CONF?").count(), 2 + 4, "the start's and 4 rounds'");
    assert!(!writes.contains(&"FETCH?".to_owned()));

    let (mut device, line) = found(Bench::new(K34465, "\"PER 1,1\""), false);
    samples(&mut device, 1);
    let (tx, _rx) = sync_channel(64);
    let e = device.start(PacketSink::new(tx, 0)).unwrap_err();
    assert!(matches!(e, Error::NotSupported(_)), "{e}");
    assert_eq!(
        written(&line),
        ["*OPC?", "CONF?", "*OPC?", "ABORT"],
        "a stop follows, as a session's"
    );

    // An empty answer is no quantity either.
    let (device, line) = found(Bench::new(K34465, ""), false);
    assert!(device.get(&Key::MeasuredQuantity, None).is_err());
    assert_eq!(written(&line), ["*OPC?", "CONF?"]);
}

#[test]
fn a_refused_start_closes_with_local() {
    let (mut device, line) = found(Bench::new(K34465, "\"PER 1,1\""), false);
    let (tx, _rx) = sync_channel(64);
    assert!(device.start(PacketSink::new(tx, 0)).is_err());
    device.close_after_capture().unwrap();
    assert_eq!(written(&line), ["*OPC?", "CONF?", "*OPC?", "ABORT", "*OPC?", "SYST:LOC"]);
}

#[test]
fn what_refuses_a_setting_says_why() {
    let (mut device, _) = found(Bench::new(K34465, "\"TEMP THER,5000,+1E+00,+1E-01\""), false);
    let text = |e: Error| e.to_string();
    let none = Value::Mq(Mq::Power, MqFlags::empty());
    assert_eq!(
        text(device.set(&Key::MeasuredQuantity, &none, None).unwrap_err()),
        "not supported: the meter has no such function"
    );
    assert_eq!(
        text(device.set(&Key::Range, &Value::Str("1".to_owned()), None).unwrap_err()),
        "not supported: TEMP has no range"
    );
}

#[test]
fn a_bad_precision_ends_a_capture_ahead_of_its_value() {
    let mut bench = Bench::new(K34465, VOLT);
    bench.function = vec![VOLT, VOLT, "\"VOLT 1,x\""];
    let (mut device, line) = found(bench, false);
    samples(&mut device, 1);
    let got = within(move || capture(&mut device));
    assert_eq!(got.failed.as_deref(), Some("device error: No usable precision in '\"VOLT 1,x\"'"));
    assert!(!written(&line).contains(&"FETCH?".to_owned()));
}

#[test]
fn a_protocol_error_in_opc_costs_a_try_as_a_real_34405a_has_them() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), true);
    samples(&mut device, 1);
    {
        let mut line = line.lock().unwrap();
        line.errors.push_back(Error::device("USBTMC invalid bulk in header."));
        line.errors.push_back(Error::device("USBTMC invalid bulk in header."));
    }
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [1.5], "{:?}", got.failed);
    let writes = written(&line);
    assert_eq!(&writes[..3], ["*OPC?", "*OPC?", "*OPC?"], "asked again, not given up");
}

#[test]
fn an_opc_that_says_zero_is_asked_a_hundred_times_then_it_goes_on() {
    struct Busy(Bench);
    impl Script for Busy {
        fn answer(&mut self, line: &str) -> Pieces {
            if line == "*OPC?" {
                return vec![(0, b"0\n".to_vec())];
            }
            self.0.answer(line)
        }
    }
    let (device, line) = found(Busy(Bench::new(K34401, VOLT)), false);
    within(move || device.get(&Key::MeasuredQuantity, None).unwrap());
    let writes = written(&line);
    assert_eq!(writes.iter().filter(|w| *w == "*OPC?").count(), 100, "C's count");
    assert_eq!(writes.last().map(String::as_str), Some("CONF?"));
}

#[test]
fn a_silent_opc_goes_on_to_the_query_at_its_bound() {
    let (mut device, line) = found(mute("*OPC?", 1), false);
    samples(&mut device, 1);
    let t0 = Instant::now();
    within(move || {
        // A stop's and a close's bound is one read, so these are quick.
        device.close_after_capture().unwrap();
    });
    assert!(t0.elapsed() < Duration::from_secs(10));
    assert_eq!(written(&line), ["*OPC?", "SYST:LOC"]);
}

#[test]
fn a_gone_meter_is_a_failed_capture_at_once() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    samples(&mut device, 5);
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    line.lock().unwrap().gone = true;
    let got = within(move || {
        let got = drain(&rx, Duration::from_secs(60));
        device.close_after_capture().unwrap();
        got
    });
    assert_eq!(got.failed.as_deref(), Some("device disconnected"));
    assert!(got.ended);
}

#[test]
fn a_gone_meter_refuses_a_start() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    line.lock().unwrap().gone = true;
    let (tx, _rx) = sync_channel(64);
    let e = within(move || device.start(PacketSink::new(tx, 0)).unwrap_err().to_string());
    assert!(e.contains("disconnected"), "{e}");
}

#[test]
fn a_slow_reader_is_no_silent_meter() {
    // Each reading takes more than a read, so the rule on silence is asked each round,
    // and each send waits well past the long bound, on a queue of one read slowly.
    let bench = Bench::new(K34465, VOLT).reads_paced(8, &["+1E+00", "+2E+00", "+3E+00"]);
    let (mut device, _line) = found(bench, false);
    samples(&mut device, 3);
    bounds(&mut device, 100_000, 1000);
    let (tx, rx) = sync_channel(1);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let mut values = Vec::new();
    let mut failed = None;
    let began = Instant::now();
    loop {
        // On the clock with short sleeps, as a sleep shim stretches a long one.
        let pause = Instant::now();
        while pause.elapsed() < Duration::from_millis(1500) {
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(began.elapsed() < Duration::from_secs(90));
        match rx.recv_timeout(Duration::from_secs(30)) {
            Ok((_, Packet::Analog(a))) => values.extend_from_slice(&a.samples),
            Ok((_, Packet::Failed(why))) => failed = Some(why),
            Ok((_, Packet::End)) => break,
            _ => {}
        }
    }
    assert_eq!((values, failed), (vec![1.0, 2.0, 3.0], None));
}

#[test]
fn the_34405a_delays_a_measurement_by_its_flag() {
    let start = |function| {
        let (scpi, _) = on_line(Bench::new("Agilent Technologies,34405A,S,F", function), false);
        let mut device = probe(scpi).unwrap();
        device.closed.as_mut().unwrap().read_timeout = READ;
        device.open().unwrap();
        let (tx, rx) = sync_channel(64);
        let t0 = Instant::now();
        device.start(PacketSink::new(tx, 0)).unwrap();
        let took = t0.elapsed();
        device.stop().unwrap();
        assert!(drain(&rx, Duration::from_secs(30)).ended);
        device.close().unwrap();
        took
    };
    assert!(start("\"FREQ +1.00000E+00,+1.00000E-05\"") >= Duration::from_millis(2500));
    let least = (0..3).map(|_| start(VOLT)).min().unwrap();
    assert!(least < Duration::from_millis(2500), "no flag, no delay: {least:?}");
}

#[test]
fn a_stop_gets_through_a_silent_read_and_a_close_joins() {
    let (mut device, line) = found(mute("FETCH?", 0), false);
    samples(&mut device, 1);
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    // Wait until it is in the read.
    let began = Instant::now();
    while !written(&line).contains(&"FETCH?".to_owned()) {
        assert!(began.elapsed() < Duration::from_secs(30));
        std::thread::sleep(Duration::from_millis(1));
    }
    device.stop().unwrap();
    within(move || device.close().unwrap());
    let writes = written(&line);
    let got = drain(&rx, Duration::from_secs(5));
    assert!(got.ended);
    assert_eq!(got.failed, None, "a stop is no failure");
    assert_eq!(&writes[writes.len() - 3..], ["ABORT", "*OPC?", "SYST:LOC"]);
    std::thread::sleep(Duration::from_millis(20));
    assert_eq!(written(&line), writes, "nothing after a close");
}

#[test]
fn a_start_after_a_capture_whose_end_is_stuck_does_not_hang() {
    let (mut device, _line) = found(Bench::new(K34465, VOLT), false);
    samples(&mut device, 1);
    // A queue of one nobody reads: its header fills it.
    let (tx, old) = sync_channel(1);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let (tx, rx) = sync_channel(64);
    let got = within(move || {
        device.stop().unwrap();
        while device.capturing.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(1));
        }
        device.start(PacketSink::new(tx, 0)).unwrap();
        let got = drain(&rx, Duration::from_secs(30));
        (got, old)
    });
    assert_eq!(got.0.values, [1.5]);
    assert!(got.0.ended);
    assert!(matches!(got.1.try_recv(), Ok((_, Packet::Header { .. }))), "the old sink");
    // The old worker was joined, not left to give up by itself, so its sink is gone.
    assert!(matches!(got.1.try_recv(), Err(std::sync::mpsc::TryRecvError::Disconnected)));
}

/// A reading that comes a byte at a time, each more than a read apart.
struct Trickle(Bench);

impl Script for Trickle {
    fn answer(&mut self, line: &str) -> Pieces {
        if line == "FETCH?" {
            return (0..2000).map(|_| (6, b"1".to_vec())).collect();
        }
        self.0.answer(line)
    }
}

#[test]
fn a_stop_gets_through_an_answer_still_coming() {
    let (mut device, line) = found(Trickle(Bench::new(K34465, VOLT)), false);
    samples(&mut device, 1);
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    // Some of it has been read.
    let began = Instant::now();
    loop {
        let piece = {
            let line = line.lock().unwrap();
            let asked = line.writes.iter().any(|(_, w)| w == b"FETCH?\n");
            asked && line.coming.len() < 1990
        };
        if piece {
            break;
        }
        assert!(began.elapsed() < Duration::from_secs(60));
        std::thread::sleep(Duration::from_millis(1));
    }
    device.stop().unwrap();
    let left = within(move || {
        device.close().unwrap();
        device
    });
    assert!(drain(&rx, Duration::from_secs(5)).ended);
    let still_coming = line.lock().unwrap().coming.len();
    assert!(still_coming > 500, "it read on to the end of the answer: {still_coming}");
    drop(left);
}

#[test]
fn a_stop_with_a_question_out_leaves_the_next_answer_its_own() {
    // The stop is seen ahead of any read: nothing else has said the line owes an answer.
    let (mut scpi, _line) = on_line(Bench::new(K34465, VOLT).reads_paced(3, &["+7E+00"]), false);
    let stop = AtomicBool::new(true);
    let limits = SharedLimits::default();
    let mut watch = Watch {
        stop: &stop,
        limits: &limits,
        started: Instant::now(),
        silence: Silence::new(Instant::now()),
    };
    assert!(matches!(Link::ask(&mut scpi, "FETCH?", &mut watch), Err(Halt::Stopped)));
    assert_eq!(scpi.get_string("CONF?").unwrap(), VOLT, "not the reading nobody read");
}

#[test]
fn a_stop_or_an_end_gives_a_dead_meter_one_read() {
    // It stops answering once the capture has its reading: the probe's *OPC?, the
    // start's three, and the round's three.
    let (mut device, _line) = found(mute("*OPC?", 7), false);
    samples(&mut device, 1);
    let t0 = Instant::now();
    let got = within(move || {
        let got = capture(&mut device);
        device.close_after_capture().unwrap();
        got
    });
    assert_eq!(got.values, [1.5]);
    assert!(t0.elapsed() < Duration::from_secs(25), "{:?}", t0.elapsed());
}

#[test]
fn a_close_joins() {
    // A queue of one, left full by the header: the worker waits on its End after a stop,
    // and holds what it was given until it has gone.
    let (mut device, _line) = found(Bench::new(K34465, VOLT), false);
    samples(&mut device, 1);
    let (tx, rx) = sync_channel(1);
    device.start(PacketSink::new(tx, 0)).unwrap();
    device.stop().unwrap();
    let held = within(move || {
        device.close().unwrap();
        Arc::strong_count(&device.link)
    });
    assert_eq!(held, 1, "the worker still has the link");
    drop(rx);
}

#[test]
fn a_full_queue_does_not_hold_a_close() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    samples(&mut device, 5);
    let (tx, rx) = sync_channel(1);
    device.start(PacketSink::new(tx, 0)).unwrap();
    // The reading has been read, so its send comes, on a queue the header has filled.
    let began = Instant::now();
    loop {
        let read = {
            let line = line.lock().unwrap();
            let asked = line.writes.iter().any(|(_, w)| w == b"FETCH?\n");
            asked && line.coming.is_empty()
        };
        if read {
            break;
        }
        assert!(began.elapsed() < Duration::from_secs(60));
        std::thread::sleep(Duration::from_millis(1));
    }
    device.stop().unwrap();
    within(move || device.close().unwrap());
    drop(rx);
}

#[test]
fn a_silent_opc_past_a_time_limit_asks_nothing_more() {
    // The probe's *OPC? and the start's three are answered, and a round's is not.
    let (mut device, line) = found(mute("*OPC?", 4), false);
    device.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
    let got = within(move || capture(&mut device));
    assert!(got.ended);
    assert_eq!(got.failed, None);
    let writes = written(&line);
    assert_eq!(writes.iter().filter(|w| *w == "CONF?").count(), 2, "the start's: {writes:?}");
}

#[test]
fn a_start_while_one_runs_is_busy() {
    let (mut device, _line) = found(mute("FETCH?", 0), false);
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let (tx2, _rx2) = sync_channel(64);
    assert!(matches!(device.start(PacketSink::new(tx2, 0)), Err(Error::Busy)));
    device.stop().unwrap();
    assert!(drain(&rx, Duration::from_secs(30)).ended);
    within(move || device.close().unwrap());
}

#[test]
fn a_start_on_a_closed_device_is_refused() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    device.close().unwrap();
    line.lock().unwrap().writes = WriteLog::default();
    let (tx, _rx) = sync_channel(64);
    assert!(device.start(PacketSink::new(tx, 0)).is_err());
    assert_eq!(written(&line), Vec::<String>::new());
    assert!(device.get(&Key::MeasuredQuantity, None).is_err());
}

#[test]
fn a_disabled_channel_asks_nothing_and_a_limit_ends_it() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    device.enable_channel(0, false).unwrap();
    device.set(&Key::LimitMsec, &Value::U64(30), None).unwrap();
    let got = within(move || capture(&mut device));
    assert!(got.ended);
    assert_eq!(got.values, Vec::<f32>::new());
    assert!(!written(&line).contains(&"FETCH?".to_owned()));
}

#[test]
fn a_limit_set_on_a_running_capture_counts() {
    let (mut device, _line) = found(Bench::new(K34465, VOLT), false);
    let (tx, rx) = sync_channel(4096);
    device.start(PacketSink::new(tx, 0)).unwrap();
    // A reading has come, with no limit.
    let began = Instant::now();
    loop {
        assert!(began.elapsed() < Duration::from_secs(30));
        if matches!(rx.recv_timeout(Duration::from_secs(1)), Ok((_, Packet::Analog(_)))) {
            break;
        }
    }
    device.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    assert!(drain(&rx, Duration::from_secs(30)).ended);
    within(move || device.close().unwrap());
}

#[test]
fn a_get_finds_its_turn_in_a_capture() {
    let (mut device, line) = found(Bench::new(K34465, VOLT), false);
    let (tx, rx) = sync_channel(4096);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let quantity = within({
        let shared = Arc::clone(&device.link);
        move || shared.lock().unwrap().get_mq().unwrap().1.setup
    });
    assert_eq!(quantity, "VOLT:DC");
    device.stop().unwrap();
    assert!(drain(&rx, Duration::from_secs(30)).ended);
    within(move || device.close().unwrap());
    assert_eq!(written(&line)[written(&line).len() - 3..], ["ABORT", "*OPC?", "SYST:LOC"]);
}

/// A meter that streams bytes with no end must not hold a read for ever or fill memory.
#[test]
fn an_answer_with_no_end_is_bounded() {
    struct Stream(Bench);
    impl Script for Stream {
        fn answer(&mut self, line: &str) -> Pieces {
            if line == "FETCH?" {
                return (0..2000).map(|_| (0, b"x".repeat(64))).collect();
            }
            self.0.answer(line)
        }
    }
    let (mut device, _line) = found(Stream(Bench::new(K34465, VOLT)), false);
    samples(&mut device, 1);
    let got = within(move || capture(&mut device));
    assert!(got.ended);
    let why = got.failed.unwrap();
    assert!(why.contains("more than 4096 bytes"), "{why}");
}

#[test]
fn a_reading_keeps_a_serial_lines_pieces() {
    // A reading cut in two, with a read's silence between, is one reading.
    struct Pieced(Bench);
    impl Script for Pieced {
        fn answer(&mut self, line: &str) -> Pieces {
            if line == "FETCH?" {
                return vec![(0, b"+2.5".to_vec()), (8, b"E+00\n".to_vec())];
            }
            self.0.answer(line)
        }
    }
    let (mut device, line) = found(Pieced(Bench::new(K34465, VOLT)), false);
    samples(&mut device, 2);
    let got = within(move || capture(&mut device));
    assert_eq!(got.values, [2.5, 2.5], "{:?}", got.failed);
    assert_eq!(written(&line).iter().filter(|w| *w == "FETCH?").count(), 2);
}
