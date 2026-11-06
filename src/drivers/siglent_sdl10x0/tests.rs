// Counterpart in sigrok:
//   libsigrok/src/hardware/siglent-sdl10x0/protocol.c,
//   Copyright (C) 2024 Timo Boettcher <timo@timoboettcher.name>,
//   with commits by Bert Vermeulen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake line: a load whose answers come at once, by a count of looks
//! at the line, or by a clock.

use super::*;
use crate::scpi::ScpiTransport;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test waits before it fails: a stop or a limit that doesn't bite must
/// fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);
/// More packets than any test here wants: a limit ignored on a fake with no delay
/// fails on this, not on memory.
const MOST_PACKETS: usize = 20_000;
/// A read's timeout on the fake, which goes by looks.
const READ: Duration = Duration::from_millis(20);
/// No bound a test isn't about.
const FAR: Bounds = Bounds {
    first: Duration::from_secs(3600),
    long: Duration::from_secs(36000),
    grace: Duration::from_secs(3600),
    credit: Duration::from_secs(2),
};

/// How an answer comes.
#[derive(Clone, Debug)]
enum Reply {
    /// By the next look at the line.
    Now(&'static str),
    /// After this many looks that find nothing: it holds under any stall.
    Late(u32, &'static str),
    /// Never.
    Silent,
}

enum Due {
    Looks(u32),
    At(Instant),
}

/// The line and what's behind it.
#[derive(Default)]
struct Wire {
    writes: crate::serial::WriteLog,
    coming: VecDeque<(Option<Due>, Vec<u8>)>,
    volts: VecDeque<Reply>,
    amps: VecDeque<Reply>,
    asked_volts: u32,
    asked_amps: u32,
    input: Option<&'static str>,
    number: Option<&'static str>,
    idn: Option<&'static str>,
    /// Answers take this long on the clock, one after the other.
    pace: Option<Duration>,
    last_due: Option<Instant>,
    /// All silent after this many measurements.
    measures: Option<u32>,
    fail_writes_after: Option<usize>,
    read_error: bool,
    closes: u32,
    reopens: u32,
    discards: u32,
    done: bool,
}

impl Wire {
    fn lines(&self) -> Vec<String> {
        self.writes.iter().map(|(_, w)| String::from_utf8_lossy(w).trim_end().to_owned()).collect()
    }

    fn enqueue(&mut self, text: &str) {
        let mut bytes = text.as_bytes().to_vec();
        bytes.push(b'\n');
        let due = self.pace.map(|pace| {
            let at = self.last_due.unwrap_or_else(Instant::now).max(Instant::now()) + pace;
            self.last_due = Some(at);
            Due::At(at)
        });
        self.coming.push_back((due, bytes));
    }

    fn reply(&mut self, reply: Option<&Reply>, otherwise: &str) {
        match reply {
            None => self.enqueue(otherwise),
            Some(Reply::Now(text)) => self.enqueue(text),
            Some(Reply::Late(looks, text)) => {
                let mut bytes = text.as_bytes().to_vec();
                bytes.push(b'\n');
                self.coming.push_back((Some(Due::Looks(*looks + 1)), bytes));
            }
            Some(Reply::Silent) => {}
        }
    }

    fn answer(&mut self, line: &str) {
        let measured = self.asked_volts + self.asked_amps;
        let spent = self.measures.is_some_and(|most| measured >= most);
        match line {
            "*IDN?" => {
                let idn = self.idn.unwrap_or("Siglent Technologies,SDL1030X-E,S1,1.2");
                self.enqueue(idn);
            }
            "MEAS:VOLT?" if spent => self.asked_volts += 1,
            "MEAS:CURR?" if spent => self.asked_amps += 1,
            "MEAS:VOLT?" => {
                let reply = self.volts.pop_front();
                self.reply(reply.as_ref(), &format!("{}.5", 10 + self.asked_volts));
                self.asked_volts += 1;
            }
            "MEAS:CURR?" => {
                let reply = self.amps.pop_front();
                self.reply(reply.as_ref(), &format!("0.{}", 1 + self.asked_amps));
                self.asked_amps += 1;
            }
            ":INPUT?" => {
                let input = self.input.unwrap_or("1");
                self.enqueue(input);
            }
            ":FUNC?" => self.enqueue("RESISTANCE"),
            ":POWer:PROTection:STATe?" | ":CURRent:PROTection:STATe?" => {
                self.enqueue("1");
            }
            q if q.ends_with('?') => {
                let number = self.number.unwrap_or("8.000");
                self.enqueue(number);
            }
            _ => {}
        }
    }

    /// What's due, as a look at the line takes it.
    fn take(&mut self) -> Option<Vec<u8>> {
        match self.coming.front()?.0 {
            Some(Due::Looks(n)) if n > 0 => None,
            Some(Due::At(at)) if at > Instant::now() => None,
            _ => self.coming.pop_front().map(|c| c.1),
        }
    }

    fn look(&mut self) {
        if let Some((Some(Due::Looks(n)), _)) = self.coming.front_mut() {
            *n = n.saturating_sub(1);
        }
    }
}

struct Fake(Arc<Mutex<Wire>>, &'static str);

impl ScpiTransport for Fake {
    fn resource(&self) -> &str {
        self.1
    }
    fn send(&mut self, data: &[u8]) -> Result<()> {
        let mut wire = self.0.lock().unwrap();
        if wire.fail_writes_after.is_some_and(|most| wire.writes.len() >= most) {
            return Err(Error::device("the write failed"));
        }
        wire.writes.push((Instant::now(), data.to_vec()));
        let line = String::from_utf8_lossy(data).trim_end().to_owned();
        wire.answer(&line);
        Ok(())
    }
    fn read_begin(&mut self) -> Result<()> {
        let mut wire = self.0.lock().unwrap();
        wire.done = false;
        wire.look();
        Ok(())
    }
    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut wire = self.0.lock().unwrap();
        if wire.read_error {
            return Err(Error::device("the line is gone"));
        }
        if let Some(bytes) = wire.take() {
            buf[..bytes.len()].copy_from_slice(&bytes);
            wire.done = true;
            return Ok(bytes.len());
        }
        drop(wire);
        std::thread::sleep(timeout.min(Duration::from_millis(1)));
        Ok(0)
    }
    fn read_complete(&self) -> bool {
        self.0.lock().unwrap().done
    }
    fn close(&mut self) -> Result<()> {
        let mut wire = self.0.lock().unwrap();
        wire.closes += 1;
        // What's on its way to a connection dies with it, if the line has those.
        if self.1.starts_with("tcp") {
            wire.coming.clear();
        }
        Ok(())
    }
    fn reopen(&mut self) -> Result<()> {
        self.0.lock().unwrap().reopens += 1;
        Ok(())
    }
    fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
        let mut wire = self.0.lock().unwrap();
        wire.discards += 1;
        // A wait for quiet outlasts what's only some looks late.
        if let Some((Some(Due::Looks(n)), _)) = wire.coming.front_mut() {
            *n = 0;
        }
        if let Some(bytes) = wire.take() {
            return Ok(Some(bytes.len()));
        }
        drop(wire);
        std::thread::sleep(timeout.min(Duration::from_millis(1)));
        Ok(Some(0))
    }
}

fn wire() -> Arc<Mutex<Wire>> {
    Arc::new(Mutex::new(Wire::default()))
}

fn scpi(w: &Arc<Mutex<Wire>>) -> Scpi {
    scpi_on(w, "fake")
}

fn scpi_on(w: &Arc<Mutex<Wire>>, resource: &'static str) -> Scpi {
    let mut scpi = Scpi::new(Box::new(Fake(Arc::clone(w), resource)));
    scpi.read_timeout = READ;
    scpi
}

/// A load found and opened, with the scan's writes gone.
fn device(w: &Arc<Mutex<Wire>>, bounds: Bounds) -> Sdl10x0Device {
    device_on(w, bounds, "fake")
}

fn device_on(w: &Arc<Mutex<Wire>>, bounds: Bounds, resource: &'static str) -> Sdl10x0Device {
    let mut d = Sdl10x0Device::from_scpi(scpi_on(w, resource)).unwrap();
    d.bounds = bounds;
    d.open().unwrap();
    w.lock().unwrap().writes = crate::serial::WriteLog::default();
    d
}

fn until_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        assert!(!left.is_zero(), "no End after {BOUND:?}");
        assert!(out.len() < MOST_PACKETS, "{MOST_PACKETS} packets: a limit was ignored");
        let (_, p) = rx.recv_timeout(left).expect("a packet in time");
        let done = matches!(p, Packet::End);
        out.push(p);
        if done {
            return out;
        }
    }
}

fn capture(d: &mut Sdl10x0Device) -> Vec<Packet> {
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    until_end(&rx)
}

/// `(volts, amps)` of each frame, checking how a frame is built.
fn frames(packets: &[Packet]) -> Vec<(f32, f32)> {
    let mut out = Vec::new();
    let mut at = packets.iter();
    while let Some(p) = at.next() {
        if !matches!(p, Packet::FrameBegin) {
            continue;
        }
        let (Some(Packet::Analog(v)), Some(Packet::Analog(i)), Some(Packet::FrameEnd)) =
            (at.next(), at.next(), at.next())
        else {
            panic!("a frame of two readings");
        };
        assert_eq!((v.mq, v.unit, v.flags), (Mq::Voltage, Unit::Volt, MqFlags::DC));
        assert_eq!((i.mq, i.unit, i.flags), (Mq::Current, Unit::Ampere, MqFlags::DC));
        assert_eq!((v.digits, i.digits), (7, 7));
        assert_eq!(
            (v.channels.as_slice(), i.channels.as_slice()),
            ([0].as_slice(), [0].as_slice())
        );
        out.push((v.samples[0], i.samples[0]));
    }
    out
}

fn failure(packets: &[Packet]) -> Option<&str> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(why) => Some(why.as_str()),
        _ => None,
    })
}

fn lines(w: &Arc<Mutex<Wire>>) -> Vec<String> {
    w.lock().unwrap().lines()
}

fn limited(d: &mut Sdl10x0Device, samples: u64) {
    d.set(&Key::LimitSamples, &Value::U64(samples), None).unwrap();
}

/// Waits for what must happen, with a bound.
fn until(what: &str, mut done: impl FnMut() -> bool) {
    let began = Instant::now();
    while !done() {
        assert!(began.elapsed() < BOUND, "{what}");
        std::thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn a_scan_asks_once_and_closes_and_an_open_opens_again() {
    let w = wire();
    let mut d = Sdl10x0Device::from_scpi(scpi(&w)).unwrap();
    assert_eq!(lines(&w), ["*IDN?"]);
    assert_eq!(w.lock().unwrap().closes, 1);
    assert_eq!(w.lock().unwrap().reopens, 0);
    let info = d.info();
    assert_eq!(info.vendor, "Siglent Technologies");
    assert_eq!(info.model, "SDL1030X-E");
    assert_eq!(info.serial, "S1");
    assert_eq!(info.version, "1.2");
    assert_eq!(info.conn, None);
    d.open().unwrap();
    d.open().unwrap();
    assert_eq!(w.lock().unwrap().reopens, 1, "an open device stays");
    assert_eq!(lines(&w), ["*IDN?"], "an open sends nothing");
    d.close().unwrap();
    assert_eq!(w.lock().unwrap().closes, 2);
    assert!(d.get(&Key::Voltage, Some("1")).is_err());
    d.open().unwrap();
    assert_eq!(w.lock().unwrap().reopens, 2);
    assert_eq!(d.get(&Key::OverCurrentProtectionThreshold, Some("1")).unwrap(), Value::F64(8.0));
}

#[test]
fn only_what_the_tables_name_is_a_load() {
    for idn in [
        "Acme,SDL1020X,1,2",
        "Siglent Technologies,SDL2000X,1,2",
        "siglent technologies,SDL1020X,1,2",
        "Siglent Technologies,sdl1020x,1,2",
        "short",
    ] {
        let w = wire();
        w.lock().unwrap().idn = Some(idn);
        assert!(probe(scpi(&w)).is_none(), "{idn}");
    }
    for model in protocol::MODELS {
        let idn: &'static str =
            Box::leak(format!("Siglent Technologies,{model},1,2").into_boxed_str());
        let w = wire();
        w.lock().unwrap().idn = Some(idn);
        assert_eq!(probe(scpi(&w)).unwrap().info().model, model);
    }
    let w = wire();
    w.lock().unwrap().idn = Some("Siglent Technologies,SDL1020X,1,2");
    assert_eq!(probe(scpi(&w)).unwrap().max_power, 200.0);
    let w = wire();
    assert_eq!(probe(scpi(&w)).unwrap().max_power, 300.0);
}

#[test]
fn each_setting_is_its_line() {
    let w = wire();
    let mut d = device(&w, FAR);
    let g = Some("1");
    d.set(&Key::Enabled, &Value::Bool(true), g).unwrap();
    d.set(&Key::Enabled, &Value::Bool(false), g).unwrap();
    d.set(&Key::Regulation, &Value::Str("CV".to_owned()), g).unwrap();
    d.set(&Key::Regulation, &Value::Str("RESISTANCE".to_owned()), g).unwrap();
    d.set(&Key::VoltageTarget, &Value::F64(5.125), g).unwrap();
    d.set(&Key::CurrentLimit, &Value::F64(1.23456), g).unwrap();
    d.set(&Key::PowerTarget, &Value::F64(250.0), g).unwrap();
    d.set(&Key::ResistanceTarget, &Value::F64(47.5), g).unwrap();
    d.set(&Key::OverPowerProtectionThreshold, &Value::F64(150.0), g).unwrap();
    d.set(&Key::OverCurrentProtectionThreshold, &Value::F64(-0.0), g).unwrap();
    assert_eq!(
        lines(&w),
        [
            ":INPUT ON",
            ":INPUT OFF",
            ":FUNC VOLTAGE",
            ":FUNC RESISTANCE",
            ":VOLT:LEV:IMM 5.125",
            ":CURR:LEV:IMM 1.235",
            ":POW:LEV:IMM 250.000",
            ":RES:LEV:IMM 47.500",
            ":POW:PROT:LEV 150.000",
            ":CURR:PROT:LEV -0.000",
        ]
    );
}

#[test]
fn what_is_refused_sends_nothing() {
    let w = wire();
    let mut d = device(&w, FAR);
    let g = Some("1");
    assert!(d.set(&Key::VoltageTarget, &Value::F64(f64::NAN), g).is_err());
    assert!(d.set(&Key::VoltageTarget, &Value::F64(f64::INFINITY), g).is_err());
    assert!(d.set(&Key::VoltageTarget, &Value::Str("1".to_owned()), g).is_err());
    assert!(d.set(&Key::Regulation, &Value::Str("Unknown".to_owned()), g).is_err());
    assert!(d.set(&Key::Regulation, &Value::F64(1.0), g).is_err());
    assert!(d.set(&Key::Enabled, &Value::F64(1.0), g).is_err());
    assert!(d.set(&Key::Voltage, &Value::F64(1.0), g).is_err(), "read only");
    assert!(d.set(&Key::VoltageTarget, &Value::F64(1.0), None).is_err(), "not without the group");
    assert!(d.set(&Key::LimitSamples, &Value::U64(1), g).is_err(), "not in the group");
    assert!(d.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::LimitMsec, &Value::U64(0), None).is_err());
    assert!(d.set(&Key::VoltageTarget, &Value::F64(1.0), Some("2")).is_err());
    assert_eq!(lines(&w), Vec::<String>::new());
}

#[test]
fn a_number_off_the_list_goes_out_with_a_warning() {
    assert_eq!(
        out_of_range(&Key::VoltageTarget, 200.0, 0.0, 150.0).as_deref(),
        Some(
            "voltage_target 200 is outside 0 to 150. It goes out, and a load that refuses it \
             keeps what it had, with no word."
        )
    );
    assert_eq!(out_of_range(&Key::VoltageTarget, 150.0, 0.0, 150.0), None);
    assert_eq!(out_of_range(&Key::VoltageTarget, 0.0, 0.0, 150.0), None);
    assert!(out_of_range(&Key::VoltageTarget, -0.5, 0.0, 150.0).is_some());
    let w = wire();
    let mut d = device(&w, FAR);
    d.set(&Key::VoltageTarget, &Value::F64(200.0), Some("1")).unwrap();
    assert_eq!(lines(&w), [":VOLT:LEV:IMM 200.000"]);
}

#[test]
fn a_failed_write_is_an_error() {
    let w = wire();
    let mut d = device(&w, FAR);
    w.lock().unwrap().fail_writes_after = Some(0);
    assert!(d.set(&Key::Enabled, &Value::Bool(true), Some("1")).is_err());
    assert!(d.get(&Key::Enabled, Some("1")).is_err());
}

#[test]
fn each_get_is_its_question() {
    let w = wire();
    let d = device(&w, FAR);
    let g = Some("1");
    assert_eq!(d.get(&Key::Enabled, g).unwrap(), Value::Bool(true));
    assert_eq!(d.get(&Key::Regulation, g).unwrap(), Value::Str("RESISTANCE".to_owned()));
    for key in [
        Key::VoltageTarget,
        Key::CurrentLimit,
        Key::PowerTarget,
        Key::ResistanceTarget,
        Key::Power,
        Key::Resistance,
        Key::OverPowerProtectionThreshold,
        Key::OverCurrentProtectionThreshold,
    ] {
        assert_eq!(d.get(&key, g).unwrap(), Value::F64(8.0), "{key:?}");
    }
    assert_eq!(d.get(&Key::OverPowerProtectionActive, g).unwrap(), Value::Bool(true));
    assert_eq!(d.get(&Key::OverCurrentProtectionActive, g).unwrap(), Value::Bool(true));
    assert_eq!(
        lines(&w),
        [
            ":INPUT?",
            ":FUNC?",
            ":VOLTage:LEVel?",
            ":CURRENT:LEVel?",
            ":POWer:LEVel?",
            ":RESistance:LEVel?",
            "MEAS:POWer?",
            "MEAS:RESistance?",
            ":POWer:PROTection:LEVel?",
            ":CURRent:PROTection:LEVel?",
            ":POWer:PROTection:STATe?",
            ":CURRent:PROTection:STATe?",
        ]
    );
}

/// A float as C keeps it, so a get is the float's double.
#[test]
fn a_reading_is_a_float_s_double() {
    let w = wire();
    w.lock().unwrap().number = Some("2.674");
    let d = device(&w, FAR);
    assert_eq!(d.get(&Key::CurrentLimit, Some("1")).unwrap(), Value::F64(f64::from(2.674_f32)));
}

#[test]
fn enabled_constants_ask_nothing() {
    let w = wire();
    let d = device(&w, FAR);
    assert_eq!(d.get(&Key::OverPowerProtectionEnabled, Some("1")).unwrap(), Value::Bool(true));
    assert_eq!(d.get(&Key::OverCurrentProtectionEnabled, Some("1")).unwrap(), Value::Bool(true));
    assert_eq!(lines(&w), Vec::<String>::new());
}

#[test]
fn a_get_of_what_is_no_number_fails() {
    for text in ["", "abc", "nan", "inf", "0x10", "1e40"] {
        let w = wire();
        w.lock().unwrap().number = Some(text);
        let d = device(&w, FAR);
        let e = d.get(&Key::CurrentLimit, Some("1")).unwrap_err().to_string();
        assert!(e.contains(&format!("'{text}'")), "{e}");
    }
    let w = wire();
    w.lock().unwrap().input = Some("maybe");
    let d = device(&w, FAR);
    assert!(d.get(&Key::Enabled, Some("1")).unwrap_err().to_string().contains("'maybe'"));
}

#[test]
fn a_key_elsewhere_asks_nothing() {
    let w = wire();
    let d = device(&w, FAR);
    assert!(d.get(&Key::Voltage, None).is_err());
    assert!(d.get(&Key::LimitSamples, Some("1")).is_err());
    assert!(d.get(&Key::Samplerate, None).is_err());
    assert!(d.get(&Key::Voltage, Some("2")).is_err());
    assert_eq!(lines(&w), Vec::<String>::new());
    assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(0));
    assert_eq!(d.get(&Key::LimitMsec, None).unwrap(), Value::U64(0));
}

#[test]
fn the_options_and_the_lists() {
    let w = wire();
    let d = device(&w, FAR);
    let keys: Vec<Key> = d.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(keys, [Key::Continuous, Key::LimitSamples, Key::LimitMsec]);
    assert_eq!(d.options(Some("1")).unwrap().len(), 16);
    assert_eq!(SiglentSdl10x0Driver.scan_options(), [Key::Conn]);
    assert_eq!(d.channels().len(), 1);
    assert_eq!(d.channels()[0].name, "1");
    let g = Some("1");
    assert_eq!(
        d.list(&Key::Regulation, g).unwrap(),
        ValueList::Str(["CURRENT", "VOLTAGE", "POWER", "RESISTANCE"].map(str::to_owned).to_vec())
    );
    let range = |key: &Key| d.list(key, g).unwrap();
    let r = |min, max, step| ValueList::RangeStepF64 { min, max, step };
    assert_eq!(range(&Key::VoltageTarget), r(0.0, 150.0, 0.001));
    assert_eq!(range(&Key::CurrentLimit), r(0.0, 30.0, 0.001));
    assert_eq!(range(&Key::PowerTarget), r(0.0, 300.0, 0.001));
    assert_eq!(range(&Key::ResistanceTarget), r(0.03, 10000.0, 0.01));
    assert_eq!(range(&Key::OverPowerProtectionThreshold), r(0.0, 300.0, 0.001));
    assert_eq!(range(&Key::OverCurrentProtectionThreshold), r(0.0, 30.0, 0.001));
    assert!(d.list(&Key::Regulation, None).is_err());
    assert!(d.list(&Key::Voltage, g).is_err());
}

/// The wire C has: the question after the limit stays out.
#[test]
fn a_capture_asks_in_turn_and_leaves_a_question_at_the_limit() {
    let w = wire();
    let mut d = device(&w, FAR);
    limited(&mut d, 2);
    let packets = capture(&mut d);
    assert!(matches!(packets[0], Packet::Header { .. }));
    assert_eq!(failure(&packets), None);
    assert_eq!(frames(&packets), [(10.5, 0.1), (11.5, 0.2)]);
    assert_eq!(lines(&w), ["MEAS:VOLT?", "MEAS:CURR?", "MEAS:VOLT?", "MEAS:CURR?", "MEAS:VOLT?"]);
    assert!(matches!(packets.last(), Some(Packet::End)));
}

/// With a time limit that passes while a voltage is on its way, C stops on the lone
/// voltage, with nothing. Real time, at a pace of its own.
#[test]
fn a_time_limit_shorter_than_a_round_still_gets_a_reading() {
    let w = wire();
    let mut d = device(&w, FAR);
    w.lock().unwrap().pace = Some(Duration::from_millis(30));
    d.set(&Key::LimitMsec, &Value::U64(10), None).unwrap();
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(frames(&packets), [(10.5, 0.1)]);
    assert_eq!(lines(&w), ["MEAS:VOLT?", "MEAS:CURR?", "MEAS:VOLT?"]);
}

/// A paced load through `--samples`: no reading lost, none out of turn.
#[test]
fn at_a_real_rate_a_sample_limit_gives_whole_frames() {
    let w = wire();
    let mut d = device(&w, FAR);
    w.lock().unwrap().pace = Some(Duration::from_millis(15));
    limited(&mut d, 1);
    let packets = capture(&mut d);
    assert_eq!(frames(&packets), [(10.5, 0.1)]);
    assert_eq!(failure(&packets), None);
}

#[test]
fn a_time_limit_ends_a_load_that_goes_on() {
    let w = wire();
    let mut d = device(&w, FAR);
    w.lock().unwrap().pace = Some(Duration::from_millis(1));
    d.set(&Key::LimitMsec, &Value::U64(30), None).unwrap();
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_ne!(frames(&packets).len(), 0);
}

/// The one thing that can't happen on a line with no tags: a late voltage read as the
/// current. It would, with a second asking, or a reading dropped at an expiry.
#[test]
fn a_late_answer_shifts_nothing_and_is_asked_for_once() {
    let w = wire();
    w.lock().unwrap().volts = VecDeque::from([Reply::Late(4, "20.5")]);
    let mut d = device(&w, FAR);
    limited(&mut d, 2);
    let packets = capture(&mut d);
    assert_eq!(frames(&packets), [(20.5, 0.1), (11.5, 0.2)]);
    assert_eq!(lines(&w), ["MEAS:VOLT?", "MEAS:CURR?", "MEAS:VOLT?", "MEAS:CURR?", "MEAS:VOLT?"]);
}

#[test]
fn a_late_current_shifts_nothing_either() {
    let w = wire();
    w.lock().unwrap().amps = VecDeque::from([Reply::Late(4, "0.9")]);
    let mut d = device(&w, FAR);
    limited(&mut d, 2);
    assert_eq!(frames(&capture(&mut d)), [(10.5, 0.9), (11.5, 0.2)]);
}

#[test]
fn silence_with_no_sample_fails_at_the_bound() {
    let w = wire();
    w.lock().unwrap().measures = Some(0);
    let bounds = Bounds { first: READ * 3, ..FAR };
    let mut d = device(&w, bounds);
    limited(&mut d, 2);
    let packets = capture(&mut d);
    // How many seconds it says goes with a stall's credit.
    assert!(failure(&packets).unwrap().starts_with("device error: No answer from the load in "));
    assert_eq!(frames(&packets), Vec::<(f32, f32)>::new());
    assert_eq!(lines(&w), ["MEAS:VOLT?"], "asked once");
}

/// It must not end on its bound with the bound far.
#[test]
fn silence_with_a_far_bound_does_not_end_until_a_stop() {
    let w = wire();
    w.lock().unwrap().measures = Some(0);
    let mut d = device(&w, FAR);
    limited(&mut d, 2);
    let (tx, rx) = sync_channel(64);
    d.start(PacketSink::new(tx, 0)).unwrap();
    assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
    assert!(rx.recv_timeout(READ * 8).is_err(), "still going");
    d.stop().unwrap();
    assert!(matches!(until_end(&rx).as_slice(), [Packet::End]));
}

/// A failing capture with a time limit and nothing captured is exit 0, with a warning.
#[test]
fn silence_under_a_time_limit_is_no_failure() {
    let w = wire();
    w.lock().unwrap().measures = Some(0);
    let bounds = Bounds { grace: READ * 2, ..FAR };
    let mut d = device(&w, bounds);
    d.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(frames(&packets), Vec::<(f32, f32)>::new());
    assert_eq!(lines(&w), ["MEAS:VOLT?"]);
}

/// A time limit far off is held by the first bound, the way a sample limit is.
#[test]
fn a_far_time_limit_lets_the_first_bound_end_it_with_no_failure() {
    let w = wire();
    w.lock().unwrap().measures = Some(0);
    let bounds = Bounds { first: READ * 2, ..FAR };
    let mut d = device(&w, bounds);
    d.set(&Key::LimitMsec, &Value::U64(3_600_000), None).unwrap();
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
}

/// After a sample the first bound only warns. Far past it, the answer's still good.
#[test]
fn a_silence_after_a_sample_is_waited_out() {
    let w = wire();
    w.lock().unwrap().volts =
        VecDeque::from([Reply::Now("10.5"), Reply::Late(10, "30.5"), Reply::Now("31.5")]);
    let bounds = Bounds { first: READ, ..FAR };
    let mut d = device(&w, bounds);
    limited(&mut d, 3);
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(frames(&packets), [(10.5, 0.1), (30.5, 0.2), (31.5, 0.3)]);
}

#[test]
fn a_silence_after_a_sample_fails_at_the_long_bound() {
    let w = wire();
    w.lock().unwrap().measures = Some(3);
    let bounds = Bounds { first: READ, long: READ * 4, ..FAR };
    let mut d = device(&w, bounds);
    limited(&mut d, 9);
    let packets = capture(&mut d);
    assert_eq!(frames(&packets), [(10.5, 0.1)]);
    assert!(failure(&packets).unwrap().starts_with("device error: No answer from the load for "));
}

#[test]
fn a_time_limit_with_samples_ends_in_a_silence_with_no_failure() {
    let w = wire();
    w.lock().unwrap().measures = Some(3);
    let mut d = device(&w, FAR);
    d.set(&Key::LimitMsec, &Value::U64(100), None).unwrap();
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(frames(&packets), [(10.5, 0.1)]);
}

#[test]
fn one_bad_answer_is_asked_again_from_the_voltage() {
    let w = wire();
    w.lock().unwrap().amps = VecDeque::from([Reply::Now("abc")]);
    let mut d = device(&w, FAR);
    limited(&mut d, 1);
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(frames(&packets), [(11.5, 0.2)], "the round asked over");
    assert_eq!(lines(&w), ["MEAS:VOLT?", "MEAS:CURR?", "MEAS:VOLT?", "MEAS:CURR?", "MEAS:VOLT?"]);
    assert!(w.lock().unwrap().discards > 0, "the line drained ahead of the next question");
}

#[test]
fn what_no_number_is_makes_no_reading() {
    for text in ["", "nan", "inf", "0x10", "1e40", "1V"] {
        let w = wire();
        w.lock().unwrap().volts = VecDeque::from([Reply::Now(text)]);
        let mut d = device(&w, FAR);
        limited(&mut d, 1);
        let packets = capture(&mut d);
        assert_eq!(frames(&packets), [(11.5, 0.1)], "{text:?}");
    }
}

#[test]
fn answers_that_never_make_a_number_fail_quickly() {
    let w = wire();
    w.lock().unwrap().volts = VecDeque::from(vec![Reply::Now("abc"); 9]);
    let mut d = device(&w, FAR);
    limited(&mut d, 2);
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), Some("device error: No usable answer from the load: 'abc'."));
    assert_eq!(lines(&w), ["MEAS:VOLT?", "MEAS:VOLT?", "MEAS:VOLT?"]);
}

/// Good voltages must not forgive a current that's never one.
#[test]
fn a_good_voltage_does_not_forgive_a_bad_current() {
    let w = wire();
    w.lock().unwrap().amps = VecDeque::from(vec![Reply::Now("abc"); 9]);
    let mut d = device(&w, FAR);
    limited(&mut d, 2);
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), Some("device error: No usable answer from the load: 'abc'."));
    assert_eq!(w.lock().unwrap().asked_amps, 3);
}

#[test]
fn junk_with_no_sample_under_a_time_limit_is_no_failure() {
    let w = wire();
    w.lock().unwrap().volts = VecDeque::from(vec![Reply::Now("abc"); 9]);
    let mut d = device(&w, FAR);
    d.set(&Key::LimitMsec, &Value::U64(3_600_000), None).unwrap();
    assert_eq!(failure(&capture(&mut d)), None);
}

#[test]
fn a_lost_line_fails_the_capture() {
    let w = wire();
    let mut d = device(&w, FAR);
    w.lock().unwrap().read_error = true;
    limited(&mut d, 2);
    assert_eq!(failure(&capture(&mut d)), Some("device error: the line is gone"));
}

#[test]
fn a_failed_write_fails_the_capture_at_either_question() {
    for most in [0, 1] {
        let w = wire();
        let mut d = device(&w, FAR);
        w.lock().unwrap().fail_writes_after = Some(most);
        limited(&mut d, 2);
        let packets = capture(&mut d);
        assert_eq!(failure(&packets), Some("device error: the write failed"), "{most}");
        assert_eq!(frames(&packets), Vec::<(f32, f32)>::new());
    }
}

/// The question at the limit is no question a failure can touch.
#[test]
fn a_failed_write_at_the_limit_fails_nothing() {
    let w = wire();
    let mut d = device(&w, FAR);
    w.lock().unwrap().fail_writes_after = Some(2);
    limited(&mut d, 1);
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(frames(&packets), [(10.5, 0.1)]);
}

/// C reads the question left out at the limit as the next capture's voltage, and each
/// voltage after it as a current, for good.
#[test]
fn a_second_capture_does_not_read_the_first_one_s_question() {
    let w = wire();
    let mut d = device(&w, FAR);
    limited(&mut d, 1);
    assert_eq!(frames(&capture(&mut d)), [(10.5, 0.1)]);
    w.lock().unwrap().discards = 0;
    let second = capture(&mut d);
    assert_eq!(frames(&second), [(12.5, 0.2)], "the voltage asked anew");
    assert!(w.lock().unwrap().discards > 0);
}

#[test]
fn a_get_after_a_capture_does_not_read_its_question() {
    let w = wire();
    let mut d = device(&w, FAR);
    limited(&mut d, 1);
    capture(&mut d);
    assert_eq!(d.get(&Key::Enabled, Some("1")).unwrap(), Value::Bool(true));
}

#[test]
fn a_stop_ends_a_capture_that_answers() {
    let w = wire();
    let mut d = device(&w, FAR);
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let w2 = Arc::clone(&w);
    until("some questions", move || w2.lock().unwrap().writes.len() > 6);
    d.stop().unwrap();
    let mut seen = 0;
    let deadline = Instant::now() + BOUND;
    loop {
        assert!(Instant::now() < deadline, "no End");
        seen += 1;
        assert!(seen < MOST_PACKETS, "a stop was ignored");
        if matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::End) {
            break;
        }
    }
    d.close().unwrap();
    let after = w.lock().unwrap().writes.len();
    std::thread::sleep(READ);
    assert_eq!(w.lock().unwrap().writes.len(), after, "no write after a close");
}

/// A question unanswered at a stop is for no one. On TCP a new connection can't hear
/// the answer, however late.
#[test]
fn a_stop_with_a_question_out_connects_again_on_tcp() {
    let w = wire();
    let mut d = device_on(&w, FAR, "tcp-raw/fake/1");
    w.lock().unwrap().volts = VecDeque::from([Reply::Late(30, "50.5")]);
    let (tx, rx) = sync_channel(64);
    d.start(PacketSink::new(tx, 0)).unwrap();
    assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
    let shared = d.shared.clone();
    until("a look that found nothing", move || shared.client().question_out);
    d.stop().unwrap();
    assert!(matches!(until_end(&rx).as_slice(), [Packet::End]));
    assert!(!d.shared.client().question_out, "over");
    let closes = w.lock().unwrap().closes;
    let reopens = w.lock().unwrap().reopens;
    limited(&mut d, 1);
    let next = capture(&mut d);
    assert_eq!(frames(&next), [(11.5, 0.1)]);
    assert_eq!(w.lock().unwrap().closes, closes + 1);
    assert_eq!(w.lock().unwrap().reopens, reopens + 1);
    assert_eq!(w.lock().unwrap().discards, 0, "no wait for quiet");
}

/// Off TCP it waits for quiet, which an answer within that gets dropped by.
#[test]
fn a_stop_with_a_question_out_waits_for_quiet_elsewhere() {
    let w = wire();
    let mut d = device(&w, FAR);
    w.lock().unwrap().volts = VecDeque::from([Reply::Late(30, "50.5")]);
    let (tx, rx) = sync_channel(64);
    d.start(PacketSink::new(tx, 0)).unwrap();
    assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
    let shared = d.shared.clone();
    until("a look that found nothing", move || shared.client().question_out);
    d.stop().unwrap();
    until_end(&rx);
    limited(&mut d, 1);
    let next = capture(&mut d);
    assert_eq!(frames(&next), [(11.5, 0.1)]);
    assert!(w.lock().unwrap().discards > 0);
}

/// One get after a limit on TCP: the same, with its own answer.
#[test]
fn on_tcp_a_get_after_a_capture_connects_again_once() {
    let w = wire();
    let mut d = device_on(&w, FAR, "tcp-raw/fake/1");
    limited(&mut d, 1);
    capture(&mut d);
    let reopens = w.lock().unwrap().reopens;
    assert_eq!(d.get(&Key::Enabled, Some("1")).unwrap(), Value::Bool(true));
    assert_eq!(d.get(&Key::Enabled, Some("1")).unwrap(), Value::Bool(true));
    assert_eq!(w.lock().unwrap().reopens, reopens + 1, "once");
}

#[test]
fn a_close_joins_and_lets_go_of_the_sink() {
    let w = wire();
    let mut d = device(&w, FAR);
    // A queue of one, unread, so the thread waits to send, and only a close ends it.
    let (tx, rx) = sync_channel(1);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let shared = Arc::clone(&d.shared.link);
    until("the thread to hold the link", move || Arc::strong_count(&shared) > 1);
    std::thread::sleep(READ);
    assert!(Arc::strong_count(&d.shared.link) > 1, "still running");
    d.close().unwrap();
    assert_eq!(Arc::strong_count(&d.shared.link), 1, "joined");
    // The sender went with it.
    while rx.try_recv().is_ok() {}
    assert!(rx.recv_timeout(READ).is_err());
    assert!(matches!(rx.try_recv(), Err(std::sync::mpsc::TryRecvError::Disconnected)));
}

#[test]
fn a_second_start_is_busy_and_a_closed_device_is_refused() {
    let w = wire();
    let mut d = device(&w, FAR);
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let (tx2, _rx2) = sync_channel(1);
    assert!(matches!(d.start(PacketSink::new(tx2, 0)), Err(Error::Busy)));
    d.stop().unwrap();
    drop(rx);
    d.close().unwrap();
    let (tx3, _rx3) = sync_channel(1);
    assert!(d.start(PacketSink::new(tx3, 0)).is_err());
}

/// A start right after a capture whose End is stuck on a full queue.
#[test]
fn a_start_after_an_end_stuck_on_a_full_queue_does_not_hang() {
    let w = wire();
    let mut d = device(&w, FAR);
    limited(&mut d, 1);
    let (tx, rx) = sync_channel(1);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let w2 = Arc::clone(&w);
    assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
    assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::FrameBegin));
    assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)));
    assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)));
    assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::FrameEnd));
    let shared = d.shared.clone();
    until("the capture over, its End held", move || {
        !shared.client().capturing && w2.lock().unwrap().writes.len() == 3
    });
    let (done_tx, done_rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let (tx, rx2) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let packets = until_end(&rx2);
        let _ = done_tx.send(packets);
    });
    let packets = done_rx.recv_timeout(BOUND).expect("the second capture ended");
    assert_eq!(failure(&packets), None);
    drop(rx);
}

/// A client's set and get between two questions, with a load that answers at once,
/// whose lock a capture takes straight back.
#[test]
fn a_client_gets_in_during_a_capture_and_reads_its_own_answer() {
    let w = wire();
    let mut d = device(&w, FAR);
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let w2 = Arc::clone(&w);
    until("a capture under way", move || w2.lock().unwrap().writes.len() > 4);
    for _ in 0..20 {
        assert_eq!(d.get(&Key::Enabled, Some("1")).unwrap(), Value::Bool(true));
        d.set(&Key::CurrentLimit, &Value::F64(1.5), Some("1")).unwrap();
    }
    assert_eq!(lines(&w).iter().filter(|l| *l == ":CURR:LEV:IMM 1.500").count(), 20);
    d.stop().unwrap();
    // Every frame's voltage is a voltage, and its current a current.
    let mut packets = Vec::new();
    while let Ok((_, p)) = rx.recv_timeout(BOUND) {
        let end = matches!(p, Packet::End);
        packets.push(p);
        if end {
            break;
        }
    }
    let got = frames(&packets);
    assert_ne!(got.len(), 0);
    assert!(got.iter().all(|(v, a)| *v >= 10.0 && *a < 10.0), "{got:?}");
}

/// On a silent line a set goes through, and a get doesn't take the answer on its way.
#[test]
fn with_a_question_out_a_set_goes_and_a_get_is_busy() {
    let w = wire();
    w.lock().unwrap().volts = VecDeque::from([Reply::Silent]);
    let mut d = device(&w, FAR);
    let (tx, rx) = sync_channel(64);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let shared = d.shared.clone();
    until("the question out", move || shared.client().question_out);
    d.set(&Key::CurrentLimit, &Value::F64(1.5), Some("1")).unwrap();
    assert_eq!(lines(&w), ["MEAS:VOLT?", ":CURR:LEV:IMM 1.500"]);
    assert_eq!(w.lock().unwrap().discards, 0, "the line was left for the answer");
    assert!(matches!(d.get(&Key::Enabled, Some("1")), Err(Error::Busy)));
    assert!(!lines(&w).contains(&":INPUT?".to_owned()));
    d.stop().unwrap();
    drop(rx);
}

/// A consumer that holds the queue up is no silence of the load's.
#[test]
fn a_slow_client_is_no_silence() {
    let w = wire();
    let bounds = Bounds { first: READ, long: READ * 2, ..FAR };
    let mut d = device(&w, bounds);
    limited(&mut d, 3);
    let (tx, rx) = sync_channel(1);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let mut packets = Vec::new();
    loop {
        std::thread::sleep(READ * 2);
        let (_, p) = rx.recv_timeout(BOUND).unwrap();
        let end = matches!(p, Packet::End);
        packets.push(p);
        if end {
            break;
        }
    }
    assert_eq!(failure(&packets), None);
    assert_eq!(frames(&packets).len(), 3);
}

#[test]
fn an_answer_loses_its_line_end_only() {
    assert_eq!(answer_text(b"1.5\r\n"), "1.5");
    assert_eq!(answer_text(b"1.5\n"), "1.5");
    assert_eq!(answer_text(b"1.5"), "1.5");
    assert_eq!(answer_text(b"\n"), "");
    assert_eq!(answer_text(b"1.5 \n"), "1.5 ");
}

/// A limit lowered during a capture counts from then, as C's limits are live.
#[test]
fn a_limit_set_during_a_capture_counts() {
    let w = wire();
    let mut d = device(&w, FAR);
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let w2 = Arc::clone(&w);
    until("some rounds", move || w2.lock().unwrap().writes.len() > 8);
    d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    let mut seen = 0;
    loop {
        seen += 1;
        assert!(seen < MOST_PACKETS, "the limit was ignored");
        if matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::End) {
            break;
        }
    }
}
