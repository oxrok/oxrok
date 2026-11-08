// Counterpart in sigrok:
//   libsigrok/src/hardware/yokogawa-dlm/api.c,
//   libsigrok/src/hardware/yokogawa-dlm/protocol.c,
//   Copyright (C) 2014 abraxa (Soeren Apel) <soeren@apelpie.net>,
//   with commits by Uwe Hermann, Soeren Apel, Lars-Peter Clausen,
//   Bert Vermeulen, Martin Ling, Gerhard Sittig, Aurelien Jacobs,
//   Daniel Elstner, Stefan Brüns
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake line: a scope's answers as messages (USBTMC's) or as a stream
//! with no ends (`tcp-raw`'s), released by the number of reads rather than by a clock,
//! so that a stall in the test changes nothing.

use super::*;
use crate::serial::WriteLog;
use crate::{core::Value, scpi::HwInfo, scpi::ScpiTransport};
use std::collections::{HashMap, VecDeque};
use std::sync::mpsc::{Receiver, sync_channel};

const BOUND: Duration = Duration::from_secs(30);
/// A test's own cap on packets: a limit ignored must not fill the memory.
const MOST_PACKETS: usize = 4096;

/// How the line ends an answer.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Carrier {
    Usbtmc,
    TcpRaw,
}

#[derive(Default)]
#[allow(clippy::struct_excessive_bools)]
struct Line {
    writes: WriteLog,
    /// What is still to come, with the empty reads to pass first.
    coming: VecDeque<(u32, Vec<u8>)>,
    /// Answers that override the scope's, each time.
    say: HashMap<String, Vec<u8>>,
    /// Played once, ahead of `say`: answers to `:WAVEFORM:SEND?`, in turn.
    blocks: VecDeque<Vec<u8>>,
    /// Empty reads ahead of each `:WAVEFORM:SEND?`'s answer, in turn.
    block_waits: VecDeque<u32>,
    /// With `blocks` played out, `:WAVEFORM:SEND?` has no answer.
    silent_after_blocks: bool,
    /// Queries with no answer, ever.
    mute: Vec<String>,
    /// Sends that fail, by text.
    refuse: Vec<String>,
    /// A failed `read_begin`s to come, and what they say.
    begin_fails: VecDeque<Error>,
    /// Bytes per piece of a block, and the empty reads ahead of each.
    piece: Option<(usize, u32)>,
    /// `true` forever: the line is gone.
    gone: bool,
    samples: usize,
    reads: u64,
    message_left: usize,
    began: bool,
    closes: u32,
    reopens: u32,
    logic_on: bool,
    bits_on: Vec<usize>,
    chans_on: Vec<usize>,
    read_past_end: u32,
}

/// The scope's answer to a query, as `emu/src/models/yokogawa-dlm.rs` has them, though
/// written for this fake.
fn answer(line: &Line, text: &str) -> Option<Vec<u8>> {
    let word = |s: &str| Some(format!("{s}\n").into_bytes());
    if let Some(n) = text.strip_prefix(":CHANNEL") {
        let ch: usize = n[..1].parse().ok()?;
        return match &n[2..] {
            "DISPLAY?" => word(if line.chans_on.contains(&ch) { "1" } else { "0" }),
            "VDIV?" => word("1.000E+00"),
            "POSITION?" => word("0.00E+00"),
            "COUPLING?" => word("DC"),
            _ => None,
        };
    }
    if let Some(n) = text.strip_prefix(":LOGIC:PODA:BIT") {
        let bit: usize = n.split(':').next()?.parse().ok()?;
        return word(if line.bits_on.contains(&bit) { "1" } else { "0" });
    }
    match text {
        ":LOGIC:MODE?" => word(if line.logic_on { "1" } else { "0" }),
        ":WAVEFORM:RANGE?" => word("5.000E+00"),
        ":WAVEFORM:OFFSET?" => word("0.000E+00"),
        ":TIMEBASE:TDIV?" => word("1.000E-03"),
        ":TRIGGER:DELAY:TIME?" => word("0.00E+00"),
        ":TRIGGER:ATRIGGER:SIMPLE:SOURCE?" | "*OPC?" => word("1"),
        ":TRIGGER:ATRIGGER:SIMPLE:SLOPE?" => word("RISE"),
        ":WAVEFORM:LENGTH?" => word(&line.samples.to_string()),
        ":WAVEFORM:SRATE?" => word("1.25E+06"),
        ":WAVEFORM:SEND? 1" => {
            let data: Vec<u8> = (0..line.samples).map(|i| (i * 3) as u8).collect();
            let mut bytes = format!("#8{:08}", data.len()).into_bytes();
            bytes.extend_from_slice(&data);
            bytes.push(b'\n');
            Some(bytes)
        }
        _ => None,
    }
}

struct Fake {
    line: Arc<Mutex<Line>>,
    carrier: Carrier,
}

impl ScpiTransport for Fake {
    fn resource(&self) -> &'static str {
        "fake"
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        let mut line = self.line.lock().unwrap();
        if line.gone {
            return Err(Error::Disconnected);
        }
        let text = String::from_utf8_lossy(data).trim_end().to_owned();
        line.writes.push((Instant::now(), data.to_vec()));
        if line.refuse.contains(&text) {
            return Err(Error::device("the line said no"));
        }
        if !text.contains('?') {
            return Ok(());
        }
        if line.mute.contains(&text) {
            return Ok(());
        }
        let block = text == ":WAVEFORM:SEND? 1";
        let wait = if block { line.block_waits.pop_front().unwrap_or(0) } else { 0 };
        let reply = if block && let Some(next) = line.blocks.pop_front() {
            Some(next)
        } else if block && line.silent_after_blocks {
            None
        } else if let Some(said) = line.say.get(&text) {
            Some(said.clone())
        } else {
            answer(&line, &text)
        };
        let Some(reply) = reply else { return Ok(()) };
        match line.piece {
            Some((size, wait)) if block => {
                for piece in reply.chunks(size) {
                    line.coming.push_back((wait, piece.to_vec()));
                }
                // No piece at all for an empty answer: a silent one.
            }
            _ => line.coming.push_back((wait, reply)),
        }
        Ok(())
    }

    fn read_begin(&mut self) -> Result<()> {
        let mut line = self.line.lock().unwrap();
        if line.gone {
            return Err(Error::Disconnected);
        }
        if let Some(e) = line.begin_fails.pop_front() {
            return Err(e);
        }
        line.began = false;
        line.message_left = 0;
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut line = self.line.lock().unwrap();
        line.reads += 1;
        if line.gone {
            return Err(Error::Disconnected);
        }
        if self.carrier == Carrier::Usbtmc && line.began && line.message_left == 0 {
            line.read_past_end += 1;
            return Err(Error::device("USBTMC: read past the end of the message"));
        }
        let ready = match line.coming.front_mut() {
            Some((wait, _)) if *wait > 0 => {
                *wait -= 1;
                false
            }
            Some(_) => true,
            None => false,
        };
        if !ready {
            if self.carrier == Carrier::TcpRaw && line.began {
                line.message_left = 0;
            }
            drop(line);
            std::thread::sleep(timeout);
            return Ok(0);
        }
        let (_, mut bytes) = line.coming.pop_front().unwrap();
        // The whole message on USBTMC, whatever its pieces.
        if !line.began {
            line.began = true;
            line.message_left = bytes.len() + line.coming.iter().map(|c| c.1.len()).sum::<usize>();
        }
        let n = bytes.len().min(buf.len());
        buf[..n].copy_from_slice(&bytes[..n]);
        if n < bytes.len() {
            let tail = bytes.split_off(n);
            line.coming.push_front((0, tail));
        }
        match self.carrier {
            Carrier::Usbtmc => line.message_left -= n,
            // A short read ends a response, a full one may continue.
            Carrier::TcpRaw => line.message_left = usize::from(n == buf.len()),
        }
        Ok(n)
    }

    fn read_complete(&self) -> bool {
        let line = self.line.lock().unwrap();
        line.began && line.message_left == 0
    }

    fn tags_answers(&self) -> bool {
        self.carrier == Carrier::Usbtmc
    }

    fn close(&mut self) -> Result<()> {
        self.line.lock().unwrap().closes += 1;
        Ok(())
    }

    fn reopen(&mut self) -> Result<()> {
        self.line.lock().unwrap().reopens += 1;
        Ok(())
    }
}

fn hw() -> HwInfo {
    HwInfo {
        manufacturer: "YOKOGAWA".to_owned(),
        model: "710110".to_owned(),
        serial_number: "1".to_owned(),
        firmware_version: "F".to_owned(),
    }
}

/// A short read timeout and short bounds, each test choosing what it is about.
fn far_bounds() -> Bounds {
    Bounds {
        warn: Duration::from_secs(600),
        first: Duration::from_secs(600),
        later: Duration::from_secs(600),
        gap: Duration::from_secs(600),
        empty: Duration::from_secs(600),
        empty_pause: Duration::ZERO,
    }
}

fn device_on(
    id: &str,
    carrier: Carrier,
    f: impl FnOnce(&mut Line),
) -> (YokogawaDevice, Arc<Mutex<Line>>) {
    let (config, name) = protocol::model_get(id).unwrap();
    let mut line = Line { samples: 16, chans_on: vec![1], ..Line::default() };
    f(&mut line);
    let line = Arc::new(Mutex::new(line));
    let fake = Fake { line: Arc::clone(&line), carrier };
    let mut scpi = Scpi::new(Box::new(fake));
    scpi.read_timeout = Duration::from_millis(1);
    let mut d = YokogawaDevice::new(config, name, &hw(), scpi);
    d.bounds = far_bounds();
    (d, line)
}

fn device(f: impl FnOnce(&mut Line)) -> (YokogawaDevice, Arc<Mutex<Line>>) {
    device_on("710110", Carrier::Usbtmc, f)
}

/// Every write so far, as text.
fn writes(line: &Arc<Mutex<Line>>) -> Vec<String> {
    let line = line.lock().unwrap();
    line.writes.iter().map(|w| String::from_utf8_lossy(&w.1).trim_end().to_owned()).collect()
}

fn forget_writes(line: &Arc<Mutex<Line>>) {
    line.lock().unwrap().writes = WriteLog::default();
}

/// A test that fails rather than hangs.
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
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("the test hung"),
    }
}

/// All packets of a capture up to its end.
fn drain(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    while out.len() < MOST_PACKETS && Instant::now() < deadline {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((_, Packet::End)) => {
                out.push(Packet::End);
                return out;
            }
            Ok((_, p)) => out.push(p),
            Err(_) => break,
        }
    }
    panic!("no end in {} packets", out.len());
}

/// A capture of an opened device.
fn capture(d: &mut YokogawaDevice) -> Vec<Packet> {
    let (tx, rx) = sync_channel(1024);
    d.start(PacketSink::new(tx, 0)).unwrap();
    drain(&rx)
}

fn failure(packets: &[Packet]) -> Option<String> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(why) => Some(why.clone()),
        _ => None,
    })
}

fn analog(packets: &[Packet]) -> Vec<f32> {
    let mut out = Vec::new();
    for p in packets {
        if let Packet::Analog(a) = p {
            out.extend_from_slice(&a.samples);
        }
    }
    out
}

fn logic(packets: &[Packet]) -> Vec<u8> {
    let mut out = Vec::new();
    for p in packets {
        if let Packet::Logic(l) = p {
            out.extend_from_slice(&l.data);
        }
    }
    out
}

fn kinds(packets: &[Packet]) -> Vec<&'static str> {
    packets
        .iter()
        .map(|p| match p {
            Packet::Header { .. } => "header",
            Packet::FrameBegin => "begin",
            Packet::FrameEnd => "end-frame",
            Packet::Analog(_) => "analog",
            Packet::Logic(_) => "logic",
            Packet::Failed(_) => "failed",
            Packet::End => "end",
            _ => "other",
        })
        .collect()
}

fn block(data: &[u8]) -> Vec<u8> {
    let mut bytes = format!("#8{:08}", data.len()).into_bytes();
    bytes.extend_from_slice(data);
    bytes.push(b'\n');
    bytes
}

#[test]
fn an_open_reads_every_setting_in_cs_order() {
    let (mut d, line) = device_on("710105", Carrier::Usbtmc, |l| l.chans_on = vec![2]);
    d.open().unwrap();
    assert_eq!(
        writes(&line),
        [
            ":CHANNEL1:DISPLAY?",
            ":CHANNEL1:VDIV?",
            ":CHANNEL1:POSITION?",
            ":WAVEFORM:TRACE 1",
            ":WAVEFORM:RANGE?",
            ":WAVEFORM:TRACE 1",
            ":WAVEFORM:OFFSET?",
            ":CHANNEL1:COUPLING?",
            ":CHANNEL2:DISPLAY?",
            ":CHANNEL2:VDIV?",
            ":CHANNEL2:POSITION?",
            ":WAVEFORM:TRACE 2",
            ":WAVEFORM:RANGE?",
            ":WAVEFORM:TRACE 2",
            ":WAVEFORM:OFFSET?",
            ":CHANNEL2:COUPLING?",
            ":TIMEBASE:TDIV?",
            ":TRIGGER:DELAY:TIME?",
            ":TRIGGER:ATRIGGER:SIMPLE:SOURCE?",
            ":TRIGGER:ATRIGGER:SIMPLE:SLOPE?",
            ":WAVEFORM:LENGTH?",
            ":WAVEFORM:TRACE 1",
            ":WAVEFORM:RECORD 0",
            ":WAVEFORM:SRATE?",
        ]
    );
    assert_eq!(line.lock().unwrap().reopens, 1);
    assert!(!d.channels()[0].enabled);
    assert!(d.channels()[1].enabled);
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(1_250_000));
    assert_eq!(d.get(&Key::Timebase, None).unwrap(), Value::Rational(1, 1000));
    assert_eq!(d.get(&Key::HorizTriggerpos, None).unwrap(), Value::F64(0.5));
    assert_eq!(d.get(&Key::TriggerSource, None).unwrap(), Value::Str("1".to_owned()));
    assert_eq!(d.get(&Key::TriggerSlope, None).unwrap(), Value::Str("r".to_owned()));
    assert_eq!(d.get(&Key::NumHdiv, None).unwrap(), Value::I32(10));
    assert_eq!(d.get(&Key::NumVdiv, Some("1")).unwrap(), Value::I32(8));
    assert_eq!(d.get(&Key::Vdiv, Some("2")).unwrap(), Value::Rational(1, 1));
    assert_eq!(d.get(&Key::Coupling, Some("2")).unwrap(), Value::Str("DC".to_owned()));
}

#[test]
fn a_digital_model_reads_its_bits_and_pods() {
    let (mut d, line) = device_on("701320", Carrier::Usbtmc, |l| {
        l.chans_on = vec![];
        l.bits_on = vec![2];
        l.logic_on = true;
    });
    d.open().unwrap();
    let w = writes(&line);
    assert_eq!(w.iter().filter(|c| c.starts_with(":LOGIC:PODA:BIT")).count(), 16);
    assert!(w.contains(&":LOGIC:PODA:BIT16:DISPLAY?".to_owned()));
    assert_eq!(w.iter().filter(|c| *c == ":LOGIC:MODE?").count(), 4, "a pod's is C's one query");
    assert!(d.channels()[5].enabled, "A1, the second bit");
    assert_eq!(d.channels()[5].name, "A1");
    assert_eq!(d.channels()[5].index, 1, "its bit in a byte");
    assert_eq!(d.channel_groups().len(), 8);
    assert_eq!(d.channel_groups()[5].channels, vec![12, 13, 14, 15, 16, 17, 18, 19]);
}

/// C's `result &=` makes these OK, and every sample the offset.
#[test]
fn a_failed_range_or_offset_fails_the_open() {
    for query in [":WAVEFORM:RANGE?", ":WAVEFORM:OFFSET?"] {
        let (mut d, _) = device(|l| l.mute.push(query.to_owned()));
        d.open().unwrap_err();
        let (mut d, _) = device(|l| {
            l.say.insert(query.to_owned(), b"abc\n".to_vec());
        });
        let text = d.open().unwrap_err().to_string();
        assert_eq!(text, format!("device error: {query}: not a number: 'abc'"));
        let (mut d, _) = device(|l| {
            l.say.insert(query.to_owned(), b"\n".to_vec());
        });
        d.open().unwrap_err();
        let (mut d, _) = device(|l| l.refuse.push(":WAVEFORM:TRACE 1".to_owned()));
        d.open().unwrap_err();
    }
}

#[test]
fn what_an_open_cannot_read_fails_it() {
    let fails = |query: &str, text: &[u8], want: &str| {
        let (mut d, _) = device(|l| {
            l.say.insert(query.to_owned(), text.to_vec());
        });
        assert_eq!(d.open().unwrap_err().to_string(), format!("device error: {want}"), "{query}");
    };
    fails(":CHANNEL1:VDIV?", b"abc\n", "vdiv: no such value: 'abc'");
    fails(":CHANNEL1:POSITION?", b"abc\n", ":CHANNEL1:POSITION?: not a number: 'abc'");
    fails(":CHANNEL1:COUPLING?", b"ICE\n", "coupling: no such value: 'ICE'");
    fails(":TIMEBASE:TDIV?", b"abc\n", "timebase: no such value: 'abc'");
    fails(":TRIGGER:DELAY:TIME?", b"abc\n", ":TRIGGER:DELAY:TIME?: not a number: 'abc'");
    fails(":TRIGGER:ATRIGGER:SIMPLE:SOURCE?", b"9\n", "trigger source: no such value: '9'");
    fails(":TRIGGER:ATRIGGER:SIMPLE:SLOPE?", b"ENT\n", "trigger slope: no such value: 'ENT'");
    fails(":WAVEFORM:LENGTH?", b"-5\n", ":WAVEFORM:LENGTH?: not a length: '-5'");
    fails(":CHANNEL1:DISPLAY?", b"maybe\n", "SCPI: not a boolean: 'maybe'");
    fails(":LOGIC:MODE?", b"maybe\n", "SCPI: not a boolean: 'maybe'");
    fails(":LOGIC:PODA:BIT1:DISPLAY?", b"maybe\n", "SCPI: not a boolean: 'maybe'");
}

/// Its result is dropped, in C too.
#[test]
fn a_silent_rate_leaves_the_open_and_a_zero() {
    let (mut d, _) = device(|l| l.mute.push(":WAVEFORM:SRATE?".to_owned()));
    d.open().unwrap();
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(0));
    let (mut d, _) = device(|l| {
        l.say.insert(":WAVEFORM:SRATE?".to_owned(), b"-1.0\n".to_vec());
    });
    d.open().unwrap();
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(0));
}

#[test]
fn settings_go_out_with_their_acknowledgements() {
    let (mut d, line) = device(|_| {});
    d.open().unwrap();
    let set = |d: &mut YokogawaDevice, key: Key, value: Value, group: Option<&str>| {
        forget_writes(&line);
        d.set(&key, &value, group).unwrap();
        writes(&line)
    };
    let v = set(&mut d, Key::Vdiv, Value::Rational(1, 2), Some("2"));
    assert_eq!(v, [":CHANNEL2:VDIV 5.000000E-01", "*OPC?", "*OPC?"]);
    assert_eq!(d.get(&Key::Vdiv, Some("2")).unwrap(), Value::Rational(500, 1000));
    assert_eq!(d.get(&Key::Vdiv, Some("1")).unwrap(), Value::Rational(1, 1), "not its neighbour");
    let v = set(&mut d, Key::Coupling, Value::Str("GND".to_owned()), Some("3"));
    assert_eq!(v, [":CHANNEL3:COUPLING GND", "*OPC?", "*OPC?"]);
    assert_eq!(d.get(&Key::Coupling, Some("3")).unwrap(), Value::Str("GND".to_owned()));
    let v = set(&mut d, Key::Timebase, Value::Rational(1, 500_000), None);
    assert_eq!(
        v,
        [
            ":TIMEBASE:TDIV 2.000000E-06",
            "*OPC?",
            ":WAVEFORM:TRACE 1",
            ":WAVEFORM:RECORD 0",
            ":WAVEFORM:SRATE?"
        ]
    );
    assert_eq!(d.get(&Key::Timebase, None).unwrap(), Value::Rational(2, 1_000_000));
    let v = set(&mut d, Key::HorizTriggerpos, Value::F64(0.25), None);
    assert_eq!(v, [":TRIGGER:DELAY:TIME 5.000000E-06", "*OPC?"]);
    assert_eq!(d.get(&Key::HorizTriggerpos, None).unwrap(), Value::F64(0.25));
    let v = set(&mut d, Key::TriggerSlope, Value::Str("f".to_owned()), None);
    assert_eq!(v, [":TRIGGER:ATRIGGER:SIMPLE:SLOPE FALL", "*OPC?"]);
    assert_eq!(d.get(&Key::TriggerSlope, None).unwrap(), Value::Str("f".to_owned()));
    let v = set(&mut d, Key::TriggerSlope, Value::Str("r".to_owned()), None);
    assert_eq!(v, [":TRIGGER:ATRIGGER:SIMPLE:SLOPE RISE", "*OPC?"]);
    let v = set(&mut d, Key::TriggerSource, Value::Str("BIT3".to_owned()), None);
    assert_eq!(v, [":TRIGGER:ATRIGGER:SIMPLE:SOURCE BIT3", "*OPC?"]);
    assert_eq!(d.get(&Key::TriggerSource, None).unwrap(), Value::Str("BIT3".to_owned()));
    let v = set(&mut d, Key::LimitFrames, Value::U64(3), None);
    assert_eq!(v, ["*OPC?"]);
    assert_eq!(d.frame_limit, 3);
}

#[test]
fn what_no_setting_is_stays_off_the_wire() {
    let (mut d, line) = device(|_| {});
    d.open().unwrap();
    forget_writes(&line);
    d.set(&Key::HorizTriggerpos, &Value::F64(1.5), None).unwrap_err();
    d.set(&Key::HorizTriggerpos, &Value::F64(-0.5), None).unwrap_err();
    d.set(&Key::HorizTriggerpos, &Value::F64(f64::NAN), None).unwrap_err();
    d.set(&Key::Timebase, &Value::Rational(3, 1000), None).unwrap_err();
    d.set(&Key::Vdiv, &Value::Rational(3, 1), Some("1")).unwrap_err();
    d.set(&Key::Vdiv, &Value::Rational(1, 1), None).unwrap_err();
    d.set(&Key::Vdiv, &Value::Rational(1, 1), Some("POD0")).unwrap_err();
    d.set(&Key::Coupling, &Value::Str("ICE".to_owned()), Some("1")).unwrap_err();
    d.set(&Key::Coupling, &Value::Str("DC".to_owned()), None).unwrap_err();
    d.set(&Key::TriggerSource, &Value::Str("9".to_owned()), None).unwrap_err();
    d.set(&Key::TriggerSlope, &Value::Str("x".to_owned()), None).unwrap_err();
    d.set(&Key::Samplerate, &Value::U64(1), None).unwrap_err();
    assert_eq!(writes(&line), Vec::<String>::new());
    assert_eq!(d.get(&Key::HorizTriggerpos, None).unwrap(), Value::F64(0.5));
}

/// C sets its state first. A get after a failed set must not answer the wanted value.
#[test]
fn a_failed_setting_leaves_the_state_as_it_was() {
    let (mut d, line) = device(|_| {});
    d.open().unwrap();
    line.lock().unwrap().refuse.push(":CHANNEL1:VDIV 2.000000E-03".to_owned());
    d.set(&Key::Vdiv, &Value::Rational(2, 1000), Some("1")).unwrap_err();
    assert_eq!(d.get(&Key::Vdiv, Some("1")).unwrap(), Value::Rational(1, 1));
    line.lock().unwrap().refuse.push(":TRIGGER:ATRIGGER:SIMPLE:SOURCE 2".to_owned());
    d.set(&Key::TriggerSource, &Value::Str("2".to_owned()), None).unwrap_err();
    assert_eq!(d.get(&Key::TriggerSource, None).unwrap(), Value::Str("1".to_owned()));
    line.lock().unwrap().refuse.push(":TRIGGER:ATRIGGER:SIMPLE:SLOPE FALL".to_owned());
    d.set(&Key::TriggerSlope, &Value::Str("f".to_owned()), None).unwrap_err();
    assert_eq!(d.get(&Key::TriggerSlope, None).unwrap(), Value::Str("r".to_owned()));
    line.lock().unwrap().refuse.push(":CHANNEL1:COUPLING AC".to_owned());
    d.set(&Key::Coupling, &Value::Str("AC".to_owned()), Some("1")).unwrap_err();
    assert_eq!(d.get(&Key::Coupling, Some("1")).unwrap(), Value::Str("DC".to_owned()));
    line.lock().unwrap().refuse.push(":TRIGGER:DELAY:TIME 5.000000E-03".to_owned());
    d.set(&Key::HorizTriggerpos, &Value::F64(0.0), None).unwrap_err();
    assert_eq!(d.get(&Key::HorizTriggerpos, None).unwrap(), Value::F64(0.5));
    line.lock().unwrap().refuse.push(":TIMEBASE:TDIV 2.000000E-03".to_owned());
    d.set(&Key::Timebase, &Value::Rational(2, 1000), None).unwrap_err();
    assert_eq!(d.get(&Key::Timebase, None).unwrap(), Value::Rational(1, 1000));
}

/// `*OPC?` gets 30 s of silence, by `Scpi`: here 1 ms, and the setting isn't taken as made.
#[test]
fn a_silent_acknowledgement_fails_the_setting() {
    let (mut d, line) = device(|_| {});
    d.open().unwrap();
    d.with_scpi(|s| {
        s.opc_silence = Duration::from_millis(5);
        Ok(())
    })
    .unwrap();
    line.lock().unwrap().mute.push("*OPC?".to_owned());
    let text = d.set(&Key::Vdiv, &Value::Rational(2, 1000), Some("1")).unwrap_err().to_string();
    assert!(text.contains("no answer to *OPC?"), "{text}");
    assert_eq!(d.get(&Key::Vdiv, Some("1")).unwrap(), Value::Rational(1, 1));
}

#[test]
fn a_timebase_set_and_a_silent_rate_says_so_with_the_timebase_taken() {
    let (mut d, line) = device(|_| {});
    d.open().unwrap();
    line.lock().unwrap().mute.push(":WAVEFORM:SRATE?".to_owned());
    d.set(&Key::Timebase, &Value::Rational(2, 1000), None).unwrap_err();
    assert_eq!(d.get(&Key::Timebase, None).unwrap(), Value::Rational(2, 1000));
}

#[test]
fn a_channel_goes_to_the_scope_when_it_changes() {
    let (mut d, line) = device(|l| l.chans_on = vec![1, 2]);
    d.open().unwrap();
    forget_writes(&line);
    d.enable_channel(0, true).unwrap();
    assert_eq!(writes(&line), Vec::<String>::new(), "no change");
    d.enable_channel(0, false).unwrap();
    assert_eq!(writes(&line), [":CHANNEL1:DISPLAY OFF"]);
    assert!(!d.channels()[0].enabled);
    forget_writes(&line);
    d.enable_channel(2, true).unwrap();
    assert_eq!(writes(&line), [":CHANNEL3:DISPLAY ON"]);
    // A logic bit brings its pod.
    forget_writes(&line);
    d.enable_channel(4, true).unwrap();
    assert_eq!(writes(&line), [":LOGIC:PODA:BIT1:DISPLAY ON", ":LOGIC:MODE ON"]);
    forget_writes(&line);
    d.enable_channel(5, true).unwrap();
    assert_eq!(writes(&line), [":LOGIC:PODA:BIT2:DISPLAY ON"], "the pod is on");
    // The pod goes only with its last bit.
    forget_writes(&line);
    d.enable_channel(4, false).unwrap();
    assert_eq!(writes(&line), [":LOGIC:PODA:BIT1:DISPLAY OFF"]);
    forget_writes(&line);
    d.enable_channel(5, false).unwrap();
    assert_eq!(writes(&line), [":LOGIC:PODA:BIT2:DISPLAY OFF", ":LOGIC:MODE OFF"]);
}

/// With the pod on and no bit shown, as C does at any channel's change.
#[test]
fn any_channel_change_can_turn_the_pod_off() {
    let (mut d, line) = device(|l| {
        l.chans_on = vec![1];
        l.logic_on = true;
    });
    d.open().unwrap();
    forget_writes(&line);
    d.enable_channel(1, true).unwrap();
    assert_eq!(writes(&line), [":CHANNEL2:DISPLAY ON", ":LOGIC:MODE OFF"]);
}

#[test]
fn a_refused_channel_stays_as_it_was() {
    let (mut d, line) = device(|l| l.chans_on = vec![1]);
    d.open().unwrap();
    line.lock().unwrap().refuse.push(":CHANNEL1:DISPLAY OFF".to_owned());
    d.enable_channel(0, false).unwrap_err();
    assert!(d.channels()[0].enabled);
    line.lock().unwrap().refuse.push(":LOGIC:PODA:BIT1:DISPLAY ON".to_owned());
    d.enable_channel(4, true).unwrap_err();
    assert!(!d.channels()[4].enabled);
    d.enable_channel(99, true).unwrap_err();
}

#[test]
fn a_frame_of_analog_samples() {
    let (mut d, line) = device(|l| {
        l.chans_on = vec![1, 3];
        l.samples = 4;
    });
    d.open().unwrap();
    forget_writes(&line);
    let packets = capture(&mut d);
    assert_eq!(
        kinds(&packets),
        ["header", "begin", "analog", "analog", "end-frame", "end"],
        "a header, which C doesn't send, and one frame",
    );
    assert_eq!(failure(&packets), None);
    assert_eq!(
        writes(&line),
        [
            ":WAVEFORM:FORMAT BYTE",
            ":WAVEFORM:RECORD 0",
            ":WAVEFORM:START 0",
            ":WAVEFORM:END 124999999",
            ":WAVEFORM:TRACE 1",
            ":WAVEFORM:SEND? 1",
            ":WAVEFORM:FORMAT BYTE",
            ":WAVEFORM:RECORD 0",
            ":WAVEFORM:START 0",
            ":WAVEFORM:END 124999999",
            ":WAVEFORM:TRACE 3",
            ":WAVEFORM:SEND? 1",
        ]
    );
    let Packet::Analog(a) = &packets[3] else { panic!() };
    assert_eq!(a.channels, vec![2]);
    assert_eq!(a.samples, vec![0.0, 1.2, 2.4, 3.6]);
    assert_eq!(a.digits, 2);
    assert_eq!(a.mq, Mq::Voltage);
    assert_eq!(a.unit, Unit::Volt);
}

#[test]
fn signed_bytes_become_volts_with_each_channels_range_and_offset() {
    let (mut d, _) = device(|l| {
        l.samples = 3;
        l.say.insert(":WAVEFORM:RANGE?".to_owned(), b"1.250E+01\n".to_vec());
        l.say.insert(":WAVEFORM:OFFSET?".to_owned(), b"1.000E+00\n".to_vec());
        l.blocks.push_back(block(&[0x00, 0x7f, 0x80]));
    });
    d.open().unwrap();
    assert_eq!(analog(&capture(&mut d)), vec![1.0, 128.0, -127.0]);
}

/// All bits share a byte, so all share a read, and each bit is its index.
#[test]
fn a_frame_of_logic_is_one_read_of_bytes() {
    let (mut d, line) = device(|l| {
        l.chans_on = vec![];
        l.bits_on = vec![1, 3];
        l.logic_on = true;
        l.samples = 4;
    });
    d.open().unwrap();
    forget_writes(&line);
    let packets = capture(&mut d);
    assert_eq!(kinds(&packets), ["header", "begin", "logic", "end-frame", "end"]);
    assert_eq!(logic(&packets), vec![0, 3, 6, 9]);
    let Packet::Logic(l) = &packets[2] else { panic!() };
    assert_eq!(l.unit_size, 1);
    let w = writes(&line);
    assert_eq!(w.iter().filter(|c| *c == ":WAVEFORM:SEND? 1").count(), 1);
    assert!(w.contains(&":WAVEFORM:TRACE LOGIC".to_owned()));
}

#[test]
fn channel_4_and_logic_are_refused_ahead_of_the_wire() {
    let (mut d, line) = device(|l| {
        l.chans_on = vec![4];
        l.bits_on = vec![1];
    });
    d.open().unwrap();
    forget_writes(&line);
    let text = d.check_capture(None).unwrap_err().to_string();
    assert_eq!(text, "not supported: Invalid channel configuration specified!");
    let (tx, _rx) = sync_channel(4);
    d.start(PacketSink::new(tx, 0)).unwrap_err();
    assert_eq!(writes(&line), Vec::<String>::new());
    // The instrument stays, for a get or another start.
    d.with_scpi(|_| Ok(())).unwrap();
}

#[test]
fn no_channel_enabled_is_refused() {
    let (mut d, _) = device(|l| l.chans_on = vec![]);
    d.open().unwrap();
    d.check_capture(None).unwrap_err();
    let (tx, _rx) = sync_channel(4);
    d.start(PacketSink::new(tx, 0)).unwrap_err();
}

#[test]
fn a_block_cut_short_fails_on_each_carrier() {
    for carrier in [Carrier::Usbtmc, Carrier::TcpRaw] {
        let (mut d, _) = device_on("710110", carrier, |l| {
            let mut cut = block(&[1; 16]);
            cut.truncate(14);
            l.blocks.push_back(cut);
        });
        d.bounds.gap = Duration::from_millis(10);
        d.open().unwrap();
        let packets = capture(&mut d);
        assert_eq!(
            failure(&packets).as_deref(),
            Some("device error: Truncated waveform data packet received.")
        );
        assert_eq!(kinds(&packets), ["header", "begin", "failed", "end"]);
    }
}

/// C reads no count: the line end is its last sample.
#[test]
fn a_header_that_says_less_than_the_record_is_cut_short() {
    let (mut d, _) = device(|l| {
        let mut less = block(&[1; 16]);
        less.splice(2..10, b"00000015".iter().copied());
        l.blocks.push_back(less);
    });
    d.open().unwrap();
    let packets = capture(&mut d);
    assert_eq!(
        failure(&packets).as_deref(),
        Some("device error: Truncated waveform data packet received.")
    );
}

#[test]
fn a_block_longer_than_the_record_gives_the_records_samples() {
    let (mut d, _) = device(|l| {
        l.samples = 2;
        l.blocks.push_back(block(&[25, 50, 75, 100]));
    });
    d.open().unwrap();
    let packets = capture(&mut d);
    assert_eq!(analog(&packets), vec![10.0, 20.0]);
}

#[test]
fn what_is_no_block_says_so() {
    for text in [&b"X\n"[..], b"\n", b"#", b"#8", b"#8abc\n", b"#A\n"] {
        let (mut d, _) = device(|l| l.blocks.push_back(text.to_vec()));
        d.open().unwrap();
        let packets = capture(&mut d);
        assert_eq!(
            failure(&packets).as_deref(),
            Some("device error: Encountered malformed block data header."),
            "{text:?}"
        );
        assert_eq!(kinds(&packets), ["header", "begin", "failed", "end"], "{text:?}");
    }
}

/// C goes on with the frame's first channel only in its own crash.
#[test]
fn an_empty_block_stops_the_scope_and_asks_again() {
    bounded(|| {
        let (mut d, line) = device(|l| {
            l.samples = 2;
            l.blocks.push_back(block(&[]));
            l.blocks.push_back(block(&[]));
            l.blocks.push_back(block(&[25, 50]));
        });
        d.open().unwrap();
        forget_writes(&line);
        let packets = capture(&mut d);
        assert_eq!(kinds(&packets), ["header", "begin", "analog", "end-frame", "end"], "one begin");
        assert_eq!(analog(&packets), vec![10.0, 20.0]);
        let w = writes(&line);
        assert_eq!(w.iter().filter(|c| *c == ":STOP").count(), 2);
        assert_eq!(w.iter().filter(|c| *c == ":WAVEFORM:SEND? 1").count(), 3);
        assert_eq!(w[6], ":STOP", "right after the first empty answer");
        assert_eq!(w[7], ":WAVEFORM:FORMAT BYTE", "and the whole request again");
    });
}

#[test]
fn empty_blocks_for_ever_end_a_capture() {
    bounded(|| {
        let (mut d, line) = device(|l| {
            l.say.insert(":WAVEFORM:SEND? 1".to_owned(), block(&[]));
        });
        d.bounds.empty = Duration::from_millis(30);
        d.bounds.empty_pause = Duration::from_millis(1);
        d.open().unwrap();
        forget_writes(&line);
        let packets = capture(&mut d);
        assert_eq!(
            failure(&packets).as_deref(),
            Some("device error: The scope sends only empty waveforms: 0.03 s through :STOP.")
        );
        assert!(writes(&line).iter().filter(|c| *c == ":STOP").count() >= 1);
    });
}

/// `#0` is 0 bytes to C.
#[test]
fn an_indefinite_block_counts_as_empty() {
    bounded(|| {
        let (mut d, line) = device(|l| {
            l.samples = 1;
            l.blocks.push_back(b"#0\n".to_vec());
            l.blocks.push_back(block(&[25]));
        });
        d.open().unwrap();
        forget_writes(&line);
        assert_eq!(analog(&capture(&mut d)), vec![10.0]);
        assert_eq!(writes(&line).iter().filter(|c| *c == ":STOP").count(), 1);
    });
}

#[test]
fn a_silent_scope_ends_a_capture_with_no_sample_at_the_short_bound() {
    bounded(|| {
        for carrier in [Carrier::Usbtmc, Carrier::TcpRaw] {
            let (mut d, _) = device_on("710110", carrier, |l| {
                l.mute.push(":WAVEFORM:SEND? 1".to_owned());
            });
            d.bounds.first = Duration::from_millis(30);
            d.bounds.warn = Duration::from_millis(10);
            d.open().unwrap();
            let packets = capture(&mut d);
            assert_eq!(
                failure(&packets).as_deref(),
                Some("device error: No answer from the scope in 0.03 s.")
            );
        }
    });
}

/// The short bound long past, and the long one not: one sample changes which.
#[test]
fn after_a_sample_silence_runs_past_the_short_bound() {
    bounded(|| {
        let (mut d, _) = device(|l| {
            l.chans_on = vec![1, 2];
            l.samples = 1;
            l.blocks.push_back(block(&[25]));
            l.blocks.push_back(block(&[50]));
            // Eight reads of a millisecond at least, against a bound of one.
            l.block_waits.push_back(0);
            l.block_waits.push_back(8);
        });
        d.bounds.first = Duration::from_millis(1);
        d.open().unwrap();
        let packets = capture(&mut d);
        assert_eq!(failure(&packets), None);
        assert_eq!(analog(&packets), vec![10.0, 20.0]);
    });
}

/// And silence past the long bound ends it, with what it had.
#[test]
fn after_a_sample_silence_ends_at_the_long_bound() {
    bounded(|| {
        let (mut d, _) = device(|l| {
            l.chans_on = vec![1, 2];
            l.samples = 1;
            l.blocks.push_back(block(&[25]));
            l.silent_after_blocks = true;
        });
        d.bounds.later = Duration::from_millis(30);
        d.bounds.warn = Duration::from_millis(5);
        d.open().unwrap();
        let packets = capture(&mut d);
        assert_eq!(
            failure(&packets).as_deref(),
            Some("device error: No answer from the scope in 0.03 s.")
        );
        assert_eq!(analog(&packets), vec![10.0], "what it had stays");
    });
}

/// A late answer on a line with no tags: a read waits for it, with no second question.
#[test]
fn a_late_block_is_the_answer_to_its_own_request() {
    bounded(|| {
        for carrier in [Carrier::Usbtmc, Carrier::TcpRaw] {
            let (mut d, line) = device_on("710110", carrier, |l| {
                l.samples = 2;
                l.piece = Some((1 << 20, 50));
                l.blocks.push_back(block(&[25, 50]));
            });
            d.open().unwrap();
            forget_writes(&line);
            let packets = capture(&mut d);
            assert_eq!(analog(&packets), vec![10.0, 20.0]);
            let w = writes(&line);
            assert_eq!(w.iter().filter(|c| *c == ":WAVEFORM:SEND? 1").count(), 1);
        }
    });
}

/// `tcp-raw` at the pace of a stack: C's short read is its end, so this is "Truncated"
/// there. Here the header says what is to come.
#[test]
fn a_block_in_pieces_on_tcp_is_whole() {
    bounded(|| {
        for (size, wait) in [(1, 0), (7, 1), (1460, 2), (4096, 0)] {
            let (mut d, _) = device_on("710110", Carrier::TcpRaw, |l| {
                l.samples = 5000;
                l.piece = Some((size, wait));
            });
            d.open().unwrap();
            let packets = capture(&mut d);
            assert_eq!(failure(&packets), None, "{size}");
            assert_eq!(analog(&packets).len(), 5000, "{size}");
            assert_eq!(kinds(&packets), ["header", "begin", "analog", "end-frame", "end"]);
        }
    });
}

/// A read of exactly the buffer's size says nothing of an end, and silence does: on
/// C's blocking read this hangs for good.
#[test]
fn an_answer_of_a_whole_read_ends_on_silence() {
    bounded(|| {
        let (mut d, _) = device_on("710110", Carrier::TcpRaw, |l| l.samples = 4085);
        d.open().unwrap();
        let packets = capture(&mut d);
        assert_eq!(failure(&packets), None);
        assert_eq!(analog(&packets).len(), 4085);
    });
}

/// A line end left behind must not pass for the head of the next answer.
#[test]
fn a_line_end_that_comes_late_begins_the_next_answer() {
    bounded(|| {
        let (mut d, _) = device_on("710110", Carrier::TcpRaw, |l| {
            l.chans_on = vec![1, 2];
            l.samples = 2;
            let mut first = block(&[25, 50]);
            first.pop();
            l.blocks.push_back(first);
            let mut second = b"\n".to_vec();
            second.extend_from_slice(&block(&[75, 100]));
            l.blocks.push_back(second);
        });
        d.open().unwrap();
        let packets = capture(&mut d);
        assert_eq!(failure(&packets), None);
        assert_eq!(analog(&packets), vec![10.0, 20.0, 30.0, 40.0]);
    });
}

/// But where nothing was left behind, a line end ahead of a header is no block.
#[test]
fn a_line_end_ahead_of_a_header_is_malformed_otherwise() {
    let (mut d, _) = device(|l| {
        let mut second = b"\n".to_vec();
        second.extend_from_slice(&block(&[1; 16]));
        l.blocks.push_back(second);
    });
    d.open().unwrap();
    assert_eq!(
        failure(&capture(&mut d)).as_deref(),
        Some("device error: Encountered malformed block data header.")
    );
}

/// `:WAVEFORM:SEND?` is answered at a slow pace: a byte every twelve reads, so the whole
/// takes longer than the bound, which each byte starts over. Each gap stays well under
/// it, for a host stopped 400 ms between two reads.
#[test]
fn a_slow_block_at_a_real_pace_keeps_the_bound_off() {
    bounded(|| {
        let (mut d, line) = device_on("710110", Carrier::TcpRaw, |l| {
            l.samples = 64;
            l.piece = Some((1, 12));
        });
        d.bounds.first = Duration::from_millis(500);
        d.bounds.warn = Duration::from_millis(500);
        d.open().unwrap();
        forget_writes(&line);
        let packets = capture(&mut d);
        assert_eq!(failure(&packets), None);
        assert_eq!(analog(&packets).len(), 64);
    });
}

#[test]
fn bytes_that_flow_for_ever_end_a_capture() {
    bounded(|| {
        let (mut d, line) = device_on("710110", Carrier::TcpRaw, |l| {
            l.samples = 16;
            // Full reads say more is to come, each time.
            let mut flood = b"X".to_vec();
            flood.resize(4096, b'X');
            l.blocks.push_back(flood);
        });
        d.open().unwrap();
        for _ in 0..600 {
            line.lock().unwrap().coming.push_back((0, vec![b'X'; 4096]));
        }
        let packets = capture(&mut d);
        assert_eq!(
            failure(&packets).as_deref(),
            Some("device error: The scope sends more than a waveform holds.")
        );
    });
}

#[test]
fn a_request_that_cannot_be_sent_fails_the_capture() {
    for (blocked, packets_seen) in [
        (":WAVEFORM:FORMAT BYTE", vec!["header", "failed", "end"]),
        (":WAVEFORM:SEND? 1", vec!["header", "failed", "end"]),
    ] {
        let (mut d, line) = device(|_| {});
        d.open().unwrap();
        line.lock().unwrap().refuse.push(blocked.to_owned());
        let packets = capture(&mut d);
        assert_eq!(failure(&packets).as_deref(), Some("device error: the line said no"));
        assert_eq!(kinds(&packets), packets_seen, "{blocked}");
    }
}

#[test]
fn a_failed_second_request_keeps_the_first_channels_samples() {
    let (mut d, line) = device(|l| {
        l.chans_on = vec![1, 2];
        l.samples = 1;
    });
    d.open().unwrap();
    line.lock().unwrap().refuse.push(":WAVEFORM:TRACE 2".to_owned());
    let packets = capture(&mut d);
    assert_eq!(kinds(&packets), ["header", "begin", "analog", "failed", "end"]);
}

/// As C's source tries again, for what a retry can mend; it is a new request on USBTMC.
#[test]
fn a_failed_read_begin_is_tried_again_within_the_bound() {
    bounded(|| {
        let (mut d, line) = device(|l| l.samples = 1);
        d.open().unwrap();
        {
            let mut l = line.lock().unwrap();
            l.begin_fails.push_back(Error::Timeout);
            l.begin_fails.push_back(Error::device("USBTMC invalid bulk in header."));
        }
        let packets = capture(&mut d);
        assert_eq!(failure(&packets), None);
        assert_eq!(analog(&packets), vec![0.0]);
    });
    bounded(|| {
        let (mut d, line) = device(|_| {});
        d.bounds.first = Duration::from_millis(20);
        d.open().unwrap();
        {
            let mut l = line.lock().unwrap();
            for _ in 0..100_000 {
                l.begin_fails.push_back(Error::Timeout);
            }
        }
        let packets = capture(&mut d);
        assert_eq!(
            failure(&packets).as_deref(),
            Some("device error: No answer from the scope in 0.02 s.")
        );
    });
}

/// C tries for ever on a device that left.
#[test]
fn a_gone_device_ends_a_capture_at_once() {
    bounded(|| {
        let (mut d, line) = device(|_| {});
        d.open().unwrap();
        // The request goes, and the line goes behind it.
        line.lock().unwrap().begin_fails.push_back(Error::Disconnected);
        let packets = capture(&mut d);
        assert_eq!(failure(&packets).as_deref(), Some("device disconnected"));
        let (mut d, line) = device(|_| {});
        d.open().unwrap();
        line.lock().unwrap().gone = true;
        let packets = capture(&mut d);
        assert_eq!(failure(&packets).as_deref(), Some("device disconnected"));
    });
}

/// A stop between reads of a silent line, with the bounds far off.
#[test]
fn a_stop_ends_a_silent_capture() {
    bounded(|| {
        let (mut d, line) = device(|l| l.mute.push(":WAVEFORM:SEND? 1".to_owned()));
        d.open().unwrap();
        forget_writes(&line);
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        while !writes(&line).iter().any(|c| c == ":WAVEFORM:SEND? 1") {
            assert!(Instant::now() < deadline, "no request");
            std::thread::yield_now();
        }
        d.stop().unwrap();
        let packets = drain(&rx);
        assert_eq!(failure(&packets), None, "a stop is no failure");
        assert!(matches!(packets.last(), Some(Packet::End)));
        // The instrument is back.
        d.with_scpi(|_| Ok(())).unwrap();
    });
}

/// A stop inside an answer: the bytes keep to their pace, and it looks between reads.
#[test]
fn a_stop_lands_between_reads_of_an_answer() {
    bounded(|| {
        let (mut d, line) = device_on("710110", Carrier::TcpRaw, |l| {
            l.samples = 8;
            l.piece = Some((1, 100_000));
        });
        d.open().unwrap();
        let begun = line.lock().unwrap().reads;
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        while line.lock().unwrap().reads < begun + 5 {
            assert!(Instant::now() < deadline, "no reads");
            std::thread::yield_now();
        }
        d.stop().unwrap();
        let reads = line.lock().unwrap().reads;
        std::thread::sleep(Duration::from_millis(5));
        assert_eq!(line.lock().unwrap().reads, reads, "no read after the stop's join");
        assert_eq!(kinds(&drain(&rx)).last(), Some(&"end"));
    });
}

/// A queue no one reads must not hold a close.
#[test]
fn a_close_with_a_full_queue_returns() {
    bounded(|| {
        let (mut d, line) = device(|l| {
            l.chans_on = vec![1, 2, 3, 4];
            l.samples = 1;
        });
        d.open().unwrap();
        forget_writes(&line);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The header fills the queue, and the frame's begin waits on it after the first
        // answer, so the worker can't get to the second channel.
        let deadline = Instant::now() + BOUND;
        while !writes(&line).iter().any(|c| c == ":WAVEFORM:SEND? 1") {
            assert!(Instant::now() < deadline, "no request");
            std::thread::yield_now();
        }
        d.close().unwrap();
        assert_eq!(line.lock().unwrap().closes, 1);
        let sent = writes(&line).len();
        std::thread::sleep(Duration::from_millis(5));
        assert_eq!(writes(&line).len(), sent, "no writes after a close");
        assert_eq!(sent, 6, "one request");
        drop(rx);
    });
}

/// A line where a request can't get a start of an answer, with the bound far off.
#[test]
fn a_stop_ends_a_capture_that_can_begin_no_answer() {
    bounded(|| {
        let (mut d, line) = device(|_| {});
        d.open().unwrap();
        {
            let mut l = line.lock().unwrap();
            for _ in 0..100_000 {
                l.begin_fails.push_back(Error::Timeout);
            }
        }
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        while line.lock().unwrap().begin_fails.len() > 99_990 {
            assert!(Instant::now() < deadline, "no read_begin");
            std::thread::yield_now();
        }
        d.stop().unwrap();
        assert_eq!(failure(&drain(&rx)), None);
    });
}

/// The second channel's samples, with the queue full of the first's.
#[test]
fn a_close_returns_with_samples_waiting_on_the_queue() {
    bounded(|| {
        let (mut d, line) = device(|l| {
            l.chans_on = vec![1, 2];
            l.samples = 1;
        });
        d.open().unwrap();
        forget_writes(&line);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(rx.recv_timeout(BOUND), Ok((_, Packet::Header { .. }))));
        assert!(matches!(rx.recv_timeout(BOUND), Ok((_, Packet::FrameBegin))));
        let deadline = Instant::now() + BOUND;
        while writes(&line).iter().filter(|c| *c == ":WAVEFORM:SEND? 1").count() < 2 {
            assert!(Instant::now() < deadline, "no second request");
            std::thread::yield_now();
        }
        std::thread::sleep(Duration::from_millis(20));
        d.close().unwrap();
        drop(rx);
    });
}

#[test]
fn a_close_returns_with_logic_waiting_on_the_queue() {
    bounded(|| {
        let (mut d, line) = device(|l| {
            l.chans_on = vec![];
            l.bits_on = vec![1];
            l.logic_on = true;
            l.samples = 1;
        });
        d.open().unwrap();
        forget_writes(&line);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(rx.recv_timeout(BOUND), Ok((_, Packet::Header { .. }))));
        // The frame's begin stays in the queue, and its logic waits behind it.
        let deadline = Instant::now() + BOUND;
        while !writes(&line).iter().any(|c| c == ":WAVEFORM:SEND? 1") {
            assert!(Instant::now() < deadline, "no request");
            std::thread::yield_now();
        }
        std::thread::sleep(Duration::from_millis(20));
        d.close().unwrap();
        drop(rx);
    });
}

/// A client that keeps a sample waiting long past the bound isn't a silent scope: the
/// silence runs from the request that follows.
#[test]
fn a_slow_client_is_no_silent_scope() {
    bounded(|| {
        let (mut d, _) = device(|l| {
            l.chans_on = vec![1, 2];
            l.samples = 1;
            l.block_waits.push_back(0);
            l.block_waits.push_back(3);
        });
        d.bounds.later = Duration::from_secs(2);
        d.open().unwrap();
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(rx.recv_timeout(BOUND), Ok((_, Packet::Header { .. }))));
        // The frame's begin stays, and the first samples wait behind it.
        std::thread::sleep(Duration::from_millis(2200));
        let packets = drain(&rx);
        assert_eq!(failure(&packets), None);
        assert_eq!(analog(&packets).len(), 2);
    });
}

/// With its end stuck on the old queue, a start must not hang; the old sink goes.
#[test]
fn a_second_capture_starts_over_a_stuck_one() {
    bounded(|| {
        let (mut d, line) = device(|l| l.samples = 1);
        d.open().unwrap();
        let (tx, old) = sync_channel(1);
        let held = PacketSink::new(tx, 0);
        d.start(held).unwrap();
        forget_writes(&line);
        let packets = capture(&mut d);
        assert_eq!(failure(&packets), None);
        assert_eq!(analog(&packets), vec![0.0]);
        // The old worker is gone with its sink.
        while old.try_recv().is_ok() {}
        assert!(matches!(old.try_recv(), Err(std::sync::mpsc::TryRecvError::Disconnected)));
    });
}

#[test]
fn a_capture_after_a_capture_reads_again() {
    let (mut d, line) = device(|l| l.samples = 1);
    d.open().unwrap();
    assert_eq!(analog(&capture(&mut d)), vec![0.0]);
    forget_writes(&line);
    assert_eq!(analog(&capture(&mut d)), vec![0.0]);
    assert_eq!(writes(&line).len(), 6);
}

#[test]
fn a_setting_during_a_capture_is_busy() {
    bounded(|| {
        let (mut d, _) = device(|l| l.mute.push(":WAVEFORM:SEND? 1".to_owned()));
        d.open().unwrap();
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        while d.scpi.lock().unwrap().is_some() {
            assert!(Instant::now() < deadline, "not taken");
            std::thread::yield_now();
        }
        assert!(matches!(
            d.set(&Key::TriggerSlope, &Value::Str("f".to_owned()), None),
            Err(Error::Busy)
        ));
        assert!(matches!(d.enable_channel(0, false), Err(Error::Busy)));
        assert!(d.channels()[0].enabled, "as it was");
        d.stop().unwrap();
        drain(&rx);
    });
}

#[test]
fn opening_twice_and_closing_twice() {
    let (mut d, line) = device(|_| {});
    d.open().unwrap();
    forget_writes(&line);
    d.open().unwrap();
    assert_eq!(writes(&line), Vec::<String>::new(), "an open device is left alone");
    d.close().unwrap();
    d.close().unwrap();
    assert_eq!(line.lock().unwrap().closes, 1);
    d.open().unwrap();
    assert_eq!(line.lock().unwrap().reopens, 2, "an open opens again");
}

/// With the device closed there is no wire, and an open reads over what was asked.
#[test]
fn a_channel_of_a_closed_device_is_only_noted() {
    let (mut d, line) = device(|l| l.chans_on = vec![1]);
    d.enable_channel(1, true).unwrap();
    d.enable_channel(0, false).unwrap();
    assert_eq!(writes(&line), Vec::<String>::new());
    d.open().unwrap();
    assert!(d.channels()[0].enabled);
    assert!(!d.channels()[1].enabled);
}

#[test]
fn the_options_are_cs() {
    let (d, _) = device(|_| {});
    let keys: Vec<Key> = d.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(
        keys,
        [
            Key::LimitFrames,
            Key::Samplerate,
            Key::Timebase,
            Key::NumHdiv,
            Key::HorizTriggerpos,
            Key::TriggerSource,
            Key::TriggerSlope,
        ]
    );
    let keys: Vec<Key> = d.options(Some("1")).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(keys, [Key::NumVdiv, Key::Vdiv, Key::Coupling]);
    assert_eq!(d.options(Some("POD0")).unwrap().len(), 0);
    d.options(Some("nope")).unwrap_err();
    d.get(&Key::LimitFrames, None).unwrap_err();
    d.get(&Key::Vdiv, None).unwrap_err();
    d.get(&Key::Vdiv, Some("POD0")).unwrap_err();
    d.get(&Key::NumVdiv, None).unwrap_err();
    d.get(&Key::Coupling, None).unwrap_err();
    assert!(
        matches!(d.list(&Key::Timebase, None).unwrap(), ValueList::Rational(v) if v.len() == 36)
    );
    assert!(
        matches!(d.list(&Key::Vdiv, Some("1")).unwrap(), ValueList::Rational(v) if v.len() == 17)
    );
    d.list(&Key::Vdiv, None).unwrap_err();
    d.list(&Key::Coupling, None).unwrap_err();
    d.list(&Key::Samplerate, None).unwrap_err();
    assert!(
        matches!(d.list(&Key::Coupling, Some("1")).unwrap(), ValueList::Str(v) if v.len() == 4)
    );
    assert!(
        matches!(d.list(&Key::TriggerSource, None).unwrap(), ValueList::Str(v) if v.len() == 14)
    );
    assert!(
        matches!(d.list(&Key::TriggerSlope, None).unwrap(), ValueList::Str(v) if v == ["r", "f"])
    );
    let (d, _) = device_on("710105", Carrier::Usbtmc, |_| {});
    assert!(
        matches!(d.list(&Key::TriggerSource, None).unwrap(), ValueList::Str(v) if v.len() == 4)
    );
    assert_eq!(d.channels().len(), 2);
    assert_eq!(d.channel_groups().len(), 2);
    assert_eq!(d.info().model, "DLM2022");
    assert_eq!(d.info().vendor, "Yokogawa");
    assert_eq!(d.info().conn, None);
}

/// What the probe takes, on a line that answers `*IDN?` alone.
#[test]
fn a_probe_takes_only_a_yokogawa_it_knows() {
    let probed = |idn: &[u8], refuse: bool| {
        let line = Arc::new(Mutex::new(Line::default()));
        {
            let mut l = line.lock().unwrap();
            l.say.insert("*IDN?".to_owned(), idn.to_vec());
            if refuse {
                l.refuse.push(":COMMUNICATE:HEADER OFF".to_owned());
            }
        }
        let fake = Fake { line: Arc::clone(&line), carrier: Carrier::Usbtmc };
        let mut scpi = Scpi::new(Box::new(fake));
        scpi.read_timeout = Duration::from_millis(1);
        (probe(scpi).map(|d| d.info().model.clone()), writes(&line), line)
    };
    let (model, sent, line) = probed(b"YOKOGAWA,710130,S1,F2\n", false);
    assert_eq!(model.as_deref(), Some("DLM2054"));
    assert_eq!(sent, ["*IDN?", ":COMMUNICATE:HEADER OFF"]);
    assert_eq!(line.lock().unwrap().closes, 1);
    let (model, sent, _) = probed(b"Yokogawa,710130,S1,F2\n", false);
    assert_eq!(model, None, "an exact match, as strcmp");
    assert_eq!(sent, ["*IDN?"], "nothing more to it");
    let (model, sent, _) = probed(b"YOKOGAWA,710140,S1,F2\n", false);
    assert_eq!(model, None);
    assert_eq!(sent, ["*IDN?"]);
    assert_eq!(probed(b"short\n", false).0, None);
    assert_eq!(probed(b"YOKOGAWA,710130,S1,F2\n", true).0, None);
}

/// Every single bit flip and lost byte of a frame's framing, through a capture.
///
/// The format has no checksum: any flip in a sample is a wrong sample, and no reader can
/// know. What is counted is what the framing lets through: of the flips in the header
/// and the line end, none gives a wrong value. Of the lost bytes, any byte of the data
/// gives a block that looks whole but for its line end, whose last sample is the line
/// end and whose others are shifted: 8 of 19 here. Only a warning says so.
#[test]
fn flips_and_lost_bytes_of_a_frame() {
    bounded(|| {
        // Value classes: all zero, the extremes, a line end, and digits, which a header
        // whose count of digits flips reads as its own.
        for data in [
            vec![0u8; 8],
            vec![0x7f, 0x80, 0xff, 0x01, 0x7f, 0x80, 0xff, 0x01],
            vec![b'\n'; 8],
            vec![b'0'; 8],
            vec![b'9', b'#', b'8', b'0', b'0', b'0', b'0', b'0'],
        ] {
            let frame = block(&data);
            let good: Vec<f32> = data.iter().map(|b| protocol::volts(*b, 5.0, 0.0)).collect();
            let run = |raw: Vec<u8>| {
                let (mut d, _) = device(|l| {
                    l.samples = 8;
                    l.blocks.push_back(raw);
                    l.blocks.push_back(block(&[]));
                });
                d.bounds.empty = Duration::ZERO;
                d.open().unwrap();
                let packets = capture(&mut d);
                (failure(&packets), analog(&packets))
            };
            assert_eq!(run(frame.clone()), (None, good.clone()), "as it is");
            let framing: Vec<usize> = (0..10).chain([frame.len() - 1]).collect();
            let mut wrong = 0;
            let mut refused = 0;
            for at in framing {
                for bit in 0..8 {
                    let mut raw = frame.clone();
                    raw[at] ^= 1 << bit;
                    let (failed, samples) = run(raw);
                    if failed.is_some() {
                        refused += 1;
                    } else if samples != good {
                        wrong += 1;
                    }
                }
            }
            assert_eq!(wrong, 0, "flips over {data:?}");
            assert!(refused > 60, "{refused} of 88 over {data:?}");
            let mut wrong_lost = 0;
            for at in 0..frame.len() {
                let mut raw = frame.clone();
                raw.remove(at);
                let (failed, samples) = run(raw);
                if failed.is_none() && samples != good {
                    wrong_lost += 1;
                    assert!((10..18).contains(&at), "a lost byte of the framing, at {at}");
                }
            }
            // A line end's own value in the last place hides the loss of one more.
            let last_is_lf = data.last() == Some(&b'\n');
            assert_eq!(wrong_lost, if last_is_lf { 0 } else { 8 }, "lost bytes over {data:?}");
        }
    });
}

#[test]
fn a_header_that_says_more_than_came_is_cut_short() {
    let (mut d, _) = device(|l| {
        let mut more = block(&[1; 16]);
        more.splice(2..10, b"00000032".iter().copied());
        l.blocks.push_back(more);
    });
    d.open().unwrap();
    let packets = capture(&mut d);
    assert_eq!(
        failure(&packets).as_deref(),
        Some("device error: Truncated waveform data packet received.")
    );
    assert_eq!(analog(&packets), Vec::<f32>::new());
}
