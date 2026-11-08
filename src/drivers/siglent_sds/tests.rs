// Counterpart in sigrok:
//   libsigrok/src/hardware/siglent-sds/api.c,
//   libsigrok/src/hardware/siglent-sds/protocol.c,
//   Copyright (C) 2018 mhooijboer <marchelh@gmail.com>,
//   Copyright (C) 2016 mhooijboer <marchelh@gmail.com>,
//   Copyright (C) 2012 Martin Ling <martin-git@earth.li>,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2013 Mathias Grimmberger <mgri@zaphod.sax.de>,
//   with commits by Uwe Hermann, marchelh, Gerhard Sittig, fenugrec
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake line: what an emulator that answers at once can't show.

use super::*;
use crate::scpi::ScpiTransport;
use crate::serial::WriteLog;
use std::collections::{HashMap, VecDeque};
use std::sync::mpsc::{RecvTimeoutError, sync_channel};

/// How long a test may take before it counts as hung.
const LIMIT: Duration = Duration::from_secs(60);
/// A read's timeout on the fake.
const READ: Duration = Duration::from_millis(3);

/// Part of an answer.
#[derive(Clone, Debug)]
enum Piece {
    Bytes(Vec<u8>),
    /// So many reads find nothing, as a line that is slow finds them.
    Gap(u32),
}

fn text(t: &str) -> Vec<Piece> {
    vec![Piece::Bytes(format!("{t}\n").into_bytes())]
}

/// What answers: `None` for a question no answer comes to.
type Answer = Option<Vec<Piece>>;

#[derive(Default)]
#[allow(clippy::struct_excessive_bools)]
struct Line {
    log: WriteLog,
    /// USBTMC's habit: a read past an answer's end says so, a begin with none waits.
    usb: bool,
    answers: VecDeque<Vec<Piece>>,
    current: VecDeque<Piece>,
    begun: bool,
    /// Answers in turn per question, and after them the scope's own.
    turns: HashMap<String, VecDeque<Answer>>,
    /// Writes that fail from here on, with what they begin with.
    refuse: Option<String>,
    closed: u32,
    depth: usize,
    series_e: bool,
    /// `:INR?` is a bare zero for ever.
    no_trigger: bool,
    /// Bytes given since the last send, and a release for a test to wait on.
    given: usize,
}

impl Line {
    fn sent(&self) -> Vec<String> {
        self.log.iter().map(|(_, b)| String::from_utf8_lossy(b).trim_end().to_owned()).collect()
    }

    /// The scope's own answer to a question.
    fn own(&self, c: &str) -> Answer {
        if !c.contains('?') {
            return None;
        }
        if let Some(rest) = c.strip_suffix(":WF? ALL") {
            let channel: u8 = rest.trim_start_matches('C').parse().unwrap_or(1);
            return Some(vec![Piece::Bytes(waveform(channel, self.depth))]);
        }
        if let Some(rest) = c.strip_suffix(":WF? DAT2") {
            if let Some(n) = rest.strip_prefix('C') {
                let channel: u8 = n.parse().unwrap_or(1);
                return Some(vec![Piece::Bytes(dat2(&codes(channel, self.depth)))]);
            }
            let channel: usize = rest.trim_start_matches('D').parse().unwrap_or(0);
            return Some(vec![Piece::Bytes(digital(channel, self.depth))]);
        }
        let t = match c {
            "DI:SW?" => "OFF",
            "TRMD?" => "STOP",
            ":TDIV?" => "5.00E-04",
            "TRSE?" => "EDGE,SR,C1,HT,OFF",
            ":INR?" => {
                if self.no_trigger {
                    "0"
                } else {
                    "8193"
                }
            }
            "SANU? C1" | "SANU? D0" => {
                if self.series_e {
                    return Some(text(&format!("{}E+00", self.depth)));
                }
                return Some(text(&format!("{}pts", self.depth)));
            }
            "FPAR?" => {
                let mut bytes = vec![0u8; 200];
                bytes[40] = 2;
                return Some(vec![Piece::Bytes(bytes)]);
            }
            _ if c.ends_with(":TRA?") => {
                if c.starts_with('D') {
                    "OFF"
                } else {
                    "ON"
                }
            }
            _ if c.ends_with(":ATTN?") => "1",
            _ if c.ends_with(":VDIV?") => "1.00E+00",
            _ if c.ends_with(":OFST?") || c.ends_with(":TRLV?") => "0.00E+00",
            _ if c.ends_with(":CPL?") => "D1M",
            _ if c.ends_with(":TRSL?") => "POS",
            _ => return None,
        };
        Some(text(t))
    }
}

/// A channel's codes: its number, then a count, so a shift or a cut shows.
fn codes(channel: u8, n: usize) -> Vec<u8> {
    (0..n).map(|i| channel.wrapping_mul(50).wrapping_add(i as u8)).collect()
}

fn waveform_of(samples: &[u8], array: usize) -> Vec<u8> {
    let mut out = format!("ALL,#9{:09}", 346 + samples.len()).into_bytes();
    let mut d = vec![0u8; 346];
    d[36..40].copy_from_slice(&346u32.to_le_bytes());
    d[60..64].copy_from_slice(&(array as u32).to_le_bytes());
    out.extend(d);
    out.extend_from_slice(samples);
    out.extend_from_slice(b"\n\n");
    out
}

/// `Cn:WF? DAT2`: the codes alone (p.264-265).
fn dat2(samples: &[u8]) -> Vec<u8> {
    let mut out = format!("DAT2,#9{:09}", samples.len()).into_bytes();
    out.extend_from_slice(samples);
    out.extend_from_slice(b"\n\n");
    out
}

fn waveform(channel: u8, n: usize) -> Vec<u8> {
    waveform_of(&codes(channel, n), n)
}

/// Bits of a period of its channel's number and 2.
fn digital(channel: usize, depth: usize) -> Vec<u8> {
    let bytes: Vec<u8> = (0..depth.div_ceil(8))
        .map(|b| {
            (0..8).fold(0u8, |acc, bit| {
                acc | (u8::from(((b * 8 + bit) / (channel + 2)).is_multiple_of(2)) << bit)
            })
        })
        .collect();
    // Points, a bit each (p.268).
    let mut out = format!("DAT2,#9{depth:09}").into_bytes();
    out.extend_from_slice(&bytes);
    out.extend_from_slice(b"\n\n");
    out
}

struct Fake {
    line: Arc<Mutex<Line>>,
}

impl ScpiTransport for Fake {
    #[allow(clippy::unnecessary_literal_bound)]
    fn resource(&self) -> &str {
        "fake"
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        let mut line = self.line.lock().unwrap();
        let command = String::from_utf8_lossy(data).trim_end().to_owned();
        if line.refuse.as_deref().is_some_and(|r| command.starts_with(r)) {
            return Err(Error::device("the fake refuses this write"));
        }
        line.log.push((Instant::now(), data.to_vec()));
        line.given = 0;
        let turn = line.turns.get_mut(&command).and_then(VecDeque::pop_front);
        let answer = match turn {
            Some(answer) => answer,
            None => line.own(&command),
        };
        if let Some(answer) = answer {
            line.answers.push_back(answer);
        }
        Ok(())
    }

    fn read_begin(&mut self) -> Result<()> {
        let mut line = self.line.lock().unwrap();
        line.begun = !line.current.is_empty();
        if line.current.is_empty() {
            match line.answers.pop_front() {
                Some(a) => {
                    line.current = a.into();
                    line.begun = true;
                }
                None if line.usb => {
                    drop(line);
                    std::thread::sleep(Duration::from_millis(1));
                    return Err(Error::Timeout);
                }
                None => {}
            }
        }
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<usize> {
        let mut line = self.line.lock().unwrap();
        if line.current.is_empty()
            && !line.usb
            && let Some(a) = line.answers.pop_front()
        {
            line.current = a.into();
            line.begun = true;
        }
        match line.current.pop_front() {
            None => {
                let usb = line.usb;
                drop(line);
                if usb {
                    return Err(Error::device("USBTMC: read past the end of the message"));
                }
                std::thread::sleep(Duration::from_millis(1));
                Ok(0)
            }
            Some(Piece::Gap(n)) => {
                if n > 1 {
                    line.current.push_front(Piece::Gap(n - 1));
                }
                drop(line);
                std::thread::sleep(Duration::from_millis(1));
                Ok(0)
            }
            Some(Piece::Bytes(mut bytes)) => {
                let n = bytes.len().min(buf.len());
                buf[..n].copy_from_slice(&bytes[..n]);
                if n < bytes.len() {
                    line.current.push_front(Piece::Bytes(bytes.split_off(n)));
                }
                line.given += n;
                Ok(n)
            }
        }
    }

    fn read_complete(&self) -> bool {
        let line = self.line.lock().unwrap();
        line.begun && line.current.is_empty()
    }

    fn close(&mut self) -> Result<()> {
        self.line.lock().unwrap().closed += 1;
        Ok(())
    }
}

/// A device of `name` on a fake line, open, with short waits.
fn device(name: &str, set: impl FnOnce(&mut Line)) -> (SiglentDevice, Arc<Mutex<Line>>) {
    let model = Model::find(name).unwrap();
    let line = Line {
        depth: 700,
        series_e: model.series.protocol == Protocol::Eseries,
        ..Line::default()
    };
    let line = Arc::new(Mutex::new(line));
    set(&mut line.lock().unwrap());
    let mut scpi = Scpi::new(Box::new(Fake { line: Arc::clone(&line) }));
    scpi.read_timeout = READ;
    let id = crate::scpi::HwInfo {
        manufacturer: "Siglent".to_owned(),
        model: name.to_owned(),
        serial_number: "S".to_owned(),
        firmware_version: "F".to_owned(),
    };
    let mut d = SiglentDevice::new(model, &id, scpi);
    d.timing = Timing {
        first: Duration::from_secs(30),
        later: Duration::from_secs(30),
        poll: Duration::from_millis(1),
        no_trigger: Duration::from_secs(30),
        sleeps: false,
    };
    d.open().unwrap();
    line.lock().unwrap().log = WriteLog::default();
    (d, line)
}

/// Runs `f` with a bound, so a stop that doesn't bite fails rather than hangs.
fn bounded<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    let (tx, rx) = sync_channel(1);
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    match rx.recv_timeout(LIMIT) {
        Ok(v) => v,
        Err(RecvTimeoutError::Timeout) => panic!("hung for {LIMIT:?}"),
        Err(RecvTimeoutError::Disconnected) => panic!("the test's thread panicked"),
    }
}

/// A capture to its end: packets, with a cap and a deadline for what never ends.
fn capture(mut d: SiglentDevice) -> (Vec<Packet>, SiglentDevice) {
    bounded(move || {
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let began = Instant::now();
        let mut out = Vec::new();
        while began.elapsed() < LIMIT && out.len() < 100_000 {
            match rx.recv_timeout(Duration::from_millis(100)) {
                Ok((_, Packet::End)) => {
                    out.push(Packet::End);
                    break;
                }
                Ok((_, p)) => out.push(p),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => break,
            }
        }
        d.stop().unwrap();
        (out, d)
    })
}

fn samples(packets: &[Packet]) -> Vec<(usize, f32)> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some(a),
            _ => None,
        })
        .flat_map(|a| a.samples.iter().map(|s| (a.channels[0], *s)))
        .collect()
}

fn failure(packets: &[Packet]) -> Option<String> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(why) => Some(why.clone()),
        _ => None,
    })
}

fn count(packets: &[Packet], f: impl Fn(&Packet) -> bool) -> usize {
    packets.iter().filter(|p| f(p)).count()
}

/// Only channel `n` on, of the analog ones; digital off.
fn only(d: &mut SiglentDevice, n: usize) {
    for pos in 0..d.channels.len() {
        d.enable_channel(pos, pos == n).unwrap();
    }
}

fn probe_on(answer: Vec<Piece>) -> (Option<SiglentDevice>, Arc<Mutex<Line>>) {
    let line = Arc::new(Mutex::new(Line::default()));
    line.lock().unwrap().turns.insert("*IDN?".to_owned(), VecDeque::from([Some(answer)]));
    let mut scpi = Scpi::new(Box::new(Fake { line: Arc::clone(&line) }));
    scpi.read_timeout = READ;
    (probe(scpi), line)
}

#[test]
fn a_probe_takes_a_model_by_name_alone_and_turns_headers_off() {
    let (d, line) = probe_on(text("Acme,SDS1202X,123,7.1"));
    let d = d.expect("a model of the table");
    assert_eq!(d.info.vendor, "Siglent", "whoever said it");
    assert_eq!(d.info.model, "SDS1202X");
    assert_eq!(d.info.serial, "123");
    assert_eq!(d.info.version, "7.1");
    assert_eq!(d.info.conn, None);
    let line = line.lock().unwrap();
    assert_eq!(line.sent(), ["*IDN?", "CHDR OFF"]);
    assert_eq!(line.closed, 1, "the probe's connection ends with it");
}

#[test]
fn a_probe_of_another_instrument_turns_nothing_off() {
    let (d, line) = probe_on(text("Siglent Technologies,SDM3055,123,7.1"));
    assert!(d.is_none());
    assert_eq!(line.lock().unwrap().sent(), ["*IDN?"]);
}

#[test]
fn a_silent_probe_asks_again_once_through_a_reopen() {
    let line = Arc::new(Mutex::new(Line::default()));
    line.lock()
        .unwrap()
        .turns
        .insert("*IDN?".to_owned(), VecDeque::from([None, Some(text("x,SDS1202X,1,2"))]));
    let mut scpi = Scpi::new(Box::new(Fake { line: Arc::clone(&line) }));
    scpi.read_timeout = READ;
    assert!(probe(scpi).is_some());
    let line = line.lock().unwrap();
    assert_eq!(line.sent(), ["*IDN?", "*IDN?", "CHDR OFF"]);
    assert_eq!(line.closed, 2);
}

#[test]
fn an_open_reads_in_cs_order() {
    let (mut d, line) = device("SDS1202X", |_| {});
    d.close().unwrap();
    line.lock().unwrap().log = WriteLog::default();
    d.open().unwrap();
    assert_eq!(
        line.lock().unwrap().sent(),
        [
            "C1:TRA?", "C2:TRA?", ":TDIV?", "C1:ATTN?", "C2:ATTN?", "C1:VDIV?", "C2:VDIV?",
            "C1:OFST?", "C2:OFST?", "C1:CPL?", "C2:CPL?", "TRSE?", "C1:TRSL?", "C1:TRLV?",
        ]
    );
}

#[test]
fn an_open_asks_the_analyzer_on_those_that_have_one() {
    let (d, line) = device("SDS1104X-E", |l| {
        l.turns.insert("DI:SW?".to_owned(), VecDeque::from([Some(text("ON"))]));
        for i in 0..16 {
            let on = if i % 2 == 0 { "ON" } else { "OFF" };
            l.turns.insert(format!("D{i}:TRA?"), VecDeque::from([Some(text(on))]));
        }
    });
    let _ = line;
    let on: Vec<&str> = d.channels.iter().filter(|c| c.enabled).map(|c| c.name.as_str()).collect();
    assert_eq!(on, ["CH1", "CH2", "CH3", "CH4", "D0", "D2", "D4", "D6", "D8", "D10", "D12", "D14"]);
    assert!(d.state.la_enabled);
    let (mut d, line) = device("SDS1104X-E", |_| {});
    assert!(!d.state.la_enabled);
    assert!(d.channels[4..].iter().all(|c| !c.enabled), "the analyzer is off");
    d.close().unwrap();
    line.lock().unwrap().log = WriteLog::default();
    d.open().unwrap();
    let sent = line.lock().unwrap().sent();
    assert!(sent.contains(&"DI:SW?".to_owned()));
    assert!(!sent.iter().any(|c| c.starts_with('D') && c.ends_with(":TRA?")), "{sent:?}");
}

#[test]
fn an_answer_that_is_no_number_fails_the_open() {
    let model = Model::find("SDS1202X").unwrap();
    for (question, answer) in [(":TDIV?", "5.00E-04S"), ("C1:VDIV?", "1.00E+00V"), ("C1:TRA?", "9")]
    {
        let line = Arc::new(Mutex::new(Line::default()));
        line.lock()
            .unwrap()
            .turns
            .insert(question.to_owned(), VecDeque::from([Some(text(answer))]));
        let mut scpi = Scpi::new(Box::new(Fake { line: Arc::clone(&line) }));
        scpi.read_timeout = READ;
        let mut d = SiglentDevice::new(model, &crate::scpi::HwInfo::default(), scpi);
        assert!(d.open().is_err(), "{question} {answer}");
    }
}

#[test]
fn a_source_that_is_no_channel_has_no_level_to_ask_for() {
    let (d, line) = device("SDS1202X", |l| {
        l.turns.insert("TRSE?".to_owned(), VecDeque::from([Some(text("EDGE,SR,EX,HT,OFF"))]));
    });
    assert_eq!(d.state.trigger_source, "EX");
    drop(d);
    let _ = line;
    let (mut d, line) = device("SDS1202X", |l| {
        l.turns.insert("TRSE?".to_owned(), VecDeque::from([Some(text("EDGE,SR,EX,HT,OFF"))]));
    });
    d.close().unwrap();
    line.lock().unwrap().log = WriteLog::default();
    line.lock()
        .unwrap()
        .turns
        .insert("TRSE?".to_owned(), VecDeque::from([Some(text("EDGE,SR,EX,HT,OFF"))]));
    d.open().unwrap();
    let sent = line.lock().unwrap().sent();
    assert_eq!(&sent[sent.len() - 2..], ["TRSE?", "EX:TRSL?"]);
    assert_eq!(d.get(&Key::TriggerSource, None).unwrap().as_str(), Some("EX"));
    assert_eq!(d.get(&Key::TriggerLevel, None).unwrap().as_f64(), Some(0.0));
}

#[test]
fn a_frame_has_all_its_samples_with_one_request_a_channel() {
    let (mut d, line) = device("SDS1202X", |_| {});
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    let (packets, d) = capture(d);
    let got = samples(&packets);
    assert_eq!(got.len(), 2 * 2 * 700, "C has 337 of each, and no end");
    let want = |channel: u8, i: usize| protocol::volts(codes(channel, 700)[i], 1.0, 0.0);
    assert_eq!(got[0], (0, want(1, 0)), "the first of them, which C skips");
    assert_eq!(got[699], (0, want(1, 699)), "the last, which it doesn't reach");
    assert_eq!(got[700], (1, want(2, 0)));
    assert_eq!(count(&packets, |p| matches!(p, Packet::Header { .. })), 1);
    assert_eq!(count(&packets, |p| matches!(p, Packet::FrameBegin)), 2);
    assert_eq!(count(&packets, |p| matches!(p, Packet::FrameEnd)), 2);
    assert_eq!(failure(&packets), None);
    assert!(matches!(packets.last(), Some(Packet::End)));
    assert!(matches!(packets[0], Packet::Header { .. }));
    assert_eq!(
        line.lock().unwrap().sent(),
        [
            "WFSU SP,0,TYPE,1",
            "ACQW SAMPLING",
            "SANU? C1",
            ":TDIV?",
            "ARM",
            ":INR?",
            "C1:WF? ALL",
            "C2:WF? ALL",
            "ARM",
            ":INR?",
            "C1:WF? ALL",
            "C2:WF? ALL",
        ]
    );
    drop(d);
}

#[test]
fn an_spo_model_sets_up_ahead_of_its_samplerate() {
    let (mut d, line) = device("SDS1202X", |_| {});
    only(&mut d, 0);
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    assert_eq!(
        line.lock().unwrap().sent(),
        [
            "C2:TRA OFF",
            "WFSU SP,0,TYPE,1",
            "ACQW SAMPLING",
            "SANU? C1",
            ":TDIV?",
            "ARM",
            ":INR?",
            "C1:WF? ALL",
        ]
    );
}

#[test]
fn averaging_says_how_many() {
    let (mut d, line) = device("SDS1202X", |_| {});
    d.set(&Key::Averaging, &Value::Bool(true), None).unwrap();
    d.set(&Key::AvgSamples, &Value::U64(16), None).unwrap();
    let (_, _d) = capture(d);
    assert!(line.lock().unwrap().sent().contains(&"ACQW AVERAGE,16".to_owned()));
}

#[test]
fn a_non_spo_model_has_no_arm_and_waits_for_inr() {
    let (mut d, line) = device("SDS1052DL+", |l| {
        l.turns.insert(
            ":INR?".to_owned(),
            VecDeque::from([Some(text("0")), Some(text("0")), Some(text("1"))]),
        );
    });
    only(&mut d, 1);
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    assert_eq!(
        line.lock().unwrap().sent(),
        [
            "C1:TRA OFF",
            "WFSU SP,0,TYPE,1",
            "ACQW SAMPLING",
            "SANU? C1",
            ":TDIV?",
            ":INR?",
            ":INR?",
            ":INR?",
            "C2:WF? ALL",
        ]
    );
}

#[test]
fn a_scope_armed_and_waiting_is_polled_until_it_says_so() {
    let (d, line) = device("SDS1202X", |l| {
        l.turns.insert(
            ":INR?".to_owned(),
            VecDeque::from([Some(text("8192")), Some(text("0")), Some(text("1"))]),
        );
    });
    let (packets, _d) = capture(d);
    assert_eq!(samples(&packets).len(), 1400);
    let sent = line.lock().unwrap().sent();
    assert_eq!(sent.iter().filter(|c| *c == ":INR?").count(), 3);
    assert_eq!(sent.iter().filter(|c| *c == "ARM").count(), 1);
}

/// p.175: 0 is single mode with no signal, 8192 a trigger ready with no waveform yet;
/// only bit 0 says one is there. C fails the start on anything but 8192 and 8193 after
/// `ARM`, and in its wait takes any value but 0.
#[test]
fn only_bit_0_of_inr_is_a_waveform() {
    for list in [["0", "1"], ["8192", "1"], ["8194", "8193"], ["2", "8193"]] {
        let (d, line) = device("SDS1202X", |l| {
            let turns = [Some(text(list[0])), Some(text("8192")), Some(text(list[1]))];
            l.turns.insert(":INR?".to_owned(), VecDeque::from(turns));
        });
        let (packets, _d) = capture(d);
        assert_eq!(failure(&packets), None, "{list:?}");
        assert_eq!(samples(&packets).len(), 1400, "{list:?}");
        let sent = line.lock().unwrap().sent();
        let at = sent.iter().position(|c| c.contains("WF?")).unwrap();
        assert_eq!(sent[..at].iter().filter(|c| *c == ":INR?").count(), 3, "{list:?}");
    }
}

/// An E series on the screen as pull request 247 has it: a single shot, `TRMD?` until
/// `STOP`, the settings read again, and `DAT2`.
#[test]
fn an_e_series_takes_a_single_shot_and_reads_dat2() {
    let (d, line) = device("SDS1202X-E", |l| {
        let polls = [Some(text("SINGLE")), Some(text("SINGLE")), Some(text("STOP"))];
        l.turns.insert("TRMD?".to_owned(), VecDeque::from(polls));
    });
    // The scale has moved on the front panel since the open.
    line.lock()
        .unwrap()
        .turns
        .insert("C1:VDIV?".to_owned(), VecDeque::from([Some(text("5.00E-01"))]));
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    let got = samples(&packets);
    assert_eq!(got.len(), 1400);
    assert_eq!(got[0], (0, protocol::volts(codes(1, 700)[0], 0.5, 0.0)), "read after the stop");
    assert_eq!(got[700], (1, protocol::volts(codes(2, 700)[0], 1.0, 0.0)));
    assert_eq!(
        line.lock().unwrap().sent(),
        [
            "SANU? C1",
            ":TDIV?",
            "TRMD SINGLE",
            "TRMD?",
            "TRMD?",
            "TRMD?",
            "C1:VDIV?",
            "C2:VDIV?",
            "C1:OFST?",
            "C2:OFST?",
            "SANU? C1",
            ":TDIV?",
            "C1:WF? DAT2",
            "C2:WF? DAT2",
        ]
    );
}

/// Only `STOP` ends the wait, as in pull request 247: `AUTO` or `NORM` is no shot done.
#[test]
fn a_single_shot_waits_for_stop_and_nothing_else() {
    let (d, line) = device("SDS1202X-E", |l| {
        let polls = [Some(text("AUTO")), Some(text("NORM")), Some(text("STOP"))];
        l.turns.insert("TRMD?".to_owned(), VecDeque::from(polls));
    });
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    assert_eq!(line.lock().unwrap().sent().iter().filter(|c| *c == "TRMD?").count(), 3);
}

/// Points that aren't a whole number of bytes: the last byte's spare bits are no samples.
#[test]
fn digital_points_need_not_fill_their_last_byte() {
    let (mut d, _) = device("SDS1104X-E", |l| {
        l.depth = 20;
        l.turns.insert("DI:SW?".to_owned(), VecDeque::from([Some(text("ON"))]));
    });
    for pos in 0..d.channels.len() {
        let name = d.channels[pos].name.clone();
        d.enable_channel(pos, name == "D0").unwrap();
    }
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    let logic: usize = packets
        .iter()
        .filter_map(|p| match p {
            Packet::Logic(l) => Some(l.data.len()),
            _ => None,
        })
        .sum();
    assert_eq!(logic, 40, "20 points of 2 bytes, from 3 bytes of bits");
}

#[test]
fn an_e_series_shot_that_never_stops_ends_only_by_a_stop() {
    let (mut d, line) = device("SDS1202X-E", |l| {
        l.turns.insert(
            "TRMD?".to_owned(),
            std::iter::repeat_n(Some(text("SINGLE")), 100_000).collect(),
        );
    });
    d.timing.first = Duration::from_millis(5);
    let line2 = Arc::clone(&line);
    bounded(move || {
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        while line2.lock().unwrap().sent().iter().filter(|c| *c == "TRMD?").count() < 20 {
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        let packets: Vec<_> = rx.try_iter().map(|(_, p)| p).collect();
        assert_eq!(failure(&packets), None, "SINGLE is an answer, not silence");
        assert!(matches!(packets.last(), Some(Packet::End)));
    });
}

#[test]
fn an_e_series_shot_takes_each_frame_again() {
    let (mut d, line) = device("SDS1202X-E", |_| {});
    only(&mut d, 0);
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    let (packets, _d) = capture(d);
    assert_eq!(count(&packets, |p| matches!(p, Packet::FrameEnd)), 2);
    let sent = line.lock().unwrap().sent();
    assert_eq!(sent.iter().filter(|c| *c == "TRMD SINGLE").count(), 2);
    assert!(!sent.iter().any(|c| c == "ARM" || c.ends_with("WF? ALL")));
}

/// A head shorter than the first read leaves samples in it: here a bare `#9` with no
/// `DAT2,`, as no scope is shown to send but the parser takes.
#[test]
fn a_dat2_block_cut_anywhere_comes_whole() {
    let block = dat2(&codes(1, 50));
    let mut pieces = Vec::new();
    for b in block {
        pieces.push(Piece::Bytes(vec![b]));
    }
    let short = vec![Piece::Bytes(dat2(&codes(2, 3))[5..].to_vec())];
    let (d, _) = device("SDS1202X-E", move |l| {
        l.turns.insert("C1:WF? DAT2".to_owned(), VecDeque::from([Some(pieces)]));
        l.turns.insert("C2:WF? DAT2".to_owned(), VecDeque::from([Some(short)]));
    });
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    let got: Vec<(usize, f32)> = samples(&packets);
    assert_eq!(got.len(), 53);
    assert_eq!(got[50], (1, protocol::volts(codes(2, 3)[0], 1.0, 0.0)), "read with the head");
}

#[test]
fn an_inr_that_is_no_number_is_no_trigger() {
    let (d, _) = device("SDS1202X", |l| {
        l.turns.insert(":INR?".to_owned(), VecDeque::from([Some(text("7.00E+05"))]));
    });
    let (packets, _d) = capture(d);
    let why = failure(&packets).expect("a failure, where C reads what its stack held");
    assert!(why.contains("No usable :INR? answer: '7.00E+05'"), "{why}");
    // In the wait as well.
    let (d, _) = device("SDS1202X", |l| {
        l.turns.insert(":INR?".to_owned(), VecDeque::from([Some(text("8192")), Some(text("OFF"))]));
    });
    let (packets, _d) = capture(d);
    assert!(failure(&packets).expect("a failure").contains("No usable :INR? answer: 'OFF'"));
}

#[test]
fn a_trigger_that_never_comes_ends_only_by_a_stop() {
    let (d, line) = device("SDS1202X", |l| {
        l.turns.insert(":INR?".to_owned(), VecDeque::from([Some(text("8192"))]));
        l.no_trigger = true;
    });
    let mut d = d;
    d.timing.first = Duration::from_millis(5);
    let line2 = Arc::clone(&line);
    bounded(move || {
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Many times the silence bound, answered, and it goes on.
        let began = Instant::now();
        loop {
            let polls = line2.lock().unwrap().sent().iter().filter(|c| *c == ":INR?").count();
            if polls >= 20 {
                break;
            }
            assert!(began.elapsed() < LIMIT, "it stopped asking");
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        let packets: Vec<_> = rx.try_iter().map(|(_, p)| p).collect();
        assert_eq!(failure(&packets), None, "an answer of 0 is no silence");
        assert!(matches!(packets.last(), Some(Packet::End)));
    });
}

#[test]
fn a_silent_scope_with_no_sample_fails_at_the_bound() {
    let (mut d, _) = device("SDS1202X", |l| {
        l.turns.insert("C1:WF? ALL".to_owned(), VecDeque::from([None]));
    });
    d.timing.first = Duration::from_millis(40);
    let began = Instant::now();
    let (packets, _d) = capture(d);
    let why = failure(&packets).expect("a failure");
    assert!(why.contains("No answer to C1:WF? ALL: the scope is silent"), "{why}");
    assert!(began.elapsed() >= Duration::from_millis(40), "too soon");
    assert_eq!(samples(&packets).len(), 0);
}

#[test]
fn a_silent_inr_fails_with_its_name() {
    let (mut d, _) = device("SDS1202X", |l| {
        l.turns.insert(":INR?".to_owned(), std::iter::repeat_n(None, 1000).collect());
    });
    d.timing.first = Duration::from_millis(40);
    let (packets, _d) = capture(d);
    let why = failure(&packets).expect("a failure");
    assert!(why.contains("No answer to :INR?: the scope is silent"), "{why}");
}

/// `:INR?` clears what it reads, so a question asked again after a read's timeout
/// would lose the 8193 that was on its way.
#[test]
fn a_late_inr_is_waited_for_and_not_asked_again() {
    let (d, line) = device("SDS1202X", |l| {
        let late = vec![Piece::Gap(20), Piece::Bytes(b"8193\n".to_vec())];
        l.turns.insert(":INR?".to_owned(), VecDeque::from([Some(late)]));
    });
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 1400);
    let sent = line.lock().unwrap().sent();
    assert_eq!(sent.iter().filter(|c| *c == ":INR?").count(), 1);
}

#[test]
fn a_late_answer_within_the_bound_makes_the_frame() {
    let (mut d, _) = device("SDS1202X", |l| {
        l.depth = 400;
        let mut late = vec![Piece::Gap(5)];
        late.push(Piece::Bytes(waveform(1, 400)));
        l.turns.insert("C1:WF? ALL".to_owned(), VecDeque::from([Some(late)]));
    });
    d.timing.first = Duration::from_secs(30);
    only(&mut d, 0);
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 400);
}

#[test]
fn after_a_sample_silence_warns_and_goes_on_to_the_long_limit() {
    // The first bound is far under what the gap takes: only the long limit ends it.
    let gap = Piece::Gap(u32::MAX);
    let (mut d, _) = device("SDS1202X", move |l| {
        l.depth = 400;
        let mut block = waveform(1, 400);
        block.truncate(15 + 346 + 100);
        l.turns.insert(
            "C1:WF? ALL".to_owned(),
            VecDeque::from([Some(vec![Piece::Bytes(block), gap])]),
        );
    });
    only(&mut d, 0);
    d.timing.first = Duration::from_millis(5);
    d.timing.later = Duration::from_millis(150);
    let began = Instant::now();
    let (packets, _d) = capture(d);
    let why = failure(&packets).expect("a failure at the long limit");
    assert!(why.contains("No answer to C1:WF? ALL"), "{why}");
    assert!(began.elapsed() >= Duration::from_millis(150), "the first bound ended it");
    assert_eq!(samples(&packets).len(), 100, "what came is kept");
}

#[test]
fn a_block_cut_at_a_messages_end_fails_with_what_came() {
    let (mut d, _) = device("SDS1202X", |l| {
        l.usb = true;
        let mut block = waveform(1, 400);
        block.truncate(15 + 346 + 39);
        l.turns.insert("C1:WF? ALL".to_owned(), VecDeque::from([Some(vec![Piece::Bytes(block)])]));
    });
    only(&mut d, 0);
    let (packets, _d) = capture(d);
    let why = failure(&packets).expect("a failure");
    assert!(why.contains("C1:WF? ALL was cut short"), "{why}");
    assert_eq!(samples(&packets).len(), 39);
    assert!(matches!(packets.last(), Some(Packet::End)));
}

#[test]
fn bytes_that_are_no_block_fail_at_once() {
    let (mut d, _) = device("SDS1202X", |l| {
        l.turns.insert("C1:WF? ALL".to_owned(), VecDeque::from([Some(text("7.00E+05"))]));
    });
    // Over any test's time if it took this for silence.
    d.timing.first = Duration::from_secs(3600);
    let (packets, _d) = capture(d);
    assert!(failure(&packets).expect("a failure").contains("No usable waveform"));
}

#[test]
fn a_descriptor_that_doesnt_fit_fails_and_a_length_that_lies_costs_samples() {
    let (mut d, _) = device("SDS1202X", |l| {
        l.turns.insert(
            "C1:WF? ALL".to_owned(),
            VecDeque::from([Some(vec![Piece::Bytes(waveform_of(&codes(1, 100), 5000))])]),
        );
    });
    only(&mut d, 0);
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 100, "no more than the block holds");

    let (mut d, _) = device("SDS1202X", |l| {
        l.turns.insert(
            "C1:WF? ALL".to_owned(),
            VecDeque::from([Some(vec![Piece::Bytes(waveform_of(&codes(1, 100), 60))])]),
        );
    });
    only(&mut d, 0);
    let (packets, _d) = capture(d);
    assert_eq!(samples(&packets).len(), 60, "the less of the two");
    assert_eq!(failure(&packets), None);

    let mut bad = waveform(1, 100);
    bad[15 + 36..15 + 40].copy_from_slice(&5000u32.to_le_bytes());
    let (mut d, _) = device("SDS1202X", move |l| {
        l.turns.insert("C1:WF? ALL".to_owned(), VecDeque::from([Some(vec![Piece::Bytes(bad)])]));
    });
    only(&mut d, 0);
    let (packets, _d) = capture(d);
    assert!(failure(&packets).expect("a failure").contains("a descriptor of 5000 bytes"));
}

/// At a real pace: the head, the descriptor and each sample its own piece behind a
/// gap of reads, the answer late, and the samples still whole and in order.
#[test]
fn a_block_at_a_slow_pace_comes_whole() {
    let block = waveform(1, 120);
    let mut pieces = vec![Piece::Gap(3)];
    for b in block {
        pieces.push(Piece::Bytes(vec![b]));
        pieces.push(Piece::Gap(1));
    }
    let (mut d, _) = device("SDS1202X", move |l| {
        l.turns.insert("C1:WF? ALL".to_owned(), VecDeque::from([Some(pieces)]));
    });
    only(&mut d, 0);
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    let want: Vec<f32> = codes(1, 120).iter().map(|c| protocol::volts(*c, 1.0, 0.0)).collect();
    let got: Vec<f32> = samples(&packets).iter().map(|s| s.1).collect();
    assert_eq!(got, want);
}

#[test]
fn a_prefix_of_any_length_takes_no_samples_with_it() {
    let mut block = b"#3408".to_vec();
    let mut desc = vec![0u8; 64];
    desc[36..40].copy_from_slice(&64u32.to_le_bytes());
    desc[60..64].copy_from_slice(&344u32.to_le_bytes());
    block.extend(desc);
    block.extend(codes(1, 344));
    // The first read of 15 runs past a head of 5.
    let (mut d, _) = device("SDS1202X", move |l| {
        l.turns.insert("C1:WF? ALL".to_owned(), VecDeque::from([Some(vec![Piece::Bytes(block)])]));
    });
    only(&mut d, 0);
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 344);
    assert_eq!(samples(&packets)[0].1, protocol::volts(codes(1, 1)[0], 1.0, 0.0));
}

#[test]
fn a_scale_and_an_offset_are_the_scopes() {
    let (mut d, _) = device("SDS1202X", |l| {
        l.turns.insert("C2:VDIV?".to_owned(), VecDeque::from([Some(text("5.00E-01"))]));
        l.turns.insert("C2:OFST?".to_owned(), VecDeque::from([Some(text("2.50E-01"))]));
    });
    only(&mut d, 1);
    let (packets, _d) = capture(d);
    assert_eq!(samples(&packets)[1], (1, protocol::volts(codes(2, 2)[1], 0.5, 0.25)));
    let digits = packets.iter().find_map(|p| match p {
        Packet::Analog(a) => Some(a.digits),
        _ => None,
    });
    assert_eq!(digits, Some(1));
}

#[test]
fn digital_channels_come_as_one_packet_of_16_bits() {
    let (mut d, line) = device("SDS1104X-E", |l| {
        l.depth = 40;
        l.turns.insert("DI:SW?".to_owned(), VecDeque::from([Some(text("ON"))]));
    });
    for pos in 0..d.channels.len() {
        let name = d.channels[pos].name.clone();
        d.enable_channel(pos, name == "D1" || name == "D9").unwrap();
    }
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    let bytes: Vec<&[u8]> = packets
        .iter()
        .filter_map(|p| match p {
            Packet::Logic(l) => {
                assert_eq!(l.unit_size, 2);
                Some(l.data.as_slice())
            }
            _ => None,
        })
        .collect();
    assert_eq!(bytes.len(), 1);
    assert_eq!(bytes[0].len(), 80);
    let mut want = vec![0u16; 40];
    protocol::merge_digital(&mut want, &digital(1, 40)[16..21], 1);
    protocol::merge_digital(&mut want, &digital(9, 40)[16..21], 9);
    let got: Vec<u16> = bytes[0].chunks(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
    assert_eq!(got, want);
    assert_ne!(got[0] & 0x0002, 0, "D1's first sample, which is no digit of a head");
    let sent = line.lock().unwrap().sent();
    assert!(sent.contains(&"D1:WF? DAT2".to_owned()));
    assert!(sent.contains(&"D9:WF? DAT2".to_owned()));
    assert!(!sent.contains(&"D0:WF? DAT2".to_owned()));
    assert!(sent.contains(&"SANU? D0".to_owned()));
    assert!(!sent.contains(&"DI:SW ON".to_owned()), "already on");
    // Every channel told, on or off.
    assert_eq!(sent.iter().filter(|c| c.ends_with(":TRA ON")).count(), 2);
    assert_eq!(sent.iter().filter(|c| c.starts_with('D') && c.ends_with(":TRA OFF")).count(), 14);
}

#[test]
fn digital_with_the_analyzer_off_turns_it_on_with_no_answer_owed() {
    let (mut d, line) = device("SDS1104X-E", |l| l.depth = 16);
    for pos in 0..d.channels.len() {
        let name = d.channels[pos].name.clone();
        d.enable_channel(pos, name == "D0").unwrap();
    }
    let (packets, d) = capture(d);
    assert_eq!(failure(&packets), None);
    let sent = line.lock().unwrap().sent();
    assert!(sent.contains(&"DI:SW ON".to_owned()));
    assert!(!sent.iter().any(|c| c == "DI:SW?"), "an answer no one reads shifts the rest");
    assert!(d.state.la_enabled);
}

#[test]
fn a_digital_answer_short_of_the_depth_keeps_what_it_has() {
    let (mut d, _) = device("SDS1104X-E", |l| {
        l.depth = 800;
        l.turns.insert("DI:SW?".to_owned(), VecDeque::from([Some(text("ON"))]));
        l.turns.insert(
            "D0:WF? DAT2".to_owned(),
            VecDeque::from([Some(vec![Piece::Bytes(digital(0, 40))])]),
        );
    });
    for pos in 0..d.channels.len() {
        let name = d.channels[pos].name.clone();
        d.enable_channel(pos, name == "D0").unwrap();
    }
    let (packets, _d) = capture(d);
    let logic: usize = packets
        .iter()
        .filter_map(|p| match p {
            Packet::Logic(l) => Some(l.data.len()),
            _ => None,
        })
        .sum();
    assert_eq!(logic, 80, "40 samples at 2 bytes, and no bytes from past the end");
}

#[test]
fn a_digital_stop_comes_between_frames_too() {
    let (mut d, _) = device("SDS1104X-E", |l| {
        l.depth = 16;
        l.turns.insert("DI:SW?".to_owned(), VecDeque::from([Some(text("ON"))]));
    });
    for pos in 0..d.channels.len() {
        let name = d.channels[pos].name.clone();
        d.enable_channel(pos, name == "D0").unwrap();
    }
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    let (packets, _d) = capture(d);
    assert_eq!(count(&packets, |p| matches!(p, Packet::Logic(_))), 2, "C stops at the first");
}

#[test]
fn the_analyzer_goes_off_once_when_nothing_uses_it() {
    let (mut d, line) = device("SDS1104X-E", |l| {
        l.turns.insert("DI:SW?".to_owned(), VecDeque::from([Some(text("ON"))]));
    });
    only(&mut d, 0);
    let (_, mut d) = capture(d);
    let sent = line.lock().unwrap().sent();
    assert!(sent.contains(&"DI:SW OFF".to_owned()));
    assert!(!sent.contains(&"SANU? D0".to_owned()), "it isn't on now");
    line.lock().unwrap().log = WriteLog::default();
    only(&mut d, 0);
    let (_, _d) = capture(d);
    assert!(!line.lock().unwrap().sent().contains(&"DI:SW OFF".to_owned()), "C says it each time");
}

#[test]
fn what_is_on_stays_known_from_one_capture_to_the_next() {
    let (mut d, line) = device("SDS1202X", |_| {});
    only(&mut d, 1);
    let (_, mut d) = capture(d);
    assert!(line.lock().unwrap().sent().contains(&"C1:TRA OFF".to_owned()));
    line.lock().unwrap().log = WriteLog::default();
    let (packets, _d) = {
        only(&mut d, 1);
        capture(d)
    };
    assert_eq!(samples(&packets).len(), 700, "a second capture on an open device");
    assert!(!line.lock().unwrap().sent().iter().any(|c| c.contains(":TRA ")));
}

#[test]
fn history_asks_for_its_count_and_its_frame() {
    let (mut d, line) = device("SDS1202X", |_| {});
    d.set(&Key::DataSource, &Value::Str("History".to_owned()), None).unwrap();
    d.set(&Key::LimitFrames, &Value::U64(0), None).unwrap();
    only(&mut d, 0);
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    assert_eq!(count(&packets, |p| matches!(p, Packet::FrameEnd)), 2, "what FPAR? counts");
    let sent = line.lock().unwrap().sent();
    let at = sent.iter().position(|c| c == "FPAR?").unwrap();
    assert_eq!(sent[at..], ["FPAR?", "FRAM 1", "C1:WF? ALL", "FPAR?", "FRAM 2", "C1:WF? ALL"]);
    assert!(!sent.contains(&"ARM".to_owned()));
}

#[test]
fn history_with_a_limit_past_its_count_says_so_and_goes_on() {
    let (mut d, line) = device("SDS1202X", |_| {});
    d.set(&Key::DataSource, &Value::Str("History".to_owned()), None).unwrap();
    d.set(&Key::LimitFrames, &Value::U64(3), None).unwrap();
    only(&mut d, 0);
    let (packets, _d) = capture(d);
    assert_eq!(count(&packets, |p| matches!(p, Packet::FrameEnd)), 3, "as C");
    assert!(line.lock().unwrap().sent().contains(&"FRAM 3".to_owned()));
}

#[test]
fn an_fpar_too_short_to_count_fails() {
    let (mut d, _) = device("SDS1202X", |l| {
        l.turns.insert("FPAR?".to_owned(), VecDeque::from([Some(text("short"))]));
    });
    d.set(&Key::DataSource, &Value::Str("History".to_owned()), None).unwrap();
    let (packets, _d) = capture(d);
    assert!(failure(&packets).expect("a failure").contains("FPAR? answered 6 bytes"));
}

#[test]
fn a_write_that_fails_ends_the_capture() {
    let (d, _) = device("SDS1202X", |l| l.refuse = Some("ARM".to_owned()));
    let (packets, _d) = capture(d);
    assert!(failure(&packets).expect("a failure").contains("the fake refuses this write"));
    assert!(matches!(packets.last(), Some(Packet::End)));
}

#[test]
fn a_failed_read_of_the_depth_leaves_the_capture_going() {
    let (d, _) = device("SDS1202X", |l| {
        l.turns.insert("SANU? C1".to_owned(), VecDeque::from([Some(text("junk"))]));
    });
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None, "C drops this result");
    assert_eq!(samples(&packets).len(), 1400);
}

#[test]
fn a_stop_in_a_block_ends_it_and_nothing_is_sent_after_a_close() {
    let block = waveform(1, 400);
    let (head, _) = block.split_at(15 + 346 + 50);
    let pieces = vec![Piece::Bytes(head.to_vec()), Piece::Gap(u32::MAX)];
    let (mut d, line) = device("SDS1202X", move |l| {
        l.turns.insert("C1:WF? ALL".to_owned(), VecDeque::from([Some(pieces)]));
    });
    only(&mut d, 0);
    let line2 = Arc::clone(&line);
    bounded(move || {
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The samples have been read, and the rest of the block isn't coming.
        let began = Instant::now();
        while rx.try_recv().is_err() || line2.lock().unwrap().given < 100 {
            assert!(began.elapsed() < LIMIT, "no bytes taken");
            std::thread::sleep(Duration::from_millis(1));
        }
        let writes = line2.lock().unwrap().log.len();
        d.close().unwrap();
        assert_eq!(line2.lock().unwrap().log.len(), writes, "no write after the close");
        assert_eq!(line2.lock().unwrap().closed, 1);
        let packets: Vec<_> = rx.try_iter().map(|(_, p)| p).collect();
        assert_eq!(failure(&packets), None, "a stop is no failure");
    });
}

#[test]
fn a_close_with_a_queue_no_one_reads_ends() {
    let (mut d, _) = device("SDS1202X", |_| {});
    d.set(&Key::LimitFrames, &Value::U64(0), None).unwrap();
    bounded(move || {
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The queue is full, the thread is held on it.
        let began = Instant::now();
        while rx.is_empty_for_test() {
            assert!(began.elapsed() < LIMIT);
            std::thread::sleep(Duration::from_millis(1));
        }
        d.close().unwrap();
        drop(rx);
    });
}

trait EmptyForTest {
    fn is_empty_for_test(&self) -> bool;
}

impl<T> EmptyForTest for std::sync::mpsc::Receiver<T> {
    /// There's no look at a queue, so a wait long enough for a sender to reach it.
    fn is_empty_for_test(&self) -> bool {
        std::thread::sleep(Duration::from_millis(20));
        false
    }
}

#[test]
fn a_join_at_a_close_takes_the_thread_with_it() {
    let (mut d, line) = device("SDS1202X", |l| {
        l.no_trigger = true;
        l.turns.insert(":INR?".to_owned(), VecDeque::from([Some(text("8192"))]));
    });
    let line2 = Arc::clone(&line);
    bounded(move || {
        let (tx, _rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        while line2.lock().unwrap().sent().iter().filter(|c| *c == ":INR?").count() < 3 {
            std::thread::sleep(Duration::from_millis(1));
        }
        d.close().unwrap();
        let after = line2.lock().unwrap().log.len();
        std::thread::sleep(Duration::from_millis(30));
        assert_eq!(line2.lock().unwrap().log.len(), after, "the poll went on after the close");
    });
}

#[test]
fn a_start_with_nothing_enabled_is_refused_and_sends_nothing() {
    let (mut d, line) = device("SDS1202X", |_| {});
    for pos in 0..2 {
        d.enable_channel(pos, false).unwrap();
    }
    let (tx, _rx) = sync_channel(4);
    let e = d.start(PacketSink::new(tx, 0)).unwrap_err().to_string();
    assert!(e.contains("No channels enabled"), "{e}");
    assert_eq!(line.lock().unwrap().sent(), Vec::<String>::new());
}

#[test]
fn a_start_on_a_running_capture_is_busy() {
    let (mut d, _) = device("SDS1202X", |l| {
        l.no_trigger = true;
        l.turns.insert(":INR?".to_owned(), VecDeque::from([Some(text("8192"))]));
    });
    bounded(move || {
        let (tx, _rx) = sync_channel(64);
        d.start(PacketSink::new(tx.clone(), 0)).unwrap();
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.close().unwrap();
    });
}

/// Each setting, as literal as it goes out.
#[test]
fn settings_go_out_as_cs_do() {
    let group = Some("CH2");
    let r = |p, q| Value::Rational(p, q);
    let s = |t: &str| Value::Str(t.to_owned());
    let cases: Vec<(Key, Value, Option<&str>, &str)> = vec![
        (Key::Timebase, r(2, 1000), None, "TDIV 2MS"),
        (Key::Timebase, r(1, 2000), None, "TDIV 500US"),
        (Key::Timebase, r(5, 1_000_000_000), None, "TDIV 5NS"),
        (Key::Timebase, r(50, 1), None, "TDIV 50S"),
        (Key::TriggerSource, s("CH2"), None, "TRSE EDGE,SR,C2,OFF"),
        (Key::TriggerSource, s("AC Line"), None, "TRSE EDGE,SR,LINE,OFF"),
        (Key::TriggerSource, s("Ext"), None, "TRSE EDGE,SR,EX,OFF"),
        (Key::TriggerSource, s("Ext /5"), None, "TRSE EDGE,SR,EX5,OFF"),
        (Key::TriggerSource, s("D3"), None, "TRSE EDGE,SR,D3,OFF"),
        (Key::TriggerSlope, s("f"), None, "C1:TRSL NEG"),
        (Key::TriggerSlope, s("r"), None, "C1:TRSL POS"),
        (Key::TriggerLevel, Value::F64(0.5), None, ":TRIG:EDGE:LEV 0.500"),
        (Key::HorizTriggerpos, Value::F64(0.25), None, ":TIM:OFFS 0.004125"),
        (Key::Vdiv, r(500, 1000), group, "C2:VDIV 500MV"),
        (Key::Vdiv, r(1, 1), group, "C2:VDIV 1V"),
        (Key::Vdiv, r(1, 2000), group, "C2:VDIV 500UV"),
        (Key::Coupling, s("D50 DC 50 Ohm"), group, "C2:CPL D50"),
        (Key::Coupling, s("GND"), group, "C2:CPL GND"),
        (Key::ProbeFactor, Value::U64(10), group, "C2:ATTN 10"),
    ];
    for (key, value, group, want) in cases {
        let (mut d, line) = device("SDS1104X-E", |_| {});
        d.set(&key, &value, group).unwrap();
        assert_eq!(line.lock().unwrap().sent()[0], want, "{}", key.id());
    }
}

#[test]
fn local_settings_send_nothing() {
    let (mut d, line) = device("SDS1202X", |_| {});
    d.set(&Key::LimitFrames, &Value::U64(3), None).unwrap();
    d.set(&Key::Averaging, &Value::Bool(true), None).unwrap();
    d.set(&Key::AvgSamples, &Value::U64(16), None).unwrap();
    d.set(&Key::DataSource, &Value::Str("History".to_owned()), None).unwrap();
    assert_eq!(line.lock().unwrap().sent(), Vec::<String>::new());
    assert_eq!(d.get(&Key::LimitFrames, None).unwrap().as_u64(), Some(3));
    assert_eq!(d.get(&Key::Averaging, None).unwrap().as_bool(), Some(true));
    assert_eq!(d.get(&Key::AvgSamples, None).unwrap().as_u64(), Some(16));
    assert_eq!(d.get(&Key::DataSource, None).unwrap().as_str(), Some("History"));
    d.set(&Key::DataSource, &Value::Str("Display".to_owned()), None).unwrap();
    assert_eq!(d.get(&Key::DataSource, None).unwrap().as_str(), Some("Screen"), "as C");
}

/// A slope after a source goes to the scope's word for it, where C's goes to `CH2:`.
#[test]
fn a_slope_follows_the_source_in_the_scopes_word() {
    for (source, word) in [("CH2", "C2"), ("AC Line", "LINE"), ("Ext", "EX"), ("D4", "D4")] {
        let (mut d, line) = device("SDS1104X-E", |_| {});
        d.set(&Key::TriggerSource, &Value::Str(source.to_owned()), None).unwrap();
        assert_eq!(d.get(&Key::TriggerSource, None).unwrap().as_str(), Some(source));
        d.set(&Key::TriggerSlope, &Value::Str("f".to_owned()), None).unwrap();
        assert_eq!(line.lock().unwrap().sent()[1], format!("{word}:TRSL NEG"));
        assert_eq!(d.get(&Key::TriggerSlope, None).unwrap().as_str(), Some("f"));
    }
}

#[test]
fn a_position_is_cs_of_the_timebases() {
    let (mut d, line) = device("SDS1052DL+", |_| {});
    d.set(&Key::HorizTriggerpos, &Value::F64(0.75), None).unwrap();
    // 5e-4 s by the 31 timebases of 5 ns to 50 s, as C (`api.c:569`).
    assert_eq!(line.lock().unwrap().sent()[0], ":TIM:OFFS -0.003875");
    assert_eq!(d.get(&Key::HorizTriggerpos, None).unwrap().as_f64(), Some(0.75));
}

#[test]
fn what_the_tables_lack_is_refused_and_sends_nothing() {
    let group = Some("CH1");
    let r = |p, q| Value::Rational(p, q);
    let s = |t: &str| Value::Str(t.to_owned());
    let cases: Vec<(Key, Value, Option<&str>)> = vec![
        (Key::Timebase, r(3, 1000), None),
        (Key::Timebase, r(100, 1), None),
        (Key::Vdiv, r(3, 1000), group),
        (Key::Vdiv, r(1000, 1), group),
        (Key::TriggerSource, s("CH3"), None),
        (Key::TriggerSlope, s("x"), None),
        (Key::Coupling, s("D1M"), group),
        (Key::ProbeFactor, Value::U64(3), group),
        (Key::HorizTriggerpos, Value::F64(1.5), None),
        (Key::HorizTriggerpos, Value::F64(f64::NAN), None),
        (Key::TriggerLevel, Value::F64(f64::NAN), None),
        (Key::TriggerLevel, Value::F64(f64::INFINITY), None),
        (Key::DataSource, s("Screen"), None),
        (Key::Vdiv, r(1, 1), None),
    ];
    for (key, value, group) in cases {
        let (mut d, line) = device("SDS1104X-E", |_| {});
        assert!(d.set(&key, &value, group).is_err(), "{} {value:?}", key.id());
        assert_eq!(line.lock().unwrap().sent(), Vec::<String>::new(), "{}", key.id());
    }
    let (mut d, _) = device("SDS1052DL+", |_| {});
    assert!(d.set(&Key::Timebase, &r(1, 1_000_000_000), None).is_err(), "under its fastest");
}

/// A failed send leaves what is known alone, where C has changed it already.
#[test]
fn a_write_that_fails_leaves_the_state_known() {
    let group = Some("CH1");
    let r = |p, q| Value::Rational(p, q);
    let s = |t: &str| Value::Str(t.to_owned());
    let cases: Vec<(Key, Value, Option<&str>)> = vec![
        (Key::Timebase, r(2, 1000), None),
        (Key::TriggerSource, s("CH2"), None),
        (Key::TriggerSlope, s("f"), None),
        (Key::TriggerLevel, Value::F64(0.5), None),
        (Key::HorizTriggerpos, Value::F64(0.25), None),
        (Key::Vdiv, r(2, 1), group),
        (Key::Coupling, s("GND"), group),
        (Key::ProbeFactor, Value::U64(10), group),
    ];
    for (key, value, group) in cases {
        let (mut d, line) = device("SDS1104X-E", |_| {});
        line.lock().unwrap().refuse = Some(String::new());
        let before = format!("{:?}", d.get(&key, group).unwrap());
        assert!(d.set(&key, &value, group).is_err(), "{}", key.id());
        assert_eq!(format!("{:?}", d.get(&key, group).unwrap()), before, "{}", key.id());
    }
}

#[test]
fn a_probe_factor_reads_the_scales_again() {
    let (mut d, line) = device("SDS1202X", |l| {
        l.turns.insert("C1:VDIV?".to_owned(), VecDeque::from([Some(text("1.00E+01"))]));
    });
    // That turn was the open's: this one's own.
    line.lock()
        .unwrap()
        .turns
        .insert("C1:VDIV?".to_owned(), VecDeque::from([Some(text("1.00E+01"))]));
    d.set(&Key::ProbeFactor, &Value::U64(10), Some("CH1")).unwrap();
    assert_eq!(
        line.lock().unwrap().sent(),
        ["C1:ATTN 10", "C1:VDIV?", "C2:VDIV?", "C1:OFST?", "C2:OFST?"]
    );
    assert_eq!(d.get(&Key::ProbeFactor, Some("CH1")).unwrap().as_u64(), Some(10));
    assert_eq!(d.get(&Key::Vdiv, Some("CH1")).unwrap(), Value::Rational(10, 1));
}

#[test]
fn a_probe_factor_whose_scales_dont_come_back_is_still_set() {
    let (mut d, line) = device("SDS1202X", |_| {});
    line.lock().unwrap().turns.insert("C1:VDIV?".to_owned(), VecDeque::from([None]));
    d.set(&Key::ProbeFactor, &Value::U64(10), Some("CH1")).unwrap();
    assert_eq!(d.get(&Key::ProbeFactor, Some("CH1")).unwrap().as_u64(), Some(10));
    assert_eq!(d.get(&Key::Vdiv, Some("CH1")).unwrap(), Value::Rational(1, 1), "as known");
}

#[test]
fn a_samplerate_is_asked_for_each_time() {
    let (d, line) = device("SDS1202X", |l| l.depth = 7000);
    assert_eq!(d.get(&Key::Samplerate, None).unwrap().as_u64(), Some(1_000_000));
    assert_eq!(line.lock().unwrap().sent(), ["SANU? C1", ":TDIV?"]);
    line.lock()
        .unwrap()
        .turns
        .insert(":TDIV?".to_owned(), VecDeque::from([Some(text("1.00E-03"))]));
    assert_eq!(d.get(&Key::Samplerate, None).unwrap().as_u64(), Some(500_000));
    assert_eq!(line.lock().unwrap().sent().len(), 4);
}

#[test]
fn a_failed_samplerate_says_what_was_known() {
    let (d, line) = device("SDS1202X", |l| l.depth = 7000);
    assert_eq!(d.get(&Key::Samplerate, None).unwrap().as_u64(), Some(1_000_000));
    assert_eq!(d.state.samplerate, 0.0, "a get changes nothing");
    line.lock().unwrap().turns.insert("SANU? C1".to_owned(), VecDeque::from([None]));
    assert_eq!(d.get(&Key::Samplerate, None).unwrap().as_u64(), Some(0));
}

#[test]
fn an_analyzer_on_asks_its_own_depth() {
    let (d, line) = device("SDS1104X-E", |l| {
        l.depth = 7000;
        l.turns.insert("DI:SW?".to_owned(), VecDeque::from([Some(text("ON"))]));
    });
    let _ = d.get(&Key::Samplerate, None).unwrap();
    assert_eq!(line.lock().unwrap().sent(), ["SANU? C1", "SANU? D0", ":TDIV?"]);
}

#[test]
fn a_get_of_another_key_asks_nothing() {
    let (d, line) = device("SDS1202X", |_| {});
    assert!(d.get(&Key::LimitSamples, None).is_err());
    assert!(d.get(&Key::Vdiv, None).is_err(), "a group is wanted");
    assert!(d.get(&Key::Coupling, Some("CH9")).is_err());
    assert_eq!(line.lock().unwrap().sent(), Vec::<String>::new());
}

#[test]
fn during_a_capture_a_get_waits_for_nothing_and_a_set_is_refused() {
    let (mut d, line) = device("SDS1202X", |l| {
        l.no_trigger = true;
        l.turns.insert(":INR?".to_owned(), VecDeque::from([Some(text("8192"))]));
    });
    let line2 = Arc::clone(&line);
    bounded(move || {
        let (tx, _rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        while line2.lock().unwrap().sent().iter().filter(|c| *c == ":INR?").count() < 2 {
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(d.get(&Key::Samplerate, None).is_ok());
        assert!(matches!(d.set(&Key::TriggerLevel, &Value::F64(1.0), None), Err(Error::Busy)));
        assert_eq!(d.state.trigger_level, 0.0, "never stored unsent");
        d.close().unwrap();
    });
}

#[test]
fn options_are_cs_in_its_order() {
    let (d, _) = device("SDS1104X-E", |_| {});
    let ids: Vec<&str> = d.options(None).unwrap().iter().map(|o| o.key.id()).collect();
    assert_eq!(
        ids,
        [
            "timebase",
            "triggersource",
            "triggerslope",
            "triggerlevel",
            "horiz_triggerpos",
            "num_hdiv",
            "samplerate",
            "limit_frames",
            "data_source",
            "averaging",
            "avg_samples",
        ]
    );
    let ids: Vec<&str> = d.options(Some("CH1")).unwrap().iter().map(|o| o.key.id()).collect();
    assert_eq!(ids, ["num_vdiv", "vdiv", "timebase", "coupling", "probe_factor", "data_source"]);
    assert_eq!(d.options(Some("LA")).unwrap().len(), 0);
    assert!(d.options(Some("CH5")).is_err());
    let (d, _) = device("SDS1202X", |_| {});
    assert!(d.options(Some("LA")).is_err(), "it has no analyzer");
    assert!(d.options(Some("CH3")).is_err());
}

#[test]
fn what_a_get_answers_on_a_fresh_scope() {
    let (d, _) = device("SDS1104X-E", |_| {});
    let g = |k| d.get(&k, None).unwrap();
    assert_eq!(g(Key::NumHdiv), Value::I32(14));
    assert_eq!(g(Key::Timebase), Value::Rational(500, 1_000_000));
    assert_eq!(g(Key::LimitFrames), Value::U64(1));
    assert_eq!(g(Key::TriggerSource).as_str(), Some("C1"), "the scope's word, as C");
    assert_eq!(d.get(&Key::NumVdiv, None).unwrap(), Value::I32(17));
    let c = Some("CH1");
    assert_eq!(d.get(&Key::Vdiv, c).unwrap(), Value::Rational(1, 1));
    assert_eq!(d.get(&Key::Coupling, c).unwrap().as_str(), Some("D1M"));
    assert_eq!(d.get(&Key::ProbeFactor, c).unwrap().as_u64(), Some(1));
    assert_eq!(d.get(&Key::NumHdiv, None).unwrap(), Value::I32(14));
}

#[test]
fn a_scale_of_500_uv_and_5_mv_are_told_apart() {
    let (d, _) = device("SDS1104X-E", |l| {
        l.turns.insert("C1:VDIV?".to_owned(), VecDeque::from([Some(text("5.00E-04"))]));
        l.turns.insert("C2:VDIV?".to_owned(), VecDeque::from([Some(text("5.00E-03"))]));
    });
    assert_eq!(d.get(&Key::Vdiv, Some("CH1")).unwrap(), Value::Rational(500, 1_000_000));
    assert_eq!(d.get(&Key::Vdiv, Some("CH2")).unwrap(), Value::Rational(5, 1000));
}

#[test]
fn an_unknown_slope_has_no_get() {
    let (d, _) = device("SDS1104X-E", |l| {
        l.turns.insert("C1:TRSL?".to_owned(), VecDeque::from([Some(text("WINDOW"))]));
    });
    assert!(d.get(&Key::TriggerSlope, None).is_err());
}

#[test]
fn lists_are_cs() {
    let (d, _) = device("SDS1202DL", |_| {});
    let Ok(ValueList::Str(sources)) = d.list(&Key::TriggerSource, None) else { panic!() };
    assert_eq!(sources.len(), 5);
    let Ok(ValueList::Rational(vdivs)) = d.list(&Key::Vdiv, Some("CH1")) else { panic!() };
    assert_eq!(vdivs[0], (2, 1000));
    let Ok(ValueList::Rational(tb)) = d.list(&Key::Timebase, None) else { panic!() };
    assert_eq!(tb[0], (20, 1_000_000_000));
    let Ok(ValueList::Str(sources)) = d.list(&Key::DataSource, None) else { panic!() };
    assert_eq!(sources, ["Display"]);
    let Ok(ValueList::U64(avg)) = d.list(&Key::AvgSamples, None) else { panic!() };
    assert_eq!(avg, [4, 16, 32, 64, 128, 256, 512, 1024]);
    assert!(d.list(&Key::Vdiv, None).is_err());
    let Ok(ValueList::Str(slopes)) = d.list(&Key::TriggerSlope, None) else { panic!() };
    assert_eq!(slopes, ["r", "f"]);
    assert!(d.list(&Key::Coupling, None).is_err());
}

#[test]
fn a_new_capture_after_one_ended_runs() {
    let (d, _) = device("SDS1202X", |_| {});
    let (first, d) = capture(d);
    let (second, _d) = capture(d);
    assert_eq!(samples(&first).len(), samples(&second).len());
}

/// C's own sleeps, by hand: an SPO's 2.5 µs a point, an E's tenth of a sweep, and the
/// thousandth of a sweep it sleeps after a trigger.
#[test]
fn the_waits_are_cs_sums() {
    let (mut d, _) = device("SDS1202X", |_| {});
    d.state.depth_analog = 14_000_000;
    let c = |d: &SiglentDevice| Capture {
        model: d.model,
        channels: Vec::new(),
        st: d.state.clone(),
        shared: Arc::new(Mutex::new(d.state.clone())),
        stop: Arc::new(AtomicBool::new(false)),
        timing: TIMING,
    };
    assert_eq!(c(&d).preparation(), Duration::from_secs(35));
    assert_eq!(c(&d).block_bound(), Duration::from_secs(105), "three times it");
    d.state.depth_analog = 700;
    assert_eq!(c(&d).block_bound(), Duration::from_secs(30), "and never under this");
    let (mut d, _) = device("SDS1202X-E", |_| {});
    d.state.timebase = 100.0;
    assert_eq!(c(&d).preparation(), Duration::from_secs(140));
    assert_eq!(c(&d).after_trigger(), Duration::ZERO, "too slow for that sleep");
    d.state.timebase = 0.1;
    let after = c(&d).after_trigger().as_secs_f64();
    assert!((after - 0.0014).abs() < 1e-6, "{after}");
    d.state.timebase = 1e-7;
    assert_eq!(c(&d).after_trigger(), Duration::ZERO, "too fast for it");
    d.state.timebase = 0.0;
    assert_eq!(c(&d).preparation(), Duration::ZERO);
    d.state.timebase = f32::NAN;
    assert_eq!(c(&d).preparation(), Duration::ZERO);
}

#[test]
fn the_long_limits_are_these() {
    assert_eq!(TIMING.first, Duration::from_secs(30));
    assert_eq!(TIMING.later, Duration::from_secs(600));
    assert_eq!(TIMING.poll, Duration::from_millis(10));
    assert_eq!(TIMING.no_trigger, Duration::from_secs(60));
}

/// The real sleep at a real timebase, from below: a sleep's length only grows. The
/// timebase is the scope's, read at the start and again after the shot.
#[test]
fn a_real_preparation_is_waited_for() {
    let (mut d, line) = device("SDS1202X-E", |_| {});
    let slow = std::iter::repeat_n(Some(text("5.00E-02")), 4).collect();
    line.lock().unwrap().turns.insert(":TDIV?".to_owned(), slow);
    d.timing.sleeps = true;
    only(&mut d, 0);
    let began = Instant::now();
    let (packets, _d) = capture(d);
    assert_eq!(failure(&packets), None);
    assert!(began.elapsed() >= Duration::from_millis(70), "a tenth of a sweep of 0.7 s");
}

#[test]
fn a_nearest_is_the_first_nearest() {
    assert_eq!(nearest(models::TIMEBASES, 1.0), Some((1, 1)));
    assert_eq!(nearest(models::TIMEBASES, 1e3), Some((100, 1)));
    assert_eq!(nearest(models::TIMEBASES, f32::NAN), None);
    assert_eq!(table_entry(models::VDIVS, (1, 200)), Some((5, 1000)));
    assert_eq!(table_entry(models::VDIVS, (3, 1000)), None);
}
