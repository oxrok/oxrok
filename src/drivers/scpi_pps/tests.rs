// Counterpart in sigrok:
//   libsigrok/src/hardware/scpi-pps/api.c,
//   libsigrok/src/hardware/scpi-pps/protocol.c,
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2017,2019 Frank Stettner <frank-stettner@gmx.net>,
//   Copyright (C) 2019 Frank Stettner <frank-stettner@gmx.net>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Martin Ling,
//   Aurelien Jacobs, Gerhard Sittig, Soeren Apel, Alexandru Gagniuc,
//   Florian Schmidt, Daniel Elstner, Kota Ando, Marc Schink, Martin Atkins
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

// This file is a port of the sigrok counterpart's behaviour, GPL-3.0-or-later like it.

//! The driver over a fake line: what goes out, in which order, and what a capture does
//! at each pace.

#![allow(clippy::match_same_arms, clippy::unnecessary_literal_bound)]

use super::*;
use crate::scpi::ScpiTransport;
use crate::serial::WriteLog;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// What the fake's instrument says to a line: its pieces, each after that many reads
/// that find nothing.
type Instrument = Box<dyn FnMut(&str) -> Vec<(u32, String)> + Send>;

#[derive(Default)]
struct State {
    /// What was asked, in order.
    written: WriteLog,
    /// Where [`written`] starts.
    from: usize,
    /// Answers to come: reads that find nothing first, the bytes, and if that ends one.
    coming: VecDeque<(u32, Vec<u8>)>,
    failing_writes: u32,
    failing_after: Option<u32>,
    gone: bool,
    /// A piece that ends a read ends an answer, line end or not: C's tcp-raw rule.
    short_ends: bool,
    closes: u32,
    discards: u32,
    /// Reads to come that fail with a device's error, as a bad USBTMC header does.
    bad_reads: u32,
    reopens: u32,
    reads: u32,
}

struct Fake {
    state: Arc<Mutex<State>>,
    respond: Instrument,
    tags: bool,
    complete: bool,
}

impl ScpiTransport for Fake {
    fn resource(&self) -> &str {
        "fake"
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        let text = String::from_utf8_lossy(data).trim_end().to_owned();
        {
            let mut s = self.state.lock().unwrap();
            if s.gone {
                return Err(Error::Disconnected);
            }
            if let Some(after) = &mut s.failing_after {
                if *after == 0 {
                    s.failing_writes = u32::MAX;
                } else {
                    *after -= 1;
                }
            }
            if s.failing_writes > 0 {
                s.failing_writes -= 1;
                return Err(Error::device("write failed"));
            }
            s.written.push((Instant::now(), text.clone().into_bytes()));
        }
        let answers = (self.respond)(&text);
        let mut s = self.state.lock().unwrap();
        for (wait, bytes) in answers {
            s.coming.push_back((wait, bytes.into_bytes()));
        }
        Ok(())
    }

    fn read_begin(&mut self) -> Result<()> {
        self.complete = false;
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut s = self.state.lock().unwrap();
        s.reads += 1;
        if s.gone {
            return Err(Error::Disconnected);
        }
        if s.bad_reads > 0 {
            s.bad_reads -= 1;
            return Err(Error::device("USBTMC invalid bulk in header."));
        }
        // A read that finds nothing takes its time.
        let nothing = match s.coming.front_mut() {
            None => true,
            Some(front) if front.0 > 0 => {
                front.0 -= 1;
                true
            }
            Some(_) => false,
        };
        if nothing {
            drop(s);
            std::thread::sleep(timeout);
            return Ok(0);
        }
        let front = s.coming.front_mut().expect("just looked");
        let n = front.1.len().min(buf.len());
        buf[..n].copy_from_slice(&front.1[..n]);
        front.1.drain(..n);
        if front.1.is_empty() {
            let last = buf[..n].last() == Some(&b'\n');
            s.coming.pop_front();
            self.complete = last || s.short_ends;
        }
        Ok(n)
    }

    fn read_complete(&self) -> bool {
        self.complete
    }

    fn close(&mut self) -> Result<()> {
        self.state.lock().unwrap().closes += 1;
        Ok(())
    }

    fn reopen(&mut self) -> Result<()> {
        self.state.lock().unwrap().reopens += 1;
        Ok(())
    }

    fn tags_answers(&self) -> bool {
        self.tags
    }

    fn discard(&mut self, _timeout: Duration) -> Result<Option<usize>> {
        if self.tags {
            return Ok(None);
        }
        let mut s = self.state.lock().unwrap();
        s.discards += 1;
        // What has come by now. An answer still far off stays on its way.
        let mut n = 0;
        while let Some(front) = s.coming.front_mut() {
            if front.0 > 0 {
                front.0 -= 1;
                break;
            }
            n += front.1.len();
            s.coming.pop_front();
        }
        Ok(Some(n))
    }
}

const DP832: &str = "RIGOL TECHNOLOGIES,DP832,DP8E,01.02\n";

/// A DP832 that answers each query at once with a number.
fn dp832(line: &str) -> Vec<(u32, String)> {
    let text = match line {
        "*IDN?" => DP832.to_owned(),
        "SYST:BEEP:STAT?" => "ON\n".to_owned(),
        ":MEAS:VOLT?" => "12.5\n".to_owned(),
        ":MEAS:CURR?" => "1.5\n".to_owned(),
        ":MEAS:POWE?" => "18.75\n".to_owned(),
        ":OUTP:MODE?" => "CV\n".to_owned(),
        ":OUTP?" => "OFF\n".to_owned(),
        l if l.ends_with('?') => "1.0\n".to_owned(),
        _ => return Vec::new(),
    };
    vec![(0, text)]
}

fn line(respond: Instrument, tags: bool) -> (Scpi, Arc<Mutex<State>>) {
    let state = Arc::new(Mutex::new(State::default()));
    let fake = Fake { state: Arc::clone(&state), respond, tags, complete: false };
    let mut scpi = Scpi::new(Box::new(fake));
    scpi.read_timeout = Duration::from_millis(1);
    (scpi, state)
}

fn device_with(respond: Instrument, tags: bool, hpib: bool) -> (PpsDevice, Arc<Mutex<State>>) {
    let (scpi, state) = line(respond, tags);
    let mut device = PpsDevice::from_scpi(scpi, hpib).unwrap();
    device.open().unwrap();
    (device, state)
}

fn device() -> (PpsDevice, Arc<Mutex<State>>) {
    device_with(Box::new(dp832), false, false)
}

fn written(state: &Arc<Mutex<State>>) -> Vec<String> {
    let s = state.lock().unwrap();
    s.written[s.from..].iter().map(|(_, w)| String::from_utf8_lossy(w).into_owned()).collect()
}

fn clear(state: &Arc<Mutex<State>>) {
    let mut s = state.lock().unwrap();
    s.from = s.written.len();
}

/// Runs `f` on a thread with a bound: a stop that doesn't bite fails, and doesn't hang.
fn bounded<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    rx.recv_timeout(Duration::from_secs(120)).expect("bounded test ran out")
}

fn collect(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let mut out = Vec::new();
    while out.len() < 10_000 {
        let packet = rx.recv_timeout(Duration::from_secs(60)).expect("a packet").1;
        let end = matches!(packet, Packet::End);
        out.push(packet);
        if end {
            return out;
        }
    }
    panic!("no end in 10000 packets");
}

fn readings(packets: &[Packet]) -> Vec<(usize, f32)> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some((a.channels[0], a.samples[0])),
            _ => None,
        })
        .collect()
}

fn failure(packets: &[Packet]) -> Option<&str> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(text) => Some(text.as_str()),
        _ => None,
    })
}

#[test]
fn a_scan_and_open_and_close() {
    let (mut device, state) = device();
    assert_eq!(
        written(&state),
        ["*IDN?", "SYST:LOCAL", "SYST:REMOTE", "SYST:BEEP:STAT?", "SYST:BEEP:STAT OFF"]
    );
    assert_eq!(device.info().driver, "scpi-pps");
    assert_eq!(device.info().vendor, "Rigol");
    assert_eq!(device.info().model, "DP832");
    assert_eq!(device.info().serial, "DP8E");
    assert_eq!(device.info().version, "01.02");
    let names: Vec<&str> = device.channels().iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["V1", "I1", "P1", "V2", "I2", "P2", "V3", "I3", "P3"]);
    assert_eq!(device.channel_groups().len(), 3);
    assert_eq!(state.lock().unwrap().closes, 1);
    assert_eq!(state.lock().unwrap().reopens, 1);
    clear(&state);
    device.close().unwrap();
    assert_eq!(written(&state), ["SYST:BEEP:STAT ON", "SYST:LOCAL"]);
    assert_eq!(state.lock().unwrap().closes, 2);
    assert!(device.get(&Key::Voltage, Some("1")).is_err());
}

#[test]
fn a_beeper_that_is_off_stays_as_it_was() {
    let respond = |l: &str| match l {
        "SYST:BEEP:STAT?" => vec![(0, "OFF\n".to_owned())],
        other => dp832(other),
    };
    let (mut device, state) = device_with(Box::new(respond), false, false);
    device.close().unwrap();
    let w = written(&state);
    assert!(!w.iter().any(|l| l.starts_with("SYST:BEEP:STAT O")), "{w:?}");
}

#[test]
fn an_hp_says_who_it_is() {
    let respond = |l: &str| match l {
        "ID?" => vec![(0, "HP6632A\n".to_owned())],
        "ROM?" => vec![(0, "B01 B02\n".to_owned())],
        _ => Vec::new(),
    };
    let (device, state) = device_with(Box::new(respond), false, true);
    assert_eq!(device.info().vendor, "HP");
    assert_eq!(device.info().model, "HP6632A");
    assert_eq!(device.info().version, "B01.B02");
    assert_eq!(device.info().driver, "scpi-pps");
    assert_eq!(written(&state), ["ID?", "ROM?"], "no remote, local or beeper in its table");
}

#[test]
fn hpib_needs_conn() {
    assert!(HpibPpsDriver.scan(&[]).unwrap().is_empty());
}

#[test]
fn a_scan_that_cannot_open_is_nothing() {
    let conn = |c: &str| vec![(Key::Conn, Value::Str(c.to_owned()))];
    for c in ["tcp-raw/host", "vxi/host", "/dev/oxrok-no-such-node"] {
        assert!(ScpiPpsDriver.scan(&conn(c)).unwrap().is_empty(), "{c}");
        assert!(HpibPpsDriver.scan(&conn(c)).unwrap().is_empty(), "{c}");
    }
    assert!(ScpiPpsDriver.scan(&[(Key::Samplerate, Value::U64(1))]).is_err());
}

#[test]
fn a_set_selects_once_and_formats_as_its_table() {
    let (mut device, state) = device();
    clear(&state);
    device.set(&Key::VoltageTarget, &Value::F64(3.3), Some("2")).unwrap();
    device.set(&Key::CurrentLimit, &Value::F64(0.5), Some("2")).unwrap();
    device.set(&Key::Enabled, &Value::Bool(true), Some("3")).unwrap();
    device.set(&Key::Enabled, &Value::Bool(false), Some("3")).unwrap();
    assert_eq!(
        written(&state),
        [
            ":INST:NSEL 2",
            ":SOUR:VOLT 3.300000",
            ":SOUR:CURR 0.500000",
            ":INST:NSEL 3",
            ":OUTP ON",
            ":OUTP OFF"
        ]
    );
}

#[test]
fn what_is_refused_goes_nowhere() {
    let (mut device, state) = device();
    clear(&state);
    // No group: it would go to whichever output is selected.
    assert!(device.set(&Key::VoltageTarget, &Value::F64(3.3), None).is_err());
    assert!(device.get(&Key::VoltageTarget, None).is_err());
    assert!(device.set(&Key::VoltageTarget, &Value::F64(3.3), Some("9")).is_err());
    assert!(device.get(&Key::Voltage, Some("9")).is_err());
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(device.set(&Key::VoltageTarget, &Value::F64(value), Some("1")).is_err());
    }
    assert!(device.set(&Key::VoltageTarget, &Value::Bool(true), Some("1")).is_err());
    assert!(device.set(&Key::Enabled, &Value::F64(1.0), Some("1")).is_err());
    // Read only, and a key this device doesn't have.
    assert!(device.set(&Key::Voltage, &Value::F64(1.0), Some("1")).is_err());
    assert!(device.get(&Key::OutputFrequency, Some("1")).is_err());
    assert!(device.list(&Key::VoltageTarget, None).is_err());
    assert!(device.list(&Key::Voltage, Some("1")).is_err());
    assert_eq!(written(&state), Vec::<String>::new());
}

/// A selection whose write failed: what was to go to that output doesn't go, and the
/// next setting selects again rather than going to the output selected before.
#[test]
fn a_failed_selection_stops_the_setting_and_selects_again() {
    let (mut device, state) = device();
    device.set(&Key::Enabled, &Value::Bool(true), Some("1")).unwrap();
    clear(&state);
    state.lock().unwrap().failing_writes = 1;
    let e = device.set(&Key::VoltageTarget, &Value::F64(3.3), Some("2")).unwrap_err();
    assert_eq!(
        e.to_string(),
        "device error: Setting voltage_target on output 2 failed, so whether it took effect \
         isn't known: device error: write failed"
    );
    assert_eq!(written(&state), Vec::<String>::new());
    device.set(&Key::VoltageTarget, &Value::F64(3.3), Some("2")).unwrap();
    assert_eq!(written(&state), [":INST:NSEL 2", ":SOUR:VOLT 3.300000"]);
}

/// The plain send behind a failed one, on a selection that stands.
#[test]
fn a_failed_setting_says_its_state_is_not_known() {
    let (mut device, state) = device();
    state.lock().unwrap().failing_after = Some(1);
    let e = device.set(&Key::VoltageTarget, &Value::F64(3.3), Some("2")).unwrap_err();
    assert!(e.to_string().contains("whether it took effect isn't known"), "{e}");
}

#[test]
fn a_close_forgets_the_selection() {
    let (mut device, state) = device();
    device.set(&Key::Enabled, &Value::Bool(true), Some("2")).unwrap();
    device.close().unwrap();
    device.open().unwrap();
    clear(&state);
    device.set(&Key::Enabled, &Value::Bool(false), Some("2")).unwrap();
    assert_eq!(written(&state), [":INST:NSEL 2", ":OUTP OFF"]);
}

#[test]
fn a_set_outside_the_listed_range_still_goes() {
    let (mut device, state) = device();
    clear(&state);
    device.set(&Key::VoltageTarget, &Value::F64(32.5), Some("1")).unwrap();
    assert_eq!(written(&state), [":INST:NSEL 1", ":SOUR:VOLT 32.500000"]);
}

#[test]
fn gets_read_the_instrument_each_time() {
    let (device, state) = device();
    clear(&state);
    assert_eq!(device.get(&Key::Voltage, Some("1")).unwrap(), Value::F64(12.5));
    assert_eq!(device.get(&Key::Voltage, Some("1")).unwrap(), Value::F64(12.5));
    assert_eq!(device.get(&Key::Regulation, Some("2")).unwrap(), Value::Str("CV".to_owned()));
    assert_eq!(device.get(&Key::Enabled, Some("2")).unwrap(), Value::Bool(false));
    assert_eq!(
        written(&state),
        [":INST:NSEL 1", ":MEAS:VOLT?", ":MEAS:VOLT?", ":INST:NSEL 2", ":OUTP:MODE?", ":OUTP?"]
    );
}

#[test]
fn a_get_with_what_is_no_answer_is_an_error() {
    let answer = |text: &'static str| {
        move |l: &str| match l {
            "*IDN?" => dp832(l),
            "SYST:BEEP:STAT?" => dp832(l),
            l if l.ends_with('?') => vec![(0, text.to_owned())],
            _ => Vec::new(),
        }
    };
    for text in ["\n", "abc\n", "0x10\n", "nan\n", "12.5 V\n"] {
        let (device, _) = device_with(Box::new(answer(text)), false, false);
        let e = device.get(&Key::VoltageTarget, Some("1")).unwrap_err();
        assert!(e.to_string().contains("not a number"), "{text:?}: {e}");
    }
    let (device, _) = device_with(Box::new(answer("Timeout\n")), false, false);
    assert!(device.get(&Key::Enabled, Some("1")).is_err());
    let (device, _) = device_with(Box::new(answer("XX\n")), false, false);
    assert!(device.get(&Key::Regulation, Some("1")).unwrap_err().to_string().contains("'XX'"));
}

#[test]
fn a_get_takes_a_late_answer_on_a_line_with_no_tags() {
    let respond = |l: &str| match l {
        ":SOUR:VOLT?" => vec![(3, "7.5\n".to_owned())],
        other => dp832(other),
    };
    let (device, _) = device_with(Box::new(respond), false, false);
    assert_eq!(device.get(&Key::VoltageTarget, Some("1")).unwrap(), Value::F64(7.5));
}

#[test]
fn a_silent_get_is_a_timeout_and_the_next_finds_the_line_clear() {
    let mut first = true;
    let respond = move |l: &str| match l {
        ":SOUR:VOLT?" if first => {
            first = false;
            Vec::new()
        }
        ":SOUR:VOLT?" => vec![(0, "7.5\n".to_owned())],
        other => dp832(other),
    };
    let (device, _) = device_with(Box::new(respond), false, false);
    assert!(matches!(device.get(&Key::VoltageTarget, Some("1")), Err(Error::Timeout)));
    assert_eq!(device.get(&Key::VoltageTarget, Some("1")).unwrap(), Value::F64(7.5));
}

#[test]
fn the_options_and_lists_of_each_kind() {
    let (device, _) = device();
    let global: Vec<Key> = device.options(None).unwrap().into_iter().map(|o| o.key).collect();
    assert_eq!(
        global,
        [Key::Continuous, Key::OverTemperatureProtection, Key::LimitSamples, Key::LimitMsec]
    );
    assert!(device.options(Some("9")).is_err());
    assert_eq!(
        device.list(&Key::VoltageTarget, Some("3")).unwrap(),
        ValueList::RangeStepF64 { min: 0.0, max: 5.0, step: 0.001 }
    );
}

/// What a profile offers and has no command for: a get answers "not supported", and a
/// set, as `sr_scpi_cmd` has it, is nothing and no error. Nothing here may be a set.
#[test]
fn what_a_profile_offers_and_has_no_command_for() {
    let sets: &[(Key, &[Cmd])] = &[
        (Key::VoltageTarget, &[Cmd::SetVoltageTarget]),
        (Key::OutputFrequencyTarget, &[Cmd::SetFrequencyTarget]),
        (Key::CurrentLimit, &[Cmd::SetCurrentLimit]),
        (Key::Enabled, &[Cmd::SetOutputEnable, Cmd::SetOutputDisable]),
        (
            Key::OverVoltageProtectionEnabled,
            &[Cmd::SetOverVoltageProtectionEnable, Cmd::SetOverVoltageProtectionDisable],
        ),
        (Key::OverVoltageProtectionThreshold, &[Cmd::SetOverVoltageProtectionThreshold]),
        (
            Key::OverCurrentProtectionEnabled,
            &[Cmd::SetOverCurrentProtectionEnable, Cmd::SetOverCurrentProtectionDisable],
        ),
        (Key::OverCurrentProtectionThreshold, &[Cmd::SetOverCurrentProtectionThreshold]),
        (Key::OverCurrentProtectionDelay, &[Cmd::SetOverCurrentProtectionDelay]),
    ];
    let mut gets = std::collections::BTreeSet::new();
    let mut unset = std::collections::BTreeSet::new();
    for p in &profiles::PROFILES {
        for o in p.options {
            let (get, _) = get_command(&o.key).expect("a key the driver has");
            let constant = p.dialect == Dialect::Hmp && o.key == Key::OverVoltageProtectionEnabled;
            if o.access.get && !constant && scpi::cmd_get(p.commands, &get).is_none() {
                gets.insert(format!("{}: {}", p.vendor, o.key.id()));
            }
            if o.access.set {
                let (_, commands) = sets.iter().find(|(k, _)| *k == o.key).expect("a set key");
                if commands.iter().any(|c| scpi::cmd_get(p.commands, c).is_none()) {
                    unset.insert(format!("{}: {}", p.vendor, o.key.id()));
                }
            }
        }
    }
    let gets: Vec<String> = gets.into_iter().collect();
    let unset: Vec<String> = unset.into_iter().collect();
    assert_eq!(unset, Vec::<String>::new());
    assert_eq!(gets, GETS_WITH_NO_COMMAND);
}

/// In C too: a key on the list whose command is on no table.
const GETS_WITH_NO_COMMAND: [&str; 4] =
    ["EEZ: ocp_active", "EEZ: ovp_active", "Envox: ocp_active", "Envox: ovp_active"];

#[test]
fn profiles_match_as_the_c_expressions_do() {
    let id = |vendor: &str, model: &str| HwInfo {
        manufacturer: vendor.to_owned(),
        model: model.to_owned(),
        ..HwInfo::default()
    };
    let found = |vendor: &str, model: &str| profiles::find(&id(vendor, model)).is_some();
    assert!(found("RIGOL TECHNOLOGIES", "DP832"));
    assert!(found("rigol technologies", "DP832A"));
    assert!(!found("RIGOL TECHNOLOGIES", "xDP832"));
    assert!(!found("RIGOL TECHNOLOGIES", "DP832B"));
    assert!(found("Rigol", "DP711"));
    assert!(found("BK", "9130"));
    assert!(!found("BK", "91300"));
    assert!(found("Chroma ATE", "61604"));
    assert!(found("Chroma ATE", "x61604y"));
    assert!(found("CHROMA", "62012P-80-60"));
    assert!(found("CHROMA", "x62012P-80-60y"));
    assert!(!found("CHROMA", "62012P-8-60"));
    assert!(!found("CHROMA", "62012P-80-"));
    assert!(!found("CHROMA", "6201P-80-60"));
    assert!(found("EA Elektro-Automatik GmbH & Co. KG", "PS 2042-06B"));
    assert!(found("EA Elektro-Automatik GmbH & Co. KG", "PS 2084-03B"));
    assert!(!found("EA Elektro-Automatik GmbH & Co. KG", "PS 2043-06B"));
    assert!(found("Envox", "EEZ H24005 r1"));
    assert!(!found("Envox", "xEEZ H24005 r1"));
    assert!(found("EEZ", "PSU 1"));
    assert!(found("Envox", "BB3 x"));
    assert!(found("HP", "HP6632A"));
    assert!(found("HEWLETT-PACKARD", "6632B"));
    assert!(found("Agilent Technologies", "N5767A"));
    assert!(found("Keysight Technologies", "E36312A"));
    assert!(found("PHILIPS", "PM2813/11"));
    assert!(found("PHILIPS", "PM2832/4"));
    assert!(!found("PHILIPS", "PM2813/5"));
    assert!(!found("PHILIPS", "PM2843/1"));
    assert!(!found("PHILIPS", "PM2813/111"));
    assert!(found("Rohde&Schwarz", "HMC8043"));
    assert!(found("ROHDE&SCHWARZ", "HMC8043"));
    assert!(found("HAMEG", "HMP4040"));
    assert!(found("OWON", "SPE6103"));
    assert!(!found("Acme", "DP832"));
}

#[test]
fn a_probe_fails_with_no_device() {
    for (idn, others) in [
        ("CHROMA,x62012P-80-60,1,2\n", None),
        ("CHROMA,62099P-601-1,1,2\n", None),
        ("CHROMA,62099P-60-121,1,2\n", None),
        ("CHROMA,62099P-60-120,1,2\n", None),
        ("PHILIPS,PM2813/41,1,2\n", None),
        ("Envox,BB3 x,1,2\n", Some("-1\n")),
        ("Envox,BB3 x,1,2\n", Some("x\n")),
    ] {
        let respond = move |l: &str| match (l, others) {
            ("*IDN?", _) => vec![(0, idn.to_owned())],
            (":SYST:CHAN:COUN?", Some(text)) => vec![(0, text.to_owned())],
            _ => Vec::new(),
        };
        let (scpi, _) = line(Box::new(respond), false);
        assert!(PpsDevice::from_scpi(scpi, false).is_err(), "{idn}");
    }
}

#[test]
fn a_chroma_gets_its_limits_from_its_name() {
    let respond = |l: &str| match l {
        "*IDN?" => vec![(0, "CHROMA,62012P-80-60,1,2\n".to_owned())],
        _ => Vec::new(),
    };
    let (device, _) = device_with(Box::new(respond), false, false);
    assert_eq!(
        device.list(&Key::CurrentLimit, Some("1")).unwrap(),
        ValueList::RangeStepF64 { min: 0.0, max: 60.0, step: 0.01 }
    );
    assert_eq!(
        device.list(&Key::VoltageTarget, Some("1")).unwrap(),
        ValueList::RangeStepF64 { min: 0.0, max: 80.0, step: 0.01 }
    );
    assert_eq!(device.specs[0].power[1], 1200.0);
}

#[test]
fn an_ea_reads_its_nominal_values_with_their_units() {
    let respond = |l: &str| match l {
        "*IDN?" => vec![(0, "EA Elektro-Automatik GmbH & Co. KG, PS 2084-05B, 1, 2\n".to_owned())],
        ":SYST:NOM:VOLT?" => vec![(0, "84.00 V\n".to_owned())],
        ":SYST:NOM:CURR?" => vec![(0, "5.00 A\n".to_owned())],
        ":SYST:NOM:POW?" => vec![(0, "junk\n".to_owned())],
        _ => Vec::new(),
    };
    let (device, _) = device_with(Box::new(respond), false, false);
    assert_eq!(device.specs[0].voltage[1], 84.0);
    assert_eq!(device.specs[0].current[1], 5.0);
    assert_eq!(device.specs[0].power[1], 0.0, "what is no number stays C's 0");
    let silent = |l: &str| match l {
        "*IDN?" => vec![(0, "EA Elektro-Automatik GmbH & Co. KG, PS 2084-05B, 1, 2\n".to_owned())],
        _ => Vec::new(),
    };
    let (scpi, _) = line(Box::new(silent), false);
    assert!(PpsDevice::from_scpi(scpi, false).is_err());
}

#[test]
fn an_eez_finds_its_outputs() {
    let respond = |l: &str| match l {
        "*IDN?" => vec![(0, "Envox,BB3 x,1,2\n".to_owned())],
        ":SYST:CHAN:COUN?" => vec![(0, "9\n".to_owned())],
        ":SYST:CHAN:INFO:CURR?" => vec![(0, "5\n".to_owned())],
        ":SYST:CHAN:INFO:VOLT?" => vec![(0, "40\n".to_owned())],
        ":SYST:CHAN:INFO:POW?" => vec![(0, "155\n".to_owned())],
        _ => Vec::new(),
    };
    let (device, state) = device_with(Box::new(respond), false, false);
    assert_eq!(device.specs.len(), 6);
    assert_eq!(device.channel_groups().len(), 6);
    assert_eq!(device.specs[5].voltage[1], 40.0);
    assert_eq!(device.specs[5].power[1], 155.0);
    assert_eq!(written(&state)[2], ":INST:NSEL 1");
}

fn with_limit(device: &mut PpsDevice, samples: u64) {
    device.set(&Key::LimitSamples, &Value::U64(samples), None).unwrap();
}

#[test]
fn a_capture_goes_round_its_channels_and_counts_rounds() {
    let (mut device, state) = device();
    with_limit(&mut device, 2);
    for pos in [2, 3, 4, 5, 6, 7, 8] {
        device.enable_channel(pos, false).unwrap();
    }
    clear(&state);
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    assert!(matches!(packets[0], Packet::Header { .. }));
    assert_eq!(readings(&packets), [(0, 12.5), (1, 1.5), (0, 12.5), (1, 1.5)]);
    assert_eq!(failure(&packets), None);
    assert_eq!(
        written(&state),
        [
            ":INST:NSEL 1",
            ":MEAS:VOLT?",
            ":MEAS:CURR?",
            ":MEAS:VOLT?",
            ":MEAS:CURR?",
            "SYST:BEEP:STAT ON",
            "SYST:LOCAL"
        ]
    );
}

#[test]
fn a_capture_with_no_channel_refuses_to_start() {
    let (mut device, _) = device();
    for pos in 0..device.channels().len() {
        device.enable_channel(pos, false).unwrap();
    }
    let (tx, _rx) = sync_channel(4);
    assert!(device.start(PacketSink::new(tx, 0)).is_err());
}

/// A silent query on a line with no tags: one question out, kept reading, and the
/// answer when it comes is for it.
#[test]
fn a_late_answer_on_a_line_with_no_tags_is_the_right_one() {
    let mut volts = true;
    let respond = move |l: &str| match l {
        ":MEAS:VOLT?" if volts => {
            volts = false;
            vec![(20, "12.5\n".to_owned())]
        }
        other => dp832(other),
    };
    let (mut device, state) = device_with(Box::new(respond), false, false);
    with_limit(&mut device, 1);
    for pos in 2..9 {
        device.enable_channel(pos, false).unwrap();
    }
    clear(&state);
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    assert_eq!(readings(&packets), [(0, 12.5), (1, 1.5)]);
    let asked = written(&state);
    assert_eq!(asked.iter().filter(|l| *l == ":MEAS:VOLT?").count(), 1, "{asked:?}");
}

/// USBTMC's tags let it ask again at a timeout, as C.
#[test]
fn a_silent_query_with_tags_is_asked_again() {
    let mut volts = 3;
    let respond = move |l: &str| match l {
        ":MEAS:VOLT?" if volts > 0 => {
            volts -= 1;
            Vec::new()
        }
        other => dp832(other),
    };
    let (mut device, state) = device_with(Box::new(respond), true, false);
    with_limit(&mut device, 1);
    for pos in 1..9 {
        device.enable_channel(pos, false).unwrap();
    }
    clear(&state);
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    assert_eq!(readings(&packets), [(0, 12.5)]);
    let asked = written(&state);
    assert_eq!(asked.iter().filter(|l| *l == ":MEAS:VOLT?").count(), 4, "{asked:?}");
}

fn silent_after_idn(l: &str) -> Vec<(u32, String)> {
    match l {
        "*IDN?" | "SYST:BEEP:STAT?" => dp832(l),
        _ => Vec::new(),
    }
}

fn watch(time_limited: bool, ms: [u64; 4]) -> Watch {
    Watch::with_bounds(time_limited, ms.map(Duration::from_millis))
}

/// A device that gives nothing in a capture fails at the bound, on both kinds of line.
#[test]
fn a_silent_capture_fails_at_its_bound() {
    for tags in [true, false] {
        let (mut device, _) = device_with(Box::new(silent_after_idn), tags, false);
        with_limit(&mut device, 1);
        device.watch = Some(watch(false, [20, 10_000, 10_000, 10_000]));
        let packets = bounded(move || {
            let (tx, rx) = sync_channel(64);
            device.start(PacketSink::new(tx, 0)).unwrap();
            let packets = collect(&rx);
            device.close().unwrap();
            packets
        });
        assert_eq!(readings(&packets), [], "{tags}");
        let text = failure(&packets).unwrap();
        assert!(text.contains("No answer from the instrument in 0 s."), "{text}");
    }
}

/// Answers that make nothing fail on the quick bound, with its words.
#[test]
fn answers_that_are_no_reading_fail_at_their_bound() {
    let respond = |l: &str| match l {
        "*IDN?" | "SYST:BEEP:STAT?" => dp832(l),
        l if l.ends_with('?') => vec![(0, "junk\n".to_owned())],
        _ => Vec::new(),
    };
    let (mut device, _) = device_with(Box::new(respond), false, false);
    with_limit(&mut device, 1);
    device.watch = Some(watch(false, [100_000, 30, 100_000, 100_000]));
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    assert_eq!(readings(&packets), []);
    assert!(failure(&packets).unwrap().contains("No usable answer"), "{packets:?}");
}

/// A time limit ends a silent capture with no failure, as nothing failed but the wait.
#[test]
fn a_time_limit_ends_a_silent_capture_in_peace() {
    let (mut device, _) = device_with(Box::new(silent_after_idn), true, false);
    device.set(&Key::LimitMsec, &Value::U64(20), None).unwrap();
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    assert_eq!(readings(&packets), []);
    assert_eq!(failure(&packets), None);
}

/// A device gone ends a capture, and with the failure.
#[test]
fn a_device_gone_ends_a_capture_with_an_error() {
    let (mut device, state) = device();
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        state.lock().unwrap().gone = true;
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    assert_eq!(readings(&packets), []);
    let text = failure(&packets).unwrap();
    assert!(!text.contains("No usable"), "gone is no bad answer: {text}");
}

/// A stop in the middle of a question out, and of a full queue: the close comes back.
#[test]
fn a_stop_ends_a_silent_capture_and_a_blocked_one() {
    bounded(|| {
        let (mut device, _) = device_with(Box::new(silent_after_idn), false, false);
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        // Until it has asked.
        rx.recv_timeout(Duration::from_secs(30)).unwrap();
        device.stop().unwrap();
        device.close().unwrap();
    });
    bounded(|| {
        let (mut device, _) = device();
        // A queue of one, full: nothing reads it.
        let (tx, rx) = sync_channel(1);
        let sink = PacketSink::new(tx, 0);
        assert!(sink.send(Packet::FrameBegin));
        device.start(sink).unwrap();
        std::thread::sleep(Duration::from_millis(5));
        device.stop().unwrap();
        device.close().unwrap();
        drop(rx);
    });
}

/// A start after a capture whose end is stuck on a full queue gives the old one up.
#[test]
fn a_restart_after_a_stuck_end_does_not_hang() {
    bounded(|| {
        let (mut device, _) = device();
        with_limit(&mut device, 1);
        for pos in 1..9 {
            device.enable_channel(pos, false).unwrap();
        }
        let (tx, rx) = sync_channel(1);
        device.start(PacketSink::new(tx, 0)).unwrap();
        // The header out, and the reading fills it: the end has no room, and the worker
        // waits with its sink.
        rx.recv_timeout(Duration::from_secs(30)).unwrap();
        let deadline = Instant::now() + Duration::from_secs(30);
        while device.worker.as_ref().unwrap().running.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        let (tx2, rx2) = sync_channel(64);
        device.start(PacketSink::new(tx2, 0)).unwrap();
        // The old worker is gone, and its sink with it.
        // The reading that filled the queue is all it holds: the end was given up, and
        // reading it can't have freed anything, as the join came first.
        assert!(matches!(rx.recv_timeout(Duration::from_secs(30)).unwrap().1, Packet::Analog(_)));
        assert_eq!(
            rx.recv_timeout(Duration::from_secs(30)).unwrap_err(),
            std::sync::mpsc::RecvTimeoutError::Disconnected
        );
        let packets = collect(&rx2);
        assert_eq!(readings(&packets).len(), 1);
        device.close().unwrap();
    });
}

/// A second start while one runs is refused, and the first goes on.
#[test]
fn a_start_while_running_is_busy() {
    bounded(|| {
        let (mut device, _) = device();
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        rx.recv_timeout(Duration::from_secs(30)).unwrap();
        let (tx2, _rx2) = sync_channel(4);
        assert!(matches!(device.start(PacketSink::new(tx2, 0)), Err(Error::Busy)));
        device.close().unwrap();
    });
}

/// A setting during a capture goes between readings, to its own output, and the
/// capture's next reading selects its own again.
#[test]
fn a_setting_during_a_capture_reaches_its_output() {
    let (mut device, state) = device();
    with_limit(&mut device, 1000);
    for pos in 1..9 {
        device.enable_channel(pos, false).unwrap();
    }
    clear(&state);
    let probe = Arc::clone(&state);
    bounded(move || {
        let (tx, rx) = sync_channel(4);
        device.start(PacketSink::new(tx, 0)).unwrap();
        rx.recv_timeout(Duration::from_secs(30)).unwrap();
        rx.recv_timeout(Duration::from_secs(30)).unwrap();
        device.set(&Key::VoltageTarget, &Value::F64(3.3), Some("2")).unwrap();
        let reader = std::thread::spawn(move || while rx.recv().is_ok() {});
        let deadline = Instant::now() + Duration::from_secs(30);
        loop {
            let w = written(&probe);
            let set = w.iter().position(|l| l == ":SOUR:VOLT 3.300000");
            if let Some(at) = set
                && w.len() > at + 2
            {
                assert_eq!(w[at - 1], ":INST:NSEL 2");
                assert_eq!(w[at + 1], ":INST:NSEL 1");
                assert_eq!(w[at + 2], ":MEAS:VOLT?");
                break;
            }
            assert!(Instant::now() < deadline, "{w:?}");
            std::thread::sleep(Duration::from_millis(1));
        }
        device.close().unwrap();
        reader.join().unwrap();
    });
}

fn hp66b(
    l: &str,
    stb: &'static str,
    ques: (&'static str, &'static str),
    oper: (&'static str, &'static str),
) -> Vec<(u32, String)> {
    let text = match l {
        "*IDN?" => "HEWLETT-PACKARD,6632B,0,A.01\n".to_owned(),
        "*STB?" => stb.to_owned(),
        "STAT:QUES:EVEN?" => ques.0.to_owned(),
        "STAT:QUES:COND?" => ques.1.to_owned(),
        "STAT:OPER:EVEN?" => oper.0.to_owned(),
        "STAT:OPER:COND?" => oper.1.to_owned(),
        "OUTP:STAT?" => "0\n".to_owned(),
        ":MEAS:VOLT?" | ":MEAS:CURR?" => "+1.000000E+00\n".to_owned(),
        _ => return Vec::new(),
    };
    vec![(0, text)]
}

fn metas(packets: &[Packet]) -> Vec<(Key, Value)> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Meta(items) => Some(items[0].clone()),
            _ => None,
        })
        .collect()
}

fn one_round(respond: Instrument) -> (Vec<Packet>, Vec<String>) {
    let (mut device, state) = device_with(respond, false, false);
    with_limit(&mut device, 1);
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    (packets, written(&state))
}

#[test]
fn an_hp_66xxb_status() {
    // OVP and OCP changed, only OVP on, and the output went off; CC now.
    let respond = |l: &str| hp66b(l, "136\n", ("3\n", "1\n"), ("1024\n", "1024\n"));
    let (packets, asked) = one_round(Box::new(respond));
    assert_eq!(
        metas(&packets),
        [
            (Key::OverVoltageProtectionActive, Value::Bool(true)),
            (Key::OverCurrentProtectionActive, Value::Bool(false)),
            (Key::Enabled, Value::Bool(false)),
            (Key::Regulation, Value::Str("CC".to_owned())),
        ]
    );
    assert!(asked.contains(&"STAT:OPER:PTR 3328;NTR 3328;ENAB 3328".to_owned()));
    assert!(asked.contains(&"STAT:QUES:PTR 1043;NTR 1043;ENAB 1043".to_owned()));
    let even = asked.iter().position(|l| l == "STAT:QUES:EVEN?").unwrap();
    assert_eq!(asked[even + 1], "STAT:QUES:COND?", "the event register first");
}

#[test]
fn an_hp_66xxb_with_both_regulations_says_nothing_of_them() {
    let respond = |l: &str| hp66b(l, "128\n", ("0\n", "0\n"), ("1280\n", "1280\n"));
    let (packets, _) = one_round(Box::new(respond));
    assert_eq!(metas(&packets), []);
}

#[test]
fn an_hp_66xxb_unregulated_and_a_status_that_fails() {
    let respond = |l: &str| hp66b(l, "8\n", ("1024\n", "1024\n"), ("0\n", "0\n"));
    let (packets, _) = one_round(Box::new(respond));
    assert!(metas(&packets).contains(&(Key::Regulation, Value::Str("UR".to_owned()))));
    // A status that's no number: the reading goes on, as C's result is dropped.
    let respond = |l: &str| hp66b(l, "junk\n", ("0\n", "0\n"), ("0\n", "0\n"));
    let (packets, _) = one_round(Box::new(respond));
    assert_eq!(metas(&packets), []);
    assert_eq!(readings(&packets).len(), 2);
}

#[test]
fn an_ea_status() {
    fn respond(l: &str, ques: &'static str, oper: &'static str) -> Vec<(u32, String)> {
        let text = match l {
            "*IDN?" => "EA Elektro-Automatik GmbH & Co. KG, PS 2084-05B, 1, 2\n",
            ":SYST:NOM:VOLT?" => "84.00 V\n",
            ":SYST:NOM:CURR?" => "5.00 A\n",
            ":SYST:NOM:POW?" => "160.00 W\n",
            "STAT:QUES?" => ques,
            "STAT:OPER?" => oper,
            ":MEAS:VOLT?" | ":MEAS:CURR?" | ":MEAS:POW?" => "1.00\n",
            _ => return Vec::new(),
        };
        vec![(0, text.to_owned())]
    }
    let (packets, _) = one_round(Box::new(|l| respond(l, "11\n", "256\n")));
    assert_eq!(
        metas(&packets),
        [
            (Key::OverVoltageProtectionActive, Value::Bool(true)),
            (Key::OverCurrentProtectionActive, Value::Bool(true)),
            (Key::OverTemperatureProtectionActive, Value::Bool(true)),
            (Key::Regulation, Value::Str("CV".to_owned())),
        ]
    );
    let (packets, _) = one_round(Box::new(|l| respond(l, "0\n", "512\n")));
    assert_eq!(metas(&packets), [(Key::Regulation, Value::Str("CC".to_owned()))]);
    let (packets, _) = one_round(Box::new(|l| respond(l, "0\n", "768\n")));
    assert_eq!(metas(&packets), []);
}

#[test]
fn an_hp_6630a_status() {
    let device = |fault: &'static str| {
        move |l: &str| match l {
            "ID?" => vec![(0, "HP6632A\n".to_owned())],
            "ROM?" => vec![(0, "x\n".to_owned())],
            "FAULT?" => vec![(0, fault.to_owned())],
            "VOUT?" | "IOUT?" => vec![(0, "1.000\n".to_owned())],
            _ => Vec::new(),
        }
    };
    let run = |respond: Instrument| {
        let (mut d, state) = device_with(respond, false, true);
        with_limit(&mut d, 1);
        let packets = bounded(move || {
            let (tx, rx) = sync_channel(64);
            d.start(PacketSink::new(tx, 0)).unwrap();
            let packets = collect(&rx);
            d.close().unwrap();
            packets
        });
        (packets, written(&state))
    };
    let (packets, asked) = run(Box::new(device("88\n")));
    assert_eq!(
        metas(&packets),
        [
            (Key::OverVoltageProtectionActive, Value::Bool(true)),
            (Key::OverCurrentProtectionActive, Value::Bool(true)),
            (Key::OverTemperatureProtectionActive, Value::Bool(true)),
        ]
    );
    assert_eq!(&asked[..3], ["ID?", "ROM?", "UNMASK 607"]);
    for (fault, word) in [("1\n", "CV"), ("2\n", "CC"), ("512\n", "CC-"), ("4\n", "UR")] {
        let (packets, _) = run(Box::new(device(fault)));
        assert_eq!(metas(&packets), [(Key::Regulation, Value::Str(word.to_owned()))], "{fault}");
    }
    let (packets, _) = run(Box::new(device("3\n")));
    assert_eq!(metas(&packets), []);
}

/// A clock-paced reply: the fake's own time is moved by the reads that find nothing, so
/// a real reading rate holds under a slowed clock.
#[test]
fn a_time_limit_takes_the_answer_to_the_question_out() {
    let respond = |l: &str| match l {
        ":MEAS:VOLT?" => vec![(20, "12.5\n".to_owned())],
        other => dp832(other),
    };
    let (mut device, state) = device_with(Box::new(respond), false, false);
    device.set(&Key::LimitMsec, &Value::U64(1), None).unwrap();
    for pos in 1..9 {
        device.enable_channel(pos, false).unwrap();
    }
    clear(&state);
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    // A time limit shorter than the device's gap still gets its reading, as C does.
    assert_eq!(readings(&packets), [(0, 12.5)]);
    assert_eq!(failure(&packets), None);
}

/// An answer cut short by more than a read's timeout: its tail must not read as an
/// answer of its own, here a 5 where the instrument said 12.5.
#[test]
fn the_tail_of_an_answer_cut_short_is_no_reading() {
    let mut first = true;
    let respond = move |l: &str| match l {
        ":MEAS:VOLT?" if first => {
            first = false;
            vec![(0, "12.".to_owned()), (1, "5\n".to_owned())]
        }
        other => dp832(other),
    };
    let (mut device, _) = device_with(Box::new(respond), false, false);
    with_limit(&mut device, 1);
    for pos in 1..9 {
        device.enable_channel(pos, false).unwrap();
    }
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    assert_eq!(readings(&packets), [(0, 12.5)]);
}

/// `sr_scpi_source_add`'s 10 ms: readings no quicker than that, from the log's stamps,
/// which sleeps running long can only widen.
#[test]
fn a_capture_is_paced() {
    let (mut device, state) = device();
    with_limit(&mut device, 6);
    for pos in 1..9 {
        device.enable_channel(pos, false).unwrap();
    }
    let probe = Arc::clone(&state);
    bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        assert_eq!(readings(&collect(&rx)).len(), 6);
        device.close().unwrap();
    });
    let s = probe.lock().unwrap();
    let stamps: Vec<Instant> =
        s.written.iter().filter(|(_, w)| w == b":MEAS:VOLT?").map(|(at, _)| *at).collect();
    assert_eq!(stamps.len(), 6);
    for pair in stamps.windows(2) {
        assert!(pair[1] - pair[0] >= Duration::from_millis(9), "{:?}", pair[1] - pair[0]);
    }
}

/// `profiles.c:82-83` has these with a question mark.
#[test]
fn an_n5700_sets_its_ocp_with_no_question_mark() {
    let respond = |l: &str| match l {
        "*IDN?" => vec![(0, "Agilent Technologies,N5767A,1,2\n".to_owned())],
        _ => Vec::new(),
    };
    let (mut device, state) = device_with(Box::new(respond), false, false);
    clear(&state);
    device.set(&Key::OverCurrentProtectionEnabled, &Value::Bool(true), Some("1")).unwrap();
    device.set(&Key::OverCurrentProtectionEnabled, &Value::Bool(false), Some("1")).unwrap();
    assert_eq!(written(&state), [":CURR:PROT:STAT ON", ":CURR:PROT:STAT OFF"]);
}

/// An HMP's status names its output in the query, and nothing is selected; an OVP that
/// is always on asks nothing.
#[test]
fn an_hmp_asks_by_its_output() {
    let respond = |l: &str| match l {
        "*IDN?" => vec![(0, "HAMEG,HMP4040,1,2\n".to_owned())],
        "STAT:QUES:INST:ISUM3:COND?" => vec![(0, "18\n".to_owned())],
        _ => Vec::new(),
    };
    let (device, state) = device_with(Box::new(respond), false, false);
    clear(&state);
    assert_eq!(device.get(&Key::Regulation, Some("3")).unwrap(), Value::Str("CV".to_owned()));
    assert_eq!(
        device.get(&Key::OverTemperatureProtectionActive, Some("3")).unwrap(),
        Value::Bool(true)
    );
    assert_eq!(
        device.get(&Key::OverVoltageProtectionEnabled, Some("3")).unwrap(),
        Value::Bool(true)
    );
    assert_eq!(written(&state), ["STAT:QUES:INST:ISUM3:COND?", "STAT:QUES:INST:ISUM3:COND?"]);
}

/// Only with more than one group does a capture select, as `protocol.c:64`.
#[test]
fn one_group_selects_nothing_in_a_capture_and_does_in_a_get() {
    let respond = |l: &str| match l {
        "*IDN?" => vec![(0, "RIGOL TECHNOLOGIES,DP711,1,2\n".to_owned())],
        "SYST:BEEP:STAT?" => vec![(0, "0\n".to_owned())],
        l if l.ends_with('?') => vec![(0, "1.0\n".to_owned())],
        _ => Vec::new(),
    };
    let (mut device, state) = device_with(Box::new(respond), false, false);
    device.get(&Key::Voltage, Some("1")).unwrap();
    device.close().unwrap();
    device.open().unwrap();
    clear(&state);
    with_limit(&mut device, 1);
    let probe = Arc::clone(&state);
    bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        collect(&rx);
        device.close().unwrap();
    });
    let w = written(&probe);
    assert!(!w.iter().any(|l| l.starts_with(":INST")), "{w:?}");
}

/// A close joins: a thread in a read only the close can end must be gone when it comes
/// back, holding nothing. Looks before anything is drained.
#[test]
fn a_stop_joins_the_thread_in_its_read() {
    bounded(|| {
        let (scpi, _) = line(Box::new(silent_after_idn), false);
        let mut device = PpsDevice::from_scpi(scpi, false).unwrap();
        device.open().unwrap();
        device.closed = None;
        device.link().scpi.as_mut().unwrap().read_timeout = Duration::from_millis(300);
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        rx.recv_timeout(Duration::from_secs(30)).unwrap();
        std::thread::sleep(Duration::from_millis(50));
        device.stop().unwrap();
        assert_eq!(Arc::strong_count(&device.link), 1);
    });
}

/// A full queue at a reading, past the header: a stop must end it there as well.
#[test]
fn a_stop_ends_a_capture_blocked_on_a_reading() {
    let (mut device, state) = device();
    for pos in 1..9 {
        device.enable_channel(pos, false).unwrap();
    }
    bounded(move || {
        // The header and one reading fit, and nothing reads them.
        let (tx, rx) = sync_channel(2);
        clear(&state);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + Duration::from_secs(30);
        while written(&state).iter().filter(|l| *l == ":MEAS:VOLT?").count() < 2 {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        device.stop().unwrap();
        device.close().unwrap();
        drop(rx);
    });
}

/// The start of a round asks for a status once, not again for each read of the same
/// question still out.
#[test]
fn a_question_out_asks_no_status() {
    let mut first = true;
    let respond = move |l: &str| match l {
        ":MEAS:VOLT?" if first => {
            first = false;
            vec![(5, "+1.000000E+00\n".to_owned())]
        }
        other => hp66b(other, "0\n", ("0\n", "0\n"), ("0\n", "0\n")),
    };
    let (mut device, state) = device_with(Box::new(respond), false, false);
    with_limit(&mut device, 1);
    device.enable_channel(1, false).unwrap();
    clear(&state);
    let probe = Arc::clone(&state);
    bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        assert_eq!(readings(&collect(&rx)).len(), 1);
        device.close().unwrap();
    });
    let asked = written(&probe);
    assert_eq!(asked.iter().filter(|l| *l == "*STB?").count(), 1, "{asked:?}");
}

/// A question no answer ever comes to is asked again in the end, on a line with no tags.
#[test]
fn a_question_lost_on_a_line_with_no_tags_is_asked_again() {
    let mut first = true;
    let respond = move |l: &str| match l {
        ":MEAS:VOLT?" if first => {
            first = false;
            Vec::new()
        }
        other => dp832(other),
    };
    let (mut device, state) = device_with(Box::new(respond), false, false);
    with_limit(&mut device, 1);
    for pos in 1..9 {
        device.enable_channel(pos, false).unwrap();
    }
    clear(&state);
    let probe = Arc::clone(&state);
    bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        assert_eq!(readings(&collect(&rx)), [(0, 12.5)]);
        device.close().unwrap();
    });
    let asked = written(&probe);
    assert_eq!(asked.iter().filter(|l| *l == ":MEAS:VOLT?").count(), 2, "{asked:?}");
}

/// A setting gives up what was owed: the send drains it, so nothing is left to wait for.
#[test]
fn a_setting_gives_up_the_question_out() {
    let (device, _) = device_with(Box::new(silent_after_idn), false, false);
    let mut link = device.link();
    assert_eq!(link.measure(None, Cmd::GetMeasVoltage).unwrap(), Reading::Silent);
    assert!(link.owed());
    link.cmd(None, Cmd::SetOutputEnable, &[]).unwrap();
    assert!(!link.owed());
}

/// A read that fails with what a device sent is no reading, and not the end.
#[test]
fn a_bad_answer_is_asked_again() {
    let (mut device, state) = device();
    with_limit(&mut device, 1);
    for pos in 1..9 {
        device.enable_channel(pos, false).unwrap();
    }
    state.lock().unwrap().bad_reads = 2;
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    assert_eq!(readings(&packets), [(0, 12.5)]);
    assert_eq!(failure(&packets), None);
}

/// A late answer that came clears the line's doubt: the next send has nothing to drain.
#[test]
fn a_late_answer_leaves_no_drain() {
    let mut first = true;
    let respond = move |l: &str| match l {
        ":MEAS:VOLT?" if first => {
            first = false;
            vec![(3, "12.5\n".to_owned())]
        }
        other => dp832(other),
    };
    let (mut device, state) = device_with(Box::new(respond), false, false);
    with_limit(&mut device, 2);
    for pos in 1..9 {
        device.enable_channel(pos, false).unwrap();
    }
    let probe = Arc::clone(&state);
    bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        assert_eq!(readings(&collect(&rx)).len(), 2);
        device.close().unwrap();
    });
    assert_eq!(probe.lock().unwrap().discards, 0);
}

/// A DP832 that sends its second voltage in two pieces, the tail `wait` reads later.
fn split_second_volts(wait: u32) -> impl FnMut(&str) -> Vec<(u32, String)> + Send {
    let mut volts = 0;
    move |l: &str| match l {
        ":MEAS:VOLT?" => {
            volts += 1;
            if volts == 2 {
                vec![(0, "12.".to_owned()), (wait, "345\n".to_owned())]
            } else {
                vec![(0, "12.5\n".to_owned())]
            }
        }
        other => dp832(other),
    }
}

fn capture_of(device: &mut PpsDevice, rounds: u64) -> Vec<Packet> {
    with_limit(device, rounds);
    for pos in [2, 3, 4, 5, 6, 7, 8] {
        device.enable_channel(pos, false).unwrap();
    }
    let (tx, rx) = sync_channel(256);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let packets = collect(&rx);
    device.stop().unwrap();
    packets
}

/// An answer cut in two, on a line with no tags: as a read that gives up on its head
/// (a tty), and as a short read that ends it (C's tcp-raw rule). Each reading after it
/// is of its own quantity, and the line is found again by `*IDN?`.
#[test]
fn an_answer_cut_in_two_shifts_nothing() {
    for (short_ends, wait) in [(false, 3), (false, 20), (true, 3), (true, 20)] {
        let (mut device, state) = device_with(Box::new(split_second_volts(wait)), false, false);
        state.lock().unwrap().short_ends = short_ends;
        clear(&state);
        let probe = Arc::clone(&state);
        let packets = bounded(move || {
            let packets = capture_of(&mut device, 6);
            device.close().unwrap();
            packets
        });
        let read = readings(&packets);
        assert_eq!(read.len(), 12, "{short_ends} {wait}: {read:?}");
        for (channel, value) in read {
            let want = if channel == 0 { 12.5 } else { 1.5 };
            assert_eq!(value, want, "{short_ends} {wait}: channel {channel}");
        }
        assert!(written(&probe).contains(&"*IDN?".to_owned()), "{short_ends} {wait}");
    }
}

/// A get whose answer came cut short is an error, and the next get is its own.
#[test]
fn a_get_cut_short_is_an_error_and_the_next_is_whole() {
    let mut first = true;
    let respond = move |l: &str| match l {
        ":SOUR:VOLT?" if first => {
            first = false;
            vec![(0, "7.".to_owned()), (3, "25\n".to_owned())]
        }
        ":SOUR:VOLT?" => vec![(0, "7.5\n".to_owned())],
        other => dp832(other),
    };
    let (device, _) = device_with(Box::new(respond), false, false);
    let e = device.get(&Key::VoltageTarget, Some("1")).unwrap_err();
    assert!(e.to_string().contains("cut short"), "{e}");
    assert_eq!(device.get(&Key::VoltageTarget, Some("1")).unwrap(), Value::F64(7.5));
}

/// A question an earlier capture left out is given up at the next start: its answer is
/// drained, not filed under the new capture's first channel.
#[test]
fn a_question_left_out_by_a_capture_is_given_up_at_the_next() {
    let mut first = true;
    let respond = move |l: &str| match l {
        ":MEAS:VOLT?" if first => {
            first = false;
            vec![(1_000_000, "99\n".to_owned())]
        }
        other => dp832(other),
    };
    let (mut device, state) = device_with(Box::new(respond), false, false);
    for pos in 1..9 {
        device.enable_channel(pos, false).unwrap();
    }
    let probe = Arc::clone(&state);
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        // Until the question is out and waited on.
        let deadline = Instant::now() + Duration::from_secs(30);
        while !device.link().owed() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        device.stop().unwrap();
        drop(rx);
        // Its answer has come by the next start.
        probe.lock().unwrap().coming.front_mut().unwrap().0 = 0;
        with_limit(&mut device, 1);
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    assert_eq!(readings(&packets), [(0, 12.5)]);
}

#[test]
fn a_set_point_against_its_range() {
    use super::Range::{Over, Refused, Unknown, Within};
    let cases = [
        ((0.0, 30.0, 15.0), Within),
        ((0.0, 30.0, 32.0), Over),
        ((0.0, 30.0, 33.0), Over),
        ((0.0, 30.0, 33.01), Refused),
        ((0.0, 30.0, 120.0), Refused),
        ((0.0, 30.0, -0.5), Refused),
        ((1e-3, 1.03, 0.0), Refused),
        ((-30.0, 0.0, -32.0), Over),
        ((-30.0, 0.0, -34.0), Refused),
        ((-30.0, 0.0, 1.0), Refused),
        ((0.0, 0.0, 5.0), Unknown),
    ];
    for ((low, high, value), want) in cases {
        assert_eq!(super::range_check(low, high, value), want, "{value} in {low}..{high}");
    }
}

/// Far past a range is refused before the wire, by `check_set` for a command line and
/// by `set` for a library; a little past it goes with a warning; a PM2800 output with no
/// module lists none, and goes.
#[test]
fn a_set_point_far_out_of_range_goes_nowhere() {
    let (mut device, state) = device();
    clear(&state);
    let far = Value::F64(120.0);
    assert!(device.check_set(&Key::VoltageTarget, &far, Some("1")).is_err());
    let e = device.set(&Key::VoltageTarget, &far, Some("1")).unwrap_err();
    assert!(e.to_string().contains("nothing was sent"), "{e}");
    assert!(device.check_set(&Key::CurrentLimit, &Value::F64(-1.0), Some("1")).is_err());
    assert_eq!(written(&state), Vec::<String>::new());
    device.set(&Key::VoltageTarget, &Value::F64(32.0), Some("1")).unwrap();
    device.set(&Key::CurrentLimit, &Value::F64(3.2), Some("1")).unwrap();
    assert_eq!(written(&state), [":INST:NSEL 1", ":SOUR:VOLT 32.000000", ":SOUR:CURR 3.200000"]);
    let respond = |l: &str| match l {
        "*IDN?" => vec![(0, "PHILIPS,PM2832/01,1,2\n".to_owned())],
        _ => Vec::new(),
    };
    let (mut pm, state) = device_with(Box::new(respond), false, false);
    clear(&state);
    pm.set(&Key::VoltageTarget, &Value::F64(5.0), Some("2")).unwrap();
    assert_eq!(written(&state), [":INST:NSEL 2", ":SOUR:VOLT 5.000000"]);
}

#[test]
fn a_sweep_sets_its_bound() {
    assert_eq!(protocol::sweep_bound(2048.0, 15.6e-6), None, "the usual is no longer");
    assert_eq!(
        protocol::sweep_bound(4096.0, 0.0312),
        Some(Duration::from_secs_f64(2.0 * 4096.0 * 0.0312)),
        "twice 128 s is past 180"
    );
    assert_eq!(protocol::sweep_bound(4096.0, 0.02), None, "twice 82 s is within it");
    assert_eq!(protocol::sweep_bound(4096.0, 0.1), Some(Duration::from_secs_f64(819.2)));
    assert_eq!(protocol::sweep_bound(f64::NAN, 1.0), None);
    assert_eq!(protocol::sweep_bound(0.0, 1.0), None);
    assert!(protocol::sweep_bound(4096.0, 31200.0).is_some());
}

/// An HP 66xxB says its sweep at a capture's start, and a long one gives a question its
/// time: an answer later than the usual patience is still the one asked for.
#[test]
fn an_hp_66xxb_waits_as_long_as_its_sweep() {
    let mut first = true;
    let respond = move |l: &str| match l {
        "SENS:SWE:POIN?" => vec![(0, "+4.096000E+03\n".to_owned())],
        "SENS:SWE:TINT?" => vec![(0, "+1.000000E-01\n".to_owned())],
        ":MEAS:VOLT?" if first => {
            first = false;
            vec![(60, "+1.000000E+00\n".to_owned())]
        }
        other => hp66b(other, "0\n", ("0\n", "0\n"), ("0\n", "0\n")),
    };
    let (mut device, state) = device_with(Box::new(respond), false, false);
    with_limit(&mut device, 1);
    device.enable_channel(1, false).unwrap();
    clear(&state);
    let probe = Arc::clone(&state);
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        let packets = collect(&rx);
        device.close().unwrap();
        packets
    });
    assert_eq!(readings(&packets), [(0, 1.0)]);
    let asked = written(&probe);
    assert!(asked.contains(&"SENS:SWE:POIN?".to_owned()), "{asked:?}");
    assert_eq!(asked.iter().filter(|l| *l == ":MEAS:VOLT?").count(), 1, "{asked:?}");
}

/// A long sweep raises the silence bound too: a capture silent past the usual bound
/// goes on.
#[test]
fn a_long_sweep_raises_the_silence_bound() {
    let respond = |l: &str| match l {
        "SENS:SWE:POIN?" => vec![(0, "4096\n".to_owned())],
        "SENS:SWE:TINT?" => vec![(0, "0.1\n".to_owned())],
        ":MEAS:VOLT?" | ":MEAS:CURR?" => Vec::new(),
        other => hp66b(other, "0\n", ("0\n", "0\n"), ("0\n", "0\n")),
    };
    let (mut device, state) = device_with(Box::new(respond), true, false);
    device.watch = Some(watch(false, [20, 10_000, 10_000, 10_000]));
    let packets = bounded(move || {
        let (tx, rx) = sync_channel(64);
        device.start(PacketSink::new(tx, 0)).unwrap();
        // Well past 20 ms of silence, in reads.
        let deadline = Instant::now() + Duration::from_secs(30);
        while state.lock().unwrap().reads < 200 {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        device.stop().unwrap();
        let mut packets = Vec::new();
        while let Ok((_, p)) = rx.recv_timeout(Duration::from_secs(5)) {
            packets.push(p);
        }
        device.close().unwrap();
        packets
    });
    assert_eq!(failure(&packets), None);
}
