// Counterpart in sigrok:
//   libsigrok/src/hardware/hp-3478a/api.c,
//   libsigrok/src/hardware/hp-3478a/protocol.c,
//   Copyright (C) 2017-2021 Frank Stettner <frank-stettner@gmx.net>,
//   with commits by Gerhard Sittig, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver against a fake meter on a fake GPIB transport.

use super::*;
use crate::core::{Mq, MqFlags, Unit};
use crate::serial::WriteLog;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test may take before it counts as hung.
const BOUND: Duration = Duration::from_secs(10);

/// The fake meter.
#[derive(Default)]
struct Meter {
    writes: WriteLog,
    /// The five status bytes.
    status: [u8; 5],
    /// What the next read gets, set by a command.
    pending: Option<Vec<u8>>,
    /// Readings to give, in turn; when none is left, readings repeat the last.
    readings: VecDeque<String>,
    last: String,
    /// Polls per reading; 0 for never.
    every: u32,
    polls: u32,
    since: u32,
    ready: bool,
    /// The poll of this number fails.
    poll_fails_at: Option<u32>,
    /// The reading read of this number times out.
    read_times_out_at: Option<u32>,
    reads: u32,
    /// Status answers of this number (from 1) come with these error bits.
    errors_at: Vec<(u32, u8)>,
    /// The status answer is this many bytes.
    status_len: Option<usize>,
    statuses: u32,
    closed: bool,
    after_close: u32,
    /// The answer being read has ended.
    complete: bool,
    /// Set by the fake when a reading is delivered, for a stop in a capture.
    stop_on_reading: Option<Arc<AtomicBool>>,
}

impl Meter {
    fn command(&mut self, text: &str) {
        match text {
            "B" => {
                self.statuses += 1;
                let mut s = self.status.to_vec();
                if let Some((_, e)) = self.errors_at.iter().find(|(n, _)| *n == self.statuses) {
                    s[3] = *e;
                }
                if let Some(n) = self.status_len {
                    s.truncate(n);
                }
                self.pending = Some(s);
            }
            "F1" => self.status[0] = self.status[0] & 0x1f | 1 << 5,
            "F2" => self.status[0] = self.status[0] & 0x1f | 2 << 5,
            "F4" => self.status[0] = 0b1000_1101,
            "R1" => {
                self.status[0] = self.status[0] & 0xe3 | 4 << 2;
                self.status[1] &= !2;
            }
            "RA" => self.status[1] |= 2,
            "N4" => self.status[0] = self.status[0] & 0xfc | 2,
            _ => {}
        }
    }
}

#[derive(Clone)]
struct Fake(Arc<Mutex<Meter>>);

impl Fake {
    fn meter(&self) -> MutexGuard<'_, Meter> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn writes(&self) -> Vec<String> {
        self.meter()
            .writes
            .iter()
            .map(|(_, w)| String::from_utf8_lossy(w).trim_end().to_owned())
            .collect()
    }
}

impl ScpiTransport for Fake {
    fn resource(&self) -> &'static str {
        "gpib/0/23"
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        let mut m = self.meter();
        if m.closed {
            m.after_close += 1;
        }
        m.writes.push((Instant::now(), data.to_vec()));
        let text = String::from_utf8_lossy(data).trim_end().to_owned();
        m.command(&text);
        Ok(())
    }

    fn read_begin(&mut self) -> Result<()> {
        self.meter().complete = false;
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut m = self.meter();
        let bytes = if let Some(p) = m.pending.take() {
            p
        } else {
            m.reads += 1;
            // From this read on the meter says nothing, and a timeout takes its time,
            // as on the GPIB transport.
            if m.read_times_out_at.is_some_and(|at| m.reads >= at) {
                drop(m);
                std::thread::sleep(timeout);
                return Err(Error::Timeout);
            }
            if let Some(next) = m.readings.pop_front() {
                m.last = next;
            }
            m.ready = false;
            m.since = 0;
            if let Some(stop) = m.stop_on_reading.take() {
                stop.store(true, Ordering::SeqCst);
            }
            format!("{}\r\n", m.last).into_bytes()
        };
        buf[..bytes.len()].copy_from_slice(&bytes);
        m.complete = true;
        Ok(bytes.len())
    }

    fn read_complete(&self) -> bool {
        self.meter().complete
    }

    fn close(&mut self) -> Result<()> {
        self.meter().closed = true;
        Ok(())
    }

    fn reopen(&mut self) -> Result<()> {
        self.meter().closed = false;
        Ok(())
    }

    fn serial_poll(&mut self) -> Result<u8> {
        let mut m = self.meter();
        m.polls += 1;
        if m.poll_fails_at == Some(m.polls) {
            return Err(Error::device("GPIB: the serial poll of address 23 got no answer"));
        }
        if m.every > 0 && !m.ready {
            m.since += 1;
            m.ready = m.since >= m.every;
        }
        Ok(u8::from(m.ready))
    }
}

/// DCV 3 V autorange 5½, internal trigger, autozero, front: the meter at power on.
const POWER_ON: [u8; 5] = [0b0010_1101, 0b0001_0111, 0, 0, 33];

fn fake() -> Fake {
    let m =
        Meter { status: POWER_ON, last: "+1.23456E+0".to_owned(), every: 1, ..Meter::default() };
    Fake(Arc::new(Mutex::new(m)))
}

fn device(fake: &Fake) -> Hp3478aDevice {
    Hp3478aDevice::from_transport(Box::new(fake.clone()))
}

/// Runs a test on a thread with a bound: a stop that does not bite must fail.
fn bounded<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    rx.recv_timeout(BOUND * 2).expect("the test did not end in time")
}

fn sink(depth: usize) -> (PacketSink, Receiver<(usize, Packet)>) {
    let (tx, rx) = sync_channel(depth);
    (PacketSink::new(tx, 0), rx)
}

/// Packets up to the End, with a bound and a cap.
fn drain(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let began = Instant::now();
    let mut out = Vec::new();
    while began.elapsed() < BOUND && out.len() < 10_000 {
        match rx.recv_timeout(BOUND) {
            Ok((_, Packet::End)) => {
                out.push(Packet::End);
                return out;
            }
            Ok((_, p)) => out.push(p),
            Err(_) => break,
        }
    }
    panic!("no End within the bound: {} packets", out.len());
}

fn analog(packets: &[Packet]) -> Vec<AnalogData> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some(a.clone()),
            _ => None,
        })
        .collect()
}

fn failed(packets: &[Packet]) -> Option<String> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(text) => Some(text.clone()),
        _ => None,
    })
}

/// A whole capture: open, limits, start, packets, close.
fn capture(d: &mut Hp3478aDevice, samples: u64, msec: u64) -> Vec<Packet> {
    d.open().unwrap();
    d.set(&Key::LimitSamples, &Value::U64(samples), None).unwrap();
    d.set(&Key::LimitMsec, &Value::U64(msec), None).unwrap();
    let (s, rx) = sink(64);
    d.start(s).unwrap();
    let packets = drain(&rx);
    d.close().unwrap();
    packets
}

#[test]
fn a_probe_asks_the_status_and_closes() {
    let f = fake();
    let d = probe(Scpi::new(Box::new(f.clone()))).expect("a meter");
    assert_eq!((d.info.vendor.as_str(), d.info.model.as_str()), ("Hewlett-Packard", "3478A"));
    assert_eq!(d.channels[0].name, "P1");
    assert_eq!(f.writes(), ["B"]);
    assert!(f.meter().closed);
}

#[test]
fn a_meter_with_its_error_register_set_is_no_device() {
    let f = fake();
    f.meter().errors_at = vec![(1, 0x20)];
    assert!(probe(Scpi::new(Box::new(f.clone()))).is_none());
    // A status that says no function is no meter either.
    let g = fake();
    g.meter().status[0] = 0x0d;
    assert!(probe(Scpi::new(Box::new(g.clone()))).is_none());
    let h = fake();
    h.meter().status_len = Some(3);
    assert!(probe(Scpi::new(Box::new(h.clone()))).is_none());
}

#[test]
fn a_capture_polls_reads_and_labels_as_the_status_says() {
    let f = fake();
    {
        let mut m = f.meter();
        m.every = 2;
        m.readings = ["+1.23456E+0", "-0.00012E+0", "+9.99999E+9"].map(str::to_owned).into();
    }
    let packets = bounded(move || {
        capture(&mut device(&f), 3, 0).into_iter().map(|p| (p, f.clone())).collect::<Vec<_>>()
    });
    let f = packets[0].1.clone();
    let packets: Vec<Packet> = packets.into_iter().map(|(p, _)| p).collect();
    assert!(matches!(packets[0], Packet::Header { .. }), "{:?}", packets[0]);
    assert_eq!(failed(&packets), None);
    let a = analog(&packets);
    assert_eq!(
        a.iter().map(|a| a.samples[0]).collect::<Vec<_>>(),
        [1.23456, -0.00012, f32::INFINITY]
    );
    assert_eq!((a[0].mq, a[0].unit, a[0].digits), (Mq::Voltage, Unit::Volt, 5));
    assert_eq!(a[0].flags, MqFlags::DC | MqFlags::AUTORANGE);
    assert_eq!(a[0].channels, [0]);
    // T1 and B, then a B after each reading, then T1 and D1.
    assert_eq!(f.writes(), ["T1", "B", "B", "B", "B", "T1", "D1"]);
    assert_eq!(f.meter().polls, 6, "two polls per reading");
    assert_eq!(f.meter().after_close, 0);
}

#[test]
fn a_reading_that_is_no_number_fails_the_capture() {
    let f = fake();
    f.meter().readings = ["+1.2X456E+0".to_owned()].into();
    let ff = f.clone();
    let packets = bounded(move || capture(&mut device(&ff), 3, 0));
    let why = failed(&packets).expect("failed");
    assert_eq!(why, "device error: The meter's reading is no number: '+1.2X456E+0'.");
    assert_eq!(analog(&packets).len(), 0);
    assert_eq!(f.writes(), ["T1", "B"], "no T1 and D1 after a failure, as in C");
}

#[test]
fn errors_in_the_status_after_a_reading_fail_the_capture() {
    let f = fake();
    f.meter().errors_at = vec![(2, 0x08)];
    let ff = f.clone();
    let packets = bounded(move || capture(&mut device(&ff), 3, 0));
    assert_eq!(
        failed(&packets).as_deref(),
        Some("device error: There has been an A/D slope error")
    );
    assert_eq!(analog(&packets).len(), 0, "the reading goes with its status");
}

#[test]
fn a_failed_poll_or_read_fails_the_capture() {
    let f = fake();
    f.meter().poll_fails_at = Some(2);
    let ff = f.clone();
    let packets = bounded(move || capture(&mut device(&ff), 3, 0));
    assert!(failed(&packets).unwrap().contains("serial poll of address 23 got no answer"));
    assert_eq!(analog(&packets).len(), 1);
    let g = fake();
    g.meter().read_times_out_at = Some(1);
    let gg = g.clone();
    let packets = bounded(move || {
        // A short wait, which the fake's sleep then takes in full: 3 s under the
        // sleep shim's x12 would pass the bound.
        let mut d = device(&gg);
        d.link().closed.as_mut().unwrap().read_timeout = Duration::from_millis(200);
        capture(&mut d, 3, 0)
    });
    assert_eq!(failed(&packets).as_deref(), Some("timeout"));
}

#[test]
fn a_meter_never_ready_fails_and_one_gone_quiet_warns_first() {
    let short = Silence {
        first: Duration::from_millis(350),
        warn: Duration::from_millis(350),
        give_up: Duration::from_millis(900),
    };
    let f = fake();
    f.meter().every = 0;
    let ff = f.clone();
    let packets =
        bounded(move || capture(&mut Hp3478aDevice::with_silence(Box::new(ff), short), 0, 0));
    assert!(failed(&packets).unwrap().contains("The meter had no reading ready in 0 s"));
    // One reading, then none: the bound after it is the long one.
    let g = fake();
    let gg = g.clone();
    let packets = bounded(move || {
        let mut d = Hp3478aDevice::with_silence(Box::new(gg.clone()), short);
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        // The first reading in, the meter goes quiet.
        let first = rx.recv_timeout(BOUND).unwrap();
        let second = rx.recv_timeout(BOUND).unwrap();
        gg.meter().every = 0;
        let mut packets = vec![first.1, second.1];
        packets.extend(drain(&rx));
        d.close().unwrap();
        packets
    });
    let why = failed(&packets).unwrap();
    assert!(why.contains("The meter has had no reading ready for 0 s."), "{why}");
    assert_ne!(analog(&packets).len(), 0);
}

#[test]
fn a_time_limit_ends_a_capture_with_no_reading() {
    let f = fake();
    f.meter().every = 0;
    let ff = f.clone();
    let began = Instant::now();
    let packets = bounded(move || capture(&mut device(&ff), 0, 250));
    assert_eq!(failed(&packets), None);
    assert!(began.elapsed() >= Duration::from_millis(250));
    assert_eq!(f.writes().last().map(String::as_str), Some("D1"));
}

#[test]
fn a_stop_ends_the_capture_and_the_meter_goes_back() {
    let f = fake();
    let stop_seen = Arc::new(AtomicBool::new(false));
    f.meter().stop_on_reading = Some(Arc::clone(&stop_seen));
    let ff = f.clone();
    let packets = bounded(move || {
        let mut d = device(&ff);
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        while !stop_seen.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        let packets = drain(&rx);
        d.close().unwrap();
        packets
    });
    assert_eq!(failed(&packets), None);
    let w = f.writes();
    assert_eq!(w[w.len() - 2..], ["T1", "D1"]);
    assert_eq!(f.meter().after_close, 0);
}

#[test]
fn settings_go_out_only_when_they_change_things() {
    let f = fake();
    let mut d = device(&f);
    d.open().unwrap();
    // Nothing known: a status first, then nothing for what is so.
    d.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Voltage, MqFlags::DC), None).unwrap();
    assert_eq!(f.writes(), ["B"]);
    d.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Voltage, MqFlags::AC), None).unwrap();
    d.set(&Key::Range, &Value::Str("Auto".into()), None).unwrap();
    d.set(&Key::Range, &Value::Str("30V".into()), None).unwrap();
    d.set(&Key::Digits, &Value::Str("4.5".into()), None).unwrap();
    d.set(&Key::Digits, &Value::Str("4.5".into()), None).unwrap();
    assert_eq!(f.writes(), ["B", "F2", "B", "R1", "B", "N4", "B"]);
    for (key, value) in [
        (Key::Range, Value::Str("30mV".into())),
        (Key::Digits, Value::Str("6.5".into())),
        (Key::Digits, Value::Str(String::new())),
        (Key::MeasuredQuantity, Value::Mq(Mq::Voltage, MqFlags::empty())),
        (Key::MeasuredQuantity, Value::Mq(Mq::Frequency, MqFlags::empty())),
    ] {
        let e = d.set(&key, &value, None).unwrap_err();
        assert!(matches!(e, Error::NotSupported(_)), "{key:?} {value:?}: {e}");
    }
    assert_eq!(f.writes().len(), 7, "nothing sent for a refusal");
    // Four-wire ohms: its own flag, and its ranges.
    d.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Resistance, MqFlags::FOUR_WIRE), None).unwrap();
    assert_eq!(
        d.get(&Key::MeasuredQuantity, None).unwrap(),
        Value::Mq(Mq::Resistance, MqFlags::FOUR_WIRE)
    );
    let ValueList::Str(ranges) = d.list(&Key::Range, None).unwrap() else { panic!() };
    assert_eq!(ranges, ["Auto", "30R", "300R", "3kR", "30kR", "300kR", "3MR", "30MR"]);
    assert_eq!(d.get(&Key::Range, None).unwrap(), Value::Str("3kR".into()));
    assert_eq!(d.get(&Key::Digits, None).unwrap(), Value::Str("5.5".into()));
    d.close().unwrap();
}

#[test]
fn a_setting_whose_status_says_an_error_fails() {
    let f = fake();
    let mut d = device(&f);
    d.open().unwrap();
    f.meter().errors_at = vec![(2, 0x02)];
    let e = d.set(&Key::MeasuredQuantity, &Value::Mq(Mq::Voltage, MqFlags::AC), None).unwrap_err();
    assert_eq!(e.to_string(), "device error: The RAM Self Test has failed");
}

#[test]
fn a_setting_during_a_capture_is_refused() {
    let f = fake();
    f.meter().every = 0;
    let ff = f.clone();
    bounded(move || {
        let mut d = device(&ff);
        d.open().unwrap();
        let (s, rx) = sink(64);
        d.start(s).unwrap();
        let e = d.set(&Key::Range, &Value::Str("3V".into()), None).unwrap_err();
        assert!(matches!(e, Error::Busy));
        assert!(matches!(d.get(&Key::Range, None), Err(Error::Busy)));
        d.stop().unwrap();
        drain(&rx);
        d.close().unwrap();
    });
}

#[test]
fn the_lists_are_cs() {
    let d = device(&fake());
    let ValueList::Mq(mqs) = d.list(&Key::MeasuredQuantity, None).unwrap() else { panic!() };
    assert_eq!(mqs.len(), 6);
    assert_eq!(mqs[5], (Mq::Resistance, MqFlags::FOUR_WIRE));
    let ValueList::Str(digits) = d.list(&Key::Digits, None).unwrap() else { panic!() };
    assert_eq!(digits, ["3.5", "4.5", "5.5"]);
    let keys: Vec<Key> = d.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(
        keys,
        [
            Key::Continuous,
            Key::LimitSamples,
            Key::LimitMsec,
            Key::MeasuredQuantity,
            Key::Range,
            Key::Digits
        ]
    );
}
