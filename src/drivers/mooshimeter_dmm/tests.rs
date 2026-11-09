// Counterpart in sigrok:
//   libsigrok/src/hardware/mooshimeter-dmm/api.c,
//   libsigrok/src/hardware/mooshimeter-dmm/protocol.c,
//   Copyright (C) 2019 Derek Hageman <hageman@inthat.cloud>,
//   with commits by Gerhard Sittig, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Unit tests with a fake meter on a fake bearer: the meter answers each write
//! in the call, and streams readings on its own clock while its trigger is on.

use std::io::{self, Write as _};
use std::sync::mpsc::{Receiver, sync_channel};

use super::*;

/// A node: type, name, children.
struct N(u8, &'static str, Vec<N>);

fn leaf(kind: u8, name: &'static str) -> N {
    N(kind, name, Vec::new())
}

fn plains(names: &[&'static str]) -> Vec<N> {
    names.iter().map(|n| leaf(0, n)).collect()
}

fn serialize(n: &N, out: &mut Vec<u8>) {
    out.push(n.0);
    out.push(n.1.len() as u8);
    out.extend_from_slice(n.1.as_bytes());
    out.push(n.2.len() as u8);
    for c in &n.2 {
        serialize(c, out);
    }
}

fn channel(name: &'static str, primary: &'static str, ranges: &[&'static str]) -> N {
    N(
        0,
        name,
        vec![
            N(
                2,
                "MAPPING",
                vec![
                    N(0, primary, plains(ranges)),
                    N(0, "TEMP", plains(&["350"])),
                    leaf(1, "SHARED"),
                ],
            ),
            leaf(3, "RANGE_I"),
            N(2, "ANALYSIS", plains(&["MEAN", "RMS", "BUFFER"])),
            leaf(11, "VALUE"),
            leaf(11, "OFFSET"),
            leaf(10, "BUF"),
            leaf(3, "BUF_BPS"),
            leaf(11, "BUF_LSB2NATIVE"),
        ],
    )
}

/// The meter's tree as `protocol.c:172-264` lists it, serialized.
fn tree_bytes() -> Vec<u8> {
    let root = N(
        0,
        "",
        vec![
            N(0, "ADMIN", vec![leaf(5, "CRC32"), leaf(10, "TREE"), leaf(9, "DIAGNOSTIC")]),
            leaf(3, "PCB_VERSION"),
            leaf(9, "NAME"),
            leaf(5, "TIME_UTC"),
            leaf(4, "TIME_UTC_MS"),
            leaf(11, "BAT_V"),
            N(2, "REBOOT", plains(&["NORMAL", "SHIPMODE"])),
            N(
                0,
                "SAMPLING",
                vec![
                    N(2, "RATE", plains(&["125", "250", "500", "1000", "2000", "4000", "8000"])),
                    N(2, "DEPTH", plains(&["32", "64", "128", "256"])),
                    N(2, "TRIGGER", plains(&["OFF", "SINGLE", "CONTINUOUS"])),
                ],
            ),
            N(
                0,
                "LOG",
                vec![
                    leaf(3, "ON"),
                    leaf(4, "INTERVAL"),
                    leaf(3, "STATUS"),
                    leaf(3, "POLLDIR"),
                    N(0, "INFO", vec![leaf(4, "INDEX"), leaf(5, "END_TIME"), leaf(5, "N_BYTES")]),
                    N(0, "STREAM", vec![leaf(4, "INDEX"), leaf(5, "OFFSET"), leaf(10, "DATA")]),
                ],
            ),
            channel("CH1", "CURRENT", &["10"]),
            channel("CH2", "VOLTAGE", &["60", "600"]),
            N(
                2,
                "SHARED",
                vec![
                    N(0, "AUX_V", plains(&["0.1", "0.3", "1.2"])),
                    N(
                        0,
                        "RESISTANCE",
                        plains(&["1000.0", "10000.0", "100000.0", "1000000.0", "10000000.0"]),
                    ),
                    N(0, "DIODE", plains(&["1.2"])),
                ],
            ),
            leaf(11, "REAL_PWR"),
        ],
    );
    let mut out = Vec::new();
    serialize(&root, &mut out);
    out
}

fn hex(bytes: &[u8]) -> String {
    use std::fmt::Write as _;
    bytes.iter().fold(String::new(), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

fn compressed_tree() -> Vec<u8> {
    let mut z = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
    z.write_all(&tree_bytes()).unwrap();
    z.finish().unwrap()
}

/// What the fake meter does wrong.
#[derive(Clone, Copy, Debug, Default)]
struct Faults {
    /// The Nth notification never goes out.
    lose: u32,
    /// The echo of the CRC is off by one.
    bad_crc: bool,
    /// The link goes after N readings.
    gone_after: Option<u32>,
    /// No reading ever comes.
    silent: bool,
    /// After N frames, nothing is answered.
    mute_after: Option<usize>,
}

/// The fake meter's state, shared with the test.
#[derive(Default)]
struct State {
    /// Every SERIN frame the host wrote, its sequence number left off.
    frames: Vec<Vec<u8>>,
    /// Every PDU the host sent.
    pdus: Vec<Vec<u8>>,
    /// PDUs for the host.
    out: Vec<Vec<u8>>,
    seq: u8,
    sent: u32,
    /// Values by id; a chooser's or integer's as i32.
    ints: std::collections::HashMap<u8, i32>,
    trigger_since: Option<Instant>,
    readings: u32,
    gone: bool,
    faults: Faults,
    /// The values of CH1, CH2, in turn.
    ch1: Vec<f32>,
    ch2: Vec<f32>,
    /// What a buffer holds.
    buffer: Vec<u8>,
}

const PERIOD: Duration = Duration::from_millis(30);

impl State {
    fn notify(&mut self, stream: &[u8]) {
        for piece in stream.chunks(19) {
            let mut pdu = vec![0x1b, 0x15, 0x00, self.seq];
            self.seq = self.seq.wrapping_add(1);
            pdu.extend_from_slice(piece);
            self.sent += 1;
            if self.sent == self.faults.lose {
                continue;
            }
            self.out.push(pdu);
        }
    }

    fn value(&self, id: u8) -> Vec<u8> {
        let v = self.ints.get(&id).copied().unwrap_or(0);
        match id {
            0 => {
                let mut out = vec![0];
                out.extend_from_slice(&(v as u32).to_le_bytes());
                out
            }
            3 => vec![3, 8],
            29 | 37 => {
                let mut out = vec![id];
                out.extend_from_slice(&(10.0f32 / 8_388_608.0).to_le_bytes());
                out
            }
            _ => vec![id, v as u8],
        }
    }

    fn written(&mut self, frame: &[u8]) {
        self.frames.push(frame[1..].to_vec());
        if self.faults.mute_after.is_some_and(|n| self.frames.len() > n) {
            return;
        }
        let body = &frame[1..];
        let id = body[0] & 0x7f;
        let mut answer = Vec::new();
        if body[0] & 0x80 == 0 {
            if id == 1 {
                let tree = compressed_tree();
                answer.push(1);
                answer.extend_from_slice(&(tree.len() as u16).to_le_bytes());
                answer.extend_from_slice(&tree);
            } else {
                answer = self.value(id);
            }
        } else if id == 0 {
            let mut crc = u32::from_le_bytes([body[1], body[2], body[3], body[4]]);
            if self.faults.bad_crc {
                crc ^= 1;
            }
            self.ints.insert(0, crc as i32);
            answer = self.value(0);
        } else {
            self.ints.insert(id, i32::from(body[1]));
            if id == 11 {
                self.trigger_since = (body[1] == 2).then(Instant::now);
            }
            answer = self.value(id);
        }
        self.notify(&answer);
    }

    /// A reading, if one is due.
    fn tick(&mut self) {
        let Some(since) = self.trigger_since else { return };
        if self.faults.silent || since.elapsed() < PERIOD * (self.readings + 1) {
            return;
        }
        if self.faults.gone_after == Some(self.readings) {
            self.gone = true;
            return;
        }
        let k = self.readings as usize;
        self.readings += 1;
        let pick = |l: &[f32], d: f32| if l.is_empty() { d } else { l[k % l.len()] };
        let (v1, v2) = (pick(&self.ch1, 0.25), pick(&self.ch2, 12.5));
        let mut s = Vec::new();
        if self.ints.get(&24) == Some(&2) {
            s.push(27);
            s.extend_from_slice(&(self.buffer.len() as u16).to_le_bytes());
            s.extend_from_slice(&self.buffer.clone());
        } else {
            s.push(25);
            s.extend_from_slice(&v1.to_le_bytes());
        }
        s.push(33);
        s.extend_from_slice(&v2.to_le_bytes());
        s.push(39);
        s.extend_from_slice(&(v1 * v2).to_le_bytes());
        self.notify(&s);
    }
}

#[derive(Clone)]
struct FakeMeter(Arc<Mutex<State>>);

impl FakeMeter {
    fn new(faults: Faults) -> FakeMeter {
        FakeMeter(Arc::new(Mutex::new(State { faults, ..State::default() })))
    }

    fn state(&self) -> MutexGuard<'_, State> {
        self.0.lock().unwrap()
    }

    /// The SERIN frames written, as hex, without their sequence numbers.
    fn frames(&self) -> Vec<String> {
        self.state().frames.iter().map(|f| hex(f)).collect()
    }
}

impl Bearer for FakeMeter {
    fn send(&mut self, pdu: &[u8]) -> io::Result<()> {
        let mut s = self.state();
        if s.gone {
            return Err(io::Error::new(io::ErrorKind::ConnectionReset, "gone"));
        }
        s.pdus.push(pdu.to_vec());
        s.out.push(vec![0x13]);
        if pdu[0] == 0x12 && pdu[1] == 0x12 {
            s.written(&pdu[3..]);
        }
        Ok(())
    }

    fn recv(&mut self, wait: Duration) -> io::Result<Option<Vec<u8>>> {
        let began = Instant::now();
        loop {
            {
                let mut s = self.state();
                s.tick();
                if s.gone {
                    return Err(io::Error::new(io::ErrorKind::ConnectionReset, "gone"));
                }
                if !s.out.is_empty() {
                    return Ok(Some(s.out.remove(0)));
                }
            }
            if began.elapsed() >= wait {
                return Ok(None);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }
}

fn opened(faults: Faults) -> (MooshimeterDevice, FakeMeter) {
    let fake = FakeMeter::new(faults);
    let mut dev = MooshimeterDevice::from_bearer(Box::new(fake.clone()));
    dev.open().unwrap();
    (dev, fake)
}

/// Runs a capture to its end, on a thread with a bound: its packets.
fn capture(dev: &mut MooshimeterDevice) -> Vec<Packet> {
    let (tx, rx) = sync_channel(64);
    dev.start(PacketSink::new(tx, 0)).unwrap();
    drain(&rx)
}

fn drain(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let mut out = Vec::new();
    loop {
        match rx.recv_timeout(Duration::from_secs(30)) {
            Ok((_, Packet::End)) => return out,
            Ok((_, p)) => out.push(p),
            Err(e) => panic!("no End within 30 s: {e}; got {out:?}"),
        }
    }
}

fn values(packets: &[Packet]) -> Vec<(usize, Vec<f32>)> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some((a.channels[0], a.samples.clone())),
            _ => None,
        })
        .collect()
}

fn failed(packets: &[Packet]) -> Option<String> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(e) => Some(e.clone()),
        _ => None,
    })
}

#[test]
fn an_open_goes_as_c_s_does() {
    let (_dev, fake) = opened(Faults::default());
    let frames = fake.frames();
    let crc = protocol::crc32(&compressed_tree()).to_le_bytes();
    let crc = hex(&crc);
    assert_eq!(
        frames,
        [
            "01".to_owned(),
            format!("80{crc}"),
            "8b00".into(), // trigger off
            "8900".into(), // 125 Hz
            "8a01".into(), // 64 samples
            "9c18".into(), // CH1:BUF_BPS 24
            "a418".into(), // CH2:BUF_BPS 24
            "9600".into(), // CH1 current
            "9700".into(), // its 10 A range
            "9800".into(), // the mean
            "9e00".into(), // CH2 voltage
            "9f00".into(), // its 60 V range
            "a000".into(), // the mean
        ]
    );
    let s = fake.state();
    assert_eq!(s.pdus[0], [0x12, 0x16, 0x00, 0x01, 0x00], "notifications on first");
    let seqs: Vec<u8> = s.pdus[1..].iter().map(|p| p[3]).collect();
    assert_eq!(seqs, (0..13).collect::<Vec<u8>>(), "the host counts its frames");
}

#[test]
fn a_crc_echoed_wrong_fails_the_open() {
    let fake = FakeMeter::new(Faults { bad_crc: true, ..Faults::default() });
    let mut dev = MooshimeterDevice::from_bearer(Box::new(fake));
    let e = dev.open().unwrap_err().to_string();
    assert!(e.contains("Tree CRC mismatch"), "{e}");
}

#[test]
fn a_capture_counts_each_channel_s_value_and_turns_the_trigger_off() {
    let (mut dev, fake) = opened(Faults::default());
    dev.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
    let packets = capture(&mut dev);
    assert_eq!(failed(&packets), None);
    assert_eq!(values(&packets), [(0, vec![0.25]), (1, vec![12.5]), (0, vec![0.25])]);
    let frames = fake.frames();
    assert_eq!(frames[frames.len() - 2..], ["8b02".to_owned(), "8b00".to_owned()]);
    let Packet::Analog(a) = &packets[1] else { panic!("{packets:?}") };
    assert_eq!((a.mq, a.unit, a.digits), (Mq::Current, Unit::Ampere, 2));
    assert_eq!(a.flags, MqFlags::DC | MqFlags::AUTORANGE);
}

#[test]
fn power_comes_on_channel_p_when_enabled() {
    let (mut dev, _fake) = opened(Faults::default());
    dev.enable_channel(0, false).unwrap();
    dev.enable_channel(2, true).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    let packets = capture(&mut dev);
    assert_eq!(values(&packets), [(1, vec![12.5]), (2, vec![3.125]),]);
}

#[test]
fn a_value_out_of_range_moves_an_autoranging_range() {
    let (mut dev, fake) = opened(Faults::default());
    fake.state().ch2 = vec![12.5, 75.0, 12.5];
    dev.set(&Key::LimitSamples, &Value::U64(6), None).unwrap();
    let packets = capture(&mut dev);
    assert_eq!(failed(&packets), None);
    let frames = fake.frames();
    let ranges: Vec<&String> = frames.iter().filter(|f| f.starts_with("9f")).collect();
    // The open's, then up to 600 V and back down.
    assert_eq!(ranges, ["9f00", "9f01", "9f00"]);
}

#[test]
fn a_fixed_range_stays() {
    let (mut dev, fake) = opened(Faults::default());
    fake.state().ch2 = vec![75.0];
    dev.set(&Key::ChannelConfig, &Value::Str("A,V;60".into()), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
    let packets = capture(&mut dev);
    let frames = fake.frames();
    assert_eq!(frames.iter().filter(|f| f.starts_with("9f")).count(), 2, "{frames:?}");
    let Some(Packet::Analog(a)) =
        packets.iter().find(|p| matches!(p, Packet::Analog(a) if a.channels[0] == 1))
    else {
        panic!()
    };
    assert_eq!(a.flags, MqFlags::DC);
}

#[test]
fn ch2_temperature_is_kelvin() {
    let (mut dev, _fake) = opened(Faults::default());
    dev.set(&Key::ChannelConfig, &Value::Str("A,T".into()), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    let packets = capture(&mut dev);
    let Some(Packet::Analog(a)) =
        packets.iter().find(|p| matches!(p, Packet::Analog(a) if a.channels[0] == 1))
    else {
        panic!()
    };
    assert_eq!((a.mq, a.unit), (Mq::Temperature, Unit::Kelvin));
}

#[test]
fn both_channels_on_the_shared_input_are_refused_after_ch1_is_set() {
    let (mut dev, fake) = opened(Faults::default());
    let before = fake.frames().len();
    assert!(dev.set(&Key::ChannelConfig, &Value::Str("R,D".into()), None).is_err());
    // SHARED, CH1's mapping, its range: then the refusal.
    assert_eq!(fake.frames()[before..], ["a601".to_owned(), "9602".into(), "9700".into()]);
}

#[test]
fn a_buffer_is_scaled_by_its_lsb() {
    let (mut dev, fake) = opened(Faults::default());
    // Two 24 bit samples: +1 and -1 LSB of 1e6 counts.
    fake.state().buffer = vec![0x40, 0x42, 0x0f, 0xc0, 0xbd, 0xf0];
    dev.set(&Key::ChannelConfig, &Value::Str("A;10;Buffer".into()), None).unwrap();
    dev.enable_channel(1, false).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    let packets = capture(&mut dev);
    assert_eq!(failed(&packets), None);
    let v = values(&packets);
    let lsb = 10.0f32 / 8_388_608.0;
    assert_eq!(v, [(0, vec![1e6 * lsb, -1e6 * lsb])]);
    let frames = fake.frames();
    assert!(frames.contains(&"1d".to_owned()), "the scale is read: {frames:?}");
    let Packet::Analog(a) = packets.iter().find(|p| matches!(p, Packet::Analog(_))).unwrap() else {
        panic!()
    };
    // A fixed range, and neither DC nor RMS.
    assert_eq!((a.flags, a.digits), (MqFlags::empty(), 0));
}

#[test]
fn a_lost_notification_fails_the_capture() {
    let (mut dev, fake) = opened(Faults::default());
    // After the open's: the echo of the start, then the first reading.
    let first_reading = fake.state().sent + 2;
    fake.state().faults.lose = first_reading;
    dev.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
    let packets = capture(&mut dev);
    let e = failed(&packets).expect("a failure");
    assert!(e.contains("Notification") && e.contains("lost"), "{e}");
    // The link is up: the sampling goes off, unwaited.
    assert_eq!(fake.frames().last().map(String::as_str), Some("8b00"));
}

#[test]
fn a_link_gone_fails_the_capture() {
    let (mut dev, _fake) = opened(Faults { gone_after: Some(2), ..Faults::default() });
    dev.set(&Key::LimitSamples, &Value::U64(100), None).unwrap();
    let packets = capture(&mut dev);
    assert_eq!(values(&packets).len(), 4);
    let e = failed(&packets).expect("a failure");
    assert!(e.contains("link to the meter failed"), "{e}");
}

#[test]
fn a_silent_meter_fails_a_sample_limit_and_warns_a_time_limit() {
    let (mut dev, _fake) = opened(Faults { silent: true, ..Faults::default() });
    dev.bounds.first_reading = Duration::from_millis(200);
    dev.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    let packets = capture(&mut dev);
    let e = failed(&packets).expect("a failure");
    assert!(e.contains("No reading from the meter"), "{e}");

    let (mut dev, fake) = opened(Faults { silent: true, ..Faults::default() });
    dev.set(&Key::LimitMsec, &Value::U64(300), None).unwrap();
    let began = Instant::now();
    let packets = capture(&mut dev);
    assert_eq!(failed(&packets), None);
    assert!(began.elapsed() < Duration::from_secs(10));
    assert_eq!(fake.frames().last().map(String::as_str), Some("8b00"));
}

#[test]
fn a_time_limit_ends_at_its_time() {
    let (mut dev, _fake) = opened(Faults::default());
    dev.set(&Key::LimitMsec, &Value::U64(100), None).unwrap();
    let packets = capture(&mut dev);
    assert_eq!(failed(&packets), None);
    // Readings every 30 ms: three by 100 ms, each three values.
    let n = values(&packets).len();
    assert!((6..=12).contains(&n), "{n}");
}

#[test]
fn the_heartbeat_reads_pcb_version() {
    let (mut dev, fake) = opened(Faults::default());
    dev.bounds.heartbeat = Duration::from_millis(50);
    dev.set(&Key::LimitMsec, &Value::U64(180), None).unwrap();
    capture(&mut dev);
    let polls = fake.frames().iter().filter(|f| *f == "03").count();
    assert!((2..=4).contains(&polls), "{polls}");
}

#[test]
fn a_stop_ends_the_capture_and_turns_the_trigger_off() {
    let (mut dev, fake) = opened(Faults::default());
    let (tx, rx) = sync_channel(64);
    dev.start(PacketSink::new(tx, 0)).unwrap();
    // Wait for a reading, with a bound.
    let began = Instant::now();
    while fake.state().readings == 0 && began.elapsed() < Duration::from_secs(10) {
        std::thread::sleep(Duration::from_millis(1));
    }
    dev.stop().unwrap();
    let packets = drain(&rx);
    assert_eq!(failed(&packets), None);
    dev.close().unwrap();
    assert_eq!(fake.frames().last().map(String::as_str), Some("8b00"));
}

#[test]
fn settings_read_and_write_the_tree() {
    let (mut dev, fake) = opened(Faults::default());
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(125));
    assert_eq!(dev.get(&Key::AvgSamples, None).unwrap(), Value::U64(64));
    dev.set(&Key::Samplerate, &Value::U64(300), None).unwrap();
    assert_eq!(fake.frames().last().map(String::as_str), Some("8902"), "the next larger, 500");
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(500));
    dev.set(&Key::AvgSamples, &Value::U64(1000), None).unwrap();
    assert_eq!(fake.frames().last().map(String::as_str), Some("8a03"), "the largest, 256");
    assert_eq!(
        dev.list(&Key::Samplerate, None).unwrap(),
        ValueList::U64(vec![125, 250, 500, 1000, 2000, 4000, 8000])
    );
    assert_eq!(dev.list(&Key::AvgSamples, None).unwrap(), ValueList::U64(vec![32, 64, 128, 256]));
}

#[test]
fn a_start_with_no_echo_fails_and_turns_the_trigger_off() {
    let fake = FakeMeter::new(Faults::default());
    let mut dev = MooshimeterDevice::from_bearer(Box::new(fake.clone()));
    dev.bounds.update = Duration::from_millis(200);
    dev.open().unwrap();
    let n = fake.frames().len();
    fake.state().faults.mute_after = Some(n);
    let (tx, _rx) = sync_channel(64);
    let e = dev.start(PacketSink::new(tx, 0)).unwrap_err().to_string();
    assert!(e.contains("No answer from the meter to SAMPLING:TRIGGER in time"), "{e}");
    assert_eq!(fake.frames()[n..], ["8b02".to_owned(), "8b00".to_owned()]);
}
