#![allow(clippy::format_collect, clippy::doc_markdown)]

//! The driver against hand written serial fixtures: packets built from
//! the chipset layouts, with the byte stream starting mid packet as a
//! real cable does.

use oxrok::core::session::Flow;
use oxrok::core::{Device, Driver, Key, Mq, MqFlags, Packet, Session, Unit, Value};
use oxrok::drivers::serial_dmm::{DmmDevice, SerialDmmDriver, detect, models};
use oxrok::serial::record::Replay;
use std::time::Duration;

/// Readings (channel, value, mq, unit, flags, digits) from one run.
type Readings = Vec<(usize, f32, Mq, Unit, MqFlags, i32)>;

fn run(model: &str, fixture: &str, limit: u64) -> Readings {
    let model = models::find(model).unwrap();
    let replay = Replay::from_text(fixture).unwrap();
    let mut dev = DmmDevice::from_port(model, Box::new(replay));
    dev.open().unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(limit), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let mut out = Readings::new();
    session
        .run(|_, p| {
            if let Packet::Analog(a) = p {
                out.push((a.channels[0], a.samples[0], a.mq, a.unit, a.flags, a.digits));
            }
            Flow::Continue
        })
        .unwrap();
    out
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// A UT61E (ES519xx, 19200 baud, 14 bytes) packet: range, 5 digits,
/// function, status, four option bytes, CR LF.
fn ut61e(range: u8, digits: &str, function: u8, status: u8, opt3: u8) -> Vec<u8> {
    let mut p = vec![b'0' + range];
    p.extend_from_slice(digits.as_bytes());
    p.extend([function, 0x30 | status, 0x30, 0x30, 0x30 | opt3, 0x30, b'\r', b'\n']);
    p
}

#[test]
fn ut61e_stream_mid_packet() {
    // 12.345 V DC auto, then -0.100 V, with the stream joining half way
    // through a packet and the bytes arriving in odd chunks.
    let a = ut61e(1, "12345", 0x3b, 0, 8 | 2);
    let b = ut61e(1, "00100", 0x3b, 4, 8 | 2);
    let mut stream = a[7..].to_vec();
    stream.extend(&a);
    stream.extend(&b);
    stream.extend(&a);
    let fixture = format!(
        "# oxrok serial fixture 1\nport /dev/ttyUSB0 19200/7o1/rts=0/dtr=1\nread 128 -> {}\nread 128 -> -\nread 128 -> {}\nread 128 -> {}\n",
        hex(&stream[..10]),
        hex(&stream[10..30]),
        hex(&stream[30..])
    );
    let out = run("uni-t-ut61e-ser", &fixture, 3);
    assert_eq!(out.len(), 3);
    let (ch, v, mq, unit, flags, digits) = out[0];
    assert_eq!((ch, mq, unit, digits), (0, Mq::Voltage, Unit::Volt, 3));
    assert!((v - 12.345).abs() < 1e-4);
    assert!(flags.contains(MqFlags::DC | MqFlags::AUTORANGE));
    assert!((out[1].1 + 0.1).abs() < 1e-5);
    assert!((out[2].1 - 12.345).abs() < 1e-4);
}

#[test]
fn ut61e_limit_is_checked_after_a_whole_read() {
    // libsigrok's receive_data checks the sample limit once a read's
    // packets are all handled: four packets in one read with a limit
    // of one are four samples.
    let mut stream = Vec::new();
    for d in ["12345", "00100", "12345", "00100"] {
        stream.extend(ut61e(1, d, 0x3b, 0, 8 | 2));
    }
    let fixture = format!(
        "# oxrok serial fixture 1\nport /dev/ttyUSB0 19200/7o1/rts=0/dtr=1\nread 128 -> {}\nread 128 -> {}\n",
        hex(&stream),
        hex(&ut61e(1, "99999", 0x3b, 0, 8 | 2))
    );
    assert_eq!(run("uni-t-ut61e-ser", &fixture, 1).len(), 4);
    assert_eq!(run("uni-t-ut61e-ser", &fixture, 5).len(), 5);
}

#[test]
fn a_port_that_does_not_open_is_no_device() {
    // As serial-dmm's scan: a failed serial_open finds nothing, it is
    // not an error.
    let driver = SerialDmmDriver(models::find("uni-t-ut61e-ser").unwrap());
    let conn = |c: &str| vec![(Key::Conn, Value::Str(c.to_owned()))];
    assert!(driver.scan(&conn("/dev/oxrok-no-such-port")).unwrap().is_empty());
    let mut opts = conn("/dev/oxrok-no-such-port");
    opts.push((Key::Serialcomm, Value::Str("200/7o1".to_owned())));
    assert!(driver.scan(&opts).unwrap().is_empty());
}

#[test]
fn metex_requests_and_four_displays() {
    // A four display Metex: the driver sends 'D', the meter answers
    // four 14 byte packets. The time limit ends the run.
    let mut four = Vec::new();
    for line in ["DC  1.000 V  \r", "AC  0.500 V  \r", "OH  1.234KOhm\r", "FR 12.34  KHz\r"] {
        four.extend_from_slice(line.as_bytes());
    }
    let fixture = format!(
        "# oxrok serial fixture 1\nport p 9600/7n2/rts=0/dtr=1\nwrite 44\nread 128 -> {}\nwrite 44\nread 128 -> -\n",
        hex(&four)
    );
    let out = run("metex-m3860m", &fixture, 1);
    assert_eq!(out.len(), 4);
    assert_eq!(out[2].0, 2);
    assert_eq!(out[2].2, Mq::Resistance);
    assert!((out[2].1 - 1234.0).abs() < 1e-2);
    assert_eq!(out[3].2, Mq::Frequency);
}

#[test]
fn detection_finds_a_packet_in_noise() {
    let p = ut61e(0, "00000", 0x33, 0, 2);
    let mut stream = vec![0x00, 0xff, 0x12];
    stream.extend(&p);
    let fixture = format!(
        "# oxrok serial fixture 1\nport p 19200/7o1\n{}read 1 -> -\n",
        stream.iter().map(|b| format!("read 1 -> {b:02x}\n")).collect::<String>()
    );
    let mut port = Replay::from_text(&fixture).unwrap();
    let model = models::find("uni-t-ut61e-ser").unwrap();
    assert!(detect(&mut port, &model.protocol, Duration::from_secs(1)).unwrap());
    let fixture = "# oxrok serial fixture 1\nport p 19200/7o1\nread 1 -> 00\nread 1 -> -\n";
    let mut port = Replay::from_text(fixture).unwrap();
    // The replay runs out of data: that is an error, not a detection.
    assert!(detect(&mut port, &model.protocol, Duration::from_millis(300)).is_err());
}

#[test]
fn bm86x_requests_and_two_displays() {
    // A BM867s on the BU-86X adapter: the driver asks with four bytes,
    // the meter answers 24 bytes in three HID reports, which reach the
    // port as a byte stream. 1.2345 V AC and 50.0 Hz.
    let mut p = [0u8; 24];
    p[16..20].copy_from_slice(&[0x86; 4]);
    for (i, b) in [0x50u8, 0x6d, 0x7c, 0x72, 0x3e].iter().enumerate() {
        p[3 + i] = b << 1;
    }
    p[4] |= 0x01;
    p[8] |= 0x01;
    p[2] |= 0x01;
    p[10] = 0x3e << 1;
    p[11] = 0x5f << 1;
    p[12] = (0x5f << 1) | 0x01;
    p[14] |= 0x04;
    let fixture = format!(
        "# oxrok serial fixture 1\nport hid/bu86x 9600/8n1\nwrite 00008666\nread 128 -> {}\nread 128 -> {}\nread 128 -> {}\nwrite 00008666\nread 128 -> -\n",
        hex(&p[..8]),
        hex(&p[8..16]),
        hex(&p[16..])
    );
    let out = run("brymen-bm86x", &fixture, 1);
    assert_eq!(out.len(), 2);
    assert_eq!((out[0].0, out[0].2, out[0].3, out[0].5), (0, Mq::Voltage, Unit::Volt, 4));
    assert!((out[0].1 - 1.2345).abs() < 1e-5);
    assert!(out[0].4.contains(MqFlags::AC));
    assert_eq!((out[1].0, out[1].2), (1, Mq::Frequency));
    assert!((out[1].1 - 50.0).abs() < 1e-4);
}

#[test]
fn bm85x_variable_length_packets() {
    // Two BM85x answers of different lengths, the second with junk in
    // front of it, both after the eight byte request.
    fn packet(text: &str) -> Vec<u8> {
        let mut payload = vec![0x06u8, 0, 0, 0];
        payload.extend_from_slice(text.as_bytes());
        let crc = payload.iter().fold(0u8, |a, b| a ^ b);
        let mut p = vec![0x10, 0x02, 0x00, payload.len() as u8];
        p.extend(&payload);
        p.extend([crc, 0x10, 0x03]);
        p
    }
    let a = packet(" 12.345 ");
    let b = packet(" -0.10 ");
    let fixture = format!(
        "# oxrok serial fixture 1\nport p 9600/8n1/dtr=1/rts=1\nwrite 1002000000001003\nread 128 -> {}\nwrite 1002000000001003\nread 128 -> 00ff{}\nwrite 1002000000001003\nread 128 -> -\n",
        hex(&a),
        hex(&b)
    );
    let out = run("brymen-bm85x", &fixture, 2);
    assert_eq!(out.len(), 2);
    assert_eq!((out[0].2, out[0].3, out[0].5), (Mq::Voltage, Unit::Volt, 3));
    assert!((out[0].1 - 12.345).abs() < 1e-4);
    assert!((out[1].1 + 0.1).abs() < 1e-5);
    assert_eq!(out[1].5, 2);
}
