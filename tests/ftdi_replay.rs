// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The FTDI layer against a recorded (here hand written) session.

use oxrok::ftdi::{BitMode, ChipType, Ftdi, Port};
use oxrok::usb::record::Replay;
use std::time::Duration;

fn replay() -> Replay {
    let text = include_str!("fixtures/ftdi/ft245bl-probe.txt");
    Replay::from_text(text).unwrap()
}

#[test]
fn probe_sequence_replays() {
    let mut f = Ftdi::open(Box::new(replay()), Port::A).unwrap();
    assert_eq!(f.chip(), ChipType::Bm);
    assert_eq!(f.max_packet_size(), 64);
    f.reset().unwrap();
    f.set_latency_timer(4).unwrap();
    assert_eq!(f.latency_timer().unwrap(), 4);
    assert_eq!(f.set_baudrate(460_800).unwrap(), 461_538);
    assert_eq!(f.read_pins().unwrap(), 0x18);
    assert_eq!(f.read_eeprom(2).unwrap(), [0, 0, 3, 4]);
    assert_eq!(f.modem_status().unwrap(), 0x6031);
    f.set_bitmode(0, BitMode::BitBang).unwrap();
    f.set_bitmode(0, BitMode::Reset).unwrap();
    f.write(&[1, 2, 3, 4, 5]).unwrap();
    // Two 4 byte packets with a 64 byte max packet size: the fixture
    // packs them in one transfer, so only the first status pair goes.
    let mut buf = [0u8; 16];
    let n = f.read(&mut buf).unwrap();
    assert_eq!(&buf[..n], [0xaa, 0xbb, 0x31, 0x60, 0xcc]);
    assert_eq!(f.read(&mut buf).unwrap(), 0);
    f.purge_read().unwrap();
    let t = f.into_transport();
    let _ = t;
}

#[test]
fn read_exact_times_out_without_progress() {
    let text = "# oxrok usb fixture 1\n\
        device vid=0403 pid=6001 version=0600\n\
        claim 0 -> ok\n\
        mps 81 -> 64\n\
        bulk_in 81 4096 -> 316001\n\
        bulk_in 81 4096 -> err timeout\n";
    let mut f = Ftdi::open(Box::new(Replay::from_text(text).unwrap()), Port::A).unwrap();
    assert_eq!(f.chip(), ChipType::R);
    let mut buf = [0u8; 2];
    let e = f.read_exact(&mut buf, Duration::ZERO).unwrap_err();
    assert!(matches!(e, oxrok::core::Error::Timeout));
    assert_eq!(buf[0], 1);
}

#[test]
fn a_deviation_fails_the_replay() {
    let mut f = Ftdi::open(Box::new(replay()), Port::A).unwrap();
    let e = f.set_latency_timer(4).unwrap_err().to_string();
    assert!(e.contains("step 3 expected"), "{e}");
}

/// The `probe` example recorded on an Intronix LA1034 (FT245BL) on
/// macOS with nusb, 2026-10-22. Replays the same sequence.
#[test]
fn logicport_probe_recording_replays() {
    let text = include_str!("fixtures/ftdi/logicport-probe.txt");
    let mut f = Ftdi::open(Box::new(Replay::from_text(text).unwrap()), Port::A).unwrap();
    assert_eq!(f.chip(), ChipType::Bm);
    assert_eq!(f.transport().info().serial.as_deref(), Some("71543551"));
    f.reset().unwrap();
    f.set_latency_timer(4).unwrap();
    assert_eq!(f.latency_timer().unwrap(), 4);
    assert_eq!(f.set_baudrate(460_800).unwrap(), 461_538);
    assert_eq!(f.read_pins().unwrap(), 0xef);
    assert_eq!(f.modem_status().unwrap(), 0x6031);
    let eeprom = f.read_eeprom(64).unwrap();
    assert_eq!(eeprom.len(), 128);
    assert_eq!(&eeprom[2..6], [0x03, 0x04, 0x48, 0xdc]);
    // Serial descriptor offset and length, then the PnP block and the
    // threshold calibration string the LogicPort driver reads.
    let pos = usize::from(eeprom[0x12] & 0x7f) + usize::from(eeprom[0x13]);
    assert_eq!(&eeprom[pos..pos + 4], [0x02, 0x03, 0x01, 0x00]);
    assert_eq!(&eeprom[pos + 4..pos + 8], b"-004");
    f.set_bitmode(0, BitMode::BitBang).unwrap();
    assert_eq!(f.read_pins().unwrap(), 0xef);
    f.set_bitmode(0, BitMode::Reset).unwrap();
    let mut buf = [0u8; 4096];
    assert_eq!(f.read(&mut buf).unwrap(), 0);
    f.purge_read().unwrap();
}
