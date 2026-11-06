// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The driver against sessions recorded from a Pico running the
//! sigrok-pico firmware (`OXROK_SERIAL_RECORD`): a four channel digital
//! capture in the compact format and a mixed analog and digital one.

use oxrok::core::session::Flow;
use oxrok::core::{Device, Key, Packet, Session, Value};
use oxrok::drivers::raspberrypi_pico::{PicoDevice, probe};
use oxrok::serial::record::Replay;

fn device(fixture: &str, enabled: &[&str]) -> PicoDevice {
    let text = std::fs::read_to_string(format!(
        "{}/tests/fixtures/raspberrypi_pico/{fixture}",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();
    let mut port: Box<dyn oxrok::serial::Port> = Box::new(Replay::from_text(&text).unwrap());
    let id = probe(port.as_mut(), None).unwrap().unwrap();
    assert_eq!((id.analog, id.analog_size, id.digital), (3, 1, 21));
    let mut dev = PicoDevice::new(id, "replay".to_owned(), port);
    for pos in 0..dev.channels().len() {
        let on = enabled.contains(&dev.channels()[pos].name.as_str());
        dev.enable_channel(pos, on).unwrap();
    }
    dev
}

/// Logic packets, analog packets as (channel, samples), trigger count.
type Run = (Vec<Vec<u8>>, Vec<(usize, Vec<f32>)>, u64);

fn run(mut dev: PicoDevice, rate: u64, samples: u64) -> Run {
    dev.open().unwrap();
    dev.set(&Key::Samplerate, &Value::U64(rate), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(samples), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let mut logic = Vec::new();
    let mut analog = Vec::new();
    let mut triggers = 0;
    session
        .run(|_, p| {
            match p {
                Packet::Logic(l) => logic.push(l.data.clone()),
                Packet::Analog(a) => analog.push((a.channels[0], a.samples.clone())),
                Packet::Trigger => triggers += 1,
                _ => {}
            }
            Flow::Continue
        })
        .unwrap();
    (logic, analog, triggers)
}

#[test]
fn four_digital_channels_compact_format() {
    let dev = device("pico-d4-bits.serial", &["D2", "D3", "D4"]);
    assert_eq!(dev.channels().len(), 24);
    assert_eq!(dev.channels()[3].name, "D2");
    assert_eq!(dev.channel_groups().len(), 3);
    let (logic, analog, triggers) = run(dev, 1_000_000, 12);
    let bytes: usize = logic.iter().map(Vec::len).sum();
    // 21 digital channels pack into 3 bytes per sample.
    assert_eq!(bytes, 12 * 3);
    assert!(logic.iter().flatten().all(|b| *b == 0));
    assert_eq!(analog.len(), 0);
    assert_eq!(triggers, 1);
}

#[test]
fn mixed_analog_and_digital() {
    let dev = device("pico-mixed.serial", &["A0", "A1", "D2", "D3"]);
    let (logic, analog, _) = run(dev, 100_000, 5);
    assert_eq!(logic.iter().map(Vec::len).sum::<usize>(), 5 * 3);
    let a0: Vec<f32> =
        analog.iter().filter(|(c, _)| *c == 0).flat_map(|(_, s)| s.clone()).collect();
    let a1: Vec<f32> =
        analog.iter().filter(|(c, _)| *c == 1).flat_map(|(_, s)| s.clone()).collect();
    assert_eq!((a0.len(), a1.len()), (5, 5));
    // 0.0257 V per count on a floating input around half a volt.
    assert!((a0[0] - 0.5397).abs() < 1e-3);
    assert!((a1[0] - 0.514).abs() < 1e-3);
}

/// A recording that ends before the limit is a device that went quiet or
/// left: the samples that came are kept, and the capture failed.
#[test]
fn a_stream_that_ends_before_the_limit_fails_the_run() {
    let mut dev = device("pico-d4-bits.serial", &["D2", "D3", "D4"]);
    dev.open().unwrap();
    dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(1 << 40), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let mut ended = false;
    let result = session.run(|_, p| {
        ended |= matches!(p, Packet::End);
        Flow::Continue
    });
    assert!(ended);
    assert!(matches!(result, Err(oxrok::core::Error::CaptureFailed(_))), "{result:?}");
}
