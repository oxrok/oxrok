// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The driver against USB sessions recorded on an Intronix LA1034 with
//! `OXROK_USB_RECORD`, 2026-10-22, macOS, nusb. The ESP32 signal
//! generator drove CLK1 with a square wave. The expected output is what
//! `oxrok -O bits` printed during the recording, which matched the C
//! driver through sigrok-cli byte for byte.

use oxrok::core::session::Flow;
use oxrok::core::{Device, Error, Key, Packet, Session, Trigger, TriggerMatch, Value};
use oxrok::drivers::logicport::LogicPortDevice;
use oxrok::usb::record::Replay;

/// Driver debug messages on stderr, visible with `--nocapture`.
struct Stderr;

impl log::Log for Stderr {
    fn enabled(&self, m: &log::Metadata) -> bool {
        m.target().starts_with("oxrok")
    }

    fn log(&self, r: &log::Record) {
        if self.enabled(r.metadata()) {
            eprintln!("{}: {}", r.level(), r.args());
        }
    }

    fn flush(&self) {}
}

fn logging() {
    static LOGGER: Stderr = Stderr;
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Debug);
}

/// The CLK1 bits and the trigger index from a `bits` output.
fn expected(bits: &str) -> (String, usize) {
    let mut s = String::new();
    let mut trigger = None;
    for line in bits.lines() {
        if let Some(rest) = line.strip_prefix("CLK1:") {
            s.extend(rest.chars().filter(|c| *c == '0' || *c == '1'));
        } else if let Some(rest) = line.strip_prefix("T:") {
            let col: usize = rest.trim_end().rsplit(' ').next().unwrap().parse().unwrap();
            trigger = Some(s.len() - 64 + col);
        }
    }
    (s, trigger.expect("a trigger marker"))
}

/// Runs a capture through the session and returns the CLK1 bits and
/// the trigger index.
fn capture(
    fixture: &str,
    config: &[(Key, Value)],
    enabled: &[usize],
    limit: u64,
) -> (String, Option<usize>) {
    logging();
    let replay = Replay::from_text(fixture).unwrap();
    let mut dev = LogicPortDevice::from_transport(Box::new(replay));
    dev.open().unwrap();
    for (k, v) in config {
        dev.set(k, v, None).unwrap();
    }
    for pos in 0..dev.channels().len() {
        dev.enable_channel(pos, enabled.contains(&pos)).unwrap();
    }
    dev.set(&Key::LimitSamples, &Value::U64(limit), None).unwrap();
    let mut trigger = Trigger::new();
    trigger.add(0, 32, TriggerMatch::Rising);
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    session.set_trigger(Some(trigger));
    let mut bits = String::new();
    let mut at = None;
    let mut count = 0u64;
    session
        .run(|_, p| {
            match p {
                Packet::Trigger => at = Some(bits.len()),
                Packet::Logic(l) => {
                    for s in l.data.chunks(l.unit_size) {
                        bits.push(if s[4] & 1 != 0 { '1' } else { '0' });
                        count += 1;
                    }
                    if count >= limit {
                        return Flow::Stop;
                    }
                }
                _ => {}
            }
            Flow::Continue
        })
        .unwrap();
    for mut d in session.into_devices() {
        d.close().unwrap();
    }
    (bits, at)
}

// Rates above 1 MHz load the vendor FPGA image, which the user extracts
// from the LogicPort software; the recording holds the bytes the device
// saw, so the test needs the same file to reproduce them.
#[test]
fn ten_megahertz_capture_with_fpga_configuration() {
    if ["intronix-logicport.ccf", "intronix-logicport-timing-sdr.rbf"]
        .iter()
        .all(|n| oxrok::core::firmware::find_file(n).is_none())
    {
        eprintln!("skipped: no LogicPort FPGA image installed, see docs/DEVICES.md");
        return;
    }
    let (want, t) = expected(include_str!("fixtures/logicport/capture-10m-clk1.bits"));
    assert_eq!(t, 32);
    let (bits, at) = capture(
        include_str!("fixtures/logicport/capture-10m-clk1.txt"),
        &[
            (Key::Samplerate, Value::U64(10_000_000)),
            (Key::VoltageThreshold, Value::RangeF64(1.5, 1.5)),
        ],
        &[0, 1, 2, 32],
        64,
    );
    assert_eq!(bits, want);
    assert_eq!(at, Some(32));
}

#[test]
fn compressed_capture_expands_count_records() {
    let (want, t) = expected(include_str!("fixtures/logicport/capture-1m-rle.bits"));
    assert_eq!(t, 250);
    let (bits, at) = capture(
        include_str!("fixtures/logicport/capture-1m-rle.txt"),
        &[
            (Key::Samplerate, Value::U64(1_000_000)),
            (Key::Rle, Value::Bool(true)),
            (Key::VoltageThreshold, Value::RangeF64(1.5, 1.5)),
        ],
        &[32],
        500,
    );
    assert_eq!(bits.len(), 500);
    assert_eq!(bits, want);
    assert_eq!(at, Some(250));
}

#[test]
fn a_different_setting_fails_the_replay() {
    let replay = Replay::from_text(include_str!("fixtures/logicport/capture-1m-rle.txt")).unwrap();
    let mut dev = LogicPortDevice::from_transport(Box::new(replay));
    dev.open().unwrap();
    dev.set(&Key::Samplerate, &Value::U64(2_000_000), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let mut ended = false;
    let result = session.run(|_, p| {
        if matches!(p, Packet::End) {
            ended = true;
        }
        Flow::Continue
    });
    // The acquisition thread logs the mismatch and ends the stream, and the
    // run says it failed.
    assert!(ended);
    assert!(matches!(result, Err(Error::CaptureFailed(_))), "{result:?}");
}
