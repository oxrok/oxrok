// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The oxla driver against sessions recorded from a Tang Nano 20K
//! (`OXROK_SERIAL_RECORD`), one per path: 8, 14 and 16 channels, one and
//! two samples per clock, raw and run length encoded, with and without a
//! trigger, each of the three clocks, and a capture that overflowed. The
//! captures are of the device's counter pattern, so what they must hold is
//! known; `NAME.out` beside each fixture is what the CLI printed as it was
//! recorded.

use oxrok::core::session::Flow;
use oxrok::core::{Device, Error, Key, Packet, Session, Trigger, TriggerMatch, Value};
use oxrok::drivers::oxla::{OxlaDevice, probe};
use oxrok::serial::record::Replay;

/// The device of a recording, scanned as it was, and its clocks in Hz.
fn device(fixture: &str) -> (OxlaDevice, Vec<u32>) {
    let text = std::fs::read_to_string(format!(
        "{}/tests/fixtures/oxla/{fixture}.serial",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();
    let mut port: Box<dyn oxrok::serial::Port> = Box::new(Replay::from_text(&text).unwrap());
    let found = probe(port.as_mut()).unwrap().expect("an oxla device");
    let clocks = found.clocks.settings().iter().map(|c| c.hz).collect();
    let mut dev = OxlaDevice::new(found, "replay".to_owned(), port);
    dev.open().unwrap();
    (dev, clocks)
}

fn set(dev: &mut OxlaDevice, settings: &[(Key, Value)]) {
    for (key, value) in settings {
        dev.set(key, value, None).unwrap();
    }
}

fn pattern() -> (Key, Value) {
    (Key::PatternMode, Value::Str("Internal".into()))
}

fn trigger(stages: &[(usize, TriggerMatch)]) -> Trigger {
    let mut t = Trigger::new();
    for (stage, (channel, matches)) in stages.iter().enumerate() {
        t.add(stage, *channel, *matches);
    }
    t
}

/// What a capture sent: the samples, how many of them came ahead of the
/// trigger packet, the bytes of a sample, and how it ended.
struct Run {
    samples: Vec<u16>,
    marker: Option<usize>,
    unit: usize,
    outcome: oxrok::core::Result<()>,
}

fn run(dev: OxlaDevice, trigger: Option<Trigger>) -> Run {
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    session.set_trigger(trigger);
    let (mut samples, mut marker, mut unit) = (Vec::new(), None, 0);
    let outcome = session.run(|_, p| {
        match p {
            Packet::Logic(l) => {
                unit = l.unit_size;
                samples.extend(
                    l.data
                        .chunks(l.unit_size)
                        .map(|s| s.iter().rev().fold(0u16, |v, b| v << 8 | u16::from(*b))),
                );
            }
            Packet::Trigger => marker = Some(samples.len()),
            _ => {}
        }
        Flow::Continue
    });
    Run { samples, marker, unit, outcome }
}

/// `n` steps of a counter of `bits` bits from `from`.
fn counter(bits: u32, from: u32, n: u32) -> Vec<u16> {
    (from..from + n).map(|v| (v & ((1 << bits) - 1)) as u16).collect()
}

const CLOCKS: [u32; 3] = [81_000_000, 108_000_000, 135_000_000];

#[test]
fn sixteen_channels_with_a_trigger_at_54_mhz() {
    let (mut dev, clocks) = device("la16-trigger");
    assert_eq!(clocks, CLOCKS);
    assert_eq!(dev.channels().len(), 16);
    assert_eq!(dev.info().version, "la16");
    assert!(dev.get(&Key::Rle, None).is_err(), "no RLE in this bitstream");
    set(
        &mut dev,
        &[
            pattern(),
            (Key::Samplerate, Value::U64(54_000_000)),
            (Key::CaptureRatio, Value::U64(50)),
            (Key::LimitSamples, Value::U64(64)),
        ],
    );
    let got = run(dev, Some(trigger(&[(7, TriggerMatch::Rising)])));
    got.outcome.unwrap();
    // Bit 7 rises at 128.
    assert_eq!((got.unit, got.marker), (2, Some(32)));
    assert_eq!(got.samples, counter(16, 96, 64));
}

#[test]
fn two_stages_are_one_term_then_the_other() {
    let (mut dev, _) = device("la16-two-stages");
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(135_000_000));
    set(
        &mut dev,
        &[pattern(), (Key::CaptureRatio, Value::U64(25)), (Key::LimitSamples, Value::U64(32))],
    );
    let stages = [(5, TriggerMatch::Rising), (3, TriggerMatch::Falling)];
    let got = run(dev, Some(trigger(&stages)));
    got.outcome.unwrap();
    // Bit 5 rises at 32, and bit 3 falls next at 48.
    assert_eq!(got.marker, Some(8));
    assert_eq!(got.samples, counter(16, 40, 32));
}

#[test]
fn fourteen_channels_at_two_samples_per_clock() {
    let (mut dev, clocks) = device("la14-ddr-trigger");
    assert_eq!(clocks, CLOCKS);
    assert_eq!((dev.channels().len(), dev.info().version.as_str()), (14, "la14-ddr"));
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(270_000_000));
    set(
        &mut dev,
        &[pattern(), (Key::CaptureRatio, Value::U64(25)), (Key::LimitSamples, Value::U64(100))],
    );
    let got = run(dev, Some(trigger(&[(9, TriggerMatch::Rising)])));
    got.outcome.unwrap();
    // Samples of 14 bits across words of 32, from one that starts inside
    // a word.
    assert_eq!((got.unit, got.marker), (2, Some(25)));
    assert_eq!(got.samples, counter(14, 512 - 25, 100));
}

#[test]
fn a_rate_of_another_clock_changes_the_clock() {
    let (mut dev, _) = device("la14-ddr-81m");
    set(
        &mut dev,
        &[
            pattern(),
            (Key::Samplerate, Value::U64(81_000_000)),
            (Key::LimitSamples, Value::U64(37)),
        ],
    );
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(81_000_000));
    let got = run(dev, None);
    got.outcome.unwrap();
    // 37 samples are 518 bits: a last word with 6 of them.
    assert_eq!((got.samples, got.marker), (counter(14, 0, 37), None));
}

#[test]
fn eight_channels_at_two_samples_per_clock() {
    let (mut dev, _) = device("la8-ddr-samples");
    assert_eq!((dev.channels().len(), dev.info().version.as_str()), (8, "la8-ddr"));
    set(&mut dev, &[pattern(), (Key::LimitSamples, Value::U64(1001))]);
    let got = run(dev, None);
    got.outcome.unwrap();
    assert_eq!(got.unit, 1);
    assert_eq!(got.samples, counter(8, 0, 1001));
}

#[test]
fn runs_with_a_trigger_at_a_quarter_of_81_mhz() {
    let (mut dev, _) = device("la8-rle-trigger");
    assert_eq!(dev.info().version, "la8-rle");
    set(
        &mut dev,
        &[
            pattern(),
            (Key::Rle, Value::Bool(true)),
            (Key::Samplerate, Value::U64(20_250_000)),
            (Key::CaptureRatio, Value::U64(25)),
            (Key::LimitSamples, Value::U64(200)),
        ],
    );
    let got = run(dev, Some(trigger(&[(7, TriggerMatch::Rising)])));
    got.outcome.unwrap();
    assert_eq!(got.marker, Some(50));
    assert_eq!(got.samples, counter(8, 128 - 50, 200));
}

#[test]
fn long_runs_of_idle_inputs() {
    let (mut dev, _) = device("la8-rle-pins");
    set(
        &mut dev,
        &[
            (Key::Rle, Value::Bool(true)),
            (Key::Samplerate, Value::U64(54_000_000)),
            (Key::LimitSamples, Value::U64(100_000)),
        ],
    );
    let got = run(dev, None);
    got.outcome.unwrap();
    // Nothing on the pins, which the FPGA pulls down: a word of memory.
    assert_eq!(got.samples.len(), 100_000);
    assert!(got.samples.iter().all(|s| *s == 0));
}

/// A counter is a word a sample under RLE, more than the memory takes at
/// the full clock rate: the device says so, and the capture failed.
#[test]
fn an_overflow_fails_the_capture() {
    let (mut dev, _) = device("la8-rle-overflow");
    set(
        &mut dev,
        &[pattern(), (Key::Rle, Value::Bool(true)), (Key::LimitSamples, Value::U64(100_000))],
    );
    let got = run(dev, None);
    assert!(got.samples.is_empty());
    assert!(
        matches!(&got.outcome, Err(Error::CaptureFailed(m)) if m.contains("overflowed")),
        "{:?}",
        got.outcome
    );
}
