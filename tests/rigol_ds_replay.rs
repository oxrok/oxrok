#![allow(clippy::format_collect)]
// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The driver against a hand written SCPI fixture of a DS1102E: the
//! configuration readback at open, one live frame of CH1 and the key
//! unlock at close, with the exact commands libsigrok's driver sends.

use oxrok::core::session::Flow;
use oxrok::core::{Device, Key, Packet, Session, Value, ValueList};
use oxrok::drivers::rigol_ds::{RigolDevice, models};
use oxrok::scpi::record::Replay;
use oxrok::scpi::{HwInfo, Scpi};

fn step(cmd: &str, response: Option<&str>) -> String {
    let mut s = format!("send \"{cmd}\\n\"\n");
    if let Some(r) = response {
        use std::fmt::Write as _;
        let _ = writeln!(s, "response \"{r}\\n\"");
    }
    s
}

/// The readback at open: both channels, timebase 1 ms, 10x probes,
/// 1 V/div, DC, CH1 rising edge at 0.5 V.
fn open_steps() -> String {
    [
        (":CHAN1:DISP?", Some("1")),
        (":CHAN2:DISP?", Some("1")),
        (":TIM:SCAL?", Some("1.000e-03")),
        (":CHAN1:PROB?", Some("10")),
        (":CHAN2:PROB?", Some("10")),
        (":CHAN1:SCAL?", Some("1.000e+00")),
        (":CHAN2:SCAL?", Some("2.000e+00")),
        (":CHAN1:OFFS?", Some("0.000e+00")),
        (":CHAN2:OFFS?", Some("-1.000e+00")),
        (":CHAN1:COUP?", Some("DC")),
        (":CHAN2:COUP?", Some("AC")),
        (":TRIG:EDGE:SOUR?", Some("CH1")),
        (":TIM:OFFS?", Some("0.000e+00")),
        (":TRIG:EDGE:SLOP?", Some("POSITIVE")),
        (":TRIG:EDGE:LEV?", Some("5.000e-01")),
    ]
    .iter()
    .map(|(c, r)| step(c, *r))
    .collect()
}

fn device(fixture: &str) -> RigolDevice {
    let model = models::find("Rigol Technologies", "DS1102E").unwrap();
    let id = HwInfo {
        manufacturer: "Rigol Technologies".into(),
        model: "DS1102E".into(),
        serial_number: "DS1EB104702974".into(),
        firmware_version: "00.04.02".into(),
    };
    RigolDevice::new(model, &id, Scpi::new(Box::new(Replay::from_text(fixture).unwrap())))
}

#[test]
fn readback_options_and_lists() {
    let fixture = format!("# oxrok scpi fixture 1\nresource usbtmc/1.5\n{}", open_steps());
    let mut dev = device(&fixture);
    dev.open().unwrap();
    assert_eq!(dev.info().model, "DS1102E");
    assert_eq!(dev.channels().len(), 2);
    assert!(dev.channels().iter().all(|c| c.enabled));
    assert_eq!(dev.channel_groups().len(), 2);
    assert_eq!(dev.get(&Key::Timebase, None).unwrap(), Value::Rational(1, 1000));
    assert_eq!(dev.get(&Key::Vdiv, Some("CH2")).unwrap(), Value::Rational(2, 1));
    assert_eq!(dev.get(&Key::Coupling, Some("CH2")).unwrap(), Value::Str("AC".into()));
    assert_eq!(dev.get(&Key::ProbeFactor, Some("CH1")).unwrap(), Value::U64(10));
    assert_eq!(dev.get(&Key::TriggerSource, None).unwrap(), Value::Str("CH1".into()));
    assert_eq!(dev.get(&Key::TriggerSlope, None).unwrap(), Value::Str("r".into()));
    assert_eq!(dev.get(&Key::TriggerLevel, None).unwrap(), Value::F64(0.5));
    assert_eq!(dev.get(&Key::NumHdiv, None).unwrap(), Value::I32(12));
    assert_eq!(dev.get(&Key::DataSource, None).unwrap(), Value::Str("Live".into()));
    // 600 samples over 12 divisions of 1 ms.
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(50_000));
    assert_eq!(
        dev.list(&Key::DataSource, None).unwrap(),
        ValueList::Str(vec!["Live".into(), "Memory".into()])
    );
    let ValueList::Rational(tb) = dev.list(&Key::Timebase, None).unwrap() else { panic!() };
    assert_eq!(tb.first(), Some(&(2, 1_000_000_000)));
    assert!(dev.list(&Key::Vdiv, None).is_err());
    assert!(dev.get(&Key::Vdiv, Some("CH3")).is_err());
    assert_eq!(dev.options(Some("CH1")).unwrap().len(), 4);
}

#[test]
fn one_live_frame_of_ch1() {
    // A ramp of 600 bytes: 0x80 is 0 V, each count is vdiv / 25.6.
    let ramp: String =
        (0..600u32).map(|i| format!("{:02x}", 0x80u32.wrapping_sub(i % 256) & 0xff)).collect();
    let fixture = format!(
        "# oxrok scpi fixture 1\nresource usbtmc/1.5\n{}{}{}{}{}send \":WAV:DATA? CHAN1\\n\"\nresponse hex 2339303030303030363030{ramp}0a\n{}",
        open_steps(),
        step(":CHAN2:DISP OFF", None),
        step(":ACQ:MEMD LONG", None),
        step(":RUN", None),
        step(":WAV:POIN:MODE NORMAL", None),
        step(":KEY:LOCK DISABLE", None),
    );
    let mut dev = device(&fixture);
    dev.open().unwrap();
    dev.enable_channel(1, false).unwrap();
    dev.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let mut frames = 0;
    let mut samples: Vec<f32> = Vec::new();
    let mut digits = None;
    session
        .run(|_, p| {
            match p {
                Packet::FrameBegin => frames += 1,
                Packet::Analog(a) => {
                    assert_eq!(a.channels, vec![0]);
                    digits = Some(a.digits);
                    samples.extend_from_slice(&a.samples);
                }
                _ => {}
            }
            Flow::Continue
        })
        .unwrap();
    let mut devices = session.into_devices();
    devices[0].close().unwrap();
    assert_eq!(frames, 1);
    assert_eq!(samples.len(), 600);
    // 1 V/div: a count is 39.0625 mV; byte 0x80 is 0 V, 0x7f is one
    // count up.
    assert!(samples[0].abs() < 1e-6);
    assert!((samples[1] - 1.0 / 25.6).abs() < 1e-6);
    assert!((samples[100] - 100.0 / 25.6).abs() < 1e-4);
    // The ramp wraps: byte 0x81 is one count below zero.
    assert!((samples[255] + 1.0 / 25.6).abs() < 1e-6);
    assert_eq!(digits, Some(2));
}

/// A session recorded from a real DS1102E (firmware 00.02.06.00.01):
/// the readback, both channels switched on, one live frame of each.
/// The real scope answers without line endings, writes the block
/// header with eight length digits, and reports an `ALTERNATION`
/// trigger slope when it is not in edge mode.
#[test]
fn recorded_ds1102e_session() {
    let text = std::fs::read_to_string(format!(
        "{}/tests/fixtures/rigol_ds/ds1102e-frame.scpi",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();
    let model = models::find("Rigol Technologies", "DS1102E").unwrap();
    let id = HwInfo {
        manufacturer: "Rigol Technologies".into(),
        model: "DS1102E".into(),
        serial_number: "DS1EB134106194".into(),
        firmware_version: "00.02.06.00.01".into(),
    };
    let mut scpi = Scpi::new(Box::new(Replay::from_text(&text).unwrap()));
    // The recording starts with the scan's identification.
    assert_eq!(scpi.hw_id().unwrap(), id);
    let mut dev = RigolDevice::new(model, &id, scpi);
    dev.open().unwrap();
    assert!(dev.channels()[0].enabled);
    assert!(!dev.channels()[1].enabled);
    assert_eq!(dev.get(&Key::Timebase, None).unwrap(), Value::Rational(2, 1_000_000));
    assert_eq!(dev.get(&Key::Vdiv, Some("CH2")).unwrap(), Value::Rational(5, 1000));
    assert!(dev.get(&Key::TriggerSlope, None).is_err());
    assert_eq!(dev.get(&Key::TriggerLevel, None).unwrap(), Value::F64(f64::from(0.24f32)));
    dev.enable_channel(1, true).unwrap();
    dev.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let mut per_channel = [Vec::new(), Vec::new()];
    session
        .run(|_, p| {
            if let Packet::Analog(a) = p {
                per_channel[a.channels[0]].extend_from_slice(&a.samples);
            }
            Flow::Continue
        })
        .unwrap();
    let mut devices = session.into_devices();
    devices[0].close().unwrap();
    assert_eq!((per_channel[0].len(), per_channel[1].len()), (600, 600));
    // Raw bytes of zero: 128 counts above the centre at 2 V/div, and at
    // 5 mV/div with a -10 mV offset.
    assert!((per_channel[0][0] - 10.0).abs() < 1e-4);
    assert!((per_channel[1][0] - 0.035).abs() < 1e-5);
}

/// A scope that stops answering after the open fails the capture, with the
/// end still delivered.
#[test]
fn a_scpi_failure_in_a_capture_fails_the_run() {
    let fixture = format!("# oxrok scpi fixture 1\n{}", open_steps());
    let mut dev = device(&fixture);
    dev.open().unwrap();
    dev.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let (mut failed, mut ended) = (false, false);
    let result = session.run(|_, p| {
        failed |= matches!(p, Packet::Failed(_));
        ended |= matches!(p, Packet::End);
        Flow::Continue
    });
    assert!(failed && ended);
    assert!(matches!(result, Err(oxrok::core::Error::CaptureFailed(_))), "{result:?}");
}
