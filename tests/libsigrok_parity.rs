// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The value tables of libsigrok's own unit tests (`tests/*.c` in the
//! libsigrok tree), run against oxrok's counterparts. Each test names
//! the libsigrok test it mirrors. Cases that only check NULL handling
//! or C memory management have no counterpart in Rust and are left out.

use oxrok::core::analog::{MqFlags, Unit, unit_string};
use oxrok::core::util::{
    parse_period, parse_voltage, period_string, samplerate_string, voltage_string,
};
use oxrok::core::{Device, Key, Packet, Trigger, TriggerMatch, Value};
use oxrok::formats::output::si_prefix;
use std::collections::HashMap;

// strutil.c: test_hz, test_khz, test_mhz, test_ghz.
#[test]
fn strutil_samplerate_string() {
    let cases: &[(u64, &str)] = &[
        (0, "0 Hz"),
        (1, "1 Hz"),
        (23, "23 Hz"),
        (644, "644 Hz"),
        (604, "604 Hz"),
        (550, "550 Hz"),
        (1000, "1 kHz"),
        (99_000, "99 kHz"),
        (225_000, "225 kHz"),
        (1234, "1.234 kHz"),
        (12_345, "12.345 kHz"),
        (123_456, "123.456 kHz"),
        (1034, "1.034 kHz"),
        (1004, "1.004 kHz"),
        (1230, "1.23 kHz"),
        (1204, "1.204 kHz"),
        (1_000_000, "1 MHz"),
        (28_000_000, "28 MHz"),
        (775_000_000, "775 MHz"),
        (1_234_567, "1.234567 MHz"),
        (12_345_678, "12.345678 MHz"),
        (123_456_789, "123.456789 MHz"),
        (1_230_007, "1.230007 MHz"),
        (1_034_567, "1.034567 MHz"),
        (1_000_007, "1.000007 MHz"),
        (1_234_000, "1.234 MHz"),
        (1_000_000_000, "1 GHz"),
        (5_000_000_000, "5 GHz"),
        (72_000_000_000, "72 GHz"),
        (388_000_000_000, "388 GHz"),
        (4_417_594_444, "4.417594444 GHz"),
        (44_175_944_444, "44.175944444 GHz"),
        (441_759_444_441, "441.759444441 GHz"),
        (441_759_000_001, "441.759000001 GHz"),
        (441_050_000_000, "441.05 GHz"),
        (441_000_000_005, "441.000000005 GHz"),
        (441_500_000_000, "441.5 GHz"),
    ];
    for &(rate, want) in cases {
        assert_eq!(samplerate_string(rate), want, "samplerate {rate}");
    }
}

// strutil.c: test_hz_period, test_ghz_period.
#[test]
fn strutil_period_string() {
    let cases: &[(u64, u64, &str)] = &[
        (1, 1, "1 s"),
        (1, 5, "200 ms"),
        (1, 72, "13.889 ms"),
        (1, 388, "2.577 ms"),
        (10, 1000, "10 ms"),
        (10, 100, "100 ms"),
        (1, 1_000_000_000, "1 ns"),
        (1, 5_000_000_000, "200 ps"),
        (1, 72_000_000_000, "13.889 ps"),
        (1, 388_000_000_000, "2.577 ps"),
        (10, 1_000_000_000_000, "10 ps"),
        (200, 1_000_000_000_000, "200 ps"),
        (10, 1_000_000_000, "10 ns"),
    ];
    for &(p, q, want) in cases {
        assert_eq!(period_string(p, q), want, "period {p}/{q}");
    }
}

// strutil.c: test_volt.
#[test]
fn strutil_voltage_string() {
    let cases: &[(u64, u64, &str)] = &[
        (34, 1, "34 V"),
        (34, 2, "17 V"),
        (1, 1, "1 V"),
        (1, 5, "0.2 V"),
        (200, 1000, "200 mV"),
        (1, 72, "0.0138889 V"),
        (1, 388, "0.00257732 V"),
        (10, 1000, "10 mV"),
    ];
    for &(p, q, want) in cases {
        assert_eq!(voltage_string(p, q), want, "voltage {p}/{q}");
    }
}

// sr_parse_period and sr_parse_voltage have no test of their own in
// libsigrok; these are the forms its implementation accepts: an integer
// and a unit, the unit separated by spaces or not, case blind for volts.
// oxrok reduces the fraction, so the ratio is what is compared.
#[test]
fn strutil_parse_period_and_voltage() {
    let same = |a: (u64, u64), b: (u64, u64)| {
        u128::from(a.0) * u128::from(b.1) == u128::from(b.0) * u128::from(a.1)
    };
    for (text, want) in [
        ("1s", (1, 1)),
        ("10 ms", (10, 1000)),
        ("500us", (500, 1_000_000)),
        ("2ns", (2, 1_000_000_000)),
        ("20ps", (20, 1_000_000_000_000)),
        ("5fs", (5, 1_000_000_000_000_000)),
    ] {
        let got = parse_period(text).unwrap_or_else(|| panic!("period {text:?} rejected"));
        assert!(same(got, want), "period {text:?}: {got:?}");
    }
    for (text, want) in
        [("500mV", (500, 1000)), ("2 V", (2, 1)), ("2v", (2, 1)), ("100 MV", (100, 1000))]
    {
        let got = parse_voltage(text).unwrap_or_else(|| panic!("voltage {text:?} rejected"));
        assert!(same(got, want), "voltage {text:?}: {got:?}");
    }
    for bad in ["", "ms", "10 parsecs", "10 kV"] {
        assert!(parse_period(bad).is_none(), "period {bad:?} accepted");
        assert!(parse_voltage(bad).is_none(), "voltage {bad:?} accepted");
    }
}

// analog.c: test_analog_si_prefix. libsigrok returns the UTF-8 micro
// sign; oxrok writes it as the same code point.
#[test]
fn analog_si_prefix() {
    let cases: &[(f32, i32, f32, i32, &str)] = &[
        (12.0, 0, 12.0, 0, ""),
        (12.0, 1, 12.0, 1, ""),
        (12.0, -1, 0.012, 2, "k"),
        (1024.0, 0, 1.024, 3, "k"),
        (1024.0, -1, 1.024, 2, "k"),
        (1024.0, -3, 1.024, 0, "k"),
        (12.0e5, 0, 1.2, 6, "M"),
        (0.123_456, 0, 0.123_456, 0, ""),
        (0.123_456, 1, 0.123_456, 1, ""),
        (0.123_456, 2, 0.123_456, 2, ""),
        (0.123_456, 3, 123.456, 0, "m"),
        (0.123_456, 4, 123.456, 1, "m"),
        (0.123_456, 5, 123.456, 2, "m"),
        (0.123_456, 6, 123.456, 3, "m"),
        (0.123_456, 7, 123.456, 4, "m"),
        (0.0123, 4, 12.3, 1, "m"),
        (0.001_23, 5, 1.23, 2, "m"),
        (0.000_123, 4, 0.123, 1, "m"),
        (0.000_123, 5, 0.123, 2, "m"),
        (0.000_123, 6, 123.0, 0, "\u{b5}"),
        (0.000_123, 7, 123.0, 1, "\u{b5}"),
    ];
    for (i, &(value, digits, want_value, want_digits, want_prefix)) in cases.iter().enumerate() {
        let (mut v, mut d) = (value, digits);
        let prefix = si_prefix(&mut v, &mut d);
        assert!((v - want_value).abs() <= 0.000_01, "case {i}: value {v}");
        assert_eq!(d, want_digits, "case {i}: digits");
        assert_eq!(prefix, want_prefix, "case {i}: prefix");
    }
}

// analog.c: test_analog_unit_to_string.
#[test]
fn analog_unit_to_string() {
    assert_eq!(unit_string(Unit::Volt, MqFlags::RMS), "V RMS");
    assert_eq!(unit_string(Unit::Ampere, MqFlags::empty()), "A");
    assert_eq!(unit_string(Unit::Celsius, MqFlags::empty()), "\u{b0}C");
}

fn binary_input(data: &[u8], samplerate: Option<u64>) -> (u64, Option<u64>, Vec<u8>) {
    let module = oxrok::formats::input::find("binary").expect("binary input");
    let mut options = HashMap::new();
    if let Some(rate) = samplerate {
        options.insert("samplerate".to_string(), rate.to_string());
    }
    let mut input = (module.create)(&options).expect("create");
    let mut packets = Vec::new();
    input.feed(data, &mut packets).expect("feed");
    input.end(&mut packets).expect("end");
    let (mut samples, mut rate, mut bytes, mut ended) = (0u64, None, Vec::new(), false);
    for p in packets {
        match p {
            Packet::Meta(items) => {
                for (k, v) in items {
                    if let (Key::Samplerate, Value::U64(r)) = (k, v) {
                        rate = Some(r);
                    }
                }
            }
            Packet::Logic(l) => {
                assert_eq!(l.unit_size, 1, "binary input defaults to 8 channels");
                samples += l.data.len() as u64;
                bytes.extend_from_slice(&l.data);
            }
            Packet::End => ended = true,
            _ => {}
        }
    }
    assert!(ended, "no end packet");
    (samples, rate, bytes)
}

// input_binary.c: test_input_binary_all_low, _all_high, _all_high_loop.
#[test]
fn input_binary_all_low_and_all_high() {
    let mut size = 1u64;
    let mut sizes = vec![0u64];
    while size < 1024 * 1024 {
        sizes.push(size);
        size *= 3;
    }
    sizes.extend((1..10).map(|i| i * 10));
    for &n in &sizes {
        let low = vec![0u8; n as usize];
        let (samples, rate, bytes) = binary_input(&low, None);
        assert_eq!((samples, rate), (n, None), "all low, {n} bytes");
        assert!(bytes.iter().all(|&b| b == 0));
        let (samples, rate, _) = binary_input(&low, Some(1250));
        assert_eq!(samples, n, "all low with samplerate, {n} bytes");
        if n > 0 {
            assert_eq!(rate, Some(1250), "samplerate meta, {n} bytes");
        }
        let high = vec![0xffu8; n as usize];
        let (samples, _, bytes) = binary_input(&high, None);
        assert_eq!(samples, n, "all high, {n} bytes");
        assert!(bytes.iter().all(|&b| b == 0xff));
    }
}

// input_binary.c: test_input_binary_hello_world.
#[test]
fn input_binary_hello_world() {
    for rate in [None, Some(1250)] {
        let (samples, got_rate, bytes) = binary_input(b"Hello world", rate);
        assert_eq!(samples, 11);
        assert_eq!(got_rate, rate);
        assert_eq!(bytes, b"Hello world");
    }
}

// input_all.c, output_all.c: modules exist, have ids, names and
// descriptions, `bits` is found by id and its first option is `width`.
#[test]
fn input_and_output_modules() {
    let inputs = oxrok::formats::input::modules();
    assert!(!inputs.is_empty());
    let outputs = oxrok::formats::output::modules();
    assert!(!outputs.is_empty());
    for m in outputs {
        assert!(!m.id.is_empty() && !m.name.is_empty() && !m.description.is_empty(), "{}", m.id);
    }
    let bits = oxrok::formats::output::find("bits").expect("bits output");
    assert_eq!(bits.id, "bits");
    assert_eq!(bits.options.first().map(|o| o.id), Some("width"));
}

// driver_all.c: test_driver_available, test_config_get_set_samplerate
// (demo at 19 kHz).
#[test]
fn driver_demo_samplerate_get_set() {
    let mut registry = oxrok::core::Registry::new();
    oxrok::register_all(&mut registry);
    assert!(registry.drivers().count() > 0);
    let demo = registry.find("demo").expect("demo driver");
    let mut devices = demo.scan(&[]).expect("scan");
    let dev: &mut Box<dyn Device> = devices.first_mut().expect("a demo device");
    dev.open().expect("open");
    dev.set(&Key::Samplerate, &Value::U64(19_000), None).expect("set");
    assert_eq!(dev.get(&Key::Samplerate, None).expect("get"), Value::U64(19_000));
    dev.close().expect("close");
}

// trigger.c: test_trigger_new_free, test_trigger_stage_add,
// test_trigger_match_add (stages numbered in order, matches kept per
// stage and channel).
#[test]
fn trigger_stages_and_matches() {
    let empty = Trigger::new();
    assert_eq!(empty.stages.len(), 0);
    let mut t = Trigger::new();
    let kinds = [
        TriggerMatch::Zero,
        TriggerMatch::One,
        TriggerMatch::Rising,
        TriggerMatch::Falling,
        TriggerMatch::Edge,
    ];
    for stage in 0..4 {
        for (ch, &m) in kinds.iter().enumerate() {
            t.add(stage, ch, m);
        }
        assert_eq!(t.stages.len(), stage + 1);
        assert_eq!(t.stages[stage].conditions.len(), kinds.len());
    }
}

// version.c: the version is a dotted triple of small numbers.
#[test]
fn version_numbers() {
    let v = oxrok::core::VERSION;
    let core = v.split('-').next().unwrap();
    let parts: Vec<u32> = core.split('.').map(|p| p.parse().expect("numeric")).collect();
    assert_eq!(parts.len(), 3, "{v}");
    assert!(parts.iter().all(|&n| n <= 20), "{v}");
    assert_eq!(v, env!("CARGO_PKG_VERSION"));
}
