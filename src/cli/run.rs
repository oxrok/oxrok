// Counterpart in sigrok:
//   sigrok-cli/session.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Acquisition runs, `--get` and `--set`.

use crate::cli::args::Args;
use crate::cli::parse;
use crate::cli::pipeline::Pipeline;
use oxrok::core::session::Flow;
use oxrok::core::util::{parse_sizestring, parse_timestring};
use oxrok::core::{
    Device, DeviceClass, Error, Key, Registry, Result, Session, Value, ValueList, config_set,
};
use std::sync::atomic::Ordering;

/// Scans and picks the one device to work with, the way sigrok-cli does:
/// with several devices found, the demo device is dropped and exactly one
/// real device must remain.
fn pick_device(registry: &Registry, args: &Args) -> Result<Box<dyn Device>> {
    let devices = parse::device_scan(registry, args)?;
    if devices.is_empty() {
        return Err(Error::arg("No devices found."));
    }
    let mut devices = devices;
    if devices.len() > 1 {
        let real: Vec<Box<dyn Device>> =
            devices.into_iter().filter(|d| !d.classes().contains(&DeviceClass::Demo)).collect();
        if real.len() != 1 {
            return Err(Error::arg("oxrok only supports one device for capturing."));
        }
        devices = real;
    }
    let mut device = devices.remove(0);
    parse::open_device(device.as_mut())?;
    Ok(device)
}

/// `--get`.
pub fn get_options(registry: &Registry, args: &Args) -> Result<()> {
    let mut device = pick_device(registry, args)?;
    let plan = parse::plan_configs(device.as_ref(), args)?;
    // A name nothing knows is found before `-c` reaches the device.
    for get in &args.gets {
        let parsed = parse::option_args(get)?;
        let group = parse::resolve_scope(device.as_ref(), parsed.scope.as_deref(), args)?;
        for (name, _) in parsed.items {
            parse::resolve_key(device.as_ref(), &name, group.as_deref())?;
        }
    }
    plan.send_settings(device.as_mut())?;
    plan.send_enables(device.as_mut())?;
    for get in &args.gets {
        let parsed = parse::option_args(get)?;
        let group = parse::resolve_scope(device.as_ref(), parsed.scope.as_deref(), args)?;
        for (name, _) in parsed.items {
            let key = parse::resolve_key(device.as_ref(), &name, group.as_deref())?;
            // sigrok-cli's maybe_config_get() (main.c:81-84) asks the
            // driver only for a key it lists as GET in this scope:
            // SR_ERR_NA for a key without GET and for one not listed.
            let readable = device
                .options(group.as_deref())
                .map(|l| l.iter().any(|o| o.key == key && o.access.get));
            if readable.is_ok_and(|r| !r) {
                return Err(Error::Arg(format!("Failed to get '{name}': not applicable")));
            }
            let value = device
                .get(&key, group.as_deref())
                .map_err(|e| Error::Arg(format!("Failed to get '{name}': {e}")))?;
            println!("{}", get_text(&value));
        }
    }
    device.close()
}

/// A `--get` value as sigrok-cli prints it: `g_variant_print` of the
/// value's `GVariant` without type annotations, except for a measured
/// quantity, which it writes as `mq/flag/...` (our `Display`). So a
/// string is a quoted literal (`'1.5'`), a double has 17 significant
/// digits and always a point or exponent (`2.0`,
/// `0.10000000000000001`), a rational or a range is a tuple (`(1,
/// 1000)`, `(1.0, 2.5)`) and a key/value pair a one entry dictionary
/// (`{'k': 'v'}`). Strings escape what `g_unichar_isprint` rejects
/// except unassigned code points, which would need the Unicode tables
/// `GLib` was built with.
fn get_text(value: &Value) -> String {
    match value {
        Value::Str(s) => glib_string(s),
        Value::F64(v) => glib_double(*v),
        Value::Rational(p, q) | Value::RangeU64(p, q) => format!("({p}, {q})"),
        Value::RangeF64(a, b) => format!("({}, {})", glib_double(*a), glib_double(*b)),
        Value::KeyValue(k, v) => format!("{{{}: {}}}", glib_string(k), glib_string(v)),
        Value::Bool(_) | Value::U64(_) | Value::I32(_) | Value::Mq(..) => value.to_string(),
    }
}

/// A double as `g_variant_print` writes it: `g_ascii_dtostr` (C's
/// `%.17g`), with `.0` added when that has no point, exponent or
/// `inf`/`nan`.
fn glib_double(v: f64) -> String {
    if v.is_nan() {
        return "nan".to_owned();
    }
    if v.is_infinite() {
        return if v > 0.0 { "inf" } else { "-inf" }.to_owned();
    }
    let trim = |t: &str| -> String {
        if t.contains('.') {
            t.trim_end_matches('0').trim_end_matches('.').to_owned()
        } else {
            t.to_owned()
        }
    };
    // %.17g: the exponent of the value rounded to 17 digits picks the
    // style, then trailing zeros go.
    let sci = format!("{v:.16e}");
    let (mantissa, exp) = sci.split_once('e').expect("{:e} has an exponent");
    let exp: i32 = exp.parse().expect("{:e} exponent");
    let mut out = if (-4..17).contains(&exp) {
        let precision = usize::try_from(16 - exp).expect("0 to 20");
        trim(&format!("{v:.precision$}"))
    } else {
        let sign = if exp < 0 { '-' } else { '+' };
        format!("{}e{sign}{:02}", trim(mantissa), exp.unsigned_abs())
    };
    if !out.contains(['.', 'e']) {
        out.push_str(".0");
    }
    out
}

/// A string literal as `g_variant_print` writes it: in single quotes,
/// or double ones when it holds a single quote; `\`, the quote and
/// what does not print escaped.
fn glib_string(s: &str) -> String {
    use std::fmt::Write as _;
    let quote = if s.contains('\'') { '"' } else { '\'' };
    let mut out = String::with_capacity(s.len() + 2);
    out.push(quote);
    for c in s.chars() {
        if c == quote || c == '\\' {
            out.push('\\');
        }
        if glib_isprint(c) {
            out.push(c);
            continue;
        }
        let _ = match c {
            '\u{07}' => out.write_str("\\a"),
            '\u{08}' => out.write_str("\\b"),
            '\u{0c}' => out.write_str("\\f"),
            '\n' => out.write_str("\\n"),
            '\r' => out.write_str("\\r"),
            '\t' => out.write_str("\\t"),
            '\u{0b}' => out.write_str("\\v"),
            c if u32::from(c) < 0x1_0000 => write!(out, "\\u{:04x}", u32::from(c)),
            c => write!(out, "\\U{:08x}", u32::from(c)),
        };
    }
    out.push(quote);
    out
}

/// `g_unichar_isprint`: everything but control (Cc), format (Cf),
/// surrogate (not a `char`) and unassigned characters; the last are
/// taken as printable here.
fn glib_isprint(c: char) -> bool {
    // The format characters (general category Cf) of Unicode 15.1.
    const FORMAT: [(u32, u32); 21] = [
        (0x00ad, 0x00ad),
        (0x0600, 0x0605),
        (0x061c, 0x061c),
        (0x06dd, 0x06dd),
        (0x070f, 0x070f),
        (0x0890, 0x0891),
        (0x08e2, 0x08e2),
        (0x180e, 0x180e),
        (0x200b, 0x200f),
        (0x202a, 0x202e),
        (0x2060, 0x2064),
        (0x2066, 0x206f),
        (0xfeff, 0xfeff),
        (0xfff9, 0xfffb),
        (0x110bd, 0x110bd),
        (0x110cd, 0x110cd),
        (0x13430, 0x1343f),
        (0x1bca0, 0x1bca3),
        (0x1d173, 0x1d17a),
        (0xe0001, 0xe0001),
        (0xe0020, 0xe007f),
    ];
    let u = u32::from(c);
    !c.is_control() && !FORMAT.iter().any(|&(lo, hi)| (lo..=hi).contains(&u))
}

/// `--set`.
pub fn set_options(registry: &Registry, args: &Args) -> Result<()> {
    if args.configs.is_empty() {
        return Err(Error::arg("No setting specified."));
    }
    let mut device = pick_device(registry, args)?;
    parse::apply_configs(device.as_mut(), args)?;
    device.close()
}

fn has_option(device: &dyn Device, key: &Key) -> bool {
    device.option(key, None).is_some()
}

/// sigrok-cli's `maybe_config_set()` (main.c:87-97), which its `--samples` and
/// `--frames` use: a key the device cannot set fails without `check_key()`'s
/// message.
fn maybe_config_set(device: &mut dyn Device, key: &Key, value: u64) -> Result<()> {
    if device.option(key, None).is_some_and(|o| o.access.set) {
        config_set(device, key, &Value::U64(value), None)
    } else {
        Err(Error::not_supported(key.id()))
    }
}

/// The samples of a time limit. sigrok-cli's multiplication wraps
/// (session.c:61) and gives a limit unrelated to the time; here it saturates.
fn time_samples(rate: u64, msec: u64) -> u64 {
    rate.saturating_mul(msec) / 1000
}

fn set_limit_time(device: &mut dyn Device, text: &str) -> Result<u64> {
    let msec = parse_timestring(text)
        .filter(|t| *t > 0)
        .ok_or_else(|| Error::Arg(format!("Invalid time '{text}'")))?;
    if device.option(&Key::LimitMsec, None).is_some_and(|o| o.access.set) {
        config_set(device, &Key::LimitMsec, &Value::U64(msec), None)
            .map_err(|_| Error::arg("Failed to configure time limit."))?;
        return Ok(0);
    }
    if device.option(&Key::Samplerate, None).is_some_and(|o| o.access.get) {
        let rate = device.get(&Key::Samplerate, None)?.as_u64().unwrap_or(0);
        let samples = time_samples(rate, msec);
        if samples == 0 {
            return Err(Error::arg("Not enough time at this samplerate."));
        }
        config_set(device, &Key::LimitSamples, &Value::U64(samples), None)
            .map_err(|_| Error::arg("Failed to configure time-based sample limit."))?;
        return Ok(samples);
    }
    Err(Error::arg("This device does not support time limits."))
}

/// Whether the command line asks for a capture of a driver that ends each
/// one itself ([`oxrok::core::Driver::ends_captures_itself`]): its output or
/// its trigger, with no limit, which such a driver has no key for.
pub fn starts_without_limit(registry: &Registry, args: &Args) -> bool {
    let driver = args.driver.as_deref().and_then(|d| registry.find(d.split(':').next()?));
    driver.is_some_and(oxrok::core::Driver::ends_captures_itself)
        && (args.output_format.is_some() || args.output_file.is_some() || args.triggers.is_some())
}

/// A capture run: `--samples`, `--time`, `--frames` or `--continuous`.
#[allow(clippy::too_many_lines)]
pub fn session(registry: &Registry, args: &Args) -> Result<()> {
    let mut device = pick_device(registry, args)?;
    // `-c` reaches the device before the rest, as in sigrok-cli, except what
    // switches an output on: that waits until the rest of the command line
    // was accepted, just before the capture.
    let plan = parse::plan_configs(device.as_ref(), args)?;
    plan.send_settings(device.as_mut())?;
    if let Some(spec) = &args.channels {
        parse::select_channels(device.as_mut(), spec)
            .map_err(|e| Error::Arg(format!("Failed to set channels: {e}")))?;
    }
    let trigger = match &args.triggers {
        Some(spec) => Some(parse::parse_triggers(device.as_ref(), spec)?),
        None => None,
    };
    if args.continuous && !has_option(device.as_ref(), &Key::Continuous) {
        return Err(Error::arg("This device does not support continuous sampling."));
    }
    let mut limit_samples = 0u64;
    if let Some(t) = &args.time {
        limit_samples = set_limit_time(device.as_mut(), t)?;
    }
    if let Some(s) = &args.samples {
        limit_samples =
            parse_sizestring(s).map_err(|_| Error::Arg(format!("Invalid sample limit '{s}'.")))?;
        if let Ok(ValueList::MinMaxU64(min, max)) = device.list(&Key::LimitSamples, None) {
            if limit_samples < min {
                return Err(Error::Arg(format!(
                    "The device stores at least {min} samples with the current settings."
                )));
            }
            if limit_samples > max {
                return Err(Error::Arg(format!(
                    "The device can store only {max} samples with the current settings."
                )));
            }
        }
        maybe_config_set(device.as_mut(), &Key::LimitSamples, limit_samples)
            .map_err(|_| Error::arg("Failed to configure sample limit."))?;
    }
    if let Some(f) = &args.frames {
        let frames =
            parse_sizestring(f).map_err(|_| Error::Arg(format!("Invalid frame limit '{f}'.")))?;
        maybe_config_set(device.as_mut(), &Key::LimitFrames, frames)
            .map_err(|_| Error::arg("Failed to configure frame limit."))?;
    }

    device.check_capture(trigger.as_ref())?;
    let samplerate = device.get(&Key::Samplerate, None).ok().and_then(|v| v.as_u64());
    let channels = device.channels().to_vec();
    let title = registry.find(device.info().driver).map_or("unknown", |d| d.long_name()).to_owned();
    let mut pipeline = Pipeline::new(args, &channels, samplerate, &title)?;
    plan.send_enables(device.as_mut())?;

    let mut session = Session::new();
    session.add_device(device);
    session.set_trigger(trigger);
    let stop = session.stop_flag();
    let _ = ctrlc::set_handler(move || stop.store(true, Ordering::SeqCst));

    let result =
        session.run(|_, packet| if pipeline.handle(packet) { Flow::Continue } else { Flow::Stop });
    let devices = session.into_devices();
    for mut d in devices {
        let _ = d.close_after_capture();
    }
    // As sigrok-cli and as after a finished capture: the state before is not
    // known, and the device may be gone. So say it, whatever the -l.
    if result.is_err() && plan.turns_on() {
        eprintln!("The capture failed with the output on: it was left on.");
    }
    result?;
    pipeline.check_limit(limit_samples);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_time_limit_does_not_overflow() {
        assert_eq!(time_samples(1000, 1500), 1500);
        assert_eq!(time_samples(200_000, 1), 200);
        assert_eq!(time_samples(999, 1), 0);
        assert_eq!(time_samples(u64::MAX, 2), u64::MAX / 1000);
        assert_eq!(time_samples(2, u64::MAX), u64::MAX / 1000);
    }

    /// A device that reads `limit_samples` and has no way to set it.
    struct ReadsLimit {
        info: oxrok::core::DeviceInfo,
        set_calls: usize,
    }

    impl Device for ReadsLimit {
        fn info(&self) -> &oxrok::core::DeviceInfo {
            &self.info
        }
        fn classes(&self) -> &[DeviceClass] {
            &[]
        }
        fn channels(&self) -> &[oxrok::core::Channel] {
            &[]
        }
        fn enable_channel(&mut self, _: usize, _: bool) -> Result<()> {
            Ok(())
        }
        fn rename_channel(&mut self, _: usize, _: &str) -> Result<()> {
            Ok(())
        }
        fn options(&self, _: Option<&str>) -> Result<Vec<oxrok::core::OptionDesc>> {
            Ok(vec![oxrok::core::OptionDesc::new(Key::LimitSamples, oxrok::core::Access::GET)])
        }
        fn get(&self, key: &Key, _: Option<&str>) -> Result<Value> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn set(&mut self, _: &Key, _: &Value, _: Option<&str>) -> Result<()> {
            self.set_calls += 1;
            Ok(())
        }
        fn list(&self, key: &Key, _: Option<&str>) -> Result<ValueList> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn start(&mut self, _: oxrok::core::PacketSink) -> Result<()> {
            Ok(())
        }
        fn stop(&mut self) -> Result<()> {
            Ok(())
        }
    }

    /// sigrok-cli's --samples and --frames fail silently here: not through
    /// `check_key()`, whose message `config_set` would log.
    #[test]
    fn samples_ask_for_set_access_first() {
        let mut dev = ReadsLimit { info: oxrok::core::DeviceInfo::default(), set_calls: 0 };
        let e = maybe_config_set(&mut dev, &Key::LimitSamples, 5).unwrap_err();
        assert!(matches!(&e, Error::NotSupported(_)), "{e}");
        let e = maybe_config_set(&mut dev, &Key::LimitFrames, 5).unwrap_err();
        assert!(matches!(&e, Error::NotSupported(_)), "{e}");
        assert_eq!(dev.set_calls, 0);
    }

    #[test]
    fn a_driver_that_ends_captures_itself_starts_with_output_or_trigger() {
        use clap::Parser as _;
        let mut registry = Registry::new();
        oxrok::register_all(&mut registry);
        let starts = |argv: &[&str]| {
            let args = Args::try_parse_from([&["oxrok"], argv].concat()).unwrap();
            starts_without_limit(&registry, &args)
        };
        assert!(starts(&["-d", "lecroy-logicstudio", "-O", "bits"]));
        assert!(starts(&["-d", "lecroy-logicstudio", "-t", "D3=r"]));
        assert!(starts(&["-d", "lecroy-logicstudio", "-o", "x.sr"]));
        // As sigrok-cli without an action: the help.
        assert!(!starts(&["-d", "lecroy-logicstudio"]));
        assert!(!starts(&["-d", "lecroy-logicstudio", "-c", "samplerate=1M"]));
        // Any other driver, and no driver.
        assert!(!starts(&["-d", "demo", "-O", "bits"]));
        assert!(!starts(&["-O", "bits"]));
    }

    /// Every expectation below is what `g_variant_print` printed for
    /// the same value (`GLib` 2.88, a small C program).
    #[test]
    fn get_prints_strings_as_glib_does() {
        let s = |t: &str| get_text(&Value::Str(t.into()));
        assert_eq!(s("1.5"), "'1.5'");
        assert_eq!(s("it's"), "\"it's\"");
        // Double quotes for any single quote, the double quote escaped.
        assert_eq!(s("a'\"b\\\n"), "\"a'\\\"b\\\\\\n\"");
        assert_eq!(s("a\"b"), "'a\"b'");
        assert_eq!(s("\u{e9}x\u{7f}"), "'\u{e9}x\\u007f'");
        assert_eq!(s("\u{1}"), "'\\u0001'");
        assert_eq!(s("\u{7}\u{8}\u{b}\u{c}\u{1b}"), "'\\a\\b\\v\\f\\u001b'");
        // Format characters do not print; private use and emoji do.
        assert_eq!(s("a\u{200b}b"), "'a\\u200bb'");
        assert_eq!(s("\u{feff}x"), "'\\ufeffx'");
        assert_eq!(s("\u{85}"), "'\\u0085'");
        assert_eq!(s("\u{ad}"), "'\\u00ad'");
        assert_eq!(s("\u{e0001}"), "'\\U000e0001'");
        assert_eq!(s("\u{e000}\u{1f600}"), "'\u{e000}\u{1f600}'");
    }

    #[test]
    fn get_prints_other_values_as_glib_does() {
        let d = |v: f64| get_text(&Value::F64(v));
        assert_eq!(d(2.0), "2.0");
        assert_eq!(d(0.1), "0.10000000000000001");
        assert_eq!(d(1e20), "1e+20");
        assert_eq!(d(1e17), "1e+17");
        assert_eq!(d(1e16), "10000000000000000.0");
        assert_eq!(d(123_456_789_012_345_678.0), "1.2345678901234568e+17");
        assert_eq!(d(1e-5), "1.0000000000000001e-05");
        assert_eq!(d(0.0001), "0.0001");
        assert_eq!(d(-0.0), "-0.0");
        assert_eq!(d(1.5e300), "1.5000000000000001e+300");
        assert_eq!(d(5e-324), "4.9406564584124654e-324");
        assert_eq!(d(f64::INFINITY), "inf");
        assert_eq!(d(f64::NEG_INFINITY), "-inf");
        assert_eq!(d(f64::NAN), "nan");
        assert_eq!(d(3.0e-10), "3e-10");
        assert_eq!(d(100.0), "100.0");
        assert_eq!(get_text(&Value::Rational(1, 1000)), "(1, 1000)");
        assert_eq!(get_text(&Value::RangeU64(1, 1000)), "(1, 1000)");
        assert_eq!(get_text(&Value::RangeF64(1.0, 2.5)), "(1.0, 2.5)");
        assert_eq!(get_text(&Value::RangeF64(-0.5, 1e-7)), "(-0.5, 9.9999999999999995e-08)");
        assert_eq!(get_text(&Value::KeyValue("k".into(), "it's".into())), "{'k': \"it's\"}");
        assert_eq!(get_text(&Value::I32(-3)), "-3");
        assert_eq!(get_text(&Value::U64(u64::MAX)), "18446744073709551615");
        assert_eq!(get_text(&Value::Bool(true)), "true");
    }
}
