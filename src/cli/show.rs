// Counterpart in sigrok:
//   sigrok-cli/show.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `--version`, `-L`, `--scan` and `--show`.

use crate::cli::args::Args;
use crate::cli::parse;
use oxrok::core::util::samplerate_string;
use oxrok::core::{Device, Error, Key, Registry, Result, Value, ValueKind, ValueList};
use std::fmt::Write as _;

/// `-V`.
pub fn version(registry: &Registry) {
    println!("oxrok {}\n", oxrok::core::VERSION);
    println!("Libraries and features:");
    println!(
        "- drivers: {}",
        registry.drivers().map(oxrok::core::Driver::id).collect::<Vec<_>>().join(", ")
    );
    println!("- Host: {}-{}.", std::env::consts::ARCH, std::env::consts::OS);
}

/// No action flag given.
pub fn help() {
    println!("Usage: oxrok [OPTIONS]\n\nRun 'oxrok --help' for the options.");
}

/// `-L`.
pub fn supported(registry: &Registry) {
    println!("Supported hardware drivers:");
    for d in registry.drivers() {
        println!("  {:<20} {}", d.id(), d.long_name());
    }
    println!("\nSupported input formats:");
    for m in oxrok::formats::input::modules() {
        println!("  {:<20} {}", m.id, m.description);
    }
    println!("\nSupported output formats:");
    for m in oxrok::formats::output::modules() {
        println!("  {:<20} {}", m.id, m.description);
    }
    println!("\nSupported protocol decoders:");
    for d in crate::cli::decode::registry().decoders() {
        println!("  {:<20} {}", d.info.id, d.info.longname);
    }
    println!();
}

/// `-I module --show`.
pub fn input_detail(args: &Args) -> Result<()> {
    let spec = args.input_format.as_deref().unwrap_or_default();
    let id = parse::generic_args(spec, true).scope.unwrap_or_default();
    let m = oxrok::formats::input::find(&id)
        .ok_or_else(|| Error::Arg(format!("Unknown input module '{id}'.")))?;
    println!("ID: {}\nName: {}\nDescription: {}", m.id, m.name, m.description);
    if !m.options.is_empty() {
        println!("Options:");
        for o in m.options {
            println!("  {}: {} (default {})", o.id, o.description, show_default(o.default));
        }
    }
    Ok(())
}

/// A default as sigrok-cli prints it: numbers and booleans bare,
/// anything else quoted.
fn show_default(d: &str) -> String {
    if d == "true" || d == "false" || d.parse::<f64>().is_ok() {
        d.to_owned()
    } else {
        let escaped =
            d.replace('\\', "\\\\").replace('\n', "\\n").replace('\r', "\\r").replace('\t', "\\t");
        format!("'{escaped}'")
    }
}

/// `-O module --show`.
pub fn output_detail(args: &Args) -> Result<()> {
    let spec = args.output_format.as_deref().unwrap_or_default();
    let id = parse::generic_args(spec, true).scope.unwrap_or_default();
    let m = oxrok::formats::output::find(&id)
        .ok_or_else(|| Error::Arg(format!("Unknown output module '{id}'.")))?;
    println!("ID: {}\nName: {}\nDescription: {}", m.id, m.name, m.description);
    if !m.options.is_empty() {
        println!("Options:");
        for o in m.options {
            println!("  {}: {} (default {})", o.id, o.description, show_default(o.default));
        }
    }
    Ok(())
}

/// One line per device, as `--scan` prints it.
fn device_line(device: &dyn Device) -> String {
    let info = device.info();
    let mut s = String::from(info.driver);
    if let Some(conn) = &info.conn {
        s.push_str(":conn=");
        s.push_str(conn);
    }
    s.push_str(" - ");
    for part in [&info.vendor, &info.model, &info.version] {
        if !part.is_empty() {
            s.push_str(part);
            s.push(' ');
        }
    }
    if !info.serial.is_empty() {
        let _ = write!(s, "[S/N: {}] ", info.serial);
    }
    let channels = device.channels();
    if channels.len() == 1 {
        let _ = write!(s, "with 1 channel: {}", channels[0].name);
    } else if !channels.is_empty() {
        let _ = write!(s, "with {} channels:", channels.len());
        let mut sorted: Vec<_> = channels.iter().collect();
        sorted.sort_by_key(|c| c.index);
        for c in sorted {
            s.push(' ');
            s.push_str(&c.name);
        }
    }
    s
}

/// `--scan`.
pub fn device_list(registry: &Registry, args: &Args) -> Result<()> {
    let devices = parse::device_scan(registry, args)?;
    if devices.is_empty() {
        return Ok(());
    }
    println!("The following devices were found:");
    for d in &devices {
        println!("{}", device_line(d.as_ref()));
    }
    Ok(())
}

fn fmt_f64(v: f64) -> String {
    let s = format!("{v:.6}");
    let s = s.trim_end_matches('0');
    let s = s.trim_end_matches('.');
    if s.is_empty() || s == "-" { "0".to_owned() } else { s.to_owned() }
}

/// `--show`.
pub fn device_detail(registry: &Registry, args: &Args) -> Result<()> {
    if let Some(spec) = &args.driver {
        let (driver, _) = parse::driver_spec(registry, spec)?;
        if !driver.classes().is_empty() {
            println!("Driver functions:");
            for c in driver.classes() {
                println!("    {}", c.name());
            }
        }
        if !driver.scan_options().is_empty() {
            println!("Scan options:");
            for k in driver.scan_options() {
                println!("    {}", k.id());
            }
        }
    }
    let mut devices = parse::device_scan(registry, args)?;
    if devices.is_empty() {
        return Err(Error::arg("No devices found."));
    }
    if devices.len() > 1 {
        return Err(Error::Arg(format!(
            "{} devices found. Use --scan to show them, and select one to show.",
            devices.len()
        )));
    }
    let mut device = devices.remove(0);
    println!("{}", device_line(device.as_ref()));
    parse::open_device(device.as_mut())?;
    if let Some(spec) = &args.channels {
        parse::select_channels(device.as_mut(), spec)?;
    }
    let group = parse::resolve_scope(device.as_ref(), None, args)?;
    let group = group.as_deref();
    // sr_dev_options() of a group whose config_list() answers
    // SR_ERR_NA: sigrok-cli prints nothing more.
    let options = match device.options(group) {
        Err(Error::NotSupported(_)) if group.is_some() => return device.close(),
        other => other?,
    };
    let groups = device.channel_groups().to_vec();
    if !groups.is_empty() {
        println!("Channel groups:");
        for g in &groups {
            let names: Vec<&str> =
                g.channels.iter().map(|p| device.channels()[*p].name.as_str()).collect();
            let label = match names.len() {
                0 => "No channels".to_owned(),
                1 => format!("channel {}", names[0]),
                _ => format!("channels {}", names.join(" ")),
            };
            println!("    {}: {label}", g.name);
        }
    }
    print!("Supported configuration options");
    if !groups.is_empty() {
        match group {
            None => print!(" across all channel groups"),
            Some(g) => print!(" on channel group {g}"),
        }
    }
    println!(":");
    for opt in options {
        print_option(device.as_ref(), &opt.key, opt.access, group);
    }
    device.close()?;
    Ok(())
}

#[allow(clippy::too_many_lines)]
fn print_option(device: &dyn Device, key: &Key, access: oxrok::core::Access, group: Option<&str>) {
    let get = || if access.get { device.get(key, group).ok() } else { None };
    let list = || if access.list { device.list(key, group).ok() } else { None };
    let id = key.id();
    match key {
        Key::TriggerMatch => {
            if let Some(ValueList::TriggerMatches(m)) = list() {
                print!("    Supported triggers: ");
                for t in m {
                    print!("{} ", t.code());
                }
                println!();
            }
        }
        Key::LimitSamples if access.list => {
            if let Some(ValueList::MinMaxU64(_, max)) = list() {
                println!("    Maximum number of samples: {max}");
            }
        }
        Key::Samplerate => {
            print!("    samplerate");
            let current = get().and_then(|v| v.as_u64());
            match list() {
                Some(ValueList::U64(rates)) => {
                    println!(" - supported samplerates:");
                    for r in rates {
                        print!("      {}", samplerate_string(r));
                        if Some(r) == current {
                            print!(" (current)");
                        }
                        println!();
                    }
                }
                Some(ValueList::RangeStepU64 { min, max, step }) => {
                    println!(
                        " ({} - {} in steps of {})",
                        samplerate_string(min),
                        samplerate_string(max),
                        samplerate_string(step)
                    );
                }
                _ => println!(),
            }
        }
        _ => match key.kind() {
            // sigrok-cli has no branch for signed integers and prints
            // the key alone.
            ValueKind::I32 => println!("    {id}"),
            ValueKind::U64 => {
                print!("    {id}: ");
                let current = get().and_then(|v| v.as_u64());
                match list() {
                    Some(ValueList::U64(values)) => {
                        println!(" - supported values:");
                        for v in values {
                            print!("      {v}");
                            if Some(v) == current {
                                print!(" (current)");
                            }
                            println!();
                        }
                    }
                    Some(ValueList::MinMaxU64(min, max)) => println!("{min} - {max}"),
                    _ => {
                        if let Some(v) = current {
                            print!("{v} (current)");
                        }
                        println!();
                    }
                }
            }
            ValueKind::Str => {
                print!("    {id}: ");
                let current = get().and_then(|v| v.as_str().map(str::to_owned));
                if let Some(ValueList::Str(values)) = list() {
                    let parts: Vec<String> = values
                        .iter()
                        .map(|v| {
                            if Some(v) == current.as_ref() {
                                format!("{v} (current)")
                            } else {
                                v.clone()
                            }
                        })
                        .collect();
                    println!("{}", parts.join(", "));
                } else {
                    if let Some(v) = current {
                        print!("{v} (current)");
                    }
                    println!();
                }
            }
            ValueKind::Bool => {
                let current = get().and_then(|v| v.as_bool());
                let mark = |b| if current == Some(b) { " (current)" } else { "" };
                println!("    {id}: on{}, off{}", mark(true), mark(false));
            }
            ValueKind::F64 => {
                print!("    {id}: ");
                let current = get().and_then(|v| v.as_f64());
                let list = list();
                if let Some(ValueList::RangeStepF64 { min, max, step }) = list {
                    // The three numbers of `std_gvar_min_max_step_array`,
                    // which sigrok-cli prints as any list of doubles, `%f`.
                    let parts: Vec<String> = [min, max, step]
                        .iter()
                        .map(|v| {
                            let mark = if Some(*v) == current { " (current)" } else { "" };
                            format!("{v:.6}{mark}")
                        })
                        .collect();
                    println!("{}", parts.join(", "));
                } else if let Some(ValueList::F64(values)) = list {
                    let parts: Vec<String> = values
                        .iter()
                        .map(|v| {
                            // `%f`, show.c's list of doubles.
                            let mark = if Some(*v) == current { " (current)" } else { "" };
                            format!("{v:.6}{mark}")
                        })
                        .collect();
                    println!("{}", parts.join(", "));
                } else {
                    if let Some(v) = current {
                        print!("{v:.6} (current)");
                    }
                    println!();
                }
            }
            ValueKind::RangeF64 => {
                print!("    {id}: ");
                let current = match get() {
                    Some(Value::RangeF64(lo, hi)) => Some((lo, hi)),
                    _ => None,
                };
                let list = list();
                if let Some(ValueList::RangeF64(pairs)) = &list {
                    // sigrok-cli: each pair as lo-hi with one decimal,
                    // the current one when both bounds are equal.
                    let parts: Vec<String> = pairs
                        .iter()
                        .map(|&(lo, hi)| {
                            let mark = if current == Some((lo, hi)) { " (current)" } else { "" };
                            format!("{lo:.1}-{hi:.1}{mark}")
                        })
                        .collect();
                    println!("{}", parts.join(", "));
                } else if let Some(values) = list.and_then(|l| l.f64_values()) {
                    let parts: Vec<String> = values
                        .iter()
                        .map(|v| {
                            let is_current = current.is_some_and(|(lo, hi)| {
                                (lo - v).abs() < 1e-9 && (hi - v).abs() < 1e-9
                            });
                            // sigrok-cli prints each tuple as lo-hi with
                            // one decimal.
                            if is_current {
                                format!("{v:.1}-{v:.1} (current)")
                            } else {
                                format!("{v:.1}-{v:.1}")
                            }
                        })
                        .collect();
                    println!("{}", parts.join(", "));
                } else {
                    if let Some((lo, hi)) = current {
                        print!("{}-{} (current)", fmt_f64(lo), fmt_f64(hi));
                    }
                    println!();
                }
            }
            ValueKind::RangeU64 => {
                print!("    {id}: ");
                let current = match get() {
                    Some(Value::RangeU64(lo, hi)) => Some((lo, hi)),
                    _ => None,
                };
                if let Some(ValueList::RangeU64(pairs)) = list() {
                    let parts: Vec<String> = pairs
                        .iter()
                        .map(|&(lo, hi)| {
                            let mark = if current == Some((lo, hi)) { " (current)" } else { "" };
                            format!("{lo}-{hi}{mark}")
                        })
                        .collect();
                    println!("{}", parts.join(", "));
                } else {
                    if let Some((lo, hi)) = current {
                        print!("{lo}-{hi} (current)");
                    }
                    println!();
                }
            }
            ValueKind::RationalPeriod | ValueKind::RationalVolt => {
                // sigrok-cli: the key alone when there is no list, else
                // one formatted value per line.
                print!("    {id}");
                let current = get();
                if let Some(ValueList::Rational(values)) = list() {
                    println!(" - supported values:");
                    for (p, q) in values {
                        let text = if key.kind() == ValueKind::RationalPeriod {
                            oxrok::core::util::period_string(p, q)
                        } else {
                            oxrok::core::util::voltage_string(p, q)
                        };
                        print!("      {text}");
                        if current == Some(Value::Rational(p, q)) {
                            print!(" (current)");
                        }
                        println!();
                    }
                } else {
                    println!();
                }
            }
            ValueKind::Mq | ValueKind::KeyValue => {
                print!("    {id}: ");
                if let Some(v) = get() {
                    print!("{v}");
                }
                println!();
            }
        },
    }
}
