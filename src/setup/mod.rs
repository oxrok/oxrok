// Counterpart in sigrok:
//   sigrok-cli/parsers.c, session.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Setting up a run from sigrok-cli's argument syntaxes, without the
//! command line: driver specs, `-c` configs, `-C` channels, `-t`
//! triggers, limits, decoder stacks and format specs, as plain strings.
//!
//! The `oxrok` binary and the MCP server both build on this, so a driver
//! spec or a `-c` means the same in both and is refused with the same
//! words. Nothing here prints; the binary does that.

#![forbid(unsafe_code)]

pub mod channels;
pub mod configs;
pub mod decoders;
pub mod formats;
pub mod limits;

pub use channels::{select_channel_list, select_channels};
pub use configs::{ConfigPlan, apply_configs, plan_configs, resolve_scope};
pub use decoders::{Instance, StackPlan, Visible, parse_stacks};
pub use formats::{input_spec, output_spec};
pub use limits::{CaptureLimits, apply_limits};

use crate::core::{
    Device, DeviceClass, Driver, Error, Key, Registry, Result, Trigger, Value, ValueKind,
};

/// A `key=value:key=value` list. `first_key` names a key whose first
/// element may appear without a value and selects a scope, e.g. the
/// channel group; it is returned separately.
pub struct GenericArgs {
    /// The scope element, if the first element was one.
    pub scope: Option<String>,
    /// The remaining elements, in order. A missing value is `None`.
    pub items: Vec<(String, Option<String>)>,
}

impl GenericArgs {
    /// The items as a map of text values, a missing value empty: what the
    /// input and output modules take.
    pub fn text_options(self) -> std::collections::HashMap<String, String> {
        self.items.into_iter().map(|(k, v)| (k, v.unwrap_or_default())).collect()
    }
}

/// Splits `text` on `:` into key value pairs. With `first_is_scope` the
/// first element is a bare scope name (an output module id, a driver
/// id). Otherwise a first element `channel_group=X` is the scope; later on
/// it is a key like any other, which no device has (parsers.c:362-371).
pub fn generic_args(text: &str, first_is_scope: bool) -> GenericArgs {
    let mut scope = None;
    let mut items = Vec::new();
    for (i, element) in text.split(':').enumerate() {
        if element.is_empty() {
            continue;
        }
        if i == 0 && first_is_scope {
            scope = Some(element.to_owned());
            continue;
        }
        let (k, v) = match element.split_once('=') {
            Some((k, v)) => (k.to_owned(), Some(v.to_owned())),
            None => (element.to_owned(), None),
        };
        if i == 0 && k.eq_ignore_ascii_case("channel_group") {
            scope = v;
            continue;
        }
        items.push((k, v));
    }
    GenericArgs { scope, items }
}

/// The argument of a `-c` or `--get`, whose first element may be the scope.
/// An empty one is refused: sigrok-cli dies on an assertion there
/// (parsers.c:363-364).
pub fn option_args(text: &str) -> Result<GenericArgs> {
    if text.starts_with(':') {
        return Err(Error::Arg(format!("Invalid option '{text}': its first element is empty.")));
    }
    Ok(generic_args(text, false))
}

/// An option's value from `-c` or `-d`, as sigrok-cli's `opt_to_gvar()`:
/// only a boolean goes without one, and is true.
pub fn option_value(name: &str, text: Option<&str>, kind: ValueKind) -> Result<Value> {
    if kind == ValueKind::Bool {
        let Some(v) = text else { return Ok(Value::Bool(true)) };
        return Value::parse(kind, v).map_err(|_| {
            Error::Arg(format!(
                "Invalid value: '{v}' for option '{name}': \
                 it takes true, yes, on, 1, false, no, off or 0."
            ))
        });
    }
    match text {
        None | Some("") => Err(Error::Arg(format!("Option '{name}' needs a value."))),
        Some(v) => Value::parse(kind, v),
    }
}

/// A driver and the scan options parsed for it.
pub type DriverSpec<'a> = (&'a dyn Driver, Vec<(Key, Value)>);

/// The driver named by `-d`, with its scan options.
pub fn driver_spec<'a>(registry: &'a Registry, spec: &str) -> Result<DriverSpec<'a>> {
    let args = generic_args(spec, true);
    let id = args.scope.ok_or_else(|| Error::arg("Invalid driver."))?;
    let driver = registry.find(&id).ok_or_else(|| Error::Arg(format!("Driver {id} not found.")))?;
    let mut options = Vec::new();
    for (k, v) in args.items {
        let key = driver
            .accepted_scan_options()
            .iter()
            .find(|key| key.id() == k)
            .cloned()
            .ok_or_else(|| Error::Arg(format!("Unknown scan option '{k}'.")))?;
        options.push((key.clone(), option_value(&k, v.as_deref(), key.kind())?));
    }
    Ok((driver, options))
}

/// Scans for devices: the `-d` driver spec, or every driver when none.
pub fn scan(registry: &Registry, driver: Option<&str>) -> Result<Vec<Box<dyn Device>>> {
    if let Some(spec) = driver {
        let (driver, options) = driver_spec(registry, spec)?;
        driver.scan(&options)
    } else {
        let mut all = Vec::new();
        for driver in registry.drivers() {
            match driver.scan(&[]) {
                Ok(devs) => all.extend(devs),
                Err(e) => log::warn!("{}: scan failed: {e}", driver.id()),
            }
        }
        Ok(all)
    }
}

/// Opens a device for a command line, saying what sigrok-cli says with the
/// reason added.
pub fn open_device(device: &mut dyn Device) -> Result<()> {
    device.open().map_err(|e| Error::Arg(format!("Failed to open device: {e}.")))
}

/// Scans and picks the one device to work with, opened, the way sigrok-cli
/// does: with several devices found, the demo device is dropped and exactly
/// one real device must remain.
pub fn pick_device(registry: &Registry, driver: Option<&str>) -> Result<Box<dyn Device>> {
    let devices = scan(registry, driver)?;
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
    open_device(device.as_mut())?;
    Ok(device)
}

/// Resolves a key name against the device's options of `group`, so a
/// driver's custom keys work like the well known ones.
pub fn resolve_key(device: &dyn Device, name: &str, group: Option<&str>) -> Result<Key> {
    crate::core::resolve_key(device, name, group)
}

/// Parses `-t`: `D0=r,D1=1` is one stage; `;` separates stages, so
/// `D0=r;D1=f` fires on a falling D1 after a rising D0. Several match
/// letters on one channel (`D0=rf`) go to successive stages, as in
/// sigrok-cli.
pub fn parse_triggers(device: &dyn Device, spec: &str) -> Result<Trigger> {
    crate::core::trigger_syntax::parse(device, spec)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_generic_args() {
        let g = generic_args("bits:width=64", true);
        assert_eq!(g.scope.as_deref(), Some("bits"));
        assert_eq!(g.items, [("width".to_owned(), Some("64".to_owned()))]);
        let g = generic_args("channel_group=Logic:pattern=oxrok:rle", false);
        assert_eq!(g.scope.as_deref(), Some("Logic"));
        assert_eq!(g.items.len(), 2);
        assert_eq!(g.items[1], ("rle".to_owned(), None));
    }

    #[test]
    fn channel_group_is_the_scope_only_as_the_first_element() {
        let g = generic_args("CHANNEL_GROUP=R1:enabled=on", false);
        assert_eq!(g.scope.as_deref(), Some("R1"));
        assert_eq!(g.items, [("enabled".to_owned(), Some("on".to_owned()))]);
        let g = generic_args("enabled=on:channel_group=R1", false);
        assert_eq!(g.scope, None);
        assert_eq!(g.items[1], ("channel_group".to_owned(), Some("R1".to_owned())));
        let g = generic_args("channel_group=R1:enabled=on:channel_group=R2", false);
        assert_eq!(g.scope.as_deref(), Some("R1"));
        assert_eq!(g.items[1], ("channel_group".to_owned(), Some("R2".to_owned())));
        // An output's or a driver's own: an element like any other.
        let g = generic_args("bits:channel_group=R1", true);
        assert_eq!(g.scope.as_deref(), Some("bits"));
        assert_eq!(g.items.len(), 1);
    }

    #[test]
    fn an_option_without_a_value_is_worded_as_in_c() {
        let mut r = Registry::new();
        r.register(Box::new(crate::drivers::demo::DemoDriver));
        for spec in ["demo:logic_channels", "demo:logic_channels="] {
            let e = driver_spec(&r, spec).err().unwrap();
            assert!(
                matches!(&e, Error::Arg(m) if m == "Option 'logic_channels' needs a value."),
                "{spec}: {e}"
            );
        }
        for (text, kind) in [(None, ValueKind::U64), (Some(""), ValueKind::Str)] {
            let e = option_value("x", text, kind).unwrap_err();
            assert!(matches!(&e, Error::Arg(m) if m == "Option 'x' needs a value."), "{e}");
        }
        assert_eq!(option_value("x", None, ValueKind::Bool).unwrap(), Value::Bool(true));
        assert!(option_value("x", Some(""), ValueKind::Bool).is_err());
    }

    #[test]
    fn picks_the_demo_device_when_asked_for() {
        let mut r = Registry::new();
        r.register(Box::new(crate::drivers::demo::DemoDriver));
        let Ok(d) = pick_device(&r, Some("demo:logic_channels=2:analog_channels=0")) else {
            panic!("demo")
        };
        assert_eq!(d.channels().len(), 2);
        let Err(e) = pick_device(&r, Some("nope")) else { panic!("an error") };
        assert!(matches!(&e, Error::Arg(m) if m == "Driver nope not found."), "{e}");
        let t = parse_triggers(d.as_ref(), "D0=r,D1=1;D0=f").unwrap();
        assert_eq!(t.stages.len(), 2);
    }
}
