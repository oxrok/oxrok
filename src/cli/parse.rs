// Counterpart in sigrok:
//   sigrok-cli/parsers.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Parsers for the sigrok-cli argument syntaxes.

use crate::cli::args::Args;
use oxrok::core::{Device, Driver, Error, Key, Registry, Result, Trigger, Value};

/// A `key=value:key=value` list. `first_key` names a key whose first
/// element may appear without a value and selects a scope, e.g. the
/// channel group; it is returned separately.
pub struct GenericArgs {
    /// The scope element, if the first element was one.
    pub scope: Option<String>,
    /// The remaining elements, in order. A missing value is `None`.
    pub items: Vec<(String, Option<String>)>,
}

/// Splits `text` on `:` into key value pairs. With `first_is_scope` the
/// first element is a bare scope name (an output module id, a driver
/// id). Otherwise an element `channel_group=X` becomes the scope.
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
        if k.eq_ignore_ascii_case("channel_group") {
            scope = v;
            continue;
        }
        items.push((k, v));
    }
    GenericArgs { scope, items }
}

/// The driver named by `-d`, with its scan options.
/// A driver and the scan options parsed for it.
pub type DriverSpec<'a> = (&'a dyn Driver, Vec<(Key, Value)>);

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
        let v = v.ok_or_else(|| Error::Arg(format!("Scan option '{k}' needs a value.")))?;
        options.push((key.clone(), Value::parse(key.kind(), &v)?));
    }
    Ok((driver, options))
}

/// Scans for devices: the `-d` driver, or every driver.
pub fn device_scan(registry: &Registry, args: &Args) -> Result<Vec<Box<dyn Device>>> {
    if let Some(spec) = &args.driver {
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

/// Resolves a key name against the device's options of `group`, so a
/// driver's custom keys work like the well known ones.
pub fn resolve_key(device: &dyn Device, name: &str, group: Option<&str>) -> Result<Key> {
    oxrok::core::resolve_key(device, name, group)
}

/// Applies every `-c` option to the device. The group comes from the
/// option's own `channel_group=` element, else from `-g`.
pub fn apply_configs(device: &mut dyn Device, args: &Args) -> Result<()> {
    for config in &args.configs {
        let parsed = generic_args(config, false);
        let group = parsed.scope.clone().or_else(|| args.channel_group.clone());
        for (k, v) in parsed.items {
            let key = resolve_key(device, &k, group.as_deref())?;
            let value = match (&v, key.kind()) {
                (None, oxrok::core::ValueKind::Bool) => Value::Bool(true),
                (None, _) => return Err(Error::Arg(format!("Option '{k}' needs a value."))),
                (Some(v), kind) => Value::parse(kind, v)?,
            };
            let access = device.option(&key, group.as_deref()).map(|o| o.access);
            if access.is_none_or(|a| !a.set) {
                return Err(Error::Arg(format!(
                    "Failed to set device option '{k}': not supported."
                )));
            }
            device
                .set(&key, &value, group.as_deref())
                .map_err(|e| Error::Arg(format!("Failed to set device option '{k}': {e}.")))?;
        }
    }
    Ok(())
}

/// Applies `-C`: enables the listed channels, disables the rest, renames
/// `name=newname` entries. Accepts names and numeric ranges (`0-3`).
pub fn select_channels(device: &mut dyn Device, spec: &str) -> Result<()> {
    let names: Vec<String> = device.channels().iter().map(|c| c.name.clone()).collect();
    let find = |name: &str| names.iter().position(|n| n == name);
    let mut selected: Vec<(usize, Option<String>)> = Vec::new();
    for token in spec.split(',') {
        if token.is_empty() {
            return Err(Error::arg("Invalid empty channel."));
        }
        if let Some((lo, hi)) = token.split_once('-') {
            let (b, e): (usize, usize) = match (lo.parse(), hi.parse()) {
                (Ok(b), Ok(e)) if b < e => (b, e),
                _ => return Err(Error::Arg(format!("Invalid channel range '{token}'."))),
            };
            for n in b..=e {
                let pos = find(&n.to_string())
                    .ok_or_else(|| Error::Arg(format!("unknown channel '{n}'.")))?;
                selected.push((pos, None));
            }
        } else {
            let (name, rename) = match token.split_once('=') {
                Some((n, r)) => (n, Some(r.to_owned())),
                None => (token, None),
            };
            let pos = find(name).ok_or_else(|| Error::Arg(format!("unknown channel '{name}'.")))?;
            selected.push((pos, rename));
        }
    }
    for pos in 0..names.len() {
        device.enable_channel(pos, selected.iter().any(|(p, _)| *p == pos))?;
    }
    for (pos, rename) in selected {
        if let Some(name) = rename {
            device.rename_channel(pos, &name)?;
        }
    }
    Ok(())
}

/// Parses `-t`: `D0=r,D1=1` is one stage; `;` separates stages, so
/// `D0=r;D1=f` fires on a falling D1 after a rising D0. Several match
/// letters on one channel (`D0=rf`) go to successive stages, as in
/// sigrok-cli.
pub fn parse_triggers(device: &dyn Device, spec: &str) -> Result<Trigger> {
    oxrok::core::trigger_syntax::parse(device, spec)
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

    fn demo(logic: i32) -> Box<dyn Device> {
        let mut r = Registry::new();
        r.register(Box::new(oxrok::drivers::demo::DemoDriver));
        let (d, o) =
            driver_spec(&r, &format!("demo:logic_channels={logic}:analog_channels=1")).unwrap();
        d.scan(&o).unwrap().remove(0)
    }

    #[test]
    fn selects_and_renames_channels() {
        let mut dev = demo(4);
        select_channels(dev.as_mut(), "D0,D2=clk,A0").unwrap();
        let enabled: Vec<&str> =
            dev.channels().iter().filter(|c| c.enabled).map(|c| c.name.as_str()).collect();
        assert_eq!(enabled, ["D0", "clk", "A0"]);
        assert!(select_channels(dev.as_mut(), "D9").is_err());
    }

    #[test]
    fn parses_multi_stage_triggers() {
        let dev = demo(4);
        let t = parse_triggers(dev.as_ref(), "D0=r,D1=1;D2=f").unwrap();
        assert_eq!(t.stages.len(), 2);
        assert_eq!(t.stages[0].conditions.len(), 2);
        assert_eq!(t.stages[1].conditions[0].channel, 2);
        let t = parse_triggers(dev.as_ref(), "D0=rf").unwrap();
        assert_eq!(t.stages.len(), 2);
        assert!(parse_triggers(dev.as_ref(), "D0=o").is_err());
    }
}
