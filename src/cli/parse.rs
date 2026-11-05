// Counterpart in sigrok:
//   sigrok-cli/parsers.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Parsers for the sigrok-cli argument syntaxes.

use crate::cli::args::Args;
use oxrok::core::{Device, Driver, Error, Key, Registry, Result, Trigger, Value, ValueKind};

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
fn option_value(name: &str, text: Option<&str>, kind: ValueKind) -> Result<Value> {
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
        options.push((key.clone(), option_value(&k, v.as_deref(), key.kind())?));
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

/// Opens a device for a command line, saying what sigrok-cli says with the
/// reason added.
pub fn open_device(device: &mut dyn Device) -> Result<()> {
    device.open().map_err(|e| Error::Arg(format!("Failed to open device: {e}.")))
}

/// Resolves a key name against the device's options of `group`, so a
/// driver's custom keys work like the well known ones.
pub fn resolve_key(device: &dyn Device, name: &str, group: Option<&str>) -> Result<Key> {
    oxrok::core::resolve_key(device, name, group)
}

/// One `-c` element, parsed and resolved.
struct ConfigItem {
    /// The name as written, for messages.
    name: String,
    key: Key,
    value: Value,
    group: Option<String>,
    /// Which `-c` it came from.
    config: usize,
}

impl ConfigItem {
    /// Whether `other` sets the same key in the same scope. A group is as
    /// the device spells it ([`resolve_scope`]).
    fn same_setting(&self, other: &ConfigItem) -> bool {
        self.key == other.key && self.group == other.group
    }

    fn is_enabled(&self, on: bool) -> bool {
        self.key == Key::Enabled && self.value == Value::Bool(on)
    }
}

/// The `-c` options of a command line, every key resolved and every value
/// parsed, so that a mistake anywhere in them is found with nothing sent.
pub struct ConfigPlan {
    /// `enabled=off` first, then the others in the order given.
    settings: Vec<ConfigItem>,
    /// What switches an output on, after every setting was accepted.
    enables: Vec<ConfigItem>,
}

/// Puts the options in the order they go to the device: `enabled=off`,
/// the others as given, `enabled=on`. sigrok-cli has no order to keep
/// (its hash table's, session.c:700-701).
///
/// Sorting must not change where an output ends up, so:
/// - The last `enabled` of a scope is the one that counts. Within one `-c`
///   that holds for every key, as sigrok-cli's hash table keeps one value
///   per key; across several `-c` the others are all sent, in order, as
///   sigrok-cli does.
/// - A device-wide `enabled=off` overrides any `enabled=on` given before it
///   on a channel group: that on goes, rather than pulse an output that ends
///   off.
/// - `enabled=off` on a group given after a device-wide `enabled=on` (`-c
///   enabled=on -c channel_group=R1:enabled=off`: all but R1) stays after it.
///   R1 does have power between the two.
fn order_configs(items: Vec<ConfigItem>) -> ConfigPlan {
    let mut kept: Vec<ConfigItem> = Vec::with_capacity(items.len());
    for item in items {
        kept.retain(|k| {
            !(k.same_setting(&item) && (k.config == item.config || item.key == Key::Enabled))
        });
        if item.is_enabled(false) && item.group.is_none() {
            kept.retain(|k| !(k.is_enabled(true) && k.group.is_some()));
        }
        kept.push(item);
    }
    let mut offs = Vec::new();
    let mut others = Vec::new();
    let mut enables: Vec<ConfigItem> = Vec::new();
    for item in kept {
        if item.is_enabled(true) {
            enables.push(item);
        } else if item.is_enabled(false) {
            if enables.iter().any(|on| on.group.is_none()) {
                enables.push(item);
            } else {
                offs.push(item);
            }
        } else {
            others.push(item);
        }
    }
    offs.append(&mut others);
    ConfigPlan { settings: offs, enables }
}

/// The scope of a `-c`, a `--get` or a `--show`: its own `channel_group=`,
/// else `-g`, as the device spells it whatever the case given, and `None` for
/// the device, `global` in any case, as sigrok-cli's `lookup_channel_group()`
/// (device.c). Refused in its words, which end sigrok-cli too (a critical
/// message exits, main.c:46-47), though after what it already sent. An empty
/// name, device-wide there, is refused too: a shell variable that came out
/// empty should not switch every output.
pub fn resolve_scope(
    device: &dyn Device,
    own: Option<&str>,
    args: &Args,
) -> Result<Option<String>> {
    let Some(name) = own.or(args.channel_group.as_deref()) else { return Ok(None) };
    if name.eq_ignore_ascii_case("global") {
        return Ok(None);
    }
    let groups = device.channel_groups();
    if !name.is_empty() && groups.is_empty() {
        return Err(Error::arg("This device does not have any channel groups."));
    }
    groups
        .iter()
        .find(|g| !name.is_empty() && g.name.eq_ignore_ascii_case(name))
        .map(|g| Some(g.name.clone()))
        .ok_or_else(|| Error::Arg(format!("Invalid channel group '{name}'")))
}

/// Resolves and checks every `-c` option: its scope, its key, its value,
/// that the key can be set in that scope and what the core refuses for every
/// driver. No driver's option list depends on what a `-c` sets (each answers
/// from its model and the group), so all this holds at send time.
pub fn plan_configs(device: &dyn Device, args: &Args) -> Result<ConfigPlan> {
    let mut items = Vec::new();
    for (config, text) in args.configs.iter().enumerate() {
        let parsed = option_args(text)?;
        let group = resolve_scope(device, parsed.scope.as_deref(), args)?;
        for (name, v) in parsed.items {
            let key = resolve_key(device, &name, group.as_deref())?;
            let value = option_value(&name, v.as_deref(), key.kind())?;
            let access = device.option(&key, group.as_deref()).map(|o| o.access);
            if access.is_none_or(|a| !a.set) {
                return Err(Error::Arg(format!(
                    "Failed to set device option '{name}': not applicable."
                )));
            }
            oxrok::core::check_value(&key, &value)
                .map_err(|e| Error::Arg(format!("Failed to set device option '{name}': {e}.")))?;
            items.push(ConfigItem { name, key, value, group: group.clone(), config });
        }
    }
    Ok(order_configs(items))
}

fn send_configs(device: &mut dyn Device, items: &[ConfigItem]) -> Result<()> {
    for item in items {
        let k = &item.name;
        let group = item.group.as_deref();
        oxrok::core::config_set(device, &item.key, &item.value, group)
            .map_err(|e| Error::Arg(format!("Failed to set device option '{k}': {e}.")))?;
    }
    Ok(())
}

impl ConfigPlan {
    /// Whether this switches an output on.
    pub fn turns_on(&self) -> bool {
        self.enables.iter().any(|item| item.is_enabled(true))
    }

    /// Sends everything but what switches an output on, stopping at the
    /// first error.
    pub fn send_settings(&self, device: &mut dyn Device) -> Result<()> {
        send_configs(device, &self.settings)
    }

    /// Sends what switches an output on: the last step, so an output goes
    /// on only when all else was accepted.
    pub fn send_enables(&self, device: &mut dyn Device) -> Result<()> {
        send_configs(device, &self.enables)
    }
}

/// Applies every `-c` option to the device; see [`order_configs`] for the
/// order.
pub fn apply_configs(device: &mut dyn Device, args: &Args) -> Result<()> {
    let plan = plan_configs(&*device, args)?;
    plan.send_settings(device)?;
    plan.send_enables(device)
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
    fn a_misplaced_scope_sends_nothing() {
        for bad in ["channel_group=R1:enabled=on:channel_group=R2", "enabled=on:channel_group=R1"] {
            let (sent, result) = run_configs(&["-c", bad]);
            let e = result.unwrap_err();
            assert!(
                matches!(&e, Error::Arg(m) if m == "Unknown device option 'channel_group'."),
                "{bad}: {e}"
            );
            assert_eq!(sent, [] as [&str; 0], "{bad}");
        }
        let (sent, result) = run_configs(&["-c", ":enabled=on"]);
        let e = result.unwrap_err();
        assert!(matches!(&e, Error::Arg(m) if m.contains("first element is empty")), "{e}");
        assert_eq!(sent, [] as [&str; 0]);
        // An empty element further on is skipped, as in sigrok-cli.
        let (sent, result) = run_configs(&["-c", "enabled=on::"]);
        result.unwrap();
        assert_eq!(sent, ["enabled=true"]);
    }

    fn demo(logic: i32) -> Box<dyn Device> {
        let mut r = Registry::new();
        r.register(Box::new(oxrok::drivers::demo::DemoDriver));
        let (d, o) =
            driver_spec(&r, &format!("demo:logic_channels={logic}:analog_channels=1")).unwrap();
        d.scan(&o).unwrap().remove(0)
    }

    #[test]
    fn an_option_without_a_value_is_worded_as_in_c() {
        let mut r = Registry::new();
        r.register(Box::new(oxrok::drivers::demo::DemoDriver));
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
    fn selects_and_renames_channels() {
        let mut dev = demo(4);
        select_channels(dev.as_mut(), "D0,D2=clk,A0").unwrap();
        let enabled: Vec<&str> =
            dev.channels().iter().filter(|c| c.enabled).map(|c| c.name.as_str()).collect();
        assert_eq!(enabled, ["D0", "clk", "A0"]);
        assert!(select_channels(dev.as_mut(), "D9").is_err());
    }

    /// A device with outputs that records what it is sent.
    struct Recorder {
        info: oxrok::core::DeviceInfo,
        channels: Vec<oxrok::core::Channel>,
        groups: Vec<oxrok::core::ChannelGroup>,
        sent: std::sync::Arc<std::sync::Mutex<Vec<String>>>,
    }

    impl Recorder {
        fn new(sent: &std::sync::Arc<std::sync::Mutex<Vec<String>>>) -> Self {
            Recorder {
                info: oxrok::core::DeviceInfo::default(),
                channels: Vec::new(),
                groups: ["R1", "R2", "CH1", "CH2"]
                    .map(|name| oxrok::core::ChannelGroup { name: name.into(), channels: vec![] })
                    .to_vec(),
                sent: std::sync::Arc::clone(sent),
            }
        }
    }

    /// A device that cannot be opened.
    struct Unopenable(Recorder);

    impl Device for Unopenable {
        fn info(&self) -> &oxrok::core::DeviceInfo {
            self.0.info()
        }
        fn classes(&self) -> &[oxrok::core::DeviceClass] {
            self.0.classes()
        }
        fn channels(&self) -> &[oxrok::core::Channel] {
            self.0.channels()
        }
        fn enable_channel(&mut self, pos: usize, on: bool) -> Result<()> {
            self.0.enable_channel(pos, on)
        }
        fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
            self.0.rename_channel(pos, name)
        }
        fn options(&self, group: Option<&str>) -> Result<Vec<oxrok::core::OptionDesc>> {
            self.0.options(group)
        }
        fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
            self.0.get(key, group)
        }
        fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
            self.0.set(key, value, group)
        }
        fn list(&self, key: &Key, group: Option<&str>) -> Result<oxrok::core::ValueList> {
            self.0.list(key, group)
        }
        fn open(&mut self) -> Result<()> {
            Err(Error::Busy)
        }
        fn start(&mut self, sink: oxrok::core::PacketSink) -> Result<()> {
            self.0.start(sink)
        }
        fn stop(&mut self) -> Result<()> {
            self.0.stop()
        }
    }

    #[test]
    fn a_failed_open_says_so_with_the_reason() {
        let mut dev = Unopenable(Recorder::new(&std::sync::Arc::default()));
        let e = open_device(&mut dev).unwrap_err();
        assert!(
            matches!(&e, Error::Arg(m) if m == "Failed to open device: device is busy."),
            "{e}"
        );
    }

    impl Device for Recorder {
        fn info(&self) -> &oxrok::core::DeviceInfo {
            &self.info
        }
        fn classes(&self) -> &[oxrok::core::DeviceClass] {
            &[]
        }
        fn channels(&self) -> &[oxrok::core::Channel] {
            &self.channels
        }
        fn channel_groups(&self) -> &[oxrok::core::ChannelGroup] {
            &self.groups
        }
        fn enable_channel(&mut self, _: usize, _: bool) -> Result<()> {
            Ok(())
        }
        fn rename_channel(&mut self, _: usize, _: &str) -> Result<()> {
            Ok(())
        }
        fn options(&self, group: Option<&str>) -> Result<Vec<oxrok::core::OptionDesc>> {
            let opt = |key| oxrok::core::OptionDesc::new(key, oxrok::core::Access::GET_SET);
            let mut list = vec![
                opt(Key::Enabled),
                opt(Key::VoltageTarget),
                opt(Key::CurrentLimit),
                opt(Key::OverCurrentProtectionEnabled),
                oxrok::core::OptionDesc::new(Key::Voltage, oxrok::core::Access::GET),
            ];
            // Not in a group.
            if group.is_none() {
                list.extend([opt(Key::LimitSamples), opt(Key::CaptureRatio)]);
            }
            Ok(list)
        }
        fn get(&self, key: &Key, _: Option<&str>) -> Result<Value> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
            if *key == Key::VoltageTarget && value.as_f64().is_some_and(|v| v > 31.0) {
                return Err(Error::arg("out of range"));
            }
            let scope = group.map_or(String::new(), |g| format!("{g}:"));
            self.sent.lock().unwrap().push(format!("{scope}{}={value}", key.id()));
            Ok(())
        }
        fn list(&self, key: &Key, _: Option<&str>) -> Result<oxrok::core::ValueList> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn start(&mut self, _: oxrok::core::PacketSink) -> Result<()> {
            Ok(())
        }
        fn stop(&mut self) -> Result<()> {
            Ok(())
        }
    }

    /// What `-c` arguments send, and how the command line ended.
    fn run_configs(extra: &[&str]) -> (Vec<String>, Result<()>) {
        use clap::Parser as _;
        let sent = std::sync::Arc::default();
        let mut dev = Recorder::new(&sent);
        let args = Args::try_parse_from([&["oxrok", "--set"], extra].concat()).unwrap();
        let result = apply_configs(&mut dev, &args);
        let sent = sent.lock().unwrap().clone();
        (sent, result)
    }

    #[test]
    fn a_bad_option_anywhere_sends_nothing() {
        for bad in [
            "enabled=on:voltage_target=abc",
            "enabled=on:volage_target=5",
            "voltage_target=5:enabled=on:current_limit",
        ] {
            let (sent, result) = run_configs(&["-c", bad]);
            assert!(result.is_err(), "{bad}");
            assert_eq!(sent, [] as [&str; 0], "{bad}");
        }
        // Across several -c too.
        let (sent, result) = run_configs(&["-c", "enabled=on", "-c", "volage_target=5"]);
        assert!(result.is_err());
        assert_eq!(sent, [] as [&str; 0]);
    }

    #[test]
    fn scope_and_settability_are_checked_with_nothing_sent() {
        for (argv, says) in [
            (
                &["-c", "channel_group=R1:enabled=on", "-c", "channel_group=R9:enabled=on"][..],
                "Invalid channel group 'R9'",
            ),
            (&["-g", "R9", "-c", "current_limit=1"][..], "Invalid channel group 'R9'"),
            // A key the device reads and does not set.
            (
                &["-c", "enabled=on", "-c", "voltage=1"][..],
                "Failed to set device option 'voltage': not applicable.",
            ),
            // A key of the device, not of a group.
            (
                &["-c", "channel_group=R1:enabled=on", "-c", "channel_group=R1:limit_samples=5"][..],
                "Failed to set device option 'limit_samples': not applicable.",
            ),
            // What the core refuses for every driver.
            (
                &["-c", "current_limit=1", "-c", "limit_samples=0"][..],
                "Failed to set device option 'limit_samples': invalid argument: it cannot be 0.",
            ),
            (
                &["-c", "current_limit=1:captureratio=101"][..],
                "Failed to set device option 'captureratio': invalid argument: it must be 0..100.",
            ),
        ] {
            let (sent, result) = run_configs(argv);
            let e = result.unwrap_err();
            assert!(matches!(&e, Error::Arg(m) if m == says), "{argv:?}: {e}");
            assert_eq!(sent, [] as [&str; 0], "{argv:?}");
        }
    }

    #[test]
    fn a_group_goes_to_the_device_as_it_spells_it() {
        let (sent, result) = run_configs(&["-g", "r1", "-c", "enabled=on"]);
        result.unwrap();
        assert_eq!(sent, ["R1:enabled=true"]);
        let (sent, result) = run_configs(&["-c", "CHANNEL_GROUP=ch2:enabled=on"]);
        result.unwrap();
        assert_eq!(sent, ["CH2:enabled=true"]);
    }

    #[test]
    fn global_is_the_device() {
        for argv in [
            &["-g", "global", "-c", "enabled=on"][..],
            &["-c", "channel_group=GLOBAL:enabled=on"][..],
            // Over -g, as a -c's own scope is.
            &["-g", "R1", "-c", "channel_group=global:enabled=on"][..],
        ] {
            let (sent, result) = run_configs(argv);
            result.unwrap();
            assert_eq!(sent, ["enabled=true"], "{argv:?}");
        }
        // The same scope as none, for the last word and the order.
        let (sent, _) =
            run_configs(&["-c", "channel_group=global:enabled=on", "-c", "enabled=off"]);
        assert_eq!(sent, ["enabled=false"]);
        let (sent, _) = run_configs(&[
            "-c",
            "channel_group=global:enabled=on",
            "-c",
            "channel_group=R1:enabled=off",
        ]);
        assert_eq!(sent, ["enabled=true", "R1:enabled=false"]);
        let (sent, _) = run_configs(&[
            "-c",
            "channel_group=R1:enabled=on",
            "-c",
            "channel_group=global:enabled=off",
        ]);
        assert_eq!(sent, ["enabled=false"]);
    }

    #[test]
    fn an_empty_group_is_refused() {
        for argv in [&["-c", "channel_group=:enabled=on"][..], &["-g", "", "-c", "enabled=on"][..]]
        {
            let (sent, result) = run_configs(argv);
            let e = result.unwrap_err();
            assert!(matches!(&e, Error::Arg(m) if m == "Invalid channel group ''"), "{e}");
            assert_eq!(sent, [] as [&str; 0]);
        }
    }

    #[test]
    fn a_plan_knows_whether_it_turns_an_output_on() {
        use clap::Parser as _;
        let dev = Recorder::new(&std::sync::Arc::default());
        let turns_on = |c: &str| {
            let args = Args::try_parse_from(["oxrok", "--set", "-c", c]).unwrap();
            plan_configs(&dev, &args).unwrap().turns_on()
        };
        assert!(turns_on("enabled=on"));
        assert!(turns_on("channel_group=R1:enabled"));
        assert!(!turns_on("enabled=off"));
        assert!(!turns_on("voltage_target=5"));
    }

    #[test]
    fn a_device_without_groups_has_no_scope() {
        use clap::Parser as _;
        let sent = std::sync::Arc::default();
        let mut dev = Recorder::new(&sent);
        dev.groups.clear();
        let args =
            Args::try_parse_from(["oxrok", "--set", "-g", "R1", "-c", "enabled=on"]).unwrap();
        let e = apply_configs(&mut dev, &args).unwrap_err();
        assert!(
            matches!(&e, Error::Arg(m) if m == "This device does not have any channel groups."),
            "{e}"
        );
        assert_eq!(sent.lock().unwrap().len(), 0);
    }

    #[test]
    fn an_output_goes_on_only_after_its_setpoints_were_accepted() {
        let (sent, result) = run_configs(&["-c", "enabled=on:voltage_target=99"]);
        assert!(result.is_err());
        assert_eq!(sent, [] as [&str; 0]);
        let (sent, result) = run_configs(&["-c", "enabled=on:current_limit=1:voltage_target=99"]);
        assert!(result.is_err());
        assert_eq!(sent, ["current_limit=1"]);
        let (sent, result) = run_configs(&["-c", "enabled=on", "-c", "voltage_target=99"]);
        assert!(result.is_err());
        assert_eq!(sent, [] as [&str; 0]);
    }

    #[test]
    fn enabled_off_goes_first_and_on_last_the_others_as_given() {
        let (sent, result) =
            run_configs(&["-c", "enabled=on:current_limit=1:voltage_target=5:ocp_enabled=on"]);
        result.unwrap();
        assert_eq!(
            sent,
            ["current_limit=1", "voltage_target=5", "ocp_enabled=true", "enabled=true"]
        );
        let (sent, result) = run_configs(&["-c", "voltage_target=5:enabled=off:current_limit=1"]);
        result.unwrap();
        assert_eq!(sent, ["enabled=false", "voltage_target=5", "current_limit=1"]);
        // A bare `enabled` is on.
        let (sent, result) = run_configs(&["-c", "enabled:voltage_target=5"]);
        result.unwrap();
        assert_eq!(sent, ["voltage_target=5", "enabled=true"]);
    }

    #[test]
    fn sorting_does_not_change_where_an_output_ends_up() {
        // Last one wins, in one -c as in sigrok-cli's hash table and
        // across them as in its sequence.
        let (sent, _) = run_configs(&["-c", "enabled=on:voltage_target=5:enabled=off"]);
        assert_eq!(sent, ["enabled=false", "voltage_target=5"]);
        let (sent, _) = run_configs(&["-c", "enabled=on", "-c", "enabled=off"]);
        assert_eq!(sent, ["enabled=false"]);
        let (sent, _) = run_configs(&["-c", "enabled=off", "-c", "enabled=on"]);
        assert_eq!(sent, ["enabled=true"]);
        // Other keys: one value per -c, every -c sent.
        let (sent, _) = run_configs(&["-c", "voltage_target=1:voltage_target=2"]);
        assert_eq!(sent, ["voltage_target=2"]);
        let (sent, _) = run_configs(&["-c", "voltage_target=1", "-c", "voltage_target=2"]);
        assert_eq!(sent, ["voltage_target=1", "voltage_target=2"]);
        // All on but one group: the order given.
        let (sent, _) = run_configs(&["-c", "enabled=on", "-c", "channel_group=R1:enabled=off"]);
        assert_eq!(sent, ["enabled=true", "R1:enabled=false"]);
        // All off after one group: no pulse. What comes after it stays.
        let (sent, _) = run_configs(&["-c", "channel_group=R1:enabled=on", "-c", "enabled=off"]);
        assert_eq!(sent, ["enabled=false"]);
        let (sent, _) = run_configs(&[
            "-c",
            "channel_group=R1:enabled=on",
            "-c",
            "enabled=off",
            "-c",
            "channel_group=R2:enabled=on",
        ]);
        assert_eq!(sent, ["enabled=false", "R2:enabled=true"]);
        // An off on another group overrides nothing.
        let (sent, _) = run_configs(&[
            "-c",
            "channel_group=R1:enabled=on",
            "-c",
            "channel_group=R2:enabled=off",
        ]);
        assert_eq!(sent, ["R2:enabled=false", "R1:enabled=true"]);
        // Groups that do not overlap: off first, on last. -g and a group
        // named another way are the same scope, and global is the device's.
        let (sent, _) = run_configs(&[
            "-c",
            "channel_group=CH1:enabled=on",
            "-c",
            "channel_group=CH2:enabled=off",
            "-c",
            "channel_group=CH1:voltage_target=5",
        ]);
        assert_eq!(sent, ["CH2:enabled=false", "CH1:voltage_target=5", "CH1:enabled=true"]);
        let (sent, _) =
            run_configs(&["-g", "ch1", "-c", "enabled=off", "-c", "channel_group=CH1:enabled=on"]);
        assert_eq!(sent, ["CH1:enabled=true"]);
    }

    #[test]
    fn a_boolean_must_be_clearly_on_or_off() {
        for (text, sent) in [
            ("on", "true"),
            ("TRUE", "true"),
            ("Yes", "true"),
            ("1", "true"),
            ("off", "false"),
            ("FALSE", "false"),
            ("no", "false"),
            ("0", "false"),
        ] {
            let (got, result) = run_configs(&["-c", &format!("ocp_enabled={text}")]);
            result.unwrap();
            assert_eq!(got, [format!("ocp_enabled={sent}")], "{text}");
        }
        for text in ["1x", "enable", "", "onward", "2", " on", "disable"] {
            for key in ["ocp_enabled", "enabled"] {
                let (got, result) = run_configs(&["-c", &format!("voltage_target=5:{key}={text}")]);
                let e = result.unwrap_err().to_string();
                assert!(e.contains(&format!("Invalid value: '{text}' for option '{key}'")), "{e}");
                assert_eq!(got, [] as [&str; 0], "{key}={text}");
            }
        }
        // Without a value it stays true.
        let (got, result) = run_configs(&["-c", "ocp_enabled"]);
        result.unwrap();
        assert_eq!(got, ["ocp_enabled=true"]);
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
