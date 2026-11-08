// Counterpart in sigrok:
//   sigrok-cli/parsers.c, session.c, device.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The `-c` options of a run: every key resolved and every value parsed
//! before anything is sent, and sent in an order that leaves outputs
//! where the command line says.

use super::{option_args, option_value, resolve_key};
use crate::core::{Device, Error, Key, Result, Value};

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

    /// An `enabled` as a `-c` would say it.
    fn spelled(&self) -> String {
        let group = self.group.as_ref().map_or(String::new(), |g| format!("channel_group={g}:"));
        let state = if self.is_enabled(true) { "on" } else { "off" };
        format!("{group}{}={state}", self.name)
    }

    /// What a warning says of `self`, dropped for `by`, a later `enabled`.
    fn dropped_for(&self, by: &ConfigItem) -> String {
        format!(
            "-c {} is not sent, as the later -c {} counts. A pulse takes two runs.",
            self.spelled(),
            by.spelled()
        )
    }
}

/// The `-c` options of a command line, every key resolved and every value
/// parsed, so that a mistake anywhere in them is found with nothing sent.
pub struct ConfigPlan {
    /// `enabled=off` first, then the others in the order given.
    settings: Vec<ConfigItem>,
    /// What switches an output on, after every setting was accepted.
    enables: Vec<ConfigItem>,
    /// A warning for each `enabled` of another `-c` that won't be sent, where
    /// sigrok-cli would have switched the output.
    dropped: Vec<String>,
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
/// - An `enabled` of another `-c` that goes for these reasons gets a warning, where
///   sigrok-cli would have switched the output: a pulse takes two runs.
/// - `enabled=off` on a group given after a device-wide `enabled=on` (`-c
///   enabled=on -c channel_group=R1:enabled=off`: all but R1) stays after it.
///   R1 does have power between the two.
/// - `range` goes before the other settings, after `enabled=off`: a driver may
///   scale what it writes by the range, so `current_limit=1:range=12A` must not
///   write the limit in the range before.
fn order_configs(items: Vec<ConfigItem>) -> ConfigPlan {
    let mut kept: Vec<ConfigItem> = Vec::with_capacity(items.len());
    let mut dropped = Vec::new();
    for item in items {
        kept.retain(|k| {
            let goes =
                k.same_setting(&item) && (k.config == item.config || item.key == Key::Enabled);
            if goes && k.config != item.config && k.value != item.value {
                dropped.push(k.dropped_for(&item));
            }
            !goes
        });
        if item.is_enabled(false) && item.group.is_none() {
            kept.retain(|k| {
                let goes = k.is_enabled(true) && k.group.is_some();
                if goes {
                    dropped.push(k.dropped_for(&item));
                }
                !goes
            });
        }
        kept.push(item);
    }
    let mut offs = Vec::new();
    let mut ranges = Vec::new();
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
        } else if item.key == Key::Range {
            ranges.push(item);
        } else {
            others.push(item);
        }
    }
    offs.append(&mut ranges);
    offs.append(&mut others);
    ConfigPlan { settings: offs, enables, dropped }
}

/// Whether an empty group name (`channel_group=`, `-g ""`) means the whole
/// device, as in sigrok-cli. Decided 2026-11-05: not for now, as a shell
/// variable that came out empty should not switch every output, and
/// `channel_group=global` says it outright. To follow sigrok-cli, set this to
/// true, delete the test `an_empty_group_is_refused` and the sentence in
/// docs/CLI.md that names this constant.
const EMPTY_GROUP_IS_DEVICE_WIDE: bool = false;

/// The scope of a `-c`, a `--get` or a `--show`: its own `channel_group=`,
/// else `default` (`-g`), as the device spells it whatever the case given,
/// and `None` for the device, `global` in any case, as sigrok-cli's
/// `lookup_channel_group()` (device.c). Refused in its words, which end
/// sigrok-cli too (a critical message exits, main.c:46-47), though after
/// what it already sent. An empty name: see [`EMPTY_GROUP_IS_DEVICE_WIDE`].
pub fn resolve_scope(
    device: &dyn Device,
    own: Option<&str>,
    default: Option<&str>,
) -> Result<Option<String>> {
    let Some(name) = own.or(default) else { return Ok(None) };
    if name.eq_ignore_ascii_case("global") || (EMPTY_GROUP_IS_DEVICE_WIDE && name.is_empty()) {
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

/// Resolves and checks every `-c` option (`configs`, in order, with
/// `group` the `-g` default): its scope, its key, its value, that the key
/// can be set in that scope and what the core refuses for every driver. No
/// driver's option list depends on what a `-c` sets (each answers from its
/// model and the group), so all this holds at send time.
pub fn plan_configs(
    device: &dyn Device,
    configs: &[String],
    group: Option<&str>,
) -> Result<ConfigPlan> {
    let mut items = Vec::new();
    for (config, text) in configs.iter().enumerate() {
        let parsed = option_args(text)?;
        let group = resolve_scope(device, parsed.scope.as_deref(), group)?;
        for (name, v) in parsed.items {
            let key = resolve_key(device, &name, group.as_deref())?;
            let value = option_value(&name, v.as_deref(), key.kind())?;
            let access = device.option(&key, group.as_deref()).map(|o| o.access);
            if access.is_none_or(|a| !a.set) {
                return Err(Error::Arg(format!(
                    "Failed to set device option '{name}': not applicable."
                )));
            }
            crate::core::check_value(&key, &value)
                .map_err(|e| Error::Arg(format!("Failed to set device option '{name}': {e}.")))?;
            device
                .check_set(&key, &value, group.as_deref())
                .map_err(|e| Error::Arg(format!("Failed to set device option '{name}': {e}.")))?;
            items.push(ConfigItem { name, key, value, group: group.clone(), config });
        }
    }
    let plan = order_configs(items);
    for warning in &plan.dropped {
        log::warn!("{warning}");
    }
    Ok(plan)
}

fn send_configs(device: &mut dyn Device, items: &[ConfigItem]) -> Result<()> {
    for item in items {
        let k = &item.name;
        let group = item.group.as_deref();
        crate::core::config_set(device, &item.key, &item.value, group)
            .map_err(|e| Error::Arg(format!("Failed to set device option '{k}': {e}.")))?;
    }
    Ok(())
}

impl ConfigPlan {
    /// Whether this switches an output on.
    pub fn turns_on(&self) -> bool {
        self.enables.iter().any(|item| item.is_enabled(true))
    }

    /// The warnings for `enabled` options that are not sent.
    pub fn dropped(&self) -> &[String] {
        &self.dropped
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
pub fn apply_configs(
    device: &mut dyn Device,
    configs: &[String],
    group: Option<&str>,
) -> Result<()> {
    let plan = plan_configs(&*device, configs, group)?;
    plan.send_settings(device)?;
    plan.send_enables(device)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{
        Access, Channel, ChannelGroup, DeviceClass, DeviceInfo, OptionDesc, PacketSink, ValueList,
    };
    use std::sync::{Arc, Mutex};

    /// A device with outputs that records what it is sent.
    struct Recorder {
        info: DeviceInfo,
        channels: Vec<Channel>,
        groups: Vec<ChannelGroup>,
        sent: Arc<Mutex<Vec<String>>>,
    }

    impl Recorder {
        fn new(sent: &Arc<Mutex<Vec<String>>>) -> Self {
            Recorder {
                info: DeviceInfo::default(),
                channels: Vec::new(),
                groups: ["R1", "R2", "CH1", "CH2"]
                    .map(|name| ChannelGroup { name: name.into(), channels: vec![] })
                    .to_vec(),
                sent: Arc::clone(sent),
            }
        }
    }

    impl Device for Recorder {
        fn info(&self) -> &DeviceInfo {
            &self.info
        }
        fn classes(&self) -> &[DeviceClass] {
            &[]
        }
        fn channels(&self) -> &[Channel] {
            &self.channels
        }
        fn channel_groups(&self) -> &[ChannelGroup] {
            &self.groups
        }
        fn enable_channel(&mut self, _: usize, _: bool) -> Result<()> {
            Ok(())
        }
        fn rename_channel(&mut self, _: usize, _: &str) -> Result<()> {
            Ok(())
        }
        fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
            let opt = |key| OptionDesc::new(key, Access::GET_SET);
            let mut list = vec![
                opt(Key::Enabled),
                opt(Key::VoltageTarget),
                opt(Key::CurrentLimit),
                opt(Key::OverCurrentProtectionEnabled),
                opt(Key::Range),
                OptionDesc::new(Key::Voltage, Access::GET),
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
        fn list(&self, key: &Key, _: Option<&str>) -> Result<ValueList> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn start(&mut self, _: PacketSink) -> Result<()> {
            Ok(())
        }
        fn stop(&mut self) -> Result<()> {
            Ok(())
        }
    }

    /// A device that cannot be opened.
    struct Unopenable(Recorder);

    impl Device for Unopenable {
        fn info(&self) -> &DeviceInfo {
            self.0.info()
        }
        fn classes(&self) -> &[DeviceClass] {
            self.0.classes()
        }
        fn channels(&self) -> &[Channel] {
            self.0.channels()
        }
        fn enable_channel(&mut self, pos: usize, on: bool) -> Result<()> {
            self.0.enable_channel(pos, on)
        }
        fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
            self.0.rename_channel(pos, name)
        }
        fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
            self.0.options(group)
        }
        fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
            self.0.get(key, group)
        }
        fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
            self.0.set(key, value, group)
        }
        fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
            self.0.list(key, group)
        }
        fn open(&mut self) -> Result<()> {
            Err(Error::Busy)
        }
        fn start(&mut self, sink: PacketSink) -> Result<()> {
            self.0.start(sink)
        }
        fn stop(&mut self) -> Result<()> {
            self.0.stop()
        }
    }

    #[test]
    fn a_failed_open_says_so_with_the_reason() {
        let mut dev = Unopenable(Recorder::new(&Arc::default()));
        let e = super::super::open_device(&mut dev).unwrap_err();
        assert!(
            matches!(&e, Error::Arg(m) if m == "Failed to open device: device is busy."),
            "{e}"
        );
    }

    fn strings(items: &[&str]) -> Vec<String> {
        items.iter().map(|s| (*s).to_owned()).collect()
    }

    /// What `-c` arguments send with `-g group`, and how the run ended.
    fn run_with(configs: &[&str], group: Option<&str>) -> (Vec<String>, Result<()>) {
        let sent = Arc::default();
        let mut dev = Recorder::new(&sent);
        let result = apply_configs(&mut dev, &strings(configs), group);
        let sent = sent.lock().unwrap().clone();
        (sent, result)
    }

    /// What `-c` arguments send, and how the run ended.
    fn run_configs(configs: &[&str]) -> (Vec<String>, Result<()>) {
        run_with(configs, None)
    }

    #[test]
    fn a_misplaced_scope_sends_nothing() {
        for bad in ["channel_group=R1:enabled=on:channel_group=R2", "enabled=on:channel_group=R1"] {
            let (sent, result) = run_configs(&[bad]);
            let e = result.unwrap_err();
            assert!(
                matches!(&e, Error::Arg(m) if m == "Unknown device option 'channel_group'."),
                "{bad}: {e}"
            );
            assert_eq!(sent, [] as [&str; 0], "{bad}");
        }
        let (sent, result) = run_configs(&[":enabled=on"]);
        let e = result.unwrap_err();
        assert!(matches!(&e, Error::Arg(m) if m.contains("first element is empty")), "{e}");
        assert_eq!(sent, [] as [&str; 0]);
        // An empty element further on is skipped, as in sigrok-cli.
        let (sent, result) = run_configs(&["enabled=on::"]);
        result.unwrap();
        assert_eq!(sent, ["enabled=true"]);
    }

    #[test]
    fn a_bad_option_anywhere_sends_nothing() {
        for bad in [
            "enabled=on:voltage_target=abc",
            "enabled=on:volage_target=5",
            "voltage_target=5:enabled=on:current_limit",
        ] {
            let (sent, result) = run_configs(&[bad]);
            assert!(result.is_err(), "{bad}");
            assert_eq!(sent, [] as [&str; 0], "{bad}");
        }
        // Across several -c too.
        let (sent, result) = run_configs(&["enabled=on", "volage_target=5"]);
        assert!(result.is_err());
        assert_eq!(sent, [] as [&str; 0]);
    }

    #[test]
    fn scope_and_settability_are_checked_with_nothing_sent() {
        for (configs, group, says) in [
            (
                &["channel_group=R1:enabled=on", "channel_group=R9:enabled=on"][..],
                None,
                "Invalid channel group 'R9'",
            ),
            (&["current_limit=1"][..], Some("R9"), "Invalid channel group 'R9'"),
            // A key the device reads and does not set.
            (
                &["enabled=on", "voltage=1"][..],
                None,
                "Failed to set device option 'voltage': not applicable.",
            ),
            // A key of the device, not of a group.
            (
                &["channel_group=R1:enabled=on", "channel_group=R1:limit_samples=5"][..],
                None,
                "Failed to set device option 'limit_samples': not applicable.",
            ),
            // What the core refuses for every driver.
            (
                &["current_limit=1", "limit_samples=0"][..],
                None,
                "Failed to set device option 'limit_samples': invalid argument: it cannot be 0.",
            ),
            (
                &["current_limit=1:captureratio=101"][..],
                None,
                "Failed to set device option 'captureratio': invalid argument: it must be 0..100.",
            ),
        ] {
            let (sent, result) = run_with(configs, group);
            let e = result.unwrap_err();
            assert!(matches!(&e, Error::Arg(m) if m == says), "{configs:?}: {e}");
            assert_eq!(sent, [] as [&str; 0], "{configs:?}");
        }
    }

    #[test]
    fn a_group_goes_to_the_device_as_it_spells_it() {
        let (sent, result) = run_with(&["enabled=on"], Some("r1"));
        result.unwrap();
        assert_eq!(sent, ["R1:enabled=true"]);
        let (sent, result) = run_configs(&["CHANNEL_GROUP=ch2:enabled=on"]);
        result.unwrap();
        assert_eq!(sent, ["CH2:enabled=true"]);
    }

    #[test]
    fn global_is_the_device() {
        for (configs, group) in [
            (&["enabled=on"][..], Some("global")),
            (&["channel_group=GLOBAL:enabled=on"][..], None),
            // Over -g, as a -c's own scope is.
            (&["channel_group=global:enabled=on"][..], Some("R1")),
        ] {
            let (sent, result) = run_with(configs, group);
            result.unwrap();
            assert_eq!(sent, ["enabled=true"], "{configs:?}");
        }
        // The same scope as none, for the last word and the order.
        let (sent, _) = run_configs(&["channel_group=global:enabled=on", "enabled=off"]);
        assert_eq!(sent, ["enabled=false"]);
        let (sent, _) =
            run_configs(&["channel_group=global:enabled=on", "channel_group=R1:enabled=off"]);
        assert_eq!(sent, ["enabled=true", "R1:enabled=false"]);
        let (sent, _) =
            run_configs(&["channel_group=R1:enabled=on", "channel_group=global:enabled=off"]);
        assert_eq!(sent, ["enabled=false"]);
    }

    #[test]
    fn an_empty_group_is_refused() {
        for (configs, group) in
            [(&["channel_group=:enabled=on"][..], None), (&["enabled=on"][..], Some(""))]
        {
            let (sent, result) = run_with(configs, group);
            let e = result.unwrap_err();
            assert!(matches!(&e, Error::Arg(m) if m == "Invalid channel group ''"), "{e}");
            assert_eq!(sent, [] as [&str; 0]);
        }
    }

    #[test]
    fn a_plan_knows_whether_it_turns_an_output_on() {
        let dev = Recorder::new(&Arc::default());
        let turns_on = |c: &str| plan_configs(&dev, &strings(&[c]), None).unwrap().turns_on();
        assert!(turns_on("enabled=on"));
        assert!(turns_on("channel_group=R1:enabled"));
        assert!(!turns_on("enabled=off"));
        assert!(!turns_on("voltage_target=5"));
    }

    #[test]
    fn a_device_without_groups_has_no_scope() {
        let sent = Arc::default();
        let mut dev = Recorder::new(&sent);
        dev.groups.clear();
        let e = apply_configs(&mut dev, &strings(&["enabled=on"]), Some("R1")).unwrap_err();
        assert!(
            matches!(&e, Error::Arg(m) if m == "This device does not have any channel groups."),
            "{e}"
        );
        assert_eq!(sent.lock().unwrap().len(), 0);
    }

    #[test]
    fn an_output_goes_on_only_after_its_setpoints_were_accepted() {
        let (sent, result) = run_configs(&["enabled=on:voltage_target=99"]);
        assert!(result.is_err());
        assert_eq!(sent, [] as [&str; 0]);
        let (sent, result) = run_configs(&["enabled=on:current_limit=1:voltage_target=99"]);
        assert!(result.is_err());
        assert_eq!(sent, ["current_limit=1"]);
        let (sent, result) = run_configs(&["enabled=on", "voltage_target=99"]);
        assert!(result.is_err());
        assert_eq!(sent, [] as [&str; 0]);
    }

    #[test]
    fn range_goes_after_enabled_off_and_before_the_others() {
        let (sent, result) = run_configs(&["current_limit=1:range=12A:enabled=off"]);
        result.unwrap();
        assert_eq!(sent, ["enabled=false", "range=12A", "current_limit=1"]);
        let (sent, result) = run_configs(&["current_limit=1", "range=12A"]);
        result.unwrap();
        assert_eq!(sent, ["range=12A", "current_limit=1"]);
        let (sent, result) = run_configs(&["enabled=on:range=12A"]);
        result.unwrap();
        assert_eq!(sent, ["range=12A", "enabled=true"]);
    }

    #[test]
    fn enabled_off_goes_first_and_on_last_the_others_as_given() {
        let (sent, result) =
            run_configs(&["enabled=on:current_limit=1:voltage_target=5:ocp_enabled=on"]);
        result.unwrap();
        assert_eq!(
            sent,
            ["current_limit=1", "voltage_target=5", "ocp_enabled=true", "enabled=true"]
        );
        let (sent, result) = run_configs(&["voltage_target=5:enabled=off:current_limit=1"]);
        result.unwrap();
        assert_eq!(sent, ["enabled=false", "voltage_target=5", "current_limit=1"]);
        // A bare `enabled` is on.
        let (sent, result) = run_configs(&["enabled:voltage_target=5"]);
        result.unwrap();
        assert_eq!(sent, ["voltage_target=5", "enabled=true"]);
    }

    #[test]
    fn a_dropped_enabled_is_named_and_nothing_else_is() {
        let dev = Recorder::new(&Arc::default());
        let dropped =
            |configs: &[&str]| plan_configs(&dev, &strings(configs), None).unwrap().dropped;
        assert_eq!(
            dropped(&["enabled=on", "enabled=off"]),
            [
                "-c enabled=on is not sent, as the later -c enabled=off counts. A pulse takes two runs."
            ]
        );
        assert_eq!(
            dropped(&["channel_group=R1:enabled=off", "channel_group=R1:enabled=on"]),
            [
                "-c channel_group=R1:enabled=off is not sent, as the later -c channel_group=R1:enabled=on counts. A pulse takes two runs."
            ]
        );
        // A device-wide off takes a group's on.
        assert_eq!(
            dropped(&["channel_group=R1:enabled=on", "enabled=off"]),
            [
                "-c channel_group=R1:enabled=on is not sent, as the later -c enabled=off counts. A pulse takes two runs."
            ]
        );
        // Each that goes, in turn.
        assert_eq!(dropped(&["enabled=on", "enabled=off", "enabled=on"]).len(), 2);
        // Nothing lost: one -c keeps one value in sigrok-cli too, a repeat switches
        // nothing, other scopes and other keys stay.
        for configs in [
            &["enabled=on:enabled=off"][..],
            &["enabled=on", "enabled=on"],
            &["enabled=on", "channel_group=R1:enabled=off"],
            &["channel_group=R1:enabled=on", "channel_group=R2:enabled=off"],
            &["voltage_target=1", "voltage_target=2"],
        ] {
            assert_eq!(dropped(configs), Vec::<String>::new(), "{configs:?}");
        }
    }

    #[test]
    fn sorting_does_not_change_where_an_output_ends_up() {
        // Last one wins, in one -c as in sigrok-cli's hash table and
        // across them as in its sequence.
        let (sent, _) = run_configs(&["enabled=on:voltage_target=5:enabled=off"]);
        assert_eq!(sent, ["enabled=false", "voltage_target=5"]);
        let (sent, _) = run_configs(&["enabled=on", "enabled=off"]);
        assert_eq!(sent, ["enabled=false"]);
        let (sent, _) = run_configs(&["enabled=off", "enabled=on"]);
        assert_eq!(sent, ["enabled=true"]);
        // Other keys: one value per -c, every -c sent.
        let (sent, _) = run_configs(&["voltage_target=1:voltage_target=2"]);
        assert_eq!(sent, ["voltage_target=2"]);
        let (sent, _) = run_configs(&["voltage_target=1", "voltage_target=2"]);
        assert_eq!(sent, ["voltage_target=1", "voltage_target=2"]);
        // All on but one group: the order given.
        let (sent, _) = run_configs(&["enabled=on", "channel_group=R1:enabled=off"]);
        assert_eq!(sent, ["enabled=true", "R1:enabled=false"]);
        // All off after one group: no pulse. What comes after it stays.
        let (sent, _) = run_configs(&["channel_group=R1:enabled=on", "enabled=off"]);
        assert_eq!(sent, ["enabled=false"]);
        let (sent, _) = run_configs(&[
            "channel_group=R1:enabled=on",
            "enabled=off",
            "channel_group=R2:enabled=on",
        ]);
        assert_eq!(sent, ["enabled=false", "R2:enabled=true"]);
        // An off on another group overrides nothing.
        let (sent, _) =
            run_configs(&["channel_group=R1:enabled=on", "channel_group=R2:enabled=off"]);
        assert_eq!(sent, ["R2:enabled=false", "R1:enabled=true"]);
        // Groups that do not overlap: off first, on last. -g and a group
        // named another way are the same scope, and global is the device's.
        let (sent, _) = run_configs(&[
            "channel_group=CH1:enabled=on",
            "channel_group=CH2:enabled=off",
            "channel_group=CH1:voltage_target=5",
        ]);
        assert_eq!(sent, ["CH2:enabled=false", "CH1:voltage_target=5", "CH1:enabled=true"]);
        let (sent, _) = run_with(&["enabled=off", "channel_group=CH1:enabled=on"], Some("ch1"));
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
            let (got, result) = run_configs(&[&format!("ocp_enabled={text}")]);
            result.unwrap();
            assert_eq!(got, [format!("ocp_enabled={sent}")], "{text}");
        }
        for text in ["1x", "enable", "", "onward", "2", " on", "disable"] {
            for key in ["ocp_enabled", "enabled"] {
                let (got, result) = run_configs(&[&format!("voltage_target=5:{key}={text}")]);
                let e = result.unwrap_err().to_string();
                assert!(e.contains(&format!("Invalid value: '{text}' for option '{key}'")), "{e}");
                assert_eq!(got, [] as [&str; 0], "{key}={text}");
            }
        }
        // Without a value it stays true.
        let (got, result) = run_configs(&["ocp_enabled"]);
        result.unwrap();
        assert_eq!(got, ["ocp_enabled=true"]);
    }
}
