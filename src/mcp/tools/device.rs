// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The device tools: finding, opening, describing and configuring.

use crate::core::util::samplerate_string;
use crate::core::{Access, Device, Key, Value as CoreValue, ValueList};
use crate::mcp::Host;
use crate::mcp::host::ScanEntry;
use crate::mcp::tool::{Annotations, Tag, Tool, ToolError, ToolResult, ToolSpec, function};
use crate::setup;
use schemars::JsonSchema;
use serde::Deserialize;
use serde_json::{Value, json};
use std::sync::Arc;

/// `scan`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ScanArgs {
    /// A driver spec as `-d` takes it, with scan options:
    /// `demo:logic_channels=4`, `fx2lafw`, `rigol-ds:conn=tcp-raw/10.0.0.5/5555`.
    /// Every driver when left out, which probes serial ports and takes a
    /// while.
    #[serde(default)]
    pub driver: Option<String>,
}

/// `open`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OpenArgs {
    /// A `device_id` from the last `scan`.
    #[serde(default)]
    pub device_id: Option<String>,
    /// Or a driver spec to scan now, which must find one device.
    #[serde(default)]
    pub driver: Option<String>,
}

/// A handle alone.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct HandleArgs {
    /// The handle `open` returned.
    pub handle: u64,
}

/// `describe_device`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DescribeArgs {
    /// The handle `open` returned.
    pub handle: u64,
    /// Only the options of this channel group; every scope otherwise.
    #[serde(default)]
    pub group: Option<String>,
}

/// `get_option`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct GetArgs {
    /// The handle `open` returned.
    pub handle: u64,
    /// The option key: `samplerate`, `voltage_target`, `pattern`, ...
    pub key: String,
    /// The channel group, for options a group has; `global` or left out
    /// for the device's own.
    #[serde(default)]
    pub group: Option<String>,
}

/// `set_option`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SetArgs {
    /// The handle `open` returned.
    pub handle: u64,
    /// The option key: `samplerate`, `voltage_target`, `enabled`, ...
    pub key: String,
    /// The value in sigrok's text syntax: `1m` or `200k` for a rate,
    /// `1.5` for a voltage, `on`/`off` for a switch, a name from the
    /// option's list. A boolean key with no value is `on`.
    #[serde(default)]
    pub value: Option<String>,
    /// The channel group, for options a group has.
    #[serde(default)]
    pub group: Option<String>,
}

/// `enable_channels`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ChannelsArgs {
    /// The handle `open` returned.
    pub handle: u64,
    /// The channels to enable, as `-C` takes them: names, numeric ranges
    /// (`0-3`), and `name=newname` to rename. Every other channel is
    /// disabled.
    pub channels: Vec<String>,
}

/// The device tools over `host`.
#[allow(clippy::too_many_lines)]
pub fn tools(host: &Arc<Host>) -> Vec<Arc<dyn Tool>> {
    let h = host.clone();
    let scan = function::<ScanArgs, _>(
        ToolSpec {
            name: "scan",
            title: "Scan for devices",
            description: "Finds devices: with a driver spec (`demo`, `fx2lafw`, \
                          `demo:logic_channels=4`), or every driver when none is given. \
                          Returns each device's `device_id` for `open`, its vendor, model, \
                          version, serial, connection, classes and channel names, and the \
                          handle when it is open already. Finding a device does not open it.",
            example: json!({"driver": "demo:logic_channels=4:analog_channels=1"}),
            annotations: Annotations {
                read_only: true,
                destructive: false,
                idempotent: true,
                open_world: true,
            },
            tags: &[Tag::Device],
        },
        move |args, _| {
            let entries = h.scan(args.driver.as_deref())?;
            let devices: Vec<Value> = entries.iter().map(scan_entry).collect();
            Ok(ToolResult::structured(json!({"count": devices.len(), "devices": devices})))
        },
    );

    let h = host.clone();
    let open = function::<OpenArgs, _>(
        ToolSpec {
            name: "open",
            title: "Open a device",
            description: "Opens a device found by `scan` (by `device_id`) or the one device a \
                          `driver` spec finds now, and returns its `handle` with everything \
                          `describe_device` reports. Open devices are closed when the \
                          server exits. Opening may upload firmware and takes the device \
                          from other programs.",
            example: json!({"driver": "demo:logic_channels=4"}),
            annotations: Annotations::HARDWARE,
            tags: &[Tag::Device, Tag::Hardware],
        },
        move |args, _| {
            let handle = h.open(args.device_id.as_deref(), args.driver.as_deref())?;
            let v = h.with_device(handle, |id, d| Ok(describe(handle, id, d, None)))?;
            Ok(ToolResult::structured(v))
        },
    );

    let h = host.clone();
    let close = function::<HandleArgs, _>(
        ToolSpec {
            name: "close",
            title: "Close a device",
            description: "Closes an open device and frees its handle. Captures made from it \
                          stay in the store.",
            example: json!({"handle": 1}),
            annotations: Annotations::MUTATING,
            tags: &[Tag::Device],
        },
        move |args, _| {
            h.close(args.handle)?;
            Ok(ToolResult::structured(json!({"closed": args.handle})))
        },
    );

    let h = host.clone();
    let describe_device = function::<DescribeArgs, _>(
        ToolSpec {
            name: "describe_device",
            title: "Describe a device",
            description: "An open device: identity, classes, channels (index, name, kind, \
                          enabled), channel groups, and every option with its access (get, \
                          set, list), current value and allowed values or range, in the \
                          device's scope and in each group's. What `--show` prints, as data.",
            example: json!({"handle": 1}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Device],
        },
        move |args, _| {
            let v = h.with_device(args.handle, |id, d| {
                Ok(describe(args.handle, id, d, args.group.as_deref()))
            })?;
            Ok(ToolResult::structured(v))
        },
    );

    let h = host.clone();
    let get_option = function::<GetArgs, _>(
        ToolSpec {
            name: "get_option",
            title: "Get an option",
            description: "Reads one option of an open device: its value as a number, string \
                          or boolean, and as sigrok's text.",
            example: json!({"handle": 1, "key": "samplerate"}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Device],
        },
        move |args, _| {
            let v = h.with_device(args.handle, |_, d| {
                let group = setup::resolve_scope(d, args.group.as_deref(), None)?;
                let key = setup::resolve_key(d, &args.key, group.as_deref())?;
                let readable = d.option(&key, group.as_deref()).is_some_and(|o| o.access.get);
                if !readable {
                    return Err(ToolError::new(format!(
                        "Failed to get '{}': not applicable",
                        args.key
                    )));
                }
                let value = d
                    .get(&key, group.as_deref())
                    .map_err(|e| ToolError::new(format!("Failed to get '{}': {e}", args.key)))?;
                Ok(option_value(&key, group.as_deref(), &value))
            })?;
            Ok(ToolResult::structured(v))
        },
    );

    let h = host.clone();
    let set_option = function::<SetArgs, _>(
        ToolSpec {
            name: "set_option",
            title: "Set an option",
            description: "Sets one option of an open device, with the checks `-c` makes: the \
                          key must exist in the scope and be settable, the value must parse \
                          and be allowed. Returns the value read back. On a supply or a load \
                          this changes a live output: `enabled=on` switches it on.",
            example: json!({"handle": 1, "key": "samplerate", "value": "1m"}),
            annotations: Annotations::HARDWARE,
            tags: &[Tag::Device, Tag::Hardware],
        },
        move |args, _| {
            let v = h.with_device(args.handle, |_, d| {
                let group = setup::resolve_scope(d, args.group.as_deref(), None)?;
                let key = setup::resolve_key(d, &args.key, group.as_deref())?;
                let value = setup::option_value(&args.key, args.value.as_deref(), key.kind())?;
                let settable = d.option(&key, group.as_deref()).is_some_and(|o| o.access.set);
                if !settable {
                    return Err(ToolError::new(format!(
                        "Failed to set device option '{}': not applicable.",
                        args.key
                    )));
                }
                crate::core::check_value(&key, &value).map_err(|e| {
                    ToolError::new(format!("Failed to set device option '{}': {e}.", args.key))
                })?;
                crate::core::config_set(d, &key, &value, group.as_deref()).map_err(|e| {
                    ToolError::new(format!("Failed to set device option '{}': {e}.", args.key))
                })?;
                let now = d.get(&key, group.as_deref()).unwrap_or(value);
                Ok(option_value(&key, group.as_deref(), &now))
            })?;
            Ok(ToolResult::structured(v))
        },
    );

    let h = host.clone();
    let enable_channels = function::<ChannelsArgs, _>(
        ToolSpec {
            name: "enable_channels",
            title: "Enable channels",
            description: "Enables the channels named and disables the rest, renaming with \
                          `name=newname`; what `-C` does. Returns the channel list.",
            example: json!({"handle": 1, "channels": ["D0", "D1=sda"]}),
            annotations: Annotations::MUTATING,
            tags: &[Tag::Device],
        },
        move |args, _| {
            let v = h.with_device(args.handle, |_, d| {
                setup::select_channels(d, &args.channels.join(","))
                    .map_err(|e| ToolError::new(format!("Failed to set channels: {e}")))?;
                Ok(json!({"handle": args.handle, "channels": channels(d)}))
            })?;
            Ok(ToolResult::structured(v))
        },
    );

    vec![scan, open, close, describe_device, get_option, set_option, enable_channels]
}

fn scan_entry(e: &ScanEntry) -> Value {
    json!({
        "device_id": e.device_id,
        "driver": e.info.driver,
        "vendor": e.info.vendor,
        "model": e.info.model,
        "version": e.info.version,
        "serial": e.info.serial,
        "conn": e.info.conn,
        "classes": e.classes.iter().map(|c| super::catalog::class_id(*c)).collect::<Vec<_>>(),
        "channels": e.channels.iter().map(|c| c.name.as_str()).collect::<Vec<_>>(),
        "handle": e.handle,
    })
}

/// The channel list of a device.
pub fn channels(d: &dyn Device) -> Vec<Value> {
    d.channels()
        .iter()
        .map(|c| {
            json!({
                "index": c.index,
                "name": c.name,
                "kind": format!("{:?}", c.kind).to_lowercase(),
                "enabled": c.enabled,
            })
        })
        .collect()
}

/// Everything about an open device.
pub fn describe(handle: u64, device_id: &str, d: &dyn Device, group: Option<&str>) -> Value {
    let info = d.info();
    let groups: Vec<Value> = d
        .channel_groups()
        .iter()
        .map(|g| {
            let names: Vec<&str> = g
                .channels
                .iter()
                .filter_map(|p| d.channels().get(*p))
                .map(|c| c.name.as_str())
                .collect();
            json!({"name": g.name, "channels": names})
        })
        .collect();
    let mut scopes: Vec<Option<String>> = Vec::new();
    if let Some(g) = group {
        scopes.push(Some(g.to_owned()));
    } else {
        scopes.push(None);
        scopes.extend(d.channel_groups().iter().map(|g| Some(g.name.clone())));
    }
    let mut options = Vec::new();
    for scope in scopes {
        let Ok(list) = d.options(scope.as_deref()) else { continue };
        for o in list {
            options.push(option_desc(d, &o.key, o.access, scope.as_deref()));
        }
    }
    json!({
        "handle": handle,
        "device_id": device_id,
        "driver": info.driver,
        "vendor": info.vendor,
        "model": info.model,
        "version": info.version,
        "serial": info.serial,
        "conn": info.conn,
        "classes": d.classes().iter().map(|c| super::catalog::class_id(*c)).collect::<Vec<_>>(),
        "channels": channels(d),
        "channel_groups": groups,
        "options": options,
    })
}

fn option_desc(d: &dyn Device, key: &Key, access: Access, group: Option<&str>) -> Value {
    let mut v = json!({
        "key": key.id(),
        "name": key.name(),
        "kind": super::catalog::kind_id(key.kind()),
        "group": group,
        "access": {"get": access.get, "set": access.set, "list": access.list},
    });
    if access.get
        && let Ok(value) = d.get(key, group)
    {
        v["value"] = value_json(&value);
        v["text"] = json!(value_text(key, &value));
    }
    if access.list
        && let Ok(list) = d.list(key, group)
    {
        v["list"] = list_json(&list);
    }
    v
}

/// A `get_option`/`set_option` answer.
fn option_value(key: &Key, group: Option<&str>, value: &CoreValue) -> Value {
    json!({
        "key": key.id(),
        "group": group,
        "value": value_json(value),
        "text": value_text(key, value),
    })
}

/// A value as JSON: numbers, strings and booleans where they fit.
pub fn value_json(v: &CoreValue) -> Value {
    match v {
        CoreValue::Bool(b) => json!(b),
        CoreValue::U64(n) => json!(n),
        CoreValue::I32(n) => json!(n),
        CoreValue::F64(f) => json!(f),
        CoreValue::Str(s) => json!(s),
        CoreValue::Rational(p, q) => json!({"p": p, "q": q}),
        CoreValue::RangeU64(a, b) => json!([a, b]),
        CoreValue::RangeF64(a, b) => json!([a, b]),
        CoreValue::KeyValue(k, val) => json!({k: val}),
        CoreValue::Mq(..) => json!(v.to_string()),
    }
}

/// A value as sigrok text: a sample rate with its unit, the rest as
/// `-c` would take it.
pub fn value_text(key: &Key, v: &CoreValue) -> String {
    match (key, v) {
        (Key::Samplerate, CoreValue::U64(rate)) => samplerate_string(*rate),
        _ => v.to_string(),
    }
}

/// The allowed values as JSON.
pub fn list_json(list: &ValueList) -> Value {
    match list {
        ValueList::U64(v) => json!(v),
        ValueList::RangeStepU64 { min, max, step } => json!({"min": min, "max": max, "step": step}),
        ValueList::MinMaxU64(min, max) => json!({"min": min, "max": max}),
        ValueList::F64(v) => json!(v),
        ValueList::RangeStepF64 { min, max, step } => json!({"min": min, "max": max, "step": step}),
        ValueList::Str(v) => json!(v),
        ValueList::Rational(v) => {
            json!(v.iter().map(|(p, q)| json!({"p": p, "q": q})).collect::<Vec<_>>())
        }
        ValueList::TriggerMatches(v) => json!(v.iter().map(|m| m.code()).collect::<Vec<_>>()),
        ValueList::RangeF64(v) => json!(v),
        ValueList::RangeU64(v) => json!(v),
        ValueList::Mq(v) => json!(
            v.iter().map(|(mq, flags)| CoreValue::Mq(*mq, *flags).to_string()).collect::<Vec<_>>()
        ),
    }
}

#[cfg(test)]
#[allow(clippy::many_single_char_names)]
mod tests {
    use super::*;
    use crate::mcp::tool::Call;

    fn host() -> Arc<Host> {
        let mut r = crate::core::Registry::new();
        r.register(Box::new(crate::drivers::demo::DemoDriver));
        Arc::new(Host::with(r, crate::decode::Registry::new()))
    }

    fn call(host: &Arc<Host>, name: &str, args: Value) -> Result<Value, ToolError> {
        let tools = tools(host);
        let tool = tools.iter().find(|t| t.desc().name == name).unwrap();
        tool.call(args, &mut Call::detached()).map(|r| r.structured.unwrap())
    }

    #[test]
    fn the_demo_device_end_to_end() {
        let h = host();
        let s =
            call(&h, "scan", json!({"driver": "demo:logic_channels=4:analog_channels=1"})).unwrap();
        assert_eq!(s["count"], 1);
        assert_eq!(s["devices"][0]["device_id"], "demo");
        assert_eq!(s["devices"][0]["channels"], json!(["D0", "D1", "D2", "D3", "A0"]));
        let o = call(&h, "open", json!({"device_id": "demo"})).unwrap();
        assert_eq!(o["handle"], 1);
        assert_eq!(o["channels"].as_array().unwrap().len(), 5);
        let rate = o["options"]
            .as_array()
            .unwrap()
            .iter()
            .find(|opt| opt["key"] == "samplerate" && opt["group"].is_null())
            .unwrap();
        assert_eq!(rate["value"], 200_000);
        assert_eq!(rate["text"], "200 kHz");
        assert_eq!(rate["access"]["set"], true);
        assert!(rate["list"]["min"].is_number() || rate["list"].is_array(), "{rate}");
        let pattern = o["options"]
            .as_array()
            .unwrap()
            .iter()
            .find(|opt| opt["key"] == "pattern" && opt["group"] == "Logic")
            .unwrap();
        assert!(pattern["list"].as_array().unwrap().contains(&json!("walking-one")));

        let g = call(&h, "get_option", json!({"handle": 1, "key": "samplerate"})).unwrap();
        assert_eq!(g["value"], 200_000);
        let s = call(&h, "set_option", json!({"handle": 1, "key": "samplerate", "value": "1m"}))
            .unwrap();
        assert_eq!(s["value"], 1_000_000);
        assert_eq!(s["text"], "1 MHz");
        let s = call(
            &h,
            "set_option",
            json!({"handle": 1, "key": "pattern", "value": "walking-one", "group": "Logic"}),
        )
        .unwrap();
        assert_eq!(s["value"], "walking-one");
        let e =
            call(&h, "set_option", json!({"handle": 1, "key": "nope", "value": "1"})).unwrap_err();
        assert_eq!(e.message, "Unknown device option 'nope'.");
        let e = call(&h, "set_option", json!({"handle": 1, "key": "samplerate", "value": "fast"}))
            .unwrap_err();
        assert!(e.message.contains("fast"), "{e}");
        // A group's key asked of the device: resolved, not readable there.
        let e = call(&h, "get_option", json!({"handle": 1, "key": "pattern"})).unwrap_err();
        assert_eq!(e.message, "Failed to get 'pattern': not applicable");

        let c = call(&h, "enable_channels", json!({"handle": 1, "channels": ["D0", "D2=clk"]}))
            .unwrap();
        let enabled: Vec<&str> = c["channels"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|c| c["enabled"] == true)
            .map(|c| c["name"].as_str().unwrap())
            .collect();
        assert_eq!(enabled, ["D0", "clk"]);
        let e = call(&h, "enable_channels", json!({"handle": 1, "channels": ["D9"]})).unwrap_err();
        assert_eq!(e.message, "Failed to set channels: invalid argument: unknown channel 'D9'.");

        let d = call(&h, "describe_device", json!({"handle": 1, "group": "Logic"})).unwrap();
        assert!(d["options"].as_array().unwrap().iter().all(|o| o["group"] == "Logic"));
        assert_eq!(call(&h, "close", json!({"handle": 1})).unwrap()["closed"], 1);
        let e = call(&h, "describe_device", json!({"handle": 1})).unwrap_err();
        assert!(e.message.contains("No open device"), "{e}");
    }

    #[test]
    fn values_render() {
        assert_eq!(value_json(&CoreValue::Rational(1, 1000)), json!({"p": 1, "q": 1000}));
        assert_eq!(value_text(&Key::Samplerate, &CoreValue::U64(1_000_000)), "1 MHz");
        assert_eq!(value_text(&Key::VoltageTarget, &CoreValue::F64(1.5)), "1.5");
        assert_eq!(list_json(&ValueList::MinMaxU64(1, 10)), json!({"min": 1, "max": 10}));
        assert_eq!(list_json(&ValueList::Str(vec!["a".into()])), json!(["a"]));
    }
}
