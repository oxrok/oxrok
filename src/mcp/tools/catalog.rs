// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The reference tools: what oxrok supports. Read-only, no hardware.

use crate::core::{DeviceClass, Driver, FirmwareKind, Key, ValueKind, firmware};
use crate::decode::{DecoderInfo, OptionValue, RadixSupport};
use crate::mcp::Host;
use crate::mcp::tool::{Annotations, Tag, Tool, ToolError, ToolResult, ToolSpec, function};
use schemars::JsonSchema;
use serde::Deserialize;
use serde_json::{Value, json};
use std::sync::Arc;

const TAGS: &[Tag] = &[Tag::Catalog];

/// No arguments.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NoArgs {}

/// `list_drivers`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ListDriversArgs {
    /// Keep the drivers of one class: `logic-analyzer`, `oscilloscope`,
    /// `multimeter`, `power-supply`, `electronic-load`, `lcr-meter`,
    /// `signal-generator`, `demo-device`, ... as `list_drivers` reports
    /// them.
    #[serde(default)]
    pub class: Option<String>,
    /// Keep the drivers whose id or name contains this, case-insensitive.
    #[serde(default)]
    pub query: Option<String>,
}

/// `driver_info`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DriverArgs {
    /// The driver id, as `-d` takes it: `fx2lafw`, `demo`.
    pub driver: String,
}

/// `list_decoders`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ListDecodersArgs {
    /// Keep the decoders with this tag, case-insensitive: `Embedded/industrial`,
    /// `Memory`, `Util`, ...
    #[serde(default)]
    pub tag: Option<String>,
    /// Keep the decoders whose id, name or description contains this,
    /// case-insensitive.
    #[serde(default)]
    pub query: Option<String>,
}

/// `decoder_info`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DecoderArgs {
    /// The decoder id, as `-P` takes it: `i2c`, `uart`, `eeprom24xx`.
    pub decoder: String,
}

/// The reference tools over `host`.
#[allow(clippy::too_many_lines)]
pub fn tools(host: &Arc<Host>) -> Vec<Arc<dyn Tool>> {
    let h = host.clone();
    let version = function::<NoArgs, _>(
        ToolSpec {
            name: "version",
            title: "Version",
            description: "The oxrok version, the features built in (USB backends, the Python \
                          decoder runtime), where Python decoders are looked for, and how many \
                          drivers and decoders this build has. Call it first.",
            example: json!({}),
            annotations: Annotations::READ_ONLY,
            tags: TAGS,
        },
        move |_, _| Ok(ToolResult::structured(version(&h))),
    );

    let h = host.clone();
    let list_drivers = function::<ListDriversArgs, _>(
        ToolSpec {
            name: "list_drivers",
            title: "List drivers",
            description: "The device drivers: id (what `-d` takes), name, classes, scan option \
                          keys, and whether the driver needs firmware files. Filter by `class` \
                          or `query`; `driver_info` has the details of one.",
            example: json!({"class": "logic-analyzer"}),
            annotations: Annotations::READ_ONLY,
            tags: TAGS,
        },
        move |args, _| {
            let class = args.class.as_deref().map(str::to_lowercase);
            let query = args.query.as_deref().map(str::to_lowercase);
            let drivers: Vec<Value> = h
                .registry()
                .drivers()
                .filter(|d| {
                    class
                        .as_deref()
                        .is_none_or(|c| d.classes().iter().any(|k| class_matches(*k, c)))
                })
                .filter(|d| {
                    query.as_deref().is_none_or(|q| {
                        d.id().to_lowercase().contains(q)
                            || d.long_name().to_lowercase().contains(q)
                    })
                })
                .map(driver_summary)
                .collect();
            Ok(ToolResult::structured(json!({"count": drivers.len(), "drivers": drivers})))
        },
    );

    let h = host.clone();
    let driver_info = function::<DriverArgs, _>(
        ToolSpec {
            name: "driver_info",
            title: "Driver info",
            description: "One driver: classes, scan options with their value kinds (what \
                          `-d driver:key=value` accepts), the firmware files it needs with \
                          where each was found, and whether its captures end on their own.",
            example: json!({"driver": "demo"}),
            annotations: Annotations::READ_ONLY,
            tags: TAGS,
        },
        move |args, _| {
            let d = h
                .registry()
                .find(&args.driver)
                .ok_or_else(|| ToolError::new(format!("Unknown driver '{}'.", args.driver)))?;
            Ok(ToolResult::structured(driver_detail(d)))
        },
    );

    let h = host.clone();
    let list_decoders = function::<ListDecodersArgs, _>(
        ToolSpec {
            name: "list_decoders",
            title: "List decoders",
            description: "The protocol decoders: id (what `-P` takes), name, description, tags, \
                          what they take as input (`logic` or another decoder's output) and \
                          whether the implementation is native or Python. Filter by `tag` or \
                          `query`; `decoder_info` has the channels and options of one.",
            example: json!({"query": "i2c"}),
            annotations: Annotations::READ_ONLY,
            tags: TAGS,
        },
        move |args, _| {
            let tag = args.tag.as_deref().map(str::to_lowercase);
            let query = args.query.as_deref().map(str::to_lowercase);
            let decoders: Vec<Value> = h
                .decoders()
                .decoders()
                .map(|d| d.info)
                .filter(|i| {
                    tag.as_deref().is_none_or(|t| i.tags.iter().any(|x| x.to_lowercase() == t))
                })
                .filter(|i| {
                    query.as_deref().is_none_or(|q| {
                        [i.id, i.name, i.longname, i.desc]
                            .iter()
                            .any(|s| s.to_lowercase().contains(q))
                    })
                })
                .map(decoder_summary)
                .collect();
            Ok(ToolResult::structured(json!({"count": decoders.len(), "decoders": decoders})))
        },
    );

    let h = host.clone();
    let decoder_info = function::<DecoderArgs, _>(
        ToolSpec {
            name: "decoder_info",
            title: "Decoder info",
            description: "One decoder: required and optional channels (the roles `-P dec:role=D0` \
                          assigns), options with defaults and allowed values, annotation classes \
                          and rows (what `-A` selects), binary classes (`-B`), inputs, outputs, \
                          tags and the documentation.",
            example: json!({"decoder": "i2c"}),
            annotations: Annotations::READ_ONLY,
            tags: TAGS,
        },
        move |args, _| {
            let d = h
                .decoders()
                .find(&args.decoder)
                .ok_or_else(|| ToolError::new(format!("Unknown decoder '{}'.", args.decoder)))?;
            Ok(ToolResult::structured(decoder_detail(d.info)))
        },
    );

    let list_formats = function::<NoArgs, _>(
        ToolSpec {
            name: "list_formats",
            title: "List formats",
            description: "The input modules (`-I`, for reading files) and output modules (`-O`, \
                          for writing captures: `bits`, `csv`, `vcd`, `srzip`, ...) with their \
                          options and file extensions.",
            example: json!({}),
            annotations: Annotations::READ_ONLY,
            tags: TAGS,
        },
        |_, _| Ok(ToolResult::structured(formats())),
    );

    let h = host.clone();
    let firmware_status = function::<NoArgs, _>(
        ToolSpec {
            name: "firmware_status",
            title: "Firmware status",
            description: "The firmware and bitstream files the drivers need, where each was \
                          found or that it was not, where to get the missing ones, and the \
                          directories searched in order.",
            example: json!({}),
            annotations: Annotations::READ_ONLY,
            tags: TAGS,
        },
        move |_, _| Ok(ToolResult::structured(firmware_status(&h))),
    );

    vec![
        version,
        list_drivers,
        driver_info,
        list_decoders,
        decoder_info,
        list_formats,
        firmware_status,
    ]
}

/// `Logic analyzer` as `logic-analyzer`.
pub(crate) fn class_id(class: DeviceClass) -> String {
    class.name().to_lowercase().replace(' ', "-")
}

fn class_matches(class: DeviceClass, wanted: &str) -> bool {
    class_id(class) == wanted || class.name().to_lowercase() == wanted
}

/// `RationalPeriod` as `rational_period`.
pub(crate) fn kind_id(kind: ValueKind) -> String {
    snake(&format!("{kind:?}"))
}

fn snake(camel: &str) -> String {
    let mut out = String::new();
    for (i, c) in camel.chars().enumerate() {
        if c.is_uppercase() {
            if i > 0 {
                out.push('_');
            }
            out.extend(c.to_lowercase());
        } else {
            out.push(c);
        }
    }
    out
}

fn key(k: &Key) -> Value {
    json!({"key": k.id(), "name": k.name(), "kind": kind_id(k.kind())})
}

fn version(host: &Host) -> Value {
    let mut features = Vec::new();
    for (name, on) in [
        ("cli", cfg!(feature = "cli")),
        ("nusb", cfg!(feature = "nusb")),
        ("libusb", cfg!(feature = "libusb")),
        ("python", cfg!(feature = "python")),
        ("rustpython", cfg!(feature = "rustpython")),
    ] {
        if on {
            features.push(name);
        }
    }
    json!({
        "version": crate::core::VERSION,
        "features": features,
        "host": format!("{}-{}", std::env::consts::ARCH, std::env::consts::OS),
        "decoder_runtime": decoder_runtime(),
        "drivers": host.registry().drivers().count(),
        "decoders": host.decoders().decoders().count(),
    })
}

fn decoder_runtime() -> Value {
    #[cfg(any(feature = "python", feature = "rustpython"))]
    {
        let python = if cfg!(feature = "python") && cfg!(feature = "rustpython") {
            if std::env::var("OXROK_PYTHON").is_ok_and(|v| v == "rustpython") {
                "rustpython"
            } else {
                "cpython"
            }
        } else if cfg!(feature = "python") {
            "cpython"
        } else {
            "rustpython"
        };
        json!({
            "native": true,
            "python": python,
            "decoders_dir": crate::pydecode::decoders_dir(),
        })
    }
    #[cfg(not(any(feature = "python", feature = "rustpython")))]
    {
        json!({"native": true, "python": Value::Null, "decoders_dir": Value::Null})
    }
}

fn driver_summary(d: &dyn Driver) -> Value {
    json!({
        "id": d.id(),
        "name": d.long_name(),
        "classes": d.classes().iter().map(|c| class_id(*c)).collect::<Vec<_>>(),
        "scan_options": d.scan_options().iter().map(Key::id).collect::<Vec<_>>(),
        "needs_firmware": d.firmware().iter().any(|f| f.kind == FirmwareKind::Upload),
    })
}

fn firmware_kind(kind: FirmwareKind) -> &'static str {
    match kind {
        FirmwareKind::Upload => "upload",
        FirmwareKind::PreFlashed => "pre-flashed",
    }
}

fn firmware_of(d: &dyn Driver) -> Vec<Value> {
    d.firmware()
        .iter()
        .map(|dep| {
            let files: Vec<Value> = dep
                .files
                .iter()
                .map(|name| json!({"name": name, "path": firmware::find_file(name)}))
                .collect();
            // Where the files come from: a package `oxrok firmware fetch`
            // downloads, or one the user has to provide.
            let package = dep.files.first().and_then(|f| firmware::source_of(f));
            json!({
                "kind": firmware_kind(dep.kind),
                "files": files,
                "source": dep.source,
                "note": dep.note,
                "package": package.map(|p| p.id),
                "fetch": package.map(|p| if p.downloadable() { "download" } else { "manual" }),
            })
        })
        .collect()
}

fn driver_detail(d: &dyn Driver) -> Value {
    json!({
        "id": d.id(),
        "name": d.long_name(),
        "classes": d.classes().iter().map(|c| class_id(*c)).collect::<Vec<_>>(),
        "scan_options": d.scan_options().iter().map(key).collect::<Vec<_>>(),
        "accepted_scan_options": d.accepted_scan_options().iter().map(key).collect::<Vec<_>>(),
        "firmware": firmware_of(d),
        "ends_captures_itself": d.ends_captures_itself(),
    })
}

fn decoder_summary(i: &DecoderInfo) -> Value {
    json!({
        "id": i.id,
        "name": i.name,
        "longname": i.longname,
        "description": i.desc,
        "tags": i.tags,
        "inputs": i.inputs,
        "outputs": i.outputs,
        "native": is_native(i.id),
        "radix_support": RadixSupport::of(i, true).id(),
    })
}

fn is_native(id: &str) -> bool {
    crate::decoders::ALL.iter().any(|d| d.info.id == id)
}

fn option_value(v: &OptionValue) -> Value {
    match v {
        OptionValue::Int(i) => json!(i),
        OptionValue::Float(f) => json!(f),
        OptionValue::Str(s) => json!(s),
    }
}

fn decoder_detail(i: &DecoderInfo) -> Value {
    let channel = |c: &crate::decode::ChannelDesc| json!({"role": c.id, "name": c.name, "description": c.desc});
    let class_ids = |ix: &[usize]| -> Vec<&str> {
        ix.iter().filter_map(|&c| i.annotations.get(c).map(|a| a.id)).collect()
    };
    json!({
        "id": i.id,
        "name": i.name,
        "longname": i.longname,
        "description": i.desc,
        "license": i.license,
        "native": is_native(i.id),
        "radix_support": RadixSupport::of(i, true).id(),
        "default_radix": crate::decode::radix::default_radix(i).id(),
        "tags": i.tags,
        "inputs": i.inputs,
        "outputs": i.outputs,
        "channels": i.channels.iter().map(channel).collect::<Vec<_>>(),
        "optional_channels": i.optional_channels.iter().map(channel).collect::<Vec<_>>(),
        "options": i.options.iter().map(|o| json!({
            "id": o.id,
            "description": o.desc,
            "default": option_value(&o.default),
            "values": o.values.iter().map(option_value).collect::<Vec<_>>(),
        })).collect::<Vec<_>>(),
        "annotations": i.annotations.iter().map(|a| json!({"id": a.id, "description": a.desc})).collect::<Vec<_>>(),
        "annotation_rows": i.annotation_rows.iter().map(|r| json!({
            "id": r.id, "description": r.desc, "classes": class_ids(r.classes),
        })).collect::<Vec<_>>(),
        "binary": i.binary.iter().map(|b| json!({"id": b.id, "description": b.desc})).collect::<Vec<_>>(),
        "doc": i.doc,
    })
}

fn formats() -> Value {
    let input: Vec<Value> = crate::formats::input::modules()
        .iter()
        .map(|m| {
            json!({
                "id": m.id,
                "name": m.name,
                "description": m.description,
                "extensions": m.extensions,
                "options": m.options.iter().map(|o| json!({
                    "id": o.id, "name": o.name, "description": o.description,
                    "kind": snake(&format!("{:?}", o.kind)), "default": o.default,
                })).collect::<Vec<_>>(),
            })
        })
        .collect();
    let output: Vec<Value> = crate::formats::output::modules()
        .iter()
        .map(|m| {
            json!({
                "id": m.id,
                "name": m.name,
                "description": m.description,
                "extensions": m.extensions,
                "analog": m.analog,
                "options": m.options.iter().map(|o| json!({
                    "id": o.id, "name": o.name, "description": o.description, "default": o.default,
                })).collect::<Vec<_>>(),
            })
        })
        .collect();
    json!({"input": input, "output": output})
}

fn firmware_status(host: &Host) -> Value {
    let drivers: Vec<Value> = host
        .registry()
        .drivers()
        .filter(|d| !d.firmware().is_empty())
        .map(|d| json!({"driver": d.id(), "firmware": firmware_of(d)}))
        .collect();
    json!({
        "search_dirs": firmware::search_dirs(),
        "drivers": drivers,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcp::tool::Call;

    fn host() -> Arc<Host> {
        Arc::new(Host::new())
    }

    fn call(name: &str, args: Value) -> Result<Value, ToolError> {
        let tools = tools(&host());
        let tool = tools.iter().find(|t| t.desc().name == name).unwrap();
        tool.call(args, &mut Call::detached()).map(|r| r.structured.unwrap())
    }

    #[test]
    fn names_follow_the_types() {
        assert_eq!(class_id(DeviceClass::LogicAnalyzer), "logic-analyzer");
        assert_eq!(kind_id(ValueKind::RationalPeriod), "rational_period");
        assert_eq!(kind_id(ValueKind::U64), "u64");
    }

    #[test]
    fn drivers_are_listed_and_filtered() {
        let all = call("list_drivers", json!({})).unwrap();
        let n = all["count"].as_u64().unwrap();
        assert!(n > 50, "{n}");
        let demo = call("list_drivers", json!({"class": "demo-device"})).unwrap();
        assert_eq!(demo["drivers"][0]["id"], "demo");
        let q = call("list_drivers", json!({"query": "DEMO"})).unwrap();
        assert!(q["drivers"].as_array().unwrap().iter().any(|d| d["id"] == "demo"));
        let none = call("list_drivers", json!({"class": "toaster"})).unwrap();
        assert_eq!(none["count"], 0);
    }

    #[test]
    fn a_driver_is_described() {
        let d = call("driver_info", json!({"driver": "demo"})).unwrap();
        assert_eq!(d["id"], "demo");
        assert!(d["scan_options"].as_array().unwrap().iter().any(|k| k["key"] == "logic_channels"));
        assert_eq!(d["firmware"], json!([]));
        let e = call("driver_info", json!({"driver": "nope"})).unwrap_err();
        assert_eq!(e.message, "Unknown driver 'nope'.");
        assert!(!e.invalid_args);
    }

    #[test]
    fn decoders_are_listed_and_described() {
        let l = call("list_decoders", json!({"query": "i2c"})).unwrap();
        let ids: Vec<&str> =
            l["decoders"].as_array().unwrap().iter().map(|d| d["id"].as_str().unwrap()).collect();
        assert!(ids.contains(&"i2c"), "{ids:?}");
        let d = call("decoder_info", json!({"decoder": "i2c"})).unwrap();
        assert_eq!(d["channels"][0]["role"], "scl");
        assert!(
            d["annotation_rows"]
                .as_array()
                .unwrap()
                .iter()
                .any(|r| r["classes"].as_array().is_some_and(|c| !c.is_empty()))
        );
        assert_eq!(d["native"], true);
        let t = call("list_decoders", json!({"tag": "memory"})).unwrap();
        assert!(t["count"].as_u64().unwrap() > 0);
        assert!(call("decoder_info", json!({"decoder": "nope"})).is_err());
    }

    #[test]
    fn formats_version_and_firmware_answer() {
        let f = call("list_formats", json!({})).unwrap();
        assert!(f["output"].as_array().unwrap().iter().any(|m| m["id"] == "bits"));
        assert!(f["input"].as_array().unwrap().iter().any(|m| m["id"] == "vcd"));
        let v = call("version", json!({})).unwrap();
        assert_eq!(v["version"], crate::core::VERSION);
        assert_eq!(v["decoder_runtime"]["native"], true);
        let s = call("firmware_status", json!({})).unwrap();
        assert_ne!(s["search_dirs"].as_array().unwrap().len(), 0);
        assert!(s["drivers"].as_array().unwrap().iter().any(|d| d["driver"] == "fx2lafw"));
    }
}
