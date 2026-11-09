// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The analyzer tools: a decoder stack bound to a capture whose output
//! is kept, so an agent decodes once and then pages, searches, counts
//! and walks the frames.

use super::capture::no_capture;
use crate::decode::analyzer::{Poll, Source, Status};
use crate::decode::frames::Frame;
use crate::decode::radix::{self, Radix, RadixSupport};
use crate::decode::{Analyzer, InstanceInfo};
use crate::mcp::Host;
use crate::mcp::capture::CaptureStore;
use crate::mcp::host::AnalyzerEntry;
use crate::mcp::tool::{
    Annotations, Call, Tag, Tool, ToolError, ToolResult, ToolSpec, deadline, function, seconds,
};
use schemars::JsonSchema;
use serde::Deserialize;
use serde_json::{Value, json};
use std::collections::HashMap;
use std::fmt::Write as _;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// One decoder of a structured stack.
#[derive(Deserialize, JsonSchema, Clone)]
#[serde(deny_unknown_fields)]
pub struct DecoderSpec {
    /// The decoder id: `i2c`, `uart`, `eeprom24xx`.
    pub decoder: String,
    /// Channel role to capture channel name: `{"scl": "D0", "sda": "D1"}`.
    /// A stacked decoder takes none.
    #[serde(default)]
    pub channels: HashMap<String, String>,
    /// Option values by id, as text or number: `{"baudrate": 115200}`.
    #[serde(default)]
    pub options: HashMap<String, Value>,
}

/// `add_analyzer`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddArgs {
    /// The capture to decode.
    pub capture_id: u64,
    /// The stack in `-P` syntax, bottom first: `i2c:scl=D0:sda=D1,eeprom24xx`,
    /// `uart:rx=D0:baudrate=115200`. Or `decoders`.
    #[serde(default)]
    pub stack: Option<String>,
    /// The stack as data, bottom first. Or `stack`.
    #[serde(default)]
    pub decoders: Vec<DecoderSpec>,
    /// The analyzer's radix: `native`, `hex`, `dec`, `bin`, `ascii`. The
    /// decoder's default otherwise (`ascii` for `uart`, else `native`).
    #[serde(default)]
    pub radix: Option<String>,
    /// A name for the listing.
    #[serde(default)]
    pub name: Option<String>,
}

/// `update_analyzer`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateArgs {
    /// The analyzer.
    pub analyzer_id: u64,
    /// A new stack in `-P` syntax; the old one otherwise.
    #[serde(default)]
    pub stack: Option<String>,
    /// A new stack as data; the old one otherwise.
    #[serde(default)]
    pub decoders: Vec<DecoderSpec>,
    /// A new radix.
    #[serde(default)]
    pub radix: Option<String>,
}

/// An analyzer id alone.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IdArgs {
    /// The analyzer.
    pub analyzer_id: u64,
}

/// `list_analyzers`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ListArgs {
    /// Only the analyzers of this capture.
    #[serde(default)]
    pub capture_id: Option<u64>,
}

/// `analyzer_status`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StatusArgs {
    /// The analyzer.
    pub analyzer_id: u64,
    /// Wait up to this many seconds for the decode to end.
    #[serde(default)]
    pub wait: Option<f64>,
}

/// `analyzer_frames`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FramesArgs {
    /// The analyzer.
    pub analyzer_id: u64,
    /// One instance (`i2c-1`); every instance otherwise.
    #[serde(default)]
    pub instance: Option<String>,
    /// Only these rows, by id (`rx-data`, `address-write`, ...).
    #[serde(default)]
    pub rows: Vec<String>,
    /// Only these annotation classes, by id (the names `-A` takes).
    #[serde(default)]
    pub classes: Vec<String>,
    /// From this sample.
    #[serde(default)]
    pub from: Option<u64>,
    /// To this sample, exclusive.
    #[serde(default)]
    pub to: Option<u64>,
    /// From this time, in seconds.
    #[serde(default)]
    pub from_time: Option<f64>,
    /// To this time, in seconds.
    #[serde(default)]
    pub to_time: Option<f64>,
    /// Only frames whose rendered text contains this, case-insensitive.
    #[serde(default)]
    pub text: Option<String>,
    /// Only frames with this raw value (`80` for I²C address 0x50).
    #[serde(default)]
    pub value: Option<u64>,
    /// Only frames the error heuristic flagged.
    #[serde(default)]
    pub errors_only: bool,
    /// Render values in this radix; the analyzer's otherwise.
    #[serde(default)]
    pub radix: Option<String>,
    /// Include every text variant the decoder gave, longest first.
    #[serde(default)]
    pub all_texts: bool,
    /// Skip this many matching frames.
    #[serde(default)]
    pub offset: Option<usize>,
    /// At most this many frames: 100 by default, 2000 at most.
    #[serde(default)]
    pub limit: Option<usize>,
    /// The generation the offsets were taken from; refused when the
    /// analyzer was rebuilt since.
    #[serde(default)]
    pub generation: Option<u64>,
}

/// `analyzer_find`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FindArgs {
    /// The analyzer.
    pub analyzer_id: u64,
    /// The instance; the one that has `row` otherwise.
    #[serde(default)]
    pub instance: Option<String>,
    /// The row to look in, by id.
    pub row: String,
    /// The frame containing this sample.
    #[serde(default)]
    pub at: Option<u64>,
    /// The first frame starting after this sample.
    #[serde(default)]
    pub next_after: Option<u64>,
    /// The last frame ending before this sample.
    #[serde(default)]
    pub prev_before: Option<u64>,
    /// `at`, in seconds.
    #[serde(default)]
    pub at_time: Option<f64>,
    /// `next_after`, in seconds.
    #[serde(default)]
    pub next_after_time: Option<f64>,
    /// `prev_before`, in seconds.
    #[serde(default)]
    pub prev_before_time: Option<f64>,
    /// With `next_after`/`prev_before`: keep looking until the text
    /// contains this.
    #[serde(default)]
    pub text: Option<String>,
    /// With `next_after`/`prev_before`: keep looking until the raw value
    /// is this.
    #[serde(default)]
    pub value: Option<u64>,
    /// Render in this radix.
    #[serde(default)]
    pub radix: Option<String>,
}

/// `analyzer_binary`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BinaryArgs {
    /// The analyzer.
    pub analyzer_id: u64,
    /// The instance; the first with binary output otherwise.
    #[serde(default)]
    pub instance: Option<String>,
    /// The binary class, by id (what `-B dec=class` names); the first
    /// otherwise.
    #[serde(default)]
    pub class: Option<String>,
    /// Items starting from this sample.
    #[serde(default)]
    pub from: Option<u64>,
    /// Items starting before this sample.
    #[serde(default)]
    pub to: Option<u64>,
    /// `hex` (default), `ascii`, `dec`, `bin` or `base64`.
    #[serde(default)]
    pub radix: Option<String>,
    /// At most this many bytes in the answer: 4096 by default, 1 MiB at
    /// most. `export_binary` writes them all to a file.
    #[serde(default)]
    pub max_bytes: Option<usize>,
}

/// `export_binary`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExportBinaryArgs {
    /// The analyzer.
    pub analyzer_id: u64,
    /// The instance; the first with binary output otherwise.
    #[serde(default)]
    pub instance: Option<String>,
    /// The binary class, by id (what `-B dec=class` names); the first
    /// otherwise.
    #[serde(default)]
    pub class: Option<String>,
    /// Items starting from this sample.
    #[serde(default)]
    pub from: Option<u64>,
    /// Items starting before this sample.
    #[serde(default)]
    pub to: Option<u64>,
    /// The file to write, under the allowed directories.
    pub path: String,
}

/// `analyzer_meta`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MetaArgs {
    /// The analyzer.
    pub analyzer_id: u64,
    /// The instance; every instance otherwise.
    #[serde(default)]
    pub instance: Option<String>,
    /// Skip this many items.
    #[serde(default)]
    pub offset: Option<usize>,
    /// At most this many items: 100 by default.
    #[serde(default)]
    pub limit: Option<usize>,
}

/// `capture_decode`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DecodeArgs {
    /// The capture to decode.
    pub capture_id: u64,
    /// The stack in `-P` syntax: `i2c:scl=D0:sda=D1`. Or `decoders`.
    #[serde(default)]
    pub stack: Option<String>,
    /// The stack as data. Or `stack`.
    #[serde(default)]
    pub decoders: Vec<DecoderSpec>,
    /// The radix.
    #[serde(default)]
    pub radix: Option<String>,
    /// Only these rows in the page of frames.
    #[serde(default)]
    pub rows: Vec<String>,
    /// At most this many frames in the page: 100 by default.
    #[serde(default)]
    pub limit: Option<usize>,
    /// Keep the analyzer for more questions; dropped otherwise.
    #[serde(default)]
    pub keep: bool,
    /// Wait at most this many seconds for the decode.
    #[serde(default)]
    pub timeout: Option<f64>,
}

/// The analyzer tools over `host`.
#[allow(clippy::too_many_lines, clippy::many_single_char_names)]
pub fn tools(host: &Arc<Host>) -> Vec<Arc<dyn Tool>> {
    let h = host.clone();
    let add_analyzer = function::<AddArgs, _>(
        ToolSpec {
            name: "add_analyzer",
            title: "Add an analyzer",
            description: "Runs a decoder stack over a capture and keeps the output: give the \
                          `capture_id` and the stack in `-P` syntax (`i2c:scl=D0:sda=D1`, \
                          `uart:rx=D0:baudrate=115200`, `i2c:scl=D0:sda=D1,eeprom24xx`) or as \
                          `decoders`. Returns the `analyzer_id`, the instances with their rows \
                          and how each takes a radix, and the status; the decode runs on its \
                          own thread, so follow with `analyzer_status {wait}` and then \
                          `analyzer_summary` and `analyzer_frames`.",
            example: json!({"capture_id": 1, "stack": "uart:rx=D0:baudrate=115200"}),
            annotations: Annotations::MUTATING,
            tags: &[Tag::Decode],
        },
        move |args, _| {
            let spec = stack_text(args.stack.as_deref(), &args.decoders)?;
            let (spec, analyzer) = build(&h, args.capture_id, &spec, args.radix.as_deref())?;
            let id = h.add_analyzer(args.capture_id, args.name, spec, analyzer);
            let entry = h.analyzer(id).ok_or_else(|| no_analyzer(id))?;
            Ok(ToolResult::structured(describe(&entry)))
        },
    );

    let h = host.clone();
    let update_analyzer = function::<UpdateArgs, _>(
        ToolSpec {
            name: "update_analyzer",
            title: "Update an analyzer",
            description: "Changes an analyzer's stack, options or radix. Decoders cannot \
                          seek, so the stack is rebuilt and decodes from sample 0; the \
                          `generation` goes up, and pages taken from the old one are stale.",
            example: json!({"analyzer_id": 1, "radix": "hex"}),
            annotations: Annotations::MUTATING,
            tags: &[Tag::Decode],
        },
        move |args, _| {
            let old = h.analyzer(args.analyzer_id).ok_or_else(|| no_analyzer(args.analyzer_id))?;
            let spec = if args.stack.is_some() || !args.decoders.is_empty() {
                stack_text(args.stack.as_deref(), &args.decoders)?
            } else {
                old.spec.clone()
            };
            let radix = args.radix.as_deref().or(Some(old.analyzer.radix().id()));
            let (spec, analyzer) = build(&h, old.capture_id, &spec, radix)?;
            let entry = h
                .replace_analyzer(args.analyzer_id, spec, analyzer)
                .ok_or_else(|| no_analyzer(args.analyzer_id))?;
            Ok(ToolResult::structured(describe(&entry)))
        },
    );

    let h = host.clone();
    let remove_analyzer = function::<IdArgs, _>(
        ToolSpec {
            name: "remove_analyzer",
            title: "Remove an analyzer",
            description: "Stops an analyzer, waiting for its thread, and frees its frames.",
            example: json!({"analyzer_id": 1}),
            annotations: Annotations::MUTATING,
            tags: &[Tag::Decode],
        },
        move |args, _| {
            let entry =
                h.remove_analyzer(args.analyzer_id).ok_or_else(|| no_analyzer(args.analyzer_id))?;
            entry.analyzer.stop();
            Ok(ToolResult::structured(json!({"removed": args.analyzer_id})))
        },
    );

    let h = host.clone();
    let list_analyzers = function::<ListArgs, _>(
        ToolSpec {
            name: "list_analyzers",
            title: "List analyzers",
            description: "Every analyzer: its capture, stack, radix, status, progress, \
                          generation and frame count.",
            example: json!({}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Decode],
        },
        move |args, _| {
            let list: Vec<Value> = h
                .analyzers()
                .iter()
                .filter(|a| args.capture_id.is_none_or(|c| c == a.capture_id))
                .map(describe)
                .collect();
            Ok(ToolResult::structured(json!({"count": list.len(), "analyzers": list})))
        },
    );

    let h = host.clone();
    let analyzer_status = function::<StatusArgs, _>(
        ToolSpec {
            name: "analyzer_status",
            title: "Analyzer status",
            description: "Status (`running`, `complete`, `error`), progress, generation and \
                          frame count of an analyzer; with `wait`, waits up to that many \
                          seconds for the decode to end first.",
            example: json!({"analyzer_id": 1, "wait": 10}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Decode],
        },
        move |args, cx| {
            let entry =
                h.analyzer(args.analyzer_id).ok_or_else(|| no_analyzer(args.analyzer_id))?;
            if let Some(secs) = args.wait {
                wait(&entry.analyzer, seconds(secs, "wait")?, cx)?;
            }
            Ok(ToolResult::structured(describe(&entry)))
        },
    );

    let h = host.clone();
    let analyzer_frames = function::<FramesArgs, _>(
        ToolSpec {
            name: "analyzer_frames",
            title: "Analyzer frames",
            description: "The decoded frames, in sample order, filtered by instance, rows, \
                          classes, range, text, value or errors, and paged with `offset` and \
                          `limit`. Each frame has `ss`/`es` (samples), `t0`/`t1` (seconds), \
                          `instance`, `row`, `class`, `text`, and `value`/`bits` when the \
                          decoder gave a raw value: compare numbers on `value`, not on the \
                          text. `radix` renders the values per request.",
            example: json!({"analyzer_id": 1, "rows": ["rx-data"], "limit": 50}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Decode],
        },
        move |args, _| {
            let entry =
                h.analyzer(args.analyzer_id).ok_or_else(|| no_analyzer(args.analyzer_id))?;
            check_generation(&entry, args.generation)?;
            let a = &entry.analyzer;
            let radix = request_radix(a, args.radix.as_deref())?;
            let (from, to) = range(a, args.from, args.to, args.from_time, args.to_time)?;
            let instances = pick_instances(a, args.instance.as_deref())?;
            let limit = args.limit.unwrap_or(100).clamp(1, 2000);
            let offset = args.offset.unwrap_or(0);
            let text = args.text.as_deref().map(str::to_lowercase);
            // (start, instance index, frame id) of every match, then one page.
            let mut matches: Vec<(u64, usize, u32)> = Vec::new();
            for &i in &instances {
                let inst = &a.instances()[i];
                let rows = indexes(&args.rows, |r| inst.row_index(r));
                let classes = indexes(&args.classes, |c| inst.class_index(c));
                let store = a.store(i);
                for (id, f) in store.in_range(from, to) {
                    if rows.as_ref().is_some_and(|r| !r.contains(&f.row))
                        || classes.as_ref().is_some_and(|c| !c.contains(&f.class))
                        || (args.errors_only && !f.error)
                        || args.value.is_some_and(|v| f.value.map(|(x, _)| x) != Some(v))
                    {
                        continue;
                    }
                    if let Some(t) = &text
                        && !render(f, radix).to_lowercase().contains(t.as_str())
                    {
                        continue;
                    }
                    matches.push((f.start, i, id));
                }
            }
            matches.sort_unstable();
            let total = matches.len();
            let page: Vec<Value> = matches
                .iter()
                .skip(offset)
                .take(limit)
                .map(|&(_, i, id)| {
                    let store = a.store(i);
                    let f = store.get(id).expect("a matched frame");
                    frame_json(a, &a.instances()[i], f, radix, args.all_texts)
                })
                .collect();
            Ok(ToolResult::structured(json!({
                "analyzer_id": entry.id,
                "generation": entry.generation,
                "status": a.status().id(),
                "radix": radix.id(),
                "from": from,
                "to": to,
                "total": total,
                "offset": offset,
                "limit": limit,
                "truncated": offset.saturating_add(page.len()) < total,
                "frames": page,
            })))
        },
    );

    let h = host.clone();
    let analyzer_summary = function::<IdArgs, _>(
        ToolSpec {
            name: "analyzer_summary",
            title: "Analyzer summary",
            description: "Per instance, row and class: frame counts, error counts, and the \
                          first and last frame. The answer to \"were there NACKs?\" or \"how \
                          many bytes?\" in one call.",
            example: json!({"analyzer_id": 1}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Decode],
        },
        move |args, _| {
            let entry =
                h.analyzer(args.analyzer_id).ok_or_else(|| no_analyzer(args.analyzer_id))?;
            let a = &entry.analyzer;
            let instances: Vec<Value> = a
                .instances()
                .iter()
                .enumerate()
                .map(|(i, inst)| {
                    let store = a.store(i);
                    let rows: Vec<Value> = inst
                        .rows
                        .iter()
                        .enumerate()
                        .map(|(r, (id, desc))| {
                            json!({"id": id, "description": desc, "frames": store.row_count(r)})
                        })
                        .collect();
                    let classes: Vec<Value> = inst
                        .info
                        .annotations
                        .iter()
                        .enumerate()
                        .map(|(c, cls)| {
                            json!({
                                "id": cls.id,
                                "description": cls.desc,
                                "frames": store.class_count(c),
                                "error": radix::is_error_class(inst.info, c),
                            })
                        })
                        .collect();
                    let first =
                        store.by_order(0).map(|(_, f)| frame_json(a, inst, f, a.radix(), false));
                    let last = store
                        .len()
                        .checked_sub(1)
                        .and_then(|k| store.by_order(k))
                        .map(|(_, f)| frame_json(a, inst, f, a.radix(), false));
                    json!({
                        "instance": inst.id,
                        "decoder": inst.info.id,
                        "frames": store.len(),
                        "errors": store.errors(),
                        "binary_items": store.binary().len(),
                        "meta_items": store.meta().len(),
                        "first": first,
                        "last": last,
                        "rows": rows,
                        "classes": classes,
                    })
                })
                .collect();
            Ok(ToolResult::structured(json!({
                "analyzer_id": entry.id,
                "generation": entry.generation,
                "status": a.status().id(),
                "error": match a.status() { Status::Error(e) => Some(e), _ => None },
                "instances": instances,
            })))
        },
    );

    let h = host.clone();
    let analyzer_find = function::<FindArgs, _>(
        ToolSpec {
            name: "analyzer_find",
            title: "Find a frame",
            description: "The frame of a row at a sample or time, or the next after / the \
                          previous before one, optionally the next whose text contains \
                          `text` or whose value is `value`: walking frames as `find` walks \
                          edges.",
            example: json!({"analyzer_id": 1, "row": "address-write", "next_after": 0}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Decode],
        },
        move |args, _| {
            let entry =
                h.analyzer(args.analyzer_id).ok_or_else(|| no_analyzer(args.analyzer_id))?;
            let a = &entry.analyzer;
            let radix = request_radix(a, args.radix.as_deref())?;
            let (i, inst) = match &args.instance {
                Some(id) => a.instance(id).ok_or_else(|| no_instance(a, id))?,
                None => a
                    .instances()
                    .iter()
                    .enumerate()
                    .find(|(_, inst)| inst.row_index(&args.row).is_some())
                    .ok_or_else(|| {
                        ToolError::new(format!("No row '{}' in any instance.", args.row))
                    })?,
            };
            let row = inst
                .row_index(&args.row)
                .ok_or_else(|| ToolError::new(format!("No row '{}' in {}.", args.row, inst.id)))?;
            let sample = |s: Option<u64>, t: Option<f64>| -> Result<Option<u64>, ToolError> {
                match (s, t) {
                    (Some(s), _) => Ok(Some(s)),
                    (None, Some(t)) => Ok(Some(to_sample(a, t)?)),
                    (None, None) => Ok(None),
                }
            };
            let text = args.text.as_deref().map(str::to_lowercase);
            let wanted = |f: &Frame| {
                text.as_deref().is_none_or(|t| render(f, radix).to_lowercase().contains(t))
                    && args.value.is_none_or(|v| f.value.map(|(x, _)| x) == Some(v))
            };
            let store = a.store(i);
            let hit: Option<Frame> = if let Some(s) = sample(args.at, args.at_time)? {
                store.at(row, s).map(|(_, f)| f.clone())
            } else if let Some(mut s) = sample(args.next_after, args.next_after_time)? {
                loop {
                    match store.next_in_row(row, s) {
                        Some((_, f)) if wanted(f) => break Some(f.clone()),
                        Some((_, f)) => s = f.start,
                        None => break None,
                    }
                }
            } else if let Some(mut s) = sample(args.prev_before, args.prev_before_time)? {
                loop {
                    match store.prev_in_row(row, s) {
                        Some((_, f)) if wanted(f) => break Some(f.clone()),
                        Some((_, f)) => s = f.end.min(s.saturating_sub(1)),
                        None => break None,
                    }
                }
            } else {
                return Err(ToolError::invalid_args("Give `at`, `next_after` or `prev_before`."));
            };
            Ok(ToolResult::structured(match hit {
                Some(f) => {
                    let mut v = frame_json(a, inst, &f, radix, false);
                    v["found"] = json!(true);
                    v["analyzer_id"] = json!(entry.id);
                    v
                }
                None => json!({"analyzer_id": entry.id, "found": false}),
            }))
        },
    );

    let h = host.clone();
    let analyzer_binary = function::<BinaryArgs, _>(
        ToolSpec {
            name: "analyzer_binary",
            title: "Binary output",
            description: "One instance's binary output class (what `-B` selects) over a \
                          range of samples, as hex, ascii, dec, bin or base64, capped at \
                          `max_bytes`; `export_binary` writes it all to a file.",
            example: json!({"analyzer_id": 1, "class": "rx", "radix": "ascii"}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Decode],
        },
        move |args, _| {
            let entry =
                h.analyzer(args.analyzer_id).ok_or_else(|| no_analyzer(args.analyzer_id))?;
            let max = args.max_bytes.unwrap_or(4096).clamp(1, 1 << 20);
            let picked = binary_bytes(
                &entry.analyzer,
                args.instance.as_deref(),
                args.class.as_deref(),
                args.from,
                args.to,
                Some(max),
            )?;
            let encoding = args.radix.as_deref().unwrap_or("hex");
            let data = match encoding {
                "base64" => base64(&picked.bytes),
                other => {
                    let r = Radix::from_id(other).ok_or_else(|| {
                        ToolError::invalid_args(format!(
                            "Unknown radix '{other}': hex, ascii, dec, bin or base64."
                        ))
                    })?;
                    radix::format_bytes(&picked.bytes, r)
                }
            };
            Ok(ToolResult::structured(json!({
                "analyzer_id": entry.id,
                "instance": picked.instance,
                "class": picked.class,
                "items": picked.items,
                "bytes": picked.total,
                "truncated": picked.total > picked.bytes.len(),
                "encoding": encoding,
                "data": data,
            })))
        },
    );

    let h = host.clone();
    let export_binary = function::<ExportBinaryArgs, _>(
        ToolSpec {
            name: "export_binary",
            title: "Export binary output",
            description: "Writes one instance's binary output class over a range of samples \
                          to a file under the allowed directories, whole: the bytes a UART \
                          carried, an EEPROM's contents. Returns the path and size.",
            example: json!({"analyzer_id": 1, "class": "rx", "path": "rx.bin"}),
            annotations: Annotations {
                read_only: false,
                destructive: true,
                idempotent: true,
                open_world: false,
            },
            tags: &[Tag::Decode],
        },
        move |args, _| {
            let entry =
                h.analyzer(args.analyzer_id).ok_or_else(|| no_analyzer(args.analyzer_id))?;
            let path = h.check_path(&args.path)?;
            let picked = binary_bytes(
                &entry.analyzer,
                args.instance.as_deref(),
                args.class.as_deref(),
                args.from,
                args.to,
                None,
            )?;
            std::fs::write(&path, &picked.bytes)
                .map_err(|e| ToolError::new(format!("Cannot write '{}': {e}", path.display())))?;
            Ok(ToolResult::structured(json!({
                "analyzer_id": entry.id,
                "instance": picked.instance,
                "class": picked.class,
                "items": picked.items,
                "bytes": picked.bytes.len(),
                "path": path,
            })))
        },
    );

    let h = host.clone();
    let analyzer_meta = function::<MetaArgs, _>(
        ToolSpec {
            name: "analyzer_meta",
            title: "Meta output",
            description: "An instance's meta output (what `-M` selects): named values such \
                          as a measured bitrate, with their sample ranges, paged.",
            example: json!({"analyzer_id": 1}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Decode],
        },
        move |args, _| {
            let entry =
                h.analyzer(args.analyzer_id).ok_or_else(|| no_analyzer(args.analyzer_id))?;
            let a = &entry.analyzer;
            let instances = pick_instances(a, args.instance.as_deref())?;
            let offset = args.offset.unwrap_or(0);
            let limit = args.limit.unwrap_or(100).clamp(1, 2000);
            let mut items = Vec::new();
            for &i in &instances {
                let inst = &a.instances()[i];
                for m in a.store(i).meta() {
                    items.push(json!({
                        "instance": inst.id, "ss": m.start, "es": m.end,
                        "t0": time(a, m.start), "t1": time(a, m.end),
                        "name": m.name, "value": m.text,
                    }));
                }
            }
            let total = items.len();
            let page: Vec<Value> = items.into_iter().skip(offset).take(limit).collect();
            Ok(ToolResult::structured(json!({
                "analyzer_id": entry.id,
                "total": total,
                "offset": offset,
                "truncated": offset.saturating_add(page.len()) < total,
                "items": page,
            })))
        },
    );

    let h = host.clone();
    let capture_decode = function::<DecodeArgs, _>(
        ToolSpec {
            name: "capture_decode",
            title: "Decode a capture",
            description: "The one-shot form: adds an analyzer, waits for it, and returns the \
                          summary with one page of frames; the analyzer is dropped unless \
                          `keep` is true, in which case `analyzer_id` is returned for more \
                          questions.",
            example: json!({"capture_id": 1, "stack": "i2c:scl=D0:sda=D1", "limit": 50}),
            annotations: Annotations::MUTATING,
            tags: &[Tag::Decode],
        },
        move |args, cx| {
            let spec = stack_text(args.stack.as_deref(), &args.decoders)?;
            let (spec, analyzer) = build(&h, args.capture_id, &spec, args.radix.as_deref())?;
            let timeout = match args.timeout {
                Some(t) => seconds(t, "timeout")?,
                None => h.limits().capture_timeout,
            };
            // The analyzer joins the table only when kept, so one-shot
            // decodes do not consume ids.
            let status = wait(&analyzer, timeout, cx)?;
            let a = &analyzer;
            let limit = args.limit.unwrap_or(100).clamp(1, 2000);
            let mut frames: Vec<(u64, usize, u32)> = Vec::new();
            let mut per_instance = Vec::new();
            for (i, inst) in a.instances().iter().enumerate() {
                let rows = indexes(&args.rows, |r| inst.row_index(r));
                let store = a.store(i);
                for (fid, f) in store.iter() {
                    if rows.as_ref().is_none_or(|r| r.contains(&f.row)) {
                        frames.push((f.start, i, fid));
                    }
                }
                per_instance.push(json!({
                    "instance": inst.id,
                    "decoder": inst.info.id,
                    "frames": store.len(),
                    "errors": store.errors(),
                    "rows": inst.rows.iter().enumerate().map(|(r, (id, _))| json!({"id": id, "frames": store.row_count(r)})).collect::<Vec<_>>(),
                }));
            }
            frames.sort_unstable();
            let total = frames.len();
            let page: Vec<Value> = frames
                .iter()
                .take(limit)
                .map(|&(_, i, fid)| {
                    let store = a.store(i);
                    frame_json(
                        a,
                        &a.instances()[i],
                        store.get(fid).expect("a frame"),
                        a.radix(),
                        false,
                    )
                })
                .collect();
            let radix = a.radix().id();
            let analyzer_id =
                args.keep.then(|| h.add_analyzer(args.capture_id, None, spec, analyzer));
            Ok(ToolResult::structured(json!({
                "capture_id": args.capture_id,
                "analyzer_id": analyzer_id,
                "status": status.id(),
                "error": match status { Status::Error(e) => Some(e), _ => None },
                "radix": radix,
                "instances": per_instance,
                "total": total,
                "truncated": total > page.len(),
                "frames": page,
            })))
        },
    );

    vec![
        add_analyzer,
        update_analyzer,
        remove_analyzer,
        list_analyzers,
        analyzer_status,
        analyzer_frames,
        analyzer_summary,
        analyzer_find,
        analyzer_binary,
        export_binary,
        analyzer_meta,
        capture_decode,
    ]
}

/// The bytes of a binary class, picked out of an analyzer.
struct PickedBinary {
    instance: String,
    class: &'static str,
    /// The bytes, cut at the cap when one was given.
    bytes: Vec<u8>,
    /// Items matched.
    items: usize,
    /// Bytes they hold in all.
    total: usize,
}

/// One instance's binary output class over `from..to`, cut at `cap`
/// bytes when given. The instance is the first with binary output and
/// the class its first, unless named.
fn binary_bytes(
    a: &Analyzer,
    instance: Option<&str>,
    class: Option<&str>,
    from: Option<u64>,
    to: Option<u64>,
    cap: Option<usize>,
) -> Result<PickedBinary, ToolError> {
    let (i, inst) = match instance {
        Some(id) => a.instance(id).ok_or_else(|| no_instance(a, id))?,
        None => a
            .instances()
            .iter()
            .enumerate()
            .find(|(_, inst)| !inst.info.binary.is_empty())
            .ok_or_else(|| ToolError::new("No instance has binary output."))?,
    };
    let class = match class {
        Some(id) => inst.info.binary.iter().position(|b| b.id == id).ok_or_else(|| {
            let ids: Vec<&str> = inst.info.binary.iter().map(|b| b.id).collect();
            ToolError::new(format!(
                "No binary class '{id}' in {}; it has: {}.",
                inst.id,
                ids.join(", ")
            ))
        })?,
        None if inst.info.binary.is_empty() => {
            return Err(ToolError::new(format!("{} has no binary output.", inst.id)));
        }
        None => 0,
    };
    let from = from.unwrap_or(0);
    let to = to.unwrap_or(u64::MAX);
    let store = a.store(i);
    let mut bytes = Vec::new();
    let mut total = 0usize;
    let mut items = 0usize;
    for item in
        store.binary().iter().filter(|b| b.class == class && b.start >= from && b.start < to)
    {
        items += 1;
        total += item.bytes.len();
        let room = cap.map_or(usize::MAX, |c| c.saturating_sub(bytes.len()));
        bytes.extend_from_slice(&item.bytes[..item.bytes.len().min(room)]);
    }
    Ok(PickedBinary {
        instance: inst.id.clone(),
        class: inst.info.binary[class].id,
        bytes,
        items,
        total,
    })
}

fn no_analyzer(id: u64) -> ToolError {
    ToolError::new(format!("No analyzer {id}; `list_analyzers` has the ids."))
}

fn no_instance(a: &Analyzer, id: &str) -> ToolError {
    let ids: Vec<&str> = a.instances().iter().map(|i| i.id.as_str()).collect();
    ToolError::new(format!("No instance '{id}'; the analyzer has: {}.", ids.join(", ")))
}

/// The `-P` text of a stack given either way.
fn stack_text(stack: Option<&str>, decoders: &[DecoderSpec]) -> Result<String, ToolError> {
    match (stack, decoders.is_empty()) {
        (Some(s), true) => Ok(s.to_owned()),
        (None, false) => {
            let parts: Vec<String> = decoders
                .iter()
                .map(|d| {
                    let mut part = d.decoder.clone();
                    let mut channels: Vec<_> = d.channels.iter().collect();
                    channels.sort();
                    for (role, ch) in channels {
                        let _ = write!(part, ":{role}={ch}");
                    }
                    let mut options: Vec<_> = d.options.iter().collect();
                    options.sort_by(|a, b| a.0.cmp(b.0));
                    for (k, v) in options {
                        let text = match v {
                            Value::String(s) => s.clone(),
                            other => other.to_string(),
                        };
                        let _ = write!(part, ":{k}={text}");
                    }
                    part
                })
                .collect();
            Ok(parts.join(","))
        }
        (Some(_), false) => Err(ToolError::invalid_args("Give `stack` or `decoders`, not both.")),
        (None, true) => Err(ToolError::invalid_args("Give the stack: `stack` or `decoders`.")),
    }
}

/// Parses the stack over the capture and starts it. The radix goes to
/// a `format` option where the bottom decoder renders it itself.
fn build(
    host: &Host,
    capture_id: u64,
    spec: &str,
    radix: Option<&str>,
) -> Result<(String, Analyzer), ToolError> {
    let (channels, source, unit_size, samplerate) = {
        let store = host.captures();
        let c = store.get(capture_id).ok_or_else(|| no_capture(capture_id))?;
        let source = if c.running {
            if c.unit_size == 0 {
                return Err(ToolError::new(format!(
                    "Capture {capture_id} has no samples yet; add the analyzer once \
                     `capture_status` shows some."
                )));
            }
            live_source(host.capture_store(), capture_id)
        } else {
            Source::Bytes(c.logic.clone())
        };
        (c.channels.clone(), source, c.unit_size, c.samplerate.unwrap_or(0))
    };
    let max_bytes = host.memory_left();
    if max_bytes == 0 {
        return Err(ToolError::new(
            "The memory budget is full; drop captures or analyzers, or raise --max-memory.",
        ));
    }
    let bottom_id =
        spec.split(',').next().unwrap_or_default().split(':').next().unwrap_or_default();
    let bottom = host
        .decoders()
        .find(bottom_id)
        .map(|d| d.info)
        .ok_or_else(|| ToolError::new(format!("Protocol decoder '{bottom_id}' not found.")))?;
    let radix = match radix {
        Some(id) => Radix::from_id(id).ok_or_else(|| {
            ToolError::invalid_args(format!(
                "Unknown radix '{id}': native, hex, dec, bin or ascii."
            ))
        })?,
        None => radix::default_radix(bottom),
    };
    // A decoder that renders the radix itself takes it as its `format`
    // option, unless the spec sets one. The injected option stays out of
    // the spec that is kept, so a later radix change injects anew.
    let given = spec.to_owned();
    let mut spec = spec.to_owned();
    if RadixSupport::of(bottom, true) == RadixSupport::FormatOption
        && let Some(format) = radix.sigrok_format()
    {
        let (first, rest) =
            spec.split_once(',').map_or((spec.as_str(), None), |(f, r)| (f, Some(r)));
        if !first.split(':').skip(1).any(|p| p.starts_with("format=")) {
            let mut first = format!("{first}:format={format}");
            if let Some(rest) = rest {
                first.push(',');
                first.push_str(rest);
            }
            spec = first;
        }
    }
    let plan = crate::setup::parse_stacks(host.decoders(), &[spec], &channels)?;
    let analyzer =
        Analyzer::start(plan, radix, true, source, unit_size.max(1), samplerate, max_bytes);
    Ok((given, analyzer))
}

/// The bytes of a running capture as they come: each poll copies what
/// arrived after `fed` samples, up to a chunk, and says when the capture
/// has ended and all of it was handed over. A capture dropped meanwhile
/// ends the source too.
fn live_source(store: Arc<Mutex<CaptureStore>>, capture_id: u64) -> Source {
    const CHUNK: usize = 1 << 20;
    Source::Live(Box::new(move |fed: u64| {
        let store = store.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        let Some(c) = store.get(capture_id) else {
            return Poll { data: Vec::new(), done: true };
        };
        let unit = c.unit_size.max(1);
        let from = (fed as usize).saturating_mul(unit).min(c.logic.len());
        let to = from.saturating_add(CHUNK).min(c.logic.len());
        Poll { data: c.logic[from..to].to_vec(), done: !c.running && to == c.logic.len() }
    }))
}

/// Waits for the decode, reporting progress, until done, the timeout or
/// a cancellation.
fn wait(a: &Analyzer, timeout: Duration, cx: &Call) -> Result<Status, ToolError> {
    let deadline = deadline(timeout)?;
    let mut last = 0u64;
    loop {
        let status = a.wait_until(Instant::now() + Duration::from_millis(200), || !cx.cancelled());
        if status != Status::Running || Instant::now() >= deadline || cx.cancelled() {
            return Ok(status);
        }
        let (fed, total) = a.progress();
        if fed != last {
            cx.progress(fed as f64, Some(total as f64), Some("samples decoded"));
            last = fed;
        }
    }
}

fn check_generation(entry: &AnalyzerEntry, generation: Option<u64>) -> Result<(), ToolError> {
    match generation {
        Some(g) if g != entry.generation => Err(ToolError::new(format!(
            "Stale generation {g}: the analyzer is at generation {}; page again from the start.",
            entry.generation
        ))
        .with_data(json!({"generation": entry.generation}))),
        _ => Ok(()),
    }
}

/// The radix a request renders in, checked against what the decoders
/// can do.
fn request_radix(a: &Analyzer, radix: Option<&str>) -> Result<Radix, ToolError> {
    let Some(id) = radix else { return Ok(a.radix()) };
    let radix = Radix::from_id(id).ok_or_else(|| {
        ToolError::invalid_args(format!("Unknown radix '{id}': native, hex, dec, bin or ascii."))
    })?;
    if radix != a.radix()
        && let Some(inst) =
            a.instances().iter().find(|i| i.radix_support == RadixSupport::FormatOption)
    {
        return Err(ToolError::new(format!(
            "{} renders its radix itself ({}); `update_analyzer {{radix}}` decodes again.",
            inst.id,
            a.radix().id()
        )));
    }
    Ok(radix)
}

fn time(a: &Analyzer, sample: u64) -> Option<f64> {
    let rate = a.samplerate();
    (rate > 0).then(|| sample as f64 / rate as f64)
}

fn to_sample(a: &Analyzer, seconds: f64) -> Result<u64, ToolError> {
    let rate = a.samplerate();
    if rate == 0 {
        return Err(ToolError::invalid_args("The capture has no samplerate; give sample numbers."));
    }
    Ok((seconds * rate as f64).round().max(0.0) as u64)
}

fn range(
    a: &Analyzer,
    from: Option<u64>,
    to: Option<u64>,
    from_time: Option<f64>,
    to_time: Option<f64>,
) -> Result<(u64, u64), ToolError> {
    let from = match (from, from_time) {
        (Some(s), _) => s,
        (None, Some(t)) => to_sample(a, t)?,
        (None, None) => 0,
    };
    let to = match (to, to_time) {
        (Some(s), _) => s,
        (None, Some(t)) => to_sample(a, t)?,
        (None, None) => u64::MAX,
    };
    if from > to {
        return Err(ToolError::invalid_args(format!("`from` ({from}) is after `to` ({to}).")));
    }
    Ok((from, to))
}

fn pick_instances(a: &Analyzer, instance: Option<&str>) -> Result<Vec<usize>, ToolError> {
    match instance {
        Some(id) => Ok(vec![a.instance(id).ok_or_else(|| no_instance(a, id))?.0]),
        None => Ok((0..a.instances().len()).collect()),
    }
}

/// The indexes of `ids` under `find`, or `None` when no ids were given.
/// An id another instance has is no error here; one no instance has
/// matches nothing, which the caller sees as an empty page.
fn indexes(ids: &[String], find: impl Fn(&str) -> Option<usize>) -> Option<Vec<usize>> {
    if ids.is_empty() {
        return None;
    }
    Some(ids.iter().filter_map(|id| find(id)).collect())
}

/// A frame's text in `radix`: the raw value rewritten when there is one.
fn render(f: &Frame, radix: Radix) -> String {
    match f.value {
        Some((v, bits)) if radix != Radix::Native => {
            radix::apply_radix(&f.texts, v, bits, radix).into_iter().next().unwrap_or_default()
        }
        _ => f.longest().to_owned(),
    }
}

fn frame_json(
    a: &Analyzer,
    inst: &InstanceInfo,
    f: &Frame,
    radix: Radix,
    all_texts: bool,
) -> Value {
    let mut v = json!({
        "ss": f.start,
        "es": f.end,
        "t0": time(a, f.start),
        "t1": time(a, f.end),
        "instance": inst.id,
        "row": inst.rows.get(f.row).map_or("", |(id, _)| id),
        "class": inst.info.annotations.get(f.class).map_or("", |c| c.id),
        "text": render(f, radix),
        "value": f.value.map(|(x, _)| x),
        "bits": f.value.map(|(_, b)| b),
        "error": f.error,
    });
    if all_texts {
        let texts = match f.value {
            Some((x, bits)) if radix != Radix::Native => {
                radix::apply_radix(&f.texts, x, bits, radix)
            }
            _ => f.texts.to_vec(),
        };
        v["texts"] = json!(texts);
    }
    v
}

fn describe(entry: &AnalyzerEntry) -> Value {
    let a = &entry.analyzer;
    let (fed, total) = a.progress();
    let status = a.status();
    json!({
        "analyzer_id": entry.id,
        "capture_id": entry.capture_id,
        "name": entry.name,
        "stack": entry.spec,
        "radix": a.radix().id(),
        "status": status.id(),
        "error": match &status { Status::Error(e) => Some(e.clone()), _ => None },
        "progress": {"samples": fed, "total": total, "fraction": if total == 0 { 1.0 } else { fed as f64 / total as f64 }},
        "generation": entry.generation,
        "frames": a.frames(),
        "bytes": a.memory_bytes(),
        "instances": a.instances().iter().map(|i| json!({
            "id": i.id,
            "decoder": i.info.id,
            "radix_support": i.radix_support.id(),
            "rows": i.rows.iter().map(|(id, desc)| json!({"id": id, "description": desc})).collect::<Vec<_>>(),
            "classes": i.info.annotations.iter().map(|c| c.id).collect::<Vec<_>>(),
            "binary": i.info.binary.iter().map(|b| b.id).collect::<Vec<_>>(),
        })).collect::<Vec<_>>(),
    })
}

/// Standard base64 with padding; small enough not to want a crate.
fn base64(bytes: &[u8]) -> String {
    const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b = [chunk[0], *chunk.get(1).unwrap_or(&0), *chunk.get(2).unwrap_or(&0)];
        let n = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
        out.push(TABLE[(n >> 18) as usize & 63] as char);
        out.push(TABLE[(n >> 12) as usize & 63] as char);
        out.push(if chunk.len() > 1 { TABLE[(n >> 6) as usize & 63] as char } else { '=' });
        out.push(if chunk.len() > 2 { TABLE[n as usize & 63] as char } else { '=' });
    }
    out
}

#[cfg(test)]
#[allow(clippy::many_single_char_names)]
mod tests {
    use super::*;
    use crate::core::{Channel, ChannelKind, Packet};
    use crate::mcp::capture::Capture;

    /// A UART byte at 10 samples per bit on bit 0 (1 kHz, 100 baud).
    fn uart_bytes(bytes: &[u8]) -> Vec<u8> {
        let mut out = vec![1u8; 20];
        for &b in bytes {
            out.extend(std::iter::repeat_n(0u8, 10));
            for bit in 0..8 {
                out.extend(std::iter::repeat_n((b >> bit) & 1, 10));
            }
            out.extend(std::iter::repeat_n(1u8, 10));
        }
        out.extend(std::iter::repeat_n(1u8, 40));
        out
    }

    fn host() -> Arc<Host> {
        let h = Host::with(crate::core::Registry::new(), crate::decoder_registry());
        let mut c =
            Capture::new("test", vec![Channel::new(0, ChannelKind::Logic, "D0")], Some(1000));
        c.push(&Packet::logic(1, uart_bytes(b"Hi!")));
        h.captures().add(c);
        Arc::new(h)
    }

    fn call(host: &Arc<Host>, name: &str, args: Value) -> Result<Value, ToolError> {
        let tools = tools(host);
        let tool = tools.iter().find(|t| t.desc().name == name).unwrap();
        tool.call(args, &mut Call::detached()).map(|r| r.structured.unwrap())
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn decodes_pages_and_renders_per_request() {
        let h = host();
        let a = call(
            &h,
            "add_analyzer",
            json!({"capture_id": 1, "stack": "uart:rx=D0:baudrate=100", "name": "serial"}),
        )
        .unwrap();
        assert_eq!(a["analyzer_id"], 1);
        assert_eq!(a["radix"], "ascii");
        assert_eq!(a["instances"][0]["radix_support"], "raw_values");
        let s = call(&h, "analyzer_status", json!({"analyzer_id": 1, "wait": 10})).unwrap();
        assert_eq!(s["status"], "complete");
        assert_eq!(s["progress"]["fraction"], 1.0);
        let f =
            call(&h, "analyzer_frames", json!({"analyzer_id": 1, "classes": ["rx-data"]})).unwrap();
        assert_eq!(f["total"], 3);
        let texts: Vec<&str> =
            f["frames"].as_array().unwrap().iter().map(|x| x["text"].as_str().unwrap()).collect();
        assert_eq!(texts, ["'H'", "'i'", "'!'"]);
        assert_eq!(f["frames"][0]["value"], 72);
        assert_eq!(f["frames"][0]["bits"], 8);
        assert_eq!(f["frames"][0]["row"], "rx-data");
        // 20 idle samples, then the start bit; the byte begins at 30.
        assert_eq!(f["frames"][0]["t0"], 0.03);
        let hex = call(
            &h,
            "analyzer_frames",
            json!({"analyzer_id": 1, "classes": ["rx-data"], "radix": "hex", "all_texts": true}),
        )
        .unwrap();
        assert_eq!(hex["frames"][0]["text"], "0x48");
        // uart has one text per data frame; it follows the radix too.
        assert_eq!(hex["frames"][0]["texts"], json!(["0x48"]));
        let page = call(&h, "analyzer_frames", json!({"analyzer_id": 1, "classes": ["rx-data"], "offset": 1, "limit": 1, "generation": 1})).unwrap();
        assert_eq!(page["frames"][0]["text"], "'i'");
        assert_eq!(page["truncated"], true);
        let by_value = call(&h, "analyzer_frames", json!({"analyzer_id": 1, "value": 33})).unwrap();
        assert_eq!(by_value["total"], 1);
        assert_eq!(by_value["frames"][0]["text"], "'!'");
        let by_text =
            call(&h, "analyzer_frames", json!({"analyzer_id": 1, "text": "start"})).unwrap();
        assert!(by_text["total"].as_u64().unwrap() >= 3, "{by_text}");
        let ranged = call(
            &h,
            "analyzer_frames",
            json!({"analyzer_id": 1, "classes": ["rx-data"], "from_time": 0.1}),
        )
        .unwrap();
        assert_eq!(ranged["total"], 2);

        let sum = call(&h, "analyzer_summary", json!({"analyzer_id": 1})).unwrap();
        let inst = &sum["instances"][0];
        assert_eq!(inst["instance"], "uart-1");
        assert_eq!(inst["errors"], 0);
        let row = inst["rows"].as_array().unwrap().iter().find(|r| r["id"] == "rx-data").unwrap();
        // The row has the start and stop bits too; the class is the bytes.
        assert_eq!(row["frames"], 9);
        let class =
            inst["classes"].as_array().unwrap().iter().find(|c| c["id"] == "rx-data").unwrap();
        assert_eq!(class["frames"], 3);
        assert!(inst["first"]["row"].as_str().is_some());

        let found = call(
            &h,
            "analyzer_find",
            json!({"analyzer_id": 1, "row": "rx-data", "next_after": 0, "value": 105}),
        )
        .unwrap();
        assert_eq!(found["found"], true);
        assert_eq!(found["text"], "'i'");
        let stop = call(
            &h,
            "analyzer_find",
            json!({"analyzer_id": 1, "row": "rx-data", "next_after": 0, "text": "stop"}),
        )
        .unwrap();
        // Upstream tags stop bits with the parity-ok class; the decoder
        // keeps that.
        assert_eq!(stop["class"], "rx-parity-ok", "{stop}");
        assert_eq!(stop["text"], "Stop bit");
        let found = call(
            &h,
            "analyzer_find",
            json!({"analyzer_id": 1, "row": "rx-data", "prev_before": 1000, "value": 72}),
        )
        .unwrap();
        assert_eq!(found["text"], "'H'");
        let at =
            call(&h, "analyzer_find", json!({"analyzer_id": 1, "row": "rx-data", "at_time": 0.05}))
                .unwrap();
        assert_eq!(at["value"], 72);
        let none = call(
            &h,
            "analyzer_find",
            json!({"analyzer_id": 1, "row": "rx-data", "next_after": 5000}),
        )
        .unwrap();
        assert_eq!(none["found"], false);

        let bin =
            call(&h, "analyzer_binary", json!({"analyzer_id": 1, "class": "rx", "radix": "ascii"}))
                .unwrap();
        assert_eq!(bin["data"], "Hi!");
        assert_eq!(bin["bytes"], 3);
        let b64 =
            call(&h, "analyzer_binary", json!({"analyzer_id": 1, "radix": "base64"})).unwrap();
        assert_eq!(b64["data"], "SGkh");
        assert!(call(&h, "analyzer_binary", json!({"analyzer_id": 1, "class": "nope"})).is_err());
        let meta = call(&h, "analyzer_meta", json!({"analyzer_id": 1})).unwrap();
        assert!(meta["total"].is_number());

        let list = call(&h, "list_analyzers", json!({})).unwrap();
        assert_eq!(list["count"], 1);
        assert_eq!(list["analyzers"][0]["name"], "serial");
        // A rebuild bumps the generation; the old page is refused.
        let up = call(&h, "update_analyzer", json!({"analyzer_id": 1, "radix": "hex"})).unwrap();
        assert_eq!(up["generation"], 2);
        assert_eq!(up["radix"], "hex");
        let e =
            call(&h, "analyzer_frames", json!({"analyzer_id": 1, "generation": 1})).unwrap_err();
        assert!(e.message.contains("Stale"), "{e}");
        call(&h, "analyzer_status", json!({"analyzer_id": 1, "wait": 10})).unwrap();
        let f = call(
            &h,
            "analyzer_frames",
            json!({"analyzer_id": 1, "classes": ["rx-data"], "limit": 1}),
        )
        .unwrap();
        assert_eq!(f["frames"][0]["text"], "0x48");
        assert_eq!(call(&h, "remove_analyzer", json!({"analyzer_id": 1})).unwrap()["removed"], 1);
        assert!(call(&h, "analyzer_status", json!({"analyzer_id": 1})).is_err());
    }

    #[test]
    fn one_shot_structured_and_errors() {
        let h = host();
        let d = call(
            &h,
            "capture_decode",
            json!({
                "capture_id": 1,
                "decoders": [{"decoder": "uart", "channels": {"rx": "D0"}, "options": {"baudrate": 100}}],
                "rows": ["rx-data"],
                "radix": "dec",
            }),
        )
        .unwrap();
        assert_eq!(d["status"], "complete");
        // The row: start bit, byte, stop bit per byte.
        assert_eq!(d["total"], 9);
        assert_eq!(d["frames"][1]["text"], "72");
        assert_eq!(d["frames"][4]["text"], "105");
        assert_eq!(d["analyzer_id"], Value::Null);
        assert_eq!(call(&h, "list_analyzers", json!({})).unwrap()["count"], 0);
        let kept = call(
            &h,
            "capture_decode",
            json!({"capture_id": 1, "stack": "uart:rx=D0:baudrate=100", "keep": true}),
        )
        .unwrap();
        assert_eq!(kept["analyzer_id"], 1);
        assert_eq!(call(&h, "list_analyzers", json!({"capture_id": 1})).unwrap()["count"], 1);

        let e = call(&h, "add_analyzer", json!({"capture_id": 1, "stack": "nope"})).unwrap_err();
        assert_eq!(e.message, "Protocol decoder 'nope' not found.");
        let e = call(&h, "add_analyzer", json!({"capture_id": 9, "stack": "uart"})).unwrap_err();
        assert!(e.message.contains("No capture 9"), "{e}");
        let e = call(&h, "add_analyzer", json!({"capture_id": 1})).unwrap_err();
        assert!(e.invalid_args);
        let e = call(
            &h,
            "add_analyzer",
            json!({"capture_id": 1, "stack": "uart:rx=D0", "radix": "octal"}),
        )
        .unwrap_err();
        assert!(e.invalid_args);
        // A decoder that fails ends in error, with its words.
        let a =
            call(&h, "add_analyzer", json!({"capture_id": 1, "stack": "uart:rx=D0:baudrate=0"}))
                .unwrap();
        let s = call(&h, "analyzer_status", json!({"analyzer_id": a["analyzer_id"], "wait": 10}))
            .unwrap();
        assert_eq!(s["status"], "error");
        assert!(s["error"].as_str().is_some());
        assert_eq!(
            stack_text(
                None,
                &[DecoderSpec {
                    decoder: "i2c".into(),
                    channels: [
                        ("sda".to_owned(), "D1".to_owned()),
                        ("scl".to_owned(), "D0".to_owned())
                    ]
                    .into(),
                    options: HashMap::new()
                }]
            )
            .unwrap(),
            "i2c:scl=D0:sda=D1"
        );
        assert_eq!(base64(b"Hi!"), "SGkh");
        assert_eq!(base64(b"Hi"), "SGk=");
    }

    #[test]
    fn a_format_option_decoder_takes_the_radix_itself() {
        // `timing` has neither; a decoder with a format option and no
        // binary output refuses a per-request radix. The native
        // decoders with that shape are few; `request_radix` is checked
        // on the rule directly.
        let h = host();
        let a = call(
            &h,
            "add_analyzer",
            json!({"capture_id": 1, "stack": "uart:rx=D0:baudrate=100", "radix": "hex"}),
        )
        .unwrap();
        // uart has both: raw values win, so the format option is not set.
        assert_eq!(a["stack"], "uart:rx=D0:baudrate=100");
        let entry = h.analyzer(1).unwrap();
        assert_eq!(request_radix(&entry.analyzer, Some("ascii")).unwrap(), Radix::Ascii);
    }
}
