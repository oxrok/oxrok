// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! `capture_view`: a window of a capture as a picture, for an agent that
//! sees, or as edges and text lanes, for one that reads.

use super::capture::no_capture;
use super::data::Range;
use crate::core::ChannelKind;
use crate::decode::radix::{self, Radix};
use crate::mcp::Host;
use crate::mcp::tool::{
    Annotations, Content, Tag, Tool, ToolError, ToolResult, ToolSpec, function,
};
use crate::mcp::view::{self, FrameLane, Picture};
use schemars::JsonSchema;
use serde::Deserialize;
use serde_json::{Value, json};
use std::sync::Arc;

/// `capture_view`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ViewArgs {
    /// The id `capture` or `load_capture` returned.
    pub capture_id: u64,
    /// The channels to draw, top to bottom; every enabled one when left
    /// out.
    #[serde(default)]
    pub channels: Vec<String>,
    /// The window: `from`/`to` in samples or `from_time`/`to_time` in
    /// seconds; the whole capture when left out.
    #[serde(flatten)]
    pub range: Range,
    /// `image` (default): a PNG of the window, logic lanes, analog traces
    /// with their scale, a time axis, the trigger and the frames of
    /// `analyzer_id`; `text`: the edges in the window merged in sample
    /// order, with the lanes drawn in characters.
    #[serde(default)]
    pub form: Option<String>,
    /// The picture's width in pixels (1024 by default, 200 to 4096), or
    /// the text lanes' columns (100 by default, 400 at most).
    #[serde(default)]
    pub width: Option<usize>,
    /// Draw this analyzer's frames under the signals, one lane per row.
    #[serde(default)]
    pub analyzer_id: Option<u64>,
    /// With `analyzer_id`: only these rows (`addr-data`); the rows that
    /// are not bit rows otherwise.
    #[serde(default)]
    pub rows: Vec<String>,
    /// With `form: text`: at most this many edges (200 by default, 2000
    /// at most).
    #[serde(default)]
    pub max_events: Option<usize>,
}

/// The view tool over `host`.
#[allow(clippy::too_many_lines)]
pub fn tools(host: &Arc<Host>) -> Vec<Arc<dyn Tool>> {
    let h = host.clone();
    let capture_view = function::<ViewArgs, _>(
        ToolSpec {
            name: "capture_view",
            title: "View a window",
            description: "A window of a capture to look at: by default a PNG with the logic \
                          channels as lanes, the analog channels as traces with their scale, \
                          a time axis, the trigger, and, with `analyzer_id`, the decoded \
                          frames as labelled boxes under the signals, so the timing between \
                          channels and against the decode is visible at a glance. `form: \
                          text` gives the window's edges merged in sample order instead, \
                          which is exact where a picture is approximate. Narrow the window \
                          (`from`/`to` or `from_time`/`to_time`) around a `find` result or a \
                          frame; a whole long capture in one picture shows only density.",
            example: json!({"capture_id": 1, "from_time": 0.0, "to_time": 0.0002, "analyzer_id": 1}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Capture],
        },
        move |args, _| {
            let analyzer = args.analyzer_id.map(|id| h.analyzer(id)).map(|entry| {
                entry.ok_or_else(|| {
                    ToolError::new("No such analyzer; `list_analyzers` has the ids.")
                })
            });
            let analyzer = analyzer.transpose()?;
            let store = h.captures();
            let c = store.get(args.capture_id).ok_or_else(|| no_capture(args.capture_id))?;
            let (from, to) =
                args.range.resolve(c, c.logic_samples().max(c.analog_samples_max()))?;
            let channels: Vec<&crate::core::Channel> = if args.channels.is_empty() {
                c.channels.iter().filter(|ch| ch.enabled).collect()
            } else {
                args.channels
                    .iter()
                    .map(|n| super::data::channel(c, n))
                    .collect::<Result<_, _>>()?
            };
            if channels.is_empty() {
                return Err(ToolError::new("The capture has no enabled channels to draw."));
            }
            let form = args.form.as_deref().unwrap_or("image");
            match form {
                "image" => {
                    let width = args.width.unwrap_or(1024).clamp(200, 4096);
                    // The frames, owned here so the lanes can borrow them.
                    let mut owned: Vec<(String, Vec<(crate::decode::frames::Frame, String)>)> =
                        Vec::new();
                    if let Some(entry) = &analyzer {
                        let a = &entry.analyzer;
                        for (i, inst) in a.instances().iter().enumerate() {
                            let wanted: Vec<usize> = if args.rows.is_empty() {
                                inst.rows
                                    .iter()
                                    .enumerate()
                                    .filter(|(_, (id, _))| !id.contains("bit"))
                                    .map(|(r, _)| r)
                                    .collect()
                            } else {
                                args.rows.iter().filter_map(|r| inst.row_index(r)).collect()
                            };
                            let st = a.store(i);
                            for r in wanted {
                                let frames: Vec<(crate::decode::frames::Frame, String)> = st
                                    .in_row(r, from, to)
                                    .map(|(_, f)| {
                                        let text = match f.value {
                                            Some((v, bits)) if a.radix() != Radix::Native => {
                                                radix::apply_radix(&f.texts, v, bits, a.radix())
                                                    .into_iter()
                                                    .next()
                                                    .unwrap_or_default()
                                            }
                                            _ => f.longest().to_owned(),
                                        };
                                        (f.clone(), text)
                                    })
                                    .collect();
                                if !frames.is_empty() {
                                    owned.push((format!("{} {}", inst.id, inst.rows[r].0), frames));
                                }
                            }
                        }
                    }
                    let lanes: Vec<FrameLane<'_>> = owned
                        .iter()
                        .map(|(label, frames)| FrameLane {
                            label: label.clone(),
                            frames: frames.iter().map(|(f, t)| (f, t.clone())).collect(),
                        })
                        .collect();
                    let pic = Picture {
                        capture: c,
                        channels: channels.clone(),
                        frames: lanes,
                        from,
                        to,
                        width,
                    };
                    let png = pic.png();
                    let structured = json!({
                        "capture_id": c.id,
                        "from": from,
                        "to": to,
                        "from_time": c.time(from),
                        "to_time": c.time(to),
                        "samples_per_pixel": (to - from) as f64 / (width.saturating_sub(120)).max(1) as f64,
                        "channels": channels.iter().map(|ch| ch.name.as_str()).collect::<Vec<_>>(),
                        "frame_lanes": owned.iter().map(|(l, f)| json!({"lane": l, "frames": f.len()})).collect::<Vec<_>>(),
                        "width": width,
                        "png_bytes": png.len(),
                    });
                    Ok(ToolResult {
                        structured: Some(structured),
                        content: vec![Content::Image {
                            data: view::base64(&png),
                            mime_type: "image/png".to_owned(),
                        }],
                    })
                }
                "text" => {
                    let columns = args.width.unwrap_or(100).clamp(10, 400);
                    let max = args.max_events.unwrap_or(200).clamp(1, 2000);
                    let lanes: Vec<(String, String)> = channels
                        .iter()
                        .map(|ch| (ch.name.clone(), view::text_lane(c, ch, from, to, columns)))
                        .collect();
                    let (events, more) = view::events(c, &channels, from, to, max);
                    let events: Vec<Value> = events
                        .iter()
                        .map(|(s, ch, l)| json!({"sample": s, "t": c.time(*s), "channel": ch, "level": l}))
                        .collect();
                    let analog: Vec<&str> = channels
                        .iter()
                        .filter(|ch| ch.kind == ChannelKind::Analog)
                        .map(|ch| ch.name.as_str())
                        .collect();
                    let text = view::text_block(&lanes, from, to, columns);
                    Ok(ToolResult::structured(json!({
                        "capture_id": c.id,
                        "from": from,
                        "to": to,
                        "from_time": c.time(from),
                        "to_time": c.time(to),
                        "samples_per_column": (to - from) as f64 / columns as f64,
                        "lanes": lanes.iter().map(|(n, l)| json!({"channel": n, "lane": l})).collect::<Vec<_>>(),
                        "events": events,
                        "events_truncated": more,
                        "analog_lanes_note": if analog.is_empty() { Value::Null } else { json!("analog lanes are the mean per column, low to high as ▁ to █") },
                    }))
                    .with_text(text))
                }
                other => Err(ToolError::invalid_args(format!(
                    "Unknown form '{other}': `image` or `text`."
                ))),
            }
        },
    );
    vec![capture_view]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Channel, Packet};
    use crate::mcp::capture::Capture;
    use crate::mcp::tool::Call;

    fn host() -> Arc<Host> {
        let h = Host::with(crate::core::Registry::new(), crate::decoder_registry());
        let mut c = Capture::new(
            "test",
            vec![
                Channel::new(0, ChannelKind::Logic, "scl"),
                Channel::new(1, ChannelKind::Logic, "sda"),
            ],
            Some(1000),
        );
        // scl toggles every two samples; sda every four.
        let data: Vec<u8> = (0..32u8).map(|i| ((i / 2) & 1) | (((i / 4) & 1) << 1)).collect();
        c.push(&Packet::logic(1, data));
        h.captures().add(c);
        Arc::new(h)
    }

    fn call(host: &Arc<Host>, args: Value) -> Result<ToolResult, ToolError> {
        let tools = tools(host);
        tools[0].call(args, &mut Call::detached())
    }

    #[test]
    fn text_and_image_forms() {
        let h = host();
        let r = call(&h, json!({"capture_id": 1, "form": "text", "width": 32, "to": 32})).unwrap();
        let v = r.structured.unwrap();
        assert_eq!(v["lanes"][0]["lane"], "__‾‾__‾‾__‾‾__‾‾__‾‾__‾‾__‾‾__‾‾");
        assert_eq!(v["lanes"][1]["lane"], "____‾‾‾‾____‾‾‾‾____‾‾‾‾____‾‾‾‾");
        assert_eq!(
            v["events"][0],
            json!({"sample": 2, "t": 0.002, "channel": "scl", "level": true})
        );
        // At sample 4 both change; scl is listed first, then sda.
        assert_eq!(
            v["events"][1],
            json!({"sample": 4, "t": 0.004, "channel": "scl", "level": false})
        );
        assert_eq!(v["events"][2]["channel"], "sda");
        assert_eq!(v["events_truncated"], false);
        let text = match &r.content[0] {
            Content::Text { text } => text.clone(),
            Content::Image { .. } => panic!("text expected"),
        };
        assert!(text.contains("scl __‾‾"), "{text}");

        let r = call(&h, json!({"capture_id": 1, "width": 400})).unwrap();
        let v = r.structured.unwrap();
        assert_eq!(v["width"], 400);
        assert_eq!(v["channels"], json!(["scl", "sda"]));
        match &r.content[0] {
            Content::Image { data, mime_type } => {
                assert_eq!(mime_type, "image/png");
                assert!(data.starts_with("iVBORw0KGgo"), "{}", &data[..20]);
            }
            Content::Text { .. } => panic!("image expected"),
        }
        let e = call(&h, json!({"capture_id": 1, "form": "svg"})).unwrap_err();
        assert!(e.invalid_args);
        let e = call(&h, json!({"capture_id": 1, "channels": ["nope"]})).unwrap_err();
        assert!(e.message.contains("No channel"), "{e}");
        let e = call(&h, json!({"capture_id": 1, "analyzer_id": 7})).unwrap_err();
        assert!(e.message.contains("No such analyzer"), "{e}");
    }
}
