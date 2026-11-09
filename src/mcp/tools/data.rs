// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The data tools: what an agent reads from a capture. Transitions and
//! statistics by default, samples only on request and capped.

use super::capture::no_capture;
use crate::core::{Channel, ChannelKind};
use crate::mcp::Host;
use crate::mcp::capture::{self, Capture, Find};
use crate::mcp::tool::{Annotations, Tag, Tool, ToolError, ToolResult, ToolSpec, function};
use schemars::JsonSchema;
use serde::Deserialize;
use serde_json::{Value, json};
use std::sync::Arc;

/// A range of a capture, in samples or seconds.
#[derive(Deserialize, JsonSchema, Default)]
#[serde(deny_unknown_fields)]
pub struct Range {
    /// The first sample, 0 when left out.
    #[serde(default)]
    pub from: Option<u64>,
    /// One past the last sample, the end when left out.
    #[serde(default)]
    pub to: Option<u64>,
    /// The start in seconds instead of `from`.
    #[serde(default)]
    pub from_time: Option<f64>,
    /// The end in seconds instead of `to`.
    #[serde(default)]
    pub to_time: Option<f64>,
}

impl Range {
    /// The sample range on `c`.
    fn resolve(&self, c: &Capture, total: u64) -> Result<(u64, u64), ToolError> {
        let at = |samples: Option<u64>,
                  seconds: Option<f64>,
                  default: u64|
         -> Result<u64, ToolError> {
            match (samples, seconds) {
                (Some(s), _) => Ok(s),
                (None, Some(t)) => c.sample_at(t).ok_or_else(|| {
                    ToolError::invalid_args("The capture has no samplerate; give sample numbers.")
                }),
                (None, None) => Ok(default),
            }
        };
        let from = at(self.from, self.from_time, 0)?;
        let to = at(self.to, self.to_time, total)?.min(total);
        if from > to {
            return Err(ToolError::invalid_args(format!("`from` ({from}) is after `to` ({to}).")));
        }
        Ok((from, to))
    }
}

/// `capture_logic`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LogicArgs {
    /// The id `capture` or `load_capture` returned.
    pub capture_id: u64,
    /// The channel names; every enabled logic channel when left out.
    #[serde(default)]
    pub channels: Vec<String>,
    /// The range of samples.
    #[serde(flatten)]
    pub range: Range,
    /// `transitions` (default): where each channel changes, as
    /// `{sample, t, level}`, with the level at `from`; `rle`: runs as
    /// `{sample, level, length}`; `samples`: the levels as 0 and 1, which
    /// costs a token per sample.
    #[serde(default)]
    pub form: Option<String>,
    /// Skip this many items per channel.
    #[serde(default)]
    pub offset: Option<usize>,
    /// At most this many items per channel: 200 by default, 5000 at most
    /// (10000 for `samples`).
    #[serde(default)]
    pub limit: Option<usize>,
}

/// `capture_analog`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AnalogArgs {
    /// The id `capture` or `load_capture` returned.
    pub capture_id: u64,
    /// The analog channel's name.
    pub channel: String,
    /// The range of samples.
    #[serde(flatten)]
    pub range: Range,
    /// How many buckets to reduce the range to, each with min, max and
    /// mean: 100 by default, 2000 at most.
    #[serde(default)]
    pub buckets: Option<usize>,
}

/// `measure`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeasureArgs {
    /// The id `capture` or `load_capture` returned.
    pub capture_id: u64,
    /// The channel's name.
    pub channel: String,
    /// The range of samples.
    #[serde(flatten)]
    pub range: Range,
}

/// `find`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FindArgs {
    /// The id `capture` or `load_capture` returned.
    pub capture_id: u64,
    /// The logic channel's name.
    pub channel: String,
    /// Look from this sample on; 0 when left out.
    #[serde(default)]
    pub after: Option<u64>,
    /// Look from this time on, in seconds, instead of `after`.
    #[serde(default)]
    pub after_time: Option<f64>,
    /// `edge` (default), `rising`, `falling`, or `pulse` for the first
    /// run (high or low) at least `min_width` long.
    #[serde(default)]
    pub what: Option<String>,
    /// With `pulse`: the shortest run wanted, in seconds.
    #[serde(default)]
    pub min_width: Option<f64>,
    /// With `pulse`: the shortest run wanted, in samples.
    #[serde(default)]
    pub min_width_samples: Option<u64>,
}

/// The data tools over `host`.
#[allow(clippy::too_many_lines)]
pub fn tools(host: &Arc<Host>) -> Vec<Arc<dyn Tool>> {
    let h = host.clone();
    let capture_logic = function::<LogicArgs, _>(
        ToolSpec {
            name: "capture_logic",
            title: "Logic data",
            description: "Logic channels of a capture over a range, as transitions by \
                          default: the level at `from` and where it changes, with sample \
                          numbers and seconds. `rle` gives runs, `samples` the raw levels \
                          (capped). Paged with `offset` and `limit`; `truncated` says when \
                          there is more. Prefer `measure` and `find` to reading samples.",
            example: json!({"capture_id": 1, "channels": ["D0"], "limit": 50}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Capture],
        },
        move |args, _| {
            let store = h.captures();
            let c = store.get(args.capture_id).ok_or_else(|| no_capture(args.capture_id))?;
            let (from, to) = args.range.resolve(c, c.logic_samples())?;
            let form = args.form.as_deref().unwrap_or("transitions");
            let (default_limit, max_limit) = match form {
                "transitions" | "rle" => (200, 5000),
                "samples" => (1000, 10_000),
                other => {
                    return Err(ToolError::invalid_args(format!(
                        "Unknown form '{other}': transitions, rle or samples."
                    )));
                }
            };
            let offset = args.offset.unwrap_or(0);
            let limit = args.limit.unwrap_or(default_limit).clamp(1, max_limit);
            let chans: Vec<&Channel> = if args.channels.is_empty() {
                c.logic_channels().collect()
            } else {
                args.channels.iter().map(|n| logic_channel(c, n)).collect::<Result<Vec<_>, _>>()?
            };
            let channels: Vec<Value> = chans
                .iter()
                .map(|ch| {
                    let mut v = json!({"name": ch.name, "index": ch.index});
                    if form == "samples" {
                            let start = from.saturating_add(offset as u64);
                            let levels: Vec<u8> = c
                                .levels(ch.index, start, to)
                                .take(limit)
                                .map(u8::from)
                                .collect();
                            let n = levels.len() as u64;
                            v["from"] = json!(start);
                            v["samples"] = json!(levels);
                            v["truncated"] = json!(start + n < to);
                    } else {
                            let edges = c.transitions(ch.index, from, to);
                            v["level_at_from"] = json!(from < to && c.level(ch.index, from));
                            v["total_transitions"] = json!(edges.len());
                            if form == "rle" {
                                let mut runs = Vec::new();
                                let mut start = from;
                                let mut level = from < to && c.level(ch.index, from);
                                for (s, l) in &edges {
                                    runs.push(json!({"sample": start, "t": c.time(start), "level": level, "length": s - start}));
                                    start = *s;
                                    level = *l;
                                }
                                if from < to {
                                    runs.push(json!({"sample": start, "t": c.time(start), "level": level, "length": to - start}));
                                }
                                let page: Vec<Value> = runs.iter().skip(offset).take(limit).cloned().collect();
                                v["truncated"] = json!(offset.saturating_add(page.len()) < runs.len());
                                v["runs"] = json!(page);
                            } else {
                                let page: Vec<Value> = edges
                                    .iter()
                                    .skip(offset)
                                    .take(limit)
                                    .map(|(s, l)| json!({"sample": s, "t": c.time(*s), "level": l}))
                                    .collect();
                                v["truncated"] = json!(offset.saturating_add(page.len()) < edges.len());
                                v["transitions"] = json!(page);
                            }
                        }
                    v
                })
                .collect();
            Ok(ToolResult::structured(json!({
                "capture_id": c.id,
                "samplerate": c.samplerate,
                "from": from,
                "to": to,
                "form": form,
                "offset": offset,
                "limit": limit,
                "channels": channels,
            })))
        },
    );

    let h = host.clone();
    let capture_analog = function::<AnalogArgs, _>(
        ToolSpec {
            name: "capture_analog",
            title: "Analog data",
            description: "One analog channel of a capture over a range, reduced to buckets \
                          with min, max and mean each, so a long trace is a short table. \
                          With the unit and what was measured.",
            example: json!({"capture_id": 1, "channel": "A0", "buckets": 20}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Capture],
        },
        move |args, _| {
            let store = h.captures();
            let c = store.get(args.capture_id).ok_or_else(|| no_capture(args.capture_id))?;
            let ch = analog_channel(c, &args.channel)?;
            let total = c.analog_samples(ch.index);
            let (from, to) = args.range.resolve(c, total)?;
            let buckets = args.buckets.unwrap_or(100).clamp(1, 2000);
            let (mq, unit) = c.analog_kind(ch.index).unwrap_or(("", ""));
            Ok(ToolResult::structured(json!({
                "capture_id": c.id,
                "channel": ch.name,
                "mq": mq,
                "unit": unit,
                "samplerate": c.samplerate,
                "from": from,
                "to": to,
                "buckets": capture::analog_buckets(c, ch.index, from, to, buckets),
            })))
        },
    );

    let h = host.clone();
    let measure = function::<MeasureArgs, _>(
        ToolSpec {
            name: "measure",
            title: "Measure a channel",
            description: "Measurements over a range of one channel. Logic: transitions, \
                          rising and falling edges, frequency and period (from rising \
                          edges), duty cycle, high and low pulse widths (min, max, mean). \
                          Analog: min, max, peak to peak, mean, RMS, and the frequency by \
                          crossings of the mean. Seconds where the rate is known.",
            example: json!({"capture_id": 1, "channel": "D0"}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Capture],
        },
        move |args, _| {
            let store = h.captures();
            let c = store.get(args.capture_id).ok_or_else(|| no_capture(args.capture_id))?;
            let ch = channel(c, &args.channel)?;
            let mut v = if ch.kind == ChannelKind::Logic {
                let (from, to) = args.range.resolve(c, c.logic_samples())?;
                capture::measure_logic(c, ch.index, from, to)
            } else {
                let (from, to) = args.range.resolve(c, c.analog_samples(ch.index))?;
                capture::measure_analog(c, ch.index, from, to)
            };
            v["capture_id"] = json!(c.id);
            v["channel"] = json!(ch.name);
            v["kind"] = json!(format!("{:?}", ch.kind).to_lowercase());
            v["samplerate"] = json!(c.samplerate);
            Ok(ToolResult::structured(v))
        },
    );

    let h = host.clone();
    let find = function::<FindArgs, _>(
        ToolSpec {
            name: "find",
            title: "Find an edge or pulse",
            description: "The first edge (any, rising or falling) or the first pulse at least \
                          `min_width` long on a logic channel at or after a sample or time: \
                          how to walk a trace without reading it. Returns the sample, the \
                          time and the level after it, or `found: false`.",
            example: json!({"capture_id": 1, "channel": "D0", "what": "rising"}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Capture],
        },
        move |args, _| {
            let store = h.captures();
            let c = store.get(args.capture_id).ok_or_else(|| no_capture(args.capture_id))?;
            let ch = logic_channel(c, &args.channel)?;
            let after = match (args.after, args.after_time) {
                (Some(s), _) => s,
                (None, Some(t)) => c.sample_at(t).ok_or_else(|| {
                    ToolError::invalid_args(
                        "The capture has no samplerate; give `after` in samples.",
                    )
                })?,
                (None, None) => 0,
            };
            let what = match args.what.as_deref().unwrap_or("edge") {
                "edge" => Find::Edge,
                "rising" => Find::Rising,
                "falling" => Find::Falling,
                "pulse" => {
                    let width = match (args.min_width_samples, args.min_width) {
                        (Some(s), _) => s,
                        (None, Some(t)) => c.sample_at(t).ok_or_else(|| {
                            ToolError::invalid_args(
                                "The capture has no samplerate; give `min_width_samples`.",
                            )
                        })?,
                        (None, None) => {
                            return Err(ToolError::invalid_args(
                                "`pulse` needs `min_width` (seconds) or `min_width_samples`.",
                            ));
                        }
                    };
                    Find::PulseAtLeast(width.max(1))
                }
                other => {
                    return Err(ToolError::invalid_args(format!(
                        "Unknown `what` '{other}': edge, rising, falling or pulse."
                    )));
                }
            };
            let hit = capture::find(c, ch.index, after, what);
            Ok(ToolResult::structured(match hit {
                Some((sample, level)) => json!({
                    "capture_id": c.id, "channel": ch.name, "found": true,
                    "sample": sample, "t": c.time(sample), "level": level,
                }),
                None => json!({"capture_id": c.id, "channel": ch.name, "found": false}),
            }))
        },
    );

    vec![capture_logic, capture_analog, measure, find]
}

fn channel<'a>(c: &'a Capture, name: &str) -> Result<&'a Channel, ToolError> {
    c.channel(name).ok_or_else(|| {
        let names: Vec<&str> = c.channels.iter().map(|ch| ch.name.as_str()).collect();
        ToolError::new(format!(
            "No channel '{name}' in capture {}; it has: {}.",
            c.id,
            names.join(", ")
        ))
    })
}

fn logic_channel<'a>(c: &'a Capture, name: &str) -> Result<&'a Channel, ToolError> {
    let ch = channel(c, name)?;
    if ch.kind != ChannelKind::Logic {
        return Err(ToolError::new(format!("Channel '{name}' is analog; this wants a logic one.")));
    }
    Ok(ch)
}

fn analog_channel<'a>(c: &'a Capture, name: &str) -> Result<&'a Channel, ToolError> {
    let ch = channel(c, name)?;
    if ch.kind != ChannelKind::Analog {
        return Err(ToolError::new(format!(
            "Channel '{name}' is logic; this wants an analog one."
        )));
    }
    Ok(ch)
}

#[cfg(test)]
#[allow(clippy::many_single_char_names)]
mod tests {
    use super::*;
    use crate::core::Packet;
    use crate::mcp::tool::Call;

    fn host_with(capture: Capture) -> Arc<Host> {
        let h = Host::with(crate::core::Registry::new(), crate::decode::Registry::new());
        h.captures().add(capture);
        Arc::new(h)
    }

    fn call(host: &Arc<Host>, name: &str, args: Value) -> Result<Value, ToolError> {
        let tools = tools(host);
        let tool = tools.iter().find(|t| t.desc().name == name).unwrap();
        tool.call(args, &mut Call::detached()).map(|r| r.structured.unwrap())
    }

    /// D0 is bit 0, D1 bit 1, at 1 kHz: D0 = 0011110000111100, D1 = all high.
    fn two_channels() -> Capture {
        let mut c = Capture::new(
            "test",
            vec![
                Channel::new(0, ChannelKind::Logic, "D0"),
                Channel::new(1, ChannelKind::Logic, "D1"),
            ],
            Some(1000),
        );
        let d0 = "0011110000111100";
        let data: Vec<u8> = d0.bytes().map(|b| u8::from(b == b'1') | 2).collect();
        c.push(&Packet::logic(1, data));
        c
    }

    #[test]
    fn logic_in_three_forms() {
        let h = host_with(two_channels());
        let v = call(&h, "capture_logic", json!({"capture_id": 1})).unwrap();
        assert_eq!(v["to"], 16);
        assert_eq!(v["channels"].as_array().unwrap().len(), 2);
        let d0 = &v["channels"][0];
        assert_eq!(d0["level_at_from"], false);
        assert_eq!(d0["total_transitions"], 4);
        assert_eq!(d0["transitions"][0], json!({"sample": 2, "t": 0.002, "level": true}));
        assert_eq!(d0["truncated"], false);
        assert_eq!(v["channels"][1]["total_transitions"], 0);

        let v = call(
            &h,
            "capture_logic",
            json!({"capture_id": 1, "channels": ["D0"], "limit": 2, "offset": 1}),
        )
        .unwrap();
        let d0 = &v["channels"][0];
        assert_eq!(d0["transitions"].as_array().unwrap().len(), 2);
        assert_eq!(d0["transitions"][0]["sample"], 6);
        assert_eq!(d0["truncated"], true);

        let v = call(&h, "capture_logic", json!({"capture_id": 1, "channels": ["D0"], "form": "rle", "from_time": 0.005, "to_time": 0.011})).unwrap();
        assert_eq!(v["from"], 5);
        assert_eq!(v["to"], 11);
        let runs = v["channels"][0]["runs"].as_array().unwrap();
        assert_eq!(runs.len(), 3);
        assert_eq!(runs[0], json!({"sample": 5, "t": 0.005, "level": true, "length": 1}));
        assert_eq!(runs[2], json!({"sample": 10, "t": 0.01, "level": true, "length": 1}));

        let v = call(
            &h,
            "capture_logic",
            json!({"capture_id": 1, "channels": ["D0"], "form": "samples", "limit": 6}),
        )
        .unwrap();
        assert_eq!(v["channels"][0]["samples"], json!([0, 0, 1, 1, 1, 1]));
        assert_eq!(v["channels"][0]["truncated"], true);

        let e = call(&h, "capture_logic", json!({"capture_id": 1, "form": "hex"})).unwrap_err();
        assert!(e.invalid_args);
        let e =
            call(&h, "capture_logic", json!({"capture_id": 1, "channels": ["D7"]})).unwrap_err();
        assert_eq!(e.message, "No channel 'D7' in capture 1; it has: D0, D1.");
        let e =
            call(&h, "capture_logic", json!({"capture_id": 1, "from": 9, "to": 3})).unwrap_err();
        assert!(e.invalid_args);
        assert!(call(&h, "capture_logic", json!({"capture_id": 7})).is_err());
    }

    #[test]
    fn measures_and_finds() {
        let h = host_with(two_channels());
        let m = call(&h, "measure", json!({"capture_id": 1, "channel": "D0"})).unwrap();
        assert_eq!(m["kind"], "logic");
        assert_eq!(m["frequency"], 125.0);
        assert_eq!(m["duty"], 0.5);
        let m = call(&h, "measure", json!({"capture_id": 1, "channel": "D1"})).unwrap();
        assert_eq!(m["transitions"], 0);
        assert_eq!(m["frequency"], Value::Null);

        let f =
            call(&h, "find", json!({"capture_id": 1, "channel": "D0", "what": "falling"})).unwrap();
        assert_eq!(f["found"], true);
        assert_eq!(f["sample"], 6);
        assert_eq!(f["t"], 0.006);
        assert_eq!(f["level"], false);
        let f = call(&h, "find", json!({"capture_id": 1, "channel": "D0", "after_time": 0.007}))
            .unwrap();
        assert_eq!(f["sample"], 10);
        let f = call(
            &h,
            "find",
            json!({"capture_id": 1, "channel": "D0", "what": "pulse", "min_width": 0.004}),
        )
        .unwrap();
        assert_eq!(f["sample"], 2);
        let f = call(&h, "find", json!({"capture_id": 1, "channel": "D1"})).unwrap();
        assert_eq!(f["found"], false);
        let e = call(&h, "find", json!({"capture_id": 1, "channel": "D0", "what": "pulse"}))
            .unwrap_err();
        assert!(e.invalid_args);
        let e = call(&h, "capture_analog", json!({"capture_id": 1, "channel": "D0"})).unwrap_err();
        assert!(e.message.contains("is logic"), "{e}");
    }

    #[test]
    fn analog_buckets() {
        let mut c = Capture::new("t", vec![Channel::new(0, ChannelKind::Analog, "A0")], Some(10));
        c.push(&Packet::Analog(crate::core::AnalogData {
            channels: vec![0],
            samples: (0..20).map(|i| i as f32).collect(),
            mq: crate::core::Mq::Voltage,
            flags: crate::core::MqFlags::default(),
            unit: crate::core::Unit::Volt,
            digits: 2,
        }));
        let h = host_with(c);
        let v = call(&h, "capture_analog", json!({"capture_id": 1, "channel": "A0", "buckets": 4}))
            .unwrap();
        assert_eq!(v["unit"], "V");
        let b = v["buckets"].as_array().unwrap();
        assert_eq!(b.len(), 4);
        assert_eq!(
            b[3],
            json!({"sample": 15, "t": 1.5, "samples": 5, "min": 15.0, "max": 19.0, "mean": 17.0})
        );
        let m = call(&h, "measure", json!({"capture_id": 1, "channel": "A0", "from": 10})).unwrap();
        assert_eq!(m["kind"], "analog");
        assert_eq!(m["min"], 10.0);
        assert_eq!(m["vpp"], 9.0);
        let e = call(
            &h,
            "measure",
            json!({"capture_id": 1, "channel": "A0", "from_time": 1.0, "to": 3}),
        )
        .unwrap_err();
        assert!(e.invalid_args);
    }
}
