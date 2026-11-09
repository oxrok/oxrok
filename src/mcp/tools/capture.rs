// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The capture tools: acquiring into the store, loading and exporting
//! files, listing and dropping, and the summary.

use crate::core::session::Flow;
use crate::core::{Key, Packet, Session};
use crate::formats::output::OutputContext;
use crate::formats::sr::{SessionReader, is_session_file};
use crate::mcp::Host;
use crate::mcp::capture::Capture;
use crate::mcp::tool::{Annotations, Call, Tag, Tool, ToolError, ToolResult, ToolSpec, function};
use crate::setup::{self, CaptureLimits};
use schemars::JsonSchema;
use serde::Deserialize;
use serde_json::{Value, json};
use std::io::Read;
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::sync::mpsc;
use std::time::Duration;

/// `capture`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CaptureArgs {
    /// The handle `open` returned.
    pub handle: u64,
    /// How many samples, with a size suffix: `1k`, `2m`. One of `samples`,
    /// `time` or `frames` is needed.
    #[serde(default)]
    pub samples: Option<String>,
    /// How long, in ms or with a unit: `10ms`, `2s`.
    #[serde(default)]
    pub time: Option<String>,
    /// How many frames (oscilloscope sweeps).
    #[serde(default)]
    pub frames: Option<String>,
    /// A trigger as `-t` takes it: `D0=r` (rising), `D1=f`, `D2=1`,
    /// `D0=r,D1=1` for one stage, `;` between stages.
    #[serde(default)]
    pub triggers: Option<String>,
    /// Keep only the samples from the trigger on.
    #[serde(default)]
    pub wait_trigger: bool,
    /// Options applied first, as `-c` takes them: `samplerate=1m`,
    /// `channel_group=Logic:pattern=walking-one`.
    #[serde(default)]
    pub config: Vec<String>,
    /// Give up after this many seconds, keeping what came. The server's
    /// limit otherwise.
    #[serde(default)]
    pub timeout: Option<f64>,
}

/// A capture id alone.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CaptureIdArgs {
    /// The id `capture` or `load_capture` returned.
    pub capture_id: u64,
}

/// No arguments.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NoArgs {}

/// `load_capture`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LoadArgs {
    /// The file: a `.sr` session, a VCD, or raw binary samples.
    pub path: String,
    /// The input module and its options as `-I` takes them
    /// (`vcd:downsample=2`, `binary:numchannels=8:samplerate=1m`); detected
    /// from the file when left out.
    #[serde(default)]
    pub format: Option<String>,
    /// The channels to keep, as `-C` takes them.
    #[serde(default)]
    pub channels: Vec<String>,
}

/// `export_capture`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExportArgs {
    /// The id `capture` or `load_capture` returned.
    pub capture_id: u64,
    /// The file to write.
    pub path: String,
    /// The output module and its options as `-O` takes them: `srzip`,
    /// `csv`, `vcd`, `bits:width=64`, ... From the extension when left out
    /// (`.sr`, `.csv`, `.vcd`), else `srzip`.
    #[serde(default)]
    pub format: Option<String>,
}

/// The capture tools over `host`.
#[allow(clippy::too_many_lines)]
pub fn tools(host: &Arc<Host>) -> Vec<Arc<dyn Tool>> {
    let h = host.clone();
    let capture = function::<CaptureArgs, _>(
        ToolSpec {
            name: "capture",
            title: "Capture",
            description: "Runs one acquisition on an open device and keeps it in the store: \
                          applies `config`, sets the limit (`samples`, `time` or `frames`), \
                          arms `triggers`, runs until the limit or the timeout, and returns \
                          the `capture_id` with the summary (`capture_summary`). The data \
                          stays on the server: read it with `capture_logic`, `measure`, \
                          `find` and `capture_analog`, or write it with `export_capture`.",
            example: json!({"handle": 1, "samples": "1k", "config": ["samplerate=1m"]}),
            annotations: Annotations::HARDWARE,
            tags: &[Tag::Capture, Tag::Hardware],
        },
        move |args, cx| {
            let open = h.take_device(args.handle)?;
            let (device, result) = run_capture(&h, open.device, &args, cx);
            h.put_device(args.handle, crate::mcp::host::Open { device_id: open.device_id, device });
            let (mut capture, timed_out) = result?;
            capture.source = format!("{} (handle {})", capture.source, args.handle);
            let (id, evicted) = h.captures().add(capture);
            let mut v = h.captures().get(id).map(Capture::summary).unwrap_or_default();
            v["evicted"] = json!(evicted);
            v["timed_out"] = json!(timed_out);
            v["cancelled"] = json!(cx.cancelled());
            Ok(ToolResult::structured(v))
        },
    );

    let h = host.clone();
    let list_captures = function::<NoArgs, _>(
        ToolSpec {
            name: "list_captures",
            title: "List captures",
            description: "The captures in the store: id, source, channels, sample count, \
                          rate and size, oldest first, with the memory used and the budget.",
            example: json!({}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Capture],
        },
        move |_, _| {
            let store = h.captures();
            let list: Vec<Value> = store.iter().map(Capture::entry).collect();
            Ok(ToolResult::structured(json!({
                "count": list.len(),
                "captures": list,
                "bytes": store.used(),
                "max_bytes": h.limits().max_memory,
            })))
        },
    );

    let h = host.clone();
    let drop_capture = function::<CaptureIdArgs, _>(
        ToolSpec {
            name: "drop_capture",
            title: "Drop a capture",
            description: "Removes a capture from the store and frees its memory, with the \
                          analyzers over it.",
            example: json!({"capture_id": 1}),
            annotations: Annotations::MUTATING,
            tags: &[Tag::Capture],
        },
        move |args, _| {
            h.captures().remove(args.capture_id).ok_or_else(|| no_capture(args.capture_id))?;
            let analyzers = h.remove_analyzers_of(args.capture_id);
            Ok(ToolResult::structured(json!({
                "dropped": args.capture_id,
                "analyzers_dropped": analyzers,
            })))
        },
    );

    let h = host.clone();
    let capture_summary = function::<CaptureIdArgs, _>(
        ToolSpec {
            name: "capture_summary",
            title: "Capture summary",
            description: "A capture at a glance: rate, sample count, duration, trigger \
                          position, and per logic channel its transitions, duty cycle, \
                          first and last level and shortest pulse; per analog channel min, \
                          max, mean and RMS with the unit. Read this before asking for data.",
            example: json!({"capture_id": 1}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Capture],
        },
        move |args, _| {
            let store = h.captures();
            let c = store.get(args.capture_id).ok_or_else(|| no_capture(args.capture_id))?;
            Ok(ToolResult::structured(c.summary()))
        },
    );

    let h = host.clone();
    let load_capture = function::<LoadArgs, _>(
        ToolSpec {
            name: "load_capture",
            title: "Load a file",
            description: "Reads a `.sr` session file, a VCD or raw binary samples into the \
                          store, under the server's allowed directories, and returns the \
                          `capture_id` with the summary.",
            example: json!({"path": "capture.sr"}),
            annotations: Annotations::MUTATING,
            tags: &[Tag::Capture],
        },
        move |args, _| {
            let path = h.check_path(&args.path)?;
            let capture = load(&path, args.format.as_deref(), &args.channels)?;
            let (id, evicted) = h.captures().add(capture);
            let mut v = h.captures().get(id).map(Capture::summary).unwrap_or_default();
            v["evicted"] = json!(evicted);
            Ok(ToolResult::structured(v))
        },
    );

    let h = host.clone();
    let export_capture = function::<ExportArgs, _>(
        ToolSpec {
            name: "export_capture",
            title: "Export a capture",
            description: "Writes a capture to a file with an output module (`srzip` for a \
                          `.sr` PulseView can open, `csv`, `vcd`, `bits`, ...) under the \
                          server's allowed directories. Returns the path and size, never the \
                          bytes.",
            example: json!({"capture_id": 1, "path": "capture.sr"}),
            annotations: Annotations {
                read_only: false,
                destructive: true,
                idempotent: true,
                open_world: false,
            },
            tags: &[Tag::Capture],
        },
        move |args, _| {
            let path = h.check_path(&args.path)?;
            let store = h.captures();
            let c = store.get(args.capture_id).ok_or_else(|| no_capture(args.capture_id))?;
            let bytes = export(c, &path, args.format.as_deref())?;
            Ok(ToolResult::structured(json!({
                "capture_id": args.capture_id,
                "path": path,
                "bytes": bytes,
            })))
        },
    );

    vec![capture, list_captures, drop_capture, capture_summary, load_capture, export_capture]
}

/// "No capture" in one place, so the tools agree.
pub fn no_capture(id: u64) -> ToolError {
    ToolError::new(format!("No capture {id}; `list_captures` has the ids."))
}

type CaptureRun = (Box<dyn crate::core::Device>, Result<(Capture, bool), ToolError>);

/// Runs the acquisition and gives the device back whatever happened.
#[allow(clippy::single_match_else)]
fn run_capture(
    host: &Host,
    mut device: Box<dyn crate::core::Device>,
    args: &CaptureArgs,
    cx: &Call,
) -> CaptureRun {
    let prepared = prepare(device.as_mut(), args);
    let (trigger, plan) = match prepared {
        Ok(p) => p,
        Err(e) => return (device, Err(e)),
    };
    let samplerate = device.get(&Key::Samplerate, None).ok().and_then(|v| v.as_u64());
    let channels = device.channels().to_vec();
    let info = device.info().clone();
    let source = format!("{} {}", info.driver, info.model).trim().to_owned();
    if let Err(e) = plan.send_enables(device.as_mut()) {
        return (device, Err(e.into()));
    }
    let mut capture = Capture::new(source, channels, samplerate);
    capture.device = Some(info);
    let mut session = Session::new();
    session.add_device(device);
    session.set_trigger(trigger);
    let stop = session.stop_flag();
    let timeout = args.timeout.map_or(host.limits().capture_timeout, Duration::from_secs_f64);
    // The watchdog ends with the run: dropping `done` wakes it.
    let (done, wake) = mpsc::channel::<()>();
    let timed_out = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let watchdog = {
        let stop = stop.clone();
        let timed_out = timed_out.clone();
        std::thread::spawn(move || {
            if wake.recv_timeout(timeout) == Err(mpsc::RecvTimeoutError::Timeout) {
                timed_out.store(true, Ordering::SeqCst);
                stop.store(true, Ordering::SeqCst);
            }
        })
    };
    let mut triggered = false;
    let mut seen = 0u64;
    let result = session.run(|_, packet| {
        if cx.cancelled() {
            return Flow::Stop;
        }
        match packet {
            Packet::Trigger => triggered = true,
            Packet::Logic(l) if args.wait_trigger && !triggered => {
                let _ = l;
                return Flow::Continue;
            }
            Packet::Logic(l) => {
                seen += l.len() as u64;
                if seen / 100_000 != (seen - l.len() as u64) / 100_000 {
                    cx.progress(seen as f64, None, Some("samples"));
                }
            }
            _ => {}
        }
        capture.push(packet);
        Flow::Continue
    });
    drop(done);
    let _ = watchdog.join();
    let device = session.into_devices().remove(0);
    match result {
        Ok(()) => {}
        Err(crate::core::Error::CaptureFailed(why)) => {
            capture.failed.get_or_insert(why);
        }
        Err(e) => return (device, Err(e.into())),
    }
    (device, Ok((capture, timed_out.load(Ordering::SeqCst))))
}

/// `-c`, the limit and the trigger, in the command line's order, before
/// the outputs are switched on.
fn prepare(
    device: &mut dyn crate::core::Device,
    args: &CaptureArgs,
) -> Result<(Option<crate::core::Trigger>, setup::ConfigPlan), ToolError> {
    let plan = setup::plan_configs(&*device, &args.config, None)?;
    plan.send_settings(device)?;
    if args.samples.is_none() && args.time.is_none() && args.frames.is_none() {
        return Err(ToolError::invalid_args(
            "Give a limit: `samples` (`1k`), `time` (`10ms`) or `frames`.",
        ));
    }
    let limits = CaptureLimits {
        time: args.time.clone(),
        samples: args.samples.clone(),
        frames: args.frames.clone(),
        continuous: false,
    };
    setup::apply_limits(device, &limits)?;
    let trigger =
        args.triggers.as_deref().map(|spec| setup::parse_triggers(device, spec)).transpose()?;
    device.check_capture(trigger.as_ref())?;
    Ok((trigger, plan))
}

/// Reads a file into a capture.
fn load(path: &Path, format: Option<&str>, channels: &[String]) -> Result<Capture, ToolError> {
    let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    if format.is_none() && is_session_file(path) {
        let mut reader = SessionReader::open(path)?;
        if !channels.is_empty() {
            setup::select_channel_list(reader.channels_mut(), &channels.join(","))?;
        }
        let rate = reader.metadata().samplerate;
        let mut capture = Capture::new(name, reader.channels().to_vec(), rate);
        reader.read(&mut |p| capture.push(&p))?;
        return Ok(capture);
    }
    let mut file = std::fs::File::open(path)
        .map_err(|e| ToolError::new(format!("Failed to load {}: {e}.", path.display())))?;
    let (module, options, head) = if let Some(spec) = format {
        let (m, o) = setup::input_spec(spec)?;
        (m, o, Vec::new())
    } else {
        let mut head = vec![0u8; 4096];
        let n = file.read(&mut head)?;
        head.truncate(n);
        let m = crate::formats::input::detect(&head, Some(&name))
            .ok_or_else(|| ToolError::new("Error: no input module found for this file."))?;
        (m, crate::formats::input::Options::new(), head)
    };
    let mut input = module.instantiate(&options)?;
    let mut capture: Option<Capture> = None;
    let mut packets = Vec::new();
    let mut buf = vec![0u8; 1 << 20];
    let mut pending = head;
    loop {
        let n = if pending.is_empty() {
            file.read(&mut buf)?
        } else {
            let n = pending.len();
            buf[..n].copy_from_slice(&pending);
            pending.clear();
            n
        };
        if n == 0 {
            input.end(&mut packets)?;
        } else {
            input.feed(&buf[..n], &mut packets)?;
        }
        if capture.is_none()
            && let Some(chans) = input.channels()
        {
            let mut chans = chans.to_vec();
            if !channels.is_empty() {
                setup::select_channel_list(&mut chans, &channels.join(","))?;
                for (pos, c) in chans.iter().enumerate() {
                    input.enable_channel(pos, c.enabled);
                }
            }
            capture = Some(Capture::new(name.clone(), chans, None));
        }
        if let Some(c) = &mut capture {
            for p in packets.drain(..) {
                c.push(&p);
            }
        } else {
            packets.clear();
        }
        if n == 0 {
            break;
        }
    }
    capture.ok_or_else(|| ToolError::new("File import failed: no channels found."))
}

/// Writes a capture with an output module; returns the bytes written.
fn export(c: &Capture, path: &Path, format: Option<&str>) -> Result<u64, ToolError> {
    let by_extension = || {
        match path.extension().and_then(|e| e.to_str()) {
            Some("csv") => "csv",
            Some("vcd") => "vcd",
            Some("bits" | "txt") => "bits",
            _ => "srzip",
        }
        .to_owned()
    };
    let spec = format.map_or_else(by_extension, str::to_owned);
    let (module, options) = setup::output_spec(&spec)?;
    let ctx = OutputContext {
        channels: &c.channels,
        samplerate: c.samplerate,
        title: c.source.clone(),
        filename: Some(path.display().to_string()),
    };
    let mut out = module.instantiate(&ctx, &options)?;
    let mut bytes = Vec::new();
    let mut feed = |p: &Packet| -> Result<(), ToolError> {
        out.receive(p, &mut bytes)?;
        Ok(())
    };
    feed(&Packet::header())?;
    if let Some(rate) = c.samplerate {
        feed(&Packet::meta(Key::Samplerate, rate))?;
    }
    // Logic in chunks and analog blocks, in sample order.
    let unit = c.unit_size.max(1);
    let chunk_samples = 65_536u64;
    let total = c.logic_samples();
    let mut at = 0u64;
    let mut analog = c.analog.iter().peekable();
    while at < total || analog.peek().is_some() {
        let next_logic = if at < total { Some(at) } else { None };
        let next_analog = analog.peek().map(|b| b.at);
        let logic_first = match (next_logic, next_analog) {
            (Some(l), Some(a)) => l <= a,
            (Some(_), None) => true,
            _ => false,
        };
        if logic_first {
            if c.trigger_at == Some(at) {
                feed(&Packet::Trigger)?;
            }
            let mut end = (at + chunk_samples).min(total);
            if let Some(t) = c.trigger_at
                && t > at
                && t < end
            {
                end = t;
            }
            let data = c.logic[at as usize * unit..end as usize * unit].to_vec();
            feed(&Packet::logic(unit, data))?;
            at = end;
        } else if let Some(block) = analog.next() {
            feed(&Packet::Analog(block.data.clone()))?;
        }
    }
    feed(&Packet::End)?;
    if out.writes_file() {
        let size = std::fs::metadata(path).map_or(0, |m| m.len());
        return Ok(size);
    }
    std::fs::write(path, &bytes)
        .map_err(|e| ToolError::new(format!("Cannot write '{}': {e}", path.display())))?;
    Ok(bytes.len() as u64)
}

#[cfg(test)]
#[allow(clippy::many_single_char_names)]
mod tests {
    use super::*;
    use crate::mcp::Limits;

    fn host(dir: &Path) -> Arc<Host> {
        let mut r = crate::core::Registry::new();
        r.register(Box::new(crate::drivers::demo::DemoDriver));
        Arc::new(
            Host::with(r, crate::decode::Registry::new())
                .with_limits(Limits { allow_paths: vec![dir.to_path_buf()], ..Limits::default() }),
        )
    }

    fn call(host: &Arc<Host>, name: &str, args: Value) -> Result<Value, ToolError> {
        let tools = tools(host);
        let tool = tools.iter().find(|t| t.desc().name == name).unwrap();
        tool.call(args, &mut Call::detached()).map(|r| r.structured.unwrap())
    }

    fn temp_dir(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("oxrok-mcp-{name}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn captures_exports_and_loads_back() {
        let dir = temp_dir("capture");
        let h = host(&dir);
        let handle = h.open(None, Some("demo:logic_channels=4:analog_channels=1")).unwrap();
        let s = call(
            &h,
            "capture",
            json!({
                "handle": handle,
                "samples": "100",
                "config": ["samplerate=1m", "channel_group=Logic:pattern=walking-one"],
            }),
        )
        .unwrap();
        assert_eq!(s["capture_id"], 1);
        assert_eq!(s["samplerate"], 1_000_000);
        assert_eq!(s["samples"], 100);
        assert_eq!(s["duration"], 0.0001);
        assert_eq!(s["timed_out"], false);
        assert_eq!(s["failed"], Value::Null);
        let d0 = &s["logic"][0];
        assert_eq!(d0["name"], "D0");
        // The demo's walking one: D0 is high one sample in five.
        assert_eq!(d0["duty"], 0.2);
        assert_eq!(d0["min_pulse_samples"], 1);
        assert_eq!(s["analog"][0]["name"], "A0");
        assert!(s["analog"][0]["samples"].as_u64().unwrap() > 0);
        // The device is back and idle.
        assert_eq!(h.open_devices(), [(handle, "demo".to_owned(), false)]);

        let l = call(&h, "list_captures", json!({})).unwrap();
        assert_eq!(l["count"], 1);
        assert_eq!(l["captures"][0]["samples"], 100);

        let sr = dir.join("out.sr");
        let e = call(&h, "export_capture", json!({"capture_id": 1, "path": sr})).unwrap();
        assert!(e["bytes"].as_u64().unwrap() > 0);
        assert!(is_session_file(&sr));
        let csv = dir.join("out.csv");
        call(&h, "export_capture", json!({"capture_id": 1, "path": csv, "format": "csv"})).unwrap();
        let text = std::fs::read_to_string(&csv).unwrap();
        assert!(text.contains("D0"), "{text}");

        let back = call(&h, "load_capture", json!({"path": sr})).unwrap();
        assert_eq!(back["capture_id"], 2);
        assert_eq!(back["samples"], 100);
        assert_eq!(back["samplerate"], 1_000_000);
        assert_eq!(back["logic"][0]["duty"], 0.2);
        let vcd = dir.join("out.vcd");
        call(&h, "export_capture", json!({"capture_id": 1, "path": vcd})).unwrap();
        let back =
            call(&h, "load_capture", json!({"path": vcd, "channels": ["D0", "D1"]})).unwrap();
        assert_eq!(back["logic"].as_array().unwrap().len(), 2);

        assert_eq!(call(&h, "drop_capture", json!({"capture_id": 1})).unwrap()["dropped"], 1);
        let e = call(&h, "capture_summary", json!({"capture_id": 1})).unwrap_err();
        assert!(e.message.contains("No capture 1"), "{e}");
        let e =
            call(&h, "export_capture", json!({"capture_id": 2, "path": "/etc/x.sr"})).unwrap_err();
        assert!(e.message.contains("outside"), "{e}");
        let e = call(&h, "load_capture", json!({"path": dir.join("missing.sr")})).unwrap_err();
        assert!(e.message.contains("Failed to load"), "{e}");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_capture_needs_a_limit_and_a_good_config() {
        let dir = temp_dir("limits");
        let h = host(&dir);
        let handle = h.open(None, Some("demo:logic_channels=2")).unwrap();
        let e = call(&h, "capture", json!({"handle": handle})).unwrap_err();
        assert!(e.invalid_args, "{e}");
        let e =
            call(&h, "capture", json!({"handle": handle, "samples": "10", "config": ["nope=1"]}))
                .unwrap_err();
        assert_eq!(e.message, "Unknown device option 'nope'.");
        let e = call(&h, "capture", json!({"handle": handle, "samples": "lots"})).unwrap_err();
        assert_eq!(e.message, "Invalid sample limit 'lots'.");
        let e = call(&h, "capture", json!({"handle": 9, "samples": "10"})).unwrap_err();
        assert!(e.message.contains("No open device"), "{e}");
        // Still usable after the refusals.
        let s = call(&h, "capture", json!({"handle": handle, "time": "1ms"})).unwrap();
        assert_eq!(s["samples"], 200);
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
