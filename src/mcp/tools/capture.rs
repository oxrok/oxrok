// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The capture tools: acquiring into the store, synchronously or on a
//! thread the client follows, loading and exporting files, listing and
//! dropping, and the summary.
//!
//! Every acquisition runs on its own thread ([`feed_run`]) and feeds the
//! capture in the store packet by packet, so a running capture can be
//! read, measured and decoded as it grows. `capture` starts one and
//! waits; `capture_start` starts one and returns, and `capture_status`
//! and `capture_stop` follow and end it.

use crate::core::session::Flow;
use crate::core::{Key, Packet, Session, Value as CoreValue};
use crate::formats::output::OutputContext;
use crate::formats::sr::{SessionReader, is_session_file};
use crate::mcp::Host;
use crate::mcp::capture::{Capture, CaptureStore};
use crate::mcp::host::{CaptureRun, Open};
use crate::mcp::tool::{
    Annotations, Call, Tag, Tool, ToolError, ToolResult, ToolSpec, deadline, function, seconds,
};
use crate::setup::{self, CaptureLimits};
use schemars::JsonSchema;
use serde::Deserialize;
use serde_json::{Value, json};
use std::io::Read;
use std::path::Path;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex, PoisonError, Weak, mpsc};
use std::time::{Duration, Instant};

/// `capture` and `capture_start`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CaptureArgs {
    /// The handle `open` returned.
    pub handle: u64,
    /// How many samples, with a size suffix: `1k`, `2m`. `capture` needs
    /// one of `samples`, `time` or `frames`; `capture_start` without one
    /// runs until `capture_stop`.
    #[serde(default)]
    pub samples: Option<String>,
    /// How long, in ms or with a unit: `10ms`, `2s`.
    #[serde(default)]
    pub time: Option<String>,
    /// How many frames (oscilloscope sweeps).
    #[serde(default)]
    pub frames: Option<String>,
    /// A trigger as `-t` takes it: `D0=r` (rising), `D1=f`, `D2=1`,
    /// `D0=r,D1=1` for one stage, `;` between stages. Where the device
    /// takes them: `D0=r*5` (the fifth edge), `D0=1>50us` (high for
    /// longer than 50 us), `D0=f<50us` (a high pulse shorter than 50 us,
    /// at its falling edge; `D0=r<50us` a low one), `D0=f>10us<50us` (a
    /// pulse between two widths), `D0-D2=3..5` (a value range).
    #[serde(default)]
    pub triggers: Option<String>,
    /// Keep only the samples from the trigger on.
    #[serde(default)]
    pub wait_trigger: bool,
    /// Options applied first, as `-c` takes them: `samplerate=1m`,
    /// `channel_group=Logic:pattern=walking-one`.
    #[serde(default)]
    pub config: Vec<String>,
    /// Give up after this many seconds, keeping what came. `capture`
    /// takes the server's limit otherwise; `capture_start` has none.
    #[serde(default)]
    pub timeout: Option<f64>,
}

/// `capture_status`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StatusArgs {
    /// The id `capture_start` returned.
    pub capture_id: u64,
    /// Wait up to this many seconds for the capture to end first,
    /// reporting progress meanwhile.
    #[serde(default)]
    pub wait: Option<f64>,
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
                          `find` and `capture_analog`, or write it with `export_capture`. \
                          For a long or an open-ended run use `capture_start`.",
            example: json!({"handle": 1, "samples": "1k", "config": ["samplerate=1m"]}),
            annotations: Annotations::HARDWARE,
            tags: &[Tag::Capture, Tag::Hardware],
        },
        move |args, cx| {
            let timeout = match args.timeout {
                Some(t) => seconds(t, "timeout")?,
                None => h.limits().capture_timeout,
            };
            let run = start_run(&h, &args, true, Some(timeout))?;
            follow(&h, &run, None, true, cx);
            h.reap_run(run.capture_id);
            let mut v = summary(&h, run.capture_id)?;
            v["handle"] = json!(args.handle);
            v["timed_out"] = json!(run.timed_out());
            v["cancelled"] = json!(cx.cancelled());
            Ok(ToolResult::structured(v))
        },
    );

    let h = host.clone();
    let capture_start = function::<CaptureArgs, _>(
        ToolSpec {
            name: "capture_start",
            title: "Start a capture",
            description: "Starts an acquisition like `capture` and returns at once with the \
                          `capture_id`; without a limit it runs until `capture_stop`. The \
                          capture grows in the store meanwhile: `capture_status` follows it \
                          (`wait` for it to end), and the data tools and `add_analyzer` work \
                          on what has come so far. A run that outgrows the memory budget is \
                          stopped. The device is busy until the capture ends.",
            example: json!({"handle": 1, "config": ["samplerate=1m"]}),
            annotations: Annotations::HARDWARE,
            tags: &[Tag::Capture, Tag::Hardware],
        },
        move |args, _| {
            let continuous = args.samples.is_none() && args.time.is_none() && args.frames.is_none();
            let timeout = args.timeout.map(|t| seconds(t, "timeout")).transpose()?;
            let run = start_run(&h, &args, false, timeout)?;
            let evicted = h.captures().get(run.capture_id).map(|c| c.evicted.clone());
            Ok(ToolResult::structured(json!({
                "capture_id": run.capture_id,
                "handle": args.handle,
                "running": true,
                "continuous": continuous,
                "evicted": evicted.unwrap_or_default(),
            })))
        },
    );

    let h = host.clone();
    let capture_status = function::<StatusArgs, _>(
        ToolSpec {
            name: "capture_status",
            title: "Capture status",
            description: "Whether a capture is still running, its samples, bytes and duration \
                          so far, the trigger, and why it ended when it did (`stopped`: \
                          `capture_stop`, `timeout`, `memory budget`; `failed` for a device \
                          error). With `wait`, waits up to that many seconds for it to end, \
                          sending progress meanwhile.",
            example: json!({"capture_id": 1, "wait": 5}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Capture],
        },
        move |args, cx| {
            let id = args.capture_id;
            if let Some(run) = h.run(id)
                && let Some(secs) = args.wait
            {
                // A cancelled wait is only that: the capture goes on.
                follow(&h, &run, Some(seconds(secs, "wait")?), false, cx);
            }
            h.reap_run(id);
            let run = h.run(id);
            let store = h.captures();
            let c = store.get(id).ok_or_else(|| no_capture(id))?;
            let mut v = c.entry();
            v["duration"] = json!(c.time(c.logic_samples()));
            v["trigger"] = json!(c.trigger_at.map(|s| json!({"sample": s, "t": c.time(s)})));
            v["frames"] = json!(c.frames);
            if let Some(run) = run {
                v["handle"] = json!(run.handle);
            }
            Ok(ToolResult::structured(v))
        },
    );

    let h = host.clone();
    let capture_stop = function::<CaptureIdArgs, _>(
        ToolSpec {
            name: "capture_stop",
            title: "Stop a capture",
            description: "Ends a running capture, waits for the device to stop, and returns \
                          the summary of what came; the device is free again. A capture that \
                          has ended already just gives its summary.",
            example: json!({"capture_id": 1}),
            annotations: Annotations::MUTATING,
            tags: &[Tag::Capture],
        },
        move |args, _| {
            let id = args.capture_id;
            h.reap_run(id);
            let run = h.stop_run(id, "capture_stop");
            let mut v = summary(&h, id)?;
            v["was_running"] = json!(run.is_some());
            if let Some(run) = run {
                v["handle"] = json!(run.handle);
            }
            Ok(ToolResult::structured(v))
        },
    );

    let h = host.clone();
    let list_captures = function::<NoArgs, _>(
        ToolSpec {
            name: "list_captures",
            title: "List captures",
            description: "The captures in the store: id, source, channels, sample count, \
                          rate, size and whether it is still running, oldest first, with the \
                          memory used by captures and by analyzer frames, and the budget.",
            example: json!({}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Capture],
        },
        move |_, _| {
            let analyzer_bytes = h.analyzer_bytes();
            let store = h.captures();
            let list: Vec<Value> = store.iter().map(Capture::entry).collect();
            Ok(ToolResult::structured(json!({
                "count": list.len(),
                "running": store.iter().filter(|c| c.running).count(),
                "captures": list,
                "bytes": store.used(),
                "analyzer_bytes": analyzer_bytes,
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
                          analyzers over it; a running one is stopped first.",
            example: json!({"capture_id": 1}),
            annotations: Annotations::MUTATING,
            tags: &[Tag::Capture],
        },
        move |args, _| {
            h.reap_run(args.capture_id);
            let stopped = h.stop_run(args.capture_id, "dropped").is_some();
            h.captures().remove(args.capture_id).ok_or_else(|| no_capture(args.capture_id))?;
            let analyzers = h.remove_analyzers_of(args.capture_id);
            Ok(ToolResult::structured(json!({
                "dropped": args.capture_id,
                "was_running": stopped,
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
                          max, mean and RMS with the unit. Read this before asking for data. \
                          On a running capture, what has come so far.",
            example: json!({"capture_id": 1}),
            annotations: Annotations::READ_ONLY,
            tags: &[Tag::Capture],
        },
        move |args, _| Ok(ToolResult::structured(summary(&h, args.capture_id)?)),
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
            let (id, _) = h.captures().add(capture);
            Ok(ToolResult::structured(summary(&h, id)?))
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
                          bytes. A running capture is written as it stands.",
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

    vec![
        capture,
        capture_start,
        capture_status,
        capture_stop,
        list_captures,
        drop_capture,
        capture_summary,
        load_capture,
        export_capture,
    ]
}

/// "No capture" in one place, so the tools agree.
pub fn no_capture(id: u64) -> ToolError {
    ToolError::new(format!("No capture {id}; `list_captures` has the ids."))
}

/// The summary of capture `id`.
fn summary(host: &Host, id: u64) -> Result<Value, ToolError> {
    let store = host.captures();
    Ok(store.get(id).ok_or_else(|| no_capture(id))?.summary())
}

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Takes the device of `args.handle`, prepares it and starts the
/// acquisition on a thread that feeds a new capture in the store; the
/// run is in the host's table until it is stopped or reaped. With
/// `need_limit`, a missing limit is refused; without one it means
/// continuous sampling. `timeout` stops the run after that long.
fn start_run(
    host: &Arc<Host>,
    args: &CaptureArgs,
    need_limit: bool,
    timeout: Option<Duration>,
) -> Result<Arc<CaptureRun>, ToolError> {
    if let Some(t) = timeout {
        deadline(t)?;
    }
    let open = host.take_device(args.handle)?;
    let Open { device_id, mut device } = open;
    let give_back =
        |device| host.put_device(args.handle, Open { device_id: device_id.clone(), device });
    let (trigger, plan) = match prepare(device.as_mut(), args, need_limit) {
        Ok(p) => p,
        Err(e) => {
            give_back(device);
            return Err(e);
        }
    };
    if let Err(e) = plan.send_enables(device.as_mut()) {
        give_back(device);
        return Err(e.into());
    }
    let samplerate = device.get(&Key::Samplerate, None).ok().and_then(|v| v.as_u64());
    let channels = device.channels().to_vec();
    let info = device.info().clone();
    let source = format!("{} {}", info.driver, info.model).trim().to_owned();
    let mut capture =
        Capture::new(format!("{source} (handle {})", args.handle), channels, samplerate);
    capture.device = Some(info);
    capture.running = true;
    let (id, _) = host.captures().add(capture);
    let run = CaptureRun::new(id, args.handle);
    let mut session = Session::new();
    session.add_device(device);
    session.set_trigger(trigger);
    host.add_run(run.clone());
    let thread = {
        let run = run.clone();
        let store = host.capture_store();
        let host = Arc::downgrade(host);
        let wait_trigger = args.wait_trigger;
        std::thread::Builder::new()
            .name(format!("capture-{id}"))
            .spawn(move || feed_run(&run, &store, &host, device_id, session, wait_trigger, timeout))
    };
    match thread {
        Ok(t) => run.set_thread(t),
        Err(e) => {
            // No thread: the session, and the device in it, are gone
            // with it; the capture stays as the record of the failure.
            run.stop("cannot start the thread");
            run.finish();
            host.reap_run(id);
            if let Some(c) = host.captures().get_mut(id) {
                c.running = false;
                c.failed = Some(format!("cannot start the capture thread: {e}"));
            }
            return Err(ToolError::new(format!("Cannot start the capture thread: {e}.")));
        }
    }
    Ok(run)
}

/// Ends a run on every way out of [`feed_run`], a panic included: the
/// capture is no longer running, the failure is recorded, and the run
/// is finished so `capture`, `capture_status` and `close_all` do not
/// wait for ever. The device cannot be given back after a panic (the
/// session went with it), so its handle is forgotten.
struct RunGuard<'a> {
    run: &'a CaptureRun,
    store: &'a Mutex<CaptureStore>,
    host: &'a Weak<Host>,
    clean: bool,
}

impl Drop for RunGuard<'_> {
    fn drop(&mut self) {
        if !self.clean {
            if let Some(c) = lock(self.store).get_mut(self.run.capture_id) {
                c.running = false;
                c.stopped = self.run.reason();
                c.failed.get_or_insert_with(|| "the capture thread panicked".to_owned());
            }
            if let Some(h) = self.host.upgrade() {
                h.forget_device(self.run.handle);
            }
        }
        self.run.finish();
    }
}

/// The acquisition, on the capture's thread: feeds the capture in the
/// store packet by packet, stops on the run's flag, the timeout, a drop
/// or the memory budget, then marks the capture ended and gives the
/// device back.
fn feed_run(
    run: &Arc<CaptureRun>,
    store: &Mutex<CaptureStore>,
    host: &Weak<Host>,
    device_id: String,
    mut session: Session,
    wait_trigger: bool,
    timeout: Option<Duration>,
) {
    let id = run.capture_id;
    let mut guard = RunGuard { run, store, host, clean: false };
    let session_stop = session.stop_flag();
    // The watchdog relays the run's stop flag to the session and keeps
    // the timeout; it ends with the run, when `done` drops.
    let (done, wake) = mpsc::channel::<()>();
    let watchdog = {
        let run = run.clone();
        // Checked when the run started; a failure here is a stop now.
        let deadline = timeout.map(|t| Instant::now().checked_add(t));
        if deadline == Some(None) {
            run.stop("timeout");
        }
        let deadline = deadline.flatten();
        std::thread::spawn(move || {
            loop {
                if wake.recv_timeout(Duration::from_millis(50))
                    != Err(mpsc::RecvTimeoutError::Timeout)
                {
                    break;
                }
                if deadline.is_some_and(|d| Instant::now() >= d) {
                    run.time_out();
                }
                if run.stopping() {
                    session_stop.store(true, Ordering::SeqCst);
                }
            }
        })
    };
    let mut triggered = false;
    let result = session.run(|_, packet| {
        if run.stopping() {
            return Flow::Stop;
        }
        match packet {
            Packet::Trigger => triggered = true,
            Packet::Logic(_) if wait_trigger && !triggered => return Flow::Continue,
            _ => {}
        }
        let mut store = lock(store);
        let Some(c) = store.get_mut(id) else {
            run.stop("dropped");
            return Flow::Stop;
        };
        c.push(packet);
        if store.over_budget() {
            let evicted = store.make_room(0, id);
            if let Some(c) = store.get_mut(id) {
                c.evicted.extend(evicted);
            }
            if store.over_budget() {
                run.stop("memory budget");
                return Flow::Stop;
            }
        }
        Flow::Continue
    });
    drop(done);
    let _ = watchdog.join();
    let mut device = session.into_devices().remove(0);
    // A run that ended on its limit was never stopped: the device's
    // acquisition thread is joined here, so the next start finds it idle.
    let _ = device.stop();
    if let Some(c) = lock(store).get_mut(id) {
        c.running = false;
        c.stopped = run.reason();
        match result {
            Ok(()) => {}
            Err(crate::core::Error::CaptureFailed(why)) => {
                c.failed.get_or_insert(why);
            }
            Err(e) => {
                c.failed.get_or_insert(e.to_string());
            }
        }
    }
    if let Some(h) = host.upgrade() {
        h.put_device(run.handle, Open { device_id, device });
    } else {
        let mut device = device;
        let _ = device.close();
    }
    guard.clean = true;
}

/// Waits for a run to end, up to `wait` when given, reporting the
/// samples so far as progress. A cancelled call stops the run when
/// `stop_on_cancel` (the blocking `capture`), else just returns (a
/// status wait). Returns whether the run ended.
fn follow(
    host: &Host,
    run: &CaptureRun,
    wait: Option<Duration>,
    stop_on_cancel: bool,
    cx: &Call,
) -> bool {
    let deadline = wait.and_then(|w| Instant::now().checked_add(w));
    let mut last_report = (Instant::now(), 0u64);
    loop {
        if run.is_done() {
            return true;
        }
        if cx.cancelled() {
            if stop_on_cancel {
                run.stop("cancelled");
            } else {
                return false;
            }
        }
        if deadline.is_some_and(|d| Instant::now() >= d) {
            return false;
        }
        std::thread::sleep(Duration::from_millis(10));
        if last_report.0.elapsed() >= Duration::from_millis(200) {
            let samples = host.captures().get(run.capture_id).map_or(0, Capture::logic_samples);
            if samples != last_report.1 {
                cx.progress(samples as f64, None, Some("samples"));
            }
            last_report = (Instant::now(), samples);
        }
    }
}

/// `-c`, the limit and the trigger, in the command line's order, before
/// the outputs are switched on. Without a limit, and `need_limit` off,
/// the device is set up for continuous sampling.
fn prepare(
    device: &mut dyn crate::core::Device,
    args: &CaptureArgs,
    need_limit: bool,
) -> Result<(Option<crate::core::Trigger>, setup::ConfigPlan), ToolError> {
    // `wait_trigger` holds back every sample before the trigger: with
    // no trigger nothing would be kept at all (the CLI refuses `-w`
    // without `-t` the same way).
    if args.wait_trigger && args.triggers.is_none() {
        return Err(ToolError::invalid_args("`wait_trigger` needs `triggers`."));
    }
    let plan = setup::plan_configs(&*device, &args.config, None)?;
    plan.send_settings(device)?;
    let no_limit = args.samples.is_none() && args.time.is_none() && args.frames.is_none();
    if no_limit && need_limit {
        return Err(ToolError::invalid_args(
            "Give a limit: `samples` (`1k`), `time` (`10ms`) or `frames`, or use \
             `capture_start` for an open-ended run.",
        ));
    }
    let limits = CaptureLimits {
        time: args.time.clone(),
        samples: args.samples.clone(),
        frames: args.frames.clone(),
        continuous: no_limit,
    };
    setup::apply_limits(device, &limits)?;
    if no_limit {
        // A limit left from an earlier capture on this handle would end
        // the run. The core refuses 0 on a `config_set`, so this goes to
        // the device directly; one that cannot take it keeps its limit.
        let _ = device.set(&Key::LimitSamples, &CoreValue::U64(0), None);
    }
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
        // The temp dir itself is outside the allowed one under it.
        let outside = std::env::temp_dir().join("oxrok-mcp-outside.sr");
        let e = call(&h, "export_capture", json!({"capture_id": 2, "path": outside})).unwrap_err();
        assert!(e.message.contains("outside"), "{e}");
        let e = call(&h, "capture", json!({"handle": handle, "samples": "10", "timeout": -1.0}))
            .unwrap_err();
        assert!(e.invalid_args, "{e}");
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
        let e =
            call(&h, "capture", json!({"handle": handle, "samples": "10", "wait_trigger": true}))
                .unwrap_err();
        assert!(e.invalid_args && e.message.contains("triggers"), "{e}");
        // Still usable after the refusals.
        let s = call(&h, "capture", json!({"handle": handle, "time": "1ms"})).unwrap();
        assert_eq!(s["samples"], 200);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    fn status_after(h: &Arc<Host>, id: u64, wait: f64) -> Value {
        call(h, "capture_status", json!({"capture_id": id, "wait": wait})).unwrap()
    }

    #[test]
    fn a_started_capture_runs_until_stopped() {
        let dir = temp_dir("start");
        let h = host(&dir);
        let handle = h.open(None, Some("demo:logic_channels=2")).unwrap();
        let s = call(&h, "capture_start", json!({"handle": handle})).unwrap();
        let id = s["capture_id"].as_u64().unwrap();
        assert_eq!(s["running"], true);
        assert_eq!(s["continuous"], true);
        // The device is busy meanwhile.
        assert_eq!(h.open_devices(), [(handle, "demo".to_owned(), true)]);
        let e = call(&h, "capture", json!({"handle": handle, "samples": "10"})).unwrap_err();
        assert!(e.message.contains("capturing"), "{e}");
        // It grows; a wait on a continuous run returns when the time is up.
        let t0 = Instant::now();
        let st = status_after(&h, id, 0.3);
        assert!(t0.elapsed() >= Duration::from_millis(250));
        assert_eq!(st["running"], true);
        assert_eq!(st["handle"], handle);
        let n = st["samples"].as_u64().unwrap();
        assert!(n > 0, "{st}");
        // What came so far can be read.
        let sum = call(&h, "capture_summary", json!({"capture_id": id})).unwrap();
        assert_eq!(sum["running"], true);
        assert!(sum["logic"][0]["transitions"].as_u64().unwrap() > 0, "{sum}");
        let l = call(&h, "list_captures", json!({})).unwrap();
        assert_eq!(l["running"], 1);
        assert_eq!(l["analyzer_bytes"], 0);

        let stopped = call(&h, "capture_stop", json!({"capture_id": id})).unwrap();
        assert_eq!(stopped["was_running"], true);
        assert_eq!(stopped["running"], false);
        assert_eq!(stopped["stopped"], "capture_stop");
        assert_eq!(stopped["failed"], Value::Null);
        assert!(stopped["samples"].as_u64().unwrap() >= n);
        // The device is back; a second stop is a plain summary.
        assert_eq!(h.open_devices(), [(handle, "demo".to_owned(), false)]);
        let again = call(&h, "capture_stop", json!({"capture_id": id})).unwrap();
        assert_eq!(again["was_running"], false);
        let st = call(&h, "capture_status", json!({"capture_id": id})).unwrap();
        assert_eq!(st["running"], false);
        assert_eq!(st["handle"], Value::Null);

        // With a limit, the run ends by itself and a wait sees it end.
        let s = call(&h, "capture_start", json!({"handle": handle, "samples": "100"})).unwrap();
        assert_eq!(s["continuous"], false);
        let id = s["capture_id"].as_u64().unwrap();
        let st = status_after(&h, id, 5.0);
        assert_eq!(st["running"], false);
        assert_eq!(st["samples"], 100);
        assert_eq!(st["stopped"], Value::Null);
        // A timeout stops a run and says so.
        let s = call(&h, "capture_start", json!({"handle": handle, "timeout": 0.2})).unwrap();
        let id = s["capture_id"].as_u64().unwrap();
        let st = status_after(&h, id, 5.0);
        assert_eq!(st["running"], false);
        assert_eq!(st["stopped"], "timeout", "{st}");
        // Dropping a running capture stops it and frees the device.
        let s = call(&h, "capture_start", json!({"handle": handle})).unwrap();
        let id = s["capture_id"].as_u64().unwrap();
        let d = call(&h, "drop_capture", json!({"capture_id": id})).unwrap();
        assert_eq!(d["was_running"], true);
        assert_eq!(h.open_devices(), [(handle, "demo".to_owned(), false)]);
        assert!(call(&h, "capture_status", json!({"capture_id": id})).is_err());
        h.close(handle).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn the_memory_budget_stops_a_run_and_keeps_older_captures_out() {
        let dir = temp_dir("budget");
        let mut r = crate::core::Registry::new();
        r.register(Box::new(crate::drivers::demo::DemoDriver));
        let h = Arc::new(Host::with(r, crate::decode::Registry::new()).with_limits(Limits {
            allow_paths: vec![dir.clone()],
            max_memory: 64 << 10,
            ..Limits::default()
        }));
        let handle = h.open(None, Some("demo:logic_channels=2")).unwrap();
        let first = call(&h, "capture", json!({"handle": handle, "samples": "100"})).unwrap();
        let first_id = first["capture_id"].as_u64().unwrap();
        let s = call(&h, "capture_start", json!({"handle": handle})).unwrap();
        let id = s["capture_id"].as_u64().unwrap();
        let st = status_after(&h, id, 10.0);
        assert_eq!(st["running"], false, "{st}");
        assert_eq!(st["stopped"], "memory budget", "{st}");
        assert!(st["bytes"].as_u64().unwrap() >= 64 << 10, "{st}");
        // The finished capture went first.
        assert_eq!(st["evicted"], json!([first_id]));
        let l = call(&h, "list_captures", json!({})).unwrap();
        assert_eq!(l["count"], 1);
        assert_eq!(h.open_devices(), [(handle, "demo".to_owned(), false)]);
        h.close(handle).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn close_all_ends_a_run() {
        let dir = temp_dir("close-all");
        let h = host(&dir);
        let handle = h.open(None, Some("demo:logic_channels=2")).unwrap();
        let s = call(&h, "capture_start", json!({"handle": handle})).unwrap();
        let id = s["capture_id"].as_u64().unwrap();
        h.close_all();
        assert_eq!(h.open_devices().len(), 0);
        let st = call(&h, "capture_status", json!({"capture_id": id})).unwrap();
        assert_eq!(st["running"], false);
        assert_eq!(st["stopped"], "server closing");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
