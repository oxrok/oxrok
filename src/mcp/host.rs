// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The host: what the tools work on. The driver and decoder registries,
//! the limits, the devices found and opened, and the captures kept.
//!
//! Open devices live here by handle. A capture takes the device out
//! ([`Host::take_device`]) and puts it back after; a call on a handle
//! whose device is out gets "busy". Dropping the host, or
//! [`Host::close_all`], closes every device: the server does both on
//! EOF and on Ctrl-C, so a device is never left open.

use super::capture::CaptureStore;
use super::tool::ToolError;
use crate::core::{Channel, Device, DeviceClass, DeviceInfo, Registry};
use crate::decode;
use crate::decode::Analyzer;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::Duration;

/// Bounds the tools keep.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Limits {
    /// How long the `cli` tool waits for a command before killing it.
    pub cli_timeout: Duration,
    /// How much of each of a command's stdout and stderr the `cli` tool
    /// keeps.
    pub cli_max_bytes: usize,
    /// How long a synchronous `capture` may run before it is stopped.
    pub capture_timeout: Duration,
    /// The bytes the capture store may hold before the oldest go.
    pub max_memory: usize,
    /// The directories files may be read from and written to.
    pub allow_paths: Vec<PathBuf>,
    /// The drivers that may be scanned and opened; every one when `None`.
    pub drivers: Option<Vec<String>>,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            cli_timeout: Duration::from_secs(60),
            cli_max_bytes: 1 << 20,
            capture_timeout: Duration::from_secs(60),
            max_memory: 512 << 20,
            allow_paths: std::env::current_dir().into_iter().collect(),
            drivers: None,
        }
    }
}

/// A device a scan found, by its id, until it is opened.
struct Scanned {
    device_id: String,
    device: Option<Box<dyn Device>>,
}

/// An open device and its id.
pub struct Open {
    /// `driver:serial` or `driver:conn`, the id a scan gave it.
    pub device_id: String,
    /// The device.
    pub device: Box<dyn Device>,
}

enum Slot {
    Idle(Open),
    /// Out on a capture; the id stays for the listing.
    Busy(String),
}

impl Slot {
    fn device_id(&self) -> &str {
        match self {
            Slot::Idle(o) => &o.device_id,
            Slot::Busy(id) => id,
        }
    }
}

#[derive(Default)]
struct Devices {
    scanned: Vec<Scanned>,
    open: HashMap<u64, Slot>,
    next_handle: u64,
}

/// What a scan reports of one device.
#[derive(Debug)]
pub struct ScanEntry {
    /// The id to `open` it by.
    pub device_id: String,
    /// Identity.
    pub info: DeviceInfo,
    /// Its classes.
    pub classes: Vec<DeviceClass>,
    /// Its channels.
    pub channels: Vec<Channel>,
    /// The handle, when it is open already.
    pub handle: Option<u64>,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// An analyzer the host keeps: a decoder stack over one capture.
#[derive(Clone)]
pub struct AnalyzerEntry {
    /// Its id.
    pub id: u64,
    /// The capture it decodes.
    pub capture_id: u64,
    /// A name the client gave it.
    pub name: Option<String>,
    /// The stack in `-P` syntax.
    pub spec: String,
    /// Goes up when the stack is rebuilt; a client paging with an older
    /// one is told its offsets are stale.
    pub generation: u64,
    /// The run and its frames.
    pub analyzer: Arc<Analyzer>,
}

#[derive(Default)]
struct Analyzers {
    next_id: u64,
    list: Vec<AnalyzerEntry>,
}

/// A capture in progress: its stop flag, why it was stopped, and its
/// thread, kept by capture id so `capture_stop`, `drop_capture` and
/// [`Host::close_all`] can end it. The thread feeds the capture in the
/// store and gives the device back when it ends.
pub struct CaptureRun {
    /// The capture being fed.
    pub capture_id: u64,
    /// The device's handle.
    pub handle: u64,
    stop: AtomicBool,
    done: AtomicBool,
    timed_out: AtomicBool,
    reason: Mutex<Option<String>>,
    thread: Mutex<Option<JoinHandle<()>>>,
}

impl CaptureRun {
    /// A run that has not started.
    pub fn new(capture_id: u64, handle: u64) -> Arc<Self> {
        Arc::new(Self {
            capture_id,
            handle,
            stop: AtomicBool::new(false),
            done: AtomicBool::new(false),
            timed_out: AtomicBool::new(false),
            reason: Mutex::new(None),
            thread: Mutex::new(None),
        })
    }

    /// Asks the run to end, for `reason`; the first reason stands.
    pub fn stop(&self, reason: &str) {
        lock(&self.reason).get_or_insert_with(|| reason.to_owned());
        self.stop.store(true, Ordering::SeqCst);
    }

    /// Whether a stop was asked for.
    pub fn stopping(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    /// Why it was stopped, once it was.
    pub fn reason(&self) -> Option<String> {
        lock(&self.reason).clone()
    }

    /// Stops for the timeout.
    pub fn time_out(&self) {
        self.timed_out.store(true, Ordering::SeqCst);
        self.stop("timeout");
    }

    /// Whether the timeout ended it.
    pub fn timed_out(&self) -> bool {
        self.timed_out.load(Ordering::SeqCst)
    }

    /// Keeps the thread, to join.
    pub fn set_thread(&self, thread: JoinHandle<()>) {
        *lock(&self.thread) = Some(thread);
    }

    /// The thread's last word: the capture is fed and the device back.
    pub fn finish(&self) {
        self.done.store(true, Ordering::SeqCst);
    }

    /// Whether the thread has finished.
    pub fn is_done(&self) -> bool {
        self.done.load(Ordering::SeqCst)
    }

    /// Waits for the thread, once.
    pub fn join(&self) {
        if let Some(h) = lock(&self.thread).take() {
            let _ = h.join();
        }
    }
}

/// What the tools work on.
pub struct Host {
    registry: Registry,
    decoders: decode::Registry,
    limits: Limits,
    cli_program: Option<PathBuf>,
    devices: Mutex<Devices>,
    captures: Arc<Mutex<CaptureStore>>,
    analyzers: Mutex<Analyzers>,
    runs: Mutex<HashMap<u64, Arc<CaptureRun>>>,
}

impl Default for Host {
    fn default() -> Self {
        Self::new()
    }
}

impl Host {
    /// Every driver and decoder this build has, default limits, and the
    /// running executable as the `cli` program.
    pub fn new() -> Self {
        let mut registry = Registry::new();
        crate::register_all(&mut registry);
        Self::with(registry, crate::decoder_registry())
    }

    /// A host over the given registries.
    pub fn with(registry: Registry, decoders: decode::Registry) -> Self {
        let limits = Limits::default();
        Self {
            registry,
            decoders,
            captures: Arc::new(Mutex::new(CaptureStore::new(limits.max_memory))),
            limits,
            cli_program: std::env::current_exe().ok(),
            devices: Mutex::new(Devices::default()),
            analyzers: Mutex::new(Analyzers::default()),
            runs: Mutex::new(HashMap::new()),
        }
    }

    /// Sets the limits.
    #[must_use]
    pub fn with_limits(mut self, limits: Limits) -> Self {
        self.captures = Arc::new(Mutex::new(CaptureStore::new(limits.max_memory)));
        self.limits = limits;
        self
    }

    /// Sets the program the `cli` tool runs, for a host inside another
    /// executable.
    #[must_use]
    pub fn with_cli_program(mut self, program: PathBuf) -> Self {
        self.cli_program = Some(program);
        self
    }

    /// The drivers.
    pub fn registry(&self) -> &Registry {
        &self.registry
    }

    /// The decoders.
    pub fn decoders(&self) -> &decode::Registry {
        &self.decoders
    }

    /// The limits.
    pub fn limits(&self) -> &Limits {
        &self.limits
    }

    /// The program the `cli` tool runs, when one is known.
    pub fn cli_program(&self) -> Option<&Path> {
        self.cli_program.as_deref()
    }

    /// The captures.
    pub fn captures(&self) -> MutexGuard<'_, CaptureStore> {
        lock(&self.captures)
    }

    /// The store itself, for a thread that feeds or follows a capture
    /// without holding the host.
    pub fn capture_store(&self) -> Arc<Mutex<CaptureStore>> {
        self.captures.clone()
    }

    /// The bytes the analyzers' frames hold.
    pub fn analyzer_bytes(&self) -> usize {
        lock(&self.analyzers).list.iter().map(|a| a.analyzer.memory_bytes()).sum()
    }

    /// What is left of the memory budget for a new analyzer's frames.
    pub fn memory_left(&self) -> usize {
        self.limits.max_memory.saturating_sub(self.captures().used() + self.analyzer_bytes())
    }

    /// Keeps a capture run while its thread goes.
    pub fn add_run(&self, run: Arc<CaptureRun>) {
        lock(&self.runs).insert(run.capture_id, run);
    }

    /// The run feeding capture `capture_id`, while one does.
    pub fn run(&self, capture_id: u64) -> Option<Arc<CaptureRun>> {
        lock(&self.runs).get(&capture_id).cloned()
    }

    /// Stops the run of `capture_id` for `reason`, waits for its thread
    /// and forgets it; `None` when no run is going.
    pub fn stop_run(&self, capture_id: u64, reason: &str) -> Option<Arc<CaptureRun>> {
        let run = lock(&self.runs).remove(&capture_id)?;
        run.stop(reason);
        run.join();
        Some(run)
    }

    /// Forgets a run whose thread has finished, joining it; `None` when
    /// none has, or it is still going.
    pub fn reap_run(&self, capture_id: u64) -> Option<Arc<CaptureRun>> {
        let mut runs = lock(&self.runs);
        if !runs.get(&capture_id)?.is_done() {
            return None;
        }
        let run = runs.remove(&capture_id)?;
        drop(runs);
        run.join();
        Some(run)
    }

    /// Stops every run, waiting for the threads.
    pub fn stop_all_runs(&self, reason: &str) {
        let runs: Vec<Arc<CaptureRun>> = lock(&self.runs).drain().map(|(_, r)| r).collect();
        for run in runs {
            run.stop(reason);
            run.join();
        }
    }

    /// Keeps a started analyzer; returns its id.
    pub fn add_analyzer(
        &self,
        capture_id: u64,
        name: Option<String>,
        spec: String,
        analyzer: Analyzer,
    ) -> u64 {
        let mut table = lock(&self.analyzers);
        table.next_id += 1;
        let id = table.next_id;
        table.list.push(AnalyzerEntry {
            id,
            capture_id,
            name,
            spec,
            generation: 1,
            analyzer: Arc::new(analyzer),
        });
        id
    }

    /// The analyzer with `id`.
    pub fn analyzer(&self, id: u64) -> Option<AnalyzerEntry> {
        lock(&self.analyzers).list.iter().find(|a| a.id == id).cloned()
    }

    /// Every analyzer, oldest first.
    pub fn analyzers(&self) -> Vec<AnalyzerEntry> {
        lock(&self.analyzers).list.clone()
    }

    /// Replaces an analyzer's run, keeping its id and name; the
    /// generation goes up. The old run is stopped once the last client
    /// holding it lets go.
    pub fn replace_analyzer(
        &self,
        id: u64,
        spec: String,
        analyzer: Analyzer,
    ) -> Option<AnalyzerEntry> {
        let mut table = lock(&self.analyzers);
        let entry = table.list.iter_mut().find(|a| a.id == id)?;
        entry.spec = spec;
        entry.generation += 1;
        entry.analyzer = Arc::new(analyzer);
        Some(entry.clone())
    }

    /// Removes an analyzer; its thread is joined when the entry drops.
    pub fn remove_analyzer(&self, id: u64) -> Option<AnalyzerEntry> {
        let mut table = lock(&self.analyzers);
        let pos = table.list.iter().position(|a| a.id == id)?;
        Some(table.list.remove(pos))
    }

    /// Removes the analyzers of a capture; returns their ids.
    pub fn remove_analyzers_of(&self, capture_id: u64) -> Vec<u64> {
        let mut table = lock(&self.analyzers);
        let (gone, kept): (Vec<_>, Vec<_>) =
            table.list.drain(..).partition(|a| a.capture_id == capture_id);
        table.list = kept;
        gone.into_iter().map(|a| a.id).collect()
    }

    /// Whether `--drivers` lets this driver be scanned and opened.
    pub fn driver_allowed(&self, id: &str) -> bool {
        self.limits.drivers.as_ref().is_none_or(|list| list.iter().any(|d| d == id))
    }

    /// `path` as an absolute path under one of the allowed directories,
    /// or a refusal naming them. The file need not exist; its directory
    /// must.
    pub fn check_path(&self, path: &str) -> Result<PathBuf, ToolError> {
        let given = Path::new(path);
        let absolute = if given.is_absolute() {
            given.to_path_buf()
        } else {
            std::env::current_dir().map_err(ToolError::from)?.join(given)
        };
        let dir = absolute.parent().unwrap_or(&absolute);
        let dir = dir.canonicalize().map_err(|e| {
            ToolError::new(format!("Cannot use '{path}': {e} ({}).", dir.display()))
        })?;
        let allowed = self
            .limits
            .allow_paths
            .iter()
            .any(|a| a.canonicalize().is_ok_and(|a| dir.starts_with(&a)));
        if !allowed {
            let dirs: Vec<String> =
                self.limits.allow_paths.iter().map(|p| p.display().to_string()).collect();
            return Err(ToolError::new(format!(
                "'{path}' is outside the allowed directories ({}).",
                dirs.join(", ")
            )));
        }
        Ok(dir.join(absolute.file_name().unwrap_or_default()))
    }

    /// Scans with a `-d` spec, or every allowed driver, and remembers
    /// what was found so `open` can take a device by its id.
    pub fn scan(&self, driver: Option<&str>) -> Result<Vec<ScanEntry>, ToolError> {
        let found: Vec<Box<dyn Device>> = if let Some(spec) = driver {
            let (d, options) = crate::setup::driver_spec(&self.registry, spec)?;
            if !self.driver_allowed(d.id()) {
                return Err(ToolError::new(format!(
                    "Driver '{}' is not allowed on this server.",
                    d.id()
                )));
            }
            d.scan(&options)?
        } else {
            let mut all = Vec::new();
            for d in self.registry.drivers().filter(|d| self.driver_allowed(d.id())) {
                match d.scan(&[]) {
                    Ok(devs) => all.extend(devs),
                    Err(e) => log::warn!("{}: scan failed: {e}", d.id()),
                }
            }
            all
        };
        let mut devices = lock(&self.devices);
        let open_ids: HashMap<String, u64> =
            devices.open.iter().map(|(h, s)| (s.device_id().to_owned(), *h)).collect();
        let mut entries: Vec<ScanEntry> = Vec::new();
        let mut scanned = Vec::new();
        for device in found {
            let mut device_id = device_id(device.info());
            let n = entries.iter().filter(|e| e.device_id == device_id).count();
            if n > 0 {
                device_id = format!("{device_id}#{}", n + 1);
            }
            let handle = open_ids.get(&device_id).copied();
            entries.push(ScanEntry {
                device_id: device_id.clone(),
                info: device.info().clone(),
                classes: device.classes().to_vec(),
                channels: device.channels().to_vec(),
                handle,
            });
            // A device that is open stays the open one.
            let device = if handle.is_some() { None } else { Some(device) };
            scanned.push(Scanned { device_id, device });
        }
        devices.scanned = scanned;
        Ok(entries)
    }

    /// Opens a device: one of the last scan by `device_id`, or the one
    /// device a scan with `driver` finds now. Returns its handle.
    pub fn open(&self, device_id: Option<&str>, driver: Option<&str>) -> Result<u64, ToolError> {
        let id = match (device_id, driver) {
            (Some(id), _) => id.to_owned(),
            (None, Some(spec)) => {
                let mut ids: Vec<String> =
                    self.scan(Some(spec))?.into_iter().map(|e| e.device_id).collect();
                if ids.len() != 1 {
                    return Err(ToolError::new(format!(
                        "'{spec}' finds {} devices; `open` one by `device_id`.",
                        ids.len()
                    ))
                    .with_data(serde_json::json!({"device_ids": ids})));
                }
                ids.remove(0)
            }
            (None, None) => {
                return Err(ToolError::invalid_args("Give `device_id` (from `scan`) or `driver`."));
            }
        };
        let mut device = {
            let mut devices = lock(&self.devices);
            let handle = devices.open.iter().find_map(|(h, s)| (s.device_id() == id).then_some(*h));
            if let Some(handle) = handle {
                return Err(ToolError::new(format!(
                    "Device '{id}' is open already, as handle {handle}."
                ))
                .with_data(serde_json::json!({"handle": handle})));
            }
            let Some(slot) = devices.scanned.iter_mut().find(|s| s.device_id == id) else {
                return Err(ToolError::new(format!(
                    "No device '{id}' in the last scan; call `scan` first."
                )));
            };
            slot.device.take().ok_or_else(|| ToolError::new(format!("Device '{id}' is gone.")))?
        };
        if let Err(e) = crate::setup::open_device(device.as_mut()) {
            // Back into the scan list, so a retry can find it.
            let mut devices = lock(&self.devices);
            if let Some(slot) = devices.scanned.iter_mut().find(|s| s.device_id == id) {
                slot.device = Some(device);
            }
            return Err(e.into());
        }
        let mut devices = lock(&self.devices);
        devices.next_handle += 1;
        let handle = devices.next_handle;
        devices.open.insert(handle, Slot::Idle(Open { device_id: id, device }));
        Ok(handle)
    }

    /// Closes and forgets a device.
    pub fn close(&self, handle: u64) -> Result<(), ToolError> {
        let mut devices = lock(&self.devices);
        match devices.open.get(&handle) {
            Some(Slot::Idle(_)) => {}
            Some(Slot::Busy(_)) => return Err(busy(handle)),
            None => return Err(no_handle(handle)),
        }
        let Some(Slot::Idle(mut open)) = devices.open.remove(&handle) else { return Ok(()) };
        drop(devices);
        open.device.close()?;
        Ok(())
    }

    /// Runs `f` on an open device.
    pub fn with_device<T>(
        &self,
        handle: u64,
        f: impl FnOnce(&str, &mut dyn Device) -> Result<T, ToolError>,
    ) -> Result<T, ToolError> {
        let mut devices = lock(&self.devices);
        match devices.open.get_mut(&handle) {
            Some(Slot::Idle(open)) => f(&open.device_id, open.device.as_mut()),
            Some(Slot::Busy(_)) => Err(busy(handle)),
            None => Err(no_handle(handle)),
        }
    }

    /// Takes a device out for a capture; [`Host::put_device`] returns it.
    pub fn take_device(&self, handle: u64) -> Result<Open, ToolError> {
        let mut devices = lock(&self.devices);
        match devices.open.get(&handle) {
            Some(Slot::Idle(_)) => {}
            Some(Slot::Busy(_)) => return Err(busy(handle)),
            None => return Err(no_handle(handle)),
        }
        let Some(Slot::Idle(open)) = devices.open.remove(&handle) else {
            return Err(no_handle(handle));
        };
        devices.open.insert(handle, Slot::Busy(open.device_id.clone()));
        Ok(open)
    }

    /// Returns a device taken with [`Host::take_device`]. A device whose
    /// handle was closed meanwhile is closed now.
    pub fn put_device(&self, handle: u64, mut open: Open) {
        let mut devices = lock(&self.devices);
        if matches!(devices.open.get(&handle), Some(Slot::Busy(_))) {
            devices.open.insert(handle, Slot::Idle(open));
        } else {
            drop(devices);
            let _ = open.device.close();
        }
    }

    /// The open devices: handle, id and whether a capture has it.
    pub fn open_devices(&self) -> Vec<(u64, String, bool)> {
        let devices = lock(&self.devices);
        let mut list: Vec<(u64, String, bool)> = devices
            .open
            .iter()
            .map(|(h, s)| (*h, s.device_id().to_owned(), matches!(s, Slot::Busy(_))))
            .collect();
        list.sort_by_key(|(h, _, _)| *h);
        list
    }

    /// The driver ids of the open devices, for the `cli` tool's refusal
    /// to run `-d` on one of them.
    pub fn open_drivers(&self) -> Vec<String> {
        self.open_devices()
            .into_iter()
            .map(|(_, id, _)| id.split(':').next().unwrap_or_default().to_owned())
            .collect()
    }

    /// Stops every running capture, closes every open device and
    /// forgets the scan.
    pub fn close_all(&self) {
        self.stop_all_runs("server closing");
        let mut devices = lock(&self.devices);
        devices.scanned.clear();
        for (_, slot) in devices.open.drain() {
            if let Slot::Idle(mut open) = slot {
                let _ = open.device.close();
            }
        }
    }
}

impl Drop for Host {
    fn drop(&mut self) {
        self.close_all();
    }
}

/// `driver:serial`, else `driver:conn`, else the driver alone.
pub fn device_id(info: &DeviceInfo) -> String {
    if !info.serial.is_empty() {
        format!("{}:{}", info.driver, info.serial)
    } else if let Some(conn) = &info.conn {
        format!("{}:{conn}", info.driver)
    } else {
        info.driver.to_owned()
    }
}

fn busy(handle: u64) -> ToolError {
    ToolError::new(format!("Device {handle} is capturing; wait for the capture to end."))
}

fn no_handle(handle: u64) -> ToolError {
    ToolError::new(format!("No open device with handle {handle}; `scan` and `open` first."))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Key;

    fn host() -> Host {
        let mut r = Registry::new();
        r.register(Box::new(crate::drivers::demo::DemoDriver));
        Host::with(r, decode::Registry::new())
    }

    #[test]
    fn scans_opens_and_closes() {
        let h = host();
        let found = h.scan(Some("demo:logic_channels=2:analog_channels=0")).unwrap();
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].device_id, "demo");
        assert_eq!(found[0].channels.len(), 2);
        assert_eq!(found[0].handle, None);
        let handle = h.open(Some("demo"), None).unwrap();
        assert_eq!(handle, 1);
        let e = h.open(Some("demo"), None).unwrap_err();
        assert!(e.message.contains("open already"), "{e}");
        // A scan again sees it open and keeps the open one.
        let found = h.scan(Some("demo:logic_channels=2:analog_channels=0")).unwrap();
        assert_eq!(found[0].handle, Some(1));
        let rate = h
            .with_device(handle, |id, d| {
                assert_eq!(id, "demo");
                Ok(d.get(&Key::Samplerate, None)?.as_u64())
            })
            .unwrap();
        assert_eq!(rate, Some(200_000));
        assert_eq!(h.open_drivers(), ["demo"]);
        let open = h.take_device(handle).unwrap();
        assert!(h.with_device(handle, |_, _| Ok(())).unwrap_err().message.contains("capturing"));
        assert!(h.close(handle).unwrap_err().message.contains("capturing"));
        assert_eq!(h.open_devices(), [(1, "demo".to_owned(), true)]);
        h.put_device(handle, open);
        h.close(handle).unwrap();
        assert!(h.close(handle).unwrap_err().message.contains("No open device"));
        assert_eq!(h.open_devices().len(), 0);
    }

    #[test]
    fn opens_by_driver_spec_and_refuses_disallowed() {
        let h = host();
        let handle = h.open(None, Some("demo:logic_channels=3")).unwrap();
        assert_eq!(h.open_devices(), [(1, "demo".to_owned(), false)]);
        h.close(handle).unwrap();
        let e = h.open(Some("nope"), None).unwrap_err();
        assert!(e.message.contains("last scan"), "{e}");
        let e = h.open(None, Some("nope")).unwrap_err();
        assert_eq!(e.message, "Driver nope not found.");
        let e = h.open(None, None).unwrap_err();
        assert!(e.invalid_args);
        let h =
            h.with_limits(Limits { drivers: Some(vec!["fx2lafw".into()]), ..Limits::default() });
        let e = h.scan(Some("demo")).unwrap_err();
        assert!(e.message.contains("not allowed"), "{e}");
        assert!(h.scan(None).is_ok_and(|found| found.is_empty()));
    }

    #[test]
    fn checks_paths() {
        let dir = std::env::temp_dir().join(format!("oxrok-mcp-paths-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let h = host().with_limits(Limits { allow_paths: vec![dir.clone()], ..Limits::default() });
        let ok = h.check_path(&dir.join("x.sr").display().to_string()).unwrap();
        assert!(ok.ends_with("x.sr"));
        let e = h.check_path("/etc/passwd").unwrap_err();
        assert!(e.message.contains("outside"), "{e}");
        let e = h.check_path(&dir.join("missing/x.sr").display().to_string()).unwrap_err();
        assert!(e.message.contains("Cannot use"), "{e}");
        std::fs::remove_dir_all(&dir).unwrap();
        let info = |serial: &str, conn: Option<&str>| DeviceInfo {
            driver: "fx2lafw",
            serial: serial.into(),
            conn: conn.map(str::to_owned),
            ..DeviceInfo::default()
        };
        assert_eq!(device_id(&info("", Some("1.4"))), "fx2lafw:1.4");
        assert_eq!(device_id(&info("S1", Some("1.4"))), "fx2lafw:S1");
        assert_eq!(device_id(&info("", None)), "fx2lafw");
    }
}
