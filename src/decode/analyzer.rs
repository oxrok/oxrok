// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! An analyzer: a decoder stack run over a capture's logic data on its
//! own thread, its output kept in a [`FrameStore`] per instance, so the
//! frames can be paged, searched and counted after the decode.

use super::frames::{Frame, FrameStore, MetaItem, RawBinary};
use super::info::DecoderInfo;
use super::radix::{Radix, RadixSupport, all_rows, binary_target, class_rows, is_error_class};
use super::{Item, Output};
use crate::setup::StackPlan;
use std::collections::HashMap;
use std::fmt::Write as _;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// Where an analyzer is.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Status {
    /// Decoding.
    Running,
    /// Every instance ended well.
    Complete,
    /// An instance failed, with its words; the frames before it stand.
    Error(String),
}

impl Status {
    /// The id on the wire.
    pub fn id(&self) -> &'static str {
        match self {
            Status::Running => "running",
            Status::Complete => "complete",
            Status::Error(_) => "error",
        }
    }
}

/// One instance of the stack, as the frames name it.
#[derive(Clone, Debug)]
pub struct InstanceInfo {
    /// The instance id, `i2c-1`.
    pub id: String,
    /// The decoder.
    pub info: &'static DecoderInfo,
    /// Each class's row.
    pub class_rows: Vec<usize>,
    /// Every row: id and description, real rows then synthetic ones.
    pub rows: Vec<(&'static str, &'static str)>,
    /// How the decoder supports a radix.
    pub radix_support: RadixSupport,
}

impl InstanceInfo {
    fn new(id: String, info: &'static DecoderInfo, prefer_raw: bool) -> Self {
        Self {
            id,
            info,
            class_rows: class_rows(info),
            rows: all_rows(info),
            radix_support: RadixSupport::of(info, prefer_raw),
        }
    }

    /// The row called `id`.
    pub fn row_index(&self, id: &str) -> Option<usize> {
        self.rows.iter().position(|(r, _)| *r == id)
    }

    /// The class called `id`.
    pub fn class_index(&self, id: &str) -> Option<usize> {
        self.info.annotations.iter().position(|a| a.id == id)
    }

    /// The classes a frame of this instance may take a raw value for.
    fn value_classes(&self) -> Vec<usize> {
        (0..self.info.binary.len()).filter_map(|b| binary_target(self.info, b)).collect()
    }
}

struct Shared {
    stores: Vec<Mutex<FrameStore>>,
    status: Mutex<Status>,
    fed: AtomicU64,
    stop: AtomicBool,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// A decoder stack over a capture, running or done.
pub struct Analyzer {
    shared: Arc<Shared>,
    thread: Mutex<Option<JoinHandle<()>>>,
    instances: Vec<InstanceInfo>,
    radix: Radix,
    total: u64,
    samplerate: u64,
}

/// The logic data in `chunk_samples` pieces, so outputs are drained
/// while decoding.
const CHUNK_SAMPLES: usize = 65_536;

impl Analyzer {
    /// Starts `plan` over `logic` (`unit_size` bytes per sample, at
    /// `samplerate`, 0 when unknown). `radix` is the analyzer's own;
    /// `prefer_raw` is decision 11 of the plan: a decoder with both a
    /// `format` option and binary output renders through its raw values.
    pub fn start(
        plan: StackPlan,
        radix: Radix,
        prefer_raw: bool,
        logic: Vec<u8>,
        unit_size: usize,
        samplerate: u64,
    ) -> Self {
        let instances: Vec<InstanceInfo> = plan
            .instances
            .iter()
            .map(|i| InstanceInfo::new(i.id.clone(), i.info, prefer_raw))
            .collect();
        let stores = instances
            .iter()
            .map(|i| Mutex::new(FrameStore::with_value_classes(i.value_classes())))
            .collect();
        let shared = Arc::new(Shared {
            stores,
            status: Mutex::new(Status::Running),
            fed: AtomicU64::new(0),
            stop: AtomicBool::new(false),
        });
        let unit = unit_size.max(1);
        let total = (logic.len() / unit) as u64;
        let runner = {
            let shared = shared.clone();
            let instances = instances.clone();
            std::thread::Builder::new()
                .name("analyzer".to_owned())
                .spawn(move || run(&shared, &instances, plan, &logic, unit, samplerate))
        };
        let thread = match runner {
            Ok(h) => Some(h),
            Err(e) => {
                *lock(&shared.status) = Status::Error(format!("cannot start the analyzer: {e}"));
                None
            }
        };
        Self { shared, thread: Mutex::new(thread), instances, radix, total, samplerate }
    }

    /// The instances, bottom first per chain.
    pub fn instances(&self) -> &[InstanceInfo] {
        &self.instances
    }

    /// The instance called `id`, with its index.
    pub fn instance(&self, id: &str) -> Option<(usize, &InstanceInfo)> {
        self.instances.iter().enumerate().find(|(_, i)| i.id == id)
    }

    /// The frames of instance `index`.
    pub fn store(&self, index: usize) -> MutexGuard<'_, FrameStore> {
        lock(&self.shared.stores[index])
    }

    /// Where it is.
    pub fn status(&self) -> Status {
        lock(&self.shared.status).clone()
    }

    /// Samples fed and samples in all.
    pub fn progress(&self) -> (u64, u64) {
        (self.shared.fed.load(Ordering::Relaxed), self.total)
    }

    /// The analyzer's own radix.
    pub fn radix(&self) -> Radix {
        self.radix
    }

    /// The capture's rate, 0 when unknown.
    pub fn samplerate(&self) -> u64 {
        self.samplerate
    }

    /// Frames over every instance.
    pub fn frames(&self) -> u64 {
        (0..self.instances.len()).map(|i| self.store(i).len() as u64).sum()
    }

    /// The stores' heap bytes.
    pub fn memory_bytes(&self) -> usize {
        (0..self.instances.len()).map(|i| self.store(i).memory_bytes()).sum()
    }

    /// Whether the run ended, well or not.
    pub fn is_done(&self) -> bool {
        self.status() != Status::Running
    }

    /// Waits for the run to end, until `deadline` or until `go_on` says
    /// no, and returns the status then.
    pub fn wait_until(&self, deadline: Instant, go_on: impl Fn() -> bool) -> Status {
        while !self.is_done() && Instant::now() < deadline && go_on() {
            std::thread::sleep(Duration::from_millis(10));
        }
        self.status()
    }

    /// Asks the run to end and waits for its thread.
    pub fn stop(&self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(h) = lock(&self.thread).take() {
            let _ = h.join();
        }
    }
}

impl Drop for Analyzer {
    fn drop(&mut self) {
        self.stop();
    }
}

/// The decode, on the analyzer's thread.
fn run(
    shared: &Shared,
    instances: &[InstanceInfo],
    plan: StackPlan,
    logic: &[u8],
    unit: usize,
    samplerate: u64,
) {
    let by_id: HashMap<&str, usize> =
        instances.iter().enumerate().map(|(i, inst)| (inst.id.as_str(), i)).collect();
    let mut stack = match plan.build(unit, samplerate) {
        Ok(s) => s,
        Err(e) => {
            *lock(&shared.status) = Status::Error(e.to_string());
            return;
        }
    };
    let Some(rx) = stack.outputs() else {
        *lock(&shared.status) = Status::Error("the stack has no output".to_owned());
        return;
    };
    let mut errors: Vec<String> = Vec::new();
    let mut handle = |o: Output| match o {
        Output::Annotation { instance, ss, es, class, texts } => {
            if let Some(&i) = by_id.get(instance.as_str()) {
                let inst = &instances[i];
                let row = inst.class_rows.get(class).copied().unwrap_or(class);
                let frame = Frame {
                    start: ss,
                    end: es,
                    class,
                    row,
                    texts: texts.into(),
                    value: None,
                    error: is_error_class(inst.info, class),
                };
                lock(&shared.stores[i]).push(frame);
            }
        }
        Output::Binary { instance, ss, es, class, data } => {
            if let Some(&i) = by_id.get(instance.as_str()) {
                let target = binary_target(instances[i].info, class);
                let item = RawBinary { start: ss, end: es, class, bytes: data };
                lock(&shared.stores[i]).push_binary(item, target);
            }
        }
        Output::Meta { instance, ss, es, name, item } => {
            if let Some(&i) = by_id.get(instance.as_str()) {
                let meta = MetaItem { start: ss, end: es, name, text: item_text(&item) };
                lock(&shared.stores[i]).push_meta(meta);
            }
        }
        Output::Python { .. } => {}
        Output::End { instance, error } => {
            if let Some(e) = error {
                errors.push(format!("{instance}: {e}"));
            }
        }
    };
    for chunk in logic.chunks(CHUNK_SAMPLES * unit) {
        if shared.stop.load(Ordering::Relaxed) {
            break;
        }
        stack.feed(chunk);
        shared.fed.fetch_add((chunk.len() / unit) as u64, Ordering::Relaxed);
        while let Ok(o) = rx.try_recv() {
            handle(o);
        }
    }
    stack.end();
    // `recv` ends when every decoder thread is gone, so a thread that
    // died without its End cannot hang the drain.
    while let Ok(o) = rx.recv() {
        handle(o);
    }
    stack.join();
    let stopped = shared.stop.load(Ordering::Relaxed);
    *lock(&shared.status) = if !errors.is_empty() {
        Status::Error(errors.join("; "))
    } else if stopped {
        Status::Error("stopped".to_owned())
    } else {
        Status::Complete
    };
}

/// A meta item as text.
pub fn item_text(item: &Item) -> String {
    match item {
        Item::Str(s) => s.clone(),
        Item::Int(i) => i.to_string(),
        Item::BigUint(bytes) => bytes.iter().fold("0x".to_owned(), |mut s, b| {
            let _ = write!(s, "{b:02x}");
            s
        }),
        Item::Float(f) => f.to_string(),
        Item::Bool(b) => b.to_string(),
        Item::None => "None".to_owned(),
        Item::Bytes(b) => format!("{} bytes", b.len()),
        Item::List(items) => {
            format!("[{}]", items.iter().map(item_text).collect::<Vec<_>>().join(", "))
        }
        Item::Opaque(_) => "<object>".to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Channel, ChannelKind};
    use crate::setup::decoders::parse_stacks;

    fn registry() -> super::super::Registry {
        let mut r = super::super::Registry::new();
        r.register(&crate::decoders::ALL);
        r
    }

    /// A UART byte at 10 samples per bit on bit 0: idle high, start,
    /// 8 data bits LSB first, stop.
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

    fn uart_plan(spec: &str) -> StackPlan {
        let channels = vec![Channel::new(0, ChannelKind::Logic, "D0")];
        parse_stacks(&registry(), &[spec.to_owned()], &channels).unwrap()
    }

    #[test]
    fn decodes_and_keeps_frames_with_values() {
        let plan = uart_plan("uart:rx=D0:baudrate=100");
        let a = Analyzer::start(plan, Radix::Ascii, true, uart_bytes(b"Hi"), 1, 1000);
        let status = a.wait_until(Instant::now() + Duration::from_secs(10), || true);
        assert_eq!(status, Status::Complete);
        assert_eq!(a.progress().0, a.progress().1);
        let (i, inst) = a.instance("uart-1").unwrap();
        assert_eq!(inst.radix_support, RadixSupport::RawValues);
        let store = a.store(i);
        // The `rx-data` row holds the start and stop bits too; the class
        // is the bytes.
        let data_row = inst.row_index("rx-data").unwrap();
        let data_class = inst.class_index("rx-data").unwrap();
        let data: Vec<(u64, Option<(u64, u32)>)> = store
            .in_row(data_row, 0, u64::MAX)
            .filter(|(_, f)| f.class == data_class)
            .map(|(_, f)| (f.start, f.value))
            .collect();
        assert_eq!(data.len(), 2, "{data:?}");
        assert_eq!(data[0].1, Some((u64::from(b'H'), 8)));
        assert_eq!(data[1].1, Some((u64::from(b'i'), 8)));
        assert_eq!(store.row_count(data_row), 6);
        assert_eq!(store.class_count(data_class), 2);
        assert!(store.binary().len() >= 2);
        // `memory_bytes` locks the store again.
        drop(store);
        assert_eq!(a.radix(), Radix::Ascii);
        assert_eq!(a.samplerate(), 1000);
        assert!(a.memory_bytes() > 0);
    }

    #[test]
    fn a_stack_names_both_instances() {
        let channels = vec![
            Channel::new(0, ChannelKind::Logic, "D0"),
            Channel::new(1, ChannelKind::Logic, "D1"),
        ];
        let plan =
            parse_stacks(&registry(), &["i2c:scl=D0:sda=D1,eeprom24xx".to_owned()], &channels)
                .unwrap();
        let a = Analyzer::start(plan, Radix::Native, true, vec![3u8; 100], 1, 1000);
        assert_eq!(
            a.wait_until(Instant::now() + Duration::from_secs(10), || true),
            Status::Complete
        );
        let ids: Vec<&str> = a.instances().iter().map(|i| i.id.as_str()).collect();
        assert_eq!(ids, ["i2c-1", "eeprom24xx-1"]);
        assert_eq!(a.frames(), 0);
    }

    #[test]
    fn a_bad_option_is_an_error_and_stop_joins() {
        let plan = uart_plan("uart:rx=D0:baudrate=0");
        let a = Analyzer::start(plan, Radix::Native, true, uart_bytes(b"x"), 1, 1000);
        let status = a.wait_until(Instant::now() + Duration::from_secs(10), || true);
        assert!(matches!(status, Status::Error(_)), "{status:?}");
        let plan = uart_plan("uart:rx=D0:baudrate=100");
        let a = Analyzer::start(plan, Radix::Native, true, uart_bytes(b"x"), 1, 1000);
        a.stop();
        assert!(a.is_done());
        assert_eq!(item_text(&Item::List(vec![Item::Int(1), Item::Str("a".into())])), "[1, a]");
    }
}
