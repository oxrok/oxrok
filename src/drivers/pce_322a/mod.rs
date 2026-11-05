// Counterpart in sigrok:
//   libsigrok/src/hardware/pce-322a/api.c,
//   libsigrok/src/hardware/pce-322a/protocol.c,
//   Copyright (C) 2016 George Hopkins <george-hopkins@null.net>,
//   Copyright (C) 2016 Matthieu Guillaumin <matthieu@guillaum.in>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The PCE PCE-322A, a class 2 sound level meter behind a USB serial port. It
//! streams its readings, and holds recordings in a memory that is read by blocks.
//!
//! `conn=` names the port, and any port that opens is a meter: the scan sends and
//! reads nothing, as in C. The open sends `connect` and the close `disconnect`.
//!
//! `data_source` picks what a capture reads. `Live` takes readings from the stream.
//! `Memory` disconnects, asks for the memory's status and reads each block in turn,
//! then ends by itself. Each recording starts with a reading of -1, as in C, which
//! has no other way of saying so.
//!
//! **The meter says how it is set only in its readings,** and only a capture reads
//! them. Before one, this assumes A, F and 30-130, whatever the meter holds, as C
//! does, and the settings are toggles: `spl_weight_freq=A` on a meter at C sends
//! nothing and leaves it at C, and `=C` on it toggles it to A. A reading then sets
//! things right, in what the capture says and in the next toggle. There is no
//! other way to know. `aa f2` is taken to step upwards, from what `MEAS_RANGE_*`
//! number in C and the wiki's list of ranges.
//!
//! Where this differs from C, each with its cause:
//!
//! - A setting counts as done, so the next one and a get know of it. C only ever
//!   takes the settings from a reading (`protocol.c:374,395,452`). There `-c
//!   spl_meas_range=30-80 -c spl_meas_range=50-100` steps three times, to 80-130;
//!   `spl_weight_freq=C` set twice toggles the meter back to A; and a get right after
//!   a set says what it was before.
//! - A sample count starts again with each capture, where `num_samples`
//!   (`protocol.c:91`) goes on, so C's second capture on an open device ends at its
//!   first sample once the first has reached a limit.
//! - A limit cuts a memory capture's output, with the same requests on the wire.
//!   C ignores the limit there, on purpose (`protocol.c:92`), and reads and sends it
//!   all.
//! - The status may end in `10` as well as `20`, the wiki's two values. C's check
//!   (`protocol.c:185`) never takes `10`, and waits for ever.
//! - A silent meter ends a capture that only a sample limit could end, and any
//!   memory capture, after [`SILENCE_BOUND`], as nothing else would: C waits for
//!   ever (`protocol.c:321`), and the scan vouches for no meter. A live capture with
//!   no limit waits for a stop.
//! - A port that is gone ends a capture. C's callback does nothing but return when
//!   `revents` holds more than `G_IO_IN` (`protocol.c:321`), so it spins for ever.
//! - A failed `connect` closes the port it opened, where `dev_open` leaves it open
//!   (`api.c:240`).
//! - What a capture leaves behind does not reach the next. A capture after a memory
//!   capture first waits for the line to go quiet, which is what a stop in mid-block
//!   needs, and a live one then connects again, as the memory capture disconnected.
//!   A live capture after another first drops what has queued up, which is old. C
//!   does none of this: its only client that runs twice is a library's, and no run
//!   here compares them. A first capture is as in C.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{Item, Measurement, Memory, Mode, RANGES, SERIALCOMM, StatusWindow, Stream};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::SoundLevelMeter];
static DEV_OPTIONS: [OptionDesc; 7] = [
    // C lists it with a set and `config_set` has no case for it: not applicable.
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::SplWeightFreq, Access::GET_SET_LIST),
    OptionDesc::new(Key::SplWeightTime, Access::GET_SET_LIST),
    OptionDesc::new(Key::SplMeasurementRange, Access::GET_SET_LIST),
    OptionDesc::new(Key::PowerOff, Access::GET_SET),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
];
/// How long a capture that nothing else can end waits for a meter that says
/// nothing. No source gives the stream's rate. A block's bytes take 22 ms.
pub const SILENCE_BOUND: Duration = Duration::from_secs(5);
/// `serial_source_add`'s timeout: how long C's callback waits before it goes on to
/// a request, if nothing comes. C's requests keep this pace.
const POLL: Duration = Duration::from_millis(150);
/// One read's wait, and so how long a setter may wait for the lock.
const READ_WAIT: Duration = Duration::from_millis(20);
/// The pause after a read that gave nothing, with the lock free: a mutex is not
/// fair, and a reader that takes it back at once would starve a setter.
const EMPTY_READ_PAUSE: Duration = Duration::from_millis(1);
/// How long with no byte says a stopped block is over. A byte takes 87 us, and a
/// USB adapter holds bytes for some ms.
const QUIET: Duration = Duration::from_millis(50);
/// The most that wait lasts, if something goes on sending.
const QUIET_BOUND: Duration = Duration::from_secs(1);
/// How long a stopped capture tries to get its End to a queue no one may read.
const END_AFTER_STOP: Duration = Duration::from_secs(1);

/// The `pce-322a` driver.
pub struct Pce322aDriver;

impl Driver for Pce322aDriver {
    fn id(&self) -> &'static str {
        "pce-322a"
    }

    fn long_name(&self) -> &'static str {
        "PCE PCE-322A"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let params = Params::parse(SERIALCOMM)?;
        // The port opens and closes, and that is all the scan asks.
        match crate::serial::open(&conn, params) {
            Ok(port) => drop(port),
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        }
        Ok(vec![Box::new(Pce322aDevice::new(conn, params))])
    }
}

/// What the capture and the frontend share: `dev_context`, and the port.
#[allow(clippy::struct_excessive_bools)]
struct Shared {
    port: Option<Box<dyn Port>>,
    /// As set, or as the last reading said.
    mode: Mode,
    memory: bool,
    limit_samples: u64,
    /// Between the first thing a capture does and the last thing before End.
    capturing: bool,
    /// A live capture has run, so what has queued up since is old.
    stream_read: bool,
    /// A memory capture has told the meter to disconnect, and may have left a block
    /// on its way.
    disconnected: bool,
}

impl Shared {
    fn port(&mut self) -> Result<&mut dyn Port> {
        match self.port.as_mut() {
            Some(port) => Ok(port.as_mut()),
            None => Err(Error::device("device is not open")),
        }
    }

    fn send(&mut self, command: &[u8]) -> Result<()> {
        self.port()?.write(command)
    }
}

/// How long the waits of a capture are: the constants, but for tests.
#[derive(Clone, Copy, Debug)]
struct Waits {
    silence: Duration,
    poll: Duration,
    quiet: Duration,
    quiet_bound: Duration,
}

impl Default for Waits {
    fn default() -> Waits {
        Waits { silence: SILENCE_BOUND, poll: POLL, quiet: QUIET, quiet_bound: QUIET_BOUND }
    }
}

/// The one meter.
pub struct Pce322aDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    waits: Waits,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    shared: Arc<Mutex<Shared>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

fn no_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        None => Ok(()),
    }
}

impl Pce322aDevice {
    fn new(conn: String, params: Params) -> Self {
        Pce322aDevice {
            info: DeviceInfo {
                driver: "pce-322a",
                vendor: "PCE".to_owned(),
                model: "PCE-322A".to_owned(),
                version: String::new(),
                serial: String::new(),
                // `conn` is not in `devopts`, so sigrok-cli names none.
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "SPL")],
            conn,
            params,
            waits: Waits::default(),
            pending: None,
            shared: Arc::new(Mutex::new(Shared {
                port: None,
                mode: Mode::default(),
                memory: false,
                limit_samples: 0,
                capturing: false,
                stream_read: false,
                disconnected: false,
            })),
            worker: None,
        }
    }

    /// A device on a port that stands in for the hardware, with no scan.
    /// [`Device::open`] takes it.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let mut device = Self::new(port.name().to_owned(), params);
        device.pending = Some(port);
        device
    }

    fn shared(&self) -> MutexGuard<'_, Shared> {
        self.shared.lock().expect("state lock")
    }
}

impl Device for Pce322aDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn enable_channel(&mut self, pos: usize, enabled: bool) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        ch.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        name.clone_into(&mut ch.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        no_group(group)?;
        Ok(DEV_OPTIONS.to_vec())
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        no_group(group)?;
        let s = self.shared();
        Ok(match key {
            Key::LimitSamples => Value::U64(s.limit_samples),
            Key::SplWeightFreq => Value::Str(if s.mode.c_weight { "C" } else { "A" }.to_owned()),
            Key::SplWeightTime => Value::Str(if s.mode.slow { "S" } else { "F" }.to_owned()),
            Key::SplMeasurementRange => {
                let (low, high) = RANGES[usize::from(s.mode.range)];
                Value::RangeU64(low, high)
            }
            Key::PowerOff => Value::Bool(false),
            Key::DataSource => Value::Str(if s.memory { "Memory" } else { "Live" }.to_owned()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// The toggles go out at once, also between a capture's reads. The source
    /// cannot change under a capture: C's callback would switch to the other
    /// state machine in mid-read.
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let bad = || Error::Arg(format!("{}: invalid value", key.id()));
        match key {
            Key::LimitSamples => {
                self.shared().limit_samples = value.as_u64().ok_or_else(bad)?;
                Ok(())
            }
            Key::SplWeightFreq => {
                let want = match value.as_str() {
                    Some("A") => false,
                    Some("C") => true,
                    _ => return Err(bad()),
                };
                let mut s = self.shared();
                if s.mode.c_weight != want {
                    s.send(&protocol::TOGGLE_WEIGHT_FREQ)?;
                    s.mode.c_weight = want;
                }
                Ok(())
            }
            Key::SplWeightTime => {
                let want = match value.as_str() {
                    Some("F") => false,
                    Some("S") => true,
                    _ => return Err(bad()),
                };
                let mut s = self.shared();
                if s.mode.slow != want {
                    s.send(&protocol::TOGGLE_WEIGHT_TIME)?;
                    s.mode.slow = want;
                }
                Ok(())
            }
            Key::SplMeasurementRange => {
                let Value::RangeU64(low, high) = value else { return Err(bad()) };
                let want = RANGES.iter().position(|r| *r == (*low, *high)).ok_or_else(bad)?;
                let want = u8::try_from(want).expect("four ranges");
                let mut s = self.shared();
                for _ in 0..protocol::range_steps(s.mode.range, want) {
                    s.send(&protocol::TOGGLE_MEAS_RANGE)?;
                    s.mode.range = (s.mode.range + 1) & 3;
                }
                Ok(())
            }
            Key::PowerOff => {
                if value.as_bool().ok_or_else(bad)? {
                    self.shared().send(&protocol::POWER_OFF)?;
                }
                Ok(())
            }
            Key::DataSource => {
                let memory = match value.as_str() {
                    Some("Live") => false,
                    Some("Memory") => true,
                    _ => return Err(bad()),
                };
                let mut s = self.shared();
                if s.capturing {
                    return Err(Error::Busy);
                }
                s.memory = memory;
                Ok(())
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        let strings = |list: &[&str]| ValueList::Str(list.iter().map(|&s| s.to_owned()).collect());
        Ok(match key {
            Key::SplWeightFreq => strings(&["A", "C"]),
            Key::SplWeightTime => strings(&["F", "S"]),
            Key::SplMeasurementRange => ValueList::RangeU64(RANGES.to_vec()),
            Key::DataSource => strings(&["Live", "Memory"]),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `std_serial_dev_open` and `connect`.
    fn open(&mut self) -> Result<()> {
        let pending = self.pending.take();
        let params = self.params;
        let conn = self.conn.clone();
        let mut s = self.shared();
        if s.port.is_some() {
            return Ok(());
        }
        let mut port = match pending {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&conn, params)?,
        };
        port.write(&protocol::CONNECT)?;
        s.port = Some(port);
        s.disconnected = false;
        s.stream_read = false;
        Ok(())
    }

    /// `disconnect`, whose failure C drops, and the port.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let mut s = self.shared();
        if let Some(mut port) = s.port.take() {
            let _ = port.write(&protocol::DISCONNECT);
        }
        Ok(())
    }

    /// Busy while a capture runs. One past its last read has only its End to send:
    /// told to stop, it gives that up within [`END_AFTER_STOP`] if the old client's
    /// queue stays full, so this does not wait for ever.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        {
            let mut s = self.shared();
            if s.capturing {
                return Err(Error::Busy);
            }
            if s.port.is_none() {
                return Err(Error::device("device is not open"));
            }
            s.capturing = true;
        }
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
        let shared = Arc::clone(&self.shared);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let waits = self.waits;
        let handle = std::thread::Builder::new()
            .name("pce-322a acquisition".to_owned())
            .spawn(move || {
                let mut capture = Capture::new(&shared, &sink, &stop2, waits);
                capture.run();
                shared.lock().expect("state lock").capturing = false;
                send_end(&sink, &stop2);
            })
            .map_err(|e| {
                self.shared().capturing = false;
                Error::Io(e)
            })?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// The End, which a client that goes on reading gets however long it takes, and
/// after a stop only if there is room within [`END_AFTER_STOP`].
fn send_end(sink: &PacketSink, stop: &AtomicBool) {
    let stopped_at = std::cell::Cell::new(None);
    sink.send_while(Packet::End, || {
        if !stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = stopped_at.get().unwrap_or_else(Instant::now);
        stopped_at.set(Some(at));
        at.elapsed() < END_AFTER_STOP
    });
}

/// A capture on its thread. `None` from a step means the capture is over, with
/// whatever needs saying said.
struct Capture<'a> {
    shared: &'a Mutex<Shared>,
    sink: &'a PacketSink,
    stop: &'a AtomicBool,
    waits: Waits,
    /// What has been read and not looked at.
    input: VecDeque<u8>,
    /// Readings sent in this capture, each recording's -1 too.
    samples: u64,
    /// When the last reading came, or block byte, or request went.
    heard: Instant,
}

impl<'a> Capture<'a> {
    fn new(
        shared: &'a Mutex<Shared>,
        sink: &'a PacketSink,
        stop: &'a AtomicBool,
        waits: Waits,
    ) -> Self {
        Capture {
            shared,
            sink,
            stop,
            waits,
            input: VecDeque::new(),
            samples: 0,
            heard: Instant::now(),
        }
    }

    fn state(&self) -> MutexGuard<'a, Shared> {
        self.shared.lock().expect("state lock")
    }

    fn stopped(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    fn run(&mut self) {
        let go_on = || !self.stop.load(Ordering::SeqCst);
        if !self.sink.send_while(Packet::header(), go_on) {
            return;
        }
        let (memory, disconnected) = {
            let s = self.state();
            (s.memory, s.disconnected)
        };
        if disconnected && self.settle().is_none() {
            return;
        }
        if memory {
            let _ = self.read_memory();
        } else {
            let _ = self.read_stream();
        }
    }

    /// One read into `input`, and whether it gave anything. With the lock held for
    /// the read alone, for a setter or a getter.
    fn read_some(&mut self) -> Option<bool> {
        let mut buf = [0; 256];
        let got = {
            let mut s = self.state();
            match s.port().and_then(|port| port.read(&mut buf, READ_WAIT)) {
                Ok(got) => got,
                Err(e) => {
                    log::error!("Reading the port failed: {e}.");
                    return None;
                }
            }
        };
        if got == 0 {
            std::thread::sleep(EMPTY_READ_PAUSE);
            return Some(false);
        }
        self.input.extend(&buf[..got]);
        Some(true)
    }

    /// Drops what comes until the line has been quiet for a while, as what follows a
    /// stop in mid-block is still on its way, and a purge drops only what has come.
    fn settle(&mut self) -> Option<()> {
        let began = Instant::now();
        let mut quiet_since = Instant::now();
        while quiet_since.elapsed() < self.waits.quiet && began.elapsed() < self.waits.quiet_bound {
            if self.stopped() {
                return None;
            }
            if self.read_some()? {
                self.input.clear();
                quiet_since = Instant::now();
            }
        }
        Some(())
    }

    /// One byte, with a bound if `bounded` on the time since `heard`, which only
    /// what counts sets: a meter that sends junk for ever would defeat a clock that
    /// any byte resets.
    fn next_byte(&mut self, bounded: impl Fn(&Self) -> bool) -> Option<u8> {
        loop {
            if self.stopped() {
                return None;
            }
            if let Some(byte) = self.input.pop_front() {
                return Some(byte);
            }
            self.read_some()?;
            if bounded(self) && self.heard.elapsed() >= self.waits.silence {
                log::error!("Nothing has come of the meter for {:?}.", self.waits.silence);
                return None;
            }
        }
    }

    /// What the callback's way to a request waits: until a byte is there, which it
    /// leaves there, or for the source's timeout.
    fn pause_before_request(&mut self) -> Option<()> {
        let began = Instant::now();
        while self.input.is_empty() && began.elapsed() < self.waits.poll {
            if self.stopped() {
                return None;
            }
            self.read_some()?;
        }
        Some(())
    }

    /// A reading of `value` with the flags as they are.
    fn send_reading(&mut self, value: f32) -> Option<()> {
        let flags = self.state().mode.flags();
        let packet = Packet::Analog(AnalogData {
            channels: vec![0],
            samples: vec![value],
            mq: Mq::SoundPressureLevel,
            flags,
            unit: Unit::DecibelSpl,
            digits: 1,
        });
        self.sink.send_while(packet, || !self.stop.load(Ordering::SeqCst)).then_some(())?;
        self.samples += 1;
        Some(())
    }

    fn limit(&self) -> u64 {
        self.state().limit_samples
    }

    fn limit_reached(&self) -> bool {
        let limit = self.limit();
        limit > 0 && self.samples >= limit
    }

    /// The stream, up to the limit.
    fn read_stream(&mut self) -> Option<()> {
        {
            let mut s = self.state();
            if s.disconnected {
                s.disconnected = false;
                if let Err(e) = s.send(&protocol::CONNECT) {
                    log::warn!("Could not connect again: {e}.");
                }
            } else if s.stream_read
                && let Err(e) = s.port().and_then(Port::purge)
            {
                log::warn!("Could not drop what had queued up: {e}.");
            }
            s.stream_read = true;
        }
        self.heard = Instant::now();
        let mut stream = Stream::default();
        loop {
            let byte = self.next_byte(|c| c.limit() > 0)?;
            if let Some(Measurement { value, mode }) = stream.feed(byte) {
                self.heard = Instant::now();
                self.state().mode = mode;
                self.send_reading(protocol::tenths(value))?;
                if self.limit_reached() {
                    return Some(());
                }
            }
        }
    }

    /// `pce_322a_receive_data`'s memory states, in order: what C's callback goes
    /// through a turn at a time.
    fn read_memory(&mut self) -> Option<()> {
        self.pause_before_request()?;
        log::warn!("Requesting memory usage.");
        {
            let mut s = self.state();
            s.disconnected = true;
            // C drops what these return: what a failure leaves is a silent line.
            let _ = s.send(&protocol::DISCONNECT);
            let _ = s.send(&protocol::MEMORY_STATUS);
        }
        self.heard = Instant::now();
        let mut window = StatusWindow::default();
        let status = loop {
            let byte = self.next_byte(|_| true)?;
            if let Some(status) = window.feed(byte) {
                break status;
            }
        };
        log::warn!(
            "Memory usage: {} blocks of 256 bytes, 1 block of {} bytes",
            i32::from(status.used) - 1,
            status.last
        );
        let mut memory = Memory::default();
        loop {
            self.pause_before_request()?;
            log::warn!("Requesting memory block {}.", memory.block());
            {
                let mut s = self.state();
                let _ = s.send(&protocol::memory_block(memory.block()));
            }
            self.heard = Instant::now();
            loop {
                if memory.done(status) {
                    log::warn!("Done reading memory ({} bytes).", memory.logged_size());
                    return Some(());
                }
                let byte = self.next_byte(|_| true)?;
                self.heard = Instant::now();
                match memory.feed(byte) {
                    Item::Nothing => {}
                    Item::Recording(recording) => {
                        log::error!("{}", recording.describe());
                        self.state().mode.c_weight = recording.c_weight();
                        self.send_memory_reading(-1.0)?;
                    }
                    Item::Sample(value) => self.send_memory_reading(protocol::tenths(value))?,
                }
                if memory.at_block_start() {
                    break;
                }
            }
        }
    }

    /// A reading of a memory capture: the wire goes on past a limit, and the output
    /// does not.
    fn send_memory_reading(&mut self, value: f32) -> Option<()> {
        if self.limit_reached() { Some(()) } else { self.send_reading(value) }
    }
}

#[cfg(test)]
mod tests;
