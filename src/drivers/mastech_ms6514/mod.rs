// Counterpart in sigrok:
//   libsigrok/src/hardware/mastech-ms6514/api.c,
//   libsigrok/src/hardware/mastech-ms6514/protocol.c,
//   Copyright (C) 2019 Dave Buechi <db@pflutsch.ch>,
//   with commits by Gerhard Sittig, Wolfram Sang, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The MASTECH MS6514, a dual-channel thermometer behind a CP2102.
//!
//! `conn=` names the port and `serialcomm=` overrides 9600/8n1. The unit sends
//! frames by itself once PC-Link is on, so a scan listens for one, and a capture
//! reads them. `data_source=Memory` asks for the stored values at a capture's
//! start, and the first live frame ends the capture.
//!
//! **The C driver finds no device.** `api.c:85` asks `serial_stream_detect` for
//! a packet of two frames, with a buffer of two frames, and `serial.c:833` wants a
//! buffer of two packets: it says "Small stream detect RX buffer, want 2x packet
//! size." and every scan ends there. The check is from 2012 (2546b05c, `maxlen <
//! packet_size * 2` at 23669c3d^), and the driver had the same 36 and 36 from its
//! pull request on (13b72fa9, 23669c3d, 2019), so no release has run it. So
//! this driver is compared with the C with that one argument changed in memory
//! (`tools/emu/mastech-ms6514-scan.c`), and C's behaviour below means that.
//!
//! Where this differs from that C, each with its cause:
//!
//! - A scan looks for one frame, as above, and for 1 s where C looks for 500 ms:
//!   the wiki's "approximately 2 Hz" leaves C's fixed scan a chance of no frame in
//!   the time, and no C run has ever had one. It keeps C's 36 bytes.
//! - `-C` picks channels. C's `protocol.c:162-165` looks at the first two
//!   channels by display, so with T2 on main `-C T1` gives T2's reading. Here a reading goes out if the channel it names is enabled.
//!   A frame counts as a sample either way, as in C.
//! - A time limit ends a capture at its time. C looks at the clock only after
//!   bytes came (`protocol.c:215`, `:244`), so a unit that went quiet holds it for
//!   ever, and a limit shorter than the wait for a frame gives that frame.
//! - A failed read ends the capture. A port that hung up gives C a `revents` of
//!   more than `G_IO_IN`, which `protocol.c:215` takes as nothing to do, for ever.
//! - The first live frame of a dump ends the capture there. C sets its stop and
//!   goes on with the read: what it sends after its end reaches no output.
//! - A second capture on an open port drops what came since the first. C reads
//!   those as readings, however old, and its `buf_len` keeps the first's tail.
//!   What is still to come of a dump a limit cut short counts in a live capture,
//!   as a stored frame does in C.
//! - A dump that ends before any stored value came warns: see below.
//!
//! Kept from C, as nothing says otherwise:
//!
//! - A sample limit is looked at after a whole read, so a read holding three
//!   frames sends them all, and a head with no end drops 18 bytes.
//! - A sample limit alone waits for ever on a unit that went quiet. It needs no
//!   request, and a silent unit may come back: `MS6514GUI.pde:465` notes "BUG IN
//!   DEVICE!! doesn't send serial in this mode".
//! - A live frame that crosses the request ends a dump before it starts, with
//!   nothing captured. Nothing tells it from the frame that follows a dump, and a
//!   unit with an empty memory may send only that, so a capture that ends with no
//!   sample says so and exits as usual. At 2 Hz a frame on the line takes up at
//!   least 3.75% of the time. No source says what follows a dump, or how fast it
//!   comes.
//!
//! A known limit: `-O csv` takes readings by count, and a frame names two of three
//! channels, often the same one twice, so its rows mix frames, with a 0 for what
//! did not come. It holds with `-C T1,T2` on frames showing T1 and T2 alone.
//! [`Device::note_output`] warns.
//!
//! A scan opens the port for reading and writing, where C opens it read-only.
//! `data_source` set during a capture counts for the next one, where C's callback
//! would see it at once.

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{BUF_SIZE, CMD_GET_STORED, FRAME_SIZE, Step};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Thermometer];
static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
];
const DATA_SOURCES: [&str; 2] = ["Live", "Memory"];
const CHANNEL_NAMES: [&str; 3] = ["T1", "T2", "T1-T2"];
const SERIALCOMM: &str = "9600/8n1";

/// How long a scan listens. At about 2 frames a second this is two chances.
const SCAN_TIME: Duration = Duration::from_secs(1);
/// What a scan may read: a frame cut by the open and a whole one.
const SCAN_BYTES: usize = 2 * FRAME_SIZE;
/// How long one read waits, and so how late a stop or a time limit can be.
const READ_TIME: Duration = Duration::from_millis(50);
/// A pause after an empty read: a hung up tty may give one at once.
const EMPTY_READ_PAUSE: Duration = Duration::from_millis(1);
/// How long an `End` waits for room after a stop: a close joins the thread.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// The `mastech-ms6514` driver.
pub struct MastechMs6514Driver;

impl Driver for MastechMs6514Driver {
    fn id(&self) -> &'static str {
        "mastech-ms6514"
    }

    fn long_name(&self) -> &'static str {
        "MASTECH MS6514"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = SERIALCOMM.to_owned();
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => {
                    value
                        .as_str()
                        .ok_or_else(|| Error::arg("serialcomm"))?
                        .clone_into(&mut serialcomm);
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (mut port, params) = match opened {
            Ok(opened) => opened,
            // A port that does not open is no device, not a failed scan.
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        log::info!("Probing serial port {conn}.");
        if !detect(port.as_mut(), &params, SCAN_TIME) {
            return Ok(Vec::new());
        }
        log::info!("Found device on port {conn}.");
        drop(port);
        Ok(vec![Box::new(MastechMs6514Device::new(conn, params, None))])
    }
}

/// Reads byte by byte until a frame shows, `timeout` has passed or
/// [`SCAN_BYTES`] came, as `serial_stream_detect` does for one frame's size.
fn detect(port: &mut dyn Port, params: &Params, timeout: Duration) -> bool {
    let byte_delay = crate::serial::serial_timeout(params, 1);
    let start = Instant::now();
    let mut buf = Vec::with_capacity(SCAN_BYTES);
    let mut check = 0;
    while buf.len() < SCAN_BYTES {
        let mut byte = [0u8];
        match port.read(&mut byte, byte_delay) {
            Ok(1) => buf.push(byte[0]),
            Ok(_) => std::thread::sleep(EMPTY_READ_PAUSE),
            // Run on to the time as C does: a failed read is no byte.
            Err(e) => {
                log::debug!("Scan read: {e}.");
                std::thread::sleep(byte_delay);
            }
        }
        if buf.len() - check >= FRAME_SIZE {
            if protocol::packet_valid(&buf[check..]) {
                return true;
            }
            check += 1;
        }
        if start.elapsed() >= timeout {
            break;
        }
    }
    log::info!("Didn't find a valid packet (read {} bytes).", buf.len());
    false
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// One MS6514.
pub struct MastechMs6514Device {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limit_samples: u64,
    limit_msec: u64,
    memory: bool,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// The port, while no capture holds it. A capture gives it back before its
    /// `End`.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    capturing: Arc<AtomicBool>,
    /// A capture ran on this open, so what is queued is old.
    captured: bool,
    worker: Option<Worker>,
}

impl MastechMs6514Device {
    fn new(conn: String, params: Params, port: Option<Box<dyn Port>>) -> Self {
        let channels = CHANNEL_NAMES
            .iter()
            .enumerate()
            .map(|(i, n)| Channel::new(i, ChannelKind::Analog, (*n).to_owned()))
            .collect();
        MastechMs6514Device {
            info: DeviceInfo {
                driver: "mastech-ms6514",
                vendor: "MASTECH".to_owned(),
                model: "MS6514".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels,
            conn,
            params,
            limit_samples: 0,
            limit_msec: 0,
            memory: false,
            pending: port,
            port: Arc::new(Mutex::new(None)),
            capturing: Arc::new(AtomicBool::new(false)),
            captured: false,
            worker: None,
        }
    }

    /// A device on an already open port, for a test. No scan is run.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let name = port.name().to_owned();
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        Self::new(name, params, Some(port))
    }

    fn port(&self) -> MutexGuard<'_, Option<Box<dyn Port>>> {
        self.port.lock().expect("port lock")
    }

    /// Ends a worker that is done or told to end, which gives up an `End` that
    /// no one takes within [`END_AFTER_STOP`].
    fn join_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for MastechMs6514Device {
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
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::LimitMsec => Value::U64(self.limit_msec),
            Key::DataSource => Value::Str(DATA_SOURCES[usize::from(self.memory)].to_owned()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => self.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::LimitMsec => self.limit_msec = value.as_u64().ok_or_else(bad)?,
            Key::DataSource => {
                self.memory = match value.as_str().ok_or_else(bad)? {
                    "Live" => false,
                    "Memory" => true,
                    _ => return Err(bad()),
                };
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::DataSource => {
                Ok(ValueList::Str(DATA_SOURCES.iter().map(|s| (*s).to_owned()).collect()))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn open(&mut self) -> Result<()> {
        if self.port().is_some() || self.capturing.load(Ordering::SeqCst) {
            return Ok(());
        }
        let port = match self.pending.take() {
            // As `serial_open` flushes a port it opens.
            Some(mut p) => {
                p.purge()?;
                p
            }
            None => crate::serial::open(&self.conn, self.params)?,
        };
        *self.port() = Some(port);
        self.captured = false;
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.join_worker();
        *self.port() = None;
        Ok(())
    }

    /// A known limit: csv takes readings by count, and a frame names two of the
    /// three channels. C warns as well (`output/csv.c:650-656`).
    fn note_output(&self, module: &str) {
        if module == "csv" {
            log::warn!(
                "Resulting CSV output data may be incomplete or incorrect: rows pair readings \
                 by count, with 0 for a channel that did not come, so they can mix frames. Use \
                 -O analog."
            );
        }
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        // The last capture has given the port back, and may be waiting to send
        // its `End` to a queue no one reads.
        self.join_worker();
        let Some(mut port) = self.port().take() else {
            return Err(Error::device("device is not open"));
        };
        if self.captured {
            // Frames from since the last capture are not this one's. A frame on
            // its way now leaves a tail, which is out of step and skipped.
            if let Err(e) = port.purge() {
                *self.port() = Some(port);
                return Err(e);
            }
        }
        self.captured = true;
        let capture = Capture {
            enabled: self.channels.iter().map(|c| c.enabled).collect(),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            memory: self.memory,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let slot = Arc::clone(&self.port);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new()
            .name("mastech-ms6514 acquisition".to_owned())
            .spawn(move || {
                capture.run(port.as_mut(), &sink, &stop2);
                *slot.lock().expect("port lock") = Some(port);
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2);
            });
        match spawned {
            Ok(handle) => {
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                self.capturing.store(false, Ordering::SeqCst);
                Err(Error::Io(e))
            }
        }
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Drop for MastechMs6514Device {
    fn drop(&mut self) {
        self.join_worker();
    }
}

/// `End`, waiting for room for as long as nobody asked to stop, and after a stop
/// only for [`END_AFTER_STOP`].
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

/// One capture, on its own thread.
struct Capture {
    enabled: Vec<bool>,
    limit_samples: u64,
    limit_msec: u64,
    memory: bool,
}

impl Capture {
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return;
        }
        let started = Instant::now();
        if self.memory {
            if let Err(e) = port.write(&[CMD_GET_STORED]) {
                // C drops the result, and whatever comes decides what happens.
                log::warn!("Could not ask for the stored values: {e}.");
            }
        }
        let limit_time = Duration::from_millis(self.limit_msec);
        let mut samples = 0u64;
        let mut buf = [0u8; BUF_SIZE];
        let mut len = 0;
        while go_on() {
            if self.limit_msec > 0 && started.elapsed() >= limit_time {
                return;
            }
            let n = match port.read(&mut buf[len..], READ_TIME) {
                Ok(n) => n,
                Err(e) => {
                    log::error!("Serial port read error: {e}.");
                    return;
                }
            };
            if n == 0 {
                std::thread::sleep(EMPTY_READ_PAUSE);
                continue;
            }
            len += n;
            let mut pos = 0;
            while len - pos >= FRAME_SIZE {
                let frame = &buf[pos..pos + FRAME_SIZE];
                match protocol::step(frame) {
                    Step::NoHead => pos += 1,
                    Step::NoEnd => pos += FRAME_SIZE,
                    Step::Frame => {
                        // The end of a dump.
                        if self.memory && !protocol::is_stored(frame) {
                            if samples == 0 {
                                log::warn!(
                                    "A live frame came before any stored value: the memory is \
                                     empty, or a live frame crossed the request."
                                );
                            }
                            return;
                        }
                        pos += FRAME_SIZE;
                        if !self.send(frame, sink, stop) {
                            return;
                        }
                        samples += 1;
                    }
                }
            }
            buf.copy_within(pos..len, 0);
            len -= pos;
            // After a whole read, as a read holding several frames sends them.
            if self.limit_samples > 0 && samples >= self.limit_samples {
                return;
            }
        }
    }

    /// Sends a frame's two readings where their channels are enabled: false when
    /// the capture should end.
    fn send(&self, frame: &[u8], sink: &PacketSink, stop: &AtomicBool) -> bool {
        for slot in 0..2 {
            let channel = protocol::channel(frame, slot);
            if !self.enabled.get(channel).copied().unwrap_or(false) {
                continue;
            }
            let (value, digits) = protocol::temperature(frame, slot);
            let data = AnalogData {
                channels: vec![channel],
                samples: vec![value],
                mq: Mq::Temperature,
                flags: protocol::flags(frame, slot),
                unit: protocol::unit(frame),
                digits,
            };
            if !sink.send_while(Packet::Analog(data), || !stop.load(Ordering::SeqCst)) {
                return false;
            }
        }
        true
    }
}
