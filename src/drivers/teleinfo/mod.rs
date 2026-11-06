// Counterpart in sigrok:
//   libsigrok/src/hardware/teleinfo/api.c,
//   libsigrok/src/hardware/teleinfo/protocol.c,
//   Copyright (C) 2013 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Martin Ling, Daniel Elstner, Gerhard Sittig, Soeren Apel, Wolfram Sang,
//   Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! EDF's Téléinfo, the customer output of a French electricity meter, in its
//! "historic" form, through a serial cable at 1200/7e1.
//!
//! `conn=` names the port and `serialcomm=` overrides the line. A meter sends for as
//! long as it has power and takes nothing, so nothing here ever writes. A scan
//! reads 292 bytes, and the `OPTARIF` group in them picks the channels. A capture
//! ends at `limit_samples`, in frames (each `ADCO` group counts one), at `limit_time`,
//! or at a stop.
//!
//! **The stock C driver finds no device.** `api.c:79` asks `serial_stream_detect` for
//! a packet of 292 bytes with a buffer of 292, and `serial.c:833` wants a buffer of
//! two packets: it says "Small stream detect RX buffer, want 2x packet size." and
//! every scan ends there. Until 0.5.1 that check was `maxlen < packet_size / 2`;
//! d03815a0 (2019) made it `* 2`, and nothing has run this driver since 0.5.2. So it
//! is compared with the C with that one check taken out in memory
//! (`tools/emu/teleinfo-scan.c`), which reads 292 bytes and then looks, as 0.5.1 did,
//! and C's behaviour below means that.
//!
//! Where this differs from that C, each with its cause:
//!
//! - A group whose control is a space is taken. A sum's low six bits of 0 give one
//!   group in 64 that control, which the specification allows (Enedis-NOI-CPT_02E,
//!   1.5.2, a control of 0x20 to 0x7e; `PTEC HP..` always has it; `LibTeleinfo`'s parser
//!   notes "checksum last one can be space separator"). In `sscanf`'s
//!   format (`protocol.c:134`) the space ahead of `%c` steps over it, so C reads the
//!   CR as the control and drops the group: `BASE` at 399 Wh gives no reading, and an
//!   `ADCO` that has it counts no sample.
//! - `-C OLD=NEW` keeps the channel's readings. C finds a channel by its name when a
//!   reading comes (`protocol.c:53,68`), so a renamed one gets none.
//! - A time limit ends a capture at its time. C returns from a read's timeout before
//!   its limit check (`protocol.c:193`, `:222`), so a meter that went quiet holds it for
//!   ever.
//! - A failed read ends the capture. A port that hung up gives C a `revents` of more
//!   than `G_IO_IN`, which `protocol.c:193` takes as nothing to do, for ever.
//! - A sample limit does not wait for ever: with nothing else to end the capture, 10 s
//!   with no `ADCO` group fails it, and it says whether bytes came. C has no bound. A
//!   frame takes at most about 4 s at 120 bytes a second. The specification warns of
//!   interruptions of no stated length while a meter is read another way, which it
//!   announces with EOT, so behind an EOT the wait is 5 minutes.
//! - Buffer full: 256 bytes with no whole group end the capture in both, but C
//!   `return FALSE`s (`protocol.c:219`) with no `SR_DF_END` and exit 0. Here it is a
//!   failed capture, saying so.
//! - A second capture on an open port drops what came since the first, as those would
//!   be up to half a minute old. A group cut by the end of the first capture stays with
//!   that capture, where C's `buf_len` keeps it.
//! - A group is read within the bytes that have come: C's `sscanf` has no bound, and on
//!   a group that crosses a CR it may read the buffer's stale tail.
//!
//! Kept from C, as nothing says otherwise:
//!
//! - A limit is looked at after a whole read, so a read holding two frames sends both.
//! - 256 bytes with no group end a capture of any limit, rather than being dropped as
//!   the C's comment says and its `return FALSE` does not.
//! - `atoi`'s readings of what are not numbers: `ABC` is 0, and all above 2^24 is a
//!   float's nearest. Nothing here rejects a group on its data.
//! - A group of 13 bytes of data never counts. `protocol.c:132`'s `data[13]` has no
//!   room for `%13s`'s NUL, which at the oracle's build clears the label. No group of
//!   the specification has 13.
//! - An `OPTARIF` over 4 bytes that starts `BBR` is Tempo, though C's `strcpy` overruns
//!   `optarif[5]` to say so (`protocol.c:95`), with no harm seen on the oracle's build.
//! - A meter that says no known option is no device, and the scan needs 292 bytes in 3
//!   s (2.43 s at the line's rate).
//! - The `P` channel exists and never gets a reading: no group feeds it.
//!
//! A scan opens the port for reading and writing, where C opens it read-only, so a
//! node a user may only read does not open.

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    MqFlags, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{BUF_SIZE, EOT, Meaning, Optarif, SCAN_BYTES};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::EnergyMeter];
const SET_ONLY: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, SET_ONLY),
    OptionDesc::new(Key::LimitMsec, SET_ONLY),
];
const SERIALCOMM: &str = "1200/7e1";

/// How long a scan listens for its bytes (api.c's 3000).
const SCAN_TIME: Duration = Duration::from_millis(3000);
/// How long a read of a capture waits, `serial_source_add`'s 50 ms.
const READ_TIME: Duration = Duration::from_millis(50);
/// A pause after an empty read: a hung up tty may give one at once.
const EMPTY_READ_PAUSE: Duration = Duration::from_millis(1);
/// How long an `End` waits for room after a stop: a close joins the thread.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How long a sample limit alone waits for an `ADCO` group.
const NO_SAMPLE_BOUND: Duration = Duration::from_secs(10);
/// The same behind an EOT. The specification gives no length of an interruption.
const INTERRUPTED_BOUND: Duration = Duration::from_secs(300);

/// The `teleinfo` driver.
pub struct TeleinfoDriver;

impl Driver for TeleinfoDriver {
    fn id(&self) -> &'static str {
        "teleinfo"
    }

    fn long_name(&self) -> &'static str {
        "Teleinfo"
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
        // A serialcomm that does not parse or a port that does not open: no device,
        // as `serial_open`'s failure there.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| crate::serial::open(&conn, params).map(|port| (params, port)));
        let (params, mut port) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        log::info!("Probing serial port {conn}.");
        let Some(optarif) = detect(port.as_mut(), &params, SCAN_TIME) else {
            return Ok(Vec::new());
        };
        log::info!("Found device on port {conn}.");
        // `serial_close`; `dev_open` opens the port again.
        drop(port);
        Ok(vec![Box::new(TeleinfoDevice::new(conn, params, optarif, None))])
    }
}

/// `serial_stream_detect` as it was for this driver up to 0.5.1: reads until it has
/// [`SCAN_BYTES`] or `timeout` passed, then looks at those bytes once. A read that
/// fails is no data, and the time still runs.
fn detect(port: &mut dyn Port, params: &Params, timeout: Duration) -> Option<Optarif> {
    let wait = crate::serial::serial_timeout(params, 1);
    let start = Instant::now();
    let mut buf = [0u8; SCAN_BYTES];
    let mut filled = 0;
    while filled < SCAN_BYTES {
        match port.read(&mut buf[filled..], wait) {
            Ok(n) => filled += n,
            Err(e) => {
                log::debug!("Read: {e}.");
                std::thread::sleep(wait);
            }
        }
        if filled < SCAN_BYTES && start.elapsed() >= timeout {
            break;
        }
    }
    let found = (filled == SCAN_BYTES).then(|| protocol::optarif_of(&buf)).flatten();
    if found.is_none() {
        log::info!("Didn't find a valid packet (read {filled} bytes).");
    }
    found
}

/// What a capture finds wrong in how long nothing came of a stream. A sample limit
/// alone needs a bound, as it may never be met.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Wait {
    /// Nothing wrong yet.
    Fine,
    /// No byte since the bound.
    NoBytes,
    /// Bytes, but no `ADCO` group since the bound.
    NoSample,
    /// An EOT, and no byte since the long bound.
    Interrupted,
}

/// How long since a stream last gave anything, as a pure state over times given to it.
#[derive(Debug)]
struct Watch {
    bound: Duration,
    interrupted_bound: Duration,
    /// When a byte last came.
    heard: Instant,
    /// When an `ADCO` group last came, or the stream came back from an interruption.
    counted: Instant,
    /// The last byte that came was an EOT.
    interrupted: bool,
}

impl Watch {
    fn new(bound: Duration, interrupted_bound: Duration, now: Instant) -> Watch {
        Watch { bound, interrupted_bound, heard: now, counted: now, interrupted: false }
    }

    /// Bytes came, whatever they were. `last` is the last of them.
    fn heard(&mut self, last: u8, now: Instant) {
        if self.interrupted {
            self.counted = now;
        }
        self.heard = now;
        self.interrupted = last == EOT;
    }

    /// An `ADCO` group came.
    fn counted(&mut self, now: Instant) {
        self.counted = now;
    }

    /// The consumer held the capture for this long, in which it read nothing: a slow
    /// one must not end it with an error about the meter.
    fn excuse(&mut self, held: Duration) {
        self.heard += held;
        self.counted += held;
    }

    /// What it comes to, at `now`.
    fn verdict(&self, now: Instant) -> Wait {
        let since = |at: Instant| now.saturating_duration_since(at);
        if self.interrupted {
            return if since(self.heard) >= self.interrupted_bound {
                Wait::Interrupted
            } else {
                Wait::Fine
            };
        }
        if since(self.heard) >= self.bound {
            Wait::NoBytes
        } else if since(self.counted) >= self.bound {
            Wait::NoSample
        } else {
            Wait::Fine
        }
    }
}

/// The software limits, which a set changes under a running capture, as in C.
#[derive(Debug, Default)]
struct Limits {
    samples: AtomicU64,
    msec: AtomicU64,
}

/// One meter.
pub struct TeleinfoDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    /// What api.c names each channel, whatever a client renames it to.
    names: &'static [&'static str],
    conn: String,
    params: Params,
    limits: Arc<Limits>,
    /// [`NO_SAMPLE_BOUND`] and [`INTERRUPTED_BOUND`], which a test shortens.
    bounds: (Duration, Duration),
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// The port, unless a capture holds it. It comes back before `End`.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    /// A capture has read from this open of the port.
    read_before: bool,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl TeleinfoDevice {
    fn new(conn: String, params: Params, optarif: Optarif, port: Option<Box<dyn Port>>) -> Self {
        let names = optarif.channels();
        let channels = names
            .iter()
            .enumerate()
            .map(|(i, name)| Channel::new(i, ChannelKind::Analog, (*name).to_owned()))
            .collect();
        TeleinfoDevice {
            info: DeviceInfo {
                driver: "teleinfo",
                vendor: "EDF".to_owned(),
                model: "Teleinfo".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels,
            names,
            conn,
            params,
            limits: Arc::new(Limits::default()),
            bounds: (NO_SAMPLE_BOUND, INTERRUPTED_BOUND),
            pending: port,
            port: Arc::new(Mutex::new(None)),
            read_before: false,
            worker: None,
        }
    }

    /// A device of this option on an already open port. No detection is run.
    pub fn from_port(optarif: Optarif, port: Box<dyn Port>) -> Self {
        let name = port.name().to_owned();
        let params = Params::parse(SERIALCOMM).expect("the parameters parse");
        Self::new(name, params, optarif, Some(port))
    }

    /// Tells a worker to stop and waits for it.
    fn join(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for TeleinfoDevice {
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

    /// C has no `config_get`: both limits may only be set.
    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Err(Error::not_supported(format!("{} cannot be read", key.id())))
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let number =
            || value.as_u64().ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())));
        match key {
            Key::LimitSamples => self.limits.samples.store(number()?, Ordering::SeqCst),
            Key::LimitMsec => self.limits.msec.store(number()?, Ordering::SeqCst),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn open(&mut self) -> Result<()> {
        let mut slot = self.port.lock().expect("port lock");
        if slot.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            // Flushed as `serial_open` flushes a port it opens.
            Some(mut p) => {
                p.purge()?;
                p
            }
            None => crate::serial::open(&self.conn, self.params)?,
        };
        *slot = Some(port);
        self.read_before = false;
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.join();
        self.port.lock().expect("port lock").take();
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        let port = self.port.lock().expect("port lock").take();
        let Some(mut port) = port else {
            // No port and a worker that lives: it holds the port. The worker gives it
            // back before its End, so a client that saw an End is never told this.
            if self.worker.as_ref().is_some_and(|w| !w.handle.is_finished()) {
                return Err(Error::Busy);
            }
            return Err(Error::device("device is not open"));
        };
        // A last capture's thread may wait to send its End to a queue nobody reads:
        // tell it, and it gives up within a bound.
        self.join();
        if self.read_before
            && let Err(e) = port.purge()
        {
            log::debug!("Flush: {e}.");
        }
        self.read_before = true;
        let capture = Capture {
            names: self.names,
            enabled: self.channels.iter().map(|c| c.enabled).collect(),
            limits: Arc::clone(&self.limits),
            bounds: self.bounds,
        };
        let slot = Arc::clone(&self.port);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("teleinfo acquisition".to_owned())
            .spawn(move || {
                let outcome = capture.run(port.as_mut(), &sink, &stop2);
                *slot.lock().expect("port lock") = Some(port);
                send_end(&sink, &stop2, outcome);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// The flag ends the capture. A close joins.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Drop for TeleinfoDevice {
    fn drop(&mut self) {
        self.join();
    }
}

/// `End`, waiting for room for as long as nobody asked to stop, and after a stop only
/// for [`END_AFTER_STOP`]: a close joins this thread.
fn send_end(sink: &PacketSink, stop: &AtomicBool, outcome: Result<()>) {
    let stopped_at = std::cell::Cell::new(None);
    sink.end_while(outcome, || {
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
    names: &'static [&'static str],
    enabled: Vec<bool>,
    limits: Arc<Limits>,
    bounds: (Duration, Duration),
}

impl Capture {
    /// Runs until a limit, a stop, a port that fails or a consumer that left.
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let send = |p: Packet| sink.send_while(p, || !stop.load(Ordering::SeqCst));
        if !send(Packet::header()) {
            return Ok(());
        }
        let started = Instant::now();
        let mut samples = 0u64;
        let mut buf = [0u8; BUF_SIZE];
        let mut held = 0;
        let mut watch = Watch::new(self.bounds.0, self.bounds.1, started);
        while !stop.load(Ordering::SeqCst) {
            let read = port.read(&mut buf[held..], READ_TIME);
            // Taken once, right after the read: what a stall or a slow consumer does
            // to the time after this is not the meter's.
            let mut seen = Instant::now();
            match read {
                Err(e) => {
                    log::error!("Serial port read error: {e}.");
                    return Err(e);
                }
                Ok(0) => std::thread::sleep(EMPTY_READ_PAUSE),
                Ok(n) => {
                    watch.heard(buf[held + n - 1], seen);
                    held += n;
                    let mut sample = false;
                    let mut at = 0;
                    while let Some((group, used)) = protocol::next_group(&buf[at..held]) {
                        if let Some(group) = group {
                            match self.group(group, &send) {
                                Handled::Nothing => {}
                                Handled::Sample => {
                                    samples = samples.saturating_add(1);
                                    sample = true;
                                }
                                Handled::SinkGone => return Ok(()),
                            }
                        }
                        at += used;
                    }
                    let parsed = Instant::now();
                    watch.excuse(parsed - seen);
                    if sample {
                        watch.counted(parsed);
                    }
                    seen = parsed;
                    buf.copy_within(at..held, 0);
                    held -= at;
                    if held >= BUF_SIZE {
                        let said = format!("{BUF_SIZE} bytes came with no group in them");
                        log::error!("{said}.");
                        return Err(Error::device(said));
                    }
                }
            }
            // Once all of a read is out, as a read may hold two frames.
            let limit = self.limits.samples.load(Ordering::SeqCst);
            if limit > 0 && samples >= limit {
                return Ok(());
            }
            let msec = self.limits.msec.load(Ordering::SeqCst);
            if msec > 0 && started.elapsed() > Duration::from_millis(msec) {
                return Ok(());
            }
            if limit > 0 && msec == 0 {
                watch_verdict(&watch, self.bounds, seen)?;
            }
        }
        Ok(())
    }

    /// `teleinfo_handle_measurement`.
    fn group(&self, group: protocol::Group<'_>, send: &impl Fn(Packet) -> bool) -> Handled {
        match protocol::meaning(group.label) {
            None => Handled::Nothing,
            Some(Meaning::Sample) => Handled::Sample,
            Some(Meaning::Reading { channel, mq, unit }) => {
                let Some(pos) = self.names.iter().position(|n| *n == channel) else {
                    return Handled::Nothing;
                };
                if !self.enabled.get(pos).copied().unwrap_or(false) {
                    return Handled::Nothing;
                }
                let data = AnalogData {
                    channels: vec![pos],
                    // C's `int` into a `float` parameter: the nearest.
                    samples: vec![protocol::atoi(group.data) as f32],
                    mq,
                    flags: MqFlags::empty(),
                    unit,
                    digits: 0,
                };
                if send(Packet::Analog(data)) { Handled::Nothing } else { Handled::SinkGone }
            }
        }
    }
}

/// What a group did.
enum Handled {
    Nothing,
    Sample,
    SinkGone,
}

/// A wait that went on too long, as the error that ends a capture.
fn watch_verdict(watch: &Watch, bounds: (Duration, Duration), now: Instant) -> Result<()> {
    let (seconds, long) = (bounds.0.as_secs(), bounds.1.as_secs());
    let said = match watch.verdict(now) {
        Wait::Fine => return Ok(()),
        Wait::NoBytes => format!("no bytes from the meter for {seconds} s, with samples to go"),
        Wait::NoSample => format!(
            "bytes come, but no ADCO group for {seconds} s, with samples to go: is the line \
             1200/7e1, and the meter's output in its historic form?"
        ),
        Wait::Interrupted => {
            format!("the meter announced an interruption and sent nothing for {long} s")
        }
    };
    log::error!("{said}.");
    Err(Error::device(said))
}
