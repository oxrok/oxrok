// Counterpart in sigrok:
//   libsigrok/src/hardware/asix-omega-rtm-cli/api.c,
//   libsigrok/src/hardware/asix-omega-rtm-cli/protocol.c,
//   libsigrok/src/hardware/asix-omega-rtm-cli/protocol.h,
//   Copyright (C) 2021 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `asix-omega-rtm-cli`: the ASIX OMEGA logic analyzer in its real-time mode,
//! through ASIX's `omegartmcli` program, which does the USB, the FPGA and the
//! device; the driver runs it and reads its stdout, 16 channels at 200 MHz with
//! run lengths. See [`protocol`] for the stream and [`child`] for the process.
//!
//! A scan runs `omegartmcli -version` and takes the version from its output; no
//! program is no device. A capture runs `omegartmcli -bin`, with `-serial SN` when
//! `conn=sn=SN` names one, waits for the first chunk, and decodes the rest until a
//! limit, a stop or the stream's end. The program is `OMEGARTMCLI`, else
//! `omegartmcli` on `PATH`; under wine that is a script that runs the `.exe`.
//!
//! What differs from C:
//!
//! - **The program is ended for sure.** C closes its pipes and leaves it: glib's
//!   spawn hands it to init, which reaps it, but nothing kills it, so one that
//!   doesn't end (the program after a USB error waits for data that never comes,
//!   and looks at neither pipe) outlives sigrok-cli. Here a close waits
//!   [`child::CLOSE_GRACE`] for it, kills its process group (a wrapper script and
//!   the program under it), and always reaps it. It runs in that group of its own,
//!   so a terminal's Ctrl-C ends oxrok's capture, which then ends the program, as
//!   a client's stop does. `-version` stays in oxrok's group.
//! - **The first chunk is awaited on the capture's thread,** so a stop reaches the
//!   wait; C waits inside its start, where nothing can stop it. The time limit
//!   starts once it is in, as in C (`api.c:375`). A program that ends or says
//!   nothing before it fails the capture rather than the start.
//! - **A stream that ends ends the capture.** C's source asks for `G_IO_IN` and
//!   `G_IO_ERR` (`api.c:369`), a closed pipe gives `G_IO_HUP`, which
//!   `omega_rtm_cli_receive_data` ignores, so sigrok-cli spins at full speed until
//!   its time limit, and with a sample limit for ever. Here the end of the stream ends the capture: with a
//!   warning when the program exited with status 0 (its own end), and failed for
//!   any other status or a signal.
//! - **Bounds where C waits for ever:** the `-version` run ([`VERSION_WAIT`]), the
//!   first chunk ([`START_WAIT`]), and a silent program ([`protocol::SILENCE_WARN`]
//!   for a warning, [`protocol::SILENCE_LIMIT`] for a failure: its bursts come every
//!   100 ms, and one silent that long is wedged after a USB error).
//! - **The first chunk can come in pieces.** C's `omega_rtm_cli_open` makes one
//!   `read()` of 6 bytes and fails the start on fewer (`protocol.c:97-106`). The
//!   program writes whole chunks, so this matters for a pipe that cuts them.
//! - **A sample limit is exact.** C submits both samples of a chunk and caps its
//!   count after (`protocol.c:231-251`), so with one sample left it sends two, one
//!   more than asked for, and sigrok-cli writes both.
//! - **The stream's end is told apart.** The program ends by itself only at its
//!   device's limit (status 0, "Gracefully closing...") or on an error (status 1);
//!   a chunk cut at the end is dropped with a warning.
//! - **`probe_names` reaches the driver from `-d`.** C takes it in `scan()`
//!   (c18f4891) but its `scanopts` list only `conn`, so sigrok-cli refuses it.
//! - Samples go out after each read, where C's feed queue holds up to 262144 of
//!   them, which a slow stream takes a long time to fill.

#![forbid(unsafe_code)]

pub mod child;
pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Value, ValueList,
};
use child::{Ended, Got, Stream};
use protocol::{CHUNK, Decoder, Fed, Queue, Silence, Verdict};
use std::ffi::OsString;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const ACCEPTED_SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::ProbeNames];
static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];

/// How long `-version` may take. Its banner is printed before the program looks
/// at any device, so this is wine's start, which the first run in a new prefix
/// makes long.
pub const VERSION_WAIT: Duration = Duration::from_secs(30);
/// How long the first chunk may take: the program opens the device and configures
/// its FPGA first, "a few seconds" by C's comment.
pub const START_WAIT: Duration = Duration::from_secs(30);
/// How long a read waits between looks at the stop and the time limit: C's
/// source's 10 ms.
const POLL: Duration = Duration::from_millis(10);
/// How long a stream's end waits for the program's status.
const EXIT_WAIT: Duration = Duration::from_secs(1);

/// How the device runs the program: the real one, or a test's fake.
pub type Opener = Arc<dyn Fn(&[&str]) -> std::io::Result<Box<dyn Stream>> + Send + Sync>;

/// The `asix-omega-rtm-cli` driver.
pub struct AsixOmegaRtmCliDriver;

impl Driver for AsixOmegaRtmCliDriver {
    fn id(&self) -> &'static str {
        "asix-omega-rtm-cli"
    }

    fn long_name(&self) -> &'static str {
        "ASIX OMEGA RTM CLI"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn accepted_scan_options(&self) -> &'static [Key] {
        &ACCEPTED_SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut names = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::ProbeNames => names = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(spec) = protocol::parse_conn(conn.as_deref()) else {
            return Ok(Vec::new());
        };
        let exe = child::program();
        log::debug!("Vendor application executable: {}", exe.to_string_lossy());
        let output = match child::run_version(&exe, VERSION_WAIT) {
            Ok(output) => output,
            Err(child::VersionError::NotFound) => {
                log::debug!("External RTM CLI execution failed.");
                return Ok(Vec::new());
            }
            Err(child::VersionError::Spawn(e)) => {
                log::error!("Cannot execute RTM CLI process: {e}");
                log::debug!("External RTM CLI execution failed.");
                return Ok(Vec::new());
            }
            Err(child::VersionError::Timeout(bound)) => {
                log::error!(
                    "The RTM CLI executable was still running {} s into -version: killed.",
                    bound.as_secs()
                );
                return Ok(Vec::new());
            }
        };
        let Some(device) = device_from(&output, spec, names.as_deref(), exe) else {
            return Ok(Vec::new());
        };
        Ok(vec![Box::new(device)])
    }
}

/// The device a `-version` output makes, with its messages: `None` is none.
fn device_from(
    output: &[u8],
    spec: protocol::Conn,
    names: Option<&str>,
    exe: OsString,
) -> Option<AsixOmegaRtmCliDevice> {
    if output.first().is_none_or(|b| *b == 0) {
        log::debug!("External RTM CLI execution failed.");
        return None;
    }
    let Some(version) = protocol::parse_version(output) else {
        log::error!("Cannot get RTM CLI executable's version.");
        return None;
    };
    log::info!("RTM CLI executable version: {version}");
    let opener: Opener = Arc::new(move |args: &[&str]| {
        child::Child::spawn(&exe, args).map(|c| Box::new(c) as Box<dyn Stream>)
    });
    Some(AsixOmegaRtmCliDevice::new(version, spec, names, opener))
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// An OMEGA behind the program.
pub struct AsixOmegaRtmCliDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    serial: Option<String>,
    opener: Opener,
    limit_samples: u64,
    limit_msec: u64,
    start_wait: Duration,
    worker: Option<Worker>,
}

impl AsixOmegaRtmCliDevice {
    /// The device of a program of `version`, with the `conn=` it was found by.
    pub fn new(version: String, spec: protocol::Conn, names: Option<&str>, opener: Opener) -> Self {
        let names = match names {
            Some(spec) => crate::core::util::parse_probe_names(spec, &protocol::CHANNEL_NAMES),
            None => protocol::CHANNEL_NAMES.iter().map(|s| (*s).to_owned()).collect(),
        };
        AsixOmegaRtmCliDevice {
            info: DeviceInfo {
                driver: "asix-omega-rtm-cli",
                vendor: "ASIX".to_owned(),
                model: "OMEGA RTM CLI".to_owned(),
                version,
                serial: spec.serial.clone().unwrap_or_default(),
                conn: spec.conn,
            },
            channels: names
                .into_iter()
                .enumerate()
                .map(|(i, n)| Channel::new(i, ChannelKind::Logic, n))
                .collect(),
            serial: spec.serial,
            opener,
            limit_samples: 0,
            limit_msec: 0,
            start_wait: START_WAIT,
            worker: None,
        }
    }

    /// The bound on the first chunk, for tests.
    pub fn set_start_wait(&mut self, wait: Duration) {
        self.start_wait = wait;
    }

    fn end_worker(&mut self) {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
    }

    /// The program started, as `omega_rtm_cli_open` starts it; its first chunk is
    /// [`first_chunk`]'s, on the worker.
    fn spawn(&self) -> Result<Box<dyn Stream>> {
        let mut args = vec!["-bin"];
        if let Some(s) = &self.serial {
            args.extend(["-serial", s.as_str()]);
        }
        log::debug!("Starting vendor application");
        (self.opener)(&args).map_err(|e| {
            log::error!("Cannot execute RTM CLI process: {e}");
            log::error!("Vendor application start failed.");
            Error::Io(e)
        })
    }
}

/// What the wait for the first chunk came to.
#[derive(Debug)]
pub enum First {
    /// The chunk, and what came after it in the same reads.
    Chunk([u8; CHUNK], Vec<u8>),
    /// A stop came first.
    Stopped,
    /// The program ended or said nothing for `wait`.
    Failed(Error),
}

/// The rest of `omega_rtm_cli_open`: its first chunk, within `wait`, where C's
/// one `read()` has no bound and takes no fewer than 6 bytes. A stop ends it.
pub fn first_chunk(stream: &mut dyn Stream, wait: Duration, stop: &AtomicBool) -> First {
    let began = Instant::now();
    let mut got = Vec::new();
    while got.len() < CHUNK {
        if stop.load(Ordering::SeqCst) {
            return First::Stopped;
        }
        if began.elapsed() >= wait {
            let text =
                format!("No first chunk from the vendor application within {} s.", wait.as_secs());
            log::error!("{text}");
            return First::Failed(Error::device(text));
        }
        match stream.read(POLL) {
            Got::Data(d) => {
                log::debug!("Read from vendor application, ret {}", d.len());
                got.extend_from_slice(&d);
            }
            Got::Nothing => {}
            Got::End(why) => {
                let status = stream.exited(EXIT_WAIT);
                let text = format!(
                    "The vendor application ended before its first chunk, at {} of {CHUNK} \
                     bytes{}.",
                    got.len(),
                    how_it_ended(why.as_deref(), status.as_ref())
                );
                log::error!("{text}");
                return First::Failed(Error::device(text));
            }
        }
    }
    let mut first = [0u8; CHUNK];
    first.copy_from_slice(&got[..CHUNK]);
    log::debug!(
        "stamp {}, samples {:x} {:x}",
        u16::from_le_bytes([first[0], first[1]]),
        u16::from_le_bytes([first[2], first[3]]),
        u16::from_le_bytes([first[4], first[5]])
    );
    First::Chunk(first, got.split_off(CHUNK))
}

/// The worker: the first chunk, then the capture, whose time limit starts after
/// it, as `sr_sw_limits_acquisition_start` comes after `omega_rtm_cli_open` in C
/// (`api.c:375`).
fn work(
    mut stream: Box<dyn Stream>,
    plan: (u64, Option<Duration>, Duration),
    sink: &PacketSink,
    stop: &AtomicBool,
) {
    let (limit_samples, limit, wait) = plan;
    let outcome = match first_chunk(stream.as_mut(), wait, stop) {
        First::Chunk(first, rest) => {
            let capture = Capture {
                decoder: Decoder::new(first, limit_samples),
                limit,
                began: Instant::now(),
                silence: Silence::new(Instant::now()),
            };
            return capture.run(stream, &rest, sink, stop);
        }
        First::Stopped => Ok(()),
        First::Failed(e) => Err(e),
    };
    close_logged(stream);
    sink.end_while(outcome, || !stop.load(Ordering::SeqCst));
}

/// `: exit status: 1`, `: signal: 9 (SIGKILL)`, a read's error, or nothing.
fn how_it_ended(why: Option<&str>, status: Option<&child::Exit>) -> String {
    match (why, status) {
        (Some(e), _) => format!(" (a read failed: {e})"),
        (None, Some(s)) => format!(" ({})", s.text),
        (None, None) => " (its output closed, and it still runs)".to_owned(),
    }
}

/// Ends the program, as `omega_rtm_cli_close`, and says how.
fn close_logged(stream: Box<dyn Stream>) {
    log::debug!("Closing vendor application.");
    match stream.close() {
        Ended::Exited(s) => log::debug!("The vendor application ended: {}.", s.text),
        Ended::Killed => {}
    }
}

impl Drop for AsixOmegaRtmCliDevice {
    fn drop(&mut self) {
        self.end_worker();
    }
}

impl Device for AsixOmegaRtmCliDevice {
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
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        c.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        name.clone_into(&mut c.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        // C lists conn as GET and answers SR_ERR_NA without one: a listing that
        // shows it, and a get that says "not applicable".
        let conn = if self.info.conn.is_some() { Access::GET } else { Access::NONE };
        Ok(vec![
            OptionDesc::new(Key::LimitMsec, Access::GET_SET),
            OptionDesc::new(Key::LimitSamples, Access::GET_SET),
            OptionDesc::new(Key::Conn, conn),
            OptionDesc::new(Key::Samplerate, Access { get: true, set: false, list: true }),
        ])
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::LimitMsec => Ok(Value::U64(self.limit_msec)),
            Key::LimitSamples => Ok(Value::U64(self.limit_samples)),
            Key::Samplerate => Ok(Value::U64(protocol::SAMPLERATE)),
            Key::Conn => match &self.info.conn {
                Some(c) => Ok(Value::Str(c.clone())),
                None => Err(Error::not_supported("not applicable")),
            },
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let number =
            || value.as_u64().ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())));
        match key {
            Key::LimitMsec => self.limit_msec = number()?,
            Key::LimitSamples => self.limit_samples = number()?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::Samplerate => Ok(ValueList::U64(vec![protocol::SAMPLERATE])),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn close(&mut self) -> Result<()> {
        self.end_worker();
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(w) = &self.worker {
            if !w.handle.is_finished() {
                log::error!("Vendor application already running.");
                return Err(Error::Busy);
            }
            self.end_worker();
        }
        // C waits for the first chunk inside its start (`omega_rtm_cli_open`), with
        // no bound; here the worker waits for it, so that a stop reaches the wait.
        let stream = self.spawn()?;
        let limit = (self.limit_msec > 0).then(|| Duration::from_millis(self.limit_msec));
        let plan = (self.limit_samples, limit, self.start_wait);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("asix-omega-rtm-cli acquisition".into())
            .spawn(move || work(stream, plan, &sink, &stop2))
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        self.end_worker();
        Ok(())
    }
}

/// `omega_rtm_cli_receive_data` and the stop, on a thread.
pub struct Capture {
    /// The stream's decoder, after the first chunk.
    pub decoder: Decoder,
    /// The time limit.
    pub limit: Option<Duration>,
    /// When the time limit began: after the first chunk, as
    /// `sr_sw_limits_acquisition_start` in C.
    pub began: Instant,
    /// The bound on a silent program.
    pub silence: Silence,
}

impl Capture {
    /// Runs the capture to its end, then ends the program and the sink.
    pub fn run(
        mut self,
        mut stream: Box<dyn Stream>,
        rest: &[u8],
        sink: &PacketSink,
        stop: &AtomicBool,
    ) {
        let go_on = || !stop.load(Ordering::SeqCst);
        let outcome = if sink.send_while(Packet::header(), go_on) {
            self.read(stream.as_mut(), rest, sink, stop)
        } else {
            Ok(())
        };
        if self.decoder.zero_stamps > 0 {
            log::warn!(
                "{} chunks had a time of 0, which the vendor application never sends: \
                 taken as 1.",
                self.decoder.zero_stamps
            );
        }
        close_logged(stream);
        sink.end_while(outcome, go_on);
    }

    fn read(
        &mut self,
        stream: &mut dyn Stream,
        rest: &[u8],
        sink: &PacketSink,
        stop: &AtomicBool,
    ) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        let mut queue = Queue::default();
        let mut send = |data: Vec<u8>| sink.send_while(Packet::logic(2, data), go_on);
        let mut fed = self.decoder.feed(rest, &mut queue, &mut send);
        while fed == Fed::More {
            if !queue.flush(&mut send) || stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            if self.limit.is_some_and(|l| self.began.elapsed() >= l) {
                break;
            }
            let looked = Instant::now();
            match stream.read(POLL) {
                Got::Data(d) => {
                    if let Some(text) = self.silence.got(Instant::now()) {
                        log::warn!("{text}");
                    }
                    fed = self.decoder.feed(&d, &mut queue, &mut send);
                }
                Got::Nothing => match self.silence.empty(looked) {
                    Verdict::Wait => {}
                    Verdict::Warn(text) => log::warn!("{text}"),
                    Verdict::Fail(text) => {
                        log::error!("{text}");
                        queue.flush(&mut send);
                        return Err(Error::device(text));
                    }
                },
                Got::End(why) => {
                    queue.flush(&mut send);
                    return self.ended(stream, why.as_deref());
                }
            }
        }
        queue.flush(&mut send);
        Ok(())
    }

    /// The stream ended: the program's own end with status 0, else a failure.
    fn ended(&self, stream: &mut dyn Stream, why: Option<&str>) -> Result<()> {
        let status = stream.exited(EXIT_WAIT);
        if self.decoder.leftover() > 0 {
            log::warn!(
                "The stream ended inside a chunk: {} of its {CHUNK} bytes dropped.",
                self.decoder.leftover()
            );
        }
        if why.is_none() && status.as_ref().is_some_and(|s| s.success) {
            log::warn!(
                "The vendor application ended its stream ({}), after {} samples.",
                status.map(|s| s.text).unwrap_or_default(),
                self.decoder.samples
            );
            return Ok(());
        }
        let text = format!(
            "The vendor application ended the stream{}, after {} samples.",
            how_it_ended(why, status.as_ref()),
            self.decoder.samples
        );
        log::error!("{text}");
        Err(Error::device(text))
    }
}

#[cfg(test)]
mod tests;
