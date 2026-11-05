// Counterpart in sigrok:
//   libsigrok/src/hardware/atten-pps3xxx/api.c,
//   libsigrok/src/hardware/atten-pps3xxx/protocol.c,
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Martin Ling,
//   Gerhard Sittig, Soeren Apel, Aurelien Jacobs, Daniel Elstner,
//   Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Atten PPS3203T-3S, a three channel power supply on a serial line
//! (USB through an FT232R, or RS232).
//!
//! `conn=` names the port and `serialcomm=` overrides 9600/8n2. A scan asks
//! once and takes the answer as the state; a setpoint cannot be read back.
//!
//! Every frame that sets something sets everything, so `set` only queues:
//! the frame goes out at the close, or as the next poll of a capture. An
//! output therefore never comes on ahead of its setpoints, whatever the
//! order of the options. The other side of that: what no option named goes
//! out as 0 and off. A capture is the same: its polls are such frames, there
//! being nothing else to get the second reading with, so **a capture turns
//! off and zeroes all that was not set on its command line**. C does this
//! too, once it gets as far as a poll, which sigrok-cli does only with its
//! stdin held open: at an EOF there it stops before the first one. A warning
//! says what the frame changes from what the device reported, and what it
//! writes. `pps-tools` polls with such frames as well, but its setpoints come
//! from a user's file, which names each channel's.
//!
//! As in C, only a change from what this run set makes a frame (`api.c:250,
//! 257,287`), so `enabled=off` alone on an output that is on sends nothing and
//! it stays on. The close says so.
//!
//! A capture has no limit, as in C, and goes on until it is stopped. It ends
//! too when the device stays silent for [`SILENCE`]: C never ends there, even
//! when stopped.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::serial::{Params, Port, read_blocking};
use protocol::{CHANNELS, CURRENT, FRAME, MODES, Range, Reported, Settings, VOLTAGE};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::PowerSupply];
const SERIALCOMM: &str = "9600/8n2";
const GROUPS: [&str; CHANNELS] = ["CH1", "CH2", "CH3"];
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::ChannelConfig, Access::GET_SET_LIST),
    OptionDesc::new(Key::OverCurrentProtectionEnabled, Access::GET_SET),
];
static GROUP_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::VoltageTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::Enabled, Access::GET_SET),
];

/// How long a scan gives the answer, which it reads only then, as `scan`.
/// Anything the device sends after its 24 bytes has this time to end too.
const SCAN_WAIT: Duration = Duration::from_millis(300);
/// How long a read keeps the state locked and a stop unseen.
const READ_SLICE: Duration = Duration::from_millis(20);
/// No byte for this long ends a capture. An answer takes 28 ms on the line
/// and the scan's own wait is 300 ms, so this is no slow answer. C has no
/// bound and waits for ever (`protocol.c:158-172`).
pub const SILENCE: Duration = Duration::from_secs(2);
/// A stop waits this long for the answer on its way, whose reading still
/// counts, as in C; a device that went silent would hold C for ever.
pub const STOP_WAIT: Duration = SCAN_WAIT;

/// The `atten-pps3203` driver.
pub struct AttenPpsDriver;

impl Driver for AttenPpsDriver {
    fn id(&self) -> &'static str {
        "atten-pps3203"
    }

    fn long_name(&self) -> &'static str {
        "Atten PPS3203T-3S"
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
        // Every failure of the port here is no device, as `scan` returns
        // NULL for each.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        Ok(match probe(port) {
            Some(reported) => vec![Box::new(AttenPpsDevice::new(conn, params, reported))],
            None => Vec::new(),
        })
    }
}

/// `scan` after the open. The port goes when this returns, where C leaves it
/// open on each way out but the one that found something.
fn probe(mut port: Box<dyn Port>) -> Option<Reported> {
    if let Err(e) = port.write(&protocol::ask()) {
        log::error!("Unable to write while probing for hardware: {e}");
        return None;
    }
    std::thread::sleep(SCAN_WAIT);
    let mut answer = [0; FRAME];
    let got = match read_blocking(port.as_mut(), &mut answer, Duration::ZERO) {
        Ok(got) => got,
        Err(e) => {
            log::error!("Unable to read while probing for hardware: {e}");
            return None;
        }
    };
    if !protocol::is_scan_answer(&answer[..got]) {
        log::debug!("{got} bytes, which do not look like an Atten PPS.");
        return None;
    }
    let mut reported = Reported::default();
    if protocol::is_answer(&answer) {
        reported.update(&answer);
    } else {
        // Its header is good, so it is a device; C finds one here too.
        log::warn!("The scan's answer holds readings no supply could give: {answer:02x?}.");
    }
    Some(reported)
}

/// What the capture and the frontend share.
struct Shared {
    port: Option<Box<dyn Port>>,
    settings: Settings,
    reported: Reported,
    /// Settings no frame has carried yet.
    dirty: bool,
    warned: bool,
    /// What to say at the close if no frame has gone out by then: settings
    /// that made none, though the device reports otherwise.
    unsent: Vec<String>,
}

impl Shared {
    /// What the next set frame does to what the device reports, once per
    /// capture, or once at a close.
    fn warn_of_what_it_turns_off(&mut self) {
        if self.warned {
            return;
        }
        if let Some(warning) = frame_warning(&self.reported, &self.settings) {
            self.warned = true;
            log::warn!("{warning}");
        }
    }

    /// Sends a frame, with what is still unread dropped first: a late or an
    /// extra byte would shift all the frames after it.
    fn send(&mut self, frame: &[u8; FRAME]) -> Result<()> {
        let port = self.port.as_mut().ok_or_else(|| Error::device("device is not open"))?;
        port.purge()?;
        port.write(frame)
    }

    /// `send_config`.
    fn send_settings(&mut self) -> Result<()> {
        self.warn_of_what_it_turns_off();
        self.send(&self.settings.frame())?;
        self.dirty = false;
        self.unsent.clear();
        Ok(())
    }
}

/// What a frame of `settings` changes from what `reported` holds, and the
/// setpoints it writes on a device with an output on, which it cannot report.
/// `None` when it changes nothing that shows and no output is on.
fn frame_warning(reported: &Reported, settings: &Settings) -> Option<String> {
    let on_off = |on: bool| if on { "on" } else { "off" };
    let mut changes = Vec::new();
    let turned: Vec<&str> = (0..CHANNELS)
        .filter(|&i| reported.enabled[i] && !settings.channels[i].enabled)
        .map(|i| GROUPS[i])
        .collect();
    if !turned.is_empty() {
        changes.push(format!("{} from on to off", turned.join(", ")));
    }
    if settings.ocp != reported.ocp {
        changes.push(format!(
            "ocp_enabled from {} to {}",
            on_off(reported.ocp),
            on_off(settings.ocp)
        ));
    }
    if settings.mode != reported.mode {
        changes.push(format!(
            "channel_config from {} to {}",
            MODES[usize::from(reported.mode)],
            MODES[usize::from(settings.mode)]
        ));
    }
    if changes.is_empty() && !reported.enabled.contains(&true) {
        return None;
    }
    let setpoints: Vec<String> = settings
        .channels
        .iter()
        .zip(GROUPS)
        .map(|(c, name)| format!("{name} {:.2} V {:.3} A", c.voltage, c.current))
        .collect();
    let changes =
        if changes.is_empty() { String::new() } else { format!("{}. ", changes.join(", ")) };
    Some(format!(
        "{changes}A frame sets everything and the device cannot report its setpoints, so what \
         this run did not set goes to 0: {}.",
        setpoints.join(", ")
    ))
}

/// The one supply.
pub struct AttenPpsDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    conn: String,
    params: Params,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    shared: Arc<Mutex<Shared>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

fn group_index(group: &str) -> Result<usize> {
    GROUPS.iter().position(|g| *g == group).ok_or_else(|| Error::ChannelGroup(group.to_owned()))
}

/// A value within `range`. A NaN passes `api.c`'s two comparisons
/// (`api.c:273,280`), and goes out as whatever x86 makes of it; here it is
/// refused. A -0.0 is 0.
fn in_range(key: &Key, value: &Value, range: &Range) -> Result<f64> {
    match value.as_f64() {
        Some(v) if (range[0]..=range[1]).contains(&v) => Ok(v + 0.0),
        _ => Err(Error::Arg(format!("{} out of range", key.id()))),
    }
}

impl AttenPpsDevice {
    fn new(conn: String, params: Params, reported: Reported) -> Self {
        let channels: Vec<Channel> = GROUPS
            .iter()
            .enumerate()
            .map(|(i, name)| Channel::new(i, ChannelKind::Analog, *name))
            .collect();
        AttenPpsDevice {
            info: DeviceInfo {
                driver: "atten-pps3203",
                vendor: "Atten".to_owned(),
                model: "PPS3203T-3S".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels,
            groups: GROUPS
                .iter()
                .enumerate()
                .map(|(i, name)| ChannelGroup::new(*name, vec![i]))
                .collect(),
            conn,
            params,
            pending: None,
            shared: Arc::new(Mutex::new(Shared {
                port: None,
                settings: Settings::default(),
                reported,
                dirty: false,
                warned: false,
                unsent: Vec::new(),
            })),
            worker: None,
        }
    }

    /// A device on a port that stands in for the hardware, as a scan that
    /// got `reported` would make it. [`Device::open`] takes the port.
    pub fn from_port(port: Box<dyn Port>, reported: Reported) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let mut device = Self::new(port.name().to_owned(), params, reported);
        device.pending = Some(port);
        device
    }

    fn shared(&self) -> std::sync::MutexGuard<'_, Shared> {
        self.shared.lock().expect("state lock")
    }

    fn set_global(&mut self, key: &Key, value: &Value) -> Result<()> {
        let mut s = self.shared();
        match key {
            Key::ChannelConfig => {
                let name = value.as_str().ok_or_else(|| Error::arg("channel_config"))?;
                let mode = MODES
                    .iter()
                    .position(|m| *m == name)
                    .ok_or_else(|| Error::Arg(format!("channel_config has no mode '{name}'")))?;
                let mode = u8::try_from(mode).expect("three modes");
                // As `api.c:250`: only a change from what this run set makes a
                // frame, whatever the device says.
                if mode != s.settings.mode {
                    s.settings.mode = mode;
                    s.dirty = true;
                } else if mode != s.reported.mode {
                    let was = MODES[usize::from(s.reported.mode)];
                    s.unsent.push(format!(
                        "channel_config={name} sent nothing: the device reports {was}, and it \
                         stays so."
                    ));
                }
            }
            Key::OverCurrentProtectionEnabled => {
                let on = value.as_bool().ok_or_else(|| Error::arg("ocp_enabled"))?;
                // As for the mode (`api.c:257`).
                if on != s.settings.ocp {
                    s.settings.ocp = on;
                    s.dirty = true;
                } else if on != s.reported.ocp {
                    let (asked, was) = if on { ("on", "off") } else { ("off", "on") };
                    s.unsent.push(format!(
                        "ocp_enabled={asked} sent nothing: the device reports it {was}, and it \
                         stays {was}."
                    ));
                }
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn set_channel(&mut self, channel: usize, key: &Key, value: &Value) -> Result<()> {
        let mut s = self.shared();
        match key {
            Key::VoltageTarget => {
                s.settings.channels[channel].voltage = in_range(key, value, &VOLTAGE[channel])?;
                s.dirty = true;
            }
            Key::CurrentLimit => {
                s.settings.channels[channel].current = in_range(key, value, &CURRENT[channel])?;
                s.dirty = true;
            }
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(|| Error::arg("enabled"))?;
                // As `api.c:287`, so a supply that is on stays on for
                // `enabled=off` alone, and the close says so.
                if on != s.settings.channels[channel].enabled {
                    s.settings.channels[channel].enabled = on;
                    s.dirty = true;
                } else if !on && s.reported.enabled[channel] {
                    s.unsent.push(format!(
                        "enabled=off on {} sent nothing: the device reports the output on, and \
                         IT STAYS ON. A setpoint with it sends a frame, which sets everything.",
                        GROUPS[channel]
                    ));
                }
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }
}

impl Device for AttenPpsDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &self.groups
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
            Some(g) => group_index(g).map(|_| GROUP_OPTIONS.to_vec()),
        }
    }

    /// What was set, or what the device said last. C says nothing until a
    /// capture's first answer: 0 and off, whatever the output is doing. Here
    /// the scan's answer counts.
    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let s = self.shared();
        let Some(group) = group else {
            return match key {
                Key::ChannelConfig => Ok(Value::from(MODES[usize::from(s.reported.mode)])),
                Key::OverCurrentProtectionEnabled => Ok(Value::Bool(s.reported.ocp)),
                other => Err(Error::UnknownKey(other.id().to_owned())),
            };
        };
        let i = group_index(group)?;
        Ok(match key {
            Key::Voltage => Value::F64(f64::from(s.reported.voltage[i])),
            Key::VoltageTarget => Value::F64(s.settings.channels[i].voltage),
            Key::Current => Value::F64(f64::from(s.reported.current[i])),
            Key::CurrentLimit => Value::F64(s.settings.channels[i].current),
            Key::Enabled => Value::Bool(s.reported.enabled[i]),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        match group {
            None => self.set_global(key, value),
            Some(g) => {
                let channel = group_index(g)?;
                self.set_channel(channel, key, value)
            }
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let Some(group) = group else {
            return match key {
                Key::ChannelConfig => {
                    Ok(ValueList::Str(MODES.iter().map(|m| (*m).to_owned()).collect()))
                }
                other => Err(Error::UnknownKey(other.id().to_owned())),
            };
        };
        let i = group_index(group)?;
        let range = match key {
            Key::VoltageTarget => VOLTAGE[i],
            Key::CurrentLimit => CURRENT[i],
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        };
        Ok(ValueList::RangeStepF64 { min: range[0], max: range[1], step: range[2] })
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        if self.shared().port.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&self.conn, self.params)?,
        };
        self.shared().port = Some(port);
        Ok(())
    }

    /// `dev_close`: what is queued goes out. A write that fails is an error
    /// here, where C logs it at debug level and a setting is lost unsaid
    /// (`protocol.c:102-103`).
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let mut s = self.shared();
        let sent = if s.dirty && s.port.is_some() { s.send_settings() } else { Ok(()) };
        for notice in std::mem::take(&mut s.unsent) {
            log::warn!("{notice}");
        }
        s.port = None;
        sent
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(worker) = &self.worker {
            if !worker.handle.is_finished() {
                return Err(Error::Busy);
            }
            let worker = self.worker.take().expect("just seen");
            let _ = worker.handle.join();
        }
        {
            let mut s = self.shared();
            if s.port.is_none() {
                return Err(Error::device("device is not open"));
            }
            // A library session's next capture warns afresh.
            s.warned = false;
        }
        let shared = Arc::clone(&self.shared);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("atten-pps3xxx acquisition".to_owned())
            .spawn(move || {
                let outcome = run_capture(&shared, &sink, &stop2);
                sink.end(outcome);
            })
            .map_err(Error::Io)?;
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

enum Read {
    Frame([u8; FRAME]),
    Stopped,
    Silent,
    Failed(Error),
}

/// Collects one answer, however it comes, and looks for a stop between
/// pieces. The lock goes between them too, for `set` and `get`.
fn read_frame(shared: &Mutex<Shared>, stop: &AtomicBool) -> Read {
    let mut frame = [0; FRAME];
    let mut got = 0;
    let mut last_byte = Instant::now();
    let mut stop_seen: Option<Instant> = None;
    loop {
        {
            let mut s = shared.lock().expect("state lock");
            let Some(port) = s.port.as_mut() else {
                return Read::Failed(Error::device("device is not open"));
            };
            match read_blocking(port.as_mut(), &mut frame[got..], READ_SLICE) {
                Ok(0) => {}
                Ok(n) => {
                    got += n;
                    last_byte = Instant::now();
                }
                Err(e) => return Read::Failed(e),
            }
        }
        if got == FRAME {
            return Read::Frame(frame);
        }
        if stop.load(Ordering::SeqCst) {
            let seen = *stop_seen.get_or_insert_with(Instant::now);
            if seen.max(last_byte).elapsed() >= STOP_WAIT {
                return Read::Stopped;
            }
        }
        if last_byte.elapsed() >= SILENCE {
            return Read::Silent;
        }
    }
}

/// `dev_acquisition_start` and `atten_pps3xxx_receive_data`: asks once, then
/// each answer gets its readings out and the settings sent back, which is
/// the next request. After a stop, an answer ends it with nothing sent.
fn run_capture(shared: &Mutex<Shared>, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
    sink.send(Packet::header());
    if let Err(e) = shared.lock().expect("state lock").send(&protocol::ask()) {
        log::error!("Acquisition failed: {e}.");
        return Err(e);
    }
    loop {
        let frame = match read_frame(shared, stop) {
            Read::Frame(frame) => frame,
            Read::Stopped => return Ok(()),
            Read::Silent => {
                log::error!("Acquisition failed: no answer from the device for {SILENCE:?}.");
                return Err(Error::device(format!("no answer from the device for {SILENCE:?}")));
            }
            Read::Failed(e) => {
                log::error!("Acquisition failed: {e}.");
                return Err(e);
            }
        };
        if protocol::is_answer(&frame) {
            let reported = {
                let mut s = shared.lock().expect("state lock");
                s.reported.update(&frame);
                s.reported
            };
            if !send_readings(&reported, sink, stop) {
                return Ok(());
            }
        } else {
            log::warn!("An answer no supply could give dropped: {frame:02x?}.");
        }
        if stop.load(Ordering::SeqCst) {
            return Ok(());
        }
        if let Err(e) = shared.lock().expect("state lock").send_settings() {
            log::error!("Acquisition failed: {e}.");
            return Err(e);
        }
    }
}

/// `handle_packet`: all channels at once, volts and then amperes.
fn send_readings(reported: &Reported, sink: &PacketSink, stop: &AtomicBool) -> bool {
    let go_on = || !stop.load(Ordering::SeqCst);
    let block = |samples: [f32; CHANNELS], mq, flags, unit, digits| {
        Packet::Analog(AnalogData {
            channels: (0..CHANNELS).collect(),
            samples: samples.to_vec(),
            mq,
            flags,
            unit,
            digits,
        })
    };
    sink.send_while(block(reported.voltage, Mq::Voltage, MqFlags::DC, Unit::Volt, 2), go_on)
        && sink.send_while(
            block(reported.current, Mq::Current, MqFlags::empty(), Unit::Ampere, 3),
            go_on,
        )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits for a capture: a stop or a bound that does not
    /// bite must fail, not hang the run.
    const CAPTURE_BOUND: Duration = Duration::from_secs(10);

    #[derive(Default)]
    struct Seen {
        writes: Vec<(Instant, Vec<u8>)>,
        purges: usize,
    }

    /// What a fake says to a write: bytes after a pause, in pieces.
    type Answer = Vec<(Duration, Vec<u8>)>;

    /// A supply that answers each frame with `reading`, and notes each write
    /// and when. A byte is there from its time on, so a purge takes what has
    /// come and no more, as a tty's input queue.
    struct Fake {
        seen: Arc<Mutex<Seen>>,
        input: VecDeque<(Instant, u8)>,
        fail_writes_after: Option<usize>,
        fail_reads: bool,
        scripted: Vec<(usize, Answer)>,
        /// Answers after this many writes: none.
        silent_after: Option<usize>,
    }

    fn reading() -> Vec<u8> {
        let mut f = vec![0; FRAME];
        f[0] = 0xaa;
        f[1] = 0xaa;
        f[2..14].copy_from_slice(&[
            0x04, 0xd2, 0x01, 0xf4, 0x01, 0xf4, 0x04, 0xe2, 0x01, 0x4a, 0x00, 0x19,
        ]);
        f[14] = 1;
        f[15] = 0b011;
        f[16] = 1;
        f[18] = 1;
        f[19] = 1;
        f
    }

    impl Fake {
        fn new() -> (Fake, Arc<Mutex<Seen>>) {
            let seen = Arc::new(Mutex::new(Seen::default()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                fail_writes_after: None,
                fail_reads: false,
                scripted: Vec::new(),
                silent_after: None,
            };
            (fake, seen)
        }

        fn arrived(&self, now: Instant) -> usize {
            self.input.iter().take_while(|(at, _)| *at <= now).count()
        }
    }

    impl Port for Fake {
        fn name(&self) -> &'static str {
            "/dev/ttyFAKE"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            if self.fail_reads {
                return Err(Error::device("Input/output error"));
            }
            let deadline = Instant::now() + timeout;
            loop {
                let now = Instant::now();
                let n = self.arrived(now).min(buf.len());
                if n > 0 {
                    for b in &mut buf[..n] {
                        *b = self.input.pop_front().expect("counted").1;
                    }
                    return Ok(n);
                }
                if now >= deadline {
                    return Ok(0);
                }
                let next = self.input.front().map_or(deadline, |(at, _)| *at);
                std::thread::sleep(next.min(deadline).saturating_duration_since(now));
            }
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let now = Instant::now();
            let number = {
                let mut seen = self.seen.lock().unwrap();
                if self.fail_writes_after.is_some_and(|n| seen.writes.len() >= n) {
                    return Err(Error::device("Broken pipe"));
                }
                seen.writes.push((now, data.to_vec()));
                seen.writes.len() - 1
            };
            let silent = self.silent_after.is_some_and(|n| number >= n);
            let answer = match self.scripted.iter().find(|(n, _)| *n == number) {
                Some((_, answer)) => answer.clone(),
                None if silent => Answer::new(),
                None => vec![(Duration::ZERO, reading())],
            };
            let mut at = self.input.back().map_or(now, |(last, _)| (*last).max(now));
            for (pause, bytes) in answer {
                at += pause;
                self.input.extend(bytes.into_iter().map(|b| (at, b)));
            }
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.seen.lock().unwrap().purges += 1;
            let n = self.arrived(Instant::now());
            self.input.drain(..n);
            Ok(())
        }
    }

    fn device(fake: Fake) -> AttenPpsDevice {
        let mut d = AttenPpsDevice::from_port(Box::new(fake), Reported::default());
        d.open().unwrap();
        d
    }

    fn device_of(fake: Fake, frame: &[u8]) -> AttenPpsDevice {
        let mut reported = Reported::default();
        reported.update(&<[u8; FRAME]>::try_from(frame).unwrap());
        let mut d = AttenPpsDevice::from_port(Box::new(fake), reported);
        d.open().unwrap();
        d
    }

    fn writes(seen: &Arc<Mutex<Seen>>) -> Vec<Vec<u8>> {
        seen.lock().unwrap().writes.iter().map(|(_, w)| w.clone()).collect()
    }

    #[allow(clippy::needless_pass_by_value)]
    fn set(d: &mut AttenPpsDevice, group: &str, key: Key, value: Value) -> Result<()> {
        d.set(&key, &value, Some(group))
    }

    type Received = Receiver<(usize, Packet)>;

    fn start(d: &mut AttenPpsDevice) -> Received {
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        rx
    }

    fn until(rx: &Received, done: fn(&Packet) -> bool, bound: Duration) -> Vec<Packet> {
        let deadline = Instant::now() + bound;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let (_, p) = rx.recv_timeout(left).expect("the packet, in time");
            let end = done(&p);
            out.push(p);
            if end {
                return out;
            }
        }
    }

    fn is_end(p: &Packet) -> bool {
        matches!(p, Packet::End)
    }

    fn is_analog(p: &Packet) -> bool {
        matches!(p, Packet::Analog(_))
    }

    fn blocks(packets: &[Packet]) -> Vec<Vec<f32>> {
        packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some(a.samples.clone()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn a_scan_asks_once_waits_and_takes_the_answer_as_the_state() {
        let (fake, seen) = Fake::new();
        let begun = Instant::now();
        let reported = probe(Box::new(fake)).expect("a device");
        assert!(begun.elapsed() >= SCAN_WAIT);
        assert_eq!(writes(&seen), [protocol::ask().to_vec()]);
        assert_eq!(reported.voltage[0], 12.34);
        assert_eq!(reported.enabled, [true, true, false]);
        assert_eq!(reported.mode, 1);
    }

    #[test]
    fn a_scan_takes_an_answer_only_by_its_time() {
        let (mut fake, _seen) = Fake::new();
        fake.scripted = vec![(0, vec![(SCAN_WAIT + Duration::from_millis(100), reading())])];
        assert!(probe(Box::new(fake)).is_none());
    }

    #[test]
    fn a_scan_refuses_what_is_no_answer() {
        let mut no_header = reading();
        no_header[0] = 0x55;
        let mut other = reading();
        other[1] = 0x20;
        for answer in [vec![], reading()[..23].to_vec(), no_header, other] {
            let (mut fake, _seen) = Fake::new();
            fake.scripted = vec![(0, vec![(Duration::ZERO, answer)])];
            assert!(probe(Box::new(fake)).is_none());
        }
    }

    #[test]
    fn a_scan_on_a_port_that_fails_finds_nothing() {
        let (mut fake, _seen) = Fake::new();
        fake.fail_writes_after = Some(0);
        assert!(probe(Box::new(fake)).is_none());
        let (mut fake, _seen) = Fake::new();
        fake.fail_reads = true;
        assert!(probe(Box::new(fake)).is_none());
    }

    #[test]
    fn nothing_goes_out_until_the_close_and_then_one_frame() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        set(&mut d, "CH1", Key::Enabled, Value::Bool(true)).unwrap();
        set(&mut d, "CH1", Key::VoltageTarget, Value::F64(5.0)).unwrap();
        set(&mut d, "CH3", Key::CurrentLimit, Value::F64(0.5)).unwrap();
        d.set(&Key::OverCurrentProtectionEnabled, &Value::Bool(true), None).unwrap();
        d.set(&Key::ChannelConfig, &Value::from("Parallel"), None).unwrap();
        assert_eq!(writes(&seen).len(), 0, "the output's bit and its voltage go together");
        d.close().unwrap();
        let w = writes(&seen);
        assert_eq!(w.len(), 1);
        assert_eq!(w[0][2..4], [0x01, 0xf4]);
        assert_eq!(w[0][12..14], [0x01, 0xf4]);
        assert_eq!((w[0][15], w[0][18], w[0][19]), (1, 1, 2));
    }

    #[test]
    fn a_close_with_nothing_queued_sends_nothing() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        d.close().unwrap();
        assert_eq!(writes(&seen).len(), 0);
        assert_eq!(seen.lock().unwrap().purges, 1, "flushed once, as serial_open");
    }

    #[test]
    fn a_setting_that_changes_nothing_sends_nothing() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        set(&mut d, "CH1", Key::Enabled, Value::Bool(false)).unwrap();
        d.set(&Key::OverCurrentProtectionEnabled, &Value::Bool(false), None).unwrap();
        d.set(&Key::ChannelConfig, &Value::from("Independent"), None).unwrap();
        d.close().unwrap();
        assert_eq!(writes(&seen).len(), 0);
    }

    #[test]
    fn off_for_a_device_that_says_on_sends_nothing_and_says_so() {
        let on = reading();
        let (fake, seen) = Fake::new();
        let mut d = device_of(fake, &on);
        set(&mut d, "CH1", Key::Enabled, Value::Bool(false)).unwrap();
        let notices = d.shared().unsent.clone();
        assert!(notices[0].contains("CH1") && notices[0].contains("STAYS ON"), "{notices:?}");
        d.close().unwrap();
        assert_eq!(writes(&seen).len(), 0, "as C: no frame it does not send");
        for (key, value, says) in [
            (Key::OverCurrentProtectionEnabled, Value::Bool(false), "stays on"),
            (Key::ChannelConfig, Value::from("Independent"), "reports Series"),
        ] {
            let (fake, seen) = Fake::new();
            let mut d = device_of(fake, &on);
            d.set(&key, &value, None).unwrap();
            let notices = d.shared().unsent.clone();
            assert!(notices[0].contains(says), "{key:?}: {notices:?}");
            d.close().unwrap();
            assert_eq!(writes(&seen).len(), 0, "{key:?}");
        }
    }

    #[test]
    fn a_device_that_agrees_gets_no_notice() {
        let (fake, _seen) = Fake::new();
        let mut d = device(fake);
        set(&mut d, "CH1", Key::Enabled, Value::Bool(false)).unwrap();
        d.set(&Key::OverCurrentProtectionEnabled, &Value::Bool(false), None).unwrap();
        d.set(&Key::ChannelConfig, &Value::from("Independent"), None).unwrap();
        assert_eq!(d.shared().unsent, Vec::<String>::new());
    }

    #[test]
    fn a_frame_takes_the_notice_back() {
        let (fake, seen) = Fake::new();
        let mut d = device_of(fake, &reading());
        set(&mut d, "CH1", Key::Enabled, Value::Bool(false)).unwrap();
        set(&mut d, "CH1", Key::VoltageTarget, Value::F64(5.0)).unwrap();
        d.shared().send_settings().unwrap();
        assert_eq!(d.shared().unsent, Vec::<String>::new(), "the frame carried it");
        assert_eq!(writes(&seen)[0][15], 0);
    }

    #[test]
    fn the_warning_goes_by_what_the_frame_does_to_each_output() {
        let mut reported = Reported::default();
        reported.update(&<[u8; FRAME]>::try_from(reading()).unwrap());
        // CH1 and CH2 are on: one named with a setpoint alone still goes off.
        let mut settings = Settings { ocp: true, mode: 1, ..Settings::default() };
        settings.channels[0].voltage = 5.0;
        let warning = frame_warning(&reported, &settings).unwrap();
        assert!(warning.starts_with("CH1, CH2 from on to off. "), "{warning}");
        settings.channels[0].enabled = true;
        let warning = frame_warning(&reported, &settings).unwrap();
        assert!(warning.starts_with("CH2 from on to off. "), "{warning}");
        // On and staying on, it still gets setpoints it cannot report.
        settings.channels[1].enabled = true;
        let warning = frame_warning(&reported, &settings).unwrap();
        assert!(warning.starts_with("A frame sets everything"), "{warning}");
        assert!(warning.contains("CH1 5.00 V 0.000 A, CH2 0.00 V 0.000 A"), "{warning}");
    }

    #[test]
    fn the_warning_says_protection_and_the_mode_change() {
        let mut reported = Reported::default();
        reported.update(&<[u8; FRAME]>::try_from(reading()).unwrap());
        let warning = frame_warning(&reported, &Settings::default()).unwrap();
        assert!(warning.contains("ocp_enabled from on to off"), "{warning}");
        assert!(warning.contains("channel_config from Series to Independent"), "{warning}");
        let settings = Settings { ocp: true, mode: 2, ..Settings::default() };
        let warning = frame_warning(&Reported::default(), &settings).unwrap();
        assert!(warning.contains("ocp_enabled from off to on"), "{warning}");
        assert!(warning.contains("channel_config from Independent to Parallel"), "{warning}");
    }

    #[test]
    fn a_frame_that_changes_nothing_on_a_supply_all_off_is_no_warning() {
        let mut settings = Settings::default();
        settings.channels[0].voltage = 5.0;
        assert_eq!(frame_warning(&Reported::default(), &settings), None);
    }

    #[test]
    fn a_new_capture_warns_afresh() {
        let (mut fake, _seen) = Fake::new();
        fake.silent_after = Some(0);
        let mut d = device_of(fake, &reading());
        d.shared().warned = true;
        let rx = start(&mut d);
        // The fake is silent, so no frame of settings can have set it again.
        assert!(!d.shared().warned);
        d.stop().unwrap();
        until(&rx, is_end, CAPTURE_BOUND);
    }

    #[test]
    fn a_capture_with_something_to_say_notes_it() {
        let (fake, _seen) = Fake::new();
        let mut d = device_of(fake, &reading());
        let rx = start(&mut d);
        until(&rx, is_analog, CAPTURE_BOUND);
        until(&rx, is_analog, CAPTURE_BOUND);
        until(&rx, is_analog, CAPTURE_BOUND);
        assert!(d.shared().warned, "it had something to warn of");
        d.close().unwrap();
    }

    #[test]
    fn a_close_says_if_its_frame_did_not_go() {
        let (mut fake, _seen) = Fake::new();
        fake.fail_writes_after = Some(0);
        let mut d = device(fake);
        set(&mut d, "CH1", Key::VoltageTarget, Value::F64(5.0)).unwrap();
        assert!(d.close().is_err());
    }

    #[test]
    fn ranges_bind_per_channel_and_a_nan_is_refused() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        for (group, key, value) in [
            ("CH1", Key::VoltageTarget, 32.01),
            ("CH1", Key::VoltageTarget, -0.01),
            ("CH1", Key::VoltageTarget, f64::NAN),
            ("CH3", Key::VoltageTarget, 6.01),
            ("CH2", Key::CurrentLimit, 3.01),
            ("CH2", Key::CurrentLimit, f64::NAN),
        ] {
            assert!(set(&mut d, group, key, Value::F64(value)).is_err(), "{group} {value}");
        }
        d.close().unwrap();
        assert_eq!(writes(&seen).len(), 0, "and nothing was queued");
    }

    #[test]
    fn a_key_in_the_wrong_scope_is_refused() {
        let (fake, _seen) = Fake::new();
        let mut d = device(fake);
        assert!(d.set(&Key::VoltageTarget, &Value::F64(1.0), None).is_err());
        assert!(set(&mut d, "CH1", Key::ChannelConfig, Value::from("Series")).is_err());
        assert!(set(&mut d, "CH4", Key::Enabled, Value::Bool(true)).is_err());
        assert!(d.set(&Key::ChannelConfig, &Value::from("Both"), None).is_err());
    }

    #[test]
    fn what_is_read_is_what_was_set_or_what_the_device_said() {
        let (fake, _seen) = Fake::new();
        let mut d = device_of(fake, &reading());
        set(&mut d, "CH1", Key::VoltageTarget, Value::F64(3.25)).unwrap();
        let get = |k: Key, g| d.get(&k, g).unwrap();
        assert_eq!(get(Key::VoltageTarget, Some("CH1")), Value::F64(3.25));
        assert_eq!(get(Key::CurrentLimit, Some("CH1")), Value::F64(0.0));
        assert_eq!(get(Key::Voltage, Some("CH1")), Value::F64(f64::from(12.34f32)));
        assert_eq!(get(Key::Current, Some("CH2")), Value::F64(1.25));
        assert_eq!(get(Key::Enabled, Some("CH1")), Value::Bool(true));
        assert_eq!(get(Key::Enabled, Some("CH3")), Value::Bool(false));
        assert_eq!(get(Key::ChannelConfig, None), Value::from("Series"));
        assert_eq!(get(Key::OverCurrentProtectionEnabled, None), Value::Bool(true));
    }

    #[test]
    fn each_answer_gives_its_readings_and_gets_the_settings_back() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        set(&mut d, "CH2", Key::VoltageTarget, Value::F64(5.0)).unwrap();
        let rx = start(&mut d);
        let first = until(&rx, is_analog, CAPTURE_BOUND);
        assert!(matches!(first[0], Packet::Header { .. }));
        let more = until(&rx, is_analog, CAPTURE_BOUND);
        assert_eq!(blocks(&first), [[12.34, 5.0, 3.3]]);
        assert_eq!(blocks(&more), [[0.5, 1.25, 0.025]]);
        d.stop().unwrap();
        until(&rx, is_end, CAPTURE_BOUND);
        d.close().unwrap();
        let w = writes(&seen);
        assert_eq!(w[0], protocol::ask());
        assert_eq!(w[1][1], 0x20);
        assert_eq!(w[1][6..8], [0x01, 0xf4]);
        assert!(w[1..].iter().all(|f| f[1] == 0x20));
    }

    #[test]
    fn a_reading_is_all_channels_with_c_s_units() {
        let (fake, _seen) = Fake::new();
        let mut d = device(fake);
        let rx = start(&mut d);
        let packets = until(&rx, is_analog, CAPTURE_BOUND);
        let more = until(&rx, is_analog, CAPTURE_BOUND);
        let (Packet::Analog(v), Packet::Analog(i)) = (packets.last().unwrap(), &more[0]) else {
            panic!("two blocks");
        };
        assert_eq!(v.channels, [0, 1, 2]);
        assert_eq!((v.mq, v.unit, v.flags, v.digits), (Mq::Voltage, Unit::Volt, MqFlags::DC, 2));
        assert_eq!(
            (i.mq, i.unit, i.flags, i.digits),
            (Mq::Current, Unit::Ampere, MqFlags::empty(), 3)
        );
        d.close().unwrap();
    }

    #[test]
    fn no_poll_goes_out_before_the_answer_is_whole() {
        let (mut fake, seen) = Fake::new();
        let (head, tail) = (reading()[..10].to_vec(), reading()[10..].to_vec());
        fake.scripted = vec![(0, vec![(Duration::ZERO, head), (Duration::from_millis(200), tail)])];
        let mut d = device(fake);
        let rx = start(&mut d);
        std::thread::sleep(Duration::from_millis(100));
        assert_eq!(writes(&seen).len(), 1, "still in the first answer");
        let packets = until(&rx, is_analog, CAPTURE_BOUND);
        assert_eq!(blocks(&packets), [[12.34, 5.0, 3.3]], "put together");
        d.close().unwrap();
    }

    #[test]
    fn a_stop_ends_it_at_the_next_answer_with_no_poll_after_it() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        let rx = start(&mut d);
        until(&rx, is_analog, CAPTURE_BOUND);
        d.stop().unwrap();
        let before = writes(&seen).len();
        let begun = Instant::now();
        let rest = until(&rx, is_end, CAPTURE_BOUND);
        assert!(begun.elapsed() < STOP_WAIT, "{:?}", begun.elapsed());
        assert!(matches!(rest.last(), Some(Packet::End)));
        // The fake always answers, so a stop that was ignored runs away.
        assert!(writes(&seen).len() <= before + 1, "at most the poll under way");
        d.close().unwrap();
    }

    #[test]
    fn a_stop_with_an_answer_on_its_way_takes_it() {
        let (mut fake, _seen) = Fake::new();
        fake.scripted = vec![(0, vec![(Duration::from_millis(100), reading())])];
        let mut d = device(fake);
        let rx = start(&mut d);
        std::thread::sleep(Duration::from_millis(20));
        d.stop().unwrap();
        let packets = until(&rx, is_end, CAPTURE_BOUND);
        assert_eq!(blocks(&packets).len(), 2, "its volts and amperes");
        d.close().unwrap();
    }

    #[test]
    fn a_stop_on_a_silent_device_ends_in_its_time() {
        let (mut fake, seen) = Fake::new();
        fake.silent_after = Some(0);
        let mut d = device(fake);
        let rx = start(&mut d);
        until(&rx, |p| matches!(p, Packet::Header { .. }), CAPTURE_BOUND);
        d.stop().unwrap();
        let begun = Instant::now();
        until(&rx, is_end, SILENCE.saturating_sub(Duration::from_millis(500)));
        assert!(begun.elapsed() >= STOP_WAIT.saturating_sub(READ_SLICE));
        assert_eq!(writes(&seen).len(), 1);
    }

    #[test]
    fn a_silent_device_ends_a_capture() {
        let (mut fake, seen) = Fake::new();
        fake.silent_after = Some(2);
        let mut d = device(fake);
        let rx = start(&mut d);
        let begun = Instant::now();
        let packets = until(&rx, is_end, CAPTURE_BOUND);
        assert!(begun.elapsed() >= SILENCE);
        assert_eq!(blocks(&packets).len(), 4, "two answers");
        assert_eq!(writes(&seen).len(), 3);
    }

    #[test]
    fn a_failed_read_or_write_ends_a_capture() {
        let (mut fake, seen) = Fake::new();
        fake.fail_writes_after = Some(1);
        let mut d = device(fake);
        let rx = start(&mut d);
        let packets = until(&rx, is_end, CAPTURE_BOUND);
        assert_eq!(blocks(&packets).len(), 2);
        assert_eq!(writes(&seen).len(), 1);

        let (mut fake, _seen) = Fake::new();
        fake.fail_reads = true;
        let mut d = device(fake);
        let rx = start(&mut d);
        let packets = until(&rx, is_end, CAPTURE_BOUND);
        assert_eq!(blocks(&packets).len(), 0);
    }

    #[test]
    fn an_extra_byte_does_not_shift_the_answers_after_it() {
        let (mut fake, _seen) = Fake::new();
        let mut long = reading();
        long.push(0xff);
        fake.scripted = vec![(0, vec![(Duration::ZERO, long)])];
        let mut d = device(fake);
        let rx = start(&mut d);
        let mut packets = Vec::new();
        for _ in 0..6 {
            packets.extend(until(&rx, is_analog, CAPTURE_BOUND));
        }
        d.close().unwrap();
        let got = blocks(&packets);
        assert_eq!(got.len(), 6);
        for pair in got.chunks(2) {
            assert_eq!(pair[0], [12.34, 5.0, 3.3]);
            assert_eq!(pair[1], [0.5, 1.25, 0.025]);
        }
    }

    #[test]
    fn an_answer_without_its_header_is_no_reading_and_the_polls_go_on() {
        let (mut fake, seen) = Fake::new();
        let mut bad = reading();
        bad[0] = 0x55;
        bad[2] = 0xff;
        fake.scripted = vec![(0, vec![(Duration::ZERO, bad)])];
        let mut d = device(fake);
        let rx = start(&mut d);
        let packets = until(&rx, is_analog, CAPTURE_BOUND);
        assert_eq!(blocks(&packets), [[12.34, 5.0, 3.3]], "the second answer");
        assert!(writes(&seen).len() >= 2);
        d.close().unwrap();
    }

    #[test]
    fn a_stray_byte_gives_no_reading_and_leaves_what_was_reported() {
        let (mut fake, _seen) = Fake::new();
        let mut stray = vec![0xaa];
        stray.extend(reading());
        fake.scripted = vec![(1, vec![(Duration::ZERO, stray)])];
        let mut d = device(fake);
        let rx = start(&mut d);
        let mut packets = Vec::new();
        for _ in 0..4 {
            packets.extend(until(&rx, is_analog, CAPTURE_BOUND));
        }
        d.close().unwrap();
        for pair in blocks(&packets).chunks(2) {
            assert_eq!(pair[0], [12.34, 5.0, 3.3], "no 435.24");
            assert_eq!(pair[1], [0.5, 1.25, 0.025]);
        }
    }

    #[test]
    fn a_scan_answer_no_supply_could_give_is_a_device_with_no_state() {
        let (mut fake, _seen) = Fake::new();
        let mut wild = reading();
        wild[2] = 0xaa;
        fake.scripted = vec![(0, vec![(Duration::ZERO, wild)])];
        let reported = probe(Box::new(fake)).expect("a device, as C finds one");
        assert_eq!(reported, Reported::default());
    }

    #[test]
    fn a_setting_during_a_capture_goes_out_as_the_next_poll() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        let rx = start(&mut d);
        until(&rx, is_analog, CAPTURE_BOUND);
        set(&mut d, "CH1", Key::VoltageTarget, Value::F64(3.25)).unwrap();
        // A few rounds on, for the one under way to pass.
        for _ in 0..4 {
            until(&rx, is_analog, CAPTURE_BOUND);
        }
        d.stop().unwrap();
        until(&rx, is_end, CAPTURE_BOUND);
        d.close().unwrap();
        let w = writes(&seen);
        assert!(w.iter().any(|f| f[2..4] == [0x01, 0x45]), "{w:?}");
    }

    #[test]
    fn an_answer_taken_clears_what_was_queued() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake);
        set(&mut d, "CH1", Key::VoltageTarget, Value::F64(5.0)).unwrap();
        let rx = start(&mut d);
        until(&rx, is_analog, CAPTURE_BOUND);
        // The settings go back after the readings.
        let deadline = Instant::now() + CAPTURE_BOUND;
        while writes(&seen).len() < 2 {
            assert!(Instant::now() < deadline, "the setting never went");
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        until(&rx, is_end, CAPTURE_BOUND);
        let during = writes(&seen).len();
        d.close().unwrap();
        assert_eq!(writes(&seen).len(), during, "the poll took it");
    }

    #[test]
    fn a_stop_before_the_first_answer_leaves_the_setting_for_the_close() {
        let (mut fake, seen) = Fake::new();
        fake.silent_after = Some(0);
        let mut d = device(fake);
        set(&mut d, "CH1", Key::VoltageTarget, Value::F64(5.0)).unwrap();
        let rx = start(&mut d);
        d.stop().unwrap();
        until(&rx, is_end, CAPTURE_BOUND);
        d.close().unwrap();
        let w = writes(&seen);
        assert_eq!(w.len(), 2);
        assert_eq!(w[1][2..4], [0x01, 0xf4]);
    }

    #[test]
    fn a_second_start_while_running_is_busy() {
        let (fake, _seen) = Fake::new();
        let mut d = device(fake);
        let _rx = start(&mut d);
        let (tx, _rx2) = sync_channel(4);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.close().unwrap();
    }

    #[test]
    fn a_start_without_an_open_is_refused() {
        let (fake, _seen) = Fake::new();
        let mut d = AttenPpsDevice::from_port(Box::new(fake), Reported::default());
        let (tx, _rx) = sync_channel(4);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
    }
}
