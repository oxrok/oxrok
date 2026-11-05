// Counterpart in sigrok:
//   libsigrok/src/hardware/rdtech-um/api.c,
//   libsigrok/src/hardware/rdtech-um/protocol.c,
//   Copyright (C) 2018-2020 Andreas Sandberg <andreas@sandberg.pp.se>,
//   with commits by Gerhard Sittig, Mathieu Pilato
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `RDTech` UM24C, UM25C and UM34C USB power meters, which report over a
//! serial link: Bluetooth SPP on the real meters, so `conn=` names an RFCOMM
//! tty as a rule. libsigrok's `bt/rfcomm/MAC` form of `conn=` is not ported,
//! as `src/serial` has no Bluetooth transport.
//!
//! A scan polls once and goes by the id in the answer. A capture polls again
//! at least 100 ms after the last, never while a reply is coming, and gives
//! one frame per dump with a sample on each of six channels, whichever are
//! enabled, as in C. The meter is only ever asked; nothing here changes it.
//!
//! Where C ends up stuck, here the capture goes on or ends:
//!
//! - A reply that stays incomplete, or that has no id, holds back every later
//!   poll in C (`protocol.c:157-158`), so a lost byte stops a capture for good.
//!   Here an incomplete reply goes after [`Pace::stall`] with a warning, and
//!   bytes ahead of an id go as they come (see [`protocol::Assembly`]).
//! - After an unplug C logs a failed poll without end (`protocol.c:350`, 361:
//!   a hung up port is not read, and the failure is dropped). Here a port that
//!   fails ends the capture.
//! - A meter that does not answer is polled for ever in C. Here that is so
//!   where another limit or the user can end it; a frame limit alone would
//!   never come, so that capture ends after [`Pace::silence`] with an error.
//!
//! A write here waits for room up to `src/serial`'s 5 s and then fails, where
//! C gives up after 1 ms (`WRITE_TO_MS`) and carries on, so a meter that has
//! stopped reading ends a capture late and with an error, past a time limit.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    MqFlags, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{self, Params, Port};
use protocol::{Assembly, DUMP, POLL, Profile};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::EnergyMeter];
/// `RDTECH_UM_SERIALCOMM`.
const SERIALCOMM: &str = "115200/8n1";
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];
/// `PROBE_TO_MS`.
const PROBE_TIMEOUT: Duration = Duration::from_millis(1000);

/// The times of a capture; tests take shorter ones.
#[derive(Clone, Copy, Debug)]
pub struct Pace {
    /// `POLL_PERIOD_MS`: the least time between two polls.
    pub period: Duration,
    /// The session source's 50 ms: how long a read waits, so how often a limit,
    /// a stop and the period are looked at.
    pub wait: Duration,
    /// How long part of a reply may stand before it is dropped. C has no such
    /// thing; this is its probe's wait for a whole reply.
    pub stall: Duration,
    /// How long a capture that only a frame limit can end waits for a dump.
    pub silence: Duration,
}

impl Default for Pace {
    fn default() -> Pace {
        Pace {
            period: Duration::from_millis(100),
            wait: Duration::from_millis(50),
            stall: PROBE_TIMEOUT,
            silence: Duration::from_secs(5),
        }
    }
}

/// The `rdtech-um` driver.
pub struct RdtechUmDriver;

impl Driver for RdtechUmDriver {
    fn id(&self) -> &'static str {
        "rdtech-um"
    }

    fn long_name(&self) -> &'static str {
        "RDTech UMxx USB power meter"
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
        // Every failure of the port here is no device, as `scan` returns NULL
        // for each.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((serial::open(&conn, params)?, params)));
        let (mut port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        let Some(profile) = probe(port.as_mut()) else {
            log::error!("Failed to find a supported RDTech UM device.");
            return Ok(Vec::new());
        };
        // The scan's close: `dev_open` opens it again.
        drop(port);
        Ok(vec![Box::new(UmDevice::new(profile, conn, params))])
    }
}

/// `rdtech_um_probe`: one poll, and the model its answer says.
fn probe(port: &mut dyn Port) -> Option<&'static Profile> {
    if let Err(e) = port.write(&[POLL]) {
        log::debug!("{e}");
        log::error!("Failed to send probe request.");
        return None;
    }
    let mut dump = [0u8; DUMP];
    let got = serial::read_blocking(port, &mut dump, PROBE_TIMEOUT).unwrap_or_else(|e| {
        log::debug!("{e}");
        0
    });
    if got != DUMP {
        log::error!("Failed to read probe response.");
        return None;
    }
    let id = u16::from_be_bytes([dump[0], dump[1]]);
    let Some(profile) = protocol::find_profile(id) else {
        log::error!("Unrecognized UM device (0x{id:04x}).");
        return None;
    };
    if !profile.check_ok(&dump) {
        log::error!("Probe response fails checksum verification.");
        return None;
    }
    Some(profile)
}

/// One meter.
pub struct UmDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    profile: &'static Profile,
    conn: String,
    params: Params,
    limit_frames: u64,
    limit_msec: u64,
    pace: Pace,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// The open port, when no capture holds it: a capture gives it back ahead
    /// of its End, so a client can start again as soon as it sees that.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl UmDevice {
    fn new(profile: &'static Profile, conn: String, params: Params) -> Self {
        let channels = profile
            .channels
            .iter()
            .enumerate()
            .map(|(i, c)| Channel::new(i, ChannelKind::Analog, c.name.to_owned()))
            .collect();
        UmDevice {
            info: DeviceInfo {
                driver: "rdtech-um",
                vendor: "RDTech".to_owned(),
                model: profile.name.to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels,
            profile,
            conn,
            params,
            limit_frames: 0,
            limit_msec: 0,
            pace: Pace::default(),
            pending: None,
            port: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device of `profile` on a port that stands in for the hardware,
    /// without a scan. [`Device::open`] takes it.
    pub fn from_port(profile: &'static Profile, port: Box<dyn Port>, pace: Pace) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device = Self::new(profile, conn, params);
        device.pending = Some(port);
        device.pace = pace;
        device
    }
}

fn no_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        None => Ok(()),
    }
}

impl Device for UmDevice {
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
        match key {
            Key::LimitFrames => Ok(Value::U64(self.limit_frames)),
            Key::LimitMsec => Ok(Value::U64(self.limit_msec)),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitFrames => self.limit_frames = value.as_u64().ok_or_else(bad)?,
            // `core::config_set` refuses a 0 and says so; this is for a library
            // caller that comes straight here.
            Key::LimitMsec => {
                self.limit_msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut slot = self.port.lock().expect("port lock");
        if slot.is_some() || self.worker.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => serial::open(&self.conn, self.params)?,
        };
        *slot = Some(port);
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        *self.port.lock().expect("port lock") = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        let Some(port) = self.port.lock().expect("port lock").take() else {
            return Err(match &self.worker {
                Some(_) => Error::Busy,
                None => Error::device("device is not open"),
            });
        };
        if let Some(worker) = self.worker.take() {
            // It gave the port back, and has only its End left to send.
            let _ = worker.handle.join();
        }
        let capture = Capture {
            profile: self.profile,
            limit_frames: self.limit_frames,
            limit_msec: self.limit_msec,
            pace: self.pace,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let slot = Arc::clone(&self.port);
        let handle = std::thread::Builder::new()
            .name("rdtech-um acquisition".to_owned())
            .spawn(move || {
                let mut port = port;
                capture.run(port.as_mut(), &sink, &stop2);
                *slot.lock().expect("port lock") = Some(port);
                sink.send(Packet::End);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending. It
    /// sends nothing to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// One capture, on its own thread: `rdtech_um_receive_data`.
struct Capture {
    profile: &'static Profile,
    limit_frames: u64,
    limit_msec: u64,
    pace: Pace,
}

impl Capture {
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) {
        sink.send(Packet::header());
        let started = Instant::now();
        let time_limit = Duration::from_millis(self.limit_msec);
        let mut assembly = Assembly::new(self.profile.id);
        let mut buffer = [0u8; protocol::BUFFER];
        let mut frames = 0u64;
        // `dev_acquisition_start`'s forced poll.
        if !poll(port) {
            return;
        }
        let mut polled = Instant::now();
        let mut last_byte = polled;
        let mut last_dump = polled;
        loop {
            if stop.load(Ordering::SeqCst) {
                return;
            }
            let room = assembly.room();
            let got = match port.read(&mut buffer[..room], self.pace.wait) {
                Ok(got) => got,
                Err(e) => {
                    log::error!("Reading the port failed: {e}.");
                    return;
                }
            };
            if got > 0 {
                last_byte = Instant::now();
                let synced = assembly.feed(&buffer[..got]);
                if synced.warn {
                    log::warn!("Unexpected response data, trying to synchronize.");
                }
                if synced.skipped > 0 {
                    log::debug!("Skipping {} bytes in attempt to sync.", synced.skipped);
                }
            }
            while let Some(dump) = assembly.next_dump() {
                if !self.profile.check_ok(&dump) {
                    log::error!("Packet checksum verification failed.");
                    log::error!("Processing response packet failed.");
                    return;
                }
                if !self.send_frame(&dump, sink) {
                    return;
                }
                frames += 1;
                last_dump = Instant::now();
                // Before the next dump, where C sends that too, after its End.
                if self.limit_frames > 0 && frames >= self.limit_frames {
                    return;
                }
            }
            if self.limit_msec > 0 && started.elapsed() > time_limit {
                return;
            }
            if assembly.accumulating() && last_byte.elapsed() >= self.pace.stall {
                log::warn!("Dropping {} bytes of a reply that stopped.", assembly.held());
                assembly.drop_partial();
            }
            let empty = last_dump.elapsed();
            if self.limit_frames > 0 && self.limit_msec == 0 && empty >= self.pace.silence {
                log::error!("No answer from the device for {} s.", empty.as_secs());
                return;
            }
            if !assembly.accumulating() && polled.elapsed() >= self.pace.period {
                if !poll(port) {
                    return;
                }
                polled = Instant::now();
            }
        }
    }

    /// `process_data`'s frame: every channel, in order.
    fn send_frame(&self, dump: &[u8; DUMP], sink: &PacketSink) -> bool {
        if !sink.send(Packet::FrameBegin) {
            return false;
        }
        for (i, channel) in self.profile.channels.iter().enumerate() {
            let sent = sink.send(Packet::Analog(AnalogData {
                channels: vec![i],
                samples: vec![channel.value(dump)],
                mq: channel.mq,
                flags: MqFlags::default(),
                unit: channel.unit,
                digits: channel.digits,
            }));
            if !sent {
                return false;
            }
        }
        sink.send(Packet::FrameEnd)
    }
}

/// One poll: whether it went out. C drops a failure in a capture and tries
/// again, without end on a port that left; here it ends the capture.
fn poll(port: &mut dyn Port) -> bool {
    match port.write(&[POLL]) {
        Ok(()) => true,
        Err(e) => {
            log::debug!("{e}");
            log::error!("Unable to send poll request.");
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit
    /// that does not bite must fail, not hang the run.
    const BOUND: Duration = Duration::from_secs(10);
    const QUICK: Pace = Pace {
        period: Duration::from_millis(20),
        wait: Duration::from_millis(2),
        stall: Duration::from_millis(60),
        silence: Duration::from_millis(100),
    };

    /// What the fake saw.
    #[derive(Default)]
    struct Seen {
        polls: Vec<Instant>,
        other_writes: usize,
        purges: usize,
    }

    /// What a poll gets.
    #[derive(Clone)]
    enum Reply {
        Dump,
        Bytes(Vec<u8>),
        /// Bytes, then the rest of the dump after a pause.
        Parts(Vec<u8>, Duration, Vec<u8>),
    }

    /// A meter that answers each poll, by a script and then with `after`.
    struct Fake {
        seen: Arc<Mutex<Seen>>,
        input: VecDeque<(Instant, u8)>,
        script: VecDeque<Reply>,
        after: Option<Reply>,
        energy: u32,
        fail_write_at: Option<usize>,
        fail_read: bool,
    }

    fn dump(energy: u32) -> Vec<u8> {
        let mut d = vec![0u8; DUMP];
        d[..2].copy_from_slice(&[0x09, 0x63]);
        d[2..4].copy_from_slice(&512u16.to_be_bytes());
        d[106..110].copy_from_slice(&energy.to_be_bytes());
        d[128] = 0xff;
        d[129] = 0xf1;
        d
    }

    impl Fake {
        fn new() -> (Fake, Arc<Mutex<Seen>>) {
            let seen = Arc::new(Mutex::new(Seen::default()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                script: VecDeque::new(),
                after: Some(Reply::Dump),
                energy: 0,
                fail_write_at: None,
                fail_read: false,
            };
            (fake, seen)
        }

        fn push(&mut self, at: Instant, bytes: &[u8]) {
            self.input.extend(bytes.iter().map(|b| (at, *b)));
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
            if self.fail_read {
                return Err(Error::device("the port went"));
            }
            let deadline = Instant::now() + timeout;
            loop {
                let now = Instant::now();
                let ready = self.input.iter().take_while(|(at, _)| *at <= now).count();
                let n = ready.min(buf.len());
                if n > 0 {
                    for slot in buf.iter_mut().take(n) {
                        *slot = self.input.pop_front().expect("counted").1;
                    }
                    return Ok(n);
                }
                if now >= deadline {
                    return Ok(0);
                }
                std::thread::sleep(Duration::from_micros(200));
            }
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let now = Instant::now();
            {
                let mut seen = self.seen.lock().unwrap();
                if self.fail_write_at == Some(seen.polls.len()) {
                    return Err(Error::device("the port went"));
                }
                if data == [POLL] {
                    seen.polls.push(now);
                } else {
                    seen.other_writes += 1;
                }
            }
            match self.script.pop_front().or_else(|| self.after.clone()) {
                Some(Reply::Dump) => {
                    let d = dump(self.energy);
                    self.energy += 1;
                    self.push(now, &d);
                }
                Some(Reply::Bytes(b)) => self.push(now, &b),
                Some(Reply::Parts(a, pause, b)) => {
                    self.push(now, &a);
                    self.push(now + pause, &b);
                }
                None => {}
            }
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.seen.lock().unwrap().purges += 1;
            self.input.clear();
            Ok(())
        }
    }

    fn um24c() -> &'static Profile {
        protocol::find_profile(0x0963).unwrap()
    }

    fn capture(fake: Fake, frames: u64, msec: u64) -> (UmDevice, Receiver<(usize, Packet)>) {
        let mut d = UmDevice::from_port(um24c(), Box::new(fake), QUICK);
        d.limit_frames = frames;
        d.limit_msec = msec;
        d.open().unwrap();
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        (d, rx)
    }

    /// Everything up to the End, or a failure.
    fn until_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
        let deadline = Instant::now() + BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let packet = rx.recv_timeout(left).expect("the capture ended in time").1;
            if matches!(packet, Packet::End) {
                return out;
            }
            out.push(packet);
        }
    }

    fn energies(packets: &[Packet]) -> Vec<f32> {
        packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) if a.channels == [5] => Some(a.samples[0]),
                _ => None,
            })
            .collect()
    }

    fn frame_count(packets: &[Packet]) -> usize {
        packets.iter().filter(|p| matches!(p, Packet::FrameEnd)).count()
    }

    #[test]
    fn a_frame_has_a_sample_per_channel_in_order() {
        let (fake, seen) = Fake::new();
        let (mut d, rx) = capture(fake, 1, 0);
        let packets = until_end(&rx);
        assert!(matches!(packets[0], Packet::Header { .. }), "the header first");
        assert!(matches!(packets[1], Packet::FrameBegin));
        assert!(matches!(packets[8], Packet::FrameEnd));
        assert_eq!(packets.len(), 9);
        let channels: Vec<usize> = packets[2..8]
            .iter()
            .map(|p| match p {
                Packet::Analog(a) => a.channels[0],
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(channels, [0, 1, 2, 3, 4, 5]);
        d.close().unwrap();
        let seen = seen.lock().unwrap();
        assert_eq!(seen.polls.len(), 1);
        assert_eq!(seen.other_writes, 0);
        assert_eq!(seen.purges, 1, "an open purges");
    }

    #[test]
    fn a_frame_limit_stops_the_polls_and_takes_no_more() {
        let (fake, seen) = Fake::new();
        let (mut d, rx) = capture(fake, 3, 0);
        let packets = until_end(&rx);
        assert_eq!(energies(&packets), [0.0, 0.001, 0.002]);
        std::thread::sleep(QUICK.period * 3);
        assert_eq!(seen.lock().unwrap().polls.len(), 3);
        d.close().unwrap();
    }

    #[test]
    fn a_second_dump_in_a_read_past_the_limit_is_cut() {
        let (mut fake, seen) = Fake::new();
        let mut two = dump(7);
        two.extend(dump(8));
        fake.script.push_back(Reply::Bytes(two));
        let (mut d, rx) = capture(fake, 1, 0);
        let packets = until_end(&rx);
        assert_eq!(energies(&packets), [0.007]);
        assert_eq!(frame_count(&packets), 1);
        assert_eq!(seen.lock().unwrap().polls.len(), 1);
        d.close().unwrap();
    }

    #[test]
    fn the_first_poll_goes_at_once() {
        let (fake, seen) = Fake::new();
        let pace = Pace { period: Duration::from_secs(30), ..QUICK };
        let mut d = UmDevice::from_port(um24c(), Box::new(fake), pace);
        d.limit_frames = 1;
        d.open().unwrap();
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).unwrap();
        assert_eq!(frame_count(&until_end(&rx)), 1, "long before the period");
        assert_eq!(seen.lock().unwrap().polls.len(), 1);
        d.close().unwrap();
    }

    #[test]
    fn polls_keep_their_period() {
        let (fake, seen) = Fake::new();
        let (mut d, rx) = capture(fake, 6, 0);
        until_end(&rx);
        d.close().unwrap();
        let seen = seen.lock().unwrap();
        assert_eq!(seen.polls.len(), 6);
        for pair in seen.polls.windows(2) {
            assert!(pair[1] - pair[0] >= QUICK.period, "{:?}", pair[1] - pair[0]);
        }
    }

    #[test]
    fn no_poll_goes_while_a_reply_is_coming() {
        let (mut fake, seen) = Fake::new();
        let d0 = dump(0);
        let pause = QUICK.period * 2;
        assert!(pause < QUICK.stall, "the reply must end before it is dropped");
        fake.script.push_back(Reply::Parts(d0[..60].to_vec(), pause, d0[60..].to_vec()));
        let (mut d, rx) = capture(fake, 1, 0);
        let packets = until_end(&rx);
        assert_eq!(frame_count(&packets), 1);
        assert_eq!(seen.lock().unwrap().polls.len(), 1, "the second part ended it");
        d.close().unwrap();
    }

    #[test]
    fn a_reply_that_stops_is_dropped_and_polling_goes_on() {
        let (mut fake, seen) = Fake::new();
        fake.script.push_back(Reply::Bytes(dump(9)[..60].to_vec()));
        let started = Instant::now();
        let (mut d, rx) = capture(fake, 2, 0);
        let packets = until_end(&rx);
        assert!(started.elapsed() >= QUICK.stall, "it waited for the rest");
        assert_eq!(energies(&packets), [0.0, 0.001]);
        let seen = seen.lock().unwrap();
        assert_eq!(seen.polls.len(), 3);
        assert!(seen.polls[1] - seen.polls[0] >= QUICK.stall, "none while it stood");
        drop(seen);
        d.close().unwrap();
    }

    #[test]
    fn junk_does_not_stop_the_polls() {
        let (mut fake, _) = Fake::new();
        fake.script.push_back(Reply::Bytes(vec![0x55; 130]));
        let (mut d, rx) = capture(fake, 1, 0);
        assert_eq!(energies(&until_end(&rx)), [0.0]);
        d.close().unwrap();
    }

    #[test]
    fn noise_cut_by_the_reads_still_syncs() {
        let (mut fake, _) = Fake::new();
        let mut stream = vec![0xab, 0xcd];
        stream.extend(dump(5));
        let rest = stream.split_off(1);
        fake.script.push_back(Reply::Parts(stream, QUICK.wait * 3, rest));
        let (mut d, rx) = capture(fake, 1, 0);
        assert_eq!(energies(&until_end(&rx)), [0.005]);
        d.close().unwrap();
    }

    #[test]
    fn a_bad_end_ends_the_capture_where_it_is() {
        let (mut fake, seen) = Fake::new();
        let mut bad = dump(9);
        bad[129] = 0xf0;
        fake.script.push_back(Reply::Dump);
        fake.script.push_back(Reply::Bytes(bad));
        let (mut d, rx) = capture(fake, 5, 0);
        assert_eq!(energies(&until_end(&rx)), [0.0]);
        assert_eq!(seen.lock().unwrap().polls.len(), 2);
        d.close().unwrap();
    }

    #[test]
    fn silence_ends_only_a_capture_nothing_else_can_end() {
        let (mut fake, seen) = Fake::new();
        fake.after = None;
        let started = Instant::now();
        let (mut d, rx) = capture(fake, 2, 0);
        assert_eq!(frame_count(&until_end(&rx)), 0);
        assert!(started.elapsed() >= QUICK.silence);
        assert!(seen.lock().unwrap().polls.len() >= 3, "it kept asking");
        d.close().unwrap();

        // With a time limit that goes first, and far past the bound.
        let (mut fake, seen) = Fake::new();
        fake.after = None;
        let started = Instant::now();
        let time = QUICK.silence * 3;
        let (mut d, rx) = capture(fake, 2, u64::try_from(time.as_millis()).expect("a short time"));
        until_end(&rx);
        assert!(started.elapsed() >= time, "{:?}", started.elapsed());
        assert!(seen.lock().unwrap().polls.len() >= 6);
        d.close().unwrap();
    }

    #[test]
    fn junk_for_ever_ends_a_capture_only_a_frame_limit_can_end() {
        let (mut fake, seen) = Fake::new();
        fake.after = Some(Reply::Bytes(vec![0x55; 130]));
        let (mut d, rx) = capture(fake, 1, 0);
        assert_eq!(frame_count(&until_end(&rx)), 0);
        assert!(seen.lock().unwrap().polls.len() >= 3, "it kept asking");
        d.close().unwrap();
    }

    #[test]
    fn a_continuous_capture_waits_out_silence_until_stopped() {
        let (mut fake, _) = Fake::new();
        fake.after = None;
        let (mut d, rx) = capture(fake, 0, 0);
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        assert!(
            rx.recv_timeout(QUICK.silence * 3).is_err(),
            "no End by itself, well past the bound"
        );
        d.stop().unwrap();
        assert_eq!(until_end(&rx).len(), 0);
        d.close().unwrap();
    }

    #[test]
    fn a_stop_ends_a_capture_the_meter_keeps_feeding() {
        let (fake, seen) = Fake::new();
        let (mut d, rx) = capture(fake, 0, 0);
        assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
        // Some frames, to know that it runs.
        while seen.lock().unwrap().polls.len() < 3 {
            assert!(!matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::End));
        }
        d.stop().unwrap();
        until_end(&rx);
        let polls = seen.lock().unwrap().polls.len();
        std::thread::sleep(QUICK.period * 3);
        assert_eq!(seen.lock().unwrap().polls.len(), polls, "none after the End");
        d.close().unwrap();
    }

    #[test]
    fn a_time_limit_ends_a_capture_the_meter_keeps_feeding() {
        let (fake, _) = Fake::new();
        let started = Instant::now();
        let (mut d, rx) = capture(fake, 0, 50);
        let packets = until_end(&rx);
        assert!(started.elapsed() >= Duration::from_millis(50));
        assert!(frame_count(&packets) >= 1);
        d.close().unwrap();
    }

    #[test]
    fn a_failed_poll_ends_the_capture() {
        let (mut fake, seen) = Fake::new();
        fake.fail_write_at = Some(0);
        let (mut d, rx) = capture(fake, 0, 0);
        assert_eq!(frame_count(&until_end(&rx)), 0);
        assert_eq!(seen.lock().unwrap().polls.len(), 0);
        d.close().unwrap();

        let (mut fake, seen) = Fake::new();
        fake.fail_write_at = Some(2);
        let (mut d, rx) = capture(fake, 0, 0);
        assert_eq!(frame_count(&until_end(&rx)), 2);
        assert_eq!(seen.lock().unwrap().polls.len(), 2);
        d.close().unwrap();
    }

    #[test]
    fn a_failed_read_ends_the_capture() {
        let (mut fake, _) = Fake::new();
        fake.fail_read = true;
        let (mut d, rx) = capture(fake, 0, 0);
        assert_eq!(frame_count(&until_end(&rx)), 0);
        d.close().unwrap();
    }

    #[test]
    fn a_client_can_start_again_at_the_end() {
        let (fake, _) = Fake::new();
        let (mut d, rx) = capture(fake, 1, 0);
        until_end(&rx);
        let (tx, rx) = sync_channel(4096);
        d.start(PacketSink::new(tx, 0)).expect("the port is back ahead of the End");
        assert_eq!(energies(&until_end(&rx)), [0.001]);
        d.close().unwrap();
    }

    #[test]
    fn a_running_capture_is_busy_and_a_closed_device_is_not_open() {
        let (fake, _) = Fake::new();
        // A time limit, so that a stop that does not bite cannot hang the close.
        let (mut d, _rx) = capture(fake, 0, 2000);
        let (tx, _rx2) = sync_channel(1);
        assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
        d.close().unwrap();
        let (tx, _rx3) = sync_channel(1);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
    }

    fn probe_with(fake: &mut Fake) -> Option<&'static str> {
        probe(fake).map(|p| p.name)
    }

    #[test]
    fn a_probe_names_the_model() {
        let (mut fake, seen) = Fake::new();
        assert_eq!(probe_with(&mut fake), Some("UM24C"));
        assert_eq!(seen.lock().unwrap().polls.len(), 1);
    }

    #[test]
    fn a_probe_wants_a_known_id_and_a_good_end() {
        let (mut fake, _) = Fake::new();
        let mut unknown = dump(0);
        unknown[0] = 0x12;
        fake.script.push_back(Reply::Bytes(unknown));
        assert_eq!(probe_with(&mut fake), None);

        let (mut fake, _) = Fake::new();
        let mut bad = dump(0);
        bad[129] = 0;
        fake.script.push_back(Reply::Bytes(bad));
        assert_eq!(probe_with(&mut fake), None);

        let (mut fake, _) = Fake::new();
        fake.fail_write_at = Some(0);
        assert_eq!(probe_with(&mut fake), None);

        let (mut fake, _) = Fake::new();
        fake.fail_read = true;
        assert_eq!(probe_with(&mut fake), None);
    }

    #[test]
    fn a_probe_waits_for_the_whole_answer() {
        let (mut fake, _) = Fake::new();
        let d0 = dump(0);
        fake.script.push_back(Reply::Parts(
            d0[..10].to_vec(),
            Duration::from_millis(50),
            d0[10..].to_vec(),
        ));
        assert_eq!(probe_with(&mut fake), Some("UM24C"));
    }

    #[test]
    fn limits_read_back_and_a_zero_time_is_refused() {
        let (fake, _) = Fake::new();
        let mut d = UmDevice::from_port(um24c(), Box::new(fake), QUICK);
        d.set(&Key::LimitFrames, &Value::U64(3), None).unwrap();
        d.set(&Key::LimitMsec, &Value::U64(500), None).unwrap();
        assert_eq!(d.get(&Key::LimitFrames, None).unwrap().as_u64(), Some(3));
        assert_eq!(d.get(&Key::LimitMsec, None).unwrap().as_u64(), Some(500));
        d.set(&Key::LimitFrames, &Value::U64(0), None).unwrap();
        assert!(d.set(&Key::LimitMsec, &Value::U64(0), None).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::U64(1), None).is_err());
        assert!(d.get(&Key::Conn, None).is_err());
        assert!(d.get(&Key::LimitFrames, Some("V")).is_err());
    }
}
