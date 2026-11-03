// Counterpart in sigrok:
//   libsigrok/src/hardware/serial-dmm/api.c,
//   libsigrok/src/hardware/serial-dmm/protocol.c,
//   libsigrok/src/hardware/serial-dmm/protocol.h,
//   Copyright (C) 2012-2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2012 Alexandru Gagniuc <mr.nuke.me@gmail.com>,
//   with commits by Bert Vermeulen, Gerhard Sittig, Daniel Elstner,
//   Martin Ling, Aurelien Jacobs, Matthias Heidbrink, Janne Huttunen,
//   Frank Stettner, Soeren Apel, Rainer Schmid, Mike McCauley,
//   Alexander Zakharov, Guido Trentalancia, Sebastian Bissbort
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! Handheld multimeters on a serial cable.
//!
//! Every supported meter is its own driver id (`uni-t-ut61e-ser`,
//! `voltcraft-vc820-ser`, ...), as in sigrok, since nothing in the
//! byte stream says which meter is talking. A scan needs `conn=` with
//! the serial port, except for the meters whose cable is a USB HID
//! adapter (`hid/bu86x`, `hid/victor`), which are found on their own;
//! `serialcomm=` overrides the cable's parameters. The
//! scan listens for up to three seconds for a packet that the chipset
//! parser accepts, and acquisition then streams one analog packet per
//! reading until the sample or time limit, or until stopped.
//!
//! `OXROK_SERIAL_RECORD=FILE` records the session for replay.

#![forbid(unsafe_code)]

pub mod models;
pub mod ut32x;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::dmm::{Protocol, Validity};
use crate::serial::{Params, Port};
use models::Model;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
// sigrok lists conn for these meters but cannot read it back, so its
// show output has neither the option nor the connection in the header.
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];

/// How long a scan listens for a valid packet.
const DETECT_TIMEOUT: Duration = Duration::from_secs(3);
/// How long one read waits for data.
const READ_TIMEOUT: Duration = Duration::from_millis(100);
/// The scan's receive buffer (api.c's `buf[128]`).
const BUF_SIZE: usize = 128;
/// The acquisition's receive buffer (protocol.h's `DMM_BUFSIZE`); when
/// it is full of bytes that start no packet it is dropped.
const DMM_BUFSIZE: usize = 256;
/// The default wait for a requested packet before asking again.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(1);

/// The driver of one meter.
pub struct SerialDmmDriver(pub &'static Model);

/// Registers a driver for every supported meter, on serial and on HID
/// cables, and the `UT32x` thermometer.
pub fn register_all(registry: &mut crate::core::Registry) {
    for model in models::MODELS.iter().chain(models::hid_models()) {
        registry.register(Box::new(SerialDmmDriver(model)));
    }
    registry.register(Box::new(ut32x::Ut32xDriver));
}

/// Turns the `VID.PID` form libsigrok's USB drivers take into the HID
/// cable name, so `uni-t-ut61e:conn=1a86.e008` works as in sigrok.
fn hid_conn(conn: &str) -> String {
    if let Some((v, p)) = conn.split_once('.')
        && let (Ok(vid), Ok(pid)) = (u16::from_str_radix(v, 16), u16::from_str_radix(p, 16))
        && let Some(chip) = crate::serial::hid::Chip::for_usb_id(vid, pid)
    {
        return format!("hid/{}", chip.name());
    }
    conn.to_owned()
}

/// Whether the port probed on keeps serving the device. sigrok's scan
/// always ends in `serial_close`, whatever the transport, and
/// `dev_open` opens the port again; `serial_open` sets the parameters
/// and flushes the receive side each time. A UART stays open here
/// instead, because reopening it would toggle the handshake lines
/// again, which resets some adapters; [`DmmDevice`] flushes it when it
/// takes it over, so what a streaming meter sent since the probe does
/// not become the first samples, as sigrok's flush on reopen ensures.
/// A serial over HID cable has no handshake lines, so it is closed and
/// reopened as in sigrok, which also sends its line parameters again
/// (a CH9325 or CP2110 feature report).
fn keep_probed(conn: &str) -> bool {
    !crate::serial::hid::is_hid_name(conn)
}

impl Driver for SerialDmmDriver {
    fn id(&self) -> &'static str {
        self.0.id
    }

    fn long_name(&self) -> &'static str {
        self.0.long_name()
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = self.0.serialcomm.to_owned();
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
        let given = conn.is_some();
        let conn = match conn {
            Some(c) if !self.0.conn.is_empty() => hid_conn(&c),
            Some(c) => c,
            None if !self.0.conn.is_empty() => self.0.conn.to_owned(),
            None => return Ok(Vec::new()),
        };
        let params = Params::parse(&serialcomm)?;
        let mut port = match crate::serial::open(&conn, params) {
            Ok(p) => p,
            // A default cable that is not plugged in is not an error.
            Err(e) if !given => {
                log::debug!("{}: {e}", self.0.id);
                return Ok(Vec::new());
            }
            // Nor is a given port that does not open (or refuses the
            // parameters): sigrok's scan finds no device, it does not
            // fail.
            Err(e) => {
                log::error!("{}: {e}", self.0.id);
                return Ok(Vec::new());
            }
        };
        if let Some(after_open) = self.0.after_open {
            after_open(port.as_mut())?;
        }
        let conn = port.name().to_owned();
        log::info!("Probing serial port {conn}.");
        let found = detect(port.as_mut(), &self.0.protocol, DETECT_TIMEOUT)?;
        if !found {
            log::info!("No {} packets on {conn}.", self.0.id);
            return Ok(Vec::new());
        }
        log::info!("Found device on port {conn}.");
        let port = keep_probed(&conn).then_some(port);
        Ok(vec![Box::new(DmmDevice::new(self.0, conn, params, port))])
    }
}

impl Model {
    fn long_name(&self) -> &'static str {
        // Built once per model: the table is static, the names are few.
        static NAMES: std::sync::OnceLock<std::sync::Mutex<Vec<(&'static str, &'static str)>>> =
            std::sync::OnceLock::new();
        let names = NAMES.get_or_init(|| std::sync::Mutex::new(Vec::new()));
        let mut names = names.lock().expect("names lock");
        if let Some((_, n)) = names.iter().find(|(id, _)| *id == self.id) {
            return n;
        }
        let n: &'static str = Box::leak(format!("{} {}", self.vendor, self.model).into_boxed_str());
        names.push((self.id, n));
        n
    }
}

/// Reads from `port` until a valid packet of `protocol` appears or
/// `timeout` passes. Returns whether one did; leftover bytes are
/// dropped.
pub fn detect(port: &mut dyn Port, protocol: &Protocol, timeout: Duration) -> Result<bool> {
    if let Some(req) = protocol.request {
        port.write(req)?;
    }
    let start = Instant::now();
    let mut buf = Vec::with_capacity(BUF_SIZE);
    let mut byte = [0u8; 1];
    while start.elapsed() < timeout {
        let n = port.read(&mut byte, READ_TIMEOUT)?;
        if n == 0 {
            continue;
        }
        buf.push(byte[0]);
        if protocol.find_packet(&buf).is_some() {
            return Ok(true);
        }
        if buf.len() >= BUF_SIZE {
            buf.drain(..BUF_SIZE / 2);
        }
    }
    Ok(false)
}

/// One meter.
pub struct DmmDevice {
    model: &'static Model,
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limit_samples: u64,
    limit_msec: u64,
    /// A port to use instead of opening `conn`, for replays.
    pending: Option<Box<dyn Port>>,
    port: Arc<std::sync::Mutex<Option<Box<dyn Port>>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl DmmDevice {
    fn new(
        model: &'static Model,
        conn: String,
        params: Params,
        port: Option<Box<dyn Port>>,
    ) -> Self {
        let p = &model.protocol;
        let channels = (0..p.channels)
            .map(|i| {
                let name = p
                    .channel_names
                    .get(i)
                    .map_or_else(|| format!("P{}", i + 1), |n| (*n).to_owned());
                Channel::new(i, ChannelKind::Analog, name)
            })
            .collect();
        DmmDevice {
            model,
            info: DeviceInfo {
                driver: model.id,
                vendor: model.vendor.to_owned(),
                model: model.model.to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels,
            conn,
            params,
            limit_samples: 0,
            limit_msec: 0,
            pending: port,
            port: Arc::new(std::sync::Mutex::new(None)),
            worker: None,
        }
    }

    /// A device of `model` on an already open port, e.g. a replay. No
    /// detection is run.
    pub fn from_port(model: &'static Model, port: Box<dyn Port>) -> Self {
        let name = port.name().to_owned();
        let params = Params::parse(model.serialcomm).expect("table parameters parse");
        Self::new(model, name, params, Some(port))
    }
}

impl Device for DmmDevice {
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
        if self.port.lock().expect("port lock").is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            // The probed port (or a replay): flushed as `serial_open`
            // flushes a port it opens, so input from before the open
            // is not taken for samples.
            Some(mut p) => {
                p.purge()?;
                p
            }
            None => crate::serial::open(&self.conn, self.params)?,
        };
        *self.port.lock().expect("port lock") = Some(port);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        self.port.lock().expect("port lock").take();
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let port = self.port.lock().expect("port lock").take();
        let Some(port) = port else {
            return Err(Error::device("device is not open"));
        };
        let acq = Acquisition {
            model: self.model,
            enabled: self.channels.iter().map(|c| c.enabled).collect(),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
        };
        let slot = Arc::clone(&self.port);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name(format!("{} acquisition", self.model.id))
            .spawn(move || {
                let mut port = port;
                if let Err(e) = acq.run(port.as_mut(), &sink, &stop2) {
                    log::error!("Acquisition failed: {e}.");
                }
                sink.send(Packet::End);
                *slot.lock().expect("port lock") = Some(port);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        Ok(())
    }
}

/// One acquisition, on its own thread.
struct Acquisition {
    model: &'static Model,
    enabled: Vec<bool>,
    limit_samples: u64,
    limit_msec: u64,
}

impl Acquisition {
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let p = &self.model.protocol;
        let request_timeout = if self.model.request_timeout_ms > 0 {
            Duration::from_millis(self.model.request_timeout_ms)
        } else {
            REQUEST_TIMEOUT
        };
        let request_delay = Duration::from_millis(self.model.request_delay_ms);
        sink.send(Packet::header());
        let started = Instant::now();
        let mut samples = 0u64;
        let mut buf: Vec<u8> = Vec::with_capacity(DMM_BUFSIZE);
        let mut chunk = [0u8; DMM_BUFSIZE];
        let mut last_request = None;
        if let Some(req) = p.request {
            port.write(req)?;
            last_request = Some(Instant::now());
        }
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            if self.limit_msec > 0 && started.elapsed() >= Duration::from_millis(self.limit_msec) {
                return Ok(());
            }
            let room = DMM_BUFSIZE - buf.len();
            let n = port.read(&mut chunk[..room.max(1)], READ_TIMEOUT)?;
            if n == 0 {
                if let (Some(req), Some(at)) = (p.request, last_request)
                    && at.elapsed() >= request_timeout
                {
                    port.write(req)?;
                    last_request = Some(Instant::now());
                }
                continue;
            }
            buf.extend_from_slice(&chunk[..n]);
            // As handle_new_data: step over the bytes that start no
            // packet, one at a time, until fewer than a packet's are
            // left or the protocol needs more.
            let mut pos = 0;
            let mut deferred = false;
            while pos < buf.len() {
                if buf.len() - pos < p.packet_size {
                    break;
                }
                let len = match p.frame_at(&buf[pos..]) {
                    Validity::Need => break,
                    Validity::Invalid => {
                        pos += 1;
                        continue;
                    }
                    Validity::Valid(n) => n,
                };
                let packet = &buf[pos..pos + len];
                log::debug!("DMM packet: {}", hex(packet));
                let mut sent = false;
                for r in (p.parse)(packet) {
                    if !self.enabled.get(r.channel).copied().unwrap_or(false) {
                        continue;
                    }
                    let data = AnalogData {
                        channels: vec![r.channel],
                        samples: vec![r.value],
                        mq: r.mq,
                        flags: r.flags,
                        unit: r.unit,
                        digits: r.digits,
                    };
                    if !sink.send(Packet::Analog(data)) {
                        return Ok(());
                    }
                    sent = true;
                }
                if sent {
                    samples += 1;
                }
                pos += len;
                // As req_packet: without a delay the next request goes
                // at once; with one it is put off (req_next_at) until
                // after this read, and so not sent when the limit ends
                // the run.
                if let Some(req) = p.request {
                    if request_delay.is_zero() {
                        port.write(req)?;
                        last_request = Some(Instant::now());
                    } else {
                        deferred = true;
                    }
                }
            }
            buf.drain(..pos);
            // As receive_data: the limit is checked once every packet
            // of a read is out, so a read holding several packets
            // sends them all.
            if self.limit_samples > 0 && samples >= self.limit_samples {
                return Ok(());
            }
            if let (Some(req), true) = (p.request, deferred) {
                std::thread::sleep(request_delay);
                port.write(req)?;
                last_request = Some(Instant::now());
            }
            if buf.len() == DMM_BUFSIZE {
                log::info!("Dropping unparsed data to resync.");
                buf.clear();
            }
        }
    }
}

pub(crate) fn hex(data: &[u8]) -> String {
    data.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hid_cables_are_reopened_uarts_kept() {
        assert!(keep_probed("/dev/ttyUSB0"));
        assert!(keep_probed("COM3"));
        assert!(!keep_probed("hid/ch9325/raw=/dev/hidraw0"));
        assert!(!keep_probed("hid/cp2110/sn=0001"));
        assert!(!keep_probed("hid"));
        assert_eq!(hid_conn("1a86.e008"), "hid/ch9325");
    }

    /// A UART with bytes left over from the probe.
    struct Stale(Arc<std::sync::Mutex<(Vec<u8>, usize)>>);

    impl Port for Stale {
        fn name(&self) -> &'static str {
            "/dev/ttyUSB0"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], _: Duration) -> Result<usize> {
            let mut s = self.0.lock().unwrap();
            let n = buf.len().min(s.0.len());
            buf[..n].copy_from_slice(&s.0[..n]);
            s.0.drain(..n);
            Ok(n)
        }
        fn write(&mut self, _: &[u8]) -> Result<()> {
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            let mut s = self.0.lock().unwrap();
            s.0.clear();
            s.1 += 1;
            Ok(())
        }
    }

    #[test]
    fn the_probed_uart_is_flushed_when_the_device_takes_it() {
        // As sigrok's dev_open: serial_open flushes what arrived since
        // the scan, so a stale packet is not the first sample.
        let state = Arc::new(std::sync::Mutex::new((b"+1234 stale\r\n".to_vec(), 0)));
        let model = models::find("victor-dmm").expect("a model");
        let params = Params::parse("2400/8n1").unwrap();
        let port = Box::new(Stale(Arc::clone(&state)));
        let mut dev = DmmDevice::new(model, "/dev/ttyUSB0".to_owned(), params, Some(port));
        dev.open().unwrap();
        assert_eq!(*state.lock().unwrap(), (Vec::new(), 1));
        // Already open: no second flush.
        dev.open().unwrap();
        assert_eq!(state.lock().unwrap().1, 1);
    }
}
