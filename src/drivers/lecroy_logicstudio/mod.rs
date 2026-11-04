// Counterpart in sigrok:
//   libsigrok/src/hardware/lecroy-logicstudio/api.c,
//   libsigrok/src/hardware/lecroy-logicstudio/protocol.c (the acquisition),
//   Copyright (C) 2015 Tilman Sauerbeck <tilman@code-monkey.de>,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Romain Tartière
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! lecroy-logicstudio: the `LeCroy` `LogicStudio` 16, 16 channels at up
//! to 500 MHz, 8 at up to 1 GHz off the list, with a sample memory of
//! 40960 bytes.
//!
//! The FX2 comes up without firmware (`05ff:a001`): the scan uploads
//! `lecroy-logicstudio16-fx2lp.fw` over [`crate::ezusb`] and the device
//! renumerates as `05ff:a002`; opening waits for it. The first capture,
//! and one with another number of channel groups, loads an FPGA bitstream
//! (see [`protocol`]). A capture arms the hardware trigger, waits on the
//! status packets, then reads the sample memory and sends it all at once.
//!
//! The device has no limit keys and ends each capture itself, so
//! sigrok-cli cannot start one (`Driver::ends_captures_itself`): oxrok
//! starts one with `-O`, `-o` or `-t` alone. Scan, open, the transfers and
//! the samples follow `api.c` and `protocol.c` step by step, checked with
//! the emulator against sigrok-cli where it can run and against
//! `tools/libsigrok-run.c` for captures (`tools/emu/lecroy-logicstudio.sh`);
//! never on a board. Unlike the C driver, this one refuses a capture with
//! no trigger (nothing would ever end it), and ends a capture on a failed
//! transfer, a short status packet or a sample endpoint that stays silent,
//! where the C driver hangs.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Trigger, TriggerFeatures, TriggerMatch, Value, ValueList,
};
use crate::usb::{Transport, UsbDeviceInfo, UsbId};
use protocol::{INTR_BUF_SIZE, Link, RateInfo, StageConfig, Status};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const DRIVER: &str = "lecroy-logicstudio";
/// The id while the FX2 lacks its firmware.
pub const ID_LACK_FIRMWARE: UsbId = UsbId { vendor: 0x05ff, product: 0xa001 };
/// The id once it runs.
pub const ID_HAVE_FIRMWARE: UsbId = UsbId { vendor: 0x05ff, product: 0xa002 };
const FX2_FIRMWARE: &str = "lecroy-logicstudio16-fx2lp.fw";
const CONFIGURATION: u8 = 0;
const INTERFACE: u8 = 0;
const MAX_RENUM_DELAY: Duration = Duration::from_secs(3);
/// What a failed start ends with, as sigrok's session reports it.
const START_FAILED: &str = "Failed to start session.";

/// `samplerates[]` of `api.c`: `--show` lists these. [`protocol::RATES`]
/// takes 1 GHz as well.
pub const SAMPLERATES: [u64; 18] = [
    1_000,
    2_500,
    5_000,
    10_000,
    25_000,
    50_000,
    100_000,
    250_000,
    500_000,
    1_000_000,
    2_500_000,
    5_000_000,
    10_000_000,
    25_000_000,
    50_000_000,
    100_000_000,
    250_000_000,
    500_000_000,
];

/// The driver.
pub struct LogicStudioDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
];
static TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];

impl Driver for LogicStudioDriver {
    fn id(&self) -> &'static str {
        DRIVER
    }

    fn long_name(&self) -> &'static str {
        "LeCroy LogicStudio"
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn ends_captures_itself(&self) -> bool {
        true
    }

    /// `scan()`. It lists no scan options, so `conn=` is refused before
    /// this runs, as `check_options()` does in libsigrok.
    fn scan(&self, _options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for info in crate::usb::find_devices(&[ID_HAVE_FIRMWARE, ID_LACK_FIRMWARE])? {
            if info.id() == ID_HAVE_FIRMWARE {
                devices.push(Box::new(LogicStudioDevice::new(info, None)));
            } else if upload(&info).is_ok() {
                // A failure has been logged, and the board is no device.
                let mut dev = LogicStudioDevice::new(info, None);
                dev.fw_updated = Some(Instant::now());
                devices.push(Box::new(dev));
            }
        }
        Ok(devices)
    }
}

/// `ezusb_upload_firmware()`: open, `SET_CONFIGURATION` 0, the image.
fn upload(info: &UsbDeviceInfo) -> Result<()> {
    let image = crate::core::firmware::read(
        FX2_FIRMWARE,
        "extract the firmware with sigrok-util, see docs/DEVICES.md",
    )
    .inspect_err(|_| {
        log::error!("Failed to open resource '{FX2_FIRMWARE}' (use loglevel 5/spew for details).");
    })?;
    log::info!("uploading firmware to device on {}.{}", info.bus, info.address);
    let mut t = crate::usb::open(info).inspect_err(|e| {
        log::error!("failed to open device: {}.", crate::usb::libusb_error_name(e));
    })?;
    t.force_configuration(CONFIGURATION).inspect_err(|e| {
        log::error!("Unable to set configuration: {}", crate::usb::libusb_error_name(e));
    })?;
    crate::ezusb::upload_firmware_logged(t.as_mut(), FX2_FIRMWARE, &image)
}

/// `usb_get_port_path()`'s text, sigrok's `connection_id`.
fn connection_id(usb: &UsbDeviceInfo) -> String {
    if usb.ports.is_empty() {
        return format!("usb/{}.{}", usb.bus, usb.address);
    }
    let ports: Vec<String> = usb.ports.iter().map(u8::to_string).collect();
    format!("usb/{}-{}", usb.bus, ports.join("."))
}

/// One board.
pub struct LogicStudioDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    rate: RateInfo,
    capture_ratio: u64,
    trigger: Option<Trigger>,
    pending: Option<Box<dyn Transport>>,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
    /// When the scan uploaded the firmware: the device renumerates and is
    /// looked for again on open.
    fw_updated: Option<Instant>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl LogicStudioDevice {
    fn new(usb: UsbDeviceInfo, transport: Option<Box<dyn Transport>>) -> Self {
        LogicStudioDevice {
            info: DeviceInfo {
                driver: DRIVER,
                vendor: "LeCroy".into(),
                model: "LogicStudio16".into(),
                version: String::new(),
                serial: String::new(),
                // The driver has no conn key.
                conn: None,
            },
            usb,
            channels: (0..16)
                .map(|i| Channel::new(i, ChannelKind::Logic, format!("D{i}")))
                .collect(),
            rate: protocol::rate_info(500_000_000).expect("in the table"),
            capture_ratio: 50,
            trigger: None,
            pending: transport,
            link: Arc::new(Mutex::new(None)),
            worker: None,
            fw_updated: None,
        }
    }

    /// A device on an already open transport (a replayed recording), its
    /// firmware running.
    pub fn from_transport(transport: Box<dyn Transport>) -> Result<Self> {
        let usb = transport.info().clone();
        if usb.id() != ID_HAVE_FIRMWARE {
            return Err(Error::arg("not a LeCroy LogicStudio 16 with its firmware"));
        }
        Ok(Self::new(usb, Some(transport)))
    }

    fn channel_mask(&self) -> u16 {
        self.channels.iter().filter(|c| c.enabled).fold(0, |m, c| m | 1 << c.index)
    }

    /// `configure_trigger()`'s stages: those with a match on an enabled
    /// channel. A condition on a disabled channel is ignored.
    fn stages(&self) -> Vec<StageConfig> {
        let enabled = |ch: usize| self.channels.iter().any(|c| c.index == ch && c.enabled);
        let Some(t) = &self.trigger else { return Vec::new() };
        t.stages
            .iter()
            .filter(|s| s.conditions.iter().any(|c| enabled(c.channel)))
            .map(|s| {
                StageConfig::from_conditions(s.conditions.iter().filter(|c| enabled(c.channel)))
            })
            .collect()
    }
}

/// Whether `d` is the board scanned as `usb`: by its hub ports, which stay
/// the same when it renumerates (`usb_get_port_path()`).
fn same_device(d: &UsbDeviceInfo, usb: &UsbDeviceInfo) -> bool {
    if d.bus != usb.bus {
        return false;
    }
    if !usb.ports.is_empty() && !d.ports.is_empty() {
        return d.ports == usb.ports;
    }
    // A backend without ports: the address, unknown after an upload.
    usb.id() == ID_LACK_FIRMWARE || d.address == usb.address
}

/// `open_device()`.
fn try_open(usb: &UsbDeviceInfo) -> Result<(UsbDeviceInfo, Link)> {
    let found = crate::usb::find_devices(&[ID_HAVE_FIRMWARE])?
        .into_iter()
        .find(|d| same_device(d, usb))
        .ok_or_else(|| Error::device("device not found"))?;
    let t = crate::usb::open(&found).inspect_err(|e| {
        log::error!("Failed to open device: {}.", crate::usb::libusb_error_name(e));
    })?;
    Ok((found, claim(t)?))
}

fn claim(mut t: Box<dyn Transport>) -> Result<Link> {
    t.claim_interface(INTERFACE).inspect_err(|e| {
        log::error!("Failed to claim interface: {}.", crate::usb::libusb_error_name(e));
    })?;
    Ok(Link::new(t))
}

impl Device for LogicStudioDevice {
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
            Key::Samplerate => Value::U64(self.rate.rate),
            Key::CaptureRatio => Value::U64(self.capture_ratio),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::Samplerate => {
                let rate = value.as_u64().ok_or_else(bad)?;
                self.rate = protocol::rate_info(rate)
                    .ok_or_else(|| Error::Arg(format!("no rate of {rate} Hz")))?;
            }
            Key::CaptureRatio => {
                let r = value.as_u64().ok_or_else(bad)?;
                // sr_config_set() refuses it before the driver sees it
                // (hwdriver.c:757-764).
                if r > 100 {
                    log::error!("Capture ratio must be 0..100.");
                    return Err(Error::arg("capture ratio must be 0..100"));
                }
                self.capture_ratio = r;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::Samplerate => ValueList::U64(SAMPLERATES.to_vec()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `dev_open()`.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() {
            return Ok(());
        }
        let link = if let Some(t) = self.pending.take() {
            claim(t)
        } else if let Some(uploaded) = self.fw_updated {
            // Takes at least 300 ms for the FX2 to leave the bus; then
            // every 100 ms until 3 s after the upload.
            log::info!("Waiting for device to reset.");
            std::thread::sleep(Duration::from_millis(300));
            let (info, link) = loop {
                match try_open(&self.usb) {
                    Ok(found) => break found,
                    Err(e) => {
                        std::thread::sleep(Duration::from_millis(100));
                        if uploaded.elapsed() >= MAX_RENUM_DELAY {
                            log::debug!("Last attempt: {e}.");
                            log::error!("Device failed to renumerate.");
                            return Err(Error::device("the device failed to renumerate"));
                        }
                    }
                }
            };
            log::info!("Device came back after {} ms.", uploaded.elapsed().as_millis());
            self.fw_updated = None;
            self.usb = info;
            Ok(link)
        } else {
            try_open(&self.usb).map(|(info, link)| {
                self.usb = info;
                link
            })
        };
        let link = link.inspect_err(|_| log::error!("Unable to open device."))?;
        *self.link.lock().expect("link lock") = Some(link);
        Ok(())
    }

    /// Joins the acquisition thread, which [`Device::stop`] only asks to
    /// end, and releases the device.
    fn close(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        drop(self.link.lock().expect("link lock").take());
        Ok(())
    }

    fn trigger_features(&self) -> TriggerFeatures {
        // The stages are counted by the commit, as in C: three of them
        // write the channel group first.
        TriggerFeatures::default()
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&self.trigger_features())?;
        }
        self.trigger = trigger;
        Ok(())
    }

    /// `config_commit()` and `dev_acquisition_start()`. A capture without
    /// a trigger is refused: nothing arms the board and nothing looks at
    /// its packets (protocol.c:331, 1119), so it could never end.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(w) = self.worker.take() {
            if !w.handle.is_finished() {
                self.worker = Some(w);
                return Err(Error::Busy);
            }
            let _ = w.handle.join();
        }
        let stages = self.stages();
        if stages.is_empty() {
            return Err(Error::arg(
                "This device captures on its trigger alone: give one with -t on an enabled \
                 channel.",
            ));
        }
        let Some(mut link) = self.link.lock().expect("link lock").take() else {
            return Err(Error::device("device is not open"));
        };
        let conn = connection_id(&self.usb);
        let begun = link
            .setup_acquisition(self.channel_mask(), &stages)
            .inspect_err(|_| {
                log::error!(
                    "Failed to commit {DRIVER} device {conn} settings before starting acquisition."
                );
            })
            .and_then(|()| {
                link.start_acquisition(self.rate, self.capture_ratio).inspect_err(|_| {
                    log::error!("Could not start {DRIVER} device {conn} acquisition.");
                    // libsigrok's session stops a device whose start
                    // failed as well (session.c:831-835).
                    if let Err(e) = link.stop_acquisition() {
                        log::debug!("Stopping after a failed start: {e}.");
                    }
                })
            });
        if begun.is_err() {
            *self.link.lock().expect("link lock") = Some(link);
            return Err(Error::arg(START_FAILED));
        }
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let rate = self.rate;
        let handle = std::thread::Builder::new()
            .name("lecroy-logicstudio acquisition".into())
            .spawn(move || {
                run(&mut link, rate, &sink, &stop2);
                sink.send(Packet::End);
                *slot.lock().expect("link lock") = Some(link);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the capture to end. The thread may have samples to send, so it
    /// is joined by [`Device::close`], not here.
    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// One capture, on its thread.
fn run(link: &mut Link, rate: RateInfo, sink: &PacketSink, stop: &AtomicBool) {
    sink.send(Packet::header());
    let Some(window) = wait_for_capture(link, rate, stop) else {
        return;
    };
    // handle_fetch_samples_done() goes on to read whatever became of its
    // write (protocol.c:244-264), which never ends when it failed.
    if let Err(e) = link.fetch_samples(window.earliest) {
        log::error!("Failed to ask for the samples: {}.", crate::usb::libusb_error_name(&e));
        return;
    }
    let memory = match link.read_memory(|| stop.load(Ordering::SeqCst)) {
        Ok(m) => m,
        Err(e) => {
            // recv_bulk_transfer() takes no status, and resubmits for ever
            // (protocol.c:1017-1027).
            log::error!("Failed to read the samples: {}.", crate::usb::libusb_error_name(&e));
            stop_if_asked(link, stop);
            return;
        }
    };
    let mask = link.mask();
    let (stretches, before) = link.stretches(window);
    for (i, &(from, len)) in stretches.iter().enumerate() {
        if i == before {
            sink.send(Packet::Trigger);
        }
        for payload in protocol::logic_payloads(&memory[from..from + len], mask) {
            sink.send(Packet::logic(2, payload));
        }
    }
    if before == stretches.len() {
        sink.send(Packet::Trigger);
    }
    stop_if_asked(link, stop);
}

/// A stop asked during the fetch: the C driver writes it at once, and
/// goes on to finish the fetch.
fn stop_if_asked(link: &mut Link, stop: &AtomicBool) {
    if stop.load(Ordering::SeqCst)
        && let Err(e) = link.stop_acquisition()
    {
        log::debug!("Stopping the acquisition: {e}.");
    }
}

/// `recv_intr_transfer()` until the capture is complete: its window, or
/// `None` when it ended otherwise. The C driver does not submit the
/// transfer again after a timeout, a failure or a short packet
/// (protocol.c:926-933), and then never ends. A timeout here is another
/// wait, and the others end the capture.
fn wait_for_capture(
    link: &mut Link,
    rate: RateInfo,
    stop: &AtomicBool,
) -> Option<protocol::Window> {
    loop {
        if stop.load(Ordering::SeqCst) {
            if let Err(e) = link.stop_acquisition() {
                log::debug!("Stopping the acquisition: {e}.");
            }
            return None;
        }
        let packet = match link.read_status() {
            Ok(p) => p,
            Err(Error::Timeout) => continue,
            Err(e) => {
                log::error!("Failed to read the status: {}.", crate::usb::libusb_error_name(&e));
                return None;
            }
        };
        if packet.len() != INTR_BUF_SIZE {
            log::error!("Invalid size of interrupt transfer: {}.", packet.len());
            return None;
        }
        if let Status::Complete(w) = link.status(&packet, rate) {
            return Some(w);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::trigger::{TriggerCondition, TriggerStage};
    use crate::usb::record::Replay;

    fn device() -> LogicStudioDevice {
        LogicStudioDevice::new(UsbDeviceInfo::default(), None)
    }

    fn stage(conditions: Vec<TriggerCondition>) -> TriggerStage {
        TriggerStage { conditions, ..TriggerStage::default() }
    }

    fn link(steps: &str) -> Link {
        let text = format!("# oxrok usb fixture 1\ndevice vid=05ff pid=a002\n{steps}");
        Link::new(Box::new(Replay::from_text(&text).unwrap()))
    }

    fn rate() -> RateInfo {
        protocol::rate_info(1_000_000).unwrap()
    }

    #[test]
    fn options_follow_the_c_driver() {
        let mut d = device();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(500_000_000));
        assert_eq!(d.get(&Key::CaptureRatio, None).unwrap(), Value::U64(50));
        // 1 GHz is taken, not listed.
        d.set(&Key::Samplerate, &Value::U64(1_000_000_000), None).unwrap();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(1_000_000_000));
        let Ok(ValueList::U64(rates)) = d.list(&Key::Samplerate, None) else { panic!("rates") };
        assert!(!rates.contains(&1_000_000_000));
        assert!(rates.contains(&500_000_000));
        assert!(d.set(&Key::Samplerate, &Value::U64(3_000_000), None).is_err());
        assert!(d.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::U64(10), None).is_err());
        assert!(d.get(&Key::Conn, None).is_err());
    }

    #[test]
    fn stages_ignore_disabled_channels() {
        let c = |channel, matches| TriggerCondition { channel, matches, value: 0.0 };
        let mut d = device();
        assert!(d.stages().is_empty());
        d.enable_channel(3, false).unwrap();
        d.trigger = Some(Trigger {
            stages: vec![
                stage(vec![c(3, TriggerMatch::Rising)]),
                stage(vec![c(3, TriggerMatch::One), c(4, TriggerMatch::Falling)]),
            ],
            ..Trigger::default()
        });
        assert_eq!(d.stages(), [StageConfig { falling: 1 << 4, ..StageConfig::default() }]);
    }

    /// A capture with no trigger is refused before anything is sent: the
    /// replay has no step for it to take.
    #[test]
    fn no_trigger_is_refused() {
        let text = "# oxrok usb fixture 1\ndevice vid=05ff pid=a002\nclaim 0 -> ok\n";
        let mut d =
            LogicStudioDevice::from_transport(Box::new(Replay::from_text(text).unwrap())).unwrap();
        d.open().unwrap();
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        assert!(d.start(PacketSink::new(tx, 0)).is_err());
        assert!(rx.try_recv().is_err());
    }

    #[test]
    fn only_the_board_with_firmware_opens_a_transport() {
        let text = "# oxrok usb fixture 1\ndevice vid=05ff pid=a001\n";
        assert!(
            LogicStudioDevice::from_transport(Box::new(Replay::from_text(text).unwrap())).is_err()
        );
    }

    #[test]
    fn the_connection_id_is_the_port_path() {
        let usb = UsbDeviceInfo { bus: 3, ports: vec![1, 2], ..UsbDeviceInfo::default() };
        assert_eq!(connection_id(&usb), "usb/3-1.2");
    }

    #[test]
    fn waiting_takes_timeouts_and_other_states() {
        let mut l = link(
            "int_in 81 32 -> err timeout\n\
             int_in 81 32 -> 00{31}09\n\
             int_in 81 32 -> 00{31}05\n\
             int_in 81 32 -> err disconnected\n",
        );
        assert!(wait_for_capture(&mut l, rate(), &AtomicBool::new(false)).is_none());
    }

    #[test]
    fn a_short_status_packet_ends_the_wait() {
        let mut l = link("int_in 81 32 -> 00{16}\n");
        assert!(wait_for_capture(&mut l, rate(), &AtomicBool::new(false)).is_none());
    }

    #[test]
    fn a_stop_writes_the_arm_register() {
        let mut l = link("ctl_out 40 b1 0012 1544 070000070002 -> ok\n");
        assert!(wait_for_capture(&mut l, rate(), &AtomicBool::new(true)).is_none());
    }

    #[test]
    fn the_complete_packet_ends_the_wait() {
        let mut l = link("int_in 81 32 -> 00{31}04\n").with_capture();
        assert!(wait_for_capture(&mut l, rate(), &AtomicBool::new(false)).is_some());
    }
}
