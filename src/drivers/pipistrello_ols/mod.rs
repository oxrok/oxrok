// Counterpart in sigrok:
//   libsigrok/src/hardware/pipistrello-ols/api.c,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, magnuskarlsson,
//   Daniel Elstner, Soeren Apel, Gerhard Sittig, Martin Ling,
//   Tilman Sauerbeck, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! p-ols: the Pipistrello LX45 FPGA board running the Open Logic
//! Sniffer bitstream, 32 channels at up to 100 MHz (200 MHz on channels
//! 0-15 in demux mode), with a four stage trigger and RLE.
//!
//! The board's FT2232H (`0403:6010`, product string `Pipistrello LX45`)
//! carries SUMP commands on port B ([`protocol`]). The scan opens it as
//! libftdi's `ftdi_usb_open_desc` does (the product string of each
//! `0403:6010` device read and compared, the first match taken), sets
//! port B up (purge, bit mode reset, a latency timer of 16 ms, 16 KiB
//! reads), resets the FPGA five times, asks its ID and metadata, and
//! closes it; opening the device does the same open again, and a
//! capture writes its commands, then reads until the board has sent the
//! capture, newest sample first, and sends it as one block. The device
//! sees the same requests as under sigrok-cli, through
//! [`crate::ftdi::libftdi`], checked with the emulator
//! (`tools/emu/pipistrello-ols.sh`), except where the C driver is
//! wrong ([`protocol`] lists those).
//!
//! Also unlike the C driver: a board whose metadata gives a maximum
//! rate below the 100 MHz default starts at that maximum, or the rate
//! below it the clock divides to (the C driver fails to set its default
//! there, reads back 0 Hz and sends a divider of 0, which samples at
//! 100 MHz, and flags without `FLAG_FILTER`, which oxrok sets as at any
//! rate up to 100 MHz).

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Trigger, TriggerMatch, Value, ValueList,
};
use crate::ftdi::libftdi::{self, Context, FtdiError, Interface};
use crate::usb::{Transport, UsbDeviceInfo};
use protocol::{
    CMD_ID, CMD_METADATA, CMD_RESET, DEFAULT_SAMPLERATE, FLAG_CLOCK_EXTERNAL, FLAG_DEMUX,
    FLAG_EXTERNAL_TEST_MODE, FLAG_FILTER, FLAG_INTERNAL_TEST_MODE, FLAG_RLE, FLAG_SWAP_CHANNELS,
    FTDI_BUF_SIZE, MIN_NUM_SAMPLES, Metadata,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

/// The driver.
pub struct PipistrelloOlsDriver;

static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static DEV_OPTIONS: [OptionDesc; 8] = [
    OptionDesc::new(Key::LimitSamples, Access::GET_SET_LIST),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::PatternMode, Access::GET_SET_LIST),
    OptionDesc::new(Key::ExternalClock, Access { get: true, set: true, list: false }),
    OptionDesc::new(Key::Swap, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::Rle, Access { get: true, set: true, list: false }),
];
static TRIGGER_MATCHES: [TriggerMatch; 4] =
    [TriggerMatch::Zero, TriggerMatch::One, TriggerMatch::Rising, TriggerMatch::Falling];

const PATTERN_NONE: &str = "None";
const PATTERN_EXTERNAL: &str = "External";
const PATTERN_INTERNAL: &str = "Internal";

/// `p_ols_open()`: libftdi's open of port B on a device whose product
/// string matches, then the purge, the bit mode reset, the latency
/// timer and the read size. `None` when the device is another one
/// (libftdi's -3, which the driver does not log).
fn open_port(info: &UsbDeviceInfo) -> Result<Option<Context>> {
    let mut t = match crate::usb::open(info) {
        Ok(t) => t,
        Err(e) => {
            log::debug!("opening the device: {e}");
            let e = FtdiError { code: -4, message: "usb_open() failed" };
            log::error!("Failed to open device {e}");
            return Err(e.into());
        }
    };
    match libftdi::check_product(t.as_mut(), protocol::PRODUCT_STRING) {
        Ok(()) => {}
        Err(e) if e.code == -3 => return Ok(None),
        Err(e) => {
            log::error!("Failed to open device {e}");
            return Err(e.into());
        }
    }
    set_up(t).map(Some)
}

/// `p_ols_open()` from `ftdi_usb_open_dev` on, on a transport whose
/// product string matched.
fn set_up(t: Box<dyn Transport>) -> Result<Context> {
    let mut ctx = Context::open_interface(t, Interface::B).map_err(|e| {
        log::error!("Failed to open device {e}");
        Error::from(e)
    })?;
    if let Err(e) = ctx.tcioflush() {
        log::error!("Failed to purge FTDI RX/TX buffers {e}.");
        return Err(e.into());
    }
    if let Err(e) = ctx.set_bitmode(0xff, crate::ftdi::BitMode::Reset as u8) {
        log::error!("Failed to reset the FTDI chip bitmode {e}.");
        return Err(e.into());
    }
    if let Err(e) = ctx.set_latency_timer(16) {
        log::error!("Failed to set FTDI latency timer {e}.");
        return Err(e.into());
    }
    ctx.read_data_set_chunksize(64 * 1024);
    Ok(ctx)
}

/// `p_ols_close()`.
fn close_port(ctx: Context) -> Result<()> {
    ctx.close().map_err(|e| {
        log::error!("Failed to close FTDI device {e}.");
        Error::from(e)
    })
}

/// `write_shortcommand()` and `write_longcommand()`: one command per
/// write.
fn write_command(ctx: &mut Context, cmd: &[u8]) -> Result<()> {
    if cmd.len() == 1 {
        log::debug!("Sending cmd 0x{:02x}.", cmd[0]);
    } else {
        log::debug!(
            "Sending cmd 0x{:02x} data 0x{:02x}{:02x}{:02x}{:02x}.",
            cmd[0],
            cmd[1],
            cmd[2],
            cmd[3],
            cmd[4]
        );
    }
    match ctx.write_data(cmd) {
        Ok(n) if n == cmd.len() => Ok(()),
        Ok(n) => {
            log::error!("FTDI write error, only {n}/{} bytes written.", cmd.len());
            Err(Error::device("short FTDI write"))
        }
        Err(e) => {
            log::error!("Failed to write FTDI data {e}.");
            Err(e.into())
        }
    }
}

/// `dev_acquisition_stop()`'s five resets, each failure logged.
fn reset_five(ctx: &mut Context) {
    for _ in 0..5 {
        let _ = write_command(ctx, &[CMD_RESET]);
    }
}

/// The scan's probe on an opened port: five resets, the ID, the
/// metadata. `None` when the board does not answer as an OLS.
fn probe(ctx: &mut Context) -> Result<Option<Metadata>> {
    for _ in 0..5 {
        if write_command(ctx, &[CMD_RESET]).is_err() {
            log::error!("Could not reset device. Quitting.");
            return Err(Error::device("could not reset the device"));
        }
    }
    let _ = write_command(ctx, &[CMD_ID]);
    let mut id = [0u8; 4];
    let n = read(ctx, &mut id)?;
    if n == 0 {
        return Ok(None);
    }
    // The C compares the whole four bytes, also after a short read.
    if &id != b"1SLO" && &id != b"1ALS" {
        log::debug!("Not an OLS: ID {:02x?}.", &id[..n]);
        return Ok(None);
    }
    let _ = write_command(ctx, &[CMD_METADATA]);
    let mut buf = [0u8; 64];
    let n = read(ctx, &mut buf)?;
    if n == 0 {
        return Ok(None);
    }
    Ok(Some(protocol::parse_metadata(&buf[..n])))
}

fn read(ctx: &mut Context, buf: &mut [u8]) -> Result<usize> {
    ctx.read_data(buf).map_err(|e| {
        log::error!("Failed to read FTDI data {e}.");
        Error::from(e)
    })
}

impl Driver for PipistrelloOlsDriver {
    fn id(&self) -> &'static str {
        "p-ols"
    }

    fn long_name(&self) -> &'static str {
        "Pipistrello OLS"
    }

    fn scan_options(&self) -> &'static [Key] {
        &[]
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        if let Some((key, _)) = options.first() {
            return Err(Error::UnknownKey(key.id().to_owned()));
        }
        // ftdi_usb_open_desc: the first device with the ids and the
        // product string; any other error ends the scan.
        for info in crate::usb::list_devices()? {
            if info.vendor_id != protocol::VENDOR_ID || info.product_id != protocol::PRODUCT_ID {
                continue;
            }
            let Ok(found) = open_port(&info) else { return Ok(Vec::new()) };
            let Some(mut ctx) = found else { continue };
            let meta = probe(&mut ctx);
            // api.c:162 logs a failed close and keeps the device.
            let _ = close_port(ctx);
            return match meta {
                Ok(Some(meta)) => Ok(vec![Box::new(PolsDevice::new(info, &meta))]),
                _ => Ok(Vec::new()),
            };
        }
        Ok(Vec::new())
    }
}

/// One Pipistrello.
pub struct PolsDevice {
    info: DeviceInfo,
    usb: UsbDeviceInfo,
    channels: Vec<Channel>,
    max_samplebytes: u32,
    max_samplerate: u32,
    samplerate: u64,
    divider: u32,
    flag_reg: u16,
    limit_samples: u64,
    capture_ratio: u64,
    trigger: Option<Trigger>,
    ctx: Arc<Mutex<Option<Context>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl PolsDevice {
    fn new(usb: UsbDeviceInfo, meta: &Metadata) -> Self {
        let mut dev = PolsDevice {
            info: DeviceInfo {
                driver: "p-ols",
                vendor: String::new(),
                model: meta.name.clone(),
                version: meta.version.clone(),
                serial: String::new(),
                conn: None,
            },
            usb,
            channels: (0..meta.channels)
                .map(|i| Channel::new(i, ChannelKind::Logic, i.to_string()))
                .collect(),
            max_samplebytes: meta.max_samplebytes,
            max_samplerate: meta.max_samplerate,
            samplerate: 0,
            divider: 0,
            flag_reg: 0,
            limit_samples: 0,
            capture_ratio: 0,
            trigger: None,
            ctx: Arc::new(Mutex::new(None)),
            worker: None,
        };
        // The default, or the board's maximum below it.
        let mut rate = DEFAULT_SAMPLERATE;
        if meta.max_samplerate != 0 {
            rate = rate.min(u64::from(meta.max_samplerate));
        }
        if dev.set_samplerate(rate).is_err() {
            log::debug!("Failed to set default samplerate ({rate}).");
        }
        dev
    }

    /// `p_ols_set_samplerate()`.
    fn set_samplerate(&mut self, rate: u64) -> Result<()> {
        let Some(r) = protocol::samplerate(rate, self.max_samplerate) else {
            return Err(Error::arg("invalid samplerate"));
        };
        if r.demux {
            log::info!("Enabling demux mode.");
            self.flag_reg |= FLAG_DEMUX;
            self.flag_reg &= !FLAG_FILTER;
        } else {
            log::info!("Disabling demux mode.");
            self.flag_reg &= !FLAG_DEMUX;
            self.flag_reg |= FLAG_FILTER;
        }
        self.divider = r.divider;
        self.samplerate = r.samplerate;
        if r.samplerate != rate {
            log::info!("Can't match samplerate {rate}, using {}.", r.samplerate);
        }
        Ok(())
    }

    /// `pols_channel_mask()`.
    fn channel_mask(&self) -> u32 {
        self.channels
            .iter()
            .filter(|c| c.enabled && c.index < 32)
            .fold(0, |m, c| m | (1 << c.index))
    }

    fn flag(&mut self, flag: u16, on: bool) {
        if on {
            self.flag_reg |= flag;
        } else {
            self.flag_reg &= !flag;
        }
    }

    /// `dev_acquisition_start()` up to `RUN`.
    fn arm(&mut self, ctx: &mut Context) -> Result<protocol::Plan> {
        let channel_mask = self.channel_mask();
        let stages =
            protocol::convert_trigger(self.trigger.as_ref(), channel_mask).map_err(|m| {
                log::error!("{m}");
                log::error!("Failed to configure channels.");
                Error::Arg(m)
            })?;
        let settings = protocol::Settings {
            flag_reg: self.flag_reg,
            channel_mask,
            max_samplebytes: self.max_samplebytes,
            limit_samples: self.limit_samples,
            capture_ratio: self.capture_ratio,
            divider: self.divider,
        };
        let plan = protocol::plan(&settings, &stages).map_err(|m| {
            log::error!("{m}");
            Error::Arg(m)
        })?;
        log::debug!("Setting samplerate to {}Hz (divider {})", self.samplerate, self.divider);
        let f = plan.flag_reg;
        let on = |flag: u16| if f & flag != 0 { "on" } else { "off" };
        log::debug!(
            "Setting intpat {}, extpat {}, RLE {}, noise_filter {}, demux {}",
            on(FLAG_INTERNAL_TEST_MODE),
            on(FLAG_EXTERNAL_TEST_MODE),
            on(FLAG_RLE),
            on(FLAG_FILTER),
            on(FLAG_DEMUX)
        );
        self.flag_reg = plan.flag_reg;
        for cmd in &plan.commands {
            write_command(ctx, cmd)?;
        }
        Ok(plan)
    }
}

impl Device for PolsDevice {
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
            Key::Samplerate => Value::U64(self.samplerate),
            Key::CaptureRatio => Value::U64(self.capture_ratio),
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::PatternMode => Value::Str(
                if self.flag_reg & FLAG_EXTERNAL_TEST_MODE != 0 {
                    PATTERN_EXTERNAL
                } else if self.flag_reg & FLAG_INTERNAL_TEST_MODE != 0 {
                    PATTERN_INTERNAL
                } else {
                    PATTERN_NONE
                }
                .to_owned(),
            ),
            Key::Rle => Value::Bool(self.flag_reg & FLAG_RLE != 0),
            Key::ExternalClock => Value::Bool(self.flag_reg & FLAG_CLOCK_EXTERNAL != 0),
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
                if !(protocol::SAMPLERATE_MIN..=protocol::SAMPLERATE_MAX).contains(&rate) {
                    return Err(Error::arg("invalid samplerate"));
                }
                self.set_samplerate(rate)?;
            }
            Key::LimitSamples => {
                let n = value.as_u64().ok_or_else(bad)?;
                if n < MIN_NUM_SAMPLES {
                    return Err(Error::arg("fewer samples than the device takes"));
                }
                self.limit_samples = n;
            }
            Key::CaptureRatio => {
                let r = value.as_u64().ok_or_else(bad)?;
                // hwdriver.c's check of the key, before the driver.
                if r > 100 {
                    return Err(bad());
                }
                self.capture_ratio = r;
            }
            Key::ExternalClock => {
                let on = value.as_bool().ok_or_else(bad)?;
                log::info!(
                    "{}",
                    if on { "Enabling external clock." } else { "Disabled external clock." }
                );
                self.flag(FLAG_CLOCK_EXTERNAL, on);
            }
            Key::PatternMode => {
                let flag = match value.as_str().ok_or_else(bad)? {
                    PATTERN_NONE => {
                        log::info!("Disabling test modes.");
                        0
                    }
                    PATTERN_INTERNAL => {
                        log::info!("Enabling internal test mode.");
                        FLAG_INTERNAL_TEST_MODE
                    }
                    PATTERN_EXTERNAL => {
                        log::info!("Enabling external test mode.");
                        FLAG_EXTERNAL_TEST_MODE
                    }
                    _ => return Err(bad()),
                };
                self.flag_reg &= !(FLAG_INTERNAL_TEST_MODE | FLAG_EXTERNAL_TEST_MODE);
                self.flag_reg |= flag;
            }
            Key::Swap => {
                let on = value.as_bool().ok_or_else(bad)?;
                log::info!(
                    "{}",
                    if on { "Enabling channel swapping." } else { "Disabling channel swapping." }
                );
                self.flag(FLAG_SWAP_CHANNELS, on);
            }
            Key::Rle => {
                let on = value.as_bool().ok_or_else(bad)?;
                log::info!("{}", if on { "Enabling RLE." } else { "Disabling RLE." });
                self.flag(FLAG_RLE, on);
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
            Key::Samplerate => ValueList::RangeStepU64 {
                min: protocol::SAMPLERATE_MIN,
                max: protocol::SAMPLERATE_MAX,
                step: 1,
            },
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::PatternMode => ValueList::Str(
                [PATTERN_NONE, PATTERN_EXTERNAL, PATTERN_INTERNAL].map(str::to_owned).to_vec(),
            ),
            Key::LimitSamples => {
                if self.flag_reg & FLAG_RLE != 0 {
                    return Err(Error::not_supported("a sample limit with RLE"));
                }
                if self.max_samplebytes == 0 {
                    return Err(Error::not_supported("a sample limit without a memory size"));
                }
                let groups = protocol::group_bytes(protocol::group_mask(self.channel_mask()));
                let max =
                    self.max_samplebytes.checked_div(groups).map_or(MIN_NUM_SAMPLES, u64::from);
                ValueList::MinMaxU64(MIN_NUM_SAMPLES, max)
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        self.trigger = trigger;
        Ok(())
    }

    fn open(&mut self) -> Result<()> {
        if self.ctx.lock().expect("p-ols lock").is_some() {
            return Ok(());
        }
        let Some(ctx) = open_port(&self.usb)? else {
            return Err(Error::device("the device is gone"));
        };
        *self.ctx.lock().expect("p-ols lock") = Some(ctx);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        match self.ctx.lock().expect("p-ols lock").take() {
            Some(ctx) => close_port(ctx),
            None => Ok(()),
        }
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let Some(mut ctx) = self.ctx.lock().expect("p-ols lock").take() else {
            return Err(Error::device("device is not open"));
        };
        let plan = match self.arm(&mut ctx) {
            Ok(plan) => plan,
            Err(e) => {
                // libsigrok's session stops a device whose start
                // failed: the five resets of dev_acquisition_stop().
                reset_five(&mut ctx);
                *self.ctx.lock().expect("p-ols lock") = Some(ctx);
                return Err(e);
            }
        };
        let limit = self.limit_samples;
        let slot = Arc::clone(&self.ctx);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("p-ols acquisition".into())
            .spawn(move || {
                run(&mut ctx, &plan, limit, &sink, &stop2);
                *slot.lock().expect("p-ols lock") = Some(ctx);
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

/// `p_ols_receive_data()` on its own thread: reads until the capture
/// is in, drains what is left, sends the samples (around the trigger
/// marker), then `dev_acquisition_stop()`. A failed read stops at once
/// with nothing sent, as in the C driver.
fn run(ctx: &mut Context, plan: &protocol::Plan, limit: u64, sink: &PacketSink, stop: &AtomicBool) {
    sink.send(Packet::header());
    let mut outcome = Ok(());
    let mut decoder = protocol::Decoder::new(plan, limit);
    let mut buf = vec![0u8; FTDI_BUF_SIZE];
    let mut received = 0usize;
    let mut complete = false;
    // No pause between reads: a read that brings only the status bytes
    // has waited out the chip's 16 ms latency timer.
    while !stop.load(Ordering::SeqCst) {
        if decoder.done() {
            complete = true;
            break;
        }
        match ctx.read_data(&mut buf) {
            Ok(0) => log::trace!("Received 0 bytes, nothing to do."),
            Ok(n) => {
                log::debug!("Received {n} bytes");
                received += n;
                decoder.feed(&buf[..n]);
            }
            Err(e) => {
                log::error!("Failed to read FTDI data {e}.");
                outcome = Err(e.into());
                break;
            }
        }
    }
    if complete {
        while matches!(ctx.read_data(&mut buf), Ok(n) if n > 0) {}
        log::debug!("Received {received} bytes, {} samples.", decoder.len());
        let (samples, trigger) = decoder.finish();
        let bytes = |s: &[u32]| s.iter().flat_map(|v| v.to_le_bytes()).collect::<Vec<u8>>();
        let at = trigger.unwrap_or(0);
        if at > 0 {
            sink.send(Packet::logic(4, bytes(&samples[..at])));
        }
        if trigger.is_some() {
            sink.send(Packet::Trigger);
        }
        if at < samples.len() {
            sink.send(Packet::logic(4, bytes(&samples[at..])));
        }
    }
    reset_five(ctx);
    sink.end(outcome);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;
    use std::sync::mpsc::sync_channel;

    /// A device on a port opened from `steps`, already through
    /// `p_ols_open()`, with 1 MiB of memory and 8 channels.
    fn opened(steps: &str) -> PolsDevice {
        let text = format!(
            "# oxrok usb fixture 1\n\
             device vid=0403 pid=6010 version=0700 bus=0 addr=0 strings=1,2,0\n\
             claim 1 -> ok\nmps 83 -> 512\n\
             ctl_out 40 00 0000 0002 - -> ok\nctl_out 40 03 04e2 0202 - -> ok\n\
             ctl_out 40 00 0001 0002 - -> ok\nctl_out 40 00 0002 0002 - -> ok\n\
             ctl_out 40 0b 00ff 0002 - -> ok\nctl_out 40 09 0010 0002 - -> ok\n{steps}"
        );
        let ctx = set_up(Box::new(Replay::from_text(&text).unwrap())).unwrap();
        let meta = Metadata { channels: 8, max_samplebytes: 1 << 20, ..Metadata::default() };
        let mut dev = PolsDevice::new(UsbDeviceInfo::default(), &meta);
        *dev.ctx.lock().unwrap() = Some(ctx);
        dev.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        dev
    }

    const RESETS: &str = "bulk_out 04 00 -> 1\nbulk_out 04 00 -> 1\nbulk_out 04 00 -> 1\n\
        bulk_out 04 00 -> 1\nbulk_out 04 00 -> 1\n";

    fn stages(n: usize) -> Trigger {
        let mut t = Trigger::new();
        for i in 0..n {
            t.add(i, 3, TriggerMatch::One);
        }
        t
    }

    #[test]
    fn a_refused_start_sends_the_resets_of_the_stop() {
        // Nothing else is written: libsigrok's session stops a device
        // whose start failed (session.c:834).
        let mut dev = opened(RESETS);
        dev.set_trigger(Some(stages(5))).unwrap();
        let (tx, _rx) = sync_channel(4);
        assert!(dev.start(PacketSink::new(tx, 0)).is_err());
        assert!(dev.ctx.lock().unwrap().is_some(), "the port is kept for the close");
    }

    #[test]
    fn a_capture_sends_samples_around_the_trigger_then_the_resets() {
        let setup = "bulk_out 04 c008000000 -> 5\nbulk_out 04 c108000000 -> 5\n\
            bulk_out 04 c200000000 -> 5\nbulk_out 04 c300000000 -> 5\n\
            bulk_out 04 c400000000 -> 5\nbulk_out 04 c500000000 -> 5\n\
            bulk_out 04 c600000108 -> 5\nbulk_out 04 c700000000 -> 5\n\
            bulk_out 04 c800000000 -> 5\nbulk_out 04 c900000000 -> 5\n\
            bulk_out 04 ca00000300 -> 5\nbulk_out 04 cb00000000 -> 5\n\
            bulk_out 04 cc00000000 -> 5\nbulk_out 04 cd00000000 -> 5\n\
            bulk_out 04 ce00000300 -> 5\nbulk_out 04 cf00000000 -> 5\n\
            bulk_out 04 8000000000 -> 5\nbulk_out 04 8401000000 -> 5\n\
            bulk_out 04 8300000000 -> 5\nbulk_out 04 823a000000 -> 5\nbulk_out 04 01 -> 1\n";
        let steps = format!(
            "{setup}bulk_in 83 16384 -> 0160\nbulk_in 83 16384 -> 01600706050403020100\n\
             bulk_in 83 16384 -> 0160\n{RESETS}"
        );
        let mut dev = opened(&steps);
        dev.set(&Key::LimitSamples, &Value::U64(8), None).unwrap();
        dev.set(&Key::CaptureRatio, &Value::U64(50), None).unwrap();
        dev.set_trigger(Some(stages(1))).unwrap();
        let (tx, rx) = sync_channel(16);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        assert!(matches!(packets[0], Packet::Header { .. }));
        // 8 samples, the match 4 words from the newest: 3 before it.
        let Packet::Logic(pre) = &packets[1] else { panic!("{packets:?}") };
        assert_eq!(pre.data.len(), 3 * 4);
        assert!(matches!(packets[2], Packet::Trigger));
        let Packet::Logic(post) = &packets[3] else { panic!("{packets:?}") };
        assert_eq!(post.data[..4], [3, 0, 0, 0]);
        assert!(matches!(packets[4], Packet::End));
        dev.stop().unwrap();
    }

    #[test]
    fn a_failed_read_sends_no_samples_and_still_the_resets() {
        let setup = "bulk_out 04 c000000000 -> 5\nbulk_out 04 c100000000 -> 5\n\
            bulk_out 04 c200000008 -> 5\nbulk_out 04 c300000000 -> 5\n\
            bulk_out 04 8000000000 -> 5\nbulk_out 04 8400000000 -> 5\n\
            bulk_out 04 8300000000 -> 5\nbulk_out 04 823a000000 -> 5\nbulk_out 04 01 -> 1\n";
        let steps = format!("{setup}bulk_in 83 16384 -> err device \"endpoint stalled\"\n{RESETS}");
        let mut dev = opened(&steps);
        let (tx, rx) = sync_channel(16);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        let packets: Vec<Packet> = rx.iter().map(|(_, p)| p).collect();
        assert_eq!(packets.len(), 3, "{packets:?}");
        assert!(matches!(packets[1], Packet::Failed(_)));
        assert!(matches!(packets[2], Packet::End));
        dev.stop().unwrap();
    }

    /// The model's scan, from `tests/emu/pipistrello-ols/scan.tx`.
    const SCAN: &str = "# oxrok usb fixture 1\n\
        device vid=0403 pid=6010 version=0700 bus=0 addr=0 strings=1,2,0\n\
        ctl_in 00 06 0300 0000 4 -> 04030904\n\
        ctl_in 00 06 0302 0409 255 -> 2203500069007000690073007400720065006c006c006f0020004c00580034003500\n\
        claim 1 -> ok\nmps 83 -> 512\n\
        ctl_out 40 00 0000 0002 - -> ok\nctl_out 40 03 04e2 0202 - -> ok\n\
        ctl_out 40 00 0001 0002 - -> ok\nctl_out 40 00 0002 0002 - -> ok\n\
        ctl_out 40 0b 00ff 0002 - -> ok\nctl_out 40 09 0010 0002 - -> ok\n\
        bulk_out 04 00 -> 1\nbulk_out 04 00 -> 1\nbulk_out 04 00 -> 1\n\
        bulk_out 04 00 -> 1\nbulk_out 04 00 -> 1\nbulk_out 04 02 -> 1\n\
        bulk_in 83 16384 -> 016031414c53\nbulk_out 04 04 -> 1\n\
        bulk_in 83 16384 -> 01600150697069737472656c6c6f204f4c530002332e30370020000000202100100000230bebc200410200\n\
        bulk_in 83 16384 -> 0160\n";

    #[test]
    fn the_scan_probe_as_the_driver_does_it() {
        let mut t: Box<dyn Transport> = Box::new(Replay::from_text(SCAN).unwrap());
        libftdi::check_product(t.as_mut(), protocol::PRODUCT_STRING).unwrap();
        let mut ctx = set_up(t).unwrap();
        let meta = probe(&mut ctx).unwrap().unwrap();
        assert_eq!(meta.name, "Pipistrello OLS");
        assert_eq!(meta.channels, 32);
        let usb = UsbDeviceInfo::default();
        let dev = PolsDevice::new(usb, &meta);
        assert_eq!(dev.info.version, "FPGA version 3.07");
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        assert_eq!(dev.list(&Key::LimitSamples, None).unwrap(), ValueList::MinMaxU64(4, 262_144));
    }

    #[test]
    fn a_board_slower_than_the_default_starts_at_its_maximum() {
        let meta = Metadata { channels: 8, max_samplerate: 50_000_000, ..Metadata::default() };
        let dev = PolsDevice::new(UsbDeviceInfo::default(), &meta);
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(50_000_000));
        assert_eq!(dev.divider, 1);
    }

    #[test]
    fn options_as_the_driver_keeps_them() {
        let meta = Metadata { channels: 32, max_samplebytes: 4096, ..Metadata::default() };
        let mut dev = PolsDevice::new(UsbDeviceInfo::default(), &meta);
        dev.set(&Key::PatternMode, &Value::Str("External".into()), None).unwrap();
        assert_eq!(dev.get(&Key::PatternMode, None).unwrap(), Value::Str("External".into()));
        assert!(dev.set(&Key::PatternMode, &Value::Str("Other".into()), None).is_err());
        dev.set(&Key::Rle, &Value::Bool(true), None).unwrap();
        assert!(dev.list(&Key::LimitSamples, None).is_err());
        assert!(dev.set(&Key::LimitSamples, &Value::U64(3), None).is_err());
        assert!(dev.set(&Key::Samplerate, &Value::U64(5), None).is_err());
        dev.set(&Key::Samplerate, &Value::U64(200_000_000), None).unwrap();
        assert_eq!(dev.flag_reg & (FLAG_DEMUX | FLAG_FILTER), FLAG_DEMUX);
    }
}
