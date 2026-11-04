// Counterpart in sigrok:
//   libsigrok/src/hardware/kingst-la2016/protocol.c,
//   libsigrok/src/hardware/kingst-la2016/protocol.h,
//   Copyright (C) 2022 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   Copyright (C) 2020 Florian Schmidt <schmidt_florian@gmx.de>,
//   Copyright (C) 2013 Marcus Comstedt <marcus@mc.pp.se>,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   with commits by Kevin Grant, Helge Kruse, Dave Platt, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Kingst LA series' host protocol: vendor requests to the FX2
//! firmware, the FPGA registers behind them, the bitstream upload and
//! the two sample layouts.
//!
//! | request | direction | wValue | data |
//! |---|---|---|---|
//! | `0x10` FPGA enable | out | 1 on, 0 off | none |
//! | `0x20` FPGA register | in or out | the register | its bytes (little endian) |
//! | `0x30` bulk start | out | 0 | none: EP6 IN starts |
//! | `0x38` bulk reset | out | 0 | none: EP6 IN FIFO flushed |
//! | `0x50` FPGA init | out: the bitstream size (u32); in: 0 when ready | 0 | |
//! | `0xa2` EEPROM | in | the offset | the bytes |
//!
//! The registers: `0x00` run mode (write 3 run, 0 halt) and run state
//! (read, u16: bit 0 idle, bit 2 triggered, bit 3 post-trigger, the rest
//! `0x85e0`), `0x02` PWM enables, `0x03` capture mode (0 to the sample
//! memory, 1 streaming), `0x08` download window (start, length; u32
//! each), `0x10` sample config (write: u40 sample limit, u40 pre-trigger
//! samples, the upper 24 bits of the pre-trigger memory in bytes, u16
//! clock divider, a zero byte; read: packets, packets before the
//! trigger, write position; u32 each), `0x20` trigger (enabled
//! channels, triggering channels, level channels, high or falling
//! channels; u32 each, written with wIndex 16), `0x68` the threshold
//! DAC's two PWM duties, `0x70` and `0x78` the PWM outputs' period and
//! duty in 200 MHz clocks.
//!
//! The EEPROM identifies the model: at `0x08` its magic, its complement,
//! a second magic and its complement, then a backup copy of the four;
//! at `0x20` the manufacturing date (BCD year and month and their
//! complements). The bitstream goes to bulk OUT endpoint 2 after FPGA
//! init announced its size, in 4096 byte chunks and zero padded to a
//! multiple of 4096.
//!
//! Captures to the sample memory come back on bulk IN endpoint 6 run
//! length encoded: 16 byte units of five (u16 pins, u8 count) packets
//! and a sequence byte on the 16 channel models, 32 byte units of six
//! (u32 pins, u8 count) packets and two sequence bytes on the LA5032.
//! Streams are not compressed: one u16 per enabled channel in turn,
//! each holding 16 samples of it, the first in bit 0.

use crate::core::{Error, Result, TriggerMatch};
use crate::usb::{ControlRequest, Transport, libusb_error_name};
use std::fmt::Write as _;
use std::sync::atomic::{AtomicU16, Ordering};
use std::time::Duration;

/// The FX2's ids, with and without its firmware.
pub const VID: u16 = 0x77a1;
/// The product id.
pub const PID: u16 = 0x01a2;
/// `iProduct` of the running firmware; the bare FX2 has none.
pub const IPRODUCT_INDEX: u8 = 2;
/// The interface the driver claims.
pub const INTERFACE: u8 = 0;
/// The configuration `ezusb_upload_firmware()` sets.
pub const CONFIGURATION: u8 = 1;
/// The bitstream's bulk OUT endpoint.
pub const EP_FPGA_BITSTREAM: u8 = 0x02;
/// The samples' bulk IN endpoint.
pub const EP_CAPTURE_DATA: u8 = 0x86;
/// Bytes per bulk IN transfer (`LA2016_USB_BUFSZ`).
pub const USB_BUFSZ: usize = 512 * 1024;
/// Bulk IN transfers kept in flight (`LA2016_USB_XFER_COUNT`).
pub const USB_XFER_COUNT: usize = 8;
/// `DEFAULT_TIMEOUT_MS`.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(200);
/// `CAPTURE_TIMEOUT_MS`.
pub const CAPTURE_TIMEOUT: Duration = Duration::from_millis(500);
/// How long after the firmware upload the scan looks for the device.
pub const RENUM_CHECK_PERIOD: Duration = Duration::from_millis(3000);
/// How long it waits before the first look.
pub const RENUM_GONE_DELAY: Duration = Duration::from_millis(1800);
/// The pause between two looks.
pub const RENUM_POLL_INTERVAL: Duration = Duration::from_millis(200);
/// The bitstream's chunks and padding (`LA2016_EP2_PADDING`).
pub const EP2_PADDING: usize = 4096;
/// The highest sample limit (`LA2016_NUM_SAMPLES_MAX`), also sent for
/// "no limit".
pub const NUM_SAMPLES_MAX: u64 = 10_000_000_000;
/// The highest PWM output frequency.
pub const MAX_PWM_FREQ: f64 = 20e6;
/// The PWM outputs' clock.
const PWM_CLOCK: f64 = 200e6;
/// Streaming bandwidths warned about, in Mbps.
const STREAM_MBPS_MAX: u64 = 200;
/// Streaming bandwidths below which libsigrok flushes its feed
/// periodically, in Mbps.
const STREAM_PUSH_THR: u64 = 16;

const CMD_FPGA_ENABLE: u8 = 0x10;
const CMD_FPGA_SPI: u8 = 0x20;
const CMD_BULK_START: u8 = 0x30;
const CMD_BULK_RESET: u8 = 0x38;
const CMD_FPGA_INIT: u8 = 0x50;
const CMD_EEPROM: u8 = 0xa2;

const REG_RUN: u16 = 0x00;
const REG_PWM_EN: u16 = 0x02;
const REG_CAPT_MODE: u16 = 0x03;
const REG_BULK: u16 = 0x08;
const REG_SAMPLING: u16 = 0x10;
const REG_TRIGGER: u16 = 0x20;
const REG_THRESHOLD: u16 = 0x68;
const REG_PWM: [u16; 2] = [0x70, 0x78];

const CAPTMODE_TO_RAM: u8 = 0x00;
const CAPTMODE_STREAM: u8 = 0x01;
/// Run mode: halt.
pub const RUNMODE_HALT: u8 = 0x00;
/// Run mode: run.
pub const RUNMODE_RUN: u8 = 0x03;

const RUNSTATE_IDLE_BIT: u16 = 1 << 0;
const RUNSTATE_TRGD_BIT: u16 = 1 << 2;
const RUNSTATE_POST_BIT: u16 = 1 << 3;

/// One entry of libsigrok's model table.
#[derive(Debug, PartialEq, Eq)]
pub struct Model {
    /// The EEPROM magic.
    pub magic: u8,
    /// The second magic byte, 0 for any.
    pub magic2: u8,
    /// The name users see.
    pub name: &'static str,
    /// The bitstream file's stem.
    pub fpga_stem: &'static str,
    /// The highest sample rate.
    pub samplerate: u64,
    /// Logic channels, 16 or 32.
    pub channel_count: usize,
    /// Sample memory in Gbit; 0 for none (streaming only).
    pub memory_bits: u64,
    /// The clock the divider applies to; 0 for the highest rate.
    pub baseclock: u64,
}

const MHZ: u64 = 1_000_000;

/// `models[]` of protocol.c, sorted by magic, the more specific entries
/// first.
pub static MODELS: [Model; 15] = [
    Model::new(0x02, 0x01, "LA2016", "la2016a1", 200, 16, 1, 0),
    Model::new(0x02, 0x00, "LA2016", "la2016", 200, 16, 1, 0),
    Model::new(0x03, 0x01, "LA1016", "la1016a1", 100, 16, 1, 0),
    Model::new(0x03, 0x00, "LA1016", "la1016", 100, 16, 1, 0),
    Model::new(0x04, 0x00, "LA1010", "la1010a0", 100, 16, 0, 800),
    Model::new(0x05, 0x00, "LA5016", "la5016a1", 500, 16, 2, 800),
    Model::new(0x06, 0x00, "LA5032", "la5032a0", 500, 32, 4, 800),
    Model::new(0x07, 0x00, "LA1010", "la1010a1", 100, 16, 0, 800),
    Model::new(0x08, 0x00, "LA2016", "la2016a1", 200, 16, 1, 0),
    Model::new(0x09, 0x00, "LA1016", "la1016a1", 100, 16, 1, 0),
    Model::new(0x0a, 0x00, "LA1010", "la1010a2", 100, 16, 0, 800),
    Model::new(0x0b, 0x10, "LA2016", "la2016a2", 200, 16, 1, 0),
    Model::new(0x0c, 0x10, "LA5016", "la5016a2", 500, 16, 2, 800),
    Model::new(0x0c, 0x00, "LA5016", "la5016a2", 500, 16, 2, 800),
    Model::new(0x41, 0x00, "LA5016", "la5016a1", 500, 16, 2, 800),
];

const KHZ: u64 = 1000;
const RATES_500MHZ: [u64; 14] = [
    20 * KHZ,
    50 * KHZ,
    100 * KHZ,
    200 * KHZ,
    500 * KHZ,
    MHZ,
    2 * MHZ,
    5 * MHZ,
    10 * MHZ,
    20 * MHZ,
    50 * MHZ,
    100 * MHZ,
    200 * MHZ,
    500 * MHZ,
];

impl Model {
    #[allow(clippy::too_many_arguments)]
    const fn new(
        magic: u8,
        magic2: u8,
        name: &'static str,
        fpga_stem: &'static str,
        mhz: u64,
        channel_count: usize,
        memory_bits: u64,
        base_mhz: u64,
    ) -> Model {
        Model {
            magic,
            magic2,
            name,
            fpga_stem,
            samplerate: mhz * MHZ,
            channel_count,
            memory_bits,
            baseclock: base_mhz * MHZ,
        }
    }

    /// `FPGA_FWFILE_FMT`.
    pub fn bitstream(&self) -> String {
        format!("kingst-{}-fpga.bitstream", self.fpga_stem)
    }

    /// The clock the divider applies to.
    pub fn baseclock(&self) -> u64 {
        if self.baseclock == 0 { self.samplerate } else { self.baseclock }
    }

    /// The rates `--show` lists (`rates_500mhz[]` and its shorter
    /// siblings): from 20 kHz to the highest rate.
    pub fn rates(&self) -> &'static [u64] {
        let n = RATES_500MHZ.iter().take_while(|&&r| r <= self.samplerate).count();
        &RATES_500MHZ[..n]
    }

    /// Bytes per sample: one bit per logic channel.
    pub fn unit_size(&self) -> usize {
        if self.channel_count == 32 { 4 } else { 2 }
    }

    /// The sample memory layout: bytes per unit and sequence bytes.
    pub fn layout(&self) -> Layout {
        if self.channel_count == 32 {
            Layout { transfer_size: 32, sequence_size: 2, unit_size: 4 }
        } else {
            Layout { transfer_size: 16, sequence_size: 1, unit_size: 2 }
        }
    }
}

/// How the sample memory packs run length encoded samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    /// Bytes per unit (`transfer_size`).
    pub transfer_size: usize,
    /// Sequence bytes at the end of a unit.
    pub sequence_size: usize,
    /// Bytes of pin values per packet.
    pub unit_size: usize,
}

impl Layout {
    /// Packets per unit (`packets_per_chunk`).
    pub fn packets_per_chunk(&self) -> usize {
        (self.transfer_size - self.sequence_size) / (self.unit_size + 1)
    }
}

/// `MCU_FWFILE_FMT`: the firmware for a product id.
pub fn mcu_firmware(pid: u16) -> String {
    format!("kingst-la-{pid:04x}.fw")
}

/// `threshold_ranges[]`: the voltages `voltage_threshold` takes, each
/// as a low and high bound of the same value.
pub const THRESHOLDS: [f64; 8] = [0.4, 0.6, 0.9, 1.2, 1.4, 2.0, 2.5, 4.0];
/// The default threshold, 1.4 V.
pub const THRESHOLD_IDX_DFLT: usize = 4;

/// `std_double_tuple_idx()`: the first threshold both bounds are within
/// 0.1 V of.
pub fn threshold_index(low: f64, high: f64) -> Option<usize> {
    THRESHOLDS.iter().position(|&t| (t - low).abs() < 0.1 && (t - high).abs() < 0.1)
}

/// `set_threshold_voltage()`'s duties (R56, R79) for a threshold, in
/// single precision as the C code computes them.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn threshold_duties(voltage: f32) -> (f32, u16, u16) {
    let max = 4.0f32;
    let voltage = voltage.clamp(-max, max);
    let (r79, r56) = if f64::from(voltage) >= 2.9 {
        (0, (302.0 * voltage - 363.0) as u16)
    } else if f64::from(voltage) > -0.4 {
        (0x00f2, (302.0 * voltage + 121.0) as u16)
    } else {
        (0x02d7, (302.0 * voltage + 1090.0) as u16)
    };
    (voltage, r56.clamp(10, 1100), r79)
}

/// One PWM output's settings, in single precision as the C driver
/// keeps them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pwm {
    /// Running.
    pub enabled: bool,
    /// Frequency in Hz.
    pub freq: f32,
    /// Duty cycle in percent.
    pub duty: f32,
}

/// `set_pwm_config()`'s register values: the period and the high time
/// in 200 MHz clocks.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn pwm_registers(p: &Pwm) -> (u32, u32) {
    let period = (PWM_CLOCK / f64::from(p.freq)) as u32;
    let duty = (f64::from(period) * f64::from(p.duty) / 100.0 + 0.5) as u32;
    (period, duty)
}

/// The trigger configuration of `set_trigger_config()`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TriggerCfg {
    /// The enabled logic channels.
    pub channels: u32,
    /// The triggering channels.
    pub enabled: u32,
    /// Those of them that trigger on a level.
    pub level: u32,
    /// Those that trigger high or on a falling edge.
    pub high_or_falling: u32,
}

impl TriggerCfg {
    /// The register's 16 bytes.
    pub fn bytes(&self) -> Vec<u8> {
        [self.channels, self.enabled, self.level, self.high_or_falling]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect()
    }
}

/// The trigger matches of `set_trigger_config()`: one stage, one edge
/// at most. `Err` is the C driver's message.
pub fn trigger_cfg(
    channels: u32,
    stages: &[Vec<(usize, TriggerMatch)>],
) -> std::result::Result<TriggerCfg, &'static str> {
    let mut cfg = TriggerCfg { channels, ..TriggerCfg::default() };
    if channels == 0 {
        return Err("Need at least one enabled logic channel.");
    }
    let Some(first) = stages.first() else { return Ok(cfg) };
    if stages.len() > 1 {
        return Err("Only one trigger stage supported for now.");
    }
    for &(index, m) in first {
        let mask = 1u32.checked_shl(u32::try_from(index).unwrap_or(32)).unwrap_or(0);
        match m {
            TriggerMatch::Zero => {
                cfg.level |= mask;
                cfg.high_or_falling &= !mask;
            }
            TriggerMatch::One => {
                cfg.level |= mask;
                cfg.high_or_falling |= mask;
            }
            TriggerMatch::Rising | TriggerMatch::Falling => {
                if cfg.enabled & !cfg.level != 0 {
                    return Err("Device only supports one edge trigger.");
                }
                cfg.level &= !mask;
                if m == TriggerMatch::Rising {
                    cfg.high_or_falling &= !mask;
                } else {
                    cfg.high_or_falling |= mask;
                }
            }
            _ => return Err("Unknown trigger condition."),
        }
        cfg.enabled |= mask;
    }
    Ok(cfg)
}

/// What `set_sample_config()` sends, and what it logs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SampleCfg {
    /// The rate the divider gives.
    pub eff_samplerate: u64,
    /// The clock divider.
    pub divider: u16,
    /// The sample limit sent (10 G for none).
    pub limit_samples: u64,
    /// Samples before the trigger.
    pub pre_trigger_samples: u64,
    /// Sample memory for them, in bytes.
    pub pre_trigger_memory: u64,
}

impl SampleCfg {
    /// The register's 16 bytes.
    pub fn bytes(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(16);
        b.extend_from_slice(&self.limit_samples.to_le_bytes()[..5]);
        b.extend_from_slice(&self.pre_trigger_samples.to_le_bytes()[..5]);
        b.extend_from_slice(&(self.pre_trigger_memory >> 8).to_le_bytes()[..3]);
        b.extend_from_slice(&self.divider.to_le_bytes());
        b.push(0);
        b
    }
}

/// `set_sample_config()`'s arithmetic. `Err` is the C driver's
/// message. Unlike the C code, a rate whose divider does not fit the
/// 16 bit register is refused: the C code truncates it (3051 Hz on the
/// LA2016 gives 65552, sent as 16: 12.5 MHz; 1525 Hz on the 100 MHz
/// models 65573, sent as 37: 2.7 MHz), and at 12207 Hz on the 800 MHz
/// models gets 65536, sent as 0, and divides by it.
pub fn sample_cfg(
    model: &Model,
    samplerate: u64,
    limit_samples: u64,
    trigger_involved: bool,
    capture_ratio: u64,
) -> std::result::Result<SampleCfg, String> {
    if samplerate > model.samplerate {
        return Err(format!("Too high a sample rate: {samplerate}."));
    }
    let baseclock = model.baseclock();
    let divider = baseclock.checked_div(samplerate).and_then(|d| u16::try_from(d).ok());
    let divider = match divider {
        Some(d) if samplerate >= baseclock / 65536 && d != 0 => d,
        _ => return Err(format!("Too low a sample rate: {samplerate}.")),
    };
    let eff_samplerate = (baseclock / u64::from(divider)).min(model.samplerate);
    let mut limit = limit_samples;
    if limit > NUM_SAMPLES_MAX {
        log::warn!("Too high a sample depth: {limit}, capping.");
        limit = NUM_SAMPLES_MAX;
    }
    if limit == 0 {
        limit = NUM_SAMPLES_MAX;
        log::debug!("Passing {limit} to HW for unlimited samples.");
    }
    let (pre_trigger_samples, mut pre_trigger_memory) = if model.memory_bits == 0 {
        log::debug!("Memory-less device, skipping pre-trigger config.");
        (0, 0)
    } else if trigger_involved {
        let mem = model.memory_bits * 1024 * 1024 * 1024 / 8 * capture_ratio / 100;
        (limit * capture_ratio / 100, mem)
    } else {
        log::debug!("No trigger setup, skipping pre-trigger config.");
        (0, 0)
    };
    if pre_trigger_memory < 0x100 {
        pre_trigger_memory = 0x100;
    }
    Ok(SampleCfg {
        eff_samplerate,
        divider,
        limit_samples: limit,
        pre_trigger_samples,
        pre_trigger_memory,
    })
}

/// `sr_hexdump_new()`: bytes as two hex digits each, space separated.
fn hexdump(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len() * 3);
    for (i, b) in data.iter().enumerate() {
        if i > 0 {
            s.push(' ');
        }
        let _ = write!(s, "{b:02x}");
    }
    s
}

/// C's `%#x`: `0` for zero, else `0x` and the digits.
fn alt_hex(v: u16) -> String {
    if v == 0 { "0".to_owned() } else { format!("{v:#x}") }
}

/// Whether the spew level (`-l 5`) is on, which makes the C driver read
/// and dump the FPGA registers.
fn spew() -> bool {
    log::log_enabled!(log::Level::Trace)
}

/// The capture info the device gives after a capture.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CaptureInfo {
    /// Packets in the sample memory.
    pub n_rep_packets: u32,
    /// Packets before the trigger point.
    pub n_rep_packets_before_trigger: u32,
    /// Where the capture ended in the sample memory.
    pub write_pos: u32,
}

/// The FX2 firmware and the FPGA behind it, over an open transport
/// with interface 0 claimed.
pub struct Link {
    t: Box<dyn Transport>,
}

/// `run_state()`'s static `previous_state`, which only rations its log.
static PREVIOUS_STATE: AtomicU16 = AtomicU16::new(0);

impl Link {
    /// A link over `t`.
    pub fn new(t: Box<dyn Transport>) -> Self {
        Link { t }
    }

    /// The transport.
    pub fn transport(&mut self) -> &mut dyn Transport {
        self.t.as_mut()
    }

    /// `la2016_close_usb()`: releases the interface; dropping the link
    /// closes the device.
    pub fn close(mut self) {
        let _ = self.t.release_interface(INTERFACE);
    }

    /// `ctrl_in()`: exactly `len` bytes or an error, logged as the C
    /// driver logs it.
    fn ctrl_in(&mut self, request: u8, value: u16, index: u16, len: usize) -> Result<Vec<u8>> {
        let req = ControlRequest::vendor(request, value, index);
        let (name, r) = match self.t.control_in(req, len, DEFAULT_TIMEOUT) {
            Ok(d) if d.len() == len => return Ok(d),
            // libusb_error_name() of a byte count.
            Ok(d) => {
                ("**UNKNOWN**", Error::Device(format!("short read: {} of {len} bytes", d.len())))
            }
            Err(e) => (libusb_error_name(&e), e),
        };
        log::debug!(
            "USB ctrl in: {len} bytes, req {request} val {} idx {index}: {name}.",
            alt_hex(value)
        );
        log::error!("Cannot read {len} bytes from USB: {name}.");
        Err(r)
    }

    /// `ctrl_out()`.
    fn ctrl_out(&mut self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
        let req = ControlRequest::vendor(request, value, index);
        let Err(e) = self.t.control_out(req, data, DEFAULT_TIMEOUT) else { return Ok(()) };
        let name = libusb_error_name(&e);
        let len = data.len();
        log::debug!(
            "USB ctrl out: {len} bytes, req {request} val {} idx {index}: {name}.",
            alt_hex(value)
        );
        log::error!("Cannot write {len} bytes to USB: {name}.");
        Err(e)
    }

    /// `la2016_dump_fpga_registers()`: at the spew level only, all the
    /// registers read and dumped.
    pub fn dump_fpga_registers(&mut self, caption: &str) {
        if !spew() {
            return;
        }
        let Ok(regs) = self.ctrl_in(CMD_FPGA_SPI, 0, 0, 0x80) else {
            log::error!("Cannot get registers space.");
            return;
        };
        log::trace!("FPGA registers dump: {caption}");
        for (i, line) in regs.chunks(16).enumerate() {
            log::trace!("  {:04x}  {}", i * 16, hexdump(line));
        }
    }

    /// `la2016_identify_device()`: the manufacturing date (quiet when
    /// `show_message` is false, as the scan's retries after an upload
    /// are), then the magic, then the model.
    pub fn identify_device(&mut self, show_message: bool) -> Result<&'static Model> {
        match self.ctrl_in(CMD_EEPROM, 0x20, 0, 4) {
            Err(e) if !show_message => {
                log::debug!("Cannot access EEPROM.");
                return Err(e);
            }
            Err(_) => log::error!("Cannot read manufacture date in EEPROM."),
            Ok(d) => {
                log::trace!("Manufacture date bytes {}.", hexdump(&d));
                log::info!("Manufacture date: 20{:02x}-{:02x}.", d[0], d[1]);
                if d[1] ^ d[3] != 0xff || d[0] ^ d[2] != 0xff {
                    log::warn!("Manufacture date fails checksum test.");
                }
            }
        }
        let b = self.ctrl_in(CMD_EEPROM, 0x08, 0, 8).inspect_err(|_| {
            log::error!("Cannot read EEPROM device identifier bytes.");
        })?;
        log::trace!("EEPROM magic bytes {}.", hexdump(&b));
        let (magic, magic2) = if b[0] ^ b[1] == 0xff && b[2] ^ b[3] == 0xff {
            log::debug!("Using primary magic {:#x} ({:#x}).", b[0], b[2]);
            (b[0], b[2])
        } else if b[4] ^ b[5] == 0xff && b[6] ^ b[7] == 0xff {
            log::debug!("Using secondary magic {:#x} ({:#x}).", b[4], b[6]);
            (b[4], b[6])
        } else if b[0] ^ b[1] == 0xff {
            log::debug!("Using primary magic {:#x}.", b[0]);
            (b[0], 0)
        } else if b[4] ^ b[5] == 0xff {
            log::debug!("Using secondary magic {:#x}.", b[4]);
            (b[4], 0)
        } else {
            log::error!("Cannot find consistent device type identification.");
            (0, 0)
        };
        let model =
            MODELS.iter().find(|m| m.magic == magic && (m.magic2 == 0 || m.magic2 == magic2));
        let Some(model) = model else {
            log::error!("Cannot identify as one of the supported models.");
            return Err(Error::device("unknown model"));
        };
        log::info!(
            "Model '{}', {} channels, max {}MHz.",
            model.name,
            model.channel_count,
            model.samplerate / MHZ
        );
        log::info!("FPGA bitstream file '{}'.", model.bitstream());
        Ok(model)
    }

    /// `check_fpga_bitstream()`: whether the FPGA runs a bitstream the
    /// driver can keep.
    fn check_fpga_bitstream(&mut self) -> Result<()> {
        log::debug!("Checking operation of the FPGA bitstream.");
        self.dump_fpga_registers("bitstream check");
        let init = self.ctrl_in(CMD_FPGA_INIT, 0, 0, 1);
        if init.as_deref().map_or(true, |r| r[0] != 0) {
            log::debug!("FPGA init query failed, or unexpected response.");
            return Err(Error::device("FPGA init query"));
        }
        let Ok(b) = self.ctrl_in(CMD_FPGA_SPI, REG_RUN, 0, 2) else {
            log::debug!("FPGA register access failed (run state).");
            return Err(Error::device("FPGA register access"));
        };
        let run_state = u16::from_le_bytes([b[0], b[1]]);
        log::trace!("FPGA register: run state 0x{run_state:04x}.");
        if run_state != 0 && (run_state & 0x3 != 0x1 || run_state & !0xf != 0x85e0) {
            log::debug!("Unexpected FPGA register content (run state).");
            return Err(Error::device("FPGA run state"));
        }
        let Ok(b) = self.ctrl_in(CMD_FPGA_SPI, REG_PWM_EN, 0, 1) else {
            log::debug!("FPGA register access failed (PWM enable).");
            return Err(Error::device("FPGA register access"));
        };
        log::trace!("FPGA register: PWM enable 0x{:02x}.", b[0]);
        if b[0] & 0x3 != 0 {
            log::debug!("Unexpected FPGA register content (PWM enable).");
            return Err(Error::device("FPGA PWM enable"));
        }
        log::info!("Could re-use current FPGA bitstream. No upload required.");
        Ok(())
    }

    /// `upload_fpga_bitstream()`.
    fn upload_fpga_bitstream(&mut self, name: &str) -> Result<()> {
        log::info!("Uploading FPGA bitstream '{name}'.");
        let Ok(image) = crate::core::firmware::read(name, "see docs/DEVICES.md") else {
            log::error!("Cannot find FPGA bitstream {name}.");
            return Err(Error::Firmware(name.to_owned()));
        };
        let size = u32::try_from(image.len()).unwrap_or(u32::MAX);
        self.ctrl_out(CMD_FPGA_INIT, 0, 0, &size.to_le_bytes()).inspect_err(|_| {
            log::error!("Cannot initiate FPGA bitstream upload.");
        })?;
        let padded = image.len().div_ceil(EP2_PADDING) * EP2_PADDING;
        let zeros = vec![0u8; EP2_PADDING];
        let mut pos = 0;
        while pos < padded {
            let block = if pos < image.len() {
                &image[pos..image.len().min(pos + EP2_PADDING)]
            } else {
                &zeros[..(padded - pos).min(EP2_PADDING)]
            };
            let len = block.len();
            match self.t.bulk_out(EP_FPGA_BITSTREAM, block, DEFAULT_TIMEOUT) {
                Ok(n) if n == len => {}
                Ok(n) => {
                    log::debug!(
                        "Short write for FPGA bitstream, block {pos:#x} len {len}: got {n}."
                    );
                    return Err(Error::device("short bitstream write"));
                }
                Err(e) => {
                    log::debug!(
                        "Cannot write FPGA bitstream, block {:#x} len {len}: {}.",
                        pos,
                        libusb_error_name(&e)
                    );
                    return Err(e);
                }
            }
            pos += len;
        }
        log::info!("FPGA bitstream upload ({} bytes) done.", image.len());
        Ok(())
    }

    /// `enable_fpga_bitstream()`.
    fn enable_fpga_bitstream(&mut self) -> Result<()> {
        let r = self.ctrl_in(CMD_FPGA_INIT, 0, 0, 1).inspect_err(|_| {
            log::error!("Cannot read response after FPGA bitstream upload.");
        })?;
        if r[0] != 0 {
            log::error!("Unexpected FPGA bitstream upload response, got 0x{:02x}, want 0.", r[0]);
            return Err(Error::device("FPGA bitstream upload response"));
        }
        std::thread::sleep(Duration::from_millis(30));
        self.ctrl_out(CMD_FPGA_ENABLE, 0x01, 0, &[]).inspect_err(|_| {
            log::error!("Cannot enable FPGA after bitstream upload.");
        })?;
        std::thread::sleep(Duration::from_millis(40));
        Ok(())
    }

    /// `la2016_init_hardware()`: the bitstream checked and uploaded when
    /// needed, the FPGA enabled, the bulk IN FIFO reset.
    pub fn init_hardware(&mut self, model: &Model) -> Result<()> {
        if self.check_fpga_bitstream().is_err() {
            self.upload_fpga_bitstream(&model.bitstream()).inspect_err(|_| {
                log::error!("Cannot upload FPGA bitstream.");
            })?;
        }
        self.enable_fpga_bitstream().inspect_err(|_| {
            log::error!("Cannot enable FPGA bitstream after upload.");
        })?;
        // The C code takes a failed read's SR_ERR_IO for a run state
        // (0xfff5) and warns about it; the failure is logged already.
        if let Ok(state) = self.run_state()
            && state & 0xfff0 != 0x85e0
        {
            log::warn!("Unexpected run state, want 0x85eX, got 0x{state:04x}.");
        }
        self.ctrl_out(CMD_BULK_RESET, 0, 0, &[]).inspect_err(|_| {
            log::error!("Cannot reset USB bulk transfer.");
        })?;
        log::debug!("Device should be initialized.");
        Ok(())
    }

    /// `run_state()`. The C code returns a failed read's `SR_ERR_IO` as
    /// the state, `0xfff5`, whose idle bit is set; here it is an error.
    pub fn run_state(&mut self) -> Result<u16> {
        let b = self.ctrl_in(CMD_FPGA_SPI, REG_RUN, 0, 2).inspect_err(|_| {
            log::error!("Cannot read run state.");
        })?;
        let state = u16::from_le_bytes([b[0], b[1]]);
        if PREVIOUS_STATE.swap(state, Ordering::Relaxed) == state {
            return Ok(state);
        }
        let step = state & (RUNSTATE_IDLE_BIT | RUNSTATE_TRGD_BIT | RUNSTATE_POST_BIT);
        let label = match step {
            0 => Some("pre-trigger sampling"),
            RUNSTATE_POST_BIT => Some("sampling, waiting for trigger"),
            s if s == RUNSTATE_TRGD_BIT | RUNSTATE_POST_BIT => Some("post-trigger sampling"),
            _ if state & RUNSTATE_IDLE_BIT != 0 => Some("idle"),
            _ => None,
        };
        match label {
            Some(l) => log::debug!("Run state: 0x{state:04x} ({l})."),
            None => log::debug!("Run state: 0x{state:04x}."),
        }
        Ok(state)
    }

    /// Whether a run state is idle.
    pub fn is_idle(state: u16) -> bool {
        state & RUNSTATE_IDLE_BIT != 0
    }

    /// `set_run_mode()`.
    pub fn set_run_mode(&mut self, mode: u8) -> Result<()> {
        self.ctrl_out(CMD_FPGA_SPI, REG_RUN, 0, &[mode]).inspect_err(|_| {
            log::error!("Cannot configure run mode {mode}.");
        })
    }

    /// `set_pwm_config()`: the output disabled, and if it is to run, its
    /// period and duty written and it enabled again.
    pub fn set_pwm_config(&mut self, pwm: &[Pwm; 2], idx: usize) -> Result<()> {
        let p = &pwm[idx];
        log::debug!(
            "PWM config, app spec, ch {idx}, en {}, freq {:.1}, duty {:.1}.",
            u8::from(p.enabled),
            p.freq,
            p.duty
        );
        let (period, duty) = pwm_registers(p);
        log::debug!("PWM config, reg 0x{:04x}, freq {period}, duty {duty}.", REG_PWM[idx]);
        let enable_all =
            pwm.iter().enumerate().fold(0u8, |m, (i, p)| m | (u8::from(p.enabled) << i));
        let enable_cfg = 1u8 << idx;
        log::trace!("PWM config, enable all 0x{enable_all:02x}, cfg 0x{enable_cfg:02x}.");
        log::trace!("PWM config, disabling before param change.");
        self.ctrl_out(CMD_FPGA_SPI, REG_PWM_EN, 0, &[enable_all & !enable_cfg]).inspect_err(
            |_| {
                log::error!("Cannot adjust PWM enabled state.");
            },
        )?;
        if !p.enabled {
            return Ok(());
        }
        log::trace!("PWM config, sending new parameters.");
        let mut buf = period.to_le_bytes().to_vec();
        buf.extend_from_slice(&duty.to_le_bytes());
        self.ctrl_out(CMD_FPGA_SPI, REG_PWM[idx], 0, &buf).inspect_err(|_| {
            log::error!("Cannot change PWM parameters.");
        })?;
        log::trace!("PWM config, enabling after param change.");
        self.ctrl_out(CMD_FPGA_SPI, REG_PWM_EN, 0, &[enable_all | enable_cfg]).inspect_err(|_| {
            log::error!("Cannot adjust PWM enabled state.");
        })
    }

    /// `set_threshold_voltage()`.
    pub fn set_threshold_voltage(&mut self, voltage: f32) -> Result<()> {
        let (voltage, r56, r79) = threshold_duties(voltage);
        log::debug!("Set threshold voltage {voltage:.2}V.");
        log::debug!("Duty cycle values: R56 0x{r56:04x}, R79 0x{r79:04x}.");
        let mut buf = r56.to_le_bytes().to_vec();
        buf.extend_from_slice(&r79.to_le_bytes());
        self.ctrl_out(CMD_FPGA_SPI, REG_THRESHOLD, 0, &buf).inspect_err(|_| {
            log::error!("Cannot set threshold voltage {voltage:.2}V.");
        })
    }

    /// `la2016_setup_acquisition()`: threshold, capture mode, trigger
    /// and sample configs. Whether a hardware trigger is involved.
    pub fn setup_acquisition(&mut self, s: &Setup<'_>) -> Result<bool> {
        self.set_threshold_voltage(s.voltage)?;
        let mode = if s.continuous { CAPTMODE_STREAM } else { CAPTMODE_TO_RAM };
        self.ctrl_out(CMD_FPGA_SPI, REG_CAPT_MODE, 0, &[mode]).inspect_err(|_| {
            log::error!("Cannot send command to stop sampling.");
        })?;
        let trigger_involved = self.set_trigger_config(s)?;
        self.set_sample_config(s, trigger_involved)?;
        Ok(trigger_involved)
    }

    /// `set_trigger_config()`.
    fn set_trigger_config(&mut self, s: &Setup<'_>) -> Result<bool> {
        let mut cfg = trigger_cfg(s.channel_mask, s.trigger).map_err(|msg| {
            log::error!("{msg}");
            Error::arg(msg)
        })?;
        log::debug!(
            "Set trigger config: enabled-channels 0x{:04x}, triggering-channels 0x{:04x}, \
             level-triggered 0x{:04x}, high/falling 0x{:04x}.",
            cfg.channels,
            cfg.enabled,
            cfg.level,
            cfg.high_or_falling
        );
        if s.model.memory_bits == 0 || s.continuous {
            if s.model.memory_bits == 0 {
                log::debug!("Device without memory. No hardware triggers.");
            } else {
                log::debug!("Streaming mode. No hardware triggers.");
            }
            cfg = TriggerCfg { channels: cfg.channels, ..TriggerCfg::default() };
        }
        self.ctrl_out(CMD_FPGA_SPI, REG_TRIGGER, 16, &cfg.bytes()).inspect_err(|_| {
            log::error!("Cannot setup trigger configuration.");
        })?;
        Ok(cfg.enabled != 0)
    }

    /// `set_sample_config()`.
    fn set_sample_config(&mut self, s: &Setup<'_>, trigger_involved: bool) -> Result<()> {
        let cfg =
            sample_cfg(s.model, s.samplerate, s.limit_samples, trigger_involved, s.capture_ratio)
                .map_err(|msg| {
                log::error!("{msg}");
                Error::Arg(msg)
            })?;
        log::debug!(
            "Set sample config: {}kHz (div {}), {} samples.",
            cfg.eff_samplerate / KHZ,
            cfg.divider,
            cfg.limit_samples
        );
        log::debug!(
            "Capture ratio {}%, count {}, mem {}.",
            s.capture_ratio,
            cfg.pre_trigger_samples,
            cfg.pre_trigger_memory
        );
        if s.continuous {
            let count = u64::from(s.channel_mask.count_ones());
            let product = cfg.eff_samplerate * count;
            log::debug!("Streaming: channel count {count}, product {product}.");
            let mbps = product / MHZ;
            if mbps >= STREAM_MBPS_MAX {
                log::warn!("High USB stream bandwidth: {mbps}Mbps.");
            }
            if mbps < STREAM_PUSH_THR {
                log::debug!("Streaming: low Mbps, suggest periodic flush.");
            }
        }
        self.ctrl_out(CMD_FPGA_SPI, REG_SAMPLING, 0, &cfg.bytes()).inspect_err(|_| {
            log::error!("Cannot setup acquisition configuration.");
        })
    }

    /// `CMD_BULK_RESET`, with no message of its own.
    pub fn bulk_reset(&mut self) -> Result<()> {
        self.ctrl_out(CMD_BULK_RESET, 0, 0, &[])
    }

    /// `CMD_BULK_START`.
    pub fn bulk_start(&mut self) -> Result<()> {
        self.ctrl_out(CMD_BULK_START, 0, 0, &[]).inspect_err(|_| {
            log::error!("Cannot start USB bulk transfers.");
        })
    }

    /// `get_capture_info()`.
    pub fn capture_info(&mut self, packets_per_chunk: u32) -> Result<CaptureInfo> {
        let b = self.ctrl_in(CMD_FPGA_SPI, REG_SAMPLING, 0, 16).inspect_err(|_| {
            log::error!("Cannot read capture info.");
        })?;
        let u = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        let info = CaptureInfo {
            n_rep_packets: u(0),
            n_rep_packets_before_trigger: u(4),
            write_pos: u(8),
        };
        log::debug!(
            "Capture info: n_rep_packets: 0x{0:08x}/{0}, before_trigger: 0x{1:08x}/{1}, \
             write_pos: 0x{2:08x}/{2}.",
            info.n_rep_packets,
            info.n_rep_packets_before_trigger,
            info.write_pos
        );
        if !info.n_rep_packets.is_multiple_of(packets_per_chunk) {
            log::warn!(
                "Unexpected packets count {}, not a multiple of {packets_per_chunk}.",
                info.n_rep_packets
            );
        }
        Ok(info)
    }

    /// The `REG_BULK` write of `la2016_start_download()`: the window the
    /// download reads.
    pub fn bulk_window(&mut self, read_pos: u32, len: u32) -> Result<()> {
        let mut buf = read_pos.to_le_bytes().to_vec();
        buf.extend_from_slice(&len.to_le_bytes());
        self.ctrl_out(CMD_FPGA_SPI, REG_BULK, 0, &buf).inspect_err(|_| {
            log::error!("Cannot send USB bulk config.");
        })
    }
}

/// What `la2016_setup_acquisition()` takes.
pub struct Setup<'a> {
    /// The model.
    pub model: &'static Model,
    /// The threshold voltage.
    pub voltage: f32,
    /// Streaming rather than a capture to the sample memory.
    pub continuous: bool,
    /// The enabled logic channels.
    pub channel_mask: u32,
    /// The trigger's stages, as (channel index, match) pairs.
    pub trigger: &'a [Vec<(usize, TriggerMatch)>],
    /// The sample rate set.
    pub samplerate: u64,
    /// The sample limit, 0 for none.
    pub limit_samples: u64,
    /// The capture ratio in percent.
    pub capture_ratio: u64,
}

/// What a chunk of samples gives the session feed.
#[derive(Debug, PartialEq, Eq)]
pub enum Feed {
    /// Samples, `unit_size` bytes each.
    Samples(Vec<u8>),
    /// The trigger point.
    Trigger,
}

/// Samples gathered for the feed (`feed_queue_logic`), cut at the
/// sample limit: the C driver passes on every sample of the transfer in
/// which the limit falls.
struct Queue {
    unit_size: usize,
    limit: u64,
    sent: u64,
    buf: Vec<u8>,
    out: Vec<Feed>,
}

/// Samples per feed packet, as the C driver's 4 MiB feed queue.
const QUEUE_BYTES: usize = 4 * 1024 * 1024;

impl Queue {
    fn new(unit_size: usize, limit: u64) -> Self {
        Queue { unit_size, limit, sent: 0, buf: Vec::new(), out: Vec::new() }
    }

    fn submit(&mut self, value: u32, count: u64) {
        let room =
            if self.limit == 0 { count } else { count.min(self.limit.saturating_sub(self.sent)) };
        let bytes = &value.to_le_bytes()[..self.unit_size];
        for _ in 0..room {
            self.buf.extend_from_slice(bytes);
            if self.buf.len() >= QUEUE_BYTES {
                self.flush();
            }
        }
        self.sent += room;
    }

    fn flush(&mut self) {
        if !self.buf.is_empty() {
            self.out.push(Feed::Samples(std::mem::take(&mut self.buf)));
        }
    }

    fn trigger(&mut self) {
        self.flush();
        self.out.push(Feed::Trigger);
    }

    fn take(&mut self) -> Vec<Feed> {
        self.flush();
        std::mem::take(&mut self.out)
    }
}

/// `send_chunk()`: a download's run length encoded units.
pub struct Download {
    layout: Layout,
    /// Bytes still to come.
    pub n_bytes_to_read: u32,
    trigger_involved: bool,
    trigger_marked: bool,
    before_trigger: u32,
    n_reps_until_trigger: u32,
    total_samples: u64,
    limit: u64,
    samplerate: u64,
    queue: Queue,
    finished: bool,
}

impl Download {
    /// A download of `n_bytes_to_read` bytes. `limit` is the sample
    /// limit (0 for none); the trigger is involved when the capture had
    /// a hardware trigger.
    pub fn new(
        layout: Layout,
        info: &CaptureInfo,
        n_bytes_to_read: u32,
        trigger_involved: bool,
        limit: u64,
        samplerate: u64,
    ) -> Self {
        Download {
            layout,
            n_bytes_to_read,
            trigger_involved,
            trigger_marked: false,
            before_trigger: info.n_rep_packets_before_trigger,
            n_reps_until_trigger: info.n_rep_packets_before_trigger,
            total_samples: 0,
            limit,
            samplerate,
            queue: Queue::new(layout.unit_size, limit),
            finished: false,
        }
    }

    /// Whether the download is complete.
    pub fn finished(&self) -> bool {
        self.finished
    }

    /// One transfer's bytes: the samples and trigger it gives.
    #[allow(clippy::cast_precision_loss)]
    pub fn chunk(&mut self, data: &[u8]) -> Vec<Feed> {
        if self.finished {
            return Vec::new();
        }
        if self.trigger_involved && !self.trigger_marked && self.before_trigger == 0 {
            self.queue.trigger();
            self.trigger_marked = true;
        }
        let len = u32::try_from(data.len()).unwrap_or(u32::MAX);
        self.n_bytes_to_read = self.n_bytes_to_read.saturating_sub(len);
        let l = self.layout;
        for unit in data.chunks_exact(l.transfer_size) {
            for p in unit[..unit.len() - l.sequence_size].chunks_exact(l.unit_size + 1) {
                let mut v = [0u8; 4];
                v[..l.unit_size].copy_from_slice(&p[..l.unit_size]);
                let value = u32::from_le_bytes(v);
                let reps = u64::from(p[l.unit_size]);
                self.total_samples += reps;
                self.queue.submit(value, reps);
                if self.trigger_involved && !self.trigger_marked {
                    self.n_reps_until_trigger = self.n_reps_until_trigger.wrapping_sub(1);
                    if self.n_reps_until_trigger == 0 {
                        // A trigger point past the sample limit is not
                        // passed on: its samples were not either.
                        if self.limit == 0 || self.total_samples <= self.limit {
                            self.queue.trigger();
                        }
                        self.trigger_marked = true;
                        log::debug!(
                            "Trigger position after {} samples, {:.6}ms.",
                            self.total_samples,
                            self.total_samples as f64 / self.samplerate as f64 * 1e3
                        );
                    }
                }
            }
        }
        if self.n_bytes_to_read == 0 {
            self.finished = true;
        } else {
            log::debug!("{} more bytes to download from the device.", self.n_bytes_to_read);
        }
        if !self.finished && self.limit != 0 && self.total_samples >= self.limit {
            log::debug!("Acquisition limit reached.");
            self.finished = true;
        }
        if self.finished {
            log::debug!("Download finished, flushing session feed queue.");
        }
        log::debug!("Total samples after chunk: {}.", self.total_samples);
        self.queue.take()
    }
}

/// `stream_data()`: a stream's words, 16 samples of one enabled channel
/// each, the channels in turn.
pub struct Stream {
    masks: Vec<u32>,
    index: usize,
    sample_data: [u32; 16],
    total_samples: u64,
    limit: u64,
    queue: Queue,
    finished: bool,
}

impl Stream {
    /// A stream of the channels in `channel_mask`.
    pub fn new(channel_mask: u32, unit_size: usize, limit: u64) -> Self {
        Stream {
            masks: (0..32).map(|i| 1u32 << i).filter(|m| channel_mask & m != 0).collect(),
            index: 0,
            sample_data: [0; 16],
            total_samples: 0,
            limit,
            queue: Queue::new(unit_size, limit),
            finished: false,
        }
    }

    /// Whether the sample limit was reached.
    pub fn finished(&self) -> bool {
        self.finished
    }

    /// One transfer's bytes.
    pub fn chunk(&mut self, data: &[u8]) -> Vec<Feed> {
        if self.finished {
            return Vec::new();
        }
        log::debug!("Stream mode, got another chunk: length {}.", data.len());
        for &w in data.as_chunks::<2>().0 {
            let word = u16::from_le_bytes(w);
            let mask = self.masks[self.index];
            for (bit, sample) in self.sample_data.iter_mut().enumerate() {
                if word & (1 << bit) != 0 {
                    *sample |= mask;
                }
            }
            self.index += 1;
            if self.index != self.masks.len() {
                continue;
            }
            for v in self.sample_data {
                self.queue.submit(v, 1);
            }
            self.total_samples += 16;
            self.sample_data = [0; 16];
            self.index = 0;
        }
        if self.limit != 0 && self.total_samples >= self.limit {
            log::debug!("Acquisition end reached (sw limits).");
            self.finished = true;
        }
        if self.finished {
            log::debug!("Stream receive done, flushing session feed queue.");
        }
        log::debug!("Total samples after chunk: {}.", self.total_samples);
        self.queue.take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;

    #[test]
    fn models_rates_and_names() {
        let la2016 = &MODELS[0];
        assert_eq!(la2016.bitstream(), "kingst-la2016a1-fpga.bitstream");
        assert_eq!(la2016.rates().len(), 13);
        assert_eq!(*la2016.rates().last().unwrap(), 200 * MHZ);
        assert_eq!(MODELS[3].rates().len(), 12);
        assert_eq!(MODELS[6].rates().len(), 14);
        assert_eq!(MODELS[6].baseclock(), 800 * MHZ);
        assert_eq!(la2016.baseclock(), 200 * MHZ);
        assert_eq!(mcu_firmware(PID), "kingst-la-01a2.fw");
        assert_eq!(MODELS[6].layout().packets_per_chunk(), 6);
        assert_eq!(la2016.layout().packets_per_chunk(), 5);
    }

    #[test]
    fn thresholds_match_std_double_tuple_idx() {
        assert_eq!(threshold_index(2.5, 2.5), Some(6));
        assert_eq!(threshold_index(3.3, 3.3), None);
        // Within 0.1 of 0.9 (0.0999...).
        assert_eq!(threshold_index(1.0, 1.0), Some(2));
        assert_eq!(threshold_index(1.4, 2.0), None);
    }

    /// The register values of the emulator's goldens (threshold 1.4,
    /// 0.4 and 4.0 V).
    #[test]
    fn threshold_duties_in_single_precision() {
        assert_eq!(threshold_duties(1.4), (1.4, 0x021f, 0x00f2));
        assert_eq!(threshold_duties(0.4), (0.4, 0x00f1, 0x00f2));
        assert_eq!(threshold_duties(4.0), (4.0, 0x034d, 0));
        assert_eq!(threshold_duties(9.0).0, 4.0);
    }

    #[test]
    fn pwm_registers_as_set_pwm_config() {
        let p = |freq, duty| pwm_registers(&Pwm { enabled: true, freq, duty });
        assert_eq!(p(12345.0, 50.0), (16200, 8100));
        assert_eq!(p(12345.0, 12.5), (16200, 2025));
        assert_eq!(p(20e6, 50.0), (10, 5));
        assert_eq!(p(1000.0, 50.0), (200_000, 100_000));
    }

    #[test]
    fn trigger_configs() {
        use TriggerMatch::{Falling, One, Rising, Zero};
        let t = |stages: &[Vec<(usize, TriggerMatch)>]| trigger_cfg(0xffff, stages);
        assert_eq!(
            t(&[]).unwrap().bytes(),
            [0xff, 0xff, 0, 0].iter().chain(&[0; 12]).copied().collect::<Vec<_>>()
        );
        let c = t(&[vec![(1, Zero), (0, Rising)]]).unwrap();
        assert_eq!(c, TriggerCfg { channels: 0xffff, enabled: 3, level: 2, high_or_falling: 0 });
        let c = t(&[vec![(0, One), (1, One), (2, Zero)]]).unwrap();
        assert_eq!(c, TriggerCfg { channels: 0xffff, enabled: 7, level: 7, high_or_falling: 3 });
        assert_eq!(t(&[vec![(0, Falling)]]).unwrap().high_or_falling, 1);
        assert_eq!(
            t(&[vec![(0, Rising), (1, Falling)]]),
            Err("Device only supports one edge trigger.")
        );
        assert_eq!(
            t(&[vec![(0, Falling), (1, Rising)]]),
            Err("Device only supports one edge trigger.")
        );
        // A level after the edge is fine.
        assert!(t(&[vec![(0, Rising), (1, One)]]).is_ok());
        assert_eq!(
            t(&[vec![(0, One)], vec![(1, One)]]),
            Err("Only one trigger stage supported for now.")
        );
        assert_eq!(t(&[vec![(0, TriggerMatch::Edge)]]), Err("Unknown trigger condition."));
        assert_eq!(trigger_cfg(0, &[]), Err("Need at least one enabled logic channel."));
    }

    /// The bytes of the emulator's goldens.
    #[test]
    fn sample_configs() {
        let la2016 = &MODELS[0];
        let hex = |c: SampleCfg| {
            c.bytes().iter().fold(String::new(), |mut h, b| {
                let _ = write!(h, "{b:02x}");
                h
            })
        };
        let c = sample_cfg(la2016, 200 * MHZ, 8, false, 50).unwrap();
        assert_eq!(hex(c), "08000000000000000000010000010000");
        let c = sample_cfg(la2016, MHZ, 100, true, 50).unwrap();
        assert_eq!(hex(c), "64000000003200000000000004c80000");
        let c = sample_cfg(la2016, 20 * KHZ, 0, false, 50).unwrap();
        assert_eq!(hex(c), "00e40b54020000000000010000102700");
        assert_eq!(c.limit_samples, NUM_SAMPLES_MAX);
        let la5016 = &MODELS[5];
        let c = sample_cfg(la5016, 200 * MHZ, 50, true, 50).unwrap();
        assert_eq!(hex(c), "32000000001900000000000008040000");
        assert_eq!(c.eff_samplerate, 200 * MHZ);
        let c = sample_cfg(la5016, 500 * MHZ, 50, false, 50).unwrap();
        assert_eq!((c.divider, c.eff_samplerate), (1, 500 * MHZ));
        let la1010 = &MODELS[7];
        let c = sample_cfg(la1010, MHZ, 100, true, 50).unwrap();
        assert_eq!(hex(c), "64000000000000000000010000200300");
        assert_eq!(c.eff_samplerate, MHZ);
        // 3 MHz from 200 MHz: divider 66.
        assert_eq!(sample_cfg(la2016, 3 * MHZ, 40, false, 50).unwrap().divider, 66);
    }

    #[test]
    fn rates_out_of_range_are_refused() {
        let la2016 = &MODELS[0];
        assert_eq!(
            sample_cfg(la2016, 500 * MHZ, 8, false, 50),
            Err("Too high a sample rate: 500000000.".into())
        );
        assert_eq!(
            sample_cfg(la2016, 1000, 8, false, 50),
            Err("Too low a sample rate: 1000.".into())
        );
        assert_eq!(sample_cfg(la2016, 0, 8, false, 50), Err("Too low a sample rate: 0.".into()));
        // The C driver takes 3051 Hz and truncates the divider 65552 to
        // 16 bits; 3052 Hz fits.
        assert_eq!(
            sample_cfg(la2016, 3051, 8, false, 50),
            Err("Too low a sample rate: 3051.".into())
        );
        assert_eq!(sample_cfg(la2016, 3052, 8, false, 50).unwrap().divider, 65530);
        // The same at 100 MHz: 65573, truncated to 37.
        let la1016 = &MODELS[3];
        assert_eq!(
            sample_cfg(la1016, 1525, 8, false, 50),
            Err("Too low a sample rate: 1525.".into())
        );
        assert_eq!(sample_cfg(la1016, 1526, 8, false, 50).unwrap().divider, 65530);
        // On the 800 MHz models the C driver's divider for 12207 Hz is
        // 65536, truncated to 0, which it divides by.
        let la5016 = &MODELS[5];
        assert_eq!(
            sample_cfg(la5016, 12207, 8, false, 50),
            Err("Too low a sample rate: 12207.".into())
        );
        assert_eq!(sample_cfg(la5016, 12208, 8, false, 50).unwrap().divider, 65530);
    }

    fn units16(packets: &[(u16, u8)]) -> Vec<u8> {
        let mut out = Vec::new();
        for (seq, unit) in packets.chunks(5).enumerate() {
            for &(v, n) in unit {
                out.extend_from_slice(&v.to_le_bytes());
                out.push(n);
            }
            for _ in unit.len()..5 {
                out.extend_from_slice(&[0, 0, 0]);
            }
            out.push(seq as u8);
        }
        out
    }

    fn samples16(runs: &[(u16, usize)]) -> Vec<u8> {
        runs.iter().flat_map(|&(v, n)| std::iter::repeat_n(v.to_le_bytes(), n).flatten()).collect()
    }

    #[test]
    fn a_download_unpacks_runs_and_marks_the_trigger() {
        let layout = MODELS[0].layout();
        let info = CaptureInfo { n_rep_packets: 5, n_rep_packets_before_trigger: 2, write_pos: 0 };
        let mut d = Download::new(layout, &info, 16, true, 10, MHZ);
        let data = units16(&[(1, 2), (2, 3), (3, 4), (4, 1), (5, 0)]);
        let feed = d.chunk(&data);
        assert_eq!(
            feed,
            [
                Feed::Samples(samples16(&[(1, 2), (2, 3)])),
                Feed::Trigger,
                Feed::Samples(samples16(&[(3, 4), (4, 1)])),
            ]
        );
        assert!(d.finished());
    }

    #[test]
    fn a_trigger_before_the_first_packet() {
        let layout = MODELS[0].layout();
        let info = CaptureInfo { n_rep_packets: 5, n_rep_packets_before_trigger: 0, write_pos: 0 };
        let mut d = Download::new(layout, &info, 32, true, 0, MHZ);
        let feed = d.chunk(&units16(&[(7, 3)]));
        assert_eq!(feed, [Feed::Trigger, Feed::Samples(samples16(&[(7, 3)]))]);
        assert!(!d.finished());
        assert_eq!(d.n_bytes_to_read, 16);
    }

    /// The C driver passes on the whole transfer in which the limit
    /// falls; the port stops at the limit, and ends the download there.
    #[test]
    fn a_download_stops_at_the_sample_limit() {
        let layout = MODELS[0].layout();
        let info = CaptureInfo { n_rep_packets: 10, n_rep_packets_before_trigger: 0, write_pos: 0 };
        let mut d = Download::new(layout, &info, 64, false, 5, MHZ);
        let feed = d.chunk(&units16(&[(1, 4), (2, 4), (3, 4)]));
        assert_eq!(feed, [Feed::Samples(samples16(&[(1, 4), (2, 1)]))]);
        assert!(d.finished());
        assert_eq!(d.chunk(&units16(&[(1, 4)])), []);
    }

    #[test]
    fn an_empty_transfer_ends_a_download_with_nothing_left() {
        let layout = MODELS[0].layout();
        let mut d = Download::new(layout, &CaptureInfo::default(), 0, false, 8, MHZ);
        assert_eq!(d.chunk(&[]), []);
        assert!(d.finished());
    }

    #[test]
    fn the_la5032_layout() {
        let layout = MODELS[6].layout();
        let info = CaptureInfo { n_rep_packets: 6, n_rep_packets_before_trigger: 0, write_pos: 0 };
        let mut unit = Vec::new();
        for i in 0..6u32 {
            unit.extend_from_slice(&(0x0001_0000u32 << i).to_le_bytes());
            unit.push(1);
        }
        unit.extend_from_slice(&[0xff, 0]);
        let mut d = Download::new(layout, &info, 32, false, 0, MHZ);
        let expect: Vec<u8> = (0..6u32).flat_map(|i| (0x0001_0000u32 << i).to_le_bytes()).collect();
        assert_eq!(d.chunk(&unit), [Feed::Samples(expect)]);
    }

    #[test]
    fn a_stream_takes_the_channels_in_turn() {
        // Channels 0 and 4: a word each, then 16 samples.
        let mut s = Stream::new(0b1_0001, 2, 0);
        let feed = s.chunk(&[0x01, 0x00, 0xff, 0xff, 0x03]);
        let mut expect = Vec::new();
        for bit in 0..16 {
            let v: u16 = u16::from(bit == 0) | 0x10;
            expect.extend_from_slice(&v.to_le_bytes());
        }
        assert_eq!(feed, [Feed::Samples(expect)]);
        // The odd byte is dropped, as data_length /= 2.
        assert_eq!(s.chunk(&[0x02, 0x00]), []);
        let feed = s.chunk(&[0, 0]);
        let mut expect = vec![0u8; 32];
        expect[2] = 1;
        assert_eq!(feed, [Feed::Samples(expect)]);
    }

    #[test]
    fn a_stream_stops_at_the_sample_limit() {
        let mut s = Stream::new(1, 2, 20);
        let feed = s.chunk(&[0xff, 0xff, 0, 0, 0xff, 0xff]);
        let mut expect = vec![0u8; 40];
        for i in 0..16 {
            expect[2 * i] = 1;
        }
        assert_eq!(feed, [Feed::Samples(expect)]);
        assert!(s.finished());
    }

    fn link(steps: &str) -> (Link, Replay) {
        let text = format!("# oxrok usb fixture 1\ndevice vid=77a1 pid=01a2\n{steps}");
        let r = Replay::from_text(&text).unwrap();
        (Link::new(Box::new(r.clone())), r)
    }

    #[test]
    fn identification_falls_back_to_the_backup_magic() {
        let (mut l, r) = link(
            "ctl_in 40 a2 0020 0000 4 -> 2004dffb\n\
             ctl_in 40 a2 0008 0000 8 -> 0000000008f70000\n",
        );
        assert_eq!(l.identify_device(true).unwrap().fpga_stem, "la2016a1");
        r.finish().unwrap();
        let (mut l, r) = link(
            "ctl_in 40 a2 0020 0000 4 -> err device \"endpoint stalled\"\n\
             ctl_in 40 a2 0008 0000 8 -> 02fd01fe00000000\n",
        );
        assert_eq!(l.identify_device(true).unwrap().fpga_stem, "la2016a1");
        r.finish().unwrap();
        // Quiet, the failed date read ends it.
        let (mut l, r) = link("ctl_in 40 a2 0020 0000 4 -> err device \"endpoint stalled\"\n");
        assert!(l.identify_device(false).is_err());
        r.finish().unwrap();
        let (mut l, r) = link(
            "ctl_in 40 a2 0020 0000 4 -> 2004dffb\n\
             ctl_in 40 a2 0008 0000 8 -> ffffffffffffffff\n",
        );
        assert!(l.identify_device(true).is_err());
        r.finish().unwrap();
    }

    #[test]
    fn a_loaded_fpga_is_reused() {
        let (mut l, r) = link(
            "ctl_in 40 50 0000 0000 1 -> 00\n\
             ctl_in 40 20 0000 0000 2 -> e185\n\
             ctl_in 40 20 0002 0000 1 -> 00\n\
             ctl_in 40 50 0000 0000 1 -> 00\n\
             ctl_out 40 10 0001 0000 - -> ok\n\
             ctl_in 40 20 0000 0000 2 -> e185\n\
             ctl_out 40 38 0000 0000 - -> ok\n",
        );
        l.init_hardware(&MODELS[0]).unwrap();
        r.finish().unwrap();
    }

    /// A failed read of the run state is no state: the C code warns
    /// about 0xfff5 here.
    #[test]
    fn a_failed_run_state_read_is_an_error() {
        let (mut l, r) = link("ctl_in 40 20 0000 0000 2 -> err device \"endpoint stalled\"\n");
        assert!(l.run_state().is_err());
        r.finish().unwrap();
    }

    #[test]
    fn pwm_config_writes() {
        let (mut l, r) = link(
            "ctl_out 40 20 0002 0000 00 -> ok\n\
             ctl_out 40 20 0078 0000 483f0000e9070000 -> ok\n\
             ctl_out 40 20 0002 0000 02 -> ok\n\
             ctl_out 40 20 0002 0000 02 -> ok\n",
        );
        let mut pwm = [
            Pwm { enabled: false, freq: 1000.0, duty: 50.0 },
            Pwm { enabled: true, freq: 12345.0, duty: 12.5 },
        ];
        l.set_pwm_config(&pwm, 1).unwrap();
        pwm[0].enabled = false;
        l.set_pwm_config(&pwm, 0).unwrap();
        r.finish().unwrap();
    }
}
