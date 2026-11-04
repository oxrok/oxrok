// Counterpart in sigrok:
//   libsigrok/src/hardware/kingst-la2016/api.c (the models, rates, thresholds, the scan),
//   libsigrok/src/hardware/kingst-la2016/protocol.c (the requests, registers and sample layout),
//   libsigrok/src/hardware/kingst-la2016/protocol.h,
//   Copyright (C) 2022 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   Copyright (C) 2020 Florian Schmidt <schmidt_florian@gmx.de>,
//   Copyright (C) 2013 Marcus Comstedt <marcus@mc.pp.se>,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   with commits by Kevin Grant, Helge Kruse, Mike Walters, Soeren Apel,
//   Uwe Hermann, Dave Platt
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Kingst LA series (LA1010, LA1016, LA2016, LA5016, LA5032 and
//! their A1/A2 revisions) as libsigrok's `kingst-la2016` driver sees
//! it, built on the EZ-USB shell ([`crate::shells::ezusb`]).
//!
//! Every model is the same FX2 (`77a1:01a2`) with an FPGA behind it;
//! the board says which it is through an EEPROM. Without `mcu=1` the
//! device comes up as the FX2 without firmware (`iProduct` 0), takes
//! `kingst-la-01a2.fw` through the boot loader (the driver builds the
//! name from the product id, `MCU_FWFILE_FMT` `kingst-la-%04x.fw`) and
//! renumerates with `iProduct` 2, which is how the driver tells the
//! two apart. With `mcu=1` the firmware runs from the start, and with
//! `fpga=1` the FPGA holds its bitstream too, as after an earlier
//! session.
//!
//! Device facts and where they come from. `protocol.h`: the ids, the
//! interface 0 and configuration 1, bulk OUT endpoint 2 for the
//! bitstream and bulk IN endpoint 6 of 512 bytes for the samples.
//! `protocol.c`: the vendor requests the FX2 firmware answers
//! (`0x10` FPGA enable, `0x20` FPGA register access over SPI with the
//! register in `wValue`, `0x30` bulk start, `0x38` bulk reset, `0x50`
//! FPGA init, `0x60` authentication, never used, `0xa2` EEPROM read
//! with the offset in `wValue`), the FPGA registers and their layouts,
//! the run states (`0x85e1` idle, `0x85e2` pre-trigger, `0x85ea`
//! waiting for the trigger, `0x85ee` post-trigger, `0x85ed` idle after
//! a capture), the EEPROM's manufacturing date at `0x20` (the comment's
//! example `20 04 df fb`, 2020-04) and model magic at `0x08` (the
//! comment's example `08 f7 00 00 08 f7 10 ef`: the primary copy fails
//! the four byte check, the backup carries magic 8 and magic2 `0x10`),
//! the model table with its rates, channels, memory and base clocks,
//! and the sample memory layout (`send_chunk()`, `stream_data()`).
//!
//! Not in the sources, so chosen here and unverified: the firmware's
//! descriptors beyond `iProduct` 2 (strings `Kingst` and `Logic
//! Analyzer`, no serial, `bcdDevice` 0, vendor class, 100 mA), the
//! bare chip being an FX2LP with `bcdDevice` 0xa001 loaded with the
//! ids from a `0xC0` EEPROM (the TRM's format, which this model puts
//! at offset 0), the answer of FPGA init to a blank FPGA (1: anything
//! but 0 makes the driver upload), what an FPGA register read gives
//! outside the registers the driver reads (0), where a capture starts
//! in the sample memory ([`CAPTURE_BASE`]), the sequence byte (the
//! unit's index), the padding of the last unit with packets of count
//! 0, which run state a capture shows when (pre-trigger only while
//! samples before the trigger point are still to come, as the driver's
//! comment has it for a capture ratio above 0 %, waiting until the
//! trigger fires, post-trigger after it or at once without a trigger,
//! then `0x85ed`), and what a halted capture holds (what it sampled so
//! far; before its trigger fired, the last samples, all before the
//! trigger point). The firmware and
//! bitstream files are stand-ins (`tools/emu/kingst-la2016.firmware`),
//! checked byte for byte; their sizes are not those of the vendor's
//! files.
//!
//! Emulation artifacts. Time is counted in the host's polls and
//! submissions, never the clock: a capture takes, per read of
//! `REG_RUN`, the samples of the 50 ms the driver waits between two
//! polls at the rate its divider gives, the download completes one bulk IN transfer per
//! transfer the host submits (with everything left, up to its size;
//! the firmware's packet and short packet habits are not known), and a
//! stream sends [`STREAM_BYTES`] per submitted transfer, where the real
//! device fills them as fast as it samples. A capture keeps at most
//! [`capture::MAX_SAMPLES`] samples.
//!
//! What the model taught about the C driver: when
//! `la2016_receive_data()` fails after the bulk transfers were
//! submitted (a refused bulk start of the download or of a stream, or
//! the stream's run mode), it returns FALSE with the eight transfers in
//! flight and the feed not ended; `dev_close()` then frees them and
//! libusb aborts (sigrok-cli dies with SIGABRT). `run_state()` returns
//! a failed read's `SR_ERR_IO` as a `u16`, `0xfff5`, whose idle bit is
//! set: one failed poll makes the driver download a capture that has
//! not finished. The driver passes on every sample of the transfer in
//! which the sample limit falls, so a device holding more samples than
//! asked yields more. A bad date is only a warning, and the magic falls
//! back from the primary copy to the backup and to two byte checks; a
//! device is refused when all four checks fail (magic 0, no model). At
//! the lowest rate it accepts, `clock / 65536` (3051 Hz on the
//! LA2016), `set_sample_config()` computes a divider of 65552, which
//! its `uint16_t` truncates to 16: the device samples at 12.5 MHz.
//!
//! The pedantic checks: the upload of the MCU firmware as `ezusb.c`
//! sends it; requests other than the firmware's, or with other types,
//! values, indexes and lengths than the driver uses; EEPROM reads other
//! than the date and the magic, any EEPROM write; the bitstream: its
//! size announced through FPGA init, chunks of at most 4096 bytes, the
//! stand-in's bytes, then zeros up to a multiple of 4096, bulk OUT
//! only during an upload, FPGA enable only of a loaded FPGA; register
//! writes before the FPGA is enabled, to registers the driver does not
//! write, of other lengths, or while a capture runs; the threshold
//! encoding of one of the driver's eight voltages; the capture mode 0
//! or 1 (1 on a device without memory); a trigger with no channel, a
//! level or polarity outside its channels, two edges, any trigger in
//! streaming mode; a sample config whose divider gives a rate above
//! the model's highest, whose pre-trigger samples and memory disagree with
//! one capture ratio, with pre-trigger samples without a trigger, or
//! written before the trigger config; a run without both configs
//! written since the last; a download window other than the one the
//! capture info gives; bulk start before a window or stream; PWM
//! parameters changed while that output is enabled, periods below the
//! 20 MHz limit, duty above the period, enable bits beyond the two
//! outputs; bulk IN transfers that are not a multiple of 512 bytes.
//!
//! Faults, all off by default: `stall=WHAT[@N],...` STALLs a request
//! (every time, or only its Nth occurrence since the model was built):
//! `eeprom`, `fpga-init-in`, `fpga-init-out`, `fpga-enable`,
//! `bulk-start`, `bulk-reset`, `bitstream` (a bulk OUT chunk), `loader`
//! (the boot loader's `0xa0`), `read:REG` and `write:REG` (an FPGA
//! register, `REG` in hex, such as `read:0x00`); `fw-dead=1` makes the
//! uploaded firmware never run, so the device comes back as the bare
//! FX2; `extra-bytes=N` sends N bytes (of zeros) more than the download
//! window; `more-samples=N` stores N samples more than asked;
//! `fpga-init=N` answers N to FPGA init after an upload;
//! `run-state=HEX` is the idle run state; `pwm=MASK` starts with PWM
//! outputs enabled (as a session that left them running); `eeprom-id=`
//! sixteen hex digits replace the magic bytes and `date=` eight the
//! date; `extra-packets=N` reports N packets more than the capture
//! holds; `short-bitstream=1` takes half of the first bitstream chunk;
//! `empty=N` and `stall-in=N` complete the first N bulk IN transfers
//! after a bulk start with no data, or STALL them, one per submission;
//! `slow-in=1` holds every bulk IN transfer until the host cancels it,
//! when it completes with one submission's data (a device slower than
//! the driver's 500 ms timeout, or a last transfer with no short packet
//! to end it); `in-limit=N` holds every bulk IN transfer after N with
//! data, so that an interrupt lands on a known number of them.
//!
//! The event point (`arm interrupt model`): the first read of the run
//! state during a capture.

mod capture;

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, VENDOR, Violations};
use crate::shells::ezusb::{self, Chip, Loader};
use crate::{Error, Result};
use capture::{Capture, Layout, Stream, Trigger};

/// The model name.
pub const NAME: &str = "kingst-la2016";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Kingst LA series logic analyzer (77a1:01a2, EZ-USB, renumerates, FPGA bitstream); \
                         model=la2016a1 (default), la2016, la1016, la1016a1, la1010a1, la5016a1, la5032a0; \
                         mcu=1 starts with the firmware running, fpga=1 with the bitstream loaded; \
                         faults: stall=WHAT[@N], fpga-init=N, run-state=HEX, pwm=MASK, eeprom-id=HEX, \
                         date=HEX, extra-packets=N, short-bitstream=1, empty=N, stall-in=N, fw-dead=1, \
                         extra-bytes=N, more-samples=N, slow-in=1, in-limit=N";

/// The vendor id, `protocol.h`.
pub const VID: u16 = 0x77a1;
/// The product id, `protocol.h`.
pub const PID: u16 = 0x01a2;
/// The MCU firmware, `MCU_FWFILE_FMT` with the product id.
pub const MCU_FIRMWARE: &str = "kingst-la-01a2.fw";
/// Its stand-in's size, as `tools/emu/kingst-la2016.firmware` lists it.
pub const MCU_FIRMWARE_SIZE: usize = 6221;

const CMD_FPGA_ENABLE: u8 = 0x10;
const CMD_FPGA_SPI: u8 = 0x20;
const CMD_BULK_START: u8 = 0x30;
const CMD_BULK_RESET: u8 = 0x38;
const CMD_FPGA_INIT: u8 = 0x50;
const CMD_EEPROM: u8 = 0xa2;

const REG_RUN: u8 = 0x00;
const REG_PWM_EN: u8 = 0x02;
const REG_CAPT_MODE: u8 = 0x03;
const REG_BULK: u8 = 0x08;
const REG_SAMPLING: u8 = 0x10;
const REG_TRIGGER: u8 = 0x20;
const REG_THRESHOLD: u8 = 0x68;
const REG_PWM1: u8 = 0x70;
const REG_PWM2: u8 = 0x78;
/// The register space `la2016_dump_fpga_registers()` reads at once.
const REG_SPACE: usize = 0x80;

const RUN_IDLE: u16 = 0x85e1;
const RUN_PRE: u16 = 0x85e2;
const RUN_WAIT: u16 = 0x85ea;
const RUN_POST: u16 = 0x85ee;
const RUN_DONE: u16 = 0x85ed;

/// The bitstream goes in chunks of this, zero padded to a multiple.
const EP2_PADDING: usize = 4096;
/// The sample endpoint's packet size.
const EP6_PACKET: usize = 512;
/// What a streaming transfer carries, an emulation artifact.
pub const STREAM_BYTES: usize = 512;
/// Where a capture starts in the sample memory, chosen here.
pub const CAPTURE_BASE: u32 = 0x0010_0000;
/// The PWM clock, `protocol.h`.
const PWM_CLOCK: u32 = 200_000_000;
/// The highest PWM frequency, `protocol.h`: a period of at least this.
const PWM_MIN_PERIOD: u32 = PWM_CLOCK / 20_000_000;
/// `LA2016_NUM_SAMPLES_MAX`.
const NUM_SAMPLES_MAX: u64 = 10_000_000_000;
/// The thresholds of `api.c`.
const THRESHOLDS: [f64; 8] = [0.4, 0.6, 0.9, 1.2, 1.4, 2.0, 2.5, 4.0];

/// A model of the driver's table, `models[]` in `protocol.c`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// The option value: the bitstream stem.
    pub key: &'static str,
    /// The EEPROM magic and second magic.
    pub magic: [u8; 2],
    /// Highest sample rate.
    pub rate: u64,
    /// Logic channels.
    pub channels: u32,
    /// Sample memory in Gbit, 0 for none.
    pub memory_gbit: u64,
    /// Base clock, 0 for the highest rate.
    pub baseclock: u64,
    /// The bitstream stand-in's size, `tools/emu/kingst-la2016.firmware`.
    pub bitstream_size: usize,
}

impl Profile {
    /// The bitstream file, `FPGA_FWFILE_FMT`.
    pub fn bitstream(&self) -> String {
        format!("kingst-{}-fpga.bitstream", self.key)
    }

    fn clock(&self) -> u64 {
        if self.baseclock == 0 { self.rate } else { self.baseclock }
    }

    fn memory_bytes(&self) -> u64 {
        self.memory_gbit * 1024 * 1024 * 1024 / 8
    }

    fn layout(&self) -> Layout {
        Layout { wide: self.channels == 32 }
    }

    fn mask(&self) -> u32 {
        if self.channels == 32 { u32::MAX } else { (1 << self.channels) - 1 }
    }
}

/// The emulated models: the LA2016 and LA1016 and their A1 revisions,
/// one without memory, one at 500 MHz and the 32 channel one.
pub const PROFILES: [Profile; 7] = [
    Profile {
        key: "la2016a1",
        magic: [0x08, 0x10],
        rate: 200_000_000,
        channels: 16,
        memory_gbit: 1,
        baseclock: 0,
        bitstream_size: 149_461,
    },
    Profile {
        key: "la2016",
        magic: [0x02, 0x00],
        rate: 200_000_000,
        channels: 16,
        memory_gbit: 1,
        baseclock: 0,
        bitstream_size: 149_333,
    },
    Profile {
        key: "la1016",
        magic: [0x03, 0x00],
        rate: 100_000_000,
        channels: 16,
        memory_gbit: 1,
        baseclock: 0,
        bitstream_size: 149_205,
    },
    Profile {
        key: "la1016a1",
        magic: [0x09, 0x00],
        rate: 100_000_000,
        channels: 16,
        memory_gbit: 1,
        baseclock: 0,
        bitstream_size: 149_077,
    },
    Profile {
        key: "la1010a1",
        magic: [0x07, 0x00],
        rate: 100_000_000,
        channels: 16,
        memory_gbit: 0,
        baseclock: 800_000_000,
        bitstream_size: 148_949,
    },
    Profile {
        key: "la5016a1",
        magic: [0x05, 0x00],
        rate: 500_000_000,
        channels: 16,
        memory_gbit: 2,
        baseclock: 800_000_000,
        bitstream_size: 340_181,
    },
    Profile {
        key: "la5032a0",
        magic: [0x06, 0x00],
        rate: 500_000_000,
        channels: 32,
        memory_gbit: 4,
        baseclock: 800_000_000,
        bitstream_size: 340_053,
    },
];

/// The threshold register's two duty values for a voltage, as
/// `set_threshold_voltage()` computes them (in `float`).
fn threshold_duties(voltage: f64) -> (u16, u16) {
    let v = (voltage as f32).clamp(-4.0, 4.0);
    let (r79, r56) = if f64::from(v) >= 2.9 {
        (0, 302.0f32 * v - 363.0)
    } else if f64::from(v) > -0.4 {
        (0x00f2, 302.0f32 * v + 121.0)
    } else {
        (0x02d7, 302.0f32 * v + 1090.0)
    };
    (((r56 as u16).clamp(10, 1100)), r79)
}

/// A request the faults can name.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Req {
    Eeprom,
    FpgaInitIn,
    FpgaInitOut,
    FpgaEnable,
    BulkStart,
    BulkReset,
    Bitstream,
    Loader,
    Read(u8),
    Write(u8),
}

impl Req {
    fn parse(s: &str) -> Option<Req> {
        let reg = |r: &str| u8::from_str_radix(r.trim_start_matches("0x"), 16).ok();
        Some(match s {
            "eeprom" => Req::Eeprom,
            "fpga-init-in" => Req::FpgaInitIn,
            "fpga-init-out" => Req::FpgaInitOut,
            "fpga-enable" => Req::FpgaEnable,
            "bulk-start" => Req::BulkStart,
            "bulk-reset" => Req::BulkReset,
            "bitstream" => Req::Bitstream,
            "loader" => Req::Loader,
            _ => {
                if let Some(r) = s.strip_prefix("read:") {
                    Req::Read(reg(r)?)
                } else {
                    Req::Write(reg(s.strip_prefix("write:")?)?)
                }
            }
        })
    }
}

/// Ways the device can misbehave, as a real one can; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    stall: Vec<(Req, Option<u32>)>,
    fpga_init: u8,
    run_state: Option<u16>,
    pwm: u8,
    eeprom_id: Option<[u8; 8]>,
    date: Option<[u8; 4]>,
    extra_packets: u32,
    short_bitstream: bool,
    empty: u32,
    stall_in: u32,
    slow_in: bool,
    in_limit: Option<u32>,
    fw_dead: bool,
    extra_bytes: u32,
    more_samples: u64,
}

fn hex_bytes<const N: usize>(k: &str, v: &str) -> Result<[u8; N]> {
    let bad = || Error(format!("{k}={v}: {} hex digits", 2 * N));
    if v.len() != 2 * N {
        return Err(bad());
    }
    let mut out = [0u8; N];
    for (i, b) in out.iter_mut().enumerate() {
        *b = u8::from_str_radix(v.get(2 * i..2 * i + 2).ok_or_else(bad)?, 16).map_err(|_| bad())?;
    }
    Ok(out)
}

impl Faults {
    /// Takes the option `k=v` when it is a fault; false when `k` is no
    /// fault option.
    fn parse(&mut self, k: &str, v: &str) -> Result<bool> {
        let count = |v: &str| v.parse::<u32>().map_err(|_| Error(format!("{k}={v}: a count")));
        match k {
            "stall" => {
                self.stall.clear();
                for item in v.split(',').filter(|n| !n.is_empty()) {
                    let (what, nth) = match item.split_once('@') {
                        Some((w, n)) => (w, Some(count(n)?)),
                        None => (item, None),
                    };
                    let req = Req::parse(what).ok_or_else(|| {
                        Error(format!(
                            "stall={item}: eeprom, fpga-init-in, fpga-init-out, fpga-enable, \
                             bulk-start, bulk-reset, bitstream, loader, read:REG or write:REG, then @N"
                        ))
                    })?;
                    self.stall.push((req, nth));
                }
            }
            "fpga-init" => {
                self.fpga_init =
                    v.parse().map_err(|_| Error(format!("fpga-init={v}: 0 to 255")))?;
            }
            "run-state" => {
                let n = u16::from_str_radix(v.trim_start_matches("0x"), 16)
                    .map_err(|_| Error(format!("run-state={v}: four hex digits")))?;
                self.run_state = Some(n);
            }
            "pwm" => {
                self.pwm = v.parse().ok().filter(|m| *m < 4).ok_or_else(|| {
                    Error(format!("pwm={v}: a mask of the two outputs, 0 to 3"))
                })?;
            }
            "eeprom-id" => self.eeprom_id = Some(hex_bytes(k, v)?),
            "date" => self.date = Some(hex_bytes(k, v)?),
            "extra-packets" => self.extra_packets = count(v)?,
            "short-bitstream" => {
                self.short_bitstream = match v {
                    "0" => false,
                    "1" => true,
                    _ => return Err(Error(format!("short-bitstream={v}: 0 or 1"))),
                }
            }
            "fw-dead" => {
                self.fw_dead = match v {
                    "0" => false,
                    "1" => true,
                    _ => return Err(Error(format!("fw-dead={v}: 0 or 1"))),
                }
            }
            "extra-bytes" => self.extra_bytes = count(v)?,
            "more-samples" => self.more_samples = u64::from(count(v)?),
            "empty" => self.empty = count(v)?,
            "stall-in" => self.stall_in = count(v)?,
            "slow-in" => {
                self.slow_in = match v {
                    "0" => false,
                    "1" => true,
                    _ => return Err(Error(format!("slow-in={v}: 0 or 1"))),
                }
            }
            "in-limit" => self.in_limit = Some(count(v)?),
            _ => return Ok(false),
        }
        Ok(true)
    }
}

fn flag(k: &str, v: &str) -> Result<bool> {
    match v {
        "0" => Ok(false),
        "1" => Ok(true),
        _ => Err(Error(format!("{k}={v}: 0 or 1"))),
    }
}

/// A device from the `-o` options: `model=KEY` (see [`PROFILES`]),
/// `mcu=0|1`, `fpga=0|1` (which needs `mcu=1`), and the faults.
///
/// # Errors
///
/// On an unknown option or value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut profile = PROFILES[0];
    let (mut mcu, mut fpga) = (false, false);
    let mut faults = Faults::default();
    for (k, v) in options {
        if faults.parse(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => {
                profile = *PROFILES.iter().find(|p| p.key == v).ok_or_else(|| {
                    let keys: Vec<&str> = PROFILES.iter().map(|p| p.key).collect();
                    Error(format!("model={v}: one of {}", keys.join(", ")))
                })?;
            }
            "mcu" => mcu = flag(k, v)?,
            "fpga" => fpga = flag(k, v)?,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    if fpga && !mcu {
        return Err(Error("fpga=1 needs mcu=1: the bitstream is loaded through the firmware".into()));
    }
    Ok(Box::new(Kingst::new(profile, mcu, fpga, faults)))
}

/// The descriptors of the running MCU firmware.
pub fn firmware_descriptors() -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 2, [0xff, 0xff, 0xff], 0));
    body.extend_from_slice(&descriptor::endpoint(0x02, crate::model::BULK, 512, 0));
    body.extend_from_slice(&descriptor::endpoint(0x86, crate::model::BULK, 512, 0));
    let config = descriptor::configuration(1, 0, 0x80, 100, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0xff, 0xff, 0xff],
        max_packet0: 64,
        vendor: VID,
        product: PID,
        device: 0x0000,
        strings: [1, 2, 0],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::High,
        device: dev.bytes(),
        configs: vec![config],
        strings: vec!["Kingst".into(), "Logic Analyzer".into()],
    }
}

/// The FPGA.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Fpga {
    Blank,
    /// An upload of `size` bytes announced; what came so far, and the
    /// size of each chunk.
    Loading { size: u32, got: Vec<u8>, chunks: Vec<usize> },
    Loaded,
}

/// What the host configured for the next run.
#[derive(Clone, Debug, Default)]
struct Config {
    mode: Option<u8>,
    trigger: Option<Trigger>,
    /// Samples, pre-trigger samples, clock divider.
    sampling: Option<(u64, u64, u16)>,
}

/// What the capture engine does.
#[derive(Clone, Debug)]
enum Run {
    Idle,
    Capturing(Box<Running>),
    Streaming(Stream),
}

/// A capture into the sample memory, which samples on as the host
/// polls: each read of `REG_RUN` stands for the 50 ms the driver waits
/// between two (`usb_source_add()`'s timeout), at the rate the divider
/// gives.
#[derive(Clone, Debug)]
struct Running {
    /// The trigger, when one is involved.
    trigger: Option<Trigger>,
    /// The sample the trigger fires at; `None` when it never does.
    at: Option<u64>,
    pre: u64,
    limit: u64,
    per_poll: u64,
    sampled: u64,
    layout: Layout,
}

impl Running {
    /// The sample after the last one the capture keeps.
    fn end(&self) -> Option<u64> {
        match self.trigger {
            None => Some(self.limit),
            Some(_) => self.at.map(|a| a - self.pre + self.limit),
        }
    }

    fn done(&self) -> bool {
        self.end().is_some_and(|e| self.sampled >= e)
    }

    /// The run state: pre-trigger while the samples before the trigger
    /// point are not all taken, then waiting for the trigger, then
    /// post-trigger; a capture without trigger is post-trigger at once.
    fn state(&self) -> u16 {
        if self.done() {
            RUN_DONE
        } else if self.trigger.is_none() {
            RUN_POST
        } else if self.sampled < self.pre {
            RUN_PRE
        } else if self.at.is_none_or(|a| self.sampled <= a) {
            RUN_WAIT
        } else {
            RUN_POST
        }
    }

    /// What the memory holds: the whole window once done; after a halt
    /// what was sampled, the last `limit` samples before the trigger
    /// when it did not fire (all of them marked before the trigger).
    fn capture(&self) -> Capture {
        let n = self.sampled;
        match (self.trigger, self.at) {
            (None, _) => capture::capture(0, self.limit.min(n), None, self.layout),
            (Some(_), Some(a)) if n > a => {
                let start = a - self.pre;
                capture::capture(start, (n - start).min(self.limit), Some(self.pre), self.layout)
            }
            (Some(_), _) => {
                let len = n.min(self.limit);
                capture::capture(n - len, len, Some(len), self.layout)
            }
        }
    }
}

/// The bulk IN side.
#[derive(Clone, Debug, Default)]
struct Bulk {
    /// The download's bytes, after a `REG_BULK` write.
    data: Option<Vec<u8>>,
    sent: usize,
    started: bool,
    ready: bool,
    empty: u32,
    stall_in: u32,
    /// Transfers completed with data since the stream or window began.
    done: u32,
}

/// The emulated device.
#[allow(clippy::struct_excessive_bools)]
pub struct Kingst {
    profile: Profile,
    faults: Faults,
    boot: Descriptors,
    firmware: Descriptors,
    loader: Option<Box<Loader>>,
    renumerate: bool,
    eeprom: Vec<u8>,
    fpga: Fpga,
    enabled: bool,
    pwm_en: u8,
    config: Config,
    run: Run,
    captured: Option<Box<Capture>>,
    bulk: Bulk,
    counts: Vec<(Req, u32)>,
    /// Whether a capture's run state was read, and the event point of the
    /// first such read, not yet taken.
    polled: bool,
    point: bool,
}

impl Kingst {
    /// A device; `mcu` starts it with the firmware running, `fpga` with
    /// the bitstream loaded and enabled.
    pub fn new(profile: Profile, mcu: bool, fpga: bool, faults: Faults) -> Kingst {
        let mut eeprom = vec![0xff; 256];
        eeprom[..8].copy_from_slice(&[
            0xc0,
            VID as u8,
            (VID >> 8) as u8,
            PID as u8,
            (PID >> 8) as u8,
            ezusb::FX2LP_DID as u8,
            (ezusb::FX2LP_DID >> 8) as u8,
            0x00,
        ]);
        let [m, m2] = profile.magic;
        let id = faults.eeprom_id.unwrap_or([m, !m, 0, 0, m, !m, m2, !m2]);
        eeprom[0x08..0x10].copy_from_slice(&id);
        eeprom[0x20..0x24].copy_from_slice(&faults.date.unwrap_or([0x20, 0x04, 0xdf, 0xfb]));
        Kingst {
            boot: ezusb::bare_descriptors(VID, PID, ezusb::FX2LP_DID),
            firmware: firmware_descriptors(),
            loader: if mcu { None } else { Some(Box::new(Loader::new(Chip::Fx2lp))) },
            renumerate: false,
            eeprom,
            fpga: if fpga { Fpga::Loaded } else { Fpga::Blank },
            enabled: fpga,
            pwm_en: if fpga { faults.pwm } else { 0 },
            config: Config::default(),
            run: Run::Idle,
            captured: None,
            bulk: Bulk::default(),
            counts: Vec::new(),
            polled: false,
            point: false,
            profile,
            faults,
        }
    }

    /// The MCU firmware image both tools must upload.
    pub fn mcu_image() -> Vec<u8> {
        ezusb::standin(MCU_FIRMWARE, MCU_FIRMWARE_SIZE)
    }

    /// The bitstream both tools must upload.
    pub fn bitstream(&self) -> Vec<u8> {
        ezusb::standin(&self.profile.bitstream(), self.profile.bitstream_size)
    }

    /// Counts an occurrence of `req`; whether a fault STALLs it.
    fn stalled(&mut self, req: Req) -> bool {
        let n = if let Some((_, n)) = self.counts.iter_mut().find(|(r, _)| *r == req) {
            *n += 1;
            *n
        } else {
            self.counts.push((req, 1));
            1
        };
        self.faults.stall.iter().any(|(r, nth)| *r == req && nth.is_none_or(|k| k == n))
    }

    fn idle_state(&self) -> u16 {
        self.faults
            .run_state
            .unwrap_or(if self.captured.is_some() { RUN_DONE } else { RUN_IDLE })
    }

    fn boot_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() == VENDOR && setup.request == ezusb::REQ_FIRMWARE_LOAD && self.stalled(Req::Loader) {
            return Reply::Stall;
        }
        let loader = self.loader.as_mut().expect("boot stage");
        let Some((reply, started)) = loader.control(setup, data, v) else {
            v.report(format!(
                "request {:#04x} {:#04x} value {:#06x} to the FX2 without firmware; only its \
                 boot loader (0xa0) answers",
                setup.request_type, setup.request, setup.value
            ));
            return Reply::Stall;
        };
        if started {
            loader.check_ezusb_c(&Self::mcu_image(), v);
            // A firmware that never runs leaves the boot loader, which
            // comes back as the bare FX2.
            self.loader = if self.faults.fw_dead { Some(Box::new(Loader::new(Chip::Fx2lp))) } else { None };
            self.renumerate = true;
        }
        reply
    }

    fn firmware_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() != VENDOR || setup.recipient() != crate::model::RECIPIENT_DEVICE {
            v.report(format!(
                "request type {:#04x} request {:#04x}: the firmware answers vendor requests to \
                 the device",
                setup.request_type, setup.request
            ));
            return Reply::Stall;
        }
        let input = setup.is_in();
        let req = match (setup.request, input) {
            (CMD_EEPROM, true) => Req::Eeprom,
            (CMD_FPGA_INIT, true) => Req::FpgaInitIn,
            (CMD_FPGA_INIT, false) => Req::FpgaInitOut,
            (CMD_FPGA_ENABLE, false) => Req::FpgaEnable,
            (CMD_BULK_START, false) => Req::BulkStart,
            (CMD_BULK_RESET, false) => Req::BulkReset,
            (CMD_FPGA_SPI, true) => Req::Read(setup.value as u8),
            (CMD_FPGA_SPI, false) => Req::Write(setup.value as u8),
            (ezusb::REQ_FIRMWARE_LOAD, _) => {
                v.report("firmware load to an FX2 whose firmware runs");
                return Reply::ack();
            }
            (r, _) => {
                v.report(format!(
                    "request {:#04x} {r:#04x} value {:#06x}: not one the driver sends{}",
                    setup.request_type,
                    setup.value,
                    if r == 0x60 { " (0x60 talks to the authentication chip; unused)" } else { "" }
                ));
                return Reply::Stall;
            }
        };
        let index = if req == Req::Write(REG_TRIGGER) { 16 } else { 0 };
        if setup.index != index {
            v.report(format!(
                "request {:#04x} value {:#06x} with wIndex {}; the driver sends {index}",
                setup.request, setup.value, setup.index
            ));
        }
        if matches!(req, Req::Read(_) | Req::Write(_)) && setup.value >= REG_SPACE as u16 {
            v.report(format!("FPGA register {:#06x}: the registers end at 0x7f", setup.value));
        }
        if self.stalled(req) {
            return Reply::Stall;
        }
        match req {
            Req::Eeprom => self.eeprom_read(setup, v),
            Req::FpgaInitIn => {
                Self::expect_len(setup, 1, v);
                Reply::Data(vec![self.fpga_init_answer(v)])
            }
            Req::FpgaInitOut => {
                self.fpga_init(setup, data, v);
                Reply::ack()
            }
            Req::FpgaEnable => {
                Self::expect_len(setup, 0, v);
                match setup.value {
                    1 if self.fpga != Fpga::Loaded => {
                        v.report("FPGA enable before a bitstream was loaded");
                    }
                    1 => self.enabled = true,
                    0 => self.enabled = false,
                    x => v.report(format!("FPGA enable with value {x:#06x}; 0 or 1")),
                }
                Reply::ack()
            }
            Req::BulkStart => {
                Self::expect_len(setup, 0, v);
                Self::expect_value(setup, v);
                self.bulk_start(v);
                Reply::ack()
            }
            Req::BulkReset => {
                Self::expect_len(setup, 0, v);
                Self::expect_value(setup, v);
                // Flushes the endpoint 6 FIFO: no download pending.
                self.bulk = Bulk::default();
                Reply::ack()
            }
            Req::Read(reg) => Reply::Data(self.read_registers(reg, setup.length, v)),
            Req::Write(reg) => {
                self.write_register(reg, data, v);
                Reply::ack()
            }
            Req::Bitstream | Req::Loader => unreachable!("not a request of the firmware"),
        }
    }

    fn expect_len(setup: Setup, len: u16, v: &mut Violations) {
        if setup.length != len {
            v.report(format!(
                "request {:#04x} with wLength {}; the driver uses {len}",
                setup.request, setup.length
            ));
        }
    }

    fn expect_value(setup: Setup, v: &mut Violations) {
        if setup.value != 0 {
            v.report(format!(
                "request {:#04x} with value {:#06x}; the driver sends 0",
                setup.request, setup.value
            ));
        }
    }

    fn eeprom_read(&mut self, setup: Setup, v: &mut Violations) -> Reply {
        let (offset, len) = (usize::from(setup.value), usize::from(setup.length));
        if !matches!((offset, len), (0x20, 4) | (0x08, 8)) {
            v.report(format!(
                "EEPROM read of {len} bytes at {offset:#06x}; the driver reads the date (4 at \
                 0x20) and the magic (8 at 0x08)"
            ));
        }
        Reply::Data((offset..offset + len).map(|a| self.eeprom.get(a).copied().unwrap_or(0xff)).collect())
    }

    fn fpga_init(&mut self, setup: Setup, data: &[u8], v: &mut Violations) {
        Self::expect_value(setup, v);
        if data.len() != 4 {
            v.report(format!("FPGA init of {} bytes; it takes the bitstream size, 4", data.len()));
            return;
        }
        let size = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        if size as usize != self.profile.bitstream_size {
            v.report(format!(
                "FPGA init for {size} bytes; {} has {}",
                self.profile.bitstream(),
                self.profile.bitstream_size
            ));
        }
        if !matches!(self.run, Run::Idle) {
            v.report("bitstream upload while a capture runs");
        }
        // A new configuration clears the FPGA and its registers.
        self.fpga = Fpga::Loading { size, got: Vec::new(), chunks: Vec::new() };
        self.enabled = false;
        self.pwm_en = 0;
        self.run = Run::Idle;
        self.captured = None;
        self.config = Config::default();
    }

    fn fpga_init_answer(&mut self, v: &mut Violations) -> u8 {
        match std::mem::replace(&mut self.fpga, Fpga::Blank) {
            Fpga::Blank => 1,
            Fpga::Loaded => {
                self.fpga = Fpga::Loaded;
                0
            }
            Fpga::Loading { size, got, chunks } => {
                let image = self.bitstream();
                let padded = image.len().div_ceil(EP2_PADDING) * EP2_PADDING;
                let mut ok = true;
                // upload_fpga_bitstream(): the file in reads of 4096,
                // then zeros up to the padding, in chunks of 4096.
                let mut want: Vec<usize> = image.chunks(EP2_PADDING).map(<[u8]>::len).collect();
                let mut pad = padded - image.len();
                while pad > 0 {
                    want.push(pad.min(EP2_PADDING));
                    pad -= pad.min(EP2_PADDING);
                }
                if chunks != want {
                    v.report(format!(
                        "bitstream in {} chunks where the driver sends {} (4096 each, the \
                         file's remainder, then the padding)",
                        chunks.len(),
                        want.len()
                    ));
                    ok = false;
                }
                if got.len() != padded {
                    v.report(format!(
                        "bitstream upload of {} bytes; {size} announced, so {padded} with the \
                         padding",
                        got.len()
                    ));
                    ok = false;
                }
                let content = got.len().min(image.len());
                if got[..content] != image[..content] {
                    v.report(format!("the bitstream differs from {}", self.profile.bitstream()));
                    ok = false;
                }
                if got.iter().skip(image.len()).any(|b| *b != 0) {
                    v.report("the bitstream's padding is not zeros");
                    ok = false;
                }
                if ok {
                    self.fpga = Fpga::Loaded;
                    self.faults.fpga_init
                } else {
                    1
                }
            }
        }
    }

    /// The register file as reads see it.
    fn registers(&mut self, reg: u8, len: usize) -> Vec<u8> {
        let mut space = [0u8; REG_SPACE];
        let reads_run = usize::from(reg) < 2 && len > 0;
        let state = if reads_run { self.poll() } else { self.peek_state() };
        space[0..2].copy_from_slice(&state.to_le_bytes());
        space[usize::from(REG_PWM_EN)] = self.pwm_en;
        if matches!(self.run, Run::Idle)
            && let Some(c) = &self.captured
        {
            let packets = c.packets + self.faults.extra_packets;
            let write_pos = CAPTURE_BASE + c.memory.len() as u32;
            let at = usize::from(REG_SAMPLING);
            space[at..at + 4].copy_from_slice(&packets.to_le_bytes());
            space[at + 4..at + 8].copy_from_slice(&c.before_trigger.to_le_bytes());
            space[at + 8..at + 12].copy_from_slice(&write_pos.to_le_bytes());
        }
        let start = usize::from(reg).min(REG_SPACE);
        let end = (start + len).min(REG_SPACE);
        let mut out = space[start..end].to_vec();
        out.resize(len, 0);
        out
    }

    fn read_registers(&mut self, reg: u8, length: u16, v: &mut Violations) -> Vec<u8> {
        let len = usize::from(length);
        let known = matches!(
            (reg, len),
            (REG_RUN, 2 | REG_SPACE) | (REG_PWM_EN, 1) | (REG_SAMPLING, 16)
        );
        if !known {
            v.report(format!(
                "FPGA register read of {len} bytes at {reg:#04x}; the driver reads 2 at 0x00, \
                 1 at 0x02, 16 at 0x10, or all 128"
            ));
        }
        // The dump of the spew level reads all registers before the
        // bitstream check, whatever the FPGA holds: a habit of the
        // driver, not flagged.
        if !self.enabled && len != REG_SPACE {
            v.report(format!("FPGA register {reg:#04x} read before the FPGA was enabled"));
        }
        self.registers(reg, len)
    }

    /// The run state without a poll's effect.
    fn peek_state(&self) -> u16 {
        match &self.run {
            Run::Idle => self.idle_state(),
            Run::Capturing(r) => r.state(),
            Run::Streaming(_) => RUN_PRE,
        }
    }

    /// A read of `REG_RUN`: a capture samples on by a poll's worth.
    fn poll(&mut self) -> u16 {
        if let Run::Capturing(r) = &mut self.run {
            self.point |= !std::mem::replace(&mut self.polled, true);
            r.sampled += r.per_poll;
            if r.done() {
                let capture = Box::new(r.capture());
                self.run = Run::Idle;
                self.captured = Some(capture);
            }
        }
        self.peek_state()
    }

    fn write_register(&mut self, reg: u8, data: &[u8], v: &mut Violations) {
        if !self.enabled {
            v.report(format!("FPGA register {reg:#04x} written before the FPGA was enabled"));
            return;
        }
        let want = match reg {
            REG_RUN | REG_PWM_EN | REG_CAPT_MODE => 1,
            REG_BULK | REG_PWM1 | REG_PWM2 => 8,
            REG_SAMPLING | REG_TRIGGER => 16,
            REG_THRESHOLD => 4,
            _ => {
                v.report(format!("FPGA register {reg:#04x} written; the driver never does"));
                return;
            }
        };
        if data.len() != want {
            v.report(format!(
                "FPGA register {reg:#04x} written with {} bytes; it takes {want}",
                data.len()
            ));
            return;
        }
        let busy = !matches!(self.run, Run::Idle);
        if busy && matches!(reg, REG_CAPT_MODE | REG_SAMPLING | REG_TRIGGER | REG_THRESHOLD) {
            v.report(format!("FPGA register {reg:#04x} written while a capture runs"));
        }
        match reg {
            REG_RUN => self.run_mode(data[0], v),
            REG_PWM_EN => {
                if data[0] & !3 != 0 {
                    v.report(format!("PWM enable {:#04x}: two outputs, bits 0 and 1", data[0]));
                }
                self.pwm_en = data[0] & 3;
            }
            REG_CAPT_MODE => {
                match data[0] {
                    0 if self.profile.memory_gbit == 0 => {
                        v.report("capture to memory on a device without memory");
                    }
                    0 | 1 => {}
                    m => v.report(format!("capture mode {m:#04x}: 0 to memory or 1 streaming")),
                }
                self.config.mode = Some(data[0]);
            }
            REG_BULK => self.bulk_window(data, v),
            REG_SAMPLING => self.sampling(data, v),
            REG_TRIGGER => self.trigger(data, v),
            REG_THRESHOLD => {
                let r56 = u16::from_le_bytes([data[0], data[1]]);
                let r79 = u16::from_le_bytes([data[2], data[3]]);
                if !THRESHOLDS.iter().any(|t| threshold_duties(*t) == (r56, r79)) {
                    v.report(format!(
                        "threshold duties R56 {r56:#06x} R79 {r79:#06x}: none of the driver's \
                         voltages"
                    ));
                }
            }
            _ => {
                let bit = if reg == REG_PWM1 { 1 } else { 2 };
                if self.pwm_en & bit != 0 {
                    v.report(format!(
                        "PWM{bit} parameters changed while it is enabled; the driver disables it first"
                    ));
                }
                let w = |i: usize| u32::from_le_bytes([data[i], data[i + 1], data[i + 2], data[i + 3]]);
                let (period, duty) = (w(0), w(4));
                if period < PWM_MIN_PERIOD {
                    v.report(format!("PWM period {period}: above the 20 MHz limit"));
                }
                if duty > period {
                    v.report(format!("PWM duty {duty} longer than its period {period}"));
                }
            }
        }
    }

    fn trigger(&mut self, data: &[u8], v: &mut Violations) {
        let t = Trigger::parse(data).expect("16 bytes");
        let mask = self.profile.mask();
        if t.channels == 0 {
            v.report("trigger config with no enabled channel");
        }
        if (t.channels | t.enabled) & !mask != 0 {
            v.report(format!(
                "trigger config with channels {:#x} {:#x} beyond the {} of the device",
                t.channels, t.enabled, self.profile.channels
            ));
        }
        if (t.level | t.high_or_falling) & !t.enabled != 0 {
            v.report(format!(
                "trigger levels {:#x} polarity {:#x} outside the trigger channels {:#x}",
                t.level, t.high_or_falling, t.enabled
            ));
        }
        if t.edge().count_ones() > 1 {
            v.report(format!("trigger on {} edges; the device has one", t.edge().count_ones()));
        }
        let streaming = self.config.mode == Some(1) || self.profile.memory_gbit == 0;
        if streaming && t.involved() {
            v.report("hardware trigger in streaming mode or without memory");
        }
        self.config.trigger = Some(t);
    }

    fn sampling(&mut self, b: &[u8], v: &mut Violations) {
        let u40 = |i: usize| {
            let mut w = [0u8; 8];
            w[..5].copy_from_slice(&b[i..i + 5]);
            u64::from_le_bytes(w)
        };
        let (limit, pre) = (u40(0), u40(5));
        let mem = u32::from_le_bytes([b[10], b[11], b[12], 0]);
        let divider = u16::from_le_bytes([b[13], b[14]]);
        if b[15] != 0 {
            v.report(format!("sample config's last byte {:#04x}; the driver writes 0", b[15]));
        }
        if limit == 0 || limit > NUM_SAMPLES_MAX {
            v.report(format!("sample limit {limit}: 1 to 10 G"));
        }
        // set_sample_config() takes any rate up to the model's highest
        // and down to the clock / 65536, and sends clock / rate.
        let clock = self.profile.clock();
        let least = (clock / self.profile.rate).max(1);
        if u64::from(divider) < least {
            v.report(format!(
                "clock divider {divider}: {clock} Hz / {divider} is above the {} Hz of the {}",
                self.profile.rate, self.profile.key
            ));
        }
        let Some(trigger) = self.config.trigger else {
            v.report("sample config before the trigger config");
            return;
        };
        let involved = trigger.involved() && self.profile.memory_gbit != 0;
        if involved {
            let bytes = self.profile.memory_bytes();
            let consistent = (0..=100u64).any(|r| {
                limit * r / 100 == pre && u64::from(mem) == (bytes * r / 100).max(0x100) >> 8
            });
            if !consistent {
                v.report(format!(
                    "pre-trigger {pre} of {limit} samples and memory {:#x}: no one capture ratio",
                    u64::from(mem) << 8
                ));
            }
        } else if pre != 0 || mem != 1 {
            v.report(format!(
                "pre-trigger {pre} samples, memory {:#x}, without a trigger; the driver sends 0 \
                 and 0x100",
                u64::from(mem) << 8
            ));
        }
        self.config.sampling = Some((limit, pre, divider));
    }

    fn run_mode(&mut self, mode: u8, v: &mut Violations) {
        match mode {
            3 => {
                if !matches!(self.run, Run::Idle) {
                    v.report("run while a capture runs");
                    return;
                }
                let config = std::mem::take(&mut self.config);
                let (Some(trigger), Some((limit, pre, divider)), Some(mode)) =
                    (config.trigger, config.sampling, config.mode)
                else {
                    v.report("run without capture mode, trigger and sample config written since the last");
                    return;
                };
                self.bulk = Bulk::default();
                self.captured = None;
                let layout = self.profile.layout();
                if mode == 1 || self.profile.memory_gbit == 0 {
                    self.run = Run::Streaming(Stream::new(trigger.channels, layout.wide));
                    return;
                }
                let rate = (self.profile.clock() / u64::from(divider.max(1))).min(self.profile.rate);
                let mut running = Running {
                    trigger: None,
                    at: None,
                    pre: 0,
                    limit: limit + self.faults.more_samples,
                    per_poll: (rate / 20).max(1),
                    sampled: 0,
                    layout,
                };
                if trigger.involved() {
                    let pre = pre.min(capture::MAX_SAMPLES);
                    let from = if trigger.edge() != 0 { pre.max(1) } else { pre };
                    running.trigger = Some(trigger);
                    running.at = capture::find_trigger(&trigger, from, layout.wide);
                    running.pre = pre;
                }
                self.run = Run::Capturing(Box::new(running));
            }
            0 => {
                // A halt: a capture keeps what it sampled so far.
                if let Run::Capturing(r) = std::mem::replace(&mut self.run, Run::Idle) {
                    self.captured = Some(Box::new(r.capture()));
                }
                self.bulk.started = false;
            }
            m => v.report(format!("run mode {m:#04x}: 0 halts, 3 runs")),
        }
    }

    fn bulk_window(&mut self, data: &[u8], v: &mut Violations) {
        let read_pos = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let len = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        if let (Run::Idle, Some(c)) = (&self.run, &self.captured) {
            let layout = self.profile.layout();
            let packets = c.packets + self.faults.extra_packets;
            let n = (packets as usize / layout.packets() * layout.xfer()) as u32;
            let write_pos = CAPTURE_BASE + c.memory.len() as u32;
            if (read_pos, len) != (write_pos.wrapping_sub(n), n) {
                v.report(format!(
                    "download of {len} bytes from {read_pos:#010x}; the capture info gives {n} \
                     from {:#010x}",
                    write_pos.wrapping_sub(n)
                ));
            }
        }
        // The sample memory: the capture from CAPTURE_BASE, zeros
        // elsewhere (a running capture has stored nothing yet).
        let memory = match (&self.run, &self.captured) {
            (Run::Idle, Some(c)) => c.memory.as_slice(),
            _ => &[],
        };
        let bytes = (0..len + self.faults.extra_bytes)
            .map(|i| {
                let a = read_pos.wrapping_add(i).wrapping_sub(CAPTURE_BASE) as usize;
                memory.get(a).copied().unwrap_or(0)
            })
            .collect();
        self.bulk = Bulk { data: Some(bytes), ..Bulk::default() };
    }

    fn bulk_start(&mut self, v: &mut Violations) {
        let streaming = matches!(self.run, Run::Streaming(_));
        if self.bulk.data.is_none() && !streaming {
            v.report("bulk start with neither a download window nor a stream");
            return;
        }
        if streaming && self.bulk.data.is_some() {
            v.report("bulk start of a stream with a download window set");
        }
        self.bulk.started = true;
        self.bulk.ready = true;
        self.bulk.empty = self.faults.empty;
        self.bulk.stall_in = self.faults.stall_in;
    }

    fn bitstream_chunk(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        let Fpga::Loading { got, chunks, .. } = &mut self.fpga else {
            v.report("bulk OUT to endpoint 2 with no bitstream upload announced");
            return Reply::Stall;
        };
        if data.len() > EP2_PADDING {
            v.report(format!("bitstream chunk of {} bytes; the driver sends 4096 at most", data.len()));
        }
        let take = if self.faults.short_bitstream && got.is_empty() { data.len() / 2 } else { data.len() };
        got.extend_from_slice(&data[..take]);
        chunks.push(take);
        if self.stalled(Req::Bitstream) {
            return Reply::Stall;
        }
        if take < data.len() { Reply::Data(vec![0; take]) } else { Reply::ack() }
    }
}

impl DeviceModel for Kingst {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        if self.loader.is_some() { &self.boot } else { &self.firmware }
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if self.loader.is_some() {
            self.boot_control(setup, data, v)
        } else {
            self.firmware_control(setup, data, v)
        }
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != 2 {
            v.report(format!("OUT transfer to endpoint {ep}; the bitstream goes to 2"));
            return Reply::Stall;
        }
        self.bitstream_chunk(data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep != 6 {
            v.report(format!("IN transfer from endpoint {ep}; the samples come from 6"));
            return Reply::Stall;
        }
        if !self.bulk.started || !self.bulk.ready {
            return Reply::Nak;
        }
        if !max.is_multiple_of(EP6_PACKET) {
            v.report(format!(
                "bulk IN transfer of {max} bytes; the driver asks for multiples of {EP6_PACKET}"
            ));
        }
        if self.faults.slow_in || self.faults.in_limit.is_some_and(|n| self.bulk.done >= n) {
            return Reply::Nak;
        }
        let bulk = &mut self.bulk;
        if bulk.empty > 0 {
            bulk.empty -= 1;
            bulk.ready = false;
            return Reply::Data(Vec::new());
        }
        if bulk.stall_in > 0 {
            bulk.stall_in -= 1;
            bulk.ready = false;
            return Reply::Stall;
        }
        if let Run::Streaming(stream) = &mut self.run {
            bulk.ready = false;
            bulk.done += 1;
            return Reply::Data(stream.take(STREAM_BYTES.min(max)));
        }
        let Some(data) = &bulk.data else {
            return Reply::Nak;
        };
        let left = data.len() - bulk.sent;
        if left == 0 {
            return Reply::Nak;
        }
        let n = left.min(max);
        let out = data[bulk.sent..bulk.sent + n].to_vec();
        bulk.sent += n;
        bulk.ready = false;
        bulk.done += 1;
        Reply::Data(out)
    }

    fn in_unlinked(&mut self, ep: u8, max: usize) -> Option<Vec<u8>> {
        if ep != 6 || !self.faults.slow_in || !self.bulk.started || !self.bulk.ready {
            return None;
        }
        let bulk = &mut self.bulk;
        if let Run::Streaming(stream) = &mut self.run {
            bulk.ready = false;
            return Some(stream.take(STREAM_BYTES.min(max)));
        }
        let data = bulk.data.as_ref()?;
        let n = (data.len() - bulk.sent).min(max);
        if n == 0 {
            return None;
        }
        let out = data[bulk.sent..bulk.sent + n].to_vec();
        bulk.sent += n;
        bulk.ready = false;
        Some(out)
    }

    fn take_event_point(&mut self) -> bool {
        std::mem::take(&mut self.point)
    }

    fn in_submitted(&mut self, ep: u8, _max: usize) {
        if ep == 6 && self.bulk.started {
            self.bulk.ready = true;
        }
    }

    fn reset(&mut self) {
        // A new attach ends a capture and a download; the firmware,
        // the bitstream and the PWM outputs keep running.
        if !matches!(self.run, Run::Idle) {
            self.run = Run::Idle;
        }
        self.bulk = Bulk::default();
    }

    /// Firmware chunks of 4 KiB, bitstream chunks, downloads.
    fn log_digest_above(&self) -> Option<usize> {
        Some(crate::txlog::HASH_ABOVE)
    }

    fn take_renumeration(&mut self) -> bool {
        std::mem::take(&mut self.renumerate)
    }
}

#[cfg(test)]
mod tests;
