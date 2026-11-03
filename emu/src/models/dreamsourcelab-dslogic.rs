// Counterpart in sigrok:
//   libsigrok/src/hardware/dreamsourcelab-dslogic/api.c (the profiles, the scan, the open),
//   libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.c (the commands, the FPGA
//   setting packet, the trigger packet and the sample transfers),
//   libsigrok/src/hardware/dreamsourcelab-dslogic/protocol.h,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   with commits by Uwe Hermann, Joel Holdsworth, Gerhard Sittig,
//   Romain Tartière, Joerg Alpers
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The DreamSourceLab DSLogic family as libsigrok's
//! `dreamsourcelab-dslogic` driver sees it: an FX2 that takes its
//! firmware through the boot loader and renumerates
//! ([`crate::shells::ezusb`]), then an FPGA that takes its bitstream and
//! a setting packet over bulk OUT endpoint 2 and sends a trigger packet
//! and the samples over bulk IN endpoint 6.
//!
//! Five profiles, those of the driver's `supported_device[]` (`api.c`),
//! all `2a0e:PID` before and after the upload: `plus` (DSLogic Plus,
//! `0020`, the default), `basic` (DSLogic Basic, `0021`, a memory depth
//! of 256 Kibit), `pro` (DSLogic Pro, `0003`), `dslogic` (the first
//! DSLogic, `0001`, whose threshold is chosen by loading the 3.3 V or
//! the 5 V bitstream) and `dscope` (DSCope, `0002`, which this driver
//! drives as a logic analyzer too). Each has its FX2 firmware file and
//! FPGA bitstream (`protocol.h`); without `flashed=1` the device comes
//! up as an FX2 whose EEPROM holds the ids only (no strings), takes the
//! upload and renumerates; with it, it comes up running the firmware.
//!
//! Facts and where they come from:
//!
//! - The firmware's strings: manufacturer `DreamSourceLab`, product
//!   `USB-based Instrument` (the driver's `usb_match_manuf_prod()` test
//!   for a running firmware). The rest of the descriptors are not in
//!   any source here: no serial number string, `bcdDevice` 0, interface
//!   0 with bulk OUT 2 and bulk IN 6 of 512 bytes at high speed (the
//!   endpoints the driver uses, `protocol.c`). Not checked against a
//!   device.
//! - The vendor requests (`protocol.c`): `0xb0` IN 2 bytes, the firmware
//!   version (major 1 required, `DSLOGIC_REQUIRED_VERSION_MAJOR`; the
//!   model answers 1.0, the minor is not known); `0xb1` IN 1 byte,
//!   REVID (1, a CY7C68013A, as `dslogic_dev_open()` logs it); `0xb2`
//!   OUT 3 bytes, start (flags `0x30`, LA mode and wide samples) or stop
//!   (`0x80`), delay bytes 0; `0xb3` OUT 3 zero bytes, the bitstream
//!   follows on bulk OUT 2; `0xb4` OUT 3 bytes, the setting packet's
//!   length in 16 bit words, little endian, which follows on bulk OUT 2;
//!   `0xb8` OUT 2 bytes, a register write, the value then the address
//!   (`0x78`, the threshold DAC: `threshold / 5 V * 255`). The firmware
//!   has more (`0xb5` to `0xbc`), which the driver never sends; the
//!   model STALLs them and flags them, their answers being unknown.
//! - The setting packet is `struct fpga_config` (packed, 360 bytes):
//!   sync `0xf5a5f5a5`, then TLV fields whose header is the variable
//!   number in the high byte and the length in words in the low one
//!   (`_DS_CFG`): mode (16 bit), divider (32, `ceil(100 MHz / rate)`),
//!   count (32, samples / 16), trigger position (32, a multiple of 64
//!   samples), global trigger (16, enabled channels << 4 | stages - 1),
//!   channel enable mask (16), then the trigger block of 160 words:
//!   masks, values, edges and logic of the 16 stages (each twice, the
//!   driver writes both copies alike) and 16 counts of 32 bits; end sync
//!   `0xfa5afa5a`. Mode bits: trigger enable, external clock, falling
//!   clock edge, RLE, half (200 MHz), quarter (400 MHz), stream.
//! - The trigger packet is `struct dslogic_trigger_pos` (512 bytes); the
//!   driver reads only `real_pos`, the trigger's sample in the data
//!   that follows. The model answers 0 there without a trigger (no
//!   trigger mark), and zero in the other fields, which are not known.
//! - The samples come in atoms of 64 samples: for each enabled channel
//!   in index order, a little endian 64 bit word whose bit `b` is the
//!   channel at sample `b` of the atom (`deinterleave_buffer()`).
//!
//! The samples are a 16 bit counter, so channel `n` toggles every 2^n
//! samples. A trigger is read as `set_trigger()` writes it into stage 0:
//! a channel whose mask bit is clear must equal its value bit, one with
//! its edge bit set must have changed since the previous sample (the
//! `edge` match sets the edge bit and leaves the mask set: any change).
//! With a trigger the FPGA keeps `trig_pos` samples before it, so the
//! data starts `trig_pos` samples before the first sample at or after
//! `trig_pos` that matches, and `real_pos` is `trig_pos`. That reading
//! of the FPGA's registers comes from the driver only.
//!
//! Emulation artifacts: the trigger packet completes right after the
//! start; the log shows the global trigger word of a setting packet
//! without a trigger as 0, as the driver leaves it uninitialised and it
//! changes from run to run ([`DeviceModel::log_out`]); the samples
//! ignore the rate, the external clock and its edge and the threshold
//! (the model takes a threshold register write on every profile; whether
//! the first DSLogic, whose threshold is its bitstream, has that
//! register is not known); a trigger that never matches (`0=1,3=r`: bit
//! 3 rises on even counts only) is never answered, as on a device, and
//! the driver waits for ever; RLE (set by the driver for captures deeper than
//! 16 Msamples per 100 MHz) changes nothing in the data; a buffered
//! capture's depth (the count) does not end the data, the model streams
//! until the host stops; and one bulk IN transfer completes per
//! transfer the host submits after the previous one completed (the
//! trigger packet first, then the first of the batch the driver
//! submits at once, then one per resubmission), where the real device
//! fills every queued transfer as fast as the bus goes: the
//! transactions of a run do not depend on timing.
//!
//! What the model taught about the C driver, see the scenario
//! (`tools/emu/dreamsourcelab-dslogic.sh`): `fpga_configure()` sends the
//! global trigger word uninitialised when there is no trigger (the
//! model ignores it then); `config_get(CLOCK_EDGE)` always answers
//! `rising`; the first DSLogic is opened with the 3.3 V bitstream and
//! then reports, and writes to the threshold register, the 1.4-3.6 V
//! range of the 5 V one; the 90 % bound of the trigger position
//! overflows 32 bits for the 256 Mibit boards (it comes out at 10 %);
//! and a trigger packet that STALLs, or comes short, leaves the
//! acquisition waiting for ever (the transfer has no timeout).
//!
//! The pedantic checks: the FX2 upload byte for byte as `ezusb.c` sends
//! it, of the profile's file; the bitstream byte for byte, one of the
//! profile's files, only after `0xb3`; the setting packet only after
//! `0xb4` announcing its 180 words, every header and sync, mode bits the
//! driver never sets, a falling clock edge without the external clock,
//! half and quarter mode together or with a divider other than 1, a
//! divider whose rate the driver's list lacks, RLE where the driver's
//! rule would not set it (or missing where it would), a trigger position
//! off the 64 sample grid or beyond the memory, a global trigger word
//! other than the driver's, trigger stages other than 0 not at their
//! defaults, the two copies differing, no channel enabled; a register
//! write before the FPGA runs or to another register; a start without
//! a fresh setting packet or before the bitstream, start flags other
//! than the two; a trigger packet asked for in other than 512 bytes,
//! sample transfers not a multiple of 512 bytes per enabled channel,
//! IN transfers on other endpoints, unknown requests or lengths, and a
//! firmware upload to the running firmware.
//!
//! Faults, all off by default: `version=MAJOR.MINOR` answers another
//! firmware version; `stall=version,revid,start,stop,config,setting,wr_reg`
//! STALLs those requests (`start` and `stop` are the two forms of
//! `0xb2`); `stall_out=fpga,setting` STALLs the bulk OUT of the
//! bitstream or the setting packet, `short_out=fpga,setting` takes only
//! half of it; `trigger=stall|short|never` STALLs the trigger packet,
//! sends 24 bytes of it, or never sends it (the driver then waits for
//! ever, so no case uses them); `empty=N` completes the first N sample
//! transfers of each acquisition with no data and `stall_in=N` STALLs
//! the N after those, paced like the samples; `short_in=N` cuts the Nth
//! sample transfer of each acquisition to half its whole atoms plus 4
//! bytes (the rest of the cut atom is lost, the next transfer goes on
//! at the next atom); `load=cpucs|chunk|release` STALLs the boot
//! loader's first `CPUCS` write (the reset hold), every firmware chunk,
//! or the `CPUCS` write that releases the reset, so the upload fails. The
//! C driver then opens the FX2 without firmware all the same and asks
//! it for the firmware version, which the boot loader STALLs; the model
//! flags no request to the bare chip once its upload fault has fired.

#![allow(clippy::doc_markdown)]

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, VENDOR, Violations};
use crate::shells::ezusb::{self, Chip, Loader};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "dreamsourcelab-dslogic";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "DreamSourceLab DSLogic (EZ-USB, renumerates, FPGA bitstream); \
                         profile=plus (2a0e:0020, default), basic, pro, dslogic or dscope, \
                         flashed=1 starts with the firmware running; faults: version=MAJOR.MINOR, \
                         stall=version,revid,start,stop,config,setting,wr_reg, \
                         stall_out=fpga,setting, short_out=fpga,setting, \
                         trigger=stall|short|never, empty=N, stall_in=N, short_in=N, \
                         load=cpucs|chunk|release";

/// DreamSourceLab's vendor id.
pub const VID: u16 = 0x2a0e;

/// A device of the driver's table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// The option value.
    pub key: &'static str,
    /// Product id, before and after the upload.
    pub product: u16,
    /// The FX2 firmware file and its stand-in size.
    pub firmware: (&'static str, usize),
    /// The FPGA bitstreams the driver may load, with their stand-in
    /// sizes; the first is the one `dev_open()` loads.
    pub bitstreams: &'static [(&'static str, usize)],
    /// Memory depth in samples per channel (`mem_depth`, "bits").
    pub mem_depth: u64,
}

/// The emulated devices. The stand-in sizes are the model's own (the
/// real files are not needed: the model checks the stand-in byte for
/// byte); `tools/emu/dreamsourcelab-dslogic.firmware` lists the same.
pub const PROFILES: [Profile; 5] = [
    Profile {
        key: "plus",
        product: 0x0020,
        firmware: ("dreamsourcelab-dslogic-plus-fx2.fw", 8_404),
        bitstreams: &[("dreamsourcelab-dslogic-plus-fpga.fw", 341_212)],
        mem_depth: 256 * 1024 * 1024,
    },
    Profile {
        key: "basic",
        product: 0x0021,
        firmware: ("dreamsourcelab-dslogic-basic-fx2.fw", 8_412),
        bitstreams: &[("dreamsourcelab-dslogic-basic-fpga.fw", 341_220)],
        mem_depth: 256 * 1024,
    },
    Profile {
        key: "pro",
        product: 0x0003,
        firmware: ("dreamsourcelab-dslogic-pro-fx2.fw", 8_396),
        bitstreams: &[("dreamsourcelab-dslogic-pro-fpga.fw", 583_340)],
        mem_depth: 256 * 1024 * 1024,
    },
    Profile {
        key: "dslogic",
        product: 0x0001,
        firmware: ("dreamsourcelab-dslogic-fx2.fw", 8_388),
        bitstreams: &[
            ("dreamsourcelab-dslogic-fpga-3v3.fw", 340_604),
            ("dreamsourcelab-dslogic-fpga-5v.fw", 340_612),
        ],
        mem_depth: 256 * 1024 * 1024,
    },
    Profile {
        key: "dscope",
        product: 0x0002,
        firmware: ("dreamsourcelab-dscope-fx2.fw", 8_420),
        bitstreams: &[("dreamsourcelab-dscope-fpga.fw", 583_348)],
        mem_depth: 256 * 1024 * 1024,
    },
];

const CMD_GET_FW_VERSION: u8 = 0xb0;
const CMD_GET_REVID_VERSION: u8 = 0xb1;
const CMD_START: u8 = 0xb2;
const CMD_CONFIG: u8 = 0xb3;
const CMD_SETTING: u8 = 0xb4;
const CMD_WR_REG: u8 = 0xb8;
/// The `stall=` name for `0xb2` with the stop flag.
const STOP: u16 = 0x1b2;

const FLAGS_STOP: u8 = 1 << 7;
const FLAGS_START: u8 = (1 << 4) | (1 << 5);
/// The threshold register.
const ADDR_VTH: u8 = 0x78;

/// The firmware version, major as `DSLOGIC_REQUIRED_VERSION_MAJOR`.
pub const VERSION: [u8; 2] = [1, 0];
/// `REVID` of a CY7C68013A.
pub const REVID: u8 = 1;

const MODE_TRIG_EN: u16 = 1 << 0;
const MODE_CLK_TYPE: u16 = 1 << 1;
const MODE_CLK_EDGE: u16 = 1 << 2;
const MODE_RLE: u16 = 1 << 3;
const MODE_HALF: u16 = 1 << 5;
const MODE_QUAR: u16 = 1 << 6;
const MODE_STREAM: u16 = 1 << 12;
const MODE_DRIVER: u16 =
    MODE_TRIG_EN | MODE_CLK_TYPE | MODE_CLK_EDGE | MODE_RLE | MODE_HALF | MODE_QUAR | MODE_STREAM;

/// `sizeof(struct fpga_config)`.
pub const SETTING_LEN: usize = 360;
/// `sizeof(struct dslogic_trigger_pos)`.
pub const TRIGGER_LEN: usize = 512;
const STAGES: usize = 16;
/// `DS_MAX_LOGIC_SAMPLERATE`.
const BASE_RATE: u64 = 100_000_000;
/// `DS_MAX_LOGIC_DEPTH`.
const MAX_DEPTH: u64 = 16_000_000;

/// The rates of the driver's list up to 100 MHz, `api.c`, as dividers
/// of 100 MHz; 200 and 400 MHz are half and quarter mode, divider 1.
const DIVIDERS: [u32; 14] = [10_000, 5_000, 2_000, 1_000, 500, 200, 100, 50, 20, 10, 5, 4, 2, 1];

/// Ways the device can misbehave, as a real one can; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    /// The firmware version `0xb0` answers, when not [`VERSION`].
    pub version: Option<[u8; 2]>,
    /// Requests the firmware STALLs; `0xb2` with the stop flag is
    /// `0x1b2`.
    pub stall: Vec<u16>,
    /// Bulk OUT the firmware STALLs: the bitstream, the setting packet.
    pub stall_out: [bool; 2],
    /// Bulk OUT of which the firmware takes only half.
    pub short_out: [bool; 2],
    /// The trigger packet: STALLed, short, or never sent.
    pub trigger: Option<TriggerFault>,
    /// Sample transfers of each acquisition that complete with no data.
    pub empty: u32,
    /// Sample transfers of each acquisition that STALL, after those.
    pub stall_in: u32,
    /// The sample transfer, from 1, that comes cut mid-atom.
    pub short_in: u32,
    /// The boot loader writes that STALL.
    pub load: Option<LoadFault>,
}

/// Which boot loader write STALLs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadFault {
    /// `CPUCS` 1, the reset hold.
    Hold,
    /// Every firmware chunk.
    Chunk,
    /// `CPUCS` 0, the reset release.
    Release,
}

/// How the trigger packet goes wrong.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriggerFault {
    /// A STALL.
    Stall,
    /// 24 bytes, the header without the first block.
    Short,
    /// It never comes, as when the trigger never matches.
    Never,
}

impl Faults {
    /// Takes the option `k=v` when it is a fault; false when `k` is no
    /// fault option.
    fn parse(&mut self, k: &str, v: &str) -> Result<bool> {
        let count = |v: &str| v.parse::<u32>().map_err(|_| Error(format!("{k}={v}: a count")));
        let outs = |v: &str| -> Result<[bool; 2]> {
            let mut o = [false; 2];
            for name in v.split(',').filter(|n| !n.is_empty()) {
                match name {
                    "fpga" => o[0] = true,
                    "setting" => o[1] = true,
                    _ => return Err(Error(format!("{k}={name}: fpga or setting"))),
                }
            }
            Ok(o)
        };
        match k {
            "version" => {
                let bad = || Error(format!("version={v}: MAJOR.MINOR, each 0 to 255"));
                let (major, minor) = v.split_once('.').ok_or_else(bad)?;
                self.version = Some([major.parse().map_err(|_| bad())?, minor.parse().map_err(|_| bad())?]);
            }
            "stall" => {
                self.stall.clear();
                for name in v.split(',').filter(|n| !n.is_empty()) {
                    self.stall.push(match name {
                        "version" => u16::from(CMD_GET_FW_VERSION),
                        "revid" => u16::from(CMD_GET_REVID_VERSION),
                        "start" => u16::from(CMD_START),
                        "stop" => STOP,
                        "config" => u16::from(CMD_CONFIG),
                        "setting" => u16::from(CMD_SETTING),
                        "wr_reg" => u16::from(CMD_WR_REG),
                        _ => {
                            return Err(Error(format!(
                                "stall={name}: version, revid, start, stop, config, setting or wr_reg"
                            )));
                        }
                    });
                }
            }
            "stall_out" => self.stall_out = outs(v)?,
            "short_out" => self.short_out = outs(v)?,
            "trigger" => {
                self.trigger = Some(match v {
                    "stall" => TriggerFault::Stall,
                    "short" => TriggerFault::Short,
                    "never" => TriggerFault::Never,
                    _ => return Err(Error(format!("trigger={v}: stall, short or never"))),
                });
            }
            "empty" => self.empty = count(v)?,
            "stall_in" => self.stall_in = count(v)?,
            "short_in" => self.short_in = count(v)?,
            "load" => {
                self.load = Some(match v {
                    "cpucs" => LoadFault::Hold,
                    "chunk" => LoadFault::Chunk,
                    "release" => LoadFault::Release,
                    _ => return Err(Error(format!("load={v}: cpucs, chunk or release"))),
                });
            }
            _ => return Ok(false),
        }
        Ok(true)
    }
}

/// A device from the `-o` options: `profile=plus|basic|pro|dslogic|dscope`,
/// `flashed=0|1`, and the faults (see [`Faults`]).
///
/// # Errors
///
/// On an unknown option or value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut profile = PROFILES[0];
    let mut flashed = false;
    let mut faults = Faults::default();
    for (k, v) in options {
        if faults.parse(k, v)? {
            continue;
        }
        match k.as_str() {
            "profile" => {
                profile = *PROFILES.iter().find(|p| p.key == v).ok_or_else(|| {
                    let keys: Vec<&str> = PROFILES.iter().map(|p| p.key).collect();
                    Error(format!("profile={v}: one of {}", keys.join(", ")))
                })?;
            }
            "flashed" => {
                flashed = match v.as_str() {
                    "0" => false,
                    "1" => true,
                    _ => return Err(Error(format!("flashed={v}: 0 or 1"))),
                }
            }
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    let mut model = Dslogic::new(profile, flashed);
    model.faults = faults;
    Ok(Box::new(model))
}

/// The descriptors of the running firmware.
pub fn firmware_descriptors(p: &Profile) -> Descriptors {
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
        product: p.product,
        device: 0x0000,
        strings: [1, 2, 0],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::High,
        device: dev.bytes(),
        configs: vec![config],
        strings: vec!["DreamSourceLab".into(), "USB-based Instrument".into()],
    }
}

/// The setting packet, decoded.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Setting {
    /// The mode bits.
    pub mode: u16,
    /// 100 MHz divided by this is the rate (1 in half or quarter mode).
    pub divider: u32,
    /// Samples / 16.
    pub count: u32,
    /// Samples kept before the trigger.
    pub trig_pos: u32,
    /// The enabled channels.
    pub ch_en: u16,
    /// Stage 0: mask, value, edge.
    pub trigger: [u16; 3],
}

impl Setting {
    /// The sample rate.
    pub fn rate(&self) -> u64 {
        if self.mode & MODE_HALF != 0 {
            2 * BASE_RATE
        } else if self.mode & MODE_QUAR != 0 {
            4 * BASE_RATE
        } else {
            BASE_RATE / u64::from(self.divider.max(1))
        }
    }

    /// Whether the trigger of stage 0 holds at sample `n` of the counter.
    fn matches(&self, n: u64) -> bool {
        let [mask, value, edge] = self.trigger;
        let (now, before) = (sample(n), sample(n.wrapping_sub(1)));
        if edge != 0 && n == 0 {
            return false;
        }
        (now ^ value) & !mask == 0 && (now ^ before) & edge == edge
    }

    /// Parses and checks the 360 bytes against `fpga_configure()`.
    fn parse(b: &[u8], profile: &Profile, v: &mut Violations) -> Setting {
        let u16_at = |at: usize| u16::from_le_bytes([b[at], b[at + 1]]);
        let u32_at = |at: usize| u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]]);
        let headers: [(usize, u32, &str); 9] = [
            (0, 0xf5a5_f5a5, "sync"),
            (4, 0x0001, "mode header"),
            (8, 0x0102, "divider header"),
            (14, 0x0302, "count header"),
            (20, 0x0502, "trigger position header"),
            (26, 0x0701, "global trigger header"),
            (30, 0x0801, "channel enable header"),
            (34, 0x40a0, "trigger header"),
            (356, 0xfa5a_fa5a, "end sync"),
        ];
        for (at, want, what) in headers {
            let got = if want > 0xffff { u32_at(at) } else { u32::from(u16_at(at)) };
            if got != want {
                v.report(format!("setting packet: {what} at {at} is {got:#x}, not {want:#x}"));
            }
        }
        // The eight arrays of 16 words after the trigger header.
        let arr = |n: usize| -> [u16; STAGES] { std::array::from_fn(|i| u16_at(36 + n * 32 + 2 * i)) };
        let (mask0, mask1, value0, value1) = (arr(0), arr(1), arr(2), arr(3));
        let (edge0, edge1, logic0, logic1) = (arr(4), arr(5), arr(6), arr(7));
        let counts: [u32; STAGES] = std::array::from_fn(|i| u32_at(292 + 4 * i));
        let s = Setting {
            mode: u16_at(6),
            divider: u32_at(10),
            count: u32_at(16),
            trig_pos: u32_at(22),
            ch_en: u16_at(32),
            trigger: [mask0[0], value0[0], edge0[0]],
        };
        let mode = s.mode;
        if mode & !MODE_DRIVER != 0 {
            v.report(format!("setting packet: mode {mode:#06x} has bits the driver never sets"));
        }
        if mode & MODE_CLK_EDGE != 0 && mode & MODE_CLK_TYPE == 0 {
            v.report("setting packet: a falling clock edge without the external clock");
        }
        if mode & MODE_HALF != 0 && mode & MODE_QUAR != 0 {
            v.report("setting packet: half and quarter mode at once");
        }
        if mode & (MODE_HALF | MODE_QUAR) != 0 && s.divider != 1 {
            v.report(format!("setting packet: divider {} in half or quarter mode", s.divider));
        }
        if !DIVIDERS.contains(&s.divider) {
            v.report(format!(
                "setting packet: divider {}: 100 MHz / {} is no rate of the driver's list",
                s.divider, s.divider
            ));
        }
        // fpga_configure(): RLE when limit_samples > 16 M * ceil(rate /
        // 100 MHz) in buffered mode; the limit is count * 16 plus 0 to 15.
        let stream = mode & MODE_STREAM != 0;
        let rle_above = MAX_DEPTH * s.rate().div_ceil(BASE_RATE);
        let (low, high) = (u64::from(s.count) * 16, u64::from(s.count) * 16 + 15);
        if mode & MODE_RLE != 0 && (stream || high <= rle_above) {
            v.report(format!(
                "setting packet: RLE for {low} samples{}; the driver sets it above {rle_above} in buffered mode",
                if stream { " in stream mode" } else { "" }
            ));
        }
        if mode & MODE_RLE == 0 && !stream && low > rle_above {
            v.report(format!(
                "setting packet: no RLE for {low} samples; the driver sets it above {rle_above}"
            ));
        }
        let depth = profile.mem_depth;
        let most = if stream { depth / 10 } else { depth * 9 / 10 };
        if !s.trig_pos.is_multiple_of(64) || s.trig_pos < 64 || u64::from(s.trig_pos) > most {
            v.report(format!(
                "setting packet: trigger position {}: a multiple of 64 from 64 to {most}",
                s.trig_pos
            ));
        }
        if s.ch_en == 0 {
            v.report("setting packet: no channel enabled");
        }
        if mode & MODE_TRIG_EN != 0 {
            let glb = u16_at(28);
            let want = (s.ch_en.count_ones() as u16) << 4;
            if glb != want {
                v.report(format!(
                    "setting packet: global trigger {glb:#06x}, where {} enabled channels and one \
                     stage make {want:#06x}",
                    s.ch_en.count_ones()
                ));
            }
        }
        check_stages([&mask0, &mask1, &value0, &value1, &edge0, &edge1, &logic0, &logic1], &counts, v);
        s
    }
}

/// The trigger block's arrays as `set_trigger()` leaves them: both
/// copies alike, stage 0 the trigger, the others at their defaults
/// (mask 0xffff, value and edge 0), logic 2 and count 0 everywhere.
fn check_stages(a: [&[u16; STAGES]; 8], counts: &[u32; STAGES], v: &mut Violations) {
    let [mask0, mask1, value0, value1, edge0, edge1, logic0, logic1] = a;
    if mask0 != mask1 || value0 != value1 || edge0 != edge1 || logic0 != logic1 {
        v.report("setting packet: the two copies of the trigger stages differ");
    }
    for i in 0..STAGES {
        let default = (i == 0 || (mask0[i] == 0xffff && value0[i] == 0 && edge0[i] == 0))
            && logic0[i] == 2
            && counts[i] == 0;
        if !default {
            v.report(format!(
                "setting packet: trigger stage {i} is not as set_trigger() leaves it (mask \
                 {:#06x}, value {:#06x}, edge {:#06x}, logic {}, count {})",
                mask0[i], value0[i], edge0[i], logic0[i], counts[i]
            ));
        }
    }
}

/// The counter's sample `n`.
fn sample(n: u64) -> u16 {
    n as u16
}

/// What the FPGA holds.
#[derive(Clone, Debug)]
enum Fpga {
    /// Nothing, or a load that went wrong.
    Unconfigured,
    /// Taking the bitstream after `0xb3`: the bytes so far.
    Loading(Vec<u8>),
    /// Running this bitstream.
    Loaded(&'static str),
}

/// A running acquisition.
#[derive(Clone, Debug)]
struct Acquisition {
    setting: Setting,
    /// The counter's sample at the start of the data.
    base: u64,
    /// Samples sent so far.
    sent: u64,
    /// `real_pos`, or `None` until the trigger packet has gone.
    trigger: Option<u32>,
    /// Whether the next bulk IN transfer may complete.
    ready: bool,
    /// Whether the last transfer offered has completed, so that the
    /// next submission makes one ready.
    idle: bool,
    empty: u32,
    stall_in: u32,
    /// Sample transfers completed with data.
    transfers: u32,
}

/// The firmware's state.
#[derive(Clone, Debug)]
struct Firmware {
    fpga: Fpga,
    /// Bytes of a setting packet `0xb4` announced, still to come.
    setting_due: Option<usize>,
    /// A setting packet came since the last start.
    setting: Option<Setting>,
    acq: Option<Acquisition>,
}

/// What the device is doing.
#[derive(Clone, Debug)]
enum Stage {
    /// The boot loader, waiting for the firmware.
    Boot(Box<Loader>),
    /// The firmware runs.
    Running(Box<Firmware>),
}

fn running() -> Stage {
    Stage::Running(Box::new(Firmware {
        fpga: Fpga::Unconfigured,
        setting_due: None,
        setting: None,
        acq: None,
    }))
}

/// The emulated device.
pub struct Dslogic {
    profile: Profile,
    boot: Descriptors,
    firmware: Descriptors,
    stage: Stage,
    renumerate: bool,
    faults: Faults,
    /// The `load=` fault has fired.
    upload_failed: bool,
}

impl Dslogic {
    /// A device; `flashed` starts it with the firmware running.
    pub fn new(profile: Profile, flashed: bool) -> Dslogic {
        Dslogic {
            boot: ezusb::bare_descriptors(VID, profile.product, ezusb::FX2LP_DID),
            firmware: firmware_descriptors(&profile),
            profile,
            stage: if flashed { running() } else { Stage::Boot(Box::new(Loader::new(Chip::Fx2lp))) },
            renumerate: false,
            faults: Faults::default(),
            upload_failed: false,
        }
    }

    /// Whether the firmware runs.
    pub fn running(&self) -> bool {
        matches!(self.stage, Stage::Running(_))
    }

    /// The bitstream the FPGA runs, if any.
    pub fn bitstream(&self) -> Option<&'static str> {
        match &self.stage {
            Stage::Running(fw) => match fw.fpga {
                Fpga::Loaded(name) => Some(name),
                _ => None,
            },
            Stage::Boot(_) => None,
        }
    }

    fn boot_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        let (name, size) = self.profile.firmware;
        let Stage::Boot(loader) = &mut self.stage else { unreachable!() };
        let load = setup.kind() == VENDOR && setup.request == ezusb::REQ_FIRMWARE_LOAD;
        let cpucs = setup.value == ezusb::CPUCS;
        let hit = match self.faults.load {
            Some(LoadFault::Hold) => cpucs && data.first() == Some(&1),
            Some(LoadFault::Chunk) => !cpucs,
            Some(LoadFault::Release) => cpucs && data.first() == Some(&0),
            None => false,
        };
        if load && !setup.is_in() && hit {
            self.upload_failed = true;
            return Reply::Stall;
        }
        let Some((reply, started)) = loader.control(setup, data, v) else {
            if self.upload_failed {
                // dslogic_dev_open() after a failed upload: the boot
                // loader STALLs what it does not know.
                return Reply::Stall;
            }
            v.report(format!(
                "request {:#04x} {:#04x} value {:#06x} to the FX2 without firmware; only its \
                 boot loader (0xa0) answers",
                setup.request_type, setup.request, setup.value
            ));
            return Reply::Stall;
        };
        if started {
            loader.check_ezusb_c(&ezusb::standin(name, size), v);
            self.stage = running();
            self.renumerate = true;
        }
        reply
    }

    fn firmware_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        let Stage::Running(fw) = &mut self.stage else { unreachable!() };
        if setup.kind() == VENDOR && setup.request == ezusb::REQ_FIRMWARE_LOAD {
            v.report("firmware load to an FX2 whose DSLogic firmware runs");
            return Reply::ack();
        }
        let length = match (setup.request_type, setup.request) {
            (0xc0, CMD_GET_FW_VERSION) | (0x40, CMD_WR_REG) => 2,
            (0xc0, CMD_GET_REVID_VERSION) => 1,
            (0x40, CMD_START | CMD_CONFIG | CMD_SETTING) => 3,
            _ => {
                v.report(format!(
                    "request {:#04x} {:#04x} value {:#06x}: not one the driver sends",
                    setup.request_type, setup.request, setup.value
                ));
                return Reply::Stall;
            }
        };
        if setup.value != 0 || setup.index != 0 {
            v.report(format!(
                "request {:#04x} with value {:#06x} index {:#06x}; the driver sends 0",
                setup.request, setup.value, setup.index
            ));
        }
        if usize::from(setup.length) != length || (!setup.is_in() && data.len() != length) {
            v.report(format!(
                "request {:#04x} with {} bytes; the driver sends {length}",
                setup.request, setup.length
            ));
            return Reply::Stall;
        }
        let key = if setup.request == CMD_START && data.first() == Some(&FLAGS_STOP) {
            STOP
        } else {
            u16::from(setup.request)
        };
        if self.faults.stall.contains(&key) {
            return Reply::Stall;
        }
        if fw.setting_due.is_some() {
            v.report(format!(
                "request {:#04x} while the setting packet 0xb4 announced is still due",
                setup.request
            ));
            fw.setting_due = None;
        }
        if let Fpga::Loading(_) = fw.fpga
            && setup.request != CMD_GET_FW_VERSION
        {
            v.report(format!(
                "request {:#04x} while the FPGA takes its bitstream; it is not complete",
                setup.request
            ));
            fw.fpga = Fpga::Unconfigured;
        }
        match setup.request {
            CMD_GET_FW_VERSION => Reply::Data(self.faults.version.unwrap_or(VERSION).to_vec()),
            CMD_GET_REVID_VERSION => Reply::Data(vec![REVID]),
            CMD_CONFIG => {
                if data != [0, 0, 0] {
                    v.report(format!("FPGA configure command {data:02x?}; the driver sends zeros"));
                }
                if fw.acq.is_some() {
                    v.report("FPGA bitstream load during an acquisition");
                }
                fw.acq = None;
                fw.setting = None;
                fw.fpga = Fpga::Loading(Vec::new());
                Reply::ack()
            }
            CMD_SETTING => {
                let words = u32::from_le_bytes([data[0], data[1], data[2], 0]) as usize;
                if words * 2 != SETTING_LEN {
                    v.report(format!(
                        "setting packet of {words} words announced; struct fpga_config has {}",
                        SETTING_LEN / 2
                    ));
                }
                if !matches!(fw.fpga, Fpga::Loaded(_)) {
                    v.report("setting packet before the FPGA has its bitstream");
                }
                fw.setting_due = Some(words * 2);
                Reply::ack()
            }
            CMD_WR_REG => {
                if data[1] != ADDR_VTH {
                    v.report(format!(
                        "register write to {:#04x}; the driver writes the threshold, {ADDR_VTH:#04x}",
                        data[1]
                    ));
                }
                if !matches!(fw.fpga, Fpga::Loaded(_)) {
                    v.report("register write before the FPGA has its bitstream");
                }
                Reply::ack()
            }
            _ => {
                start_command(fw, data, &self.faults, v);
                Reply::ack()
            }
        }
    }

    fn fpga_out(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        let profile = self.profile;
        let Stage::Running(fw) = &mut self.stage else { unreachable!() };
        if let Some(due) = fw.setting_due.take() {
            if self.faults.stall_out[1] {
                return Reply::Stall;
            }
            if self.faults.short_out[1] {
                return Reply::Data(vec![0; data.len() / 2]);
            }
            if data.len() != due || due != SETTING_LEN {
                v.report(format!(
                    "setting packet of {} bytes, after 0xb4 announced {due}; struct fpga_config has \
                     {SETTING_LEN}",
                    data.len()
                ));
                return Reply::ack();
            }
            fw.setting = Some(Setting::parse(data, &profile, v));
            return Reply::ack();
        }
        let Fpga::Loading(got) = &mut fw.fpga else {
            v.report(format!(
                "bulk OUT of {} bytes with neither 0xb3 nor 0xb4 before it",
                data.len()
            ));
            return Reply::Stall;
        };
        if self.faults.stall_out[0] {
            return Reply::Stall;
        }
        let take = if self.faults.short_out[0] { data.len() / 2 } else { data.len() };
        got.extend_from_slice(&data[..take]);
        let candidates: Vec<(&'static str, Vec<u8>)> = profile
            .bitstreams
            .iter()
            .map(|&(name, size)| (name, ezusb::standin(name, size)))
            .collect();
        if let Some((name, _)) = candidates.iter().find(|(_, image)| image == got) {
            fw.fpga = Fpga::Loaded(name);
        } else if !candidates.iter().any(|(_, image)| image.starts_with(got)) {
            let names: Vec<&str> = candidates.iter().map(|(n, _)| *n).collect();
            v.report(format!(
                "FPGA bitstream: the {} bytes so far are no prefix of {}",
                got.len(),
                names.join(" or ")
            ));
            fw.fpga = Fpga::Unconfigured;
        }
        if take < data.len() { Reply::Data(vec![0; take]) } else { Reply::ack() }
    }
}

/// `0xb2`: a stop, or a start with the setting packet that came since
/// the last one.
fn start_command(fw: &mut Firmware, data: &[u8], faults: &Faults, v: &mut Violations) {
    if data[1..] != [0, 0] {
        v.report(format!("start command {data:02x?}: the delay bytes are 0"));
    }
    if data[0] == FLAGS_STOP {
        fw.acq = None;
    } else if data[0] == FLAGS_START {
        if !matches!(fw.fpga, Fpga::Loaded(_)) {
            v.report("start before the FPGA has its bitstream");
        }
        match fw.setting.take() {
            None => v.report("start without a setting packet since the last start"),
            Some(s) => fw.acq = Some(start(s, faults)),
        }
    } else {
        v.report(format!(
            "start flags {:#04x}: the driver sends {FLAGS_START:#04x} or {FLAGS_STOP:#04x}",
            data[0]
        ));
    }
}

/// A start with setting `s`.
fn start(s: Setting, faults: &Faults) -> Acquisition {
    let (base, trigger) = if s.mode & MODE_TRIG_EN == 0 {
        (0, Some(0))
    } else {
        // The first match at or after trig_pos, with trig_pos samples
        // before it; within two counter periods or never.
        let from = u64::from(s.trig_pos);
        match (from..from + (1 << 17)).find(|&n| s.matches(n)) {
            Some(n) => (n - from, Some(s.trig_pos)),
            None => (0, None),
        }
    };
    Acquisition {
        // `None` here is a trigger that never comes; `Some` is the
        // packet still to send.
        trigger: trigger.map(|t| t | 0x8000_0000),
        setting: s,
        base,
        sent: 0,
        ready: false,
        idle: true,
        empty: faults.empty,
        stall_in: faults.stall_in,
        transfers: 0,
    }
}

impl DeviceModel for Dslogic {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        if self.running() { &self.firmware } else { &self.boot }
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if self.running() {
            self.firmware_control(setup, data, v)
        } else {
            self.boot_control(setup, data, v)
        }
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != 2 {
            v.report(format!("OUT transfer to endpoint {ep}; the FPGA takes its data on 2"));
            return Reply::Stall;
        }
        self.fpga_out(data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        let Stage::Running(fw) = &mut self.stage else { return Reply::Nak };
        if ep != 6 {
            v.report(format!("IN transfer from endpoint {ep}; the FPGA sends on 6"));
            return Reply::Stall;
        }
        let Some(acq) = &mut fw.acq else { return Reply::Nak };
        if !acq.ready {
            return Reply::Nak;
        }
        // The trigger packet first: bit 31 marks it as still to send.
        match acq.trigger {
            None => return Reply::Nak,
            Some(t) if t & 0x8000_0000 != 0 => {
                if max != TRIGGER_LEN {
                    v.report(format!(
                        "trigger packet asked for in {max} bytes; struct dslogic_trigger_pos has \
                         {TRIGGER_LEN}"
                    ));
                }
                let real_pos = t & !0x8000_0000;
                let mut packet = vec![0; TRIGGER_LEN];
                packet[4..8].copy_from_slice(&real_pos.to_le_bytes());
                match self.faults.trigger {
                    Some(TriggerFault::Never) => return Reply::Nak,
                    Some(TriggerFault::Stall) => {
                        acq.trigger = Some(real_pos);
                        acq.ready = false;
                        acq.idle = true;
                        return Reply::Stall;
                    }
                    Some(TriggerFault::Short) => packet.truncate(24),
                    None => {}
                }
                acq.trigger = Some(real_pos);
                acq.ready = false;
                acq.idle = true;
                return Reply::Data(packet);
            }
            Some(_) => {}
        }
        let channels: Vec<u32> = (0..16).filter(|c| acq.setting.ch_en & (1 << c) != 0).collect();
        let atom = 8 * channels.len().max(1);
        if !max.is_multiple_of(64 * channels.len().max(1)) || max == 0 {
            v.report(format!(
                "sample transfer of {max} bytes; the driver asks for multiples of 512 bytes per \
                 enabled channel ({} channels)",
                channels.len()
            ));
        }
        acq.ready = false;
        acq.idle = true;
        if acq.empty > 0 {
            acq.empty -= 1;
            return Reply::Data(Vec::new());
        }
        if acq.stall_in > 0 {
            acq.stall_in -= 1;
            return Reply::Stall;
        }
        let mut data = Vec::with_capacity(max);
        for _ in 0..max / atom {
            let first = acq.base + acq.sent;
            for &c in &channels {
                let mut word = 0u64;
                for b in 0..64 {
                    word |= u64::from((sample(first + b) >> c) & 1) << b;
                }
                data.extend_from_slice(&word.to_le_bytes());
            }
            acq.sent += 64;
        }
        acq.transfers += 1;
        if acq.transfers == self.faults.short_in {
            // Half the atoms and 4 bytes of the next; the rest of that
            // atom is lost.
            let atoms = max / atom / 2;
            data.truncate(atoms * atom + 4);
            acq.sent -= (max / atom - atoms - 1) as u64 * 64;
        }
        Reply::Data(data)
    }

    fn in_submitted(&mut self, ep: u8, _max: usize) {
        if let Stage::Running(fw) = &mut self.stage
            && let Some(acq) = &mut fw.acq
            && ep == 6
            && acq.idle
        {
            acq.ready = true;
            acq.idle = false;
        }
    }

    fn reset(&mut self) {
        // A new attach ends an acquisition; the FPGA keeps its bitstream.
        if let Stage::Running(fw) = &mut self.stage {
            fw.acq = None;
            fw.setting_due = None;
        }
    }

    /// Firmware chunks, bitstreams and sample transfers.
    fn log_digest_above(&self) -> Option<usize> {
        Some(crate::txlog::HASH_ABOVE)
    }

    /// Without a trigger, `fpga_configure()` leaves the global trigger
    /// word of the setting packet uninitialised (`set_trigger()` returns
    /// before it sets it): the log shows it as 0.
    fn log_out(&self, ep: u8, data: &[u8]) -> Option<Vec<u8>> {
        let setting = ep == 2 && data.len() == SETTING_LEN && data[..4] == 0xf5a5_f5a5u32.to_le_bytes();
        if !setting || u16::from_le_bytes([data[6], data[7]]) & MODE_TRIG_EN != 0 {
            return None;
        }
        let mut shown = data.to_vec();
        shown[28..30].fill(0);
        Some(shown)
    }

    fn take_renumeration(&mut self) -> bool {
        std::mem::take(&mut self.renumerate)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Device;

    fn vendor(request_type: u8, request: u8, length: u16) -> Setup {
        Setup { request_type, request, value: 0, index: 0, length }
    }

    const SET_CONFIG: Setup = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };

    /// `fpga_configure()`'s packet: the rate's divider, the limit, the
    /// enabled channels, a trigger on stage 0 (mask, value, edge) or
    /// none, and the mode bits beyond the trigger's.
    fn setting(divider: u32, limit: u32, ch_en: u16, trigger: Option<[u16; 3]>, mode: u16) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&0xf5a5_f5a5u32.to_le_bytes());
        let mode = mode | u16::from(trigger.is_some());
        for (h, val) in [(0x0001u16, u32::from(mode)), (0x0102, divider), (0x0302, limit / 16), (0x0502, 64)]
        {
            b.extend_from_slice(&h.to_le_bytes());
            if h == 0x0001 {
                b.extend_from_slice(&(val as u16).to_le_bytes());
            } else {
                b.extend_from_slice(&val.to_le_bytes());
            }
        }
        let glb = if trigger.is_some() { (ch_en.count_ones() as u16) << 4 } else { 0xdead };
        for (h, val) in [(0x0701u16, glb), (0x0801, ch_en)] {
            b.extend_from_slice(&h.to_le_bytes());
            b.extend_from_slice(&val.to_le_bytes());
        }
        b.extend_from_slice(&0x40a0u16.to_le_bytes());
        let [mask, value, edge] = trigger.unwrap_or([0xffff, 0, 0]);
        for (first, rest) in [(mask, 0xffff), (mask, 0xffff), (value, 0), (value, 0), (edge, 0), (edge, 0), (2, 2), (2, 2)] {
            b.extend_from_slice(&first.to_le_bytes());
            for _ in 1..16 {
                b.extend_from_slice(&u16::to_le_bytes(rest));
            }
        }
        b.extend_from_slice(&[0; 64]);
        b.extend_from_slice(&0xfa5a_fa5au32.to_le_bytes());
        assert_eq!(b.len(), SETTING_LEN);
        b
    }

    fn flashed(profile: usize) -> Device {
        let mut dev = Device::new(Box::new(Dslogic::new(PROFILES[profile], true)));
        dev.control(SET_CONFIG, &[], &mut Violations::default());
        dev
    }

    /// `dslogic_fpga_firmware_upload()`, then the threshold register.
    fn open(dev: &mut Device, bitstream: usize, v: &mut Violations) {
        assert_eq!(dev.control(vendor(0xc0, 0xb0, 2), &[], v), Reply::Data(vec![1, 0]));
        assert_eq!(dev.control(vendor(0xc0, 0xb1, 1), &[], v), Reply::Data(vec![1]));
        assert_eq!(dev.control(vendor(0x40, 0xb3, 3), &[0, 0, 0], v), Reply::ack());
        let mut found = None;
        for p in PROFILES {
            if p.product == dev.descriptors().product_id() {
                found = Some(p.bitstreams[bitstream]);
            }
        }
        let (name, size) = found.unwrap();
        assert_eq!(dev.transfer_out(2, &ezusb::standin(name, size), v), Reply::ack());
        assert_eq!(dev.control(vendor(0x40, 0xb8, 2), &[71, 0x78], v), Reply::ack());
    }

    /// `dslogic_acquisition_start()` up to the trigger packet.
    fn acquire(dev: &mut Device, packet: &[u8], v: &mut Violations) -> Reply {
        dev.control(vendor(0x40, 0xb2, 3), &[0x80, 0, 0], v);
        dev.control(vendor(0x40, 0xb4, 3), &[180, 0, 0], v);
        dev.transfer_out(2, packet, v);
        dev.control(vendor(0x40, 0xb2, 3), &[0x30, 0, 0], v);
        dev.in_submitted(6, 512);
        dev.transfer_in(6, 512, v)
    }

    #[test]
    fn the_standin_sizes_are_those_of_the_firmware_list() {
        let list = include_str!("../../../tools/emu/dreamsourcelab-dslogic.firmware");
        let size_of = |name: &str| -> usize {
            let line = list
                .lines()
                .find(|l| l.split_whitespace().next() == Some(name))
                .unwrap_or_else(|| panic!("{name} is not in the firmware list"));
            line.split_whitespace().nth(1).unwrap().parse().unwrap()
        };
        for p in PROFILES {
            assert_eq!(size_of(p.firmware.0), p.firmware.1, "{}", p.firmware.0);
            for (name, size) in p.bitstreams {
                assert_eq!(size_of(name), *size, "{name}");
                assert!(*size < 1024 * 1024, "{name} fits FW_BUFSIZE: one bulk transfer");
            }
        }
    }

    #[test]
    fn uploads_renumerates_loads_the_fpga_and_streams() {
        let model = Dslogic::new(PROFILES[0], false);
        let mut dev = Device::new(Box::new(model));
        let mut v = Violations::default();
        assert_eq!(dev.descriptors().strings.len(), 0, "a bare FX2 has no strings");
        assert_eq!(dev.descriptors().product_id(), 0x0020);
        dev.control(SET_CONFIG, &[], &mut v);
        let (name, size) = PROFILES[0].firmware;
        let image = ezusb::standin(name, size);
        dev.control(Setup { request_type: 0x40, request: 0xa0, value: 0xe600, index: 0, length: 1 }, &[1], &mut v);
        for (i, c) in image.chunks(4096).enumerate() {
            let s = Setup { request_type: 0x40, request: 0xa0, value: (i * 4096) as u16, index: 0, length: c.len() as u16 };
            dev.control(s, c, &mut v);
        }
        dev.control(Setup { request_type: 0x40, request: 0xa0, value: 0xe600, index: 0, length: 1 }, &[0], &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert!(dev.take_renumeration());
        dev.reset();
        assert_eq!(dev.descriptors().strings, ["DreamSourceLab", "USB-based Instrument"]);
        dev.control(SET_CONFIG, &[], &mut v);
        open(&mut dev, 0, &mut v);
        assert!(v.is_empty(), "{v:?}");
        // 1 MHz, 3 channels, no trigger: real_pos 0.
        let Reply::Data(t) = acquire(&mut dev, &setting(100, 1000, 0b1011, None, 0), &mut v) else {
            panic!()
        };
        assert_eq!(t.len(), 512);
        assert_eq!(&t[4..8], &[0, 0, 0, 0]);
        assert_eq!(dev.transfer_in(6, 3 * 512, &mut v), Reply::Nak, "one per submission");
        // A batch submitted at once: the first completes, the rest wait
        // for a resubmission each.
        for _ in 0..3 {
            dev.in_submitted(6, 3 * 512);
        }
        let Reply::Data(d) = dev.transfer_in(6, 3 * 512, &mut v) else { panic!() };
        assert_eq!(dev.transfer_in(6, 3 * 512, &mut v), Reply::Nak, "the batch waits");
        dev.in_submitted(6, 3 * 512);
        assert!(matches!(dev.transfer_in(6, 3 * 512, &mut v), Reply::Data(_)));
        assert_eq!(d.len(), 3 * 512);
        // Atom 0: channel 0 toggles each sample, 1 every two, 3 every 8.
        assert_eq!(u64::from_le_bytes(d[0..8].try_into().unwrap()), 0xaaaa_aaaa_aaaa_aaaa);
        assert_eq!(u64::from_le_bytes(d[8..16].try_into().unwrap()), 0xcccc_cccc_cccc_cccc);
        assert_eq!(u64::from_le_bytes(d[16..24].try_into().unwrap()), 0xff00_ff00_ff00_ff00);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_trigger_keeps_trig_pos_samples_before_it() {
        let mut dev = flashed(0);
        let mut v = Violations::default();
        open(&mut dev, 0, &mut v);
        // Rising edge on channel 3 (mask clear, value and edge set): the
        // counter's bit 3 rises at 72, the first at or after 64.
        let rising = [!8u16, 8, 8];
        let Reply::Data(t) = acquire(&mut dev, &setting(100, 1000, 0xffff, Some(rising), 0), &mut v)
        else {
            panic!()
        };
        assert_eq!(&t[4..8], &64u32.to_le_bytes());
        dev.in_submitted(6, 16 * 512);
        let Reply::Data(d) = dev.transfer_in(6, 16 * 512, &mut v) else { panic!() };
        // The data starts at sample 72 - 64 = 8: channel 3 is high for
        // its first 8 samples.
        assert_eq!(u64::from_le_bytes(d[24..32].try_into().unwrap()) & 0xffff, 0x00ff);
        assert!(v.is_empty(), "{v:?}");
        // Channel 0 high; mask1 differs from mask0.
        let mut s = setting(100, 1000, 0xffff, Some([!1u16, 1, 0]), 0);
        s[68..70].copy_from_slice(&0u16.to_le_bytes());
        let Reply::Data(t) = acquire(&mut dev, &s, &mut v) else { panic!() };
        assert_eq!(&t[4..8], &64u32.to_le_bytes());
        assert_eq!(v.drain().len(), 1, "the copies differ");
    }

    #[test]
    fn flags_what_the_driver_never_sends() {
        let mut v = Violations::default();
        let mut dev = flashed(0);
        dev.control(vendor(0x40, 0xb4, 3), &[180, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "setting before the bitstream");
        dev.control(vendor(0x40, 0xb8, 2), &[71, 0x78], &mut v);
        assert_eq!(v.drain().len(), 2, "the setting due, and a register write before the bitstream");
        dev.control(vendor(0x40, 0xb3, 3), &[0, 0, 0], &mut v);
        let (name, size) = PROFILES[1].bitstreams[0];
        dev.transfer_out(2, &ezusb::standin(name, size), &mut v);
        assert_eq!(v.drain().len(), 1, "another profile's bitstream");
        open(&mut dev, 0, &mut v);
        assert!(v.is_empty(), "{v:?}");
        dev.control(vendor(0x40, 0xb8, 2), &[71, 0x70], &mut v);
        assert_eq!(v.drain().len(), 1, "another register");
        assert_eq!(dev.control(vendor(0xc0, 0xb6, 4), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "a request the driver never sends");
        assert_eq!(dev.control(vendor(0xc0, 0xb0, 4), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "a version of 4 bytes");
        dev.control(vendor(0x40, 0xb2, 3), &[0x30, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "start without a setting");
        dev.control(vendor(0x40, 0xb2, 3), &[0x70, 0, 1], &mut v);
        assert_eq!(v.drain().len(), 2, "other flags and a delay");
        dev.transfer_out(2, &[0; 16], &mut v);
        assert_eq!(v.drain().len(), 1, "bulk OUT without a command");
        dev.control(vendor(0x40, 0xb4, 3), &[100, 0, 0], &mut v);
        dev.transfer_out(2, &[0; 200], &mut v);
        assert_eq!(v.drain().len(), 2, "a setting of the wrong length");
        // The packet's fields.
        let check = |packet: Vec<u8>, n: usize, what: &str| {
            let mut v = Violations::default();
            let mut dev = flashed(1);
            open(&mut dev, 0, &mut v);
            dev.control(vendor(0x40, 0xb4, 3), &[180, 0, 0], &mut v);
            dev.transfer_out(2, &packet, &mut v);
            assert_eq!(v.drain().len(), n, "{what}: {v:?}");
        };
        check(setting(100, 1000, 3, None, 0), 0, "a plain one");
        check(setting(3, 1000, 3, None, 0), 1, "a divider of no rate");
        check(setting(2, 1000, 3, None, MODE_HALF), 1, "half mode, divider 2");
        check(setting(1, 1000, 3, None, MODE_HALF | MODE_QUAR), 1, "half and quarter");
        check(setting(1, 1000, 3, None, MODE_CLK_EDGE), 1, "an edge without the external clock");
        check(setting(1, 1000, 3, None, 1 << 9), 1, "instant mode");
        check(setting(1, 1000, 0, None, 0), 1, "no channel");
        check(setting(1, 1000, 3, None, MODE_RLE), 1, "RLE for a short capture");
        check(setting(1, 16_000_016, 3, None, 0), 1, "no RLE for a long one");
        check(setting(1, 16_000_016, 3, None, MODE_RLE), 0, "RLE for a long one");
        check(setting(1, 16_000_016, 3, None, MODE_RLE | MODE_STREAM), 1, "RLE in stream mode");
        let mut p = setting(100, 1000, 3, Some([!1, 1, 0]), 0);
        p[28] = 0x31;
        check(p, 1, "a global trigger word with two stages");
        let mut p = setting(100, 1000, 3, None, 0);
        p[22] = 65;
        check(p, 1, "a trigger position off the grid");
        let mut p = setting(100, 1000, 3, None, 0);
        p[23..26].copy_from_slice(&[0, 0x10, 0]); // 1 Mi samples on a Basic
        check(p, 1, "a trigger position beyond the memory");
        let mut p = setting(100, 1000, 3, None, 0);
        p[38] = 0; // stage 1's mask
        p[70] = 0;
        check(p, 1, "a second stage");
        let mut p = setting(100, 1000, 3, None, 0);
        p[356] = 0;
        check(p, 1, "the end sync");
        assert!(build(&[("profile".into(), "nope".into())]).is_err());
        assert!(build(&[("flashed".into(), "2".into())]).is_err());
    }

    #[test]
    fn faults_stall_requests_and_transfers() {
        let mut v = Violations::default();
        let faulty = |o: &[(&str, &str)]| {
            let o: Vec<(String, String)> = [("flashed", "1")]
                .iter()
                .chain(o)
                .map(|(k, v)| ((*k).to_owned(), (*v).to_owned()))
                .collect();
            let mut dev = Device::new(build(&o).unwrap());
            dev.control(SET_CONFIG, &[], &mut Violations::default());
            dev
        };
        let mut dev = faulty(&[("stall", "revid,stop"), ("version", "2.3")]);
        assert_eq!(dev.control(vendor(0xc0, 0xb0, 2), &[], &mut v), Reply::Data(vec![2, 3]));
        assert_eq!(dev.control(vendor(0xc0, 0xb1, 1), &[], &mut v), Reply::Stall);
        assert_eq!(dev.control(vendor(0x40, 0xb2, 3), &[0x80, 0, 0], &mut v), Reply::Stall);
        let mut dev = faulty(&[("stall_out", "fpga")]);
        dev.control(vendor(0x40, 0xb3, 3), &[0, 0, 0], &mut v);
        assert_eq!(dev.transfer_out(2, &[1; 64], &mut v), Reply::Stall);
        let mut dev = faulty(&[("short_out", "setting")]);
        open(&mut dev, 0, &mut v);
        dev.control(vendor(0x40, 0xb4, 3), &[180, 0, 0], &mut v);
        assert_eq!(dev.transfer_out(2, &setting(100, 64, 1, None, 0), &mut v), Reply::Data(vec![0; 180]));
        let mut dev = faulty(&[("empty", "1"), ("stall_in", "1"), ("trigger", "short")]);
        open(&mut dev, 0, &mut v);
        let Reply::Data(t) = acquire(&mut dev, &setting(100, 64, 1, None, 0), &mut v) else { panic!() };
        assert_eq!(t.len(), 24);
        let mut got = Vec::new();
        for _ in 0..3 {
            dev.in_submitted(6, 512);
            got.push(dev.transfer_in(6, 512, &mut v));
        }
        assert_eq!(got[..2], [Reply::Data(vec![]), Reply::Stall]);
        let Reply::Data(d) = &got[2] else { panic!() };
        assert_eq!(d[0], 0xaa, "the samples start at 0 after the faults");
        assert!(v.is_empty(), "a fault is the device's, not the host's: {v:?}");
        // The second sample transfer cut: 32 of its 64 atoms (1
        // channel), then 4 bytes; the third goes on at atom 64 + 33.
        let mut dev = faulty(&[("short_in", "2")]);
        open(&mut dev, 0, &mut v);
        acquire(&mut dev, &setting(100, 64, 1, None, 0), &mut v);
        let mut got = Vec::new();
        for _ in 0..3 {
            dev.in_submitted(6, 512);
            let Reply::Data(d) = dev.transfer_in(6, 512, &mut v) else { panic!() };
            got.push(d);
        }
        assert_eq!(got.iter().map(Vec::len).collect::<Vec<_>>(), [512, 32 * 8 + 4, 512]);
        assert_eq!(&got[2][..8], &[0xaa; 8], "atom 97 starts at an even sample");
        // A STALLed upload: the request to the bare chip that follows
        // is no violation.
        let o: Vec<(String, String)> = vec![("load".into(), "chunk".into())];
        let mut dev = Device::new(build(&o).unwrap());
        dev.control(SET_CONFIG, &[], &mut v);
        let load = |value, length| Setup { request_type: 0x40, request: 0xa0, value, index: 0, length };
        assert_eq!(dev.control(load(0xe600, 1), &[1], &mut v), Reply::ack());
        assert_eq!(dev.control(load(0, 4096), &[0; 4096], &mut v), Reply::Stall);
        assert_eq!(dev.control(vendor(0xc0, 0xb0, 2), &[], &mut v), Reply::Stall);
        for bad in [("stall", "reset"), ("stall_out", "x"), ("trigger", "late"), ("empty", "-1"), ("load", "x")] {
            assert!(build(&[(bad.0.into(), bad.1.into())]).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn the_log_shows_the_uninitialised_global_trigger_word_as_0() {
        let model = Dslogic::new(PROFILES[0], true);
        let mut p = setting(100, 1000, 3, None, 0);
        assert_eq!(&p[28..30], &[0xad, 0xde]);
        let shown = model.log_out(2, &p).unwrap();
        assert_eq!(&shown[28..30], &[0, 0]);
        assert_eq!(shown[..28], p[..28]);
        assert_eq!(shown[30..], p[30..]);
        p = setting(100, 1000, 3, Some([!1, 1, 0]), 0);
        assert_eq!(model.log_out(2, &p), None, "set with a trigger");
        assert_eq!(model.log_out(2, &[0; 360]), None, "not a setting packet");
    }

    #[test]
    fn the_first_dslogic_takes_either_bitstream() {
        let mut v = Violations::default();
        let mut dev = flashed(3);
        open(&mut dev, 1, &mut v);
        open(&mut dev, 0, &mut v);
        assert!(v.is_empty(), "{v:?}");
    }
}
