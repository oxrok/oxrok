// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Hantek DSO-2090 USB oscilloscope (and its siblings DSO-2150,
//! DSO-2250, DSO-5200 and DSO-5200A) as libsigrok's `hantek-dso` driver
//! sees it, built on the EZ-USB shell ([`crate::shells::ezusb`]).
//!
//! Where the facts come from. Everything protocol is from the C driver
//! (`api.c`, `protocol.c`, `protocol.h`); no Hantek firmware source and
//! no recording of the device is available here, so what the driver
//! does not say is the model's choice, named as such below.
//!
//! - Ids (`dev_profiles[]`): `04b4:PID` before the firmware upload,
//!   `04b5:PID` after it, PID `2090`, `2150`, `2250`, `5200`, `520a`;
//!   firmware `hantek-dso-<model>.fw`; buffer sizes 10240 and 32768 samples
//!   (2090, 2150), 524288 (2250, 5200A), 14336 (5200). The stand-in
//!   files' sizes are the model's (`tools/emu/hantek-dso.firmware`).
//! - The firmware's interface (`dso_getmps()`, `protocol.h`): one
//!   configuration, interface 0 whose alternate setting 0 has exactly
//!   two endpoints, bulk OUT 2 first and bulk IN 6 (`0x86`) second; the
//!   IN endpoint's packet size decides the transfer size. 512 bytes at
//!   high speed; `speed=full` makes the board enumerate at full speed,
//!   64 bytes, as on a USB 1.1 port. Strings and `bcdDevice` of the
//!   firmware are unknown and left out (none, 0x0000); the chip is taken
//!   to be an FX2LP and the boot EEPROM a `0xC0` one (both unverified).
//! - Control requests (`protocol.h`): `0xa2` reads the EEPROM (the
//!   driver: `wValue` 8, 72 bytes, the calibration), `0xb3` "begin
//!   command" (10 fixed bytes) precedes every bulk command, `0xb4` sets
//!   the offsets (17 bytes), `0xb5` the relays (17 bytes). `0xb2`
//!   (GETSPEED) is defined but never sent and is flagged.
//! - Bulk commands on endpoint 2, their layouts as `protocol.c` builds
//!   them: 0 filters, 1 trigger and sample rate (12 bytes: source,
//!   frame size, fast and slow timebase, channels, slope, trigger
//!   position), 2 force trigger, 3 capture start, 4 enable trigger,
//!   5 get channel data, 6 get capture state, 7 voltage; on the DSO-2250
//!   1 is replaced by 0x0b channels, 0x0c trigger source, 0x0d record
//!   length, 0x0e sample rate and 0x0f trigger position.
//! - The capture state answer (`dso_get_capturestate()`): byte 0 the
//!   state (0 empty, 1 filling, 2 ready, 3 ready on the 2250, 7 ready
//!   with 9 bit samples on the 5200, 127 timeout), bytes 1, 3, 2 the
//!   trigger offset, high to low, in the code the driver decodes ("each
//!   set bit inverts all bits with a lower value"): Gray code, which the
//!   model encodes as `n ^ (n >> 1)`. The answer is 512 bytes, the
//!   driver's buffer (the real length is unknown; the driver does not
//!   look at it).
//! - The frame (`receive_transfer()`, `send_chunk()`): `framesize`
//!   samples of two bytes, CH2 first and CH1 second, 0 to 255 across
//!   the 8 divisions of the channel's range; a disabled channel's byte
//!   holds a copy of the enabled one. The frame is the device's ring
//!   buffer: the driver takes the samples from the reported offset on
//!   first and those before it last.
//! - The EEPROM: 8 bytes of the FX2's `0xC0` boot record (the ids the
//!   board enumerates with before the upload), then at 8 the channel
//!   levels, 2 channels x 9 ranges x (low, high) big endian 16 bit
//!   words whose second byte is 0, as `get_channel_offsets()` says of
//!   the DSO-2090. The values are the model's.
//!
//! How the model behaves (its choices, not the driver's facts):
//!
//! - Signals: CH1 a triangle of +-1 V with a period of 400 samples, CH2
//!   a square of +-0.5 V with a period of 250 samples; EXT has nothing
//!   connected unless `ext=1`. They do not depend on the timebase, the sample rate,
//!   the coupling or the filters, which the model checks but does not
//!   apply. `ext=1` connects a signal to EXT, in step with CH1 (it
//!   fires at the triangle's minimum). A sample is the channel's voltage across its range (8 times
//!   the volts per division from the voltage command and the relays)
//!   moved by the vertical position the channel's offset sets within its
//!   EEPROM levels, 0 to 255, clipped.
//! - The capture: `CAPTURE_START` starts filling, which the state
//!   answers as 1 for `fill` polls (default 1); then the frame is ready
//!   (2, 3 on the 2250, 7 on the 5200s) once triggered, and the device
//!   reports 0, empty, while it waits. A force trigger triggers; with
//!   the trigger enabled (`ENABLE_TRIGGER` once) and CH1 or CH2 as the
//!   source, the trigger fires when the source's sample code crosses
//!   the trigger level's code (the trigger offset, 0 to 254) on the
//!   slope, within one period of the signal; EXT fires only with
//!   `ext=1`. So the
//!   driver's default, a trigger source of none (EXT) without a force
//!   on the first capture, sees three empty states and restarts with a
//!   force, as `handle_event()` does. A ready frame starts at the
//!   capture ratio's share of samples before the trigger (decoded from
//!   the trigger position), and its first sample sits at the ring
//!   position `wrap + 2500 * frame` (`wrap` default 1000), which is the
//!   trigger offset the state reports.
//! - Everything is paced by the host's polls and transfers; nothing by
//!   time. The channel data come as one transfer per bulk IN the host
//!   submits, the frame's bytes in order, the transfer size the
//!   endpoint's packet size. A known source of flakes, not of the
//!   model's making: `dso_get_channeldata()` submits all of a frame's
//!   transfers at once with a timeout of 40 ms each (protocol.c:925-927),
//!   so up to 320 USB/IP round trips (a full speed frame; 128 for the
//!   large 2090 frame) must finish within 40 ms of the submission; a
//!   transfer that times out completes empty, which leaves the frame
//!   unfinished and sigrok-cli waiting for ever (a case then times out).
//!   No run here has hit it.
//!
//! The pedantic checks: the upload byte for byte as `ezusb.c` sends it;
//! the begin command before every bulk command, and its 10 bytes; each
//! command's length, fixed bytes, reserved bits and fields (trigger
//! source, frame size the profile has, a timebase pair the driver's
//! tables give for that frame size, channels, trigger position in the
//! driver's range, voltage codes, filter bits; on the 2250 its own
//! commands and a sample rate of the driver's list); commands of the
//! other family; relay bytes each the value or its complement and a
//! range relay only with its lower one; offsets marked with 0x20 and
//! within the EEPROM levels of the channel's range (all zero before the
//! EEPROM was read, which the driver does when the trigger level is set
//! before an acquisition: it sends offsets from its zeroed calibration);
//! a capture before the configuration, a force trigger without a
//! capture, channel data before the frame is ready, and data transfers
//! other than the packet size; a firmware upload to the running
//! firmware and vendor requests the driver never sends.
//!
//! What the model taught about the C driver, see the cases'
//! comments (`tools/emu/hantek-dso.sh`): no error during a capture ends
//! it (a failed poll is polled again, an empty or missing data transfer
//! leaves the frame unfinished), so a vanished device or a DSO-5200's
//! state 7 keeps sigrok-cli running for ever; the 2250's voltage header
//! `0x08` shares bit 3 with CH2's range code (see `voltage()`: the model
//! reads CH2 at 5 V, 500 mV or 50 mV on a 2250 as 1 V, 100 mV or 10 mV,
//! so the driver's CH2 volts come out five times too large there); a
//! trigger offset of 0 makes the driver send an empty analog packet at
//! the frame's end, on which `output/analog.c` frees its buffer twice.
//!
//! Faults, all off by default: `stall=WHAT[@N],...` STALLs the request
//! WHAT (`load` the boot loader, `eeprom`, `begin`, `offset`, `relays`,
//! `out` a bulk command, `state` a capture state answer), every one or
//! only the Nth since the model was built (a one-shot STALL is an
//! emulation artifact: a real halt lasts until cleared);
//! `state=V@N,...` answers the Nth capture state poll with V instead;
//! `empty=N,...` completes the Nth channel data transfer since the
//! model was built with no data, the frame's bytes left for the next
//! (the driver, which submitted a transfer per packet of the frame,
//! then waits for a last one it never submits);
//! `endpoints=bad` puts the firmware's IN endpoint at 2, `swapped`
//! lists IN 6 before OUT 2, `one` leaves out IN 6 (firmwares the driver
//! does not know, which `dso_getmps()` refuses); `in-limit=N` NAKs every
//! IN after the Nth on the data endpoint, an emulation artifact so that an
//! interrupt lands on a known number of transfers (docs/USB.md).

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, VENDOR, Violations};
use crate::shells::ezusb::{self, Chip, Loader};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "hantek-dso";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Hantek DSO-2090 USB scope (EZ-USB, renumerates 04b4 -> 04b5); \
                         profile=dso-2090 (default), dso-2150, dso-2250, dso-5200, dso-5200a, \
                         flashed=1, speed=full, ext=1, fill=N, wrap=N; faults: \
                         stall=load|eeprom|begin|offset|relays|out|state[@N], state=V@N, empty=N, \
                         endpoints=bad|swapped|one, in-limit=N";

/// The command family a board speaks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    /// DSO-2090 and DSO-2150: command 1 sets trigger and timebase.
    Dso2090,
    /// DSO-2250: commands 0x0b to 0x0f instead of 1; ready is 3.
    Dso2250,
    /// DSO-5200 and 5200A: the 2090's commands; ready is 7 (9 bit).
    Dso5200,
}

/// A board of the driver's table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// The option value.
    pub key: &'static str,
    /// The product id, before and after the upload.
    pub product: u16,
    /// The firmware file the driver uploads.
    pub firmware: &'static str,
    /// Its size, as `tools/emu/hantek-dso.firmware` lists it.
    pub firmware_size: usize,
    /// The two frame sizes, in samples.
    pub buffers: [usize; 2],
    /// The command family.
    pub family: Family,
}

/// The vendor id before the upload.
pub const ORIG_VID: u16 = 0x04b4;
/// The vendor id the firmware enumerates with.
pub const FW_VID: u16 = 0x04b5;

/// The emulated boards, `dev_profiles[]` of `api.c`.
pub const PROFILES: [Profile; 5] = [
    Profile {
        key: "dso-2090",
        product: 0x2090,
        firmware: "hantek-dso-2090.fw",
        firmware_size: 5642,
        buffers: [10 * 1024, 32 * 1024],
        family: Family::Dso2090,
    },
    Profile {
        key: "dso-2150",
        product: 0x2150,
        firmware: "hantek-dso-2150.fw",
        firmware_size: 5714,
        buffers: [10 * 1024, 32 * 1024],
        family: Family::Dso2090,
    },
    Profile {
        key: "dso-2250",
        product: 0x2250,
        firmware: "hantek-dso-2250.fw",
        firmware_size: 6890,
        buffers: [10 * 1024, 512 * 1024],
        family: Family::Dso2250,
    },
    Profile {
        key: "dso-5200",
        product: 0x5200,
        firmware: "hantek-dso-5200.fw",
        firmware_size: 7024,
        buffers: [10 * 1024, 14 * 1024],
        family: Family::Dso5200,
    },
    Profile {
        key: "dso-5200a",
        product: 0x520a,
        firmware: "hantek-dso-5200A.fw",
        firmware_size: 7120,
        buffers: [10 * 1024, 512 * 1024],
        family: Family::Dso5200,
    },
];

const CTRL_READ_EEPROM: u8 = 0xa2;
const CTRL_GETSPEED: u8 = 0xb2;
const CTRL_BEGINCOMMAND: u8 = 0xb3;
const CTRL_SETOFFSET: u8 = 0xb4;
const CTRL_SETRELAYS: u8 = 0xb5;

/// `send_begin()`'s bytes.
pub const BEGIN: [u8; 10] = [0x0f, 0x03, 0x03, 0x03, 0x68, 0xac, 0xfe, 0x00, 0x01, 0x00];
/// The EEPROM address of the channel levels, `EEPROM_CHANNEL_OFFSETS`.
const EEPROM_CHANNEL_OFFSETS: usize = 0x08;
/// 2 channels x 9 ranges x 2 words.
const LEVELS_LEN: usize = 72;

const CMD_SET_FILTERS: u8 = 0x0;
const CMD_SET_TRIGGER_SAMPLERATE: u8 = 0x1;
const CMD_FORCE_TRIGGER: u8 = 0x2;
const CMD_CAPTURE_START: u8 = 0x3;
const CMD_ENABLE_TRIGGER: u8 = 0x4;
const CMD_GET_CHANNELDATA: u8 = 0x5;
const CMD_GET_CAPTURESTATE: u8 = 0x6;
const CMD_SET_VOLTAGE: u8 = 0x7;
const CMD_2250_SET_CHANNELS: u8 = 0xb;
const CMD_2250_SET_TRIGGERSOURCE: u8 = 0xc;
const CMD_2250_SET_RECORD_LENGTH: u8 = 0xd;
const CMD_2250_SET_SAMPLERATE: u8 = 0xe;
const CMD_2250_SET_TRIGGERPOS_AND_BUFFER: u8 = 0xf;

const CAPTURE_EMPTY: u8 = 0;
const CAPTURE_FILLING: u8 = 1;

/// The volts per division, in millivolts, `vdivs[]` of `api.c`.
const VDIV_MV: [u32; 9] = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000];

/// `timebase_small[]` and `timebase_large[]` of `protocol.c`.
const TIMEBASE_SMALL: [u16; 12] =
    [0xffff, 0xfffc, 0xfff7, 0xffe8, 0xffce, 0xff9c, 0xff07, 0xfe0d, 0xfc19, 0xf63d, 0xec79, 0xd8f1];
const TIMEBASE_LARGE: [u16; 12] =
    [0xffff, 0x0000, 0xfffc, 0xfff7, 0xffe8, 0xffce, 0xff9d, 0xff07, 0xfe0d, 0xfc19, 0xf63d, 0xec79];
/// The timebases of `api.c`, 10 us to 400 ms.
const NUM_TIMEBASES: usize = 15;

/// The sample rates of `api.c` the 2250 command can carry, and the
/// driver's default of 10 kHz (`DEFAULT_SAMPLERATE`, not in its list).
const SAMPLERATES: [u64; 15] = [
    10_000, 20_000, 25_000, 50_000, 100_000, 200_000, 250_000, 500_000, 1_000_000, 2_000_000,
    5_000_000, 10_000_000, 20_000_000, 25_000_000, 50_000_000,
];

/// The signals' periods, in samples.
const CH1_PERIOD: i64 = 400;
const CH2_PERIOD: i64 = 250;
/// Data longer than this is logged as a digest.
const DIGEST_ABOVE: usize = 128;
/// How far the ring position moves per frame.
const WRAP_STEP: usize = 2500;

/// The fixed (fast, slow) pair the driver sends for timebase `idx`,
/// or `None` where it refuses it (below 40 us with the large frame).
fn timebase_pair(idx: usize, large: bool) -> Option<(u8, u16)> {
    let fast = if large {
        match idx {
            0 | 1 => return None,
            2 => 0,
            3 => 2,
            4 => 3,
            _ => 4,
        }
    } else {
        match idx {
            0..=3 => idx as u8,
            _ => 4,
        }
    };
    let slow = if idx < 3 {
        0
    } else if large {
        TIMEBASE_LARGE[idx - 3]
    } else {
        TIMEBASE_SMALL[idx - 3]
    };
    Some((fast, slow))
}

/// The Gray code the device reports offsets in.
pub fn gray(n: u32) -> u32 {
    n ^ (n >> 1)
}

/// What a fault can STALL.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum What {
    Load,
    Eeprom,
    Begin,
    Offset,
    Relays,
    Out,
    State,
}

const WHATS: [(&str, What); 7] = [
    ("load", What::Load),
    ("eeprom", What::Eeprom),
    ("begin", What::Begin),
    ("offset", What::Offset),
    ("relays", What::Relays),
    ("out", What::Out),
    ("state", What::State),
];

/// Ways the board can misbehave, as a real one can; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    /// Requests that STALL: every one (`None`) or the Nth.
    stall: Vec<(What, Option<u32>)>,
    /// The Nth capture state answer reports this state instead.
    states: Vec<(u32, u8)>,
    /// The Nth channel data transfers complete empty.
    empty: Vec<u32>,
    /// The firmware's endpoints, when not the driver's.
    endpoints: Endpoints,
    /// NAK every IN on the data endpoint after this many.
    in_limit: Option<u32>,
}

/// The firmware's endpoint layout.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Endpoints {
    /// OUT 2, then IN 6.
    #[default]
    Good,
    /// OUT 2, then IN 2.
    Bad,
    /// IN 6, then OUT 2.
    Swapped,
    /// OUT 2 alone.
    One,
}

impl Faults {
    /// Takes the option `k=v` when it is a fault; false when `k` is no
    /// fault option.
    fn parse(&mut self, k: &str, v: &str) -> Result<bool> {
        let nth = |s: &str| {
            s.parse::<u32>().ok().filter(|n| *n > 0).ok_or_else(|| Error(format!("{k}={v}: N from 1")))
        };
        match k {
            "stall" => {
                self.stall.clear();
                for item in v.split(',').filter(|i| !i.is_empty()) {
                    let (name, n) = match item.split_once('@') {
                        Some((name, n)) => (name, Some(nth(n)?)),
                        None => (item, None),
                    };
                    let what = WHATS.iter().find(|(w, _)| *w == name).map(|(_, w)| *w).ok_or_else(
                        || {
                            Error(format!(
                                "stall={name}: load, eeprom, begin, offset, relays, out or state"
                            ))
                        },
                    )?;
                    self.stall.push((what, n));
                }
            }
            "state" => {
                self.states.clear();
                for item in v.split(',').filter(|i| !i.is_empty()) {
                    let (state, n) = item
                        .split_once('@')
                        .ok_or_else(|| Error(format!("state={item}: V@N")))?;
                    let state = state
                        .parse::<u8>()
                        .map_err(|_| Error(format!("state={item}: V from 0 to 255")))?;
                    self.states.push((nth(n)?, state));
                }
            }
            "empty" => {
                self.empty.clear();
                for item in v.split(',').filter(|i| !i.is_empty()) {
                    self.empty.push(nth(item)?);
                }
            }
            "in-limit" => {
                self.in_limit = Some(
                    v.parse::<u32>().map_err(|_| Error(format!("{k}={v}: a count")))?,
                );
            }
            "endpoints" => {
                self.endpoints = match v {
                    "good" => Endpoints::Good,
                    "bad" => Endpoints::Bad,
                    "swapped" => Endpoints::Swapped,
                    "one" => Endpoints::One,
                    _ => return Err(Error(format!("endpoints={v}: good, bad, swapped or one"))),
                }
            }
            _ => return Ok(false),
        }
        Ok(true)
    }
}

/// A board from the `-o` options: `profile=`, `flashed=0|1`,
/// `speed=high|full`, `ext=0|1`, `fill=N`, `wrap=N` and the faults (see
/// [`Faults`]).
///
/// # Errors
///
/// On an unknown option or value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut profile = PROFILES[0];
    let mut flashed = false;
    let mut full = false;
    let mut faults = Faults::default();
    let mut polls = 1;
    let mut wrap = 1000;
    let mut ext = false;
    for (k, v) in options {
        if faults.parse(k, v)? {
            continue;
        }
        let count = || v.parse::<u32>().map_err(|_| Error(format!("{k}={v}: a count")));
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
            "speed" => {
                full = match v.as_str() {
                    "high" => false,
                    "full" => true,
                    _ => return Err(Error(format!("speed={v}: high or full"))),
                }
            }
            "fill" => polls = count()?,
            "ext" => {
                ext = match v.as_str() {
                    "0" => false,
                    "1" => true,
                    _ => return Err(Error(format!("ext={v}: 0 or 1"))),
                }
            }
            "wrap" => wrap = count()? as usize,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    let mut model = HantekDso::new(profile, flashed, full, faults);
    model.fill = polls;
    model.wrap = wrap;
    model.ext = ext;
    Ok(Box::new(model))
}

/// The descriptors of the running firmware: vendor class, interface 0
/// with bulk OUT 2 and bulk IN 6 (2 with `bad`), no strings.
pub fn firmware_descriptors(p: &Profile, full: bool, endpoints: Endpoints) -> Descriptors {
    let size = if full { 64 } else { 512 };
    let out = descriptor::endpoint(0x02, crate::model::BULK, size, 0);
    let in_ep = |address| descriptor::endpoint(address, crate::model::BULK, size, 0);
    let eps: Vec<[u8; 7]> = match endpoints {
        Endpoints::Good => vec![out, in_ep(0x86)],
        Endpoints::Bad => vec![out, in_ep(0x82)],
        Endpoints::Swapped => vec![in_ep(0x86), out],
        Endpoints::One => vec![out],
    };
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, eps.len() as u8, [0xff, 0xff, 0xff], 0));
    for e in eps {
        body.extend_from_slice(&e);
    }
    let config = descriptor::configuration(1, 0, 0x80, 100, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0xff, 0xff, 0xff],
        max_packet0: 64,
        vendor: FW_VID,
        product: p.product,
        device: 0x0000,
        strings: [0, 0, 0],
        configurations: 1,
    };
    Descriptors {
        speed: if full { Speed::Full } else { Speed::High },
        device: dev.bytes(),
        configs: vec![config],
        strings: Vec::new(),
    }
}

/// The bare FX2's descriptors; at full speed its bulk endpoints have
/// 64 byte packets.
fn boot_descriptors(p: &Profile, full: bool) -> Descriptors {
    let mut d = ezusb::bare_descriptors(ORIG_VID, p.product, ezusb::FX2LP_DID);
    if full {
        d.speed = Speed::Full;
        for config in &mut d.configs {
            let mut at = 0;
            while at + 1 < config.len() {
                let len = usize::from(config[at]);
                if len == 0 {
                    break;
                }
                if config[at + 1] == descriptor::ENDPOINT && len >= 7 {
                    config[at + 4..at + 6].copy_from_slice(&64u16.to_le_bytes());
                }
                at += len;
            }
        }
    }
    d
}

/// The model's EEPROM: the `0xC0` boot record and the channel levels.
pub fn eeprom(p: &Profile) -> [u8; 256] {
    let mut e = [0xff; 256];
    let pid = p.product.to_le_bytes();
    let did = ezusb::FX2LP_DID.to_le_bytes();
    let vid = ORIG_VID.to_le_bytes();
    e[..8].copy_from_slice(&[0xc0, vid[0], vid[1], pid[0], pid[1], did[0], did[1], 0x00]);
    for c in 0..2 {
        for v in 0..9 {
            let (low, high) = levels_of(c, v);
            let at = EEPROM_CHANNEL_OFFSETS + (c * 9 + v) * 4;
            e[at..at + 2].copy_from_slice(&low.to_be_bytes());
            e[at + 2..at + 4].copy_from_slice(&high.to_be_bytes());
        }
    }
    e
}

/// The levels of channel `c` (0 is CH1) at range `v`: low and high
/// offset, second byte 0.
fn levels_of(c: usize, v: usize) -> (u16, u16) {
    let low = (0x01 + ((v + c) % 3) as u16) << 8;
    let high = (0x1e - ((v + c) % 2) as u16) << 8;
    (low, high)
}

/// Which channels the device samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Used {
    Ch1,
    Ch2,
    Both,
}

/// The trigger source.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Source {
    Ch1,
    Ch2,
    Ext,
}

/// What the host configured.
#[derive(Clone, Debug, Default)]
struct Settings {
    source: Option<Source>,
    slope_negative: bool,
    /// The frame size, 1 small or 2 large.
    size: Option<u8>,
    used: Option<Used>,
    /// The share of the frame before the trigger, 0 to 1.
    pre: Option<f64>,
    /// The voltage command's range codes, 0 to 2, per channel.
    vbits: Option<[u8; 2]>,
    /// The relays' range groups, 0 to 2, per channel.
    vgroup: Option<[u8; 2]>,
    /// The offsets, CH1, CH2 and the trigger level, as set.
    offsets: Option<[u16; 3]>,
    /// The 2250's commands seen, for its configuration.
    seen_2250: u8,
}

/// A running capture.
#[derive(Clone, Debug, Default)]
struct Capture {
    polls: u32,
    forced: bool,
}

/// What the next bulk IN carries.
#[derive(Clone, Debug, Default)]
enum Pending {
    #[default]
    Nothing,
    State,
    Data {
        bytes: Vec<u8>,
        at: usize,
    },
}

/// The emulated scope.
#[allow(clippy::struct_excessive_bools)]
pub struct HantekDso {
    profile: Profile,
    full: bool,
    boot: Descriptors,
    firmware: Descriptors,
    loader: Option<Box<Loader>>,
    renumerate: bool,
    faults: Faults,
    /// Requests seen per kind, for the faults' Nth.
    seen: Vec<(What, u32)>,
    state_polls: u32,
    /// Channel data transfers completed, for `empty=N`.
    data_transfers: u32,
    /// INs answered on the data endpoint, for `in-limit=N`.
    in_transfers: u32,
    eeprom: [u8; 256],
    eeprom_read: bool,
    begun: bool,
    settings: Settings,
    armed: bool,
    capture: Option<Capture>,
    pending: Pending,
    frames: usize,
    fill: u32,
    wrap: usize,
    ext: bool,
}

impl HantekDso {
    /// A board; `flashed` starts it with the firmware running, `full`
    /// on a full speed port.
    pub fn new(profile: Profile, flashed: bool, full: bool, faults: Faults) -> HantekDso {
        HantekDso {
            boot: boot_descriptors(&profile, full),
            firmware: firmware_descriptors(&profile, full, faults.endpoints),
            loader: (!flashed).then(|| Box::new(Loader::new(Chip::Fx2lp))),
            eeprom: eeprom(&profile),
            profile,
            full,
            renumerate: false,
            faults,
            seen: Vec::new(),
            state_polls: 0,
            data_transfers: 0,
            in_transfers: 0,
            eeprom_read: false,
            begun: false,
            settings: Settings::default(),
            armed: false,
            capture: None,
            pending: Pending::Nothing,
            frames: 0,
            fill: 1,
            wrap: 1000,
            ext: false,
        }
    }

    /// The firmware image both tools must upload.
    pub fn image(&self) -> Vec<u8> {
        ezusb::standin(self.profile.firmware, self.profile.firmware_size)
    }

    fn packet(&self) -> usize {
        if self.full { 64 } else { 512 }
    }

    /// Counts a request of kind `what`; whether a fault STALLs it.
    fn stalls(&mut self, what: What) -> bool {
        let n = if let Some((_, n)) = self.seen.iter_mut().find(|(w, _)| *w == what) {
            *n += 1;
            *n
        } else {
            self.seen.push((what, 1));
            1
        };
        self.faults.stall.iter().any(|(w, at)| *w == what && at.is_none_or(|a| a == n))
    }

    fn boot_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() == VENDOR && setup.request == ezusb::REQ_FIRMWARE_LOAD && self.stalls(What::Load)
        {
            return Reply::Stall;
        }
        let image = self.image();
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
            loader.check_ezusb_c(&image, v);
            self.loader = None;
            self.renumerate = true;
        }
        reply
    }

    fn firmware_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() == VENDOR && setup.request == ezusb::REQ_FIRMWARE_LOAD {
            v.report("firmware load to an FX2 whose Hantek firmware runs");
            return Reply::ack();
        }
        let (what, length) = match (setup.request_type, setup.request) {
            (0xc0, CTRL_READ_EEPROM) => (What::Eeprom, LEVELS_LEN),
            (0x40, CTRL_BEGINCOMMAND) => (What::Begin, BEGIN.len()),
            (0x40, CTRL_SETOFFSET) => (What::Offset, 17),
            (0x40, CTRL_SETRELAYS) => (What::Relays, 17),
            (_, CTRL_GETSPEED) => {
                v.report("CTRL_GETSPEED (0xb2), which the driver defines but never sends");
                return Reply::Stall;
            }
            _ => {
                v.report(format!(
                    "request {:#04x} {:#04x} value {:#06x}: not one the Hantek firmware answers",
                    setup.request_type, setup.request, setup.value
                ));
                return Reply::Stall;
            }
        };
        let value = if what == What::Eeprom { EEPROM_CHANNEL_OFFSETS as u16 } else { 0 };
        if setup.value != value || setup.index != 0 || usize::from(setup.length) != length {
            v.report(format!(
                "request {:#04x} with value {:#06x} index {:#06x} length {}; the driver sends \
                 {value:#06x}, 0 and {length}",
                setup.request, setup.value, setup.index, setup.length
            ));
        }
        if self.stalls(what) {
            return Reply::Stall;
        }
        match what {
            What::Eeprom => {
                let start = usize::from(setup.value);
                let end = (start + usize::from(setup.length)).min(self.eeprom.len());
                // The calibration is read once the levels were.
                self.eeprom_read |= start <= EEPROM_CHANNEL_OFFSETS
                    && end >= EEPROM_CHANNEL_OFFSETS + LEVELS_LEN;
                Reply::Data(self.eeprom[start.min(end)..end].to_vec())
            }
            What::Begin => {
                if data != BEGIN {
                    v.report(format!("begin command {data:02x?}; send_begin() sends {BEGIN:02x?}"));
                }
                if self.begun {
                    v.report("a second begin command before the bulk command");
                }
                self.begun = true;
                Reply::ack()
            }
            What::Relays => {
                self.relays(data, v);
                Reply::ack()
            }
            _ => {
                self.offsets(data, v);
                Reply::ack()
            }
        }
    }

    fn relays(&mut self, data: &[u8], v: &mut Violations) {
        const BASE: [u8; 8] = [0x00, 0x04, 0x08, 0x02, 0x20, 0x40, 0x10, 0x01];
        if data.len() != 17 {
            v.report(format!("relays of {} bytes; the driver sends 17", data.len()));
            return;
        }
        let mut on = [false; 8];
        for (i, b) in BASE.iter().enumerate() {
            if data[i] == *b {
                continue;
            }
            if i > 0 && data[i] == !*b {
                on[i] = true;
            } else {
                v.report(format!("relay byte {i} is {:#04x}: {b:#04x} or its complement", data[i]));
            }
        }
        if data[8..].iter().any(|b| *b != 0) {
            v.report(format!("relay bytes 8 to 16 are {:02x?}; the driver sends 0", &data[8..]));
        }
        let group = |r1: bool, r2: bool, ch: usize, v: &mut Violations| match (r1, r2) {
            (false, false) => 2,
            (true, false) => 1,
            (true, true) => 0,
            (false, true) => {
                v.report(format!("CH{ch} relays: the 10 mV range relay without the 100 mV one"));
                2
            }
        };
        let g1 = group(on[1], on[2], 1, v);
        let g2 = group(on[4], on[5], 2, v);
        self.settings.vgroup = Some([g1, g2]);
    }

    /// The range index of channel `c`, once both commands told it.
    fn range(&self, c: usize) -> Option<usize> {
        let bits = self.settings.vbits?;
        let group = self.settings.vgroup?;
        Some(usize::from(group[c]) * 3 + usize::from(bits[c]))
    }

    fn offsets(&mut self, data: &[u8], v: &mut Violations) {
        if data.len() != 17 {
            v.report(format!("offsets of {} bytes; the driver sends 17", data.len()));
            return;
        }
        let mut values = [0u16; 3];
        for (i, value) in values.iter_mut().enumerate() {
            let (hi, lo) = (data[2 * i], data[2 * i + 1]);
            if hi & 0xe0 != 0x20 {
                v.report(format!("offset {i} high byte {hi:#04x}: marked with 0x20, below 0x40"));
            }
            *value = u16::from_be_bytes([hi & 0x1f, lo]);
        }
        if data[6..].iter().any(|b| *b != 0) {
            v.report(format!("offset bytes 6 to 16 are {:02x?}; the driver sends 0", &data[6..]));
        }
        if values[2] > 0xfe {
            v.report(format!("trigger level {:#06x}; the driver's maximum is 0xfe", values[2]));
        }
        if self.eeprom_read {
            for (c, value) in values[..2].iter().enumerate() {
                let Some(r) = self.range(c) else {
                    v.report("offsets before the voltage and the relays");
                    break;
                };
                let (low, high) = levels_of(c, r);
                if *value < low || *value > high {
                    v.report(format!(
                        "CH{} offset {value:#06x} outside its EEPROM levels {low:#06x} to {high:#06x}",
                        c + 1
                    ));
                }
            }
        } else if values[..2] != [0, 0] {
            v.report(format!(
                "channel offsets {:#06x} {:#06x} before the EEPROM was read",
                values[0], values[1]
            ));
        }
        self.settings.offsets = Some(values);
    }

    /// A bulk command on endpoint 2.
    fn command(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        if !self.begun {
            v.report(format!("bulk command {data:02x?} without the begin command"));
        }
        self.begun = false;
        if self.stalls(What::Out) {
            return Reply::Stall;
        }
        let Some(&cmd) = data.first() else {
            v.report("an empty bulk command");
            return Reply::ack();
        };
        let family = self.profile.family;
        let length = match cmd {
            CMD_SET_FILTERS | CMD_SET_VOLTAGE => 8,
            CMD_SET_TRIGGER_SAMPLERATE if family != Family::Dso2250 => 12,
            CMD_FORCE_TRIGGER | CMD_CAPTURE_START | CMD_ENABLE_TRIGGER | CMD_GET_CHANNELDATA
            | CMD_GET_CAPTURESTATE => 2,
            CMD_2250_SET_CHANNELS | CMD_2250_SET_RECORD_LENGTH if family == Family::Dso2250 => 4,
            CMD_2250_SET_TRIGGERSOURCE | CMD_2250_SET_SAMPLERATE if family == Family::Dso2250 => 8,
            CMD_2250_SET_TRIGGERPOS_AND_BUFFER if family == Family::Dso2250 => 10,
            _ => {
                v.report(format!("bulk command {cmd:#04x}: not one the {} takes", self.profile.key));
                return Reply::ack();
            }
        };
        if data.len() != length {
            v.report(format!(
                "bulk command {cmd:#04x} of {} bytes; the driver sends {length}",
                data.len()
            ));
            return Reply::ack();
        }
        match cmd {
            CMD_SET_FILTERS => {
                if data[1] != 0x0f || data[2] & !0xc0 != 0 || data[3..].iter().any(|b| *b != 0) {
                    v.report(format!(
                        "filters {data:02x?}: 00 0f, bits 7 (CH1) and 6 (CH2), then zeros"
                    ));
                }
            }
            CMD_SET_VOLTAGE => self.voltage(data, v),
            CMD_SET_TRIGGER_SAMPLERATE => self.trigger_samplerate(data, v),
            CMD_FORCE_TRIGGER | CMD_CAPTURE_START | CMD_ENABLE_TRIGGER | CMD_GET_CHANNELDATA
            | CMD_GET_CAPTURESTATE
                if data[1] != 0 =>
            {
                v.report(format!("bulk command {cmd:#04x} with {:#04x}; the driver sends 0", data[1]));
            }
            CMD_FORCE_TRIGGER => match &mut self.capture {
                Some(c) => c.forced = true,
                None => v.report("force trigger without a capture"),
            },
            CMD_CAPTURE_START => self.capture_start(v),
            CMD_ENABLE_TRIGGER => self.armed = true,
            CMD_GET_CHANNELDATA => self.channel_data(v),
            CMD_GET_CAPTURESTATE => {
                if matches!(self.pending, Pending::Data { .. }) {
                    v.report("capture state asked before the frame was read");
                }
                self.pending = Pending::State;
            }
            _ => self.command_2250(data, v),
        }
        Reply::ack()
    }

    fn voltage(&mut self, data: &[u8], v: &mut Violations) {
        let is_2250 = self.profile.family == Family::Dso2250;
        let (h1, h2) = if is_2250 { (0x00, 0x08) } else { (0x0f, 0x30) };
        // The 2250's header 0x08 is bit 3, which is also CH2's high range
        // bit in the 2090 layout the driver ORs in (protocol.c:580-626),
        // probably a driver bug: CH2's code 2 (5 V, 500 mV, 50 mV) cannot
        // be told from 0 (1 V, 100 mV, 10 mV). The model takes bit 3 as
        // the header and CH2's code from bit 2 alone, so it reads CH2's
        // code 2 as 0: what a 2250 that decodes the header so would set.
        let bits = if is_2250 {
            [data[2] & 3, (data[2] >> 2) & 1]
        } else {
            [data[2] & 3, (data[2] >> 2) & 3]
        };
        if data[1] != h1 || data[2] & (0xf0 | h2) != h2 || data[3..].iter().any(|b| *b != 0) {
            v.report(format!("voltage {data:02x?}: 07 {h1:02x} {h2:02x}|ranges, then zeros"));
        }
        if bits.iter().any(|b| *b > 2) {
            v.report(format!("voltage range codes {bits:?}: 0 to 2"));
        }
        self.settings.vbits = Some(bits.map(|b| b.min(2)));
    }

    fn trigger_samplerate(&mut self, data: &[u8], v: &mut Violations) {
        let s = &mut self.settings;
        let b2 = data[2];
        s.source = match b2 & 3 {
            0 => Some(Source::Ch2),
            1 => Some(Source::Ch1),
            2 => Some(Source::Ext),
            _ => {
                v.report("trigger source 3: 0 (CH2), 1 (CH1) or 2 (EXT)");
                None
            }
        };
        let size = (b2 >> 2) & 3;
        if size != 1 && size != 2 {
            v.report(format!("frame size code {size}: 1 (small) or 2 (large)"));
        }
        if b2 & 0x10 != 0 {
            v.report("trigger and sample rate byte 2 bit 4, which is unused");
        }
        let fast = b2 >> 5;
        let slow = u16::from_le_bytes([data[4], data[5]]);
        let large = size == 2;
        if !(0..NUM_TIMEBASES).any(|i| timebase_pair(i, large) == Some((fast, slow))) {
            v.report(format!(
                "timebase fast {fast} slow {slow:#06x}: no timebase of the driver's for the {} frame",
                if large { "large" } else { "small" }
            ));
        }
        let b3 = data[3];
        s.used = match b3 & 3 {
            0 => Some(Used::Ch1),
            1 => Some(Used::Ch2),
            2 => Some(Used::Both),
            _ => {
                v.report("channels code 3: 0 (CH1), 1 (CH2) or 2 (both)");
                None
            }
        };
        s.slope_negative = b3 & 0x08 != 0;
        if b3 & 0x04 != 0 {
            // The fast rate channel, set only below TIME_10us, the first
            // timebase (protocol.c:481-482).
            v.report("trigger and sample rate byte 3 bit 2 (fast rates), which the driver never sets");
        }
        if b3 & 0xf0 != 0 {
            v.report(format!("trigger and sample rate byte 3 {b3:#04x}: bits 4 to 7 unused"));
        }
        let pos = u32::from(data[6]) | u32::from(data[7]) << 8 | u32::from(data[10]) << 16;
        if !(0x77fff..=0x7ffff).contains(&pos) {
            v.report(format!("trigger position {pos:#x}: 0x77fff to 0x7ffff"));
        }
        s.pre = Some(f64::from(pos.clamp(0x77fff, 0x7ffff) - 0x77fff) / f64::from(0x8000));
        if data[8] != 0 || data[9] != 0 || data[11] != 0 {
            v.report(format!("trigger and sample rate bytes 8, 9, 11 {data:02x?}: 0"));
        }
        s.size = Some(size.clamp(1, 2));
    }

    fn command_2250(&mut self, data: &[u8], v: &mut Violations) {
        let s = &mut self.settings;
        if data[1] != 0 {
            v.report(format!("2250 command {:#04x} byte 1 {:#04x}: 0", data[0], data[1]));
        }
        match data[0] {
            CMD_2250_SET_TRIGGERSOURCE => {
                s.source = match data[2] & 7 {
                    0 => Some(Source::Ext),
                    2 => Some(Source::Ch1),
                    3 => Some(Source::Ch2),
                    other => {
                        v.report(format!("2250 trigger source {other}: 0 (EXT), 2 (CH1), 3 (CH2)"));
                        None
                    }
                };
                s.slope_negative = data[2] & 8 != 0;
                if data[2] & 0xf0 != 0 || data[3..].iter().any(|b| *b != 0) {
                    v.report(format!("2250 trigger source {data:02x?}: one byte, then zeros"));
                }
                s.seen_2250 |= 1;
            }
            CMD_2250_SET_RECORD_LENGTH => {
                if !(1..=2).contains(&data[2]) || data[3] != 0 {
                    v.report(format!("2250 record length {data:02x?}: 1 or 2"));
                }
                s.size = Some(data[2].clamp(1, 2));
                s.seen_2250 |= 2;
            }
            CMD_2250_SET_SAMPLERATE => {
                let rate = match data[2] {
                    0 if data[4] == 0 && data[5] == 0 => Some(100_000_000),
                    2 => {
                        let div = u64::from(!u16::from_le_bytes([data[4], data[5]])) + 2;
                        (100_000_000 % div == 0).then_some(100_000_000 / div)
                    }
                    _ => None,
                };
                if rate.is_none_or(|r| r != 100_000_000 && !SAMPLERATES.contains(&r))
                    || data[3] != 0
                    || data[6..].iter().any(|b| *b != 0)
                {
                    v.report(format!("2250 sample rate {data:02x?}: no rate of the driver's"));
                }
                s.seen_2250 |= 4;
            }
            CMD_2250_SET_CHANNELS => {
                // The driver's (ch1 ? 0 : 1) + (ch2 ? 2 : 0): 3 is CH2
                // alone, 1 none, which sigrok cannot ask for.
                s.used = match data[2] {
                    0 => Some(Used::Ch1),
                    2 => Some(Used::Both),
                    3 => Some(Used::Ch2),
                    other => {
                        v.report(format!("2250 channels {other}: 0 (CH1), 2 (both) or 3 (CH2)"));
                        None
                    }
                };
                if data[3] != 0 {
                    v.report(format!("2250 channels {data:02x?}: byte 3 is 0"));
                }
                s.seen_2250 |= 8;
            }
            _ => {
                let large = s.size == Some(2);
                let min = if large { 0 } else { 0x7d7ff };
                let max = 0x7ffff_u32;
                let p = |at: usize| {
                    u32::from(data[at]) | u32::from(data[at + 1]) << 8 | u32::from(data[at + 2]) << 16
                };
                let (p1, p2) = (p(2), p(6));
                let diff = max - min;
                let sum = p1.wrapping_sub(min).wrapping_add(p2.wrapping_sub(min));
                if !(min..=max).contains(&p1)
                    || !(min..=max).contains(&p2)
                    || data[5] != 0
                    || data[9] != 0
                    || sum.abs_diff(diff) > 1
                {
                    v.report(format!(
                        "2250 trigger position {data:02x?}: two positions in {min:#x} to \
                         {max:#x} that add up to the {} frame",
                        if large { "large" } else { "small" }
                    ));
                }
                s.pre = Some(f64::from(p1.clamp(min, max) - min) / f64::from(diff));
                s.seen_2250 |= 16;
            }
        }
    }

    fn configured(&self) -> bool {
        let s = &self.settings;
        let family = if self.profile.family == Family::Dso2250 { s.seen_2250 == 31 } else { true };
        family
            && s.size.is_some()
            && s.used.is_some()
            && s.pre.is_some()
            && s.vbits.is_some()
            && s.vgroup.is_some()
            && s.offsets.is_some()
            && self.eeprom_read
    }

    fn capture_start(&mut self, v: &mut Violations) {
        if !self.configured() {
            v.report(
                "capture start before the trigger, voltage, relays and offsets (after the \
                 EEPROM read) were set",
            );
        }
        if matches!(self.pending, Pending::Data { .. }) {
            v.report("capture start before the frame was read");
        }
        self.pending = Pending::Nothing;
        self.capture = Some(Capture::default());
    }

    fn ready_state(&self) -> u8 {
        match self.profile.family {
            Family::Dso2090 => 2,
            Family::Dso2250 => 3,
            Family::Dso5200 => 7,
        }
    }

    fn frame_len(&self) -> usize {
        self.profile.buffers[usize::from(self.settings.size.unwrap_or(1)) - 1]
    }

    /// The ring position of the frame's first sample.
    fn ring(&self) -> usize {
        (self.wrap + WRAP_STEP * self.frames) % self.frame_len()
    }

    /// The voltage of channel `c` at time `t`, in millivolts.
    fn signal(c: usize, t: i64) -> i64 {
        if c == 0 {
            let u = t.rem_euclid(CH1_PERIOD);
            if u < CH1_PERIOD / 2 { -1000 + 10 * u } else { 3000 - 10 * u }
        } else if t.rem_euclid(CH2_PERIOD) < CH2_PERIOD / 2 {
            500
        } else {
            -500
        }
    }

    /// Channel `c`'s vertical position, 0 to 1, from its offset.
    fn position(&self, c: usize) -> f64 {
        match (self.settings.offsets, self.range(c)) {
            (Some(o), Some(r)) if self.eeprom_read => {
                let (low, high) = levels_of(c, r);
                (f64::from(o[c]) - f64::from(low)) / (f64::from(high) - f64::from(low))
            }
            _ => 0.5,
        }
    }

    /// Channel `c`'s sample code at time `t`.
    fn code(&self, c: usize, t: i64) -> u8 {
        let range = f64::from(VDIV_MV[self.range(c).unwrap_or(5)] * 8);
        let x = (Self::signal(c, t) as f64 / range + self.position(c)) * 255.0;
        x.round().clamp(0.0, 255.0) as u8
    }

    /// The first time within a period of the source's signal at which
    /// its code crosses the trigger level on the slope.
    fn trigger_time(&self) -> Option<i64> {
        let (c, period) = match self.settings.source? {
            Source::Ch1 => (0, CH1_PERIOD),
            Source::Ch2 => (1, CH2_PERIOD),
            // A signal on EXT is in step with CH1: it fires at the
            // triangle's minimum.
            Source::Ext => return self.ext.then_some(0),
        };
        let level = self.settings.offsets.map_or(0x7f, |o| o[2]);
        let level = i64::from(level);
        (0..period).find(|t| {
            let before = i64::from(self.code(c, t - 1));
            let now = i64::from(self.code(c, *t));
            if self.settings.slope_negative {
                before > level && now <= level
            } else {
                before < level && now >= level
            }
        })
    }

    fn triggered(&self) -> bool {
        let Some(c) = &self.capture else { return false };
        c.forced || (self.armed && self.trigger_time().is_some())
    }

    /// The answer to a capture state poll.
    fn state(&mut self) -> Vec<u8> {
        let ready = self.ready_state();
        let fill = self.fill;
        let triggered = self.triggered();
        let mut state = match &mut self.capture {
            None => CAPTURE_EMPTY,
            Some(c) => {
                c.polls += 1;
                if c.polls <= fill {
                    CAPTURE_FILLING
                } else if triggered {
                    ready
                } else {
                    CAPTURE_EMPTY
                }
            }
        };
        if let Some((_, s)) = self.faults.states.iter().find(|(n, _)| *n == self.state_polls) {
            state = *s;
        }
        let ring = if self.capture.is_some() { gray(self.ring() as u32) } else { 0 };
        let mut answer = vec![0; 512];
        answer[0] = state;
        answer[1] = (ring >> 16) as u8;
        answer[2] = ring as u8;
        answer[3] = (ring >> 8) as u8;
        answer
    }

    fn channel_data(&mut self, v: &mut Violations) {
        let ready = self.capture.as_ref().is_some_and(|c| c.polls > self.fill) && self.triggered();
        if !ready {
            v.report("channel data asked before the capture state said ready");
            return;
        }
        let n = self.frame_len();
        let pre = (self.settings.pre.unwrap_or(1.0) * n as f64) as i64;
        let start = match self.trigger_time() {
            Some(t) if !self.capture.as_ref().is_some_and(|c| c.forced) => t - pre,
            _ => (1000 * self.frames) as i64,
        };
        let ring = self.ring();
        let used = self.settings.used.unwrap_or(Used::Both);
        let mut bytes = vec![0; 2 * n];
        for i in 0..n {
            let t = start + i as i64;
            let (ch1, ch2) = (self.code(0, t), self.code(1, t));
            let (ch1, ch2) = match used {
                Used::Both => (ch1, ch2),
                Used::Ch1 => (ch1, ch1),
                Used::Ch2 => (ch2, ch2),
            };
            let at = 2 * ((ring + i) % n);
            bytes[at] = ch2;
            bytes[at + 1] = ch1;
        }
        self.pending = Pending::Data { bytes, at: 0 };
        self.capture = None;
        self.frames += 1;
    }
}

impl DeviceModel for HantekDso {
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
            v.report(format!("OUT transfer to endpoint {ep}; the commands go to 2"));
            return Reply::Stall;
        }
        self.command(data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        let in_ep = if self.faults.endpoints == Endpoints::Bad { 2 } else { 6 };
        if ep != in_ep {
            v.report(format!("IN transfer from endpoint {ep}; the answers come from {in_ep}"));
            return Reply::Stall;
        }
        if self.faults.in_limit.is_some_and(|n| self.in_transfers >= n) {
            return Reply::Nak;
        }
        self.in_transfers += 1;
        match &mut self.pending {
            Pending::Nothing => Reply::Nak,
            Pending::State => {
                self.pending = Pending::Nothing;
                self.state_polls += 1;
                if max != 512 {
                    v.report(format!("capture state read of {max} bytes; the driver reads 512"));
                }
                if self.stalls(What::State) {
                    return Reply::Stall;
                }
                Reply::Data(self.state())
            }
            Pending::Data { bytes, at } => {
                self.data_transfers += 1;
                if self.faults.empty.contains(&self.data_transfers) {
                    // A transfer the firmware ends with a zero length
                    // packet: the frame does not move on.
                    return Reply::Data(Vec::new());
                }
                let take = max.min(bytes.len() - *at);
                let chunk = bytes[*at..*at + take].to_vec();
                *at += take;
                if *at == bytes.len() {
                    self.pending = Pending::Nothing;
                }
                let packet = self.packet();
                if max != packet {
                    v.report(format!(
                        "channel data transfer of {max} bytes; the driver asks for the packet \
                         size, {packet}"
                    ));
                }
                Reply::Data(chunk)
            }
        }
    }

    fn reset(&mut self) {
        // A new attach ends a capture; the firmware stays in RAM.
        self.capture = None;
        self.pending = Pending::Nothing;
        self.begun = false;
    }

    /// Firmware chunks, capture state answers and channel data, which
    /// would make a frame's log some 40 KiB of hex; the EEPROM read (72
    /// bytes), the relays and offsets (17) and the commands stay in hex.
    fn log_digest_above(&self) -> Option<usize> {
        Some(DIGEST_ABOVE)
    }

    fn take_renumeration(&mut self) -> bool {
        std::mem::take(&mut self.renumerate)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Device;

    fn vendor(request_type: u8, request: u8, value: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index: 0, length }
    }

    const SET_CONFIG: Setup = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };

    fn upload(dev: &mut Device, image: &[u8], v: &mut Violations) {
        assert_eq!(dev.control(SET_CONFIG, &[], v), Reply::ack());
        assert_eq!(dev.control(vendor(0x40, 0xa0, 0xe600, 1), &[1], v), Reply::ack());
        for (i, c) in image.chunks(4096).enumerate() {
            let s = vendor(0x40, 0xa0, (i * 4096) as u16, c.len() as u16);
            assert_eq!(dev.control(s, c, v), Reply::ack());
        }
        assert_eq!(dev.control(vendor(0x40, 0xa0, 0xe600, 1), &[0], v), Reply::ack());
    }

    /// The driver's `send_bulkcmd()`.
    fn bulk(dev: &mut Device, cmd: &[u8], v: &mut Violations) -> Reply {
        let r = dev.control(vendor(0x40, 0xb3, 0, 10), &BEGIN, v);
        if r != Reply::ack() {
            return r;
        }
        dev.transfer_out(2, cmd, v)
    }

    /// The trigger and sample rate command of the driver's defaults:
    /// no trigger source (EXT), small frame, 100 us, both channels,
    /// capture ratio 100.
    fn trigger_default() -> [u8; 12] {
        let pos: u32 = 0x77fff + 0x8000;
        let p = pos.to_le_bytes();
        [1, 0, 2 | 1 << 2 | 3 << 5, 2, 0xff, 0xff, p[0], p[1], 0, 0, p[2], 0]
    }

    /// `dso_init()` with the defaults (500 mV, DC, offsets at 0.5).
    fn init(dev: &mut Device, v: &mut Violations) {
        let levels = dev.control(vendor(0xc0, 0xa2, 8, 72), &[], v);
        let Reply::Data(levels) = levels else { panic!("{levels:?}") };
        assert_eq!(levels.len(), 72);
        assert_eq!(bulk(dev, &trigger_default(), v), Reply::ack());
        assert_eq!(bulk(dev, &[0, 0x0f, 0, 0, 0, 0, 0, 0], v), Reply::ack());
        assert_eq!(bulk(dev, &[7, 0x0f, 0x30 | 2 | 2 << 2, 0, 0, 0, 0, 0], v), Reply::ack());
        let mut relays = [0u8; 17];
        relays[..8].copy_from_slice(&[0x00, 0xfb, 0x08, 0xfd, 0xdf, 0x40, 0xef, 0x01]);
        assert_eq!(dev.control(vendor(0x40, 0xb5, 0, 17), &relays, v), Reply::ack());
        // 500 mV is range 5: the midpoint of its levels.
        let mut offsets = [0u8; 17];
        for c in 0..2 {
            let at = 8 + (c * 9 + 5) * 4 - 8;
            let low = u16::from_be_bytes([levels[at], levels[at + 1]]);
            let high = u16::from_be_bytes([levels[at + 2], levels[at + 3]]);
            let mid = (high - low) / 2 + low;
            offsets[2 * c] = (mid >> 8) as u8 | 0x20;
            offsets[2 * c + 1] = mid as u8;
        }
        offsets[4] = 0x20;
        offsets[5] = 0x7f;
        assert_eq!(dev.control(vendor(0x40, 0xb4, 0, 17), &offsets, v), Reply::ack());
        assert_eq!(bulk(dev, &[4, 0], v), Reply::ack());
    }

    fn poll(dev: &mut Device, v: &mut Violations) -> (u8, u32) {
        assert_eq!(bulk(dev, &[6, 0], v), Reply::ack());
        let Reply::Data(a) = dev.transfer_in(6, 512, v) else { panic!() };
        let mut toff = u32::from(a[1]) << 16 | u32::from(a[3]) << 8 | u32::from(a[2]);
        // dso_get_capturestate()'s decoding.
        let mut bit = 1u32;
        for _ in 0..24 {
            if toff & bit != 0 {
                toff ^= bit - 1;
            }
            bit <<= 1;
        }
        (a[0], toff)
    }

    fn running(options: &[(&str, &str)]) -> Device {
        let mut o: Vec<(String, String)> = vec![("flashed".into(), "1".into())];
        o.extend(options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())));
        let mut dev = Device::new(build(&o).unwrap());
        dev.control(SET_CONFIG, &[], &mut Violations::default());
        dev
    }

    #[test]
    fn the_standin_sizes_are_those_of_the_firmware_list() {
        let list = include_str!("../../../tools/emu/hantek-dso.firmware");
        for p in PROFILES {
            let line = list
                .lines()
                .find(|l| l.split_whitespace().next() == Some(p.firmware))
                .unwrap_or_else(|| panic!("{} is not in the firmware list", p.firmware));
            let size: usize = line.split_whitespace().nth(1).unwrap().parse().unwrap();
            assert_eq!(size, p.firmware_size, "{}", p.firmware);
        }
    }

    #[test]
    fn gray_codes_are_what_the_driver_decodes() {
        for n in [0u32, 1, 2, 3, 1000, 10239, 0x7ffff, 0x00ab_cdef] {
            let mut toff = gray(n);
            let mut bit = 1u32;
            for _ in 0..24 {
                if toff & bit != 0 {
                    toff ^= bit - 1;
                }
                bit <<= 1;
            }
            assert_eq!(toff, n);
        }
    }

    #[test]
    fn uploads_renumerates_and_captures_as_the_driver_drives_it() {
        let model = HantekDso::new(PROFILES[0], false, false, Faults::default());
        let image = model.image();
        let mut dev = Device::new(Box::new(model));
        let mut v = Violations::default();
        assert_eq!(dev.descriptors().vendor_id(), 0x04b4);
        upload(&mut dev, &image, &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert!(dev.take_renumeration());
        dev.reset();
        assert_eq!(dev.descriptors().vendor_id(), 0x04b5);
        assert_eq!(dev.descriptors().product_id(), 0x2090);
        let eps = Descriptors::endpoints(&dev.descriptors().configs[0]);
        assert_eq!((eps[0].address, eps[1].address, eps[1].max_packet), (0x02, 0x86, 512));
        dev.control(SET_CONFIG, &[], &mut v);
        init(&mut dev, &mut v);
        assert_eq!(bulk(&mut dev, &[3, 0], &mut v), Reply::ack());
        // Filling, then waiting for the EXT trigger: three empty states.
        assert_eq!(poll(&mut dev, &mut v).0, 1);
        for _ in 0..3 {
            assert_eq!(poll(&mut dev, &mut v).0, 0);
        }
        // handle_event() restarts with a force.
        bulk(&mut dev, &[3, 0], &mut v);
        bulk(&mut dev, &[4, 0], &mut v);
        bulk(&mut dev, &[2, 0], &mut v);
        assert_eq!(poll(&mut dev, &mut v).0, 1);
        assert_eq!(poll(&mut dev, &mut v), (2, 1000));
        bulk(&mut dev, &[5, 0], &mut v);
        let mut frame = Vec::new();
        for _ in 0..40 {
            let Reply::Data(d) = dev.transfer_in(6, 512, &mut v) else { panic!() };
            assert_eq!(d.len(), 512);
            frame.extend(d);
        }
        assert_eq!(dev.transfer_in(6, 512, &mut v), Reply::Nak, "the frame is over");
        assert!(v.is_empty(), "{v:?}");
        // From the ring position on: CH1 at -1 V is 0x40 at 500 mV/div
        // (range 4 V), CH2 at +0.5 V is 0x9f.
        assert_eq!(&frame[2000..2002], &[0x9f, 0x40]);
    }

    #[test]
    fn a_trigger_on_ch1_puts_the_crossing_at_the_capture_ratio() {
        let mut v = Violations::default();
        let mut dev = running(&[("wrap", "0")]);
        init(&mut dev, &mut v);
        // CH1, rising, capture ratio 50.
        let mut cmd = trigger_default();
        let pos: u32 = 0x77fff + 0x8000 * 50 / 100;
        cmd[2] = 1 | 1 << 2 | 3 << 5;
        cmd[6..8].copy_from_slice(&pos.to_le_bytes()[..2]);
        cmd[10] = (pos >> 16) as u8;
        bulk(&mut dev, &cmd, &mut v);
        bulk(&mut dev, &[3, 0], &mut v);
        assert_eq!(poll(&mut dev, &mut v).0, 1);
        assert_eq!(poll(&mut dev, &mut v), (2, 0));
        bulk(&mut dev, &[5, 0], &mut v);
        let mut frame = Vec::new();
        while let Reply::Data(d) = dev.transfer_in(6, 512, &mut v) {
            frame.extend(d);
        }
        let ch1: Vec<u8> = frame.chunks(2).map(|s| s[1]).collect();
        assert!(ch1[5119] < 0x7f && ch1[5120] >= 0x7f, "{:?}", &ch1[5115..5125]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_what_the_driver_and_the_firmware_forbid() {
        let mut v = Violations::default();
        let model = HantekDso::new(PROFILES[0], false, false, Faults::default());
        let mut image = model.image();
        let mut dev = Device::new(Box::new(model));
        assert_eq!(dev.control(vendor(0xc0, 0xa2, 8, 72), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "a request to the boot loader");
        image[10] ^= 1;
        upload(&mut dev, &image, &mut v);
        assert_eq!(v.drain().len(), 1, "the image differs from the stand-in");
        dev.reset();
        dev.control(SET_CONFIG, &[], &mut v);
        dev.control(vendor(0x40, 0xa0, 0xe600, 1), &[1], &mut v);
        assert_eq!(v.drain().len(), 1, "upload to the running firmware");
        assert_eq!(dev.control(vendor(0xc0, 0xb2, 0, 1), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "GETSPEED");
        dev.control(vendor(0xc0, 0xa2, 0, 72), &[], &mut v);
        assert_eq!(v.drain().len(), 1, "EEPROM read at 0");
        let mut begin = BEGIN;
        begin[4] = 0;
        dev.control(vendor(0x40, 0xb3, 0, 10), &begin, &mut v);
        assert_eq!(v.drain().len(), 1, "wrong begin bytes");
        dev.control(vendor(0x40, 0xb3, 0, 10), &BEGIN, &mut v);
        assert_eq!(v.drain().len(), 1, "two begins");
        dev.transfer_out(2, &[4, 0], &mut v);
        dev.transfer_out(2, &[4, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "a command without begin");
        bulk(&mut dev, &[0x0b, 0, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "a 2250 command to a 2090");
        bulk(&mut dev, &[4, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "a command of the wrong length");
        bulk(&mut dev, &[0, 0x0f, 0x20, 0, 0, 0, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "a filter bit the driver never sets");
        bulk(&mut dev, &[7, 0x0f, 0x33, 0, 0, 0, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "range code 3");
        bulk(&mut dev, &[2, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "force without a capture");
        bulk(&mut dev, &[3, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "capture before the configuration");
        bulk(&mut dev, &[5, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "data before ready");
        let mut cmd = trigger_default();
        cmd[2] = 2 | 2 << 2; // large frame at 10 us
        bulk(&mut dev, &cmd, &mut v);
        assert_eq!(v.drain().len(), 1, "10 us with the large frame");
        cmd = trigger_default();
        cmd[3] = 3 | 0x10 | 0x04;
        cmd[10] = 0x08;
        bulk(&mut dev, &cmd, &mut v);
        assert_eq!(v.drain().len(), 4, "channels 3, fast rates, an unused bit, a position beyond");
        let mut relays = [0u8; 17];
        relays[..8].copy_from_slice(&[0x00, 0x04, 0xf7, 0x05, 0x20, 0x40, 0x10, 0x01]);
        relays[9] = 1;
        dev.control(vendor(0x40, 0xb5, 0, 17), &relays, &mut v);
        assert_eq!(v.drain().len(), 3, "a lone 10 mV relay, a bad byte, a nonzero tail");
        let mut offsets = [0u8; 17];
        offsets[0] = 0x21;
        offsets[2] = 0x20;
        offsets[4] = 0x20;
        dev.control(vendor(0x40, 0xb4, 0, 17), &offsets, &mut v);
        let got = v.drain();
        assert_eq!(got.len(), 1, "a channel offset before the EEPROM read: {got:?}");
        dev.control(vendor(0xc0, 0xa2, 8, 72), &[], &mut v);
        offsets[0] = 0x40;
        dev.control(vendor(0x40, 0xb4, 0, 17), &offsets, &mut v);
        let got = v.drain();
        assert!(got.len() >= 2, "an unmarked offset, below the levels: {got:?}");
        assert_eq!(dev.transfer_in(6, 512, &mut v), Reply::Nak, "nothing asked");
        assert!(build(&[("profile".into(), "dso-9999".into())]).is_err());
        for bad in [("stall", "begin@0"), ("stall", "nope"), ("state", "7"), ("speed", "low")] {
            assert!(build(&[(bad.0.into(), bad.1.into())]).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn the_2250_takes_its_own_commands() {
        let mut v = Violations::default();
        let mut dev = running(&[("profile", "dso-2250")]);
        dev.control(vendor(0xc0, 0xa2, 8, 72), &[], &mut v);
        bulk(&mut dev, &[0x0c, 0, 0, 0, 0, 0, 0, 0], &mut v);
        bulk(&mut dev, &[0x0d, 0, 1, 0], &mut v);
        // 10 kHz: ~(10000 - 2).
        let d = (!9998u16).to_le_bytes();
        bulk(&mut dev, &[0x0e, 0, 2, 0, d[0], d[1], 0, 0], &mut v);
        bulk(&mut dev, &[0x0b, 0, 2, 0], &mut v);
        // Capture ratio 100: 0x7ffff and 0x7d7ff.
        bulk(&mut dev, &[0x0f, 0, 0xff, 0xff, 0x07, 0, 0xff, 0xd7, 0x07, 0], &mut v);
        assert!(v.is_empty(), "{v:?}");
        bulk(&mut dev, &[1; 12], &mut v);
        assert_eq!(v.drain().len(), 1, "the 2090's command");
        // Voltage: the header 0x08, CH1 500 mV (2), CH2 2 V (1 << 2).
        bulk(&mut dev, &[7, 0, 0x08 | 2 | 4, 0, 0, 0, 0, 0], &mut v);
        assert!(v.is_empty(), "{v:?}");
        // CH2 at 5 V: 0x08 | 0x08, read as code 0.
        bulk(&mut dev, &[7, 0, 0x08 | 2 | 8, 0, 0, 0, 0, 0], &mut v);
        assert!(v.is_empty(), "{v:?}");
        bulk(&mut dev, &[7, 0, 0x02, 0, 0, 0, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "no header");
        let d = (!3u16).to_le_bytes();
        bulk(&mut dev, &[0x0e, 0, 2, 0, d[0], d[1], 0, 0], &mut v);
        assert!(v.drain().is_empty(), "20 MHz, 100 / 5, is in the list");
        let d = (!1u16).to_le_bytes();
        bulk(&mut dev, &[0x0e, 0, 2, 0, d[0], d[1], 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "33.3 MHz is no rate of the list");
        bulk(&mut dev, &[0x0b, 0, 1, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "no channel");
        bulk(&mut dev, &[0x0f, 0, 0xff, 0xff, 0x07, 0, 0xff, 0xff, 0x07, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "positions that do not add up");
    }

    #[test]
    fn faults_stall_the_nth_request_and_answer_other_states() {
        let mut v = Violations::default();
        let mut dev = running(&[("stall", "begin@2,state@1"), ("state", "127@2")]);
        assert_eq!(bulk(&mut dev, &[4, 0], &mut v), Reply::ack());
        assert_eq!(bulk(&mut dev, &[4, 0], &mut v), Reply::Stall);
        assert_eq!(bulk(&mut dev, &[6, 0], &mut v), Reply::ack());
        assert_eq!(dev.transfer_in(6, 512, &mut v), Reply::Stall);
        bulk(&mut dev, &[6, 0], &mut v);
        let Reply::Data(a) = dev.transfer_in(6, 512, &mut v) else { panic!() };
        assert_eq!(a[0], 127);
        assert!(v.is_empty(), "a fault is the device's, not the host's: {v:?}");
        // The second data transfer empty: the frame goes on with the third.
        let mut dev = running(&[("empty", "2")]);
        init(&mut dev, &mut v);
        bulk(&mut dev, &[3, 0], &mut v);
        bulk(&mut dev, &[2, 0], &mut v);
        poll(&mut dev, &mut v);
        assert_eq!(poll(&mut dev, &mut v).0, 2);
        bulk(&mut dev, &[5, 0], &mut v);
        let lens: Vec<usize> = (0..42)
            .map(|_| match dev.transfer_in(6, 512, &mut v) {
                Reply::Data(d) => d.len(),
                _ => usize::MAX,
            })
            .collect();
        assert_eq!(&lens[..3], &[512, 0, 512]);
        assert_eq!(lens[40], 512, "the frame's last packet comes 41st");
        assert_eq!(lens[41], usize::MAX, "then nothing");
        assert!(v.is_empty(), "{v:?}");
        // in-limit: answers up to the Nth IN, NAKs after it.
        let mut dev = running(&[("in-limit", "1")]);
        init(&mut dev, &mut v);
        bulk(&mut dev, &[3, 0], &mut v);
        assert_eq!(poll(&mut dev, &mut v).0, 1);
        bulk(&mut dev, &[6, 0], &mut v);
        assert_eq!(dev.transfer_in(6, 512, &mut v), Reply::Nak);
        let mut dev = running(&[("stall", "eeprom")]);
        assert_eq!(dev.control(vendor(0xc0, 0xa2, 8, 72), &[], &mut v), Reply::Stall);
        let dev = running(&[("endpoints", "bad"), ("speed", "full")]);
        let eps = Descriptors::endpoints(&dev.descriptors().configs[0]);
        assert_eq!((eps[1].address, eps[1].max_packet), (0x82, 64));
        let dev = running(&[("endpoints", "swapped")]);
        let eps = Descriptors::endpoints(&dev.descriptors().configs[0]);
        assert_eq!((eps[0].address, eps[1].address), (0x86, 0x02));
        let dev = running(&[("endpoints", "one")]);
        assert_eq!(Descriptors::endpoints(&dev.descriptors().configs[0]).len(), 1);
        let mut boot = Device::new(build(&[("stall".into(), "load".into())]).unwrap());
        boot.control(SET_CONFIG, &[], &mut v);
        assert_eq!(boot.control(vendor(0x40, 0xa0, 0xe600, 1), &[1], &mut v), Reply::Stall);
        assert!(v.is_empty(), "{v:?}");
    }
}
