// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The Hantek 6022BE/6022BL USB oscilloscopes and their relatives
//! (Voltcraft DSO-2020, Sainsmart DDS120, `YiXingDianZi` MDSO, Instrustar
//! ISDS205B) with sigrok's open firmware, as libsigrok's `hantek-6xxx`
//! driver sees them, built on the EZ-USB shell ([`crate::shells::ezusb`]).
//!
//! Sources, and nothing from memory:
//!
//! - the profiles (ids before and after the upload, `bcdDevice`,
//!   firmware file, coupling, V/div table), the requests and what the
//!   driver sends: `api.c`, `protocol.c` and `protocol.h` of the driver;
//! - the firmware: sigrok-firmware-fx2lafw 0.1.7-10-g0f2d324, `include/scope.inc`
//!   (the vendor commands), `hw/<board>/fw.c` (the gains and rates each
//!   board accepts, the coupling), `include/dscr_scope.inc` and
//!   `hw/<board>/dscr.a51` (the descriptors and strings), and
//!   `Makefile.am`, whose scope link layout (`--code-size 0x3c00`,
//!   descriptors at `0x3d00`, the jump table at `0x3f00`) needs the 16
//!   KiB of an FX2LP: the model's chip.
//!
//! Without `flashed=1` the scope comes up as an FX2 without firmware
//! under the board's own ids (`04b4:6022` and so on, from its EEPROM),
//! with the bare FX2LP's `bcdDevice` 0xa001: an assumption, as a real
//! board's `0xC0` EEPROM load sets its own, which the driver does not
//! look at before the upload, so it changes nothing here. It takes the upload through the boot loader and renumerates as
//! `1d50:608e`, the `bcdDevice` telling the boards apart (`VER = 0x0100`
//! in `dscr.a51` is big endian, as in fx2lafw: 0x0001 for the 6022BE, 2
//! DDS120, 3 6022BL, 4 MDSO, 5 ISDS205B). Strings `sigrok`, `fx2lafw` and
//! the board (`Hantek 6022BE`, ...); max power 110 mA (`0x37`);
//! interface 0 alternate 0 has bulk IN endpoint 6 of 512 bytes, the one
//! the driver reads, alternates 1 to 7 the isochronous endpoint 2 the
//! driver never selects (kept, as the device has them).
//!
//! The firmware's vendor commands (`handle_vendorcommand()` of
//! `scope.inc`), each a one byte OUT; every command, known or not,
//! first stops a running acquisition (`stop_sampling()`):
//!
//! | request | data | effect |
//! |---|---|---|
//! | `0xe0`, `0xe1` | gain register | CH1, CH2 input range; the driver sends 1, 2, 5, 10 (11, 12 on the ISDS205B) |
//! | `0xe2` | rate register | 48, 30, 24, 16, 12, 8, 4, 2, 1 MHz; 50, 20, 10 are 500, 200, 100 kHz |
//! | `0xe3` | 1 or 0 | 1 clears the FIFO and starts sampling; anything else only stops |
//! | `0xe4` | 1 or 2 | channels in the stream; the driver never sends it |
//! | `0xe5` | coupling | bit 0 CH1, bit 4 CH2, set for DC; DDS120 and ISDS205B only |
//! | `0xe6` | pulse | calibration output frequency (DDS120); never sent |
//!
//! A value the board does not know is ignored (the command still
//! succeeds); any other request STALLs (`handle_vendorcommand()`
//! returns FALSE). The pedantic checks hold the requests to what the
//! driver sends: OUT, `wValue` and `wIndex` 0, one byte; the gains and
//! rates of the driver's tables (`VDIV_REG`, `SAMPLERATE_REGS`), not
//! only those the firmware takes (it also has 2 and 12 MHz, a gain of 20
//! on the DDS120 and 16 on the MDSO); coupling only to a board that has
//! it, with bits 0 and 4 only; `0xe3` with 0 or 1; `bmRequestType`
//! 0x40 (vendor, device, OUT); a start without the rate, both gains
//! and, on the DDS120 and ISDS205B, the coupling sent since the previous
//! start (the driver's `hantek_6xxx_init()` sends them before every
//! start);
//! bulk IN transfers that are not a multiple of 512 bytes or from
//! another endpoint; the upload byte for byte as `ezusb.c` sends it, of
//! the profile's file; a firmware upload to the running firmware.
//!
//! The stream: with two channels the FIFO is word wide, one byte of CH1
//! then one of CH2 per sample, which is how `send_chunk()` reads it. A
//! byte is the 8 bit ADC code of the input over the selected range, 0
//! at the bottom and 255 at the top. The model's inputs (its own
//! choice): CH1 a 1 kHz square wave between 0 and 1 V, CH2 a 10 kHz
//! triangle between -1 and +1 V, both as functions of the time of the
//! sample at the selected rate, so the rate register shows in the
//! data. AC coupling removes their mean (0.5 V on CH1). The ranges per
//! gain register are the driver's own (`VDIV_VALUES`, 10 divisions: a
//! gain of 1 is 10 V across the codes on the Hantek), since no other
//! source gives the analog front end's gains; the firmware's comments
//! give the selector bits only. Codes are rounded and clipped.
//!
//! Emulation artifacts: the real board samples continuously and drops
//! what the host does not fetch in time; the model's stream has no
//! gaps, sample n is at time n / rate from the start whatever the host
//! does. It completes one bulk IN transfer per transfer the host
//! submits after the previous one completed, with all the bytes it
//! asks for, so the transactions do not depend on timing (the driver
//! keeps a single transfer in flight anyway). The "glitch" the driver's
//! first 1024 byte transfer flushes is not modelled: those bytes are
//! ordinary samples, and the output starts at sample 512.
//!
//! Faults, all off by default: `stall=NAME[/VALUE],...` STALLs the
//! command (`vdiv1`, `vdiv2`, `samplerate`, `trigger`, `coupling`), or
//! only the one with that data byte, with no effect at all, or a step
//! of the boot loader's upload (`hold`, `chunk`, `release`: the CPUCS
//! write of 1, the code writes, the CPUCS write of 0); `empty=N`
//! completes the first N bulk IN transfers of each acquisition with no
//! data, `stall_in=N` STALLs the N after those; `short=N` ends every
//! bulk IN transfer after at most N bytes (even, not a multiple of 512:
//! a short packet); `bcd=HEX` makes the firmware report another
//! `bcdDevice`, as a firmware of another board would; `slow=N` lets the
//! first N bulk IN transfers of each acquisition complete and holds the
//! later ones until the host cancels them, when they complete with what
//! came by then, at most [`SLOW_BYTES`] (a transfer longer than the
//! driver's 4 s timeout: 1 MiB takes 5.2 s at 100 kHz).
//!
//! What the model taught about the C driver (see `tools/emu/hantek-6xxx.sh`):
//! a transfer that completes empty or fails after the flush is neither
//! resubmitted nor ends the acquisition, which then never ends (the
//! `--hangs` cases with `empty=2` and `stall_in=2`: sigrok-cli waits,
//! with no transfer after, until the harness stops it); with
//! CH1 disabled, CH2's samples go out as CH1's (`send_chunk()` walks
//! `enabled_channels`, which holds every channel); on a DC only board,
//! setting coupling `DC` stores index 0, `COUPLING_AC`, so the driver
//! then reports `AC`; a rate outside its list is accepted and sent as
//! 100 kHz; the errors of the configuration requests are logged and
//! the capture goes on. A plain `--time` limit ends the capture in
//! `receive_transfer()` by wall time (`g_get_monotonic_time()` against
//! `limit_msec`), so how many transfers it takes depends on timing: no
//! case has it; `--time` with `--samples` covers the transfer sizing.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{
    BULK, Descriptors, DeviceModel, ISOCHRONOUS, Reply, Setup, Speed, VENDOR, Violations,
};
use crate::shells::ezusb::{self, Chip, Loader};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "hantek-6xxx";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Hantek 6022BE/BL USB scope and relatives (EZ-USB, renumerates as 1d50:608e); \
                         profile=6022be (default), 6022be-04b5, dso2020, dds120, 6022bl, 6022bl-04b5, \
                         mdso, isds205b; flashed=1 starts with the firmware running; faults: \
                         stall=NAME[/VALUE],..., empty=N, stall_in=N, short=N, bcd=HEX, slow=N";

/// What a transfer held by `slow=N` holds when it is cancelled.
const SLOW_BYTES: usize = 512;

/// A board the driver knows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// The option value.
    pub key: &'static str,
    /// Vendor id before the upload.
    pub vendor: u16,
    /// Product id before the upload.
    pub product: u16,
    /// `bcdDevice` of the firmware.
    pub version: u16,
    /// The firmware file the driver uploads.
    pub firmware: &'static str,
    /// String 3 of the firmware.
    pub board: &'static str,
    /// Whether the board switches AC/DC coupling (`0xe5`).
    pub coupling: bool,
    /// The driver's V/div table for it, `(numerator, denominator)`.
    pub vdivs: &'static [(u32, u32)],
    /// The gain registers the firmware accepts.
    pub gains: &'static [u8],
    /// The rate registers the firmware accepts.
    pub rates: &'static [u8],
}

/// The driver's `VDIV_VALUES`.
const VDIVS: [(u32, u32); 4] = [(1, 1), (500, 1000), (250, 1000), (100, 1000)];
/// The driver's `VDIV_VALUES_INSTRUSTAR`.
const VDIVS_INSTRUSTAR: [(u32, u32); 6] =
    [(128, 100), (705, 1000), (288, 1000), (140, 1000), (576, 10000), (176, 10000)];
/// The driver's `VDIV_REG`: the gain register of each V/div index.
pub const VDIV_REG: [u8; 7] = [1, 2, 5, 10, 11, 12, 13];
/// The driver's `SAMPLERATE_VALUES` and `SAMPLERATE_REGS`.
pub const SAMPLERATES: [(u64, u8); 10] = [
    (48_000_000, 48),
    (30_000_000, 30),
    (24_000_000, 24),
    (16_000_000, 16),
    (8_000_000, 8),
    (4_000_000, 4),
    (1_000_000, 1),
    (500_000, 50),
    (200_000, 20),
    (100_000, 10),
];
/// The rates of `hw/hantek-6022be/fw.c` (and 6022BL, MDSO, ISDS205B).
const RATES_6022: [u8; 12] = [48, 30, 24, 16, 12, 8, 4, 2, 1, 50, 20, 10];
/// The rates of `hw/sainsmart-dds120/fw.c`.
const RATES_DDS120: [u8; 17] = [48, 30, 24, 16, 15, 12, 11, 8, 6, 5, 4, 3, 2, 1, 50, 20, 10];

const FW_6022BE: &str = "fx2lafw-hantek-6022be.fw";
const FW_6022BL: &str = "fx2lafw-hantek-6022bl.fw";

/// The emulated boards, in the driver's order.
pub const PROFILES: [Profile; 8] = [
    Profile {
        key: "6022be",
        vendor: 0x04b4,
        product: 0x6022,
        version: 1,
        firmware: FW_6022BE,
        board: "Hantek 6022BE",
        coupling: false,
        vdivs: &VDIVS,
        gains: &[1, 2, 5, 10],
        rates: &RATES_6022,
    },
    Profile {
        key: "6022be-04b5",
        vendor: 0x04b5,
        product: 0x6022,
        version: 1,
        firmware: FW_6022BE,
        board: "Hantek 6022BE",
        coupling: false,
        vdivs: &VDIVS,
        gains: &[1, 2, 5, 10],
        rates: &RATES_6022,
    },
    Profile {
        // The driver uploads the 6022BE firmware to it.
        key: "dso2020",
        vendor: 0x04b4,
        product: 0x2020,
        version: 1,
        firmware: FW_6022BE,
        board: "Hantek 6022BE",
        coupling: false,
        vdivs: &VDIVS,
        gains: &[1, 2, 5, 10],
        rates: &RATES_6022,
    },
    Profile {
        key: "dds120",
        vendor: 0x8102,
        product: 0x8102,
        version: 2,
        firmware: "fx2lafw-sainsmart-dds120.fw",
        board: "SainSmart DDS120",
        coupling: true,
        vdivs: &VDIVS,
        gains: &[1, 2, 5, 10, 20],
        rates: &RATES_DDS120,
    },
    Profile {
        key: "6022bl",
        vendor: 0x04b4,
        product: 0x602a,
        version: 3,
        firmware: FW_6022BL,
        board: "Hantek 6022BL",
        coupling: false,
        vdivs: &VDIVS,
        gains: &[1, 2, 5, 10],
        rates: &RATES_6022,
    },
    Profile {
        key: "6022bl-04b5",
        vendor: 0x04b5,
        product: 0x602a,
        version: 3,
        firmware: FW_6022BL,
        board: "Hantek 6022BL",
        coupling: false,
        vdivs: &VDIVS,
        gains: &[1, 2, 5, 10],
        rates: &RATES_6022,
    },
    Profile {
        key: "mdso",
        vendor: 0xd4a2,
        product: 0x5660,
        version: 4,
        firmware: "fx2lafw-yixingdianzi-mdso.fw",
        board: "YiXingDianZi MDSO",
        coupling: false,
        vdivs: &VDIVS,
        gains: &[1, 2, 5, 10, 16],
        rates: &RATES_6022,
    },
    Profile {
        key: "isds205b",
        vendor: 0xd4a2,
        product: 0x5661,
        version: 5,
        firmware: "fx2lafw-instrustar-isds205b.fw",
        board: "Instrustar ISDS205B",
        coupling: true,
        vdivs: &VDIVS_INSTRUSTAR,
        gains: &[1, 2, 5, 10, 11, 12],
        rates: &RATES_6022,
    },
];

/// The size of every stand-in firmware, as `tools/emu/hantek-6xxx.firmware`
/// lists it: up to the end of the scope build's jump table area
/// (`INT2JT=0x3f00`, 128 bytes), four chunks of `ezusb.c`.
pub const FIRMWARE_SIZE: usize = 0x3f80;

/// The fixed vendor and product id of the running firmware.
pub const FW_VENDOR: u16 = 0x1d50;
/// See [`FW_VENDOR`].
pub const FW_PRODUCT: u16 = 0x608e;

const VDIV_CH1: u8 = 0xe0;
const VDIV_CH2: u8 = 0xe1;
const SAMPLERATE: u8 = 0xe2;
const TRIGGER: u8 = 0xe3;
const CHANNELS: u8 = 0xe4;
const COUPLING: u8 = 0xe5;
const CALIBRATION: u8 = 0xe6;
/// The boot loader's steps in the `stall=` fault (no vendor command
/// has these numbers): the 8051 held in reset, a chunk of code, the
/// reset released.
const LOAD_HOLD: u8 = 1;
const LOAD_CHUNK: u8 = 2;
const LOAD_RELEASE: u8 = 3;
/// The command names of the `stall=` fault.
const NAMES: [(&str, u8); 8] = [
    ("hold", LOAD_HOLD),
    ("chunk", LOAD_CHUNK),
    ("release", LOAD_RELEASE),
    ("vdiv1", VDIV_CH1),
    ("vdiv2", VDIV_CH2),
    ("samplerate", SAMPLERATE),
    ("trigger", TRIGGER),
    ("coupling", COUPLING),
];
/// The bulk endpoint's packet size at high speed.
const PACKET: usize = 512;
/// The stream's endpoint.
const EP_IN: u8 = 6;

/// Ways the board can misbehave, as a real one can; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    /// Commands the firmware STALLs, with the data byte when only that one.
    pub stall: Vec<(u8, Option<u8>)>,
    /// Bulk IN transfers of each acquisition that complete with no data.
    pub empty: u32,
    /// Bulk IN transfers of each acquisition that STALL, after the empty ones.
    pub stall_in: u32,
    /// At most this many bytes per bulk IN transfer.
    pub short: Option<usize>,
    /// The firmware's `bcdDevice`, when not the profile's.
    pub bcd: Option<u16>,
    /// Bulk IN transfers of each acquisition that complete before the
    /// rest are held until cancelled.
    pub slow: Option<u32>,
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
                    let (name, value) = match item.split_once('/') {
                        Some((n, x)) => {
                            let x = x
                                .parse::<u8>()
                                .map_err(|_| Error(format!("stall={item}: a data byte 0 to 255")))?;
                            (n, Some(x))
                        }
                        None => (item, None),
                    };
                    let cmd = NAMES.iter().find(|(n, _)| *n == name).map(|(_, c)| *c).ok_or_else(
                        || {
                            let names: Vec<&str> = NAMES.iter().map(|(n, _)| *n).collect();
                            Error(format!("stall={name}: one of {}", names.join(", ")))
                        },
                    )?;
                    self.stall.push((cmd, value));
                }
            }
            "empty" => self.empty = count(v)?,
            "stall_in" => self.stall_in = count(v)?,
            "slow" => self.slow = Some(count(v)?),
            "short" => {
                let n = v.parse::<usize>().ok().filter(|n| {
                    *n > 0 && n.is_multiple_of(2) && !n.is_multiple_of(PACKET)
                });
                self.short = Some(n.ok_or_else(|| {
                    Error(format!("short={v}: an even byte count, not a multiple of {PACKET}"))
                })?);
            }
            "bcd" => {
                let hex = v.strip_prefix("0x").unwrap_or(v);
                self.bcd = Some(
                    u16::from_str_radix(hex, 16)
                        .map_err(|_| Error(format!("bcd={v}: a 16 bit hex number")))?,
                );
            }
            _ => return Ok(false),
        }
        Ok(true)
    }

    fn stalls(&self, cmd: u8, value: Option<u8>) -> bool {
        self.stall.iter().any(|(c, x)| *c == cmd && (x.is_none() || *x == value))
    }
}

/// A board from the `-o` options: `profile=KEY` (see [`PROFILES`]),
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
    Ok(Box::new(Hantek::new(profile, flashed, faults)))
}

/// The descriptors of the running firmware, `dscr_scope.inc` at high
/// speed, with `bcdDevice` `version`.
pub fn firmware_descriptors(p: &Profile, version: u16) -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 1, [0xff, 0, 0], 0));
    body.extend_from_slice(&descriptor::endpoint(0x86, BULK, 512, 0));
    // The isochronous alternates on endpoint 2: wMaxPacketSize with the
    // transactions per microframe in bits 11 and 12, and bInterval.
    for (alt, size, interval) in [
        (1, 0x1400, 1),
        (2, 0x0c00, 1),
        (3, 0x0400, 1),
        (4, 0x0400, 2),
        (5, 0x0400, 3),
        (6, 0x0400, 4),
        (7, 0x0200, 4),
    ] {
        body.extend_from_slice(&descriptor::interface(0, alt, 1, [0xff, 0, 1], 0));
        body.extend_from_slice(&descriptor::endpoint(0x82, ISOCHRONOUS, size, interval));
    }
    let config = descriptor::configuration(1, 0, 0x80, 110, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0xff, 0xff, 0xff],
        max_packet0: 64,
        vendor: FW_VENDOR,
        product: FW_PRODUCT,
        device: version,
        strings: [1, 2, 3],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::High,
        device: dev.bytes(),
        configs: vec![config],
        strings: vec!["sigrok".into(), "fx2lafw".into(), p.board.into()],
    }
}

/// The rate of a rate register, in Hz.
pub fn rate_hz(reg: u8) -> u64 {
    match reg {
        50 => 500_000,
        20 => 200_000,
        10 => 100_000,
        r => u64::from(r) * 1_000_000,
    }
}

/// A running acquisition.
#[derive(Clone, Debug)]
struct Acquisition {
    /// The next sample's number.
    sample: u64,
    /// Whether the next bulk IN transfer may complete.
    ready: bool,
    /// Faulty transfers still to come: empty ones, then stalled ones.
    empty: u32,
    stall_in: u32,
    /// Transfers still to complete before `slow=N` holds them.
    slow: Option<u32>,
}

/// The firmware's settings, and what the host sent since it came up.
#[derive(Clone, Debug)]
struct Scope {
    gain: [u8; 2],
    rate: u8,
    /// Bit 0 CH1, bit 4 CH2: set for DC.
    coupling: u8,
    /// What the driver's `hantek_6xxx_init()` sent since the last start
    /// (or since the firmware came up): the rate, the gains, the coupling.
    sent_rate: bool,
    sent_gain: [bool; 2],
    sent_coupling: bool,
    acq: Option<Acquisition>,
}

impl Scope {
    /// `init()` of `scope.inc`: gains 1, 1 MHz; the coupling as the
    /// ports come up (DDS120: `IOE` clear, AC; ISDS205B: the shift
    /// register clear, DC; the others have none, DC).
    fn new(p: &Profile) -> Scope {
        Scope {
            gain: [1, 1],
            rate: 1,
            coupling: if p.key == "dds120" { 0 } else { 0x11 },
            sent_rate: false,
            sent_gain: [false; 2],
            sent_coupling: false,
            acq: None,
        }
    }

    /// Notes that the host sent configuration command `cmd`, applied or
    /// not.
    fn note_sent(&mut self, cmd: u8) {
        match cmd {
            VDIV_CH1 | VDIV_CH2 => self.sent_gain[usize::from(cmd - VDIV_CH1)] = true,
            SAMPLERATE => self.sent_rate = true,
            COUPLING => self.sent_coupling = true,
            _ => {}
        }
    }
}

/// What the firmware is doing.
#[derive(Clone, Debug)]
enum Stage {
    /// The boot loader, waiting for the firmware.
    Boot(Box<Loader>),
    /// The firmware runs.
    Running(Scope),
}

/// The emulated scope.
pub struct Hantek {
    profile: Profile,
    boot: Descriptors,
    firmware: Descriptors,
    stage: Stage,
    renumerate: bool,
    faults: Faults,
}

impl Hantek {
    /// A board; `flashed` starts it with the firmware running.
    pub fn new(profile: Profile, flashed: bool, faults: Faults) -> Hantek {
        let stage = if flashed {
            Stage::Running(Scope::new(&profile))
        } else {
            Stage::Boot(Box::new(Loader::new(Chip::Fx2lp)))
        };
        Hantek {
            boot: ezusb::bare_descriptors(profile.vendor, profile.product, ezusb::FX2LP_DID),
            firmware: firmware_descriptors(&profile, faults.bcd.unwrap_or(profile.version)),
            profile,
            stage,
            renumerate: false,
            faults,
        }
    }

    /// Whether the firmware runs.
    pub fn running(&self) -> bool {
        matches!(self.stage, Stage::Running(_))
    }

    /// The firmware image both tools must upload.
    pub fn image(&self) -> Vec<u8> {
        ezusb::standin(self.profile.firmware, FIRMWARE_SIZE)
    }

    /// The full scale of a gain register in volts: ten divisions of
    /// the driver's V/div for it, or `None` for a gain the driver does
    /// not send to this board.
    fn full_scale(&self, gain: u8) -> Option<f64> {
        let i = VDIV_REG.iter().position(|g| *g == gain)?;
        let (num, den) = self.profile.vdivs.get(i)?;
        Some(10.0 * f64::from(*num) / f64::from(*den))
    }

    /// The input of channel `ch` at sample `n` of rate `rate`, in volts:
    /// CH1 a 1 kHz square wave from 0 to 1 V, CH2 a 10 kHz triangle from
    /// -1 to 1 V; AC coupling takes their mean away.
    fn input(ch: usize, n: u64, rate: u64, dc: bool) -> f64 {
        if ch == 0 {
            let phase = (n * 1_000) % rate;
            let v = if phase < rate / 2 { 1.0 } else { 0.0 };
            if dc { v } else { v - 0.5 }
        } else {
            let x = ((n * 10_000) % rate) as f64 / rate as f64;
            let tri = if x < 0.5 { 4.0 * x - 1.0 } else { 3.0 - 4.0 * x };
            // The triangle's mean is 0: the coupling changes nothing.
            tri
        }
    }

    /// The ADC code of channel `ch` at sample `n`.
    fn code(&self, scope: &Scope, ch: usize, n: u64) -> u8 {
        let dc = scope.coupling & (if ch == 0 { 0x01 } else { 0x10 }) != 0;
        let v = Self::input(ch, n, rate_hz(scope.rate), dc);
        // A gain the driver does not send was flagged; read it as 1.
        let range = self.full_scale(scope.gain[ch]).unwrap_or(10.0);
        ((v + range / 2.0) * 255.0 / range).round().clamp(0.0, 255.0) as u8
    }

    fn boot_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() == VENDOR && setup.request == ezusb::REQ_FIRMWARE_LOAD && !setup.is_in() {
            let step = match (setup.value, data.first()) {
                (ezusb::CPUCS, Some(x)) if x & 1 == 1 => LOAD_HOLD,
                (ezusb::CPUCS, _) => LOAD_RELEASE,
                _ => LOAD_CHUNK,
            };
            if self.faults.stalls(step, None) {
                // A STALLed write has no effect.
                return Reply::Stall;
            }
        }
        let image = self.image();
        let Stage::Boot(loader) = &mut self.stage else { unreachable!() };
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
            // The firmware disconnects and comes back as the scope.
            self.stage = Stage::Running(Scope::new(&self.profile));
            self.renumerate = true;
        }
        reply
    }

    fn firmware_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() == VENDOR && setup.request == ezusb::REQ_FIRMWARE_LOAD {
            // The boot loader is in silicon and still answers, but the
            // driver never uploads to a device with the firmware.
            v.report("firmware load to an FX2 whose scope firmware runs");
            return Reply::ack();
        }
        let cmd = setup.request;
        if !(VDIV_CH1..=CALIBRATION).contains(&cmd) || setup.kind() != VENDOR {
            v.report(format!(
                "request {:#04x} {cmd:#04x} value {:#06x}: not one the scope firmware answers",
                setup.request_type, setup.value
            ));
            // handle_vendorcommand() stops sampling, then returns FALSE.
            self.stop();
            return Reply::Stall;
        }
        if setup.is_in() {
            v.report(format!("request {cmd:#04x} as an IN; the driver writes its one byte"));
        }
        if setup.recipient() != crate::model::RECIPIENT_DEVICE {
            v.report(format!(
                "request {cmd:#04x} with bmRequestType {:#04x}; the driver sends 0x40 (vendor, \
                 device)",
                setup.request_type
            ));
        }
        if setup.value != 0 || setup.index != 0 {
            v.report(format!(
                "request {cmd:#04x} with value {:#06x} index {:#06x}; the driver sends 0",
                setup.value, setup.index
            ));
        }
        if setup.length != 1 || (!setup.is_in() && data.len() != 1) {
            v.report(format!("request {cmd:#04x} with {} bytes; the driver sends 1", setup.length));
        }
        let value = data.first().copied();
        if self.faults.stalls(cmd, value) {
            // A STALLed command has no effect, but the host did send it.
            if let Stage::Running(scope) = &mut self.stage {
                scope.note_sent(cmd);
            }
            return Reply::Stall;
        }
        // Every command stops a running acquisition first.
        self.stop();
        let Some(value) = value.filter(|_| !setup.is_in()) else {
            // The firmware arms EP0 for nothing and reads a stale byte.
            return if setup.is_in() { Reply::Data(Vec::new()) } else { Reply::ack() };
        };
        self.apply(cmd, value, v);
        Reply::ack()
    }

    /// A command the firmware takes: what it changes.
    fn apply(&mut self, cmd: u8, value: u8, v: &mut Violations) {
        let p = self.profile;
        let Stage::Running(scope) = &mut self.stage else { unreachable!() };
        scope.note_sent(cmd);
        match cmd {
            VDIV_CH1 | VDIV_CH2 => {
                let ch = usize::from(cmd - VDIV_CH1);
                let driver = &VDIV_REG[..p.vdivs.len()];
                if !driver.contains(&value) {
                    v.report(format!(
                        "CH{} gain {value}: the driver sends {driver:?} to the {}",
                        ch + 1,
                        p.board
                    ));
                }
                if p.gains.contains(&value) {
                    scope.gain[ch] = value;
                }
            }
            SAMPLERATE => {
                if !SAMPLERATES.iter().any(|(_, r)| *r == value) {
                    v.report(format!("rate register {value}: not one of the driver's rates"));
                }
                if p.rates.contains(&value) {
                    scope.rate = value;
                }
            }
            TRIGGER => match value {
                1 => {
                    // dev_acquisition_start() runs hantek_6xxx_init()
                    // before every start: the rate, both gains and, on a
                    // board with coupling, the coupling, each time.
                    if !scope.sent_rate || scope.sent_gain != [true; 2] {
                        v.report(
                            "acquisition started without the rate and both gains sent since the \
                             last start; hantek_6xxx_init() sends them first",
                        );
                    }
                    if p.coupling && !scope.sent_coupling {
                        v.report(format!(
                            "acquisition started without the coupling sent since the last start; \
                             hantek_6xxx_init() sends it to the {}",
                            p.board
                        ));
                    }
                    scope.sent_rate = false;
                    scope.sent_gain = [false; 2];
                    scope.sent_coupling = false;
                    scope.acq = Some(Acquisition {
                        sample: 0,
                        ready: true,
                        empty: self.faults.empty,
                        stall_in: self.faults.stall_in,
                        slow: self.faults.slow,
                    });
                }
                0 => {}
                x => v.report(format!("trigger {x}: the driver sends 1 to start and 0 to stop")),
            },
            COUPLING => {
                if p.coupling {
                    if value & !0x11 != 0 {
                        v.report(format!("coupling {value:#04x}: only bits 0 and 4 are defined"));
                    }
                    scope.coupling = value & 0x11;
                } else {
                    v.report(format!("coupling sent to the {}, which has none", p.board));
                }
            }
            CHANNELS | CALIBRATION => {
                // CHANNELS and CALIBRATION: hantek_6xxx_update_channels()
                // is commented out, and nothing sends the pulse.
                v.report(format!("request {cmd:#04x}: the driver never sends it"));
            }
            _ => unreachable!("checked above"),
        }
    }

    /// `stop_sampling()`: the GPIF stops, no more data.
    fn stop(&mut self) {
        if let Stage::Running(scope) = &mut self.stage {
            scope.acq = None;
        }
    }
}

impl DeviceModel for Hantek {
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

    fn out(&mut self, ep: u8, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!("OUT transfer to endpoint {ep}; the scope has none"));
        Reply::Stall
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep != EP_IN {
            v.report(format!("IN transfer from endpoint {ep}; the samples come from 6"));
            return Reply::Stall;
        }
        let Stage::Running(scope) = &self.stage else {
            return Reply::Nak;
        };
        let Some(acq) = &scope.acq else {
            // Not sampling: the FIFO stays empty and the host waits.
            return Reply::Nak;
        };
        if !acq.ready {
            return Reply::Nak;
        }
        if !max.is_multiple_of(PACKET) {
            v.report(format!(
                "bulk IN transfer of {max} bytes; the driver asks for multiples of {PACKET}"
            ));
        }
        if acq.slow == Some(0) {
            // Held: see in_unlinked().
            return Reply::Nak;
        }
        let mut acq = acq.clone();
        acq.ready = false;
        if let Some(n) = &mut acq.slow {
            *n -= 1;
        }
        // The faults, paced like the samples: one per submission.
        let reply = if acq.empty > 0 {
            acq.empty -= 1;
            Reply::Data(Vec::new())
        } else if acq.stall_in > 0 {
            acq.stall_in -= 1;
            Reply::Stall
        } else {
            let len = self.faults.short.map_or(max, |s| s.min(max)) / 2;
            let mut data = Vec::with_capacity(len * 2);
            for _ in 0..len {
                data.push(self.code(scope, 0, acq.sample));
                data.push(self.code(scope, 1, acq.sample));
                acq.sample += 1;
            }
            Reply::Data(data)
        };
        if let Stage::Running(scope) = &mut self.stage {
            scope.acq = Some(acq);
        }
        reply
    }

    fn in_submitted(&mut self, ep: u8, _max: usize) {
        if let Stage::Running(Scope { acq: Some(acq), .. }) = &mut self.stage
            && ep == EP_IN
        {
            acq.ready = true;
        }
    }

    /// A transfer `slow=N` holds completes with what came by the cancel.
    fn in_unlinked(&mut self, ep: u8, max: usize) -> Option<Vec<u8>> {
        let Stage::Running(scope) = &self.stage else { return None };
        let acq = scope.acq.as_ref()?;
        if ep != EP_IN || acq.slow != Some(0) || !acq.ready {
            return None;
        }
        let len = SLOW_BYTES.min(max) / 2;
        let mut sample = acq.sample;
        let mut data = Vec::with_capacity(len * 2);
        for _ in 0..len {
            data.push(self.code(scope, 0, sample));
            data.push(self.code(scope, 1, sample));
            sample += 1;
        }
        if let Stage::Running(Scope { acq: Some(acq), .. }) = &mut self.stage {
            acq.sample = sample;
            acq.ready = false;
        }
        Some(data)
    }

    fn reset(&mut self) {
        // A new attach ends an acquisition; the firmware stays in RAM
        // and keeps its settings.
        self.stop();
    }

    /// Firmware chunks of 4 KiB and transfers of up to 12 MiB.
    fn log_digest_above(&self) -> Option<usize> {
        Some(crate::txlog::HASH_ABOVE)
    }

    fn take_renumeration(&mut self) -> bool {
        std::mem::take(&mut self.renumerate)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Device;

    const SET_CONFIG: Setup = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };

    fn cmd(request: u8) -> Setup {
        Setup { request_type: 0x40, request, value: 0, index: 0, length: 1 }
    }

    fn load(value: u16, length: u16) -> Setup {
        Setup { request_type: 0x40, request: 0xa0, value, index: 0, length }
    }

    /// The upload as `ezusb_upload_firmware()` sends it.
    fn upload(dev: &mut Device, image: &[u8], v: &mut Violations) {
        assert_eq!(dev.control(SET_CONFIG, &[], v), Reply::ack());
        assert_eq!(dev.control(load(0xe600, 1), &[1], v), Reply::ack());
        for (i, c) in image.chunks(4096).enumerate() {
            assert_eq!(dev.control(load((i * 4096) as u16, c.len() as u16), c, v), Reply::ack());
        }
        assert_eq!(dev.control(load(0xe600, 1), &[0], v), Reply::ack());
    }

    /// `hantek_6xxx_init()` then the start, as `dev_acquisition_start()`.
    fn start(dev: &mut Device, rate: u8, gains: [u8; 2], coupling: Option<u8>, v: &mut Violations) {
        dev.control(cmd(SAMPLERATE), &[rate], v);
        dev.control(cmd(VDIV_CH1), &[gains[0]], v);
        dev.control(cmd(VDIV_CH2), &[gains[1]], v);
        if let Some(c) = coupling {
            dev.control(cmd(COUPLING), &[c], v);
        }
        assert_eq!(dev.control(cmd(TRIGGER), &[1], v), Reply::ack());
    }

    fn flashed(key: &str, options: &[(&str, &str)]) -> Device {
        let mut o: Vec<(String, String)> =
            vec![("flashed".into(), "1".into()), ("profile".into(), key.into())];
        o.extend(options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())));
        let mut dev = Device::new(build(&o).unwrap());
        dev.control(SET_CONFIG, &[], &mut Violations::default());
        dev
    }

    #[test]
    fn the_firmware_list_holds_every_profiles_file() {
        let list = include_str!("../../../tools/emu/hantek-6xxx.firmware");
        for p in PROFILES {
            let line = list
                .lines()
                .find(|l| l.split_whitespace().next() == Some(p.firmware))
                .unwrap_or_else(|| panic!("{} is not in the firmware list", p.firmware));
            let size: usize = line.split_whitespace().nth(1).unwrap().parse().unwrap();
            assert_eq!(size, FIRMWARE_SIZE, "{}", p.firmware);
        }
    }

    #[test]
    fn the_driver_tables_fit_the_firmware() {
        for p in PROFILES {
            for g in &VDIV_REG[..p.vdivs.len()] {
                assert!(p.gains.contains(g), "{}: gain {g}", p.key);
            }
            for (_, r) in SAMPLERATES {
                assert!(p.rates.contains(&r), "{}: rate {r}", p.key);
            }
        }
        assert_eq!(rate_hz(50), 500_000);
        assert_eq!(rate_hz(48), 48_000_000);
    }

    #[test]
    fn uploads_renumerates_and_streams_as_the_driver_drives_it() {
        let model = Hantek::new(PROFILES[0], false, Faults::default());
        let image = model.image();
        let mut dev = Device::new(Box::new(model));
        let mut v = Violations::default();
        assert_eq!(dev.descriptors().vendor_id(), 0x04b4);
        assert_eq!(dev.descriptors().product_id(), 0x6022);
        upload(&mut dev, &image, &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert!(dev.take_renumeration());
        dev.reset();
        assert_eq!(dev.descriptors().vendor_id(), FW_VENDOR);
        assert_eq!(dev.descriptors().product_id(), FW_PRODUCT);
        assert_eq!(dev.descriptors().bcd_device(), 1);
        assert_eq!(dev.descriptors().strings, ["sigrok", "fx2lafw", "Hantek 6022BE"]);
        dev.control(SET_CONFIG, &[], &mut v);
        // Not sampling yet: the host waits.
        dev.in_submitted(6, 1024);
        assert_eq!(dev.transfer_in(6, 1024, &mut v), Reply::Nak);
        // 100 kHz, 1 V/div on both: 10 V across the codes.
        start(&mut dev, 10, [1, 1], None, &mut v);
        let Reply::Data(d) = dev.transfer_in(6, 1024, &mut v) else { panic!() };
        assert_eq!(d.len(), 1024);
        // CH1 1 V: 11.5 / 10 * 255; CH2 at -1 V: 4 / 10 * 255.
        assert_eq!((d[0], d[1]), (153, 102));
        // Half a period of 1 kHz at 100 kHz is 50 samples: 0 V.
        assert_eq!(d[100], 128);
        assert_eq!(dev.transfer_in(6, 1024, &mut v), Reply::Nak, "one per submission");
        dev.in_submitted(6, 512);
        let Reply::Data(d) = dev.transfer_in(6, 512, &mut v) else { panic!() };
        assert_eq!(d[0], 153, "sample 512 is in the high half of a period");
        // The stop: no more data.
        dev.control(cmd(TRIGGER), &[0], &mut v);
        dev.in_submitted(6, 512);
        assert_eq!(dev.transfer_in(6, 512, &mut v), Reply::Nak);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn the_range_and_the_coupling_show_in_the_codes() {
        let mut v = Violations::default();
        let mut dev = flashed("dds120", &[]);
        assert_eq!(dev.descriptors().bcd_device(), 2);
        // 100 mV/div: 1 V across the codes; CH1 at 1 V clips. AC: 0.5 V.
        start(&mut dev, 10, [10, 10], Some(0x10), &mut v);
        let Reply::Data(d) = dev.transfer_in(6, 512, &mut v) else { panic!() };
        assert_eq!(d[0], 255, "AC: 0.5 V is the top");
        start(&mut dev, 10, [10, 10], Some(0x11), &mut v);
        dev.in_submitted(6, 512);
        let Reply::Data(d) = dev.transfer_in(6, 512, &mut v) else { panic!() };
        assert_eq!((d[0], d[1]), (255, 0), "DC: 1 V and -1 V clip");
        start(&mut dev, 10, [2, 2], Some(0x11), &mut v);
        let Reply::Data(d) = dev.transfer_in(6, 512, &mut v) else { panic!() };
        // 500 mV/div: 5 V across; 1 V is 3.5 / 5 * 255.
        assert_eq!(d[0], 179);
        assert!(v.is_empty(), "{v:?}");
        // The ISDS205B's ranges are its own: gain 12 is 176 mV across.
        let mut dev = flashed("isds205b", &[]);
        start(&mut dev, 1, [12, 1], Some(0x11), &mut v);
        let Reply::Data(d) = dev.transfer_in(6, 512, &mut v) else { panic!() };
        assert_eq!(d[0], 255);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_what_the_driver_and_the_firmware_forbid() {
        let mut v = Violations::default();
        // Bare chip: anything but the loader, and a wrong upload.
        let model = Hantek::new(PROFILES[4], false, Faults::default());
        let mut image = model.image();
        let mut dev = Device::new(Box::new(model));
        assert_eq!(dev.control(cmd(SAMPLERATE), &[1], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "vendor request to the boot loader");
        image[5000] ^= 1;
        upload(&mut dev, &image, &mut v);
        assert_eq!(v.drain().len(), 1, "the image differs from the stand-in");
        dev.reset();
        dev.control(SET_CONFIG, &[], &mut v);
        dev.control(load(0xe600, 1), &[1], &mut v);
        assert_eq!(v.drain().len(), 1, "upload to the running firmware");
        assert_eq!(dev.control(cmd(TRIGGER), &[1], &mut v), Reply::ack());
        assert_eq!(v.drain().len(), 1, "start before the configuration");
        assert_eq!(dev.control(cmd(0xe7), &[0], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "unknown command");
        dev.in_submitted(6, 512);
        assert_eq!(dev.transfer_in(6, 512, &mut v), Reply::Nak, "it stopped the acquisition");
        dev.control(cmd(COUPLING), &[0x11], &mut v);
        assert_eq!(v.drain().len(), 1, "coupling on the 6022BL");
        dev.control(cmd(VDIV_CH2), &[11], &mut v);
        assert_eq!(v.drain().len(), 1, "gain 11 is the ISDS205B's");
        dev.control(cmd(SAMPLERATE), &[12], &mut v);
        assert_eq!(v.drain().len(), 1, "12 MHz: the firmware's, not the driver's");
        dev.control(cmd(TRIGGER), &[2], &mut v);
        assert_eq!(v.drain().len(), 1, "trigger 2");
        dev.control(cmd(CHANNELS), &[2], &mut v);
        assert_eq!(v.drain().len(), 1, "channels, never sent");
        dev.control(Setup { value: 1, index: 2, ..cmd(VDIV_CH1) }, &[1], &mut v);
        assert_eq!(v.drain().len(), 1, "wValue and wIndex");
        dev.control(Setup { length: 2, ..cmd(VDIV_CH1) }, &[1, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "two bytes");
        dev.control(Setup { request_type: 0xc0, ..cmd(VDIV_CH1) }, &[], &mut v);
        assert_eq!(v.drain().len(), 1, "an IN");
        start(&mut dev, 1, [1, 1], None, &mut v);
        dev.transfer_in(6, 1000, &mut v);
        assert_eq!(v.drain().len(), 1, "not a multiple of 512");
        dev.transfer_in(2, 512, &mut v);
        assert!(!v.drain().is_empty(), "the wrong endpoint");
        let mut dev = flashed("dds120", &[]);
        dev.control(cmd(COUPLING), &[0x31], &mut v);
        assert_eq!(v.drain().len(), 1, "an undefined coupling bit");
        dev.control(Setup { request_type: 0x41, ..cmd(SAMPLERATE) }, &[1], &mut v);
        assert_eq!(v.drain().len(), 1, "to the interface");
        start(&mut dev, 1, [1, 1], Some(0x11), &mut v);
        assert!(v.is_empty(), "{v:?}");
        // A second start: init() sends everything again.
        dev.control(cmd(TRIGGER), &[1], &mut v);
        assert_eq!(v.drain().len(), 2, "rate and gains, and coupling, not sent again");
        start(&mut dev, 1, [1, 1], None, &mut v);
        assert_eq!(v.drain().len(), 1, "the coupling not sent on the DDS120");
        for bad in [
            ("profile", "6022"),
            ("flashed", "2"),
            ("stall", "reset"),
            ("stall", "trigger/x"),
            ("short", "512"),
            ("short", "3"),
            ("bcd", "xyz"),
            ("empty", "-1"),
        ] {
            assert!(build(&[(bad.0.into(), bad.1.into())]).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn faults_stall_commands_and_shape_the_stream() {
        let mut v = Violations::default();
        let mut dev = flashed("6022be", &[("stall", "samplerate,trigger/0"), ("bcd", "0x0009")]);
        assert_eq!(dev.descriptors().bcd_device(), 9);
        assert_eq!(dev.control(cmd(SAMPLERATE), &[48], &mut v), Reply::Stall);
        dev.control(cmd(VDIV_CH1), &[1], &mut v);
        dev.control(cmd(VDIV_CH2), &[1], &mut v);
        assert_eq!(dev.control(cmd(TRIGGER), &[1], &mut v), Reply::ack());
        let Reply::Data(d) = dev.transfer_in(6, 512, &mut v) else { panic!() };
        // Still 1 MHz: half a period of 1 kHz is 500 samples.
        assert_eq!((d[0], d[510]), (153, 153));
        assert_eq!(dev.control(cmd(TRIGGER), &[0], &mut v), Reply::Stall);
        dev.in_submitted(6, 512);
        assert!(matches!(dev.transfer_in(6, 512, &mut v), Reply::Data(_)), "still sampling");
        assert!(v.is_empty(), "a fault is the device's, not the host's: {v:?}");

        // The upload's steps: a STALLed one changes nothing.
        let o = [("stall".to_owned(), "release".to_owned())];
        let model = build(&o).unwrap();
        let image = ezusb::standin(FW_6022BE, FIRMWARE_SIZE);
        let mut dev = Device::new(model);
        dev.control(SET_CONFIG, &[], &mut v);
        assert_eq!(dev.control(load(0xe600, 1), &[1], &mut v), Reply::ack());
        assert_eq!(dev.control(load(0, 4096), &image[..4096], &mut v), Reply::ack());
        assert_eq!(dev.control(load(0xe600, 1), &[0], &mut v), Reply::Stall);
        assert!(!dev.take_renumeration(), "the 8051 stays in reset");
        let o = [("stall".to_owned(), "hold,chunk".to_owned())];
        let mut dev = Device::new(build(&o).unwrap());
        dev.control(SET_CONFIG, &[], &mut v);
        assert_eq!(dev.control(load(0xe600, 1), &[1], &mut v), Reply::Stall);
        assert_eq!(dev.control(load(0, 4096), &image[..4096], &mut v), Reply::Stall);
        assert!(v.is_empty(), "{v:?}");

        let mut dev = flashed("6022be", &[("empty", "1"), ("stall_in", "1"), ("short", "300")]);
        start(&mut dev, 10, [1, 1], None, &mut v);
        let mut got = Vec::new();
        for _ in 0..3 {
            got.push(dev.transfer_in(6, 1024, &mut v));
            dev.in_submitted(6, 1024);
        }
        assert_eq!(got[..2], [Reply::Data(vec![]), Reply::Stall]);
        let Reply::Data(d) = &got[2] else { panic!() };
        assert_eq!(d.len(), 300);
        assert!(v.is_empty(), "{v:?}");
    }

    /// `slow=N`: the first N transfers complete, the next is held until
    /// the host cancels it, and comes with data once per submission.
    #[test]
    fn a_slow_transfer_holds_its_data_until_cancelled() {
        let mut v = Violations::default();
        let mut dev = flashed("6022be", &[("slow", "1")]);
        start(&mut dev, 10, [1, 1], None, &mut v);
        assert_eq!(dev.in_unlinked(6, 1024), None, "the first one is not held");
        let Reply::Data(d) = dev.transfer_in(6, 1024, &mut v) else { panic!() };
        assert_eq!(d.len(), 1024);
        dev.in_submitted(6, 1024);
        assert_eq!(dev.transfer_in(6, 1024, &mut v), Reply::Nak);
        assert_eq!(dev.in_unlinked(6, 1024).map(|d| d.len()), Some(SLOW_BYTES));
        assert_eq!(dev.in_unlinked(6, 1024), None, "one per submission");
        dev.in_submitted(6, 1024);
        assert_eq!(dev.in_unlinked(6, 1024).map(|d| d.len()), Some(SLOW_BYTES));
        assert!(v.is_empty(), "{v:?}");
    }
}
