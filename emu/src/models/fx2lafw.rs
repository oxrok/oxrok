// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An FX2 logic analyzer with the sigrok `fx2lafw` firmware, as
//! libsigrok's `fx2lafw` driver sees it, built on the EZ-USB shell
//! ([`crate::shells::ezusb`]).
//!
//! Four profiles of the driver's table: `cypress-fx2`, the bare
//! CY7C68013A board (`04b4:8613`, 16 channels, firmware
//! `fx2lafw-cypress-fx2.fw`), `saleae-logic`, the Saleae Logic and
//! its clones (`0925:3881` from a `0xC0` EEPROM, 8 channels,
//! `fx2lafw-saleae-logic.fw`), `sigrok-fx2-8ch`, sigrok's own
//! fx2grok-flat (`1d50:608c` before and after the upload, 8 channels,
//! `fx2lafw-sigrok-fx2-8ch.fw`), and `usbee-ax`, the CWAV `USBee AX` and
//! its clones (`08a9:0014`, 8 channels and an analog input,
//! `fx2lafw-cwav-usbeeax.fw`, string 3 `CWAV USBee AX` from
//! `hw/cwav-usbeeax/dscr.a51`). Without `flashed=1` the device comes up
//! as the FX2 without firmware (no strings), takes the upload through
//! the boot loader and renumerates; with it, it comes up running the
//! firmware already, as after an earlier upload.
//!
//! Once running, the firmware's descriptors are those of `dscr.inc`:
//! the same ids, `bcdDevice` 0x0001 (`.dw 0x0100`, and `.dw` is big
//! endian there, as the swapped `VID` and `PID` of `hw/*/dscr.a51`
//! show), strings `sigrok`, `fx2lafw` and the board's name as string 3
//! (`hw/<board>/dscr.a51`: `Cypress FX2`, `Saleae Logic`, `sigrok FX2
//! 8ch`), which the driver takes for the serial number; interface 0
//! with one bulk IN endpoint 2 of 512 bytes, at high speed, 100 mA.
//! Its vendor requests are those of `fx2lafw.c` (checked against
//! sigrok-firmware-fx2lafw 0.1.7):
//!
//! | request | data | answer |
//! |---|---|---|
//! | `0xb0` IN | 2 bytes | firmware version 1.4 |
//! | `0xb1` OUT | 3 bytes: flags, delay high, delay low | starts the acquisition |
//! | `0xb2` IN | 1 byte | `REVID`, 1 on the FX2LP |
//!
//! The start flags are bit 4 CTL2 clock (analog boards only), bit 5
//! 16 bit samples, bit 6 the 48 MHz clock (30 MHz without); the GPIF
//! takes a sample every `delay + 1` clocks (`gpif_acquisition_prepare()`:
//! `delay >> 8` states of 256 clocks, one of `delay & 0xff`, and the
//! decision point; the low byte is never 0 for a rate of the list but
//! 48 MHz, which has no delay at all). A start of another length is
//! dropped, a high byte of 6 or more refused, and the GPIF starts at
//! the first bulk IN after the start (`ibn_isr()`). Any other vendor
//! request makes `handle_vendorcommand()` return FALSE: a STALL. The
//! firmware itself looks at neither `wValue`, `wIndex` nor `wLength`;
//! the model holds them to what the driver sends. The samples are a counter
//! from 0 at each start, one byte per sample (two, little endian, with
//! 16 bit samples), so channel `Dn` toggles every 2^n samples.
//!
//! The real firmware streams as fast as the GPIF samples, and every
//! queued transfer fills in turn; how many complete before the host
//! cancels the rest then depends on timing. The model does not do
//! that: as an artifact of the emulation, it completes one bulk IN
//! transfer per transfer the host submits after the previous one
//! completed (the first right after the start), so the transactions
//! of a run do not depend on timing and two runs can be compared. The
//! other queued transfers wait meanwhile; the cases stay far below the
//! driver's per-transfer timeout (at least about 400 ms).
//!
//! The `USBee AX`'s analog input, and where each fact comes from:
//!
//! - The profile has `DEV_CAPS_AX_ANALOG` (`api.c`, the `08a9:0014`
//!   entry), which gives it one analog channel, `A0`, after D0 to D7,
//!   in a channel group of its own (`scan()`).
//! - With `A0` enabled, `configure_channels()` (`protocol.c`) makes the
//!   samples wide, and `command_start_acquisition()` sets
//!   `CMD_START_FLAGS_SAMPLE_16BIT` and `CMD_START_FLAGS_CLK_CTL2`
//!   (`0x30` with the 30 MHz clock, `0x70` with 48 MHz), and refuses a
//!   rate above `MAX_16BIT_SAMPLE_RATE`, 12 MHz (`protocol.h`).
//! - The firmware (`gpif_acquisition_prepare()`, `gpif-acquisition.c`)
//!   then makes EP2 word wide (`bmWORDWIDE`) and drives CTL1 and CTL2
//!   high in the delay states before the last (`0x06`) and low in the
//!   last: one pulse per sample, the ADC's clock. It needs a low delay
//!   byte other than 0 for that (`sample_delay_l` "is always != 0 for the
//!   supported rates").
//! - `mso_send_data_proc()` (`protocol.c`) reads each 16 bit sample as
//!   the logic byte first and the ADC's byte second (`data[i * 2]`,
//!   `data[i * 2 + 1]`, as an FX2 word wide FIFO sends `FD[7:0]` before
//!   `FD[15:8]`), and rescales the ADC's byte as `(code - 128) / 12.8`
//!   V, "-10V - +10V from 0-255". Which port pins the ADC drives and
//!   its input range are the C driver's view; no schematic is in the
//!   source.
//!
//! The model streams, with CTL2 and 16 bit samples on this board, the
//! counter's low byte as the logic byte and the ADC code
//! `128 + n + n / 256` (modulo 256) for sample `n`: a ramp that reads
//! `n / 12.8` V for the first 128 samples (5 V at sample 64), -10 V at
//! 128, each block of 256 samples one step above the last. Without
//! CTL2 it streams 8 bit samples as the other 8 channel boards.
//!
//! What the model taught about the C driver: its soft trigger takes
//! the sample width from all logic channels of the device
//! (`logic_channel_unitsize(sdi->channels)`, two bytes on a 16 channel
//! board) while the stream has one byte per sample when only D0 to D7
//! are enabled, so a trigger on such a capture matches every other
//! byte against the wrong channels and starts the data at half the
//! offset. oxrok's port uses the stream's width; the cases trigger on
//! 8 channel boards, or on the 16 channel one with every channel. The
//! same on the `USBee AX` with `A0` enabled: one byte per sample (its 8
//! logic channels) on a stream of two, so the trigger looks at the
//! ADC's bytes as logic samples too, starts the data at twice the
//! offset, and sends the pre-trigger samples as logic packets of the
//! interleaved bytes. A trigger on `A0` itself is taken for bit 8 of
//! the logic stream (`logic_check_match()` reads
//! `*(sample + index / 8)`), which on one byte samples is the next
//! byte; oxrok refuses it. sigrok-cli exits 0 in both, so the cases
//! are `--oracle-differs` ones: oxrok pinned, sigrok-cli's output kept.
//! With `probe_names` starting with `-`, `scan()` names all 8 logic
//! channels from a list that holds only the names given, and
//! sigrok-cli crashes (an `--oracle-bug` case).
//!
//! The pedantic checks follow `protocol.c` and the firmware: the
//! upload byte for byte as `ezusb.c` sends it, of the profile's
//! firmware file; vendor requests other than the three, or with other
//! lengths, values or indexes; start flags outside the three, CTL2 on a
//! board without analog input, CTL2 without 16 bit samples or with a
//! low delay byte of 0, 16 bit samples on an 8 channel board (on the
//! AX: without CTL2) or above 12 MHz, a delay the firmware refuses (high byte 6 or more), a
//! rate the driver's list lacks or encoded with the 30 MHz clock where
//! the driver would take 48 MHz; bulk IN transfers that are not a
//! multiple of 512 bytes; a firmware upload to the running firmware.
//!
//! Faults, all off by default, make the board misbehave as a real one
//! can, to reach the driver's error paths: `version=MAJOR.MINOR`
//! answers another firmware version, `stall=version,start,revid` STALLs
//! those requests, `empty=N` completes the first N bulk IN transfers of
//! each acquisition with no data and `stall_in=N` STALLs the N after
//! those, both paced like the samples (one per submission), so the
//! number of transactions stays that of the driver's logic.
//! `stall_string=I,...` STALLs every read of string descriptor I (0
//! the language table, 1 to 3 the manufacturer, product and serial
//! number) and `short_string=I,...` answers it with its first two bytes
//! alone, a `bLength` the answer does not hold, which libusb takes for
//! an I/O error: the string reads of `scan()` (`api.c`) fail. They do
//! not change the descriptors, so turning them on re-attaches nothing
//! and the kernel keeps the strings it read at the attach.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{
    Descriptors, DeviceModel, Reply, Setup, Speed, StandardFault, VENDOR, Violations,
};
use crate::shells::ezusb::{self, Chip, Loader};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "fx2lafw";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "FX2 logic analyzer with the fx2lafw firmware (EZ-USB, renumerates); \
                         profile=cypress-fx2 (04b4:8613, 16 ch, default), saleae-logic \
                         (0925:3881, 8 ch), sigrok-fx2-8ch (1d50:608c, 8 ch) or usbee-ax (08a9:0014, 8 ch and an \
                         analog input), flashed=1 starts with the firmware running; \
                         faults: version=MAJOR.MINOR, stall=version,start,revid, empty=N, stall_in=N, \
                         stall_string=I,..., short_string=I,...";

/// A board the driver knows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// The option value.
    pub key: &'static str,
    /// Vendor id, before and after the upload.
    pub vendor: u16,
    /// Product id, before and after the upload.
    pub product: u16,
    /// The firmware file the driver uploads.
    pub firmware: &'static str,
    /// Its size, as `tools/emu/fx2lafw.firmware` lists it.
    pub firmware_size: usize,
    /// String 3 of the firmware.
    pub board: &'static str,
    /// 16 channels.
    pub wide: bool,
    /// The analog input of the `USBee AX` (`DEV_CAPS_AX_ANALOG`): an ADC
    /// read as the high byte of 16 bit samples, clocked by CTL2.
    pub analog: bool,
}

/// The emulated boards.
pub const PROFILES: [Profile; 4] = [
    Profile {
        key: "cypress-fx2",
        vendor: ezusb::CYPRESS_VID,
        product: ezusb::FX2_PID,
        firmware: "fx2lafw-cypress-fx2.fw",
        firmware_size: 7690,
        board: "Cypress FX2",
        wide: true,
        analog: false,
    },
    Profile {
        key: "saleae-logic",
        vendor: 0x0925,
        product: 0x3881,
        firmware: "fx2lafw-saleae-logic.fw",
        firmware_size: 7642,
        board: "Saleae Logic",
        wide: false,
        analog: false,
    },
    Profile {
        key: "sigrok-fx2-8ch",
        vendor: 0x1d50,
        product: 0x608c,
        firmware: "fx2lafw-sigrok-fx2-8ch.fw",
        firmware_size: 7570,
        board: "sigrok FX2 8ch",
        wide: false,
        analog: false,
    },
    Profile {
        key: "usbee-ax",
        vendor: 0x08a9,
        product: 0x0014,
        firmware: "fx2lafw-cwav-usbeeax.fw",
        firmware_size: 7618,
        board: "CWAV USBee AX",
        wide: false,
        analog: true,
    },
];

/// Ways the board can misbehave, as a real one can; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    /// The firmware version `0xb0` answers, when not [`VERSION`].
    pub version: Option<[u8; 2]>,
    /// Vendor requests the firmware STALLs: `0xb0`, `0xb1`, `0xb2`.
    pub stall: Vec<u8>,
    /// Bulk IN transfers of each acquisition that complete with no
    /// data before the samples come.
    pub empty: u32,
    /// Bulk IN transfers of each acquisition that STALL before the
    /// samples come.
    pub stall_in: u32,
    /// String descriptors whose reads STALL.
    pub stall_string: Vec<u8>,
    /// String descriptors whose reads answer two bytes alone.
    pub short_string: Vec<u8>,
}

impl Faults {
    /// Takes the option `k=v` when it is a fault; false when `k` is no
    /// fault option.
    fn parse(&mut self, k: &str, v: &str) -> Result<bool> {
        let count = |v: &str| v.parse::<u32>().map_err(|_| Error(format!("{k}={v}: a count")));
        match k {
            "version" => {
                let bad = || Error(format!("version={v}: MAJOR.MINOR, each 0 to 255"));
                let (major, minor) = v.split_once('.').ok_or_else(bad)?;
                let major = major.parse().map_err(|_| bad())?;
                let minor = minor.parse().map_err(|_| bad())?;
                self.version = Some([major, minor]);
            }
            "stall" => {
                self.stall.clear();
                for name in v.split(',').filter(|n| !n.is_empty()) {
                    self.stall.push(match name {
                        "version" => CMD_GET_FW_VERSION,
                        "start" => CMD_START,
                        "revid" => CMD_GET_REVID_VERSION,
                        _ => return Err(Error(format!("stall={name}: version, start or revid"))),
                    });
                }
            }
            "empty" => self.empty = count(v)?,
            "stall_in" => self.stall_in = count(v)?,
            "stall_string" | "short_string" => {
                let mut list = Vec::new();
                for i in v.split(',').filter(|i| !i.is_empty()) {
                    list.push(i.parse::<u8>().ok().filter(|i| *i <= 3).ok_or_else(|| {
                        Error(format!("{k}={i}: a string index, 0 to 3"))
                    })?);
                }
                if k == "stall_string" {
                    self.stall_string = list;
                } else {
                    self.short_string = list;
                }
            }
            _ => return Ok(false),
        }
        Ok(true)
    }
}

/// A board from the `-o` options: `profile=cypress-fx2|saleae-logic|sigrok-fx2-8ch|usbee-ax`,
/// `flashed=0|1`, and the faults (see [`Faults`]): `version=MAJOR.MINOR`,
/// `stall=version,start,revid`, `empty=N`, `stall_in=N`, `stall_string=I,...`,
/// `short_string=I,...`.
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
    let mut model = Fx2lafw::new(profile, flashed);
    model.faults = faults;
    Ok(Box::new(model))
}

const CMD_GET_FW_VERSION: u8 = 0xb0;
const CMD_START: u8 = 0xb1;
const CMD_GET_REVID_VERSION: u8 = 0xb2;
/// The firmware version, `fx2lafw.h`.
pub const VERSION: [u8; 2] = [1, 4];
/// `REVID` of a CY7C68013A.
pub const REVID: u8 = 1;
const FLAG_CTL2: u8 = 1 << 4;
const FLAG_WIDE: u8 = 1 << 5;
const FLAG_48MHZ: u8 = 1 << 6;
/// 6 delay states of up to 256 clocks, `protocol.h`.
const MAX_SAMPLE_DELAY: u32 = 6 * 256;
/// The bulk endpoint's packet size at high speed.
const PACKET: usize = 512;

/// The rates of the driver's list, `api.c`.
pub const SAMPLERATES: [u32; 17] = [
    20_000, 25_000, 50_000, 100_000, 200_000, 250_000, 500_000, 1_000_000, 2_000_000, 3_000_000,
    4_000_000, 6_000_000, 8_000_000, 12_000_000, 16_000_000, 24_000_000, 48_000_000,
];

/// The descriptors of the running firmware, `dscr.inc`.
pub fn firmware_descriptors(p: &Profile) -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 1, [0xff, 0xff, 0xff], 0));
    body.extend_from_slice(&descriptor::endpoint(0x82, crate::model::BULK, 512, 0));
    let config = descriptor::configuration(1, 0, 0x80, 100, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0xff, 0xff, 0xff],
        max_packet0: 64,
        vendor: p.vendor,
        product: p.product,
        // dscr.inc's `.dw 0x0100` is big endian: bcdDevice 0x0001, as a
        // Saleae clone with fx2lafw 1.4 reports it.
        device: 0x0001,
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

/// A running acquisition.
#[derive(Clone, Debug)]
struct Acquisition {
    wide: bool,
    /// The high byte of each sample is the ADC's (the `USBee AX`, CTL2).
    adc: bool,
    /// The next sample's number.
    sample: u64,
    /// Whether the next bulk IN transfer may complete.
    ready: bool,
    /// Faulty transfers still to come: empty ones, then stalled ones.
    empty: u32,
    stall_in: u32,
}

/// What the firmware is doing.
#[derive(Clone, Debug)]
enum Stage {
    /// The boot loader, waiting for the firmware.
    Boot(Box<Loader>),
    /// The firmware runs.
    Running(Option<Acquisition>),
}

/// The emulated board.
pub struct Fx2lafw {
    profile: Profile,
    boot: Descriptors,
    firmware: Descriptors,
    stage: Stage,
    renumerate: bool,
    faults: Faults,
}

impl Fx2lafw {
    /// A board; `flashed` starts it with the firmware running.
    pub fn new(profile: Profile, flashed: bool) -> Fx2lafw {
        let stage =
            if flashed { Stage::Running(None) } else { Stage::Boot(Box::new(Loader::new(Chip::Fx2lp))) };
        Fx2lafw {
            boot: ezusb::bare_descriptors(profile.vendor, profile.product, ezusb::FX2LP_DID),
            firmware: firmware_descriptors(&profile),
            profile,
            stage,
            renumerate: false,
            faults: Faults::default(),
        }
    }

    /// Whether the firmware runs.
    pub fn running(&self) -> bool {
        matches!(self.stage, Stage::Running(_))
    }

    /// The firmware image both tools must upload.
    pub fn image(&self) -> Vec<u8> {
        ezusb::standin(self.profile.firmware, self.profile.firmware_size)
    }

    /// The sample `n` of the stream.
    fn sample(n: u64, wide: bool, adc: bool) -> u16 {
        if adc {
            u16::from_le_bytes([n as u8, Self::adc(n)])
        } else if wide {
            n as u16
        } else {
            u16::from(n as u8)
        }
    }

    /// The ADC code of sample `n` on the `USBee AX`: `128 + n + n / 256`,
    /// modulo 256. The driver's `(code - 128) / 12.8` V makes it a ramp
    /// of 0.078125 V a sample from 0 V: sample `n` reads `n / 12.8` V up
    /// to 127 (5 V at 64, 9.921875 V at 127), -10 V at 128, and each
    /// block of 256 samples starts one step higher than the one before
    /// (sample 256 reads 0.078125 V), so that a transfer (always a
    /// multiple of 256 samples of two bytes) out of place shows.
    pub fn adc(n: u64) -> u8 {
        (n as u8).wrapping_add((n >> 8) as u8).wrapping_add(128)
    }

    fn boot_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
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
            // The firmware disconnects and comes back as itself.
            self.stage = Stage::Running(None);
            self.renumerate = true;
        }
        reply
    }

    fn start(&mut self, data: &[u8], v: &mut Violations) -> Option<Acquisition> {
        if data.len() != 3 {
            // fx2lafw_poll() ignores a start of another length.
            v.report(format!("start command of {} bytes; it has 3", data.len()));
            return None;
        }
        let (flags, delay) = (data[0], u32::from(u16::from_be_bytes([data[1], data[2]])));
        if flags & !(FLAG_CTL2 | FLAG_WIDE | FLAG_48MHZ) != 0 {
            v.report(format!("start flags {flags:#04x}: only bits 4, 5 and 6 are defined"));
        }
        let ctl2 = flags & FLAG_CTL2 != 0;
        if ctl2 && !self.profile.analog {
            v.report("start with the CTL2 clock, which only boards with analog input use");
        }
        if ctl2 && self.profile.analog {
            // command_start_acquisition() sets CTL2 when an analog
            // channel is enabled, and configure_channels() makes the
            // samples wide then: the ADC is the high byte.
            if flags & FLAG_WIDE == 0 {
                v.report("CTL2 clock with 8 bit samples; the ADC is read as the high byte of 16");
            }
            // gpif_acquisition_prepare(): "sample_delay_l is always != 0
            // for the supported rates"; the CTL1/2 pulse needs it.
            if data[2] == 0 {
                v.report(format!(
                    "CTL2 clock with sample delay {delay}: the low byte must not be 0"
                ));
            }
        }
        if data[1] >= 6 {
            // gpif_acquisition_prepare() refuses it: no acquisition.
            v.report(format!("sample delay {delay}: the firmware takes at most {}", 6 * 256 - 1));
            return None;
        }
        let wide = flags & FLAG_WIDE != 0;
        let clock = if flags & FLAG_48MHZ != 0 { 48_000_000 } else { 30_000_000 };
        let rate = clock / (delay + 1);
        if clock % (delay + 1) != 0 || !SAMPLERATES.contains(&rate) {
            v.report(format!(
                "start at {clock} Hz / {}: {rate} Hz is not one of the driver's rates",
                delay + 1
            ));
        } else if clock == 30_000_000
            && 48_000_000 % rate == 0
            && 48_000_000 / rate - 1 <= MAX_SAMPLE_DELAY
            && 48_000_000 / rate - 1 != 0
        {
            v.report(format!("{rate} Hz from the 30 MHz clock; the driver takes 48 MHz for it"));
        }
        if wide && !self.profile.wide && !self.profile.analog {
            v.report("16 bit samples on an 8 channel board");
        }
        if wide && self.profile.analog && !ctl2 {
            // Only an enabled analog channel makes the AX's samples wide.
            v.report("16 bit samples without the CTL2 clock on the USBee AX: no ADC sample");
        }
        if wide && rate > 12_000_000 {
            v.report(format!("16 bit samples at {rate} Hz; the driver allows 12 MHz at most"));
        }
        Some(Acquisition {
            wide,
            adc: wide && ctl2 && self.profile.analog,
            sample: 0,
            ready: true,
            empty: self.faults.empty,
            stall_in: self.faults.stall_in,
        })
    }

    fn firmware_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() == VENDOR && setup.request == ezusb::REQ_FIRMWARE_LOAD {
            // The boot loader is in silicon and still answers, but the
            // driver never uploads to a device with the firmware.
            v.report("firmware load to an FX2 whose fx2lafw firmware runs");
            return Reply::ack();
        }
        let request = (setup.request_type, setup.request);
        let length = match request {
            (0xc0, CMD_GET_FW_VERSION) => 2,
            (0xc0, CMD_GET_REVID_VERSION) => 1,
            (0x40, CMD_START) => 3,
            _ => {
                // handle_vendorcommand() returns FALSE: a STALL.
                v.report(format!(
                    "request {:#04x} {:#04x} value {:#06x}: not one fx2lafw answers",
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
        if setup.length != length {
            v.report(format!(
                "request {:#04x} with wLength {}; the driver uses {length}",
                setup.request, setup.length
            ));
        }
        if self.faults.stall.contains(&setup.request) {
            // As handle_vendorcommand() returning FALSE: fx2lib STALLs.
            return Reply::Stall;
        }
        match setup.request {
            CMD_GET_FW_VERSION => Reply::Data(self.faults.version.unwrap_or(VERSION).to_vec()),
            CMD_GET_REVID_VERSION => Reply::Data(vec![REVID]),
            _ => {
                let acq = self.start(data, v);
                if acq.is_some() {
                    self.stage = Stage::Running(acq);
                }
                Reply::ack()
            }
        }
    }
}

impl DeviceModel for Fx2lafw {
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
        v.report(format!("OUT transfer to endpoint {ep}; fx2lafw has none"));
        Reply::Stall
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        let Stage::Running(Some(acq)) = &mut self.stage else {
            // No acquisition: the FIFO stays empty and the host waits.
            return Reply::Nak;
        };
        if ep != 2 {
            v.report(format!("IN transfer from endpoint {ep}; the samples come from 2"));
            return Reply::Stall;
        }
        if !acq.ready {
            return Reply::Nak;
        }
        if !max.is_multiple_of(PACKET) {
            v.report(format!(
                "bulk IN transfer of {max} bytes; the driver asks for multiples of {PACKET}"
            ));
        }
        // The faults, paced like the samples: one per submission.
        if acq.empty > 0 {
            acq.empty -= 1;
            acq.ready = false;
            return Reply::Data(Vec::new());
        }
        if acq.stall_in > 0 {
            acq.stall_in -= 1;
            acq.ready = false;
            return Reply::Stall;
        }
        let unit = if acq.wide { 2 } else { 1 };
        let mut data = Vec::with_capacity(max);
        for _ in 0..max / unit {
            let s = Self::sample(acq.sample, acq.wide, acq.adc).to_le_bytes();
            data.extend_from_slice(&s[..unit]);
            acq.sample += 1;
        }
        acq.ready = false;
        Reply::Data(data)
    }

    fn in_submitted(&mut self, ep: u8, _max: usize) {
        if let Stage::Running(Some(acq)) = &mut self.stage
            && ep == 2
        {
            acq.ready = true;
        }
    }

    fn reset(&mut self) {
        // A new attach ends an acquisition; the firmware stays in RAM.
        if let Stage::Running(acq) = &mut self.stage {
            *acq = None;
        }
    }

    /// Firmware chunks of 4 KiB and transfers of up to 240 KiB.
    fn log_digest_above(&self) -> Option<usize> {
        Some(crate::txlog::HASH_ABOVE)
    }

    fn take_renumeration(&mut self) -> bool {
        std::mem::take(&mut self.renumerate)
    }

    /// `stall_string` and `short_string`: `GET_DESCRIPTOR` of a string.
    fn fail_standard(&mut self, setup: Setup) -> Option<StandardFault> {
        const GET_DESCRIPTOR: u8 = 6;
        if setup.request_type != 0x80
            || setup.request != GET_DESCRIPTOR
            || setup.value >> 8 != u16::from(descriptor::STRING)
        {
            return None;
        }
        let index = (setup.value & 0xff) as u8;
        if self.faults.stall_string.contains(&index) {
            Some(StandardFault::Stall)
        } else if self.faults.short_string.contains(&index) {
            Some(StandardFault::Short(2))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Device;

    fn vendor(request_type: u8, request: u8, value: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index: 0, length }
    }

    /// The upload as `ezusb_upload_firmware()` sends it.
    fn upload(dev: &mut Device, image: &[u8], v: &mut Violations) {
        let set_config = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };
        assert_eq!(dev.control(set_config, &[], v), Reply::ack());
        assert_eq!(dev.control(vendor(0x40, 0xa0, 0xe600, 1), &[1], v), Reply::ack());
        for (i, c) in image.chunks(4096).enumerate() {
            let s = vendor(0x40, 0xa0, (i * 4096) as u16, c.len() as u16);
            assert_eq!(dev.control(s, c, v), Reply::ack());
        }
        assert_eq!(dev.control(vendor(0x40, 0xa0, 0xe600, 1), &[0], v), Reply::ack());
    }

    #[test]
    fn the_standin_sizes_are_those_of_the_firmware_list() {
        let list = include_str!("../../../tools/emu/fx2lafw.firmware");
        for p in PROFILES {
            let line = list
                .lines()
                .find(|l| l.split_whitespace().next() == Some(p.firmware))
                .unwrap_or_else(|| panic!("{} is not in tools/emu/fx2lafw.firmware", p.firmware));
            let size: usize = line.split_whitespace().nth(1).unwrap().parse().unwrap();
            assert_eq!(size, p.firmware_size, "{}", p.firmware);
        }
    }

    #[test]
    fn uploads_renumerates_and_streams_as_the_driver_drives_it() {
        let model = Fx2lafw::new(PROFILES[0], false);
        let image = model.image();
        let mut dev = Device::new(Box::new(model));
        let mut v = Violations::default();
        assert_eq!(dev.descriptors().strings.len(), 0, "a bare FX2 has no strings");
        upload(&mut dev, &image, &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert!(dev.take_renumeration());
        assert!(!dev.take_renumeration(), "once per renumeration");
        assert!(dev.needs_reenumeration());
        // The new attach: the firmware's descriptors.
        dev.reset();
        assert_eq!(dev.descriptors().strings, ["sigrok", "fx2lafw", "Cypress FX2"]);
        assert_eq!(dev.descriptors().device[12..14], [0x01, 0x00], "bcdDevice 0x0001");
        let set_config = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };
        dev.control(set_config, &[], &mut v);
        assert_eq!(dev.control(vendor(0xc0, 0xb0, 0, 2), &[], &mut v), Reply::Data(vec![1, 4]));
        assert_eq!(dev.control(vendor(0xc0, 0xb2, 0, 1), &[], &mut v), Reply::Data(vec![1]));
        // Transfers submitted before the start wait.
        dev.in_submitted(2, 512);
        dev.in_submitted(2, 512);
        assert_eq!(dev.transfer_in(2, 512, &mut v), Reply::Nak);
        // 1 MHz from 48 MHz, 16 bit samples.
        assert_eq!(dev.control(vendor(0x40, 0xb1, 0, 3), &[0x60, 0, 47], &mut v), Reply::ack());
        let Reply::Data(d) = dev.transfer_in(2, 512, &mut v) else { panic!() };
        assert_eq!(&d[..6], &[0, 0, 1, 0, 2, 0]);
        // The next one waits for the host to submit again.
        assert_eq!(dev.transfer_in(2, 512, &mut v), Reply::Nak);
        dev.in_submitted(2, 512);
        let Reply::Data(d) = dev.transfer_in(2, 512, &mut v) else { panic!() };
        assert_eq!(&d[..2], &[0, 1], "sample 256 goes on");
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn eight_bit_samples_and_the_flashed_start() {
        let mut dev = Device::new(Box::new(Fx2lafw::new(PROFILES[1], true)));
        let mut v = Violations::default();
        assert_eq!(dev.descriptors().product_id(), 0x3881);
        assert_eq!(dev.descriptors().strings[2], "Saleae Logic");
        let set_config = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };
        dev.control(set_config, &[], &mut v);
        // 20 kHz: 30 MHz / 1500.
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x00, 0x05, 0xdb], &mut v);
        let Reply::Data(d) = dev.transfer_in(2, 1024, &mut v) else { panic!() };
        assert_eq!(d.len(), 1024);
        assert_eq!((d[0], d[255], d[256]), (0, 255, 0));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_what_the_driver_and_the_firmware_forbid() {
        let mut v = Violations::default();
        // Bare chip: anything but the loader, and a wrong upload.
        let model = Fx2lafw::new(PROFILES[0], false);
        let mut image = model.image();
        let mut dev = Device::new(Box::new(model));
        assert_eq!(dev.control(vendor(0xc0, 0xb0, 0, 2), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "vendor request to the boot loader");
        image[100] ^= 0xff;
        upload(&mut dev, &image, &mut v);
        assert_eq!(v.drain().len(), 1, "the image differs from the stand-in");
        dev.reset();
        let set_config = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };
        dev.control(set_config, &[], &mut v);
        dev.control(vendor(0x40, 0xa0, 0xe600, 1), &[1], &mut v);
        assert_eq!(v.drain().len(), 1, "upload to the running firmware");
        assert_eq!(dev.control(vendor(0xc0, 0xb3, 0, 1), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "unknown command");
        dev.control(vendor(0xc0, 0xb0, 0, 4), &[], &mut v);
        assert_eq!(v.drain().len(), 1, "version of 4 bytes");
        dev.control(vendor(0x40, 0xb1, 0, 2), &[0x40, 0], &mut v);
        assert_eq!(v.drain().len(), 2, "short start (wLength and length)");
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x40, 6, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "delay the firmware refuses");
        assert_eq!(dev.transfer_in(2, 512, &mut v), Reply::Nak, "no acquisition");
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x00, 0, 29], &mut v);
        assert_eq!(v.drain().len(), 1, "1 MHz from 30 MHz, where the driver takes 48");
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x40, 0, 6], &mut v);
        assert_eq!(v.drain().len(), 1, "48 / 7 MHz is no rate of the list");
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x60, 0, 1], &mut v);
        assert_eq!(v.drain().len(), 1, "16 bit at 24 MHz");
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x51, 0, 1], &mut v);
        assert_eq!(v.drain().len(), 2, "an undefined flag and CTL2");
        dev.transfer_in(2, 500, &mut v);
        assert_eq!(v.drain().len(), 1, "not a multiple of 512");
        let mut dev = Device::new(Box::new(Fx2lafw::new(PROFILES[1], true)));
        dev.control(set_config, &[], &mut v);
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x60, 0, 47], &mut v);
        assert_eq!(v.drain().len(), 1, "16 bit samples on the 8 channel board");
        assert!(build(&[("profile".into(), "nope".into())]).is_err());
        assert!(build(&[("flashed".into(), "2".into())]).is_err());
    }

    #[test]
    fn the_usbee_ax_streams_logic_and_adc_bytes_with_ctl2() {
        let mut v = Violations::default();
        let model = Fx2lafw::new(PROFILES[3], false);
        let image = model.image();
        let mut dev = Device::new(Box::new(model));
        assert_eq!(dev.descriptors().product_id(), 0x0014);
        upload(&mut dev, &image, &mut v);
        dev.reset();
        assert_eq!(dev.descriptors().strings[2], "CWAV USBee AX");
        let set_config = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };
        dev.control(set_config, &[], &mut v);
        // 1 MHz from 48 MHz, 16 bit samples, CTL2: A0 enabled.
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x70, 0, 47], &mut v);
        let Reply::Data(d) = dev.transfer_in(2, 1024, &mut v) else { panic!() };
        assert_eq!(&d[..8], &[0, 128, 1, 129, 2, 130, 3, 131], "logic, then the ADC");
        assert_eq!((d[2 * 64 + 1], d[2 * 127 + 1], d[2 * 128 + 1]), (192, 255, 0));
        assert_eq!((d[2 * 256], d[2 * 256 + 1]), (0, 129), "a block starts one step higher");
        assert!(v.is_empty(), "{v:?}");
        // Without CTL2 the AX streams 8 bit samples, as the other boards.
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x40, 0, 47], &mut v);
        let Reply::Data(d) = dev.transfer_in(2, 512, &mut v) else { panic!() };
        assert_eq!(&d[..3], &[0, 1, 2]);
        assert!(v.is_empty(), "{v:?}");
        // What the driver never sends: CTL2 with 8 bit samples, 16 bit
        // samples without CTL2, a CTL2 delay with a low byte of 0.
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x50, 0, 47], &mut v);
        assert_eq!(v.drain().len(), 1, "CTL2 with 8 bit samples");
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x60, 0, 47], &mut v);
        assert_eq!(v.drain().len(), 1, "16 bit samples without CTL2");
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x30, 1, 0], &mut v);
        assert_eq!(v.drain().len(), 2, "low delay byte 0, and 30 MHz / 257 is no rate");
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x70, 0, 1], &mut v);
        assert_eq!(v.drain().len(), 1, "16 bit samples at 24 MHz");
    }

    fn faulty(options: &[(&str, &str)]) -> Device {
        let mut o: Vec<(String, String)> = vec![("flashed".into(), "1".into())];
        o.extend(options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())));
        let mut dev = Device::new(build(&o).unwrap());
        let set_config = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };
        dev.control(set_config, &[], &mut Violations::default());
        dev
    }

    #[test]
    fn faults_stall_requests_and_answer_another_version() {
        let mut v = Violations::default();
        let mut dev = faulty(&[("stall", "version,revid"), ("version", "2.7")]);
        assert_eq!(dev.control(vendor(0xc0, 0xb0, 0, 2), &[], &mut v), Reply::Stall);
        assert_eq!(dev.control(vendor(0xc0, 0xb2, 0, 1), &[], &mut v), Reply::Stall);
        assert_eq!(dev.control(vendor(0x40, 0xb1, 0, 3), &[0x40, 0, 47], &mut v), Reply::ack());
        let mut dev = faulty(&[("version", "2.7"), ("stall", "start")]);
        assert_eq!(dev.control(vendor(0xc0, 0xb0, 0, 2), &[], &mut v), Reply::Data(vec![2, 7]));
        assert_eq!(dev.control(vendor(0x40, 0xb1, 0, 3), &[0x40, 0, 47], &mut v), Reply::Stall);
        // A refused start starts nothing.
        dev.in_submitted(2, 512);
        assert_eq!(dev.transfer_in(2, 512, &mut v), Reply::Nak);
        assert!(v.is_empty(), "a fault is the device's, not the host's: {v:?}");
        // String reads: the serial number STALLed, the language table
        // cut short; the rest as ever.
        let mut dev = faulty(&[("stall_string", "3"), ("short_string", "0")]);
        let string = |i: u16| Setup {
            request_type: 0x80,
            request: 6,
            value: 0x0300 | i,
            index: if i == 0 { 0 } else { 0x0409 },
            length: 255,
        };
        assert_eq!(dev.control(string(3), &[], &mut v), Reply::Stall);
        assert_eq!(dev.control(string(0), &[], &mut v), Reply::Data(vec![4, 3]));
        let Reply::Data(d) = dev.control(string(1), &[], &mut v) else { panic!() };
        assert_eq!(d.len(), 2 + 2 * "sigrok".len());
        assert!(v.is_empty(), "{v:?}");
        for bad in [
            ("version", "2"),
            ("version", "256.0"),
            ("stall", "reset"),
            ("empty", "-1"),
            ("stall_string", "4"),
            ("short_string", "x"),
        ] {
            assert!(build(&[(bad.0.into(), bad.1.into())]).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn faults_empty_then_stalled_transfers_paced_by_submissions() {
        let mut v = Violations::default();
        let mut dev = faulty(&[("profile", "saleae-logic"), ("empty", "2"), ("stall_in", "1")]);
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x40, 0, 47], &mut v);
        let mut got = Vec::new();
        for _ in 0..4 {
            got.push(dev.transfer_in(2, 512, &mut v));
            assert_eq!(dev.transfer_in(2, 512, &mut v), Reply::Nak, "one per submission");
            dev.in_submitted(2, 512);
        }
        assert_eq!(got[..3], [Reply::Data(vec![]), Reply::Data(vec![]), Reply::Stall]);
        let Reply::Data(d) = &got[3] else { panic!("{:?}", got[3]) };
        assert_eq!(&d[..3], &[0, 1, 2], "the samples start at 0 after the faults");
        assert!(v.is_empty(), "{v:?}");
        // Each acquisition has them again.
        dev.control(vendor(0x40, 0xb1, 0, 3), &[0x40, 0, 47], &mut v);
        assert_eq!(dev.transfer_in(2, 512, &mut v), Reply::Data(vec![]));
    }
}
