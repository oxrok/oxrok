// Counterpart in sigrok:
//   libsigrok/src/hardware/fx2lafw/api.c (the profiles, the scan),
//   libsigrok/src/hardware/fx2lafw/protocol.c (the commands and the transfers),
//   libsigrok/src/hardware/fx2lafw/protocol.h,
//   sigrok-firmware-fx2lafw (fx2lafw.c, gpif-acquisition.c, include/dscr.inc, hw/*/dscr.a51),
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Diego Asanza,
//   Gerhard Sittig, Soeren Apel, Aurelien Jacobs, Daniel Elstner,
//   Martin Ling, Romain Tartière, Angus Gratton, Benjamin Larsson,
//   Carl-Fredrik Sundström, Daniel Gröber, Diego F. Asanza, Jan Losinski,
//   Karl Palsson, Kate J. Temkin, Peetz0r, Thomas, Tilman Sauerbeck,
//   Valentin Ochs, Vlad Ivanov, eightdot, Ralf
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! An FX2 logic analyzer with the sigrok `fx2lafw` firmware, as
//! libsigrok's `fx2lafw` driver sees it, built on the EZ-USB shell
//! ([`crate::shells::ezusb`]).
//!
//! Three profiles of the driver's table: `cypress-fx2`, the bare
//! CY7C68013A board (`04b4:8613`, 16 channels, firmware
//! `fx2lafw-cypress-fx2.fw`), `saleae-logic`, the Saleae Logic and
//! its clones (`0925:3881` from a `0xC0` EEPROM, 8 channels,
//! `fx2lafw-saleae-logic.fw`), and `sigrok-fx2-8ch`, sigrok's own
//! fx2grok-flat (`1d50:608c` before and after the upload, 8 channels,
//! `fx2lafw-sigrok-fx2-8ch.fw`). Without `flashed=1` the device comes up
//! as the FX2 without firmware (no strings), takes the upload through
//! the boot loader and renumerates; with it, it comes up running the
//! firmware already, as after an earlier upload.
//!
//! Once running, the firmware's descriptors are those of `dscr.inc`:
//! the same ids, `bcdDevice` 0x0001, strings `sigrok`, `fx2lafw` and the
//! board's name as string 3 (`Cypress FX2`, `Saleae Logic`), which the
//! driver takes for the serial number; interface 0 with one bulk IN
//! endpoint 2 of 512 bytes, at high speed. Its vendor requests are
//! those of `fx2lafw.c`:
//!
//! | request | data | answer |
//! |---|---|---|
//! | `0xb0` IN | 2 bytes | firmware version 1.4 |
//! | `0xb1` OUT | 3 bytes: flags, delay high, delay low | starts the acquisition |
//! | `0xb2` IN | 1 byte | `REVID`, 1 on the FX2LP |
//!
//! The start flags are bit 4 CTL2 clock (analog boards only), bit 5
//! 16 bit samples, bit 6 the 48 MHz clock (30 MHz without); the GPIF
//! takes a sample every `delay + 1` clocks. The samples are a counter
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
//! What the model taught about the C driver: its soft trigger takes
//! the sample width from all logic channels of the device
//! (`logic_channel_unitsize(sdi->channels)`, two bytes on a 16 channel
//! board) while the stream has one byte per sample when only D0 to D7
//! are enabled, so a trigger on such a capture matches every other
//! byte against the wrong channels and starts the data at half the
//! offset. oxrok's port uses the stream's width; the cases trigger on
//! 8 channel boards, or on the 16 channel one with every channel.
//!
//! The pedantic checks follow `protocol.c` and the firmware: the
//! upload byte for byte as `ezusb.c` sends it, of the profile's
//! firmware file; vendor requests other than the three, or with other
//! lengths, values or indexes; start flags outside the three, CTL2 on a
//! board without analog input, 16 bit samples on an 8 channel board or
//! above 12 MHz, a delay the firmware refuses (high byte 6 or more), a
//! rate the driver's list lacks or encoded with the 30 MHz clock where
//! the driver would take 48 MHz; bulk IN transfers that are not a
//! multiple of 512 bytes; a firmware upload to the running firmware.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, VENDOR, Violations};
use crate::shells::ezusb::{self, Chip, Loader};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "fx2lafw";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "FX2 logic analyzer with the fx2lafw firmware (EZ-USB, renumerates); \
                         profile=cypress-fx2 (04b4:8613, 16 ch, default), saleae-logic \
                         (0925:3881, 8 ch) or sigrok-fx2-8ch (1d50:608c, 8 ch), flashed=1 starts with the firmware running";

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
}

/// The emulated boards.
pub const PROFILES: [Profile; 3] = [
    Profile {
        key: "cypress-fx2",
        vendor: ezusb::CYPRESS_VID,
        product: ezusb::FX2_PID,
        firmware: "fx2lafw-cypress-fx2.fw",
        firmware_size: 7690,
        board: "Cypress FX2",
        wide: true,
    },
    Profile {
        key: "saleae-logic",
        vendor: 0x0925,
        product: 0x3881,
        firmware: "fx2lafw-saleae-logic.fw",
        firmware_size: 7642,
        board: "Saleae Logic",
        wide: false,
    },
    Profile {
        key: "sigrok-fx2-8ch",
        vendor: 0x1d50,
        product: 0x608c,
        firmware: "fx2lafw-sigrok-fx2-8ch.fw",
        firmware_size: 7570,
        board: "fx2grok-flat",
        wide: false,
    },
];

/// A board from the `-o` options: `profile=cypress-fx2|saleae-logic`,
/// `flashed=0|1`.
///
/// # Errors
///
/// On an unknown option or value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut profile = PROFILES[0];
    let mut flashed = false;
    for (k, v) in options {
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
    Ok(Box::new(Fx2lafw::new(profile, flashed)))
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
    /// The next sample's number.
    sample: u64,
    /// Whether the next bulk IN transfer may complete.
    ready: bool,
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
    fn sample(n: u64, wide: bool) -> u16 {
        if wide { n as u16 } else { u16::from(n as u8) }
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
        if flags & FLAG_CTL2 != 0 {
            v.report("start with the CTL2 clock, which only boards with analog input use");
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
        if wide && !self.profile.wide {
            v.report("16 bit samples on an 8 channel board");
        }
        if wide && rate > 12_000_000 {
            v.report(format!("16 bit samples at {rate} Hz; the driver allows 12 MHz at most"));
        }
        Some(Acquisition { wide, sample: 0, ready: true })
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
        match setup.request {
            CMD_GET_FW_VERSION => Reply::Data(VERSION.to_vec()),
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
        let unit = if acq.wide { 2 } else { 1 };
        let mut data = Vec::with_capacity(max);
        for _ in 0..max / unit {
            let s = Self::sample(acq.sample, acq.wide).to_le_bytes();
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
}
