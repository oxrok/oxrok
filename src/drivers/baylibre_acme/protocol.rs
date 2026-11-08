// Counterpart in sigrok:
//   libsigrok/src/hardware/baylibre-acme/protocol.c,
//   libsigrok/src/hardware/baylibre-acme/protocol.h,
//   libsigrok/src/hardware/baylibre-acme/gpio.c,
//   libsigrok/src/hardware/baylibre-acme/gpio.h,
//   Copyright (C) 2015 Bartosz Golaszewski <bgolaszewski@baylibre.com>,
//   with commits by Uwe Hermann, Daniel Elstner, Daniel Lezcano,
//   Lars-Peter Clausen, Aurelien Jacobs, Gerhard Sittig, Martin Ling
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The files: what the kernel's drivers of an ACME cape put in sysfs, at the
//! paths C hard-codes.
//!
//! - `/sys/class/i2c-adapter/i2c-1/1-00XX/name` is the chip declared at address
//!   XX: `ina226` (a power probe, at 0x40, 0x41, 0x44, 0x45, 0x42, 0x43, 0x46, 0x47
//!   for connectors 1 to 8) or `tmp435` (a temperature probe, at 0x4c, 0x49, 0x4f,
//!   0x4b for connectors 5 to 8). A probe is there when the chip's device has a
//!   `hwmon` directory, whose one entry `hwmonN` names its hwmon device.
//! - `/sys/class/hwmon/hwmonN/` holds the values, each `%ld\n`: `power1_input`
//!   (µW), `curr1_input` (mA), `in1_input` (mV), `temp1_input` and `temp2_input`
//!   (m°C); `shunt_resistor` (µΩ, written in mΩ times 1000) and `update_interval`
//!   (ms, written as 1000 / samplerate).
//! - `/sys/class/i2c-dev/i2c-1/device/1-00YY/eeprom`, YY the probe's address plus
//!   0x10: a revision B probe's EEPROM, 61 bytes read, big endian: type (1 USB,
//!   2 jack, 3 HE10), revision (`B`), shunt in µΩ, a power switch flag. A probe
//!   without one is of revision A.
//! - `/sys/class/gpio/`: revision A probes have a line that reads 1 when the
//!   probe has a power switch, and the switch on another; revision B probes have
//!   the switch alone, and the EEPROM says whether it is there. The switch's 1 is
//!   power on.
//!
//! Where this differs from C, in the reading of the files: a value file read is a
//! number and a newline, as the kernel writes it, or an error: C's `strtol` makes
//! 0 of junk and reads a stack buffer it never filled when a read returns nothing
//! (`protocol.c:644-653`). A GPIO that isn't exported is exported first: C writes
//! its direction before exporting it (`gpio.c:67-73`), which fails, so it never
//! exports one. A write's error is an error: C checks only the `fprintf` into
//! stdio's buffer (`gpio.c:44-45`), and a shunt's or a sample rate's not at all.

use std::fs::{File, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use crate::core::{Error, Mq, Result, Unit};

/// The connectors (`MAX_PROBES`).
pub const MAX_PROBES: usize = 8;
/// The first connector a temperature probe can be on, from 0
/// (`TEMP_PRB_START_INDEX`).
pub const TEMP_PRB_START_INDEX: usize = 4;
/// The name a power probe's chip has (`ENRG_PROBE_NAME`).
pub const ENRG_PROBE_NAME: &str = "ina226";
/// The name a temperature probe's chip has (`TEMP_PROBE_NAME`).
pub const TEMP_PROBE_NAME: &str = "tmp435";

const ENRG_I2C_ADDRS: [u8; MAX_PROBES] = [0x40, 0x41, 0x44, 0x45, 0x42, 0x43, 0x46, 0x47];
const TEMP_I2C_ADDRS: [u8; MAX_PROBES] = [0x0, 0x0, 0x0, 0x0, 0x4c, 0x49, 0x4f, 0x4b];
const REVA_PWS_GPIOS: [u32; MAX_PROBES] = [486, 498, 502, 482, 478, 506, 510, 474];
const REVA_PWS_INFO_GPIOS: [u32; MAX_PROBES] = [487, 499, 503, 483, 479, 507, 511, 475];
const REVB_PWS_GPIOS: [u32; MAX_PROBES] = [489, 491, 493, 495, 497, 499, 501, 503];

/// What C reads of an EEPROM (`EEPROM_SIZE`).
pub const EEPROM_SIZE: usize = 3 * 4 + 1 + 16 + 32;
const EEPROM_PROBE_TYPE_USB: u32 = 1;
const EEPROM_PROBE_TYPE_JACK: u32 = 2;
const EEPROM_PROBE_TYPE_HE10: u32 = 3;

/// The highest sample rate, Hz (`MAX_SAMPLE_RATE`).
pub const MAX_SAMPLE_RATE: u64 = 500;
/// A value file's read: C's `char buf[16]`.
const READ_SIZE: usize = 16;

/// The ina226 of connector `index` (0 to 7).
pub fn enrg_addr(index: usize) -> u8 {
    ENRG_I2C_ADDRS[index]
}

/// The tmp435 of connector `index`, 0 for none.
pub fn temp_addr(index: usize) -> u8 {
    TEMP_I2C_ADDRS[index]
}

/// For the user probes are numbered from 1 (`PROBE_NUM`).
pub fn probe_num(index: usize) -> usize {
    index + 1
}

/// A probe's kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProbeType {
    /// An ina226: power, current, voltage.
    Energy,
    /// A tmp435: two temperatures.
    Temperature,
}

/// A probe's revision.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rev {
    /// No EEPROM.
    A,
    /// An EEPROM C reads.
    B,
}

/// What a channel measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChannelType {
    /// `P%d_ENRG_PWR`.
    EnrgPwr,
    /// `P%d_ENRG_CURR`.
    EnrgCurr,
    /// `P%d_ENRG_VOL`.
    EnrgVol,
    /// `P%d_TEMP_IN`.
    TempIn,
    /// `P%d_TEMP_OUT`.
    TempOut,
}

impl ChannelType {
    /// The channels of a probe, in C's order.
    pub fn of(kind: ProbeType) -> &'static [ChannelType] {
        match kind {
            ProbeType::Energy => {
                &[ChannelType::EnrgPwr, ChannelType::EnrgCurr, ChannelType::EnrgVol]
            }
            ProbeType::Temperature => &[ChannelType::TempIn, ChannelType::TempOut],
        }
    }

    /// The channel's name for probe `num`.
    pub fn name(self, num: usize) -> String {
        let what = match self {
            ChannelType::EnrgPwr => "ENRG_PWR",
            ChannelType::EnrgCurr => "ENRG_CURR",
            ChannelType::EnrgVol => "ENRG_VOL",
            ChannelType::TempIn => "TEMP_IN",
            ChannelType::TempOut => "TEMP_OUT",
        };
        format!("P{num}_{what}")
    }

    /// The hwmon file (`bl_acme_open_channel`).
    pub fn file(self) -> &'static str {
        match self {
            ChannelType::EnrgPwr => "power1_input",
            ChannelType::EnrgCurr => "curr1_input",
            ChannelType::EnrgVol => "in1_input",
            ChannelType::TempIn => "temp1_input",
            ChannelType::TempOut => "temp2_input",
        }
    }

    /// `channel_to_mq`.
    pub fn mq(self) -> Mq {
        match self {
            ChannelType::EnrgPwr => Mq::Power,
            ChannelType::EnrgCurr => Mq::Current,
            ChannelType::EnrgVol => Mq::Voltage,
            ChannelType::TempIn | ChannelType::TempOut => Mq::Temperature,
        }
    }

    /// `channel_to_unit`.
    pub fn unit(self) -> Unit {
        match self {
            ChannelType::EnrgPwr => Unit::Watt,
            ChannelType::EnrgCurr => Unit::Ampere,
            ChannelType::EnrgVol => Unit::Volt,
            ChannelType::TempIn | ChannelType::TempOut => Unit::Celsius,
        }
    }

    /// The decimal places the file's unit is below the channel's (`type_digits`):
    /// µW to W, mA to A, mV to V, m°C to °C.
    pub fn digits(self) -> i32 {
        match self {
            ChannelType::EnrgPwr => 6,
            _ => 3,
        }
    }
}

/// A value as C makes it of the file's number: `strtol(...) * powf(10, -digits)`,
/// in single precision.
#[allow(clippy::cast_precision_loss)]
pub fn scale(raw: i64, digits: i32) -> f32 {
    raw as f32 * 10f32.powf(-(digits as f32))
}

/// A value file's contents as the kernel's hwmon writes them, `%ld\n`: digits
/// with an optional minus and the newline. Anything else is no reading.
pub fn parse_value(data: &[u8]) -> Option<i64> {
    let text = std::str::from_utf8(data).ok()?;
    let text = text.strip_suffix('\n').unwrap_or(text);
    let digits = text.strip_prefix('-').unwrap_or(text);
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    text.parse().ok()
}

/// What `strtol(s, NULL, 10)` reads of a file, for what C reads as a number
/// without checking it: white space, a sign and digits, 0 for nothing, the
/// extremes on overflow.
pub fn c_strtol(data: &[u8]) -> i64 {
    let mut i = 0;
    while i < data.len() && matches!(data[i], b' ' | b'\t' | b'\n' | b'\r' | 0x0b | 0x0c) {
        i += 1;
    }
    let neg = match data.get(i) {
        Some(b'-') => {
            i += 1;
            true
        }
        Some(b'+') => {
            i += 1;
            false
        }
        _ => false,
    };
    let mut v: i64 = 0;
    while let Some(d) = data.get(i).filter(|b| b.is_ascii_digit()) {
        let digit = i64::from(d - b'0');
        v = match v
            .checked_mul(10)
            .and_then(|v| if neg { v.checked_sub(digit) } else { v.checked_add(digit) })
        {
            Some(v) => v,
            None => return if neg { i64::MIN } else { i64::MAX },
        };
        i += 1;
    }
    v
}

/// `sscanf(name, "hwmon%d", &n)`: the number after `hwmon`, white space and a
/// sign allowed before it, anything after it ignored.
pub fn hwmon_number(name: &str) -> Option<i64> {
    let rest = name.strip_prefix("hwmon")?;
    let rest = rest.trim_start_matches([' ', '\t', '\n', '\r', '\x0b', '\x0c']);
    let (neg, rest) = match rest.as_bytes().first() {
        Some(b'-') => (true, &rest[1..]),
        Some(b'+') => (false, &rest[1..]),
        _ => (false, rest),
    };
    let end = rest.bytes().position(|b| !b.is_ascii_digit()).unwrap_or(rest.len());
    if end == 0 {
        return None;
    }
    // %d of a number past an int is undefined in C; it is no hwmon here.
    let v: i64 = rest[..end].parse().ok().filter(|v| *v <= i64::from(i32::MAX))?;
    Some(if neg { -v } else { v })
}

/// `revB_addr_to_num`: revision B probes are numbered by address.
pub fn rev_b_addr_to_num(addr: u8) -> usize {
    match addr {
        0x44 => 5,
        0x45 => 6,
        0x42 => 3,
        0x43 => 4,
        _ => usize::from(addr).wrapping_sub(0x3f),
    }
}

/// A revision B probe's EEPROM, as far as C reads it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Eeprom {
    /// 1 USB, 2 jack, 3 HE10.
    pub kind: u32,
    /// `B`.
    pub rev: u32,
    /// µΩ; 0 keeps the kernel's.
    pub shunt: u32,
    /// Whether the probe has a power switch.
    pub pwr_sw: u8,
}

impl Eeprom {
    /// `read_probe_eeprom`'s checks on the first [`EEPROM_SIZE`] bytes: `None`
    /// for anything but a revision B probe of a known type.
    pub fn parse(buf: &[u8]) -> Option<Eeprom> {
        if buf.len() != EEPROM_SIZE {
            return None;
        }
        let be = |at: usize| u32::from_be_bytes([buf[at], buf[at + 1], buf[at + 2], buf[at + 3]]);
        let e = Eeprom { kind: be(0), rev: be(4), shunt: be(8), pwr_sw: buf[12] };
        if e.rev != u32::from(b'B') {
            return None;
        }
        if ![EEPROM_PROBE_TYPE_USB, EEPROM_PROBE_TYPE_JACK, EEPROM_PROBE_TYPE_HE10]
            .contains(&e.kind)
        {
            return None;
        }
        Some(e)
    }
}

/// A probe found at a scan, and what C keeps of it (`struct channel_group_priv`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Probe {
    /// The group's name, `Probe_%d`.
    pub name: String,
    /// The number in its channels' names.
    pub num: usize,
    /// The revision.
    pub rev: Rev,
    /// The kind.
    pub kind: ProbeType,
    /// Its hwmon device's number.
    pub hwmon: i64,
    /// Whether it has a power switch.
    pub has_pws: bool,
    /// The switch's GPIO: none for a number past the tables (a revision B
    /// temperature probe, numbered by its address, `Probe_13` and up).
    pub pws_gpio: Option<u32>,
}

/// The system's files, under a root that is `/` but in tests.
#[derive(Clone, Debug)]
pub struct Sysfs {
    root: PathBuf,
}

impl Default for Sysfs {
    fn default() -> Sysfs {
        Sysfs::new("/")
    }
}

impl Sysfs {
    /// Files under `root`: `/sys/...` is `root/sys/...`.
    pub fn new(root: impl Into<PathBuf>) -> Sysfs {
        Sysfs { root: root.into() }
    }

    /// The real path of what C calls `path`.
    pub fn path(&self, path: &str) -> PathBuf {
        self.root.join(path.trim_start_matches('/'))
    }

    pub(super) fn is_dir(&self, path: &str) -> bool {
        self.path(path).is_dir()
    }

    fn exists(&self, path: &str) -> bool {
        self.path(path).exists()
    }

    /// `bl_acme_is_sane`: sysfs is there.
    pub fn is_sane(&self) -> bool {
        if !self.is_dir("/sys") {
            log::error!("/sys/ directory not found - sysfs not mounted?");
            return false;
        }
        true
    }

    /// `bl_acme_detect_probe`: a chip of that name at `addr` that has a hwmon
    /// device, so a probe answered.
    pub fn detect_probe(&self, addr: u8, prb_num: usize, prb_name: &str) -> bool {
        let path = format!("/sys/class/i2c-adapter/i2c-1/1-00{addr:02x}/name");
        let buf = match std::fs::read(self.path(&path)) {
            Ok(buf) => buf,
            Err(e) => {
                // Don't log "No such file or directory" messages.
                if e.kind() != std::io::ErrorKind::NotFound {
                    log::debug!("Name for probe {prb_num} can't be read: {}", strerror(&e));
                }
                return false;
            }
        };
        buf.starts_with(prb_name.as_bytes())
            && self.is_dir(&format!("/sys/class/i2c-adapter/i2c-1/1-00{addr:02x}/hwmon"))
    }

    /// `get_hwmon_index`: the number of the one entry `hwmonN` in the chip's
    /// `hwmon` directory.
    fn hwmon_index(&self, addr: u8) -> Option<i64> {
        let path = format!("/sys/class/i2c-adapter/i2c-1/1-00{addr:02x}/hwmon");
        let mut dir = match std::fs::read_dir(self.path(&path)) {
            Ok(dir) => dir,
            Err(e) => {
                log::error!(
                    "Error opening {path}: Error opening directory “{path}”: {}",
                    strerror(&e)
                );
                return None;
            }
        };
        // g_dir_read_name: the first entry. C passes the NULL of an empty
        // directory to sscanf.
        let first = dir.find_map(std::result::Result::ok);
        let Some(n) = first.and_then(|e| hwmon_number(&e.file_name().to_string_lossy())) else {
            log::error!("Unable to determine the hwmon entry");
            return None;
        };
        (n >= 0).then_some(n)
    }

    /// `read_probe_eeprom`.
    fn read_eeprom(&self, addr: u8) -> Option<Eeprom> {
        let path =
            format!("/sys/class/i2c-dev/i2c-1/device/1-00{:02x}/eeprom", u16::from(addr) + 0x10);
        let mut file = File::open(self.path(&path)).ok()?;
        let mut buf = [0u8; EEPROM_SIZE];
        // One read(2) of the size, as C's.
        let got = file.read(&mut buf).ok()?;
        Eeprom::parse(&buf[..got])
    }

    /// `bl_acme_register_probe` up to the channels: the probe's group, or `None`
    /// when its hwmon device can't be named. A revision B probe gets its
    /// EEPROM's shunt.
    pub fn register_probe(&self, kind: ProbeType, addr: u8, prb_num: usize) -> Option<Probe> {
        let hwmon = self.hwmon_index(addr)?;
        let eeprom = self.read_eeprom(addr);
        let rev = if eeprom.is_some() { Rev::B } else { Rev::A };
        let num = match rev {
            Rev::A => prb_num,
            Rev::B => rev_b_addr_to_num(addr),
        };
        let index = num.wrapping_sub(1);
        let mut probe = Probe {
            name: format!("Probe_{num}"),
            num,
            rev,
            kind,
            hwmon,
            has_pws: false,
            pws_gpio: None,
        };
        match eeprom {
            None => {
                // The line that says a switch is there; one that can't be read
                // says there is none. C takes its -1 for a switch.
                if let Some(&gpio) = REVA_PWS_INFO_GPIOS.get(index) {
                    probe.has_pws = super::gpio::getval_export(self, gpio).is_ok_and(|v| v != 0);
                }
                probe.pws_gpio = REVA_PWS_GPIOS.get(index).copied();
            }
            Some(e) => {
                probe.has_pws = e.pwr_sw != 0;
                probe.pws_gpio = REVB_PWS_GPIOS.get(index).copied();
                // Keep the default value if shunt in EEPROM == 0.
                if e.shunt > 0
                    && let Err(e) = self.set_shunt(&probe, u64::from(e.shunt / 1000))
                {
                    log::debug!("The EEPROM's shunt was not set: {e}");
                }
            }
        }
        Some(probe)
    }

    /// `get_shunt_path`.
    fn shunt_path(&self, probe: &Probe) -> Result<String> {
        if probe.kind != ProbeType::Energy {
            log::error!("Probe doesn't support shunt resistance setting");
            return Err(Error::arg("Probe doesn't support shunt resistance setting."));
        }
        let path = format!("/sys/class/hwmon/hwmon{}/shunt_resistor", probe.hwmon);
        // The shunt_resistor sysfs attribute is available in the Linux kernel
        // since version 3.20.
        if !self.exists(&path) {
            log::error!(
                "shunt_resistance attribute not present, please update your kernel to version >=3.20"
            );
            return Err(sr_error("not applicable"));
        }
        Ok(path)
    }

    /// `bl_acme_get_shunt`: mΩ.
    pub fn get_shunt(&self, probe: &Probe) -> Result<u64> {
        let path = self.shunt_path(probe)?;
        let data = read_contents(&self.path(&path)).map_err(|(opening, e)| {
            let what = if opening { "Failed to open" } else { "Error reading" };
            log::error!("Error reading shunt resistance: {what} file “{path}”: {}", strerror(&e));
            sr_error("input/output error")
        })?;
        let Some(uohm) = parse_value(&data).and_then(|v| u64::try_from(v).ok()) else {
            log::error!("Error reading shunt resistance: {path} holds no resistance");
            return Err(Error::device(format!("{path} holds no resistance")));
        };
        Ok(uohm / 1000)
    }

    /// `bl_acme_set_shunt`: mΩ, written in µΩ. C's `uint64_t` product wraps.
    pub fn set_shunt(&self, probe: &Probe, shunt: u64) -> Result<()> {
        let path = self.shunt_path(probe)?;
        self.write_attr(&path, &format!("{}\n", shunt.wrapping_mul(1000)))
            .map_err(|_| sr_error("input/output error"))
    }

    /// `bl_acme_maybe_set_update_interval`: each probe's `update_interval`, when it
    /// has one, to 1000 / samplerate ms. A write that fails is passed over, as in C.
    pub fn maybe_set_update_interval(&self, probes: &[Probe], samplerate: u64) {
        for probe in probes {
            let path = format!("/sys/class/hwmon/hwmon{}/update_interval", probe.hwmon);
            if self.exists(&path)
                && let Err(e) = self.write_quiet(&path, &format!("{}\n", 1000 / samplerate.max(1)))
            {
                log::debug!("{path}: {e}");
            }
        }
    }

    /// `bl_acme_read_power_state`: whether the probe's power is off.
    pub fn read_power_state(&self, probe: &Probe) -> Result<bool> {
        let gpio = Self::pws(probe)?;
        Ok(super::gpio::getval_export(self, gpio)? == 0)
    }

    /// `bl_acme_set_power_off`.
    pub fn set_power_off(&self, probe: &Probe, off: bool) -> Result<()> {
        let gpio = Self::pws(probe)?;
        super::gpio::setval_export(self, gpio, !off).map_err(|_| {
            log::error!("Error setting power-off state: gpio: {gpio}");
            sr_error("input/output error")
        })
    }

    fn pws(probe: &Probe) -> Result<u32> {
        if let (true, Some(gpio)) = (probe.has_pws, probe.pws_gpio) {
            return Ok(gpio);
        }
        log::error!("Probe has no power-switch");
        Err(Error::arg("Probe has no power-switch."))
    }

    /// Writes a file as C's `fopen`, `fprintf` and `fclose` do, with the close's
    /// error too; the error says whether the open failed.
    pub(super) fn write_raw(
        &self,
        path: &str,
        text: &str,
    ) -> std::result::Result<(), (bool, std::io::Error)> {
        let mut file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .open(self.path(path))
            .map_err(|e| (true, e))?;
        file.write_all(text.as_bytes()).and_then(|()| file.flush()).map_err(|e| (false, e))?;
        // A store's error comes from write(2), which flush gives; the close has
        // nothing left to say on sysfs.
        drop(file);
        Ok(())
    }

    /// [`Sysfs::write_raw`], saying what failed in C's words.
    fn write_attr(&self, path: &str, text: &str) -> Result<()> {
        self.write_raw(path, text).map_err(|(opening, e)| {
            if opening {
                log::error!("Error opening {path}: {}", strerror(&e));
            } else {
                log::error!("Error writing to {path}: {}", strerror(&e));
            }
            Error::Io(e)
        })
    }

    /// [`Sysfs::write_attr`] with nothing logged.
    fn write_quiet(&self, path: &str, text: &str) -> Result<()> {
        let mut file = OpenOptions::new().write(true).truncate(true).open(self.path(path))?;
        file.write_all(text.as_bytes())?;
        file.flush()?;
        Ok(())
    }

    /// `bl_acme_open_channel`: the value file, kept open for the capture.
    ///
    /// # Errors
    ///
    /// When it can't be opened, said in C's words.
    pub fn open_channel(&self, hwmon: i64, ch: ChannelType) -> std::io::Result<ValueFile> {
        let path = format!("/sys/class/hwmon/hwmon{hwmon}/{}", ch.file());
        match File::open(self.path(&path)) {
            Ok(file) => Ok(ValueFile { file }),
            Err(e) => {
                log::error!("Error opening {path}: {}", strerror(&e));
                Err(e)
            }
        }
    }
}

/// An error that says what libsigrok's code of it says (`sr_strerror`), for the
/// command line's line after the driver's own, which said why.
pub fn sr_error(words: &str) -> Error {
    Error::Io(std::io::Error::other(words.to_owned()))
}

/// An error's words as C's `g_strerror` has them, without Rust's ` (os error N)`.
pub fn strerror(e: &std::io::Error) -> String {
    let text = e.to_string();
    text.split(" (os error").next().unwrap_or_default().to_owned()
}

/// A whole file, as `g_file_get_contents` reads it; the error says whether the
/// open failed.
fn read_contents(path: &Path) -> std::result::Result<Vec<u8>, (bool, std::io::Error)> {
    let mut file = File::open(path).map_err(|e| (true, e))?;
    let mut data = Vec::new();
    file.read_to_end(&mut data).map_err(|e| (false, e))?;
    Ok(data)
}

/// An open value file.
#[derive(Debug)]
pub struct ValueFile {
    file: File,
}

/// Why a value file gave no reading.
#[derive(Debug)]
pub enum ReadError {
    /// read(2) failed.
    Io(std::io::Error),
    /// It returned nothing: a removed attribute, never a live one.
    Empty,
    /// What it holds is not the kernel's number.
    Junk(Vec<u8>),
}

impl std::fmt::Display for ReadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ReadError::Io(e) => write!(f, "{}", strerror(e)),
            ReadError::Empty => write!(f, "nothing to read"),
            ReadError::Junk(data) => write!(f, "not a number: {:?}", String::from_utf8_lossy(data)),
        }
    }
}

impl ValueFile {
    /// `read_sample`'s read: from the start, one read(2) of up to 16 bytes, the
    /// number in it.
    ///
    /// # Errors
    ///
    /// When the read fails, returns nothing or holds no number.
    pub fn read(&mut self) -> std::result::Result<i64, ReadError> {
        self.file.seek(SeekFrom::Start(0)).map_err(ReadError::Io)?;
        let mut buf = [0u8; READ_SIZE];
        let n = self.file.read(&mut buf).map_err(ReadError::Io)?;
        if n == 0 {
            return Err(ReadError::Empty);
        }
        parse_value(&buf[..n]).ok_or_else(|| ReadError::Junk(buf[..n].to_vec()))
    }
}

/// The timer of a capture: C's timerfd, with its first expiry one period after
/// the start and one each period after that, as a count of expirations since
/// the last look.
#[derive(Clone, Copy, Debug)]
pub struct Timer {
    start: Instant,
    period: Duration,
    /// Expirations taken so far.
    taken: u64,
}

impl Timer {
    /// A period of 1 / `samplerate` s from `start`.
    pub fn new(start: Instant, samplerate: u64) -> Timer {
        let period = Duration::from_nanos(1_000_000_000 / samplerate.max(1));
        Timer { start, period, taken: 0 }
    }

    /// When the next expiry is due.
    pub fn next(&self) -> Instant {
        self.start + self.period.saturating_mul(u32::try_from(self.taken + 1).unwrap_or(u32::MAX))
    }

    /// The expirations since the last look, which a read of the timerfd gives,
    /// taken: 0 before the next is due.
    pub fn take(&mut self, now: Instant) -> u64 {
        let elapsed = now.saturating_duration_since(self.start).as_nanos();
        let due = u64::try_from(elapsed / self.period.as_nanos().max(1)).unwrap_or(u64::MAX);
        let n = due.saturating_sub(self.taken);
        self.taken = self.taken.max(due);
        n
    }
}

/// `sr_sw_limits`: a count of samples and a time from the header.
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    /// Samples, 0 for none.
    pub samples: u64,
    /// Milliseconds, 0 for none.
    pub msec: u64,
}

impl Limits {
    /// `sr_sw_limits_check`: whether `read` samples by `elapsed` meet a limit.
    pub fn met(&self, read: u64, elapsed: Duration) -> bool {
        (self.samples != 0 && read >= self.samples)
            || (self.msec != 0 && elapsed >= Duration::from_millis(self.msec))
    }
}

/// The path of a test tree's file, for the tests.
pub fn under(root: &Path, path: &str) -> PathBuf {
    root.join(path.trim_start_matches('/'))
}
