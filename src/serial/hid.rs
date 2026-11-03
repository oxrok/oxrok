// Counterpart in sigrok:
//   libsigrok/src/serial_hid.c,
//   libsigrok/src/serial_hid_bu86x.c,
//   libsigrok/src/serial_hid_ch9325.c,
//   libsigrok/src/serial_hid_cp2110.c,
//   libsigrok/src/serial_hid_victor.c,
//   Copyright (C) 2017-2019 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! Serial over USB HID: the cables that carry a meter's byte stream in
//! HID reports. Each chip frames the bytes its own way; this module
//! turns a [`HidDevice`] into a [`Port`] so a driver does not care.
//!
//! The port name follows sigrok: `hid`, `hid/CHIP`,
//! `hid[/CHIP]/sn=SERIAL` or `hid[/CHIP]/raw=PATH`, where CHIP is one
//! of `bu86x`, `ch9325`, `cp2110` and `victor` and PATH the device
//! node (`/dev/hidrawN` on Linux; it may hold slashes, so it runs to
//! the end, as the serial number does). sigrok's `usb=BUS.DEV.IF`
//! (hidapi-libusb) and `iokit=PATH` (macOS) forms parse too, but the
//! HID backend here names devices by node only, so they match nothing
//! yet. Without a chip any cable with
//! a known USB id is taken, and with a path the chip follows from the
//! device's USB id; without a serial number or a path the first match
//! is.

use crate::core::{Error, Result};
use crate::hid::HidDevice;
use crate::serial::{Flow, Params, Parity, Port};
use std::collections::VecDeque;
use std::time::Duration;

/// A serial over HID chip.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chip {
    /// Brymen BU-86X infrared adapter: a bare byte stream in 8 byte
    /// reports, fixed line parameters.
    Bu86x,
    /// WCH CH9325 (and Hoitek HE2325U): 7 bytes per report behind a
    /// count byte, line parameters set with a feature report.
    Ch9325,
    /// Silicon Labs CP2110: 63 bytes per report behind a count byte,
    /// a full UART behind feature reports.
    Cp2110,
    /// Victor 70C/86C cable: 14 byte reports, scrambled.
    Victor,
}

impl Chip {
    /// All chips.
    pub const ALL: [Chip; 4] = [Chip::Bu86x, Chip::Ch9325, Chip::Cp2110, Chip::Victor];

    /// The name used in port names.
    pub fn name(self) -> &'static str {
        match self {
            Chip::Bu86x => "bu86x",
            Chip::Ch9325 => "ch9325",
            Chip::Cp2110 => "cp2110",
            Chip::Victor => "victor",
        }
    }

    /// The chip with this name.
    pub fn from_name(name: &str) -> Option<Chip> {
        Chip::ALL.into_iter().find(|c| c.name() == name)
    }

    /// The USB ids the chip is known under.
    pub fn usb_ids(self) -> &'static [(u16, u16)] {
        match self {
            Chip::Bu86x => &[(0x0820, 0x0001)],
            Chip::Ch9325 => &[(0x1a86, 0xe008), (0x04fa, 0x2490)],
            Chip::Cp2110 => &[(0x10c4, 0xea80)],
            Chip::Victor => &[(0x1244, 0xd237)],
        }
    }

    /// The chip behind a USB id.
    pub fn for_usb_id(vid: u16, pid: u16) -> Option<Chip> {
        Chip::ALL.into_iter().find(|c| c.usb_ids().contains(&(vid, pid)))
    }

    /// Payload bytes per report.
    fn chunk(self) -> usize {
        match self {
            Chip::Bu86x => 8,
            Chip::Ch9325 => 7,
            Chip::Cp2110 => 63,
            Chip::Victor => 14,
        }
    }
}

/// A parsed `hid[/CHIP][/sn=SERIAL|/raw=PATH|/usb=PATH|/iokit=PATH]`
/// port name.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Spec {
    /// The chip, when named.
    pub chip: Option<Chip>,
    /// The serial number, when given.
    pub serial: Option<String>,
    /// The device path as the HID library has it (`/dev/hidrawN` from
    /// `raw=`; `usb=` and `iokit=` converted back as sigrok's
    /// `extract_hidapi_path` does), when given.
    pub raw: Option<String>,
}

/// The device path a `raw=`, `usb=` or `iokit=` field of a port name
/// stands for, as libsigrok's `extract_hidapi_path`: `raw=` is the path
/// itself, `usb=` the hidapi-libusb `bus:device:interface` path with
/// the colons written as periods, `iokit=` a macOS path without its
/// `IOService:` prefix.
fn hidapi_path(field: &str) -> Option<String> {
    if let Some(p) = field.strip_prefix("iokit=") {
        return Some(format!("IOService:{p}"));
    }
    if let Some(p) = field.strip_prefix("raw=") {
        return Some(p.to_owned());
    }
    let p = field.strip_prefix("usb=")?;
    Some(p.chars().map(|c| if c.is_ascii_hexdigit() || c == ':' { c } else { ':' }).collect())
}

/// The port name field for a device path, as libsigrok's
/// `get_hidapi_path_copy`: hex digits and colons become `usb=` with
/// periods, a path without a colon `raw=`, a macOS `IOService:` path
/// `iokit=`. `None` for a path a port name cannot carry.
fn port_path(path: &str) -> Option<String> {
    if path.chars().all(|c| c.is_ascii_hexdigit() || c == ':') {
        return Some(format!("usb={}", path.replace(':', ".")));
    }
    if !path.contains(':') {
        return Some(format!("raw={path}"));
    }
    match path.strip_prefix("IOService:") {
        Some(rest) if !rest.contains(':') => Some(format!("iokit={rest}")),
        _ => None,
    }
}

impl Spec {
    /// Parses a port name. `None` when it is not a HID name at all.
    ///
    /// As in `ser_hid_parse_conn_spec`, a serial number or a path takes
    /// the rest of the name, slashes included, and a chip may come
    /// once, before it.
    pub fn parse(name: &str) -> Option<Result<Spec>> {
        let mut rest = name.strip_prefix("hid")?;
        if !rest.is_empty() && !rest.starts_with('/') {
            return None;
        }
        let bad = || Some(Err(Error::Arg(format!("Invalid HID port name '{name}'."))));
        let mut spec = Spec::default();
        loop {
            rest = rest.trim_start_matches('/');
            if rest.is_empty() {
                break;
            }
            if let Some(sn) = rest.strip_prefix("sn=") {
                spec.serial = Some(sn.to_owned());
                break;
            }
            if let Some(path) = hidapi_path(rest) {
                if rest.split_once('=').is_some_and(|(_, p)| p.is_empty()) {
                    return bad();
                }
                spec.raw = Some(path);
                break;
            }
            let (field, tail) = rest.split_once('/').unwrap_or((rest, ""));
            match Chip::from_name(field) {
                Some(chip) if spec.chip.is_none() => spec.chip = Some(chip),
                _ => return bad(),
            }
            rest = tail;
        }
        Some(Ok(spec))
    }

    /// The cable this spec picks among `devices`, in listing order, and
    /// its chip; libsigrok's `ser_hid_chip_search`. A path picks that
    /// node, of any USB id when the chip is named (a cable with an
    /// unlisted id), else of a known one. Otherwise only cables with a
    /// known USB id count, of the named chip when there is one, with
    /// the serial number when there is one; the first such is taken.
    pub fn select(&self, devices: &[Candidate<'_>]) -> Option<(Chip, usize)> {
        devices.iter().enumerate().find_map(|(i, d)| {
            if let Some(path) = &self.raw {
                if d.path != Some(path.as_str()) {
                    return None;
                }
                let chip = self.chip.or_else(|| Chip::for_usb_id(d.vendor_id, d.product_id))?;
                return Some((chip, i));
            }
            let chip = Chip::for_usb_id(d.vendor_id, d.product_id)?;
            if self.chip.is_some_and(|c| c != chip) {
                return None;
            }
            if let Some(sn) = &self.serial
                && d.serial != Some(sn.as_str())
            {
                return None;
            }
            Some((chip, i))
        })
    }

    /// The full port name of the cable [`Spec::select`] picked: the
    /// path as given, else the serial number, else the chip alone.
    pub fn full_name(&self, chip: Chip, device: &Candidate<'_>) -> String {
        let field = match (&self.raw, device.serial) {
            (Some(path), _) => port_path(path),
            (None, Some(sn)) if !sn.is_empty() => Some(format!("sn={sn}")),
            _ => None,
        };
        match field {
            Some(f) => format!("hid/{}/{f}", chip.name()),
            None => format!("hid/{}", chip.name()),
        }
    }
}

/// What [`Spec::select`] needs to know of a HID device.
#[derive(Clone, Copy, Debug, Default)]
pub struct Candidate<'a> {
    /// USB vendor id.
    pub vendor_id: u16,
    /// USB product id.
    pub product_id: u16,
    /// Serial number, when the device has one.
    pub serial: Option<&'a str>,
    /// The device path, when the system has one.
    pub path: Option<&'a str>,
}

/// True when `name` names a HID port.
pub fn is_hid_name(name: &str) -> bool {
    Spec::parse(name).is_some()
}

/// Opens the cable `name` describes.
pub fn open(name: &str, params: Params) -> Result<Box<dyn Port>> {
    let spec =
        Spec::parse(name).ok_or_else(|| Error::Arg(format!("Not a HID port: '{name}'.")))??;
    let devices = crate::hid::list()?;
    let candidates: Vec<Candidate<'_>> = devices
        .iter()
        .map(|d| Candidate {
            vendor_id: d.vendor_id,
            product_id: d.product_id,
            serial: d.serial.as_deref(),
            path: d.path.as_deref(),
        })
        .collect();
    let Some((chip, i)) = spec.select(&candidates) else {
        return Err(Error::device(format!("No HID cable matches '{name}'.")));
    };
    let full = spec.full_name(chip, &candidates[i]);
    let info = &devices[i];
    log::info!("Opening {full} ({:04x}:{:04x}).", info.vendor_id, info.product_id);
    let dev = info.open()?;
    Ok(Box::new(HidPort::open(chip, dev, full, params)?))
}

/// A [`Port`] over a serial over HID cable.
pub struct HidPort {
    name: String,
    chip: Chip,
    dev: Box<dyn HidDevice>,
    rx: VecDeque<u8>,
    data_bits: u8,
}

impl HidPort {
    /// A port on an open device of `chip`, named `name`.
    pub fn new(chip: Chip, dev: Box<dyn HidDevice>, name: String) -> HidPort {
        HidPort { name, chip, dev, rx: VecDeque::new(), data_bits: 8 }
    }

    /// A port on an open device, set up as libsigrok's `serial_open`
    /// does: the line parameters, then a flush (for the CP2110 a purge
    /// of both FIFOs).
    ///
    /// # Errors
    ///
    /// When the chip refuses the parameters or the purge fails.
    pub fn open(
        chip: Chip,
        dev: Box<dyn HidDevice>,
        name: String,
        params: Params,
    ) -> Result<HidPort> {
        let mut port = HidPort::new(chip, dev, name);
        port.set_params(params)?;
        port.purge()?;
        Ok(port)
    }

    /// Reads one report and queues the bytes it carries. Returns how
    /// many were queued; 0 on a timeout or an empty report.
    fn fill(&mut self, timeout: Duration) -> Result<usize> {
        let mut report = [0u8; crate::hid::MAX_REPORT];
        let n = self.dev.read_report(&mut report, timeout)?;
        if n == 0 {
            return Ok(0);
        }
        let bytes = decode(self.chip, &report[..n])?;
        let mask =
            if self.data_bits.is_multiple_of(8) { 0xff } else { (1u8 << self.data_bits) - 1 };
        self.rx.extend(bytes.iter().map(|b| b & mask));
        Ok(bytes.len())
    }
}

/// The payload bytes of one input report of `chip`.
fn decode(chip: Chip, report: &[u8]) -> Result<Vec<u8>> {
    Ok(match chip {
        Chip::Bu86x => report.to_vec(),
        Chip::Ch9325 => {
            let count = report[0];
            if count & 0xf0 != 0xf0 || usize::from(count & 0x0f) + 1 > report.len() {
                return Err(Error::device(format!("ch9325: bad report header {count:#04x}")));
            }
            report[1..=usize::from(count & 0x0f)].to_vec()
        }
        Chip::Cp2110 => {
            let count = usize::from(report[0]);
            if count == 0 {
                return Ok(Vec::new());
            }
            if count > 63 || count + 1 > report.len() {
                return Err(Error::device(format!("cp2110: bad report header {count}")));
            }
            report[1..=count].to_vec()
        }
        Chip::Victor => victor_unscramble(report),
    })
}

/// Undoes the Victor cable's scrambling: bytes are shuffled, offset by
/// a fixed pattern and bit reversed. Anything that is not a 14 byte
/// report passes through; an all zero report is dropped.
pub fn victor_unscramble(report: &[u8]) -> Vec<u8> {
    const OFFSETS: [u8; 14] = *b"jodenxunickxia";
    const SHUFFLE: [usize; 14] = [6, 13, 5, 11, 2, 7, 9, 8, 3, 10, 12, 0, 4, 1];
    if report.len() != 14 {
        return report.to_vec();
    }
    if report.iter().all(|b| *b == 0) {
        return Vec::new();
    }
    let mut out = [0u8; 14];
    for (i, b) in report.iter().enumerate() {
        out[13 - SHUFFLE[i]] = b.wrapping_sub(OFFSETS[i]).reverse_bits();
    }
    out.to_vec()
}

impl Port for HidPort {
    fn name(&self) -> &str {
        &self.name
    }

    fn set_params(&mut self, p: Params) -> Result<()> {
        self.data_bits = p.data_bits;
        match self.chip {
            Chip::Bu86x | Chip::Victor => Ok(()),
            Chip::Ch9325 => {
                let [lo, hi] = (p.baud as u16).to_le_bytes();
                self.dev.set_feature(&[0, lo, hi, 0, 0, p.data_bits - 5])
            }
            Chip::Cp2110 => {
                if !(300..=1_000_000).contains(&p.baud) {
                    return Err(Error::Arg(format!("cp2110: baud rate {} out of range.", p.baud)));
                }
                let parity = match p.parity {
                    Parity::None => 0,
                    Parity::Even => 1,
                    Parity::Odd => 2,
                };
                let flow = match p.flow {
                    Flow::None => 0,
                    Flow::RtsCts => 1,
                    Flow::XonXoff => {
                        return Err(Error::Arg("cp2110: no XON/XOFF flow control.".to_owned()));
                    }
                };
                self.dev.set_feature(&[0x41, 1])?;
                let [b3, b2, b1, b0] = p.baud.to_be_bytes();
                self.dev.set_feature(&[
                    0x50,
                    b3,
                    b2,
                    b1,
                    b0,
                    parity,
                    flow,
                    p.data_bits - 5,
                    u8::from(p.stop_bits == 2),
                ])
            }
        }
    }

    fn set_handshake(&mut self, _rts: Option<bool>, _dtr: Option<bool>) -> Result<()> {
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        if self.rx.is_empty() {
            let deadline = std::time::Instant::now() + timeout;
            loop {
                let left = deadline.saturating_duration_since(std::time::Instant::now());
                if left.is_zero() {
                    return Ok(0);
                }
                if self.fill(left)? > 0 {
                    break;
                }
            }
            // As libsigrok's ser_hid_read: keep taking reports while
            // they come within 10 ms of each other (an empty one ends
            // it too), so a burst of reports is one read.
            while self.rx.len() < buf.len() && self.fill(Duration::from_millis(10))? > 0 {}
        }
        let n = buf.len().min(self.rx.len());
        for b in &mut buf[..n] {
            *b = self.rx.pop_front().expect("queued byte");
        }
        Ok(n)
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        for chunk in data.chunks(self.chip.chunk()) {
            match self.chip {
                Chip::Bu86x => self.dev.write_report(chunk)?,
                Chip::Ch9325 => {
                    let mut report = [0u8; 8];
                    report[0] = chunk.len() as u8;
                    report[1..=chunk.len()].copy_from_slice(chunk);
                    self.dev.write_report(&report)?;
                }
                Chip::Cp2110 => {
                    let mut report = [0u8; 64];
                    report[0] = chunk.len() as u8;
                    report[1..=chunk.len()].copy_from_slice(chunk);
                    self.dev.write_report(&report)?;
                }
                Chip::Victor => {}
            }
        }
        Ok(())
    }

    fn purge(&mut self) -> Result<()> {
        self.rx.clear();
        if self.chip == Chip::Cp2110 {
            self.dev.write_report(&[0x43, 0x03])?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hid::SharedFake;

    const T: Duration = Duration::from_millis(10);

    fn port(chip: Chip, input: &[&[u8]]) -> (HidPort, SharedFake) {
        let fake = SharedFake::default();
        for r in input {
            fake.push_input(r);
        }
        (HidPort::new(chip, Box::new(fake.clone()), "hid".to_owned()), fake)
    }

    #[test]
    fn parses_names() {
        assert_eq!(Spec::parse("hid").unwrap().unwrap(), Spec::default());
        assert_eq!(
            Spec::parse("hid/bu86x").unwrap().unwrap(),
            Spec { chip: Some(Chip::Bu86x), ..Spec::default() }
        );
        assert_eq!(
            Spec::parse("hid/ch9325/sn=AB12").unwrap().unwrap(),
            Spec { chip: Some(Chip::Ch9325), serial: Some("AB12".to_owned()), raw: None }
        );
        assert_eq!(
            Spec::parse("hid/sn=X").unwrap().unwrap(),
            Spec { serial: Some("X".to_owned()), ..Spec::default() }
        );
        // A path or a serial number runs to the end, slashes included.
        assert_eq!(
            Spec::parse("hid/victor/raw=/dev/hidraw3").unwrap().unwrap(),
            Spec { chip: Some(Chip::Victor), raw: Some("/dev/hidraw3".to_owned()), serial: None }
        );
        assert_eq!(
            Spec::parse("hid/raw=/dev/hidraw0").unwrap().unwrap(),
            Spec { raw: Some("/dev/hidraw0".to_owned()), ..Spec::default() }
        );
        assert_eq!(
            Spec::parse("hid/sn=a/b").unwrap().unwrap(),
            Spec { serial: Some("a/b".to_owned()), ..Spec::default() }
        );
        assert!(Spec::parse("hid/raw=").unwrap().is_err());
        assert_eq!(Spec::parse("hid/raw=/dev/x/bu86x").unwrap().unwrap().chip, None);
        assert!(Spec::parse("/dev/ttyUSB0").is_none());
        assert!(Spec::parse("hidden").is_none());
        assert!(Spec::parse("hid/nochip").unwrap().is_err());
        assert!(Spec::parse("hid/bu86x/victor").unwrap().is_err());
        assert_eq!(Chip::for_usb_id(0x04fa, 0x2490), Some(Chip::Ch9325));
    }

    #[test]
    fn parses_the_usb_and_iokit_paths() {
        // serial_hid.c: usb= turns its periods back into colons,
        // iokit= gets its IOService: prefix back; both run to the end.
        assert_eq!(
            Spec::parse("hid/ch9325/usb=0001.0004.00").unwrap().unwrap(),
            Spec { chip: Some(Chip::Ch9325), raw: Some("0001:0004:00".to_owned()), serial: None }
        );
        assert_eq!(
            Spec::parse("hid/iokit=AppleUSB/x@1").unwrap().unwrap().raw.unwrap(),
            "IOService:AppleUSB/x@1"
        );
        assert!(Spec::parse("hid/usb=").unwrap().is_err());
        // And back, as get_hidapi_path_copy names a listed device.
        assert_eq!(port_path("0001:0004:00").unwrap(), "usb=0001.0004.00");
        assert_eq!(port_path("/dev/hidraw2").unwrap(), "raw=/dev/hidraw2");
        assert_eq!(port_path("IOService:AppleUSB/x@1").unwrap(), "iokit=AppleUSB/x@1");
        assert_eq!(port_path("IOService:a:b"), None);
        assert_eq!(port_path("\\\\?\\hid#vid_1a86:x"), None);
    }

    fn cand(
        vid: u16,
        pid: u16,
        serial: Option<&'static str>,
        path: &'static str,
    ) -> Candidate<'static> {
        Candidate { vendor_id: vid, product_id: pid, serial, path: Some(path) }
    }

    #[test]
    fn selects_a_cable_as_ser_hid_chip_search() {
        let devices = [
            cand(0x046d, 0xc077, None, "/dev/hidraw0"), // a mouse
            cand(0x1244, 0xd237, None, "/dev/hidraw1"), // victor
            cand(0x1a86, 0xe008, Some("A1"), "/dev/hidraw2"), // ch9325
            cand(0x1a86, 0xe008, Some("B2"), "/dev/hidraw3"), // ch9325
            cand(0x0820, 0x0001, Some(""), "/dev/hidraw4"), // bu86x
        ];
        let pick = |name: &str| {
            let spec = Spec::parse(name).unwrap().unwrap();
            spec.select(&devices).map(|(chip, i)| (chip, i, spec.full_name(chip, &devices[i])))
        };
        // No chip: the first cable with a known id, not the mouse.
        assert_eq!(pick("hid"), Some((Chip::Victor, 1, "hid/victor".to_owned())));
        // A chip: the first of its ids; its serial number names it.
        assert_eq!(pick("hid/ch9325"), Some((Chip::Ch9325, 2, "hid/ch9325/sn=A1".to_owned())));
        assert_eq!(
            pick("hid/ch9325/sn=B2"),
            Some((Chip::Ch9325, 3, "hid/ch9325/sn=B2".to_owned()))
        );
        assert_eq!(pick("hid/sn=B2"), Some((Chip::Ch9325, 3, "hid/ch9325/sn=B2".to_owned())));
        assert_eq!(pick("hid/victor/sn=B2"), None);
        assert_eq!(pick("hid/sn=C3"), None);
        assert_eq!(pick("hid/cp2110"), None);
        // An empty serial number is no serial number.
        assert_eq!(pick("hid/bu86x"), Some((Chip::Bu86x, 4, "hid/bu86x".to_owned())));
        // A path: the chip from the id, or as named for any id.
        assert_eq!(
            pick("hid/raw=/dev/hidraw3"),
            Some((Chip::Ch9325, 3, "hid/ch9325/raw=/dev/hidraw3".to_owned()))
        );
        assert_eq!(pick("hid/raw=/dev/hidraw0"), None, "a mouse is no cable");
        assert_eq!(
            pick("hid/bu86x/raw=/dev/hidraw0"),
            Some((Chip::Bu86x, 0, "hid/bu86x/raw=/dev/hidraw0".to_owned()))
        );
        assert_eq!(pick("hid/raw=/dev/hidraw9"), None);
        assert_eq!(pick("hid/usb=0001.0004.00"), None, "nodes only");
    }

    #[test]
    fn bu86x_is_a_bare_stream() {
        let (mut p, fake) = port(Chip::Bu86x, &[&[1, 2, 3, 4, 5, 6, 7, 8], &[9, 10], &[]]);
        let mut buf = [0u8; 5];
        assert_eq!(p.read(&mut buf, T).unwrap(), 5);
        assert_eq!(buf, [1, 2, 3, 4, 5]);
        let mut buf = [0u8; 16];
        assert_eq!(p.read(&mut buf, T).unwrap(), 3);
        assert_eq!(p.read(&mut buf, T).unwrap(), 2);
        assert_eq!(&buf[..2], &[9, 10]);
        p.write(&[0x00, 0x00, 0x52, 0x66]).unwrap();
        assert_eq!(fake.output(), vec![vec![0x00, 0x00, 0x52, 0x66]]);
    }

    #[test]
    fn ch9325_frames_and_params() {
        let (mut p, fake) = port(Chip::Ch9325, &[&[0xf3, b'a', b'b', b'c', 0, 0, 0, 0], &[]]);
        p.set_params(Params::parse("2400/7o1").unwrap()).unwrap();
        assert_eq!(fake.features(), vec![vec![0, 0x60, 0x09, 0, 0, 2]]);
        let mut buf = [0u8; 8];
        assert_eq!(p.read(&mut buf, T).unwrap(), 3);
        assert_eq!(&buf[..3], b"abc");
        p.write(b"0123456789").unwrap();
        let out = fake.output();
        assert_eq!(out.len(), 2);
        assert_eq!(out[0], [7, b'0', b'1', b'2', b'3', b'4', b'5', b'6']);
        assert_eq!(out[1], [3, b'7', b'8', b'9', 0, 0, 0, 0]);
        // The 7 bit mask applies to the queued bytes.
        let (mut q, _) = port(Chip::Ch9325, &[&[0xf1, 0xc1], &[]]);
        q.set_params(Params::parse("2400/7o1").unwrap()).unwrap();
        assert_eq!(q.read(&mut buf, T).unwrap(), 1);
        assert_eq!(buf[0], 0x41);
        let (mut bad, _) = port(Chip::Ch9325, &[&[0x00]]);
        assert!(bad.read(&mut buf, T).is_err());
    }

    #[test]
    fn cp2110_frames_params_and_purge() {
        let (mut p, fake) = port(Chip::Cp2110, &[&[2, b'o', b'k'], &[0], &[]]);
        p.set_params(Params::parse("115200/8n2/flow=1").unwrap()).unwrap();
        let f = fake.features();
        assert_eq!(f[0], [0x41, 1]);
        assert_eq!(f[1], [0x50, 0, 1, 0xc2, 0, 0, 1, 3, 1]);
        let mut buf = [0u8; 8];
        assert_eq!(p.read(&mut buf, T).unwrap(), 2);
        assert_eq!(&buf[..2], b"ok");
        // An empty report and then a timeout: nothing.
        assert_eq!(p.read(&mut buf, T).unwrap(), 0);
        p.write(b"D").unwrap();
        p.purge().unwrap();
        let out = fake.output();
        assert_eq!(out[0][..2], [1, b'D']);
        assert_eq!(out[0].len(), 64);
        assert_eq!(out[1], [0x43, 0x03]);
        assert!(p.set_params(Params::parse("110/8n1").unwrap()).is_err());
        assert!(p.set_params(Params::parse("9600/8n1/flow=2").unwrap()).is_err());
    }

    #[test]
    fn cp2110_open_purges_after_the_line_settings() {
        // libsigrok's serial_open: set_params (enable, config), then
        // serial_flush, which the CP2110 sends as output report 0x43.
        let fake = SharedFake::default();
        let params = Params::parse("19200/7o1/rts=0/dtr=1").unwrap();
        HidPort::open(Chip::Cp2110, Box::new(fake.clone()), "hid".to_owned(), params).unwrap();
        assert_eq!(fake.features(), [vec![0x41, 1], vec![0x50, 0, 0, 0x4b, 0, 2, 0, 2, 0]]);
        assert_eq!(fake.output(), [vec![0x43, 0x03]]);
    }

    #[test]
    fn a_burst_of_reports_is_one_read() {
        // An empty report: nothing more within 10 ms.
        let (mut p, _) =
            port(Chip::Cp2110, &[&[2, b'a', b'b'], &[3, b'c', b'd', b'e'], &[1, b'f'], &[]]);
        let mut buf = [0u8; 16];
        assert_eq!(p.read(&mut buf, T).unwrap(), 6);
        assert_eq!(&buf[..6], b"abcdef");
        // A smaller buffer stops taking reports once it is full.
        let (mut p, _) =
            port(Chip::Cp2110, &[&[2, b'a', b'b'], &[3, b'c', b'd', b'e'], &[1, b'f'], &[]]);
        let mut buf = [0u8; 4];
        assert_eq!(p.read(&mut buf, T).unwrap(), 4);
        assert_eq!(p.read(&mut buf, T).unwrap(), 1);
        assert_eq!(p.read(&mut buf, T).unwrap(), 1);
        assert_eq!(buf[0], b'f');
    }

    #[test]
    fn victor_unscrambles() {
        // Scramble a packet the way the cable does and check it comes
        // back.
        const OFFSETS: [u8; 14] = *b"jodenxunickxia";
        const SHUFFLE: [usize; 14] = [6, 13, 5, 11, 2, 7, 9, 8, 3, 10, 12, 0, 4, 1];
        let plain: [u8; 14] = *b"+1234 5\x31\x00\x00\x00\x00\r\n";
        let mut wire = [0u8; 14];
        for i in 0..14 {
            wire[i] = plain[13 - SHUFFLE[i]].reverse_bits().wrapping_add(OFFSETS[i]);
        }
        assert_eq!(victor_unscramble(&wire), plain.to_vec());
        assert_eq!(victor_unscramble(&[0u8; 14]), Vec::<u8>::new());
        assert_eq!(victor_unscramble(&[1, 2]), vec![1, 2]);
        let (mut p, fake) = port(Chip::Victor, &[&wire]);
        let mut buf = [0u8; 14];
        assert_eq!(p.read(&mut buf, T).unwrap(), 14);
        assert_eq!(buf, plain);
        p.write(b"ignored").unwrap();
        assert_eq!(fake.output().len(), 0);
    }
}
