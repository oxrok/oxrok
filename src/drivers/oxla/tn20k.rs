// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! `oxla-tn20k`: a Sipeed Tang Nano 20K as an oxla logic analyzer, with
//! the gateware loaded into it when it does not run it.
//!
//! The board is found on USB, by its debugger (a BL616 that stands in for
//! an FT2232: `0403:6010`, "SIPEED", "USB Debugger"). The first of the
//! debugger's two ports is JTAG to the FPGA, the second the UART the
//! gateware talks on, which the scan finds among the serial ports by the
//! debugger's serial number. Other Sipeed boards have the same debugger:
//! one whose FPGA is no GW2A-18 is listed too, and refused at the load.
//!
//! What the board can be given is not in this file. Every bitstream of a
//! release of <https://github.com/oxrok/fpga-bitstreams> has a description
//! beside it, `NAME.fs.json` (that repository's `docs/SIDECAR.md`): the
//! variant, the board and FPGA it is for, the checksum of the file, and
//! the ID the bitstream gave on a board. The driver looks for those in the
//! firmware directories ([`sidecars`]) and offers the ones for this board;
//! `oxrok firmware fetch oxla-tn20k` gets them. A bitstream with no
//! description beside it is not offered.
//!
//! A scan asks each board what it runs, on its UART (the bytes of
//! [`super::protocol::RESYNC`] and an ID command: a board that runs
//! another design gets those nine bytes), and loads nothing:
//!
//! - a board that answers as oxla is the device it says it is;
//! - one that does not, or that runs another variant than `bitstream=`
//!   names, is listed as the variant it would be given (the one its
//!   description marks as the default, unless named), from that
//!   description, and the bitstream is loaded when the device is opened.
//!
//! So `oxrok --scan` changes no board, and `-d oxla-tn20k` with anything
//! that opens the device does. The load is into the FPGA, and is gone at
//! power off: the board's flash, and what it starts with, are not touched.
//! `conn=` picks one board of several, by its serial port or its USB
//! `bus.address`.
//!
//! Everything after the load is the `oxla` driver's, and what the board
//! then says of itself is what counts, not the description.

use super::gowin::{self, Bitstream};
use super::protocol::{self, Id};
use super::{LOAD_SETTLE, Loader, OxlaDevice};
use crate::core::firmware::FirmwareDependency;
use crate::core::key::CustomKey;
use crate::core::{Device, DeviceClass, Driver, Error, Key, Result, UsbClaim, Value, ValueKind};
use crate::ftdi::mpsse::Jtag;
use crate::serial::Params;
use crate::usb::UsbDeviceInfo;
use std::path::PathBuf;

const VENDOR_ID: u16 = 0x0403;
const PRODUCT_ID: u16 = 0x6010;
const MANUFACTURER: &str = "SIPEED";
const PRODUCT: &str = "USB Debugger";
/// What the board is listed as.
pub const MODEL: &str = "Tang Nano 20K";
/// The board in a description's `target.board`.
const BOARD: &str = "sipeed-tang-nano-20k";
/// The load this driver does, in a description's `target.loader`.
const LOADER: &str = "gowin-jtag-sram";
/// The package of the firmware registry that has the bitstreams.
const PACKAGE: &str = "oxla-tn20k";
const FETCH_HINT: &str = "oxrok firmware fetch oxla-tn20k downloads the bitstreams";
/// The JTAG clock: what openFPGALoader loads this board at.
const JTAG_HZ: u32 = 6_000_000;

/// A bitstream as the description beside it has it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sidecar {
    /// The variant's name in the build, `la16-rle`: what `bitstream=` takes.
    pub variant: String,
    /// The one a board gets when none is asked for.
    pub default: bool,
    /// The release it is of.
    pub version: String,
    /// The `.fs` file, beside the description.
    pub file: PathBuf,
    /// The SHA-256 of that file, lowercase hex.
    pub sha256: String,
    /// The board the pins are for.
    pub board: String,
    /// The JTAG ID code of the FPGA it is for.
    pub idcode: u32,
    /// The ID the bitstream gives when it starts.
    pub id: Id,
    /// The clock of each value of `CLOCK`, in Hz.
    pub clocks_hz: Vec<u32>,
}

fn number(v: &serde_json::Value, what: &str) -> std::result::Result<u64, String> {
    v.as_u64().ok_or_else(|| format!("no number {what}"))
}

fn text<'a>(v: &'a serde_json::Value, what: &str) -> std::result::Result<&'a str, String> {
    v.as_str().ok_or_else(|| format!("no text {what}"))
}

impl Sidecar {
    /// The description in `json`, of the bitstream at `file`; none when it
    /// is of something this driver has no use for (another design, another
    /// way to load, a format it does not know).
    ///
    /// # Errors
    ///
    /// Text that is no description, or one with a field missing or wrong.
    pub fn parse(json: &str, file: PathBuf) -> std::result::Result<Option<Sidecar>, String> {
        let v: serde_json::Value = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let target = &v["target"];
        if v["format"].as_u64() != Some(1)
            || v["design"].as_str() != Some("oxla")
            || target["loader"].as_str() != Some(LOADER)
        {
            return Ok(None);
        }
        let idcode = text(&target["idcode"], "target.idcode")?;
        let idcode = u32::from_str_radix(idcode.trim_start_matches("0x"), 16)
            .map_err(|_| format!("target.idcode {idcode} is no hex number"))?;
        // The ID as its 16 bytes, so that what is wrong with it is what
        // is wrong with a device's.
        let id = &v["id"];
        let mut bytes = Vec::from(*b"OXLA");
        for field in ["protocol", "caps", "channels", "samples_per_clock"] {
            let n = number(&id[field], field)?;
            bytes.push(u8::try_from(n).map_err(|_| format!("id.{field} is {n}"))?);
        }
        for field in ["clock_hz", "mem_words"] {
            let n = number(&id[field], field)?;
            let n = u32::try_from(n).map_err(|_| format!("id.{field} is {n}"))?;
            bytes.extend_from_slice(&n.to_le_bytes());
        }
        let id = Id::parse(&bytes)?;
        let clocks_hz = match v["description"]["clocks_hz"].as_array() {
            Some(clocks) => clocks
                .iter()
                .map(|c| {
                    number(c, "clocks_hz").and_then(|n| u32::try_from(n).map_err(|e| e.to_string()))
                })
                .collect::<std::result::Result<Vec<u32>, String>>()?,
            None => Vec::new(),
        };
        let sha256 = text(&v["sha256"], "sha256")?.to_ascii_lowercase();
        if sha256.len() != 64 || !sha256.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err("sha256 is no SHA-256".to_owned());
        }
        Ok(Some(Sidecar {
            variant: text(&v["variant"], "variant")?.to_owned(),
            default: v["default"].as_bool().unwrap_or(false),
            version: v["version"].as_str().unwrap_or_default().to_owned(),
            file,
            sha256,
            board: text(&target["board"], "target.board")?.to_owned(),
            idcode,
            id,
            clocks_hz,
        }))
    }

    /// The bitstream, read and checked against the description's checksum.
    fn bitstream(&self) -> Result<Bitstream> {
        let name = self.file.display();
        log::debug!("Loading {name}.");
        let text = std::fs::read(&self.file)
            .map_err(|e| Error::Firmware(format!("{name}: {e} ({FETCH_HINT})")))?;
        let sum = sha256_hex(&text);
        if sum != self.sha256 {
            return Err(Error::Firmware(format!(
                "{name} is not the file its description is of: its SHA-256 is {sum}"
            )));
        }
        let bitstream = Bitstream::parse(&text)
            .map_err(|e| Error::Firmware(format!("{name} is no Gowin bitstream: {e}")))?;
        if bitstream.idcode != self.idcode {
            return Err(Error::Firmware(format!(
                "{name} is for ID code {:#010x}, its description says {:#010x}",
                bitstream.idcode, self.idcode
            )));
        }
        Ok(bitstream)
    }
}

fn sha256_hex(data: &[u8]) -> String {
    use sha2::Digest as _;
    use std::fmt::Write as _;
    sha2::Sha256::digest(data).iter().fold(String::new(), |mut s, b| {
        write!(s, "{b:02x}").expect("a string takes it");
        s
    })
}

/// The bitstreams described in `dirs`, by variant: of two descriptions of
/// one file name the first directory's counts, as for every firmware file.
/// A description that cannot be read is logged and left out.
pub fn sidecars_in(dirs: &[PathBuf]) -> Vec<Sidecar> {
    let mut seen = std::collections::BTreeSet::new();
    let mut found = Vec::new();
    for dir in dirs {
        let Ok(entries) = std::fs::read_dir(dir) else {
            continue;
        };
        let mut paths: Vec<PathBuf> = entries.filter_map(|e| e.ok().map(|e| e.path())).collect();
        paths.sort();
        for path in paths {
            let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
                continue;
            };
            let is_fs = |n: &&str| {
                std::path::Path::new(n).extension().is_some_and(|e| e.eq_ignore_ascii_case("fs"))
            };
            let Some(fs) = name.strip_suffix(".json").filter(is_fs) else {
                continue;
            };
            if !seen.insert(name.to_owned()) {
                continue;
            }
            let parsed = std::fs::read_to_string(&path)
                .map_err(|e| e.to_string())
                .and_then(|json| Sidecar::parse(&json, dir.join(fs)));
            match parsed {
                Ok(Some(sidecar)) => found.push(sidecar),
                Ok(None) => log::debug!("{}: not for this driver.", path.display()),
                Err(e) => log::warn!("{}: {e}.", path.display()),
            }
        }
    }
    found.sort_by(|a, b| a.variant.cmp(&b.variant));
    found
}

/// The bitstreams described in the firmware directories.
pub fn sidecars() -> Vec<Sidecar> {
    sidecars_in(&crate::core::firmware::search_dirs())
}

/// Those of `all` that are for this board.
fn for_this_board(all: Vec<Sidecar>) -> Vec<Sidecar> {
    all.into_iter()
        .filter(|s| s.board == BOARD && s.idcode & 0x0fff_ffff == gowin::IDCODE_GW2A_18)
        .collect()
}

/// The one of `variants` a board gets when none is named: the one marked
/// so, or else the first.
fn default_of(variants: &[Sidecar]) -> Option<&Sidecar> {
    variants.iter().find(|s| s.default).or(variants.first())
}

/// Notes that the bitstreams are missing, for a front end to offer the
/// download, and gives the error that says so.
fn missing() -> Error {
    let file = crate::core::firmware::fetch::package(PACKAGE)
        .and_then(|p| p.files.first().copied())
        .unwrap_or("oxla-tn20k-la16-rle.fs.json");
    crate::core::firmware::note_missing(file);
    Error::Firmware(format!("no oxla bitstream for the {MODEL} ({FETCH_HINT})"))
}

static KEY_BITSTREAM: CustomKey = CustomKey {
    id: "bitstream",
    name: "Bitstream",
    description: "The variant of the gateware the board is to run, as the descriptions beside the \
                  installed bitstreams name them (la16-rle, la8-ddr). It is loaded when the \
                  board runs another.",
    kind: ValueKind::Str,
};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Custom(&KEY_BITSTREAM)];
static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];

/// A Tang Nano 20K on USB, or a board with the same debugger.
#[derive(Clone, Debug)]
pub struct Board {
    /// The debugger.
    pub usb: UsbDeviceInfo,
    /// The serial port of its UART, when one was found.
    pub port: Option<String>,
}

impl Board {
    /// The debugger's serial number, or nothing.
    pub fn serial(&self) -> &str {
        self.usb.serial.as_deref().unwrap_or_default()
    }

    /// The bitstreams installed for it, by variant.
    pub fn variants(&self) -> Vec<Sidecar> {
        for_this_board(sidecars())
    }

    /// Loads the bitstream of `sidecar` into the board's FPGA. What it ran
    /// is gone; its flash is as it was.
    ///
    /// # Errors
    ///
    /// A file that is missing, is not the one described or is no
    /// bitstream ([`Error::Firmware`]), a debugger that cannot be opened,
    /// an FPGA that is another part or does not take it.
    pub fn load(&self, sidecar: &Sidecar) -> Result<()> {
        let bitstream = sidecar.bitstream()?;
        log::debug!(
            "{} {}: {} bytes for ID code {:#010x}, checksum {:#06x}.",
            sidecar.variant,
            sidecar.version,
            bitstream.data.len(),
            bitstream.idcode,
            bitstream.checksum
        );
        let transport = crate::usb::open(&self.usb)?;
        let mut jtag = Jtag::open(transport, crate::ftdi::Port::A, JTAG_HZ)?;
        log::debug!("JTAG at {} Hz.", jtag.hz());
        let loaded = gowin::load(&mut jtag, &bitstream);
        // The port goes back to what it was whatever the load came to.
        let closed = jtag.close().map(drop);
        loaded.and(closed)
    }
}

/// Orders port names as their numbers go: `ttyUSB9` ahead of `ttyUSB10`.
fn port_order(name: &str) -> (usize, &str) {
    (name.len(), name)
}

/// The UART of the debugger with serial number `serial` among `ports`:
/// the second of its two ports.
fn uart_of(serial: Option<&str>, ports: &[crate::serial::UsbPort]) -> Option<String> {
    let mut names: Vec<&str> = ports
        .iter()
        .filter(|p| p.vid == VENDOR_ID && p.pid == PRODUCT_ID && p.serial.as_deref() == serial)
        .map(|p| p.name.as_str())
        .collect();
    names.sort_by_key(|n| port_order(n));
    names.dedup();
    match names.as_slice() {
        [_, uart] => Some((*uart).to_owned()),
        _ => None,
    }
}

fn is_debugger(usb: &UsbDeviceInfo) -> bool {
    usb.vendor_id == VENDOR_ID
        && usb.product_id == PRODUCT_ID
        && usb.manufacturer.as_deref() == Some(MANUFACTURER)
        && usb.product.as_deref() == Some(PRODUCT)
}

/// The boards on USB, with nothing sent to any: for a front end to say a
/// board is there and offer to use it.
///
/// # Errors
///
/// A USB bus that cannot be listed.
pub fn boards() -> Result<Vec<Board>> {
    let ports = crate::serial::list_usb_ports();
    Ok(crate::usb::list_devices()?
        .into_iter()
        .filter(is_debugger)
        .map(|usb| {
            let port = uart_of(usb.serial.as_deref(), &ports);
            Board { usb, port }
        })
        .collect())
}

/// What `board` is as a device: the analyzer it answers as, with the port
/// kept for the open, or the one `variants` has for it at the open, when
/// `want` names another variant or it runs none. None when it runs none
/// and there is none to give it.
fn device(
    board: &Board,
    port_name: &str,
    want: Option<&Sidecar>,
    variants: &[Sidecar],
) -> Option<OxlaDevice> {
    let params = Params::parse(protocol::SERIALCOMM).expect("the default parses");
    let running = match crate::serial::open(port_name, params) {
        Ok(mut port) => match super::probe_board(port.as_mut()) {
            Ok(Some(found)) => Some((found, port)),
            Ok(None) => None,
            Err(e) => {
                log::debug!("{port_name}: {e}");
                None
            }
        },
        Err(e) => {
            log::debug!("{port_name}: {e}");
            None
        }
    };
    let device = match running {
        Some((found, port)) if want.is_none_or(|w| w.variant == found.id.variant()) => {
            OxlaDevice::with_params(found, port_name.to_owned(), params, Some(port))
        }
        _ => {
            let sidecar = want.or_else(|| default_of(variants))?.clone();
            let (id, clocks_hz) = (sidecar.id, sidecar.clocks_hz.clone());
            let board = board.clone();
            let name = port_name.to_owned();
            let load: Loader = Box::new(move || {
                board.load(&sidecar)?;
                std::thread::sleep(LOAD_SETTLE);
                crate::serial::open(&name, params)
            });
            OxlaDevice::to_load(id, &clocks_hz, port_name.to_owned(), load)
        }
    };
    Some(device.listed_as("oxla-tn20k", MODEL, board.serial()))
}

/// The `oxla-tn20k` driver.
pub struct OxlaTn20kDriver;

impl Driver for OxlaTn20kDriver {
    fn id(&self) -> &'static str {
        "oxla-tn20k"
    }

    fn long_name(&self) -> &'static str {
        "oxla logic analyzer on a Sipeed Tang Nano 20K"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn firmware(&self) -> Vec<FirmwareDependency> {
        // The files are those of the registry's package, whatever
        // variants it has: there is no list of them here.
        let Some(package) = crate::core::firmware::fetch::package(PACKAGE) else {
            return Vec::new();
        };
        vec![
            FirmwareDependency::upload(
                package.files.iter().copied(),
                "a release of github.com/oxrok/fpga-bitstreams",
            )
            .note("loaded when the board does not run it; bitstream= chooses the variant"),
        ]
    }

    fn usb_claims(&self) -> Vec<UsbClaim> {
        vec![UsbClaim::ids(VENDOR_ID, PRODUCT_ID, "Sipeed", "Tang Nano 20K (its debugger)")]
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut bitstream = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Custom(k) if k.id == KEY_BITSTREAM.id => {
                    let name = value.as_str().ok_or_else(|| Error::arg("bitstream"))?;
                    bitstream = Some(name.to_owned());
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let boards = boards()?;
        if boards.is_empty() {
            return Ok(Vec::new());
        }
        let variants = for_this_board(sidecars());
        let want = match &bitstream {
            Some(name) => Some(variants.iter().find(|s| s.variant == *name).ok_or_else(|| {
                if variants.is_empty() {
                    return missing();
                }
                let names: Vec<&str> = variants.iter().map(|s| s.variant.as_str()).collect();
                Error::Arg(format!("no bitstream {name}: there are {}", names.join(", ")))
            })?),
            None => None,
        };
        let mut devices: Vec<Box<dyn Device>> = Vec::new();
        for board in boards {
            let Some(port) = board.port.clone() else {
                log::warn!(
                    "A {MODEL} (serial number {}) with no serial port of its UART.",
                    board.serial()
                );
                continue;
            };
            if conn.as_ref().is_some_and(|c| *c != port && !board.usb.matches_conn(c)) {
                continue;
            }
            match device(&board, &port, want, &variants) {
                Some(device) => devices.push(Box::new(device)),
                None => {
                    // Nothing to list it as: say what gets it.
                    log::warn!("A {MODEL} at {port} that does not run oxla: {}.", missing());
                }
            }
        }
        Ok(devices)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::UsbPort;
    use std::path::Path;

    fn port(name: &str, serial: &str) -> UsbPort {
        UsbPort {
            name: name.to_owned(),
            vid: VENDOR_ID,
            pid: PRODUCT_ID,
            serial: Some(serial.to_owned()),
            manufacturer: None,
            product: None,
        }
    }

    #[test]
    fn the_uart_is_the_second_port_of_the_debugger() {
        let ports = [
            port("/dev/ttyUSB10", "A"),
            port("/dev/ttyUSB9", "A"),
            port("/dev/ttyUSB3", "B"),
            port("/dev/ttyUSB2", "B"),
        ];
        assert_eq!(uart_of(Some("A"), &ports).as_deref(), Some("/dev/ttyUSB10"));
        assert_eq!(uart_of(Some("B"), &ports).as_deref(), Some("/dev/ttyUSB3"));
        // One port, or another device's, is no UART to go by.
        assert_eq!(uart_of(Some("C"), &ports), None);
        assert_eq!(uart_of(Some("A"), &ports[..1]), None);
        let other = UsbPort { pid: 0x6001, ..port("/dev/ttyUSB11", "A") };
        assert_eq!(uart_of(Some("A"), &[ports[0].clone(), other]), None);
    }

    /// The description of `la14-ddr` in release 0.2.0, as `make release`
    /// wrote it.
    const LA14_DDR: &str = include_str!("testdata/oxla-tn20k-la14-ddr.fs.json");

    #[test]
    fn a_description_is_the_variant_and_the_id_it_gave() {
        let s = Sidecar::parse(LA14_DDR, PathBuf::from("/fw/oxla-tn20k-la14-ddr.fs"))
            .unwrap()
            .expect("for this driver");
        assert_eq!(
            (s.variant.as_str(), s.default, s.version.as_str()),
            ("la14-ddr", false, "0.2.0")
        );
        assert_eq!((s.board.as_str(), s.idcode), (BOARD, gowin::IDCODE_GW2A_18));
        assert_eq!(s.file, Path::new("/fw/oxla-tn20k-la14-ddr.fs"));
        // The ID is the one the device gives: its name is the variant's.
        assert_eq!(s.id.variant(), "la14-ddr");
        assert_eq!((s.id.channels, s.id.mem_words, s.id.clock_hz), (14, 2_097_151, 108_000_000));
        assert!(s.id.has_ddr() && s.id.has_clocks() && s.id.has_description());
        assert_eq!(s.clocks_hz, [81_000_000, 108_000_000, 135_000_000]);
        assert_eq!(s.sha256.len(), 64);
        assert_eq!(for_this_board(vec![s]).len(), 1);
    }

    #[test]
    fn what_is_for_another_driver_is_left_and_what_is_wrong_is_said() {
        let file = || PathBuf::from("x.fs");
        for (from, to) in [
            ("\"format\": 1", "\"format\": 2"),
            ("\"design\": \"oxla\"", "\"design\": \"scope\""),
            ("gowin-jtag-sram", "gowin-jtag-flash"),
        ] {
            assert!(LA14_DDR.contains(from));
            assert_eq!(Sidecar::parse(&LA14_DDR.replace(from, to), file()), Ok(None), "{to}");
        }
        for (from, to, what) in [
            ("\"channels\": 14", "\"channels\": 15", "memory"),
            ("\"channels\": 14", "\"channels\": 300", "channels"),
            ("\"idcode\": \"0x0000081b\"", "\"idcode\": \"GW2A\"", "idcode"),
            ("\"sha256\": \"", "\"sha256\": \"zz", "sha256"),
            ("\"variant\"", "\"name\"", "variant"),
        ] {
            assert!(LA14_DDR.contains(from));
            let e = Sidecar::parse(&LA14_DDR.replace(from, to), file()).unwrap_err();
            assert!(e.contains(what), "{e}");
        }
        assert!(Sidecar::parse("{", file()).is_err());
        // Another board, or another part, is not this board's.
        let mut s = Sidecar::parse(LA14_DDR, file()).unwrap().unwrap();
        s.board = "sipeed-tang-primer-25k".to_owned();
        assert_eq!(for_this_board(vec![s.clone()]), []);
        s.board = BOARD.to_owned();
        s.idcode = 0x0001_281b;
        assert_eq!(for_this_board(vec![s]), []);
    }

    #[test]
    fn descriptions_are_found_beside_their_bitstreams() {
        let base = std::env::temp_dir().join(format!("oxrok-tn20k-{}", std::process::id()));
        let (first, second) = (base.join("a"), base.join("b"));
        std::fs::create_dir_all(&first).unwrap();
        std::fs::create_dir_all(&second).unwrap();
        let la16 = LA14_DDR
            .replace("la14-ddr", "la16-rle")
            .replace("\"default\": false", "\"default\": true");
        std::fs::write(first.join("oxla-tn20k-la14-ddr.fs.json"), LA14_DDR).unwrap();
        std::fs::write(second.join("oxla-tn20k-la16-rle.fs.json"), &la16).unwrap();
        // The first directory's counts; what is no description is passed.
        std::fs::write(second.join("oxla-tn20k-la14-ddr.fs.json"), &la16).unwrap();
        std::fs::write(second.join("oxla-tn20k-la8.fs.json"), "{").unwrap();
        std::fs::write(second.join("other.json"), &la16).unwrap();
        std::fs::write(second.join("oxla-tn20k-la8.fs"), "0101").unwrap();

        let found = sidecars_in(&[first.clone(), base.join("none"), second.clone()]);
        let names: Vec<(&str, &Path)> =
            found.iter().map(|s| (s.variant.as_str(), s.file.as_path())).collect();
        assert_eq!(
            names,
            [
                ("la14-ddr", first.join("oxla-tn20k-la14-ddr.fs").as_path()),
                ("la16-rle", second.join("oxla-tn20k-la16-rle.fs").as_path()),
            ]
        );
        assert_eq!(default_of(&found).map(|s| s.variant.as_str()), Some("la16-rle"));
        assert_eq!(default_of(&found[..1]).map(|s| s.variant.as_str()), Some("la14-ddr"));
        assert_eq!(default_of(&[]), None);

        // The file must be the one described.
        let s = &found[0];
        assert!(matches!(s.bitstream(), Err(Error::Firmware(m)) if m.contains("downloads")));
        std::fs::write(&s.file, "0101").unwrap();
        assert!(matches!(s.bitstream(), Err(Error::Firmware(m)) if m.contains("SHA-256")));
        std::fs::remove_dir_all(&base).unwrap();
    }

    #[test]
    fn a_checksum_is_lowercase_hex() {
        assert_eq!(
            sha256_hex(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
}
