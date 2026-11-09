//! Parallel ports through Linux's `ppdev`, as libieee1284 drives them.
//!
//! libsigrok's parallel port driver (`hung-chang-dso-2100`) goes through
//! libieee1284, which on Linux finds the ports in `/proc/sys/dev/parport`
//! and opens each as `/dev/parportN`, with `ppdev`'s ioctls for every line
//! change. This module does the same calls in the same order, so the port
//! sees the same accesses: on a USB cable (`uss720`) each is a request on
//! the wire, and the emulator compares them.
//!
//! What libieee1284 does that is mirrored here:
//!
//! - [`find_ports`]: the first listing of a process probes `/dev/parport0`
//!   to `7` (and `/dev/parports/N`) for one that opens, and claims and
//!   releases it once "to force the low-level port driver to get loaded"
//!   (`detect.c`, `check_dev_node`); then every directory of
//!   `/proc/sys/dev/parport` but `default` is a port, sorted by name,
//!   whose device is `/dev/parport` and the name's trailing digits.
//! - [`Parport::open`] opens the device read and write and asks its
//!   modes with a claim, `PPGETMODES` and a release (`access_ppdev.c`,
//!   `find_capabilities`); [`Caps`] is what that gives.
//! - [`Parport::write_control`] takes libieee1284's control bits, in
//!   which a set `C1284_NSTROBE`, `C1284_NAUTOFD` or `C1284_NSELECTIN`
//!   is a high line, and inverts those three for the PC register
//!   `PPWCONTROL` takes; only the low four bits go.
//!
//! Errors: libieee1284 ignores a failed write ioctl; here a failed one is
//! an error too. A port that goes away (its cable pulled) keeps answering
//! the ioctls of an open descriptor, with the kernel's dead operations
//! reading 0, so [`Parport::gone`] looks at whether the port is still the
//! one that was opened.

use crate::core::{Error, Result};
use std::path::PathBuf;

/// libieee1284's control bits: nStrobe high.
pub const C1284_NSTROBE: u8 = 0x01;
/// nAutoFd high.
pub const C1284_NAUTOFD: u8 = 0x02;
/// nInit high.
pub const C1284_NINIT: u8 = 0x04;
/// nSelectIn high.
pub const C1284_NSELECTIN: u8 = 0x08;
/// The bits the PC's control register holds inverted.
pub const C1284_INVERTED: u8 = C1284_NSTROBE | C1284_NAUTOFD | C1284_NSELECTIN;

/// `PARPORT_MODE_PCSPP`: the PC's registers.
pub const MODE_PCSPP: u32 = 1 << 0;
/// `PARPORT_MODE_TRISTATE`: the data lines can be read.
pub const MODE_TRISTATE: u32 = 1 << 1;

/// How long a claim waits before the next access. `uss720` restores the
/// port's registers at a claim with two writes and a read it does not wait
/// for (`parport_uss720_restore_state`); that read's answer, coming late,
/// overwrites the driver's soft copy of the control register, so a
/// direction set right after the claim (or `ppdev`'s IRQ enable bit) is lost from the next control
/// write (seen in the emulator: `PPDATADIR` then `PPWCONTROL` sent `0x11`
/// for `0x31`). A USB round trip is a few ms; 20 ms covers three under
/// load. libieee1284 does not wait, and can meet the same race.
///
/// A waited read after the claim (`PPRSTATUS`) would be an exact barrier,
/// as the control endpoint completes in order, but it is a request
/// libieee1284 does not send: the cable would see other transactions than
/// under sigrok, and the emulator compares them. So the claim sleeps, and
/// the emulator's oracle sleeps as long (`tools/oracle-ppclaim-settle.c`).
pub const CLAIM_SETTLE: std::time::Duration = std::time::Duration::from_millis(20);

/// What a port can do, from its modes, as libieee1284's capabilities
/// that matter here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Caps {
    /// `CAP1284_RAW`: the lines can be driven one by one.
    pub raw: bool,
    /// `CAP1284_BYTE`: the data lines can be read.
    pub byte: bool,
}

impl Caps {
    /// From `PPGETMODES`'s answer. libieee1284 starts with `CAP1284_BYTE`
    /// and takes it away without a tristate mode.
    pub fn from_modes(modes: u32) -> Caps {
        Caps { raw: modes & MODE_PCSPP != 0, byte: modes & MODE_TRISTATE != 0 }
    }
}

/// A port of the listing.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PortInfo {
    /// Its name, `parportN`, which `conn=` takes.
    pub name: String,
    /// Its device node.
    pub device: PathBuf,
}

/// The PC control register `PPWCONTROL` takes for libieee1284's control
/// bits: the low four, with nStrobe, nAutoFd and nSelectIn inverted.
pub fn pc_control(control: u8) -> u8 {
    (control & 0x0f) ^ C1284_INVERTED
}

/// The device node of a port named `name`: `/dev/parport` and the
/// name's trailing digits, as libieee1284's `populate_from_sys_dev_parport`
/// builds it. `None` for a name with none.
pub fn device_of(name: &str) -> Option<PathBuf> {
    let digits = name.len() - name.trim_end_matches(|c: char| c.is_ascii_digit()).len();
    (digits > 0).then(|| PathBuf::from(format!("/dev/parport{}", &name[name.len() - digits..])))
}

/// The ports, as `ieee1284_find_ports` lists them, sorted by name.
pub fn find_ports() -> Vec<PortInfo> {
    sys::detect();
    let Ok(dir) = std::fs::read_dir("/proc/sys/dev/parport") else { return Vec::new() };
    let mut ports: Vec<PortInfo> = dir
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().into_owned();
            if name == "default" {
                return None;
            }
            // libieee1284 lists a name with no digits too, with no device,
            // which no open then finds: the same, with an empty path.
            let device = device_of(&name).unwrap_or_default();
            Some(PortInfo { name, device })
        })
        .collect();
    ports.sort_by(|a, b| a.name.cmp(&b.name));
    ports
}

/// The sysfs device behind the port's node, which a replug changes.
///
/// The device's sysfs path, and for a USB device its bus and device
/// numbers: a cable plugged back gets the same path when it goes into the
/// same port, and the port the same name when the kernel hands the number
/// out again, but never the same device number.
fn identity(name: &str) -> Option<(PathBuf, Option<String>)> {
    let path = std::fs::canonicalize(format!("/sys/class/ppdev/{name}/device")).ok()?;
    let read = |f: &str| std::fs::read_to_string(path.join(f)).ok().map(|v| v.trim().to_owned());
    let usb = match (read("busnum"), read("devnum")) {
        (Some(b), Some(d)) => Some(format!("{b}.{d}")),
        _ => None,
    };
    Some((path, usb))
}

/// An open port.
#[derive(Debug)]
pub struct Parport {
    name: String,
    file: std::fs::File,
    identity: Option<(PathBuf, Option<String>)>,
}

impl Parport {
    /// Opens the port as `ieee1284_open` does, with its capabilities.
    ///
    /// # Errors
    ///
    /// When the node does not open.
    pub fn open(info: &PortInfo) -> Result<(Parport, Caps)> {
        let file = std::fs::OpenOptions::new().read(true).write(true).open(&info.device)?;
        let port = Parport { name: info.name.clone(), identity: identity(&info.name), file };
        let caps = port.capabilities();
        Ok((port, caps))
    }

    /// The port's name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `find_capabilities`: a claim, the modes, a release. A port that
    /// cannot be claimed or asked is guessed to have the extended modes,
    /// which here are none of the two that matter, as there.
    fn capabilities(&self) -> Caps {
        let all = Caps { raw: false, byte: true };
        if sys::claim(&self.file).is_err() {
            return all;
        }
        std::thread::sleep(CLAIM_SETTLE);
        let modes = sys::modes(&self.file);
        sys::release(&self.file);
        match modes {
            Ok(m) => Caps::from_modes(m),
            Err(_) => all,
        }
    }

    /// `ieee1284_claim`.
    ///
    /// # Errors
    ///
    /// When another holds the port.
    pub fn claim(&self) -> Result<()> {
        sys::claim(&self.file)?;
        std::thread::sleep(CLAIM_SETTLE);
        Ok(())
    }

    /// `ieee1284_release`.
    pub fn release(&self) {
        sys::release(&self.file);
    }

    /// `ieee1284_read_data`.
    ///
    /// # Errors
    ///
    /// When the ioctl fails.
    pub fn read_data(&self) -> Result<u8> {
        sys::read_data(&self.file)
    }

    /// `ieee1284_write_data`.
    ///
    /// # Errors
    ///
    /// When the ioctl fails.
    pub fn write_data(&self, value: u8) -> Result<()> {
        sys::write_data(&self.file, value)
    }

    /// `ieee1284_write_control`, in libieee1284's bits: see [`pc_control`].
    ///
    /// # Errors
    ///
    /// When the ioctl fails.
    pub fn write_control(&self, control: u8) -> Result<()> {
        sys::write_control(&self.file, pc_control(control))
    }

    /// `ieee1284_data_dir`: the data lines read (`reverse`) or driven.
    ///
    /// # Errors
    ///
    /// When the ioctl fails.
    pub fn data_dir(&self, reverse: bool) -> Result<()> {
        sys::data_dir(&self.file, reverse)
    }

    /// Whether the port went away since it was opened: its node's device
    /// is gone, or is another one.
    pub fn gone(&self) -> bool {
        match &self.identity {
            Some(was) => identity(&self.name).as_ref() != Some(was),
            None => !std::path::Path::new(&format!("/proc/sys/dev/parport/{}", self.name)).exists(),
        }
    }
}

#[cfg(target_os = "linux")]
mod sys {
    //! The `ppdev` ioctls (`include/uapi/linux/ppdev.h`).

    use super::{Error, Result};
    use rustix::ioctl::{Getter, NoArg, Opcode, Setter, ioctl, opcode};
    use std::os::fd::AsFd;
    use std::sync::Once;

    const PP_IOCTL: u8 = b'p';
    const PPWCONTROL: Opcode = opcode::write::<u8>(PP_IOCTL, 0x84);
    const PPRDATA: Opcode = opcode::read::<u8>(PP_IOCTL, 0x85);
    const PPWDATA: Opcode = opcode::write::<u8>(PP_IOCTL, 0x86);
    const PPCLAIM: Opcode = opcode::none(PP_IOCTL, 0x8b);
    const PPRELEASE: Opcode = opcode::none(PP_IOCTL, 0x8c);
    const PPDATADIR: Opcode = opcode::write::<i32>(PP_IOCTL, 0x90);
    const PPGETMODES: Opcode = opcode::read::<u32>(PP_IOCTL, 0x97);

    fn io(e: rustix::io::Errno) -> Error {
        Error::Io(e.into())
    }

    pub(super) fn claim(fd: impl AsFd) -> Result<()> {
        // SAFETY: PPCLAIM takes no argument.
        unsafe { ioctl(fd, NoArg::<PPCLAIM>::new()) }.map_err(io)
    }

    pub(super) fn release(fd: impl AsFd) {
        // SAFETY: PPRELEASE takes no argument. Its failure (a port not
        // claimed) is ignored, as libieee1284 ignores it.
        let _ = unsafe { ioctl(fd, NoArg::<PPRELEASE>::new()) };
    }

    pub(super) fn modes(fd: impl AsFd) -> Result<u32> {
        // SAFETY: PPGETMODES writes one unsigned int.
        unsafe { ioctl(fd, Getter::<PPGETMODES, u32>::new()) }.map_err(io)
    }

    pub(super) fn read_data(fd: impl AsFd) -> Result<u8> {
        // SAFETY: PPRDATA writes one unsigned char.
        unsafe { ioctl(fd, Getter::<PPRDATA, u8>::new()) }.map_err(io)
    }

    pub(super) fn write_data(fd: impl AsFd, value: u8) -> Result<()> {
        // SAFETY: PPWDATA reads one unsigned char.
        unsafe { ioctl(fd, Setter::<PPWDATA, u8>::new(value)) }.map_err(io)
    }

    pub(super) fn write_control(fd: impl AsFd, value: u8) -> Result<()> {
        // SAFETY: PPWCONTROL reads one unsigned char.
        unsafe { ioctl(fd, Setter::<PPWCONTROL, u8>::new(value)) }.map_err(io)
    }

    pub(super) fn data_dir(fd: impl AsFd, reverse: bool) -> Result<()> {
        // SAFETY: PPDATADIR reads one int.
        unsafe { ioctl(fd, Setter::<PPDATADIR, i32>::new(i32::from(reverse))) }.map_err(io)
    }

    /// libieee1284's `detect_environment`, once a process: the first
    /// `/dev/parportN` that opens is claimed and released.
    pub(super) fn detect() {
        static ONCE: Once = Once::new();
        ONCE.call_once(|| {
            for i in 0..8 {
                let open = |p: String| std::fs::OpenOptions::new().read(true).open(p).ok();
                let file =
                    open(format!("/dev/parport{i}")).or_else(|| open(format!("/dev/parports/{i}")));
                if let Some(f) = file {
                    if claim(&f).is_ok() {
                        std::thread::sleep(super::CLAIM_SETTLE);
                        release(&f);
                    }
                    return;
                }
            }
        });
    }
}

#[cfg(not(target_os = "linux"))]
mod sys {
    //! No `ppdev` here: no port is listed, so nothing below is reached.

    use super::{Error, Result};

    fn none() -> Error {
        Error::not_supported("parallel ports need Linux's ppdev")
    }

    pub(super) fn claim(_fd: &std::fs::File) -> Result<()> {
        Err(none())
    }

    pub(super) fn release(_fd: &std::fs::File) {}

    pub(super) fn modes(_fd: &std::fs::File) -> Result<u32> {
        Err(none())
    }

    pub(super) fn read_data(_fd: &std::fs::File) -> Result<u8> {
        Err(none())
    }

    pub(super) fn write_data(_fd: &std::fs::File, _value: u8) -> Result<()> {
        Err(none())
    }

    pub(super) fn write_control(_fd: &std::fs::File, _value: u8) -> Result<()> {
        Err(none())
    }

    pub(super) fn data_dir(_fd: &std::fs::File, _reverse: bool) -> Result<()> {
        Err(none())
    }

    pub(super) fn detect() {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn devices_from_names() {
        assert_eq!(device_of("parport0"), Some(PathBuf::from("/dev/parport0")));
        assert_eq!(device_of("parport12"), Some(PathBuf::from("/dev/parport12")));
        assert_eq!(device_of("lpt"), None);
    }

    #[test]
    fn control_bits_are_inverted_as_the_pc_has_them() {
        // libieee1284's bits, a set bit a high line, to the PC register.
        assert_eq!(pc_control(0x0f), 0x04);
        assert_eq!(pc_control(0x00), 0x0b);
        assert_eq!(pc_control(C1284_NSTROBE | C1284_NAUTOFD | C1284_NSELECTIN), 0x00);
        assert_eq!(pc_control(C1284_NINIT), 0x0f);
        assert_eq!(pc_control(0xf1), 0x0a, "only the low four bits go");
    }

    #[test]
    fn caps_from_modes() {
        // uss720: PCSPP, TRISTATE, EPP, COMPAT (and ECP with 3 endpoints).
        assert_eq!(Caps::from_modes(0x17), Caps { raw: true, byte: true });
        assert_eq!(Caps::from_modes(0x01), Caps { raw: true, byte: false });
        assert_eq!(Caps::from_modes(0x02), Caps { raw: false, byte: true });
    }
}
