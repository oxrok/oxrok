// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! `oxrok-emu gpib-config`: what linux-gpib's `gpib_config` does for a board, for
//! the emulated GPIB adapters, so that a tester needs no linux-gpib userspace.
//!
//! The kernel's `gpib` subsystem binds an adapter's USB interface at the plug, but
//! a board (`/dev/gpibN`) goes online only when told its type and brought online,
//! both of which need `CAP_SYS_ADMIN` (`drivers/gpib/common/gpib_os.c`,
//! `board_type_ioctl` and `online_ioctl`). This follows `gpib_config.c`'s
//! `configure_board`: offline, `CFCBOARDTYPE`, `IBPAD`, `IBSAD`, the device path,
//! online, then with the board's lock `IBSRE`, `IBRSC` and `IBSIC`, as its defaults
//! do (remote enable, system controller, interface clear). The device path is the
//! emulated interface's, so the board takes no other adapter. Then the node is made
//! world readable and writable, as `--open-devices` does for the USB nodes, so the
//! tools run as a user.
//!
//! A machine's own boards are left alone. The board taken is one this command took
//! before (a marker in [`MARKS`] names it, with the node's mode before), or else the
//! first of minors 0 to 14 with no board type at all (`IBBOARD_INFO` answers
//! `ENODEV`). Minor 15 is never taken: a case can name it as a board that is not
//! configured. `--release` takes each marked board offline, puts its node's mode
//! back and drops the marker, at a session's end.

use std::fs::{File, OpenOptions};
use std::io;
use std::os::unix::fs::PermissionsExt as _;
use std::path::Path;

use rustix::ioctl::{Opcode, Updater, ioctl, opcode};

/// `GPIB_CODE` of `gpib_ioctl.h`.
const GPIB_CODE: u8 = 160;

/// Where the boards this command took are marked, one file per minor holding the
/// node's mode before, in octal.
pub const MARKS: &str = "/run/oxrok-emu-gpib";

/// The minors a session may take; 15 stays unconfigured for the cases.
const MINORS: std::ops::Range<u32> = 0..15;

#[repr(C)]
struct BoardType {
    name: [u8; 100],
}

#[repr(C)]
struct OnlineCmd {
    init_data_ptr: u64,
    init_data_length: i32,
    online: i32,
}

#[repr(C)]
struct Pad {
    handle: u32,
    pad: u32,
}

#[repr(C)]
struct Sad {
    handle: u32,
    sad: i32,
}

#[repr(C)]
struct DevicePath {
    path: [u8; 0x1000],
}

#[repr(C)]
struct Eos {
    eos: i32,
    eos_flags: i32,
}

/// `struct gpib_board_info_ioctl`, its bit fields as one word.
#[repr(C)]
#[derive(Default)]
struct BoardInfo {
    pad: u32,
    sad: i32,
    parallel_poll_configuration: i32,
    autopolling: i32,
    is_system_controller: i32,
    t1_delay: u32,
    bits: u32,
}

const IBSIC: Opcode = opcode::write::<u32>(GPIB_CODE, 9);
const IBSRE: Opcode = opcode::write::<i32>(GPIB_CODE, 10);
const IBPAD: Opcode = opcode::write::<Pad>(GPIB_CODE, 15);
const IBSAD: Opcode = opcode::write::<Sad>(GPIB_CODE, 16);
const IBEOS: Opcode = opcode::write::<Eos>(GPIB_CODE, 19);
const CFCBOARDTYPE: Opcode = opcode::write::<BoardType>(GPIB_CODE, 24);
const IBMUTEX: Opcode = opcode::write::<i32>(GPIB_CODE, 26);
const IBBOARD_INFO: Opcode = opcode::read::<BoardInfo>(GPIB_CODE, 29);
const IBRSC: Opcode = opcode::write::<i32>(GPIB_CODE, 34);
const IBONL: Opcode = opcode::write::<OnlineCmd>(GPIB_CODE, 39);
const IBSELECT_DEVICE_PATH: Opcode = opcode::write::<DevicePath>(GPIB_CODE, 43);

/// `REOS | BIN` of `gpib.h`.
const REOS_BIN: i32 = 0x0400 | 0x1000;
/// `ENODEV`.
const ENODEV: i32 = 19;

/// One ioctl whose argument the kernel reads, and maybe writes back.
///
/// # Safety
///
/// `OP` must be the opcode `gpib_ioctl.h` gives the struct `T`.
unsafe fn call<const OP: Opcode, T>(board: &File, what: &str, value: &mut T) -> io::Result<()> {
    // SAFETY: the caller pairs the opcode with its struct, which is `repr(C)` as
    // the header has it, and it lives for the call.
    unsafe { ioctl(board, Updater::<OP, T>::new(value)) }
        .map_err(|e| io::Error::new(io::Error::from(e).kind(), format!("{what}: {e}")))
}

fn node(minor: u32) -> String {
    format!("/dev/gpib{minor}")
}

fn open(minor: u32) -> io::Result<File> {
    let node = node(minor);
    OpenOptions::new()
        .read(true)
        .write(true)
        .open(&node)
        .map_err(|e| io::Error::new(e.kind(), format!("{node}: {e} (is gpib_common loaded?)")))
}

fn mark(minor: u32) -> String {
    format!("{MARKS}/{minor}")
}

/// Whether board `minor` has no board type, so belongs to nobody.
fn unconfigured(minor: u32) -> bool {
    let Ok(board) = open(minor) else { return false };
    let mut info = BoardInfo::default();
    // SAFETY: IBBOARD_INFO fills a gpib_board_info_ioctl.
    let r = unsafe { ioctl(&board, Updater::<IBBOARD_INFO, BoardInfo>::new(&mut info)) };
    matches!(r, Err(e) if e.raw_os_error() == ENODEV)
}

/// The board to take: one marked as this command's, or the first with no type.
fn choose() -> io::Result<u32> {
    if let Some(m) = MINORS.clone().find(|m| Path::new(&mark(*m)).exists()) {
        return Ok(m);
    }
    MINORS.clone().find(|m| unconfigured(*m)).ok_or_else(|| {
        io::Error::other("no unconfigured GPIB board among /dev/gpib0 to /dev/gpib14")
    })
}

/// Configures a board as an `ni_usb_b` on the USB device `busid`'s first interface,
/// marks it, and returns its minor.
///
/// # Errors
///
/// When no board is free, or the node, the interface or an ioctl fails, naming which.
pub fn configure(busid: Option<&str>) -> io::Result<u32> {
    let minor = choose()?;
    let node = node(minor);
    let board = open(minor)?;
    if !Path::new(&mark(minor)).exists() {
        let mode = std::fs::metadata(&node)?.permissions().mode() & 0o7777;
        std::fs::create_dir_all(MARKS)?;
        std::fs::write(mark(minor), format!("{mode:o}\n"))?;
    }
    let mut off = OnlineCmd { init_data_ptr: 0, init_data_length: 0, online: 0 };
    // SAFETY: each opcode below is paired with its own struct.
    unsafe {
        call::<IBONL, _>(&board, "IBONL 0", &mut off)?;
        let mut kind = BoardType { name: [0; 100] };
        kind.name[..8].copy_from_slice(b"ni_usb_b");
        call::<CFCBOARDTYPE, _>(
            &board,
            "CFCBOARDTYPE ni_usb_b (is ni_usb_gpib loaded?)",
            &mut kind,
        )?;
        call::<IBPAD, _>(&board, "IBPAD", &mut Pad { handle: 0, pad: 0 })?;
        call::<IBSAD, _>(&board, "IBSAD", &mut Sad { handle: 0, sad: -1 })?;
        if let Some(busid) = busid {
            let path = interface_path(busid)?;
            let mut selection = DevicePath { path: [0; 0x1000] };
            selection.path[..path.len()].copy_from_slice(path.as_bytes());
            call::<IBSELECT_DEVICE_PATH, _>(&board, "IBSELECT_DEVICE_PATH", &mut selection)?;
        }
        let mut on = OnlineCmd { init_data_ptr: 0, init_data_length: 0, online: 1 };
        call::<IBONL, _>(&board, "IBONL 1 (no adapter bound to ni_usb_gpib?)", &mut on)?;
        call::<IBMUTEX, _>(&board, "IBMUTEX 1", &mut 1i32)?;
        let controlled = (|| {
            call::<IBSRE, _>(&board, "IBSRE", &mut 1i32)?;
            call::<IBRSC, _>(&board, "IBRSC", &mut 1i32)?;
            call::<IBSIC, _>(&board, "IBSIC", &mut 100u32)
        })();
        call::<IBMUTEX, _>(&board, "IBMUTEX 0", &mut 0i32)?;
        controlled?;
    }
    std::fs::set_permissions(&node, std::fs::Permissions::from_mode(0o666))?;
    Ok(minor)
}

/// Leaves board `minor` in an end of string mode on `byte`, 8-bit compare, as a
/// program before might have: what a tool's reads must clear.
///
/// # Errors
///
/// When the node or an ioctl fails.
pub fn set_eos(minor: u32, byte: u8) -> io::Result<()> {
    let board = open(minor)?;
    // SAFETY: each opcode below is paired with its own struct.
    unsafe {
        call::<IBMUTEX, _>(&board, "IBMUTEX 1", &mut 1i32)?;
        let mut eos = Eos { eos: i32::from(byte), eos_flags: REOS_BIN };
        let set = call::<IBEOS, _>(&board, "IBEOS", &mut eos);
        call::<IBMUTEX, _>(&board, "IBMUTEX 0", &mut 0i32)?;
        set
    }
}

/// Takes each board this command took offline, puts its node's mode back and drops
/// its marker. Returns how many.
///
/// # Errors
///
/// When a marker can't be read or an ioctl fails; the other boards are still done.
pub fn release() -> io::Result<u32> {
    let mut done = 0;
    let mut first_error = None;
    for minor in MINORS {
        let Ok(text) = std::fs::read_to_string(mark(minor)) else { continue };
        let r = (|| {
            let mode = u32::from_str_radix(text.trim(), 8)
                .map_err(|_| io::Error::other(format!("{}: no mode", mark(minor))))?;
            let board = open(minor)?;
            let mut off = OnlineCmd { init_data_ptr: 0, init_data_length: 0, online: 0 };
            // SAFETY: IBONL takes a gpib_online_ioctl.
            unsafe { call::<IBONL, _>(&board, "IBONL 0", &mut off)? };
            std::fs::set_permissions(node(minor), std::fs::Permissions::from_mode(mode))?;
            std::fs::remove_file(mark(minor))
        })();
        match r {
            Ok(()) => done += 1,
            Err(e) => {
                first_error.get_or_insert(e);
            }
        }
    }
    first_error.map_or(Ok(done), Err)
}

/// The kobject path the kernel compares (`gpib_match_device_path`): the interface's
/// sysfs path without `/sys`.
fn interface_path(busid: &str) -> io::Result<String> {
    let link = format!("/sys/bus/usb/devices/{busid}:1.0");
    let real = std::fs::canonicalize(Path::new(&link))
        .map_err(|e| io::Error::new(e.kind(), format!("{link}: {e}")))?;
    let text = real.to_string_lossy();
    let path = text.strip_prefix("/sys").unwrap_or(&text).to_owned();
    if path.len() >= 0x1000 {
        return Err(io::Error::other(format!("{path}: too long a device path")));
    }
    Ok(path)
}
