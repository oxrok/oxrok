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
//! emulated interface's, so a real adapter on the machine is never taken. Then the
//! node is made world readable and writable, as `--open-devices` does for the USB
//! nodes, so the tools run as a user.

use std::fs::{File, OpenOptions};
use std::io;
use std::os::unix::fs::PermissionsExt as _;
use std::path::Path;

use rustix::ioctl::{Opcode, Updater, ioctl, opcode};

/// `GPIB_CODE` of `gpib_ioctl.h`.
const GPIB_CODE: u8 = 160;

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

const IBSIC: Opcode = opcode::write::<u32>(GPIB_CODE, 9);
const IBSRE: Opcode = opcode::write::<i32>(GPIB_CODE, 10);
const IBPAD: Opcode = opcode::write::<Pad>(GPIB_CODE, 15);
const IBSAD: Opcode = opcode::write::<Sad>(GPIB_CODE, 16);
const CFCBOARDTYPE: Opcode = opcode::write::<BoardType>(GPIB_CODE, 24);
const IBMUTEX: Opcode = opcode::write::<i32>(GPIB_CODE, 26);
const IBRSC: Opcode = opcode::write::<i32>(GPIB_CODE, 34);
const IBONL: Opcode = opcode::write::<OnlineCmd>(GPIB_CODE, 39);
const IBSELECT_DEVICE_PATH: Opcode = opcode::write::<DevicePath>(GPIB_CODE, 43);

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

/// Configures board `minor` as an `ni_usb_b` on the USB device `busid`'s first
/// interface, or only takes it offline.
///
/// # Errors
///
/// When the node, the interface or an ioctl fails, naming which.
pub fn configure(minor: u32, busid: Option<&str>, offline: bool) -> io::Result<()> {
    let node = format!("/dev/gpib{minor}");
    let board =
        OpenOptions::new().read(true).write(true).open(&node).map_err(|e| {
            io::Error::new(e.kind(), format!("{node}: {e} (is gpib_common loaded?)"))
        })?;
    let mut off = OnlineCmd { init_data_ptr: 0, init_data_length: 0, online: 0 };
    // SAFETY: each opcode below is paired with its own struct.
    unsafe {
        call::<IBONL, _>(&board, "IBONL 0", &mut off)?;
        if offline {
            return Ok(());
        }
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
    std::fs::set_permissions(&node, std::fs::Permissions::from_mode(0o666))
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
