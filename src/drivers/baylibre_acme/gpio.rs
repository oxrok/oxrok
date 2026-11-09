// Counterpart in sigrok:
//   libsigrok/src/hardware/baylibre-acme/gpio.c,
//   libsigrok/src/hardware/baylibre-acme/gpio.h,
//   Copyright (C) 2015 Bartosz Golaszewski <bgolaszewski@baylibre.com>,
//   with commits by Daniel Elstner, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The kernel's GPIO files (`/sys/class/gpio`, its sysfs interface): `export`
//! makes `gpioN`, whose `direction` is `in` or `out` (`high` and `low` set the
//! level with it) and whose `value` is 0 or 1. Its log lines are C's `gpio`
//! ones.
//!
//! C exports a GPIO by writing its `direction` first (`gpio.c:67-73`), which the
//! kernel has no file for until it is exported, so the export never comes. Here
//! `export` goes first. A line read is left as it is: C sets it as an output
//! first, which for a probe's sense line would drive an input.

use std::time::{Duration, Instant};

use super::protocol::{Sysfs, parse_value, strerror};
use crate::core::{Error, Result};

/// `sr_gpio_export`, in the order the kernel needs: `export` first. For an
/// output, `direction` then sets the level with it (`high` or `low`), so a
/// switch exported to be turned on never goes off on the way. Whether it
/// exported it.
fn export(sysfs: &Sysfs, gpio: u32, level: Option<bool>) -> Result<bool> {
    if sysfs.is_dir(&format!("/sys/class/gpio/gpio{gpio}")) {
        return Ok(false); // Already exported.
    }
    write(sysfs, "/sys/class/gpio/export", &format!("{gpio}\n"))?;
    settle(sysfs, gpio);
    if let Some(high) = level {
        let direction = if high { "high" } else { "low" };
        write(sysfs, &format!("/sys/class/gpio/gpio{gpio}/direction"), &format!("{direction}\n"))?;
    }
    Ok(true)
}

/// Writes a GPIO's file, saying what failed in C's words (`open_and_write`).
fn write(sysfs: &Sysfs, path: &str, text: &str) -> Result<()> {
    sysfs.write_raw(path, text).map_err(|(opening, e)| {
        if opening {
            log::error!("Error opening {path}: {}", strerror(&e));
        } else {
            log::error!("Error writing to {path}: {}", strerror(&e));
        }
        Error::Io(e)
    })
}

/// How long an export may take to show its files.
const SETTLE: Duration = Duration::from_secs(1);

/// Waits, a while at most, for the files of a GPIO just exported. The kernel
/// makes them in the write, but udev gives them their modes after it on a real
/// system, and an emulated sysfs makes them after the write is seen.
fn settle(sysfs: &Sysfs, gpio: u32) {
    let value = sysfs.path(&format!("/sys/class/gpio/gpio{gpio}/value"));
    let began = Instant::now();
    while !value.exists() && began.elapsed() < SETTLE {
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// `sr_gpio_setval_export`.
///
/// # Errors
///
/// When a file can't be written, said in C's words.
///
/// A switch exported as an input (by a read here, or by anyone) takes no
/// `value`: the kernel's write(2) fails with EPERM. C sets the direction to
/// `out` once, right after its export (`b41bbfdb`), and never sees that failure.
/// Here a `value` that won't take is followed by `direction`, which makes the
/// line an output with its level. Reading the direction first would cost every
/// set a read C doesn't make.
pub fn setval_export(sysfs: &Sysfs, gpio: u32, high: bool) -> Result<()> {
    if export(sysfs, gpio, Some(high))? {
        return Ok(());
    }
    let value = format!("/sys/class/gpio/gpio{gpio}/value");
    match sysfs.write_raw(&value, &format!("{}\n", u8::from(high))) {
        Ok(()) => Ok(()),
        Err((true, e)) => {
            log::error!("Error opening {value}: {}", strerror(&e));
            Err(Error::Io(e))
        }
        Err((false, e)) => {
            log::warn!("Error writing to {value}: {}; setting its direction", strerror(&e));
            let level = if high { "high" } else { "low" };
            write(sysfs, &format!("/sys/class/gpio/gpio{gpio}/direction"), &format!("{level}\n"))
        }
    }
}

/// `sr_gpio_getval_export`: the level, the line's direction left alone.
///
/// # Errors
///
/// When the export fails, or the value can't be read or holds no number.
pub fn getval_export(sysfs: &Sysfs, gpio: u32) -> Result<i64> {
    export(sysfs, gpio, None)?;
    let path = format!("/sys/class/gpio/gpio{gpio}/value");
    let data = std::fs::read(sysfs.path(&path)).map_err(|e| {
        log::error!("Error opening {path}: {}", strerror(&e));
        Error::Io(e)
    })?;
    parse_value(&data).ok_or_else(|| {
        log::error!("Error reading from {path}: not a number");
        Error::device(format!("{path} holds no level"))
    })
}
