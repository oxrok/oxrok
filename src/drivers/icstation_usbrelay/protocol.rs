// Counterpart in sigrok:
//   libsigrok/src/hardware/icstation-usbrelay/api.c,
//   libsigrok/src/hardware/icstation-usbrelay/protocol.c,
//   libsigrok/src/hardware/icstation-usbrelay/protocol.h,
//   Copyright (C) 2021-2023 Frank Stettner <frank-stettner@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire of the `ICStation` relay boards: single bytes at 9600/8n1, with no
//! framing.
//!
//! After power on, `50` asks for the board's id, one byte back, and `51` enters
//! command mode, with nothing back. In command mode every byte is the state of
//! all relays, active low, and nothing back. Nothing leaves that mode but a power
//! cycle, and a `50` or `51` sent there is a relay byte like any other.
//!
//! No request after the identification has an answer, so no late reply can meet
//! a reading: the cached relay state is all there is to read.

#![forbid(unsafe_code)]

use crate::core::{Error, Result};
use crate::serial::{Port, read_blocking};
use std::time::Duration;

/// The line, unless `serialcomm=` says otherwise.
pub const SERIALCOMM: &str = "9600/8n1";
/// What each read and write may take.
const SERIAL_TIMEOUT: Duration = Duration::from_millis(1000);
/// Asks for the id.
const CMD_ID: u8 = 0x50;
/// Enters command mode.
const CMD_START: u8 = 0x51;

/// A board.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// Its answer to [`CMD_ID`].
    pub id: u8,
    /// Its name.
    pub name: &'static str,
    /// Its relays, at most 8.
    pub relays: usize,
}

const PROFILES: [Profile; 3] = [
    Profile { id: 0xab, name: "ICSE012A", relays: 4 },
    Profile { id: 0xad, name: "ICSE013A", relays: 2 },
    Profile { id: 0xac, name: "ICSE014A", relays: 8 },
];

/// The board with this id.
pub fn profile(id: u8) -> Option<Profile> {
    PROFILES.iter().copied().find(|p| p.id == id)
}

/// A mask of `relays` bits.
pub fn relay_mask(relays: usize) -> u8 {
    u8::try_from((1u16 << relays) - 1).expect("at most 8 relays")
}

/// `state` with the relay of bit `relay` set to `on`, or all of `mask` when
/// `relay` is `None`.
pub fn switched(state: u8, mask: u8, relay: Option<usize>, on: bool) -> u8 {
    let bits = relay.map_or(mask, |r| 1 << r);
    if on { state | bits } else { state & !bits }
}

/// The byte for `state`: the board wants its relays active low, and no bit above
/// them.
pub fn wire_state(state: u8, mask: u8) -> u8 {
    !state & mask
}

/// `icstation_usbrelay_send_byte`: a write that takes none in time fails, where
/// `Port::write` would wait longer.
pub fn send_byte(port: &mut dyn Port, byte: u8) -> Result<()> {
    match port.write_timeout(&[byte], SERIAL_TIMEOUT) {
        Ok(1) => Ok(()),
        Ok(_) => Err(Error::Timeout),
        Err(e) => Err(e),
    }
}

/// Sends the identification request and reads its answer. The board switches
/// relays on a request it takes as a state, if it was started before: no other
/// way to name a board exists.
pub fn identify(port: &mut dyn Port) -> Result<u8> {
    send_byte(port, CMD_ID).inspect_err(|_| {
        log::debug!("Could not send identification request.");
    })?;
    let mut id = [0u8];
    let got = read_blocking(port, &mut id, SERIAL_TIMEOUT);
    match got {
        Ok(1) => {
            log::debug!("Identification response 0x{:02x}.", id[0]);
            Ok(id[0])
        }
        Ok(_) => {
            log::debug!("Could not receive identification response.");
            Err(Error::Timeout)
        }
        Err(e) => {
            log::debug!("Could not receive identification response.");
            Err(e)
        }
    }
}

/// Enters command mode.
pub fn start(port: &mut dyn Port) -> Result<()> {
    send_byte(port, CMD_START)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_table() {
        assert_eq!(profile(0xab).map(|p| (p.name, p.relays)), Some(("ICSE012A", 4)));
        assert_eq!(profile(0xad).map(|p| (p.name, p.relays)), Some(("ICSE013A", 2)));
        assert_eq!(profile(0xac).map(|p| (p.name, p.relays)), Some(("ICSE014A", 8)));
        assert_eq!(profile(0x50), None);
    }

    #[test]
    fn masks_reach_all_eight() {
        assert_eq!(relay_mask(2), 0x03);
        assert_eq!(relay_mask(4), 0x0f);
        assert_eq!(relay_mask(8), 0xff);
    }

    #[test]
    fn states_and_their_bytes() {
        assert_eq!(switched(0, 0x0f, Some(0), true), 1);
        assert_eq!(switched(0b1010, 0x0f, Some(3), false), 0b0010);
        assert_eq!(switched(0b0010, 0x0f, None, true), 0x0f);
        assert_eq!(switched(0b0010, 0x0f, None, false), 0);
        assert_eq!(switched(0b0010, 0x0f, Some(0), true), 0b0011, "the others stay");
        assert_eq!(wire_state(0, 0x0f), 0x0f);
        assert_eq!(wire_state(0x0f, 0x0f), 0);
        assert_eq!(wire_state(1, 0x03), 0x02);
        assert_eq!(wire_state(0x80, 0xff), 0x7f);
    }
}
