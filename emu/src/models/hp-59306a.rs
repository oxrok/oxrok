// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The HP 59306A relay actuator: six relays, each joining its common contact C to
//! A or to B. Really on HP-IB, which no shell carries, and here on a socket, as
//! `tcp-raw/HOST/PORT` would reach it through a bridge. libsigrok's `scpi.c` is
//! the same either way, so what goes out is what `scpi_libgpib.c` would be given.
//!
//! The device, and how sure this is. The HP manual could not be read here. These
//! come from the time-nuts thread of May 2008 (febo.com, messages 031434 to
//! 031449), people who owned one and had the manual open:
//!
//! - There is no microcontroller, only TTL, and it only listens: the state of the
//!   relays cannot be read back. So this model never sends anything, and no `*IDN?`
//!   is possible, which is what `api.c:52-58` says.
//! - `A1` joins A and C on relay 1, and `B123456` joins B and C on them all (a
//!   user's own program, 031435). The manual's example goes byte by byte, `A`, `3`,
//!   `5`, `B`, `3`, `A`, as its author reads table 3-3 (031434), so a letter stays
//!   until the next one here. `A35B6` in one message works here as a guess: the same
//!   author says "It's not clear to me if this can accept multiple commands at once".
//! - A line end is the thread's open question: one owner says it is ignored
//!   (031440), others not to send one (031437, 031443). libsigrok sends one with
//!   every command (`scpi.c:161-162`). This model ignores it, and a real one may not.
//! - 031443 also says to send a remote command, `<$>`, first and to allow a delay
//!   after a command. Neither libsigrok nor the port does, and this model doesn't ask.
//! - Which way the relays lie at power on, and what a digit does with no letter
//!   since then, nothing here says: the first state is B, as an artifact, and such
//!   a digit is flagged. So is any byte but those above. Bus commands (`?`, `%`) go
//!   with ATN, which a socket has none of, so those are flagged as well.
//! - Not modelled: the local mode, where the front panel's buttons take over again
//!   (031443), and the time a relay takes.
//!
//! A close changes nothing: a relay stays where it is.
//!
//! Options, none a device's:
//!
//! - `mark-at=N`, `mark-conn=N`: the point `--unplug tcp-model` waits for, at the
//!   end of the Nth command or at the Nth connection, counted from power on.
//! - `banner=TEXT`: said at each connection, as a bridge might. The real unit has
//!   nothing to say it with.

use crate::model::DeviceModel;
use crate::shells::tcp::{NoUsb, Port, Stream};
use crate::{Error, Result};

/// The model's name.
pub const NAME: &str = "hp-59306a";
/// One line for `oxrok-emu list`.
pub const ABOUT: &str =
    "HP 59306A relay actuator, on TCP alone (mark-at=N, mark-conn=N, banner=TEXT)";

/// Which contact a relay joins C to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Contact {
    A,
    B,
}

/// The actuator.
#[derive(Debug)]
pub struct Actuator {
    relays: [Contact; 6],
    /// The letter a digit acts by.
    sense: Option<Contact>,
    mark_at: u32,
    mark_conn: u32,
    banner: Vec<u8>,
    commands: u32,
    connections: u32,
}

impl Actuator {
    fn new() -> Actuator {
        Actuator {
            relays: [Contact::B; 6],
            sense: None,
            mark_at: 0,
            mark_conn: 0,
            banner: Vec::new(),
            commands: 0,
            connections: 0,
        }
    }

    fn with(options: &[(String, String)]) -> Result<Actuator> {
        let mut a = Actuator::new();
        for (k, v) in options {
            let count = || {
                v.parse::<u32>()
                    .map_err(|_| Error::Invalid(format!("{NAME}: {k} takes a count, not {v}")))
            };
            match k.as_str() {
                "mark-at" => a.mark_at = count()?,
                "mark-conn" => a.mark_conn = count()?,
                "banner" => a.banner = v.as_bytes().to_vec(),
                _ => return Err(Error::Invalid(format!("{NAME} has no option {k}"))),
            }
        }
        Ok(a)
    }

    /// One byte as the logic takes it; the bytes come as the host cut them.
    fn byte(&mut self, b: u8, port: &mut Port) {
        match b {
            b'A' => self.sense = Some(Contact::A),
            b'B' => self.sense = Some(Contact::B),
            b'1'..=b'6' => match self.sense {
                Some(sense) => self.relays[usize::from(b - b'1')] = sense,
                None => port.violation(format!("relay {} with no A or B", char::from(b))),
            },
            b'\r' => {}
            b'\n' => {
                self.commands += 1;
                if self.commands == self.mark_at {
                    port.mark();
                }
            }
            _ => port.violation(format!("byte {b:#04x} is no program code")),
        }
    }
}

impl Stream for Actuator {
    fn received(&mut self, data: &[u8], port: &mut Port) {
        for &b in data {
            self.byte(b, port);
        }
    }

    fn connected(&mut self, port: &mut Port) {
        self.connections += 1;
        if !self.banner.is_empty() {
            port.send(&self.banner);
        }
        if self.connections == self.mark_conn {
            port.mark();
        }
    }
}

/// The USB side, which this device has none of.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Actuator::with(options)?;
    Ok(Box::new(NoUsb::new(NAME)))
}

/// The device on a socket.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    Ok(Box::new(Actuator::with(options)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(a: &mut Actuator, text: &[u8]) -> Port {
        let mut port = Port::default();
        a.received(text, &mut port);
        port
    }

    fn state(a: &Actuator) -> String {
        a.relays.iter().map(|c| if *c == Contact::A { 'A' } else { 'B' }).collect()
    }

    fn opts(pairs: &[(&str, &str)]) -> Vec<(String, String)> {
        pairs.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    #[test]
    fn the_commands_c_sends() {
        let mut a = Actuator::new();
        assert!(feed(&mut a, b"A123456\n").violations().is_empty());
        assert_eq!(state(&a), "AAAAAA");
        assert!(feed(&mut a, b"B3\n").violations().is_empty());
        assert_eq!(state(&a), "AABAAA");
        assert!(feed(&mut a, b"B123456\n").violations().is_empty());
        assert_eq!(state(&a), "BBBBBB");
        assert!(feed(&mut a, b"A6\n").violations().is_empty());
        assert_eq!(state(&a), "BBBBBA");
    }

    #[test]
    fn however_the_bytes_are_cut_and_with_a_letter_in_the_middle() {
        let mut a = Actuator::new();
        for b in b"A35B6A1\r\n" {
            assert!(feed(&mut a, &[*b]).violations().is_empty());
        }
        assert_eq!(state(&a), "ABABAB");
    }

    #[test]
    fn what_it_flags_and_leaves_alone() {
        for text in [&b"3"[..], b"A7", b"A0", b"C1", b"%", b"?", b"A1 ", b"a1"] {
            let mut a = Actuator::new();
            assert!(!feed(&mut a, text).violations().is_empty(), "{text:?}");
        }
        let mut a = Actuator::new();
        feed(&mut a, b"A7");
        assert_eq!(state(&a), "BBBBBB");
    }

    /// Nothing but a command moves a relay, and it never answers.
    #[test]
    fn a_new_connection_keeps_the_relays_and_nothing_is_said() {
        let mut a = Actuator::new();
        assert_eq!(feed(&mut a, b"A1\n").sent(), Vec::<Vec<u8>>::new());
        a.closed();
        let mut port = Port::default();
        a.connected(&mut port);
        assert_eq!(port.sent(), Vec::<Vec<u8>>::new());
        assert_eq!(state(&a), "ABBBBB");
    }

    #[test]
    fn marks_at_the_count_only() {
        let mut a = Actuator::with(&opts(&[("mark-at", "2")])).unwrap();
        assert!(!feed(&mut a, b"A1\n").marked());
        assert!(feed(&mut a, b"A2\n").marked());
        assert!(!feed(&mut a, b"A3\n").marked());
        let mut a = Actuator::with(&opts(&[("mark-conn", "2")])).unwrap();
        for want in [false, true, false] {
            let mut port = Port::default();
            a.connected(&mut port);
            assert_eq!(port.marked(), want);
        }
        // Off by default, a count of 0 never being reached.
        let mut a = Actuator::new();
        let mut port = Port::default();
        a.connected(&mut port);
        assert!(!port.marked());
        assert!(!feed(&mut a, b"A1\n").marked());
    }

    #[test]
    fn a_banner_is_said_at_each_connection() {
        let mut a = Actuator::with(&opts(&[("banner", "hello")])).unwrap();
        for _ in 0..2 {
            let mut port = Port::default();
            a.connected(&mut port);
            assert_eq!(port.sent(), [b"hello".to_vec()]);
        }
    }

    #[test]
    fn options_are_checked() {
        assert!(Actuator::with(&opts(&[("nope", "1")])).is_err());
        assert!(Actuator::with(&opts(&[("mark-at", "x")])).is_err());
        assert!(build(&opts(&[("nope", "1")])).is_err());
        assert!(build_tcp(&opts(&[("nope", "1")])).is_err());
        assert_eq!(build(&[]).unwrap().name(), NAME);
        assert!(build_tcp(&[]).is_ok());
    }
}
