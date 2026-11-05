// Counterpart in sigrok:
//   libsigrok/src/hardware/icstation-usbrelay/protocol.c,
//   Copyright (C) 2021-2023 Frank Stettner <frank-stettner@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake port, which is where the write and read errors can be
//! had: an emulator case can't fail a write without a race against the unplug.

use super::*;
use crate::core::Result;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

/// What every port of a test shares: a board that keeps its state across opens.
#[allow(clippy::struct_excessive_bools)]
#[derive(Default)]
struct Line {
    /// Bytes written, in order, on any port.
    written: Vec<u8>,
    /// Answers for reads, in order.
    answers: VecDeque<u8>,
    /// Empty reads before an answer comes.
    silent_reads: usize,
    /// How many opens succeeded.
    opens: usize,
    /// The next opens fail.
    fail_open: bool,
    /// Writes take nothing: a timeout.
    write_takes_none: bool,
    /// Writes fail: a port gone.
    write_fails: bool,
    /// Reads fail: a port gone.
    read_fails: bool,
    /// The timeouts asked, reads and writes alike.
    timeouts: Vec<Duration>,
}

#[derive(Clone, Default)]
struct Shared(Arc<Mutex<Line>>);

impl Shared {
    fn line(&self) -> MutexGuard<'_, Line> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn written(&self) -> Vec<u8> {
        self.line().written.clone()
    }

    fn opener(&self) -> Opener {
        let shared = self.clone();
        Box::new(move || {
            let mut line = shared.line();
            if line.fail_open {
                return Err(Error::device("no such port"));
            }
            line.opens += 1;
            drop(line);
            Ok(Box::new(FakePort(shared.clone())) as Box<dyn Port>)
        })
    }
}

struct FakePort(Shared);

impl Port for FakePort {
    fn name(&self) -> &'static str {
        "/dev/fake"
    }

    fn set_params(&mut self, _params: Params) -> Result<()> {
        Ok(())
    }

    fn set_handshake(&mut self, _rts: Option<bool>, _dtr: Option<bool>) -> Result<()> {
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut line = self.0.line();
        line.timeouts.push(timeout);
        if line.read_fails {
            return Err(Error::device("read failed"));
        }
        if line.silent_reads > 0 {
            line.silent_reads -= 1;
            return Ok(0);
        }
        match line.answers.pop_front() {
            Some(b) if !buf.is_empty() => {
                buf[0] = b;
                Ok(1)
            }
            _ => Ok(0),
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.0.line().written.extend_from_slice(data);
        Ok(())
    }

    fn write_timeout(&mut self, data: &[u8], timeout: Duration) -> Result<usize> {
        let mut line = self.0.line();
        line.timeouts.push(timeout);
        if line.write_fails {
            return Err(Error::device("write failed"));
        }
        if line.write_takes_none {
            return Ok(0);
        }
        line.written.extend_from_slice(data);
        Ok(data.len())
    }

    fn purge(&mut self) -> Result<()> {
        Ok(())
    }
}

fn board(shared: &Shared, name: &str) -> IcstationDevice {
    let id = [0xab, 0xad, 0xac]
        .into_iter()
        .filter_map(protocol::profile)
        .find(|p| p.name == name)
        .expect("a board");
    IcstationDevice::new(id, "/dev/fake".to_owned(), shared.opener())
}

fn probed(shared: &Shared) -> Option<IcstationDevice> {
    let mut opener = shared.opener();
    let port = opener().expect("a port");
    probe(port, "/dev/fake".to_owned(), opener)
}

fn enabled(d: &IcstationDevice, g: &str) -> bool {
    d.get(&Key::Enabled, Some(g)).unwrap() == Value::Bool(true)
}

#[test]
fn a_scan_asks_once_and_names_the_board() {
    let shared = Shared::default();
    shared.line().answers.push_back(0xac);
    let d = probed(&shared).expect("a board");
    assert_eq!(d.info().vendor, "ICStation");
    assert_eq!(d.info().model, "ICSE014A");
    assert_eq!(d.info().conn, None);
    assert_eq!(d.channel_groups().len(), 8);
    assert_eq!(d.channel_groups()[7].name, "R8");
    assert_eq!(shared.written(), [0x50], "and no start before an open");
}

/// Only a request that went out and got nothing can have switched relays, which
/// is what the scan's warning says.
#[test]
fn identify_tells_silence_from_a_failure() {
    let shared = Shared::default();
    shared.line().answers.push_back(0xab);
    let mut opener = shared.opener();
    assert_eq!(protocol::identify(opener().unwrap().as_mut()).unwrap(), Some(0xab));
    shared.line().silent_reads = usize::MAX;
    assert_eq!(protocol::identify(opener().unwrap().as_mut()).unwrap(), None);
    shared.line().read_fails = true;
    assert!(protocol::identify(opener().unwrap().as_mut()).is_err());
    let shared = Shared::default();
    shared.line().write_takes_none = true;
    let mut opener = shared.opener();
    assert!(protocol::identify(opener().unwrap().as_mut()).is_err(), "nothing went out");
}

#[test]
fn an_unknown_id_is_no_board() {
    let shared = Shared::default();
    shared.line().answers.push_back(0x12);
    assert!(probed(&shared).is_none());
}

#[test]
fn a_silent_board_is_none_and_the_waits_are_a_second() {
    let shared = Shared::default();
    let began = std::time::Instant::now();
    assert!(probed(&shared).is_none());
    assert!(began.elapsed() >= Duration::from_millis(1000), "a load only stretches it");
    let line = shared.line();
    assert_eq!(line.timeouts[0], Duration::from_millis(1000), "the write");
    assert!(line.timeouts[1] <= Duration::from_millis(1000), "the read");
}

#[test]
fn a_scan_fails_on_a_read_or_write_error() {
    let shared = Shared::default();
    shared.line().read_fails = true;
    assert!(probed(&shared).is_none());
    let shared = Shared::default();
    shared.line().write_fails = true;
    shared.line().answers.push_back(0xab);
    assert!(probed(&shared).is_none());
    let shared = Shared::default();
    shared.line().write_takes_none = true;
    shared.line().answers.push_back(0xab);
    assert!(probed(&shared).is_none());
    assert_eq!(shared.line().answers.len(), 1, "no read without a request");
}

/// An answer that comes after a few empty reads, within the read's bound, still
/// counts: `Port::read`'s 0 is not the end of a read.
#[test]
fn a_late_answer_within_the_second_counts() {
    let shared = Shared::default();
    shared.line().silent_reads = 3;
    shared.line().answers.push_back(0xab);
    assert_eq!(probed(&shared).expect("a board").info().model, "ICSE012A");
}

/// A second byte behind the id is for no one. The scan reads one, and nothing
/// reads after it: no later request has an answer it could shift.
#[test]
fn a_byte_behind_the_id_is_left_alone() {
    let shared = Shared::default();
    shared.line().answers.extend([0xab, 0xcd]);
    let mut d = probed(&shared).expect("a board");
    assert_eq!(d.info().model, "ICSE012A");
    d.open().unwrap();
    d.set(&Key::Enabled, &Value::Bool(true), Some("R1")).unwrap();
    assert!(enabled(&d, "R1"));
    assert_eq!(shared.line().answers, [0xcd], "nothing read it");
}

#[test]
fn an_open_starts_and_a_setting_is_one_active_low_byte() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    d.open().unwrap();
    assert_eq!(shared.written(), [0x51]);
    d.set(&Key::Enabled, &Value::Bool(true), Some("R1")).unwrap();
    d.set(&Key::Enabled, &Value::Bool(true), Some("R3")).unwrap();
    d.set(&Key::Enabled, &Value::Bool(false), Some("R1")).unwrap();
    d.set(&Key::Enabled, &Value::Bool(true), None).unwrap();
    d.set(&Key::Enabled, &Value::Bool(false), None).unwrap();
    assert_eq!(shared.written(), [0x51, 0x0e, 0x0a, 0x0b, 0x00, 0x0f]);
    assert_eq!(shared.line().timeouts[1], Duration::from_millis(1000));
}

#[test]
fn each_board_has_its_mask() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE013A");
    d.open().unwrap();
    d.set(&Key::Enabled, &Value::Bool(false), None).unwrap();
    d.set(&Key::Enabled, &Value::Bool(true), Some("R2")).unwrap();
    assert_eq!(shared.written(), [0x51, 0x03, 0x01]);
    assert!(d.get(&Key::Enabled, Some("R3")).is_err());
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE014A");
    d.open().unwrap();
    d.set(&Key::Enabled, &Value::Bool(true), Some("R8")).unwrap();
    d.set(&Key::Enabled, &Value::Bool(true), None).unwrap();
    assert_eq!(shared.written(), [0x51, 0x7f, 0x00]);
    assert!(enabled(&d, "R8"));
}

#[test]
fn a_get_reads_the_cache_and_conn_and_sends_nothing() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    d.open().unwrap();
    assert!(!enabled(&d, "R1"));
    assert_eq!(d.get(&Key::Conn, None).unwrap(), Value::Str("/dev/fake".to_owned()));
    assert!(d.get(&Key::Enabled, None).is_err());
    assert!(d.get(&Key::Conn, Some("R1")).is_err());
    assert_eq!(shared.written(), [0x51]);
}

#[test]
fn a_failed_setting_leaves_the_cache() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    d.open().unwrap();
    d.set(&Key::Enabled, &Value::Bool(true), Some("R1")).unwrap();
    shared.line().write_takes_none = true;
    let e = d.set(&Key::Enabled, &Value::Bool(true), Some("R2")).unwrap_err();
    assert_eq!(e.to_string(), "input/output error");
    assert!(!enabled(&d, "R2"));
    shared.line().write_takes_none = false;
    shared.line().write_fails = true;
    let e = d.set(&Key::Enabled, &Value::Bool(true), Some("R2")).unwrap_err();
    assert_eq!(e.to_string(), "input/output error");
    assert!(!enabled(&d, "R2"));
    shared.line().write_fails = false;
    d.set(&Key::Enabled, &Value::Bool(true), Some("R2")).unwrap();
    assert_eq!(shared.written(), [0x51, 0x0e, 0x0c], "R1 still in the byte");
}

#[test]
fn a_setting_on_a_closed_port_fails() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    assert!(d.set(&Key::Enabled, &Value::Bool(true), None).is_err());
    assert!(!enabled(&d, "R1"));
    assert_eq!(shared.written(), [0u8; 0]);
}

#[test]
fn a_failed_start_fails_the_open_and_counts_for_nothing() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    shared.line().write_fails = true;
    let e = d.open().unwrap_err();
    assert_eq!(e.to_string(), "input/output error");
    assert!(d.set(&Key::Enabled, &Value::Bool(true), None).is_err(), "the port went");
    shared.line().write_fails = false;
    d.open().unwrap();
    assert_eq!(shared.written(), [0x51], "the next open still starts it");
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    shared.line().write_takes_none = true;
    assert_eq!(d.open().unwrap_err().to_string(), "input/output error");
}

#[test]
fn a_port_that_does_not_open_fails_the_open() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    shared.line().fail_open = true;
    assert!(d.open().is_err());
    assert_eq!(shared.written(), [0u8; 0]);
}

/// The board takes a `51` in command mode as a relay byte, here R2 to R4 on: a
/// reopen must not send it, and the cache must still hold.
#[test]
fn a_second_open_does_not_start_again() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    d.open().unwrap();
    d.set(&Key::Enabled, &Value::Bool(true), Some("R1")).unwrap();
    d.close().unwrap();
    d.open().unwrap();
    assert_eq!(shared.line().opens, 2, "it did reopen");
    assert!(enabled(&d, "R1"));
    d.set(&Key::Enabled, &Value::Bool(true), Some("R2")).unwrap();
    assert_eq!(shared.written(), [0x51, 0x0e, 0x0c]);
}

#[test]
fn an_open_port_is_kept_by_a_second_open() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    d.open().unwrap();
    d.open().unwrap();
    assert_eq!(shared.line().opens, 1);
    assert_eq!(shared.written(), [0x51]);
}

#[test]
fn a_setting_refuses_what_is_no_boolean_or_no_relay() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    d.open().unwrap();
    assert!(d.set(&Key::Enabled, &Value::Str("on".to_owned()), None).is_err());
    assert!(d.set(&Key::Enabled, &Value::Bool(true), Some("R5")).is_err());
    assert!(d.set(&Key::Conn, &Value::Str("x".to_owned()), None).is_err());
    assert_eq!(shared.written(), [0x51]);
}

#[test]
fn the_options_are_as_in_c() {
    let shared = Shared::default();
    let d = board(&shared, "ICSE012A");
    let keys: Vec<Key> = d.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(keys, [Key::Conn, Key::Enabled]);
    let keys: Vec<Key> = d.options(Some("R1")).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(keys, [Key::Enabled]);
    assert!(d.options(Some("R5")).is_err());
}

#[test]
fn a_capture_is_refused_with_no_channels() {
    let shared = Shared::default();
    let mut d = board(&shared, "ICSE012A");
    d.open().unwrap();
    let (tx, _rx) = std::sync::mpsc::sync_channel(1);
    let sink = PacketSink::new(tx, 0);
    let e = d.start(sink).unwrap_err();
    assert!(e.to_string().contains("has no enabled channels"), "{e}");
    assert_eq!(shared.written(), [0x51], "and the board sees nothing");
    d.stop().unwrap();
}
