// Counterpart in sigrok:
//   libsigrok/src/hardware/hp-59306a/api.c,
//   libsigrok/src/hardware/hp-59306a/protocol.c,
//   libsigrok/src/hardware/hp-59306a/protocol.h,
//   Copyright (C) 2021 Frank Stettner <frank-stettner@gmx.net>,
//   with commits by Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The HP 59306A relay actuator: six relays, each joining its common contact to A
//! or to B, as channel groups `R1` to `R6`. `enabled=on` sends `A` and the relay's
//! number, `enabled=off` sends `B`; with no group it is all six, `A123456` or
//! `B123456`. Each command ends in a line feed, as `scpi.c:161-162` adds one.
//!
//! The real one is on HP-IB, and oxrok has no GPIB, so this reaches it by what
//! `conn=` names of the rest of [`scpi`]: `tcp-raw/HOST/PORT` through a bridge. The
//! device only listens. It has no way to say what it is, so with a `conn=` anything
//! that opens is a 59306A, and with none there is no device and nothing is probed
//! (`api.c:52-58`, `:91-95`). It has no way to say where its relays lie either, so
//! nothing can read them and `enabled` has no get.
//!
//! So a setting that went out is all that is ever known. Of the owners on time-nuts
//! in May 2008, one has it ignore a line end and others say not to send one, and one
//! says to send a remote command first and allow a delay after a command. Nothing
//! here can tell: this keeps what C sends, with no remote command.
//!
//! Where this leaves `api.c` and `protocol.c`:
//!
//! - After each command, the line gets a look of [`LOOK`], with nothing on the wire.
//!   A connection that has closed, or has reset, fails the setting and says the
//!   state isn't known, where C's `send` into it succeeds and it exits 0. A bridge
//!   that takes one client accepts a second and closes it, so this is no corner. It
//!   is a look and no proof, wrong both ways. A close after the command was taken
//!   fails a setting that went through. A close later than the look, or a bridge
//!   that keeps the connection and drops the command, passes with exit 0. Bytes on
//!   the line are dropped: this device has none to send.
//! - A write that fails is an error. `tools/emu/rigol-ds.sh` has sigrok-cli dying
//!   of SIGPIPE on such a write. That was not reproduced on this driver, where two
//!   commands in a row went out ahead of the close.
//!
//! The device acquires nothing and has no channels: it has no limit keys, so a
//! command line asking for a capture is refused, and a session refuses to start
//! one. Only a client that calls [`Device::start`] itself gets a header and an end
//! at once, where C's dummy start sends neither.

#![forbid(unsafe_code)]

use std::time::{Duration, Instant};

use crate::core::{
    Access, Channel, ChannelGroup, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Value, ValueList,
};
use crate::scpi::{self, Resource, Scpi};

/// The relays, numbered from 1 as on the device (`api.c:74-82`).
const RELAYS: usize = 6;
/// How long a setting watches the line for a close. A close on a LAN takes
/// milliseconds; this is what a client pays for each setting.
const LOOK: Duration = Duration::from_millis(100);
/// The most looks one setting takes, for a line that keeps talking or being
/// interrupted: each that isn't a full quiet [`LOOK`] goes again.
const LOOK_TURNS: usize = 20;

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multiplexer];
const SET_ONLY: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 2] =
    [OptionDesc::new(Key::Conn, Access::GET), OptionDesc::new(Key::Enabled, SET_ONLY)];
static GROUP_OPTIONS: [OptionDesc; 1] = [OptionDesc::new(Key::Enabled, SET_ONLY)];

/// `hp_59306a_switch_cg`'s command: one relay, or with `None` all of them.
fn command(relay: Option<usize>, on: bool) -> String {
    let sense = if on { 'A' } else { 'B' };
    match relay {
        Some(n) => format!("{sense}{n}"),
        None => format!("{sense}123456"),
    }
}

/// What a closed line makes of a command. It may have got out before the close, or
/// a part of it, so nobody knows where the relays lie.
fn lost(command: &str, why: &Error) -> Error {
    let why = match why {
        Error::Device(text) => text.clone(),
        other => other.to_string(),
    };
    Error::device(format!(
        "Command {command}: {why}. It may not have reached the device, and where its \
         relays lie is not known"
    ))
}

/// Watches the line for a close, for a quiet [`LOOK`] and at most [`LOOK_TURNS`]
/// reads. A read that came back early found a bridge's bytes or, on a serial port,
/// a signal, and neither is a quiet line. A transport with no read that is only a
/// read has nothing to look with.
fn look(scpi: &mut Scpi) -> Result<()> {
    for _ in 0..LOOK_TURNS {
        let began = Instant::now();
        match scpi.discard(LOOK)? {
            None => return Ok(()),
            Some(0) if began.elapsed() >= LOOK => return Ok(()),
            Some(0) => {}
            Some(n) => log::debug!("{n} bytes nobody asked for: this device has none to send."),
        }
    }
    Ok(())
}

/// The `hp-59306a` driver.
pub struct Hp59306aDriver;

impl Driver for Hp59306aDriver {
    fn id(&self) -> &'static str {
        "hp-59306a"
    }

    fn long_name(&self) -> &'static str {
        "HP 59306A"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        // Only a device somebody named: nothing here can identify one.
        let Some(conn) = conn else { return Ok(Vec::new()) };
        // What fails here is no device, as `sr_scpi_scan_resource` has it.
        let resource = match Resource::parse(&conn) {
            Ok(resource) => resource,
            Err(e) => {
                log::info!("{e}");
                return Ok(Vec::new());
            }
        };
        let found = match resource {
            Resource::Usbtmc(None) => match scpi::scan(Some(&conn), None, |s| Ok(Some(probe(s)))) {
                Ok(found) => found,
                Err(e) => {
                    log::info!("{e}");
                    Vec::new()
                }
            },
            resource => match scpi::open_keeping_line(&resource, None) {
                Ok(s) => vec![probe(s)],
                Err(e) => {
                    log::warn!("Couldn't open SCPI device: {e}.");
                    Vec::new()
                }
            },
        };
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// `probe_device` and the close after it: nothing goes out, as anything sent to
/// what may be another device could do harm.
fn probe(mut scpi: Scpi) -> Actuator {
    let _ = scpi.close();
    Actuator::new(scpi)
}

/// One actuator.
pub struct Actuator {
    info: DeviceInfo,
    resource: String,
    groups: Vec<ChannelGroup>,
    scpi: Scpi,
    open: bool,
}

impl Actuator {
    /// An actuator on `scpi`, which [`Device::open`] opens again. For tests and for
    /// clients with a transport of their own.
    pub fn new(scpi: Scpi) -> Actuator {
        let resource = scpi.resource().to_owned();
        Actuator {
            info: DeviceInfo {
                driver: "hp-59306a",
                vendor: "Hewlett-Packard".to_owned(),
                model: "59306A".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: Some(resource.clone()),
            },
            resource,
            groups: (1..=RELAYS).map(|n| ChannelGroup::new(format!("R{n}"), Vec::new())).collect(),
            scpi,
            open: false,
        }
    }

    /// The relay a group names, or `None` for no group.
    fn relay(&self, group: Option<&str>) -> Result<Option<usize>> {
        let Some(g) = group else { return Ok(None) };
        let at = self
            .groups
            .iter()
            .position(|cg| cg.name == g)
            .ok_or_else(|| Error::ChannelGroup(g.to_owned()))?;
        Ok(Some(at + 1))
    }

    fn switch(&mut self, relay: Option<usize>, on: bool) -> Result<()> {
        if !self.open {
            return Err(Error::device("The device is not open."));
        }
        let command = command(relay, on);
        self.scpi.send(&command).map_err(|e| lost(&command, &e))?;
        look(&mut self.scpi).map_err(|e| lost(&command, &e))
    }
}

impl Device for Actuator {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &[]
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &self.groups
    }

    fn enable_channel(&mut self, pos: usize, _enabled: bool) -> Result<()> {
        Err(Error::Arg(format!("No channel {pos}.")))
    }

    fn rename_channel(&mut self, pos: usize, _name: &str) -> Result<()> {
        Err(Error::Arg(format!("No channel {pos}.")))
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(match self.relay(group)? {
            None => DEV_OPTIONS.to_vec(),
            Some(_) => GROUP_OPTIONS.to_vec(),
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        match (key, self.relay(group)?) {
            (Key::Conn, None) => Ok(Value::Str(self.resource.clone())),
            (other, _) => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let relay = self.relay(group)?;
        match key {
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(|| Error::arg("enabled takes on or off"))?;
                self.switch(relay, on)
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    /// `sr_scpi_open`: a connection and nothing on it.
    fn open(&mut self) -> Result<()> {
        self.scpi.reopen()?;
        self.open = true;
        Ok(())
    }

    /// `sr_scpi_close`. The relays stay where they are.
    fn close(&mut self) -> Result<()> {
        self.open = false;
        self.scpi.close()
    }

    /// `std_dummy_dev_acquisition_start`: nothing to acquire, so what a client
    /// waits for comes at once. On a thread of its own, as a queue of one would
    /// hold up the caller at the second packet. It owns nothing and needs no join.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        std::thread::spawn(move || {
            sink.send(Packet::header());
            sink.end(Ok(()));
        });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scpi::ScpiTransport;
    use crate::serial::WriteLog;
    use std::collections::VecDeque;
    use std::io::Read;
    use std::net::{TcpListener, TcpStream};
    use std::sync::{Arc, Mutex};

    /// What the fake's line does at each look, then what it does for good.
    #[derive(Default)]
    struct Line {
        writes: WriteLog,
        write_fails: bool,
        looks: VecDeque<Result<Option<usize>>>,
        looks_taken: usize,
        waits: Vec<Duration>,
        /// A look that isn't scripted returns at once if set, else sleeps its time.
        quick: bool,
        opens: usize,
        closes: usize,
    }

    struct Fake(Arc<Mutex<Line>>);

    impl ScpiTransport for Fake {
        fn resource(&self) -> &'static str {
            "fake/1"
        }
        fn send(&mut self, data: &[u8]) -> Result<()> {
            let mut line = self.0.lock().unwrap();
            if line.write_fails {
                return Err(Error::device("broken pipe"));
            }
            line.writes.push((Instant::now(), data.to_vec()));
            Ok(())
        }
        fn read_begin(&mut self) -> Result<()> {
            panic!("this device has nothing to read");
        }
        fn read_data(&mut self, _: &mut [u8], _: Duration) -> Result<usize> {
            panic!("this device has nothing to read");
        }
        fn read_complete(&self) -> bool {
            true
        }
        fn close(&mut self) -> Result<()> {
            self.0.lock().unwrap().closes += 1;
            Ok(())
        }
        fn reopen(&mut self) -> Result<()> {
            self.0.lock().unwrap().opens += 1;
            Ok(())
        }
        fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
            let (next, quick) = {
                let mut line = self.0.lock().unwrap();
                line.looks_taken += 1;
                line.waits.push(timeout);
                (line.looks.pop_front(), line.quick)
            };
            if let Some(answer) = next {
                return answer;
            }
            if !quick {
                // Past its time by a margin: a sleep and a clock needn't agree.
                std::thread::sleep(timeout + Duration::from_millis(20));
            }
            Ok(Some(0))
        }
    }

    fn fake() -> (Actuator, Arc<Mutex<Line>>) {
        let line = Arc::new(Mutex::new(Line::default()));
        let device = probe(Scpi::new(Box::new(Fake(line.clone()))));
        (device, line)
    }

    fn opened() -> (Actuator, Arc<Mutex<Line>>) {
        let (mut device, line) = fake();
        device.open().unwrap();
        (device, line)
    }

    fn written(line: &Arc<Mutex<Line>>) -> Vec<String> {
        line.lock()
            .unwrap()
            .writes
            .iter()
            .map(|(_, w)| String::from_utf8_lossy(w).into_owned())
            .collect()
    }

    fn on(on: bool) -> Value {
        Value::Bool(on)
    }

    #[test]
    fn the_commands_as_literals() {
        assert_eq!(command(None, true), "A123456");
        assert_eq!(command(None, false), "B123456");
        assert_eq!(command(Some(1), true), "A1");
        assert_eq!(command(Some(6), false), "B6");
    }

    #[test]
    fn a_probe_sends_nothing_and_closes() {
        let (device, line) = fake();
        assert_eq!(written(&line), Vec::<String>::new());
        assert_eq!(line.lock().unwrap().closes, 1);
        let info = device.info();
        assert_eq!(info.vendor, "Hewlett-Packard");
        assert_eq!(info.model, "59306A");
        assert_eq!(info.conn.as_deref(), Some("fake/1"));
    }

    #[test]
    fn every_group_switches_its_own_relay() {
        let (mut device, line) = opened();
        line.lock().unwrap().quick = true;
        let names: Vec<String> = device.channel_groups().iter().map(|cg| cg.name.clone()).collect();
        assert_eq!(names, ["R1", "R2", "R3", "R4", "R5", "R6"]);
        for name in &names {
            device.set(&Key::Enabled, &on(true), Some(name)).unwrap();
            device.set(&Key::Enabled, &on(false), Some(name)).unwrap();
        }
        device.set(&Key::Enabled, &on(true), None).unwrap();
        device.set(&Key::Enabled, &on(false), None).unwrap();
        let want = [
            "A1\n",
            "B1\n",
            "A2\n",
            "B2\n",
            "A3\n",
            "B3\n",
            "A4\n",
            "B4\n",
            "A5\n",
            "B5\n",
            "A6\n",
            "B6\n",
            "A123456\n",
            "B123456\n",
        ];
        assert_eq!(written(&line), want);
    }

    #[test]
    fn what_is_refused_sends_nothing() {
        let (mut device, line) = opened();
        assert!(matches!(
            device.set(&Key::Enabled, &on(true), Some("R7")),
            Err(Error::ChannelGroup(_))
        ));
        assert!(device.set(&Key::Enabled, &Value::Str("x".to_owned()), None).is_err());
        assert!(device.set(&Key::Conn, &Value::Str("x".to_owned()), None).is_err());
        assert_eq!(written(&line), Vec::<String>::new());
    }

    #[test]
    fn a_set_on_a_closed_device_sends_nothing() {
        let (mut device, line) = fake();
        assert!(device.set(&Key::Enabled, &on(true), None).is_err());
        device.open().unwrap();
        line.lock().unwrap().quick = true;
        device.set(&Key::Enabled, &on(true), None).unwrap();
        device.close().unwrap();
        assert!(device.set(&Key::Enabled, &on(true), None).is_err());
        assert_eq!(written(&line), ["A123456\n"]);
    }

    #[test]
    fn open_and_close_go_to_the_transport() {
        let (mut device, line) = fake();
        device.open().unwrap();
        assert_eq!(line.lock().unwrap().opens, 1);
        device.close().unwrap();
        assert_eq!(line.lock().unwrap().closes, 2);
    }

    #[test]
    fn the_options_and_what_cannot_be_read() {
        let (device, _line) = opened();
        let of = |group| -> Vec<(Key, bool, bool, bool)> {
            device
                .options(group)
                .unwrap()
                .iter()
                .map(|o| (o.key.clone(), o.access.get, o.access.set, o.access.list))
                .collect()
        };
        assert_eq!(of(None), [(Key::Conn, true, false, false), (Key::Enabled, false, true, false)]);
        assert_eq!(of(Some("R3")), [(Key::Enabled, false, true, false)]);
        assert!(device.options(Some("R0")).is_err());
        assert_eq!(device.get(&Key::Conn, None).unwrap(), Value::Str("fake/1".to_owned()));
        assert!(device.get(&Key::Conn, Some("R1")).is_err());
        assert!(device.get(&Key::Enabled, None).is_err());
        assert!(device.get(&Key::Enabled, Some("R1")).is_err());
        assert!(device.list(&Key::Enabled, None).is_err());
        assert_eq!(device.channels().len(), 0);
        assert_eq!(Hp59306aDriver.scan_options(), [Key::Conn]);
        assert_eq!(Hp59306aDriver.classes(), [DeviceClass::Multiplexer]);
    }

    /// The whole of the words, as a filter can't see them all.
    #[test]
    fn a_failed_write_says_what_is_not_known() {
        let (mut device, line) = opened();
        line.lock().unwrap().write_fails = true;
        let e = device.set(&Key::Enabled, &on(true), Some("R2")).unwrap_err();
        assert_eq!(
            e.to_string(),
            "device error: Command A2: broken pipe. It may not have reached the device, and where its \
             relays lie is not known"
        );
        assert_eq!(line.lock().unwrap().looks_taken, 0);
    }

    #[test]
    fn a_close_found_by_the_look_fails_the_setting() {
        let (mut device, line) = opened();
        line.lock().unwrap().looks.push_back(Err(Error::device("it closed")));
        let e = device.set(&Key::Enabled, &on(false), None).unwrap_err();
        assert_eq!(
            e.to_string(),
            "device error: Command B123456: it closed. It may not have reached the device, and \
             where its relays lie is not known"
        );
        assert_eq!(written(&line), ["B123456\n"]);
    }

    #[test]
    fn a_reset_found_by_the_look_keeps_the_systems_words() {
        let (mut device, line) = opened();
        let reset = std::io::Error::other("reset by peer");
        line.lock().unwrap().looks.push_back(Err(Error::Io(reset)));
        let e = device.set(&Key::Enabled, &on(true), None).unwrap_err().to_string();
        assert!(e.starts_with("device error: Command A123456: "), "{e}");
        assert!(e.contains("reset by peer. It may not have reached the device"), "{e}");
    }

    #[test]
    fn the_look_waits_a_tenth_of_a_second() {
        assert_eq!(LOOK, Duration::from_millis(100));
        let (mut device, line) = opened();
        let began = Instant::now();
        device.set(&Key::Enabled, &on(true), None).unwrap();
        assert!(began.elapsed() >= Duration::from_millis(100));
        assert_eq!(line.lock().unwrap().waits[0], Duration::from_millis(100));
    }

    /// A signal or a bridge's chatter is no quiet line: a close right after them is
    /// still found.
    #[test]
    fn a_look_cut_short_goes_again() {
        let (mut device, line) = opened();
        {
            let mut line = line.lock().unwrap();
            line.looks.push_back(Ok(Some(0)));
            line.looks.push_back(Ok(Some(5)));
            line.looks.push_back(Err(Error::device("it closed")));
        }
        assert!(device.set(&Key::Enabled, &on(true), None).is_err());
        assert_eq!(line.lock().unwrap().looks_taken, 3);
    }

    #[test]
    fn a_line_that_keeps_interrupting_is_given_up_on_with_no_error() {
        let (mut device, line) = opened();
        line.lock().unwrap().quick = true;
        device.set(&Key::Enabled, &on(true), None).unwrap();
        assert_eq!(line.lock().unwrap().looks_taken, 20);
    }

    #[test]
    fn a_line_that_keeps_talking_is_given_up_on_with_no_error() {
        let (mut device, line) = opened();
        for _ in 0..40 {
            line.lock().unwrap().looks.push_back(Ok(Some(1)));
        }
        device.set(&Key::Enabled, &on(true), None).unwrap();
        assert_eq!(line.lock().unwrap().looks_taken, 20);
    }

    /// USBTMC has no read that is only a read.
    #[test]
    fn a_transport_with_no_look_passes() {
        let (mut device, line) = opened();
        line.lock().unwrap().looks.push_back(Ok(None));
        device.set(&Key::Enabled, &on(true), None).unwrap();
        assert_eq!(line.lock().unwrap().looks_taken, 1);
    }

    fn conn(c: &str) -> Vec<(Key, Value)> {
        vec![(Key::Conn, Value::Str(c.to_owned()))]
    }

    #[test]
    fn a_scan_needs_a_conn_that_opens() {
        assert!(Hp59306aDriver.scan(&[]).unwrap().is_empty());
        for c in ["tcp-raw/host", "", "vxi/host", "/dev/oxrok-no-such-node"] {
            assert!(Hp59306aDriver.scan(&conn(c)).unwrap().is_empty(), "{c}");
        }
        assert!(Hp59306aDriver.scan(&[(Key::Samplerate, Value::U64(1))]).is_err());
    }

    /// The next connection, or a failure at a bound, as a blocking accept would hang
    /// a mutant for good.
    fn accept(listener: &TcpListener) -> TcpStream {
        listener.set_nonblocking(true).unwrap();
        let began = Instant::now();
        loop {
            match listener.accept() {
                Ok((peer, _)) => {
                    peer.set_nonblocking(false).unwrap();
                    peer.set_read_timeout(Some(Duration::from_secs(30))).unwrap();
                    return peer;
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    assert!(began.elapsed() < Duration::from_secs(30), "no connection came");
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(e) => panic!("{e}"),
            }
        }
    }

    /// On a real socket: a scan's connection, then an open's, with nothing on the
    /// first; a command, a look with the peer still there, and what a close does.
    #[test]
    fn on_a_socket_a_scan_and_an_open_are_two_connections() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let name = format!("tcp-raw/127.0.0.1/{port}");
        let mut found = Hp59306aDriver.scan(&conn(&name)).unwrap();
        assert_eq!(found.len(), 1);
        let mut first = accept(&listener);
        let mut rest = Vec::new();
        first.read_to_end(&mut rest).unwrap();
        assert_eq!(rest, b"", "the scan sends nothing and closes");
        let device = &mut found[0];
        assert_eq!(device.get(&Key::Conn, None).unwrap(), Value::Str(name.clone()));
        device.open().unwrap();
        let mut second = accept(&listener);
        device.set(&Key::Enabled, &on(true), Some("R4")).unwrap();
        device.close().unwrap();
        second.read_to_end(&mut rest).unwrap();
        assert_eq!(rest, b"A4\n");
    }

    #[test]
    fn on_a_socket_a_peer_that_closed_fails_the_setting() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let name = format!("tcp-raw/127.0.0.1/{port}");
        let mut found = Hp59306aDriver.scan(&conn(&name)).unwrap();
        drop(accept(&listener));
        let device = &mut found[0];
        device.open().unwrap();
        drop(accept(&listener));
        let e = device.set(&Key::Enabled, &on(true), None).unwrap_err().to_string();
        // Which of the FIN and the reset the read reports goes with the system.
        assert!(e.starts_with("device error: Command A123456: "), "{e}");
        assert!(e.ends_with("where its relays lie is not known"), "{e}");
        // And the next one fails too, with no SIGPIPE to die of.
        assert!(device.set(&Key::Enabled, &on(false), None).is_err());
    }

    #[test]
    fn nobody_listening_is_no_device() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        drop(listener);
        let name = format!("tcp-raw/127.0.0.1/{port}");
        assert!(Hp59306aDriver.scan(&conn(&name)).unwrap().is_empty());
    }

    #[test]
    fn a_client_that_starts_gets_a_header_and_an_end() {
        let (mut device, _line) = opened();
        // A queue of one: the start must not wait for it to be read. On a thread,
        // so that one that does fails here.
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let (started, returned) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            device.start(PacketSink::new(tx, 0)).unwrap();
            device.stop().unwrap();
            started.send(()).unwrap();
        });
        returned.recv_timeout(Duration::from_secs(30)).unwrap();
        assert!(matches!(rx.recv().unwrap().1, Packet::Header { .. }));
        assert!(matches!(rx.recv().unwrap().1, Packet::End));
    }
}
