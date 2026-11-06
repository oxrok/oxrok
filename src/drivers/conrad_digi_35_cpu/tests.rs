// Counterpart in sigrok:
//   libsigrok/src/hardware/conrad-digi-35-cpu/api.c,
//   libsigrok/src/hardware/conrad-digi-35-cpu/protocol.c,
//   Copyright (C) 2014 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Frank Stettner, Martin Ling, Soeren Apel, Aurelien Jacobs,
//   Daniel Elstner, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The driver on a fake port, which is where the pause can be seen, and a write
//! that fails without a race.

use super::protocol::{DRAIN_BOUND, Failure, PAUSE, send};
use super::*;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

/// What every port of a test shares.
#[derive(Default)]
struct Line {
    /// Each write that was taken, when it came, and the timeout it had.
    writes: Vec<(Instant, Vec<u8>, Duration)>,
    /// How many opens succeeded.
    opens: usize,
    /// The next opens fail.
    fail_open: bool,
    /// Writes take this many bytes at most.
    takes: Option<usize>,
    /// Writes fail: a port gone.
    write_fails: bool,
    /// What the drains give, once each, and `Ok` when none is left.
    drains: Vec<Result<()>>,
    /// The order of what happened: `write`, `drain MS`, `pause MS`.
    events: Vec<String>,
}

#[derive(Clone, Default)]
struct Shared(Arc<Mutex<Line>>);

impl Shared {
    fn line(&self) -> MutexGuard<'_, Line> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// What went out, each write as a string.
    fn frames(&self) -> Vec<String> {
        self.line().writes.iter().map(|w| String::from_utf8_lossy(&w.1).into_owned()).collect()
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

    fn read(&mut self, _buf: &mut [u8], _timeout: Duration) -> Result<usize> {
        panic!("this device is never read");
    }

    fn write(&mut self, _data: &[u8]) -> Result<()> {
        panic!("a command has a timeout: write_timeout");
    }

    fn write_timeout(&mut self, data: &[u8], timeout: Duration) -> Result<usize> {
        let mut line = self.0.line();
        if line.write_fails {
            return Err(Error::device("write failed"));
        }
        let n = line.takes.map_or(data.len(), |t| t.min(data.len()));
        line.writes.push((Instant::now(), data[..n].to_vec(), timeout));
        line.events.push("write".to_owned());
        Ok(n)
    }

    fn drain(&mut self, timeout: Duration) -> Result<()> {
        let mut line = self.0.line();
        line.events.push(format!("drain {}", timeout.as_millis()));
        if line.drains.is_empty() { Ok(()) } else { line.drains.remove(0) }
    }

    fn purge(&mut self) -> Result<()> {
        panic!("this device has nothing to purge");
    }
}

fn params(line: &str) -> Params {
    Params::parse(line).unwrap()
}

fn device(shared: &Shared) -> ConradDevice {
    ConradDevice::new("/dev/fake".to_owned(), params("9600/8n1"), shared.opener())
}

/// A pause that takes no time and goes in the events.
fn logged_pause(shared: &Shared) -> Pause {
    let shared = shared.clone();
    Box::new(move |t| shared.line().events.push(format!("pause {}", t.as_millis())))
}

/// An open device whose pauses are only logged.
fn logged(shared: &Shared) -> ConradDevice {
    let mut d = opened(shared);
    d.pause = logged_pause(shared);
    d
}

fn opened(shared: &Shared) -> ConradDevice {
    let mut d = device(shared);
    d.open().unwrap();
    d
}

/// What `-c key=text` sends on a new device, through the core's own parse.
fn sent(key: &Key, text: &str) -> Result<String> {
    let value = Value::parse(key.kind(), text)?;
    let shared = Shared::default();
    let mut d = opened(&shared);
    d.set(key, &value, None)?;
    let frames = shared.frames();
    assert_eq!(frames.len(), 1, "{frames:?}");
    Ok(frames[0].clone())
}

#[test]
fn the_voltages_as_sigrok_cli_sends_them() {
    // Each is an emulator case too, against sigrok-cli. 1.15 and 0.15 are the ones
    // single precision moves.
    for (text, frame) in [
        ("0", "V000\r"),
        ("-0", "V000\r"),
        ("0.04", "V000\r"),
        ("0.05", "V001\r"),
        ("0.15", "V002\r"),
        ("1.15", "V011\r"),
        ("3.3", "V033\r"),
        ("5", "V050\r"),
        ("12.34", "V123\r"),
        ("34.96", "V350\r"),
        ("35", "V350\r"),
        ("1e1", "V100\r"),
    ] {
        assert_eq!(sent(&Key::VoltageTarget, text).unwrap(), frame, "{text}");
    }
}

#[test]
fn the_currents_as_sigrok_cli_sends_them() {
    for (text, frame) in [
        ("0", "C000\r"),
        ("-0", "C000\r"),
        ("0.004", "C000\r"),
        ("0.005", "C000\r"),
        ("0.29", "C029\r"),
        ("1", "C100\r"),
        ("1.005", "C100\r"),
        ("2.55", "C255\r"),
    ] {
        assert_eq!(sent(&Key::CurrentLimit, text).unwrap(), frame, "{text}");
    }
}

#[test]
fn protection_goes_through_v() {
    let key = Key::OverCurrentProtectionEnabled;
    assert_eq!(sent(&key, "on").unwrap(), "V900\r");
    assert_eq!(sent(&key, "off").unwrap(), "V901\r");
}

#[test]
fn what_is_out_of_range_is_refused_and_nothing_goes_out() {
    let cases = [
        (Key::VoltageTarget, f64::NAN),
        (Key::VoltageTarget, -f64::NAN),
        (Key::VoltageTarget, f64::INFINITY),
        (Key::VoltageTarget, f64::NEG_INFINITY),
        (Key::VoltageTarget, -0.01),
        (Key::VoltageTarget, 35.00001),
        (Key::VoltageTarget, 35.01),
        (Key::VoltageTarget, -0.001),
        (Key::VoltageTarget, 900.0),
        (Key::CurrentLimit, f64::NAN),
        (Key::CurrentLimit, f64::INFINITY),
        (Key::CurrentLimit, -0.001),
        (Key::CurrentLimit, -0.01),
        (Key::CurrentLimit, 2.5501),
        (Key::CurrentLimit, 2.558),
        (Key::CurrentLimit, 2.56),
    ];
    for (key, v) in cases {
        let shared = Shared::default();
        let mut d = opened(&shared);
        let e = d.set(&key, &Value::F64(v), None).expect_err("refused");
        assert_eq!(e.to_string(), Error::Arg(format!("{} out of range", key.id())).to_string());
        assert_eq!(shared.frames(), [""; 0], "{key:?} {v}");
    }
}

/// The ends of what single precision lets in: see the module's note.
#[test]
fn the_narrowing_widens_the_range_and_moves_values() {
    for (key, value, frame) in [
        (Key::VoltageTarget, 35.000_001_9, "V350\r"),
        (Key::CurrentLimit, 2.550_000_07, "C255\r"),
        (Key::VoltageTarget, -1e-50, "V000\r"),
        (Key::VoltageTarget, 1.249_999_999_9, "V013\r"),
        (Key::VoltageTarget, 1.15, "V011\r"),
    ] {
        let shared = Shared::default();
        let mut d = logged(&shared);
        d.set(&key, &Value::F64(value), None).unwrap();
        assert_eq!(shared.frames(), [frame], "{value}");
    }
}

#[test]
fn a_wrong_kind_of_value_is_refused_and_nothing_goes_out() {
    let shared = Shared::default();
    let mut d = opened(&shared);
    for (key, value) in [
        (Key::VoltageTarget, Value::Bool(true)),
        (Key::CurrentLimit, Value::Bool(true)),
        (Key::OverCurrentProtectionEnabled, Value::F64(1.0)),
    ] {
        assert!(d.set(&key, &value, None).is_err(), "{key:?}");
    }
    assert!(d.set(&Key::Enabled, &Value::Bool(true), None).is_err());
    assert_eq!(shared.frames(), [""; 0]);
}

/// sigrok-cli's `strtof` takes these and sends a number. The shared CLI's parse
/// refuses them, and on a live supply that matters: a typo there goes out as V000.
#[test]
fn the_value_forms_only_sigrok_cli_takes_are_refused() {
    for text in ["", "abc", "3.3V", "0x10", "5,5"] {
        assert!(sent(&Key::VoltageTarget, text).is_err(), "{text:?}");
    }
}

#[test]
fn a_command_gets_the_lines_time_for_five_bytes() {
    let shared = Shared::default();
    let mut port = FakePort(shared.clone());
    for (line, ms) in [("9600/8n1", 15), ("4800/8n1", 20), ("300/8n1", 176)] {
        send(&mut port, &params(line), Command::Voltage(1), &mut |_| {}).unwrap();
        assert_eq!(shared.line().writes.last().unwrap().2, Duration::from_millis(ms), "{line}");
    }
}

/// C's reason for the pause. Only the least gap is asserted: a loaded box stretches
/// one and never shortens it.
#[test]
fn a_command_is_followed_by_50_ms_before_the_next_and_before_the_return() {
    assert_eq!(PAUSE, Duration::from_millis(50));
    let shared = Shared::default();
    let mut d = opened(&shared);
    let values = [1.0, 2.0, 3.0, 4.0];
    let mut returned = Vec::new();
    for v in values {
        d.set(&Key::VoltageTarget, &Value::F64(v), None).unwrap();
        returned.push(Instant::now());
    }
    d.set(&Key::OverCurrentProtectionEnabled, &Value::Bool(true), None).unwrap();
    let line = shared.line();
    for (w, back) in line.writes.iter().zip(&returned) {
        assert!(back.duration_since(w.0) >= Duration::from_millis(50), "the return");
    }
    for pair in line.writes.windows(2) {
        let gap = pair[1].0.duration_since(pair[0].0);
        assert!(gap >= Duration::from_millis(50), "{gap:?}");
    }
    assert_eq!(line.writes.len(), 5);
}

/// A frame's wait comes before its pause, and only after a whole write.
#[test]
fn a_command_writes_drains_and_then_pauses() {
    assert_eq!(DRAIN_BOUND, Duration::from_secs(1));
    let shared = Shared::default();
    let mut d = logged(&shared);
    d.set(&Key::VoltageTarget, &Value::F64(5.0), None).unwrap();
    d.set(&Key::CurrentLimit, &Value::F64(1.0), None).unwrap();
    assert_eq!(
        shared.line().events,
        ["write", "drain 1015", "pause 50", "write", "drain 1015", "pause 50"]
    );
}

/// What each failure says, with what follows it, with no pause on any.
struct Failed {
    message: String,
    /// Whether another set then reaches the port.
    next_goes_out: bool,
    events: Vec<String>,
}

fn failing(arm: impl FnOnce(&mut Line)) -> Failed {
    let shared = Shared::default();
    let mut d = logged(&shared);
    arm(&mut shared.line());
    let e = d.set(&Key::VoltageTarget, &Value::F64(5.0), None).expect_err("it fails");
    let events = shared.line().events.clone();
    let mut line = shared.line();
    line.takes = None;
    line.write_fails = false;
    line.drains.clear();
    line.writes.clear();
    drop(line);
    let again = d.set(&Key::CurrentLimit, &Value::F64(1.0), None);
    let next_goes_out = again.is_ok();
    if let Err(e) = again {
        assert_eq!(
            e.to_string(),
            "device error: an earlier command failed and the supply's state is unknown: C100 \
             did not go out, reopen the device to send more"
        );
        assert_eq!(shared.frames(), [""; 0], "the refused set sends nothing");
        // A reopen ends the refusal, and only that does.
        d.open().unwrap();
        assert!(d.set(&Key::CurrentLimit, &Value::F64(1.0), None).is_err(), "still open");
        d.close().unwrap();
        d.open().unwrap();
        d.pause = logged_pause(&shared);
        d.set(&Key::CurrentLimit, &Value::F64(1.0), None).unwrap();
        assert_eq!(shared.frames(), ["C100\r"]);
    }
    Failed { message: e.to_string(), next_goes_out, events }
}

#[test]
fn a_write_that_takes_none_says_so_and_leaves_the_state_known() {
    let f = failing(|l| l.takes = Some(0));
    assert_eq!(f.message, "device error: V050 did not go out: the port took none of it in 15 ms");
    assert!(f.next_goes_out, "nothing of it is on its way");
    assert_eq!(f.events, ["write"], "no wait and no pause");
}

#[test]
fn a_write_cut_short_says_how_far_and_stops_the_sets() {
    for (takes, words) in [(1, "1 of 5"), (4, "4 of 5")] {
        let f = failing(|l| l.takes = Some(takes));
        assert_eq!(
            f.message,
            format!(
                "device error: V050 was cut short after {words} bytes, which stay in the \
                 supply's buffer and run into the next command"
            )
        );
        assert!(!f.next_goes_out);
        assert_eq!(f.events, ["write"], "no wait and no pause");
    }
}

#[test]
fn a_port_that_fails_says_so_and_stops_the_sets() {
    let f = failing(|l| l.write_fails = true);
    assert_eq!(f.message, "device error: the port failed sending V050: device error: write failed");
    assert!(!f.next_goes_out);
    assert_eq!(f.events, [""; 0], "no wait and no pause");
}

#[test]
fn a_port_that_fails_in_the_wait_has_the_same_words() {
    let f = failing(|l| l.drains.push(Err(Error::device("write failed"))));
    assert_eq!(f.message, "device error: the port failed sending V050: device error: write failed");
    assert!(!f.next_goes_out);
    assert_eq!(f.events, ["write", "drain 1015"], "no pause");
}

#[test]
fn a_frame_stuck_in_the_queue_says_so_and_stops_the_sets() {
    let f = failing(|l| l.drains.push(Err(Error::Timeout)));
    assert_eq!(
        f.message,
        "device error: V050 is still in the port's queue after 1015 ms, and may come late or \
         never"
    );
    assert!(!f.next_goes_out);
    assert_eq!(f.events, ["write", "drain 1015"], "no pause");
}

#[test]
fn only_a_writes_own_failures_have_cs_line() {
    let v = Command::Voltage(50);
    let c = Command::Current(100);
    let line = "Write error for cmd=V";
    let unsent = Failure::Unsent(Duration::from_millis(1));
    assert_eq!(unsent.write_error_line(v).as_deref(), Some(line));
    assert_eq!(Failure::Cut(1).write_error_line(v).as_deref(), Some(line));
    assert_eq!(Failure::Cut(1).write_error_line(c).as_deref(), Some("Write error for cmd=C"));
    assert_eq!(Failure::Port(Error::Timeout).write_error_line(v), None);
    assert_eq!(Failure::Queued(Duration::from_millis(1)).write_error_line(v), None);
}

#[test]
fn which_failures_leave_the_state_unknown() {
    assert!(!Failure::Unsent(Duration::from_millis(1)).leaves_state_unknown());
    assert!(Failure::Cut(1).leaves_state_unknown());
    assert!(Failure::Port(Error::Timeout).leaves_state_unknown());
    assert!(Failure::Queued(Duration::from_millis(1)).leaves_state_unknown());
}

#[test]
fn a_set_on_a_closed_device_says_so_and_sends_nothing() {
    let shared = Shared::default();
    let mut d = device(&shared);
    let e = d.set(&Key::VoltageTarget, &Value::F64(5.0), None).expect_err("closed");
    assert_eq!(e.to_string(), "device error: the port is not open: V050 did not go out");
    d.open().unwrap();
    d.set(&Key::VoltageTarget, &Value::F64(5.0), None).unwrap();
    d.close().unwrap();
    assert!(d.set(&Key::VoltageTarget, &Value::F64(6.0), None).is_err());
    assert_eq!(shared.frames(), ["V050\r"]);
}

#[test]
fn an_open_reopens_once_and_says_when_it_cannot() {
    let shared = Shared::default();
    let mut d = device(&shared);
    assert_eq!(shared.line().opens, 0, "the scan's open is not the device's");
    d.open().unwrap();
    d.open().unwrap();
    assert_eq!(shared.line().opens, 1, "a second open keeps the port");
    d.close().unwrap();
    d.open().unwrap();
    assert_eq!(shared.line().opens, 2);
    d.close().unwrap();
    shared.line().fail_open = true;
    assert!(d.open().is_err());
    assert_eq!(shared.frames(), [""; 0], "an open sends nothing");
}

#[test]
fn nothing_can_be_read_and_a_group_does_not_exist() {
    let shared = Shared::default();
    let mut d = opened(&shared);
    assert!(d.get(&Key::VoltageTarget, None).is_err());
    assert!(d.get(&Key::Conn, None).is_err());
    assert!(d.get(&Key::VoltageTarget, Some("CH1")).is_err());
    assert!(d.options(Some("CH1")).is_err());
    assert!(d.list(&Key::VoltageTarget, Some("CH1")).is_err());
    assert!(d.set(&Key::VoltageTarget, &Value::F64(5.0), Some("CH1")).is_err());
    assert_eq!(shared.frames(), [""; 0]);
    assert_eq!(d.channels().len(), 0);
    assert_eq!(d.channel_groups().len(), 0);
}

#[test]
fn the_lists_are_cs() {
    let shared = Shared::default();
    let d = device(&shared);
    let keys: Vec<Key> = d.options(None).unwrap().into_iter().map(|o| o.key).collect();
    assert_eq!(
        keys,
        [Key::VoltageTarget, Key::CurrentLimit, Key::OverCurrentProtectionEnabled],
        "the order --show prints"
    );
    for option in d.options(None).unwrap() {
        assert!(option.access.set && !option.access.get, "{:?}", option.key);
        assert_eq!(option.access.list, option.key != Key::OverCurrentProtectionEnabled);
    }
    let range = |key| match d.list(&key, None).unwrap() {
        ValueList::RangeStepF64 { min, max, step } => (min, max, step),
        other => panic!("{other:?}"),
    };
    assert_eq!(range(Key::VoltageTarget), (0.0, 35.0, 0.1));
    assert_eq!(range(Key::CurrentLimit), (0.0, 2.55, 0.01));
    assert!(d.list(&Key::OverCurrentProtectionEnabled, None).is_err());
}

/// The opens a scan makes, how many of those ports have gone, and what they wrote.
#[derive(Clone, Default)]
struct ScanLog {
    opens: Arc<Mutex<Vec<(String, Params)>>>,
    closes: Arc<Mutex<usize>>,
    line: Shared,
}

struct CountedPort(FakePort, Arc<Mutex<usize>>);

impl Drop for CountedPort {
    fn drop(&mut self) {
        *self.1.lock().unwrap_or_else(std::sync::PoisonError::into_inner) += 1;
    }
}

impl Port for CountedPort {
    fn name(&self) -> &str {
        self.0.name()
    }
    fn set_params(&mut self, params: Params) -> Result<()> {
        self.0.set_params(params)
    }
    fn set_handshake(&mut self, rts: Option<bool>, dtr: Option<bool>) -> Result<()> {
        self.0.set_handshake(rts, dtr)
    }
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        self.0.read(buf, timeout)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.0.write(data)
    }
    fn write_timeout(&mut self, data: &[u8], timeout: Duration) -> Result<usize> {
        self.0.write_timeout(data, timeout)
    }
    fn drain(&mut self, timeout: Duration) -> Result<()> {
        self.0.drain(timeout)
    }
    fn purge(&mut self) -> Result<()> {
        self.0.purge()
    }
}

impl ScanLog {
    fn open(&self) -> impl Fn(&str, Params) -> Result<Box<dyn Port>> + Send + 'static {
        let log = self.clone();
        move |name: &str, params: Params| {
            log.opens.lock().unwrap().push((name.to_owned(), params));
            let fake = FakePort(log.line.clone());
            Ok(Box::new(CountedPort(fake, log.closes.clone())) as Box<dyn Port>)
        }
    }

    fn opens(&self) -> usize {
        self.opens.lock().unwrap().len()
    }

    fn closes(&self) -> usize {
        *self.closes.lock().unwrap()
    }
}

fn conn() -> (Key, Value) {
    (Key::Conn, Value::Str("/dev/fake".to_owned()))
}

#[test]
fn a_scan_without_a_conn_opens_nothing() {
    let log = ScanLog::default();
    assert!(scan_with(&[], log.open()).unwrap().is_none());
    assert_eq!(log.opens(), 0);
}

#[test]
fn a_scan_opens_closes_and_assumes_and_the_device_opens_again() {
    let log = ScanLog::default();
    let mut d = scan_with(&[conn()], log.open()).unwrap().expect("assumed");
    assert_eq!((log.opens(), log.closes()), (1, 1), "the port has gone again");
    assert_eq!(log.opens.lock().unwrap()[0].1.to_text(), "9600/8n1");
    assert_eq!(log.line.frames(), [""; 0], "and nothing was written");
    assert_eq!(d.info().vendor, "Conrad");
    assert_eq!(d.info().model, "DIGI 35 CPU");
    assert_eq!(d.info().conn, None);
    d.open().unwrap();
    assert_eq!(log.opens(), 2);
    d.close().unwrap();
    assert_eq!(log.closes(), 2);
}

#[test]
fn a_scan_takes_serialcomm_and_the_opens_keep_it() {
    let log = ScanLog::default();
    let line = (Key::Serialcomm, Value::Str("4800/8n1".to_owned()));
    let mut d = scan_with(&[conn(), line], log.open()).unwrap().expect("assumed");
    d.open().unwrap();
    let opens = log.opens.lock().unwrap();
    assert_eq!(opens.len(), 2);
    for (name, line) in opens.iter() {
        assert_eq!((name.as_str(), line.to_text().as_str()), ("/dev/fake", "4800/8n1"));
    }
}

#[test]
fn a_scan_with_no_such_serialcomm_or_port_finds_nothing() {
    let log = ScanLog::default();
    let bad = (Key::Serialcomm, Value::Str("fast".to_owned()));
    assert!(scan_with(&[conn(), bad], log.open()).unwrap().is_none());
    assert_eq!(log.opens(), 0, "the line is read first");
    let fails = |_: &str, _: Params| -> Result<Box<dyn Port>> { Err(Error::device("no port")) };
    assert!(scan_with(&[conn()], fails).unwrap().is_none());
    assert!(scan_with(&[(Key::Enabled, Value::Bool(true))], log.open()).is_err());
    assert!(ConradDigi35CpuDriver.scan(&[]).unwrap().is_empty());
}

#[test]
fn a_capture_is_refused_and_stop_is_a_no_op() {
    let shared = Shared::default();
    let mut d = opened(&shared);
    let (tx, rx) = std::sync::mpsc::sync_channel(4);
    let e = d.start(PacketSink::new(tx, 0)).expect_err("no channels");
    assert_eq!(
        e.to_string(),
        "device error: conrad-digi-35-cpu device /dev/fake has no enabled channels."
    );
    assert!(rx.try_recv().is_err(), "no packet");
    d.stop().unwrap();
    assert_eq!(shared.frames(), [""; 0]);
}
