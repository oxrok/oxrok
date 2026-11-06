// Counterpart in sigrok:
//   libsigrok/src/hardware/beaglelogic/api.c,
//   libsigrok/src/hardware/beaglelogic/beaglelogic_tcp.c,
//   libsigrok/src/hardware/beaglelogic/protocol.c,
//   Copyright (C) 2014-2017 Kumar Abhishek <abhishek@theembeddedkitchen.net>,
//   Copyright (C) 2017 Kumar Abhishek <abhishek@theembeddedkitchen.net>,
//   Copyright (C) 2013 Martin Ling <martin-sigrok@earth.li>,
//   Copyright (C) 2013 poljar (Damir Jelić) <poljarinho@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Aurelien Jacobs, Daniel Elstner, Soeren Apel, Gerhard Sittig,
//   Tilman Sauerbeck, Michał Janiszewski
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

use super::protocol::Times;
use super::*;
use crate::serial::{Params, WriteLog};
use std::collections::{HashMap, VecDeque};
use std::io::{BufRead as _, BufReader, Write as _};
use std::net::TcpListener;
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, RecvTimeoutError, sync_channel};

/// How long a test waits for what must happen: a stop or a limit that does not bite
/// must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);
/// Bounds that no test of anything else should meet.
const FAR: Bounds = Bounds {
    first: Duration::from_secs(600),
    warn: Duration::from_secs(600),
    fail: Duration::from_secs(600),
};
/// A slice's worth of reads, with the bounds of an exchange far off.
const TIMES: Times = Times {
    late: Duration::from_secs(60),
    no_answer: Duration::from_secs(60),
    line_tail: Duration::from_millis(20),
    slice: Duration::from_millis(5),
    drain_quiet: Duration::from_millis(5),
    drain_most: Duration::from_secs(2),
};
/// The most packets a helper takes: a limit ignored on a stream that doesn't stop
/// fills the memory within any deadline.
const MOST_PACKETS: usize = 100_000;

/// What the server does with a command.
#[derive(Clone, Debug)]
enum Reply {
    /// The server's own answer.
    Own,
    /// This line, with its line end.
    Say(String),
    /// These bytes as they are.
    Raw(Vec<u8>),
    /// Nothing.
    Mute,
    /// The connection closes.
    Hangup,
    /// This line, on the Nth read after the command: a read waits, so this is late
    /// by a count, and a stall can't make it less so.
    AfterReads(u32, String),
    /// Pieces of an answer, one for each read.
    Pieces(Vec<Vec<u8>>),
}

/// When a part of a stream comes.
#[derive(Clone, Debug)]
enum Part {
    /// These bytes, a time after `get`.
    At(Duration, Vec<u8>),
    /// The connection closes after what came.
    Close,
}

/// A stream that never ends: this many bytes for every read.
#[derive(Clone, Copy, Debug, Default)]
struct Flood(usize);

#[derive(Debug)]
struct Server {
    replies: HashMap<String, Reply>,
    samplerate: u32,
    unit: u32,
    memalloc: u32,
    bufunitsize: u32,
    stream: Vec<Part>,
    flood: Flood,
    /// After a `close`, on the same connection: a slow module's last read.
    tail: Option<(u32, Vec<u8>)>,
    log: WriteLog,
    /// How many connections were made, the last being the live one.
    connects: usize,
    /// What the live connection has on its way, with the reads it takes, by connection.
    queue: VecDeque<(usize, u32, Vec<u8>)>,
    /// When a stream started, and what of it has yet to come.
    started: Option<Instant>,
    parts: VecDeque<Part>,
    streaming: bool,
    closed: bool,
    line: Vec<u8>,
    no_connect: bool,
    /// How many reads came through at all.
    reads: u64,
}

impl Default for Server {
    fn default() -> Self {
        Server {
            replies: HashMap::new(),
            samplerate: 100_000_000,
            unit: 1,
            memalloc: 65536,
            bufunitsize: 4096,
            stream: Vec::new(),
            flood: Flood::default(),
            tail: None,
            log: WriteLog::default(),
            connects: 0,
            queue: VecDeque::new(),
            started: None,
            parts: VecDeque::new(),
            streaming: false,
            closed: false,
            line: Vec::new(),
            no_connect: false,
            reads: 0,
        }
    }
}

impl Server {
    /// The server's own answer, or nothing for what has none.
    fn own(&mut self, command: &str) -> Option<String> {
        let mut words = command.split(' ');
        let name = words.next().unwrap_or_default();
        Some(match (name, words.next()) {
            ("version", None) => "BeagleLogic 1.0".to_owned(),
            ("samplerate", None) => self.samplerate.to_string(),
            ("sampleunit", None) => self.unit.to_string(),
            ("memalloc", None) => self.memalloc.to_string(),
            ("bufunitsize", None) => self.bufunitsize.to_string(),
            ("sampleunit", Some(v)) => {
                self.unit = v.parse().unwrap();
                "OK".to_owned()
            }
            ("samplerate", Some(v)) => {
                self.samplerate = v.parse().unwrap();
                "OK".to_owned()
            }
            ("triggerflags", Some(_)) => "OK".to_owned(),
            ("get", None) => {
                self.started = Some(Instant::now());
                self.parts = self.stream.iter().cloned().collect();
                self.streaming = true;
                return None;
            }
            ("close", None) => {
                self.streaming = false;
                self.parts.clear();
                if let Some((reads, bytes)) = self.tail.take() {
                    let conn = self.connects;
                    self.queue.push_back((conn, reads, bytes));
                }
                return None;
            }
            _ => panic!("no such command: {command:?}"),
        })
    }

    fn command(&mut self, command: &str) {
        let conn = self.connects;
        match self.replies.get(command).cloned().unwrap_or(Reply::Own) {
            Reply::Own => {
                if let Some(text) = self.own(command) {
                    self.queue.push_back((conn, 0, format!("{text}\r\n").into_bytes()));
                }
            }
            Reply::Say(text) => {
                self.queue.push_back((conn, 0, format!("{text}\r\n").into_bytes()));
            }
            Reply::Raw(bytes) => self.queue.push_back((conn, 0, bytes)),
            Reply::Mute => {}
            Reply::Hangup => self.closed = true,
            Reply::AfterReads(reads, text) => {
                self.queue.push_back((conn, reads, format!("{text}\r\n").into_bytes()));
            }
            Reply::Pieces(pieces) => {
                for piece in pieces {
                    self.queue.push_back((conn, 1, piece));
                }
            }
        }
    }
}

type Shared = Arc<Mutex<Server>>;

/// A connection to the server.
#[derive(Debug)]
struct Fake {
    server: Shared,
    conn: usize,
}

impl Port for Fake {
    fn name(&self) -> &'static str {
        "fake"
    }

    fn set_params(&mut self, _params: Params) -> Result<()> {
        Ok(())
    }

    fn set_handshake(&mut self, _rts: Option<bool>, _dtr: Option<bool>) -> Result<()> {
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let got = {
            let mut s = self.server.lock().unwrap();
            s.reads += 1;
            let live = s.connects == self.conn;
            let mine = s.queue.iter().position(|f| f.0 == self.conn);
            if let Some(at) = mine {
                if s.queue[at].1 > 0 {
                    s.queue[at].1 -= 1;
                    0
                } else {
                    let (_, _, mut bytes) = s.queue.remove(at).unwrap();
                    let n = bytes.len().min(buf.len());
                    buf[..n].copy_from_slice(&bytes[..n]);
                    if n < bytes.len() {
                        let rest = bytes.split_off(n);
                        s.queue.insert(at, (self.conn, 0, rest));
                    }
                    n
                }
            } else if live && s.closed {
                return Err(Error::Disconnected);
            } else if live && s.streaming && s.flood.0 > 0 {
                let n = s.flood.0.min(buf.len());
                buf[..n].fill(0x55);
                n
            } else if live && s.streaming {
                let due = s.started.map(|at| at.elapsed()).unwrap_or_default();
                match s.parts.front() {
                    Some(Part::At(when, _)) if *when <= due => {
                        let Some(Part::At(_, mut bytes)) = s.parts.pop_front() else {
                            unreachable!()
                        };
                        let n = bytes.len().min(buf.len());
                        buf[..n].copy_from_slice(&bytes[..n]);
                        if n < bytes.len() {
                            let rest = bytes.split_off(n);
                            s.parts.push_front(Part::At(Duration::ZERO, rest));
                        }
                        n
                    }
                    Some(Part::Close) => {
                        s.closed = true;
                        return Err(Error::Disconnected);
                    }
                    _ => 0,
                }
            } else {
                0
            }
        };
        if got == 0 {
            std::thread::sleep(timeout.min(Duration::from_millis(5)));
        }
        Ok(got)
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut s = self.server.lock().unwrap();
        if s.closed {
            return Err(Error::Disconnected);
        }
        s.log.push((Instant::now(), data.to_vec()));
        for &b in data {
            if b == b'\n' {
                let line = std::mem::take(&mut s.line);
                let text = String::from_utf8(line).unwrap();
                s.command(&text);
            } else {
                s.line.push(b);
            }
        }
        Ok(())
    }

    fn purge(&mut self) -> Result<()> {
        Ok(())
    }
}

fn server() -> Shared {
    Arc::new(Mutex::new(Server::default()))
}

fn dial(server: &Shared) -> Dial {
    let server = Arc::clone(server);
    Box::new(move || {
        let conn = {
            let mut s = server.lock().unwrap();
            if s.no_connect {
                return Err(Error::device("no connection"));
            }
            s.connects += 1;
            s.closed = false;
            s.streaming = false;
            s.line.clear();
            s.connects
        };
        Ok(Box::new(Fake { server: Arc::clone(&server), conn }) as Box<dyn Port>)
    })
}

/// A device with bounds far off, unopened.
fn device(server: &Shared) -> BeagleLogicDevice {
    let mut d = BeagleLogicDevice::new(14, dial(server));
    d.times = TIMES;
    d.bounds = Some(FAR);
    d
}

fn opened(server: &Shared) -> BeagleLogicDevice {
    let mut d = device(server);
    d.open().unwrap();
    d
}

/// The lines written, as text, with no line ends.
fn written(server: &Shared) -> Vec<String> {
    let s = server.lock().unwrap();
    let all: Vec<u8> = s.log.iter().flat_map(|(_, b)| b.clone()).collect();
    String::from_utf8(all).unwrap().lines().map(str::to_owned).collect()
}

fn connects(server: &Shared) -> usize {
    server.lock().unwrap().connects
}

fn reply(server: &Shared, command: &str, how: Reply) {
    server.lock().unwrap().replies.insert(command.to_owned(), how);
}

fn counter(n: usize) -> Vec<u8> {
    (0..n).map(|i| (i % 251) as u8).collect()
}

fn stream(server: &Shared, parts: Vec<Part>) {
    server.lock().unwrap().stream = parts;
}

fn at_once(bytes: Vec<u8>) -> Part {
    Part::At(Duration::ZERO, bytes)
}

/// A test on its own thread with a bound: one that hangs fails.
fn bounded(test: impl FnOnce() + Send + 'static) {
    let (tx, rx) = sync_channel(1);
    let handle = std::thread::spawn(move || {
        test();
        let _ = tx.send(());
    });
    match rx.recv_timeout(BOUND * 2) {
        Ok(()) => handle.join().unwrap(),
        Err(RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(handle.join().unwrap_err());
        }
        Err(RecvTimeoutError::Timeout) => panic!("the test hung"),
    }
}

type Rx = Receiver<(usize, Packet)>;

fn start(d: &mut BeagleLogicDevice) -> Rx {
    let (tx, rx) = sync_channel(64);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// What a capture sent up to its End. A capture that doesn't end fails here.
fn until_end(rx: &Rx) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut packets = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((_, Packet::End)) => return packets,
            Ok((_, p)) => packets.push(p),
            Err(e) => panic!("no End: {e}, after {} packets", packets.len()),
        }
        assert!(packets.len() < MOST_PACKETS, "a capture that does not end");
    }
}

fn samples(packets: &[Packet]) -> Vec<u8> {
    let mut out = Vec::new();
    for p in packets {
        if let Packet::Logic(l) = p {
            out.extend_from_slice(&l.data);
        }
    }
    out
}

fn failure(packets: &[Packet]) -> Option<String> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(why) => Some(why.clone()),
        _ => None,
    })
}

fn fast_open() -> Vec<&'static str> {
    vec!["samplerate", "sampleunit", "memalloc", "bufunitsize", "triggerflags 1"]
}

// An open.

#[test]
fn an_open_asks_in_c_s_order_and_holds_what_it_heard() {
    bounded(|| {
        let s = server();
        {
            let mut g = s.lock().unwrap();
            g.samplerate = 2_000_000;
            g.unit = 0;
            g.memalloc = 1000;
            g.bufunitsize = 32;
        }
        let d = opened(&s);
        assert_eq!(written(&s), fast_open());
        assert_eq!(connects(&s), 1);
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(2_000_000));
        assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(10_000_000));
        assert_eq!(d.get(&Key::CaptureRatio, None).unwrap(), Value::U64(0));
        assert_eq!(d.get(&Key::NumLogicChannels, None).unwrap(), Value::I32(14));
        assert_eq!(d.settings.unit, Unit::Sixteen);
        assert_eq!(d.settings.buffersize, 1000);
        assert_eq!(d.settings.bufunitsize, 32);
        assert!(!d.settings.oneshot);
    });
}

#[test]
fn an_open_refuses_what_is_no_number_and_can_be_tried_again() {
    bounded(|| {
        for (command, answer) in [
            ("samplerate", "abc"),
            ("samplerate", ""),
            ("samplerate", "-5"),
            ("samplerate", "0"),
            ("sampleunit", "abc"),
            ("sampleunit", "2"),
            ("memalloc", "abc"),
            ("memalloc", "-1"),
            ("bufunitsize", "abc"),
        ] {
            let s = server();
            reply(&s, command, Reply::Say(answer.to_owned()));
            let mut d = device(&s);
            let e = d.open().unwrap_err().to_string();
            assert!(e.contains(command), "{command} {answer}: {e}");
            assert!(d.start(PacketSink::new(sync_channel(1).0, 0)).is_err());
            s.lock().unwrap().replies.clear();
            d.open().unwrap();
            assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        }
    });
}

#[test]
fn a_0_buffer_is_a_number_and_is_kept() {
    bounded(|| {
        let s = server();
        s.lock().unwrap().memalloc = 0;
        let d = opened(&s);
        assert_eq!(d.settings.buffersize, 0);
    });
}

#[test]
fn a_silent_open_fails_at_its_bound_and_says_which_command() {
    bounded(|| {
        let s = server();
        reply(&s, "memalloc", Reply::Mute);
        let mut d = device(&s);
        d.times.no_answer = Duration::from_millis(50);
        let e = d.open().unwrap_err().to_string();
        assert!(e.contains("No answer to 'memalloc'"), "{e}");
    });
}

#[test]
fn an_open_goes_on_past_a_triggerflags_with_no_answer_refused_or_closed() {
    bounded(|| {
        for how in [Reply::Say("ERR".to_owned()), Reply::Mute, Reply::Hangup] {
            let s = server();
            reply(&s, "triggerflags 1", how);
            let mut d = device(&s);
            d.times.no_answer = Duration::from_millis(50);
            d.open().unwrap();
            assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        }
    });
}

#[test]
fn an_open_fails_on_a_connection_that_went_or_never_came() {
    bounded(|| {
        let s = server();
        reply(&s, "memalloc", Reply::Hangup);
        let e = device(&s).open().unwrap_err().to_string();
        assert!(e.contains("The device closed the connection"), "{e}");
        let s = server();
        s.lock().unwrap().no_connect = true;
        assert!(device(&s).open().is_err());
    });
}

#[test]
fn a_second_open_is_nothing() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        d.open().unwrap();
        assert_eq!(connects(&s), 1);
        assert_eq!(written(&s).len(), 5);
    });
}

#[test]
fn a_close_says_nothing_and_an_open_after_it_connects_again() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        d.close().unwrap();
        assert_eq!(written(&s).len(), 5);
        assert!(d.start(PacketSink::new(sync_channel(1).0, 0)).is_err());
        d.open().unwrap();
        assert_eq!(connects(&s), 2);
        assert_eq!(written(&s).len(), 10);
    });
}

// An answer.

fn link(server: &Shared, times: Times) -> Link {
    let mut l = Link::new(dial(server));
    l.times = times;
    l.connect().unwrap();
    l
}

#[test]
fn a_late_answer_is_c_s_error_and_leaves_the_line_in_step() {
    bounded(|| {
        let s = server();
        // 40 reads of at least 5 ms: past 50 ms however the box runs.
        reply(&s, "samplerate", Reply::AfterReads(40, "123".to_owned()));
        let mut l = link(&s, Times { late: Duration::from_millis(50), ..TIMES });
        assert!(matches!(l.ask("samplerate"), Err(Error::Timeout)));
        assert!(!l.is_stale());
        assert_eq!(l.ask("memalloc").unwrap(), "65536");
        assert_eq!(connects(&s), 1);
    });
}

#[test]
fn an_answer_within_the_time_is_taken() {
    bounded(|| {
        let s = server();
        reply(&s, "samplerate", Reply::AfterReads(2, "123".to_owned()));
        let mut l = link(&s, TIMES);
        assert_eq!(l.ask("samplerate").unwrap(), "123");
    });
}

#[test]
fn a_late_answer_past_the_wait_can_t_be_the_next_one_s() {
    bounded(|| {
        let s = server();
        // Comes on its connection long after the wait gave up.
        reply(&s, "samplerate", Reply::AfterReads(1000, "123".to_owned()));
        let mut l = link(&s, Times { no_answer: Duration::from_millis(30), ..TIMES });
        let e = l.ask("samplerate").unwrap_err().to_string();
        assert!(e.contains("No answer to 'samplerate'"), "{e}");
        assert!(l.is_stale());
        assert_eq!(l.ask("memalloc").unwrap(), "65536", "on a new connection");
        assert_eq!(connects(&s), 2);
        assert!(!l.is_stale());
    });
}

#[test]
fn an_answer_cut_in_two_is_one_answer() {
    bounded(|| {
        let s = server();
        reply(
            &s,
            "samplerate",
            Reply::Pieces(vec![b"123".to_vec(), b"45\r".to_vec(), b"\n".to_vec()]),
        );
        let mut l = link(&s, TIMES);
        assert_eq!(l.ask_number("samplerate").unwrap(), 12345);
        assert!(!l.is_stale());
    });
}

#[test]
fn an_answer_with_no_line_end_is_taken_after_a_moment() {
    bounded(|| {
        let s = server();
        reply(&s, "version", Reply::Raw(b"BeagleLogic 1.0".to_vec()));
        let mut l = link(&s, TIMES);
        assert_eq!(l.ask("version").unwrap(), "BeagleLogic 1.0");
    });
}

#[test]
fn bytes_behind_an_answer_put_the_next_on_a_new_connection() {
    bounded(|| {
        let s = server();
        reply(&s, "samplerate", Reply::Raw(b"123\r\n456\r\n".to_vec()));
        let mut l = link(&s, TIMES);
        assert_eq!(l.ask("samplerate").unwrap(), "123");
        assert!(l.is_stale());
        assert_eq!(l.ask("memalloc").unwrap(), "65536");
        assert_eq!(connects(&s), 2);
    });
}

#[test]
fn an_endless_line_is_no_usable_answer() {
    bounded(|| {
        let s = server();
        reply(&s, "version", Reply::Raw(vec![b'x'; 300]));
        let mut l = link(&s, TIMES);
        let e = l.ask("version").unwrap_err().to_string();
        assert!(e.contains("No usable answer to 'version'"), "{e}");
        assert!(l.is_stale());
    });
}

// Settings.

#[test]
fn a_refused_samplerate_leaves_the_value_and_says_so() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        reply(&s, "samplerate 1000", Reply::Say("ERR".to_owned()));
        let e = d.set(&Key::Samplerate, &Value::U64(1000), None).unwrap_err().to_string();
        assert!(e.contains("The device refused 'samplerate 1000'"), "{e}");
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
    });
}

#[test]
fn a_set_with_no_answer_fails_and_the_next_finds_out_on_a_new_connection() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        d.link.as_mut().unwrap().times.no_answer = Duration::from_millis(30);
        reply(&s, "samplerate 1000", Reply::Mute);
        assert!(d.set(&Key::Samplerate, &Value::U64(1000), None).is_err());
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        d.set(&Key::Samplerate, &Value::U64(2000), None).unwrap();
        assert_eq!(connects(&s), 2);
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(2000));
    });
}

#[test]
fn a_rate_is_what_the_module_keeps() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        d.set(&Key::Samplerate, &Value::U64(60_000_000), None).unwrap();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        assert_eq!(written(&s).last().unwrap(), "samplerate 60000000");
        d.set(&Key::Samplerate, &Value::U64(30_000_000), None).unwrap();
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(33_333_333));
    });
}

#[test]
fn a_rate_past_32_bits_goes_nowhere() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        let e = d.set(&Key::Samplerate, &Value::U64(4_294_968_296), None).unwrap_err().to_string();
        assert!(e.contains("at most 100 MHz, not 4294968296 Hz"), "{e}");
        assert_eq!(written(&s).len(), 5);
    });
}

#[test]
fn a_limit_makes_the_stream_one_shot_if_the_device_takes_it() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        reply(&s, "triggerflags 0", Reply::Say("ERR".to_owned()));
        let e = d.set(&Key::LimitSamples, &Value::U64(7), None).unwrap_err().to_string();
        assert!(e.contains("The device refused 'triggerflags 0'"), "{e}");
        assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(10_000_000));
        assert!(!d.settings.oneshot);
        s.lock().unwrap().replies.clear();
        d.set(&Key::LimitSamples, &Value::U64(7), None).unwrap();
        assert_eq!(d.get(&Key::LimitSamples, None).unwrap(), Value::U64(7));
        assert!(d.settings.oneshot);
        assert_eq!(written(&s).last().unwrap(), "triggerflags 0");
    });
}

#[test]
fn a_ratio_is_a_percentage_and_stays_on_the_host() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        d.set(&Key::CaptureRatio, &Value::U64(100), None).unwrap();
        assert_eq!(d.get(&Key::CaptureRatio, None).unwrap(), Value::U64(100));
        assert!(d.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
        assert_eq!(d.get(&Key::CaptureRatio, None).unwrap(), Value::U64(100));
        assert_eq!(written(&s).len(), 5);
    });
}

#[test]
fn the_options_are_c_s_in_c_s_order() {
    let s = server();
    let d = device(&s);
    let keys: Vec<_> = d.options(None).unwrap().into_iter().map(|o| o.key).collect();
    assert_eq!(
        keys,
        [
            Key::Continuous,
            Key::LimitSamples,
            Key::Samplerate,
            Key::TriggerMatch,
            Key::CaptureRatio,
            Key::NumLogicChannels,
        ]
    );
    assert!(matches!(
        d.list(&Key::Samplerate, None).unwrap(),
        ValueList::RangeStepU64 { min: 10, max: 100_000_000, step: 1 }
    ));
    assert!(d.get(&Key::Conn, None).is_err());
    assert!(d.list(&Key::Conn, None).is_err());
    assert_eq!(d.info().model, "BeagleLogic");
    assert_eq!(d.info().version, "1.0");
    assert_eq!(d.channels().len(), 14);
    assert_eq!(d.channels()[9].name, "P8_29");
}

// A capture.

#[test]
fn a_capture_says_what_c_says_in_c_s_order() {
    bounded(|| {
        let s = server();
        stream(&s, vec![at_once(counter(100))]);
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        let rx = start(&mut d);
        let p = until_end(&rx);
        assert!(matches!(p[0], Packet::Header { .. }));
        assert_eq!(samples(&p), counter(8));
        assert_eq!(failure(&p), None);
        d.close().unwrap();
        let mut want = fast_open();
        want.extend(["triggerflags 0", "sampleunit 0", "get", "close"]);
        assert_eq!(written(&s), want);
        assert_eq!(connects(&s), 1);
    });
}

#[test]
fn the_width_goes_with_the_channels_enabled_past_the_eighth() {
    bounded(|| {
        let s = server();
        stream(&s, vec![at_once(counter(100))]);
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        for pos in 8..14 {
            d.enable_channel(pos, false).unwrap();
        }
        let p = until_end(&start(&mut d));
        assert_eq!(samples(&p), counter(4));
        match p.iter().find(|x| matches!(x, Packet::Logic(_))) {
            Some(Packet::Logic(l)) => assert_eq!(l.unit_size, 1),
            other => panic!("{other:?}"),
        }
        assert!(written(&s).contains(&"sampleunit 1".to_owned()));
        d.enable_channel(13, true).unwrap();
        assert!(d.start(PacketSink::new(sync_channel(64).0, 0)).is_ok());
        d.close().unwrap();
        assert!(written(&s).contains(&"sampleunit 0".to_owned()));
    });
}

#[test]
fn a_refused_width_is_a_refused_start_and_the_stream_never_starts() {
    bounded(|| {
        let s = server();
        reply(&s, "sampleunit 0", Reply::Say("ERR".to_owned()));
        let mut d = opened(&s);
        let e = d.start(PacketSink::new(sync_channel(64).0, 0)).unwrap_err().to_string();
        assert!(e.contains("The device refused 'sampleunit 0'"), "{e}");
        assert!(!written(&s).contains(&"get".to_owned()));
    });
}

#[test]
fn a_real_pace_gives_all_of_a_limit_cut_inside_a_buffer() {
    bounded(|| {
        // Whole buffers, spaced, as the module hands them out. The limit ends in the
        // third, with nothing at all for the first gap.
        let s = server();
        let data = counter(12288);
        let gap = Duration::from_millis(40);
        stream(
            &s,
            vec![
                Part::At(gap, data[..4096].to_vec()),
                Part::At(gap * 2, data[4096..8192].to_vec()),
                Part::At(gap * 3, data[8192..].to_vec()),
            ],
        );
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(5000), None).unwrap();
        let p = until_end(&start(&mut d));
        assert_eq!(samples(&p), data[..10000]);
        assert_eq!(failure(&p), None);
        d.close().unwrap();
        assert_eq!(written(&s).last().unwrap(), "close");
    });
}

#[test]
fn a_stream_in_odd_pieces_stays_in_step() {
    bounded(|| {
        let s = server();
        let data = counter(1000);
        let every = Duration::from_millis(2);
        let parts = data
            .chunks(37)
            .enumerate()
            .map(|(i, c)| Part::At(every * u32::try_from(i).unwrap(), c.to_vec()))
            .collect();
        stream(&s, parts);
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(500), None).unwrap();
        let p = until_end(&start(&mut d));
        assert_eq!(samples(&p), data);
        for packet in &p {
            if let Packet::Logic(l) = packet {
                assert_eq!(l.data.len() % 2, 0);
            }
        }
    });
}

#[test]
fn a_one_shot_stream_ends_with_the_buffer_with_the_limit_unmet() {
    bounded(|| {
        let s = server();
        s.lock().unwrap().memalloc = 100;
        stream(&s, vec![at_once(counter(100))]);
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let p = until_end(&start(&mut d));
        assert_eq!(samples(&p).len(), 100);
        assert_eq!(failure(&p), None, "a natural end");
    });
}

#[test]
fn a_trigger_goes_through_the_capture() {
    bounded(|| {
        let s = server();
        let mut data = vec![0u8; 20];
        data.extend_from_slice(&[1, 0, 9, 0, 8, 0]);
        stream(&s, vec![at_once(data)]);
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        d.set(&Key::CaptureRatio, &Value::U64(25), None).unwrap();
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::Rising);
        d.set_trigger(Some(t)).unwrap();
        let p = until_end(&start(&mut d));
        let at = p.iter().position(|x| matches!(x, Packet::Trigger)).unwrap();
        assert_eq!(samples(&p[..at]), [0, 0], "one pre-trigger sample");
        assert_eq!(samples(&p[at..]), [1, 0, 9, 0, 8, 0]);
    });
}

#[test]
fn a_match_on_a_disabled_channel_is_skipped() {
    bounded(|| {
        let s = server();
        let d = {
            let mut d = opened(&s);
            let mut t = Trigger::new();
            t.add(0, 0, TriggerMatch::Rising);
            t.add(0, 3, TriggerMatch::One);
            d.set_trigger(Some(t)).unwrap();
            d.enable_channel(3, false).unwrap();
            d
        };
        let mut stl = d.soft_trigger(Unit::Eight, Some(4)).unwrap();
        assert_eq!(stl.check(&[0, 1]).map(|(at, _)| at), Some(1));
    });
}

#[test]
fn a_trigger_with_more_than_levels_and_edges_is_refused() {
    let s = server();
    let mut d = device(&s);
    let mut t = Trigger::new();
    t.add(0, 0, TriggerMatch::Rising);
    t.set_count(0, crate::core::TriggerCount { count: 3, consecutive: false });
    assert!(d.set_trigger(Some(t)).is_err());
}

#[test]
fn a_trigger_with_no_fire_ends_with_the_buffer() {
    bounded(|| {
        let s = server();
        s.lock().unwrap().memalloc = 100;
        stream(&s, vec![at_once(vec![0; 100])]);
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(8), None).unwrap();
        let mut t = Trigger::new();
        t.add(0, 0, TriggerMatch::One);
        d.set_trigger(Some(t)).unwrap();
        let p = until_end(&start(&mut d));
        assert_eq!(samples(&p).len(), 0);
        assert_eq!(failure(&p), None);
    });
}

// A stop.

#[test]
fn a_stop_ends_a_silent_stream_with_a_close() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        let rx = start(&mut d);
        let deadline = Instant::now() + BOUND;
        while !written(&s).contains(&"get".to_owned()) {
            assert!(Instant::now() < deadline, "no get");
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        let p = until_end(&rx);
        assert_eq!(failure(&p), None);
        d.close().unwrap();
        assert_eq!(written(&s).last().unwrap(), "close");
    });
}

#[test]
fn a_stop_ends_a_stream_that_floods() {
    bounded(|| {
        let s = server();
        s.lock().unwrap().flood = Flood(1024);
        let mut d = opened(&s);
        let rx = start(&mut d);
        // Wait for data, so the stop lands in the stream.
        let deadline = Instant::now() + BOUND;
        loop {
            let (_, p) = rx.recv_timeout(BOUND).unwrap();
            if matches!(p, Packet::Logic(_)) {
                break;
            }
            assert!(Instant::now() < deadline);
        }
        d.stop().unwrap();
        // A close joins: it holds the time a drain has against a flood.
        d.close().unwrap();
        assert_eq!(written(&s).last().unwrap(), "close");
    });
}

#[test]
fn a_full_queue_does_not_hold_a_close_for_ever() {
    bounded(|| {
        let s = server();
        s.lock().unwrap().flood = Flood(1024);
        let mut d = opened(&s);
        // A queue of one, never read.
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        let began = Instant::now();
        d.stop().unwrap();
        d.close().unwrap();
        assert!(began.elapsed() < BOUND, "{:?}", began.elapsed());
        let writes = written(&s).len();
        std::thread::sleep(Duration::from_millis(30));
        assert_eq!(written(&s).len(), writes, "nothing after a close");
        // The thread let go of its sink.
        while rx.try_recv().is_ok() {}
        assert!(matches!(
            rx.recv_timeout(Duration::from_secs(1)),
            Err(RecvTimeoutError::Disconnected)
        ));
        assert_eq!(written(&s).last().unwrap(), "close");
    });
}

#[test]
fn a_device_dropped_in_a_capture_tells_the_server() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        let rx = start(&mut d);
        let deadline = Instant::now() + BOUND;
        while !written(&s).contains(&"get".to_owned()) {
            assert!(Instant::now() < deadline, "no get");
            std::thread::sleep(Duration::from_millis(1));
        }
        drop(d);
        assert_eq!(written(&s).last().unwrap(), "close");
        drop(rx);
    });
}

#[test]
fn a_set_and_a_start_in_a_capture_are_busy_and_stay_off_the_wire() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        let rx = start(&mut d);
        assert!(matches!(d.set(&Key::Samplerate, &Value::U64(1000), None), Err(Error::Busy)));
        assert!(matches!(d.set(&Key::LimitSamples, &Value::U64(1000), None), Err(Error::Busy)));
        assert!(matches!(d.set(&Key::CaptureRatio, &Value::U64(1), None), Err(Error::Busy)));
        assert!(matches!(d.start(PacketSink::new(sync_channel(1).0, 0)), Err(Error::Busy)));
        assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
        d.stop().unwrap();
        until_end(&rx);
        d.close().unwrap();
        assert!(!written(&s).contains(&"samplerate 1000".to_owned()));
    });
}

// A second capture.

#[test]
fn what_a_slow_module_sends_after_close_can_t_answer_the_next_command() {
    bounded(|| {
        let s = server();
        stream(&s, vec![at_once(counter(100))]);
        // The last read of a stream, later than a drain looks.
        s.lock().unwrap().tail = Some((1000, b"77\r\n".to_vec()));
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        until_end(&start(&mut d));
        d.set(&Key::Samplerate, &Value::U64(1000), None).unwrap();
        assert_eq!(connects(&s), 2, "on a new connection");
        let p = until_end(&start(&mut d));
        assert_eq!(samples(&p), counter(8), "the second capture from the start");
        assert_eq!(connects(&s), 2);
    });
}

#[test]
fn a_capture_after_a_capture_starts_clean_and_lets_go_of_the_old_sink() {
    bounded(|| {
        let s = server();
        stream(&s, vec![at_once(counter(100))]);
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(samples(&until_end(&rx)), counter(8));
        let rx2 = start(&mut d);
        assert!(matches!(
            rx.recv_timeout(Duration::from_secs(5)),
            Err(RecvTimeoutError::Disconnected)
        ));
        assert_eq!(samples(&until_end(&rx2)), counter(8));
        d.close().unwrap();
    });
}

// Silence and ends.

fn tiny(first: u64, warn: u64, fail: u64) -> Bounds {
    Bounds {
        first: Duration::from_millis(first),
        warn: Duration::from_millis(warn),
        fail: Duration::from_millis(fail),
    }
}

#[test]
fn a_limited_capture_with_no_sample_fails_at_its_bound_and_still_closes() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        d.bounds = Some(tiny(30, 600_000, 600_000));
        let p = until_end(&start(&mut d));
        let why = failure(&p).unwrap();
        assert!(why.contains("has sent no data in"), "{why}");
        d.close().unwrap();
        assert_eq!(written(&s).last().unwrap(), "close");
    });
}

#[test]
fn a_limited_capture_that_stopped_sending_fails_only_at_the_long_bound() {
    bounded(|| {
        let s = server();
        stream(&s, vec![at_once(counter(2))]);
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        // The first bound is short, and has no say after a sample.
        d.bounds = Some(tiny(1, 10, 200));
        let began = Instant::now();
        let p = until_end(&start(&mut d));
        let why = failure(&p).unwrap();
        assert!(why.contains("has sent no data for"), "{why}");
        assert_eq!(samples(&p), counter(2), "what came is good");
        assert!(began.elapsed() >= Duration::from_millis(200));
    });
}

#[test]
fn a_continuous_capture_does_not_fail_on_silence() {
    bounded(|| {
        let s = server();
        let mut d = opened(&s);
        d.bounds = Some(tiny(1, 1, 1));
        let rx = start(&mut d);
        // Many bounds' time.
        assert!(matches!(rx.recv_timeout(BOUND), Ok((_, Packet::Header { .. }))));
        assert!(matches!(
            rx.recv_timeout(Duration::from_millis(100)),
            Err(RecvTimeoutError::Timeout)
        ));
        d.stop().unwrap();
        assert_eq!(failure(&until_end(&rx)), None);
    });
}

#[test]
fn a_slow_consumer_is_no_silence() {
    bounded(|| {
        let s = server();
        let data = counter(64);
        stream(&s, data.chunks(8).map(|c| at_once(c.to_vec())).collect());
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(32), None).unwrap();
        d.bounds = Some(tiny(30, 30, 30));
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Each packet's wait in the send is many bounds long.
        let mut p = Vec::new();
        loop {
            std::thread::sleep(Duration::from_millis(50));
            match rx.recv_timeout(BOUND) {
                Ok((_, Packet::End)) => break,
                Ok((_, x)) => p.push(x),
                Err(e) => panic!("{e}"),
            }
        }
        assert_eq!(failure(&p), None);
        assert_eq!(samples(&p), data);
    });
}

#[test]
fn bytes_put_a_bound_back_across_many_bounds() {
    bounded(|| {
        let s = server();
        let data = counter(40);
        let every = Duration::from_millis(30);
        let parts = data
            .chunks(4)
            .enumerate()
            .map(|(i, c)| Part::At(every * u32::try_from(i).unwrap(), c.to_vec()))
            .collect();
        stream(&s, parts);
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(20), None).unwrap();
        // More than a gap and less than the whole: removing the reset fails this.
        d.bounds = Some(Bounds {
            first: Duration::from_secs(30),
            warn: Duration::from_millis(100),
            fail: Duration::from_millis(150),
        });
        let p = until_end(&start(&mut d));
        assert_eq!(failure(&p), None);
        assert_eq!(samples(&p), data);
    });
}

#[test]
fn a_closed_connection_fails_a_capture_at_once_keeping_what_came() {
    bounded(|| {
        let s = server();
        stream(&s, vec![at_once(counter(4)), Part::Close]);
        let mut d = opened(&s);
        d.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
        let p = until_end(&start(&mut d));
        assert_eq!(
            failure(&p).unwrap(),
            Error::device("The device closed the connection").to_string()
        );
        assert_eq!(samples(&p), counter(4));
        d.close().unwrap();
        assert_eq!(written(&s).last().unwrap(), "get", "no one to tell");
    });
}

#[test]
fn a_connection_closed_ahead_of_the_stream_fails_the_capture() {
    bounded(|| {
        let s = server();
        reply(&s, "sampleunit 0", Reply::Raw(b"OK\r\n".to_vec()));
        let mut d = opened(&s);
        // Goes in the drain ahead of `get`, where C spins for good.
        let (tx, rx) = sync_channel(64);
        s.lock().unwrap().replies.insert("sampleunit 0".to_owned(), Reply::Hangup);
        let outcome = d.start(PacketSink::new(tx, 0));
        assert!(outcome.is_err(), "the answer is what found it");
        drop(rx);
    });
}

#[test]
fn a_flood_of_a_consumer_gone_ends_with_no_failure() {
    bounded(|| {
        let s = server();
        s.lock().unwrap().flood = Flood(1024);
        let mut d = opened(&s);
        let rx = start(&mut d);
        drop(rx);
        // Nobody listens, so the thread ends, and a close gets the link.
        let deadline = Instant::now() + BOUND;
        while d.capturing.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline, "the thread goes on");
            std::thread::sleep(Duration::from_millis(1));
        }
        d.close().unwrap();
        assert_eq!(written(&s).last().unwrap(), "close");
    });
}

// A scan, on real sockets.

/// A server on a real socket, answering lines by a function until it says stop.
fn listen(answer: impl Fn(&str) -> Option<String> + Send + 'static) -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    std::thread::spawn(move || {
        while let Ok((stream, _)) = listener.accept() {
            let mut reader = BufReader::new(stream.try_clone().unwrap());
            let mut stream = stream;
            let mut line = String::new();
            while reader.read_line(&mut line).is_ok_and(|n| n > 0) {
                match answer(line.trim()) {
                    Some(text) => {
                        if stream.write_all(text.as_bytes()).is_err() {
                            break;
                        }
                    }
                    None => break,
                }
                line.clear();
            }
        }
    });
    port
}

fn scan(port: u16, more: &[(Key, Value)]) -> Vec<Box<dyn Device>> {
    let mut options = vec![(Key::Conn, Value::Str(format!("tcp-raw/127.0.0.1/{port}")))];
    options.extend_from_slice(more);
    BeagleLogicDriver.scan(&options).unwrap()
}

#[test]
fn a_scan_finds_what_says_it_is_one() {
    bounded(|| {
        let port = listen(|_| Some("BeagleLogic Standalone 1.0\r\n".to_owned()));
        let found = scan(port, &[]);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].channels().len(), 14);
        assert_eq!(found[0].info().conn, None);
        let port = listen(|_| Some("Something else\r\n".to_owned()));
        assert_eq!(scan(port, &[]).len(), 0);
        let port = listen(|_| None);
        assert_eq!(scan(port, &[]).len(), 0, "closed at version");
    });
}

#[test]
fn logic_channels_makes_8_or_14() {
    bounded(|| {
        let port = listen(|_| Some("BeagleLogic 1.0\r\n".to_owned()));
        for (asked, count) in [(-1, 8), (0, 8), (4, 8), (8, 8), (9, 14), (14, 14), (100, 14)] {
            let found = scan(port, &[(Key::NumLogicChannels, Value::I32(asked))]);
            assert_eq!(found[0].channels().len(), count, "{asked}");
        }
        assert!(
            BeagleLogicDriver.scan(&[(Key::NumLogicChannels, Value::Str("x".to_owned()))]).is_err()
        );
        assert!(BeagleLogicDriver.scan(&[(Key::Samplerate, Value::U64(1))]).is_err());
    });
}

#[test]
fn a_scan_with_no_way_in_finds_nothing() {
    bounded(|| {
        assert_eq!(BeagleLogicDriver.scan(&[]).unwrap().len(), 0);
        for conn in ["tcp-raw", "tcp-raw/h", "serial/h/1"] {
            let options = [(Key::Conn, Value::Str(conn.to_owned()))];
            assert_eq!(BeagleLogicDriver.scan(&options).unwrap().len(), 0, "{conn}");
        }
    });
}

#[test]
fn a_connect_says_what_c_says() {
    let err = |host: &str, port: &str| dial_tcp(host, port).err().unwrap().to_string();
    assert!(err("127.0.0.1", "no-such").contains("Address lookup failed: 127.0.0.1:no-such: "));
    let unused = {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        l.local_addr().unwrap().port()
    };
    let e = err("127.0.0.1", &unused.to_string());
    assert!(e.contains(&format!("Failed to connect to 127.0.0.1:{unused}: ")), "{e}");
    assert!(!e.contains("os error"), "{e}");
}

/// The whole of it on a socket: a real close in the stream, a real capture.
#[test]
fn a_capture_on_a_real_socket() {
    bounded(|| {
        let got_close = Arc::new(AtomicBool::new(false));
        let close = Arc::clone(&got_close);
        let port = listen(move |line| {
            Some(match line {
                "version" => "BeagleLogic 1.0\r\n".to_owned(),
                "samplerate" => "1000000\r\n".to_owned(),
                "sampleunit" => "1\r\n".to_owned(),
                "memalloc" => "100\r\n".to_owned(),
                "bufunitsize" => "32\r\n".to_owned(),
                "get" => "abcdefgh".to_owned(),
                "close" => {
                    close.store(true, Ordering::SeqCst);
                    String::new()
                }
                _ => "OK\r\n".to_owned(),
            })
        });
        let mut found = scan(port, &[]);
        let mut d = found.pop().unwrap();
        d.open().unwrap();
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        for pos in 8..14 {
            d.enable_channel(pos, false).unwrap();
        }
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let p = until_end(&rx);
        assert_eq!(samples(&p), b"ab");
        d.close().unwrap();
        let deadline = Instant::now() + BOUND;
        while !got_close.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline, "no close");
            std::thread::sleep(Duration::from_millis(1));
        }
    });
}

#[test]
fn a_real_connection_s_end_in_the_stream_fails_the_capture() {
    bounded(|| {
        let port = listen(|line| match line {
            "version" => Some("BeagleLogic 1.0\r\n".to_owned()),
            "samplerate" => Some("1000000\r\n".to_owned()),
            "sampleunit" => Some("1\r\n".to_owned()),
            "memalloc" => Some("100\r\n".to_owned()),
            "bufunitsize" => Some("32\r\n".to_owned()),
            "get" => None,
            _ => Some("OK\r\n".to_owned()),
        });
        let mut found = scan(port, &[]);
        let mut d = found.pop().unwrap();
        d.open().unwrap();
        let (tx, rx) = sync_channel(64);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let p = until_end(&rx);
        assert_eq!(
            failure(&p).unwrap(),
            Error::device("The device closed the connection").to_string()
        );
        d.close().unwrap();
    });
}
