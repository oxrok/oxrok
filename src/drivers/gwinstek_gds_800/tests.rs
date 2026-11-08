//! Tests on a fake scope: one that answers at once, one at a line's real pace, one
//! that sends its bytes in bursts, and the pure rules on their own clock.

use super::*;
use crate::scpi::serial::Serial;
use crate::serial::{Params, Port};
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// A capture test's bound. A mutant must fail here, not hang.
const BOUND: Duration = Duration::from_secs(20);

const VOLT: &str = "1.00V\n";
// Of the flips of each scale answer that give a wrong reading, how many warn.
const FLIPS_WARNED_V: usize = 10;
const FLIPS_WARNED_MV: usize = 10;
const FLIPS_WARNED_NR3: usize = 17;

/// What the fake says, by what it was asked.
#[derive(Default)]
struct Scope {
    writes: crate::serial::WriteLog,
    /// What is on the line: bytes, and when each may be read.
    input: VecDeque<(Option<Instant>, u8)>,
    /// `:ACQ<X>:MEM?`'s answers, in turn; the last one stays.
    blocks: Vec<Vec<u8>>,
    block_at: usize,
    scale: Vec<u8>,
    /// Nothing passes once this many bytes were read, until a test takes it away.
    hold_after: Option<usize>,
    /// The read that finds this many bytes read takes this long and finds nothing, once:
    /// a stall inside a read, with the next byte there for the next one.
    stall_at: Option<(usize, Duration)>,
    /// A read is in that stall now.
    in_stall: bool,
    /// `:ACQ<X>:MEM?` from this one on gets no answer.
    silent_from: Option<usize>,
    /// Time per byte, as a line at this rate has it.
    byte_time: Option<Duration>,
    /// Bytes appear only this often.
    burst: Option<Duration>,
    fail_writes_from: Option<usize>,
    /// A memory read's write after the first takes this long: a port with no room.
    write_delay: Duration,
    /// Reads fail once this many reads have found bytes.
    fail_reads_after: Option<usize>,
    bytes_read: usize,
    empty_reads: usize,
    open: bool,
    after_close_writes: usize,
}

/// A byte of what comes at `at`, after what is on its way.
fn queue(s: &mut Scope, bytes: &[u8]) {
    let mut at = s.input.back().and_then(|(t, _)| *t).unwrap_or_else(Instant::now);
    if s.byte_time.is_none() && s.burst.is_none() {
        s.input.extend(bytes.iter().map(|b| (None, *b)));
        return;
    }
    for b in bytes {
        if let Some(t) = s.byte_time {
            at += t;
        }
        s.input.push_back((Some(at), *b));
    }
}

struct Fake(Arc<Mutex<Scope>>);

impl Port for Fake {
    fn name(&self) -> &'static str {
        "fake"
    }

    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }

    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let until = Instant::now() + timeout;
        loop {
            {
                let mut s = self.0.lock().unwrap();
                if let Some((at, stall)) = s.stall_at
                    && s.bytes_read == at
                {
                    s.stall_at = None;
                    s.in_stall = true;
                    drop(s);
                    std::thread::sleep(stall);
                    self.0.lock().unwrap().in_stall = false;
                    return Ok(0);
                }
                if s.fail_reads_after.is_some_and(|n| s.bytes_read >= n) {
                    return Err(Error::device("read failed"));
                }
                let now = Instant::now();
                // A burst's start, by the clock.
                let burst_open =
                    s.burst.is_none_or(|b| u128::from(now_millis()) % b.as_millis() < 5);
                let held = s.hold_after.is_some_and(|at| s.bytes_read >= at);
                let mut n = 0;
                while n < buf.len() && burst_open && !held {
                    match s.input.front() {
                        Some((at, b))
                            if at.is_none_or(|t| t <= now)
                                && s.hold_after.is_none_or(|at| s.bytes_read + n < at) =>
                        {
                            buf[n] = *b;
                            n += 1;
                            s.input.pop_front();
                        }
                        _ => break,
                    }
                }
                if n > 0 {
                    s.bytes_read += n;
                    return Ok(n);
                }
                s.empty_reads += 1;
            }
            if Instant::now() >= until {
                return Ok(0);
            }
            std::thread::sleep(Duration::from_micros(200));
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        if data.ends_with(b":MEM?\n") {
            // The second memory read on: before the first block there is no limit.
            let delay = {
                let s = self.0.lock().unwrap();
                if s.block_at >= 1 { s.write_delay } else { Duration::ZERO }
            };
            std::thread::sleep(delay);
        }
        let mut s = self.0.lock().unwrap();
        if !s.open {
            s.after_close_writes += 1;
        }
        let n = s.writes.len();
        if s.fail_writes_from.is_some_and(|from| n >= from) {
            return Err(Error::device("write failed"));
        }
        s.writes.push((Instant::now(), data.to_vec()));
        let text = String::from_utf8_lossy(data).trim().to_owned();
        if text.ends_with(":MEM?") {
            let at = s.block_at.min(s.blocks.len().saturating_sub(1));
            let silent = s.silent_from.is_some_and(|from| s.block_at >= from);
            let block =
                if silent { Vec::new() } else { s.blocks.get(at).cloned().unwrap_or_default() };
            s.block_at += 1;
            queue(&mut s, &block);
        } else if text.ends_with(":SCAL?") {
            let scale = s.scale.clone();
            queue(&mut s, &scale);
        }
        Ok(())
    }

    fn purge(&mut self) -> Result<()> {
        self.0.lock().unwrap().input.clear();
        Ok(())
    }
}

fn now_millis() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64)
}

/// A block: `points` as they stand, for channel `ch`.
fn block_of(points: &[u8], ch: u8) -> Vec<u8> {
    let size = points.len() + FIELDS;
    let mut out = format!("#{}{size}", size.to_string().len()).into_bytes();
    out.extend_from_slice(&1e8f32.to_bits().to_be_bytes());
    out.extend_from_slice(&[ch, 0, 0, 0]);
    out.extend_from_slice(points);
    out
}

/// A record of `n` points in the manual's sizes: 500 at least, as `#` wants 4 digits.
fn ramp(n: usize) -> Vec<u8> {
    (0..n).flat_map(|i| ((i % 200) as i16 - 100).to_be_bytes()).collect()
}

fn fast_times() -> Times {
    Times {
        read: Duration::from_millis(2),
        answer: Duration::from_millis(100),
        warn: Duration::from_secs(1000),
        limit: Duration::from_secs(1000),
        quiet: Duration::from_millis(5),
        leftover: Duration::from_millis(200),
    }
}

fn scope(setup: impl FnOnce(&mut Scope)) -> (GdsDevice, Arc<Mutex<Scope>>) {
    let mut s = Scope { scale: VOLT.as_bytes().to_vec(), open: true, ..Scope::default() };
    setup(&mut s);
    let seen = Arc::new(Mutex::new(s));
    let scpi = Scpi::new(Box::new(Serial::from_port(Box::new(Fake(Arc::clone(&seen))))));
    let id = HwInfo {
        manufacturer: "GW".to_owned(),
        model: "GDS-820".to_owned(),
        serial_number: "P920130".to_owned(),
        firmware_version: "V.2.03".to_owned(),
    };
    let mut device = GdsDevice::new(&id, scpi);
    device.times = fast_times();
    device.open().unwrap();
    (device, seen)
}

type Rx = Receiver<(usize, Packet)>;

fn start(d: &mut GdsDevice) -> Rx {
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    rx
}

/// Up to the `End`, with a count cap and a deadline, so a limit ignored fails.
fn to_end(rx: &Rx) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((_, Packet::End)) => return out,
            Ok((_, p)) => out.push(p),
            Err(e) => panic!("no End: {e}, after {} packets", out.len()),
        }
        assert!(out.len() < 10_000, "a limit was ignored");
    }
}

/// A short name per packet.
fn shape(packets: &[Packet]) -> Vec<String> {
    packets
        .iter()
        .map(|p| match p {
            Packet::Header { .. } => "H".to_owned(),
            Packet::FrameBegin => "[".to_owned(),
            Packet::FrameEnd => "]".to_owned(),
            Packet::Analog(a) => format!("{:?}x{}", a.channels, a.samples.len()),
            Packet::Failed(_) => "F".to_owned(),
            other => format!("{other:?}"),
        })
        .collect()
}

fn failure(packets: &[Packet]) -> Option<String> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(e) => Some(e.clone()),
        _ => None,
    })
}

fn sent(seen: &Arc<Mutex<Scope>>) -> Vec<String> {
    let s = seen.lock().unwrap();
    s.writes.iter().map(|(_, w)| String::from_utf8_lossy(w).trim().to_owned()).collect()
}

fn samples(packets: &[Packet]) -> Vec<f32> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some(a.samples.clone()),
            _ => None,
        })
        .flatten()
        .collect()
}

fn capture(d: &mut GdsDevice) -> Vec<Packet> {
    let rx = start(d);
    let packets = to_end(&rx);
    d.stop().unwrap();
    packets
}

/// A capture with what a stop must end stopped: past `wait` with no End, it stops it.
fn capture_or_stop(d: &mut GdsDevice, wait: Duration) -> Vec<Packet> {
    let rx = start(d);
    let until = Instant::now() + wait;
    let mut packets = Vec::new();
    while Instant::now() < until {
        match rx.recv_timeout(Duration::from_millis(5)) {
            Ok((_, Packet::End)) => {
                d.stop().unwrap();
                return packets;
            }
            Ok((_, p)) => packets.push(p),
            Err(_) => {}
        }
    }
    d.stop().unwrap();
    packets.extend(to_end(&rx));
    packets
}

/// One capture of one channel from `block`, its output and what went on. What keeps
/// waiting for a first `#` is stopped, which only a damaged `#` should need.
fn capture_block(block: Vec<u8>) -> (Vec<Packet>, Arc<Mutex<Scope>>) {
    let (mut d, seen) = scope(|s| s.blocks = vec![block]);
    d.enable_channel(1, false).unwrap();
    d.times.limit = Duration::from_millis(100);
    (capture_or_stop(&mut d, Duration::from_secs(3)), seen)
}

/// Waits `how_long` by the clock, in steps of a millisecond, so a sleep shim that scales
/// sleeps scales only the steps: a pause that must stay under a limit stays so.
fn pause(how_long: Duration) {
    let until = Instant::now() + how_long;
    while Instant::now() < until {
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// The capture is over: its thread gave the instrument back. A wait for something it
/// should do fails at once then, rather than at its bound.
fn ended(d: &GdsDevice) -> bool {
    d.scpi.lock().unwrap().is_some()
}

/// What a device warned of.
fn warned(d: &GdsDevice) -> Vec<String> {
    d.shared.warned.lock().unwrap().clone()
}

fn a_frame_goes_as_c_sends_it_body() {
    let (mut d, seen) =
        scope(|s| s.blocks = vec![block_of(&ramp(500), 1), block_of(&ramp(500), 2)]);
    let packets = capture(&mut d);
    assert_eq!(shape(&packets), ["H", "[", "[0]x500", "[1]x500", "]"]);
    assert_eq!(
        sent(&seen),
        [
            ":TRIG:MOD 3",
            ":STOP",
            ":RUN",
            ":ACQ1:MEM?",
            ":CHAN1:SCAL?",
            ":ACQ2:MEM?",
            ":CHAN2:SCAL?"
        ]
    );
    let Packet::Analog(a) = &packets[2] else { panic!() };
    assert_eq!(a.samples[0], -100.0 * (10.0f32 / 256.0));
    assert_eq!(a.digits, 2);
    assert_eq!(a.unit, Unit::Volt);
    assert_eq!(a.mq, Mq::Voltage);
    assert_eq!(a.flags, MqFlags::empty());
}

fn a_limit_of_frames_counts_each_frame_and_arms_each_body() {
    let (mut d, seen) = scope(|s| s.blocks = vec![block_of(&ramp(500), 1)]);
    d.enable_channel(1, false).unwrap();
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    assert_eq!(d.get(&Key::LimitFrames, None).unwrap(), Value::U64(2));
    let packets = capture(&mut d);
    assert_eq!(shape(&packets), ["H", "[", "[0]x500", "]", "[", "[0]x500", "]"]);
    assert_eq!(sent(&seen).iter().filter(|w| *w == ":RUN").count(), 2);
}

fn only_the_second_channel_is_read_when_the_first_is_off_body() {
    let (mut d, seen) = scope(|s| s.blocks = vec![block_of(&ramp(500), 2)]);
    d.enable_channel(0, false).unwrap();
    assert_eq!(shape(&capture(&mut d)), ["H", "[", "[1]x500", "]"]);
    assert_eq!(sent(&seen), [":TRIG:MOD 3", ":STOP", ":RUN", ":ACQ2:MEM?", ":CHAN2:SCAL?"]);
}

fn no_channel_refuses_the_start_body() {
    let (mut d, seen) = scope(|_| {});
    d.enable_channel(0, false).unwrap();
    d.enable_channel(1, false).unwrap();
    let (tx, _rx) = sync_channel(4);
    assert!(d.start(PacketSink::new(tx, 0)).is_err());
    assert_eq!(sent(&seen), Vec::<String>::new());
    d.enable_channel(0, true).unwrap();
    let (tx, _rx) = sync_channel(4);
    d.start(PacketSink::new(tx, 0)).expect("the instrument came back");
}

fn a_limit_of_zero_goes_on_until_stopped_body() {
    let (mut d, seen) = scope(|s| s.blocks = vec![block_of(&ramp(500), 1)]);
    d.enable_channel(1, false).unwrap();
    d.set(&Key::LimitFrames, &Value::U64(0), None).unwrap();
    let rx = start(&mut d);
    let mut analog = 0;
    let deadline = Instant::now() + BOUND;
    while analog < 3 {
        assert!(Instant::now() < deadline, "no frames");
        if let Ok((_, Packet::Analog(_))) = rx.recv_timeout(BOUND) {
            analog += 1;
        }
    }
    d.stop().unwrap();
    assert!(sent(&seen).iter().filter(|w| *w == ":RUN").count() >= 3);
}

fn a_limit_set_on_a_running_capture_counts_body() {
    let (mut d, _seen) = scope(|s| {
        s.byte_time = Some(Duration::from_micros(50));
        s.blocks = vec![block_of(&ramp(500), 1)];
    });
    d.enable_channel(1, false).unwrap();
    d.set(&Key::LimitFrames, &Value::U64(0), None).unwrap();
    let rx = start(&mut d);
    while !matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)) {}
    // Frame 0 or frame 1 is under way, and 1 is the last of 2 whichever it is.
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    let rest = to_end(&rx);
    assert_eq!(failure(&rest), None);
    d.stop().unwrap();
}

fn the_rate_is_there_after_a_capture_body() {
    let (mut d, _seen) = scope(|s| s.blocks = vec![block_of(&ramp(500), 1)]);
    d.enable_channel(1, false).unwrap();
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(0), "none yet");
    capture(&mut d);
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(100_000_000));
}

fn a_line_feed_in_the_points_is_a_point_body() {
    // A point of 0x000A, and one at a read's end for sure, and the bytes C's serial
    // layer cuts: an LF and a CR, here 0x0A0D.
    let mut points = ramp(500);
    points[10..12].copy_from_slice(&[0x00, 0x0a]);
    points[20..22].copy_from_slice(&[0x0a, 0x0d]);
    let last = points.len();
    points[last - 2..].copy_from_slice(&[0x00, 0x0a]);
    let (packets, _) = capture_block(block_of(&points, 1));
    assert_eq!(failure(&packets), None);
    let values = samples(&packets);
    assert_eq!(values.len(), 500);
    let vbit = 10.0f32 / 256.0;
    assert_eq!(values[5], 10.0 * vbit);
    assert_eq!(values[10], f32::from(0x0a0du16 as i16) * vbit);
    assert_eq!(values[499], 10.0 * vbit);
}

fn a_line_feed_behind_the_block_is_no_scale_body() {
    // The manual's words: an LF behind each answer. C reads it as its scale, and 0 V.
    let mut block = block_of(&ramp(500), 1);
    block.push(b'\n');
    let (packets, _) = capture_block(block);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets)[1], -99.0 * (10.0f32 / 256.0));
}

fn an_empty_answer_and_nothing_more_is_no_answer_body() {
    let (mut d, _) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.scale = b"\n".to_vec();
    });
    d.enable_channel(1, false).unwrap();
    let packets = capture(&mut d);
    assert_eq!(failure(&packets).unwrap(), "device error: No answer to :CHAN1:SCAL?.");
    assert_eq!(samples(&packets), Vec::<f32>::new());
    assert_eq!(shape(&packets).last().map(String::as_str), Some("F"));
}

#[test]
fn the_scales_forms_and_what_is_no_scale() {
    assert_eq!(parse_scale("1.00V"), Some(1.0));
    assert_eq!(parse_scale("1v"), Some(1.0));
    assert_eq!(parse_scale("500mV"), Some((500.0f64 * 1e-3) as f32));
    assert_eq!(parse_scale("2.000E-03"), Some(2.000e-3));
    assert_eq!(parse_scale("5.000e+00"), Some(5.0));
    assert_eq!(parse_scale(" 5 V\r"), Some(5.0));
    assert_eq!(parse_scale("500 mV"), Some((500.0f64 * 1e-3) as f32));
    assert_eq!(parse_scale("+.5"), Some(0.5));
    for bad in [
        "", "V", "mV", "abc", "0", "0V", "-1V", "1kV", "1mv", "1uV", "nan", "inf", "0x10", "1e",
        "1e400", "1,5V", "1.5.5", "e5", ".", "+",
    ] {
        assert_eq!(parse_scale(bad), None, "{bad}");
    }
}

#[test]
fn the_scales_arithmetic_is_cs() {
    // `(double)(float)500 * 1e-3` to float, against a plain 0.5.
    assert_eq!(parse_scale("3mV"), Some((3.0f64 * 1e-3) as f32));
    assert_eq!(parse_scale("5mV"), Some(0.005), "not 5 * 1e-3f32, 0.005000000354");
    assert_ne!(parse_scale("5mV"), Some(5.0 * 1e-3f32));
    let (vbit, digits) = volts_per_count(1.0);
    assert_eq!(vbit, 0.039_062_5);
    assert_eq!(digits, 2, "log10 is -1.4, -(-1) + 1");
    assert_eq!(volts_per_count(0.002).1, 5);
    assert_eq!(volts_per_count(0.01).1, 4);
    assert_eq!(volts_per_count(5.0).1, 1);
    // 1 V a count exactly: a log of 0 is not below 0.
    assert_eq!(volts_per_count(25.6).1, 0);
    assert_eq!(volts_per_count(2560.0).1, -2);
    assert_eq!(to_volts(&[0x80, 0x00, 0x7f, 0xff, 0x00], 1.0), [-32768.0, 32767.0], "odd byte");
}

#[test]
fn a_size_is_digits_that_fit() {
    assert_eq!(parse_size(b"1008").unwrap(), 1008);
    assert_eq!(parse_size(b"000008").unwrap(), 8);
    assert_eq!(parse_size(b"250008").unwrap(), 250_008);
    assert_eq!(
        parse_size(b"250009").unwrap_err(),
        "Data size 250009 is out of range: 8 to 250008."
    );
    assert_eq!(parse_size(b"0007").unwrap_err(), "Data size 7 is out of range: 8 to 250008.");
    for bad in [&b"+100"[..], b" 100", b"-100", b"10a8", b"1e03"] {
        let text = parse_size(bad).unwrap_err();
        assert!(text.starts_with("Could not parse data size '"), "{text}");
    }
}

#[test]
fn firmware_before_2_03_is_told_apart() {
    for old in ["V.2.01", "V.1.99", "v.2.02", "2.0"] {
        assert!(firmware_lacks_memory_read(old), "{old}");
    }
    for new in ["V.2.03", "V.2.10", "V.3.00", "3.1", "", "unknown", "V.x.y"] {
        assert!(!firmware_lacks_memory_read(new), "{new}");
    }
}

#[test]
fn who_a_gds_800_is() {
    let id = |m: &str, model: &str| HwInfo {
        manufacturer: m.to_owned(),
        model: model.to_owned(),
        ..HwInfo::default()
    };
    assert!(is_gds_800(&id("GW", "GDS-820")));
    assert!(is_gds_800(&id("GW", "GDS-840C")));
    assert!(!is_gds_800(&id("GW", "GDS-2204")));
    assert!(!is_gds_800(&id("Rigol", "GDS-820")));
    assert!(!is_gds_800(&id("gw", "GDS-820")), "as C's strcmp");
}

#[test]
fn silence_warns_once_then_gives_up_and_a_word_clears_it() {
    let t0 = Instant::now();
    let warn = Duration::from_secs(30);
    let limit = Duration::from_secs(600);
    let mut s = Silence::new(t0, warn, limit);
    s.block_began();
    assert_eq!(s.after_empty_read(t0 + Duration::from_secs(29)), Verdict::Wait);
    assert_eq!(s.after_empty_read(t0 + Duration::from_secs(30)), Verdict::Warn);
    assert_eq!(s.after_empty_read(t0 + Duration::from_secs(31)), Verdict::Wait, "said once");
    assert_eq!(s.after_empty_read(t0 + Duration::from_secs(599)), Verdict::Wait);
    assert_eq!(s.after_empty_read(t0 + Duration::from_secs(600)), Verdict::GiveUp);
    assert_eq!(s.give_up(), "No answer from the scope in 600 s.");
    assert_eq!(s.warning(), "No answer from the scope for 30 s, still waiting.");
    let t1 = t0 + Duration::from_secs(40);
    assert!(s.heard(t1), "a warning stood");
    assert!(!s.heard(t1), "said once");
    assert_eq!(s.after_empty_read(t1 + Duration::from_secs(29)), Verdict::Wait);
    assert_eq!(s.after_empty_read(t1 + Duration::from_secs(30)), Verdict::Warn, "again");
}

#[test]
fn before_a_first_block_silence_has_no_limit() {
    let t0 = Instant::now();
    let warn = Duration::from_secs(30);
    let limit = Duration::from_secs(600);
    let mut s = Silence::new(t0, warn, limit);
    assert_eq!(s.after_empty_read(t0 + warn), Verdict::Warn);
    assert_eq!(s.after_empty_read(t0 + limit), Verdict::Wait, "at the limit");
    assert_eq!(s.after_empty_read(t0 + limit * 1000), Verdict::Wait, "a week on");
    assert_eq!(s.after_junk(t0 + limit * 1000), Verdict::Wait, "a stray byte a week on");
    assert_eq!(
        s.warning(),
        "No answer from the scope for 30 s, still waiting: a single trigger may not have \
         come. A stop ends this."
    );
    // The first `#`, however late, and the limit runs from the next thing heard.
    s.block_began();
    let t1 = t0 + limit * 1000;
    assert!(s.heard(t1), "it was warned of");
    assert_eq!(s.after_empty_read(t1 + limit), Verdict::GiveUp);
    // Junk's count has no first block to wait for.
    let mut s = Silence::new(t0, warn, limit);
    for _ in 0..JUNK_LIMIT - 1 {
        assert_eq!(s.after_junk(t0), Verdict::Wait);
    }
    assert_eq!(s.after_junk(t0), Verdict::GiveUp);
}

#[test]
fn the_silence_constants_are_what_the_docs_say() {
    assert_eq!(SILENCE_WARN, Duration::from_secs(30));
    assert_eq!(SILENCE_LIMIT, Duration::from_secs(600));
    assert_eq!(JUNK_LIMIT, 4096);
    assert_eq!(TIMES.limit, Duration::from_secs(600));
    assert_eq!(TIMES.warn, Duration::from_secs(30));
    assert_eq!(TIMES.answer, Duration::from_secs(1));
    assert_eq!(TIMES.read, Duration::from_millis(50));
}

#[test]
fn junk_ends_a_wait_at_its_count_and_never_resets_the_silence() {
    let t0 = Instant::now();
    let mut s = Silence::new(t0, Duration::from_secs(30), Duration::from_secs(600));
    for _ in 0..JUNK_LIMIT - 1 {
        assert_eq!(s.after_junk(t0), Verdict::Wait);
    }
    assert_eq!(s.after_junk(t0), Verdict::GiveUp);
    assert_eq!(s.give_up(), "No usable answer: 4096 bytes and no block.");

    // A trickle of a few bytes does not hold the limit off, once it has one.
    let mut s = Silence::new(t0, Duration::from_secs(30), Duration::from_secs(600));
    s.block_began();
    assert_eq!(s.after_junk(t0 + Duration::from_secs(599)), Verdict::Wait);
    assert_eq!(s.after_empty_read(t0 + Duration::from_secs(600)), Verdict::GiveUp);
    let mut s = Silence::new(t0, Duration::from_secs(30), Duration::from_secs(600));
    s.block_began();
    assert_eq!(s.after_junk(t0 + Duration::from_secs(600)), Verdict::GiveUp);
}

fn a_silent_scope_warns_and_waits_on_past_any_limit_for_a_stop_body() {
    let (mut d, seen) = scope(|_| {});
    d.enable_channel(1, false).unwrap();
    d.times.warn = Duration::from_millis(5);
    d.times.limit = Duration::from_millis(10);
    let rx = start(&mut d);
    // Well past both, by empty reads and not by a sleep alone.
    let wait = Instant::now() + BOUND;
    while seen.lock().unwrap().empty_reads < 200 {
        assert!(Instant::now() < wait && !ended(&d), "no reads");
        std::thread::sleep(Duration::from_millis(1));
    }
    pause(Duration::from_millis(50));
    assert!(rx.try_recv().is_err(), "a first block has no limit");
    assert_eq!(
        warned(&d),
        ["No answer from the scope for 0.005 s, still waiting: a single trigger may not have \
          come. A stop ends this."],
        "and one warning"
    );
    d.stop().unwrap();
    assert_eq!(shape(&to_end(&rx)), Vec::<String>::new(), "a stop is no failure, and no header");
    assert_eq!(sent(&seen).last().map(String::as_str), Some(":ACQ1:MEM?"));
}

fn a_first_block_that_comes_late_says_so_and_is_a_block_body() {
    let (mut d, seen) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.hold_after = Some(0);
    });
    d.enable_channel(1, false).unwrap();
    d.times.warn = Duration::from_millis(5);
    d.times.limit = Duration::from_millis(10);
    let rx = start(&mut d);
    let wait = Instant::now() + BOUND;
    while warned(&d).is_empty() {
        assert!(Instant::now() < wait && !ended(&d), "no warning");
        std::thread::sleep(Duration::from_millis(1));
    }
    pause(Duration::from_millis(30));
    d.times.limit = Duration::from_secs(1000);
    // The limit is the thread's copy, so the rest must come with no pause: all at once.
    seen.lock().unwrap().hold_after = None;
    let packets = to_end(&rx);
    d.stop().unwrap();
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);
    assert_eq!(warned(&d).last().map(String::as_str), Some("The scope is answering again."));
}

fn after_a_sample_silence_has_its_limit_body() {
    let (mut d, _) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.silent_from = Some(1);
    });
    d.times.limit = Duration::from_millis(40);
    let packets = capture(&mut d);
    assert_eq!(shape(&packets), ["H", "[", "[0]x500", "]", "F"]);
    assert_eq!(failure(&packets).unwrap(), "device error: No answer from the scope in 0.04 s.");
}

/// A stop returns once the thread is done: it is caught inside a read that only ends
/// later, and the instrument must be back when the stop returns.
fn a_stop_joins_the_thread_body() {
    let (mut d, seen) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.stall_at = Some((50, Duration::from_millis(300)));
    });
    d.enable_channel(1, false).unwrap();
    let rx = start(&mut d);
    let wait = Instant::now() + BOUND;
    while !seen.lock().unwrap().in_stall {
        assert!(Instant::now() < wait && !ended(&d), "no stall");
        std::thread::sleep(Duration::from_millis(1));
    }
    d.stop().unwrap();
    assert!(d.scpi.lock().unwrap().is_some(), "the stop returned with the thread still in a read");
    assert!(!seen.lock().unwrap().in_stall);
    drop(rx);
}

fn a_stall_inside_a_read_is_no_silence_body() {
    let (mut d, _) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.stall_at = Some((100, Duration::from_millis(300)));
    });
    d.enable_channel(1, false).unwrap();
    d.times.limit = Duration::from_millis(200);
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);
}

fn silence_in_a_block_runs_from_its_last_byte_body() {
    // Four pauses, each under the limit and together over it.
    let (mut d, seen) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.hold_after = Some(20);
    });
    d.enable_channel(1, false).unwrap();
    d.times.limit = Duration::from_secs(2);
    let rx = start(&mut d);
    for at in [40, 60, 80, 0] {
        let wait = Instant::now() + BOUND * 6;
        loop {
            assert!(Instant::now() < wait && !ended(&d), "no bytes");
            let s = seen.lock().unwrap();
            if s.hold_after.is_some_and(|h| s.bytes_read >= h) {
                break;
            }
            drop(s);
            std::thread::sleep(Duration::from_millis(1));
        }
        pause(Duration::from_millis(800));
        seen.lock().unwrap().hold_after = (at > 0).then_some(at);
    }
    let packets = to_end(&rx);
    d.stop().unwrap();
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);
}

fn silence_starts_again_at_a_blocks_start_body() {
    // A first `#` long after the limit, then a pause: the time ahead of it doesn't count.
    let (mut d, seen) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.hold_after = Some(0);
    });
    d.enable_channel(1, false).unwrap();
    d.times.limit = Duration::from_secs(1);
    let rx = start(&mut d);
    pause(Duration::from_millis(1500));
    seen.lock().unwrap().hold_after = Some(1);
    let wait = Instant::now() + BOUND * 6;
    while seen.lock().unwrap().bytes_read < 1 {
        assert!(Instant::now() < wait && !ended(&d), "no #");
        std::thread::sleep(Duration::from_millis(1));
    }
    pause(Duration::from_millis(100));
    seen.lock().unwrap().hold_after = None;
    let packets = to_end(&rx);
    d.stop().unwrap();
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);
}

fn old_firmware_warns_and_still_asks_body() {
    let (mut d, seen) = scope(|s| s.blocks = vec![block_of(&ramp(500), 1)]);
    "V.2.01".clone_into(&mut d.info.version);
    d.enable_channel(1, false).unwrap();
    let packets = capture(&mut d);
    assert_eq!(samples(&packets).len(), 500, "a warning, no refusal");
    assert!(sent(&seen).iter().any(|w| w == ":ACQ1:MEM?"));
    assert_eq!(
        warned(&d),
        ["Firmware V.2.01 is older than 2.03, and the manual has :ACQ<X>:MEM? from then on."]
    );
}

fn other_warnings_are_said_body() {
    let (mut d, _) = scope(|s| s.blocks = vec![block_of(&ramp(500), 2)]);
    d.enable_channel(1, false).unwrap();
    capture(&mut d);
    assert_eq!(warned(&d), ["The block read for channel 1 says it is of channel 2."]);

    let (mut d, _) = scope(|s| s.blocks = vec![block_of(&ramp(499), 1)]);
    d.enable_channel(1, false).unwrap();
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    capture(&mut d);
    assert_eq!(warned(&d), ["A block of 1006 bytes is no record length in the manual."], "once");
}

#[test]
fn the_manuals_steps() {
    for good in [
        0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0,
        200.0, 500.0,
    ] {
        assert!(on_the_steps(good), "{good}");
        // As they come from the answers, in single precision.
        assert!(on_the_steps(parse_scale(&format!("{good}V")).unwrap()), "{good} V");
    }
    assert!(on_the_steps(parse_scale("5mV").unwrap()));
    assert!(on_the_steps(parse_scale("2.000E-03").unwrap()));
    for bad in [0.001, 0.003, 0.0019, 1.1, 1.5, 3.0, 4.0, 7.0, 25.0, 1000.0, 5000.0] {
        assert!(!on_the_steps(bad), "{bad}");
    }
}

fn a_scale_off_the_steps_warns_once_and_is_used_body() {
    let (mut d, _) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.scale = b"3.00V\n".to_vec();
    });
    d.enable_channel(1, false).unwrap();
    d.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None, "a warning, no refusal");
    assert_eq!(samples(&packets)[1], -99.0 * (3.0 * 10.0f32 / 256.0));
    assert_eq!(
        warned(&d),
        [":CHAN1:SCAL? said '3.00V', which is no step in the manual: the volts may be wrong."]
    );
}

/// A capture with `answer` for the scale: wrong, same or failed against a good one.
fn scale_outcome(answer: &[u8], good: &[f32]) -> (&'static str, bool) {
    let (mut d, _) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.scale = answer.to_vec();
    });
    d.enable_channel(1, false).unwrap();
    d.times.answer = Duration::from_millis(20);
    let packets = capture(&mut d);
    let warned = warned(&d).iter().any(|w| w.contains("no step in the manual"));
    if failure(&packets).is_some() {
        ("failed", warned)
    } else if samples(&packets) == good {
        ("same", warned)
    } else {
        ("wrong", warned)
    }
}

/// The field that can give wrong readings: all of a channel, by one flip, at exit 0.
fn every_flip_and_lost_byte_of_a_scale_body() {
    // (answer, flips wrong, same, failed, of the wrong how many warn; lost bytes wrong)
    #[allow(clippy::type_complexity)]
    let table: [(&[u8], usize, usize, usize, usize, usize); 3] = [
        (b"1.00V\n", 11, 2, 35, FLIPS_WARNED_V, 1),
        (b"500mV\n", 12, 0, 36, FLIPS_WARNED_MV, 3),
        (b"2.000E-03\n", 20, 1, 59, FLIPS_WARNED_NR3, 3),
    ];
    for (answer, wrong, same, failed, warn, lost) in table {
        let (mut d, _) = scope(|s| {
            s.blocks = vec![block_of(&ramp(500), 1)];
            s.scale = answer.to_vec();
        });
        d.enable_channel(1, false).unwrap();
        let good = samples(&capture(&mut d));
        assert_eq!(good.len(), 500);
        let mut count = [0usize; 3];
        let mut warned_wrong = 0;
        for byte in 0..answer.len() {
            for bit in 0..8 {
                let mut damaged = answer.to_vec();
                damaged[byte] ^= 1 << bit;
                let (outcome, warned) = scale_outcome(&damaged, &good);
                match outcome {
                    "wrong" => {
                        count[0] += 1;
                        warned_wrong += usize::from(warned);
                    }
                    "same" => count[1] += 1,
                    _ => count[2] += 1,
                }
            }
        }
        let name = String::from_utf8_lossy(answer);
        assert_eq!(count, [wrong, same, failed], "{name} flips: wrong, same, failed");
        assert_eq!(warned_wrong, warn, "{name}: of the wrong ones, those that warn");
        let (mut lost_wrong, mut lost_warned) = (0, 0);
        for byte in 0..answer.len() {
            let mut damaged = answer.to_vec();
            damaged.remove(byte);
            let (outcome, warned) = scale_outcome(&damaged, &good);
            lost_wrong += usize::from(outcome == "wrong");
            lost_warned += usize::from(outcome == "wrong" && warned);
        }
        assert_eq!(lost_wrong, lost, "{name} lost bytes that are wrong");
        // Only 2.000E03, 2 kV, is off the steps.
        assert_eq!(lost_warned, usize::from(answer.len() == 10), "{name} lost bytes that warn");
    }
}

fn a_block_cut_short_fails_with_its_frame_closed_body() {
    let points = ramp(500);
    let mut block = block_of(&points, 1);
    block.truncate(block.len() - 100);
    let (packets, _) = capture_block(block);
    assert_eq!(shape(&packets), ["H", "[", "]", "F"]);
    assert!(failure(&packets).unwrap().contains("No answer from the scope"));
}

fn a_late_block_is_a_block_body() {
    let (mut d, seen) = scope(|s| s.blocks = vec![block_of(&ramp(500), 1)]);
    d.enable_channel(1, false).unwrap();
    d.times.limit = Duration::from_secs(1000);
    // A trigger that comes: nothing on the line for a while, well past a read's wait.
    let hold = Arc::clone(&seen);
    {
        let mut s = hold.lock().unwrap();
        s.byte_time = Some(Duration::from_micros(1));
        let pause = Instant::now() + Duration::from_millis(30);
        s.input.push_back((Some(pause), b'x'));
    }
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);
}

fn junk_ahead_of_a_block_is_passed_over_and_a_flood_is_not_body() {
    let mut block = b"stale answer\n".to_vec();
    block.extend_from_slice(&block_of(&ramp(500), 1));
    let (packets, _) = capture_block(block);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);

    let (packets, _) = capture_block(vec![b'x'; 2 * JUNK_LIMIT]);
    assert_eq!(shape(&packets), ["F"]);
    assert_eq!(
        failure(&packets).unwrap(),
        "device error: No usable answer: 4096 bytes and no block."
    );
}

fn a_bad_head_fails_as_c_says_or_says_why_body() {
    let cases: [(&[u8], &str); 5] = [
        (b"#3100", "device error: Data size digits is not 4, 5 or 6 but '3'."),
        (b"#x100", "device error: Data size digits is not 4, 5 or 6 but 'x'."),
        (b"#41a08", "device error: Could not parse data size '1a08'"),
        (b"#40004", "device error: Data size 4 is out of range: 8 to 250008."),
        (b"#6999999", "device error: Data size 999999 is out of range: 8 to 250008."),
    ];
    for (head, words) in cases {
        let (packets, _) = capture_block(head.to_vec());
        assert_eq!(shape(&packets), ["F"], "no header before the size: {words}");
        assert_eq!(failure(&packets).unwrap(), words);
    }
}

fn a_block_of_the_most_points_passes_and_no_point_is_a_frame_with_none_body() {
    let (packets, _) = capture_block(block_of(&ramp(125_000), 1));
    assert_eq!(samples(&packets).len(), 125_000);
    assert_eq!(failure(&packets), None);
    let mut empty = b"#40008".to_vec();
    empty.extend_from_slice(&[0x4c, 0xbe, 0xbc, 0x20, 1, 0, 0, 0]);
    let (packets, _) = capture_block(empty);
    assert_eq!(failure(&packets), None);
    assert_eq!(shape(&packets), ["H", "[", "[0]x0", "]"]);
    let (packets, _) = capture_block(block_of(&ramp(500)[..999], 1));
    assert_eq!(samples(&packets).len(), 499, "an odd byte is no point, as C");
}

/// The eight fields at each bit, with what the format can catch. No sum, so a flipped
/// bit in a point is a wrong point, as for C; the head is where a check can be.
fn every_single_bit_flip_of_a_head_and_its_fields_body() {
    let points = ramp(500);
    let good_block = block_of(&points, 1);
    let good = samples(&capture_block(good_block.clone()).0);
    let head = good_block.len() - points.len();
    let (mut same, mut failed, mut wrong) = (0, 0, 0);
    for byte in 0..head {
        // The channel, the reserved and the rate make no reading.
        for bit in 0..8 {
            let mut block = good_block.clone();
            block[byte] ^= 1 << bit;
            let (packets, _) = capture_block(block);
            let got = samples(&packets);
            if failure(&packets).is_some() {
                failed += 1;
            } else if got == good {
                same += 1;
            } else {
                wrong += 1;
            }
        }
    }
    // # 4 1 0 0 8: of 48 flips, 24 are the size's digits, and of those only ones giving
    // another digit parse. A size that is too big leaves the block short and fails; one
    // that is too small takes fewer points and meets the next answer in its place, which
    // the scale's check refuses.
    assert_eq!(same + failed + wrong, head * 8);
    assert_eq!((same, failed, wrong), (64, 48, 0), "same {same} failed {failed} wrong {wrong}");
}

/// A byte lost anywhere leaves the block one short, or its head unreadable: never a
/// reading. Here each of the head and fields, and points at each end of the record.
fn every_lost_byte_of_a_head_and_some_points_body() {
    let points = ramp(500);
    let good_block = block_of(&points, 1);
    let head = good_block.len() - points.len();
    let lost = (0..head + 4).chain(good_block.len() - 2..good_block.len());
    let mut outcomes = 0;
    for at in lost {
        let mut block = good_block.clone();
        block.remove(at);
        let (mut d, _) = scope(|s| s.blocks = vec![block]);
        d.enable_channel(1, false).unwrap();
        d.times.limit = Duration::from_millis(10);
        let packets = capture(&mut d);
        assert!(failure(&packets).is_some(), "byte {at} lost: {:?}", shape(&packets));
        assert_eq!(samples(&packets), Vec::<f32>::new(), "byte {at} lost");
        outcomes += 1;
    }
    assert_eq!(outcomes, head + 6);
}

fn a_block_of_another_channel_is_noted_and_kept_body() {
    let (mut d, _) = scope(|s| s.blocks = vec![block_of(&ramp(500), 2)]);
    d.enable_channel(1, false).unwrap();
    assert_eq!(samples(&capture(&mut d)).len(), 500, "as C");
}

fn a_stop_in_a_block_ends_it_with_the_frame_closed_and_the_next_capture_drains_it_body() {
    let (mut d, seen) = scope(|s| {
        s.byte_time = Some(Duration::from_micros(100));
        s.blocks = vec![block_of(&ramp(5000), 1)];
    });
    d.enable_channel(1, false).unwrap();
    let rx = start(&mut d);
    let (_, first) = rx.recv_timeout(BOUND).unwrap();
    assert!(matches!(first, Packet::Header { .. }));
    let began = Instant::now();
    d.stop().unwrap();
    assert!(began.elapsed() < Duration::from_millis(500), "stop looks between bytes");
    let rest = to_end(&rx);
    assert_eq!(shape(&rest), ["[", "]"]);
    let left = seen.lock().unwrap().input.len();
    assert!(left > 0, "the stop came in the block");

    // What is left must not pass for the next block.
    {
        let mut s = seen.lock().unwrap();
        s.byte_time = None;
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.block_at = 0;
        for (at, _) in &mut s.input {
            *at = None;
        }
    }
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);
}

fn a_stop_before_any_block_ends_it_with_no_header_body() {
    let (mut d, _) = scope(|_| {});
    let rx = start(&mut d);
    d.stop().unwrap();
    assert_eq!(to_end(&rx).len(), 0);
}

fn a_close_joins_and_nothing_writes_after_it_body() {
    let (mut d, seen) = scope(|s| s.blocks = vec![block_of(&ramp(500), 1)]);
    d.set(&Key::LimitFrames, &Value::U64(0), None).unwrap();
    let rx = start(&mut d);
    while !matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Analog(_)) {}
    d.close().unwrap();
    let writes = sent(&seen).len();
    seen.lock().unwrap().open = false;
    assert!(d.worker.is_none());
    std::thread::sleep(Duration::from_millis(20));
    assert_eq!(sent(&seen).len(), writes);
    assert_eq!(seen.lock().unwrap().after_close_writes, 0);
    d.open().unwrap();
    assert!(d.scpi.lock().unwrap().is_some(), "and it opens again");
}

fn a_full_queue_does_not_hold_a_stop_or_a_restart_body() {
    let (mut d, seen) = scope(|s| s.blocks = vec![block_of(&ramp(500), 1)]);
    d.set(&Key::LimitFrames, &Value::U64(0), None).unwrap();
    d.enable_channel(1, false).unwrap();
    let (tx, _rx) = sync_channel(1);
    d.start(PacketSink::new(tx, 0)).unwrap();
    // Nobody reads: the header fills the queue and the frame's start waits. The head
    // and the rate come ahead of the header.
    let wait = Instant::now() + BOUND;
    while seen.lock().unwrap().bytes_read < 10 {
        assert!(Instant::now() < wait && !ended(&d), "no block read");
        std::thread::sleep(Duration::from_millis(1));
    }
    std::thread::sleep(Duration::from_millis(20));
    let (done_tx, done_rx) = sync_channel(1);
    std::thread::spawn(move || {
        d.stop().unwrap();
        let _ = done_tx.send(d);
    });
    let mut d = done_rx.recv_timeout(BOUND).expect("a stop held by a full queue");
    let (tx, rx2) = sync_channel(4096);
    d.set(&Key::LimitFrames, &Value::U64(1), None).unwrap();
    d.start(PacketSink::new(tx, 0)).expect("a start after it");
    assert!(samples(&to_end(&rx2)).len() >= 500);
}

fn a_start_while_one_runs_is_busy_body() {
    let (mut d, _) = scope(|s| s.blocks = vec![block_of(&ramp(500), 1)]);
    d.set(&Key::LimitFrames, &Value::U64(0), None).unwrap();
    let _rx = start(&mut d);
    let (tx, _rx2) = sync_channel(4);
    assert!(matches!(d.start(PacketSink::new(tx, 0)), Err(Error::Busy)));
    d.stop().unwrap();
}

fn a_start_after_an_end_nobody_took_does_not_hang_body() {
    let (mut d, _) = scope(|s| s.blocks = vec![block_of(&ramp(500), 1)]);
    d.enable_channel(1, false).unwrap();
    let (tx, rx) = sync_channel(8);
    d.start(PacketSink::new(tx, 0)).unwrap();
    let wait = Instant::now() + BOUND;
    while d.scpi.lock().unwrap().is_none() {
        assert!(Instant::now() < wait);
        std::thread::sleep(Duration::from_millis(1));
    }
    let (tx2, rx2) = sync_channel(4096);
    d.start(PacketSink::new(tx2, 0)).unwrap();
    assert_eq!(samples(&to_end(&rx2)).len(), 500);
    drop(rx);
}

fn nobody_listening_ends_it_quietly_body() {
    let (mut d, seen) = scope(|s| s.blocks = vec![block_of(&ramp(500), 1)]);
    d.set(&Key::LimitFrames, &Value::U64(0), None).unwrap();
    let (tx, rx) = sync_channel(4);
    d.start(PacketSink::new(tx, 0)).unwrap();
    drop(rx);
    let wait = Instant::now() + BOUND;
    while d.scpi.lock().unwrap().is_none() {
        assert!(Instant::now() < wait, "it went on with no one to send to");
        std::thread::sleep(Duration::from_millis(1));
    }
    assert!(sent(&seen).len() < 100);
}

fn a_failed_write_at_each_step_is_a_failed_capture_body() {
    for (from, count) in [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)] {
        let (mut d, seen) = scope(|s| {
            s.blocks = vec![block_of(&ramp(500), 1)];
            s.fail_writes_from = Some(from);
        });
        d.enable_channel(1, false).unwrap();
        let packets = capture(&mut d);
        assert_eq!(failure(&packets).unwrap(), "device error: write failed", "write {from}");
        assert_eq!(sent(&seen).len(), count);
        let closed = shape(&packets).contains(&"]".to_owned());
        assert_eq!(closed, from == 4, "a frame is open only once a block has come");
    }
}

fn a_failed_read_is_a_failed_capture_at_once_body() {
    let (mut d, _) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.fail_reads_after = Some(50);
    });
    d.enable_channel(1, false).unwrap();
    d.times.limit = Duration::from_secs(1000);
    let packets = capture(&mut d);
    assert_eq!(shape(&packets), ["H", "[", "]", "F"]);
    assert_eq!(failure(&packets).unwrap(), "device error: read failed");
}

fn a_scale_that_never_comes_is_a_failed_capture_body() {
    let (mut d, _) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.scale = Vec::new();
    });
    d.enable_channel(1, false).unwrap();
    let packets = capture(&mut d);
    assert_eq!(shape(&packets), ["H", "[", "]", "F"]);
    assert_eq!(failure(&packets).unwrap(), "device error: No answer to :CHAN1:SCAL?.");
}

fn a_scale_that_is_none_is_a_failed_capture_and_no_samples_body() {
    for answer in ["abc\n", "0V\n", "1kV\n"] {
        let (mut d, _) = scope(|s| {
            s.blocks = vec![block_of(&ramp(500), 1)];
            s.scale = answer.as_bytes().to_vec();
        });
        d.enable_channel(1, false).unwrap();
        let packets = capture(&mut d);
        assert_eq!(samples(&packets), Vec::<f32>::new(), "{answer}");
        let word = answer.trim();
        assert_eq!(
            failure(&packets).unwrap(),
            format!("device error: :CHAN1:SCAL? said '{word}', which is no scale.")
        );
    }
}

fn a_real_rate_changes_nothing_body() {
    // 1014 bytes at 38400 baud 8n1, about 0.26 s.
    let (mut d, _) = scope(|s| {
        s.byte_time = Some(Duration::from_micros(260));
        s.blocks = vec![block_of(&ramp(500), 1)];
    });
    d.enable_channel(1, false).unwrap();
    d.times.limit = Duration::from_secs(1000);
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);
}

fn a_burst_every_50_ms_changes_nothing_body() {
    let (mut d, _) = scope(|s| {
        s.burst = Some(Duration::from_millis(50));
        s.blocks = vec![block_of(&ramp(500), 1)];
    });
    d.enable_channel(1, false).unwrap();
    d.times.limit = Duration::from_secs(1000);
    d.times.answer = Duration::from_secs(2);
    let packets = capture(&mut d);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);
}

/// Holds the line after the head and the rate, so what comes next is an empty read.
const HEAD_AND_RATE: usize = 6 + 4;

fn a_slow_client_is_no_silence_body() {
    let (mut d, seen) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.hold_after = Some(HEAD_AND_RATE);
    });
    d.enable_channel(1, false).unwrap();
    // Longer than the test's last pause, at the shim's 12 times too.
    d.times.limit = Duration::from_secs(2);
    let (tx, rx) = sync_channel(1);
    d.start(PacketSink::new(tx, 0)).unwrap();
    // The header fills the queue, and the frame's start waits on it, past the limit.
    pause(Duration::from_millis(2500));
    assert!(matches!(rx.recv_timeout(BOUND).unwrap().1, Packet::Header { .. }));
    assert_eq!(rx.recv_timeout(BOUND).unwrap().1, Packet::FrameBegin);
    pause(Duration::from_millis(100));
    seen.lock().unwrap().hold_after = None;
    let packets = to_end(&rx);
    d.stop().unwrap();
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);
}

fn a_slow_write_is_no_silence_body() {
    // The first channel's block and scale pass; the second's write takes longer than
    // the limit, and the line is held until it is out.
    let first = block_of(&ramp(500), 1).len() + VOLT.len();
    let (mut d, seen) = scope(|s| {
        s.write_delay = Duration::from_millis(2500);
        s.blocks = vec![block_of(&ramp(500), 1), block_of(&ramp(500), 2)];
        s.hold_after = Some(first);
    });
    // Longer than the test's last pause, at the shim's 12 times too.
    d.times.limit = Duration::from_secs(2);
    let rx = start(&mut d);
    let wait = Instant::now() + BOUND * 6;
    while sent(&seen).iter().filter(|w| w.ends_with(":MEM?")).count() < 2 {
        assert!(Instant::now() < wait && !ended(&d), "no second memory read");
        std::thread::sleep(Duration::from_millis(1));
    }
    pause(Duration::from_millis(100));
    seen.lock().unwrap().hold_after = None;
    let packets = to_end(&rx);
    d.stop().unwrap();
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 1000);
}

fn a_scale_has_the_answers_wait_and_not_a_reads_body() {
    let (mut d, seen) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.scale = Vec::new();
    });
    d.enable_channel(1, false).unwrap();
    d.times.answer = Duration::from_secs(5);
    let rx = start(&mut d);
    let wait = Instant::now() + BOUND;
    while !sent(&seen).iter().any(|w| w.ends_with(":SCAL?")) {
        assert!(Instant::now() < wait && !ended(&d), "no scale asked");
        std::thread::sleep(Duration::from_millis(1));
    }
    // Far more than a read's 2 ms.
    pause(Duration::from_millis(100));
    queue(&mut seen.lock().unwrap(), VOLT.as_bytes());
    let packets = to_end(&rx);
    d.stop().unwrap();
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets).len(), 500);
}

fn the_instrument_is_back_before_the_end_body() {
    // The line is held until the test has the slot, or a quick capture ends first.
    let (mut d, seen) = scope(|s| {
        s.blocks = vec![block_of(&ramp(500), 1)];
        s.hold_after = Some(0);
    });
    d.enable_channel(1, false).unwrap();
    let rx = start(&mut d);
    // With the slot held, the End can't come, as the thread puts the instrument back
    // first. Taken after the start, which needs it.
    let slot = Arc::clone(&d.scpi);
    let guard = slot.lock().unwrap();
    seen.lock().unwrap().hold_after = None;
    let wait = Instant::now() + BOUND;
    loop {
        assert!(Instant::now() < wait, "no frame end");
        if rx.recv_timeout(BOUND).unwrap().1 == Packet::FrameEnd {
            break;
        }
    }
    assert!(rx.recv_timeout(Duration::from_millis(150)).is_err(), "an End ahead of the instrument");
    drop(guard);
    assert_eq!(rx.recv_timeout(BOUND).unwrap().1, Packet::End);
    assert!(d.scpi.lock().unwrap().is_some());
    d.stop().unwrap();
}

#[test]
fn settings_and_lists_are_the_two_keys() {
    let (mut d, seen) = scope(|_| {});
    assert_eq!(d.options(None).unwrap().len(), 2);
    assert_eq!(d.options(Some("")).unwrap().len(), 0);
    assert!(d.options(Some("CH1")).is_err());
    assert!(d.get(&Key::Timebase, None).is_err());
    assert!(d.set(&Key::Samplerate, &Value::U64(1), None).is_err());
    assert!(d.list(&Key::LimitFrames, None).is_err());
    assert!(d.set(&Key::LimitFrames, &Value::Str("x".to_owned()), None).is_err());
    assert_eq!(d.get(&Key::LimitFrames, None).unwrap(), Value::U64(1), "1 by default");
    assert_eq!(sent(&seen), Vec::<String>::new(), "no key asks the scope");
}

/// A test that a stop that doesn't bite would hang, on a thread with a bound, so that a
/// mutant fails rather than holds the run.
fn bounded(body: fn()) {
    let (done_tx, done_rx) = sync_channel(1);
    let test = std::thread::spawn(move || {
        body();
        let _ = done_tx.send(());
    });
    assert!(
        done_rx.recv_timeout(BOUND * 4 + BOUND / 2).is_ok() || test.is_finished(),
        "a test hung"
    );
    test.join().unwrap();
}

macro_rules! bounded_tests {
    ($($name:ident: $body:ident,)*) => {
        $(#[test]
        fn $name() {
            bounded($body);
        })*
    };
}

bounded_tests! {
    a_stop_joins_the_thread: a_stop_joins_the_thread_body,
    a_frame_goes_as_c_sends_it: a_frame_goes_as_c_sends_it_body,
    a_limit_of_frames_counts_each_frame_and_arms_each: a_limit_of_frames_counts_each_frame_and_arms_each_body,
    only_the_second_channel_is_read_when_the_first_is_off: only_the_second_channel_is_read_when_the_first_is_off_body,
    the_rate_is_there_after_a_capture: the_rate_is_there_after_a_capture_body,
    a_line_feed_in_the_points_is_a_point: a_line_feed_in_the_points_is_a_point_body,
    a_line_feed_behind_the_block_is_no_scale: a_line_feed_behind_the_block_is_no_scale_body,
    an_empty_answer_and_nothing_more_is_no_answer: an_empty_answer_and_nothing_more_is_no_answer_body,
    a_silent_scope_warns_and_waits_on_past_any_limit_for_a_stop: a_silent_scope_warns_and_waits_on_past_any_limit_for_a_stop_body,
    a_first_block_that_comes_late_says_so_and_is_a_block: a_first_block_that_comes_late_says_so_and_is_a_block_body,
    after_a_sample_silence_has_its_limit: after_a_sample_silence_has_its_limit_body,
    a_stall_inside_a_read_is_no_silence: a_stall_inside_a_read_is_no_silence_body,
    silence_in_a_block_runs_from_its_last_byte: silence_in_a_block_runs_from_its_last_byte_body,
    silence_starts_again_at_a_blocks_start: silence_starts_again_at_a_blocks_start_body,
    old_firmware_warns_and_still_asks: old_firmware_warns_and_still_asks_body,
    other_warnings_are_said: other_warnings_are_said_body,
    a_scale_off_the_steps_warns_once_and_is_used: a_scale_off_the_steps_warns_once_and_is_used_body,
    every_flip_and_lost_byte_of_a_scale: every_flip_and_lost_byte_of_a_scale_body,
    a_block_cut_short_fails_with_its_frame_closed: a_block_cut_short_fails_with_its_frame_closed_body,
    a_late_block_is_a_block: a_late_block_is_a_block_body,
    junk_ahead_of_a_block_is_passed_over_and_a_flood_is_not: junk_ahead_of_a_block_is_passed_over_and_a_flood_is_not_body,
    a_bad_head_fails_as_c_says_or_says_why: a_bad_head_fails_as_c_says_or_says_why_body,
    a_block_of_the_most_points_passes_and_no_point_is_a_frame_with_none: a_block_of_the_most_points_passes_and_no_point_is_a_frame_with_none_body,
    every_single_bit_flip_of_a_head_and_its_fields: every_single_bit_flip_of_a_head_and_its_fields_body,
    every_lost_byte_of_a_head_and_some_points: every_lost_byte_of_a_head_and_some_points_body,
    a_block_of_another_channel_is_noted_and_kept: a_block_of_another_channel_is_noted_and_kept_body,
    a_full_queue_does_not_hold_a_stop_or_a_restart: a_full_queue_does_not_hold_a_stop_or_a_restart_body,
    a_failed_write_at_each_step_is_a_failed_capture: a_failed_write_at_each_step_is_a_failed_capture_body,
    a_failed_read_is_a_failed_capture_at_once: a_failed_read_is_a_failed_capture_at_once_body,
    a_scale_that_never_comes_is_a_failed_capture: a_scale_that_never_comes_is_a_failed_capture_body,
    a_scale_that_is_none_is_a_failed_capture_and_no_samples: a_scale_that_is_none_is_a_failed_capture_and_no_samples_body,
    a_real_rate_changes_nothing: a_real_rate_changes_nothing_body,
    a_burst_every_50_ms_changes_nothing: a_burst_every_50_ms_changes_nothing_body,
    a_slow_client_is_no_silence: a_slow_client_is_no_silence_body,
    a_slow_write_is_no_silence: a_slow_write_is_no_silence_body,
    a_scale_has_the_answers_wait_and_not_a_reads: a_scale_has_the_answers_wait_and_not_a_reads_body,
    the_instrument_is_back_before_the_end: the_instrument_is_back_before_the_end_body,
    a_limit_of_zero_goes_on_until_stopped: a_limit_of_zero_goes_on_until_stopped_body,
    a_stop_in_a_block_ends_it_with_the_frame_closed_and_the_next_capture_drains_it: a_stop_in_a_block_ends_it_with_the_frame_closed_and_the_next_capture_drains_it_body,
    a_stop_before_any_block_ends_it_with_no_header: a_stop_before_any_block_ends_it_with_no_header_body,
    a_close_joins_and_nothing_writes_after_it: a_close_joins_and_nothing_writes_after_it_body,
    a_start_while_one_runs_is_busy: a_start_while_one_runs_is_busy_body,
    a_start_after_an_end_nobody_took_does_not_hang: a_start_after_an_end_nobody_took_does_not_hang_body,
    nobody_listening_ends_it_quietly: nobody_listening_ends_it_quietly_body,
    a_limit_set_on_a_running_capture_counts: a_limit_set_on_a_running_capture_counts_body,
    no_channel_refuses_the_start: no_channel_refuses_the_start_body,
}
