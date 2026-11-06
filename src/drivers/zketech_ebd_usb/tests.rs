// Counterpart in sigrok:
//   libsigrok/src/hardware/zketech-ebd-usb/protocol.c,
//   Copyright (C) 2018 Sven Bursch-Osewold <sb_git@bursch.com>,
//   Copyright (C) 2019 King Kévin <kingkevin@cuvoodoo.info>,
//   with commits by Uwe Hermann, Florian Ragwitz, Frank Stettner, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The capture on a fake [`Port`]: pacing, stops and limits, which an emulator
//! model cannot check.

use super::protocol::tests::{measurement, without_load};
use super::*;
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// How long a test waits before it fails: a stop or a limit that does not bite
/// must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(20);
/// Stands in for 11 ms, so a silent device is quick.
const BYTE_TIMEOUT: Duration = Duration::from_millis(1);

#[derive(Default)]
struct Seen {
    /// Each write, a byte as a rule, with its time.
    writes: crate::serial::WriteLog,
    /// The commands whole, when they end in `f8`.
    commands: Vec<Vec<u8>>,
    partial: Vec<u8>,
    /// Bytes for the driver to read.
    input: VecDeque<u8>,
    /// Commands after which the device says nothing more.
    silent: bool,
    /// What each `05` or measurement due sends: the stream.
    stream: VecDeque<Vec<u8>>,
    read_fails: bool,
    fail_writes_after: Option<usize>,
}

/// A device: the test queues what comes, and sees what goes.
#[derive(Clone)]
struct Fake(Arc<Mutex<Seen>>);

impl Fake {
    fn new() -> Fake {
        Fake(Arc::new(Mutex::new(Seen::default())))
    }

    /// Frames that come one per read that finds nothing queued.
    fn stream(&self, frames: impl IntoIterator<Item = Vec<u8>>) {
        self.0.lock().unwrap().stream.extend(frames);
    }

    fn commands(&self) -> Vec<Vec<u8>> {
        self.0.lock().unwrap().commands.clone()
    }

    fn command_types(&self) -> Vec<u8> {
        self.commands().iter().map(|c| c[1]).collect()
    }

    fn device(&self) -> ZketechEbdUsbDevice {
        let mut device = ZketechEbdUsbDevice::from_port(Box::new(self.clone()));
        device.byte_timeout = BYTE_TIMEOUT;
        device.open().unwrap();
        device
    }
}

impl Port for Fake {
    fn name(&self) -> &'static str {
        "/dev/ttyFAKE"
    }
    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }
    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut seen = self.0.lock().unwrap();
        if seen.read_fails {
            return Err(Error::device("Input/output error"));
        }
        if seen.input.is_empty()
            && !seen.silent
            && let Some(frame) = seen.stream.pop_front()
        {
            seen.input.extend(frame);
        }
        let mut n = 0;
        while n < buf.len() {
            let Some(b) = seen.input.pop_front() else { break };
            buf[n] = b;
            n += 1;
        }
        drop(seen);
        if n == 0 {
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
        }
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        let mut seen = self.0.lock().unwrap();
        if seen.fail_writes_after.is_some_and(|n| seen.writes.len() >= n) {
            return Err(Error::device("Broken pipe"));
        }
        seen.writes.push((Instant::now(), data.to_vec()));
        seen.partial.extend_from_slice(data);
        if data.last() == Some(&protocol::FRAME_END) && seen.partial.len() >= protocol::COMMAND_LEN
        {
            let command = std::mem::take(&mut seen.partial);
            seen.commands.push(command);
        }
        Ok(())
    }
    fn purge(&mut self) -> Result<()> {
        Ok(())
    }
}

fn collect(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let deadline = Instant::now() + BOUND;
    let mut packets = Vec::new();
    while Instant::now() < deadline {
        let Ok((_, packet)) = rx.recv_timeout(Duration::from_millis(50)) else { continue };
        let end = matches!(packet, Packet::End);
        packets.push(packet);
        if end {
            return packets;
        }
    }
    panic!("no End within {BOUND:?}, after {} packets", packets.len());
}

fn run(device: &mut ZketechEbdUsbDevice) -> Vec<Packet> {
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    collect(&rx)
}

/// The analog values, in order.
fn values(packets: &[Packet]) -> Vec<(usize, f32)> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some((a.channels[0], a.samples[0])),
            _ => None,
        })
        .collect()
}

fn frames(packets: &[Packet]) -> usize {
    packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count()
}

fn set_limit(device: &mut ZketechEbdUsbDevice, value: f64) -> Result<()> {
    device.set(&Key::CurrentLimit, &Value::F64(value), None)
}

fn set_threshold(device: &mut ZketechEbdUsbDevice, value: f64) -> Result<()> {
    device.set(&Key::UnderVoltageConditionThreshold, &Value::F64(value), None)
}

fn limit_samples(device: &mut ZketechEbdUsbDevice, n: u64) {
    device.set(&Key::LimitSamples, &Value::U64(n), None).unwrap();
}

#[test]
fn a_probe_starts_reads_and_stops() {
    let fake = Fake::new();
    fake.stream([without_load()]);
    let mut port = fake.clone();
    assert!(probe(&mut port, BYTE_TIMEOUT));
    assert_eq!(fake.command_types(), [0x05, 0x06]);
}

#[test]
fn a_probe_of_a_silent_device_still_stops() {
    let fake = Fake::new();
    let mut port = fake.clone();
    assert!(!probe(&mut port, BYTE_TIMEOUT));
    assert_eq!(fake.command_types(), [0x05, 0x06]);
}

#[test]
fn a_probe_takes_a_short_frame_and_not_a_bare_one() {
    let fake = Fake::new();
    fake.stream([vec![0xfa, 0x00, 0xf8]]);
    let mut port = fake.clone();
    assert!(probe(&mut port, BYTE_TIMEOUT));
    let fake = Fake::new();
    fake.stream([vec![0xfa, 0xf8]]);
    let mut port = fake.clone();
    assert!(!probe(&mut port, BYTE_TIMEOUT));
}

#[test]
fn a_probe_that_fails_reading_is_no_device() {
    let fake = Fake::new();
    fake.0.lock().unwrap().read_fails = true;
    let mut port = fake.clone();
    assert!(!probe(&mut port, BYTE_TIMEOUT));
}

/// C waits after each byte, the last too: the device loses them otherwise.
#[test]
fn a_command_goes_out_a_byte_at_a_time_10_ms_apart() {
    let fake = Fake::new();
    let mut port = fake.clone();
    let before = Instant::now();
    send_command(&mut port, &protocol::init()).unwrap();
    assert!(before.elapsed() >= Duration::from_millis(100), "{:?}", before.elapsed());
    let seen = fake.0.lock().unwrap();
    assert_eq!(seen.writes.len(), 10);
    assert!(seen.writes.iter().all(|(_, w)| w.len() == 1));
    let least = seen.writes.windows(2).map(|w| w[1].0 - w[0].0).min().unwrap();
    assert!(least >= Duration::from_millis(10), "{least:?}");
}

#[test]
fn a_write_that_fails_ends_the_command_there() {
    let fake = Fake::new();
    fake.0.lock().unwrap().fail_writes_after = Some(3);
    let mut port = fake.clone();
    assert!(send_command(&mut port, &protocol::init()).is_err());
    assert_eq!(fake.0.lock().unwrap().writes.len(), 3);
}

#[test]
fn settings_read_back_in_single_precision_starting_at_0() {
    let fake = Fake::new();
    let mut device = fake.device();
    assert_eq!(device.get(&Key::CurrentLimit, None).unwrap().as_f64(), Some(0.0));
    set_limit(&mut device, 0.3).unwrap();
    assert_eq!(device.get(&Key::CurrentLimit, None).unwrap().as_f64(), Some(f64::from(0.3f32)));
    set_threshold(&mut device, 3.3).unwrap();
    assert_eq!(
        device.get(&Key::UnderVoltageConditionThreshold, None).unwrap().as_f64(),
        Some(f64::from(3.3f32))
    );
    assert!(fake.commands().is_empty(), "not before a measurement");
}

#[test]
fn settings_out_of_range_are_refused_and_change_nothing() {
    let mut device = Fake::new().device();
    set_limit(&mut device, 1.0).unwrap();
    for bad in [4.001, -0.001, f64::NAN, f64::INFINITY] {
        assert!(set_limit(&mut device, bad).is_err(), "{bad}");
    }
    assert_eq!(device.get(&Key::CurrentLimit, None).unwrap().as_f64(), Some(1.0));
    for bad in [21.01, -1.0, f64::NAN] {
        assert!(set_threshold(&mut device, bad).is_err(), "{bad}");
    }
    assert!(set_limit(&mut device, 4.0).is_ok());
    assert!(set_threshold(&mut device, 21.0).is_ok());
    assert!(set_limit(&mut device, 4.000_000_1).is_ok(), "4.0 in single precision");
}

#[test]
fn a_sample_limit_ends_it_at_that_count() {
    let fake = Fake::new();
    fake.stream(vec![without_load(); 5]);
    let mut device = fake.device();
    limit_samples(&mut device, 2);
    let packets = run(&mut device);
    assert_eq!(frames(&packets), 2);
    assert_eq!(values(&packets).len(), 8);
    assert!(matches!(packets[0], Packet::Header { .. }));
    assert_eq!(fake.command_types(), [0x05, 0x06]);
    assert_eq!(fake.0.lock().unwrap().stream.len(), 3, "the rest unread");
}

#[test]
fn the_values_go_in_the_channels_order() {
    let fake = Fake::new();
    fake.stream([measurement(0x0a, 1500, 300)]);
    let mut device = fake.device();
    limit_samples(&mut device, 1);
    let packets = run(&mut device);
    assert_eq!(values(&packets), [(0, 4.997), (1, 1.5), (2, 1.849), (3, 1.853)]);
    let digits: Vec<i32> = packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some(a.digits),
            _ => None,
        })
        .collect();
    assert_eq!(digits, [3, 4, 3, 3]);
}

#[test]
fn only_channels_that_are_on_go_out() {
    let fake = Fake::new();
    fake.stream([without_load()]);
    let mut device = fake.device();
    limit_samples(&mut device, 1);
    for pos in [0, 2, 3] {
        device.enable_channel(pos, false).unwrap();
    }
    let packets = run(&mut device);
    assert_eq!(values(&packets), [(1, 0.0)]);
}

#[test]
fn a_limit_starts_the_load_with_its_values() {
    let fake = Fake::new();
    fake.stream([without_load(), measurement(0x0a, 1500, 300)]);
    let mut device = fake.device();
    set_limit(&mut device, 1.5).unwrap();
    set_threshold(&mut device, 3.0).unwrap();
    limit_samples(&mut device, 2);
    run(&mut device);
    let commands = fake.commands();
    assert_eq!(fake.command_types(), [0x05, 0x01, 0x02, 0x06], "off at the stop");
    assert_eq!(commands[1], protocol::load_start(1.5, 3.0));
}

/// As in C: the frame that would have said the load is on never came.
#[test]
fn a_capture_of_one_frame_leaves_a_load_it_started_on() {
    let fake = Fake::new();
    fake.stream(vec![without_load(); 2]);
    let mut device = fake.device();
    set_limit(&mut device, 1.5).unwrap();
    limit_samples(&mut device, 1);
    run(&mut device);
    assert_eq!(fake.command_types(), [0x05, 0x01, 0x06]);
}

#[test]
fn a_load_that_stays_off_is_started_each_frame() {
    let fake = Fake::new();
    fake.stream(vec![without_load(); 3]);
    let mut device = fake.device();
    set_limit(&mut device, 1.5).unwrap();
    limit_samples(&mut device, 3);
    run(&mut device);
    assert_eq!(fake.command_types(), [0x05, 0x01, 0x01, 0x01, 0x06]);
}

#[test]
fn no_limit_turns_off_a_load_that_is_on() {
    let fake = Fake::new();
    fake.stream([measurement(0x0a, 1500, 300), without_load()]);
    let mut device = fake.device();
    limit_samples(&mut device, 2);
    run(&mut device);
    assert_eq!(fake.command_types(), [0x05, 0x02, 0x06]);
    assert!(!device.shared().toggle_unconfirmed, "the second frame said so");
}

#[test]
fn a_load_on_with_the_values_set_gets_no_command() {
    let fake = Fake::new();
    fake.stream(vec![measurement(0x0a, 1500, 300); 2]);
    let mut device = fake.device();
    set_limit(&mut device, 1.5).unwrap();
    set_threshold(&mut device, 3.0).unwrap();
    limit_samples(&mut device, 2);
    run(&mut device);
    assert_eq!(fake.command_types(), [0x05, 0x02, 0x06], "off at the stop alone");
}

#[test]
fn a_load_on_with_another_threshold_alone_gets_it() {
    let fake = Fake::new();
    fake.stream([measurement(0x0a, 1500, 300)]);
    let mut device = fake.device();
    set_limit(&mut device, 1.5).unwrap();
    set_threshold(&mut device, 4.0).unwrap();
    limit_samples(&mut device, 1);
    run(&mut device);
    assert_eq!(fake.commands()[1], protocol::config(1.5, 4.0).to_vec());
}

/// The frame that toggled the load off cannot yet say so: both tools toggle again.
#[test]
fn a_capture_of_one_frame_with_nothing_set_toggles_twice() {
    let fake = Fake::new();
    fake.stream([measurement(0x0a, 1500, 300)]);
    let mut device = fake.device();
    limit_samples(&mut device, 1);
    run(&mut device);
    assert_eq!(fake.command_types(), [0x05, 0x02, 0x02, 0x06]);
    assert!(device.shared().toggle_unconfirmed, "so the stop warned");
}

/// Each `unconfirmed` lasts until the next measurement.
#[test]
fn a_measurement_clears_what_was_unconfirmed() {
    let fake = Fake::new();
    fake.stream([without_load(), measurement(0x0a, 1500, 300)]);
    let mut device = fake.device();
    set_limit(&mut device, 1.5).unwrap();
    set_threshold(&mut device, 3.0).unwrap();
    limit_samples(&mut device, 1);
    run(&mut device);
    assert!(device.shared().start_unconfirmed, "the start is out, and no frame since");
    limit_samples(&mut device, 1);
    run(&mut device);
    let s = device.shared();
    assert!(!s.start_unconfirmed, "the second frame said so");
    assert!(s.load_activated);
}

#[test]
fn a_load_on_with_other_values_gets_them_each_frame() {
    let fake = Fake::new();
    fake.stream(vec![measurement(0x0a, 1500, 300); 2]);
    let mut device = fake.device();
    set_limit(&mut device, 1.0).unwrap();
    set_threshold(&mut device, 3.0).unwrap();
    limit_samples(&mut device, 2);
    run(&mut device);
    let commands = fake.commands();
    assert_eq!(fake.command_types(), [0x05, 0x07, 0x07, 0x02, 0x06]);
    assert_eq!(commands[1], protocol::config(1.0, 3.0));
}

/// A `6e` has other things where a `0a` has its values.
#[test]
fn a_device_frame_with_load_is_not_read_for_values() {
    let fake = Fake::new();
    let mut f = measurement(0x0a, 1500, 300);
    f[1] = 0x6e;
    f[17] = f[1..17].iter().fold(0, |x, b| x ^ b);
    fake.stream(vec![f; 2]);
    let mut device = fake.device();
    set_limit(&mut device, 1.0).unwrap();
    limit_samples(&mut device, 2);
    run(&mut device);
    assert_eq!(fake.command_types(), [0x05, 0x02, 0x06]);
}

#[test]
fn frames_that_fail_a_check_do_not_count() {
    let fake = Fake::new();
    let mut bad_sum = without_load();
    bad_sum[17] ^= 1;
    let mut other_type = without_load();
    other_type[1] = 0x01;
    fake.stream([
        bad_sum,
        vec![0xfa, 0x00, 0xf8],
        other_type,
        vec![0xfa, 0xf8],
        vec![0x55],
        without_load(),
        without_load(),
    ]);
    let mut device = fake.device();
    limit_samples(&mut device, 1);
    let packets = run(&mut device);
    assert_eq!(frames(&packets), 1);
    assert_eq!(fake.0.lock().unwrap().stream.len(), 1, "the good one ended it");
}

#[test]
fn silence_ends_it_with_no_stop_and_says_not_running() {
    let fake = Fake::new();
    let mut device = fake.device();
    limit_samples(&mut device, 3);
    let packets = run(&mut device);
    assert_eq!(frames(&packets), 0);
    assert!(matches!(&packets[..], [.., Packet::Failed(_), Packet::End]), "{packets:?}");
    assert_eq!(fake.command_types(), [0x05], "C sends no stop here");
}

#[test]
fn silence_after_a_frame_sets_later_again() {
    let fake = Fake::new();
    fake.stream([without_load()]);
    let mut device = fake.device();
    let packets = run(&mut device);
    assert_eq!(frames(&packets), 1);
    set_limit(&mut device, 1.0).unwrap();
    assert_eq!(fake.command_types(), [0x05], "not running, so nothing goes out");
}

#[test]
fn a_read_that_fails_ends_it() {
    let fake = Fake::new();
    fake.stream([without_load()]);
    let mut device = fake.device();
    let (tx, rx) = sync_channel(64);
    fake.0.lock().unwrap().read_fails = true;
    device.start(PacketSink::new(tx, 0)).unwrap();
    let packets = collect(&rx);
    assert_eq!(frames(&packets), 0);
    assert_eq!(fake.command_types(), [0x05, 0x06], "what a stop sends, for a port still there");
}

/// C keeps the source after a read error, so a later stop still sends these.
#[test]
fn a_read_that_fails_with_the_load_on_sends_what_a_stop_sends() {
    let fake = Fake::new();
    fake.stream([measurement(0x0a, 1500, 300)]);
    let mut device = fake.device();
    set_limit(&mut device, 1.5).unwrap();
    set_threshold(&mut device, 3.0).unwrap();
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let deadline = Instant::now() + BOUND;
    loop {
        assert!(Instant::now() < deadline, "no frame");
        if let Ok((_, Packet::FrameEnd)) = rx.recv_timeout(Duration::from_millis(50)) {
            break;
        }
    }
    fake.0.lock().unwrap().read_fails = true;
    collect(&rx);
    assert_eq!(fake.command_types(), [0x05, 0x02, 0x06]);
}

#[test]
fn a_read_that_fails_on_a_port_gone_ends_all_the_same() {
    let fake = Fake::new();
    {
        let mut seen = fake.0.lock().unwrap();
        seen.read_fails = true;
        seen.fail_writes_after = Some(10);
    }
    let mut device = fake.device();
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let packets = collect(&rx);
    assert!(matches!(&packets[..], [.., Packet::Failed(_), Packet::End]), "{packets:?}");
    assert_eq!(fake.command_types(), [0x05]);
}

/// A stream that goes on, ended by the stop alone.
#[test]
fn a_stop_ends_a_stream_and_sends_what_a_stop_sends() {
    let fake = Fake::new();
    fake.stream(vec![measurement(0x0a, 1500, 300); 100_000]);
    let mut device = fake.device();
    set_limit(&mut device, 1.5).unwrap();
    set_threshold(&mut device, 3.0).unwrap();
    let (tx, rx) = sync_channel(1024);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let deadline = Instant::now() + BOUND;
    let mut seen = 0;
    while seen < 8 {
        assert!(Instant::now() < deadline, "no frames");
        if let Ok((_, Packet::Analog(_))) = rx.recv_timeout(Duration::from_millis(50)) {
            seen += 1;
        }
    }
    device.stop().unwrap();
    let packets = collect(&rx);
    assert!(matches!(packets.last(), Some(Packet::End)));
    assert_eq!(fake.command_types(), [0x05, 0x02, 0x06]);
    assert!(fake.0.lock().unwrap().stream.len() > 90_000, "it stopped reading");
    device.close().unwrap();
}

/// A stop during a silent device's wait does not wait for the turns.
#[test]
fn a_stop_in_a_wait_is_seen_within_its_turns() {
    let fake = Fake::new();
    let mut device = fake.device();
    device.byte_timeout = Duration::from_millis(50);
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    // The init is out by the time it reads, so the stop falls in the reads.
    let deadline = Instant::now() + BOUND;
    while fake.command_types().is_empty() {
        assert!(Instant::now() < deadline, "no init");
        std::thread::sleep(Duration::from_millis(1));
    }
    let before = Instant::now();
    device.stop().unwrap();
    let packets = collect(&rx);
    assert!(before.elapsed() < Duration::from_secs(5), "{:?}", before.elapsed());
    assert!(matches!(packets.last(), Some(Packet::End)));
    assert_eq!(fake.command_types(), [0x05, 0x06]);
}

/// A close joins the thread, whose queue no one reads: `send_while`.
#[test]
fn a_close_with_a_full_queue_ends() {
    let fake = Fake::new();
    fake.stream(vec![without_load(); 10_000]);
    let mut device = fake.device();
    let (tx, rx) = sync_channel(1);
    device.start(PacketSink::new(tx, 0)).unwrap();
    // The queue fills: the header, and the thread then blocks on a frame.
    let deadline = Instant::now() + BOUND;
    while fake.command_types().is_empty() {
        let _ = rx.try_recv();
        assert!(Instant::now() < deadline, "no init");
        std::thread::sleep(Duration::from_millis(1));
    }
    std::thread::sleep(Duration::from_millis(20));
    let (done_tx, done_rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        device.close().unwrap();
        done_tx.send(()).unwrap();
    });
    assert!(done_rx.recv_timeout(BOUND).is_ok(), "the close did not end");
    drop(rx);
    assert_eq!(fake.command_types().last(), Some(&0x06));
}

/// Wherever in a frame the queue fills: each capacity blocks on another packet,
/// the header and the first frame's begin, values and end, and the next one's.
#[test]
fn a_close_ends_whichever_packet_the_queue_is_full_at() {
    for capacity in 0..=8 {
        let fake = Fake::new();
        fake.stream(vec![without_load(); 10]);
        let mut device = fake.device();
        let (tx, rx) = sync_channel(capacity);
        device.start(PacketSink::new(tx, 0)).unwrap();
        std::thread::sleep(Duration::from_millis(150));
        let (done_tx, done_rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            device.close().unwrap();
            done_tx.send(()).unwrap();
        });
        assert!(done_rx.recv_timeout(BOUND).is_ok(), "capacity {capacity}: no end to the close");
        drop(rx);
    }
}

/// The consumer gone is a stop too: the load must not stay on for it.
#[test]
fn a_consumer_gone_sends_what_a_stop_sends() {
    let fake = Fake::new();
    fake.stream(vec![measurement(0x0a, 1500, 300); 100]);
    let mut device = fake.device();
    let (tx, rx) = sync_channel(2);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let _ = rx.recv_timeout(BOUND);
    drop(rx);
    let deadline = Instant::now() + BOUND;
    while fake.command_types().last() != Some(&0x06) {
        assert!(Instant::now() < deadline, "no stop: {:?}", fake.command_types());
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(fake.command_types().contains(&0x02));
    device.close().unwrap();
}

#[test]
fn a_second_start_is_refused_while_one_runs_and_taken_after() {
    let fake = Fake::new();
    let mut device = fake.device();
    device.byte_timeout = Duration::from_millis(5);
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let (tx2, _rx2) = sync_channel(64);
    assert!(matches!(device.start(PacketSink::new(tx2, 0)), Err(Error::Busy)));
    device.stop().unwrap();
    collect(&rx);
    fake.stream([without_load()]);
    device.byte_timeout = BYTE_TIMEOUT;
    limit_samples(&mut device, 1);
    assert_eq!(frames(&run(&mut device)), 1);
}

/// A client that holds a full queue and starts again: the End it cannot take must
/// not hold the start for ever.
#[test]
fn a_start_after_a_capture_whose_end_is_blocked_returns() {
    let fake = Fake::new();
    fake.stream(vec![without_load(); 3]);
    let mut device = fake.device();
    limit_samples(&mut device, 1);
    // Room for the header and one frame of six, and none for the End.
    let (tx, rx) = sync_channel(7);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let deadline = Instant::now() + BOUND;
    while device.shared().capturing {
        assert!(Instant::now() < deadline, "the capture went on");
        std::thread::sleep(Duration::from_millis(5));
    }
    let (done_tx, done_rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let (tx2, rx2) = sync_channel(64);
        let started = device.start(PacketSink::new(tx2, 0)).is_ok();
        done_tx.send(started).unwrap();
        // The device and the new queue stay until the test has its answer.
        std::thread::sleep(Duration::from_millis(100));
        drop((device, rx2));
    });
    let got = done_rx.recv_timeout(BOUND);
    drop(rx);
    assert_eq!(got, Ok(true), "the start did not come back");
}

/// `protocol.c:379-380` reads the limit each time.
#[test]
fn a_sample_limit_set_in_a_capture_counts() {
    let fake = Fake::new();
    fake.stream(vec![without_load(); 100_000]);
    let mut device = fake.device();
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let deadline = Instant::now() + BOUND;
    loop {
        assert!(Instant::now() < deadline, "no frame");
        if let Ok((_, Packet::FrameEnd)) = rx.recv_timeout(Duration::from_millis(50)) {
            break;
        }
    }
    limit_samples(&mut device, 1);
    assert_eq!(device.get(&Key::LimitSamples, None).unwrap().as_u64(), Some(1));
    let packets = collect(&rx);
    assert!(matches!(packets.last(), Some(Packet::End)));
    assert!(fake.0.lock().unwrap().stream.len() > 90_000, "it read on");
    assert_eq!(fake.command_types(), [0x05, 0x06]);
}

/// All frames dropped: only a stop ends it, with what a stop sends and no sample.
#[test]
fn a_stop_ends_a_run_of_dropped_frames() {
    let fake = Fake::new();
    let mut bad = without_load();
    bad[17] ^= 1;
    fake.stream(vec![bad; 100_000]);
    let mut device = fake.device();
    limit_samples(&mut device, 1);
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let deadline = Instant::now() + BOUND;
    while fake.0.lock().unwrap().stream.len() > 99_000 {
        assert!(Instant::now() < deadline, "it did not read");
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(fake.command_types(), [0x05], "nothing has ended it");
    device.stop().unwrap();
    let packets = collect(&rx);
    assert_eq!(frames(&packets), 0);
    assert_eq!(fake.command_types(), [0x05, 0x06]);
}

/// `protocol.c:411-439`: after a measurement a setting goes out at once, which
/// no sigrok-cli run reaches, and counts as done where C's `send_cmd` says failed.
#[test]
fn a_setting_after_a_measurement_goes_out_at_once() {
    let cases: [(&str, Vec<u8>, f64, Vec<u8>); 4] = [
        ("on, a limit", measurement(0x0a, 1000, 0), 2.0, vec![0x07]),
        ("on, none", measurement(0x0a, 1000, 0), 0.0, vec![0x02]),
        ("off, a limit", without_load(), 2.0, vec![0x01]),
        ("off, none", without_load(), 0.0, vec![]),
    ];
    for (name, frame, limit, sent) in cases {
        let fake = Fake::new();
        // Frames go on until the stop, so that no silence ends `running` first.
        fake.stream(vec![frame; 100_000]);
        let mut device = fake.device();
        let (tx, rx) = sync_channel(64);
        // The load's own answer to the frame is not the point: set it equal.
        if name.starts_with("on") {
            set_limit(&mut device, 1.0).unwrap();
        }
        device.start(PacketSink::new(tx, 0)).unwrap();
        let deadline = Instant::now() + BOUND;
        loop {
            assert!(Instant::now() < deadline, "{name}: no frame");
            if let Ok((_, Packet::FrameEnd)) = rx.recv_timeout(Duration::from_millis(50)) {
                break;
            }
        }
        device.stop().unwrap();
        collect(&rx);
        let before = fake.commands().len();
        set_limit(&mut device, limit).unwrap();
        let after: Vec<u8> = fake.commands()[before..].iter().map(|c| c[1]).collect();
        assert_eq!(after, sent, "{name}");
    }
}

#[test]
fn a_threshold_after_a_measurement_goes_out_too() {
    let fake = Fake::new();
    fake.stream(vec![measurement(0x0a, 1000, 0); 100_000]);
    let mut device = fake.device();
    set_limit(&mut device, 1.0).unwrap();
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let deadline = Instant::now() + BOUND;
    loop {
        assert!(Instant::now() < deadline, "no frame");
        if let Ok((_, Packet::FrameEnd)) = rx.recv_timeout(Duration::from_millis(50)) {
            break;
        }
    }
    device.stop().unwrap();
    collect(&rx);
    set_threshold(&mut device, 3.0).unwrap();
    assert_eq!(fake.commands().last().unwrap(), &protocol::config(1.0, 3.0).to_vec());
}

/// A setter's turn comes between two reads of a capture that only waits.
#[test]
fn a_setting_gets_in_during_a_capture_that_waits() {
    let fake = Fake::new();
    fake.stream([without_load()]);
    let mut device = fake.device();
    // 200 turns of this are seconds: a reader that kept the lock would hold the set
    // until the silence ended the capture and `running` with it.
    device.byte_timeout = Duration::from_millis(20);
    let (tx, rx) = sync_channel(64);
    device.start(PacketSink::new(tx, 0)).unwrap();
    let deadline = Instant::now() + BOUND;
    loop {
        assert!(Instant::now() < deadline, "no frame");
        if let Ok((_, Packet::FrameEnd)) = rx.recv_timeout(Duration::from_millis(50)) {
            break;
        }
    }
    std::thread::sleep(Duration::from_millis(50));
    let before = Instant::now();
    set_limit(&mut device, 1.0).unwrap();
    assert!(before.elapsed() < BOUND, "{:?}", before.elapsed());
    assert_eq!(fake.command_types().last(), Some(&0x01), "sent, in the wait");
    device.stop().unwrap();
    collect(&rx);
}

#[test]
fn a_close_says_a_setting_went_nowhere_only_when_it_did() {
    let fake = Fake::new();
    let mut device = fake.device();
    set_limit(&mut device, 1.0).unwrap();
    assert!(device.shared().unsent);
    device.close().unwrap();
    assert!(!device.shared().unsent);

    let fake = Fake::new();
    fake.stream([without_load()]);
    let mut device = fake.device();
    set_limit(&mut device, 1.0).unwrap();
    limit_samples(&mut device, 1);
    run(&mut device);
    assert!(!device.shared().unsent, "a capture compared it");
}

#[test]
fn a_port_closed_gives_an_error_to_a_start() {
    let mut device = Fake::new().device();
    device.close().unwrap();
    let (tx, _rx) = sync_channel(4);
    assert!(device.start(PacketSink::new(tx, 0)).is_err());
}

#[test]
fn the_options_are_cs_in_its_order() {
    let device = Fake::new().device();
    let keys: Vec<Key> = device.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
    assert_eq!(
        keys,
        [
            Key::Continuous,
            Key::CurrentLimit,
            Key::UnderVoltageConditionThreshold,
            Key::LimitSamples
        ]
    );
    assert!(device.options(Some("x")).is_err());
    let names: Vec<&str> = device.channels().iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["V.VBUS", "I.VBUS", "V.D+", "V.D-"]);
}
