//! The driver against a hand written session of a Saleae Logic clone
//! with the firmware running: open (configuration, version
//! checks, claim), a start command for 1 MHz and three transfers of samples.

use oxrok::core::session::Flow;
use oxrok::core::{Device, Key, Packet, Session, Trigger, TriggerMatch, Value};
use oxrok::drivers::fx2lafw::Fx2lafwDevice;
use oxrok::usb::record::Replay;

const SESSION: &str = "# oxrok usb fixture 1\n\
    device vid=0925 pid=3881 version=0001 bus=1 addr=4 manufacturer=\"sigrok\" product=\"fx2lafw\"\n\
    config 1 -> ok\n\
    ctl_in 40 b0 0000 0000 2 -> 0104\n\
    ctl_in 40 b2 0000 0000 1 -> 01\n\
    claim 0 -> ok\n\
    ctl_out 40 b1 0000 0000 40002f -> ok\n\
    bulk_in 82 10240 -> 00{5000}01{5240}\n\
    bulk_in 82 10240 -> ff{10240}\n\
    bulk_in 82 10240 -> aa{10240}\n";

fn capture(trigger: Option<Trigger>, limit: u64, ratio: u64) -> (Vec<u8>, Option<usize>) {
    let replay = Replay::from_text(SESSION).unwrap();
    let mut dev = Fx2lafwDevice::from_transport(Box::new(replay)).unwrap();
    dev.open().unwrap();
    dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(limit), None).unwrap();
    dev.set(&Key::CaptureRatio, &Value::U64(ratio), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    session.set_trigger(trigger);
    let mut data = Vec::new();
    let mut at = None;
    session
        .run(|_, p| {
            match p {
                Packet::Trigger => at = Some(data.len()),
                Packet::Logic(l) => {
                    assert_eq!(l.unit_size, 1);
                    data.extend_from_slice(&l.data);
                    if data.len() as u64 >= limit {
                        return Flow::Stop;
                    }
                }
                _ => {}
            }
            Flow::Continue
        })
        .unwrap();
    for mut d in session.into_devices() {
        d.close().unwrap();
    }
    (data, at)
}

#[test]
fn samples_stream_until_the_limit() {
    let (data, at) = capture(None, 20_000, 0);
    assert_eq!(data.len(), 20_000);
    assert_eq!(at, None);
    assert_eq!(&data[4_999..5_001], [0, 1]);
    assert_eq!(data[10_240], 0xff);
}

#[test]
fn software_trigger_keeps_the_pre_trigger_window() {
    let mut t = Trigger::new();
    t.add(0, 0, TriggerMatch::Rising);
    let (data, at) = capture(Some(t), 12_000, 50);
    // 6000 samples of pre-trigger window were asked for, 5000 exist.
    assert_eq!(at, Some(5_000));
    assert_eq!(data.len(), 12_000);
    assert!(data[..5_000].iter().all(|&b| b == 0));
    assert!(data[5_000..10_240].iter().all(|&b| b == 1));
    assert!(data[10_240..].iter().all(|&b| b == 0xff));
    // A smaller pre-trigger window drops the oldest samples.
    let mut t = Trigger::new();
    t.add(0, 0, TriggerMatch::Rising);
    let (data, at) = capture(Some(t), 1_000, 10);
    assert_eq!(at, Some(100));
    assert_eq!(data.len(), 1_000);
}

/// As `receive_transfer()` does: the second frame goes on in the same
/// transfer, it does not start the device again.
#[test]
fn frames_take_the_samples_that_follow() {
    let text = "# oxrok usb fixture 1\n\
        device vid=0925 pid=3881 version=0001 bus=1 addr=4 manufacturer=\"sigrok\" product=\"fx2lafw\"\n\
        config 1 -> ok\n\
        ctl_in 40 b0 0000 0000 2 -> 0104\n\
        ctl_in 40 b2 0000 0000 1 -> 01\n\
        claim 0 -> ok\n\
        ctl_out 40 b1 0000 0000 40002f -> ok\n\
        bulk_in 82 10240 -> 00{4}01{4}02{10232}\n";
    let mut dev =
        Fx2lafwDevice::from_transport(Box::new(Replay::from_text(text).unwrap())).unwrap();
    dev.open().unwrap();
    dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
    dev.set(&Key::LimitFrames, &Value::U64(2), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let mut frames = Vec::new();
    session
        .run(|_, p| {
            match p {
                Packet::FrameBegin => frames.push(Vec::new()),
                Packet::Logic(l) => frames.last_mut().unwrap().extend_from_slice(&l.data),
                _ => {}
            }
            Flow::Continue
        })
        .unwrap();
    // The C code begins a third frame in what is left of the transfer
    // before it sees that the second was the last.
    assert_eq!(frames, [vec![0; 4], vec![1; 4], vec![]]);
}

/// Frames that end exactly at the end of a transfer: the next frame
/// starts with the next transfer (libsigrok crashes there).
#[test]
fn frames_ending_on_a_transfer_boundary() {
    let text = "# oxrok usb fixture 1\n\
        device vid=0925 pid=3881 version=0001 bus=1 addr=4 manufacturer=\"sigrok\" product=\"fx2lafw\"\n\
        config 1 -> ok\n\
        ctl_in 40 b0 0000 0000 2 -> 0104\n\
        ctl_in 40 b2 0000 0000 1 -> 01\n\
        claim 0 -> ok\n\
        ctl_out 40 b1 0000 0000 40002f -> ok\n\
        bulk_in 82 10240 -> 00{10240}\n\
        bulk_in 82 10240 -> 01{10240}\n\
        bulk_in 82 10240 -> 02{10240}\n";
    let mut dev =
        Fx2lafwDevice::from_transport(Box::new(Replay::from_text(text).unwrap())).unwrap();
    dev.open().unwrap();
    dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(5120), None).unwrap();
    dev.set(&Key::LimitFrames, &Value::U64(3), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let mut frames = Vec::new();
    session
        .run(|_, p| {
            match p {
                Packet::FrameBegin => frames.push(Vec::new()),
                Packet::Logic(l) => frames.last_mut().unwrap().extend_from_slice(&l.data),
                _ => {}
            }
            Flow::Continue
        })
        .unwrap();
    let firsts: Vec<(usize, Option<u8>)> =
        frames.iter().map(|f| (f.len(), f.first().copied())).collect();
    // The last, empty frame begins in what is left of the third
    // transfer, as in frames_take_the_samples_that_follow.
    assert_eq!(firsts, [(5120, Some(0)), (5120, Some(0)), (5120, Some(1)), (0, None)]);
}

#[test]
fn a_wrong_rate_fails_before_the_device_is_touched() {
    let replay = Replay::from_text(SESSION).unwrap();
    let mut dev = Fx2lafwDevice::from_transport(Box::new(replay)).unwrap();
    assert!(dev.set(&Key::Samplerate, &Value::U64(7_000_000), None).is_err());
    // 48 MHz is in the list and, as in libsigrok, taken on 8 channels.
    dev.set(&Key::Samplerate, &Value::U64(48_000_000), None).unwrap();
}

/// A session recorded on a Saleae Logic clone (firmware 1.4) with
/// `OXROK_USB_RECORD`, 2026-10-22, inputs floating high. The claim
/// has since moved after the version reads, as in the C driver; it is
/// no transfer, so the device saw the same.
#[test]
fn recorded_capture_replays() {
    let replay = Replay::from_text(include_str!("fixtures/fx2lafw/capture-1m.txt")).unwrap();
    let mut dev = Fx2lafwDevice::from_transport(Box::new(replay)).unwrap();
    dev.open().unwrap();
    dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(2000), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let mut data = Vec::new();
    session
        .run(|_, p| {
            if let Packet::Logic(l) = p {
                data.extend_from_slice(&l.data);
                if data.len() >= 2000 {
                    return Flow::Stop;
                }
            }
            Flow::Continue
        })
        .unwrap();
    assert_eq!(data.len(), 2000);
    assert!(data.iter().all(|&b| b == 0xff));
    // The bits output printed during the recording agrees.
    let bits = include_str!("fixtures/fx2lafw/capture-1m.bits");
    let d0 = bits.lines().find(|l| l.starts_with("D0:")).unwrap();
    assert!(d0.chars().skip(3).all(|c| c == '1' || c == ' '), "{d0}");
}

#[test]
fn probe_names_rename_and_trim_channels() {
    let replay = Replay::from_text(SESSION).unwrap();
    let mut dev = Fx2lafwDevice::from_transport(Box::new(replay)).unwrap();
    dev.name_channels("uart,cs");
    let names: Vec<&str> = dev.channels().iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["rx", "tx", "cs", "D3", "D4", "D5", "D6", "D7"]);
    dev.name_channels("-i2c");
    assert_eq!(dev.channels().len(), 2);
    assert_eq!(dev.channel_groups()[0].channels, [0, 1]);
}

const OPEN: &str = "# oxrok usb fixture 1\n\
    device vid=0925 pid=3881 version=0001 bus=1 addr=4 manufacturer=\"sigrok\" product=\"fx2lafw\"\n\
    config 1 -> ok\n";

/// A capture of `limit` samples at 1 MHz on a replay of `OPEN` plus
/// `steps`: the samples, or the error the open or the session fails
/// with.
fn run_steps(steps: &str, limit: u64) -> Result<Vec<u8>, String> {
    let replay = Replay::from_text(&format!("{OPEN}{steps}")).unwrap();
    let mut dev = Fx2lafwDevice::from_transport(Box::new(replay)).unwrap();
    dev.open().map_err(|e| e.to_string())?;
    dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(limit), None).unwrap();
    let mut session = Session::new();
    session.add_device(Box::new(dev));
    let mut data = Vec::new();
    session
        .run(|_, p| {
            if let Packet::Logic(l) = p {
                data.extend_from_slice(&l.data);
            }
            Flow::Continue
        })
        .map_err(|e| e.to_string())?;
    Ok(data)
}

const CHECKS: &str = "ctl_in 40 b0 0000 0000 2 -> 0104\n\
    ctl_in 40 b2 0000 0000 1 -> 01\n\
    claim 0 -> ok\n";

/// As `receive_transfer()`: a failed transfer (a STALL) counts as an
/// empty one and the acquisition goes on; 65 in a row and the FX2
/// "gave up", the acquisition ends without samples.
#[test]
fn stalled_and_empty_transfers_count_alike() {
    let start = "ctl_out 40 b1 0000 0000 40002f -> ok\n";
    let stall = "bulk_in 82 10240 -> err device \"endpoint stalled\"\n";
    let empty = "bulk_in 82 10240 -> -\n";
    let data = "bulk_in 82 10240 -> 07{10240}\n";
    let steps = format!("{CHECKS}{start}{stall}{empty}{stall}{data}");
    assert_eq!(run_steps(&steps, 16).unwrap(), [7; 16]);
    let steps = format!("{CHECKS}{start}{}{}{data}", stall.repeat(30), empty.repeat(35));
    assert_eq!(run_steps(&steps, 16).unwrap(), Vec::<u8>::new());
    // 64 in a row are not enough.
    let steps = format!("{CHECKS}{start}{}{}{data}", stall.repeat(30), empty.repeat(34));
    assert_eq!(run_steps(&steps, 16).unwrap(), [7; 16]);
}

/// A refused start command fails the session's start, as in
/// `fx2lafw_start_acquisition()`.
#[test]
fn a_refused_start_fails_the_session() {
    let steps = format!("{CHECKS}ctl_out 40 b1 0000 0000 40002f -> err device \"refused\"\n");
    assert!(run_steps(&steps, 16).unwrap_err().contains("refused"));
}

/// The version and the REVID are read before the version is judged
/// and before the claim, as in `fx2lafw_dev_open()` and `dev_open()`.
#[test]
fn open_reads_both_versions_then_claims() {
    // A REVID that fails shows it is read before the version counts.
    let steps = "ctl_in 40 b0 0000 0000 2 -> 0200\n\
        ctl_in 40 b2 0000 0000 1 -> err device \"no revid\"\n";
    assert!(run_steps(steps, 16).unwrap_err().contains("no revid"));
    let steps = "ctl_in 40 b0 0000 0000 2 -> 0200\n\
        ctl_in 40 b2 0000 0000 1 -> 01\n";
    assert!(run_steps(steps, 16).unwrap_err().contains("expected firmware version 1.x, got 2.0"));
    let steps = "ctl_in 40 b0 0000 0000 2 -> 0104\n\
        ctl_in 40 b2 0000 0000 1 -> 01\n\
        claim 0 -> err busy\n";
    assert!(run_steps(steps, 16).unwrap_err().contains("Another program or driver"));
}
