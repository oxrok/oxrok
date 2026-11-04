//! The binary against the demo driver, compared with output captured from
//! sigrok-cli 0.7.2 with libsigrok 0.5.2 on 2026-10-22. The first header
//! line names the tool and is skipped; everything after it must match
//! byte for byte.

use std::process::Command;

fn oxrok(args: &[&str]) -> (String, String, bool) {
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok")).args(args).output().expect("run oxrok");
    (
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
        out.status.success(),
    )
}

/// Everything after the first line.
fn body(stdout: &str) -> String {
    stdout.split_once('\n').map_or("", |(_, rest)| rest).to_owned()
}

const DEMO4: &str = "demo:logic_channels=4:analog_channels=0";
const DEMO8: &str = "demo:logic_channels=8:analog_channels=0";

#[test]
fn scan_lists_the_demo_device() {
    let (out, _, ok) = oxrok(&["-d", "demo", "--scan"]);
    assert!(ok);
    assert_eq!(
        out,
        "The following devices were found:\ndemo - Demo device with 13 channels: D0 D1 D2 D3 D4 D5 D6 D7 A0 A1 A2 A3 A4\n"
    );
    let (out, _, _) = oxrok(&["-d", "demo:logic_channels=3:analog_channels=1", "--scan"]);
    assert_eq!(
        out,
        "The following devices were found:\ndemo - Demo device with 4 channels: D0 D1 D2 A0\n"
    );
}

#[test]
fn walking_one_bits_matches_sigrok_cli() {
    let (out, err, ok) = oxrok(&[
        "-d",
        DEMO4,
        "-g",
        "Logic",
        "--samples",
        "20",
        "-O",
        "bits",
        "-c",
        "pattern=walking-one",
    ]);
    assert!(ok, "{err}");
    assert_eq!(
        body(&out),
        "Acquisition with 4/4 channels at 200 kHz\n\
         D0:01000010 00010000 1000\n\
         D1:00100001 00001000 0100\n\
         D2:00010000 10000100 0010\n\
         D3:00001000 01000010 0001\n"
    );
}

#[test]
fn walking_zero_and_graycode_match_sigrok_cli() {
    let (out, _, _) = oxrok(&[
        "-d",
        DEMO4,
        "-g",
        "Logic",
        "--samples",
        "20",
        "-O",
        "bits",
        "-c",
        "pattern=walking-zero",
    ]);
    assert_eq!(
        body(&out),
        "Acquisition with 4/4 channels at 200 kHz\n\
         D0:10111101 11101111 0111\n\
         D1:11011110 11110111 1011\n\
         D2:11101111 01111011 1101\n\
         D3:11110111 10111101 1110\n"
    );
    let (out, _, _) = oxrok(&[
        "-d",
        DEMO4,
        "-g",
        "Logic",
        "--samples",
        "20",
        "-O",
        "bits",
        "-c",
        "pattern=graycode",
    ]);
    assert_eq!(
        body(&out),
        "Acquisition with 4/4 channels at 200 kHz\n\
         D0:11001100 11001100 1100\n\
         D1:01111000 01111000 0111\n\
         D2:00011111 11100000 0001\n\
         D3:00000001 11111110 0000\n"
    );
}

#[test]
fn incremental_bits_wrap_lines_like_sigrok_cli() {
    let (out, _, _) = oxrok(&[
        "-d",
        DEMO8,
        "-g",
        "Logic",
        "--samples",
        "100",
        "-O",
        "bits",
        "-c",
        "pattern=incremental",
    ]);
    assert_eq!(
        body(&out),
        "Acquisition with 8/8 channels at 200 kHz\n\
         D0:01010101 01010101 01010101 01010101 01010101 01010101 01010101 01010101\n\
         D1:00110011 00110011 00110011 00110011 00110011 00110011 00110011 00110011\n\
         D2:00001111 00001111 00001111 00001111 00001111 00001111 00001111 00001111\n\
         D3:00000000 11111111 00000000 11111111 00000000 11111111 00000000 11111111\n\
         D4:00000000 00000000 11111111 11111111 00000000 00000000 11111111 11111111\n\
         D5:00000000 00000000 00000000 00000000 11111111 11111111 11111111 11111111\n\
         D6:00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000\n\
         D7:00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000\n\
         D0:01010101 01010101 01010101 01010101 0101\n\
         D1:00110011 00110011 00110011 00110011 0011\n\
         D2:00001111 00001111 00001111 00001111 0000\n\
         D3:00000000 11111111 00000000 11111111 0000\n\
         D4:00000000 00000000 11111111 11111111 0000\n\
         D5:00000000 00000000 00000000 00000000 1111\n\
         D6:11111111 11111111 11111111 11111111 1111\n\
         D7:00000000 00000000 00000000 00000000 0000\n"
    );
    let (out, _, _) = oxrok(&[
        "-d",
        "demo:logic_channels=2:analog_channels=0",
        "-g",
        "Logic",
        "--samples",
        "25",
        "-O",
        "bits:width=10",
        "-c",
        "pattern=incremental",
    ]);
    assert_eq!(
        body(&out),
        "Acquisition with 2/2 channels at 200 kHz\n\
         D0:01010101 01\nD1:00110011 00\nD0:01010101 01\nD1:11001100 11\nD0:01010\nD1:00110\n"
    );
}

#[test]
fn hex_and_ascii_match_sigrok_cli() {
    let (out, _, _) = oxrok(&[
        "-d",
        DEMO4,
        "-g",
        "Logic",
        "--samples",
        "20",
        "-O",
        "hex",
        "-c",
        "pattern=walking-one",
    ]);
    assert_eq!(
        body(&out),
        "Acquisition with 4/4 channels at 200 kHz\nD0:42 10 80 \nD1:21 08 40 \nD2:10 84 20 \nD3:08 42 10 \n"
    );
    let (out, _, _) = oxrok(&[
        "-d",
        DEMO8,
        "-g",
        "Logic",
        "--samples",
        "30",
        "-O",
        "hex",
        "-c",
        "pattern=graycode",
    ]);
    assert_eq!(
        body(&out),
        "Acquisition with 8/8 channels at 200 kHz\n\
         D0:cc cc cc cc \nD1:78 78 78 78 \nD2:1f e0 1f e0 \nD3:01 ff fe 00 \n\
         D4:00 01 ff fc \nD5:00 00 00 00 \nD6:00 00 00 00 \nD7:00 00 00 00 \n"
    );
    let (out, _, _) = oxrok(&[
        "-d",
        DEMO4,
        "-g",
        "Logic",
        "--samples",
        "20",
        "-O",
        "ascii",
        "-c",
        "pattern=walking-one",
    ]);
    assert_eq!(
        body(&out),
        "Acquisition with 4/4 channels at 200 kHz\n\
         D0:./\\.../\\.../\\.../\\..\n\
         D1:../\\.../\\.../\\.../\\.\n\
         D2:.../\\.../\\.../\\.../\\\n\
         D3:..../\\.../\\.../\\.../\n"
    );
}

#[test]
fn analog_patterns_match_sigrok_cli() {
    let demo = "demo:logic_channels=0:analog_channels=1";
    let (out, _, _) = oxrok(&["-d", demo, "--samples", "6", "-O", "analog"]);
    assert_eq!(out, "A0: -10.0000 V DC\n".repeat(5) + "A0: 10.0000 V DC\n");
    let (out, _, _) =
        oxrok(&["-d", demo, "--samples", "12", "-O", "analog", "-g", "A0", "-c", "pattern=sine"]);
    assert_eq!(
        out,
        "A0: 0.0 mV DC\nA0: 3.0902 V DC\nA0: 5.8779 V DC\nA0: 8.0902 V DC\nA0: 9.5106 V DC\n\
         A0: 10.0000 V DC\nA0: 9.5106 V DC\nA0: 8.0902 V DC\nA0: 5.8779 V DC\nA0: 3.0902 V DC\n\
         A0: -0.0 mV DC\nA0: -3.0902 V DC\n"
    );
    let (out, _, _) = oxrok(&[
        "-d",
        demo,
        "--samples",
        "6",
        "-O",
        "analog",
        "-g",
        "A0",
        "-c",
        "pattern=triangle",
    ]);
    assert_eq!(
        out,
        "A0: 0.0 mV DC\nA0: 2.0000 V DC\nA0: 4.0000 V DC\nA0: 6.0000 V DC\nA0: 8.0000 V DC\nA0: 10.0000 V DC\n"
    );
    let (out, _, _) = oxrok(&[
        "-d",
        demo,
        "--samples",
        "6",
        "-O",
        "analog",
        "-g",
        "A0",
        "-c",
        "pattern=sawtooth",
    ]);
    assert_eq!(
        out,
        "A0: 0.0 mV DC\nA0: 1000.0 mV DC\nA0: 2.0000 V DC\nA0: 3.0000 V DC\nA0: 4.0000 V DC\nA0: 5.0000 V DC\n"
    );
    let (out, _, _) = oxrok(&[
        "-d",
        demo,
        "--samples",
        "3",
        "-O",
        "analog",
        "-g",
        "A0",
        "-c",
        "amplitude=2:offset=1",
    ]);
    assert_eq!(out, "A0: -1000.0 mV DC\n".repeat(3));
}

#[test]
fn default_output_prints_logic_and_analog() {
    let (out, _, ok) = oxrok(&["-d", "demo", "--samples", "10", "-C", "D0,D2,A0"]);
    assert!(ok);
    // sigrok-cli counts only the logic channels in the header.
    assert!(out.contains("Acquisition with 2/13 channels at 200 kHz\n"));
    assert!(out.contains("A0: -10.0000 V DC\n"));
    // The default pattern spells "oxrok", text low: row 2 of the "o".
    assert!(out.ends_with("D0:11111111 11\nD2:10001101 11\n"), "{out}");
}

#[test]
fn show_prints_the_device_detail() {
    let (out, _, ok) = oxrok(&["-d", "demo", "--show"]);
    assert!(ok);
    assert!(out.starts_with(
        "Driver functions:\n    Demo device\n    Logic analyzer\n    Oscilloscope\n\
         Scan options:\n    logic_channels\n    analog_channels\n    limit_frames\n\
         demo - Demo device with 13 channels: D0 D1 D2 D3 D4 D5 D6 D7 A0 A1 A2 A3 A4\n\
         Channel groups:\n    Logic: channels D0 D1 D2 D3 D4 D5 D6 D7\n    Analog: channels A0 A1 A2 A3 A4\n    A0: channel A0\n"
    ));
    assert!(out.contains("Supported configuration options across all channel groups:\n    continuous: on, off\n    limit_samples: 0 (current)\n"));
    assert!(out.contains(
        "    samplerate (1 Hz - 1 GHz in steps of 1 Hz)\n    averaging: on, off (current)\n"
    ));
    assert!(out.contains("    Supported triggers: 0 1 r f e \n    captureratio: 20 (current)\n"));
    let (out, _, _) = oxrok(&["-d", "demo", "--show", "-g", "Logic"]);
    assert!(out.ends_with(
        "Supported configuration options on channel group Logic:\n    pattern: oxrok (current), random, incremental, walking-one, walking-zero, all-low, all-high, graycode\n"
    ));
}

#[test]
fn get_and_errors() {
    let (out, _, ok) = oxrok(&["-d", "demo", "--get", "samplerate", "-c", "samplerate=1m"]);
    assert!(ok);
    assert_eq!(out, "1000000\n");
    let (_, err, ok) = oxrok(&["-d", "demo", "--samples", "1", "-c", "nonsense=1"]);
    assert!(!ok);
    assert_eq!(err, "Unknown device option 'nonsense'.\n");
    let (_, err, ok) = oxrok(&["-d", "nosuch", "--scan"]);
    assert!(!ok);
    assert_eq!(err, "Driver nosuch not found.\n");
}

#[test]
fn time_limit_and_output_file() {
    let dir = std::env::temp_dir().join(format!("oxrok-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let file = dir.join("out.txt");
    let (out, err, ok) = oxrok(&[
        "-d",
        DEMO4,
        "--time",
        "10",
        "-C",
        "D0",
        "-O",
        "hex",
        "-o",
        file.to_str().unwrap(),
    ]);
    assert!(ok, "{err}");
    assert_eq!(out.len(), 0);
    let text = std::fs::read_to_string(&file).unwrap();
    // 10 ms at 200 kHz is 2000 samples: 192 per line, 11 lines.
    assert_eq!(text.lines().count(), 2 + 11, "{text}");
    std::fs::remove_dir_all(&dir).unwrap();
}

/// The reader of the output goes away mid-capture (`| head -2`): the
/// run ends, rather than waiting for the device, whose thread is
/// blocked on a full queue, to stop.
#[test]
fn a_pipe_closing_mid_capture_ends_the_run() {
    use std::io::Read as _;
    use std::process::Stdio;
    use std::time::{Duration, Instant};
    let mut child = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(["-d", DEMO8, "-c", "samplerate=10m", "--samples", "1000000000", "-O", "hex"])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("run oxrok");
    let mut stdout = child.stdout.take().expect("stdout");
    let mut some = [0u8; 256];
    stdout.read_exact(&mut some).expect("the capture's first output");
    // Reading no more, the pipe and then the session's queue fill up,
    // so the device's thread is blocked when the pipe closes.
    std::thread::sleep(Duration::from_secs(2));
    drop(stdout);
    let limit = Instant::now() + Duration::from_secs(20);
    loop {
        if child.try_wait().expect("wait").is_some() {
            return;
        }
        if Instant::now() > limit {
            child.kill().expect("kill");
            let _ = child.wait();
            panic!("oxrok still runs 20 s after its output closed");
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}
