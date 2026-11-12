// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

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

/// The core's refusals for every driver, hwdriver.c:746-765. What sigrok-cli
/// says is compared in tools/diffcli.sh.
#[test]
fn the_core_refuses_limits_of_zero() {
    for (args, reason, last) in [
        (
            &["-d", "demo", "--samples", "0"][..],
            "Cannot set 'limit_samples' to 0.",
            "Failed to configure sample limit.",
        ),
        (
            &["-d", "demo", "--samples", "18446744073709551615"][..],
            "Cannot set 'limit_samples' to 0.",
            "Failed to configure sample limit.",
        ),
        (
            &["-d", "demo", "-c", "limit_samples=0", "--samples", "5"][..],
            "Cannot set 'limit_samples' to 0.",
            "Failed to set device option 'limit_samples': invalid argument",
        ),
        (
            &["-d", "demo", "-c", "samplerate=0", "--set"][..],
            "Cannot set 'samplerate' to 0.",
            "Failed to set device option 'samplerate': invalid argument",
        ),
        (
            &["-d", "demo", "-c", "limit_time=0", "--set"][..],
            "Cannot set 'limit_time' to 0.",
            "Failed to set device option 'limit_time': invalid argument",
        ),
        (
            &["-d", "demo", "-c", "captureratio=101", "--set"][..],
            "Capture ratio must be 0..100.",
            "Failed to set device option 'captureratio': invalid argument",
        ),
    ] {
        let (_, err, ok) = oxrok(args);
        assert!(!ok, "{args:?}");
        assert!(err.contains(&format!("oxrok::core::hwdriver: {reason}\n")), "{args:?}: {err}");
        assert!(err.contains(last), "{args:?}: {err}");
    }
}

#[test]
fn the_null_output_discards_logic() {
    let (out, err, ok) = oxrok(&["-d", DEMO4, "--samples", "10", "-O", "null"]);
    assert!(ok, "{err}");
    assert_eq!(out, "");
    // Analog data takes the fallback, as in sigrok-cli.
    let (out, _, ok) =
        oxrok(&["-d", "demo:logic_channels=2:analog_channels=1", "--samples", "2", "-O", "null"]);
    assert!(ok);
    assert_eq!(out, "A0: -10.0000 V DC\nA0: -10.0000 V DC\n");
}

/// `-w` holds back what comes before the trigger, as sigrok-cli's manual
/// says: the output is the rest of the capture, from the trigger on. A
/// difference from sigrok-cli 0.7.2, whose output module gets every sample
/// (docs/CLI.md).
#[test]
fn wait_trigger_starts_the_output_at_the_trigger() {
    let run = |w: bool| {
        let mut args = vec![
            "-d",
            DEMO4,
            "-g",
            "Logic",
            "-c",
            "pattern=incremental",
            "--samples",
            "12",
            "-t",
            "D2=r",
            "-O",
            "csv:header=false:trigger=true",
        ];
        if w {
            args.push("-w");
        }
        let (out, _, ok) = oxrok(&args);
        assert!(ok);
        out
    };
    let all = run(false);
    let waited = run(true);
    let lines: Vec<&str> = all.lines().collect();
    // The title, then a row per sample with the trigger column last.
    let at = lines.iter().position(|l| l.ends_with(",1")).expect("a trigger row");
    assert_eq!(at, 5);
    let want: Vec<&str> = std::iter::once(lines[0]).chain(lines[at..].iter().copied()).collect();
    assert_eq!(waited.lines().collect::<Vec<_>>(), want);
}

/// The software trigger's extras on the counting pattern: the row of the
/// trigger for value ranges and the or/and combines.
#[test]
fn soft_trigger_ranges_and_combines() {
    let row = |t: &str| {
        let (out, err, ok) = oxrok(&[
            "-d",
            DEMO4,
            "-g",
            "Logic",
            "-c",
            "pattern=incremental",
            "--samples",
            "12",
            "-t",
            t,
            "-O",
            "csv:header=false:trigger=true",
        ]);
        assert!(ok, "{t}: {err}");
        // The title, then a row per sample.
        out.lines().position(|l| l.ends_with(",1")).map(|at| at - 1)
    };
    assert_eq!(row("D0-D2=3..5"), Some(3));
    assert_eq!(row("D0-D2=!0..5"), Some(6));
    // D0 first rises at 1, D1 at 2.
    assert_eq!(row("D0=r|D1=r"), Some(1));
    assert_eq!(row("D0=r&D1=r"), Some(2));
}

/// Pulse width triggers on the counting pattern, where D1 is high for
/// two samples in every four and D2 for four in every eight: the row of
/// the edge that ends the pulse.
#[test]
fn soft_trigger_pulse_widths() {
    let run = |t: &str| {
        oxrok(&[
            "-d",
            DEMO4,
            "-g",
            "Logic",
            "-c",
            "pattern=incremental",
            "--samples",
            "12",
            "-t",
            t,
            "-O",
            "csv:header=false:trigger=true",
        ])
    };
    let row = |t: &str| {
        let (out, err, ok) = run(t);
        assert!(ok, "{t}: {err}");
        out.lines().position(|l| l.ends_with(",1")).map(|at| at - 1)
    };
    // D1 is high on 2 and 3, D2 from 4 to 7.
    assert_eq!(row("D1=f<3"), Some(4));
    assert_eq!(row("D1=f>1<3"), Some(4));
    assert_eq!(row("D2=f>3<5"), Some(8));
    assert_eq!(row("D2=f>3"), Some(8));
    // The other channels are read where the pulse ends: D2 is high at
    // 4 and low at 8.
    assert_eq!(row("D1=f<3,D2=1"), Some(4));
    assert_eq!(row("D1=f<3,D2=0"), Some(8));
    // D1's low time on 0 and 1 began before the capture; the one on 4
    // and 5 ends at 6.
    assert_eq!(row("D1=r<3"), Some(6));

    for (t, text) in [
        ("D1=1<3", "A pulse width needs the edge that ends the pulse (r or f) in trigger stage 1."),
        ("D1=e<3", "A pulse width needs one rising or falling edge in trigger stage 1."),
        ("D1=f<1", "No pulse is shorter than 1 samples in trigger stage 1."),
        ("D1=f>4<5", "No pulse width is within 5..4 samples in trigger stage 1."),
    ] {
        let (_, err, ok) = run(t);
        assert!(!ok && err.contains(text), "{t}: {err}");
    }
}

/// With no trigger, `-w` would write nothing at all: refused.
#[test]
fn wait_trigger_needs_a_trigger() {
    let (out, err, ok) = oxrok(&["-d", DEMO4, "--samples", "12", "-w", "-O", "bits"]);
    assert!(!ok);
    assert_eq!(out, "");
    assert_eq!(err, "-w waits for a trigger: give one with -t.\n");
}
