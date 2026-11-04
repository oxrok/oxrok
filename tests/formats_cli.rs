// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! File formats through the binary: session files, VCD, CSV and binary,
//! compared with output captured from sigrok-cli 0.7.2 / libsigrok 0.5.2
//! on 2026-10-22, and round trips through our own readers.

use std::path::PathBuf;
use std::process::Command;

fn oxrok(args: &[&str]) -> (String, String, bool) {
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok")).args(args).output().expect("run oxrok");
    (
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
        out.status.success(),
    )
}

fn oxrok_bytes(args: &[&str]) -> Vec<u8> {
    Command::new(env!("CARGO_BIN_EXE_oxrok")).args(args).output().expect("run oxrok").stdout
}

fn body(stdout: &str) -> String {
    stdout.split_once('\n').map_or("", |(_, rest)| rest).to_owned()
}

fn tempdir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("oxrok-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

const DEMO4: &str = "demo:logic_channels=4:analog_channels=0";
const W1: [&str; 6] = ["-d", DEMO4, "-g", "Logic", "-c", "pattern=walking-one"];

const WALKING_ONE_BITS: &str = "Acquisition with 4/4 channels at 200 kHz\n\
     D0:01000010 00010000 1000\n\
     D1:00100001 00001000 0100\n\
     D2:00010000 10000100 0010\n\
     D3:00001000 01000010 0001\n";

/// The VCD sigrok-cli writes for the walking-one capture, minus the
/// lines that name the tool and the date.
const WALKING_ONE_VCD_BODY: &str = "$comment\n  Acquisition with 4/4 channels at 200 kHz\n$end\n\
$timescale 1 us $end\n$var wire 1 ! D0 $end\n$var wire 1 \" D1 $end\n$var wire 1 # D2 $end\n\
$var wire 1 $ D3 $end\n$upscope $end\n$enddefinitions $end\n\
\n#0  0! 0\" 0# 0$\n#5  1!\n#10  0! 1\"\n#15  0\" 1#\n#20  0# 1$\n#25  0$\n#30  1!\n#35  0! 1\"\n\
#40  0\" 1#\n#45  0# 1$\n#50  0$\n#55  1!\n#60  0! 1\"\n#65  0\" 1#\n#70  0# 1$\n#75  0$\n#80  1!\n\
#85  0! 1\"\n#90  0\" 1#\n#95  0# 1$\n#100\n";

#[test]
fn session_file_round_trip() {
    let dir = tempdir("sr");
    let file = dir.join("w1.sr");
    let args: Vec<&str> =
        W1.iter().copied().chain(["--samples", "20", "-o", file.to_str().unwrap()]).collect();
    let (out, err, ok) = oxrok(&args);
    assert!(ok, "{err}");
    assert_eq!(out.len(), 0);
    // Read it back: same bits, and the properties sigrok-cli --show prints.
    let (out, err, ok) = oxrok(&["-i", file.to_str().unwrap(), "-O", "bits"]);
    assert!(ok, "{err}");
    assert_eq!(body(&out), WALKING_ONE_BITS);
    let (out, _, _) = oxrok(&["-i", file.to_str().unwrap(), "--show"]);
    assert_eq!(
        out,
        "Samplerate: 200000\nChannels: 4\n- D0: logic\n- D1: logic\n- D2: logic\n- D3: logic\n\
         Logic unitsize: 1\nLogic sample count: 20\n"
    );
    // Channel selection on a file.
    let (out, _, _) = oxrok(&["-i", file.to_str().unwrap(), "-C", "D1,D3", "-O", "bits"]);
    assert_eq!(
        body(&out),
        "Acquisition with 2/4 channels at 200 kHz\nD1:00100001 00001000 0100\nD3:00001000 01000010 0001\n"
    );
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn session_file_with_analog() {
    let dir = tempdir("sra");
    let file = dir.join("w1a.sr");
    let (_, err, ok) = oxrok(&[
        "-d",
        "demo:logic_channels=4:analog_channels=2",
        "-g",
        "Logic",
        "-c",
        "pattern=walking-one",
        "--samples",
        "20",
        "-o",
        file.to_str().unwrap(),
    ]);
    assert!(ok, "{err}");
    let (out, _, _) = oxrok(&["-i", file.to_str().unwrap(), "--show"]);
    assert!(
        out.ends_with("Logic unitsize: 1\nLogic sample count: 20\nAnalog sample count: 20\n"),
        "{out}"
    );
    let (out, _, _) = oxrok(&["-i", file.to_str().unwrap(), "-O", "bits"]);
    // Session files carry two digits of analog precision, like libsigrok.
    assert!(out.contains("A0: -10.00 V DC\n"), "{out}");
    assert!(out.contains("A1: 3.09 V DC\n"), "{out}");
    assert!(out.ends_with("D3:00001000 01000010 0001\n"), "{out}");
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn vcd_output_matches_sigrok_cli() {
    let args: Vec<&str> = W1.iter().copied().chain(["--samples", "20", "-O", "vcd"]).collect();
    let (out, err, ok) = oxrok(&args);
    assert!(ok, "{err}");
    let mut lines = out.lines();
    assert!(lines.next().unwrap().starts_with("$date "));
    assert_eq!(lines.next().unwrap(), format!("$version oxrok {} $end", env!("CARGO_PKG_VERSION")));
    let rest: Vec<&str> = lines.filter(|l| !l.starts_with("$scope")).collect();
    assert_eq!(rest.join("\n") + "\n", WALKING_ONE_VCD_BODY);
}

#[test]
fn vcd_input_reads_our_file_and_sigrok_files() {
    let dir = tempdir("vcd");
    let file = dir.join("w1.vcd");
    let args: Vec<&str> = W1
        .iter()
        .copied()
        .chain(["--samples", "20", "-O", "vcd", "-o", file.to_str().unwrap()])
        .collect();
    oxrok(&args);
    // The timescale is 1 us, so reading back gives 1 MHz and five samples
    // per original one; downsample=5 restores the capture.
    let (out, err, ok) =
        oxrok(&["-i", file.to_str().unwrap(), "-I", "vcd:downsample=5", "-O", "bits"]);
    assert!(ok, "{err}");
    assert_eq!(body(&out), WALKING_ONE_BITS);
    let (out, _, _) = oxrok(&["-i", file.to_str().unwrap(), "-O", "bits"]);
    assert!(out.contains("Acquisition with 4/4 channels at 1 MHz\n"));
    assert!(
        out.contains(
            "D0:00000111 11000000 00000000 00000011 11100000 00000000 00000001 11110000\n"
        )
    );
    // A file written by sigrok's VCD output: the libsigrok scope is not
    // prefixed to the names, like libsigrok's reader does.
    let theirs = "$date Thu Oct 22 16:00:23 2026 $end\n$version libsigrok 0.5.2 $end\n$comment\n  Acquisition with 2/2 channels at 200 kHz\n$end\n$timescale 1 us $end\n$scope module libsigrok $end\n$var wire 1 ! D0 $end\n$var wire 1 \" D1 $end\n$upscope $end\n$enddefinitions $end\n#0 0! 1\"\n#5 1!\n#10\n";
    let f2 = dir.join("theirs.vcd");
    std::fs::write(&f2, theirs).unwrap();
    let (out, _, _) = oxrok(&["-i", f2.to_str().unwrap(), "-I", "vcd:downsample=5", "-O", "bits"]);
    assert_eq!(body(&out), "Acquisition with 2/2 channels at 200 kHz\nD0:01\nD1:11\n");
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn csv_matches_sigrok_cli() {
    let args: Vec<&str> = W1.iter().copied().chain(["--samples", "6", "-O", "csv"]).collect();
    let (out, _, _) = oxrok(&args);
    let mut lines = out.lines();
    assert_eq!(
        lines.next().unwrap(),
        format!("; CSV generated by oxrok {}", env!("CARGO_PKG_VERSION"))
    );
    assert!(lines.next().unwrap().starts_with("; from Demo driver and pattern generator on "));
    let rest: Vec<&str> = lines.collect();
    assert_eq!(
        rest.join("\n") + "\n",
        "; Channels (4/4): D0, D1, D2, D3\n; Samplerate: 200 kHz\nlogic,logic,logic,logic\n\
         0,0,0,0\n1,0,0,0\n0,1,0,0\n0,0,1,0\n0,0,0,1\n0,0,0,0\n"
    );
    let args: Vec<&str> = W1
        .iter()
        .copied()
        .chain(["--samples", "6", "-O", "csv:time=true:label=channel:header=false"])
        .collect();
    let (out, _, _) = oxrok(&args);
    assert_eq!(
        out,
        "Time,D0,D1,D2,D3\n0,0,0,0,0\n5,1,0,0,0\n10,0,1,0,0\n15,0,0,1,0\n20,0,0,0,1\n25,0,0,0,0\n"
    );
    let (out, _, _) = oxrok(&[
        "-d",
        "demo:logic_channels=0:analog_channels=1",
        "--samples",
        "6",
        "-O",
        "csv:header=false",
    ]);
    assert_eq!(out, "V DC\n-10\n-10\n-10\n-10\n-10\n10\n");
    let (out, _, _) = oxrok(&[
        "-d",
        DEMO4,
        "-g",
        "Logic",
        "-c",
        "pattern=all-high",
        "--samples",
        "6",
        "-O",
        "csv:time=true:dedup=true:header=false",
    ]);
    assert_eq!(out, "microseconds,logic,logic,logic,logic\n0,1,1,1,1\n25,1,1,1,1\n");
    let args: Vec<&str> = W1
        .iter()
        .copied()
        .chain(["--samples", "3", "-O", "csv:trigger=true:label=off:header=false"])
        .collect();
    let (out, _, _) = oxrok(&args);
    assert_eq!(out, "0,0,0,0,0\n1,0,0,0,0\n0,1,0,0,0\n");
    let (out, _, _) = oxrok(&[
        "-d",
        "demo:logic_channels=0:analog_channels=2",
        "--samples",
        "3",
        "-O",
        "csv:time=true:header=false",
    ]);
    assert_eq!(out, "microseconds,V DC,V DC\n0,-10,0\n5,-10,3.09017\n10,-10,5.87785\n");
}

#[test]
fn binary_output_and_input() {
    let args: Vec<&str> = W1.iter().copied().chain(["--samples", "20", "-O", "binary"]).collect();
    let raw = oxrok_bytes(&args);
    assert_eq!(raw, [0, 1, 2, 4, 8, 0, 1, 2, 4, 8, 0, 1, 2, 4, 8, 0, 1, 2, 4, 8]);
    let dir = tempdir("bin");
    let file = dir.join("w1.bin");
    std::fs::write(&file, &raw).unwrap();
    let (out, err, ok) = oxrok(&[
        "-i",
        file.to_str().unwrap(),
        "-I",
        "binary:numchannels=4:samplerate=200k",
        "-O",
        "bits",
    ]);
    assert!(ok, "{err}");
    // Unlike sigrok-cli, the size suffix is honoured: 200k is 200 kHz,
    // and no META line precedes the data.
    assert_eq!(
        body(&out),
        "Acquisition with 4/4 channels at 200 kHz\n0:01000010 00010000 1000\n\
         1:00100001 00001000 0100\n2:00010000 10000100 0010\n3:00001000 01000010 0001\n"
    );
    let (_, err, ok) = oxrok(&["-i", file.to_str().unwrap(), "-O", "bits"]);
    assert!(!ok);
    assert_eq!(err, "Error: no input module found for this file.\n");
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn list_shows_inputs_and_outputs() {
    let (out, _, _) = oxrok(&["-L"]);
    assert!(out.contains("Supported input formats:\n  binary               Raw binary logic data\n  vcd                  Value Change Dump\n"));
    assert!(out.contains("  srzip                srzip session file format data\n  vcd                  Value Change Dump data\n"));
    let (out, _, _) = oxrok(&["-I", "vcd", "--show"]);
    assert!(out.starts_with("ID: vcd\nName: VCD\n"));
    assert!(out.contains("  downsample: "));
}

#[test]
fn strict_turns_vcd_warnings_into_errors() {
    let dir = tempdir("strict");
    let file = dir.join("x.vcd");
    std::fs::write(
        &file,
        "$timescale 1 us $end\n$scope module top $end\n$var wire 1 ! d0 $end\n\
         $upscope $end\n$enddefinitions $end\n#0\n1!\n#1\nx!\n#2\n0!\n",
    )
    .unwrap();
    let path = file.to_str().unwrap();

    let (out, err, ok) = oxrok(&["-i", path, "-O", "bits"]);
    assert!(ok, "{err}");
    assert!(err.contains("Undefined bit values"), "{err}");
    assert_eq!(body(&out), "Acquisition with 1/1 channels at 1 MHz\ntop.d0:100\n");

    let (_, err, ok) = oxrok(&["-i", path, "-O", "bits", "--strict"]);
    assert!(!ok);
    assert!(err.contains("Undefined bit values"), "{err}");
    assert!(err.contains("1 warning treated as error (--strict)."), "{err}");

    // The count does not depend on the log level.
    let (_, err, ok) = oxrok(&["-i", path, "-O", "bits", "--strict", "-l", "0"]);
    assert!(!ok);
    assert!(!err.contains("Undefined bit values"), "{err}");
    assert!(err.contains("1 warning treated as error (--strict)."), "{err}");

    let (_, err, ok) = oxrok(&["-d", DEMO4, "--samples", "4", "-O", "bits", "--strict"]);
    assert!(ok, "{err}");
}
