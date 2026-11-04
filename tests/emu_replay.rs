//! Replays of the sessions the emulator records: every
//! `tests/fixtures/DRIVER/emu-*.txt` was written by an `emu_case
//! --record` of `tools/diffcli.sh --emu` (docs/USB.md, "Record and
//! replay"), with the command line as `# arg:` comments, one per
//! argument, and oxrok's stdout next to it as `emu-*.out`. Each runs
//! the oxrok binary again with the same arguments and the fixture as
//! `OXROK_USB_REPLAY` or `OXROK_HID_REPLAY`, which stand in for the
//! device in the listings and opens; the run must succeed (a replay not
//! played to its end fails it) and print the same. So the drivers'
//! logic runs in `cargo test` on every platform, without the emulator
//! or root. Drivers that upload firmware read the emulator's stand-ins
//! (`tools/emu-firmware.sh`), which [`firmware_dir`] writes.

use sha2::{Digest as _, Sha256};
use std::io::Read as _;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::OnceLock;
use std::time::{Duration, Instant};

/// The drivers with an emulator model: each must have a recording.
const RECORDED: [&str; 20] = [
    "asix-sigma",
    "chronovu-la",
    "fx2lafw",
    "hantek-dso",
    "dreamsourcelab-dslogic",
    "saleae-logic16",
    "saleae-logic-pro",
    "ftdi-la",
    "kingst-la2016",
    "p-ols",
    "microchip-pickit2",
    "dcttech-usbrelay",
    "brymen-bm86x",
    "uni-t-ut61e-ser",
    "victor-dmm",
    "ikalogic-scanaplus",
    "sysclk-lwla",
    "lecroy-logicstudio",
    "hantek-4032l",
    "zeroplus-logic-cube",
];

/// How long one replay may take; a driver that waits for ever on a
/// replay that went astray (the `PICkit2` waits for its trigger until it
/// is stopped) fails then rather than holding the test.
const LIMIT: Duration = Duration::from_secs(60);

fn fixtures() -> Vec<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests").join("fixtures");
    let mut out = Vec::new();
    for dir in std::fs::read_dir(&root).unwrap() {
        let dir = dir.unwrap().path();
        if !dir.is_dir() {
            continue;
        }
        for f in std::fs::read_dir(&dir).unwrap() {
            let f = f.unwrap().path();
            let name = f.file_name().unwrap().to_string_lossy().into_owned();
            if name.starts_with("emu-") && f.extension().is_some_and(|e| e == "txt") {
                out.push(f);
            }
        }
    }
    out.sort();
    out
}

/// The firmware stand-ins of every `tools/emu/*.firmware` list, as
/// `tools/emu-firmware.sh` writes them (each file `SHA-256(NAME:0)`,
/// `SHA-256(NAME:1)`, ... cut to its size), in a directory of the
/// test's own: a recording made under the emulator holds the digests of
/// what was uploaded, and a driver that reads its firmware on every open
/// (the `DSLogic`'s FPGA bitstream) replays only with the same bytes.
fn firmware_dir() -> &'static Path {
    static DIR: OnceLock<PathBuf> = OnceLock::new();
    DIR.get_or_init(|| {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("emu-firmware");
        std::fs::create_dir_all(&dir).unwrap();
        for list in std::fs::read_dir(root.join("tools").join("emu")).unwrap() {
            let list = list.unwrap().path();
            if list.extension().is_none_or(|e| e != "firmware") {
                continue;
            }
            let text = std::fs::read_to_string(&list).unwrap();
            for line in text.lines() {
                let fields: Vec<&str> =
                    line.split('#').next().unwrap_or("").split_whitespace().collect();
                let [name, size] = fields[..] else { continue };
                let size = match size.strip_prefix("0x") {
                    Some(hex) => usize::from_str_radix(hex, 16),
                    None => size.parse(),
                }
                .unwrap();
                let mut data = Vec::with_capacity(size + 32);
                for i in 0.. {
                    if data.len() >= size {
                        break;
                    }
                    data.extend_from_slice(&Sha256::digest(format!("{name}:{i}").as_bytes()));
                }
                data.truncate(size);
                let path = dir.join(name);
                if std::fs::read(&path).is_ok_and(|d| d == data) {
                    continue;
                }
                let tmp = dir.join(format!("{name}.tmp"));
                std::fs::write(&tmp, &data).unwrap();
                std::fs::rename(&tmp, &path).unwrap();
            }
        }
        dir
    })
}

/// The arguments of the recorded command line.
fn args(fixture: &str) -> Vec<String> {
    fixture.lines().filter_map(|l| l.strip_prefix("# arg: ")).map(str::to_owned).collect()
}

/// The driver id of `-d DRIVER[:options]`.
fn driver(args: &[String]) -> Option<String> {
    let i = args.iter().position(|a| a == "-d")?;
    Some(args.get(i + 1)?.split(':').next()?.to_owned())
}

/// Runs oxrok on one fixture: its output, or why it failed.
fn replay(path: &Path) -> Result<(), String> {
    // Git may check text out with CRLF on Windows.
    let fixture = std::fs::read_to_string(path).map_err(|e| e.to_string())?.replace("\r\n", "\n");
    let var = match fixture.lines().next() {
        Some("# oxrok usb fixture 1") => "OXROK_USB_REPLAY",
        Some("# oxrok hid fixture 1") => "OXROK_HID_REPLAY",
        other => return Err(format!("not a USB or HID fixture: {other:?}")),
    };
    let out_path = path.with_extension("out");
    let expected = std::fs::read_to_string(&out_path)
        .map_err(|e| format!("{}: {e}", out_path.display()))?
        .replace("\r\n", "\n");
    let mut child = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(args(&fixture))
        .env(var, path)
        .env("OXROK_FIRMWARE_DIR", firmware_dir())
        .env_remove(if var == "OXROK_USB_REPLAY" { "OXROK_HID_REPLAY" } else { "OXROK_USB_REPLAY" })
        .env_remove("OXROK_USB_RECORD")
        .env_remove("OXROK_HID_RECORD")
        .env_remove("OXROK_SERIAL_RECORD")
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| e.to_string())?;
    let mut stdout = child.stdout.take().unwrap();
    let mut stderr = child.stderr.take().unwrap();
    let out = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = stdout.read_to_string(&mut s);
        s
    });
    let err = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = stderr.read_to_string(&mut s);
        s
    });
    let start = Instant::now();
    let status = loop {
        if let Some(st) = child.try_wait().map_err(|e| e.to_string())? {
            break st;
        }
        if start.elapsed() > LIMIT {
            let _ = child.kill();
            let _ = child.wait();
            return Err(format!("still running after {} s, killed", LIMIT.as_secs()));
        }
        std::thread::sleep(Duration::from_millis(20));
    };
    let got = out.join().unwrap().replace("\r\n", "\n");
    let err = err.join().unwrap();
    if !status.success() {
        return Err(format!("oxrok exited with {status}:\n{err}"));
    }
    // As diffcli.sh keeps it: the version line goes.
    let got = match got.split_once('\n') {
        Some((first, rest)) if first.starts_with("oxrok ") => rest.to_owned(),
        _ => got,
    };
    if got != expected {
        return Err(format!("output differs:\n--- recorded\n{expected}--- replayed\n{got}"));
    }
    Ok(())
}

#[test]
fn emulator_recordings_replay() {
    let paths = fixtures();
    let mut failures = Vec::new();
    let mut drivers = Vec::new();
    // In parallel: each is a process of its own.
    let runs: Vec<_> = paths
        .iter()
        .map(|p| {
            let p = p.clone();
            std::thread::spawn(move || replay(&p))
        })
        .collect();
    for (path, run) in paths.iter().zip(runs) {
        let text = std::fs::read_to_string(path).unwrap().replace("\r\n", "\n");
        drivers.extend(driver(&args(&text)));
        if let Err(e) = run.join().unwrap() {
            failures.push(format!("{}: {e}", path.display()));
        }
    }
    for d in RECORDED {
        if !drivers.iter().any(|x| x == d) {
            failures.push(format!("no emulator recording of {d}"));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}
