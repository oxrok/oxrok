// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! `conn=tcp-raw/HOST/PORT` through the command line, against a fake on a thread.
//! The emulator carries no TCP, so this is all that holds these paths in CI.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::process::{Command, Output, Stdio};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// A run that takes longer has hung.
const BOUND: Duration = Duration::from_secs(60);

/// oxrok with these arguments, killed and failed at [`BOUND`].
fn oxrok(args: &[&str]) -> Output {
    let mut child = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(args)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("run oxrok");
    let began = Instant::now();
    loop {
        if child.try_wait().expect("a wait").is_some() {
            return child.wait_with_output().expect("its output");
        }
        if began.elapsed() > BOUND {
            let _ = child.kill();
            let out = child.wait_with_output().expect("its output");
            panic!("oxrok {args:?} still running: {}", String::from_utf8_lossy(&out.stderr));
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// What a fake does with a request: the bytes to answer, or to end.
#[derive(Clone, Copy, Debug)]
enum Reply {
    Bytes(&'static [u8]),
    Silence,
    Close,
}

/// Serves connections for ever, for the life of the test binary. Each read is one
/// request, as a board takes them. Returns the `conn=` and what came, by request.
fn fake(answer: fn(&[u8]) -> Reply) -> (String, Arc<Mutex<Vec<Vec<u8>>>>) {
    let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
    let port = listener.local_addr().expect("an address").port();
    let seen = Arc::new(Mutex::new(Vec::new()));
    let log = Arc::clone(&seen);
    std::thread::spawn(move || {
        for stream in listener.incoming() {
            let Ok(stream) = stream else { return };
            let log = Arc::clone(&log);
            std::thread::spawn(move || serve(stream, answer, &log));
        }
    });
    (format!("tcp-raw/127.0.0.1/{port}"), seen)
}

fn serve(mut stream: TcpStream, answer: fn(&[u8]) -> Reply, log: &Mutex<Vec<Vec<u8>>>) {
    let mut request = [0u8; 64];
    loop {
        let n = match stream.read(&mut request) {
            Ok(0) | Err(_) => return,
            Ok(n) => n,
        };
        log.lock().unwrap_or_else(std::sync::PoisonError::into_inner).push(request[..n].to_vec());
        match answer(&request[..n]) {
            Reply::Bytes(bytes) => {
                if stream.write_all(bytes).is_err() {
                    return;
                }
            }
            Reply::Silence => {}
            Reply::Close => return,
        }
    }
}

fn eth484(request: &[u8]) -> Reply {
    match request {
        [0x10] => Reply::Bytes(&[20, 2, 3]),
        [0x77] => Reply::Bytes(&[0, 0x04, 0xa3, 0x12, 0x34, 0x56]),
        [0x24] => Reply::Bytes(&[0x01, 0x80]),
        [0x25] => Reply::Bytes(&[0x00, 0x81]),
        [0x32, 2] => Reply::Bytes(&[0x02, 0x01]),
        [0x78] => Reply::Bytes(&[125]),
        [0x20 | 0x21, _, 0] | [0x23, _, _] => Reply::Bytes(&[0]),
        _ => Reply::Close,
    }
}

fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

fn seen(log: &Mutex<Vec<Vec<u8>>>) -> Vec<Vec<u8>> {
    log.lock().unwrap_or_else(std::sync::PoisonError::into_inner).clone()
}

#[test]
fn a_scan_finds_a_board_and_says_what_it_is() {
    let (conn, log) = fake(eth484);
    let d = format!("devantech-eth008:conn={conn}");
    let out = oxrok(&["-d", &d, "--scan"]);
    assert!(out.status.success(), "{}", text(&out.stderr));
    let stdout = text(&out.stdout);
    assert!(
        stdout.contains(&format!(
            "devantech-eth008:conn={conn} - Devantech ETH484 HW2 FW3 [S/N: 0004a3123456]"
        )),
        "{stdout}"
    );
    assert_eq!(seen(&log), [vec![0x10], vec![0x77]]);
}

#[test]
fn gets_read_what_the_board_says() {
    let (conn, _) = fake(eth484);
    let d = format!("devantech-eth008:conn={conn}");
    for (group, key, want) in [
        ("DO1", "enabled", "true"),
        ("DO2", "enabled", "false"),
        ("DO16", "enabled", "true"),
        ("DI8", "enabled", "true"),
        ("AI2", "voltage", "513"),
        ("Vsupply", "voltage", "12.5"),
    ] {
        let out = oxrok(&["-d", &d, "-g", group, "--get", key]);
        assert!(out.status.success(), "{group}: {}", text(&out.stderr));
        assert_eq!(text(&out.stdout).trim(), want, "{group}");
    }
}

#[test]
fn sets_send_their_requests() {
    let (conn, log) = fake(eth484);
    let d = format!("devantech-eth008:conn={conn}");
    let out = oxrok(&["-d", &d, "-c", "enabled=on", "--set"]);
    assert!(out.status.success(), "{}", text(&out.stderr));
    let out = oxrok(&["-d", &d, "-g", "DO9", "-c", "enabled=off", "--set"]);
    assert!(out.status.success(), "{}", text(&out.stderr));
    let log = seen(&log);
    assert!(log.contains(&vec![0x23, 0x0f, 0xff]), "{log:?}");
    assert!(log.contains(&vec![0x21, 9, 0]), "{log:?}");
}

/// Where C's blocking `recv` never returns.
#[test]
fn a_silent_board_ends() {
    let (conn, log) = fake(|_| Reply::Silence);
    let d = format!("devantech-eth008:conn={conn}");
    let out = oxrok(&["-d", &d, "--scan"]);
    assert!(out.status.success(), "{}", text(&out.stderr));
    assert!(!text(&out.stdout).contains("Devantech"), "{}", text(&out.stdout));
    assert_eq!(seen(&log), [vec![0x10]]);
}

#[test]
fn a_board_silent_in_a_get_fails_it() {
    fn silent_get(request: &[u8]) -> Reply {
        if request == [0x24] { Reply::Silence } else { eth484(request) }
    }
    let (conn, _) = fake(silent_get);
    let d = format!("devantech-eth008:conn={conn}");
    let out = oxrok(&["-d", &d, "-g", "DO1", "--get", "enabled"]);
    assert!(!out.status.success());
    assert!(
        text(&out.stderr).contains("Failed to get 'enabled': data is invalid"),
        "{}",
        text(&out.stderr)
    );
}

#[test]
fn a_board_that_closes_is_a_disconnect() {
    fn close_on_get(request: &[u8]) -> Reply {
        if request == [0x24] { Reply::Close } else { eth484(request) }
    }
    let (conn, _) = fake(close_on_get);
    let d = format!("devantech-eth008:conn={conn}");
    let out = oxrok(&["-d", &d, "-g", "DO1", "--get", "enabled"]);
    assert!(!out.status.success());
    assert!(
        text(&out.stderr).contains("Failed to get 'enabled': device disconnected"),
        "{}",
        text(&out.stderr)
    );
}

#[test]
fn nothing_listening_is_no_device() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
    let port = listener.local_addr().expect("an address").port();
    drop(listener);
    let d = format!("devantech-eth008:conn=tcp-raw/127.0.0.1/{port}");
    let out = oxrok(&["-d", &d, "--scan"]);
    assert!(out.status.success(), "{}", text(&out.stderr));
    assert!(text(&out.stderr).contains("Failed to connect to 127.0.0.1"), "{}", text(&out.stderr));
}

fn korad(request: &[u8]) -> Reply {
    match request {
        b"*IDN?" => Reply::Bytes(b"KORADKA3005PV2.0"),
        b"IOUT1?" => Reply::Bytes(b"0.000"),
        b"ISET1?" => Reply::Bytes(b"1.000"),
        b"VOUT1?" => Reply::Bytes(b"00.00"),
        b"VSET1?" => Reply::Bytes(b"05.00"),
        b"STATUS?" => Reply::Bytes(&[0x40]),
        _ => Reply::Silence,
    }
}

/// Any serial driver takes the transport. Where sigrok-cli's `recv` doesn't end
/// this driver's read until a silence.
#[test]
fn another_driver_scans_over_tcp() {
    let (conn, log) = fake(korad);
    let d = format!("korad-kaxxxxp:conn={conn}");
    let out = oxrok(&["-d", &d, "--scan"]);
    assert!(out.status.success(), "{}", text(&out.stderr));
    assert!(
        text(&out.stdout).contains("Korad KA3005P"),
        "{}{}",
        text(&out.stdout),
        text(&out.stderr)
    );
    assert_eq!(seen(&log)[0], b"*IDN?");
}

/// A SCPI resource that won't open finds nothing, and says why.
#[test]
fn a_scpi_resource_that_will_not_open_says_why() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
    let port = listener.local_addr().expect("an address").port();
    drop(listener);
    let conn = format!("tcp-raw/127.0.0.1/{port}");
    for (conn, says) in [
        (conn.as_str(), format!("Couldn't open SCPI device {conn}: ")),
        ("usbtmc/9.9", "Couldn't open SCPI device usbtmc/9.9: ".to_owned()),
        ("/dev/no-such-tty", "Couldn't open SCPI device /dev/no-such-tty: ".to_owned()),
    ] {
        let spec = format!("rigol-ds:conn={conn}");
        let out = oxrok(&["-d", &spec, "--scan"]);
        let err = String::from_utf8_lossy(&out.stderr);
        assert!(out.status.success(), "{conn}: {err}");
        assert!(err.contains(&says), "{conn}: {err}");
    }
}

/// A stop and continue ends a read on a socket with a timeout with EINTR, which is
/// neither a failure nor silence: a shell's Ctrl-Z, or a machine that checkpoints. A
/// stop can miss under load, and that passes, so each goes round a few times.
#[cfg(unix)]
mod stops {
    use super::*;

    /// What `Scpi` asks of a read, `src/scpi/mod.rs`'s `CHUNK`.
    const READ_SIZE: usize = 4096;

    fn signal(name: &str, pid: u32) {
        let done = Command::new("kill").arg(name).arg(pid.to_string()).status();
        assert!(done.expect("kill").success(), "kill {name}");
    }

    /// One stop of `child` that lasts 20 ms.
    fn stall(child: &std::process::Child) {
        signal("-STOP", child.id());
        std::thread::sleep(Duration::from_millis(20));
        signal("-CONT", child.id());
    }

    fn accept(listener: &TcpListener) -> TcpStream {
        let began = Instant::now();
        loop {
            match listener.accept() {
                Ok((stream, _)) => {
                    stream.set_nonblocking(false).expect("blocking");
                    stream.set_read_timeout(Some(BOUND)).expect("a timeout");
                    return stream;
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    assert!(began.elapsed() < BOUND, "no connection came");
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(e) => panic!("{e}"),
            }
        }
    }

    /// oxrok on a new listener, as `-d DRIVER:conn=...` with `args`.
    fn start(driver: &str, args: &[&str]) -> (std::process::Child, TcpListener) {
        let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
        listener.set_nonblocking(true).expect("not blocking");
        let port = listener.local_addr().expect("an address").port();
        let spec = format!("{driver}:conn=tcp-raw/127.0.0.1/{port}");
        let child = Command::new(env!("CARGO_BIN_EXE_oxrok"))
            .args(["-d", &spec])
            .args(args)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .expect("run oxrok");
        (child, listener)
    }

    fn finish(mut child: std::process::Child) -> Output {
        let began = Instant::now();
        while child.try_wait().expect("a wait").is_none() {
            assert!(began.elapsed() < BOUND, "oxrok still running");
            std::thread::sleep(Duration::from_millis(5));
        }
        child.wait_with_output().expect("its output")
    }

    /// `hp-59306a` reads for 100 ms after a command, to see the line close.
    fn set_up_to_the_command() -> (std::process::Child, TcpStream, TcpStream) {
        let (child, listener) = start("hp-59306a", &["--config", "enabled=on", "--set"]);
        let scan = accept(&listener);
        let mut open = accept(&listener);
        let mut command = [0u8; 8];
        open.read_exact(&mut command).expect("the command");
        assert_eq!(&command, b"A123456\n");
        (child, scan, open)
    }

    #[test]
    fn a_stop_in_a_read_is_no_failure() {
        for _ in 0..5 {
            let (child, _scan, _open) = set_up_to_the_command();
            stall(&child);
            let out = finish(child);
            let err = String::from_utf8_lossy(&out.stderr);
            assert!(out.status.success(), "{err}");
            assert_eq!(err, "");
        }
    }

    /// Stops closer together than the wait must not start it again each time, or it
    /// never ends. Far past the wait's 100 ms, with the stops going on.
    #[test]
    fn stops_again_and_again_do_not_hold_a_read_for_ever() {
        let (mut child, _scan, _open) = set_up_to_the_command();
        let began = Instant::now();
        while child.try_wait().expect("a wait").is_none() {
            if began.elapsed() > Duration::from_secs(10) {
                let _ = child.kill();
                panic!("each stop started the wait again");
            }
            stall(&child);
        }
        assert!(child.wait_with_output().expect("its output").status.success());
    }

    /// A raw answer of more than a read holds a read's worth and then waits. A stop
    /// there is no silence, which would end the answer at the read's size.
    #[test]
    fn a_stop_in_a_long_answer_does_not_end_it() {
        for _ in 0..5 {
            let (child, listener) = start("rigol-ds", &["--scan"]);
            let mut stream = accept(&listener);
            let mut query = [0u8; 6];
            stream.read_exact(&mut query).expect("the query");
            assert_eq!(&query, b"*IDN?\n");
            let head = format!("Rigol Technologies,DS1102E,{:-<1$}", "", READ_SIZE - 27);
            assert_eq!(head.len(), READ_SIZE);
            stream.write_all(head.as_bytes()).expect("the head");
            std::thread::sleep(Duration::from_millis(10));
            stall(&child);
            stream.write_all(b",00.02.06.00.01\n").expect("the tail");
            let out = finish(child);
            let said = String::from_utf8_lossy(&out.stdout);
            assert!(
                said.contains("00.02.06.00.01"),
                "{said} {}",
                String::from_utf8_lossy(&out.stderr)
            );
        }
    }
}
