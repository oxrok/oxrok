// Counterpart in sigrok:
//   libsigrok/src/hardware/asix-omega-rtm-cli/api.c,
//   libsigrok/src/hardware/asix-omega-rtm-cli/protocol.c,
//   Copyright (C) 2021 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The vendor's program as a child process: `-version` run to its end with a
//! bound, and `-bin` read through a thread, closed as C closes it and then ended
//! for sure.
//!
//! C's `omega_rtm_cli_close` closes the child's stdin and stdout and calls
//! `g_spawn_close_pid`, which does nothing on Unix: the child is never waited for
//! (a zombie until sigrok-cli exits) and never killed (a program that doesn't end
//! by itself outlives sigrok-cli). Here a close shuts stdin, lets the reader drop
//! stdout at its next read, gives the program [`CLOSE_GRACE`] to end, then kills
//! it, and always waits for it. A dropped [`Child`] does the same.

use std::ffi::OsString;
use std::io::{self, Read};
use std::process::{ChildStdin, Command, ExitStatus, Stdio};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender};
use std::thread;
use std::time::{Duration, Instant};

/// How long a closed program has to end before it is killed. It ends at a closed
/// stdout at its next write, at most 100 ms on (it writes what it decoded at
/// least that often), and, by C's author, at a closed stdin.
pub const CLOSE_GRACE: Duration = Duration::from_secs(2);
/// The most bytes a read hands over.
const READ_SIZE: usize = 64 * 1024;
/// How many reads may wait for the capture.
const READS_AHEAD: usize = 16;
/// The most of a `-version` output kept: a program that writes more is not the one
/// meant, and the rest is read and dropped.
const VERSION_KEEP: usize = 64 * 1024;
/// How often a wait looks at a child.
const LOOK: Duration = Duration::from_millis(5);

/// The program `OMEGARTMCLI` names, else `omegartmcli` from `PATH` (`api.c`).
pub fn program() -> OsString {
    match std::env::var_os("OMEGARTMCLI") {
        Some(p) if !p.is_empty() => p,
        _ => OsString::from("omegartmcli"),
    }
}

/// A `Command` for the program: its own process group on Unix, so a terminal's
/// Ctrl-C reaches oxrok, which ends the capture and then the program, and not the
/// program first.
fn command(exe: &OsString, args: &[&str]) -> Command {
    let mut cmd = Command::new(exe);
    cmd.args(args);
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt as _;
        cmd.process_group(0);
    }
    cmd
}

/// How a `-version` run failed.
#[derive(Debug)]
pub enum VersionError {
    /// No such program: no device, in silence.
    NotFound,
    /// It could not be run.
    Spawn(io::Error),
    /// It ran past the bound and was killed.
    Timeout(Duration),
}

/// Runs `exe -version` with stdin and stderr on the null device, as
/// `g_spawn_sync` does, and returns its stdout once it has ended and closed it.
/// Its exit status means nothing: the program exits with 1 after its banner.
///
/// # Errors
///
/// No program, one that can't run, or one still running after `bound`.
pub fn run_version(exe: &OsString, bound: Duration) -> Result<Vec<u8>, VersionError> {
    let mut child = command(exe, &["-version"])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .map_err(|e| {
            if e.kind() == io::ErrorKind::NotFound {
                VersionError::NotFound
            } else {
                VersionError::Spawn(e)
            }
        })?;
    let mut stdout = child.stdout.take().expect("piped stdout");
    let (tx, rx) = mpsc::sync_channel(1);
    let _ = thread::Builder::new().name("omegartmcli -version".into()).spawn(move || {
        let mut kept = Vec::new();
        let mut buf = [0u8; 4096];
        loop {
            match stdout.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    let room = VERSION_KEEP.saturating_sub(kept.len());
                    kept.extend_from_slice(&buf[..n.min(room)]);
                }
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(_) => break,
            }
        }
        let _ = tx.send(kept);
    });
    let began = Instant::now();
    let mut output = None;
    let mut exited = false;
    while output.is_none() || !exited {
        if output.is_none() {
            match rx.recv_timeout(LOOK) {
                Ok(o) => output = Some(o),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => output = Some(Vec::new()),
            }
        } else {
            thread::sleep(LOOK);
        }
        if !exited {
            exited = matches!(child.try_wait(), Ok(Some(_)) | Err(_));
        }
        if began.elapsed() >= bound && !(exited && output.is_some()) {
            let _ = child.kill();
            let _ = child.wait();
            return Err(VersionError::Timeout(bound));
        }
    }
    Ok(output.unwrap_or_default())
}

/// What a read of the program's stdout gave.
#[derive(Debug, PartialEq, Eq)]
pub enum Got {
    /// Bytes.
    Data(Vec<u8>),
    /// Nothing within the time.
    Nothing,
    /// The end of its stdout, with the read error if one ended it.
    End(Option<String>),
}

/// A program's exit status, as far as the driver needs it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Exit {
    /// Status 0.
    pub success: bool,
    /// How it ended, in words: `exit status: 1`, `signal: 9 (SIGKILL)`.
    pub text: String,
}

impl From<ExitStatus> for Exit {
    fn from(status: ExitStatus) -> Exit {
        Exit { success: status.success(), text: status.to_string() }
    }
}

/// How the program ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Ended {
    /// It ended by itself, with this status.
    Exited(Exit),
    /// It was still running after [`CLOSE_GRACE`] and was killed.
    Killed,
}

/// A source of the program's stream: the program, or a test's fake.
pub trait Stream: Send {
    /// Reads what there is, waiting up to `timeout`.
    fn read(&mut self, timeout: Duration) -> Got;
    /// Waits up to `timeout` for the program to end by itself, as after its stdout
    /// closed: its status if it did.
    fn exited(&mut self, timeout: Duration) -> Option<Exit>;
    /// Ends it: stdin closed, stdout dropped, a grace, a kill, a wait.
    fn close(self: Box<Self>) -> Ended;
}

/// The running program, `-bin`.
pub struct Child {
    process: std::process::Child,
    stdin: Option<ChildStdin>,
    rx: Option<Receiver<Got>>,
    done: bool,
}

impl Child {
    /// Starts `exe` with `args`, stdin and stdout piped and stderr on the null
    /// device, as `g_spawn_async_with_pipes` with `G_SPAWN_STDERR_TO_DEV_NULL`.
    ///
    /// # Errors
    ///
    /// The program can't be run.
    pub fn spawn(exe: &OsString, args: &[&str]) -> io::Result<Child> {
        let mut child = command(exe, args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()?;
        let stdin = child.stdin.take();
        let stdout = child.stdout.take().expect("piped stdout");
        let (tx, rx) = mpsc::sync_channel(READS_AHEAD);
        let spawned = thread::Builder::new()
            .name("omegartmcli reader".into())
            .spawn(move || read_all(stdout, &tx));
        if let Err(e) = spawned {
            let _ = child.kill();
            let _ = child.wait();
            return Err(e);
        }
        Ok(Child { process: child, stdin, rx: Some(rx), done: false })
    }

    /// The process id, for tests.
    pub fn id(&self) -> u32 {
        self.process.id()
    }

    fn end(&mut self) -> Ended {
        if self.done {
            return Ended::Killed;
        }
        self.done = true;
        // C's order: stdin first, then stdout.
        drop(self.stdin.take());
        drop(self.rx.take());
        if let Some(status) = wait_for(&mut self.process, CLOSE_GRACE) {
            return Ended::Exited(status.into());
        }
        log::warn!(
            "The vendor application was still running {} s after its pipes closed: killed.",
            CLOSE_GRACE.as_secs()
        );
        let _ = self.process.kill();
        let _ = self.process.wait();
        Ended::Killed
    }
}

/// Waits up to `timeout` for `child` to end.
fn wait_for(child: &mut std::process::Child, timeout: Duration) -> Option<ExitStatus> {
    let began = Instant::now();
    loop {
        match child.try_wait() {
            Ok(Some(status)) => return Some(status),
            Ok(None) => {}
            Err(_) => return None,
        }
        if began.elapsed() >= timeout {
            return None;
        }
        thread::sleep(LOOK);
    }
}

/// The reader: hands each read over, and ends at the stdout's end or once the
/// capture has gone, dropping the stdout, which ends the program at its next write
/// as C's close of it does.
fn read_all(mut stdout: impl Read, tx: &SyncSender<Got>) {
    loop {
        let mut buf = vec![0u8; READ_SIZE];
        match stdout.read(&mut buf) {
            Ok(0) => {
                let _ = tx.send(Got::End(None));
                return;
            }
            Ok(n) => {
                buf.truncate(n);
                if tx.send(Got::Data(buf)).is_err() {
                    return;
                }
            }
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => {
                let _ = tx.send(Got::End(Some(e.to_string())));
                return;
            }
        }
    }
}

impl Stream for Child {
    fn read(&mut self, timeout: Duration) -> Got {
        let Some(rx) = &self.rx else { return Got::End(None) };
        match rx.recv_timeout(timeout) {
            Ok(got) => got,
            Err(RecvTimeoutError::Timeout) => Got::Nothing,
            Err(RecvTimeoutError::Disconnected) => Got::End(None),
        }
    }

    fn exited(&mut self, timeout: Duration) -> Option<Exit> {
        wait_for(&mut self.process, timeout).map(Exit::from)
    }

    fn close(mut self: Box<Self>) -> Ended {
        self.end()
    }
}

impl Drop for Child {
    fn drop(&mut self) {
        let _ = self.end();
    }
}
