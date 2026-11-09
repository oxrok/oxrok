// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The ASIX OMEGA as libsigrok's `asix-omega-rtm-cli` meets it: not on USB, but
//! through ASIX's `omegartmcli`, a program the driver runs and reads the stdout of.
//! The model is a stand-in for that program. `oxrok-emu` run under the name
//! [`STANDIN`] (a link to it, first on `PATH`) is [`standin`]: it takes the
//! arguments the driver gives and writes what the program writes. The boundary is
//! where C has it: the process, its arguments, its three standard streams and its
//! exit.
//!
//! The program, and how sure this is. ASIX publishes the source of its Windows
//! software (`sigma-omega_src_2025-06-05.zip` from asix.tech, read 2026-11-08):
//! `SIGMA/omegartmcli.dpr` and `SIGMA/omegartmcliU.pas`, Delphi 7.
//!
//! - **Arguments.** `-version` (or `-V`) prints four lines and exits with status 1,
//!   whatever else is there (`halt(1)` right after the `WriteLn`s). A Delphi console
//!   program's `WriteLn` ends lines with CR LF. `-bin` makes the output binary,
//!   `-serial SN` picks a device: `StrToInt('$' + SN)`, a hex number, which with no
//!   bits in its third byte gets `0x01` there, and then `A6` on top of its low 24
//!   bits. A number that isn't hex raises Delphi's `EConvertError` ("'$SN' is not a
//!   valid integer value"). A parameter it doesn't know is a format error on stderr
//!   and status 2. C sends `-version` alone, and `-bin` with or without
//!   `-serial SN`; anything else is a violation here, as no C path sends it.
//! - **Errors** go to stderr as `omegartmcli: An error was encountered: ` and the
//!   exception's text, then status 1: no device is `Cannot open OMEGA.`, CR LF,
//!   `OMEGA was not found.` (`OmegaU.pas`, `OpenDevice`).
//! - **The stream.** Chunks of 6 bytes, three 16 bit words least significant byte
//!   first: the time since the chunk before, in 10 ns ticks (`tslen + 1` of a
//!   decoded vector, so 1 to 32768 and never 0), then two samples 5 ns apart. The
//!   samples of a chunk are at twice its time and one after, so a gap of `T` ticks
//!   repeats the last sample `2 * (T - 1)` times. Each `WriteFile` is a multiple of 6
//!   bytes and at most `3 * 16384`. The decoder thread hands over what it decoded
//!   every 100 ms or when its buffer of `3 * 65536` words is nearly full, so the
//!   real pace is bursts about 100 ms apart.
//! - **The end.** A `WriteFile` that fails raises `WriteFile to Output failed:` and
//!   ends the program with status 1, so a closed stdout ends it at its next write.
//!   The device's own end (the node counter's limit, about 2^30 vectors) ends it
//!   with "Gracefully closing..." on stderr and status 0. A USB error leaves the
//!   decoder thread looping with no device and the main loop waiting for data that
//!   never comes, writing nothing: such a program ends on no signal from its pipes.
//!
//! What is the C driver's word and not the source's: "The RTM CLI application
//! terminates when its stdin closes" (`api.c`). The source asks the console for
//! Ctrl-C or Ctrl-D, which a pipe isn't, so on Windows a closed stdin does
//! nothing; its author saw it under wine. By default the stand-in ends at a closed
//! stdin, as C's author saw, with "Gracefully closing..." and status 0;
//! `eof=ignore` takes the source's reading, where only the next write ends it.
//!
//! What the stand-in leaves out of the source: the decoder skips the first
//! in-sync vector and, after a FIFO overflow, resyncs with no mark in the stream
//! (`omegartmcliU.pas:389-397`, `435-449`), so a real stream can lose time with no
//! sign; and without `-serial` the program opens the `Omega.Serial` its settings
//! hold, the last one used (`OmegaU.pas:1166`), not simply the first OMEGA found.
//!
//! What is the stand-in's own: the samples (a Gray code count, see [`chunk`]); the
//! device's serial, `A6031234`; and wall time, which drives a stand-in where nothing
//! else does. Free running, it writes as fast as the pipe takes it, so its pace is
//! the host's reads; `pace=real` writes the bursts at the times their stamps say.
//! `stamp=1` at the real pace is 6 bytes each 10 ns, 600 MB/s, far beyond what the
//! OMEGA's FT2232H carries (some 40 MB/s): the device would overflow its FIFO long
//! before, so such a case is the host's limit, not a device's.
//!
//! An `LD_PRELOAD` the tool was run with (the oracle's shims, under sigrok-cli only)
//! would reach the stand-in too; it runs itself again without it, so both tools
//! meet the same program.
//!
//! The transaction log. Each run of the stand-in appends to the file
//! `EMU_STANDIN_LOG` names (`tools/diffcli.sh` makes one for each tool's run):
//!
//! - `exec ARGV stdin=KIND stdout=KIND stderr=KIND`, and ` fds=+N` when it was
//!   given N descriptors beyond the three; a kind is `pipe`, `null`, `tty`, `file`,
//!   `socket`, `char` or `closed`. `argv[0]` under `EMU_STANDIN_DIR` is written as
//!   `STANDIN/NAME`.
//! - `stdin closed` when the host closes the write end of a stdin pipe;
//!   `! violation: ...` for what the host did outside the protocol (bytes on stdin,
//!   arguments no C path sends, an unknown option).
//! - `exit N` as it exits by itself; nothing when a signal ends it.
//!
//! It holds a shared lock on that file while it runs, which is how the harness
//! tells that a stand-in outlived the tool that started it.
//!
//! Options come in `EMU_STANDIN_OPTIONS`, `KEY=VALUE` words apart, since the tool
//! under test, not the server, starts the stand-in. The server's half of the model
//! is a socket the stand-in touches at `mark=N`, for `--interrupt tcp-model`.

use std::ffi::OsString;
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crate::model::DeviceModel;
use crate::shells::tcp::{NoUsb, Port, Stream};
use crate::{Error, Result};

/// The model's name.
pub const NAME: &str = "asix-omega-rtm-cli";
/// What `oxrok-emu list` says.
pub const ABOUT: &str = "ASIX OMEGA through its omegartmcli program: a stand-in executable, \
     no USB; options in EMU_STANDIN_OPTIONS: bytes=N then=exit|quiet|hang exit=N \
     version-exit=N version-hang=1 banner=HEX serial=HEX stamp=N idle=1 piece=N gap=MS pace=real start=MS \
     stderr=N eof=ignore hang-max=MS mark=N";
/// The program's name, which C runs from `PATH` unless `OMEGARTMCLI` says otherwise.
pub const STANDIN: &str = "omegartmcli";

/// The stand-in's options, from the environment.
pub const OPTIONS_VAR: &str = "EMU_STANDIN_OPTIONS";
/// The file the stand-in logs to.
pub const LOG_VAR: &str = "EMU_STANDIN_LOG";
/// The directory of the links, written as `STANDIN` in the log.
pub const DIR_VAR: &str = "EMU_STANDIN_DIR";

/// The bytes of a chunk.
pub const CHUNK: usize = 6;
/// The most one `WriteFile` writes: `OUTBUFSIZE`.
pub const WRITE_MAX: usize = 3 * 16384;
/// How often the decoder thread hands over what it has: `Decode2Flush` after 100 ms.
pub const FLUSH: Duration = Duration::from_millis(100);
/// The device's serial number.
pub const SERIAL: u32 = 0xa603_1234;
/// The first chunk's stamp, which C ignores: any value does.
const FIRST_STAMP: u16 = 0x5a5a;
/// The first chunk's first sample, which C drops.
const FIRST_SAMPLE: u16 = 0xdead;
/// The stamps of the chunks after the first, in turn: adjacent chunks, short and
/// longer gaps.
const STAMPS: [u16; 16] = [1, 1, 2, 1, 3, 1, 1, 7, 1, 2, 1, 1, 1, 40, 1, 5];

/// The `-version` output, `omegartmcli.dpr`'s four `WriteLn`s.
pub const BANNER: &[u8] = b"omegartmcli.exe Omega Real-Time Mode\r\nVersion 2016-12-14\r\n\
Copyright (c) 1991-2016 ASIX s.r.o.\r\nEmail: support@asix.net\r\n";

/// What the stand-in does once `bytes=` are out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Then {
    /// Ends, with `exit=`.
    Exit,
    /// Writes nothing more and ends as ever: at a closed stdin.
    Quiet,
    /// Writes nothing more and ends on nothing, until `hang-max=`: the program
    /// after a USB error.
    Hang,
}

/// The stand-in's behaviour: independent knobs, so flags.
#[derive(Clone, Debug, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct Options {
    /// The most bytes of `-bin` output, then [`Options::then`]; none: for ever.
    pub bytes: Option<u64>,
    /// What comes after `bytes`.
    pub then: Then,
    /// The status of an end the stand-in chooses, in `-bin`.
    pub exit: u8,
    /// The status of `-version`: 1, as `halt(1)`.
    pub version_exit: u8,
    /// `-version` ends on nothing after its banner, until `hang-max=`.
    pub version_hang: bool,
    /// The `-version` output.
    pub banner: Vec<u8>,
    /// The device's serial number.
    pub serial: u32,
    /// The stamp of every chunk after the first, in place of the turn of `STAMPS`.
    pub stamp: Option<u16>,
    /// Inputs that never change.
    pub idle: bool,
    /// Writes of this many bytes at most, a slow pipe; not a multiple of 6 is not
    /// what the program does.
    pub piece: Option<usize>,
    /// A pause after each piece.
    pub gap: Duration,
    /// Bursts at the times their stamps say, every 100 ms.
    pub pace_real: bool,
    /// The time before the first byte: the FPGA's configuration.
    pub start: Duration,
    /// Bytes of noise on stderr first, as wine writes its own.
    pub stderr: usize,
    /// A closed stdin ends nothing: the source's reading.
    pub ignore_eof: bool,
    /// The longest the stand-in waits for an end that may never come.
    pub hang_max: Duration,
    /// After this many chunks, a touch of the server's socket; 0 is before the
    /// first byte (and `-version`'s hang).
    pub mark: Option<u64>,
}

impl Default for Options {
    fn default() -> Options {
        Options {
            bytes: None,
            then: Then::Exit,
            exit: 0,
            version_exit: 1,
            version_hang: false,
            banner: BANNER.to_vec(),
            serial: SERIAL,
            stamp: None,
            idle: false,
            piece: None,
            gap: Duration::ZERO,
            pace_real: false,
            start: Duration::ZERO,
            stderr: 0,
            ignore_eof: false,
            hang_max: Duration::from_secs(10),
            mark: None,
        }
    }
}

fn number<T: std::str::FromStr>(key: &str, value: &str) -> std::result::Result<T, String> {
    value.parse().map_err(|_| format!("{key}={value}: not a number"))
}

impl Options {
    /// Parses `KEY=VALUE` words.
    ///
    /// # Errors
    ///
    /// An unknown key or a value that isn't one.
    pub fn parse(text: &str) -> std::result::Result<Options, String> {
        let mut o = Options::default();
        for word in text.split_whitespace() {
            let Some((key, value)) = word.split_once('=') else {
                return Err(format!("option '{word}' is not KEY=VALUE"));
            };
            let ms = |v: &str| number::<u64>(key, v).map(Duration::from_millis);
            match key {
                "bytes" => o.bytes = Some(number(key, value)?),
                "then" => {
                    o.then = match value {
                        "exit" => Then::Exit,
                        "quiet" => Then::Quiet,
                        "hang" => Then::Hang,
                        _ => return Err(format!("then={value}: exit, quiet or hang")),
                    };
                }
                "exit" => o.exit = number(key, value)?,
                "version-exit" => o.version_exit = number(key, value)?,
                "version-hang" => o.version_hang = value == "1",
                "banner" => {
                    o.banner = crate::options::parse_hex_list(key, value, 0, 4096)
                        .map_err(|e| e.to_string())?
                        .concat();
                }
                "serial" => {
                    o.serial = u32::from_str_radix(value, 16)
                        .map_err(|_| format!("serial={value}: a hex number"))?;
                }
                "stamp" => o.stamp = Some(number(key, value)?),
                "idle" => o.idle = value == "1",
                "piece" => {
                    let n: usize = number(key, value)?;
                    if n == 0 {
                        return Err("piece=0: at least one byte".to_owned());
                    }
                    o.piece = Some(n);
                }
                "gap" => o.gap = ms(value)?,
                "pace" => {
                    if value != "real" {
                        return Err(format!("pace={value}: real"));
                    }
                    o.pace_real = true;
                }
                "start" => o.start = ms(value)?,
                "stderr" => o.stderr = number(key, value)?,
                "eof" => {
                    if value != "ignore" {
                        return Err(format!("eof={value}: ignore"));
                    }
                    o.ignore_eof = true;
                }
                "hang-max" => o.hang_max = ms(value)?,
                "mark" => o.mark = Some(number(key, value)?),
                _ => return Err(format!("{NAME} has no option {key}")),
            }
        }
        Ok(o)
    }
}

/// What the arguments ask for.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Mode {
    /// `-version`.
    Version,
    /// `-bin`, with the `-serial` argument if any.
    Bin(Option<String>),
    /// What no C path sends.
    Other,
}

/// The mode of the arguments after `argv[0]`, as C builds them (`api.c`, `scan`).
pub fn mode(args: &[String]) -> Mode {
    match args {
        [v] if v == "-version" => Mode::Version,
        [b] if b == "-bin" => Mode::Bin(None),
        [b, s, n] if b == "-bin" && s == "-serial" => Mode::Bin(Some(n.clone())),
        _ => Mode::Other,
    }
}

/// The device `-serial TEXT` names, as the program computes it, or its error's text.
///
/// # Errors
///
/// What `StrToInt('$' + TEXT)` raises: not a hex number, or one over 32 bits.
pub fn serial_number(text: &str) -> std::result::Result<u32, String> {
    let sn = if !text.is_empty() && text.bytes().all(|c| c.is_ascii_hexdigit()) {
        u32::from_str_radix(text, 16).ok()
    } else {
        None
    };
    let mut sn = sn.ok_or_else(|| format!("'${text}' is not a valid integer value"))?;
    if sn & 0x00ff_0000 == 0 {
        sn |= 0x0001_0000;
    }
    Ok((sn & 0x00ff_ffff) | 0xa600_0000)
}

/// A Gray code: one input changes from one sample to the next.
fn gray(n: u64) -> u16 {
    (n ^ (n >> 1)) as u16
}

/// Chunk `i` of the stream: its stamp and its two samples. The first chunk's stamp
/// and first sample mean nothing to C, which keeps its second sample alone.
pub fn chunk(index: u64, opts: &Options) -> [u8; CHUNK] {
    let (stamp, first, second) = if index == 0 {
        (FIRST_STAMP, FIRST_SAMPLE, 0)
    } else {
        let stamp = stamp(index, opts);
        if opts.idle { (stamp, 0, 0) } else { (stamp, gray(2 * index - 1), gray(2 * index)) }
    };
    let mut bytes = [0u8; CHUNK];
    bytes[0..2].copy_from_slice(&stamp.to_le_bytes());
    bytes[2..4].copy_from_slice(&first.to_le_bytes());
    bytes[4..6].copy_from_slice(&second.to_le_bytes());
    bytes
}

/// The stamp of chunk `index`, one after the first.
fn stamp(index: u64, opts: &Options) -> u16 {
    opts.stamp.unwrap_or(STAMPS[((index - 1) % STAMPS.len() as u64) as usize])
}

/// The time chunk `index` takes, in 10 ns ticks: the first takes none.
pub fn ticks(index: u64, opts: &Options) -> u64 {
    if index == 0 { 0 } else { u64::from(stamp(index, opts)) }
}

/// The server's half: a socket the stand-in touches at `mark=N`, which marks the
/// point an event armed with `tcp-model` waits for.
struct Touch;

impl Stream for Touch {
    fn received(&mut self, data: &[u8], port: &mut Port) {
        port.violation(format!("{} bytes from the stand-in, which sends none", data.len()));
    }

    fn connected(&mut self, port: &mut Port) {
        port.mark();
    }
}

fn no_options(options: &[(String, String)]) -> Result<()> {
    match options.first() {
        None => Ok(()),
        Some((k, _)) => Err(Error::Invalid(format!(
            "{NAME} has no option {k}: the stand-in's options go in {OPTIONS_VAR}"
        ))),
    }
}

/// The USB side, which is nothing.
///
/// # Errors
///
/// Any option: they go in the environment.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    no_options(options)?;
    Ok(Box::new(NoUsb::new(NAME)))
}

/// The socket of `mark=N`.
///
/// # Errors
///
/// Any option: they go in the environment.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    no_options(options)?;
    Ok(Box::new(Touch))
}

/// Where the stand-in logs: the file `EMU_STANDIN_LOG` names, under a shared lock for
/// as long as the process lives, or nowhere.
#[derive(Clone)]
struct Log(Option<Arc<Mutex<File>>>);

impl Log {
    fn open() -> Log {
        let Some(path) = std::env::var_os(LOG_VAR) else { return Log(None) };
        let file = OpenOptions::new().create(true).append(true).open(&path);
        let Ok(file) = file else {
            eprintln!("{STANDIN} (oxrok-emu stand-in): cannot open {}", Path::new(&path).display());
            return Log(None);
        };
        let _ = file.lock_shared();
        Log(Some(Arc::new(Mutex::new(file))))
    }

    /// One line, in one write, so that two stand-ins' lines don't mix.
    fn line(&self, text: &str) {
        if let Some(f) = &self.0 {
            let mut f = f.lock().expect("stand-in log");
            let _ = f.write_all(format!("{text}\n").as_bytes());
        }
    }

    fn violation(&self, text: &str) {
        eprintln!("{STANDIN} (oxrok-emu stand-in): {text}");
        self.line(&format!("! violation: {text}"));
    }
}

/// What descriptor `fd` is, from `/proc/self/fd`.
#[cfg(unix)]
fn fd_kind(fd: u32) -> &'static str {
    use std::os::unix::fs::{FileTypeExt as _, MetadataExt as _};
    let Ok(m) = std::fs::metadata(format!("/proc/self/fd/{fd}")) else { return "closed" };
    let t = m.file_type();
    if t.is_fifo() {
        "pipe"
    } else if t.is_socket() {
        "socket"
    } else if t.is_file() {
        "file"
    } else if t.is_char_device() {
        // /dev/null is 1:3; ttys are 4, 5 and 136 to 143.
        match (m.rdev() >> 8) & 0xfff {
            1 if m.rdev() & 0xff == 3 => "null",
            4 | 5 | 136..=143 => "tty",
            _ => "char",
        }
    } else {
        "other"
    }
}

#[cfg(not(unix))]
fn fd_kind(_fd: u32) -> &'static str {
    "?"
}

/// How many descriptors beyond the three the process was given.
fn extra_fds() -> usize {
    let Ok(dir) = std::fs::read_dir("/proc/self/fd") else { return 0 };
    let n = dir
        .filter_map(|e| e.ok()?.file_name().to_str()?.parse::<u32>().ok())
        .filter(|fd| *fd > 2)
        .count();
    // The listing's own descriptor.
    n.saturating_sub(1)
}

/// How an argument shows in the log.
fn shown(arg: &str) -> String {
    if arg.is_empty() || arg.chars().any(|c| c.is_whitespace() || c.is_control() || c == '"') {
        format!("{arg:?}")
    } else {
        arg.to_owned()
    }
}

/// The `exec` line.
fn exec_line(argv: &[String], fds: usize) -> String {
    let mut words: Vec<String> = Vec::new();
    if let Some(first) = argv.first() {
        let dir = std::env::var(DIR_VAR).unwrap_or_default();
        let first = match first.strip_prefix(&dir) {
            Some(rest) if !dir.is_empty() => format!("STANDIN{rest}"),
            _ => first.clone(),
        };
        words.push(shown(&first));
    }
    words.extend(argv.iter().skip(1).map(|a| shown(a)));
    let fds = if fds > 0 { format!(" fds=+{fds}") } else { String::new() };
    format!(
        "exec {} stdin={} stdout={} stderr={}{fds}",
        words.join(" "),
        fd_kind(0),
        fd_kind(1),
        fd_kind(2)
    )
}

/// The stand-in: runs as the program would with `raw` (`argv[0]` first) and
/// returns the exit status. A signal ends it as it would end the program.
pub fn standin(raw: &[OsString]) -> u8 {
    #[cfg(target_os = "linux")]
    if std::env::var_os("LD_PRELOAD").is_some() {
        use std::os::unix::process::CommandExt as _;
        let e = std::process::Command::new("/proc/self/exe")
            .arg0(raw.first().cloned().unwrap_or_default())
            .args(raw.iter().skip(1))
            .env_remove("LD_PRELOAD")
            .exec();
        eprintln!("{STANDIN} (oxrok-emu stand-in): cannot run again without LD_PRELOAD: {e}");
        return 2;
    }
    let fds = extra_fds();
    let log = Log::open();
    let argv: Vec<String> = raw.iter().map(|a| a.to_string_lossy().into_owned()).collect();
    log.line(&exec_line(&argv, fds));
    let options = match Options::parse(&std::env::var(OPTIONS_VAR).unwrap_or_default()) {
        Ok(o) => o,
        Err(e) => {
            log.violation(&e);
            return finish(&log, 2);
        }
    };
    let status = match mode(argv.get(1..).unwrap_or_default()) {
        Mode::Version => version(&options, &log),
        Mode::Bin(serial) => bin(&options, serial.as_deref(), &log),
        Mode::Other => {
            let rest = argv.get(1..).unwrap_or_default().iter().map(|a| shown(a));
            log.violation(&format!(
                "arguments no C path sends: {}",
                rest.collect::<Vec<_>>().join(" ")
            ));
            2
        }
    };
    finish(&log, status)
}

fn finish(log: &Log, status: u8) -> u8 {
    log.line(&format!("exit {status}"));
    status
}

/// Waits for an end that comes from outside, up to `hang-max`, then says so.
fn hang(o: &Options, log: &Log) -> u8 {
    thread::sleep(o.hang_max);
    log.line(&format!("hung for {} ms", o.hang_max.as_millis()));
    o.exit
}

/// `-version`: the banner, then the status, or a hang.
fn version(o: &Options, log: &Log) -> u8 {
    let mut out = io::stdout().lock();
    let _ = out.write_all(&o.banner);
    let _ = out.flush();
    drop(out);
    if o.version_hang {
        if o.mark == Some(0) {
            touch();
        }
        return hang(&Options { exit: o.version_exit, ..o.clone() }, log);
    }
    o.version_exit
}

/// The program's message for an error, and status 1.
fn error(text: &str) -> u8 {
    eprint!("omegartmcli: An error was encountered: {text}\r\n");
    1
}

/// The program's end of its own: `Gracefully closing...` for 0.
fn end(o: &Options) -> u8 {
    if o.exit == 0 {
        eprint!("Gracefully closing...\r\n");
        0
    } else {
        error(&format!("stand-in exit={}", o.exit));
        o.exit
    }
}

/// The host's side of stdin: closed or not.
struct Stdin {
    closed: AtomicBool,
}

/// Watches stdin: `stdin closed` at its end, a violation for bytes on it.
fn watch_stdin(log: &Log) -> Arc<Stdin> {
    let state = Arc::new(Stdin { closed: AtomicBool::new(false) });
    let (s, log) = (Arc::clone(&state), log.clone());
    let _ = thread::Builder::new().name("stdin".to_owned()).spawn(move || {
        let mut buf = [0u8; 256];
        let mut stdin = io::stdin().lock();
        loop {
            match stdin.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => log.violation(&format!("the host wrote {n} bytes to stdin")),
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(_) => break,
            }
        }
        log.line("stdin closed");
        s.closed.store(true, Ordering::SeqCst);
    });
    state
}

/// Touches the server's socket, which marks the model's point.
fn touch() {
    if let Ok(port) = std::env::var("EMU_TCP_PORT")
        && let Ok(port) = port.parse::<u16>()
    {
        let _ = TcpStream::connect(("127.0.0.1", port));
    }
}

/// `-bin`: the stream until the host ends it, or `bytes=` says otherwise.
fn bin(o: &Options, serial: Option<&str>, log: &Log) -> u8 {
    let mut err = io::stderr().lock();
    let noise = b"fixme:stand-in:noise as wine writes it\n";
    for _ in 0..o.stderr.div_ceil(noise.len()) {
        if err.write_all(noise).is_err() {
            break;
        }
    }
    drop(err);
    if let Some(text) = serial {
        match serial_number(text) {
            Ok(sn) if sn == o.serial => {}
            Ok(_) => return error("Cannot open OMEGA.\r\nOMEGA was not found."),
            Err(e) => return error(&e),
        }
    }
    let stdin = watch_stdin(log);
    let ended = || stdin.closed.load(Ordering::SeqCst) && !o.ignore_eof;
    if o.mark == Some(0) {
        touch();
    }
    thread::sleep(o.start);
    let mut out = io::stdout().lock();
    let mut i = 0u64;
    let mut sent = 0u64;
    let began = Instant::now();
    let mut due = 0u64; // ticks
    loop {
        if ended() {
            return end(&Options { exit: 0, ..o.clone() });
        }
        let room = o.bytes.map_or(u64::MAX, |b| b - sent);
        if room == 0 {
            drop(out);
            return match o.then {
                Then::Exit => end(o),
                Then::Quiet => quiet(o, &stdin, log),
                Then::Hang => hang(o, log),
            };
        }
        // One burst: what a write of the program holds, or 100 ms of stamps.
        let mut burst = Vec::with_capacity(WRITE_MAX);
        if o.pace_real {
            let window = due + 10_000_000; // 100 ms in 10 ns ticks
            while due < window && burst.len() + CHUNK <= WRITE_MAX {
                burst.extend_from_slice(&chunk(i, o));
                due += ticks(i, o);
                i += 1;
            }
            let at = Duration::from_nanos(due.saturating_mul(10));
            if let Some(wait) = at.checked_sub(began.elapsed()) {
                thread::sleep(wait.min(FLUSH));
            }
        } else {
            while burst.len() + CHUNK <= WRITE_MAX {
                burst.extend_from_slice(&chunk(i, o));
                i += 1;
            }
        }
        let first = i - (burst.len() / CHUNK) as u64;
        burst.truncate(room.min(burst.len() as u64) as usize);
        if let Err(e) = write_out(&mut out, &burst, o) {
            drop(out);
            return closed_stdout(o, &stdin, &e);
        }
        sent += burst.len() as u64;
        if let Some(m) = o.mark
            && m > 0
            && first <= m
            && m < i
        {
            touch();
        }
    }
}

/// Writes `burst`, in `piece=` bytes with `gap=` pauses when asked.
fn write_out(out: &mut impl Write, burst: &[u8], o: &Options) -> io::Result<()> {
    match o.piece {
        None => {
            out.write_all(burst)?;
            out.flush()
        }
        Some(n) => {
            for p in burst.chunks(n) {
                out.write_all(p)?;
                out.flush()?;
                thread::sleep(o.gap);
            }
            Ok(())
        }
    }
}

/// After a failed write: the program's end, once a closing stdin had its say. C
/// closes stdin and then stdout, so whether the write or the reader sees the end
/// first is the pace's: waiting a little for stdin, and ending as a closed stdin
/// ends things when it closed, keeps the log the same.
fn closed_stdout(o: &Options, stdin: &Stdin, e: &io::Error) -> u8 {
    let began = Instant::now();
    while !stdin.closed.load(Ordering::SeqCst) && began.elapsed() < Duration::from_secs(1) {
        thread::sleep(Duration::from_millis(2));
    }
    if stdin.closed.load(Ordering::SeqCst) && !o.ignore_eof {
        return end(&Options { exit: 0, ..o.clone() });
    }
    error(&format!("WriteFile to Output failed: {}", e.kind()))
}

/// Writes nothing and ends at a closed stdin, as ever, up to `hang-max`.
fn quiet(o: &Options, stdin: &Stdin, log: &Log) -> u8 {
    let began = Instant::now();
    while began.elapsed() < o.hang_max {
        if stdin.closed.load(Ordering::SeqCst) && !o.ignore_eof {
            return end(&Options { exit: 0, ..o.clone() });
        }
        thread::sleep(Duration::from_millis(2));
    }
    log.line(&format!("quiet for {} ms", o.hang_max.as_millis()));
    o.exit
}

#[cfg(test)]
mod tests {
    use super::*;

    fn words(args: &[&str]) -> Vec<String> {
        args.iter().map(|a| (*a).to_owned()).collect()
    }

    #[test]
    fn the_arguments_c_sends_and_nothing_else() {
        assert_eq!(mode(&words(&["-version"])), Mode::Version);
        assert_eq!(mode(&words(&["-bin"])), Mode::Bin(None));
        assert_eq!(mode(&words(&["-bin", "-serial", "031234"])), Mode::Bin(Some("031234".into())));
        for other in [&[][..], &["-V"], &["-bin", "-serial"], &["-serial", "1", "-bin"], &["-h"]] {
            assert_eq!(mode(&words(other)), Mode::Other, "{other:?}");
        }
    }

    /// `omegartmcli.dpr`: `StrToInt('$'+...)`, `0x01` in an empty third byte, `A6`
    /// over the low 24 bits.
    #[test]
    fn a_serial_as_the_program_reads_it() {
        assert_eq!(serial_number("031234"), Ok(0xa603_1234));
        assert_eq!(serial_number("A6031234"), Ok(0xa603_1234));
        assert_eq!(serial_number("a6031234"), Ok(0xa603_1234));
        // The top byte goes, whatever it is.
        assert_eq!(serial_number("B6031234"), Ok(0xa603_1234));
        // Four digits are 0x01 in the third byte to the program; C's 03 prefix
        // is what makes them an OMEGA's.
        assert_eq!(serial_number("1234"), Ok(0xa601_1234));
        assert_eq!(serial_number("zz1234"), Err("'$zz1234' is not a valid integer value".into()));
        assert!(serial_number("").is_err());
        assert!(serial_number("123456789").is_err());
    }

    #[test]
    fn options_parse_and_refuse() {
        let o =
            Options::parse("bytes=12 then=hang exit=3 stamp=32768 idle=1 piece=7 gap=5").unwrap();
        assert_eq!(o.bytes, Some(12));
        assert_eq!(o.then, Then::Hang);
        assert_eq!(o.exit, 3);
        assert_eq!(o.stamp, Some(32768));
        assert!(o.idle);
        assert_eq!(o.piece, Some(7));
        assert_eq!(o.gap, Duration::from_millis(5));
        assert_eq!(Options::parse("").unwrap(), Options::default());
        assert_eq!(Options::parse("banner=0d0a").unwrap().banner, b"\r\n");
        for (bad, says) in [
            ("nope=1", "has no option nope"),
            ("bytes", "not KEY=VALUE"),
            ("bytes=x", "not a number"),
            ("then=later", "exit, quiet or hang"),
            ("piece=0", "at least one byte"),
            ("pace=fast", "real"),
            ("serial=xyz", "a hex number"),
        ] {
            let e = Options::parse(bad).unwrap_err();
            assert!(e.contains(says), "{bad}: {e}");
        }
    }

    #[test]
    fn the_stream_starts_with_a_chunk_c_keeps_one_sample_of() {
        let o = Options::default();
        assert_eq!(chunk(0, &o), [0x5a, 0x5a, 0xad, 0xde, 0, 0]);
        // Gray: 1, 3 then 2, 6, ...
        assert_eq!(chunk(1, &o), [1, 0, 1, 0, 3, 0]);
        assert_eq!(chunk(2, &o), [1, 0, 2, 0, 6, 0]);
        assert_eq!(chunk(3, &o), [2, 0, 7, 0, 5, 0]);
        assert_eq!(ticks(0, &o), 0);
        assert_eq!(ticks(14, &o), 40);
        // Every stamp of the turn is one the program sends: 1 to 32768.
        assert!(STAMPS.iter().all(|s| (1..=32768).contains(s)));
        let idle = Options { stamp: Some(32768), idle: true, ..Options::default() };
        assert_eq!(chunk(5, &idle), [0, 0x80, 0, 0, 0, 0]);
    }

    #[test]
    fn the_banner_is_the_programs() {
        let text = std::str::from_utf8(BANNER).unwrap();
        assert_eq!(text.split('\n').nth(1), Some("Version 2016-12-14\r"));
        assert!(text.ends_with("support@asix.net\r\n"));
    }

    #[test]
    fn the_socket_marks_and_takes_nothing() {
        let mut t = Touch;
        let mut p = Port::default();
        t.connected(&mut p);
        assert!(p.marked());
        let mut p = Port::default();
        t.received(b"x", &mut p);
        assert_eq!(p.violations().drain().len(), 1);
    }

    #[test]
    fn the_server_takes_no_options() {
        assert!(build(&[]).is_ok());
        assert!(build_tcp(&[]).is_ok());
        let e = build(&[("bytes".into(), "1".into())]).err().unwrap().to_string();
        assert!(e.contains(OPTIONS_VAR), "{e}");
    }

    #[test]
    fn arguments_show_plainly_unless_they_need_quotes() {
        assert_eq!(shown("-bin"), "-bin");
        assert_eq!(shown("a b"), "\"a b\"");
        assert_eq!(shown(""), "\"\"");
    }
}
