// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A sysfs carrier: a model that is a tree of files, for the drivers that read
//! and write the kernel's own files (hwmon, `/sys/class/gpio`, an EEPROM) rather
//! than a device. `oxrok-emu serve MODEL --sysfs DIR` builds the model's tree in
//! `DIR` and serves it; the tools run in a user and mount namespace with
//! `DIR/class` bound over `/sys/class` (`tools/emu-ns.sh`, `EMU_WRAP` in
//! `tools/diffcli.sh`), so what C opens at its hard-coded paths is the model.
//!
//! A model implements [`Tree`], as a TCP one implements `tcp::Stream`, and has a
//! `pub fn build_sysfs(options)` beside its `build`.
//!
//! - **The tree** is real files on a tmpfs, owned by root as sysfs is (the server
//!   runs as root): in the tools' namespace root is not mapped, so a file's mode
//!   decides as for a user on a real machine. An attribute that is read only
//!   (0444) can't be opened for writing (EACCES), and no file can be created.
//! - **What the host does** comes from inotify, one watch per directory, and goes
//!   in the log: `sysfs_get PATH -> TEXT` for a file opened and closed with no
//!   write (TEXT what it held), `sysfs_put PATH TEXT -> RESULT` for one written and
//!   closed, and `sysfs_dir PATH` for a directory listed. A *paced* file, a
//!   measurement the kernel reads afresh each time, has a line per event:
//!   `sysfs_open PATH`, `sysfs_read PATH -> TEXT` for each read(2) that returned
//!   data, and `sysfs_close PATH`. A failed open makes no event, so no line: the
//!   tool's stderr says it. TEXT is a quoted string, or hex for bytes that are not
//!   text.
//! - **Reads pace the values.** After each read of a paced file the model may give
//!   it its next value ([`Tree::read`]), written in place, so a tool that keeps the
//!   file open, as C does, sees a live meter, and how many reads a capture made
//!   decides what it read, not the clock. inotify says so after the fact, so as
//!   root (the harness's server) each paced file also has a fanotify permission
//!   mark: a read(2) of it waits until the carrier has taken in everything before
//!   it (the last read's next value, a write's new setting, an armed unplug), and
//!   only then goes on. So what a read gets never depends on how late the carrier
//!   is. Without the permission (a unit test as a user) the carrier says so on
//!   standard error and paces after the fact, as fast as it is. [`Tree::tick`] comes every 5 ms too, for a model whose values
//!   follow the clock. A new value is written and then the file cut to its length,
//!   two calls a reader could fall between, so a value from a tick whose length
//!   differs from the file's waits for that file's next read and goes in right
//!   after it, when its reader is between reads; one of the same length is a
//!   single write.
//! - **A write** is what the file holds when the host closes it. The model accepts
//!   it ([`Written::Kept`]) or refuses it as the kernel's store would
//!   ([`Written::Refused`], with the errno's name): the old contents go back. The
//!   tool can't see that refusal, which on a real attribute is an error from the
//!   write(2) (C ignores it, as it ignores fclose's result); a model that needs a
//!   write to fail puts `/dev/full` behind the file ([`Fs::link`]). A file opened
//!   for writing and closed with nothing written keeps its contents, as an
//!   attribute does (`-> nothing written`).
//! - **Lifetimes.** A `reset` builds the tree afresh from the model, so each tool
//!   of each case starts from power on. `sync` takes in what is queued and returns,
//!   so a tool's lines land in its section. The tree goes when the session ends.
//! - **Events.** `--interrupt sysfs-read=N` and `--unplug sysfs-read=N` fire after
//!   the Nth read of a paced file: an unplug calls [`Tree::unplug`], and `replug`
//!   [`Tree::replug`].
//!
//! What it can't show: a read error on an open attribute (the kernel's -EIO for
//! an I2C fault, -ENODEV for a removed attribute) other than as a directory in the
//! file's place (EISDIR), or a file emptied, which no attribute ever reads as;
//! sysfs's 4096 byte size of every attribute; and a read atomic with a rewrite: a
//! tick's value of the same length is one write(2), but a reader in the middle of
//! its copy could still see part of it, and a value whose length changes shows
//! one read late.

use crate::model::Violations;

/// A node of the tree, as a model asks for it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Node {
    /// A directory.
    Dir,
    /// A file.
    File {
        /// Its contents.
        data: Vec<u8>,
        /// Its mode.
        mode: u32,
        /// Whether its reads go to [`Tree::read`] one by one.
        paced: bool,
    },
    /// A symbolic link to a path, which the host resolves in its own namespace.
    Link(String),
}

/// What a model asks the tree to do, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Op {
    /// A new node at a path, its parent directories made as needed.
    Put(String, Node),
    /// New contents for a file, written in place: what keeps it open reads them.
    Set(String, Vec<u8>),
    /// The node goes: a file is emptied first, so what keeps it open reads nothing.
    Remove(String),
}

/// Where a model says what to change, and what the host did wrong.
#[derive(Debug, Default)]
pub struct Fs {
    ops: Vec<Op>,
    v: Violations,
}

impl Fs {
    /// A directory.
    pub fn dir(&mut self, path: &str) {
        self.ops.push(Op::Put(path.to_owned(), Node::Dir));
    }

    /// A file of `mode` that holds `data`.
    pub fn file(&mut self, path: &str, data: impl Into<Vec<u8>>, mode: u32) {
        self.ops
            .push(Op::Put(path.to_owned(), Node::File { data: data.into(), mode, paced: false }));
    }

    /// A read-only file whose reads go to [`Tree::read`] one by one.
    pub fn paced(&mut self, path: &str, data: impl Into<Vec<u8>>) {
        self.ops.push(Op::Put(
            path.to_owned(),
            Node::File { data: data.into(), mode: 0o444, paced: true },
        ));
    }

    /// A symbolic link.
    pub fn link(&mut self, path: &str, target: &str) {
        self.ops.push(Op::Put(path.to_owned(), Node::Link(target.to_owned())));
    }

    /// New contents for a file.
    pub fn set(&mut self, path: &str, data: impl Into<Vec<u8>>) {
        self.ops.push(Op::Set(path.to_owned(), data.into()));
    }

    /// A node goes.
    pub fn remove(&mut self, path: &str) {
        self.ops.push(Op::Remove(path.to_owned()));
    }

    /// Reports what the host did wrong.
    pub fn violation(&mut self, message: impl Into<String>) {
        self.v.report(message);
    }

    /// What the model asked, for a unit test or the carrier.
    pub fn ops(&self) -> &[Op] {
        &self.ops
    }

    /// Takes what the model asked and the violations it reported.
    pub fn take(&mut self) -> (Vec<Op>, Vec<String>) {
        (std::mem::take(&mut self.ops), self.v.drain())
    }
}

/// What became of a write.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Written {
    /// The attribute took it: the file keeps what was written.
    Kept,
    /// The kernel's store refused it, with this errno's name: the old contents
    /// go back.
    Refused(&'static str),
}

/// A device as files. Nothing here blocks: answer in the call.
pub trait Tree: Send {
    /// The tree at power on, under the root (`class/...`).
    fn build(&mut self, fs: &mut Fs);
    /// The host read a paced file and got data.
    fn read(&mut self, path: &str, fs: &mut Fs) {
        let _ = (path, fs);
    }
    /// The host wrote `data` to a file and closed it.
    fn written(&mut self, path: &str, data: &[u8], fs: &mut Fs) -> Written {
        let _ = (path, data, fs);
        Written::Kept
    }
    /// Every 5 ms of wall time or so, whatever the host does.
    fn tick(&mut self, now: std::time::Instant, fs: &mut Fs) {
        let _ = (now, fs);
    }
    /// Whether a listing of the directory `path` goes in the log: a model can
    /// leave out what libraries list that is no part of its device (libudev,
    /// under libusb, lists `class`).
    fn listed(&self, path: &str) -> bool {
        let _ = path;
        true
    }
    /// An armed unplug fired.
    fn unplug(&mut self, fs: &mut Fs) {
        let _ = fs;
    }
    /// The unplugged device comes back.
    fn replug(&mut self, fs: &mut Fs) {
        let _ = fs;
    }
}

/// Bytes that are not text and longer than this show as their digest.
const DIGEST_ABOVE: usize = 64;

/// Contents as a log line shows them: a quoted string with `\n`, `\t`, `\\` and
/// `\"` escaped when they are text, hex otherwise, and over 64 bytes of what is
/// not text `@LEN:SHA256`, the fixtures' digest.
pub fn render(data: &[u8]) -> String {
    let text = data.iter().all(|&b| b == b'\n' || b == b'\t' || (0x20..0x7f).contains(&b));
    if !text && data.len() > DIGEST_ABOVE {
        use sha2::{Digest as _, Sha256};
        return format!("@{}:{}", data.len(), crate::txlog::hex(&Sha256::digest(data)));
    }
    if !text {
        return crate::txlog::hex(data);
    }
    let mut out = String::from("\"");
    for &b in data {
        match b {
            b'\n' => out.push_str("\\n"),
            b'\t' => out.push_str("\\t"),
            b'\\' => out.push_str("\\\\"),
            b'"' => out.push_str("\\\""),
            _ => out.push(char::from(b)),
        }
    }
    out.push('"');
    out
}

#[cfg(target_os = "linux")]
pub use carrier::Carrier;

#[cfg(target_os = "linux")]
mod carrier {
    use std::collections::HashMap;
    use std::fs::{self, File, OpenOptions, Permissions};
    use std::io::ErrorKind;
    use std::mem::MaybeUninit;
    use std::os::fd::OwnedFd;
    use std::os::unix::fs::{FileExt, OpenOptionsExt, PermissionsExt, symlink};
    use std::path::{Path, PathBuf};
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::sync::{Arc, Mutex, MutexGuard};
    use std::thread;
    use std::time::{Duration, Instant};

    use nix::sys::fanotify::{
        EventFFlags, Fanotify, FanotifyResponse, InitFlags, MarkFlags, MaskFlags, Response,
    };
    use rustix::event::{PollFd, PollFlags, Timespec, poll};
    use rustix::fs::inotify::{self, CreateFlags, ReadFlags, WatchFlags};
    use serde_json::{Value, json};
    use std::os::fd::AsFd;
    use vusb::arm::Event;
    use vusb::session::{Builder, Ctx};

    use super::{Fs, Node, Op, Tree, Written, render};
    use crate::shells::tcp::Log;
    use crate::{Error, Result};

    /// How long the carrier waits for an event before a tick.
    const TICK: Duration = Duration::from_millis(5);
    /// The most read back of a file the host wrote: an attribute is a page.
    const PAGE: usize = 4096;

    struct FileState {
        file: File,
        data: Vec<u8>,
        paced: bool,
    }

    struct Inner {
        root: PathBuf,
        inotify: OwnedFd,
        tree: Option<Box<dyn Tree>>,
        /// Watch descriptor to the directory's path under the root ("" the root).
        watches: HashMap<i32, String>,
        files: HashMap<String, FileState>,
        log: Arc<Mutex<dyn Log>>,
        armed: Option<(Event, u32)>,
        reads: u32,
        held: Option<Arc<Fanotify>>,
        /// A tick's values whose length differs from the file's, until its next read.
        pending: HashMap<String, Vec<u8>>,
    }

    /// The carrier: the tree, its watches and a thread that serves them.
    #[derive(Clone)]
    pub struct Carrier {
        inner: Arc<Mutex<Inner>>,
        /// The inotify descriptor again, to wait on without the lock.
        ready: Arc<OwnedFd>,
        /// The fanotify group whose permission events hold each read of a paced
        /// file, when the carrier may have one (root).
        held: Option<Arc<Fanotify>>,
        violations: Arc<AtomicUsize>,
        stop: Arc<AtomicBool>,
    }

    impl Carrier {
        /// Serves a tree in `root`, which it makes (and empties first).
        ///
        /// # Errors
        ///
        /// When inotify or the directory can't be had.
        pub fn new(root: &Path, log: Arc<Mutex<dyn Log>>) -> Result<Carrier> {
            let inotify = inotify::init(CreateFlags::NONBLOCK | CreateFlags::CLOEXEC)
                .map_err(|e| Error::Other(format!("inotify: {e}")))?;
            let ready = Arc::new(inotify.try_clone()?);
            let held = match Fanotify::init(
                InitFlags::FAN_CLASS_CONTENT | InitFlags::FAN_NONBLOCK | InitFlags::FAN_CLOEXEC,
                EventFFlags::O_RDONLY,
            ) {
                Ok(f) => Some(Arc::new(f)),
                Err(e) => {
                    eprintln!(
                        "oxrok-emu: no fanotify permission events ({e}): reads of the tree are \
                         paced after the fact"
                    );
                    None
                }
            };
            let inner = Inner {
                held: held.clone(),
                root: root.to_owned(),
                inotify,
                tree: None,
                watches: HashMap::new(),
                files: HashMap::new(),
                log,
                armed: None,
                reads: 0,
                pending: HashMap::new(),
            };
            inner.clear()?;
            let carrier = Carrier {
                inner: Arc::new(Mutex::new(inner)),
                ready,
                held,
                violations: Arc::new(AtomicUsize::new(0)),
                stop: Arc::new(AtomicBool::new(false)),
            };
            let serving = carrier.clone();
            thread::Builder::new()
                .name("sysfs carrier".to_owned())
                .spawn(move || serving.serve())?;
            Ok(carrier)
        }

        /// Whether reads of paced files wait for the carrier (fanotify, as root).
        pub fn synchronous(&self) -> bool {
            self.held.is_some()
        }

        /// The violations it logged.
        pub fn violations(&self) -> usize {
            self.violations.load(Ordering::SeqCst)
        }

        /// Ends the thread and takes the tree away.
        pub fn stop(&self) {
            self.stop.store(true, Ordering::SeqCst);
            let mut inner = self.lock();
            inner.tree = None;
            let _ = inner.teardown();
        }

        /// The carrier held up, for a test, until what this returns is dropped.
        #[cfg(test)]
        pub(super) fn hold(&self) -> impl Sized + '_ {
            self.lock()
        }

        fn lock(&self) -> MutexGuard<'_, Inner> {
            self.inner.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
        }

        fn serve(&self) {
            let timeout = Timespec { tv_sec: 0, tv_nsec: TICK.as_nanos().try_into().unwrap_or(0) };
            while !self.stop.load(Ordering::SeqCst) {
                let polled = if let Some(held) = &self.held {
                    let mut fds = [
                        PollFd::new(&*self.ready, PollFlags::IN),
                        PollFd::new(&**held, PollFlags::IN),
                    ];
                    poll(&mut fds, Some(&timeout))
                } else {
                    let mut fds = [PollFd::new(&*self.ready, PollFlags::IN)];
                    poll(&mut fds, Some(&timeout))
                };
                if polled.is_err() {
                    thread::sleep(TICK);
                }
                let mut inner = self.lock();
                inner.release(&self.violations);
                inner.take_in(&self.violations);
                inner.tick(&self.violations);
            }
        }

        /// A fresh tree: the old one goes, with what was armed.
        ///
        /// # Errors
        ///
        /// When the tree can't be made on disk.
        pub fn install(&self, mut tree: Box<dyn Tree>) -> Result<()> {
            let mut inner = self.lock();
            inner.tree = None;
            inner.armed = None;
            inner.reads = 0;
            inner.pending.clear();
            inner.teardown()?;
            inner.clear()?;
            let mut fs = Fs::default();
            tree.build(&mut fs);
            let (ops, _) = fs.take();
            for op in ops {
                inner.apply(op)?;
            }
            // What making it caused is nobody's.
            inner.discard();
            inner.tree = Some(tree);
            Ok(())
        }

        /// Takes in what the host did so far.
        pub fn sync(&self) {
            self.lock().take_in(&self.violations);
        }

        /// Arms an event at `sysfs-read=N`.
        ///
        /// # Errors
        ///
        /// On another point, or an N of 0.
        pub fn arm(&self, event: Event, when: &str) -> Result<()> {
            let n = when
                .strip_prefix("sysfs-read=")
                .and_then(|n| n.parse().ok())
                .filter(|n| *n > 0)
                .ok_or_else(|| Error::Invalid(format!("{when}: a sysfs point is sysfs-read=N")))?;
            let mut inner = self.lock();
            inner.armed = Some((event, n));
            inner.reads = 0;
            Ok(())
        }

        /// The unplugged device comes back.
        pub fn replug(&self) {
            let mut inner = self.lock();
            let mut fs = Fs::default();
            if let Some(tree) = &mut inner.tree {
                tree.replug(&mut fs);
            }
            inner.apply_fs(fs, &self.violations);
        }

        /// The session's `sync`, `arm` and `replug` for a session with a tree.
        #[must_use]
        pub fn commands(&self, builder: Builder) -> Builder {
            let (sync, arm, replug) = (self.clone(), self.clone(), self.clone());
            builder
                .command(
                    "sync",
                    "answers after what the host did to the tree is in the log",
                    move |_, _| {
                        sync.sync();
                        Ok(json!({}))
                    },
                )
                .command(
                    "arm",
                    "event=unplug or event=mark name=NAME, when=sysfs-read=N",
                    move |args, _: &mut Ctx| arm_command(&arm, args),
                )
                .command("replug", "an unplugged device comes back", move |_, _| {
                    replug.replug();
                    Ok(json!({}))
                })
        }
    }

    fn arm_command(carrier: &Carrier, args: &Value) -> Result<Value> {
        let text = |key| args.get(key).and_then(Value::as_str);
        let (Some(kind), Some(when)) = (text("event"), text("when")) else {
            return Err(Error::Invalid("arm takes event=, name= for a mark, and when=".into()));
        };
        let event = match (kind, text("name")) {
            ("mark", Some(name)) => Event::Mark(name.to_owned()),
            ("unplug", _) => Event::Unplug,
            _ => return Err(Error::Invalid(format!("no event {kind}"))),
        };
        carrier.arm(event, when)?;
        Ok(json!({}))
    }

    fn io(path: &Path, e: &std::io::Error) -> Error {
        Error::Other(format!("{}: {e}", path.display()))
    }

    impl Inner {
        fn line(&self, text: &str) {
            self.log.lock().unwrap_or_else(std::sync::PoisonError::into_inner).line(text);
        }

        fn violation(&self, message: &str, violations: &AtomicUsize) {
            violations.fetch_add(1, Ordering::SeqCst);
            self.log.lock().unwrap_or_else(std::sync::PoisonError::into_inner).violation(message);
        }

        fn path(&self, rel: &str) -> PathBuf {
            if rel.is_empty() { self.root.clone() } else { self.root.join(rel) }
        }

        /// The watches and files go, so that nothing of the old tree is logged.
        /// Lets the reads waiting on their permission go on, once all that came
        /// before them is taken in: the reader's own last read included, whose
        /// inotify event was queued before this read began.
        fn release(&mut self, violations: &AtomicUsize) {
            let Some(held) = self.held.clone() else { return };
            loop {
                let events = match held.read_events() {
                    Ok(events) if !events.is_empty() => events,
                    _ => return,
                };
                self.take_in(violations);
                for event in &events {
                    if let Some(fd) = event.fd() {
                        // A reader left waiting would hang: an error here is the
                        // group's, and its end lets the read go.
                        let _ = held.write_response(FanotifyResponse::new(fd, Response::FAN_ALLOW));
                    }
                }
            }
        }

        fn teardown(&mut self) -> Result<()> {
            for wd in self.watches.keys() {
                let _ = inotify::remove_watch(&self.inotify, *wd);
            }
            self.watches.clear();
            self.files.clear();
            if let Some(held) = &self.held {
                let _ = held.mark(
                    MarkFlags::FAN_MARK_FLUSH,
                    MaskFlags::empty(),
                    held.as_fd(),
                    None::<&Path>,
                );
            }
            self.discard();
            match fs::remove_dir_all(&self.root) {
                Ok(()) => Ok(()),
                Err(e) if e.kind() == ErrorKind::NotFound => Ok(()),
                Err(e) => Err(io(&self.root, &e)),
            }
        }

        /// An empty root, watched.
        fn clear(&self) -> Result<()> {
            match fs::remove_dir_all(&self.root) {
                Ok(()) => {}
                Err(e) if e.kind() == ErrorKind::NotFound => {}
                Err(e) => return Err(io(&self.root, &e)),
            }
            fs::create_dir_all(&self.root).map_err(|e| io(&self.root, &e))?;
            fs::set_permissions(&self.root, Permissions::from_mode(0o755))
                .map_err(|e| io(&self.root, &e))
        }

        fn watch(&mut self, rel: &str) -> Result<()> {
            let path = self.path(rel);
            let mask = WatchFlags::OPEN
                | WatchFlags::ACCESS
                | WatchFlags::CLOSE_WRITE
                | WatchFlags::CLOSE_NOWRITE
                | WatchFlags::ONLYDIR
                | WatchFlags::DONT_FOLLOW;
            let wd = inotify::add_watch(&self.inotify, &path, mask)
                .map_err(|e| Error::Other(format!("{}: inotify: {e}", path.display())))?;
            self.watches.insert(wd, rel.to_owned());
            Ok(())
        }

        /// The directory `rel` and those above it, watched.
        fn ensure_dir(&mut self, rel: &str) -> Result<()> {
            if !self.watches.values().any(String::is_empty) {
                self.watch("")?;
            }
            let mut at = String::new();
            for part in rel.split('/').filter(|p| !p.is_empty()) {
                if !at.is_empty() {
                    at.push('/');
                }
                at.push_str(part);
                if self.watches.values().any(|w| *w == at) {
                    continue;
                }
                let path = self.path(&at);
                match fs::create_dir(&path) {
                    Ok(()) => {}
                    Err(e) if e.kind() == ErrorKind::AlreadyExists => {}
                    Err(e) => return Err(io(&path, &e)),
                }
                fs::set_permissions(&path, Permissions::from_mode(0o755))
                    .map_err(|e| io(&path, &e))?;
                self.watch(&at)?;
            }
            Ok(())
        }

        fn apply(&mut self, op: Op) -> Result<()> {
            match op {
                Op::Put(rel, Node::Dir) => self.ensure_dir(&rel),
                Op::Put(rel, Node::File { data, mode, paced }) => {
                    let parent = rel.rsplit_once('/').map_or("", |(p, _)| p).to_owned();
                    self.ensure_dir(&parent)?;
                    let path = self.path(&rel);
                    let file = OpenOptions::new()
                        .read(true)
                        .write(true)
                        .create(true)
                        .truncate(true)
                        .mode(mode)
                        .open(&path)
                        .map_err(|e| io(&path, &e))?;
                    file.write_all_at(&data, 0).map_err(|e| io(&path, &e))?;
                    fs::set_permissions(&path, Permissions::from_mode(mode))
                        .map_err(|e| io(&path, &e))?;
                    if paced && let Some(held) = &self.held {
                        held.mark(
                            MarkFlags::FAN_MARK_ADD,
                            MaskFlags::FAN_ACCESS_PERM,
                            held.as_fd(),
                            Some(path.as_path()),
                        )
                        .map_err(|e| Error::Other(format!("{}: fanotify: {e}", path.display())))?;
                    }
                    self.files.insert(rel, FileState { file, data, paced });
                    Ok(())
                }
                Op::Put(rel, Node::Link(target)) => {
                    let parent = rel.rsplit_once('/').map_or("", |(p, _)| p).to_owned();
                    self.ensure_dir(&parent)?;
                    let path = self.path(&rel);
                    symlink(&target, &path).map_err(|e| io(&path, &e))
                }
                Op::Set(rel, data) => {
                    let Some(state) = self.files.get_mut(&rel) else {
                        return Err(Error::Other(format!("{rel}: no such file in the tree")));
                    };
                    rewrite(&state.file, &data).map_err(|e| io(Path::new(&rel), &e))?;
                    state.data = data;
                    Ok(())
                }
                Op::Remove(rel) => {
                    let path = self.path(&rel);
                    if let Some(state) = self.files.remove(&rel) {
                        let _ = state.file.set_len(0);
                    }
                    let gone = if path.is_dir() && !path.is_symlink() {
                        self.watches.retain(|_, w| *w != rel && !w.starts_with(&format!("{rel}/")));
                        fs::remove_dir_all(&path)
                    } else {
                        fs::remove_file(&path)
                    };
                    gone.map_err(|e| io(&path, &e))
                }
            }
        }

        /// Applies what a model asked; a failure is the model's, and a violation.
        fn apply_fs(&mut self, mut fs: Fs, violations: &AtomicUsize) {
            let (ops, said) = fs.take();
            for message in said {
                self.violation(&message, violations);
            }
            for op in ops {
                if let Err(e) = self.apply(op) {
                    self.violation(&format!("the model's tree: {e}"), violations);
                }
            }
        }

        /// Reads the queue empty, throwing it away.
        fn discard(&self) {
            let _ = self.events();
        }

        /// What is queued, as (watch, flags, name).
        fn events(&self) -> Vec<(i32, ReadFlags, Option<String>)> {
            let mut out = Vec::new();
            let mut buf = [MaybeUninit::<u8>::uninit(); 8192];
            let mut reader = inotify::Reader::new(&self.inotify, &mut buf);
            while let Ok(e) = reader.next() {
                let name = e.file_name().map(|n| n.to_string_lossy().into_owned());
                out.push((e.wd(), e.events(), name));
            }
            out
        }

        fn take_in(&mut self, violations: &AtomicUsize) {
            for (wd, flags, name) in self.events() {
                self.event(wd, flags, name, violations);
            }
        }

        fn tick(&mut self, violations: &AtomicUsize) {
            let Some(mut tree) = self.tree.take() else { return };
            let mut fs = Fs::default();
            tree.tick(Instant::now(), &mut fs);
            self.tree = Some(tree);
            // A paced file's new length waits for its next read.
            let (ops, said) = fs.take();
            let mut now = Fs::default();
            for message in said {
                now.violation(message);
            }
            for op in ops {
                match op {
                    Op::Set(rel, data)
                        if self.files.get(&rel).is_some_and(|s| s.paced)
                            && (self.pending.contains_key(&rel)
                                || self.files[&rel].data.len() != data.len()) =>
                    {
                        self.pending.insert(rel, data);
                    }
                    other => now.ops.push(other),
                }
            }
            self.apply_fs(now, violations);
        }

        fn event(
            &mut self,
            wd: i32,
            flags: ReadFlags,
            name: Option<String>,
            violations: &AtomicUsize,
        ) {
            if flags.contains(ReadFlags::QUEUE_OVERFLOW) {
                self.violation(
                    "the carrier's inotify queue overflowed: events are lost",
                    violations,
                );
                return;
            }
            let Some(dir) = self.watches.get(&wd).cloned() else { return };
            let Some(name) = name else {
                // The directory itself: listed once it is closed.
                if flags.contains(ReadFlags::CLOSE_NOWRITE)
                    && self.tree.as_ref().is_none_or(|t| t.listed(&dir))
                {
                    self.line(&format!("sysfs_dir {dir}"));
                }
                return;
            };
            // A subdirectory says so on its own watch.
            if flags.contains(ReadFlags::ISDIR) {
                return;
            }
            let rel = if dir.is_empty() { name } else { format!("{dir}/{name}") };
            let Some(paced) = self.files.get(&rel).map(|s| s.paced) else { return };
            if paced {
                if flags.contains(ReadFlags::OPEN) {
                    self.line(&format!("sysfs_open {rel}"));
                }
                if flags.contains(ReadFlags::ACCESS) {
                    self.paced_read(&rel, violations);
                }
                if flags.contains(ReadFlags::CLOSE_NOWRITE) {
                    self.line(&format!("sysfs_close {rel}"));
                }
            } else if flags.contains(ReadFlags::CLOSE_NOWRITE) {
                let text = render(&self.files[&rel].data);
                self.line(&format!("sysfs_get {rel} -> {text}"));
            }
            if flags.contains(ReadFlags::CLOSE_WRITE) {
                self.written(&rel, violations);
            }
        }

        fn paced_read(&mut self, rel: &str, violations: &AtomicUsize) {
            let text = render(&self.files[rel].data);
            self.line(&format!("sysfs_read {rel} -> {text}"));
            if let Some(data) = self.pending.remove(rel) {
                let mut fs = Fs::default();
                fs.set(rel, data);
                self.apply_fs(fs, violations);
            }
            if let Some(mut tree) = self.tree.take() {
                let mut fs = Fs::default();
                tree.read(rel, &mut fs);
                self.tree = Some(tree);
                self.apply_fs(fs, violations);
            }
            self.reads += 1;
            if matches!(self.armed, Some((_, n)) if n == self.reads) {
                self.fire(violations);
            }
        }

        fn fire(&mut self, violations: &AtomicUsize) {
            let Some((event, _)) = self.armed.take() else { return };
            self.line(event.word());
            println!("EVENT {}", event.word());
            if event == Event::Unplug
                && let Some(mut tree) = self.tree.take()
            {
                let mut fs = Fs::default();
                tree.unplug(&mut fs);
                self.tree = Some(tree);
                self.apply_fs(fs, violations);
            }
        }

        fn written(&mut self, rel: &str, violations: &AtomicUsize) {
            let Some(state) = self.files.get(rel) else { return };
            let mut buf = vec![0u8; PAGE + 1];
            let got = state.file.read_at(&mut buf, 0).unwrap_or(0);
            buf.truncate(got);
            let old = state.data.clone();
            if buf.is_empty() {
                // fopen's O_TRUNC reaches no attribute.
                let _ = rewrite(&state.file, &old);
                self.line(&format!("sysfs_put {rel} \"\" -> nothing written"));
                return;
            }
            let text = render(&buf);
            let Some(mut tree) = self.tree.take() else { return };
            let mut fs = Fs::default();
            let outcome = tree.written(rel, &buf, &mut fs);
            self.tree = Some(tree);
            match outcome {
                Written::Kept => {
                    if let Some(state) = self.files.get_mut(rel) {
                        state.data = buf;
                    }
                    self.line(&format!("sysfs_put {rel} {text} -> ok"));
                }
                Written::Refused(errno) => {
                    if let Some(state) = self.files.get(rel) {
                        let _ = rewrite(&state.file, &old);
                    }
                    self.line(&format!("sysfs_put {rel} {text} -> {errno}"));
                }
            }
            self.apply_fs(fs, violations);
        }
    }

    /// New contents in place: a reader that keeps the file open reads them.
    fn rewrite(file: &File, data: &[u8]) -> std::io::Result<()> {
        file.write_all_at(data, 0)?;
        file.set_len(u64::try_from(data.len()).unwrap_or(u64::MAX))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contents_render_as_text_or_hex() {
        assert_eq!(render(b"ina226\n"), "\"ina226\\n\"");
        assert_eq!(render(b"a\"b\\c\td"), "\"a\\\"b\\\\c\\td\"");
        assert_eq!(render(b""), "\"\"");
        assert_eq!(render(&[0, 0, 0, 1, b'B']), "0000000142");
        let long = render(&[0xff; 65]);
        assert!(long.starts_with("@65:") && long.len() == 4 + 64, "{long}");
    }

    #[test]
    fn a_model_says_what_to_change_in_order() {
        let mut fs = Fs::default();
        fs.dir("class/hwmon");
        fs.paced("class/hwmon/hwmon0/in1_input", "5000\n");
        fs.file("class/hwmon/hwmon0/shunt_resistor", "10000\n", 0o666);
        fs.link("class/x", "/dev/full");
        fs.set("class/hwmon/hwmon0/in1_input", "5001\n");
        fs.remove("class/x");
        fs.violation("nope");
        let (ops, said) = fs.take();
        assert_eq!(ops.len(), 6);
        assert_eq!(
            ops[1],
            Op::Put(
                "class/hwmon/hwmon0/in1_input".into(),
                Node::File { data: b"5000\n".to_vec(), mode: 0o444, paced: true }
            )
        );
        assert_eq!(said, ["nope"]);
        assert_eq!(fs.ops(), []);
    }
}

#[cfg(all(test, target_os = "linux"))]
mod carrier_tests {
    use std::io::{Read, Seek, SeekFrom, Write};
    use std::sync::{Arc, Mutex};

    use super::*;
    use crate::shells::tcp::Log;
    use crate::txlog::Rules;

    #[derive(Default)]
    struct Lines(Vec<String>);

    impl Log for Lines {
        fn line(&mut self, text: &str) {
            self.0.push(text.to_owned());
        }
        fn violation(&mut self, message: &str) {
            self.0.push(format!("! {message}"));
        }
        fn rules(&self) -> Rules {
            Rules::default()
        }
    }

    /// A counter that counts its reads, and a setting that takes digits only.
    struct Counter(u32);

    impl Tree for Counter {
        fn build(&mut self, fs: &mut Fs) {
            fs.paced("class/c/value", "0\n");
            fs.file("class/c/setting", "1\n", 0o666);
            fs.file("class/c/fixed", "x\n", 0o444);
        }
        fn read(&mut self, _path: &str, fs: &mut Fs) {
            self.0 += 1;
            fs.set("class/c/value", format!("{}\n", self.0));
        }
        fn written(&mut self, _path: &str, data: &[u8], _fs: &mut Fs) -> Written {
            if data.iter().all(|b| b.is_ascii_digit() || *b == b'\n') {
                Written::Kept
            } else {
                Written::Refused("EINVAL")
            }
        }
    }

    fn carrier() -> (Carrier, Arc<Mutex<Lines>>, tempdir::Dir) {
        let dir = tempdir::Dir::new();
        let lines = Arc::new(Mutex::new(Lines::default()));
        let log: Arc<Mutex<dyn Log>> = lines.clone();
        let carrier = Carrier::new(&dir.0.join("tree"), log).unwrap();
        carrier.install(Box::new(Counter(0))).unwrap();
        (carrier, lines, dir)
    }

    mod tempdir {
        /// A directory of its own under the target's temporary directory.
        pub struct Dir(pub std::path::PathBuf);
        impl Dir {
            pub fn new() -> Dir {
                use std::sync::atomic::{AtomicU32, Ordering};
                static N: AtomicU32 = AtomicU32::new(0);
                let p = std::env::temp_dir().join(format!(
                    "oxrok-emu-sysfs-{}-{}",
                    std::process::id(),
                    N.fetch_add(1, Ordering::SeqCst)
                ));
                std::fs::create_dir_all(&p).unwrap();
                Dir(p)
            }
        }
        impl Drop for Dir {
            fn drop(&mut self) {
                let _ = std::fs::remove_dir_all(&self.0);
            }
        }
    }

    #[test]
    fn reads_of_a_kept_file_pace_its_values_and_go_in_the_log() {
        let (carrier, lines, dir) = carrier();
        let path = dir.0.join("tree/class/c/value");
        let mut f = std::fs::File::open(&path).unwrap();
        let mut got = Vec::new();
        for _ in 0..3 {
            let mut buf = [0u8; 16];
            f.seek(SeekFrom::Start(0)).unwrap();
            let n = f.read(&mut buf).unwrap();
            got.push(String::from_utf8_lossy(&buf[..n]).into_owned());
            carrier.sync();
        }
        drop(f);
        carrier.sync();
        assert_eq!(got, ["0\n", "1\n", "2\n"]);
        let lines = lines.lock().unwrap();
        assert_eq!(
            lines.0,
            [
                "sysfs_open class/c/value",
                "sysfs_read class/c/value -> \"0\\n\"",
                "sysfs_read class/c/value -> \"1\\n\"",
                "sysfs_read class/c/value -> \"2\\n\"",
                "sysfs_close class/c/value",
            ]
        );
        carrier.stop();
    }

    /// As root, a read waits for the carrier: back to back reads, with no sync
    /// between them and the carrier held up, still get one value each. A user
    /// has no fanotify, and the test says so and passes.
    #[test]
    fn as_root_a_read_waits_for_the_last_reads_next_value() {
        let (carrier, _lines, dir) = carrier();
        if !carrier.synchronous() {
            eprintln!("not root: no fanotify, the reads are paced after the fact");
            carrier.stop();
            return;
        }
        let path = dir.0.join("tree/class/c/value");
        let mut f = std::fs::File::open(&path).unwrap();
        let mut got = Vec::new();
        for _ in 0..50 {
            // The carrier is busy elsewhere for a while.
            let held = carrier.hold();
            let reader = std::thread::spawn({
                let mut f = f.try_clone().unwrap();
                move || {
                    let mut buf = [0u8; 16];
                    f.seek(SeekFrom::Start(0)).unwrap();
                    let n = f.read(&mut buf).unwrap();
                    String::from_utf8_lossy(&buf[..n]).into_owned()
                }
            });
            std::thread::sleep(std::time::Duration::from_millis(2));
            drop(held);
            got.push(reader.join().unwrap());
        }
        drop(f.seek(SeekFrom::Start(0)));
        let want: Vec<String> = (0..50).map(|n| format!("{n}\n")).collect();
        assert_eq!(got, want);
        carrier.stop();
    }

    #[test]
    fn writes_are_kept_or_refused_and_reads_of_the_rest_are_gets() {
        let (carrier, lines, dir) = carrier();
        let path = dir.0.join("tree/class/c/setting");
        std::fs::write(&path, "42\n").unwrap();
        carrier.sync();
        std::fs::write(&path, "junk\n").unwrap();
        carrier.sync();
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "42\n");
        std::fs::File::create(&path).unwrap().flush().unwrap();
        carrier.sync();
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "42\n");
        std::fs::read_dir(dir.0.join("tree/class/c")).unwrap().for_each(drop);
        carrier.sync();
        let lines = lines.lock().unwrap();
        assert_eq!(
            lines.0,
            [
                "sysfs_put class/c/setting \"42\\n\" -> ok",
                "sysfs_put class/c/setting \"junk\\n\" -> EINVAL",
                "sysfs_get class/c/setting -> \"42\\n\"",
                "sysfs_put class/c/setting \"\" -> nothing written",
                "sysfs_get class/c/setting -> \"42\\n\"",
                "sysfs_dir class/c",
            ]
        );
        carrier.stop();
        assert!(!dir.0.join("tree").exists());
    }

    /// A tick's value of another length waits for the next read, so no reader
    /// falls between its write and its cut.
    #[test]
    fn a_ticks_value_of_another_length_waits_for_a_read() {
        struct Clock(u32);
        impl Tree for Clock {
            fn build(&mut self, fs: &mut Fs) {
                fs.paced("class/c/value", "1003125\n");
            }
            fn tick(&mut self, _now: std::time::Instant, fs: &mut Fs) {
                self.0 += 1;
                let v = if self.0 % 2 == 1 { "987500\n" } else { "987501\n" };
                fs.set("class/c/value", v);
            }
        }
        let dir = tempdir::Dir::new();
        let lines = Arc::new(Mutex::new(Lines::default()));
        let log: Arc<Mutex<dyn Log>> = lines.clone();
        let carrier = Carrier::new(&dir.0.join("tree"), log).unwrap();
        carrier.install(Box::new(Clock(0))).unwrap();
        let path = dir.0.join("tree/class/c/value");
        std::thread::sleep(std::time::Duration::from_millis(50));
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "1003125\n", "not before a read");
        carrier.sync();
        assert!(lines.lock().unwrap().0.iter().any(|l| l.starts_with("sysfs_read")));
        std::thread::sleep(std::time::Duration::from_millis(50));
        let after = std::fs::read_to_string(&path).unwrap();
        assert!(after == "987500\n" || after == "987501\n", "{after:?}");
        carrier.stop();
    }

    #[test]
    fn an_armed_mark_fires_at_its_read_and_a_reset_disarms() {
        let (carrier, lines, dir) = carrier();
        carrier.arm(vusb::arm::Event::Mark("interrupt".into()), "sysfs-read=2").unwrap();
        assert!(carrier.arm(vusb::arm::Event::Unplug, "in=1:2").is_err());
        assert!(carrier.arm(vusb::arm::Event::Unplug, "sysfs-read=0").is_err());
        let path = dir.0.join("tree/class/c/value");
        for _ in 0..3 {
            std::fs::read(&path).unwrap();
            carrier.sync();
        }
        let lines = lines.lock().unwrap().0.clone();
        let reads: Vec<usize> =
            (0..lines.len()).filter(|&i| lines[i].starts_with("sysfs_read")).collect();
        assert_eq!(reads.len(), 3, "{lines:?}");
        assert_eq!(lines.iter().filter(|l| *l == "interrupt").count(), 1, "{lines:?}");
        assert_eq!(lines[reads[1] + 1], "interrupt", "{lines:?}");
        carrier.stop();
    }
}
