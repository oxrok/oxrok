// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Bluetooth LE carrier: a virtual HCI controller behind the kernel's
//! `hci_vhci` (`/dev/vhci`), with one peripheral on its air, for the drivers
//! libsigrok reaches through its `bt/` layer. That layer, and oxrok's, open an
//! L2CAP socket on the ATT fixed channel and leave the link to the kernel, so
//! the boundary they touch is the kernel's: what stands in for the device is the
//! controller the kernel talks HCI to, as `BlueZ`'s `emulator/btdev.c` does. No
//! bluetoothd is involved; `oxrok-emu serve MODEL --ble` (root) creates the
//! controller, brings it up (the `HCIDEVUP` a `hciconfig hciN up` makes, which
//! `main.rs` does) and prints `BLE hci=hciN addr=PERIPHERAL`. Closing
//! `/dev/vhci` removes it.
//!
//! A model implements [`Peripheral`], and has a `pub fn build_ble(options)`
//! beside its `build`: its address and advertising, its attribute table, and what
//! a write to one of its characteristics does.
//!
//! - **The controller.** Version 4.0, LE only, one connection, no encryption,
//!   no accept list, no extended anything: the kernel's init then takes the short
//!   path, and the commands it sends are those this file answers. A command it
//!   does not know is answered "unknown command" (0x01), as a controller does,
//!   and noted on standard error. Packets the host sends are acknowledged at once
//!   (Number of Completed Packets). The data length is 27, so an ATT PDU of the
//!   default MTU (23) is one ACL packet; longer L2CAP frames are reassembled.
//! - **The air.** While the host scans, the peripheral's advertisement is
//!   reported once per scan, unless it is gone; a connection to its address
//!   completes at once (or fails, as [`Peripheral::connect_status`] says).
//! - **The GATT server.** Exchange MTU (always 23), Find Information, Read By
//!   Type, Read By Group Type, Read, Write Request and Write Command are answered
//!   from the table; a Client Characteristic Configuration is the shell's, and
//!   a notification goes out only while its CCCD has them on, as a stack does.
//!   Anything else gets an Error Response, "request not supported".
//! - **Lifetimes.** A `reset` builds a fresh peripheral and drops the link, in
//!   silence, so each tool of each case starts from power on.
//! - **The log.** `ble_connect`, `ble_disconnect host|device REASON`,
//!   `att_write HANDLE VALUE -> ok`, `att_write_cmd HANDLE VALUE`,
//!   `att_notify HANDLE VALUE`, `att_req PDU -> PDU` for the rest, in order.
//!   Handles are 4 hex digits, values hex. The HCI traffic is not logged: it is
//!   the kernel's, not the tool's (with `-v` it goes to standard error).
//! - **`sync`** waits, up to 8 s, for the link of a tool that has ended to go:
//!   the kernel drops an LE link 2 s after its last socket closes, and the
//!   `ble_disconnect` lands in the tool's section.
//! - **An unplug** is `--unplug ble-notify=N` (after the peripheral's Nth
//!   notification) or `ble-model` (when it marks its point): the link drops as
//!   a supervision timeout does (reason 08), `unplug` goes in the log, and the
//!   peripheral neither advertises nor connects until the next `replug` or
//!   `reset`. `--interrupt` takes the same points.
//! - **What a later BLE model must know.** A CCCD write turns notifications on
//!   by its bit 0 only: indications (bit 1, which libsigrok's `ble122` path
//!   writes as 0x0003) are not sent, and nothing indicates. The ATT MTU is fixed
//!   at 23: an Exchange MTU is answered 23, nothing is fragmented, and a value
//!   longer than 20 bytes is neither sent nor refused whole. Write Requests sent
//!   before the last one's response (libsigrok pipelines them) are answered in
//!   turn and not flagged. bluetoothd must not run: it would claim the
//!   controller and connect a GATT client of its own (`tools/diffcli.sh` refuses
//!   a session while it does).
//! - **Time.** [`Peripheral::tick`] comes every 20 ms of wall time while a host
//!   is connected, with the time in [`Port::now`]: a meter streams on its own
//!   clock, and nothing on the host side paces it.

use std::fs::File;
use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use serde_json::{Value, json};
use vusb::arm::{Arm, Event};
use vusb::observer::fixture::hex;
use vusb::session::{Builder, Ctx};

use crate::model::Violations;
use crate::shells::tcp::Log;
use crate::{Error, Result};

/// The period of [`Peripheral::tick`].
const TICK: Duration = Duration::from_millis(20);
/// How long `sync` waits for the host to drop the link.
const DRAIN: Duration = Duration::from_secs(8);
/// The connection handle of the one link.
const HANDLE: u16 = 0x0040;
/// The LE data length: an ATT PDU of 23 bytes and its L2CAP header.
const LE_MTU: u16 = 27;
/// The ATT MTU this server keeps to.
pub const ATT_MTU: usize = 23;

/// HCI packet types.
const CMD: u8 = 0x01;
const ACL: u8 = 0x02;
const EVT: u8 = 0x04;
const VENDOR: u8 = 0xff;

/// L2CAP channels.
const CID_ATT: u16 = 0x0004;
const CID_LE_SIGNAL: u16 = 0x0005;
const CID_SMP: u16 = 0x0006;

/// The GATT declarations and descriptors.
/// The type of a primary service declaration.
pub const PRIMARY_SERVICE: u16 = 0x2800;
/// The type of a characteristic declaration.
pub const CHARACTERISTIC: u16 = 0x2803;
/// The type of a Client Characteristic Configuration.
pub const CCCD: u16 = 0x2902;
/// The type of a Characteristic User Description.
pub const USER_DESCRIPTION: u16 = 0x2901;

/// Disconnection reasons.
const REMOTE_USER_TERMINATED: u8 = 0x13;
const SUPERVISION_TIMEOUT: u8 = 0x08;

/// A UUID as it goes on the air: 2 bytes, or 16, little endian.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Uuid {
    /// A Bluetooth SIG 16 bit one.
    Short(u16),
    /// A 128 bit one, written as usual (big endian, the dashes left out).
    Long([u8; 16]),
}

impl Uuid {
    /// A 128 bit UUID from its text, `1bc5ffa0-0200-62ab-e411-f254e005dbd4`.
    ///
    /// # Panics
    ///
    /// On text that is not 32 hex digits and dashes: these are constants.
    pub fn long(text: &str) -> Uuid {
        let digits: Vec<u8> = text.bytes().filter(|c| *c != b'-').collect();
        assert_eq!(digits.len(), 32, "{text}: a UUID is 32 hex digits");
        let mut out = [0u8; 16];
        for (i, pair) in digits.chunks(2).enumerate() {
            let s = std::str::from_utf8(pair).expect("ascii");
            out[i] = u8::from_str_radix(s, 16).expect("hex digits");
        }
        Uuid::Long(out)
    }

    /// The bytes on the air.
    pub fn wire(&self) -> Vec<u8> {
        match self {
            Uuid::Short(u) => u.to_le_bytes().to_vec(),
            Uuid::Long(b) => b.iter().rev().copied().collect(),
        }
    }

    fn from_wire(bytes: &[u8]) -> Option<Uuid> {
        match bytes.len() {
            2 => Some(Uuid::Short(u16::from_le_bytes([bytes[0], bytes[1]]))),
            16 => {
                let mut b = [0u8; 16];
                for (i, v) in bytes.iter().rev().enumerate() {
                    b[i] = *v;
                }
                Some(Uuid::Long(b))
            }
            _ => None,
        }
    }
}

/// What a host may do to an attribute.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Access {
    /// Read Request.
    pub read: bool,
    /// Write Request.
    pub write: bool,
    /// Write Command.
    pub write_cmd: bool,
}

/// One row of a peripheral's attribute table.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Attribute {
    /// Its handle.
    pub handle: u16,
    /// Its type.
    pub kind: Uuid,
    /// Its value, for a declaration or a value the table holds. A
    /// characteristic the peripheral answers itself has none here.
    pub value: Vec<u8>,
    /// What the host may do.
    pub access: Access,
}

impl Attribute {
    /// A primary service declaration.
    pub fn service(handle: u16, uuid: &Uuid) -> Attribute {
        Attribute {
            handle,
            kind: Uuid::Short(PRIMARY_SERVICE),
            value: uuid.wire(),
            access: Access { read: true, ..Access::default() },
        }
    }

    /// A characteristic declaration: its properties, value handle and UUID.
    pub fn characteristic(handle: u16, properties: u8, value: u16, uuid: &Uuid) -> Attribute {
        let mut v = vec![properties];
        v.extend_from_slice(&value.to_le_bytes());
        v.extend_from_slice(&uuid.wire());
        Attribute {
            handle,
            kind: Uuid::Short(CHARACTERISTIC),
            value: v,
            access: Access { read: true, ..Access::default() },
        }
    }

    /// A value or a descriptor.
    pub fn value(handle: u16, kind: Uuid, value: &[u8], access: Access) -> Attribute {
        Attribute { handle, kind, value: value.to_vec(), access }
    }

    /// A Client Characteristic Configuration, notifications off.
    pub fn cccd(handle: u16) -> Attribute {
        Attribute {
            handle,
            kind: Uuid::Short(CCCD),
            value: vec![0, 0],
            access: Access { read: true, write: true, write_cmd: false },
        }
    }
}

/// A peripheral on the air. Nothing is blocking here: answer in the call.
pub trait Peripheral: Send {
    /// Its public address, as written (`D0:5F:B8:00:00:01` is
    /// `[0xd0, 0x5f, 0xb8, 0, 0, 1]`).
    fn address(&self) -> [u8; 6];
    /// Its advertising data, at most 31 bytes.
    fn advertising(&self) -> Vec<u8>;
    /// Its attribute table, in handle order.
    fn attributes(&self) -> Vec<Attribute>;
    /// Whether it is on the air at all.
    fn present(&self) -> bool {
        true
    }
    /// The status of a connection to it: 0, or the HCI error it fails with.
    fn connect_status(&self) -> u8 {
        0
    }
    /// The host wrote a characteristic the table lets it write (a value with
    /// no CCCD type), by a Write Request or a Write Command.
    fn written(&mut self, handle: u16, value: &[u8], port: &mut Port);
    /// A host connected.
    fn connected(&mut self, port: &mut Port) {
        let _ = port;
    }
    /// The link went.
    fn disconnected(&mut self) {}
    /// Every 20 ms of wall time, with a host connected.
    fn tick(&mut self, port: &mut Port) {
        let _ = port;
    }
}

/// What a [`Peripheral`] can do to the link at hand.
#[derive(Debug)]
pub struct Port {
    now: Instant,
    notified: Vec<(u16, Vec<u8>)>,
    v: Violations,
    hang_up: bool,
    mark: bool,
    subscribed: Vec<u16>,
}

impl Port {
    /// A port at time `now`, with notifications on for `subscribed`, for a
    /// model's unit test.
    pub fn new(now: Instant, subscribed: &[u16]) -> Port {
        Port {
            now,
            notified: Vec::new(),
            v: Violations::default(),
            hang_up: false,
            mark: false,
            subscribed: subscribed.to_vec(),
        }
    }

    /// The time of this call.
    pub fn now(&self) -> Instant {
        self.now
    }

    /// Notifies the value of `handle`. It goes out only while the host has
    /// notifications on for it.
    pub fn notify(&mut self, handle: u16, value: &[u8]) {
        self.notified.push((handle, value.to_vec()));
    }

    /// Whether the host has notifications on for `handle`.
    pub fn subscribed(&self, handle: u16) -> bool {
        self.subscribed.contains(&handle)
    }

    /// Reports what the host did wrong.
    pub fn violation(&mut self, message: impl Into<String>) {
        self.v.report(message);
    }

    /// Reaches the point an event armed with `ble-model` waits for.
    pub fn mark(&mut self) {
        self.mark = true;
    }

    /// Ends the link from the peripheral's side, after what was notified.
    pub fn hang_up(&mut self) {
        self.hang_up = true;
    }

    /// What was notified, for a model's unit test.
    pub fn notified(&self) -> &[(u16, Vec<u8>)] {
        &self.notified
    }

    /// Whether [`Port::hang_up`] was called, for a model's unit test.
    pub fn hung_up(&self) -> bool {
        self.hang_up
    }

    /// Whether [`Port::mark`] was called, for a model's unit test.
    pub fn marked(&self) -> bool {
        self.mark
    }

    /// The violations reported, for a model's unit test.
    pub fn violations(&mut self) -> Vec<String> {
        self.v.drain()
    }
}

/// When an armed event fires.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum When {
    /// After the peripheral's Nth notification.
    Notified(u32),
    /// When the peripheral marks its point.
    Model,
}

/// The link to the host.
#[derive(Debug, Default)]
struct Link {
    /// An L2CAP frame being reassembled: its length and what came.
    partial: Option<(usize, Vec<u8>)>,
    /// The CCCDs the host turned notifications on with.
    subscribed: Vec<u16>,
    /// The handle a Read Remote Features is pending on (answered next round).
    features: bool,
}

/// The controller and the peripheral on its air.
#[allow(clippy::struct_excessive_bools)]
struct Inner {
    /// Where HCI packets to the host go.
    out: Box<dyn Write + Send>,
    model: Option<Box<dyn Peripheral>>,
    log: Arc<Mutex<dyn Log>>,
    /// The host's scan is on, and whether this scan has reported yet.
    scanning: bool,
    reported: bool,
    /// A Create Connection waits for its completion.
    connecting: Option<[u8; 6]>,
    link: Option<Link>,
    armed: Option<(Event, When)>,
    notifications: u32,
    unplugged: bool,
    last_tick: Instant,
    verbose: bool,
    /// The controller's own address, as written.
    own: [u8; 6],
    violations: Arc<AtomicUsize>,
}

/// The carrier: a controller on `/dev/vhci` and a thread that serves it.
#[derive(Clone)]
pub struct Carrier {
    inner: Arc<Mutex<Inner>>,
    index: u16,
    violations: Arc<AtomicUsize>,
    stop: Arc<AtomicBool>,
}

/// The controller's address: the public one the kernel reads.
const OWN: [u8; 6] = [0x00, 0xaa, 0x01, 0x00, 0x00, 0x23];

impl Carrier {
    /// Creates the controller on `/dev/vhci` and serves it; `power_up` is
    /// called with the controller's index once the thread answers the
    /// kernel, and brings it up.
    ///
    /// # Errors
    ///
    /// When `/dev/vhci` cannot be opened (no `hci_vhci`, or not root), the
    /// kernel does not create the controller, or `power_up` fails.
    pub fn open(
        log: Arc<Mutex<dyn Log>>,
        verbose: bool,
        power_up: impl FnOnce(u16) -> io::Result<()>,
    ) -> Result<Carrier> {
        let mut vhci = File::options().read(true).write(true).open("/dev/vhci").map_err(|e| {
            Error::Other(format!("/dev/vhci: {e} (is hci_vhci there, and are we root?)"))
        })?;
        // A primary controller; bits 6 and 7 (external configuration, raw)
        // left clear.
        vhci.write_all(&[VENDOR, 0x00])?;
        let mut buf = [0u8; 4096];
        let index = loop {
            let n = vhci.read(&mut buf)?;
            if n >= 4 && buf[0] == VENDOR && buf[1] == 0x00 {
                break u16::from_le_bytes([buf[2], buf[3]]);
            }
            if n == 0 {
                return Err(Error::Other("/dev/vhci: no controller was created".into()));
            }
        };
        let (packets, rx) = mpsc::channel();
        let mut reader = vhci.try_clone()?;
        thread::Builder::new().name("vhci reader".to_owned()).spawn(move || {
            let mut buf = vec![0u8; 4096];
            loop {
                match reader.read(&mut buf) {
                    Ok(0) | Err(_) => return,
                    Ok(n) => {
                        if packets.send(buf[..n].to_vec()).is_err() {
                            return;
                        }
                    }
                }
            }
        })?;
        let carrier = Carrier::with_output(Box::new(vhci), log, verbose, index);
        let serving = carrier.clone();
        thread::Builder::new().name("ble carrier".to_owned()).spawn(move || serving.serve(&rx))?;
        power_up(index).map_err(|e| Error::Other(format!("hci{index}: up: {e}")))?;
        Ok(carrier)
    }

    fn with_output(
        out: Box<dyn Write + Send>,
        log: Arc<Mutex<dyn Log>>,
        verbose: bool,
        index: u16,
    ) -> Carrier {
        let violations = Arc::new(AtomicUsize::new(0));
        let inner = Inner {
            out,
            model: None,
            log,
            scanning: false,
            reported: false,
            connecting: None,
            link: None,
            armed: None,
            notifications: 0,
            unplugged: false,
            last_tick: Instant::now(),
            verbose,
            own: OWN,
            violations: Arc::clone(&violations),
        };
        Carrier {
            inner: Arc::new(Mutex::new(inner)),
            index,
            violations,
            stop: Arc::new(AtomicBool::new(false)),
        }
    }

    /// The controller's index, N of `hciN`.
    pub fn index(&self) -> u16 {
        self.index
    }

    /// The violations it logged.
    pub fn violations(&self) -> usize {
        self.violations.load(Ordering::SeqCst)
    }

    /// Ends the thread.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::SeqCst);
    }

    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner.lock().expect("carrier lock")
    }

    fn serve(&self, rx: &Receiver<Vec<u8>>) {
        while !self.stop.load(Ordering::SeqCst) {
            match rx.recv_timeout(Duration::from_millis(5)) {
                Ok(packet) => self.lock().packet(&packet),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return,
            }
            self.lock().round();
        }
    }

    /// A fresh peripheral: the link goes, in silence, and so does what was
    /// armed.
    pub fn install(&self, model: Box<dyn Peripheral>) {
        let mut inner = self.lock();
        if inner.link.is_some() {
            inner.drop_link(REMOTE_USER_TERMINATED);
        }
        inner.model = Some(model);
        inner.armed = None;
        inner.notifications = 0;
        inner.unplugged = false;
        inner.reported = false;
    }

    /// The address of the peripheral installed, as written.
    pub fn address(&self) -> Option<[u8; 6]> {
        self.lock().model.as_ref().map(|m| m.address())
    }

    /// Waits for the host to drop the link, with a bound.
    pub fn sync(&self) {
        let began = Instant::now();
        while self.lock().link.is_some() && began.elapsed() < DRAIN {
            thread::sleep(Duration::from_millis(5));
        }
    }

    /// The peripheral is on the air again.
    pub fn replug(&self) {
        let mut inner = self.lock();
        inner.unplugged = false;
        inner.reported = false;
    }

    /// Arms an event at `ble-notify=N` or `ble-model`.
    ///
    /// # Errors
    ///
    /// On another point, or an N of 0.
    pub fn arm(&self, event: Event, when: &str) -> Result<()> {
        let when = if when == "ble-model" {
            When::Model
        } else {
            let n =
                when.strip_prefix("ble-notify=").and_then(|n| n.parse().ok()).filter(|n| *n > 0);
            When::Notified(n.ok_or_else(|| {
                Error::Invalid(format!("{when}: a BLE point is ble-notify=N or ble-model"))
            })?)
        };
        let mut inner = self.lock();
        inner.armed = Some((event, when));
        inner.notifications = 0;
        Ok(())
    }

    /// The session's `sync`, `arm` and `replug` for a session with a carrier.
    #[must_use]
    pub fn commands(&self, builder: Builder) -> Builder {
        let (sync, arm, replug) = (self.clone(), self.clone(), self.clone());
        builder
            .command("sync", "as ever, and the tool's link has gone", move |_, _| {
                sync.sync();
                Ok(json!({}))
            })
            .command(
                "arm",
                "event=unplug or event=mark name=NAME, when=ble-notify=N or ble-model",
                move |args, ctx| arm_command(&arm, args, ctx),
            )
            .command("replug", "an unplugged peripheral comes back", move |_, ctx: &mut Ctx| {
                replug.replug();
                ctx.replug().map(|()| json!({}))
            })
    }
}

fn arm_command(carrier: &Carrier, args: &Value, ctx: &mut Ctx) -> Result<Value> {
    let text = |key| args.get(key).and_then(Value::as_str);
    let (Some(kind), Some(when)) = (text("event"), text("when")) else {
        return Err(Error::Invalid("arm takes event=, name= for a mark, and when=".into()));
    };
    if when.starts_with("ble-") {
        let event = match (kind, text("name")) {
            ("mark", Some(name)) => Event::Mark(name.to_owned()),
            ("unplug", _) => Event::Unplug,
            _ => return Err(Error::Invalid(format!("no event {kind}"))),
        };
        carrier.arm(event, when)?;
    } else {
        let spec = match (kind, text("name")) {
            ("mark", Some(name)) => format!("mark {name} {when}"),
            _ => format!("{kind} {when}"),
        };
        ctx.bus().arm(Arm::parse(&spec)?);
    }
    Ok(json!({}))
}

/// The address as the host reads it: little endian.
fn wire_addr(a: [u8; 6]) -> [u8; 6] {
    let mut w = a;
    w.reverse();
    w
}

fn le16(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}

impl Inner {
    fn line(&self, text: &str) {
        self.log.lock().expect("log lock").line(text);
    }

    fn note(&self, text: &str) {
        if self.verbose {
            eprintln!("emu: ble: {text}");
        }
    }

    fn violation(&self, message: &str) {
        self.violations.fetch_add(1, Ordering::SeqCst);
        self.log.lock().expect("log lock").violation(message);
    }

    fn send(&mut self, packet: &[u8]) {
        // One write is one packet to hci_vhci. A host gone (the controller
        // being removed) leaves nothing to tell.
        let _ = self.out.write_all(packet);
        let _ = self.out.flush();
    }

    fn event(&mut self, code: u8, params: &[u8]) {
        let mut p = vec![EVT, code, params.len() as u8];
        p.extend_from_slice(params);
        self.send(&p);
    }

    fn complete(&mut self, opcode: u16, ret: &[u8]) {
        let mut p = vec![1];
        p.extend_from_slice(&opcode.to_le_bytes());
        p.extend_from_slice(ret);
        self.event(0x0e, &p);
    }

    fn status(&mut self, opcode: u16, status: u8) {
        let mut p = vec![status, 1];
        p.extend_from_slice(&opcode.to_le_bytes());
        self.event(0x0f, &p);
    }

    fn meta(&mut self, sub: u8, params: &[u8]) {
        let mut p = vec![sub];
        p.extend_from_slice(params);
        self.event(0x3e, &p);
    }

    /// An ATT PDU to the host, on the link.
    fn att_out(&mut self, pdu: &[u8]) {
        let mut l2 = Vec::with_capacity(pdu.len() + 4);
        l2.extend_from_slice(&(pdu.len() as u16).to_le_bytes());
        l2.extend_from_slice(&CID_ATT.to_le_bytes());
        l2.extend_from_slice(pdu);
        self.acl_out(&l2);
    }

    fn l2cap_out(&mut self, cid: u16, payload: &[u8]) {
        let mut l2 = Vec::with_capacity(payload.len() + 4);
        l2.extend_from_slice(&(payload.len() as u16).to_le_bytes());
        l2.extend_from_slice(&cid.to_le_bytes());
        l2.extend_from_slice(payload);
        self.acl_out(&l2);
    }

    fn acl_out(&mut self, l2: &[u8]) {
        // A first, automatically flushable packet: PB 10.
        let mut p = vec![ACL];
        p.extend_from_slice(&(HANDLE | 0x2000).to_le_bytes());
        p.extend_from_slice(&(l2.len() as u16).to_le_bytes());
        p.extend_from_slice(l2);
        self.send(&p);
    }

    /// A packet from the host.
    fn packet(&mut self, p: &[u8]) {
        match p.first() {
            Some(&CMD) if p.len() >= 4 => {
                let opcode = le16(p, 1);
                let len = usize::from(p[3]).min(p.len() - 4);
                self.command(opcode, &p[4..4 + len]);
            }
            Some(&ACL) if p.len() >= 5 => self.acl_in(p),
            _ => self.note(&format!("packet ignored: {}", hex(p))),
        }
    }

    /// The controller's supported commands (Read Local Supported Commands).
    fn commands_mask() -> [u8; 64] {
        let mut m = [0u8; 64];
        m[0] |= 0x20; // Disconnect
        m[5] |= 0x40 | 0x80; // Set Event Mask, Reset
        m[14] |= 0x08 | 0x20; // Read Local Version, Read Local Features
        m[15] |= 0x02 | 0x20; // Read BD ADDR, Read RSSI
        // LE Set Event Mask, Read Buffer Size, Read Local Features, Set Random
        // Address, Set Advertising Parameters, Set Advertising Data.
        m[25] |= 0x01 | 0x02 | 0x04 | 0x10 | 0x20 | 0x80;
        // LE Set Scan Response Data, Set Advertise Enable, Set Scan Parameters,
        // Set Scan Enable, Create Connection, Create Connection Cancel.
        m[26] |= 0x01 | 0x02 | 0x04 | 0x08 | 0x10 | 0x20;
        // LE Connection Update, Read Remote Features.
        m[27] |= 0x04 | 0x20;
        // LE Read Supported States.
        m[28] |= 0x08;
        m
    }

    #[allow(clippy::too_many_lines)]
    fn command(&mut self, opcode: u16, params: &[u8]) {
        self.note(&format!("command {opcode:04x} {}", hex(params)));
        match opcode {
            // Reset.
            0x0c03 => {
                self.scanning = false;
                self.connecting = None;
                if self.link.is_some() {
                    self.link = None;
                    if let Some(m) = &mut self.model {
                        m.disconnected();
                    }
                }
                self.complete(opcode, &[0]);
            }
            // Set Event Mask, Set Event Mask Page 2, Write LE Host Supported,
            // Write Default Erroneous Data Reporting, the LE setters and the
            // accept list.
            0x0c01 | 0x0c63 | 0x0c6d | 0x0c5b | 0x2001 | 0x2005 | 0x2006 | 0x2008 | 0x2009
            | 0x200a | 0x200b | 0x2010 | 0x2011 | 0x2012 | 0x2024 | 0x2031 => {
                self.complete(opcode, &[0]);
            }
            // Read Local Version: 4.0, the Linux Foundation's id, as btdev's.
            0x1001 => self.complete(opcode, &[0, 0x06, 0, 0, 0x06, 0xf1, 0x05, 0, 0]),
            // Read Local Supported Commands.
            0x1002 => {
                let mut r = vec![0];
                r.extend_from_slice(&Self::commands_mask());
                self.complete(opcode, &r);
            }
            // Read Local Supported Features: BR/EDR not supported, LE supported.
            0x1003 => self.complete(opcode, &[0, 0, 0, 0, 0, 0x60, 0, 0, 0]),
            // Read BD ADDR.
            0x1009 => {
                let mut r = vec![0];
                r.extend_from_slice(&wire_addr(self.own));
                self.complete(opcode, &r);
            }
            // Read RSSI.
            0x1405 => {
                let mut r = vec![0];
                r.extend_from_slice(params.get(..2).unwrap_or(&[0, 0]));
                r.push((-50i8) as u8);
                self.complete(opcode, &r);
            }
            // LE Read Buffer Size: 27 bytes, 8 packets.
            0x2002 => {
                let mut r = vec![0];
                r.extend_from_slice(&LE_MTU.to_le_bytes());
                r.push(8);
                self.complete(opcode, &r);
            }
            // LE Read Local Supported Features: none of the optional ones.
            0x2003 => self.complete(opcode, &[0; 9]),
            // LE Read Advertising Channel TX Power, LE Read Accept List Size (none).
            0x2007 | 0x200f => self.complete(opcode, &[0, 0]),
            // LE Set Scan Enable.
            0x200c => {
                let on = params.first() == Some(&1);
                if on && !self.scanning {
                    self.reported = false;
                }
                self.scanning = on;
                self.complete(opcode, &[0]);
            }
            // LE Create Connection.
            0x200d => self.create_connection(opcode, params),
            // LE Create Connection Cancel.
            0x200e => {
                self.complete(opcode, &[0]);
                if let Some(peer) = self.connecting.take() {
                    self.connection_complete(0x02, peer);
                }
            }
            // LE Connection Update.
            0x2013 => {
                self.status(opcode, 0);
                if self.link.is_some() && params.len() >= 14 {
                    let mut p = vec![0];
                    p.extend_from_slice(&HANDLE.to_le_bytes());
                    p.extend_from_slice(&params[4..6]); // the maximum interval
                    p.extend_from_slice(&params[8..12]); // latency, timeout
                    self.meta(0x03, &p);
                }
            }
            // LE Read Remote Features.
            0x2016 => {
                if self.link.is_some() && params.get(..2) == Some(&HANDLE.to_le_bytes()[..]) {
                    self.status(opcode, 0);
                    if let Some(link) = &mut self.link {
                        link.features = true;
                    }
                } else {
                    self.status(opcode, 0x02);
                }
            }
            // LE Read Supported States: all of them.
            0x201c => self.complete(opcode, &[0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0, 0]),
            // Disconnect.
            0x0406 => {
                let ours = params.get(..2) == Some(&HANDLE.to_le_bytes()[..]);
                if self.link.is_some() && ours {
                    self.status(opcode, 0);
                    let reason = params.get(2).copied().unwrap_or(REMOTE_USER_TERMINATED);
                    // The host's own end: Connection Terminated by Local Host.
                    let mut p = vec![0];
                    p.extend_from_slice(&HANDLE.to_le_bytes());
                    p.push(0x16);
                    self.event(0x05, &p);
                    self.end_link(&format!("ble_disconnect host {reason:02x}"));
                } else {
                    self.status(opcode, 0x02);
                }
            }
            _ => {
                eprintln!("oxrok-emu: ble: unknown HCI command {opcode:04x} {}", hex(params));
                self.complete(opcode, &[0x01]);
            }
        }
    }

    fn create_connection(&mut self, opcode: u16, params: &[u8]) {
        if params.len() < 25 || self.connecting.is_some() || self.link.is_some() {
            self.status(opcode, 0x0c);
            return;
        }
        self.status(opcode, 0);
        let mut peer = [0u8; 6];
        peer.copy_from_slice(&params[6..12]);
        peer.reverse();
        self.scanning = false;
        let (present, status, address) = match &self.model {
            Some(m) => (m.present() && !self.unplugged, m.connect_status(), m.address()),
            None => (false, 0, [0; 6]),
        };
        if !present || peer != address {
            // Nobody answers: the host cancels when its time is up.
            self.connecting = Some(peer);
            return;
        }
        if status != 0 {
            self.connection_complete(status, peer);
            return;
        }
        self.connection_complete(0, peer);
    }

    fn connection_complete(&mut self, status: u8, peer: [u8; 6]) {
        let mut p = vec![status];
        p.extend_from_slice(&HANDLE.to_le_bytes());
        p.push(0x00); // central
        p.push(0x00); // public
        p.extend_from_slice(&wire_addr(peer));
        p.extend_from_slice(&0x0018u16.to_le_bytes()); // 30 ms
        p.extend_from_slice(&0u16.to_le_bytes());
        p.extend_from_slice(&0x0048u16.to_le_bytes()); // 720 ms
        p.push(0x00);
        self.meta(0x01, &p);
        if status == 0 {
            self.link = Some(Link::default());
            self.line("ble_connect");
            self.with_model(|m, p| m.connected(p));
        }
    }

    /// The link is gone; `what` says how, in the log.
    fn end_link(&mut self, what: &str) {
        self.link = None;
        self.line(what);
        if let Some(m) = &mut self.model {
            m.disconnected();
        }
    }

    /// The peripheral drops the link, with `reason`, without a word in the log.
    fn drop_link(&mut self, reason: u8) {
        let mut p = vec![0];
        p.extend_from_slice(&HANDLE.to_le_bytes());
        p.push(reason);
        self.event(0x05, &p);
        self.link = None;
        if let Some(m) = &mut self.model {
            m.disconnected();
        }
    }

    fn acl_in(&mut self, p: &[u8]) {
        let hf = le16(p, 1);
        let handle = hf & 0x0fff;
        let pb = (hf >> 12) & 0x3;
        let len = usize::from(le16(p, 3)).min(p.len() - 5);
        let data = &p[5..5 + len];
        // Acknowledged at once: the host counts its buffers by these.
        let mut nc = vec![1];
        nc.extend_from_slice(&handle.to_le_bytes());
        nc.extend_from_slice(&1u16.to_le_bytes());
        self.event(0x13, &nc);
        if handle != HANDLE || self.link.is_none() {
            self.note(&format!("ACL for no link: {}", hex(p)));
            return;
        }
        let Some(link) = &mut self.link else { return };
        let frame = if pb == 0x1 {
            // A continuation.
            let Some((want, got)) = &mut link.partial else {
                self.violation("an ACL continuation with no start");
                return;
            };
            got.extend_from_slice(data);
            if got.len() < *want {
                return;
            }
            link.partial.take().expect("partial").1
        } else {
            if data.len() < 4 {
                self.violation(&format!("an L2CAP frame of {} bytes", data.len()));
                return;
            }
            let want = usize::from(le16(data, 0)) + 4;
            if data.len() < want {
                link.partial = Some((want, data.to_vec()));
                return;
            }
            data.to_vec()
        };
        let cid = le16(&frame, 2);
        let payload = &frame[4..];
        match cid {
            CID_ATT => self.att_in(payload),
            CID_SMP => {
                // Pairing Failed: pairing not supported.
                self.l2cap_out(CID_SMP, &[0x05, 0x05]);
            }
            CID_LE_SIGNAL => {
                // Command Reject, not understood, with the request's id.
                let id = payload.get(1).copied().unwrap_or(0);
                self.l2cap_out(CID_LE_SIGNAL, &[0x01, id, 0x02, 0x00, 0x00, 0x00]);
            }
            _ => self.violation(&format!("L2CAP on channel {cid:04x}")),
        }
    }

    fn attributes(&self) -> Vec<Attribute> {
        self.model.as_ref().map(|m| m.attributes()).unwrap_or_default()
    }

    fn att_error(request: u8, handle: u16, code: u8) -> Vec<u8> {
        let mut e = vec![0x01, request];
        e.extend_from_slice(&handle.to_le_bytes());
        e.push(code);
        e
    }

    #[allow(clippy::too_many_lines)]
    fn att_in(&mut self, pdu: &[u8]) {
        let Some(&op) = pdu.first() else {
            self.violation("an empty ATT PDU");
            return;
        };
        let table = self.attributes();
        let find = |h: u16| table.iter().find(|a| a.handle == h);
        match op {
            // Write Request, Write Command.
            0x12 | 0x52 if pdu.len() >= 3 => {
                let handle = le16(pdu, 1);
                let value = &pdu[3..];
                let req = op == 0x12;
                let word = if req {
                    format!("att_write {handle:04x} {}", hex(value))
                } else {
                    format!("att_write_cmd {handle:04x} {}", hex(value))
                };
                let Some(attr) = find(handle) else {
                    if req {
                        let e = Self::att_error(op, handle, 0x01);
                        self.line(&format!("{word} -> err 01"));
                        self.att_out(&e);
                    } else {
                        self.line(&word);
                    }
                    self.violation(&format!("a write to no attribute, {handle:04x}"));
                    return;
                };
                let allowed = if req { attr.access.write } else { attr.access.write_cmd };
                if !allowed {
                    if req {
                        let e = Self::att_error(op, handle, 0x03);
                        self.line(&format!("{word} -> err 03"));
                        self.att_out(&e);
                    } else {
                        self.line(&word);
                    }
                    self.violation(&format!("{handle:04x} cannot be written so"));
                    return;
                }
                if attr.kind == Uuid::Short(CCCD) {
                    if value.len() != 2 {
                        let e = Self::att_error(op, handle, 0x0d);
                        self.line(&format!("{word} -> err 0d"));
                        self.att_out(&e);
                        self.violation("a CCCD of other than 2 bytes");
                        return;
                    }
                    // The characteristic's value is the handle before it.
                    let of = handle - 1;
                    if let Some(link) = &mut self.link {
                        link.subscribed.retain(|h| *h != of);
                        if value[0] & 0x01 != 0 {
                            link.subscribed.push(of);
                        }
                    }
                    if req {
                        self.line(&format!("{word} -> ok"));
                        self.att_out(&[0x13]);
                    } else {
                        self.line(&word);
                    }
                    return;
                }
                if req {
                    self.line(&format!("{word} -> ok"));
                    self.att_out(&[0x13]);
                } else {
                    self.line(&word);
                }
                let value = value.to_vec();
                self.with_model(|m, p| m.written(handle, &value, p));
            }
            // Handle Value Confirmation: nothing indicated, so none is due.
            0x1e => {
                self.line(&format!("att_in {}", hex(pdu)));
                self.violation("a confirmation with no indication");
            }
            _ => {
                let answer = self.att_request(op, pdu, &table);
                self.line(&format!("att_req {} -> {}", hex(pdu), hex(&answer)));
                self.att_out(&answer);
            }
        }
    }

    /// The answer to a request the table answers: discovery and reads.
    fn att_request(&mut self, op: u8, pdu: &[u8], table: &[Attribute]) -> Vec<u8> {
        let range = |pdu: &[u8]| (pdu.len() >= 5).then(|| (le16(pdu, 1), le16(pdu, 3)));
        match op {
            // Exchange MTU: this server keeps to 23.
            0x02 if pdu.len() == 3 => {
                let mut r = vec![0x03];
                r.extend_from_slice(&(ATT_MTU as u16).to_le_bytes());
                r
            }
            // Find Information.
            0x04 => {
                let Some((start, end)) = range(pdu).filter(|(s, e)| *s != 0 && s <= e) else {
                    return Self::att_error(op, 0, 0x01);
                };
                let mut found = table.iter().filter(|a| a.handle >= start && a.handle <= end);
                let Some(first) = found.next() else {
                    return Self::att_error(op, start, 0x0a);
                };
                let format = if first.kind.wire().len() == 2 { 1 } else { 2 };
                let mut r = vec![0x05, format];
                for a in std::iter::once(first).chain(found) {
                    let w = a.kind.wire();
                    if (w.len() == 2) != (format == 1) || r.len() + 2 + w.len() > ATT_MTU {
                        break;
                    }
                    r.extend_from_slice(&a.handle.to_le_bytes());
                    r.extend_from_slice(&w);
                }
                r
            }
            // Read By Type, Read By Group Type.
            0x08 | 0x10 => {
                let Some((start, end)) = range(pdu).filter(|(s, e)| *s != 0 && s <= e) else {
                    return Self::att_error(op, 0, 0x01);
                };
                let Some(kind) = Uuid::from_wire(&pdu[5..]) else {
                    return Self::att_error(op, start, 0x04);
                };
                if op == 0x10 && kind != Uuid::Short(PRIMARY_SERVICE) {
                    return Self::att_error(op, start, 0x10);
                }
                let matching: Vec<&Attribute> = table
                    .iter()
                    .filter(|a| a.handle >= start && a.handle <= end && a.kind == kind)
                    .collect();
                let Some(first) = matching.first() else {
                    return Self::att_error(op, start, 0x0a);
                };
                let group_end = |a: &Attribute| {
                    table
                        .iter()
                        .filter(|b| b.handle > a.handle && b.kind == Uuid::Short(PRIMARY_SERVICE))
                        .map(|b| b.handle - 1)
                        .next()
                        .unwrap_or(0xffff)
                };
                let item = |a: &Attribute| {
                    let mut v = a.handle.to_le_bytes().to_vec();
                    if op == 0x10 {
                        v.extend_from_slice(&group_end(a).to_le_bytes());
                    }
                    v.extend_from_slice(&a.value);
                    v
                };
                let size = item(first).len().min(ATT_MTU - 2);
                let mut r = vec![op + 1, size as u8];
                for a in matching {
                    let mut v = item(a);
                    if v.len() != size && r.len() > 2 {
                        break;
                    }
                    v.truncate(size);
                    if r.len() + v.len() > ATT_MTU {
                        break;
                    }
                    r.extend_from_slice(&v);
                }
                r
            }
            // Read.
            0x0a if pdu.len() == 3 => {
                let handle = le16(pdu, 1);
                match table.iter().find(|a| a.handle == handle) {
                    None => Self::att_error(op, handle, 0x01),
                    Some(a) if !a.access.read => Self::att_error(op, handle, 0x02),
                    Some(a) if a.kind == Uuid::Short(CCCD) => {
                        let on = self
                            .link
                            .as_ref()
                            .is_some_and(|l| l.subscribed.contains(&(handle - 1)));
                        vec![0x0b, u8::from(on), 0]
                    }
                    Some(a) => {
                        let mut r = vec![0x0b];
                        r.extend_from_slice(&a.value[..a.value.len().min(ATT_MTU - 1)]);
                        r
                    }
                }
            }
            _ => Self::att_error(op, 0, 0x06),
        }
    }

    /// Runs a call of the peripheral and does what it asked.
    fn with_model(&mut self, call: impl FnOnce(&mut dyn Peripheral, &mut Port)) {
        let Some(mut model) = self.model.take() else { return };
        let subscribed = self.link.as_ref().map(|l| l.subscribed.clone()).unwrap_or_default();
        let mut port = Port::new(Instant::now(), &subscribed);
        call(model.as_mut(), &mut port);
        self.model = Some(model);
        for message in port.v.drain() {
            self.violation(&message);
        }
        for (handle, value) in port.notified {
            if self.link.is_none() {
                break;
            }
            if !subscribed.contains(&handle) {
                continue;
            }
            let mut pdu = vec![0x1b];
            pdu.extend_from_slice(&handle.to_le_bytes());
            pdu.extend_from_slice(&value);
            self.line(&format!("att_notify {handle:04x} {}", hex(&value)));
            self.att_out(&pdu);
            self.notifications += 1;
            if matches!(self.armed, Some((_, When::Notified(n))) if n == self.notifications) {
                self.fire();
            }
        }
        if port.mark && matches!(self.armed, Some((_, When::Model))) {
            self.fire();
        }
        if port.hang_up && self.link.is_some() {
            let mut p = vec![0];
            p.extend_from_slice(&HANDLE.to_le_bytes());
            p.push(REMOTE_USER_TERMINATED);
            self.event(0x05, &p);
            self.end_link(&format!("ble_disconnect device {REMOTE_USER_TERMINATED:02x}"));
        }
    }

    /// The armed event happens: its word in the log, its line on standard
    /// output, and for an unplug the end of the link.
    fn fire(&mut self) {
        let Some((event, _)) = self.armed.take() else { return };
        self.line(event.word());
        println!("EVENT {}", event.word());
        if event == Event::Unplug {
            self.unplugged = true;
            if self.link.is_some() {
                let mut p = vec![0];
                p.extend_from_slice(&HANDLE.to_le_bytes());
                p.push(SUPERVISION_TIMEOUT);
                self.event(0x05, &p);
                self.link = None;
                if let Some(m) = &mut self.model {
                    m.disconnected();
                }
            }
        }
    }

    /// What is due without a packet: an advertising report, a remote features
    /// answer, a tick.
    fn round(&mut self) {
        if self.link.as_ref().is_some_and(|l| l.features) {
            if let Some(link) = &mut self.link {
                link.features = false;
            }
            let mut p = vec![0];
            p.extend_from_slice(&HANDLE.to_le_bytes());
            p.extend_from_slice(&[0; 8]);
            self.meta(0x04, &p);
        }
        if self.scanning
            && !self.reported
            && self.link.is_none()
            && !self.unplugged
            && let Some(m) = &self.model
            && m.present()
        {
            let (address, data) = (m.address(), m.advertising());
            self.reported = true;
            let mut p = vec![1, 0x00, 0x00]; // one report: ADV_IND, public
            p.extend_from_slice(&wire_addr(address));
            p.push(data.len() as u8);
            p.extend_from_slice(&data);
            p.push((-60i8) as u8);
            self.meta(0x02, &p);
        }
        if self.link.is_some() && self.last_tick.elapsed() >= TICK {
            self.last_tick = Instant::now();
            self.with_model(|m, p| m.tick(p));
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::*;

    /// A queue a test reads the controller's packets from.
    #[derive(Clone, Default)]
    struct Wire(Arc<Mutex<VecDeque<Vec<u8>>>>);

    impl Write for Wire {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.0.lock().expect("wire").push_back(buf.to_vec());
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[derive(Default)]
    struct Lines(Vec<String>);

    impl Log for Lines {
        fn line(&mut self, text: &str) {
            self.0.push(text.to_owned());
        }
        fn violation(&mut self, message: &str) {
            self.0.push(format!("! {message}"));
        }
        fn rules(&self) -> crate::txlog::Rules {
            crate::txlog::Rules::default()
        }
    }

    const ADDR: [u8; 6] = [0xd0, 0x5f, 0xb8, 0, 0, 1];

    /// Echoes what is written to 0x0012 as a notification of 0x0015; `h` hangs
    /// up, `m` marks.
    struct Echo;

    impl Peripheral for Echo {
        fn address(&self) -> [u8; 6] {
            ADDR
        }
        fn advertising(&self) -> Vec<u8> {
            vec![2, 1, 6]
        }
        fn attributes(&self) -> Vec<Attribute> {
            let svc = Uuid::long("1bc5ffa0-0200-62ab-e411-f254e005dbd4");
            let write = Access { write: true, ..Access::default() };
            vec![
                Attribute::service(0x0010, &svc),
                Attribute::characteristic(0x0011, 0x08, 0x0012, &Uuid::Short(0xffa1)),
                Attribute::value(0x0012, Uuid::Short(0xffa1), &[], write),
                Attribute::characteristic(0x0014, 0x10, 0x0015, &Uuid::Short(0xffa2)),
                Attribute::value(0x0015, Uuid::Short(0xffa2), &[], Access::default()),
                Attribute::cccd(0x0016),
            ]
        }
        fn written(&mut self, handle: u16, value: &[u8], port: &mut Port) {
            assert_eq!(handle, 0x0012);
            match value {
                b"h" => port.hang_up(),
                b"m" => port.mark(),
                _ => port.notify(0x0015, value),
            }
        }
    }

    struct Rig {
        carrier: Carrier,
        wire: Wire,
        lines: Arc<Mutex<Lines>>,
    }

    fn rig() -> Rig {
        let lines = Arc::new(Mutex::new(Lines::default()));
        let log: Arc<Mutex<dyn Log>> = lines.clone();
        let wire = Wire::default();
        let carrier = Carrier::with_output(Box::new(wire.clone()), log, false, 0);
        carrier.install(Box::new(Echo));
        Rig { carrier, wire, lines }
    }

    impl Rig {
        fn cmd(&self, opcode: u16, params: &[u8]) {
            let mut p = vec![CMD];
            p.extend_from_slice(&opcode.to_le_bytes());
            p.push(params.len() as u8);
            p.extend_from_slice(params);
            self.carrier.lock().packet(&p);
        }
        fn att(&self, pdu: &[u8]) {
            let mut p = vec![ACL];
            p.extend_from_slice(&(HANDLE | 0x2000).to_le_bytes());
            p.extend_from_slice(&((pdu.len() + 4) as u16).to_le_bytes());
            p.extend_from_slice(&(pdu.len() as u16).to_le_bytes());
            p.extend_from_slice(&CID_ATT.to_le_bytes());
            p.extend_from_slice(pdu);
            self.carrier.lock().packet(&p);
        }
        fn round(&self) {
            self.carrier.lock().round();
        }
        fn take(&self) -> Vec<Vec<u8>> {
            self.wire.0.lock().unwrap().drain(..).collect()
        }
        /// The ATT PDUs among the packets to the host.
        fn att_out(&self) -> Vec<Vec<u8>> {
            self.take().into_iter().filter(|p| p[0] == ACL).map(|p| p[9..].to_vec()).collect()
        }
        fn lines(&self) -> Vec<String> {
            self.lines.lock().unwrap().0.clone()
        }
        fn connect(&self) {
            self.cmd(0x200c, &[1, 1]);
            self.round();
            let mut cc = vec![0x60, 0, 0x30, 0, 0, 0];
            cc.extend_from_slice(&wire_addr(ADDR));
            cc.extend_from_slice(&[0; 13]);
            self.cmd(0x200d, &cc);
            self.round();
            self.take();
        }
    }

    #[test]
    fn a_scan_reports_the_peripheral_once_and_a_connection_completes() {
        let r = rig();
        r.cmd(0x200c, &[1, 1]);
        r.round();
        r.round();
        let reports: Vec<Vec<u8>> =
            r.take().into_iter().filter(|p| p[1] == 0x3e && p[3] == 0x02).collect();
        assert_eq!(reports.len(), 1);
        assert_eq!(reports[0][7..13], wire_addr(ADDR));
        let mut cc = vec![0x60, 0, 0x30, 0, 0, 0];
        cc.extend_from_slice(&wire_addr(ADDR));
        cc.extend_from_slice(&[0; 13]);
        r.cmd(0x200d, &cc);
        let out = r.take();
        assert_eq!(out[0], [EVT, 0x0f, 4, 0, 1, 0x0d, 0x20], "command status");
        assert_eq!(out[1][..7], [EVT, 0x3e, 19, 0x01, 0, 0x40, 0x00], "connection complete");
        r.cmd(0x2016, &HANDLE.to_le_bytes());
        r.round();
        let out = r.take();
        assert_eq!(out[1][..4], [EVT, 0x3e, 12, 0x04], "remote features complete");
        assert_eq!(r.lines(), ["ble_connect"]);
    }

    #[test]
    fn a_connection_to_another_address_waits_for_the_cancel() {
        let r = rig();
        let mut cc = vec![0x60, 0, 0x30, 0, 0, 0];
        cc.extend_from_slice(&[1, 2, 3, 4, 5, 6]);
        cc.extend_from_slice(&[0; 13]);
        r.cmd(0x200d, &cc);
        assert_eq!(r.take().len(), 1, "a command status alone");
        r.cmd(0x200e, &[]);
        let out = r.take();
        assert_eq!(out[1][..5], [EVT, 0x3e, 19, 0x01, 0x02], "failed: unknown connection");
        assert_eq!(r.lines(), Vec::<String>::new());
    }

    #[test]
    fn notifications_go_out_only_once_turned_on() {
        let r = rig();
        r.connect();
        r.att(&[0x12, 0x12, 0x00, b'a']);
        assert_eq!(r.att_out(), [vec![0x13]], "a write response, no notification");
        r.att(&[0x12, 0x16, 0x00, 0x01, 0x00]);
        assert_eq!(r.att_out(), [vec![0x13]]);
        r.att(&[0x12, 0x12, 0x00, b'b']);
        assert_eq!(r.att_out(), [vec![0x13], vec![0x1b, 0x15, 0x00, b'b']]);
        assert_eq!(
            r.lines(),
            [
                "ble_connect",
                "att_write 0012 61 -> ok",
                "att_write 0016 0100 -> ok",
                "att_write 0012 62 -> ok",
                "att_notify 0015 62",
            ]
        );
    }

    #[test]
    fn every_host_packet_is_acknowledged() {
        let r = rig();
        r.connect();
        r.att(&[0x02, 0x17, 0x00]);
        let out = r.take();
        assert_eq!(out[0], [EVT, 0x13, 5, 1, 0x40, 0x00, 1, 0]);
        assert_eq!(out[1][9..], [0x03, 23, 0]);
    }

    #[test]
    fn discovery_answers_from_the_table() {
        let r = rig();
        r.connect();
        r.att(&[0x10, 0x01, 0x00, 0xff, 0xff, 0x00, 0x28]);
        let svc = r.att_out();
        assert_eq!(svc[0][..6], [0x11, 20, 0x10, 0x00, 0xff, 0xff]);
        r.att(&[0x08, 0x10, 0x00, 0xff, 0xff, 0x03, 0x28]);
        let chars = r.att_out();
        assert_eq!(
            chars[0],
            [0x09, 7, 0x11, 0, 0x08, 0x12, 0, 0xa1, 0xff, 0x14, 0, 0x10, 0x15, 0, 0xa2, 0xff]
        );
        r.att(&[0x04, 0x16, 0x00, 0x16, 0x00]);
        assert_eq!(r.att_out()[0], [0x05, 1, 0x16, 0, 0x02, 0x29]);
        r.att(&[0x0a, 0x16, 0x00]);
        assert_eq!(r.att_out()[0], [0x0b, 0, 0]);
        r.att(&[0x20]);
        assert_eq!(r.att_out()[0], [0x01, 0x20, 0, 0, 0x06]);
    }

    #[test]
    fn a_write_the_table_forbids_is_refused_and_flagged() {
        let r = rig();
        r.connect();
        r.att(&[0x12, 0x15, 0x00, 1]);
        assert_eq!(r.att_out(), [vec![0x01, 0x12, 0x15, 0x00, 0x03]]);
        r.att(&[0x52, 0x12, 0x00, 1]);
        assert_eq!(r.carrier.violations(), 2);
    }

    #[test]
    fn a_host_disconnect_and_a_hang_up_are_logged() {
        let r = rig();
        r.connect();
        r.cmd(0x0406, &[0x40, 0x00, 0x13]);
        let out = r.take();
        assert_eq!(out[1], [EVT, 0x05, 4, 0, 0x40, 0x00, 0x16]);
        r.connect();
        r.att(&[0x12, 0x12, 0x00, b'h']);
        let out = r.take();
        assert_eq!(out.last().unwrap(), &[EVT, 0x05, 4, 0, 0x40, 0x00, 0x13]);
        assert_eq!(
            r.lines(),
            [
                "ble_connect",
                "ble_disconnect host 13",
                "ble_connect",
                "att_write 0012 68 -> ok",
                "ble_disconnect device 13",
            ]
        );
    }

    #[test]
    fn an_unplug_after_a_notification_drops_the_link_and_keeps_it_off_the_air() {
        let r = rig();
        r.connect();
        r.att(&[0x12, 0x16, 0x00, 0x01, 0x00]);
        r.carrier.arm(Event::Unplug, "ble-notify=2").unwrap();
        r.att(&[0x12, 0x12, 0x00, b'a']);
        r.att(&[0x12, 0x12, 0x00, b'b']);
        let out = r.take();
        assert_eq!(out.last().unwrap(), &[EVT, 0x05, 4, 0, 0x40, 0x00, SUPERVISION_TIMEOUT]);
        assert_eq!(r.lines().last().unwrap(), "unplug");
        r.cmd(0x200c, &[0, 0]);
        r.cmd(0x200c, &[1, 1]);
        r.round();
        assert!(r.take().iter().all(|p| p[1] != 0x3e), "no report while unplugged");
        r.carrier.replug();
        r.round();
        assert!(r.take().iter().any(|p| p[1] == 0x3e), "a report once back");
        assert!(r.carrier.arm(Event::Unplug, "ble-notify=0").is_err());
        assert!(r.carrier.arm(Event::Unplug, "in=1:1").is_err());
    }

    #[test]
    fn a_mark_fires_what_waits_for_the_model() {
        let r = rig();
        r.connect();
        r.carrier.arm(Event::Mark("interrupt".into()), "ble-model").unwrap();
        r.att(&[0x12, 0x12, 0x00, b'm']);
        assert_eq!(r.lines().last().unwrap(), "interrupt");
    }

    #[test]
    fn a_fresh_peripheral_drops_the_link_in_silence() {
        let r = rig();
        r.connect();
        let before = r.lines();
        r.carrier.install(Box::new(Echo));
        assert_eq!(r.take(), [vec![EVT, 0x05, 4, 0, 0x40, 0x00, REMOTE_USER_TERMINATED]]);
        assert_eq!(r.lines(), before);
    }

    #[test]
    fn a_frame_in_two_packets_is_reassembled() {
        let r = rig();
        r.connect();
        let pdu = [0x12, 0x16, 0x00, 0x01, 0x00];
        let mut l2 = (pdu.len() as u16).to_le_bytes().to_vec();
        l2.extend_from_slice(&CID_ATT.to_le_bytes());
        l2.extend_from_slice(&pdu);
        for (pb, part) in [(0x2000u16, &l2[..6]), (0x1000, &l2[6..])] {
            let mut p = vec![ACL];
            p.extend_from_slice(&(HANDLE | pb).to_le_bytes());
            p.extend_from_slice(&(part.len() as u16).to_le_bytes());
            p.extend_from_slice(part);
            r.carrier.lock().packet(&p);
        }
        assert_eq!(r.att_out(), [vec![0x13]]);
    }

    #[test]
    fn the_init_commands_get_answers_of_their_size() {
        let r = rig();
        for (opcode, size) in [
            (0x0c03, 1),
            (0x1003, 9),
            (0x1001, 9),
            (0x1009, 7),
            (0x1002, 65),
            (0x2003, 9),
            (0x2002, 4),
            (0x201c, 9),
            (0x0c01, 1),
            (0x2001, 1),
        ] {
            r.cmd(opcode, &[]);
            let out = r.take();
            assert_eq!(out.len(), 1);
            assert_eq!(out[0][1], 0x0e, "{opcode:04x}: command complete");
            assert_eq!(le16(&out[0], 4), opcode);
            assert_eq!(out[0].len() - 6, size, "{opcode:04x}");
            assert_eq!(out[0][6], 0, "{opcode:04x}: success");
        }
    }

    #[test]
    fn uuids_go_on_the_air_little_endian() {
        let u = Uuid::long("1bc5ffa0-0200-62ab-e411-f254e005dbd4");
        assert_eq!(u.wire()[0], 0xd4);
        assert_eq!(u.wire()[15], 0x1b);
        assert_eq!(Uuid::from_wire(&u.wire()), Some(u));
    }
}
