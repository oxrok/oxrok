// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The Mooshimeter, a two channel Bluetooth LE multimeter, as a peripheral on
//! the BLE carrier (`shells::ble`): what libsigrok's `mooshimeter-dmm` reaches
//! through its `bt/` layer's L2CAP socket.
//!
//! Sources, and how sure this is:
//!
//! - The GATT handles are libsigrok's (`api.c:104-114`, read with
//!   btgatt-client off a real meter): the service at 0x0010, SERIN written at
//!   0x0012 (properties 0x08), SEROUT notified at 0x0015 (0x10) with its CCCD at
//!   0x0016, and user descriptions at 0x0013 and 0x0017. The UUIDs are the
//!   vendor's (`Mooshimeter.py` of Mooshimeter-PythonAPI). The GAP service and
//!   the descriptions' texts are this model's.
//! - The serial layer: each notification is a sequence number and 1 to 19 bytes
//!   of a byte stream, and each write likewise (`protocol.c:37-49`). The
//!   meter's numbers run on across connections; what it does with the host's is
//!   not known: the vendor's Python API always sends 0, libsigrok counts, and
//!   both work, so this model ignores it. A message is split over notifications
//!   as it comes, and one notification may hold several.
//! - The config tree: a node is a type, a name and children, each with a 7 bit
//!   id given depth first to all but plain and link nodes (`protocol.c:51-145`,
//!   `ConfigNode.py`'s `serialize`). The tree is libsigrok's listing of a meter
//!   (`protocol.c:170-265`), with an unnamed plain root as the vendor's code
//!   builds it. It goes out zlib compressed in ADMIN:TREE; the host writes the
//!   CRC-32 of the compressed bytes to ADMIN:CRC32, the meter echoes it, and only
//!   then answers the rest of the tree. Before that only ids 0 to 2 are open
//!   (`protocol.c:114-126`); the vendor's app reads ADMIN:CRC32 first, so a read
//!   of it answers the tree's CRC. The compressed bytes are this model's
//!   (flate2), not a real meter's: no real meter's are published.
//! - A read is `ID`, a write `0x80|ID VALUE`; the answer to either is `ID VALUE`
//!   with the value now held (`protocol.c:65-88`).
//! - Sampling: with SAMPLING:TRIGGER at CONTINUOUS the meter takes DEPTH samples
//!   at RATE and sends CH1:VALUE, CH2:VALUE and `REAL_PWR` after each buffer, in
//!   MEAN or RMS analysis; in BUFFER analysis it sends CH1:BUF and CH2:BUF, the
//!   samples as `BUF_BPS` bit integers times `BUF_LSB2NATIVE` (the vendor's app,
//!   `MooshimeterDevice.java` `interpretSampleBuffer`), then `REAL_PWR`. Which
//!   messages a buffer brings and in what order is this model's reading of the
//!   vendor's code, which waits on each by its callback; no firmware source is
//!   published. So is the period, DEPTH / RATE (64 / 125 Hz = 512 ms).
//! - TEMP inputs read kelvin (the vendor's app labels both channels' K), `AUX_V`
//!   and DIODE volts, RESISTANCE ohms.
//! - The meter drops a host it has not heard from for 20 s (the vendor's app,
//!   which polls `PCB_VERSION` every 10 s to keep it; libsigrok says 30 s, with a
//!   question mark).
//! - Not modelled: logging (LOG:*), the clock, rebooting, OFFSET's effect, a
//!   reading's noise, ranges that saturate.
//!
//! Options:
//!
//! - `addr=XX:XX:XX:XX:XX:XX`: the meter's address (default D0:5F:B8:4D:00:01).
//! - `seq=N`: the meter's first sequence number (default 0).
//! - `ch1=V,V...`, `ch2=V,V...`, `pwr=V,V...`: the readings, in turn (defaults
//!   by input: 0.25 -0.125 1.5 A, 12.5 12.25 12.75 V, 296.15 296.65 K, 0.05
//!   -0.0625 V aux, 4700 4725 ohm, 0.625 0.6 V diode; power is CH1 times CH2).
//! - `period=MS`: the time between readings (default DEPTH / RATE).
//! - `battery=N`: `BAT_V` comes, unasked, after every Nth reading.
//! - `diag=TEXT`: an ADMIN:DIAGNOSTIC message comes, unasked, after the first
//!   reading (`_` for a space).
//! - `lose=N`: the Nth notification since power on never goes out, its sequence
//!   number spent; `swap=N`: the Nth goes out after the next one; `dup=N`: the
//!   Nth goes out twice.
//! - `mute-at=N`: after N writes to SERIN the meter answers nothing more.
//! - `stop-at=N`: the meter sends no reading after the Nth (the link stays up).
//! - `mark-at=N`: the point `ble-model` waits for, after the Nth reading.
//! - `crc=bad`: the echo of the CRC is another value, and the meter stays locked.
//! - `tree=garbage`: ADMIN:TREE holds bytes that are not zlib; `tree=no-pwr`: a
//!   tree without `REAL_PWR`.
//! - `idle=S`: the silence after which the meter drops the host, 0 for never
//!   (default 20).
//! - `absent=1`: the meter is not on the air; `refuse=1`: a connection to it
//!   fails (0x3e, failed to be established).

use std::io::Write as _;
use std::time::{Duration, Instant};

use flate2::Compression;
use flate2::write::ZlibEncoder;

use crate::model::DeviceModel;
use crate::shells::ble::{Access, Attribute, Peripheral, Port, Uuid, USER_DESCRIPTION};
use crate::shells::tcp::NoUsb;
use crate::{Error, Result};

/// The model's name.
pub const NAME: &str = "mooshimeter-dmm";
/// One line for `oxrok-emu list`.
pub const ABOUT: &str = "Mooshimeter BLE multimeter, on the BLE carrier alone (addr=, seq=N, \
                         ch1=, ch2=, pwr=, period=MS, battery=N, diag=TEXT, lose=N, swap=N, \
                         dup=N, mute-at=N, stop-at=N, mark-at=N, crc=bad, \
                         tree=garbage|no-pwr, idle=S, absent=1, refuse=1)";

/// SERIN, the value the host writes.
pub const SERIN: u16 = 0x0012;
/// SEROUT, the value the meter notifies.
pub const SEROUT: u16 = 0x0015;
/// The bytes of a stream a notification or a write carries, after its
/// sequence number.
const CHUNK: usize = 19;

/// The node types of the tree (`protocol.c:93-106`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Plain = 0,
    Link = 1,
    Chooser = 2,
    U8 = 3,
    U16 = 4,
    U32 = 5,
    // The signed types: the format has them, the meter's tree uses none.
    #[allow(dead_code)]
    S8 = 6,
    #[allow(dead_code)]
    S16 = 7,
    #[allow(dead_code)]
    S32 = 8,
    Str = 9,
    Bin = 10,
    Float = 11,
}

impl Kind {
    fn has_id(self) -> bool {
        !matches!(self, Kind::Plain | Kind::Link)
    }

    /// The bytes of a fixed size value; `None` for a string or binary.
    fn size(self) -> Option<usize> {
        match self {
            Kind::Plain | Kind::Link => Some(0),
            Kind::Chooser | Kind::U8 | Kind::S8 => Some(1),
            Kind::U16 | Kind::S16 => Some(2),
            Kind::U32 | Kind::S32 | Kind::Float => Some(4),
            Kind::Str | Kind::Bin => None,
        }
    }
}

/// A node of the tree.
#[derive(Clone, Debug)]
struct Node {
    name: &'static str,
    kind: Kind,
    children: Vec<Node>,
}

fn n(name: &'static str, kind: Kind, children: Vec<Node>) -> Node {
    Node { name, kind, children }
}

fn leaf(name: &'static str, kind: Kind) -> Node {
    n(name, kind, Vec::new())
}

fn plains(names: &[&'static str]) -> Vec<Node> {
    names.iter().map(|s| leaf(s, Kind::Plain)).collect()
}

/// The tree of `protocol.c:172-264`, with or without `REAL_PWR`.
fn tree(power: bool) -> Node {
    use Kind::{Bin, Chooser, Float, Link, Plain, Str, U8, U16, U32};
    let channel = |name: &'static str, primary: Node| {
        n(
            name,
            Plain,
            vec![
                n("MAPPING", Chooser, vec![primary, n("TEMP", Plain, plains(&["350"])), leaf("SHARED", Link)]),
                leaf("RANGE_I", U8),
                n("ANALYSIS", Chooser, plains(&["MEAN", "RMS", "BUFFER"])),
                leaf("VALUE", Float),
                leaf("OFFSET", Float),
                leaf("BUF", Bin),
                leaf("BUF_BPS", U8),
                leaf("BUF_LSB2NATIVE", Float),
            ],
        )
    };
    let mut top = vec![
        n("ADMIN", Plain, vec![leaf("CRC32", U32), leaf("TREE", Bin), leaf("DIAGNOSTIC", Str)]),
        leaf("PCB_VERSION", U8),
        leaf("NAME", Str),
        leaf("TIME_UTC", U32),
        leaf("TIME_UTC_MS", U16),
        leaf("BAT_V", Float),
        n("REBOOT", Chooser, plains(&["NORMAL", "SHIPMODE"])),
        n(
            "SAMPLING",
            Plain,
            vec![
                n("RATE", Chooser, plains(&["125", "250", "500", "1000", "2000", "4000", "8000"])),
                n("DEPTH", Chooser, plains(&["32", "64", "128", "256"])),
                n("TRIGGER", Chooser, plains(&["OFF", "SINGLE", "CONTINUOUS"])),
            ],
        ),
        n(
            "LOG",
            Plain,
            vec![
                leaf("ON", U8),
                leaf("INTERVAL", U16),
                leaf("STATUS", U8),
                leaf("POLLDIR", U8),
                n("INFO", Plain, vec![leaf("INDEX", U16), leaf("END_TIME", U32), leaf("N_BYTES", U32)]),
                n("STREAM", Plain, vec![leaf("INDEX", U16), leaf("OFFSET", U32), leaf("DATA", Bin)]),
            ],
        ),
        channel("CH1", n("CURRENT", Plain, plains(&["10"]))),
        channel("CH2", n("VOLTAGE", Plain, plains(&["60", "600"]))),
        n(
            "SHARED",
            Chooser,
            vec![
                n("AUX_V", Plain, plains(&["0.1", "0.3", "1.2"])),
                n(
                    "RESISTANCE",
                    Plain,
                    plains(&["1000.0", "10000.0", "100000.0", "1000000.0", "10000000.0"]),
                ),
                n("DIODE", Plain, plains(&["1.2"])),
            ],
        ),
    ];
    if power {
        top.push(leaf("REAL_PWR", Float));
    }
    n("", Plain, top)
}

/// A node with an id, by its path.
#[derive(Clone, Debug)]
struct Slot {
    path: String,
    kind: Kind,
    /// The names of its children, for a chooser.
    choices: Vec<String>,
}

/// The tree's serialization (`ConfigNode.py`'s `serialize`) and its nodes with
/// an id, in id order.
fn flatten(node: &Node, path: &str, bytes: &mut Vec<u8>, slots: &mut Vec<Slot>) {
    bytes.push(node.kind as u8);
    bytes.push(node.name.len() as u8);
    bytes.extend_from_slice(node.name.as_bytes());
    bytes.push(node.children.len() as u8);
    if node.kind.has_id() {
        slots.push(Slot {
            path: path.to_owned(),
            kind: node.kind,
            choices: node.children.iter().map(|c| c.name.to_owned()).collect(),
        });
    }
    for c in &node.children {
        let p = if path.is_empty() { c.name.to_owned() } else { format!("{path}:{}", c.name) };
        flatten(c, &p, bytes, slots);
    }
}

/// CRC-32 (IEEE), as zlib's `crc32` and `protocol.c:1138-1153`.
pub fn crc32(data: &[u8]) -> u32 {
    let mut r = 0xffff_ffffu32;
    for b in data {
        r ^= u32::from(*b);
        for _ in 0..8 {
            r = if r & 1 != 0 { (r >> 1) ^ 0xedb8_8320 } else { r >> 1 };
        }
    }
    !r
}

/// A value the meter holds.
#[derive(Clone, Debug, PartialEq)]
enum Val {
    Int(i64),
    Float(f32),
    Bytes(Vec<u8>),
}

/// The fault and reading options.
#[derive(Clone, Debug, Default)]
#[allow(clippy::struct_excessive_bools)]
struct Options {
    addr: [u8; 6],
    seq: u8,
    ch1: Vec<f32>,
    ch2: Vec<f32>,
    pwr: Vec<f32>,
    period: Option<Duration>,
    battery: u32,
    diag: Option<String>,
    lose: u32,
    swap: u32,
    dup: u32,
    mute_at: Option<u32>,
    stop_at: Option<u32>,
    mark_at: u32,
    crc_bad: bool,
    garbage: bool,
    no_power: bool,
    idle: Option<Duration>,
    absent: bool,
    refuse: bool,
}

/// The meter.
#[derive(Debug)]
pub struct Meter {
    o: Options,
    slots: Vec<Slot>,
    values: Vec<Val>,
    tree_crc: u32,
    unlocked: bool,
    /// The next sequence number out; it runs on across connections.
    seq: u8,
    /// What came of the host's stream and is not a whole message yet.
    inbox: Vec<u8>,
    /// Notifications since power on.
    sent: u32,
    /// A notification held back by `swap`.
    held: Option<Vec<u8>>,
    writes: u32,
    readings: u32,
    next_reading: Option<Instant>,
    heard: Instant,
}

fn parse_floats(key: &str, value: &str) -> Result<Vec<f32>> {
    value
        .split(',')
        .map(|v| v.parse::<f32>().map_err(|_| Error::Invalid(format!("{key}={value}: numbers"))))
        .collect()
}

fn parse_addr(value: &str) -> Result<[u8; 6]> {
    let parts: Vec<&str> = value.split(':').collect();
    let bad = || Error::Invalid(format!("addr={value}: six hex bytes, colon separated"));
    if parts.len() != 6 {
        return Err(bad());
    }
    let mut a = [0u8; 6];
    for (i, p) in parts.iter().enumerate() {
        a[i] = u8::from_str_radix(p, 16).map_err(|_| bad())?;
    }
    Ok(a)
}

fn parse_options(options: &[(String, String)]) -> Result<Options> {
    let mut o = Options { addr: [0xd0, 0x5f, 0xb8, 0x4d, 0x00, 0x01], idle: Some(Duration::from_secs(20)), ..Options::default() };
    let num = |k: &str, v: &str| v.parse::<u32>().map_err(|_| Error::Invalid(format!("{k}={v}: a number")));
    for (k, v) in options {
        match k.as_str() {
            "addr" => o.addr = parse_addr(v)?,
            "seq" => {
                o.seq = v.parse().map_err(|_| Error::Invalid(format!("seq={v}: 0 to 255")))?;
            }
            "ch1" => o.ch1 = parse_floats(k, v)?,
            "ch2" => o.ch2 = parse_floats(k, v)?,
            "pwr" => o.pwr = parse_floats(k, v)?,
            "period" => o.period = Some(Duration::from_millis(u64::from(num(k, v)?.max(1)))),
            "battery" => o.battery = num(k, v)?,
            "diag" => o.diag = Some(v.replace('_', " ")),
            "lose" => o.lose = num(k, v)?,
            "swap" => o.swap = num(k, v)?,
            "dup" => o.dup = num(k, v)?,
            "mute-at" => o.mute_at = Some(num(k, v)?),
            "stop-at" => o.stop_at = Some(num(k, v)?),
            "mark-at" => o.mark_at = num(k, v)?,
            "crc" if v == "bad" => o.crc_bad = true,
            "tree" if v == "garbage" => o.garbage = true,
            "tree" if v == "no-pwr" => o.no_power = true,
            "idle" => {
                let s = num(k, v)?;
                o.idle = (s > 0).then(|| Duration::from_secs(u64::from(s)));
            }
            "absent" if v == "1" => o.absent = true,
            "refuse" if v == "1" => o.refuse = true,
            _ => return Err(Error::Invalid(format!("{NAME}: no option {k}={v}"))),
        }
    }
    Ok(o)
}

impl Meter {
    fn new(o: Options) -> Meter {
        let mut bytes = Vec::new();
        let mut slots = Vec::new();
        flatten(&tree(!o.no_power), "", &mut bytes, &mut slots);
        let compressed = if o.garbage {
            b"not a zlib stream at all".to_vec()
        } else {
            let mut z = ZlibEncoder::new(Vec::new(), Compression::default());
            z.write_all(&bytes).expect("in memory");
            z.finish().expect("in memory")
        };
        let tree_crc = crc32(&compressed);
        let mut m = Meter {
            seq: o.seq,
            values: slots.iter().map(|s| match s.kind {
                Kind::Float => Val::Float(0.0),
                Kind::Str | Kind::Bin => Val::Bytes(Vec::new()),
                _ => Val::Int(0),
            })
            .collect(),
            slots,
            tree_crc,
            unlocked: false,
            inbox: Vec::new(),
            sent: 0,
            held: None,
            writes: 0,
            readings: 0,
            next_reading: None,
            heard: Instant::now(),
            o,
        };
        m.set("ADMIN:CRC32", Val::Int(i64::from(tree_crc)));
        m.set("ADMIN:TREE", Val::Bytes(compressed));
        m.set("PCB_VERSION", Val::Int(8));
        m.set("NAME", Val::Bytes(b"Mooshimeter V.1".to_vec()));
        m.set("BAT_V", Val::Float(2.95));
        // A meter left at 256 samples, so that an open's 64 shows.
        m.set("SAMPLING:DEPTH", Val::Int(3));
        m.set("CH1:BUF_BPS", Val::Int(24));
        m.set("CH2:BUF_BPS", Val::Int(24));
        m.update_lsb();
        m
    }

    fn id(&self, path: &str) -> Option<usize> {
        self.slots.iter().position(|s| s.path == path)
    }

    fn set(&mut self, path: &str, v: Val) {
        if let Some(i) = self.id(path) {
            self.values[i] = v;
        }
    }

    fn int(&self, path: &str) -> i64 {
        match self.id(path).map(|i| &self.values[i]) {
            Some(Val::Int(v)) => *v,
            _ => 0,
        }
    }

    fn choice(&self, path: &str) -> String {
        let i = self.int(path);
        self.id(path)
            .and_then(|s| self.slots[s].choices.get(usize::try_from(i).ok()?).cloned())
            .unwrap_or_default()
    }

    /// The input a channel reads now: CURRENT, VOLTAGE, TEMP, `AUX_V`,
    /// RESISTANCE or DIODE.
    fn input(&self, ch: u8) -> String {
        let mapped = self.choice(&format!("CH{ch}:MAPPING"));
        if mapped == "SHARED" { self.choice("SHARED") } else { mapped }
    }

    /// The ranges of a channel's input, as the tree names them.
    fn ranges(&self, ch: u8) -> Vec<String> {
        let input = self.input(ch);
        let names: &[&str] = match input.as_str() {
            "CURRENT" => &["10"],
            "VOLTAGE" => &["60", "600"],
            "TEMP" => &["350"],
            "AUX_V" => &["0.1", "0.3", "1.2"],
            "RESISTANCE" => &["1000.0", "10000.0", "100000.0", "1000000.0", "10000000.0"],
            _ => &["1.2"],
        };
        names.iter().map(|s| (*s).to_owned()).collect()
    }

    fn range(&self, ch: u8) -> f32 {
        let r = self.ranges(ch);
        let i = usize::try_from(self.int(&format!("CH{ch}:RANGE_I"))).unwrap_or(0);
        r.get(i).or(r.first()).and_then(|s| s.parse().ok()).unwrap_or(1.0)
    }

    fn bps(&self, ch: u8) -> u32 {
        u32::try_from(self.int(&format!("CH{ch}:BUF_BPS"))).unwrap_or(24).clamp(1, 32)
    }

    fn update_lsb(&mut self) {
        for ch in [1, 2] {
            let full = (1u64 << (self.bps(ch) - 1)) as f32;
            let lsb = self.range(ch) / full;
            self.set(&format!("CH{ch}:BUF_LSB2NATIVE"), Val::Float(lsb));
        }
    }

    /// The bytes of value `id` on the wire.
    fn encode(&self, id: usize) -> Vec<u8> {
        let kind = self.slots[id].kind;
        let mut out = vec![id as u8];
        match &self.values[id] {
            Val::Int(v) => {
                let size = kind.size().unwrap_or(0);
                out.extend_from_slice(&v.to_le_bytes()[..size]);
            }
            Val::Float(f) => out.extend_from_slice(&f.to_le_bytes()),
            Val::Bytes(b) => {
                out.extend_from_slice(&(b.len() as u16).to_le_bytes());
                out.extend_from_slice(b);
            }
        }
        out
    }

    /// Sends a stream of messages: as many notifications as it takes.
    fn emit(&mut self, stream: &[u8], port: &mut Port) {
        for piece in stream.chunks(CHUNK) {
            let mut p = vec![self.seq];
            self.seq = self.seq.wrapping_add(1);
            p.extend_from_slice(piece);
            self.sent += 1;
            let nth = self.sent;
            if nth == self.o.lose {
                continue;
            }
            if nth == self.o.swap {
                self.held = Some(p);
                continue;
            }
            port.notify(SEROUT, &p);
            if nth == self.o.dup {
                port.notify(SEROUT, &p);
            }
            if let Some(h) = self.held.take() {
                port.notify(SEROUT, &h);
            }
        }
    }

    /// The value of reading `k` of a channel's input.
    fn reading(&self, ch: u8, k: u32) -> f32 {
        let given = if ch == 1 { &self.o.ch1 } else { &self.o.ch2 };
        let list: &[f32] = if given.is_empty() {
            match self.input(ch).as_str() {
                "CURRENT" => &[0.25, -0.125, 1.5],
                "VOLTAGE" => &[12.5, 12.25, 12.75],
                "TEMP" => &[296.15, 296.65],
                "AUX_V" => &[0.05, -0.0625],
                "RESISTANCE" => &[4700.0, 4725.0],
                _ => &[0.625, 0.6],
            }
        } else {
            given
        };
        list[k as usize % list.len()]
    }

    /// A buffer of DEPTH samples of `value`, every other one a step up.
    fn buffer(&self, ch: u8, value: f32) -> Vec<u8> {
        let depth: usize = self.choice("SAMPLING:DEPTH").parse().unwrap_or(64);
        let bps = self.bps(ch);
        let bytes = bps.div_ceil(8) as usize;
        let full = (1i64 << (bps - 1)) - 1;
        let lsb = self.range(ch) / (1u64 << (bps - 1)) as f32;
        let raw = (f64::from(value) / f64::from(lsb)).round() as i64;
        let mut out = Vec::with_capacity(depth * bytes);
        for i in 0..depth {
            let v = (raw + (i % 2) as i64).clamp(-full - 1, full);
            out.extend_from_slice(&v.to_le_bytes()[..bytes]);
        }
        out
    }

    /// One buffer's messages, as the meter sends them.
    fn take_reading(&mut self, port: &mut Port) {
        let k = self.readings;
        self.readings += 1;
        let (v1, v2) = (self.reading(1, k), self.reading(2, k));
        let p = if self.o.pwr.is_empty() { v1 * v2 } else { self.o.pwr[k as usize % self.o.pwr.len()] };
        let mut stream = Vec::new();
        for (ch, v) in [(1u8, v1), (2, v2)] {
            if self.choice(&format!("CH{ch}:ANALYSIS")) == "BUFFER" {
                let b = self.buffer(ch, v);
                self.set(&format!("CH{ch}:BUF"), Val::Bytes(b));
                let id = self.id(&format!("CH{ch}:BUF")).expect("in the tree");
                stream.extend(self.encode(id));
            } else {
                self.set(&format!("CH{ch}:VALUE"), Val::Float(v));
                let id = self.id(&format!("CH{ch}:VALUE")).expect("in the tree");
                stream.extend(self.encode(id));
            }
        }
        if let Some(id) = self.id("REAL_PWR") {
            self.values[id] = Val::Float(p);
            stream.extend(self.encode(id));
        }
        if self.o.battery > 0 && self.readings.is_multiple_of(self.o.battery) {
            stream.extend(self.encode(self.id("BAT_V").expect("in the tree")));
        }
        if self.readings == 1
            && let Some(text) = self.o.diag.clone()
        {
            self.set("ADMIN:DIAGNOSTIC", Val::Bytes(text.into_bytes()));
            stream.extend(self.encode(2));
        }
        self.emit(&stream, port);
        if self.readings == self.o.mark_at {
            port.mark();
        }
    }

    fn period(&self) -> Duration {
        self.o.period.unwrap_or_else(|| {
            let rate: u64 = self.choice("SAMPLING:RATE").parse().unwrap_or(125);
            let depth: u64 = self.choice("SAMPLING:DEPTH").parse().unwrap_or(64);
            Duration::from_micros(depth * 1_000_000 / rate.max(1))
        })
    }

    /// A write to node `id` with `value`; what the meter answers.
    fn write(&mut self, id: usize, value: Val, port: &mut Port) -> Vec<u8> {
        let slot = self.slots[id].clone();
        let path = slot.path.as_str();
        match (path, &value) {
            ("ADMIN:CRC32", Val::Int(v)) => {
                let v = *v as u32;
                if self.o.crc_bad {
                    let mut out = vec![0];
                    out.extend_from_slice(&(v ^ 0x5a5a_5a5a).to_le_bytes());
                    return out;
                }
                if v == self.tree_crc {
                    self.unlocked = true;
                } else {
                    port.violation(format!("ADMIN:CRC32 written {v:08x}, the tree's is {:08x}", self.tree_crc));
                }
                let mut out = vec![0];
                out.extend_from_slice(&v.to_le_bytes());
                return out;
            }
            ("ADMIN:TREE" | "CH1:BUF" | "CH2:BUF" | "CH1:VALUE" | "CH2:VALUE" | "REAL_PWR" | "BAT_V"
            | "CH1:BUF_LSB2NATIVE" | "CH2:BUF_LSB2NATIVE" | "LOG:STREAM:DATA", _) => {
                port.violation(format!("{path} is the meter's to say"));
                return self.encode(id);
            }
            _ => {}
        }
        if let (Kind::Chooser, Val::Int(v)) = (slot.kind, &value)
            && usize::try_from(*v).map_or(true, |i| i >= slot.choices.len())
        {
            port.violation(format!("{path} has no choice {v}"));
            return self.encode(id);
        }
        if let ("CH1:RANGE_I" | "CH2:RANGE_I", Val::Int(v)) = (path, &value) {
            let ch = if path.starts_with("CH1") { 1 } else { 2 };
            if usize::try_from(*v).map_or(true, |i| i >= self.ranges(ch).len()) {
                port.violation(format!("{path} {v}: {} has no such range", self.input(ch)));
                return self.encode(id);
            }
        }
        self.values[id] = value;
        if path == "SAMPLING:TRIGGER" {
            self.next_reading = match self.choice(path).as_str() {
                "OFF" => None,
                _ => Some(port.now() + self.period()),
            };
        }
        self.update_lsb();
        self.encode(id)
    }

    /// The host's stream: whole messages are answered, a part waits.
    fn serve(&mut self, port: &mut Port) {
        let mut answer = Vec::new();
        while let Some(&head) = self.inbox.first() {
            let id = usize::from(head & 0x7f);
            let write = head & 0x80 != 0;
            let Some(slot) = self.slots.get(id).cloned() else {
                port.violation(format!("no node has id {id}"));
                self.inbox.clear();
                break;
            };
            if !self.unlocked && id > 2 {
                port.violation(format!("{} before the CRC handshake", slot.path));
                self.inbox.clear();
                break;
            }
            if !write {
                self.inbox.remove(0);
                answer.extend(self.encode(id));
                continue;
            }
            let value = if let Some(size) = slot.kind.size() {
                if self.inbox.len() < 1 + size {
                    break;
                }
                let b = &self.inbox[1..=size];
                let mut w = [0u8; 8];
                w[..size].copy_from_slice(b);
                let v = match slot.kind {
                    Kind::Float => Val::Float(f32::from_le_bytes([b[0], b[1], b[2], b[3]])),
                    Kind::S8 => Val::Int(i64::from(b[0] as i8)),
                    Kind::S16 => Val::Int(i64::from(i16::from_le_bytes([b[0], b[1]]))),
                    Kind::S32 => Val::Int(i64::from(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))),
                    _ => Val::Int(i64::from_le_bytes(w)),
                };
                self.inbox.drain(..=size);
                v
            } else {
                if self.inbox.len() < 3 {
                    break;
                }
                let len = usize::from(u16::from_le_bytes([self.inbox[1], self.inbox[2]]));
                if self.inbox.len() < 3 + len {
                    break;
                }
                let v = Val::Bytes(self.inbox[3..3 + len].to_vec());
                self.inbox.drain(..3 + len);
                v
            };
            let a = self.write(id, value, port);
            answer.extend(a);
        }
        if !answer.is_empty() {
            self.emit(&answer, port);
        }
    }
}

impl Peripheral for Meter {
    fn address(&self) -> [u8; 6] {
        self.o.addr
    }

    fn advertising(&self) -> Vec<u8> {
        let mut a = vec![2, 0x01, 0x06, 17, 0x07];
        a.extend_from_slice(&Uuid::long("1bc5ffa0-0200-62ab-e411-f254e005dbd4").wire());
        a
    }

    fn attributes(&self) -> Vec<Attribute> {
        let base = |n: &str| Uuid::long(&format!("1bc5{n}-0200-62ab-e411-f254e005dbd4"));
        let read = Access { read: true, ..Access::default() };
        let write = Access { write: true, ..Access::default() };
        vec![
            Attribute::service(0x0001, &Uuid::Short(0x1800)),
            Attribute::characteristic(0x0002, 0x02, 0x0003, &Uuid::Short(0x2a00)),
            Attribute::value(0x0003, Uuid::Short(0x2a00), b"Mooshimeter V.1", read),
            Attribute::service(0x0010, &base("ffa0")),
            Attribute::characteristic(0x0011, 0x08, SERIN, &base("ffa1")),
            Attribute::value(SERIN, base("ffa1"), &[], write),
            Attribute::value(0x0013, Uuid::Short(USER_DESCRIPTION), b"SERIN", read),
            Attribute::characteristic(0x0014, 0x10, SEROUT, &base("ffa2")),
            Attribute::value(SEROUT, base("ffa2"), &[], Access::default()),
            Attribute::cccd(0x0016),
            Attribute::value(0x0017, Uuid::Short(USER_DESCRIPTION), b"SEROUT", read),
        ]
    }

    fn present(&self) -> bool {
        !self.o.absent
    }

    fn connect_status(&self) -> u8 {
        if self.o.refuse { 0x3e } else { 0 }
    }

    fn written(&mut self, handle: u16, value: &[u8], port: &mut Port) {
        if handle != SERIN {
            port.violation(format!("a write to {handle:04x}"));
            return;
        }
        self.heard = port.now();
        self.writes += 1;
        if self.o.mute_at.is_some_and(|n| self.writes > n) {
            return;
        }
        let Some((_seq, data)) = value.split_first() else {
            port.violation("an empty SERIN write");
            return;
        };
        if data.len() > CHUNK {
            port.violation(format!("a SERIN write of {} bytes", value.len()));
        }
        if !port.subscribed(SEROUT) {
            port.violation("SERIN written with SEROUT's notifications off");
        }
        self.inbox.extend_from_slice(data);
        self.serve(port);
    }

    fn connected(&mut self, port: &mut Port) {
        self.heard = port.now();
        self.inbox.clear();
        self.unlocked = false;
    }

    fn disconnected(&mut self) {
        self.next_reading = None;
        self.set("SAMPLING:TRIGGER", Val::Int(0));
        self.unlocked = false;
        self.held = None;
    }

    fn tick(&mut self, port: &mut Port) {
        let now = port.now();
        if self.o.idle.is_some_and(|idle| now.duration_since(self.heard) > idle) {
            port.hang_up();
            return;
        }
        if self.o.mute_at.is_some_and(|n| self.writes > n) {
            return;
        }
        let Some(due) = self.next_reading else { return };
        if now < due || self.o.stop_at.is_some_and(|n| self.readings >= n) {
            return;
        }
        self.take_reading(port);
        let next = due + self.period();
        self.next_reading = Some(if next < now { now + self.period() } else { next });
        if self.choice("SAMPLING:TRIGGER") == "SINGLE" {
            self.next_reading = None;
            self.set("SAMPLING:TRIGGER", Val::Int(0));
        }
    }
}

/// The USB side: none. The meter is on the BLE carrier alone.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    parse_options(options)?;
    Ok(Box::new(NoUsb::new(NAME)))
}

/// The meter, on the BLE carrier.
pub fn build_ble(options: &[(String, String)]) -> Result<Box<dyn Peripheral>> {
    Ok(Box::new(Meter::new(parse_options(options)?)))
}

#[cfg(test)]
mod tests {
    use std::io::Read as _;

    use flate2::read::ZlibDecoder;

    use super::*;

    fn meter(options: &[(&str, &str)]) -> Meter {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Meter::new(parse_options(&o).unwrap())
    }

    /// Writes `stream` in one SERIN write and gives the notifications'
    /// payloads, their sequence numbers taken off.
    fn write(m: &mut Meter, stream: &[u8], now: Instant) -> (Vec<u8>, Vec<String>) {
        let mut port = Port::new(now, &[SEROUT]);
        let mut v = vec![0];
        v.extend_from_slice(stream);
        m.written(SERIN, &v, &mut port);
        let mut out = Vec::new();
        for (h, p) in port.notified() {
            assert_eq!(*h, SEROUT);
            out.extend_from_slice(&p[1..]);
        }
        (out, port.violations())
    }

    fn unlock(m: &mut Meter) {
        let now = Instant::now();
        let (tree, v) = write(m, &[0x01], now);
        assert_eq!(v, Vec::<String>::new());
        let len = usize::from(u16::from_le_bytes([tree[1], tree[2]]));
        let crc = crc32(&tree[3..3 + len]);
        let mut w = vec![0x80];
        w.extend_from_slice(&crc.to_le_bytes());
        let (echo, v) = write(m, &w, now);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(echo[1..], crc.to_le_bytes());
    }

    #[test]
    fn the_tree_gives_libsigrok_listing_ids() {
        let m = meter(&[]);
        let id = |p: &str| m.id(p).unwrap();
        assert_eq!(id("ADMIN:CRC32"), 0);
        assert_eq!(id("ADMIN:DIAGNOSTIC"), 2);
        assert_eq!(id("BAT_V"), 7);
        assert_eq!(id("SAMPLING:TRIGGER"), 11);
        assert_eq!(id("LOG:STREAM:DATA"), 21);
        assert_eq!(id("CH1:MAPPING"), 22);
        assert_eq!(id("CH1:BUF_LSB2NATIVE"), 29);
        assert_eq!(id("CH2:VALUE"), 33);
        assert_eq!(id("SHARED"), 38);
        assert_eq!(id("REAL_PWR"), 39);
        assert_eq!(m.slots.len(), 40);
    }

    #[test]
    fn the_tree_unpacks_to_its_serialization_and_its_crc_unlocks() {
        let mut m = meter(&[]);
        let (tree, _) = write(&mut m, &[0x01], Instant::now());
        assert_eq!(tree[0], 0x01);
        let len = usize::from(u16::from_le_bytes([tree[1], tree[2]]));
        assert_eq!(tree.len(), 3 + len);
        let mut plain = Vec::new();
        ZlibDecoder::new(&tree[3..]).read_to_end(&mut plain).unwrap();
        // The unnamed plain root, its 13 children, then ADMIN.
        assert_eq!(plain[..4], [0, 0, 13, 0]);
        assert_eq!(&plain[4..10], b"\x05ADMIN");
        unlock(&mut m);
        assert!(m.unlocked);
    }

    #[test]
    fn notifications_carry_19_bytes_and_a_running_sequence_number() {
        let mut m = meter(&[("seq", "254")]);
        let mut port = Port::new(Instant::now(), &[SEROUT]);
        m.written(SERIN, &[7, 0x01], &mut port);
        let seqs: Vec<u8> = port.notified().iter().map(|(_, p)| p[0]).collect();
        assert_eq!(seqs[..3], [254, 255, 0]);
        assert!(port.notified().iter().all(|(_, p)| p.len() <= 20));
        assert!(port.notified()[..port.notified().len() - 1].iter().all(|(_, p)| p.len() == 20));
    }

    #[test]
    fn the_rest_of_the_tree_waits_for_the_handshake() {
        let mut m = meter(&[]);
        let (out, v) = write(&mut m, &[0x8b, 2], Instant::now());
        assert_eq!(out, Vec::<u8>::new());
        assert_eq!(v, ["SAMPLING:TRIGGER before the CRC handshake"]);
    }

    #[test]
    fn a_write_is_echoed_and_a_bad_one_flagged() {
        let mut m = meter(&[]);
        unlock(&mut m);
        let now = Instant::now();
        assert_eq!(write(&mut m, &[0x8a, 1], now), (vec![0x0a, 1], vec![]));
        let (out, v) = write(&mut m, &[0x8a, 9], now);
        assert_eq!(out, [0x0a, 1], "the old value");
        assert_eq!(v, ["SAMPLING:DEPTH has no choice 9"]);
        // CH2 maps VOLTAGE: two ranges.
        let (out, v) = write(&mut m, &[0x9f, 2], now);
        assert_eq!(out, [0x1f, 0]);
        assert_eq!(v.len(), 1);
        assert_eq!(write(&mut m, &[0x9f, 1], now).0, [0x1f, 1]);
        // A read.
        assert_eq!(write(&mut m, &[0x03], now).0, [0x03, 8]);
    }

    #[test]
    fn a_message_split_over_two_writes_is_answered_once_whole() {
        let mut m = meter(&[]);
        unlock(&mut m);
        let now = Instant::now();
        assert_eq!(write(&mut m, &[0x8d], now).0, Vec::<u8>::new());
        assert_eq!(write(&mut m, &[0x10, 0x00], now).0, [0x0d, 0x10, 0x00]);
    }

    #[test]
    fn continuous_sampling_sends_a_reading_each_period() {
        let mut m = meter(&[]);
        unlock(&mut m);
        let t0 = Instant::now();
        write(&mut m, &[0x8a, 1], t0); // 64 samples at 125 Hz
        write(&mut m, &[0x8b, 2], t0);
        let mut tick = |at: Duration| {
            let mut port = Port::new(t0 + at, &[SEROUT]);
            m.tick(&mut port);
            port.notified().iter().flat_map(|(_, p)| p[1..].to_vec()).collect::<Vec<u8>>()
        };
        assert_eq!(tick(Duration::from_millis(500)), Vec::<u8>::new());
        let r = tick(Duration::from_millis(520));
        assert_eq!(r.len(), 15);
        assert_eq!(r[0], 25);
        assert_eq!(f32::from_le_bytes([r[1], r[2], r[3], r[4]]), 0.25);
        assert_eq!(r[5], 33);
        assert_eq!(f32::from_le_bytes([r[6], r[7], r[8], r[9]]), 12.5);
        assert_eq!(r[10], 39);
        assert_eq!(f32::from_le_bytes([r[11], r[12], r[13], r[14]]), 3.125);
        assert_eq!(tick(Duration::from_millis(1000)), Vec::<u8>::new());
        assert_eq!(tick(Duration::from_millis(1030))[0], 25);
    }

    #[test]
    fn a_buffer_holds_depth_samples_of_bps_bits() {
        let mut m = meter(&[("period", "1")]);
        unlock(&mut m);
        let t0 = Instant::now();
        write(&mut m, &[0x8a, 0], t0); // 32 samples
        write(&mut m, &[0x98, 2], t0); // CH1 BUFFER
        write(&mut m, &[0x8b, 2], t0);
        let mut port = Port::new(t0 + Duration::from_millis(5), &[SEROUT]);
        m.tick(&mut port);
        let r: Vec<u8> = port.notified().iter().flat_map(|(_, p)| p[1..].to_vec()).collect();
        assert_eq!(r[0], 27);
        assert_eq!(u16::from_le_bytes([r[1], r[2]]), 96);
        let lsb = 10.0 / 8_388_608.0f32;
        let first = i32::from_le_bytes([r[3], r[4], r[5], 0]);
        assert!((first as f32 * lsb - 0.25).abs() < 1e-5);
        assert_eq!(r[3 + 96], 33, "then CH2's value");
    }

    #[test]
    fn faults_on_notifications_keep_the_sequence() {
        let seqs = |opt: (&str, &str)| {
            let mut m = meter(&[opt]);
            let mut port = Port::new(Instant::now(), &[SEROUT]);
            m.written(SERIN, &[0, 0x01], &mut port);
            port.notified().iter().map(|(_, p)| p[0]).take(5).collect::<Vec<u8>>()
        };
        assert_eq!(seqs(("lose", "2")), [0, 2, 3, 4, 5]);
        assert_eq!(seqs(("swap", "2")), [0, 2, 1, 3, 4]);
        assert_eq!(seqs(("dup", "2")), [0, 1, 1, 2, 3]);
    }

    #[test]
    fn a_silent_host_is_dropped_after_its_idle_time() {
        let mut m = meter(&[("idle", "20")]);
        let t0 = Instant::now();
        let mut port = Port::new(t0, &[SEROUT]);
        m.connected(&mut port);
        let mut port = Port::new(t0 + Duration::from_secs(19), &[SEROUT]);
        m.tick(&mut port);
        assert!(!port.hung_up());
        let mut port = Port::new(t0 + Duration::from_secs(21), &[SEROUT]);
        m.tick(&mut port);
        assert!(port.hung_up());
    }

    #[test]
    fn options_are_checked() {
        let bad = |k: &str, v: &str| {
            parse_options(&[(k.to_owned(), v.to_owned())]).unwrap_err().to_string()
        };
        assert!(bad("addr", "1:2:3").contains("six hex bytes"));
        assert!(bad("ch1", "x").contains("numbers"));
        assert!(bad("tree", "pine").contains("no option"));
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }
}
