// Counterpart in sigrok:
//   libsigrok/src/hardware/mooshimeter-dmm/protocol.c,
//   libsigrok/src/hardware/mooshimeter-dmm/protocol.h,
//   Copyright (C) 2019 Derek Hageman <hageman@inthat.cloud>,
//   with commits by Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Mooshimeter's layers above BLE, from `protocol.c:26-265`:
//!
//! - **The serial stream.** Each notification and each write is a sequence
//!   number and 1 to 19 bytes of a byte stream. The meter's numbers don't
//!   restart at a connection, so the first one sets where the count is; one
//!   ahead of it waits for the gap to fill ([`Rx`]).
//! - **The config tree.** A node has a type, a name and children; each node of a
//!   type with a value has a 7 bit id, given depth first. A read is `ID`, a write
//!   `0x80|ID VALUE`, and the meter answers either with `ID VALUE`, the value it
//!   now holds; it also sends values unasked (readings, the battery). Values are
//!   little endian: u8, u16, u32, their signed kin, an f32, and a string or a
//!   blob behind a u16 length ([`Tree`]).
//! - **The handshake.** At first only ADMIN:CRC32 (0), ADMIN:TREE (1) and
//!   ADMIN:DIAGNOSTIC (2) exist. The host reads ADMIN:TREE, a zlib stream of the
//!   tree's serialization (per node: type, name length, name, child count, then
//!   the children), and writes its CRC-32 to ADMIN:CRC32; the echo of the CRC
//!   opens the rest.
//!
//! Where it goes its own way:
//!
//! - A notification ahead of its turn waits only so long: once [`HELD_MAX`]
//!   more have come with the gap still open, it was lost, and the stream can't be
//!   read past it ([`Lost`]). C keeps them for ever, and from then on nothing
//!   that comes is read. C's reorder list also shifts against the numbers when a
//!   gap is two long, and appends what it holds out of order then. A number
//!   behind the count is a repeat and is dropped; C files it 255 ahead.
//! - An id no node has ends the reading of the stream, as the bytes after it
//!   can't be framed. C drops the one byte and reads on, so what follows is
//!   framed wherever a byte happens to fall.

use std::collections::BTreeMap;
use std::fmt;

/// The bytes of the stream one notification or write carries.
pub const CHUNK: usize = 19;
/// Notifications held ahead of a gap before it counts as a loss.
pub const HELD_MAX: usize = 8;
/// The most ids a tree has: 7 bits, and C's `tree_id_lookup[0x7F]`.
pub const MAX_IDS: usize = 0x7f;

/// A node's type (`protocol.c:93-106`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// No value: information, or a choice of a chooser.
    Plain,
    /// No value: a link elsewhere in the tree.
    Link,
    /// A u8 index among its children.
    Chooser,
    /// u8.
    U8,
    /// u16.
    U16,
    /// u32.
    U32,
    /// i8.
    S8,
    /// i16.
    S16,
    /// i32.
    S32,
    /// A u16 length and text.
    Str,
    /// A u16 length and bytes.
    Bin,
    /// f32.
    Float,
}

impl Kind {
    fn from_code(code: u8) -> Option<Kind> {
        Some(match code {
            0 => Kind::Plain,
            1 => Kind::Link,
            2 => Kind::Chooser,
            3 => Kind::U8,
            4 => Kind::U16,
            5 => Kind::U32,
            6 => Kind::S8,
            7 => Kind::S16,
            8 => Kind::S32,
            9 => Kind::Str,
            10 => Kind::Bin,
            11 => Kind::Float,
            _ => return None,
        })
    }

    /// Whether a node of this type has an id (`tree_node_has_id`).
    pub fn has_id(self) -> bool {
        !matches!(self, Kind::Plain | Kind::Link)
    }
}

/// A value the tree holds.
#[derive(Clone, Debug, PartialEq)]
pub enum Val {
    /// None yet, or none at all.
    None,
    /// An integer, a chooser's index included, as C's `int32_t`.
    Int(i32),
    /// A float.
    Float(f32),
    /// A string or a blob.
    Bytes(Vec<u8>),
}

/// One node.
#[derive(Clone, Debug)]
pub struct Node {
    /// Its name.
    pub name: String,
    /// Its type.
    pub kind: Kind,
    /// Its id, for a type with a value.
    pub id: Option<u8>,
    /// Its children, by index in [`Tree::nodes`].
    pub children: Vec<usize>,
    /// Its place among its parent's children.
    pub index: usize,
    /// What it holds.
    pub value: Val,
    /// How many times the meter has said its value (`update_number`).
    pub updates: u32,
}

/// The tree, its root at 0.
#[derive(Clone, Debug)]
pub struct Tree {
    /// Every node.
    pub nodes: Vec<Node>,
    /// The node of each id.
    by_id: Vec<Option<usize>>,
}

/// Why a tree did not unpack.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TreeError {
    /// The zlib stream is broken.
    Decompress(String),
    /// The serialization is (`deserialize_tree`'s `SR_ERR_DATA`).
    Data,
}

impl fmt::Display for TreeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TreeError::Decompress(e) => write!(f, "Tree decompression failed: {e}."),
            TreeError::Data => f.write_str("Tree deserialization failed."),
        }
    }
}

impl Tree {
    fn node(name: &str, kind: Kind, id: Option<u8>, index: usize) -> Node {
        let value = match kind {
            Kind::Str | Kind::Bin => Val::Bytes(Vec::new()),
            _ => Val::None,
        };
        Node { name: name.to_owned(), kind, id, children: Vec::new(), index, value, updates: 0 }
    }

    /// The three nodes a meter has before the handshake
    /// (`allocate_startup_tree`).
    pub fn startup() -> Tree {
        let mut root = Tree::node("ADMIN", Kind::Plain, None, 0);
        root.children = vec![1, 2, 3];
        Tree {
            nodes: vec![
                root,
                Tree::node("CRC", Kind::U32, Some(0), 0),
                Tree::node("TREE", Kind::Bin, Some(1), 1),
                Tree::node("DIAGNOSTIC", Kind::Str, Some(2), 2),
            ],
            by_id: vec![Some(1), Some(2), Some(3)],
        }
    }

    /// The tree a meter sent in ADMIN:TREE: inflated and read.
    ///
    /// # Errors
    ///
    /// A broken zlib stream, or a serialization `deserialize_tree` refuses.
    pub fn unpack(compressed: &[u8]) -> Result<Tree, TreeError> {
        use std::io::Read as _;
        let mut plain = Vec::new();
        // C grows its buffer up to 1 MiB; a tree is a few hundred bytes.
        flate2::read::ZlibDecoder::new(compressed)
            .take(1 << 20)
            .read_to_end(&mut plain)
            .map_err(|e| TreeError::Decompress(e.to_string()))?;
        Tree::deserialize(&plain)
    }

    /// `deserialize_tree`, with its checks.
    ///
    /// # Errors
    ///
    /// `TreeError::Data` where C returns `SR_ERR_DATA`, and for more ids than
    /// 7 bits hold, which C writes past its table.
    pub fn deserialize(data: &[u8]) -> Result<Tree, TreeError> {
        let mut tree = Tree { nodes: Vec::new(), by_id: Vec::new() };
        let mut at = 0;
        tree.read_node(data, &mut at, 0)?;
        Ok(tree)
    }

    fn read_node(&mut self, data: &[u8], at: &mut usize, index: usize) -> Result<usize, TreeError> {
        let left = |at: usize| data.len() - at;
        if left(*at) < 2 {
            return Err(TreeError::Data);
        }
        let kind = Kind::from_code(data[*at]).ok_or(TreeError::Data)?;
        let len = usize::from(data[*at + 1]);
        *at += 2;
        if len > left(*at) {
            return Err(TreeError::Data);
        }
        let name = String::from_utf8_lossy(&data[*at..*at + len]).into_owned();
        *at += len;
        if left(*at) == 0 {
            return Err(TreeError::Data);
        }
        let id = if kind.has_id() {
            let id = self.by_id.len();
            if id >= MAX_IDS {
                return Err(TreeError::Data);
            }
            Some(id as u8)
        } else {
            None
        };
        let me = self.nodes.len();
        self.nodes.push(Tree::node(&name, kind, id, index));
        if id.is_some() {
            self.by_id.push(Some(me));
        }
        let count = usize::from(data[*at]);
        *at += 1;
        for i in 0..count {
            let child = self.read_node(data, at, i)?;
            self.nodes[me].children.push(child);
        }
        Ok(me)
    }

    /// The node of an id.
    pub fn by_id(&self, id: u8) -> Option<usize> {
        self.by_id.get(usize::from(id)).copied().flatten()
    }

    /// `lookup_tree_path`: `A:B:C` from the root, each name in any case; of two
    /// children of one name, the last.
    pub fn lookup(&self, path: &str) -> Option<usize> {
        let mut at = 0;
        for part in path.split(':') {
            at = self.nodes[at]
                .children
                .iter()
                .copied()
                .rfind(|c| self.nodes[*c].name.eq_ignore_ascii_case(part))?;
        }
        Some(at)
    }

    /// `get_tree_integer`: an integer value, a float's cut toward zero, 0 for
    /// the rest.
    pub fn integer(&self, node: usize) -> i32 {
        match self.nodes[node].value {
            Val::Int(v) => v,
            Val::Float(f) => f as i32,
            _ => 0,
        }
    }

    /// `select_next_largest_in_tree`: of the children of `parent` whose names
    /// read as a positive number (`strtof`), the smallest at or above `number`,
    /// or failing any, the largest below it.
    pub fn next_largest(&self, parent: usize, number: f32) -> Option<usize> {
        let mut selected: Option<usize> = None;
        let mut best = 0f32;
        for &c in &self.nodes[parent].children {
            let value = crate::core::util::g_ascii_strtod(self.nodes[c].name.as_bytes())
                .map_or(0.0, |v| v as f32);
            if value <= 0.0 || value.is_nan() {
                continue;
            }
            let distance = value - number;
            if selected.is_none() {
                selected = Some(c);
                best = distance;
                continue;
            }
            if distance < 0.0 {
                if best > 0.0 {
                    continue;
                }
                if distance > best {
                    selected = Some(c);
                    best = distance;
                }
                continue;
            }
            if best < 0.0 || distance < best {
                selected = Some(c);
                best = distance;
            }
        }
        selected
    }

    /// The numbers the children of `node` are named, as `g_ascii_strtod` reads
    /// them (0 for none).
    pub fn child_numbers(&self, node: usize) -> Vec<f64> {
        self.nodes[node]
            .children
            .iter()
            .map(|c| {
                crate::core::util::g_ascii_strtod(self.nodes[*c].name.as_bytes()).unwrap_or(0.0)
            })
            .collect()
    }

    /// A message of the stream, `ID VALUE`, taken in: the node it updated and the
    /// bytes it took, or `None` when it is not whole yet. An unknown id is an
    /// `Err` with the id.
    ///
    /// # Errors
    ///
    /// The id of a node the tree doesn't have.
    pub fn take_message(&mut self, contents: &[u8]) -> Result<Option<(usize, usize)>, u8> {
        if contents.len() < 2 {
            return Ok(None);
        }
        let id = contents[0] & 0x7f;
        let node = self.by_id(id).ok_or(id)?;
        let body = &contents[1..];
        let (value, used) = match self.nodes[node].kind {
            Kind::Plain | Kind::Link => (None, 2),
            Kind::Chooser | Kind::U8 => (Some(Val::Int(i32::from(body[0]))), 2),
            Kind::S8 => (Some(Val::Int(i32::from(body[0] as i8))), 2),
            Kind::U16 | Kind::S16 => {
                if body.len() < 2 {
                    return Ok(None);
                }
                let raw = u16::from_le_bytes([body[0], body[1]]);
                let v = if self.nodes[node].kind == Kind::S16 {
                    i32::from(raw as i16)
                } else {
                    i32::from(raw)
                };
                (Some(Val::Int(v)), 3)
            }
            Kind::U32 | Kind::S32 => {
                if body.len() < 4 {
                    return Ok(None);
                }
                let v = i32::from_le_bytes([body[0], body[1], body[2], body[3]]);
                (Some(Val::Int(v)), 5)
            }
            Kind::Float => {
                if body.len() < 4 {
                    return Ok(None);
                }
                (Some(Val::Float(f32::from_le_bytes([body[0], body[1], body[2], body[3]]))), 5)
            }
            Kind::Str | Kind::Bin => {
                if body.len() < 2 {
                    return Ok(None);
                }
                let len = usize::from(u16::from_le_bytes([body[0], body[1]]));
                if body.len() < 2 + len {
                    return Ok(None);
                }
                (Some(Val::Bytes(body[2..2 + len].to_vec())), 3 + len)
            }
        };
        match value {
            Some(v) => self.nodes[node].value = v,
            None => log::error!("Update for dataless node."),
        }
        self.nodes[node].updates += 1;
        Ok(Some((node, used)))
    }

    /// The bytes of a write of `value` to an integer node, its value held now
    /// (`set_tree_integer`); `None` for a node that takes no integer, which C
    /// leaves unsent.
    pub fn integer_write(&mut self, node: usize, value: i32) -> Option<Vec<u8>> {
        let n = &mut self.nodes[node];
        let id = n.id?;
        let mut out = vec![0x80 | id];
        match n.kind {
            Kind::Plain | Kind::Link => {
                log::error!("Set attempted for dataless node.");
                return None;
            }
            Kind::Chooser | Kind::U8 | Kind::S8 => out.push(value as u8),
            Kind::U16 | Kind::S16 => out.extend_from_slice(&(value as u16).to_le_bytes()),
            Kind::U32 | Kind::S32 => out.extend_from_slice(&value.to_le_bytes()),
            Kind::Str | Kind::Bin | Kind::Float => return None,
        }
        n.value = Val::Int(value);
        Some(out)
    }
}

/// CRC-32 (IEEE), as `protocol.c:1138-1153`.
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

/// A notification that never came: the stream can't be read past it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Lost {
    /// Its sequence number.
    pub seq: u8,
}

/// The incoming stream: notifications put back in order by their numbers.
#[derive(Debug, Default)]
pub struct Rx {
    next: Option<u8>,
    /// Those ahead of `next`, by their numbers.
    held: BTreeMap<u8, Vec<u8>>,
    /// The stream so far, not yet taken.
    pub contents: Vec<u8>,
}

impl Rx {
    /// One notification: `SEQ DATA`.
    ///
    /// # Errors
    ///
    /// [`HELD_MAX`] notifications came after a gap that is still open.
    pub fn frame(&mut self, frame: &[u8]) -> Result<(), Lost> {
        let Some((&seq, data)) = frame.split_first() else { return Ok(()) };
        let Some(next) = self.next else {
            self.next = Some(seq.wrapping_add(1));
            self.contents.extend_from_slice(data);
            return Ok(());
        };
        let ahead = seq.wrapping_sub(next);
        if ahead == 0 {
            self.contents.extend_from_slice(data);
            let mut next = seq.wrapping_add(1);
            while let Some(d) = self.held.remove(&next) {
                self.contents.extend_from_slice(&d);
                next = next.wrapping_add(1);
            }
            self.next = Some(next);
            return Ok(());
        }
        if ahead >= 0x80 {
            log::debug!("Notification {seq} again, or late; dropped.");
            return Ok(());
        }
        log::debug!("Notification {seq} ahead of {next}; held.");
        self.held.insert(seq, data.to_vec());
        if self.held.len() > HELD_MAX {
            return Err(Lost { seq: next });
        }
        Ok(())
    }

    /// Whether a notification is still awaited behind others.
    pub fn gap(&self) -> Option<u8> {
        (!self.held.is_empty()).then(|| self.next.unwrap_or(0))
    }
}

/// What a channel measures: C's `channel_meaning`, with oxrok's unit for a
/// temperature.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Input {
    /// CH1's current.
    Current,
    /// CH2's voltage.
    Voltage,
    /// The internal temperature.
    Temperature,
    /// The shared input, a low voltage.
    AuxV,
    /// The shared input, a resistance.
    Resistance,
    /// The shared input, a diode's drop.
    Diode,
}

impl Input {
    /// The node a channel maps it to, the chooser that picks a shared one, and
    /// the node whose children are its ranges.
    pub fn nodes(self, ch: u8) -> (String, Option<&'static str>, String) {
        match self {
            Input::Current => {
                (format!("CH{ch}:MAPPING:CURRENT"), None, format!("CH{ch}:MAPPING:CURRENT"))
            }
            Input::Voltage => {
                (format!("CH{ch}:MAPPING:VOLTAGE"), None, format!("CH{ch}:MAPPING:VOLTAGE"))
            }
            Input::Temperature => {
                (format!("CH{ch}:MAPPING:TEMP"), None, format!("CH{ch}:MAPPING:TEMP"))
            }
            Input::AuxV => {
                (format!("CH{ch}:MAPPING:SHARED"), Some("SHARED:AUX_V"), "SHARED:AUX_V".to_owned())
            }
            Input::Resistance => (
                format!("CH{ch}:MAPPING:SHARED"),
                Some("SHARED:RESISTANCE"),
                "SHARED:RESISTANCE".to_owned(),
            ),
            Input::Diode => {
                (format!("CH{ch}:MAPPING:SHARED"), Some("SHARED:DIODE"), "SHARED:DIODE".to_owned())
            }
        }
    }

    /// Whether it is on the shared input.
    pub fn shared(self) -> bool {
        matches!(self, Input::AuxV | Input::Resistance | Input::Diode)
    }
}

/// An analysis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Analysis {
    /// The mean.
    Mean,
    /// The RMS.
    Rms,
    /// Every sample.
    Buffer,
}

impl Analysis {
    /// Its node under `CHn:ANALYSIS`.
    pub fn node(self) -> &'static str {
        match self {
            Analysis::Mean => "MEAN",
            Analysis::Rms => "RMS",
            Analysis::Buffer => "BUFFER",
        }
    }
}

/// One channel's part of `channel_config`: a mode, its range (0 for auto) and
/// maybe an analysis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChannelSetting {
    /// The input.
    pub input: Input,
    /// The range's top, 0 or less for autorange.
    pub range: f32,
    /// The analysis, if given.
    pub analysis: Option<Analysis>,
}

/// `channel_config`, read as `apply_channel_config` reads it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ChannelConfig {
    /// CH1's, if given.
    pub ch1: Option<ChannelSetting>,
    /// CH2's, if given.
    pub ch2: Option<ChannelSetting>,
    /// CH2's mode named nothing C knows (an info line, then voltage).
    pub ch2_unrecognized: Option<String>,
}

/// Whether `text` starts with `word`, in any case (`g_ascii_strncasecmp` with
/// the word's length).
fn starts(text: &str, word: &str) -> bool {
    text.len() >= word.len() && text[..word.len()].eq_ignore_ascii_case(word)
}

fn analysis(params: &[&str]) -> Option<Analysis> {
    if params.len() < 3 {
        return None;
    }
    let p = params[2];
    Some(if p.eq_ignore_ascii_case("RMS") || p.eq_ignore_ascii_case("AC") {
        Analysis::Rms
    } else if p.eq_ignore_ascii_case("Buffer") || p.eq_ignore_ascii_case("Samples") {
        Analysis::Buffer
    } else {
        Analysis::Mean
    })
}

fn range(params: &[&str]) -> f32 {
    params
        .get(1)
        .and_then(|r| crate::core::util::g_ascii_strtod(r.as_bytes()))
        .map_or(0.0, |v| v as f32)
}

/// Reads `CH1[,CH2]` (or `/`), each `MODE[:RANGE[:ANALYSIS]]` (or `;`). CH1's
/// modes: Current/A, Temperature/T/K, Resistance/Ohm/W/R, Diode/D, Aux/LV, any
/// other text being current; CH2's: Voltage/V, the same temperature and shared
/// ones, any other being voltage with a note. A mode is the start of the word
/// in any case, as C compares. An empty part leaves its channel be.
///
/// # Errors
///
/// Both channels on the shared input, which C refuses with `SR_ERR` once CH1
/// is set.
pub fn parse_channel_config(text: &str) -> (ChannelConfig, bool) {
    let mut cfg = ChannelConfig::default();
    let parts: Vec<&str> = text.split([',', '/']).collect();
    let one = |part: &str, ch: u8| -> Option<(ChannelSetting, Option<String>)> {
        let params: Vec<&str> = part.split([':', ';']).collect();
        let mode = params[0];
        if mode.is_empty() {
            return None;
        }
        let shared = if starts(mode, "Resistance")
            || starts(mode, "Ohm")
            || starts(mode, "W")
            || starts(mode, "R")
        {
            Some(Input::Resistance)
        } else if starts(mode, "Diode") || starts(mode, "D") {
            Some(Input::Diode)
        } else if starts(mode, "Aux") || starts(mode, "LV") {
            Some(Input::AuxV)
        } else {
            None
        };
        let mut note = None;
        let input = if let Some(s) = shared {
            s
        } else if starts(mode, "T") || starts(mode, "K") {
            Input::Temperature
        } else if ch == 1 {
            Input::Current
        } else {
            if !starts(mode, "V") {
                note = Some(mode.to_owned());
            }
            Input::Voltage
        };
        Some((ChannelSetting { input, range: range(&params), analysis: analysis(&params) }, note))
    };
    if let Some(p) = parts.first() {
        cfg.ch1 = one(p, 1).map(|(s, _)| s);
    }
    let mut refused = false;
    if let Some(p) = parts.get(1)
        && let Some((s, note)) = one(p, 2)
    {
        if s.input.shared() && cfg.ch1.is_some_and(|c| c.input.shared()) {
            refused = true;
        } else {
            cfg.ch2 = Some(s);
            cfg.ch2_unrecognized = note;
        }
    }
    (cfg, refused)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A small tree: root { ADMIN { CRC32 U32, TREE BIN, DIAGNOSTIC STR }, X
    /// CHOOSER { 10, 600 } }.
    fn serialized() -> Vec<u8> {
        let mut t = vec![0, 0, 2];
        t.extend_from_slice(&[0, 5]);
        t.extend_from_slice(b"ADMIN");
        t.push(3);
        for (k, n) in [(5u8, &b"CRC32"[..]), (10, b"TREE"), (9, b"DIAGNOSTIC")] {
            t.extend_from_slice(&[k, n.len() as u8]);
            t.extend_from_slice(n);
            t.push(0);
        }
        t.extend_from_slice(&[2, 1, b'X', 2]);
        for n in [&b"10"[..], b"600"] {
            t.extend_from_slice(&[0, n.len() as u8]);
            t.extend_from_slice(n);
            t.push(0);
        }
        t
    }

    #[test]
    fn a_tree_reads_with_depth_first_ids() {
        let tree = Tree::deserialize(&serialized()).unwrap();
        let x = tree.lookup("x").unwrap();
        assert_eq!(tree.nodes[x].id, Some(3));
        assert_eq!(tree.lookup("ADMIN:crc32"), tree.by_id(0));
        assert_eq!(tree.child_numbers(x), [10.0, 600.0]);
        assert_eq!(tree.lookup("ADMIN:NOPE"), None);
    }

    #[test]
    fn a_tree_cut_short_is_refused_as_c_refuses_it() {
        let t = serialized();
        for cut in [1, 3, 8, t.len() - 1] {
            assert_eq!(Tree::deserialize(&t[..cut]).unwrap_err(), TreeError::Data, "{cut}");
        }
        let mut bad = t.clone();
        bad[0] = 12;
        assert_eq!(Tree::deserialize(&bad).unwrap_err(), TreeError::Data);
        assert!(matches!(Tree::unpack(b"not zlib"), Err(TreeError::Decompress(_))));
    }

    #[test]
    fn the_next_largest_is_the_least_above_or_the_most_below() {
        let tree = Tree::deserialize(&serialized()).unwrap();
        let x = tree.lookup("X").unwrap();
        let name = |n: f32| tree.nodes[tree.next_largest(x, n).unwrap()].name.clone();
        assert_eq!(name(0.0), "10");
        assert_eq!(name(10.0), "10");
        assert_eq!(name(10.5), "600");
        assert_eq!(name(1e6), "600");
        assert_eq!(name(-5.0), "10");
    }

    #[test]
    fn messages_are_framed_by_their_node_types() {
        let mut tree = Tree::deserialize(&serialized()).unwrap();
        assert_eq!(tree.take_message(&[0x00, 1, 2, 3]), Ok(None), "a u32 cut short");
        assert_eq!(tree.take_message(&[0x00, 1, 2, 3, 4, 9]), Ok(Some((2, 5))));
        assert_eq!(tree.nodes[2].value, Val::Int(0x0403_0201));
        assert_eq!(tree.take_message(&[0x02, 3, 0, b'a', b'b']), Ok(None));
        assert_eq!(tree.take_message(&[0x02, 2, 0, b'a', b'b']), Ok(Some((4, 5))));
        assert_eq!(tree.take_message(&[0x03, 1]), Ok(Some((5, 2))));
        assert_eq!(tree.take_message(&[0x7f, 1]), Err(0x7f));
        assert_eq!(tree.integer_write(5, 1), Some(vec![0x83, 1]));
        assert_eq!(tree.integer_write(2, -1), Some(vec![0x80, 0xff, 0xff, 0xff, 0xff]));
        assert_eq!(tree.integer_write(3, 1), None, "a blob takes no integer");
    }

    #[test]
    fn the_stream_is_put_back_in_order() {
        let mut rx = Rx::default();
        rx.frame(&[250, b'a']).unwrap();
        rx.frame(&[252, b'c']).unwrap();
        rx.frame(&[251, b'b']).unwrap();
        rx.frame(&[251, b'B']).unwrap(); // again: dropped
        assert_eq!(rx.contents, b"abc");
        // A gap of two, filled out of order.
        rx.frame(&[0, b'g']).unwrap();
        rx.frame(&[254, b'e']).unwrap();
        rx.frame(&[253, b'd']).unwrap();
        assert_eq!(rx.contents, b"abcde");
        rx.frame(&[255, b'f']).unwrap();
        assert_eq!(rx.contents, b"abcdefg");
        assert_eq!(rx.gap(), None);
    }

    #[test]
    fn a_gap_held_open_too_long_is_a_loss() {
        let mut rx = Rx::default();
        rx.frame(&[0, b'a']).unwrap();
        for s in 2..2 + HELD_MAX as u8 {
            rx.frame(&[s, b'x']).unwrap();
        }
        assert_eq!(rx.gap(), Some(1));
        assert_eq!(rx.frame(&[2 + HELD_MAX as u8, b'x']), Err(Lost { seq: 1 }));
    }

    #[test]
    fn channel_config_reads_as_c_reads_it() {
        let cfg = |t: &str| parse_channel_config(t);
        let s = |input, range, analysis| Some(ChannelSetting { input, range, analysis });
        assert_eq!(cfg("A").0.ch1, s(Input::Current, 0.0, None));
        assert_eq!(cfg("Volts").0.ch1, s(Input::Current, 0.0, None), "any word is current");
        assert_eq!(cfg("Rubbish:5e6").0.ch1, s(Input::Resistance, 5e6, None));
        assert_eq!(cfg("Wx").0.ch1, s(Input::Resistance, 0.0, None));
        assert_eq!(cfg("T:350:ac").0.ch1, s(Input::Temperature, 350.0, Some(Analysis::Rms)));
        assert_eq!(cfg("LV:0.2:x").0.ch1, s(Input::AuxV, 0.2, Some(Analysis::Mean)));
        assert_eq!(cfg("A;10;Samples/V;600").0.ch2, s(Input::Voltage, 600.0, None));
        assert_eq!(cfg("A;10;Samples").0.ch1.unwrap().analysis, Some(Analysis::Buffer));
        assert_eq!(cfg(",V").0.ch1, None);
        assert_eq!(cfg("A,").0.ch2, None);
        assert_eq!(cfg("A:x").0.ch1, s(Input::Current, 0.0, None));
        let (c, refused) = cfg("A,Xyz");
        assert_eq!(c.ch2, s(Input::Voltage, 0.0, None));
        assert_eq!(c.ch2_unrecognized.as_deref(), Some("Xyz"));
        assert!(!refused);
        assert!(cfg("R,D").1);
        assert!(cfg("LV,R").1);
        assert!(!cfg("T,R").1);
    }

    #[test]
    fn crc32_is_zlib_s() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }
}
