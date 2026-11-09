// Counterpart in sigrok:
//   libsigrok/src/hardware/mooshimeter-dmm/api.c,
//   libsigrok/src/hardware/mooshimeter-dmm/protocol.c,
//   Copyright (C) 2019 Derek Hageman <hageman@inthat.cloud>,
//   with commits by Gerhard Sittig, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Mooshimeter, a two channel multimeter on Bluetooth LE, through
//! [`crate::bt`]: `conn=` is its address (`D0-5F-B8-4D-00-01`, or with colons
//! from a library, or dense). Linux only, as libsigrok's `bt/` layer is.
//!
//! A scan connects and disconnects, and finds a meter in whatever answers.
//! An open connects again, turns SEROUT's notifications on, reads the tree and
//! writes its CRC back (see [`protocol`]), then sets what C's `dev_open` sets:
//! trigger off, 125 Hz, 64 samples, 24 bit buffers, CH1 current and CH2 voltage
//! on autorange, both the mean. Each setting waits up to 5 s for the meter's
//! echo, the handshake 30 s. A capture sets the trigger to continuous and takes
//! what the meter sends: CH1:VALUE, CH2:VALUE and `REAL_PWR` (channel P, off by
//! default), or in buffer analysis CH1:BUF and CH2:BUF. A value outside its
//! channel's range moves an autoranging channel's range, unasked. Every 10 s it
//! reads `PCB_VERSION`, which keeps the meter from dropping a quiet host. The end
//! sets the trigger off again.
//!
//! `channel_config` is `CH1[,CH2]`, each `MODE[:RANGE[:ANALYSIS]]`, with `;`
//! too between the fields, as sigrok-cli splits a setting at `:`; see
//! [`protocol::parse_channel_config`].
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - CH2's temperature is in kelvin, as CH1's: the meter has one internal
//!   sensor, and its vendor's app says K on both. C says °C on CH2.
//! - A buffer's samples are scaled by `BUF_LSB2NATIVE`, read at the start of the
//!   capture and after an autorange of a buffered channel. C checks the node
//!   for the wrong type and never takes its value, so every sample is 0; it also
//!   autoranges a buffer on 0, not on its largest sample.
//! - A time limit ends the capture when it is up. C looks at it only when a
//!   value comes, so it sends one more after the time, and never ends while the
//!   meter is silent.
//! - A lost notification, the link gone or a meter that stops answering ends the
//!   capture with an error; C polls on for ever. With a sample limit and no time
//!   limit, no reading in 10 s fails the capture, and after readings, 30 s of
//!   silence is told and 10 minutes fails. With a time limit, or none at all
//!   (`--continuous`), silence is only told.
//! - Channel P on a tree without `REAL_PWR` is told at the start, and alone it
//!   refuses the start; C waits for it for ever.
//! - A connection is given 20 s, C's `CONNECT_BLE_TIMEOUT`; C waits for the
//!   kernel's 40 s.
//! - Before the handshake, a message of a node the startup tree lacks (a meter
//!   still sampling for a host that left, say) drops what the stream holds, as
//!   the vendor's app does; C drops one byte and frames on from the next. After
//!   the handshake such a message ends the capture.
//! - The tree takes over within the notification that ends it, so what follows
//!   ADMIN:TREE in it is read with the new tree, as C's `consume_packets` does.
//! - A notification of another handle, an ATT PDU C doesn't handle and an empty
//!   notification are passed over. C's `sr_bt_check_notify` returns -4, -3 and
//!   -1 for them, which fails a setting's wait or the open's handshake, and is
//!   ignored by its capture's poll.
//! - An empty `channel_config` is refused, as C refuses it (`api.c:654-656`).
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::bt::{self, Bearer, Incoming, Notify};
use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use protocol::{Analysis, ChannelConfig, Input, Rx, Tree, TreeError, Val};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
const SET_LIST: Access = Access { get: false, set: true, list: true };
const ALL: Access = Access { get: true, set: true, list: true };
const SET: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 6] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, SET_LIST),
    OptionDesc::new(Key::LimitMsec, SET_LIST),
    OptionDesc::new(Key::AvgSamples, ALL),
    OptionDesc::new(Key::Samplerate, ALL),
    OptionDesc::new(Key::ChannelConfig, SET),
];

/// The handles libsigrok read off a meter with btgatt-client (`api.c:104-114`).
pub const NOTIFY: Notify =
    Notify { read_handle: 0x0015, write_handle: 0x0012, cccd_handle: 0x0016, cccd_value: 0x0001 };

/// `CONNECT_BLE_TIMEOUT`.
pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(20);
/// `wait_for_update`'s bound.
const UPDATE_TIMEOUT: Duration = Duration::from_secs(5);
/// `startup_run`'s bound.
const STARTUP_TIMEOUT: Duration = Duration::from_secs(30);
/// The capture's times, which a test shortens.
#[derive(Clone, Copy, Debug)]
struct Bounds {
    /// The heartbeat's period (`dev_acquisition_start`).
    heartbeat: Duration,
    /// A capture with no reading yet fails after this.
    first_reading: Duration,
    /// After a reading, silence this long is told.
    silence_warning: Duration,
    /// And this long fails.
    silence_limit: Duration,
    /// A setting's wait for its echo.
    update: Duration,
}

const BOUNDS: Bounds = Bounds {
    heartbeat: Duration::from_secs(10),
    first_reading: FIRST_READING,
    silence_warning: SILENCE_WARNING,
    silence_limit: SILENCE_LIMIT,
    update: UPDATE_TIMEOUT,
};
/// How long one look at the link waits: C's poll period.
const POLL: Duration = Duration::from_millis(50);
/// A capture with no reading yet fails after this: the slowest the meter goes
/// is a reading every 2.048 s (256 samples at 125 Hz).
const FIRST_READING: Duration = Duration::from_secs(10);
/// After a reading, silence this long is told.
const SILENCE_WARNING: Duration = Duration::from_secs(30);
/// And this long fails.
const SILENCE_LIMIT: Duration = Duration::from_secs(600);

/// The `mooshimeter-dmm` driver.
pub struct MooshimeterDriver;

/// What a channel's packets say it measures (C's `channel_meaning`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Meaning {
    /// The quantity.
    pub mq: Mq,
    /// The unit.
    pub unit: Unit,
    /// The flags.
    pub flags: MqFlags,
}

/// An open connection to a meter: the link, both sequence counts and the tree.
pub struct Meter {
    bearer: Box<dyn Bearer>,
    tx_seq: u8,
    rx: Rx,
    tree: Tree,
    /// CH1, CH2 and P.
    meanings: [Meaning; 3],
    /// The input an autoranging channel ranges in.
    autorange: [Option<Input>; 2],
    /// ADMIN:DIAGNOSTIC, whose updates are told (`tree_diagnostic_updated`).
    diagnostic_node: Option<usize>,
    /// How long a setting waits for its echo (`wait_for_update`).
    update_timeout: Duration,
    /// The link failed: nothing more can be sent.
    link_failed: bool,
    /// The CRC of the tree that took over from the startup one, once one has.
    tree_crc: Option<u32>,
}

/// `flags` with `off` cleared.
fn without(flags: MqFlags, off: MqFlags) -> MqFlags {
    flags.iter().filter(|f| !off.contains(*f)).fold(MqFlags::empty(), |a, f| a | f)
}

/// Why a wait for the meter ended without what it waited for.
fn timeout(what: &str) -> Error {
    Error::device(format!("No answer from the meter to {what} in time."))
}

fn link_error(e: &std::io::Error) -> Error {
    Error::device(format!("The link to the meter failed: {e}."))
}

impl Meter {
    fn new(bearer: Box<dyn Bearer>) -> Meter {
        Meter {
            bearer,
            tx_seq: 0,
            rx: Rx::default(),
            tree: Tree::startup(),
            meanings: [
                Meaning { mq: Mq::Current, unit: Unit::Ampere, flags: MqFlags::DC },
                Meaning { mq: Mq::Voltage, unit: Unit::Volt, flags: MqFlags::DC },
                Meaning { mq: Mq::Power, unit: Unit::Watt, flags: MqFlags::RMS },
            ],
            autorange: [None, None],
            diagnostic_node: Some(3),
            update_timeout: UPDATE_TIMEOUT,
            link_failed: false,
            tree_crc: None,
        }
    }

    /// A link error, noted: nothing more goes out on this link.
    fn link_error(&mut self, e: &std::io::Error) -> Error {
        self.link_failed = true;
        link_error(e)
    }

    /// `mooshimeter_dmm_open`: notifications on, the tree read and unlocked.
    ///
    /// # Errors
    ///
    /// The link fails, the tree is broken, its CRC comes back wrong, or the
    /// meter is silent for 30 s.
    pub fn connect(bearer: Box<dyn Bearer>) -> Result<Meter> {
        let mut m = Meter::new(bearer);
        NOTIFY.start(m.bearer.as_mut()).map_err(|e| m.link_error(&e))?;
        log::trace!("Initiating startup handshake.");
        let tree_node = m.tree.by_id(1).expect("the startup tree has it");
        m.poll(tree_node)?;
        let started = Instant::now();
        let crc = loop {
            m.pump(POLL)?;
            if let Some(crc) = m.tree_crc {
                break crc;
            }
            if started.elapsed() > STARTUP_TIMEOUT {
                log::debug!("Startup handshake failed: timeout.");
                return Err(timeout("the tree request"));
            }
        };
        let Some(crc_node) = m.tree.lookup("ADMIN:CRC32") else {
            log::error!("ADMIN:CRC32 node not found in received startup tree.");
            return Err(Error::device("ADMIN:CRC32 node not found in received startup tree."));
        };
        let before = m.tree.nodes[crc_node].updates;
        m.set_integer(crc_node, crc as i32)?;
        m.wait_update(
            crc_node,
            before,
            STARTUP_TIMEOUT.saturating_sub(started.elapsed()),
            "the CRC",
        )?;
        let echoed = m.tree.integer(crc_node) as u32;
        if echoed != crc {
            log::error!("Tree CRC mismatch, expected {crc:08X} but received {echoed:08X}.");
            return Err(Error::device(format!(
                "Tree CRC mismatch, expected {crc:08X} but received {echoed:08X}."
            )));
        }
        log::debug!("Startup handshake completed.");
        m.check_paths();
        Ok(m)
    }

    /// What `startup_tree_updated` does with ADMIN:TREE: its CRC, and the tree
    /// it holds in place of the startup one.
    fn take_tree(&mut self, node: usize) -> Result<u32> {
        let Val::Bytes(compressed) = self.tree.nodes[node].value.clone() else {
            return Err(Error::device("ADMIN:TREE holds no bytes"));
        };
        let crc = protocol::crc32(&compressed);
        let tree = Tree::unpack(&compressed).map_err(|e| {
            log::error!("{e}");
            Error::device(match e {
                TreeError::Decompress(d) => format!("Tree decompression failed: {d}."),
                TreeError::Data => "Tree deserialization failed.".to_owned(),
            })
        })?;
        log::debug!("Config tree received ({} bytes) with CRC {crc:08X}.", compressed.len());
        self.diagnostic_node = tree.lookup("ADMIN:DIAGNOSTIC");
        self.tree = tree;
        Ok(crc)
    }

    /// `install_update_handlers`' warnings for what a tree lacks.
    fn check_paths(&self) {
        for (path, what) in [
            ("CH1:VALUE", "channel 1 values"),
            ("CH1:BUF", "channel 1 buffer"),
            ("CH1:BUF_BPS", "channel 1 buffer BPS"),
            ("CH1:BUF_LSB2NATIVE", "channel 1 buffer conversion factor"),
            ("CH2:VALUE", "channel 2 values"),
            ("CH2:BUF", "channel 2 buffer"),
            ("CH2:BUF_BPS", "channel 2 buffer BPS"),
            ("CH2:BUF_LSB2NATIVE", "channel 2 buffer conversion factor"),
            ("REAL_PWR", "real power"),
        ] {
            if self.tree.lookup(path).is_none() {
                log::warn!("No tree path for {what}.");
            }
        }
    }

    /// `tree_diagnostic_updated`.
    fn diagnostic(&self, node: usize) {
        match &self.tree.nodes[node].value {
            Val::Bytes(b) if !b.is_empty() => {
                let end = b.iter().position(|c| *c == 0).unwrap_or(b.len());
                log::warn!("Mooshimeter error: {}.", String::from_utf8_lossy(&b[..end]));
            }
            _ => log::warn!("Mooshimeter error with no information."),
        }
    }

    /// One frame out: the next sequence number and `bytes`.
    fn write_frame(&mut self, bytes: &[u8]) -> Result<()> {
        let mut frame = vec![self.tx_seq];
        frame.extend_from_slice(bytes);
        self.tx_seq = self.tx_seq.wrapping_add(1);
        NOTIFY.write(self.bearer.as_mut(), &frame).map_err(|e| self.link_error(&e))
    }

    /// `poll_tree_value`: a read.
    fn poll(&mut self, node: usize) -> Result<()> {
        let id = self.tree.nodes[node].id.ok_or_else(|| Error::device("a node with no id"))?;
        self.write_frame(&[id])
    }

    /// `set_tree_integer`: a write, not waited for. C sends nothing for a node
    /// of a type with no integer, and neither does this.
    fn set_integer(&mut self, node: usize, value: i32) -> Result<()> {
        match self.tree.integer_write(node, value) {
            Some(bytes) => self.write_frame(&bytes),
            None => Ok(()),
        }
    }

    /// One look at the link: the nodes the meter updated, in order.
    ///
    /// # Errors
    ///
    /// The link has gone, a notification was lost, or a message names a node
    /// the tree lacks.
    fn pump(&mut self, wait: Duration) -> Result<Vec<usize>> {
        let incoming = NOTIFY.check(self.bearer.as_mut(), wait).map_err(|e| self.link_error(&e))?;
        let Some(Incoming::Data(frame)) = incoming else { return Ok(Vec::new()) };
        if let Err(lost) = self.rx.frame(&frame) {
            return Err(Error::device(format!(
                "Notification {} from the meter was lost; its stream can't be read past it.",
                lost.seq
            )));
        }
        let mut updated = Vec::new();
        loop {
            let startup = self.tree_crc.is_none();
            match self.tree.take_message(&self.rx.contents) {
                Ok(Some((node, used))) => {
                    self.rx.contents.drain(..used);
                    if Some(node) == self.diagnostic_node {
                        self.diagnostic(node);
                    }
                    if startup && self.tree.nodes[node].id == Some(1) {
                        // `startup_tree_updated`, here, so that what follows in
                        // this notification is read with the new tree.
                        let crc = self.take_tree(node)?;
                        self.tree_crc = Some(crc);
                        updated.clear();
                        continue;
                    }
                    updated.push(node);
                }
                Ok(None) => break,
                Err(id) if startup => {
                    // The vendor's app clears its buffer at an unknown id: what
                    // a meter sends ahead of the handshake can't be framed.
                    log::error!("Command {id} code does not map to a known node.");
                    log::debug!(
                        "Dropping {} bytes received before the handshake.",
                        self.rx.contents.len()
                    );
                    self.rx.contents.clear();
                    break;
                }
                Err(id) => {
                    log::error!("Command {id} code does not map to a known node.");
                    return Err(Error::device(format!(
                        "The meter sent node {id}, which its tree lacks; its stream can't be read past it."
                    )));
                }
            }
        }
        Ok(updated)
    }

    /// `wait_for_update`: until `node` has been said again, up to `bound`.
    fn wait_update(&mut self, node: usize, before: u32, bound: Duration, what: &str) -> Result<()> {
        let started = Instant::now();
        while self.tree.nodes[node].updates == before {
            if started.elapsed() > bound {
                return Err(timeout(what));
            }
            self.pump(POLL)?;
        }
        Ok(())
    }

    fn path(&self, path: &str) -> Result<usize> {
        self.tree.lookup(path).ok_or_else(|| {
            log::error!("Tree path {path} not found.");
            Error::device(format!("Tree path {path} not found."))
        })
    }

    /// `mooshimeter_dmm_set_chooser`.
    ///
    /// # Errors
    ///
    /// A path the tree lacks, the link's, or no echo in 5 s.
    pub fn set_chooser(&mut self, path: &str, choice: &str) -> Result<()> {
        let Some(chosen) = self.tree.lookup(choice) else {
            log::error!("Value {choice} not found for chooser {path}.");
            return Err(Error::device(format!("Value {choice} not found for chooser {path}.")));
        };
        let index = self.tree.nodes[chosen].index as i32;
        let node = self.path(path)?;
        log::trace!("Setting chooser {path} to {choice} ({index}).");
        let before = self.tree.nodes[node].updates;
        self.set_integer(node, index)?;
        self.wait_update(node, before, self.update_timeout, path)
    }

    /// The trigger off, sent and not waited for.
    fn trigger_off_unwaited(&mut self) -> Result<()> {
        let node = self.path("SAMPLING:TRIGGER")?;
        let off = self.tree.lookup("SAMPLING:TRIGGER:OFF").map_or(0, |n| self.tree.nodes[n].index);
        self.set_integer(node, off as i32)
    }

    /// `mooshimeter_dmm_set_integer`.
    fn set_value(&mut self, path: &str, value: i32) -> Result<()> {
        let node = self.path(path)?;
        log::trace!("Setting integer {path} to {value}.");
        let before = self.tree.nodes[node].updates;
        self.set_integer(node, value)?;
        self.wait_update(node, before, self.update_timeout, path)
    }

    /// `select_next_largest_in_tree`, with its errors.
    fn next_largest(&self, parent: &str, number: f32) -> Result<usize> {
        let p = self.path(parent)?;
        if self.tree.nodes[p].children.is_empty() {
            log::error!("Tree path {parent} has no children.");
            return Err(Error::device(format!("Tree path {parent} has no children.")));
        }
        self.tree.next_largest(p, number).ok_or_else(|| {
            log::error!("No choice available for {number:.6} at {parent}.");
            Error::not_supported(format!("No choice available for {number:.6} at {parent}."))
        })
    }

    /// `mooshimeter_dmm_set_larger_number`.
    ///
    /// # Errors
    ///
    /// As [`Meter::set_chooser`], and no choice at all.
    pub fn set_larger_number(&mut self, path: &str, parent: &str, number: f32) -> Result<()> {
        let choice = self.next_largest(parent, number)?;
        let node = self.path(path)?;
        let index = self.tree.nodes[choice].index as i32;
        log::trace!("Setting number choice {path} to index {index} for requested {number}.");
        let before = self.tree.nodes[node].updates;
        self.set_integer(node, index)?;
        self.wait_update(node, before, self.update_timeout, path)
    }

    /// `mooshimeter_dmm_set_autorange`: a range for `latest`, if another, sent
    /// and not waited for. Whether it moved.
    fn autorange(&mut self, ch: usize, latest: f32) -> Result<bool> {
        let Some(input) = self.autorange[ch] else { return Ok(false) };
        let (_, _, ranges) = input.nodes(ch as u8 + 1);
        let path = format!("CH{}:RANGE_I", ch + 1);
        let Ok(choice) = self.next_largest(&ranges, latest.abs()) else { return Ok(false) };
        let Ok(node) = self.path(&path) else { return Ok(false) };
        let index = self.tree.nodes[choice].index as i32;
        if self.tree.integer(node) == index {
            return Ok(false);
        }
        log::trace!("Changing autorange {path} to index {index} for {latest}.");
        self.set_integer(node, index)?;
        Ok(true)
    }

    /// `mooshimeter_dmm_get_chosen_number`.
    fn chosen_number(&self, path: &str, parent: &str) -> Result<f64> {
        let node = self.path(path)?;
        let available = self.path(parent)?;
        let selected = self.tree.integer(node);
        let children = &self.tree.nodes[available].children;
        let child = usize::try_from(selected)
            .ok()
            .and_then(|i| children.get(i))
            .ok_or_else(|| Error::device(format!("{path} holds no choice of {parent}")))?;
        Ok(crate::core::util::g_ascii_strtod(self.tree.nodes[*child].name.as_bytes())
            .unwrap_or(0.0))
    }

    /// `mooshimeter_dmm_get_available_number_choices`.
    fn number_choices(&self, path: &str) -> Result<Vec<f64>> {
        let node = self.tree.lookup(path).ok_or_else(|| {
            log::error!("Tree path {path} not found.");
            Error::not_supported(format!("Tree path {path} not found."))
        })?;
        Ok(self.tree.child_numbers(node))
    }

    /// The `configure_channelN_*` of an input: its mapping, its range, and the
    /// meaning of the channel's packets.
    fn configure(&mut self, ch: u8, input: Input, range: f32) -> Result<()> {
        let (mapping, shared, ranges) = input.nodes(ch);
        if let Some(shared) = shared {
            self.set_chooser("SHARED", shared)?;
        }
        self.set_chooser(&format!("CH{ch}:MAPPING"), &mapping)?;
        self.set_larger_number(&format!("CH{ch}:RANGE_I"), &ranges, range)?;
        let i = usize::from(ch - 1);
        let m = &mut self.meanings[i];
        if range <= 0.0 {
            self.autorange[i] = Some(input);
            m.flags |= MqFlags::AUTORANGE;
        } else {
            self.autorange[i] = None;
            m.flags = without(m.flags, MqFlags::AUTORANGE);
        }
        if input == Input::Diode {
            m.flags |= MqFlags::DIODE;
        } else {
            m.flags = without(m.flags, MqFlags::DIODE);
        }
        (m.mq, m.unit) = match input {
            Input::Current => (Mq::Current, Unit::Ampere),
            Input::Temperature => (Mq::Temperature, Unit::Kelvin),
            Input::Resistance => (Mq::Resistance, Unit::Ohm),
            Input::Voltage | Input::AuxV | Input::Diode => (Mq::Voltage, Unit::Volt),
        };
        Ok(())
    }

    /// `set_channelN_mean`, `_rms` and `_buffer`.
    fn set_analysis(&mut self, ch: u8, analysis: Analysis) -> Result<()> {
        let m = &mut self.meanings[usize::from(ch - 1)];
        match analysis {
            Analysis::Mean => m.flags = without(m.flags, MqFlags::RMS) | MqFlags::DC,
            Analysis::Rms => m.flags = without(m.flags, MqFlags::DC) | MqFlags::RMS,
            Analysis::Buffer => m.flags = without(without(m.flags, MqFlags::DC), MqFlags::RMS),
        }
        self.set_chooser(
            &format!("CH{ch}:ANALYSIS"),
            &format!("CH{ch}:ANALYSIS:{}", analysis.node()),
        )
    }

    /// `apply_channel_config`.
    fn apply(&mut self, cfg: &ChannelConfig, refused: bool) -> Result<()> {
        if let Some(s) = cfg.ch1 {
            self.configure(1, s.input, s.range)?;
            if let Some(a) = s.analysis {
                self.set_analysis(1, a)?;
            }
        }
        if refused {
            return Err(Error::arg("CH1 and CH2 can't both take the shared input"));
        }
        if let Some(s) = cfg.ch2 {
            if let Some(mode) = &cfg.ch2_unrecognized {
                log::info!("Unrecognized mode for CH2: {mode}.");
            }
            self.configure(2, s.input, s.range)?;
            if let Some(a) = s.analysis {
                self.set_analysis(2, a)?;
            }
        }
        Ok(())
    }

    /// What `dev_open` sets after the handshake.
    fn defaults(&mut self) -> Result<()> {
        self.set_chooser("SAMPLING:TRIGGER", "SAMPLING:TRIGGER:OFF")?;
        self.set_larger_number("SAMPLING:RATE", "SAMPLING:RATE", 125.0)?;
        self.set_larger_number("SAMPLING:DEPTH", "SAMPLING:DEPTH", 64.0)?;
        // "Looks like these sometimes get set to 8, somehow?"
        self.set_value("CH1:BUF_BPS", 24)?;
        self.set_value("CH2:BUF_BPS", 24)?;
        self.configure(1, Input::Current, 0.0)?;
        self.set_analysis(1, Analysis::Mean)?;
        self.configure(2, Input::Voltage, 0.0)?;
        self.set_analysis(2, Analysis::Mean)
    }

    /// Whether a channel's analysis is the buffer, as the tree says.
    fn buffered(&self, ch: u8) -> bool {
        self.tree.lookup(&format!("CH{ch}:ANALYSIS")).is_some_and(|node| {
            let i = usize::try_from(self.tree.integer(node)).ok();
            i.and_then(|i| self.tree.nodes[node].children.get(i))
                .is_some_and(|c| self.tree.nodes[*c].name.eq_ignore_ascii_case("BUFFER"))
        })
    }

    /// The samples of a buffer node, decoded as `chX_buffer_update` does,
    /// scaled by `BUF_LSB2NATIVE`; `None` where C sends nothing.
    fn buffer_samples(&self, ch: u8, raw: &[u8]) -> Option<Vec<f32>> {
        let bps_node = self.tree.lookup(&format!("CH{ch}:BUF_BPS"))?;
        let bits = u32::try_from(self.tree.integer(bps_node)).ok().filter(|b| *b > 0)?;
        let bytes = bits.div_ceil(8) as usize;
        if bytes > 4 || raw.is_empty() || raw.len() < bytes {
            return None;
        }
        let lsb = match self
            .tree
            .lookup(&format!("CH{ch}:BUF_LSB2NATIVE"))
            .map(|n| &self.tree.nodes[n].value)
        {
            Some(Val::Float(f)) => *f,
            _ => 0.0,
        };
        let sign = 1i64 << (bits - 1);
        Some(
            raw.chunks_exact(bytes)
                .map(|s| {
                    let mut w = [0u8; 8];
                    w[..bytes].copy_from_slice(s);
                    let u = i64::from_le_bytes(w);
                    let v = (u & (sign - 1)) - (u & sign);
                    v as f32 * lsb
                })
                .collect(),
        )
    }

    /// The packet a node's update makes for a capture: its channel, the packet
    /// and how many samples it counts; `None` for any other node.
    fn reading(&self, node: usize) -> Option<(usize, AnalogData)> {
        let path = |p: &str| self.tree.lookup(p) == Some(node);
        let (ch, buffer) = if path("CH1:VALUE") {
            (0, false)
        } else if path("CH2:VALUE") {
            (1, false)
        } else if path("REAL_PWR") {
            (2, false)
        } else if path("CH1:BUF") {
            (0, true)
        } else if path("CH2:BUF") {
            (1, true)
        } else {
            return None;
        };
        let m = self.meanings[ch];
        let (samples, digits) = if buffer {
            let Val::Bytes(raw) = &self.tree.nodes[node].value else { return None };
            (self.buffer_samples(ch as u8 + 1, raw)?, 0)
        } else {
            let Val::Float(v) = self.tree.nodes[node].value else { return None };
            (vec![v], 2)
        };
        Some((
            ch,
            AnalogData {
                channels: vec![ch],
                samples,
                mq: m.mq,
                flags: m.flags,
                unit: m.unit,
                digits,
            },
        ))
    }

    /// The buffered channels' scale, read: at the start of a capture.
    fn read_scales(&mut self, enabled: [bool; 3]) -> Result<()> {
        for ch in [1u8, 2] {
            if !enabled[usize::from(ch - 1)] || !self.buffered(ch) {
                continue;
            }
            let path = format!("CH{ch}:BUF_LSB2NATIVE");
            let node = self.path(&path)?;
            let before = self.tree.nodes[node].updates;
            self.poll(node)?;
            self.wait_update(node, before, self.update_timeout, &path)?;
        }
        Ok(())
    }
}

/// One meter.
pub struct MooshimeterDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    addr: [u8; 6],
    limit_samples: u64,
    limit_msec: u64,
    meter: Arc<Mutex<Option<Meter>>>,
    /// A link to use instead of connecting, for tests.
    pending: Option<Box<dyn Bearer>>,
    worker: Option<Worker>,
    capturing: Arc<AtomicBool>,
    bounds: Bounds,
    /// How many times a capture has told of silence, for the tests.
    silences_told: Arc<std::sync::atomic::AtomicUsize>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl MooshimeterDevice {
    fn new(addr: [u8; 6]) -> MooshimeterDevice {
        let mut power = Channel::new(2, ChannelKind::Analog, "P");
        power.enabled = false;
        MooshimeterDevice {
            info: DeviceInfo {
                driver: "mooshimeter-dmm",
                vendor: "Mooshim Engineering".to_owned(),
                model: "Mooshimeter".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: vec![
                Channel::new(0, ChannelKind::Analog, "CH1"),
                Channel::new(1, ChannelKind::Analog, "CH2"),
                power,
            ],
            addr,
            limit_samples: 0,
            limit_msec: 0,
            meter: Arc::new(Mutex::new(None)),
            pending: None,
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            silences_told: Arc::default(),
            bounds: BOUNDS,
        }
    }

    /// A device on a link that stands in for the meter, without a scan.
    /// [`Device::open`] takes it.
    pub fn from_bearer(bearer: Box<dyn Bearer>) -> MooshimeterDevice {
        let mut d = MooshimeterDevice::new([0; 6]);
        d.pending = Some(bearer);
        d
    }

    fn meter(&self) -> MutexGuard<'_, Option<Meter>> {
        self.meter.lock().expect("meter lock")
    }

    /// The open meter, for a setting.
    fn with_meter<T>(&self, f: impl FnOnce(&mut Meter) -> Result<T>) -> Result<T> {
        let mut guard = self.meter();
        let m = guard.as_mut().ok_or_else(|| Error::device("device is not open"))?;
        f(m)
    }

    fn end_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for MooshimeterDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn enable_channel(&mut self, pos: usize, enabled: bool) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        ch.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        name.clone_into(&mut ch.name);
        Ok(())
    }

    fn options(&self, _group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(DEV_OPTIONS.to_vec())
    }

    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        match key {
            Key::Samplerate => self
                .with_meter(|m| m.chosen_number("SAMPLING:RATE", "SAMPLING:RATE"))
                .map(|v| Value::U64(v as f32 as u64)),
            Key::AvgSamples => self
                .with_meter(|m| m.chosen_number("SAMPLING:DEPTH", "SAMPLING:DEPTH"))
                .map(|v| Value::U64(v as f32 as u64)),
            Key::LimitSamples => Ok(Value::U64(self.limit_samples)),
            Key::LimitMsec => Ok(Value::U64(self.limit_msec)),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let number =
            || value.as_u64().ok_or_else(|| Error::arg(format!("invalid value for {}", key.id())));
        match key {
            Key::Samplerate => {
                let n = number()? as f32;
                self.with_meter(|m| m.set_larger_number("SAMPLING:RATE", "SAMPLING:RATE", n))
            }
            Key::AvgSamples => {
                let n = number()? as f32;
                self.with_meter(|m| m.set_larger_number("SAMPLING:DEPTH", "SAMPLING:DEPTH", n))
            }
            Key::ChannelConfig => {
                let text = value.as_str().ok_or_else(|| Error::arg("channel_config"))?;
                if text.is_empty() {
                    return Err(Error::arg("channel_config is empty"));
                }
                let (cfg, refused) = protocol::parse_channel_config(text);
                self.with_meter(|m| m.apply(&cfg, refused))
            }
            Key::LimitSamples => {
                self.limit_samples = number()?;
                Ok(())
            }
            Key::LimitMsec => {
                self.limit_msec = number()?;
                Ok(())
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        let numbers = |path: &str| -> Result<Vec<u64>> {
            Ok(self
                .with_meter(|m| m.number_choices(path))?
                .into_iter()
                .map(|v| v as f32 as u64)
                .collect())
        };
        match key {
            Key::Samplerate => Ok(ValueList::U64(numbers("SAMPLING:RATE")?)),
            Key::AvgSamples => Ok(ValueList::U64(numbers("SAMPLING:DEPTH")?)),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `dev_open`: connect, the handshake, and the settings C makes.
    fn open(&mut self) -> Result<()> {
        if self.meter().is_some() {
            return Ok(());
        }
        let bearer = match self.pending.take() {
            Some(b) => b,
            None => bt::connect(self.addr, CONNECT_TIMEOUT).map_err(|e| {
                Error::device(format!("BLE connect to {} failed: {e}", bt::mac_text(self.addr)))
            })?,
        };
        let mut m = Meter::connect(bearer)?;
        m.update_timeout = self.bounds.update;
        m.defaults()?;
        *self.meter() = Some(m);
        Ok(())
    }

    /// `dev_close`: the link goes.
    fn close(&mut self) -> Result<()> {
        self.end_worker();
        if self.meter().take().is_some() {
            log::debug!("BLE disconnect");
        }
        Ok(())
    }

    /// `dev_acquisition_start`: the trigger to continuous, then a thread takes
    /// what comes.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.end_worker();
        let enabled =
            [self.channels[0].enabled, self.channels[1].enabled, self.channels[2].enabled];
        let no_power =
            enabled[2] && self.with_meter(|m| Ok(m.tree.lookup("REAL_PWR").is_none()))?;
        if no_power {
            if !enabled[0] && !enabled[1] {
                return Err(Error::device(
                    "Channel P has nothing to show: the meter's tree has no REAL_PWR.",
                ));
            }
            log::warn!("Channel P has nothing to show: the meter's tree has no REAL_PWR.");
        }
        self.with_meter(|m| {
            let started = m
                .set_chooser("SAMPLING:TRIGGER", "SAMPLING:TRIGGER:CONTINUOUS")
                .and_then(|()| m.read_scales(enabled));
            if started.is_err() {
                // The meter may have taken it unechoed: off again, as C's
                // stop does after a failed start.
                if let Err(e) = m.set_chooser("SAMPLING:TRIGGER", "SAMPLING:TRIGGER:OFF") {
                    log::warn!("The meter's sampling may still be on: {e}");
                }
            }
            started
        })?;
        let capture = Capture {
            meter: Arc::clone(&self.meter),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            enabled,
            bounds: self.bounds,
            silences_told: Arc::clone(&self.silences_told),
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new().name("mooshimeter-dmm acquisition".into()).spawn(
            move || {
                let outcome = capture.run(&sink, &stop2);
                capturing.store(false, Ordering::SeqCst);
                sink.end_while(outcome, || !stop2.load(Ordering::SeqCst));
            },
        );
        match spawned {
            Ok(handle) => {
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                self.capturing.store(false, Ordering::SeqCst);
                Err(Error::Io(e))
            }
        }
    }

    /// Asks the thread to end; it sets the trigger off on its way out.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// One capture, on its own thread.
struct Capture {
    meter: Arc<Mutex<Option<Meter>>>,
    limit_samples: u64,
    limit_msec: u64,
    enabled: [bool; 3],
    bounds: Bounds,
    silences_told: Arc<std::sync::atomic::AtomicUsize>,
}

/// How a capture's loop ended.
enum Ended {
    /// A limit met, a stop, or nobody listening.
    Done,
    /// It failed.
    Failed(Error),
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            self.trigger_off();
            return Ok(());
        }
        match self.take(sink, stop) {
            Ended::Done => {
                self.trigger_off();
                Ok(())
            }
            Ended::Failed(e) => {
                // A stream that can't be read on, or a silent meter, on a link
                // still up: its sampling goes off, as C's stop does.
                // Not waited for: a broken stream would not show the echo.
                let mut guard = self.meter.lock().expect("meter lock");
                if let Some(m) = guard.as_mut().filter(|m| !m.link_failed)
                    && let Err(off) = m.trigger_off_unwaited()
                {
                    log::warn!("The meter's sampling may still be on: {off}");
                }
                Err(e)
            }
        }
    }

    /// `dev_acquisition_stop`'s trigger off, waited for; a failure is told and
    /// ends nothing.
    fn trigger_off(&self) {
        let mut guard = self.meter.lock().expect("meter lock");
        if let Some(m) = guard.as_mut()
            && let Err(e) = m.set_chooser("SAMPLING:TRIGGER", "SAMPLING:TRIGGER:OFF")
        {
            log::warn!("The meter's sampling may still be on: {e}");
        }
    }

    #[allow(clippy::too_many_lines)]
    fn take(&self, sink: &PacketSink, stop: &AtomicBool) -> Ended {
        let go_on = || !stop.load(Ordering::SeqCst);
        let started = Instant::now();
        let deadline =
            (self.limit_msec > 0).then(|| started + Duration::from_millis(self.limit_msec));
        let mut heartbeat = started + self.bounds.heartbeat;
        let mut samples = 0u64;
        let mut last_reading: Option<Instant> = None;
        let mut warned = false;
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ended::Done;
            }
            let now = Instant::now();
            if deadline.is_some_and(|d| now >= d) {
                if last_reading.is_none() {
                    log::warn!("No reading came from the meter in the time given.");
                }
                return Ended::Done;
            }
            let mut guard = self.meter.lock().expect("meter lock");
            let Some(m) = guard.as_mut() else {
                return Ended::Failed(Error::device("device is not open"));
            };
            if now >= heartbeat {
                log::trace!("Sending heartbeat request.");
                let pcb = match m.path("PCB_VERSION") {
                    Ok(p) => p,
                    Err(e) => return Ended::Failed(e),
                };
                if let Err(e) = m.poll(pcb) {
                    return Ended::Failed(e);
                }
                heartbeat += self.bounds.heartbeat;
            }
            let wait = deadline.map_or(POLL, |d| d.saturating_duration_since(now).min(POLL));
            let updates = match m.pump(wait) {
                Ok(u) => u,
                Err(e) => return Ended::Failed(e),
            };
            for node in updates {
                let Some((ch, data)) = m.reading(node) else { continue };
                if !self.enabled[ch] {
                    continue;
                }
                let count = data.len() as u64;
                let largest = data.samples.iter().fold(0f32, |a, v| a.max(v.abs()));
                let buffered = data.digits == 0;
                if !sink.send_while(Packet::Analog(data), go_on) {
                    return Ended::Done;
                }
                last_reading = Some(Instant::now());
                warned = false;
                if ch < 2 {
                    match m.autorange(ch, largest) {
                        Ok(true) if buffered => {
                            // The scale goes with the range.
                            let path = format!("CH{}:BUF_LSB2NATIVE", ch + 1);
                            if let Ok(n) = m.path(&path)
                                && let Err(e) = m.poll(n)
                            {
                                return Ended::Failed(e);
                            }
                        }
                        Ok(_) => {}
                        Err(e) => return Ended::Failed(e),
                    }
                }
                samples += count;
                if self.limit_samples > 0 && samples >= self.limit_samples {
                    return Ended::Done;
                }
            }
            drop(guard);
            let now = Instant::now();
            // Only a sample limit with no time limit fails on silence.
            let bounded = self.limit_samples > 0 && deadline.is_none();
            match last_reading {
                None if bounded && now.duration_since(started) > self.bounds.first_reading => {
                    return Ended::Failed(Error::device(format!(
                        "No reading from the meter in {} s.",
                        self.bounds.first_reading.as_secs()
                    )));
                }
                Some(at) if bounded && now.duration_since(at) > self.bounds.silence_limit => {
                    return Ended::Failed(Error::device(format!(
                        "No reading from the meter in {} s.",
                        self.bounds.silence_limit.as_secs()
                    )));
                }
                Some(at) if now.duration_since(at) > self.bounds.silence_warning && !warned => {
                    log::warn!(
                        "No reading from the meter for {} s; still waiting.",
                        self.bounds.silence_warning.as_secs()
                    );
                    warned = true;
                    self.silences_told.fetch_add(1, Ordering::SeqCst);
                }
                _ => {}
            }
        }
    }
}

impl Driver for MooshimeterDriver {
    fn id(&self) -> &'static str {
        "mooshimeter-dmm"
    }

    fn long_name(&self) -> &'static str {
        "Mooshimeter DMM"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    /// `scan`: the address, a connection and its close; whatever answers is a
    /// meter.
    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else { return Ok(Vec::new()) };
        // A bad address is logged where it is read, and finds nothing.
        let Ok(addr) = bt::parse_mac(&conn) else { return Ok(Vec::new()) };
        match bt::connect(addr, CONNECT_TIMEOUT) {
            Ok(bearer) => {
                log::debug!("BLE disconnect");
                drop(bearer);
            }
            Err(e) => {
                log::error!("BLE connect to {} failed: {e}", bt::mac_text(addr));
                return Ok(Vec::new());
            }
        }
        Ok(vec![Box::new(MooshimeterDevice::new(addr))])
    }
}
