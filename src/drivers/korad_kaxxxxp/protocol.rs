// Counterpart in sigrok:
//   libsigrok/src/hardware/korad-kaxxxxp/api.c,
//   libsigrok/src/hardware/korad-kaxxxxp/protocol.c,
//   libsigrok/src/hardware/korad-kaxxxxp/protocol.h,
//   Copyright (C) 2015 Hannu Vuolasaho <vuokkosetae@gmail.com>,
//   Copyright (C) 2018-2019 Frank Stettner <frank-stettner@gmx.net>,
//   with commits by Gerhard Sittig, Uwe Hermann, Lars-Peter Clausen,
//   Axel Hinrichs, Christian Fruth, David Sastre Medina, Dewelde Anthony,
//   Elen Eisendle, Holger Müller, JD Steffen, Peter van der Perk,
//   Peter Åstrand, Thomas Andres, Tim Van Helleputte, Walter Goossens,
//   Wolfram Sang, motte, Aurelien Jacobs, Georg Hofmann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the Korad `KAxxxxP` family, and its table of models.
//!
//! ASCII with no terminators, no echo and no checksum. A setting has no
//! answer, `VSET1?`, `VOUT1?`, `ISET1?` and `IOUT1?` give five characters
//! and `STATUS?` one byte, so a reply ends at its known length or at a
//! silence. `*IDN?` has no known length: it is read until a silence of
//! three characters' time. `ISET1?` may give a sixth byte, read and dropped.
//!
//! The device is slow: the next request waits 80 ms from the last, and on
//! the one model with `SLOW_PROCESSING` 450 ms more after a setting or a
//! `STATUS?`. A request that comes too early is said to be lost, and then
//! each reply answers another request. All of it is as in `protocol.c`.

use crate::core::{Error, Result};
use crate::serial::{Params, Port, read_blocking, serial_timeout};
use std::time::{Duration, Instant};

/// Min, max and step, as `api.c`'s tables.
pub type Range = [f64; 3];

const VOLTS_30: Range = [0.0, 31.0, 0.01];
const VOLTS_60: Range = [0.0, 61.0, 0.01];
const AMPS_3: Range = [0.0, 3.1, 0.001];
const AMPS_5: Range = [0.0, 5.1, 0.001];

/// What a model does differently. Plain flags, as `protocol.h`'s.
#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Quirks {
    /// Bit 7 of the status means something only with the output on.
    pub labps_ovp_en: bool,
    /// The ID starts at the model's name.
    pub id_no_vendor: bool,
    /// One byte of 0x01 or 0xbc may end the ID.
    pub id_trailing: bool,
    /// A version may follow the literal ID.
    pub id_opt_version: bool,
    /// 450 ms more after a setting and a `STATUS?`.
    pub slow_processing: bool,
}

const NONE: Quirks = Quirks {
    labps_ovp_en: false,
    id_no_vendor: false,
    id_trailing: false,
    id_opt_version: false,
    slow_processing: false,
};

/// One line of the table.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Model {
    /// The vendor shown.
    pub vendor: &'static str,
    /// The model's name shown.
    pub name: &'static str,
    /// An ID to match literally, or empty for vendor and name.
    pub id: &'static str,
    /// The voltage target's range.
    pub voltage: Range,
    /// The current limit's range.
    pub current: Range,
    /// Its quirks.
    pub quirks: Quirks,
}

const fn model(
    vendor: &'static str,
    name: &'static str,
    id: &'static str,
    voltage: Range,
    current: Range,
    quirks: Quirks,
) -> Model {
    Model { vendor, name, id, voltage, current, quirks }
}

/// `api.c`'s `models[]`, whose order decides a match.
pub const MODELS: [Model; 14] = [
    model("Korad", "KA3005P", "", VOLTS_30, AMPS_5, Quirks { id_trailing: true, ..NONE }),
    model("Korad", "KD3005P", "", VOLTS_30, AMPS_5, NONE),
    model("Korad", "KD6005P", "", VOLTS_60, AMPS_5, NONE),
    model(
        "RND",
        "KA3005P",
        "RND 320-KA3005P",
        VOLTS_30,
        AMPS_5,
        Quirks { id_opt_version: true, ..NONE },
    ),
    model(
        "RND",
        "KD3005P",
        "RND 320-KD3005P",
        VOLTS_30,
        AMPS_5,
        Quirks { id_opt_version: true, ..NONE },
    ),
    model(
        "Stamos Soldering",
        "S-LS-31",
        "",
        VOLTS_30,
        AMPS_5,
        Quirks { id_no_vendor: true, ..NONE },
    ),
    model("Tenma", "72-2535", "", VOLTS_30, AMPS_3, NONE),
    model("Tenma", "72-2540", "", VOLTS_30, AMPS_5, NONE),
    model("Tenma", "72-2550", "", VOLTS_60, AMPS_3, NONE),
    model("Tenma", "72-2705", "", VOLTS_30, AMPS_3, NONE),
    model("Tenma", "72-2710", "", VOLTS_30, AMPS_5, NONE),
    model("Velleman", "LABPS3005D", "", VOLTS_30, AMPS_5, Quirks { labps_ovp_en: true, ..NONE }),
    model(
        "Velleman",
        "PS3005D V1.3",
        "VELLEMANPS3005DV1.3",
        VOLTS_30,
        AMPS_5,
        Quirks { id_trailing: true, slow_processing: true, ..NONE },
    ),
    model("Velleman", "PS3005D", "", VOLTS_30, AMPS_5, NONE),
];

/// C's `isspace` in the C locale.
fn is_space(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

fn skip_spaces(text: &[u8]) -> &[u8] {
    let n = text.iter().take_while(|c| is_space(**c)).count();
    &text[n..]
}

/// A `V` and its digits and points, and the spaces after them.
fn skip_version(text: &[u8]) -> &[u8] {
    let Some(rest) = text.strip_prefix(b"V") else { return text };
    let n = rest.iter().take_while(|c| **c == b'.' || c.is_ascii_digit()).count();
    skip_spaces(&rest[n..])
}

/// `text` after `want`, in either case.
fn after_ignoring_case<'a>(text: &'a [u8], want: &str) -> Option<&'a [u8]> {
    let n = want.len();
    (text.len() >= n && text[..n].eq_ignore_ascii_case(want.as_bytes())).then(|| &text[n..])
}

/// `model_matches`: whether an `*IDN?` reply, its serial number cut off,
/// is this model's.
fn matches(model: &Model, id: &[u8]) -> bool {
    if !model.id.is_empty() {
        if !model.quirks.id_opt_version {
            return id == model.id.as_bytes();
        }
        let Some(rest) = id.strip_prefix(model.id.as_bytes()) else { return false };
        return skip_version(skip_spaces(rest)).is_empty();
    }
    let mut text = id;
    if !model.quirks.id_no_vendor {
        let Some(rest) = after_ignoring_case(text, model.vendor) else { return false };
        text = skip_spaces(rest);
    }
    let Some(rest) = after_ignoring_case(text, model.name) else { return false };
    text = skip_version(skip_spaces(rest));
    if model.quirks.id_trailing && matches!(text.first(), Some(0x01 | 0xbc)) {
        text = &text[1..];
    }
    text.is_empty()
}

/// `model_lookup`: the first model of the table the reply is.
pub fn lookup(id: &[u8]) -> Option<&'static Model> {
    if id.is_empty() {
        return None;
    }
    let found = MODELS.iter().find(|m| matches(m, id));
    if let Some(m) = found {
        log::debug!("Found: [{}] [{}]", m.vendor, m.name);
    }
    found
}

/// A reply as C reads it: a string, up to its first NUL.
pub fn c_string(reply: &[u8]) -> &[u8] {
    let n = reply.iter().position(|c| *c == 0).unwrap_or(reply.len());
    &reply[..n]
}

/// The ID and the serial number after the last ` SN:`, if any.
pub fn split_serial(reply: &[u8]) -> (&[u8], Option<&[u8]>) {
    const PREFIX: &[u8] = b" SN:";
    match reply.windows(PREFIX.len()).rposition(|w| w == PREFIX) {
        Some(at) => (&reply[..at], Some(&reply[at + PREFIX.len()..])),
        None => (reply, None),
    }
}

/// `sr_atof_ascii`: a whole reply as a number, or `None` for anything else
/// (the caller keeps what it had). Not as `strtod`: hexadecimal, which falls
/// under "anything else"; and an empty reply, which `strtod` takes for 0.0.
/// A missing reply is the caller's to see, see [`Link::get_value`].
pub fn parse_reply(reply: &[u8]) -> Option<f32> {
    let text = c_string(reply);
    if text.is_empty() {
        return None;
    }
    let text = std::str::from_utf8(skip_spaces(text)).ok()?;
    let value: f64 = text.parse().ok()?;
    // ERANGE: strtod's overflow is an error there, a text that spells
    // infinity is not.
    if value.is_infinite() && text.as_bytes().iter().any(u8::is_ascii_digit) {
        return None;
    }
    Some(value as f32)
}

/// `VSET1:%05.2f` of what a C float keeps.
pub fn voltage_command(volts: f64) -> String {
    format!("VSET1:{:05.2}", f64::from(volts as f32))
}

/// `ISET1:%05.3f` of what a C float keeps.
pub fn current_command(amps: f64) -> String {
    format!("ISET1:{:05.3}", f64::from(amps as f32))
}

/// What a query asks for, in `protocol.h`'s order, which `get_all_values`
/// walks up to [`Target::Status`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    /// `IOUT1?`.
    Current,
    /// `ISET1?`.
    CurrentLimit,
    /// `VOUT1?`.
    Voltage,
    /// `VSET1?`.
    VoltageTarget,
    /// `STATUS?`, for the regulation or in a capture: the long kind of
    /// request on the slow model.
    Status,
    /// `STATUS?`, for `enabled`, `ocp_enabled` or `ovp_enabled`: `config_get`
    /// passes `KAXXXXP_OUTPUT` and the like, and `next_req_time` calls only
    /// `KAXXXXP_STATUS` long.
    StatusShort,
}

/// What a setting changes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Setting {
    /// `VSET1:`.
    VoltageTarget(f64),
    /// `ISET1:`.
    CurrentLimit(f64),
    /// `OUT`.
    Output(bool),
    /// `OCP`.
    Ocp(bool),
    /// `OVP`.
    Ovp(bool),
}

const PROCESSING: Duration = Duration::from_millis(80);
const EXTRA_PROCESSING: Duration = Duration::from_millis(450);
/// How long the sixth byte of `ISET1?` is waited for.
const SIXTH_BYTE: Duration = Duration::from_millis(10);
/// Attempts at a first byte, each of `serial_timeout` of the whole reply.
const RETRIES_FIRST: u32 = 100;
/// The buffer the reply to `*IDN?` goes into.
pub const ID_BUFFER: usize = 48;

/// The port and what the driver knows of the device: `dev_context`.
#[allow(clippy::struct_excessive_bools)]
pub struct Link {
    /// The port, while open.
    pub port: Option<Box<dyn Port>>,
    /// Its parameters, which the timeouts follow.
    pub params: Params,
    /// The model.
    pub model: &'static Model,
    next_request: Option<Instant>,
    /// Amperes last read.
    pub current: f32,
    /// The limit last read.
    pub current_limit: f32,
    /// Volts last read.
    pub voltage: f32,
    /// The target last read.
    pub voltage_target: f32,
    /// Constant current, per channel.
    pub cc_mode: [bool; 2],
    // As the C's gboolean: the status's bit where the device said so, 1
    // where a setting did. So the first status after a setting always
    // counts as a change (see `api.c`'s META after `-c enabled=on`), though
    // the setting's comment means the opposite. Nonzero is on.
    output_enabled: u8,
    ocp_enabled: u8,
    ovp_enabled: u8,
    /// A status changed what it says of channel 1's regulation.
    pub cc_mode_1_changed: bool,
    /// The same of channel 2.
    pub cc_mode_2_changed: bool,
    /// A status changed the output.
    pub output_enabled_changed: bool,
    /// A status changed OCP.
    pub ocp_enabled_changed: bool,
    /// A status changed OVP.
    pub ovp_enabled_changed: bool,
    /// What the capture asks next, kept over captures as in C.
    pub acquisition_target: Target,
}

impl Link {
    /// A link on `port`, with nothing read yet.
    pub fn new(port: Option<Box<dyn Port>>, params: Params, model: &'static Model) -> Link {
        Link {
            port,
            params,
            model,
            next_request: None,
            current: 0.0,
            current_limit: 0.0,
            voltage: 0.0,
            voltage_target: 0.0,
            cc_mode: [false; 2],
            output_enabled: 0,
            ocp_enabled: 0,
            ovp_enabled: 0,
            cc_mode_1_changed: false,
            cc_mode_2_changed: false,
            output_enabled_changed: false,
            ocp_enabled_changed: false,
            ovp_enabled_changed: false,
            acquisition_target: Target::Current,
        }
    }

    /// Whether the output is on, as last known.
    pub fn output_enabled(&self) -> bool {
        self.output_enabled != 0
    }

    /// Whether OCP is on, as last known.
    pub fn ocp_enabled(&self) -> bool {
        self.ocp_enabled != 0
    }

    /// Whether OVP is on, as last known.
    pub fn ovp_enabled(&self) -> bool {
        self.ovp_enabled != 0
    }

    /// `dev_acquisition_start`'s `next_req_time = 0`: the capture's first
    /// request does not wait for the last one before it.
    pub fn forget_pace(&mut self) {
        self.next_request = None;
    }

    fn give_device_time(&self) {
        if let Some(at) = self.next_request {
            let now = Instant::now();
            if at > now {
                std::thread::sleep(at - now);
            }
        }
    }

    fn pace(&mut self, long_command: bool) {
        let mut wait = PROCESSING;
        if self.model.quirks.slow_processing && long_command {
            wait += EXTRA_PROCESSING;
        }
        self.next_request = Some(Instant::now() + wait);
    }

    fn port(&mut self) -> Result<&mut Box<dyn Port>> {
        self.port.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `korad_kaxxxxp_send_cmd`.
    fn send(&mut self, command: &str) -> Result<()> {
        log::debug!("Sending '{command}'.");
        self.port()?.write(command.as_bytes()).inspect_err(|e| {
            log::error!("Error sending command: {e}.");
        })
    }

    fn read_blocking(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        read_blocking(self.port()?.as_mut(), buf, timeout)
    }

    /// `korad_kaxxxxp_read_chars`: up to `count` bytes. The first may take
    /// a hundred times the line's time for all of them; after it, a pause
    /// of three bytes' time ends the reply. A reply may stay empty.
    ///
    /// The attempts are also bound by the time they should take: where a
    /// system's sleeps run long, a macOS runner's by six times, C's count
    /// alone would stretch 1.5 s of silence to 10.
    pub fn read_chars(&mut self, count: usize) -> Result<Vec<u8>> {
        let mut buf = vec![0; count];
        let width = u32::try_from(count).unwrap_or(u32::MAX);
        let first = serial_timeout(&self.params, width);
        let later = serial_timeout(&self.params, 3);
        let mut received = 0;
        let mut timeout = first;
        let mut retries = RETRIES_FIRST;
        let first_by = Instant::now() + first * RETRIES_FIRST;
        while received < count && retries > 0 {
            retries -= 1;
            let n = self.read_blocking(&mut buf[received..], timeout).inspect_err(|e| {
                log::error!("Error reading {count} bytes from device: {e}.");
            })?;
            if n == 0 {
                if received > 0 {
                    break;
                }
                if Instant::now() >= first_by {
                    break;
                }
                continue;
            }
            received += n;
            timeout = later;
            retries = 1;
        }
        buf.truncate(received);
        log::debug!("got {received} bytes, received: '{}'.", String::from_utf8_lossy(&buf));
        Ok(buf)
    }

    /// `korad_kaxxxxp_send_cmd` with `*IDN?`, which sets nothing's pace.
    pub fn ask_id(&mut self) -> Result<Vec<u8>> {
        self.send("*IDN?")?;
        self.read_chars(ID_BUFFER)
    }

    /// `korad_kaxxxxp_get_value`: whether the device answered.
    ///
    /// A missing reading leaves the value as it was and gives `false`. C reads
    /// it as 0 (`read_chars` counts no reply as no error, and
    /// `sr_atof_ascii("")` is 0.0), which tells a script waiting for 0 V that
    /// it has them. A reply that makes no number leaves the value too, and a
    /// missing status reads as 0 as in C, since firmware is reported that
    /// does not answer `STATUS?` with the output off.
    ///
    /// Bytes still on their way from an earlier request are dropped first,
    /// which C does not do: without it one late reply, or a sixth byte that
    /// missed its 10 ms, shifts every reading after it. This puts nothing on
    /// the wire.
    pub fn get_value(&mut self, target: Target) -> Result<bool> {
        self.give_device_time();
        self.port()?.purge()?;
        let (command, count) = match target {
            Target::Current => ("IOUT1?", 5),
            Target::CurrentLimit => ("ISET1?", 5),
            Target::Voltage => ("VOUT1?", 5),
            Target::VoltageTarget => ("VSET1?", 5),
            Target::Status | Target::StatusShort => ("STATUS?", 1),
        };
        self.send(command)?;
        self.pace(target == Target::Status);
        let reply = self.read_chars(count)?;
        let answered = !c_string(&reply).is_empty();
        if !answered {
            log::warn!("No answer to {command}.");
        }
        let slot = match target {
            Target::Current => Some(&mut self.current),
            Target::CurrentLimit => Some(&mut self.current_limit),
            Target::Voltage => Some(&mut self.voltage),
            Target::VoltageTarget => Some(&mut self.voltage_target),
            Target::Status | Target::StatusShort => None,
        };
        if let Some(slot) = slot {
            if let Some(v) = parse_reply(&reply) {
                *slot = v;
                log::debug!("value: {v}");
            }
        } else {
            self.take_status(reply.first().copied().unwrap_or(0));
        }
        if target == Target::CurrentLimit {
            // The sixth byte, on the firmware that sends one. C ignores
            // how this went.
            let mut sixth = [0];
            let _ = self.read_blocking(&mut sixth, SIXTH_BYTE);
        }
        Ok(answered)
    }

    fn take_status(&mut self, status: u8) {
        log::debug!("Status: {status:#04x}");
        let cc = [status & 1 == 0, status & 2 == 0];
        self.cc_mode_1_changed = cc[0] != self.cc_mode[0];
        self.cc_mode_2_changed = cc[1] != self.cc_mode[1];
        self.cc_mode = cc;
        let ocp = status & 0x20;
        self.ocp_enabled_changed = ocp != self.ocp_enabled;
        self.ocp_enabled = ocp;
        let output = status & 0x40;
        self.output_enabled_changed = output != self.output_enabled;
        self.output_enabled = output;
        if !self.model.quirks.labps_ovp_en || output != 0 {
            let ovp = status & 0x80;
            self.ovp_enabled_changed = ovp != self.ovp_enabled;
            self.ovp_enabled = ovp;
        }
    }

    /// `korad_kaxxxxp_get_all_values`, what a scan reads, in the C's order. A
    /// missing reply does not fail a scan, as in C: each read asks again.
    pub fn get_all_values(&mut self) -> Result<()> {
        for target in [
            Target::Current,
            Target::CurrentLimit,
            Target::Voltage,
            Target::VoltageTarget,
            Target::Status,
        ] {
            self.get_value(target)?;
        }
        Ok(())
    }

    /// `korad_kaxxxxp_set_value`.
    pub fn set_value(&mut self, setting: Setting) -> Result<()> {
        self.give_device_time();
        let flag = |name: &str, on: bool| format!("{name}{}", u8::from(on));
        let command = match setting {
            Setting::VoltageTarget(v) => voltage_command(v),
            Setting::CurrentLimit(v) => current_command(v),
            Setting::Output(on) => {
                self.output_enabled = u8::from(on);
                flag("OUT", on)
            }
            Setting::Ocp(on) => {
                self.ocp_enabled = u8::from(on);
                flag("OCP", on)
            }
            Setting::Ovp(on) => {
                self.ovp_enabled = u8::from(on);
                flag("OVP", on)
            }
        };
        let sent = self.send(&command);
        self.pace(true);
        sent
    }

    /// `next_measurement`.
    pub fn next_measurement(&mut self) {
        self.acquisition_target = match self.acquisition_target {
            Target::Current => Target::Voltage,
            Target::Voltage => Target::Status,
            _ => Target::Current,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn found(id: &[u8]) -> Option<(&'static str, &'static str)> {
        lookup(id).map(|m| (m.vendor, m.name))
    }

    #[test]
    fn the_ids_of_the_old_table_find_their_models() {
        // api.c's literal IDs up to libsigrok d241cc2.
        for (id, vendor, name) in [
            (&b"KORADKA3005PV2.0"[..], "Korad", "KA3005P"),
            (b"KORADKA3005PV2.0\x01", "Korad", "KA3005P"),
            (b"KORADKA3005PV2.0\xbc", "Korad", "KA3005P"),
            (b"KORAD KA3005P V4.2", "Korad", "KA3005P"),
            (b"KORAD KD3005P V2.0", "Korad", "KD3005P"),
            (b"KORADKD3005PV2.1", "Korad", "KD3005P"),
            (b"KORAD KD6005P V2.2", "Korad", "KD6005P"),
            (b"RND 320-KA3005P V5.5", "RND", "KA3005P"),
            (b"RND 320-KD3005P V4.2", "RND", "KD3005P"),
            (b"S-LS-31 V2.0", "Stamos Soldering", "S-LS-31"),
            (b"TENMA 72-2535 V2.1", "Tenma", "72-2535"),
            (b"TENMA72-2540V2.0", "Tenma", "72-2540"),
            (b"TENMA72-2550V2.0", "Tenma", "72-2550"),
            (b"TENMA 72-2710 V6.6", "Tenma", "72-2710"),
            (b"VELLEMANLABPS3005DV2.0", "Velleman", "LABPS3005D"),
            (b"VELLEMANPS3005DV2.0", "Velleman", "PS3005D"),
            (b"VELLEMANPS3005DV1.3", "Velleman", "PS3005D V1.3"),
        ] {
            assert_eq!(found(id), Some((vendor, name)), "{}", id.escape_ascii());
        }
    }

    #[test]
    fn the_check_stretches_as_far_as_the_c() {
        assert_eq!(found(b"korad ka3005p v2.0"), None, "lower case v is no version");
        assert_eq!(found(b"korad ka3005p V2.0"), Some(("Korad", "KA3005P")));
        assert_eq!(found(b"KORAD \t KA3005P"), Some(("Korad", "KA3005P")));
        assert_eq!(found(b"KORADKA3005P\x0bV2\x0c"), Some(("Korad", "KA3005P")));
        assert_eq!(found(b"RND 320-KA3005P V5.5 "), Some(("RND", "KA3005P")));
        for no in [
            &b""[..],
            b"KORAD",
            b"KORADKA3005P2.0",
            b"KORADKD3005PV2.0\x01",
            b"KORADKA3005PV2.0\x01\x01",
            b"RND 320-KA3005P V5.5x",
            b"rnd 320-ka3005p",
            b"VELLEMANPS3005DV1.3\x01x",
            b"ACME",
        ] {
            assert_eq!(found(no), None, "{}", no.escape_ascii());
        }
        // The literal ignores its trailing quirk, and the generic line
        // after it has none.
        assert_eq!(found(b"VELLEMANPS3005DV1.3\x01"), None);
    }

    #[test]
    fn each_range_is_the_tables() {
        let m = |id: &[u8]| lookup(id).unwrap();
        assert_eq!(m(b"KORAD KD6005P").voltage[1], 61.0);
        assert_eq!(m(b"KORAD KD6005P").current[1], 5.1);
        assert_eq!(m(b"TENMA 72-2535").voltage[1], 31.0);
        assert_eq!(m(b"TENMA 72-2535").current[1], 3.1);
        assert_eq!(m(b"TENMA 72-2550").voltage[1], 61.0);
        assert_eq!(m(b"TENMA 72-2550").current[1], 3.1);
        assert_eq!(m(b"TENMA 72-2705").current[1], 3.1);
        assert!(m(b"VELLEMANPS3005DV1.3").quirks.slow_processing);
        assert!(m(b"VELLEMAN LABPS3005D").quirks.labps_ovp_en);
    }

    #[test]
    fn a_reply_is_a_string_and_its_last_sn_the_serial() {
        assert_eq!(c_string(b"abc\0def"), b"abc");
        assert_eq!(
            split_serial(b"KORAD KA3005P V5.8 SN:1234"),
            (&b"KORAD KA3005P V5.8"[..], Some(&b"1234"[..]))
        );
        assert_eq!(split_serial(b"X SN:1 SN:2"), (&b"X SN:1"[..], Some(&b"2"[..])));
        assert_eq!(split_serial(b"X SN"), (&b"X SN"[..], None));
    }

    #[test]
    fn replies_parse_as_sr_atof_ascii() {
        assert_eq!(parse_reply(b"12.34"), Some(12.34));
        assert_eq!(parse_reply(b"1."), Some(1.0));
        assert_eq!(parse_reply(b"1.5e1"), Some(15.0));
        assert_eq!(parse_reply(b" 1.5"), Some(1.5));
        assert_eq!(parse_reply(b"12\0.3"), Some(12.0));
        assert_eq!(parse_reply(b""), None, "silence is no reading");
        assert_eq!(parse_reply(b"\0\0"), None);
        assert_eq!(parse_reply(b"xx.xx"), None);
        assert_eq!(parse_reply(b"1.2 x"), None);
        assert_eq!(parse_reply(b"     "), None);
        assert_eq!(parse_reply(b"1e999"), None, "ERANGE");
        assert_eq!(parse_reply(b"inf"), Some(f32::INFINITY));
        assert!(parse_reply(b"nan").unwrap().is_nan());
        assert_eq!(parse_reply(b"\xff\xfe"), None);
    }

    #[test]
    fn commands_format_as_the_c_in_single_precision() {
        assert_eq!(voltage_command(5.0), "VSET1:05.00");
        assert_eq!(voltage_command(31.0), "VSET1:31.00");
        assert_eq!(voltage_command(60.0), "VSET1:60.00");
        assert_eq!(voltage_command(1.115), "VSET1:01.12", "1.115f is above, 1.115 below");
        assert_eq!(voltage_command(0.125), "VSET1:00.12", "an exact tie goes to even");
        assert_eq!(voltage_command(-0.0), "VSET1:-0.00");
        assert_eq!(current_command(2.225), "ISET1:2.225");
        assert_eq!(current_command(5.1), "ISET1:5.100");
        assert_eq!(current_command(-0.0), "ISET1:-0.000");
    }
}
