// Counterpart in sigrok:
//   libsigrok/src/hardware/testo/protocol.c,
//   libsigrok/src/hardware/testo/protocol.h,
//   libsigrok/src/hardware/testo/api.c (receive_data, receive_transfer),
//   Copyright (C) 2014 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Daniel Elstner, Martin Ling, Richard,
//   Lars-Peter Clausen, Soeren Apel, Romain Tartière
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the Testo 435, `protocol.c`: a 9 byte request over
//! an FT232R's UART, answered by one packet.
//!
//! ```text
//! request  12 00 00 00 01 01 55 d1 b7
//! answer   21 00 00 00 01 size N  (value:f32le unit unused exponent){N}  crc:u16le
//! ```
//!
//! The CRC is CRC-16/MCRF4XX from `0xffff` over all before it. Two sources
//! agree on this: the C, and Testo's own description, `hhtcde.chm` in its
//! `toolbox.exe` (pages `hhx35_e`, `hhx35r_e`, `hhx35ee_x`, `hhtcd7b_e`,
//! `HHTCD4_e`, `exto_e`), which names the fields. Nobody here reads `size`,
//! `unused` or the decimal exponent: `digits` stays 2 as in the C
//! (`protocol.c:249`). No capture of the device is known.
//!
//! The chip puts two status bytes in front of every 64 byte USB packet.
//! The C strips them once per transfer (`api.c:289-291`, `protocol.c:95`), so
//! a transfer that holds more than one packet keeps the later status bytes as
//! data and fails its CRC. That needs an answer of at least 65 bytes (8
//! measurements), and even then not every time: a chip whose latency timer
//! fires with 4 to 61 bytes buffered sends two short packets, two transfers.
//! [`strip_status`] strips them per packet.

use crate::core::{Mq, Packet, PacketSink, Result, Unit};
use crate::usb::{ControlRequest, Transport, libusb_error_name};
use std::fmt::Write as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// `MAX_REPLY_SIZE`: the transfer size and the longest answer taken.
pub const MAX_REPLY_SIZE: usize = 128;
/// `MAX_CHANNELS`.
pub const MAX_CHANNELS: usize = 16;
/// The FT232R's bulk packet size at full speed.
const PACKET_SIZE: usize = 64;
/// `EP_IN`.
const EP_IN: u8 = 0x81;
/// `EP_OUT`.
const EP_OUT: u8 = 0x02;
/// The request, `TESTO_x35_REQUEST`.
pub const REQUEST: [u8; 9] = [0x12, 0, 0, 0, 1, 1, 0x55, 0xd1, 0xb7];
/// The start of every well-formed answer.
const PREFIX: [u8; 5] = [0x21, 0, 0, 0, 1];

/// The 4 vendor requests' timeout. The C's 10 ms is no protocol: a
/// control transfer the chip answers at once only fails it on a loaded host.
const CONTROL_TIMEOUT: Duration = Duration::from_millis(100);
/// The timeout of the request's write and of the flush's reads. The C's
/// 10 ms is no protocol for the write, as for the requests above; for a flush
/// read it is shorter than the chip's default 16 ms latency timer.
const SHORT_TIMEOUT: Duration = Duration::from_millis(100);
/// The probe's read timeout, `protocol.c:87`.
const PROBE_TIMEOUT: Duration = Duration::from_millis(250);
/// The capture's read timeout, `api.c:374`: how often a stop and a time
/// limit are looked at while the chip holds a read.
const READ_TIMEOUT: Duration = Duration::from_millis(100);

/// How long a device may take to answer: Testo's suggested timeout for the
/// 435/635/735 (`exto_e`). The C waits for ever (`protocol.c:85`): a chip with
/// nothing sends status bytes each latency timer, so its read never times
/// out. Time rather than reads alone, as that timer is not the driver's: it
/// sends none, and a udev rule can set 1 ms.
pub const ANSWER_TIMEOUT: Duration = Duration::from_secs(7);
/// The reads a device has at least, whatever the clock says.
const MIN_READS: usize = 256;
/// Failed reads in a row that end a capture. These come back at once
/// whatever the latency timer, where the C resubmits for ever (`api.c:293`).
const MAX_ERRORS: usize = 256;
/// Reads of data the flush takes from a device that keeps sending. The C
/// flushes for ever (`protocol.c:76`).
const MAX_FLUSH: usize = 256;
/// Damaged answers in a row that end a capture. The C asks again for ever
/// after a bad CRC, and stops asking after a bad prefix or an overrun.
const MAX_BAD: usize = 16;

/// The wait for an answer: over when both `limit` and [`MIN_READS`] are spent.
struct Patience {
    limit: Duration,
    start: Instant,
    reads: usize,
}

impl Patience {
    fn new(limit: Duration) -> Patience {
        Patience { limit, start: Instant::now(), reads: 0 }
    }

    /// A read with no answer in it: whether the wait is over.
    fn spent(&mut self) -> bool {
        self.reads = self.reads.saturating_add(1);
        self.reads >= MIN_READS && self.start.elapsed() >= self.limit
    }

    /// Something came: the wait starts over.
    fn restart(&mut self) {
        self.start = Instant::now();
        self.reads = 0;
    }
}

/// CRC-16/MCRF4XX, `crc16_mcrf4xx()` from `0xffff`.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xffff_u16;
    for &b in data {
        crc ^= u16::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0x8408 } else { crc >> 1 };
        }
    }
    crc
}

/// The payload of a transfer: every packet without its two status bytes.
pub fn strip_status(raw: &[u8]) -> Vec<u8> {
    raw.chunks(PACKET_SIZE).flat_map(|p| p.get(2..).unwrap_or_default()).copied().collect()
}

/// `testo_check_packet_prefix()`.
pub fn prefix_ok(buf: &[u8]) -> bool {
    let ok = buf.starts_with(&PREFIX);
    if !ok {
        log::debug!("Packet has invalid prefix.");
    }
    ok
}

/// The length of a whole answer of `count` measurements.
pub fn packet_size(count: u8) -> usize {
    7 + usize::from(count) * 7 + 2
}

/// What a unit byte measures, or `None` for a unit the driver has no name for.
pub fn unit_info(unit: u8) -> Option<(&'static str, Mq, Unit)> {
    match unit {
        1 => Some(("Temperature", Mq::Temperature, Unit::Celsius)),
        3 => Some(("Humidity", Mq::RelativeHumidity, Unit::Humidity293K)),
        5 => Some(("Windspeed", Mq::WindSpeed, Unit::MeterSecond)),
        24 => Some(("Pressure", Mq::Pressure, Unit::Hectopascal)),
        _ => None,
    }
}

/// One measurement of an answer, matched with its channel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// The position of the channel.
    pub channel: usize,
    /// The value, in the unit.
    pub value: f32,
    /// What it measures.
    pub mq: Mq,
    /// Its unit.
    pub unit: Unit,
}

/// Why [`decode`] gave up on the rest of an answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Problem {
    /// A unit the driver has no name for.
    Unsupported(u8),
    /// A unit no channel has.
    Hotswapped,
}

/// The measurements of a whole answer, as `testo_receive_packet()` sends them,
/// up to the first it cannot place.
///
/// The k-th measurement of a unit goes to the k-th channel of that unit. The
/// C matches the first channel of the unit and leaves the result in the
/// index of the loop over the measurements (`protocol.c:255`, `281`), so that
/// two measurements of one unit loop for ever and a change of order skips
/// measurements.
pub fn decode(reply: &[u8], channel_units: &[u8]) -> (Vec<Reading>, Option<Problem>) {
    let mut readings = Vec::new();
    let mut seen = [0_usize; 256];
    for m in reply[7..].as_chunks::<7>().0.iter().take(usize::from(reply[6])) {
        let unit = m[4];
        let Some((_, mq, u)) = unit_info(unit) else {
            return (readings, Some(Problem::Unsupported(unit)));
        };
        let nth = seen[usize::from(unit)];
        seen[usize::from(unit)] += 1;
        let Some(channel) =
            channel_units.iter().enumerate().filter(|(_, c)| **c == unit).nth(nth).map(|(i, _)| i)
        else {
            return (readings, Some(Problem::Hotswapped));
        };
        let value = f32::from_le_bytes([m[0], m[1], m[2], m[3]]);
        readings.push(Reading { channel, value, mq, unit: u });
    }
    (readings, None)
}

/// `testo_set_serial_params()`: 115200 8N1, DTR and RTS high. The values are
/// the C's, the odd `wValue` of the flow control too (`protocol.h:41`).
///
/// # Errors
///
/// The first request that fails.
pub fn set_serial_params(t: &mut dyn Transport) -> Result<()> {
    const STEPS: [(u8, u16, &str); 4] = [
        (0x03, 0x001a, "Failed to set baudrate"),
        (0x04, 0x0008, "Failed to set comm parameters"),
        (0x02, 0x0008, "Failed to set flow control"),
        (0x01, 0x0303, "Failed to set modem control"),
    ];
    for (request, value, what) in STEPS {
        let req = ControlRequest::vendor(request, value, 0);
        if let Err(e) = t.control_out(req, &[], CONTROL_TIMEOUT) {
            log::error!("{what}: {}", libusb_error_name(&e));
            return Err(e);
        }
    }
    Ok(())
}

/// `testo_probe_channels()` on an open device: the units the device measures.
///
/// # Errors
///
/// When the device does not give a whole answer of no more than
/// [`MAX_CHANNELS`] known units.
pub fn probe_channels(t: &mut dyn Transport) -> Result<Vec<u8>> {
    log::debug!("Probing for channels.");
    set_serial_params(t)?;
    // Flush anything buffered from a previous run. Its errors do not count.
    for _ in 0..MAX_FLUSH {
        match t.bulk_in(EP_IN, MAX_REPLY_SIZE, SHORT_TIMEOUT) {
            Ok(raw) if raw.len() > 2 => {}
            _ => break,
        }
    }
    t.bulk_out(EP_OUT, &REQUEST, SHORT_TIMEOUT)?;
    let packet = read_answer(t, ANSWER_TIMEOUT)?;
    let count = usize::from(packet[6]);
    if count > MAX_CHANNELS {
        log::error!("Device says it has {count} channels!");
        return Err(crate::core::Error::device("too many channels"));
    }
    let units: Vec<u8> = (0..count).map(|i| packet[7 + i * 7 + 4]).collect();
    if let Some(u) = units.iter().find(|u| unit_info(**u).is_none()) {
        log::debug!("Unsupported measurement unit {u}");
        return Err(crate::core::Error::device("unsupported measurement unit"));
    }
    log::debug!("Found {count} channel{}.", if count > 1 { "s" } else { "" });
    Ok(units)
}

/// The probe's reading loop: a whole answer, for `limit` from the request.
fn read_answer(t: &mut dyn Transport, limit: Duration) -> Result<Vec<u8>> {
    let mut packet = Vec::with_capacity(MAX_REPLY_SIZE);
    let mut patience = Patience::new(limit);
    while !patience.spent() {
        let raw = t.bulk_in(EP_IN, MAX_REPLY_SIZE, PROBE_TIMEOUT)?;
        let payload = strip_status(&raw);
        if payload.is_empty() {
            continue;
        }
        if packet.len() + payload.len() > MAX_REPLY_SIZE {
            return Err(crate::core::Error::device("answer too long"));
        }
        packet.extend_from_slice(&payload);
        if packet.len() < 5 {
            continue;
        }
        if !prefix_ok(&packet) {
            // The tail end of some previous data.
            packet.clear();
            continue;
        }
        if packet.len() >= 7 && packet.len() >= packet_size(packet[6]) {
            return Ok(packet);
        }
    }
    log::debug!("No answer to the request.");
    Err(crate::core::Error::Timeout)
}

/// What the capture stops at.
#[derive(Clone, Copy, Debug, Default)]
pub struct Limits {
    /// Answers, 0 for none.
    pub samples: u64,
    /// Milliseconds, 0 for none.
    pub msec: u64,
}

/// A capture: from `dev_acquisition_start()` on.
struct Capture<'a> {
    t: &'a mut dyn Transport,
    units: &'a [u8],
    sink: &'a PacketSink,
    limits: Limits,
    reply: Vec<u8>,
    samples: u64,
    bad: usize,
    start: Instant,
}

impl Capture<'_> {
    /// `testo_request_packet()`.
    fn request(&mut self) -> Result<()> {
        match self.t.bulk_out(EP_OUT, &REQUEST, READ_TIMEOUT) {
            Ok(_) => {
                log::debug!("Requested new packet.");
                Ok(())
            }
            Err(e) => {
                if !matches!(e, crate::core::Error::Disconnected) {
                    log::error!("Failed to request packet: {}.", libusb_error_name(&e));
                }
                Err(e)
            }
        }
    }

    /// `sr_sw_limits_check()`.
    fn limit_reached(&self) -> bool {
        (self.limits.samples > 0 && self.samples >= self.limits.samples)
            || (self.limits.msec > 0
                && self.start.elapsed() >= Duration::from_millis(self.limits.msec))
    }

    /// `receive_data()`: `Ok(true)` when the capture is over.
    fn receive(&mut self, data: &[u8]) -> Result<bool> {
        if self.reply.len() + data.len() > MAX_REPLY_SIZE {
            log::debug!("Receive buffer overrun.");
            self.reply.clear();
            return self.damaged();
        }
        self.reply.extend_from_slice(data);
        if self.reply.len() < 7 || self.reply.len() < packet_size(self.reply[6]) {
            return Ok(false);
        }
        if !prefix_ok(&self.reply) {
            self.reply.clear();
            return self.damaged();
        }
        let n = self.reply.len();
        if crc16(&self.reply[..n - 2]).to_le_bytes() != self.reply[n - 2..] {
            log::debug!("Packet has invalid CRC.");
            self.reply.clear();
            return self.damaged();
        }
        self.bad = 0;
        if !self.send_packet() {
            return Ok(true);
        }
        self.samples += 1;
        self.reply.clear();
        if self.limit_reached() {
            return Ok(true);
        }
        self.request()?;
        Ok(false)
    }

    /// An answer that did not pass: asks again. The C does that after a bad
    /// CRC only; after a bad prefix or an overrun (`api.c:237-242`, `254-255`)
    /// it asks no more and waits for ever.
    fn damaged(&mut self) -> Result<bool> {
        self.bad += 1;
        if self.bad >= MAX_BAD {
            log::error!("Too many damaged answers.");
            return Ok(true);
        }
        if self.limit_reached() {
            return Ok(true);
        }
        self.request()?;
        Ok(false)
    }

    /// `testo_receive_packet()`: false when nobody listens any more.
    fn send_packet(&self) -> bool {
        log::debug!("Got {}-byte packet.", self.reply.len());
        if log::log_enabled!(log::Level::Trace) {
            let mut line = String::from("Packet:");
            for b in &self.reply {
                let _ = write!(line, " {b:02x}");
            }
            log::trace!("{line}");
        }
        let (readings, problem) = decode(&self.reply, self.units);
        for r in readings {
            let data = crate::core::AnalogData {
                channels: vec![r.channel],
                samples: vec![r.value],
                mq: r.mq,
                flags: crate::core::MqFlags::empty(),
                unit: r.unit,
                digits: 2,
            };
            if !self.sink.send(Packet::Analog(data)) {
                return false;
            }
        }
        match problem {
            Some(Problem::Unsupported(u)) => log::debug!("Unsupported measurement unit {u}."),
            Some(Problem::Hotswapped) => log::error!("Some channel hotswapped in!"),
            None => {}
        }
        true
    }

    /// The transfers: a request, and reads until the end.
    fn run(&mut self, stop: &AtomicBool) -> Result<()> {
        self.request()?;
        let mut patience = Patience::new(ANSWER_TIMEOUT);
        let mut errors = 0;
        while !stop.load(Ordering::SeqCst) && !self.limit_reached() {
            match self.t.bulk_in(EP_IN, MAX_REPLY_SIZE, READ_TIMEOUT) {
                Err(crate::core::Error::Disconnected) => return Ok(()),
                // A held read, for as long as an idle chip.
                Err(crate::core::Error::Timeout) => {
                    if patience.spent() {
                        log::error!("The device stopped answering.");
                        return Ok(());
                    }
                }
                // An error, "which is fine" (api.c:293), until it is not.
                Err(e) => {
                    log::trace!("Read failed: {e}");
                    errors += 1;
                    if errors >= MAX_ERRORS {
                        log::error!("The device stopped answering.");
                        return Ok(());
                    }
                }
                Ok(raw) => {
                    errors = 0;
                    let payload = strip_status(&raw);
                    if payload.is_empty() {
                        if patience.spent() {
                            log::error!("The device stopped answering.");
                            return Ok(());
                        }
                    } else {
                        patience.restart();
                        if self.receive(&payload)? {
                            return Ok(());
                        }
                    }
                }
            }
        }
        Ok(())
    }
}

/// The acquisition on its own thread. The header goes first, the end last.
pub fn run(
    t: &mut dyn Transport,
    units: &[u8],
    limits: Limits,
    sink: &PacketSink,
    stop: &AtomicBool,
) {
    sink.send(Packet::header());
    let mut capture = Capture {
        t,
        units,
        sink,
        limits,
        reply: Vec::with_capacity(MAX_REPLY_SIZE),
        samples: 0,
        bad: 0,
        start: Instant::now(),
    };
    if let Err(e) = capture.run(stop) {
        log::debug!("acquisition ended: {e}");
    }
    sink.send(Packet::End);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn answer(units: &[u8], values: &[f32]) -> Vec<u8> {
        let mut a = vec![0x21, 0, 0, 0, 1, 0, units.len() as u8];
        for (u, v) in units.iter().zip(values) {
            a.extend_from_slice(&v.to_le_bytes());
            a.extend_from_slice(&[*u, 0, 0]);
        }
        a.extend_from_slice(&crc16(&a).to_le_bytes());
        a
    }

    #[test]
    fn the_requests_crc_is_the_answers_crc() {
        assert_eq!(crc16(&REQUEST[..7]).to_le_bytes(), [0xd1, 0xb7]);
    }

    #[test]
    fn status_bytes_go_per_packet() {
        assert_eq!(strip_status(&[1, 0x60]), [0_u8; 0]);
        assert_eq!(strip_status(&[1, 0x60, 9]), [9]);
        let mut raw = vec![1, 0x60];
        raw.extend(0..62);
        raw.extend([1, 0x60, 100, 101]);
        let p = strip_status(&raw);
        assert_eq!(p.len(), 64);
        assert_eq!(&p[61..], &[61, 100, 101]);
        // No status bytes at all, which an FT232R never sends: nothing,
        // where the C copies len - 2 bytes (protocol.c:95).
        assert_eq!(strip_status(&[]), [0_u8; 0]);
        assert_eq!(strip_status(&[1]), [0_u8; 0]);
    }

    #[test]
    fn prefix_and_size() {
        assert!(prefix_ok(&[0x21, 0, 0, 0, 1, 7]));
        assert!(!prefix_ok(&[0x21, 0, 0, 0, 2]));
        assert!(!prefix_ok(&[0x21, 0]));
        assert_eq!(packet_size(0), 9);
        assert_eq!(packet_size(2), 23);
    }

    #[test]
    fn readings_go_to_their_channels() {
        let (r, p) = decode(&answer(&[1, 3], &[21.5, 45.0]), &[1, 3]);
        assert_eq!(p, None);
        assert_eq!(r.len(), 2);
        assert_eq!((r[0].channel, r[0].value, r[0].mq), (0, 21.5, Mq::Temperature));
        assert_eq!((r[1].channel, r[1].value, r[1].unit), (1, 45.0, Unit::Humidity293K));
    }

    #[test]
    fn two_of_a_unit_go_to_their_own_channels_and_end() {
        let (r, p) = decode(&answer(&[1, 1], &[1.0, 2.0]), &[1, 1]);
        assert_eq!(p, None);
        assert_eq!((r[0].channel, r[0].value), (0, 1.0));
        assert_eq!((r[1].channel, r[1].value), (1, 2.0));
    }

    #[test]
    fn another_order_skips_nothing() {
        let (r, _) = decode(&answer(&[3, 1, 5], &[1.0, 2.0, 3.0]), &[1, 3, 5]);
        assert_eq!(r.iter().map(|r| r.channel).collect::<Vec<_>>(), [1, 0, 2]);
    }

    #[test]
    fn what_cannot_be_placed_ends_the_answer() {
        let (r, p) = decode(&answer(&[1, 3], &[1.0, 2.0]), &[1]);
        assert_eq!((r.len(), p), (1, Some(Problem::Hotswapped)));
        let (r, p) = decode(&answer(&[1, 1], &[1.0, 2.0]), &[1]);
        assert_eq!((r.len(), p), (1, Some(Problem::Hotswapped)));
        let (r, p) = decode(&answer(&[7, 1], &[1.0, 2.0]), &[1]);
        assert_eq!((r.len(), p), (0, Some(Problem::Unsupported(7))));
    }

    const STATUS_ONLY: &str = "bulk_in 81 128 -> 0160\n";

    fn fixture(steps: &str) -> crate::usb::record::Replay {
        let text = format!(
            "# oxrok usb fixture 1\n\
             device vid=128d pid=0003 version=0600 bus=0 addr=0 strings=1,2,0\n{steps}"
        );
        crate::usb::record::Replay::from_text(&text).unwrap()
    }

    #[test]
    fn testos_timeout_is_kept() {
        assert_eq!(ANSWER_TIMEOUT, Duration::from_secs(7));
        assert_eq!(MIN_READS, 256);
    }

    /// `units=1 delay=300` on the emulator, at the real limit: 300 reads
    /// are 300 ms at a latency timer of 1 ms, 4.8 s at 16 ms.
    #[test]
    fn an_answer_after_more_reads_than_the_minimum_is_taken() {
        let mut a = String::new();
        for b in answer(&[1], &[1.0]) {
            write!(a, "{b:02x}").unwrap();
        }
        let steps = format!("{}bulk_in 81 128 -> 0160{a}\n", STATUS_ONLY.repeat(300));
        let mut t = fixture(&steps);
        assert_eq!(read_answer(&mut t, ANSWER_TIMEOUT).unwrap()[6], 1);
    }

    #[test]
    fn the_wait_takes_the_reads_even_when_the_time_is_spent() {
        let mut t = fixture(&STATUS_ONLY.repeat(MIN_READS + 5));
        assert!(matches!(read_answer(&mut t, Duration::ZERO), Err(crate::core::Error::Timeout)));
        // `spent()` ends the loop on the read after the last it took.
        assert_eq!(t.remaining(), 6);
    }

    #[test]
    fn the_wait_takes_the_time_even_when_the_reads_are_spent() {
        let mut p = Patience::new(Duration::from_secs(3600));
        for _ in 0..2 * MIN_READS {
            assert!(!p.spent());
        }
        p.limit = Duration::ZERO;
        assert!(p.spent());
        p.restart();
        assert!(!p.spent(), "the reads count again");
    }

    #[test]
    fn floats_are_little_endian_binary32() {
        let (r, _) = decode(&answer(&[1], &[f32::NEG_INFINITY]), &[1]);
        assert_eq!(r[0].value, f32::NEG_INFINITY);
        let mut a = answer(&[1], &[0.0]);
        a[7..11].copy_from_slice(&[0, 0, 0xc0, 0x7f]);
        assert!(decode(&a, &[1]).0[0].value.is_nan());
    }
}
