// Counterpart in sigrok:
//   libsigrok/src/hardware/ikalogic-scanalogic2/api.c (the open sequence, the capture command),
//   libsigrok/src/hardware/ikalogic-scanalogic2/protocol.c,
//   libsigrok/src/hardware/ikalogic-scanalogic2/protocol.h,
//   Copyright (C) 2013 Marc Schink <sigrok-dev@marcschink.de>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Soeren Apel, Martin Ling, Daniel Elstner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of the IKALOGIC Scanalogic-2: an `ATmega168PA` with
//! four 23K256 SRAMs behind a low speed HID interface, `20a0:4123`.
//!
//! Every command and answer is one 128 byte feature report on endpoint 0,
//! `SET_REPORT` and `GET_REPORT` with wValue `0x0300`, wIndex 0. Commands
//! ([`CMD_SAMPLE`], [`CMD_RESET`], [`CMD_IDLE`], [`CMD_INFO`]) are byte 0
//! of a report. The info answer is `0a`, the serial (u32, little endian)
//! and the firmware's major and minor. A status is `05 CODE`
//! ([`STATUS_DATA_READY`], `0x61` waiting for the trigger, `0x62` sampling,
//! [`STATUS_DEVICE_READY`]); until the device has a valid one, a read gives
//! the old contents of its buffer. A sample packet is `05 CHANNEL NUMBER
//! 00` and 124 sample bytes, most significant bit first, and they come
//! channel after channel.
//!
//! Sources: the C driver, and the sigrok wiki's `IKALOGIC_Scanalogic-2`
//! page by the driver's author. No firmware source was found. Nothing here
//! has run on a board; `emu/src/models/ikalogic-scanalogic2.rs` is the
//! device it was checked against, and says what that model assumes.
//!
//! `CMD_RESET` and `CMD_IDLE` carry whatever the buffer they are sent from
//! held: the info answer in the scan, the `CMD_SAMPLE` fields after a
//! capture. The wiki says those bytes are arbitrary; they stay as the C
//! driver sends them, so that the two tools' transactions compare equal.
//!
//! [`Machine`] is the C's pair of transfer callbacks, with one transfer at
//! a time; [`run`] drives it with blocking control transfers. A status
//! read that does not say "data ready" is followed by a wait of 1.5 s, the
//! C's `WAIT_DATA_READY_INTERVAL` ([`WAIT_DATA_READY`]): the device's pace
//! is not known to allow less, and a stop cuts the wait short.

use crate::core::{Error, Packet, PacketSink, Result, Trigger, TriggerMatch};
use crate::usb::{ControlRequest, Transport};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// `PACKET_LENGTH`: every report.
pub const PACKET_LENGTH: usize = 128;
/// `PACKET_NUM_SAMPLE_BYTES`.
pub const PACKET_SAMPLE_BYTES: usize = 124;
/// `NUM_CHANNELS`.
pub const NUM_CHANNELS: usize = 4;
/// `MAX_SAMPLES`: the 32765 sample bytes the driver asks for at most.
pub const MAX_SAMPLES: u64 = 32765 * 8;
/// `USB_INTERFACE`.
pub const INTERFACE: u8 = 0;
/// `WAIT_DATA_READY_INTERVAL`.
pub const WAIT_DATA_READY: Duration = Duration::from_millis(1500);

/// `USB_TIMEOUT_MS`.
const USB_TIMEOUT: Duration = Duration::from_secs(5);
/// How often a status wait looks at the stop flag.
const STOP_POLL: Duration = Duration::from_millis(10);
/// Status reads to wait for a ready device after a capture or a stop, where
/// the C waits for ever and nothing can stop it. The loop has no status
/// wait (protocol.c:342, 360, 367), so at low speed this is an estimated 1
/// to 4 s, which is no time the device is known to need: a stop of a long
/// capture at a low rate, on a device that does not recognize the reset by
/// then, leaves it un-idled until the next scan or open resets it. Sizing it
/// for a whole capture (210 s at 1.25 kHz) would mean many thousands of
/// rounds, with no event that could end them sooner.
const MAX_RESET_ROUNDS: u32 = 100;

/// `CMD_SAMPLE`.
pub const CMD_SAMPLE: u8 = 0x01;
/// `CMD_RESET`.
pub const CMD_RESET: u8 = 0x02;
/// `CMD_IDLE`.
pub const CMD_IDLE: u8 = 0x07;
/// `CMD_INFO`.
pub const CMD_INFO: u8 = 0x0a;

const STATUS_VALID: u8 = 0x05;
/// `STATUS_DATA_READY`.
pub const STATUS_DATA_READY: u8 = 0x60;
const STATUS_WAITING_FOR_TRIGGER: u8 = 0x61;
const STATUS_SAMPLING: u8 = 0x62;
/// `STATUS_DEVICE_READY`.
pub const STATUS_DEVICE_READY: u8 = 0x63;

const TRIGGER_CHANNEL_ALL: u8 = 0x00;
const TRIGGER_CHANNEL_0: u8 = 0x01;
const TRIGGER_TYPE_NEGEDGE: u8 = 0x00;
const TRIGGER_TYPE_POSEDGE: u8 = 0x01;
const TRIGGER_TYPE_ANYEDGE: u8 = 0x02;
const TRIGGER_TYPE_NONE: u8 = 0x03;

/// `sl2_samplerates[]`.
pub const SAMPLERATES: [u64; 11] = [
    1_250, 10_000, 50_000, 100_000, 250_000, 500_000, 1_000_000, 2_500_000, 5_000_000, 10_000_000,
    20_000_000,
];

/// The device's code of a samplerate of the table, which counts down.
pub fn samplerate_code(rate: u64) -> Option<u8> {
    let i = SAMPLERATES.iter().position(|r| *r == rate)?;
    u8::try_from(SAMPLERATES.len() - i - 1).ok()
}

/// `libusb_error_name(transfer->status)`, which the C's callbacks log.
fn transfer_status_name(e: &Error) -> &'static str {
    match e {
        Error::Timeout => "LIBUSB_TRANSFER_TIMED_OUT",
        Error::Disconnected => "LIBUSB_TRANSFER_NO_DEVICE",
        Error::Device(m) if m.contains("stalled") => "LIBUSB_TRANSFER_STALL",
        _ => "LIBUSB_TRANSFER_ERROR",
    }
}

/// `SET_REPORT`, feature (`USB_REQUEST_TYPE_OUT`).
fn set_report(t: &mut dyn Transport, data: &[u8; PACKET_LENGTH]) -> Result<()> {
    let req = ControlRequest { request_type: 0x21, request: 0x09, value: 0x0300, index: 0 };
    t.control_out(req, data, USB_TIMEOUT)
}

/// `GET_REPORT`, feature (`USB_REQUEST_TYPE_IN`).
fn get_report(t: &mut dyn Transport) -> Result<Vec<u8>> {
    let req = ControlRequest { request_type: 0x21, request: 0x01, value: 0x0300, index: 0 };
    t.control_in(req, PACKET_LENGTH, USB_TIMEOUT)
}

/// What the info report says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeviceInfo {
    /// The serial number.
    pub serial: u32,
    /// The firmware's major version.
    pub major: u8,
    /// The firmware's minor version.
    pub minor: u8,
}

/// The `libusb_kernel_driver_active`, `libusb_detach_kernel_driver` and
/// `libusb_claim_interface` steps of `sl2_get_device_info()` and
/// `dev_open()`. As in the C, usbhid stays detached until the device is
/// plugged in again.
fn claim(t: &mut dyn Transport) -> Result<()> {
    t.detach_kernel_driver(INTERFACE).inspect_err(|e| {
        log::error!("Failed to detach kernel driver: {}.", crate::usb::libusb_error_name(e));
    })?;
    t.claim_interface(INTERFACE).inspect_err(|e| {
        log::error!("Failed to claim interface: {}.", crate::usb::libusb_error_name(e));
    })
}

/// `sl2_get_device_info()` after the open. The transport is closed by its
/// owner.
///
/// Byte 0 of the answer only gets a warning: the wiki says `0a` marks a
/// valid report and the C does not look (protocol.c:674-678), but nothing
/// shows how a real device answers, so a device that answers something else
/// stays listed.
///
/// # Errors
///
/// A failed step, logged as the C logs it, except that a short answer says
/// so where the C names its length as a libusb error (protocol.c:666-668).
pub fn get_device_info(t: &mut dyn Transport) -> Result<DeviceInfo> {
    claim(t)?;
    match device_info_claimed(t) {
        Ok(info) => {
            t.release_interface(INTERFACE).inspect_err(|e| {
                log::error!("Failed to release interface: {}.", crate::usb::libusb_error_name(e));
            })?;
            Ok(info)
        }
        Err(e) => {
            if let Err(e) = t.release_interface(INTERFACE) {
                log::debug!("Releasing the interface: {e}.");
            }
            Err(e)
        }
    }
}

fn device_info_claimed(t: &mut dyn Transport) -> Result<DeviceInfo> {
    let name = crate::usb::libusb_error_name;
    let mut buffer = [0u8; PACKET_LENGTH];
    buffer[0] = CMD_RESET;
    set_report(t, &buffer).inspect_err(|e| {
        log::error!("Resetting of device failed: {}.", name(e));
    })?;
    buffer[0] = CMD_INFO;
    set_report(t, &buffer).inspect_err(|e| {
        log::error!("Requesting of device information failed: {}.", name(e));
    })?;
    let answer = get_report(t).inspect_err(|e| {
        log::error!("Receiving of device information failed: {}.", name(e));
    })?;
    if answer.len() != PACKET_LENGTH {
        let n = answer.len();
        log::error!("Receiving of device information failed: {n} bytes of {PACKET_LENGTH}.");
        return Err(Error::device("short answer"));
    }
    buffer.copy_from_slice(&answer);
    if buffer[0] != CMD_INFO {
        log::warn!(
            "The device information may be stale: its first byte is {:#04x}, not {CMD_INFO:#04x}.",
            buffer[0]
        );
    }
    let info = DeviceInfo {
        serial: u32::from_le_bytes([buffer[1], buffer[2], buffer[3], buffer[4]]),
        major: buffer[5],
        minor: buffer[6],
    };
    buffer[0] = CMD_RESET;
    set_report(t, &buffer).inspect_err(|e| {
        log::error!("Device reset failed: {}.", name(e));
    })?;
    buffer[0] = CMD_IDLE;
    set_report(t, &buffer).inspect_err(|e| {
        log::error!("Failed to set device in idle state: {}.", name(e));
    })?;
    Ok(info)
}

/// `dev_open()` after the open: claim, `CMD_RESET`, then `CMD_IDLE`, which
/// the C's comment says keeps the device from resetting itself.
///
/// # Errors
///
/// A failed step, logged as the C logs it.
pub fn open(t: &mut dyn Transport) -> Result<()> {
    let name = crate::usb::libusb_error_name;
    claim(t)?;
    let mut buffer = [0u8; PACKET_LENGTH];
    buffer[0] = CMD_RESET;
    set_report(t, &buffer).inspect_err(|e| {
        log::error!("Device reset failed: {}.", name(e));
    })?;
    buffer[0] = CMD_IDLE;
    set_report(t, &buffer).inspect_err(|e| {
        log::error!("Failed to set device in idle state: {}.", name(e));
    })
}

/// A trigger as the device takes it: one channel code and one type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeviceTrigger {
    /// `trigger_channel`.
    pub channel: u8,
    /// `trigger_type`.
    pub kind: u8,
    /// The matches on enabled channels that had to be left out.
    pub ignored: usize,
}

impl DeviceTrigger {
    /// No trigger.
    pub const NONE: DeviceTrigger =
        DeviceTrigger { channel: TRIGGER_CHANNEL_0, kind: TRIGGER_TYPE_NONE, ignored: 0 };

    /// Whether a trigger is set.
    pub fn is_set(self) -> bool {
        self.kind != TRIGGER_TYPE_NONE
    }
}

/// `sl2_convert_trigger()`, for a trigger that passed [`check_trigger`]. A
/// match on a disabled channel is ignored; of the others the last decides,
/// as the device has one trigger channel (protocol.c:495-513). All four on
/// any edge is the code for all channels.
pub fn convert_trigger(trigger: Option<&Trigger>, enabled: &[bool; NUM_CHANNELS]) -> DeviceTrigger {
    let mut out = DeviceTrigger::NONE;
    let Some(trigger) = trigger else { return out };
    let mut matches = 0;
    let mut any_edge = 0;
    for stage in &trigger.stages {
        for c in &stage.conditions {
            if !enabled.get(c.channel).copied().unwrap_or(false) {
                continue;
            }
            matches += 1;
            out.channel = TRIGGER_CHANNEL_0 + c.channel as u8;
            out.kind = match c.matches {
                TriggerMatch::Rising => TRIGGER_TYPE_POSEDGE,
                TriggerMatch::Falling => TRIGGER_TYPE_NEGEDGE,
                _ => {
                    any_edge += 1;
                    TRIGGER_TYPE_ANYEDGE
                }
            };
        }
    }
    if any_edge == NUM_CHANNELS {
        out.channel = TRIGGER_CHANNEL_ALL;
        return out;
    }
    out.ignored = usize::max(matches, 1) - 1;
    out
}

/// What the device cannot do, refused before a capture. The C logs "only
/// supports 1 trigger stage", then drops the error
/// (`sl2_convert_trigger()`'s `SR_ERR` at api.c:342) and captures with no
/// trigger at all.
///
/// # Errors
///
/// More than one stage, or a match that is no edge.
pub fn check_trigger(trigger: &Trigger) -> Result<()> {
    if trigger.stages.len() > 1 {
        return Err(Error::arg("This device only supports 1 trigger stage."));
    }
    let edges = trigger.stages.iter().flat_map(|s| &s.conditions).all(|c| c.matches.is_edge());
    if !edges {
        return Err(Error::arg("This device only triggers on edges."));
    }
    Ok(())
}

/// One capture's numbers, from the settings at its start.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Plan {
    /// `limit_samples`.
    pub limit: u64,
    /// `pre_trigger_samples`.
    pub pre_samples: u64,
    /// `pre_trigger_bytes`.
    pub pre_bytes: u16,
    /// `post_trigger_bytes`.
    pub post_bytes: u16,
    /// `samplerate_id`.
    pub rate_code: u8,
    /// The trigger.
    pub trigger: DeviceTrigger,
    /// `channel_map`: the enabled channels, in order.
    pub channels: Vec<u8>,
}

impl Plan {
    /// `sl2_calculate_trigger_samples()`. The limit is at most
    /// [`MAX_SAMPLES`] and the ratio at most 100, so nothing here wraps,
    /// and the bytes fit the C's `uint16_t`.
    pub fn new(
        limit: u64,
        capture_ratio: u64,
        rate_code: u8,
        trigger: DeviceTrigger,
        channels: Vec<u8>,
    ) -> Plan {
        let limit = limit.min(MAX_SAMPLES);
        // Without a trigger the capture ratio is ignored.
        let cr = if trigger.is_set() { capture_ratio.min(100) } else { 0 };
        let pre_samples = (limit * cr) / 100;
        let mut post_samples = (limit * (100 - cr)) / 100;
        // Makes up for a sample lost to the rounding.
        if pre_samples + post_samples != limit {
            post_samples += 1;
        }
        // Whole bytes of 8 samples, rounded up.
        let bytes = |samples: u64| samples.div_ceil(8) as u16;
        Plan {
            limit,
            pre_samples,
            pre_bytes: bytes(pre_samples),
            post_bytes: bytes(post_samples),
            rate_code,
            trigger,
            channels,
        }
    }

    /// `num_sample_packets`.
    pub fn packets(&self) -> usize {
        (usize::from(self.pre_bytes) + usize::from(self.post_bytes)).div_ceil(PACKET_SAMPLE_BYTES)
    }

    /// `CMD_SAMPLE`, as `dev_acquisition_start()` fills the transfer. The
    /// after trigger delay is 0: nothing in the C sets it.
    pub fn sample_command(&self) -> [u8; PACKET_LENGTH] {
        let mut b = [0; PACKET_LENGTH];
        b[0] = CMD_SAMPLE;
        b[2..4].copy_from_slice(&self.pre_bytes.to_le_bytes());
        b[4..6].copy_from_slice(&self.post_bytes.to_le_bytes());
        b[6] = self.rate_code;
        b[7] = self.trigger.kind;
        b[8] = self.trigger.channel;
        b
    }
}

/// The sample packets, channel by channel: those of the channels before
/// the last enabled one are kept, and the last one's turn the samples out
/// (`buffer_sample_data()`, `process_sample_data()`).
#[derive(Debug)]
struct Samples {
    plan: Plan,
    /// The kept packets of the enabled channels before the last.
    kept: [Vec<u8>; NUM_CHANNELS],
    /// `channel`.
    channel: usize,
    /// `sample_packet`.
    packet: usize,
    /// `samples_processed`.
    processed: u64,
}

impl Samples {
    fn new(plan: Plan) -> Samples {
        let mut kept: [Vec<u8>; NUM_CHANNELS] = Default::default();
        // The plan's limit bounds this to 265 packets per channel.
        let size = plan.packets() * PACKET_SAMPLE_BYTES;
        if let Some((_, others)) = plan.channels.split_last() {
            for c in others {
                kept[usize::from(*c)] = vec![0; size];
            }
        }
        Samples { plan, kept, channel: 0, packet: 0, processed: 0 }
    }

    /// `channel_map[num_enabled_channels - 1]`.
    fn last_channel(&self) -> usize {
        self.plan.channels.last().map_or(0, |c| usize::from(*c))
    }

    /// Whether the next packet's sample bytes are used.
    fn wants_data(&self) -> bool {
        self.channel == self.last_channel()
            || (self.channel < self.last_channel() && !self.kept[self.channel].is_empty())
    }

    /// `buffer_sample_data()`.
    fn keep(&mut self, bytes: &[u8]) {
        let bytes = &bytes[..bytes.len().min(PACKET_SAMPLE_BYTES)];
        let offset = self.packet * PACKET_SAMPLE_BYTES;
        if let Some(to) = self.kept[self.channel].get_mut(offset..offset + bytes.len()) {
            to.copy_from_slice(bytes);
        }
    }

    /// The sample bytes of the current packet that [`Samples::process`] will
    /// use, in any enabled channel: the samples skipped at the start and the
    /// limit decide how many of the bytes asked for count.
    fn bytes_used(&self) -> usize {
        let skip = u64::from(self.plan.pre_bytes) * 8 - self.plan.pre_samples;
        let total = (skip + self.plan.limit).div_ceil(8) as usize;
        total.saturating_sub(self.packet * PACKET_SAMPLE_BYTES).min(PACKET_SAMPLE_BYTES)
    }

    /// `process_sample_data()`: the current packet is the last channel's.
    /// The device stores whole bytes, so the first one starts where the
    /// samples before the trigger begin, and the trigger comes after
    /// `pre_samples` of them.
    fn process(&mut self, bytes: &[u8], emit: &mut impl FnMut(Packet)) {
        let offset = self.packet * PACKET_SAMPLE_BYTES;
        let triggered = self.plan.trigger.is_set();
        let mut out = Vec::with_capacity(PACKET_SAMPLE_BYTES * 8);
        for (i, current) in bytes.iter().enumerate().take(PACKET_SAMPLE_BYTES) {
            if self.processed == self.plan.limit {
                break;
            }
            let mut first_bit = 7;
            if self.processed == 0 {
                let skip = u64::from(self.plan.pre_bytes) * 8 - self.plan.pre_samples;
                first_bit = 7 - skip as i32;
                log::debug!("Start processing at sample: {skip}.");
                if triggered && self.plan.pre_samples == 0 {
                    emit(Packet::Trigger);
                }
            }
            for bit in (0..=first_bit).rev() {
                if self.processed == self.plan.limit {
                    break;
                }
                let mut sample = 0u8;
                for c in &self.plan.channels {
                    let c = usize::from(*c);
                    let byte = if c == self.channel { *current } else { self.kept[c][offset + i] };
                    sample |= ((byte >> bit) & 1) << c;
                }
                out.push(sample);
                self.processed += 1;
                if triggered && self.processed == self.plan.pre_samples {
                    emit(Packet::logic(1, std::mem::take(&mut out)));
                    emit(Packet::Trigger);
                }
            }
        }
        if !out.is_empty() {
            emit(Packet::logic(1, out));
        }
    }

    /// `sample_packet++`, `%= num_sample_packets`, `channel++`. The C
    /// divides by zero here when no bytes were asked for (protocol.c:317),
    /// which a library client with no limit can reach on a device that then
    /// says "data ready"; [`Samples::past_the_end`] holds for that capture.
    fn next_packet(&mut self) {
        self.packet += 1;
        if self.packet >= self.plan.packets() {
            self.packet = 0;
            self.channel += 1;
        }
    }

    /// Whether the enabled channels are done.
    fn past_the_end(&self) -> bool {
        self.channel > self.last_channel() || self.plan.packets() == 0
    }
}

/// `STATE_*`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Sample,
    WaitDataReady,
    ReceiveData,
    ResetAndIdle,
    WaitDeviceReady,
}

/// What comes after a transfer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Next {
    /// Send [`Machine::out`].
    Out,
    /// Read a report.
    In,
    /// Wait, then read: the status is not "data ready" yet.
    Wait,
    /// The capture is over.
    Done,
}

/// The state machine of `sl2_receive_transfer_in()` and
/// `sl2_receive_transfer_out()`, each called with the transfer's result.
#[derive(Debug)]
pub struct Machine {
    state: State,
    next: State,
    stopping_in_progress: bool,
    /// `xfer_data_out`: it keeps the `CMD_SAMPLE` fields behind later
    /// commands.
    out: [u8; PACKET_LENGTH],
    samples: Samples,
    /// Status reads spent waiting for a ready device.
    rounds: u32,
}

impl Machine {
    /// Before `CMD_SAMPLE` is sent.
    pub fn new(plan: Plan) -> Machine {
        Machine {
            state: State::Idle,
            next: State::Sample,
            stopping_in_progress: false,
            out: plan.sample_command(),
            samples: Samples::new(plan),
            rounds: 0,
        }
    }

    /// The report to send.
    pub fn out(&self) -> &[u8; PACKET_LENGTH] {
        &self.out
    }

    /// The first thing either callback does: a stop asked for reads the
    /// status and goes on from there, dropping what this transfer brought.
    fn stop_asked(&mut self, stopping: bool) -> bool {
        if stopping && !self.stopping_in_progress {
            self.next = State::ResetAndIdle;
            self.stopping_in_progress = true;
            return true;
        }
        false
    }

    /// `ikalogic_scanalogic2_receive_data()`'s look at a stop during a
    /// status wait.
    pub fn stop_while_waiting(&mut self) {
        self.stop_asked(true);
    }

    /// `sl2_receive_transfer_out()`.
    pub fn after_out(&mut self, stopping: bool) -> Next {
        if self.stop_asked(stopping) {
            return Next::In;
        }
        self.state = self.next;
        match self.state {
            State::Sample => {
                self.next = State::WaitDataReady;
                Next::In
            }
            State::WaitDeviceReady => Next::In,
            _ => Next::Done,
        }
    }

    /// `sl2_receive_transfer_in()`.
    ///
    /// # Errors
    ///
    /// The message of what ends the capture. The C ignores `actual_length`
    /// and goes on with what its buffer held before, so a sample packet that
    /// lacks bytes this capture uses ends the capture here, like a failed
    /// transfer; a shorter report is fine when it holds them all, as nothing
    /// shows that the firmware always sends 128 bytes. So does a device that
    /// does not get ready within [`MAX_RESET_ROUNDS`] status reads. A status
    /// of fewer than two bytes is no valid one, and cannot harm the samples.
    pub fn after_in(
        &mut self,
        data: &[u8],
        stopping: bool,
        emit: &mut impl FnMut(Packet),
    ) -> std::result::Result<Next, String> {
        if self.stop_asked(stopping) {
            return Ok(Next::In);
        }
        self.state = self.next;
        match self.state {
            State::WaitDataReady => {
                let [valid, code] = status(data);
                if valid == STATUS_VALID {
                    if code == STATUS_WAITING_FOR_TRIGGER {
                        log::debug!("Waiting for trigger.");
                    } else if code == STATUS_SAMPLING {
                        log::debug!("Sampling in progress.");
                    }
                }
                if valid == STATUS_VALID && code == STATUS_DATA_READY {
                    self.next = State::ReceiveData;
                    Ok(Next::In)
                } else {
                    Ok(Next::Wait)
                }
            }
            State::ReceiveData => {
                self.receive(data, emit)?;
                Ok(Next::In)
            }
            State::ResetAndIdle | State::WaitDeviceReady => {
                self.rounds += 1;
                if self.rounds > MAX_RESET_ROUNDS {
                    return Err(format!(
                        "The device is not ready after {MAX_RESET_ROUNDS} status reads."
                    ));
                }
                let [valid, code] = status(data);
                if valid != STATUS_VALID {
                    // Not able to take a command yet: ask again.
                    return Ok(Next::In);
                }
                if code == STATUS_DEVICE_READY {
                    self.next = State::Idle;
                    self.out[0] = CMD_IDLE;
                } else {
                    // After a capture, or a reset the device did not
                    // recognize: (again) a reset.
                    self.next = State::WaitDeviceReady;
                    self.out[0] = CMD_RESET;
                }
                Ok(Next::Out)
            }
            State::Idle | State::Sample => Ok(Next::Done),
        }
    }

    fn receive(
        &mut self,
        data: &[u8],
        emit: &mut impl FnMut(Packet),
    ) -> std::result::Result<(), String> {
        let s = &mut self.samples;
        if s.past_the_end() {
            self.next = State::ResetAndIdle;
        } else if s.wants_data() {
            let bytes = data.get(4..).unwrap_or(&[]);
            let used = s.bytes_used();
            if bytes.len() < used {
                return Err(format!(
                    "Sample packet {} of channel {} is {} bytes: it needs {}.",
                    s.packet,
                    s.channel,
                    data.len(),
                    used + 4
                ));
            }
            if s.channel == s.last_channel() {
                s.process(bytes, emit);
            } else {
                s.keep(bytes);
            }
        }
        s.next_packet();
        Ok(())
    }
}

/// The two bytes of a status; a shorter one is no valid status, where the C
/// reads the rest from its old buffer.
fn status(data: &[u8]) -> [u8; 2] {
    match data {
        [valid, code, ..] => [*valid, *code],
        _ => [0, 0],
    }
}

/// The waits and transfers of one capture, on its thread. The header goes
/// first, as `dev_acquisition_start()` sends it with `CMD_SAMPLE` under
/// way. A failure ends the capture with no more requests; the C's
/// `abort_acquisition()` also closes the device, which here waits for
/// [`crate::core::Device::close`].
pub fn run(
    t: &mut dyn Transport,
    plan: Plan,
    wait: Duration,
    sink: &PacketSink,
    stop: &AtomicBool,
) {
    let mut machine = Machine::new(plan);
    sink.send(Packet::header());
    let mut emit = |p: Packet| {
        sink.send(p);
    };
    let mut next = Next::Out;
    let mut waited = false;
    loop {
        next = match next {
            Next::Out => {
                if let Err(e) = set_report(t, machine.out()) {
                    log::error!("Transfer to device failed: {}.", transfer_status_name(&e));
                    return;
                }
                machine.after_out(stop.load(Ordering::SeqCst))
            }
            Next::In => {
                let data = match get_report(t) {
                    Ok(data) => data,
                    // A device that left during the wait: libusb refuses
                    // the C's submit, where this finds out at completion.
                    Err(Error::Disconnected) if waited => {
                        log::error!("Submit transfer failed: LIBUSB_ERROR_NO_DEVICE.");
                        return;
                    }
                    Err(e) => {
                        log::error!("Transfer to device failed: {}.", transfer_status_name(&e));
                        return;
                    }
                };
                waited = false;
                match machine.after_in(&data, stop.load(Ordering::SeqCst), &mut emit) {
                    Ok(next) => next,
                    Err(message) => {
                        log::error!("{message}");
                        return;
                    }
                }
            }
            Next::Wait => {
                if wait_or_stop(wait, stop) {
                    machine.stop_while_waiting();
                }
                waited = true;
                Next::In
            }
            Next::Done => return,
        };
    }
}

/// Waits, and says whether a stop came.
fn wait_or_stop(wait: Duration, stop: &AtomicBool) -> bool {
    let start = Instant::now();
    loop {
        if stop.load(Ordering::SeqCst) {
            return true;
        }
        let left = wait.saturating_sub(start.elapsed());
        if left.is_zero() {
            return false;
        }
        std::thread::sleep(left.min(STOP_POLL));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::record::Replay;
    use crate::usb::{BulkStream, UsbDeviceInfo};
    use std::sync::Arc;

    fn trigger(stages: &[&[(usize, TriggerMatch)]]) -> Trigger {
        let mut t = Trigger::new();
        for (i, conds) in stages.iter().enumerate() {
            for (channel, m) in *conds {
                t.add(i, *channel, *m);
            }
        }
        t
    }

    const RISING_0: DeviceTrigger = DeviceTrigger { channel: 1, kind: 1, ignored: 0 };

    #[test]
    fn samplerate_codes_count_down() {
        assert_eq!(samplerate_code(1_250), Some(0x0a));
        assert_eq!(samplerate_code(20_000_000), Some(0));
        assert_eq!(samplerate_code(3_000_000), None);
    }

    #[test]
    fn triggers_convert_as_the_c_does() {
        use TriggerMatch::{Edge as E, Falling as F, Rising as R};
        let all = [true; 4];
        assert_eq!(convert_trigger(None, &all), DeviceTrigger::NONE);
        let t = trigger(&[&[(0, R)]]);
        assert_eq!(convert_trigger(Some(&t), &all), RISING_0);
        let t = trigger(&[&[(3, F)]]);
        assert_eq!(
            convert_trigger(Some(&t), &all),
            DeviceTrigger { channel: 4, kind: 0, ignored: 0 }
        );
        // All four on any edge.
        let t = trigger(&[&[(0, E), (1, E), (2, E), (3, E)]]);
        assert_eq!(
            convert_trigger(Some(&t), &all),
            DeviceTrigger { channel: 0, kind: 2, ignored: 0 }
        );
        // Three of them, and a mix: the last wins, and the rest are counted.
        let t = trigger(&[&[(0, E), (1, E), (2, E)]]);
        assert_eq!(
            convert_trigger(Some(&t), &all),
            DeviceTrigger { channel: 3, kind: 2, ignored: 2 }
        );
        let t = trigger(&[&[(0, R), (3, F)]]);
        assert_eq!(
            convert_trigger(Some(&t), &all),
            DeviceTrigger { channel: 4, kind: 0, ignored: 1 }
        );
        // A disabled channel's match counts for nothing.
        let t = trigger(&[&[(0, R), (3, F)]]);
        assert_eq!(convert_trigger(Some(&t), &[true, true, true, false]), RISING_0);
        let t = trigger(&[&[(3, F)]]);
        assert_eq!(convert_trigger(Some(&t), &[true, true, true, false]), DeviceTrigger::NONE);
    }

    #[test]
    fn what_the_device_cannot_do_is_refused() {
        use TriggerMatch::{Falling as F, One, Rising as R};
        let t = trigger(&[&[(0, R)], &[(0, F)]]);
        assert!(check_trigger(&t).unwrap_err().to_string().contains("1 trigger stage"));
        let t = trigger(&[&[(0, One)]]);
        assert!(check_trigger(&t).is_err());
        assert!(check_trigger(&trigger(&[&[(0, R), (1, F)]])).is_ok());
    }

    #[test]
    fn pre_and_post_round_up_to_bytes() {
        let p = Plan::new(100, 1, 4, RISING_0, vec![0]);
        assert_eq!((p.pre_samples, p.pre_bytes, p.post_bytes), (1, 1, 13));
        // 0.25 and 24.75 lose a sample between them: post gets it back, and
        // 25 samples take a fourth byte.
        let p = Plan::new(25, 1, 4, RISING_0, vec![0]);
        assert_eq!((p.pre_samples, p.pre_bytes, p.post_bytes), (0, 0, 4));
        // No trigger: no ratio.
        let p = Plan::new(100, 50, 4, DeviceTrigger::NONE, vec![0]);
        assert_eq!((p.pre_samples, p.pre_bytes, p.post_bytes), (0, 0, 13));
        // The corners: nothing wraps, in a debug build.
        let p = Plan::new(MAX_SAMPLES, 50, 0, RISING_0, vec![0]);
        assert_eq!((p.pre_bytes, p.post_bytes, p.packets()), (16383, 16383, 265));
        let p = Plan::new(MAX_SAMPLES, 100, 0, RISING_0, vec![0]);
        assert_eq!((p.pre_bytes, p.post_bytes), (32765, 0));
        let p = Plan::new(u64::MAX, 0, 0, RISING_0, vec![0]);
        assert_eq!((p.limit, p.post_bytes, p.packets()), (MAX_SAMPLES, 32765, 265));
        let p = Plan::new(0, 100, 0, RISING_0, vec![0]);
        assert_eq!((p.pre_bytes, p.post_bytes, p.packets()), (0, 0, 0));
    }

    #[test]
    fn the_command_holds_the_plan() {
        let t = DeviceTrigger { channel: 4, kind: 0, ignored: 0 };
        let b = Plan::new(800, 25, 0x06, t, vec![0]).sample_command();
        assert_eq!(b[..12], [1, 0, 25, 0, 75, 0, 6, 0, 4, 0, 0, 0]);
        assert_eq!(b[12..], [0; 116]);
    }

    /// Feeds a `Machine` packets and collects what it sends.
    fn feed(m: &mut Machine, data: &[u8]) -> (std::result::Result<Next, String>, Vec<Packet>) {
        let mut out = Vec::new();
        let r = m.after_in(data, false, &mut |p| out.push(p));
        (r, out)
    }

    fn status_report(code: u8) -> [u8; 128] {
        let mut r = [0; 128];
        r[0] = 5;
        r[1] = code;
        r
    }

    fn packet(channel: u8, number: u8, fill: u8) -> [u8; 128] {
        let mut r = [fill; 128];
        r[0] = 5;
        r[1] = channel;
        r[2] = number;
        r[3] = 0;
        r
    }

    /// `CMD_SAMPLE` sent, "data ready" read.
    fn at_data(plan: Plan) -> Machine {
        let mut m = Machine::new(plan);
        assert_eq!(m.after_out(false), Next::In);
        assert_eq!(feed(&mut m, &status_report(STATUS_DATA_READY)).0, Ok(Next::In));
        m
    }

    fn logic(p: &[Packet]) -> Vec<u8> {
        let mut out = Vec::new();
        for x in p {
            if let Packet::Logic(l) = x {
                assert_eq!(l.unit_size, 1);
                out.extend_from_slice(&l.data);
            }
        }
        out
    }

    #[test]
    fn a_status_that_is_not_ready_waits_and_asks_again() {
        let mut m = Machine::new(Plan::new(8, 0, 0, DeviceTrigger::NONE, vec![0]));
        m.after_out(false);
        assert_eq!(feed(&mut m, &status_report(0x61)).0, Ok(Next::Wait));
        assert_eq!(feed(&mut m, &status_report(0x62)).0, Ok(Next::Wait));
        // The old buffer: not a status.
        assert_eq!(feed(&mut m, &[0x0a; 128]).0, Ok(Next::Wait));
        // "Data ready" counts only behind the byte that makes it a status.
        assert_eq!(feed(&mut m, &[0x0a, STATUS_DATA_READY]).0, Ok(Next::Wait));
        assert_eq!(feed(&mut m, &status_report(STATUS_DATA_READY)).0, Ok(Next::In));
    }

    #[test]
    fn two_channels_combine_at_their_indexes_and_the_rest_are_skipped() {
        let mut m = at_data(Plan::new(8, 0, 0, DeviceTrigger::NONE, vec![1, 3]));
        // Channel 0 is not enabled: its bytes do not count, not even short.
        let (r, out) = feed(&mut m, &[5, 0]);
        assert_eq!((r, out.len()), (Ok(Next::In), 0));
        assert_eq!(feed(&mut m, &packet(1, 0, 0xf0)).1.len(), 0);
        assert_eq!(feed(&mut m, &packet(2, 0, 0xff)).1.len(), 0);
        let (r, out) = feed(&mut m, &packet(3, 0, 0xaa));
        assert_eq!(r, Ok(Next::In));
        assert_eq!(logic(&out), [0x0a, 0x02, 0x0a, 0x02, 0x08, 0x00, 0x08, 0x00]);
        // The read after the last enabled channel, then the end: ready, IDLE.
        assert_eq!(feed(&mut m, &packet(0, 0, 0)).0, Ok(Next::In));
        assert_eq!(feed(&mut m, &status_report(STATUS_DEVICE_READY)).0, Ok(Next::Out));
        assert_eq!(m.out()[0], CMD_IDLE);
        assert_eq!(m.after_out(false), Next::Done);
        assert_eq!(
            m.out()[1..12],
            Plan::new(8, 0, 0, DeviceTrigger::NONE, vec![1]).sample_command()[1..12]
        );
    }

    #[test]
    fn packets_of_a_later_channel_meet_the_kept_ones_at_their_offset() {
        // Two packets per channel.
        let mut m = at_data(Plan::new(1000, 0, 0, DeviceTrigger::NONE, vec![0, 1]));
        let _ = feed(&mut m, &packet(0, 0, 0xff));
        let _ = feed(&mut m, &packet(0, 1, 0x00));
        let (_, first) = feed(&mut m, &packet(1, 0, 0x00));
        assert_eq!(logic(&first), [1; 992]);
        let (_, second) = feed(&mut m, &packet(1, 1, 0xff));
        assert_eq!(logic(&second), [2; 8]);
    }

    #[test]
    fn the_first_samples_are_skipped_and_the_trigger_falls_after_pre() {
        // pre 3 samples: one byte, of which 5 go. 13 samples in all.
        let plan = Plan::new(13, 25, 0, RISING_0, vec![0]);
        assert_eq!((plan.pre_samples, plan.pre_bytes, plan.post_bytes), (3, 1, 2));
        let mut m = at_data(plan);
        let mut r = packet(0, 0, 0);
        // The byte's last three bits, then 10 of the next 16.
        r[4] = 0b1110_0101;
        r[5] = 0b1000_0000;
        r[6] = 0b0100_0000;
        let (_, out) = feed(&mut m, &r);
        assert_eq!(out.len(), 3);
        assert_eq!(out[0], Packet::logic(1, vec![1, 0, 1]));
        assert_eq!(out[1], Packet::Trigger);
        assert_eq!(out[2], Packet::logic(1, vec![1, 0, 0, 0, 0, 0, 0, 0, 0, 1]));
    }

    #[test]
    fn no_pre_trigger_samples_send_the_trigger_first() {
        let mut m = at_data(Plan::new(8, 0, 0, RISING_0, vec![0]));
        let (_, out) = feed(&mut m, &packet(0, 0, 0xff));
        assert_eq!(out[0], Packet::Trigger);
        assert_eq!(out[1], Packet::logic(1, vec![1; 8]));
        assert_eq!(out.len(), 2);
    }

    #[test]
    fn a_full_ratio_ends_on_the_trigger() {
        let mut m = at_data(Plan::new(8, 100, 0, RISING_0, vec![0]));
        let (_, out) = feed(&mut m, &packet(0, 0, 0xff));
        assert_eq!(out, [Packet::logic(1, vec![1; 8]), Packet::Trigger]);
    }

    #[test]
    fn the_limit_cuts_the_samples() {
        let mut m = at_data(Plan::new(9, 0, 0, DeviceTrigger::NONE, vec![0]));
        assert_eq!(logic(&feed(&mut m, &packet(0, 0, 0xff)).1), [1; 9]);
    }

    #[test]
    fn a_short_packet_ends_the_capture_instead_of_repeating_old_samples() {
        let mut m = at_data(Plan::new(32, 0, 0, DeviceTrigger::NONE, vec![0]));
        let (r, out) = feed(&mut m, &[5, 0, 0, 0]);
        assert_eq!(r.unwrap_err(), "Sample packet 0 of channel 0 is 4 bytes: it needs 8.");
        assert_eq!(out.len(), 0);
        // Of a channel that is kept too.
        let mut m = at_data(Plan::new(32, 0, 0, DeviceTrigger::NONE, vec![0, 1]));
        assert!(feed(&mut m, &[5, 0, 0, 0]).0.is_err());
    }

    #[test]
    fn a_short_packet_that_holds_every_byte_used_is_fine() {
        // 16 samples: two sample bytes, in a report of six.
        let mut m = at_data(Plan::new(16, 0, 0, DeviceTrigger::NONE, vec![0]));
        let (r, out) = feed(&mut m, &[5, 0, 0, 0, 0xff, 0x0f]);
        assert_eq!(r, Ok(Next::In));
        assert_eq!(logic(&out), [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1]);
        // One byte less is not.
        let mut m = at_data(Plan::new(16, 0, 0, DeviceTrigger::NONE, vec![0]));
        let e = feed(&mut m, &[5, 0, 0, 0, 0xff]).0.unwrap_err();
        assert_eq!(e, "Sample packet 0 of channel 0 is 5 bytes: it needs 6.");
        // A kept channel's, which the last one's samples then meet.
        let mut m = at_data(Plan::new(16, 0, 0, DeviceTrigger::NONE, vec![0, 1]));
        assert_eq!(feed(&mut m, &[5, 0, 0, 0, 0xff, 0x0f]).0, Ok(Next::In));
        let (_, out) = feed(&mut m, &[5, 1, 0, 0, 0x00, 0xff]);
        assert_eq!(logic(&out), [1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3]);
        // The samples skipped at the start count: 3 + 10 samples in 3 bytes.
        let plan = Plan::new(13, 25, 0, RISING_0, vec![0]);
        let mut m = at_data(plan.clone());
        assert!(feed(&mut m, &[5, 0, 0, 0, 0xff, 0xff]).0.is_err());
        let mut m = at_data(plan);
        assert_eq!(feed(&mut m, &[5, 0, 0, 0, 0xff, 0xff, 0xff]).0, Ok(Next::In));
        // The second packet of 125 bytes needs one.
        let mut m = at_data(Plan::new(1000, 0, 0, DeviceTrigger::NONE, vec![0]));
        assert_eq!(feed(&mut m, &packet(0, 0, 0xff)).0, Ok(Next::In));
        assert!(feed(&mut m, &[5, 0, 1, 0]).0.is_err());
        let mut m = at_data(Plan::new(1000, 0, 0, DeviceTrigger::NONE, vec![0]));
        assert_eq!(feed(&mut m, &packet(0, 0, 0xff)).0, Ok(Next::In));
        assert_eq!(logic(&feed(&mut m, &[5, 0, 1, 0, 0xff]).1), [1; 8]);
        // A packet asked for only by the rounding needs none: 8 samples at a
        // ratio of 100 fill one byte, and nothing of a later channel counts.
        let mut m = at_data(Plan::new(8, 100, 0, RISING_0, vec![0]));
        assert_eq!(feed(&mut m, &packet(0, 0, 0xff)).0, Ok(Next::In));
        assert_eq!(feed(&mut m, &[5]).0, Ok(Next::In));
    }

    #[test]
    fn a_status_of_fewer_than_two_bytes_is_no_status() {
        for short in [&[][..], &[5][..]] {
            let mut m = Machine::new(Plan::new(8, 0, 0, DeviceTrigger::NONE, vec![0]));
            m.after_out(false);
            assert_eq!(feed(&mut m, short).0, Ok(Next::Wait));
            // Two are enough.
            assert_eq!(feed(&mut m, &[5, 0x60]).0, Ok(Next::In));
            let _ = feed(&mut m, &packet(0, 0, 0));
            let _ = feed(&mut m, &packet(1, 0, 0));
            // After the capture: ask again, under the bound.
            let mut rounds = 0;
            loop {
                match feed(&mut m, short).0 {
                    Ok(Next::In) => rounds += 1,
                    Err(e) => {
                        assert!(e.contains("not ready"), "{e}");
                        break;
                    }
                    other => panic!("{other:?}"),
                }
            }
            assert_eq!(rounds, MAX_RESET_ROUNDS);
        }
    }

    #[test]
    fn no_limit_ends_where_the_c_divides_by_zero() {
        let mut m = at_data(Plan::new(0, 0, 0, DeviceTrigger::NONE, vec![0, 1, 2, 3]));
        let (r, out) = feed(&mut m, &status_report(STATUS_DEVICE_READY));
        assert_eq!((r, out.len()), (Ok(Next::In), 0));
        assert_eq!(feed(&mut m, &status_report(STATUS_DEVICE_READY)).0, Ok(Next::Out));
        assert_eq!(m.out()[0], CMD_IDLE);
        assert_eq!(m.after_out(false), Next::Done);
    }

    #[test]
    fn a_reset_goes_on_until_the_device_is_ready() {
        let mut m = at_data(Plan::new(8, 0, 0, DeviceTrigger::NONE, vec![0]));
        let _ = feed(&mut m, &packet(0, 0, 0));
        let _ = feed(&mut m, &packet(1, 0, 0));
        // A packet: valid, not ready. RESET, once more, and it asks again
        // when a report is no status.
        assert_eq!(feed(&mut m, &packet(2, 0, 0)).0, Ok(Next::Out));
        assert_eq!(m.out()[0], CMD_RESET);
        assert_eq!(m.after_out(false), Next::In);
        assert_eq!(feed(&mut m, &[0; 128]).0, Ok(Next::In));
        assert_eq!(feed(&mut m, &packet(2, 0, 0)).0, Ok(Next::Out));
        assert_eq!(m.out()[0], CMD_RESET);
        assert_eq!(m.after_out(false), Next::In);
        assert_eq!(feed(&mut m, &status_report(STATUS_DEVICE_READY)).0, Ok(Next::Out));
        assert_eq!(m.out()[0], CMD_IDLE);
        assert_eq!(m.after_out(false), Next::Done);
    }

    #[test]
    fn a_device_never_ready_ends_the_wait() {
        let mut m = at_data(Plan::new(8, 0, 0, DeviceTrigger::NONE, vec![0]));
        let _ = feed(&mut m, &packet(0, 0, 0));
        let _ = feed(&mut m, &packet(1, 0, 0));
        let mut rounds = 0;
        loop {
            match feed(&mut m, &packet(2, 0, 0)).0 {
                Ok(Next::Out) => assert_eq!(m.after_out(false), Next::In),
                Ok(other) => panic!("{other:?}"),
                Err(e) => {
                    assert!(e.contains("not ready"), "{e}");
                    break;
                }
            }
            rounds += 1;
        }
        assert_eq!(rounds, MAX_RESET_ROUNDS);
    }

    /// A stop drops what the transfer brought: a mutant that ignores the
    /// flag sends these samples and reads the packets after them.
    #[test]
    fn a_stop_after_a_packet_drops_it_and_reads_a_status() {
        let mut m = at_data(Plan::new(5000, 0, 0, DeviceTrigger::NONE, vec![0]));
        assert_eq!(logic(&feed(&mut m, &packet(0, 0, 0xff)).1), [1; 992]);
        let mut out = Vec::new();
        let r = m.after_in(&packet(0, 1, 0xff), true, &mut |p| out.push(p));
        assert_eq!((r, out.len()), (Ok(Next::In), 0));
        // The flag stays up. A valid status that is not ready: RESET.
        let r = m.after_in(&packet(0, 2, 0xff), true, &mut |p| out.push(p));
        assert_eq!((r, out.len()), (Ok(Next::Out), 0));
        assert_eq!(m.out()[0], CMD_RESET);
        assert_eq!(m.after_out(true), Next::In);
        let r = m.after_in(&status_report(STATUS_DEVICE_READY), true, &mut |p| out.push(p));
        assert_eq!((r, out.len()), (Ok(Next::Out), 0));
        assert_eq!(m.out()[0], CMD_IDLE);
        assert_eq!(m.after_out(true), Next::Done);
    }

    #[test]
    fn a_stop_before_the_first_status_is_seen_after_the_command() {
        let mut m = Machine::new(Plan::new(8, 0, 0, DeviceTrigger::NONE, vec![0]));
        assert_eq!(m.after_out(true), Next::In);
        // Ready: straight to IDLE.
        let r = m.after_in(&status_report(STATUS_DEVICE_READY), true, &mut |_| panic!());
        assert_eq!(r, Ok(Next::Out));
        assert_eq!(m.out()[0], CMD_IDLE);
    }

    #[test]
    fn a_stop_during_a_status_wait_reads_at_once_and_resets() {
        let mut m = Machine::new(Plan::new(8, 0, 0, DeviceTrigger::NONE, vec![0]));
        m.after_out(false);
        assert_eq!(feed(&mut m, &status_report(0x61)).0, Ok(Next::Wait));
        m.stop_while_waiting();
        assert_eq!(feed(&mut m, &status_report(0x61)).0, Ok(Next::Out));
        assert_eq!(m.out()[0], CMD_RESET);
    }

    #[test]
    fn a_wait_ends_on_time_or_on_a_stop() {
        let stop = AtomicBool::new(false);
        assert!(!wait_or_stop(Duration::from_millis(1), &stop));
        stop.store(true, Ordering::SeqCst);
        let start = Instant::now();
        assert!(wait_or_stop(Duration::from_secs(60), &stop));
        assert!(start.elapsed() < Duration::from_secs(30));
    }

    #[test]
    fn transfer_statuses_are_named_as_libusb_names_them() {
        assert_eq!(
            transfer_status_name(&Error::device("endpoint stalled")),
            "LIBUSB_TRANSFER_STALL"
        );
        assert_eq!(transfer_status_name(&Error::Disconnected), "LIBUSB_TRANSFER_NO_DEVICE");
        assert_eq!(transfer_status_name(&Error::Timeout), "LIBUSB_TRANSFER_TIMED_OUT");
        assert_eq!(transfer_status_name(&Error::device("other")), "LIBUSB_TRANSFER_ERROR");
    }

    /// A replay that sets the stop flag during a transfer, as a stop arrives
    /// while that one is under way: `run()` must look at the flag after it.
    struct StopAt {
        replay: Replay,
        reads: usize,
        writes: usize,
        stop: Arc<AtomicBool>,
        /// The transfers in order: a command's byte, or 0 for a read. The
        /// replay would answer an extra status read by repeating the last.
        log: Vec<u8>,
    }

    impl Transport for StopAt {
        fn info(&self) -> &UsbDeviceInfo {
            self.replay.info()
        }
        fn set_configuration(&mut self, _: u8) -> Result<()> {
            unreachable!()
        }
        fn claim_interface(&mut self, _: u8) -> Result<()> {
            unreachable!()
        }
        fn release_interface(&mut self, _: u8) -> Result<()> {
            unreachable!()
        }
        fn reset(&mut self) -> Result<()> {
            unreachable!()
        }
        fn max_packet_size(&mut self, _: u8) -> Result<usize> {
            unreachable!()
        }
        fn interfaces(&mut self) -> Result<Vec<crate::usb::InterfaceDesc>> {
            unreachable!()
        }
        fn control_out(&mut self, req: ControlRequest, data: &[u8], t: Duration) -> Result<()> {
            self.log.push(data[0]);
            self.writes = self.writes.saturating_sub(1);
            if self.writes == 0 {
                self.stop.store(true, Ordering::SeqCst);
            }
            self.replay.control_out(req, data, t)
        }
        fn control_in(&mut self, req: ControlRequest, n: usize, t: Duration) -> Result<Vec<u8>> {
            self.log.push(0);
            self.reads = self.reads.saturating_sub(1);
            if self.reads == 0 {
                self.stop.store(true, Ordering::SeqCst);
            }
            self.replay.control_in(req, n, t)
        }
        fn bulk_out(&mut self, _: u8, _: &[u8], _: Duration) -> Result<usize> {
            unreachable!()
        }
        fn bulk_in(&mut self, _: u8, _: usize, _: Duration) -> Result<Vec<u8>> {
            unreachable!()
        }
        fn interrupt_out(&mut self, _: u8, _: &[u8], _: Duration) -> Result<usize> {
            unreachable!()
        }
        fn interrupt_in(&mut self, _: u8, _: usize, _: Duration) -> Result<Vec<u8>> {
            unreachable!()
        }
        fn bulk_in_stream(&mut self, _: u8, _: usize, _: usize) -> Result<Box<dyn BulkStream>> {
            unreachable!()
        }
    }

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    fn set(bytes: &[u8; 128]) -> String {
        format!("ctl_out 21 09 0300 0000 {} -> ok\n", hex(bytes))
    }

    fn get(bytes: &[u8; 128]) -> String {
        format!("ctl_in 21 01 0300 0000 128 -> {}\n", hex(bytes))
    }

    /// `run()` on a replay of `steps`, the flag set on the Nth read or write.
    /// Every step must be used, in order, and the packets and the transfers
    /// come back.
    fn run_stopped(plan: Plan, steps: &str, reads: usize, writes: usize) -> (Vec<Packet>, Vec<u8>) {
        let text = format!(
            "# oxrok usb fixture 1\ndevice vid=20a0 pid=4123 version=0100 bus=0 addr=0 \
             strings=1,2,0\n{steps}"
        );
        let replay = Replay::from_text(&text).unwrap();
        let check = replay.clone();
        let stop = Arc::new(AtomicBool::new(false));
        let mut t = StopAt { replay, reads, writes, stop: Arc::clone(&stop), log: Vec::new() };
        let (tx, rx) = std::sync::mpsc::sync_channel(16);
        run(&mut t, plan, Duration::from_millis(1), &PacketSink::new(tx, 0), &stop);
        check.finish().unwrap();
        (rx.try_iter().map(|(_, p)| p).collect(), t.log)
    }

    fn with_command(mut report: [u8; 128], command: u8) -> [u8; 128] {
        report[0] = command;
        report
    }

    /// The stop comes during the second packet's read: its samples go, and
    /// so do the packets after it, which the device still offers. A mutant
    /// passing `false` for the flag to `after_in` sends them.
    #[test]
    fn run_stops_on_a_read_and_drops_what_it_brought() {
        let plan = Plan::new(5000, 0, 0, DeviceTrigger::NONE, vec![0]);
        let cmd = plan.sample_command();
        let steps = [
            set(&cmd),
            get(&status_report(STATUS_DATA_READY)),
            get(&packet(0, 0, 0xff)),
            get(&packet(0, 1, 0xff)),
            // The status read: a packet is valid, and not ready.
            get(&packet(0, 2, 0xff)),
            set(&with_command(cmd, CMD_RESET)),
            get(&status_report(STATUS_DEVICE_READY)),
            set(&with_command(cmd, CMD_IDLE)),
        ]
        .concat();
        let (packets, log) = run_stopped(plan, &steps, 3, usize::MAX);
        assert_eq!(log, [CMD_SAMPLE, 0, 0, 0, 0, CMD_RESET, 0, CMD_IDLE]);
        assert!(matches!(packets[0], Packet::Header { .. }));
        assert_eq!(packets[1..], [Packet::logic(1, vec![1; 992])]);
    }

    /// The stop comes during `CMD_SAMPLE`: the next read is taken as the
    /// status. A mutant passing `false` to `after_out` reads it as the wait
    /// for data, and goes on to one read more before any RESET.
    #[test]
    fn run_stops_on_a_write_and_reads_a_status() {
        let plan = Plan::new(8, 0, 0, DeviceTrigger::NONE, vec![0]);
        let cmd = plan.sample_command();
        let steps = [
            set(&cmd),
            get(&status_report(0x61)),
            set(&with_command(cmd, CMD_RESET)),
            get(&status_report(STATUS_DEVICE_READY)),
            set(&with_command(cmd, CMD_IDLE)),
        ]
        .concat();
        let (packets, log) = run_stopped(plan, &steps, usize::MAX, 1);
        assert_eq!(log, [CMD_SAMPLE, 0, CMD_RESET, 0, CMD_IDLE]);
        assert_eq!(packets.len(), 1, "{packets:?}");
    }
}
