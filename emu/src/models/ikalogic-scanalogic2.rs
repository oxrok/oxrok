// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The IKALOGIC Scanalogic-2: an `ATmega168PA` with four 23K256 SRAMs
//! behind a low speed HID interface. Every command and answer is one
//! 128 byte feature report, wValue `0x0300`, wIndex 0, on endpoint 0.
//!
//! # Sources
//!
//! - Firmware or HDL source: looked for (sigrok-firmware, web searches,
//!   the vendor's pages) and found none. The driver uploads nothing.
//! - The sigrok wiki, `IKALOGIC_Scanalogic-2` ("Protocol", written in 2013
//!   by Marc Schink, the driver's author, as the page history shows): the
//!   byte layout
//!   of the commands, of a sample packet (`05 CHANNEL NUMBER 00`) and of
//!   the info report (`0a`, the serial, the version), "sample packets are
//!   received sequentially per channel", "if no more sample data is
//!   available the device status changes to ready", "it might take a
//!   short time until a valid device status is received; in the meantime
//!   the last valid data of the device buffer will be received", "the
//!   device status can not be obtained in idle state", "an other status
//!   \[after a reset\] indicates that the device did not recognize the
//!   reset", the 262120 sample limit and the example serial 1371371152,
//!   firmware 1.3, and the packet number's wrap "if more than 253952 samples
//!   (124 bytes per packet * 8 samples per byte * 256 packets) are
//!   requested", which fits a number that restarts with each channel. The
//!   driver author's own description, the only one found; not checked
//!   against a device here.
//! - The sigrok wiki, `IKALOGIC_Scanalogic-2/Info` (lsusb and dmesg of a
//!   real device): every descriptor here, byte for byte: USB 1.10, HID
//!   1.01, low speed, `bMaxPacketSize0` 8, the strings, the 22 byte report
//!   descriptor, interrupt IN 0x81 of 8 bytes every 100 ms. Sure.
//! - The C driver: the commands' use, `USB_TIMEOUT_MS`, the comments in
//!   `sl2_receive_transfer_in` (a status that is not valid while the
//!   device cannot take commands). That the device sends all four channels
//!   whichever are enabled is an inference from the driver, not something
//!   the wiki says: `CMD_SAMPLE` has no channel mask, and
//!   `buffer_sample_data` checks `devc->channels[devc->channel]->enabled`
//!   (protocol.c:54) for channels it is handed.
//!
//! # The protocol as modelled
//!
//! A read answers by the phase the device is in: `05 63` when ready; after
//! `CMD_SAMPLE` `05 61` while it waits for the trigger, `05 62` while it
//! samples, `05 60` once when the data is ready, then the packets: channel
//! 0, 1, 2, 3, each of `(pre + post) / 124` packets rounded up, whichever
//! channels the host enabled. After the last packet the status is ready.
//! A driver that has read its last enabled channel reads once more, then
//! looks at the next read: if a packet is still left for it, that is a valid
//! status that is not ready, so it sends `CMD_RESET`; if the once more took
//! the last packet (with one packet per channel, channels 0 to 2 enabled), or
//! channel 3 was enabled, it reads ready and goes straight to `CMD_IDLE`.
//! `CMD_RESET` makes it ready from anywhere; `CMD_IDLE` makes it idle;
//! `CMD_INFO` makes the next read the info report.
//!
//! # Checks
//!
//! Reports through [`Violations`]: a report of another kind, id or length
//! than the one feature report (kind other than feature also STALLs); an
//! unknown command; `CMD_SAMPLE`'s reserved bytes 1 and 9, a samplerate
//! code over 0x0a, a trigger type over 3, a trigger channel over 4, "all
//! channels" with anything but any edge, an after trigger delay over
//! 65000 ms, pre + post over the 32766 bytes of the memory; `CMD_SAMPLE`
//! while a capture runs; `CMD_INFO` while not ready
//! (idle included: the wiki's "reset to leave idle"); `CMD_IDLE` while not
//! ready (the wiki's recommendation, which the driver keeps); a read in
//! idle, which has no status to give (it gets the old buffer).
//!
//! # C habits accepted, not flagged
//!
//! - `dev_open()` leaves the device idle and `dev_acquisition_start()`
//!   sends `CMD_SAMPLE` with no reset (against the wiki's "reset to leave
//!   idle", yet the wiki lists the driver as supported, so a real device
//!   must take it), so `CMD_SAMPLE` is taken from idle.
//! - The bytes after the meaningful ones are not checked, as "can have an
//!   arbitrary value": `sl2_get_device_info()` reuses the info answer as
//!   its buffer, so the `CMD_RESET` and `CMD_IDLE` after the read carry
//!   the serial and version; after a capture the same goes for the
//!   `CMD_SAMPLE` fields behind `CMD_RESET` and `CMD_IDLE`.
//! - A capture of 0 bytes is what the driver sends without a limit (a
//!   library client only: sigrok-cli starts no session without one). What a
//!   real device answers is not known: the model says data ready, then
//!   ready. The driver's division by zero (protocol.c:317) depends on that
//!   choice: it needs a read in `STATE_RECEIVE_DATA`, so only after `05 60`.
//!   A device that never said data ready would have the driver poll for
//!   ever instead.
//!
//! # Emulation artifacts
//!
//! - Time is counted in status reads, never the clock. By default the
//!   capture is done at the first status read whatever the rate and the
//!   trigger; `wait-trigger=N` and `sampling=N` make that N reads of each.
//!   A real capture at 1.25 kHz takes up to 210 s.
//! - Samples are a function of channel and byte offset, so both tools see
//!   the same; on the trigger channel the samples around the byte boundary
//!   `pre` are forced to the edge asked for (any edge: rising; all
//!   channels: on channel 0). The device stores samples before and after
//!   the trigger in whole bytes, as the driver's `sl2_calculate_trigger_samples`
//!   comment says; that the trigger falls exactly on the boundary is the
//!   model's choice.
//! - The bytes the wiki says can be ignored (a status report's bytes 2 and
//!   up, an info report's 7 and up) and the rest of a channel's last packet
//!   are 0. The packet number restarts with each channel: the wiki says
//!   "sequentially per channel" and its wrap figure (above) fits that, but
//!   does not say so in so many words.
//! - The real device resets itself after some seconds out of idle (the
//!   wiki, `dev_open()`'s comment): on wall time, so not modelled.
//! - The interrupt endpoint never answers: nothing in either source says
//!   what it would send, and the driver never reads it.
//!
//! # Options
//!
//! `serial=N` (u32, default 1371371152), `fw=MAJOR.MINOR` (default 1.3),
//! `wait-trigger=N|never`, `sampling=N`: the board. `digest=1` logs every
//! report as `@128:SHA256`, for captures whose `.tx` would be large
//! (`EMU_TX_FOLD` folds them); it changes the log, not the device.
//!
//! Faults, off by default; N counts from 1, once, in the feature reads
//! (`-get`) and writes (`-set`) since the model began, so they are one-shot,
//! as firmware that drops one request does (a real STALL on endpoint 0 ends
//! with the transfer anyway):
//! `stall-get=N`, `stall-set=N` (the request has no effect), `short-get=N`
//! (the answer's first `short-len=` bytes, default 4: a short control read,
//! which any device can end early).
//! Device behaviour named by the sources, counted in status reads:
//! `stale-sample=N` (the first N reads after each `CMD_SAMPLE` answer the
//! old buffer), `stale-info=1` (the read after `CMD_INFO` does),
//! `stale-reset=N` (the first N after the first `CMD_RESET` of a running
//! capture do), `busy-reset=N` (the same, with a zeroed report: byte 0 not
//! 0x05, as `sl2_receive_transfer_in`'s comment describes),
//! `ignore-reset=N|never` (that many `CMD_RESET` of a running capture are
//! not recognized; `never` means none ever is).

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::hid::{self, HidInterface, HidReports};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "ikalogic-scanalogic2";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "IKALOGIC Scanalogic-2 (low speed HID, 20a0:4123, 128 byte feature \
     reports); serial=N, fw=MAJOR.MINOR, wait-trigger=N|never, sampling=N, digest=1; faults: \
     stall-get=N, stall-set=N, short-get=N, short-len=N, stale-sample=N, stale-info=1, stale-reset=N, busy-reset=N, \
      ignore-reset=N|never";

/// The USB vendor id (Clay Logic).
pub const VENDOR_ID: u16 = 0x20a0;
/// The USB product id.
pub const PRODUCT_ID: u16 = 0x4123;
/// `USB_INTERFACE`.
pub const INTERFACE: u8 = 0;
/// `PACKET_LENGTH`: every report.
pub const REPORT_BYTES: usize = 128;
/// `PACKET_NUM_SAMPLE_BYTES`: the sample bytes of a packet.
pub const PACKET_SAMPLE_BYTES: usize = 124;
/// `NUM_CHANNELS`.
pub const CHANNELS: u8 = 4;

const CMD_SAMPLE: u8 = 0x01;
const CMD_RESET: u8 = 0x02;
const CMD_IDLE: u8 = 0x07;
const CMD_INFO: u8 = 0x0a;

const STATUS_VALID: u8 = 0x05;
const STATUS_DATA_READY: u8 = 0x60;
const STATUS_WAITING_FOR_TRIGGER: u8 = 0x61;
const STATUS_SAMPLING: u8 = 0x62;
const STATUS_DEVICE_READY: u8 = 0x63;

const LAST_SAMPLERATE: u8 = 0x0a;
const TRIGGER_ALL: u8 = 0x00;
const LAST_TRIGGER_CHANNEL: u8 = 0x04;
const TRIGGER_NEGEDGE: u8 = 0x00;
const TRIGGER_ANYEDGE: u8 = 0x02;
const TRIGGER_NONE: u8 = 0x03;
const MAX_AFTER_TRIGGER_DELAY: u16 = 65000;
/// `MAX_DEV_SAMPLE_BYTES`, the wiki's 262120 samples plus the byte the
/// driver's rounding can add.
const MAX_SAMPLE_BYTES: u32 = 32766;

/// The default serial number, the wiki's example.
pub const SERIAL: u32 = 1_371_371_152;
/// The default firmware version, the wiki's example.
pub const FIRMWARE: (u8, u8) = (1, 3);

/// A count of status reads or resets that can also be endless.
const NEVER: u32 = u32::MAX;

/// The report descriptor, as lsusb decodes it (22 bytes): vendor page, one
/// feature report of 128 bytes with no id.
pub const REPORT_DESCRIPTOR: [u8; 22] = [
    0x06, 0x00, 0xff, // usage page, vendor defined
    0x09, 0x01, // usage 1
    0xa1, 0x01, // collection, application
    0x15, 0x00, // logical minimum 0
    0x26, 0xff, 0x00, // logical maximum 255
    0x75, 0x08, // report size 8
    0x95, 0x80, // report count 128
    0x09, 0x00, // usage 0
    0xb2, 0x02, 0x01, // feature, data, variable, absolute, buffered bytes
    0xc0, // end collection
];

/// `digest=1` logs every report as a digest.
pub fn log_rules(options: &[(String, String)]) -> crate::txlog::Rules {
    crate::txlog::Rules {
        digest_above: options.iter().any(|(k, v)| k == "digest" && v == "1").then_some(REPORT_BYTES - 1),
        undefined: None,
    }
}

/// A board from the `-o` options (see the module doc).
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut config = Config::default();
    let number = |k: &str, v: &str| -> Result<u32> {
        v.parse().map_err(|_| Error::Invalid(format!("{k}={v}: a number")))
    };
    let nth = |k: &str, v: &str| -> Result<Option<u32>> {
        v.parse()
            .ok()
            .filter(|n| *n > 0)
            .map(Some)
            .ok_or_else(|| Error::Invalid(format!("{k}={v}: N from 1")))
    };
    let endless = |k: &str, v: &str| -> Result<u32> {
        if v == "never" { Ok(NEVER) } else { number(k, v) }
    };
    for (k, v) in options {
        match k.as_str() {
            "serial" => config.serial = number(k, v)?,
            "fw" => {
                config.firmware = v
                    .split_once('.')
                    .and_then(|(a, b)| Some((a.parse().ok()?, b.parse().ok()?)))
                    .ok_or_else(|| Error::Invalid(format!("fw={v}: MAJOR.MINOR")))?;
            }
            "wait-trigger" => config.wait_trigger = endless(k, v)?,
            "sampling" => config.sampling = number(k, v)?,
            // log_rules reads it.
            "digest" => {}
            "stall-get" => config.stall_get = nth(k, v)?,
            "stall-set" => config.stall_set = nth(k, v)?,
            "short-get" => config.short_get = nth(k, v)?,
            "short-len" => {
                config.short_len = v
                    .parse()
                    .ok()
                    .filter(|n| *n <= REPORT_BYTES)
                    .ok_or_else(|| Error::Invalid(format!("{k}={v}: 0 to {REPORT_BYTES}")))?;
            }
            "stale-sample" => config.stale_sample = number(k, v)?,
            "stale-info" => config.stale_info = v == "1",
            "stale-reset" => config.stale_reset = number(k, v)?,
            "busy-reset" => config.busy_reset = number(k, v)?,
            "ignore-reset" => config.ignore_reset = endless(k, v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(Box::new(Scanalogic2::new(config)))
}

/// The board and its faults, from the options.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// The serial number of the info report.
    pub serial: u32,
    /// The firmware version of the info report.
    pub firmware: (u8, u8),
    /// Status reads that say "waiting for trigger", with a trigger set.
    pub wait_trigger: u32,
    /// Status reads that say "sampling".
    pub sampling: u32,
    /// This feature read (from 1) gets a STALL.
    pub stall_get: Option<u32>,
    /// This feature write (from 1) gets a STALL and is not taken.
    pub stall_set: Option<u32>,
    /// This feature read answers its first 4 bytes.
    pub short_get: Option<u32>,
    /// How many bytes that short answer has.
    pub short_len: usize,
    /// Reads after each `CMD_SAMPLE` that answer the old buffer.
    pub stale_sample: u32,
    /// The read after `CMD_INFO` answers the old buffer.
    pub stale_info: bool,
    /// Reads after the first `CMD_RESET` of a running capture that answer
    /// the old buffer.
    pub stale_reset: u32,
    /// Reads after the first `CMD_RESET` of a running capture that answer
    /// a report of zeros.
    pub busy_reset: u32,
    /// `CMD_RESET`s of a running capture that are not recognized.
    pub ignore_reset: u32,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            serial: SERIAL,
            firmware: FIRMWARE,
            wait_trigger: 0,
            sampling: 0,
            stall_get: None,
            stall_set: None,
            short_get: None,
            short_len: 4,
            stale_sample: 0,
            stale_info: false,
            stale_reset: 0,
            busy_reset: 0,
            ignore_reset: 0,
        }
    }
}

/// What the device is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    /// Status ready.
    Ready,
    /// Idle: no status to give.
    Idle,
    /// Reads left that say "waiting for trigger".
    WaitTrigger(u32),
    /// Reads left that say "sampling".
    Sampling(u32),
    /// The next read says "data ready".
    DataReady,
    /// The next read is this packet of this channel.
    Data { channel: u8, packet: u32 },
}

impl Phase {
    fn capturing(self) -> bool {
        !matches!(self, Phase::Ready | Phase::Idle)
    }
}

/// What a `CMD_SAMPLE` asked for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Capture {
    /// Sample bytes before the trigger.
    pre: u16,
    /// Sample bytes after it.
    post: u16,
    /// The trigger type.
    trigger_type: u8,
    /// The trigger channel code.
    trigger_channel: u8,
}

impl Capture {
    /// Bytes per channel.
    fn bytes(self) -> u32 {
        u32::from(self.pre) + u32::from(self.post)
    }

    /// Packets per channel.
    fn packets(self) -> u32 {
        self.bytes().div_ceil(PACKET_SAMPLE_BYTES as u32)
    }

    /// The sample byte of `channel` at `offset`: a function of the two,
    /// with the trigger's edge at the byte boundary `pre`. A byte's most
    /// significant bit is its first sample.
    fn sample_byte(self, channel: u8, offset: u32) -> u8 {
        if offset >= self.bytes() {
            return 0;
        }
        let mixed = (offset ^ (u32::from(channel) << 16))
            .wrapping_add(u32::from(channel) * 0x51)
            .wrapping_mul(2_654_435_761);
        let byte = (mixed >> 24) as u8;
        if self.trigger_type == TRIGGER_NONE {
            return byte;
        }
        let on_channel = match self.trigger_channel {
            TRIGGER_ALL => channel == 0,
            code => channel + 1 == code,
        };
        if !on_channel {
            return byte;
        }
        let (before, after) = if self.trigger_type == TRIGGER_NEGEDGE { (1, 0) } else { (0, 1) };
        let pre = u32::from(self.pre);
        if offset + 1 == pre {
            (byte & 0xfe) | before
        } else if offset == pre {
            (byte & 0x7f) | (after << 7)
        } else {
            byte
        }
    }

    /// A sample packet: `05 CHANNEL NUMBER 00` and 124 sample bytes.
    fn packet(self, channel: u8, packet: u32) -> [u8; REPORT_BYTES] {
        let mut r = [0; REPORT_BYTES];
        r[0] = STATUS_VALID;
        r[1] = channel;
        r[2] = packet as u8;
        for (i, b) in r[4..].iter_mut().enumerate() {
            *b = self.sample_byte(channel, packet * PACKET_SAMPLE_BYTES as u32 + i as u32);
        }
        r
    }
}

/// The firmware behind the reports.
#[derive(Clone, Debug)]
struct Board {
    config: Config,
    phase: Phase,
    capture: Capture,
    /// The next read is the info report.
    info_pending: bool,
    /// What the last read answered: "the last valid data of the device
    /// buffer".
    buffer: [u8; REPORT_BYTES],
    /// Reads left that answer `buffer`.
    stale_left: u32,
    /// Reads left that answer zeros.
    busy_left: u32,
    /// Resets left to ignore.
    ignore_left: u32,
    /// A `CMD_RESET` of a running capture was recognized.
    capture_reset_seen: bool,
    /// Feature reads and writes so far, for the faults.
    gets: u32,
    sets: u32,
}

impl Board {
    fn new(config: Config) -> Board {
        Board {
            config,
            phase: Phase::Ready,
            capture: Capture::default(),
            info_pending: false,
            buffer: [0; REPORT_BYTES],
            stale_left: 0,
            busy_left: 0,
            ignore_left: config.ignore_reset,
            capture_reset_seen: false,
            gets: 0,
            sets: 0,
        }
    }

    fn status(code: u8) -> [u8; REPORT_BYTES] {
        let mut r = [0; REPORT_BYTES];
        r[0] = STATUS_VALID;
        r[1] = code;
        r
    }

    fn info(&self) -> [u8; REPORT_BYTES] {
        let mut r = [0; REPORT_BYTES];
        r[0] = CMD_INFO;
        r[1..5].copy_from_slice(&self.config.serial.to_le_bytes());
        r[5] = self.config.firmware.0;
        r[6] = self.config.firmware.1;
        r
    }

    /// What a read answers, and the phase it leaves.
    fn read(&mut self, v: &mut Violations) -> [u8; REPORT_BYTES] {
        if self.info_pending {
            self.info_pending = false;
            if !self.config.stale_info {
                self.buffer = self.info();
            }
            return self.buffer;
        }
        if self.busy_left > 0 {
            self.busy_left -= 1;
            return [0; REPORT_BYTES];
        }
        if self.stale_left > 0 {
            self.stale_left -= 1;
            return self.buffer;
        }
        let answer = match self.phase {
            Phase::Idle => {
                v.report("a read in idle state, which has no status to give");
                return self.buffer;
            }
            Phase::Ready => Board::status(STATUS_DEVICE_READY),
            Phase::WaitTrigger(left) => {
                if left != NEVER {
                    self.phase = next_after_trigger(left - 1, self.config.sampling);
                }
                Board::status(STATUS_WAITING_FOR_TRIGGER)
            }
            Phase::Sampling(left) => {
                self.phase = if left > 1 { Phase::Sampling(left - 1) } else { Phase::DataReady };
                Board::status(STATUS_SAMPLING)
            }
            Phase::DataReady => {
                self.phase = if self.capture.packets() == 0 {
                    Phase::Ready
                } else {
                    Phase::Data { channel: 0, packet: 0 }
                };
                Board::status(STATUS_DATA_READY)
            }
            Phase::Data { channel, packet } => {
                let r = self.capture.packet(channel, packet);
                self.phase = if packet + 1 < self.capture.packets() {
                    Phase::Data { channel, packet: packet + 1 }
                } else if channel + 1 < CHANNELS {
                    Phase::Data { channel: channel + 1, packet: 0 }
                } else {
                    Phase::Ready
                };
                r
            }
        };
        self.buffer = answer;
        answer
    }

    fn command(&mut self, data: &[u8], v: &mut Violations) {
        let Some(&command) = data.first() else {
            v.report("an empty command");
            return;
        };
        match command {
            CMD_SAMPLE => self.sample(data, v),
            CMD_RESET => self.reset_command(),
            CMD_IDLE => {
                if self.phase != Phase::Ready {
                    v.report(format!("CMD_IDLE in {:?}, not when ready", self.phase));
                }
                self.phase = Phase::Idle;
                self.info_pending = false;
            }
            CMD_INFO => {
                if self.phase != Phase::Ready {
                    v.report(format!("CMD_INFO in {:?}, not when ready", self.phase));
                }
                self.info_pending = true;
            }
            other => v.report(format!("command {other:#04x} is none of 01, 02, 07, 0a")),
        }
    }

    fn reset_command(&mut self) {
        self.info_pending = false;
        if self.phase.capturing() {
            if self.ignore_left > 0 {
                if self.ignore_left != NEVER {
                    self.ignore_left -= 1;
                }
                return;
            }
            if !self.capture_reset_seen {
                self.capture_reset_seen = true;
                self.stale_left = self.config.stale_reset;
                self.busy_left = self.config.busy_reset;
            }
        }
        self.phase = Phase::Ready;
    }

    fn sample(&mut self, data: &[u8], v: &mut Violations) {
        if data.len() < 12 {
            v.report(format!("CMD_SAMPLE of {} bytes, needs 12", data.len()));
            return;
        }
        if self.phase.capturing() {
            v.report(format!("CMD_SAMPLE in {:?}, during a capture", self.phase));
        }
        if data[1] != 0 {
            v.report(format!("CMD_SAMPLE byte 1 is {:#04x}, not 0", data[1]));
        }
        if data[9] != 0 {
            v.report(format!("CMD_SAMPLE byte 9 is {:#04x}, not 0", data[9]));
        }
        if data[6] > LAST_SAMPLERATE {
            v.report(format!("samplerate code {:#04x}, over {LAST_SAMPLERATE:#04x}", data[6]));
        }
        let trigger_type = data[7];
        let trigger_channel = data[8];
        if trigger_type > TRIGGER_NONE {
            v.report(format!("trigger type {trigger_type:#04x}, over {TRIGGER_NONE:#04x}"));
        }
        if trigger_channel > LAST_TRIGGER_CHANNEL {
            v.report(format!(
                "trigger channel {trigger_channel:#04x}, over {LAST_TRIGGER_CHANNEL:#04x}"
            ));
        }
        if trigger_type != TRIGGER_NONE
            && trigger_channel == TRIGGER_ALL
            && trigger_type != TRIGGER_ANYEDGE
        {
            v.report(format!("trigger on all channels with type {trigger_type:#04x}, not any edge"));
        }
        let delay = u16::from_le_bytes([data[10], data[11]]);
        if delay > MAX_AFTER_TRIGGER_DELAY {
            v.report(format!("after trigger delay {delay} ms, over {MAX_AFTER_TRIGGER_DELAY}"));
        }
        self.capture = Capture {
            pre: u16::from_le_bytes([data[2], data[3]]),
            post: u16::from_le_bytes([data[4], data[5]]),
            trigger_type,
            trigger_channel,
        };
        if self.capture.bytes() > MAX_SAMPLE_BYTES {
            v.report(format!(
                "pre + post of {} bytes, over the {MAX_SAMPLE_BYTES} of the memory",
                self.capture.bytes()
            ));
        }
        self.info_pending = false;
        self.capture_reset_seen = false;
        self.stale_left = self.config.stale_sample;
        self.phase = if trigger_type != TRIGGER_NONE && self.config.wait_trigger > 0 {
            Phase::WaitTrigger(self.config.wait_trigger)
        } else {
            next_after_trigger(0, self.config.sampling)
        };
    }
}

/// The phase after the trigger reads, with `left` of them still to come.
fn next_after_trigger(left: u32, sampling: u32) -> Phase {
    if left > 0 {
        Phase::WaitTrigger(left)
    } else if sampling > 0 {
        Phase::Sampling(sampling)
    } else {
        Phase::DataReady
    }
}

impl HidReports for Board {
    fn get_report(&mut self, kind: u8, id: u8, length: usize, v: &mut Violations) -> Reply {
        if kind != hid::FEATURE {
            v.report(format!("GET_REPORT of kind {kind}; the device has one feature report"));
            return Reply::Stall;
        }
        if id != 0 {
            v.report(format!("GET_REPORT with report id {id}; the device has none"));
        }
        if length != REPORT_BYTES {
            v.report(format!("GET_REPORT of {length} bytes; the report has {REPORT_BYTES}"));
        }
        self.gets += 1;
        if self.config.stall_get == Some(self.gets) {
            return Reply::Stall;
        }
        let answer = self.read(v);
        let short = self.config.short_get == Some(self.gets);
        let n = if short { self.config.short_len } else { REPORT_BYTES };
        Reply::Data(answer[..n].to_vec())
    }

    fn set_report(&mut self, kind: u8, id: u8, data: &[u8], v: &mut Violations) -> Reply {
        if kind != hid::FEATURE {
            v.report(format!("SET_REPORT of kind {kind}; the device has one feature report"));
            return Reply::Stall;
        }
        if id != 0 {
            v.report(format!("SET_REPORT with report id {id}; the device has none"));
        }
        if data.len() != REPORT_BYTES {
            v.report(format!("SET_REPORT of {} bytes; the report has {REPORT_BYTES}", data.len()));
        }
        self.sets += 1;
        if self.config.stall_set == Some(self.sets) {
            return Reply::Stall;
        }
        self.command(data, v);
        Reply::ack()
    }
}

/// The emulated Scanalogic-2.
pub struct Scanalogic2 {
    descriptors: Descriptors,
    hid: HidInterface,
    board: Board,
}

impl Scanalogic2 {
    /// A board with these options.
    pub fn new(config: Config) -> Scanalogic2 {
        // lsusb says HID 1.01.
        let hid = HidInterface::new(INTERFACE, REPORT_DESCRIPTOR.to_vec()).with_version(0x0101);
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(INTERFACE, 0, 1, [3, 0, 0], 0));
        body.extend_from_slice(&hid.descriptor());
        body.extend_from_slice(&descriptor::endpoint(0x81, 3, 8, 100));
        let device = DeviceDescriptor {
            usb: 0x0110,
            class: [0, 0, 0],
            max_packet0: 8,
            vendor: VENDOR_ID,
            product: PRODUCT_ID,
            device: 0x0100,
            strings: [1, 2, 0],
            configurations: 1,
        };
        Scanalogic2 {
            descriptors: Descriptors {
                speed: Speed::Low,
                device: device.bytes(),
                configs: vec![descriptor::configuration(1, 0, 0x80, 100, &body)],
                strings: vec!["ikalogic".to_owned(), "SCANALOGIC".to_owned()],
            },
            hid,
            board: Board::new(config),
        }
    }
}

impl DeviceModel for Scanalogic2 {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if let Some(reply) = self.hid.control(&mut self.board, setup, data, v) {
            return reply;
        }
        v.report(format!(
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the device answers",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!("OUT transfer to endpoint {ep}; the device has none"));
        Reply::Stall
    }

    fn in_transfer(&mut self, _ep: u8, _max: usize, _v: &mut Violations) -> Reply {
        // Nothing says the interrupt endpoint ever sends; a host's poll waits.
        Reply::Nak
    }

    fn reset(&mut self) {
        self.hid.reset();
        self.board = Board::new(self.board.config);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GET: Setup =
        Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 128 };
    const SET: Setup =
        Setup { request_type: 0x21, request: 9, value: 0x0300, index: 0, length: 128 };

    fn board(config: Config) -> Scanalogic2 {
        Scanalogic2::new(config)
    }

    fn cmd(command: u8) -> [u8; 128] {
        let mut b = [0; 128];
        b[0] = command;
        b
    }

    /// `CMD_SAMPLE` as `dev_acquisition_start()` builds it.
    fn sample(pre: u16, post: u16, rate: u8, kind: u8, channel: u8) -> [u8; 128] {
        let mut b = cmd(CMD_SAMPLE);
        b[2..4].copy_from_slice(&pre.to_le_bytes());
        b[4..6].copy_from_slice(&post.to_le_bytes());
        b[6] = rate;
        b[7] = kind;
        b[8] = channel;
        b
    }

    fn write(dev: &mut Scanalogic2, data: &[u8], v: &mut Violations) -> Reply {
        dev.control(SET, data, v)
    }

    fn read(dev: &mut Scanalogic2, v: &mut Violations) -> Vec<u8> {
        match dev.control(GET, &[], v) {
            Reply::Data(d) => d,
            other => panic!("{other:?}"),
        }
    }

    fn status(dev: &mut Scanalogic2, v: &mut Violations) -> [u8; 2] {
        let r = read(dev, v);
        [r[0], r[1]]
    }

    fn open(dev: &mut Scanalogic2, v: &mut Violations) {
        write(dev, &cmd(CMD_RESET), v);
        write(dev, &cmd(CMD_IDLE), v);
    }

    #[test]
    fn the_descriptors_are_lsusbs() {
        let dev = board(Config::default());
        let d = dev.descriptors();
        assert_eq!(d.speed, Speed::Low);
        assert_eq!((d.vendor_id(), d.product_id(), d.bcd_device()), (0x20a0, 0x4123, 0x0100));
        assert_eq!(&d.device[2..4], &[0x10, 0x01], "bcdUSB 1.10");
        assert_eq!(d.device[7], 8, "bMaxPacketSize0");
        assert_eq!(d.device[16], 0, "iSerial");
        let config = d.config_by_value(1).unwrap();
        assert_eq!(config.len(), 34, "wTotalLength");
        assert_eq!(config[7], 0x80, "bus powered");
        assert_eq!(config[8], 50, "100 mA");
        let ep = Descriptors::endpoints(config)[0];
        assert_eq!((ep.address, ep.max_packet, ep.interval), (0x81, 8, 100));
        assert_eq!(REPORT_DESCRIPTOR.len(), 22);
        assert_eq!(config[18..27], [9, 0x21, 0x01, 0x01, 0, 1, 0x22, 22, 0], "bcdHID 1.01");
        assert_eq!(d.strings, ["ikalogic", "SCANALOGIC"]);
    }

    #[test]
    fn answers_the_hid_descriptors() {
        let mut dev = board(Config::default());
        let mut v = Violations::default();
        let s = Setup { request_type: 0x81, request: 6, value: 0x2200, index: 0, length: 255 };
        assert_eq!(dev.control(s, &[], &mut v), Reply::Data(REPORT_DESCRIPTOR.to_vec()));
        let s = Setup { request_type: 0x81, request: 6, value: 0x2100, index: 0, length: 255 };
        assert_eq!(
            dev.control(s, &[], &mut v),
            Reply::Data(vec![9, 0x21, 0x01, 0x01, 0, 1, 0x22, 22, 0])
        );
        assert_eq!(dev.in_transfer(1, 8, &mut v), Reply::Nak);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_a_hid_descriptor_request_that_is_not_one() {
        let mut v = Violations::default();
        let mut dev = board(Config::default());
        let good = Setup { request_type: 0x81, request: 6, value: 0x2100, index: 0, length: 255 };
        for (bad, what) in [
            (Setup { request_type: 0x80, ..good }, "to the device"),
            (Setup { index: 1, ..good }, "another interface"),
            (Setup { request: 7, ..good }, "not GET_DESCRIPTOR"),
            (Setup { request_type: 0x01, ..good }, "OUT"),
        ] {
            assert_eq!(dev.control(bad, &[], &mut v), Reply::Stall, "{what}");
            assert_eq!(v.drain().len(), 1, "{what}");
        }
    }

    #[test]
    fn scan_reads_the_info_report() {
        let mut dev = board(Config::default());
        let mut v = Violations::default();
        write(&mut dev, &cmd(CMD_RESET), &mut v);
        write(&mut dev, &cmd(CMD_INFO), &mut v);
        let info = read(&mut dev, &mut v);
        assert_eq!(info.len(), 128);
        // The wiki's example bytes.
        assert_eq!(info[..7], [0x0a, 0x90, 0x76, 0xbd, 0x51, 0x01, 0x03]);
        // The driver reuses the buffer, so the info bytes follow the command.
        let mut again = info.clone();
        again[0] = CMD_RESET;
        write(&mut dev, &again, &mut v);
        again[0] = CMD_IDLE;
        write(&mut dev, &again, &mut v);
        assert!(v.is_empty(), "{v:?}");
    }

    /// Reads a whole capture of `bytes` per channel, checking the headers.
    fn read_all(dev: &mut Scanalogic2, bytes: u32, v: &mut Violations) {
        assert_eq!(status(dev, v), [5, 0x60]);
        let packets = bytes.div_ceil(124);
        for channel in 0..4u8 {
            for packet in 0..packets {
                let r = read(dev, v);
                assert_eq!(r.len(), 128);
                assert_eq!(r[..4], [5, channel, packet as u8, 0], "{channel} {packet}");
            }
        }
    }

    #[test]
    fn a_capture_from_idle_sends_each_channel() {
        let mut dev = board(Config::default());
        let mut v = Violations::default();
        open(&mut dev, &mut v);
        // 1000 bytes: nine packets, with four sample bytes in the last.
        write(&mut dev, &sample(0, 1000, 0x0a, TRIGGER_NONE, 1), &mut v);
        read_all(&mut dev, 1000, &mut v);
        // The extra read of the driver, then the one that ends it.
        assert_eq!(status(&mut dev, &mut v), [5, 0x63]);
        assert_eq!(status(&mut dev, &mut v), [5, 0x63]);
        write(&mut dev, &cmd(CMD_IDLE), &mut v);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn packet_number_wraps_and_the_last_packet_ends_in_zeros() {
        let c = Capture { pre: 0, post: 32766, trigger_type: TRIGGER_NONE, trigger_channel: 1 };
        assert_eq!(c.packets(), 265);
        assert_eq!(c.packet(2, 256)[2], 0, "wraps after 256");
        assert_eq!(c.packet(2, 255)[2], 255);
        let last = c.packet(0, 264);
        // 32736 sample bytes come before the last packet; 30 are its own.
        assert_ne!(last[4..34], [0; 30]);
        assert_eq!(last[34..], [0; 94]);
    }

    #[test]
    fn channels_differ() {
        let c = Capture { pre: 0, post: 124, trigger_type: TRIGGER_NONE, trigger_channel: 1 };
        for a in 0..4 {
            for b in 0..a {
                assert_ne!(c.packet(a, 0)[4..], c.packet(b, 0)[4..], "{a} {b}");
            }
        }
    }

    #[test]
    fn the_trigger_edge_is_at_the_byte_boundary() {
        let mut c = Capture { pre: 2, post: 4, trigger_type: 1, trigger_channel: 3 };
        // Rising on channel 2 (code 3): low before, high after.
        assert_eq!(c.sample_byte(2, 1) & 1, 0);
        assert_eq!(c.sample_byte(2, 2) & 0x80, 0x80);
        c.trigger_type = TRIGGER_NEGEDGE;
        assert_eq!(c.sample_byte(2, 1) & 1, 1);
        assert_eq!(c.sample_byte(2, 2) & 0x80, 0);
        // All channels: on channel 0, any edge as rising.
        c = Capture { pre: 1, post: 1, trigger_type: TRIGGER_ANYEDGE, trigger_channel: 0 };
        assert_eq!(c.sample_byte(0, 0) & 1, 0);
        assert_eq!(c.sample_byte(0, 1) & 0x80, 0x80);
        let plain = Capture { trigger_type: TRIGGER_NONE, ..c };
        assert_eq!(c.sample_byte(1, 0), plain.sample_byte(1, 0), "other channels keep theirs");
    }

    #[test]
    fn wait_trigger_and_sampling_are_counted_in_reads() {
        let config = Config { wait_trigger: 2, sampling: 2, ..Config::default() };
        let mut dev = board(config);
        let mut v = Violations::default();
        open(&mut dev, &mut v);
        write(&mut dev, &sample(1, 1, 0, 1, 1), &mut v);
        for want in [0x61, 0x61, 0x62, 0x62, 0x60] {
            assert_eq!(status(&mut dev, &mut v), [5, want]);
        }
        // No trigger: no wait for one.
        let mut dev = board(config);
        write(&mut dev, &sample(0, 1, 0, TRIGGER_NONE, 1), &mut v);
        for want in [0x62, 0x62, 0x60] {
            assert_eq!(status(&mut dev, &mut v), [5, want]);
        }
        let mut dev = board(Config { wait_trigger: NEVER, ..Config::default() });
        write(&mut dev, &sample(1, 1, 0, 1, 1), &mut v);
        for _ in 0..4 {
            assert_eq!(status(&mut dev, &mut v), [5, 0x61]);
        }
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_reset_stops_a_capture() {
        let mut dev = board(Config { wait_trigger: NEVER, ..Config::default() });
        let mut v = Violations::default();
        write(&mut dev, &sample(1, 1, 0, 1, 1), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0x61]);
        write(&mut dev, &cmd(CMD_RESET), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0x63]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn stale_sample_answers_the_old_buffer() {
        let mut dev = board(Config { stale_sample: 2, ..Config::default() });
        let mut v = Violations::default();
        write(&mut dev, &cmd(CMD_INFO), &mut v);
        assert_eq!(read(&mut dev, &mut v)[0], 0x0a);
        write(&mut dev, &cmd(CMD_RESET), &mut v);
        write(&mut dev, &cmd(CMD_IDLE), &mut v);
        write(&mut dev, &sample(0, 8, 0, TRIGGER_NONE, 1), &mut v);
        assert_eq!(read(&mut dev, &mut v)[0], 0x0a);
        assert_eq!(read(&mut dev, &mut v)[0], 0x0a);
        assert_eq!(status(&mut dev, &mut v), [5, 0x60]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn stale_info_is_not_the_info_report() {
        let mut dev = board(Config { stale_info: true, ..Config::default() });
        let mut v = Violations::default();
        write(&mut dev, &cmd(CMD_INFO), &mut v);
        assert_eq!(read(&mut dev, &mut v), vec![0; 128]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn stale_reset_answers_the_last_packet_once_per_capture() {
        let mut dev = board(Config { stale_reset: 1, ..Config::default() });
        let mut v = Violations::default();
        // A reset outside a capture does not count.
        write(&mut dev, &cmd(CMD_RESET), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0x63]);
        write(&mut dev, &sample(0, 8, 0, TRIGGER_NONE, 1), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0x60]);
        assert_eq!(status(&mut dev, &mut v), [5, 0]);
        write(&mut dev, &cmd(CMD_RESET), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0], "the old buffer");
        write(&mut dev, &cmd(CMD_RESET), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0x63], "not armed again");
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn busy_reset_answers_zeros() {
        let mut dev = board(Config { busy_reset: 2, ..Config::default() });
        let mut v = Violations::default();
        write(&mut dev, &sample(0, 8, 0, TRIGGER_NONE, 1), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0x60]);
        write(&mut dev, &cmd(CMD_RESET), &mut v);
        assert_eq!(status(&mut dev, &mut v), [0, 0]);
        assert_eq!(status(&mut dev, &mut v), [0, 0]);
        assert_eq!(status(&mut dev, &mut v), [5, 0x63]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn ignore_reset_leaves_the_capture_running() {
        let mut dev = board(Config { ignore_reset: 1, ..Config::default() });
        let mut v = Violations::default();
        // A reset outside a capture is not one to ignore.
        write(&mut dev, &cmd(CMD_RESET), &mut v);
        write(&mut dev, &sample(0, 8, 0, TRIGGER_NONE, 1), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0x60]);
        write(&mut dev, &cmd(CMD_RESET), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0], "still sending");
        write(&mut dev, &cmd(CMD_RESET), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0x63]);
        let mut dev =
            board(Config { ignore_reset: NEVER, wait_trigger: NEVER, ..Config::default() });
        write(&mut dev, &sample(1, 1, 0, 1, 1), &mut v);
        for _ in 0..4 {
            write(&mut dev, &cmd(CMD_RESET), &mut v);
            assert_eq!(status(&mut dev, &mut v), [5, 0x61]);
        }
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_capture_of_nothing_goes_straight_to_ready() {
        let mut dev = board(Config::default());
        let mut v = Violations::default();
        write(&mut dev, &sample(0, 0, 0, TRIGGER_NONE, 1), &mut v);
        assert_eq!(status(&mut dev, &mut v), [5, 0x60]);
        assert_eq!(status(&mut dev, &mut v), [5, 0x63]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_a_bad_capture() {
        let mut v = Violations::default();
        let mut one = |data: [u8; 128], what: &str| {
            let mut dev = board(Config::default());
            write(&mut dev, &data, &mut v);
            assert_eq!(v.drain().len(), 1, "{what}");
        };
        let mut b = sample(0, 8, 0, TRIGGER_NONE, 1);
        b[1] = 1;
        one(b, "byte 1");
        let mut b = sample(0, 8, 0, TRIGGER_NONE, 1);
        b[9] = 1;
        one(b, "byte 9");
        one(sample(0, 8, 0x0b, TRIGGER_NONE, 1), "samplerate code");
        one(sample(0, 8, 0, 4, 1), "trigger type");
        one(sample(0, 8, 0, 1, 5), "trigger channel");
        one(sample(0, 8, 0, 1, TRIGGER_ALL), "rising on all channels");
        one(sample(0, 8, 0, TRIGGER_NEGEDGE, TRIGGER_ALL), "falling on all channels");
        let mut b = sample(0, 8, 0, TRIGGER_NONE, 1);
        b[10..12].copy_from_slice(&65001u16.to_le_bytes());
        one(b, "delay");
        one(sample(16384, 16383, 0, 1, 1), "over the memory");
        let mut v = Violations::default();
        let mut dev = board(Config::default());
        write(&mut dev, &sample(16383, 16383, 0, 1, 1), &mut v);
        assert!(v.is_empty(), "the whole memory, split: {v:?}");
        let mut dev = board(Config::default());
        write(&mut dev, &sample(0, 8, 0, TRIGGER_ANYEDGE, TRIGGER_ALL), &mut v);
        assert!(v.is_empty(), "any edge on all channels is fine: {v:?}");
        write(&mut dev, &sample(0, 8, 0, TRIGGER_NONE, 1), &mut v);
        assert_eq!(v.drain().len(), 1, "a capture during a capture");
        let mut dev = board(Config::default());
        write(&mut dev, &cmd(CMD_SAMPLE)[..4], &mut v);
        // wLength against the data, the report's length, the fields' length.
        assert_eq!(v.drain().len(), 3, "short CMD_SAMPLE");
    }

    #[test]
    fn flags_commands_out_of_order() {
        let mut v = Violations::default();
        let mut dev = board(Config::default());
        write(&mut dev, &cmd(0x03), &mut v);
        assert_eq!(v.drain().len(), 1, "unknown command");
        open(&mut dev, &mut v);
        assert!(v.is_empty(), "{v:?}");
        let before = read(&mut dev, &mut v);
        assert_eq!(v.drain().len(), 1, "a read in idle");
        assert_eq!(before, vec![0; 128], "the old buffer");
        write(&mut dev, &cmd(CMD_INFO), &mut v);
        assert_eq!(v.drain().len(), 1, "info in idle");
        let mut dev = board(Config { wait_trigger: 3, ..Config::default() });
        write(&mut dev, &sample(0, 8, 0, 1, 1), &mut v);
        write(&mut dev, &cmd(CMD_INFO), &mut v);
        assert_eq!(v.drain().len(), 1, "info during a capture");
        let mut dev = board(Config { wait_trigger: 3, ..Config::default() });
        write(&mut dev, &sample(0, 8, 0, 1, 1), &mut v);
        write(&mut dev, &cmd(CMD_IDLE), &mut v);
        assert_eq!(v.drain().len(), 1, "idle during a capture");
    }

    #[test]
    fn flags_other_reports() {
        let mut v = Violations::default();
        let mut dev = board(Config::default());
        // An input report: the device has none.
        let s = Setup { value: 0x0100, ..GET };
        assert_eq!(dev.control(s, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1);
        let s = Setup { value: 0x0200, ..SET };
        assert_eq!(dev.control(s, &cmd(CMD_RESET), &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1);
        let s = Setup { value: 0x0301, ..GET };
        dev.control(s, &[], &mut v);
        assert_eq!(v.drain().len(), 1, "report id on a read");
        let s = Setup { value: 0x0301, ..SET };
        dev.control(s, &cmd(CMD_RESET), &mut v);
        assert_eq!(v.drain().len(), 1, "report id on a write");
        let s = Setup { length: 64, ..GET };
        dev.control(s, &[], &mut v);
        assert_eq!(v.drain().len(), 1, "a short read");
        let s = Setup { length: 2, ..SET };
        dev.control(s, &[CMD_RESET, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "a short write");
        let s = Setup { length: 0, ..SET };
        dev.control(s, &[], &mut v);
        // Short, and no command in it.
        assert_eq!(v.drain().len(), 2, "an empty write");
        let s = Setup { request_type: 0xc0, request: 0x55, value: 0, index: 0, length: 4 };
        assert_eq!(dev.control(s, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "a vendor request");
        assert_eq!(dev.out(1, &[0], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "no OUT endpoint");
    }

    #[test]
    fn the_stall_and_short_faults_are_counted() {
        let mut v = Violations::default();
        let mut dev = board(Config { stall_set: Some(2), ..Config::default() });
        assert_eq!(write(&mut dev, &cmd(CMD_RESET), &mut v), Reply::ack());
        assert_eq!(write(&mut dev, &cmd(CMD_IDLE), &mut v), Reply::Stall);
        assert_eq!(status(&mut dev, &mut v), [5, 0x63], "not taken");
        assert_eq!(write(&mut dev, &cmd(CMD_RESET), &mut v), Reply::ack());
        let mut dev = board(Config { stall_get: Some(2), ..Config::default() });
        read(&mut dev, &mut v);
        assert_eq!(dev.control(GET, &[], &mut v), Reply::Stall);
        read(&mut dev, &mut v);
        let mut dev = board(Config { short_get: Some(1), ..Config::default() });
        assert_eq!(read(&mut dev, &mut v).len(), 4);
        assert_eq!(read(&mut dev, &mut v).len(), 128);
        let mut dev = board(Config { short_get: Some(1), short_len: 0, ..Config::default() });
        assert_eq!(read(&mut dev, &mut v).len(), 0);
        assert_eq!(read(&mut dev, &mut v).len(), 128);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_bus_reset_starts_over_with_the_options() {
        let mut dev = board(Config { stall_get: Some(1), ..Config::default() });
        let mut v = Violations::default();
        assert_eq!(dev.control(GET, &[], &mut v), Reply::Stall);
        write(&mut dev, &cmd(CMD_IDLE), &mut v);
        dev.reset();
        assert_eq!(dev.control(GET, &[], &mut v), Reply::Stall, "counted again");
        assert_eq!(status(&mut dev, &mut v), [5, 0x63], "out of idle");
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn options_parse() {
        let o = |k: &str, val: &str| vec![(k.to_owned(), val.to_owned())];
        let mut v = Violations::default();
        for bad in [
            ("serial", "x"),
            ("fw", "1"),
            ("fw", "a.b"),
            ("wait-trigger", "x"),
            ("sampling", "x"),
            ("stall-get", "0"),
            ("stall-set", "x"),
            ("short-get", "0"),
            ("short-len", "129"),
            ("short-len", "x"),
            ("stale-sample", "x"),
            ("stale-reset", "x"),
            ("busy-reset", "x"),
            ("ignore-reset", "x"),
            ("nothing", "1"),
        ] {
            assert!(build(&o(bad.0, bad.1)).is_err(), "{bad:?}");
        }
        let mut dev = build(&[
            ("serial".to_owned(), "258".to_owned()),
            ("fw".to_owned(), "2.7".to_owned()),
        ])
        .unwrap();
        dev.control(SET, &cmd(CMD_INFO), &mut v);
        let Reply::Data(info) = dev.control(GET, &[], &mut v) else { panic!() };
        assert_eq!(info[..7], [0x0a, 2, 1, 0, 0, 2, 7]);
        assert_eq!(log_rules(&o("digest", "1")).digest_above, Some(127));
        assert_eq!(log_rules(&[]).digest_above, None);
        assert!(build(&o("ignore-reset", "never")).is_ok());
        assert!(build(&o("wait-trigger", "never")).is_ok());
        assert!(build(&o("stale-info", "1")).is_ok());
        assert!(v.is_empty(), "{v:?}");
    }
}
