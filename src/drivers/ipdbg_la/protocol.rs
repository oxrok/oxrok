// Counterpart in sigrok:
//   libsigrok/src/hardware/ipdbg-la/api.c,
//   libsigrok/src/hardware/ipdbg-la/protocol.c,
//   libsigrok/src/hardware/ipdbg-la/protocol.h,
//   Copyright (C) 2016 Eva Kissling <eva.kissling@bluewin.ch>,
//   with commits by Uwe Hermann, Daniel Anselmi, danselmi
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The IPDBG logic analyzer core's wire, with nothing that does I/O.
//!
//! Bytes to the core: `0xEE` resets it, and `0x55` makes the next byte data
//! (`rtl/common/IpdbgEscaping.vhd`), so a field's `0xEE` and `0x55` are escaped and a
//! command never needs it.
//!
//! | Command | What follows |
//! |---------|--------------|
//! | `EE` | a reset, of the state and any dump |
//! | `BB` | the answer `IDBG` (`idbg` with run length compression, which this has not) |
//! | `AA` | the answer: data width, address width, 32 bits each, low byte first |
//! | `F0 F1 F3` | the mask of the sample, most significant byte first |
//! | `F0 F1 F7` | its value |
//! | `F0 F9 FB` | the mask of the sample before |
//! | `F0 F9 FF` | its value |
//! | `F0 F5 F6` | the mask of the channels on either edge |
//! | `0F 1F` | the samples kept ahead of the trigger, in the bytes of the address |
//! | `FE` | arms; when the memory has filled, all of it comes |
//!
//! A dump is 2^address samples, oldest first, each low byte first, whatever limit was
//! set, and it holds no framing. Its trigger is sample number `delay`. The core can't
//! say how long that takes: its samples come on a signal of the design under test.

use crate::core::{Trigger, TriggerMatch};
use std::time::{Duration, Instant};

/// A reset.
pub const CMD_RESET: u8 = 0xEE;
/// The id's request.
pub const CMD_GET_LA_ID: u8 = 0xBB;
/// The widths' request.
pub const CMD_GET_BUS_WIDTHS: u8 = 0xAA;
/// Arms the core.
pub const CMD_START: u8 = 0xFE;
const CMD_ESCAPE: u8 = 0x55;
const CMD_CFG_TRIGGER: u8 = 0xF0;
const CMD_CFG_LA: u8 = 0x0F;
const CMD_TRIG_MASKS: u8 = 0xF1;
const CMD_TRIG_MASK: u8 = 0xF3;
const CMD_TRIG_VALUE: u8 = 0xF7;
const CMD_TRIG_MASKS_LAST: u8 = 0xF9;
const CMD_TRIG_MASK_LAST: u8 = 0xFB;
const CMD_TRIG_VALUE_LAST: u8 = 0xFF;
const CMD_TRIG_SELECT_EDGE_MASK: u8 = 0xF5;
const CMD_TRIG_SET_EDGE_MASK: u8 = 0xF6;
const CMD_LA_DELAY: u8 = 0x1F;

/// The id a core without run length compression answers.
pub const ID: &[u8; 4] = b"IDBG";
/// `tcp_receive_blocking`'s 2000 tries of a millisecond each: "Timeout after 2s of not
/// receiving data". By the clock here: a socket's 1 ms read waits about 4 ms on Linux,
/// so a count of them took some 8 s, and an id at 3 s passed that C doesn't take.
pub const RECEIVE_WAIT: Duration = Duration::from_secs(2);

/// The widest address taken: `LogicAnalyserMemory.vhd`'s `2**ADDR_WIDTH` is a VHDL
/// integer, so no core has 31 or more. C's `0x01 << addr_width` (`protocol.c:505`) is
/// an `int`, so 31 is 18446744071562067968 samples and 32 is 1, on x86.
pub const MAX_ADDR_WIDTH: u32 = 30;
/// The widest sample taken: an inference, as the HDL has no limit. Each channel is
/// an allocation, from what a device said, and C's `scan` makes 2^32 of them for
/// garbage. A core takes block RAM per bit, so this is far above any there can be.
pub const MAX_DATA_WIDTH: u32 = 1 << 16;

/// What the sizes' answer says, or why it is no device.
///
/// # Errors
///
/// The message for a width past [`MAX_DATA_WIDTH`] or [`MAX_ADDR_WIDTH`].
pub fn parse_widths(answer: &[u8; 8]) -> Result<(u32, u32), String> {
    let data = u32::from_le_bytes([answer[0], answer[1], answer[2], answer[3]]);
    let addr = u32::from_le_bytes([answer[4], answer[5], answer[6], answer[7]]);
    if data > MAX_DATA_WIDTH {
        return Err(format!(
            "Device reports {data} channels, more than the {MAX_DATA_WIDTH} here."
        ));
    }
    if addr > MAX_ADDR_WIDTH {
        return Err(format!(
            "Device reports {addr} address bits, more than the {MAX_ADDR_WIDTH} a core can have."
        ));
    }
    Ok((data, addr))
}

/// The host and port of `conn`: `ipdbg_la_split_addr_port`, a `g_strsplit(conn, "/", 3)`
/// whose first field nobody looks at. `None` for no second field, where C reads past
/// the end of its list (`api.c:51`) and crashes, as `conn=tcp-raw` does.
pub fn split_conn(conn: &str) -> Option<(&str, Option<&str>)> {
    let mut fields = conn.splitn(3, '/');
    fields.next()?;
    let host = fields.next()?;
    Some((host, fields.next()))
}

/// The five trigger fields, least significant byte first: `ipdbg_la_convert_trigger`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TriggerFields {
    /// Which channels the sample's value counts for.
    pub mask: Vec<u8>,
    /// The sample's value.
    pub value: Vec<u8>,
    /// Which channels the last sample's value counts for.
    pub mask_last: Vec<u8>,
    /// The last sample's value.
    pub value_last: Vec<u8>,
    /// The channels that must change.
    pub edge_mask: Vec<u8>,
}

/// `ipdbg_la_convert_trigger`: every stage's conditions in one set of fields, the
/// disabled channels' left out, and a later condition on a channel changing what an
/// earlier one set as C's statements do. `enabled` says of a channel index.
pub fn convert_trigger(
    trigger: Option<&Trigger>,
    data_bytes: usize,
    enabled: impl Fn(usize) -> bool,
) -> TriggerFields {
    let mut f = TriggerFields {
        mask: vec![0; data_bytes],
        value: vec![0; data_bytes],
        mask_last: vec![0; data_bytes],
        value_last: vec![0; data_bytes],
        edge_mask: vec![0; data_bytes],
    };
    let Some(trigger) = trigger else {
        return f;
    };
    for condition in trigger.stages.iter().flat_map(|s| &s.conditions) {
        let i = condition.channel / 8;
        let bit = 1u8 << (condition.channel % 8);
        if !enabled(condition.channel) || i >= data_bytes {
            continue;
        }
        match condition.matches {
            TriggerMatch::One => {
                f.value[i] |= bit;
                f.mask[i] |= bit;
                f.mask_last[i] &= !bit;
                f.edge_mask[i] &= !bit;
            }
            TriggerMatch::Zero => {
                f.value[i] &= !bit;
                f.mask[i] |= bit;
                f.mask_last[i] &= !bit;
                f.edge_mask[i] &= !bit;
            }
            TriggerMatch::Rising => {
                f.value[i] |= bit;
                f.value_last[i] &= !bit;
                f.mask[i] |= bit;
                f.mask_last[i] |= bit;
                f.edge_mask[i] &= !bit;
            }
            TriggerMatch::Falling => {
                f.value[i] &= !bit;
                f.value_last[i] |= bit;
                f.mask[i] |= bit;
                f.mask_last[i] |= bit;
                f.edge_mask[i] &= !bit;
            }
            TriggerMatch::Edge => {
                f.mask[i] &= !bit;
                f.mask_last[i] &= !bit;
                f.edge_mask[i] |= bit;
            }
            // Not in the list this driver gives, and the CLI checks against that.
            TriggerMatch::Over | TriggerMatch::Under => {}
        }
    }
    f
}

/// `ipdbg_la_send_delay`'s arithmetic, in `f64` as C has it, so a rounding comes out
/// the same: 150 samples at 33 % is 49. What is sent can't pass the limit.
pub fn delay_value(limit_samples: u64, capture_ratio: u64) -> u64 {
    let raw = (limit_samples.wrapping_sub(1) as f64 / 100.0) * capture_ratio as f64;
    (raw as u64).min(limit_samples.saturating_sub(1))
}

/// A field's bytes, most significant first, with what needs it escaped:
/// `send_escaping` over C's loop from the top.
fn push_field(out: &mut Vec<u8>, low_first: &[u8]) {
    for &b in low_first.iter().rev() {
        if b == CMD_RESET || b == CMD_ESCAPE {
            out.push(CMD_ESCAPE);
        }
        out.push(b);
    }
}

/// `ipdbg_la_send_trigger` and `ipdbg_la_send_delay`, whose bytes C sends one at a time.
pub fn configuration(fields: &TriggerFields, delay: u64, addr_bytes: usize) -> Vec<u8> {
    let mut out = Vec::new();
    for (select, set, field) in [
        (CMD_TRIG_MASKS, CMD_TRIG_MASK, &fields.mask),
        (CMD_TRIG_MASKS, CMD_TRIG_VALUE, &fields.value),
        (CMD_TRIG_MASKS_LAST, CMD_TRIG_MASK_LAST, &fields.mask_last),
        (CMD_TRIG_MASKS_LAST, CMD_TRIG_VALUE_LAST, &fields.value_last),
        (CMD_TRIG_SELECT_EDGE_MASK, CMD_TRIG_SET_EDGE_MASK, &fields.edge_mask),
    ] {
        out.extend_from_slice(&[CMD_CFG_TRIGGER, select, set]);
        push_field(&mut out, field);
    }
    out.extend_from_slice(&[CMD_CFG_LA, CMD_LA_DELAY]);
    let low_first = delay.to_le_bytes();
    push_field(&mut out, &low_first[..addr_bytes.min(low_first.len())]);
    out
}

/// How long the wait may go quiet before a line says so.
pub const SILENCE_WARNING: Duration = Duration::from_secs(30);
/// How long a dump that has begun may go quiet before the capture fails. A dump goes
/// over a server's poll of the JTAG hub, so a gap this long is no pace of a core's.
pub const SILENCE_LIMIT: Duration = Duration::from_secs(600);
/// A gap between two looks past this is the thread that didn't run.
const STALL: Duration = Duration::from_secs(1);
/// The most of that credit, or a process stopped again and again never gets there.
const STALL_CREDIT: Duration = Duration::from_secs(60);

/// What [`Silence::empty`] says of a read that found nothing.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Nothing to say.
    Wait,
    /// A warning, once for each quiet spell.
    Warn(String),
    /// The capture fails.
    Fail(String),
}

/// The rule on a capture's silence, over times given to it.
///
/// Before the first byte there is no bound, only a warning. The core is waiting for
/// its trigger and the design's sample signal, of which nothing is known, and a
/// trigger that doesn't come isn't an error: C waits too, and a stop ends it. Inside
/// a dump the core's memory is only being read, so a long silence is a fault, and
/// C's wait for ever isn't kept: a warning, then a failure at [`SILENCE_LIMIT`].
#[derive(Clone, Debug)]
pub struct Silence {
    /// When the last byte came, or the capture began.
    last: Instant,
    /// When the last read began, for the bound and for a stall.
    looked: Instant,
    /// What the thread didn't run for, capped.
    credit: Duration,
    warned: bool,
    received: u64,
    total: u64,
}

impl Silence {
    /// A capture's wait from `now`, for `total` bytes of dump.
    pub fn new(now: Instant, total: u64) -> Silence {
        Silence {
            last: now,
            looked: now,
            credit: Duration::ZERO,
            warned: false,
            received: 0,
            total,
        }
    }

    /// To be called ahead of each read: the time a read that finds nothing goes by.
    pub fn look(&mut self, now: Instant) {
        let gap = now.saturating_duration_since(self.looked);
        if gap > STALL {
            self.credit = (self.credit + gap).min(STALL_CREDIT);
        }
        self.looked = now;
    }

    /// `n` bytes came. A line if a warning went out for this quiet spell.
    pub fn got(&mut self, now: Instant, n: u64) -> Option<String> {
        self.received = self.received.saturating_add(n);
        let back = self.warned.then(|| {
            format!(
                "Data again after {} s, at {} of {} bytes.",
                now.saturating_duration_since(self.last).as_secs(),
                self.received,
                self.total
            )
        });
        self.last = now;
        self.credit = Duration::ZERO;
        self.warned = false;
        back
    }

    /// A read found nothing; the times are those of the last [`Silence::look`].
    pub fn empty(&mut self) -> Verdict {
        let quiet = self.looked.saturating_duration_since(self.last).saturating_sub(self.credit);
        if self.received > 0 && quiet >= SILENCE_LIMIT {
            return Verdict::Fail(format!(
                "No data for {} s inside a dump, at {} of {} bytes.",
                SILENCE_LIMIT.as_secs(),
                self.received,
                self.total
            ));
        }
        if self.warned || quiet < SILENCE_WARNING {
            return Verdict::Wait;
        }
        self.warned = true;
        Verdict::Warn(if self.received > 0 {
            format!(
                "No data for {} s inside a dump, at {} of {} bytes: still waiting, for up to {} s.",
                SILENCE_WARNING.as_secs(),
                self.received,
                self.total,
                SILENCE_LIMIT.as_secs()
            )
        } else {
            format!(
                "No data for {} s: the core is waiting for its trigger or its sample signal. \
                 A stop ends this.",
                SILENCE_WARNING.as_secs()
            )
        })
    }
}
