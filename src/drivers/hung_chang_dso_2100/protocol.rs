// Counterpart in sigrok:
//   libsigrok/src/hardware/hung-chang-dso-2100/protocol.c,
//   libsigrok/src/hardware/hung-chang-dso-2100/protocol.h,
//   Copyright (C) 2015 Daniel Glöckner <daniel-gl@gmx.net>,
//   with commits by Uwe Hermann, Daniel Elstner, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The DSO-2100's parallel port protocol.
//!
//! The scope has a microcontroller behind a one byte mailbox and an FPGA
//! that hands out the capture a sample per nStrobe pulse. The lines say
//! which one the port talks to (libieee1284's bits, a set bit a high
//! line):
//!
//! - id: nSelectIn and nAutoFd high, nInit low, nStrobe pulled low: the
//!   scope drives `0x55`.
//! - mailbox write: nInit and nSelectIn high, nAutoFd low, the data lines
//!   driven, an nStrobe pulse.
//! - mailbox read: nSelectIn high, the rest low: the microcontroller's
//!   state, or `0` while it has none to say. A first read may be unstable,
//!   so a value is the read after the first that is not `0`.
//! - readout: nSelectIn and nInit low, nAutoFd picking the ADC (high the
//!   second): each nStrobe pulse a sample, 1000 of them, then the
//!   signature `01 fe 80`.
//!
//! The microcontroller's states and how one moves between them are in the
//! C driver's comment (`protocol.c`): idle `0x01`, armed `0x21`, a capture
//! extracted `0x03`, scroll mode `0x14`, and a chain of states that each
//! take one setting, from `0x0f` (twice) to `0x13` and back to idle.
//! [`move_to`] walks them, answering each state with what it wants.

use crate::core::{Error, Result};
use crate::parport::{C1284_NAUTOFD, C1284_NINIT, C1284_NSELECTIN, C1284_NSTROBE};
use std::time::{Duration, Instant};

/// What the protocol needs of a port: [`crate::parport::Parport`], or a
/// test's fake.
pub trait Port: Send {
    /// `ieee1284_read_data`.
    fn read_data(&mut self) -> Result<u8>;
    /// `ieee1284_write_data`.
    fn write_data(&mut self, value: u8) -> Result<()>;
    /// `ieee1284_write_control`.
    fn write_control(&mut self, control: u8) -> Result<()>;
    /// `ieee1284_data_dir`.
    fn data_dir(&mut self, reverse: bool) -> Result<()>;
    /// Whether the port went away.
    fn gone(&self) -> bool;
    /// `ieee1284_release`.
    fn release(&mut self) {}
}

impl Port for crate::parport::Parport {
    fn read_data(&mut self) -> Result<u8> {
        crate::parport::Parport::read_data(self)
    }

    fn write_data(&mut self, value: u8) -> Result<()> {
        crate::parport::Parport::write_data(self, value)
    }

    fn write_control(&mut self, control: u8) -> Result<()> {
        crate::parport::Parport::write_control(self, control)
    }

    fn data_dir(&mut self, reverse: bool) -> Result<()> {
        crate::parport::Parport::data_dir(self, reverse)
    }

    fn gone(&self) -> bool {
        crate::parport::Parport::gone(self)
    }

    fn release(&mut self) {
        crate::parport::Parport::release(self);
    }
}

/// `MAX_RETRIES`: readouts with a bad signature before a frame gives up.
pub const MAX_RETRIES: u8 = 4;
/// The scope's id byte.
pub const ID: u8 = 0x55;
/// A readout's signature after its samples.
pub const SIGNATURE: [u8; 3] = [0x01, 0xfe, 0x80];
/// The samples a readout holds.
pub const READOUT: usize = 1000;
/// `move_to`'s tries.
pub const MOVE_TRIES: u32 = 40;
/// `move_to`'s wait for an answer.
pub const MOVE_WAIT: Duration = Duration::from_millis(100);
/// `config_commit`'s and the readout loop's wait.
pub const STATE_WAIT: Duration = Duration::from_millis(20);
/// The trigger poll's wait.
pub const POLL_WAIT: Duration = Duration::from_micros(250);
/// The wait after asking for the next subset, before its state is read.
pub const EXTRACT_WAIT: Duration = Duration::from_micros(1700);

/// The states [`move_to`] and the capture name.
pub mod state {
    /// Idle.
    pub const IDLE: u8 = 0x01;
    /// A capture extracted for readout.
    pub const EXTRACTED: u8 = 0x03;
    /// Scroll mode.
    pub const SCROLL: u8 = 0x14;
    /// Armed.
    pub const ARMED: u8 = 0x21;
}

/// One readout step, `readout_steps[]`: the samples it reads, the first
/// subset's step size, the shift of the second, and whether the two ADCs
/// interleave. Must stay in step with [`super::BUFFERSIZES`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Step {
    /// Samples read, after skipping the rest of the 1000.
    pub num: u16,
    /// The first subset's step size.
    pub step1: u8,
    /// The second subset's offset parameter.
    pub shift: u8,
    /// Both ADCs, each read in turn.
    pub interleave: bool,
}

const fn step(num: u16, step1: u8, shift: u8, interleave: bool) -> Step {
    Step { num, step1, shift, interleave }
}

/// `readout_steps[]`.
pub const STEPS: [Step; 15] = [
    step(1000, 1, 100, false),
    step(500, 100, 2, false),
    step(500, 100, 3, false),
    step(500, 100, 4, false),
    step(500, 100, 5, false),
    step(500, 100, 6, false),
    step(500, 100, 7, false),
    step(500, 100, 8, false),
    step(500, 100, 9, false),
    step(499, 212, 41, true),
    step(500, 157, 56, true),
    step(500, 247, 36, true),
    step(500, 232, 180, true),
    step(500, 230, 182, true),
    step(120, 212, 43, true),
];

/// The settings [`move_to`] hands the chain.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Settings {
    /// The enabled channel, 1 or 2, or 0 for none.
    pub channel: u8,
    /// Index into the samplerates.
    pub rate: u8,
    /// Each channel's byte: vdiv, coupling, and on the first the trigger
    /// source.
    pub cctl: [u8; 2],
    /// The trigger slope, 0 falling.
    pub edge: u8,
    /// The trigger level.
    pub tlevel: u8,
    /// Position, offset and gain calibration, per channel.
    pub pos: [u8; 2],
    /// Offset calibration.
    pub offset: [u8; 2],
    /// Gain calibration.
    pub gain: [u8; 2],
    /// The readout step the chain sets up.
    pub step: usize,
    /// In an interleaved step, the second ADC's turn.
    pub adc2: bool,
}

impl Default for Settings {
    /// The scan's: 100 MHz, 1 V/div, DC, triggered on A.
    fn default() -> Settings {
        Settings {
            channel: 0,
            rate: 0,
            cctl: [0x31, 0x31],
            edge: 0,
            tlevel: 0x80,
            pos: [0x80, 0x80],
            offset: [0x80, 0x80],
            gain: [0x80, 0x80],
            step: 0,
            adc2: false,
        }
    }
}

/// `hung_chang_dso_2100_reset_port`.
pub fn reset_port(port: &mut dyn Port) -> Result<()> {
    port.write_control(C1284_NSTROBE | C1284_NAUTOFD | C1284_NSELECTIN)?;
    port.data_dir(false)
}

/// `hung_chang_dso_2100_check_id`: whether a scope answers. The port is
/// reset either way.
pub fn check_id(port: &mut dyn Port) -> Result<bool> {
    let read = (|| {
        port.data_dir(true)?;
        port.write_control(C1284_NSTROBE | C1284_NAUTOFD | C1284_NSELECTIN)?;
        port.write_control(C1284_NAUTOFD | C1284_NSELECTIN)?;
        port.read_data()
    })();
    reset_port(port)?;
    Ok(read? == ID)
}

/// `hung_chang_dso_2100_write_mbox`.
pub fn write_mbox(port: &mut dyn Port, value: u8) -> Result<()> {
    log::debug!("mbox <= {value:X}");
    port.write_control(C1284_NSTROBE | C1284_NINIT | C1284_NSELECTIN)?;
    port.data_dir(false)?;
    port.write_data(value)?;
    port.write_control(C1284_NINIT | C1284_NSELECTIN)?;
    port.write_control(C1284_NSTROBE | C1284_NINIT | C1284_NSELECTIN)?;
    port.data_dir(true)?;
    port.write_control(C1284_NSTROBE | C1284_NAUTOFD | C1284_NINIT | C1284_NSELECTIN)
}

/// `hung_chang_dso_2100_read_mbox`: the state, or 0 when nothing came
/// within `timeout`. As in C, the clock starts at the first read that
/// gives 0 and is looked at after each later one, so an answer at the
/// second read always counts.
pub fn read_mbox(port: &mut dyn Port, timeout: Duration) -> Result<u8> {
    port.write_control(C1284_NSTROBE | C1284_NSELECTIN)?;
    port.write_control(C1284_NSELECTIN)?;
    let mut timer: Option<Instant> = None;
    let val = loop {
        if port.read_data()? != 0 {
            // The first may be unstable.
            break port.read_data()?;
        }
        match timer {
            None => timer = Some(Instant::now()),
            Some(t) if t.elapsed() > timeout => break 0,
            Some(_) => {}
        }
    };
    port.write_control(C1284_NSTROBE | C1284_NSELECTIN)?;
    port.write_control(C1284_NSTROBE | C1284_NAUTOFD | C1284_NINIT | C1284_NSELECTIN)?;
    log::debug!("mbox == {val:X}");
    Ok(val)
}

/// Why [`move_to`] did not get there.
#[derive(Debug)]
pub enum MoveError {
    /// A state no firmware state has (`SR_ERR_DATA`).
    Data(u8),
    /// 40 answers and not there (`SR_ERR_TIMEOUT`).
    Timeout(u8),
    /// The port failed or went away.
    Port(Error),
}

impl From<Error> for MoveError {
    fn from(e: Error) -> MoveError {
        MoveError::Port(e)
    }
}

impl MoveError {
    /// As an error of the capture or the open, naming the state wanted.
    pub fn into_error(self, target: u8) -> Error {
        match self {
            MoveError::Data(s) => Error::device(format!(
                "The DSO-2100 answered state 0x{s:02X}, which it has none of, on the way to 0x{target:02X}."
            )),
            MoveError::Timeout(0) => Error::device(format!(
                "The DSO-2100 stopped answering on the way to state 0x{target:02X}."
            )),
            MoveError::Timeout(s) => Error::device(format!(
                "The DSO-2100 did not reach state 0x{target:02X} in {MOVE_TRIES} steps (last 0x{s:02X})."
            )),
            MoveError::Port(e) => e,
        }
    }
}

/// What the chain's state `c` wants, as `move_to` answers it.
fn answer(c: u8, target: u8, s: &Settings, tries_left: u32) -> Option<u8> {
    let step = &STEPS[s.step];
    Some(match c {
        // A write the microcontroller did not expect: 0xff and 4 in turn
        // help in every state, the comment says.
        0x00 => {
            if tries_left & 1 == 1 {
                0xff
            } else {
                0x04
            }
        }
        0x01 => match target {
            state::ARMED => 2,
            state::EXTRACTED => 3,
            _ => 4,
        },
        0x03 => 4,
        0x05 => s.rate + 1,
        0x06 => s.cctl[0],
        0x07 => s.cctl[1],
        // The second subset's step, 1.
        0x08 => 2,
        0x09 => step.step1 + 1,
        0x0a => step.shift + 1,
        0x0b => s.edge + 1,
        0x0c => s.pos[0],
        0x0d => s.pos[1],
        0x0e => s.tlevel,
        0x0f => {
            if s.channel == 0 {
                1
            } else if step.interleave {
                if s.adc2 { 2 } else { 1 }
            } else {
                s.channel
            }
        }
        0x10 => s.offset[0],
        0x11 => s.gain[0],
        0x12 => s.offset[1],
        0x13 => s.gain[1],
        0x14 | 0x21 => 0xff,
        _ => return None,
    })
}

/// `hung_chang_dso_2100_move_to`: answers states until the scope is in
/// `target`. A mailbox that stays silent is looked at for the port going
/// away, which C does not do: there each of the 40 tries waits out its
/// 100 ms on a port whose reads now give 0.
pub fn move_to(
    port: &mut dyn Port,
    target: u8,
    s: &Settings,
) -> std::result::Result<(), MoveError> {
    let mut last = 0;
    for i in 0..MOVE_TRIES {
        let tries_left = MOVE_TRIES - 1 - i;
        let c = read_mbox(port, MOVE_WAIT)?;
        if c == target {
            return Ok(());
        }
        if c == 0 && port.gone() {
            return Err(MoveError::Port(Error::Disconnected));
        }
        last = c;
        let Some(next) = answer(c, target, s, tries_left) else {
            return Err(MoveError::Data(c));
        };
        write_mbox(port, next)?;
    }
    Err(MoveError::Timeout(last))
}

/// `skip_samples`.
pub fn skip_samples(port: &mut dyn Port, ctrl: u8, num: usize) -> Result<()> {
    for _ in 0..num {
        port.write_control(ctrl & !C1284_NSTROBE)?;
        port.write_control(ctrl)?;
    }
    Ok(())
}

/// `read_samples`: `num` samples into `buf` from `at`, `stride` apart.
pub fn read_samples(
    port: &mut dyn Port,
    ctrl: u8,
    buf: &mut [u8],
    at: usize,
    num: usize,
    stride: usize,
) -> Result<()> {
    for i in 0..num {
        port.write_control(ctrl & !C1284_NSTROBE)?;
        buf[at + i * stride] = port.read_data()?;
        port.write_control(ctrl)?;
    }
    Ok(())
}

/// The control bits of a readout of `s`'s step.
pub fn readout_control(s: &Settings) -> u8 {
    let interleave = STEPS[s.step].interleave;
    if (interleave && s.adc2) || (!interleave && s.channel == 2) {
        C1284_NSTROBE | C1284_NAUTOFD
    } else {
        C1284_NSTROBE
    }
}

/// One readout of `s`'s step into `buf`: its samples, then whether the
/// signature came.
pub fn read_readout(port: &mut dyn Port, s: &Settings, buf: &mut [u8; READOUT]) -> Result<bool> {
    let step = STEPS[s.step];
    let ctrl = readout_control(s);
    port.write_control(ctrl)?;
    let num = usize::from(step.num);
    if num < READOUT {
        skip_samples(port, ctrl, READOUT - num)?;
    }
    let stride = if step.interleave { 2 } else { 1 };
    read_samples(port, ctrl, buf, usize::from(s.adc2), num, stride)?;
    let mut sig = [0u8; 3];
    read_samples(port, ctrl, &mut sig, 0, 3, 1)?;
    Ok(sig == SIGNATURE)
}

/// Volts per count of the enabled channel, as `dev_acquisition_start`
/// computes it, in single precision.
pub fn factor(probe: u64, cctl: u8) -> f32 {
    const RES: [f32; 4] = [0.5, 1.0, 2.0, 5.0];
    const RELAYS: [f32; 4] = [100.0, 10.0, 10.0, 1.0];
    let mut f = probe as f32 / 32.0;
    f *= RES[usize::from(cctl & 0x03)];
    f /= RELAYS[usize::from((cctl >> 4) & 0x03)];
    f
}

/// `push_samples`'s digits for a factor.
pub fn digits(factor: f32) -> i32 {
    let log = factor.log10();
    -(log as i32) + i32::from(log < 0.0)
}

/// Samples in volts.
pub fn to_volts(buf: &[u8], factor: f32) -> Vec<f32> {
    buf.iter().map(|&b| (i32::from(b) - 0x80) as f32 * factor).collect()
}
