// Counterpart in sigrok:
//   libsigrok/src/hardware/conrad-digi-35-cpu/api.c,
//   libsigrok/src/hardware/conrad-digi-35-cpu/protocol.c,
//   libsigrok/src/hardware/conrad-digi-35-cpu/protocol.h,
//   Copyright (C) 2014 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Frank Stettner, Martin Ling, Soeren Apel, Aurelien Jacobs,
//   Daniel Elstner, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire of the Conrad DIGI 35 CPU: ASCII commands at 9600/8n1, one way only.
//!
//! `Vxxx` CR sets the voltage in 100 mV and `Cxxx` CR the current in 10 mA. `V900`
//! turns over-current protection on, and `V901` turns it off, which leaves constant
//! current. The device sends nothing, ever, so a command that got lost looks the
//! same as one that worked. It is said to swallow a command that follows another
//! within 50 ms, so each one is followed by a pause. A command cut short stays in
//! its buffer and runs into the next: `V0` then `C100` CR is `V0C100` CR. A bare CR
//! wouldn't clear that, as `V0` CR could be taken for a command.
//!
//! sigrok's wiki lists more (`L`, `E`, `U`, `D`, `V4xx` to `V803`, `V902`: keyboard
//! lock, steps, timers, the baud rate). C doesn't send them, and neither does this.

#![forbid(unsafe_code)]

use crate::core::Error;
use crate::serial::{Params, Port, serial_timeout};
use std::time::Duration;

/// The line, unless `serialcomm=` says otherwise.
pub const SERIALCOMM: &str = "9600/8n1";
/// What follows every command once the tty's queue is empty, for the device's
/// sake, where C's starts at the write. An adapter's own FIFO can still hold the
/// bytes then, and this is enough only at 9600: at 300 baud 5 bytes take about
/// 167 ms, so 50 ms may leave no gap, as in C.
pub const PAUSE: Duration = Duration::from_millis(50);
/// How long a frame may take to leave the tty's queue, beyond its time on the line.
pub const DRAIN_BOUND: Duration = Duration::from_secs(1);
/// The top of the voltage, in V.
pub const MAX_VOLTAGE: f64 = 35.0;
/// The step of the voltage, in V.
pub const VOLTAGE_STEP: f64 = 0.1;
/// The top of the current, in A.
pub const MAX_CURRENT: f64 = 2.55;
/// The step of the current, in A.
pub const CURRENT_STEP: f64 = 0.01;
/// A command with its CR.
const FRAME_LEN: u8 = 5;

/// One command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    /// A voltage from [`Command::voltage`].
    Voltage(u16),
    /// A current from [`Command::current`].
    Current(u16),
    /// Over-current protection on, or off for constant current.
    Protection(bool),
}

impl Command {
    /// A voltage in V, from 0 to [`MAX_VOLTAGE`]. It takes the double that C's
    /// `(int)(v * 10 + 0.5)` takes, so a value halfway between two steps falls
    /// where that arithmetic has it.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn voltage(volts: f64) -> Command {
        Command::Voltage((volts * 10.0 + 0.5) as u16)
    }

    /// A current in A, from 0 to [`MAX_CURRENT`], as [`Command::voltage`] has it.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn current(amps: f64) -> Command {
        Command::Current((amps * 100.0 + 0.5) as u16)
    }

    fn letter(self) -> u8 {
        match self {
            Command::Voltage(_) | Command::Protection(_) => b'V',
            Command::Current(_) => b'C',
        }
    }

    fn param(self) -> u16 {
        match self {
            Command::Voltage(n) | Command::Current(n) => n,
            Command::Protection(true) => 900,
            Command::Protection(false) => 901,
        }
    }

    /// The command as text, without its CR.
    pub fn shown(self) -> String {
        let frame = self.frame();
        String::from_utf8_lossy(&frame[..frame.len() - 1]).into_owned()
    }

    /// The bytes on the wire: three digits and a CR. Nothing above 999 reaches
    /// this from [`Command::voltage`] or [`Command::current`] in their ranges.
    pub fn frame(self) -> [u8; FRAME_LEN as usize] {
        let n = self.param();
        let digit = |place: u16| b'0' + u8::try_from(n / place % 10).expect("under 10");
        [self.letter(), digit(100), digit(10), digit(1), b'\r']
    }
}

/// Why a command didn't get out. None of these is seen by the device as anything:
/// it has no way to say.
#[derive(Debug)]
pub enum Failure {
    /// The port took none of it within its time. The frame is not on its way.
    Unsent(Duration),
    /// The port took this many bytes of it within its time.
    Cut(usize),
    /// The port failed, during the write or the wait.
    Port(Error),
    /// It was taken, and is still in the queue after this long.
    Queued(Duration),
}

impl Failure {
    /// Whether the device may hold part of a command, or get one late, so that
    /// what is sent next can't be trusted to mean what it says. C carries on.
    pub fn leaves_state_unknown(&self) -> bool {
        !matches!(self, Failure::Unsent(_))
    }

    /// C's line, for the failures that are a write's own.
    pub fn write_error_line(&self, command: Command) -> Option<String> {
        matches!(self, Failure::Unsent(_) | Failure::Cut(_))
            .then(|| format!("Write error for cmd={}", char::from(command.letter())))
    }

    /// What the user gets, with its log lines. C has `Write error for cmd=V` and
    /// its generic error for all of these, and returns success for
    /// [`Failure::Queued`]; this doesn't. Only the first two are errors of the
    /// write itself. A hung up tty fails in the write or the wait, as the hangup
    /// falls, so [`Failure::Port`] has one wording for both.
    pub fn report(self, command: Command) -> Error {
        let frame = command.shown();
        if let Some(line) = self.write_error_line(command) {
            log::error!("{line}");
        }
        if self.leaves_state_unknown() {
            log::warn!("The supply's state is unknown: check its front panel.");
        }
        Error::device(match self {
            Failure::Unsent(t) => {
                format!("{frame} did not go out: the port took none of it in {} ms", t.as_millis())
            }
            Failure::Cut(n) => format!(
                "{frame} was cut short after {n} of {FRAME_LEN} bytes, which stay in the \
                 supply's buffer and run into the next command"
            ),
            Failure::Port(e) => format!("the port failed sending {frame}: {e}"),
            Failure::Queued(t) => format!(
                "{frame} is still in the port's queue after {} ms, and may come late or never",
                t.as_millis()
            ),
        })
    }
}

/// `send_msg1`: writes `command` within the line's time, waits for it to leave the
/// tty's queue and pauses. There is no answer to wait for, so `Ok` says only that
/// the bytes left the tty. A failure gets no pause, as in C: an unsent frame left
/// nothing to keep apart from what follows, and after the others the device refuses
/// to send more.
///
/// # Errors
///
/// [`Failure`]. C doesn't wait for the queue, so it returns success on a frame that
/// is stuck there and fails only when the queue is full.
pub fn send(
    port: &mut dyn Port,
    params: &Params,
    command: Command,
    pause: &mut dyn FnMut(Duration),
) -> Result<(), Failure> {
    let frame = command.frame();
    log::trace!("send_msg1(): {}\\r", command.shown());
    let line_time = serial_timeout(params, u32::from(FRAME_LEN));
    match port.write_timeout(&frame, line_time) {
        Ok(n) if n == frame.len() => {}
        Ok(0) => return Err(Failure::Unsent(line_time)),
        Ok(n) => return Err(Failure::Cut(n)),
        Err(e) => return Err(Failure::Port(e)),
    }
    let bound = line_time + DRAIN_BOUND;
    match port.drain(bound) {
        Ok(()) => {}
        Err(Error::Timeout) => return Err(Failure::Queued(bound)),
        Err(e) => return Err(Failure::Port(e)),
    }
    pause(PAUSE);
    Ok(())
}
